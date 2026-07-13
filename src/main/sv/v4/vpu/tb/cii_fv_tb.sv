// ---------------------------------------------------------------------------
// cii_fv_tb -- functional smoke test for the CII VPU coprocessor (Track C).
//
// Plays the HOST on the CII interface (through the real tt_cii credit relay):
//   1. issues one vadd.vv (vd=v3, vs2=v2, vs1=v1, unmasked, SEW=32, LMUL=1, vl=8)
//      with the vtype config carried in the extended issue packet,
//   2. serves the coprocessor's source-operand requests with known VS1/VS2 data,
//   3. captures the writeback and checks it equals the per-lane sum.
//
// This exercises C1 (issue) -> C3 (operand pull) -> tt_vec compute -> C4
// (writeback). It is the first stimulus that moves signals through the loop and
// is expected to surface the C3/C4 skeleton + gating gaps.
// ---------------------------------------------------------------------------
`include "tt_cii_caracal_pkg.svh"

module cii_fv_tb import tt_cii_caracal_pkg::*; ;
  localparam int VLEN = 256;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // Host<->relay and relay<->coproc interfaces (Caracal-typed).
  `define CII_PARAMS \
    .INSTR_T(cii_caracal_instr_t), .SRCID_T(cii_caracal_srcid_t), \
    .SRC_OFFSET_T(cii_caracal_offset_t), .DST_OFFSET_T(cii_caracal_offset_t), \
    .TAG_T(cii_caracal_tag_t), .SRC_DATA_T(cii_caracal_data_t), \
    .DST_DATA_T(cii_caracal_data_t), .WB_STATUS_T(cii_caracal_wb_status_t), \
    .SRC_FRWD_HINT_T(cii_caracal_frwd_hint_t), \
    .CII_NUM_INST_ISSUE(CII_NUM_INST_ISSUE), .CII_NUM_SRC_REQ(CII_NUM_SRC_REQ), \
    .CII_NUM_SRC_DAT_RSP(CII_NUM_SRC_DAT_RSP), .CII_NUM_DST_WB(CII_NUM_DST_WB), \
    .CII_MISA(CII_MISA)

  tt_cii_interface #(`CII_PARAMS) ifh ();   // host  <-> relay
  tt_cii_interface #(`CII_PARAMS) ifc ();   // relay <-> coproc

  tt_cii #(.CII_N_ISS_CREDITS(16), .CII_N_REQ_CREDITS(16),
           .CII_N_DAT_CREDITS(16), .CII_N_WB_CREDITS(16))
    relay (.clk(clk), .rst_n(rst_n), .cii_host(ifc), .cii_coproc(ifh));

  logic dbg_v; logic [VLEN*8-1:0] dbg_d; logic [7:0] dbg_m;
  tt_vpu_cii_wrapper_top #(.VLEN(VLEN)) dut (
    .clk(clk), .reset_n(rst_n), .cii_intf(ifc),
    .debug_wb_vec_valid(dbg_v), .debug_wb_vec_wdata(dbg_d), .debug_wb_vec_wmask(dbg_m));

  // ---- operand data (8 x 32b lanes) ----
  logic [VLEN-1:0] vs1_data, vs2_data, exp_data;
  initial begin
    for (int i = 0; i < 8; i++) begin
      vs1_data[i*32 +: 32] = i + 1;          // 1,2,...,8
      vs2_data[i*32 +: 32] = (i + 1) * 10;   // 10,20,...,80
      exp_data[i*32 +: 32] = (i + 1) * 11;   // 11,22,...,88
    end
  end

  // vadd.vv v3, v2, v1 (unmasked): {f6=0,vm=1,vs2=2,vs1=1,f3=0(OPIVV),vd=3,op=0x57}
  wire [31:0] vadd_insn = {6'b000000, 1'b1, 5'd2, 5'd1, 3'b000, 5'd3, 7'b1010111};

  cii_caracal_instr_t iss_instr;
  always_comb begin
    iss_instr           = '0;
    iss_instr.insn      = vadd_insn;
    iss_instr.vtype.vsew  = 3'd2;   // SEW=32
    iss_instr.vtype.vlmul = 3'd0;   // LMUL=1
    iss_instr.vtype.vta   = 1'b0;
    iss_instr.vtype.vma   = 1'b0;
    iss_instr.vl        = 9'd8;
    iss_instr.vstart    = 9'd0;
    iss_instr.vxrm      = 2'd0;
    iss_instr.frm       = 3'd0;
  end

  // ---- HOST: issue channel (sender). Push one beat, then idle. ----
  logic issued;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      issued           <= 1'b0;
      ifh.iss_valid    <= 1'b0;
      ifh.iss_data[0]  <= '0;
    end else begin
      ifh.iss_valid <= 1'b0;
      if (!issued) begin
        ifh.iss_valid          <= 1'b1;
        ifh.iss_data[0].tag             <= 4'd1;
        ifh.iss_data[0].instr           <= iss_instr;
        ifh.iss_data[0].instr_src_valid <= '0;
        issued <= 1'b1;
      end
    end
  end

  // ---- HOST: source-request receiver + data responder ----
  // Receiver of req (assert req_credit to pop; req_valid arrives registered).
  // For each popped request, drive the matching operand back on dat next cycle.
  assign ifh.req_credit = 1'b1;    // always ready to accept a request
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ifh.dat_valid   <= 1'b0;
      ifh.dat_data[0] <= '0;
      ifh.dat_data[1] <= '0;
    end else begin
      ifh.dat_valid <= 1'b0;
      if (ifh.req_valid) begin
        // respond to lane 0 (coproc drives lane 1 = NONE in the skeleton).
        case (ifh.req_data[0].rsp_src_id)
          CII_SRC_VS1: ifh.dat_data[0].rsp_dat <= vs1_data;
          CII_SRC_VS2: ifh.dat_data[0].rsp_dat <= vs2_data;
          default:     ifh.dat_data[0].rsp_dat <= '0;
        endcase
        ifh.dat_valid <= 1'b1;
      end
    end
  end

  // ---- HOST: writeback receiver + self-check ----
  assign ifh.wb_credit = 1'b1;     // always ready to accept a writeback
  int   errors;
  logic got_wb;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      errors <= 0;
      got_wb <= 1'b0;
    end else if (ifh.wb_valid) begin
      got_wb <= 1'b1;
      $display("[FV] writeback: tag=%0d dst_off=%0d wr_en=%b data=%h",
               ifh.wb_data[0].inst_tag, ifh.wb_data[0].wb_dst_offset,
               ifh.wb_data[0].wb_wr_en, ifh.wb_data[0].wb_data);
      if (ifh.wb_data[0].wb_data !== exp_data) begin
        errors <= errors + 1;
        $display("[FV] MISMATCH: got %h expected %h", ifh.wb_data[0].wb_data, exp_data);
      end else begin
        $display("[FV] RESULT OK");
      end
    end
  end

  // ---- cycle monitor: find where the flow stalls ----
  initial begin
    @(posedge rst_n);
    for (int c = 0; c < 90; c++) begin
      @(posedge clk);
      $display("[MON] c=%0d iss_v=%b iss_cr=%b rd_rtr=%b rd_val=%b iss_st=%0d | id_vex_rts=%b pf_idx=%0d rcv=%0d req_v=%b dat_v=%b vex_rtr=%b | lqvld=%b%b%b%b",
        c, ifh.iss_valid, ifh.iss_credit, dut.ocelot_read_req, dut.read_valid, dut.iss_state,
        dut.id_vex_rts, dut.pf_idx, dut.rcv_cnt, ifh.req_valid, ifh.dat_valid, dut.vex_id_rtr,
        dut.vex_mem_lqvld_1c, dut.vex_mem_lqvld_2c, dut.vex_mem_lqvld_3c, dut.vex_mem_lqvld_div);
      $display("[MON2] c=%0d ldqid=%0d res_lqid=%0d wb_fire=%b", c, dut.id_vec_autogen.ldqid, dut.res_lqid, dut.wb_fire);
    end
  end

  // ---- reset + timeout ----
  initial begin
    rst_n = 1'b0; repeat (5) @(posedge clk); rst_n = 1'b1;
    repeat (400) @(posedge clk);
    if (!got_wb) $display("[FV] TIMEOUT: no writeback observed");
    if (got_wb && errors == 0) $display("[FV] *** PASSED ***");
    else                       $display("[FV] *** FAILED *** (errors=%0d, got_wb=%b)", errors, got_wb);
    $finish;
  end
endmodule

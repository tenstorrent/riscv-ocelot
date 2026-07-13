// ---------------------------------------------------------------------------
// cii_fv_tb -- functional smoke test for the CII VPU coprocessor (Track C).
//
// Plays the HOST on the CII interface (through the real tt_cii credit relay):
//   1. issues one vadd.vv at LMUL=2 (vd=v4, vs2=v6, vs1=v2, unmasked, SEW=32,
//      vl=16) with the vtype config carried in the extended issue packet,
//   2. serves the coprocessor's per-member source-operand requests with known
//      per-member VS1/VS2 data (keyed by rsp_src_id + rsp_src_offset),
//   3. captures the TWO member writeback beats and checks each equals the
//      per-lane sum for its member, with `last` set only on the final member.
//
// This exercises the member walk: C1 issue -> C3 per-member operand pull ->
// tt_vec compute (LMUL replay) -> C4 per-member writeback.
// ---------------------------------------------------------------------------
`include "tt_cii_caracal_pkg.svh"

module cii_fv_tb import tt_cii_caracal_pkg::*; ;
  localparam int VLEN = 256;
  // LMUL_LOG2 selects the register-group size: 0/1/2/3 => LMUL 1/2/4/8 => NM
  // members. Override at runtime with +LMUL_LOG2=<n>.
  int LMUL_LOG2 = 1;
  int NM = 2;                    // members = 1<<LMUL_LOG2 (set in initial below)

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

  // ---- per-member operand data (8 x 32b lanes per member) ----
  // NM-aligned register groups: vd = NM, vs1 = 2*NM, vs2 = 3*NM (max 4*NM-1 <= 31).
  logic [VLEN-1:0] vs1_mem [0:7];
  logic [VLEN-1:0] vs2_mem [0:7];
  logic [VLEN-1:0] exp_mem [0:7];
  logic [4:0] r_vd, r_vs1, r_vs2;
  logic [2:0] r_vlmul;
  logic [8:0] r_vl;
  initial begin
    void'($value$plusargs("LMUL_LOG2=%d", LMUL_LOG2));
    NM = 1 << LMUL_LOG2;
    for (int mm = 0; mm < NM; mm++)
      for (int i = 0; i < 8; i++) begin
        automatic int e = mm*8 + i + 1;         // 1..(NM*8) across the group
        vs1_mem[mm][i*32 +: 32] = e;
        vs2_mem[mm][i*32 +: 32] = e * 10;
        exp_mem[mm][i*32 +: 32] = e * 11;
      end
    r_vd    = NM[4:0];
    r_vs1   = (2*NM);
    r_vs2   = (3*NM);
    r_vlmul = LMUL_LOG2[2:0];
    r_vl    = NM*8;
  end

  // vadd.vv vd, vs2, vs1 (unmasked): {f6=0,vm=1,vs2,vs1,f3=0(OPIVV),vd,op=0x57}
  wire [31:0] vadd_insn = {6'b000000, 1'b1, r_vs2, r_vs1, 3'b000, r_vd, 7'b1010111};

  cii_caracal_instr_t iss_instr;
  always_comb begin
    iss_instr           = '0;
    iss_instr.insn      = vadd_insn;
    iss_instr.vtype.vsew  = 3'd2;   // SEW=32
    iss_instr.vtype.vlmul = r_vlmul;
    iss_instr.vtype.vta   = 1'b0;
    iss_instr.vtype.vma   = 1'b0;
    iss_instr.vl        = r_vl;
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
  // For each popped request, drive the matching member's operand back on dat.
  assign ifh.req_credit = 1'b1;    // always ready to accept a request
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ifh.dat_valid   <= 1'b0;
      ifh.dat_data[0] <= '0;
      ifh.dat_data[1] <= '0;
    end else begin
      ifh.dat_valid <= 1'b0;
      if (ifh.req_valid) begin
        // respond to lane 0 by {source, member}; coproc drives lane 1 = NONE.
        case (ifh.req_data[0].rsp_src_id)
          CII_SRC_VS1: ifh.dat_data[0].rsp_dat <= vs1_mem[ifh.req_data[0].rsp_src_offset];
          CII_SRC_VS2: ifh.dat_data[0].rsp_dat <= vs2_mem[ifh.req_data[0].rsp_src_offset];
          default:     ifh.dat_data[0].rsp_dat <= '0;
        endcase
        ifh.dat_valid <= 1'b1;
      end
    end
  end

  // ---- HOST: writeback receiver + self-check (one beat per member) ----
  assign ifh.wb_credit = 1'b1;     // always ready to accept a writeback
  int   errors;
  logic seen [0:7];
  logic saw_last;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      errors   <= 0;
      saw_last <= 1'b0;
      for (int k=0;k<8;k++) seen[k] <= 1'b0;
    end else if (ifh.wb_valid) begin
      automatic int m = ifh.wb_data[0].wb_dst_offset;
      seen[m] <= 1'b1;
      if (ifh.wb_data[0].wb_fp_flags.last) saw_last <= 1'b1;
      $display("[FV] writeback: tag=%0d dst_off=%0d last=%b wr_en=%b data=%h",
               ifh.wb_data[0].inst_tag, ifh.wb_data[0].wb_dst_offset,
               ifh.wb_data[0].wb_fp_flags.last, ifh.wb_data[0].wb_wr_en, ifh.wb_data[0].wb_data);
      if (ifh.wb_data[0].wb_data !== exp_mem[m]) begin
        errors <= errors + 1;
        $display("[FV] MISMATCH m=%0d: got %h expected %h", m, ifh.wb_data[0].wb_data, exp_mem[m]);
      end else begin
        $display("[FV] RESULT OK m=%0d", m);
      end
    end
  end

  // ---- cycle monitor: find where the flow stalls ----
  initial begin
    @(posedge rst_n);
    for (int c = 0; c < 90; c++) begin
      @(posedge clk);
      $display("[MON] c=%0d iss_v=%b iss_cr=%b rd_rtr=%b rd_val=%b iss_st=%0d | pf_src=%0d pf_mem=%0d rcv=%0d req_v=%b dat_v=%b vex_rtr=%b | lqvld=%b%b%b%b",
        c, ifh.iss_valid, ifh.iss_credit, dut.ocelot_read_req, dut.read_valid, dut.iss_state,
        dut.pf_src, dut.pf_mem, dut.rcv_cnt, ifh.req_valid, ifh.dat_valid, dut.vex_id_rtr,
        dut.vex_mem_lqvld_1c, dut.vex_mem_lqvld_2c, dut.vex_mem_lqvld_3c, dut.vex_mem_lqvld_div);
    end
  end

  // ---- reset + timeout ----
  int wb_beats;
  always_comb begin
    wb_beats = 0;
    for (int k=0;k<NM;k++) if (seen[k]) wb_beats++;
  end
  initial begin
    rst_n = 1'b0; repeat (5) @(posedge clk); rst_n = 1'b1;
    repeat (400) @(posedge clk);
    if (wb_beats != NM) $display("[FV] TIMEOUT/MISSING: %0d of %0d member writebacks seen", wb_beats, NM);
    if (wb_beats == NM && errors == 0 && saw_last)
      $display("[FV] *** PASSED *** (%0d members)", NM);
    else
      $display("[FV] *** FAILED *** (errors=%0d, wb_beats=%0d/%0d, saw_last=%b)",
               errors, wb_beats, NM, saw_last);
    $finish;
  end
endmodule

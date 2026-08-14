// ---------------------------------------------------------------------------
// cii_fv_tb -- functional smoke test for the CII VPU coprocessor (Track C).
//
// Plays the HOST on the CII interface (through the real tt_cii credit relay):
//   1. issues one vector op (selectable) with the vtype config in the extended
//      issue packet,
//   2. serves the coprocessor's per-member source-operand requests with known
//      per-member data (keyed by rsp_src_id + rsp_src_offset),
//   3. captures the per-member writeback beats and checks each equals the
//      expected result for its dest member, with `last` on the final member.
//
// Exercises the member walk incl. widening/narrowing (dst EMUL != src EMUL):
//   +OP_SEL=0  vadd.vv    normal    SEW=32, dst NM = src NM
//   +OP_SEL=1  vwaddu.vv  widening  src SEW=16, dst SEW=32, dst NM = 2*src NM
//   +OP_SEL=2  vnsrl.wv   narrowing dst/vs1 SEW=16, vs2 wide SEW=32 (2*NM), dst NM
//   +LMUL_LOG2=<n>  source LMUL = 1<<n
// ---------------------------------------------------------------------------
`include "tt_cii_caracal_pkg.svh"

module cii_fv_tb import tt_cii_caracal_pkg::*; ;
  localparam int VLEN = 256;

  int OP_SEL    = 0;
  int LMUL_LOG2 = 1;
  int NM        = 2;    // source members = 1<<LMUL_LOG2
  int DST_NM    = 2;    // dest members (result beats)

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

  // ---- operand data + instruction (per member, keyed by src_id+offset) ----
  logic [VLEN-1:0] vs1_mem [0:15];
  logic [VLEN-1:0] vs2_mem [0:15];
  logic [VLEN-1:0] vs3_mem [0:15];   // old destination group (RMW / masked-undisturbed)
  logic [VLEN-1:0] vm_mem  [0:15];   // v0 mask register
  logic [VLEN-1:0] exp_mem [0:15];
  logic [63:0]     scalar_val;       // .vx/.vf scalar operand (SRC_SCALAR)
  logic [4:0] r_vd, r_vs1, r_vs2;
  logic [2:0] r_vlmul, r_funct3;
  logic [5:0] r_funct6;
  logic [2:0] r_vsew;
  logic       r_vm;                  // 1 = unmasked, 0 = masked by v0
  logic [8:0] r_vl;

  // pack element `val` (width w bits, w in {16,32}) at global element index
  // `idx` into arr. Part-select width must be constant, so split on w.
  task automatic pack(ref logic [VLEN-1:0] arr [0:15], input int idx, input int w, input longint val);
    if (w == 16) begin
      int per = VLEN/16; arr[idx/per][(idx%per)*16 +: 16] = val[15:0];
    end else if (w == 64) begin
      int per = VLEN/64; arr[idx/per][(idx%per)*64 +: 64] = val[63:0];
    end else begin
      int per = VLEN/32; arr[idx/per][(idx%per)*32 +: 32] = val[31:0];
    end
  endtask

  initial begin
    void'($value$plusargs("OP_SEL=%d",    OP_SEL));
    void'($value$plusargs("LMUL_LOG2=%d", LMUL_LOG2));
    NM = 1 << LMUL_LOG2;
    for (int k=0;k<16;k++) begin
      vs1_mem[k]='0; vs2_mem[k]='0; vs3_mem[k]='0; vm_mem[k]='0; exp_mem[k]='0;
    end
    scalar_val = '0; r_vm = 1'b1;
    case (OP_SEL)
      // ---- vadd.vv (normal, SEW=32) : dst NM = src NM -------------------
      1: begin  // vwaddu.vv widening: src SEW=16, dst SEW=32, dst NM = 2*NM
        r_vsew=3'd1; r_funct6=6'b110000; r_funct3=3'b010; // OPMVV
        r_vd=5'(4*NM); r_vs1=5'(NM); r_vs2=5'(2*NM); r_vlmul=LMUL_LOG2[2:0];
        r_vl=9'(NM*(VLEN/16)); DST_NM = 2*NM;
        for (int g=0; g<NM*(VLEN/16); g++) begin // narrow src elems (16b)
          pack(vs1_mem, g, 16, g+1);
          pack(vs2_mem, g, 16, (g+1)*10);
        end
        for (int g=0; g<NM*(VLEN/16); g++)        // wide dst elems (32b)
          pack(exp_mem, g, 32, (g+1)*11);
      end
      2: begin  // vnsrl.wv narrowing: dst/vs1 SEW=16, vs2 wide SEW=32 (2*NM)
        r_vsew=3'd1; r_funct6=6'b101100; r_funct3=3'b000; // OPIVV
        r_vd=5'(NM); r_vs1=5'(2*NM); r_vs2=5'(4*NM); r_vlmul=LMUL_LOG2[2:0];
        r_vl=9'(NM*(VLEN/16)); DST_NM = NM;
        for (int g=0; g<NM*(VLEN/16); g++) begin
          pack(vs1_mem, g, 16, 4);                // narrow shift amount = 4
          pack(vs2_mem, g, 32, (g+1) << 4);       // wide src (32b)
          pack(exp_mem, g, 16, g+1);              // narrow dst = vs2>>4
        end
      end
      3: begin  // vadd.vv MASKED (vm=0), SEW=32: inactive lanes keep old dest (vma=0)
        r_vsew=3'd2; r_funct6=6'b000000; r_funct3=3'b000; r_vm=1'b0; // OPIVV, masked
        r_vd=5'(NM); r_vs1=5'(2*NM); r_vs2=5'(3*NM); r_vlmul=LMUL_LOG2[2:0];
        r_vl=9'(NM*(VLEN/32)); DST_NM = NM;
        for (int g=0; g<NM*(VLEN/32); g++) begin
          automatic bit active = (g % 2 == 0);      // even elements active
          pack(vs1_mem, g, 32, g+1);
          pack(vs2_mem, g, 32, (g+1)*10);
          pack(vs3_mem, g, 32, 32'hD00 + g);        // old dest value
          vm_mem[0][g] = active;                     // v0: single reg, bit g = element g
          pack(exp_mem, g, 32, active ? (g+1)*11 : (32'hD00 + g));
        end
      end
      4: begin  // vadd.vx (scalar rs1), SEW=32: dst = vs2 + x
        r_vsew=3'd2; r_funct6=6'b000000; r_funct3=3'b100; // OPIVX
        r_vd=5'(NM); r_vs1=5'd5 /*rs1 idx (value delivered via SRC_SCALAR)*/;
        r_vs2=5'(3*NM); r_vlmul=LMUL_LOG2[2:0];
        r_vl=9'(NM*(VLEN/32)); DST_NM = NM; scalar_val = 64'd100;
        for (int g=0; g<NM*(VLEN/32); g++) begin
          pack(vs2_mem, g, 32, (g+1)*10);
          pack(exp_mem, g, 32, (g+1)*10 + 100);
        end
      end
      5: begin  // vadd.vv normal SEW=64 (matches the integrated ms12 cosim test)
        r_vsew=3'd3; r_funct6=6'b000000; r_funct3=3'b000; // OPIVV, e64
        r_vd=5'(NM); r_vs1=5'(2*NM); r_vs2=5'(3*NM); r_vlmul=LMUL_LOG2[2:0];
        r_vl=9'(NM*(VLEN/64)); DST_NM = NM;
        for (int g=0; g<NM*(VLEN/64); g++) begin
          pack(vs1_mem, g, 64, g+1);
          pack(vs2_mem, g, 64, (g+1)*10);
          pack(exp_mem, g, 64, (g+1)*11);
        end
      end
      default: begin // vadd.vv normal SEW=32
        r_vsew=3'd2; r_funct6=6'b000000; r_funct3=3'b000; // OPIVV
        r_vd=5'(NM); r_vs1=5'(2*NM); r_vs2=5'(3*NM); r_vlmul=LMUL_LOG2[2:0];
        r_vl=9'(NM*(VLEN/32)); DST_NM = NM;
        for (int g=0; g<NM*(VLEN/32); g++) begin
          pack(vs1_mem, g, 32, g+1);
          pack(vs2_mem, g, 32, (g+1)*10);
          pack(exp_mem, g, 32, (g+1)*11);
        end
      end
    endcase
  end

  // instruction word: {funct6, vm, vs2, vs1, funct3, vd, opcode=OP-V}
  wire [31:0] insn_w = {r_funct6, r_vm, r_vs2, r_vs1, r_funct3, r_vd, 7'b1010111};

  cii_caracal_instr_t iss_instr;
  always_comb begin
    iss_instr           = '0;
    iss_instr.insn      = insn_w;
    iss_instr.vtype.vsew  = r_vsew;
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
  assign ifh.req_credit = ifh.req_valid;  // updated tt_cii: return 1 credit per req consumed
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ifh.dat_valid   <= 1'b0;
      ifh.dat_data[0] <= '0;
      ifh.dat_data[1] <= '0;
    end else begin
      ifh.dat_valid <= 1'b0;
      if (ifh.req_valid) begin
        case (ifh.req_data[0].rsp_src_id)
          CII_SRC_VS1:    ifh.dat_data[0].rsp_dat <= vs1_mem[ifh.req_data[0].rsp_src_offset];
          CII_SRC_VS2:    ifh.dat_data[0].rsp_dat <= vs2_mem[ifh.req_data[0].rsp_src_offset];
          CII_SRC_VS3_VD:    ifh.dat_data[0].rsp_dat <= vs3_mem[ifh.req_data[0].rsp_src_offset];
          CII_SRC_VM:     ifh.dat_data[0].rsp_dat <= vm_mem [ifh.req_data[0].rsp_src_offset];
          CII_SRC_SCALAR: ifh.dat_data[0].rsp_dat <= {{(VLEN-64){1'b0}}, scalar_val};
          default:        ifh.dat_data[0].rsp_dat <= '0;
        endcase
        ifh.dat_valid <= 1'b1;
      end
    end
  end

  // ---- HOST: writeback receiver + self-check (one beat per dest member) ----
  assign ifh.wb_credit = ifh.wb_valid;   // updated tt_cii: return 1 credit per wb consumed
  int   errors;
  logic seen [0:15];
  logic saw_last;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      errors   <= 0;
      saw_last <= 1'b0;
      for (int k=0;k<16;k++) seen[k] <= 1'b0;
    end else if (ifh.wb_valid) begin
      automatic int m = ifh.wb_data[0].wb_dst_offset;
      if (seen[m]) $display("[FV] DUP dst_off=%0d (member written twice)", m);
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

  // ---- cycle monitor ----
  initial begin
    @(posedge rst_n);
    for (int c = 0; c < 120; c++) begin
      @(posedge clk);
      $display("[MON] c=%0d iss_st=%0d src_beat=%0d wbx_pend=%b req_v=%b dat_v=%b | dst_nm=%0d lqvld=%b%b%b%b",
        c, dut.iss_state, dut.next_src_beat, dut.wbx_pending, ifh.req_valid, ifh.dat_valid,
        dut.dst_nm, dut.vex_mem_lqvld_1c, dut.vex_mem_lqvld_2c, dut.vex_mem_lqvld_3c, dut.vex_mem_lqvld_div);
    end
  end

  // ---- reset + timeout ----
  int wb_beats;
  always_comb begin
    wb_beats = 0;
    for (int k=0;k<16;k++) if (seen[k]) wb_beats++;
  end
  initial begin
    rst_n = 1'b0; repeat (5) @(posedge clk); rst_n = 1'b1;
    repeat (500) @(posedge clk);
    if (wb_beats != DST_NM) $display("[FV] TIMEOUT/MISSING: %0d of %0d member writebacks seen", wb_beats, DST_NM);
    if (wb_beats == DST_NM && errors == 0 && saw_last)
      $display("[FV] *** PASSED *** (OP_SEL=%0d, LMUL=%0d, dst_members=%0d)", OP_SEL, NM, DST_NM);
    else
      $display("[FV] *** FAILED *** (OP_SEL=%0d, errors=%0d, wb_beats=%0d/%0d, saw_last=%b)",
               OP_SEL, errors, wb_beats, DST_NM, saw_last);
    $finish;
  end
endmodule

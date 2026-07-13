// -----------------------------------------------------------------------------
// tt_cii_host_wrap  (Caracal Milestone 2, Track B, Step B0)
//
// Flatten wrapper bridging the Chisel `TTCii` BlackBox to the SystemVerilog
// TT-CII stack. A Chisel BlackBox can only bind flat `logic` ports, not SV
// `interface`/`modport` ports nor `parameter type` packed-struct ports; this
// module exposes the four CII channels as flat, lane-packed ports and repacks
// them into `tt_cii_interface` structs (typed by tt_cii_caracal_pkg).
//
//   flat host ports  <->  iface_A(host)  --tt_cii relay--  iface_B  <->  coproc
//
// The two interface instances are joined by the `tt_cii` credit relay. This B0
// wraps a NULL coprocessor on iface_B (accepts issues/data, never requests or
// writes back) so the host side is self-contained and buildable. TRACK C swaps
// the null coproc for `tt_vpu_cii_wrapper_top .cii_intf(iface_B)`.
//
// Port names/directions MUST match boom.v4.vec.cii.TTCii's BlackBox io.
// -----------------------------------------------------------------------------
`include "tt_cii_caracal_pkg.svh"

module tt_cii_host_wrap
  import tt_cii_caracal_pkg::*;
(
  input  logic                        clk,
  input  logic                        rst_n,

  // Instruction issue  (host -> cop)
  input  logic                        iss_valid,
  input  logic [CII_TAG_W-1:0]        iss_tag,
  input  logic [31:0]                 iss_insn,
  input  logic [7:0]                  iss_vtype,     // {vsew[3],vlmul[3],vta,vma}
  input  logic [CII_VL_W-1:0]         iss_vl,
  input  logic [CII_VL_W-1:0]         iss_vstart,
  input  logic [1:0]                  iss_vxrm,
  input  logic [2:0]                  iss_frm,
  input  logic [CII_NUM_SRC_SLOTS-1:0] iss_hint,
  output logic                        iss_credit,

  // Source-operand request  (cop -> host)  [CII_NUM_SRC_REQ lanes packed]
  output logic                                  req_valid,
  output logic [CII_NUM_SRC_REQ*CII_TAG_W-1:0]  req_tag,
  output logic [CII_NUM_SRC_REQ*3-1:0]          req_op_id,      // srcid_e is 3b
  output logic [CII_NUM_SRC_REQ*CII_MEMBER_W-1:0] req_op_offset,
  input  logic                                  req_credit,

  // Source-operand data  (host -> cop)  [CII_NUM_SRC_DAT_RSP lanes packed]
  input  logic                                   dat_valid,
  input  logic [CII_NUM_SRC_DAT_RSP*CII_VLEN-1:0] dat_data,
  output logic                                   dat_credit,

  // Result writeback  (cop -> host)  [CII_NUM_DST_WB lanes packed]
  output logic                                 wb_valid,
  output logic [CII_NUM_DST_WB*CII_TAG_W-1:0]  wb_tag,
  output logic [CII_NUM_DST_WB*CII_VLEN-1:0]   wb_data,
  output logic [CII_NUM_DST_WB*CII_MEMBER_W-1:0] wb_dst_offset,
  output logic [CII_NUM_DST_WB-1:0]            wb_wr_en,
  output logic [CII_NUM_DST_WB*$bits(cii_caracal_wb_status_t)-1:0] wb_status,
  input  logic                                 wb_credit
);

  // ---- the two interface instances (Caracal-typed) -------------------------
  tt_cii_interface #(
    .INSTR_T         (cii_caracal_instr_t),
    .SRCID_T         (cii_caracal_srcid_t),
    .SRC_OFFSET_T    (cii_caracal_offset_t),
    .DST_OFFSET_T    (cii_caracal_offset_t),
    .TAG_T           (cii_caracal_tag_t),
    .SRC_DATA_T      (cii_caracal_data_t),
    .DST_DATA_T      (cii_caracal_data_t),
    .WB_STATUS_T     (cii_caracal_wb_status_t),
    .SRC_FRWD_HINT_T (cii_caracal_frwd_hint_t),
    .CII_NUM_INST_ISSUE  (CII_NUM_INST_ISSUE),
    .CII_NUM_SRC_REQ     (CII_NUM_SRC_REQ),
    .CII_NUM_SRC_DAT_RSP (CII_NUM_SRC_DAT_RSP),
    .CII_NUM_DST_WB      (CII_NUM_DST_WB),
    .CII_MISA            (CII_MISA)
  ) iface_A ();  // host  <-> relay

  tt_cii_interface #(
    .INSTR_T         (cii_caracal_instr_t),
    .SRCID_T         (cii_caracal_srcid_t),
    .SRC_OFFSET_T    (cii_caracal_offset_t),
    .DST_OFFSET_T    (cii_caracal_offset_t),
    .TAG_T           (cii_caracal_tag_t),
    .SRC_DATA_T      (cii_caracal_data_t),
    .DST_DATA_T      (cii_caracal_data_t),
    .WB_STATUS_T     (cii_caracal_wb_status_t),
    .SRC_FRWD_HINT_T (cii_caracal_frwd_hint_t),
    .CII_NUM_INST_ISSUE  (CII_NUM_INST_ISSUE),
    .CII_NUM_SRC_REQ     (CII_NUM_SRC_REQ),
    .CII_NUM_SRC_DAT_RSP (CII_NUM_SRC_DAT_RSP),
    .CII_NUM_DST_WB      (CII_NUM_DST_WB),
    .CII_MISA            (CII_MISA)
  ) iface_B ();  // relay <-> coproc

  // ---- credit relay --------------------------------------------------------
  // Per tt_cii.sv: the real host attaches to the relay's cii_coproc port, the
  // real coprocessor to its cii_host port.
  tt_cii #(
    .CII_N_ISS_CREDITS (CII_N_ISS_CREDITS),
    .CII_N_REQ_CREDITS (CII_N_REQ_CREDITS),
    .CII_N_DAT_CREDITS (CII_N_DAT_CREDITS),
    .CII_N_WB_CREDITS  (CII_N_WB_CREDITS)
  ) u_relay (
    .clk        (clk),
    .rst_n      (rst_n),
    .cii_host   (iface_B),   // relay drives host modport -> the coprocessor
    .cii_coproc (iface_A)    // relay drives coproc modport -> the real host (this wrapper)
  );

  // ---- host side: flat ports <-> iface_A (this wrapper IS the real host) ---
  // Issue (host drives iss_*, reads iss_credit)
  assign iface_A.iss_valid             = iss_valid;
  assign iface_A.iss_data[0].tag       = iss_tag;
  assign iface_A.iss_data[0].instr.insn   = iss_insn;
  assign iface_A.iss_data[0].instr.vtype  = cii_caracal_vtype_t'(iss_vtype);
  assign iface_A.iss_data[0].instr.vl     = iss_vl;
  assign iface_A.iss_data[0].instr.vstart = iss_vstart;
  assign iface_A.iss_data[0].instr.vxrm   = iss_vxrm;
  assign iface_A.iss_data[0].instr.frm    = iss_frm;
  assign iface_A.iss_data[0].instr_src_valid = iss_hint;
  assign iss_credit = iface_A.iss_credit;

  // Src-request (host reads req_*, drives req_credit)
  assign req_valid = iface_A.req_valid;
  for (genvar i = 0; i < CII_NUM_SRC_REQ; i++) begin : g_req
    assign req_tag      [i*CII_TAG_W    +: CII_TAG_W]    = iface_A.req_data[i].tag;
    assign req_op_id    [i*3            +: 3]            = iface_A.req_data[i].rsp_src_id;
    assign req_op_offset[i*CII_MEMBER_W +: CII_MEMBER_W] = iface_A.req_data[i].rsp_src_offset;
  end
  assign iface_A.req_credit = req_credit;

  // Src-data (host drives dat_*, reads dat_credit)
  assign iface_A.dat_valid = dat_valid;
  for (genvar j = 0; j < CII_NUM_SRC_DAT_RSP; j++) begin : g_dat
    assign iface_A.dat_data[j].rsp_dat = dat_data[j*CII_VLEN +: CII_VLEN];
  end
  assign dat_credit = iface_A.dat_credit;

  // Writeback (host reads wb_*, drives wb_credit)
  assign wb_valid = iface_A.wb_valid;
  for (genvar k = 0; k < CII_NUM_DST_WB; k++) begin : g_wb
    assign wb_tag       [k*CII_TAG_W    +: CII_TAG_W]    = iface_A.wb_data[k].inst_tag;
    assign wb_data      [k*CII_VLEN     +: CII_VLEN]     = iface_A.wb_data[k].wb_data;
    assign wb_dst_offset[k*CII_MEMBER_W +: CII_MEMBER_W] = iface_A.wb_data[k].wb_dst_offset;
    assign wb_wr_en     [k]                              = iface_A.wb_data[k].wb_wr_en;
    assign wb_status[k*$bits(cii_caracal_wb_status_t) +: $bits(cii_caracal_wb_status_t)]
                                                         = iface_A.wb_data[k].wb_fp_flags;
  end
  assign iface_A.wb_credit = wb_credit;

  // ---- coproc side: the real VPU coprocessor on iface_B (Track C, C6) -------
  // tt_vpu_cii_wrapper_top attaches to iface_B's coprocessor modport: it
  // receives issues, pulls source operands, and pushes result writebacks over
  // the four CII channels. Debug commit-trace outputs are left unconnected here
  // (the host-side cosim trace is driven from BOOM, not this bridge).
  tt_vpu_cii_wrapper_top #(.VLEN(CII_VLEN)) u_vpu (
    .clk                 (clk),
    .reset_n             (rst_n),
    .cii_intf            (iface_B),
    .debug_wb_vec_valid  (),
    .debug_wb_vec_wdata  (),
    .debug_wb_vec_wmask  ()
  );

endmodule

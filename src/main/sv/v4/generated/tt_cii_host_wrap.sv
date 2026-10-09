/*
Tenstorrent CONFIDENTIAL
__________________
 Tenstorrent Inc.
 All Rights Reserved.

NOTICE:  All information contained herein is, and remains
the property of Tenstorrent Inc.  The intellectual
and technical concepts contained
herein are proprietary to Tenstorrent Inc.
and may be covered by U.S., Canadian and Foreign Patents,
patents in process, and are protected by trade secret or copyright law.
Dissemination of this information or reproduction of this material
is strictly forbidden unless prior written permission is obtained
from Tenstorrent Inc.
*/

/*
  tt_cii_host_wrap — the flatten shim joining the Chisel BOOM host to the
  SystemVerilog TT-CII stack, and the only new SystemVerilog module in v2.
*/

`include "tt_cii_caracal_pkg.svh"

module tt_cii_host_wrap
  import tt_cii_caracal_pkg::*;
(
  //@req-spec-cii.c6
  input  logic                          clk,
  input  logic                          core_reset,

  input  logic                          iss_valid,
  input  logic [CII_TAG_W-1:0]          iss_tag,
  input  logic [31:0]                   iss_insn,
  input  logic [7:0]                    iss_vtype,
  input  logic [CII_VL_W-1:0]           iss_vl,
  input  logic [CII_VL_W-1:0]           iss_vstart,
  input  logic [1:0]                    iss_vxrm,
  input  logic [2:0]                    iss_frm,
  input  logic [CII_NUM_SRC_SLOTS-1:0]  iss_hint,
  output logic                          iss_credit,

  output logic                                        req_valid,
  output logic [CII_NUM_SRC_REQ*CII_TAG_W-1:0]         req_tag,
  output logic [CII_NUM_SRC_REQ*$bits(cii_caracal_srcid_t)-1:0]  req_op_id,
  output logic [CII_NUM_SRC_REQ*CII_MEMBER_W-1:0]      req_op_offset,
  input  logic                                        req_credit,

  input  logic                                         dat_valid,
  input  logic [CII_NUM_SRC_DAT_RSP*CII_VLEN-1:0]       dat_data,
  output logic                                         dat_credit,

  output logic                                                        wb_valid,
  output logic [CII_NUM_DST_WB*CII_TAG_W-1:0]                         wb_tag,
  output logic [CII_NUM_DST_WB*CII_VLEN-1:0]                          wb_data,
  output logic [CII_NUM_DST_WB*CII_MEMBER_W-1:0]                      wb_dst_offset,
  output logic [CII_NUM_DST_WB-1:0]                                   wb_wr_en,
  output logic [CII_NUM_DST_WB*$bits(cii_caracal_wb_status_t)-1:0]    wb_status,
  input  logic                                                        wb_credit
);

  //@req-spec-cii.c2
  logic core_rst_n;
  assign core_rst_n = ~core_reset;

  //@req-spec-cii.c5
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
  ) ifh (
    .clk   (clk),
    .rst_n (core_rst_n)
  );

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
  ) ifc (
    .clk   (clk),
    .rst_n (core_rst_n)
  );

  //@req-spec-cii.c5
  tt_cii #(
    .CII_NUM_INST_ISSUE  (CII_NUM_INST_ISSUE),
    .CII_NUM_SRC_REQ     (CII_NUM_SRC_REQ),
    .CII_NUM_SRC_DAT_RSP (CII_NUM_SRC_DAT_RSP),
    .CII_NUM_DST_WB      (CII_NUM_DST_WB),
    .CII_N_ISS_CREDITS   (CII_N_ISS_CREDITS),
    .CII_N_REQ_CREDITS   (CII_N_REQ_CREDITS),
    .CII_N_DAT_CREDITS   (CII_N_DAT_CREDITS),
    .CII_N_WB_CREDITS    (CII_N_WB_CREDITS)
  ) relay (
    .clk        (clk),
    .rst_n      (core_rst_n),
    .cii_host   (ifc),
    .cii_coproc (ifh)
  );

  //@req-spec-cii.c7
  //@req-spec-cii.c4
  assign ifh.iss_valid              = iss_valid;
  assign ifh.iss_data[0].tag        = iss_tag;
  assign ifh.iss_data[0].instr.insn    = iss_insn;
  assign ifh.iss_data[0].instr.vtype   = cii_caracal_vtype_t'(iss_vtype);
  assign ifh.iss_data[0].instr.vl      = iss_vl;
  assign ifh.iss_data[0].instr.vstart  = iss_vstart;
  assign ifh.iss_data[0].instr.vxrm    = iss_vxrm;
  assign ifh.iss_data[0].instr.frm     = iss_frm;
  assign ifh.iss_data[0].instr_src_valid = iss_hint;
  assign iss_credit = ifh.iss_credit;

  //@req-spec-cii.c7
  //@req-spec-cii.c4
  assign req_valid = ifh.req_valid;
  for (genvar i = 0; i < CII_NUM_SRC_REQ; i++) begin : g_req
    assign req_tag[i*CII_TAG_W +: CII_TAG_W] = ifh.req_data[i].tag;
    //@req-spec-cii.c6
    assign req_op_id[i*$bits(cii_caracal_srcid_t) +: $bits(cii_caracal_srcid_t)] = ifh.req_data[i].rsp_src_id;
    assign req_op_offset[i*CII_MEMBER_W +: CII_MEMBER_W] = ifh.req_data[i].rsp_src_offset;
  end
  assign ifh.req_credit = req_credit;

  //@req-spec-cii.c7
  //@req-spec-cii.c4
  assign ifh.dat_valid = dat_valid;
  for (genvar j = 0; j < CII_NUM_SRC_DAT_RSP; j++) begin : g_dat
    assign ifh.dat_data[j].rsp_dat = dat_data[j*CII_VLEN +: CII_VLEN];
  end
  assign dat_credit = ifh.dat_credit;

  //@req-spec-cii.c7
  //@req-spec-cii.c4
  assign wb_valid = ifh.wb_valid;
  for (genvar k = 0; k < CII_NUM_DST_WB; k++) begin : g_wb
    assign wb_tag[k*CII_TAG_W +: CII_TAG_W]                = ifh.wb_data[k].inst_tag;
    assign wb_data[k*CII_VLEN +: CII_VLEN]                  = ifh.wb_data[k].wb_data;
    assign wb_dst_offset[k*CII_MEMBER_W +: CII_MEMBER_W]    = ifh.wb_data[k].wb_dst_offset;
    assign wb_wr_en[k]                                      = ifh.wb_data[k].wb_wr_en;
    //@req-spec-cii.c6
    assign wb_status[k*$bits(cii_caracal_wb_status_t) +: $bits(cii_caracal_wb_status_t)]
                                                             = ifh.wb_data[k].wb_fp_flags;
  end
  assign ifh.wb_credit = wb_credit;

  //@req-spec-cii.c5
  tt_vpu_cii_wrapper_top #(
    .VLEN (CII_VLEN)
  ) vpu (
    .clk                (clk),
    .reset_n             (core_rst_n),
    .cii_intf            (ifc),
    .debug_wb_vec_valid  (),
    .debug_wb_vec_wdata  (),
    .debug_wb_vec_wmask  ()
  );

  //@req-spec-cii.c6
  //@req-spec-cii.c5
endmodule

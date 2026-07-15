#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# VCS elaboration lint for the Caracal CII VPU coprocessor wrapper (Track C).
#
# Assembles the full SystemVerilog filelist (VPU + tt-cii + HardFloat + common
# util/arith), generates a minimal interface testbench, and elaborates
# tt_vpu_cii_wrapper_top with VCS. Use this to verify each Track C slice.
#
#   Usage:  ./lint_vpu_cii.sh [REPO_ROOT]     (default REPO_ROOT=/root/my-chipyard)
#   Env:    BUILD=<dir>  VCS_HOME must be set (vcs on PATH or under $VCS_HOME/bin)
#
# Exit 0 + "elaboration OK" => the wrapper + VPU + CII stack elaborate cleanly.
# ---------------------------------------------------------------------------
set -euo pipefail

REPO="${1:-/root/my-chipyard}"
SV="$REPO/generators/boom/src/main/sv/v4"
HF="$REPO/generators/boom/src/main/resources/HardFloat/source"
BUILD="${BUILD:-/tmp/vpu_cii_lint}"
[ -n "${VCS_HOME:-}" ] && export PATH="$VCS_HOME/bin:$PATH"

rm -rf "$BUILD"; mkdir -p "$BUILD"; cd "$BUILD"

# HardFloat macro pre-include (defines `flControl_default etc.). Quoted heredoc
# so the backtick-includes are written literally.
cat > hf_pre.sv <<'HFEOF'
`include "HardFloat_consts.vi"
`include "HardFloat_specialize.vi"
HFEOF

# Minimal testbench: the Caracal-typed CII interface + the wrapper.
cat > cii_lint_tb.sv <<'TBEOF'
`include "tt_cii_caracal_pkg.svh"
module cii_lint_tb import tt_cii_caracal_pkg::*; ;
  logic clk = 1'b0, rst_n = 1'b0;
  tt_cii_interface #(
    .INSTR_T(cii_caracal_instr_t), .SRCID_T(cii_caracal_srcid_t),
    .SRC_OFFSET_T(cii_caracal_offset_t), .DST_OFFSET_T(cii_caracal_offset_t),
    .TAG_T(cii_caracal_tag_t), .SRC_DATA_T(cii_caracal_data_t),
    .DST_DATA_T(cii_caracal_data_t), .WB_STATUS_T(cii_caracal_wb_status_t),
    .SRC_FRWD_HINT_T(cii_caracal_frwd_hint_t),
    .CII_NUM_INST_ISSUE(CII_NUM_INST_ISSUE), .CII_NUM_SRC_REQ(CII_NUM_SRC_REQ),
    .CII_NUM_SRC_DAT_RSP(CII_NUM_SRC_DAT_RSP), .CII_NUM_DST_WB(CII_NUM_DST_WB),
    .CII_MISA(CII_MISA)
  ) intf ();
  logic dbg_v; logic [256*8-1:0] dbg_d; logic [7:0] dbg_m;
  tt_vpu_cii_wrapper_top #(.VLEN(256)) dut (
    .clk(clk), .reset_n(rst_n), .cii_intf(intf.coprocessor),
    .debug_wb_vec_valid(dbg_v), .debug_wb_vec_wdata(dbg_d), .debug_wb_vec_wmask(dbg_m));
endmodule
TBEOF

# common/utility minus the bind-style *_assert.sv helpers (don't compile alone).
UTIL=$(ls "$SV"/common/utility/*.sv | grep -v assert)
ARITH=$(ls "$SV"/common/arithmetic/*.sv)

vcs -sverilog -full64 -nc -q -no_save +error+30 -top cii_lint_tb -o "$BUILD/simv" \
  +incdir+"$SV"/vpu/packages +incdir+"$SV"/vpu/decoder +incdir+"$SV"/tt-cii/src \
  +incdir+"$SV"/common/utility +incdir+"$SV"/common/arithmetic +incdir+"$HF" +incdir+"$HF"/RISCV \
  hf_pre.sv \
  "$SV"/vpu/packages/tt_briscv_pkg.svh "$SV"/tt-cii/src/tt_cii_caracal_pkg.svh \
  $UTIL $ARITH "$HF"/*.v "$HF"/RISCV/*.v \
  "$SV"/tt-cii/src/rv_async_rst_dff.sv "$SV"/tt-cii/src/rv_async_rst_dff_Tdat.sv "$SV"/tt-cii/src/tt_cii_fifo.sv \
  "$SV"/tt-cii/src/tt_cii_channel.sv "$SV"/tt-cii/src/tt_cii_interface.sv "$SV"/tt-cii/src/tt_cii.sv \
  "$SV"/vpu/decoder/autogen_riscv_imabfv.v "$SV"/vpu/decoder/tt_ascii_instrn_decode.sv \
  "$SV"/vpu/decoder/tt_decoded_mux.sv "$SV"/vpu/decoder/tt_decoder.sv "$SV"/vpu/decoder/tt_id.sv \
  "$SV"/vpu/execution/int_datapath_unit/*.sv "$SV"/vpu/execution/fp_datapath_unit/*.sv \
  "$SV"/vpu/reg/tt_vec_regfile.sv "$SV"/vpu/tt_vec_top.sv "$SV"/vpu/tt_vpu_cii_wrapper_top.sv \
  cii_lint_tb.sv

echo "=== elaboration OK: $BUILD/simv ==="

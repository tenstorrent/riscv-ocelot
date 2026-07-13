#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Functional smoke test for the CII VPU coprocessor (Track C).
# Compiles the full VPU + CII stack with tb/cii_fv_tb.sv as top and runs it.
#   Usage:  ./fv_vpu_cii.sh [REPO_ROOT]     (default /root/my-chipyard)
# ---------------------------------------------------------------------------
set -euo pipefail
REPO="${1:-/root/my-chipyard}"
SV="$REPO/generators/boom/src/main/sv/v4"
HF="$REPO/generators/boom/src/main/resources/HardFloat/source"
BUILD="${BUILD:-/tmp/vpu_cii_fv}"
[ -n "${VCS_HOME:-}" ] && export PATH="$VCS_HOME/bin:$PATH"

rm -rf "$BUILD"; mkdir -p "$BUILD"; cd "$BUILD"
cat > hf_pre.sv <<'HFEOF'
`include "HardFloat_consts.vi"
`include "HardFloat_specialize.vi"
HFEOF

UTIL=$(ls "$SV"/common/utility/*.sv | grep -v assert)
ARITH=$(ls "$SV"/common/arithmetic/*.sv)

vcs -sverilog -full64 -nc -q -no_save +error+30 -top cii_fv_tb -o "$BUILD/simv" \
  +incdir+"$SV"/vpu/packages +incdir+"$SV"/vpu/decoder +incdir+"$SV"/tt-cii/src \
  +incdir+"$SV"/common/utility +incdir+"$SV"/common/arithmetic +incdir+"$HF" +incdir+"$HF"/RISCV \
  hf_pre.sv \
  "$SV"/vpu/packages/tt_briscv_pkg.svh "$SV"/tt-cii/src/tt_cii_caracal_pkg.svh \
  $UTIL $ARITH "$HF"/*.v "$HF"/RISCV/*.v \
  "$SV"/tt-cii/src/rv_async_rst_dff.sv "$SV"/tt-cii/src/tt_cii_fifo.sv \
  "$SV"/tt-cii/src/tt_cii_channel.sv "$SV"/tt-cii/src/tt_cii_interface.sv "$SV"/tt-cii/src/tt_cii.sv \
  "$SV"/vpu/decoder/autogen_riscv_imabfv.v "$SV"/vpu/decoder/tt_ascii_instrn_decode.sv \
  "$SV"/vpu/decoder/tt_decoded_mux.sv "$SV"/vpu/decoder/tt_decoder.sv "$SV"/vpu/decoder/tt_id.sv \
  "$SV"/vpu/execution/int_datapath_unit/*.sv "$SV"/vpu/execution/fp_datapath_unit/*.sv \
  "$SV"/vpu/reg/tt_vec_regfile.sv "$SV"/vpu/tt_vec_top.sv "$SV"/vpu/tt_vpu_cii_wrapper_top.sv \
  "$SV"/vpu/tb/cii_fv_tb.sv

echo "=== compile OK, running simv ==="
"$BUILD/simv" -no_save

// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._

/** The single bind layer for all Caracal formal properties.
  *
  * `layer.Convention.Bind` makes firtool lower everything inside a
  * `layer.block(BoomSvaLayer) { ... }` into TWO separate files instead of
  * inlining it in the DUT:
  *
  *   <Dut>_BoomSvaLayer.sv          the checker module (the properties)
  *   layers_<Dut>_BoomSvaLayer.sv   a SystemVerilog `bind` hooking it up
  *
  * The DUT's own .sv contains no assertion text and gains no ports.
  *
  * Exactly one layer object exists in this tree. Properties are grouped by
  * checker file, not by layer — a second layer would mean a second set of
  * `layers_*.sv` files to remember to put on the compile line.
  *
  * TWO THINGS THAT WILL BITE:
  *
  *  1. `layers_*.sv` is NOT listed in firtool's `filelist.f`, so chipyard's
  *     default simulation source list does not include it. Without
  *     `EXTRA_SIM_SOURCES='$(wildcard $(GEN_COLLATERAL_DIR)/layers_*.sv)'` the
  *     build succeeds and binds nothing. See
  *     req-formal-chisel/references/validate.md.
  *
  *  2. Chisel 7 replaces `layer.Convention.Bind` with `LayerConfig.Extract()`.
  *     The `USE_CHISEL7` path in chipyard's common.mk additionally passes
  *     `--disable-layers=Verification.Assume,Verification.Cover`, which would
  *     strip every AssumeProperty and CoverProperty in the tree.
  *
  * Verified with Chisel 6.7.0 / firtool 1.75.0 (the chipyard pins).
  */
object BoomSvaLayer extends layer.Layer(layer.Convention.Bind)

// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.lsu.VecRangeAgen]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in
  * VecRangeAgen.scala; firtool emits these into their own .sv and a
  * SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/reqs/formal-lsu.yaml.
  */
object VecRangeAgenChecks {
  def apply(
    rangeValid:   Bool,
    rangeLen:     UInt,
    rangeEew:     UInt,
    scalarValid:  Bool,
    scalarVl:     UInt,
    releaseValid: Bool,
    isWholeReg:   Bool,
    isMaskOp:     Bool
  ): Unit = {

    //@formal-req-spec-lsu.c3
    AssertProperty(
      rangeValid |=> Sequence.BoolSequence(!rangeValid),
      label = Some("vec_range_agen_one_range_per_activation")
    )

    // Antecedent reachability for vec_range_agen_one_range_per_activation (formal-lsu.c6).
    CoverProperty(rangeValid, label = Some("vec_range_agen_range_push_seen"))

    //@formal-req-spec-lsu.c4
    AssertProperty(
      (rangeValid && !isWholeReg && !isMaskOp) |-> Sequence.BoolSequence((rangeLen >> rangeEew) === scalarVl),
      label = Some("vec_range_agen_len_is_vl_times_eew")
    )

    // Antecedent reachability for vec_range_agen_len_is_vl_times_eew (formal-lsu.c8).
    CoverProperty(rangeValid && !isWholeReg && !isMaskOp, label = Some("vec_range_agen_us_range_push_seen"))

    //@formal-req-spec-lsu.j6
    AssertProperty(
      releaseValid |-> scalarValid,
      label = Some("vec_range_agen_release_needs_vl")
    )

    // Antecedent reachability for vec_range_agen_release_needs_vl (formal-lsu.j6).
    CoverProperty(releaseValid, label = Some("vec_range_agen_release_seen"))
  }
}

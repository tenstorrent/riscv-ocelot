// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.lsu.VecQueueReservation]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in
  * VecQueueReservation.scala; firtool emits these into their own .sv and a
  * SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/formal/formal-lsu.yaml.
  */
object VecQueueReservationChecks {
  def apply(
    laneValid:      Bool,
    laneUsesStq:    Bool,
    laneAddrCount:  UInt,
    laneDataCount:  UInt,
    resvAddrValid:  Bool,
    resvDataValid:  Bool,
    retireValid:    Bool,
    retireRowValid: Bool,
    occSsiAddr:     UInt,
    ssiDepthLit:    UInt,
    ldCapLit:       UInt
  ): Unit = {

    //@formal-req-spec-lsu.b19
    AssertProperty(
      (laneValid && laneUsesStq) |-> Sequence.BoolSequence((laneAddrCount =/= 0.U) && (laneDataCount =/= 0.U)),
      label = Some("vec_resv_store_count_nonzero")
    )

    // Antecedent reachability for vec_resv_store_count_nonzero (formal-lsu.b6).
    CoverProperty(laneValid && laneUsesStq, label = Some("vec_resv_store_dispatch_seen"))

    //@formal-req-spec-lsu.b4
    //@formal-req-spec-lsu.b5
    AssertProperty(
      retireValid |-> retireRowValid,
      label = Some("vec_resv_retire_row_live")
    )

    // Antecedent reachability for vec_resv_retire_row_live (formal-lsu.b8).
    CoverProperty(retireValid, label = Some("vec_resv_retire_seen"))

    //@formal-req-spec-lsu.d12
    //@formal-req-spec-lsu.d13
    AssertProperty(
      (resvAddrValid && laneUsesStq) |-> resvDataValid,
      label = Some("vec_resv_store_claims_both_queues")
    )

    // Antecedent reachability for vec_resv_store_claims_both_queues (formal-lsu.d12).
    CoverProperty(resvAddrValid && laneUsesStq, label = Some("vec_resv_store_grant_seen"))

    //@formal-req-spec-lsu.b14
    //@formal-req-spec-lsu.b15
    AssertProperty(
      Sequence.BoolSequence(occSsiAddr <= ssiDepthLit),
      label = Some("vec_resv_occupancy_within_depth")
    )

    //@formal-req-spec-lsu.b18
    AssertProperty(
      Sequence.BoolSequence(laneValid && !laneUsesStq) |-> Sequence.BoolSequence(laneAddrCount <= ldCapLit),
      label = Some("vec_resv_load_count_within_cap")
    )

    // Antecedent reachability for vec_resv_load_count_within_cap (formal-lsu.b12).
    CoverProperty(laneValid && !laneUsesStq, label = Some("vec_resv_load_dispatch_seen"))
  }
}

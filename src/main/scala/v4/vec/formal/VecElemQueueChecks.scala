// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.lsu.VecElemQueue]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in
  * VecElemQueue.scala; firtool emits these into their own .sv and a
  * SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/formal/formal-lsu.yaml.
  */
object VecElemQueueChecks {
  def apply(
    enqValid:       Bool,
    enqInRegion:    Bool,
    claimValid:     Bool,
    claimEntries:   UInt,
    avail:          UInt,
    freeValid:      Bool,
    freeBase:       UInt,
    qHead:          UInt,
    updateValid:    Bool,
    updateInRegion: Bool,
    rdReqValid:     Bool,
    rdFilled:       Bool,
    squashValid:    Bool,
    squashLen:      UInt,
    occRegion:      UInt,
    consumeValid:   Bool,
    consumeInRegion: Bool
  ): Unit = {

    //@formal-req-spec-lsu.b6
    //@formal-req-spec-lsu.d11
    AssertProperty(
      enqValid |-> enqInRegion,
      label = Some("vec_elem_queue_enq_in_reserved_region")
    )

    // Antecedent reachability for vec_elem_queue_enq_in_reserved_region (formal-lsu.b1).
    CoverProperty(enqValid, label = Some("vec_elem_queue_enq_seen"))

    //@formal-req-spec-lsu.b7
    //@formal-req-spec-lsu.d8
    AssertProperty(
      freeValid |-> Sequence.BoolSequence(freeBase === qHead),
      label = Some("vec_elem_queue_free_base_is_head")
    )

    // Antecedent reachability for vec_elem_queue_free_base_is_head (formal-lsu.b3).
    CoverProperty(freeValid, label = Some("vec_elem_queue_free_seen"))

    //@formal-req-spec-lsu.d13
    AssertProperty(
      claimValid |-> Sequence.BoolSequence(claimEntries <= avail),
      label = Some("vec_elem_queue_claim_fits_avail")
    )

    // Antecedent reachability for vec_elem_queue_claim_fits_avail (formal-lsu.d1).
    CoverProperty(claimValid, label = Some("vec_elem_queue_claim_seen"))

    //@formal-req-spec-lsu.j8
    AssertProperty(
      updateValid |-> updateInRegion,
      label = Some("vec_elem_queue_xlate_update_in_region")
    )

    // Antecedent reachability for vec_elem_queue_xlate_update_in_region (formal-lsu.j1).
    CoverProperty(updateValid, label = Some("vec_elem_queue_xlate_update_seen"))

    //@formal-req-spec-lsu.j10
    AssertProperty(
      Sequence.BoolSequence(!rdReqValid) |=> Sequence.BoolSequence(!rdFilled),
      label = Some("vec_elem_queue_no_filled_resp_without_req")
    )

    // Antecedent reachability for vec_elem_queue_no_filled_resp_without_req (formal-lsu.j3).
    CoverProperty(!rdReqValid, label = Some("vec_elem_queue_rd_idle_seen"))

    //@formal-req-spec-lsu.i3
    //@formal-req-spec-lsu.i5
    AssertProperty(
      squashValid |-> Sequence.BoolSequence(squashLen <= occRegion),
      label = Some("vec_elem_queue_squash_within_region")
    )

    // Antecedent reachability for vec_elem_queue_squash_within_region (formal-lsu.i9).
    CoverProperty(squashValid, label = Some("vec_elem_queue_squash_seen"))

    //@formal-req-spec-lsu.k7
    //@formal-req-spec-lsu.b10
    AssertProperty(
      consumeValid |-> consumeInRegion,
      label = Some("vec_elem_queue_consume_in_region")
    )

    // Antecedent reachability for vec_elem_queue_consume_in_region (formal-lsu.k2).
    CoverProperty(consumeValid, label = Some("vec_elem_queue_consume_seen"))
  }
}

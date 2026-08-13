// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.lsu.VecElemAgen]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in VecElemAgen.scala;
  * firtool emits these into their own .sv and a SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/reqs/formal-lsu.yaml.
  */
object VecElemAgenChecks {
  def apply(
    pushValid: Bool,
    pushFire:  Bool,
    emitCtr:   UInt,
    resvCount: UInt,
    newFault:  Bool,
    faultSeen: Bool
  ): Unit = {

    //@formal-req-spec-lsu.b11
    //@formal-req-spec-lsu.j1
    AssertProperty(
      pushValid |-> Sequence.BoolSequence(emitCtr < resvCount),
      label = Some("vec_elem_agen_emit_within_reservation")
    )

    // Antecedent reachability for vec_elem_agen_emit_within_reservation (formal-lsu.b9).
    CoverProperty(pushValid, label = Some("vec_elem_agen_emit_seen"))

    //@formal-req-spec-lsu.f5
    AssertProperty(
      newFault |=> faultSeen,
      label = Some("vec_elem_agen_fault_latches_seen")
    )

    // Antecedent reachability for vec_elem_agen_fault_latches_seen (formal-lsu.f4).
    CoverProperty(newFault, label = Some("vec_elem_agen_new_fault_seen"))

    //@formal-req-spec-lsu.f9
    //@formal-req-spec-lsu.a10
    AssertProperty(
      faultSeen |-> Sequence.BoolSequence(!pushValid),
      label = Some("vec_elem_agen_no_emit_after_fault")
    )

    // Antecedent reachability for vec_elem_agen_no_emit_after_fault (formal-lsu.f6).
    CoverProperty(faultSeen, label = Some("vec_elem_agen_fault_seen_state"))

    //@formal-req-spec-lsu.f8
    AssertProperty(
      pushFire |=> Sequence.BoolSequence(emitCtr =/= 0.U),
      label = Some("vec_elem_agen_emit_counter_advances")
    )

    // Antecedent reachability for vec_elem_agen_emit_counter_advances (formal-lsu.f8).
    CoverProperty(pushFire, label = Some("vec_elem_agen_push_fire_seen"))
  }
}

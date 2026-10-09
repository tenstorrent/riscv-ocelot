// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.lsu.VecSquashUnit]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in VecSquashUnit.scala;
  * firtool emits these into their own .sv and a SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/reqs/formal-lsu.yaml.
  */
object VecSquashUnitChecks {
  def apply(
    branch:        Bool,
    flush:         Bool,
    rollbackLdq:   UInt,
    brLdqIdx:      UInt,
    killLdq:       UInt,
    kill0:         Bool,
    killUop0Valid: Bool
  ): Unit = {

    //@formal-req-spec-lsu.i2
    AssertProperty(
      branch |-> Sequence.BoolSequence(rollbackLdq === brLdqIdx),
      label = Some("vec_squash_branch_pivot_is_branch_ldq")
    )

    // Antecedent reachability for vec_squash_branch_pivot_is_branch_ldq (formal-lsu.i1).
    CoverProperty(branch, label = Some("vec_squash_branch_seen"))

    //@formal-req-spec-lsu.i7
    AssertProperty(
      Sequence.BoolSequence(!branch && !flush) |-> Sequence.BoolSequence(killLdq === 0.U),
      label = Some("vec_squash_no_kill_without_event")
    )

    // Antecedent reachability for vec_squash_no_kill_without_event (formal-lsu.i3).
    CoverProperty(!branch && !flush, label = Some("vec_squash_quiet_seen"))

    //@formal-req-spec-lsu.i4
    AssertProperty(
      flush |-> Sequence.BoolSequence(killLdq.andR),
      label = Some("vec_squash_flush_kills_every_entry")
    )

    // Antecedent reachability for vec_squash_flush_kills_every_entry (formal-lsu.i5).
    CoverProperty(flush, label = Some("vec_squash_flush_seen"))

    //@formal-req-spec-lsu.i6
    AssertProperty(
      kill0 |-> killUop0Valid,
      label = Some("vec_squash_kill_requires_uop")
    )

    // Antecedent reachability for vec_squash_kill_requires_uop (formal-lsu.i7).
    CoverProperty(kill0, label = Some("vec_squash_stage_kill_seen"))
  }
}

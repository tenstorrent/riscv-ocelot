// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.lsu.VecGroupCopy]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in VecGroupCopy.scala;
  * firtool emits these into their own .sv and a SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/reqs/formal-lsu.yaml.
  */
object VecGroupCopyChecks {
  def apply(
    needCopy:       Bool,
    noCopyFire:     Bool,
    noCopyValid:    Bool,
    vlZero:         Bool,
    allInactive:    Bool,
    gcFlush:        Bool,
    w0Granted:      Bool,
    stageBDst:      UInt,
    stageBSrc:      UInt,
    copyDoneValid:  Bool,
    groupDoneValid: Bool,
    lcbR2Valid:     Bool,
    lcbR2Bits:      UInt,
    vrfR2Bits:      UInt,
    lcbW0Valid:     Bool,
    lcbW0Addr:      UInt,
    vrfW0Addr:      UInt,
    vrfW0Mask:      UInt,
    allOnesMask:    UInt
  ): Unit = {

    //@formal-req-spec-lsu.m12
    //@formal-req-spec-lsu.m3
    AssertProperty(
      needCopy |-> Sequence.BoolSequence(vlZero || allInactive),
      label = Some("vec_gcopy_copy_only_for_no_execution")
    )

    // Antecedent reachability for vec_gcopy_copy_only_for_no_execution (formal-lsu.m2).
    CoverProperty(needCopy, label = Some("vec_gcopy_copy_start_seen"))

    //@formal-req-spec-lsu.m5
    AssertProperty(
      w0Granted |-> Sequence.BoolSequence(stageBDst =/= stageBSrc),
      label = Some("vec_gcopy_member_copy_between_distinct_prns")
    )

    // Antecedent reachability for vec_gcopy_member_copy_between_distinct_prns
    // and vec_gcopy_write_mask_is_whole_register (formal-lsu.m4).
    CoverProperty(w0Granted, label = Some("vec_gcopy_member_write_seen"))

    //@formal-req-spec-lsu.m8
    AssertProperty(
      groupDoneValid |-> Sequence.BoolSequence(!(noCopyValid && copyDoneValid)),
      label = Some("vec_gcopy_single_group_done")
    )

    // Antecedent reachability for vec_gcopy_single_group_done (formal-lsu.m6).
    CoverProperty(groupDoneValid, label = Some("vec_gcopy_group_done_seen"))

    //@formal-req-spec-lsu.m9
    //@formal-req-spec-lsu.m10
    AssertProperty(
      Sequence.BoolSequence(noCopyFire && !gcFlush) |=> noCopyValid,
      label = Some("vec_gcopy_no_copy_path_completes_next_cycle")
    )

    // Antecedent reachability for vec_gcopy_no_copy_path_completes_next_cycle (formal-lsu.m8).
    CoverProperty(noCopyFire && !gcFlush, label = Some("vec_gcopy_no_copy_launch_seen"))

    //@formal-req-spec-lsu.m13
    AssertProperty(
      lcbR2Valid |-> Sequence.BoolSequence(vrfR2Bits === lcbR2Bits),
      label = Some("vec_gcopy_lcb_wins_r2")
    )

    // Antecedent reachability for vec_gcopy_lcb_wins_r2 (formal-lsu.m10).
    CoverProperty(lcbR2Valid, label = Some("vec_gcopy_lcb_r2_request_seen"))

    //@formal-req-spec-lsu.m14
    AssertProperty(
      lcbW0Valid |-> Sequence.BoolSequence(vrfW0Addr === lcbW0Addr),
      label = Some("vec_gcopy_lcb_wins_w0")
    )

    // Antecedent reachability for vec_gcopy_lcb_wins_w0 (formal-lsu.m12).
    CoverProperty(lcbW0Valid, label = Some("vec_gcopy_lcb_w0_write_seen"))

    //@formal-req-spec-lsu.m11
    AssertProperty(
      w0Granted |-> Sequence.BoolSequence(vrfW0Mask === allOnesMask),
      label = Some("vec_gcopy_write_mask_is_whole_register")
    )
  }
}

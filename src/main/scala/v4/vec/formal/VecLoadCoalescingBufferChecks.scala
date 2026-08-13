// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.util._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.lsu.VecLoadCoalescingBuffer]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in
  * VecLoadCoalescingBuffer.scala; firtool emits these into their own .sv and a
  * SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/reqs/formal-lsu.yaml.
  */
object VecLoadCoalescingBufferChecks {
  def apply(
    respValid:            Bool,
    respDstByte:          UInt,
    hitVec:               UInt,
    vLenBytesLit:         UInt,
    e0Valid:              Bool,
    e0Prn:                UInt,
    e0LdqIdx:             UInt,
    e0ByteValid:          UInt,
    e0OwnBytes:           UInt,
    e0ActiveBytes:        UInt,
    e1Valid:              Bool,
    e1Prn:                UInt,
    e1LdqIdx:             UInt,
    killHere0:            Bool,
    wrValid:              Bool,
    wrMask:               UInt,
    wrWritten:            Bool,
    wrActiveCovered:      Bool,
    wrPreloadPending:     Bool,
    wrActiveBytes:        UInt,
    wrKilled:             Bool,
    staleReqValid:        Bool,
    preloadUndisturbed:   Bool,
    groupDoneValid:       Bool,
    groupDoneMembers:     UInt,
    winnerActiveCovered:  Bool,
    groupCountWinner:     UInt,
    pvdestAtWinnerMember: UInt,
    winnerPrn:            UInt,
    elemDoneValid:        Bool,
    elemDoneNelem:        UInt,
    vlWbValid:            Bool
  ): Unit = {

    //@formal-req-spec-lsu.e5
    AssertProperty(
      respValid |-> Sequence.BoolSequence(respDstByte < vLenBytesLit),
      label = Some("vec_lcb_resp_byte_offset_in_entry")
    )

    // Antecedent reachability for vec_lcb_resp_byte_offset_in_entry and
    // vec_lcb_resp_matches_at_most_one_entry (formal-lsu.e6).
    CoverProperty(respValid, label = Some("vec_lcb_resp_seen"))

    //@formal-req-spec-lsu.e4
    AssertProperty(
      respValid |-> Sequence.BoolSequence(PopCount(hitVec) <= 1.U),
      label = Some("vec_lcb_resp_matches_at_most_one_entry")
    )

    //@formal-req-spec-lsu.e20
    AssertProperty(
      e0Valid |-> Sequence.BoolSequence((e0ByteValid & ~e0OwnBytes) === 0.U),
      label = Some("vec_lcb_byte_valid_within_own_bytes")
    )

    // Antecedent reachability for vec_lcb_byte_valid_within_own_bytes and
    // vec_lcb_active_within_own_bytes (formal-lsu.e9).
    CoverProperty(e0Valid, label = Some("vec_lcb_entry_live_seen"))

    //@formal-req-spec-lsu.e21
    //@formal-req-spec-lsu.e22
    //@formal-req-spec-lsu.e23
    AssertProperty(
      e0Valid |-> Sequence.BoolSequence((e0ActiveBytes & ~e0OwnBytes) === 0.U),
      label = Some("vec_lcb_active_within_own_bytes")
    )

    //@formal-req-spec-lsu.e6
    AssertProperty(
      wrValid |-> wrActiveCovered,
      label = Some("vec_lcb_write_needs_active_bytes_present")
    )

    // Antecedent reachability for vec_lcb_write_needs_active_bytes_present,
    // vec_lcb_write_only_once_per_entry, vec_lcb_write_after_preload,
    // vec_lcb_write_mask_covers_active, and vec_lcb_killed_entry_never_written
    // (formal-lsu.e12).
    CoverProperty(wrValid, label = Some("vec_lcb_vrf_write_seen"))

    //@formal-req-spec-lsu.e19
    AssertProperty(
      wrValid |-> Sequence.BoolSequence(!wrWritten),
      label = Some("vec_lcb_write_only_once_per_entry")
    )

    //@formal-req-spec-lsu.e9
    //@formal-req-spec-lsu.e10
    AssertProperty(
      wrValid |-> Sequence.BoolSequence(!wrPreloadPending),
      label = Some("vec_lcb_write_after_preload")
    )

    //@formal-req-spec-lsu.e8
    AssertProperty(
      wrValid |-> Sequence.BoolSequence((wrMask & wrActiveBytes) === wrActiveBytes),
      label = Some("vec_lcb_write_mask_covers_active")
    )

    //@formal-req-spec-lsu.e11
    AssertProperty(
      staleReqValid |-> preloadUndisturbed,
      label = Some("vec_lcb_preload_only_undisturbed")
    )

    // Antecedent reachability for vec_lcb_preload_only_undisturbed (formal-lsu.e16).
    CoverProperty(staleReqValid, label = Some("vec_lcb_preload_request_seen"))

    //@formal-req-spec-lsu.e15
    AssertProperty(
      groupDoneValid |-> winnerActiveCovered,
      label = Some("vec_lcb_group_done_last_member_complete")
    )

    // Antecedent reachability for vec_lcb_group_done_last_member_complete,
    // vec_lcb_group_done_count_matches_target, and
    // vec_lcb_group_done_pvdest_matches_entry (formal-lsu.e19).
    CoverProperty(groupDoneValid, label = Some("vec_lcb_group_done_seen"))

    //@formal-req-spec-lsu.e13
    //@formal-req-spec-lsu.e14
    AssertProperty(
      groupDoneValid |-> Sequence.BoolSequence(groupCountWinner === groupDoneMembers),
      label = Some("vec_lcb_group_done_count_matches_target")
    )

    //@formal-req-spec-lsu.e16
    //@formal-req-spec-lsu.l1
    //@formal-req-spec-lsu.l3
    //@formal-req-spec-lsu.l4
    //@formal-req-spec-lsu.l8
    //@formal-req-spec-lsu.l9
    AssertProperty(
      groupDoneValid |-> Sequence.BoolSequence(pvdestAtWinnerMember === winnerPrn),
      label = Some("vec_lcb_group_done_pvdest_matches_entry")
    )

    //@formal-req-spec-lsu.e2
    AssertProperty(
      Sequence.BoolSequence(e0Valid && e1Valid) |-> Sequence.BoolSequence((e0Prn =/= e1Prn) || (e0LdqIdx =/= e1LdqIdx)),
      label = Some("vec_lcb_no_duplicate_prn")
    )

    // Antecedent reachability for vec_lcb_no_duplicate_prn (formal-lsu.e23).
    CoverProperty(e0Valid && e1Valid, label = Some("vec_lcb_two_entries_live_seen"))

    //@formal-req-spec-lsu.f15
    //@formal-req-spec-lsu.i8
    AssertProperty(
      wrValid |-> Sequence.BoolSequence(!wrKilled),
      label = Some("vec_lcb_killed_entry_never_written")
    )

    //@formal-req-spec-lsu.f14
    //@formal-req-spec-lsu.i7
    AssertProperty(
      killHere0 |=> Sequence.BoolSequence(!e0Valid),
      label = Some("vec_lcb_kill_clears_entry")
    )

    // Antecedent reachability for vec_lcb_kill_clears_entry (formal-lsu.f3).
    CoverProperty(killHere0, label = Some("vec_lcb_entry_killed_seen"))

    //@formal-req-spec-lsu.g7
    //@formal-req-spec-lsu.g6
    AssertProperty(
      vlWbValid |-> groupDoneValid,
      label = Some("vec_lcb_vl_wb_with_group_done")
    )

    // Antecedent reachability for vec_lcb_vl_wb_with_group_done (formal-lsu.g2).
    CoverProperty(vlWbValid, label = Some("vec_lcb_vl_wb_seen"))

    //@formal-req-spec-lsu.j4
    AssertProperty(
      elemDoneValid |-> Sequence.BoolSequence(elemDoneNelem =/= 0.U),
      label = Some("vec_lcb_elem_done_nonzero_elements")
    )

    // Antecedent reachability for vec_lcb_elem_done_nonzero_elements (formal-lsu.j16).
    CoverProperty(elemDoneValid, label = Some("vec_lcb_elem_done_seen"))
  }
}

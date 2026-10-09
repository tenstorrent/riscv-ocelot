// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.lsu.VecDcacheArbiter]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in
  * VecDcacheArbiter.scala; firtool emits these into their own .sv and a
  * SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/formal/formal-lsu.yaml.
  */
object VecDcacheArbiterChecks {
  def apply(
    ldReqValid:   Bool,
    ldReqDcache:  Bool,
    ldWins:       Bool,
    stWins:       Bool,
    ldWinsElev:   Bool,
    stWinsElev:   Bool,
    ldEligible:   Bool,
    vecFireValid: Bool,
    claimTlb:     Bool,
    claimDcache:  Bool,
    claimLcam:    Bool,
    availDcache:  Bool,
    rrPtr:        UInt,
    blockedLd:    Bool,
    dmemReady:    Bool,
    lcbFree:      UInt
  ): Unit = {

    //@formal-req-spec-lsu.e24
    AssertProperty(
      ldWins |-> Sequence.BoolSequence(lcbFree =/= 0.U),
      label = Some("vec_dcache_arb_load_grant_has_lcb_credit")
    )

    // Antecedent reachability for vec_dcache_arb_load_grant_has_lcb_credit (formal-lsu.e1).
    CoverProperty(ldWins, label = Some("vec_dcache_arb_load_grant_seen"))

    //@formal-req-spec-lsu.e25
    AssertProperty(
      Sequence.BoolSequence(ldReqValid && (lcbFree === 0.U)) |-> Sequence.BoolSequence(!ldWins),
      label = Some("vec_dcache_arb_load_stalls_without_lcb_entry")
    )

    // Antecedent reachability for vec_dcache_arb_load_stalls_without_lcb_entry (formal-lsu.e3).
    CoverProperty(
      ldReqValid && (lcbFree === 0.U),
      label = Some("vec_dcache_arb_load_pending_without_credit_seen")
    )

    //@formal-req-spec-lsu.h1
    //@formal-req-spec-lsu.k5
    //@formal-req-spec-lsu.a11
    AssertProperty(
      Sequence.BoolSequence(!(ldWins && stWins)),
      label = Some("vec_dcache_arb_lane_exclusive")
    )

    //@formal-req-spec-lsu.h3
    AssertProperty(
      Sequence.BoolSequence(rrPtr === 0.U) |=> Sequence.BoolSequence(rrPtr === 1.U),
      label = Some("vec_dcache_arb_rr_pointer_rotates")
    )

    // Antecedent reachability for vec_dcache_arb_rr_pointer_rotates (formal-lsu.h2).
    CoverProperty(rrPtr === 0.U, label = Some("vec_dcache_arb_phase_zero_seen"))

    //@formal-req-spec-lsu.h4
    AssertProperty(
      Sequence.BoolSequence(ldWins && !ldWinsElev && ldReqDcache) |-> availDcache,
      label = Some("vec_dcache_arb_default_grant_respects_scalar")
    )

    // Antecedent reachability for vec_dcache_arb_default_grant_respects_scalar (formal-lsu.h4).
    CoverProperty(
      ldWins && !ldWinsElev && ldReqDcache,
      label = Some("vec_dcache_arb_default_load_dcache_grant_seen")
    )

    //@formal-req-spec-lsu.h6
    AssertProperty(
      Sequence.BoolSequence((rrPtr === 1.U) && blockedLd && ldEligible && (!ldReqDcache || dmemReady)) |-> ldWins,
      label = Some("vec_dcache_arb_elevation_grants_blocked_load")
    )

    // Antecedent reachability for vec_dcache_arb_elevation_grants_blocked_load (formal-lsu.h6).
    CoverProperty(
      (rrPtr === 1.U) && blockedLd && ldEligible,
      label = Some("vec_dcache_arb_load_elevation_phase_seen")
    )

    //@formal-req-spec-lsu.h9
    //@formal-req-spec-lsu.h5
    //@formal-req-spec-lsu.h11
    AssertProperty(
      Sequence.BoolSequence(claimTlb || claimDcache || claimLcam) |-> Sequence.BoolSequence(ldWinsElev || stWinsElev),
      label = Some("vec_dcache_arb_claim_only_on_elevation")
    )

    // Antecedent reachability for vec_dcache_arb_claim_only_on_elevation (formal-lsu.h8)
    // and vec_dcache_arb_claim_only_in_elevation_phase (formal-lsu.h10).
    CoverProperty(
      claimTlb || claimDcache || claimLcam,
      label = Some("vec_dcache_arb_resource_claim_seen")
    )

    //@formal-req-spec-lsu.h10
    AssertProperty(
      Sequence.BoolSequence(claimTlb || claimDcache || claimLcam) |-> Sequence.BoolSequence((rrPtr === 1.U) || (rrPtr === 2.U)),
      label = Some("vec_dcache_arb_claim_only_in_elevation_phase")
    )

    //@formal-req-spec-lsu.h8
    //@formal-req-spec-lsu.h7
    AssertProperty(
      Sequence.BoolSequence(ldWins || stWins) |-> vecFireValid,
      label = Some("vec_dcache_arb_grant_presents_fire")
    )

    // Antecedent reachability for vec_dcache_arb_grant_presents_fire (formal-lsu.h11).
    CoverProperty(ldWins || stWins, label = Some("vec_dcache_arb_any_grant_seen"))
  }
}

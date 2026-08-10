/*
Tenstorrent CONFIDENTIAL
__________________
 Tenstorrent Inc.
 All Rights Reserved.

NOTICE:  All information contained herein is, and remains
the property of Tenstorrent Inc.  The intellectual
and technical concepts contained
herein are proprietary to Tenstorrent Inc.
and may be covered by U.S., Canadian and Foreign Patents,
patents in process, and are protected by trade secret or copyright law.
Dissemination of this information or reproduction of this material
is strictly forbidden unless prior written permission is obtained
from Tenstorrent Inc.
*/

package boom.v4.vec.generated.issue

import chisel3._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.vec.generated.VecTrace

// GENERATED from src/main/nlhdl/vec/issue/VecStoreDgenPath.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
//
// VecStoreDgenPath — the vector STORE slot's second grant path: the AGEN/DGEN
// path-sequencing rule plus the `is_shared` store-data operand mux.
//
// Exists ONLY in `iq_v_store` slots (count zero elsewhere) — see
// hierarchy.yaml's `dgen_path` instantiation note. Elaborated only when
// `usingRVV` (a Scala Boolean, never rocket's `usingVector`) is true, by
// virtue of VecIssueSlot's own instance-count expression; this module
// performs no internal usingRVV gating of its own.
//
// ===> THE DGEN GATED OPERAND IS SELECTED BY `is_shared`, AND GETTING THIS
//      WRONG IS ONE OF THE MILESTONE-1 BUGS. The gated operand is `pvtmp`
//      when `is_shared` is set and `pvs3` otherwise. Gating DGEN
//      unconditionally on `pvs3` reads the wrong group for a segmented store
//      and DEADLOCKS the six-step chain: for such an op `pvs3` is the
//      COPROCESSOR half's source group, ready long before the transpose has
//      run, while the group the LSU half must read (`pvtmp`) is written *by*
//      that coprocessor half.
//
// ===> THE TWO PATHS ARE NEVER ONE GRANT. AGEN is granted first, DGEN
//      second, and the gap between them is unbounded: for a segmented store
//      DGEN cannot fire until the coprocessor half has written `pvtmp`.
//
// This module is COMBINATIONAL. It declares no register and no counter — the
// two-bit "still outstanding" encoding lives in the migrating `MicroOp`'s
// `fu_code(FC_AGEN)`/`fu_code(FC_DGEN)` pair, not in a register private to
// this module, because the vector issue queues are age-ordered COLLAPSING
// queues and a private register here would be left behind whenever a uop
// migrates between slot instances.
//
// Deliberately no `busy` output toward an issue unit and no port reaching the
// vector LSU: this module observes grants only and never learns whether the
// LSU has drained anything (ground rule 6).
//
// Governing spec anchors: issue.rst `shared-store-chain`,
// issue.rst `issue-sched-stage` ("The Vector Issue Slot"),
// issue.rst `cii-shared-sched` ("Segmented Store"), cii.rst `cii-segmented`,
// midcore.rst `midcore-segmented-store` and `vrf-ports`,
// glossary.rst `glossary-terms` ("Scheduler").

class VecStoreDgenPathIO(implicit p: Parameters) extends BoomBundle
{
  // Type-only helper, kept SEPARATE from the `slot_uop` port instance below
  // (rather than deriving the output types from `slot_uop` itself) so that
  // wrapping `slot_uop` in `Input(...)` can never be suspected of mutating
  // the template this module reads widths from. Every width below is taken
  // from the corresponding MicroOp field's own Chisel type (nlhdl
  // `parameters` section), never a literal re-derived from
  // `maxVecMembers`/`vecPregSz` here, so the operand mux ports cannot
  // desynchronise from a future change to MicroOp's own group sizing.
  // `uopT` is `private`, so it contributes no Bundle element of its own —
  // only `slot_uop` below (and the two derived output types) are actual
  // ports.
  private val uopT = new MicroOp

  // ---- Inputs from the slot ----
  val slot_valid          = Input(Bool())
  val grant                = Input(Bool())
  val squash_grant         = Input(Bool())
  val slot_uop             = Input(new MicroOp)
  // Readiness for the ADDRESS path ONLY (base GPR, VL, index group, mask);
  // must not include the store-data operand (logic part 5 / req c13).
  val agen_operands_ready  = Input(Bool())
  // Group-ready for the operand THIS module selected, from the slot's
  // `rdy_vs3` VecGroupReady instance.
  val dgen_operand_ready   = Input(Bool())
  // The AGEN path's scalar feeder was re-busied by a speculative-wakeup
  // retraction (the slot's `rebusied_prs1` equivalent).
  val agen_rebusied        = Input(Bool())

  // ---- Outputs to the slot: the operand mux ----
  // `.cloneType`, NOT `chiselTypeOf`. Both express "the same type as MicroOp's
  // field", which is the point of `uopT` above -- but `chiselTypeOf` requires
  // its argument to be HARDWARE, and `uopT` is deliberately a bare Chisel type
  // (`new MicroOp`, wrapped in no Wire/IO), so it throws
  // `ExpectedHardwareException: 'UInt<7>[8]' must be hardware` at elaboration.
  // `.cloneType` is the bare-type form and keeps MicroOp as the single source
  // of the group sizing. Do NOT "fix" this by restating
  // `Vec(maxVecMembers, UInt(vecPregSz.W))` here -- that reintroduces exactly
  // the desynchronisation the comment above exists to prevent.
  val dgen_operand          = Output(uopT.pvs3.get.cloneType)
  val dgen_operand_members  = Output(uopT.v_emul.get.cloneType)
  val dgen_operand_busy     = Output(Bool())
  val dgen_operand_is_pvtmp = Output(Bool())

  // ---- Outputs to the slot: path sequencing ----
  val agen_request         = Output(Bool())
  val dgen_request         = Output(Bool())
  val iss_fu_code_agen     = Output(Bool())
  val iss_fu_code_dgen     = Output(Bool())
  val next_fu_code_agen    = Output(Bool())
  val next_fu_code_dgen    = Output(Bool())
  val issued_partial_agen  = Output(Bool())
  val issued_partial_dgen  = Output(Bool())
  val keep_valid           = Output(Bool())
  val both_paths_done      = Output(Bool())

  // Deliberately absent: any VecGroupDone port or wakeup-port array (the
  // per-member match belongs to the slot's VecGroupReady instance), any
  // `busy` output toward an issue unit, and any port reaching the vector LSU.
}

/**
 * VecStoreDgenPath — see the file header for the full design rationale.
 * Instantiated once per store-capable slot by `VecIssueSlot`, as `dgen_path`.
 */
class VecStoreDgenPath(implicit p: Parameters) extends BoomModule
{
  val io = IO(new VecStoreDgenPathIO)

  val uop = io.slot_uop

  // =========================================================================
  // ---- 1. Where the two grant bits live, and why not here ----
  // =========================================================================
  //
  //@req-spec-core.f13
  //@req-spec-issue.d10
  //@req-spec-issue.d13
  // The two independently-grantable paths are tracked as the two-bit "still
  // outstanding" encoding held in the pair `fu_code(FC_AGEN)`/
  // `fu_code(FC_DGEN)` of the slot's own uop, exactly as baseline BOOM's mem
  // slot already does it: both bits set means neither path is granted yet,
  // the DGEN bit alone means AGEN is granted and DGEN outstanding, neither
  // means both are granted and the entry retires, and the AGEN bit alone is
  // unreachable for a vector store (part 2 below; asserted in part 8).
  val fu_agen = uop.fu_code(FC_AGEN)
  val fu_dgen = uop.fu_code(FC_DGEN)

  // =========================================================================
  // ---- 2. Offer order: AGEN strictly before DGEN ----
  // =========================================================================
  //
  //@req-spec-issue.d11
  //@req-spec-issue.c13
  // When both bits are set, this module offers the ADDRESS path and only
  // that path: `dgen_request` (part 3) is gated on `!fu_agen`, so it is held
  // false while the AGEN bit is still set even if the store data is already
  // ready. `agen_request` is a function of `agen_operands_ready` ALONE — no
  // term here reaches `dgen_operand_ready`/`dgen_operand_busy` — which is
  // exactly what keeps a `pvtmp` group-done (part 5) from ever waking the
  // AGEN path (req c13).
  val agen_request = io.slot_valid && !uop.iw_issued && fu_agen && io.agen_operands_ready

  // =========================================================================
  // ---- 3. What DGEN is gated on ----
  // =========================================================================
  //
  //@req-spec-issue.g22
  //@req-spec-issue.g23
  //@req-spec-issue.g24
  //@req-spec-issue.c12
  //@req-spec-cii.i12
  // The gating term is the VECTOR store-data operand's group-ready bit
  // (`dgen_operand_ready`, matched on the vector wakeup network through the
  // slot's `rdy_vs3` matcher against whichever group the part-4 mux
  // selected) and its busy bit (`dgen_operand_busy`, same mux) — NEVER
  // `prs2_busy`. A vector store's data is a group of up to EMUL member PRNs
  // in the VRF, read at execute on VRF port R3, not one scalar register, so
  // `prs2` plays no part here. For a segmented store this is exactly what
  // makes the DGEN path grantable once the coprocessor half's `pvtmp`
  // group-done clears the selected group's busy bit (req c12, cii.i12) —
  // step 2 of the segmented-store wakeup list and the only wakeup this slot
  // receives from the coprocessor half.
  //
  // Baseline's scalar DGEN operand rewrite (`io.iss_uop.prs1 :=
  // slot_uop.prs2`) has no counterpart here and must not be re-added: vector
  // store data is not delivered through a scalar operand slot.
  val dgen_request = io.slot_valid && !uop.iw_issued && fu_dgen && !fu_agen &&
    !io.dgen_operand_busy && io.dgen_operand_ready

  // =========================================================================
  // ---- 4. The operand mux — the M1 bug ----
  // =========================================================================
  //
  //@req-spec-issue.g25
  //@req-spec-issue.g26
  // The gated operand is selected by `is_shared`, and the same select drives
  // every companion output, so the member PRNs, the busy bit and the mux
  // select itself always agree on which group is live.
  //
  //@req-spec-issue.g27
  //@req-spec-issue.g28
  //@req-spec-cii.i6
  /* WARNING — do not "simplify" this to pvs3. Gating DGEN unconditionally on
     pvs3 is incorrect for a segmented store and earlier Caracal drafts did
     exactly that. For such an op pvs3 is the COPROCESSOR half's source group
     and is ready long before the transpose has run, while the LSU half's
     data source is pvtmp, written BY the coprocessor half. A DGEN woken on
     pvs3 reads pvtmp before it exists and stores garbage; and pvs3's
     group-done never arrives at this slot at all, so the chain deadlocks.
     Whenever is_shared is set, DGEN wakes on pvtmp's group-done and on
     nothing else. */
  io.dgen_operand          := Mux(uop.is_shared.get, uop.pvtmp.get, uop.pvs3.get)
  io.dgen_operand_busy     := Mux(uop.is_shared.get, uop.pvtmp_busy.get, uop.pvs3_busy.get)
  io.dgen_operand_is_pvtmp := uop.is_shared.get
  // `v_emul` is a single MicroOp field shared by every vector operand of this
  // uop (there is no separate per-group EMUL), so the member count already
  // comes from the same group as the muxed member PRNs without a select of
  // its own.
  io.dgen_operand_members  := uop.v_emul.get

  // `pvs3` and `pvtmp` are not merged and not reinterpreted above: both stay
  // intact on the issued uop, so the downstream store-data read must repeat
  // this exact selection (via `is_shared`) when it addresses VRF port R3 —
  // reading `uop.pvs3` unconditionally at that later site reintroduces this
  // bug one stage further downstream.

  io.agen_request := agen_request
  io.dgen_request := dgen_request

  // =========================================================================
  // ---- Path naming for the issued uop ----
  // =========================================================================
  //
  //@req-spec-issue.d11
  // `iss_fu_code_agen`/`iss_fu_code_dgen` name exactly the path offered THIS
  // cycle — at most one, per parts 2/3 above — so the slot's `iss_uop.fu_code`
  // override always advertises the single path being granted, never both.
  io.iss_fu_code_agen := agen_request
  io.iss_fu_code_dgen := dgen_request

  // =========================================================================
  // ---- 6. Capturing a grant ----
  // =========================================================================
  //
  //@req-spec-issue.d13
  // On `grant && !squash_grant` exactly one partial marker is set, chosen by
  // the path offered this cycle. A squashed grant (`squash_grant`) advances
  // neither marker, so a squashed AGEN or DGEN grant leaves the encoding
  // untouched and the path re-offers.
  val grantedThisCycle = io.grant && !io.squash_grant
  io.issued_partial_agen := grantedThisCycle && agen_request
  io.issued_partial_dgen := grantedThisCycle && dgen_request

  // The encoding update happens the FOLLOWING cycle off that marker, riding
  // in `slot_uop.iw_issued_partial_agen`/`_dgen` (the same migrating MicroOp
  // fields the scalar mem slot already uses), mirroring baseline
  // (issue-slot.scala's `isMem` block):
  //
  //   AGEN marker set, agen_rebusied false:
  //       next_fu_code_agen := false, next_fu_code_dgen := true
  //       keep_valid        := true          // the slot survives the grant
  //   AGEN marker set, agen_rebusied true:
  //       encoding left unchanged, so AGEN re-offers next cycle
  //   DGEN marker set:
  //       both_paths_done := true; the slot retires normally
  //
  // The DGEN side has no re-busy term at all (asymmetric by design): vector
  // operands are never woken speculatively, so a DGEN grant can never be
  // retracted.
  val agenMarker = uop.iw_issued_partial_agen
  val dgenMarker = uop.iw_issued_partial_dgen

  val next_fu_code_agen_w = WireInit(fu_agen)
  val next_fu_code_dgen_w = WireInit(fu_dgen)
  //@req-spec-issue.d13
  when (agenMarker && !io.agen_rebusied) {
    next_fu_code_agen_w := false.B
    next_fu_code_dgen_w := true.B
  }
  // agenMarker && agen_rebusied: no assignment — next_fu_code_* hold their
  // WireInit default (the CURRENT fu_code pair), so AGEN re-offers next
  // cycle exactly as the pseudocode above specifies. dgenMarker: likewise no
  // assignment here — fu_code no longer matters once the entry retires
  // (`both_paths_done` below), and baseline's own code takes the same "leave
  // it, the slot is going away" stance for the terminal grant.
  io.next_fu_code_agen := next_fu_code_agen_w
  io.next_fu_code_dgen := next_fu_code_dgen_w

  // ASSUMPTION (spec silent on this exact point): `keep_valid` is asserted
  // whenever the AGEN marker fired, in BOTH the rebusied and non-rebusied
  // sub-cases — the pseudocode above only annotates `keep_valid := true` on
  // the non-rebusied line, but baseline's analogous code
  // (`next_valid := true.B` under `iw_issued_partial_agen`, before it
  // separately decides whether to also roll the encoding) forces validity
  // unconditionally on that marker; a retracted AGEN grant still needs the
  // slot to persist so AGEN can re-offer. `both_paths_done` (dgenMarker)
  // deliberately does NOT set `keep_valid`: that is the one entry point
  // where this module intends the slot to retire via its normal `iw_issued`
  // path instead (unlike baseline's scalar mem slot, which forces validity
  // on its second grant too — the deliberate divergence part 2 calls out).
  io.keep_valid      := agenMarker
  io.both_paths_done := dgenMarker

  // =========================================================================
  // ---- 7. Tolerating an unbounded AGEN-to-DGEN gap ----
  // =========================================================================
  //
  //@req-spec-issue.d12
  // Nothing here measures, bounds or times out the interval between the two
  // grants: no counter, no shift register, no watchdog. The DGEN-outstanding
  // encoding above is level-held in the migrating uop's `fu_code` pair and is
  // read fresh, combinationally, every cycle — equally valid one cycle or ten
  // thousand cycles after the AGEN grant. This module cannot poll and has no
  // path to the coprocessor or the LSU; it only ever reacts to
  // `dgen_operand_ready` changing on the vector wakeup network like any
  // other consumer. Kill/flush/squash need nothing extra from it either: the
  // two bits die with the uop in the slot's own registers (not this
  // module's), and `squash_grant` (part 6) is honoured on both paths.

  // =========================================================================
  // ---- 8. Assertions (synthesizable, on hardware conditions) ----
  // =========================================================================

  assert(!(io.issued_partial_agen && io.issued_partial_dgen),
    "VecStoreDgenPath: both partial markers set in the same cycle")
  assert(!(io.agen_request && io.dgen_request),
    "VecStoreDgenPath: both paths offered in the same cycle")
  //@req-spec-issue.c13
  assert(!(io.dgen_request && fu_agen),
    "VecStoreDgenPath: DGEN offered while AGEN bit still set")
  assert(!(io.issued_partial_dgen && fu_agen),
    "VecStoreDgenPath: DGEN granted while AGEN bit still set")
  assert(!io.agen_request || io.agen_operands_ready,
    "VecStoreDgenPath: agen_request asserted without agen_operands_ready")
  assert(!(io.slot_valid && !uop.iw_issued && !fu_agen && !fu_dgen),
    "VecStoreDgenPath: a live, un-issued store slot advertises neither FC_AGEN nor FC_DGEN")
  assert(!io.dgen_request || (io.dgen_operand_members =/= 0.U),
    "VecStoreDgenPath: dgen_request asserted against an empty operand group")
  assert(!(io.grant && !io.slot_valid),
    "VecStoreDgenPath: grant asserted against an invalid slot")

  // =========================================================================
  // ---- 9. Tracing ----
  // =========================================================================
  //
  // Guarded via the shared VecTrace package, gated on the `vecTrace` plusarg
  // (off by default) and tagged with this module's name and `rob_idx`.
  when (io.issued_partial_agen) {
    VecTrace.trace("VecStoreDgenPath", "agen_grant", uop)
  }
  when (io.issued_partial_dgen) {
    VecTrace.trace("VecStoreDgenPath", "dgen_grant", uop)
  }
  // SPEC DEFECT (reported, not resolved) -- the third event the nlhdl logic
  // section (part 9) calls for, `dgen_operand_select`, is specified to fire
  // "on the cycle the slot is filled", carrying `is_shared` and which of
  // pvtmp/pvs3 was chosen. No port on this module carries a "slot filled"
  // pulse (the slot's dispatch-time fill event is not part of this
  // interface), and detecting the slot_valid rising edge locally would
  // require a register -- explicitly forbidden by this module's own logic
  // section preamble ("This module is COMBINATIONAL. It declares no register
  // and no counter") and by ground rule 6 (no per-instruction state outside
  // the six VecElemQueue instances / the LCB / VecLsu's descriptor table).
  // Emitting it on some other, always-available condition (e.g. every cycle
  // `agen_request` holds) would not be "once per fill" and would misrepresent
  // the event this line exists to make visible. This event is therefore
  // OMITTED rather than approximated; the choice it would have logged
  // (`is_shared`, `dgen_operand_is_pvtmp`) is still visible in the
  // `dgen_grant` line above once DGEN is actually granted, just not at fill
  // time as the spec intended.
}

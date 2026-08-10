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
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common.{BoomBundle, BoomModule}
import boom.v4.vec.generated.VecGroupDone

// GENERATED from src/main/nlhdl/vec/issue/VecGroupReady.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
//
// VecGroupReady — the per-source-group readiness matcher of a vector issue
// slot: it AND-reduces one vector source operand's per-member wakeup state
// into the single group-ready bit that operand contributes to `request`.
//
// ONE definition, FIVE instances in an `IQ_V_LOAD` or `IQ_V_ALU` slot and FOUR
// in an `IQ_V_STORE` slot — `rdy_vs1`, `rdy_vs2`, `rdy_vs3` and (load/ALU only)
// `rdy_vold`, all `isMask = false`, plus `rdy_vm` (`isMask = true`). Split out
// of VecIssueSlot because the same per-member match was being restated once
// per operand; the four non-mask instances differ in nothing.
//
// ===> `rdy_vold` (decision D6) MATCHES THE `stale_pvdest` GROUP, and it is an
// ORDINARY instance of this module with no new port and no new behaviour —
// that is the point of the decision. `stale_pvdest` is the PREVIOUS mapping of
// the destination arch vregs, so its producer is an OLDER instruction, and
// age-ordered issue grants the oldest READY entry, which does not guarantee an
// older producer has completed. Two consumers read the group anyway: the LCB
// pre-loads inactive-lane data from it on VRF port R2 for a vta=0/vma=0 load,
// and the coprocessor may pull it as the STALE_VD source slot. Without this
// instance both can read a BUSY group and get garbage. An aggregate
// `stale_pvdest_busy` bit could not have done the job — see the sub-range
// note below — which is why the fifth instance is the fix.
//
// ===> WHY THIS IS A SEPARATE MODULE FROM THE SLOT'S OTHER COMPARATORS: a
// POLICY split, not a tidiness one. Vector operands wake ONLY on actual
// completion, never speculatively. The slot's scalar feeders — base address,
// stride and the .vx operand on the INT network, the .vf scalar on FP — do
// ride BOOM's existing speculative load-hit wakeup unchanged, and so need the
// re-busy and replay machinery that comes with it. Nothing here may ever gain
// a speculative-wakeup input or a re-busy path.
//
// THE MATCH IS PER MEMBER, NOT A SINGLE BASE COMPARATOR. A consumer may read a
// SUB-RANGE of a producer's in-flight group, or a group fragmented across two
// producers, so base-PRN equality is not merely pessimistic — it never
// matches at all. An LMUL=8 write to v0..v7 followed by an LMUL=2 read at v4
// sources {p4, p5}; the producer's base is p0, so base equality never fires
// and the consumer hangs. The per-member match is the area/timing cost of the
// vector slot and it is not removable.
//
// ONE GROUP-DONE PER GROUP, EVER: no repeat, no retry. A match dropped for one
// cycle is a permanent hang, so the load path, the collapse-move path and the
// match are ONE next-state expression below, never sequenced.
//
// Governing spec anchors: issue.rst `issue-sched-stage` ("The Vector Issue
// Slot" and "The match-port budget"), midcore.rst `group-done` ("Busy Table")
// and `spec-wakeups` ("Speculative Wakeups").

/**
 * VecGroupReadyIO
 *
 * `isMask` collapses `groupMembers` to the elaboration-time constant 1 (a
 * mask is one physical register, never a group: `pvm` is a plain
 * `UInt(vecPregSz.W)` on the uop, not a `Vec`) and elides the `members` port
 * entirely, since there is no member count to bound. A Scala constructor
 * parameter, not a hardware mode bit, so the collapse happens at elaboration.
 *
 * `numVecWbPorts` is NOT a separate constructor parameter here: the nlhdl
 * source's parameters section states it must equal
 * `VectorParams.numVecWbPorts` and that "VecIssueUnit and VecIssueSlot bind
 * to the same field" — i.e. every binder reads the one VectorParams field
 * rather than each carrying its own copy of the same number. This module
 * already has `vectorParams` in scope via `HasBoomCoreParameters` (mixed
 * into BoomBundle/BoomModule), so it reads `vectorParams.numVecWbPorts`
 * directly instead of re-declaring it as a redundant knob that could disagree
 * with the field it is meant to bind to. ASSUMPTION, recorded here since the
 * nlhdl parameters section lists it as a "parameter" without being explicit
 * about constructor-arg vs. direct-read.
 */
class VecGroupReadyIO(val isMask: Boolean)(implicit p: Parameters) extends BoomBundle
{
  // `groupMembers` — derived, `if (isMask) 1 else maxVecMembers` (the
  // HasBoomCoreParameters re-export of VectorParams.maxMembers, per
  // parameters.scala; VecBundles and MicroOp both bind to this same name):
  // the member lanes this instance elaborates.
  private val groupMembers: Int = if (isMask) 1 else maxVecMembers

  // `load` — write this matcher's state this cycle. Asserted for BOTH reasons
  // a slot's occupant changes: a dispatch into an empty slot, and a collapse
  // move shifting the slot above down into this one. One port for both
  // because BOOM's age-ordered collapsing queue treats them identically.
  val load = Input(Bool())

  // `in_member_rdy` — the per-member state to load: on a dispatch, the
  // INVERSE of the vector Busy-Table's per-member source read for this
  // operand; on a collapse move, the upstream slot's `out_member_rdy` for the
  // same operand index.
  val in_member_rdy = Input(Vec(groupMembers, Bool()))

  //@req-spec-issue.g13
  // `prns` — this operand's member PRNs: `pvs1`/`pvs2`/`pvs3` of the resident
  // uop, `stale_pvdest` on the `rdy_vold` instance, or `pvm` in lane 0 when
  // `isMask`. Read COMBINATIONALLY from the enclosing slot's `slot_uop`
  // register (or its `in_uop` payload while `load` is high) — this module
  // keeps no copy of the PRN vector itself; only the per-member ready bits
  // below are state.
  val prns = Input(Vec(groupMembers, UInt(vecPregSz.W)))

  // `members` — this operand's EMUL as a 1..maxMembers count (`v_emul` of the
  // resident uop, or the same field on `stale_pvdest`'s owner for `rdy_vold`).
  // NOT elaborated when `isMask`: a mask is never a group, so there is no
  // member count to bound.
  val members = if (isMask) None else Some(Input(UInt((log2Ceil(maxVecMembers) + 1).W)))

  // `used` — does this operand participate in readiness this cycle; the slot
  // owns the decision (mask-vm-bit gating, is_shared, v_uses_vs*, stale-group
  // applicability), this module owns only the consequence (see `ready` below).
  val used = Input(Bool())

  //@req-spec-rename.g19
  //@req-spec-issue.g12
  //@req-spec-vrf.e8
  // `group_done` — the only wakeup this module listens to is the VECTOR
  // network: `pvs1`/`pvs2`/`pvs3`/`pvm`/`stale_pvdest` are matched here
  // against group-done events and never against the integer or FP networks.
  // Each valid port carries the completing group's FULL member-PRN vector
  // (`VecGroupDone.pvdest`, `Vec(maxVecMembers, UInt(vecPregSz.W))`) plus its
  // `members` count — VecBundles owns the bundle; only those two fields are
  // read by this module (`rob_idx` and `pvl` belong to the ROB busy-clear and
  // the VL network). Broadcast, unregistered, identical at every slot and
  // instance.
  val group_done = Input(Vec(vectorParams.numVecWbPorts, Valid(new VecGroupDone)))

  // `ready` — this operand's group-ready bit, one term of the slot's
  // `vector_operands_ready`, combinational and valid in the same cycle as the
  // group-done that completes the group.
  val ready = Output(Bool())

  // `out_member_rdy` — the NEXT-STATE per-member vector, exported so a
  // collapse move carries partial readiness to the slot below. Next state and
  // not the register value on purpose — a group-done landing in the cycle of
  // the move would otherwise be lost by both slots.
  val out_member_rdy = Output(Vec(groupMembers, Bool()))

  // THERE IS NO MicroOp PORT: hierarchy.yaml's depends_on omits MicroOp to
  // enforce it. A MicroOp port would be a wide bundle replicated up to five
  // times per slot in all three queues.
}

/**
 * VecGroupReady ("the per-source-group readiness matcher") — see the file
 * header for the full design rationale. Instantiated by VecIssueSlot, FIVE
 * times per IQ_V_LOAD/IQ_V_ALU slot and FOUR times per IQ_V_STORE slot.
 *
 * `isMask` — Boolean, default false. True for the single `rdy_vm` instance
 * only; see [[VecGroupReadyIO]].
 */
class VecGroupReady(isMask: Boolean = false)(implicit p: Parameters) extends BoomModule
{
  // `groupMembers` — mirrors the IO bundle's own derivation (see there); kept
  // as a second, identical elaboration-time computation rather than reaching
  // into the IO bundle's private field, since `io.in_member_rdy.length` etc.
  // already expose the same information structurally.
  private val groupMembers: Int = if (isMask) 1 else maxVecMembers

  // Checks that an isMask instance elaborates exactly one member lane — a
  // compile-time (Scala) check, not hardware, since isMask/groupMembers are
  // both elaboration-time constants and this can never be violated at run
  // time. Recorded per the nlhdl logic section's "checks, not behaviour" list.
  require(!isMask || groupMembers == 1,
    "VecGroupReady: an isMask instance must elaborate exactly one member lane")

  val io = IO(new VecGroupReadyIO(isMask))

  // =========================================================================
  // ---- What is held, and where ----
  // =========================================================================

  //@req-spec-issue.g14
  // What this module DOES hold is one register bit per member, `member_rdy`:
  // each member PRN has its own readiness bit rather than the group sharing
  // one. Only these `groupMembers` bits are state; the rest is combinational.
  // Reset to READY (see below) — see the reset note further down.
  //
  // The uop carries one AGGREGATED busy bit per operand (`pvs1_busy` and
  // friends) and no per-member vector, by MicroOp's design. That aggregate
  // cannot initialize these bits: a group whose members come from two
  // producers can have members 0..2 complete while member 3 is in flight,
  // and the aggregate then says only "not ready". Broadcasting it to all
  // members would make this matcher wait on group-dones for 0..2 that already
  // fired and will never fire again — a permanent hang. Hence the per-member
  // `in_member_rdy` port, driven from the same Busy-Table read the aggregate
  // is derived from.
  val member_rdy = RegInit(VecInit(Seq.fill(groupMembers)(true.B)))

  // =========================================================================
  // ---- The per-member match ----
  // =========================================================================

  //@req-spec-issue.g15
  //@req-spec-issue.g18
  // For each member lane `i` of this operand and each wakeup port `w`,
  // compare `prns(i)` against EVERY member slot `j` of
  // `group_done(w).bits.pvdest` (VecBundles' field name for the group-done's
  // member-PRN vector — the nlhdl body's pseudocode calls this field `prns`;
  // that is a naming mismatch against the real VecGroupDone interface, not a
  // second field, and is flagged as a spec defect in the final report), and
  // take the hit when the port is valid and `j` is within that port's
  // `members` count. `j` always ranges over `maxVecMembers` — the producer's
  // full group width — regardless of this instance's own `groupMembers`
  // (relevant for the `isMask` instance, whose single lane may still match
  // any member of an up-to-8-member producing group).
  //
  //@req-spec-issue.g33
  // Matched against ALL `vectorParams.numVecWbPorts` group-done ports EVERY
  // cycle — not fewer, and not one port per cycle in rotation: the network is
  // `numVecWbPorts` wide and completion is single-shot, so a port not
  // examined in the cycle it is valid is a lost, unrecoverable wakeup.
  //
  // A single base comparator (this operand's member 0 against the
  // group-done's member 0) cannot be substituted — see the file header.
  // Comparison is on the full `vecPregSz` bits of the PRN, never a group base,
  // a member index or an architectural register number.
  //
  // PERMITTED FACTORING (not implemented here, kept as a documented
  // alternative per the nlhdl perf section): OR the group-done ports' member
  // PRNs into one numVecPhysRegisters-wide one-hot "completing this cycle"
  // vector shared by every instance in a queue, and read member_hit(i) as an
  // indexed lookup of it at prns(i). Same function, trading the comparator
  // array below for one decode plus a wide mux per member. Not taken here:
  // the array form is the spec's primary statement and the factoring is an
  // escape valve "if that path fails timing", not a mandate.
  val member_hit = Wire(Vec(groupMembers, Bool()))
  for (i <- 0 until groupMembers) {
    member_hit(i) := (for {
      w <- 0 until vectorParams.numVecWbPorts
      j <- 0 until maxVecMembers
    } yield {
      io.group_done(w).valid &&
      (j.U < io.group_done(w).bits.members) &&
      (io.group_done(w).bits.pvdest(j) === io.prns(i))
    }).reduce(_ || _)
  }
  //@req-spec-rename.g20
  // Sub-range and fragmented source groups need no extra logic beyond the
  // per-member match above: a consumer reading a sub-range of a producer's
  // in-flight group wakes when that producer's group-done fires, because each
  // of the consumer's members appears in that event's member-PRN vector and
  // matches its own lane; a group fragmented across two producers is covered
  // identically, since each lane latches its own hit from whichever port and
  // cycle carries it, and the AND-reduce below waits for the later. This is
  // conservative but correct: the consumer may wake LATER than strictly
  // necessary (it waits on the whole producing group, not only the members it
  // reads), never earlier. This is also why a single aggregate
  // `stale_pvdest_busy` bit was rejected (D6) in favour of a fifth instance of
  // this very module for `rdy_vold`: a `stale_pvdest` group can span up to 8
  // producers, and one bit cannot express "waiting on producer 3 of 8".

  // Lanes `i >= members` hold DON'T-CARE PRNs (vector rename writes only the
  // group's first EMUL entries) and are folded into `group_all_rdy` below via
  // the `i.U >= members` disjunct rather than here, so `member_hit` itself
  // stays a pure function of the group-done network.

  // =========================================================================
  // ---- Next state: load, collapse move and match compose in one cycle ----
  // =========================================================================

  //@req-spec-rename.g14
  //@req-spec-issue.g17
  // The next state and both outputs are one expression, so a load, a collapse
  // move and a match landing in the same cycle compose: the OR of
  // `member_hit` into the LOADED value is load-bearing, because a group-done
  // fires exactly once, so applying the match only to the already-registered
  // state would drop an event arriving in a dispatch or collapse-move cycle
  // and the consumer would never issue. Hence `ready`/`out_member_rdy` are
  // driven from this next-state wire, not from the `member_rdy` register.
  // Because the reduction below is an AND over all valid members, the operand
  // wakes only when its LAST member becomes ready — the group-ready bit rises
  // in the cycle the final outstanding member matches, not before, and that
  // holds even when several members complete in the same cycle on different
  // ports. A member's ready bit is STICKY: once set, only a `load` overwrites
  // it. There is no re-busy path.
  val member_rdy_next = Wire(Vec(groupMembers, Bool()))
  for (i <- 0 until groupMembers) {
    member_rdy_next(i) := Mux(io.load, io.in_member_rdy(i), member_rdy(i)) || member_hit(i)
  }
  member_rdy := member_rdy_next
  io.out_member_rdy := member_rdy_next

  // =========================================================================
  // ---- AND-reduce to one bit, and conditional participation ----
  // =========================================================================

  //@req-spec-rename.g13
  //@req-spec-issue.g16
  // The per-member bits are AND-ed into ONE group-ready bit for this operand.
  // No partial-readiness output and no per-member signal leaves this module
  // towards `request`; the slot sees one bit per operand. Lanes `i >= members`
  // are treated as ready regardless of their match state (don't-care PRNs,
  // see above); with `isMask` there is one lane and that disjunct elaborates
  // away entirely (no `members` port exists in that instance).
  //
  // `used` low forces `ready` high regardless of every member bit: this is
  // what makes the mask instance correct (an unmasked op's `pvm` names a
  // don't-care physical register), and is also how the slot drops a source
  // the instruction does not encode, drops `rdy_vs3` from the LSU half under
  // `is_shared`, and drops the stale group on `rdy_vold` when the OP.v has no
  // vector destination. This module decodes none of those conditions — it is
  // given the bit — but honours it by GATING, never by freezing the member
  // state: `member_rdy`/`member_rdy_next` keep updating while `used` is low,
  // so an operand that becomes used later cannot have missed an event.
  val group_all_rdy: Bool = if (isMask) {
    member_rdy_next(0)
  } else {
    (0 until groupMembers)
      .map(i => member_rdy_next(i) || (i.U >= io.members.get))
      .reduce(_ && _)
  }
  io.ready := !io.used || group_all_rdy

  // =========================================================================
  // ---- No speculative wakeup, therefore no re-busy (reject list) ----
  // =========================================================================

  //@req-spec-vrf.e7
  //@req-spec-vrf.e9
  // Every input above that can set a member bit (`group_done`) is an ACTUAL
  // COMPLETION — a group-done from a real writeback. Vector operands are
  // never woken speculatively: a vector-load producer completes through the
  // LCB after a long, variable, streaming latency and a vector-arithmetic
  // producer completes over the CII in program order, so neither has the
  // fixed short load-use latency that makes speculation profitable. Waking on
  // real writeback is also what lets this module own no re-busy or replay
  // machinery at all.
  //
  // REJECT LIST for this port set, deliberately absent above and forever:
  // no speculative-wakeup input, no load-hit or load-miss input, no re-busy /
  // clear-ready input, no brupdate or flush input (the slot's valid bit and
  // the queue's compaction handle a squash, and every new occupant arrives
  // through `load`, which overwrites all member bits), and — ground rule 6 —
  // no `busy` output of any kind: this module reads no `busy` from VecLsu and
  // exports none of its own. `ready`/`out_member_rdy` are the only outputs.

  // =========================================================================
  // ---- pvl is deliberately not matched here ----
  // =========================================================================

  //@req-spec-issue.g34
  //@req-spec-vrf.e10
  // The per-slot match budget is these instances — five in a load/ALU slot,
  // four in a store slot — PLUS the VL wakeup network for `pvl`, which is NOT
  // an instance of this module but a plain equality comparator per VL lane in
  // the enclosing slot's scalar-feeder set. `pvl` is one register in its own
  // register space with exactly one busy bit, so an instance here would spend
  // `maxVecMembers * numVecWbPorts` comparators to model one bit, and would
  // connect to the wrong network. The POLICY is nonetheless this module's and
  // not the scalar feeders': `pvl` wakes on ACTUAL COMPLETION on its own VL
  // network, not on the integer network and not speculatively — no VL
  // producer suits speculation, so the VL network needs no re-busy machinery
  // either. This module has no `pvl` port and never will.

  // =========================================================================
  // ---- Reset, assertions ----
  // =========================================================================

  // On synchronous active-high reset (the Chisel/BoomModule default, per
  // hierarchy.yaml's `defaults`), `member_rdy` is initialized above to READY.
  // The value is functionally irrelevant — `ready` is consumed only while the
  // slot is valid and a valid slot always arrived through `load` — and
  // ready-at-reset is chosen so an idle slot shows no phantom stall at time 0.

  // Assert that `members` is 1..maxVecMembers whenever the operand is used
  // (only meaningful, and only ported, when !isMask).
  if (!isMask) {
    assert(!io.used || (io.members.get >= 1.U && io.members.get <= maxVecMembers.U),
      "VecGroupReady: members out of range 1..maxVecMembers while the operand is used")
  }

  // Assert that group_done(w).bits.members is likewise in range on every
  // valid port.
  for (w <- 0 until vectorParams.numVecWbPorts) {
    assert(!io.group_done(w).valid ||
      (io.group_done(w).bits.members >= 1.U && io.group_done(w).bits.members <= maxVecMembers.U),
      "VecGroupReady: group_done member count out of range 1..maxVecMembers on a valid port")
  }

  // This module emits NO trace line of its own, deliberately: VecTrace's
  // helper requires a MicroOp so every line carries rob_idx, and adding one
  // here for debug would put a wide port on a module instantiated four or
  // five times per slot in all three queues. VecIssueSlot traces instead,
  // holding the uop — it watches the rising edge of each instance's `ready`
  // and emits one guarded line naming the operand and the exported
  // `out_member_rdy`. Same convention, same plusarg gate, off by default —
  // hence no `boom.v4.vec.generated.VecTrace` call site in this file, even
  // though hierarchy.yaml's depends_on lists it (a compile-order edge only).
}

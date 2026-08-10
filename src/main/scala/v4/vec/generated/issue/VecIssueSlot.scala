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

import boom.v4.common._
import boom.v4.exu.{BrUpdateInfo, Wakeup}
import boom.v4.util.{IsKilledByBranch, IsOlder, UpdateBrMask}
import boom.v4.vec.generated.{VecGroupDone, VecMemberRdy, VecTrace}

// GENERATED from src/main/nlhdl/vec/issue/VecIssueSlot.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
//
// VecIssueSlot -- one entry of a vector issue queue: a SUPERSET of BOOM v4's
// `IssueSlot` (src/main/scala/v4/exu/issue-units/issue-slot.scala) that tracks
// BOTH operand classes, scalar feeders and vector source groups, and asserts
// `request` only when every one of them is ready.
//
// It is a NEW module, not a delta on the scalar `IssueSlot` -- but it
// deliberately reuses that file's structure and names (`slot_valid`,
// `slot_uop`, `next_valid`, `next_uop`, `in_uop`/`out_uop`, `will_be_valid`,
// `iw_issued`, `killed`, `rebusied_prs1`/`rebusied_prs2`) so the two can be
// diffed. The scalar `IssueSlot` is untouched; only the IQ_V_* queues
// instantiate this module.
//
// ===> THE TWO OPERAND CLASSES HAVE DIFFERENT WAKEUP POLICIES, AND THAT IS
// THE WHOLE REASON THIS FILE HAS A CHILD MODULE (VecGroupReady). The scalar
// feeders ride BOOM's existing SPECULATIVE load-hit wakeup unchanged,
// re-busy machinery included; the vector source groups wake ONLY on actual
// completion. The per-member vector match lives in VecGroupReady, which owns
// no speculation input at all; the speculative comparators stay here.
//
// ===> THE SLOT MUST NOT CAPTURE THE VL VALUE. `pvl` is matched on the VL
// network as a plain readiness wakeup; the value is read from the VL RF at
// execute. `vtype` is not an operand at all (part 5).
//
// ===> NOTHING HERE MAY CONSULT A `busy` FROM THE VECTOR LSU (ground rule 6).
// This slot learns of progress only through group-done events on the vector
// wakeup network, exactly like every other consumer. No `busy` is exported
// by this module either.
//
// Governing spec anchors: issue.rst `issue-sched-stage` ("Wakeup Networks",
// "The Vector Issue Slot"), issue.rst `issue-vl-delivery`, issue.rst
// `cii-shared-sched`, midcore.rst `spec-wakeups`, `vl-vtype-rename`,
// `group-done-wb`, `midcore-segmented-load`, overview.rst `caracal-pipeline`,
// glossary.rst `glossary-terms`, frontend.rst `vl-delivery`.

/**
 * VecIssueSlotIO
 *
 * Baseline `IssueSlotIO`'s port list (`valid`, `will_be_valid`, `request`,
 * `grant`, `squash_grant`, `iss_uop`, `in_uop`, `out_uop`, `brupdate`,
 * `kill`, `clear`, `child_rebusys`) plus the vector additions. Deliberately
 * ABSENT, per issue.f15: `pred_wakeup_port`/`ppred`, any speculative-wakeup
 * or re-busy port on the vector side, any `busy` input from VecLsu or any
 * vector datapath, any `vtype` operand or wakeup port, and any VL VALUE
 * input.
 *
 * `numIntWakeupPorts` -- width of the INTEGER wakeup network (same value
 * baseline passes as `numWakeupPorts`). `numFpWakeupPorts` -- width of the FP
 * network; only meaningful (and only nonzero in a real build) when
 * `isAluSlot`. `pnrGate` -- true only for `IQ_V_ALU`; elaborates
 * `rob_pnr_idx`/`rob_head_idx`.
 */
class VecIssueSlotIO(
  val numIntWakeupPorts: Int,
  val numFpWakeupPorts:  Int,
  val isAluSlot:         Boolean,
  val pnrGate:           Boolean)(implicit p: Parameters) extends BoomBundle
{
  // ---- baseline, unchanged in name and meaning ----
  val valid         = Output(Bool())
  val will_be_valid = Output(Bool())
  val request       = Output(Bool())
  val grant         = Input(Bool())
  val squash_grant  = Input(Bool())
  val iss_uop       = Output(new MicroOp())
  val in_uop        = Input(Valid(new MicroOp()))
  val out_uop       = Output(new MicroOp())
  val brupdate      = Input(new BrUpdateInfo())
  val kill          = Input(Bool())
  val clear         = Input(Bool())
  val child_rebusys = Input(UInt(aluWidth.W))

  //@req-spec-issue.g5
  //@req-spec-vrf.e3
  // The existing BOOM integer wakeup network. All three vector queues
  // connect to it: the base address, the stride and the `.vx` integer
  // operand are GPR-sourced.
  val int_wakeup_ports = Flipped(Vec(numIntWakeupPorts, Valid(new Wakeup)))

  //@req-spec-issue.g9
  //@req-spec-issue.g10
  // The FP network, elaborated ONLY when `isAluSlot`: a `.vf`-form vector FP
  // op or `vfmv.*.f` sources one scalar FP register, matched here. A load or
  // store slot has no such port and pays for no FP comparator.
  val fp_wakeup_ports = if (isAluSlot) Some(Flipped(Vec(numFpWakeupPorts, Valid(new Wakeup)))) else None

  //@req-spec-rename.h16
  //@req-spec-issue.g6
  // The dedicated VL wakeup network: `numVlWakeupPorts` (= `aluWidth + 1`)
  // lanes, each a bare VL-PRN readiness event (`Valid(index)`, no payload).
  // All three queues connect to it. Sized from `numVlWakeupPorts`, never a
  // literal -- see the parameters note in the nlhdl source (decision D8: the
  // vset writeback is REPLICATED per ALU EU, never arbitrated).
  val vl_wakeup = Flipped(Vec(numVlWakeupPorts, Valid(UInt(vlPregSz.W))))

  // `vec_group_done`, broadcast to every VecGroupReady instance and read
  // nowhere else in this module -- this slot never inspects a group-done
  // itself. `numVecWbPorts` is read from `vectorParams` directly (forwarded,
  // not re-declared as a constructor parameter), the same binding
  // VecGroupReady itself uses, so the literal 3 exists in one place only.
  //
  // (part 10 of the nlhdl logic section, no reqs of its own): `numVecClrPorts`
  // is `VectorParams`' OTHER "3" -- the ROB busy-clear lane count. This slot
  // has NO clear port and no ROB-facing port at all; the single group-done
  // event drives this slot's matchers, the busy table and the ROB clear in
  // the SAME cycle from three separate consumers, of which this is only one.
  val vec_group_done = Flipped(Vec(vectorParams.numVecWbPorts, Valid(new VecGroupDone)))

  // The per-member readiness SIDE CHANNEL: the single canonical `VecMemberRdy`
  // bundle declared in VecBundles (five per-member groups -- vs1_rdy/vs2_rdy/
  // vs3_rdy/vtmp_rdy/vold_rdy, each Vec(maxVecMembers, Bool) -- plus vm_rdy, a
  // single Bool). VecPipeline part 13's ruling -- one declaration, not a
  // second copy under a second name -- is satisfied by binding to it here.
  val in_member_rdy  = Input(new VecMemberRdy)
  val out_member_rdy = Output(new VecMemberRdy)

  // `rob_pnr_idx`/`rob_head_idx` -- elaborated only when `pnrGate` (IQ_V_ALU).
  // Both are needed: `IsOlder(a, b, head)` takes the ROB head to resolve
  // wraparound.
  val rob_pnr_idx  = if (pnrGate) Some(Input(UInt(robAddrSz.W))) else None
  val rob_head_idx = if (pnrGate) Some(Input(UInt(robAddrSz.W))) else None

  // The per-entry selection input the issue unit's priority encoder uses:
  // `request` on a non-`pnrGate` slot, `request && IsOlder(...)` on IQ_V_ALU.
  val eligible = Output(Bool())
}

/**
 * VecIssueSlot -- see the file header for the full design rationale.
 * Instantiated by VecIssueUnit as `slots`, `count: numEntries`, in all three
 * IQ_V_* instances.
 *
 * @param iqType             one of IQ_V_LOAD/IQ_V_STORE/IQ_V_ALU
 *                            (boom.v4.common.constants.ScalarOpConstants).
 *                            Used only to derive the elaboration-time Scala
 *                            Booleans below; no hardware ever compares it.
 * @param numIntWakeupPorts   width of the integer wakeup network.
 * @param pnrGate             true only for IQ_V_ALU (see VecIssueSlotIO).
 * @param numFpWakeupPorts    width of the FP wakeup network (isAluSlot only).
 */
class VecIssueSlot(
  val iqType:           Int,
  val numIntWakeupPorts: Int,
  val pnrGate:           Boolean = false,
  val numFpWakeupPorts:  Int = 0)(implicit p: Parameters) extends BoomModule
{
  // ---- Elaboration-time Booleans; no hardware ever compares `iqType` ----
  val isLoadSlot  = iqType == IQ_V_LOAD
  val isStoreSlot = iqType == IQ_V_STORE
  val isAluSlot   = iqType == IQ_V_ALU
  require(isLoadSlot || isStoreSlot || isAluSlot,
    "VecIssueSlot: iqType must be one of IQ_V_LOAD/IQ_V_STORE/IQ_V_ALU")
  require(!pnrGate || isAluSlot,
    "VecIssueSlot: pnrGate is meaningful only for IQ_V_ALU")
  require(numFpWakeupPorts == 0 || isAluSlot,
    "VecIssueSlot: the FP wakeup network reaches IQ_V_ALU alone")

  val io = IO(new VecIssueSlotIO(numIntWakeupPorts, numFpWakeupPorts, isAluSlot, pnrGate))

  // =========================================================================
  // ---- 1. The frame: baseline's slot, unchanged ----
  // =========================================================================

  //@req-spec-core.e13
  //@req-spec-issue.g1
  //@req-spec-issue.g3
  val slot_valid = RegInit(false.B)
  val slot_uop   = Reg(new MicroOp())

  val next_valid = WireInit(slot_valid)
  val next_uop   = WireInit(UpdateBrMask(io.brupdate, slot_uop))

  val killed = IsKilledByBranch(io.brupdate, io.kill, slot_uop)

  io.valid         := slot_valid
  io.out_uop       := next_uop
  io.will_be_valid := next_valid && !killed

  when (io.kill) {
    slot_valid := false.B
  } .elsewhen (io.in_uop.valid) {
    slot_valid := true.B
  } .elsewhen (io.clear) {
    slot_valid := false.B
  } .otherwise {
    slot_valid := next_valid && !killed
  }

  when (io.in_uop.valid) {
    slot_uop := io.in_uop.bits
    assert (!slot_valid || io.clear || io.kill)
  } .otherwise {
    slot_uop := next_uop
  }

  //@req-spec-issue.f14
  // The vector additions cannot collide with the scalar ones because they
  // are matched on DIFFERENT networks against DIFFERENT physical numbering
  // spaces: prs1/prs2 against integer pdsts, the FP scalar against FP pdsts,
  // pvl against VL PRNs on vl_wakeup, and pvs*/pvm against VRF PRNs inside
  // the VecGroupReady instances. This is the same int-vs-FP partitioning
  // BOOM already relies on, extended by two networks rather than replaced.

  // "END-OF-CYCLE" uop: the value every vector matcher's prns/members/used
  // must be driven from -- io.in_uop.bits while a load/dispatch/collapse-move
  // is landing this cycle (io.in_uop.valid), slot_uop otherwise. One shared
  // wire rather than repeating the Mux five times.
  val active_uop: MicroOp = Mux(io.in_uop.valid, io.in_uop.bits, slot_uop)

  // Whether `active_uop` actually HOLDS a uop. `slot_uop` is a plain Reg with no
  // reset value, so in an EMPTY slot every field of `active_uop` is garbage --
  // at time 0 it is X, and after an occupant leaves it is that occupant's stale
  // fields.
  //
  // ===> EVERY `VecGroupReady.io.used` MUST BE QUALIFIED BY THIS. Not doing so
  //      cost the first cosim run of gate (e1): `iq_v_load.slots_3.rdy_vs2`
  //      tripped "members out of range 1..maxVecMembers while the operand is
  //      used" at 785 ns, running vset_test.elf -- a test with NO VECTOR LOADS,
  //      so slot 3 of the load queue was empty the whole time. The garbage
  //      `slot_uop` presented `v_uses_vs2 = 1` with `v_emul = 0`, and the
  //      assertion in `VecGroupReady` (which is right to check the invariant)
  //      had no way to know the slot was idle.
  //
  //      Qualifying `used` rather than weakening the assertion is the correct
  //      direction, and it is FUNCTIONALLY FREE: `io.used` feeds only
  //      `io.ready := !io.used || group_all_rdy` and that assertion, and
  //      `ready` is consumed only while the slot is valid (VecGroupReady's own
  //      reset comment says so). An idle slot therefore reports ready, which is
  //      what it already effectively did.
  val active_valid: Bool = io.in_uop.valid || slot_valid

  // =========================================================================
  // ---- 2. Scalar feeders: baseline comparators, verbatim per network ----
  // =========================================================================

  next_uop.iw_p1_bypass_hint := false.B
  next_uop.iw_p2_bypass_hint := false.B
  next_uop.iw_p3_bypass_hint := false.B
  next_uop.iw_p1_speculative_child := 0.U
  next_uop.iw_p2_speculative_child := 0.U

  val rebusied_prs1 = WireInit(false.B)
  val rebusied_prs2 = WireInit(false.B)
  val rebusied = rebusied_prs1 || rebusied_prs2

  //@req-spec-issue.g4
  // The base address, the stride and any `.vx` integer operand REUSE the
  // existing prs1/prs2 operand slots -- no new operand field on MicroOp. A
  // unit-stride or indexed access uses prs1 for the base and leaves prs2
  // unused; a strided access uses both.
  val int_prs1_matches = io.int_wakeup_ports.map { w => w.bits.uop.pdst === slot_uop.prs1 }
  val int_prs1_wakeups = (io.int_wakeup_ports zip int_prs1_matches).map { case (w, m) => w.valid && m }
  val int_prs1_rebusys = (io.int_wakeup_ports zip int_prs1_matches).map { case (w, m) => w.bits.rebusy && m }
  val int_bypassables       = io.int_wakeup_ports.map(_.bits.bypassable)
  val int_speculative_masks = io.int_wakeup_ports.map(_.bits.speculative_mask)

  val prs2_matches = io.int_wakeup_ports.map { w => w.bits.uop.pdst === slot_uop.prs2 }
  val prs2_wakeups = (io.int_wakeup_ports zip prs2_matches).map { case (w, m) => w.valid && m }
  val prs2_rebusys = (io.int_wakeup_ports zip prs2_matches).map { case (w, m) => w.bits.rebusy && m }

  // ASSUMPTION: the nlhdl pseudocode gates the readiness AND with
  // "prs1 participates"/"prs2 participates" terms distinct from the busy
  // bits themselves. Baseline's own `iss_ready` has no such separate term --
  // it reads `prs1_busy`/`prs2_busy` directly -- relying on the invariant
  // that rename never marks an unencoded operand (RT_X) busy in the first
  // place. Reused unchanged here (part 2's own instruction: "copied from the
  // scalar slot unchanged"), so "participates" collapses into the plain busy
  // read below; not re-derived from lrs*_rtype.
  //
  // ASSUMPTION: unlike baseline (one merged wakeup bus feeding prs1/prs2/
  // prs3 alike), this module has two PHYSICALLY SEPARATE networks
  // (int_wakeup_ports, fp_wakeup_ports) with independent PRN numbering
  // spaces that can coincidentally share a numeric value. The wakeup-clear
  // for prs1 is therefore explicitly qualified by lrs1_rtype below (not just
  // the rebusy term, as baseline qualifies), so an FP-network match can
  // never clear an integer-sourced prs1 and vice versa.
  when (int_prs1_wakeups.reduce(_ || _) && slot_uop.lrs1_rtype === RT_FIX) {
    next_uop.prs1_busy := false.B
    next_uop.iw_p1_speculative_child := Mux1H(int_prs1_wakeups, int_speculative_masks)
    next_uop.iw_p1_bypass_hint := Mux1H(int_prs1_wakeups, int_bypassables)
  }
  //@req-spec-vrf.e1
  //@req-spec-vrf.e2
  //@req-spec-vrf.e5
  //@req-spec-vrf.e6
  // The SPECULATIVE load-hit wakeup reaches the scalar feeders unchanged: a
  // vector uOP waiting on a scalar-load result is woken speculatively just
  // like any integer/FP consumer, and the retraction path is baseline's too
  // -- prs1_rebusys/prs2_rebusys and the child_rebusys & iw_p*_speculative_
  // child term set next_uop.prs*_busy back to true and raise rebusied_prs1/
  // rebusied_prs2, so the entry re-requests. Speculation and re-busy exist
  // HERE, on the scalar half, and nowhere on the vector half (VecGroupReady
  // owns no such port at all). Both rebusy terms keep baseline's
  // lrs*_rtype === RT_FIX qualification, so the FP-sourced case is not
  // re-busied by an integer retraction.
  when ((int_prs1_rebusys.reduce(_ || _) || ((io.child_rebusys & slot_uop.iw_p1_speculative_child) =/= 0.U)) &&
    slot_uop.lrs1_rtype === RT_FIX) {
    next_uop.prs1_busy := true.B
    rebusied_prs1 := true.B
  }
  when (prs2_wakeups.reduce(_ || _)) {
    next_uop.prs2_busy := false.B
    next_uop.iw_p2_speculative_child := Mux1H(prs2_wakeups, int_speculative_masks)
    next_uop.iw_p2_bypass_hint := Mux1H(prs2_wakeups, int_bypassables)
  }
  when ((prs2_rebusys.reduce(_ || _) || ((io.child_rebusys & slot_uop.iw_p2_speculative_child) =/= 0.U)) &&
    slot_uop.lrs2_rtype === RT_FIX) {
    next_uop.prs2_busy := true.B
    rebusied_prs2 := true.B
  }

  //@req-spec-issue.g8
  // The scalar FP source of a `.vf`-form op or `vfmv.*.f` rides prs1 as well,
  // with lrs1_rtype === RT_FLT selecting the FP network for that comparator
  // (RT_FIX selects the integer one, above). One scalar source is enough:
  // the two forms are mutually exclusive (a `.vf` op has no `.vx` operand and
  // no address), so no third operand slot and no new MicroOp field is
  // needed. Elaborated only when isAluSlot -- the FP network reaches
  // IQ_V_ALU alone.
  if (isAluSlot) {
    val fp_prs1_matches = io.fp_wakeup_ports.get.map { w => w.bits.uop.pdst === slot_uop.prs1 }
    val fp_prs1_wakeups = (io.fp_wakeup_ports.get zip fp_prs1_matches).map { case (w, m) => w.valid && m }
    val fp_bypassables       = io.fp_wakeup_ports.get.map(_.bits.bypassable)
    val fp_speculative_masks = io.fp_wakeup_ports.get.map(_.bits.speculative_mask)

    when (fp_prs1_wakeups.reduce(_ || _) && slot_uop.lrs1_rtype === RT_FLT) {
      next_uop.prs1_busy := false.B
      next_uop.iw_p1_speculative_child := Mux1H(fp_prs1_wakeups, fp_speculative_masks)
      next_uop.iw_p1_bypass_hint := Mux1H(fp_prs1_wakeups, fp_bypassables)
    }
    // No FP-side rebusy/retraction path, deliberately: baseline's re-busy
    // machinery exists for the fixed, speculatable load-hit latency on the
    // integer network only. The FP network's wakeups here are ordinary
    // (non-speculative) writebacks, and the integer side's own rebusy above
    // is already qualified lrs1_rtype === RT_FIX, so an FP-sourced prs1 is
    // never retracted by an integer-domain event.
  }

  // Assert that a slot never sees RT_FLT on lrs1_rtype unless isAluSlot.
  if (!isAluSlot) {
    assert(!slot_valid || slot_uop.lrs1_rtype =/= RT_FLT,
      "VecIssueSlot: RT_FLT lrs1_rtype seen outside an ALU slot")
  }

  // Readiness on the scalar half reads the REGISTERED slot_uop.prs*_busy /
  // pvl_busy, as baseline does, so an INT/FP wakeup at cycle N produces a
  // request at N+1. The vector half (part 3) is same-cycle. The asymmetry is
  // deliberate, per the nlhdl source: "improving" the scalar half to be
  // combinational would change the timing of machinery this design promised
  // to reuse unchanged.
  //@req-spec-core.f5
  //@req-spec-rename.g2
  val scalar_operands_ready = !slot_uop.prs1_busy && !slot_uop.prs2_busy && !slot_uop.pvl_busy.get

  // =========================================================================
  // ---- 3. Vector operands: five matchers, one bit each ----
  // =========================================================================

  //@req-spec-issue.g11
  // The vector source operands are pvs1, pvs2, pvs3 and the mask pvm (V0),
  // each given one VecGroupReady instance: rdy_vs1, rdy_vs2, rdy_vs3
  // (isMask = false) and rdy_vm (isMask = true). A LOAD or ALU slot adds a
  // FIFTH instance, rdy_vold (isMask = false), pointed at the stale_pvdest
  // group (part 11). Each returns ONE group-ready bit, high only when the
  // operand's LAST valid member has completed.
  val rdy_vs1 = Module(new VecGroupReady(isMask = false))
  val rdy_vs2 = Module(new VecGroupReady(isMask = false))
  val rdy_vs3 = Module(new VecGroupReady(isMask = false))
  val rdy_vm  = Module(new VecGroupReady(isMask = true))
  //@req-spec-issue.g35
  //@req-spec-issue.g36
  //@req-spec-issue.g37
  // stale_pvdest is the PREVIOUS mapping of the destination arch vregs, so
  // its producer is an OLDER instruction -- age-ordered issue grants the
  // oldest READY entry, which does NOT mean an older producer has completed.
  // Two consumers would otherwise read a BUSY group and get garbage: the LCB
  // pre-loading inactive-lane data on VRF port R2 for a vta=0/vma=0 load, and
  // the coprocessor pulling it as the STALE_VD source slot. A STORE slot
  // carries no such matcher: a store has no vector destination, hence no
  // stale group, so rdy_vold is elaborated with count 0 there (this `if`).
  val rdy_vold: Option[VecGroupReady] = if (!isStoreSlot) Some(Module(new VecGroupReady(isMask = false))) else None

  // Broadcast, unregistered, identical at every instance.
  rdy_vs1.io.group_done := io.vec_group_done
  rdy_vs2.io.group_done := io.vec_group_done
  rdy_vs3.io.group_done := io.vec_group_done
  rdy_vm.io.group_done  := io.vec_group_done
  rdy_vold.foreach(_.io.group_done := io.vec_group_done)

  // `load`, `in_member_rdy`, `prns`, `members`, `used` are END-OF-CYCLE
  // values, driven from `active_uop` (io.in_uop.bits while io.in_uop.valid,
  // slot_uop otherwise); `load` itself is io.in_uop.valid.
  rdy_vs1.io.load := io.in_uop.valid
  rdy_vs2.io.load := io.in_uop.valid
  rdy_vs3.io.load := io.in_uop.valid
  rdy_vm.io.load  := io.in_uop.valid
  rdy_vold.foreach(_.io.load := io.in_uop.valid)

  rdy_vs1.io.prns    := active_uop.pvs1.get
  rdy_vs1.io.members.get := active_uop.v_emul.get
  rdy_vs2.io.prns    := active_uop.pvs2.get
  rdy_vs2.io.members.get := active_uop.v_emul.get
  rdy_vm.io.prns     := VecInit(active_uop.pvm.get)

  //@req-spec-issue.g29
  //@req-spec-issue.g30
  //@req-spec-issue.g31
  // rdy_vm.used is read from the MicroOp field and NEVER re-derived from
  // inst(25) -- the instruction word does not reach issue. An unmasked OP.v
  // leaves pvm DON'T-CARE and this forces its matcher ready: pvm otherwise
  // names whatever physical register V0 was last mapped to, and the mapper
  // renames lvm only for a masked op, so waiting on it would wait forever on
  // a STALE mask PRN no producer will ever complete.
  rdy_vm.io.used := active_valid && active_uop.v_is_masked.get

  rdy_vs1.io.in_member_rdy := io.in_member_rdy.vs1_rdy
  rdy_vs2.io.in_member_rdy := io.in_member_rdy.vs2_rdy
  rdy_vm.io.in_member_rdy  := VecInit(io.in_member_rdy.vm_rdy)

  io.out_member_rdy.vs1_rdy := rdy_vs1.io.out_member_rdy
  io.out_member_rdy.vs2_rdy := rdy_vs2.io.out_member_rdy
  io.out_member_rdy.vm_rdy  := rdy_vm.io.out_member_rdy(0)

  //@req-spec-issue.g32
  //@req-spec-issue.c14
  //@req-spec-lsu.l2
  // Per-queue `used` gating, expressed in v_uses_vs* (D11's MicroOp fields
  // written by VDecode/VLSDecode), NOT in is_shared/uses_ldq alone: the ALU
  // slot's OR term is load-bearing (part 7 below), but the LOAD slot's rdy_vs3
  // must be gated off by v_uses_vs3 first -- per VLSDecode uses_vs3 is
  // is_store, so a plain unit-stride LOAD has it false, and gating on
  // `!is_shared` alone would incorrectly wait on the CURRENT mapping of v0
  // (the D11 fix for the most common vector instruction in the design: an
  // unsegmented `vle64.v` hanging forever on a v0 group-done that already
  // fired). ALL QUEUES additionally gate rdy_vs1/rdy_vs2 on v_uses_vs1/
  // v_uses_vs2 -- an unencoded source is neither renamed nor waited on, one
  // mechanism read from two sides of the rename/issue seam.
  rdy_vs1.io.used := active_valid && active_uop.v_uses_vs1.get
  rdy_vs2.io.used := active_valid && active_uop.v_uses_vs2.get

  //@req-spec-issue.c11
  //@req-spec-rob.d12
  //@req-spec-lsu.l5
  // ---- 7. pvtmp: how the consumer half of a shared op wakes ----
  //
  // The two halves of a shared OP.v rendezvous on the pvtmp group, woken by
  // the ORDINARY vector wakeup network -- no private path, no cross-queue
  // signal, no poll. For a segmented STORE, VecStoreDgenPath (below) selects
  // pvtmp into rdy_vs3 whenever is_shared -- that half is handled entirely
  // inside the `isStoreSlot` block further down. For a segmented LOAD, the
  // consumer is the coprocessor half in IQ_V_ALU: this slot performs the
  // mirror-image selection itself, since dgen_path does not exist in an ALU
  // slot. WARNING -- the select is qualified by DIRECTION (is_shared &&
  // uses_ldq), and dropping that qualification is the mirror image of the
  // milestone-1 DGEN bug: for a segmented STORE the coprocessor half WRITES
  // pvtmp and READS pvs3, so a bare Mux(is_shared, pvtmp, pvs3) would make
  // that half wait for the group-done of the group it is itself about to
  // produce -- an immediate self-deadlock.
  // `vs3_selects_tmp` is computed ONCE per elaborated isAluSlot instance and
  // reused both for the rdy_vs3 wiring below and for that operand's busy
  // mirror (part 3) later in this block, so the two can never disagree about
  // which group rdy_vs3 is tracking this cycle.
  val vs3_selects_tmp = if (isAluSlot) active_uop.is_shared.get && active_uop.uses_ldq else false.B
  if (isAluSlot) {
    rdy_vs3.io.prns := Mux(vs3_selects_tmp, active_uop.pvtmp.get, active_uop.pvs3.get)
    // v_emul is a single MicroOp field shared by every vector operand of a
    // uop (no separate per-group EMUL), so no select is needed for members.
    rdy_vs3.io.members.get := active_uop.v_emul.get
    //@req-spec-issue.g32
    // THE ALU SLOT'S (is_shared && uses_ldq) || TERM IS LOAD-BEARING AND MUST
    // NOT BE SIMPLIFIED TO v_uses_vs3: for a segmented LOAD both halves carry
    // the SAME uop (decoded by VLSDecode), so v_uses_vs3 is false, yet the
    // select above has re-pointed this instance at pvtmp, which the
    // coprocessor half genuinely must wait on. Dropping the OR term silently
    // deletes the coprocessor half's only rendezvous with the LSU half.
    rdy_vs3.io.used := active_valid && (vs3_selects_tmp || active_uop.v_uses_vs3.get)

    // The per-member side channel follows the SAME select: each matcher
    // writes its next state into the FIELD IT SELECTED (vtmp_rdy when the
    // select took pvtmp, vs3_rdy otherwise), so the receiving slot's
    // identical select reads it back after a collapse move. The field NOT
    // selected this cycle passes its input straight through unchanged (the
    // same convention rdy_vold uses in a store slot for the field it never
    // reads).
    rdy_vs3.io.in_member_rdy := Mux(vs3_selects_tmp, io.in_member_rdy.vtmp_rdy, io.in_member_rdy.vs3_rdy)
    io.out_member_rdy.vs3_rdy  := Mux(vs3_selects_tmp, io.in_member_rdy.vs3_rdy, rdy_vs3.io.out_member_rdy)
    io.out_member_rdy.vtmp_rdy := Mux(vs3_selects_tmp, rdy_vs3.io.out_member_rdy, io.in_member_rdy.vtmp_rdy)
  } else if (isLoadSlot) {
    // The LSU half of a segmented load never reads pvs3 (that is the
    // COPROCESSOR half's source group) and never touches pvtmp either (it is
    // pvtmp's PRODUCER, not a consumer) -- rdy_vs3 is never re-pointed here.
    rdy_vs3.io.prns    := active_uop.pvs3.get
    rdy_vs3.io.members.get := active_uop.v_emul.get
    //@req-spec-issue.g32
    // The `!is_shared` term is the named structural discharge of g32 on the
    // load path: with v_uses_vs3 in front of it the term is REDUNDANT (a
    // load's uses_vs3 is already false per VLSDecode) and KEPT ANYWAY, so a
    // future change to what decode puts in uses_vs3 cannot silently re-admit
    // pvs3 into this cone.
    rdy_vs3.io.used := active_valid && active_uop.v_uses_vs3.get && !active_uop.is_shared.get
    rdy_vs3.io.in_member_rdy := io.in_member_rdy.vs3_rdy
    io.out_member_rdy.vs3_rdy  := rdy_vs3.io.out_member_rdy
    // vtmp_rdy is untouched by a load slot; pass it through unchanged, same
    // convention as the store slot's vold_rdy passthrough below.
    io.out_member_rdy.vtmp_rdy := io.in_member_rdy.vtmp_rdy
  }
  // isStoreSlot's rdy_vs3 wiring (prns/members/used/in_member_rdy/
  // out_member_rdy, all selected by VecStoreDgenPath's own is_shared mux) is
  // set up together with dgen_path below, since the two are one seam.

  //@req-spec-decode.i6
  //@req-spec-issue.h2
  // ---- 5. pvl and vtype ----
  //
  // Every vector uOP carries pvl as an implicit operand, woken on the VL
  // network: one comparator PER LANE, OR-ed, clearing next_uop.pvl_busy. No
  // value is captured here -- the slot holds no VL register and no width for
  // one; the value is read from the VL RF by the vector EU at execute. pvl
  // gets no speculative wakeup and no re-busy term: no VL producer has a
  // load-use latency worth speculating on.
  val vl_hit = io.vl_wakeup.map(w => w.valid && w.bits === slot_uop.pvl.get).reduce(_ || _)
  when (vl_hit) {
    next_uop.pvl_busy.get := false.B
  }

  //@req-spec-issue.g7
  //@req-spec-issue.f11
  // vtype is NOT an issue-slot operand and is never woken on any network. It
  // rides the per-uOP VConfig snapshot taken at decode from the speculative
  // VCFG mirror, so there is no pvtype, no vtype busy bit, no vtype
  // comparator and no vtype wakeup port anywhere in this file.

  //@req-spec-issue.f15
  // Deliberately ABSENT (per the ports section): pred_wakeup_port/ppred (a
  // vector OP.v is never an SFB shadow), any speculative-wakeup or re-busy
  // port on the vector side, any busy input from VecLsu or any vector
  // datapath, any vtype operand or wakeup port, and any VL VALUE input. This
  // module is the only slot with vector match ports; baseline's scalar
  // IssueSlot gains none of them.

  // =========================================================================
  // ---- 4. rdy_vold: `used`, and part 11's conservative gate ----
  // =========================================================================

  //@req-spec-issue.g38
  // The match is PER MEMBER, never a single aggregate busy bit -- an
  // aggregate MicroOp `stale_pvdest_busy` field would be UNSAFE (it cannot
  // encode "waiting on producer 3 of 8" for an LMUL=8 stale group installed
  // by up to eight instructions), which is exactly why the fifth VecGroupReady
  // instance exists instead. `members` is v_emul, identical to the
  // destination's own member count by construction.
  rdy_vold.foreach { m =>
    m.io.prns    := active_uop.stale_pvdest.get
    m.io.members.get := active_uop.v_emul.get
    //@req-spec-issue.g39
    // Deliberately CONSERVATIVE in both queues: on IQ_V_ALU the host cannot
    // know whether the coprocessor will actually pull the STALE_VD source
    // slot (no VPU-side signal exists to ask), so any op with a vector
    // destination waits; on IQ_V_LOAD, VL is not known at issue (it is read
    // from the VL RF at execute), so the VL=0 group-copy reader cannot be
    // narrowed either. A segmented STORE's coprocessor half is excluded for
    // free: it writes pvtmp and its dst_rtype is not RT_VEC.
    m.io.used := active_valid && active_uop.dst_rtype === RT_VEC
    m.io.in_member_rdy := io.in_member_rdy.vold_rdy
    io.out_member_rdy.vold_rdy := m.io.out_member_rdy
  }
  if (isStoreSlot) {
    // A store slot receives the whole side-channel bundle (one declaration,
    // no per-queue variant) and simply leaves vold_rdy unread, driving it
    // through to out_member_rdy unchanged.
    io.out_member_rdy.vold_rdy := io.in_member_rdy.vold_rdy
  }

  // `members` out-of-range assertion, and the g39 conservative-gate
  // consistency check ("rdy_vold.used implies slot_uop.pvdest is a valid
  // group") -- part 8.
  rdy_vold.foreach { m =>
    assert(!m.io.used || (active_uop.v_emul.get >= 1.U && active_uop.v_emul.get <= maxVecMembers.U),
      "VecIssueSlot: rdy_vold used but v_emul out of range 1..maxVecMembers")
  }

  // The mask instance's `members` port does not exist (isMask elides it);
  // the four non-mask instances share the identical `members := v_emul`
  // binding (rdy_vs1/rdy_vs2 above, rdy_vs3 in the isAluSlot/isLoadSlot
  // branches above and in the isStoreSlot block below, rdy_vold here).

  // =========================================================================
  // ---- Vector operand busy mirror (part 3) ----
  // =========================================================================
  //
  // The slot mirrors each matcher's result into the outgoing uop's aggregate
  // busy bit, so a downstream reader of iss_uop/out_uop never sees a stale
  // busy bit. This mirror is NOT the readiness path -- it must not be, and is
  // not, read back into any request term above. rdy_vold HAS NO MIRROR: that
  // is deliberate (part 11) -- MicroOp declares no stale_pvdest_busy field
  // and must not gain one.
  next_uop.pvs1_busy.get := !rdy_vs1.io.ready
  next_uop.pvs2_busy.get := !rdy_vs2.io.ready
  next_uop.pvm_busy.get  := !rdy_vm.io.ready
  if (isAluSlot) {
    // Reuses the SAME vs3_selects_tmp computed in part 7 above.
    when (vs3_selects_tmp) {
      next_uop.pvtmp_busy.get := !rdy_vs3.io.ready
    } .otherwise {
      next_uop.pvs3_busy.get := !rdy_vs3.io.ready
    }
  } else if (isLoadSlot) {
    next_uop.pvs3_busy.get := !rdy_vs3.io.ready
  }
  // isStoreSlot's mirror (selected by dgen_operand_is_pvtmp) is set with the
  // rest of the store-slot wiring below.

  // =========================================================================
  // ---- 6. request and eligibility, and the store slot's second grant path ----
  // =========================================================================

  val request  = Wire(Bool())
  val eligible = Wire(Bool())

  if (isStoreSlot) {
    // VecStoreDgenPath -- the store slot's AGEN/DGEN path-sequencing rule
    // (spec-issue.g22-g24/c12/spec-cii.i12 -- VecStoreDgenPath's OWN reqs,
    // discharged inside that module, not re-tagged here)
    // plus the is_shared store-data operand mux. Combinational, holds no
    // register and no counter: the "still outstanding" state rides
    // slot_uop.fu_code(FC_AGEN)/fu_code(FC_DGEN), migrating with the uop
    // through a collapse move exactly like every other MicroOp field, rather
    // than in a private register that a collapse move would strand.
    val dgen_path = Module(new VecStoreDgenPath)

    dgen_path.io.slot_valid := slot_valid
    dgen_path.io.grant      := io.grant
    dgen_path.io.squash_grant := io.squash_grant
    dgen_path.io.slot_uop   := slot_uop

    // The gated operand is selected by is_shared: pvtmp when set, pvs3
    // otherwise (spec-issue.g25-g28/spec-cii.i6 -- also VecStoreDgenPath's
    // OWN reqs, discharged inside that module's operand mux, not re-tagged
    // here). dgen_path owns that mux internally; this slot only routes the
    // mux's outputs into rdy_vs3's inputs and reads rdy_vs3.ready back as
    // dgen_operand_ready -- pvs3/pvtmp themselves are never merged or
    // reinterpreted.
    rdy_vs3.io.prns    := dgen_path.io.dgen_operand
    rdy_vs3.io.members.get := dgen_path.io.dgen_operand_members
    // IQ_V_STORE rdy_vs3.used := true: it tracks the SELECTED DGEN group,
    // always in play for a store.
    rdy_vs3.io.used := active_valid
    rdy_vs3.io.in_member_rdy := Mux(dgen_path.io.dgen_operand_is_pvtmp,
      io.in_member_rdy.vtmp_rdy, io.in_member_rdy.vs3_rdy)
    io.out_member_rdy.vs3_rdy  := Mux(dgen_path.io.dgen_operand_is_pvtmp,
      io.in_member_rdy.vs3_rdy, rdy_vs3.io.out_member_rdy)
    io.out_member_rdy.vtmp_rdy := Mux(dgen_path.io.dgen_operand_is_pvtmp,
      rdy_vs3.io.out_member_rdy, io.in_member_rdy.vtmp_rdy)

    // Busy mirror for the store slot's selected group (part 3), using
    // dgen_operand_is_pvtmp -- the same is_shared select the mux above used
    // -- to choose which aggregate field to update.
    when (dgen_path.io.dgen_operand_is_pvtmp) {
      next_uop.pvtmp_busy.get := !rdy_vs3.io.ready
    } .otherwise {
      next_uop.pvs3_busy.get := !rdy_vs3.io.ready
    }

    // The store-data operand must not leak into the ADDRESS path's
    // readiness: agen_operands_ready EXCLUDES rdy_vs3.
    val agen_operands_ready = scalar_operands_ready && rdy_vs1.io.ready && rdy_vs2.io.ready && rdy_vm.io.ready
    dgen_path.io.agen_operands_ready := agen_operands_ready
    dgen_path.io.dgen_operand_ready  := rdy_vs3.io.ready
    // "agen_rebusied (= rebusied_prs1)" -- the nlhdl dependencies section's
    // literal binding; the AGEN path's scalar feeder is the base address
    // (prs1), not the stride, so only prs1's retraction re-offers AGEN.
    dgen_path.io.agen_rebusied := rebusied_prs1

    //@req-spec-issue.g19
    //@req-spec-issue.g20
    request := dgen_path.io.agen_request || dgen_path.io.dgen_request

    io.iss_uop := slot_uop
    io.iss_uop.fu_code(FC_AGEN) := dgen_path.io.iss_fu_code_agen
    io.iss_uop.fu_code(FC_DGEN) := dgen_path.io.iss_fu_code_dgen
    // ===> TWO PIECES OF BASELINE'S isMem BLOCK MUST NOT BE COPIED: (a) no
    // io.iss_uop.prs1 := slot_uop.prs2 scalar DGEN operand rewrite (vector
    // store data is a VRF group read on port R3, not a scalar operand); (b)
    // no lrs2_rtype/prs2 DCE clobber (prs2 carries the STRIDE and must reach
    // the AGEN intact).

    next_uop.iw_issued := io.grant && !io.squash_grant
    next_uop.iw_issued_partial_agen := dgen_path.io.issued_partial_agen
    next_uop.iw_issued_partial_dgen := dgen_path.io.issued_partial_dgen

    // The migrated fu_code(FC_AGEN)/fu_code(FC_DGEN) pair (spec-issue.d13 --
    // VecStoreDgenPath's OWN req; the parent-side wiring below just carries
    // its next_fu_code_* outputs, it does not separately discharge d13):
    // dgen_path computes
    // the NEXT state of the two-bit "still outstanding" encoding
    // combinationally from the CURRENT slot_uop.fu_code and this cycle's
    // grant; writing it into next_uop.fu_code is what makes it migrate with
    // the uop through a collapse move instead of living in a private
    // register that a move would strand.
    next_uop.fu_code(FC_AGEN) := dgen_path.io.next_fu_code_agen
    next_uop.fu_code(FC_DGEN) := dgen_path.io.next_fu_code_dgen

    when (slot_valid && slot_uop.iw_issued) {
      // A DGEN-pending store survives its AGEN grant via dgen_path.keep_valid.
      next_valid := rebusied || dgen_path.io.keep_valid
    }

    eligible := request
  } else {
    //@req-spec-core.f5
    //@req-spec-rename.g2
    //@req-spec-issue.g19
    //@req-spec-issue.g20
    // A non-shared OP.v occupies ONE slot and is granted ONCE, when ALL of
    // its physical operands are ready -- scalar feeders on the integer
    // network, the .vf scalar on FP, pvl on the VL network, and the vector
    // groups on the vector network. There is no partial grant and no
    // operand class that may be skipped.
    // isAluSlot and isLoadSlot share the identical formula: both instantiate
    // rdy_vold (part 4), so the fifth term is always present in this branch.
    val vector_operands_ready =
      rdy_vs1.io.ready && rdy_vs2.io.ready && rdy_vs3.io.ready && rdy_vm.io.ready && rdy_vold.get.io.ready
    request := slot_valid && !slot_uop.iw_issued && scalar_operands_ready && vector_operands_ready

    io.iss_uop := slot_uop

    next_uop.iw_issued := io.grant && !io.squash_grant
    next_uop.iw_issued_partial_agen := false.B
    next_uop.iw_issued_partial_dgen := false.B

    when (slot_valid && slot_uop.iw_issued) {
      next_valid := rebusied
    }

    //@req-spec-issue.g21
    if (pnrGate) {
      // ===> STRICT IsOlder, no equality term OR-ed on: baseline's SNI block
      // admits rob_idx === rob_pnr_idx because rob_pnr_idx there names the
      // oldest UNSAFE entry and SNI is permissive-by-construction/off-by-
      // default. Here that admission would hand a still-speculative op to
      // the coprocessor and defeat the entire reason pnrGate exists.
      eligible := request && IsOlder(slot_uop.rob_idx, io.rob_pnr_idx.get, io.rob_head_idx.get)
    } else {
      // IQ_V_LOAD: vector memory may issue speculatively; ordering and
      // replay are the LSU's business.
      eligible := request
    }
  }

  io.request  := request
  io.eligible := eligible

  // =========================================================================
  // ---- 8. Assertions ----
  // =========================================================================

  assert(!(io.grant && !slot_valid),
    "VecIssueSlot: grant asserted against an invalid slot")
  assert(!(slot_valid && !slot_uop.is_vec.get),
    "VecIssueSlot: a scalar uop occupies a vector issue slot (dispatch-routing bug)")
  assert(!(slot_valid && slot_uop.is_sfb_shadow),
    "VecIssueSlot: a vector uop is marked as an SFB shadow")
  assert(!slot_valid || (slot_uop.v_emul.get >= 1.U && slot_uop.v_emul.get <= maxVecMembers.U),
    "VecIssueSlot: v_emul out of range 1..maxVecMembers while valid")
  if (pnrGate) {
    assert(!io.grant || eligible,
      "VecIssueSlot: a granted pnrGate entry was not eligible this cycle")
  }
  // A store slot elaborating no rdy_vold, and rdy_vold's own used-implies-
  // valid-group check, are covered above (Scala `if`/`foreach`, part 4) --
  // the former is an elaboration-time property, not a hardware assertion.
  // dgen_path's own assertions (instantiated only in isStoreSlot) cover path
  // ordering; nothing here duplicates them.

  // =========================================================================
  // ---- 9. Tracing ----
  // =========================================================================
  //
  // Guarded through the shared VecTrace package, off by default. VecGroupReady
  // deliberately emits nothing itself (it holds no MicroOp); this slot traces
  // on its behalf, using the exported out_member_rdy of each instance.
  //
  // NOTE: VecTrace.nlhdl.scala's spec has moved to a three-step ladder, but
  // VecTrace.scala (the generated package) has NOT been regenerated -- only
  // the old `trace(module, event, uop, extra)` API exists, which is what is
  // used below. Nothing here hand-rolls a printf behind traceEnabled; every
  // call site goes through VecTrace.trace.

  // Event 1: slot fill. "On the cycle the slot is filled" is exactly
  // io.in_uop.valid -- no edge-detection register needed, unlike
  // VecStoreDgenPath's orphaned dgen_operand_select event (see below).
  when (io.in_uop.valid) {
    VecTrace.trace("VecIssueSlot", "fill", io.in_uop.bits, Seq(
      ("v_emul",      io.in_uop.bits.v_emul.get),
      ("v_is_masked", io.in_uop.bits.v_is_masked.get.asUInt),
      ("is_shared",   io.in_uop.bits.is_shared.get.asUInt)))
  }

  // Event 2: the rising edge of request.
  val request_prev = RegNext(request, false.B)
  when (request && !request_prev) {
    VecTrace.trace("VecIssueSlot", "request_rise", slot_uop)
  }

  // Event 3: the grant.
  when (io.grant) {
    VecTrace.trace("VecIssueSlot", "grant", slot_uop)
  }

  // Event 4: the rising edge of each matcher's ready, rdy_vold included --
  // without this line a wait on stale_pvdest and a wait on a real source
  // group look identical from outside the slot in a cosim log.
  def traceMatcherReadyRise(opName: String, m: VecGroupReady): Unit = {
    val readyPrev = RegNext(m.io.ready, false.B)
    when (m.io.ready && !readyPrev) {
      VecTrace.trace("VecIssueSlot", s"${opName}_ready", slot_uop, Seq(
        ("out_member_rdy", m.io.out_member_rdy.asUInt)))
    }
  }
  traceMatcherReadyRise("vs1", rdy_vs1)
  traceMatcherReadyRise("vs2", rdy_vs2)
  traceMatcherReadyRise("vs3", rdy_vs3)
  traceMatcherReadyRise("vm",  rdy_vm)
  rdy_vold.foreach(m => traceMatcherReadyRise("vold", m))
}

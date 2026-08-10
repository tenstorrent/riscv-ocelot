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
import boom.v4.exu.{BrUpdateInfo, IssueParams, Wakeup}
import boom.v4.util.IsKilledByBranch
import boom.v4.vec.generated.{VecGroupDone, VecMemberRdy, VecTrace}

// GENERATED from src/main/nlhdl/vec/issue/VecIssueUnit.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
//
// VecIssueUnit -- one vector issue queue: BOOM v4's age-ordered COLLAPSING
// issue queue and its priority-encoder select, reused unchanged, holding
// VecIssueSlot entries instead of scalar IssueSlot entries.
//
// ONE definition, THREE instances, all created by VecPipeline:
//   `iq_v_load`  iqType = IQ_V_LOAD  (4)  pnrGate = false  numEntries = 16
//   `iq_v_store` iqType = IQ_V_STORE (5)  pnrGate = false  numEntries = 16
//   `iq_v_alu`   iqType = IQ_V_ALU   (6)  pnrGate = true   numEntries = 16
// All three are age-ordered collapsing and grant the OLDEST READY entry. The
// only difference between them is `pnrGate` and which optional slot ports
// exist.
//
// A NEW module, not a delta on
// `v4/exu/issue-units/issue-unit-age-ordered.scala`, but it deliberately
// COPIES that file's structure and every name a reader already knows --
// `dis_uops`, `slots`/`issue_slots`, `vacants`, `shamts_oh`,
// `SaturatingCounterOH`, `will_be_valid`, `uops`, `is_available`, `requests`,
// `port_issued`, `iss_uops` -- so the two diff line for line. Baseline
// `IssueUnitCollapsing` is untouched and the scalar queues keep using it.
// Exactly three things change: the slot type, the signal the encoder selects
// on, and the added per-member readiness side channel.
//
// ===> THE PRIORITY ENCODER SELECTS ON `eligible`, NOT ON `request`.
// VecIssueSlot computes past-PNR eligibility ITSELF, per entry, and exports
// BOTH bits. Selecting on `request` here would silently delete the past-PNR
// gate on `iq_v_alu` and hand speculative work to the coprocessor. Baseline's
// own SNI block is NOT reproduced here, and its extra
// `| (rob_idx === rob_pnr_idx)` term is never introduced either: `rob_pnr_idx`
// names the OLDEST UNSAFE entry (rob.scala:531), so admitting equality would
// admit the entry whose speculation is NOT YET RESOLVED. The past-PNR test
// is a STRICT `IsOlder(rob_idx, rob_pnr_idx, rob_head_idx)`, computed inside
// VecIssueSlot, with nothing OR-ed onto it.
//
// ===> `out_member_rdy` IS ROUTED TO `in_member_rdy` ON THE SAME COLLAPSE
// MOVE THAT ROUTES `out_uop` TO `in_uop` (part 3, below), via one combined
// local bundle so a future edit cannot add a term to one path and forget the
// other.
//
// ===> `IQ_V_ALU` IS DELIBERATELY NOT A HEAD-ONLY FIFO. See part 8 below.
//
// Governing spec anchors: issue.rst `issue-sched-stage`, `shared-store-chain`,
// `cii-shared-sched`, `vec-queue-reservation`; cii.rst `cii-issue`,
// `cii-segmented`, `cii-flush`; execution.rst `vector-execution`;
// overview.rst `caracal-pipeline`; glossary.rst `glossary-terms`;
// midcore.rst `midcore-segmented-store`.

/**
 * VecIssueUnitIO
 *
 * Baseline `IssueUnit`'s port list, minus `wakeup_ports`/`pred_wakeup_port`/
 * `tsc_reg`/`rob_head`/unconditional `rob_pnr_idx` -- plus the vector wakeup
 * networks and the per-member readiness side channel. Deliberately ABSENT,
 * per the nlhdl ports section: `pred_wakeup_port`/`ppred_*`, `tsc_reg`, any
 * port to or from `VecLsu`, any `busy`/`fu_ready` input, any element-queue
 * RESERVATION port, any combinational `rob_flush`, any second `iss_uops`
 * stage, and any cross-queue grant/kill signal.
 */
class VecIssueUnitIO(
  val dispatchWidth:     Int,
  val issueWidth:        Int,
  val numIntWakeupPorts: Int,
  val numFpWakeupPorts:  Int,
  val numVecWbPorts:     Int,
  val isAluQueue:        Boolean,
  val pnrGate:           Boolean)(implicit p: Parameters) extends BoomBundle
{
  // ---- baseline dispatch interface, ready unchanged ----
  val dis_uops = Vec(dispatchWidth, Flipped(Decoupled(new MicroOp())))

  // The per-member readiness side channel at dispatch (decision D6: FIVE
  // groups -- vs1_rdy/vs2_rdy/vs3_rdy/vtmp_rdy/vold_rdy -- plus vm_rdy).
  // Binds to the single `VecMemberRdy` declared in VecBundles; no local
  // shim is declared here.
  val dis_member_rdy = Input(Vec(dispatchWidth, new VecMemberRdy))

  // The grants. On `iq_v_alu` lane 0 is `VecCiiIssue.io.iss` (no `ready`,
  // never refuses a grant); on `iq_v_load`/`iq_v_store` the lanes reach the
  // vector LSU's AGEN/DGEN paths.
  val iss_uops = Output(Vec(issueWidth, Valid(new MicroOp())))

  // The existing BOOM integer wakeup network. All three vector queues
  // connect: base address, stride and the `.vx` integer operand are
  // GPR-sourced.
  val int_wakeup_ports = Flipped(Vec(numIntWakeupPorts, Valid(new Wakeup)))

  // The FP network, elaborated ONLY when `isAluQueue`: a `.vf`-form vector
  // FP op or `vfmv.*.f` sources one scalar FP register. A load or store
  // queue has no such port.
  val fp_wakeup_ports = if (isAluQueue) Some(Flipped(Vec(numFpWakeupPorts, Valid(new Wakeup)))) else None

  // The dedicated VL network: `numVlWakeupPorts` (= `aluWidth + 1`) lanes,
  // sized from that name directly (never a literal 1) -- decision D8
  // replicates the vset writeback per ALU EU rather than arbitrating it.
  // All three queues connect.
  val vl_wakeup = Flipped(Vec(numVlWakeupPorts, Valid(UInt(vlPregSz.W))))

  // The vector (group-done) wakeup network, broadcast unregistered to every
  // slot. This module never inspects a group-done itself. `numVecWbPorts`
  // is a constructor parameter (VecPipeline passes the actual producer
  // count) that must equal `VectorParams.numVecWbPorts`, the single binding
  // site -- see the elaboration-time checks in the module body.
  val vec_group_done = Flipped(Vec(numVecWbPorts, Valid(new VecGroupDone)))

  // Baseline's speculative-child retraction bus, broadcast to every slot and
  // used by the dispatch-cycle pre-correction.
  val child_rebusys = Input(UInt(aluWidth.W))

  // Baseline's per-port functional-unit advertisement -- this design's ONLY
  // back-pressure into issue. `VecCiiIssue` drives `iq_v_alu`'s lane to zero
  // when it has no CII Issue credit; the vector LSU's units drive theirs the
  // same way.
  val fu_types = Input(Vec(issueWidth, Vec(FC_SZ, Bool())))

  val brupdate       = Input(new BrUpdateInfo())
  // BoomCore's `RegNext(rob.io.flush.valid)`, wired to every slot's `kill`
  // exactly as the scalar queues receive it (part 10 -- the one-cycle window
  // this unit deliberately keeps).
  val flush_pipeline = Input(Bool())
  val squash_grant   = Input(Bool())

  // Elaborated ONLY when `pnrGate` (IQ_V_ALU). Both are needed: BOOM's age
  // comparison is the three-argument `IsOlder(a, b, head)`, and the ROB is
  // circular.
  val rob_pnr_idx  = if (pnrGate) Some(Input(UInt(robAddrSz.W))) else None
  val rob_head_idx = if (pnrGate) Some(Input(UInt(robAddrSz.W))) else None
}

/**
 * VecIssueUnit -- see the file header for the full design rationale.
 * Instantiated by VecPipeline three times: `iq_v_load`, `iq_v_store`,
 * `iq_v_alu`.
 *
 * @param params              BOOM's OWN `IssueParams` case class
 *                             (dispatchWidth, issueWidth, numEntries, iqType,
 *                             numSlowEntries, useFullIssueSel, useMatrixIssue)
 *                             -- the same record the scalar queues use, so a
 *                             reader has one shape to learn.
 * @param numIntWakeupPorts   width of the integer wakeup network.
 * @param pnrGate             true only for `iq_v_alu`.
 * @param numFpWakeupPorts    width of the FP wakeup network (isAluQueue only).
 * @param numVecWbPorts       width of the vector (group-done) wakeup network.
 *                             Must equal `vectorParams.numVecWbPorts`, the
 *                             single binding site -- checked at elaboration.
 */
class VecIssueUnit(
  val params:            IssueParams,
  val numIntWakeupPorts: Int,
  val pnrGate:           Boolean = false,
  val numFpWakeupPorts:  Int = 0,
  val numVecWbPorts:     Int)(implicit p: Parameters) extends BoomModule
{
  // ---- Elaboration-time Booleans; no hardware ever compares `iqType` ----
  val iqType       = params.iqType
  val isLoadQueue  = iqType == IQ_V_LOAD
  val isStoreQueue = iqType == IQ_V_STORE
  val isAluQueue   = iqType == IQ_V_ALU
  require(isLoadQueue || isStoreQueue || isAluQueue,
    "VecIssueUnit: iqType must be one of IQ_V_LOAD/IQ_V_STORE/IQ_V_ALU")
  require(!pnrGate || isAluQueue,
    "VecIssueUnit: pnrGate is true only for IQ_V_ALU")
  require(numFpWakeupPorts == 0 || isAluQueue,
    "VecIssueUnit: the FP wakeup network reaches IQ_V_ALU alone")
  // "All three instances exist only in a usingRVV build" (parameters
  // section). The gate is usingRVV, never rocket's usingVector.
  require(usingRVV, "VecIssueUnit: elaborates only under usingRVV")

  val dispatchWidth = params.dispatchWidth
  val issueWidth    = params.issueWidth
  val numIssueSlots = params.numEntries
  // "Require numEntries - numSlowEntries >= dispatchWidth, as baseline does."
  require(numIssueSlots - params.numSlowEntries >= dispatchWidth,
    "VecIssueUnit: numEntries - numSlowEntries must be >= dispatchWidth")

  val io = IO(new VecIssueUnitIO(dispatchWidth, issueWidth, numIntWakeupPorts,
    numFpWakeupPorts, numVecWbPorts, isAluQueue, pnrGate))

  // =========================================================================
  // ---- Port-count binding-site checks (parameters section) ----
  // =========================================================================
  //
  // THE SINGLE BINDING SITE FOR THE VECTOR WAKEUP NETWORK WIDTH IS
  // `VectorParams.numVecWbPorts`. This module still takes it as a
  // constructor parameter (VecPipeline passes the actual producer count and
  // this module forwards the resulting signal to every slot), but the
  // parameter itself must equal the canonical field, or a slot's matcher
  // would examine fewer group-done ports than the network drives, miss a
  // single-shot group-done, and hang the consumer forever.
  //
  // NOTE (adaptation to the child's actual interface): VecIssueSlot's own
  // generated IO sizes its `vec_group_done` port directly from
  // `vectorParams.numVecWbPorts` (the trait value), not from a constructor
  // parameter -- so "forwards it verbatim to every slot" is realized here as
  // a same-width `:=` connection (io.vec_group_done -> issue_slots(i).
  // vec_group_done), not a constructor-parameter passthrough (VecIssueSlot's
  // constructor has no such parameter to receive one). The three requires
  // below are what keep the two independently-sized Vecs the same width.
  require(numVecWbPorts == vectorParams.numVecWbPorts,
    s"VecIssueUnit: numVecWbPorts ($numVecWbPorts) must equal " +
    s"vectorParams.numVecWbPorts (${vectorParams.numVecWbPorts})")
  require(numVecWbPorts == io.vec_group_done.length,
    "VecIssueUnit: numVecWbPorts must equal io.vec_group_done.length")
  require(io.vl_wakeup.length == numVlWakeupPorts,
    s"VecIssueUnit: io.vl_wakeup width (${io.vl_wakeup.length}) must equal " +
    s"numVlWakeupPorts ($numVlWakeupPorts)")
  // A non-pnrGate instance elaborates no rob_pnr_idx port.
  require(pnrGate || io.rob_pnr_idx.isEmpty,
    "VecIssueUnit: a non-pnrGate instance must elaborate no rob_pnr_idx port")

  // =========================================================================
  // ---- 1. The frame: baseline's collapsing queue, copied ----
  // =========================================================================

  //@req-spec-core.e12
  //@req-spec-issue.e9
  //@req-spec-issue.e10
  // The age-ordered collapsing queue and its priority-encoder select are
  // REUSED, not reinvented, from `IssueUnitCollapsing`
  // (v4/exu/issue-units/issue-unit-age-ordered.scala): `vacants`, `shamts_oh`
  // with `SaturatingCounterOH`, the `will_be_valid` array, the `uops` array,
  // the `clear` derivation, the registered `is_available` dispatch-readiness
  // calculation and the `port_issued`/`uop_issued` select loop are all taken
  // verbatim, with the same names, so this file diffs cleanly against the
  // scalar one. `IQ_V_LOAD`/`IQ_V_STORE` reuse the identical structure
  // unchanged; `IQ_V_ALU` reuses it too and adds only the per-entry
  // eligibility term the slot already computes (part 5).
  //
  // ASSUMPTION / SPEC NOTE: the dependencies section says this module "binds
  // to" IssueUnitCollapsing's `SaturatingCounterOH` helper "rather than
  // re-spelling it". That helper is a method nested inside
  // `IssueUnitCollapsing`'s class body (package boom.v4.exu), not a
  // top-level/companion-object declaration -- there is no public symbol in
  // a different package to bind to. The logic section's own instruction
  // ("taken verbatim... with the same names") is followed literally instead:
  // the identical helper is copied below under the same name, exactly like
  // every other baseline-derived name in this file.
  def SaturatingCounterOH(count_oh: UInt, inc: Bool, max: Int): UInt = {
    val next = Wire(UInt(max.W))
    next := count_oh
    when (count_oh === 0.U && inc) {
      next := 1.U
    } .elsewhen (!count_oh(max - 1) && inc) {
      next := (count_oh << 1.U)
    }
    next
  }

  val dis_uops = Array.fill(dispatchWidth) { Wire(new MicroOp()) }

  // =========================================================================
  // ---- 2. The dispatch cycle: pre-correct the SCALAR half only ----
  // =========================================================================
  for (w <- 0 until dispatchWidth) {
    dis_uops(w) := io.dis_uops(w).bits
    dis_uops(w).iw_issued              := false.B
    dis_uops(w).iw_issued_partial_agen := false.B
    dis_uops(w).iw_issued_partial_dgen := false.B
    dis_uops(w).iw_p1_bypass_hint      := false.B
    dis_uops(w).iw_p2_bypass_hint      := false.B
    dis_uops(w).iw_p3_bypass_hint      := false.B

    // Baseline's dispatch-cycle wakeup pre-correction against
    // prs1/prs2, kept as-is in structure but sourced from the INT network
    // unconditionally, and from the FP network too when isAluQueue &&
    // lrs1_rtype === RT_FLT -- the same int-vs-FP partitioning VecIssueSlot
    // applies at the slot level (two physically separate networks with
    // independent PRN numbering spaces that can coincidentally share a
    // numeric value).
    val int_prs1_matches = io.int_wakeup_ports.map { wu => wu.bits.uop.pdst === io.dis_uops(w).bits.prs1 }
    val int_prs1_wakeups = (io.int_wakeup_ports zip int_prs1_matches).map { case (wu, m) => wu.valid && m }
    val int_prs1_rebusys = (io.int_wakeup_ports zip int_prs1_matches).map { case (wu, m) => wu.bits.rebusy && m }
    val int_bypassables       = io.int_wakeup_ports.map(_.bits.bypassable)
    val int_speculative_masks = io.int_wakeup_ports.map(_.bits.speculative_mask)

    val prs2_matches = io.int_wakeup_ports.map { wu => wu.bits.uop.pdst === io.dis_uops(w).bits.prs2 }
    val prs2_wakeups = (io.int_wakeup_ports zip prs2_matches).map { case (wu, m) => wu.valid && m }
    val prs2_rebusys = (io.int_wakeup_ports zip prs2_matches).map { case (wu, m) => wu.bits.rebusy && m }

    when (int_prs1_wakeups.reduce(_ || _) && io.dis_uops(w).bits.lrs1_rtype === RT_FIX) {
      dis_uops(w).prs1_busy := false.B
      dis_uops(w).iw_p1_speculative_child := Mux1H(int_prs1_wakeups, int_speculative_masks)
      dis_uops(w).iw_p1_bypass_hint := Mux1H(int_prs1_wakeups, int_bypassables)
    }
    when ((int_prs1_rebusys.reduce(_ || _) || ((io.child_rebusys & io.dis_uops(w).bits.iw_p1_speculative_child) =/= 0.U)) &&
      io.dis_uops(w).bits.lrs1_rtype === RT_FIX) {
      dis_uops(w).prs1_busy := true.B
    }
    when (prs2_wakeups.reduce(_ || _)) {
      dis_uops(w).prs2_busy := false.B
      dis_uops(w).iw_p2_speculative_child := Mux1H(prs2_wakeups, int_speculative_masks)
      dis_uops(w).iw_p2_bypass_hint := Mux1H(prs2_wakeups, int_bypassables)
    }
    when ((prs2_rebusys.reduce(_ || _) || ((io.child_rebusys & io.dis_uops(w).bits.iw_p2_speculative_child) =/= 0.U)) &&
      io.dis_uops(w).bits.lrs2_rtype === RT_FIX) {
      dis_uops(w).prs2_busy := true.B
    }

    // The FP network reaches IQ_V_ALU alone: a `.vf`-form op or `vfmv.*.f`
    // sources one scalar FP register on prs1. No FP-side rebusy/retraction
    // path, deliberately -- baseline's re-busy machinery exists for the
    // fixed, speculatable load-hit latency on the integer network only.
    if (isAluQueue) {
      val fp_prs1_matches = io.fp_wakeup_ports.get.map { wu => wu.bits.uop.pdst === io.dis_uops(w).bits.prs1 }
      val fp_prs1_wakeups = (io.fp_wakeup_ports.get zip fp_prs1_matches).map { case (wu, m) => wu.valid && m }
      val fp_bypassables       = io.fp_wakeup_ports.get.map(_.bits.bypassable)
      val fp_speculative_masks = io.fp_wakeup_ports.get.map(_.bits.speculative_mask)
      when (fp_prs1_wakeups.reduce(_ || _) && io.dis_uops(w).bits.lrs1_rtype === RT_FLT) {
        dis_uops(w).prs1_busy := false.B
        dis_uops(w).iw_p1_speculative_child := Mux1H(fp_prs1_wakeups, fp_speculative_masks)
        dis_uops(w).iw_p1_bypass_hint := Mux1H(fp_prs1_wakeups, fp_bypassables)
      }
    }

    // The VL comparators, added in the same shape as baseline's
    // pred_wakeup_port handling: clear dis_uops(w).pvl_busy when ANY VL lane
    // matches -- scanned over ALL numVlWakeupPorts lanes, not lane 0 alone.
    val vl_hit = io.vl_wakeup.map(l => l.valid && l.bits === io.dis_uops(w).bits.pvl.get).reduce(_ || _)
    when (vl_hit) {
      dis_uops(w).pvl_busy.get := false.B
    }

    // NO VECTOR PRE-CORRECTION HAPPENS HERE, and that asymmetry is
    // deliberate: VecGroupReady applies the group-done match to the LOADED
    // value in the load cycle, so a group-done arriving in the dispatch
    // cycle is captured at the slot's port. Correcting it here too would put
    // the same single-shot correction in two places and a one-cycle
    // disagreement is a lost wakeup and a permanent hang.

    //@req-spec-cii.d1
    // Vector arithmetic reaches the coprocessor only through iq_v_alu, and
    // the routing is checked at this boundary rather than trusted.
    // MicroOp.iq_type is a Vec(IQ_SZ, Bool()) bitmask, which is what lets a
    // shared OP.v name two queues at once (part 8) with no new field.
    assert(!io.dis_uops(w).valid ||
      (io.dis_uops(w).bits.is_vec.get && io.dis_uops(w).bits.iq_type(iqType)),
      "VecIssueUnit: dispatched op missing is_vec or iq_type(iqType)")

    // ===> BASELINE'S iqType-SPECIFIC DISPATCH FIXUPS ARE ON A REJECT LIST
    // (A49) -- each entry below is a block a generic "reuse baseline's
    // dispatch path" generator would copy, and each is actively wrong here:
    // (a) IQ_MEM's FP-store fixup (prs2_busy := false / lrs2_rtype := RT_X
    //     on uses_stq && lrs2_rtype===RT_FLT) -- in a vector store, prs2
    //     CARRIES THE STRIDE and must be waited on and delivered intact.
    // (b) IQ_MEM's `prs3_busy := false` (and IQ_FP's prs1 mirror) -- vector
    //     store data is a VRF group read on port R3, never a scalar operand,
    //     so prs3 has no vector meaning here.
    // (c) IQ_UNQ's `prs2 := Cat(fp_rm, fp_typ)` / `pimm := mem_size`
    //     rewrites -- prs2 (the stride) and pimm are read by vector AGEN and
    //     must not be repurposed.
    // None of (a)-(c) is reproduced.

    // What IS copied is baseline's `iqType != IQ_ALU` clause -- always taken
    // here since iqType is one of IQ_V_LOAD/IQ_V_STORE/IQ_V_ALU, never
    // IQ_ALU, but kept behind the same Scala `if` as baseline for a clean
    // diff. This is how VecIssueSlot's "no ppred term" claim is discharged
    // without a port: a vector OP.v is never an SFB shadow.
    if (iqType != IQ_ALU) {
      assert(!(io.dis_uops(w).bits.ppred_busy && io.dis_uops(w).valid),
        "VecIssueUnit: dispatched vector op has ppred_busy set")
      dis_uops(w).ppred_busy := false.B
    }
  }

  //@req-spec-core.f12
  //@req-spec-core.h6
  // ---- 8a. Shared instructions: two slots, two queues, no coupling ----
  //
  // A shared instruction (an is_shared segmented load or store) occupies TWO
  // slots -- one in iq_v_alu, one in iq_v_load/iq_v_store -- while sharing a
  // SINGLE ROB entry. Dispatch presents the same uop to both instances in the
  // same cycle, iq_type naming both queues (checked above), and EACH
  // instance allocates one slot and grants it once through its OWN select
  // below. There is deliberately NO cross-queue signal of any kind anywhere
  // in this file: no shared grant, no cross-queue kill, no cross-queue
  // ready. The two halves rendezvous only on the pvtmp group's group-done on
  // the ordinary vector wakeup network (inside VecIssueSlot), and the single
  // ROB entry's completion is tracked by the ROB's own "other half pending"
  // flag -- neither of which this module touches.

  // =========================================================================
  // ---- Issue Table ----
  // =========================================================================

  val slots = (0 until numIssueSlots) map { i =>
    Module(new VecIssueSlot(iqType, numIntWakeupPorts, pnrGate, numFpWakeupPorts))
  }
  val issue_slots = VecInit(slots.map(_.io))

  for (i <- 0 until numIssueSlots) {
    issue_slots(i).int_wakeup_ports := io.int_wakeup_ports
    if (isAluQueue) {
      issue_slots(i).fp_wakeup_ports.get := io.fp_wakeup_ports.get
    }
    issue_slots(i).vl_wakeup      := io.vl_wakeup
    issue_slots(i).vec_group_done := io.vec_group_done
    issue_slots(i).child_rebusys  := io.child_rebusys
    issue_slots(i).squash_grant   := io.squash_grant
    issue_slots(i).brupdate       := io.brupdate
    issue_slots(i).kill           := io.flush_pipeline

    //@req-spec-issue.d6
    //@req-spec-issue.d7
    //@req-spec-cii.i5
    //@req-spec-cii.i13
    // pnrGate forwards rob_pnr_idx/rob_head_idx unmodified to every slot and
    // reads neither in this module. For a segmented STORE, this same
    // per-entry past-PNR gate (computed inside the slot) is the WHOLE of the
    // AGEN-before-coprocessor-half dependency: the LSU half's AGEN clears
    // `unsafe` on the shared ROB entry, the PNR advances past it, and only
    // then is the coprocessor half (sitting in iq_v_alu, same rob_idx)
    // eligible -- through this SAME gate every other CII op passes, with no
    // translation-complete signal and no private path.
    //@req-spec-issue.d15
    // There is no circular wait: the eligibility term reads ONLY
    // rob_pnr_idx, rob_head_idx and the entry's own rob_idx -- nothing the
    // coprocessor half produces. This module is never given (and must never
    // be given) a signal that would let it wait on the other half.
    if (pnrGate) {
      issue_slots(i).rob_pnr_idx.get  := io.rob_pnr_idx.get
      issue_slots(i).rob_head_idx.get := io.rob_head_idx.get
    }
  }

  for (w <- 0 until issueWidth) {
    io.iss_uops(w).valid := false.B
  }

  //@req-spec-issue.e14
  //@req-spec-cii.i7
  // ---- 8b. IQ_V_ALU is NOT a head-only FIFO ----
  //
  // No term anywhere in this file's select (below) references index 0,
  // rob_head_idx (the SLOTS read it, only for wrap resolution inside
  // IsOlder -- see the forwarding above), or "the oldest valid entry".
  // Program-order issue is not required: rename has resolved every register
  // dependence before an op crosses the CII, vtype/vl/vstart/vxrm ride the
  // per-instruction CII issue packet, vxsat/fflags accumulate at commit in
  // ROB order, and CII tags are opaque with results already allowed to
  // return out of order. A head-only queue would let a segmented store's
  // coprocessor half (which can stall for a 256-element translation pass)
  // block every younger vector arithmetic op. Ordering between the vector
  // queues, and between vector and scalar work, is enforced exclusively by
  // the ROB -- not by this module and not by any cross-queue signal.

  assert(PopCount(issue_slots.map(s => s.grant)) <= issueWidth.U,
    "[vec-issue] window giving out too many grants.")

  // =========================================================================
  // ---- Figure out how much to shift entries by (baseline, unchanged) ----
  // =========================================================================
  val nSlowSlots = params.numSlowEntries
  val nFastSlots = numIssueSlots - nSlowSlots
  require(nFastSlots >= dispatchWidth)
  require(nFastSlots <= numIssueSlots)

  val vacants = issue_slots.map(s => !(s.valid)) ++ io.dis_uops.map(_.valid).map(!_.asBool)
  val shamts_oh = Wire(Vec(numIssueSlots + dispatchWidth, UInt(dispatchWidth.W)))
  shamts_oh(0) := 0.U
  for (i <- 1 until numIssueSlots + dispatchWidth) {
    val shift = if (i < nSlowSlots) (dispatchWidth min 1 + (i * (dispatchWidth - 1) / nSlowSlots).toInt) else dispatchWidth
    if (dispatchWidth == 1 || shift == 1) {
      shamts_oh(i) := vacants.take(i).reduce(_ || _)
    } else {
      shamts_oh(i) := SaturatingCounterOH(shamts_oh(i - 1), vacants(i - 1), shift)
    }
  }

  // =========================================================================
  // ---- 3. The collapse move, and the side channel that must ride with it ----
  // =========================================================================

  // which entries' uops will still be next cycle? (not being issued and vacated)
  val will_be_valid = (0 until numIssueSlots).map(i => issue_slots(i).will_be_valid) ++
                       (0 until dispatchWidth).map(i => io.dis_uops(i).valid &&
                                                          !dis_uops(i).exception &&
                                                          !dis_uops(i).is_fence &&
                                                          !dis_uops(i).is_fencei)

  // ===> IMPLEMENTED AS ONE MUX OVER ONE COMBINED WIRE: a local two-field
  // bundle {uop, member_rdy}, filled from the two parallel arrays with the
  // SAME indexing, so the collapse `when` below selects both fields from the
  // SAME index in one shot -- no future edit can add a term to one path and
  // forget the other. Desynchronised, the migrated slot would reload its
  // matchers with the WRONG slot's partial readiness and wait on group-dones
  // that already fired: a silent permanent hang with no assertion anywhere.
  class CollapseEntry(implicit p: Parameters) extends Bundle {
    val uop        = new MicroOp()
    val member_rdy = new VecMemberRdy
  }

  val combined = Wire(Vec(numIssueSlots + dispatchWidth, new CollapseEntry))
  for (i <- 0 until numIssueSlots) {
    combined(i).uop        := issue_slots(i).out_uop
    combined(i).member_rdy := issue_slots(i).out_member_rdy
  }
  for (w <- 0 until dispatchWidth) {
    combined(numIssueSlots + w).uop        := dis_uops(w)
    combined(numIssueSlots + w).member_rdy := io.dis_member_rdy(w)
  }

  for (i <- 0 until numIssueSlots) {
    issue_slots(i).in_uop.valid    := false.B
    issue_slots(i).in_uop.bits     := combined(i + 1).uop
    issue_slots(i).in_member_rdy   := combined(i + 1).member_rdy
    for (j <- 1 to dispatchWidth by 1) {
      when (shamts_oh(i + j) === (1 << (j - 1)).U) {
        issue_slots(i).in_uop.valid  := will_be_valid(i + j)
        issue_slots(i).in_uop.bits   := combined(i + j).uop
        issue_slots(i).in_member_rdy := combined(i + j).member_rdy

        // Tracing (part 12, event 1): "dispatch accept". i+j >= numIssueSlots
        // means this index is dispatch-sourced (not an internal shift from
        // another slot) -- exactly the cycle a fresh dispatch lands in slot
        // i. Reuses this same select condition, so it can never disagree
        // with the real datapath.
        if (i + j >= numIssueSlots) {
          when (will_be_valid(i + j)) {
            VecTrace.trace("VecIssueUnit", "dispatch_accept", combined(i + j).uop, Seq(
              ("slot_idx",  i.U),
              ("is_shared", combined(i + j).uop.is_shared.get.asUInt)))
          }
        }
      }
    }
    issue_slots(i).clear := shamts_oh(i) =/= 0.U
  }

  // ===> SEAM, DECISION D2: this queue's per-lane io.dis_uops(w).ready IS the
  // back-pressure CompactingDispatcher consumes natively (not a
  // `ready := true.B` hack with fullness re-routed elsewhere) -- unchanged
  // from baseline.
  val is_available = Reg(Vec(nFastSlots, Bool()))
  is_available := VecInit((nSlowSlots until numIssueSlots).map(i =>
    (!issue_slots(i).will_be_valid || issue_slots(i).clear) && !(issue_slots(i).in_uop.valid)))
  for (w <- 0 until dispatchWidth) {
    io.dis_uops(w).ready := RegNext(PopCount(is_available) > w.U(log2Ceil(nFastSlots).W) + PopCount(io.dis_uops.map(_.fire)))
    assert(!io.dis_uops(w).ready || (shamts_oh(w + numIssueSlots) >> w) =/= 0.U)
  }

  // =========================================================================
  // ---- 5. `eligible`, not `request` -- the per-entry past-PNR gate ----
  // =========================================================================

  //@req-spec-issue.e13
  //@req-spec-cii.d3
  //@req-spec-cii.d4
  // The array the encoder scans (part 4, below) is eligibles, NOT requests.
  // On a pnrGate slot the slot has already formed
  // `request && IsOlder(slot_uop.rob_idx, rob_pnr_idx, rob_head_idx)` --
  // computed per entry, on every slot, and not only the head -- so every op
  // this unit hands the CII is individually non-speculative, RoCC-style. On
  // a non-pnrGate slot eligible is request unchanged, so the load/store
  // queues issue speculatively and ordering/replay stay the LSU's business.
  // `requests` is exported for TRACING and assertions only, never read by
  // the select.
  //
  // ===> BASELINE'S SNI BLOCK (issue_slot_past_pnr / issue_past_pnr /
  // can_issue_sni / enableConservativeSNI) IS NOT RE-ADDED (A49, sharpened):
  // it is a DIFFERENT mechanism (speculative non-interference, permissive by
  // construction), and one of its terms is actively wrong here --
  // `| (rob_idx === rob_pnr_idx)` would admit the UNRESOLVED entry
  // rob_pnr_idx itself names (rob.scala:531), defeating the entire reason
  // pnrGate exists. The eligibility test used is the STRICT IsOlder the slot
  // computes, nothing OR-ed onto it.
  val eligibles = issue_slots.map(s => s.eligible)
  val requests  = issue_slots.map(s => s.request)

  //@req-spec-cii.d13
  // A squashed IQ_V_ALU entry never issues, via two mechanisms: kill (=
  // io.flush_pipeline) clears slot_valid before any select can see it, and a
  // branch kill cannot reach an ELIGIBLE entry at all -- a past-PNR entry's
  // br_mask is necessarily clear, since the PNR cannot sweep past an
  // unresolved branch (starts_unsafe). Checked here rather than left as a
  // comment: on a pnrGate instance, an eligible entry is never
  // IsKilledByBranch.
  if (pnrGate) {
    for (i <- 0 until numIssueSlots) {
      assert(!(eligibles(i) && IsKilledByBranch(io.brupdate, false.B, issue_slots(i).iss_uop)),
        "VecIssueUnit: a pnrGate-eligible entry was killed by a branch")
    }
  }

  // =========================================================================
  // ---- 4. Select: the oldest READY entry, out of order among ready ops ----
  // =========================================================================

  //@req-spec-issue.e11
  //@req-spec-issue.e12
  //@req-spec-cii.d2
  // The select is baseline's priority encoder over the slots in age order,
  // index 0 oldest, granting the OLDEST ELIGIBLE entry and freely SKIPPING a
  // not-ready older one -- out-of-order issue among ready ops. All three
  // queues do this: iq_v_load/iq_v_store because the vector LSU is
  // out-of-order, and iq_v_alu because age-ordered collapsing with a
  // per-entry past-PNR gate is exactly the specified policy. The collapse
  // keeps the array age ordered, so "first in index order" IS "oldest", and
  // the gate narrows which entries the encoder may pick without changing the
  // encoder itself.
  val port_issued = Array.fill(issueWidth) { false.B }

  val iss_select_mask = Array.ofDim[Boolean](issueWidth, numIssueSlots)
  if (params.useFullIssueSel) {
    for (w <- 0 until issueWidth) {
      for (i <- 0 until numIssueSlots) {
        iss_select_mask(w)(i) = true
      }
    }
  } else {
    for (w <- 0 until issueWidth) {
      for (i <- 0 until numIssueSlots) {
        iss_select_mask(w)(i) = (w % 2) == (i % 2)
      }
      iss_select_mask(w)(0) = true
    }
  }

  val iss_uops = Wire(Vec(issueWidth, Valid(new MicroOp)))
  for (w <- 0 until issueWidth) {
    iss_uops(w).valid := false.B
    iss_uops(w).bits  := DontCare
  }

  for (i <- 0 until numIssueSlots) {
    issue_slots(i).grant := false.B
    var uop_issued = false.B

    for (w <- 0 until issueWidth) {
      val fu_code_match = (issue_slots(i).iss_uop.fu_code zip io.fu_types(w)).map {
        case (r, c) => r && c
      }.reduce(_ || _)

      val can_allocate = fu_code_match && iss_select_mask(w)(i).B

      //@req-spec-issue.e8
      //@req-spec-cii.d12
      //@req-spec-cii.i9
      // The grant conjunction is exactly `eligibles(i) && can_allocate` --
      // this unit contributes NO readiness term of its own. `eligibles(i)`
      // already carries every operand class ANDed together inside the slot
      // (scalar feeders, .vf/.vx, pvl, pvs1/pvs2/pvs3/pvm), plus, on
      // iq_v_alu, the past-PNR term -- "past-PNR AND operands ready", both
      // required and neither sufficient. The coprocessor half of a shared op
      // is issued by this SAME conjunction, with no additional condition.
      when (eligibles(i) && !uop_issued && can_allocate && !port_issued(w)) {
        issue_slots(i).grant := true.B
        iss_uops(w).valid := true.B
        iss_uops(w).bits  := issue_slots(i).iss_uop

        // Tracing (part 12, event 2): the grant, with the issue lane and,
        // on the store queue, which of AGEN/DGEN was offered.
        val grantExtra: Seq[(String, Bits)] =
          if (isStoreQueue)
            Seq(("iss_lane", w.U),
                ("fc_agen",  issue_slots(i).iss_uop.fu_code(FC_AGEN).asUInt),
                ("fc_dgen",  issue_slots(i).iss_uop.fu_code(FC_DGEN).asUInt))
          else
            Seq(("iss_lane", w.U))
        VecTrace.trace("VecIssueUnit", "grant", issue_slots(i).iss_uop, grantExtra)
      }
      val was_port_issued_yet = port_issued(w)
      port_issued(w) = (eligibles(i) && !uop_issued && can_allocate) | port_issued(w)

      //@req-spec-core.f10
      //@req-spec-core.f11
      //@req-spec-issue.e6
      //@req-spec-issue.e7
      // A non-shared vector OP.v occupies exactly ONE slot in exactly ONE
      // queue and is granted ONCE: dispatch (part 2 above) writes one slot in
      // the queue its iq_type bit names, and `uop_issued` here admits that
      // slot at most once per cycle. `spec-core.f11` is kept with its ID and
      // its reading annotated (decision D12 case 2): "each issue slot must
      // be granted once" is loosely worded and FALSE AS LITERALLY WRITTEN --
      // a vector STORE slot is granted TWICE (once for AGEN, once for DGEN,
      // via VecStoreDgenPath inside the slot), and squash_grant/speculative
      // re-busy can retract and re-grant the SAME select on the scalar half.
      // Neither is a second scheduling decision; both are baseline machinery
      // or VecStoreDgenPath's own contract, not re-implemented here.
      uop_issued = (eligibles(i) && can_allocate && !was_port_issued_yet) | uop_issued
    }
  }

  //@req-spec-core.f14
  // An OP.v is ALLOCATED and SELECTED once: this unit has no second issue
  // stage, no second priority-encoder select, no re-select of an
  // already-granted entry and no intermediate buffer between iss_uops and
  // the execution units. iss_uops is the grant, driven combinationally in
  // the request cycle -- load-bearing for the store queue, whose AGEN/DGEN
  // path bits (fu_code(FC_AGEN)/fu_code(FC_DGEN)) must already be visible on
  // iss_uop when fu_code_match and the grant above resolve; a register
  // between request and grant would desynchronise them.
  io.iss_uops := iss_uops
  when (io.squash_grant) {
    io.iss_uops.map { u => u.valid := false.B }
  }

  // Tracing (part 12, event 3): on pnrGate instances only, the rising edge
  // of request && !eligible -- the past-PNR stall. Without this line a PNR
  // stall and an operand stall look identical from outside the queue in a
  // cosim log.
  if (pnrGate) {
    for (i <- 0 until numIssueSlots) {
      val pnr_stall      = requests(i) && !eligibles(i)
      val pnr_stall_prev = RegNext(pnr_stall, false.B)
      when (pnr_stall && !pnr_stall_prev) {
        VecTrace.trace("VecIssueUnit", "pnr_stall", issue_slots(i).iss_uop)
      }
    }
  }

  // =========================================================================
  // ---- 9. The element-queue reservation is an ASSERTION, not a stall ----
  // =========================================================================
  //
  // For iq_v_load/iq_v_store, granting an entry additionally requires that
  // the dispatch-time element-queue RESERVATION exists. It always does --
  // capacity was claimed in program order at dispatch and VecQueueReservation
  // refuses the dispatch otherwise -- so it is CHECKED here, not waited on:
  // no reservation port exists on this module (a Bool from the reservation
  // unit into this select would be a stall condition that can never be
  // false: dead logic that reads as a dependency).
  //
  // ===> AND NOTHING HERE CONSULTS A `busy` FROM `VecLsu` (ground rule 6).
  // This unit has no port to the vector LSU at all. Its ONLY back-pressure
  // is fu_types, credit- and capacity-metered per execution resource, not
  // instruction-scoped state.
  if (isLoadQueue) {
    for (i <- 0 until numIssueSlots) {
      assert(!issue_slots(i).valid || issue_slots(i).out_uop.uses_ldq,
        "VecIssueUnit: a valid iq_v_load entry lacks uses_ldq")
    }
  }
  if (isStoreQueue) {
    for (i <- 0 until numIssueSlots) {
      assert(!issue_slots(i).valid || issue_slots(i).out_uop.uses_stq,
        "VecIssueUnit: a valid iq_v_store entry lacks uses_stq")
    }
  }

  // =========================================================================
  // ---- 11. Remaining assertions and elaboration checks ----
  // =========================================================================

  // A grant only ever lands on a slot whose eligible was high this cycle.
  for (i <- 0 until numIssueSlots) {
    assert(!issue_slots(i).grant || eligibles(i),
      "VecIssueUnit: a grant landed on a slot that was not eligible")
  }

  // An isStoreQueue instance's slots elaborate no rdy_vold (part 13 /
  // decision D6): a store has no stale_pvdest reader.
  require(!isStoreQueue || slots.forall(_.rdy_vold.isEmpty),
    "VecIssueUnit: an isStoreQueue instance's slots must elaborate no rdy_vold")
}

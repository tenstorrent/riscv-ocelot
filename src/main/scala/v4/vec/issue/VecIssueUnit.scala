//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Issue Units (Step 6)
//------------------------------------------------------------------------------
//
// STANDALONE duplicates of the scalar issue-unit machinery
// (exu/issue-units/issue-unit.scala, issue-unit-age-ordered.scala) that
// instantiate VecIssueSlot and fan the vector / VL wakeup networks to every
// slot. Per ground-rule 3 the scalar files are left untouched; we duplicate.
//
// Two flavors:
//   - VecIssueUnitCollapsing  : the age-ordered collapsing window, used for the
//                               vector LOAD and STORE queues (IQ_V_LOAD/STORE).
//                               Age-ordered; does NOT gate on PNR.
//   - VecAluIssueUnit         : an in-order FIFO used for the vector ALU queue
//                               (IQ_V_ALU). Head-only select, gated past PNR.
//
// All units are DORMANT in Step 6: core.scala supplies fu_types=0 and ties off
// every wakeup port, so nothing ever grants.

package boom.v4.vec.issue

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.util._
import boom.v4.exu.{BrUpdateInfo, Wakeup, IssueParams}
import boom.v4.vec.rename.{VecGroupDone, VlWakeup}

//------------------------------------------------------------------------------
// Shared IO bundle for the vector issue units. Mirrors the abstract IssueUnit io
// (issue-unit.scala:46-67) plus the two Caracal wakeup networks.
//------------------------------------------------------------------------------
class VecIssueUnitIO(
  val params: IssueParams,
  val numWakeupPorts: Int)
  (implicit p: Parameters) extends BoomBundle
{
  val dis_uops         = Vec(params.dispatchWidth, Flipped(Decoupled(new MicroOp)))

  val iss_uops         = Output(Vec(params.issueWidth, Valid(new MicroOp())))
  val wakeup_ports     = Flipped(Vec(numWakeupPorts, Valid(new Wakeup)))
  val pred_wakeup_port = Flipped(Valid(UInt(log2Ceil(ftqSz).W)))

  val vec_wakeup_ports = Flipped(Vec(numVecWakeupPorts, Valid(new VecGroupDone)))
  val vl_wakeup_ports  = Flipped(Vec(numVlWakeupPorts,  Valid(new VlWakeup)))

  val child_rebusys    = Input(UInt(aluWidth.W))

  // tell the issue unit what each execution pipeline has in terms of functional units
  val fu_types         = Input(Vec(params.issueWidth, Vec(FC_SZ, Bool())))

  val brupdate         = Input(new BrUpdateInfo())
  val flush_pipeline   = Input(Bool())
  val squash_grant     = Input(Bool())

  val tsc_reg          = Input(UInt(xLen.W))

  // For the speculative non-interference (SNI) implementation.
  val rob_head    = Input(UInt(robAddrSz.W))
  val rob_pnr_idx = Input(UInt(robAddrSz.W))
}

//------------------------------------------------------------------------------
// Common abstract base so the factory can return a single type whose `io` the
// caller (core.scala, Step 6) can drive uniformly.
//------------------------------------------------------------------------------
abstract class AbstractVecIssueUnit(
  val params: IssueParams,
  val numWakeupPorts: Int)
  (implicit p: Parameters)
  extends BoomModule
{
  val numIssueSlots = params.numEntries
  val issueWidth    = params.issueWidth
  val iqType        = params.iqType
  val dispatchWidth = params.dispatchWidth

  val io = IO(new VecIssueUnitIO(params, numWakeupPorts))
}

//==============================================================================
// Age-ordered collapsing vector issue unit (IQ_V_LOAD / IQ_V_STORE).
//
// DUPLICATES IssueUnitCollapsing (issue-unit-age-ordered.scala:23-285): same
// dispatch-time wakeup pre-clear, same collapse/shift/select machinery, but
// instantiates VecIssueSlot and fans the vector/VL wakeup networks. This unit is
// age-ordered and does NOT gate on PNR (loads/stores order through the LSU).
//==============================================================================
class VecIssueUnitCollapsing(
  params: IssueParams,
  numWakeupPorts: Int)
  (implicit p: Parameters)
  extends AbstractVecIssueUnit(params, numWakeupPorts)
{
  //-------------------------------------------------------------
  // Set up the dispatch uops. Duplicate issue-unit-age-ordered.scala:34-115,
  // keeping the scalar wakeup pre-clear and adding the vector pre-clear for
  // consistency. We drop the rebusy/speculative-child terms (no rebusy on the
  // vector network) and the IQ_MEM/IQ_FP/IQ_UNQ scalar-specific store fixups.
  val dis_uops = Array.fill(dispatchWidth) {Wire(new MicroOp())}
  for (w <- 0 until dispatchWidth) {
    dis_uops(w) := io.dis_uops(w).bits
    dis_uops(w).iw_issued := false.B
    dis_uops(w).iw_issued_partial_agen := false.B
    dis_uops(w).iw_issued_partial_dgen := false.B
    dis_uops(w).iw_p1_bypass_hint := false.B
    dis_uops(w).iw_p2_bypass_hint := false.B
    dis_uops(w).iw_p3_bypass_hint := false.B

    // Scalar wakeups on dispatch (issue-unit-age-ordered.scala:45-83).
    val prs1_matches = io.wakeup_ports.map { wu => wu.bits.uop.pdst === io.dis_uops(w).bits.prs1 }
    val prs2_matches = io.wakeup_ports.map { wu => wu.bits.uop.pdst === io.dis_uops(w).bits.prs2 }
    val prs3_matches = io.wakeup_ports.map { wu => wu.bits.uop.pdst === io.dis_uops(w).bits.prs3 }
    val prs1_wakeups = (io.wakeup_ports zip prs1_matches).map { case (wu,m) => wu.valid && m }
    val prs2_wakeups = (io.wakeup_ports zip prs2_matches).map { case (wu,m) => wu.valid && m }
    val prs3_wakeups = (io.wakeup_ports zip prs3_matches).map { case (wu,m) => wu.valid && m }
    when (prs1_wakeups.reduce(_||_)) { dis_uops(w).prs1_busy := false.B }
    when (prs2_wakeups.reduce(_||_)) { dis_uops(w).prs2_busy := false.B }
    when (prs3_wakeups.reduce(_||_)) { dis_uops(w).prs3_busy := false.B }
    when (io.pred_wakeup_port.valid && io.pred_wakeup_port.bits === io.dis_uops(w).bits.ppred) {
      dis_uops(w).ppred_busy := false.B
    }

    // Vector group-done pre-clear (mirrors the scalar pre-clear). A source group
    // whose every active member is woken this cycle clears its collapsed busy
    // bit. Uses the same memberWoken logic as the slot.
    val mc = boom.v4.vec.rename.VecEmul.memberCount(io.dis_uops(w).bits.v_emul)
    def memberWoken(prn: UInt): Bool =
      io.vec_wakeup_ports.map { wk =>
        wk.valid && (0 until boom.v4.vec.rename.VecEmul.MAX_MEMBERS).map { j =>
          wk.bits.mask(j) && (wk.bits.prn(j) === prn)
        }.reduce(_||_)
      }.reduce(_||_)
    def groupAllWoken(grp: Vec[UInt]): Bool =
      (0 until boom.v4.vec.rename.VecEmul.MAX_MEMBERS).map { j =>
        !(j.U < mc) || memberWoken(grp(j))
      }.reduce(_&&_)
    when (groupAllWoken(io.dis_uops(w).bits.pvs1_grp)) { dis_uops(w).pvs1_busy := false.B }
    when (groupAllWoken(io.dis_uops(w).bits.pvs2_grp)) { dis_uops(w).pvs2_busy := false.B }
    when (groupAllWoken(io.dis_uops(w).bits.pvs3_grp)) { dis_uops(w).pvs3_busy := false.B }
    when (memberWoken(io.dis_uops(w).bits.pvm)) { dis_uops(w).pvm_busy := false.B }
    when (io.vl_wakeup_ports.map { wk => wk.valid && (wk.bits.pvl === io.dis_uops(w).bits.pvl) }.reduce(_||_)) {
      dis_uops(w).pvl_busy := false.B
    }
  }

  //-------------------------------------------------------------
  // Issue Table. Duplicate issue-unit-age-ordered.scala:122-132.
  val slots = (0 until numIssueSlots) map { w =>
    Module(new VecIssueSlot(numWakeupPorts)) }
  val issue_slots = VecInit(slots.map(_.io))

  for (i <- 0 until numIssueSlots) {
    issue_slots(i).wakeup_ports     := io.wakeup_ports
    issue_slots(i).vec_wakeup_ports := io.vec_wakeup_ports
    issue_slots(i).vl_wakeup_ports  := io.vl_wakeup_ports
    issue_slots(i).pred_wakeup_port := io.pred_wakeup_port
    issue_slots(i).child_rebusys    := io.child_rebusys
    issue_slots(i).squash_grant     := io.squash_grant
    issue_slots(i).brupdate         := io.brupdate
    issue_slots(i).kill             := io.flush_pipeline
  }

  for (w <- 0 until issueWidth) {
    io.iss_uops(w).valid := false.B
  }

  assert (PopCount(issue_slots.map(s => s.grant)) <= issueWidth.U, "[vec-issue] window giving out too many grants.")

  //-------------------------------------------------------------
  // Figure out how much to shift entries by. Duplicate :147-179.
  val nSlowSlots = params.numSlowEntries
  val nFastSlots = numIssueSlots - nSlowSlots

  require (nFastSlots >= dispatchWidth)
  require (nFastSlots <= numIssueSlots)

  val vacants = issue_slots.map(s => !(s.valid)) ++ io.dis_uops.map(_.valid).map(!_.asBool)
  val shamts_oh = Wire(Vec(numIssueSlots+dispatchWidth, UInt(width=dispatchWidth.W)))
  def SaturatingCounterOH(count_oh:UInt, inc: Bool, max: Int): UInt = {
     val next = Wire(UInt(width=max.W))
     next := count_oh
     when (count_oh === 0.U && inc) {
       next := 1.U
     } .elsewhen (!count_oh(max-1) && inc) {
       next := (count_oh << 1.U)
     }
     next
  }
  shamts_oh(0) := 0.U
  for (i <- 1 until numIssueSlots + dispatchWidth) {
    val shift = if (i < nSlowSlots) (dispatchWidth min 1 + (i * (dispatchWidth-1)/nSlowSlots).toInt) else dispatchWidth
    if (dispatchWidth == 1 || shift == 1) {
      shamts_oh(i) := vacants.take(i).reduce(_||_)
    } else {
      shamts_oh(i) := SaturatingCounterOH(shamts_oh(i-1), vacants(i-1), shift)
    }
  }

  //-------------------------------------------------------------
  // Duplicate :184-201.
  val will_be_valid = (0 until numIssueSlots).map(i => issue_slots(i).will_be_valid) ++
                      (0 until dispatchWidth).map(i => io.dis_uops(i).valid &&
                                                        !dis_uops(i).exception &&
                                                        !dis_uops(i).is_fence &&
                                                        !dis_uops(i).is_fencei)

  val uops = issue_slots.map(s=>s.out_uop) ++ dis_uops.map(s=>s)
  for (i <- 0 until numIssueSlots) {
    issue_slots(i).in_uop.valid := false.B
    issue_slots(i).in_uop.bits  := uops(i+1)
    for (j <- 1 to dispatchWidth by 1) {
      when (shamts_oh(i+j) === (1 << (j-1)).U) {
        issue_slots(i).in_uop.valid := will_be_valid(i+j)
        issue_slots(i).in_uop.bits  := uops(i+j)
      }
    }
    issue_slots(i).clear        := shamts_oh(i) =/= 0.U
  }

  //-------------------------------------------------------------
  // Dispatch/Entry Logic. Duplicate :209-216.
  val is_available = Reg(Vec(nFastSlots, Bool()))
  is_available := VecInit((nSlowSlots until numIssueSlots).map(i =>
    (!issue_slots(i).will_be_valid || issue_slots(i).clear) && !(issue_slots(i).in_uop.valid)))
  for (w <- 0 until dispatchWidth) {
    io.dis_uops(w).ready := RegNext(PopCount(is_available) >
                                    w.U(log2Ceil(nFastSlots).W) + PopCount(io.dis_uops.map(_.fire)))
    assert (!io.dis_uops(w).ready || (shamts_oh(w+numIssueSlots) >> w) =/= 0.U)
  }

  //-------------------------------------------------------------
  // Issue Select Logic. Duplicate :219-284, MINUS the PNR (SNI) gate -- this
  // age-ordered LS unit does not gate on PNR.
  val requests = issue_slots.map(s => s.request)
  val port_issued = Array.fill(issueWidth){Bool()}
  for (w <- 0 until issueWidth) {
    port_issued(w) = false.B
  }

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
        case (r,c) => r && c
      } .reduce(_||_)

      val can_allocate = fu_code_match && iss_select_mask(w)(i).B

      when (requests(i) && !uop_issued && can_allocate && !port_issued(w)) {
        issue_slots(i).grant := true.B
        iss_uops(w).valid := true.B
        iss_uops(w).bits  := issue_slots(i).iss_uop
      }
      val was_port_issued_yet = port_issued(w)
      port_issued(w) = (requests(i) && !uop_issued && can_allocate) | port_issued(w)
      uop_issued = (requests(i) && can_allocate && !was_port_issued_yet) | uop_issued
    }
  }
  io.iss_uops := iss_uops
  when (io.squash_grant) {
    io.iss_uops.map { u => u.valid := false.B }
  }
}

//==============================================================================
// In-order FIFO vector ALU issue unit (IQ_V_ALU).
//
// NOT collapsing: a ring of VecIssueSlots with a head/tail pointer. Uops enqueue
// at the tail in program order from dis_uops; the head (and up to issueWidth
// in-order successors) is the only candidate for issue. The head is granted only
// when it requests AND is past PNR (issue-unit-age-ordered.scala:253 idiom).
// Branch-killed entries are dropped (their valid is squashed) before the head.
//==============================================================================
class VecAluIssueUnit(
  params: IssueParams,
  numWakeupPorts: Int)
  (implicit p: Parameters)
  extends AbstractVecIssueUnit(params, numWakeupPorts)
{
  require (isPow2(numIssueSlots), "[vec-alu-issue] FIFO depth must be a power of 2")
  val idxSz = log2Ceil(numIssueSlots)

  //-------------------------------------------------------------
  // Slots laid out as a ring. head points at the oldest entry; tail at the next
  // free slot. count tracks occupancy.
  val slots = (0 until numIssueSlots) map { w =>
    Module(new VecIssueSlot(numWakeupPorts)) }
  val issue_slots = VecInit(slots.map(_.io))

  val head  = RegInit(0.U(idxSz.W))
  val tail  = RegInit(0.U(idxSz.W))
  val count = RegInit(0.U((idxSz+1).W))

  // Common per-slot wiring. In-order slots never shift, so clear is only set on
  // dequeue/kill (handled below); brupdate/kill drive the standard squash.
  for (i <- 0 until numIssueSlots) {
    issue_slots(i).wakeup_ports     := io.wakeup_ports
    issue_slots(i).vec_wakeup_ports := io.vec_wakeup_ports
    issue_slots(i).vl_wakeup_ports  := io.vl_wakeup_ports
    issue_slots(i).pred_wakeup_port := io.pred_wakeup_port
    issue_slots(i).child_rebusys    := io.child_rebusys
    issue_slots(i).squash_grant     := io.squash_grant
    issue_slots(i).brupdate         := io.brupdate
    issue_slots(i).kill             := io.flush_pipeline
    issue_slots(i).grant            := false.B
    issue_slots(i).in_uop.valid     := false.B
    issue_slots(i).in_uop.bits      := DontCare
    issue_slots(i).clear            := false.B
  }

  //-------------------------------------------------------------
  // Dispatch / enqueue. Walk the dispatch lanes in program order; lane w writes
  // the slot at (tail + #older-firing-lanes). dis_uops are accepted while there
  // is room. Pre-clear scalar/vector/VL wakeups at dispatch as the collapsing
  // unit does, for consistency.
  val dis_uops = Array.fill(dispatchWidth) {Wire(new MicroOp())}
  for (w <- 0 until dispatchWidth) {
    dis_uops(w) := io.dis_uops(w).bits
    dis_uops(w).iw_issued := false.B
    dis_uops(w).iw_issued_partial_agen := false.B
    dis_uops(w).iw_issued_partial_dgen := false.B

    val prs1_matches = io.wakeup_ports.map { wu => wu.bits.uop.pdst === io.dis_uops(w).bits.prs1 }
    val prs2_matches = io.wakeup_ports.map { wu => wu.bits.uop.pdst === io.dis_uops(w).bits.prs2 }
    val prs3_matches = io.wakeup_ports.map { wu => wu.bits.uop.pdst === io.dis_uops(w).bits.prs3 }
    when ((io.wakeup_ports zip prs1_matches).map { case (wu,m) => wu.valid && m }.reduce(_||_)) {
      dis_uops(w).prs1_busy := false.B }
    when ((io.wakeup_ports zip prs2_matches).map { case (wu,m) => wu.valid && m }.reduce(_||_)) {
      dis_uops(w).prs2_busy := false.B }
    when ((io.wakeup_ports zip prs3_matches).map { case (wu,m) => wu.valid && m }.reduce(_||_)) {
      dis_uops(w).prs3_busy := false.B }

    val mc = boom.v4.vec.rename.VecEmul.memberCount(io.dis_uops(w).bits.v_emul)
    def memberWoken(prn: UInt): Bool =
      io.vec_wakeup_ports.map { wk =>
        wk.valid && (0 until boom.v4.vec.rename.VecEmul.MAX_MEMBERS).map { j =>
          wk.bits.mask(j) && (wk.bits.prn(j) === prn)
        }.reduce(_||_)
      }.reduce(_||_)
    def groupAllWoken(grp: Vec[UInt]): Bool =
      (0 until boom.v4.vec.rename.VecEmul.MAX_MEMBERS).map { j =>
        !(j.U < mc) || memberWoken(grp(j))
      }.reduce(_&&_)
    when (groupAllWoken(io.dis_uops(w).bits.pvs1_grp)) { dis_uops(w).pvs1_busy := false.B }
    when (groupAllWoken(io.dis_uops(w).bits.pvs2_grp)) { dis_uops(w).pvs2_busy := false.B }
    when (groupAllWoken(io.dis_uops(w).bits.pvs3_grp)) { dis_uops(w).pvs3_busy := false.B }
    when (memberWoken(io.dis_uops(w).bits.pvm)) { dis_uops(w).pvm_busy := false.B }
    when (io.vl_wakeup_ports.map { wk => wk.valid && (wk.bits.pvl === io.dis_uops(w).bits.pvl) }.reduce(_||_)) {
      dis_uops(w).pvl_busy := false.B
    }
  }

  // Dispatch-accept readiness must NOT depend on the incoming valids: otherwise
  // dis_uops(w).ready -> (BasicDispatcher cross-queue ready-AND) -> dis_hazards ->
  // dis_fire -> dis_uops(w).valid closes a combinational loop (firtool CheckCombLoops).
  // Compute ready purely from the REGISTERED free space: lane w is accepted iff
  // there are at least w+1 free slots, conservatively reserving one slot for every
  // older lane in the bundle. Hole compaction (when an older lane is invalid) is
  // handled by the older_fires write-index routing below, so this only costs an
  // occasional 1-cycle dispatch stall, never correctness.
  val space = numIssueSlots.U - count
  for (w <- 0 until dispatchWidth) {
    io.dis_uops(w).ready := (w + 1).U <= space
  }
  val num_enq = PopCount((0 until dispatchWidth).map(w => io.dis_uops(w).fire))

  // Route each firing lane into tail + (#older firing lanes).
  for (w <- 0 until dispatchWidth) {
    val older_fires = if (w == 0) 0.U else PopCount((0 until w).map(x => io.dis_uops(x).fire))
    val widx = (tail + older_fires)(idxSz-1, 0)
    when (io.dis_uops(w).fire) {
      issue_slots(widx).in_uop.valid := true.B
      issue_slots(widx).in_uop.bits  := dis_uops(w)
    }
  }

  //-------------------------------------------------------------
  // Head-only select. Examine the head and up to issueWidth-1 in-order
  // successors. Each candidate must (a) be valid, (b) request, (c) be past PNR.
  for (w <- 0 until issueWidth) {
    io.iss_uops(w).valid := false.B
    io.iss_uops(w).bits  := DontCare
  }

  // A killed head entry must be dropped before it can block the FIFO. A global
  // flush resets the pointers (below); a per-branch kill is detected via the
  // slot's own will_be_valid (it folds in IsKilledByBranch), so a branch-killed
  // head stops being valid and we dequeue (drop) it below to keep order.
  val cand_idx = (0 until issueWidth).map(w => (head + w.U)(idxSz-1, 0))

  // num_deq: how many in-order head entries leave this cycle (granted, or
  // killed). We grant strictly in order: candidate w can issue only if every
  // older candidate also issued (or was empty/killed -> dequeued).
  val deqs   = Wire(Vec(issueWidth, Bool()))
  var prev_ok = true.B          // all older candidates resolved (issued or dropped)
  for (w <- 0 until issueWidth) {
    val idx     = cand_idx(w)
    val occupied = w.U < count
    val slot    = issue_slots(idx)
    val past_pnr = IsOlder(slot.iss_uop.rob_idx, io.rob_pnr_idx, io.rob_head) ||
                   (slot.iss_uop.rob_idx === io.rob_pnr_idx)
    val can_issue_sni = past_pnr || !enableConservativeSNI.B

    val fu_code_match = (slot.iss_uop.fu_code zip io.fu_types(w)).map {
      case (r,c) => r && c
    }.reduce(_||_)

    // A counted slot is DEAD (branch-killed/flushed) once its valid register has
    // cleared and nothing is being written into it this cycle. (An entry written
    // THIS cycle has slot.valid==false but in_uop.valid==true -- it is alive, not
    // dead, and simply not yet issuable; we leave it for next cycle.) A dead head
    // must be dropped so the in-order FIFO does not stall forever behind it; the
    // slot's own register already squashed it (IsKilledByBranch in the slot), so
    // here we only advance the ring pointers past it.
    val dead  = occupied && !slot.valid && !slot.in_uop.valid
    val drop  = dead
    val grant = occupied && !dead && prev_ok && slot.request && fu_code_match && can_issue_sni && !io.squash_grant

    // Architect invariant: the in-order V-ALU FIFO force-clears a granted head on
    // dequeue, which would corrupt a two-issue store (AGEN+DGEN from one slot). By
    // routing, the ALU queue is arith-only, so no entry should ever be both an AGEN
    // and a DGEN op. Build-enforce it on every counted candidate.
    when (occupied) {
      assert(!(slot.iss_uop.fu_code(FC_AGEN) && slot.iss_uop.fu_code(FC_DGEN)),
        "[valu] AGEN+DGEN op in the in-order V-ALU FIFO")
    }

    deqs(w)   := occupied && prev_ok && (grant || drop)

    slot.grant := grant
    when (grant) {
      io.iss_uops(w).valid := true.B
      io.iss_uops(w).bits  := slot.iss_uop
    }

    // Successors stay in order only if this candidate left the head (granted or
    // dropped). A stalled head (occupied, !grant, !drop) blocks all younger.
    prev_ok = prev_ok && occupied && (grant || drop)
  }

  val num_deq = PopCount(deqs)

  // Clear dequeued head slots so they can be re-enqueued.
  for (w <- 0 until issueWidth) {
    when (deqs(w)) {
      issue_slots(cand_idx(w)).clear := true.B
    }
  }

  //-------------------------------------------------------------
  // Pointer / count update. On flush, reset the FIFO. Branch-killed entries are
  // dropped only when they reach the head (handled by deqs above); the count
  // bookkeeping below stays exact because num_deq counts every head removal.
  when (io.flush_pipeline) {
    head  := 0.U
    tail  := 0.U
    count := 0.U
  } .otherwise {
    head  := (head + num_deq)(idxSz-1, 0)
    tail  := (tail + num_enq)(idxSz-1, 0)
    count := count + num_enq - num_deq
  }

  assert (count <= numIssueSlots.U, "[vec-alu-issue] FIFO overflow.")
}

//==============================================================================
// Factory: pick the collapsing unit for the LS queues, the FIFO for the ALU
// queue, keyed on params.iqType.
//==============================================================================
object VecIssueUnit
{
  def apply(
    params: IssueParams,
    numWakeupPorts: Int)
    (implicit p: Parameters): AbstractVecIssueUnit = {
    if (params.iqType == IQ_V_ALU) {
      Module(new VecAluIssueUnit(params, numWakeupPorts))
    } else {
      // IQ_V_LOAD / IQ_V_STORE
      Module(new VecIssueUnitCollapsing(params, numWakeupPorts))
    }
  }
}

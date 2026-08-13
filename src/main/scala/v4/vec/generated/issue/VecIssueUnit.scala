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
  val dis_member_rdy = Input(Vec(dispatchWidth, new VecMemberRdy))
  val iss_uops = Output(Vec(issueWidth, Valid(new MicroOp())))
  val int_wakeup_ports = Flipped(Vec(numIntWakeupPorts, Valid(new Wakeup)))
  val fp_wakeup_ports = if (isAluQueue) Some(Flipped(Vec(numFpWakeupPorts, Valid(new Wakeup)))) else None
  val vl_wakeup = Flipped(Vec(numVlWakeupPorts, Valid(UInt(vlPregSz.W))))
  val vec_group_done = Flipped(Vec(numVecWbPorts, Valid(new VecGroupDone)))
  val child_rebusys = Input(UInt(aluWidth.W))
  val fu_types = Input(Vec(issueWidth, Vec(FC_SZ, Bool())))
  val brupdate       = Input(new BrUpdateInfo())
  val flush_pipeline = Input(Bool())
  val squash_grant   = Input(Bool())
  val rob_pnr_idx  = if (pnrGate) Some(Input(UInt(robAddrSz.W))) else None
  val rob_head_idx = if (pnrGate) Some(Input(UInt(robAddrSz.W))) else None
}

class VecIssueUnit(
  val params:            IssueParams,
  val numIntWakeupPorts: Int,
  val pnrGate:           Boolean = false,
  val numFpWakeupPorts:  Int = 0,
  val numVecWbPorts:     Int)(implicit p: Parameters) extends BoomModule
{
  // =========================================================================
  // ---- Elaboration-time Booleans; no hardware ever compares `iqType` ----
  // =========================================================================

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
  require(usingRVV, "VecIssueUnit: elaborates only under usingRVV")

  val dispatchWidth = params.dispatchWidth
  val issueWidth    = params.issueWidth
  val numIssueSlots = params.numEntries
  require(numIssueSlots - params.numSlowEntries >= dispatchWidth,
    "VecIssueUnit: numEntries - numSlowEntries must be >= dispatchWidth")

  val io = IO(new VecIssueUnitIO(dispatchWidth, issueWidth, numIntWakeupPorts,
    numFpWakeupPorts, numVecWbPorts, isAluQueue, pnrGate))

  // =========================================================================
  // ---- Port-count binding-site checks (parameters section) ----
  // =========================================================================

  require(numVecWbPorts == vectorParams.numVecWbPorts,
    s"VecIssueUnit: numVecWbPorts ($numVecWbPorts) must equal " +
    s"vectorParams.numVecWbPorts (${vectorParams.numVecWbPorts})")
  require(numVecWbPorts == io.vec_group_done.length,
    "VecIssueUnit: numVecWbPorts must equal io.vec_group_done.length")
  require(io.vl_wakeup.length == numVlWakeupPorts,
    s"VecIssueUnit: io.vl_wakeup width (${io.vl_wakeup.length}) must equal " +
    s"numVlWakeupPorts ($numVlWakeupPorts)")
  require(pnrGate || io.rob_pnr_idx.isEmpty,
    "VecIssueUnit: a non-pnrGate instance must elaborate no rob_pnr_idx port")

  //@req-spec-core.e12
  //@req-spec-issue.e9
  //@req-spec-issue.e10
  // =========================================================================
  // ---- 1. The frame: baseline's collapsing queue, copied ----
  // =========================================================================

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

    val vl_hit = io.vl_wakeup.map(l => l.valid && l.bits === io.dis_uops(w).bits.pvl_src.get).reduce(_ || _)
    when (vl_hit) {
      dis_uops(w).pvl_busy.get := false.B
    }

    //@req-spec-cii.d1
    assert(!io.dis_uops(w).valid ||
      (io.dis_uops(w).bits.is_vec.get && io.dis_uops(w).bits.iq_type(iqType)),
      "VecIssueUnit: dispatched op missing is_vec or iq_type(iqType)")

    if (iqType != IQ_ALU) {
      assert(!(io.dis_uops(w).bits.ppred_busy && io.dis_uops(w).valid),
        "VecIssueUnit: dispatched vector op has ppred_busy set")
      dis_uops(w).ppred_busy := false.B
    }
  }

  //@req-spec-core.f12
  //@req-spec-core.h6
  // ---- 8a. Shared instructions: two slots, two queues, no coupling ----
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
    //@req-spec-issue.d15
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

  // ---- 3. The collapse move, and the side channel that must ride with it ----

  val will_be_valid = (0 until numIssueSlots).map(i => issue_slots(i).will_be_valid) ++
                       (0 until dispatchWidth).map(i => io.dis_uops(i).valid &&
                                                          !dis_uops(i).exception &&
                                                          !dis_uops(i).is_fence &&
                                                          !dis_uops(i).is_fencei)

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

  val is_available = Reg(Vec(nFastSlots, Bool()))
  is_available := VecInit((nSlowSlots until numIssueSlots).map(i =>
    (!issue_slots(i).will_be_valid || issue_slots(i).clear) && !(issue_slots(i).in_uop.valid)))
  for (w <- 0 until dispatchWidth) {
    io.dis_uops(w).ready := RegNext(PopCount(is_available) > w.U(log2Ceil(nFastSlots).W) + PopCount(io.dis_uops.map(_.fire)))
    assert(!io.dis_uops(w).ready || (shamts_oh(w + numIssueSlots) >> w) =/= 0.U)
  }

  //@req-spec-issue.e13
  //@req-spec-cii.d3
  //@req-spec-cii.d4
  // =========================================================================
  // ---- 5. `eligible`, not `request` -- the per-entry past-PNR gate ----
  // =========================================================================

  val eligibles = issue_slots.map(s => s.eligible)
  val requests  = issue_slots.map(s => s.request)

  //@req-spec-cii.d13
  if (pnrGate) {
    for (i <- 0 until numIssueSlots) {
      assert(!(eligibles(i) && IsKilledByBranch(io.brupdate, false.B, issue_slots(i).iss_uop)),
        "VecIssueUnit: a pnrGate-eligible entry was killed by a branch")
    }
  }

  //@req-spec-issue.e11
  //@req-spec-issue.e12
  //@req-spec-cii.d2
  // =========================================================================
  // ---- 4. Select: the oldest READY entry, out of order among ready ops ----
  // =========================================================================

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
      when (eligibles(i) && !uop_issued && can_allocate && !port_issued(w)) {
        issue_slots(i).grant := true.B
        iss_uops(w).valid := true.B
        iss_uops(w).bits  := issue_slots(i).iss_uop
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
      uop_issued = (eligibles(i) && can_allocate && !was_port_issued_yet) | uop_issued
    }
  }

  //@req-spec-core.f14
  io.iss_uops := iss_uops
  when (io.squash_grant) {
    io.iss_uops.map { u => u.valid := false.B }
  }

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

  for (i <- 0 until numIssueSlots) {
    assert(!issue_slots(i).grant || eligibles(i),
      "VecIssueUnit: a grant landed on a slot that was not eligible")
  }

  require(!isStoreQueue || slots.forall(_.rdy_vold.isEmpty),
    "VecIssueUnit: an isStoreQueue instance's slots must elaborate no rdy_vold")
}

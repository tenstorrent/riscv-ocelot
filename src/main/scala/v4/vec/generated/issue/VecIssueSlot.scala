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
import chisel3.layer
import freechips.rocketchip.util.PlusArg

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.exu.{BrUpdateInfo, Wakeup}
import boom.v4.util.{IsKilledByBranch, IsOlder, UpdateBrMask}
import boom.v4.lsu.GetRealLSQIdx
import boom.v4.vec.generated.{VecGroupDone, VecMemberRdy, VecTrace}
import boom.v4.vec.formal.BoomSvaLayer

// GENERATED from src/main/nlhdl/vec/issue/VecIssueSlot.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

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
  val int_wakeup_ports = Flipped(Vec(numIntWakeupPorts, Valid(new Wakeup)))

  //@req-spec-issue.g9
  //@req-spec-issue.g10
  val fp_wakeup_ports = if (isAluSlot) Some(Flipped(Vec(numFpWakeupPorts, Valid(new Wakeup)))) else None

  //@req-spec-rename.h16
  //@req-spec-issue.g6
  val vl_wakeup = Flipped(Vec(numVlWakeupPorts, Valid(UInt(vlPregSz.W))))

  val vec_group_done = Flipped(Vec(vectorParams.numVecWbPorts, Valid(new VecGroupDone)))

  //@req-spec-issue.g15
  // The rename busy table's LIVE view, broadcast unmodified. Each readiness
  // tracker indexes it by its own prns, so a `group_done` pulse missed in the
  // cycle it fired is recovered from the level on any later cycle.
  val live_busytable = Input(Bits(numVecPhysRegs.W))

  //@req-spec-lsu.k2b
  // Per-STQ-entry "AGEN capture done", from VecLsu. Store slots only.
  val st_dgen_base_vld = if (isAluSlot) None else Some(Input(UInt(numStqEntries.W)))
  val st_dgen_ready    = if (isAluSlot) None else Some(Input(Bool()))

  val in_member_rdy  = Input(new VecMemberRdy)
  val out_member_rdy = Output(new VecMemberRdy)

  val rob_pnr_idx  = if (pnrGate) Some(Input(UInt(robAddrSz.W))) else None
  val rob_head_idx = if (pnrGate) Some(Input(UInt(robAddrSz.W))) else None

  val eligible = Output(Bool())
}

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

  val active_uop: MicroOp = Mux(io.in_uop.valid, io.in_uop.bits, slot_uop)

  val active_valid: Bool = io.in_uop.valid || slot_valid

  //@req-spec-issue.g17
  // THE READINESS TRACKERS FOLLOW THE REGISTER OCCUPANT, NOT `active_uop`.
  // `request`, `iss_uop` and `scalar_operands_ready` are all `slot_uop`, so the
  // vector readiness that gates them must be `slot_uop` too; feeding the trackers
  // `active_uop` made them describe the INCOMING occupant on every collapse-shift
  // cycle and let an op issue against the next op's operand state (see
  // VecGroupReady's gate comment for the conv1d-vector measurement).
  // Nothing is lost by dropping the incoming uop here: on the fill cycle the
  // tracker loads `io.in_member_rdy`, which the source slot exported WITH its own
  // same-cycle hits folded in, and from the next cycle `slot_uop` IS that uop, so
  // its PRNs are compared from then on. A slot taking a FRESH DISPATCH has
  // `slot_valid` low that cycle, so `used` is false and `request` was already low.
  val track_uop: MicroOp = slot_uop
  val track_valid: Bool  = slot_valid

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
  val int_prs1_matches = io.int_wakeup_ports.map { w => w.bits.uop.pdst === slot_uop.prs1 }
  val int_prs1_wakeups = (io.int_wakeup_ports zip int_prs1_matches).map { case (w, m) => w.valid && m }
  val int_prs1_rebusys = (io.int_wakeup_ports zip int_prs1_matches).map { case (w, m) => w.bits.rebusy && m }
  val int_bypassables       = io.int_wakeup_ports.map(_.bits.bypassable)
  val int_speculative_masks = io.int_wakeup_ports.map(_.bits.speculative_mask)

  val prs2_matches = io.int_wakeup_ports.map { w => w.bits.uop.pdst === slot_uop.prs2 }
  val prs2_wakeups = (io.int_wakeup_ports zip prs2_matches).map { case (w, m) => w.valid && m }
  val prs2_rebusys = (io.int_wakeup_ports zip prs2_matches).map { case (w, m) => w.bits.rebusy && m }

  when (int_prs1_wakeups.reduce(_ || _) && slot_uop.lrs1_rtype === RT_FIX) {
    next_uop.prs1_busy := false.B
    next_uop.iw_p1_speculative_child := Mux1H(int_prs1_wakeups, int_speculative_masks)
    next_uop.iw_p1_bypass_hint := Mux1H(int_prs1_wakeups, int_bypassables)
  }
  //@req-spec-vrf.e1
  //@req-spec-vrf.e2
  //@req-spec-vrf.e5
  //@req-spec-vrf.e6
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
  }

  // Assert that a slot never sees RT_FLT on lrs1_rtype unless isAluSlot.
  if (!isAluSlot) {
    assert(!slot_valid || slot_uop.lrs1_rtype =/= RT_FLT,
      "VecIssueSlot: RT_FLT lrs1_rtype seen outside an ALU slot")
  }

  //@req-spec-core.f5
  //@req-spec-rename.g2
  val scalar_operands_ready = !slot_uop.prs1_busy && !slot_uop.prs2_busy && !slot_uop.pvl_busy.get

  //@req-spec-issue.g17
  // ⚠ DO NOT ISSUE ON A CYCLE THIS SLOT IS BEING RE-FILLED. `request`, `iss_uop`
  // and `scalar_operands_ready` all describe the REGISTER occupant (`slot_uop`),
  // but every VECTOR readiness bit describes `active_uop` -- which is the INCOMING
  // occupant whenever `io.in_uop.valid`, because the five VecGroupReady instances
  // take their `prns`, `members` and `used` from it (they must: those inputs set up
  // NEXT cycle's tracking) and because `group_all_rdy` reads `member_rdy_next`,
  // which on a load cycle IS the incoming occupant's readiness.
  // So on a collapse-shift cycle the outgoing occupant can be granted on the
  // strength of the INCOMING occupant's operands. Measured on conv1d-vector
  // (MegaBoom): `vmacc.vv v10,v14,v12` was granted with `vs3_mem=1` while its own
  // `emul=2` -- the incoming uop's member count -- so the AND-reduce ignored
  // members 1..7, and it read a `pvs3` whose member 0 was still busy, ~3000 trace
  // lines before its producer's writeback. The result was arithmetically perfect
  // over a stale old-dest, which is why nothing downstream could catch it.
  // VecGroupReady already documents the mirror image of this hazard for its
  // `out_member_rdy` EXPORT (":103") and fixes it there; the ISSUE GATE was left
  // reading the incoming occupant.
  // ⚠ AND THE ONE-LINE FIX DOES NOT WORK. Gating `request` on `!io.in_uop.valid`
  // (tried, measured) moves conv1d-vector's first failure EARLIER, to step 8142 on
  // VecScalarOperandRead's `lateWritebackObserved` assertion: delaying a grant by
  // the shift cycle lands the base-GPR read in the window that module's forward
  // logic is explicitly NOT calibrated for (see its :100 block). So this needs the
  // REGISTER/INCOMING SPLIT done properly instead:
  //   - `member_hit` must be compared against the REGISTER occupant's prns for the
  //     gate and the current occupant's update, and against the INCOMING prns for
  //     the readiness arriving this cycle -- two comparator sets, which is what
  //     collapsing everything onto `active_uop` was avoiding;
  //   - `group_all_rdy` must use `members`/`used` of the REGISTER occupant;
  //   - and note the related hole in VecGroupReady:103, where `out_member_rdy`
  //     drops `member_hit` on a load cycle (`&& !io.load`), so a wakeup arriving
  //     for the OUTGOING occupant on a shift cycle is observed by nobody -- a lost
  //     group_done pulse, i.e. a stuck slot, not a wrong value.
  // That is a design change with an nlhdl spec update, not a one-line guard.

  // ---- 3. Vector operands: five matchers, one bit each ----
  //@req-spec-issue.g11
  val rdy_vs1 = Module(new VecGroupReady(isMask = false))
  val rdy_vs2 = Module(new VecGroupReady(isMask = false))
  val rdy_vs3 = Module(new VecGroupReady(isMask = false))
  val rdy_vm  = Module(new VecGroupReady(isMask = true))
  //@req-spec-issue.g35
  //@req-spec-issue.g36
  //@req-spec-issue.g37
  val rdy_vold: Option[VecGroupReady] = if (!isStoreSlot) Some(Module(new VecGroupReady(isMask = false))) else None

  rdy_vs1.io.group_done := io.vec_group_done
  rdy_vs2.io.group_done := io.vec_group_done
  rdy_vs3.io.group_done := io.vec_group_done
  rdy_vm.io.group_done  := io.vec_group_done
  rdy_vold.foreach(_.io.group_done := io.vec_group_done)

  // One indexed read per tracker member off the broadcast table.
  def liveBusyOf(prns: Seq[UInt]): Vec[Bool] = VecInit(prns.map(p => io.live_busytable(p)))

  rdy_vs1.io.load := io.in_uop.valid
  rdy_vs2.io.load := io.in_uop.valid
  rdy_vs3.io.load := io.in_uop.valid
  rdy_vm.io.load  := io.in_uop.valid
  rdy_vold.foreach(_.io.load := io.in_uop.valid)

  rdy_vs1.io.prns    := track_uop.pvs1.get
  rdy_vs1.io.members.get := track_uop.v_emul.get
  rdy_vs2.io.prns    := track_uop.pvs2.get
  rdy_vs2.io.members.get := track_uop.v_emul.get
  rdy_vm.io.prns     := VecInit(track_uop.pvm.get)

  //@req-spec-issue.g29
  //@req-spec-issue.g30
  //@req-spec-issue.g31
  rdy_vm.io.used := track_valid && track_uop.v_is_masked.get

  rdy_vs1.io.live_busy := liveBusyOf(rdy_vs1.io.prns)
  rdy_vs2.io.live_busy := liveBusyOf(rdy_vs2.io.prns)
  rdy_vs3.io.live_busy := liveBusyOf(rdy_vs3.io.prns)
  rdy_vm.io.live_busy  := liveBusyOf(rdy_vm.io.prns)
  rdy_vold.foreach(r => r.io.live_busy := liveBusyOf(r.io.prns))

  rdy_vs1.io.in_member_rdy := io.in_member_rdy.vs1_rdy
  rdy_vs2.io.in_member_rdy := io.in_member_rdy.vs2_rdy
  rdy_vm.io.in_member_rdy  := VecInit(io.in_member_rdy.vm_rdy)

  io.out_member_rdy.vs1_rdy := rdy_vs1.io.out_member_rdy
  io.out_member_rdy.vs2_rdy := rdy_vs2.io.out_member_rdy
  io.out_member_rdy.vm_rdy  := rdy_vm.io.out_member_rdy(0)

  //@req-spec-issue.g32
  //@req-spec-issue.c14
  //@req-spec-lsu.l2
  rdy_vs1.io.used := track_valid && track_uop.v_uses_vs1.get
  rdy_vs2.io.used := track_valid && track_uop.v_uses_vs2.get

  // ---- 7. pvtmp: how the consumer half of a shared op wakes ----
  //@req-spec-issue.c11
  //@req-spec-rob.d12
  //@req-spec-lsu.l5
  val vs3_selects_tmp = if (isAluSlot) track_uop.is_shared.get && track_uop.uses_ldq else false.B
  if (isAluSlot) {
    rdy_vs3.io.prns := Mux(vs3_selects_tmp, track_uop.pvtmp.get, track_uop.pvs3.get)
    rdy_vs3.io.members.get := track_uop.v_emul.get
    //@req-spec-issue.g32
    rdy_vs3.io.used := track_valid && (vs3_selects_tmp || track_uop.v_uses_vs3.get)

    rdy_vs3.io.in_member_rdy := Mux(vs3_selects_tmp, io.in_member_rdy.vtmp_rdy, io.in_member_rdy.vs3_rdy)
    io.out_member_rdy.vs3_rdy  := Mux(vs3_selects_tmp, io.in_member_rdy.vs3_rdy, rdy_vs3.io.out_member_rdy)
    io.out_member_rdy.vtmp_rdy := Mux(vs3_selects_tmp, rdy_vs3.io.out_member_rdy, io.in_member_rdy.vtmp_rdy)
  } else if (isLoadSlot) {
    rdy_vs3.io.prns    := track_uop.pvs3.get
    rdy_vs3.io.members.get := track_uop.v_emul.get
    //@req-spec-issue.g32
    rdy_vs3.io.used := track_valid && track_uop.v_uses_vs3.get && !track_uop.is_shared.get
    rdy_vs3.io.in_member_rdy := io.in_member_rdy.vs3_rdy
    io.out_member_rdy.vs3_rdy  := rdy_vs3.io.out_member_rdy
    io.out_member_rdy.vtmp_rdy := io.in_member_rdy.vtmp_rdy
  }

  // ---- 5. pvl and vtype ----
  //@req-spec-decode.i6
  //@req-spec-issue.h2
  val vl_hit = io.vl_wakeup.map(w => w.valid && w.bits === slot_uop.pvl_src.get).reduce(_ || _)
  when (vl_hit) {
    next_uop.pvl_busy.get := false.B
  }

  //@req-spec-issue.g7
  //@req-spec-issue.f11

  //@req-spec-issue.f15

  // ---- 4. rdy_vold: `used`, and part 11's conservative gate ----
  //@req-spec-issue.g38
  rdy_vold.foreach { m =>
    m.io.prns    := track_uop.stale_pvdest.get
    m.io.members.get := track_uop.v_emul.get
    //@req-spec-issue.g39
    val v_needs_old_dest = track_uop.v_tail_undist.get || track_uop.v_mask_undist.get
    // A/B escape hatch: restores the unconditional arm this gate replaced, so one
    // simv measures both sides of the change with no RTL-version confound.
    val vOldDepAll = PlusArg("vec_vold_dep_all", 0,
      "Arm rdy_vold on every vector load, ignoring vta/vma (pre-fix behaviour)",
      width = 1).orR
    m.io.used := track_valid && track_uop.dst_rtype === RT_VEC &&
      (if (isLoadSlot) (v_needs_old_dest || vOldDepAll) else true.B)
    m.io.in_member_rdy := io.in_member_rdy.vold_rdy
    io.out_member_rdy.vold_rdy := m.io.out_member_rdy
  }
  if (isStoreSlot) {
    io.out_member_rdy.vold_rdy := io.in_member_rdy.vold_rdy
  }

  rdy_vold.foreach { m =>
    assert(!m.io.used || (track_uop.v_emul.get >= 1.U && track_uop.v_emul.get <= maxVecMembers.U),
      "VecIssueSlot: rdy_vold used but v_emul out of range 1..maxVecMembers")
  }

  // ---- Vector operand busy mirror (part 3) ----
  next_uop.pvs1_busy.get := !rdy_vs1.io.ready
  next_uop.pvs2_busy.get := !rdy_vs2.io.ready
  next_uop.pvm_busy.get  := !rdy_vm.io.ready
  if (isAluSlot) {
    when (vs3_selects_tmp) {
      next_uop.pvtmp_busy.get := !rdy_vs3.io.ready
    } .otherwise {
      next_uop.pvs3_busy.get := !rdy_vs3.io.ready
    }
  } else if (isLoadSlot) {
    next_uop.pvs3_busy.get := !rdy_vs3.io.ready
  }

  // ---- 6. request and eligibility, and the store slot's second grant path ----
  val request  = Wire(Bool())
  val eligible = Wire(Bool())

  if (isStoreSlot) {
    val dgen_path = Module(new VecStoreDgenPath)

    dgen_path.io.slot_valid := slot_valid
    dgen_path.io.grant      := io.grant
    dgen_path.io.squash_grant := io.squash_grant
    dgen_path.io.slot_uop   := slot_uop

    // End-of-cycle values: on the fill cycle the DGEN group belongs to the
    // INCOMING uop, so it is selected off io.in_uop here rather than through
    // dgen_path, whose slot_uop cone also drives iss_uop.fu_code.
    val dgen_in_is_pvtmp = io.in_uop.bits.is_shared.get
    val dgen_sel_is_pvtmp = Mux(io.in_uop.valid, dgen_in_is_pvtmp,
      dgen_path.io.dgen_operand_is_pvtmp)

    rdy_vs3.io.prns := Mux(io.in_uop.valid,
      Mux(dgen_in_is_pvtmp, io.in_uop.bits.pvtmp.get, io.in_uop.bits.pvs3.get),
      dgen_path.io.dgen_operand)
    rdy_vs3.io.members.get := Mux(io.in_uop.valid, io.in_uop.bits.v_emul.get,
      dgen_path.io.dgen_operand_members)
    rdy_vs3.io.used := active_valid
    rdy_vs3.io.in_member_rdy := Mux(dgen_sel_is_pvtmp,
      io.in_member_rdy.vtmp_rdy, io.in_member_rdy.vs3_rdy)
    io.out_member_rdy.vs3_rdy  := Mux(dgen_sel_is_pvtmp,
      io.in_member_rdy.vs3_rdy, rdy_vs3.io.out_member_rdy)
    io.out_member_rdy.vtmp_rdy := Mux(dgen_sel_is_pvtmp,
      rdy_vs3.io.out_member_rdy, io.in_member_rdy.vtmp_rdy)

    when (dgen_sel_is_pvtmp) {
      next_uop.pvtmp_busy.get := !rdy_vs3.io.ready
    } .otherwise {
      next_uop.pvs3_busy.get := !rdy_vs3.io.ready
    }

    val agen_operands_ready = scalar_operands_ready && rdy_vs1.io.ready && rdy_vs2.io.ready && rdy_vm.io.ready
    dgen_path.io.agen_operands_ready := agen_operands_ready
    dgen_path.io.dgen_operand_ready  := rdy_vs3.io.ready
    dgen_path.io.agen_rebusied := rebusied_prs1
    //@req-spec-lsu.k2b
    // Indexed by THIS slot's stq entry; the bit is set by the AGEN pass's capture
    // (`stOpPulse`) and cleared by VecLsu on the DGEN grant it gates, so it is a
    // one-shot permission per store rather than a level that could admit two.
    dgen_path.io.agen_captured := io.st_dgen_base_vld.get(GetRealLSQIdx(slot_uop.stq_idx))
    dgen_path.io.dgen_sink_ready := io.st_dgen_ready.get

    //@req-spec-issue.g19
    //@req-spec-issue.g20
    request := dgen_path.io.agen_request || dgen_path.io.dgen_request

    io.iss_uop := slot_uop
    //@req-spec-cii.e3
    // THE ISSUED UOP CARRIES THIS CYCLE'S RESOLVED BRANCH MASK, NOT THE REGISTER'S.
    // `slot_uop.br_mask` is one b1 behind: a branch resolving THIS cycle is already
    // out of `next_uop.br_mask` (that is `GetNewBrMask`), and the eligibility gate
    // above reads exactly that, so the grant is correct -- but the uop handed to the
    // consumer would still carry the stale bit. For the CII that is fatal on the
    // spot: VecCiiIssue registers `io.iss.bits` verbatim into `alloc_br_mask`, and
    // VecCiiFlush asserts it is 0 on every allocation. Measured on jacobi2d_bare,
    // spmv_bare and spmv_small, which fire that assertion and nothing else.
    // The grant-cycle kill half of the same contract is carried by `io.eligible`,
    // which ANDs `!killed` for EVERY slot type (see its assignment): a uop b1 is killing
    // this cycle is never granted, so dropping its bit here cannot hide a kill.
    io.iss_uop.br_mask := next_uop.br_mask
    io.iss_uop.fu_code(FC_AGEN) := dgen_path.io.iss_fu_code_agen
    io.iss_uop.fu_code(FC_DGEN) := dgen_path.io.iss_fu_code_dgen

    next_uop.iw_issued := io.grant && !io.squash_grant
    next_uop.iw_issued_partial_agen := dgen_path.io.issued_partial_agen
    next_uop.iw_issued_partial_dgen := dgen_path.io.issued_partial_dgen

    next_uop.fu_code(FC_AGEN) := dgen_path.io.next_fu_code_agen
    next_uop.fu_code(FC_DGEN) := dgen_path.io.next_fu_code_dgen

    when (slot_valid && slot_uop.iw_issued) {
      next_valid := rebusied || dgen_path.io.keep_valid
    }

    // TEMP PROBE -- A STORE WHOSE DGEN HALF NEVER ISSUES. The relaxed CII gate
    // wedges with the store drain parked in its write pass on a store whose data
    // was never enqueued (VecBeatExpander's `usDataRdy` requires st_us_data.valid),
    // and rob_idx ALIASES across ROB wrap so no per-rob grep can identify it. This
    // names the tracker state by PRN, which is unique while the op is live: match
    // `prn` against VecBusyTable's `clr pvdest=` stream to tell a lost wakeup pulse
    // from a producer that never wrote that register at all.
    {
      val dgenPending = slot_valid && !slot_uop.iw_issued &&
        slot_uop.fu_code(FC_DGEN) && !slot_uop.fu_code(FC_AGEN)
      val dgenStuck = RegInit(0.U(16.W))
      when (dgenPending && !dgen_path.io.dgen_request) { dgenStuck := dgenStuck + 1.U }
        .otherwise { dgenStuck := 0.U }
      when (dgenStuck > 64.U && dgenStuck(5, 0) === 0.U) {
        VecTrace.trace("VecIssueSlot", "dgen_stuck", slot_uop, Seq(
          ("prn",      rdy_vs3.io.prns(0)),
          ("members",  rdy_vs3.io.members.get),
          ("used",     rdy_vs3.io.used),
          ("ready",    rdy_vs3.io.ready),
          ("out_mrdy", rdy_vs3.io.out_member_rdy.asUInt),
          ("in_mrdy",  io.in_member_rdy.vs3_rdy.asUInt),
          ("fill",     io.in_uop.valid),
          ("op_busy",  dgen_path.io.dgen_operand_busy),
          ("is_pvtmp", dgen_path.io.dgen_operand_is_pvtmp),
          ("stq_idx",  slot_uop.stq_idx),
          ("stuck",    dgenStuck)))
      }
    }

    eligible := request
  } else {
    //@req-spec-core.f5
    //@req-spec-rename.g2
    //@req-spec-issue.g19
    //@req-spec-issue.g20
    val vector_operands_ready =
      rdy_vs1.io.ready && rdy_vs2.io.ready && rdy_vs3.io.ready && rdy_vm.io.ready && rdy_vold.get.io.ready
    request := slot_valid && !slot_uop.iw_issued && scalar_operands_ready && vector_operands_ready

    // Why this op is NOT allowed to issue yet -- the mirror of `grant_rdy` below,
    // and the one that carries information. `grant_rdy` prints the five matchers at
    // the grant, where every one of them reads 1 by definition; measuring WHY a load
    // waits needs the terms sampled while it is still waiting. Load slots only: the
    // ALU slot's wait is the CII dependence and is already visible in the vpu trace.
    //
    // This exists because axpy_bare's vector loads sit ~8.6 cycles between dispatch
    // and their first request with nothing downstream to blame -- dispatch unblocked,
    // the arbiter refusing on 7% of cycles, the IQ granting 2.6 cycles after
    // readiness -- so the wait is a readiness term and no existing trace names which.
    if (isLoadSlot) {
      when (slot_valid && !slot_uop.iw_issued && !request) {
        VecTrace.trace("VecIssueSlot", "load_not_rdy", slot_uop, Seq(
          ("prs1_bsy", slot_uop.prs1_busy), ("prs2_bsy", slot_uop.prs2_busy),
          ("pvl_bsy", slot_uop.pvl_busy.get),
          ("vs1", rdy_vs1.io.ready), ("vs2", rdy_vs2.io.ready),
          ("vs3", rdy_vs3.io.ready), ("vm", rdy_vm.io.ready),
          ("vold", rdy_vold.get.io.ready)))
      }
    }

    // Why this op was allowed to issue. An operand that reads ready with a busy
    // producer is either (a) woken -- see VecGroupReady's `wake` -- or (b) never
    // tracked at all, because `used` was false or `members` collapsed the
    // AND-reduce. This line separates those without another build.
    when (io.grant) {
      VecTrace.trace("VecIssueSlot", "grant_rdy", slot_uop, Seq(
        ("vs1", rdy_vs1.io.ready), ("vs2", rdy_vs2.io.ready), ("vs3", rdy_vs3.io.ready),
        ("vm", rdy_vm.io.ready), ("vold", rdy_vold.get.io.ready),
        ("vs3_used", rdy_vs3.io.used), ("vs3_mem", rdy_vs3.io.members.get),
        ("uses_vs3", slot_uop.v_uses_vs3.get), ("emul", slot_uop.v_emul.get),
        ("vs3_rdy_mask", rdy_vs3.io.out_member_rdy.asUInt)))
    }

    io.iss_uop := slot_uop
    //@req-spec-cii.e3
    // THE ISSUED UOP CARRIES THIS CYCLE'S RESOLVED BRANCH MASK, NOT THE REGISTER'S.
    // `slot_uop.br_mask` is one b1 behind: a branch resolving THIS cycle is already
    // out of `next_uop.br_mask` (that is `GetNewBrMask`), and the eligibility gate
    // above reads exactly that, so the grant is correct -- but the uop handed to the
    // consumer would still carry the stale bit. For the CII that is fatal on the
    // spot: VecCiiIssue registers `io.iss.bits` verbatim into `alloc_br_mask`, and
    // VecCiiFlush asserts it is 0 on every allocation. Measured on jacobi2d_bare,
    // spmv_bare and spmv_small, which fire that assertion and nothing else.
    // The grant-cycle kill half of the same contract is carried by `io.eligible`,
    // which ANDs `!killed` for EVERY slot type (see its assignment): a uop b1 is killing
    // this cycle is never granted, so dropping its bit here cannot hide a kill.
    io.iss_uop.br_mask := next_uop.br_mask

    next_uop.iw_issued := io.grant && !io.squash_grant
    next_uop.iw_issued_partial_agen := false.B
    next_uop.iw_issued_partial_dgen := false.B

    when (slot_valid && slot_uop.iw_issued) {
      next_valid := rebusied
    }

    // The CII queue's mirror of the load slot's `load_not_rdy`. Excluding the ALU
    // slot from that probe was a mistake: `IsOlder(rob_idx, pnr, head)` appears in no
    // other trace, so a CII op held by the PNR gate and one held by an operand look
    // identical from outside. `pnr` here is the gate term ALONE -- read it against
    // the operand columns to tell "waiting to become non-speculative" from "waiting
    // for data".
    if (isAluSlot) {
      // `pnr` is the OLD past-PNR term and `br` the one that now gates -- both are
      // printed, and unconditionally, so an A/B arm and the default arm produce the
      // same columns and the relaxation's effect is readable as the gap between them.
      val pastPnr = if (pnrGate) IsOlder(slot_uop.rob_idx, io.rob_pnr_idx.get,
        io.rob_head_idx.get) else true.B
      val brRdy   = (next_uop.br_mask === 0.U) && !killed
      when (slot_valid && !slot_uop.iw_issued && !(request && brRdy)) {
        VecTrace.trace("VecIssueSlot", "alu_not_rdy", slot_uop, Seq(
          ("pnr", pastPnr), ("br", brRdy), ("brmask", next_uop.br_mask),
          ("req", request),
          ("prs1_bsy", slot_uop.prs1_busy), ("prs2_bsy", slot_uop.prs2_busy),
          ("pvl_bsy", slot_uop.pvl_busy.get),
          ("vs1", rdy_vs1.io.ready), ("vs2", rdy_vs2.io.ready),
          ("vs3", rdy_vs3.io.ready), ("vm", rdy_vm.io.ready),
          ("vold", rdy_vold.get.io.ready)))
      }
    }

    //@req-spec-issue.g21
    if (pnrGate) {
      // ---- THE GATE IS BRANCH-RESOLUTION, NOT PAST-PNR ----
      //
      // The gate used to be `IsOlder(rob_idx, rob_pnr_idx, rob_head_idx)`: issue only
      // once the op is older than the ROB's oldest UNSAFE entry. That is far stronger
      // than what the CII actually needs, and the difference is the whole CII issue
      // rate. `starts_unsafe` is `uses_ldq || (uses_stq && !is_fence) || is_br ||
      // is_jalr`, so on a vector loop the PNR pointer is parked behind every in-flight
      // vector load and store -- the ops the CII op is waiting for DATA from anyway --
      // and the CII queue sits ~10 deep with operands ready and nothing to do.
      //
      // What the CII can and cannot survive is asymmetric, and only ONE of those four
      // producers is on the wrong side of it:
      //   - a ROB flush (exception, memory-ordering fail, load replay, any redirect
      //     that goes through the ROB) IS recoverable: VecCiiFlush raises `kill_all`
      //     on `rob_flush || rob_flush_kill`, every live tag is marked killed and
      //     drained, and the drain watchdog proves it completes. LOADS, STORES and
      //     EXCEPTIONS -- three of the four `starts_unsafe` producers -- can only
      //     squash through that path, so they need no gate at all.
      //   - a BRANCH MISPREDICT is NOT recoverable: b2 is a partial squash by
      //     `br_mask` with no flush, so it never reaches `kill_all`, and an op already
      //     handed to the coprocessor cannot be un-handed. VecCiiFlush encodes exactly
      //     this as `alloc_br_mask === 0` on every allocation.
      //
      // So the gate is the mispredict condition ALONE: no unresolved older branch in
      // this op's mask. Taken on `next_uop.br_mask` (`GetNewBrMask`) rather than the
      // register, so a branch resolving in b1 this cycle frees the op the SAME cycle
      // instead of one later, and ANDed with `killed` so the grant cycle cannot issue
      // an op that b1 is simultaneously killing (the grant-cycle kill contract).
      // VecCiiFlush's `alloc_br_mask === 0` assertion and VecIssueUnit's
      // "eligible entry was killed by a branch" assertion both still hold by
      // construction, unchanged.
      //
      // `+vec_strict_pnr=1` restores the old past-PNR term for A/B, and
      // `+vec_no_pnr_gate=1` still drops the gate entirely. That last one is MEASURE
      // ONLY and known to hang (axpy_bare, 2026-09-15: a mispredicted CII op wedges
      // the pipeline after ~5.4k instructions) -- it is the direct evidence that the
      // branch term is the one that must stay.
      val strictPnr = PlusArg("vec_strict_pnr", 0,
        "A/B: restore the old past-PNR eligibility term on IQ_V_ALU",
        width = 1).orR
      val gateOn = !PlusArg("vec_no_pnr_gate", 0,
        "MEASUREMENT ONLY: drop the CII eligibility gate on IQ_V_ALU entirely",
        width = 1).orR
      val pastPnrTerm = IsOlder(slot_uop.rob_idx, io.rob_pnr_idx.get, io.rob_head_idx.get)
      val brResolved  = (next_uop.br_mask === 0.U) && !killed
      val gateTerm    = Mux(strictPnr, pastPnrTerm, brResolved)
      eligible := request && (gateTerm || !gateOn)
    } else {
      eligible := request
    }
  }

  io.request  := request
  //@req-spec-cii.e3
  //@req-spec-issue.g21
  // NO SLOT MAY BE ELIGIBLE IN THE CYCLE b1 KILLS IT -- on EVERY slot type, not
  // only the pnrGate arm's `brResolved`. `iss_uop.br_mask` is `next_uop.br_mask`
  // (GetNewBrMask), which clears a bit on RESOLVE, mispredicted or not, so the
  // grant-cycle uop no longer carries the bit that kills it and no downstream
  // `IsKilledByBranch(..., iss.bits)` can see the kill; b1 is gone next cycle.
  // Measured on canneal_bare (MegaBoomV4VectorWideConfig): the `vle32.v` at rob 16
  // was granted from IQ_V_LOAD in the b1 cycle of the mispredicted loop branch
  // (rob 8) with br_mask=0, survived the squash as a zombie, and its LCB
  // group_done tripped `[rob] vec_clr_bsy naming an invalid entry`.
  io.eligible := eligible && !killed

  // ---- 8. Assertions ----
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


  // ---- 9. Tracing ----
  // Event 1: slot fill.
  when (io.in_uop.valid) {
    VecTrace.trace("VecIssueSlot", "fill", io.in_uop.bits, Seq(
      ("v_emul",      io.in_uop.bits.v_emul.get),
      ("v_is_masked", io.in_uop.bits.v_is_masked.get.asUInt),
      ("is_shared",   io.in_uop.bits.is_shared.get.asUInt)))
  }

  val request_prev = RegNext(request, false.B)
  when (request && !request_prev) {
    VecTrace.trace("VecIssueSlot", "request_rise", slot_uop)
  }

  when (io.grant) {
    VecTrace.trace("VecIssueSlot", "grant", slot_uop)
  }

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

  // ---- 10. Stall watchdog ----
  // Debug layer only: the rise traces above show an operand ARRIVING, and a slot that
  // hangs is one where it never does, so the level has to be sampled some other way.
  layer.block(BoomSvaLayer) {
    val stallCnt = RegInit(0.U(log2Ceil(stallReportCycles + 1).W))
    when (!slot_valid || io.grant || io.in_uop.valid) {
      stallCnt := 0.U
    } .otherwise {
      stallCnt := stallCnt + 1.U
    }
    when (stallCnt === stallReportCycles.U) {
      VecTrace.trace("VecIssueSlot", "stalled", slot_uop, Seq(
        ("scalar_rdy",  scalar_operands_ready.asUInt),
        ("prs1_busy",   slot_uop.prs1_busy.asUInt),
        ("prs2_busy",   slot_uop.prs2_busy.asUInt),
        ("pvl_busy",    slot_uop.pvl_busy.get.asUInt),
        ("iw_issued",   slot_uop.iw_issued.asUInt),
        ("vs1_rdy",     rdy_vs1.io.ready.asUInt),
        ("vs2_rdy",     rdy_vs2.io.ready.asUInt),
        ("vs3_rdy",     rdy_vs3.io.ready.asUInt),
        ("vm_rdy",      rdy_vm.io.ready.asUInt),
        ("vold_rdy",    rdy_vold.map(_.io.ready.asUInt).getOrElse(1.U)),
        ("vs3_mrdy",    rdy_vs3.io.out_member_rdy.asUInt),
        ("vm_mrdy",     rdy_vm.io.out_member_rdy.asUInt),
        ("vold_mrdy",   rdy_vold.map(_.io.out_member_rdy.asUInt).getOrElse(0.U)),
        ("request",     request.asUInt),
        ("eligible",    eligible.asUInt)))
    }
  }
}

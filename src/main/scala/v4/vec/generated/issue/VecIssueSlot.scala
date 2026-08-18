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

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.exu.{BrUpdateInfo, Wakeup}
import boom.v4.util.{IsKilledByBranch, IsOlder, UpdateBrMask}
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
    m.io.used := track_valid && track_uop.dst_rtype === RT_VEC
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

    //@req-spec-issue.g19
    //@req-spec-issue.g20
    request := dgen_path.io.agen_request || dgen_path.io.dgen_request

    io.iss_uop := slot_uop
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

    eligible := request
  } else {
    //@req-spec-core.f5
    //@req-spec-rename.g2
    //@req-spec-issue.g19
    //@req-spec-issue.g20
    val vector_operands_ready =
      rdy_vs1.io.ready && rdy_vs2.io.ready && rdy_vs3.io.ready && rdy_vm.io.ready && rdy_vold.get.io.ready
    request := slot_valid && !slot_uop.iw_issued && scalar_operands_ready && vector_operands_ready

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

    next_uop.iw_issued := io.grant && !io.squash_grant
    next_uop.iw_issued_partial_agen := false.B
    next_uop.iw_issued_partial_dgen := false.B

    when (slot_valid && slot_uop.iw_issued) {
      next_valid := rebusied
    }

    //@req-spec-issue.g21
    if (pnrGate) {
      eligible := request && IsOlder(slot_uop.rob_idx, io.rob_pnr_idx.get, io.rob_head_idx.get)
    } else {
      eligible := request
    }
  }

  io.request  := request
  io.eligible := eligible

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

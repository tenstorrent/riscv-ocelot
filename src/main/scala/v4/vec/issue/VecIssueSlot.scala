//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Issue Slot (Step 6)
//------------------------------------------------------------------------------
//
// A STANDALONE duplicate of the scalar IssueSlot skeleton
// (exu/issue-units/issue-slot.scala), adapted to vector operands. It does NOT
// subclass IssueSlot: ground-rule 3 sanctions duplication so the scalar issue
// files stay untouched. The generic valid/shift/brupdate/kill/clear/iw_issued
// bookkeeping is copied verbatim from the scalar slot; the readiness logic is
// rewritten to wait on EMUL-group vector sources (pvs1/pvs2/pvs3 groups), the
// single-PRN mask (pvm), the VL PRN (pvl), and any scalar sources (prs1/prs2/
// prs3) the uop still consumes.
//
// Vector group readiness mirrors VecBusyTable.groupBusy (VecBusyTable.scala:95-100):
// a source group is ready iff every active member (j < memberCount(v_emul)) is
// not busy OR is woken THIS cycle by a vector group-done wakeup. Same-cycle
// wakeups are forwarded so a source woken now reads ready, exactly as the busy
// table forwards clear_mask.
//
// This unit is DORMANT in Step 6: core.scala ties off the wakeup networks and
// supplies fu_types=0, so io.request never turns into a grant. Correctness here
// is by construction, not yet by simulation.

package boom.v4.vec.issue

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.util._
import boom.v4.exu.{BrUpdateInfo, Wakeup}
import boom.v4.vec.rename.{VecGroupDone, VlWakeup, VecEmul}

class VecIssueSlotIO(
  val numWakeupPorts: Int)
  (implicit p: Parameters) extends BoomBundle
{
  val valid         = Output(Bool())
  val will_be_valid = Output(Bool())
  val request       = Output(Bool())
  val grant         = Input(Bool())
  val iss_uop       = Output(new MicroOp())

  val in_uop        = Input(Valid(new MicroOp())) // if valid, this WILL overwrite an entry!
  val out_uop       = Output(new MicroOp())

  val brupdate      = Input(new BrUpdateInfo())
  val kill          = Input(Bool()) // pipeline flush
  val clear         = Input(Bool()) // entry being moved elsewhere (not mutually exclusive with grant)

  val squash_grant  = Input(Bool())

  val wakeup_ports     = Flipped(Vec(numWakeupPorts, Valid(new Wakeup)))
  val pred_wakeup_port = Flipped(Valid(UInt(log2Ceil(ftqSz).W)))
  val child_rebusys    = Input(UInt(aluWidth.W))

  // Caracal vector / VL wakeup networks (tied off in Step 6).
  val vec_wakeup_ports = Flipped(Vec(numVecWakeupPorts, Valid(new VecGroupDone)))
  val vl_wakeup_ports  = Flipped(Vec(numVlWakeupPorts,  Valid(new VlWakeup)))
}

class VecIssueSlot(
  val numWakeupPorts: Int)
  (implicit p: Parameters)
  extends BoomModule
{
  val io = IO(new VecIssueSlotIO(numWakeupPorts))

  //--------------------------------------------------------------------------
  // Generic slot bookkeeping -- copied VERBATIM from issue-slot.scala:55-82.
  //--------------------------------------------------------------------------
  val slot_valid = RegInit(false.B)
  val slot_uop = Reg(new MicroOp())

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

  //--------------------------------------------------------------------------
  // Scalar source feeders.
  //
  // Adapted from issue-slot.scala:86-134. The vector network never re-busies
  // (VecBusyTable.scala:18-20) and the scalar sources a vector uop reads (the
  // .vx base/stride for indexed/strided LS, the .vf/.vx scalar for arith) wake
  // on the ordinary scalar wakeup network, so we keep the scalar prs1/prs2/prs3
  // clear logic. We drop the iw_pN_speculative_child / rebusy bookkeeping that
  // only matters for the scalar ALU SNI path (kept as tie-offs below for the
  // mem AGEN/DGEN swap that we deliberately do NOT perform -- see note).
  //--------------------------------------------------------------------------
  next_uop.iw_p1_bypass_hint := false.B
  next_uop.iw_p2_bypass_hint := false.B
  next_uop.iw_p3_bypass_hint := false.B
  next_uop.iw_p1_speculative_child := 0.U
  next_uop.iw_p2_speculative_child := 0.U

  val prs1_matches = io.wakeup_ports.map { w => w.bits.uop.pdst === slot_uop.prs1 }
  val prs2_matches = io.wakeup_ports.map { w => w.bits.uop.pdst === slot_uop.prs2 }
  val prs3_matches = io.wakeup_ports.map { w => w.bits.uop.pdst === slot_uop.prs3 }
  val prs1_wakeups = (io.wakeup_ports zip prs1_matches).map { case (w,m) => w.valid && m }
  val prs2_wakeups = (io.wakeup_ports zip prs2_matches).map { case (w,m) => w.valid && m }
  val prs3_wakeups = (io.wakeup_ports zip prs3_matches).map { case (w,m) => w.valid && m }

  when (prs1_wakeups.reduce(_||_)) {
    next_uop.prs1_busy := false.B
  }
  when (prs2_wakeups.reduce(_||_)) {
    next_uop.prs2_busy := false.B
  }
  when (prs3_wakeups.reduce(_||_)) {
    next_uop.prs3_busy := false.B
  }
  when (io.pred_wakeup_port.valid && io.pred_wakeup_port.bits === slot_uop.ppred) {
    next_uop.ppred_busy := false.B
  }

  // scalar_operands_ready: prs1/prs2 are the integer base/stride (or arith
  // scalar). prs3 is conservatively kept as a wait term -- a vector arith uop
  // that takes a .vf/.vx scalar in prs3 must wait for it. We have no explicit
  // ".reads_scalar_prs3" decode bit yet (Step 2 leaves this to the cracker), so
  // we conservatively gate on the busy bit directly: if rename never set
  // prs3_busy for a uop that does not read prs3, this is a no-op. DEVIATION /
  // UNCERTAIN: revisit once a "reads .vf scalar" flag exists.
  val scalar_operands_ready = !slot_uop.prs1_busy && !slot_uop.prs2_busy && !slot_uop.prs3_busy

  //--------------------------------------------------------------------------
  // Vector source group readiness.
  //
  // Mirrors VecBusyTable.groupBusy (VecBusyTable.scala:95-100): for a source
  // group, every active member (j < memberCount(v_emul)) must be non-busy or
  // woken this cycle. We track readiness via the collapsed pvsN_busy bit that
  // rename set, clearing it (in next_uop) when ALL active members are woken --
  // mirroring how the scalar slot clears prsN_busy on a matching wakeup.
  //--------------------------------------------------------------------------
  val mc = VecEmul.memberCount(slot_uop.v_emul)

  // memberWoken(prn): is this PRN cleared by a VECTOR group-done THIS cycle?
  // OR over all vec wakeup ports of (valid && OR_j(mask(j) && prn(j) === prn)).
  def memberWoken(prn: UInt): Bool =
    io.vec_wakeup_ports.map { w =>
      w.valid && (0 until VecEmul.MAX_MEMBERS).map { j =>
        w.bits.mask(j) && (w.bits.prn(j) === prn)
      }.reduce(_||_)
    }.reduce(_||_)

  // A whole source group is woken this cycle iff every active member is woken.
  def groupAllWoken(grp: Vec[UInt]): Bool =
    (0 until VecEmul.MAX_MEMBERS).map { j =>
      !(j.U < mc) || memberWoken(grp(j))
    }.reduce(_&&_)

  val pvs1_all_woken = groupAllWoken(slot_uop.pvs1_grp)
  val pvs2_all_woken = groupAllWoken(slot_uop.pvs2_grp)
  val pvs3_all_woken = groupAllWoken(slot_uop.pvs3_grp)
  // Old-dest (stale_pvdest) group: the CII reads it as a source; wait for its producer.
  val pvold_all_woken = groupAllWoken(slot_uop.stale_pvdest_grp)

  when (pvs1_all_woken) { next_uop.pvs1_busy := false.B }
  when (pvs2_all_woken) { next_uop.pvs2_busy := false.B }
  when (pvs3_all_woken) { next_uop.pvs3_busy := false.B }
  when (pvold_all_woken) { next_uop.pvold_busy := false.B }

  // Ready bits combine the latched busy bit with this-cycle wakeup forwarding.
  val pvs1_ready = !slot_uop.pvs1_busy || pvs1_all_woken
  val pvs2_ready = !slot_uop.pvs2_busy || pvs2_all_woken
  val pvs3_ready = !slot_uop.pvs3_busy || pvs3_all_woken
  val pvold_ready = !slot_uop.pvold_busy || pvold_all_woken

  //--------------------------------------------------------------------------
  // Mask (pvm) readiness. Masked ops read v0 (single PRN, not a group); an
  // unmasked op (v_unmasked) does not read the mask at all. Clear pvm_busy on a
  // vector wakeup matching pvm. Mirrors VecBusyTable.scala:106-109.
  //--------------------------------------------------------------------------
  val pvm_woken = memberWoken(slot_uop.pvm)
  when (pvm_woken) { next_uop.pvm_busy := false.B }
  val pvm_ready = slot_uop.v_unmasked || !slot_uop.pvm_busy || pvm_woken

  //--------------------------------------------------------------------------
  // VL (pvl) readiness. pvl lives in its own RF; cleared by the VL wakeup
  // network (VlRename.scala:324-329 clears the busy bit on this same event).
  //--------------------------------------------------------------------------
  val pvl_woken = io.vl_wakeup_ports.map { w => w.valid && (w.bits.pvl === slot_uop.pvl) }.reduce(_||_)
  when (pvl_woken) { next_uop.pvl_busy := false.B }
  val pvl_ready = !slot_uop.pvl_busy || pvl_woken

  val vector_operands_ready = pvs1_ready && pvs2_ready && pvs3_ready && pvold_ready && pvm_ready && pvl_ready

  //--------------------------------------------------------------------------
  // STORE AGEN/DGEN readiness (computed before io.request so request can fire on
  // either half). Adapted from issue-slot.scala:137-138: AGEN gates on the
  // scalar base/stride (prs1/prs2), DGEN gates on the pvs3 store-data group on
  // the VECTOR network. pvm/pvl still gate the slot's overall validity through
  // vector_operands_ready below.
  //--------------------------------------------------------------------------
  val agen_ready = slot_uop.fu_code(FC_AGEN) && !slot_uop.prs1_busy && !slot_uop.prs2_busy
  val dgen_ready = slot_uop.fu_code(FC_DGEN) && pvs3_ready

  //--------------------------------------------------------------------------
  // Request / grant. Copied shape from issue-slot.scala:140-154. A plain vector
  // uop requests when all its operands are ready; a store-half uop (AGEN/DGEN)
  // requests when that half's sources are ready, plus the mask/VL gate.
  //--------------------------------------------------------------------------
  val iss_ready = scalar_operands_ready && vector_operands_ready
  // Caracal vec store (Step 11a.2): issues ONCE (no separate scalar-style DGEN
  // issue) -- VecDgen reads the pvs3 store-data group at issue -- so a store must
  // ALSO wait for pvs3_ready, even on the AGEN fast path.
  val st_data_ok = !slot_uop.uses_stq || pvs3_ready
  io.request := slot_valid && !slot_uop.iw_issued && (
    iss_ready || ((agen_ready || dgen_ready) && pvm_ready && pvl_ready && st_data_ok)
  )

  io.iss_uop := slot_uop

  next_uop.iw_issued := false.B
  next_uop.iw_issued_partial_agen := false.B
  next_uop.iw_issued_partial_dgen := false.B
  when (io.grant && !io.squash_grant) {
    next_uop.iw_issued := true.B
  }

  //--------------------------------------------------------------------------
  // STORE AGEN/DGEN split. Adapted from issue-slot.scala:156-205.
  //
  // A vector store is broken into two issues from one slot: AGEN (address gen,
  // gated on the scalar base/stride prs1/prs2) and DGEN (store data, gated on
  // the pvs3 group read at execute by the V-LSU). We key on fu_code(FC_AGEN)/
  // fu_code(FC_DGEN) exactly like the scalar mem slot and rewrite iss_uop.fu_code
  // so each issue presents a distinct fu_code. DEVIATION from the scalar slot:
  // we do NOT do the scalar prs1:=prs2 swap (store data is the pvs3 vector group,
  // read directly by the V-LSU, not a scalar prs). next_valid keeps the slot
  // alive between the two issues. There is no rebusy on the vector network so
  // next_valid after the final issue is simply false.
  //--------------------------------------------------------------------------
  when (slot_uop.fu_code(FC_AGEN) && slot_uop.fu_code(FC_DGEN)) {
    when (agen_ready) {
      // Issue the AGEN, next slot entry is a DGEN.
      when (io.grant && !io.squash_grant) {
        next_uop.iw_issued_partial_agen := true.B
      }
      io.iss_uop.fu_code(FC_AGEN) := true.B
      io.iss_uop.fu_code(FC_DGEN) := false.B
    } .otherwise {
      // Issue the DGEN, next slot entry is the AGEN.
      when (io.grant && !io.squash_grant) {
        next_uop.iw_issued_partial_dgen := true.B
      }
      io.iss_uop.fu_code(FC_AGEN) := false.B
      io.iss_uop.fu_code(FC_DGEN) := true.B
    }
  }

  when (slot_valid && slot_uop.iw_issued) {
    next_valid := false.B
    when (slot_uop.iw_issued_partial_agen) {
      // AGEN issued last cycle; the DGEN half remains.
      next_valid := true.B
      next_uop.fu_code(FC_AGEN) := false.B
      next_uop.fu_code(FC_DGEN) := true.B
    } .elsewhen (slot_uop.iw_issued_partial_dgen) {
      // DGEN issued last cycle; the AGEN half remains.
      next_valid := true.B
      next_uop.fu_code(FC_AGEN) := true.B
      next_uop.fu_code(FC_DGEN) := false.B
    }
  }
}

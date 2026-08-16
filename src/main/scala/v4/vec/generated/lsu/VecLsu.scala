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

package boom.v4.vec.generated.lsu

import chisel3._
import chisel3.util._
import chisel3.layer

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.exu.BrUpdateInfo
import boom.v4.lsu.{VecLsuCoreIO, GetRealLSQIdx, IsOlderLSU, EntryValidFromAge}
import boom.v4.util.{GetNewBrMask, IsKilledByBranch}
import boom.v4.vec.generated.{VecElemAccess, VecMemAccess, VecRangeEntry, VecException, IntWbSnoop, VecRobFlags, VecGroupDone, VecTrace}
import boom.v4.vec.formal.{BoomSvaLayer, VecLsuChecks}

// GENERATED from src/main/nlhdl/vec/lsu/VecLsu.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecLsuVlWb(implicit p: Parameters) extends BoomBundle
{
  val pvl = UInt(vlPregSz.W)
  val vl  = UInt(vecVLSz.W)
}

class VecLsuVrfWrite(implicit p: Parameters) extends BoomBundle
{
  val addr = UInt(vecPregSz.W)
  val data = UInt(vecVLen.W)
  val mask = UInt((vecVLen / 8).W)
}

class VecLsuIO(implicit p: Parameters) extends BoomBundle
{
  val dis_uops = Input(Vec(coreWidth, Valid(new MicroOp())))
  val dis_fire = Input(Vec(coreWidth, Bool()))
  val dis_ok   = Output(Vec(coreWidth, Bool()))

  val iss_ld = Input(Valid(new MicroOp()))
  val iss_st = Input(Valid(new MicroOp()))

  val brupdate       = Input(new BrUpdateInfo)
  val rob_flush      = Input(Bool())
  val rob_flush_kill = Input(Bool())
  val commit_valids  = Input(Vec(coreWidth, Bool()))
  val commit_uops    = Input(Vec(coreWidth, new MicroOp()))
  val rob_head_idx   = Input(UInt(robAddrSz.W))
  val rob_pnr_idx    = Input(UInt(robAddrSz.W))

  val int_rf_read_req = Vec(4, Decoupled(UInt(ipregSz.W)))
  val int_rf_read_rsp = Input(Vec(4, UInt(xLen.W)))
  val int_wb_snoop    = Input(Vec(numIrfWritePorts, Valid(new IntWbSnoop)))

  val vl_read_addr = Output(Vec(2, UInt(vlPregSz.W)))
  val vl_read_data = Input(Vec(2, UInt(vecVLSz.W)))
  val vl_wb        = Output(Valid(new VecLsuVlWb))

  val vrf_r0_req  = Output(Valid(UInt(vecPregSz.W)))
  val vrf_r0_resp = Input(Valid(UInt(vecVLen.W)))
  val vrf_r1_req  = Output(Valid(UInt(vecPregSz.W)))
  val vrf_r1_resp = Input(UInt(vecVLen.W))
  val vrf_r2_req  = Output(Valid(UInt(vecPregSz.W)))
  val vrf_r2_resp = Input(UInt(vecVLen.W))
  val vrf_r3_req  = Output(Valid(UInt(vecPregSz.W)))
  val vrf_r3_gnt  = Input(Bool())
  val vrf_r3_resp = Input(Valid(UInt(vecVLen.W)))
  val vrf_r4_req  = Output(Valid(UInt(vecPregSz.W)))
  val vrf_r4_resp = Input(UInt(vecVLen.W))
  val vrf_w0      = Output(Valid(new VecLsuVrfWrite))
  val vrf_w1      = if (lsuWidth == 2) Some(Output(Valid(new VecLsuVrfWrite))) else None

  val lsu_vec = Flipped(new VecLsuCoreIO)

  val vec_clr_bsy        = Output(Vec(2, Valid(UInt(robAddrSz.W))))
  val vec_group_done     = Output(Vec(2, Valid(new VecGroupDone)))
  val vec_rob_flags      = Output(Vec(2, Valid(new VecRobFlags)))
  val vec_clr_unsafe     = Output(Valid(UInt(robAddrSz.W)))
  val vec_xcpt           = Output(Valid(new VecException))
  val lsu_fencei_rdy_vec = Output(Bool())
}

class VecLsu(
  // Cycles a load lane may be refused by the D$ arbiter with a beat pending
  // before that counts as a deadlock; 0 disables (no register emitted).
  val ldAcceptWatchdog: Int = 4096
)(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecLsu: elaborates only under usingRVV")

  val io = IO(new VecLsuIO)

  //@req-spec-lsu.a13
  require((dcacheArbiterMode == "single" && lsuWidth == 1) || (dcacheArbiterMode == "dual-dynamic" && lsuWidth == 2),
    s"VecLsu: dcacheArbiterMode ($dcacheArbiterMode) must agree with lsuWidth ($lsuWidth)")

  val queuePorts = lsuWidth
  val addrWidth    = (new VecElemAccess).getWidth
  val rangeWidth   = (new VecRangeEntry).getWidth
  // VecDgen enqueues the FULL VecDgenSsiPayload/VecDgenUsPayload struct (data plus
  // byte_en/rob_idx/stq_idx/last), not a bare eLen/vLen data field -- sizing the
  // queue at eLen/vLen would truncate every enqueue and silently corrupt store data.
  val ssiDataWidth = (new VecDgenSsiPayload).getWidth
  val usDataWidth  = (new VecDgenUsPayload).getWidth
  val nKillClients = 5

  // =========================================================================
  // ---- 25 children ----
  // =========================================================================

  val resv = Module(new VecQueueReservation)

  val ld_opnd = Module(new VecScalarOperandRead(isStore = false))
  val st_opnd = Module(new VecScalarOperandRead(isStore = true))

  val ld_msk = Module(new VecMaskStream(isStore = false))
  val st_msk = Module(new VecMaskStream(isStore = true))

  val ld_elem_agen  = Module(new VecElemAgen(isStore = false))
  val st_elem_agen  = Module(new VecElemAgen(isStore = true))
  val ld_range_agen = Module(new VecRangeAgen(isStore = false))
  val st_range_agen = Module(new VecRangeAgen(isStore = true))

  val ld_SSI_ADDR_Q = Module(new VecElemQueue("ld_SSI_ADDR_Q", ssiQueueEntries, addrWidth,    isStore = false, hasXlatePass = false, reserved = true, ports = queuePorts))
  val st_SSI_ADDR_Q = Module(new VecElemQueue("st_SSI_ADDR_Q", ssiQueueEntries, addrWidth,    isStore = true,  hasXlatePass = true,  reserved = true, ports = queuePorts))
  val st_SSI_DATA_Q = Module(new VecElemQueue("st_SSI_DATA_Q", ssiQueueEntries, ssiDataWidth, isStore = true,  hasXlatePass = false, reserved = true, ports = queuePorts))
  val ld_US_ADDR_Q  = Module(new VecElemQueue("ld_US_ADDR_Q",  usQueueEntries,  rangeWidth,   isStore = false, hasXlatePass = false, reserved = true, ports = queuePorts))
  val st_US_ADDR_Q  = Module(new VecElemQueue("st_US_ADDR_Q",  usQueueEntries,  rangeWidth,   isStore = true,  hasXlatePass = true,  reserved = true, ports = queuePorts))
  val st_US_DATA_Q  = Module(new VecElemQueue("st_US_DATA_Q",  usQueueEntries,  usDataWidth,  isStore = true,  hasXlatePass = false, reserved = true, ports = queuePorts))

  val ld_beat = Module(new VecBeatExpander(isStore = false, nLanes = lsuWidth, dmemBeatBytes = coreDataBytes))
  val st_beat = Module(new VecBeatExpander(isStore = true,  nLanes = lsuWidth, dmemBeatBytes = coreDataBytes))

  val dgen  = Module(new VecDgen)
  val lcb   = Module(new VecLoadCoalescingBuffer)
  val gcopy = Module(new VecGroupCopy())

  val arb = Module(new VecDcacheArbiter)

  val snoop  = Module(new VecCrossLsuSnoop(searchPorts = lsuWidth))
  //@req-spec-memord.b16
  val vecBeatForwardEnable = false
  val fwd    = Module(new VecStoreForward(enableVecStoreForward = vecScalarSnoopEnable, enableVecBeatForward = vecBeatForwardEnable))
  val hold   = Module(new VecOrderHold(enableOrderHold = true, forwardingEnabled = vecBeatForwardEnable))
  val squash = Module(new VecSquashUnit(nQueues = 6, nKillClients = nKillClients))

  //@req-spec-core.c10
  //@req-spec-agen.a1
  //@req-spec-agen.a2
  // io.iss_ld/io.iss_st are the only inputs from the issue side, and vec_fire/
  // lcam (below, via arb/snoop) the only outputs toward memory: this stage
  // sits strictly between IQ_V_LOAD/IQ_V_STORE and the Unified LSU, and an
  // OP.v is the single uop dispatch/issue ever see -- its expansion into nOP.v
  // happens only inside the children this file instantiates.
  val vec = io.lsu_vec

  // =========================================================================
  // ---- 4. Reservation and the six queues ----
  // =========================================================================

  resv.io.dis_uops := io.dis_uops
  resv.io.dis_fire := io.dis_fire
  io.dis_ok        := resv.io.dis_ok
  resv.io.ldq_head := vec.ldq_head
  resv.io.stq_head := vec.stq_head

  val queueSeq = Seq(ld_SSI_ADDR_Q, st_SSI_ADDR_Q, st_SSI_DATA_Q, ld_US_ADDR_Q, st_US_ADDR_Q, st_US_DATA_Q)

  for (q <- 0 until 6) {
    val claimTerms = for (w <- 0 until coreWidth; k <- 0 until 2) yield {
      val slot = resv.io.resv_out(w)(k)
      (slot.valid && slot.bits.queue === q.U, slot.bits.entries)
    }
    queueSeq(q).io.resv.claim.valid := claimTerms.map(_._1).reduce(_ || _)
    queueSeq(q).io.resv.claim.bits  := claimTerms.map { case (v, e) => Mux(v, e, 0.U) }.reduce(_ +& _)

    // Spec defect (see report): VecQueueReservation exposes no live per-queue
    // tail for the surplus trim, only rollback_tail (squash-only). release_tail
    // is tied off here; the queue's own tail then only moves on claim/free/
    // squash, staying conservatively larger than resv's view until the whole
    // entry retires and its region is freed below.
    queueSeq(q).io.resv.release_tail.valid := false.B
    queueSeq(q).io.resv.release_tail.bits  := 0.U

    queueSeq(q).io.resv.free.valid        := resv.io.region_free(q).valid
    queueSeq(q).io.resv.free.bits.base    := resv.io.region_free(q).bits.base
    queueSeq(q).io.resv.free.bits.entries := resv.io.region_free(q).bits.count

    queueSeq(q).io.squash := squash.io.q_squash(q)
  }

  // ---- resv_lookup / release: the four fixed lanes (0 ld_elem_agen, 1
  // st_elem_agen, 2 ld_range_agen, 3 st_range_agen) ----
  resv.io.resv_lookup(0).valid         := ld_elem_agen.io.resv_lookup.valid
  resv.io.resv_lookup(0).bits.is_store := ld_elem_agen.io.resv_lookup.bits.is_store
  resv.io.resv_lookup(0).bits.q_idx    := ld_elem_agen.io.resv_lookup.bits.q_idx
  ld_elem_agen.io.resv_resp(0) := resv.io.resv_resp(0)(0)
  ld_elem_agen.io.resv_resp(1) := resv.io.resv_resp(0)(1)
  resv.io.release(0).valid             := ld_elem_agen.io.release.valid
  resv.io.release(0).bits.is_store     := ld_elem_agen.io.release.bits.is_store
  resv.io.release(0).bits.q_idx        := ld_elem_agen.io.release.bits.q_idx
  resv.io.release(0).bits.used_count   := ld_elem_agen.io.release.bits.used_count
  ld_elem_agen.io.release_ok := resv.io.release_ok(0)

  resv.io.resv_lookup(1).valid         := st_elem_agen.io.resv_lookup.valid
  resv.io.resv_lookup(1).bits.is_store := st_elem_agen.io.resv_lookup.bits.is_store
  resv.io.resv_lookup(1).bits.q_idx    := st_elem_agen.io.resv_lookup.bits.q_idx
  st_elem_agen.io.resv_resp(0) := resv.io.resv_resp(1)(0)
  st_elem_agen.io.resv_resp(1) := resv.io.resv_resp(1)(1)
  resv.io.release(1).valid             := st_elem_agen.io.release.valid
  resv.io.release(1).bits.is_store     := st_elem_agen.io.release.bits.is_store
  resv.io.release(1).bits.q_idx        := st_elem_agen.io.release.bits.q_idx
  resv.io.release(1).bits.used_count   := st_elem_agen.io.release.bits.used_count
  st_elem_agen.io.release_ok := resv.io.release_ok(1)

  resv.io.resv_lookup(2).valid         := ld_range_agen.io.resv_lookup.valid
  resv.io.resv_lookup(2).bits.is_store := ld_range_agen.io.resv_lookup.bits.is_store
  resv.io.resv_lookup(2).bits.q_idx    := ld_range_agen.io.resv_lookup.bits.q_idx
  ld_range_agen.io.resv_resp := resv.io.resv_resp(2)
  resv.io.release(2).valid             := ld_range_agen.io.release.valid
  resv.io.release(2).bits.is_store     := ld_range_agen.io.release.bits.is_store
  resv.io.release(2).bits.q_idx        := ld_range_agen.io.release.bits.q_idx
  resv.io.release(2).bits.used_count   := ld_range_agen.io.release.bits.used_count
  ld_range_agen.io.release_ok := resv.io.release_ok(2)

  resv.io.resv_lookup(3).valid         := st_range_agen.io.resv_lookup.valid
  resv.io.resv_lookup(3).bits.is_store := st_range_agen.io.resv_lookup.bits.is_store
  resv.io.resv_lookup(3).bits.q_idx    := st_range_agen.io.resv_lookup.bits.q_idx
  st_range_agen.io.resv_resp := resv.io.resv_resp(3)
  resv.io.release(3).valid             := st_range_agen.io.release.valid
  resv.io.release(3).bits.is_store     := st_range_agen.io.release.bits.is_store
  resv.io.release(3).bits.q_idx        := st_range_agen.io.release.bits.q_idx
  resv.io.release(3).bits.used_count   := st_range_agen.io.release.bits.used_count
  st_range_agen.io.release_ok := resv.io.release_ok(3)

  // ---- head-side reclamation ----
  // Assumption: ldq_head/stq_head each advance by at most one entry per cycle
  // (resv.io.retire itself is a single-slot port, so this is a pre-existing
  // constraint of that interface, not one this container introduces). Walked
  // with an explicit pointer rather than an edge-delta so a multi-cycle
  // catch-up never silently skips an entry if that assumption is ever wrong.
  val ldRetirePtr = RegInit(0.U((1 + ldqAddrSz).W))
  val stRetirePtr = RegInit(0.U((1 + stqAddrSz).W))
  val ldRetireWant = vec.ldq_head =/= ldRetirePtr
  val stRetireWant = vec.stq_head =/= stRetirePtr
  val ldRetireNow = ldRetireWant
  val stRetireNow = stRetireWant && !ldRetireWant

  resv.io.retire.bits.is_store := stRetireNow
  resv.io.retire.bits.q_idx    := Mux(ldRetireNow, ldRetirePtr, stRetirePtr)
  // The LSQ is shared with scalar memory: the walk must step past an entry that
  // reserved nothing without retiring against its empty row.
  resv.io.retire.valid         := (ldRetireNow || stRetireNow) && resv.io.retire_row_valid
  when (ldRetireNow) { ldRetirePtr := ldRetirePtr + 1.U }
  when (stRetireNow) { stRetirePtr := stRetirePtr + 1.U }

  // =========================================================================
  // ---- 9. Squash ----
  // =========================================================================

  squash.io.brupdate       := io.brupdate
  squash.io.rob_flush      := io.rob_flush
  squash.io.rob_flush_kill := io.rob_flush_kill
  squash.io.ldq_head        := vec.ldq_head
  squash.io.ldq_tail        := vec.ldq_tail
  squash.io.stq_commit_head := vec.stq_commit_head
  squash.io.stq_tail        := vec.stq_tail
  squash.io.resv_rollback_tail := resv.io.rollback_tail
  resv.io.rollback := squash.io.resv_rollback

  lcb.io.kill_ldq := squash.io.kill_ldq

  // =========================================================================
  // ---- 3b. Pending descriptor table + per-direction fill-side pulse ----
  // =========================================================================

  class PendRow(val idxW: Int) extends BoomBundle {
    val valid   = Bool()
    val fullIdx = UInt(idxW.W)
    val uop     = new MicroOp
  }

  // ---- LOAD direction ----
  val ldRows = RegInit(VecInit(Seq.fill(numLdqEntries)(0.U.asTypeOf(new PendRow(1 + ldqAddrSz)))))
  when (io.iss_ld.valid) {
    val idx = GetRealLSQIdx(io.iss_ld.bits.ldq_idx)
    assert(!ldRows(idx).valid, "VecLsu: iss_ld granted into an already-valid pending row")
    ldRows(idx).valid   := true.B
    ldRows(idx).fullIdx := io.iss_ld.bits.ldq_idx
    ldRows(idx).uop     := io.iss_ld.bits
  }
  for (i <- 0 until numLdqEntries) {
    when (ldRows(i).valid && !(io.iss_ld.valid && GetRealLSQIdx(io.iss_ld.bits.ldq_idx) === i.U)) {
      ldRows(i).uop.br_mask := GetNewBrMask(io.brupdate, ldRows(i).uop)
      when (IsKilledByBranch(io.brupdate, io.rob_flush, ldRows(i).uop)) {
        ldRows(i).valid := false.B
      }
    }
  }

  val ldCandValid   = Wire(Vec(numLdqEntries + 1, Bool()))
  val ldCandFullIdx = Wire(Vec(numLdqEntries + 1, UInt((1 + ldqAddrSz).W)))
  for (i <- 0 until numLdqEntries) {
    ldCandValid(i)   := ldRows(i).valid
    ldCandFullIdx(i) := ldRows(i).fullIdx
  }
  ldCandValid(numLdqEntries)   := io.iss_ld.valid
  ldCandFullIdx(numLdqEntries) := io.iss_ld.bits.ldq_idx

  val ldFound   = Wire(Vec(numLdqEntries + 1, Bool()))
  val ldBestIdx = Wire(Vec(numLdqEntries + 1, UInt(log2Ceil(numLdqEntries + 1).W)))
  ldFound(0)   := ldCandValid(0)
  ldBestIdx(0) := 0.U
  for (i <- 1 until numLdqEntries + 1) {
    val curBestFull = ldCandFullIdx(ldBestIdx(i - 1))
    val newer = !ldFound(i - 1) || IsOlderLSU(ldCandFullIdx(i.U(log2Ceil(numLdqEntries + 1).W)), curBestFull, vec.ldq_head)
    val take  = ldCandValid(i) && newer
    ldFound(i)   := ldFound(i - 1) || ldCandValid(i)
    ldBestIdx(i) := Mux(take, i.U, ldBestIdx(i - 1))
  }
  val ldWinnerValid   = ldFound(numLdqEntries)
  val ldWinnerSlot    = ldBestIdx(numLdqEntries)
  val ldWinnerFresh   = ldWinnerSlot === numLdqEntries.U
  val ldWinnerFullIdx = ldCandFullIdx(ldWinnerSlot)
  val ldWinnerUop     = Mux(ldWinnerFresh, io.iss_ld.bits, ldRows(ldWinnerSlot).uop)

  val ldStreamerBusy = RegInit(false.B)
  // Forward declaration. `lcbWalkActive` is defined beside the allocation walk
  // several hundred lines below, and Scala would read a null here -- the same
  // trap the LCB-credit note in the spec calls out. Declare the Wire and connect
  // it there.
  val lcbWalkBusy = Wire(Bool())
  // The LCB allocation walk holds ONE op at a time (see the assert beside it),
  // and the walk stalls whenever the LCB has no free entry, so admitting the next
  // load while a walk is live would begin a second one and lose the first's
  // remaining members. Back-pressure HERE, at admission, and nowhere later: the
  // op simply stays valid in `ldRows` because only `ldPresent` clears it.
  //
  // It specifically must NOT be done by gating `ldMaskFire`. For a unit-stride
  // op VecMaskStream retires the instant its latch loads
  // (`done_unit_stride = fresh_this_cycle && us_now`), so `us_mask.valid` is a
  // ONE-CYCLE PULSE -- that is what "the launch cycle must be able to fire" below
  // is protecting, and deferring it would silently drop the op rather than stall
  // it.
  val ldPresent = ldWinnerValid && !ldStreamerBusy && !lcbWalkBusy
  when (ldPresent) {
    ldRows(GetRealLSQIdx(ldWinnerFullIdx)).valid := false.B
    ldStreamerBusy := true.B
  } .elsewhen (ld_msk.io.done) {
    ldStreamerBusy := false.B
  }

  val ldAwaitingPulse = RegInit(false.B)
  when (ldPresent) { ldAwaitingPulse := true.B }
  val ldOpPulse = ldAwaitingPulse && ld_opnd.io.out.valid
  when (ldOpPulse) { ldAwaitingPulse := false.B }

  val ldMaskAwaiting = RegInit(false.B)
  // An unmasked OP.v elides the mask read, so us_mask is valid in the very cycle
  // ldOpPulse launches the streamer -- the launch cycle must be able to fire.
  val ldMaskFire = (ldMaskAwaiting || ldOpPulse) && ld_msk.io.us_mask.valid
  when (ldOpPulse) { ldMaskAwaiting := true.B }
  when (ldMaskFire) { ldMaskAwaiting := false.B }
  val ldIsRangeClass = ld_opnd.io.out.bits.uop.v_is_unit_stride.get || ld_opnd.io.out.bits.uop.v_is_whole_reg.get ||
    ld_opnd.io.out.bits.uop.v_is_mask.get

  //@req-spec-agen.a6
  // ld_opnd + ld_elem_agen + ld_range_agen are ld_vAGEN_1; the store mirror
  // below (st_opnd/st_elem_agen/st_range_agen) is st_vagen_1; dgen is
  // st_vdgen, granted on its own pulse alongside the store vAGEN.
  ld_opnd.io.iss.valid := ldPresent
  ld_opnd.io.iss.bits  := ldWinnerUop
  ld_opnd.io.brupdate  := io.brupdate
  ld_opnd.io.rob_flush := io.rob_flush

  // ---- STORE direction (mirrors the load direction above) ----
  val stRows = RegInit(VecInit(Seq.fill(numStqEntries)(0.U.asTypeOf(new PendRow(1 + stqAddrSz)))))
  // A store issues TWICE -- VecStoreDgenPath offers the AGEN pass then the DGEN
  // pass, and the two are exclusive on fu_code. This container drives dgen off
  // the AGEN pulse, so the DGEN grant is not a new op and must not restart one.
  val issStAgen = io.iss_st.valid && io.iss_st.bits.fu_code(FC_AGEN)
  when (issStAgen) {
    val idx = GetRealLSQIdx(io.iss_st.bits.stq_idx)
    assert(!stRows(idx).valid, "VecLsu: iss_st granted into an already-valid pending row")
    stRows(idx).valid   := true.B
    stRows(idx).fullIdx := io.iss_st.bits.stq_idx
    stRows(idx).uop     := io.iss_st.bits
  }
  for (i <- 0 until numStqEntries) {
    when (stRows(i).valid && !(issStAgen && GetRealLSQIdx(io.iss_st.bits.stq_idx) === i.U)) {
      stRows(i).uop.br_mask := GetNewBrMask(io.brupdate, stRows(i).uop)
      when (IsKilledByBranch(io.brupdate, io.rob_flush, stRows(i).uop)) {
        stRows(i).valid := false.B
      }
    }
  }

  val stCandValid   = Wire(Vec(numStqEntries + 1, Bool()))
  val stCandFullIdx = Wire(Vec(numStqEntries + 1, UInt((1 + stqAddrSz).W)))
  for (i <- 0 until numStqEntries) {
    stCandValid(i)   := stRows(i).valid
    stCandFullIdx(i) := stRows(i).fullIdx
  }
  stCandValid(numStqEntries)   := issStAgen
  stCandFullIdx(numStqEntries) := io.iss_st.bits.stq_idx

  val stFound   = Wire(Vec(numStqEntries + 1, Bool()))
  val stBestIdx = Wire(Vec(numStqEntries + 1, UInt(log2Ceil(numStqEntries + 1).W)))
  stFound(0)   := stCandValid(0)
  stBestIdx(0) := 0.U
  for (i <- 1 until numStqEntries + 1) {
    val curBestFull = stCandFullIdx(stBestIdx(i - 1))
    val newer = !stFound(i - 1) || IsOlderLSU(stCandFullIdx(i.U(log2Ceil(numStqEntries + 1).W)), curBestFull, vec.stq_head)
    val take  = stCandValid(i) && newer
    stFound(i)   := stFound(i - 1) || stCandValid(i)
    stBestIdx(i) := Mux(take, i.U, stBestIdx(i - 1))
  }
  val stWinnerValid   = stFound(numStqEntries)
  val stWinnerSlot    = stBestIdx(numStqEntries)
  val stWinnerFresh   = stWinnerSlot === numStqEntries.U
  val stWinnerFullIdx = stCandFullIdx(stWinnerSlot)
  val stWinnerUop     = Mux(stWinnerFresh, io.iss_st.bits, stRows(stWinnerSlot).uop)

  val stStreamerBusy = RegInit(false.B)
  // dgen.io.req.ready is included so a second store cannot be granted into
  // st_opnd while dgen is still draining the first one's SSI/US data -- dgen's
  // own completion lags st_msk.io.done whenever the R3 refill stalls (see the
  // DGEN skid-buffer note below), so streamer-busy alone is not enough here.
  val stPresent = stWinnerValid && !stStreamerBusy && dgen.io.req.ready
  when (stPresent) {
    stRows(GetRealLSQIdx(stWinnerFullIdx)).valid := false.B
    stStreamerBusy := true.B
  } .elsewhen (st_msk.io.done) {
    stStreamerBusy := false.B
  }

  val stAwaitingPulse = RegInit(false.B)
  when (stPresent) { stAwaitingPulse := true.B }
  val stOpPulse = stAwaitingPulse && st_opnd.io.out.valid
  when (stOpPulse) { stAwaitingPulse := false.B }

  val stMaskAwaiting = RegInit(false.B)
  val stMaskFire = (stMaskAwaiting || stOpPulse) && st_msk.io.us_mask.valid
  when (stOpPulse) { stMaskAwaiting := true.B }
  when (stMaskFire) { stMaskAwaiting := false.B }
  val stIsRangeClass = st_opnd.io.out.bits.uop.v_is_unit_stride.get || st_opnd.io.out.bits.uop.v_is_whole_reg.get ||
    st_opnd.io.out.bits.uop.v_is_mask.get

  st_opnd.io.iss.valid := stPresent
  st_opnd.io.iss.bits  := stWinnerUop
  st_opnd.io.brupdate  := io.brupdate
  st_opnd.io.rob_flush := io.rob_flush

  // =========================================================================
  // ---- 3. Mask streamers + fill-side agens ----
  // =========================================================================

  ld_msk.io.op        := DontCare
  ld_msk.io.op.valid   := ldOpPulse
  ld_msk.io.op.bits    := ld_opnd.io.out.bits.uop
  ld_msk.io.op_masked := ld_opnd.io.out.bits.uop.v_is_masked.get
  ld_msk.io.op_vl      := ld_opnd.io.out.bits.vl
  ld_msk.io.kill        := squash.io.kill(0)
  ld_msk.io.step        := ld_elem_agen.io.msk_step
  ld_msk.io.skip        := ld_elem_agen.io.msk_skip
  io.vrf_r1_req.valid   := ld_msk.io.mask_rd_req.valid
  io.vrf_r1_req.bits    := ld_msk.io.mask_rd_req.addr
  ld_msk.io.mask_rd_data := io.vrf_r1_resp

  st_msk.io.op         := DontCare
  st_msk.io.op.valid    := stOpPulse
  st_msk.io.op.bits     := st_opnd.io.out.bits.uop
  st_msk.io.op_masked  := st_opnd.io.out.bits.uop.v_is_masked.get
  st_msk.io.op_vl       := st_opnd.io.out.bits.vl
  st_msk.io.kill         := squash.io.kill(1)
  st_msk.io.step         := st_elem_agen.io.msk_step
  st_msk.io.skip         := st_elem_agen.io.msk_skip

  //@req-spec-agen.a3
  //@req-spec-agen.b1
  //@req-spec-agen.b2
  //@req-spec-agen.b3
  //@req-spec-agen.b4
  // Stage 1 (strided/indexed/segmented, absorbed into VecElemAgen) and the
  // range agen (unit-stride/whole-reg/mask) both observe the SAME broadcast
  // pulse and self-select on class -- no routing mux here decides between
  // them (VecRangeAgen is the one exception forced by its own as-built
  // interface; see report). Cracking-by-EMUL's member/byte-offset fields
  // (v_split_dst_prn/v_split_dst_byte_off) are stamped inside the agen and
  // read back verbatim wherever this container reconstructs a beat's uop.
  ld_elem_agen.io.op.valid := ldOpPulse
  ld_elem_agen.io.op.bits.uop        := ld_opnd.io.out.bits.uop
  ld_elem_agen.io.op.bits.base       := ld_opnd.io.out.bits.base
  ld_elem_agen.io.op.bits.stride     := ld_opnd.io.out.bits.stride
  ld_elem_agen.io.op.bits.scalar_data := ld_opnd.io.out.bits.scalar_data
  ld_elem_agen.io.op.bits.vl         := ld_opnd.io.out.bits.vl
  ld_elem_agen.io.op.bits.vl_zero    := ld_opnd.io.out.bits.vl_zero
  ld_elem_agen.io.msk_staged      := ld_msk.io.staged
  ld_elem_agen.io.msk_ahead       := ld_msk.io.ahead
  ld_elem_agen.io.msk_skip_log2   := ld_msk.io.skip_log2
  ld_elem_agen.io.msk_skip_valid  := ld_msk.io.skip_valid
  ld_elem_agen.io.msk_all_inactive := ld_msk.io.all_inactive
  ld_elem_agen.io.brupdate := io.brupdate
  ld_elem_agen.io.rob_flush := io.rob_flush
  io.vrf_r0_req.valid          := ld_elem_agen.io.vrf_read_req.valid
  io.vrf_r0_req.bits           := ld_elem_agen.io.vrf_read_req.bits
  ld_elem_agen.io.vrf_read_gnt := true.B
  ld_elem_agen.io.vrf_read_rsp := io.vrf_r0_resp

  st_elem_agen.io.op.valid := stOpPulse
  st_elem_agen.io.op.bits.uop        := st_opnd.io.out.bits.uop
  st_elem_agen.io.op.bits.base       := st_opnd.io.out.bits.base
  st_elem_agen.io.op.bits.stride     := st_opnd.io.out.bits.stride
  st_elem_agen.io.op.bits.scalar_data := st_opnd.io.out.bits.scalar_data
  st_elem_agen.io.op.bits.vl         := st_opnd.io.out.bits.vl
  st_elem_agen.io.op.bits.vl_zero    := st_opnd.io.out.bits.vl_zero
  st_elem_agen.io.msk_staged      := st_msk.io.staged
  st_elem_agen.io.msk_ahead       := st_msk.io.ahead
  st_elem_agen.io.msk_skip_log2   := st_msk.io.skip_log2
  st_elem_agen.io.msk_skip_valid  := st_msk.io.skip_valid
  st_elem_agen.io.msk_all_inactive := st_msk.io.all_inactive
  st_elem_agen.io.brupdate := io.brupdate
  st_elem_agen.io.rob_flush := io.rob_flush

  //@req-spec-agen.c17 (R4 mux: mask reader has static priority; index waits)
  val r4MaskReq = st_msk.io.mask_rd_req
  io.vrf_r4_req.valid := r4MaskReq.valid || st_elem_agen.io.vrf_read_req.valid
  io.vrf_r4_req.bits  := Mux(r4MaskReq.valid, r4MaskReq.addr, st_elem_agen.io.vrf_read_req.bits)
  st_msk.io.mask_rd_data       := io.vrf_r4_resp
  st_elem_agen.io.vrf_read_gnt := !st_msk.io.owns_port
  val r4IdxGrantedPrev = RegNext(st_elem_agen.io.vrf_read_req.valid && st_elem_agen.io.vrf_read_gnt, false.B)
  st_elem_agen.io.vrf_read_rsp.valid := r4IdxGrantedPrev
  st_elem_agen.io.vrf_read_rsp.bits  := io.vrf_r4_resp

  // ---- range agens: VecRangeAgen is stateless/one-shot and asserts its mask
  // is already valid the SAME cycle io.req fires, so (unlike VecElemAgen, which
  // accepts immediately and stalls its own walk internally) its accept must be
  // deferred to the cycle the streamer's us_mask actually becomes valid. ----
  ld_range_agen.io.req.valid    := ldMaskFire && ldIsRangeClass
  ld_range_agen.io.req.bits     := ld_opnd.io.out.bits.uop
  ld_range_agen.io.scalar.valid := ldMaskFire && ldIsRangeClass
  ld_range_agen.io.scalar.bits  := ld_opnd.io.out.bits
  ld_range_agen.io.mask.valid   := ld_msk.io.us_mask.valid
  ld_range_agen.io.mask.bits    := ld_msk.io.us_mask.bits
  ld_range_agen.io.mask.rob_idx := ld_msk.io.us_mask.rob_idx
  ld_range_agen.io.brupdate     := io.brupdate
  ld_range_agen.io.rob_flush    := io.rob_flush

  st_range_agen.io.req.valid    := stMaskFire && stIsRangeClass
  st_range_agen.io.req.bits     := st_opnd.io.out.bits.uop
  st_range_agen.io.scalar.valid := stMaskFire && stIsRangeClass
  st_range_agen.io.scalar.bits  := st_opnd.io.out.bits
  st_range_agen.io.mask.valid   := st_msk.io.us_mask.valid
  st_range_agen.io.mask.bits    := st_msk.io.us_mask.bits
  st_range_agen.io.mask.rob_idx := st_msk.io.us_mask.rob_idx
  st_range_agen.io.brupdate     := io.brupdate
  st_range_agen.io.rob_flush    := io.rob_flush
  st_range_agen.io.st_data.foreach(_.ready := true.B)

  // =========================================================================
  // ---- INT RF (4 lanes) / VL RF (2 read lanes + 1 write) ----
  // =========================================================================

  io.int_rf_read_req(0) <> ld_opnd.io.int_rf_read_req(0)
  io.int_rf_read_req(1) <> ld_opnd.io.int_rf_read_req(1)
  io.int_rf_read_req(2) <> st_opnd.io.int_rf_read_req(0)
  io.int_rf_read_req(3) <> st_opnd.io.int_rf_read_req(1)
  ld_opnd.io.int_rf_read_rsp(0) := io.int_rf_read_rsp(0)
  ld_opnd.io.int_rf_read_rsp(1) := io.int_rf_read_rsp(1)
  st_opnd.io.int_rf_read_rsp(0) := io.int_rf_read_rsp(2)
  st_opnd.io.int_rf_read_rsp(1) := io.int_rf_read_rsp(3)
  ld_opnd.io.int_wb_snoop := io.int_wb_snoop
  st_opnd.io.int_wb_snoop := io.int_wb_snoop

  io.vl_read_addr(0) := ld_opnd.io.vl_read_addr
  io.vl_read_addr(1) := st_opnd.io.vl_read_addr
  ld_opnd.io.vl_read_data := io.vl_read_data(0)
  st_opnd.io.vl_read_data := io.vl_read_data(1)

  io.vl_wb.valid    := lcb.io.vl_wb.valid
  io.vl_wb.bits.pvl := lcb.io.vl_wb.bits.pvl
  io.vl_wb.bits.vl  := lcb.io.vl_wb.bits.vl

  // =========================================================================
  // ---- 4 (cont). Fill side into the six queues ----
  // =========================================================================

  ld_SSI_ADDR_Q.io.enq(0).valid := ld_elem_agen.io.addr_enq.valid
  ld_SSI_ADDR_Q.io.enq(0).bits.idx  := ld_elem_agen.io.addr_enq.bits.idx
  ld_SSI_ADDR_Q.io.enq(0).bits.data := ld_elem_agen.io.addr_enq.bits.data.asUInt
  ld_elem_agen.io.addr_enq.ready := ld_SSI_ADDR_Q.io.enq(0).ready
  for (i <- 1 until queuePorts) { ld_SSI_ADDR_Q.io.enq(i).valid := false.B; ld_SSI_ADDR_Q.io.enq(i).bits := DontCare }

  // .valid/.ready for this port are driven below, alongside the DGEN skid buffer.
  st_SSI_ADDR_Q.io.enq(0).bits.idx  := st_elem_agen.io.addr_enq.bits.idx
  st_SSI_ADDR_Q.io.enq(0).bits.data := st_elem_agen.io.addr_enq.bits.data.asUInt
  for (i <- 1 until queuePorts) { st_SSI_ADDR_Q.io.enq(i).valid := false.B; st_SSI_ADDR_Q.io.enq(i).bits := DontCare }

  ld_US_ADDR_Q.io.enq(0).valid := ld_range_agen.io.range.valid
  ld_US_ADDR_Q.io.enq(0).bits.idx  := resv.io.resv_resp(2)(0).base
  ld_US_ADDR_Q.io.enq(0).bits.data := ld_range_agen.io.range.bits.asUInt
  ld_range_agen.io.range.ready := ld_US_ADDR_Q.io.enq(0).ready
  for (i <- 1 until queuePorts) { ld_US_ADDR_Q.io.enq(i).valid := false.B; ld_US_ADDR_Q.io.enq(i).bits := DontCare }

  st_US_ADDR_Q.io.enq(0).valid := st_range_agen.io.range.valid
  st_US_ADDR_Q.io.enq(0).bits.idx  := resv.io.resv_resp(3)(0).base
  st_US_ADDR_Q.io.enq(0).bits.data := st_range_agen.io.range.bits.asUInt
  st_range_agen.io.range.ready := st_US_ADDR_Q.io.enq(0).ready
  for (i <- 1 until queuePorts) { st_US_ADDR_Q.io.enq(i).valid := false.B; st_US_ADDR_Q.io.enq(i).bits := DontCare }

  // ---- 8/DGEN: the SSI address->data cursor hand-off. VecElemAgen.elem_pub is
  // an un-back-pressured Valid; VecDgen.cursor is a real Decoupled that can
  // stall for several cycles on an R3 refill miss. Wiring them directly is a
  // combinational loop (cursor.valid would depend on pushFire which depends on
  // addr_enq.ready which would depend on cursor.ready). Resolved with a small
  // skid buffer: it absorbs the refill latency, and its own (registered)
  // occupancy -- not dgen's ready -- gates the SSI store's address enqueue, so
  // a beat is never presented to the address queue unless the buffer already
  // has guaranteed room for the matching elem_pub. See report. ----
  class ElemPubQEntry extends BoomBundle {
    val q_idx    = UInt(resvPtrSz.W)
    val elem_idx = UInt(vecVLSz.W)
    val seg_idx  = UInt(3.W)
    val last     = Bool()
  }
  val dgenSkidDepth = 4
  val dgenSkid = Module(new Queue(new ElemPubQEntry, dgenSkidDepth))
  val dgenSkidRoom = dgenSkid.io.count <= (dgenSkidDepth - 1).U

  st_elem_agen.io.addr_enq.ready := st_SSI_ADDR_Q.io.enq(0).ready && dgenSkidRoom
  st_SSI_ADDR_Q.io.enq(0).valid  := st_elem_agen.io.addr_enq.valid && dgenSkidRoom

  dgenSkid.io.enq.valid       := st_elem_agen.io.elem_pub.get.valid
  dgenSkid.io.enq.bits.q_idx    := st_elem_agen.io.elem_pub.get.bits.q_idx
  dgenSkid.io.enq.bits.elem_idx := st_elem_agen.io.elem_pub.get.bits.elem_idx
  dgenSkid.io.enq.bits.seg_idx  := st_elem_agen.io.elem_pub.get.bits.seg_idx
  dgenSkid.io.enq.bits.last     := st_elem_agen.io.elem_pub.get.bits.last

  dgen.io.cursor.valid       := dgenSkid.io.deq.valid
  dgen.io.cursor.bits.elem_idx := dgenSkid.io.deq.bits.elem_idx
  dgen.io.cursor.bits.seg_idx  := dgenSkid.io.deq.bits.seg_idx
  dgen.io.cursor.bits.eew      := st_opnd.io.out.bits.uop.v_eew.get
  dgen.io.cursor.bits.ordinal  := dgenSkid.io.deq.bits.q_idx
  dgen.io.cursor.bits.last     := dgenSkid.io.deq.bits.last
  dgenSkid.io.deq.ready      := dgen.io.cursor.ready

  dgen.io.brupdate := io.brupdate
  dgen.io.rob_flush := io.rob_flush
  io.vrf_r3_req := dgen.io.vrf_r3.req
  dgen.io.vrf_r3.gnt  := io.vrf_r3_gnt
  dgen.io.vrf_r3.resp := io.vrf_r3_resp

  //@req-spec-issue.d2
  //@req-spec-issue.d4
  // dgen.io.req.ready gates stPresent below, so by the time a store reaches
  // this pulse dgen is guaranteed free -- no separate FC_DGEN grant is visible
  // on this container's interface (see report on the six-step chain).
  //@req-spec-lsu.l1
  //@req-spec-lsu.l7
  //@req-spec-lsu.l8
  //@req-spec-lsu.l9
  // A segmented (is_shared) access hands off through pvtmp: dgen_operand
  // (Mux(is_shared, pvtmp, pvs3)) is VecDgen's own logic, fed here by the
  // uop this container passes through unmodified; a segmented load's pvtmp
  // placement group is likewise stamped by the agens and only read back here
  // (e.g. the LCB allocation walk's prn selection below).
  // The DGEN grant, not the AGEN pulse: VecStoreDgenPath offers it only once
  // dgen_operand_ready holds, so starting dgen on the AGEN pulse reads pvs3
  // before the producer has written it. data_base is still SAMPLED at the AGEN
  // pulse, where its resv lookup is live, and held for the later grant.
  // Held PER STORE, not in one register: a younger store's AGEN pulse can land
  // before an older store's DGEN grant, and a single register hands the older
  // store the younger one's vl and base.
  val stDgenVlTbl   = Reg(Vec(numStqEntries, UInt(vecVLSz.W)))
  val stDgenBaseTbl = Reg(Vec(numStqEntries, UInt(log2Ceil(ssiQueueEntries).W)))
  when (stOpPulse) {
    val agenIdx = GetRealLSQIdx(st_opnd.io.out.bits.uop.stq_idx)
    stDgenVlTbl(agenIdx)   := st_opnd.io.out.bits.vl
    stDgenBaseTbl(agenIdx) := Mux(stIsRangeClass, resv.io.resv_resp(3)(1).base, resv.io.resv_resp(1)(1).base)
  }
  //@req-spec-lsu.d7
  // uop, vl and data_base all name the store the issue unit granted. Taking the
  // uop from st_opnd instead sources bits and valid from different instructions.
  val stDgenIdx = GetRealLSQIdx(io.iss_st.bits.stq_idx)
  dgen.io.req.valid          := io.iss_st.valid && io.iss_st.bits.fu_code(FC_DGEN)
  dgen.io.req.bits.uop       := io.iss_st.bits
  dgen.io.req.bits.vl        := stDgenVlTbl(stDgenIdx)
  dgen.io.req.bits.data_base := stDgenBaseTbl(stDgenIdx)

  // =========================================================================
  // ---- 5. Drain side: per-queue drain pointers and the beat expanders ----
  // =========================================================================
  // None of the six queues expose a "next entry to drain" pointer (head only
  // moves at retire, via resv's free); each direction/class therefore carries
  // its own drain-position register here, adjacent to the queues in spirit.

  val ldUsElemIdxSz = log2Ceil(vecVLen + 1)

  // ---- load, unit-stride ----
  val ldUsDrainPtr = RegInit(0.U(ld_US_ADDR_Q.ptrW.W))
  val ldUsCursor   = RegInit(0.U(ldUsElemIdxSz.W))
  ld_US_ADDR_Q.io.rd(0).req.valid := true.B
  ld_US_ADDR_Q.io.rd(0).req.bits  := ldUsDrainPtr
  val ldUsDrainPtrPrev = RegNext(ldUsDrainPtr)
  val ldUsStagedValid = ld_US_ADDR_Q.io.rd(0).resp.filled && (ldUsDrainPtrPrev === ldUsDrainPtr)
  val ldUsStagedBits  = ld_US_ADDR_Q.io.rd(0).resp.data.asTypeOf(new VecRangeEntry)

  // Forward declaration. `ldRangeFaulted` is defined beside the exception shadow
  // several hundred lines below, and Scala would read a null here -- the same
  // trap the LCB-credit note in the spec calls out. Declare the Wire and connect
  // it there.
  val ldBeatStop = Wire(Bool())
  // A trimmed vleff retires its range entry HERE, because ld_beat cannot: us_pop
  // needs usAnyAdvance, which needs usGate, which needs !stop -- so a stopped
  // entry never pops itself. That is right for a trap (the flush retires it) and
  // wrong for a trim, which raises no trap: the instruction is architecturally
  // complete at the trimmed VL and its entry has no further beats to give.
  val ldUsTrimRetire = Wire(Bool())

  ld_beat.io.us_head.valid := ldUsStagedValid
  ld_beat.io.us_head.bits  := ldUsStagedBits
  ld_beat.io.us_cursor     := ldUsCursor
  ld_beat.io.stop          := ldBeatStop
  ld_beat.io.kill          := squash.io.kill(2)
  val ldUsRetire = ld_beat.io.us_pop || ldUsTrimRetire
  when (ld_beat.io.us_cursor_wr.valid) { ldUsCursor := ld_beat.io.us_cursor_wr.bits }
  when (ldUsRetire) { ldUsCursor := 0.U; ldUsDrainPtr := ldUsDrainPtr + 1.U }
  ld_US_ADDR_Q.io.consume(0).valid := ldUsRetire
  ld_US_ADDR_Q.io.consume(0).bits  := ldUsDrainPtr
  for (i <- 1 until queuePorts) { ld_US_ADDR_Q.io.consume(i).valid := false.B; ld_US_ADDR_Q.io.consume(i).bits := 0.U }
  // Only rd(0) is a live drain lane for the US queues (VecBeatExpander takes a
  // single us_head regardless of nLanes); the remaining ports+1 read lanes are
  // unused here and tied off.
  for (i <- 1 to queuePorts) { ld_US_ADDR_Q.io.rd(i).req.valid := false.B; ld_US_ADDR_Q.io.rd(i).req.bits := 0.U }

  // ---- load, SSI: one shared base index, lane i always reads base+i ----
  val ldSsiBase = RegInit(0.U(ld_SSI_ADDR_Q.ptrW.W))
  val ldSsiBasePrev = RegNext(ldSsiBase)
  for (i <- 0 until lsuWidth) {
    val idxNow  = ldSsiBase + i.U
    val idxPrev = ldSsiBasePrev + i.U
    ld_SSI_ADDR_Q.io.rd(i).req.valid := true.B
    ld_SSI_ADDR_Q.io.rd(i).req.bits  := idxNow
    val stagedValid = ld_SSI_ADDR_Q.io.rd(i).resp.filled && (idxPrev === idxNow)
    ld_beat.io.ssi_head(i).valid := stagedValid
    ld_beat.io.ssi_head(i).bits  := ld_SSI_ADDR_Q.io.rd(i).resp.data.asTypeOf(new VecElemAccess)
    ld_SSI_ADDR_Q.io.consume(i).valid := ld_beat.io.ssi_pop(i)
    ld_SSI_ADDR_Q.io.consume(i).bits  := idxNow
  }
  ldSsiBase := ldSsiBase + PopCount(ld_beat.io.ssi_pop)
  // The shared extra read port (beyond the lsuWidth drain lanes) has no
  // reader on the load address side -- forwarding/snoop only read data queues.
  ld_SSI_ADDR_Q.io.rd(queuePorts).req.valid := false.B
  ld_SSI_ADDR_Q.io.rd(queuePorts).req.bits  := 0.U

  // Driven below, beside the LCB allocation walk whose state it reads.

  // ---- store, unit-stride: two passes over the same retained entry ----
  //@req-spec-lsu.j7
  //@req-spec-lsu.j8
  val stUsDrainPtr = RegInit(0.U(st_US_ADDR_Q.ptrW.W))
  val stUsCursor   = RegInit(0.U(ldUsElemIdxSz.W))
  val stUsWritePass = RegInit(false.B)
  st_US_ADDR_Q.io.rd(0).req.valid := true.B
  st_US_ADDR_Q.io.rd(0).req.bits  := stUsDrainPtr
  val stUsDrainPtrPrev = RegNext(stUsDrainPtr)
  val stUsStagedValidRaw = st_US_ADDR_Q.io.rd(0).resp.filled && (stUsDrainPtrPrev === stUsDrainPtr)
  val stUsStagedBits = st_US_ADDR_Q.io.rd(0).resp.data.asTypeOf(new VecRangeEntry)
  val stUsCommitted = EntryValidFromAge(vec.stq_head, vec.stq_commit_head, stUsStagedBits.stq_idx)
  // Pass 1 (translate) is unconditional on the retained entry; pass 2 (write)
  // additionally requires that entry's STQ placeholder to have committed.
  val stUsStagedValid = stUsStagedValidRaw && (!stUsWritePass || stUsCommitted)

  st_beat.io.us_head.valid   := stUsStagedValid
  st_beat.io.us_head.bits    := stUsStagedBits
  st_beat.io.us_cursor       := stUsCursor
  st_beat.io.is_write_pass.get := stUsWritePass
  st_beat.io.stop            := false.B
  st_beat.io.kill            := squash.io.kill(3)

  val stUsTotalElems = stUsStagedBits.len >> stUsStagedBits.eew
  when (st_beat.io.us_cursor_wr.valid) {
    stUsCursor := st_beat.io.us_cursor_wr.bits
    // Spec defect (see report): VecRangeAgen's `len` for a segmented
    // unit-stride access is not multiplied by nf, so this pass-1-complete
    // detection (and VecBeatExpander's own completion) undercounts nf>1.
    when (!stUsWritePass && stUsStagedValidRaw && (st_beat.io.us_cursor_wr.bits >= stUsTotalElems)) {
      stUsWritePass := true.B
      stUsCursor    := 0.U
    }
  }
  when (st_beat.io.us_pop) { stUsDrainPtr := stUsDrainPtr + 1.U; stUsCursor := 0.U; stUsWritePass := false.B }
  for (i <- 0 until queuePorts) { st_US_ADDR_Q.io.consume(i).valid := false.B; st_US_ADDR_Q.io.consume(i).bits := 0.U }
  for (i <- 1 to queuePorts) { st_US_ADDR_Q.io.rd(i).req.valid := false.B; st_US_ADDR_Q.io.rd(i).req.bits := 0.U }

  val stUsXlateFire = st_beat.io.req(0).fire && st_beat.io.req(0).bits.uses_tlb && !stUsWritePass
  // ONLY the range's FIRST beat may rewrite base: every beat translates, and each
  // later paddr is base+offset, so an unqualified write-back leaves the entry
  // pointing at whichever beat translated last and the write pass starts there.
  val stUsXlateStagePrev = RegNext(stUsXlateFire && st_beat.io.req(0).bits.first, false.B)
  val stUsXlateIdxPrev   = RegNext(stUsDrainPtr)
  st_US_ADDR_Q.io.update.get(0).valid      := stUsXlateStagePrev && vec.xlate_resp(0).valid
  st_US_ADDR_Q.io.update.get(0).bits.idx   := stUsXlateIdxPrev
  st_US_ADDR_Q.io.update.get(0).bits.data  := {
    val u = WireDefault(stUsStagedBits)
    u.base := vec.xlate_resp(0).bits.paddr
    u.asUInt
  }
  for (i <- 1 until queuePorts) { st_US_ADDR_Q.io.update.get(i).valid := false.B; st_US_ADDR_Q.io.update.get(i).bits := DontCare }

  // The US store's data queue holds one entry PER GROUP MEMBER at
  // us_data_base + member, independent of the address queue's own index.
  val stUsDataMemberPtr = RegInit(0.U(log2Ceil(maxVecMembers).W))
  when (st_beat.io.us_pop) { stUsDataMemberPtr := 0.U }
    .elsewhen (st_beat.io.st_us_data_pop.get) { stUsDataMemberPtr := stUsDataMemberPtr + 1.U }
  val stUsDataIdx = stUsStagedBits.us_data_base + stUsDataMemberPtr
  st_US_DATA_Q.io.rd(0).req.valid := stUsWritePass
  st_US_DATA_Q.io.rd(0).req.bits  := stUsDataIdx
  val stUsDataIdxPrev = RegNext(stUsDataIdx)
  val stUsDataValid = st_US_DATA_Q.io.rd(0).resp.filled && (stUsDataIdxPrev === stUsDataIdx)
  st_beat.io.st_us_data.get.valid := stUsDataValid
  st_beat.io.st_us_data.get.bits  := st_US_DATA_Q.io.rd(0).resp.data.asTypeOf(new VecDgenUsPayload).data
  for (i <- 1 until queuePorts) { st_US_DATA_Q.io.rd(i).req.valid := false.B; st_US_DATA_Q.io.rd(i).req.bits := 0.U }
  for (i <- 0 until queuePorts) { st_US_DATA_Q.io.consume(i).valid := false.B; st_US_DATA_Q.io.consume(i).bits := 0.U }

  // ---- store, SSI: pass 1 (translate, opportunistic) and pass 2 (write,
  // committed-gated) share the drain read ports, one pass active per cycle.
  // Simplification (see report): pass 2 for the whole SSI store queue does not
  // start until pass 1 has drained everything claimed so far -- always safe,
  // not maximally concurrent. ----
  val stSsiPass1Base = RegInit(0.U(st_SSI_ADDR_Q.ptrW.W))
  val stSsiPass2Base = RegInit(0.U(st_SSI_ADDR_Q.ptrW.W))
  val stSsiDoPass2 = stSsiPass1Base === st_SSI_ADDR_Q.io.resv.tail
  val stSsiActiveBase = Mux(stSsiDoPass2, stSsiPass2Base, stSsiPass1Base)
  val stSsiActiveBasePrev = RegNext(stSsiActiveBase)
  val stSsiDoPass2Prev = RegNext(stSsiDoPass2, false.B)

  for (i <- 0 until lsuWidth) {
    val idxNow  = stSsiActiveBase + i.U
    val idxPrev = stSsiActiveBasePrev + i.U
    st_SSI_ADDR_Q.io.rd(i).req.valid := true.B
    st_SSI_ADDR_Q.io.rd(i).req.bits  := idxNow
    val entry = st_SSI_ADDR_Q.io.rd(i).resp.data.asTypeOf(new VecElemAccess)
    val committed = EntryValidFromAge(vec.stq_head, vec.stq_commit_head, entry.uop.stq_idx)
    val readOk = (idxPrev === idxNow) && (stSsiDoPass2Prev === stSsiDoPass2)
    val stagedValid = st_SSI_ADDR_Q.io.rd(i).resp.filled && readOk && (!stSsiDoPass2Prev || committed)
    st_beat.io.ssi_head(i).valid := stagedValid
    st_beat.io.ssi_head(i).bits  := entry

    //@req-spec-lsu.d5 (dgen pairs the SSI address/data halves by absolute index)
    st_SSI_DATA_Q.io.rd(i).req.valid := true.B
    st_SSI_DATA_Q.io.rd(i).req.bits  := idxNow
    val dataStagedValid = st_SSI_DATA_Q.io.rd(i).resp.filled && readOk
    st_beat.io.st_ssi_data.get(i).valid := dataStagedValid
    st_beat.io.st_ssi_data.get(i).bits  := st_SSI_DATA_Q.io.rd(i).resp.data.asTypeOf(new VecDgenSsiPayload).data
  }
  // ONE PORT, TWO PASS STATES: the US head and the SSI heads share
  // is_write_pass, so it must follow whichever class the expander is servicing.
  // A bare SSI assignment here last-connect-wins and strands every US store in
  // its translate pass -- it never pops, never drains, and the fence hangs.
  st_beat.io.is_write_pass.get := Mux(stUsStagedValid, stUsWritePass, stSsiDoPass2Prev)
  val stSsiAdvance = PopCount(st_beat.io.ssi_pop)
  when (stSsiDoPass2Prev) { stSsiPass2Base := stSsiPass2Base + stSsiAdvance }
    .otherwise            { stSsiPass1Base := stSsiPass1Base + stSsiAdvance }
  // A store's filled bit is never cleared by consume (VecElemQueue: consume_mask
  // is forced to 0 for isStore); the region frees only at retire, already wired.
  for (i <- 0 until lsuWidth) { st_SSI_ADDR_Q.io.consume(i).valid := false.B; st_SSI_ADDR_Q.io.consume(i).bits := 0.U }
  for (i <- 0 until lsuWidth) { st_SSI_DATA_Q.io.consume(i).valid := false.B; st_SSI_DATA_Q.io.consume(i).bits := 0.U }
  st_SSI_ADDR_Q.io.rd(queuePorts).req.valid := false.B
  st_SSI_ADDR_Q.io.rd(queuePorts).req.bits  := 0.U

  val stSsiXlateStageValid = RegInit(VecInit(Seq.fill(lsuWidth)(false.B)))
  val stSsiXlateStageIdx   = Reg(Vec(lsuWidth, UInt(st_SSI_ADDR_Q.ptrW.W)))
  val stSsiXlateStageEntry = Reg(Vec(lsuWidth, new VecElemAccess))
  for (i <- 0 until lsuWidth) {
    val fires = st_beat.io.req(i).fire && st_beat.io.req(i).bits.uses_tlb && !stSsiDoPass2Prev
    stSsiXlateStageValid(i) := fires
    stSsiXlateStageIdx(i)   := stSsiActiveBase + i.U
    stSsiXlateStageEntry(i).uop     := st_beat.io.req(i).bits.uop
    stSsiXlateStageEntry(i).vaddr   := st_beat.io.req(i).bits.vaddr
    stSsiXlateStageEntry(i).eew     := st_beat.io.req(i).bits.eew
    stSsiXlateStageEntry(i).byte_en := st_beat.io.req(i).bits.byte_en
    stSsiXlateStageEntry(i).first   := st_beat.io.req(i).bits.first
    stSsiXlateStageEntry(i).last    := st_beat.io.req(i).bits.last

    st_SSI_ADDR_Q.io.update.get(i).valid    := stSsiXlateStageValid(i) && vec.xlate_resp(i).valid
    st_SSI_ADDR_Q.io.update.get(i).bits.idx := stSsiXlateStageIdx(i)
    st_SSI_ADDR_Q.io.update.get(i).bits.data := {
      val u = WireDefault(stSsiXlateStageEntry(i))
      u.vaddr := vec.xlate_resp(i).bits.paddr
      u.asUInt
    }
  }

  // ---- DGEN's two data enqueues, into the two data queues named at the
  // absolute indices it derived from resv (SSI) / its own us_data_base (US) ----
  st_SSI_DATA_Q.io.enq(0).valid      := dgen.io.ssi_data_enq.valid
  st_SSI_DATA_Q.io.enq(0).bits.idx   := dgen.io.ssi_data_enq.bits.idx
  st_SSI_DATA_Q.io.enq(0).bits.data  := dgen.io.ssi_data_enq.bits.data.asUInt
  dgen.io.ssi_data_enq.ready         := st_SSI_DATA_Q.io.enq(0).ready
  for (i <- 1 until queuePorts) { st_SSI_DATA_Q.io.enq(i).valid := false.B; st_SSI_DATA_Q.io.enq(i).bits := DontCare }

  st_US_DATA_Q.io.enq(0).valid      := dgen.io.us_data_enq.valid
  st_US_DATA_Q.io.enq(0).bits.idx   := dgen.io.us_data_enq.bits.idx
  st_US_DATA_Q.io.enq(0).bits.data  := dgen.io.us_data_enq.bits.data.asUInt
  dgen.io.us_data_enq.ready         := st_US_DATA_Q.io.enq(0).ready
  for (i <- 1 until queuePorts) { st_US_DATA_Q.io.enq(i).valid := false.B; st_US_DATA_Q.io.enq(i).bits := DontCare }

  //@req-spec-memord.b6 (fwd's forwarding read of the store data queues -- the
  // shared port beyond the drain lanes; snoop needs no data-queue read of its
  // own under the data_filled approximation below, so no arbitration is built)
  st_SSI_DATA_Q.io.rd(queuePorts).req.valid := fwd.io.st_ssi_rd.req.valid
  st_SSI_DATA_Q.io.rd(queuePorts).req.bits  := fwd.io.st_ssi_rd.req.bits
  fwd.io.st_ssi_rd.req.ready := true.B
  fwd.io.st_ssi_rd.resp.data   := st_SSI_DATA_Q.io.rd(queuePorts).resp.data.asTypeOf(new VecDgenSsiPayload).data
  fwd.io.st_ssi_rd.resp.filled := st_SSI_DATA_Q.io.rd(queuePorts).resp.filled

  st_US_DATA_Q.io.rd(queuePorts).req.valid := fwd.io.st_us_rd.req.valid
  st_US_DATA_Q.io.rd(queuePorts).req.bits  := fwd.io.st_us_rd.req.bits
  fwd.io.st_us_rd.req.ready := true.B
  fwd.io.st_us_rd.resp.data   := st_US_DATA_Q.io.rd(queuePorts).resp.data.asTypeOf(new VecDgenUsPayload).data
  fwd.io.st_us_rd.resp.filled := st_US_DATA_Q.io.rd(queuePorts).resp.filled

  // =========================================================================
  // ---- 7. VecGroupCopy's launch, and the R2/W0 mux ----
  // =========================================================================

  //@req-spec-lsu.m13
  //@req-spec-lsu.m14
  val ldLcbTrigger = ldMaskFire && (ld_opnd.io.out.bits.uop.v_is_whole_reg.get ||
    !(ld_opnd.io.out.bits.vl_zero || ld_msk.io.all_inactive))
  // Exact complement of ldLcbTrigger: gcopy completes the group ONLY for the ops
  // that generate no memory access, or it races the load that does.
  gcopy.io.launch.valid            := ldMaskFire && !ld_opnd.io.out.bits.uop.v_is_whole_reg.get &&
    (ld_opnd.io.out.bits.vl_zero || ld_msk.io.all_inactive)
  gcopy.io.launch.bits.uop         := ld_opnd.io.out.bits.uop
  gcopy.io.launch.bits.vl_zero     := ld_opnd.io.out.bits.vl_zero
  gcopy.io.launch.bits.all_inactive := ld_msk.io.all_inactive
  gcopy.io.kill   := squash.io.kill(4)
  gcopy.io.squash.valid := squash.io.resv_rollback.valid
  gcopy.io.squash.bits  := squash.io.resv_rollback.bits.ldq_idx
  gcopy.io.flush  := io.rob_flush_kill

  gcopy.io.lcb_r2_req := lcb.io.stale_req
  lcb.io.stale_resp    := gcopy.io.lcb_r2_data
  gcopy.io.lcb_w0.valid     := lcb.io.vrf_write(0).valid
  gcopy.io.lcb_w0.bits.addr := lcb.io.vrf_write(0).bits.addr
  gcopy.io.lcb_w0.bits.data := lcb.io.vrf_write(0).bits.data
  gcopy.io.lcb_w0.bits.mask := lcb.io.vrf_write(0).bits.mask

  io.vrf_r2_req        := gcopy.io.vrf_r2_req
  gcopy.io.vrf_r2_data := io.vrf_r2_resp

  io.vrf_w0.valid     := gcopy.io.vrf_w0.valid
  io.vrf_w0.bits.addr := gcopy.io.vrf_w0.bits.addr
  io.vrf_w0.bits.data := gcopy.io.vrf_w0.bits.data
  io.vrf_w0.bits.mask := gcopy.io.vrf_w0.bits.mask

  io.vrf_w1.foreach { w1 =>
    w1.valid     := lcb.io.vrf_write(1).valid
    w1.bits.addr := lcb.io.vrf_write(1).bits.addr
    w1.bits.data := lcb.io.vrf_write(1).bits.data
    w1.bits.mask := lcb.io.vrf_write(1).bits.mask
  }

  // =========================================================================
  // ---- LCB: allocation walk (one member per cycle), responses, completion ----
  // =========================================================================

  val lcbWalkActive  = RegInit(false.B)
  val lcbWalkUop     = Reg(new MicroOp)
  val lcbWalkMembers = Reg(UInt(log2Ceil(maxVecMembers + 1).W))
  val lcbWalkIdx     = Reg(UInt(log2Ceil(maxVecMembers).W))
  val lcbWalkVl      = Reg(UInt(vecVLSz.W))
  val lcbWalkMask    = Reg(UInt(vecVLen.W))

  when (ldLcbTrigger) {
    lcbWalkActive  := true.B
    lcbWalkUop     := ld_opnd.io.out.bits.uop
    lcbWalkMembers := ld_opnd.io.out.bits.uop.v_emul.get
    lcbWalkIdx     := 0.U
    lcbWalkVl      := ld_opnd.io.out.bits.vl
    lcbWalkMask    := ld_msk.io.us_mask.bits
  }
  when (lcbWalkActive && lcb.io.alloc.ready) {
    // `+&`: at EMUL = maxVecMembers the truncating `+` wraps the narrower walk index
    // to 0, the test never holds, and the walk deadlocks the drain at free_count = 0.
    val done = (lcbWalkIdx +& 1.U) === lcbWalkMembers
    lcbWalkIdx := lcbWalkIdx + 1.U
    when (done) { lcbWalkActive := false.B }
  }
  // Connects the forward declaration at the load-admission point above, which is
  // what now ENFORCES the assert below rather than merely checking it.
  lcbWalkBusy := lcbWalkActive

  // A walk that fails to terminate has no downstream signature but a hang, and it
  // cannot be caught by bounding the index: the wrap it comes from is what hides it.
  assert(!(ldLcbTrigger && lcbWalkActive),
    "VecLsu: a load's LCB allocation walk began while the previous walk was still allocating")

  lcb.io.alloc.valid := lcbWalkActive
  lcb.io.alloc.bits.prn := Mux(lcbWalkUop.is_shared.get, lcbWalkUop.pvtmp.get(lcbWalkIdx), lcbWalkUop.pvdest.get(lcbWalkIdx))
  lcb.io.alloc.bits.ldq_idx        := lcbWalkUop.ldq_idx
  lcb.io.alloc.bits.rob_idx        := lcbWalkUop.rob_idx
  lcb.io.alloc.bits.member_idx     := lcbWalkIdx
  lcb.io.alloc.bits.members_target := lcbWalkMembers

  //@req-spec-lsu.e8
  //@req-spec-lsu.e21
  //@req-spec-lsu.e22
  //@req-spec-lsu.e23
  // active_bytes is the COMPLETION CONTRACT: the LCB holds the entry until every
  // active byte is covered, so a byte no beat will ever deliver hangs the group.
  val lcbVLenBytes = vecVLen / 8
  val lcbAllBytes  = ((BigInt(1) << lcbVLenBytes) - 1).U(lcbVLenBytes.W)

  val lcbActiveByEew = (0 until 4).map { e =>
    val epr    = lcbVLenBytes >> e
    val eprW   = log2Ceil(epr + 1)
    val first  = (lcbWalkIdx << log2Ceil(epr)).asUInt
    val rel    = (lcbWalkMask >> first)(epr - 1, 0)
    val relVl  = Mux(lcbWalkVl > first, lcbWalkVl - first, 0.U)
    // Clamped to eprW bits before the shift: the unclamped operand is vecVLSz wide
    // and would infer a shifter hundreds of bits wider than the epr+1 it can reach.
    val relVlC = Mux(relVl > epr.U, epr.U(eprW.W), relVl(eprW - 1, 0))
    val below  = ((1.U((epr + 1).W) << relVlC) - 1.U)(epr - 1, 0)
    FillInterleaved(1 << e, rel & below)
  }

  // A mask op's EVL is ceil(vl/8) BYTES in member 0; its vtype elements do not
  // describe the transfer, so the per-EEW form above does not apply to it.
  val lcbMaskBytes  = (lcbWalkVl +& 7.U) >> 3
  val lcbMaskBytesW = log2Ceil(lcbVLenBytes + 1)
  val lcbMaskBytesC = Mux(lcbMaskBytes > lcbVLenBytes.U,
    lcbVLenBytes.U(lcbMaskBytesW.W), lcbMaskBytes(lcbMaskBytesW - 1, 0))
  val lcbMaskActive = Mux(lcbWalkIdx === 0.U,
    ((1.U((lcbVLenBytes + 1).W) << lcbMaskBytesC) - 1.U)(lcbVLenBytes - 1, 0), 0.U)

  val lcbActive = Mux(lcbWalkUop.v_is_whole_reg.get, lcbAllBytes,
    Mux(lcbWalkUop.v_is_mask.get, lcbMaskActive,
      Mux1H(UIntToOH(lcbWalkUop.v_eew.get, 4), lcbActiveByEew)))

  lcb.io.alloc.bits.active_bytes   := lcbActive
  lcb.io.alloc.bits.inactive_bytes := (~lcbActive).asUInt & lcbAllBytes
  lcb.io.alloc.bits.undisturbed    := Mux(lcbWalkUop.v_is_masked.get,
    lcbWalkUop.v_mask_undist.get || lcbWalkUop.v_tail_undist.get,
    lcbWalkUop.v_tail_undist.get)
  lcb.io.alloc.bits.stale_prn      := lcbWalkUop.stale_pvdest.get(lcbWalkIdx)
  lcb.io.alloc.bits.is_ff          := lcbWalkUop.v_is_ff.get
  lcb.io.alloc.bits.pvl            := lcbWalkUop.pvl.get
  lcb.io.alloc.bits.vl_final       := lcbWalkVl

  // Hand the beat expander the three RAW terms, not a reduced ready bit. A free
  // entry is needed only while the walk is still ALLOCATING; once it has
  // finished, that op's entries exist and its beats need no new one -- gating
  // them on free_count alone deadlocks at EMUL = lcbEntries, where one op owns
  // every entry. The third term is the op identity, and it must be compared
  // against the head the BEAT belongs to. Only the expander knows that: its
  // unit-stride path reads `us_head` while each SSI lane reads its own
  // `ssi_head(i)`, and those are different in-flight ops. Reducing here would
  // force one of them onto both paths -- which is exactly how the strided
  // (`vlse`/SSI) path kept starving after the unit-stride path was fixed.
  //
  // Observed on `axpy-vector` (LMUL=8, e64, lcbEntries = 2*maxVecMembers): two
  // groups owned all 16 entries, a third op's walk stalled, and the second
  // group's cursor froze mid-stream at element 25 of 32 -- members 6 and 7 never
  // filled, so its group_done never fired and rename deadlocked on an empty
  // vector free list, tens of thousands of cycles later and three modules away.
  ld_beat.io.lcb_free_nonzero.get := lcb.io.free_count =/= 0.U
  ld_beat.io.lcb_walk_active.get  := lcbWalkActive
  ld_beat.io.lcb_walk_rob.get     := lcbWalkUop.rob_idx

  //@req-spec-lsu.g7 (vleff's element-i>0 trim -> LCB {member, keep_bytes})
  ld_range_agen.io.ff_trim.foreach { trim =>
    val trimElem   = trim.bits
    val eew        = ldUsStagedBits.eew
    val bytePos    = trimElem << eew
    val member     = bytePos(log2Ceil(vecVLen / 8) + log2Ceil(maxVecMembers) - 1, log2Ceil(vecVLen / 8))
    val byteOff    = bytePos(log2Ceil(vecVLen / 8) - 1, 0)
    val keepBytes  = (((1.U << byteOff) - 1.U) | (0.U((vecVLen / 8).W)))((vecVLen / 8) - 1, 0)
    lcb.io.trim.valid        := trim.valid
    lcb.io.trim.bits.ldq_idx := ldUsStagedBits.ldq_idx
    lcb.io.trim.bits.vl_final := trimElem
    lcb.io.trim.bits.member   := member
    lcb.io.trim.bits.keep_bytes := keepBytes
  }
  if (ld_range_agen.io.ff_trim.isEmpty) {
    lcb.io.trim.valid := false.B
    lcb.io.trim.bits  := DontCare
  }
  // ld_range_agen.io.fault is driven below, beside the exception shadow it
  // depends on.
  ld_elem_agen.io.fault.valid := false.B
  ld_elem_agen.io.fault.bits  := 0.U
  st_elem_agen.io.fault.valid := false.B
  st_elem_agen.io.fault.bits  := 0.U

  // ---- LCB responses: the destination (prn, byte_off) comes off the response's
  // OWN uop, which the D$ carries back unchanged. src_off/nbytes/nelem are the
  // request's alignment, which the response does not carry, so each in-flight
  // beat reserves a tag and parks its alignment in a tag-keyed table. Keyed by
  // tag and not by lane because ll_resp (the miss return) always arrives on lane
  // lsuWidth-1 regardless of which lane issued the request; per-lane replay in
  // request order therefore aligns a miss against the wrong request as soon as
  // lsuWidth > 1, and tolerates no nack. ----
  // The whole request is parked, not just the alignment: a nacked beat is replayed
  // from here, so the table is both the alignment record and the replay source.
  val nLdTags     = ldRespTags
  val ldTagBusy   = RegInit(0.U(nLdTags.W))
  val ldTagReplay = RegInit(0.U(nLdTags.W))
  val ldTagTable  = Reg(Vec(nLdTags, new VecMemAccess))

  // Every lane's tag is picked from the busy REGISTER alone, never from another
  // lane's fire this cycle: the arbiter's round-robin makes lane 0's grant depend
  // on lane 1's request, so a valid gated on a same-cycle allocation is a loop.
  val ldTagAvail = PopCount((~ldTagBusy).asUInt) >= lsuWidth.U
  val ldLaneTag  = Wire(Vec(lsuWidth, UInt(ldRespTagSz.W)))
  var ldFreeMask: UInt = (~ldTagBusy).asUInt
  for (w <- 0 until lsuWidth) {
    ldLaneTag(w) := PriorityEncoder(ldFreeMask)
    ldFreeMask = ldFreeMask & (~UIntToOH(ldLaneTag(w), nLdTags)).asUInt
  }

  // Replay candidates also come from the register alone, for the same loop reason.
  val ldRpyPick  = Wire(Vec(lsuWidth, UInt(ldRespTagSz.W)))
  val ldRpyValid = Wire(Vec(lsuWidth, Bool()))
  var ldRpyMask: UInt = ldTagReplay
  for (w <- 0 until lsuWidth) {
    ldRpyValid(w) := ldRpyMask.orR
    ldRpyPick(w)  := PriorityEncoder(ldRpyMask)
    ldRpyMask = ldRpyMask & (~UIntToOH(ldRpyPick(w), nLdTags)).asUInt
  }

  val ldTagSet = Wire(Vec(lsuWidth, UInt(nLdTags.W)))
  val ldTagClr = Wire(Vec(lsuWidth, UInt(nLdTags.W)))
  val ldRpySet = Wire(Vec(lsuWidth, UInt(nLdTags.W)))
  val ldRpyClr = Wire(Vec(lsuWidth, UInt(nLdTags.W)))
  for (w <- 0 until lsuWidth) {
    val isLdFire = arb.io.vec_fire(w).valid && !arb.io.vec_fire(w).bits.uop.uses_stq
    // A replayed beat already owns its tag and its table row; only a fresh beat allocates.
    val isFreshFire = isLdFire && !ldRpyValid(w)
    ldTagSet(w) := Mux(isFreshFire, UIntToOH(ldLaneTag(w), nLdTags), 0.U)
    when (isFreshFire) {
      ldTagTable(ldLaneTag(w)) := arb.io.vec_fire(w).bits
    }
    ldRpyClr(w) := Mux(isLdFire && ldRpyValid(w), UIntToOH(ldRpyPick(w), nLdTags), 0.U)

    // A nack does NOT free the tag -- it marks the beat for replay and keeps the tag
    // (and its parked request) alive. Freeing here instead would drop the beat: the
    // beat expander has already advanced its cursor past it and will never re-emit it.
    val respIsLd = vec.resp(w).valid && !vec.resp(w).bits.uop.uses_stq
    val nackIsLd = vec.nack(w).valid && !vec.nack(w).bits.uop.uses_stq
    ldTagClr(w) := Mux(respIsLd, UIntToOH(vec.resp(w).bits.uop.v_mem_tag.get, nLdTags), 0.U)
    ldRpySet(w) := Mux(nackIsLd, UIntToOH(vec.nack(w).bits.uop.v_mem_tag.get, nLdTags), 0.U)

    when (respIsLd) {
      VecTrace.traceId("VecLsu", "ld_resp", vec.resp(w).bits.uop.rob_idx,
        Seq(("lane", w.U), ("tag", vec.resp(w).bits.uop.v_mem_tag.get)))
    }
    when (nackIsLd) {
      VecTrace.traceId("VecLsu", "ld_nack", vec.nack(w).bits.uop.rob_idx,
        Seq(("lane", w.U), ("tag", vec.nack(w).bits.uop.v_mem_tag.get)))
    }
    when (vec.nack(w).valid && vec.nack(w).bits.uop.uses_stq) {
      VecTrace.traceId("VecLsu", "st_nack", vec.nack(w).bits.uop.rob_idx, Seq(("lane", w.U),
        ("tag", vec.nack(w).bits.uop.v_mem_tag.get), ("is_vec", vec.nack(w).bits.uop.is_vec.get.asUInt)))
    }
    when (vec.store_ack(w).valid) {
      VecTrace.traceId("VecLsu", "st_ack", vec.store_ack(w).bits.uop.rob_idx, Seq(("lane", w.U),
        ("tag", vec.store_ack(w).bits.uop.v_mem_tag.get), ("is_vec", vec.store_ack(w).bits.uop.is_vec.get.asUInt)))
    }

    val shadow = ldTagTable(vec.resp(w).bits.uop.v_mem_tag.get)
    assert(!(respIsLd && !ldTagBusy(vec.resp(w).bits.uop.v_mem_tag.get)),
      "VecLsu: load response with an unallocated tag -- alignment would be stale")

    lcb.io.resp(w).valid        := vec.resp(w).valid
    lcb.io.resp(w).bits.data     := vec.resp(w).bits.data
    lcb.io.resp(w).bits.prn      := vec.resp(w).bits.uop.v_split_dst_prn.get
    lcb.io.resp(w).bits.ldq_idx  := vec.resp(w).bits.uop.ldq_idx
    lcb.io.resp(w).bits.dst_byte := vec.resp(w).bits.uop.v_split_dst_byte_off.get
    lcb.io.resp(w).bits.src_off  := shadow.vaddr(log2Ceil(coreDataBytes) - 1, 0)
    lcb.io.resp(w).bits.nbytes   := PopCount(shadow.byte_en)
    lcb.io.resp(w).bits.nelem    := PopCount(shadow.byte_en) >> shadow.eew
  }
  ldTagBusy   := (ldTagBusy | ldTagSet.reduce(_ | _)) & (~ldTagClr.reduce(_ | _)).asUInt
  ldTagReplay := (ldTagReplay | ldRpySet.reduce(_ | _)) &
                 (~(ldRpyClr.reduce(_ | _) | ldTagClr.reduce(_ | _))).asUInt

  //@req-spec-rob.d16
  io.vec_group_done(0) := lcb.io.group_done
  io.vec_group_done(1) := gcopy.io.group_done
  io.vec_clr_bsy(0).valid := lcb.io.group_done.valid
  io.vec_clr_bsy(0).bits  := lcb.io.group_done.bits.rob_idx
  io.vec_clr_bsy(1).valid := gcopy.io.group_done.valid
  io.vec_clr_bsy(1).bits  := gcopy.io.group_done.bits.rob_idx
  io.vec_rob_flags(0).valid := false.B
  io.vec_rob_flags(0).bits  := DontCare
  io.vec_rob_flags(1).valid := false.B
  io.vec_rob_flags(1).bits  := DontCare

  //@req-spec-lsu.a8
  vec.ld_group_done := lcb.io.group_done_ldq

  // =========================================================================
  // ---- 4 (cont). Arbiter ----
  // =========================================================================

  //@req-spec-lsu.k10
  //@req-spec-rob.d13
  //@req-spec-rob.d17
  //@req-spec-lsu.j5
  // Both drains present to arb, which reuses the scalar cache port (one
  // dmem.req lane per lsuWidth) rather than a VLEN-wide interface; the load
  // path's dispatch/eligibility/fire/writeback are otherwise the scalar
  // structures unchanged -- this file adds only the vector queues, the LCB,
  // and the element cursor already threaded through the beat expanders.
  // A load beat may not fire without a free tag to record its response alignment
  // under; firing without one would leave the response unable to find its request.
  // A replay outranks a fresh beat on its lane: it is older, its tag is already
  // spent, and the group it belongs to cannot complete until it lands.
  for (w <- 0 until lsuWidth) {
    arb.io.ld_req(w).valid := Mux(ldRpyValid(w), true.B,
      ld_beat.io.req(w).valid && ldTagAvail)
    arb.io.ld_req(w).bits  := Mux(ldRpyValid(w), ldTagTable(ldRpyPick(w)),
      ld_beat.io.req(w).bits)
    when (!ldRpyValid(w)) {
      arb.io.ld_req(w).bits.uop.v_mem_tag.get := ldLaneTag(w)
    }
    ld_beat.io.req(w).ready := !ldRpyValid(w) && arb.io.ld_req(w).ready && ldTagAvail

    when (arb.io.vec_fire(w).valid && !arb.io.vec_fire(w).bits.uop.uses_stq) {
      VecTrace.traceId("VecLsu", "ld_fire", arb.io.vec_fire(w).bits.uop.rob_idx, Seq(("lane", w.U),
        ("tag", arb.io.vec_fire(w).bits.uop.v_mem_tag.get), ("rpy", ldRpyValid(w).asUInt),
        ("busy", ldTagBusy), ("rpy_mask", ldTagReplay)))
    }
    val ldBlocked = (ld_beat.io.req(w).valid || ldRpyValid(w)) && !arb.io.ld_req(w).ready
    when (ldBlocked) {
      VecTrace.traceId("VecLsu", "ld_blocked", ld_beat.io.req(w).bits.uop.rob_idx, Seq(("lane", w.U),
        ("beat_v", ld_beat.io.req(w).valid.asUInt), ("avail", ldTagAvail.asUInt),
        ("rpy", ldRpyValid(w).asUInt), ("busy", ldTagBusy), ("rpy_mask", ldTagReplay)))
    }

    //@req-spec-lsu.a12
    // D$-ACCEPTANCE WATCHDOG. A lane holding a beat the arbiter will not take is
    // normal for a few cycles -- the scalar side wins, or an MSHR is filling.
    // Permanently refused is a different thing: the beats behind it are the ones
    // a destination group is still waiting on, so the group never completes and
    // the deadlock surfaces two modules away, in rename, as an empty free list.
    //
    // This is the assertion that names the ACTUAL stall point. On `axpy-vector`
    // the lane sat here with `beat_v=1 avail=1` -- a beat ready to go and a tag
    // free to carry it -- while the arbiter refused it forever. Both of those
    // being high is what rules out the vector side and puts the fault at the
    // cache interface, so keep them in the message.
    if (ldAcceptWatchdog > 0) {
      val blocked_cnt = RegInit(0.U(log2Ceil(ldAcceptWatchdog + 2).W))
      when (ldBlocked) {
        blocked_cnt := blocked_cnt + 1.U
      } .otherwise {
        blocked_cnt := 0.U
      }
      assert(blocked_cnt <= ldAcceptWatchdog.U,
        s"VecLsu: load lane $w has been refused by the D\\$$ arbiter for ldAcceptWatchdog " +
        "cycles with a beat pending -- the destination group can never fill, so its " +
        "group_done will never fire and rename will deadlock on an empty free list. " +
        "Check the arbiter's grant policy and whether an MSHR is stuck.")
    }
  }
  // ---- Store beats get the same tag pool and replay path. A store returns no data,
  // so its tag exists only to name a nacked beat for replay, and is freed by
  // store_ack. Only a WRITE-pass beat reaches the D$ and can be nacked, so the
  // translate pass allocates nothing. ----
  val stTagBusy   = RegInit(0.U(nLdTags.W))
  val stTagReplay = RegInit(0.U(nLdTags.W))
  val stTagTable  = Reg(Vec(nLdTags, new VecMemAccess))

  val stTagAvail = PopCount((~stTagBusy).asUInt) >= lsuWidth.U
  val stLaneTag  = Wire(Vec(lsuWidth, UInt(ldRespTagSz.W)))
  var stFreeMask: UInt = (~stTagBusy).asUInt
  for (w <- 0 until lsuWidth) {
    stLaneTag(w) := PriorityEncoder(stFreeMask)
    stFreeMask = stFreeMask & (~UIntToOH(stLaneTag(w), nLdTags)).asUInt
  }
  // A WRITE-pass beat may only fire on lane 0. dcache.scala reports
  // io.lsu.store_ack ONLY for w == 0 (`&& (w == 0).B`), so a successful beat on any
  // other lane is never acknowledged and its tag leaks until the pool starves.
  val stRpyPick  = Wire(Vec(lsuWidth, UInt(ldRespTagSz.W)))
  val stRpyValid = Wire(Vec(lsuWidth, Bool()))
  var stRpyMask: UInt = stTagReplay
  for (w <- 0 until lsuWidth) {
    if (w == 0) {
      stRpyValid(w) := stRpyMask.orR
      stRpyPick(w)  := PriorityEncoder(stRpyMask)
      stRpyMask = stRpyMask & (~UIntToOH(stRpyPick(w), nLdTags)).asUInt
    } else {
      stRpyValid(w) := false.B
      stRpyPick(w)  := 0.U
    }
  }

  val stTagSet = Wire(Vec(lsuWidth, UInt(nLdTags.W)))
  val stTagClr = Wire(Vec(lsuWidth, UInt(nLdTags.W)))
  val stRpySet = Wire(Vec(lsuWidth, UInt(nLdTags.W)))
  val stRpyClr = Wire(Vec(lsuWidth, UInt(nLdTags.W)))
  val stFiredOH = Wire(Vec(lsuWidth, UInt(nLdTags.W)))
  for (w <- 0 until lsuWidth) {
    val isStFire  = arb.io.vec_fire(w).valid && arb.io.vec_fire(w).bits.uop.uses_stq &&
      arb.io.vec_fire(w).bits.uses_dcache
    val isFreshSt = isStFire && !stRpyValid(w)
    stTagSet(w) := Mux(isFreshSt, UIntToOH(stLaneTag(w), nLdTags), 0.U)
    when (isFreshSt) { stTagTable(stLaneTag(w)) := arb.io.vec_fire(w).bits }
    stRpyClr(w) := Mux(isStFire && stRpyValid(w), UIntToOH(stRpyPick(w), nLdTags), 0.U)

    stFiredOH(w) := Mux(isStFire,
      UIntToOH(Mux(stRpyValid(w), stRpyPick(w), stLaneTag(w)), nLdTags), 0.U)

    val ackIsSt  = vec.store_ack(w).valid && vec.store_ack(w).bits.uop.uses_stq
    val nackIsSt = vec.nack(w).valid && vec.nack(w).bits.uop.uses_stq
    stTagClr(w) := Mux(ackIsSt, UIntToOH(vec.store_ack(w).bits.uop.v_mem_tag.get, nLdTags), 0.U)
    stRpySet(w) := Mux(nackIsSt, UIntToOH(vec.nack(w).bits.uop.v_mem_tag.get, nLdTags), 0.U)
  }

  // A nacked store makes the D$ squash the beats in its s0 and s1 too, with neither
  // store_ack nor nack (dcache.scala's s2_store_failed feeds s1_valid and s2_valid).
  // vec_fire drives dmem_req combinationally, so those two are the beat firing this
  // cycle and the beat that fired last cycle. Without replaying them their tags stay
  // busy forever and their bytes never reach memory.
  val stFiredThisOH = stFiredOH.reduce(_ | _)
  val stFiredPrevOH = RegNext(stFiredThisOH, 0.U(nLdTags.W))
  val stKillSet = Mux(vec.store_failed, stFiredThisOH | stFiredPrevOH, 0.U(nLdTags.W))

  stTagBusy   := (stTagBusy | stTagSet.reduce(_ | _)) & (~stTagClr.reduce(_ | _)).asUInt
  // stKillSet is OR-ed AFTER the clears: a squashed beat that was itself a replay has
  // its bit in stRpyClr from firing, and the squash must win or the beat is lost.
  stTagReplay := ((stTagReplay | stRpySet.reduce(_ | _)) &
                 (~(stRpyClr.reduce(_ | _) | stTagClr.reduce(_ | _))).asUInt) | stKillSet

  // A squashed beat cannot also be acked this cycle: an ack arrives at s2, two cycles
  // after its fire, and both victims are younger than that.
  assert((stKillSet & stTagClr.reduce(_ | _)) === 0.U,
    "VecLsu: a store beat was acked and squashed in the same cycle")

  for (w <- 0 until lsuWidth) {
    // Lanes above 0 carry translate-pass beats only; the expander leaves the cursor
    // where it is when a lane does not fire, so the write pass degrades to one
    // beat per cycle instead of deadlocking.
    val stLaneOk = if (w == 0) true.B else !st_beat.io.req(w).bits.uses_dcache
    arb.io.st_req(w).valid := Mux(stRpyValid(w), true.B,
      st_beat.io.req(w).valid && stTagAvail && stLaneOk)
    arb.io.st_req(w).bits  := Mux(stRpyValid(w), stTagTable(stRpyPick(w)),
      st_beat.io.req(w).bits)
    when (!stRpyValid(w)) {
      arb.io.st_req(w).bits.uop.v_mem_tag.get := stLaneTag(w)
    }
    st_beat.io.req(w).ready := !stRpyValid(w) && arb.io.st_req(w).ready && stTagAvail && stLaneOk

    when (arb.io.vec_fire(w).valid && arb.io.vec_fire(w).bits.uop.uses_stq &&
          arb.io.vec_fire(w).bits.uses_dcache) {
      VecTrace.traceId("VecLsu", "st_fire", arb.io.vec_fire(w).bits.uop.rob_idx, Seq(("lane", w.U),
        ("tag", arb.io.vec_fire(w).bits.uop.v_mem_tag.get), ("rpy", stRpyValid(w).asUInt),
        ("busy", stTagBusy), ("rpy_mask", stTagReplay)))
    }
    when (st_beat.io.req(w).valid && !arb.io.st_req(w).ready) {
      VecTrace.traceId("VecLsu", "st_blocked", st_beat.io.req(w).bits.uop.rob_idx, Seq(("lane", w.U),
        ("avail", stTagAvail.asUInt), ("rpy", stRpyValid(w).asUInt),
        ("busy", stTagBusy), ("rpy_mask", stTagReplay)))
    }
  }
  for (w <- 0 until lsuWidth) {
    arb.io.scalar_demand(w) := vec.scalar_demand(w).claim
    arb.io.scalar_avail(w)  := vec.scalar_avail(w)
    arb.io.dmem_req_ready(w) := vec.dmem_req_ready(w)
  }
  arb.io.lcb_free_count := lcb.io.free_count
  arb.io.hold_ldq        := hold.io.hold_ldq

  vec.vec_claim := arb.io.vec_claim
  vec.vec_fire  := arb.io.vec_fire

  // =========================================================================
  // ---- 10. Memory ordering: snoop / forward / hold ----
  // =========================================================================

  //@req-spec-agen.e13
  // Item (a): VecRangeEntry.mask is element-granular; VecSnoopCandidate's
  // active_mask is byte-granular. Expand explicitly rather than reuse the two
  // fields verbatim -- they coincide in width at the defaults, which is why
  // skipping this conversion would be invisible until measured.
  def expandElemMaskToBytes(elemMask: UInt, eew: UInt): UInt = {
    val outW = maxVecMembers * vecVLen / 8
    val variants = (0 until 4).map { e =>
      val perElem = 1 << e
      val nElems  = math.min(elemMask.getWidth, outW / perElem)
      val bits    = (0 until nElems).flatMap(k => Seq.fill(perElem)(elemMask(k)))
      val padded  = bits ++ Seq.fill(outW - bits.length)(false.B)
      e.U -> VecInit(padded).asUInt
    }
    MuxLookup(eew, 0.U(outW.W))(variants)
  }

  //@req-spec-memord.a15
  //@req-spec-memord.a26 (the item-(a) mask-granularity conversion)
  // Candidates are built from the arbiter's OWN grants (one per lsuWidth
  // lane): every granted beat with uses_lcam set needs an LCAM presentation,
  // and lcam_range_len distinguishes the US range beat (nonzero, first beat
  // only, whose active_mask is the RETAINED range entry's element mask
  // expanded to bytes) from an SSI per-element beat (whose active_mask is
  // just that element's own byte_en, zero-extended).
  for (w <- 0 until lsuWidth) {
    val fired    = arb.io.vec_fire(w)
    val isStoreWinner = arb.io.st_req(w).fire
    val isUs     = fired.bits.lcam_range_len =/= 0.U
    snoop.io.cand(w).valid          := fired.valid && fired.bits.uses_lcam
    snoop.io.cand(w).bits.is_store       := isStoreWinner
    snoop.io.cand(w).bits.is_unit_stride := isUs
    snoop.io.cand(w).bits.paddr          := fired.bits.vaddr
    snoop.io.cand(w).bits.len            := Mux(isUs, fired.bits.lcam_range_len, 1.U << fired.bits.eew)
    snoop.io.cand(w).bits.eew            := fired.bits.eew
    snoop.io.cand(w).bits.active_mask    :=
      Mux(isUs, expandElemMaskToBytes(Mux(isStoreWinner, stUsStagedBits.mask, ldUsStagedBits.mask), fired.bits.eew),
                fired.bits.byte_en.pad(maxVecMembers * vecVLen / 8))
    snoop.io.cand(w).bits.uop            := fired.bits.uop
    snoop.io.cand(w).bits.queue_idx      := Mux(isStoreWinner, stSsiActiveBase, ldSsiBase) + w.U
    snoop.io.cand(w).bits.ordinal        := Mux(isStoreWinner, stSsiActiveBase, ldSsiBase) + w.U
    snoop.io.cand(w).bits.q_base         := Mux(isStoreWinner, stSsiPass1Base, 0.U)
    snoop.io.cand(w).bits.members        := Mux(isStoreWinner, stUsStagedBits.members, ldUsStagedBits.members)
    snoop.io.cand(w).bits.us_data_base   := Mux(isStoreWinner, stUsStagedBits.us_data_base, 0.U)
    //@req-spec-memord.a22
    // Answered from the data queues' OWN filled bits. This was previously
    // approximated as a constant true on the argument that resv claims the
    // address and data regions together, so data is always written first. It is
    // not: with the scalar forward enabled, `ms4p5_vle64_2` trips
    // VecStoreForward's own "SSI forward read an unfilled data-queue entry"
    // assertion, because the constant defeats the snoop's `c.ready` data gate
    // and lets a store address reach the LCAM ahead of its bytes. Forwarding
    // then reads an unwritten entry, which is silent corruption everywhere the
    // assertion is compiled out.
    //
    // A US candidate describes ONE range whose data is `members` entries, and a
    // future load may want any of them, so every member must be filled; an SSI
    // candidate is one element at the same ordinal as its address entry.
    val stUsDataFilled = {
      val f    = st_US_DATA_Q.io.filled_vec
      val base = stUsStagedBits.us_data_base
      (0 until usQueueEntries).map { k =>
        !(k.U < stUsStagedBits.members) || f((base + k.U)(st_US_DATA_Q.idxW - 1, 0))
      }.reduce(_ && _)
    }
    val stSsiDataFilled =
      st_SSI_DATA_Q.io.filled_vec((stSsiActiveBase + w.U)(st_SSI_DATA_Q.idxW - 1, 0))
    snoop.io.cand(w).bits.data_filled    :=
      Mux(!isStoreWinner, true.B, Mux(isUs, stUsDataFilled, stSsiDataFilled))
  }
  snoop.io.ld_search     := vec.ld_search
  snoop.io.stq_vec_valid := vec.stq_vec_valid
  snoop.io.stq_alloc     := vec.st_alloc
  snoop.io.brupdate      := io.brupdate
  snoop.io.rob_flush     := io.rob_flush
  vec.lcam      := snoop.io.lcam
  vec.vst_match := snoop.io.vst_addr_match

  fwd.io.ld_search  := vec.ld_search
  fwd.io.snoop_cand := snoop.io.snoop_cand
  fwd.io.stq_addr_matches    := vec.stq_addr_matches
  fwd.io.stq_forward_matches := vec.stq_forward_matches
  fwd.io.brupdate := io.brupdate
  fwd.io.rob_flush := io.rob_flush
  vec.fwd_resp := fwd.io.fwd_resp
  vec.replay   := fwd.io.replay

  //@req-spec-memord.b19
  // st_drained is a LEVEL, keyed on the post-commit write-pass cursor, never
  // the translate cursor. Retirement of a vector STQ entry (which clears
  // stq_vec_valid) is itself gated on this container's own st_drain_done
  // (below), which only fires once the write pass has completed, so "not a
  // live vector store" already implies "write pass complete" here.
  val stDrained = VecInit((0 until numStqEntries).map(i => !vec.stq_vec_valid(i)))

  //@req-spec-memord.b16
  hold.io.ld_ctx := DontCare
  for (w <- 0 until lsuWidth) {
    val ls = vec.ld_search(w)
    hold.io.ld_ctx(w).valid                := ls.valid
    hold.io.ld_ctx(w).bits.ldq_idx          := ls.bits.ldq_idx
    hold.io.ld_ctx(w).bits.rob_idx          := ls.bits.uop.rob_idx
    hold.io.ld_ctx(w).bits.is_vec           := ls.bits.is_vec
    hold.io.ld_ctx(w).bits.is_unit_stride   := ls.bits.is_unit_stride
  }
  hold.io.known_overlap := fwd.io.known_overlap
  hold.io.pred_overlap  := vec.pred_overlap
  hold.io.st_drained    := stDrained
  hold.io.ldq_valid     := vec.ldq_valid
  hold.io.ldq_alloc     := vec.ld_alloc
  hold.io.stq_head        := vec.stq_head
  hold.io.ldq_next_stq_idx := vec.ldq_next_stq_idx

  // =========================================================================
  // ---- squash: kill_uop roster (fixed order: 0 ld_msk, 1 st_msk, 2 ld_beat,
  // 3 st_beat, 4 gcopy) ----
  // =========================================================================
  //@req-spec-lsu.i6
  squash.io.kill_uop(0).valid := ldStreamerBusy
  squash.io.kill_uop(0).bits  := ld_opnd.io.out.bits.uop
  squash.io.kill_uop(1).valid := stStreamerBusy
  squash.io.kill_uop(1).bits  := st_opnd.io.out.bits.uop

  // Spec-adjacent gap (see report): VecRangeEntry carries no br_mask, so the
  // drain-side beat expanders cannot be selectively branch-killed -- only a
  // whole-machine rob_flush reaches them (IsKilledByBranch degrades to that
  // when br_mask is zero). SSI drain lanes are additionally covered by the
  // underlying queue's own squash_mask, which does carry real per-entry state.
  val ldBeatKillUop = WireDefault(0.U.asTypeOf(new MicroOp))
  ldBeatKillUop.rob_idx := ldUsStagedBits.rob_idx
  squash.io.kill_uop(2).valid := ldUsStagedValid || (0 until lsuWidth).map(i => ld_beat.io.ssi_head(i).valid).reduce(_ || _)
  squash.io.kill_uop(2).bits  := ldBeatKillUop

  val stBeatKillUop = WireDefault(0.U.asTypeOf(new MicroOp))
  stBeatKillUop.rob_idx := stUsStagedBits.rob_idx
  squash.io.kill_uop(3).valid := stUsStagedValid || (0 until lsuWidth).map(i => st_beat.io.ssi_head(i).valid).reduce(_ || _)
  squash.io.kill_uop(3).bits  := stBeatKillUop

  squash.io.kill_uop(4).valid := gcopy.io.launch.valid
  squash.io.kill_uop(4).bits  := gcopy.io.launch.bits.uop

  // =========================================================================
  // ---- 6. Completion: store group-safe / busy-clear, and the exception tap ----
  // =========================================================================

  //@req-spec-rob.e5
  //@req-spec-rob.c6
  //@req-spec-rob.c7
  //@req-spec-rob.c9
  // A store's group-safe (translate-pass complete) and, for a non-shared
  // store, its busy-clear fire together on the SAME event; a shared store's
  // busy-clear instead waits for dgen's last SSI push (decision c9).
  val stUsPass1CompleteThisCycle = stUsStagedValidRaw && !stUsWritePass &&
    st_beat.io.us_cursor_wr.valid && (st_beat.io.us_cursor_wr.bits >= stUsTotalElems)
  val stSsiPass1CompleteThisCycle = Wire(Vec(lsuWidth, Bool()))
  for (i <- 0 until lsuWidth) {
    val entry = st_SSI_ADDR_Q.io.rd(i).resp.data.asTypeOf(new VecElemAccess)
    stSsiPass1CompleteThisCycle(i) := !stSsiDoPass2Prev && st_beat.io.ssi_pop(i) && entry.last
  }
  val dgenLastSsiPush = dgen.io.ssi_data_enq.fire && dgen.io.ssi_data_enq.bits.data.last

  val stGroupSafeUs  = stUsPass1CompleteThisCycle
  val stGroupSafeSsi = stSsiPass1CompleteThisCycle.reduce(_ || _)
  val ssiPass1RobIdx   = PriorityMux(stSsiPass1CompleteThisCycle,
    (0 until lsuWidth).map(i => st_SSI_ADDR_Q.io.rd(i).resp.data.asTypeOf(new VecElemAccess).uop.rob_idx))
  val ssiPass1StqIdx   = PriorityMux(stSsiPass1CompleteThisCycle,
    (0 until lsuWidth).map(i => st_SSI_ADDR_Q.io.rd(i).resp.data.asTypeOf(new VecElemAccess).uop.stq_idx))
  val ssiPass1IsShared = PriorityMux(stSsiPass1CompleteThisCycle,
    (0 until lsuWidth).map(i => st_SSI_ADDR_Q.io.rd(i).resp.data.asTypeOf(new VecElemAccess).uop.is_shared.get))

  //@req-spec-rob.e1
  //@req-spec-rob.e2
  //@req-spec-rob.e3
  // A load's group-safe: the LCAM pass's cursor reached its active element
  // count (the last SSI beat granted, or the one US range query granted).
  val ldGroupSafeHits = (0 until lsuWidth).map { w =>
    val r = arb.io.ld_req(w)
    val isUsFirst = r.bits.lcam_range_len =/= 0.U
    r.fire && r.bits.uses_lcam && (isUsFirst || r.bits.last)
  }
  val ldGroupSafeAny    = ldGroupSafeHits.reduce(_ || _)
  val ldGroupSafeRobIdx = PriorityMux(ldGroupSafeHits, (0 until lsuWidth).map(w => arb.io.ld_req(w).bits.uop.rob_idx))
  val ldGroupSafeLdqIdx = PriorityMux(ldGroupSafeHits, (0 until lsuWidth).map(w => arb.io.ld_req(w).bits.uop.ldq_idx))

  // Known limitation (see report): vec_clr_unsafe/group_safe are single-event
  // ports; a same-cycle collision between a load's and a store's group-safe
  // is resolved with load priority, and the losing event is dropped rather
  // than queued.
  io.vec_clr_unsafe.valid := ldGroupSafeAny || stGroupSafeUs || stGroupSafeSsi
  io.vec_clr_unsafe.bits  := Mux(ldGroupSafeAny, ldGroupSafeRobIdx, Mux(stGroupSafeUs, stUsStagedBits.rob_idx, ssiPass1RobIdx))

  //@req-spec-lsu.a9
  vec.group_safe.valid        := io.vec_clr_unsafe.valid
  vec.group_safe.bits.is_load := ldGroupSafeAny
  vec.group_safe.bits.idx     := Mux(ldGroupSafeAny, ldGroupSafeLdqIdx, Mux(stGroupSafeUs, stUsStagedBits.stq_idx, ssiPass1StqIdx))

  // Assumption (see report): a unit-stride store is never is_shared -- segment
  // count only pairs with indexed/strided addressing in this design, and
  // VecRangeEntry has no is_shared bit to check even if that assumption fails.
  val dgenReqIsShared = st_opnd.io.out.bits.uop.is_shared.get
  val stPassDoneUs  = stGroupSafeUs
  val stPassDoneSsi = stGroupSafeSsi && !ssiPass1IsShared
  val stPassDoneShared = dgenLastSsiPush && dgenReqIsShared
  vec.st_pass_done.valid := stPassDoneUs || stPassDoneSsi || stPassDoneShared
  vec.st_pass_done.bits := Mux(stPassDoneShared, dgen.io.ssi_data_enq.bits.data.stq_idx,
    Mux(stPassDoneUs, stUsStagedBits.stq_idx, ssiPass1StqIdx))

  //@req-spec-lsu.j11
  val stUsPass2Done = stUsStagedValidRaw && stUsWritePass && st_beat.io.us_pop
  val stSsiPass2Done = Wire(Vec(lsuWidth, Bool()))
  for (i <- 0 until lsuWidth) {
    val entry = st_SSI_ADDR_Q.io.rd(i).resp.data.asTypeOf(new VecElemAccess)
    stSsiPass2Done(i) := stSsiDoPass2Prev && st_beat.io.ssi_pop(i) && entry.last
  }
  vec.st_drain_done.valid := stUsPass2Done || stSsiPass2Done.reduce(_ || _)
  vec.st_drain_done.bits  := Mux(stUsPass2Done, stUsStagedBits.stq_idx,
    PriorityMux(stSsiPass2Done, (0 until lsuWidth).map(i => st_SSI_ADDR_Q.io.rd(i).resp.data.asTypeOf(new VecElemAccess).uop.stq_idx)))

  // ---- exception, from the translate responses ----
  class XcptShadow extends BoomBundle {
    val valid = Bool()
    val uop   = new MicroOp
    val vaddr = UInt(coreMaxAddrBits.W)
  }
  val xcptShadow = RegInit(VecInit(Seq.fill(lsuWidth)(0.U.asTypeOf(new XcptShadow))))
  for (w <- 0 until lsuWidth) {
    xcptShadow(w).valid := arb.io.vec_fire(w).valid && arb.io.vec_fire(w).bits.uses_tlb
    xcptShadow(w).uop   := arb.io.vec_fire(w).bits.uop
    xcptShadow(w).vaddr := arb.io.vec_fire(w).bits.vaddr
  }
  val xcptHit = (0 until lsuWidth).map(w => xcptShadow(w).valid && vec.xlate_resp(w).valid && vec.xlate_resp(w).bits.xcpt_valid)

  // The fault report is formed HERE, from the exception shadow above, restricted
  // to the load side and the unit-stride path and qualified against the walking
  // rob_idx so a fault for a squashed op cannot classify against a live one.
  val ldRangeFaultCand = (0 until lsuWidth).map { w =>
    xcptHit(w) && xcptShadow(w).uop.uses_ldq && xcptShadow(w).uop.v_is_unit_stride.get
  }
  val ldRangeFaultMatch = (0 until lsuWidth).map { w =>
    ldRangeFaultCand(w) && ldUsStagedValid && (xcptShadow(w).uop.rob_idx === ldUsStagedBits.rob_idx)
  }
  val ldRangeFaultDrop  = (0 until lsuWidth).map { w => ldRangeFaultCand(w) && !ldRangeFaultMatch(w) }
  val ldRangeFaultAny   = ldRangeFaultMatch.reduce(_ || _)
  val ldRangeFaultVaddr = PriorityMux(ldRangeFaultMatch, xcptShadow.map(_.vaddr))
  // Both addresses are virtual on the load side (ld_US_ADDR_Q has no translate
  // pass), so the subtraction stays exact across a page boundary.
  val ldRangeFaultElemIdx = ((ldRangeFaultVaddr - ldUsStagedBits.base) >> ldUsStagedBits.eew)(vecVLSz - 1, 0)

  ld_range_agen.io.fault.foreach { f =>
    f.valid         := ldRangeFaultAny
    f.bits.elem_idx  := ldRangeFaultElemIdx
    f.bits.is_ff     := ldUsStagedBits.is_ff
    f.bits.rob_idx   := ldUsStagedBits.rob_idx
    f.bits.ldq_idx   := ldUsStagedBits.ldq_idx
  }

  // stop is a level while the fault hit above is a one-cycle pulse. One entry
  // is sufficient: io.stop gates usGate alone, which serves the single retained
  // unit-stride head.
  class LdRangeFaulted extends BoomBundle {
    val valid   = Bool()
    val rob_idx = UInt(robAddrSz.W)
  }
  val ldRangeFaulted = RegInit(0.U.asTypeOf(new LdRangeFaulted))
  // Retire outranks set: a trim raises ff_trim in the SAME cycle as the fault, so
  // an elsewhen'd clear would leave the latch holding a retired op's rob_idx.
  when (ldUsRetire || squash.io.kill(2)) {
    ldRangeFaulted.valid := false.B
  } .elsewhen (ldRangeFaultAny) {
    ldRangeFaulted.valid   := true.B
    ldRangeFaulted.rob_idx := ldUsStagedBits.rob_idx
  }
  // Combinational on the hit as well as on the latch: the latch asserts a cycle
  // later, and that cycle is one more beat past the fault.
  ldBeatStop := ldRangeFaultAny ||
    (ldRangeFaulted.valid && (ldRangeFaulted.rob_idx === ldUsStagedBits.rob_idx))
  ldUsTrimRetire := ld_range_agen.io.ff_trim.get.valid

  when (ldRangeFaultAny) {
    VecTrace.traceId("VecLsu", "vleff_fault_report", ldUsStagedBits.rob_idx,
      Seq(("ldq_idx", ldUsStagedBits.ldq_idx), ("elem_idx", ldRangeFaultElemIdx),
          ("is_ff", ldUsStagedBits.is_ff.asUInt), ("vaddr", ldRangeFaultVaddr),
          ("base", ldUsStagedBits.base), ("eew", ldUsStagedBits.eew)))
  }
  for (w <- 0 until lsuWidth) {
    when (ldRangeFaultDrop(w)) {
      VecTrace.traceId("VecLsu", "vleff_fault_dropped", xcptShadow(w).uop.rob_idx)
    }
  }
  when (ld_range_agen.io.fault_trap.get) {
    VecTrace.traceId("VecLsu", "vleff_fault_trap", ldUsStagedBits.rob_idx)
  }
  when (ld_range_agen.io.ff_trim.get.valid) {
    VecTrace.traceId("VecLsu", "vleff_trim", ldUsStagedBits.rob_idx,
      Seq(("elem_idx", ld_range_agen.io.ff_trim.get.bits)))
  }

  // ldUsStagedBits is raw read data and is meaningful ONLY under ldUsStagedValid;
  // unqualified, this fires on an empty queue where the fields are undefined.
  assert(!(ldUsStagedValid && ldUsStagedBits.is_ff) || ldUsStagedBits.is_unit_stride,
    "VecLsu: is_ff asserted on a retained range entry without is_unit_stride")
  assert(!(ld_range_agen.io.fault_trap.get && ld_range_agen.io.ff_trim.get.valid),
    "VecLsu: ld_range_agen fault_trap and ff_trim valid in the same cycle")
  assert(!ldRangeFaultAny || (ldRangeFaultElemIdx < (ldUsStagedBits.len >> ldUsStagedBits.eew)),
    "VecLsu: vleff elem_idx not below the retained range entry's element count")
  assert(!ld_range_agen.io.ff_trim.get.valid || ldBeatStop,
    "VecLsu: ff_trim valid without ld_beat.io.stop in the same cycle")

  // vec_xcpt.valid is QUALIFIED by fault_trap on the unit-stride load path, not
  // merely accompanied by it -- an element-i>0 vleff fault must trim, not trap.
  // Every other path keeps the generic derivation off xlate_resp unchanged.
  val xcptHitQualified = (0 until lsuWidth).map { w =>
    xcptHit(w) && (!ldRangeFaultMatch(w) || ld_range_agen.io.fault_trap.get)
  }
  val xcptAny = xcptHitQualified.reduce(_ || _)
  val xcptWinUop    = PriorityMux(xcptHitQualified, xcptShadow.map(_.uop))
  val xcptWinVaddr  = PriorityMux(xcptHitQualified, xcptShadow.map(_.vaddr))
  val xcptWinCause  = PriorityMux(xcptHitQualified, vec.xlate_resp.map(_.bits.xcpt_cause))
  io.vec_xcpt.valid         := xcptAny
  io.vec_xcpt.bits.uop      := xcptWinUop
  io.vec_xcpt.bits.cause    := xcptWinCause
  io.vec_xcpt.bits.badvaddr := xcptWinVaddr

  // =========================================================================
  // ---- 11. vec_lsu_empty ----
  // =========================================================================

  val vecLsuEmpty = queueSeq.map(_.io.empty).reduce(_ && _) && lcb.io.empty
  io.lsu_fencei_rdy_vec := vecLsuEmpty

  //@formal-anchor VecLsuChecks
  layer.block(BoomSvaLayer) {
    VecLsuChecks(
      stUsWritePass     = stUsWritePass,
      stSsiPass2Prev    = stSsiDoPass2Prev,
      stUsCursorWrValid = st_beat.io.us_cursor_wr.valid,
      stUsCursorWrBits  = st_beat.io.us_cursor_wr.bits,
      stUsTotalElems    = stUsTotalElems,
      stUsUpdateValid   = st_US_ADDR_Q.io.update.get(0).valid,
      stBeatReqValid    = st_beat.io.req(0).valid,
      stBeatUsHead      = st_beat.io.us_head.valid,
      stBeatSsiHead     = st_beat.io.ssi_head(0).valid,
      stBeatWritePass   = st_beat.io.is_write_pass.get,
      stUsDataValid     = st_beat.io.st_us_data.get.valid,
      stUsDataPop       = st_beat.io.st_us_data_pop.get,
      stSsiDataValid    = st_beat.io.st_ssi_data.get(0).valid,
      dgenReqValid      = dgen.io.req.valid,
      dgenReqRobIdx     = dgen.io.req.bits.uop.rob_idx,
      issStRobIdx       = io.iss_st.bits.rob_idx,
      issStAgen         = issStAgen,
      stRowTargetValid  = stRows(GetRealLSQIdx(io.iss_st.bits.stq_idx)).valid,
      gcopyLaunchValid  = gcopy.io.launch.valid,
      ldLcbTrigger      = ldLcbTrigger
    )
  }
}

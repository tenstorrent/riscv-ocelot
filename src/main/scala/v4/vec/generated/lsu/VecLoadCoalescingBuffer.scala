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

import boom.v4.common.{BoomModule, BoomBundle}
import boom.v4.lsu.GetRealLSQIdx
import boom.v4.util.SelectFirstN
import boom.v4.vec.generated.{VecGroupDone, VecTrace}
import boom.v4.vec.formal.{BoomSvaLayer, VecLoadCoalescingBufferChecks}

// GENERATED from src/main/nlhdl/vec/lsu/VecLoadCoalescingBuffer.nlhdl.scala. Do
// not hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class LcbAllocReq(implicit p: Parameters) extends BoomBundle
{
  val prn            = UInt(vecPregSz.W)
  val ldq_idx        = UInt((1 + ldqAddrSz).W)
  val rob_idx        = UInt(robAddrSz.W)
  val member_idx     = UInt(log2Ceil(maxVecMembers).W)
  val members_target = UInt(log2Ceil(maxVecMembers + 1).W)
  val active_bytes   = UInt((vecVLen / 8).W)
  val inactive_bytes = UInt((vecVLen / 8).W)
  val undisturbed    = Bool()
  val stale_prn      = UInt(vecPregSz.W)
  val is_ff          = Bool()
  val pvl            = UInt(vlPregSz.W)
  val vl_final       = UInt(vecVLSz.W)
}

class LcbBeat(implicit p: Parameters) extends BoomBundle
{
  val data     = UInt(coreDataBits.W)
  val prn      = UInt(vecPregSz.W)
  val ldq_idx  = UInt((1 + ldqAddrSz).W)
  val dst_byte = UInt(log2Ceil(vecVLen / 8 + 1).W)
  val src_off  = UInt(log2Ceil(coreDataBytes).W)
  val nbytes   = UInt(log2Ceil(coreDataBytes + 1).W)
  val nelem    = UInt(log2Ceil(coreDataBytes + 1).W)
}

class LcbTrim(implicit p: Parameters) extends BoomBundle
{
  val ldq_idx    = UInt((1 + ldqAddrSz).W)
  val vl_final   = UInt(vecVLSz.W)
  val member     = UInt(log2Ceil(maxVecMembers).W)
  val keep_bytes = UInt((vecVLen / 8).W)
}

class LcbEntry(implicit p: Parameters) extends BoomBundle
{
  val valid           = Bool()
  val prn             = UInt(vecPregSz.W)
  val ldq_idx         = UInt((1 + ldqAddrSz).W)
  val rob_idx         = UInt(robAddrSz.W)
  val member_idx      = UInt(log2Ceil(maxVecMembers).W)
  val members_target  = UInt(log2Ceil(maxVecMembers + 1).W)
  val data            = UInt(vecVLen.W)
  val byte_valid      = UInt((vecVLen / 8).W)
  val active_bytes    = UInt((vecVLen / 8).W)
  val own_bytes       = UInt((vecVLen / 8).W)
  val undisturbed     = Bool()
  val stale_prn       = UInt(vecPregSz.W)
  val preload_pending = Bool()
  val preload_done    = Bool()
  val written         = Bool()
  val is_ff           = Bool()
  val pvl             = UInt(vlPregSz.W)
  val vl_final        = UInt(vecVLSz.W)
}

class VecLoadCoalescingBuffer(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecLoadCoalescingBuffer: elaborates only under usingRVV")
  require(lcbEntries >= maxVecMembers,
    s"VecLoadCoalescingBuffer: lcbEntries ($lcbEntries) must be >= maxVecMembers ($maxVecMembers): " +
    "entries are retained until their whole group retires, so a smaller array could never hold " +
    "every member of one destination group and that load could never complete")

  val vLenBytes = vecVLen / 8

  private def extractBytes(field: UInt, byteOff: UInt, outBytes: Int): UInt =
    (field >> (byteOff << 3.U))(outBytes * 8 - 1, 0)

  private def placeBytes(field: UInt, byteOff: UInt, outBytes: Int): UInt =
    (field << (byteOff << 3.U))(outBytes * 8 - 1, 0)

  val io = IO(new Bundle {
    val alloc      = Flipped(Decoupled(new LcbAllocReq))
    val free_count = Output(UInt(log2Ceil(lcbEntries + 1).W))
    val resp       = Vec(lsuWidth, Flipped(Valid(new LcbBeat)))
    val vrf_write  = Vec(lsuWidth, Valid(new Bundle {
      val addr = UInt(vecPregSz.W)
      val data = UInt(vecVLen.W)
      val mask = UInt(vLenBytes.W)
    }))
    val stale_req  = Output(Valid(UInt(vecPregSz.W)))
    val stale_resp = Input(UInt(vecVLen.W))
    val group_done = Output(Valid(new VecGroupDone))
    // The same completion, keyed by LDQ index: the ROB needs rob_idx, but the LSU
    // marks the load executed by ldq_idx, and neither can be derived from the other.
    val group_done_ldq = Output(Valid(UInt((1 + ldqAddrSz).W)))
    val vl_wb      = Output(Valid(new Bundle {
      val pvl = UInt(vlPregSz.W)
      val vl  = UInt(vecVLSz.W)
    }))
    val elem_done  = Vec(lsuWidth, Valid(new Bundle {
      val ldq_idx = UInt((1 + ldqAddrSz).W)
      val nelem   = UInt(log2Ceil(coreDataBytes + 1).W)
    }))
    val trim     = Flipped(Valid(new LcbTrim))
    val kill_ldq = Input(UInt(numLdqEntries.W))
    val empty    = Output(Bool())
  })

  //@req-spec-lsu.e1
  //@req-spec-lsu.e2
  //@req-spec-lsu.e3
  //@req-spec-lsu.e20
  val entries = RegInit(VecInit(Seq.fill(lcbEntries)(0.U.asTypeOf(new LcbEntry))))

  val respRegValid = RegInit(VecInit(Seq.fill(lsuWidth)(false.B)))
  val respRegBits  = Reg(Vec(lsuWidth, new LcbBeat))
  for (w <- 0 until lsuWidth) {
    respRegValid(w) := io.resp(w).valid
    respRegBits(w)  := io.resp(w).bits
  }

  //@req-spec-lsu.f14
  //@req-spec-lsu.f15
  val killHere = VecInit((0 until lcbEntries).map(i =>
    entries(i).valid && io.kill_ldq(GetRealLSQIdx(entries(i).ldq_idx))))

  //@req-spec-lsu.e4
  //@req-spec-lsu.e5
  val hitMatrix = Seq.tabulate(lsuWidth, lcbEntries) { (w, i) =>
    respRegValid(w) && entries(i).valid && !killHere(i) &&
      (respRegBits(w).prn === entries(i).prn) && (respRegBits(w).ldq_idx === entries(i).ldq_idx)
  }
  val portMatched  = (0 until lsuWidth).map(w => VecInit(hitMatrix(w)).asUInt.orR)
  val portMatchIdx = (0 until lsuWidth).map(w => OHToUInt(VecInit(hitMatrix(w)).asUInt))

  //@req-spec-lsu.e24
  val freeVec  = VecInit(entries.map(!_.valid)).asUInt
  val allocSel = SelectFirstN(freeVec, 1)(0)
  val allocIdx = OHToUInt(allocSel)
  io.alloc.ready := freeVec.orR
  //@req-spec-lsu.e25
  io.free_count := PopCount(freeVec)
  val isAllocTarget = VecInit((0 until lcbEntries).map(i => io.alloc.fire && (allocIdx === i.U)))

  //@req-spec-lsu.e11
  //@req-spec-lsu.e12
  val allocPreloadPending = io.alloc.bits.undisturbed && io.alloc.bits.inactive_bytes.orR
  //@req-spec-lsu.e8
  val allocAgnosticOnes = Mux(io.alloc.bits.undisturbed, 0.U(vLenBytes.W), io.alloc.bits.inactive_bytes)
  val allocData = FillInterleaved(8, allocAgnosticOnes)

  //@req-spec-lsu.e9
  //@req-spec-lsu.e10
  //@req-spec-vrf.g4
  //@req-spec-vrf.i5
  //@req-spec-vrf.i6
  //@req-spec-vrf.i7
  val preloadCandidates = VecInit((0 until lcbEntries).map(i =>
    entries(i).valid && !killHere(i) && entries(i).preload_pending)).asUInt
  val preloadSel = PriorityEncoderOH(preloadCandidates)
  val preloadIdx = OHToUInt(preloadSel)
  io.stale_req.valid := preloadCandidates.orR
  io.stale_req.bits  := entries(preloadIdx).stale_prn

  val preloadReqValid = RegInit(false.B)
  val preloadReqIdx   = Reg(UInt(log2Ceil(lcbEntries).W))
  preloadReqValid := io.stale_req.valid
  preloadReqIdx   := preloadIdx
  val preloadHit = VecInit((0 until lcbEntries).map(i => preloadReqValid && (preloadReqIdx === i.U)))

  //@req-spec-lsu.e6
  val activeCovered = VecInit(entries.map(e => (e.byte_valid & e.active_bytes) === e.active_bytes))
  val completeThisCycle = VecInit((0 until lcbEntries).map(i =>
    entries(i).valid && !entries(i).written && !killHere(i) && activeCovered(i) &&
      !entries(i).preload_pending)).asUInt
  val writeSel = SelectFirstN(completeThisCycle, lsuWidth)
  //@req-spec-lsu.e17
  //@req-spec-lsu.e18
  //@req-spec-lsu.e19
  for (w <- 0 until lsuWidth) {
    val sel = writeSel(w)
    val idx = OHToUInt(sel)
    io.vrf_write(w).valid      := sel.orR
    io.vrf_write(w).bits.addr := entries(idx).prn
    io.vrf_write(w).bits.data := entries(idx).data
    io.vrf_write(w).bits.mask := entries(idx).own_bytes
  }
  val writtenNowMask = writeSel.map(_.asUInt).reduce(_ | _)
  val writtenNow = VecInit((0 until lcbEntries).map(i => writtenNowMask(i)))

  //@req-spec-lsu.e13
  //@req-spec-lsu.e14
  //@req-spec-lsu.e15
  //@req-spec-lsu.e16
  // group_done reads entries + kill_ldq only, never io.alloc/io.resp of this cycle -- that combinational
  // path (issue.vec_wakeup -> decode -> AGEN -> nop -> group_done) is the M1 bug this module must not repeat.
  val groupCount = VecInit((0 until lcbEntries).map { i =>
    PopCount((0 until lcbEntries).map { j =>
      entries(j).valid && !killHere(j) && (entries(j).ldq_idx === entries(i).ldq_idx) &&
        (entries(j).written || writtenNow(j))
    })
  })
  val groupReady = VecInit((0 until lcbEntries).map { i =>
    entries(i).valid && !killHere(i) && (entries(i).written || writtenNow(i)) &&
      (groupCount(i) === entries(i).members_target)
  }).asUInt
  val anyGroupReady = groupReady.orR
  val winnerOH  = PriorityEncoderOH(groupReady)
  val winnerIdx = OHToUInt(winnerOH)
  val winnerLdqIdx = entries(winnerIdx).ldq_idx
  val freeThisGroup = VecInit((0 until lcbEntries).map(i =>
    anyGroupReady && entries(i).valid && (entries(i).ldq_idx === winnerLdqIdx)))

  io.group_done_ldq.valid        := anyGroupReady
  io.group_done_ldq.bits         := winnerLdqIdx
  io.group_done.valid            := anyGroupReady
  io.group_done.bits.rob_idx     := entries(winnerIdx).rob_idx
  io.group_done.bits.members     := entries(winnerIdx).members_target
  io.group_done.bits.pvl.valid   := false.B
  io.group_done.bits.pvl.bits    := 0.U
  //@req-spec-lsu.l3
  //@req-spec-lsu.l4
  //@req-spec-rob.d16
  for (m <- 0 until maxVecMembers) {
    val hit = VecInit((0 until lcbEntries).map(i =>
      entries(i).valid && (entries(i).ldq_idx === winnerLdqIdx) && (entries(i).member_idx === m.U))).asUInt
    io.group_done.bits.pvdest(m) := Mux1H(hit, entries.map(_.prn))
  }

  //@req-spec-lsu.g7
  val lastMemberTarget = entries(winnerIdx).members_target - 1.U
  val lastMemberHit = VecInit((0 until lcbEntries).map(i =>
    entries(i).valid && (entries(i).ldq_idx === winnerLdqIdx) &&
      (entries(i).member_idx === lastMemberTarget))).asUInt
  val vlWbSrc = Mux1H(lastMemberHit, entries)
  io.vl_wb.valid     := anyGroupReady && vlWbSrc.is_ff
  io.vl_wb.bits.pvl  := vlWbSrc.pvl
  io.vl_wb.bits.vl   := vlWbSrc.vl_final

  //@req-spec-lsu.j4
  for (w <- 0 until lsuWidth) {
    io.elem_done(w).valid          := portMatched(w)
    io.elem_done(w).bits.ldq_idx   := respRegBits(w).ldq_idx
    io.elem_done(w).bits.nelem     := respRegBits(w).nelem
  }

  for (i <- 0 until lcbEntries) {
    val e = entries(i)

    var mergedData      = e.data
    var mergedByteValid = e.byte_valid
    for (w <- 0 until lsuWidth) {
      val extracted    = extractBytes(respRegBits(w).data, respRegBits(w).src_off, coreDataBytes)
      val placed       = placeBytes(extracted, respRegBits(w).dst_byte, vLenBytes)
      val beatMask     = (((1.U << respRegBits(w).nbytes) - 1.U) << respRegBits(w).dst_byte)(vLenBytes - 1, 0)
      val appliedMask  = Mux(hitMatrix(w)(i), beatMask, 0.U(vLenBytes.W))
      val bitMask      = FillInterleaved(8, appliedMask)
      mergedData      = (placed & bitMask) | (mergedData & ~bitMask)
      mergedByteValid = mergedByteValid | appliedMask
    }

    //@req-spec-lsu.e9
    //@req-spec-vrf.g4
    // own_bytes is fixed at allocation and active_bytes only shrinks (part 5), so the current
    // inactive set is always own_bytes & ~active_bytes -- no separate stored field is needed.
    val inactiveBitMask = FillInterleaved(8, e.own_bytes & ~e.active_bytes)
    val dataAfterPreload = Mux(preloadHit(i), (io.stale_resp & inactiveBitMask) | (mergedData & ~inactiveBitMask), mergedData)

    val trimHit          = io.trim.valid && e.valid && (io.trim.bits.ldq_idx === e.ldq_idx)
    val isTrimMember      = trimHit && (e.member_idx === io.trim.bits.member)
    val isAboveTrimMember = trimHit && (e.member_idx > io.trim.bits.member)
    val activeBytesAfterTrim = Mux(isTrimMember, e.active_bytes & io.trim.bits.keep_bytes,
      Mux(isAboveTrimMember, 0.U(vLenBytes.W), e.active_bytes))
    val newlyInactiveBytes   = e.active_bytes & ~activeBytesAfterTrim
    val trimRaisesPreload = (isTrimMember || isAboveTrimMember) && e.undisturbed &&
      newlyInactiveBytes.orR && !e.preload_done
    val vlFinalAfterTrim = Mux(isTrimMember || isAboveTrimMember, io.trim.bits.vl_final, e.vl_final)

    val preloadPendingNext = Mux(trimRaisesPreload, true.B, Mux(preloadHit(i), false.B, e.preload_pending))
    val preloadDoneNext    = Mux(preloadHit(i), true.B, e.preload_done)

    when (killHere(i)) {
      entries(i).valid := false.B
    } .elsewhen (freeThisGroup(i)) {
      entries(i).valid := false.B
    } .elsewhen (isAllocTarget(i)) {
      entries(i).valid           := true.B
      entries(i).prn             := io.alloc.bits.prn
      entries(i).ldq_idx         := io.alloc.bits.ldq_idx
      entries(i).rob_idx         := io.alloc.bits.rob_idx
      entries(i).member_idx      := io.alloc.bits.member_idx
      entries(i).members_target  := io.alloc.bits.members_target
      entries(i).data            := allocData
      entries(i).byte_valid      := 0.U
      //@req-spec-lsu.e21
      //@req-spec-lsu.e22
      //@req-spec-lsu.e23
      entries(i).active_bytes    := io.alloc.bits.active_bytes
      //@req-spec-lsu.e8
      entries(i).own_bytes       := io.alloc.bits.active_bytes | io.alloc.bits.inactive_bytes
      entries(i).undisturbed     := io.alloc.bits.undisturbed
      entries(i).stale_prn       := io.alloc.bits.stale_prn
      entries(i).preload_pending := allocPreloadPending
      entries(i).preload_done    := false.B
      entries(i).written         := false.B
      entries(i).is_ff           := io.alloc.bits.is_ff
      entries(i).pvl             := io.alloc.bits.pvl
      entries(i).vl_final        := io.alloc.bits.vl_final
    } .otherwise {
      entries(i).data            := dataAfterPreload
      entries(i).byte_valid      := mergedByteValid
      entries(i).active_bytes    := activeBytesAfterTrim
      entries(i).vl_final        := vlFinalAfterTrim
      entries(i).preload_pending := preloadPendingNext
      entries(i).preload_done    := preloadDoneNext
      entries(i).written         := e.written || writtenNow(i)
    }
  }

  io.empty := !entries.map(_.valid).reduce(_ || _)

  when (io.alloc.fire) {
    VecTrace.traceId("VecLoadCoalescingBuffer", "alloc", io.alloc.bits.rob_idx, Seq(
      ("entry", allocIdx), ("prn", io.alloc.bits.prn), ("member_idx", io.alloc.bits.member_idx),
      ("members_target", io.alloc.bits.members_target),
      ("active_bytes", PopCount(io.alloc.bits.active_bytes)), ("undisturbed", io.alloc.bits.undisturbed.asUInt)))
  }
  when (io.stale_req.valid) {
    VecTrace.traceId("VecLoadCoalescingBuffer", "preload", entries(preloadIdx).rob_idx, Seq(
      ("entry", preloadIdx), ("stale_prn", io.stale_req.bits)))
  }
  for (w <- 0 until lsuWidth) {
    when (portMatched(w)) {
      VecTrace.traceId("VecLoadCoalescingBuffer", "place", entries(portMatchIdx(w)).rob_idx, Seq(
        ("entry", portMatchIdx(w)), ("dst_byte", respRegBits(w).dst_byte), ("nbytes", respRegBits(w).nbytes)))
    }
  }
  for (w <- 0 until lsuWidth) {
    when (io.vrf_write(w).valid) {
      VecTrace.traceStruct("VecLoadCoalescingBuffer", "prn_write", Seq(
        ("prn", io.vrf_write(w).bits.addr), ("own_bytes", PopCount(io.vrf_write(w).bits.mask))))
    }
  }
  when (io.group_done.valid) {
    VecTrace.traceId("VecLoadCoalescingBuffer", "group_done", io.group_done.bits.rob_idx,
      Seq(("members", io.group_done.bits.members)) ++
        (0 until maxVecMembers).map(m => (s"pvdest$m", io.group_done.bits.pvdest(m))))
  }
  for (i <- 0 until lcbEntries) {
    when (killHere(i)) {
      VecTrace.traceId("VecLoadCoalescingBuffer", "kill", entries(i).rob_idx, Seq(
        ("entry", i.U), ("ldq_idx", entries(i).ldq_idx)))
    }
  }

  //@formal-anchor VecLoadCoalescingBufferChecks
  layer.block(BoomSvaLayer) {
    VecLoadCoalescingBufferChecks(
      respValid            = respRegValid(0),
      respDstByte          = respRegBits(0).dst_byte,
      hitVec               = VecInit(hitMatrix(0)).asUInt,
      vLenBytesLit         = vLenBytes.U,
      e0Valid              = entries(0).valid,
      e0Prn                = entries(0).prn,
      e0LdqIdx             = entries(0).ldq_idx,
      e0ByteValid          = entries(0).byte_valid,
      e0OwnBytes           = entries(0).own_bytes,
      e0ActiveBytes        = entries(0).active_bytes,
      e1Valid              = entries(1).valid,
      e1Prn                = entries(1).prn,
      e1LdqIdx             = entries(1).ldq_idx,
      killHere0            = killHere(0),
      wrValid              = io.vrf_write(0).valid,
      wrMask               = io.vrf_write(0).bits.mask,
      wrWritten            = entries(OHToUInt(writeSel(0))).written,
      wrActiveCovered      = activeCovered(OHToUInt(writeSel(0))),
      wrPreloadPending     = entries(OHToUInt(writeSel(0))).preload_pending,
      wrActiveBytes        = entries(OHToUInt(writeSel(0))).active_bytes,
      wrKilled             = killHere(OHToUInt(writeSel(0))),
      staleReqValid        = io.stale_req.valid,
      preloadUndisturbed   = entries(preloadIdx).undisturbed,
      groupDoneValid       = io.group_done.valid,
      groupDoneMembers     = io.group_done.bits.members,
      winnerActiveCovered  = activeCovered(winnerIdx),
      groupCountWinner     = groupCount(winnerIdx),
      pvdestAtWinnerMember = io.group_done.bits.pvdest(entries(winnerIdx).member_idx),
      winnerPrn            = entries(winnerIdx).prn,
      elemDoneValid        = io.elem_done(0).valid,
      elemDoneNelem        = io.elem_done(0).bits.nelem,
      vlWbValid            = io.vl_wb.valid
    )
  }
}

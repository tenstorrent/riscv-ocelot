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

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.exu.BrUpdateInfo
import boom.v4.lsu.GetRealLSQIdx
import boom.v4.util.{IsKilledByBranch, UpdateBrMask}
import boom.v4.vec.generated.{VecSnoopHit, VecLdSearch, VecTrace}

// GENERATED from src/main/nlhdl/vec/lsu/VecStoreForward.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecStoreForwardResp(implicit p: Parameters) extends BoomBundle
{
  val ldq_idx         = UInt((1 + ldqAddrSz).W)
  val uop             = new MicroOp
  val data            = UInt(xLen.W)
  val forward_std_val = Bool()
  val forward_stq_idx = UInt((1 + stqAddrSz).W)
}

class VecStoreForwardBeat(implicit p: Parameters) extends BoomBundle
{
  val prn         = UInt(vecPregSz.W)
  val byte_offset = UInt(log2Ceil(vecVLen / 8).W)
  val data        = UInt((coreDataBytes * 8).W)
  val byte_mask   = UInt(coreDataBytes.W)
  val rob_idx     = UInt(robAddrSz.W)
  val ldq_idx     = UInt((1 + ldqAddrSz).W)
  val last        = Bool()
}

class VecStoreForwardIO(implicit p: Parameters) extends BoomBundle
{
  val ld_search  = Input(Vec(lsuWidth, Valid(new VecLdSearch)))
  val snoop_cand = Input(Vec(lsuWidth, Vec(numStqEntries, Valid(new VecSnoopHit))))

  val stq_addr_matches    = Input(Vec(lsuWidth, UInt(numStqEntries.W)))
  val stq_forward_matches = Input(Vec(lsuWidth, UInt(numStqEntries.W)))

  val st_ssi_rd = new Bundle {
    val req  = Decoupled(UInt(resvPtrSz.W))
    val resp = Input(new Bundle {
      val data   = UInt(vecELen.W)
      val filled = Bool()
    })
  }
  val st_us_rd = new Bundle {
    val req  = Decoupled(UInt(resvPtrSz.W))
    val resp = Input(new Bundle {
      val data   = UInt(vecVLen.W)
      val filled = Bool()
    })
  }

  val fwd_resp      = Output(Vec(lsuWidth, Valid(new VecStoreForwardResp)))
  val fwd_beat      = Output(Vec(lsuWidth, Valid(new VecStoreForwardBeat)))
  val replay        = Output(Vec(lsuWidth, Valid(UInt((1 + ldqAddrSz).W))))
  val known_overlap = Output(Vec(lsuWidth, Valid(new VecHoldStEvent)))

  val brupdate  = Input(new BrUpdateInfo)
  val rob_flush = Input(Bool())
}

class VecStoreForward(val enableVecStoreForward: Boolean = true)(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecStoreForward: elaborates only under usingRVV")

  val io = IO(new VecStoreForwardIO)

  val vLenBytes = vecVLen / 8

  // Reimplements ForwardingAgeLogic's own algorithm combinationally: that
  // module's internal register would add a third pipeline stage here.
  def combForwardAge(n: Int, matches: UInt, youngest: UInt): (UInt, Bool) = {
    val ageMask = VecInit((0 until n).map(i => i.U < youngest)).asUInt
    val doubled = Cat(matches & ageMask, matches)
    val hit     = doubled.orR
    val revIdx  = PriorityEncoder(Reverse(doubled))
    val hiIdx   = (2 * n - 1).U - revIdx
    val idx     = Mux(hiIdx >= n.U, hiIdx - n.U, hiIdx)
    (idx(log2Ceil(n) - 1, 0), hit)
  }

  def byteMaskAt(addr: UInt, eew: UInt): UInt = {
    val lo = addr(log2Ceil(coreDataBytes) - 1, 0)
    val base = Mux1H(Seq(
      (eew === 0.U) -> 1.U(coreDataBytes.W),
      (eew === 1.U) -> 3.U(coreDataBytes.W),
      (eew === 2.U) -> 15.U(coreDataBytes.W),
      (eew === 3.U) -> 255.U(coreDataBytes.W)))
    (base << lo)(coreDataBytes - 1, 0)
  }

  val s1_valid       = Wire(Vec(lsuWidth, Bool()))
  val s1_isVecBeat   = Wire(Vec(lsuWidth, Bool()))
  val s1_isUS        = Wire(Vec(lsuWidth, Bool()))
  val s1_attempt     = Wire(Vec(lsuWidth, Bool()))
  val s1_knownValid  = Wire(Vec(lsuWidth, Bool()))
  val s1_knownStqIdx = Wire(Vec(lsuWidth, UInt((1 + stqAddrSz).W)))
  val s1_knownIsUS   = Wire(Vec(lsuWidth, Bool()))
  val s1_rdIdx       = Wire(Vec(lsuWidth, UInt(resvPtrSz.W)))
  val s1_uop         = Wire(Vec(lsuWidth, new MicroOp))
  val s1_ldqIdx      = Wire(Vec(lsuWidth, UInt((1 + ldqAddrSz).W)))
  val s1_paddr       = Wire(Vec(lsuWidth, UInt(corePAddrBits.W)))
  val s1_ldMask      = Wire(Vec(lsuWidth, UInt(coreDataBytes.W)))
  val s1_rangeBase   = Wire(Vec(lsuWidth, UInt(corePAddrBits.W)))
  val s1_forwardStq  = Wire(Vec(lsuWidth, UInt((1 + stqAddrSz).W)))
  val s1_entryPaddr  = Wire(Vec(lsuWidth, UInt(corePAddrBits.W)))
  val s1_eew         = Wire(Vec(lsuWidth, UInt(2.W)))

  for (w <- 0 until lsuWidth) {
    val ld   = io.ld_search(w)
    val cand = io.snoop_cand(w)

    val candValidBits = VecInit(cand.map(_.valid)).asUInt
    val eligibleBits  = VecInit((0 until numStqEntries).map { i =>
      cand(i).valid && (!ld.bits.is_vec || (ld.bits.is_unit_stride && cand(i).bits.is_unit_stride))
    }).asUInt

    val addrPool = io.stq_addr_matches(w) | candValidBits
    val fwdPool  = io.stq_forward_matches(w) | eligibleBits

    val loadRealStq = GetRealLSQIdx(ld.bits.uop.stq_idx)

    //@req-spec-memord.a21
    val (youngestMatching, matchFound)      = combForwardAge(numStqEntries, addrPool, loadRealStq)
    val (youngestForwarder, forwarderFound) = combForwardAge(numStqEntries, fwdPool, loadRealStq)

    val winner     = cand(youngestMatching)
    val sameWinner = matchFound && forwarderFound && (youngestMatching === youngestForwarder)

    //@req-spec-memord.b15
    val knownValid = ld.valid && matchFound && winner.valid
    s1_knownValid(w) := knownValid
    s1_knownIsUS(w)  := winner.bits.is_unit_stride
    s1_knownStqIdx(w) := winner.bits.stq_idx

    //@req-spec-memord.a23
    //@req-spec-memord.b10
    //@req-spec-memord.b14
    val killedNow = IsKilledByBranch(io.brupdate, io.rob_flush, ld.bits.uop)
    val attemptForward = knownValid && sameWinner && enableVecStoreForward.B &&
      ld.bits.can_forward && !ld.bits.kill_forward && !killedNow

    val eew      = winner.bits.eew
    val eewBytes = 1.U << eew

    //@req-spec-memord.b21
    val ssiActiveBytes = byteMaskAt(winner.bits.paddr, eew)
    val ssiCovers       = (ld.bits.byte_mask & ssiActiveBytes) === ld.bits.byte_mask
    val ssiTooNarrow    = eewBytes < (1.U << ld.bits.uop.mem_size)

    val usByteIdx      = ld.bits.paddr - winner.bits.paddr
    val usActiveBytes  = (winner.bits.active_mask >> usByteIdx)(coreDataBytes - 1, 0)
    val usCovers       = (ld.bits.byte_mask & usActiveBytes) === ld.bits.byte_mask

    val memberOfFirst     = usByteIdx >> log2Ceil(vLenBytes)
    val lastByte           = ld.bits.paddr + (coreDataBytes - 1).U
    val memberOfLast       = (lastByte - winner.bits.paddr) >> log2Ceil(vLenBytes)
    val straddlesMember    = memberOfFirst =/= memberOfLast
    val rangeEndExclusive  = winner.bits.paddr + winner.bits.len
    val straddlesRangeEnd  = lastByte >= rangeEndExclusive

    //@req-spec-memord.b6
    //@req-spec-memord.b7
    val partialCover = Mux(winner.bits.is_unit_stride,
      !usCovers || straddlesMember || straddlesRangeEnd,
      !ssiCovers || ssiTooNarrow)

    s1_valid(w)      := ld.valid && knownValid
    s1_isVecBeat(w)  := ld.bits.is_vec
    s1_isUS(w)       := winner.bits.is_unit_stride
    s1_attempt(w)    := attemptForward && !partialCover
    s1_rdIdx(w)      := winner.bits.queue_idx
    //@req-spec-memord.c21
    //@req-spec-memord.c22
    s1_uop(w)        := UpdateBrMask(io.brupdate, ld.bits.uop)
    s1_ldqIdx(w)     := ld.bits.ldq_idx
    s1_paddr(w)      := ld.bits.paddr
    s1_ldMask(w)     := ld.bits.byte_mask
    s1_rangeBase(w)  := ld.bits.range_base
    s1_forwardStq(w) := winner.bits.stq_idx
    s1_entryPaddr(w) := winner.bits.paddr
    s1_eew(w)        := eew

    assert(!(attemptForward && !winner.bits.data_filled),
      "VecStoreForward: attempted forward from a store candidate whose data queue entry is not yet filled")
    assert(!(winner.valid && !winner.bits.is_store),
      "VecStoreForward: snoop_cand winner is not tagged as a store")
  }

  //@req-spec-memord.a24
  val wantSSI = VecInit((0 until lsuWidth).map(w => s1_attempt(w) && !s1_isUS(w))).asUInt
  val wantUS  = VecInit((0 until lsuWidth).map(w => s1_attempt(w) && s1_isUS(w))).asUInt

  val ssiGrantLane = PriorityEncoder(wantSSI)
  val usGrantLane  = PriorityEncoder(wantUS)

  io.st_ssi_rd.req.valid := wantSSI.orR
  io.st_ssi_rd.req.bits  := s1_rdIdx(ssiGrantLane)
  io.st_us_rd.req.valid  := wantUS.orR
  io.st_us_rd.req.bits   := s1_rdIdx(usGrantLane)

  val s1_portWon = Wire(Vec(lsuWidth, Bool()))
  for (w <- 0 until lsuWidth) {
    val isSSIGrantee = !s1_isUS(w) && wantSSI(w) && (w.U === ssiGrantLane) && io.st_ssi_rd.req.ready
    val isUSGrantee  = s1_isUS(w) && wantUS(w) && (w.U === usGrantLane) && io.st_us_rd.req.ready
    s1_portWon(w) := s1_attempt(w) && (isSSIGrantee || isUSGrantee)
  }

  class Stage2 extends Bundle {
    val doForward = Bool()
    val isVecBeat = Bool()
    val isUS      = Bool()
    val uop       = new MicroOp
    val ldqIdx    = UInt((1 + ldqAddrSz).W)
    val paddr     = UInt(corePAddrBits.W)
    val ldMask    = UInt(coreDataBytes.W)
    val rangeBase = UInt(corePAddrBits.W)
    val entryPaddr = UInt(corePAddrBits.W)
    val forwardStq = UInt((1 + stqAddrSz).W)
    val eew        = UInt(2.W)
  }

  val s2      = Reg(Vec(lsuWidth, new Stage2))
  val s2Valid = RegInit(VecInit(Seq.fill(lsuWidth)(false.B)))

  for (w <- 0 until lsuWidth) {
    s2Valid(w)          := s1_valid(w)
    s2(w).doForward      := s1_portWon(w)
    s2(w).isVecBeat      := s1_isVecBeat(w)
    s2(w).isUS           := s1_isUS(w)
    s2(w).uop            := s1_uop(w)
    s2(w).ldqIdx         := s1_ldqIdx(w)
    s2(w).paddr          := s1_paddr(w)
    s2(w).ldMask         := s1_ldMask(w)
    s2(w).rangeBase      := s1_rangeBase(w)
    s2(w).entryPaddr     := s1_entryPaddr(w)
    s2(w).forwardStq     := s1_forwardStq(w)
    s2(w).eew            := s1_eew(w)
  }

  //@req-spec-memord.a25
  for (w <- 0 until lsuWidth) {
    io.known_overlap(w).valid               := s1_knownValid(w)
    io.known_overlap(w).bits.stq_idx        := s1_knownStqIdx(w)
    io.known_overlap(w).bits.is_unit_stride := s1_knownIsUS(w)
  }

  for (w <- 0 until lsuWidth) {
    io.fwd_resp(w).valid := false.B
    io.fwd_resp(w).bits  := DontCare
    io.fwd_beat(w).valid := false.B
    io.fwd_beat(w).bits  := DontCare
    io.replay(w).valid   := false.B
    io.replay(w).bits    := DontCare

    val e = s2(w)
    when (s2Valid(w)) {
      when (!e.doForward) {
        //@req-spec-memord.b6
        //@req-spec-memord.b7
        io.replay(w).valid := true.B
        io.replay(w).bits  := e.ldqIdx
      } .elsewhen (e.isVecBeat) {
        //@req-spec-memord.a24
        assert(io.st_us_rd.resp.filled, "VecStoreForward: US forward read an unfilled data-queue entry")
        val byteOff     = (e.paddr - e.entryPaddr)(log2Ceil(vLenBytes) - 1, 0)
        val slice       = (io.st_us_rd.resp.data >> Cat(byteOff, 0.U(3.W)))(coreDataBytes * 8 - 1, 0)
        val loadMember  = (e.paddr - e.rangeBase) >> log2Ceil(vLenBytes)
        val loadByteOff = (e.paddr - e.rangeBase)(log2Ceil(vLenBytes) - 1, 0)

        io.fwd_beat(w).valid            := true.B
        io.fwd_beat(w).bits.prn         := e.uop.pvdest.get(loadMember)
        io.fwd_beat(w).bits.byte_offset := loadByteOff
        io.fwd_beat(w).bits.data        := slice
        io.fwd_beat(w).bits.byte_mask   := e.ldMask
        io.fwd_beat(w).bits.rob_idx     := e.uop.rob_idx
        io.fwd_beat(w).bits.ldq_idx     := e.ldqIdx
        io.fwd_beat(w).bits.last        := true.B
      } .otherwise {
        //@req-spec-memord.a21
        val loadgenData = Mux(e.isUS, {
          assert(io.st_us_rd.resp.filled, "VecStoreForward: US forward read an unfilled data-queue entry")
          val byteOff = (e.paddr - e.entryPaddr)(log2Ceil(vLenBytes) - 1, 0)
          (io.st_us_rd.resp.data >> Cat(byteOff, 0.U(3.W)))(coreDataBytes * 8 - 1, 0)
        }, {
          assert(io.st_ssi_rd.resp.filled, "VecStoreForward: SSI forward read an unfilled data-queue entry")
          val storegen = new freechips.rocketchip.rocket.StoreGen(
            e.eew, e.entryPaddr, io.st_ssi_rd.resp.data, coreDataBytes)
          storegen.data
        })
        val loadAddr = Mux(e.isUS, 0.U(corePAddrBits.W), e.paddr)

        val loadgen = new freechips.rocketchip.rocket.LoadGen(
          e.uop.mem_size, e.uop.mem_signed, loadAddr, loadgenData, false.B, coreDataBytes)

        io.fwd_resp(w).valid               := e.uop.dst_rtype === RT_FIX || e.uop.dst_rtype === RT_FLT
        io.fwd_resp(w).bits.ldq_idx         := e.ldqIdx
        io.fwd_resp(w).bits.uop             := e.uop
        io.fwd_resp(w).bits.data            := loadgen.data
        io.fwd_resp(w).bits.forward_std_val := true.B
        io.fwd_resp(w).bits.forward_stq_idx := e.forwardStq
      }
    }

    assert(!(io.fwd_resp(w).valid && io.fwd_beat(w).valid),
      "VecStoreForward: fwd_resp and fwd_beat both valid on one lane in one cycle")
    assert(!(io.replay(w).valid && (io.fwd_resp(w).valid || io.fwd_beat(w).valid)),
      "VecStoreForward: replay asserted alongside a forwarded result on the same lane")

    when (io.fwd_resp(w).valid) {
      VecTrace.trace("VecStoreForward", "forward_resp", e.uop, Seq(
        ("ldq_idx", e.ldqIdx), ("stq_idx", e.forwardStq), ("is_us", e.isUS.asUInt)))
    }
    when (io.fwd_beat(w).valid) {
      VecTrace.trace("VecStoreForward", "forward_beat", e.uop, Seq(
        ("ldq_idx", e.ldqIdx), ("prn", io.fwd_beat(w).bits.prn)))
    }
    when (io.replay(w).valid) {
      VecTrace.traceId("VecStoreForward", "replay", e.uop.rob_idx, Seq(
        ("ldq_idx", e.ldqIdx)))
    }
  }

  for (w <- 0 until lsuWidth) {
    when (io.known_overlap(w).valid) {
      VecTrace.traceId("VecStoreForward", "known_overlap", io.ld_search(w).bits.uop.rob_idx, Seq(
        ("stq_idx", io.known_overlap(w).bits.stq_idx),
        ("is_unit_stride", io.known_overlap(w).bits.is_unit_stride.asUInt)))
    }
  }
}

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

import boom.v4.common.BoomModule
import boom.v4.vec.generated.VecTrace

// GENERATED from src/main/nlhdl/vec/lsu/VecIdxGen.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
class VecIdxGen(val isStore: Boolean = false)(implicit p: Parameters) extends BoomModule
{
  //@req-spec-vrf.g2
  //@req-spec-agen.c27
  //@req-spec-agen.c29
  val vrfReadPort: Int = if (isStore) 4 else 0

  val stageDepth = 2

  require(vecELen <= xLen,
    s"VecIdxGen: eLen ($vecELen) must be <= xLen ($xLen): widening an index element " +
    "to the address width must always be a zero-extension, never a truncation")
  require(vecVLen % vecELen == 0,
    s"VecIdxGen: vLen ($vecVLen) must be a multiple of eLen ($vecELen): elements-per-member " +
    "must be a power of two for every legal index EEW")

  val mbrIdxW    = log2Ceil(maxVecMembers)
  val posBitsMax = log2Ceil(vecVLen / 8)
  val vLenBytes  = vecVLen / 8

  class StartBits extends Bundle {
    val pvs2    = Vec(maxVecMembers, UInt(vecPregSz.W))
    val idx_eew = UInt(2.W)
    val vl      = UInt(vecVLSz.W)
    val rob_idx = UInt(robAddrSz.W)
  }

  class VrfReadIO extends Bundle {
    val req = new Bundle {
      val valid = Output(Bool())
      val prn   = Output(UInt(vecPregSz.W))
      val ready = Input(Bool())
    }
    val resp = Input(Valid(UInt(vecVLen.W)))
  }

  class IdxIO extends Bundle {
    val valid      = Output(Bool())
    val offset     = Output(UInt(xLen.W))
    val elem_idx   = Output(UInt(vecVLSz.W))
    val last       = Output(Bool())
    val next_valid = Output(Bool())
    val stall      = Output(Bool())
    val taken      = Input(Bool())
  }

  val io = IO(new Bundle {
    val start    = Flipped(Decoupled(new StartBits))
    val vrf_read = new VrfReadIO
    val idx       = new IdxIO
    val kill      = Input(Bool())
  })

  val active   = RegInit(false.B)
  val pvs2Reg   = Reg(Vec(maxVecMembers, UInt(vecPregSz.W)))
  val idxEewReg = Reg(UInt(2.W))
  val vlReg     = Reg(UInt(vecVLSz.W))
  val robIdxReg = Reg(UInt(robAddrSz.W))

  val elem_ptr = RegInit(0.U(vecVLSz.W))

  val rd_mbr         = RegInit(0.U(mbrIdxW.W))
  val rd_outstanding = RegInit(false.B)
  val pendingBuf     = Reg(Bool())
  val pendingMbr     = Reg(UInt(mbrIdxW.W))

  val mbr_valid = RegInit(VecInit(Seq.fill(stageDepth)(false.B)))
  val mbr_data  = Reg(Vec(stageDepth, UInt(vecVLen.W)))
  val mbr_num   = Reg(Vec(stageDepth, UInt(mbrIdxW.W)))

  def posArm(ep: UInt, eew: Int): UInt = ep(posBitsMax - eew - 1, 0)
  def mbrArm(ep: UInt, eew: Int): UInt = ep(posBitsMax - eew + mbrIdxW - 1, posBitsMax - eew)

  def memberOf(ep: UInt, eewSel: UInt): UInt =
    MuxLookup(eewSel, mbrArm(ep, 0))((0 until 4).map(e => e.U -> mbrArm(ep, e)))

  def isLastOfMember(ep: UInt, eewSel: UInt): Bool =
    MuxLookup(eewSel, posArm(ep, 0) === (vecVLen / 8 - 1).U)(
      (0 until 4).map(e => e.U -> (posArm(ep, e) === (vecVLen / (8 << e) - 1).U)))

  //@req-spec-agen.c17
  // pad is zero-extend and never truncates, so an arm wider than xLen (only
  // reachable via an index EEW above ELEN, which is reserved) must be cut explicitly.
  private def zextToXLen(x: UInt): UInt = x.pad(xLen)(xLen - 1, 0)

  def extendIndex(memberBits: UInt, eewSel: UInt): UInt =
    MuxLookup(eewSel, zextToXLen(memberBits.asTypeOf(Vec(vecVLen / 8, UInt(8.W)))(posArm(elem_ptr, 0))))(
      (0 until 4).map { e =>
        val w = 8 << e
        val n = vecVLen / w
        e.U -> zextToXLen(memberBits.asTypeOf(Vec(n, UInt(w.W)))(posArm(elem_ptr, e)))
      })

  //@req-spec-agen.c17
  def deriveNumMembers(vl: UInt, eew: UInt): UInt = {
    val totalBytes = vl << eew
    (totalBytes + (vLenBytes - 1).U) >> log2Ceil(vLenBytes)
  }
  val numMembers = deriveNumMembers(vlReg, idxEewReg)

  val startFire  = io.start.fire
  val curMbr     = memberOf(elem_ptr, idxEewReg)
  val nextMbr    = memberOf(elem_ptr + 1.U, idxEewReg)
  val curBufHit  = Seq.tabulate(stageDepth)(b => mbr_valid(b) && mbr_num(b) === curMbr)
  val nextBufHit = Seq.tabulate(stageDepth)(b => mbr_valid(b) && mbr_num(b) === nextMbr)
  val curValid   = curBufHit.reduce(_ || _)
  val curBufIdx  = curBufHit(1)
  val curData    = Mux(curBufHit(0), mbr_data(0), mbr_data(1))

  val takingLast = active && io.idx.taken && (elem_ptr === (vlReg - 1.U))
  val freeCurBuf = active && io.idx.taken && (isLastOfMember(elem_ptr, idxEewReg) || takingLast)

  val fetchBufIdx = rd_mbr(0)
  val canFetch    = active && !rd_outstanding && (rd_mbr < numMembers) && !mbr_valid(fetchBufIdx)
  val grant       = canFetch && io.vrf_read.req.ready
  val respValid   = io.vrf_read.resp.valid

  when (io.kill) {
    active := false.B
  } .elsewhen (startFire) {
    active := io.start.bits.vl =/= 0.U
  } .elsewhen (active && takingLast) {
    active := false.B
  }

  when (startFire) {
    pvs2Reg   := io.start.bits.pvs2
    idxEewReg := io.start.bits.idx_eew
    vlReg     := io.start.bits.vl
    robIdxReg := io.start.bits.rob_idx
  }

  when (io.kill || startFire) {
    elem_ptr := 0.U
  } .elsewhen (active && io.idx.taken) {
    elem_ptr := elem_ptr + 1.U
  }

  when (io.kill || startFire) {
    rd_mbr := 0.U
  } .elsewhen (grant) {
    rd_mbr := rd_mbr + 1.U
  }

  when (io.kill) {
    rd_outstanding := false.B
  } .elsewhen (grant) {
    rd_outstanding := true.B
  } .elsewhen (rd_outstanding && respValid) {
    rd_outstanding := false.B
  }

  when (grant) {
    pendingBuf := fetchBufIdx
    pendingMbr := rd_mbr
  }

  for (b <- 0 until stageDepth) {
    when (io.kill || startFire || takingLast) {
      mbr_valid(b) := false.B
    } .elsewhen (rd_outstanding && respValid && !io.kill && (pendingBuf === b.U)) {
      mbr_valid(b) := true.B
    } .elsewhen (freeCurBuf && (curBufIdx === b.U)) {
      mbr_valid(b) := false.B
    }
  }
  when (rd_outstanding && respValid && !io.kill) {
    mbr_data(pendingBuf) := io.vrf_read.resp.bits
    mbr_num(pendingBuf)  := pendingMbr
  }

  io.start.ready := !active || takingLast

  io.vrf_read.req.valid := canFetch
  io.vrf_read.req.prn   := pvs2Reg(rd_mbr)

  io.idx.valid      := active && curValid
  //@req-spec-agen.c17
  io.idx.offset     := extendIndex(curData, idxEewReg)
  io.idx.elem_idx   := elem_ptr
  io.idx.last       := active && (elem_ptr === (vlReg - 1.U))
  io.idx.next_valid := (active && nextBufHit.reduce(_ || _)) || io.idx.last
  io.idx.stall      := !io.idx.valid || !io.idx.next_valid

  assert(!(io.idx.taken && !io.idx.valid),
    "VecIdxGen: parent asserted io.idx.taken while io.idx.valid was low")

  val stallPrev  = RegNext(active && io.idx.stall, false.B)
  val stallStart = active && io.idx.stall && !stallPrev

  when (startFire) {
    VecTrace.traceId("VecIdxGen", "start", io.start.bits.rob_idx, Seq(
      ("idx_eew", io.start.bits.idx_eew), ("vl", io.start.bits.vl),
      ("nmembers", deriveNumMembers(io.start.bits.vl, io.start.bits.idx_eew))))
  }
  when (grant) {
    VecTrace.traceId("VecIdxGen", "read", robIdxReg, Seq(
      ("member", rd_mbr), ("prn", pvs2Reg(rd_mbr)), ("port", vrfReadPort.U)))
  }
  when (rd_outstanding && respValid && !io.kill) {
    VecTrace.traceId("VecIdxGen", "stage", robIdxReg, Seq(
      ("member", pendingMbr), ("buffer", pendingBuf.asUInt)))
  }
  when (io.idx.valid && io.idx.taken) {
    VecTrace.traceId("VecIdxGen", "present", robIdxReg, Seq(
      ("elem_idx", io.idx.elem_idx), ("offset", io.idx.offset)))
  }
  when (stallStart) {
    VecTrace.traceId("VecIdxGen", "stall", robIdxReg, Seq(
      ("elem_idx", elem_ptr), ("valid_low", (!io.idx.valid).asUInt),
      ("next_valid_low", (!io.idx.next_valid).asUInt)))
  }
  when (active && takingLast) {
    VecTrace.traceId("VecIdxGen", "retire", robIdxReg, Nil)
  }
  when (startFire && (io.start.bits.vl === 0.U)) {
    VecTrace.traceId("VecIdxGen", "retire", io.start.bits.rob_idx, Nil)
  }
  when (io.kill && active) {
    VecTrace.traceId("VecIdxGen", "kill", robIdxReg, Nil)
  }
}

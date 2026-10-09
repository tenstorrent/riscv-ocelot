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
import boom.v4.util.{GetNewBrMask, IsKilledByBranch}
import boom.v4.vec.generated.VecTrace
import boom.v4.vec.formal.{BoomSvaLayer, VecDgenChecks}

// GENERATED from src/main/nlhdl/vec/lsu/VecDgen.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecDgenReqBits(implicit p: Parameters) extends BoomBundle
{
  val uop       = new MicroOp
  val vl        = UInt(vecVLSz.W)
  val data_base = UInt(resvPtrSz.W)
}

//@req-spec-agen.d11
//@req-spec-agen.d12
class VecDgenCursorBits(implicit p: Parameters) extends BoomBundle
{
  val elem_idx = UInt(vecVLSz.W)
  val seg_idx  = UInt(3.W)
  val eew      = UInt(2.W)
  val ordinal  = UInt(resvPtrSz.W)
  val last     = Bool()
}

class VecDgenSsiPayload(implicit p: Parameters) extends BoomBundle
{
  val data    = UInt(vecELen.W)
  val byte_en = UInt((vecELen / 8).W)
  val rob_idx = UInt(robAddrSz.W)
  val stq_idx = UInt((1 + stqAddrSz).W)
  val last    = Bool()
}
class VecDgenSsiEnqBits(implicit p: Parameters) extends BoomBundle
{
  val idx  = UInt(resvPtrSz.W)
  val data = new VecDgenSsiPayload
}

class VecDgenUsPayload(implicit p: Parameters) extends BoomBundle
{
  val data        = UInt(vecVLen.W)
  val valid_bytes = UInt(log2Ceil(vecVLen / 8 + 1).W)
  val rob_idx     = UInt(robAddrSz.W)
  val stq_idx     = UInt((1 + stqAddrSz).W)
  val last        = Bool()
}
class VecDgenUsEnqBits(implicit p: Parameters) extends BoomBundle
{
  val idx  = UInt(resvPtrSz.W)
  val data = new VecDgenUsPayload
}

class VecDgenIO(implicit p: Parameters) extends BoomBundle
{
  val req    = Flipped(Decoupled(new VecDgenReqBits))
  val cursor = Flipped(Decoupled(new VecDgenCursorBits))

  //@req-spec-agen.d2
  //@req-spec-vrf.g7
  //@req-spec-lsu.d7
  val vrf_r3 = new Bundle {
    val req  = Output(Valid(UInt(vecPregSz.W)))
    val gnt  = Input(Bool())
    val resp = Input(Valid(UInt(vecVLen.W)))
  }

  val ssi_data_enq = Decoupled(new VecDgenSsiEnqBits)
  val us_data_enq  = Decoupled(new VecDgenUsEnqBits)

  val brupdate  = Input(new BrUpdateInfo)
  val rob_flush = Input(Bool())
}

class VecDgen(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecDgen: elaborates only under usingRVV")

  //@req-spec-agen.d1
  val io = IO(new VecDgenIO)

  val vLenBytes  = vecVLen / 8
  val mbrIdxW    = log2Ceil(maxVecMembers)
  val byteEnBits = vecELen / 8

  val req_valid = RegInit(false.B)
  val req_uop   = Reg(new MicroOp)
  val req_vl    = Reg(UInt(vecVLSz.W))
  val req_base  = Reg(UInt(resvPtrSz.W))

  val killed = req_valid && IsKilledByBranch(io.brupdate, io.rob_flush, req_uop)

  //@req-spec-issue.d9
  val dgenOperand = Mux(req_uop.is_shared.get, req_uop.pvtmp.get, req_uop.pvs3.get)   // M1 BUG 2

  val nf  = req_uop.v_seg_nf.get + 1.U
  val eew = req_uop.v_eew.get

  //@req-spec-agen.d15
  //@req-spec-agen.b6
  // The same three-way length VecRangeAgen computes. Whole-register and mask
  // stores ignore vl, so vl*eew names a shorter range than the addresses the
  // agen already pushed, and the tail of the range keeps its old memory.
  val totalBytes   = Mux(req_uop.v_is_whole_reg.get, req_uop.v_emul.get * vLenBytes.U,
    Mux(req_uop.v_is_mask.get, (req_vl +& 7.U) >> 3,
      (req_vl * nf) << eew))
  val zeroLen      = totalBytes === 0.U
  val membersUsed  = (totalBytes +& (vLenBytes - 1).U) >> log2Ceil(vLenBytes)
  val lastMbrBytes = totalBytes - ((membersUsed - 1.U) * vLenBytes.U)

  val isUs = req_uop.v_is_unit_stride.get || req_uop.v_is_whole_reg.get || req_uop.v_is_mask.get

  io.req.ready := !req_valid
  val accept = io.req.fire

  //@req-spec-lsu.d9
  //@req-spec-lsu.d10
  when (accept) {
    req_uop  := io.req.bits.uop
    req_vl   := io.req.bits.vl
    req_base := io.req.bits.data_base
  } .otherwise {
    req_uop.br_mask := GetNewBrMask(io.brupdate, req_uop)
  }

  //@req-spec-agen.d4
  val bufTag = Reg(Vec(2, UInt(mbrIdxW.W)))
  val bufVal = RegInit(VecInit(Seq.fill(2)(false.B)))
  val bufDat = Reg(Vec(2, UInt(vecVLen.W)))

  val outReqValid = RegInit(false.B)
  val outReqSlot  = Reg(UInt(1.W))
  val outReqMbr   = Reg(UInt(mbrIdxW.W))

  val usPtr = Reg(UInt(mbrIdxW.W))

  when (accept) {
    bufVal(0)   := false.B
    bufVal(1)   := false.B
    outReqValid := false.B
    usPtr       := 0.U
  }

  val bytePos   = ((io.cursor.bits.elem_idx * nf) + io.cursor.bits.seg_idx) << io.cursor.bits.eew   // ELEMENTS -> BYTES
  val ssiMember = bytePos(log2Ceil(vLenBytes) + mbrIdxW - 1, log2Ceil(vLenBytes))
  val ssiOff    = bytePos(log2Ceil(vLenBytes) - 1, 0)

  //@req-spec-agen.d13
  //@req-spec-agen.d14
  val need       = Mux(isUs, usPtr, ssiMember)
  val haveDemand = Mux(isUs, req_valid && !killed && !zeroLen && (usPtr < membersUsed), io.cursor.valid)

  assert(!(io.cursor.valid && !isUs && ssiMember >= membersUsed),
    "VecDgen: SSI element offset resolved to a member beyond members_used")

  val hit0     = bufVal(0) && bufTag(0) === need
  val hit1     = bufVal(1) && bufTag(1) === need
  val haveData = hit0 || hit1
  val dataSel  = Mux(hit0, bufDat(0), bufDat(1))

  val nextNeed    = (need + 1.U)(mbrIdxW - 1, 0)
  val nextHit     = (bufVal(0) && bufTag(0) === nextNeed) || (bufVal(1) && bufTag(1) === nextNeed) ||
                     (outReqValid && outReqMbr === nextNeed)
  val nextInRange = nextNeed < membersUsed

  val slot0Busy    = bufVal(0) || (outReqValid && outReqSlot === 0.U)
  val slot1Busy    = bufVal(1) || (outReqValid && outReqSlot === 1.U)
  val freeSlot     = Mux(!slot0Busy, 0.U(1.W), 1.U(1.W))
  val haveFreeSlot = !slot0Busy || !slot1Busy

  val wantCur  = req_valid && !killed && haveDemand && !haveData
  val wantNext = req_valid && !killed && haveDemand && haveData && !nextHit && nextInRange

  val fetchTarget = Mux(wantCur, need, nextNeed)
  val doFetch     = (wantCur || wantNext) && !outReqValid && haveFreeSlot

  io.vrf_r3.req.valid := doFetch
  io.vrf_r3.req.bits  := dgenOperand(fetchTarget)

  when (doFetch && io.vrf_r3.gnt) {
    outReqValid := true.B
    outReqSlot  := freeSlot
    outReqMbr   := fetchTarget
  }
  when (outReqValid && io.vrf_r3.resp.valid) {
    bufDat(outReqSlot) := io.vrf_r3.resp.bits
    bufTag(outReqSlot) := outReqMbr
    bufVal(outReqSlot) := true.B
    outReqValid        := false.B
  }
  when (bufVal(0) && bufTag(0) < need) { bufVal(0) := false.B }
  when (bufVal(1) && bufTag(1) < need) { bufVal(1) := false.B }

  val byteEnMask = MuxLookup(io.cursor.bits.eew, 0.U(byteEnBits.W))(
    (0 until 4).map { e =>
      val n = if ((1 << e) < byteEnBits) (1 << e) else byteEnBits
      e.U -> ((BigInt(1) << n) - 1).U(byteEnBits.W)
    })
  val extracted = (dataSel >> (ssiOff << 3))(vecELen - 1, 0)

  //@req-spec-agen.d6
  //@req-spec-agen.d8
  val ssiCanPush = req_valid && !killed && !isUs && !zeroLen && io.cursor.valid && haveData
  val usPushCan  = req_valid && !killed && isUs && !zeroLen && haveData

  io.cursor.ready := ssiCanPush && io.ssi_data_enq.ready

  //@req-spec-agen.d5
  //@req-spec-agen.d10
  //@req-spec-agen.d7
  io.ssi_data_enq.valid          := ssiCanPush
  io.ssi_data_enq.bits.idx       := (req_base + io.cursor.bits.ordinal)(resvPtrSz - 1, 0)
  io.ssi_data_enq.bits.data.data    := extracted
  io.ssi_data_enq.bits.data.byte_en := byteEnMask
  io.ssi_data_enq.bits.data.rob_idx := req_uop.rob_idx
  io.ssi_data_enq.bits.data.stq_idx := req_uop.stq_idx
  io.ssi_data_enq.bits.data.last    := io.cursor.bits.last

  val usPushFire = usPushCan && io.us_data_enq.ready
  val isLastMbr  = usPtr === (membersUsed - 1.U)

  val validBytesW    = log2Ceil(vLenBytes + 1)
  val lastMbrBytesTrunc = lastMbrBytes(validBytesW - 1, 0)
  val mbrPushBytes   = Mux(isLastMbr, lastMbrBytesTrunc, vLenBytes.U(validBytesW.W))

  io.us_data_enq.valid          := usPushCan
  io.us_data_enq.bits.idx       := (req_base + usPtr)(resvPtrSz - 1, 0)
  io.us_data_enq.bits.data.data        := dataSel
  io.us_data_enq.bits.data.valid_bytes := mbrPushBytes
  io.us_data_enq.bits.data.rob_idx     := req_uop.rob_idx
  io.us_data_enq.bits.data.stq_idx     := req_uop.stq_idx
  io.us_data_enq.bits.data.last        := isLastMbr

  when (usPushFire) {
    usPtr := usPtr + 1.U
  }

  val cursorPushBytes = 1.U << io.cursor.bits.eew

  val bytesPushed = Reg(UInt(vecVLSz.W))
  when (accept) {
    bytesPushed := 0.U
  } .elsewhen (io.cursor.fire) {
    bytesPushed := bytesPushed + cursorPushBytes
  } .elsewhen (usPushFire) {
    bytesPushed := bytesPushed + mbrPushBytes
  }
  assert(!(io.cursor.fire && (bytesPushed +& cursorPushBytes) > totalBytes),
    "VecDgen: SSI push exceeded total_bytes -- phantom-member bug")
  assert(!(io.cursor.fire && !req_uop.v_is_masked.get && io.cursor.bits.last &&
           ((bytesPushed + cursorPushBytes) =/= totalBytes)),
    "VecDgen: unmasked stream's last push did not reach total_bytes -- phantom/short-member bug")
  assert(!(usPushFire && (bytesPushed +& mbrPushBytes) > totalBytes),
    "VecDgen: unit-stride push exceeded total_bytes -- phantom-member bug")

  val retireZero = req_valid && !killed && zeroLen
  val retireSsi  = io.cursor.fire && !isUs && io.cursor.bits.last
  val retireUs   = usPushFire && isLastMbr

  when (accept) {
    req_valid := true.B
  } .elsewhen (killed) {
    req_valid := false.B
  } .elsewhen (retireZero || retireSsi || retireUs) {
    req_valid := false.B
  }

  when (accept) {
    val accNf         = io.req.bits.uop.v_seg_nf.get + 1.U
    val accEew         = io.req.bits.uop.v_eew.get
    val accTotalBytes  = Mux(io.req.bits.uop.v_is_whole_reg.get,
      io.req.bits.uop.v_emul.get * vLenBytes.U,
      Mux(io.req.bits.uop.v_is_mask.get, (io.req.bits.vl +& 7.U) >> 3,
        (io.req.bits.vl * accNf) << accEew))
    val accMembersUsed = (accTotalBytes +& (vLenBytes - 1).U) >> log2Ceil(vLenBytes)
    VecTrace.traceId("VecDgen", "accept", io.req.bits.uop.rob_idx, Seq(
      ("is_shared", io.req.bits.uop.is_shared.get.asUInt),
      ("operand0", Mux(io.req.bits.uop.is_shared.get,
        io.req.bits.uop.pvtmp.get.head, io.req.bits.uop.pvs3.get.head)),
      ("vl", io.req.bits.vl), ("v_eew", accEew),
      ("total_bytes", accTotalBytes), ("members_used", accMembersUsed)))
  }
  when (outReqValid && io.vrf_r3.resp.valid) {
    VecTrace.traceId("VecDgen", "r3_read", req_uop.rob_idx, Seq(("member", outReqMbr)))
  }
  when (io.cursor.fire) {
    VecTrace.traceId("VecDgen", "ssi_enq", req_uop.rob_idx, Seq(
      ("ordinal", io.cursor.bits.ordinal), ("byte_en", byteEnMask),
      ("last", io.cursor.bits.last.asUInt)))
  }
  when (usPushFire) {
    VecTrace.traceId("VecDgen", "us_enq", req_uop.rob_idx, Seq(
      ("member", usPtr), ("valid_bytes", mbrPushBytes),
      ("last", isLastMbr.asUInt)))
  }
  when (killed) {
    VecTrace.traceId("VecDgen", "abandon", req_uop.rob_idx, Nil)
  }

  //@formal-anchor VecDgenChecks
  layer.block(BoomSvaLayer) {
    VecDgenChecks(
      reqResident  = req_valid,
      killed       = killed,
      r3ReqValid   = io.vrf_r3.req.valid,
      outReqValid  = outReqValid,
      r3RespValid  = io.vrf_r3.resp.valid,
      outReqMbr    = outReqMbr,
      membersUsed  = membersUsed,
      haveData     = haveData,
      ssiEnqValid  = io.ssi_data_enq.valid,
      usEnqValid   = io.us_data_enq.valid,
      usEnqBytes   = io.us_data_enq.bits.data.valid_bytes,
      vLenBytesLit = vLenBytes.U,
      isLastMbr    = isLastMbr
    )
  }
}

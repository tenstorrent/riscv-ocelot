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

import boom.v4.common.{BoomBundle, BoomModule, MicroOp}
import boom.v4.exu.BrUpdateInfo
import boom.v4.lsu.GetRealLSQIdx
import boom.v4.util.IsKilledByBranch
import boom.v4.vec.generated.{VecRangeEntry, VecTrace}
import boom.v4.vec.formal.{BoomSvaLayer, VecRangeAgenChecks}

// GENERATED from src/main/nlhdl/vec/lsu/VecRangeAgen.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecFaultReport(implicit p: Parameters) extends BoomBundle
{
  val elem_idx = UInt(vecVLSz.W)
  val is_ff    = Bool()
  val rob_idx  = UInt(robAddrSz.W)
  val ldq_idx  = UInt((1 + ldqAddrSz).W)
}

class VecStDataCheck(implicit p: Parameters) extends BoomBundle
{
  val member  = UInt(log2Ceil(maxVecMembers).W)
  val src_prn = UInt(vecPregSz.W)
  val rob_idx = UInt(robAddrSz.W)
  val stq_idx = UInt((1 + stqAddrSz).W)
  val last    = Bool()
}

class VecRangeAgenIO(val isStore: Boolean)(implicit p: Parameters) extends BoomBundle
{
  val req         = Input(Valid(new MicroOp()))
  val scalar       = Input(Valid(new VecScalarOperands()))
  val mask         = Input(new VecUsMask())

  val resv_lookup  = Output(Valid(new VecResvReq()))
  val resv_resp    = Input(Vec(2, new VecResvSlotResp()))
  val release      = Output(Valid(new VecResvUsedCount()))
  val release_ok   = Input(Bool())

  val range        = Decoupled(new VecRangeEntry())
  val st_data      = if (isStore)  Some(Decoupled(new VecStDataCheck())) else None

  val fault        = if (!isStore) Some(Input(Valid(new VecFaultReport()))) else None
  val fault_trap   = if (!isStore) Some(Output(Bool())) else None
  val ff_trim      = if (!isStore) Some(Output(Valid(UInt(vecVLSz.W)))) else None

  val brupdate     = Input(new BrUpdateInfo())
  val rob_flush    = Input(Bool())
}

class VecRangeAgen(val isStore: Boolean = false)(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecRangeAgen: elaborates only under usingRVV")

  val io = IO(new VecRangeAgenIO(isStore))

  val req    = io.req.bits
  val scalar = io.scalar.bits

  val killed = IsKilledByBranch(io.brupdate, io.rob_flush, req)

  assert(io.req.valid === io.scalar.valid,
    "VecRangeAgen: io.req and io.scalar valids disagree")
  assert(!io.req.valid || io.scalar.bits.uop.rob_idx === req.rob_idx,
    "VecRangeAgen: io.scalar operand bundle names a different instruction than io.req")
  assert(!io.req.valid || io.mask.valid,
    "VecRangeAgen: OP.v request valid before its staged mask arrived")
  assert(!io.req.valid || io.mask.rob_idx === req.rob_idx,
    "VecRangeAgen: staged mask belongs to a different rob_idx than the request")

  assert(!io.req.valid || (req.v_is_unit_stride.get && !req.v_is_segment.get),
    "VecRangeAgen: request is not a contiguous unit-stride/whole-reg/mask access")
  assert(!io.req.valid || !req.v_is_ff.get || req.v_is_unit_stride.get,
    "VecRangeAgen: is_ff asserted without is_unit_stride")

  val isWholeReg = req.v_is_whole_reg.get
  val isMaskOp   = req.v_is_mask.get

  val vLenBytes = vecVLen / 8
  val eewBytes  = 1.U << req.v_eew.get

  //@req-spec-lsu.c4
  val usBytes = scalar.vl * eewBytes
  //@req-spec-agen.b6
  val wholeRegBytes = req.v_emul.get * vLenBytes.U
  //@req-spec-agen.b6
  val maskBytes = (scalar.vl +& 7.U) >> 3
  val totalBytes = Mux(isWholeReg, wholeRegBytes, Mux(isMaskOp, maskBytes, usBytes))

  val zeroLen = totalBytes === 0.U

  val membersUsed = (totalBytes +& (vLenBytes - 1).U) >> log2Ceil(vLenBytes)

  //@req-spec-agen.b6
  //@req-spec-lsu.c3
  io.range.valid := io.req.valid && !killed && !zeroLen

  //@req-spec-agen.b7
  val stride = Mux(isWholeReg || isMaskOp, 1.U, eewBytes)

  io.range.bits.base           := scalar.base
  io.range.bits.len            := totalBytes
  io.range.bits.eew            := req.v_eew.get
  io.range.bits.stride         := stride
  io.range.bits.is_unit_stride := true.B
  io.range.bits.is_ff          := req.v_is_ff.get
  io.range.bits.nf             := req.v_seg_nf.get
  //@req-spec-agen.e13
  // The drain side scales this mask by the entry's `eew`, and both forms below
  // carry eew=0 with a byte-granular length, so a vl-shaped mask would gate BYTES
  // by ELEMENT bits and truncate the transfer. Neither form can be masked.
  io.range.bits.mask           := Mux(isWholeReg || isMaskOp,
    ((BigInt(1) << vecVLen) - 1).U(vecVLen.W), io.mask.bits)
  io.range.bits.pvdest         := req.pvdest.get
  io.range.bits.members        := req.v_emul.get
  io.range.bits.us_data_base   := (if (isStore) io.resv_resp(1).base else 0.U)
  io.range.bits.rob_idx        := req.rob_idx
  io.range.bits.ldq_idx        := req.ldq_idx
  io.range.bits.stq_idx        := req.stq_idx

  assert(!io.range.valid || io.range.ready,
    "VecRangeAgen: io.range.ready expected tied high -- the push cannot be refused")

  io.resv_lookup.valid       := io.req.valid
  io.resv_lookup.bits.is_store := isStore.B
  io.resv_lookup.bits.q_idx  := GetRealLSQIdx(if (isStore) req.stq_idx else req.ldq_idx)

  io.release.valid              := io.req.valid
  io.release.bits.is_store      := isStore.B
  io.release.bits.q_idx         := GetRealLSQIdx(if (isStore) req.stq_idx else req.ldq_idx)
  io.release.bits.used_count(0) := Mux(zeroLen, 0.U, 1.U)
  io.release.bits.used_count(1) := (if (isStore) Mux(zeroLen, 0.U, membersUsed) else 0.U)

  io.st_data.foreach { sd =>
    //@req-spec-lsu.d5
    // Index the group vector; a group's member PRNs are not contiguous, so
    // base + offset names an unrelated register.
    val srcGroup    = Mux(req.is_shared.get, req.pvtmp.get, req.pvs3.get)
    sd.valid        := io.range.valid
    sd.bits.member  := membersUsed - 1.U
    sd.bits.src_prn := srcGroup(membersUsed - 1.U)
    sd.bits.rob_idx := req.rob_idx
    sd.bits.stq_idx := req.stq_idx
    sd.bits.last    := true.B

    assert(!sd.valid || sd.ready,
      "VecRangeAgen: io.st_data.ready expected tied high by VecLsu -- it is a check, not a handshake")
  }

  io.fault.foreach { f =>
    //@req-spec-lsu.g1
    //@req-spec-lsu.g2
    //@req-spec-lsu.g3
    //@req-spec-lsu.g4
    //@req-spec-lsu.g9
    //@req-spec-lsu.g10
    val isFFFault  = f.valid && f.bits.is_ff
    val trapAtZero = f.bits.elem_idx === 0.U

    io.fault_trap.get     := f.valid && (!isFFFault || trapAtZero)
    io.ff_trim.get.valid  := isFFFault && !trapAtZero
    io.ff_trim.get.bits   := f.bits.elem_idx

    when (isFFFault) {
      VecTrace.traceId("VecRangeAgen", "ff_report", f.bits.rob_idx,
        Seq(("elem_idx", f.bits.elem_idx), ("trap", trapAtZero.asUInt)))
    }
  }

  when (io.range.valid) {
    VecTrace.trace("VecRangeAgen", "range_push", req,
      Seq(("base", scalar.base), ("len", totalBytes), ("eew", req.v_eew.get),
          ("v_emul", req.v_emul.get), ("mask_pop", PopCount(io.mask.bits))))
  }
  when (io.req.valid && !killed && zeroLen) {
    VecTrace.trace("VecRangeAgen", "zero_length", req, Seq(("vl", scalar.vl)))
  }

  //@formal-anchor VecRangeAgenChecks
  layer.block(BoomSvaLayer) {
    VecRangeAgenChecks(
      rangeValid   = io.range.valid,
      rangeLen     = io.range.bits.len,
      rangeEew     = io.range.bits.eew,
      scalarValid  = io.scalar.valid,
      scalarVl     = io.scalar.bits.vl,
      releaseValid = io.release.valid,
      isWholeReg   = isWholeReg,
      isMaskOp     = isMaskOp
    )
  }
}

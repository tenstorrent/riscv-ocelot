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
import boom.v4.vec.generated.{VecElemAccess, VecTrace}
import boom.v4.vec.formal.{BoomSvaLayer, VecElemAgenChecks}

// GENERATED from src/main/nlhdl/vec/lsu/VecElemAgen.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecElemAgen(val isStore: Boolean)(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecElemAgen: elaborates only under usingRVV")
  require(vecVLen % vecELen == 0,
    s"VecElemAgen: vLen ($vecVLen) must be a multiple of eLen ($vecELen)")

  val vLenBytes = vecVLen / 8
  val qIdxSz    = log2Ceil(ssiQueueEntries)
  val ptrW      = qIdxSz + 1
  val segIdxSz  = 3
  val mbrIdxW   = log2Ceil(maxVecMembers)
  val skipLog2Sz = log2Ceil(3 + 1)
  val lsqIdxSz  = 1 + (if (isStore) stqAddrSz else ldqAddrSz)

  class ResvSlot extends Bundle {
    val base  = UInt(ptrW.W)
    val count = UInt(ptrW.W)
  }
  class ResvLookupBits extends Bundle {
    val is_store = Bool()
    val q_idx    = UInt(lsqIdxSz.W)
  }
  class ReleaseBits extends Bundle {
    val is_store   = Bool()
    val q_idx      = UInt(lsqIdxSz.W)
    val used_count = Vec(2, UInt(ptrW.W))
  }
  class AddrEnqBits extends Bundle {
    val idx  = UInt(ptrW.W)
    val data = new VecElemAccess
  }
  class ElemPubBits extends Bundle {
    val q_idx    = UInt(ptrW.W)
    val elem_idx = UInt(vecVLSz.W)
    val seg_idx  = UInt(segIdxSz.W)
    val active   = Bool()
    val first    = Bool()
    val last     = Bool()
  }

  val io = IO(new Bundle {
    val op = Input(Valid(new VecScalarOperands))

    val msk_staged      = Input(new VecMaskCursor)
    val msk_ahead        = Input(new VecMaskCursor)
    val msk_skip_log2    = Input(UInt(skipLog2Sz.W))
    val msk_skip_valid   = Input(Bool())
    val msk_all_inactive = Input(Bool())
    val msk_step         = Output(Bool())
    val msk_skip         = Output(Bool())

    val vrf_read_req = Output(Valid(UInt(vecPregSz.W)))
    val vrf_read_gnt = Input(Bool())
    val vrf_read_rsp = Input(Valid(UInt(vecVLen.W)))

    val resv_lookup = Output(Valid(new ResvLookupBits))
    val resv_resp   = Input(Vec(2, new ResvSlot))
    val release     = Output(Valid(new ReleaseBits))
    val release_ok  = Input(Bool())

    val addr_enq = Decoupled(new AddrEnqBits)
    val elem_pub = if (isStore) Some(Output(Valid(new ElemPubBits))) else None

    val fault      = Input(Valid(UInt(vecVLSz.W)))
    val fault_trap = Output(Bool())
    val ff_trim    = Output(Valid(UInt(vecVLSz.W)))

    val brupdate  = Input(new BrUpdateInfo)
    val rob_flush = Input(Bool())
  })

  val idx = Module(new VecIdxGen(isStore))

  val w_valid    = RegInit(false.B)
  val w_uop      = Reg(new MicroOp)
  val w_base     = Reg(UInt(xLen.W))
  val w_stride   = Reg(UInt(xLen.W))
  val w_vl       = Reg(UInt(vecVLSz.W))
  val resv_base  = Reg(UInt(ptrW.W))
  val resv_count = Reg(UInt(ptrW.W))

  val elem_ptr     = RegInit(0.U(vecVLSz.W))
  val seg_ptr      = RegInit(0.U(segIdxSz.W))
  val addr_acc     = Reg(UInt(xLen.W))
  val emit_ctr     = RegInit(0.U(qIdxSz.W))
  val dst_byte_acc = RegInit(0.U(vecVLSz.W))

  val fault_seen = RegInit(false.B)
  val fault_elem = Reg(UInt(vecVLSz.W))

  val opUop = io.op.bits.uop

  //@req-spec-agen.b5
  //@req-spec-agen.c24
  //@req-spec-agen.c6
  //@req-spec-agen.c14
  val classMatch = opUop.v_is_strided.get || opUop.v_is_indexed.get || opUop.v_is_segment.get
  val dirMatch   = if (isStore) opUop.uses_stq else opUop.uses_ldq
  val accept     = io.op.valid && opUop.is_vec.get && dirMatch && classMatch

  val complementMatch = opUop.v_is_unit_stride.get || opUop.v_is_whole_reg.get || opUop.v_is_mask.get
  assert(!io.op.valid || (classMatch =/= complementMatch),
    "VecElemAgen: SSI-class and range-class selection are not mutually exclusive/jointly exhaustive")
  assert(!(accept && opUop.v_is_ff.get), "VecElemAgen: v_is_ff reached the element agen")
  assert(!(accept && w_valid), "VecElemAgen: OP.v accepted while a walk was still in flight")

  val emptyOp = accept && (io.op.bits.vl_zero || io.msk_all_inactive)

  //@req-spec-lsu.j1
  io.resv_lookup.valid        := accept
  io.resv_lookup.bits.is_store := isStore.B
  io.resv_lookup.bits.q_idx   := (if (isStore) opUop.stq_idx else opUop.ldq_idx)
  when (accept) {
    resv_base  := io.resv_resp(0).base
    resv_count := io.resv_resp(0).count
  }
  if (isStore) {
    assert(!accept || (io.resv_resp(0).base === io.resv_resp(1).base &&
                        io.resv_resp(0).count === io.resv_resp(1).count),
      "VecElemAgen: store address/data reservation slots disagree")
  }

  val relCount = Mux(opUop.v_is_segment.get, io.op.bits.vl * (opUop.v_seg_nf.get + 1.U), io.op.bits.vl)
  val relCountFinal = Mux(io.op.bits.vl_zero || io.msk_all_inactive, 0.U(ptrW.W), relCount(ptrW - 1, 0))
  assert(!accept || !(relCount >> ptrW).orR, "VecElemAgen: release count exceeds queue pointer width")

  io.release.valid             := accept
  io.release.bits.is_store     := isStore.B
  io.release.bits.q_idx        := (if (isStore) opUop.stq_idx else opUop.ldq_idx)
  io.release.bits.used_count(0) := relCountFinal
  io.release.bits.used_count(1) := relCountFinal

  when (accept) {
    w_uop    := opUop
    w_base   := io.op.bits.base
    w_stride := io.op.bits.stride
    w_vl     := io.op.bits.vl
  } .otherwise {
    w_uop.br_mask := GetNewBrMask(io.brupdate, w_uop)
  }

  val killed = w_valid && IsKilledByBranch(io.brupdate, io.rob_flush, w_uop)
  idx.io.kill := killed

  idx.io.start.valid        := accept && !emptyOp && opUop.v_is_indexed.get
  idx.io.start.bits.pvs2    := opUop.pvs2.get
  idx.io.start.bits.idx_eew := opUop.v_idx_eew.get
  idx.io.start.bits.vl      := io.op.bits.vl
  idx.io.start.bits.rob_idx := opUop.rob_idx
  assert(!(idx.io.start.valid && !idx.io.start.ready),
    "VecElemAgen: idx.io.start fired while VecIdxGen was not ready")

  io.vrf_read_req.valid     := idx.io.vrf_read.req.valid
  io.vrf_read_req.bits      := idx.io.vrf_read.req.prn
  idx.io.vrf_read.req.ready := io.vrf_read_gnt
  idx.io.vrf_read.resp      := io.vrf_read_rsp

  val isIndexed = w_uop.v_is_indexed.get
  val curEew    = w_uop.v_eew.get
  val numFields = w_uop.v_seg_nf.get + 1.U
  val isFirstFld = seg_ptr === 0.U
  val isLastFld   = seg_ptr === (numFields - 1.U)

  assert(!(w_valid && seg_ptr =/= 0.U && !io.msk_staged.active),
    "VecElemAgen: mid-segment element unexpectedly staged inactive")

  //@req-spec-agen.c20
  val maskOk      = io.msk_staged.valid && io.msk_ahead.valid
  val idxBoundary = isFirstFld || isLastFld
  val idxOk       = !isIndexed || !idxBoundary || !idx.io.idx.stall
  val gateOk      = w_valid && !fault_seen && !killed && maskOk && idxOk

  val elemActive = io.msk_staged.active

  val elemBase       = Mux(isIndexed, w_base + idx.io.idx.offset, addr_acc)
  //@req-spec-agen.c19
  //@req-spec-agen.c8
  //@req-spec-agen.c18
  val fieldAddrFull = elemBase + (seg_ptr << curEew)
  val fieldAddr     = fieldAddrFull(coreMaxAddrBits - 1, 0)

  val fieldsBytes = numFields << curEew
  val bytePos     = dst_byte_acc + (seg_ptr << curEew)
  val member      = bytePos(log2Ceil(vLenBytes) + mbrIdxW - 1, log2Ceil(vLenBytes))
  val byteOff     = bytePos(log2Ceil(vLenBytes) - 1, 0)
  val destPrn     = Mux(w_uop.v_is_segment.get, w_uop.pvtmp.get(member), w_uop.pvdest.get(member))

  val isFirstAccess = emit_ctr === 0.U
  val newElemPtrPush = elem_ptr + 1.U
  val retireAfterPush = newElemPtrPush >= w_vl
  val isLastAccess   = isLastFld && retireAfterPush

  val byteEnBits = vecELen / 8
  val byteEnMask = MuxLookup(curEew, 0.U(byteEnBits.W))(
    (0 until 4).map { e =>
      val n = if ((1 << e) < byteEnBits) (1 << e) else byteEnBits
      e.U -> ((BigInt(1) << n) - 1).U(byteEnBits.W)
    })

  //@req-spec-core.c13
  val outUop = Wire(new MicroOp)
  outUop := w_uop
  outUop.v_split_first.get        := isFirstAccess
  outUop.v_split_last.get         := isLastAccess
  outUop.v_split_idx.get          := elem_ptr
  outUop.v_split_total.get        := w_vl
  outUop.v_split_dst_prn.get      := destPrn
  outUop.v_split_dst_byte_off.get := byteOff

  //@req-spec-agen.e2
  //@req-spec-agen.e6
  //@req-spec-agen.e7
  val pushValid = gateOk && elemActive
  io.addr_enq.valid          := pushValid
  io.addr_enq.bits.idx       := (resv_base + emit_ctr)(ptrW - 1, 0)
  io.addr_enq.bits.data.uop     := outUop
  io.addr_enq.bits.data.vaddr   := fieldAddr
  io.addr_enq.bits.data.eew     := curEew
  io.addr_enq.bits.data.byte_en := byteEnMask
  io.addr_enq.bits.data.first   := isFirstAccess
  io.addr_enq.bits.data.last    := isLastAccess

  val pushFire = io.addr_enq.fire

  //@req-spec-agen.e11
  io.elem_pub.foreach { p =>
    p.valid          := pushFire
    p.bits.q_idx      := io.addr_enq.bits.idx
    p.bits.elem_idx    := elem_ptr
    p.bits.seg_idx     := seg_ptr
    p.bits.active      := true.B
    p.bits.first        := isFirstAccess
    p.bits.last          := isLastAccess
  }

  //@req-spec-agen.c7
  //@req-spec-agen.e8
  //@req-spec-agen.e9
  val suppressFire = gateOk && !elemActive

  //@req-spec-agen.c10
  val doSkip              = suppressFire && !isIndexed && io.msk_skip_valid
  val skipAdvance          = 1.U << io.msk_skip_log2
  val newElemPtrSkip        = elem_ptr + skipAdvance
  val newElemPtrStep        = elem_ptr + 1.U
  val newElemPtrSuppress    = Mux(doSkip, newElemPtrSkip, newElemPtrStep)
  val retireAfterSuppress   = newElemPtrSuppress >= w_vl

  //@req-spec-agen.c16
  io.msk_step := (pushFire && isLastFld) || (suppressFire && !doSkip)
  io.msk_skip := doSkip
  idx.io.idx.taken := isIndexed && ((pushFire && isLastFld) || suppressFire)

  when (accept) {
    w_valid := !emptyOp
  } .elsewhen (killed) {
    w_valid := false.B
  } .elsewhen (pushFire && isLastFld && retireAfterPush) {
    w_valid := false.B
  } .elsewhen (suppressFire && retireAfterSuppress) {
    w_valid := false.B
  }

  when (accept || killed) {
    elem_ptr := 0.U
  } .elsewhen (pushFire && isLastFld) {
    elem_ptr := newElemPtrPush
  } .elsewhen (suppressFire) {
    elem_ptr := newElemPtrSuppress
  }

  when (accept || killed) {
    seg_ptr := 0.U
  } .elsewhen (pushFire) {
    seg_ptr := Mux(isLastFld, 0.U, seg_ptr + 1.U)
  } .elsewhen (suppressFire) {
    seg_ptr := 0.U
  }

  when (accept) {
    addr_acc := io.op.bits.base
  } .elsewhen (killed) {
    addr_acc := 0.U
  } .elsewhen (pushFire && isLastFld && !isIndexed) {
    addr_acc := addr_acc + w_stride
  } .elsewhen (suppressFire && !isIndexed) {
    addr_acc := addr_acc + Mux(doSkip, w_stride << io.msk_skip_log2, w_stride)
  }

  when (accept || killed) {
    dst_byte_acc := 0.U
  } .elsewhen (pushFire && isLastFld) {
    dst_byte_acc := dst_byte_acc + fieldsBytes
  } .elsewhen (suppressFire) {
    dst_byte_acc := dst_byte_acc + Mux(doSkip, fieldsBytes << io.msk_skip_log2, fieldsBytes)
  }

  //@req-spec-lsu.b11
  assert(!(w_valid && emit_ctr >= resv_count), "VecElemAgen: emit_ctr reached resv_count")
  when (accept || killed) {
    emit_ctr := 0.U
  } .elsewhen (pushFire) {
    emit_ctr := emit_ctr + 1.U
  }

  //@req-spec-lsu.f8
  // (emission order follows the single sequential elem_ptr/seg_ptr cursor above;
  //  no separate mechanism needed.)

  //@req-spec-lsu.f5
  //@req-spec-lsu.f9
  //@req-spec-rob.g13
  val newFault = w_valid && io.fault.valid && !fault_seen
  assert(!(io.fault.valid && !w_valid), "VecElemAgen: fault reported while no walk was in progress")
  when (accept || killed) {
    fault_seen := false.B
  } .elsewhen (newFault) {
    fault_seen := true.B
  }
  when (newFault) {
    fault_elem := io.fault.bits
  }
  io.fault_trap := io.fault.valid

  io.ff_trim.valid := false.B
  io.ff_trim.bits  := 0.U

  val stallNow  = w_valid && !fault_seen && !killed && (!gateOk || (gateOk && elemActive && !io.addr_enq.ready))
  val stallPrev = RegNext(stallNow, false.B)
  val stallStart = stallNow && !stallPrev

  when (accept) {
    VecTrace.traceId("VecElemAgen", "accept", opUop.rob_idx, Seq(
      ("is_store", isStore.B.asUInt), ("is_indexed", opUop.v_is_indexed.get.asUInt),
      ("is_segment", opUop.v_is_segment.get.asUInt), ("vl", io.op.bits.vl),
      ("v_eew", opUop.v_eew.get), ("resv_base", io.resv_resp(0).base),
      ("resv_count", io.resv_resp(0).count)))
  }
  when (accept) {
    VecTrace.traceId("VecElemAgen", "release", opUop.rob_idx, Seq(
      ("used_count", relCountFinal), ("granted", io.release_ok.asUInt)))
  }
  when (pushFire) {
    VecTrace.traceId("VecElemAgen", "emit", w_uop.rob_idx, Seq(
      ("elem_idx", elem_ptr), ("seg_idx", seg_ptr), ("vaddr", fieldAddr),
      ("q_idx", io.addr_enq.bits.idx), ("dst_prn", destPrn), ("byte_off", byteOff)))
  }
  when (suppressFire) {
    VecTrace.traceId("VecElemAgen", "suppress", w_uop.rob_idx, Seq(
      ("elem_idx", elem_ptr)))
  }
  when (doSkip) {
    VecTrace.traceId("VecElemAgen", "skip", w_uop.rob_idx, Seq(
      ("elem_idx", elem_ptr), ("msk_skip_log2", io.msk_skip_log2)))
  }
  when (stallStart) {
    VecTrace.traceId("VecElemAgen", "stall", w_uop.rob_idx, Seq(
      ("mask_low", (!maskOk).asUInt), ("idx_low", (isIndexed && idxBoundary && idx.io.idx.stall).asUInt),
      ("ready_low", (gateOk && elemActive && !io.addr_enq.ready).asUInt)))
  }
  when (newFault) {
    VecTrace.traceId("VecElemAgen", "fault", w_uop.rob_idx, Seq(("fault_elem", io.fault.bits)))
  }
  when ((pushFire && isLastFld && retireAfterPush) || (suppressFire && retireAfterSuppress)) {
    VecTrace.traceId("VecElemAgen", "retire", w_uop.rob_idx, Seq(
      ("emitted", emit_ctr + pushFire.asUInt), ("vl", w_vl)))
  }
  when (emptyOp) {
    VecTrace.traceId("VecElemAgen", "retire", opUop.rob_idx, Seq(
      ("emitted", 0.U), ("vl", io.op.bits.vl)))
  }
  when (killed) {
    VecTrace.traceId("VecElemAgen", "kill", w_uop.rob_idx, Nil)
  }

  //@formal-anchor VecElemAgenChecks
  layer.block(BoomSvaLayer) {
    VecElemAgenChecks(
      pushValid = pushValid,
      pushFire  = io.addr_enq.fire,
      emitCtr   = emit_ctr,
      resvCount = resv_count,
      newFault  = newFault,
      faultSeen = fault_seen
    )
  }
}

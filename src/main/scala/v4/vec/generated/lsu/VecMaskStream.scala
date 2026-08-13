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

import boom.v4.common.{BoomBundle, BoomModule, MicroOp}
import boom.v4.vec.generated.VecTrace

// GENERATED from src/main/nlhdl/vec/lsu/VecMaskStream.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecMaskRdReq(implicit p: Parameters) extends BoomBundle
{
  val valid = Bool()
  val addr  = UInt(vecPregSz.W)
}

class VecMaskCursor(implicit p: Parameters) extends BoomBundle
{
  val valid  = Bool()
  val elem   = UInt(log2Ceil(vecVLen + 1).W)
  val active = Bool()
  val last   = Bool()
}

class VecUsMask(implicit p: Parameters) extends BoomBundle
{
  val valid   = Bool()
  val bits    = UInt(vecVLen.W)
  val rob_idx = UInt(robAddrSz.W)
}

class VecMaskStream(
  val isStore:     Boolean,
  val maxSkipLog2: Int = 3
)(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, s"VecMaskStream instantiated with usingRVV=false")
  require(maxSkipLog2 >= 0 && maxSkipLog2 <= log2Ceil(vecVLen),
    s"VecMaskStream: maxSkipLog2 ($maxSkipLog2) must be in [0, log2Ceil(vecVLen)=${log2Ceil(vecVLen)}]")

  val elemW   = log2Ceil(vecVLen + 1)
  val windowW = 1 << maxSkipLog2
  val skipW   = log2Ceil(maxSkipLog2 + 1)

  val io = IO(new Bundle {
    val op        = Input(Valid(new MicroOp()))
    val op_masked = Input(Bool())
    val op_vl     = Input(UInt(vecVLSz.W))
    val kill      = Input(Bool())
    val step      = Input(Bool())
    val skip      = Input(Bool())

    val mask_rd_req  = Output(new VecMaskRdReq)
    val mask_rd_data = Input(UInt(vecVLen.W))
    val owns_port    = Output(Bool())

    val staged       = Output(new VecMaskCursor)
    val ahead        = Output(new VecMaskCursor)
    val skip_log2    = Output(UInt(skipW.W))
    val skip_valid   = Output(Bool())
    val all_inactive = Output(Bool())
    val done         = Output(Bool())
    val us_mask      = Output(new VecUsMask)
  })

  def tailMask(vl: UInt): UInt = {
    val amt = vl(elemW - 1, 0)
    ((1.U((vecVLen + 1).W) << amt) - 1.U)(vecVLen - 1, 0)
  }

  //@req-spec-agen.e14
  val mask_valid    = RegInit(false.B)
  val mask_q        = RegInit(0.U(vecVLen.W))
  val elem_ptr      = RegInit(0.U(elemW.W))
  val op_vl_q       = RegInit(0.U(vecVLSz.W))
  val rob_idx_q     = RegInit(0.U(robAddrSz.W))
  val is_us_q       = RegInit(false.B)
  val req_fired_prev = RegInit(false.B)

  //@req-spec-agen.c28
  //@req-spec-agen.e4
  //@req-spec-agen.e5
  //@req-spec-vrf.g3
  //@req-spec-vrf.g8
  val mask_req = io.op.valid && io.op_masked
  io.mask_rd_req.valid := mask_req
  io.mask_rd_req.addr  := io.op.bits.pvm.get
  io.owns_port         := mask_req

  val vl_now      = Mux(io.op.valid, io.op_vl, op_vl_q)
  val rob_idx_now = Mux(io.op.valid, io.op.bits.rob_idx, rob_idx_q)
  val us_now      = Mux(io.op.valid, io.op.bits.v_is_unit_stride.get, is_us_q)

  val elided_load     = io.op.valid && !io.op_masked
  val fresh_this_cycle = elided_load || req_fired_prev

  //@req-spec-agen.e1
  val fresh_raw  = Mux(elided_load, ~(0.U(vecVLen.W)), io.mask_rd_data)
  val fresh_mask = fresh_raw & tailMask(vl_now)

  val mask_now       = Mux(fresh_this_cycle, fresh_mask, mask_q)
  val mask_valid_now = mask_valid || fresh_this_cycle

  val staged_active_now = mask_now(elem_ptr)
  val staged_last_now   = elem_ptr === (vl_now - 1.U)
  val skip_valid_now    = mask_valid_now && !staged_active_now

  //@req-spec-agen.c9
  val window       = (mask_now >> elem_ptr)(windowW - 1, 0)
  val window_clear = window === 0.U
  val first_active = PriorityEncoder(window)
  val skip_log2_now = Mux(window_clear, maxSkipLog2.U(skipW.W), Log2(first_active))

  val consume = io.step || io.skip
  val advance = Mux(io.step, 1.U, Mux(io.skip, 1.U << skip_log2_now, 0.U))
  val next_elem_wide = elem_ptr +& advance
  val next_elem_ptr = next_elem_wide(elemW - 1, 0)

  // done has three causes: unit-stride and zero-active-elements retire the
  // instant the latch loads (no walk); SSI retires via step/skip below.
  val done_unit_stride = fresh_this_cycle && us_now
  val done_empty       = fresh_this_cycle && (mask_now === 0.U)
  // PASSES vl, not lands on vl-1: a skip window can extend into the zero tail and
  // step OVER the last element, and an equality test would then never retire.
  val done_walk        = mask_valid_now && consume && (next_elem_wide >= vl_now)
  val done_now         = done_unit_stride || done_empty || done_walk
  val clear            = io.kill || done_now

  assert(!(io.step && io.skip), "VecMaskStream: step and skip asserted in the same cycle")
  assert(!(io.op.valid && (mask_valid || req_fired_prev)), "VecMaskStream: new OP.v granted while cursor still occupied")

  mask_valid     := Mux(clear, false.B, mask_valid_now)
  mask_q         := Mux(clear, 0.U, mask_now)
  elem_ptr       := Mux(clear, 0.U, Mux(consume, next_elem_ptr, elem_ptr))
  op_vl_q        := Mux(clear, 0.U, vl_now)
  rob_idx_q      := Mux(clear, 0.U, rob_idx_now)
  is_us_q        := Mux(clear, false.B, us_now)
  req_fired_prev := Mux(clear, false.B, mask_req)

  io.staged.valid  := mask_valid_now
  io.staged.elem   := elem_ptr
  io.staged.active := staged_active_now
  io.staged.last   := staged_last_now

  val ahead_elem_wide = elem_ptr +& 1.U
  val mask_ext         = Cat(0.U(1.W), mask_now)

  // ahead.valid must not go high before mask_q is loaded; once loaded the
  // whole vLen-bit mask is resident, so ahead is never gated mid-walk.
  io.ahead.valid  := mask_valid_now
  io.ahead.elem   := ahead_elem_wide(elemW - 1, 0)
  io.ahead.active := mask_ext(ahead_elem_wide)
  io.ahead.last   := ahead_elem_wide === (vl_now - 1.U)

  io.skip_log2    := skip_log2_now
  io.skip_valid   := skip_valid_now
  io.all_inactive := mask_valid_now && (mask_now === 0.U)
  io.done         := done_now

  //@req-spec-agen.e12
  io.us_mask.valid   := mask_valid_now
  io.us_mask.bits    := mask_now
  io.us_mask.rob_idx := rob_idx_now

  when (io.op.valid) {
    VecTrace.traceId("VecMaskStream", "mask_read", io.op.bits.rob_idx,
      Seq(("pvm", io.op.bits.pvm.get), ("elided", (!io.op_masked).asUInt)))
  }
  when (fresh_this_cycle) {
    VecTrace.traceId("VecMaskStream", "latch_load", rob_idx_now,
      Seq(("vl", vl_now), ("popcount", PopCount(fresh_mask))))
  }
  when (mask_valid_now && io.skip) {
    VecTrace.traceId("VecMaskStream", "skip", rob_idx_now,
      Seq(("elem_ptr", elem_ptr), ("skip_log2", skip_log2_now)))
  }
  when (done_now) {
    VecTrace.traceId("VecMaskStream", "done", rob_idx_now)
  }
}

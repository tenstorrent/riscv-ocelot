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
import boom.v4.util.{GetNewBrMask, IsKilledByBranch}
import boom.v4.vec.generated.{IntWbSnoop, VecTrace}

// GENERATED from src/main/nlhdl/vec/lsu/VecScalarOperandRead.nlhdl.scala. Do
// not hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecScalarOperands(implicit p: Parameters) extends BoomBundle
{
  val uop          = new MicroOp
  val base         = UInt(xLen.W)
  val stride       = UInt(xLen.W)
  val scalar_data  = UInt(xLen.W)
  val vl           = UInt(vecVLSz.W)
  val vl_zero      = Bool()
}

class VecScalarOperandReadIO(implicit p: Parameters) extends BoomBundle
{
  val iss        = Input(Valid(new MicroOp))
  val brupdate   = Input(new BrUpdateInfo)
  val rob_flush  = Input(Bool())

  val int_rf_read_req = Vec(2, Decoupled(UInt(ipregSz.W)))
  val int_rf_read_rsp = Input(Vec(2, UInt(xLen.W)))
  val int_wb_snoop    = Input(Vec(numIrfWritePorts, Valid(new IntWbSnoop)))

  val vl_read_addr = Output(UInt(vlPregSz.W))
  val vl_read_data = Input(UInt(vecVLSz.W))

  val out = Output(Valid(new VecScalarOperands))
}

class VecScalarOperandRead(val isStore: Boolean = false)(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecScalarOperandRead: elaborates only under usingRVV")

  val io = IO(new VecScalarOperandReadIO)

  //@req-spec-issue.h3
  val rr_valid       = RegInit(false.B)
  val rr_need        = RegInit(VecInit(Seq.fill(2)(false.B)))
  val rr_uop         = Reg(new MicroOp)
  val rr_data        = Reg(Vec(2, UInt(xLen.W)))
  val rr_vl          = Reg(UInt(vecVLSz.W))
  val rr_held_cycles = Reg(UInt(8.W))

  val killedNow = IsKilledByBranch(io.brupdate, io.rob_flush, rr_uop)

  //@req-spec-issue.d3
  val heldAddr = Seq(rr_uop.prs1, rr_uop.prs2)
  val grantAddr = Seq(io.iss.bits.prs1, io.iss.bits.prs2)
  val heldActive = VecInit((0 until 2).map(n => rr_valid && rr_need(n)))

  //@req-spec-agen.d16
  for (n <- 0 until 2) {
    io.int_rf_read_req(n).valid := heldActive(n) || io.iss.valid
    io.int_rf_read_req(n).bits  := Mux(heldActive(n), heldAddr(n), grantAddr(n))(ipregSz - 1, 0)
  }

  val fire       = VecInit((0 until 2).map(n => io.int_rf_read_req(n).fire))
  val fired_prev = RegNext(fire, VecInit(Seq.fill(2)(false.B)))

  val prevBits  = RegNext(VecInit(io.int_rf_read_req.map(_.bits)))
  val prevValid = RegNext(VecInit(io.int_rf_read_req.map(_.valid)), VecInit(Seq.fill(2)(false.B)))
  for (n <- 0 until 2) {
    assert(!(prevValid(n) && !fired_prev(n)) ||
           (io.int_rf_read_req(n).valid && io.int_rf_read_req(n).bits === prevBits(n)),
      "VecScalarOperandRead: held INT read request dropped valid or changed address before fire")
  }

  //@req-spec-issue.d3
  val hits = (0 until 2).map(n => VecInit(io.int_wb_snoop.map(w => w.valid && w.bits.addr === heldAddr(n))))
  for (n <- 0 until 2) {
    assert(!rr_valid || PopCount(hits(n)) <= 1.U,
      "VecScalarOperandRead: multiple writeback ports hit the same forward compare")
  }
  // Do not delete this substitution: BOOM's INT Mem read has no
  // read-during-write bypass, so this lane's response is stale without it.
  val fwd = (0 until 2).map(n =>
    Mux(hits(n).asUInt.orR, Mux1H(hits(n), io.int_wb_snoop.map(_.bits.data)), io.int_rf_read_rsp(n)))

  for (n <- 0 until 2) {
    when (fired_prev(n)) { rr_data(n) := fwd(n) }
  }

  val presented = (0 until 2).map(n => Mux(fired_prev(n), fwd(n), rr_data(n)))

  when (io.iss.valid) {
    rr_uop         := io.iss.bits
    rr_valid       := true.B
    rr_need(0)     := !fire(0)
    rr_need(1)     := !fire(1)
    rr_held_cycles := 0.U
  } .otherwise {
    rr_uop.br_mask := GetNewBrMask(io.brupdate, rr_uop)
    when (killedNow) {
      rr_valid   := false.B
      rr_need(0) := false.B
      rr_need(1) := false.B
    } .otherwise {
      when (fire(0)) { rr_need(0) := false.B }
      when (fire(1)) { rr_need(1) := false.B }
      when (rr_valid && (rr_need(0) || rr_need(1))) {
        rr_held_cycles := rr_held_cycles + 1.U
      }
    }
  }

  //@req-spec-issue.h3
  io.vl_read_addr := io.iss.bits.pvl.get
  when (io.iss.valid) { rr_vl := io.vl_read_data }

  io.out.bits.uop         := rr_uop
  io.out.bits.base        := presented(0)
  io.out.bits.stride      := presented(1)
  io.out.bits.scalar_data := (if (isStore) presented(1) else 0.U(xLen.W))
  io.out.bits.vl          := rr_vl
  //@req-spec-lsu.m1
  io.out.bits.vl_zero     := rr_vl === 0.U
  io.out.valid            := rr_valid && !rr_need(0) && !rr_need(1) && !killedNow

  assert(!(io.iss.valid && rr_valid && (rr_need(0) || rr_need(1))),
    "VecScalarOperandRead: grant landed on a still-unfired hold")

  assert(!io.iss.valid ||
    (io.iss.bits.is_vec.get && (if (isStore) io.iss.bits.uses_stq else io.iss.bits.uses_ldq)),
    "VecScalarOperandRead: granted uop missing is_vec or direction-appropriate uses_ldq/uses_stq")

  when (io.out.valid) {
    VecTrace.traceVl("VecScalarOperandRead", if (isStore) "st_resolve" else "ld_resolve", rr_uop, rr_vl,
      Seq(("base", presented(0)), ("stride", presented(1)), ("v_eew", rr_uop.v_eew.get),
          ("held_cycles", rr_held_cycles)))
  }
  for (n <- 0 until 2) {
    when (fired_prev(n) && hits(n).asUInt.orR) {
      VecTrace.trace("VecScalarOperandRead", "wb_forward", rr_uop,
        Seq(("lane", n.U), ("addr", heldAddr(n)), ("data", fwd(n))))
    }
  }
}

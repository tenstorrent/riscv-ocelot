// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

import boom.exu.FUConstants._
import boom.common._
import boom.exu._
import boom.util._

import chisel3.dontTouch // this is for debugging purposes

// module for the VLSIQue but with poison updates
class OviQueue(val num_entries: Int)(implicit p: Parameters)
extends BoomModule {
  val io = IO(new Bundle {
    // enqueue and dequeue interfaces
    val enq = Flipped(DecoupledIO(new EnhancedFuncUnitReq(xLen, vLen)))
    val deq = DecoupledIO(new EnhancedFuncUnitReq(xLen, vLen))
    // core signals for poison updates
    val core = new Bundle {
      val rob_pnr_idx  = Input(UInt(robAddrSz.W))
      val rob_head_idx = Input(UInt(robAddrSz.W))
      val brupdate     = Input(new BrUpdateInfo())
      val exception    = Input(Bool())
    }
  })

  // internal buffer and ptrs
  val entries = Reg(Vec(num_entries, new EnhancedFuncUnitReq(xLen, vLen)))
  val size = RegInit(0.U((log2Ceil(num_entries)+1).W))
  val enq_ptr, deq_ptr = RegInit(0.U(log2Ceil(num_entries+1).W))

  // helper def
  def wrap_inc(ptr: UInt): UInt = {
    if (num_entries != 1) {
      Mux((ptr === (num_entries-1).U), 0.U, ptr + 1.U)
    } else {
      ptr // if only one entry, just keep the ptr at the only entry 0
    }
  }

  // ready and valid signals
  io.enq.ready := (size < num_entries.U)
  io.deq.valid := (size > 0.U)

  // Keep poison-bit up-to-date
  for (idx <- 0 until num_entries) {
    entries(idx).req.uop.br_mask := GetNewBrMask(io.core.brupdate, entries(idx).req.uop)
    when (
      (io.core.exception && !IsOlder(entries(idx).req.uop.rob_idx, io.core.rob_pnr_idx, io.core.rob_head_idx)) ||
      (IsKilledByBranch(io.core.brupdate, entries(idx).req.uop))) {
      entries(idx).poison := true.B
    }
  }

  // enqueue logic
  when(io.enq.fire) {
    entries(enq_ptr) := io.enq.bits
    entries(enq_ptr).req.uop.br_mask := GetNewBrMask(io.core.brupdate, io.enq.bits.req.uop)
    entries(enq_ptr).poison := (
      (entries(enq_ptr).poison) ||
      (io.core.exception && !IsOlder(io.enq.bits.req.uop.rob_idx, io.core.rob_pnr_idx, io.core.rob_head_idx)) ||
      (IsKilledByBranch(io.core.brupdate, io.enq.bits.req.uop))
    )
    enq_ptr := wrap_inc(enq_ptr)
  }

  // dequeue logic
  io.deq.bits := entries(deq_ptr)
  io.deq.bits.req.uop.br_mask := GetNewBrMask(io.core.brupdate, entries(deq_ptr).req.uop)
  io.deq.bits.poison := (
    (entries(deq_ptr).poison) ||
    (io.core.exception && !IsOlder(entries(deq_ptr).req.uop.rob_idx, io.core.rob_pnr_idx, io.core.rob_head_idx)) ||
    (IsKilledByBranch(io.core.brupdate, entries(deq_ptr).req.uop))
  )
  when(io.deq.fire) {
    deq_ptr := wrap_inc(deq_ptr)
  }

  // size update logic
  when (io.enq.fire && !io.deq.fire) {
    size := size + 1.U
  } .elsewhen (io.deq.fire && !io.enq.fire) {
    size := size - 1.U
  }

  // debug signals
  dontTouch(io.enq)
  dontTouch(io.deq)
  dontTouch(io.core)
  dontTouch(entries)
  dontTouch(size)
  dontTouch(enq_ptr)
  dontTouch(deq_ptr)

  assert(enq_ptr < num_entries.U, "Enqueue pointer is out of bounds")
  assert(deq_ptr < num_entries.U, "Dequeue pointer is out of bounds")
  assert(size <= num_entries.U, "Size is out of bounds")
  assert(num_entries >= 1, "Number of entries must be at least 1")
}

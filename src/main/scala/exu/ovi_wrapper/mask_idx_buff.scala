// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

// Mask Index Buffer from the OVI
class MaskIdxBuff(val WIDTH: Int, val DEPTH: Int) extends Module {
  // ======== Parameters ========
  val DEPTH_LOG = log2Ceil(DEPTH)

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    val mask_idx_in  = Flipped(DecoupledIO(UInt(WIDTH.W)))
    val mask_idx_out = DecoupledIO(UInt(WIDTH.W))
    val credit = Output(Bool())
  })

  // ======== Implementation ========

  val queue = Module(new Queue(UInt(WIDTH.W), DEPTH))
  queue.io.enq    <> io.mask_idx_in
  io.mask_idx_out <> queue.io.deq

  val count = RegInit(0.U(DEPTH_LOG.W))

  when (io.mask_idx_in.fire && !io.mask_idx_out.fire) {
    count := count + 1.U
  } .elsewhen (!io.mask_idx_in.fire && io.mask_idx_out.fire) {
    count := count - 1.U
  }

  io.credit := (io.mask_idx_out.ready && io.mask_idx_out.valid)

  // ======== Debug and Assert ========

  dontTouch(count)
  assert(count <= DEPTH.U, "MaskIdxBuff: count is greater than DEPTH")

  // no "!ready" tolerance (vpu should know when to send data if credit is available)
  when (io.mask_idx_in.valid) {
    assert((io.mask_idx_in.ready), "MaskIdxBuff: mask_idx_in.ready is false when mask_idx_in.valid is true")
  }
}

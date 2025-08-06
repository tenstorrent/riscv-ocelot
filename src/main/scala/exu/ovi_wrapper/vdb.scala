// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

// Vector Data Buffer from the OVI
class VecDataBuffer(val R_WIDTH: Int, val W_WIDTH: Int, val DEPTH: Int) extends Module {
  // ======== Parameters ========
  val R_WIDTH_BYTES = R_WIDTH/8
  val R_WIDTH_SIZE  = log2Ceil(R_WIDTH/8+1)
  val W_WIDTH_BYTES = W_WIDTH/8
  val W_WIDTH_SIZE  = log2Ceil(W_WIDTH/8+1)
  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // data input from the VPU
    val data_in  = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val bits  = Input(UInt(W_WIDTH.W))
    }
    // data output to the OVI
    val data_out = new Bundle {
      val read_bytes  = Input(UInt(R_WIDTH_SIZE.W))  // OVI tells VDB how many bytes to consume
      val read_all    = Input(Bool())                // force dequeue of all bytes
      val valid_bytes = Output(UInt(R_WIDTH_SIZE.W)) // VDB tells OVI how many bytes are available
      val bits        = Output(UInt(R_WIDTH.W))      // Data from VDB to OVI
    }
    // credit signal
    val credit = Output(Bool())
  })

  // ======== Defs and Reqs ========

  require((R_WIDTH <= W_WIDTH), "R_WIDTH must be less than or equal to W_WIDTH")

  def WrapInc(ptr: UInt, MAX: Int): UInt = {
    Mux((ptr === (MAX-1).U), 0.U, (ptr + 1.U))
  }

  // ======== FIFO Buffer ========

  // actual buffer (stores W_WIDTH entries)
  val buffer = RegInit(VecInit(Seq.fill(DEPTH)(0.U(W_WIDTH.W))))

  // simple head and tail pointers
  val wr_ptr = RegInit(0.U(log2Ceil(DEPTH).W))
  val rd_ptr = RegInit(0.U(log2Ceil(DEPTH).W))
  val rd_idx = RegInit(0.U(W_WIDTH_SIZE.W))
  val count  = RegInit(0.U(log2Ceil(DEPTH+1).W))

  // ======== Status Signals ========

  val fifo_full  = (count === DEPTH.U)
  val fifo_empty = (count === 0.U)

  val enq_fire   = !(fifo_full)  && (io.data_in.valid)
  val shift_fire = !(fifo_empty) && (io.data_out.read_bytes =/= 0.U)
  val deq_fire   = !(fifo_empty) && Mux(
    (io.data_out.read_all),
    (true.B),
    ((shift_fire) && ((rd_idx + io.data_out.read_bytes) >= W_WIDTH_BYTES.U))
  )

  // ======== Write ========

  io.data_in.ready := !fifo_full

  when (enq_fire) {
    buffer(wr_ptr) := io.data_in.bits
    wr_ptr := WrapInc(wr_ptr, DEPTH)
  }

  // ======== Read ========

  val remaining_bytes = W_WIDTH_BYTES.U - rd_idx
  val available_bytes = Mux((remaining_bytes >= R_WIDTH_BYTES.U), R_WIDTH_BYTES.U, remaining_bytes)
  io.data_out.valid_bytes := Mux(fifo_empty, 0.U, available_bytes(R_WIDTH_SIZE-1, 0))
  io.data_out.bits := buffer(rd_ptr)(R_WIDTH-1, 0)

  io.credit := deq_fire

  when (shift_fire) {
    buffer(rd_ptr) := buffer(rd_ptr) >> (io.data_out.read_bytes << 3.U)
    when (!deq_fire) {
      rd_idx := rd_idx + io.data_out.read_bytes
    } .otherwise {
      rd_ptr := WrapInc(rd_ptr, DEPTH)
      rd_idx := 0.U
    }
  }

  // ======== Count ========

  when (enq_fire && !deq_fire) {
    count := count + 1.U
  }.elsewhen (!enq_fire && deq_fire) {
    count := count - 1.U
  }

  // ======== Debug ========

  when (
    (io.data_out.read_bytes =/= 0.U) &&
    (io.data_out.valid_bytes =/= 0.U) &&
    !(io.data_out.read_all)
  ) {
    assert(io.data_out.read_bytes <= io.data_out.valid_bytes, "attempting to read more than available bytes in entry")
    assert(io.data_out.read_bytes <= R_WIDTH_BYTES.U, "attempting to read more than max of R_WIDTH_BYTES")
  }

}

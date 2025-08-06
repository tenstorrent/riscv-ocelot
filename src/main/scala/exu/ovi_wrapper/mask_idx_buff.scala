// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

// Mask Index Buffer from the OVI
class MaskIdxBuff(val WIDTH: Int, val DEPTH: Int) extends Module {
  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    val mask_idx_in  = Flipped(DecoupledIO(UInt(WIDTH.W)))
    val mask_idx_out = DecoupledIO(UInt(WIDTH.W))
  })
  // ======== Implementation ========
  val queue = Module(new Queue(UInt(WIDTH.W), DEPTH))
  queue.io.enq <> io.mask_idx_in
  io.mask_idx_out <> queue.io.deq
}

// class MaskIdxBuff(val WIDTH: Int, val DEPTH: Int) extends Module {
//     // ======== Input-Output Ports ========
//     val io = IO(new Bundle {
//         val mask_idx_in  = Flipped(DecoupledIO(UInt(WIDTH.W)))
//         val mask_idx_out = DecoupledIO(UInt(WIDTH.W))
//     })

//     // ======== Implementation ========
    
//     // Custom FIFO implementation for better debugging visibility
//     val buffer = RegInit(VecInit(Seq.fill(DEPTH)(0.U(WIDTH.W))))
//     val head   = RegInit(0.U(log2Ceil(DEPTH).W))
//     val tail   = RegInit(0.U(log2Ceil(DEPTH).W))
//     val count  = RegInit(0.U(log2Ceil(DEPTH + 1).W))
    
//     // Status signals
//     val empty = count === 0.U
//     val full  = count === DEPTH.U
    
//     // Input interface
//     io.mask_idx_in.ready := !full
    
//     // Output interface  
//     io.mask_idx_out.valid := !empty
//     io.mask_idx_out.bits  := buffer(head)
    
//     // Enqueue logic
//     when (io.mask_idx_in.fire) {
//         buffer(tail) := io.mask_idx_in.bits
//         tail := Mux(tail === (DEPTH - 1).U, 0.U, tail + 1.U)
//     }
    
//     // Dequeue logic
//     when (io.mask_idx_out.fire) {
//         head := Mux(head === (DEPTH - 1).U, 0.U, head + 1.U)
//     }
    
//     // Count update
//     val enq_fire = io.mask_idx_in.fire
//     val deq_fire = io.mask_idx_out.fire
//     when (enq_fire && !deq_fire) {
//         count := count + 1.U
//     } .elsewhen (!enq_fire && deq_fire) {
//         count := count - 1.U
//     }
    
//     // Debug signals (visible in waveforms)
//     dontTouch(head)
//     dontTouch(tail) 
//     dontTouch(count)
//     dontTouch(empty)
//     dontTouch(full)
    
//     // Assertions for validation
//     assert(count <= DEPTH.U, "Buffer overflow!")
//     assert(!(empty && io.mask_idx_out.valid), "Valid when empty!")
//     assert(!(full && io.mask_idx_in.ready), "Ready when full!")
    
// }

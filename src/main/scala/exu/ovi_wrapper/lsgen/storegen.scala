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

class StorePacket(override val VLEN: Int, override val DMEM_WIDTH: Int)
extends Bundle with VecLSGenConstants {
  val addr     = UInt(64.W)
  val data     = UInt(DMEM_WIDTH.W)
  val mem_size = UInt(log2Ceil(DMEM_WIDTH/8).W) // bytes to store
  val sb_id    = UInt(5.W) // store buffer id
  val is_fake  = Bool()    // must ignore (since BOOM doesn't support masked stores)
  val misaligned = Bool()
  val last     = Bool()    // last element in sequence
}

// Store Generator for OVI
// holds all the store generators for OVI
class StoreGen(override val VLEN: Int, override val DMEM_WIDTH: Int)
extends Module with VecLSGenConstants {
  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = Flipped(DecoupledIO(new ConfigInfo(VLEN, DMEM_WIDTH)))
    // mask/index interface (for masked/indexed stores)
    val mask_idx = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val data  = Input(UInt((MASK_W+2).W))
    }
    // store data interface (from VDB)
    val vdb_data = new Bundle {
      val read_bytes  = Output(UInt(log2Ceil(DMEM_WIDTH/8).W)) // like a ready signal
      val valid_bytes = Input(UInt(log2Ceil(DMEM_WIDTH/8).W))  // like a valid signal
      val data        = Input(UInt(DMEM_WIDTH.W))
    }
    // kill signal (used to reset the FSM)
    val kill = Input(Bool())
    // store process FSM outputs (packet info)
    val store_packet = DecoupledIO(new StorePacket(VLEN, DMEM_WIDTH))
  })

  // ======== Definitions ========

  object State extends ChiselEnum {
    val IDLE, PACKING, SKIPPING, WALKING, BYPASS = Value
  }

  // state and config registers
  val state   = RegInit(State.IDLE)
  val start_q = RegInit(0.U.asTypeOf(new ConfigInfo(VLEN, DMEM_WIDTH))) // latched config info

  // pass through config info
  val config_info = Mux((state === State.IDLE), io.start.bits, start_q)

  // control signals
  val bypassable = (config_info.vl === 0.U)
  val packable   = (config_info.is_unit_stride && config_info.is_good_seg && !config_info.is_mask)
  val skipable   = (config_info.is_mask  && !config_info.is_index)
  val walkable   = (!config_info.is_mask ||  config_info.is_index)

  // ======== Store Generators ========

  // --- packer ---
  val packer = Module(new StorePacker(VLEN, DMEM_WIDTH))
  // start config
  packer.io.start.valid := (io.start.valid && packable)
  packer.io.start.bits  := config_info
  // vdb data config
  packer.io.vdb_data.valid_bytes := io.vdb_data.valid_bytes
  packer.io.vdb_data.data        := io.vdb_data.data
  // kill config
  packer.io.kill := io.kill
  // store packet config
  packer.io.store_packet.ready := io.store_packet.ready

  // --- skipper ---
  val skipper = Module(new StoreSkipper(VLEN, DMEM_WIDTH))
  // start config
  skipper.io.start.valid := (io.start.valid && skipable)
  skipper.io.start.bits  := config_info
  // mask config
  skipper.io.mask.valid     := io.mask_idx.valid
  skipper.io.mask.mask_data := io.mask_idx.data(MASK_W-1, 0) // data only
  // vdb data config
  skipper.io.vdb_data.valid_bytes := io.vdb_data.valid_bytes
  skipper.io.vdb_data.data        := io.vdb_data.data
  // kill config
  skipper.io.kill := io.kill
  // store packet config
  skipper.io.store_packet.ready := io.store_packet.ready

  // --- walker ---
  val walker = Module(new StoreWalker(VLEN, DMEM_WIDTH))
  // start config
  walker.io.start.valid := (io.start.valid && walkable)
  walker.io.start.bits  := config_info
  // index config
  walker.io.index.valid       := io.mask_idx.valid
  walker.io.index.index_value := io.mask_idx.data(MASK_W-1, 0).asSInt // idx val
  walker.io.index.mask_bit    := io.mask_idx.data(MASK_W)      // mask bit
  walker.io.index.last_index  := io.mask_idx.data(MASK_W+1)    // last bit
  // vdb data config
  walker.io.vdb_data.valid_bytes := io.vdb_data.valid_bytes
  walker.io.vdb_data.data        := io.vdb_data.data
  // kill config
  walker.io.kill := io.kill
  // store packet config
  walker.io.store_packet.ready := io.store_packet.ready

  // ======== Outputs ========

  // bypass packet: had to do this because of chisel binding conflicts
  val bypass_packet = Wire(new StorePacket(VLEN, DMEM_WIDTH))
  bypass_packet.addr       := DontCare
  bypass_packet.data       := DontCare
  bypass_packet.mem_size   := 0.U      // this doesnt make a lot of sense cuz its in log domain but its just like a "default" value
  bypass_packet.sb_id      := DontCare
  bypass_packet.is_fake    := true.B   // fake store
  bypass_packet.misaligned := DontCare
  bypass_packet.last       := true.B   // assert end

  // BYPASS case (no output but assert last)
  when (state === State.BYPASS) {
    io.start.ready         := false.B
    io.mask_idx.ready      := false.B
    io.vdb_data.read_bytes := 0.U
    io.store_packet.valid  := true.B
    io.store_packet.bits := bypass_packet

  // PACKING case (pass through packer)
  } .elsewhen (state === State.PACKING) {
    io.start.ready         := packer.io.start.ready
    io.mask_idx.ready      := false.B
    io.vdb_data.read_bytes := packer.io.vdb_data.read_bytes
    io.store_packet.valid  := packer.io.store_packet.valid
    io.store_packet.bits   := packer.io.store_packet.bits

  // SKIPPING case (pass through skipper)
  } .elsewhen (state === State.SKIPPING) {
    io.start.ready         := skipper.io.start.ready
    io.mask_idx.ready      := skipper.io.mask.ready
    io.vdb_data.read_bytes := skipper.io.vdb_data.read_bytes
    io.store_packet.valid  := skipper.io.store_packet.valid
    io.store_packet.bits   := skipper.io.store_packet.bits

  // WALKING case (pass through walker)
  } .elsewhen (state === State.WALKING) {
    io.start.ready         := walker.io.start.ready
    io.mask_idx.ready      := walker.io.index.ready
    io.vdb_data.read_bytes := walker.io.vdb_data.read_bytes
    io.store_packet.valid  := walker.io.store_packet.valid
    io.store_packet.bits   := walker.io.store_packet.bits

  // IDLE case (no output but transparent to inputs)
  } .otherwise {
    io.start.ready       := PriorityMux(Seq(
      (packable)   -> packer.io.start.ready,
      (skipable)   -> skipper.io.start.ready,
      (walkable)   -> walker.io.start.ready,
      (true.B)     -> true.B
    ))
    io.mask_idx.ready     := PriorityMux(Seq(
      (packable)   -> false.B,
      (skipable)   -> skipper.io.mask.ready,
      (walkable)   -> walker.io.index.ready,
      (true.B)     -> false.B
    ))
    io.vdb_data.read_bytes := 0.U
    io.store_packet.valid := false.B
    io.store_packet.bits  := DontCare
  }

  // ======== State Machine ========

  when (state === State.IDLE) {
    // go to respective generator and latch config
    when (io.start.fire) {
      state := PriorityMux(Seq(
        (bypassable) -> State.BYPASS,
        (packable)   -> State.PACKING,
        (skipable)   -> State.SKIPPING,
        (walkable)   -> State.WALKING,
        (true.B)     -> State.IDLE
      ))
      start_q := io.start.bits
    }
  }.otherwise {
    // end on kill
    when (io.kill) {
      state := State.IDLE
    // end on last packet
    }.elsewhen (io.store_packet.fire) {
      when (io.store_packet.bits.last) {
        state := State.IDLE
      }
    }
  }

}
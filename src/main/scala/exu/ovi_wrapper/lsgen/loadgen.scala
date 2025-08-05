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

class LoadPacket(override val VLEN: Int, override val DMEM_WIDTH: Int)
extends Bundle with VecLSGenConstants {
  val addr     = UInt(64.W)
  val v_reg    = UInt(5.W)
  val el_id    = UInt(EL_ID_W.W)
  val el_off   = UInt(6.W)
  val el_count = UInt(7.W)
  val sb_id    = UInt(5.W)
  val mask_data  = UInt(64.W)
  val mask_valid = Bool()
  val is_fake  = Bool() // not significant (all masked off or el_count is 0)
  val misaligned = Bool()
  val last     = Bool()
}

// Load Generator for OVI
// holds all the load generators for OVI
class LoadGen(override val VLEN: Int, override val DMEM_WIDTH: Int)
extends Module with VecLSGenConstants {
  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = Flipped(DecoupledIO(new ConfigInfo(VLEN, DMEM_WIDTH)))
    // mask/index interface (for masked/indexed loads)
    val mask_idx = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val data  = Input(UInt((MASK_W+2).W))
    }
    // kill signal (used to reset the FSM)
    val kill = Input(Bool())
    // load process FSM outputs (packet info)
    val load_packet = DecoupledIO(new LoadPacket(VLEN, DMEM_WIDTH))
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
  val packable   = (config_info.is_good_stride && config_info.is_good_seg)
  val skipable   = (config_info.is_mask && !config_info.is_index)
  val walkable   = !(config_info.is_mask && !config_info.is_index)

  // ======== Load Generators ========

  // --- packer ---
  val packer = Module(new LoadPacker(VLEN, DMEM_WIDTH))
  // start config
  packer.io.start.valid := (io.start.valid && packable)
  packer.io.start.bits  := config_info
  // mask config
  packer.io.mask.valid     := io.mask_idx.valid
  packer.io.mask.mask_data := io.mask_idx.data(MASK_W-1, 0) // data only
  // kill config
  packer.io.kill := io.kill
  // load packet config
  packer.io.load_packet.ready := io.load_packet.ready

  // --- skipper ---
  val skipper = Module(new LoadSkipper(VLEN, DMEM_WIDTH))
  // start config
  skipper.io.start.valid := (io.start.valid && skipable)
  skipper.io.start.bits  := config_info
  // mask config
  skipper.io.mask.valid     := io.mask_idx.valid
  skipper.io.mask.mask_data := io.mask_idx.data(MASK_W-1, 0) // data only
  // kill config
  skipper.io.kill := io.kill
  // load packet config
  skipper.io.load_packet.ready := io.load_packet.ready

  // --- walker ---
  val walker = Module(new LoadWalker(VLEN, DMEM_WIDTH))
  // start config
  walker.io.start.valid := (io.start.valid && walkable)
  walker.io.start.bits  := config_info
  // index config
  walker.io.index.valid       := io.mask_idx.valid
  walker.io.index.index_value := io.mask_idx.data(MASK_W-1, 0).asSInt // idx val
  walker.io.index.mask_bit    := io.mask_idx.data(MASK_W)             // mask bit
  walker.io.index.last_index  := io.mask_idx.data(MASK_W+1)           // last bit
  // kill config
  walker.io.kill := io.kill
  // load packet config
  walker.io.load_packet.ready := io.load_packet.ready

  // ======== Outputs ========

  // bypass packet: had to do this because of chisel binding conflicts
  val bypass_packet = Wire(new LoadPacket(VLEN, DMEM_WIDTH))
  bypass_packet.addr       := DontCare
  bypass_packet.v_reg      := DontCare
  bypass_packet.el_id      := DontCare
  bypass_packet.el_off     := DontCare
  bypass_packet.el_count   := config_info.vl // should be 0 if bypassable
  bypass_packet.sb_id      := DontCare
  bypass_packet.mask_data  := DontCare
  bypass_packet.mask_valid := DontCare
  bypass_packet.is_fake    := true.B         // fake load
  bypass_packet.misaligned := DontCare
  bypass_packet.last       := true.B         // assert end

  // BYPASS case (no output but assert last and el_count = 0)
  when (state === State.BYPASS) {
    io.start.ready       := false.B
    io.mask_idx.ready    := false.B
    io.load_packet.valid := true.B
    io.load_packet.bits  := bypass_packet

  // PACKING case (pass through packer)
  } .elsewhen (state === State.PACKING) {
    io.start.ready       := packer.io.start.ready
    io.mask_idx.ready    := packer.io.mask.ready
    io.load_packet.valid := packer.io.load_packet.valid
    io.load_packet.bits  := packer.io.load_packet.bits

  // SKIPPING case (pass through skipper)
  } .elsewhen (state === State.SKIPPING) {
    io.start.ready       := skipper.io.start.ready
    io.mask_idx.ready    := skipper.io.mask.ready
    io.load_packet.valid := skipper.io.load_packet.valid
    io.load_packet.bits  := skipper.io.load_packet.bits

  // WALKING case (pass through walker)
  } .elsewhen (state === State.WALKING) {
    io.start.ready       := walker.io.start.ready
    io.mask_idx.ready    := walker.io.index.ready
    io.load_packet.valid := walker.io.load_packet.valid
    io.load_packet.bits  := walker.io.load_packet.bits

  // IDLE case (no output but transparent to inputs)
  } .otherwise {
    io.start.ready       := PriorityMux(Seq(
      (packable)   -> packer.io.start.ready,
      (skipable)   -> skipper.io.start.ready,
      (walkable)   -> walker.io.start.ready,
      (true.B)     -> true.B
    ))
    io.mask_idx.ready    := PriorityMux(Seq(
      (packable)   -> packer.io.mask.ready,
      (skipable)   -> skipper.io.mask.ready,
      (walkable)   -> walker.io.index.ready,
      (true.B)     -> false.B
    ))
    io.load_packet.valid := false.B
    io.load_packet.bits  := DontCare
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
    }.elsewhen (io.load_packet.fire) {
      when (io.load_packet.bits.last) {
        state := State.IDLE
      }
    }
  }

}

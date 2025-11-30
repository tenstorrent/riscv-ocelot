// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

import boom.exu.FUConstants._
import boom.common._
import boom.exu._
import boom.util._

import chisel3.dontTouch // this is for debugging purposes

class StorePacket(override val VLEN: Int, override val DMEM_WIDTH: Int)(implicit p: Parameters)
extends Bundle with VecLSGenConstants {
  val addr     = UInt(64.W)
  val data     = UInt(DMEM_WIDTH.W)
  val mem_size = UInt(log2Ceil(DMEM_WIDTH/8).W) // bytes to store
  val elem_id  = UInt(EL_ID_W.W) // element ID for vstart
  val sb_id    = UInt(5.W) // store buffer id
  val is_fake  = Bool()    // must ignore (since BOOM doesn't support masked stores)
  val misaligned = Bool()
  val last     = Bool()    // last element in sequence
  val uop      = new MicroOp()
  val poison   = Bool()
}

// Store Generator for OVI
// holds all the store generators for OVI
class StoreGen(override val VLEN: Int, override val DMEM_WIDTH: Int)(implicit p: Parameters)
extends BoomModule with VecLSGenConstants {
  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = Flipped(DecoupledIO(new ConfigInfo(VLEN, DMEM_WIDTH)))
    // core signals for poison updates
    val core = new Bundle {
      val rob_pnr_idx  = Input(UInt(robAddrSz.W))
      val rob_head_idx = Input(UInt(robAddrSz.W))
      val brupdate     = Input(new BrUpdateInfo())
      val exception    = Input(Bool())
    }
    // mask/index interface (for masked/indexed stores)
    val mask_idx = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val data  = Input(UInt((MASK_W+2).W))
    }
    // store data interface (from VDB)
    val vdb_data = new Bundle {
      val read_bytes  = Output(UInt(VDB_R_SIZE_BYTES.W)) // like a ready signal
      val read_all    = Output(Bool())
      val valid_bytes = Input(UInt(VDB_R_SIZE_BYTES.W))  // like a valid signal
      val data        = Input(UInt(DMEM_WIDTH.W))
    }
    // store process FSM outputs (packet info)
    val store_packet = DecoupledIO(new StorePacket(VLEN, DMEM_WIDTH))
    // status signal
    val gen_active = Output(Bool())
  })

  // ======== Definitions ========

  object State extends ChiselEnum {
    val IDLE, PACKING, SKIPPING, WALKING, BYPASS = Value
  }

  // state and config registers
  val state   = RegInit(State.IDLE)
  val start_q = RegInit(0.U.asTypeOf(new ConfigInfo(VLEN, DMEM_WIDTH))) // latched config info

  // mux between immideate input and latched config info
  val muxed_start_bits = Mux((state === State.IDLE), io.start.bits, start_q)
  // update speculative info (poison bit and branch stuff)
  val config_info = Wire(new ConfigInfo(VLEN, DMEM_WIDTH))
  config_info := muxed_start_bits
  config_info.uop.br_mask := GetNewBrMask(io.core.brupdate, muxed_start_bits.uop)
  config_info.poison := (
    (muxed_start_bits.poison) ||
    (io.core.exception && !IsOlder(muxed_start_bits.uop.rob_idx, io.core.rob_pnr_idx, io.core.rob_head_idx)) ||
    (IsKilledByBranch(io.core.brupdate, muxed_start_bits.uop))
  )

  // control signals
  val bypassable = (config_info.vl === 0.U) || (config_info.vstart >= config_info.vl)
  val packable   = (config_info.stride_is_1 && config_info.is_good_seg && !config_info.is_mask)
  val skipable   = (config_info.is_mask  && !config_info.is_index)
  val walkable   = (!config_info.is_mask ||  config_info.is_index)

  // to force and not pack across segments to reduce hardware complexity (ckicken bit)
  val use_seg_constraint = (config_info.seg_count > 1.U)

  // ======== Store Generators ========

  // --- packer ---
  val packer = Module(new StorePacker(VLEN, DMEM_WIDTH))
  // start config
  packer.io.start.bits  := config_info
  packer.io.use_seg_constraint := use_seg_constraint
  // vdb data
  packer.io.vdb_data.valid_bytes := io.vdb_data.valid_bytes
  packer.io.vdb_data.data        := io.vdb_data.data
  // store packet
  packer.io.store_packet.ready := io.store_packet.ready

  // --- skipper ---
  val skipper = Module(new StoreSkipper(VLEN, DMEM_WIDTH))
  // start config
  skipper.io.start.bits  := config_info
  skipper.io.use_seg_constraint := use_seg_constraint
  // mask config
  skipper.io.mask.valid     := io.mask_idx.valid
  skipper.io.mask.mask_data := io.mask_idx.data(MASK_W-1, 0) // data only
  // vdb data
  skipper.io.vdb_data.valid_bytes := io.vdb_data.valid_bytes
  skipper.io.vdb_data.data        := io.vdb_data.data
  // store packet
  skipper.io.store_packet.ready := io.store_packet.ready

  // --- walker ---
  val walker = Module(new StoreWalker(VLEN, DMEM_WIDTH))
  // start config
  walker.io.start.bits  := config_info
  // index config
  walker.io.index.valid       := io.mask_idx.valid
  walker.io.index.index_value := io.mask_idx.data(MASK_W-1, 0).asSInt // idx val
  walker.io.index.mask_bit    := io.mask_idx.data(MASK_W)      // mask bit
  walker.io.index.last_index  := io.mask_idx.data(MASK_W+1)    // last bit
  walker.io.use_seg_constraint := use_seg_constraint
  // vdb data
  walker.io.vdb_data.valid_bytes := io.vdb_data.valid_bytes
  walker.io.vdb_data.data        := io.vdb_data.data
  // store packet
  walker.io.store_packet.ready := io.store_packet.ready

  // ======== Start Mux ========

  // BYPASS case (no gens valid)
  when (bypassable) {
    packer.io.start.valid  := false.B
    skipper.io.start.valid := false.B
    walker.io.start.valid  := false.B
  // PACKING case (packer valid)
  } .elsewhen (packable) {
    packer.io.start.valid  := io.start.valid
    skipper.io.start.valid := false.B
    walker.io.start.valid  := false.B
  // SKIPPING case (skipper valid)
  } .elsewhen (skipable) {
    packer.io.start.valid  := false.B
    skipper.io.start.valid := io.start.valid
    walker.io.start.valid  := false.B
  // WALKING case (walker valid)
  } .elsewhen (walkable) {
    packer.io.start.valid  := false.B
    skipper.io.start.valid := false.B
    walker.io.start.valid  := io.start.valid
  // IDLE case (no gens valid)
  } .otherwise {
    packer.io.start.valid  := false.B
    skipper.io.start.valid := false.B
    walker.io.start.valid  := false.B
  }

  // ======== Outputs ========

  // bypass packet: had to do this because of chisel binding conflicts
  val bypass_packet = Wire(new StorePacket(VLEN, DMEM_WIDTH))
  bypass_packet.addr       := DontCare
  bypass_packet.data       := DontCare
  bypass_packet.mem_size   := 0.U      // this doesnt make a lot of sense cuz its in log domain but its just like a "default" value
  bypass_packet.elem_id    := 0.U 
  bypass_packet.sb_id      := config_info.sb_id
  bypass_packet.is_fake    := true.B   // fake store
  bypass_packet.misaligned := DontCare
  bypass_packet.last       := true.B   // assert end
  bypass_packet.uop        := config_info.uop
  bypass_packet.poison     := config_info.poison
  
  // BYPASS case (no output but assert last)
  when (state === State.BYPASS) {
    io.start.ready         := false.B
    io.mask_idx.ready      := false.B
    io.vdb_data.read_bytes := 0.U
    io.vdb_data.read_all   := false.B
    io.store_packet.valid  := true.B
    io.store_packet.bits   := bypass_packet
    io.gen_active          := true.B

  // PACKING case (pass through packer)
  } .elsewhen (state === State.PACKING) {
    io.start.ready         := packer.io.start.ready
    io.mask_idx.ready      := false.B
    io.vdb_data.read_bytes := packer.io.vdb_data.read_bytes
    io.vdb_data.read_all   := packer.io.vdb_data.read_all
    io.store_packet.valid  := packer.io.store_packet.valid
    io.store_packet.bits   := packer.io.store_packet.bits
    io.gen_active          := packer.io.gen_active

  // SKIPPING case (pass through skipper)
  } .elsewhen (state === State.SKIPPING) {
    io.start.ready         := skipper.io.start.ready
    io.mask_idx.ready      := skipper.io.mask.ready
    io.vdb_data.read_bytes := skipper.io.vdb_data.read_bytes
    io.vdb_data.read_all   := skipper.io.vdb_data.read_all
    io.store_packet.valid  := skipper.io.store_packet.valid
    io.store_packet.bits   := skipper.io.store_packet.bits
    io.gen_active          := skipper.io.gen_active

  // WALKING case (pass through walker)
  } .elsewhen (state === State.WALKING) {
    io.start.ready         := walker.io.start.ready
    io.mask_idx.ready      := walker.io.index.ready
    io.vdb_data.read_bytes := walker.io.vdb_data.read_bytes
    io.vdb_data.read_all   := walker.io.vdb_data.read_all
    io.store_packet.valid  := walker.io.store_packet.valid
    io.store_packet.bits   := walker.io.store_packet.bits
    io.gen_active          := walker.io.gen_active

  // IDLE case (no output but transparent to inputs)
  } .otherwise {
    io.start.ready       := PriorityMux(Seq(
      (bypassable) -> true.B,
      (packable)   -> packer.io.start.ready,
      (skipable)   -> skipper.io.start.ready,
      (walkable)   -> walker.io.start.ready,
      (true.B)     -> false.B
    ))
    io.mask_idx.ready     := PriorityMux(Seq(
      (bypassable) -> false.B,
      (packable)   -> false.B,
      (skipable)   -> skipper.io.mask.ready,
      (walkable)   -> walker.io.index.ready,
      (true.B)     -> false.B
    ))
    io.vdb_data.read_bytes := 0.U
    io.vdb_data.read_all   := false.B
    io.store_packet.valid  := false.B
    io.store_packet.bits   := DontCare
    io.gen_active          := false.B
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
      start_q := config_info
    }
  }.otherwise {
    // end on last packet
    when (io.store_packet.fire) {
      when (io.store_packet.bits.last) {
        state := State.IDLE
      }
    }
    // update speculative state
    start_q := config_info
  }

  // ======== Debug Signals ========

  switch (state) {
    is (State.IDLE) {
      assert(packer.io.gen_active === false.B, "packer should not be active in IDLE")
      assert(skipper.io.gen_active === false.B, "skipper should not be active in IDLE")
      assert(walker.io.gen_active === false.B, "walker should not be active in IDLE")
    }
    is (State.BYPASS) {
      assert(packer.io.gen_active === false.B, "packer should not be active in BYPASS")
      assert(skipper.io.gen_active === false.B, "skipper should not be active in BYPASS")
      assert(walker.io.gen_active === false.B, "walker should not be active in BYPASS")
    }
    is (State.PACKING) {
      assert(packer.io.gen_active === true.B, "packer should be active in PACKING")
      assert(skipper.io.gen_active === false.B, "skipper should not be active in PACKING")
      assert(walker.io.gen_active === false.B, "walker should not be active in PACKING")
    }
    is (State.SKIPPING) {
      assert(packer.io.gen_active === false.B, "packer should not be active in SKIPPING")
      assert(skipper.io.gen_active === true.B, "skipper should be active in SKIPPING")
      assert(walker.io.gen_active === false.B, "walker should not be active in SKIPPING")
    }
    is (State.WALKING) {
      assert(packer.io.gen_active === false.B, "packer should not be active in WALKING")
      assert(skipper.io.gen_active === false.B, "skipper should not be active in WALKING")
      assert(walker.io.gen_active === true.B, "walker should be active in WALKING")
    }
  }

  // when (io.store_packet.fire) {
  //   assert(!io.store_packet.bits.misaligned, "StoreGen: misaligned store at addr = %x\n", io.store_packet.bits.addr)
  // }

}
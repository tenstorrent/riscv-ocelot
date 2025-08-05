// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

import boom.exu.FUConstants._

class StoreWalker(override val VLEN: Int, override val DMEM_WIDTH: Int)
extends Module with VecLSGenConstants {
  // ======== Parameters ========
  val CTR_WIDTH = (EL_ID_W+((1<<EMUL_ENC_W)-1)) // same as max of vl

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = DecoupledIO(new ConfigInfo(VLEN, DMEM_WIDTH))
    // index interface (for indexed stores)
    val index = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val index_value = Input(SInt(MASK_W.W))  // Signed index offset in bytes
      val mask_bit    = Input(Bool())      // Element valid/invalid
      val last_index  = Input(Bool())      // Last index in sequence
    }
    // kill signal (used to reset the FSM)
    val kill = Input(Bool())
    // store data interface
    val vdb_data = new Bundle {
      val read_bytes  = Output(UInt(log2Ceil(DMEM_WIDTH/8).W)) // like a ready signal
      val valid_bytes = Input(UInt(log2Ceil(DMEM_WIDTH/8).W))  // like a valid signal
      val data        = Input(UInt(DMEM_WIDTH.W))
    }
    // store packet
    val store_packet = Flipped(DecoupledIO(new StorePacket(VLEN, DMEM_WIDTH)))
  })

  // ======== Definitions ========

  object State extends ChiselEnum {
    val IDLE, WALKING = Value
  }

  // ======== Config info ========

  val state     = RegInit(State.IDLE)
  val sb_id     = io.start.bits.sb_id
  val base_v_reg = io.start.bits.base_v_reg
  val vl        = io.start.bits.vl
  val eew_enc   = io.start.bits.eew_enc
  val emul_enc  = io.start.bits.emul_enc
  val stride    = io.start.bits.stride
  val seg_count = io.start.bits.seg_count
  val is_mask   = io.start.bits.is_mask
  val is_index  = io.start.bits.is_index
  val base_addr = io.start.bits.base_addr

  // ======== Walking state ========

  val current_seg_id   = RegInit(0.U(SEG_W.W))                 // current segment index
  val current_el_id    = RegInit(0.U(EL_ID_W.W))               // current element index
  val current_v_group_id = RegInit(0.U(((1<<EMUL_ENC_W)-1).W)) // current vector group
  val current_addr     = RegInit(0.U(64.W))                    // current address (incremental)
  val current_ctr      = RegInit(0.U(CTR_WIDTH.W))             // element counter to vl
  val current_mask_bit = RegInit(false.B)                      // current mask bit (for indexed stores)
  val current_last_index = RegInit(false.B)                    // current "last index" state (remember across segments)
  val dmem_off   = RegInit(0.U(DMEM_ENC.W)) // memory alignment offset
  val dmem_max   = RegInit(0.U(DMEM_ENC.W)) // max elements to fit in DMEM

  // ======== Packing Constraints ========

  // elements that can be stored contiguously depending on mem alignment
  val dmem_constraint = PriorityEncoderOH(dmem_off | ~(dmem_max-1.U))

  // elements until the next stride update
  val packing_constraint = seg_count - current_seg_id

  val seg_inc_val = WireInit(0.U(SEG_W.W))
  when (dmem_constraint <= packing_constraint) {
    seg_inc_val := Reverse(PriorityEncoderOH(Reverse(dmem_constraint)))
  }.elsewhen (packing_constraint <= dmem_constraint) {
    seg_inc_val := Reverse(PriorityEncoderOH(Reverse(packing_constraint)))
  }
  val seg_inc_enc = PriorityEncoder(seg_inc_val)

  val dmem_constraint_met    = (seg_inc_val === dmem_constraint)
  val packing_constraint_met = (seg_inc_val === packing_constraint)

  // ======== Max Constraints ========

  val max_seg_id_met  = (seg_inc_val === packing_constraint)
  val max_el_id_met   = (current_el_id === ((VLEN_BYTES.U >> eew_enc) - 1.U))
  val max_v_group_met = (current_v_group_id === ((1.U << emul_enc) - 1.U))
  val max_ctr_met     = ((current_ctr === (vl - 1.U)) || (current_last_index)) && max_seg_id_met // last ever packet

  // ======== Outputs ========

  // ready-valid signals
  val vdb_valid           = (io.vdb_data.valid_bytes =/= 0.U)
  io.start.ready         := ((state === State.IDLE)    && (!is_index || io.index.valid))
  io.index.ready         := ((state === State.WALKING) && (is_index && max_seg_id_met && !max_ctr_met) && vdb_valid) ||
                            ((state === State.IDLE)    && (is_index && io.start.valid))
  io.store_packet.valid  := ((state === State.WALKING) && (!io.index.ready || io.index.valid) && (vdb_valid))
  val vdb_ready           = ((state === State.WALKING) && (!io.index.ready || io.index.valid) && (io.store_packet.ready))
  io.vdb_data.read_bytes := Mux(vdb_ready, (1.U << (seg_inc_enc + eew_enc)), 0.U)

  // packet info
  io.store_packet.bits.addr     := current_addr + (current_seg_id << emul_enc)
  io.store_packet.bits.data     := io.vdb_data.data
  io.store_packet.bits.mem_size := (seg_inc_enc + eew_enc)
  io.store_packet.bits.sb_id    := sb_id
  io.store_packet.bits.is_fake  := (seg_inc_val === 0.U) || (current_mask_bit === false.B)
  io.store_packet.bits.misaligned := false.B
  io.store_packet.bits.last     := (state === State.WALKING) && (max_ctr_met)

  // ======== State Machine ========

  switch (state) {
    is (State.IDLE) {
      when (io.start.fire) {
        // -- Input config --
        state := State.WALKING

        // -- Initialize counters --
        current_el_id   := 0.U
        current_seg_id  := 0.U
        current_v_group_id := 0.U
        current_addr    := (base_addr.asSInt + Mux(is_index, io.index.index_value, 0.S)).asUInt
        current_ctr     := 0.U
        current_mask_bit   := io.index.mask_bit
        current_last_index := io.index.last_index

        // -- Initialize DMEM info --
        val direction = Mux(is_index, io.index.index_value, stride)(63)
        val high_off  = (((1<<(ADDR_BREAK))-1).U - base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from end of DMEM (high) to base_addr
        val low_off   = (base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from start of DMEM (low) to base_addr
        dmem_off := Mux(direction, high_off, low_off)
        dmem_max := (DMEM_BYTES.U >> eew_enc)
      }
    }
    is (State.WALKING) {
      when (io.kill) {
        state := State.IDLE
      } .elsewhen (io.store_packet.fire) {

        // -- Next Address calculation --
        val next_addr = Mux(
          is_index,   // load can be index / stride type
          (base_addr.asSInt + io.index.index_value).asUInt,
          (current_addr.asSInt + stride).asUInt
        )

        // -- Counter ripple logic --
        // [v_group, el, seg]: [x, x, +1]
        when (!max_seg_id_met) {
          current_seg_id := current_seg_id + seg_inc_val
        // [v_group, el, seg]: [x, +1, 0]
        }.elsewhen (!max_el_id_met) {
          current_seg_id := 0.U
          current_el_id := current_el_id + 1.U
        // [v_group, el, seg]: [+1, 0, 0]
        }.otherwise {
          current_seg_id := 0.U
          current_el_id := 0.U
          current_v_group_id := current_v_group_id + 1.U
        }

        // -- Next element config/updates --
        when (max_seg_id_met) {
          current_addr       := next_addr           // update address
          current_ctr        := current_ctr + 1.U   // increment CTR
          current_mask_bit   := io.index.mask_bit   // update mask bit
          current_last_index := io.index.last_index // update last index
        }

        // -- DMEM offset increment --
        // recalc dmem_off for next element
        when (max_seg_id_met) {
          val direction = Mux(is_index, io.index.index_value, stride)(63)
          val high_off  = (((1<<(ADDR_BREAK))-1).U - next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          val low_off   = (next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          dmem_off := Mux(direction, high_off, low_off)
        // wrap inc dmem_off
        }.elsewhen(dmem_constraint_met) {
          dmem_off := 0.U
        // inc dmem_off for next segment
        }.otherwise{
          dmem_off := dmem_off + seg_inc_val
        }

        // -- Last packet transition --
        when (io.store_packet.bits.last) {
          state := State.IDLE
        }

      }
    }
  }

} 
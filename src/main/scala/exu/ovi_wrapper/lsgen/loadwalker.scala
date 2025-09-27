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

// Load Walker for OVI
// This module walks through vector load elements one by one sequentially

// Segments are packed. Masked elements are walked through.
// no non-index masked loads (use skipper for that)
// This walker will not send a packet (or even start) if the next requires a new index entry (to avoid "wait" states)
class LoadWalker(override val VLEN: Int, override val DMEM_WIDTH: Int)(implicit p: Parameters)
extends Module with VecLSGenConstants {
  // ======== Parameters ========
  val CTR_WIDTH    = (EL_ID_W+((1<<EMUL_ENC_W)-1)) // should be same as vl

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = Flipped(DecoupledIO(new ConfigInfo(VLEN, DMEM_WIDTH))) // expects latched config info (DO NOT CHANGE DURING FSM)
    val use_seg_constraint = Input(Bool())
    // index interface (for indexed loads)
    val index = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val index_value = Input(SInt(MASK_W.W))  // Signed index offset in bytes
      val mask_bit    = Input(Bool())      // Element valid/invalid
      val last_index  = Input(Bool())      // Last index in sequence
    }
    // kill signal (used to reset the FSM)
    val kill = Input(Bool())
    // status signal
    val gen_active = Output(Bool())
    // load process FSM outputs (packet info)
    val load_packet = DecoupledIO(new LoadPacket(VLEN, DMEM_WIDTH))
    // debug signals
    val debug = new Bundle {
      val debug_curr_seg_id     = Output(UInt(SEG_W.W))
      val debug_curr_el_id      = Output(UInt(EL_ID_W.W))
      val debug_curr_v_group_id = Output(UInt(((1<<EMUL_ENC_W)-1).W))
      val debug_curr_addr       = Output(UInt(64.W))
      val debug_curr_ctr        = Output(UInt(CTR_WIDTH.W))
      val debug_curr_dmem_off   = Output(UInt(DMEM_ENC.W))
      val debug_curr_dmem_max   = Output(UInt(DMEM_ENC.W))
      val debug_seg_inc_val     = Output(UInt(SEG_W.W))
    }
  })

  // ======== Definitions ========

  object State extends ChiselEnum {
    val IDLE, VSTART_HANDLING, WALKING = Value
  }

  // ======== Config info ========

  val state      = RegInit(State.IDLE)
  val sb_id      = io.start.bits.sb_id
  val base_v_reg = io.start.bits.base_v_reg
  val vl         = io.start.bits.vl
  val vstart     = io.start.bits.vstart
  val eew_enc    = io.start.bits.eew_enc
  val emul_enc   = io.start.bits.emul_enc
  val stride     = io.start.bits.stride
  val seg_count  = io.start.bits.seg_count
  val is_mask    = io.start.bits.is_mask
  val is_index   = io.start.bits.is_index
  val base_addr  = io.start.bits.base_addr
  val use_seg_constraint = io.use_seg_constraint

  // ======== Walking state ========

  val current_seg_id   = RegInit(0.U(SEG_W.W))                 // current segment index
  val current_el_id    = RegInit(0.U(EL_ID_W.W))               // current element index
  val current_v_group_id = RegInit(0.U(((1<<EMUL_ENC_W)-1).W)) // current vector group
  val current_addr     = RegInit(0.U(64.W))                    // current address (incremental)
  val current_ctr      = RegInit(0.U(CTR_WIDTH.W))             // element counter to vl
  val current_mask_bit = RegInit(false.B)                      // current mask bit (for indexed loads)
  val current_last_index = RegInit(false.B)                    // current "last index" state (remember across segments)
  val current_dir        = RegInit(false.B)                    // current direction of stride/index
  val dmem_off   = RegInit(0.U(DMEM_ENC.W)) // memory alignment offset
  val dmem_max   = RegInit(0.U(DMEM_ENC.W)) // max elements to fit in DMEM

  // ======== Packing Constraints ========
  // elements until DMEM has hit its boundary
  val dmem_constraint = dmem_max - dmem_off
  // elements until the next stride update
  val packing_constraint = seg_count - current_seg_id
  // max constraints (for "wrapping")
  val max_seg_id_met  = (packing_constraint <= dmem_constraint)
  val max_el_id_met   = (current_el_id === ((VLEN_BYTES.U >> eew_enc) - 1.U))
  val max_v_group_met = (current_v_group_id === ((1.U << emul_enc) - 1.U))
  val max_ctr_met     = ((current_ctr === (vl - 1.U)) || (is_index && current_last_index)) && max_seg_id_met // last ever packet

  // ======== Advance SEG based on constraints ========
  val seg_inc_val = WireInit(0.U(SEG_W.W))
  
  // check constraints hit (NOTE: multiple can be triggered)
  val dmem_constraint_met    = (dmem_constraint <= packing_constraint)
  val packing_constraint_met = (packing_constraint <= dmem_constraint)

  // increment based on constraint met
  // Segment Packing is smallest
  when (packing_constraint_met) {
    seg_inc_val := packing_constraint
  // DMEM is smallest
  }.otherwise {
    seg_inc_val := dmem_constraint
  }


  // ======== Outputs ========

  // ready-valid signals
  // NOTE: I am not releasing the last segment for any index element until I get the next index
  //       I am also not starting the config until I get the first index (this is to avoid using a wait state)
  val need_next_index   = (is_index && max_seg_id_met && !max_ctr_met)
  io.start.ready       := ((state === State.IDLE)    && (!is_index || io.index.valid))
  io.index.ready       := ((state === State.WALKING) && need_next_index && (io.load_packet.ready)) ||
                          ((state === State.IDLE)    && (is_index && io.start.valid))
  io.load_packet.valid := ((state === State.WALKING) && (!need_next_index || io.index.valid))
  io.gen_active        := (state === State.WALKING) || (state === State.VSTART_HANDLING)

  val addr_off = (current_seg_id << eew_enc).asSInt
  
  // packet info
  io.load_packet.bits.addr     := (current_addr.asSInt + addr_off).asUInt
  io.load_packet.bits.v_reg    := base_v_reg + (current_seg_id << emul_enc) + current_v_group_id
  io.load_packet.bits.el_id    := current_el_id
  io.load_packet.bits.el_off   := dmem_off
  io.load_packet.bits.el_count := seg_inc_val
  io.load_packet.bits.sb_id    := sb_id
  io.load_packet.bits.mask_data  := current_mask_bit
  io.load_packet.bits.mask_valid := (is_index && is_mask)
  io.load_packet.bits.is_fake    := (seg_inc_val === 0.U) || (is_mask && (current_mask_bit === false.B))
  io.load_packet.bits.misaligned := ((current_addr & ((1.U << eew_enc) - 1.U)) =/= 0.U)
  io.load_packet.bits.last     := (state === State.WALKING) && (max_ctr_met)
  io.load_packet.bits.uop      := io.start.bits.uop
  io.load_packet.bits.dir      := current_dir && !use_seg_constraint
  io.load_packet.bits.is_fof   := io.start.bits.is_fof

  // ======== Vstart Handling Constraints ========
  // handling vstart for the walker has to be done across multiple cycles
  // only in the case of strided (not index) since VPU should not send the idx/mask below vstart

  // split vstart into it's el_id and v_group_id components
  val el_mask_width = (log2Ceil(VLEN).U - eew_enc)
  val vstart_el_id      = vstart & ((1.U << el_mask_width) - 1.U)
  val vstart_v_group_id = vstart >> el_mask_width

  // calculate jump to vstart
  val vstart_dist     = (vstart - current_ctr) // distance to vstart
  val vstart_last_inc = ((vstart_dist & (vstart_dist - 1.U)) === 0.U) // if the distance is 1-hot, then only 1 jump away from vstart
  val vstart_skip_enc = PriorityEncoder(vstart_dist) // jump by power of 2

  // ======== State Machine ========

  switch(state) {
    // IDLE STATE
    is(State.IDLE) {
      when(io.start.fire) {
        
        // -- Next Address and dir calculation --
        val next_addr = (base_addr.asSInt + Mux(is_index, io.index.index_value, 0.S)).asUInt
        val next_direction = Mux(is_index, io.index.index_value, stride)(63)
        
        // -- Input config --
        val goto_handling = (vstart =/= 0.U) && !is_index // only strided need handling
        state := Mux(
          goto_handling,
          State.VSTART_HANDLING,
          State.WALKING
        )

        // -- Initialize counters --
        current_el_id      := vstart_el_id
        current_seg_id     := 0.U
        current_v_group_id := vstart_v_group_id
        current_addr       := next_addr
        current_ctr        := 0.U
        current_mask_bit   := io.index.mask_bit
        current_last_index := io.index.last_index
        current_dir        := next_direction && !use_seg_constraint

        // -- Initialize DMEM info --
        val high_off  = (((1<<(ADDR_BREAK))-1).U - next_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from end of DMEM (high) to base_addr
        val low_off   = (next_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from start of DMEM (low) to base_addr
        dmem_off := Mux(next_direction && !use_seg_constraint, high_off, low_off)
        dmem_max := (DMEM_BYTES.U >> eew_enc)
      }
    }
    // VSTART HANDLING STATE
    is(State.VSTART_HANDLING) {
      when (io.kill) {
        state := State.IDLE
      }.otherwise {

        // -- Next Address calculation --
        val next_addr = (current_addr.asSInt + (stride << vstart_skip_enc)).asUInt
        val next_direction = stride(63)

        // -- Increment address and counter --
        current_addr := next_addr
        current_ctr  := current_ctr + (1.U << vstart_skip_enc)

        // -- State transition --
        when (vstart_last_inc) {
          // next state
          state := State.WALKING
          // realign dmem offset to the new address
          val high_off = (((1<<(ADDR_BREAK))-1).U - next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          val low_off  = (next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          dmem_off := Mux(next_direction && !use_seg_constraint, high_off, low_off)
        }
      }
    }
    // WALKING STATE
    is(State.WALKING) {
      when (io.kill) {
        state := State.IDLE
      }.elsewhen (io.load_packet.fire) {

        // -- Next Address and dir calculation --
        val next_addr = Mux(
          is_index,   // load can be index / stride type
          (base_addr.asSInt + io.index.index_value).asUInt,
          (current_addr.asSInt + stride).asUInt
        )
        val next_direction = Mux(is_index, io.index.index_value, stride)(63)

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
          current_dir        := next_direction     // update direction
        }

        // -- DMEM offset increment --
        // recalc dmem_off for next element
        when (max_seg_id_met) {
          val high_off  = (((1<<(ADDR_BREAK))-1).U - next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          val low_off   = (next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          dmem_off := Mux(next_direction && !use_seg_constraint, high_off, low_off)
        // wrap inc dmem_off
        }.elsewhen(dmem_constraint_met) {
          dmem_off := 0.U
        // inc dmem_off for next segment
        }.otherwise{
          dmem_off := dmem_off + seg_inc_val
        }

        // -- Last packet transition --
        when (io.load_packet.bits.last) {
          state := State.IDLE
        }
        
      }
    }
  }

  // ======== Debug ========

  when(io.start.fire) {
    assert ((!is_mask || is_index), "Loadwalker does not support non-indexed loads that are masked")
  }

  io.debug.debug_curr_seg_id := current_seg_id
  io.debug.debug_curr_el_id := current_el_id
  io.debug.debug_curr_v_group_id := current_v_group_id
  io.debug.debug_curr_addr := current_addr
  io.debug.debug_curr_ctr := current_ctr
  io.debug.debug_curr_dmem_off := dmem_off
  io.debug.debug_curr_dmem_max := dmem_max
  io.debug.debug_seg_inc_val := seg_inc_val
  
  dontTouch(io.debug)
  dontTouch(io.start)
  dontTouch(io.load_packet)
  dontTouch(io.kill)

}


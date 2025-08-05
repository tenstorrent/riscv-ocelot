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

// Load Skipper for OVI
// This module is responsible for skipping masked off elements

// Segments are packed. Masked off elements are skipped. Masked elements are walked through.
// no index support (use walker for that)
// This packer will not send a packet (or even start) if the next requires a new mask entry (to avoid "wait" states)
class LoadSkipper(override val VLEN: Int, override val DMEM_WIDTH: Int)
extends Module with VecLSGenConstants {
  // ======== Parameters ========
  val CTR_WIDTH    = (EL_ID_W+((1<<EMUL_ENC_W)-1)) // should be same as vl

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // config signals
    val start = Flipped(DecoupledIO(new ConfigInfo(VLEN, DMEM_WIDTH))) // expects latched config info (DO NOT CHANGE DURING FSM)
    // mask interface (for masked loads)
    val mask = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val mask_data = Input(UInt(MASK_W.W))
    }
    // kill signal (used to reset the FSM)
    val kill = Input(Bool())
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
    val IDLE, SKIPPING = Value
  }

  // ======== Config info ========

  val state      = RegInit(State.IDLE)
  val sb_id      = io.start.bits.sb_id
  val base_v_reg = io.start.bits.base_v_reg
  val vl         = io.start.bits.vl
  val eew_enc    = io.start.bits.eew_enc
  val emul_enc   = io.start.bits.emul_enc
  val stride     = io.start.bits.stride
  val stride_dir = io.start.bits.stride_dir
  val seg_count  = io.start.bits.seg_count
  val is_mask    = io.start.bits.is_mask
  val base_addr  = io.start.bits.base_addr

  // ======== Walking state ========

  val current_seg_id   = RegInit(0.U(SEG_W.W))                 // current segment index
  val current_el_id    = RegInit(0.U(EL_ID_W.W))               // current element index
  val current_v_group_id = RegInit(0.U(((1<<EMUL_ENC_W)-1).W)) // current vector group
  val current_addr     = RegInit(0.U(64.W))                    // current address (incremental)
  val current_ctr      = RegInit(0.U(CTR_WIDTH.W))             // element counter to vl
  val current_mask_data = RegInit(0.U(MASK_W.W))               // mask data will be shifted as used
  val current_mask_off = RegInit(0.U(log2Ceil(MASK_W).W))      // running shifted offset amount
  val dmem_max   = RegInit(0.U(DMEM_ENC.W)) // max elements to fit in DMEM
  val dmem_off   = RegInit(0.U(DMEM_ENC.W)) // memory alignment offset

  // ======== Skipping Constraints ========
  val mask_constraint = Mux(
    (current_mask_data =/= 0.U),
    PriorityEncoder(current_mask_data),
    MASK_W.U - current_mask_off
  )
  val vreg_constraint = ((VLEN_BYTES.U >> eew_enc) -1.U) - current_el_id
  val vl_constraint   = (vl - 1.U) - current_ctr

  // skip_val is the biggest power of 2 value smaller than the smallest of skipping constraints
  val skip_val = WireInit(0.U(11.W))
  when ((mask_constraint <= vreg_constraint) && (mask_constraint <= vl_constraint)) {
    skip_val := Reverse(PriorityEncoderOH(Reverse(mask_constraint)))
  }.elsewhen ((vreg_constraint <= mask_constraint) && (vreg_constraint <= vl_constraint)) {
    skip_val := Reverse(PriorityEncoderOH(Reverse(vreg_constraint)))
  }.elsewhen ((vl_constraint <= mask_constraint) && (vl_constraint <= vreg_constraint)) {
    skip_val := Reverse(PriorityEncoderOH(Reverse(vl_constraint)))
  }

  val skippable = (skip_val =/= 0.U)
  val skip_enc  = PriorityEncoder(skip_val)

  val vreg_constraint_met = (skip_val === vreg_constraint)
  val vl_constraint_met   = (skip_val === vl_constraint)

  // ======== Packing Constraints ========
  // elements until DMEM has hit its boundary
  val dmem_constraint = dmem_max - dmem_off
  // elements until the next stride update
  val packing_constraint = seg_count - current_seg_id

  // max constraints (for "wrapping")
  val max_seg_id_met  = (packing_constraint <= dmem_constraint)
  val max_el_id_met   = (current_el_id === ((VLEN_BYTES.U >> eew_enc) - 1.U))
  val max_v_group_met = (current_v_group_id === ((1.U << emul_enc) - 1.U))
  val max_ctr_met     = (current_ctr === (vl - 1.U)) && max_seg_id_met // last ever packet
  val max_mask_met    = ((current_mask_off + skip_val) === MASK_W.U)

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

  io.start.ready       := ((state === State.IDLE) && (!is_mask || io.mask.valid))
  io.mask.ready        := ((state === State.SKIPPING) && (is_mask && max_mask_met && !max_ctr_met)) ||
                          ((state === State.IDLE)     && (is_mask && io.start.valid))
  io.load_packet.valid := ((state === State.SKIPPING) && (!io.mask.ready || io.mask.valid))

  // packet info
  io.load_packet.bits.addr     := current_addr + (current_seg_id << emul_enc)
  io.load_packet.bits.v_reg    := base_v_reg + (current_seg_id << emul_enc) + current_v_group_id
  io.load_packet.bits.el_id    := current_el_id
  io.load_packet.bits.el_off   := dmem_off
  io.load_packet.bits.el_count := (seg_inc_val << skip_enc)
  io.load_packet.bits.sb_id    := sb_id
  io.load_packet.bits.mask_data  := Mux(skippable, 0.U, current_mask_data)
  io.load_packet.bits.mask_valid := is_mask
  io.load_packet.bits.is_fake    := (seg_inc_val === 0.U) || skippable
  io.load_packet.bits.misaligned := ((current_addr & ((1.U << eew_enc) - 1.U)) =/= 0.U)
  io.load_packet.bits.last     := (state === State.SKIPPING) && (max_ctr_met || vl_constraint_met)

  // ======== State Machine ========

  switch(state) {
    is(State.IDLE) {
      when(io.start.fire) {
        // -- state config --
        state := State.SKIPPING

        // -- Initialize counters --
        current_el_id   := 0.U
        current_seg_id  := 0.U
        current_v_group_id := 0.U
        current_addr    := base_addr
        current_ctr     := 0.U
        current_mask_data := io.mask.mask_data
        current_mask_off  := 0.U

        // -- Initialize DMEM info --
        val high_off = (((1<<(ADDR_BREAK))-1).U - base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from end of DMEM (high) to base_addr
        val low_off  = (base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from start of DMEM (low) to base_addr
        dmem_off := Mux(stride_dir, high_off, low_off)
        dmem_max := (DMEM_BYTES.U >> eew_enc)
      }
    }

    is(State.SKIPPING) {
      when (io.kill) {
        state := State.IDLE
      }.elsewhen (io.load_packet.fire) {

        // -- Next Address calculation --
        val next_addr = (current_addr.asSInt + (stride << skip_enc)).asUInt

        // -- Counter ripple logic --
        // direct skip case (seg_id is always 0 when this happens)
        when (skippable) {
          current_el_id := current_el_id + skip_val
        // [v_group, el, seg]: [x, x, +1]
        }.elsewhen (!max_seg_id_met) {
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
        when (max_seg_id_met || skippable) {
          current_addr := next_addr
          current_ctr  := current_ctr + Mux(skippable, skip_val, 1.U)
        }

        // -- DMEM offset increment --
        // recalc dmem_off for new element
        when (max_seg_id_met || skippable) {
          dmem_off := (next_addr(DMEM_ENC-2, 0)) >> eew_enc
        // wrap inc dmem_off
        }.elsewhen(dmem_constraint_met) {
          dmem_off := 0.U
        // inc dmem_off for next segment
        }.otherwise{
          dmem_off := dmem_off + seg_inc_val
        }

        // -- Mask buffer update --
        when (max_mask_met) {
          current_mask_off  := 0.U
          current_mask_data := io.mask.mask_data
        } .elsewhen (skippable) {
          current_mask_off  := current_mask_off + skip_val
          current_mask_data := current_mask_data << skip_val
        } .elsewhen (max_seg_id_met) {
          current_mask_off  := current_mask_off + 1.U
          current_mask_data := current_mask_data << 1.U
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
    assert ((is_mask), "Loadskipper only supports masked loads")
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


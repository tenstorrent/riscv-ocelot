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

// Load Walker for OVI - Naive Approach
// This module walks through vector load elements one by one sequentially

// NOTE: "signal_enc" refers to the signal value encoded as its log2Ceil(value)
// since they are expected to be in powers of 2
class LoadWalker(val VLEN: Int, val DMEM_WIDTH: Int) extends Module {
  // ======== Parameters ========
  val VLEN_BYTES = VLEN/8
  val DMEM_BYTES = DMEM_WIDTH/8
  val DMEM_ENC   = log2Ceil(DMEM_BYTES+1)
  val EEW_ENC_W    = 3
  val EMUL_ENC_W   = 3
  val SEG_W        = 3
  val CTR_WIDTH    = 9 // should be same as vl

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val sb_id       = Input(UInt(5.W))
      val base_v_reg  = Input(UInt(5.W))
      val vl          = Input(UInt(9.W))
      val eew_enc     = Input(UInt(EEW_ENC_W.W))      // encoded in bytes (0=1B, 1=2B, 2=4B, 3=8B)
      val emul_enc    = Input(UInt(EMUL_ENC_W.W))
      val stride      = Input(UInt(64.W))
      val seg_count   = Input(UInt(SEG_W.W))
      val is_mask     = Input(Bool())
      val base_addr   = Input(UInt(64.W))
    }
    // kill signal (used to reset the FSM)
    val kill = Input(Bool())
    // load process FSM outputs (packet info)
    val load_packet = new Bundle {
      val ready = Input(Bool())
      val valid = Output(Bool())
      val addr     = Output(UInt(64.W))
      val v_reg    = Output(UInt(5.W))
      val el_id    = Output(UInt(11.W))
      val el_off   = Output(UInt(6.W))
      val el_count = Output(UInt(7.W))
      val sb_id    = Output(UInt(5.W))
      val last     = Output(Bool())
    }
    val debug = new Bundle {
      val debug_curr_seg_id     = Output(UInt(SEG_W.W))
      val debug_curr_el_id      = Output(UInt(11.W))
      val debug_curr_v_group_id = Output(UInt(EMUL_ENC_W.W))
      val debug_curr_addr       = Output(UInt(64.W))
      val debug_curr_ctr        = Output(UInt(CTR_WIDTH.W))
      val debug_curr_dmem_off   = Output(UInt(DMEM_ENC.W))
      val debug_curr_dmem_max   = Output(UInt(DMEM_ENC.W))
      val debug_ctr_inc_val     = Output(UInt(CTR_WIDTH.W))
    }
  })

  // ======== Definitions ========

  object State extends ChiselEnum {
    val IDLE, WALKING = Value
  }

  class CTR_INFO extends Bundle {
    val seg_id     = UInt(SEG_W.W)
    val el_id      = UInt(11.W)
    val v_group_id = UInt(EMUL_ENC_W.W)
  }

  // ======== Config info ========

  val state      = RegInit(State.IDLE)
  val sb_id      = RegInit(0.U(5.W))
  val base_v_reg = RegInit(0.U(5.W))
  val vl         = RegInit(0.U(9.W))
  val eew_enc    = RegInit(0.U(EEW_ENC_W.W))
  val emul_enc   = RegInit(0.U(EMUL_ENC_W.W))
  val stride     = RegInit(0.U(64.W))
  val seg_count  = RegInit(0.U(SEG_W.W))
  val is_mask    = RegInit(false.B)
  val base_addr  = RegInit(0.U(64.W))
  val dmem_off   = RegInit(0.U(DMEM_ENC.W)) // memory alignment offset
  val dmem_max   = RegInit(0.U(DMEM_ENC.W)) // max elements to fit in DMEM

  // ======== Walking state ========

  val current_seg_id   = RegInit(0.U(SEG_W.W))   // current segment index
  val current_el_id    = RegInit(0.U(11.W))  // current element index
  val current_v_group_id = RegInit(0.U(3.W))   // current vector group
  val current_addr     = RegInit(0.U(64.W))  // current address (incremental)
  val current_ctr      = RegInit(0.U(CTR_WIDTH.W)) // current CTR (counts to vl)

  // ======== Packing Constraints ========
  // elements until DMEM has hit its boundary
  val dmem_constraint = dmem_max - dmem_off
  // elements until the next stride update
  val packing_constraint = seg_count - current_seg_id
  // max constraints (for "wrapping")
  val max_seg_id_met  = (packing_constraint <= dmem_constraint)
  val max_el_id_met   = (current_el_id === ((VLEN_BYTES.U >> eew_enc) - 1.U))
  val max_v_group_met = (current_v_group_id === ((1.U << emul_enc) - 1.U))
  val max_ctr_met     = (current_ctr === (vl - 1.U))

  // ======== Advance CTR based on constraints ========
  val ctr_inc_val = WireInit(0.U(CTR_WIDTH.W))
  
  // check constraints hit (NOTE: multiple can be triggered)
  val dmem_constraint_met    = (dmem_constraint <= packing_constraint)
  val packing_constraint_met = (packing_constraint <= dmem_constraint)

  // increment based on constraint met
  // Segment Packing is smallest
  when (packing_constraint_met) {
    ctr_inc_val := packing_constraint
  // DMEM is smallest
  }.otherwise {
    ctr_inc_val := dmem_constraint
  }


  // ======== Outputs ========

  // ready-valid signals
  io.start.ready := (state === State.IDLE)
  io.load_packet.valid := (state === State.WALKING)

  // packet info
  io.load_packet.addr     := current_addr
  io.load_packet.v_reg    := base_v_reg + (current_seg_id << emul_enc) + current_v_group_id
  io.load_packet.el_id    := current_el_id
  io.load_packet.el_off   := dmem_off
  io.load_packet.el_count := ctr_inc_val
  io.load_packet.sb_id    := sb_id
  io.load_packet.last     := (state === State.WALKING) && (max_ctr_met)

  // ======== State Machine ========

  switch(state) {
    is(State.IDLE) {
      when(io.start.ready && io.start.valid) {
        // -- Input config --
        state       := State.WALKING
        sb_id       := io.start.sb_id
        base_v_reg  := io.start.base_v_reg
        vl          := io.start.vl
        eew_enc     := io.start.eew_enc
        emul_enc    := io.start.emul_enc
        stride      := io.start.stride
        seg_count   := io.start.seg_count
        is_mask     := io.start.is_mask
        base_addr   := io.start.base_addr
        
        // -- Initialize counters --
        current_el_id   := 0.U
        current_seg_id  := 0.U
        current_v_group_id := 0.U
        current_addr    := io.start.base_addr
        current_ctr     := 0.U
        dmem_off        := io.start.base_addr(DMEM_ENC-2, 0) >> io.start.eew_enc
        dmem_max        := (DMEM_BYTES.U >> io.start.eew_enc)
      }
    }
    
    is(State.WALKING) {
      when (io.kill) {
        state := State.IDLE
      }.elsewhen (io.load_packet.ready && io.load_packet.valid) {

        // -- Counter ripple logic --
        // [v_group, el, seg]: [x, x, +1]
        when (!max_seg_id_met) {
          current_seg_id := current_seg_id + ctr_inc_val
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

        // -- Address increment --
        when (max_seg_id_met) {
          current_addr := current_addr + stride
        }

        // -- CTR increment --
        current_ctr := current_ctr + ctr_inc_val

        // -- DMEM offset increment --
        when (max_seg_id_met) {
          val next_addr = (current_addr + stride)
          dmem_off := (next_addr(DMEM_ENC-2, 0)) >> eew_enc // using sample val of addr
        }.elsewhen(dmem_constraint_met) {
          dmem_off := 0.U
        }.otherwise{
          dmem_off := dmem_off + ctr_inc_val
        }

        // -- Last packet check --
        when (io.load_packet.last) {
          state := State.IDLE
        }
        
      }
    }
  }

  // ======== Debug ========

  io.debug.debug_curr_seg_id := current_seg_id
  io.debug.debug_curr_el_id := current_el_id
  io.debug.debug_curr_v_group_id := current_v_group_id
  io.debug.debug_curr_addr := current_addr
  io.debug.debug_curr_ctr := current_ctr
  io.debug.debug_curr_dmem_off := dmem_off
  io.debug.debug_curr_dmem_max := dmem_max
  io.debug.debug_ctr_inc_val := ctr_inc_val
  
  dontTouch(io.debug)
  dontTouch(io.start)
  dontTouch(io.load_packet)
  dontTouch(io.kill)

}


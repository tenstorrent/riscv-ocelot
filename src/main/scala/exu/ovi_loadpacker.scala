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

// Load Packer for OVI
// This module is responsible for packing vector loads into packets for the LSU

// TODO: mask, and negative stride; walker: mask, index

// NOTE: "signal_enc" refers to the signal value encoded as its log2Ceil(value)
// since they are expected to be in powers of 2
class LoadPacker(val VLEN: Int, val DMEM_WIDTH: Int) extends Module {
  // ======== Parameters ========
  val VLEN_BYTES = VLEN/8
  val DMEM_BYTES = DMEM_WIDTH/8
  val DMEM_ENC   = log2Ceil(DMEM_BYTES+1)
  val EEW_ENC_W    = 3
  val EMUL_ENC_W   = 3
  val STRIDE_ENC_W = 3
  val SEG_ENC_W    = 3
  val CTR_WIDTH    = SEG_ENC_W+STRIDE_ENC_W+11+EMUL_ENC_W

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val sb_id       = Input(UInt(5.W))
      val base_v_reg  = Input(UInt(5.W))
      val vl          = Input(UInt(9.W))
      val eew_enc     = Input(UInt(EEW_ENC_W.W))    // encoded in bytes (not bits)
      val emul_enc    = Input(UInt(EMUL_ENC_W.W))
      val stride_enc  = Input(UInt(STRIDE_ENC_W.W))
      val seg_enc     = Input(UInt(SEG_ENC_W.W))
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
      // mask stuff (TODO: add mask info)
    }
    val debug = new Bundle {
      val debug_CTR_seg_id     = Output(UInt(SEG_ENC_W.W))
      val debug_CTR_stride_id  = Output(UInt(STRIDE_ENC_W.W))
      val debug_CTR_el_id      = Output(UInt(11.W))
      val debug_CTR_v_group_id = Output(UInt(EMUL_ENC_W.W))
      val debug_CTR        = Output(UInt(CTR_WIDTH.W))
      val debug_ctr_inc_val = Output(UInt(CTR_WIDTH.W))
      val debug_ctr_inc_case = Output(Vec(3, Bool())) // 0: VL, 1: VREG, 2: DMEM
      val debug_dmem_constraint = Output(UInt(CTR_WIDTH.W))
      val debug_vreg_constraint = Output(UInt(CTR_WIDTH.W))
      val debug_vl_constraint = Output(UInt(CTR_WIDTH.W))
      val debug_sb_id = Output(UInt(5.W))
      val debug_base_v_reg = Output(UInt(5.W))
      val debug_vl = Output(UInt(9.W))
      val debug_eew_enc = Output(UInt(EEW_ENC_W.W))
      val debug_emul_enc = Output(UInt(EMUL_ENC_W.W))
      val debug_stride_enc = Output(UInt(STRIDE_ENC_W.W))
      val debug_seg_enc = Output(UInt(SEG_ENC_W.W))
      val debug_is_mask = Output(Bool())
      val debug_just_el_off = Output(UInt(6.W))
      val debug_dmem_off = Output(UInt(DMEM_ENC.W))
      val debug_dmem_max = Output(UInt(DMEM_ENC.W))
      val debug_seg_mask_off = Output(UInt(SEG_ENC_W.W))
      val debug_seg_mask_width = Output(UInt(SEG_ENC_W.W))
      val debug_stride_mask_off = Output(UInt(STRIDE_ENC_W.W))
      val debug_stride_mask_width = Output(UInt(STRIDE_ENC_W.W))
      val debug_el_mask_off = Output(UInt(11.W))
      val debug_el_mask_width = Output(UInt(11.W))
      val debug_v_group_mask_off = Output(UInt(EMUL_ENC_W.W))
      val debug_v_group_mask_width = Output(UInt(EMUL_ENC_W.W))
    }
  })


  // ======== Definitions ========

  object State extends ChiselEnum {
    val IDLE, PACKING = Value
  }

  class CTR_INFO extends Bundle {
    val seg_id     = UInt((1 << SEG_ENC_W).W)
    val stride_id  = UInt((1 << STRIDE_ENC_W).W)
    val el_id      = UInt(11.W)
    val v_group_id = UInt((1 << EMUL_ENC_W).W)
  }


  // ======== Config info ========

  val state      = RegInit(State.IDLE)
  val sb_id      = RegInit(0.U(5.W))
  val base_v_reg = RegInit(0.U(5.W))
  val vl         = RegInit(0.U(9.W))
  val eew_enc    = RegInit(0.U(EEW_ENC_W.W))
  val emul_enc   = RegInit(0.U(EMUL_ENC_W.W))
  val stride_enc = RegInit(0.U(STRIDE_ENC_W.W))
  val seg_enc    = RegInit(0.U(SEG_ENC_W.W))
  val is_mask    = RegInit(false.B)
  val base_addr  = RegInit(0.U(64.W))
  val dmem_off   = RegInit(0.U(DMEM_ENC.W)) // offset tracking for mem alignment
  val dmem_max   = RegInit(0.U(DMEM_ENC.W)) // max elements to fit in DMEM


  // ======== Packing info ========
  val EEW_CTR = RegInit(0.U(CTR_WIDTH.W)) // single EEW wide slice counter

  // mask offs and widths (to read CTR fields)
  val seg_mask_off      = 0.U
  val seg_mask_width    = seg_enc
  val stride_mask_off   = seg_mask_off + seg_mask_width
  val stride_mask_width = stride_enc
  val el_mask_off       = stride_mask_off + stride_mask_width
  val el_mask_width     = log2Ceil(VLEN_BYTES).U - eew_enc // eq to log2Ceil(VLEN_BYTES >> eew_enc)
  val v_group_mask_off  = el_mask_off + el_mask_width
  val v_group_mask_width = emul_enc

  // split ctr into specific fields
  def split_ctr(ctr: UInt): CTR_INFO = {
    val ctr_info = Wire(new CTR_INFO)
    ctr_info.seg_id     := (ctr >> seg_mask_off)     & ((1.U << seg_mask_width) - 1.U)
    ctr_info.stride_id  := (ctr >> stride_mask_off)  & ((1.U << stride_mask_width) - 1.U)
    ctr_info.el_id      := (ctr >> el_mask_off)      & ((1.U << el_mask_width) - 1.U)
    ctr_info.v_group_id := (ctr >> v_group_mask_off) & ((1.U << v_group_mask_width) - 1.U)
    ctr_info
  }


  // ======== Packing Constraints ========

  // elements until DMEM has hit its boundary
  val dmem_constraint = dmem_max - dmem_off             // check distance to boundary

  // elements until the next EMUL_FIELD in CTR
  val vreg_constraint = (
    (EEW_CTR & ~((1.U << v_group_mask_off) - 1.U)) +    // clear all fields except the EMUL_FIELD
    (1.U << v_group_mask_off)                           // add one to the EMUL_FIELD (to get to next group_id)
  ) - EEW_CTR                                           // take distance of value from current ctr

  // elements until VL is reached in CTR
  val vl_constraint   = (
    (vl << el_mask_off) |                               // EMUL_FIELD/EL_ID_FIELD: vl
    (((1.U << (seg_mask_width)) - 1.U) << seg_mask_off) // SEG_ID_FIELD:           last seg_id (all 1s)
  ) - EEW_CTR                                           // take distance of value from current ctr


  // ======== Advance CTR based on constraints ========
  val ctr_inc_val = WireInit(0.U(CTR_WIDTH.W))
  
  // check constraints hit (NOTE: multiple can be triggered)
  val dmem_constraint_met = (dmem_constraint <= vl_constraint) && (dmem_constraint <= vreg_constraint)
  val vreg_constraint_met = (vreg_constraint <= vl_constraint) && (vreg_constraint <= dmem_constraint)
  val vl_constraint_met   = (vl_constraint <= vreg_constraint) && (vl_constraint <= dmem_constraint)
  
  // increment based on constraint met
  // VL is smallest
  when (vl_constraint_met) {
    ctr_inc_val := vl_constraint
  // VREG is smallest
  }.elsewhen (vreg_constraint_met) {
    ctr_inc_val := vreg_constraint
  // DMEM is smallest
  }.otherwise {
    ctr_inc_val := dmem_constraint
  }

  // ======== look past unwanted stride offsets ========
  val el_off = Mux(
    (split_ctr(EEW_CTR).stride_id === 0.U), // stride_id = 0 means within the valid EEW chunk (so no need to look past)
    0.U, (
      (EEW_CTR & ~((1.U << el_mask_off) - 1.U)) + // clear all fields under the EL_ID_FIELD
      (1.U << el_mask_off)                        // add one to the EL_ID_FIELD (to get to next el_id)
    ) - EEW_CTR                                   // take distance of value from current ctr
  )
  val valid_start_ctr = EEW_CTR + el_off


  // ======== Outputs ========

  // ready-valid signals
  io.start.ready := (state === State.IDLE)
  io.load_packet.valid := (state === State.PACKING)

  // packet info
  io.load_packet.addr   := base_addr + (EEW_CTR << eew_enc) // base + (EEW_CTR * EEW)
  io.load_packet.v_reg  := base_v_reg + (split_ctr(valid_start_ctr).seg_id << emul_enc) + split_ctr(valid_start_ctr).v_group_id // base + [(seg_id * total_groups) + group_id]
  io.load_packet.el_id  := split_ctr(valid_start_ctr).el_id
  io.load_packet.el_off := el_off + dmem_off // offset to valid strided element + offset to align dmem
  io.load_packet.el_count := 0.U // io.load_packet.el_count = <use ctr_inc_val and case> TODO: need to figure this out
  io.load_packet.sb_id  := sb_id
  io.load_packet.last   := (state === State.PACKING) && (vl_constraint_met)


  // ======== State Machine ========

  switch(state) {
    is(State.IDLE) {
      when(io.start.ready && io.start.valid) {
        // input config
        state      := State.PACKING
        sb_id      := io.start.sb_id
        base_v_reg := io.start.base_v_reg
        vl         := io.start.vl
        eew_enc    := io.start.eew_enc
        emul_enc   := io.start.emul_enc
        stride_enc := io.start.stride_enc
        seg_enc    := io.start.seg_enc
        is_mask    := io.start.is_mask
        base_addr  := io.start.base_addr
        EEW_CTR    := 0.U
        // calculated config (mem alignment)
        // val initialNegativeStrideElemOffset = DMEM_ENC.U - io.start.eew_enc - 1.U
        // val initialPositiveStrideElemOffset = io.start.base_addr(DMEM_ENC-2, 0) >> io.start.eew_enc
        // val currentOffset := Mux (
        //   (io.isStride && io.stride(63)),
        //   (initialNegativeStrideElemOffset - initialPositiveStrideElemOffset), 
        //   (initialPositiveStrideElemOffset)
        // )
        dmem_off := io.start.base_addr(DMEM_ENC-2, 0) >> io.start.eew_enc
        dmem_max := DMEM_BYTES.U >> io.start.eew_enc
      }
    }
    is (State.PACKING) {
      when (io.kill) {
        state := State.IDLE
      }.elsewhen (io.load_packet.ready && io.load_packet.valid) {
        // increment counter
        EEW_CTR := EEW_CTR + ctr_inc_val
        // wrap increment dmem offset
        dmem_off := Mux(
          ((dmem_off + ctr_inc_val) === dmem_max),
          (0.U),                   // reset to 0
          (dmem_off + ctr_inc_val) // increment
        )
      }
      when (io.load_packet.last) {
        state := State.IDLE
      }
    }
  }

  // ======== Debugging ========

  io.debug.debug_CTR_seg_id     := split_ctr(EEW_CTR).seg_id
  io.debug.debug_CTR_stride_id  := split_ctr(EEW_CTR).stride_id
  io.debug.debug_CTR_el_id      := split_ctr(EEW_CTR).el_id
  io.debug.debug_CTR_v_group_id := split_ctr(EEW_CTR).v_group_id
  io.debug.debug_CTR            := EEW_CTR
  io.debug.debug_ctr_inc_val    := ctr_inc_val
  io.debug.debug_ctr_inc_case(0) := vl_constraint_met
  io.debug.debug_ctr_inc_case(1) := vreg_constraint_met
  io.debug.debug_ctr_inc_case(2) := dmem_constraint_met
  io.debug.debug_dmem_constraint := dmem_constraint
  io.debug.debug_vreg_constraint := vreg_constraint
  io.debug.debug_vl_constraint   := vl_constraint
  io.debug.debug_sb_id           := sb_id
  io.debug.debug_base_v_reg      := base_v_reg
  io.debug.debug_vl              := vl
  io.debug.debug_eew_enc         := eew_enc
  io.debug.debug_emul_enc        := emul_enc
  io.debug.debug_stride_enc      := stride_enc
  io.debug.debug_seg_enc         := seg_enc
  io.debug.debug_is_mask         := is_mask
  io.debug.debug_just_el_off     := el_off
  io.debug.debug_dmem_off        := dmem_off
  io.debug.debug_dmem_max        := dmem_max
  io.debug.debug_seg_mask_off    := seg_mask_off
  io.debug.debug_seg_mask_width  := seg_mask_width
  io.debug.debug_stride_mask_off := stride_mask_off
  io.debug.debug_stride_mask_width := stride_mask_width
  io.debug.debug_el_mask_off     := el_mask_off
  io.debug.debug_el_mask_width   := el_mask_width
  io.debug.debug_v_group_mask_off := v_group_mask_off
  io.debug.debug_v_group_mask_width := v_group_mask_width
  // dontTouch(io.start)
  dontTouch(io.load_packet)
  dontTouch(io.kill)
  dontTouch(io.debug)
  
}

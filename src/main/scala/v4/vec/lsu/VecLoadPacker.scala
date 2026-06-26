//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------
package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.exu._
import boom.v4.util._

import chisel3.dontTouch // this is for debugging purposes

// Load Packer for OVI
// This module is responsible for packing vector loads into packets for the LSU

// Segments are packed and elements are packed
// no index or mask support (use walker or skipper for that)

// NOTE: "signal_enc" refers to the signal value encoded as its log2Ceil(value)
// since they are expected to be in powers of 2
class VecLoadPacker(implicit p: Parameters)
extends BoomModule with VecLsConstants {
  // ======== Parameters ========
  val CTR_WIDTH = (((1<<SEG_ENC_W)-1)+((1<<STRIDE_ENC_W)-1)+EL_ID_W+((1<<EMUL_ENC_W)-1))

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = Flipped(DecoupledIO(new ConfigInfo)) // expects latched config info (DO NOT CHANGE DURING FSM)
    val use_seg_constraint = Input(Bool())
    // mask interface (for masked loads)
    val mask = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val mask_data = Input(UInt(MASK_W.W))
    }
    // kill signal (used to reset the FSM)
    val kill = Input(Bool())
    // load process FSM outputs (packet info)
    val load_packet = DecoupledIO(new VecLoadNop)
    // status signal
    val gen_active = Output(Bool())
  })


  // ======== Definitions ========

  object State extends ChiselEnum {
    val IDLE, PACKING = Value
  }

  class CTR_INFO extends Bundle {
    val seg_id     = UInt(((1<<SEG_ENC_W)-1).W)
    val stride_id  = UInt(((1<<STRIDE_ENC_W)-1).W)
    val el_id      = UInt(EL_ID_W.W)
    val v_group_id = UInt(((1<<EMUL_ENC_W)-1).W)
  }


  // ======== Config info ========

  val state      = RegInit(State.IDLE)
  val sb_id      = io.start.bits.uop.rob_idx
  val base_v_reg = io.start.bits.base_v_reg
  val vl         = io.start.bits.vl
  val vstart     = io.start.bits.vstart
  val eew_enc    = io.start.bits.eew_enc
  val emul_enc   = io.start.bits.emul_enc
  val stride_enc = io.start.bits.stride_enc
  val seg_enc    = io.start.bits.seg_enc
  val stride_dir = io.start.bits.stride_dir
  val is_mask    = io.start.bits.is_mask
  val base_addr  = io.start.bits.base_addr
  val use_seg_constraint = io.use_seg_constraint

  // ======== Packing info ========
  val EEW_CTR = RegInit(0.U(CTR_WIDTH.W)) // single EEW wide slice counter
  val current_mask_off = RegInit(0.U(MASK_W_SIZE.W))  // mask offset tracking for mask data
  val current_mask_data = RegInit(0.U(MASK_W.W))           // mask data will be shifted as used
  val dmem_off   = RegInit(0.U(DMEM_ENC.W)) // offset tracking for mem alignment
  val dmem_max   = RegInit(0.U(DMEM_ENC.W)) // max elements to fit in DMEM

  // mask offs and widths (to read CTR fields: <msb> [GROUP_ID, EL_ID, STRIDE_ID, SEG_ID] <lsb>)
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

  // elements until mask buffer is used up
  val mask_constraint = (
    (EEW_CTR & ~((1.U << (el_mask_off+MASK_W_SIZE.U(6.W))) - 1.U)) + // clear bits under the mask offset of EL_ID_FIELD
    (1.U << (el_mask_off+MASK_W_SIZE.U(6.W)))                        // add one to the mask part of the EL_ID_FIELD
  ) - EEW_CTR                                                        // take distance of value from current ctr

  // elements until the next EMUL_FIELD in CTR
  val vreg_constraint = (
    (EEW_CTR & ~((1.U << v_group_mask_off) - 1.U)) +    // clear all fields except the EMUL_FIELD
    (1.U << v_group_mask_off)                           // add one to the EMUL_FIELD (to get to next group_id)
  ) - EEW_CTR                                           // take distance of value from current ctr

  // elements until the next EL_ID_FIELD in CTR (to force and not pack across segments to reduce hardware complexity)
  val seg_constraint = (
    (EEW_CTR & ~((1.U << el_mask_off) - 1.U)) +         // clear all fields upto the EL_ID_FIELD
    (1.U << el_mask_off)                                // add one to the EL_ID_FIELD (to get to next el_id)
  ) - EEW_CTR                                           // take distance of value from current ctr

  // elements until VL is reached in CTR
  val vl_constraint   = (
    ((vl - 1.U) << el_mask_off) +                     // EMUL_FIELD/EL_ID_FIELD: vl
    (((1.U << seg_mask_width) - 1.U) << seg_mask_off) // SEG_ID_FIELD: last seg_id (all 1s)
  ) + 1.U - EEW_CTR                                   // take distance of value from current ctr

  // ======== Advance CTR based on constraints ========
  val ctr_inc_val = WireInit(0.U(CTR_WIDTH.W))

  // check constraints hit (NOTE: multiple can be triggered due to "less OR eq")
  val dmem_constraint_met = (dmem_constraint <= vl_constraint) && (dmem_constraint <= vreg_constraint) && ((dmem_constraint <= mask_constraint) || !is_mask) && ((dmem_constraint <= seg_constraint) || !use_seg_constraint)
  val vreg_constraint_met = (vreg_constraint <= vl_constraint) && (vreg_constraint <= dmem_constraint) && ((vreg_constraint <= mask_constraint) || !is_mask) && ((vreg_constraint <= seg_constraint) || !use_seg_constraint)
  val vl_constraint_met   = (vl_constraint <= vreg_constraint) && (vl_constraint <= dmem_constraint)   && ((vl_constraint <= mask_constraint)   || !is_mask) && ((vl_constraint <= seg_constraint)   || !use_seg_constraint)
  val seg_constraint_met  = (seg_constraint <= vl_constraint)  && (seg_constraint <= vreg_constraint)  && ((seg_constraint <= dmem_constraint)  || !is_mask) && ((seg_constraint <= mask_constraint) &&  use_seg_constraint)
  val mask_constraint_met = (mask_constraint <= vl_constraint) && (mask_constraint <= vreg_constraint) && ((mask_constraint <= dmem_constraint) &&  is_mask) && ((mask_constraint <= seg_constraint) || !use_seg_constraint)

  // increment based on constraint met
  // VL is smallest
  when (vl_constraint_met) {
    ctr_inc_val := vl_constraint
  // VREG is smallest
  }.elsewhen (vreg_constraint_met) {
    ctr_inc_val := vreg_constraint
  // DMEM is smallest
  }.elsewhen (dmem_constraint_met) {
    ctr_inc_val := dmem_constraint
  // SEG is smallest
  }.elsewhen (seg_constraint_met) {
    ctr_inc_val := seg_constraint
  // MASK is smallest
  }.elsewhen (mask_constraint_met) {
    ctr_inc_val := mask_constraint
  }

  // ======== look past unwanted stride offsets ========
  val el_off = Mux(
    (split_ctr(EEW_CTR).stride_id === 0.U), // stride_id = 0 means within the valid EEW chunk (so no need to look past)
    0.U, (
      (EEW_CTR & ~((1.U << el_mask_off) - 1.U)) + // clear all fields under the EL_ID_FIELD
      (1.U << el_mask_off)                        // add one to the EL_ID_FIELD (to get to next el_id)
    ) - EEW_CTR                                   // take distance of value from current ctr
  )
  val ctr_past_off = EEW_CTR + el_off


  // ======== Outputs ========

  // ready-valid signals
  val need_next_mask    = (is_mask && mask_constraint_met && !vl_constraint_met)
  io.start.ready       := (state === State.IDLE) && (!is_mask || io.mask.valid)
  io.mask.ready        := ((state === State.PACKING) && need_next_mask && (io.load_packet.ready)) ||
                          ((state === State.IDLE)   && (is_mask && io.start.valid))
  io.load_packet.valid := (state === State.PACKING) && (!need_next_mask || io.mask.valid)
  io.gen_active        := (state === State.PACKING)

  // ======== Counting elements ========
  // the value is pretty much the increment amount with the stride field ripped out but with some exceptions to this rule
  val el_count = Wire(UInt(7.W))

  // when starting from a valid stride, preserve the seg_id (pretend like we're starting from the base then remove at end)
  when (split_ctr(EEW_CTR).stride_id === 0.U) {
    val inc_from_base_seg = ctr_inc_val + split_ctr(EEW_CTR).seg_id
    dontTouch(inc_from_base_seg)
    // if ending at a valid stride, just keep the extra seg_id
    when (split_ctr(inc_from_base_seg).stride_id === 0.U) {
      el_count := (((inc_from_base_seg & ~((1.U << el_mask_off) - 1.U)) >> stride_mask_width) | split_ctr(inc_from_base_seg).seg_id) - split_ctr(EEW_CTR).seg_id
    // if ending at an invalid stride, pretend like we completed everything in the final segment
    }.otherwise {
      el_count := (((inc_from_base_seg & ~((1.U << el_mask_off) - 1.U)) >> stride_mask_width) + (1.U << seg_mask_width)) - split_ctr(EEW_CTR).seg_id
    }
  // when starting from an invalid stride, start from the next valid stride
  }.otherwise {
    val inc_past_off = ctr_inc_val - el_off
    dontTouch(inc_past_off)
    // check if if that next valid stride is even a part of the packet
    when (ctr_inc_val <= el_off) {
      el_count := 0.U
    // if ending at a valid stride, just keep the extra seg_id
    }.elsewhen (split_ctr(inc_past_off).stride_id === 0.U) {
      el_count := (((inc_past_off & ~((1.U << el_mask_off) - 1.U)) >> stride_mask_width) | split_ctr(inc_past_off).seg_id)
    // if ending at an invalid stride, pretend like we completed everything in the final segment
    }.otherwise {
      el_count := (((inc_past_off & ~((1.U << el_mask_off) - 1.U)) >> stride_mask_width) + (1.U << seg_mask_width))
    }
  }


  val addr_off = (Mux(
    stride_dir,
    -(((EEW_CTR & ~((1.U<<seg_mask_width)-1.U)) - split_ctr(EEW_CTR).seg_id) << eew_enc).asSInt, // negate everything except the seg_id (have to double negate because of type issues)
    ((EEW_CTR) << eew_enc).asSInt
  ))

  // packet info
  io.load_packet.bits.addr   := (base_addr.asSInt + addr_off).asUInt // base + (EEW_CTR * EEW)
  io.load_packet.bits.v_reg  := base_v_reg + (split_ctr(ctr_past_off).seg_id << emul_enc) + split_ctr(ctr_past_off).v_group_id // base + [(seg_id * total_groups) + group_id]
  io.load_packet.bits.el_id  := split_ctr(ctr_past_off).el_id
  io.load_packet.bits.el_off := el_off + dmem_off // offset to valid strided element + offset to align dmem (remember this exists since address is forcefully aligned later)
  io.load_packet.bits.el_count   := el_count
  io.load_packet.bits.mask_data  := current_mask_data & ((1.U << el_count) - 1.U)
  io.load_packet.bits.mask_valid := is_mask
  io.load_packet.bits.is_fake    := (el_count === 0.U) || (is_mask && ((current_mask_data & ((1.U << el_count) - 1.U)) === 0.U))
  io.load_packet.bits.misaligned := ((base_addr & ((1.U << eew_enc) - 1.U)) =/= 0.U)
  io.load_packet.bits.last   := (state === State.PACKING) && (vl_constraint_met)
  io.load_packet.bits.uop    := io.start.bits.uop
  io.load_packet.bits.dir    := stride_dir && !use_seg_constraint
  io.load_packet.bits.is_fof := io.start.bits.is_fof
  // Caracal physical-PRN fields (filled by downstream remap; default 0 here)
  io.load_packet.bits.pdst        := 0.U
  io.load_packet.bits.pdst_member := 0.U

  // ======== Vstart Handling ========
  // need to re-align dmem offset to the new address after vstart
  val vstart_EEW_CTR = (vstart << el_mask_off)

  val vstart_addr = (base_addr.asSInt + (Mux(
    stride_dir,
    -((vstart_EEW_CTR) << eew_enc).asSInt,
    ((vstart_EEW_CTR) << eew_enc).asSInt
  ))).asUInt

  // ======== State Machine ========

  switch(state) {
    // IDLE STATE
    is(State.IDLE) {
      when(io.start.fire) {

        // -- CTR config --
        state   := State.PACKING
        EEW_CTR := vstart_EEW_CTR

        // -- Initialize mask info --
        current_mask_off := 0.U
        current_mask_data := io.mask.mask_data

        // -- Initialize mem alignment info --
        val high_off = (((1<<(ADDR_BREAK))-1).U - vstart_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from end of DMEM (high) to base_addr
        val low_off  = (vstart_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from start of DMEM (low) to base_addr
        dmem_off := Mux(stride_dir && !use_seg_constraint, high_off, low_off)
        dmem_max := (DMEM_BYTES.U >> eew_enc)
      }
    }
    // PACKING STATE
    is (State.PACKING) {
      when (io.kill) {
        state := State.IDLE
      }.elsewhen (io.load_packet.fire) {

        // -- Increment counter --
        EEW_CTR := EEW_CTR + ctr_inc_val

        // -- Wrap increment dmem offset --
        when (dmem_constraint_met) {
          dmem_off := 0.U
        }.otherwise {
          dmem_off := dmem_off + ctr_inc_val
        }

        // -- Wrap increment mask offset --
        val next_mask_off = ((EEW_CTR + ctr_inc_val) >> el_mask_off) & (MASK_W-1).U
        when (mask_constraint_met) {
          current_mask_off := 0.U
          current_mask_data := io.mask.mask_data
        }.otherwise {
          current_mask_off := next_mask_off
          current_mask_data := current_mask_data >> (next_mask_off - current_mask_off)
        }

        // -- Reset when last packet is reached --
        when (io.load_packet.bits.last) {
          state := State.IDLE
        }

      }
    }
  }

  // ======== Debugging ========

  dontTouch(io.start)
  dontTouch(io.load_packet)
  dontTouch(io.kill)
  dontTouch(seg_constraint)
  dontTouch(seg_constraint_met)
  dontTouch(el_count)

}

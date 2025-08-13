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

// Load Packer for OVI
// This module is responsible for packing vector loads into packets for the LSU

// Segments are packed and elements are packed
// no index or mask support (use walker or skipper for that)

// NOTE: "signal_enc" refers to the signal value encoded as its log2Ceil(value)
// since they are expected to be in powers of 2
class LoadPacker(override val VLEN: Int, override val DMEM_WIDTH: Int)(implicit p: Parameters)
extends Module with VecLSGenConstants {
  // ======== Parameters ========
  val CTR_WIDTH = (((1<<SEG_ENC_W)-1)+((1<<STRIDE_ENC_W)-1)+EL_ID_W+((1<<EMUL_ENC_W)-1))

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = Flipped(DecoupledIO(new ConfigInfo(VLEN, DMEM_WIDTH))) // expects latched config info (DO NOT CHANGE DURING FSM)
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
    val load_packet = DecoupledIO(new LoadPacket(VLEN, DMEM_WIDTH))
    // status signal
    val gen_active = Output(Bool())
    // debug signals
    val debug = new Bundle {
      val debug_CTR_seg_id     = Output(UInt(SEG_ENC_W.W))
      val debug_CTR_stride_id  = Output(UInt(STRIDE_ENC_W.W))
      val debug_CTR_el_id      = Output(UInt(EL_ID_W.W))
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
    val seg_id     = UInt(((1<<SEG_ENC_W)-1).W)
    val stride_id  = UInt(((1<<STRIDE_ENC_W)-1).W)
    val el_id      = UInt(EL_ID_W.W)
    val v_group_id = UInt(((1<<EMUL_ENC_W)-1).W)
  }


  // ======== Config info ========

  val state      = RegInit(State.IDLE)
  val sb_id      = io.start.bits.sb_id
  val base_v_reg = io.start.bits.base_v_reg
  val vl         = io.start.bits.vl
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

  // some math to figure out how many elements to load (comments below this file for explanation)
  // val inc_past_off = ctr_inc_val - el_off
  // val el_count = PriorityMux(Seq(
  //   (ctr_inc_val <= el_off)
  //     -> (0.U),
  //   (split_ctr(inc_past_off).stride_id =/= 0.U)
  //     -> (((inc_past_off & ~((1.U << el_mask_off) - 1.U)) >> stride_mask_width) + (1.U << seg_mask_width)),
  //   (split_ctr(inc_past_off).stride_id === 0.U)
  //     -> (((inc_past_off & ~((1.U << el_mask_off) - 1.U)) >> stride_mask_width) | split_ctr(inc_past_off).seg_id)
  // ))

  val addr_off = (EEW_CTR << eew_enc).asSInt

  // packet info
  io.load_packet.bits.addr   := (base_addr.asSInt + Mux(stride_dir, -addr_off, addr_off)).asUInt // base + (EEW_CTR * EEW)
  io.load_packet.bits.v_reg  := base_v_reg + (split_ctr(ctr_past_off).seg_id << emul_enc) + split_ctr(ctr_past_off).v_group_id // base + [(seg_id * total_groups) + group_id]
  io.load_packet.bits.el_id  := split_ctr(ctr_past_off).el_id
  io.load_packet.bits.el_off := el_off + dmem_off // offset to valid strided element + offset to align dmem (remember this exists since address is forcefully aligned later)
  io.load_packet.bits.el_count   := el_count
  io.load_packet.bits.sb_id  := sb_id
  io.load_packet.bits.mask_data  := current_mask_data & ((1.U << el_count) - 1.U)
  io.load_packet.bits.mask_valid := is_mask
  io.load_packet.bits.is_fake    := (el_count === 0.U) || (is_mask && ((current_mask_data & ((1.U << el_count) - 1.U)) === 0.U))
  io.load_packet.bits.misaligned := ((base_addr & ((1.U << eew_enc) - 1.U)) =/= 0.U)
  io.load_packet.bits.last   := (state === State.PACKING) && (vl_constraint_met)
  io.load_packet.bits.uop    := io.start.bits.uop
  io.load_packet.bits.dir    := stride_dir

  // ======== State Machine ========

  switch(state) {
    is(State.IDLE) {
      when(io.start.fire) {
        // -- CTR config --
        state   := State.PACKING
        EEW_CTR := 0.U

        // -- Initialize mask info --
        current_mask_off := 0.U
        current_mask_data := io.mask.mask_data

        // -- Initialize mem alignment info --
        val high_off = (((1<<(ADDR_BREAK))-1).U - base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from end of DMEM (high) to base_addr
        val low_off  = (base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from start of DMEM (low) to base_addr
        dmem_off := Mux(stride_dir, high_off, low_off)
        dmem_max := (DMEM_BYTES.U >> eew_enc)
      }
    }
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
  dontTouch(io.start)
  dontTouch(io.load_packet)
  dontTouch(io.kill)
  dontTouch(io.debug)
  dontTouch(seg_constraint)
  dontTouch(seg_constraint_met)
  dontTouch(el_count)
  
}

/*
  val inc_past_off = ctr_inc_val - el_off
  val el_count = Wire(UInt(7.W))

  // if the invalid strided elements from the start of the packet take up the entire ctr_inc_val,
  // then there is nothing to be considered in this packet
  // (you are simply setting the el_count to 0)

  when (ctr_inc_val <= el_off) {
    el_count := 0.U

  // take the quotient of number of elements (group and id field) divided by the strides (because only one valid stride id "00" out of all the increments matter)
  // if the increment of ctr from the first valid stride id ends up at an invalid stride ignore the additional seg_ids packed with it
  // (you are simply pulling out the stride field from the increment of ctr from the first valid stride id and setting the seg_id to 0)

  }.elsewhen (split_ctr(inc_past_off).stride_id =/= 0.U) {
    el_count := ((inc_past_off & ~((1.U << el_mask_off) - 1.U)) >> stride_mask_width)
  
  // take the quotient of number of elements (group and id field) divided by the strides (because only one valid stride id "00" out of all the increments matter)
  // if the increment of ctr from the first valid stride id ends up at a valid stride make sure to include the seg_id
  // (you are simply pulling out the stride field from the increment of ctr from the first valid stride id)
  
  }.otherwise {
    el_count := (((inc_past_off & ~((1.U << el_mask_off) - 1.U)) >> stride_mask_width) | split_ctr(inc_past_off).seg_id)
  }
*/
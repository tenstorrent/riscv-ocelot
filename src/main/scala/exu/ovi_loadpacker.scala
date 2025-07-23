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

// TODO: none of the width-sizes are correct. Need to fix.
class LoadPacker(val VLEN: Int) extends Module {
  val vlen_bytes = VLEN/8
  val io = IO(new Bundle {
    val start = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val sb_id       = Input(UInt(6.W))
      val base_v_reg  = Input(UInt(6.W))
      val vl          = Input(UInt(log2Ceil(VLEN+1).W))
      val eew_enc     = Input(UInt(2.W)) // encoded in bytes (not bits)
      val emul_enc    = Input(UInt(3.W))
      val stride_enc  = Input(UInt(6.W))
      val seg_enc     = Input(UInt(6.W))
      val is_mask     = Input(Bool())
      val base_addr   = Input(UInt(64.W))
    }
    val kill = Input(Bool())
    val load_packet = new Bundle {
      val ready = Input(Bool())
      val valid = Output(Bool())
      val addr     = Output(UInt(64.W))
      val v_reg    = Output(UInt(6.W))
      val el_id    = Output(UInt(6.W))
      val el_off   = Output(UInt(6.W))
      val el_count = Output(UInt(6.W))
      val sb_id    = Output(UInt(6.W))
      val last     = Output(Bool())
      // mask stuff
    }
  })

  // ===== Definitions =====
  object State extends ChiselEnum {
    val IDLE, PACKING = Value
  }

  object CTR_INC_CASE extends ChiselEnum {
    val VLEN, VREG, VL, NONE = Value
  }

  class CTR_INFO extends Bundle {
    val seg_id     = UInt(64.W)
    val stride_id  = UInt(64.W)
    val el_id      = UInt(64.W)
    val v_group_id = UInt(64.W)
  }

  // ===== Config info =====
  val state      = RegInit(State.IDLE)
  val sb_id      = RegInit(0.U(6.W))
  val base_v_reg = RegInit(0.U(6.W))
  val vl         = RegInit(0.U(log2Ceil(VLEN+1).W))
  val eew_enc    = RegInit(0.U(2.W))
  val emul_enc   = RegInit(0.U(3.W))
  val stride_enc = RegInit(0.U(6.W))
  val seg_enc    = RegInit(0.U(6.W))
  val is_mask    = RegInit(false.B)
  val base_addr  = RegInit(0.U(64.W))


  // ===== Packing info =====
  val EEW_CTR = RegInit(0.U(6.W)) // single EEW wide slice counter
  // mask offs and widths
  val seg_mask_off      = 0.U
  val seg_mask_width    = seg_enc
  val stride_mask_off   = seg_mask_off + seg_mask_width
  val stride_mask_width = stride_enc
  val el_mask_off       = stride_mask_off + stride_mask_width
  val el_mask_width     = log2Ceil(vlen_bytes).U - eew_enc // eq to log2Ceil(vlen_bytes >> eew_enc)
  val v_group_mask_off  = el_mask_off + el_mask_width
  val v_group_mask_width = emul_enc
  // split ctr into info
  def split_ctr(ctr: UInt): CTR_INFO = {
    val ctr_info = Wire(new CTR_INFO)
    ctr_info.seg_id     := (ctr >> seg_mask_off)     & ((1.U << seg_mask_width) - 1.U)
    ctr_info.stride_id  := (ctr >> stride_mask_off)  & ((1.U << stride_mask_width) - 1.U)
    ctr_info.el_id      := (ctr >> el_mask_off)      & ((1.U << el_mask_width) - 1.U)
    ctr_info.v_group_id := (ctr >> v_group_mask_off) & ((1.U << v_group_mask_width) - 1.U)
    ctr_info
  }

  // ===== Packing Constraints =====
  // VLEN / EEW
  val vlen_constraint = (vlen_bytes.U >> eew_enc)
  // {next group_id (group_id+1), first everything else} - CTR 
  val vreg_constraint = (EEW_CTR & ~((1.U << v_group_mask_off) - 1.U)) + (1.U << v_group_mask_off) - EEW_CTR
  // {last group_id (EMUL), last el_id (final_el_id), first stride_off ('0'), last seg_id ('1')} - CTR + 1
  val final_el_id     = vl & ((1.U << el_mask_width) - 1.U) // final el_id = vl % ids_per_vlen
  val vl_constraint   = ((emul_enc << v_group_mask_off) | (final_el_id << el_mask_off) | ((1.U << stride_mask_off) - 1.U)) - EEW_CTR + 1.U

  // ===== Advance CTR based on constraints =====
  val ctr_inc_val = WireInit(0.U(6.W))
  val ctr_inc_case = WireInit(CTR_INC_CASE.NONE)
  // find min with priority: VL > VREG > VLEN
  // VL is smallest
  when ((vl_constraint <= vreg_constraint) &&
        (vl_constraint <= vlen_constraint)) {
    ctr_inc_val := vl_constraint
    ctr_inc_case := CTR_INC_CASE.VL
  // VREG is smallest
  }.elsewhen (vreg_constraint <= vlen_constraint) {
    ctr_inc_val := vreg_constraint
    ctr_inc_case := CTR_INC_CASE.VREG
  // VLEN is smallest
  }.otherwise {
    ctr_inc_val := vlen_constraint
    ctr_inc_case := CTR_INC_CASE.VLEN
  }

  // ===== look past unwanted stride offsets =====
  val el_off = Mux(
    (split_ctr(EEW_CTR).stride_id =/= 0.U),
    (1.U << v_group_mask_off) - (EEW_CTR & ((1.U << v_group_mask_off) - 1.U)), 0.U)
  val valid_start_ctr = EEW_CTR + el_off


  // ===== Outputs =====
  // ready-valid signals
  io.start.ready := (state === State.IDLE)
  io.load_packet.valid := (state === State.PACKING)
  // packet info
  io.load_packet.addr   := base_addr + (1.U << (EEW_CTR + eew_enc + 3.U)) // +3 since eew is in bytes
  io.load_packet.v_reg  := base_v_reg + (split_ctr(valid_start_ctr).seg_id << emul_enc) + split_ctr(valid_start_ctr).v_group_id
  io.load_packet.el_id  := split_ctr(valid_start_ctr).el_id
  io.load_packet.el_off := el_off
  io.load_packet.el_count := 0.U // io.load_packet.el_count = <use ctr_inc_val and case> TODO: need to figure this out
  io.load_packet.sb_id  := sb_id
  io.load_packet.last   := (ctr_inc_case === CTR_INC_CASE.VL)


  // ===== State Machine =====
  switch(state) {
    is(State.IDLE) {
      when(io.start.ready && io.start.valid) {
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
      }
    }
    is (State.PACKING) {
      when (io.kill) {
        state := State.IDLE
      }.elsewhen (io.load_packet.ready && io.load_packet.valid) {
        EEW_CTR := EEW_CTR + ctr_inc_val
      }
      when (io.load_packet.last) {
        state := State.IDLE
      }
    }
  }

  // ===== Debugging =====
  dontTouch(io.start)
  dontTouch(io.load_packet)

}

//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Index Generator (VecIdxGen) -- Step 11b (indexed LS)
//------------------------------------------------------------------------------
//
// VecIdxGen is the producer side of the per-element INDEX stream consumed by the
// Vector LS AGEN's Walker FSM (`io.index`: index_value/mask_bit/last_index, with
// a ready/valid handshake). For an indexed load/store (vluxei/vloxei/vsuxei/
// vsoxei) the Walker computes each element's address as `base + index[el]`, where
// index[el] is an EEW-wide signed offset read from the index VECTOR register
// (vs2 -> pvs2_grp). This module reads `pvs2_grp` one 256b member at a time via a
// registered-address VRF read port (same shape as VecDgen/VecRegFile), buffers
// the member, and presents one signed index per element on demand, advancing on
// the Walker's `ready`. The per-element mask bit (v0) is paired in alongside.
//
// M1 scope: index EEW == data EEW == e64 (one 64b index per VRF lane, 4 lanes per
// 256b member). Sub-64b index EEW (sign-extension of e8/e16/e32 indices) is a
// later refinement; here the 64b slice is taken directly. vl <= MASK_W elements.

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

/** Per-element index stream: VRF index-group member -> signed index + mask bit. */
class VecIdxGen(implicit p: Parameters) extends BoomModule with VecLsConstants
{
  val io = IO(new Bundle {
    // start: which indexed instruction + its (latched) config (uop.pvs2_grp, vl, eew).
    val start    = Flipped(DecoupledIO(new ConfigInfo))
    val kill     = Input(Bool())
    // VRF read port (registered address) -- reads a pvs2_grp member (index vector).
    val vrf_read = new Bundle {
      val req_valid = Output(Bool())
      val req_addr  = Output(UInt(vecPregSz.W))   // a pvs2_grp member PRN
      val resp_data = Input(UInt(vecVLen.W))      // 256b group member
    }
    // per-element mask bits (v0[MASK_W-1:0]); only used when the op is masked.
    val mask_in  = Input(UInt(MASK_W.W))
    // index stream to the Walker (drives io.mask_idx in core.scala):
    //  valid/index_value/mask_bit/last_index out; ready in (Walker consumed).
    val idx = new Bundle {
      val valid       = Output(Bool())
      val index_value = Output(SInt(MASK_W.W))
      val mask_bit    = Output(Bool())
      val last_index  = Output(Bool())
      val ready       = Input(Bool())
    }
    val active   = Output(Bool())
  })

  object State extends ChiselEnum {
    val sIdle, sFetch, sCap, sStream = Value
  }
  val state = RegInit(State.sIdle)

  val start_q  = Reg(new ConfigInfo)
  val mask_q   = Reg(UInt(MASK_W.W))                  // latched v0 (valid only at start)
  val buf      = Reg(UInt(vecVLen.W))                 // current 256b index member
  val el       = RegInit(0.U(vecVLSz.W))              // global element index (0..vl-1)
  val member   = RegInit(0.U(vecSplitSz.W))           // which pvs2_grp member

  // e64 index: byte offset of element `el` within its 256b member, and which member.
  val eew_enc      = start_q.eew_enc
  val byte_in_mem  = (el << eew_enc) & (VLEN_BYTES - 1).U     // (el*idx_bytes) mod 32
  val cur_member   = (el << eew_enc) >> log2Ceil(VLEN_BYTES).U // (el*idx_bytes) / 32
  val is_masked    = !start_q.uop.v_unmasked

  // ---- defaults ----
  io.start.ready        := state === State.sIdle
  io.vrf_read.req_valid := false.B
  io.vrf_read.req_addr  := start_q.uop.pvs2_grp(member)
  io.active             := state =/= State.sIdle
  io.idx.valid          := false.B
  // signed index: low MASK_W bits of the selected lane (e64 -> exactly the index).
  io.idx.index_value    := (buf >> (byte_in_mem << 3.U))(MASK_W - 1, 0).asSInt
  io.idx.mask_bit       := Mux(is_masked, mask_q(el(log2Ceil(MASK_W) - 1, 0)), true.B)
  io.idx.last_index     := (el === (start_q.vl - 1.U))

  switch (state) {
    is (State.sIdle) {
      when (io.start.fire) {
        start_q := io.start.bits
        mask_q  := io.mask_in
        el      := 0.U
        member  := 0.U
        state   := State.sFetch
      }
    }
    is (State.sFetch) {
      // drive the registered read of pvs2_grp(member); data lands next cycle.
      io.vrf_read.req_valid := true.B
      state                 := State.sCap
    }
    is (State.sCap) {
      // resp_data is now the member -> capture; then stream its indices.
      io.vrf_read.req_valid := true.B
      buf                   := io.vrf_read.resp_data
      state                 := State.sStream
    }
    is (State.sStream) {
      io.idx.valid := true.B
      when (io.idx.ready) {
        when (io.idx.last_index) {
          state := State.sIdle                 // whole index vector streamed
        } .otherwise {
          val next_el = el + 1.U
          el := next_el
          // advance to the next member when the next element crosses the boundary.
          when (((next_el << eew_enc) >> log2Ceil(VLEN_BYTES).U) =/= cur_member) {
            member := member + 1.U
            state  := State.sFetch
          }
        }
      }
    }
  }

  when (io.kill) {
    state  := State.sIdle
    el     := 0.U
    member := 0.U
  }
}

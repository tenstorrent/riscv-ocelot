//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Store-DATA Generator (VecDgen) -- Step 10 (DORMANT)
//------------------------------------------------------------------------------
//
// VecDgen is the producer side of the `VecDgenDataIF` handshake consumed by the
// store Skipper/Walker FSMs. The FSM requests bytes (`read_bytes`/`read_all`);
// VecDgen presents ELEN-wide (DMEM_WIDTH=64b) slices of the store data, sourced
// from a 256b (VLEN) vector group member read out of the VRF (or a scalar src).
//
// It reads one `pvs3_grp` member at a time via a registered-address VRF read
// port (drive addr, data arrives next cycle -- shape mirrors VecRegFile's read
// port so Step-11 core.scala can wire them directly), buffers the 256b member
// in a register, and presents 64b slices on demand. A byte cursor tracks the
// position within the current 256b buffer; a member cursor (0..7) tracks which
// group member is live. When a member is exhausted the next member is fetched.
//
// DORMANT in Step 10: the core ties the consumer ready false, so this never
// actually fires. Priorities: elaborates clean, every port driven, no comb
// loops (all DGEN->FSM outputs derive from REGISTERED buffer/cursor state, never
// combinationally from the incoming `read_bytes`/`read_all`). Byte-accounting is
// best-effort / Step-11-refinable.

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

/** Vector store-data generator: VRF/scalar group member -> ELEN-wide store-data slices. */
class VecDgen(implicit p: Parameters) extends BoomModule with VecLsConstants
{
  val io = IO(new Bundle {
    // start: which store instruction + its (latched) config. Decoupled so the
    // producer knows when a new store's data stream begins.
    val start    = Flipped(DecoupledIO(new ConfigInfo))
    val kill     = Input(Bool())
    // VRF read port -- mirrors VecRegFile's read port (registered-address read:
    // drive a physical reg number, get the 256b group member next cycle). The
    // `req_valid` is an extra Step-11 hint; VecRegFile itself has no valid.
    val vrf_read = new Bundle {
      val req_valid = Output(Bool())
      val req_addr  = Output(UInt(vecPregSz.W))   // physical vector reg number (a pvs3_grp member)
      val resp_data = Input(UInt(VLEN.W))         // 256b group member (concatenated 64b lanes)
    }
    // scalar store-data source (e.g. a future scalar-sourced store path); tie-able.
    val scalar_data = Input(UInt(xLen.W))
    // producer side of the FSM handshake (we DRIVE valid_bytes/data, READ read_bytes/read_all).
    val vdb_data = Flipped(new VecDgenDataIF)
    val active   = Output(Bool())
  })

  // ---------------------------------------------------------------------------
  // State machine: IDLE waits for `start`; FETCH issues a VRF read and waits one
  // cycle for the registered read data; STREAM presents 64b slices until the
  // member (and then the group) is exhausted.
  // ---------------------------------------------------------------------------
  object State extends ChiselEnum {
    val sIdle, sFetch, sCap, sStream = Value
  }
  val state = RegInit(State.sIdle)

  // Latched config for the in-flight store.
  val start_q = Reg(new ConfigInfo)

  // 256b member buffer (registered read-data path -- written the cycle after a
  // FETCH request) and its cursors.
  val buf        = Reg(UInt(VLEN.W))                 // current 256b group member
  val byte_cur   = RegInit(0.U(log2Ceil(VLEN_BYTES + 1).W)) // byte offset within `buf`
  val member_cur = RegInit(0.U(vecSplitSz.W))        // which pvs3_grp member (0..7)
  val bytes_done = RegInit(0.U(vecVLSz.W + 3))       // total store bytes streamed so far

  dontTouch(state)
  dontTouch(byte_cur)
  dontTouch(member_cur)
  dontTouch(bytes_done)

  // Total bytes this store must stream = vl * EEW-bytes. Completion is by TOTAL
  // bytes, NOT by a fixed member count: a store may span a partial last 256b
  // member (vl not a whole-member multiple), and must finish exactly when its
  // data is exhausted -- otherwise VecDgen stalls active and the NEXT store
  // reuses this store's stale start_q/pvs3 (garbage store data in back-to-back
  // stores, e.g. a strip-mined memcpy). Members advance on 256b boundaries.
  val total_bytes = (start_q.vl << start_q.eew_enc)

  // ---------------------------------------------------------------------------
  // Defaults (every output driven on every path).
  // ---------------------------------------------------------------------------
  io.start.ready          := state === State.sIdle
  io.vrf_read.req_valid   := false.B
  io.vrf_read.req_addr    := start_q.uop.pvs3_grp(member_cur(2, 0))  // member 0..7
  io.active               := state =/= State.sIdle
  io.vdb_data.valid_bytes := 0.U
  io.vdb_data.data        := 0.U

  // Bytes remaining in the current 256b buffer, clamped to one ELEN chunk
  // (DMEM_BYTES). Derived purely from the REGISTERED `byte_cur` -- never from the
  // incoming read_bytes/read_all (avoids a comb loop).
  val bytes_left   = (VLEN_BYTES.U - byte_cur)
  val avail_bytes  = Mux(bytes_left > DMEM_BYTES.U, DMEM_BYTES.U, bytes_left)

  // Current 64b slice: shift the buffer down by the byte cursor and take the low
  // ELEN bits. Registered-state-derived, so no feedback from the request.
  val cur_slice    = (buf >> (byte_cur << 3.U))(DMEM_WIDTH - 1, 0)

  // ---------------------------------------------------------------------------
  // FSM
  // ---------------------------------------------------------------------------
  switch (state) {
    is (State.sIdle) {
      when (io.start.fire) {
        start_q    := io.start.bits
        byte_cur   := 0.U
        member_cur := 0.U
        bytes_done := 0.U
        state      := State.sFetch
      }
    }

    is (State.sFetch) {
      // Drive the registered-address read for the current member. The VRF read is
      // registered (data = vrf[RegNext(addr)]), so the data is NOT valid until the
      // NEXT cycle -- capture it in sCap, not here (reading resp_data now would
      // return the previous address's stale value).
      io.vrf_read.req_valid := true.B
      byte_cur              := 0.U
      state                 := State.sCap
    }

    is (State.sCap) {
      // addr held since sFetch -> resp_data is now vrf[pvs3_grp(member_cur)].
      io.vrf_read.req_valid := true.B
      buf                   := io.vrf_read.resp_data
      state                 := State.sStream
    }

    is (State.sStream) {
      io.vdb_data.valid_bytes := avail_bytes
      io.vdb_data.data        := cur_slice

      // Consume on an FSM request. `read_all` consumes the whole available chunk;
      // otherwise consume min(read_bytes, avail_bytes).
      val req_bytes = Mux(io.vdb_data.read_all, avail_bytes,
                          Mux(io.vdb_data.read_bytes > avail_bytes, avail_bytes, io.vdb_data.read_bytes))
      when (io.vdb_data.read_all || io.vdb_data.read_bytes =/= 0.U) {
        val next_byte = byte_cur + req_bytes
        val next_done = bytes_done + req_bytes
        bytes_done := next_done
        when (next_done >= total_bytes) {
          state := State.sIdle              // whole store's data streamed out
        } .elsewhen (next_byte >= VLEN_BYTES.U) {
          // Current 256b member exhausted -- advance to the next group member.
          member_cur := member_cur + 1.U
          byte_cur   := 0.U
          state      := State.sFetch
        } .otherwise {
          byte_cur := next_byte
        }
      }
    }
  }

  // ---------------------------------------------------------------------------
  // kill: drop back to IDLE and reset cursors regardless of current state.
  // ---------------------------------------------------------------------------
  when (io.kill) {
    state      := State.sIdle
    byte_cur   := 0.U
    member_cur := 0.U
    bytes_done := 0.U
  }
}

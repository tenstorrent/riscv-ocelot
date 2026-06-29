//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector LSU -- unit-stride load beat engine (Step 11a.2)
//------------------------------------------------------------------------------
//
// VecLSU owns the per-beat memory loop for an OoO unit-stride vector load.
// VecAgenStage1(load) cracks the load into a serial stream of `VecLoadNop`
// beats, each describing one DMEM_WIDTH (64b) dcache access plus where its bytes
// land in the renamed vector register group. VecLSU drives those beats, one
// outstanding at a time, onto a dedicated `vec_dmem` port that the scalar LSU
// forwards into its dcache request mux at LOWEST priority (so the scalar
// will_fire schedule is preserved by construction -- see
// docs_caracal/step11a2-lsu-mechanism.md).
//
// Serial / single-outstanding model: issue beat k, wait for its response, then
// issue k+1. The current-beat register uniquely identifies the in-flight
// response, so no per-response tag is needed; the scalar LSU routes responses
// back here by `resp.uop.is_vec`.
//
// A vle occupies ONE entry in the existing scalar LDQ (VDecode sets uses_ldq for
// vector loads); the cracked beats are this module's separate queue, never LDQ
// slots. Completion on the last beat is dual:
//   - ld_done (ldq_idx) -> the LSU writes the placeholder LDQ entry's
//     executed/succeeded ONCE, so it retires at the ROB head.
//   - clr_rob (rob_idx) -> rob.vec_clr_bsy clears rob_bsy (a RT_VEC load has no
//     iresp writeback to clear it), and the LCB emits one VecGroupDone (prn) ->
//     vec-rename / vec-issue wakeups clear the dest-group busy bits.
//
// M1 scope: unit-stride `vle` only, fully-aligned base (el_off == 0, each beat
// is exactly one 64b dest lane). Misaligned / strided / segmented / masked /
// store paths are later steps.

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.constants.MemoryOpConstants

import boom.v4.common._
import boom.v4.vec.rename.VecGroupDone

/** One serial vector dcache beat request: VecLSU -> scalar LSU. */
class VecDmemReq(implicit p: Parameters) extends BoomBundle with VecLsConstants
{
  val addr    = UInt(coreMaxAddrBits.W)   // 8B-aligned beat address (physical, M1 bare-mode)
  val data    = UInt(coreDataBits.W)      // store data (loads ignore)
  val uop     = new MicroOp()             // beat uop (is_vec set, mem_cmd, mem_size=3)
  val is_load = Bool()
}

/** One vector dcache beat response / nack: scalar LSU -> VecLSU. */
class VecDmemResp(implicit p: Parameters) extends BoomBundle with VecLsConstants
{
  val data = UInt(coreDataBits.W)         // 64b load data
}

/** The vec_dmem sub-bundle as seen FROM THE SCALAR LSU (`io.core.vec_dmem`).
  * VecLSU drives the flipped view. Decoupled req in; Valid resp/nack out;
  * ld_done in (VecLSU -> LSU: write the placeholder LDQ entry's exec/succ once). */
class VecDmemIO(implicit p: Parameters) extends BoomBundle with VecLsConstants
{
  val req     = Flipped(DecoupledIO(new VecDmemReq))    // VecLSU -> LSU
  val resp    = Valid(new VecDmemResp)                  // LSU -> VecLSU
  val nack    = Valid(new VecDmemResp)                  // LSU -> VecLSU
  val ld_done = Flipped(Valid(UInt((1 + ldqAddrSz).W))) // VecLSU -> LSU (ldq idx)
}

class VecLSU(implicit p: Parameters) extends BoomModule with VecLsConstants
  with MemoryOpConstants
{
  val io = IO(new Bundle {
    // cracked unit-stride beats from VecAgenStage1(load)
    val load_nop = Flipped(DecoupledIO(new VecLoadNop))
    // dedicated vector dcache port (flipped LSU view)
    val dmem     = Flipped(new VecDmemIO)
    // VecRegFile write port (one lane per beat)
    val vrf_write = Valid(new Bundle {
      val addr = UInt(vecPregSz.W)
      val data = UInt(vecVLen.W)
      val mask = UInt((vecVLen / 64).W)
    })
    // group-done -> vec rename / issue wakeups (prn-based, from the LCB)
    val group_done = Valid(new VecGroupDone)
    // ROB busy clear -> rob.vec_clr_bsy (rob_idx-based) for the completing vle
    val clr_rob    = Valid(UInt(robAddrSz.W))
    val kill       = Input(Bool())
    val busy       = Output(Bool())
  })

  // ---- coalescing buffer: maps each beat to a VRF lane write + group_done ----
  val lcb = Module(new VecLoadCoalescingBuffer)

  object State extends ChiselEnum {
    val sIdle, sReq, sResp, sFin = Value
  }
  val state      = RegInit(State.sIdle)
  val cur        = Reg(new VecLoadNop)       // latched current beat
  val grp_active = RegInit(false.B)          // a vle group is in progress

  // byte-width per element (EEW), encoded log2 in the uop.
  def eewEnc(uop: MicroOp): UInt = uop.v_eew(1, 0)

  // ---- defaults ----
  io.load_nop.ready      := false.B
  io.dmem.req.valid      := false.B
  io.dmem.req.bits       := DontCare
  io.clr_rob.valid       := false.B
  io.clr_rob.bits        := cur.uop.rob_idx
  io.dmem.ld_done.valid  := false.B
  io.dmem.ld_done.bits   := cur.uop.ldq_idx

  // ---- LCB start: latch dest group descriptor on the first beat of a group ----
  val start_grp = io.load_nop.fire && !grp_active
  lcb.io.start.valid     := start_grp
  lcb.io.start.bits.prn  := io.load_nop.bits.uop.pvdest_grp
  lcb.io.start.bits.mask := io.load_nop.bits.uop.pvdest_grp_mask
  lcb.io.kill            := io.kill

  // ---- LCB beat: driven on a real response OR on a fake/bypass last beat ----
  lcb.io.beat.valid          := false.B
  lcb.io.beat.bits           := DontCare
  lcb.io.beat.bits.is_fake   := true.B

  // place an LCB beat from a VecLoadNop + a 64b data payload
  def driveBeat(nop: VecLoadNop, data: UInt, isFake: Bool): Unit = {
    val ee = eewEnc(nop.uop)
    lcb.io.beat.valid         := true.B
    lcb.io.beat.bits.data     := data
    lcb.io.beat.bits.pdst     := nop.pdst
    lcb.io.beat.bits.dst_byte := nop.el_id << ee                // byte offset within the 256b member
    lcb.io.beat.bits.src_off  := nop.el_off                     // source byte offset within the 64b beat
    lcb.io.beat.bits.nbytes   := nop.el_count << ee
    lcb.io.beat.bits.is_fake  := isFake
    lcb.io.beat.bits.last     := nop.last
  }

  // ---- request payload for the current beat ----
  val beat_uop = WireInit(cur.uop)
  beat_uop.is_vec     := true.B
  beat_uop.uses_ldq   := false.B            // no scalar LDQ entry; ROB-busy completion
  beat_uop.uses_stq   := false.B
  beat_uop.dst_rtype  := RT_VEC
  beat_uop.mem_cmd    := M_XRD               // doubleword load
  beat_uop.mem_size   := 3.U                 // 64b
  io.dmem.req.bits.addr    := cur.addr & ~(7.U(coreMaxAddrBits.W))   // align down to 8B
  io.dmem.req.bits.data    := 0.U
  io.dmem.req.bits.uop     := beat_uop
  io.dmem.req.bits.is_load := true.B

  // ---- FSM ----
  // NOTE: group_done / clr_rob / the LCB beat are driven ONLY from the latched
  // `cur` (a Reg), never combinationally from io.load_nop. The vector issue unit
  // feeds load_nop (via decode/AGEN) and consumes group_done on its wakeup port,
  // so a combinational load_nop -> group_done path would form a cycle. Even the
  // fake/bypass beat is finalized one cycle later in sFin, off `cur`.
  switch (state) {
    is (State.sIdle) {
      io.load_nop.ready := !io.kill
      when (io.load_nop.fire) {
        cur        := io.load_nop.bits
        grp_active := true.B
        // is_fake (all-masked / el_count==0 / vl==0 bypass): no memory access,
        // finalize from the registered beat in sFin. Otherwise issue the beat.
        state := Mux(io.load_nop.bits.is_fake, State.sFin, State.sReq)
      }
    }

    is (State.sFin) {
      // registered fake/bypass beat: notify the LCB so a fake LAST finalizes.
      driveBeat(cur, 0.U, isFake = true.B)
      when (cur.last) {
        io.clr_rob.valid      := true.B
        io.clr_rob.bits       := cur.uop.rob_idx
        io.dmem.ld_done.valid := true.B
        io.dmem.ld_done.bits  := cur.uop.ldq_idx
        grp_active            := false.B
      }
      state := State.sIdle
    }

    is (State.sReq) {
      io.dmem.req.valid := !io.kill
      when (io.kill) {
        state := State.sIdle
      } .elsewhen (io.dmem.req.fire) {
        state := State.sResp
      }
    }

    is (State.sResp) {
      when (io.kill) {
        state := State.sIdle
      } .elsewhen (io.dmem.nack.valid) {
        // retry the same beat (serial: only this beat is outstanding)
        state := State.sReq
      } .elsewhen (io.dmem.resp.valid) {
        driveBeat(cur, io.dmem.resp.bits.data, isFake = false.B)
        when (cur.last) {
          io.clr_rob.valid      := true.B
          io.clr_rob.bits       := cur.uop.rob_idx
          io.dmem.ld_done.valid := true.B
          io.dmem.ld_done.bits  := cur.uop.ldq_idx
          grp_active            := false.B
        }
        state := State.sIdle
      }
    }
  }

  when (io.kill) {
    state      := State.sIdle
    grp_active := false.B
  }

  // ---- VRF write + group-done from the coalescing buffer ----
  io.vrf_write  := lcb.io.vrf_write
  io.group_done := lcb.io.group_done

  io.busy := grp_active || (state =/= State.sIdle)
}

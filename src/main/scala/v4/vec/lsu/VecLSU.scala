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
  val req       = Flipped(DecoupledIO(new VecDmemReq))    // VecLSU -> LSU
  val resp      = Valid(new VecDmemResp)                  // LSU -> VecLSU (load data)
  val nack      = Valid(new VecDmemResp)                  // LSU -> VecLSU
  val store_ack = Valid(new VecDmemResp)                  // LSU -> VecLSU (store committed to D$)
  val ld_done   = Flipped(Valid(UInt((1 + ldqAddrSz).W))) // VecLSU -> LSU (ldq idx, write exec/succ once)
  val st_done   = Flipped(Valid(UInt((1 + stqAddrSz).W))) // VecLSU -> LSU (stq idx, mark succeeded once)
}

class VecLSU(implicit p: Parameters) extends BoomModule with VecLsConstants
  with MemoryOpConstants
{
  val io = IO(new Bundle {
    // cracked unit-stride beats from VecAgenStage1(load) / VecAgenStage1(store)
    val load_nop  = Flipped(DecoupledIO(new VecLoadNop))
    val store_nop = Flipped(DecoupledIO(new VecStoreNop))
    // dedicated vector dcache port (flipped LSU view)
    val dmem     = Flipped(new VecDmemIO)
    // VecRegFile write port (one lane per beat)
    val vrf_write = Valid(new Bundle {
      val addr = UInt(vecPregSz.W)
      val data = UInt(vecVLen.W)
      val mask = UInt((vecVLen / 64).W)
    })
    // VecRegFile read port for the undisturbed copy: a masked load first copies
    // the OLD group (stale_pvdest_grp) into the new pdst group, so masked-off
    // lanes keep their architectural value (mask-undisturbed) -- with renaming
    // the new physical reg is otherwise uninitialized. Registered-address read.
    val vrf_read = new Bundle {
      val req_addr  = Output(UInt(vecPregSz.W))
      val resp_data = Input(UInt(vecVLen.W))
    }
    // group-done -> vec rename / issue wakeups (prn-based, from the LCB)
    val group_done = Valid(new VecGroupDone)
    // ROB busy clear -> rob.vec_clr_bsy (rob_idx-based) for the completing vle
    val clr_rob    = Valid(UInt(robAddrSz.W))
    val kill       = Input(Bool())
    val busy       = Output(Bool())
  })

  // ---- coalescing buffer: maps each beat to a VRF lane write + group_done ----
  val lcb = Module(new VecLoadCoalescingBuffer)

  // Load states (sReq/sResp) and store states (sSReq/sSAck); sFin finalizes a
  // fake/bypass beat (load or store) one cycle later off the registered beat.
  object State extends ChiselEnum {
    val sIdle, sCopyRd, sCopyWr, sReq, sResp, sFin, sSReq, sSAck = Value
  }
  val state      = RegInit(State.sIdle)
  val cur        = Reg(new VecLoadNop)       // latched current LOAD beat
  val curs       = Reg(new VecStoreNop)      // latched current STORE beat
  val grp_active = RegInit(false.B)          // a vec mem group is in progress
  val is_store   = RegInit(false.B)          // current group is a store (vs load)
  val copy_member = RegInit(0.U(vecSplitSz.W)) // member being copied (undisturbed phase)

  val nLanes      = vecVLen / 64
  // # of valid group members to copy (contiguous EMUL group -> popcount of mask).
  val copy_count  = PopCount(cur.uop.pvdest_grp_mask)

  // byte-width per element (EEW), encoded log2 in the uop.
  def eewEnc(uop: MicroOp): UInt = uop.v_eew(1, 0)

  // ---- defaults ----
  io.load_nop.ready      := false.B
  io.store_nop.ready     := false.B
  io.dmem.req.valid      := false.B
  io.dmem.req.bits       := DontCare
  io.clr_rob.valid       := false.B
  io.clr_rob.bits        := cur.uop.rob_idx
  io.dmem.ld_done.valid  := false.B
  io.dmem.ld_done.bits   := cur.uop.ldq_idx
  io.dmem.st_done.valid  := false.B
  io.dmem.st_done.bits   := curs.uop.stq_idx
  io.vrf_read.req_addr   := cur.uop.stale_pvdest_grp(copy_member)

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

  // ---- request payload: LOAD beat in sReq, STORE beat in sSReq ----
  val ld_uop = WireInit(cur.uop)
  ld_uop.is_vec     := true.B
  ld_uop.uses_ldq   := false.B              // nano-op: no scalar LDQ entry
  ld_uop.uses_stq   := false.B
  ld_uop.dst_rtype  := RT_VEC
  ld_uop.mem_cmd    := M_XRD                 // doubleword load
  ld_uop.mem_size   := 3.U                   // 64b

  val st_uop = WireInit(curs.uop)
  st_uop.is_vec     := true.B
  st_uop.uses_ldq   := false.B
  st_uop.uses_stq   := false.B              // nano-op: not the placeholder STQ entry
  st_uop.dst_rtype  := RT_X
  st_uop.mem_cmd    := M_XWR                 // doubleword store
  st_uop.mem_size   := 3.U                   // 64b

  when (state === State.sSReq) {
    io.dmem.req.bits.addr    := curs.addr & ~(7.U(coreMaxAddrBits.W))
    io.dmem.req.bits.data    := curs.data
    io.dmem.req.bits.uop     := st_uop
    io.dmem.req.bits.is_load := false.B
  } .otherwise {
    io.dmem.req.bits.addr    := cur.addr & ~(7.U(coreMaxAddrBits.W))   // align down to 8B
    io.dmem.req.bits.data    := 0.U
    io.dmem.req.bits.uop     := ld_uop
    io.dmem.req.bits.is_load := true.B
  }

  // ---- FSM ----
  // NOTE: group_done / clr_rob / the LCB beat are driven ONLY from the latched
  // `cur`/`curs` (Regs), never combinationally from io.load_nop/store_nop. The
  // vector issue units feed the nops (via decode/AGEN) and consume group_done on
  // their wakeup ports, so a combinational nop -> group_done path would form a
  // cycle. Even fake/bypass beats finalize one cycle later in sFin, off the Reg.
  switch (state) {
    is (State.sIdle) {
      // Accept a LOAD beat (priority) or, if none, a STORE beat. Loads/stores are
      // serial here (fu_ready gates issue on !busy), so at most one is in flight.
      io.load_nop.ready  := !io.kill
      io.store_nop.ready := !io.kill && !io.load_nop.valid
      when (io.load_nop.fire) {
        cur        := io.load_nop.bits
        grp_active := true.B
        is_store   := false.B
        // Copy the OLD group first when masked-off OR tail lanes won't be written
        // (mask-/tail-undisturbed), so those lanes keep their architectural value
        // rather than the uninitialized new physical reg. One-shot at group start;
        // a full unmasked load writes every lane, so no copy (and no stale read).
        val needs_copy = (!io.load_nop.bits.uop.v_unmasked) || io.load_nop.bits.tail_undist
        when (needs_copy && !grp_active) {
          copy_member := 0.U
          state       := State.sCopyRd
        } .otherwise {
          state := Mux(io.load_nop.bits.is_fake, State.sFin, State.sReq)
        }
      } .elsewhen (io.store_nop.fire) {
        curs       := io.store_nop.bits
        grp_active := true.B
        is_store   := true.B
        state := Mux(io.store_nop.bits.is_fake, State.sFin, State.sSReq)
      }
    }

    is (State.sCopyRd) {
      // drive the registered read of stale_pvdest_grp(copy_member); data lands
      // next cycle (sCopyWr). req_addr is the default (stable across both states).
      when (io.kill) { state := State.sIdle }
      .otherwise     { state := State.sCopyWr }
    }

    is (State.sCopyWr) {
      // resp_data = old member; write it whole (all lanes) to the new pdst member.
      // (The write itself is driven in the vrf_write mux below.)
      when (io.kill) {
        state := State.sIdle
      } .elsewhen (copy_member === (copy_count - 1.U)) {
        // group fully copied -> now place the latched first beat.
        state := Mux(cur.is_fake, State.sFin, State.sReq)
      } .otherwise {
        copy_member := copy_member + 1.U
        state       := State.sCopyRd
      }
    }

    is (State.sFin) {
      // registered fake/bypass beat. A load also notifies the LCB so a fake LAST
      // finalizes the group; a store has no VRF write.
      when (!is_store) {
        // (load fake) -- cur holds the load nop
        driveBeat(cur, 0.U, isFake = true.B)
        when (cur.last) {
          io.clr_rob.valid      := true.B
          io.clr_rob.bits       := cur.uop.rob_idx
          io.dmem.ld_done.valid := true.B
          io.dmem.ld_done.bits  := cur.uop.ldq_idx
          grp_active            := false.B
        }
      } .otherwise {
        // (store fake/bypass) -- curs holds the store nop
        when (curs.last) {
          io.clr_rob.valid      := true.B
          io.clr_rob.bits       := curs.uop.rob_idx
          io.dmem.st_done.valid := true.B
          io.dmem.st_done.bits  := curs.uop.stq_idx
          grp_active            := false.B
        }
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
        state := State.sReq                  // retry the same (single-outstanding) beat
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

    is (State.sSReq) {
      io.dmem.req.valid := !io.kill
      when (io.kill) {
        state := State.sIdle
      } .elsewhen (io.dmem.req.fire) {
        state := State.sSAck
      }
    }

    is (State.sSAck) {
      when (io.kill) {
        state := State.sIdle
      } .elsewhen (io.dmem.nack.valid) {
        state := State.sSReq                 // retry the same store beat
      } .elsewhen (io.dmem.store_ack.valid) {
        when (curs.last) {
          // last store beat acked: clear the vse's rob_bsy and mark its STQ
          // placeholder entry succeeded (so it retires at the ROB head).
          io.clr_rob.valid      := true.B
          io.clr_rob.bits       := curs.uop.rob_idx
          io.dmem.st_done.valid := true.B
          io.dmem.st_done.bits  := curs.uop.stq_idx
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

  // ---- VRF write: the undisturbed copy (sCopyWr) writes a whole old member to
  // the new pdst member (all lanes); otherwise the LCB's per-beat lane write. ----
  when (state === State.sCopyWr) {
    io.vrf_write.valid     := !io.kill
    io.vrf_write.bits.addr := cur.uop.pvdest_grp(copy_member)
    io.vrf_write.bits.data := io.vrf_read.resp_data
    io.vrf_write.bits.mask := ~0.U(nLanes.W)            // all lanes (full member copy)
  } .otherwise {
    io.vrf_write := lcb.io.vrf_write
  }
  io.group_done := lcb.io.group_done

  io.busy := grp_active || (state =/= State.sIdle)
}

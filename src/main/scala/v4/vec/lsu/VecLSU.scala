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
  val data    = UInt(coreDataBits.W)      // 64b load data
  val beat_id = UInt(4.W)                 // which in-flight beat this responds to (Phase 1)
}

/** Phase 1: per-in-flight-beat descriptor kept in VecLSU. Holds the request info (so
  * a nacked beat can be re-issued) plus the LCB placement fields, indexed by beat id.
  * A beat's dcache response (matched by beat_id) reads this to drive the LCB. */
class VecLoadBeatDesc(implicit p: Parameters) extends BoomBundle with VecLsConstants
{
  val valid    = Bool()                   // slot occupied (beat issued, not yet done)
  val inflight = Bool()                   // false => needs (re)issue (fresh or nacked)
  val addr     = UInt(64.W)               // 8B-aligned beat address (for re-issue)
  val uop      = new MicroOp()            // beat uop (carries vec_beat_id, mem fields)
  // LCB placement (see VecLoadCoalescingBuffer): where the returned bytes land.
  val pdst     = UInt(vecPregSz.W)
  val dst_byte = UInt(log2Ceil(VLEN_BYTES + 1).W)
  val src_off  = UInt(6.W)
  val nbytes   = UInt(log2Ceil(VLEN_BYTES + 1).W)
  val is_fake  = Bool()
}

/** The vec_dmem sub-bundle as seen FROM THE SCALAR LSU (`io.core.vec_dmem`).
  * VecLSU drives the flipped view. Decoupled req in; Valid resp/nack out;
  * ld_done in (VecLSU -> LSU: write the placeholder LDQ entry's exec/succ once). */
class VecDmemIO(implicit p: Parameters) extends BoomBundle with VecLsConstants
{
  val req       = Flipped(DecoupledIO(new VecDmemReq))    // VecLSU -> LSU (stores + primary load beat)
  val resp      = Valid(new VecDmemResp)                  // LSU -> VecLSU (load data)
  val nack      = Valid(new VecDmemResp)                  // LSU -> VecLSU
  val store_ack = Valid(new VecDmemResp)                  // LSU -> VecLSU (store committed to D$)
  val ld_done   = Flipped(Valid(UInt((1 + ldqAddrSz).W))) // VecLSU -> LSU (ldq idx, write exec/succ once)
  val st_done   = Flipped(Valid(UInt((1 + stqAddrSz).W))) // VecLSU -> LSU (stq idx, mark succeeded once)
  // Phase 2 (dual-dynamic): a SECOND LOAD-beat lane. VecLSU issues a second beat/
  // cycle on req2; the LSU fires it into an idle scalar dcache pipe opportunistically
  // (lowest priority, never perturbs scalar). resp2/nack2 carry a second vector
  // response/nack the same cycle. Present only when vecMemWidth>1.
  val req2      = if (vecMemWidth > 1) Some(Flipped(DecoupledIO(new VecDmemReq))) else None
  val resp2     = if (vecMemWidth > 1) Some(Valid(new VecDmemResp)) else None
  val nack2     = if (vecMemWidth > 1) Some(Valid(new VecDmemResp)) else None
}

class VecLSU(implicit p: Parameters) extends BoomModule with VecLsConstants
  with MemoryOpConstants
{
  val io = IO(new Bundle {
    // cracked unit-stride beats from VecAgenStage1(load) / VecAgenStage1(store)
    val load_nop  = Flipped(DecoupledIO(new VecLoadNop))
    // Phase 2 (dual-dynamic): a second cracked load beat/cycle from the AGEN fast path.
    val load_nop2 = if (vecMemWidth > 1) Some(Flipped(DecoupledIO(new VecLoadNop))) else None
    val store_nop = Flipped(DecoupledIO(new VecStoreNop))
    // dedicated vector dcache port (flipped LSU view)
    val dmem     = Flipped(new VecDmemIO)
    // VecRegFile write port (per-byte write enable)
    val vrf_write = Valid(new Bundle {
      val addr = UInt(vecPregSz.W)
      val data = UInt(vecVLen.W)
      val mask = UInt((vecVLen / 8).W)             // per-BYTE write enable
    })
    // Phase 2 (dual-dynamic): a second VRF write port for a distinct-member second
    // load beat (the LCB merges same-member beats onto port 0). Present only when
    // vecMemWidth>1; wired to VecRegFile write port 2 in core.scala.
    val vrf_write2 = if (vecMemWidth > 1) Some(Valid(new Bundle {
      val addr = UInt(vecPregSz.W)
      val data = UInt(vecVLen.W)
      val mask = UInt((vecVLen / 8).W)
    })) else None
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

  // Load: sCopyRd/sCopyWr do the undisturbed group copy; sLoadIssue is the Phase-1
  // multi-outstanding beat engine (issue up to vecLoadMaxInflight beats, place
  // responses out-of-order via the descriptor table). Store: sSReq/sSAck (single-
  // outstanding); sFin finalizes a fake/bypass STORE beat.
  object State extends ChiselEnum {
    val sIdle, sCopyRd, sCopyWr, sLoadIssue, sFin, sSReq, sSAck = Value
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

  // ---- Phase 1: multi-outstanding LOAD descriptor table ----
  // Up to vecLoadMaxInflight load beats outstanding on the single vec_dmem port. A
  // beat's (possibly out-of-order) response is matched back by vec_beat_id = slot idx.
  // At vecLoadMaxInflight==1 this reduces to the original serial one-beat behavior.
  val maxInflight = vectorParams.vecLoadMaxInflight
  val desc     = RegInit(VecInit(Seq.fill(maxInflight)(0.U.asTypeOf(new VecLoadBeatDesc))))
  val saw_last = RegInit(false.B)            // consumed the AGEN's last load beat of the group

  val slot_free   = VecInit(desc.map(!_.valid))
  val has_free    = slot_free.asUInt.orR
  val free_id     = PriorityEncoder(slot_free)
  // Phase 2 (dual-dynamic): a SECOND free slot for accepting beat1 the same cycle.
  val free2_mask  = VecInit(slot_free.zipWithIndex.map { case (f, i) => f && (i.U =/= free_id) })
  val has_2free   = free2_mask.asUInt.orR
  val free_id2    = PriorityEncoder(free2_mask)
  val all_done    = !desc.map(_.valid).reduce(_ || _)          // no outstanding load beats
  val need_issue  = VecInit(desc.map(d => d.valid && !d.inflight))
  val issue_valid = need_issue.asUInt.orR
  val issue_id    = PriorityEncoder(need_issue)
  // Phase 2 (dual-dynamic): a SECOND distinct beat to issue the same cycle on req2.
  // The next-lowest need_issue slot (excluding the primary issue_id) -> the two
  // beats are always different descriptors, so their `inflight` sets never conflict.
  val need_issue2  = VecInit(need_issue.zipWithIndex.map { case (n, i) => n && (i.U =/= issue_id) })
  val issue_valid2 = need_issue2.asUInt.orR
  val issue_id2    = PriorityEncoder(need_issue2)

  // Fill a descriptor from a cracked load beat: LCB placement + re-issue info.
  def fillDesc(nop: VecLoadNop): VecLoadBeatDesc = {
    val d  = Wire(new VecLoadBeatDesc)
    val ee = eewEnc(nop.uop)
    val u  = WireInit(nop.uop)
    u.is_vec := true.B; u.uses_ldq := false.B; u.uses_stq := false.B
    u.dst_rtype := RT_VEC; u.mem_cmd := M_XRD; u.mem_size := 3.U
    d.valid    := true.B
    d.inflight := false.B
    d.addr     := nop.addr
    d.uop      := u
    d.pdst     := nop.pdst
    d.dst_byte := nop.el_id << ee
    d.src_off  := nop.el_off << ee    // element->byte (see the old driveBeat note)
    d.nbytes   := nop.el_count << ee
    d.is_fake  := nop.is_fake
    d
  }

  // ---- defaults ----
  io.load_nop.ready      := false.B
  if (vecMemWidth > 1) { io.load_nop2.get.ready := false.B }
  io.store_nop.ready     := false.B
  io.dmem.req.valid      := false.B
  io.dmem.req.bits       := DontCare
  if (vecMemWidth > 1) {
    io.dmem.req2.get.valid := false.B
    io.dmem.req2.get.bits  := DontCare
  }
  io.clr_rob.valid       := false.B
  io.clr_rob.bits        := cur.uop.rob_idx
  io.dmem.ld_done.valid  := false.B
  io.dmem.ld_done.bits   := cur.uop.ldq_idx
  io.dmem.st_done.valid  := false.B
  io.dmem.st_done.bits   := curs.uop.stq_idx
  io.vrf_read.req_addr   := cur.uop.stale_pvdest_grp(copy_member(2, 0))  // member 0..7

  // ---- LCB start: latch dest group descriptor on the first beat of a group ----
  val start_grp = io.load_nop.fire && !grp_active
  lcb.io.start.valid     := start_grp
  lcb.io.start.bits.prn  := io.load_nop.bits.uop.pvdest_grp
  lcb.io.start.bits.mask := io.load_nop.bits.uop.pvdest_grp_mask
  lcb.io.kill            := io.kill

  // ---- LCB beat(s): driven on a real response OR on a fake/bypass last beat ----
  for (i <- 0 until vecMemWidth) {
    lcb.io.beat(i).valid        := false.B
    lcb.io.beat(i).bits         := DontCare
    lcb.io.beat(i).bits.is_fake := true.B
  }

  // ---- Response / nack handling (LOAD beats). The LSU routes vector responses here
  // keyed by is_vec; beat_id selects the descriptor. A response PLACES the beat into
  // the LCB (by byte offset -> out-of-order safe) and frees the slot; a nack re-arms
  // the slot for re-issue. Up to vecMemWidth responses/cycle (one per vec dcache
  // pipe). Gated on sLoadIssue so a stale response can never touch the VRF outside a
  // live group. beat_id (not the lane) identifies the descriptor, so resp/resp2 are
  // interchangeable -- either may carry any in-flight beat.
  def placeResp(port: Int, resp: VecDmemResp): Unit = {
    val id = resp.beat_id
    lcb.io.beat(port).valid         := true.B
    lcb.io.beat(port).bits.data     := resp.data
    lcb.io.beat(port).bits.pdst     := desc(id).pdst
    lcb.io.beat(port).bits.dst_byte := desc(id).dst_byte
    lcb.io.beat(port).bits.src_off  := desc(id).src_off
    lcb.io.beat(port).bits.nbytes   := desc(id).nbytes
    lcb.io.beat(port).bits.is_fake  := desc(id).is_fake
    lcb.io.beat(port).bits.last     := false.B           // LCB group_done unused (see below)
    desc(id).valid                  := false.B
  }
  when (io.dmem.resp.valid && state === State.sLoadIssue) {
    placeResp(0, io.dmem.resp.bits)
  }
  when (io.dmem.nack.valid && state === State.sLoadIssue) {
    desc(io.dmem.nack.bits.beat_id).inflight := false.B  // re-arm for re-issue
  }
  if (vecMemWidth > 1) {
    when (io.dmem.resp2.get.valid && state === State.sLoadIssue) {
      placeResp(1, io.dmem.resp2.get.bits)
    }
    when (io.dmem.nack2.get.valid && state === State.sLoadIssue) {
      desc(io.dmem.nack2.get.bits.beat_id).inflight := false.B
    }
  }

  // ---- request payload: LOAD beat from desc(issue_id) in sLoadIssue, STORE in sSReq ----
  // The load beat uop (mem fields set in fillDesc) is tagged with vec_beat_id = the
  // issued slot, so the response routes back to the right descriptor.
  val ld_uop = WireInit(desc(issue_id).uop)
  ld_uop.vec_beat_id := issue_id

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
    io.dmem.req.bits.addr    := desc(issue_id).addr & ~(7.U(coreMaxAddrBits.W))   // align down to 8B
    io.dmem.req.bits.data    := 0.U
    io.dmem.req.bits.uop     := ld_uop
    io.dmem.req.bits.is_load := true.B
  }

  // ---- second LOAD beat payload (dual-dynamic): always load, from desc(issue_id2) ----
  if (vecMemWidth > 1) {
    val ld_uop2 = WireInit(desc(issue_id2).uop)
    ld_uop2.vec_beat_id := issue_id2
    io.dmem.req2.get.bits.addr    := desc(issue_id2).addr & ~(7.U(coreMaxAddrBits.W))
    io.dmem.req2.get.bits.data    := 0.U
    io.dmem.req2.get.bits.uop     := ld_uop2
    io.dmem.req2.get.bits.is_load := true.B
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
      // Store->load memory ordering (Track A) is enforced UPSTREAM: a vector load's
      // issue is squashed until all its program-order-older stores have drained
      // (see core.scala vload squash_grant), so a load only ever reaches here once
      // the dcache already holds those stores' data. No gate is needed at accept.
      io.load_nop.ready  := !io.kill
      io.store_nop.ready := !io.kill && !io.load_nop.valid
      when (io.load_nop.fire) {
        cur        := io.load_nop.bits            // GROUP-level latch (uop for copy + completion)
        grp_active := true.B
        is_store   := false.B
        saw_last   := io.load_nop.bits.last
        // Beat 0 -> descriptor slot 0 (a fake beat accesses no memory: no slot, it
        // just contributes its `last` to completion).
        when (!io.load_nop.bits.is_fake) { desc(0) := fillDesc(io.load_nop.bits) }
        // Copy the OLD group first when masked-off OR tail lanes won't be written
        // (mask-/tail-undisturbed), so those lanes keep their architectural value.
        val needs_copy = (!io.load_nop.bits.uop.v_unmasked) || io.load_nop.bits.tail_undist
        when (needs_copy && !grp_active) {
          copy_member := 0.U
          state       := State.sCopyRd
        } .otherwise {
          state := State.sLoadIssue
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
        // group fully copied -> issue the load beats (or complete if all-fake).
        state := State.sLoadIssue
      } .otherwise {
        copy_member := copy_member + 1.U
        state       := State.sCopyRd
      }
    }

    is (State.sFin) {
      // Fake/bypass STORE finalize (loads finalize in sLoadIssue via the counter).
      when (curs.last) {
        io.clr_rob.valid      := true.B
        io.clr_rob.bits       := curs.uop.rob_idx
        io.dmem.st_done.valid := true.B
        io.dmem.st_done.bits  := curs.uop.stq_idx
        grp_active            := false.B
      }
      state := State.sIdle
    }

    is (State.sLoadIssue) {
      // Accept subsequent beats into free slots until the last one is consumed.
      // (A fake beat takes no slot: it just carries `last` into completion.)
      io.load_nop.ready := has_free && !saw_last && !io.kill
      when (io.load_nop.fire) {
        saw_last := io.load_nop.bits.last
        when (!io.load_nop.bits.is_fake) { desc(free_id) := fillDesc(io.load_nop.bits) }
      }
      // Phase 2: accept a SECOND beat/cycle into free_id2 -- only alongside a firing,
      // non-last beat0 and only with a 2nd free slot. beat1 may itself be the last
      // beat, so it takes over saw_last (last-connect; beat0 is non-last here).
      if (vecMemWidth > 1) {
        io.load_nop2.get.ready := has_2free && !saw_last && !io.kill &&
                                  io.load_nop.fire && !io.load_nop.bits.last
        when (io.load_nop2.get.fire) {
          saw_last := io.load_nop2.get.bits.last
          when (!io.load_nop2.get.bits.is_fake) { desc(free_id2) := fillDesc(io.load_nop2.get.bits) }
        }
      }
      // Issue one beat/cycle (fresh, or re-armed after a nack). Response/nack are
      // handled in the always-block above; a nacked beat drops back to !inflight and
      // is re-selected here.
      when (issue_valid && !io.kill) {
        io.dmem.req.valid := true.B
        when (io.dmem.req.fire) { desc(issue_id).inflight := true.B }
      }
      // Second beat/cycle on req2 (dual-dynamic): a distinct descriptor. The LSU
      // fires it opportunistically into an idle scalar pipe -- if it isn't granted,
      // the descriptor stays !inflight and is re-selected next cycle.
      if (vecMemWidth > 1) {
        when (issue_valid2 && !io.kill) {
          io.dmem.req2.get.valid := true.B
          when (io.dmem.req2.get.fire) { desc(issue_id2).inflight := true.B }
        }
      }
      // Complete once the last beat has been consumed AND every slot has drained
      // (all responses placed -- out-of-order safe).
      when (saw_last && all_done && !io.kill) {
        io.clr_rob.valid      := true.B
        io.clr_rob.bits       := cur.uop.rob_idx
        io.dmem.ld_done.valid := true.B
        io.dmem.ld_done.bits  := cur.uop.ldq_idx
        grp_active            := false.B
        saw_last              := false.B
        state                 := State.sIdle
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
    saw_last   := false.B
    desc.foreach(_.valid := false.B)
  }

  // ---- VRF write: the undisturbed copy (sCopyWr) writes a whole old member to
  // the new pdst member (all lanes); otherwise the LCB's per-beat lane write. ----
  when (state === State.sCopyWr) {
    io.vrf_write.valid     := !io.kill
    io.vrf_write.bits.addr := cur.uop.pvdest_grp(copy_member(2, 0))  // member 0..7
    io.vrf_write.bits.data := io.vrf_read.resp_data
    io.vrf_write.bits.mask := ~0.U((vecVLen / 8).W)    // all bytes (full member copy)
  } .otherwise {
    io.vrf_write := lcb.io.vrf_write(0)
  }
  // Second VRF write port (dual-dynamic): the LCB's distinct-member second beat.
  // Never used by the copy phase (single-member/cycle), so no mux needed.
  if (vecMemWidth > 1) {
    io.vrf_write2.get := lcb.io.vrf_write(1)
  }
  // group_done is driven by the completion counter (responses can arrive out of
  // order), NOT by the LCB's last-beat pulse. The LCB's own group_done is unused.
  io.group_done.valid     := state === State.sLoadIssue && saw_last && all_done && !io.kill
  io.group_done.bits.prn  := cur.uop.pvdest_grp
  io.group_done.bits.mask := cur.uop.pvdest_grp_mask

  io.busy := grp_active || (state =/= State.sIdle)
}

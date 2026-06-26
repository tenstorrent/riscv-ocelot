//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector LS AGEN stage-1 wrapper (Step 10)
//------------------------------------------------------------------------------
//
// `VecAgenStage1(isStore)` is the ONE parameterized router around the already-
// ported Packer + Skipper + Walker FSMs. It mirrors the bobtail loadgen/storegen
// router (reference/vec-lsgen/{loadgen,storegen}.scala): latch a `ConfigInfo`,
// route it to the sub-FSM that handles the access type, and mux the sub-FSMs'
// packet outputs into one. It faithfully mirrors the reference's full 5-state
// FSM (IDLE/BYPASS/PACKING/SKIPPING/WALKING) and its start-mux / output-mux /
// bypass_packet. Differences vs the reference:
//   - after selecting the output packet, the architectural `v_reg` the FSM drives
//     is remapped to the renamed physical PRN (`pdst`/`pdst_member` for loads,
//     `pvs3`/`pvs3_member` for stores) off `uop.pvdest_grp`/`uop.pvs3_grp`, since
//     Caracal renames the EMUL group to NON-contiguous PRNs.
//
// DORMANT in Step 10: core.scala ties this module's output `.ready` false and
// never asserts `io.start.valid`, so the FSMs stay in IDLE. The functional
// unit-stride fast-path and per-member store resolution are deferred to Step 11.

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.exu._
import boom.v4.util._

class VecAgenStage1(isStore: Boolean)(implicit p: Parameters)
extends BoomModule with VecLsConstants {
  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    val start    = Flipped(DecoupledIO(new ConfigInfo))   // latched config from VecLsDecode
    val kill     = Input(Bool())                          // branch-kill / flush
    val mask_idx = new Bundle {                           // tied off in Step 10 (no masked/indexed yet)
      val ready = Output(Bool())
      val valid = Input(Bool())
      val data  = Input(UInt((MASK_W + 2).W))
    }
    // store-data port present ONLY when isStore
    val vdb_data   = if (isStore) Some(new VecDgenDataIF) else None
    // single muxed output nop
    val load_nop   = if (!isStore) Some(DecoupledIO(new VecLoadNop))  else None
    val store_nop  = if (isStore)  Some(DecoupledIO(new VecStoreNop)) else None
    val gen_active = Output(Bool())
  })

  // ======== State / latched config ========
  // mirror loadgen/storegen: latch the incoming ConfigInfo into start_q so the
  // sub-FSMs see a stable config for the whole run, and pick which sub-gen owns
  // the access while IDLE (from io.start.bits) vs while running (from start_q).
  object State extends ChiselEnum {
    val IDLE, BYPASS, PACKING, SKIPPING, WALKING = Value
  }
  val state   = RegInit(State.IDLE)
  val start_q = RegInit(0.U.asTypeOf(new ConfigInfo))

  val config_info = Mux((state === State.IDLE), io.start.bits, start_q)

  // ======== Routing predicates (mirror loadgen/storegen, all 5 routes) ========
  // bypassable: vl == 0 (or vstart >= vl) -> BYPASS (single fake last packet).
  // packable:   good unit/strided segment -> Packer (unit-stride fast-path).
  // skipable:   non-indexed masked LS     -> Skipper.
  // walkable:   everything else           -> Walker.
  // priority: bypassable > packable > skipable > walkable.
  val bypassable = (config_info.vl === 0.U) || (config_info.vstart >= config_info.vl)
  val packable   = config_info.is_good_stride && config_info.is_good_seg
  val skipable   = (config_info.is_mask && !config_info.is_index)
  val walkable   = !(config_info.is_mask && !config_info.is_index)

  // chicken bit (mirror reference): don't pack across segments
  val use_seg_constraint = (config_info.seg_count > 1.U)

  if (isStore) {
    // ============================================================ STORES

    // --- packer ---
    val packer = Module(new VecStorePacker)
    packer.io.start.bits           := config_info
    packer.io.use_seg_constraint   := use_seg_constraint
    packer.io.kill                 := io.kill
    packer.io.store_packet.ready   := io.store_nop.get.ready
    // store-data: forward straight through (inactive one's read_bytes/read_all ignored)
    packer.io.vdb_data.valid_bytes := io.vdb_data.get.valid_bytes
    packer.io.vdb_data.data        := io.vdb_data.get.data

    // --- skipper ---
    val skipper = Module(new VecStoreSkipper)
    skipper.io.start.bits          := config_info
    skipper.io.use_seg_constraint  := use_seg_constraint
    skipper.io.mask.valid          := io.mask_idx.valid
    skipper.io.mask.mask_data      := io.mask_idx.data(MASK_W - 1, 0)
    skipper.io.kill                := io.kill
    skipper.io.store_packet.ready  := io.store_nop.get.ready
    skipper.io.vdb_data.valid_bytes := io.vdb_data.get.valid_bytes
    skipper.io.vdb_data.data        := io.vdb_data.get.data

    // --- walker ---
    val walker = Module(new VecStoreWalker)
    walker.io.start.bits           := config_info
    walker.io.use_seg_constraint   := use_seg_constraint
    walker.io.index.valid          := io.mask_idx.valid
    walker.io.index.index_value    := io.mask_idx.data(MASK_W - 1, 0).asSInt
    walker.io.index.mask_bit       := io.mask_idx.data(MASK_W)
    walker.io.index.last_index     := io.mask_idx.data(MASK_W + 1)
    walker.io.kill                 := io.kill
    walker.io.store_packet.ready   := io.store_nop.get.ready
    walker.io.vdb_data.valid_bytes := io.vdb_data.get.valid_bytes
    walker.io.vdb_data.data        := io.vdb_data.get.data

    // ---- start mux (priority: bypassable > packable > skipable > walkable) ----
    when (bypassable) {
      packer.io.start.valid  := false.B
      skipper.io.start.valid := false.B
      walker.io.start.valid  := false.B
    } .elsewhen (packable) {
      packer.io.start.valid  := io.start.valid
      skipper.io.start.valid := false.B
      walker.io.start.valid  := false.B
    } .elsewhen (skipable) {
      packer.io.start.valid  := false.B
      skipper.io.start.valid := io.start.valid
      walker.io.start.valid  := false.B
    } .elsewhen (walkable) {
      packer.io.start.valid  := false.B
      skipper.io.start.valid := false.B
      walker.io.start.valid  := io.start.valid
    } .otherwise {
      packer.io.start.valid  := false.B
      skipper.io.start.valid := false.B
      walker.io.start.valid  := false.B
    }

    // ---- bypass packet (vl == 0): single fake last packet, no real access ----
    val bypass_packet = Wire(new VecStoreNop)
    bypass_packet.addr        := config_info.base_addr
    bypass_packet.data        := 0.U
    bypass_packet.mem_size    := 0.U
    bypass_packet.is_fake     := true.B
    bypass_packet.misaligned  := false.B
    bypass_packet.last        := true.B
    bypass_packet.uop         := config_info.uop
    bypass_packet.pvs3        := 0.U
    bypass_packet.pvs3_member := 0.U

    // ---- output mux (only one sub-gen is active per access) ----
    val out = io.store_nop.get
    when (state === State.BYPASS) {
      io.start.ready             := false.B
      io.vdb_data.get.read_bytes := 0.U
      io.vdb_data.get.read_all   := false.B
      out.valid                  := true.B
      out.bits                   := bypass_packet
    } .elsewhen (state === State.PACKING) {
      io.start.ready             := packer.io.start.ready
      io.vdb_data.get.read_bytes := packer.io.vdb_data.read_bytes
      io.vdb_data.get.read_all   := packer.io.vdb_data.read_all
      out.valid                  := packer.io.store_packet.valid
      out.bits                   := packer.io.store_packet.bits
    } .elsewhen (state === State.SKIPPING) {
      io.start.ready             := skipper.io.start.ready
      io.vdb_data.get.read_bytes := skipper.io.vdb_data.read_bytes
      io.vdb_data.get.read_all   := skipper.io.vdb_data.read_all
      out.valid                  := skipper.io.store_packet.valid
      out.bits                   := skipper.io.store_packet.bits
    } .elsewhen (state === State.WALKING) {
      io.start.ready             := walker.io.start.ready
      io.vdb_data.get.read_bytes := walker.io.vdb_data.read_bytes
      io.vdb_data.get.read_all   := walker.io.vdb_data.read_all
      out.valid                  := walker.io.store_packet.valid
      out.bits                   := walker.io.store_packet.bits
    } .otherwise {
      // IDLE: transparent to start.ready, no output
      io.start.ready             := PriorityMux(Seq(
        (bypassable) -> true.B,
        (packable)   -> packer.io.start.ready,
        (skipable)   -> skipper.io.start.ready,
        (walkable)   -> walker.io.start.ready,
        (true.B)     -> false.B
      ))
      io.vdb_data.get.read_bytes := 0.U
      io.vdb_data.get.read_all   := false.B
      out.valid                  := false.B
      out.bits                   := DontCare
    }

    // ---- physical-PRN remap (stores) ----
    // VecStoreNop has no v_reg, so the per-member index cannot be derived from
    // the packet. Step 11: per-member pvs3 resolution from the store FSM segment/
    // element progression. For dormancy, default to group member 0.
    out.bits.pvs3_member := 0.U
    out.bits.pvs3        := start_q.uop.pvs3_grp(0)

    // ---- gen_active ----
    io.gen_active := (state === State.BYPASS) ||
                     packer.io.gen_active || skipper.io.gen_active || walker.io.gen_active

    // ---- state machine (mirror reference) ----
    when (state === State.IDLE) {
      when (io.start.fire) {
        state := PriorityMux(Seq(
          (bypassable) -> State.BYPASS,
          (packable)   -> State.PACKING,
          (skipable)   -> State.SKIPPING,
          (walkable)   -> State.WALKING,
          (true.B)     -> State.IDLE
        ))
        start_q := io.start.bits
      }
    } .otherwise {
      when (io.kill) {
        state := State.IDLE
      } .elsewhen (out.fire) {
        when (out.bits.last) {
          state := State.IDLE
        }
      }
    }

  } else {
    // ============================================================ LOADS

    // --- packer ---
    val packer = Module(new VecLoadPacker)
    packer.io.start.bits         := config_info
    packer.io.use_seg_constraint := use_seg_constraint
    packer.io.mask.valid         := io.mask_idx.valid
    packer.io.mask.mask_data     := io.mask_idx.data(MASK_W - 1, 0)
    packer.io.kill               := io.kill
    packer.io.load_packet.ready  := io.load_nop.get.ready

    // --- skipper ---
    val skipper = Module(new VecLoadSkipper)
    skipper.io.start.bits         := config_info
    skipper.io.use_seg_constraint := use_seg_constraint
    skipper.io.mask.valid         := io.mask_idx.valid
    skipper.io.mask.mask_data     := io.mask_idx.data(MASK_W - 1, 0)
    skipper.io.kill               := io.kill
    skipper.io.load_packet.ready  := io.load_nop.get.ready

    // --- walker ---
    val walker = Module(new VecLoadWalker)
    walker.io.start.bits          := config_info
    walker.io.use_seg_constraint  := use_seg_constraint
    walker.io.index.valid         := io.mask_idx.valid
    walker.io.index.index_value   := io.mask_idx.data(MASK_W - 1, 0).asSInt
    walker.io.index.mask_bit      := io.mask_idx.data(MASK_W)
    walker.io.index.last_index    := io.mask_idx.data(MASK_W + 1)
    walker.io.kill                := io.kill
    walker.io.load_packet.ready   := io.load_nop.get.ready

    // ---- start mux (priority: bypassable > packable > skipable > walkable) ----
    when (bypassable) {
      packer.io.start.valid  := false.B
      skipper.io.start.valid := false.B
      walker.io.start.valid  := false.B
    } .elsewhen (packable) {
      packer.io.start.valid  := io.start.valid
      skipper.io.start.valid := false.B
      walker.io.start.valid  := false.B
    } .elsewhen (skipable) {
      packer.io.start.valid  := false.B
      skipper.io.start.valid := io.start.valid
      walker.io.start.valid  := false.B
    } .elsewhen (walkable) {
      packer.io.start.valid  := false.B
      skipper.io.start.valid := false.B
      walker.io.start.valid  := io.start.valid
    } .otherwise {
      packer.io.start.valid  := false.B
      skipper.io.start.valid := false.B
      walker.io.start.valid  := false.B
    }

    // ---- bypass packet (vl == 0): single fake last packet, no real access ----
    val bypass_packet = Wire(new VecLoadNop)
    bypass_packet.addr        := config_info.base_addr
    bypass_packet.v_reg       := config_info.base_v_reg
    bypass_packet.el_id       := 0.U
    bypass_packet.el_off      := 0.U
    bypass_packet.el_count    := config_info.vl // should be 0 if bypassable
    bypass_packet.mask_data   := 0.U
    bypass_packet.mask_valid  := false.B
    bypass_packet.is_fake     := true.B
    bypass_packet.misaligned  := false.B
    bypass_packet.last        := true.B
    bypass_packet.dir         := false.B
    bypass_packet.is_fof      := config_info.is_fof
    bypass_packet.uop         := config_info.uop
    bypass_packet.pdst        := 0.U
    bypass_packet.pdst_member := 0.U

    // ---- output mux (only one sub-gen is active per access) ----
    val out = io.load_nop.get
    when (state === State.BYPASS) {
      io.start.ready := false.B
      out.valid      := true.B
      out.bits       := bypass_packet
    } .elsewhen (state === State.PACKING) {
      io.start.ready := packer.io.start.ready
      out.valid      := packer.io.load_packet.valid
      out.bits       := packer.io.load_packet.bits
    } .elsewhen (state === State.SKIPPING) {
      io.start.ready := skipper.io.start.ready
      out.valid      := skipper.io.load_packet.valid
      out.bits       := skipper.io.load_packet.bits
    } .elsewhen (state === State.WALKING) {
      io.start.ready := walker.io.start.ready
      out.valid      := walker.io.load_packet.valid
      out.bits       := walker.io.load_packet.bits
    } .otherwise {
      io.start.ready := PriorityMux(Seq(
        (bypassable) -> true.B,
        (packable)   -> packer.io.start.ready,
        (skipable)   -> skipper.io.start.ready,
        (walkable)   -> walker.io.start.ready,
        (true.B)     -> false.B
      ))
      out.valid := false.B
      out.bits  := DontCare
    }

    // ---- physical-PRN remap (loads) ----
    // The FSM drives the architectural v_reg; the member index is its offset from
    // the base group reg. pvdest_grp is Vec(8) so the index is masked to 3 bits.
    val pdst_member = (out.bits.v_reg - start_q.base_v_reg)(vecSplitSz - 1, 0)
    out.bits.pdst_member := pdst_member
    out.bits.pdst        := start_q.uop.pvdest_grp(pdst_member(2, 0))

    // ---- gen_active ----
    io.gen_active := (state === State.BYPASS) ||
                     packer.io.gen_active || skipper.io.gen_active || walker.io.gen_active

    // ---- state machine (mirror reference) ----
    when (state === State.IDLE) {
      when (io.start.fire) {
        state := PriorityMux(Seq(
          (bypassable) -> State.BYPASS,
          (packable)   -> State.PACKING,
          (skipable)   -> State.SKIPPING,
          (walkable)   -> State.WALKING,
          (true.B)     -> State.IDLE
        ))
        start_q := io.start.bits
      }
    } .otherwise {
      when (io.kill) {
        state := State.IDLE
      } .elsewhen (out.fire) {
        when (out.bits.last) {
          state := State.IDLE
        }
      }
    }
  }

  // ---- mask_idx: tied off at the stage level (no masked/indexed support yet) ----
  // The sub-FSMs' mask/index .valid is wired to io.mask_idx.valid (which core.scala
  // holds false in Step 10), keeping them quiescent. We do NOT forward the sub-FSMs'
  // mask/index .ready up; the stage advertises not-ready so nothing is consumed.
  io.mask_idx.ready := false.B
}

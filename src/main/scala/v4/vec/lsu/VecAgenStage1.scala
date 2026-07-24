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
    // Phase 2 (dual-dynamic): a second load nop/cycle, from the Packer fast path only
    // (Skipper/Walker leave it invalid). Present only when vecMemWidth>1 && !isStore.
    val load_nop2  = if (!isStore && vecMemWidth > 1) Some(DecoupledIO(new VecLoadNop)) else None
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
  // The LOAD Packer is mask-aware (emits is_fake for masked-off elements), so a
  // masked unit-stride load stays packable. The STORE Packer has NO mask support
  // (is_fake hardcoded false), so a masked unit-stride store must NOT pack -- it
  // routes to the Skipper instead (which skips masked-off elements and keeps the
  // VecDgen store-data stream in sync).
  val packable   = config_info.is_good_stride && config_info.is_good_seg &&
                   (if (isStore) !config_info.is_mask else true.B)
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
    // propagate the Walker's index request up to the index producer (VecIdxGen).
    io.mask_idx.ready              := walker.io.index.ready
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
    if (vecMemWidth > 1) {
      // beat1 is accepted only in PACKING (the fast path); other states force it off.
      packer.io.load_packet2.get.ready := (state === State.PACKING) && io.load_nop2.get.ready
    }

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
    // propagate the Walker's index request up to the index producer (VecIdxGen).
    io.mask_idx.ready             := walker.io.index.ready

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
    bypass_packet.tail_undist := false.B          // set by the remap below
    bypass_packet.grp_lo      := 0.U              // set by the remap below
    bypass_packet.grp_hi      := 0.U
    bypass_packet.grp_rng_v   := false.B

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

    // ---- tail-undisturbed flag (group property) ----
    // The dest group holds (members * VLEN_BYTES) bytes; the load writes
    // (vl << eew_enc) bytes. If it writes fewer, the unwritten TAIL lanes must
    // keep their old value -> VecLSU does a stale_pvdest group-copy first. vl==0
    // (bypass) also lands here (0 < capacity) -> whole group undisturbed.
    val grp_bytes = PopCount(start_q.uop.pvdest_grp_mask) << log2Ceil(VLEN_BYTES).U
    val ld_bytes  = start_q.vl << start_q.eew_enc
    out.bits.tail_undist := ld_bytes < grp_bytes

    // Track A: the load's [grp_lo,grp_hi) byte range for the LDQ, so an older store's
    // LCAM search can order_fail this speculative load if it overlaps. PRECISE for the
    // contiguous unit-stride fast path (non-masked, single-segment, unit-stride,
    // positive dir); a conservative FULL range for strided/indexed/masked (over-match
    // -> correct, just extra replays). vl==0 has no footprint (rng_v=false).
    val us_contig = !start_q.is_mask && (start_q.seg_enc === 0.U) &&
                    (start_q.stride_enc === 0.U) && !start_q.stride_dir
    out.bits.grp_rng_v := (start_q.vl =/= 0.U)
    out.bits.grp_lo    := Mux(us_contig, start_q.base_addr, 0.U)
    out.bits.grp_hi    := Mux(us_contig, start_q.base_addr + ld_bytes,
                              ~(0.U(coreMaxAddrBits.W)))

    // ---- Phase 2 (dual-dynamic): second output nop (Packer fast path only) ----
    // Valid only in PACKING off the Packer's beat1; same PRN remap + tail_undist.
    val out2_last = WireInit(false.B)
    val out2_fire = WireInit(false.B)
    if (vecMemWidth > 1) {
      val out2 = io.load_nop2.get
      out2.valid := (state === State.PACKING) && packer.io.load_packet2.get.valid
      out2.bits  := packer.io.load_packet2.get.bits
      val pdst_member2 = (out2.bits.v_reg - start_q.base_v_reg)(vecSplitSz - 1, 0)
      out2.bits.pdst_member := pdst_member2
      out2.bits.pdst        := start_q.uop.pvdest_grp(pdst_member2(2, 0))
      out2.bits.tail_undist := ld_bytes < grp_bytes
      out2_last := out2.bits.last
      out2_fire := out2.fire
    }

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
        // complete on beat0's last, or beat1's last if the fast-path second beat fired
        when (out.bits.last || (out2_fire && out2_last)) {
          state := State.IDLE
        }
      }
    }
  }

  // io.mask_idx.ready is driven inside each branch above (propagating the Walker's
  // per-element index request to the producer, VecIdxGen).
}

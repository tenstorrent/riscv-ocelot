//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Free List (Step 3)
//------------------------------------------------------------------------------
//
// Allocates whole EMUL groups of vector PRNs atomically. A vector instruction
// renames up to 8 member PRNs per architectural vector destination; segmented
// (is_shared) load/stores additionally allocate a second "pvtmp" group from the
// same free list (up to 16 PRNs total for one uop).
//
// Mirrors boom.v4.exu.RenameFreeList (src/main/scala/v4/exu/rename/rename-freelist.scala)
// for the SelectFirstN selector, the free_list reg + update, and the branch /
// commit / rollback reclaim machinery (br_alloc_lists, spec_alloc_list,
// br_deallocs, com_deallocs, rollback_deallocs). Differences from the scalar
// free list:
//   - Allocations are GROUPS of up to 8 PRNs per uop instead of one PRN per
//     lane. allocWidth = coreWidth*16 single-PRN selects are carved into per-uop
//     slices via a program-order prefix sum of EMUL demands. The *16 (not *8,
//     as the spec text states) sizes the selector to the true worst case: one
//     segmented (is_shared) uop needs up to 8 dest + 8 tmp = 16 PRNs, so a full
//     bundle of shared uops needs coreWidth*16 selects. This guarantees the
//     plan's forward-progress property (a segmented LS can always allocate
//     pvdest+pvtmp in one cycle) and keeps every sels index in range — the max
//     index used is sel_off(last)+mc+j = coreWidth*16-1 = allocWidth-1.
//   - The vector mapper is single-cycle / parallel, so alloc_groups outputs are
//     COMBINATIONAL (no internal output register). This is a deliberate
//     deviation from rename-freelist.scala, whose alloc_pregs are produced by a
//     2-stage RegEnable/r_valid pipeline.
//   - A reserve headroom (numVecTmpGroups*8 PRNs) is kept so the oldest
//     segmented op can always allocate both its groups (midcore.rst free-list /
//     pvtmp notes). Non-shared dest requests may not breach it; shared requests
//     (which carry the tmp) may dip into it.
//   - There is no `despec` port: the vector busy table is cleared by group-done
//     events, not by an immediate-read despec path, so com_despec is dropped and
//     the isImm=true branch is not used.

package boom.v4.vec.rename

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.util._
import boom.v4.exu.BrUpdateInfo

// NOTE: `coreWidth` is declared `override val` because HasBoomCoreParameters
// (parameters.scala:210) already provides a concrete `coreWidth = decodeWidth`.
// The frozen contract names this constructor param `coreWidth`; overriding the
// inherited val keeps the name/type/position while satisfying Scala's rule that
// a constructor param shadowing an inherited concrete val must be `override`.
// `commitWidth` and `numPregs` do not collide with any inherited member.
class VecFreeList(override val coreWidth: Int, val commitWidth: Int, val numPregs: Int)(implicit p: Parameters)
  extends BoomModule
{
  val pregSz       = log2Ceil(numPregs)
  val allocWidth   = coreWidth * 16    // up to 8 dest + 8 tmp PRNs per uop (segmented LS)
  val deallocWidth = commitWidth * 8   // up to 8 member PRNs per committed stale group
  val n            = numPregs

  val io = IO(new Bundle {
    val initial_allocation = Input(UInt(numPregs.W))

    // Per-uop group allocation requests / responses (combinational).
    val reqs         = Input (Vec(coreWidth, new VecAllocReq))
    val alloc_groups = Output(Vec(coreWidth, new VecAllocResp))

    // Stale groups returned by the ROB at commit.
    val dealloc      = Input (Vec(commitWidth, new VecGroupDealloc))

    // Branch info for starting new allocation lists.
    val ren_br_tags  = Input (Vec(coreWidth + 1, Valid(UInt(brTagSz.W))))

    // Mispredict info for recovering speculatively-allocated groups.
    val brupdate     = Input (new BrUpdateInfo)
    val rollback     = Input (Bool())

    val debug_freelist = Output(UInt(numPregs.W))

    // Gated trace.
    val vec_trace    = Input (Bool())
    val dec_uop_id   = Input (Vec(coreWidth, UInt(32.W)))
  })

  // --------------------------------------------------------------------------
  // Free list register + branch allocation snapshots (mirrors rename-freelist:62-64)
  // --------------------------------------------------------------------------
  val free_list       = RegInit(UInt(numPregs.W), io.initial_allocation)
  val spec_alloc_list = RegInit(0.U(numPregs.W))
  val br_alloc_lists  = Reg(Vec(maxBrCount, UInt(numPregs.W)))

  // Select allocWidth distinct free PRNs (one-hot each). (rename-freelist:67, util.scala:431)
  val sels = SelectFirstN(free_list, allocWidth)

  val free_count = PopCount(free_list)

  // Reserve headroom: keep numVecTmpGroups full groups (8 PRNs each) available so
  // the oldest segmented op can always allocate its pvdest + pvtmp.
  // (midcore.rst:174-178)
  val reserveThreshold = (vectorParams.numVecTmpGroups * 8).U

  // --------------------------------------------------------------------------
  // Program-order grouping + reserve check.
  //
  // Each uop w demands memberCount(v_emul) PRNs for its dest group, plus another
  // memberCount if is_shared (the pvtmp group). We carve the flat `sels` index
  // space into per-uop slices via a prefix sum of demands. Separately we track a
  // prefix sum of NON-reserve (non-shared) demand to enforce the headroom.
  // --------------------------------------------------------------------------
  // Worst-case demand is 2*8 PRNs per uop (shared, EMUL=8), so a prefix sum over
  // coreWidth lanes can reach coreWidth*16; size the accumulators to hold it.
  val cntW    = log2Ceil(coreWidth * 16 + 1)
  val mc      = Wire(Vec(coreWidth, UInt(4.W)))   // member count of uop w
  val demand  = Wire(Vec(coreWidth, UInt(cntW.W))) // sels consumed by w (mc, or 2*mc if shared)
  val sel_off = Wire(Vec(coreWidth, UInt(cntW.W))) // base index into sels for uop w

  // Cumulative non-reserve PRN demand of lanes strictly older than w PLUS w
  // itself (only non-shared uops count; shared uops carry their own tmp and may
  // dip into the reserve).
  val nonreserve_prefix = Wire(Vec(coreWidth, UInt(cntW.W)))

  for (w <- 0 until coreWidth) {
    mc(w)     := VecEmul.memberCount(io.reqs(w).v_emul)
    demand(w) := Mux(io.reqs(w).is_shared, mc(w) +& mc(w), 0.U +& mc(w))
  }

  // sel_off / nonreserve prefix sums in program order.
  for (w <- 0 until coreWidth) {
    if (w == 0) {
      sel_off(w)           := 0.U
      nonreserve_prefix(w) := Mux(!io.reqs(w).is_shared && io.reqs(w).valid, mc(w), 0.U)
    } else {
      // Only fired+valid uops consume sels; gate the running offset by valid so a
      // bubble lane does not waste the index space of younger lanes.
      sel_off(w)           := sel_off(w - 1) + Mux(io.reqs(w - 1).valid, demand(w - 1), 0.U)
      nonreserve_prefix(w) := nonreserve_prefix(w - 1) +
                                Mux(!io.reqs(w).is_shared && io.reqs(w).valid, mc(w), 0.U)
    }
  }

  // sel_fire(w) = the slice for uop w was actually consumed (drives free_list update).
  val sel_fire = Wire(Vec(coreWidth, Bool()))

  for (w <- 0 until coreWidth) {
    val req     = io.reqs(w)
    val mc_w     = mc(w)
    val mask     = VecEmul.memberMask(req.v_emul)

    // Gather the dest-group members: sels(sel_off + j) for j < mc.
    val have_dest = Wire(Vec(VecEmul.MAX_MEMBERS, Bool()))
    for (j <- 0 until VecEmul.MAX_MEMBERS) {
      val idx     = sel_off(w) + j.U
      val sel_oh   = sels(idx)
      io.alloc_groups(w).pdst(j) := OHToUInt(sel_oh)
      // a needed member (j < mc) must have a non-zero one-hot select
      have_dest(j) := Mux(j.U < mc_w, sel_oh.orR, true.B)
    }

    // Gather the tmp-group members when shared: sels(sel_off + mc + j) for j < mc.
    val have_tmp = Wire(Vec(VecEmul.MAX_MEMBERS, Bool()))
    for (j <- 0 until VecEmul.MAX_MEMBERS) {
      val idx     = sel_off(w) + mc_w + j.U
      val sel_oh   = sels(idx)
      io.alloc_groups(w).pdst_tmp(j) := OHToUInt(sel_oh)
      val needed   = req.is_shared && (j.U < mc_w)
      have_tmp(j) := Mux(needed, sel_oh.orR, true.B)
    }

    io.alloc_groups(w).mask     := mask
    io.alloc_groups(w).mask_tmp := Mux(req.is_shared, mask, 0.U)

    // Reserve check: a NON-shared dest may consume sels only if, after the
    // cumulative non-reserve demand of older lanes + itself, the free count would
    // not drop below reserveThreshold. Shared uops bypass the reserve.
    val reserve_ok = req.is_shared ||
      (free_count >= (nonreserve_prefix(w) +& reserveThreshold))

    val have_all = have_dest.reduce(_ && _) && have_tmp.reduce(_ && _)

    io.alloc_groups(w).valid := req.valid && have_all && reserve_ok
    sel_fire(w)              := io.alloc_groups(w).valid
  }

  // --------------------------------------------------------------------------
  // Masks that modify the free list.
  // --------------------------------------------------------------------------
  // per_uop_alloc(w): the set of PRN bits actually consumed by uop w (dest slice
  // [0, mc) plus tmp slice [mc, 2*mc) when shared), gated by sel_fire.
  val per_uop_alloc = Wire(Vec(coreWidth, UInt(n.W)))
  for (w <- 0 until coreWidth) {
    val dest_mask = (0 until VecEmul.MAX_MEMBERS).map { j =>
      Mux(j.U < mc(w), sels(sel_off(w) + j.U), 0.U(n.W))
    }.reduce(_ | _)
    val tmp_mask = (0 until VecEmul.MAX_MEMBERS).map { j =>
      Mux(io.reqs(w).is_shared && (j.U < mc(w)), sels(sel_off(w) + mc(w) + j.U), 0.U(n.W))
    }.reduce(_ | _)
    per_uop_alloc(w) := (dest_mask | tmp_mask) & Fill(n, sel_fire(w))
  }

  // sel_mask: OR of every slice consumed this cycle (drives free_list clear).
  val sel_mask = per_uop_alloc.reduce(_ | _)

  // alloc_masks: per-branch-slot view of younger-or-equal allocations, used to
  // snapshot br_alloc_lists. scanRight matches rename-freelist:72 (coreWidth+1
  // entries). alloc_masks(0) = all allocations this cycle.
  val alloc_masks = per_uop_alloc.scanRight(0.U(n.W)) { case (a, m) => m | a }

  // --------------------------------------------------------------------------
  // Branch / commit / rollback reclaim (mirrors rename-freelist:77-119, isImm=false).
  // --------------------------------------------------------------------------
  val br_deallocs = br_alloc_lists(io.brupdate.b2.uop.br_tag) & Fill(n, io.brupdate.b2.mispredict)

  // Stale groups freed at commit: 8 bits per commit slot, OR'd. RegNext matches
  // rename-freelist:78 (deallocs arrive registered from the ROB).
  val com_deallocs = RegNext(io.dealloc).map { d =>
    (0 until VecEmul.MAX_MEMBERS).map { j =>
      UIntToOH(d.prn(j))(numPregs - 1, 0) & Fill(n, d.valid && d.mask(j))
    }.reduce(_ | _)
  }.reduce(_ | _)

  val rollback_deallocs = spec_alloc_list & Fill(n, io.rollback)
  val dealloc_mask      = com_deallocs | br_deallocs | rollback_deallocs

  // Update branch snapshots (isImm=false path only).
  for (i <- 0 until maxBrCount) {
    br_alloc_lists(i) := br_alloc_lists(i) & ~br_deallocs | alloc_masks(0)
  }
  if (enableSuperscalarSnapshots) {
    val br_slots = VecInit(io.ren_br_tags.map(tag => tag.valid)).asUInt
    for (i <- 0 until maxBrCount) {
      val list_req = VecInit(io.ren_br_tags.map(tag => tag.bits === i.U)).asUInt & br_slots
      val new_list = list_req.orR
      when (new_list) {
        br_alloc_lists(i) := Mux1H(list_req, alloc_masks)
      }
    }
  } else {
    assert(PopCount(io.ren_br_tags.map(_.valid)) <= 1.U)
    val do_br_snapshot  = io.ren_br_tags.map(_.valid).reduce(_ || _)
    val br_snapshot_tag  = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_tags.map(_.bits))
    val br_snapshot_list = Mux1H(io.ren_br_tags.map(_.valid), alloc_masks)
    when (do_br_snapshot) {
      br_alloc_lists(br_snapshot_tag) := br_snapshot_list
    }
  }

  spec_alloc_list := (spec_alloc_list | alloc_masks(0)) & ~dealloc_mask

  // Update the free list (rename-freelist:119).
  free_list := (free_list & ~sel_mask) | dealloc_mask

  // --------------------------------------------------------------------------
  // Debug leak assertion (mirrors rename-freelist:134-137).
  // --------------------------------------------------------------------------
  val alloc_groups_mask = (0 until coreWidth).map { w =>
    val dest_mask = (0 until VecEmul.MAX_MEMBERS).map { j =>
      UIntToOH(io.alloc_groups(w).pdst(j))(n - 1, 0) & Fill(n, io.alloc_groups(w).mask(j))
    }.reduce(_ | _)
    val tmp_mask = (0 until VecEmul.MAX_MEMBERS).map { j =>
      UIntToOH(io.alloc_groups(w).pdst_tmp(j))(n - 1, 0) & Fill(n, io.alloc_groups(w).mask_tmp(j))
    }.reduce(_ | _)
    (dest_mask | tmp_mask) & Fill(n, io.alloc_groups(w).valid)
  }.reduce(_ | _)

  io.debug_freelist := free_list | alloc_groups_mask

  assert(!(io.debug_freelist & dealloc_mask).orR, "[vfl] Returning a free physical register.")

  // --------------------------------------------------------------------------
  // Gated trace.
  // --------------------------------------------------------------------------
  when (io.vec_trace) {
    for (w <- 0 until coreWidth) {
      when (io.alloc_groups(w).valid) {
        printf("[vfl] uop_id=%d alloc=[%d %d %d %d %d %d %d %d] mask=%b is_shared=%d " +
            "tmp=[%d %d %d %d %d %d %d %d] free_count=%d\n",
          io.dec_uop_id(w),
          io.alloc_groups(w).pdst(0), io.alloc_groups(w).pdst(1),
          io.alloc_groups(w).pdst(2), io.alloc_groups(w).pdst(3),
          io.alloc_groups(w).pdst(4), io.alloc_groups(w).pdst(5),
          io.alloc_groups(w).pdst(6), io.alloc_groups(w).pdst(7),
          io.alloc_groups(w).mask, io.reqs(w).is_shared,
          io.alloc_groups(w).pdst_tmp(0), io.alloc_groups(w).pdst_tmp(1),
          io.alloc_groups(w).pdst_tmp(2), io.alloc_groups(w).pdst_tmp(3),
          io.alloc_groups(w).pdst_tmp(4), io.alloc_groups(w).pdst_tmp(5),
          io.alloc_groups(w).pdst_tmp(6), io.alloc_groups(w).pdst_tmp(7),
          free_count)
      }
      // A non-shared uop requested but blocked solely by the reserve headroom.
      val reserve_block = io.reqs(w).valid && !io.reqs(w).is_shared &&
        !(free_count >= (nonreserve_prefix(w) +& reserveThreshold))
      when (reserve_block) {
        printf("[vfl-stall] uop_id=%d reserve_block=1\n", io.dec_uop_id(w))
      }
    }
  }
}

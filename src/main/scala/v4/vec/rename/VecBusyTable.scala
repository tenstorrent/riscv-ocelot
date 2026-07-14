//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Busy Table (Step 3)
//------------------------------------------------------------------------------
//
// Per-PRN readiness for the vector register file. A vector instruction renames a
// whole EMUL group atomically (up to 8 member PRNs per architectural dest), and a
// consumer may read a sub-range of a larger producer group, so readiness is tracked
// per member PRN -- NOT by a group base -- as a flat bit vector over numPregs (see
// docs_caracal/src/midcore.rst, "Busy Table" / group-done).
//
// This mirrors the scalar RenameBusyTable (exu/rename/rename-busytable.scala:27-93):
// set-on-alloc, clear-on-wakeup, read-for-sources. The key SIMPLIFICATION is that the
// vector busy table has NO rebusy / speculative-wakeup machinery: vector operands wake
// only on actual completion, never speculatively (midcore.rst:516-521), so there is no
// speculative_mask, no child_rebusy, and no per-wakeup rebusy bit.
//
// A group-ready ("busy") bit collapses the per-member bits: a source group is busy if
// ANY active member is busy. Because a producer emits a single group-done that clears
// all of its members at once, a consumer reading a sub-range stays busy until that
// completion fires (conservative but correct).

package boom.v4.vec.rename

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

class VecBusyTable(
  val plWidth: Int,
  val numPregs: Int,
  val numWbPorts: Int)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    val ren_srcs    = Input (Vec(plWidth, new VecBusySrc))             // source groups + masks + v_emul + reads_mask
    val busy_resps  = Output(Vec(plWidth, new VecBusyResp))            // one group-ready (busy) bit per source
    val rebusy_reqs = Input (Vec(plWidth, new VecRebusyReq))           // set dest+tmp group busy
    val wakeups     = Input (Vec(numWbPorts, Valid(new VecGroupDone))) // group-done clears

    val vec_trace   = Input (Bool())
    val dec_uop_id  = Input (Vec(plWidth, UInt(32.W)))

    val debug = new Bundle { val busytable = Output(UInt(numPregs.W)) }
  })

  val busy_table = RegInit(0.U(numPregs.W))

  // --------------------------------------------------------------------------
  // Clear (group-done): a single completion event carries the producer group's
  // full member-PRN vector; clear every active member's bit. Mirrors the scalar
  // clear at rename-busytable.scala:55-57, but per-member rather than per-pdst.
  // --------------------------------------------------------------------------
  val clear_mask = io.wakeups.map { w =>
    (0 until VecEmul.MAX_MEMBERS).map { j =>
      UIntToOH(w.bits.prn(j), numPregs) & Fill(numPregs, w.valid && w.bits.mask(j))
    }.reduce(_ | _)
  }.reduce(_ | _)

  // --------------------------------------------------------------------------
  // Set (allocation): set all active member bits of the destination group, plus
  // the segmented-LS pvtmp group when is_shared. Mirrors the scalar set at
  // rename-busytable.scala:59-61, but per-member and with the extra tmp group.
  // No rebusy / speculative set path (vector wakeups are non-speculative;
  // midcore.rst:516-521).
  // --------------------------------------------------------------------------
  // Per-lane set masks (kept separate for the older-lane same-cycle bypass below),
  // plus the reduced set_mask that updates the (registered) busy table.
  val set_masks = io.rebusy_reqs.map { r =>
    val dst = (0 until VecEmul.MAX_MEMBERS).map { j =>
      UIntToOH(r.pdst(j), numPregs) & Fill(numPregs, r.valid && r.mask(j))
    }.reduce(_ | _)
    val tmp = (0 until VecEmul.MAX_MEMBERS).map { j =>
      UIntToOH(r.pdst_tmp(j), numPregs) & Fill(numPregs, r.valid && r.is_shared && r.mask_tmp(j))
    }.reduce(_ | _)
    dst | tmp
  }
  val set_mask = set_masks.reduce(_ | _)

  busy_table := (busy_table & ~clear_mask) | set_mask

  // --------------------------------------------------------------------------
  // Source reads: collapse the per-member busy bits of a group into one
  // group-ready ("busy") bit. Busy if ANY active member is busy. Same-cycle
  // group-done is forwarded by masking out clear_mask (so a source woken this
  // cycle reads ready).
  //
  // OLDER-LANE SET BYPASS: a source of lane i that is (a member of) a dest group
  // freshly allocated by an OLDER lane k<i in THIS SAME rename packet must read
  // BUSY -- the busy-table write is registered (visible next cycle), so without
  // this bypass a younger consumer of a same-packet producer reads not-busy and
  // issues before the producer's result exists. (This is the RAW hazard that let
  // a vadd read a just-loaded vreg before the vector load's VRF write landed.)
  // olderSet(i) = OR of the older lanes' set masks; fresh dests are never woken
  // this cycle, so it is OR'd in after the clear.
  // --------------------------------------------------------------------------
  def groupBusy(prn: Vec[UInt], v_emul: UInt, olderSet: UInt): Bool = {
    val mc = VecEmul.memberCount(v_emul)
    (0 until VecEmul.MAX_MEMBERS).map { j =>
      (j.U < mc) && ((busy_table(prn(j)) && !clear_mask(prn(j))) || olderSet(prn(j)))
    }.reduce(_ || _)
  }

  for (i <- 0 until plWidth) {
    val olderSet = if (i == 0) 0.U(numPregs.W) else set_masks.take(i).reduce(_ | _)
    io.busy_resps(i).pvs1_busy := groupBusy(io.ren_srcs(i).pvs1, io.ren_srcs(i).v_emul, olderSet)
    io.busy_resps(i).pvs2_busy := groupBusy(io.ren_srcs(i).pvs2, io.ren_srcs(i).v_emul, olderSet)
    io.busy_resps(i).pvs3_busy := groupBusy(io.ren_srcs(i).pvs3, io.ren_srcs(i).v_emul, olderSet)
    // pvm is a single PRN (always v0); it participates only when the op is masked.
    io.busy_resps(i).pvm_busy  := io.ren_srcs(i).reads_mask &&
                                  (((busy_table(io.ren_srcs(i).pvm) &&
                                     !clear_mask(io.ren_srcs(i).pvm)) ||
                                    olderSet(io.ren_srcs(i).pvm)))
  }

  io.debug.busytable := busy_table

  // --------------------------------------------------------------------------
  // Gated trace (one line per event).
  // --------------------------------------------------------------------------
  when (io.vec_trace) {
    for (i <- 0 until plWidth) {
      val r = io.rebusy_reqs(i)
      when (r.valid) {
        printf("[vbt-set] uop_id=%d set=[%d %d %d %d %d %d %d %d] is_shared=%d\n",
          io.dec_uop_id(i),
          r.pdst(0), r.pdst(1), r.pdst(2), r.pdst(3),
          r.pdst(4), r.pdst(5), r.pdst(6), r.pdst(7),
          r.is_shared)
      }
    }
    for (w <- io.wakeups) {
      when (w.valid) {
        printf("[vbt-clr] gd_prn=[%d %d %d %d %d %d %d %d] mask=%b\n",
          w.bits.prn(0), w.bits.prn(1), w.bits.prn(2), w.bits.prn(3),
          w.bits.prn(4), w.bits.prn(5), w.bits.prn(6), w.bits.prn(7),
          w.bits.mask)
      }
    }
    for (i <- 0 until plWidth) {
      printf("[vbt-rdy] uop_id=%d pvs1_busy=%d pvs2_busy=%d pvs3_busy=%d pvm_busy=%d\n",
        io.dec_uop_id(i),
        io.busy_resps(i).pvs1_busy, io.busy_resps(i).pvs2_busy,
        io.busy_resps(i).pvs3_busy, io.busy_resps(i).pvm_busy)
    }
  }
}

/*
Tenstorrent CONFIDENTIAL
__________________
 Tenstorrent Inc.
 All Rights Reserved.

NOTICE:  All information contained herein is, and remains
the property of Tenstorrent Inc.  The intellectual
and technical concepts contained
herein are proprietary to Tenstorrent Inc.
and may be covered by U.S., Canadian and Foreign Patents,
patents in process, and are protected by trade secret or copyright law.
Dissemination of this information or reproduction of this material
is strictly forbidden unless prior written permission is obtained
from Tenstorrent Inc.
*/

package boom.v4.vec.generated.rename

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common.BoomModule
import boom.v4.exu.BrUpdateInfo
import boom.v4.util.SelectFirstN
import boom.v4.vec.generated.VecTrace

// GENERATED from src/main/nlhdl/vec/rename/VecFreeList.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
class VecFreeList(
  numPhysRegs:    Int,
  numArchRegs:    Int,
  maxGroupSize:   Int,
  freeDiscipline: String
)(implicit p: Parameters) extends BoomModule
{
  // `require` it is one of the two known spellings: a mistyped discipline
  // falling back to the other would leak or double-free PRNs thousands of
  // cycles later.
  require(freeDiscipline == "stale_group" || freeDiscipline == "committed_ptr",
    s"""freeDiscipline ("$freeDiscipline") must be "stale_group" or "committed_ptr"""")

  val pregSz = log2Ceil(numPhysRegs)
  val n = numPhysRegs

  //@req-spec-rename.f4
  //@req-spec-rename.f5
  // The 2* is not slack: halving it makes alloc_ok's `i >= total_demand` escape
  // unreachable, so an over-demanding lane silently re-gets an older lane's PRN.
  val allocWidth = coreWidth * 2 * maxGroupSize

  //@req-spec-rename.f12
  val deallocWidth = retireWidth * maxGroupSize

  require(numPhysRegs >= numArchRegs + allocWidth + 2 * maxGroupSize,
    s"numPhysRegs ($numPhysRegs) must be >= numArchRegs ($numArchRegs) + allocWidth " +
    s"($allocWidth) + 2*maxGroupSize (${2 * maxGroupSize}): the pre-selection stage parks " +
    s"allocWidth PRNs in its holding registers permanently, so only " +
    s"numPhysRegs - numArchRegs - allocWidth of them are reachable at rest. Size the file " +
    "for the reservoir as well, or free_list is empty with the machine idle, port 0 can " +
    "never refill, and no full group can ever be renamed -- the machine deadlocks at rename")
  require(allocWidth >= 2 * maxGroupSize,
    s"allocWidth ($allocWidth) must be >= 2*maxGroupSize (${2 * maxGroupSize}): a shared OP.v " +
    "needs both its groups from one cycle's selection, or the machine deadlocks rather than stalls")

  val io = IO(new Bundle {
    // Clock/reset: implicit, posedge clock, synchronous active-high reset
    val initial_allocation = Input(UInt(numPhysRegs.W))

    // ---- Request side: one request per rename LANE, not per PRN ----
    val reqs        = Input(Vec(coreWidth, Bool()))
    val req_members = Input(Vec(coreWidth, UInt((log2Ceil(maxGroupSize) + 1).W)))
    val req_shared  = Input(Vec(coreWidth, Bool()))

    // ---- Grant side ----
    val alloc_pvdest = Output(Vec(coreWidth, Vec(maxGroupSize, UInt(pregSz.W))))
    val alloc_pvtmp  = Output(Vec(coreWidth, Vec(maxGroupSize, UInt(pregSz.W))))
    val alloc_ok     = Output(Bool())
    val alloc_fire   = Input(Vec(coreWidth, Bool()))

    // ---- Reclaim side ----
    val dealloc = Input(Vec(deallocWidth, Valid(UInt(pregSz.W))))
    val dealloc_tmp = if (maxGroupSize > 1) Some(Input(Vec(deallocWidth, Valid(UInt(pregSz.W))))) else None

    val ren_br_tags = Input(Vec(coreWidth + 1, Valid(UInt(brTagSz.W))))
    val brupdate    = Input(new BrUpdateInfo)
    val rollback    = Input(Bool())

    // ---- Observability ----
    val debug_freelist = Output(UInt(numPhysRegs.W))
    val stall_cnt_inc  = Output(Bool())
  })

  // ===========================================================================
  // ---- State ----
  // ===========================================================================

  //@req-spec-rename.f1
  //@req-spec-rename.h11
  val free_list = RegInit(UInt(numPhysRegs.W), io.initial_allocation)
  val spec_alloc_list = RegInit(0.U(numPhysRegs.W))
  val br_alloc_lists = Reg(Vec(maxBrCount, UInt(numPhysRegs.W)))

  // ===========================================================================
  // ---- Selection and the registered pre-selection stage ----
  // ===========================================================================

  //@req-spec-rename.f2
  //@req-spec-rename.f7
  val sels = SelectFirstN(free_list, allocWidth)
  val sel_fire = Wire(Vec(allocWidth, Bool()))

  // ===========================================================================
  // ---- Demand, and the all-or-nothing decision ----
  // ===========================================================================

  //@req-spec-rename.f3
  //@req-spec-rename.f6
  val demandW = log2Ceil(2 * maxGroupSize + 1)
  val demand = Wire(Vec(coreWidth, UInt(demandW.W)))
  for (w <- 0 until coreWidth) {
    val doubled = Mux(io.req_shared(w), io.req_members(w) << 1, io.req_members(w))
    demand(w) := Mux(io.reqs(w), doubled, 0.U)
  }

  val baseW = log2Ceil(coreWidth * 2 * maxGroupSize + 1)
  val base = demand.scanLeft(0.U(baseW.W))(_ + _)
  val total_demand = base(coreWidth)

  //@req-spec-rename.e14
  //@req-spec-issue.c3
  //@req-spec-rename.e15
  val r_valid = Wire(Vec(allocWidth, Bool()))
  val alloc_ok = (0 until allocWidth).map(i => r_valid(i) || i.U >= total_demand).reduce(_ && _)
  io.alloc_ok := alloc_ok

  //@req-spec-vrf.b11
  io.stall_cnt_inc := !alloc_ok

  when (!alloc_ok) {
    VecTrace.traceStruct("VecFreeList", "stall", Seq(
      ("lane", PriorityEncoder(io.reqs)),
      ("demand", total_demand),
      ("free", PopCount(free_list))))
  }

  // ===========================================================================
  // ---- Partial-prefix fire and per-lane consumption ----
  // ===========================================================================

  def portInFiringWindow(i: Int): Bool =
    (0 until coreWidth).map { w =>
      io.alloc_fire(w) && i.U >= base(w) && i.U < (base(w) + demand(w))
    }.reduce(_ || _)

  // M1 BUG: bundle-wide fire would leak/double-allocate -- every consumption must use per-lane io.alloc_fire(w).
  val r_sel = Wire(Vec(allocWidth, UInt(pregSz.W)))
  for (i <- 0 until allocWidth) {
    val can_sel = sels(i).orR
    val consumed = portInFiringWindow(i)
    val valid_reg = RegInit(false.B)
    val sel_reg = RegEnable(OHToUInt(sels(i)), sel_fire(i))

    valid_reg := valid_reg && !consumed || can_sel
    sel_fire(i) := (!valid_reg || consumed) && can_sel

    r_valid(i) := valid_reg
    r_sel(i) := sel_reg
  }

  // ===========================================================================
  // ---- Grant assembly: per-lane windows, pvdest + pvtmp from the same pool ----
  // ===========================================================================

  //@req-spec-rename.e4
  //@req-spec-vrf.d7
  // An out-of-range r_sel index WRAPS rather than faults, and a wrap is
  // indistinguishable from the double-allocation this module must not do.
  val selIdxW = log2Ceil(allocWidth)
  def selIdx(x: UInt): UInt = x(selIdxW - 1, 0)
  for (w <- 0 until coreWidth) {
    val members  = io.req_members(w)
    val laneBase = Mux(io.reqs(w), base(w), 0.U)
    val tmp_base = Mux(io.req_shared(w), laneBase + members, 0.U)

    assert(!io.reqs(w) || laneBase + members <= allocWidth.U,
      "VecFreeList: pvdest window ran past allocWidth -- demand exceeded one cycle's " +
      "selection and alloc_ok did not catch it (r_sel would wrap onto an older lane's PRN)")
    assert(!(io.reqs(w) && io.req_shared(w)) || tmp_base + members <= allocWidth.U,
      "VecFreeList: pvtmp window ran past allocWidth -- a shared OP.v needs 2*members " +
      "PRNs from one selection; see the allocWidth derivation")

    for (m <- 0 until maxGroupSize) {
      val pvdestIdx = Mux(m.U < members, laneBase + m.U, laneBase)
      io.alloc_pvdest(w)(m) := r_sel(selIdx(pvdestIdx))

      val pvtmpIdx = Mux(m.U < members, tmp_base + m.U, tmp_base)
      io.alloc_pvtmp(w)(m) := Mux(io.req_shared(w), r_sel(selIdx(pvtmpIdx)), 0.U)
    }

    when (io.alloc_fire(w)) {
      VecTrace.traceStruct("VecFreeList", "alloc", Seq(
        ("lane", w.U),
        ("pvdest", io.alloc_pvdest(w)(0)),
        ("nmem", io.req_members(w)),
        ("pvtmp", io.alloc_pvtmp(w)(0))))
    }
  }

  // ===========================================================================
  // ---- Free-list update: bit-vector ORs only ----
  // ===========================================================================

  //@req-spec-rename.f10
  //@req-spec-rename.f11
  val sel_mask = (sels zip sel_fire).map { case (s, f) => s & Fill(n, f) }.reduce(_ | _)

  // ===========================================================================
  // ---- Branch reclaim: unchanged ----
  // ===========================================================================

  //@req-spec-rename.f8
  //@req-spec-rename.e8
  //@req-spec-rename.h22
  def onehot(x: UInt): UInt = UIntToOH(x)(n - 1, 0)
  val allocs: Seq[UInt] = (0 until coreWidth).map { w =>
    val pvdest_oh = (0 until maxGroupSize).map(m => onehot(io.alloc_pvdest(w)(m))).reduce(_ | _)
    val pvtmp_oh = (0 until maxGroupSize).map(m => onehot(io.alloc_pvtmp(w)(m))).reduce(_ | _) &
      Fill(n, io.req_shared(w))
    pvdest_oh | pvtmp_oh
  }
  val alloc_masks = (allocs zip io.alloc_fire).scanRight(0.U(n.W)) {
    case ((a, f), m) => m | (a & Fill(n, f))
  }

  val br_deallocs = br_alloc_lists(io.brupdate.b2.uop.br_tag) & Fill(n, io.brupdate.b2.mispredict)

  for (i <- 0 until maxBrCount) {
    br_alloc_lists(i) := br_alloc_lists(i) & ~br_deallocs | alloc_masks(0)
  }
  if (enableSuperscalarSnapshots) {
    val br_slots = VecInit(io.ren_br_tags.map(_.valid)).asUInt
    for (i <- 0 until maxBrCount) {
      val list_req = VecInit(io.ren_br_tags.map(_.bits === i.U)).asUInt & br_slots
      val new_list = list_req.orR
      when(new_list) {
        br_alloc_lists(i) := Mux1H(list_req, alloc_masks)
      }
    }
  } else {
    assert(PopCount(io.ren_br_tags.map(_.valid)) <= 1.U,
      "VecFreeList: more than one ren_br_tags entry valid in the same cycle")
    val do_br_snapshot = io.ren_br_tags.map(_.valid).reduce(_ || _)
    val br_snapshot_tag = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_tags.map(_.bits))
    val br_snapshot_list = Mux1H(io.ren_br_tags.map(_.valid), alloc_masks)
    when(do_br_snapshot) {
      br_alloc_lists(br_snapshot_tag) := br_snapshot_list
    }
  }

  // ===========================================================================
  // ---- Commit reclaim ----
  // ===========================================================================

  //@req-spec-rename.f9
  //@req-spec-rename.f13
  val dealloc_r = RegNext(io.dealloc)
  val com_deallocs_pvdest = dealloc_r
    .map(d => UIntToOH(d.bits)(n - 1, 0) & Fill(n, d.valid))
    .reduce(_ | _)

  //@req-spec-rename.e9
  val dealloc_tmp_r = io.dealloc_tmp.map(RegNext(_))
  val com_deallocs = dealloc_tmp_r match {
    case Some(dtr) =>
      com_deallocs_pvdest | dtr.map(d => UIntToOH(d.bits)(n - 1, 0) & Fill(n, d.valid)).reduce(_ | _)
    case None => com_deallocs_pvdest
  }

  for (i <- 0 until deallocWidth) {
    when (dealloc_r(i).valid) {
      VecTrace.traceStruct("VecFreeList", "free", Seq(
        ("slot", i.U), ("prn", dealloc_r(i).bits), ("tmp", 0.U)))
    }
  }
  dealloc_tmp_r.foreach { dtr =>
    for (i <- 0 until deallocWidth) {
      when (dtr(i).valid) {
        VecTrace.traceStruct("VecFreeList", "free", Seq(
          ("slot", i.U), ("prn", dtr(i).bits), ("tmp", 1.U)))
      }
    }
  }

  //@req-spec-rename.h17
  // SPEC DEFECT (reported, not resolved) -- cannot assert "no dealloc/dealloc_tmp slot
  // beyond committing group's member count is valid" (no per-commit-lane member count port).

  val rollback_deallocs = spec_alloc_list & Fill(n, io.rollback)
  val dealloc_mask = com_deallocs | br_deallocs | rollback_deallocs

  //@req-spec-rename.f10
  //@req-spec-rename.f11
  free_list := (free_list & ~sel_mask) | dealloc_mask
  spec_alloc_list := (spec_alloc_list | alloc_masks(0)) & ~dealloc_mask

  // ===========================================================================
  // ---- Observability, assertions ----
  // ===========================================================================

  val held_mask = (0 until allocWidth)
    .map(i => UIntToOH(r_sel(i))(n - 1, 0) & Fill(n, r_valid(i)))
    .reduce(_ | _)
  io.debug_freelist := free_list | held_mask

  assert(!(io.debug_freelist & dealloc_mask).orR,
    "VecFreeList: returning a free physical register")

  for (w <- 0 until coreWidth) {
    assert(!io.reqs(w) || (io.req_members(w) >= 1.U && io.req_members(w) <= maxGroupSize.U),
      "VecFreeList: req_members out of [1, maxGroupSize] range while reqs is asserted")
  }

  for (w <- 0 until coreWidth) {
    assert(!io.alloc_fire(w) || alloc_ok,
      "VecFreeList: alloc_fire asserted without alloc_ok")
  }

  for (w <- 0 until coreWidth) {
    for (k <- 0 until w) {
      assert(!(io.alloc_fire(w) && io.reqs(k)) || io.alloc_fire(k),
        "VecFreeList: alloc_fire is not prefix-shaped across requesting lanes")
    }
  }

  val firing_alloc_mask = (allocs zip io.alloc_fire).map { case (a, f) => a & Fill(n, f) }.reduce(_ | _)
  val firing_member_count = (0 until coreWidth)
    .map(w => Mux(io.alloc_fire(w), Mux(io.req_shared(w), io.req_members(w) << 1, io.req_members(w)), 0.U))
    .reduce(_ + _)
  assert(PopCount(firing_alloc_mask) === firing_member_count,
    "VecFreeList: duplicate PRN granted across firing lanes")

  // SPEC DEFECT (reported, not resolved) -- cannot assert "ren_br_tags(w+1).valid implies dis_fire(w)":
  // dis_fire not available as port, and alloc_fire(w) not equivalent (would fire falsely on scalar ops).
}

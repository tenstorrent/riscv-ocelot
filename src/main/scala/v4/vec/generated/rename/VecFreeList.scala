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
//
// VecFreeList -- the group-granular rename free list for the vector and VL
// physical register spaces. Instantiated TWICE by VecRenameSpace: once as the
// vector space (numPhysRegs = numVecPhysRegisters, maxGroupSize = 8,
// freeDiscipline = "stale_group") and once as the VL space (numPhysRegs =
// numVlPhysRegisters, maxGroupSize = 1, freeDiscipline = "committed_ptr").
//
// This is the vector analogue of `v4/exu/rename/rename-freelist.scala`
// (`RenameFreeList`) and deliberately keeps that module's structure and
// signal names (`free_list`, `spec_alloc_list`, `br_alloc_lists`, `sels`,
// `sel_fire`, `alloc_masks`, `sel_mask`, `dealloc_mask`, `debug_freelist`,
// ...). The ONLY structural change is that a request is a GROUP of up to
// `maxGroupSize` PRNs instead of one PRN.
//
// ALL-OR-NOTHING allocation of `pvdest` (+ `pvtmp` when shared) is the whole
// forward-progress argument: rename is in program order, so an op that
// cannot allocate simply STALLS while older ops keep committing and freeing
// PRNs -- no headroom reservation, no `numVecTmpGroups`. `alloc_ok` is ONE
// bit for the whole bundle and can never be partial; `alloc_fire` is PER LANE
// and routinely partial (see the M1 double-free discussion at `sel_fire`
// below).
//
// Elaborated only under `usingRVV` (a Scala Boolean, never rocket's
// `usingVector`): a vectors-off build has VecRenameSpace instantiate neither
// copy of this module, so this module performs no internal `usingRVV` gating
// of its own -- same convention as the sibling generated vec modules (e.g.
// `VConfigUnit`).
//
// TRACING -- all three mandated lines (event "stall" on `alloc_ok` low,
// event "alloc" per granted lane, event "free" per commit-side reclaim) are
// now emitted via `VecTrace`'s three-step ladder (`trace*` -> `traceId` ->
// `traceStruct`; see `VecTrace.scala`'s "two uOP-less variants" note). This
// module's ports (`reqs`/`req_members`/`req_shared`/`alloc_pvdest`/
// `alloc_pvtmp`/`alloc_ok`/`alloc_fire`/`dealloc`/`dealloc_tmp`/
// `ren_br_tags`/`brupdate`/`rollback`/`debug_freelist`/`stall_cnt_inc`)
// carry no `MicroOp` and no `rob_idx` for any lane -- by design, per this
// module's own `dependencies` note that it deliberately excludes `MicroOp`
// -- so the first two rungs are unreachable without inventing an identifier,
// which this flow's ground rules forbid; no port was added to reach a
// higher rung. This module's events are inherently about PRNs and
// free-list state, not instructions -- a group is allocated for one uOP and
// freed on behalf of another, so a `rob_idx` would often be dishonest even
// where one happened to be available -- so all three lines use
// `VecTrace.traceStruct`, keyed on `lane`/`pvdest`/`nmem`/`pvtmp` (alloc),
// `lane`/`demand`/`free` (stall), and `slot`/`prn`/`tmp` (free). See each
// call site below for the exact keys.
//
// Governing spec anchors: midcore.rst `free-list`, `cii-shared-mapping`,
// `vl-vtype-rename`, `regfiles-bypass`; issue.rst `cii-shared-sched`.

/**
 * VecFreeList ("freelist") -- see the file header for the full design
 * rationale. Instantiated twice by `VecRenameSpace`, once per rename space.
 *
 * @param numPhysRegs the size of the physical register space this instance
 *                     manages (`numVecPhysRegisters` or `numVlPhysRegisters`,
 *                     both from `VectorParams` -- neither is a literal here).
 * @param numArchRegs the PRNs the committed map table holds permanently: 32
 *                     for the vector space, 1 for the VL space. Used only by
 *                     the capacity `require` below; indexes nothing.
 * @param maxGroupSize the largest member count of one EMUL group
 *                      (`VectorParams.maxMembers`, 8 for the vector space, 1
 *                      for the VL space).
 * @param freeDiscipline "stale_group" (frees the whole `stale_pvdest` group
 *                        at commit -- the vector space) or "committed_ptr"
 *                        (frees the outgoing committed pointer, no
 *                        `stale_pvl` exists -- the VL space). Explicit rather
 *                        than derived from `maxGroupSize`: the two are
 *                        unrelated decisions that merely correlate in today's
 *                        two instances.
 */
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

  // named as in the scalar file so the expressions read the same.
  val pregSz = log2Ceil(numPhysRegs)
  val n = numPhysRegs

  //@req-spec-rename.f4
  //@req-spec-rename.f5
  // `allocWidth` is DERIVED, not passed: `coreWidth * maxGroupSize` -- the
  // number of prioritized outputs taken from `SelectFirstN`, because every
  // lane of a dispatch group may be an LMUL=8 OP.v taking a whole group at
  // once.
  val allocWidth = coreWidth * maxGroupSize

  //@req-spec-rename.f12
  // `deallocWidth` is likewise derived: `commitWidth * maxGroupSize`
  // (`retireWidth`, which equals `coreWidth`, times `maxGroupSize`) -- wide
  // enough for every commit lane to return a whole stale group in one cycle.
  // Commit cannot be back-pressured, so this width is a correctness
  // requirement, not a throughput choice.
  val deallocWidth = retireWidth * maxGroupSize

  // At least one full LMUL=8 group can always be renamed.
  require(numPhysRegs >= numArchRegs + 2 * maxGroupSize,
    s"numPhysRegs ($numPhysRegs) must be >= numArchRegs ($numArchRegs) + 2*maxGroupSize " +
    s"(${2 * maxGroupSize}): otherwise no full group can ever be renamed and the machine " +
    "deadlocks at rename")
  // A shared OP.v can always draw BOTH of its groups from one cycle's
  // selector output. This bites only at coreWidth == 1 (allocWidth ==
  // maxGroupSize, half of what a shared op needs), which must fail the build
  // rather than deadlock silently.
  require(allocWidth >= 2 * maxGroupSize,
    s"allocWidth ($allocWidth) must be >= 2*maxGroupSize (${2 * maxGroupSize}): a shared OP.v " +
    "needs both its groups from one cycle's selection, or the machine deadlocks rather than stalls")

  val io = IO(new Bundle {
    // Clock/reset: implicit, posedge clock, synchronous active-high reset --
    // the Chisel/BOOM default. Every register below is reset-initialized
    // from `initial_allocation` or to zero.
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
    // Present only when maxGroupSize > 1 (the VL space has no pvtmp).
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
  // Three registers, exactly as in the scalar free list: `free_list` (bit `i`
  // set iff PRN `i` is currently un-used); `spec_alloc_list` (everything
  // allocated since the last commit point, so a rollback can return it); and
  // `br_alloc_lists`, one per-branch-tag snapshot. State is one bit per PRN,
  // so the VL instance's free vector is exactly `numVlPhysRegisters` (64 at
  // the default) bits wide -- a plain bit vector over `numPhysRegs`,
  // identical in kind to the vector instance's wider one.
  val free_list = RegInit(UInt(numPhysRegs.W), io.initial_allocation)
  val spec_alloc_list = RegInit(0.U(numPhysRegs.W))
  val br_alloc_lists = Reg(Vec(maxBrCount, UInt(numPhysRegs.W)))

  // ===========================================================================
  // ---- Selection and the registered pre-selection stage ----
  // ===========================================================================

  //@req-spec-rename.f2
  //@req-spec-rename.f7
  // `sels` produces `allocWidth` one-hot selections of DISTINCT free PRNs in
  // priority order. Each output is an independent one-hot over the whole
  // space, so a group assembled from them is NON-CONTIGUOUS by construction
  // -- no contiguous-run allocator exists anywhere in this module.
  val sels = SelectFirstN(free_list, allocWidth)
  val sel_fire = Wire(Vec(allocWidth, Bool()))

  // ===========================================================================
  // ---- Demand, and the all-or-nothing decision ----
  // ===========================================================================

  //@req-spec-rename.f3
  //@req-spec-rename.f6
  // Per-lane demand: an entire EMUL group per OP.v, doubled when the lane
  // asked for TWO groups (`req_shared`, which is NOT `is_shared` -- a
  // segmented store asks for one). Sized to hold the worst case
  // (2*maxGroupSize) without truncation.
  val demandW = log2Ceil(2 * maxGroupSize + 1)
  val demand = Wire(Vec(coreWidth, UInt(demandW.W)))
  for (w <- 0 until coreWidth) {
    val doubled = Mux(io.req_shared(w), io.req_members(w) << 1, io.req_members(w))
    demand(w) := Mux(io.reqs(w), doubled, 0.U)
  }

  // A prefix sum over `demand` assigns each lane a contiguous WINDOW of
  // pre-selection PORTS: lane `w` takes ports `base(w) until base(w) +
  // demand(w)`. ASSUMPTION (documented, not a guess left silent): lane index
  // 0 is the OLDEST lane in the bundle (BOOM's dispatch-group convention --
  // `core.scala`'s `dis_stalls` prefix-scans from index 0 upward, and this
  // module's own "prefix-shaped fire" invariant below requires it), so
  // `base(w)` is defined here as the sum of the STRICTLY OLDER lanes'
  // (indices < w) demand. NOTE: the nlhdl source's prose literally says
  // "base(w) is the sum of the younger lanes' demands", which -- taken with
  // lane 0 as oldest -- would place lane 0's window at the END of the port
  // range and contradicts the unambiguous "consumed ports are exactly the
  // prefix [0, sum of the firing lanes' demands)" invariant stated later in
  // the same section. Flagged as a spec wording defect; resolved in favor of
  // the later, unambiguous statement, which is also the only reading under
  // which `alloc_fire`'s prefix-shape assertion (implemented below) can hold.
  val baseW = log2Ceil(coreWidth * 2 * maxGroupSize + 1)
  val base = demand.scanLeft(0.U(baseW.W))(_ + _) // base.length == coreWidth + 1
  val total_demand = base(coreWidth)

  //@req-spec-rename.e14
  //@req-spec-issue.c3
  //@req-spec-rename.e15
  // `alloc_ok` is the single all-or-nothing verdict: the total demand is
  // covered by valid pre-selection ports. Demand is computed from `reqs`,
  // which is fire-independent (see `sel_fire`'s note below), so `alloc_ok`
  // carries NO dependence on `alloc_fire` and cannot form a combinational
  // loop through `dis_fire`/`dis_ready`. Capacity is judged from the
  // registered `r_valid` outputs (declared below), available at the start of
  // the cycle rather than after the selector's priority chain.
  val r_valid = Wire(Vec(allocWidth, Bool()))
  val alloc_ok = (0 until allocWidth).map(i => r_valid(i) || i.U >= total_demand).reduce(_ && _)
  io.alloc_ok := alloc_ok

  //@req-spec-vrf.b11
  // When `alloc_ok` is low no lane allocates and nothing is consumed: the
  // whole dispatch group stalls and retries intact next cycle. This is the
  // ONLY place the vector-PRN capacity limit is enforced -- a consequence of
  // the free vector being short, not any explicit bound; there is NO
  // group-credit counter. `stall_cnt_inc` is high exactly when a requesting
  // lane was denied -- `alloc_ok` is trivially true when `total_demand == 0`
  // (every port index is `>= 0`), so `!alloc_ok` alone already implies some
  // lane requested.
  io.stall_cnt_inc := !alloc_ok

  // Tracing (see file header): no MicroOp/rob_idx on any lane -> traceStruct.
  // Keyed on `lane` (the oldest REQUESTING lane -- lane 0 is oldest per the
  // resolved `base`/window convention above, so `PriorityEncoder` over
  // `io.reqs` names it directly), `demand` (`total_demand`) and `free` (the
  // free-PRN count, `PopCount(free_list)`) -- the honest stand-ins for the
  // nlhdl's "oldest requesting lane's rob_idx, total_demand, free count"
  // now that no uOP identity is available on this boundary. Guarded on
  // `!alloc_ok`, which -- per the comment above -- already implies some
  // lane requested, so `PriorityEncoder(io.reqs)` names a real requester.
  when (!alloc_ok) {
    VecTrace.traceStruct("VecFreeList", "stall", Seq(
      ("lane", PriorityEncoder(io.reqs)),
      ("demand", total_demand),
      ("free", PopCount(free_list))))
  }

  // ===========================================================================
  // ---- Partial-prefix fire and per-lane consumption ----
  // ===========================================================================

  // Port `i` is CONSUMED this cycle iff it lies inside the window of some
  // lane `w` whose `alloc_fire(w)` is high. Computed directly from the
  // per-lane windows (not via the prefix-sum shortcut `i < sum of firing
  // demands`) so correctness does not depend on the prefix-fire invariant
  // that is otherwise only checked by assertion.
  def portInFiringWindow(i: Int): Bool =
    (0 until coreWidth).map { w =>
      io.alloc_fire(w) && i.U >= base(w) && i.U < (base(w) + demand(w))
    }.reduce(_ || _)

  // Keep the scalar file's per-port pre-selection pipeline verbatim: port `i`
  // holds `r_valid(i)` and `r_sel(i) = RegEnable(OHToUInt(sels(i)),
  // sel_fire(i))`, refilling when it is empty or its held PRN was consumed.
  // The PRN leaves `free_list` at PRE-SELECT time (`sel_mask` below), not at
  // grant time.
  //
  // A single bundle-wide fire bit here (rather than the per-lane
  // `io.alloc_fire(w)` qualifying `portInFiringWindow`) is the M1
  // leak/double-allocate: it would let a non-firing lane's window PRNs leave
  // `free_list` in a cycle that lane did not dispatch, that lane would then
  // be granted a SECOND group on retry, and the first is named by no uop --
  // gone until reset. Every consumption term in this module is therefore
  // qualified by `io.alloc_fire(w)` per lane, never by `alloc_ok` and never
  // by an OR-reduction of `alloc_fire`.
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
  // The `pvtmp` group is allocated FROM THIS FREE LIST -- the main vector
  // free list -- out of the same cycle's `SelectFirstN` output as `pvdest`,
  // through the same port window, with the same member count. It is not
  // drawn from a reserved region, a separate pool or a temp register file:
  // `tmp_base(w)` below is simply the port immediately following lane `w`'s
  // own `pvdest` members, inside that same lane's window.
  for (w <- 0 until coreWidth) {
    val members = io.req_members(w)
    val tmp_base = base(w) + members
    for (m <- 0 until maxGroupSize) {
      // For members beyond `req_members(w)`, pad with member 0's PRN (never
      // 0.U -- PRN 0 is a live register belonging to someone else, and an
      // unmasked consumer that iterated all `maxGroupSize` slots would free
      // it, which is the M1 double-free of PRN 0).
      val pvdestIdx = Mux(m.U < members, base(w) + m.U, base(w))
      io.alloc_pvdest(w)(m) := r_sel(pvdestIdx)

      // Meaningful only where `req_shared(w)`; pinned to 0 otherwise so an
      // unused pvtmp output never reads outside lane w's own window (a
      // non-shared lane's window is only `members` ports wide).
      val pvtmpIdx = Mux(m.U < members, tmp_base + m.U, tmp_base)
      io.alloc_pvtmp(w)(m) := Mux(io.req_shared(w), r_sel(pvtmpIdx), 0.U)
    }

    // Tracing (see file header): no MicroOp/rob_idx on this lane ->
    // traceStruct. One line per GRANTED lane (`io.alloc_fire(w)`, never a
    // bundle-wide OR -- same per-lane discipline as every consumption term
    // above). Keyed on `lane`, `pvdest` (the granted group's base PRN,
    // member 0 -- the same "base PRN + member count" convention
    // `VecTrace.tracePrn` uses), `nmem` (`req_members(w)`, == `v_emul`), and
    // `pvtmp` (the granted tmp group's base PRN; already forced to 0 by the
    // `io.alloc_pvtmp` Mux above when this lane is not `req_shared`, so the
    // field is always present and reads honestly either way).
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
  // Every mask below is a `numPhysRegs`-bit vector and every combination is
  // an OR or an AND-NOT: setting 8 bits for an OP.v is the same expression as
  // setting 1, with wider ports only.
  val sel_mask = (sels zip sel_fire).map { case (s, f) => s & Fill(n, f) }.reduce(_ | _)

  // ===========================================================================
  // ---- Branch reclaim: unchanged ----
  // ===========================================================================

  //@req-spec-rename.f8
  //@req-spec-rename.e8
  //@req-spec-rename.h22
  // `allocs(w)` is the OR of the one-hots of ALL PRNs lane `w` was granted:
  // `pvdest` members and `pvtmp` members together (the `req_shared` gate on
  // the pvtmp term keeps the pinned-0 pad above from spuriously OR-ing PRN 0
  // into a non-shared lane's contribution). `alloc_masks` is the scalar
  // file's `scanRight`, over LANES instead of ports, each lane contributing
  // QUALIFIED BY ITS OWN `alloc_fire(w)` -- a lane that did not fire
  // contributes ZERO, so its window's PRNs enter neither `spec_alloc_list`
  // nor any branch snapshot. (The nlhdl prose parenthetically calls this
  // "exactly as the scalar file qualifies by its per-port sel_fire" -- the
  // scalar file at `rename-freelist.scala` actually qualifies by `io.reqs`,
  // not `sel_fire`; a minor spec inaccuracy in the citation, not in the
  // instruction actually given, which unambiguously names `alloc_fire(w)`.)
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

  // No `despec` port and no `isImm` mode exist here (those serve the scalar
  // immediate free list only), so the branch-snapshot update always takes the
  // scalar file's non-`isImm` form.
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
  // Unchanged from the scalar file INCLUDING the `RegNext` -- that commit-
  // side pipe stage is part of the existing timing contract, not an
  // accident. With `deallocWidth = commitWidth * maxGroupSize` a commit lane
  // presents every member of its `stale_pvdest` group in one cycle and the
  // whole stale group is freed together. Under "committed_ptr" the same OR
  // tree serves a single valid slot per lane.
  // Named once so the trace loop below reuses the SAME registers rather than
  // instantiating a second `RegNext` purely for tracing.
  val dealloc_r = RegNext(io.dealloc)
  val com_deallocs_pvdest = dealloc_r
    .map(d => UIntToOH(d.bits)(n - 1, 0) & Fill(n, d.valid))
    .reduce(_ | _)

  //@req-spec-rename.e9
  // `dealloc_tmp` is OR-ed into `com_deallocs` by the identical expression,
  // so a committing shared OP.v frees its `pvtmp` group in the SAME cycle as
  // its stale `pvdest` group. Present only when `maxGroupSize > 1`.
  val dealloc_tmp_r = io.dealloc_tmp.map(RegNext(_))
  val com_deallocs = dealloc_tmp_r match {
    case Some(dtr) =>
      com_deallocs_pvdest | dtr.map(d => UIntToOH(d.bits)(n - 1, 0) & Fill(n, d.valid)).reduce(_ | _)
    case None => com_deallocs_pvdest
  }

  // Tracing (see file header): no MicroOp/rob_idx on any commit lane ->
  // traceStruct. One line per valid commit-side reclaim SLOT (the same
  // `deallocWidth` granularity `dealloc`/`dealloc_tmp` themselves use, since
  // no per-lane member count exists here to regroup slots into whole groups
  // -- see the "no per-commit-lane member count" spec-defect note below).
  // Keyed on `slot`, `prn` (the freed PRN) and `tmp` (0 for a `dealloc`
  // slot, 1 for a `dealloc_tmp` slot), so `event=free` greps find every
  // commit-side reclaim at the PRN it actually frees.
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
  // Under `freeDiscipline == "committed_ptr"` the PRN freed at commit is the
  // OUTGOING committed pointer, driven onto `dealloc(w*maxGroupSize)` by
  // VecMapTable/VecRenameSpace from `com_stale_resps` (`exportComStale`) --
  // this module does not distinguish which discipline is in effect anywhere
  // in `com_deallocs_pvdest`/`com_deallocs` above: the two disciplines differ
  // ONLY in who drives `dealloc`, and everything from this point on is
  // shared, which is why one definition serves both spaces. There is no
  // `stale_pvl` uop field and this module expects none.
  //
  // SPEC DEFECT (reported, not resolved) -- the assertion "no dealloc/
  // dealloc_tmp slot beyond a committing group's member count is valid"
  // cannot be implemented here: this module has no port carrying a per-
  // commit-lane member count (`v_emul`) to check `dealloc`'s valid slots
  // against, and inferring it from which slots happen to be invalid would
  // check nothing the parent didn't already guarantee. Omitted.

  val rollback_deallocs = spec_alloc_list & Fill(n, io.rollback)
  val dealloc_mask = com_deallocs | br_deallocs | rollback_deallocs

  //@req-spec-rename.f10
  //@req-spec-rename.f11
  // The state update is the scalar file's verbatim (no `despec`/`isImm`
  // term: there is no such concept here).
  free_list := (free_list & ~sel_mask) | dealloc_mask
  spec_alloc_list := (spec_alloc_list | alloc_masks(0)) & ~dealloc_mask

  // ===========================================================================
  // ---- Observability, assertions ----
  // ===========================================================================

  // The PRN leaves `free_list` at pre-select time, so `debug_freelist` adds
  // the held-but-ungranted PRNs (every currently-valid pre-selection
  // register, not just those actually granted to a lane this cycle) back
  // before the double-free assertion.
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

  // A fire implies the grant: `alloc_ok` gates `dis_ready`, which gates every
  // lane's `dis_fire`, so `alloc_fire(w)` may never be high while `alloc_ok`
  // is low.
  for (w <- 0 until coreWidth) {
    assert(!io.alloc_fire(w) || alloc_ok,
      "VecFreeList: alloc_fire asserted without alloc_ok")
  }

  // Fires are PREFIX-SHAPED: if lane `w` fires then every REQUESTING lane
  // `k < w` fires too (lane 0 is the oldest -- see the `base`/window
  // ASSUMPTION note above). Violating it means dispatch is no longer a
  // prefix scan and the port-window assignment must be revisited.
  for (w <- 0 until coreWidth) {
    for (k <- 0 until w) {
      assert(!(io.alloc_fire(w) && io.reqs(k)) || io.alloc_fire(k),
        "VecFreeList: alloc_fire is not prefix-shaped across requesting lanes")
    }
  }

  // No PRN appears twice across the granted members of the lanes whose
  // `alloc_fire(w)` is high: the total distinct-PRN count granted to firing
  // lanes (sum of each firing lane's real, un-padded member count) must
  // equal the population count of the OR of their one-hot masks -- equality
  // holds iff no bit was set by more than one firing lane.
  val firing_alloc_mask = (allocs zip io.alloc_fire).map { case (a, f) => a & Fill(n, f) }.reduce(_ | _)
  val firing_member_count = (0 until coreWidth)
    .map(w => Mux(io.alloc_fire(w), Mux(io.req_shared(w), io.req_members(w) << 1, io.req_members(w)), 0.U))
    .reduce(_ + _)
  assert(PopCount(firing_alloc_mask) === firing_member_count,
    "VecFreeList: duplicate PRN granted across firing lanes")

  // SPEC DEFECT (reported, not resolved) -- "assert that ren_br_tags(w+1)
  // .valid implies dis_fire(w)" cannot be implemented: `dis_fire` is not a
  // port of this module (it is not in the ports section), and
  // `io.alloc_fire(w)` is NOT an equivalent stand-in -- `alloc_fire(w)` is
  // defined as `dis_fire(w) && reqs(w)`, so a dispatching lane that did not
  // request a vector/VL group (`reqs(w)` false, e.g. a scalar op) would have
  // `alloc_fire(w)` false while legitimately allocating a branch tag,
  // making a substituted assertion fire falsely. Omitted rather than
  // implemented incorrectly.

  // The "emit one guarded VecTrace line per granted lane (event 'alloc')
  // and one per commit-side free (event 'free')" tracing convention (ground
  // rule 11; this paragraph cites no requirement ID of its own) is now
  // implemented -- see the file header's TRACING note and the `alloc`/
  // `free`/`stall` call sites above.
}

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

import boom.v4.common.{BoomBundle, BoomModule}
import boom.v4.exu.BrUpdateInfo

// GENERATED from src/main/nlhdl/vec/rename/VecMapTable.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
//
// VecMapTable -- the vector Rename Map Table: architectural vreg -> physical
// GROUP, read EMUL-wide and written atomically per group. The vector analogue
// of `class RenameMapTable` in v4/exu/rename/rename-maptable.scala, kept as
// close as possible to that module's structure and names
// (map_table/com_map_table/br_snapshots/remap_table/map_reqs/map_resps/
// remap_reqs/com_remap_reqs/ren_br_tags/rollback). Diffed against it there
// are exactly two differences: mappings are read/written EMUL-wide (a member
// vector, not a scalar), and the stale read is itself a Vec.
//
// ONE PRN PER ARCHITECTURAL VREG. That single property is the whole design.
// An EMUL-wide read is just EMUL adjacent row reads, correct however badly
// the physical registers are fragmented, so a contiguous-run allocator, a
// whole-group validity check and a fragmentation-recovery walk are ABSENT BY
// CONSTRUCTION -- see the "structures deliberately absent" section below.
//
// NO LMUL TAG TABLE AND NO WHOLE-GROUP CHECKER (hierarchy.yaml's own
// decision on this node). Atomic group rename makes every read whole-group
// by construction, so a checker could only confirm what the mapper already
// guarantees, at the cost of 32x2b of state and a comparator tree on the
// rename critical path. Do not add either back.
//
// ONE DEFINITION, TWO INSTANCES. VecRenameSpace instantiates this module
// twice: the vector space (numArchRegs = 32, maxGroupSize = 8, numPhysRegs =
// numVecPhysRegisters) and the VL space (numArchRegs = 1, maxGroupSize = 1,
// numPhysRegs = numVlPhysRegisters). The VL instance is not special-cased
// anywhere below -- every group construct degenerates at elaboration when
// maxGroupSize/numArchRegs collapse to 1 (a size-1 `Vec` indexed dynamically
// always returns its one element, so the row decode becomes constant 0 with
// no extra logic).
//
// Elaborated only when `usingRVV` is true, and not internally gated on it:
// this module reads no `vectorParams` of its own (every size comes in as a
// constructor parameter), so it is simply not instantiated by VecRenameSpace
// when vectors are off, and a non-vector build is bit-identical to
// pre-Caracal BOOM v4 -- the same convention VConfigUnit documents for
// itself.
//
// SPEC DEFECT (reported, not resolved) -- VecMapReq/VecMapResp/VecRemapReq
// ARE DECLARED LOCALLY, NOT IN VecBundles. This file's own nlhdl source
// (dependencies section) states "VecBundles -- VecMapReq, VecMapResp and
// VecRemapReq are declared there, not here". That is contradicted by two
// independent sources: (1) the actually-generated
// src/main/scala/v4/vec/generated/VecBundles.scala contains no such
// declarations anywhere, and (2) a sibling spec,
// src/main/nlhdl/vec/VecPipeline.nlhdl.scala part 13 ("Where the homeless
// bundles live"), explicitly settles this the other way: "VecMapReq /
// VecMapResp / VecRemapReq STAY LOCAL to their producers, mirroring baseline
// BOOM (which declares `class BusyResp` beside its user...)". Resolution:
// follow the settled, corroborated decision -- declare the three bundles
// locally in this file (below), exactly beside the module that is their one
// producer/consumer pair with VecRenameSpace, using the field lists this
// file's own ports section already gives verbatim. This is not inventing
// spec content (the field lists are given), only choosing the file that was
// actually settled as their home.
//
// SPEC DEFECT (reported, not resolved) -- NO GUARDED VecTrace CALLS ARE
// EMITTED. The logic section calls for three: one per remap (`tracePrn` with
// `rob_idx`, `lvd`, `emul`, installed members), one per stale-group capture,
// one per recovery event (arm + `br_tag`). `VecTrace.tracePrn`/`trace` both
// require a `MicroOp` (to read `rob_idx` off), and `VecTrace.traceDecode`
// requires `ftq_idx`/`pc_lob`. This module's ports section (as written) has
// none of the three: no `MicroOp`, no `rob_idx`, no `ftq_idx`/`pc_lob`
// anywhere on `map_reqs`/`map_resps`/`remap_reqs`/`com_remap_reqs`/
// `ren_br_tags`/`brupdate`/`rollback`. Inventing a fabricated rob_idx (e.g.
// tagging with `0.U`) would silently alias with a real ROB entry 0 in every
// grep -- exactly the failure mode `VConfigUnit`'s own SPEC DEFECT note (and
// `VecTrace.traceDecode`'s doc comment) already flags as worse than omitting
// the line. All three trace call sites are therefore omitted, flagged inline
// below. Ground rule 11 is otherwise honored: the assertions this section
// also calls for (duplicate-mapping, `emul` range, group-overflow) ARE
// implemented, since they need no identifier to be correct.
//
// Governing spec anchors: midcore.rst `rmt`, `rename-stage`, `snapshots`,
// `cii-shared-mapping`, `vl-vtype-rename`, `old-vd`; glossary.rst
// `glossary-terms`.

/**
 * VecMapReq -- one lane's rename-read request. `lvd`/`lvs1`/`lvs2`/`lvs3`/
 * `lvm` are `lregSz` wide to match the `MicroOp` fields of the same names;
 * only the low `lvregSz` bits actually index the table (see
 * [[VecMapTable.lvregSz]]). `emul` is the group member COUNT (1..maxGroupSize),
 * `valid` is the lane's `is_vec` ("produces or reads VL" for the VL
 * instance). `valid` is carried for the requester's own bookkeeping only --
 * unlike the scalar `MapReq`, reads here are always computed combinationally
 * regardless of it; correctness of using an invalid lane's response is
 * arbitrated downstream (VecRenameSpace / dispatch), not here.
 */
class VecMapReq(val emulSz: Int)(implicit p: Parameters) extends BoomBundle
{
  val lvd   = UInt(lregSz.W)
  val lvs1  = UInt(lregSz.W)
  val lvs2  = UInt(lregSz.W)
  val lvs3  = UInt(lregSz.W)
  val lvm   = UInt(lregSz.W)
  val emul  = UInt(emulSz.W)
  val valid = Bool()
}

/**
 * VecMapResp -- one lane's rename-read response. `pvs1`/`pvs2`/`pvs3`/
 * `stale_pvdest` are the EMUL-wide member vectors (members at index >= the
 * echoed `v_emul` are don't-care); `pvm` is a single PRN (the mask register
 * is never a group); `v_emul` is `emul` echoed back so exactly one structure
 * is the authority on how many members are meaningful.
 */
class VecMapResp(val pregSz: Int, val maxGroupSize: Int, val emulSz: Int)(implicit p: Parameters) extends BoomBundle
{
  val pvs1         = Vec(maxGroupSize, UInt(pregSz.W))
  val pvs2         = Vec(maxGroupSize, UInt(pregSz.W))
  val pvs3         = Vec(maxGroupSize, UInt(pregSz.W))
  val stale_pvdest = Vec(maxGroupSize, UInt(pregSz.W))
  val pvm          = UInt(pregSz.W)
  val v_emul       = UInt(emulSz.W)
}

/**
 * VecRemapReq -- installs a freshly allocated group (from VecFreeList, via
 * VecRenameSpace) under architectural name `lvd`, atomically over its `emul`
 * members, when `valid`. Used identically for the speculative install
 * (`remap_reqs`) and, as the same bundle, the committed install
 * (`com_remap_reqs`).
 *
 * WIDTH ASSUMPTION: the ports section states `lvd`'s width (`lregSz`, low
 * `lvregSz` bits index the table) explicitly only for `VecMapReq`. `lvd` here
 * names the same architectural specifier field, so this file reuses the
 * identical convention (full `lregSz`, truncated to `lvregSz` at every use)
 * rather than inventing a narrower dedicated width -- the nlhdl source is
 * silent on this specific field's width and this is the conservative,
 * consistent reading.
 */
class VecRemapReq(val pregSz: Int, val maxGroupSize: Int, val emulSz: Int)(implicit p: Parameters) extends BoomBundle
{
  val lvd    = UInt(lregSz.W)
  val pvdest = Vec(maxGroupSize, UInt(pregSz.W))
  val emul   = UInt(emulSz.W)
  val valid  = Bool()
}

/**
 * VecMapTable -- see the file header for the full design rationale.
 * Instantiated once per rename space by `VecRenameSpace`, as `maptable`.
 *
 * @param plWidth        rename lanes per cycle (legal 1..coreWidth; the
 *                        vector-space instance passes coreWidth). No Scala
 *                        default: the nlhdl parameters section states
 *                        "default coreWidth" as the value callers should
 *                        pass, but a default expression here cannot reach
 *                        the implicit `p` a `coreWidth` lookup would need
 *                        before `p` itself is bound -- the same reason the
 *                        scalar `RenameMapTable` also takes every size
 *                        parameter explicitly, with no defaults at all.
 * @param numArchRegs    architectural registers in this space: 32 (vector,
 *                        fixed by RVV) or 1 (VL).
 * @param maxGroupSize   most registers one instruction may rename atomically:
 *                        8 (LMUL/EMUL <= 8) or 1 (VL).
 * @param numPhysRegs    physical registers in this space's free-list/busy-
 *                        table domain: numVecPhysRegisters or
 *                        numVlPhysRegisters, read by the caller from
 *                        VectorParams and passed in -- never recomputed here.
 * @param bypass         build the in-bundle prefix bypass (default true for
 *                        both instances; false only to rule it in/out during
 *                        bring-up).
 * @param exportComStale expose `com_stale_resps` (default false; VecRenameSpace
 *                        sets true only for the VL instance, whose free
 *                        discipline is `committed_ptr`).
 */
class VecMapTable(
  val plWidth:        Int,
  val numArchRegs:    Int,
  val maxGroupSize:   Int,
  val numPhysRegs:    Int,
  val bypass:         Boolean = true,
  val exportComStale: Boolean = false)
  (implicit p: Parameters) extends BoomModule
{
  private def isPow2(x: Int): Boolean = x > 0 && (x & (x - 1)) == 0
  require(isPow2(numArchRegs), s"numArchRegs ($numArchRegs) must be a power of two")
  require(isPow2(maxGroupSize), s"maxGroupSize ($maxGroupSize) must be a power of two")
  require(maxGroupSize <= numArchRegs,
    s"maxGroupSize ($maxGroupSize) must be <= numArchRegs ($numArchRegs)")

  // ---- Derived widths (parameters section: "No width in this file is a
  // literal") ----
  val pregSz  = log2Ceil(numPhysRegs)
  val lvregSz = math.max(log2Ceil(numArchRegs), 1)
  val emulSz  = log2Ceil(maxGroupSize) + 1

  val io = IO(new BoomBundle()(p) {
    val map_reqs       = Input(Vec(plWidth, new VecMapReq(emulSz)))
    val map_resps       = Output(Vec(plWidth, new VecMapResp(pregSz, maxGroupSize, emulSz)))

    val remap_reqs      = Input(Vec(plWidth, new VecRemapReq(pregSz, maxGroupSize, emulSz)))
    val com_remap_reqs  = Input(Vec(plWidth, new VecRemapReq(pregSz, maxGroupSize, emulSz)))
    val com_stale_resps = if (exportComStale) Some(Output(Vec(plWidth, Vec(maxGroupSize, UInt(pregSz.W))))) else None

    val ren_br_tags = Input(Vec(plWidth + 1, Valid(UInt(brTagSz.W))))
    val brupdate    = Input(new BrUpdateInfo)
    val rollback    = Input(Bool())
  })

  // =========================================================================
  // ---- State ----
  // =========================================================================

  //@req-spec-rename.d7
  // ONE PRN PER ARCHITECTURAL VREG and nothing else -- no per-entry group
  // size, no base+count descriptor, no validity bit, no LMUL tag.
  // `com_map_table` mirrors it for the committed side, `br_snapshots` is one
  // full speculative-table copy per outstanding branch. All reset to the
  // identity mapping ARN i -> PRN i, as the scalar table does -- which is
  // also why the low `numArchRegs` PRNs of this space are permanently
  // committed state and never enter the free list.
  //@req-spec-rename.h7
  //@req-spec-rename.h19
  // With numArchRegs = 1 and maxGroupSize = 1 (the VL instance) all three of
  // the declarations below collapse at elaboration: `map_table` IS the VL
  // space's current-PRN pointer, `com_map_table` IS its single committed
  // entry, and every group construct in this file folds away -- no separate
  // VlMapTable module exists or is written here.
  val map_table     = RegInit(VecInit((0 until numArchRegs).map(i => i.U(pregSz.W))))
  val com_map_table = RegInit(VecInit((0 until numArchRegs).map(i => i.U(pregSz.W))))
  val br_snapshots  = Reg(Vec(maxBrCount, Vec(numArchRegs, UInt(pregSz.W))))

  //@req-spec-core.h4
  //@req-spec-rename.e10
  // pvtmp IS NEVER INSTALLED IN THIS TABLE, deliberately: the temp group a
  // shared (segmented) instruction allocates has no architectural name, no
  // reserved row and no third remap port here. The binding lives only in the
  // OP.v's own `pvtmp` field (declared on MicroOp, not here); excluding it
  // from this table guarantees no architectural read (`lvs*`) can ever alias
  // the rendezvous buffer.

  // =========================================================================
  // ---- Per-lane destination-group bounds (shared by the group write, the
  // read-side prefix bypass, and the committed-stale export below -- one
  // definition, so the range test used everywhere is provably the same one) ----
  // =========================================================================

  private def loOf(lvd: UInt): UInt = lvd(lvregSz - 1, 0)
  // `+&` (not `+`) so the upper bound is exact at lo + emul == numArchRegs
  // (the emul-range assert below makes that the only way to reach it) rather
  // than silently wrapping in a truncated width.
  private def hiOf(lo: UInt, emul: UInt): UInt = lo +& emul

  val remap_lo = io.remap_reqs.map(r => loOf(r.lvd))
  val remap_hi = (remap_lo zip io.remap_reqs).map { case (lo, r) => hiOf(lo, r.emul) }

  val com_remap_lo = io.com_remap_reqs.map(r => loOf(r.lvd))
  val com_remap_hi = (com_remap_lo zip io.com_remap_reqs).map { case (lo, r) => hiOf(lo, r.emul) }

  // =========================================================================
  // ---- The EMUL-wide group read, with the in-bundle prefix bypass ----
  // =========================================================================
  //
  // RVV requires a group's base register to be a multiple of EMUL, so
  // `base + m` for a valid member (m < emul) needs no adder: the low
  // log2(emul) bits of `base` are architecturally zero, so bitwise OR
  // computes the same row index a real add would -- a mux over the legal
  // member-count shapes (1, 2, 4, 8) implemented as gates, not a dynamic
  // adder per member, per the perf section's timing callout. Members at
  // m >= emul read a row too (the OR is unconditional) but the value is
  // don't-care there, exactly as the ports section specifies.
  private def groupRow(specBase: UInt, m: Int): UInt =
    loOf(specBase) | m.U(lvregSz.W)

  // THE BYPASS COMPARE IS PER MEMBER AND AGAINST A RANGE, NOT AGAINST THE
  // BASE (perf/logic section callout): comparing the read row only to an
  // older lane's `lvd` (the scalar table's test) would miss every
  // sub-range read where an older wide group's write covers this row without
  // the bases matching. Bypassed value is that lane's own
  // `pvdest(row - lo)`.
  private def bypassFold(i: Int, row: UInt): UInt = {
    val raw = map_table(row)
    if (!bypass) raw else {
      (0 until i).foldLeft(raw) { case (prev, k) =>
        val inGroup = io.remap_reqs(k).valid && row >= remap_lo(k) && row < remap_hi(k)
        Mux(inGroup, io.remap_reqs(k).pvdest(row - remap_lo(k)), prev)
      }
    }
  }

  //@req-spec-rename.d8
  //@req-spec-rename.d9
  // A source group read is `maxGroupSize` adjacent row reads: because each
  // row independently names one PRN, the read returns the group's CURRENT
  // mappings directly and is correct under arbitrary fragmentation.
  private def groupReadBypassed(i: Int, specBase: UInt): Vec[UInt] =
    VecInit((0 until maxGroupSize).map(m => bypassFold(i, groupRow(specBase, m))))

  // `pvm` gets no member loop and no `emul`: the mask register is one row,
  // never a group.
  private def singleReadBypassed(i: Int, specBase: UInt): UInt =
    bypassFold(i, loOf(specBase))

  for (i <- 0 until plWidth) {
    //@req-spec-rename.a10
    // These reads (plus the mask read) ARE the mapper's renaming of
    // lvd/lvs*/lvm to pvdest/pvs*/pvm: sources come straight from the reads,
    // and pvdest itself is not produced here at all -- it arrives on
    // `remap_reqs` and this table only installs it (group-write section
    // below).
    io.map_resps(i).pvs1 := groupReadBypassed(i, io.map_reqs(i).lvs1)
    io.map_resps(i).pvs2 := groupReadBypassed(i, io.map_reqs(i).lvs2)
    //@req-spec-vrf.j7
    // `pvs3` and `stale_pvdest` are two independent reads at two independent
    // specifiers (`lvs3` and `lvd`) -- never compared, never collapsed, so a
    // read-modify-write op (lvs3 == lvd) naturally gets the same group in
    // both fields and an op with a different/absent third source naturally
    // gets a different one, with no special case either way.
    io.map_resps(i).pvs3 := groupReadBypassed(i, io.map_reqs(i).lvs3)
    io.map_resps(i).pvm  := singleReadBypassed(i, io.map_reqs(i).lvm)

    //@req-spec-rename.d3
    //@req-spec-rename.d4
    //@req-spec-rename.d6
    //@req-spec-vrf.i1
    //@req-spec-vrf.i2
    //@req-spec-vrf.i3
    //@req-spec-vrf.j8
    // The lane reads its OWN destination specifier (lvd) through the same
    // EMUL-wide structure, BEFORE this lane's own remap is installed (the
    // bypass fold above only sees STRICTLY OLDER lanes k < i), so it returns
    // the architectural OLD-vd group -- the only legitimate source of
    // undisturbed lanes (tail/masked-off/vstart prefix) and what lets commit
    // free the group. Captured in the SAME rename cycle as the source reads,
    // with no second table and no pipeline register between the read and
    // this response.
    io.map_resps(i).stale_pvdest := groupReadBypassed(i, io.map_reqs(i).lvd)

    //@req-spec-rename.d14
    // EMUL itself is not recomputed here; it is echoed from the request onto
    // the response (and from there into the OP.v's `v_emul` field) so
    // exactly one structure is the authority on member count.
    io.map_resps(i).v_emul := io.map_reqs(i).emul
  }

  // =========================================================================
  // ---- The group write ----
  // =========================================================================

  //@req-spec-rename.d2
  //@req-spec-rename.i15
  // Installation is ATOMIC PER GROUP: a valid lane writes all `emul` rows
  // lvd..lvd+emul-1 in one cycle from pvdest(0..emul-1). Built as the scalar
  // table does, generalized from a bit to a member: `remap_table` is the
  // scanLeft of `map_table` over the lanes, per row, with lane k overriding
  // row i when row i falls inside lane k's destination group (the shared
  // range test above) with value pvdest(i - lvd). `map_table` normally takes
  // remap_table(plWidth) -- the table simply ADVANCES THROUGH ITS REMAP
  // REQUESTS, no recovery source involved -- and `com_map_table` always takes
  // com_remap_table(plWidth).
  val remap_table     = Wire(Vec(plWidth + 1, Vec(numArchRegs, UInt(pregSz.W))))
  val com_remap_table = Wire(Vec(plWidth + 1, Vec(numArchRegs, UInt(pregSz.W))))

  for (i <- 0 until numArchRegs) {
    val row = i.U(lvregSz.W)

    val remappedRow = (0 until plWidth).scanLeft(map_table(i)) { (pdst, k) =>
      val inGroup = io.remap_reqs(k).valid && row >= remap_lo(k) && row < remap_hi(k)
      Mux(inGroup, io.remap_reqs(k).pvdest(row - remap_lo(k)), pdst)
    }
    val comRemappedRow = (0 until plWidth).scanLeft(com_map_table(i)) { (pdst, k) =>
      val inGroup = io.com_remap_reqs(k).valid && row >= com_remap_lo(k) && row < com_remap_hi(k)
      Mux(inGroup, io.com_remap_reqs(k).pvdest(row - com_remap_lo(k)), pdst)
    }

    for (j <- 0 until plWidth + 1) {
      remap_table(j)(i)     := remappedRow(j)
      com_remap_table(j)(i) := comRemappedRow(j)
    }
  }

  //@req-spec-rename.h20
  // For the VL instance, `exportComStale` presents -- per commit lane, before
  // that lane's own com_remap_req is applied -- the value the committed
  // install DISPLACES (the PRN VecFreeList frees): `com_remap_table(k)` is
  // exactly the committed table's state after lanes 0..k-1 and before lane
  // k, so reading lane k's own destination group out of it here is what
  // keeps a `stale_pvl` field out of every ROB entry. Read the same
  // EMUL-wide way as the speculative stale read above (folds away to a
  // single row at maxGroupSize = 1).
  if (exportComStale) {
    for (k <- 0 until plWidth) {
      io.com_stale_resps.get(k) := VecInit((0 until maxGroupSize).map { m =>
        com_remap_table(k)(com_remap_lo(k) | m.U(lvregSz.W))
      })
    }
  }

  // =========================================================================
  // ---- Snapshots and recovery ----
  // =========================================================================

  //@req-spec-rename.i9
  //@req-spec-rename.i10
  //@req-spec-rename.i6
  // Reuse BOOM's branch snapshot mechanism unchanged in structure, on the
  // SAME `ren_br_tags` event the scalar RMT snapshots on and indexed by the
  // SAME br_tag -- no vector-private tag space, no delayed-br_tag path, so
  // skew between this table and the scalar one would be a bug with nothing
  // to hide it. `enableSuperscalarSnapshots` is honoured both ways, exactly
  // as the scalar table honours it.
  if (enableSuperscalarSnapshots) {
    for (i <- 0 until plWidth + 1) {
      when (io.ren_br_tags(i).valid) {
        br_snapshots(io.ren_br_tags(i).bits) := remap_table(i)
      }
    }
  } else {
    assert(PopCount(io.ren_br_tags.map(_.valid)) <= 1.U,
      "VecMapTable: more than one ren_br_tags entry valid in the same cycle")
    val do_br_snapshot   = io.ren_br_tags.map(_.valid).reduce(_ || _)
    val br_snapshot_tag  = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_tags.map(_.bits))
    val br_snapshot_table = Mux1H(io.ren_br_tags.map(_.valid), remap_table)
    when (do_br_snapshot) {
      br_snapshots(br_snapshot_tag) := br_snapshot_table
    }
  }
  //@req-spec-rename.i11
  // NO PERIODIC SNAPSHOT, and no snapshot on any event but `ren_br_tags`
  // above: a per-ARN table has no group structure to reconstruct after a
  // rollback, so a periodic checkpoint would be pure area plus a second,
  // rarely exercised recovery path.

  //@req-spec-rename.i5
  //@req-spec-rename.i7
  //@req-spec-rename.h8
  // Recovery is a three-way priority on the map_table write: mispredict
  // restores a branch snapshot in ONE CYCLE (flushing only state younger
  // than the branch); else rollback copies the committed table in ONE CYCLE
  // (flushing everything in flight); else the normal group-write advance
  // above. Never a ROB one-entry-per-cycle walk-back -- the committed table
  // already holds the newest correct mapping before the trapping
  // instruction. For the VL instance these are the same two arms, one entry
  // wide.
  when (io.brupdate.b2.mispredict) {
    map_table := br_snapshots(io.brupdate.b2.uop.br_tag)
  } .elsewhen (io.rollback) {
    map_table := com_map_table
  } .otherwise {
    map_table := remap_table(plWidth)
  }
  com_map_table := com_remap_table(plWidth)

  // =========================================================================
  // ---- The structures that are deliberately absent ----
  // =========================================================================

  //@req-spec-rename.d10
  // NO CONTIGUOUS-RUN ALLOCATOR. Nothing above requires a group's PRNs to be
  // adjacent or even ordered: rows are read and written independently via
  // `pvdest(m)` and the group travels as an explicit member vector, never a
  // base + count. Any code here computing a member PRN as base + m rather
  // than reading pvdest(m) would reintroduce this requirement silently --
  // note that none of the code above does: `pvdest(row - lo)` and
  // `pvdest(m)` both index the member vector VecFreeList (elsewhere)
  // populated, they never synthesize a PRN from an offset.

  //@req-spec-rename.d11
  // NO WHOLE-GROUP VALIDITY CHECK. No signal, register or comparator
  // anywhere above asks whether a read returned a whole group: every row
  // read is a live mapping, whatever wrote it at whatever LMUL, so such a
  // check would have no failure case to report and no recovery to trigger.

  //@req-spec-rename.d12
  // NO FRAGMENTATION-RECOVERY WALK. Both restores above (mispredict,
  // rollback) are single-cycle whole-table writes; fragmentation is not a
  // state this table can be in, so there is nothing here for a walk to
  // repair.

  //@req-spec-rename.d13
  // NO LMUL TAG WHOLE VECTOR GROUP CHECKER: no 32x2-bit per-ARN tag array, no
  // per-LMUL base-ARN comparator tree, no tag write on the remap path exists
  // anywhere in this file. If that observability is ever wanted, add a
  // per-ARN "last-write EMUL" performance counter in a `perfEvents`
  // EventSet -- off the rename critical path -- not a checker here.

  // =========================================================================
  // ---- Assertions (simulation-only; feed no functional signal) ----
  // =========================================================================
  //
  // Carried over from the scalar table's duplicate-mapping assertion,
  // generalized to members: for every valid remap_req member PRN, assert
  // that PRN is not already in map_table -- the cheapest detector for a
  // free-list double-allocation. As in the scalar table this needs no
  // explicit "shortly after reset" suppression: both tables reset to the
  // identity mapping and the free list does not reissue those low PRNs, so
  // there is no reset-adjacent false positive to special-case.
  for (k <- 0 until plWidth) {
    for (m <- 0 until maxGroupSize) {
      val memberValid = io.remap_reqs(k).valid && m.U < io.remap_reqs(k).emul
      assert(!memberValid || !map_table.contains(io.remap_reqs(k).pvdest(m)),
        "VecMapTable: trying to write a duplicate mapping.")
    }
    assert(!io.remap_reqs(k).valid ||
      (io.remap_reqs(k).emul >= 1.U && io.remap_reqs(k).emul <= maxGroupSize.U),
      "VecMapTable: emul out of legal range (1..maxGroupSize) on a valid remap request.")
    assert(!io.remap_reqs(k).valid || remap_hi(k) <= numArchRegs.U,
      "VecMapTable: remap request's destination group overflows numArchRegs.")
  }

  // (trace, SPEC DEFECT -- see file header) A per-remap `tracePrn` line, a
  // per-stale-group-capture line, and a per-recovery-event line all belong
  // here, but no port on this module carries a rob_idx, MicroOp, or
  // ftq_idx/pc_lob to correctly tag any of the three with. All three are
  // omitted rather than tagged with a fabricated identifier.
}

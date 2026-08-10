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

import boom.v4.common.{BoomBundle, BoomModule, MicroOp, RT_VEC}
import boom.v4.exu.BrUpdateInfo
import boom.v4.vec.generated.{VecGroupDone, VecMemberRdy, VecTrace}

// GENERATED from src/main/nlhdl/vec/rename/VecRenameSpace.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
//
// VecRenameSpace -- ONE parameterized rename space: map table + free list +
// busy table + the in-bundle prefix bypass, wired into a single rename cycle.
// The vector analogue of `class RenameStage`
// (v4/exu/rename/rename-stage.scala, untouched, still serving INT/FP), kept
// as close as possible to that module's structure and names
// (ren2_alloc_reqs, ren2_alloc_fire, ren2_br_tags, map_reqs, remap_reqs,
// com_remap_reqs, com_valids, BypassAllocations).
//
// ONE DEFINITION, TWO INSTANCES. VecPipeline instantiates this module twice:
// `vec_rename` (numArchRegs 32, maxGroupSize 8, numPhysRegs
// numVecPhysRegisters, freeDiscipline "stale_group", wakeupKind
// "group_done") and `vl_rename` (numArchRegs 1, maxGroupSize 1, numPhysRegs
// numVlPhysRegisters, freeDiscipline "committed_ptr", wakeupKind
// "ready_bit"). `vectorInstance` below is exactly `wakeupKind ==
// "group_done"`; every VL construct in this file is a `maxGroupSize == 1` /
// `numArchRegs == 1` degeneration of the vector construct beside it -- there
// is deliberately no second module for VL.
//
// THE LOCKSTEP CONTRACT -- THE M1 FREE-LIST DOUBLE-FREE, AND WHERE IT
// HAPPENED. Allocation is driven from `ren2_uops` and `dis_fire`: the
// REGISTERED output of the scalar RenameStage's ren1->ren2 pipeline, and the
// dispatch fire of that same registered bundle. `freelist.io.alloc_fire(w)`
// below is driven from `dis_fire(w) && ren2_alloc_reqs(w)`, where
// `ren2_alloc_reqs` is itself built from the REGISTERED `ren2_uops` port --
// never from a combinationally-one-cycle-ahead `dec_uops`. See part 2 below
// for the exact expression.
//
// Elaborated only when `usingRVV` is true (never rocket's `usingVector`).
//
// SPEC DEFECT (reported, not resolved) -- PLWIDTH HAS NO WORKABLE DEFAULT AND
// MUST EQUAL coreWidth. The nlhdl parameters section states "plWidth --
// rename lanes. Default coreWidth (3)", and the hierarchy.yaml instantiation
// sites for both `vec_rename` and `vl_rename` pass no `plWidth` at all,
// implying a default is expected. Two independent facts make a Scala default
// here impossible AND make anything other than `plWidth == coreWidth`
// unsafe: (1) a default expression in this parameter list cannot read
// `coreWidth` because that name resolves through the *implicit* `p:
// Parameters` bound in the SECOND parameter list, which Scala does not make
// visible to defaults in the first -- the identical defect VecMapTable's own
// file already reports for its own `plWidth`; (2) VecFreeList (this module's
// own child) does not take a `plWidth` parameter at all -- its
// `reqs`/`req_members`/`req_shared`/`alloc_fire`/`alloc_pvdest`/
// `alloc_pvtmp` ports are sized directly from the bare `coreWidth` constant.
// A `plWidth != coreWidth` here would therefore be a width mismatch at the
// freelist boundary that neither file's spec calls out. Resolved
// conservatively: no default is given (the caller, VecPipeline, must pass
// `plWidth = coreWidth` explicitly at both instantiation sites) and a
// `require` below enforces the equality so a future caller error fails the
// build instead of miscompiling.
//
// SPEC DEFECT (reported, not resolved) -- `rob_unsafe` IS NOT WRITTEN HERE.
// Part 9 of the nlhdl logic section is explicit that no ROB-safety field is
// written by this module: "rob_unsafe for vector ops is cleared by the Rob
// delta's group-safe path (vec_clr_unsafe)... If the PNR assertion in
// rob.scala trips at bring-up on a vector op -- the M2 symptom -- the fix is
// that path, NOT a tie-off in the rename stage." This module has no port
// carrying a ROB index or an unsafe-clear channel (no such port appears
// anywhere in the ports section), so there is nothing here that could even
// drive such a signal. If the rob.scala:438-441 assertion trips on a vector
// op, the fix belongs to the (not-yet-built) Rob delta's `vec_clr_unsafe`
// path, not to a tie-off added in this file.
//
// Governing spec anchors: midcore.rst `midcore-rename`, `rename-stage`,
// `rmt`, `cii-shared-mapping`, `vl-vtype-rename`, `snapshots`; frontend.rst
// `vset-dual-dest`, `vl-delivery`.

/**
 * VecRenameSpace -- see the file header for the full design rationale.
 * Instantiated twice by VecPipeline, as `vec_rename` and `vl_rename`.
 *
 * @param plWidth         rename lanes per cycle. Must equal `coreWidth` (see
 *                        the file-header SPEC DEFECT); no default is offered
 *                        because a default here cannot reach the implicit
 *                        `p` a `coreWidth` lookup would need.
 * @param numArchRegs     architectural registers in this space: 32 (vector)
 *                        or 1 (VL).
 * @param maxGroupSize    most registers one instruction renames atomically:
 *                        `maxMembers` (8, vector) or 1 (VL).
 * @param numPhysRegs     physical registers in this space's free-list/busy-
 *                        table domain: `numVecPhysRegisters` or
 *                        `numVlPhysRegisters`, from VectorParams.
 * @param numWbPorts      completion ports on the busy-table clear side:
 *                        `numVecWbPorts` (vector) or the VL writeback port
 *                        count.
 * @param freeDiscipline  "stale_group" (frees the whole `stale_pvdest` group
 *                        at commit) or "committed_ptr" (frees the outgoing
 *                        committed pointer; no `stale_pvl` exists). EXPLICIT,
 *                        never derived from `maxGroupSize`.
 * @param wakeupKind      "group_done" (member-PRN vector wakeup) or
 *                        "ready_bit" (plain readiness bit, value read at
 *                        execute). EXPLICIT, never derived from
 *                        `maxGroupSize`.
 * @param hasRenameWrite  true for `vl_rename` only: gates the rename-cycle VL
 *                        register-file write port and its two input ports.
 *                        Implies `maxGroupSize == 1`.
 * @param exportMemberRdy true for `vec_rename` only: gates the per-member
 *                        source-readiness export (`member_rdy`) and is
 *                        passed straight down to VecBusyTable under the same
 *                        name. Implies `maxGroupSize > 1`.
 * @param bypass          passed to VecMapTable (build the in-bundle prefix
 *                        bypass). Default true for both instances.
 */
class VecRenameSpace(
  val plWidth:         Int,
  val numArchRegs:     Int,
  val maxGroupSize:    Int,
  val numPhysRegs:     Int,
  val numWbPorts:      Int,
  val freeDiscipline:  String,
  val wakeupKind:      String,
  val hasRenameWrite:  Boolean = false,
  val exportMemberRdy: Boolean = false,
  val bypass:          Boolean = true)
  (implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecRenameSpace: elaborated only under usingRVV (never rocket's usingVector)")

  //@req-spec-rename.h5
  // `freeDiscipline`/`wakeupKind` stay EXPLICIT, named parameters -- never
  // derived from `maxGroupSize` -- because the two instances genuinely
  // differ only in (a) which PRN commit releases and (b) what a wakeup
  // carries; both are independent of group size and of each other.
  require(freeDiscipline == "stale_group" || freeDiscipline == "committed_ptr",
    s"""VecRenameSpace: freeDiscipline ("$freeDiscipline") must be "stale_group" or "committed_ptr"""")
  require(wakeupKind == "group_done" || wakeupKind == "ready_bit",
    s"""VecRenameSpace: wakeupKind ("$wakeupKind") must be "group_done" or "ready_bit"""")
  require(maxGroupSize <= numArchRegs,
    s"VecRenameSpace: maxGroupSize ($maxGroupSize) must be <= numArchRegs ($numArchRegs)")
  require(!hasRenameWrite || maxGroupSize == 1,
    "VecRenameSpace: hasRenameWrite implies maxGroupSize == 1 -- a rename-cycle write of a " +
    "whole group is not defined and no register file offers it")
  require(!exportMemberRdy || maxGroupSize > 1,
    "VecRenameSpace: exportMemberRdy implies maxGroupSize > 1 -- a one-member group's " +
    "per-member vector carries no information the aggregate does not")
  // SPEC DEFECT (reported): see the file header. VecFreeList sizes every
  // per-lane port from the bare `coreWidth` constant, not from a `plWidth`
  // parameter, so this module can only be correct when the two are equal.
  require(plWidth == coreWidth,
    s"VecRenameSpace: plWidth ($plWidth) must equal coreWidth ($coreWidth) -- VecFreeList's " +
    "own ports are sized directly from coreWidth, not from a plWidth parameter")
  // ASSUMPTION (not stated by either file, added defensively): MicroOp's
  // pvdest/pvs1/pvs2/pvs3/pvtmp/stale_pvdest fields are always
  // Vec(maxVecMembers, ...), globally, regardless of which rename space wrote
  // them. This module copies those fields to/from VecMapTable's/VecFreeList's
  // own maxGroupSize-wide Vecs directly (no per-member remap), which is only
  // type-correct on the vector instance if maxGroupSize == maxVecMembers --
  // true by the hierarchy.yaml pairing (8 == 8) but not enforced anywhere
  // else, hence the require.
  if (wakeupKind == "group_done") require(maxGroupSize == maxVecMembers,
    s"VecRenameSpace: on the vector instance maxGroupSize ($maxGroupSize) must equal " +
    s"maxVecMembers ($maxVecMembers) -- MicroOp's pvdest/pvs*/pvtmp/stale_pvdest Vecs are " +
    "sized from maxVecMembers globally, and this module connects them directly to this " +
    "module's own maxGroupSize-wide children without a per-member remap")

  // ---- Derived widths (mirrors each child's own derivation exactly, since
  // numArchRegs/maxGroupSize/numPhysRegs are the SAME values passed to all
  // three) ----
  val pregSz  = log2Ceil(numPhysRegs)
  val lvregSz = math.max(log2Ceil(numArchRegs), 1)
  val emulSz  = log2Ceil(maxGroupSize) + 1

  // "vectorInstance" is not a new knob: it is exactly `wakeupKind ==
  // "group_done"`, the same reading VecBusyTable's own file already commits
  // to -- reading the spec, not inventing a distinguishing parameter.
  private val vectorInstance = wakeupKind == "group_done"

  val io = IO(new Bundle {
    // ---- The lockstep inputs (see file header) ----
    val ren2_uops  = Input(Vec(plWidth, new MicroOp))
    val ren2_mask  = Input(Vec(plWidth, Bool()))
    val dis_fire   = Input(Vec(plWidth, Bool()))

    // Only when hasRenameWrite (vl_rename): VConfigUnit's dec_vl_imm and its
    // frontend_only-qualified valid, registered by VecPipeline into the same
    // ren2 stage as the uop it belongs to.
    val ren2_vl_imm       = if (hasRenameWrite) Some(Input(Vec(plWidth, UInt(vecVLSz.W)))) else None
    val ren2_vl_imm_valid = if (hasRenameWrite) Some(Input(Vec(plWidth, Bool()))) else None

    val brupdate = Input(new BrUpdateInfo)
    val rollback = Input(Bool())

    val com_valids = Input(Vec(retireWidth, Bool()))
    val com_uops   = Input(Vec(retireWidth, new MicroOp))

    // Forwarded unmodified to the busy table; this module reads no field of
    // these. Payload type differs by wakeupKind, unified at Data and cast
    // back inside VecBusyTable with wakeupKind as the elaboration-constant
    // discriminant (mirrors VecBusyTableIO's own construction).
    val wakeups = Input(Vec(numWbPorts, Valid(
      (if (vectorInstance) new VecGroupDone else UInt(pregSz.W)): Data
    )))

    // ---- The outputs ----
    val ren2_uops_out = Output(Vec(plWidth, new MicroOp))
    val alloc_ok      = Output(Bool())

    // Only when exportMemberRdy (vec_rename).
    val member_rdy = if (exportMemberRdy) Some(Output(Vec(plWidth, new VecMemberRdy))) else None

    // Only when hasRenameWrite (vl_rename): VlRegFile's W_ren port,
    // replicated per rename lane and never arbitrated.
    val vl_rf_write = if (hasRenameWrite) Some(Output(Vec(plWidth, Valid(new Bundle {
      val addr = UInt(pregSz.W)
      val data = UInt(vecVLSz.W)
    })))) else None

    // Observability only -- nothing functional may read either.
    val debug_freelist  = Output(UInt(numPhysRegs.W))
    val debug_busytable = Output(Bits(numPhysRegs.W))
  })

  // ===========================================================================
  // ---- 1. One stage, three children, no state of its own ----
  // ===========================================================================
  //
  //@req-spec-rename.a1
  //@req-spec-rename.a5
  //@req-spec-rename.a6
  //@req-spec-rename.d1
  // This module extends the existing RenameStage implementation to vector
  // renaming in a SINGLE PIPELINE STAGE: it instantiates `maptable`,
  // `freelist` and `busytable`, wires them exactly as RenameStage wires its
  // scalar three, and declares NO register on the path from `io.ren2_uops` to
  // `io.ren2_uops_out` (see `uops_renamed` below -- a Wire, not a Reg). There
  // is consequently no separate vector-mapping pipeline stage and no
  // 1-cycle-delayed pipeline register: the ren1->ren2 registers this module
  // reads belong to the scalar RenameStage and are shared, not duplicated.
  //
  //@req-spec-rename.a2
  //@req-spec-rename.a3
  //@req-spec-rename.a4
  // For a vector instruction the scalar (INT/FP) rename and this vector group
  // rename run IN PARALLEL IN THE SAME CYCLE, over independent register
  // spaces, free lists and busy tables: nothing below reads `pdst`, `prs*`,
  // `stale_pdst` or any scalar busy bit, and nothing in the scalar
  // RenameStage reads a `pv*` field. The one shared signal is
  // `dis_fire`/`dis_ready`, a bundle-level stall and not a data dependence.
  val maptable = Module(new VecMapTable(
    plWidth        = plWidth,
    numArchRegs    = numArchRegs,
    maxGroupSize   = maxGroupSize,
    numPhysRegs    = numPhysRegs,
    bypass         = bypass,
    exportComStale = (freeDiscipline == "committed_ptr")))
  val freelist = Module(new VecFreeList(
    numPhysRegs    = numPhysRegs,
    numArchRegs    = numArchRegs,
    maxGroupSize   = maxGroupSize,
    freeDiscipline = freeDiscipline))
  val busytable = Module(new VecBusyTable(
    plWidth         = plWidth,
    numPregs        = numPhysRegs,
    maxGroupSize    = maxGroupSize,
    numWbPorts      = numWbPorts,
    wakeupKind      = wakeupKind,
    exportMemberRdy = exportMemberRdy))

  // ===========================================================================
  // ---- 2. Requests: which lanes rename in this space ----
  // ===========================================================================
  val ren2_uops = io.ren2_uops
  val ren2_mask = io.ren2_mask
  val dis_fire  = io.dis_fire
  val brupdate  = io.brupdate
  val rollback  = io.rollback

  // Destination predicate, per lane: `needs_pvdest || needs_pvtmp` for
  // vec_rename, `is_vl_producer` for vl_rename -- ORTHOGONAL to dst_rtype
  // (vsetvli x0, rs1, vtype has dst_rtype === RT_ZERO and still writes the VL
  // RF), so never inferred from it.
  val needs_pvdest: Seq[Bool] = ren2_uops.map(_.dst_rtype === RT_VEC)
  val needs_pvtmp:  Seq[Bool] = ren2_uops.map(u => u.is_vec.get && u.is_shared.get)
  val req_shared:   Seq[Bool] = (needs_pvdest zip needs_pvtmp).map { case (d, t) => d && t }
  val vl_producer:  Seq[Bool] = ren2_uops.map(_.is_vl_producer.get)
  val dest_pred: Seq[Bool] =
    if (vectorInstance) (needs_pvdest zip needs_pvtmp).map { case (d, t) => d || t }
    else vl_producer

  val ren2_alloc_reqs: Seq[Bool] = (0 until plWidth).map(w => ren2_mask(w) && dest_pred(w))
  //@req-spec-rename.h5 (repeated -- the two disciplines share this one event)
  val ren2_alloc_fire: Seq[Bool] = (0 until plWidth).map(w => dis_fire(w) && ren2_alloc_reqs(w))

  // Installation predicate: matches VecMapTable's own remap_reqs(k).valid
  // EXACTLY (needs_pvdest for vec_rename -- a tmp-only shared op has no
  // vector destination and installs nothing; is_vl_producer for vl_rename).
  // Reused, unmodified, by (a) maptable.io.remap_reqs.valid below and (b) the
  // readiness bypass's qualifier in part 6, so the two can never disagree by
  // construction.
  val remap_valid: Seq[Bool] = (0 until plWidth).map { w =>
    ren2_alloc_fire(w) && (if (vectorInstance) needs_pvdest(w) else vl_producer(w))
  }

  // ===========================================================================
  // ---- 3. Map table wiring ----
  // ===========================================================================
  val map_reqs       = Wire(Vec(plWidth, new VecMapReq(emulSz)))
  val remap_reqs     = Wire(Vec(plWidth, new VecRemapReq(pregSz, maxGroupSize, emulSz)))
  val com_remap_reqs = Wire(Vec(plWidth, new VecRemapReq(pregSz, maxGroupSize, emulSz)))

  for (w <- 0 until plWidth) {
    val u = ren2_uops(w)
    if (vectorInstance) {
      map_reqs(w).lvd  := u.lvd.get
      map_reqs(w).lvs1 := u.lvs1.get
      map_reqs(w).lvs2 := u.lvs2.get
      map_reqs(w).lvs3 := u.lvs3.get
      map_reqs(w).lvm  := u.lvm.get
      map_reqs(w).emul := u.v_emul.get
    } else {
      // For vl_rename all five specifiers are tied to 0 -- the space has one
      // row -- and the response's stale_pvdest(0) IS the current VL pointer.
      map_reqs(w).lvd  := 0.U
      map_reqs(w).lvs1 := 0.U
      map_reqs(w).lvs2 := 0.U
      map_reqs(w).lvs3 := 0.U
      map_reqs(w).lvm  := 0.U
      map_reqs(w).emul := 1.U
    }
    map_reqs(w).valid := ren2_mask(w) && u.is_vec.get

    if (vectorInstance) {
      remap_reqs(w).lvd    := u.lvd.get
      remap_reqs(w).emul   := u.v_emul.get
      remap_reqs(w).pvdest := freelist.io.alloc_pvdest(w)
    } else {
      remap_reqs(w).lvd       := 0.U
      remap_reqs(w).emul      := 1.U
      remap_reqs(w).pvdest(0) := freelist.io.alloc_pvdest(w)(0)
    }
    remap_reqs(w).valid := remap_valid(w)
  }

  maptable.io.map_reqs   := map_reqs
  maptable.io.remap_reqs := remap_reqs

  // ===========================================================================
  // ---- 4. Free list wiring, and the shared instruction ----
  // ===========================================================================
  freelist.io.initial_allocation := Cat(~(0.U((numPhysRegs - numArchRegs).W)), 0.U(numArchRegs.W))

  for (w <- 0 until plWidth) {
    freelist.io.reqs(w)        := ren2_alloc_reqs(w)
    // req_members(w) := v_emul, always 1 in the VL space (is_vl_producer
    // instructions never carry a real EMUL group).
    freelist.io.req_members(w) := ren2_uops(w).v_emul.get
    //@req-spec-rename.e1
    //@req-spec-rename.e3
    //@req-spec-rename.e5
    // req_shared is NOT is_shared: a segmented STORE needs pvtmp and no
    // pvdest (needs_pvdest false), so it must request only ONE group, not
    // two, or the second is granted but named by no uop field and leaked
    // forever. req_shared counts GROUPS to allocate.
    freelist.io.req_shared(w) := (if (vectorInstance) req_shared(w) else false.B)
    // ===========================================================================
    // THE KEY WIRE. `alloc_fire(w)` is driven from `dis_fire(w) &&
    // ren2_alloc_reqs(w)` -- i.e. `ren2_alloc_fire(w)`, itself built from the
    // REGISTERED `ren2_uops`/`ren2_mask` and the dispatch fire of that same
    // registered bundle. This is the M1 free-list double-free fix: never
    // driven from a combinational `dec_uops`/`dec_fire`, which would run this
    // module one cycle ahead of the scalar rename and free the same PRN
    // twice.
    // ===========================================================================
    freelist.io.alloc_fire(w) := ren2_alloc_fire(w)
  }

  // ===========================================================================
  // ---- 5. Busy table wiring, and the two busy bits of a vset ----
  // ===========================================================================
  //
  // uops_renamed is THE single Wire this space's renaming writes into, and is
  // ALSO io.ren2_uops_out combinationally (part 1/perf: zero cycles, no
  // register on this path).
  val uops_renamed = Wire(Vec(plWidth, new MicroOp))
  for (w <- 0 until plWidth) { uops_renamed(w) := ren2_uops(w) }

  if (vectorInstance) {
    for (w <- 0 until plWidth) {
      val resp = maptable.io.map_resps(w)
      // Response into the uop, per lane, taken from map_resps(w) VERBATIM:
      // an unencoded source's PRN is real (belongs to whoever owns lvs<s>'s
      // don't-care encoding) and is only ever suppressed via its busy bit and
      // member_rdy below (D11), never by zeroing the PRN here.
      uops_renamed(w).pvs1.get         := resp.pvs1
      uops_renamed(w).pvs2.get         := resp.pvs2
      // SPEC DEFECT (reported, not resolved): the nlhdl source tags this
      // sentence with a vrf-family req ID (group j, item 7), but that ID is
      // not in this module's own 33-ID hierarchy.yaml `reqs:` list -- it
      // belongs to, and is already correctly tagged by, VecMapTable (see
      // VecMapTable.scala:334, the identical "pvs3 and stale_pvdest are two
      // independent reads" sentence). Deliberately not re-tagged here (as a
      // literal req comment) to avoid claiming ownership hierarchy.yaml did
      // not allocate to this module.
      // pvs3 and stale_pvdest are two independent reads at two independent
      // specifiers (lvs3, lvd) -- never compared, never collapsed.
      uops_renamed(w).pvs3.get         := resp.pvs3
      uops_renamed(w).pvm.get          := resp.pvm
      uops_renamed(w).stale_pvdest.get := resp.stale_pvdest
      uops_renamed(w).v_emul.get       := resp.v_emul

      //@req-spec-rename.e1
      //@req-spec-rename.e3
      //@req-spec-rename.e5
      // pvdest/pvtmp are ALL-OR-NOTHING per the free list's own contract:
      // two-group case: pvdest := alloc_pvdest, pvtmp := alloc_pvtmp
      // tmp-only case : pvtmp  := alloc_pvdest, pvdest left UNWRITTEN
      // dest-only case: pvdest := alloc_pvdest, pvtmp  left UNWRITTEN
      when (needs_pvdest(w)) {
        uops_renamed(w).pvdest.get := freelist.io.alloc_pvdest(w)
      }
      when (needs_pvtmp(w)) {
        uops_renamed(w).pvtmp.get := Mux(needs_pvdest(w), freelist.io.alloc_pvtmp(w), freelist.io.alloc_pvdest(w))
      }
    }
  } else {
    //@req-spec-decode.i5
    //@req-spec-decode.i2
    // Every VL PRODUCER allocates a FRESH VL PRN; every younger vector uOP
    // carries the CURRENT pvl read from the VL map table at rename as an
    // implicit operand -- one read, one mux, no second structure. The
    // non-producing `vsetvli x0, x0` keep-VL form takes the consumer arm by
    // construction (is_vl_producer clear), so the surviving pvl is preserved.
    // The PRN half of this mux is VecMapTable's own bypass (trap 1, part 6):
    // this line must never be re-muxed a second time here.
    for (w <- 0 until plWidth) {
      uops_renamed(w).pvl.get := Mux(
        vl_producer(w) && ren2_alloc_fire(w),
        freelist.io.alloc_pvdest(w)(0),
        maptable.io.map_resps(w).stale_pvdest(0))
    }
  }

  // ---- bt_uops: the PRIVATE wire the busy table's set path reads ----
  //
  // For vec_rename this is exactly uops_renamed (pvdest/pvtmp already
  // correct). For vl_rename it is uops_renamed with pvdest(0) OVERRIDDEN to
  // hold the just-computed pvl -- because VecBusyTable's generic set-path
  // (part 6 of its own file) reads uop.pvdest unconditionally, and the VL
  // instance never writes a real pvdest group.
  //
  // vl_rename MUST NOT WRITE pvdest ON ITS OUTPUT PATH (uops_renamed / the
  // eventual io.ren2_uops_out): that field belongs to the vector space and
  // holds a real 7-bit vector group. bt_uops is therefore a SEPARATE wire,
  // never connected back to uops_renamed.
  val bt_uops = Wire(Vec(plWidth, new MicroOp))
  bt_uops := uops_renamed
  if (!vectorInstance) {
    // WIDTH ASSUMPTION: pvdest(0) is vecPregSz wide, pvl is vlPregSz wide
    // (narrower at the default sizing) -- a plain Chisel `:=` between the two
    // implicitly zero-extends/truncates to match, which is safe here because
    // every value written is a real, in-range PRN for whichever space it
    // came from.
    for (w <- 0 until plWidth) {
      bt_uops(w).pvdest.get(0) := uops_renamed(w).pvl.get
    }
  }
  busytable.io.ren_uops := bt_uops

  //@req-spec-decode.c21
  //@req-spec-decode.i3
  // rebusy_reqs(w) := ren2_alloc_fire(w), the SAME event the free list
  // allocates on, EXCEPT that vl_rename additionally suppresses it when
  // ren2_vl_imm_valid(w): a born-ready vsetivli's pvl needs no busy bit
  // because its value arrives WITH the allocation (vl_rf_write below, same
  // cycle). A register-sourced vset therefore sets BOTH busy bits -- the
  // scalar pdst busy in the untouched scalar RenameStage, and the VL busy
  // here -- and both wakeup networks later fire.
  for (w <- 0 until plWidth) {
    busytable.io.rebusy_reqs(w) := (if (hasRenameWrite)
      ren2_alloc_fire(w) && !io.ren2_vl_imm_valid.get(w)
    else
      ren2_alloc_fire(w))
  }
  busytable.io.wakeups := io.wakeups

  if (hasRenameWrite) {
    for (w <- 0 until plWidth) {
      io.vl_rf_write.get(w).valid     := dis_fire(w) && io.ren2_vl_imm_valid.get(w)
      io.vl_rf_write.get(w).bits.addr := uops_renamed(w).pvl.get
      io.vl_rf_write.get(w).bits.data := io.ren2_vl_imm.get(w)
    }
  }

  // ===========================================================================
  // ---- 6. The in-bundle prefix bypass -- the part this module owns ----
  // ===========================================================================
  //
  //@req-spec-rename.b2
  //@req-spec-rename.b3
  //@req-spec-rename.h10
  // BypassAllocations is the vector override of baseline's method of the
  // same name: it ORs the in-bundle dependence into the READINESS fields
  // only. THE PRN HALF OF THE BYPASS IS VecMapTable'S -- never re-muxed here
  // (trap 1). Agreement between the two is checked by assertion below rather
  // than re-implemented.
  private def loOf(x: UInt): UInt = x(lvregSz - 1, 0)
  private def hiOf(lo: UInt, emul: UInt): UInt = lo +& emul
  // Bitwise OR, not add: RVV requires a group base to be a multiple of EMUL,
  // exactly as VecMapTable's own groupRow relies on.
  private def groupRow(specBase: UInt, m: Int): UInt = loOf(specBase) | m.U(lvregSz.W)

  val lo: Seq[UInt] =
    if (vectorInstance) ren2_uops.map(u => loOf(u.lvd.get))
    else Seq.fill(plWidth)(0.U(lvregSz.W))
  val emulOf: Seq[UInt] =
    if (vectorInstance) ren2_uops.map(_.v_emul.get)
    else Seq.fill(plWidth)(1.U)
  val hi: Seq[UInt] = (lo zip emulOf).map { case (l, e) => hiOf(l, e) }

  // Trap (3): a born-ready vsetivli must not set the dependent's pvl_busy --
  // the READINESS bypass (not the installation predicate remap_valid) is
  // additionally qualified by !ren2_vl_imm_valid(k).
  val readiness_bypass_qual: Seq[Bool] =
    if (hasRenameWrite) (0 until plWidth).map(k => remap_valid(k) && !io.ren2_vl_imm_valid.get(k))
    else remap_valid

  private def hitRow(i: Int, row: UInt): Bool =
    (0 until i).map(k => readiness_bypass_qual(k) && row >= lo(k) && row < hi(k))
      .foldLeft(false.B)(_ || _)

  if (vectorInstance) {
    for (i <- 0 until plWidth) {
      val u    = ren2_uops(i)
      val busy = busytable.io.busy_resps(i)
      val emul = u.v_emul.get

      def bypassHit(specBase: UInt): Bool =
        (0 until maxGroupSize).map(m => m.U < emul && hitRow(i, groupRow(specBase, m)))
          .foldLeft(false.B)(_ || _)

      // Busy responses reach the uop with the qualifications only this
      // module can apply (D11 for pvs1/pvs2/pvs3, v_is_masked for pvm,
      // is_shared for pvtmp), each ORed with the in-bundle bypass hit.
      uops_renamed(i).pvs1_busy.get  := u.v_uses_vs1.get  && (busy.pvs1_busy.get  || bypassHit(u.lvs1.get))
      uops_renamed(i).pvs2_busy.get  := u.v_uses_vs2.get  && (busy.pvs2_busy.get  || bypassHit(u.lvs2.get))
      uops_renamed(i).pvs3_busy.get  := u.v_uses_vs3.get  && (busy.pvs3_busy.get  || bypassHit(u.lvs3.get))
      uops_renamed(i).pvm_busy.get   := u.v_is_masked.get && (busy.pvm_busy.get   || hitRow(i, loOf(u.lvm.get)))
      // pvtmp is not map-table-addressed (no lvd-style specifier), so no
      // bypass term applies -- is_shared HERE, not req_shared (every shared
      // op has a real pvtmp busy lifetime).
      uops_renamed(i).pvtmp_busy.get := u.is_shared.get && busy.pvtmp_busy.get
    }
  } else {
    for (i <- 0 until plWidth) {
      val u    = ren2_uops(i)
      val busy = busytable.io.busy_resps(i)
      //@req-spec-rename.h10 (VL degeneration -- one row wide)
      uops_renamed(i).pvl_busy.get := u.is_vec.get && (busy.pvl_busy.get || hitRow(i, 0.U(lvregSz.W)))
    }
  }

  // ---- member_rdy (only when exportMemberRdy: vec_rename) ----
  if (exportMemberRdy) {
    for (i <- 0 until plWidth) {
      val u   = ren2_uops(i)
      val mb  = busytable.io.member_busy_resps.get(i)
      val mr  = io.member_rdy.get(i)
      for (m <- 0 until maxGroupSize) {
        mr.vs1_rdy(m)  := !u.v_uses_vs1.get  || (!mb.pvs1_busy(m)  && !hitRow(i, groupRow(u.lvs1.get, m)))
        mr.vs2_rdy(m)  := !u.v_uses_vs2.get  || (!mb.pvs2_busy(m)  && !hitRow(i, groupRow(u.lvs2.get, m)))
        mr.vs3_rdy(m)  := !u.v_uses_vs3.get  || (!mb.pvs3_busy(m)  && !hitRow(i, groupRow(u.lvs3.get, m)))
        // vtmp_rdy/vold_rdy carry no information on a lane with no such
        // group -- the consumer's `used` input suppresses them there.
        mr.vtmp_rdy(m) := !mb.pvtmp_busy(m)
        // vold_rdy: stale_pvdest's per-member readiness, range-tested on
        // lvd (trap 4) -- lane i's own destination rows, not an lvs*.
        mr.vold_rdy(m) := !mb.pvold_busy(m) && !hitRow(i, groupRow(u.lvd.get, m))
      }
      mr.vm_rdy := !u.v_is_masked.get || (!mb.pvm_busy && !hitRow(i, loOf(u.lvm.get)))
    }
  }

  // ---- Trap 1 cross-check (simulation-only, feeds no functional signal):
  // the readiness bypass above must never disagree with VecMapTable's own
  // PRN bypass for the same row. A second bypass here could only mask a bug
  // in the first, so this asserts agreement instead of re-computing it. ----
  private def assertRowAgreement(i: Int, respMembers: Vec[UInt], specBase: UInt): Unit = {
    for (m <- 0 until maxGroupSize) {
      val row = groupRow(specBase, m)
      for (k <- 0 until i) {
        when (readiness_bypass_qual(k) && row >= lo(k) && row < hi(k)) {
          assert(respMembers(m) === freelist.io.alloc_pvdest(k)(row - lo(k)),
            "VecRenameSpace: readiness bypass disagrees with VecMapTable's PRN bypass")
        }
      }
    }
  }
  if (vectorInstance) {
    for (i <- 0 until plWidth) {
      val resp = maptable.io.map_resps(i)
      assertRowAgreement(i, resp.pvs1, ren2_uops(i).lvs1.get)
      assertRowAgreement(i, resp.pvs2, ren2_uops(i).lvs2.get)
      assertRowAgreement(i, resp.pvs3, ren2_uops(i).lvs3.get)
      assertRowAgreement(i, resp.stale_pvdest, ren2_uops(i).lvd.get)
    }
  } else {
    for (i <- 0 until plWidth) {
      val row = 0.U(lvregSz.W)
      for (k <- 0 until i) {
        when (readiness_bypass_qual(k) && row >= lo(k) && row < hi(k)) {
          assert(maptable.io.map_resps(i).stale_pvdest(0) === freelist.io.alloc_pvdest(k)(0),
            "VecRenameSpace: VL readiness bypass disagrees with VecMapTable's PRN bypass")
        }
      }
    }
  }

  // ===========================================================================
  // ---- 7. Branch tags, snapshots and recovery ----
  // ===========================================================================
  //
  //@req-spec-rename.i2
  //@req-spec-rename.i3
  //@req-spec-rename.i4
  //@req-spec-decode.i8
  // ren2_br_tags is COMPUTED HERE, not received -- character for character
  // baseline's ren2_br_tags. Because both derive from the SAME registered
  // ren2_uops and the SAME dis_fire, the vector RMT and the VL map table are
  // snapshotted on the same event as the scalar RMT by construction.
  val ren2_br_tags = Wire(Vec(plWidth + 1, Valid(UInt(brTagSz.W))))
  ren2_br_tags(0).valid := false.B
  ren2_br_tags(0).bits  := DontCare
  for (w <- 0 until plWidth) {
    ren2_br_tags(w + 1).valid := dis_fire(w) && ren2_uops(w).allocate_brtag
    ren2_br_tags(w + 1).bits  := ren2_uops(w).br_tag
  }
  maptable.io.ren_br_tags := ren2_br_tags
  freelist.io.ren_br_tags := ren2_br_tags

  //@req-spec-rename.i12
  //@req-spec-rename.i13
  //@req-spec-decode.i9
  // Recovery is entirely the children's, driven from two forwarded signals
  // and nothing else: mispredict restores a branch snapshot in one cycle,
  // rollback copies the committed table in one cycle. This module adds no
  // third recovery arm, no ROB walk-back and no periodic snapshot.
  maptable.io.brupdate := brupdate
  maptable.io.rollback := rollback
  freelist.io.brupdate := brupdate
  freelist.io.rollback := rollback

  // ===========================================================================
  // ---- 8. Commit wiring, and the two free paths ----
  // ===========================================================================
  //
  //@req-spec-decode.c24
  // Per commit lane w, com_valids(w) is io.com_valids(w) AND this space's
  // commit predicate: dst_rtype === RT_VEC for vec_rename, is_vl_producer for
  // vl_rename.
  val com_valids = Wire(Vec(retireWidth, Bool()))
  for (w <- 0 until retireWidth) {
    com_valids(w) := io.com_valids(w) &&
      (if (vectorInstance) io.com_uops(w).dst_rtype === RT_VEC else io.com_uops(w).is_vl_producer.get)
  }

  for (w <- 0 until plWidth) {
    if (vectorInstance) {
      com_remap_reqs(w).lvd    := io.com_uops(w).lvd.get
      com_remap_reqs(w).emul   := io.com_uops(w).v_emul.get
      com_remap_reqs(w).pvdest := io.com_uops(w).pvdest.get
    } else {
      com_remap_reqs(w).lvd       := 0.U
      com_remap_reqs(w).emul      := 1.U
      com_remap_reqs(w).pvdest(0) := io.com_uops(w).pvl.get
    }
    com_remap_reqs(w).valid := com_valids(w)
  }
  maptable.io.com_remap_reqs := com_remap_reqs

  if (freeDiscipline == "stale_group") {
    for (w <- 0 until retireWidth) {
      for (j <- 0 until maxGroupSize) {
        val idx = w * maxGroupSize + j
        freelist.io.dealloc(idx).valid := com_valids(w) && j.U < io.com_uops(w).v_emul.get
        freelist.io.dealloc(idx).bits  := io.com_uops(w).stale_pvdest.get(j)
      }
    }
    // dealloc_tmp: separate gate io.com_valids(w) && is_vec && is_shared --
    // NOT com_valids(w) (this space's dst_rtype-qualified predicate) -- a
    // segmented STORE has a pvtmp group to free and no vector destination at
    // all, so it would never satisfy dst_rtype === RT_VEC.
    freelist.io.dealloc_tmp.foreach { dt =>
      for (w <- 0 until retireWidth) {
        for (j <- 0 until maxGroupSize) {
          val idx = w * maxGroupSize + j
          dt(idx).valid := io.com_valids(w) && io.com_uops(w).is_vec.get && io.com_uops(w).is_shared.get &&
            j.U < io.com_uops(w).v_emul.get
          dt(idx).bits  := io.com_uops(w).pvtmp.get(j)
        }
      }
    }
  } else {
    // committed_ptr: slot w*maxGroupSize takes maptable.io.com_stale_resps(w)(0)
    // -- the pointer the commit install DISPLACES, read from the committed
    // table before the update. No stale_pvl field exists and none is needed.
    for (w <- 0 until retireWidth) {
      for (j <- 0 until maxGroupSize) {
        val idx = w * maxGroupSize + j
        freelist.io.dealloc(idx).valid := (if (j == 0) com_valids(w) else false.B)
        freelist.io.dealloc(idx).bits  := maptable.io.com_stale_resps.get(w)(0)
      }
    }
  }

  // ===========================================================================
  // ---- The lockstep outputs ----
  // ===========================================================================
  //
  //@req-spec-rename.a7
  //@req-spec-rename.b1
  //@req-spec-rename.a11
  //@req-spec-rename.h9
  // The whole dispatch group leaves rename in this one cycle: combinational
  // from ren2_uops, no bubble between the scalar and vector halves. The two
  // instances are CHAINED by VecPipeline (vec_rename's ren2_uops_out is
  // vl_rename's ren2_uops); this output is a wire pass-through of fields the
  // other instance never reads, adding no logic depth. br_mask is NOT
  // re-derived and GetNewUopAndBrMask is NOT re-applied -- the scalar
  // RenameStage already did it and owns that field.
  io.ren2_uops_out := uops_renamed

  //@req-spec-rename.e13
  //@req-spec-rename.e16
  //@req-spec-rename.h26
  // alloc_ok is the free list's whole-bundle verdict, UNMODIFIED: an OP.v
  // that cannot allocate its vector group, or a VL producer that finds no
  // free VL PRN, simply STALLS AT RENAME, and the failure stalls the WHOLE
  // dispatch group -- deadlock-free because rename is in program order.
  io.alloc_ok := freelist.io.alloc_ok

  io.debug_freelist  := freelist.io.debug_freelist
  io.debug_busytable := busytable.io.debug.busytable

  // ===========================================================================
  // ---- 9. What this module deliberately does NOT do ----
  // ===========================================================================
  //
  // No ROB-safety field is written here -- see the file-header SPEC DEFECT
  // note. No child_rebusys port and no speculative rebusy. No despec port and
  // no isImm mode. No LDQ/STQ or ROB index assignment.

  // ===========================================================================
  // ---- Assertions (simulation-only; feed no functional signal) ----
  // ===========================================================================
  //
  // Carried over from baseline's leak assertion, per space.
  assert(!RegNext(rollback) || PopCount(freelist.io.debug_freelist) === (numPhysRegs - numArchRegs).U,
    "VecRenameSpace: leaking physical registers")

  // ADOPTED (VecFreeList could not implement this -- it has no per-commit-
  // lane member count port; this module does, via io.com_uops(w).v_emul).
  // No dealloc/dealloc_tmp slot beyond a committing group's member count may
  // be valid.
  for (w <- 0 until retireWidth) {
    for (j <- 0 until maxGroupSize) {
      val idx = w * maxGroupSize + j
      assert(!freelist.io.dealloc(idx).valid || j.U < io.com_uops(w).v_emul.get,
        "VecRenameSpace: dealloc slot beyond committing group's member count is valid")
    }
    freelist.io.dealloc_tmp.foreach { dt =>
      for (j <- 0 until maxGroupSize) {
        val idx = w * maxGroupSize + j
        assert(!dt(idx).valid || j.U < io.com_uops(w).v_emul.get,
          "VecRenameSpace: dealloc_tmp slot beyond committing group's member count is valid")
      }
    }
  }

  // ADOPTED (VecFreeList could not implement this -- it has no dis_fire
  // port; this module does). alloc_fire(w) is NOT a valid substitute for
  // dis_fire(w) here: alloc_fire(w) additionally requires reqs(w), so a
  // non-vector dispatching lane would falsely trip a substituted assertion.
  for (w <- 0 until plWidth) {
    assert(!ren2_br_tags(w + 1).valid || dis_fire(w),
      "VecRenameSpace: ren_br_tags(w+1).valid without dis_fire(w)")
  }

  // ===========================================================================
  // ---- 10. Trace ----
  // ===========================================================================
  //
  // Guarded VecTrace lines, gated internally by VecTrace itself on the
  // vecTrace plusarg and !reset, off by default. Every call below uses a real
  // MicroOp (ren2_uops/uops_renamed) and its real rob_idx -- never a
  // fabricated identifier.
  if (vectorInstance) {
    //@req-spec-rename.b2 (trace half -- "ren" per renamed lane)
    for (w <- 0 until plWidth) {
      when (ren2_alloc_fire(w)) {
        val extra = Seq(("pvtmp", uops_renamed(w).pvtmp.get.head))
        VecTrace.tracePrn("VecRenameSpace", "ren", uops_renamed(w), extra)
      }
    }
  } else {
    // SPEC DEFECT (reported, not resolved) -- "traceVl per VL rename (event
    // 'vl'): pvl, the written value, born-ready or not" cannot be
    // implemented via VecTrace.traceVl AS WRITTEN for the common
    // register-sourced case: traceVl's `vl` argument must be THE numeric VL
    // value, but for a register-sourced vset that value is not known at
    // rename (it is computed at execute); only vsetivli's immediate is known
    // here. Fabricating a value for the register-sourced case would silently
    // alias with a real resolved VL in every grep -- exactly the failure mode
    // VecTrace's own doc comments warn against. Resolved by using the
    // generic `trace()` primitive instead, with explicit, real fields
    // (`pvl`, `born_ready`, and `vl_imm` -- the last meaningful only when
    // `born_ready`) rather than the canonical single `vl` value, so a reader
    // is never misled into treating an unknown register-sourced value as
    // known.
    for (w <- 0 until plWidth) {
      when (ren2_alloc_fire(w)) {
        VecTrace.trace("VecRenameSpace", "vl", uops_renamed(w), Seq(
          ("pvl",        uops_renamed(w).pvl.get),
          ("born_ready", io.ren2_vl_imm_valid.get(w)),
          ("vl_imm",     io.ren2_vl_imm.get(w))))
      }
    }
  }

  // One line when alloc_ok is low (event "stall"), tagged with the OLDEST
  // requesting lane's real uop -- !alloc_ok alone already implies some lane
  // requested (VecFreeList's own note), so PriorityEncoder always finds one.
  when (!io.alloc_ok) {
    val stall_lane = PriorityEncoder(ren2_alloc_reqs)
    VecTrace.trace("VecRenameSpace", "stall", ren2_uops(stall_lane))
  }

  // One per recovery event naming the arm that fired and the br_tag. This is
  // already the TOP rung of VecTrace's ladder: `brupdate.b2.uop` is a genuine
  // `MicroOp` (`BrResolutionInfo extends BoomBundle with HasBoomUOP`), so
  // `traceTag` -- which takes a real uop, not merely a bare `rob_idx` -- is
  // the right call here, not `traceId`/`traceStruct`.
  when (brupdate.b2.mispredict) {
    VecTrace.traceTag("VecRenameSpace", "recover_mispredict", brupdate.b2.uop, brupdate.b2.uop.br_tag)
  }
  // RE-CHECKED against VecTrace's new `traceId`/`traceStruct` ladder rungs --
  // still SPEC DEFECT (reported, not resolved): the rollback recovery arm has
  // no implementable trace line, even with the two new entry points.
  // `rollback` is Input(Bool()) only, with no uop and no rob_idx attached
  // anywhere on this module's ports, so `traceId` (needs a real `rob_idx`)
  // does not apply. `traceStruct` was considered next: its `extra` must be a
  // genuinely IDENTIFYING key, not a filler value, and nothing at this port
  // boundary qualifies. `com_valids`/`com_uops` are visible on this module,
  // but they are NOT the rollback event's identity: rob.scala's FSM enters
  // `s_rollback` only on `RegNext(RegNext(exception_thrown))` (rob.scala:828),
  // i.e. two cycles after the excepting instruction's own commit cycle, by
  // which point `rob_head` (and therefore `com_uops`) has already advanced to
  // whatever instruction incidentally sits there that cycle -- unrelated to
  // the flush's cause. Tagging the line with that uop's `rob_idx` would
  // misattribute the rollback to the wrong instruction, which is worse than
  // `rob=?`. A literal marker (e.g. `("rollback", 1.U)`) was also considered
  // and rejected: the event string is already "recover_rollback"-equivalent
  // in context, so a constant field identifies nothing `traceStruct`'s own
  // non-empty-`extra` require is trying to guarantee. Omitted rather than
  // tagged with a fabricated identifier, per the same discipline VecMapTable's
  // own generated file applies to this exact port and this exact event pair
  // (VecMapTable.scala:512-517, its "recover_mispredict"/rollback arms): same
  // bare `Input(Bool())`, same absence of an accompanying uop, same
  // conclusion after the same check. (VecFreeList never had a recovery-event
  // trace line to begin with -- its rollback handling is a pure state update,
  // `rollback_deallocs` -- so there is no equivalent omission to cite there.)
}

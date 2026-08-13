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
// SPEC DEFECT (reported, not resolved) -- PLWIDTH HAS NO WORKABLE DEFAULT AND MUST EQUAL coreWidth.
// See hierarchy.yaml instantiation sites for plWidth handling and VecFreeList's port sizing.
//
// SPEC DEFECT (reported, not resolved) -- `rob_unsafe` IS NOT WRITTEN HERE.
// The fix belongs to Rob delta's `vec_clr_unsafe` path, not this file.

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
  // SPEC DEFECT (reported): VecFreeList sizes ports from bare coreWidth, not plWidth parameter.
  require(plWidth == coreWidth,
    s"VecRenameSpace: plWidth ($plWidth) must equal coreWidth ($coreWidth) -- VecFreeList's " +
    "own ports are sized directly from coreWidth, not from a plWidth parameter")
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
  //@req-spec-rename.a2
  //@req-spec-rename.a3
  //@req-spec-rename.a4
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

  val needs_pvdest: Seq[Bool] = ren2_uops.map(_.dst_rtype === RT_VEC)
  val needs_pvtmp:  Seq[Bool] = ren2_uops.map(u => u.is_vec.get && u.is_shared.get)
  val req_shared:   Seq[Bool] = (needs_pvdest zip needs_pvtmp).map { case (d, t) => d && t }
  val vl_producer:  Seq[Bool] = ren2_uops.map(_.is_vl_producer.get)
  val dest_pred: Seq[Bool] =
    if (vectorInstance) (needs_pvdest zip needs_pvtmp).map { case (d, t) => d || t }
    else vl_producer

  private def renMembers(w: Int): UInt =
    if (vectorInstance) ren2_uops(w).v_emul.get else 1.U
  private def comMembers(w: Int): UInt =
    if (vectorInstance) io.com_uops(w).v_emul.get else 1.U

  val ren2_alloc_reqs: Seq[Bool] = (0 until plWidth).map(w => ren2_mask(w) && dest_pred(w))
  //@req-spec-rename.h5 (repeated -- the two disciplines share this one event)
  val ren2_alloc_fire: Seq[Bool] = (0 until plWidth).map(w => dis_fire(w) && ren2_alloc_reqs(w))

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
    // BUG NOTE: v_emul is 0 in the VL space, not 1 -- use renMembers() helper.
    freelist.io.req_members(w) := renMembers(w)
    //@req-spec-rename.e1
    //@req-spec-rename.e3
    //@req-spec-rename.e5
    freelist.io.req_shared(w) := (if (vectorInstance) req_shared(w) else false.B)
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
      uops_renamed(w).pvs1.get         := resp.pvs1
      uops_renamed(w).pvs2.get         := resp.pvs2
      // SPEC DEFECT (reported, not resolved): pvs3 and stale_pvdest are independent reads at independent specifiers.
      // Already correctly tagged by VecMapTable (VecMapTable.scala:334).
      uops_renamed(w).pvs3.get         := resp.pvs3
      uops_renamed(w).pvm.get          := resp.pvm
      uops_renamed(w).stale_pvdest.get := resp.stale_pvdest
      uops_renamed(w).v_emul.get       := resp.v_emul

      //@req-spec-rename.e1
      //@req-spec-rename.e3
      //@req-spec-rename.e5
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
    for (w <- 0 until plWidth) {
      uops_renamed(w).pvl.get := Mux(
        vl_producer(w) && ren2_alloc_fire(w),
        freelist.io.alloc_pvdest(w)(0),
        maptable.io.map_resps(w).stale_pvdest(0))
      // The displaced mapping is what a producer READS, and a `vle*ff.v` is a
      // producer that reads VL, so the read PRN cannot come off `pvl`.
      uops_renamed(w).pvl_src.get := maptable.io.map_resps(w).stale_pvdest(0)
    }
  }

  val bt_uops = Wire(Vec(plWidth, new MicroOp))
  bt_uops := uops_renamed
  if (!vectorInstance) {
    // WIDTH ASSUMPTION: pvdest(0) is vecPregSz wide, pvl is vlPregSz wide; Chisel := zero-extends/truncates safely.
    for (w <- 0 until plWidth) {
      bt_uops(w).pvdest.get(0) := uops_renamed(w).pvl.get

      // BUG NOTE: v_emul must be forced to 1 here. Without this, the VL busy bit is silently never set.
      // Register-sourced vsetvli/vsetvl writes pvl at ALU writeback, so dependent OP.v must wait on that busy bit.
      bt_uops(w).v_emul.get := 1.U

      bt_uops(w).is_shared.get := false.B
    }
  }
  busytable.io.ren_uops := bt_uops

  //@req-spec-decode.c21
  //@req-spec-decode.i3
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
  private def loOf(x: UInt): UInt = x(lvregSz - 1, 0)
  private def hiOf(lo: UInt, emul: UInt): UInt = lo +& emul
  private def groupRow(specBase: UInt, m: Int): UInt = loOf(specBase) | m.U(lvregSz.W)

  val lo: Seq[UInt] =
    if (vectorInstance) ren2_uops.map(u => loOf(u.lvd.get))
    else Seq.fill(plWidth)(0.U(lvregSz.W))
  val emulOf: Seq[UInt] =
    if (vectorInstance) ren2_uops.map(_.v_emul.get)
    else Seq.fill(plWidth)(1.U)
  val hi: Seq[UInt] = (lo zip emulOf).map { case (l, e) => hiOf(l, e) }

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

      uops_renamed(i).pvs1_busy.get  := u.v_uses_vs1.get  && (busy.pvs1_busy.get  || bypassHit(u.lvs1.get))
      uops_renamed(i).pvs2_busy.get  := u.v_uses_vs2.get  && (busy.pvs2_busy.get  || bypassHit(u.lvs2.get))
      uops_renamed(i).pvs3_busy.get  := u.v_uses_vs3.get  && (busy.pvs3_busy.get  || bypassHit(u.lvs3.get))
      uops_renamed(i).pvm_busy.get   := u.v_is_masked.get && (busy.pvm_busy.get   || hitRow(i, loOf(u.lvm.get)))
      uops_renamed(i).pvtmp_busy.get := u.is_shared.get && busy.pvtmp_busy.get
    }
  } else {
    for (i <- 0 until plWidth) {
      val u    = ren2_uops(i)
      val busy = busytable.io.busy_resps(i)
      //@req-spec-rename.h10
      uops_renamed(i).pvl_busy.get := u.is_vec.get && (busy.pvl_busy.get || hitRow(i, 0.U(lvregSz.W)))
    }
  }

  if (exportMemberRdy) {
    for (i <- 0 until plWidth) {
      val u   = ren2_uops(i)
      val mb  = busytable.io.member_busy_resps.get(i)
      val mr  = io.member_rdy.get(i)
      for (m <- 0 until maxGroupSize) {
        mr.vs1_rdy(m)  := !u.v_uses_vs1.get  || (!mb.pvs1_busy(m)  && !hitRow(i, groupRow(u.lvs1.get, m)))
        mr.vs2_rdy(m)  := !u.v_uses_vs2.get  || (!mb.pvs2_busy(m)  && !hitRow(i, groupRow(u.lvs2.get, m)))
        mr.vs3_rdy(m)  := !u.v_uses_vs3.get  || (!mb.pvs3_busy(m)  && !hitRow(i, groupRow(u.lvs3.get, m)))
        mr.vtmp_rdy(m) := !mb.pvtmp_busy(m)
        mr.vold_rdy(m) := !mb.pvold_busy(m) && !hitRow(i, groupRow(u.lvd.get, m))
      }
      mr.vm_rdy := !u.v_is_masked.get || (!mb.pvm_busy && !hitRow(i, loOf(u.lvm.get)))

      // The issue slot LOADS these on its fill cycle and can only ever OR into
      // them, so a member reported not-ready here that no group_done will ever
      // name is an unrecoverable stall with no signature of its own downstream.
      when (remap_valid(i)) {
        VecTrace.trace("VecRenameSpace", "member_rdy", u, Seq(
          ("lvd",          u.lvd.get),
          ("stale0",       u.stale_pvdest.get(0)),
          ("v_emul",       u.v_emul.get),
          ("pvold_busy0",  mb.pvold_busy(0).asUInt),
          ("hitrow_vold0", hitRow(i, groupRow(u.lvd.get, 0)).asUInt),
          ("vold_rdy0",    mr.vold_rdy(0).asUInt),
          ("vs3_rdy0",     mr.vs3_rdy(0).asUInt),
          ("vm_rdy",       mr.vm_rdy.asUInt)))
      }
    }
  }

  // VecMapTable folds its PRN bypass over `remap_reqs`, youngest older lane
  // last, so only the YOUNGEST hitting lane may be compared against -- and over
  // `remap_valid`, not the readiness qualification, which drops `vsetivli`.
  private def assertRowAgreement(i: Int, respMembers: Vec[UInt], specBase: UInt, what: String): Unit = {
    for (m <- 0 until maxGroupSize) {
      val row  = groupRow(specBase, m)
      val hits = (0 until i).map(k => remap_valid(k) && row >= lo(k) && row < hi(k))
      for (k <- 0 until i) {
        val youngest_hit = hits(k) && (k + 1 until i).map(!hits(_)).foldLeft(true.B)(_ && _)
        when (youngest_hit) {
          assert(respMembers(m) === freelist.io.alloc_pvdest(k)(row - lo(k)),
            s"VecRenameSpace: $what disagrees with VecMapTable's PRN bypass")
        }
      }
    }
  }
  if (vectorInstance) {
    for (i <- 0 until plWidth) {
      val resp = maptable.io.map_resps(i)
      assertRowAgreement(i, resp.pvs1, ren2_uops(i).lvs1.get, "readiness bypass")
      assertRowAgreement(i, resp.pvs2, ren2_uops(i).lvs2.get, "readiness bypass")
      assertRowAgreement(i, resp.pvs3, ren2_uops(i).lvs3.get, "readiness bypass")
      assertRowAgreement(i, resp.stale_pvdest, ren2_uops(i).lvd.get, "readiness bypass")
    }
  } else {
    for (i <- 0 until plWidth) {
      assertRowAgreement(i, maptable.io.map_resps(i).stale_pvdest, 0.U(lvregSz.W),
        "VL readiness bypass")
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
  maptable.io.brupdate := brupdate
  maptable.io.rollback := rollback
  freelist.io.brupdate := brupdate
  freelist.io.rollback := rollback

  // ===========================================================================
  // ---- 8. Commit wiring, and the two free paths ----
  // ===========================================================================
  //
  //@req-spec-decode.c24
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
  io.ren2_uops_out := uops_renamed

  //@req-spec-rename.e13
  //@req-spec-rename.e16
  //@req-spec-rename.h26
  io.alloc_ok := freelist.io.alloc_ok

  io.debug_freelist  := freelist.io.debug_freelist
  io.debug_busytable := busytable.io.debug.busytable

  // ===========================================================================
  // ---- 9. What this module deliberately does NOT do ----
  // ===========================================================================

  // ===========================================================================
  // ---- Assertions (simulation-only; feed no functional signal) ----
  // ===========================================================================
  assert(!RegNext(rollback) || PopCount(freelist.io.debug_freelist) === (numPhysRegs - numArchRegs).U,
    "VecRenameSpace: leaking physical registers")

  // BUG NOTE: use comMembers() helper, not v_emul directly (v_emul is 0 in VL space).
  for (w <- 0 until retireWidth) {
    for (j <- 0 until maxGroupSize) {
      val idx = w * maxGroupSize + j
      assert(!freelist.io.dealloc(idx).valid || j.U < comMembers(w),
        "VecRenameSpace: dealloc slot beyond committing group's member count is valid")
    }
    freelist.io.dealloc_tmp.foreach { dt =>
      for (j <- 0 until maxGroupSize) {
        val idx = w * maxGroupSize + j
        assert(!dt(idx).valid || j.U < comMembers(w),
          "VecRenameSpace: dealloc_tmp slot beyond committing group's member count is valid")
      }
    }
  }

  for (w <- 0 until plWidth) {
    assert(!ren2_br_tags(w + 1).valid || dis_fire(w),
      "VecRenameSpace: ren_br_tags(w+1).valid without dis_fire(w)")
  }

  // ===========================================================================
  // ---- 10. Trace ----
  // ===========================================================================
  if (vectorInstance) {
    //@req-spec-rename.b2
    for (w <- 0 until plWidth) {
      when (ren2_alloc_fire(w)) {
        val extra = Seq(("pvtmp", uops_renamed(w).pvtmp.get.head))
        VecTrace.tracePrn("VecRenameSpace", "ren", uops_renamed(w), extra)
      }
    }
  } else {
    // SPEC DEFECT (reported, not resolved) -- traceVl cannot be implemented for register-sourced vset
    // (value not known at rename). Using generic trace() instead with explicit fields.
    for (w <- 0 until plWidth) {
      when (ren2_alloc_fire(w)) {
        VecTrace.trace("VecRenameSpace", "vl", uops_renamed(w), Seq(
          ("pvl",        uops_renamed(w).pvl.get),
          ("born_ready", io.ren2_vl_imm_valid.get(w)),
          ("vl_imm",     io.ren2_vl_imm.get(w))))
      }
    }
  }

  when (!io.alloc_ok) {
    val stall_lane = PriorityEncoder(ren2_alloc_reqs)
    VecTrace.trace("VecRenameSpace", "stall", ren2_uops(stall_lane))
  }

  when (brupdate.b2.mispredict) {
    VecTrace.traceTag("VecRenameSpace", "recover_mispredict", brupdate.b2.uop, brupdate.b2.uop.br_tag)
  }
  // SPEC DEFECT (reported, not resolved) -- rollback recovery arm has no implementable trace line.
  // rollback is Input(Bool()) only with no accompanying uop/rob_idx; omitted per VecMapTable precedent.
}

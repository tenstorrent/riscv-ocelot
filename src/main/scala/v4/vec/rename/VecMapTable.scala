//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Rename Map Table (Step 3)
//------------------------------------------------------------------------------
//
// Generalizes BOOM's scalar RenameMapTable (src/main/scala/v4/exu/rename/
// rename-maptable.scala) from single-register ops to EMUL-group ops. The vector
// map table stores ONE PRN per architectural vreg (32 entries), so an EMUL-wide
// read of arch base..base+memberCount-1 returns the group's current mappings
// directly and is correct under arbitrary fragmentation (no whole-group gate on
// the read). Each remap writes up to memberCount member PRNs of a dest group.
//
// On top of the PRN table we keep a 32x2b LMUL Tag Table (midcore.rst ~103-155):
// each accepted remap stamps tag_table(ldst+j) := emulTag(v_emul) for j<members.
// A read group is "whole" iff its base is a multiple of memberCount and all
// memberCount tags equal emulTag. This is OBSERVABILITY ONLY: it never gates the
// read; it only drives perf_non_whole_group.
//
// Like VConfigUnit, this is a standalone module -- it elaborates but is NOT wired
// into core.scala here (Step 4 does that).

package boom.v4.vec.rename

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.exu.BrUpdateInfo

class VecMapTable(val plWidth: Int)(implicit p: Parameters) extends BoomModule
{
  val numLregs = 32
  val M = VecEmul.MAX_MEMBERS

  val io = IO(new BoomBundle()(p) {
    // Logical source/dest groups -> physical source/stale-dest groups.
    val map_reqs    = Input (Vec(plWidth, new VecMapReq))
    val map_resps   = Output(Vec(plWidth, new VecMapResp))

    // Remapping an ldst group to freshly-allocated pdst members?
    val remap_reqs     = Input(Vec(plWidth, new VecRemapReq))
    val com_remap_reqs = Input(Vec(plWidth, new VecRemapReq))

    // Dispatching branches: need to take snapshots of table state.
    val ren_br_tags = Input (Vec(plWidth+1, Valid(UInt(brTagSz.W))))

    // Signals for restoring state following misspeculation.
    val brupdate = Input (new BrUpdateInfo)
    val rollback = Input (Bool())

    // Observability: count of valid vector-source-reading slots whose read group
    // was NOT a whole group (i.e. fragmented by an intervening narrower write).
    val perf_non_whole_group = Output(UInt(log2Ceil(plWidth+1).W))

    // Gated trace.
    val vec_trace  = Input (Bool())
    val dec_uop_id = Input (Vec(plWidth, UInt(32.W)))
  })

  // --------------------------------------------------------------------------
  // State -- mirror rename-maptable.scala:71-73, plus the LMUL Tag Table.
  // map_table holds one PRN per arch vreg; initialized arch i -> PRN i.
  // --------------------------------------------------------------------------
  val map_table     = RegInit(VecInit((0 until numLregs) map { i => i.U(vecPregSz.W) }))
  val com_map_table = RegInit(VecInit((0 until numLregs) map { i => i.U(vecPregSz.W) }))
  val br_snapshots  = Reg(Vec(maxBrCount, Vec(numLregs, UInt(vecPregSz.W))))

  // LMUL Tag Table -- branch-snapshotted on the SAME events as map_table.
  val tag_table     = RegInit(VecInit(Seq.fill(numLregs)(0.U(2.W))))
  val com_tag_table = RegInit(VecInit(Seq.fill(numLregs)(0.U(2.W))))
  val tag_br_snapshots = Reg(Vec(maxBrCount, Vec(numLregs, UInt(2.W))))

  // The intermediate states of each table following modification by each slot.
  val remap_table         = Wire(Vec(plWidth+1, Vec(numLregs, UInt(vecPregSz.W))))
  val com_remap_table     = Wire(Vec(plWidth+1, Vec(numLregs, UInt(vecPregSz.W))))
  val tag_remap_table     = Wire(Vec(plWidth+1, Vec(numLregs, UInt(2.W)))) // speculative tag
  val com_tag_remap_table = Wire(Vec(plWidth+1, Vec(numLregs, UInt(2.W)))) // committed tag

  // --------------------------------------------------------------------------
  // Remap fan-out -- generalize rename-maptable.scala:80-98 to member groups.
  //
  // For each slot/source the scalar code built a single per-entry one-hot
  // (UIntToOH(ldst) & valid) carrying one pdst. Here each remap writes member j
  // to arch (ldst+j) for j < memberCount, so per arch index `i` we must, for
  // each slot, select the matching member's pdst (and the slot's emulTag). We
  // precompute per slot: a per-arch-index "written" one-hot, the per-index pdst,
  // and the per-index tag.
  // --------------------------------------------------------------------------
  // Arch vreg index of member j relative to base, wrapped into v0..v31. The
  // architectural register space is 32 deep, so a base+offset that runs past v31
  // wraps -- and the index must stay vecLregSz-wide so it never reads/writes
  // outside the 32-entry table.
  def archIdx(base: UInt, off: Int): UInt = (base + off.U)(vecLregSz - 1, 0)

  def memberWrites(req: VecRemapReq): (Vec[Bool], Vec[UInt], UInt) = {
    val cnt = VecEmul.memberCount(req.v_emul)
    val tag = VecEmul.emulTag(req.v_emul)
    // Per arch index i: written? and which pdst lands there.
    val hit  = Wire(Vec(numLregs, Bool()))
    val pdst = Wire(Vec(numLregs, UInt(vecPregSz.W)))
    for (i <- 0 until numLregs) {
      // member j writes arch (ldst+j); find the (unique) j with ldst+j == i.
      val perMemberHit = (0 until M).map { j =>
        req.valid && (j.U < cnt) && (archIdx(req.ldst, j) === i.U)
      }
      hit(i)  := VecInit(perMemberHit).asUInt.orR
      pdst(i) := Mux1H(perMemberHit, (0 until M).map(j => req.pdst(j)))
    }
    (hit, pdst, tag)
  }

  val spec_writes = io.remap_reqs.map(memberWrites(_))
  val com_writes  = io.com_remap_reqs.map(memberWrites(_))

  val spec_emul_tags = io.remap_reqs.map(req => VecEmul.emulTag(req.v_emul))
  val com_emul_tags  = io.com_remap_reqs.map(req => VecEmul.emulTag(req.v_emul))

  // Figure out the new mappings seen by each pipeline slot (scanLeft per index,
  // exactly like rename-maptable.scala:87-98, one scan for PRNs and one for tags).
  for (i <- 0 until numLregs) {
    val remapped_row = spec_writes.map { case (hit, pdst, _) => (hit(i), pdst(i)) }
      .scanLeft(map_table(i)) { case (prn, (hit, new_prn)) => Mux(hit, new_prn, prn) }
    val com_remapped_row = com_writes.map { case (hit, pdst, _) => (hit(i), pdst(i)) }
      .scanLeft(com_map_table(i)) { case (prn, (hit, new_prn)) => Mux(hit, new_prn, prn) }

    val tag_remapped_row = (spec_writes.map { case (hit, _, _) => hit(i) } zip spec_emul_tags)
      .scanLeft(tag_table(i)) { case (tag, (hit, new_tag)) => Mux(hit, new_tag, tag) }
    val com_tag_remapped_row = (com_writes.map { case (hit, _, _) => hit(i) } zip com_emul_tags)
      .scanLeft(com_tag_table(i)) { case (tag, (hit, new_tag)) => Mux(hit, new_tag, tag) }

    for (j <- 0 until plWidth+1) {
      remap_table(j)(i)         := remapped_row(j)
      com_remap_table(j)(i)     := com_remapped_row(j)
      tag_remap_table(j)(i)     := tag_remapped_row(j)
      com_tag_remap_table(j)(i) := com_tag_remapped_row(j)
    }
  }

  // --------------------------------------------------------------------------
  // Snapshot / restore / commit update -- byte-for-byte rename-maptable.scala
  // :100-126, done for BOTH map_table and tag_table.
  // --------------------------------------------------------------------------
  if (enableSuperscalarSnapshots) {
    for (i <- 0 until plWidth+1) {
      when (io.ren_br_tags(i).valid) {
        br_snapshots(io.ren_br_tags(i).bits)     := remap_table(i)
        tag_br_snapshots(io.ren_br_tags(i).bits) := tag_remap_table(i)
      }
    }
  } else {
    assert(PopCount(io.ren_br_tags.map(_.valid)) <= 1.U)
    val do_br_snapshot    = io.ren_br_tags.map(_.valid).reduce(_||_)
    val br_snapshot_tag   = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_tags.map(_.bits))
    val br_snapshot_table = Mux1H(io.ren_br_tags.map(_.valid), remap_table)
    val tag_snapshot_table = Mux1H(io.ren_br_tags.map(_.valid), tag_remap_table)
    when (do_br_snapshot) {
      br_snapshots(br_snapshot_tag)     := br_snapshot_table
      tag_br_snapshots(br_snapshot_tag) := tag_snapshot_table
    }
  }

  when (io.brupdate.b2.mispredict) {
    // Restore the map / tag tables to a branch snapshot.
    map_table := br_snapshots(io.brupdate.b2.uop.br_tag)
    tag_table := tag_br_snapshots(io.brupdate.b2.uop.br_tag)
  } .elsewhen (io.rollback) {
    map_table := com_map_table
    tag_table := com_tag_table
  } .otherwise {
    // Update mappings.
    map_table := remap_table(plWidth)
    tag_table := tag_remap_table(plWidth)
  }
  com_map_table := com_remap_table(plWidth)
  com_tag_table := com_tag_remap_table(plWidth)

  // --------------------------------------------------------------------------
  // EMUL-wide read + intra-bundle bypass -- generalize rename-maptable.scala
  // :130-137. For each older lane k<i with a valid remap whose dest group covers
  // arch register R (i.e. exists member m<memberCount_k: ldst_k+m == R), forward
  // remap_reqs(k).pdst(m). Bypass is ENABLED (the scalar maptable uses bypass=false
  // for FP/vector; here intra-cycle forwarding is required for correctness).
  // --------------------------------------------------------------------------
  def bypassMember(R: UInt, default: UInt, i: Int): UInt = {
    (0 until i).foldLeft(default) { (prn, k) =>
      val req     = io.remap_reqs(k)
      val cnt_k   = VecEmul.memberCount(req.v_emul)
      val hits    = (0 until M).map { m => req.valid && (m.U < cnt_k) && (archIdx(req.ldst, m) === R) }
      val fwd     = Mux1H(hits, (0 until M).map(m => req.pdst(m)))
      Mux(VecInit(hits).asUInt.orR, fwd, prn)
    }
  }

  for (i <- 0 until plWidth) {
    val req = io.map_reqs(i)

    // Group reads: member j of a source reads arch (base + j), then bypasses.
    def readGroup(base: UInt): Vec[UInt] = {
      val g = Wire(Vec(M, UInt(vecPregSz.W)))
      for (j <- 0 until M) {
        val arch = archIdx(base, j)
        g(j) := bypassMember(arch, map_table(arch), i)
      }
      g
    }

    io.map_resps(i).pvs1         := readGroup(req.lvs1)
    io.map_resps(i).pvs2         := readGroup(req.lvs2)
    io.map_resps(i).pvs3         := readGroup(req.lvs3)
    io.map_resps(i).stale_pvdest := readGroup(req.lvd)
    // Mask source is a single PRN (always v0), read + bypassed like a 1-member group.
    io.map_resps(i).pvm          := bypassMember(req.lvm, map_table(req.lvm), i)
  }

  // --------------------------------------------------------------------------
  // LMUL whole-group checker (OBSERVABILITY ONLY -- never gates the read).
  // midcore.rst:127-144: a source read group in slot i is "whole" iff base lvs
  // is a multiple of memberCount AND all memberCount tags == emulTag(v_emul).
  // LMUL=1 (memberCount=1) is always whole.
  // --------------------------------------------------------------------------
  def isWholeGroup(base: UInt, v_emul: UInt): Bool = {
    val cnt  = VecEmul.memberCount(v_emul)
    val tag  = VecEmul.emulTag(v_emul)
    // base aligned to memberCount: low log2(cnt) bits are zero. cnt is 1/2/4/8,
    // so equivalently (base & (cnt-1)) == 0.
    val aligned = (base & (cnt - 1.U)) === 0.U
    val tags_ok = (0 until M).map { j =>
      // members beyond the group are don't-care.
      !(j.U < cnt) || (tag_table(archIdx(base, j)) === tag)
    }.reduce(_ && _)
    aligned && tags_ok
  }

  // A slot reads a vector source group iff its map req is valid as a vector read.
  // VecMapReq carries no explicit valid bit; the enclosing rename stage (Step 4)
  // only drives requests for vector uops. For the standalone perf counter we gate
  // on the remap req's valid (a vector uop renaming a dest is the read producer);
  // this keeps the counter zero in the default/scalar case.
  val slot_non_whole = Wire(Vec(plWidth, Bool()))
  for (i <- 0 until plWidth) {
    val req     = io.map_reqs(i)
    val active  = io.remap_reqs(i).valid
    // Check the primary source read groups (pvs1/pvs2/pvs3). A group is non-whole
    // if any of its read sources is non-whole.
    val whole1  = isWholeGroup(req.lvs1, req.v_emul)
    val whole2  = isWholeGroup(req.lvs2, req.v_emul)
    val whole3  = isWholeGroup(req.lvs3, req.v_emul)
    slot_non_whole(i) := active && !(whole1 && whole2 && whole3)
  }
  io.perf_non_whole_group := PopCount(slot_non_whole)

  // --------------------------------------------------------------------------
  // Gated trace -- one greppable line per event, prefixed "[vmap]".
  // --------------------------------------------------------------------------
  when (io.vec_trace) {
    for (i <- 0 until plWidth) {
      when (io.remap_reqs(i).valid) {
        val r = io.map_resps(i)
        printf("[vmap] uop_id=%d lvd=%d v_emul=%d pvdest_stale=[%d %d %d %d %d %d %d %d] " +
               "pvs1=[%d %d %d %d %d %d %d %d] pvs2=[%d %d %d %d %d %d %d %d] pvm=%d\n",
          io.dec_uop_id(i), io.map_reqs(i).lvd, io.map_reqs(i).v_emul,
          r.stale_pvdest(0), r.stale_pvdest(1), r.stale_pvdest(2), r.stale_pvdest(3),
          r.stale_pvdest(4), r.stale_pvdest(5), r.stale_pvdest(6), r.stale_pvdest(7),
          r.pvs1(0), r.pvs1(1), r.pvs1(2), r.pvs1(3),
          r.pvs1(4), r.pvs1(5), r.pvs1(6), r.pvs1(7),
          r.pvs2(0), r.pvs2(1), r.pvs2(2), r.pvs2(3),
          r.pvs2(4), r.pvs2(5), r.pvs2(6), r.pvs2(7),
          r.pvm)
      }
      when (slot_non_whole(i)) {
        printf("[vmap-nwg] uop_id=%d non_whole=1\n", io.dec_uop_id(i))
      }
    }
  }
}

//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal VL Rename (Step 3 / midcore.rst _vl-vtype-rename)
//------------------------------------------------------------------------------
//
// VL is renamed by the vector mapper into "its own register space (64 entries),
// separate from the integer/FP/vector PRFs" (midcore.rst:249). VTYPE is NOT
// renamed -- it rides the VCFG mirror (VConfigUnit.scala); only VL gets a
// register file. This module is "a one-architectural-register rename with the
// same structures the scalar rename already provides, just one ARN wide"
// (midcore.rst:250-252): it folds the scalar RMT (rename-maptable.scala),
// free-list (rename-freelist.scala) and busy-table (rename-busytable.scala)
// into a single module whose map state is a *scalar* current-PRN pointer (one
// ARN) instead of a Vec of pointers.
//
// VL is renamed in the SINGLE rename cycle "in parallel with scalar and
// vector-group rename ... so its snapshot is taken on the same ren_br_tags event
// as the other RMTs -- no delayed-br_tag path; a vset->dependent pair in one
// dispatch group uses the in-bundle prefix bypass so the dependent picks up the
// just-renamed pvl" (midcore.rst:256-261). Outputs are combinational
// (single-cycle, no ren2 pipeline register), mirroring VConfigUnit.scala and the
// VecFreeList combinational-output deviation.
//
// Structures (midcore.rst:255-270):
//   - Map table:  a current-PRN pointer (cur_pvl), br-snapshotted per br_tag and
//     restored on mispredict; restored from a committed pointer (com_pvl_r) on
//     rollback. The committed pointer "already holds the PRN this producer
//     displaces" so NO per-uop stale field is needed.
//   - Free list:  64-bit free vector; a producer allocates a fresh PRN.
//   - Busy table: one bit per PRN; set on allocation, cleared by the producer's
//     VL writeback (the VL wakeup network).
//   - Commit:     at commit of a VL producer "the outgoing committed pointer is
//     freed", the committed pointer advances to the new PRN.
//
// PRN 0 is reserved as the reset committed/current pvl (free_list starts ~1.U).
//
// NOTE: the wakeup network (numWbPorts ports) is tied off by core.scala in
// Step 4; VL value/wakeup producers arrive in Steps 8/9/11. This module is sized
// for the final design but expects all-invalid wakeups now.

package boom.v4.vec.rename

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.util._
import boom.v4.exu.BrUpdateInfo

/** A VL writeback event: the completing producer's VL PRN. This is a plain
  * readiness wakeup on the dedicated VL network (midcore.rst:264-265) -- it
  * carries NO value (the value lands in VL_RF; consumers only need the busy bit
  * cleared).
  */
class VlWakeup(implicit p: Parameters) extends BoomBundle
{
  val pvl = UInt(vlPregSz.W)
}

class VlRename(plWidth: Int, numVlPhysRegs: Int, numWbPorts: Int)(implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numVlPhysRegs) // == vlPregSz

  // commitWidth == retireWidth == coreWidth (parameters.scala:144,210). The
  // frozen contract names the commit-port width `commitWidth`; HasBoomCoreParameters
  // exposes it as `coreWidth`, so we alias locally to keep the contract names.
  val commitWidth = coreWidth

  val io = IO(new BoomBundle()(p) {
    val dec_fire      = Input(Vec(plWidth, Bool()))
    val dec_valids    = Input(Vec(plWidth, Bool()))
    val dec_uops      = Input(Vec(plWidth, new MicroOp))
    val ren_br_tags   = Input(Vec(plWidth + 1, Valid(UInt(brTagSz.W))))
    val brupdate      = Input(new BrUpdateInfo)
    val rollback      = Input(Bool())
    val kill          = Input(Bool())
    val wakeups       = Input(Vec(numWbPorts, Valid(new VlWakeup)))
    val com_valids    = Input(Vec(commitWidth, Bool()))
    val com_is_vlprod = Input(Vec(commitWidth, Bool()))
    val com_pvl       = Input(Vec(commitWidth, UInt(vlPregSz.W)))
    val ren2_uops     = Output(Vec(plWidth, new MicroOp))
    val ren_stalls    = Output(Vec(plWidth, Bool()))
    val vec_trace     = Input(Bool())
    val dec_uop_id    = Input(Vec(plWidth, UInt(32.W)))
  })

  val n = numVlPhysRegs

  // io.kill: in the scalar rename it squashes the ren2 pipeline register's valid
  // bit (rename-stage.scala:99). VlRename is combinational (no ren2 register), so
  // there is no register here to squash -- state recovery uses brupdate/rollback.
  // Kept in the IO for interface symmetry with the scalar rename; intentionally
  // not referenced in this module (an unconnected Input is legal in Chisel).

  // --------------------------------------------------------------------------
  // Producer predicate.
  //
  // "Every producer allocates a VL PRN at rename and writes the VL RF"
  // (midcore.rst:274). The producers are vsetivli/vsetvli/vsetvl and vleff
  // (midcore.rst:277-280). Each fresh producer allocates a new pvl; younger
  // vector consumers read the nearest preceding producer's pvl.
  // --------------------------------------------------------------------------
  // Split into a fire-INDEPENDENT REQUEST and a fire-gated FIRE predicate, to
  // break the rename-stall combinational loop (ren_stalls -> dec_fire ->
  // alloc -> ren_stalls). is_vl_producer_req drives the free-list prefix-count
  // + SelectFirstN availability (and hence ren_stalls); is_vl_producer_fire
  // drives every STATE update (free_list consume, busy set, cur_pvl advance, and
  // the intra-bundle pvl prefix bypass).
  def isVlProducer(w: Int): Bool =
    io.dec_uops(w).is_vsetivli ||
    io.dec_uops(w).is_vsetvli  ||
    io.dec_uops(w).is_vsetvl   ||
    (io.dec_uops(w).is_vec && io.dec_uops(w).is_vleff)

  val is_vl_producer_req  = Wire(Vec(plWidth, Bool()))  // fire-INDEPENDENT (drives ren_stalls)
  val is_vl_producer_fire = Wire(Vec(plWidth, Bool()))  // fire-gated (drives state)
  for (w <- 0 until plWidth) {
    is_vl_producer_req(w)  := io.dec_valids(w) && isVlProducer(w)
    is_vl_producer_fire(w) := io.dec_fire(w)   && isVlProducer(w)
  }

  // ==========================================================================
  // VL Free List (64, single-PRN, up to plWidth allocations/cycle)
  //
  // Mirrors RenameFreeList (rename-freelist.scala:62-119) / VecFreeList: a 64-bit
  // free vector, SelectFirstN selector, br_alloc_lists reclaim on mispredict,
  // spec_alloc_list reclaim on rollback. Differences:
  //   - PRN 0 is reserved (reset cur/committed pvl) so free_list resets to ~1.U.
  //   - Outputs are combinational (single-cycle mapper), so there is no
  //     RegEnable/r_valid alloc pipeline -- alloc_pvl/alloc_valid are wires.
  //   - Commit FREES THE OUTGOING COMMITTED POINTER (no stale_pvl): the single
  //     committed pointer holds the displaced PRN (midcore.rst:266-269).
  // ==========================================================================
  val free_list       = RegInit(UInt(n.W), ~(1.U(n.W))) // PRN 0 reserved as reset committed/current pvl
  val spec_alloc_list = RegInit(0.U(n.W))
  val br_alloc_lists  = Reg(Vec(maxBrCount, UInt(n.W)))

  // Carve up to plWidth single-PRN selects out of the free list.
  val sels = SelectFirstN(free_list, plWidth)

  // Lane w consumes the (number-of-producers strictly-older-than-w)-th select.
  // This is an in-bundle program-order prefix count of producers (same idea as
  // VecFreeList's prefix-sum of EMUL demands, here demand == 1 PRN per producer).
  // AVAILABILITY (drives ren_stalls): fire-INDEPENDENT. sel_idx counts REQUESTING
  // producers older than w, so alloc_pvl/alloc_valid depend only on dec_valids +
  // the free-list register, never on dec_fire.
  val alloc_pvl   = Wire(Vec(plWidth, UInt(pregSz.W)))
  val alloc_valid = Wire(Vec(plWidth, Bool()))
  for (w <- 0 until plWidth) {
    // Number of REQUESTING producers strictly older (lower index) than this lane
    // -- the index into `sels` this lane should consume. PopCount over the static
    // slice keeps the count's width bounded (no scanLeft width growth).
    val sel_idx = if (w == 0) 0.U else PopCount((0 until w).map(is_vl_producer_req(_)))
    // chosen select = sels(sel_idx). UIntToOH(sel_idx, plWidth) is all-zeros when
    // sel_idx == plWidth (no select left), so chosen is 0 and the lane can't win.
    val chosen = Mux1H(UIntToOH(sel_idx, plWidth), sels)
    alloc_pvl(w)   := OHToUInt(chosen)
    alloc_valid(w) := is_vl_producer_req(w) && chosen.orR
  }

  // STATE CONSUMPTION: fire-gated. A lane removes its PRN from the free list only
  // when it actually fires. Because dispatch fires lanes in program order (fire
  // is a prefix mask), a firing producer's sel_idx (counted over REQUESTS) equals
  // its count over FIRES, so alloc_pvl(w) names the correct PRN to consume.
  // Per-lane one-hot of the PRN this lane allocates. Program-order scanRight
  // accumulation as in rename-freelist.scala:72 gives, for each branch slot, the
  // allocations made by older lanes.
  val allocs = (0 until plWidth).map(w =>
    UIntToOH(alloc_pvl(w))(n - 1, 0) & Fill(n, is_vl_producer_fire(w) && alloc_valid(w)))
  // alloc_masks(i) = OR of allocations by lanes >= i (slots seen by branch i).
  val alloc_masks = allocs.scanRight(0.U(n.W)) { case (a, m) => m | a }
  // alloc_mask = all allocations this bundle (lanes 0..plWidth-1).
  val alloc_mask = alloc_masks(0)

  // Branch reclaim / rollback reclaim (rename-freelist.scala:77-80).
  val br_deallocs       = br_alloc_lists(io.brupdate.b2.uop.br_tag) & Fill(n, io.brupdate.b2.mispredict)
  val rollback_deallocs = spec_alloc_list & Fill(n, io.rollback)

  // ------------------------------------------------------------------------
  // Commit: free the OUTGOING committed pointer per committing producer.
  //
  // midcore.rst:266-269: "at commit of a VL producer the outgoing committed
  // pointer is freed ... the committed pointer is advanced to the new PRN". We
  // walk the commit lanes in program order with a scanLeft seeded by com_pvl_r:
  // the running value BEFORE a committing producer is the PRN that producer
  // displaces (free it), and the running value advances to com_pvl(w). The final
  // running value is the next committed pointer (com_pvl_r).
  // ------------------------------------------------------------------------
  val com_pvl_r = RegInit(0.U(vlPregSz.W))

  // running committed pointer seen at the START of each commit lane (prefix).
  val com_prefix = (0 until commitWidth)
    .map(w => (io.com_valids(w) && io.com_is_vlprod(w), io.com_pvl(w)))
    .scanLeft(com_pvl_r) { case (cur, (upd, newp)) => Mux(upd, newp, cur) }

  // For each committing producer, the displaced PRN is com_prefix(w) (the
  // pointer BEFORE this lane advanced it). OR their one-hots into dealloc_mask.
  val com_deallocs = (0 until commitWidth).map { w =>
    val freeing = io.com_valids(w) && io.com_is_vlprod(w)
    UIntToOH(com_prefix(w))(n - 1, 0) & Fill(n, freeing)
  }.reduce(_ | _)

  val dealloc_mask = com_deallocs | br_deallocs | rollback_deallocs

  // Advance the committed pointer to the end-of-commit-bundle running value.
  com_pvl_r := com_prefix(commitWidth)

  // Update branch alloc lists (rename-freelist.scala:85-113). Not isImm: the VL
  // busy table is cleared by writeback wakeups, not by an immediate-read despec
  // path, so we use the non-isImm reclaim form.
  for (i <- 0 until maxBrCount) {
    br_alloc_lists(i) := (br_alloc_lists(i) & ~br_deallocs) | alloc_masks(0)
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
    val br_snapshot_tag = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_tags.map(_.bits))
    val br_snapshot_lst = Mux1H(io.ren_br_tags.map(_.valid), alloc_masks)
    when (do_br_snapshot) {
      br_alloc_lists(br_snapshot_tag) := br_snapshot_lst
    }
  }

  // Speculative alloc list for rollback (rename-freelist.scala:116).
  spec_alloc_list := (spec_alloc_list | alloc_masks(0)) & ~dealloc_mask

  // Update the free list (rename-freelist.scala:119): remove allocations, add
  // commit/branch/rollback deallocations.
  free_list := (free_list & ~alloc_mask) | dealloc_mask

  // A producer that could not win a free PRN stalls (rename stall). Both terms
  // are fire-INDEPENDENT (is_vl_producer_req uses dec_valids; alloc_valid uses
  // the free-list register), so this cone does not feed back into dec_fire.
  for (w <- 0 until plWidth) {
    io.ren_stalls(w) := is_vl_producer_req(w) && !alloc_valid(w)
  }

  // ==========================================================================
  // VL Map Table (1 ARN)
  //
  // Mirrors VConfigUnit's scanLeft prefix-select (VConfigUnit.scala:117-120) and
  // snapshot/restore (VConfigUnit.scala:135-163), except the map is a scalar
  // UInt PRN instead of a Vec of VConfig. cur_pvl is the working pointer;
  // com_pvl_r (above) is the committed pointer; br_snapshots is per-br_tag.
  // ==========================================================================
  val cur_pvl      = RegInit(0.U(vlPregSz.W)) // PRN 0 reserved == reset pvl
  val br_snapshots = Reg(Vec(maxBrCount, UInt(vlPregSz.W)))

  // Intra-bundle prefix bypass: scanLeft over (producer?, alloc_pvl) seeded with
  // cur_pvl. prefix(w) is the pvl the CONSUMER at lane w reads (the nearest
  // PRECEDING in-bundle producer, else cur_pvl); prefix(plWidth) is the next
  // cur_pvl. A producer at lane w writes alloc_pvl(w), which all younger lanes
  // see -- this is the vset->dependent bypass of midcore.rst:259-261.
  // Driven from the FIRE-gated producer + fire-gated alloc: dispatch fires lanes
  // IN PROGRAM ORDER (fire is a prefix mask), so if a consumer lane fires, every
  // older producer lane fired too -- the fire-gated prefix therefore gives a
  // firing consumer the correct bypassed pvl, while a non-firing consumer's read
  // is simply re-evaluated next cycle. Keeping the prefix on FIRE (not REQ) keeps
  // it -- and thus cur_pvl's next state -- out of the ren_stalls cone.
  val prefix = (0 until plWidth)
    .map(w => (is_vl_producer_fire(w), alloc_pvl(w)))
    .scanLeft(cur_pvl) { case (cur, (prod, ap)) => Mux(prod, ap, cur) }

  // Branch snapshots on the same ren_br_tags event as the scalar RMT
  // (rename-maptable.scala:101-115 / VConfigUnit.scala:135-149). The snapshot
  // value is prefix(i): the pvl the branch at slot i sees.
  if (enableSuperscalarSnapshots) {
    for (i <- 0 until plWidth + 1) {
      when (io.ren_br_tags(i).valid) {
        br_snapshots(io.ren_br_tags(i).bits) := prefix(i)
      }
    }
  } else {
    assert(PopCount(io.ren_br_tags.map(_.valid)) <= 1.U)
    val do_br_snapshot  = io.ren_br_tags.map(_.valid).reduce(_ || _)
    val br_snapshot_tag = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_tags.map(_.bits))
    val br_snapshot_pvl = Mux1H(io.ren_br_tags.map(_.valid), prefix)
    when (do_br_snapshot) {
      br_snapshots(br_snapshot_tag) := br_snapshot_pvl
    }
  }

  // Next-state for cur_pvl, same priority as the RMT (rename-maptable.scala:117-126):
  // mispredict restores the br_tag snapshot, rollback restores the committed
  // pointer, otherwise advance to the end-of-bundle prefix value.
  when (io.brupdate.b2.mispredict) {
    cur_pvl := br_snapshots(io.brupdate.b2.uop.br_tag)
  } .elsewhen (io.rollback) {
    cur_pvl := com_pvl_r
  } .otherwise {
    cur_pvl := prefix(plWidth)
  }

  // ==========================================================================
  // VL Busy Table (64)
  //
  // Mirrors RenameBusyTable (rename-busytable.scala:53-66): one bit per PRN, set
  // on allocation, cleared by the producer's VL writeback (the VL wakeup
  // network). pvl is a plain readiness wakeup -- NO value broadcast.
  // ==========================================================================
  val busy_table = RegInit(0.U(n.W))

  // Set on allocation (rebusy): a producer's freshly-allocated PRN is busy.
  // STATE WRITE -> fire-gated.
  val set_mask = (0 until plWidth).map { w =>
    UIntToOH(alloc_pvl(w), n) & Fill(n, is_vl_producer_fire(w) && alloc_valid(w))
  }.reduce(_ | _)

  // Clear on writeback: each valid wakeup clears its PRN's busy bit.
  val clear_mask = io.wakeups.map { wk =>
    UIntToOH(wk.bits.pvl, n) & Fill(n, wk.valid)
  }.reduce(_ | _)

  busy_table := (busy_table & ~clear_mask) | set_mask

  // ==========================================================================
  // ren2 outputs (combinational; no ren2 pipeline register).
  //
  // Each lane carries its consumer-read pvl = prefix(w). pvl_busy is the busy
  // bit of that PRN with same-cycle writeback bypassed (a wakeup clearing this
  // PRN this cycle means it is no longer busy). pvl_busy only matters for vector
  // uops (is_vec). Finally fold the resolved branch mask in via
  // GetNewUopAndBrMask (util.scala:77, the helper rename-stage.scala:344 uses).
  // ==========================================================================
  for (w <- 0 until plWidth) {
    val read_pvl  = prefix(w)
    val read_busy = busy_table(read_pvl) && !clear_mask(read_pvl)

    // Intra-bundle producer->consumer dependency: if an OLDER lane in this same
    // bundle is a VL producer that fired, prefix(w) bypassed to its freshly
    // ALLOCATED pvl -- which is necessarily busy (the producer hasn't written
    // back yet, and busy_table's set_mask is registered so it does not yet show).
    // Without this a consumer in the same bundle as its vset would see pvl_busy=0
    // and issue BEFORE the VL-RF write -> stale/garbage vl. Mirrors prefix's
    // is_vl_producer_fire bypass condition.
    val older_inbundle_vl_prod =
      if (w == 0) false.B else (0 until w).map(is_vl_producer_fire(_)).reduce(_ || _)

    val out = WireInit(io.dec_uops(w))
    // A VL PRODUCER (vsetvl*) carries its newly-ALLOCATED pvl as its dest, so its
    // VL writeback (vset_wb.pvl -> VL-RF write + the VL wakeup) lands on the same
    // PRN that younger CONSUMERS read (prefix(w)). A consumer carries the read pvl.
    // Without this, the producer wrote VL-RF[prefix] while consumers read
    // VL-RF[alloc] -> consumers got a stale/garbage vl.
    out.pvl      := Mux(is_vl_producer_req(w), alloc_pvl(w), read_pvl)
    out.pvl_busy := io.dec_uops(w).is_vec && (read_busy || older_inbundle_vl_prod)
    io.ren2_uops(w) := GetNewUopAndBrMask(out, io.brupdate)
  }

  // ==========================================================================
  // Gated trace (io.vec_trace). One greppable line per event, prefixed "[vlr]".
  // ==========================================================================
  when (io.vec_trace) {
    for (w <- 0 until plWidth) {
      when (is_vl_producer_fire(w)) {
        printf("[vlr] uop_id=%d producer=%d alloc_pvl=%d cur_pvl_prefix=%d\n",
          io.dec_uop_id(w), is_vl_producer_fire(w), alloc_pvl(w), prefix(w))
      }
    }
    for (w <- 0 until commitWidth) {
      when (io.com_valids(w) && io.com_is_vlprod(w)) {
        printf("[vlr-free] freed_pvl=%d\n", com_prefix(w))
      }
    }
  }
}

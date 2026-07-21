//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Rename Stage (Step 4 wrapper)
//------------------------------------------------------------------------------
//
// Wraps the three Step-3 vector mapper modules -- VecMapTable, VecFreeList,
// VecBusyTable -- into a single rename stage for vector (RVV) uops. This mirrors
// the IDIOMS of the scalar boom.v4.exu.RenameStage (exu/rename/rename-stage.scala
// :138-347) but is NOT a subclass of AbstractRenameStage: the Step-3 modules are
// combinational, so this stage is single-cycle / combinational too (no ren1->ren2
// pipeline register; dec_* feed the mapper and ren2_uops fall out the same cycle).
//
// Per-uop alloc gating, the freelist->maptable combinational feedback, and the
// final GetNewUopAndBrMask rewrite all follow the scalar RenameStage. The
// freelist.alloc_groups -> maptable.remap_reqs.pdst path is combinational and
// intended (mirrors rename-stage.scala:254-256); VecMapTable reads its REGISTER
// map_table (not the post-write value), so there is no combinational loop.

package boom.v4.vec.rename

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.util.GetNewUopAndBrMask
import boom.v4.exu.BrUpdateInfo

class VecRenameStage(plWidth: Int, numVecPhysRegs: Int, commitWidth: Int, numWbPorts: Int)(implicit p: Parameters)
  extends BoomModule
{
  val io = IO(new BoomBundle()(p) {
    val dec_fire    = Input(Vec(plWidth, Bool()))
    val dec_valids  = Input(Vec(plWidth, Bool()))
    val dec_uops    = Input(Vec(plWidth, new MicroOp))
    val ren_br_tags = Input(Vec(plWidth+1, Valid(UInt(brTagSz.W))))
    val brupdate    = Input(new BrUpdateInfo)
    val rollback    = Input(Bool())
    val kill        = Input(Bool())
    val com_valids  = Input(Vec(commitWidth, Bool()))
    val com_remap   = Input(Vec(commitWidth, new VecRemapReq))
    val com_dealloc = Input(Vec(commitWidth, new VecGroupDealloc))
    val wakeups     = Input(Vec(numWbPorts, Valid(new VecGroupDone)))
    val dis_fire    = Input(Vec(plWidth, Bool()))
    val dis_ready   = Input(Bool())
    val ren2_uops   = Output(Vec(plWidth, new MicroOp))
    val ren_stalls  = Output(Vec(plWidth, Bool()))
    val vec_trace   = Input(Bool())
    val dec_uop_id  = Input(Vec(plWidth, UInt(32.W)))
  })

  //-------------------------------------------------------------
  // Rename Structures (the Step-3 modules).

  val maptable = Module(new VecMapTable(plWidth))
  val freelist = Module(new VecFreeList(plWidth, commitWidth, numVecPhysRegs))
  val busytable = Module(new VecBusyTable(plWidth, numVecPhysRegs, numWbPorts))

  //-------------------------------------------------------------
  // Per-uop allocation gate -- a vector uop renaming a vector dest.
  // Split into a fire-INDEPENDENT REQUEST and a fire-gated STATE-WRITE predicate
  // to break the rename-stall combinational loop (ren_stalls -> dec_fire ->
  // alloc -> ren_stalls). Mirrors the scalar request-vs-fire separation
  // (rename-freelist.scala: reqs/availability is state-driven, sel_fire is
  // dis_fire-gated). needs_alloc drives the free-list REQUEST / AVAILABILITY (and
  // hence ren_stalls); alloc_fire drives the map-table remap and busy-table set.

  val needs_alloc = Wire(Vec(plWidth, Bool()))  // fire-INDEPENDENT (drives ren_stalls)
  val alloc_fire  = Wire(Vec(plWidth, Bool()))  // fire-gated (drives state writes)
  for (w <- 0 until plWidth) {
    val is_vec_dst = io.dec_uops(w).is_vec && (io.dec_uops(w).dst_rtype === RT_VEC)
    needs_alloc(w) := is_vec_dst && io.dec_valids(w)
    alloc_fire(w)  := is_vec_dst && io.dec_fire(w)
  }

  //-------------------------------------------------------------
  // Map Table (rename-stage.scala:240-279).

  for (w <- 0 until plWidth) {
    maptable.io.map_reqs(w).lvs1   := io.dec_uops(w).lvs1
    maptable.io.map_reqs(w).lvs2   := io.dec_uops(w).lvs2
    maptable.io.map_reqs(w).lvs3   := io.dec_uops(w).lvs3
    maptable.io.map_reqs(w).lvd    := io.dec_uops(w).lvd
    maptable.io.map_reqs(w).lvm    := io.dec_uops(w).lvm
    maptable.io.map_reqs(w).v_emul := io.dec_uops(w).v_emul

    // Combinational feedback from the free list (rename-stage.scala:254-256).
    // STATE WRITE: fire-gated (mirrors remap_reqs.valid := ren2_alloc_fire).
    maptable.io.remap_reqs(w).valid  := alloc_fire(w)
    maptable.io.remap_reqs(w).ldst   := io.dec_uops(w).lvd
    maptable.io.remap_reqs(w).pdst   := freelist.io.alloc_groups(w).pdst
    maptable.io.remap_reqs(w).v_emul := io.dec_uops(w).v_emul
  }

  maptable.io.com_remap_reqs := io.com_remap
  maptable.io.ren_br_tags    := io.ren_br_tags
  maptable.io.brupdate       := io.brupdate
  maptable.io.rollback       := io.rollback
  maptable.io.vec_trace      := io.vec_trace
  maptable.io.dec_uop_id     := io.dec_uop_id

  //-------------------------------------------------------------
  // Free List (rename-stage.scala:283-306).

  // Arch vregs v0..v31 are pre-mapped to PRNs 0..31; only PRNs 32+ are free.
  freelist.io.initial_allocation := Cat(~(0.U((numVecPhysRegs-32).W)), 0.U(32.W))

  for (w <- 0 until plWidth) {
    // REQUEST / AVAILABILITY: fire-INDEPENDENT (drives alloc_groups.valid ->
    // ren_stalls). CONSUMPTION: fire-gated via freelist.io.fire.
    freelist.io.reqs(w).valid     := needs_alloc(w)
    freelist.io.reqs(w).v_emul    := io.dec_uops(w).v_emul
    freelist.io.reqs(w).is_shared := io.dec_uops(w).is_shared
    freelist.io.fire(w)           := io.dec_fire(w)
  }

  freelist.io.dealloc     := io.com_dealloc
  freelist.io.ren_br_tags := io.ren_br_tags
  freelist.io.brupdate    := io.brupdate
  freelist.io.rollback    := io.rollback
  freelist.io.vec_trace   := io.vec_trace
  freelist.io.dec_uop_id  := io.dec_uop_id

  //-------------------------------------------------------------
  // Busy Table (rename-stage.scala:308-330).

  for (w <- 0 until plWidth) {
    busytable.io.ren_srcs(w).pvs1       := maptable.io.map_resps(w).pvs1
    busytable.io.ren_srcs(w).pvs2       := maptable.io.map_resps(w).pvs2
    busytable.io.ren_srcs(w).pvs3       := maptable.io.map_resps(w).pvs3
    busytable.io.ren_srcs(w).pvold      := maptable.io.map_resps(w).stale_pvdest
    busytable.io.ren_srcs(w).pvm        := maptable.io.map_resps(w).pvm
    busytable.io.ren_srcs(w).v_emul     := io.dec_uops(w).v_emul
    busytable.io.ren_srcs(w).reads_mask := io.dec_uops(w).is_vec && !io.dec_uops(w).v_unmasked
    // The CII coprocessor reads the OLD dest group (stale_pvdest) as a source for any
    // vector-dest arith op (accumulate, undisturbed tail/mask, reduction/vmv.s merge).
    // Conservatively treat every vector-dest uop as reading it: the producer is always
    // older in program order, so this adds a real (never-deadlocking) source dependency.
    busytable.io.ren_srcs(w).reads_old  := io.dec_uops(w).is_vec && (io.dec_uops(w).dst_rtype === RT_VEC)

    // STATE WRITE (set busy on the freshly-allocated group): fire-gated.
    busytable.io.rebusy_reqs(w).valid     := alloc_fire(w)
    busytable.io.rebusy_reqs(w).pdst      := freelist.io.alloc_groups(w).pdst
    busytable.io.rebusy_reqs(w).mask      := freelist.io.alloc_groups(w).mask
    busytable.io.rebusy_reqs(w).pdst_tmp  := freelist.io.alloc_groups(w).pdst_tmp
    busytable.io.rebusy_reqs(w).mask_tmp  := freelist.io.alloc_groups(w).mask_tmp
    busytable.io.rebusy_reqs(w).is_shared := io.dec_uops(w).is_shared
  }

  busytable.io.wakeups    := io.wakeups
  busytable.io.vec_trace  := io.vec_trace
  busytable.io.dec_uop_id := io.dec_uop_id

  //-------------------------------------------------------------
  // Outputs (rename-stage.scala:332-345).

  for (w <- 0 until plWidth) {
    // Push back against Decode if the EMUL group could not be allocated.
    // fire-INDEPENDENT: needs_alloc uses dec_valids (not dec_fire) and
    // alloc_groups.valid is itself fire-independent (free-list state only), so
    // this cone does not feed back into dec_fire.
    io.ren_stalls(w) := needs_alloc(w) && !freelist.io.alloc_groups(w).valid

    val resp = maptable.io.map_resps(w)
    val alloc = freelist.io.alloc_groups(w)
    val busy = busytable.io.busy_resps(w)

    val ren_uop = WireInit(io.dec_uops(w))
    ren_uop.pvdest          := alloc.pdst(0)
    ren_uop.pvdest_grp      := alloc.pdst
    ren_uop.pvdest_grp_mask := alloc.mask
    ren_uop.stale_pvdest    := resp.stale_pvdest(0)
    ren_uop.stale_pvdest_grp := resp.stale_pvdest
    ren_uop.pvs1            := resp.pvs1(0)
    ren_uop.pvs1_grp        := resp.pvs1
    ren_uop.pvs2            := resp.pvs2(0)
    ren_uop.pvs2_grp        := resp.pvs2
    ren_uop.pvs3            := resp.pvs3(0)
    ren_uop.pvs3_grp        := resp.pvs3
    ren_uop.pvm             := resp.pvm
    ren_uop.pvtmp           := alloc.pdst_tmp
    ren_uop.pvtmp_mask      := alloc.mask_tmp
    ren_uop.pvs1_busy       := busy.pvs1_busy
    ren_uop.pvs2_busy       := busy.pvs2_busy
    ren_uop.pvs3_busy       := busy.pvs3_busy
    ren_uop.pvold_busy      := busy.pvold_busy
    ren_uop.pvm_busy        := busy.pvm_busy
    ren_uop.v_emul          := io.dec_uops(w).v_emul

    io.ren2_uops(w) := GetNewUopAndBrMask(ren_uop, io.brupdate)
  }
}

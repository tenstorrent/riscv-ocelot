//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector LS AGEN -- shared nOP.v bundles (Step 10)
//------------------------------------------------------------------------------
//
// A vector load/store OP.v is a SINGLE uop through rename/ROB/issue; the Vector
// LS AGEN cracks it into element/segment micro-ops (nOP.v). Unit-stride is a
// SINGLE nOP.v (is_unit_stride) expanded just-in-time by the stage-2 Packer at
// the LSU US queues (Step 11); strided/indexed/segmented/masked (SSI) are
// expanded to per-element nOP.v in AGEN stage 1 (Skipper/Walker).
//
// These bundles are the bobtail LoadPacket/StorePacket (reference/vec-lsgen/
// loadgen.scala:17, storegen.scala:17) reworked for Caracal:
//   - the OVI `sb_id` field is DROPPED; consumers use the embedded `uop.rob_idx`,
//   - the FSM still drives the architectural `v_reg` (group member 0..7 relative
//     to vd/vs3); the AGEN's post-FSM remap fills the physical `pdst`/`pdst_member`
//     (resp. `pvs3`/`pvs3_member`) from `uop.pvdest_grp` / `uop.pvs3_grp`, since
//     Caracal renames the EMUL group to NON-contiguous PRNs. Left 0 while dormant.
// Every other field is a field-for-field copy of the reference packet so the FSM
// port stays mechanical. The nOP.v carries the full uop (v_eew/v_emul/v_seg_nf/pvl),
// so the Step-11 US Packer needs no separate ConfigInfo snapshot.

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

/** One cracked load micro-op (element or packed segment).
  * Port of bobtail `LoadPacket` (loadgen.scala:17) minus `sb_id`, plus `pdst`. */
class VecLoadNop(implicit p: Parameters) extends BoomBundle with VecLsConstants
{
  val addr        = UInt(64.W)
  val v_reg       = UInt(5.W)            // architectural dest group member (FSM-driven)
  val el_id       = UInt(EL_ID_W.W)      // element index within the dest PRN
  val el_off      = UInt(6.W)
  val el_count    = UInt(7.W)
  val mask_data   = UInt(64.W)
  val mask_valid  = Bool()
  val is_fake     = Bool()               // not significant (all-masked or el_count == 0)
  val misaligned  = Bool()
  val last        = Bool()
  val dir         = Bool()               // negative-stride return-data padding direction
  val is_fof      = Bool()
  val uop         = new MicroOp()
  // --- Caracal physical resolution (filled by the AGEN remap; 0 while dormant) ---
  val pdst        = UInt(vecPregSz.W)    // resolved member PRN this packet writes
  val pdst_member = UInt(vecSplitSz.W)   // which dest group member (0..7)
  // group property (set by the AGEN remap): the load does NOT fill the whole dest
  // register group (vl<capacity, or vl==0), so the unwritten TAIL lanes must keep
  // their old value (undisturbed) -- VecLSU does the stale_pvdest group-copy.
  val tail_undist = Bool()
  // Track A disambiguation: the load's [grp_lo, grp_hi) BYTE range, registered in the
  // LDQ so an older store's LCAM search can catch a younger speculative vector load it
  // overlaps (-> order_fail replay). PRECISE for unit-stride; a conservative full range
  // [0, ~0) for strided/indexed (over-match: correct, just more replays). grp_rng_v
  // marks it recorded (a real load).
  val grp_lo      = UInt(coreMaxAddrBits.W)
  val grp_hi      = UInt(coreMaxAddrBits.W)
  val grp_rng_v   = Bool()
}

/** One cracked store micro-op (element or packed segment).
  * Port of bobtail `StorePacket` (storegen.scala:17) minus `sb_id`, plus `pvs3`. */
class VecStoreNop(implicit p: Parameters) extends BoomBundle with VecLsConstants
{
  val addr        = UInt(64.W)
  val data        = UInt(DMEM_WIDTH.W)   // ELEN store data, filled by VecDgen
  val mem_size    = UInt(log2Ceil(DMEM_WIDTH / 8).W) // bytes to store
  val is_fake     = Bool()               // must ignore (BOOM has no masked stores yet)
  val misaligned  = Bool()
  val last        = Bool()               // last element in sequence
  val uop         = new MicroOp()
  // --- Caracal physical resolution (filled by the AGEN remap; 0 while dormant) ---
  val pvs3        = UInt(vecPregSz.W)    // resolved member PRN this packet reads (store data src)
  val pvs3_member = UInt(vecSplitSz.W)   // which src group member (0..7)
}

/** Store-data handshake between VecDgen (producer) and the store FSMs (consumer).
  * Field-for-field copy of the bobtail `vdb_data` interface (storegen.scala:44):
  * the FSM requests bytes; VecDgen presents the VRF/INT/FP store data. */
class VecDgenDataIF(implicit p: Parameters) extends BoomBundle with VecLsConstants
{
  val read_bytes  = Output(UInt(VDB_R_SIZE_BYTES.W)) // FSM -> DGEN: bytes requested (ready-like)
  val read_all    = Output(Bool())                   // FSM -> DGEN: consume whole chunk
  val valid_bytes = Input(UInt(VDB_R_SIZE_BYTES.W))  // DGEN -> FSM: bytes available (valid-like)
  val data        = Input(UInt(DMEM_WIDTH.W))        // DGEN -> FSM: the store data (ELEN)
}

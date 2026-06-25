//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Mapper -- shared rename types (Step 3)
//------------------------------------------------------------------------------
//
// One definition, imported by VecMapTable / VecFreeList / VecBusyTable (and, in
// Step 4, VecRenameStage), so the three modules agree on the group/PRN
// representations. A vector instruction renames a whole EMUL group atomically:
// up to 8 member PRNs per architectural vector destination.
//
// Conventions (frozen):
//   - A group is Vec(8, UInt(vecPregSz.W)). member(0) = the base arch vreg's PRN;
//     member(j), 1 <= j < memberCount, = PRN of arch vreg (base + j), each
//     independently mapped (so EMUL-wide reads are correct under arbitrary
//     fragmentation). Members j >= memberCount are don't-care (mask them).
//   - v_emul is the FINAL destination EMUL in the 3-bit vtype-LMUL encoding
//     (the Step-4 core derivation already applied widen/narrow); the mapper
//     treats it as the literal group size and does NOT re-derive it.
//   - The vector mask source pvm is a single PRN (always v0), not a group.

package boom.v4.vec.rename

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

/** EMUL (vtype-LMUL encoding) -> group member count helpers.
  *   v_emul: 0=m1,1=m2,2=m4,3=m8 (group 1/2/4/8); 5=mf8,6=mf4,7=mf2 and the
  *   reserved 4 all map to a single-member group.
  */
object VecEmul
{
  val MAX_MEMBERS = 8

  /** Number of physical members in the group (1/2/4/8). */
  def memberCount(v_emul: UInt): UInt =
    MuxLookup(v_emul, 1.U(4.W))(Seq(
      0.U -> 1.U(4.W),
      1.U -> 2.U(4.W),
      2.U -> 4.U(4.W),
      3.U -> 8.U(4.W)))

  /** Per-member active mask: bit j set iff member j is part of the group. */
  def memberMask(v_emul: UInt): UInt = {
    val c = memberCount(v_emul)
    VecInit((0 until MAX_MEMBERS).map(j => j.U < c)).asUInt
  }

  /** LMUL tag for the 32x2b LMUL Tag Table: m1=00,m2=01,m4=10,m8=11; else 00. */
  def emulTag(v_emul: UInt): UInt =
    MuxLookup(v_emul, 0.U(2.W))(Seq(
      0.U -> 0.U(2.W),
      1.U -> 1.U(2.W),
      2.U -> 2.U(2.W),
      3.U -> 3.U(2.W)))
}

/** A renamed EMUL group: up to 8 member PRNs plus the EMUL it was sized with. */
class VecGroup(implicit p: Parameters) extends BoomBundle
{
  val prn    = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W)) // member(0)=base, member(j)=arch (base+j)
  val v_emul = UInt(3.W)
}

// ----------------------------------------------------------------------------
// VecMapTable ports
// ----------------------------------------------------------------------------

/** Logical source/dest specifiers + EMUL for one uop's map request. */
class VecMapReq(implicit p: Parameters) extends BoomBundle
{
  val lvs1   = UInt(vecLregSz.W)
  val lvs2   = UInt(vecLregSz.W)
  val lvs3   = UInt(vecLregSz.W)
  val lvd    = UInt(vecLregSz.W)
  val lvm    = UInt(vecLregSz.W)
  val v_emul = UInt(3.W)
}

/** EMUL-wide group read result for one uop (consumed by the busy table). */
class VecMapResp(implicit p: Parameters) extends BoomBundle
{
  val pvs1         = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W)) // group read of lvs1..lvs1+emul-1
  val pvs2         = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
  val pvs3         = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
  val pvm          = UInt(vecPregSz.W)                            // mask is single-reg (v0)
  val stale_pvdest = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))  // stale group of lvd (commit free)
}

/** Remap (dest write) request: base arch vreg + freshly-allocated group. */
class VecRemapReq(implicit p: Parameters) extends BoomBundle
{
  val ldst   = UInt(vecLregSz.W)                          // base arch vreg of the dest group
  val pdst   = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
  val v_emul = UInt(3.W)
  val valid  = Bool()
}

// ----------------------------------------------------------------------------
// VecFreeList ports
// ----------------------------------------------------------------------------

/** Per-uop group allocation request. */
class VecAllocReq(implicit p: Parameters) extends BoomBundle
{
  val valid     = Bool()
  val v_emul    = UInt(3.W)
  val is_shared = Bool()   // segmented LS: also needs a pvtmp group
}

/** Per-uop group allocation response (combinational). */
class VecAllocResp(implicit p: Parameters) extends BoomBundle
{
  val pdst     = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
  val mask     = UInt(VecEmul.MAX_MEMBERS.W)                  // valid members (= memberMask)
  val pdst_tmp = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))  // pvtmp group when is_shared
  val mask_tmp = UInt(VecEmul.MAX_MEMBERS.W)
  val valid    = Bool()                                       // whole group (+tmp if shared) allocated
}

/** A stale group freed at commit. */
class VecGroupDealloc(implicit p: Parameters) extends BoomBundle
{
  val prn   = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
  val mask  = UInt(VecEmul.MAX_MEMBERS.W)
  val valid = Bool()
}

// ----------------------------------------------------------------------------
// VecBusyTable ports
// ----------------------------------------------------------------------------

/** Group-done event: the completing group's member PRNs (drives busy clear).
  * Produced by the LCB (loads) now and the CII (arith) in a later milestone. */
class VecGroupDone(implicit p: Parameters) extends BoomBundle
{
  val prn  = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
  val mask = UInt(VecEmul.MAX_MEMBERS.W)
}

/** Per-uop source groups for a busy-table read. */
class VecBusySrc(implicit p: Parameters) extends BoomBundle
{
  val pvs1       = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
  val pvs2       = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
  val pvs3       = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
  val pvm        = UInt(vecPregSz.W)
  val v_emul     = UInt(3.W)
  val reads_mask = Bool()   // pvm participates only when the op is masked
}

/** Per-uop group-ready (one collapsed bit per source). */
class VecBusyResp(implicit p: Parameters) extends BoomBundle
{
  val pvs1_busy = Bool()
  val pvs2_busy = Bool()
  val pvs3_busy = Bool()
  val pvm_busy  = Bool()
}

/** Per-uop set-busy request for the dest (and tmp) group. */
class VecRebusyReq(implicit p: Parameters) extends BoomBundle
{
  val pdst      = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
  val mask      = UInt(VecEmul.MAX_MEMBERS.W)
  val valid     = Bool()
  val pdst_tmp  = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
  val mask_tmp  = UInt(VecEmul.MAX_MEMBERS.W)
  val is_shared = Bool()
}

//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal CII host-adapter bundles (Milestone 2, Track B, Step B0)
//------------------------------------------------------------------------------
//
// Chisel-side mirror of the TT-CII contract that BOOM (the host) drives. The
// numeric widths and the op-id / dst-kind encodings here MUST stay in lockstep
// with `src/main/sv/v4/tt-cii/src/tt_cii_caracal_pkg.svh` -- that package
// parameterizes the SV `tt_cii_interface`, and these constants describe the
// flat wire boundary the `TTCii` BlackBox presents to `VecCiiHost`.
//
// The generic 4-channel interface is payload-agnostic; Caracal folds its three
// contract extensions into the type parameters (see the package): vtype/vl/vxrm
// ride inside the issue payload (no CSR channel exists), and the writeback
// `last` marker + scalar-dest routing ride inside the wb status.

package boom.v4.vec.cii

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

/** Fixed CII sizing + encodings. Must match tt_cii_caracal_pkg. */
object CiiConsts
{
  // Lane counts (what BOOM can source/sink per cycle).
  val NUM_INST_ISSUE  = 1   // single in-order IQ_V_ALU head
  val NUM_SRC_REQ     = 2   // 2 CII VRF read ports (5,6)
  val NUM_SRC_DAT_RSP = 2
  val NUM_DST_WB      = 1   // 1 CII VRF write port (port 1)

  // Group / member sizing.
  val MAX_MEMBERS = 8
  val MEMBER_W    = log2Ceil(MAX_MEMBERS)   // 3

  // Tags: opaque side-table ids, bounded by in-flight issue credits.
  val N_TAGS = 16
  val TAG_W  = log2Ceil(N_TAGS)             // 4

  // Issue-payload sub-field widths.
  val VTYPE_W = 8                           // {vsew[3], vlmul[3], vta, vma}
  val VXRM_W  = 2
  val FRM_W   = 3
  val NUM_SRC_SLOTS = 4                     // reuse-hint bits (VS1,VS2,VS3,VM); ignored M2

  // Writeback status sub-fields.
  val FFLAGS_W    = 5                        // {NV,DZ,OF,UF,NX}
  val DSTKIND_W   = 2
  val WB_STATUS_W = 1 + DSTKIND_W + 1 + FFLAGS_W  // last + dst_kind + vxsat + fflags = 9

  // Per-channel credit-FIFO depths.
  val ISS_CREDITS = 16
  val REQ_CREDITS = 16
  val DAT_CREDITS = 16
  val WB_CREDITS  = 16

  // op-id (abstract source slot) -- matches cii_caracal_srcid_e.
  val SRCID_W    = 3
  val SRC_NONE   = 0
  val SRC_VS1    = 1
  val SRC_VS2    = 2
  val SRC_VS3    = 3   // 3rd source / old dest (RMW-undisturbed)
  val SRC_VM     = 4   // v0 mask
  val SRC_SCALAR = 5   // .vx/.vf scalar captured at issue

  // dst-kind -- matches cii_caracal_dst_kind_e.
  val DST_VEC = 0      // write VRF (pvdest_grp member)
  val DST_INT = 1      // write INT RF (vmv.x.s, vcpop.m, vfirst.m)
  val DST_FP  = 2      // write FP  RF (vfmv.f.s)
}

/** vtype snapshot carried in the issue payload (8 b). */
class CiiVtype extends Bundle
{
  val vsew  = UInt(3.W)
  val vlmul = UInt(3.W)
  val vta   = Bool()
  val vma   = Bool()
}

/** The config the coprocessor needs, delivered inside the issue packet (the SV
  * has no CSR channel). Host fills from uop.vconfig + pvl read + csr.io.vector. */
class CiiIssuePayload(implicit p: Parameters) extends BoomBundle
{
  val insn   = UInt(32.W)
  val vtype  = new CiiVtype
  val vl     = UInt(vecVLSz.W)
  val vstart = UInt(vecVLSz.W)
  val vxrm   = UInt(CiiConsts.VXRM_W.W)
  val frm    = UInt(CiiConsts.FRM_W.W)
}

/** Writeback status (folds `last` + scalar-dest routing into the fp-flags slot). */
class CiiWbStatus extends Bundle
{
  val last     = Bool()
  val dst_kind = UInt(CiiConsts.DSTKIND_W.W)
  val vxsat    = Bool()
  val fflags   = UInt(CiiConsts.FFLAGS_W.W)
}

/** One coprocessor source-operand request (decoded from the flat req bus). */
class CiiSrcReq(implicit p: Parameters) extends BoomBundle
{
  val tag        = UInt(CiiConsts.TAG_W.W)
  val op_id      = UInt(CiiConsts.SRCID_W.W)   // source slot (see CiiConsts.SRC_*)
  val op_offset  = UInt(CiiConsts.MEMBER_W.W)  // member index within the group
}

/** Per-tag side-table record: everything the host needs to (B2) map an operand
  * request to a physical reg + member, and (B3) place the result + complete the
  * ROB entry. Written at issue (B1), read on Src-Request / Writeback. */
class CiiTagEntry(implicit p: Parameters) extends BoomBundle
{
  val rob_idx         = UInt(robAddrSz.W)
  val pvdest_grp      = Vec(CiiConsts.MAX_MEMBERS, UInt(vecPregSz.W))
  val pvdest_grp_mask = UInt(CiiConsts.MAX_MEMBERS.W)
  val pvs1_grp        = Vec(CiiConsts.MAX_MEMBERS, UInt(vecPregSz.W))
  val pvs2_grp        = Vec(CiiConsts.MAX_MEMBERS, UInt(vecPregSz.W))
  val pvs3_grp        = Vec(CiiConsts.MAX_MEMBERS, UInt(vecPregSz.W))
  val pvm             = UInt(vecPregSz.W)
  val scalar          = UInt(xLen.W)  // captured .vx/.vf scalar operand (B2b: real value)
  val dst_rtype       = UInt(3.W)   // RT_VEC vs RT_FIX/RT_FLT (scalar-dest routing, B3)
  val is_shared       = Bool()      // segment-LS second group-done (B5)
}

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

package boom.v4.vec.generated

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile.FPConstants
import freechips.rocketchip.rocket.VConfig
import freechips.rocketchip.util.IntToAugmentedInt

import boom.v4.common.{BoomBundle, HasBoomUOP, MicroOp}
import boom.v4.exu.{BrUpdateInfo, ExeUnitResp, Wakeup}

// GENERATED from src/main/nlhdl/pkg/VecBundles.nlhdl.scala. Do not hand-edit;
// regenerate via the nlhdl gen-rtl flow instead.

// ---- VecGroupDone: the completion event ----
//@req-spec-core.g2
//@req-spec-issue.f6
class VecGroupDone(implicit p: Parameters) extends BoomBundle
{
  // Full member-PRN vector of the completing group, not base+count.
  val pvdest  = Vec(maxVecMembers, UInt(vecPregSz.W))
  // How many of the members above are valid (0..maxVecMembers).
  val members = UInt(log2Ceil(maxVecMembers + 1).W)
  val rob_idx = UInt(robAddrSz.W)
  // Set (with the VL PRN) when the completing producer also wrote VL
  // (the dual-destination vset case) — "is_vl_producer-style".
  val pvl     = Valid(UInt(vlPregSz.W))
}

// ---- VecMemberRdy: the per-member readiness side channel ----
class VecMemberRdy(implicit p: Parameters) extends BoomBundle
{
  val vs1_rdy  = Vec(maxVecMembers, Bool())
  val vs2_rdy  = Vec(maxVecMembers, Bool())
  val vs3_rdy  = Vec(maxVecMembers, Bool())
  val vtmp_rdy = Vec(maxVecMembers, Bool())
  // stale_pvdest's per-member readiness (decision D6): fed to IQ_V_LOAD /
  // IQ_V_ALU's rdy_vold matcher.
  val vold_rdy = Vec(maxVecMembers, Bool())
  val vm_rdy   = Bool()
}

// ---- VecElemAccess: the nOP.v ----
class VecElemAccess(implicit p: Parameters) extends BoomBundle
  with HasBoomUOP
{
  val vaddr   = UInt(coreMaxAddrBits.W)
  val eew     = UInt(2.W) // matches MicroOp.v_eew's encoding width
  // Byte enable across the up-to-eLen-wide element access.
  val byte_en = UInt((vecELen / 8).W)
  val first   = Bool() // first access of the group
  val last    = Bool() // last access of the group
}

// ---- VecSnoopCandidate / VecLcamSearch: the disambiguation seam ----
// Two-sided contracts (VecLsu + the LSU delta on one side, VecCrossLsuSnoop and
// VecStoreForward on the other), so they live here rather than in either module.
//@req-spec-memord.a15
class VecSnoopCandidate(implicit p: Parameters) extends BoomBundle
{
  val is_store       = Bool()
  val is_unit_stride = Bool()
  val paddr          = UInt(corePAddrBits.W)
  val len            = UInt(log2Ceil(maxVecMembers * vecVLen / 8 + 1).W)
  val eew            = UInt(2.W)
  // BYTE-granular over a whole LMUL=8 group. Not vecVLen/8, and not the same
  // quantity as VecRangeEntry.mask, which counts ELEMENTS.
  val active_mask    = UInt((maxVecMembers * vecVLen / 8).W)
  val uop            = new MicroOp
  val queue_idx      = UInt(resvPtrSz.W)
  val ordinal        = UInt(log2Ceil(ssiQueueEntries + 1).W)
  val q_base         = UInt(resvPtrSz.W)
  val members        = UInt(log2Ceil(maxVecMembers + 1).W)
  val us_data_base   = UInt(log2Ceil(usQueueEntries).W)
  val data_filled    = Bool()
}

// The REDUCED tier-2 hit VecCrossLsuSnoop exports and VecStoreForward consumes. Not
// VecSnoopCandidate: this one is stored per snoop-window entry, where a full MicroOp
// would be prohibitive, so it carries the wrapped stq_idx directly instead.
//@req-spec-memord.a15
class VecSnoopHit(implicit p: Parameters) extends BoomBundle
{
  val is_store       = Bool()
  val is_unit_stride = Bool()
  val stq_idx        = UInt((1 + stqAddrSz).W)
  val paddr          = UInt(corePAddrBits.W)
  val len            = UInt(log2Ceil(maxVecMembers * vecVLen / 8 + 1).W)
  val eew            = UInt(2.W)
  val active_mask    = UInt((maxVecMembers * vecVLen / 8).W)
  val queue_idx      = UInt(resvPtrSz.W)
  val ordinal        = UInt(log2Ceil(ssiQueueEntries + 1).W)
  val us_data_base   = UInt(log2Ceil(usQueueEntries).W)
  val members        = UInt(log2Ceil(maxVecMembers + 1).W)
  val data_filled    = Bool()
}

// The LCAM-stage load tap, consumed by BOTH VecCrossLsuSnoop and VecStoreForward.
// One declaration because the LSU delta must drive them from identical values in the
// same cycle; two views of one tap is how the two sides silently disagree.
//@req-spec-memord.a19
class VecLdSearch(implicit p: Parameters) extends BoomBundle
{
  val paddr          = UInt(corePAddrBits.W)
  val byte_mask      = UInt(coreDataBytes.W)
  val uop            = new MicroOp
  val ldq_idx        = UInt((1 + ldqAddrSz).W)
  val next_stq_idx   = UInt((1 + stqAddrSz).W)
  val is_vec         = Bool()
  val is_unit_stride = Bool()
  val range_base     = UInt(corePAddrBits.W)
  val range_len      = UInt(log2Ceil(maxVecMembers * vecVLen / 8 + 1).W)
  // Forwarding-side qualifiers.
  val can_forward    = Bool()
  val kill_forward   = Bool()
  // Ordering-side: the LSU's existing per-STQ-entry "older than this load" mask.
  val stq_age_mask   = UInt(numStqEntries.W)
}

//@req-spec-memord.a19
class VecLcamSearch(implicit p: Parameters) extends BoomBundle
{
  val is_store_search = Bool()
  val is_load_search  = Bool()
  val paddr           = UInt(corePAddrBits.W)
  val byte_mask       = UInt(coreDataBytes.W)
  val is_range        = Bool()
  val range_lo        = UInt((corePAddrBits - 3).W)
  val range_hi        = UInt((corePAddrBits - 3).W)
  val uop             = new MicroOp
}

// ---- VecMemAccess: the drain-to-arbiter beat ----
//@req-spec-lsu.h1
class VecMemAccess(implicit p: Parameters) extends VecElemAccess
{
  val data           = UInt((coreDataBytes * 8).W)
  val uses_tlb       = Bool()
  val uses_dcache    = Bool()
  val uses_lcam      = Bool()
  val lcam_range_len = UInt(log2Ceil(maxVecMembers * vecVLen / 8 + 1).W)
}

// ---- The element queue set ----
//@req-spec-lsu.a14
//@req-spec-lsu.a15
//@req-spec-agen.a5
object VecQueueId {
  val ld_SSI_ADDR_Q :: st_SSI_ADDR_Q :: st_SSI_DATA_Q :: ld_US_ADDR_Q :: st_US_ADDR_Q :: st_US_DATA_Q :: Nil =
    Enum(6)
}

//@req-spec-lsu.a14
//@req-spec-lsu.a15
//@req-spec-agen.a5
class VecRangeEntry(implicit p: Parameters) extends BoomBundle
{
  // Effective base virtual address of the range.
  val base           = UInt(coreMaxAddrBits.W)
  // TOTAL ACTIVE BYTE LENGTH, sized for a whole LMUL=8 group of BYTES:
  // log2Ceil(maxVecMembers * vecVLen / 8 + 1) = 9 bits at the defaults. NEVER
  // sized from vecVLSz: same width, different QUANTITY (bytes vs. elements),
  // coinciding only because the narrowest SEW is one byte.
  val len            = UInt(log2Ceil(maxVecMembers * vecVLen / 8 + 1).W)
  // Element width. Travels even though address arithmetic no longer needs it
  // once `len` is known: the drain side scales a mask bit into a byte enable
  // with it, and the LCB needs it for placement.
  val eew            = UInt(2.W)
  // EFFECTIVE byte stride (1 << eew for unit-stride, 1 for whole-register/
  // mask forms). Carried explicitly (spec-agen.b7) even though derivable from
  // eew, so the drain side decodes ONE self-describing entry format instead
  // of special-casing three access classes. Width covers 1..(eLen/8) bytes.
  val stride         = UInt(log2Ceil(vecELen / 8 + 1).W)
  // Set on every entry in these queues. Tells VecBeatExpander to coalesce
  // rather than issue one access per entry, and tells VecCrossLsuSnoop /
  // VecStoreForward that this entry is a RANGE (spec-agen.b7).
  val is_unit_stride = Bool()
  // FAULT-ONLY-FIRST. vle<eew>ff.v self-selects into VecRangeAgen and never
  // reaches the element agen, but the fault is raised on the DRAIN side
  // against this retained entry; without this bit the drain side cannot tell
  // a fault-only-first load from an ordinary one. Assert is_unit_stride on
  // every entry carrying it.
  val is_ff          = Bool()
  val nf             = UInt(3.W) // matches MicroOp.v_seg_nf's encoding width
  // ELEMENT-granular, vecVLen bits: one bit per element, matching VecMaskStream's
  // us_mask. At (vecVLen/8) it silently dropped every mask bit above element 31.
  val mask           = UInt(vecVLen.W)
  // Destination group's base PRN and member count, from pvdest/v_emul.
  // FULL per-member PRN vector, never base+count: a renamed group's members are NOT
  // contiguous (VecFreeList hands out a contiguous window of PORTS, not of PRNs).
  val pvdest         = Vec(maxVecMembers, UInt(vecPregSz.W))
  val members        = UInt(log2Ceil(maxVecMembers + 1).W)
  val us_data_base   = UInt(log2Ceil(usQueueEntries).W)
  val rob_idx        = UInt(robAddrSz.W)
  val ldq_idx        = UInt((1 + ldqAddrSz).W)
  val stq_idx        = UInt((1 + stqAddrSz).W)
}

// ---- VecReservation ----
class VecReservation(implicit p: Parameters) extends BoomBundle
{
  // One of the six VecQueueId values. Width derived from the enumeration
  // itself (not a literal) so a widening of the queue set can't silently
  // desync this field from VecQueueId.
  val queue   = UInt(VecQueueId.ld_SSI_ADDR_Q.getWidth.W)
  // Base index of the reserved region. Sized against the deeper (SSI) queue
  // class so one field covers a reservation in either class.
  val base    = UInt(resvPtrSz.W)
  val entries = UInt(log2Ceil(ssiQueueEntries + 1).W)
  val rob_idx = UInt(robAddrSz.W)
  val ldq_idx = UInt((1 + ldqAddrSz).W)
  val stq_idx = UInt((1 + stqAddrSz).W)
}

// ---- VecException ----
// `uop` is load-bearing, not convenience: the ROB latches it and reads uop.br_mask
// for GetNewBrMask, so a rob_idx-only bundle cannot drive rob.io.lxcpt at all.
class VecException(implicit p: Parameters) extends BoomBundle
{
  val uop      = new MicroOp
  val cause    = UInt(log2Ceil(freechips.rocketchip.rocket.Causes.all.max + 2).W)
  val badvaddr = UInt(coreMaxAddrBits.W)
}

// ---- The four CII channel payloads ----
object VecBundlesConsts
{
  //@req-spec-cii.a12
  // Frozen wb_status total width. Stated as 9 by the spec, which also requires
  // wb_status be declared as a NAMED sub-bundle of {last, dst_kind, vxsat,
  // fflags} rather than a bare 9-bit field, so a consumer cannot slice it by
  // hand and get the fields wrong. Unlike the four above this is a total width
  // rather than a mirrored localparam, so it is pinned here as one named
  // constant that a reviewer can diff against the SV package.
  val ciiWbStatusBits: Int = 9
}

//@req-spec-cii.a6
//@req-spec-cii.a14
class CiiIssueReq(implicit p: Parameters) extends BoomBundle
{
  val tag   = UInt(ciiTagBits.W)
  val instr = UInt(32.W) // raw RVV instruction word
  // {vsew(3), vlmul(3, sign+mag), vta(1), vma(1)} packed into 8 bits. Binds
  // to rocket's VConfig/VType (see file-level dependency note): this is a
  // REPACK, not a slice — building it through VType's deprecated `vlmul`
  // accessor drops the fractional-LMUL sign and turns every mf2/mf4/mf8 op
  // into m1/m2/m4 with no width error. That repack is the PRODUCER's
  // (VecCiiIssue's) obligation; this field only fixes the wire shape.
  val vtype = UInt(8.W)
  val vl    = UInt(vecVLSz.W) // 9 bits at the defaults; see HasVectorParams
  // Width assumption: the nlhdl source states vl's width (9b, cross-
  // referenced from HasVectorParams) but not vstart's. vstart shares vl's
  // element-index domain (both range over 0..VLMAX), so it is sized the same
  // — documented here since the spec is silent on this one field.
  val vstart = UInt(vecVLSz.W)
  val vxrm  = UInt(2.W) // architectural width, matches rocket CSR.io.vector.vxrm
  val frm   = UInt(3.W) // matches MicroOp.fp_rm's encoding width
  //@req-spec-cii.a14 (src_reuse_hint IS FOUR BITS, NOT THREE — see file note above)
  val src_reuse_hint = UInt(ciiNumSrcSlots.W)
}

//@req-spec-cii.a8
class CiiSrcReq(implicit p: Parameters) extends BoomBundle
{
  val tag       = UInt(ciiTagBits.W)
  // Abstract source slot (VS1/VS2/VS3/VM), plus the CII_SRC_NONE encoding for
  // an unused lane: log2Ceil(ciiNumSrcSlots + 1) = 3 bits at the defaults.
  val op_id     = UInt(log2Ceil(ciiNumSrcSlots + 1).W)
  // Group member index.
  val op_offset = UInt(log2Ceil(maxVecMembers).W)
}

//@req-spec-cii.a10
class CiiSrcData(implicit p: Parameters) extends BoomBundle
{
  val data = UInt(vecVLen.W)
}

//@req-spec-cii.a12
//@req-spec-cii.a15
class CiiWbStatus(implicit p: Parameters) extends BoomBundle
{
  val last     = Bool() // final beat of this tag
  // Width assumption: the total (9 bits, cii.a12) and the three other
  // sub-fields' widths are given by the spec; dst_kind's width is not, and is
  // inferred here as whatever remains: ciiWbStatusBits - last(1) - vxsat(1) -
  // fflags(FPConstants.FLAGS_SZ) = 2 bits at the defaults.
  val dst_kind = UInt((VecBundlesConsts.ciiWbStatusBits - 1 - 1 - FPConstants.FLAGS_SZ).W)
  val vxsat    = Bool()
  val fflags   = UInt(FPConstants.FLAGS_SZ.W)
}

class CiiWriteback(implicit p: Parameters) extends BoomBundle
{
  val tag            = UInt(ciiTagBits.W)
  val wb_data        = UInt(vecVLen.W)
  val wb_dst_offset  = UInt(log2Ceil(maxVecMembers).W) // destination member index
  val wb_wr_en       = Bool() // per-beat write enable
  val wb_status      = new CiiWbStatus
}

// ---- The host-seam declarations A2 deferred to D2 ----

// ---- VecRobFlags (this is A34) ----
class VecRobFlags(implicit p: Parameters) extends BoomBundle
{
  val rob_idx = UInt(robAddrSz.W)
  val fflags  = UInt(FPConstants.FLAGS_SZ.W)
  val vxsat   = Bool()
}

// ---- IntWbSnoop ----
class IntWbSnoop(implicit p: Parameters) extends BoomBundle
{
  val addr = UInt(maxPregSz.W)
  val data = UInt(xLen.W)
}

// ---- The CSR seam: there is no CSRVectorIO ----
class VecCsrRead(implicit p: Parameters) extends BoomBundle
{
  val vconfig = new VConfig
  val vstart  = UInt(maxVLMax.log2.W)
  val vxrm    = UInt(2.W)
}

// ---- VecPipelineIO ----
class VecPipelineIO(val numIntWakeupPorts: Int, val numFpWakeupPorts: Int)
  (implicit p: Parameters) extends BoomBundle
{
  // ---- decode feed (vector decode lives inside VecPipeline) ------------
  val dec_insns       = Input(Vec(coreWidth, UInt(32.W)))
  val dec_valids      = Input(Vec(coreWidth, Bool()))
  // FIRE, not validity (see nlhdl source): VConfigUnit's vtype mirror must
  // advance only on a lane that actually leaves decode, or a partially-firing
  // bundle double-absorbs a vset on re-presentation.
  val dec_fire        = Input(Vec(coreWidth, Bool()))
  val dec_uops_in     = Input(Vec(coreWidth, new MicroOp()))
  val dec_uops_out    = Output(Vec(coreWidth, new MicroOp()))
  val dec_vec_illegal = Output(Vec(coreWidth, Bool()))

  // ---- rename / dispatch lockstep ----
  val ren2_uops       = Input(Vec(coreWidth, new MicroOp()))
  val ren2_mask       = Input(Vec(coreWidth, Bool()))
  val dis_fire        = Input(Vec(coreWidth, Bool()))
  val dis_ready       = Output(Bool())
  // Three vector issue queues wired natively elsewhere; this carries only the
  // per-lane, per-queue capacity check. Ordering is
  // {IQ_V_LOAD, IQ_V_STORE, IQ_V_ALU} x coreWidth.
  val dis_vec_valids  = Input(Vec(3, Vec(coreWidth, Bool())))
  // The COMPACTED payload that goes with dis_vec_valids. The CompactingDispatcher moves
  // uops between lanes, so pairing dis_vec_valids(q)(w) with the lane-w renamed uop
  // enqueues the wrong instruction as soon as more than one vector op co-dispatches.
  val dis_vec_uops    = Input(Vec(3, Vec(coreWidth, new MicroOp())))
  val dis_vec_ready   = Output(Vec(3, Vec(coreWidth, Bool())))
  val dis_uops_out    = Output(Vec(coreWidth, new MicroOp()))

  // ---- speculation / recovery -----------------------------------------
  val brupdate        = Input(new BrUpdateInfo)
  val rob_pnr_idx     = Input(UInt(robAddrSz.W))
  val rob_head_idx    = Input(UInt(robAddrSz.W))
  val rob_flush       = Input(Bool())
  val rob_flush_kill  = Input(Bool())
  val rob_empty       = Input(Bool())

  // ---- commit ---------------------------------------------------------
  val commit_valids   = Input(Vec(coreWidth, Bool()))
  val commit_uops     = Input(Vec(coreWidth, new MicroOp()))
  val commit_rollback = Input(Bool())

  // ---- completion back into the ROB (group-done) -----------------------
  val vec_clr_bsy      = Output(Vec(vectorParams.numVecClrPorts, Valid(UInt(robAddrSz.W))))
  val vec_clr_unsafe   = Output(Valid(UInt(robAddrSz.W)))
  val vec_xcpt         = Output(Valid(new VecException))
  val vec_rob_flags    = Output(Vec(vectorParams.numVecClrPorts, Valid(new VecRobFlags)))

  // ---- scalar feeders: vector ops read INT/FP for base/stride/.vx/.vf ---
  val int_rf_read_req  = Vec(5, Decoupled(UInt(ipregSz.W)))
  val int_rf_read_rsp  = Input(Vec(5, UInt(xLen.W)))
  val int_wakeups      = Input(Vec(numIntWakeupPorts, Valid(new Wakeup)))
  val int_child_rebusys = Input(UInt(aluWidth.W))
  val int_squash_grant  = Input(Bool())
  val fp_wakeups       = Input(Vec(numFpWakeupPorts,  Valid(new Wakeup)))
  val int_wb_snoop     = Input(Vec(numIrfWritePorts,  Valid(new IntWbSnoop)))
  // The host LSU tap. VecLsu owns its contents; this file inspects no member of it.
  val lsu_vec          = Flipped(new boom.v4.lsu.VecLsuCoreIO)
  val fp_rf_read_req   = Output(UInt(fpregSz.W))
  val fp_rf_read_rsp   = Input(UInt(xLen.W))

  // ---- scalar-dest writeback (vmv.x.s, vcpop.m, vfirst.m, vfmv.f.s) ----
  val int_wb           = Output(Valid(new ExeUnitResp(xLen)))
  val fp_wb            = Output(Valid(new ExeUnitResp(xLen)))

  // ---- vector CSR state (OWNED by rocket CSRFile, see frontend.rst) -----
  val csr_vector       = Input(new VecCsrRead)
  val csr_frm          = Input(UInt(3.W))
  val csr_vs_dirty     = Output(Bool())

  // ---- vset: the dual-destination rule (is_vl_producer) ----------------
  val vset_resp        = Input(Vec(aluWidth, Valid(new ExeUnitResp(xLen))))
  val vl_wakeup        = Output(Vec(numVlWakeupPorts, Valid(UInt(vlPregSz.W))))
  val commit_vl        = Output(Valid(UInt(vecVLSz.W)))

  // ---- memory: the vector LSU's own D$ port + ordering hooks -----------
  val lsu_fencei_rdy_vec = Output(Bool())

  // ---- debug / trace (ground rule: guarded printf, off by default) -----
  val vec_trace_en   = Input(Bool())
  val debug_vrf_read = Output(Vec(coreWidth, UInt(vecVLen.W)))
}

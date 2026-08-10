//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// MicroOp
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v4.common

import chisel3._
import chisel3.util._

import freechips.rocketchip.util._
import freechips.rocketchip.rocket.VType
import org.chipsalliance.cde.config.Parameters

abstract trait HasBoomUOP extends BoomBundle
{
  val uop = new MicroOp()
}

/**
 * MicroOp for the Debug Harness (whisper-cosim DPI bridge).
 * vLen-sized fields are kept on the bundle so the harness SV interface stays stable;
 * they're driven to zero by core.scala until v4 grows a VPU.
 *
 * Ported verbatim from `Caracal/addvector` (`v4/common/micro-op.scala:30`) so the
 * field order and widths keep matching `vsrc/core_harness_wrapper_N.v`, which is a
 * BlackBox: a reordered or resized field here is a SILENT cosim mismatch, not a
 * compile error. Deliberately a plain `Bundle` with explicit Int parameters rather
 * than a `BoomBundle` reading `p` -- `DebugCommitSignals` constructs it with literal
 * widths (`new DebugCommitSignals(40, N, 64, vlen, 5, 1)` in core.scala) to match
 * the SV side, and pulling values from `p` instead would let the two drift.
 *
 * NOTE: addvector also declares `VConfig` and `VsetWbResp` beside this class. Neither
 * is ported: v2 carries the vtype snapshot as rocket's `VType` (`MicroOp.vconfig` is
 * `Option[VType]`), and the vset writeback path is the `ALUUnit`/`Rob` delta's at D3.
 */
class DebugMicroOp(val coreMaxAddrBits: Int, val xLen: Int, val vLen: Int, val lregSz: Int) extends Bundle
{
  val ldst             = UInt(lregSz.W)
  val dst_rtype        = UInt(3.W)
  val debug_pc         = UInt(coreMaxAddrBits.W)
  val debug_tag        = UInt(64.W)
  val debug_inst       = UInt(32.W)
  val debug_wdata      = UInt(xLen.W)
  val debug_vec_wdata  = UInt((vLen*8).W)
  val debug_vec_wmask  = UInt(8.W)
}

// Per-nOP.v element cursor. A vector LDQ/STQ entry carries this in addition
// to its existing scalar fields simply by virtue of embedding a MicroOp (see
// HasBoomUOP above) -- it is not declared separately on the queue entries.
//@req-spec-lsu.f1
//@req-spec-lsu.f2
//@req-spec-lsu.f3
//@req-spec-lsu.f4
class VecElemCursor(implicit p: Parameters) extends BoomBundle
{
  // Width assumption: the spec does not pin a width for this sub-bundle
  // (unlike every other entry in its interface-delta table). vecVLSz sizes
  // the sibling v_split_idx/v_split_total cursor fields declared below in
  // MicroOp, so the same width is used here for the same index/count role.
  val elem_next  = UInt(vecVLSz.W) // index of the next element to drain
  val elem_done  = UInt(vecVLSz.W) // count of completed elements
  val fault_elem = UInt(vecVLSz.W) // index of the oldest faulting element
}

class MicroOp(implicit p: Parameters) extends BoomBundle
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
  with freechips.rocketchip.rocket.constants.ScalarOpConstants
{
  val inst             = UInt(32.W)
  val debug_inst       = UInt(32.W)
  val is_rvc           = Bool()
  val debug_pc         = UInt(coreMaxAddrBits.W)
  val iq_type          = Vec(IQ_SZ, Bool())       // which issue unit do we use?
  val fu_code          = Vec(FC_SZ, Bool()) // which functional unit do we use?

  val iw_issued              = Bool() // Was this uop issued last cycle? If so, it can vacate this cycle
  val iw_issued_partial_agen = Bool()
  val iw_issued_partial_dgen = Bool()
  val iw_p1_speculative_child = UInt(aluWidth.W)
  val iw_p2_speculative_child = UInt(aluWidth.W)


  // Get the operand off the bypass network, avoid a register read port allocation
  val iw_p1_bypass_hint = Bool()
  val iw_p2_bypass_hint = Bool()
  val iw_p3_bypass_hint = Bool()

  val dis_col_sel      = UInt(coreWidth.W) // If using column-issue ALUs with 1-wide dispatch, which column to issue to?

  val br_mask          = UInt(maxBrCount.W)  // which branches are we being speculated under?
  val br_tag           = UInt(brTagSz.W)


  val br_type          = UInt(4.W)
  val is_sfb           = Bool()                      // is this a sfb or in the shadow of a sfb
  val is_fence         = Bool()
  val is_fencei        = Bool()
  val is_sfence        = Bool()
  val is_amo           = Bool()
  val is_eret          = Bool()
  val is_sys_pc2epc    = Bool()                      // Is a ECall or Breakpoint -- both set EPC to PC.
  val is_rocc          = Bool()
  val is_mov           = Bool()

  // Index into FTQ to figure out our fetch PC.
  val ftq_idx          = UInt(log2Ceil(ftqSz).W)
  // This inst straddles two fetch packets
  val edge_inst        = Bool()
  // Low-order bits of our own PC. Combine with ftq[ftq_idx] to get PC.
  // Aligned to a cache-line size, as that is the greater fetch granularity.
  // TODO: Shouldn't this be aligned to fetch-width size?
  val pc_lob           = UInt(log2Ceil(icBlockBytes).W)

  // Was this a branch that was predicted taken?
  val taken            = Bool()

  val imm_rename       = Bool()
  val imm_sel          = UInt(IS_N.getWidth.W)
  val pimm             = UInt(immPregSz.W)
  val imm_packed       = UInt(LONGEST_IMM_SZ.W) // densely pack the imm in decode

  val op1_sel          = UInt(OP1_X.getWidth.W)
  val op2_sel          = UInt(OP2_X.getWidth.W)

  val fp_ctrl          = new freechips.rocketchip.tile.FPUCtrlSigs

  val rob_idx          = UInt(robAddrSz.W)
  val ldq_idx          = UInt((1+ldqAddrSz).W)
  val stq_idx          = UInt((1+stqAddrSz).W)
  val rxq_idx          = UInt(log2Ceil(numRxqEntries).W)
  val pdst             = UInt(maxPregSz.W)
  val prs1             = UInt(maxPregSz.W)
  val prs2             = UInt(maxPregSz.W)
  val prs3             = UInt(maxPregSz.W)
  val ppred            = UInt(log2Ceil(ftqSz).W)

  val prs1_busy        = Bool()
  val prs2_busy        = Bool()
  val prs3_busy        = Bool()
  val ppred_busy       = Bool()

  val stale_pdst       = UInt(maxPregSz.W)
  val exception        = Bool()
  val exc_cause        = UInt(xLen.W)          // TODO compress this down, xlen is insanity
  val mem_cmd          = UInt(M_SZ.W)          // sync primitives/cache flushes
  val mem_size         = UInt(2.W)
  val mem_signed       = Bool()
  val uses_ldq         = Bool()
  val uses_stq         = Bool()
  val is_unique        = Bool()                      // only allow this instruction in the pipeline, wait for STQ to
                                                     // drain, clear fetcha fter it (tell ROB to un-ready until empty)
  val flush_on_commit  = Bool()                      // some instructions need to flush the pipeline behind them
  val csr_cmd          = UInt(freechips.rocketchip.rocket.CSR.SZ.W)


  // Predication
  def is_br            = br_type.isOneOf(B_NE, B_EQ, B_GE, B_GEU, B_LT, B_LTU)
  def is_jal           = br_type === B_J
  def is_jalr          = br_type === B_JR
  def is_sfb_br        = br_type =/= B_N && is_sfb && enableSFBOpt.B // Does this write a predicate
  def is_sfb_shadow    = br_type === B_N && is_sfb && enableSFBOpt.B // Is this predicated
  val ldst_is_rs1      = Bool() // If this is set and we are predicated off, copy rs1 to dst,
                                // else copy rs2 to dst

  // logical specifiers (only used in Decode->Rename), except rollback (ldst)
  val ldst             = UInt(lregSz.W)
  val lrs1             = UInt(lregSz.W)
  val lrs2             = UInt(lregSz.W)
  val lrs3             = UInt(lregSz.W)

  //@req-spec-decode.c25
  val dst_rtype        = UInt(3.W)
  val lrs1_rtype       = UInt(3.W)
  val lrs2_rtype       = UInt(3.W)
  val frs3_en          = Bool()

  // ==========================================================================
  // Vector (RVV) fields -- Caracal delta. Every field below is gated on
  // usingRVV via the Option idiom: with vectors disabled each is `None` and
  // elaborates to nothing, so the scalar-only bundle stays bit-identical to
  // pre-Caracal baseline BOOM v4 (no zero-width field, no tied-off field).
  // ==========================================================================

  //@req-spec-core.c4
  // An OP.v is an ordinary uop with is_vec set -- not a separate bundle type,
  // so it flows decode/rename/ROB/issue on the same paths as a scalar uop.
  val is_vec              = if (usingRVV) Some(Bool()) else None
  // Segmented load/store forms: needs both the vector LSU and the CII
  // coprocessor, gating the pvtmp rendezvous allocation and two-half issue.
  val is_shared            = if (usingRVV) Some(Bool()) else None

  // -- logical vector specifiers (decode -> rename only) --------------------
  val lvd                  = if (usingRVV) Some(UInt(lregSz.W)) else None
  val lvs1                 = if (usingRVV) Some(UInt(lregSz.W)) else None
  val lvs2                 = if (usingRVV) Some(UInt(lregSz.W)) else None
  val lvs3                 = if (usingRVV) Some(UInt(lregSz.W)) else None
  val lvm                  = if (usingRVV) Some(UInt(lregSz.W)) else None // mask reg (always v0)

  // -- renamed vector operands: groups, not registers ------------------------
  //@req-spec-rename.d5
  val pvdest               = if (usingRVV) Some(Vec(maxVecMembers, UInt(vecPregSz.W))) else None
  val stale_pvdest         = if (usingRVV) Some(Vec(maxVecMembers, UInt(vecPregSz.W))) else None

  //@req-spec-vrf.j1
  //@req-spec-vrf.j2
  //@req-spec-vrf.j4
  //@req-spec-vrf.j5
  //@req-spec-vrf.j6
  // pvs3 names an EXPLICITLY ENCODED third source (e.g. vse.v store data) and
  // is independent of stale_pvdest above -- they coincide for RMW arithmetic
  // but diverge for masked non-RMW ops, vslideup's prefix, vcompress's tail,
  // and must not be merged into one field.
  val pvs1                 = if (usingRVV) Some(Vec(maxVecMembers, UInt(vecPregSz.W))) else None
  val pvs2                 = if (usingRVV) Some(Vec(maxVecMembers, UInt(vecPregSz.W))) else None
  val pvs3                 = if (usingRVV) Some(Vec(maxVecMembers, UInt(vecPregSz.W))) else None
  val pvm                  = if (usingRVV) Some(UInt(vecPregSz.W)) else None // mask is one register, never a group

  //@req-spec-core.h8
  //@req-spec-rename.e6
  // The binding from the abstract pvtmp rendezvous to real PRNs -- there is
  // no separate table anywhere that records it; the two halves of a shared
  // instruction find each other by reading this field off the same OP.v.
  val pvtmp                = if (usingRVV) Some(Vec(maxVecMembers, UInt(vecPregSz.W))) else None

  // One busy bit per OPERAND (group-level, AND-reduced by the busy table),
  // not per member -- an operand wakes only when its last member is ready.
  val pvs1_busy            = if (usingRVV) Some(Bool()) else None
  val pvs2_busy            = if (usingRVV) Some(Bool()) else None
  val pvs3_busy            = if (usingRVV) Some(Bool()) else None
  val pvm_busy             = if (usingRVV) Some(Bool()) else None
  val pvtmp_busy           = if (usingRVV) Some(Bool()) else None
  val pvl_busy             = if (usingRVV) Some(Bool()) else None

  //@req-spec-rename.h18
  // Renamed VL this uop reads. Deliberately no stale_pvl (VL has a single
  // committed-map-table pointer released at commit, not a per-uop stale
  // value) and no pvtype (vtype is not renamed).
  val pvl                  = if (usingRVV) Some(UInt(vlPregSz.W)) else None

  // -- the static access descriptor ------------------------------------------
  val v_eew                = if (usingRVV) Some(UInt(2.W)) else None // data element width
  val v_idx_eew            = if (usingRVV) Some(UInt(2.W)) else None // index element width (indexed forms only)
  val v_emul               = if (usingRVV) Some(UInt((log2Ceil(maxVecMembers) + 1).W)) else None // group member count, 1..8
  val v_seg_nf             = if (usingRVV) Some(UInt(3.W)) else None // segment field count, segmented access

  // Decoded sense of the instruction's vm bit. Must be a field, not
  // re-derived from inst(25): the issue slot, VecMaskStream and VecElemAgen
  // each need it and none of them re-decodes the instruction word.
  val v_is_masked          = if (usingRVV) Some(Bool()) else None

  // access CLASS -- which agen this op goes to, decoded once by VLSDecode.
  val v_mop                = if (usingRVV) Some(UInt(2.W)) else None
  val v_is_unit_stride     = if (usingRVV) Some(Bool()) else None
  val v_is_strided         = if (usingRVV) Some(Bool()) else None
  val v_is_indexed         = if (usingRVV) Some(Bool()) else None
  val v_is_segment         = if (usingRVV) Some(Bool()) else None
  val v_is_whole_reg       = if (usingRVV) Some(Bool()) else None
  val v_is_mask            = if (usingRVV) Some(Bool()) else None
  val v_is_ff              = if (usingRVV) Some(Bool()) else None

  // "does this instruction's FORMAT actually encode this vector source" --
  // prevents e.g. vadd.vx's unencoded vs1 from waiting forever on a stale or
  // already-complete group that pvs1 happens to name.
  val v_uses_vs1           = if (usingRVV) Some(Bool()) else None
  val v_uses_vs2           = if (usingRVV) Some(Bool()) else None
  val v_uses_vs3           = if (usingRVV) Some(Bool()) else None

  // -- the vtype snapshot ------------------------------------------------------
  //@req-spec-vrf.c7
  //@req-spec-decode.d6
  // Taken at DECODE from the speculative VCFG mirror; carries VTYPE ONLY --
  // vstart/vxrm/vxsat are read from the CSR file at execute rather than
  // snapshotted, and vl is reached via pvl instead. vill doubles as the
  // mirror's poison flag.
  val vconfig              = if (usingRVV) Some(new VType) else None

  // -- the dual-destination bit -------------------------------------------------
  //@req-spec-decode.c18
  // Orthogonal to dst_rtype: a register-sourced vset has TWO destinations in
  // TWO independent rename spaces (pdst in the int file, pvl in the VL
  // file), and dst_rtype is single-valued so it cannot express both.
  val is_vl_producer       = if (usingRVV) Some(Bool()) else None

  // -- nOP.v-scoped element/segment cursor fields -------------------------------
  // Inert on the OP.v itself: populated only once it has been through the
  // vector LS AGEN, don't-care before that, and no consumer outside the
  // vector LSU may read any field in this group.
  //@req-spec-core.c11
  //@req-spec-core.c12
  val v_split_first        = if (usingRVV) Some(Bool()) else None
  val v_split_last         = if (usingRVV) Some(Bool()) else None
  val v_split_idx          = if (usingRVV) Some(UInt(vecVLSz.W)) else None
  val v_split_total        = if (usingRVV) Some(UInt(vecVLSz.W)) else None

  //@req-spec-agen.a4
  // Which register the access targets: the destination PRN it will write and
  // the byte offset within that PRN. A response can return out of order, so
  // the LCB places it by these two fields alone -- it cannot recover them
  // from the element index without re-deriving EMUL and the mask.
  val v_split_dst_prn      = if (usingRVV) Some(UInt(vecPregSz.W)) else None
  val v_split_dst_byte_off = if (usingRVV) Some(UInt(log2Ceil(vecVLen / 8).W)) else None

  // One named sub-bundle (see VecElemCursor above), not three loose fields.
  val v_elem_cursor        = if (usingRVV) Some(new VecElemCursor) else None

  val fcn_dw           = Bool()
  val fcn_op           = UInt(freechips.rocketchip.rocket.ALU.SZ_ALU_FN.W)

  // floating point information
  val fp_val           = Bool()             // is a floating-point instruction (F- or D-extension)?
                                            // If it's non-ld/st it will write back exception bits to the fcsr.
  val fp_rm            = UInt(3.W)
  val fp_typ           = UInt(2.W)

  // frontend exception information
  val xcpt_pf_if       = Bool()             // I-TLB page fault.
  val xcpt_ae_if       = Bool()             // I$ access exception.
  val xcpt_ma_if       = Bool()             // Misaligned fetch (jal/brjumping to misaligned addr).
  val bp_debug_if      = Bool()             // Breakpoint
  val bp_xcpt_if       = Bool()             // Breakpoint


  // What prediction structure provides the prediction FROM this op
  val debug_fsrc       = UInt(BSRC_SZ.W)
  // What prediction structure provides the prediction TO this op
  val debug_tsrc       = UInt(BSRC_SZ.W)

  // Do we allocate a branch tag for this?
  // SFB branches don't get a mask, they get a predicate bit
  def allocate_brtag   = (is_br && !is_sfb) || is_jalr

  def starts_bsy       = !(is_fence || is_fencei)
  // Is it possible for this uop to misspeculate, preventing the commit of subsequent uops?
  def starts_unsafe    = uses_ldq || (uses_stq && !is_fence) || is_br || is_jalr
}


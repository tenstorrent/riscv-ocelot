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

import boom.v4.common.{BoomBundle, HasBoomUOP, MicroOp}
import boom.v4.exu.{BrUpdateInfo, ExeUnitResp}

// GENERATED from src/main/nlhdl/pkg/VecBundles.nlhdl.scala. Do not hand-edit;
// regenerate via the nlhdl gen-rtl flow instead.
//
// VecBundles — every bundle that crosses a boundary between two vector nodes.
// `kind: package`: pure Bundle/enum declarations, no Module, no I/O of its
// own, no state, never instantiated. Other modules bind to it only through
// `depends_on:` (a compile-order edge) in hierarchy.yaml.
//
// PACKAGE NODE CONVENTION: the parameters a bundle needs come from
// VectorParams (via HasVectorParams's re-exports on HasBoomCoreParameters,
// already mixed into BoomBundle) and from HasBoomCoreParameters directly —
// never a literal width. See the per-bundle notes below for the few places
// this file had to depart from that rule because a named constant the spec
// calls for does not yet exist anywhere in the generated sources; each is
// flagged "SPEC DEFECT (reported, not resolved)" at the point it bites.
//
// Governing spec anchors: midcore.rst (group-done and the completion model),
// loadstore.rst `ssi-queues` and `elem-progress` (the element access and the
// queue set), cii.rst `cii-interface` (the four channel payloads),
// issue.rst `vec-queue-reservation`.

// =============================================================================
// ---- VecGroupDone: the completion event ----
// =============================================================================
//
// Announces that one whole destination group has completed. It carries the
// completing group's FULL MEMBER-PRN VECTOR (not a base+count: the free list
// allocates a group without requiring contiguous PRNs, so a consumer matches
// each of its source group's members against this vector) together with a
// valid-member count, the rob_idx of the owning OP.v, and an
// is_vl_producer-style `pvl` field for the case where the producer also wrote
// VL.
//
// ONE event, THREE consumers (the ROB's single-shot rob_bsy clear, the vector
// Busy-Table clear, and the vector wakeup network) is why this is one bundle
// rather than three narrower ones: the three consumers must see the same
// completion in the same cycle, and a split bundle would let them drift.
//
// perf: this bundle is matched per member against every wakeup port in every
// vector issue slot each cycle (width multiplied by slots x ports in the
// issue-stage comparator budget) — kept to exactly the member PRNs, the count
// and the ownership fields for that reason.
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

// =============================================================================
// ---- VecElemAccess: the nOP.v ----
// =============================================================================
//
// The cracked element access ("nOP.v") that address generation emits and the
// drain side consumes. Wraps the originating OP.v's MicroOp (via HasBoomUOP,
// the same idiom BoomDCacheReq uses) and adds the access payload only.
//
// The fields identifying WHICH register the access targets (destination PRN,
// byte offset within it) are deliberately NOT declared here a second time:
// they are the nOP.v-scoped cursor fields already carried by the wrapped
// MicroOp (v_split_dst_prn, v_split_dst_byte_off, v_elem_cursor) — declaring
// them again would give the drain side two places to read the same thing
// from, and one of them would go stale.
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

// =============================================================================
// ---- The element queue set ----
// =============================================================================
//
// Named enumeration rather than six unrelated queue instances, so a module
// naming a queue cannot name one that does not exist. Exactly six members,
// `{ld,st}_{SSI,US}_{ADDR,DATA}_Q`. Deliberately NO ld_*_DATA_Q in either
// class: a load's returning data goes to the LCB for assembly, not into a
// queue — the asymmetry is real, not an omission.
//
// Address generation delivers its nOP.v bundles into these dedicated queues
// and nowhere else — never into an LDQ or STQ slot, which hold one
// placeholder entry per vector instruction for ordering/commit only.
//@req-spec-lsu.a14
//@req-spec-lsu.a15
//@req-spec-agen.a5
object VecQueueId {
  val ld_SSI_ADDR_Q :: st_SSI_ADDR_Q :: st_SSI_DATA_Q :: ld_US_ADDR_Q :: st_US_ADDR_Q :: st_US_DATA_Q :: Nil =
    Enum(6)
}

// `VecRangeEntry` is the unit-stride counterpart of `VecElemAccess`: one entry
// describing a whole contiguous byte range, standing in for what would
// otherwise be up to `vLen` element accesses.
//
// THE FIELD LIST BELOW IS THE COMPLETE AUTHORITATIVE ENUMERATION per the
// nlhdl source: it is the ONLY thing that remembers a unit-stride instruction
// (which is precisely how VecRangeAgen stays free of per-instruction state),
// so anything the drain side, the LCB, the forwarder or the squash unit needs
// about that instruction must be on the entry. A generator must emit exactly
// this list; a consumer needing a fifth thing amends this list rather than
// deriving it locally.
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
  // Segment field count, read by VecBeatExpander's segment constraint
  // (1 << eew when nf > 1, unbounded when nf == 1). nf > 1 does not in
  // practice reach the US queues; this is a representability safety net.
  val nf             = UInt(3.W) // matches MicroOp.v_seg_nf's encoding width
  // The access's ACTIVE MASK.
  // ===> REPORTED, NOT RESOLVED (mirrors the nlhdl source's own note): this
  // file's own text states BYTE mask, vLen/8 = 32 bits, and that is what is
  // implemented below. Three consumers (VecRangeAgen, VecBeatExpander,
  // VecMaskStream) are cited in the spec as reading it ELEMENT-granular and
  // wider (up to VLMAX = 256 bits). The two readings are not interchangeable
  // and this file does not pick a side beyond restating its own declared
  // position — see the nlhdl source's dependencies section for the full
  // cross-file conflict. A generator must not silently truncate; flagged here
  // for the same reason the source flags it.
  val mask           = UInt((vecVLen / 8).W)
  // Destination group's base PRN and member count, from pvdest/v_emul.
  val pvdest_base    = UInt(vecPregSz.W)
  val members        = UInt(log2Ceil(maxVecMembers + 1).W)
  // For a STORE, the absolute base INDEX of this access's region in
  // st_US_DATA_Q, filled from reservation slot 1. NOT derivable from the
  // address region's base (see VecReservation / the nlhdl source's note on
  // why the US store address/data regions are not in identity
  // correspondence). Meaningless (don't-care) on a load or on an ld_*
  // queue's entry.
  val us_data_base   = UInt(log2Ceil(usQueueEntries).W)
  // Ownership fields, so the drain side, the LCB and the squash unit can each
  // name the instruction from the entry alone. Both are carried (rather than
  // only whichever applies) because one VecRangeEntry class is shared by both
  // the ld_* and st_* members of the queue set; the unused one is don't-care.
  val rob_idx        = UInt(robAddrSz.W)
  val ldq_idx        = UInt((1 + ldqAddrSz).W)
  val stq_idx        = UInt((1 + stqAddrSz).W)
}

// =============================================================================
// ---- VecReservation ----
// =============================================================================
//
// The dispatch-time capacity claim: which queue, how many entries, and the
// owning rob_idx plus the reserving ldq_idx/stq_idx. Carries the
// reservation's index region (base + count) so a squash can roll a queue's
// tail pointer back to the youngest surviving reservation without a
// per-entry comparison.
class VecReservation(implicit p: Parameters) extends BoomBundle
{
  // One of the six VecQueueId values. Width derived from the enumeration
  // itself (not a literal) so a widening of the queue set can't silently
  // desync this field from VecQueueId.
  val queue   = UInt(VecQueueId.ld_SSI_ADDR_Q.getWidth.W)
  // Base index of the reserved region. Sized against the deeper (SSI) queue
  // class so one field covers a reservation in either class.
  val base    = UInt(log2Ceil(ssiQueueEntries).W)
  val entries = UInt(log2Ceil(ssiQueueEntries + 1).W)
  val rob_idx = UInt(robAddrSz.W)
  val ldq_idx = UInt((1 + ldqAddrSz).W)
  val stq_idx = UInt((1 + stqAddrSz).W)
}

// =============================================================================
// ---- VecException ----
// =============================================================================
//
// Reports a vector memory fault to the ROB as a plain precise exception.
// Deliberately carries NO element index: a faulting vector op traps with
// vstart = 0 and restarts whole, so an element index reaching the ROB could
// only be misused.
class VecException(implicit p: Parameters) extends BoomBundle
{
  val valid    = Bool()
  val rob_idx  = UInt(robAddrSz.W)
  val cause    = UInt(xLen.W)
  val badvaddr = UInt(coreMaxAddrBits.W)
}

// =============================================================================
// ---- The four CII channel payloads ----
// =============================================================================
//
// The Chisel view of the frozen SV contract in tt_cii_caracal_pkg.svh. Every
// width is derived from ciiTagBits, vecVLen or a named constant of that
// package — never a literal — because the SV side is authoritative and a
// disagreement here is a silent protocol break, not a compile error.
//
// ALL FOUR ARE PER-LANE PAYLOADS, AND THE CHANNEL'S GRAIN IS THE BEAT:
// tt_cii_interface.sv gives each channel exactly ONE valid and ONE credit for
// a beat of N lanes. So NO bundle here may grow a valid, credit, ready or
// lane-index field: per-lane ACTIVITY is encoded in the payload itself (e.g.
// op_id = CII_SRC_NONE on an unused Src-Request lane). The only index a
// payload carries is a MEMBER index (op_offset, wb_dst_offset), never a lane
// index and never a register number.

// All four tt_cii_caracal_pkg.svh localparams the spec names are mirrored, so
// no width below is a bare literal: CII_TAG_W -> `ciiTagBits`,
// CII_NUM_SRC_SLOTS -> `ciiNumSrcSlots` (added to VectorParams at A2, for
// exactly this), CII_VL_W -> the derived `vecVLSz`, CII_MEMBER_W ->
// `log2Ceil(maxVecMembers)`. All are in scope on BoomBundle via
// HasBoomCoreParameters. The rule matters because a flat BlackBox port
// disagreeing by one bit shifts a whole payload without a width error.
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
// Host to coprocessor. vtype/vl/vxrm are the Caracal EXTENSION to the
// generic tt_cii_interface.sv issue struct (which carries tag+instr only) —
// they let the coprocessor hold no cross-instruction configuration state.
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
// Coprocessor to host. Names operands by abstract slot and group member
// only — no register number of any kind, architectural or physical.
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
// Host to coprocessor. Carries the operand data and NOTHING else — not even
// the tag: the channel is ordered, so the coprocessor correlates a beat with
// its request by arrival order rather than by a field. That is exactly why a
// killed tag's request must still be answered: an omitted beat would
// desynchronise the channel for every surviving instruction (see
// VecCiiFlush).
class CiiSrcData(implicit p: Parameters) extends BoomBundle
{
  val data = UInt(vecVLen.W)
}

//@req-spec-cii.a12
//@req-spec-cii.a15
// Coprocessor to host. `last` is the Caracal extension to the generic
// writeback struct and is the ONLY completion signal: the channel carries no
// expected-count field, and the host must never infer completion by counting
// beats (widening/narrowing ops emit a member count that differs from the
// source EMUL).
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

// =============================================================================
// ---- VecPipelineIO ----
// =============================================================================
//
// Realizes the `vec_pipeline_io` interface declared in hierarchy.yaml — the
// single bundle across which BoomCore and VecPipeline communicate. Declared
// field-for-field against that interface entry, as the nlhdl source
// instructs.
//
// Direction convention (the nlhdl source does not spell this out, so it is
// documented here as the conservative, single choice this file makes): this
// bundle is authored from VecPipeline's OWN io perspective, matching the
// `instantiates: connect: io: { interface: vec_pipeline_io }` entry on
// VecPipeline in hierarchy.yaml. `dir: fwd` (BoomCore -> VecPipeline) is
// therefore Input(...) and `dir: bwd` (VecPipeline -> BoomCore) is
// Output(...).
//
// ===> THE RENAME INPUTS ARE NAMED ren2_uops AND dis_fire, NOT dec_uops, and
// the name is load-bearing rather than cosmetic. Vector rename must allocate
// in lockstep with the scalar RenameStage's REGISTERED ren1->ren2 pipeline.
// Driving it combinationally from dec_uops runs it one cycle ahead, so at
// dispatch the vector fields describe the NEXT cycle's (bubble) uop and two
// ops free the same PRN. With these port names, connecting dec_uops here is
// visibly wrong at the connection site.
//
// ===> SPEC DEFECT (reported, not resolved) — FIVE FIELDS OMITTED. Five
// signals named by hierarchy.yaml's vec_pipeline_io interface are typed
// against bundles that have NO field-level definition anywhere: not in this
// file's own logic section, and not declared by any other generated source
// in the repo as of this writing. Declaring them here with an invented field
// list would be fabricating spec content this file was never given — the
// same category of problem the nlhdl source itself already flags for
// VecCiiTagEntry ("recorded... so the discrepancy cannot be closed by each
// side assuming the other did it"), and the same discipline is applied here:
// report, do not invent.
//   - vec_rob_flags : Vec(numVecClrPorts, Valid(VecRobFlags))  [bwd]
//   - int_rf_read_req : Vec(5, DecoupledReadReq)               [bwd]
//   - int_wakeups : IntWakeupBus                                [fwd]
//   - fp_wakeups : FpWakeupBus                                  [fwd]
//   - int_wb_snoop : Vec(numIrfWritePorts, IntWbSnoop)          [fwd]
// `src/main/nlhdl/vec/VecPipeline.nlhdl.scala` (a sibling node, not this
// one's spec) explicitly asserts that IntWakeupBus, FpWakeupBus, IntWbSnoop
// and VecRobFlags "still" need to be declared in VecBundles — i.e. it
// documents the very node this file implements as owing a declaration this
// file's own nlhdl source never wrote. `src/main/nlhdl/host/Rob.nlhdl.scala`
// makes the identical claim about VecRobFlags independently. Until
// VecBundles.nlhdl.scala is amended with these five bundles' field lists (an
// architect/gen-nlhdl action, not a gen-rtl one), int_rf_read_req's
// correlated response half (int_rf_read_rsp, a plain UInt) is declared below
// on its own — the two were never a single indivisible type.
//
// csr_vector's type, `freechips.rocketchip.rocket.CSRVectorIO`, is kept
// (unlike the five above) because the nlhdl source's own dependency section
// and VecPipeline.nlhdl.scala both name an intended home for it (rocket-chip,
// not this package) — but a repo-wide search at generation time found no
// such class anywhere under generators/rocket-chip. This is flagged
// separately from the five omissions above: it is not a missing declaration
// site, it is a dependency on a rocket-chip class that does not yet exist in
// this checkout. The reference is kept (as a forward declaration) rather
// than omitted, since — unlike the five above — omitting it would contradict
// an otherwise-unambiguous intended home.
//
// lsu_vec's type, `boom.v4.lsu.VecLsuCoreIO`, is a genuine forward reference
// too, but a RESOLVED one: `src/main/nlhdl/host/LSU.nlhdl.scala` states
// unambiguously "VecLsuCoreIO is declared HERE, beside LSUCoreIO, NOT [in
// VecBundles]" — so its absence from this file is the documented design, not
// a gap.
class VecPipelineIO(implicit p: Parameters) extends BoomBundle
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

  // ---- rename / dispatch lockstep (see load-bearing-names note above) ---
  val ren2_uops       = Input(Vec(coreWidth, new MicroOp()))
  val ren2_mask       = Input(Vec(coreWidth, Bool()))
  val dis_fire        = Input(Vec(coreWidth, Bool()))
  val dis_ready       = Output(Bool())
  // Three vector issue queues wired natively elsewhere; this carries only the
  // per-lane, per-queue capacity check. Ordering is
  // {IQ_V_LOAD, IQ_V_STORE, IQ_V_ALU} x coreWidth.
  val dis_vec_valids  = Input(Vec(3, Vec(coreWidth, Bool())))
  val dis_vec_ready   = Output(Vec(3, Vec(coreWidth, Bool())))
  // The return path for the renamed uop: Rob.io.enq_uops needs the vector
  // PRNs. Consumed by rob.io.enq_uops ONLY.
  val dis_uops_out    = Output(Vec(coreWidth, new MicroOp()))

  // ---- speculation / recovery -----------------------------------------
  val brupdate        = Input(new BrUpdateInfo)
  val rob_pnr_idx     = Input(UInt(robAddrSz.W))
  // The 3-arg IsOlder(a, b, head) age comparison needs the head to
  // disambiguate ROB wraparound; without it a past-PNR gate would invert
  // across a wrap boundary.
  val rob_head_idx    = Input(UInt(robAddrSz.W))
  val rob_flush       = Input(Bool())
  val rob_flush_kill  = Input(Bool())
  val rob_empty       = Input(Bool())

  // ---- commit ---------------------------------------------------------
  val commit_valids   = Input(Vec(coreWidth, Bool()))
  val commit_uops     = Input(Vec(coreWidth, new MicroOp()))
  val commit_rollback = Input(Bool())

  // ---- completion back into the ROB (group-done) -----------------------
  // ONE LANE PER PRODUCER, never arbitrated: a lost clear is unrecoverable
  // and the ROB entry never retires. Lane 0 = LCB group-done, lane 1 =
  // VecCiiComplete, lane 2 = VecGroupCopy.
  val vec_clr_bsy      = Output(Vec(vectorParams.numVecClrPorts, Valid(UInt(robAddrSz.W))))
  val vec_clr_unsafe   = Output(Valid(UInt(robAddrSz.W)))
  val vec_xcpt         = Output(Valid(new VecException))
  // vec_rob_flags (VecRobFlags * numVecClrPorts, bwd) OMITTED — see the
  // SPEC DEFECT note above the class.

  // ---- scalar feeders: vector ops read INT/FP for base/stride/.vx/.vf ---
  // int_rf_read_req (DecoupledReadReq * 5, bwd) OMITTED — see the SPEC DEFECT
  // note above the class. Its correlated response half is declared below on
  // its own, since the two were never a single indivisible type.
  val int_rf_read_rsp  = Input(Vec(5, UInt(xLen.W)))
  // int_wakeups (IntWakeupBus, fwd) OMITTED — see the SPEC DEFECT note above.
  // fp_wakeups (FpWakeupBus, fwd) OMITTED — see the SPEC DEFECT note above.
  // int_wb_snoop (IntWbSnoop * numIrfWritePorts, fwd) OMITTED — see the SPEC
  // DEFECT note above.
  // Naming assumption: the nlhdl source names this field's width bare
  // `pregSz`, a name with no declaration anywhere in HasBoomCoreParameters.
  // Read as `fpregSz` (the FP physical-register address width) since the
  // sole reader is VecCiiIssue's `.vf` scalar operand, which is FP-typed.
  val fp_rf_read_req   = Output(UInt(fpregSz.W))
  val fp_rf_read_rsp   = Input(UInt(xLen.W))

  // ---- scalar-dest writeback (vmv.x.s, vcpop.m, vfirst.m, vfmv.f.s) ----
  val int_wb           = Output(Valid(new ExeUnitResp(xLen)))
  val fp_wb            = Output(Valid(new ExeUnitResp(xLen)))

  // ---- vector CSR state (OWNED by rocket CSRFile, see frontend.rst) -----
  // ===> SPEC DEFECT, DEFERRED TO D2 (not papered over). The spec names a
  // `CSRVectorIO` in rocket-chip as this field's type. NO SUCH CLASS EXISTS:
  // rocket declares the port anonymously as
  //   val vector = usingVector.option(new Bundle { ... })
  // at rocket-chip/src/main/scala/rocket/CSR.scala:310, so there is no name to
  // reference and no way to declare this field without either inventing a
  // Chisel type or restating rocket's anonymous bundle field-for-field (which
  // would then silently drift from it). Both are worse than leaving it out.
  // The field is therefore OMITTED here rather than declared wrong. Ground
  // rule 9 is unaffected — the CSR state is still rocket's; only the Chisel
  // handle for it is unresolved. Owner: step D2's BoomCore delta, which is
  // where `csr.io.vector` is actually in scope and where the seam is wired.
  // Fix in the spec first (frontend.rst `vector-csr-ownership` + VecBundles),
  // then regenerate; do not hand-patch this file.
  val csr_frm          = Input(UInt(3.W))
  val csr_vs_dirty     = Output(Bool())

  // ---- vset: the dual-destination rule (is_vl_producer) ----------------
  // Replicated per ALU EU (never arbitrated: a single-shot VL wakeup lost to
  // arbitration is a permanent hang).
  val vset_resp        = Input(Vec(aluWidth, Valid(new ExeUnitResp(xLen))))
  // Two lanes, never arbitrated: the ALU vset writeback and the vleff VL trim
  // are independent producers.
  val vl_wakeup        = Output(Vec(numVlWakeupPorts, Valid(UInt(vlPregSz.W))))
  // VL-RF commit read data. The read ADDRESS is selected inside VecPipeline
  // from commit_uops, so no address member is needed here.
  val commit_vl        = Output(Valid(UInt(vecVLSz.W)))

  // ---- memory: the vector LSU's own D$ port + ordering hooks -----------
  // FORWARD REFERENCE TO A LATER PHASE, deliberately omitted at A2.
  // `VecLsuCoreIO` is declared by the LSU delta (src/main/nlhdl/host/LSU.nlhdl
  // .scala -> src/main/scala/v4/lsu/lsu.scala), which lands at step E7, not
  // here: A2 generates the four packages only. Declaring it now would mean
  // inventing the LSU seam ahead of the step that owns it.
  // When E7 lands, restore as:
  //   val lsu_vec = Flipped(new boom.v4.lsu.VecLsuCoreIO)
  // with Flipped() presenting that sub-bundle's LSU-side Input/Output split
  // from VecPipelineIO's (core-receives) perspective.
  val lsu_fencei_rdy_vec = Output(Bool())

  // ---- debug / trace (ground rule: guarded printf, off by default) -----
  val vec_trace_en   = Input(Bool())
  val debug_vrf_read = Output(Vec(coreWidth, UInt(vecVLen.W)))
}

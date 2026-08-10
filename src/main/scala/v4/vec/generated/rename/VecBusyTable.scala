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

import boom.v4.common.{BoomBundle, BoomModule, MicroOp}
import boom.v4.vec.generated.{VecGroupDone, VecTrace}

// GENERATED from src/main/nlhdl/vec/rename/VecBusyTable.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
//
// VecBusyTable -- the per-PRN readiness bit vector of one renamed vector-side
// register space, set on group allocation and cleared by group-done. The
// vector analogue of `RenameBusyTable` (v4/exu/rename/rename-busytable.scala,
// untouched, still serving INT/FP): same three-part shape (state, set,
// clear+read), but every difference below exists because a vector operand is
// a GROUP, not a register.
//
// Instantiated TWICE by VecRenameSpace, as `busytable`: once as the VECTOR
// busy table (`numPregs = numVecPhysRegisters`, `maxGroupSize = maxMembers`,
// `wakeupKind = "group_done"`, `exportMemberRdy = true`), once as the VL busy
// table (`numPregs = numVlPhysRegisters`, `maxGroupSize = 1`,
// `wakeupKind = "ready_bit"`, `exportMemberRdy = false`). `vectorInstance`
// below is exactly `wakeupKind == "group_done"` -- the nlhdl parameters
// section states that mapping directly ("group_done" for the vector
// instance, "ready_bit" for the VL instance), so branching on it is reading
// the spec, not inventing a new distinguishing parameter.
//
// TWO THINGS A READER MUST NOT GET WRONG (nlhdl header):
// (1) The table is indexed PER MEMBER PRN, never a group base -- a source
//     group can be a sub-range of a larger in-flight destination group, and
//     a group's members can come from different producers.
// (2) `busy_resps` is the rename-stage AGGREGATE (one Bool per operand);
//     `member_busy_resps` (exportMemberRdy only) is the PRE-REDUCTION
//     per-member export VecGroupReady needs to initialize its per-member
//     state -- two consumers of the same completion event at two
//     granularities, on purpose.
//
// GROUND RULE 7 (group-done, single-shot): every vector producer signals
// completion of a whole destination group ONCE, carrying the group's
// member-PRN vector. ONE event drives THREE consumers -- the ROB's
// single-shot busy-clear, this table's clear, and the vector wakeup network
// -- and this module is exactly one of those three. There is no per-entry
// ROB completion counter and no per-PRN clear port beside the group-done
// ports.
//
// Governing spec anchors: midcore.rst `busy-table` and `group-done`,
// `vl-vtype-rename`; `cii-shared-mapping`, `regfiles-bypass`; glossary.rst
// `glossary-terms`.

// =============================================================================
// ---- VecBusyResp: the rename-stage AGGREGATE response ----
// =============================================================================
//
// Declared here (not VecBundles) for the same locality reason the scalar
// `BusyResp` is declared beside `RenameBusyTable`: it crosses only the
// boundary to this module's own parent.
//
// SPEC NOTE (not a defect, a necessary reading): the nlhdl source names ONE
// class `VecBusyResp` but gives it a DIFFERENT field set per instance --
// {pvs1_busy, pvs2_busy, pvs3_busy, pvm_busy, pvtmp_busy} on the vector
// instance, the single field {pvl_busy} on the VL instance -- and says
// "Names match the MicroOp busy fields exactly". `MicroOp` itself carries
// all six of those fields unconditionally (they are joined by
// VecRenameSpace, not this module). A single non-parameterized Scala class
// cannot have two different field sets, so this class is parameterized on
// `vectorInstance`, using the `Option`-field idiom `MicroOp` itself already
// uses for usingRVV-gated fields. This keeps one class name, matching the
// letter of "VecBusyResp is declared in this file" (singular), while
// actually emitting the field set the spec describes for each instance.
class VecBusyResp(val vectorInstance: Boolean)(implicit p: Parameters) extends BoomBundle
{
  //@req-spec-rename.g21
  val pvs1_busy  = if (vectorInstance) Some(Bool()) else None
  //@req-spec-rename.g21
  val pvs2_busy  = if (vectorInstance) Some(Bool()) else None
  //@req-spec-rename.g21
  val pvs3_busy  = if (vectorInstance) Some(Bool()) else None
  //@req-spec-rename.g21
  val pvm_busy   = if (vectorInstance) Some(Bool()) else None
  //@req-spec-rename.g21
  val pvtmp_busy = if (vectorInstance) Some(Bool()) else None
  // VL instance only -- part 7. NOT present on the vector instance: that
  // table has no pvl read port at all (VecRenameSpace joins the two
  // responses when it writes the uop's busy fields).
  //@req-spec-rename.h12
  //@req-spec-rename.h13
  //@req-spec-rename.h14
  //@req-spec-rename.g12
  //@req-spec-rename.g26
  val pvl_busy   = if (!vectorInstance) Some(Bool()) else None

  // Deliberately NO `pvold_busy` field here (part 10): an aggregated
  // stale_pvdest busy bit would be unsafe, not merely conservative -- see
  // VecMemberBusyResp below and the module's part-10 note.
}

// =============================================================================
// ---- VecMemberBusyResp: the PRE-REDUCTION per-member export (D6 / A2) ----
// =============================================================================
//
// Exists ONLY when `exportMemberRdy` (vector instance only). Same per-member
// wires part 5's reductions consume, tapped BEFORE the AND-tree -- no second
// read of `busy_table`. `pvold_busy` is the per-member readiness of
// `stale_pvdest` (decision D6): the ONE thing this port adds that `busy_resps`
// does not carry, because a single aggregate bit cannot express "waiting on
// producer 3 of 8" when a stale group's members come from different
// producers (part 10).
class VecMemberBusyResp(val maxGroupSize: Int)(implicit p: Parameters) extends BoomBundle
{
  //@req-spec-rename.g11
  val pvs1_busy  = Vec(maxGroupSize, Bool())
  //@req-spec-rename.g11
  val pvs2_busy  = Vec(maxGroupSize, Bool())
  //@req-spec-rename.g11
  val pvs3_busy  = Vec(maxGroupSize, Bool())
  //@req-spec-rename.g11
  val pvtmp_busy = Vec(maxGroupSize, Bool())
  //@req-spec-vrf.d6
  val pvold_busy = Vec(maxGroupSize, Bool())
  //@req-spec-rename.g11
  val pvm_busy   = Bool() // pvm is one register, never a group
}

// =============================================================================
// ---- VecBusyTableIO ----
// =============================================================================
//
// Deliberately NO port for: a brupdate/flush input (part 8 -- a wrong-path
// group's busy bits are self-correcting through realloc+group-done), a
// child_rebusys input or any speculative-rebusy path (part 4 -- vector
// operands are never woken speculatively), a per-PRN clear port beside the
// group-done ports, a `busy` output of any kind, and an AGGREGATED
// `stale_pvdest` busy bit in `VecBusyResp` (part 10).
class VecBusyTableIO(
  val plWidth:         Int,
  val numPregs:        Int,
  val maxGroupSize:    Int,
  val numWbPorts:      Int,
  val wakeupKind:      String,
  val exportMemberRdy: Boolean)
  (implicit p: Parameters) extends BoomBundle
{
  private val pregSz          = log2Ceil(numPregs)
  private val vectorInstance  = wakeupKind == "group_done"

  //@req-spec-rename.g22
  // Vec(plWidth, MicroOp): GROUP MEMBER PRNs plus EMUL, not a base and a
  // size -- pvdest/pvs1/pvs2/pvs3/pvtmp/stale_pvdest are each
  // Vec(maxVecMembers, UInt(vecPregSz.W)), pvm/pvl are single PRNs, v_emul is
  // the 1..8 member count. Member PRNs, not a base, because the free list
  // allocates a group without requiring contiguous PRNs.
  val ren_uops = Input(Vec(plWidth, new MicroOp))

  //@req-spec-rename.g23
  // One bit per lane: this lane allocated a destination group and its
  // members must be marked busy. One request per OP.v setting up to 8 bits,
  // never one request per member.
  val rebusy_reqs = Input(Vec(plWidth, Bool()))

  //@req-spec-rename.g21
  // Per-lane, PER-GROUP source readiness already aggregated to group-ready
  // bits -- see VecBusyResp above for why its field set is parameterized.
  val busy_resps = Output(Vec(plWidth, new VecBusyResp(vectorInstance)))

  //@req-spec-rename.g11
  // Elaborated only when exportMemberRdy. Three downstream readers
  // (VecRenameSpace's member_rdy, VecIssueUnit's dis_member_rdy,
  // VecGroupReady's in_member_rdy, including its rdy_vold instance) are the
  // reason this port exists.
  val member_busy_resps =
    if (exportMemberRdy) Some(Output(Vec(plWidth, new VecMemberBusyResp(maxGroupSize)))) else None

  //@req-spec-rename.g24
  //@req-spec-rename.g25
  // Exactly numWbPorts wide, no wider -- the same bus the issue slots match
  // on. With wakeupKind = "group_done" each port carries a VecGroupDone (the
  // completing group's member-PRN vector plus a members count and rob_idx).
  // With "ready_bit" the member vector degenerates to one PRN and the port
  // is Valid(UInt(pregSz.W)) -- the PRN whose VL writeback completed. Both
  // shapes share all the clear logic below; only member-list extraction
  // differs (see wakeupMemberPrns/wakeupMemberCount in the module).
  //
  // Chisel-mechanics note: the port's NAME is fixed ("wakeups") across both
  // instances, but its PAYLOAD TYPE differs by elaboration-time
  // `wakeupKind`. Since a Bundle field's Chisel type must be a single
  // concrete `Data`, the two payload shapes are unified here at their common
  // supertype (`Data`); the module casts back to the concrete type with
  // `wakeupKind` as the (elaboration-constant, therefore safe) discriminant.
  val wakeups = Input(Vec(numWbPorts, Valid(
    (if (vectorInstance) new VecGroupDone else UInt(pregSz.W)): Data
  )))

  // Raw state for waveform inspection only -- nothing functional may read
  // it. Mirrors RenameBusyTable's identically-named port.
  val debug = new Bundle { val busytable = Output(Bits(numPregs.W)) }
}

/**
 * VecBusyTable -- see the file header for the full design rationale.
 * Instantiated twice by VecRenameSpace, as `busytable`.
 *
 * @param plWidth         rename lanes served (coreWidth).
 * @param numPregs        size of the space this instance covers
 *                        (numVecPhysRegisters or numVlPhysRegisters).
 * @param maxGroupSize    largest member count of one group (maxMembers, or 1
 *                        on the VL instance).
 * @param numWbPorts      completion ports on the clear side (numVecWbPorts,
 *                        or the VL writeback port count).
 * @param wakeupKind      "group_done" or "ready_bit" -- selects the clear
 *                        port shape. Kept explicit rather than derived from
 *                        `maxGroupSize == 1`: free discipline and wakeup
 *                        kind are independent decisions.
 * @param exportMemberRdy true for the vector instance only. Gates the
 *                        `member_busy_resps` port and the `stale_pvdest`
 *                        read.
 */
class VecBusyTable(
  val plWidth:         Int,
  val numPregs:        Int,
  val maxGroupSize:    Int,
  val numWbPorts:      Int,
  val wakeupKind:      String,
  val exportMemberRdy: Boolean = false)
  (implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecBusyTable: elaborated only under usingRVV (never rocket's usingVector)")
  require(wakeupKind == "group_done" || wakeupKind == "ready_bit",
    s"""VecBusyTable: wakeupKind ("$wakeupKind") must be "group_done" or "ready_bit"""")
  require(!exportMemberRdy || maxGroupSize > 1,
    "VecBusyTable: exportMemberRdy requires maxGroupSize > 1 -- a one-member group's " +
    "per-member vector carries nothing the aggregate does not")

  val vectorInstance = wakeupKind == "group_done"
  val pregSz         = log2Ceil(numPregs)

  // "on the vector instance" (nlhdl parameters section): at least one whole
  // LMUL=8 group must be representable, or the machine deadlocks at rename
  // rather than stalling. The tighter capacity requires live in
  // VectorParams; this is the defensive floor at the module boundary.
  if (vectorInstance) {
    require(numPregs >= 32 + maxGroupSize,
      s"VecBusyTable: numPregs ($numPregs) must be >= 32 + maxGroupSize ($maxGroupSize) " +
      "on the vector instance, or no full group can ever be renamed")
  }

  val io = IO(new VecBusyTableIO(plWidth, numPregs, maxGroupSize, numWbPorts, wakeupKind, exportMemberRdy))

  // ===========================================================================
  // ---- Wakeup-port extraction (shared by clear-mask and the assertion) ----
  // ===========================================================================
  //
  // `wakeupKind` is an elaboration-time Scala String, fixed for the whole
  // life of this module instance, so exactly one of the two branches below
  // is ever exercised by a given elaborated instance -- the `asInstanceOf`
  // casts are therefore exact, not a lie, matching how `io.wakeups`' payload
  // was actually constructed above.
  private def wakeupMemberPrns(w: Int): Seq[UInt] =
    if (vectorInstance) io.wakeups(w).bits.asInstanceOf[VecGroupDone].pvdest
    else Seq(io.wakeups(w).bits.asInstanceOf[UInt])

  private def wakeupMemberCount(w: Int): UInt =
    if (vectorInstance) io.wakeups(w).bits.asInstanceOf[VecGroupDone].members
    else 1.U

  // =========================================================================
  // ---- 1. State ----
  // =========================================================================

  //@req-spec-rename.g1
  //@req-spec-rename.g6
  //@req-spec-rename.g7
  // The readiness status of EACH physical register in this space, bit p set
  // meaning "PRN p has an in-flight producer and is not yet readable". Flat,
  // not Vec(numGroups, Bits): a group's members are non-contiguous PRNs from
  // the free list, so no grouping of the storage could match a group that is
  // actually allocated. RegInit(0.U(...)): every PRN ready at reset, because
  // no producer is in flight then and a table coming up all-busy would
  // deadlock the first vector consumer.
  val busy_table = RegInit(0.U(numPregs.W))

  // =========================================================================
  // ---- 2. Set-busy on allocation ----
  // =========================================================================

  //@req-spec-rename.g8
  //@req-spec-rename.g9
  //@req-spec-rename.g10
  // Per lane, OR over members j of UIntToOH(pvdest(j)), qualified by
  // rebusy_reqs(i) and by j < v_emul: ALL member bits of the destination
  // group are set in the one cycle the group is renamed. The j < v_emul
  // qualifier keeps an EMUL=1 op from marking 7 unrelated PRNs busy -- PRN 0
  // is a real allocatable vector PRN, so a stale member would hang the first
  // future owner of it.
  //
  //@req-spec-core.h3
  //@req-spec-rename.e7
  //@req-spec-rename.e12
  //@req-spec-vrf.d6
  // (part 6) pvtmp is an ORDINARY VRF GROUP with a REAL BUSY LIFETIME, no
  // separate temp busy table: the producer half's pvtmp members are set busy
  // by this SAME OR-of-UIntToOH structure, qualified by the same
  // rebusy_reqs(i) and additionally by is_shared. This code is generic
  // across both instances (part 7: "no VL-specific code path is needed") --
  // on the VL instance, VecRenameSpace's private uop must drive is_shared
  // false, so this term evaluates to zero there. ASSUMPTION: pvtmp shares
  // pvdest's own v_emul (MicroOp has no separate temp-group member count).
  private def setMaskFor(i: Int): UInt = {
    val uop  = io.ren_uops(i)
    val emul = uop.v_emul.get
    val destMask = (0 until maxGroupSize).map { j =>
      UIntToOH(uop.pvdest.get(j), numPregs) & Fill(numPregs, io.rebusy_reqs(i) && (j.U < emul))
    }.reduce(_ | _)
    val tmpMask = (0 until maxGroupSize).map { j =>
      UIntToOH(uop.pvtmp.get(j), numPregs) &
        Fill(numPregs, io.rebusy_reqs(i) && uop.is_shared.get && (j.U < emul))
    }.reduce(_ | _)
    destMask | tmpMask
  }
  val setMaskOR = (0 until plWidth).map(setMaskFor).reduce(_ | _)

  // =========================================================================
  // ---- 3. Clear-busy on group-done ----
  // =========================================================================

  //@req-spec-rename.g15
  //@req-spec-rename.g18
  //@req-spec-rename.g16
  //@req-spec-rename.g17
  // Per wakeup port, OR over that port's members of UIntToOH(member_prn),
  // qualified by valid and by the port's members count. ONE EVENT, THREE
  // CONSUMERS (ground rule 7): the ROB's single-shot rob_bsy clear, this
  // table's clear, and the vector wakeup network -- same group-done, same
  // cycle. No per-PRN completion port and no per-entry counter exists on
  // this path.
  private def clearMaskFor(w: Int): UInt = {
    val prns  = wakeupMemberPrns(w)
    val cnt   = wakeupMemberCount(w)
    val valid = io.wakeups(w).valid
    prns.zipWithIndex.map { case (prn, j) =>
      UIntToOH(prn, numPregs) & Fill(numPregs, valid && (j.U < cnt))
    }.reduce(_ | _)
  }
  val clearMaskOR = (0 until numWbPorts).map(clearMaskFor).reduce(_ | _)

  // =========================================================================
  // ---- 4. Next state, and set beats clear ----
  // =========================================================================

  // SET WINS over CLEAR in the same cycle: a set is a PRN acquiring a NEW
  // producer this cycle, and a clear from its previous owner winning would
  // bring the new group up ready with nothing written.
  //
  // No RegNext on the wakeup ports, no speculative_mask/child_rebusys
  // cancellation, and consequently no rebusy term here at all -- vector
  // operands are never woken speculatively (VecGroupReady owns that policy).
  //
  // `busy_table_clr` also serves as the BYPASSED read source for part 5/5b:
  // a member whose PRN matches a group-done firing this cycle reads READY
  // through this same expression, with no separate Mux1H bypass needed.
  val busy_table_clr  = busy_table & ~clearMaskOR
  val busy_table_next = busy_table_clr | setMaskOR
  busy_table := busy_table_next

  io.debug.busytable := busy_table

  // =========================================================================
  // ---- 5 / 5b. Source reads, per-operand aggregation, and the per-member
  //              export (D6 / seam review A2) ----
  // =========================================================================
  //
  //@req-spec-rename.h12
  //@req-spec-rename.h13
  //@req-spec-rename.h14
  // (part 7) On the VL instance (maxGroupSize = 1, wakeupKind = "ready_bit"),
  // this same module IS the VL busy table: one bit per VL PRN, and every
  // reduction below degenerates to the single pvl read -- no VL-specific
  // per-member code path.
  //@req-spec-rename.g12
  //@req-spec-rename.g26
  // Each rename lane's busy read on the VL instance is `pvl` ONLY, read from
  // the VL busy table's own state -- never from pvs1/pvs2/pvs3/pvm (those
  // fields belong to the vector instance's read only), and never joined here
  // with the vector instance's response: VecRenameSpace does that join.
  for (i <- 0 until plWidth) {
    val uop  = io.ren_uops(i)
    val resp = io.busy_resps(i)

    if (vectorInstance) {
      val emul = uop.v_emul.get

      //@req-spec-rename.g11
      // Per-member busy, bypassed against this cycle's clears, masking out
      // members at or beyond v_emul (absent members neither block nor
      // falsely ready the operand). Computed ONCE per source and reused for
      // both the aggregate below and the per-member export (part 5b) -- "no
      // second read of busy_table".
      def memberBits(vec: Seq[UInt]): Seq[Bool] =
        (0 until maxGroupSize).map(j => busy_table_clr(vec(j)) && (j.U < emul))

      val pvs1Bits  = memberBits(uop.pvs1.get)
      val pvs2Bits  = memberBits(uop.pvs2.get)
      val pvs3Bits  = memberBits(uop.pvs3.get)
      //@req-spec-core.h3
      //@req-spec-rename.e7
      //@req-spec-rename.e12
      //@req-spec-vrf.d6
      // (part 6) pvtmp read+aggregated alongside the encoded sources, same
      // shape as pvs1/pvs2/pvs3 -- an ordinary VRF group with a real busy
      // lifetime, cleared indistinguishably from any other group-done.
      val pvtmpBits = memberBits(uop.pvtmp.get)
      // pvm is a single register, never a group: exactly one read.
      val pvmBit    = busy_table_clr(uop.pvm.get)

      //@req-spec-rename.g11
      // Aggregate into ONE GROUP-READY BIT per operand: OR of the masked
      // per-member busy bits (equivalently, ready only when every member is
      // ready).
      resp.pvs1_busy.get  := pvs1Bits.reduce(_ || _)
      resp.pvs2_busy.get  := pvs2Bits.reduce(_ || _)
      resp.pvs3_busy.get  := pvs3Bits.reduce(_ || _)
      resp.pvtmp_busy.get := pvtmpBits.reduce(_ || _)
      resp.pvm_busy.get   := pvmBit

      if (exportMemberRdy) {
        //@req-spec-vrf.d6
        // stale_pvdest: read PER MEMBER, exported PER MEMBER, never
        // aggregated (part 10). A single aggregate bit cannot express
        // "waiting on producer 3 of 8" when stale_pvdest's members come
        // from different producers -- the common case at LMUL > 1.
        val pvoldBits = memberBits(uop.stale_pvdest.get)
        val mresp = io.member_busy_resps.get(i)
        for (j <- 0 until maxGroupSize) {
          //@req-spec-rename.g11
          // Slots at or beyond v_emul are driven CLEAR (memberBits already
          // masks by j < emul) -- fail-safe direction: a stale BUSY bit in
          // an unused slot could hang a matcher that forgot to mask.
          mresp.pvs1_busy(j)  := pvs1Bits(j)
          mresp.pvs2_busy(j)  := pvs2Bits(j)
          mresp.pvs3_busy(j)  := pvs3Bits(j)
          mresp.pvtmp_busy(j) := pvtmpBits(j)
          //@req-spec-vrf.d6
          mresp.pvold_busy(j) := pvoldBits(j)
        }
        //@req-spec-rename.g11
        mresp.pvm_busy := pvmBit
      }

      //@req-spec-rename.g11
      // (part 9) guarded trace: "read" with the aggregated group-ready bits.
      VecTrace.trace("VecBusyTable", "read", uop, Seq(
        ("pvs1_busy",  resp.pvs1_busy.get),
        ("pvs2_busy",  resp.pvs2_busy.get),
        ("pvs3_busy",  resp.pvs3_busy.get),
        ("pvtmp_busy", resp.pvtmp_busy.get),
        ("pvm_busy",   resp.pvm_busy.get)))
    } else {
      //@req-spec-rename.h12
      //@req-spec-rename.h13
      //@req-spec-rename.h14
      //@req-spec-rename.g12
      //@req-spec-rename.g26
      // VL instance: the single pvl read, bypassed against this cycle's
      // clears exactly like the vector instance's member reads.
      resp.pvl_busy.get := busy_table_clr(uop.pvl.get)

      // (part 9) guarded trace: "read" with the aggregated (here, singular)
      // group-ready bit.
      VecTrace.trace("VecBusyTable", "read", uop, Seq(("pvl_busy", resp.pvl_busy.get)))
    }
  }

  // (part 9) guarded trace: "set" with the lane's pvdest member list and
  // v_emul -- tracePrn reports pvdest's base member (member 0) and v_emul,
  // per its own doc comment. On the VL instance this reports whatever
  // VecRenameSpace placed at pvdest(0) for its private uop (the freshly
  // allocated pvl, per part 7) -- VecTrace has no VL-specific variant, so
  // the field name in the trace line is "pvdest" even there.
  for (i <- 0 until plWidth) {
    when (io.rebusy_reqs(i)) {
      VecTrace.tracePrn("VecBusyTable", "set", io.ren_uops(i))
    }
  }

  // (part 9) guarded trace: "clr" with the completing port's member list.
  // VecTrace gained the entry points this needed (VecTrace.nlhdl.scala's
  // `traceId`/`traceStruct` -- see that file's header, which names THIS
  // module's group-done wakeup as the motivating case for `traceId`):
  //
  //   - vector instance ("group_done"): the wakeup's payload IS a
  //     VecGroupDone, which carries a real `rob_idx` alongside the
  //     member-PRN vector -- so this rung is `traceId`, not `traceStruct`.
  //     Reporting `rob=?` here would discard exactly the cross-stage/Whisper
  //     correlation the line exists for. Extra fields name the base member
  //     and count the same way the "set" line above does (`pvdest`/`nmem`),
  //     so the two lines join by field name.
  //   - VL instance ("ready_bit"): the wakeup's payload is a BARE PRN
  //     `UInt` with no rob_idx anywhere in it -- there genuinely is no
  //     honest rob_idx to report, so this rung is `traceStruct` with a
  //     `prn` key (matching VecRegFileBank's `traceStruct` convention for a
  //     resource-scoped, not instruction-scoped, event).
  for (w <- 0 until numWbPorts) {
    when (io.wakeups(w).valid) {
      if (vectorInstance) {
        val gd = io.wakeups(w).bits.asInstanceOf[VecGroupDone]
        VecTrace.traceId("VecBusyTable", "clr", gd.rob_idx,
          Seq(("pvdest", gd.pvdest.head), ("nmem", gd.members)))
      } else {
        VecTrace.traceStruct("VecBusyTable", "clr",
          Seq(("prn", io.wakeups(w).bits.asInstanceOf[UInt])))
      }
    }
  }

  // =========================================================================
  // ---- 9. Assertions ----
  // =========================================================================

  // A group-done must never clear an already-clear bit on the vector
  // instance -- that would be a double completion for one OP.v and would
  // corrupt a later owner of the PRN.
  if (vectorInstance) {
    for (w <- 0 until numWbPorts) {
      val prns = wakeupMemberPrns(w)
      val cnt  = wakeupMemberCount(w)
      for (j <- 0 until maxGroupSize) {
        when (io.wakeups(w).valid && j.U < cnt) {
          assert(busy_table(prns(j)),
            "VecBusyTable: group-done cleared an already-clear bit (double completion)")
        }
      }
    }
  }

  // No lane may raise rebusy_reqs with v_emul === 0.
  for (i <- 0 until plWidth) {
    assert(!(io.rebusy_reqs(i) && io.ren_uops(i).v_emul.get === 0.U),
      "VecBusyTable: rebusy_reqs asserted with v_emul == 0")
  }
}

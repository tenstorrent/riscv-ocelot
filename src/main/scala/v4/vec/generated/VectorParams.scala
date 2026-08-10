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

import chisel3.util.log2Ceil

// GENERATED from src/main/nlhdl/pkg/VectorParams.nlhdl.scala. Do not hand-edit;
// regenerate via the nlhdl gen-rtl flow instead.
//
// VectorParams — the single declaration of every Caracal vector sizing
// constant. This is a `kind: package` node, not an instantiable module: it has
// no ports, no clock, no reset, and is never instantiated — other modules bind
// to it only through `depends_on:` (a compile-order edge) in hierarchy.yaml.
//
// Every value here is a Scala Int/Boolean/String resolved at elaboration, never
// a hardware signal, so that `usingRVV = false` elaborates to bit-identical
// baseline RTL (see the .nlhdl file's perf section).
//
// Governing spec anchors: frontend.rst `vector-rvv-decode` (VLEN/ELEN/SEW),
// midcore.rst `vrf-ports` and the register-file sizing section, loadstore.rst
// `ssi-queues` (queue depth as an architectural limit), cii.rst `cii-interface`
// (the frozen bus and tag widths).

/**
 * VectorParams — Caracal vector sizing constants.
 *
 * Carried in `BoomCoreParams.vector: Option[VectorParams]`. A config that does
 * not enable vectors leaves that option `None` and none of this is elaborated.
 *
 * Two of these are NOT tuning knobs: `ssiQueueEntries` bounds how many vector
 * memory ops can be in flight (capacity is reserved at dispatch against a
 * worst-case element count), and `numVecPhysRegisters` bounds how many LMUL=8
 * groups can be renamed at once. See the `require`s below and
 * [[HasVectorParams]] for the derived consequences.
 */
case class VectorParams(
  // ---- Machine sizes ----

  //@req-spec-decode.a8
  // Vector register width in bits. Default and only tested value 256. Must be
  // a power of two and a multiple of eLen; rocket-chip's HasCoreParameters
  // already requires both when useVector is set, so that is not restated here.
  vLen: Int = 256,

  //@req-spec-decode.a9
  // Maximum element width in bits, i.e. the widest single element the
  // datapath handles. Default 64. Legal values 32 and 64, again required by
  // rocket-chip's HasCoreParameters.
  eLen: Int = 64,

  // ---- Physical register files ----

  //@req-spec-vrf.a1
  //@req-spec-vrf.a4
  // Size of the vector physical register file. Default 96. The vector
  // architectural register count is fixed at 32 by RVV and is not a
  // parameter. Legal range is bounded below by the capacity requirement in
  // the `require` below (see HasVectorParams.maxRenamableGroups for what this
  // buys: 8 in-flight LMUL=8 groups at the default, 4 of those if segmented).
  numVecPhysRegisters: Int = 96,

  //@req-spec-vrf.a5
  //@req-spec-rename.h1
  //@req-spec-rename.h2
  // Size of the VL physical register file. Default 64. The VL space has
  // exactly ONE architectural register (`vl`) and is a register file in its
  // own right, separate from the integer, floating-point and vector files: it
  // is sized independently of them and shares none of their storage. Only the
  // *rename* logic is shared, through a second instance of VecRenameSpace.
  numVlPhysRegisters: Int = 64,

  // ---- Element queues (vector LSU) ----

  //@req-spec-lsu.b2
  // Depth of each strided/segmented/indexed element queue. Default 512
  // entries.
  ssiQueueEntries: Int = 512,

  // Depth of each unit-stride queue. A unit-stride access occupies exactly
  // one entry per instruction rather than one per element, so this is small.
  usQueueEntries: Int = 16,

  // Number of VLEN-wide Load-Coalescing-Buffer assembly entries. Default 8,
  // i.e. one whole LMUL=8 group in flight.
  lcbEntries: Int = 8,

  // ---- Coprocessor interface figures (DERIVED, not chosen) ----

  //@req-spec-cii.b4
  // Width of the CII transaction tag. Default 4, giving 16 tags in flight.
  // Not a free choice: must equal CII_TAG_W in tt_cii_caracal_pkg.svh, the
  // frozen contract both sides bind to.
  ciiTagBits: Int = 4,

  //@req-spec-cii.a19
  // Largest number of registers in one LMUL/EMUL group. Fixed at 8, matching
  // MAX_MEMBERS in tt_cii_caracal_pkg.svh. Exposed as a parameter only so
  // every module reads one name instead of writing 8.
  maxMembers: Int = 8,

  // Number of hintable coprocessor source slots. Default 4; not a free choice
  // -- must equal CII_NUM_SRC_SLOTS in tt_cii_caracal_pkg.svh, the frozen
  // contract both sides bind to. Sizes CiiSrcReq.src_reuse_hint (one bit per
  // slot, so FOUR bits, not three) and, via log2Ceil(ciiNumSrcSlots + 1),
  // CiiSrcReq.op_id, whose extra encoding is the unused-lane case.
  //
  // The other three localparams VecBundles' spec names are already mirrored:
  // CII_TAG_W by ciiTagBits, CII_VL_W by the derived vecVLSz, CII_MEMBER_W by
  // log2Ceil(maxMembers). This one had no mirror, which is why it exists.
  ciiNumSrcSlots: Int = 4,

  // ---- Per-tier width knobs ----

  // Either "single" or "dual-dynamic". Selects how many D$ request lanes
  // VecDcacheArbiter drives per cycle. Default "single".
  dcacheArbiterMode: String = "single",

  // Number of grants each vector issue queue may make per cycle. Default 1;
  // the widest tier raises it to 2.
  vecIssueGrantWidth: Int = 1,

  // Number of slots in each of the three vector issue queues.
  vecIssueEntries: Int = 16,

  // ---- Load reservation quantum (decision D9/D10) ----

  // How many destination MEMBERS' worth of element-queue capacity a LOAD
  // reserves at dispatch: the reservation is
  // min(worstCase, ldResvMembers * vLen/eew) entries. Default 4.
  //
  // STORES are unaffected and still reserve the full worst case — the
  // four-step deadlock argument in VecQueueReservation depends on it. Loads
  // may under-reserve safely because they reserve for SQUASHABILITY, not
  // deadlock avoidance (spec-lsu.b10: a load completes out of the LCB without
  // gating on commit), and because VecElemQueue does WITHIN-REGION circular
  // reuse and never extends past its tail — so an older load's region sits
  // ahead, drains first and refills into its own region, and no younger
  // reservation can block it.
  //
  // The quantum is EEW-RELATIVE, not a flat entry count, because the agen produces one
  // element per cycle while the drain consumes up to `lsuWidth` per cycle: too small a
  // reservation lets the agen STARVE the drain and undercut target P2.
  // ===> DO NOT SET IT TO 8 OR MORE. worstCase = EMUL * vLen/eew and EMUL <= 8 always,
  //      so `min` would always select worstCase, spec-lsu.b11's streaming precondition
  //      would be unreachable, and the streaming path would be DEAD CODE in the most
  //      instantiated leaf in the subtree. At 512 entries / EMUL=8 / SEW=8:
  //      2 -> 8 loads in flight, 4 -> 4 loads, 8 -> 2 loads (i.e. no streaming at all).
  //
  // DELEGATED (A2): `require(ldResvMembers * vLen/eew_min >= lsuWidth * 2)` is
  // stated here but CANNOT be checked here — `lsuWidth` is a BoomCoreParams
  // quantity and this node has no dependencies. BoomCoreParams owns the
  // check; see its logic section. The obligation is unchanged, only its
  // location.
  ldResvMembers: Int = 4,

  // ---- Port counts named by requirements but previously declared nowhere ----

  // Group-done producer count: the LCB, VecCiiComplete and VecGroupCopy. Named
  // by issue.g33 and rename.g25, and until now defaulted independently by two
  // nodes.
  numVecWbPorts: Int = 3,

  // ROB busy-clear lane count, one per producer, never arbitrated: a lost
  // clear is unrecoverable and the ROB entry never retires.
  numVecClrPorts: Int = 3

  // `numVlWakeupPorts` IS NOT A FIELD HERE. It is `aluWidth + 1` — a per-ALU
  // vset writeback plus the vleff trim (decision D8: replicate rather than
  // arbitrate) — and `aluWidth` is a BoomCoreParams-level quantity this node
  // cannot see. It is DERIVED on the BoomCoreParams trait alongside the other
  // re-exports, not passed in: declaring it here as a mandatory field would
  // make `VectorParams()` — the default instance BoomCoreParams constructs
  // when `enableVector` is true and `vector` is `None` — uncompilable, and a
  // tier-independent default would be a lie for every tier but one.
)
{
  // ---- Elaboration-time requires that depend only on this class's own fields ----

  // Require at least one full LMUL=8 group can ever be renamed; without it the
  // machine cannot make forward progress on a wide group and would deadlock at
  // rename rather than stall. (Legal-range floor referenced by spec-vrf.a4.)
  require(numVecPhysRegisters >= 32 + maxMembers,
    s"numVecPhysRegisters ($numVecPhysRegisters) must be >= 32 + maxMembers ($maxMembers): " +
    "otherwise no full LMUL=8 group can ever be renamed and the machine deadlocks at rename")

  // DELEGATED (A2): `require(numVlPhysRegisters >= 1 + coreWidth)` — a full
  // dispatch group of VL producers must be able to allocate, plus the one
  // committed pointer — is stated here but CANNOT be checked here:
  // `coreWidth` is a BoomCoreParams quantity and this node has no
  // dependencies. BoomCoreParams owns the check; see its logic section. The
  // obligation is unchanged, only its location.

  //@req-spec-lsu.b16
  //@req-spec-lsu.b17
  // Every element-granular queue must hold at least `vLen` elements: at SEW=8,
  // LMUL=8 a single OP.v has `vLen` active elements, so a shallower queue
  // could not hold one instruction's reservation and the oldest store could
  // never complete — deadlock, not stall. Checked at elaboration (a static
  // property of the configuration) against `ssiQueueEntries` specifically, per
  // spec-lsu.b17; `usQueueEntries` (one entry per instruction, not per
  // element) and `lcbEntries` (one per LMUL=8 group) are out of this check's
  // scope by construction.
  require(ssiQueueEntries >= vLen,
    s"ssiQueueEntries ($ssiQueueEntries) must be >= vLen ($vLen): every element-granular " +
    "queue must hold at least vLen elements, or the machine can deadlock on a wide masked store")

  // Fail elaboration on anything but the two known arbiter modes rather than
  // defaulting silently — a mistyped mode that fell back to "single" would
  // look like a performance bug much later.
  require(dcacheArbiterMode == "single" || dcacheArbiterMode == "dual-dynamic",
    s"""dcacheArbiterMode ("$dcacheArbiterMode") must be "single" or "dual-dynamic"""")
}

/**
 * HasVectorParams — derived values built from a [[VectorParams]] instance.
 *
 * No hardware and no state; every member here is an elaboration-time Int
 * computed from `vectorParams`'s own fields. `vectorParams` is left abstract
 * (rather than resolved via e.g. `HasBoomCoreParameters.boomParams.vector`) so
 * this file keeps the zero-dependency contract stated in the .nlhdl source:
 * whichever concrete trait/module mixes this in supplies the instance.
 */
trait HasVectorParams
{
  val vectorParams: VectorParams
  import vectorParams._

  // ---- Derived widths ----

  // 7 bits at the default 96.
  lazy val vecPregSz: Int = log2Ceil(numVecPhysRegisters)

  // 6 bits at the default 64.
  lazy val vlPregSz: Int = log2Ceil(numVlPhysRegisters)

  lazy val elenBytes: Int = eLen / 8

  //@req-spec-decode.a10
  // The supported SEW values are 8, 16, 32 and 64 — every power of two from 8
  // up to eLen. There is no separate field for this: the set is `8, 16, ...
  // eLen` by construction, and a vtype naming an SEW above eLen is illegal
  // rather than unsupported (VtypeTable owns that check).

  // maxVecVL is the largest architecturally reachable VL, which is VLMAX at
  // the widest LMUL and narrowest SEW: VLEN * LMUL / SEW with LMUL = maxMembers
  // and SEW = 8, i.e. vLen * maxMembers / 8 — which reduces to vLen, so 256
  // elements at the defaults.
  //
  // ===> DO NOT WRITE `maxVecVL = vLen / 8`. That is VLMAX for LMUL=1 only (32
  // elements, 6 bits) and it SILENTLY TRUNCATES: a real VL of 256 at LMUL=8,
  // SEW=8 wraps to 0 in a 6-bit field. This is the same failure mode as the M1
  // AVL-truncation bug that ALUUnit's delta warns about — a value that should
  // saturate instead wraps — and it would corrupt every consumer of VL at once:
  // the VL register file entry width, the element cursors, the reservation
  // sizing, and the 9-bit `vl` field of the CII issue packet.
  // The spec is explicit and independently confirms 9: midcore.rst and
  // frontend.rst both describe the VL register file as "64 entries of ~9 bits".
  // Require `vecVLSz >= log2Ceil(vLen * maxMembers / 8 + 1)` so a future edit
  // that narrows it fails the build rather than truncating at run time.
  lazy val maxVecVL: Int = vLen * maxMembers / 8

  // vecVLSz is the width needed to hold a VL value: log2Ceil(maxVecVL) + 1 = 9
  // bits at the defaults.
  // The truncation check lives INSIDE the lazy val rather than as a bare
  // `require` in the trait body: a statement in the body executes during trait
  // initialization, before a subclass assigns the abstract `vectorParams`, so it
  // dereferenced `vLen` on a null and elaboration died with an NPE. Folding it
  // into the value it guards keeps the check (first access of `vecVLSz` runs it)
  // and defers it past construction. See the nlhdl logic section's `// ===>`
  // note on why every member here is lazy.
  lazy val vecVLSz: Int = {
    val w = log2Ceil(maxVecVL) + 1
    require(w >= log2Ceil(vLen * maxMembers / 8 + 1),
      s"vecVLSz ($w) is narrower than the architectural VL width " +
      s"(log2Ceil(vLen * maxMembers / 8 + 1) = ${log2Ceil(vLen * maxMembers / 8 + 1)}); " +
      "check the maxVecVL derivation for a truncation bug")
    w
  }

  //@req-spec-cii.a16
  // The CII operand and result buses are vLen bits wide — 256 by default —
  // for both the Src-Data and Writeback payloads. Derived from vLen rather
  // than hard-coded: the frozen SV package parameterizes its payloads on VLEN
  // too, and a hard-coded 256 on the Chisel side would silently disagree the
  // moment either changes.
  lazy val ciiSrcDataBits: Int = vLen
  lazy val ciiWritebackBits: Int = vLen

  // ---- Vector PRN capacity: what numVecPhysRegisters actually buys ----

  //@req-spec-vrf.b2
  //@req-spec-vrf.b3
  // The committed rename map table maps all 32 architectural vector registers
  // at all times, whatever the current LMUL — a mapping is a mapping
  // regardless of how many registers the current vtype groups together. So 32
  // of the numVecPhysRegisters PRNs are permanently committed architectural
  // state and are never available for in-flight renaming. Only
  // numVecPhysRegisters - 32 are.
  //
  //@req-spec-vrf.b4
  // An LMUL=8 OP.v renames its whole destination group atomically and
  // therefore takes 8 PRNs at once, never a partial group.
  //
  //@req-spec-vrf.b6
  // A segmented (shared) op takes TWO groups — pvdest plus the pvtmp
  // rendezvous group — so 16 PRNs at LMUL=8.
  //
  // At 128 PRNs these were 12 and 6. The 96-PRN choice trades in-flight groups
  // for area (the port count already dominates the storage term), so the
  // free-list stall rate is the number to watch in the LS regression.
  lazy val maxRenamableGroups: Int = (numVecPhysRegisters - 32) / maxMembers
  lazy val maxRenamableSegGroups: Int = maxRenamableGroups / 2

  // ---- Element-queue depth is an architectural limit ----

  // maxInflightWorstCaseStores is the machine's worst-case vector-store
  // memory-level parallelism, because capacity is reserved at dispatch against
  // the worst-case active element count: 2 at the defaults (512 / 256).
  // Typical VL reserves far less and gets correspondingly more overlap.
  lazy val maxInflightWorstCaseStores: Int = ssiQueueEntries / vLen
}

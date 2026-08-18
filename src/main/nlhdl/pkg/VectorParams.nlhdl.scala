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

/*
  VectorParams — the single declaration of every Caracal vector sizing constant.
*/

  hierarchy.yaml: kind: package, mode: new,
  output src/main/scala/v4/vec/generated/VectorParams.scala,
  package boom.v4.vec.generated.

  PACKAGE NODE CONVENTION. This is a `kind: package`, not an instantiable
  module: it emits a Scala `case class VectorParams` plus a `HasVectorParams`
  trait of derived values, and has no I/O whatsoever. The sections below are
  therefore read as: the parameters section = the case-class fields a config
  author sets, the ports section = nothing (stated explicitly, not omitted), and
  the logic section = the derived values and the elaboration-time requires.
  (Section names are spelled in words here on purpose: writing the delimiter
  token itself inside a comment would give a parser a second, spurious marker.)

  Governing spec anchors: frontend.rst `vector-rvv-decode` (VLEN/ELEN/SEW),
  midcore.rst `vrf-ports` and the register-file sizing section,
  loadstore.rst `ssi-queues` (queue depth as an architectural limit),
  cii.rst `cii-interface` (the frozen bus and tag widths).

  ===> READ THIS BEFORE CHANGING A DEFAULT. Two of the numbers below are NOT
       tuning knobs. The element-queue depth bounds how many vector memory ops
       can be in flight, because capacity is reserved at dispatch against a
       worst-case element count; and the vector PRN count bounds how many
       LMUL=8 groups can be renamed at once. Changing either changes an
       architectural limit, so each is documented with its derived consequence
       rather than just its value.

<|begin_module|>

  <|begin_parameters|>
  Every field below is a constructor parameter of `case class VectorParams`,
  carried in `BoomCoreParams.vector: Option[VectorParams]`. A config that does
  not enable vectors leaves that option `None` and none of this is elaborated.

  ---- Machine sizes ----

  //@req-spec-decode.a8
  `vLen` is the vector register width in bits. Default and only tested value
  256. Must be a power of two and a multiple of `eLen`; rocket-chip's
  `HasCoreParameters` already requires both when `useVector` is set, so do not
  restate those requires here.

  //@req-spec-decode.a9
  `eLen` is the maximum element width in bits, i.e. the widest single element
  the datapath handles. Default 64. Legal values 32 and 64, again required by
  rocket-chip.

  //@req-spec-decode.a10
  The supported SEW values are 8, 16, 32 and 64 — that is every power of two
  from 8 up to `eLen`. There is no separate parameter for this: the set is
  `8, 16, ... eLen` by construction, and a `vtype` naming an SEW above `eLen`
  is illegal rather than unsupported (see VtypeTable, which owns that check).

  ---- Physical register files ----

  //@req-spec-vrf.a1
  //@req-spec-vrf.a4
  `numVecPhysRegisters` is the size of the vector physical register file.
  Default 128. The vector architectural register count is fixed at 32 by RVV and
  is not a parameter. Legal range is bounded below by the capacity requirement
  in the logic section AND by VecFreeList's own require, which is the binding one
  because it alone can see `coreWidth`.

  //@req-spec-vrf.a5
  //@req-spec-rename.h1
  //@req-spec-rename.h2
  `numVlPhysRegisters` is the size of the VL physical register file. Default 64.
  The VL space has exactly ONE architectural register — `vl` — and is a
  register file in its own right, separate from the integer, floating-point and
  vector files. It is sized independently of them and shares none of their
  storage; only the *rename* logic is shared, through a second instance of
  VecRenameSpace.

  ---- Element queues (vector LSU) ----

  //@req-spec-lsu.b2
  `ssiQueueEntries` is the depth of each strided/segmented/indexed element
  queue. Default 512 entries.

  `usQueueEntries` is the depth of each unit-stride queue. A unit-stride access
  occupies exactly one entry per instruction rather than one per element, so
  this is small; default 16.

  `lcbEntries` is the number of VLEN-wide Load-Coalescing-Buffer assembly
  entries. Default 8, i.e. one whole LMUL=8 group in flight.

  `ldRespTags` is how many vector load beats may be outstanding at once, each
  holding one entry of the tag-keyed response-alignment table in `VecLsu` and one
  value of `uop.v_mem_tag`. Default 8. A beat cannot fire without a free tag, so
  this bounds load-beat concurrency; it must be at least `lsuWidth`, since every
  lane reserves its own tag from the busy register in the same cycle.

  ---- Coprocessor interface figures (DERIVED, not chosen) ----

  //@req-spec-cii.b4
  `ciiTagBits` is the width of the CII transaction tag. Default 4, giving 16
  tags in flight. This is not a free choice: it must equal `CII_TAG_W` in
  `tt_cii_caracal_pkg.svh`, which is the frozen contract both sides bind to.

  //@req-spec-cii.a19
  `maxMembers` is the largest number of registers in one LMUL/EMUL group.
  Fixed at 8, matching `MAX_MEMBERS` in the same package. It is exposed as a
  parameter only so that every module reads one name instead of writing 8.

  ADDED AT A2. VecBundles' spec requires every CII width to derive from a
  mirror of a tt_cii_caracal_pkg.svh localparam, naming four: CII_TAG_W,
  CII_NUM_SRC_SLOTS, CII_VL_W and CII_MEMBER_W. Three were already covered
  — CII_TAG_W by `ciiTagBits`, CII_VL_W by the derived `vecVLSz`, and
  CII_MEMBER_W by `log2Ceil(maxMembers)`. CII_NUM_SRC_SLOTS had no mirror
  at all, so VecBundles had nowhere to read it from and pinned a bare
  literal instead. That is the gap this field closes.
  `ciiNumSrcSlots` is the number of hintable coprocessor source slots. Default
  4, and again not a free choice: it must equal `CII_NUM_SRC_SLOTS` in the
  frozen package. It sizes the `src_reuse_hint` bit-per-slot field (which is
  therefore FOUR bits, not three) and, via `log2Ceil(ciiNumSrcSlots + 1)`, the
  `op_id` field whose extra encoding is the unused-lane case.

  ---- Per-tier width knobs ----

  `dcacheArbiterMode` is a string, either "single" or "dual-dynamic". It selects
  how many D$ request lanes VecDcacheArbiter drives per cycle. Default "single".

  `vecIssueGrantWidth` is the number of grants each vector issue queue may make
  per cycle. Default 1; the widest tier raises it to 2.

  `vecIssueEntries` is the number of slots in each of the three vector issue
  queues. Default 16.

  ---- Load reservation quantum (decision D9/D10) ----

  `ldResvMembers` is how many destination MEMBERS' worth of element-queue capacity a
  LOAD reserves at dispatch: the reservation is
  `min(worstCase, ldResvMembers * vLen/eew)` entries. **Default 4.**

  STORES are unaffected and still reserve the full worst case — the four-step deadlock
  argument in VecQueueReservation depends on it. Loads may under-reserve safely because
  they reserve for SQUASHABILITY, not deadlock avoidance (spec-lsu.b10: a load completes
  out of the LCB without gating on commit), and because VecElemQueue does WITHIN-REGION
  circular reuse and never extends past its tail — so an older load's region sits ahead,
  drains first and refills into its own region, and no younger reservation can block it.

  The quantum is EEW-RELATIVE, not a flat entry count, because the agen produces one
  element per cycle while the drain consumes up to `lsuWidth` per cycle: too small a
  reservation lets the agen STARVE the drain and undercut target P2.
  ===> DO NOT SET IT TO 8 OR MORE. worstCase = EMUL * vLen/eew and EMUL <= 8 always,
       so `min` would always select worstCase, spec-lsu.b11's streaming precondition
       would be unreachable, and the streaming path would be DEAD CODE in the most
       instantiated leaf in the subtree. At 512 entries / EMUL=8 / SEW=8:
       2 -> 8 loads in flight, 4 -> 4 loads, 8 -> 2 loads (i.e. no streaming at all).
  DELEGATED (A2): `require(ldResvMembers * vLen/eew_min >= lsuWidth * 2)` is stated
  here but CANNOT be checked here — `lsuWidth` is a BoomCoreParams quantity and this
  node has no dependencies. BoomCoreParams owns the check; see its logic section.
  The obligation is unchanged, only its location.

  ---- Port counts named by requirements but previously declared nowhere ----

  `numVecWbPorts` (default 3) is the group-done producer count — the LCB,
  VecCiiComplete and VecGroupCopy. Named by `issue.g33` and `rename.g25`, and until now
  defaulted independently by two nodes.
  `numVecClrPorts` (default 3) is the ROB busy-clear lane count, one per producer,
  never arbitrated: a lost clear is unrecoverable and the ROB entry never retires.

  ===> `numVlWakeupPorts` IS NOT A FIELD OF THIS CASE CLASS. It is `aluWidth + 1` — a
       per-ALU vset writeback plus the vleff trim (decision D8, replicate rather than
       arbitrate) — and `aluWidth` is a BoomCoreParams quantity this node cannot see.
       It is DERIVED on the BoomCoreParams trait alongside the other re-exports, not
       passed in. Declaring it here as a mandatory field would make `VectorParams()`
       — the default instance BoomCoreParams is specified to construct when
       `enableVector` is true and `vector` is `None` — uncompilable, and giving it a
       tier-independent default would be a lie for every tier but one.

  ===> EVERY FIELD OF THIS CASE CLASS MUST HAVE A DEFAULT, for that same reason.
  <|end_parameters|>

  <|begin_ports|>
  None. VectorParams is a declaration unit and has no I/O, no clock and no
  reset. It is never instantiated — modules bind to it through `depends_on:` in
  hierarchy.yaml, which is a compile-order edge, not an instance edge.
  <|end_ports|>

  <|begin_logic|>
  This section declares derived values and elaboration-time checks. There is no
  hardware and no state.

  ===> EVERY DERIVED VALUE IN `HasVectorParams` MUST BE A `lazy val`, NOT A `val`.
       The trait declares `vectorParams` ABSTRACT and its consumers supply it —
       `HasBoomCoreParameters` does so with an anonymous instance,
       `new HasVectorParams { val vectorParams = vp }` (`parameters.scala:346`).
       Scala runs a trait's own initializers BEFORE a subclass assigns its `val`s,
       so an eager `val vecPregSz = log2Ceil(numVecPhysRegisters)` dereferences
       `vectorParams` while it is still `null` and elaboration dies with:

         java.lang.NullPointerException: Cannot invoke
           "VectorParams.numVecPhysRegisters()" because the return value of
           "HasVectorParams.vectorParams()" is null

       `lazy val` defers each computation to first access, which is after the
       subclass is constructed. `def` would work too but recomputes; these feed
       hardware widths and are read many times.

       ===> NEITHER GATE (a) NOR GATE (f) CAN CATCH THIS, WHICH IS WHY IT SAT
       UNDETECTED FROM PHASE A THROUGH PHASE C. It is not a compile error —
       the types are fine — and a `usingRVV = false` build never constructs
       `HasVectorParams` at all, so the vectors-off gate cannot reach it
       either. It surfaced only when a VECTOR config was first elaborated
       end-to-end (the `MegaBoomV4VectorConfig` cosim pipeclean, 2026-08-10),
       and it fired BEFORE the known D2 dispatcher `require`, masking it.
       The abstract `val vectorParams: VectorParams` itself stays a plain
       `val` — it is the thing being supplied, not a derived value.

  ===> AND NO ELABORATION CHECK MAY SIT AS A BARE STATEMENT IN THE TRAIT BODY,
       for the same reason: a bare `require(...)` executes during trait
       initialization and hits the same null. Fold each check INTO the `lazy val`
       whose invariant it guards, as a block that computes the value, requires on
       it, then yields it. The check still runs — on first access of that value —
       and it runs after construction. A check hoisted out of the value it
       protects looks tidier and does not work.

  ---- Derived widths ----

  `vecPregSz` is `log2Ceil(numVecPhysRegisters)`, 7 bits at the default 96.
  `vlPregSz` is `log2Ceil(numVlPhysRegisters)`, 6 bits at the default 64.
  `elenBytes` is `eLen / 8`.

  `maxVecVL` is the largest architecturally reachable VL, which is VLMAX at the
  widest LMUL and narrowest SEW: `VLEN * LMUL / SEW` with `LMUL = maxMembers`
  and `SEW = 8`, i.e. `vLen * maxMembers / 8` — which reduces to `vLen`, so
  **256 elements** at the defaults. `vecVLSz` is the width needed to hold a VL
  value, `log2Ceil(maxVecVL) + 1` = **9 bits**.

  ===> DO NOT WRITE `maxVecVL = vLen / 8`. That is VLMAX for LMUL=1 only (32
  elements, 6 bits) and it SILENTLY TRUNCATES: a real VL of 256 at LMUL=8,
  SEW=8 wraps to 0 in a 6-bit field. This is the same failure mode as the M1
  AVL-truncation bug that ALUUnit's delta warns about — a value that should
  saturate instead wraps — and it would corrupt every consumer of VL at once:
  the VL register file entry width, the element cursors, the reservation
  sizing, and the 9-bit `vl` field of the CII issue packet.
  The spec is explicit and independently confirms 9: midcore.rst and
  frontend.rst both describe the VL register file as "64 entries of ~9 bits".
  Require `vecVLSz >= log2Ceil(vLen * maxMembers / 8 + 1)` so a future edit
  that narrows it fails the build rather than truncating at run time.

  //@req-spec-cii.a16
  The CII operand and result buses are `vLen` bits wide — 256 by default — for
  both the Src-Data and Writeback payloads. Derive both widths from `vLen`
  here rather than writing 256 anywhere: the frozen SV package parameterizes
  its payloads on VLEN too, and a hard-coded 256 on the Chisel side would
  silently disagree the moment either changes.

  ---- Vector PRN capacity: what 128 actually buys ----

  //@req-spec-vrf.b3
  //@req-spec-vrf.b2
  The committed rename map table maps all 32 architectural vector registers at
  all times, whatever the current LMUL — a mapping is a mapping regardless of
  how many registers the current `vtype` groups together. So 32 of the 128 PRNs
  are permanently committed architectural state and are never available for
  in-flight renaming. Only `numVecPhysRegisters - 32` are.

  ===> AND NOT EVEN ALL OF THOSE. VecFreeList's pre-selection stage parks one PRN
  per port in a holding register, `allocWidth = coreWidth * 2 * maxMembers` of them,
  refillable only from `free_list`. Those are unavailable at rest too, so the pool
  a running machine can actually reach is `numVecPhysRegisters - 32 - allocWidth`.
  This term is invisible from here — `coreWidth` is a BoomCoreParams quantity — which
  is why the binding require lives in VecFreeList and this file's is the weaker
  necessary condition, not the sufficient one. Do not size this parameter from the
  `- 32` figure alone.

  //@req-spec-vrf.b4
  An LMUL=8 OP.v renames its whole destination group atomically and therefore
  takes 8 PRNs at once, never a partial group.

  //@req-spec-vrf.b6
  A segmented (shared) op takes TWO groups — `pvdest` plus the `pvtmp`
  rendezvous group — so 16 PRNs at LMUL=8.

  Declare these consequences as named derived values so the limit is visible
  where the parameter is read, not just in a comment:
  `maxRenamableGroups = (numVecPhysRegisters - 32) / maxMembers`, which is 12 at
  the default, and `maxRenamableSegGroups = maxRenamableGroups / 2`, which is 6.
  Both overstate the reachable figure by `allocWidth / maxMembers` for the reason
  above; they bound the file, not the free list.

  The default was 96 until E7, which trades in-flight groups for area (the port
  count already dominates the storage term). That is a sound trade in isolation and
  it was ALSO the deadlock: at `coreWidth = 4` it left `96 - 32 - 64 = 0` reachable
  PRNs, so the first vector load hung the machine at rename. Restored to 128, which
  leaves 32. Free-list stall rate remains the number to watch in the LS regression;
  it is now a throughput signal rather than a liveness one.

  Require `numVecPhysRegisters >= 32 + maxMembers` so at least one full LMUL=8
  group can ever be renamed; without it the machine cannot make forward
  progress on a wide group and would deadlock at rename rather than stall.

  DELEGATED (A2): `require(numVlPhysRegisters >= 1 + coreWidth)` — a full dispatch
  group of VL producers must be able to allocate, plus the one committed pointer.
  `coreWidth` is a BoomCoreParams quantity this node cannot see, so BoomCoreParams
  owns the check. The obligation is unchanged, only its location.

  ---- Element-queue depth is an architectural limit ----

  //@req-spec-lsu.b16
  Every element-granular queue must hold at least `vLen` elements. The bound is
  `vLen` and not `maxVecVL` because the worst case is one element per BYTE of a
  register group: at SEW=8 with LMUL=8 a single OP.v has `vLen` active
  elements. A queue shallower than that could not hold one instruction's
  reservation, so the oldest store could never complete and the machine would
  deadlock rather than merely stall.

  //@req-spec-lsu.b17
  Check that floor AT ELABORATION, with a `require` naming the offending
  parameter — this is a static property of the configuration, so it must fail
  the build rather than produce a machine that deadlocks on a wide masked
  store. Apply it to `ssiQueueEntries`.

  Then express the consequence, again as a named value rather than a comment:
  `maxInflightWorstCaseStores = ssiQueueEntries / vLen`, which is 2 at the
  defaults (512 / 256). That figure IS the machine's worst-case vector-store
  memory-level parallelism, because capacity is reserved at dispatch against
  the worst-case active element count. Typical VL reserves far less and gets
  correspondingly more overlap.

  ---- Arbiter mode ----

  Require `dcacheArbiterMode` is one of "single" or "dual-dynamic", failing
  elaboration on anything else rather than defaulting silently — a mistyped
  mode that fell back to "single" would look like a performance bug much later.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
No runtime behaviour, so no throughput or latency target. One elaboration-time
constraint does belong here: every value in this file must be a Scala `Int`,
`Boolean` or `String` resolved at elaboration, never a hardware signal. Widths
and queue depths derived from them must be constants in the emitted Verilog, so
that `usingRVV = false` elaborates to bit-identical baseline RTL.
<|end_perf|>

<|begin_dependencies|>
None. VectorParams is the root of the vector declaration graph — every other
vector node depends on it, and it depends on nothing.

It is READ BY (fan-in over `depends_on:`, i.e. the blast radius of any change
here): BoomCoreParams, BoomConfigMixins, VtypeTable, VecBundles, and every
`vec/**` module. Two of its fields are mirrors of the frozen SV contract in
`tt_cii_caracal_pkg.svh` — `ciiTagBits` = `CII_TAG_W` and `maxMembers` =
`MAX_MEMBERS` — so changing either without changing the SV breaks the CII seam
silently. Derive, do not duplicate.
<|end_dependencies|>

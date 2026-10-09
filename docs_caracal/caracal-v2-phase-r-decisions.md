# Caracal v2 — Phase R decision record

Thirteen decisions taken after Phase N (all 58 NL_HDL specs authored, `inspect-hierarchy`
clean) resolving the open rulings in `caracal-v2-seam-review.md`. Each entry gives the
decision, why, and what it obliges — including where it *weakens* a guarantee, because
several of these do.

Format note: this is ADR content in the repo's existing `docs_caracal/*.md` convention
rather than a `docs/adr/` tree, to keep one decision location rather than two.

---

## D1 — Gate (f) bit-identity is RELAXED to a bounded, enumerated exception

`ScalarOpConstants` is a bare Scala `trait` with no `Parameters` in scope, so it widens
`RT_*` 2b→3b and `IQ_SZ` 4→7 **unconditionally**; `MicroOp` and `Rob` must follow. A
vectors-off build therefore cannot be bit-identical to pre-Caracal BOOM v4.

**Decision.** Amend plan §6 gate (f) from "bit-identical to pre-Caracal v4" to
"**identical except for the enumerated encoding widths below**", and diff against a
**re-baselined reference** — a checked-in artifact with the widened encodings and no
vector logic.

The enumerated exception, and nothing else:
- `RT_FIX/RT_FLT/RT_X/RT_ZERO`: `UInt(2.W)` → `UInt(3.W)` (values 0..3 unchanged)
- `IQ_SZ`: 4 → 7, hence `MicroOp.iq_type` 4 → 7 bits
- `MicroOp.dst_rtype`, `lrs1_rtype`, `lrs2_rtype`: 2b → 3b
- `Rob`'s compact `dst_rtype` tracks `MicroOp`'s width

**Rejected:** parameterizing the encodings so a non-vector build emits 2-bit rtypes.
Cleaner in principle, but it means the `RT_*`/`IQ_*` space stops being a bare trait, and
that trait is consumed during `issueParams` construction before `Parameters` is available.

**Obligation this creates.** A new plan step, analogous to A0, must generate and check in
the re-baselined reference. Without it "identical except the enumerated widths" is
unfalsifiable, and every later step's bit-identity claim rests on it.

**Status — set up; half discharged.** Plan **step A1** now exists, and the machinery is in
`docs_caracal/v2-rebaseline/` (`regen.sh`, `gate-f-check.py`, `selftest.sh`, `README.md`).

- The reference is **two** artifacts, not one. A lone post-widening reference cannot be
  audited — it already contains the exception, so a diff against it cannot show whether the
  widening dragged anything else along. `prebaseline` (before the widening) is the anchor;
  `rebaseline` (after it, no vector logic) is what later steps diff against; the
  `prebaseline`↔`rebaseline` diff is what actually discharges D1.
- **`prebaseline` is generated and checked in** — `Small`/`Medium`/`Mega` V4, boom `2d7cf02e`,
  firtool-1.75.0, per-module hashes in `manifest/prebaseline.json`. Elaboration determinism
  was *verified* by elaborating twice to the same hash, not assumed.
- **`rebaseline` cannot be generated yet**, and `manifest/rebaseline.json` is absent by
  design: the widening currently exists only in the `edit_existing` nlhdl specs under
  `src/main/nlhdl/pkg/`, not in `src/main/scala`. It lands with step A2.
- The checker is **structural, not textual**, because widening `dst_rtype` renumbers bit
  positions in every bundle packing a `MicroOp`; a line diff reports thousands of
  consequential subscript changes and buries the one that matters. Modules mentioning neither
  `rtype` nor `iq_type` are held to strict textual equality (568 of 620 on Medium); the rest
  are checked for identical ports, instances and statement counts, with every width delta
  required to be exactly the enumerated widening on an allowlisted name. `selftest.sh` asserts
  each violation class actually fires. Sanity-checked against a real M1/M2 tree: 1,826 deltas
  classified as the exception, 22,257 flagged as violations.
- **Newly discovered, and it blocks all of Phase A:** the working tree does not compile at
  all. `generators/chipyard/.../BoomConfigs.scala` is committed against boom `9f67941b`
  (M1/M2, has `v4/vec/`) and calls `WithVector`, `boom.v4.vec.common.VectorParams`,
  `enableVectorArith`, `dcacheArbiterMode`, `WithBoomDebugHarness` — none present on
  `Caracal/addvector2`. Because `WithBoomDebugHarness` is mixed into the *plain* V4 configs
  too, **no** config elaborates, vector or not. `regen.sh` neutralizes those mixins
  temporarily (restoring via an `EXIT` trap); **A2 must fix it properly**, since A2 is what
  re-adds the Chipyard vector configs.

**Also note (A23):** a passing gate (f) does **not** prove the three new `IQ_V_*` iq_type
bits are explicitly defaulted. `DecodeUnit` does `uop := io.enq.uop` where `io.enq.uop`
comes from a bundle the frontend sets `:= DontCare`, so without six explicit defaults every
*scalar* uop carries don't-care vector routing bits into dispatch. A don't-care bit can
elaborate bit-identically and still mis-route.

---

## D2 — Vector configs use `CompactingDispatcher`; the vector queues are wired natively

A vector config **fails elaboration today**: the three `IQ_V_*` `issueParams` entries fall
through `core.scala:837-849`'s dispatch loop to `require(false)`.

**Decision.** Vector configs instantiate `CompactingDispatcher` instead of
`BasicDispatcher`, and the three vector queues are wired natively from
`dispatcher.io.dis_uops(i)`.

**Why, with the mechanism.** `BasicDispatcher` computes
`ren_readys = io.dis_uops.map(d => VecInit(d.map(_.ready)).asUInt).reduce(_&_)` — the ready
is **not** masked by `iq_type`, so every queue's ready ANDs into every lane. Wiring the
vector queues in under `BasicDispatcher` would make a full `IQ_V_LOAD` stall **pure-scalar
lanes carrying no vector uop at all**, directly threatening gate (d) and target P6.
`CompactingDispatcher` already implements the needed masking:
`rdy := ren zip uses_iq map {case (u,q) => u.ready || !q}` — "the queue is considered ready
if the uop doesn't use it."

**Rejected:** tying the vector lanes' dispatcher ready `true.B` and routing queue fullness
through the seam's single `dis_ready` bit. Works, and is smaller, but leaves a hack in place
and collapses per-lane back-pressure to one bit.

**Cost.** A `Compactor` per queue (7), and the scalar dispatch path differs in a vector
build — noted so a scalar regression is not mis-attributed.

---

## D3 — SmallBoom is REMOVED from the vector configuration matrix

Small+vector is broken two independent ways:
1. `CompactingDispatcher` cannot elaborate: `IQ_MEM` is forced to `issueWidth >= 2` by
   `require(memWidth >= 2)`, but `dispatchWidth = coreWidth = 1`, so
   `require(dispatchWidth >= issueWidth)` is `require(1 >= 2)`. Widening is blocked by
   `parameters.scala:275`, `require(dispatchWidth <= coreWidth)`.
2. `allocWidth = coreWidth*8 = 8`, but a shared `OP.v` needs `2*maxGroupSize = 16` PRNs
   all-or-nothing — a **deadlock**, not a stall.

**Decision.** Plan §5 rule 4 becomes **Medium/Large/Mega**. `WithNSmallBoomsVector` is
dropped from the `BoomConfigMixins` delta.

**This closes the `allocWidth` open ruling with no parameter change**: `coreWidth >= 2`
gives `allocWidth >= 16`, exactly meeting the shared-op requirement. The elaboration
`require(allocWidth >= 2*maxGroupSize)` stays as the guard and now documents *why* Small has
no vector config.

**Rejected:** "Small forbids segmented vector LS" (the seam review's own suggestion). It is
**architecturally invalid** — segmented load/store is base RVV 1.0 and `spec-core.a2`
requires RVV 1.0, so such a core would be non-conformant.

**Perf note to record:** at Medium, `allocWidth = 16` and one shared `OP.v` consumes the
*entire* allocation window, so nothing else in that bundle allocates that cycle. A stall,
not a deadlock.

**Re-open cost:** a future 1-wide vector core needs `allocWidth`, the dispatcher and the
rename timing spike revisited together.

---

## D4 — The store-side FP read lane is DELETED

`execution.rst:160-162` says the store DGEN reads "the FP/INT register file for scalar
source operands, **for instance vfmul.vf**". `vfmul.vf` is a vector-scalar floating-point
multiply — *arithmetic*, dispatched to the CII. **It is not a store.** No RVV store form
takes an FP scalar operand: store data is always `vs3`, and the scalar operands are `rs1`
(base) and `rs2` (stride), both integer.

**Decision.** Delete the store-side FP reader. `fp_rf_read_req` 2 → 1 (only
`VecCiiIssue`'s `.vf`). Fix `execution.rst`; narrow/retire `spec-agen.d3`.

**Guard (explicitly retained):** `spec-vrf.e4` and `spec-issue.g8/g9/g10` stay intact — the
slot still matches `.vf` on the FP wakeup network, which is how it learns the operand is
ready. Only the *store-side reader* and the claim that a bypass window exists are deleted.

**This closes two open items:**
- **A30 dissolves.** The only remaining FP reader is `VecCiiIssue`, and `IQ_V_ALU` is
  past-PNR gated — so a granted CII op is *older than the PNR* and its FP producer has
  genuinely written back. There is no fast-wakeup bypass window to close.
- **A32 is moot.** The Giga-tier FP read-port shortfall only arose from needing 2 lanes, and
  Giga is not in the vector matrix. If a Giga vector config is ever added,
  `numFrfReadPorts` must rise to 6.

---

## D5 — ONE shared back-pressure structure; ground rule 6 amended to THREE locations

Two independent problems have one cause — the operand-read→agen chain has no back-pressure:
1. `PartiallyPortedRF` denies INT reads **by index priority**
   (`ready := PopCount(io.arb_read_reqs.take(i).map(_.valid)) < numPhysicalReadPorts`), the
   vector lanes are appended **last**, and there are ~7-9 existing logical readers plus 5
   vector lanes against **5** physical ports on Medium. Denial is routine, and
   `VecScalarOperandRead` was written for "UNARBITRATED ports with no ready line".
2. An element walk takes up to `vl` cycles, and nothing stops a second grant next cycle.

**Decision.** `VecLsu` holds ONE per-LDQ/STQ-entry pending table absorbing both. The issue
grant is **accepted unconditionally** (no `busy`, no dropped grant — the row is indexed by
an LDQ/STQ entry the reservation already guaranteed, so overflow is unrepresentable). The
table presents a descriptor only when that direction's mask streamer is free **and** its
INT/VL reads have been granted.

**This preserves the downstream contract every agen was written against**: the
`VecScalarOperandRead → agen` link stays a `Valid` the agen latches unconditionally. Only
the *upstream* link becomes `Decoupled`. `int_rf_read_req` becomes per-lane `Decoupled`.

**⇒ GROUND RULE 6 IS AMENDED.** It previously enumerated *exactly two* legal homes for
in-flight vector-LSU state. It now permits **three**:
> (a) the six `VecElemQueue` instances, (b) the LCB's per-PRN assembly entries, and
> (c) `VecLsu`'s per-LDQ/STQ-entry descriptor pending table.

(c) is the same *kind* of state as (a) — per-queue-entry, capacity reserved at dispatch, no
`busy` exported, structurally un-overflowable. The rule is amended explicitly rather than
letting a table appear that the rule forbids: gate H4 reviews against this text, and a
reviewer would be right to reject it otherwise.

**Rejected:** qualifying the `FC_AGEN` grant — a `busy` reaching an issue unit under another
name, which is what H4 greps for.

---

## D6 — `stale_pvdest` readiness is gated by a 5th group matcher

`stale_pvdest` is the *previous* mapping of the destination arch vregs, so its producer is an
older instruction. Age-ordered issue grants oldest-**ready** — it does not guarantee an older
producer finished. So `IQ_V_LOAD` (LCB pre-loads from `stale_pvdest` on R2 for
`vta=0`/`vma=0`) and `IQ_V_ALU` (coprocessor may pull `STALE_VD`) can read a **busy** group.
Four nodes reported this independently; it matches the prior M2 `pvold_busy` hang.

**Decision.** Add a 5th `VecGroupReady` (`rdy_vold`) on `IQ_V_LOAD` and `IQ_V_ALU` slots plus
a 5th group in the per-member side channel. New requirement via `/spec-to-reqs` into
`spec-issue` group g. **This is a corpus gap, not a spec-to-code defect.**

**Rejected — and it is the tempting wrong answer:** a single aggregate `stale_pvdest_busy`
bit. `stale_pvdest` can span **multiple producers**: if an `LMUL=1` op wrote `v0` and a later
`LMUL=8` op renames `v0..v7`, its `stale_pvdest` is the current mappings of eight arch
vregs, installed by up to eight different instructions. An aggregate bit cannot express
"waiting on producer 3 of 8" and one group-done cannot clear it correctly. Same reason
`pvs*` needs per-member matching (`rename.g20`).

**Accepted conservatism.** The host cannot know whether the VPU will actually pull
`STALE_VD` — the coprocessor decides, and no VPU-side signal exists. So any CII op with a
vector destination waits on `stale_pvdest`.

**Cost.** +1 matcher x (16 load + 16 ALU slots), in the stage that is already the design's
#1 timing risk.

---

## D7 — `vfmv.f.s` gets a DEDICATED FP write port and wakeup slot

`ll_wbarb` is an `Arbiter` (in(0)=mem, in(1)=ifpu, in(2)=fdiv) feeding
`fregfile.io.write_ports(0)`. Adding a 4th input means the CII writeback **can be denied**,
and the CII channel has no back-pressure.

**Decision.** A dedicated FP write port + wakeup slot under `usingRVV`.

**⚠ CORRECTION to this decision's original wording.** It said "`numFrfWritePorts` 1→2".
That is **wrong**, and dangerously so. The baseline expression is
`numFrfWritePorts = fpWidth + lsuWidth` (`fp-pipeline.scala:66`) — so **2 on
Medium/Large and 3 on Mega**, never 1. The correct form is
`numFrfWritePorts = fpWidth + lsuWidth + (if (usingRVV) 1 else 0)`, i.e. **append one**,
never assign a literal. A generator that hard-coded `2` would silently **drop Mega's
second exe-unit write port**. Caught by the `FpPipeline` amendment; recorded here rather
than quietly fixed, because the wrong number was in the decision record a generator would
have been handed.

**Why.** Correctness, not cost: an arbitrated path needs a provable bound and none is
constructible — the arbiter can lose to mem *and* ifpu *and* fdiv, and consecutive
scalar-FP CII ops can produce back-to-back beats. `vfmv.f.s` is rare, which is exactly what
makes arbitration dangerous: the failure would be a once-in-a-blue-moon wrong FP register
value with no assertion. Also note anything joining `ll_wbarb` must be hardfloat-`recode`d,
while `VecCiiWriteback` emits IEEE.

**⇒ `FpPipeline`'s reject list is AMENDED.** It explicitly refused this port. This is a
deliberate widening of an `edit_existing` node's allowed change — the thing plan §10 flags
as the previous attempt's worst bug site — so it is amended in the text, not ignored. Stays
within the ~50-line budget, and is `usingRVV`-gated so it lands in D1's enumerated exception.

---

## D8 — the multi-ALU vset writeback is handled by REPLICATION

`aluWidth == coreWidth` on every tier, so with Small dropped the matrix is Medium(2)/
Large(3)/Mega(4) — `aluWidth` is **never 1**. Every config in the matrix replicates.

**Decision.** `vset_resp` becomes `Vec(aluWidth, Valid(ExeUnitResp))`,
`VlRegFile.numAluWritePorts = aluWidth`, `numVlWakeupPorts = aluWidth + 1`.
VL RF write ports: Medium `2+2+1 = 5`, Mega `4+4+1 = 9`.

**Why.** "Replicate, never arbitrate" is already `VlRegFile`'s stated discipline, for the
right reason: a single-shot VL wakeup lost to arbitration is a **permanent hang**. And the
cost is specific to this file — the VL RF is `64 x 9b`, so write ports are cheap there in a
way VRF ports emphatically are not (9 ports on 576 flops is not 12 ports on 24 kbit).

**Rejected:** routing vsets to `IQ_UNQ` (`unqWidth = 1` on every tier, so one writeback
falls out by construction, and the throughput cost is ~nil since a strip-mined iteration is
6+ instructions). It would require **`UniqueExeUnit`** to advertise the vset capability — a
unit that is not a node in the map — and stretches `spec-decode.c7` ("must execute on an
integer ALU execution unit").

---

## D9/D10 — loads UNDER-RESERVE and stream; `ldResvMembers = 4`

`spec-issue.b2`/`b3` (reserve worst case from EMUL/EEW) make `spec-lsu.b11`'s streaming
precondition unreachable, because
`worstCase = EMUL x VLEN/EEW = LMUL x VLEN/SEW = VLMAX >= VL >= active count`.

**Decision.** Keep streaming **live**. `spec-issue.b2` splits: **stores** reserve worst case
(the 4-step deadlock argument depends on it); **loads** reserve
`min(worstCase, ldResvMembers x VLEN/EEW)` with **`ldResvMembers = 4`**.

**Why loads may under-reserve safely.** Loads reserve in program order for
**squashability**, not deadlock avoidance — `spec-lsu.b10`: a load completes out of the LCB
without gating on commit. Only stores have the deadlock exposure. And `VecElemQueue` already
specified **within-region circular reuse, never an extension past tail**, which is what makes
it deadlock-free: an older load's region sits ahead of a younger one's, drains first, and
refills into its *own* region, so no younger reservation can block it. Squashability survives
because the region is still contiguous and program-ordered, just smaller.

**Why the quantum is EEW-relative and why 4.** The agen produces 1 element/cycle but the
drain consumes up to `lsuWidth`/cycle, so too small a reservation lets the agen **starve the
drain**, undercutting P2. At 512 entries / `EMUL=8` / `SEW=8`: `ldResvMembers` of 2 → 8 loads
in flight, 4 → 4 loads, 8 → 2 loads. **8 would make the mechanism dead again** (since
`EMUL <= 8` always, `min` would always pick worst case) — i.e. identical to deleting it. 4
gives 2x P3's floor with double the starvation margin.

Add an elaboration `require(ldResvMembers x VLEN/EEW_min >= lsuWidth x 2)` so a future tier
that widens `lsuWidth` without revisiting the quantum fails the build rather than starving.

---

## D11 — `MicroOp` gains `v_uses_vs1` / `v_uses_vs2` / `v_uses_vs3`

For `vadd.vx` (`vd, vs2, rs1`), `lvs1` is unencoded — so `pvs1` resolves to the current
mapping of **v0**, the mask register, which is written constantly. If v0's producer's
group-done already fired *before* the slot captured its member-ready bits, no future
group-done clears them and **the slot waits forever**. `VecIssueSlot` placed the obligation
on `VecRenameSpace`; `VecRenameSpace` reported it cannot discharge it because no
`lvs*_rtype` exists. Only decode knows the instruction format.

**Decision.** Three Bools in `MicroOp`, set by `VDecode`/`VLSDecode`, consumed by the vector
mapper (skip renaming an unencoded source, leave its busy clear) **and** by `VecIssueSlot` as
its `used` input. Mirrors `v_is_masked`, which exists for the identical reason.

**Rejected:** a reserved sentinel in `lvs*` (encodings 32-63 are free when `lregSz = 6`).
Zero new bits, but correctness would depend on every future reader remembering an implicit
convention, and the failure mode is a silent hang. Also rejected: full `lvs*_rtype` fields
(3x the cost for no extra information).

**Cost.** 3 bits in the bundle with **49 transitive dependents**; the fix touches 4 files.

---

## D12 — corpus policy: amend the `.rst` only where the text is FACTUALLY FALSE

**Decision — split by cause:**

**(1) Factually false → amend `.rst`, then `/spec-to-reqs extract` on the affected family.**
Leaving these means the next `extract` faithfully regenerates the falsehood.
`agen.d3` (`vfmul.vf` as a store); the signed-index claim; `overview.rst:158` (execute-time
mirror write); `memord.b16` (names a store-set predictor BOOM v4 lacks — what exists is
match-driven blocking at `lsu.scala:1413-1431`); `memord.a19` (a 512-entry broadcast compare,
not synthesizable); `agen.a8`/`c11` (superseded OVI modules); `decode.d8` (unsatisfiable for
`vsetvl`); `decode.c3` (impossible for `vsetivli rd != x0`); `issue.b2`/`lsu.b11` (the
load/store split from D9/D10).

**(2) True in spirit, loosely worded → KEEP the ID, annotate the reading.**
`core.f11` ("each issue slot granted once" — a vector store is granted twice, AGEN then
DGEN; the true obligation is "once per execution resource"). `agen.c10` vs `c16` (skip vs
one-at-a-time — resolved class-conditionally: skip is strided/segmented only, never indexed,
because `VecIdxGen.taken` pulses per element including masked-off ones).
Retiring a true-but-loosely-worded requirement loses coverage; annotating preserves it.

**(3) Allocation wrong, requirement fine → re-allocate via `architect`, no corpus change.**
`lsu.g1-g4/g9/g10` (`vleff` is unit-stride, so it belongs to `VecRangeAgen` not
`VecElemAgen` — an architect-pass error). `cii.f5-f8/f10` (tagged in
`VecCiiOperandServer`, mux emitted in `VecCiiTagTable`).

**Both re-allocations are DONE.** `lsu.g1-g4/g9/g10` now sit on `VecRangeAgen`;
`cii.f5-f8/f10` now sit on `VecCiiTagTable`, with the `//@req-` tags moved to the per-slot
resolutions that emit them and `VecCiiOperandServer` keeping the encoding prose untagged.
Both nlhdl files had flagged the `cii` split themselves and demanded it be closed
*explicitly* rather than by quietly re-tagging one side; those notes are rewritten to record
which option was taken. `VecCiiOperandServer` retains the OUTCOME of each slot through the
IDs that stayed (`f4`, `f9`, `f12`, `vrf.j13`/`j14`), so nothing was stranded. Validator
after the move: 0 errors, 0 warnings, 1172 live / 1075 tagged — unchanged totals, i.e. no
coverage lost and nothing double-counted.

**Pipeline order, which matters:** amend `.rst` → `/spec-to-reqs extract` → `/nlhdl
architect` re-allocate → fix tags in affected specs → **then** `gen-rtl`. Generating RTL
first would bake in requirements already known to be wrong.

---

## D13 — the INT-read retry model ACCUMULATES; the table hands off and goes quiet

D5 settled *that* a pending table absorbs INT-RF read denial, but not *who holds the
partial progress*. Two files then implemented incompatible answers:
`VecScalarOperandRead` accumulated (per-lane `rr_need`, per-lane `rr_data`, address held
from `rr_uop`), while `VecLsu` said a partial grant is "retried in full, not accumulated"
— and contradicted itself by listing "the per-lane read-grant bits" among the row's
contents. The observable symptom: `VecLsu` also said a denied row "stays presented and
re-requests next cycle", which keeps `iss.valid` high on a descriptor already latched, so
`VecScalarOperandRead`'s assertion `!(iss.valid && rr_valid && rr_need.orR)` would fire on
**every denial**.

**Decision — model (a), accumulate.** The per-lane hold lives in
`VecScalarOperandRead`; `prs1` fires and STAYS fired while only the still-outstanding
lanes re-request. `VecLsu`'s table treats presentation as a **HAND-OFF**: it drops `valid`
once a row is accepted and re-presents only after the hold clears. Its "retried in full"
rule and its per-lane row-contents mirror are both deleted — the consumer's copy is the
one the RF handshake actually advances, and two places recording one fact is how they
drift.

**Why, and this is a correctness argument rather than a performance one.** The vector
lanes are appended **LAST** in `PartiallyPortedRF`'s index priority, against ~7-9 existing
logical readers and 5 physical ports on Medium — so denial is routine. Retry-in-full
requires base AND stride to win in the **same** cycle against all of that, which under
sustained scalar pressure is a **livelock, not a slow path**; it would also re-serialize
the very read D5's 3 -> 5 seam widening existed to parallelize. Accumulation makes
progress **monotonic**: each lane fires once and stays fired, so the worst case is bounded
by the unluckiest single lane rather than by the coincidence of all of them. It is also
the idiom every scalar EU already uses — hold the address until `fire` — so it is what
the register file was built for.

**The rejected argument, answered rather than dismissed.** `VecLsu`'s objection was that
per-lane progress bits mean "a half-read descriptor exists", which is state that must be
got right on a kill. True, but it is a **kill** question, not a correctness-of-read
question, and it is answered where the state lives: `VecScalarOperandRead`'s kill clears
`rr_need` so a squashed uop stops consuming arbitration, and a register read has no side
effect, so a lane that fired for a killed descriptor has merely wasted a port cycle.

**Cost.** 2 bits plus 2 x `xLen` of holding state per direction — negligible against the
24 kbit VRF.

---

## Fact established by lookup, not decided

**RVV index offsets are UNSIGNED.** Spike's `VI_LDST_GET_INDEX` reads them as
`uint8_t`/`uint16_t`/`uint32_t`. The map and plan saying "signed" is a spec defect; since
Whisper is the cosim reference, sign-extension would diverge on any index with the top EEW
bit set. `VecIdxGen` isolated the extension into one named function, so the fix is local.

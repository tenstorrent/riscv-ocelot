# Caracal Milestone Plan v2

**Objective.** Build the complete RVV 1.0 vector datapath on BOOM v4 — decode, atomic
LMUL/EMUL group rename, VL rename, ROB integration, vector issue queues, the vector
register file, the **out-of-order vector load/store unit**, and vector arithmetic offloaded
over the TT-CII to the SV coprocessor — as **host-side Chisel only**, against an SV VPU that
is already refactored and verified.

This plan **supersedes** `caracal-milestone1-plan.md` and `caracal-milestone2-plan.md`. Both
remain in the tree as the record of the previous attempt; their *as-built* sections are
required reading (see [§2](#2-what-went-wrong-and-what-v2-does-differently)) because every
bug they document is a test case for v2.

**Two things make v2 different from "M1+M2 again":**

1. **Design-first methodology, design-wide.** Every module in the design is specified as an
   NL_HDL file under `src/main/nlhdl/`, and the **whole set** is reviewed before **any** RTL is
   generated — not per module and not per phase. RTL generation then runs phase by phase on a
   cheaper model, gated per phase. See [§4](#4-methodology-the-whole-design-in-nl_hdl-then-rtl).
2. **Performance is a gate, not a hope.** The previous attempt was functionally correct and
   architecturally serialized. v2 measures the baseline first, states numeric targets, and
   treats a missed target as a failed step. See [§6](#6-verification-gates) rule (h).

---

## 1. Reference

| What | Where |
|---|---|
| Architecture spec of record | `docs_caracal/src/*.rst` (built HTML under `docs_caracal/_build/html`) |
| Module map / design authority | `src/main/nlhdl/hierarchy.yaml` — 63 nodes, and the allocation of all 1162 live requirements |
| Requirement corpus | `src/main/nlhdl/reqs/` — `families.yaml` + 10 `spec-<family>.yaml` |
| Requirement flow | `spec-to-reqs/SKILL.md` (`curate`/`extract`/`validate`/`trace`) |
| NL_HDL format + flow | `nlhdl/SKILL.md`, `nlhdl/references/{format,gen-nlhdl,gen-rtl,inspect-hierarchy}.md` |
| Frozen CII contract | `src/main/sv/v4/tt-cii/src/tt_cii_caracal_pkg.svh` |
| Verified coprocessor | `src/main/sv/v4/vpu/tt_vpu_cii_wrapper_top.sv`, tb `src/main/sv/v4/vpu/tb/cii_fv_tb.sv` |
| Previous attempt (reference code) | `origin/Caracal/addvector` — 41 files, ~10.4k lines under `src/main/scala/v4/vec/` |
| Previous attempt's bug list | `caracal-milestone1-plan.md` §*Bugs found and fixed during bring-up* |
| v4 scalar baseline | `docs/boom-v4-architecture.md` |

Load-bearing spec anchors, cited by step below:

- `loadstore.rst` — `ssi-queues`, `load-coalesce`, `elem-progress`, `mem-order`,
  `vec-squash`, `vec-queue-reservation`, `dcache-arbiter`
- `midcore.rst` — `vrf-ports`, `old-vd`, `vl-vtype-rename`
- `cii.rst` — `cii-flush`, `cii-operands`
- `frontend.rst` — `vector-csr-ownership`, `vset-dual-dest`, `vector-rvv-decode`
- `issue.rst` — `shared-store-chain`, `vec-queue-reservation`

**Branch strategy.** Current branch is `Caracal/addvector2`, which carries the docs and the
refactored SV but **no vector Chisel**. Each step below is a `Caracal/v2/<step-slug>` feature
branch merging into `Caracal/addvector2`, then into `Caracal/main`. Steps are ordered so that
every intermediate state compiles, elaborates, and is bit-identical to baseline when
`usingRVV=false`.

Whenever you are uncertain about implementation details or planning, ask/prompt the user.

---

## 2. What went wrong, and what v2 does differently

The previous attempt shipped a working unit-stride/strided/indexed vector load/store path
that passed a full SEW×LMUL matrix under Whisper cosim — and was serialized to roughly one
element access per several cycles, with one vector memory instruction in flight machine-wide.

**The root cause was structural, not tuning.** `VecLSU.scala:159` declared module-level
`state` and `grp_active` registers; line 506 exported

```scala
io.busy := grp_active || (state =/= State.sIdle)
```

which gated `fu_ready`. One FSM, shared by both directions, spanning operand-read → AGEN →
D$ → completion. A concurrency ceiling of 1, by construction. Everything else followed from
it, and each follow-on was fixed locally without the ceiling ever being questioned:

| Symptom (from the M1 as-built log) | Actually a consequence of |
|---|---|
| Load-priority mux; "the store does not advertise `FC_AGEN` in a cycle the load is granting" | one FSM serving two directions |
| Serial `sCopyRd`/`sCopyWr` undisturbed-copy prologue, 2 cycles × 8 members, in front of every masked or tail load | no VRF read port for `stale_pvdest`, so the copy could not overlap |
| `agen_active` having to OR in `vec_dgen.active` | completion state living outside the queues |
| Beats bare-physical, `uses_tlb=false`, `uses_lcam=false`, appended **last** in `lsu_sched` | vector accesses treated as an afterthought on the scalar port |
| Packer on the fill side → every access one 64b beat per element | coalescing placed where it cannot help |

### The five structural changes

1. **No module owns "the current vector memory op."** In-flight state lives only in the six
   element queues and the LCB's assembly entries. Stated as a ground rule
   ([§5](#5-global-ground-rules) rule 6) so a violation is reviewable.
2. **Address generation is cut by pipeline position, not by direction.** `VecElemAgen`
   (fill, SSI) / `VecRangeAgen` (fill, US) / `VecBeatExpander` (**drain**, coalescing). This
   replaces the six inherited OVI modules (Packer/Skipper/Walker × Load/Store, 2110 lines)
   whose duplication had already produced divergent mask support between the load and store
   Packers.
3. **Direction is a parameter.** Every agen is instantiated twice, so the two directions
   never arbitrate.
4. **The undisturbed copy becomes an overlapped pre-load** on new VRF read port `R2`, inside
   the LCB, instead of a serial prologue.
5. **Vector accesses are first-class in the memory system** — real DTLB translation,
   bidirectional LCAM, and a priority round-robin arbiter with an anti-starvation floor.

### What survives from `origin/Caracal/addvector`

10.4k lines of debugged Chisel is not thrown away. Roughly 3000 lines are superseded; the
rest is ported, mostly with small deltas.

| `addvector` module | Lines | v2 fate |
|---|---|---|
| `lsu/VecLSU.scala` | 525 | **Deleted.** The global FSM *is* the defect. |
| `lsu/Vec{Load,Store}{Packer,Skipper,Walker}.scala` | 2110 | **Superseded** by `VecElemAgen` + `VecRangeAgen` + `VecBeatExpander`. Carry over the mask/index **lookahead hazard** rules (§ note below). |
| `rename/VlRename.scala` | 405 | **Superseded** by a second `VecRenameSpace` instance (`numArchRegs: 1`). |
| `lsu/DcacheArbiter.scala` | 91 | **Rewrite** as `VecDcacheArbiter`: priority round-robin + anti-starvation, not lowest-priority append. |
| `lsu/VecAgenStage1.scala` | 426 | **Split** into `VecElemAgen` / `VecRangeAgen`. |
| `lsu/VecMemQueues.scala` | 120 | **Split** into `VecElemQueue` (×6) + `VecQueueReservation`. |
| `lsu/VecLSRegRead.scala` | 176 | **De-shared** into `VecScalarOperandRead` ×2. Keep the INT-writeback snoop/forward. |
| `cii/VecCiiHost.scala` | — | **Decomposed** into `VecCiiHost` + TagTable / **Issue** / OperandServer / Writeback / **Complete** / Flush — one node per channel direction, plus last-beat completion. |
| `lsu/VecLoadCoalescingBuffer.scala` | 121 | Port + **extend**: `R2` stale pre-load, per-byte write mask. |
| `lsu/VecDgen.scala` | 179 | Port + **fix**: `is_shared` operand mux, total-bytes completion. |
| `lsu/CrossLsuSnoop.scala` | 86 | Port + **extend** to bidirectional. |
| `rename/VecMapTable.scala` | 281 | Port, **minus** the LMUL tag table / whole-group checker. |
| `rename/VecFreeList.scala` | 314 | Port, all-or-nothing `pvdest`+`pvtmp`. |
| `decode/*`, `issue/*`, `regfile/*`, `rename/VecBusyTable.scala`, `lsu/VecIdxGen.scala` | ~1500 | Port with small deltas (see the per-step notes). |
| `rename/VecRenameTypes.scala`, `cii/CiiBundles.scala`, `lsu/VecNop.scala`, `lsu/ConfigInfo.scala` | ~475 | **Merged** into `VecBundles`. |

> **The `addvector` bug list is a test suite.** Every bug in `caracal-milestone1-plan.md`
> §*Bugs found and fixed during bring-up* was found by simulation, not review, and most are
> re-introducible in a fresh implementation. Each one is attached to the step that owns it
> below, and its **absence must be positively demonstrated**, not assumed.

---

## 3. Scope boundary: host-side only, with one exception

The SV coprocessor is refactored and **verified** (`cii_fv_tb.sv` plays the host through the
real credit relay and checks per-member writeback including widening/narrowing where dest
`EMUL` ≠ source `EMUL`). v2 therefore writes **no** VPU RTL, and two SV nodes only:

| SV node | Mode | Why |
|---|---|---|
| `tt_cii_host_wrap.sv` | **new** | A Chisel `BlackBox` binds flat `logic` ports only — not an SV `interface`/`modport`, nor `parameter type` packed-struct ports. The VPU wrapper's port is `tt_cii_interface.coprocessor cii_intf`, so it cannot be a `BlackBox` target directly. This shim flattens the four channels and repacks them. `addvector`'s 203-line version is a working reference. |
| `tt_cii_caracal_pkg.svh` | **edit_existing** | Add `CII_SRC_STALE_VD = 3'd6` so the coprocessor can request the old-`vd` group independently of the encoded third source. One enum value; the wire type is already `logic [2:0]`, so no payload/struct/interface/relay change and the verified tb stays valid. |

> **Cross-team dependency.** The `STALE_VD` slot needs the **VPU decoder** to emit it. That
> is outside this plan and must be coordinated with the VPU owner. It is safely stageable:
> until the VPU emits slot 6, the host serves it correctly and simply never sees a request.
> Slot 3 also wants renaming `CII_SRC_VS3_VD` → `CII_SRC_VS3`; declare that in the nlhdl
> `<|begin_edit_scope|>` interface delta or leave the name and narrow its meaning in a
> comment. A silent rename violates `edit_existing` discipline.

**No SV is copied into `src/main/resources/vsrc/`.** `addvector` copied 7 files there —
one of them submodule content — because `HasBlackBoxResource` only resolves under
`src/main/resources/`. v2 uses `HasBlackBoxPath` against `src/main/sv/v4/**` directly. A
stale copy would silently falsify this plan's central premise: you would verify one file and
simulate another, with nothing to tell you. If `addPath` fights the Chipyard flow, the
fallback is a sync step with a checksum guard that **fails the build** on drift — never a
silent copy.

---

## 4. Methodology: the whole design in NL_HDL, then RTL

**Every module in the design is written in NL_HDL and the whole set is reviewed before any RTL
is generated.** Not per module, and not per phase — the entire design. This is the primary
process change in v2, and it exists because the previous attempt's worst defects were
*integration* defects in baseline files — the free-list double-free, the rename lockstep bug,
the `stq_execute_head` hang — none of which a unit test would have caught and all of which a
written interface contract would have made visible.

The three stages run strictly in order:

| Stage | What | Exit condition |
|---|---|---|
| **0. Architect** | `hierarchy.yaml`: the node set, and every live requirement allocated | `inspect-hierarchy` clean; 0 unallocated requirements |
| **1. Author** | Every `.nlhdl.*` file for all 58 non-blackbox nodes | `inspect-hierarchy` clean; no module left unwritten |
| **2. Review** | The complete set, read against the `.rst` specs as one artifact | Every seam agreed on both sides; every requirement traced |
| **3. Generate** | RTL, **phase by phase**, each phase gated before the next starts | Per-phase gate green ([§6](#6-verification-gates)) |

**Stage 0 is done, and it changed the node count: 49 non-blackbox nodes → 58.**
`/nlhdl architect` allocated all **1162** live requirements in `src/main/nlhdl/reqs/`
— **1065** to a specific node, **97** to the map's `reqs_out_of_scope:` ledger, **0**
unallocated. Allocating them is what exposed the gaps, and each of the nine added nodes
names one:

| Added node | Why it exists |
|---|---|
| `VecGroupCopy` | **13 requirements with no owner at all.** A `VL = 0` op still reaches issue for every non-immediate-AVL form, its `pvdest` group is already renamed, and its ROB entry cannot commit until that group is architecturally correct — but with `VL = 0` the LSU executes no element, so nothing would ever write it. `loadstore.rst` specifies a `stale_pvdest` → `pvdest` group copy on the Load Unit's ports. The previous map had nowhere to put it. |
| `VecOrderHold` | **6 requirements with no owner.** A younger vector load overlapping an older draining vector store in a combination that does not forward (SSI/SSI, US/SSI) must *wait*, released by the store's element cursor completing. Neither a search nor a forward, so it fell between `VecCrossLsuSnoop` and the arbiter. |
| `VecCiiIssue` | The CII host had a node per channel direction **except Issue** — 26 requirements, including the entire issue-packet contract (`cii.k*`), left inside the container next to the BlackBox binding. |
| `VecGroupReady`, `VecStoreDgenPath` | `VecIssueSlot` was carrying **56** requirements. The corpus splits them: per-member group readiness (identical for `pvs1/2/3`/`pvm` **and `stale_pvdest`** — define once, instantiate **five** times) and the store-only AGEN/DGEN dual-grant path. |
| `VecRegFileBank` | `VecRegFile` was carrying **48**, spanning two unrelated things: *who owns which port* (a contract, canonical in `midcore.rst`) and *how the array is built*. The split also confines the VRF-area risk — flops vs latch/SRAM banking changes this node only. |
| `VtypeTable` | `VConfigUnit` was carrying **44**. The `vtype` → `{VLMAX, EMUL, vill}` rules were restated in three places (`VConfigUnit`, `VsetDecode`, `ALUUnit`); now a `kind: package` all three bind to. Three copies of a `vill` rule is three chances to disagree. |
| `VecStoreForward` | Split from `VecCrossLsuSnoop`, which held both directions of cross-queue ordering. One raises `order_fail`, the other returns **data**; they land in different steps (G1 vs G3). |
| `VecCiiComplete` | `VecCiiWriteback` was carrying **41**. Completion is driven by the beat's `last` bit and must *not* be inferred by counting beats, so it needs almost nothing from beat placement — a clean seam. |

Three nodes stay above 40 requirements deliberately, for two different reasons.
`Rob` (85) and `LSU` (45) are `edit_existing`: most of their obligations are
*must-not-regress* constraints discharged collectively by the `<|begin_edit_scope|>`
out-of-scope list, not per-line logic — their line budgets in [§11](#11-file-touch-summary)
are unchanged. `VecPipeline` (49) is the container: its requirements are wiring topology
and design-wide invariants checked by cross-file inspection in **R3**, not logic in one file.

The reason to finish all authoring first is that an interface defect is only visible from both
sides at once. Writing `VecLsu`'s spec in isolation cannot reveal that `host/LSU`'s delta spec
expects a different handshake; reading the two together can, and reading all 58 together is the
only point at which a *design-wide* invariant — ground rule 6, the VRF port partition, the
`pvtmp` rendezvous — can actually be checked rather than assumed.

Stage 0 is the same argument applied one level up, and it earned its keep: allocating the
corpus is what turned "the LSU is where v2's redesign lives" into a specific finding that two
of its mechanisms had no module at all. Neither `VecGroupCopy` nor `VecOrderHold` would have
been noticed by writing the fourteen `vec/lsu/` specs one at a time — they are visible only
against the requirement list.

### 4.1 Layout

```
src/main/nlhdl/
├── hierarchy.yaml                     # the module map + requirement allocation (DONE)
├── reqs/                              # the requirement corpus, 1162 live (DONE)
├── pkg/   8   MicroOp, ScalarOpConstants, BoomCoreParams, BoomConfigMixins,
│             VectorParams, VecBundles, VecTrace, VtypeTable
├── host/  7   BoomCore, Rob, LSU, DecodeUnit, ALUUnit, ALUExeUnit, FpPipeline
├── vec/   1   VecPipeline
│   ├── decode/  5  VecDecode, VDecode, VLSDecode, VsetDecode, VConfigUnit
│   ├── rename/  4  VecRenameSpace, VecMapTable, VecFreeList, VecBusyTable
│   ├── issue/   4  VecIssueUnit, VecIssueSlot, VecGroupReady, VecStoreDgenPath
│   ├── regfile/ 3  VecRegFile, VecRegFileBank, VlRegFile
│   ├── lsu/    17  VecLsu, VecQueueReservation, VecScalarOperandRead,
│   │                VecElemAgen, VecRangeAgen, VecIdxGen, VecMaskStream,
│   │                VecElemQueue, VecBeatExpander, VecDgen,
│   │                VecLoadCoalescingBuffer, VecGroupCopy, VecDcacheArbiter,
│   │                VecCrossLsuSnoop, VecStoreForward, VecOrderHold,
│   │                VecSquashUnit
│   └── cii/     7  VecCiiHost, VecCiiIssue, VecCiiTagTable,
│                    VecCiiOperandServer, VecCiiWriteback, VecCiiComplete,
│                    VecCiiFlush
└── sv/    2   tt_cii_host_wrap, tt_cii_caracal_pkg
                                                             total 58 nodes
```

File naming is `<module_name>.nlhdl.<hdl>` — `.scala` for Chisel, `.sv` for SystemVerilog.
The module name **must** match the `hierarchy.yaml` key and the emitted RTL module name.

### 4.2 The three modes

`hierarchy.yaml` assigns every non-blackbox node a `mode:`, which decides how much of the
file the nlhdl source is authoritative over:

| `mode:` | The nlhdl file describes | Artifact |
|---|---|---|
| `new` | the whole module | `output:` written fresh |
| `edit_generated` | the whole module | `output:` regenerated |
| `edit_existing` | **only the delta** | `target:` edited in place |

**`edit_existing` is how baseline BOOM files are touched, and it is a hard constraint, not a
guideline.** For these 12 nodes the nlhdl file is a *delta spec*: it describes only the
change, carries a required `<|begin_edit_scope|>` section bounding it, and never restates the
existing port list. The generator must:

- Edit in place, preserving existing style, naming, formatting, and license header.
- Change **only** what the spec calls for. No drive-by cleanups, renames, or reordering.
- Keep everything outside the specified change **bit- and cycle-identical**.
- **Stop and report** rather than guess, if the spec cannot be applied without touching
  unspecified behavior.

### 4.3 Stage 1 — author every nlhdl spec

Walk `hierarchy.yaml` in topological order and write every non-blackbox node. Per module:

1. `/nlhdl gen-nlhdl` — author the spec. Cite the governing `.rst` anchor in a file-level
   comment, and the requirement IDs it discharges (`src/main/nlhdl/reqs/spec-*.yaml`). For
   `edit_existing`, write the `<|begin_edit_scope|>` first — target, in scope, out of scope /
   must not regress, interface delta.
2. `/nlhdl inspect-hierarchy src/main/nlhdl/hierarchy.yaml` — must be clean. It cross-checks
   `<|begin_dependencies|>` against `instantiates:`, so a dependency you forgot to declare
   surfaces here.

**No RTL is generated in this stage, for any module.** Topological order is for the author's
benefit — a module's dependencies are easier to write about once its dependencies exist on
paper — not a licence to start emitting the early ones.

### 4.4 Stage 2 — review the complete set

Nothing is generated until this passes. Read the whole set as one artifact, in this order:

1. **Each spec against its `.rst`, not against the old Chisel.** Where they disagree, the
   `.rst` wins, or the `.rst` is wrong and gets fixed first. The previous attempt's deviations
   happened because the code was the only artifact.
2. **Each seam from both sides.** For every edge in `hierarchy.yaml`, read the producer's and
   consumer's specs together and confirm they describe the same handshake, the same widths, and
   the same back-pressure direction. This is the check that only exists once everything is
   written, and it is the one that would have caught `stq_execute_head`.
3. **Design-wide invariants, by inspection across all files at once:** ground rule 6 (no module
   owns "the current vector memory op"), the `vrf-ports` partition, the `pvtmp` rendezvous, and
   the `usingRVV` gating of every baseline delta.
4. **Requirement coverage.** Every live requirement in `src/main/nlhdl/reqs/` is either cited by
   some nlhdl spec or explicitly recorded as out of v2 scope. 1162 requirements exist and none
   are tagged yet; this review is where the mapping is established, before RTL makes it
   expensive to change.

Defects found here are fixed in nlhdl and the affected seams re-read. Stage 2 can iterate; what
it cannot do is hand a known-defective spec to Stage 3.

### 4.5 Stage 3 — phased RTL generation, gated per phase

RTL is generated **phase by phase**, in the Phase A→H order of [§8](#8-steps). A phase's RTL is
generated, then verified against that phase's gate, and **the next phase does not start until
the gate is green.** This is the second reason to finish authoring first: because the specs are
settled, a failure in generated RTL is a generation defect or a spec defect, and the two are
distinguishable — under the old interleaved flow a Phase C failure could always be blamed on
Phase D's interface not being written yet.

If a phase's RTL fails its gate and the cause is the *spec*, stop and fix the spec, then
regenerate that phase. Do not patch generated RTL to pass a gate: the nlhdl file is
authoritative, and a hand-patched output is overwritten by the next `gen-rtl` run.

### 4.6 Model tiers

The two stages have different failure costs, so they use different models:

| Stage | Model | Why |
|---|---|---|
| 1. Author nlhdl | strongest available | A spec defect propagates into RTL, tests, and every consumer of the seam. This is where design judgement lives. |
| 2. Review | strongest available | Same reason, plus cross-file reasoning over 58 files. |
| 3. `gen-rtl` | **cheapest model that passes the phase gate** | Mechanical translation from a settled spec to Chisel/SV. The nlhdl file is the contract and [§6](#6-verification-gates) is the check, so a weaker model's errors are caught by the gate rather than shipped. |

Start Stage 3 on the cheapest tier and escalate **per phase, only on gate failure** — a phase
that fails twice on the cheap tier moves up rather than being retried indefinitely. Record which
tier each phase was generated on in [§12](#12-implementation-notes--deviations-v2-as-built), so a
later correctness question can be traced to the tier that produced the code.

The economics only work because the gate is real. A cheap generator behind a weak gate is how
unreviewed RTL enters the tree; the phase gate in [§6](#6-verification-gates) is what makes the
tier choice safe, and it is not negotiable per phase.

---

## 5. Global ground rules

1. **Feature flags.** All vector logic gated by `enableVector: Boolean = false`
   (`BoomCoreParams`), surfaced as derived `usingRVV`. Default off — every existing config and
   test stays bit-identical to pre-Caracal output. Sub-flags so tracks land independently:
   `enableVectorArith` (CII attach + `IQ_V_ALU` grant) and `vecScalarSnoopEnable` (cross-LSU
   disambiguation).
2. **Code location.** All new Chisel under `src/main/scala/v4/vec/generated/`, package
   `boom.v4.vec.generated.*`. sbt compiles only under `src/main/scala/`, and the nlhdl
   validator wants `output:` under a `generated/` directory; this path satisfies both.
3. **Minimize intrusive edits, with a budget.** `addvector` added **1101 lines to
   `core.scala`** under a rule that said "touch baseline files only where unavoidable." v2
   replaces that with one `VecPipeline` container behind one explicit bundle, and a stated
   budget: **`core.scala` ≤ 200 added lines.** Over budget means the container boundary is
   leaking and the excess belongs inside `VecPipeline`. Reviewable, unlike "minimize."
4. **Parameters.** `VLEN = 256`, `ELEN = 64`, **`numVecPhysRegisters = 96`**,
   `numVlPhysRegisters = 64`. Everything else parameterized. **No cracker parameters** — the
   atomic mapper removes frontend cracking. **No `numVecTmpGroups`** — no free-list headroom
   is needed for forward progress, because rename is in program order so an op that cannot
   allocate simply stalls; the requirement is that `pvdest`+`pvtmp` allocation is
   all-or-nothing. Default tier under test is `MediumBoomV4VectorConfig`; must elaborate
   cleanly at **Medium/Large/Mega**. **SmallBoom is NOT in the vector matrix (D3)** —
   at `coreWidth = 1`, `CompactingDispatcher` cannot elaborate (`IQ_MEM` is forced to
   `issueWidth >= 2` by `require(memWidth >= 2)` while `dispatchWidth = coreWidth = 1`,
   and `parameters.scala:275` forbids widening it) *and* `allocWidth = 8` cannot serve a
   shared `OP.v`'s all-or-nothing 16 PRNs, which is a deadlock. "Small forbids segmented
   vector LS" was rejected as architecturally invalid: segmented load/store is base
   RVV 1.0 and rule 1 of §1 requires RVV 1.0.
5. **No frontend cracking; atomic group rename; OoO across queues.** A vector instruction is
   a single `OP.v` through decode/rename/ROB/issue. The mapper renames a whole `LMUL`/`EMUL`
   group atomically (≤ 8 PRNs per `vdest`); element cracking into `nOP.v` happens only in the
   vector LS AGEN. All three vector queues are **age-ordered collapsing**; `IQ_V_ALU` adds a
   **per-entry past-PNR eligibility gate** (RoCC-style), so every op handed to the CII is
   individually non-speculative. That removes **branch**-kill from the CII but **not** flush
   recovery — past-PNR is not a commit guarantee (`rob.scala:436-442`), so the CII needs the
   drain-on-flush contract in `cii.rst` `cii-flush`. `IQ_V_ALU` is deliberately **not** a
   head-only FIFO: the VPU being in-order requires one-at-a-time *execution*, not
   program-order *issue*, and a head-only queue would let a segmented store's coprocessor half
   block all younger vector arithmetic. Inter-queue ordering is enforced exclusively by the ROB.
6. **⇒ THE VECTOR-LSU INVARIANT (new in v2).** **No module in the vector LSU may hold state
   scoped to "the current instruction," and no module may export a `busy` that gates issue.**
   In-flight state lives only in (a) the six `VecElemQueue` instances, whose capacity was
   reserved at dispatch in program order, (b) the LCB's per-PRN assembly entries, and
   **(c) `VecLsu`'s per-LDQ/STQ-entry descriptor pending table (added by D5)**.
   (c) was added because nothing throttled the *producer* of a multi-cycle element walk:
   the `iss -> VecScalarOperandRead -> agen` chain has no `ready` anywhere, yet an SSI walk
   takes up to `vl` cycles and an INT-RF read can be denied by `PartiallyPortedRF`. (c) is
   the same *kind* of state as (a) — per-queue-entry, capacity reserved at dispatch, no
   `busy` exported, structurally un-overflowable. Qualifying the `FC_AGEN` grant instead was
   rejected as a `busy` under another name, which is exactly what gate H4 greps for.
   *The rule is amended in the text rather than letting a table appear that the rule forbids.*
   `VecElemAgen` keeps an element cursor but retires it the moment the last element is pushed
   into the queue — it never waits on a D$ response or on completion, so it cannot gate the
   next instruction. Issue eligibility is purely "is there a reservation," decided at dispatch.
   *A `busy`-style signal reaching an issue unit is a failed review, regardless of measured
   performance.*
7. **Group-done completion.** Every vector producer signals completion of a whole destination
   group **once**, carrying the group's member-PRN vector. One event drives three consumers:
   the ROB single-shot busy-clear, the vector Busy-Table clear, and the vector wakeup. Loads
   emit it from the LCB after the last destination PRN lands; the CII emits it on the writeback
   beat marked `last`. **No per-entry ROB completion counter.**
8. **Precise exceptions.** Vector loads/stores trap with **`vstart = 0` and restart the whole
   instruction**. Mid-vector resume (`vstart = k`) is not implementable here: the faulting
   `OP.v` never commits, so its fresh `pvdest` group is reclaimed and elements `0..k-1` were
   never architecturally visible. `fault_elem` is retained only as the element-cursor stop
   signal and a debug counter. Stores translate/disambiguate all active elements pre-commit and
   do not write the DCache until ROB-committed.
9. **Vector architectural CSR state is rocket's.** `vtype` (incl. `vill`), `vl`, `vstart`,
   `vxrm`, `vxsat`, `vcsr`, `vlenb`, and `mstatus.VS` (dirty tracking + the `VS=Off`
   illegal-instruction gate) come from rocket-chip's `CSRFile` under `usingVector`
   (`csr.io.vector`). Caracal owns **only** the speculative VCFG `vtype` mirror and the VL
   register file. See `frontend.rst` `vector-csr-ownership`.
10. **Reuse the existing wakeup machinery.** Scalar feeders (base/stride/`.vx`) ride the
    existing INT network; `.vf` rides FP. Two new networks: **VL** (`pvl`) and **VECTOR**
    (group-done, member-PRN vector). No VLBU / value broadcast — `pvl` is a plain readiness
    wakeup and the VL value is read from the VL RF at execute.
11. **No unit tests — module `printf` tracing instead.** No `generators/boom/src/tests/`, no
    `chiseltest`/`ChiselSim`, no per-module `*Spec.scala`. Validation is end-to-end
    VCS+Whisper regressions plus the per-step gate. To make those debuggable, **every vec
    module emits guarded trace statements** via the shared `VecTrace` package: gated on a
    `vecTrace` plusarg (off by default, so the `usingRVV=false` bit-identical baseline is
    unaffected), one line per key event, each line tagged with module name and `rob_idx` (plus
    `pvdest`/`pvl`/`vl`/`v_emul` where relevant) so stages correlate with each other and with
    the Whisper trace, and greppable.
12. **Vector configs use `CompactingDispatcher`, not `BasicDispatcher` (D2).** A vector
    config does not elaborate otherwise — the three `IQ_V_*` `issueParams` entries fall
    through `core.scala:837-849` to `require(false)`. And `BasicDispatcher`'s ready is not
    masked by `iq_type`, so wiring the vector queues in under it would let a full
    `IQ_V_LOAD` stall **pure-scalar lanes carrying no vector uop**, threatening gate (d) and
    P6. `CompactingDispatcher` already masks correctly ("the queue is considered ready if
    the uop doesn't use it"). Costs a `Compactor` per queue.
13. **End-to-end ELF tests go through VCS with the Whisper cosim sidecar — never Verilator.**
    New ELFs are appended to `sims/vcs/tests_regr/*.txt`.
14. **`hierarchy.yaml` is the design authority.** Adding a module, a VRF port, or an
    instantiation means amending the map **first**. In particular the `vrf-ports` partition is
    canonical: nothing adds a VRF port without amending `midcore.rst` and the map.

---

## 6. Verification gates

A step is done only when **all** of (a)–(h) pass. Do not merge a step branch until they are green.

**The phase gate.** Under the Stage-3 discipline of [§4.5](#45-stage-3--phased-rtl-generation-gated-per-phase),
(a)–(h) are additionally run **for the phase as a whole** once its last step lands, and the next
phase does not start until that run is green. Two checks are specific to the phase gate and do
not apply per step:

| | Phase gate |
|---|---|
| **i** | **Generated RTL matches its nlhdl spec.** For every module the phase generated, diff the emitted RTL against the spec's stated interface — port names, widths, directions — and confirm no behavior appears in the RTL that the spec does not describe. This is the check that makes a cheap generation tier safe; without it, a weaker model's invention is indistinguishable from design intent. |
| **j** | **No hand edits to generated output.** `git diff` on the phase's `output:` paths after a clean re-run of `/nlhdl gen-rtl` must be empty. A non-empty diff means either the output was patched by hand — which the next run silently discards — or generation is not deterministic. |

Gate (i) failure is triaged before anything else: **spec defect → fix the nlhdl and regenerate;
generation defect → escalate the model tier for that phase per [§4.6](#46-model-tiers).** Patching
the output is not a third option.

| | Gate |
|---|---|
| **a** | `sbt compile` clean in `generators/boom/` |
| **b** | `make checkstyle` clean in `generators/boom/` |
| **c** | **Chipyard build with vector enabled** — catches diplomacy/parameter-ripple breakage an isolated `sbt compile` misses: `cd /root/my-chipyard/sims/vcs && make CONFIG=MediumBoomV4VectorConfig -j$(nproc) debug` |
| **d** | **Scalar perf regression with vector enabled** — proves the scalar datapath is unbroken by live vector plumbing. Required at *every* step, including steps that add no executable vector behavior: `./run_regr_rvv_scalar.sh MediumBoomV4VectorConfig` |
| **e** | **Vector regression, phased.** (e1) `vset` smoke — required from step D3 on, must pass before any LS test is attempted. (e2) `vset` + load/store — from step E7 on. (e3) memory-ordering suite — from step G3 on. (e4) vector-arith suite — from step F5 on. Earlier steps may skip the phases that do not yet apply. |
| **f** | **`usingRVV=false` identical to the RE-BASELINED reference, except for the enumerated encoding widths.** Elaborate the plain `Small`/`Medium`/`MegaBoomV4Config` (vector mixin not applied) and run `docs_caracal/v2-rebaseline/gate-f-check.py --pre <rebaseline> --post <fresh>`; it must report **zero violations**. Normalization ignores only `@[...]` source locators, the `firtool` banner, and assertion strings embedding `file:line`. The check is *structural* for modules carrying the widened fields and *strict textual equality* for all others — see that directory's README, including what it does not prove. **RELAXED from "bit-identical to pre-Caracal v4" — see §6a for why, the exact exception, and the artifact it depends on.** |
| **g** | The step's listed artifacts pass, and its trace output shows the expected instruction flow in the (d)/(e) logs, cross-checked against Whisper. |
| **h** | **Performance gate.** The step's stated numeric target in [§7](#7-performance-targets) is met, or the step fails. New in v2. |

Additionally, for every `edit_existing` step: the report must include **a diff of the touched
hunks plus an explicit statement of what was left untouched**, and the added-line count must
be within the step's stated budget.

---

### 6a. Gate (f) is a bounded exception, not bit-identity (decision D1)

Gate (f) originally claimed `usingRVV=false` is **bit-identical to pre-Caracal BOOM v4**.
That is **false by construction** and cannot be fixed cheaply: `ScalarOpConstants` is a bare
Scala `trait` with no `Parameters` in scope, so it widens the register-type space and `IQ_SZ`
**unconditionally**, and `MicroOp` and `Rob` must track it.

**The exception, exhaustively. Nothing outside this list is permitted to differ:**

| What | From | To |
|---|---|---|
| `RT_FIX`/`RT_FLT`/`RT_X`/`RT_ZERO` | `UInt(2.W)` | `UInt(3.W)` (values 0..3 unchanged) |
| `IQ_SZ`, hence `MicroOp.iq_type` | 4 | 7 |
| `MicroOp.dst_rtype`/`lrs1_rtype`/`lrs2_rtype` | `UInt(2.W)` | `UInt(3.W)` |
| `Rob`'s compact `dst_rtype` | 2b | tracks `MicroOp` (and `compactUopWidth`, which sizes `rob_compact_uop_mem`, therefore +1b) |
| `decode.scala`'s `decode_default` rs1/rs2 regtype don't-cares | `DC(2)` | `DC(3)` — *added at A2* |

*The last row was added during A2 and is not optional.* `DecodeLogic` refuses to pad a
`BitPat` containing don't-cares, so a stale `DC(2)` fails elaboration outright
(`Cannot pad 'BitPat(??)' to '3' bits because it has don't cares`) for **every** config,
vector or not — the widening cannot be landed without it. Note the cross-phase consequence:
this line lives in `decode.scala`, the `DecodeUnit` node's file, which is otherwise Phase B2 —
just as the `Rob` row lives in D3's file. **A widening that is unconditional by construction
has an unconditional blast radius, and it does not respect phase boundaries.** Any step that
touches the `RT_*` or `IQ_*` encodings must re-check this list rather than assume it is closed.

**New step, and gate (f) is meaningless without it.** A0 has a sibling, **step A1**: generate
and check in a **re-baselined reference** (`docs_caracal/v2-rebaseline/`) — pre-Caracal BOOM v4
plus exactly the widenings above and no vector logic. Every later step's (f) claim diffs
against that. Without the artifact, "identical except the enumerated widths" is unfalsifiable
and the guarantee that makes every other step cheap to review evaporates.

A1 checks in **two** artifacts, not one, because a lone post-widening reference cannot be
audited — it already contains the exception, so diffing against it cannot show whether the
widening dragged anything else along. `prebaseline` (before the widening) is the anchor;
`rebaseline` (after it, still no vector logic) is what later steps diff against; and the
`prebaseline`↔`rebaseline` diff is the check that proves the exception is bounded. Tooling and
the exact method are in `docs_caracal/v2-rebaseline/README.md`; the checker is
`gate-f-check.py`, which is structural rather than textual because widening `dst_rtype`
renumbers bit positions in every bundle that packs a `MicroOp`.

*Rejected:* parameterizing the encodings so a non-vector build emits 2-bit rtypes. Cleaner,
but the `RT_*`/`IQ_*` trait is consumed during `issueParams` construction before
`Parameters` exists.

**⚠ A passing gate (f) does NOT prove the `IQ_V_*` defaults exist.** `DecodeUnit` does
`uop := io.enq.uop`, and `io.enq.uop` comes from a bundle the frontend sets `:= DontCare`.
Without six explicit default assignments, every *scalar* uop carries three don't-care
vector-queue routing bits into dispatch — mis-routing, not just an X in a waveform. A
don't-care bit can elaborate bit-identically and still mis-route.

---

## 7. Performance targets

The previous attempt had no performance gate, which is why a design with a concurrency ceiling
of 1 passed every milestone. v2 fixes the process as well as the design.

### Step A0 — measure the baseline first

Before any v2 code, build `origin/Caracal/addvector` at `MediumBoomV4VectorConfig` and record,
per kernel in `tests_regr/vset_loadstore_tests.txt`:

- total cycles;
- D$ accesses per vector memory instruction;
- vector-LS pipe busy cycles (the M1 HPM event already exists);
- max and mean concurrent vector memory instructions.

These are the numbers v2 must beat. **A0 produces a checked-in artifact**
(`docs_caracal/v2-baseline.md`); without it, gate (h) has nothing to compare against and the
whole plan repeats the previous failure.

### Targets

With `VLEN=256` and a 64-bit D$ port, the **access-count floor** for a unit-stride access is
`ceil(active_bytes / 8)` — 4 accesses for an `LMUL=1` full-vector op, 32 for `LMUL=8`,
independent of `SEW`. Targets:

| # | Target | Measurement | Rationale |
|---|---|---|---|
| P1 | Unit-stride D$ accesses per op **= the floor** `ceil(active_bytes/8)` | HPM: vec D$ accesses ÷ vec mem retire | `VecBeatExpander` coalesces at drain. Baseline emitted one access per *element* — 8× worse at `SEW=8`. |
| P2 | Sustained **≥ 1 D$ access per cycle** while the arbiter grants and the queue is non-empty | HPM: accesses ÷ (vec LS busy − stall) | Baseline needed a multi-cycle FSM round-trip per beat. |
| P3 | **≥ 2** vector memory instructions concurrently in flight on a mixed load/store kernel | HPM: max concurrent | Baseline ceiling was 1. Queue depth bounds this at 512÷256 = 2 worst-case stores (`loadstore.rst`), more for typical `VL`. |
| P4 | A `vle` and a `vse` **overlap** — neither's issue is gated by the other | trace: interleaved element accesses from two `rob_idx` | Baseline had a load-priority mux that dropped store grants. |
| P5 | A masked / tail-undisturbed load costs **no extra cycles** vs unmasked at equal active-byte count | cycles(masked) − cycles(unmasked) ≈ 0 | The `R2` stale pre-load overlaps memory latency; baseline paid a serial 2-cycle-per-member copy prologue. |
| P6 | Scalar kernels: **no regression** vs the vector-disabled build | `run_regr_rvv_scalar.sh` | The arbiter's scalar-priority floor must actually hold. |
| P7 | Strip-mined `memcpy` total cycles **≤ 40%** of the A0 baseline | cycles | Composite of P1–P4; the single headline number. |

P7's 40% is a target set from P1 alone (8× fewer accesses at `SEW=8`, ~4× at `SEW=32`) with
generous allowance for other bottlenecks; **revise it once A0 exists**, and treat a revision as
a plan amendment rather than a silent adjustment.

---

## 8. Steps

**Phases N and R are Stages 1 and 2 — all nlhdl, then the design-wide review. Phases A–H are
Stage 3 and are pure RTL generation**, each gated as a whole before the next begins
([§4.5](#45-stage-3--phased-rtl-generation-gated-per-phase)). Numbers are step identifiers, not a
strict serialization — steps with no dependency edge in `hierarchy.yaml` may proceed in parallel
*within* a phase.

### Phase N — Author every nlhdl spec (Stage 1)

All 58 non-blackbox nodes: 46 `new`, 12 `edit_existing`. **No RTL is generated in this phase.**
Steps are grouped by subsystem only so the work is reviewable in chunks; N1–N7 must all complete
before Phase R. Each node's `reqs:` list in `hierarchy.yaml` is the checklist for its spec — the
IDs it must cite are already decided, so authoring is not also a scoping exercise.

| Step | Kind | Scope |
|---|---|---|
| **N1** | nlhdl | `pkg/` — all eight: `VectorParams`, `VecBundles`, `VecTrace`, **`VtypeTable`** (`new`); `MicroOp`, `ScalarOpConstants`, `BoomCoreParams`, `BoomConfigMixins` (`edit_existing`, each with `<|begin_edit_scope|>`). |
| **N2** | nlhdl | `vec/decode/` — `VecDecode`, `VDecode`, `VLSDecode`, `VsetDecode`, `VConfigUnit`; plus `host/DecodeUnit` (`edit_existing`). |
| **N3** | nlhdl | `vec/rename/` (4), `vec/regfile/` (`VecRegFile`, **`VecRegFileBank`**, `VlRegFile`), `vec/issue/` (`VecIssueUnit`, `VecIssueSlot`, **`VecGroupReady`**, **`VecStoreDgenPath`**). |
| **N4** | nlhdl | `vec/VecPipeline`; `host/BoomCore`, `host/Rob`, `host/ALUUnit`, `host/ALUExeUnit`, `host/FpPipeline` (all `edit_existing`). |
| **N5** | nlhdl | All **17** `vec/lsu/` specs — including `VecCrossLsuSnoop`, **`VecStoreForward`**, **`VecOrderHold`** and `VecSquashUnit`, whose RTL does not land until Phases G and E8 — plus `host/LSU` (`edit_existing`). |
| **N6** | nlhdl | `vec/cii/` (**7** specs) + `sv/tt_cii_host_wrap` (`new`) + `sv/tt_cii_caracal_pkg` (`edit_existing`, one added enum value). |
| **N7** | nlhdl | Sweep: `inspect-hierarchy` clean, every `hierarchy.yaml` node has a source file, every file cites its `.rst` anchor and every requirement ID allocated to it. |

**N5 is the phase's centre of gravity** — the LSU is where v2's redesign lives, and its 17 specs
must be written and read as a set against `loadstore.rst`. Note that `VecCrossLsuSnoop` is
authored here even though its RTL is Phase G: under the old plan its spec had no authoring step
at all, which is exactly the gap that design-wide-first closes. `VecGroupCopy` and `VecOrderHold`
are the same gap one level deeper — they had no *node*, let alone an authoring step.

### Phase R — Review the complete set (Stage 2)

| Step | Kind | Scope |
|---|---|---|
| **R1** | review | Every spec against its `.rst`. Disagreement means the `.rst` wins, or the `.rst` is fixed first. |
| **R2** | review | Every `hierarchy.yaml` edge read from both sides — same handshake, widths, back-pressure direction. |
| **R3** | review | Design-wide invariants across all files at once: ground rule 6, the `vrf-ports` partition, the `pvtmp` rendezvous, `usingRVV` gating of every baseline delta. |
| **R4** | review | Requirement coverage: every live req in `src/main/nlhdl/reqs/` **cited by the spec of the node it is allocated to**. Stage 0 already fixed *which* node owes what and ledgered the 97 that are out of scope; R4 checks the specs actually discharge their share, and it is the step that turns allocation into citation. Artifact: the citation map. |

**Phase R is a hard gate on all of Phase A–H.** It may iterate; it may not be skipped for a
subsystem "to unblock" its RTL.

### Phase A — Baseline, scaffolding, packages

| Step | Kind | Scope |
|---|---|---|
| **A0** | measure | Baseline measurement per [§7](#7-performance-targets). Artifact: `docs_caracal/v2-baseline.md`. **Blocks gate (h) everywhere; do this first — it needs no nlhdl and may run during Phase N.** |
| **A1** | reference | **Gate (f) re-baselined reference** per [§6a](#6a-gate-f-is-a-bounded-exception-not-bit-identity-decision-d1) and decision D1. Artifact: `docs_caracal/v2-rebaseline/`. Two halves — `./regen.sh prebaseline` needs no nlhdl and **may run during Phase N** (done); `./regen.sh rebaseline` runs **immediately after A2** and the `--pre`/`--post` diff between them is what discharges D1. **Blocks gate (f) everywhere.** |
| **A2** | Chisel | Generate the four `new` Caracal packages `VectorParams`, `VecBundles`, `VecTrace`, **`VtypeTable`** (specs from **N1**) + apply the baseline package deltas. Add Chipyard configs `MediumBoomV4VectorConfig`, `MegaBoomV4VectorConfig`. |

**A1 is what makes gate (f) mean something.** A0 and A1 are the same shape — a checked-in
artifact that a later gate compares against, produced before the code it judges exists. The
`prebaseline`↔`rebaseline` diff must report *every* difference as the enumerated encoding
widths and nothing else; that single check is the evidence that the unavoidable `RT_*`/`IQ_SZ`
widening is bounded. Every later vectors-off build is then checked against `rebaseline`.
See `docs_caracal/v2-rebaseline/README.md` for what the check does and does not prove — in
particular it does **not** discharge A23 (don't-care `IQ_V_*` routing bits elaborate
bit-identically and still mis-route).

*Discovered while generating the A1 anchor, and it blocks all of Phase A:* the working tree
does not compile. `generators/chipyard/.../BoomConfigs.scala` is committed against the M1/M2
boom (`9f67941b`, which has `src/main/scala/v4/vec/`) and calls `WithVector`,
`boom.v4.vec.common.VectorParams`, `enableVectorArith`, `dcacheArbiterMode` and
`WithBoomDebugHarness` — none of which exist on the v2 branch `Caracal/addvector2`
(`2d7cf02e`). `WithBoomDebugHarness` is mixed into the *plain* V4 configs too, so **no** config
currently elaborates, vector or not. A1's `regen.sh` works around this by neutralizing those
mixins temporarily; **A2 must fix it for real**, since it is A2 that re-adds the Chipyard
vector configs.

**Package notes** (authored in N1, generated in A2).
`MicroOp` gains `is_vec`; `lvs1/lvs2/lvs3/lvd/lvm`; `pvs1/pvs2/pvs3/pvdest/stale_pvdest/pvm/pvtmp/pvl`
+ busy bits; `v_eew/v_emul/v_seg_nf`; the `vconfig` snapshot; `is_shared`; `is_vl_producer`.
**`pvs3` and `stale_pvdest` are separate fields and must stay separate** — `pvs3` is an
explicitly encoded third source, `stale_pvdest` is the group that held the destination arch
vreg before this `OP.v` renamed it. They coincide for RMW arithmetic (`vfmacc`) and diverge for
masked non-RMW ops, `vslideup`, and `vcompress`. **No** `pvtype` (VTYPE is not renamed), **no**
`stale_pvl`, **no** `vl_is_known`.
`ScalarOpConstants` widens `dst_rtype`/`lrs*_rtype` 2b → 3b for `RT_VEC = 4` and appends
`IQ_V_LOAD`/`IQ_V_STORE`/`IQ_V_ALU` (`IQ_SZ` 4 → 7).
`BoomCoreParams` adds `enableVector`, `vector: Option[VectorParams]`, the derived sizes, and
`override def hasV = enableVector`. *Bug from M1: `hasV`'s default `vLen>=128 && eLen>=64 &&
vfLen>=64` is false because `vfLen=0`, so `misa.V` was not advertised; the Whisper reference
`boom.json` misa reset must match (keep `V`, drop the spurious `X`).*

### Phase B — Decode and the VCFG mirror

| Step | Kind | Scope |
|---|---|---|
| **B2** | Chisel | Generate all five (specs from **N2**); apply the `DecodeUnit` delta (`v_legal` gate, vector-field merge, `vsetvl` marked **both** `is_unique` and `flush_on_commit`, explicit vector-CSR writes marked `flush_on_commit`). |

**Notes.** `VDecode` is single-cycle combinational, one uOP per instruction, no cracking.
`VLSDecode` produces the static access descriptor (`v_eew`, `mop`, `nf`, unit-stride / strided /
indexed / segment / whole-register / mask flags) — this descriptor is what selects
`VecRangeAgen` vs `VecElemAgen` downstream. `VsetDecode` splits three ways: `vsetivli` is
**front-end only** (VCFG computes VL at decode; the VL-RF write happens in the **rename** cycle
where `pvl` is allocated — no busy bit, `pvl` born ready, ROB entry dispatched non-busy);
`vsetvli`/`vsetvl` execute on the integer ALU EU. `vsetvli rd=x0, rs1=x0` is the reserved
keep-VL case, not `AVL=0`.
`VConfigUnit` owns exactly two things: the speculative decode-stage `vtype` mirror
(branch-snapshotted per `br_tag`) and the **committed shadow** it is recovered from on any
flush. It mirrors `vtype` only, never `vl`. **There is no execute-time mirror write** —
recovery is committed-shadow + `flush_on_commit` refetch, because `is_unique` alone does not
order the mirror against younger decode (`core.scala:739-740`). A `vill`-setting `vset` must
**poison** the mirror so younger `vtype`-dependent uOPs trap at decode rather than allocating a
mis-sized PRN group. The LMUL table is constrained by `VLMAX ≥ 1`.

### Phase C — Rename, register files, issue

| Step | Kind | Scope |
|---|---|---|
| **C2** | Chisel | Generate the rename space and its three internals (specs from **N3**). |
| **C3** | Chisel | Generate `VecRegFile` (96 PRNs, **9R/3W**, per-byte write mask) + **`VecRegFileBank`** ×4 (VLEN/4 = 64b of flops each, own decoder per port, single-cycle read with write-forwarding) and `VlRegFile`. |
| **C4** | Chisel | Generate `VecIssueUnit` ×3 + `VecIssueSlot` + **`VecGroupReady`** ×5 per slot + **`VecStoreDgenPath`** (store slots only). |

**Notes.**
`VecRenameSpace` is **one definition, two instances** — BOOM already does this for INT and FP
(`AbstractRenameStage` + parameterized `RenameFreeList`/`RenameBusyTable`/`MapTable`), whereas
`addvector` hand-wrote two spaces and `VlRename` came to 405 lines reimplementing map + free +
busy + wakeup + commit for a space with one architectural register. Parameters
`{numArchRegs, maxGroupSize, numPhysRegs, freeDiscipline, wakeupKind}`; the last two stay
**explicit and named** because the two instances genuinely differ:

| Instance | `numArchRegs` | `maxGroupSize` | `freeDiscipline` | `wakeupKind` |
|---|---|---|---|---|
| `vec_rename` | 32 | 8 | `stale_group` — frees the whole `stale_pvdest` group at commit | `group_done` (member-PRN vector) |
| `vl_rename` | 1 | 1 | `committed_ptr` — frees the outgoing committed pointer; no `stale_pvl` exists | `ready_bit` (value read at execute) |

Deriving those from group size would silently couple two unrelated design decisions.

> **⚠ Bug to not re-introduce (M1 free-list double-free).** Vector rename must allocate on
> `dis_fire` off **`ren2_uops`**, in lockstep with the scalar `RenameStage`'s registered
> ren1→ren2 pipeline. `addvector` drove it combinationally from `dec_uops`, one cycle *ahead*,
> so at dispatch the vec fields reflected the next cycle's (bubble) uop and two ops freed PRN 0.
> The `vec_pipeline_io` bundle names these ports `ren2_uops`/`dis_fire` precisely so that wiring
> `dec_uops` is visibly wrong at the connection site.
>
> Also: `rob_unsafe` must be cleared for vec ops, or the PNR assert at `rob.scala:436-442`
> trips — but **that is NOT this step's to do, and must not be a tie-off in the rename stage.**
> It is owned by the **`Rob` delta at D3** via `io.vec_clr_unsafe`, cleared by ONE group-safe
> event when the last element address has been checked, never per sub-access
> (`Rob.nlhdl.scala` part 5). `VecRenameSpace` has no port that could carry it. Ownership
> clarified 2026-08-10 after this bullet's placement inside the rename warning caused it to be
> briefed as a C2 obligation.

> **⚠ Top timing risk.** Atomic group rename performs up to `coreWidth * 8` PRN allocations per
> cycle. If the vector side loses, the whole core's rename stage pays.
>
> **AMENDED 2026-08-10 (owner's decision): the timing spike is WAIVED and C2 does not gate on
> it.** The original text required a spike on `VecRenameSpace` before C2 merged. There is no
> synthesis or STA tool in this environment (`dc_shell`, `genus`, `yosys`, `opensta` all
> absent), so the check was not executable as written; the owner's direction is not to worry
> about timing. **The risk is not thereby retired — it is accepted and unmeasured**, which is
> the honest description. Recorded here rather than dropped so that a later frequency problem
> is traceable to a decision instead of looking like an oversight. If a synthesis flow becomes
> available, this is the first thing to point it at.

`VecMapTable` carries **no** LMUL tag table and **no** whole-group checker — atomic group rename
makes every read whole-group by construction, so a checker can only confirm what the mapper
already guarantees, at the cost of 32×2b of state and a comparator tree in the critical path.
`VecFreeList` allocation is **all-or-nothing** for `pvdest`+`pvtmp`. Capacity note: with 96 PRNs
and 32 permanently held by the committed map table (the RMT maps all 32 arch vregs at all times
regardless of current LMUL), `(96−32)/8 = 8` `LMUL=8` groups in flight, and a segmented op takes
two groups so at most 4 — tighter than 128's 12, so **watch the free-list stall rate** in (e2).
`VecRegFile` port partition is the canonical `midcore.rst` `vrf-ports` table; both coprocessor
figures are *derived* from `tt_cii_caracal_pkg` (`CII_NUM_SRC_REQ = 4` → `R5`–`R8`;
`CII_NUM_DST_WB = 1` → `W2`), not chosen here. Per-byte write mask (`vLen/8` bits), not per 64b
lane — required so a sub-lane write (`vlm`'s `ceil(vl/8)` bytes, a partial tail) leaves the rest
undisturbed.
`VecIssueUnit` is one definition, three instances, all age-ordered collapsing; only `IQ_V_ALU`
sets `pnrGate`. Nothing in it may consult a `busy` from `VecLsu` (rule 6).

### Phase D — `vset` execution and the first end-to-end vector behaviour

| Step | Kind | Scope |
|---|---|---|
| **D2** | Chisel | Generate `VecPipeline` (specs from **N4**) (container only — LSU and CII tied off). Apply the `BoomCore` delta: one gated instantiation + one bundle connect. **Budget ≤ 200 added lines.** |
| **D3** | Chisel | Apply the `Rob`, `ALUUnit`, `ALUExeUnit`, `FpPipeline` deltas. **Gate (e1) `vset` smoke must pass here.** |

**Notes.** `ALUUnit` executes `vsetvli`/`vsetvl` under `usingRVV`: computes VL (and VTYPE for
`vsetvl`) and, for these uops, targets the **VL RF** and drives the VL wakeup network. Two
destinations in two rename spaces (`pdst` in INT RF, `pvl` in VL RF), hence `is_vl_producer` —
`dst_rtype` is single-valued and reads `RT_ZERO` when `rd == x0` while the VL RF is still
written.

> **⚠ Bug to not re-introduce.** Compare the **full-width** AVL against VLMAX. `addvector`
> truncated to `vecVLSz+1` bits, so a large AVL wrapped instead of saturating (AVL=2048 → vl=0),
> breaking the canonical strip-mining idiom where AVL is the remaining count.

`Rob` gains `vec_clr_bsy` (an `RT_VEC` load has no iresp writeback to clear `rob_bsy`),
**`vec_clr_unsafe`**, and the 3-bit `dst_rtype`. **No per-entry group completion counter.**

`vec_clr_unsafe` was missing from this list until 2026-08-10 and is **not optional**: without
it a vector op never clears `rob_unsafe` and the PNR assert at `rob.scala:436-442` trips.
`Rob.nlhdl.scala` part 5 specifies it as **one group-safe event, never per sub-access** —
`io.vec_clr_unsafe: Input(Valid(UInt(robAddrSz.W)))`, raised when the last element address has
been checked. The `Rob` spec also declares `vec_rob_flags` and `com_vxsat`; D3 should take its
scope from that spec's own port list rather than from this paragraph, which has now been wrong
once.

### Phase E — The vector LSU (the performance work)

The largest phase and the reason v2 exists. **N5 wrote every LSU nlhdl file before any LSU
Chisel** — the fill/drain seam has to be settled across the whole subsystem before
implementation, because that seam is what rule 6 depends on.

| Step | Kind | Scope |
|---|---|---|
| **E2** | Chisel | `VecElemQueue` (×6) + `VecQueueReservation`. The decoupling substrate — build it first, so nothing downstream can be written against a `busy` that does not exist. |
| **E3** | Chisel | `VecScalarOperandRead` ×2, `VecIdxGen`, `VecMaskStream`. |
| **E4** | Chisel | `VecElemAgen` ×2 (fill, SSI) and `VecRangeAgen` ×2 (fill, US). |
| **E5** | Chisel | `VecBeatExpander` ×2 (drain, coalescing) + `VecDcacheArbiter`. **P1 and P2 land here.** |
| **E6** | Chisel | `VecLoadCoalescingBuffer` + `VecDgen` + **`VecGroupCopy`**. **P5 lands here.** |
| **E7** | Chisel | `VecLsu` container + the `LSU` delta. **Gate (e2) must pass. P3, P4, P7 land here.** |
| **E8** | Chisel | `VecSquashUnit`. |

**Notes.**
`VecQueueReservation` claims capacity **in program order at dispatch**, sized to the worst-case
active element count from `EMUL`/`EEW`, releasing the surplus at execute once `VL` is known. A
vector store reserves in **both** its address and data queue or does not dispatch. This is a
correctness requirement, not tuning — the deadlock argument is in `loadstore.rst` `ssi-queues`
and liveness under opportunistic allocation would need `64 × 256 = 16384` entries. Loads have no
such failure mode; they are reserved in program order for the *other* reason, squashability.

`VecElemAgen` must carry over the inherited lookahead hazards **explicitly** — they were found
by bring-up, not design, and are not optional:
- Do not start, and do not emit, an element access whose **mask** bit is not yet staged.
- Same for the **index** entry.
- `VecIdxGen` serializes the index vector into per-element **unsigned** (zero-extended)
  offsets. *(Corrected: RVV 1.0 indexed offsets are unsigned — spike reads them as
  `uint8_t`/`uint16_t`/`uint32_t`. Signed extension would diverge against the Whisper
  cosim reference on any index with the top EEW bit set.)*
- Complete by **total bytes** (`vl << eew`), handling a partial final member.

> **⚠ Bugs to not re-introduce (all four from the M1 log).**
> **(1) Back-to-back store data corruption.** `VecDgen.num_members` was hardcoded 8, so after a
> 1-member store it streamed phantom members. Complete by total bytes.
> **(2) Shared-op operand.** `dgen_operand := Mux(uop.is_shared, uop.pvtmp, uop.pvs3)` — gating
> DGEN on `pvs3` for a shared op reads the wrong group.
> **(3) Stale scalar base (`prs1` RAW race).** The vector LS is woken speculatively, so
> `VecScalarOperandRead`'s INT-RF read can fire the same cycle the base GPR's writeback commits;
> the INT RF is a registered `Mem` read with no read-during-write bypass → stale base. It **must**
> snoop the INT writeback bus and forward on a same-cycle match. Masked in most tests by ≥2
> instructions of `la`→use slack, so it will not surface casually.
> **(4) Combinational loop.** `group_done` / `clr_rob` / the LCB beat must be driven from the
> **registered** beat, never combinationally from the incoming nop — `issue.vec_wakeup` →
> `iss_uops` → decode → AGEN → nop → `group_done` closes a loop. Finalize a fake/bypass beat one
> cycle later.

`VecDcacheArbiter` is a **rewrite**, not a port: priority round-robin with a scalar-priority
floor plus anti-starvation, gating the D$, LCAM, and TLB ports together. `addvector` appended
vector requests **last** at lowest priority with `uses_tlb=false`/`uses_lcam=false`, which is
why vector accesses were both slow and invisible to disambiguation. Element addresses go through
the DTLB like any other access — bare-physical makes cross-queue disambiguation impossible.

`VecLoadCoalescingBuffer` pre-loads inactive lanes from `stale_pvdest` on `R2` for `vta=0`/`vma=0`,
overlapping the load's memory latency, replacing the serial `sCopyRd`/`sCopyWr` prologue. Only
members that actually contain inactive lanes are pre-loaded.

**`VecGroupCopy` is the `VL = 0` case, and it is *not* the masked/tail case above.** Only an
immediate-AVL `vset` resolves VL in the front end; for every other form a `VL = 0` op reaches
issue with `pvl` resolving to 0. Its `pvdest` group is already renamed and its ROB entry cannot
commit until that group is architecturally correct — but with `VL = 0` the LSU executes no
element, so nothing would otherwise write it. With `vta = 0` it does one VLEN-wide copy per group
member from `stale_pvdest` and completes with a single group-done; with `vta = 1` there is nothing
to preserve, so it takes the complete-without-execute path and emits group-done immediately.
It borrows the Load Unit's VRF ports and adds **none**, so a **strict-priority** mux gives an
active load drain the ports unconditionally — the copy is catch-up work with no consumer waiting
on latency, so it is the correct loser. Routing the *ordinary* masked/partial-tail case through
here instead of the LCB's overlapped `R2` pre-load would resurrect exactly the serial copy
prologue v2 exists to delete.

`LSU` delta: expose the `VecLsuCoreIO` tap; route vector element addresses through the LCAM in
**both** directions; fold `vec_lsu_empty` into `fencei_rdy`
(`io.core.fencei_rdy := !stq_nonempty && io.dmem.ordered && vec_lsu_empty` — the old head-side
handshake deadlocks on younger vector stores). A vector load holds **one** LDQ entry, a store
**one** STQ entry, as the ordering + commit placeholder; cracked element accesses are `VecLsu`'s
own queues, **never** LDQ/STQ slots.

> **⚠ Bug to not re-introduce (`vse` teardown hang).** A vector-store STQ placeholder never
> executes via the normal path (addr/data never set), so `can_enq_store_execute` is false and
> `stq_execute_head` never advances past it — stranding every younger scalar store including the
> HTIF `tohost` write, so the sim runs to timeout even though the test passes. Like a fence,
> advance `stq_execute_head` when a committed+succeeded vector-store placeholder is cleared.

`VecSquashUnit` does **pointer rollback** on all six queues plus LCB invalidation by `ldq_idx`.
**Drain-and-discard is unsafe here** and this is the one place the CII's contract must not be
copied: a CII tag is opaque and is not reused until reclaimed, but a squashed vector load's
destination PRNs are recycled by the free list immediately, so a late response draining into a
stale `pvdest` corrupts whatever now owns that PRN.

### Phase F — Vector arithmetic over the CII

| Step | Kind | Scope |
|---|---|---|
| **F2** | SV | Generate `tt_cii_host_wrap.sv` (specs from **N6**); apply the `tt_cii_caracal_pkg` delta. Bind via `HasBlackBoxPath` against `src/main/sv/v4/**`. Verify the tb still passes. |
| **F3** | Chisel | `VecCiiTagTable` + `VecCiiHost` skeleton + **`VecCiiIssue`** — the issue path (`IQ_V_ALU` → Issue channel): tag allocation, the full issue packet, credit-metered `fu_types`. |
| **F4** | Chisel | `VecCiiOperandServer` — Src-Request → VRF read (`R5`–`R8`) / scalar from side-table → Src-Data, **in the exact order requests arrived** (a small ordering FIFO covers the registered 1-cycle VRF read). |
| **F5** | Chisel | `VecCiiWriteback` — beats by `dst_kind` to VRF `W2` / INT RF / FP RF — plus **`VecCiiComplete`**: accrue `vxsat`/`fflags`, and on `last` one group-done + one ROB clear + the tag free. **Gate (e4) must pass.** |
| **F6** | Chisel | `VecCiiFlush` — the drain-on-flush contract. |
| **F7** | Chisel | Segmented-LS transpose half: the `is_shared` two-half op, `pvtmp` rendezvous, and the six-step chain in `issue.rst` `shared-store-chain`. |

**Notes.** The tag is an opaque index into the host side-table (`tag → {rob_idx, pvdest_grp,
pvs*_grp, stale_pvdest_grp, pvm, scalar, dest-group size, killed}`); it is **not** the `rob_idx`
and not an architectural register. `CII_TAG_W = 4`, 16 tags.

**`VS3` and `STALE_VD` are distinct slots naming distinct groups, and the host performs no
instruction-dependent reinterpretation** — it serves whichever slot is requested straight from
the side-table. The **coprocessor** decides what to pull: one request when `pvs3` and
`stale_pvdest` coincide (RMW arithmetic), both when they differ. An earlier draft had the *host*
resolve a single `VS3` slot to `stale_pvdest` "when the instruction encodes no third source,"
which put instruction decoding in the adapter and made the diverging case unrepresentable.

`VecCiiFlush` is load-bearing and subtle:

- **Kill scope is all in-flight tags, no age comparator.** Flushes always fire at the ROB head,
  and `rob.io.flush.bits` carries no `rob_idx` to compare against. Kill-all is both correct and
  cheaper.
- **⇒ The Src-Data beat is MANDATORY.** A killed tag's Src-Request must still be answered, with
  don't-care data. It is **not** valid to ignore the request and return credit: on the Src-Data
  channel the **host is the sender** and holds no credit to return, and the VPU has no kill
  line — so a swallowed request stalls the channel for every *surviving* instruction, not just
  the killed one. Drain with don't-care beats, then suppress the four effects at writeback (no
  VRF write, no group-done, no ROB clear, no CSR side effect).

### Phase G — Memory ordering and disambiguation

| Step | Kind | Scope |
|---|---|---|
| **G1** | Chisel | `VecCrossLsuSnoop`: US **range-overlap** check (one per instruction) + SSI **per-element** search, both directions. Ordering searches are deliberately mask-**oblivious**. |
| **G2** | Chisel | ST→LD ordering via the existing `order_fail` replay path; **`VecOrderHold`** for the non-forwarding overlap (SSI→SSI, US→SSI) — BOOM's existing mem-dep predictor plus the load's existing per-load store-dependency block, released by the older store's element cursor completing, with the arbiter suppressing a held load's LCAM/D$ grant. |
| **G3** | Chisel | **`VecStoreForward`** — LD→ST forwarding out of the vector store **data** queues, qualified by the store's active byte mask, youngest matching SSI element wins, vector→vector only when both sides are unit-stride. **Gate (e3) must pass.** |
| **G4** | Chisel | Real `vleff` fault-trim + `vstart` handling. |
| **G5** | Chisel | Mega: wide vector cache port + `dual-dynamic` arbiter mode. |

**Notes.** An order-fail replay raises `MINI_EXCEPTION_MEM_ORDERING` at the ROB head, which
**can squash past-PNR coprocessor work** — this is exactly the hole `VecCiiFlush` closes, and G2
is the step that makes it reachable in practice. Do not land G2 before F6.

`vleff` is **not** `is_unique` and **not** serialized — it is an ordinary speculative vector
load and a **VL producer**: on completion it writes the final element count (full VL, or `i` if
element `i > 0` faulted) to its VL-RF destination, waking `pvl` in dependents like any `vset`.
Only an element-0 fault raises `rob_exception`. `vleff` is hot in `strlen`/`memchr` loops, so
serializing it would be a large and gratuitous cost.

### Phase H — Performance validation and sign-off

| Step | Kind | Scope |
|---|---|---|
| **H1** | measure | Full P1–P7 run against the A0 baseline. Any miss is a defect, not a tuning note. |
| **H2** | Chisel | HPM counters for the P1–P5 measurements (a `usingRVV`-only `perfEvents` EventSet; vector-off configs keep the original 3 sets, bit-identical). HPM counters are **not** cosim-comparable — validation is by elaboration + wiring + bit-identical. |
| **H3** | docs | Fill the as-built section of this plan per phase: bugs found, deviations, known gaps. |
| **H4** | review | Confirm rule 6 by inspection: **grep the whole `vec/lsu` tree for any `busy`-style output reaching an issue unit.** Structural check, independent of measured performance. |

---

## 9. Non-goals (deferred beyond v2)

- Misaligned unit-stride (needs cross-beat byte assembly the "one beat = one clean lane" LCB
  does not do).
- MSHR-style per-op walker contexts for strided/indexed throughput. Revisit only if H1 shows
  strided/indexed as the bottleneck — it reintroduces a module owning per-op state, which rule 6
  forbids by default.
- VRF banking / port tuning beyond 9R/3W, unless C3's area estimate forces it.
- A memory-compare cosim hook. **Recommended follow-on and a known verification gap:** Whisper
  does not deeply compare vector-**store** memory data, so a garbage vector store passes the
  per-instruction check and is caught only by a later scalar load-back. That is how the M1
  back-to-back store-data bug was found — by accident.
- Vector crypto / bf16 / any extension beyond RVV 1.0 base + the coprocessor's advertised
  `CII_MISA` (V, M, F, D).

---

## 9a. Open spec defect: `execution.rst` still specifies the superseded AGEN units

Stage 0 found one genuine **plan-vs-corpus conflict**, and it is in the spec, not the plan.
`execution.rst` still describes the inherited bobtail AGEN structure by name, so two live
requirements mandate module *identity* that [§2](#2-what-went-wrong-and-what-v2-does-differently)
deletes:

| ID | Statement | Why it cannot be allocated |
|---|---|---|
| `spec-agen.a8` | "The design must reuse the Load/Store Packer, Load/Store Skipper and Load/Store Walker AGEN units from bobtail." | Those six modules (2110 lines) are exactly what v2 replaces with `VecElemAgen` / `VecRangeAgen` / `VecBeatExpander`, cut by pipeline position instead of by the direction × class cross-product whose duplication had already produced divergent mask support between the load and store Packers. |
| `spec-agen.c11` | "The Skipper must emit fake packets for skipped regions." | Fake packets existed so a hardcoded member walk could account for skipped elements. v2 completes by **total bytes** (`vl << eew`), so there is nothing to account for — and the requirement contradicts `spec-agen.e7`, "a masked-off element must produce no `nOP.v`", which *is* allocated. |

Everything else in the `agen` family **is** allocated, because the behaviours survive the
rename — the units moved, the obligations did not:

| `execution.rst` unit | v2 node | Requirements carried over |
|---|---|---|
| Packer (stage 2, just-in-time at the queues) | `VecBeatExpander` | `b8`, `c1`–`c3`, `c23`; `lsu.c5`, `c6`, `d6`, `j2`, `k8` |
| Skipper (mask-skip, priority encoder, power-of-2 jumps) | `VecElemAgen` + `VecMaskStream` | `c6`–`c10`, `c24`, `e1`–`e14` |
| Walker (indexed, per-element index lookahead) | `VecElemAgen` + `VecIdxGen` | `c14`, `c16`–`c20`, `c27`–`c29` |
| Unit-stride single `nOP.v` | `VecRangeAgen` | `b6`, `b7`, `e13` |
| vDGEN | `VecDgen` | all of `d*` |

Note that `spec-agen.c26` — "the AGEN generator must be selected by access class rather than by
direction" — is not merely compatible with v2, it is v2's central structural claim.

**Owner and next step:** amend `execution.rst` to describe the fill/drain split, then re-run
`/spec-to-reqs extract` so `agen.a8` and `agen.c11` are retired with a tombstone naming their
successors. Until that lands they sit in the map's `superseded-ovi-agen` ledger entry. **Do not
implement them**, and do not quietly drop them either — a ledgered conflict is reviewable, a
deleted requirement is not.

---

## 10. Cross-cutting risks

| Risk | Severity | Mitigation |
|---|---|---|
| **Rename critical path.** `coreWidth * 8` PRN allocations/cycle; if the vector side loses, the whole core's rename pays. | **High** — and now **accepted unmeasured** | ~~Timing spike on `VecRenameSpace` before C2 merges.~~ **Spike waived 2026-08-10 by the owner; no synthesis/STA tool exists here.** See the amended Phase C note. Mitigation is now: none. |
| **VRF area.** `96 × 256b` = 24 kbit of flops with 12 ports; port count now dominates the storage term (128→96 cut 8 kbit while ports went 11→12). | High | Area/timing estimate at C3 before committing to a flop-based file over latch/SRAM-banked. |
| **96 PRNs may be too tight.** 8 `LMUL=8` groups in flight, 4 segmented. | Medium | Free-list stall rate is an (e2) artifact from C2 onward; the parameter is a one-line change. |
| **`STALE_VD` needs the VPU decoder.** Cross-team, outside this plan. | Medium | Stageable — the host serves slot 6 before the VPU emits it. Raise with the VPU owner at F1. |
| **`HasBlackBoxPath` vs the Chipyard flow.** | Medium | Prove it at **F2, early**. Fallback is a checksum-guarded sync step that fails the build on drift — never a silent copy. |
| **P7's 40% is a guess** until A0 exists. | Medium | Revise after A0 as an explicit plan amendment. |
| **`edit_existing` scope creep** — 12 nodes touching baseline files, the previous attempt's worst bug site. | Medium | `<|begin_edit_scope|>` is a hard constraint; every such step reports a hunk diff + what was left untouched; `core.scala` has a ≤ 200-line budget. |
| **nlhdl-first slows early phases.** | Low | Accepted deliberately. The previous attempt's expensive bugs were integration defects that a written interface contract makes visible. |

---

## 11. File-touch summary

**Design artifacts:** `src/main/nlhdl/hierarchy.yaml` and `src/main/nlhdl/reqs/` (both done)
+ **58** `.nlhdl.*` files: `pkg/` 8, `host/` 7, `vec/` 1, `vec/decode/` 5, `vec/rename/` 4,
`vec/issue/` 4, `vec/regfile/` 3, `vec/lsu/` 17, `vec/cii/` 7, `sv/` 2.

**Generated Chisel (new):** **45** nodes under `src/main/scala/v4/vec/generated/` —
41 modules + 4 packages (`VectorParams`, `VecBundles`, `VecTrace`, `VtypeTable`).

**Requirement allocation** (the map is authoritative; see `hierarchy.yaml`):

| | Count |
|---|---|
| Live requirements in `src/main/nlhdl/reqs/` (10 families) | 1162 |
| Allocated to a node's `reqs:` | 1065 |
| In `reqs_out_of_scope:` with a reason | 97 |
| **Unallocated** | **0** |

The 97 ledgered fall into nine groups, and the two that matter for planning are
**`vpu-internal`** (26 + 1 — the coprocessor behind the CII, which v2 writes none of) and
**`vpu-decoder-cross-team`** (7 — which slots the VPU pulls, including emitting `STALE_VD`;
the risk table's cross-team item). The rest are "unchanged from BOOM v4" obligations landing
on files **no node targets** (the front end, `rename-stage.scala`, the scalar EUs, the memory
system) plus rocket-chip's `CSRFile`. Where such a requirement lands on a file that *is* an
`edit_existing` target it is **allocated**, not ledgered: that node's
`<|begin_edit_scope|>` must-not-regress list is the artifact discharging it, and gate (f) is
the check. This is why `Rob` and `LSU` carry more requirements than their line budgets suggest.

**Baseline Chisel (`edit_existing`, targeted deltas only):**

| File | Node | Budget |
|---|---|---|
| `v4/exu/core.scala` | `BoomCore` | ≤ 200 lines (was 1101) |
| `v4/lsu/lsu.scala` | `LSU` | ~350 |
| `v4/common/micro-op.scala` | `MicroOp` | ~170 |
| `v4/exu/rob.scala` | `Rob` | ~110 |
| `v4/common/parameters.scala` | `BoomCoreParams` | ~90 |
| `v4/exu/execution-units/functional-unit.scala` | `ALUUnit` | ~70 |
| `v4/common/config-mixins.scala` | `BoomConfigMixins` | ~60 |
| `v4/exu/fp-pipeline.scala` | `FpPipeline` | ~50 |
| `v4/exu/decode.scala` | `DecodeUnit` | ~40 |
| `v4/common/consts.scala` | `ScalarOpConstants` | ~30 |
| `v4/exu/execution-units/execution-unit.scala` | `ALUExeUnit` | ~10 |

**SV:** `src/main/sv/v4/generated/tt_cii_host_wrap.sv` (new);
`src/main/sv/v4/tt-cii/src/tt_cii_caracal_pkg.svh` (one enum value).

**Docs:** `docs_caracal/v2-baseline.md` (A0); the as-built section of this file (H3).

---

## 12. Implementation notes & deviations (v2 as-built)

*(To be filled during implementation, per phase, as the M1 plan's as-built section was —
bugs found and fixed, design deviations, and known gaps. Phase H3 owns this.)*

Record per phase, in addition:

| Field | Why |
|---|---|
| **Generation tier** — which model generated this phase's RTL, and whether it was escalated | [§4.6](#46-model-tiers) makes the tier a variable; a later correctness question needs to know which tier produced the code. |
| **Gate (i) failures** — spec defect vs generation defect, and the fix | Distinguishes "the cheap tier is too weak here" from "the spec was wrong", which is the signal for whether the tier policy is working. |
| **Spec defects found during Stage 3** | A defect that survived Phase R is a review escape. Counting them is how the Stage-2 checklist gets better. |

---

### Phase A — as built

**Generation tier.** All eight A2 nlhdl nodes were generated on **Sonnet**, per
[§4.6](#46-model-tiers). **No phase escalation was needed** — every failure below was a spec
defect or a stale-interface race, not a translation error. The stronger tier was used only
for work the tier policy assigns to it anyway: amending the nlhdl specs, the non-nlhdl
chipyard config repair, and the four post-generation compile fixes.

**A0** was already complete (`docs_caracal/boomv4_baseline_perf.md`). **A1's anchor half**
(`prebaseline`) was verified reproducible: `gate-f-check.py --verify` matched the checked-in
manifest on all three configs (616 / 643 / 613 modules), `selftest.sh` passed 13/13, and
boom's `src/main/scala` was confirmed byte-identical between the pinned `2d7cf02e` and the
current `ac61029e`.

#### Spec defects found during Stage 3 — seven, all review escapes from Phase R

| # | Node | Defect | Resolution |
|---|---|---|---|
| 1 | `VectorParams` / `BoomCoreParams` | `numVlWakeupPorts` was specified as `aluWidth + 1`, and two `require`s as functions of `coreWidth` / `lsuWidth` — none of which a zero-dependency `VectorParams` can see. Forced a mandatory field, which made `VectorParams()` uncompilable — and `BoomCoreParams`'s spec **requires** constructing exactly that default instance. Self-contradictory across two specs. | Both specs amended. New `BoomCoreParams` spec section **2b** owns all three, where the operands are in scope. `VectorParams` carries a `DELEGATED (A2)` note at each site. Obligations unchanged, only location. |
| 2 | `VectorParams` / `VecBundles` | `VecBundles`'s spec requires every CII width to derive from a mirror of a `tt_cii_caracal_pkg.svh` localparam and names four. Only three were mirrored (`CII_TAG_W`→`ciiTagBits`, `CII_VL_W`→derived `vecVLSz`, `CII_MEMBER_W`→`log2Ceil(maxMembers)`). **`CII_NUM_SRC_SLOTS` had no mirror at all**, so `src_reuse_hint`'s width had nothing to derive from — while the spec explicitly forbids writing it as a literal. | `ciiNumSrcSlots: Int = 4` added to `VectorParams` (spec + output) and re-exported on the trait. |
| 3 | `VecBundles` | Five `VecPipelineIO` fields (`vec_rob_flags`, `int_rf_read_req`, `int_wakeups`, `fp_wakeups`, `int_wb_snoop`) have types (`VecRobFlags`, `DecoupledReadReq`, `IntWakeupBus`, `FpWakeupBus`, `IntWbSnoop`) **defined in no spec anywhere**. `VecPipeline` and `Rob` both assert they belong in `VecBundles`; `VecBundles` never declared them. | Omitted rather than invented. **Owner: D2.** Fix the specs first, then regenerate. |
| 4 | `VecBundles` | Spec types `csr_vector` as `freechips.rocketchip.rocket.CSRVectorIO`. **No such class exists** — rocket declares the port anonymously (`new Bundle { ... }`, `rocket/CSR.scala:310`), so there is no name to reference and no way to declare the field without restating rocket's bundle and letting it drift. | Field omitted with a flagged comment. Ground rule 9 is unaffected (the state is still rocket's; only the Chisel handle is unresolved). **Owner: D2**, where `csr.io.vector` is in scope. |
| 5 | `VecBundles` | `VecCiiTagEntry` is listed in `hierarchy.yaml` as one of this package's bundles, but the spec's own dependencies section already concedes it has no declaration site and `VecCiiTagTable` claims it locally. | Not declared, matching the spec's own stated position. **Owner: F3.** |
| 6 | `MicroOp` | The edit scope's must-not-regress bullet says the `usingRVV=false` bundle is bit-identical with "identical widths" **unqualified**, which contradicts the same scope's own interface-delta table listing the three ungated `*_rtype` widenings. A literal reading forbids the one change the node exists to make. | Wording defect. Resolved per the interface-delta table + [§6a](#6a-gate-f-is-a-bounded-exception-not-bit-identity-decision-d1). Spec should read "identical except the three enumerated `*_rtype` widenings". |
| 7 | `MicroOp` | Two interface-delta entries give neither identifier nor width (the nOP.v dest-PRN and byte-offset fields), and `VecElemCursor`'s three fields have no width — unlike every other row in that table. | Named `v_split_dst_prn` / `v_split_dst_byte_off`, cursor fields sized `vecVLSz`, all documented inline. A later LCB/AGEN delta must reconcile the identifiers. |
| 8 | **this plan, [§6a](#6a-gate-f-is-a-bounded-exception-not-bit-identity-decision-d1)** | **The exception table is incomplete as a change list.** It enumerates the four *field* widths that widen, but not the decode-table don't-care literals that must track them. `decode.scala:65`'s `decode_default` pads the rs1/rs2 regtype columns with `DC(2)`. A stale `DC(2)` does not merely mis-size — `DecodeLogic` refuses to pad a `BitPat` containing don't-cares, so elaboration fails outright with `Cannot pad 'BitPat(??)' to '3' bits because it has don't cares`, **for every config, vector or not**. | `DC(2)` → `DC(3)` on both columns. §6a amended with a fifth row. |

#### Process defect: a stale-interface race between parallel generations

`BoomCoreParams` and the `VectorParams` regeneration were dispatched concurrently. The
former read `VectorParams.scala` **before** the latter rewrote it, coded against the removed
`numVlWakeupPorts` field, and produced a file that could not compile. Not a model failure —
an orchestration one.

> **Rule for later phases: never dispatch a node concurrently with a regeneration of one of
> its `depends_on:` nodes.** A dependency's interface must be settled before a dependent
> reads it. This is the RTL-generation analogue of the seam rule that motivates
> [§4.4](#44-stage-2--review-the-complete-set) step 2.

#### Deviations from the plan as written

| Deviation | Why |
|---|---|
| **`Rob`'s compact `dst_rtype` widened at A2, not D3.** | [§6a](#6a-gate-f-is-a-bounded-exception-not-bit-identity-decision-d1)'s exception table lists it as row 4, so A1's `rebaseline` cannot be generated without it. Left at 2b it would silently truncate `RT_VEC` (4) to `RT_FIX` (0) through the ROB. Minimal change: the field width and the `compactUopWidth` literal that sizes `rob_compact_uop_mem`. No other `Rob` logic touched — the rest of the `Rob` delta remains D3's. |
| **`WithBoomDebugHarness` dropped, not restored.** | It exists only on the M1/M2 boom and no Phase-N spec describes it, so re-creating it would be unspecified RTL outside A2's scope. It was mixed into the **plain** V4 configs too, which is why its absence broke every config. **Must return as a specified node before gate (e) can run at D3** — this is a real blocker on the cosim regressions, not a cleanup. |
| **M1/M2 `*VectorArithConfig` / `*VectorSnoopConfig` removed.** | They passed `enableVectorArith` / `vecScalarSnoopEnable` as `VectorParams` fields; in v2 those are `BoomCoreParams` sub-flags with their own fragments. A2's stated scope is the two configs only. Re-add by composing fragments at Phase F / G. |
| **`BoomConfigMixins` 93 lines vs ~60 budget.** | Six classes (`WithVector` + two per-tier + two sub-flag fragments) plus four `//@req-` lines. Structural, not padding. |
| **`BoomCoreParams` 93 lines vs ~90 budget.** | Within noise. |

#### A1 close-out: D1 discharged, and what it cost

`GATE (f) PASS -- every difference is the enumerated exception`, on all three tiers. Module
sets identical (613/616/643, the "no vector logic" half of D1); **Tier 1 strict clean over
537 / 538 / 557 modules with 0 differing**; 6495 allowed deltas; 0 violations.

Getting there required fixing the **checker**, not the design. Its first run reported **986
violations of which zero were design changes** — it was counting `firtool` artifacts. Four
classes were normalized (`ENABLE_INITIAL_*` randomization blocks, `_GEN_*`/`_T_*` temporaries,
`ram_<d>x<w>` module names, and packed containers of a `MicroOp`), each with a **sharpness
twin** in `selftest.sh` that must still fail — 26 assertions, all passing. The full rationale
and the two new limitations this buys are in `docs_caracal/v2-rebaseline/README.md`.

> **The instrument was the problem, and that is worth remembering.** A gate reporting 88%
> false positives is not a strict gate; it is a gate nobody will read by Phase E. Tier 1 went
> from 15 modules differing to 0 — it got *sharper*, not weaker.

#### Known gaps carried out of Phase A

- **Gate (c) is unreachable until D2, independent of A2.** `core.scala:131` hardcodes
  `BasicDispatcher`, so a vector config's three `IQ_V_*` `issueParams` entries fall through
  `core.scala:837-849` to `require(false)`. Decision **D2** (switch to `CompactingDispatcher`)
  is owned by step D2's `BoomCore` delta. `MediumBoomV4VectorConfig` and
  `MegaBoomV4VectorConfig` are therefore declared but do not yet elaborate — noted in
  `BoomConfigs.scala` itself.
- **Gate (b) `make checkstyle` is not clean, and was not clean before A2.** 32 pre-existing
  scalastyle errors on the pristine tree (trailing whitespace, tabs in `lsu.scala`, and four
  scalastyle *parser* failures on `core.scala`, `decode.scala`, `regfile.scala` and
  `parameters.scala` — the last from a trailing comma before `)` that predates Caracal).
  Verified by running scalastyle in a throwaway `HEAD` worktree and diffing the error sets
  offset-normalized: **identical**. A2 adds zero new errors. The gate as written is
  unachievable on this tree; the falsifiable form is "no new error", which holds.

---

### Phase B — as built

**Generation tier.** All six B2 nodes were generated on **Sonnet**, per
[§4.6](#46-model-tiers). **No phase escalation was needed.** Every failure below was a spec
defect, not a translation error — with one exception (the `v_legal`/`v_opcode` conflation,
gate (i) below), which was a generation defect whose root cause was an ambiguous spec
sentence and which was fixed in both places. The stronger tier was used only for amending
nlhdl specs, adjudicating defects, and the gate runs.

**What landed.** `VecDecode` (588), `VDecode` (412), `VLSDecode` (371), `VsetDecode` (374),
`VConfigUnit` (605) under `src/main/scala/v4/vec/generated/decode/`, plus the `DecodeUnit`
delta in `src/main/scala/v4/exu/decode.scala` (+56 lines vs a ~40 budget; the overage is 19
mandatory `//@req-` tag lines).

**Dispatch order.** Five nodes ran in parallel (the four leaf decoders + `DecodeUnit`, which
depends only on settled Phase-A packages and touches a disjoint file), then `VecDecode`
alone against its four children's *real* generated interfaces. Every later regeneration ran
strictly serially, per Phase A's process rule.

#### Gate results

| Gate | Result |
|---|---|
| **a** `sbt boom/compile` | **PASS** — 7 sources, warnings only |
| **b** `make checkstyle` | **No new error.** 31 errors, none naming a B2 file except `decode.scala`'s **pre-existing parser failure**. See the caveat below — this gate is *blind* on `decode.scala`, not clean. |
| **c** Chipyard vector build | **Blocked until D2**, as Phase A predicted — `core.scala:131` still hardcodes `BasicDispatcher`. Not a B2 defect. |
| **d** scalar perf w/ vector enabled | **Blocked by (c)** — needs a vector config to elaborate. |
| **e** vector regression | **N/A at B2** ((e1) starts at D3), and still blocked by the missing `WithBoomDebugHarness`. |
| **f** `usingRVV=false` vs rebaseline | **PASS** — `VERIFY OK`, all three tiers match exactly (Small 613 / Medium 616 / Mega 643 modules). The whole decode delta is invisible with vectors off. |
| **i** RTL matches spec | **PASS** after the fixes below. 97/97 req IDs tagged, mechanically verified. |
| **j** no hand edits to generated output | **One hand edit**, to `decode.scala` (an `edit_existing` target, not a generated `output:`); the spec was amended so a regeneration reproduces it. |

> **Gate (b) is blind, not clean, on this phase's largest edit.** scalastyle cannot *parse*
> `decode.scala` — that is one of the four pre-existing parser failures A2 recorded — so it
> reports no style findings for the file's contents at all. The `DecodeUnit` delta therefore
> received zero style checking. Worth fixing before a phase edits it again.

#### Gate (f) tooling defect — the gate could not have failed

`regen.sh` accepted only `prebaseline|rebaseline`, but the README specifies that every phase
after A2 re-runs the check with `--post` pointing at a **fresh** vectors-off build. The only
way to produce one was `./regen.sh rebaseline` — which **overwrites `manifest/rebaseline.json`,
the very reference the gate is judged against.** Running the gate would have silently
re-baselined it instead of failing. Added a non-destructive **`check`** mode: elaborates into
`generated-src-gatef-check` and runs `--verify … --against manifest/rebaseline.json`, treating
the reference strictly as an input and writing no manifest. *A gate that cannot fail is not a
gate — this is the same lesson as A1's 88%-false-positive checker, in the opposite direction.*

#### Spec defects found during Stage 3 — six more review escapes, plus one from Phase A

| # | Node | Defect | Resolution |
|---|---|---|---|
| 9 | **`VtypeTable`** (Phase A) | `emul()` asserted `EMUL <= maxMembers` "since a legal `vtype` cannot produce a group wider than 8". **False**: a legal `vtype` at LMUL=8 *plus* a widening op (EEW=2×SEW) gives EMUL=16, and an indexed access with EEW=64 against SEW=8 gives EMUL=8×LMUL. RVV 1.0 reserves those and Caracal traps them at decode — so the assertion fires on a machine behaving *correctly*, aborting a cosim run on every `vwadd` at LMUL=8. With no unit tests (rule 11) that abort is all an engineer would see. | Spec amended. `emul` now returns **0 for out-of-range**, a documented contract (sound because `raw` is always a power of two, so every overflow is ≡0 mod 2^`emulWidth`, and 0 is otherwise unreachable since the fractional case clamps *up* to 1). The assertion was **replaced, not deleted**, with the invariant that makes the contract sound: `result === 0 \|\| result <= maxMembers`, which fires exactly when `raw` stops being a power of two. `VDecode` already assumed this contract; `VecDecode`'s `memEmulIllegal` was extended to test it. |
| 10 | `VConfigUnit` | Section 7 mandates `VecTrace` lines but the ports section declares no `rob_idx`/`ftq_idx`/`pc_lob` — while `VecTrace`'s own doc comment names `VConfigUnit` as a caller needing exactly those. Tracing was unimplementable, contradicting ground rule 11. | Spec amended: added trace-only `dec_ftq_idx`/`dec_pc_lob` inputs. **Narrowed, not closed** — the decode-lane event now traces; the rename/mispredict/commit events still have no stage-appropriate identifier and remain omitted with inline flags. |
| 11 | `VConfigUnit` / `VsetDecode` / `VecDecode` | `VsetDecode` takes a `prev_vtype` input "from VConfigUnit"; `VConfigUnit` exported no such port. `VecDecode` reconstructed it as `dec_vconfig(w-1)` — an identity that genuinely holds for `w>=1` but **has no `w=0` case**, forcing `prev_vtype(0) := DontCare` straight into a legality comparison. | Spec amended: added `dec_prev_vconfig`, the strictly-exclusive prefix — the `scanLeft`'s value *entering* lane `w`, seeded by `vcfg_mirror` at `w=0`. Both outputs are taps of the one existing prefix network, so it costs no logic. The shifted reconstruction is now explicitly forbidden. |
| 12 | `VConfigUnit` | Should a reserved keep-VL `vsetvli` poison the mirror? `VecDecode` could not route `keep_vl_illegal` there. | **Resolved as "no port", with reasoning recorded in the spec.** The vtype such a `vsetvli` carries is itself legal (`vill` clear), so the mirror absorbs a *valid* configuration; the trap's commit-time flush restores from the committed shadow — the argument part 4 already relies on for `vsetvl`. The illegality still reaches the uOP via `dec_vec_illegal`. |
| 13 | `VDecode` | `v_uses_vs1`'s funct5 exclusion is given **twice and inconsistently**: an algebraic test (`funct6(5,2) === 0b0100` → {0x10,0x11,0x12,0x13}) and a named enumeration ({0x10,0x12,0x13,0x14}), disagreeing at both ends. | Enumeration implemented (matches the real RVV table); formula unused. **Spec still needs the contradiction removed.** |
| 14 | `VDecode` | FP widening/narrowing converts (VFUNARY0, funct6 `0x12`) fall outside the dest-EEW rule, which is bounded to funct6 0x30–0x3F. `vfwcvt.*`/`vfncvt.*` genuinely change element width, so they take a vtype-derived EMUL instead of a doubled one. | Implemented literally; **not invented**. This is a real design hole, not a wording slip: a mis-sized EMUL is a mis-sized PRN group. **Owner: unassigned — must be resolved before Phase F.** |
| 15 | `VsetDecode` | Req c9's text clears `lrs3_rtype`; **`MicroOp` has no such field** (only `frs3_en`, `micro-op.scala:154`). Separately, c9 groups `vsetivli` with `rd != x0` under "`lrs1 = rs1`, `lrs1_rtype = RT_FIX`", contradicting part 2, which says its AVL is an immediate and no register is read. | `frs3_en` cleared, `lrs3_rtype` omitted with an inline flag. The c9/part-2 contradiction resolved in favour of part 2 (`RT_X`) — taking c9 literally would make rename wait on a PRN for a bit-position that is an immediate. |

#### Gate (i) failure — one generation defect, and what it teaches

`DecodeUnit`'s scalar-lane pass-through assertion was guarded on `v_legal`, but the spec says
to guard it on *"the local RVV **opcode** predicate"*. `v_legal` is that predicate **AND**
`!io.csr_decode.vector_illegal`, so an RVV encoding executed with `mstatus.VS=Off` has
`v_legal` false while the vector decoders — which recognize from `inst` alone and cannot see
`vector_illegal` — still legitimately write its fields. The assertion would fire on a machine
trapping VS=Off *correctly*.

Root cause was the spec naming only **one** predicate while referring to **two** concepts.
Fixed in both: `decode.scala` now defines `v_opcode` and `v_legal` separately and guards the
assertion on `v_opcode`; the nlhdl now mandates two named predicates and explains which
belongs where. Per [§4.5](#45-stage-3--phased-rtl-generation-gated-per-phase) this is the
"spec defect → fix the spec" branch, so the tier was **not** escalated.

#### Process note: self-reported req coverage is not evidence

`VConfigUnit`'s first generation reported all 39 req IDs tagged. **Four were missing**
(`spec-decode.d4/.d5/.d13`, `spec-vrf.c6`) — two spec paragraphs had been silently merged,
taking their tags with them — and this surfaced only because the regeneration re-derived the
list. Coverage is now checked **mechanically** (`reqcheck.py`: read each node's `reqs:` from
`hierarchy.yaml`, grep the node's actual `output:`/`target:` file). Current state: **97/97
across all six B2 nodes.** Run it as part of gate (i) from Phase C on.

#### Known gaps carried out of Phase B

- **Defect 14 (`vfwcvt`/`vfncvt` EMUL) is unowned** and must be resolved before Phase F.
- **Defect 13's spec contradiction** is worked around in RTL but not yet removed from the spec.
- `VtypeTable.VtypeInfo` still drops `vsew`/`vlmul_sign`/`vlmul_mag`, so `VConfigUnit` and
  `VecDecode` call rocket's `VType.fromUInt` directly for the full-`VType` path. Single-sourced
  and correct, but it means `VtypeTable.decode` is not the only vtype decode site it claims to be.
- `VConfigUnit`'s rename/mispredict/commit trace events remain unimplemented (defect 10).

---

### Phase C — as built

**Generation tier.** All eleven C2/C3/C4 nodes on **Sonnet**, **no escalation**. Every failure was
a spec defect or an interface-ownership defect, not a translation error. The strong tier did the
spec amendments, the defect adjudication and the gates — the tier policy of
[§4.6](#46-model-tiers) holding for a second consecutive phase.

**What landed** — 4,997 lines across three new package directories:

| Node | Lines | Reqs | | Node | Lines | Reqs |
|---|---|---|---|---|---|---|
| `VecRenameSpace` | 868 | 33 | | `VecRegFile` | 403 | 38 |
| `VecIssueSlot` | 834 | 41 | | `VlRegFile` | 385 | 15 |
| `VecIssueUnit` | 785 | 28 | | `VecStoreDgenPath` | 373 | 16 |
| `VecBusyTable` | 545 | 25 | | `VecRegFileBank` | 316 | 10 |
| `VecFreeList` | 543 | 24 | | | | |
| `VecMapTable` | 537 | 31 | | `VecGroupReady` | 414 | 17 |

**Dispatch.** Three waves by dependency depth: 7 leaves in parallel → 3 containers in parallel →
`VecIssueUnit`. Two mid-phase interface fixes forced extra serialized rounds (below). The
Phase-A rule — never generate a node while one of its `depends_on` is being regenerated — held
throughout and was the reason those rounds were serialized rather than overlapped.

#### Gate results

| Gate | Result |
|---|---|
| **a** `sbt boom/compile` | **PASS** (39 s, warnings only). Two real Phase C errors found and fixed first — see the gate (i) section. |
| **f** vectors-off vs rebaseline | **PASS** — `VERIFY OK`, all three tiers exact (613 / 616 / 643). Phase C adds no module to a vectors-off build, as expected: none of it is instantiated until D2 wires `VecPipeline`. |
| **i** RTL matches spec | **278/278 req IDs tagged**, mechanically verified with `reqcheck.py`. **Tracing is NOT yet conformant** — see the open item below. |
| **b/c/d/e** | Unchanged from B2: (b) no new scalastyle error; (c)/(d) blocked until D2's dispatcher switch; (e) N/A until D3. |

#### Three defects that would not have compiled

Phase C's real yield was ownership defects at seams — the class the design-wide-first method is
supposed to catch, and which Phase R missed three more of.

| # | Defect | Resolution |
|---|---|---|
| 16 | **`VecMemberRdy` had no declaration site.** `VecIssueSlot`, `VecIssueUnit` and `VecPipeline` part 13 all place the single declaration in `VecBundles`; `VecRenameSpace`'s spec said "declared IN THIS FILE"; `VecBundles` declared it nowhere. Generation therefore produced two structurally identical types facing each other across one seam — `VecRenameSpace`'s `VecMemberRdy(maxGroupSize)` and `VecIssueSlot`'s local `VecIssueSlotMemberRdyShim` — and `VecIssueUnit` exists to wire `dis_member_rdy` between exactly those two. **A type error, not a subtle bug.** | `VecPipeline` part 13 had already adjudicated it ("ONE BUNDLE WITH TWO NAMES, and that is a defect, not a synonym"), so its ruling was applied rather than a new one invented: promoted to `VecBundles`, **unparameterized**, five per-member groups plus `vm_rdy`. Both generated shapes had independently converged on that exact layout, so the fix was a rename, not a redesign. `VecBundles` +47/−0; `VecRenameSpace` 893→868; `VecIssueSlot` 871→834. |
| 17 | **`VecMapReq`/`VecRemapReq` took an `lregSz` constructor parameter**, which does not compile: `BoomBundle` mixes in `HasBoomCoreParameters`, which already declares `val lregSz` (`parameters.scala:430`), and scalac demands an `override`. | Parameter **dropped**, value taken from the trait. The proof it was redundant: every call site was passing the trait's own `lregSz` into it. `override` was rejected as worse than the error — it creates a second source of truth for a width whose stated purpose is "match the `MicroOp` fields of the same names". Fixed in the spec **and** the RTL so a regeneration reproduces it. Only `pregSz`/`maxGroupSize`/`emulSz` stay parameterized; those the trait does not provide and they genuinely differ between the two instances. |
| 18 | `VecMapTable`'s spec says `VecMapReq`/`VecMapResp`/`VecRemapReq` are declared in `VecBundles`. They are not, and `VecPipeline` part 13 independently says they **stay local to their producers**, mirroring baseline BOOM. | Declared locally in `VecMapTable.scala`, following the corroborated position over the node's own spec. **Note for D3:** the `Rob` delta will need `boom.v4.vec.generated.rename`, not `boom.v4.vec.rename`. |

#### Other spec defects found

- **`VecTrace`'s API cannot serve most of this module set.** Every helper except `traceDecode`
  requires a `MicroOp`, and **six of wave 1's seven nodes deliberately exclude `MicroOp` from
  `depends_on`** — they are lookup and storage structures whose events are about a resource, not
  an instruction. Ground rule 11 requires *every* vec module to trace, so it was unsatisfiable.
  Five nodes hit it and **two incompatible workarounds appeared** (`VConfigUnit`, `VlRegFile`,
  `VecFreeList`, `VecBusyTable` omitted lines — `VecFreeList` could tag zero of three; while
  `VecRegFileBank` hand-rolled a `printf` behind the public `traceEnabled`). Spec amended with a
  **three-step ladder**: `trace*` with a uOP → **`traceId`** with a bare `rob_idx` →
  **`traceStruct`** with neither (`rob=?`), plus an explicit rule that `rob=?` is acceptable only
  when no honest answer exists, and that hand-rolled `printf`s are not the escape hatch.
  `VecBusyTable` diagnosed this itself and asked for precisely `traceId`.
- `VecFreeList` had to omit two mandated assertions needing ports only its parent has. **Both
  adopted by `VecRenameSpace`** — and it correctly refused to substitute `alloc_fire(w)` for
  `dis_fire(w)`, since the former also requires `reqs(w)` and would falsely trip on a non-vector
  dispatching lane.
- `VecGroupReady`: the spec's pseudocode names the group-done member vector `prns`; the real
  `VecGroupDone` names it `pvdest`. Coded against the real field.
- `VecStoreDgenPath` / `VecIssueSlot`: the `dgen_operand_select` trace event is specified to fire
  "on the cycle the slot is filled", but no port carries a fill pulse and edge-detection needs a
  register the module is forbidden to declare. Omitted at both levels rather than approximated.
- `VecIssueUnit`: `SaturatingCounterOH` "binds to `IssueUnitCollapsing`" is unactionable — the
  helper is a private method in another class in another package. Copied per the spec's own
  "taken verbatim" instruction.

#### The M1 free-list double-free is prevented structurally

`VecFreeList` gates allocation on an `alloc_fire(w)` **input** and explicitly declined to claim
the bug was prevented, since it cannot see which stage drives that signal — correctly locating
the obligation in its parent. `VecRenameSpace` drives it as
`freelist.io.alloc_fire(w) := ren2_alloc_fire(w)`, with
`ren2_alloc_fire(w) = dis_fire(w) && ren2_alloc_reqs(w)` built from the `ren2_uops`/`ren2_mask`/
`dis_fire` input ports — **and `dec_uops`/`dec_fire` do not exist as ports on the module at all**,
so the M1 wiring is not merely avoided, it is unavailable. The `vec_pipeline_io` naming
([§8 Phase C](#phase-c--rename-register-files-issue)) did the job it was designed for.

#### Accepted risks recorded, not silently taken

- **`VecRegFile` duplicate-write-PRN → silent OR corruption.** The bank's write/forward path is a
  mutually-exclusive OR rather than a priority mux, licensed by "two writers can never target one
  PRN". The invariant is asserted **in `VecRegFileBank` only** (the spec says do not duplicate it),
  and a Chisel `assert` is simulation/formal-only — so a rename bug that ever produced a duplicate
  write PRN would corrupt data with no hardware guard. Accepted: the `vrf-ports` partition assigns
  distinct producers structurally, and a priority mux would cost area and timing for a case that
  cannot occur if rename is correct.
- **The `VecRenameSpace` timing spike is waived** — see the amended
  [§8 Phase C](#phase-c--rename-register-files-issue) note and [§10](#10-cross-cutting-risks).
  Owner's decision, no synthesis or STA tool in this environment. The risk is **accepted and
  unmeasured**, not retired.

#### Plan corrections made during Phase C

| Correction | Why it mattered |
|---|---|
| **`VecGroupReady` is ×5 per slot, not ×4** (§8 C4 and §4's node-split rationale). | `hierarchy.yaml` always said 5 and ground rule 14 makes the map authoritative; the prose was stale. The fifth instance is `stale_pvdest`, load-bearing because the CII reads the old destination as a source. |
| **`rob_unsafe` ownership moved out of the C2 warning** to the `Rob` delta's `vec_clr_unsafe` at D3. | The bullet sat inside the rename double-free warning and was duly briefed as a C2 obligation. `VecRenameSpace` refused it, citing its own spec's part 9 — correctly, since no port here could carry it. |
| **`vec_clr_unsafe` added to §8 Phase D's `Rob` scope.** | It was **missing from D3's scope list** while `Rob.nlhdl.scala` part 5 specifies it in full. D3 would have shipped `vec_clr_bsy` plus the widened `dst_rtype`, declared itself done, and the PNR assert at `rob.scala:436-442` would have fired at gate (e1) with no obvious cause. §8 Phase D now also points D3 at the `Rob` spec's own port list rather than that paragraph. |

#### Process: self-reported req coverage is still not evidence

`VConfigUnit`'s Phase-B miss repeated in form: agents reliably *claim* full coverage.
`reqcheck.py` was run after every wave and is now the gate-(i) instrument. **278/278 across
Phase C.** Keep running it; do not accept a report's word.

#### Open items carried out of Phase C

1. **The `VecTrace` conformance sweep has NOT run.** `VecTrace.nlhdl.scala` is amended but
   `VecTrace.scala` is not regenerated, so `traceId`/`traceStruct` do not exist in Scala yet and
   roughly eight nodes still carry flagged omissions (plus `VecRegFileBank`'s hand-rolled
   `printf`, which the amendment now forbids). Ground rule 11 is therefore **not** satisfied for
   Phase C. Deferred deliberately — traces are emit-only and move no interface, so one sweep
   regenerates each node once. **Do this before Phase D**: with no unit tests, tracing is the
   only debug instrument the plan provides.
2. **`vfwcvt`/`vfncvt` EMUL (defect 14) is still unowned** — required before Phase F.
3. **`VecRangeEntry.mask` byte-vs-element granularity** is an open Phase-A `VecBundles` defect,
   surfaced during the `VecMemberRdy` promotion and left un-adjudicated. Matters at Phase E.
4. `VecIssueSlot`'s `dgen_operand_busy` dependencies-section text still disagrees with
   `VecStoreDgenPath`'s real interface.
5. **A stale M1/M2 `CommitSignals` hunk was parked, not merged**, out of `rob.scala`: it
   referenced `boom.v4.vec.rename.VecGroupDealloc`/`VecRemapReq` and a boom-local `VConfig`,
   none of which exist in v2 (the vtype snapshot is rocket's `VType`). It duplicates the D3 `Rob`
   delta. Recoverable from `git stash` (message names it) and from
   `scratchpad/stale-rob-commitsignals-m1m2.patch`. **D3 should implement the `Rob` delta from
   its nlhdl spec, not by un-parking that hunk.**

#### Debug harness

`DebugMicroOp` was ported verbatim from `Caracal/addvector` (`v4/common/micro-op.scala:30`) into
`boom.v4.common` to complete the harness restoration, whose commit referenced it without
declaring it. Kept a plain `Bundle` with explicit `Int` parameters, because `core.scala`
constructs it with literal widths to match `vsrc/core_harness_wrapper_N.v` — a BlackBox, where a
reordered or resized field is a **silent cosim mismatch, not a compile error**. addvector's
neighbouring `VConfig` and `VsetWbResp` were deliberately **not** ported: v2 carries the vtype
snapshot as rocket's `VType`, and the vset writeback path is the `ALUUnit`/`Rob` delta's at D3.
This removes the [§12 Phase A](#phase-a--as-built) blocker on gate (e); `enableDebugHarness`
defaults false and the whole harness sits under one `if (DEBUG_HARNESS)` (`core.scala:1479`),
which is why gate (f) is unaffected.

---

### Phase C addendum — the trace sweep, the cosim pipeclean, and a Phase-A latent bug

Three follow-ups closed the same day, after the Phase C commit.

#### 1. The `VecTrace` sweep — ground rule 11 now satisfied

`VecTrace` regenerated with the three-step ladder (`traceId`, `traceStruct`), then nine nodes
regenerated to use it. **`VecTrace` is no longer the constraint on ground rule 11.**

| Node | Lines | Rungs used |
|---|---|---|
| `VecRenameSpace` | 868 → 891 | 4 calls, all rung 1 (`tracePrn`/`trace`/`traceTag`) |
| `VConfigUnit` | 605 → 669 | `traceDecode`, `traceTag`, `traceStruct` ×2 |
| `VecMapTable` | 537 → 600 | `traceTag`, `traceStruct` ×2 |
| `VecFreeList` | 543 → 593 | `traceStruct` ×3 |
| `VecBusyTable` | 545 → 559 | `traceId` (vector) + `traceStruct` (VL) |
| `VecRegFile` | 403 → 452 | `traceId` / `traceStruct` per port validity |
| `VlRegFile` | 385 → 408 | `traceStruct` ×3 (writes only) |
| `VecRegFileBank` | 316 → 318 | `traceStruct`; the hand-rolled `printf` removed |

The ladder held in **both** directions, which is the evidence that it is a real discrimination and
not a licence for `rob=?`: `VecBusyTable` split its two instances (`traceId` where the wakeup
carries a `rob_idx`, `traceStruct` where the payload is a bare PRN), and **three nodes
independently moved UP a rung** on discovering that `io.brupdate.b2.uop` is a genuine `MicroOp`
(`BrResolutionInfo extends BoomBundle with HasBoomUOP`), so `recover_mispredict` uses `traceTag`
with a real `rob_idx`. No node took `rob=?` where an honest identifier existed.

Two spec corrections fell out of the sweep:

- **`VecRegFileBank`'s own nlhdl still specified the hand-rolled `printf`** ("guarded printf …
  gated on `VecTrace.traceEnabled && !reset`"), so a regeneration would have reintroduced it.
  Amended to mandate `traceStruct` and to record why hand-rolled emission is now forbidden.
- **`VlRegFile`'s `rd_commit` line is removed from the mandated set.** `R_commit` is a bare
  `addr`/`data` pair with no valid, so the line fired **every cycle** — precisely the "would emit
  every cycle" argument the same paragraph already used to exclude `R_exe`. Gating it would need an
  enable added *solely* to make a trace line emit, which the `VecTrace` spec forbids. The commit
  event stays observable from the consumer side, where a real `rob_idx` exists.

> **Rollback recovery is untraceable BY CONSTRUCTION, across the whole vector subsystem.** Four
> nodes (`VecMapTable`, `VecFreeList`, `VecRenameSpace`, `VConfigUnit`) independently omitted it,
> and the sharpest reason is not "no identifier" but **misattribution**: `rob.scala:828` enters
> `s_rollback` on `RegNext(RegNext(exception_thrown))`, two cycles after the excepting instruction
> commits, so `rob_head`/`com_uops` on the rollback cycle name whatever instruction incidentally
> sits there. Tagging the event with them would be *wrong*, not merely imprecise. Making rollback
> traceable is a **design change** — giving `rollback` an identifier at D2/D3 — not something any
> of these nodes can fix. Four agents reaching the same conclusion is a design property, not four
> shortfalls.

**Deviation worth naming:** `VecRegFile` is the one place tracing costs state — two trace-only
shadow registers (`read_req_valid_r`, `read_rob_r`, ~72 flops over 9 read ports) so the
response-cycle line can carry the request cycle's `rob_idx`. Necessary (a response is a cycle
later than its request) and it feeds no functional logic, but the `VecTrace` spec constrains only
the *helpers* and is silent on a **caller** adding state, and these flops exist even with the
plusarg off since `printf` is not dead-code-eliminated. Recorded so the precedent is deliberate.

#### 2. Defect 14 closed — `vfwcvt` EMUL

The `dest_eew` widening rule was bounded to funct6 `0x30`..`0x3F`, missing `VFUNARY0`
(funct6 `0x12`, OPFVV). Amended and regenerated:

```scala
val is_vfunary0       = funct6 === 0x12.U && funct3 === OPFVV
val is_vfwcvt         = is_vfunary0 && vs1f(4, 3) === "b01".U
val is_widening_total = is_widening || is_vfwcvt
val dest_eew          = Mux(is_widening_total, sew +& 1.U, sew)
```

**The original framing of this defect was wrong on half of it.** `vfncvt` needs **no** adjustment:
all eight forms have a SEW destination with a 2*SEW *source*, structurally identical to
`vnsrl`/`vnsra`/`vnclip`, which the spec already exempts. Only `vfwcvt` (`vs1(4,3) === 0b01`)
writes 2*SEW. The spec now lists all three `vs1(4,3)` cases explicitly so nobody "fixes" `vfncvt`
by symmetry, and notes that `VXUNARY0` shares funct6 `0x12` under **OPMVV** (`vzext`/`vsext`,
destination SEW) — two families behind one funct6, so the test must gate on funct3 too.

Why it mattered: a `vfwcvt` took LMUL instead of 2*LMUL, so the mapper allocated **half** the
destination group; under atomic group rename the upper members were never allocated, and the
coprocessor's writeback would land on PRNs owned by another architectural vreg — silent
corruption, no assertion, no trap.

#### 3. A latent Phase-A bug that NO existing gate could catch

The cosim pipeclean (`MegaBoomV4VectorConfig`, `run-binary-debug-hex`) was expected to fail at
D2's dispatcher `require(false)`. It failed **earlier**, twice, on the same latent defect:

```
java.lang.NullPointerException: Cannot invoke "VectorParams.numVecPhysRegisters()"
  because the return value of "HasVectorParams.vectorParams()" is null
```

`HasVectorParams` declares `vectorParams` **abstract**, then dereferences it in **10 eager `val`s
and one bare `require`** in the trait body. Scala runs a trait's initializers before a subclass
assigns its `val`s, and `HasBoomCoreParameters` supplies it as
`new HasVectorParams { val vectorParams = vp }` (`parameters.scala:346`) — so every one hit `null`.

> **This is the important part: neither gate (a) nor gate (f) can detect it, by construction.** It
> is not a compile error — the types are correct — and a `usingRVV = false` build never constructs
> `HasVectorParams`, so the vectors-off gate cannot reach it. It survived **Phases A, B and C**
> and would have greeted whoever started D2, *masking* the dispatcher failure behind an NPE. The
> only thing that found it was elaborating a vector config end-to-end. **Gate (c) is not merely
> "blocked until D2" — its absence is load-bearing, and this phase gate set has a real hole in it
> until D2 lands.** Consider an interim check that elaborates a vector config far enough to
> construct `BoomCoreParams`, independent of the dispatcher.

Fixed in spec and RTL: all 10 derived values are `lazy val`; the truncation check is folded inside
the `lazy val vecVLSz` it guards, because a bare `require` in the body has the identical problem —
a check hoisted out of the value it protects looks tidier and does not work. The abstract
`val vectorParams` stays a plain `val`.

#### 4. Cosim pipeclean — everything up to elaboration verified

`USE_IMAGE_WHISPER=1 make CONFIG=MegaBoomV4VectorConfig run-binary-debug-hex BINARY=…/vset_test.elf`
now reaches, and stops at, exactly the expected blocker:

```
requirement failed
  at boom.v4.exu.BoomCore.$anonfun$new$260(core.scala:848)   <- require(false)
  at boom.v4.exu.BoomCore.<init>(core.scala:838)             <- the issueParams loop
```

Verified working up to that point: the `run-binary-debug-hex` target (`common.mk:491`),
`USE_IMAGE_WHISPER=1` resolving `whisperdir` to `/chipyard/sims/whisper` (binary at
`build-Linux/whisper`), the ELF, config resolution, `sbt` compile, and Scala elaboration entered
properly. **`core.scala:132` still reads `Module(new BasicDispatcher)`** — that one line, plus the
three missing `IQ_V_*` arms in the `core.scala:838` loop, is all that stands between here and a
running cosim.

**Chipyard side:** `WithBoomDebugHarness` is now mixed into all six V4 configs
(Small/Medium/Large/Mega + both Vector). It had been *absent from every config* since A2, so the
harness existed in boom but no config could use it — the cosim command could not have worked for
that reason alone, independent of D2. The fragment self-gates (`enableDebugHarness = isVcs`,
sniffing `SIMULATOR`/`SIM_NAME`/cwd), so Verilator builds elaborate without it rather than
failing, and gate (f) is unaffected because `regen.sh` strips the mixin before building its
reference trees.

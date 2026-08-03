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
| Module map / design authority | `src/main/nlhdl/hierarchy.yaml` |
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
| `cii/VecCiiHost.scala` | — | **Decomposed** into `VecCiiHost` + TagTable / OperandServer / Writeback / Flush. |
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
| **1. Author** | Every `.nlhdl.*` file for all 49 non-blackbox nodes | `inspect-hierarchy` clean; no module left unwritten |
| **2. Review** | The complete set, read against the `.rst` specs as one artifact | Every seam agreed on both sides; every requirement traced |
| **3. Generate** | RTL, **phase by phase**, each phase gated before the next starts | Per-phase gate green ([§6](#6-verification-gates)) |

The reason to finish all authoring first is that an interface defect is only visible from both
sides at once. Writing `VecLsu`'s spec in isolation cannot reveal that `host/LSU`'s delta spec
expects a different handshake; reading the two together can, and reading all 49 together is the
only point at which a *design-wide* invariant — ground rule 6, the VRF port partition, the
`pvtmp` rendezvous — can actually be checked rather than assumed.

### 4.1 Layout

```
src/main/nlhdl/
├── hierarchy.yaml                     # the module map (DONE)
├── pkg/      MicroOp, ScalarOpConstants, BoomCoreParams, BoomConfigMixins,
│             VectorParams, VecBundles, VecTrace
├── host/     BoomCore, Rob, LSU, DecodeUnit, ALUUnit, ALUExeUnit, FpPipeline
├── vec/      VecPipeline
│   ├── decode/   VecDecode, VDecode, VLSDecode, VsetDecode, VConfigUnit
│   ├── rename/   VecRenameSpace, VecMapTable, VecFreeList, VecBusyTable
│   ├── issue/    VecIssueUnit, VecIssueSlot
│   ├── regfile/  VecRegFile, VlRegFile
│   ├── lsu/      VecLsu + 15 modules
│   └── cii/      VecCiiHost, VecCiiTagTable, VecCiiOperandServer,
│                 VecCiiWriteback, VecCiiFlush
└── sv/       tt_cii_host_wrap, tt_cii_caracal_pkg
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
| 2. Review | strongest available | Same reason, plus cross-file reasoning over 49 files. |
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
   cleanly at Small/Large/Mega too.
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
   reserved at dispatch in program order, and (b) the LCB's per-PRN assembly entries.
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
12. **End-to-end ELF tests go through VCS with the Whisper cosim sidecar — never Verilator.**
    New ELFs are appended to `sims/vcs/tests_regr/*.txt`.
13. **`hierarchy.yaml` is the design authority.** Adding a module, a VRF port, or an
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
| **f** | **`usingRVV=false` bit-identical to pre-Caracal v4.** Elaborate `MediumBoomV4Config` (vector mixin not applied); RTL diff against the pre-Caracal baseline must be empty, ignoring only `@[...]` source locators and `$error` message line numbers. |
| **g** | The step's listed artifacts pass, and its trace output shows the expected instruction flow in the (d)/(e) logs, cross-checked against Whisper. |
| **h** | **Performance gate.** The step's stated numeric target in [§7](#7-performance-targets) is met, or the step fails. New in v2. |

Additionally, for every `edit_existing` step: the report must include **a diff of the touched
hunks plus an explicit statement of what was left untouched**, and the added-line count must
be within the step's stated budget.

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

All 49 non-blackbox nodes: 37 `new`, 12 `edit_existing`. **No RTL is generated in this phase.**
Steps are grouped by subsystem only so the work is reviewable in chunks; N1–N7 must all complete
before Phase R.

| Step | Kind | Scope |
|---|---|---|
| **N1** | nlhdl | `pkg/` — all seven: `VectorParams`, `VecBundles`, `VecTrace` (`new`); `MicroOp`, `ScalarOpConstants`, `BoomCoreParams`, `BoomConfigMixins` (`edit_existing`, each with `<|begin_edit_scope|>`). |
| **N2** | nlhdl | `vec/decode/` — `VecDecode`, `VDecode`, `VLSDecode`, `VsetDecode`, `VConfigUnit`; plus `host/DecodeUnit` (`edit_existing`). |
| **N3** | nlhdl | `vec/rename/` (`VecRenameSpace`, `VecMapTable`, `VecFreeList`, `VecBusyTable`), `vec/regfile/` (`VecRegFile`, `VlRegFile`), `vec/issue/` (`VecIssueUnit`, `VecIssueSlot`). |
| **N4** | nlhdl | `vec/VecPipeline`; `host/BoomCore`, `host/Rob`, `host/ALUUnit`, `host/ALUExeUnit`, `host/FpPipeline` (all `edit_existing`). |
| **N5** | nlhdl | All 14 `vec/lsu/` specs — including `VecCrossLsuSnoop` and `VecSquashUnit`, whose RTL does not land until Phases G and E8 — plus `host/LSU` (`edit_existing`). |
| **N6** | nlhdl | `vec/cii/` (5 specs) + `sv/tt_cii_host_wrap` (`new`) + `sv/tt_cii_caracal_pkg` (`edit_existing`, one added enum value). |
| **N7** | nlhdl | Sweep: `inspect-hierarchy` clean, every `hierarchy.yaml` node has a source file, every file cites its `.rst` anchor and the requirement IDs it discharges. |

**N5 is the phase's centre of gravity** — the LSU is where v2's redesign lives, and its 15 specs
must be written and read as a set against `loadstore.rst`. Note that `VecCrossLsuSnoop` is
authored here even though its RTL is Phase G: under the old plan its spec had no authoring step
at all, which is exactly the gap that design-wide-first closes.

### Phase R — Review the complete set (Stage 2)

| Step | Kind | Scope |
|---|---|---|
| **R1** | review | Every spec against its `.rst`. Disagreement means the `.rst` wins, or the `.rst` is fixed first. |
| **R2** | review | Every `hierarchy.yaml` edge read from both sides — same handshake, widths, back-pressure direction. |
| **R3** | review | Design-wide invariants across all files at once: ground rule 6, the `vrf-ports` partition, the `pvtmp` rendezvous, `usingRVV` gating of every baseline delta. |
| **R4** | review | Requirement coverage: every live req in `src/main/nlhdl/reqs/` cited by some spec or recorded as out of v2 scope. Artifact: the citation map. |

**Phase R is a hard gate on all of Phase A–H.** It may iterate; it may not be skipped for a
subsystem "to unblock" its RTL.

### Phase A — Baseline, scaffolding, packages

| Step | Kind | Scope |
|---|---|---|
| **A0** | measure | Baseline measurement per [§7](#7-performance-targets). Artifact: `docs_caracal/v2-baseline.md`. **Blocks gate (h) everywhere; do this first — it needs no nlhdl and may run during Phase N.** |
| **A2** | Chisel | Generate the four `new`/`edit_existing` Caracal packages (specs from **N1**) + apply the baseline package deltas. Add Chipyard configs `MediumBoomV4VectorConfig`, `MegaBoomV4VectorConfig`. |

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
| **C3** | Chisel | Generate `VecRegFile` (96 PRNs, **9R/3W**, banked 4×64b, per-byte write mask, single-cycle read with write-forwarding) and `VlRegFile`. |
| **C4** | Chisel | Generate `VecIssueUnit` ×3 + `VecIssueSlot`. |

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
> `dec_uops` is visibly wrong at the connection site. Also: `rob_unsafe` must be cleared for vec
> ops, or the PNR assert at `rob.scala:438-441` trips.

> **⚠ Top timing risk.** Atomic group rename performs up to `coreWidth * 8` PRN allocations per
> cycle. **Run a timing spike on `VecRenameSpace` before C2 merges.** If the vector side loses,
> the whole core's rename stage pays. This is the design's largest unquantified risk.

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

`Rob` gains `vec_clr_bsy` (an `RT_VEC` load has no iresp writeback to clear `rob_bsy`) and the
3-bit `dst_rtype`. **No per-entry group completion counter.**

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
| **E6** | Chisel | `VecLoadCoalescingBuffer` + `VecDgen`. **P5 lands here.** |
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
- `VecIdxGen` serializes the index vector into per-element **signed** offsets.
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
| **F3** | Chisel | `VecCiiTagTable` + `VecCiiHost` skeleton; issue path (`IQ_V_ALU` → Issue channel), credit-metered. |
| **F4** | Chisel | `VecCiiOperandServer` — Src-Request → VRF read (`R5`–`R8`) / scalar from side-table → Src-Data, **in the exact order requests arrived** (a small ordering FIFO covers the registered 1-cycle VRF read). |
| **F5** | Chisel | `VecCiiWriteback` — beats by `dst_kind` to VRF `W2` / INT RF / FP RF; accrue `vxsat`/`fflags`; on `last`, one group-done. **Gate (e4) must pass.** |
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
| **G1** | Chisel | `VecCrossLsuSnoop`: US **range-overlap** check (one per instruction) + SSI **per-element** search, both directions. |
| **G2** | Chisel | ST→LD ordering via the existing `order_fail` replay path; the SSI→SSI serialization edge case. |
| **G3** | Chisel | LD→ST forwarding out of the vector store **data** queues. **Gate (e3) must pass.** |
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

## 10. Cross-cutting risks

| Risk | Severity | Mitigation |
|---|---|---|
| **Rename critical path.** `coreWidth * 8` PRN allocations/cycle; if the vector side loses, the whole core's rename pays. | **High** — the design's largest unquantified risk | Timing spike on `VecRenameSpace` **before C2 merges** (Phase C note). |
| **VRF area.** `96 × 256b` = 24 kbit of flops with 12 ports; port count now dominates the storage term (128→96 cut 8 kbit while ports went 11→12). | High | Area/timing estimate at C3 before committing to a flop-based file over latch/SRAM-banked. |
| **96 PRNs may be too tight.** 8 `LMUL=8` groups in flight, 4 segmented. | Medium | Free-list stall rate is an (e2) artifact from C2 onward; the parameter is a one-line change. |
| **`STALE_VD` needs the VPU decoder.** Cross-team, outside this plan. | Medium | Stageable — the host serves slot 6 before the VPU emits it. Raise with the VPU owner at F1. |
| **`HasBlackBoxPath` vs the Chipyard flow.** | Medium | Prove it at **F2, early**. Fallback is a checksum-guarded sync step that fails the build on drift — never a silent copy. |
| **P7's 40% is a guess** until A0 exists. | Medium | Revise after A0 as an explicit plan amendment. |
| **`edit_existing` scope creep** — 12 nodes touching baseline files, the previous attempt's worst bug site. | Medium | `<|begin_edit_scope|>` is a hard constraint; every such step reports a hunk diff + what was left untouched; `core.scala` has a ≤ 200-line budget. |
| **nlhdl-first slows early phases.** | Low | Accepted deliberately. The previous attempt's expensive bugs were integration defects that a written interface contract makes visible. |

---

## 11. File-touch summary

**Design artifacts (new):** `src/main/nlhdl/hierarchy.yaml` + **49** `.nlhdl.*` files:
`pkg/` 7, `host/` 7, `vec/` 1, `vec/decode/` 5, `vec/rename/` 4, `vec/issue/` 2,
`vec/regfile/` 2, `vec/lsu/` 14, `vec/cii/` 5, `sv/` 2.

**Generated Chisel (new):** **36** nodes under `src/main/scala/v4/vec/generated/` —
33 modules + 3 packages (`VectorParams`, `VecBundles`, `VecTrace`).

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

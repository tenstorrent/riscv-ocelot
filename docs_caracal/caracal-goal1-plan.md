# Caracal Goal 1 Implementation Plan

**Objective.** Add RVV 1.0 instruction *support* to BOOM v4 — decode, vector uop cracking, vector rename, ROB allocation, vector issue queues, vector register file, and vector load/store unit — all integrated into the BOOM out-of-order core itself (not as an OVI side-car as in `bobtail/main`). The vector ALU datapath is **deferred to Goal 2**: V-ALU issue ports are tied off; arithmetic uops are decoded, cracked, renamed, and queued, but never execute. Vector loads and stores **must** execute end-to-end.

**Reference.** Architecture diagram at `./caraval.png` (XML at `./caraval.drawio.xml`). v4 baseline documented in `docs/boom-v4-architecture.md`. Prior OVI-based vector integration on `bobtail/main` is reference material for SystemVerilog primitives to port (`src/main/resources/vsrc/vpu/`) and for one already-Chisel module (`src/main/scala/exu/ovi_wrapper/ls_decode.scala`).

**Branch strategy.** Each step below is a `Caracal/addvector/<step-slug>` feature branch that merges into `Caracal/addvector` and ultimately into `Caracal/main`. Steps are ordered so that every intermediate state compiles, elaborates, and (when `enableVector=false`) is bit-identical to baseline.

Whenever you are uncertain about implmentation details and planning ask/prompt the user.


---

## Global ground rules

1. **Feature flag.** All vector logic gated by `enableVector: Boolean = false` (new `BoomCoreParams` field). Default off — every existing config and test must remain bit-identical to pre-Caracal output.
2. **Code location.** All new Chisel under `src/main/scala/v4/vec/`. Subdirs: `decode/`, `rename/`, `issue/`, `regfile/`, `lsu/`, `common/`. Package `boom.v4.vec.{decode,rename,issue,regfile,lsu,common}`.
3. **Minimize intrusive edits.** Whenever logic *can* live in `v4/vec/` rather than in a baseline file, put it there — even if it duplicates a small amount of BOOM code. Touch baseline files only where unavoidable (`MicroOp`, `BoomCoreParams`, `config-mixins.scala`, `core.scala`, `decode.scala`, `rob.scala`, and the `IssueSlot`/`IssueUnit` bundles).
4. **Parameters.** Defaults: `VLEN = 256`, `VLMAX = 256` bits, `numVecPhysRegisters = 128`. Everything else parameterized (`numVecLoadQueueEntries`, `numVecStoreQueueEntries`, issue-queue widths/entries, `numDecodeToCrackerBufferEntries`, etc.). The default tier under test is `WithNSmallBooms ++ WithVector`, but the implementation must elaborate cleanly at Medium and Large widths too.
5. **In-order within queue, OoO across queues.** Each new `IQ_V_*` is age-ordered (collapsing) just like existing queues. Inter-queue ordering is enforced exclusively by the ROB.
6. **Precise exceptions.** Vector loads/stores must update `vstart` on element-level exceptions so the trap handler can resume mid-vector. Stores do not retire to the DCache until ROB-committed.
7. **Don't fight the existing wakeup network.** Whenever possible, treat new operands (VL, V0, vector pregs) as additional source operands on the issue slot using the *existing* preg-match wakeup network. Avoid building a parallel broadcast bus unless absolutely necessary (see Step 9).
8. **Per-step verification gate (universal).** Every step is "done" only when **all** of the following pass — this is the same checklist for every step, regardless of whether the step adds executable functionality or not:

   a. `sbt compile` clean inside `generators/boom/`.
   b. `make checkstyle` clean inside `generators/boom/`.
   c. **Base Chipyard build with vector enabled** — proves Chisel still elaborates end-to-end and catches diplomacy / parameter-ripple breakage that an isolated `sbt compile` misses:
      ```bash
      cd /root/my-chipyard/sims/vcs
      make CONFIG=MediumBoomV4VectorConfig -j$(nproc) debug
      ```
   d. **Scalar performance regression with vector enabled** — proves the scalar datapath is not broken by vector logic. Must pass at *every* step (including Steps 0, 1, 4, 6, 7, 12, 13 that add no executable vector behavior), since the entire point of `enableVector=true` Goal 1 is that scalar workloads continue to run correctly even with all vector plumbing live. Doubles as the per-step perf baseline (cycles column in the summary):
      ```bash
      cd /root/my-chipyard/sims/vcs
      ./run_regr_rvv_scalar.sh MediumBoomV4VectorConfig
      ```
   e. **Vector vset + load/store regression** — required from Step 11 onward and on any later step that can touch the vector LS datapath (V-LSU, dispatch routing into `IQ_V_LOAD`/`IQ_V_STORE`, vector rename, vector regfile, DCache arbitration). Steps before Step 11 may skip (e), but (a)–(d) and (f) still apply:
      ```bash
      cd /root/my-chipyard/sims/vcs
      ./run_regr.sh tests_regr/vset_loadstore_tests.txt MediumBoomV4VectorConfig
      ```
   f. **Baseline `enableVector=false` is bit-identical to pre-Caracal v4.** Elaborate `MediumBoomV4Config` (vector mixin not applied); RTL diff against the pre-Caracal v4 baseline must be empty.
   g. The step's listed step-specific verification artifacts pass.

   Don't merge a step branch into `Caracal/addvector` until (a)–(g) are all green.

9. **Unit-test convention — one folder per test, single shared SBT project.** Unit-level testbenches live in `generators/boom/src/tests/<TestName>/` (the `src/tests/` directory exists but is empty — pre-Caracal `src/test/` was removed; do not resurrect it). Use the in-tree Chisel testing API (`chiseltest` / `ChiselSim`) with the **Verilator** backend — no VCS, no external simulator dependencies, no waveform plumbing. Each unit test pokes a single Chisel module's bundle interface against a small golden Scala model. Keep them fast (sub-minute) and add a unit test **only when** the module under test cannot be exercised through the e2e regressions in (8d)/(8e). Prefer extending the e2e suite over writing a unit test.

   **Layout.** Each unit test gets its own folder containing exactly two files: a `<TestName>Spec.scala` and a `run.sh`. The Scala package inside the spec file lives under `boom.v4.vec.<area>` (matching the module under test); the filesystem path is intentionally flat and decoupled from the package — one folder = one test = one run-script entry point. Example:
   ```
   generators/boom/src/tests/
     VDecode/
       VDecodeSpec.scala         # package boom.v4.vec.decode
       run.sh                    # entry point — see template below
     VecUopCracker/
       VecUopCrackerSpec.scala   # package boom.v4.vec.decode
       run.sh
     VecRegFile/
       VecRegFileSpec.scala      # package boom.v4.vec.regfile
       run.sh
     ...
   ```

   **Build wiring (do once, in `build.sbt` / `build.sc`).** Register `src/tests/` as an additional test source root so SBT/Mill discovers every `*Spec.scala` automatically. Dependencies (`chiseltest`, `rocketchip`, Chisel itself) come from the existing top-level project — **do not** add a per-folder build file. The benefit of the per-folder layout is *operational* (each test is one cd + one script), not build-system isolation.

   **Per-test `run.sh` template.** Each folder ships an identical 4-line script differing only in the FQN:
   ```bash
   #!/usr/bin/env bash
   # Run this unit test in isolation. Invokes the shared SBT project.
   set -euo pipefail
   cd "$(git rev-parse --show-toplevel)/generators/boom"
   exec sbt -batch "Test/testOnly boom.v4.vec.<area>.<TestName>Spec"
   ```
   Make it executable (`chmod +x run.sh`) when the folder is created. From CI or from `sbt test` at the boom root, every spec runs together as usual — the per-folder scripts are for human "what does this single test do" workflows.

10. **End-to-end ELF-test convention.** Every test that runs a real RISC-V ELF (`riscv-tests`, `tests/rvv/{bringup_tests,segment_tests,ovi_lsgen_tests,kernels}`, etc.) **must** go through VCS with the Whisper cosim sidecar — never Verilator. The two regression scripts referenced in (8d) and (8e) are the only sanctioned ELF harnesses; new ELFs are added by appending to the appropriate `sims/vcs/tests_regr/*.txt` list file. Whisper cosim catches functional divergence per-instruction; Verilator-only ELF runs would skip that check and are forbidden.

---

## Step 0 — Project scaffolding and feature flag

**Scope.** Create the empty directory structure, the feature flag, and the config mixin shell so subsequent steps have a home. No functional behavior.

**Files created.**
- `src/main/scala/v4/vec/common/VectorParams.scala` —
  ```scala
  case class VectorParams(
    vLen: Int = 256,
    numVecPhysRegisters: Int = 128,         // bump after perf tuning (issue 7)
    numVecLoadQueueEntries: Int = 64,       // up from 32 — single LMUL=8 NF=8 inst should fit (issue 8)
    numVecStoreQueueEntries: Int = 64,
    numDecodeToCrackerBufferEntries: Int = 4,  // small — cracker is now wide (issue 5)
    crackerWidth: Int = 8,                  // up to 8 sub-uops/cycle per lane (issue 5)
    vecIssueInsertWidth: Int = 8,           // queue write-port count, matches crackerWidth
    vecIssueGrantWidth: Int = 1,            // FU-pipeline-in-order: 1 on Medium, lift to 2 on Mega
    dcacheArbiterMode: String = "single",   // "single" (Goal 1 default) | "dual-dynamic" (Mega + cross-LSU snoop)
    vecScalarSnoopEnable: Boolean = false,  // turn on with dual-dynamic arbiter (issue 9)
    mshrAllocPolicy: String = "fair-floor"  // "fair-floor" (default) | "hard-partition" | "fcfs" (issue 10)
  )
  ```
  Every field is parametrizable; defaults above are Medium-tier Goal 1. Mega tier overrides `vecIssueGrantWidth=2`, `dcacheArbiterMode="dual-dynamic"`, `vecScalarSnoopEnable=true`.
- `src/main/scala/v4/vec/common/package.scala` — package object declaring `boom.v4.vec.common`.
- `src/main/scala/v4/vec/{decode,rename,issue,regfile,lsu}/.keep` — placeholders.

**Files modified.**
- `src/main/scala/v4/common/parameters.scala` — add `enableVector: Boolean = false` and `vector: Option[VectorParams] = None` to `BoomCoreParams`. Add derived `usingVector` to `HasBoomCoreParameters`. **Do not** alter any `require()` yet.
- `src/main/scala/v4/common/config-mixins.scala` — add `class WithVector extends Config(...)` that sets `enableVector = true` and `vector = Some(VectorParams())`. Add `WithNSmallBoomsVector` convenience composition. Mixin order is `new WithNSmallBooms ++ new WithVector` (chipyard precedence is right-to-left; since `WithNSmallBooms` doesn't touch any vector field, the order is functionally irrelevant — stated for clarity).

**Verification.** Per-step gate (8a–f) applies. Step-specific (8g):
- Also add the Chipyard config name `MediumBoomV4VectorConfig` in `chipyard/src/main/scala/config/BoomConfigs.scala` (= `MediumBoomV4Config + WithVector`) and `MegaBoomV4VectorConfig` (= `MegaBoomV4Config + WithVector` with the Mega overrides above) so subsequent steps' (8c)/(8d)/(8e) commands have valid CONFIGs. No new unit tests — the feature flag is exercised by the (8c) build itself.

---

## Step 1 — `MicroOp` vector field extension

**Scope.** Extend the `MicroOp` bundle so every downstream stage can see vector-related state. No logic changes; all new fields default to inert (zero / `false.B`) at decode for scalar uops.

**Files modified.**
- `src/main/scala/v4/common/micro-op.scala`:
  - Add `is_vec: Bool` (top-level flag — gates every downstream "is this a vector uop?" check).
  - Add `RT_VEC` register-type constant in `consts.scala`. **Widen `*_rtype` fields from 2 to 3 bits** (`dst_rtype`, `lrs1_rtype`, `lrs2_rtype`, `lrs3_rtype`). This is the single most invasive bundle change.
  - Add vector logical regs: `lvs1`, `lvs2`, `lvs3`, `lvd`, `lvm` (V0 mask, always `0.U` when masked).
  - Add vector physical regs (post-rename): `pvs1`, `pvs2`, `pvs3`, `pvdest`, `stale_pvdest`, `pvm` (mask preg), `pvl` (the integer preg holding VL when not statically known). Companion busy bits: `pvs*_busy`, `pvm_busy`, `pvl_busy`.
  - Add `VConfig` sub-bundle: `vstart`, `vl`, `vlmax`, `vsew (UInt(3.W))`, `vlmul (UInt(3.W))`, `vma`, `vta`, `vxrm`, `vxsat`. The full `vcsr` snapshot lives here.
  - Add `v_eew (UInt(3.W))`, `v_emul (UInt(3.W))` — effective element width/mul (differs from `vsew`/`vlmul` for widening/narrowing ops).
  - Add cracker bookkeeping: `v_split_first`, `v_split_last`, `v_split_idx (UInt(log2Ceil(9).W))`, `v_split_total (UInt(log2Ceil(9).W))`. Width is 4 bits to hold counts 0..8 inclusive (Issue 5 — RVV 1.0 constrains `EMUL × NF ≤ 8`, so max sub-uops per arch inst = 8 and `v_split_total` ∈ {1..8}).
  - Add `vsetvl_id: UInt(3.W)` — monotonic 3-bit id stamped on every vector uop at decode-time of its governing `vsetvl`/`vsetvli`/`vsetivli`. Used by the cracker-broadcast match logic (Issue 4) and by issue slots to resolve VL from the right writeback (Issue 6). 3 bits = 8 outstanding vset-class uops, plenty for Goal 1.
  - Add segment LS fields: `v_seg_nf (UInt(3.W))`, `v_seg_idx`.
  - Add `vl_is_known: Bool` — true when VL is determined at decode (`vsetivli` or const after broadcast); false when waiting on a renamed scalar.
- `src/main/scala/v4/common/consts.scala` — `RT_VEC = 3.U(3.W)`; widen the `dst_rtype` Mux constants.

**Verification.** Per-step gate (8a–f) applies — (8d) is the load-bearing check here: the `*_rtype` widening must not break any scalar issue/wakeup path. Step-specific (8g):
- One small unit test at `generators/boom/src/tests/MicroOpWidth/` (`MicroOpWidthSpec.scala` + `run.sh`, package `boom.v4.common`, per rule 9): instantiate `MicroOp()`, check `getWidth` matches the expected bit total after adding the new vec fields. One-shot width assertion, not a behavioral test.

**Risks.** Widening `*_rtype` from 2 to 3 bits is a ripple change. Audit every `*_rtype === RT_…` and `*_rtype =/= RT_X` site in `decode.scala`, `rename-stage.scala`, `dispatch.scala`, `rob.scala`, the issue-units, and `core.scala`. Most will be untouched by widening, but a few `UInt(2.W)` casts will need to become `UInt(3.W)`.

---

## Step 2 — RVV 1.0 decode tables

**Scope.** Recognize RVV 1.0 instructions in `decode.scala`. Populate the new `MicroOp` vector fields. Handle `vsetvli`/`vsetivli`/`vsetvl` semantics. No cracker yet — for now decode emits a single uop per instruction.

**Files modified.**
- `src/main/scala/v4/exu/decode.scala` — gate behind `usingVector`: add new tables.
- New file `src/main/scala/v4/vec/decode/VDecode.scala` — vector decode table: vector arithmetic ops (set `iq_type = IQ_V_ALU`, `is_vec = true`), vector loads (`IQ_V_LOAD`), vector stores (`IQ_V_STORE`).
- New file `src/main/scala/v4/vec/decode/VLSDecode.scala` — vector load/store table with unit-stride, strided, indexed (ordered + unordered), segment, fault-only-first variants. Encodes `mop`, `lumop`, `nf`, `width` into `v_eew`/`v_emul`/`v_seg_nf`.
- New file `src/main/scala/v4/vec/decode/VsetDecode.scala` — handles all three vset-class instructions. Every vset-class uop is stamped with a monotonic `vsetvl_id` (3 bits) at decode for downstream matching (Issue 4 / Issue 6).
  - **`vsetivli rd, uimm, vtypei`** — all immediate. Set `vl_is_known = true`; fill `VConfig` directly from the immediates. Cracker can crack downstream uops on the same cycle.
  - **`vsetvli rd, rs1, vtypei`** — vtype from immediate, VL from renamed `rs1`. Vtype is known at decode → cracker can crack downstream uops immediately. VL is delivered to vector issue slots via the int-writeback-data tap (Issue 6 / Step 9); downstream vector uops carry `pvl_busy=true` until then. **Four sub-cases of vsetvli** must all be decoded (spec section 6.1):
    1. `rs1 ≠ x0 ∧ rd ≠ x0` → VL := min(rs1, VLMAX); write rd. The "full" form.
    2. `rs1 ≠ x0 ∧ rd == x0` → VL := min(rs1, VLMAX); suppress rd write. Decoder still allocates an int writeback for the VL value (issue-slot tap reads from the writeback bus before the regfile write is suppressed — or write to a "discarded pdst" and tap data en-route, identical broadcast semantics).
    3. `rs1 == x0 ∧ rd ≠ x0` → VL := VLMAX (constant). VL is statically known; decode sets `pvl_busy = false` and pre-loads `vl_captured` (via the issue slot's init path) with VLMAX. rd is written with VLMAX.
    4. `rs1 == x0 ∧ rd == x0` → **keep VL unchanged**. Critical idiom for "change SEW/LMUL without touching VL" in autovectorized loops. Vtype is updated in the cracker mirror + architectural CSR (Issue 7); VL is left untouched. `pvl_busy = false`; `vl_captured` pre-loaded from the cracker's current VL mirror at decode (which downstream uops will use). rd is suppressed.
  - **`vsetvl rd, rs1, rs2`** — vtype from renamed `rs2`, VL from renamed `rs1`. **This is the only case that parks the cracker.** Decode emits **two micro-ops** (Issue 1):
    1. A **scalar uop** that flows normally through sRRU → dispatch → scalar issue queue → scalar EU. Reads `rs1` (VL) and `rs2` (vtype) from the int regfile, computes new VL and vtype, writes rd (= new VL) via standard int writeback, and drives a **dedicated cracker-broadcast bus** (Issue 2) with `(vsetvl_id, vtype, vl)` on its execute cycle.
    2. A **cracker sentinel** that stays in the decode→cracker buffer as a barrier. Carries the `vsetvl_id` of its sibling scalar uop. The cracker cannot advance past the sentinel until the cracker-broadcast bus delivers a match on `vsetvl_id` (Issue 4 below). On match, sentinel is consumed, cracker latches the new vtype/vl, and cracking resumes.

**Cracker-broadcast bus (Issue 2).** A dedicated bus from the scalar EU(s) that execute vset-class uops directly to the cracker, parallel to (not part of) the int wakeup network. Bundle: `(valid: Bool, vsetvl_id: UInt(3.W), vtype: UInt(~12.W), vl: UInt(log2Ceil(vlMax+1).W))`. Fires the cycle a vset-class scalar uop completes execute. **Two consumers:** (a) the cracker (matches against the sentinel at its buffer head); (b) every vector issue slot's VL-value-capture register (so slot's `vl_captured` is filled in lockstep — see also Issue 6, which keeps the int-writeback-data tap as the primary path for `vsetvli`'s VL since `vsetvli` doesn't go through the sentinel mechanism). For `vsetvl`, the bus is the sole VL/vtype delivery path.

**Out-of-order vsetvl broadcasts (Issue 4).** Multiple vsetvl uops in flight may execute OoO; broadcasts can arrive at the cracker in non-program order. Cracker holds a **`next_expected_id`** pointer at the head of its sentinel-list and a small **ordered side buffer** keyed by `vsetvl_id` (~4 entries — bounded by max outstanding vset-class uops, set by the 3-bit id). On broadcast arrival: if id matches `next_expected_id`, apply immediately and advance the pointer; if id is ahead, store the (vtype, vl) in the side buffer and apply when its turn comes; if id is stale (killed by a branch mispredict), discard.

**Stall location summary.**
- `vsetivli` / `vsetvli`: cracker never parks — vtype is from immediate, available at decode.
- `vsetvl`: cracker parks at the sentinel until the matching cracker-broadcast arrives. Decoder back-pressures *transitively* only if the decode→cracker buffer fills while parked.
- Scalar uops on other decode lanes are unaffected — they bypass the cracker entirely.

**Configuration tracking.** The cracker maintains a "current architectural vtype/vl mirror." Updates:
- `vsetivli` entering the cracker → mirror updates immediately from immediates.
- `vsetvli` entering the cracker → vtype mirror updates immediately from immediate; VL mirror update depends on the sub-case (case 1/2: VL value not yet known, downstream uops carry `pvl_busy`; case 3: VL=VLMAX, mirror updates immediately; case 4: VL untouched).
- `vsetvl` sentinel consumed → mirror updates from the (vtype, vl) on the matched cracker-broadcast.
- ROB commit of any non-trapping vector inst or any vset* (Issue 8) → mirror's `vstart` field clears to 0.

Downstream vector uops being cracked snapshot the current mirror state into their per-uop `VConfig`. The mirror is **branch-snapshotted per br_tag** (Issue 3 / Step 5).

**Verification.** Per-step gate (8a–f) applies. Step-specific (8g):
- Unit test at `generators/boom/src/tests/VDecode/` (`VDecodeSpec.scala` + `run.sh`, package `boom.v4.vec.decode`, per rule 9): drive instruction bit-patterns at the decode input; check that `is_vec`, `iq_type`, `v_eew`, `v_emul`, `vl_is_known`, etc. match a golden Scala model. Required cases: `vsetivli x0, 4, e32, m2`, `vsetvli a0, a1, e16, m4`, `vsetvl a0, a1, a2`, `vle32.v v0, (a0)`, `vse64.v v8, (a0)`, `vlseg3e8.v v0, (a0)`, `vlsseg2e16.v v0, (a0), a1`, `vluxei8.v v0, (a0), v8`, `vadd.vv v0, v1, v2` (just to confirm `iq_type=IQ_V_ALU`). No e2e ELF test at this step — decode alone can't run a program; the (8c)/(8d) gate covers integration.

**Risks.** RVV decode is one of the larger opcode spaces in RISC-V; getting `v_eew`/`v_emul` right for widening/narrowing arithmetic (e.g., `vwadd.vv` has EEW=2×SEW for the destination) is the part where bugs hide. Cover widening cases explicitly in the unit test.

---

## Step 3 — Vector UOP cracker

**Scope.** A per-decode-lane cracker that expands one input vector uop into `LMUL` (or `EMUL` / `NF × EMUL`) sub-uops in **up to `crackerWidth = 8` sub-uops per cycle** (one architectural inst at LMUL ≤ 8 fully cracks in 1 cycle on Medium / Goal 1).

**Files created.**
- `src/main/scala/v4/vec/decode/VecUopCracker.scala`.

**Behavior.**
- One cracker module per decode lane. Input: `Decoupled[MicroOp]`. Output: `Vec(crackerWidth, Valid(MicroOp))` plus a single ready handshake against the downstream stage.
- Per-cycle emit: up to `crackerWidth` sub-uops (default 8). If the architectural inst needs more than `crackerWidth` sub-uops (e.g. NF=4 × LMUL=4 = 16), the cracker takes `ceil(N / crackerWidth)` cycles, emitting `crackerWidth` per cycle except possibly the last.
- Each emitted sub-uop has:
  - `lvs1`, `lvs2`, `lvs3`, `lvd` incremented by the per-sub-uop iteration index.
  - `v_split_idx = i`, `v_split_total = N`, `v_split_first = (i == 0)`, `v_split_last = (i == N-1)`.
  - All other fields copied from the source uop, including `rob_idx` — **all sub-uops of one architectural instruction share the same `rob_idx`** (the ROB allocates one entry per architectural instruction, not per cracked uop). The ROB busy-bit-counter `rob_bsy_split_count` is decremented per sub-uop completion (real writeback OR slot-squash per Step 7); ROB clears the busy bit when the counter hits zero.
- **Vtype/VL stall (per Step 2).** Cracker parks when its sentinel for an in-flight `vsetvl` sits at the head of the buffer. Resume comes from the dedicated cracker-broadcast bus (Issue 2) matching the sentinel's `vsetvl_id`. `vsetvli` / `vsetivli` do **not** park the cracker — vtype is from immediate.
- **OoO broadcast matching (Issue 4).** Cracker holds a `next_expected_id` pointer + a small (~4-entry) ordered side buffer keyed by `vsetvl_id`. Incoming broadcasts with `id == next_expected_id` apply immediately; ahead-of-order broadcasts park in the side buffer and apply in order as their turn comes; stale ids (from killed paths) are discarded. Side buffer is part of `VecUopCracker.scala`.
- Widening (`v_emul > v_lmul`): cracker emits `v_emul` sub-uops for the destination. Sources may be replicated (source EMUL differs from destination EMUL). Captured by per-operand emul-derived stride values in the uop.
- Segmented LS (`v_seg_nf > 1`): cracker emits `nf × emul` sub-uops. At NF=8 × LMUL=8 = 64 sub-uops, this takes 8 cycles at `crackerWidth=8`.
- **In-order downstream guarantee.** Sub-uops are emitted in program/element order on the output vec. Downstream stages (rename → dispatch → queues → FU) must preserve this order — see in-order constraint in Step 7 and Step 11.
- **Conservative over-emit (Goal 1, Issue 15).** Cracker always emits full LMUL (or NF×EMUL for segments) sub-uops, even when runtime VL is small. Sub-uops past VL are then squashed at the issue slot (Step 7 Case 1) — wasteful but functionally correct. **Perf-TODO for Goal 2:** when `vl_is_known` holds at crack time (vsetivli / vsetvli sub-cases 3 & 4 / vsetvli sub-cases 1 & 2 after VL broadcast arrives at the cracker), cracker can elide over-emit by emitting only `ceil(VL / elems_per_subuop)` sub-uops. Not implemented in Goal 1; comment in `VecUopCracker.scala` should mark this site.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — instantiate `Seq.fill(coreWidth)(Module(new VecUopCracker))` only when `usingVector`. Wire between the decode→cracker buffer (Step 4) and rename.

**Verification.** Per-step gate (8a–f) applies. Step-specific (8g):
- Unit test at `generators/boom/src/tests/VecUopCracker/` (`VecUopCrackerSpec.scala` + `run.sh`, package `boom.v4.vec.decode`):
  - `vle32.v v4, (a0)` at LMUL=2 → expect 2 output uops with `lvd = 4, 5`, `v_split_idx = 0, 1`, both sharing the same `rob_idx`.
  - LMUL=8 → 8 outputs over 8 cycles.
  - Widening `vwadd.vv v4, v2, v1` at LMUL=2 (EMUL_dest=4) → 4 output uops; sources stride differently from destinations.
  - Segment `vlseg3e16.v v0, (a0)` at LMUL=1 → 3 output uops (one per field), all writing to fields of `v0`/`v1`/`v2`.
  - Scalar instructions pass through unchanged in 1 cycle.
  - Branch-kill mid-crack: assert all unsent sub-uops drop when `br_mask` fires.

**Risks.** Branch-kill mid-crack: if the originating architectural instruction is mispredicted and killed while the cracker is in the middle of expansion, the cracker must drop all unsent sub-uops. Handle via the standard `br_mask` propagation — sub-uops carry the original instruction's `br_mask`.

---

## Step 4 — Decoder→Cracker buffer

**Scope.** A small per-lane FIFO between `DecodeUnit` and `VecUopCracker` to absorb the cracker's `vsetvl`-induced parks. With the 8-wide cracker (Step 3), an LMUL=8 instruction cracks in a single cycle, so the buffer only needs to hold instructions while the cracker waits on an in-flight `vsetvl` to complete. A 4-entry depth is plenty.

**Files created.**
- `src/main/scala/v4/vec/decode/DecodeToCrackerBuffer.scala` — a queue of `MicroOp`, depth `numDecodeToCrackerBufferEntries` (default 4).

**Why a separate buffer rather than relying on the FetchBuffer.** The FetchBuffer holds *fetched instructions*, not decoded uops. We don't want the cracker's back-pressure (during a `vsetvl` park) to stall the decode lane for scalar uops on adjacent lanes — scalar dispatch should be unaffected.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — when `usingVector`, insert one buffer per decode lane between `decode_units(w).io.deq` and `cracker(w).io.in`. When `!usingVector`, the buffer is bypassed entirely. Scalar uops bypass the buffer at all times (only vector uops are queued).

**Verification.** Per-step gate (8a–f) applies — (8d) scalar regression is the load-bearing check (any back-pressure regression on the scalar decode lane will surface in scalar cycle counts). Step-specific (8g):
- Unit test at `generators/boom/src/tests/DecodeToCrackerBuffer/` (`DecodeToCrackerBufferSpec.scala` + `run.sh`, package `boom.v4.vec.decode`): synthetic stress test — drive decoder with a sequence of 8 vector uops at LMUL=8 (each crack takes 8 cycles); confirm decoder can issue one per cycle until the buffer fills, then back-pressures; no uops lost; commit order matches issue order; branch-kill drains the buffer.

---

## Step 5 — Vector Rename Unit (vRRU)

**Scope.** A second rename stage that runs *after* the scalar rename stage (sRRU). All scalar register renaming (int, FP, mask source if it's scalar) happens in sRRU. The vRRU then renames vector pregs (`vs1`/`vs2`/`vs3`/`vd`/`vm = V0`).

**Files created.**
- `src/main/scala/v4/vec/rename/VecRenameStage.scala` — module that wraps a `VecMapTable`, `VecFreeList`, `VecBusyTable`.
- `src/main/scala/v4/vec/rename/VecMapTable.scala` — 32 logical vector regs → physical pregs (size `numVecPhysRegisters`, default 128, parametrizable — bump after perf tuning). Branch snapshots `[maxBrCount][32]`. The V0 mask register is logical reg 0 in this table — it gets the same maptable treatment as any other vector reg.
- `src/main/scala/v4/vec/rename/VecFreeList.scala` — bitvector of free vector pregs; per-branch allocation lists for branch-mispredict reclaim. **Also holds `store_pending: Vec(numVecPhysRegisters, Bool())` — a 1-bit-per-pdst flag set when a vector store sub-uop has pinned the pdst as its data source (see Step 11 store-writeback semantics). The freelist's `dealloc(pdst)` path is gated: if `store_pending[pdst] == true`, dealloc is deferred until the V-StoreBuffer drain FSM clears the bit; if false, dealloc proceeds normally.** Without this, a later writer to the same arch reg can commit and free the pdst out from under an in-flight uncommitted store.
- `src/main/scala/v4/vec/rename/VecBusyTable.scala` — bitvector of in-flight vector pregs.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — instantiate `Module(new VecRenameStage)` only when `usingVector`. Pipeline order: cracker out → sRRU → (if `is_vec`) vRRU → dispatch.
  - **Dispatch-group depth alignment.** sRRU is 1 cycle. vRRU adds 1 cycle for vector uops. Insert a 1-cycle pipeline register on the *scalar bypass* path so scalar and vector uops in the same dispatch group emerge from rename in the same cycle and dispatch atomically. The ROB sees a single intact dispatch group per cycle; no ROB-allocation timing changes, no branch-snapshot split. The scalar perf cost is one extra rename latency cycle, lands mostly in branch-mispredict refill. The (8d) gate enforces functional PASS; cycle delta is reviewed manually at Step 14 sign-off (Issue 14) — `vec_rename_bubble_cycles` perf counter (Issue 16) makes the bubble's contribution attributable.
- Wire vector wakeup ports (Step 11 vector LSU writeback + Step 12 tied-off V-ALU placeholder) into `VecBusyTable`. Wire `rob.io.dealloc_vec_pregs` and the V-StoreBuffer drain's `store_pending_clear` port into the freelist.

**Mask register handling.** When a vector uop carries `lvm = 0` (the encoded `vm` bit indicates masked — read V0), vRRU looks up `map_table(0)` to get `pvm` and marks `pvm_busy` according to the busytable. When the encoded `vm` bit indicates *unmasked*, `pvm_busy = false` and `pvm` is left as don't-care; the V-LSU / V-ALU must perform a **conditional read** of `vec_regfile(pvm)` — gated on the encoded `vm` bit — so a stale `pvm` value is never read. Document this gating explicitly in the V-LSU and V-ALU read-stage code.

**Snapshotting.** vRRU consumes the *same* `br_tag` that sRRU allocates. The vector maptable snaps on the same branch-tag allocation event as the int/FP maptables; restore on `brupdate.b2.mispredict` is parallel to the existing int/FP restore.

**Cracker mirror snapshot (Issue 3).** The cracker's `(vtype, vl, vstart) mirror` (Step 2) is also snapshotted on the same `br_tag` allocation event. On `brupdate.b2.mispredict`, the cracker restores the mirror to the snapshot at the killed branch's tag in lockstep with the maptable restore. Without this, a `vsetvl` on a mispredicted path could leave the mirror corrupted for downstream uops on the surviving path. Cost: `maxBrCount` × ~24 bits of snapshot state — small. Wired identically to the maptable snapshot.

**`store_pending` is not snapshotted** — they're cleared by the V-StoreBuffer's branch-kill logic (Step 11): on `brupdate.b2.mispredict`, any SB entry whose sub-uop's `br_mask` matches the killed branch is dropped, and the freelist's `store_pending` bit for that entry's pinned pdst is cleared in the same cycle. This is the same kill machinery the scalar SQ already uses.

**Verification.** Per-step gate (8a–f) applies — (8d) scalar regression must show no scalar rename regression from the added vRRU stage. Step-specific (8g):
- Unit test at `generators/boom/src/tests/VecRenameStage/` (`VecRenameStageSpec.scala` + `run.sh`, package `boom.v4.vec.rename`):
  - Allocate vector destinations across 10 uops; confirm freelist decrement and busytable set.
  - Branch snapshot/restore: dispatch a branch + 4 vector uops; mispredict; freelist reclaims; maptable snaps back.
  - Mixed scalar+vector dispatch group: 1 scalar + 1 vector uop in same rename group → sRRU renames scalar parts of both; vRRU renames only the vector uop's vector pregs; dispatch group integrity preserved.

---

## Step 6 — ROB writeback-port extension

**Scope.** ROB now receives writebacks from vector LSU and (Goal 2) vector EU. For Goal 1 only the vector LSU produces writebacks. Extend the port count + the staling logic for vector pdsts.

**Files modified.**
- `src/main/scala/v4/exu/rob.scala`:
  - Add `numVecWbPorts` parameter (derived: 1 for the V-LSU writeback). Add them to `wb_resps`.
  - Add `stale_pvdest` tracking; on commit, drive `dealloc_vec_pregs.valid` with `stale_pvdest` for the vector freelist.
  - For cracked uops: all sub-uops share the same `rob_idx`. Add `rob_bsy_split_count[rob_idx]` (small counter) that decrements per sub-uop writeback; ROB clears `rob_bsy` only when the counter hits zero.
  - **Architectural CSR writes (Issue 7).** On commit of any vset-class uop, ROB drives a new CSR write port to update `vtype` and `vl` in the architectural CSR file. Uses the existing scalar CSR-write machinery in `csr.scala`. Ensures `csrr a0, vl` / `csrr a0, vtype` from a trap handler see correct committed values.
  - **`vstart` clear on retire (Issue 8).** On commit of any non-trapping vector inst OR any vset-class uop, ROB drives a CSR write to clear architectural `vstart` to 0 (spec section 3.7). A separate "vstart-clear pulse" signal also goes to the cracker so the cracker mirror's `vstart` field clears in lockstep — keeps the cracker and the architectural CSR consistent without an extra CSR-read in the cracker.
- `src/main/scala/v4/exu/core.scala` — wire `rob.io.dealloc_vec_pregs` into `vRRU.io.dealloc_pregs`. Wire vector-LSU writeback into `rob.io.wb_resps`. Wire ROB's new `csr_write_vtype_vl`, `csr_clear_vstart` outputs into the CSR file; wire `vstart_clear_pulse` to the cracker.

**Verification.** Per-step gate (8a–f) applies — the `require()` port-count checks are exercised by the (8c) build. Step-specific (8g):
- Unit test at `generators/boom/src/tests/RobVecWb/` (`RobVecWbSpec.scala` + `run.sh`, package `boom.v4.exu`): cracked-uop commit test — push a vector load at LMUL=4 into a minimally-stubbed ROB; confirm the ROB entry doesn't commit until all 4 sub-uop writebacks land, and `stale_pvdest` flows to `dealloc_vec_pregs` on commit. No e2e ELF coverage here — Step 11 will exercise this in the (8e) vset+LS regression.

---

## Step 7 — Vector issue queues

**Scope.** Three new issue queues: `IQ_V_LOAD`, `IQ_V_STORE`, `IQ_V_ALU`. Each is age-ordered (collapsing). Default sizing for Medium / Goal 1: each **64 entries** (`numVecLoadQueueEntries` / `numVecStoreQueueEntries` from `VectorParams`, parametrizable — bump after perf tuning). Insert is **8-wide** to absorb cracker bursts; **issue grant to the FU is strictly in-order and 1-wide** on Medium (matching the V-LSU's 1-port DCache); `2-wide` on Mega (matching the dual-port DCache + dynamic arbiter). All three queues use the same grant-width parameter `vecIssueGrantWidth`.

**Files created.**
- `src/main/scala/v4/vec/issue/VecIssueSlot.scala` — extension of `IssueSlot` adding:
  - Operand slots for `pvs1`/`pvs2`/`pvs3`/`pvm`/`pvl` plus their busy bits.
  - A **VL-value capture register** (`vl_captured: UInt(log2Ceil(vlMax+1).W)`). Two delivery paths feed it (Issue 6):
    1. **Int-writeback data tap (primary path).** Every vector issue slot subscribes to the *data lane* of every int writeback port — fanout receivers tap the wire en route from the int EU output to the int regfile write port. On `pdst` match against the slot's `pvl`, the slot latches the data into `vl_captured` and clears `pvl_busy` in the same cycle. Reuses the existing scalar writeback data path; no new bus. This handles all `vsetvli` sub-cases 1 & 2 (rs1 ≠ x0).
    2. **Cracker-broadcast bus (`vsetvl` path).** For `vsetvl` (Issue 2), the dedicated cracker-broadcast bus also drives every vector issue slot. Slot matches on `vsetvl_id` (carried in the uop's MicroOp field per Step 1) and latches `vl` from the bus into `vl_captured`.
    3. **Decode-time pre-load** (statically-known VL — `vsetivli`, `vsetvli` sub-cases 3 & 4). Slot enters with `pvl_busy = false` and `vl_captured` initialized at dispatch from the cracker's current mirror snapshot in the uop's `VConfig`.
  - A **3-case active-range comparator** computed each cycle once VL is known: given the uop's `v_split_idx`, `v_split_total`, and the snapshotted `vconfig.vstart`:
    - `elem_start = v_split_idx * elems_per_subuop` (derived from `v_eew` + `vLen`)
    - `elem_end   = elem_start + elems_per_subuop`
    - **Case 1 — fully outside** (`elem_end ≤ vstart` OR `elem_start ≥ vl_captured`) → mark this sub-uop **squash-ready**. The slot's `request` line drives a "complete-without-execute" grant instead of an FU-issue grant.
    - **Case 2 — fully inside** (`elem_start ≥ vstart` AND `elem_end ≤ vl_captured`) → issue normally to the FU.
    - **Case 3 — partial** → issue to the FU but with an `active_lo` / `active_hi` field so the AGU / V-ALU processes only the active subset. The V-LSU AGU already needs per-element bounds for masking; reuse that machinery.
- `src/main/scala/v4/vec/issue/VecIssueUnit.scala` — wrapper around the existing `IssueUnitCollapsing` parametrized for `crackerWidth` insert ports and `vecIssueGrantWidth` grant ports. The collapsing-shift logic already age-orders entries.

**Squash-grant mechanism (Case 1 above).** At grant time, when the slot fires its squash-ready path:
1. Drive the writeback bus with `valid=true, pdst=this_pdst, rob_idx=this_rob_idx, squash=true` (a new `squash` bit on the existing writeback bundle). The ROB decrements `rob_bsy_split_count[rob_idx]` the same way it would for a real writeback.
2. Broadcast `(valid, pdst)` on the wakeup network so dependents wake.
3. Free the slot (collapsing-shift consumes the entry).
4. **`vta=0` corner case (load-class arch ops only).** If `vconfig.vta == 0` AND this is a load/op that produces a vector destination, the squash path cannot leave the new pdst with garbage. Instead of pure squash, the slot issues a single **regfile-copy micro-op** `pdst ← stale_pvdest` through the dedicated copy ports added in Step 10. Completion of the copy drives the same writeback bus (with `squash=false, copy=true`). For stores (no vector destination) `vta` is irrelevant; pure squash is always safe.

**Files modified.**
- `src/main/scala/v4/common/consts.scala` — add `IQ_V_LOAD`, `IQ_V_STORE`, `IQ_V_ALU` to the `iq_type` bitfield. Widen `iq_type` from 3 to 4 bits.
- `src/main/scala/v4/common/parameters.scala`:
  - Add the three new `require(issueParams.count(_.iqType == IQ_V_*) == (if (usingVector) 1 else 0))` invariants.
  - Update `WithVector` mixin's `issueParams` to append three `IssueParams(issueWidth=vecIssueGrantWidth, numEntries=64, iqType=IQ_V_LOAD/STORE/ALU, dispatchWidth=crackerWidth)`.
- `src/main/scala/v4/exu/dispatch.scala` — the existing bitwise `iq_type & issueParam.iqType` routing extends once `iq_type` is widened. The dispatch lane width matches `crackerWidth` for vector lanes.
- `src/main/scala/v4/exu/core.scala` — instantiate the three new issue units; wire their wakeups into the integer wakeup network (for `pvl` wakeups + the VL-value broadcast) plus the new vector wakeup network. Wire the squash writeback port into the ROB busy-counter decrement path alongside the real writeback ports.

**In-order grant to FU.** The age-ordered collapsing queue grants the oldest ready entry first; with `vecIssueGrantWidth=1` on Medium, only one uop fires per cycle. The downstream V-LSU pipeline consumes uops in the order they were granted. Combined, this gives strict program-order execution of vector sub-uops within each queue — required for `vstart` precision (see Step 11). On Mega the queue grants 2/cycle in age order; the V-LSU pipeline accepts both on its two lanes, processing them in age order on lanes 0/1.

**Verification.** Per-step gate (8a–f) applies — the (8c) build proves the widened `iq_type` and three new issue units elaborate together; (8d) scalar regression catches any dispatch-routing regression that misroutes scalar uops. Step-specific (8g):
- Unit test at `generators/boom/src/tests/DispatchRouting/` (`DispatchRoutingSpec.scala` + `run.sh`, package `boom.v4.exu`): a sequence of `vle32.v / vse32.v / vadd.vv` decoded uops routes to V_LOAD / V_STORE / V_ALU respectively; scalar uops route unchanged. Cracker burst (8 sub-uops in one cycle) inserts atomically into the queue.
- Unit test at `generators/boom/src/tests/VecIssueOrder/` (`VecIssueOrderSpec.scala` + `run.sh`, package `boom.v4.vec.issue`): two V_LOAD uops with different ROB indices arrive — must be issued in age order even if their operands wake in reverse order (standard age-ordered behavior; confirm the vector slot extension didn't break it). On Medium config grant width = 1, so exactly one fires per cycle; older fires first regardless of younger's earlier readiness.
- Unit test at `generators/boom/src/tests/VecSlotSquash/` (`VecSlotSquashSpec.scala` + `run.sh`, package `boom.v4.vec.issue`): VL captured into slot, 3-case comparator drives the correct fully-outside / fully-inside / partial path; squash-grant drives the writeback bus with `squash=true` and decrements ROB busy-counter; `vta=0` triggers the regfile-copy micro-op instead.

---

## Step 8 — Vector L/S decoder (port of `ls_decode.scala`)

**Scope.** Bobtail's `src/main/scala/exu/ovi_wrapper/ls_decode.scala` is already Chisel. Port to v4 and rename to `VecLsDecode`. It receives uops issued from `IQ_V_LOAD`/`IQ_V_STORE` and finishes the LS-specific decoding (stride, segment, EEW disambiguation, address-generation parameters) since by the time the uop issues, all scalar operands (including stride, base, VL) are resolved.

**Files created.**
- `src/main/scala/v4/vec/lsu/VecLsDecode.scala` — direct port. Replace `boom.exu` package with `boom.v4.vec.lsu`; replace `boom.common._` with `boom.v4.common._`; adapt `EnhancedFuncUnitReq` (port the bundle as well, or substitute the v4 equivalent).
- `src/main/scala/v4/vec/lsu/ConfigInfo.scala` — port the `ConfigInfo` bundle from bobtail.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — instantiate one `VecLsDecode` between issue and the VLSU.

**Verification.** Per-step gate (8a–f) applies. Step-specific (8g):
- Unit test at `generators/boom/src/tests/VecLsDecode/` (`VecLsDecodeSpec.scala` + `run.sh`, package `boom.v4.vec.lsu`): parity with bobtail — drive a handful of known LS uops (unit-stride, strided, indexed, segment, masked) through both bobtail's `OviLsDecode` (snapshot the expected `ConfigInfo` once and check it in as a golden) and the new `VecLsDecode`; confirm identical `ConfigInfo` outputs.

---

## Step 9 — VL broadcast (reuse existing wakeup network)

**Scope.** Make the issue queues hold vector uops until VL is resolved.

**Design choice.** Treat VL as just another source operand on the vector issue slot, participating in the existing preg-match wakeup network. Avoid a separate `VLBroadcastManager`. Three concrete delivery paths cover all vset variants:

1. **Int-writeback data tap** (Issue 6) — primary path for `vsetvli` sub-cases 1 & 2 (rs1 ≠ x0). The slot subscribes to the data lane of every int writeback port; on `pdst` match against `pvl`, slot latches the data into `vl_captured` and clears `pvl_busy`. Reuses existing scalar writeback data path.
2. **Cracker-broadcast bus** — primary path for `vsetvl` (Issue 2). The bus drives every vector issue slot in addition to the cracker; slot matches on `vsetvl_id` and latches `vl`.
3. **Decode-time pre-load** — `vsetivli` (always) and `vsetvli` sub-cases 3 & 4 (rs1 == x0). VL is statically known; slot enters with `pvl_busy = false` and `vl_captured` initialized from `VConfig.vl` snapshotted by the cracker.

**Files modified.**
- `VecIssueSlot` (Step 7): the `pvl` operand and its `pvl_busy` bit are *part of* the slot's operand-ready mask. The slot's `request := all_ready` already gates issue; just include `pvl_ready` in the AND.
- `core.scala`: wire the int writeback data lane into every vector issue slot (fanout receivers); wire the cracker-broadcast bus to every vector issue slot in addition to the cracker.

**Verification.** Per-step gate (8a–f) applies. Step-specific (8g):
- Unit test at `generators/boom/src/tests/VLWakeup/` (`VLWakeupSpec.scala` + `run.sh`, package `boom.v4.vec.issue`): the sequence `vsetvli a0, a1, e32, m2; vle32.v v0, (a0)` enters IQ_V_LOAD; the `vle32.v` slot reports `pvl_busy=true` until the `vsetvli` writeback drives the integer-writeback broadcast, then `request` asserts. Confirm the slot also captures the *value* of VL into its `vl_captured` register on the same broadcast.
- Over-emit squash test in the same spec: cracker emits 8 sub-uops for an LMUL=8 instruction at runtime VL=2 sub-uops worth. Confirm 6 of the 8 cracked sub-uops squash at their issue slot (Case 1), broadcast their pdst wakeups, and decrement `rob_bsy_split_count` to zero on the 8th completion. Repeat with `vta=0`: the 6 over-emitted sub-uops must instead issue regfile-copy micro-ops (`pdst ← stale_pvdest`), with completion of each copy driving the same writeback bus.
- Static check (test asserts on the elaborated graph, not on behavior): no new top-level broadcast bus is introduced — the wakeup wiring counts on `IQ_V_LOAD` / `IQ_V_STORE` / `IQ_V_ALU` equal the integer writeback port count plus the vector writeback port count plus the squash-grant port count, with no extras.

---

## Step 10 — Vector register file

**Scope.** Implement the vector physical register file in Chisel, sized for `numVecPhysRegisters = 128`, width `VLEN = 256` bits. Reference `bobtail/main:src/main/resources/vsrc/vpu/tt_vec_regfile.sv`.

**Files created.**
- `src/main/scala/v4/vec/regfile/VecRegFile.scala`.

**Ports (Goal 1).** Per the deferred-data-read store model (Step 11) and the slot-squash `vta=0` copy (Step 7), the Goal 1 port budget is:
- **Write ports (2):**
  - `W0` — V-LSU load writeback (data from DCache → preg).
  - `W1` — squash-copy write port (data from `stale_pvdest` → new `pdst`, used by the slot-squash `vta=0` micro-op).
- **Read ports (2):**
  - `R0` — V-LSU AGU-time read (e.g. indexed-load index vector, store-data preview for forwarding-snoop checks).
  - `R1` — V-StoreBuffer drain read (data from preg → DCache, fires post-commit element-by-element).
- The `W1` write and the `R0` AGU read share scheduling: `W1` only fires when a slot-squash `vta=0` is granted, which is rare; AGU reads are continuous. Arbitrate `W1` over `R0` cycles where both contend, with `R0` taking priority since AGU is on the active issue path.
- Implementation: a `Mem(numVecPhysRegisters, UInt(VLEN.W))` is fine for Goal 1. Banking deferred to Goal 2 once port counts grow (V-ALU adds 2R + 1W).

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — instantiate; wire load writeback to `W0`, V-StoreBuffer drain to `R1`, AGU to `R0`, slot-squash copy data path to `R0` (read of `stale_pvdest`) feeding `W1` (write of `pdst`).

**Verification.** Per-step gate (8a–f) applies. Step-specific (8g):
- Unit test at `generators/boom/src/tests/VecRegFile/` (`VecRegFileSpec.scala` + `run.sh`, package `boom.v4.vec.regfile`): deterministic writes to a sequence of pregs (cover index 0, midrange, and `numVecPhysRegisters-1`); read back; confirm values bit-equal. Exercise both write ports (`W0`, `W1`) and both read ports (`R0`, `R1`) simultaneously; assert no read-during-write hazard on the same address (or document the explicit behavior — Goal 1 uses `Mem`, which has read-after-write same-cycle defined behavior in Chisel; pick one and pin it in the spec).

---

## Step 11 — Vector LSU (Chisel ports of bobtail VPU memory primitives)

**Scope.** A vector load/store unit that owns its own load queue, store queue, load buffer, store buffer, and address generators. References:
- `bobtail/main:src/main/resources/vsrc/vpu/tt_lq.sv` → `VecLoadQueue.scala`
- `bobtail/main:src/main/resources/vsrc/vpu/tt_store_buffer.sv` → `VecStoreBuffer.scala`
- `bobtail/main:src/main/resources/vsrc/vpu/tt_mem.sv` → `VecMemUnit.scala` (top-level vector memory pipeline)
- `bobtail/main:src/main/resources/vsrc/vpu/tt_memop_fsm.sv` → `VecMemopFsm.scala`
- `bobtail/main:src/main/resources/vsrc/vpu/tt_idxldst_fsm.sv` → `VecIdxLsFsm.scala`
- `bobtail/main:src/main/resources/vsrc/vpu/tt_mask_fsm.sv` → `VecMaskFsm.scala`

**Files created (under `src/main/scala/v4/vec/lsu/`).**
- `VecLoadQueue.scala` (`numVecLoadQueueEntries` default 64; one entry per dispatched vector load sub-uop; tracks element-level progress and `vstart` on faults).
- `VecStoreQueue.scala` (`numVecStoreQueueEntries` default 64; **metadata-only** entries per the deferred-data-read store semantics below — no VLEN-wide data payload in the entry).
- `VecLoadBuffer.scala` (line-fill / element-coalescing buffer in front of the DCache).
- `VecStoreBuffer.scala` (commit-deferred store buffer; entries carry address descriptor + pdst pointer + per-element drained bitmap, NOT data).
- `VecAgu.scala` (one for loads, one for stores — generates per-element addresses based on `ConfigInfo` from Step 8).
- `VecMemUnit.scala` (top-level vector memory pipeline; owns its lane(s) on the DCache req bus via the dynamic arbiter below).
- `CrossLsuSnoop.scala` (CAM machinery for bidirectional snoop between scalar LSU and V-LSU — see below).
- `DCacheArbiter.scala` (mode-parametrized DCache request arbiter — `single` or `dual-dynamic` per `VectorParams.dcacheArbiterMode`).

**In-order execution within the V-LSU pipeline.** Each issue queue (`IQ_V_LOAD`, `IQ_V_STORE`, `IQ_V_ALU`) issues sub-uops in age order at `vecIssueGrantWidth` per cycle (Step 7). The V-LSU pipeline consumes them strictly in that order — no sub-uop reordering downstream of issue. Writebacks to the vector regfile (`W0`) fire in age order on the queue's writeback bus. This guarantees:
- **`vstart` precision.** If sub-uop *k* of an LMUL=8 load faults, no sub-uop > *k* has already written its result; `vstart = k * elems_per_subuop + faulting_element_idx` is correct without partial-write tracking.
- **OoO across queues, in-order within.** V_LOAD and V_STORE may execute OoO with respect to each other; element-address ordering is enforced by the cross-LSU snoop + memory-dependency speculation (below), not by serializing the queues.

**Vector store sub-uop "writeback" semantics (deferred-data-read).** A store sub-uop's writeback event — the one that decrements `rob_bsy_split_count[rob_idx]`, broadcasts on the wakeup network for ordering-dependents, and frees the issue slot — fires when **all** of:
1. The AGU has produced all active element addresses for this sub-uop.
2. Every active address has been snooped against the V-LoadQueue **and the scalar LoadQueue** (cross-LSU snoop, below); any ordering violation is resolved into a replay *before* the writeback signal asserts.
3. The active element mask is finalized (VL bounds from the slot-squash logic in Step 7 + element-mask read from `pvm` if `vm=0`).
4. The data-source pdst (`pvs3`) is **pinned**: the V-StoreBuffer sets `vec_freelist.store_pending[pvs3] := true`, deferring dealloc until drain (see Step 5 freelist).

At this point the V-StoreBuffer entry holds metadata only: `(address_descriptor, pvs3_ptr, active_mask, per_element_drained_bitmap, rob_idx, br_mask)`. No vector data is read from the regfile yet.

**Drain.** After the architectural inst commits (ROB releases the entry), the V-StoreBuffer drain FSM walks the entry's active elements. For each element: read data via the vector regfile drain port (`R1`, Step 10), arbitrate for a DCache lane via the `DCacheArbiter`, issue the TileLink put, and on grant, set the element's drained bit. When the bitmap is full, the SB entry is freed AND the freelist's `store_pending[pvs3]` bit is cleared — if a deferred dealloc was pending on that pdst, the freelist now releases it.

**Bidirectional cross-LSU snoop (per the RVWMO discussion).** The Step 11 design replaces "share DCache port with arbitration" with full memory-ordering machinery:
- **Vector LoadQueue snoops scalar StoreQueue.** Each active element address checked against the scalar SQ (~32 entries). On match: if the scalar store data is present, forward to the vector LB; if not yet present (store data still being computed), stall the vector load element until the scalar store completes — same memory-dependency replay machinery scalar BOOM uses internally.
- **Scalar LoadQueue snoops vector StoreBuffer.** Each scalar load address checked against the V-SB (~64 entries). On match: read the relevant element(s) from the vector regfile via `R1` (drain port, shared) and forward; if the covering element's `drained_bit` is set, the data is already in the cache and a normal scalar load hit handles it. If the SB entry's `per_element_computed` flag for the covering element is not yet set (mask still pending), stall and replay.
- **Vector LoadQueue snoops vector StoreBuffer.** Same as the scalar-load case, intra-vector — both directions.
- **Scalar LoadQueue snoops scalar StoreQueue.** Unchanged from baseline BOOM.

**Forwarding merge point (Issue 11).** All cross-LSU snoop forwarding happens at the **V-LoadBuffer allocation stage** — the cycle the V-LB allocates an entry for a vector load element, it simultaneously fires the snoop against the scalar SQ + V-SB. Allocation decides per-element:
- **Hit, data ready** → V-LB entry marked "data ready from forward"; element is satisfied without ever issuing a DCache request. Cheapest path; saves DCache bandwidth.
- **Hit, data not ready** (covering store still computing data) → V-LB entry stalls; replay snoop next cycle.
- **Miss** → V-LB entry issues a DCache request via the arbiter; mark "awaiting DCache."

This is the same shape as scalar BOOM's LSU (which decides forward vs DCache at load-issue time). Don't perform the snoop at AGU (data may not be ready) or at writeback (DCache request already wasted).

CAM cost: per-element vector load snoop against scalar SQ is `elements_per_subuop × scalarSqEntries` comparators per cycle of grant. At LMUL=8 / SEW=8 / VLEN=256 this is 256 × 32 ≈ 8k comparators. Real but standard for OoO LSUs.

**Memory-dependency speculation across LSUs.** Vector loads may issue speculatively past scalar stores whose addresses haven't been generated yet, predicted-no-conflict by BOOM's existing memory-dependency predictor. On mispredict (older scalar store address arrives at the scalar SQ and matches an already-completed vector load): squash and replay the vector load using the existing ordering-violation path. Symmetric for scalar-load-past-vector-store.

**RVWMO fences.** `fence rw,rw` and `fence.i` must drain *both* LSUs before the fence retires:
- Block ROB commit of the fence until: scalar SQ is empty of pre-fence stores AND V-SB has drained all pre-fence committed stores AND V-LQ has no **non-killed** in-flight loads with `seqnum < fence.seqnum` (Issue 13 — exclude entries that have been squashed by `brupdate.b2.mispredict` but not yet collected).
- Implementation: add a `fence_pending` signal from the ROB to both LSUs; each LSU drives `fence_drain_done` back when its pre-fence work is clear; the ROB AND-s them. Each LSU's drain check gates on the explicit `valid && !killed && seqnum < fence.seqnum` predicate per V-LQ entry.

**DCache port arbitration — `DCacheArbiter` modes.**
- **`single` (Goal 1 default, Medium config, `lsuWidth=1`).** Single DCache request lane. Arbitrate scalar request, V-LSU AGU request, V-StoreBuffer drain request → 1 grant/cycle. Default priority: scalar > V-LSU AGU > drain (configurable; round-robin with anti-starvation also acceptable).
- **`dual-dynamic` (Mega config, `lsuWidth=2`).** Two DCache request lanes. Both scalar LSU and V-LSU drive 2 candidate requests each per cycle; the arbiter picks 2 from up to 4 candidates. Anti-starvation: each side gets ≥1 lane every N cycles when contending. **All-scalar workload → both lanes carry scalar traffic; all-vector workload → both lanes carry vector traffic.**

**MSHR allocation policy (Issue 10).** Selected via `VectorParams.mshrAllocPolicy` (parametrized — pick at elaboration time per workload class):
- **`fair-floor` (default).** MSHRs are pooled and allocated on demand, with reservation floors: scalar always has ≥`scalarMshrFloor` MSHRs free (default 2), vector always has ≥`vectorMshrFloor` MSHRs free (default 2). Soft partition — prevents starvation either direction without wasting capacity when one side is idle.
- **`hard-partition`.** Fixed split (e.g. on a 16-MSHR DCache: 8 scalar / 8 vector). Each LSU has its own pool. No contention; capacity wasted when one side idles; vector throughput capped at half regardless of scalar activity.
- **`fcfs`.** First-come-first-served, no reservation. Simplest; prone to scalar or vector starvation. Listed for completeness — don't ship with this in production.

**Files modified.**
- `src/main/scala/v4/lsu/lsu.scala` — wire scalar SQ snoop output to the cross-LSU snoop module; wire scalar LQ to receive vector-SB forwards. Drop the old "vector-as-tertiary-priority" logic — replaced by `DCacheArbiter`.
- `src/main/scala/v4/lsu/dcache.scala` — no internal changes; `DCacheArbiter` lives outside the cache and presents the same `lsu_io.dmem.req` interface. Existing `lsuWidth=2` support on Mega already gives the cache 2 lanes — Mega-only behavior simply uses both.
- `src/main/scala/v4/exu/core.scala` — instantiate `VecMemUnit`, `CrossLsuSnoop`, `DCacheArbiter`; wire issue → `VecLsDecode` → `VecMemUnit`; wire load writeback (`W0`), drain reads (`R1`), AGU reads (`R0`); wire `fence_pending`/`fence_drain_done` between ROB and both LSUs.
- `src/main/scala/v4/exu/rob.scala` — add `fence_pending` output gated on fence-class uops at the head of the ROB; AND `fence_drain_done` from scalar + vector LSUs to commit the fence.

**Verification.** Per-step gate (8a–f) applies — this is the first step where (8e) the vset + load/store regression is *required* and is the primary functional gate. Sub-step requirements:
- **11a (LQ/SQ/AGU + unit-stride, `single` arbiter mode):** the (8e) regression must PASS for the unit-stride rows `ms2_vse64`, `ms3p5_pureload`, `ms4_vle64`, `ms4p5_vle64_2`, `ms4p6_vle32_2`, `ms4p6_vle32_8`, `ms4p9_vl`, `ms4p10_vl0`. The remaining 9 rows may FAIL/ERROR — gate is `grep -c '^bringup_tests .* PASS' results.txt >= 8`. **Also:** the (8d) scalar regression with vector enabled passes (the cross-LSU snoop must not regress scalar memory throughput).
- **11b (strided + indexed):** adds `ms5p3_stride_m`, `ms5p4_index`, `ms5p5_index_mask` to the PASS set.
- **11c (segment):** adds `stress_seg`.
- **11d (mask + whole-register + mask-load + FoF):** adds `ms5p1_vse64_m`, `ms4p7_vlnr_vsnr`, `ms4p8_vlm` and the fault-only-first test (`vleff.v` — add a dedicated ELF if not already present and append to `tests_regr/vset_loadstore_tests.txt`). **Also adds three `vsetvl` coverage ELFs (Issue 12)** that must be authored before the regression can pass: `vsetvl_basic.elf` (basic r-r-r), `vsetvli_keep_vl.elf` (Issue 9 case 1 idiom), `vsetvl_back_to_back.elf` (two vsetvls in flight to exercise OoO broadcast handling from Issue 4). Author these under `tests/rvv/bringup_tests/`. After 11d the (8e) regression must be all-PASS on Medium, no exceptions.
- **11e (Mega + dual-dynamic arbiter):** swap the per-step gate's CONFIG to `MegaBoomV4VectorConfig`; (8c) build + (8d) scalar regression + (8e) vector regression must all PASS. This validates the `dual-dynamic` arbiter mode, the cross-LSU snoop under 2-lane DCache traffic, and that scalar dual-issue (`lsuWidth=2`) is not regressed when vector traffic is light. Add a hand-crafted ELF mixing scalar memory loops with vector loads to `tests_regr/` (e.g. `mixed_scalar_vector_ls.elf`) that explicitly forces address overlap between scalar SQ and vector LB — confirms the bidirectional snoop forwards correctly.

Step-specific (8g) — none of these are unit tests in isolation; they're all ELF runs handled by the VCS+Whisper harness through (8d)/(8e) per rule 10. Do **not** add Verilator-based ELF testbenches for the V-LSU; the cosim divergence check is essential here. Spike-comparison coverage that the (8e) list does not yet enumerate (e.g. negative-stride `vlse32.v` with stride=-8, strided=0 broadcast load, or scalar/vector cross-snoop scenarios) should be added as new ELFs to `tests_regr/vset_loadstore_tests.txt`, not as new unit tests.

**Risks.** This is the largest step by far. Consider splitting into 11a (LQ/SQ/AGU bring-up, unit-stride only), 11b (strided + indexed), 11c (segment), 11d (mask + fault-only-first). Treat 11a as the minimum viable for the Goal 1 deliverable; 11b–d can land before the Goal 1 closeout but are not blockers for downstream integration testing.

---

## Step 12 — Tie off `IQ_V_ALU`

**Scope.** Vector arithmetic uops are decoded, cracked, renamed, and queued in `IQ_V_ALU`, but **must not** issue (no V-ALU exists yet). Goal 2 will replace this with a real VPU attach.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — for the IQ_V_ALU issue port, drive `grant := false.B`; assert that `IQ_V_ALU.io.iss_uops(0).valid` is never raised. Or equivalently: set `issueWidth = 0` for the V_ALU queue when in Goal 1 mode (require `if (goal1Only) issueWidth == 0`). Use an `enableVectorArith` sub-flag (default off in Goal 1, on in Goal 2).
- Add an assertion (`assert(!io.iq_v_alu.iss_valid, "Goal 1: V-ALU should not issue")`) so simulations explicitly trip if a vector ALU op accidentally gets through.

**Verification.** Per-step gate (8a–f) applies — (8d) scalar regression with vector enabled is the canonical "no vector ops queued for scalar workloads" check (if a scalar test caused the tie-off assertion to fire, the run would ERROR). (8e) vset+LS regression must remain all-PASS (the tie-off must not break the LS issue ports). Step-specific (8g):
- No new unit test required. If desired, a sanity ELF that contains a single `vadd.vv` and expects an explicit assertion failure can be added under `tests_regr/`; treat that as a regression artifact, not a unit test.

---

## Step 13 — `core.scala` integration sweep

**Scope.** Final wiring sweep. By this point every individual module has been instantiated and wired piecewise across Steps 0–12. Step 13 reviews `core.scala` end-to-end and fixes any lingering inconsistencies in wakeup port counts, branch-update fan-out, commit fan-out, and require() invariants.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — final cleanup.
- `src/main/scala/v4/exu/rob.scala` — if any port-count adjustment is still needed.
- `src/main/scala/v4/common/parameters.scala` — finalize `usingVector` derived parameter, finalize require() set.

**Verification.** Per-step gate (8a–f) applies — this step is *defined* by the gate passing cleanly. Step-specific (8g):
- (8c) and (8d) and (8e) all clean — no warnings, no `WARN` rows in any regression summary.
- (8f) baseline diff strictly empty — any non-empty diff is a Step-13 bug to fix in `core.scala`, not a baseline mismatch to accept.

---

## Step 14 — Verification and sign-off

**Scope.** Sign off Goal 1 as a coherent feature.

**Tasks.** All ELF runs below go through the VCS + Whisper harness per ground rule 10; no Verilator ELF runs.
1. **Scalar baseline regression with vector off** — `MediumBoomV4Config` (vector mixin not applied) — full csmith run, Linux boot test in Chipyard. Must match pre-Caracal byte-for-byte.
2. **Scalar performance regression with vector on** — confirms the scalar datapath is functionally unaffected by the live vector plumbing:
   ```bash
   cd /root/my-chipyard/sims/vcs
   ./run_regr_rvv_scalar.sh MediumBoomV4VectorConfig
   ```
   Every kernel row must `PASS`. **Cycle delta vs (1) baseline is reviewed manually at sign-off** (Issue 14): inspect the cycles column of `results.txt` side-by-side with the baseline run; investigate any kernel whose cycles regress notably. No automated bound enforcement in Goal 1; if a perf cliff appears, the `vec_rename_bubble_cycles` counter (task 5) attributes the rename bubble's share.
3. **vset + load/store regression** — the canonical Goal-1 functional check:
   ```bash
   cd /root/my-chipyard/sims/vcs
   ./run_regr.sh tests_regr/vset_loadstore_tests.txt MediumBoomV4VectorConfig
   ```
   Every row in `regr_vset_loadstore_tests_MediumBoomV4VectorConfig/results.txt` must report `PASS`. Any `FAIL` or `ERROR` blocks sign-off.
4. **Vector ISA cosim** (VCS + Whisper; per rule 10): run the riscv-tests vector subset (`rv64uv-p-*`) at LMUL ∈ {1, 2, 4, 8}, SEW ∈ {8, 16, 32, 64}, with and without mask. Whisper is treated as the architectural reference — divergence is a Caracal bug. The arithmetic tests will fail (expected — V-ALU is tied off); the load/store tests must pass. Add the LS subset as new ELFs in `tests_regr/vset_loadstore_tests.txt` so they become part of the per-step (8e) gate going forward.
5. **Performance counters**: confirm the following are exposed via `BoomCustomCSRs` or the perf-counter infrastructure for future tuning:
   - `vec_retire_count` — vector instructions retired.
   - `vec_load_q_occ` / `vec_store_q_occ` — V-LOAD / V-STORE queue occupancy.
   - **`vec_rename_bubble_cycles`** (Issue 16) — cycles the scalar-bypass pipeline register on the vRRU bypass path (Step 5) holds a valid uop, attributable to the 1-cycle bubble added when any vector uop is in the dispatch group. Lets perf analysis directly attribute scalar regressions to the rename bubble vs other causes.
   - V-LSU MSHR allocation counters (per `mshrAllocPolicy` mode, Issue 10) — useful for choosing the right policy at perf-tuning time.
   - **TLB-pressure counters** (per `caracal-goal1-area-analysis.md` #5 — TLB is not extended in Goal 1, so vector AGU traffic shares the scalar TLB ports). Split per source so the cause of any scalar-TLB-miss spike is attributable:
     - `tlb_miss_scalar` — TLB misses on scalar memops.
     - `tlb_miss_vec` — TLB misses on vector AGU lookups.
     - `tlb_port_stall_scalar_blocked_by_vec` — cycles a scalar memop waited for a TLB port held by a vector AGU lookup (or vice versa).
   - Without these, a (8d) scalar regression caused by vector strided/indexed loads stealing TLB ports would be invisible to root-cause analysis. The V-TLB mitigation is deferred to Goal 2; the *measurement* lands in Goal 1.
6. **Documentation update**: refresh `docs/boom-v4-architecture.md` to reflect the new vec pipeline (or write `docs/caracal-vec-architecture.md` as a sister doc).

**Acceptance criteria for Goal 1 sign-off.**
- `./run_regr.sh tests_regr/vset_loadstore_tests.txt MediumBoomV4VectorConfig` is all-PASS.
- `./run_regr_rvv_scalar.sh MediumBoomV4VectorConfig` is all-PASS; cycle delta vs vector-off baseline reviewed manually at sign-off, no automated bound (Issue 14).
- All vector load/store ISA tests pass spike comparison at LMUL ≤ 8.
- Cracker handles widening (`vw*`) and segment (`vl*seg*`/`vs*seg*`) correctly.
- `vstart` is precise on element-level exceptions.
- `enableVector=false` RTL diff vs. pre-Caracal v4 is empty.
- Every per-folder unit test under `generators/boom/src/tests/<TestName>/` passes when invoked by its own `run.sh`, and `sbt test` at the boom root passes them as an aggregate.

---

## Non-goals for Goal 1 (deferred to Goal 2+)

- Vector arithmetic execution (V-ALU). `IQ_V_ALU` is built but tied off.
- VPU interface (custom interface between BOOM and an external vector unit).
- Reduction ops, mask logic ops, permutation ops.
- FP vector ops (vfadd, vfmul, ...) — depend on V-ALU.
- Vector register file banking — 2W/2R suffices for LSU-only operation in Goal 1 (Step 10).
- DCache widening beyond what `lsuWidth` already provides — Goal 1 uses the existing 1-lane (Medium) or 2-lane (Mega) DCache request bus via the new dynamic arbiter (Step 11).
- Performance tuning. Goal 1 is functional correctness; Goal 3+ will tune.

---

## Cross-cutting risks and mitigations

| Risk | Mitigation |
|------|-----------|
| Widening `*_rtype` from 2→3 bits ripples into many files | Done in Step 1; audited and tested before any vec logic lands |
| Cracker × branch-kill correctness | Sub-uops inherit parent's `br_mask`; tested in Step 3 verification |
| V-LSU and scalar LSU memory ordering (RVWMO) | Step 11 adds bidirectional cross-LSU snoop + memory-dependency speculation extension + fence-drains-both; not just bus arbitration |
| DCache port contention | Step 11 `DCacheArbiter` — `single` on Medium, `dual-dynamic` on Mega; cross-LSU snoop eliminates same-address contention at the cache |
| `vstart` precision on faults | Sub-uops execute in-order in the V-LSU pipeline (Step 11); writebacks to vec regfile in age order; trap sets `vstart` to the faulting element's global index |
| Vector-store pdst freed before drain | Per-pdst `store_pending` bit in `VecFreeList` (Step 5); freelist defers dealloc while set, clear on V-StoreBuffer drain completion (Step 11) |
| Cracker explosion at LMUL=8 with segment+widening (up to 64 sub-uops per arch op) | 8-wide cracker (Step 3) cracks 8/cycle; queues sized at 64 entries (Step 7); buffer (Step 4) only absorbs `vsetvl` parks; ROB stays at 1 entry/arch-op via shared `rob_idx` |
| Test coverage gap for indexed/segment/FoF loads | Step 14 explicitly enumerates these; new ELFs append to `tests_regr/vset_loadstore_tests.txt` so they become per-step (8e) gates |
| `IQ_V_ALU` tied off accidentally allowing a vector ALU op to issue and corrupt state | Hard assertion in Step 12 |
| Slot squash leaves new pdst with garbage when `vta=0` | Slot issues regfile-copy `pdst ← stale_pvdest` instead of pure squash (Step 7); uses dedicated regfile copy ports (Step 10) |
| `pvm` stale value read on unmasked ops | V-LSU / V-ALU does conditional read of `vec_regfile(pvm)` gated on encoded `vm` bit (Step 5) |
| Scalar+vector dispatch group depth mismatch | 1-cycle pipeline register on scalar bypass aligns to vRRU latency (Step 5); dispatch group remains atomic |
| `vsetvl` cracker park deadlock | `vsetvl` decoded into two micro-ops (Issue 1): scalar uop flows through normal pipeline + executes; cracker sentinel sits in the buffer until matching cracker-broadcast arrives. Scalar uop is never trapped at the cracker. |
| Vtype delivery to pre-rename cracker | Dedicated cracker-broadcast bus (Issue 2) from scalar EU carries `(vsetvl_id, vtype, vl)` directly to the cracker and to vector issue slots. |
| Speculative vtype corrupts cracker mirror on mispredict | Cracker mirror snapshotted per br_tag, restored on `brupdate.b2.mispredict` in lockstep with vector maptable (Issue 3 / Step 5). |
| Out-of-order vsetvl broadcasts | Cracker maintains `next_expected_id` + small ordered side buffer keyed by `vsetvl_id` (Issue 4 / Step 3). |
| Architectural vtype/vl/vstart CSR drift from cracker mirror | ROB drives commit-time CSR write port for vtype/vl + clear-on-retire pulse for vstart (Issues 7/8); cracker mirror tracks via the same retire pulse. |
| `vsetvli rs1=x0` / `rd=x0` idioms silently miscompiled | `VsetDecode` handles all four spec sub-cases explicitly (Issue 9). |
| MSHR allocation under cross-LSU contention | `mshrAllocPolicy` parametrized — `fair-floor` default with per-LSU reservation floors, `hard-partition` and `fcfs` available (Issue 10). |
| Cross-LSU snoop forwarding ambiguity | All cross-LSU forwarding merges at V-LB allocation stage (Issue 11) — single point, same shape as scalar BOOM LSU. |
| Goal-1 regression silently misses `vsetvl` codepath | Three hand-crafted vsetvl ELFs added to `tests_regr/vset_loadstore_tests.txt` (Issue 12), gated in Step 11d. |
| Fence-drain checks killed entries | Explicit `valid && !killed && seqnum < fence.seqnum` predicate (Issue 13). |

---

## File-touch summary

**Created (all under `src/main/scala/v4/vec/`):**

| Step | Path |
|------|------|
| 0 | `common/VectorParams.scala`, `common/package.scala` |
| 2 | `decode/VDecode.scala`, `decode/VLSDecode.scala`, `decode/VsetDecode.scala` |
| 3 | `decode/VecUopCracker.scala` |
| 4 | `decode/DecodeToCrackerBuffer.scala` |
| 5 | `rename/VecRenameStage.scala`, `rename/VecMapTable.scala`, `rename/VecFreeList.scala`, `rename/VecBusyTable.scala` |
| 7 | `issue/VecIssueSlot.scala`, `issue/VecIssueUnit.scala` (optional) |
| 8 | `lsu/VecLsDecode.scala`, `lsu/ConfigInfo.scala` |
| 10 | `regfile/VecRegFile.scala` |
| 11 | `lsu/VecLoadQueue.scala`, `lsu/VecStoreQueue.scala`, `lsu/VecLoadBuffer.scala`, `lsu/VecStoreBuffer.scala`, `lsu/VecAgu.scala`, `lsu/VecMemUnit.scala`, `lsu/VecMemopFsm.scala`, `lsu/VecIdxLsFsm.scala`, `lsu/VecMaskFsm.scala`, `lsu/CrossLsuSnoop.scala`, `lsu/DCacheArbiter.scala` |

**Modified (baseline files — kept minimal):**

| File | Steps touching | Reason |
|------|----------------|--------|
| `v4/common/parameters.scala` | 0, 7, 13 | `enableVector`, `VectorParams`, new `require`s, derived `usingVector` |
| `v4/common/config-mixins.scala` | 0 | `WithVector` mixin, Mega override |
| `v4/common/consts.scala` | 1, 7 | `RT_VEC`; widen `iq_type`; new `IQ_V_*` |
| `v4/common/micro-op.scala` | 1 | Vector fields (`is_vec`, `pvs*`, `VConfig`, cracker bookkeeping, segment fields) |
| `v4/exu/decode.scala` | 2 | Hook in `VDecode`/`VLSDecode`/`VsetDecode`; cracker-side `vsetvl` stall plumbing |
| `v4/exu/dispatch.scala` | 7 | Route to new vector queues (no change if bitwise routing already works post-widening) |
| `v4/exu/rob.scala` | 6, 11 | New writeback ports (including squash-grant), `stale_pvdest`, cracked-uop busy-counter, fence-drain handshake |
| `v4/exu/core.scala` | 3, 4, 5, 7, 8, 10, 11, 12, 13 | Top-level wiring of every new module; 1-cycle bubble on scalar bypass to vRRU (Step 5) |
| `v4/lsu/lsu.scala` | 11 | Cross-LSU snoop wiring (scalar SQ ↔ vector LB; scalar LQ ↔ vector SB); replaces old "shared port with priority" approach |

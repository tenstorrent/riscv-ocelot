# Caracal Milestone 1 Implementation Plan

**Objective.** Add RVV 1.0 instruction *support* to BOOM v4 — decode, the **atomic LMUL/EMUL vector mapper** (no frontend cracking), VL register-file rename, ROB allocation, vector issue queues, the vector register file, and the unified vector load/store unit — all integrated into the BOOM out-of-order core itself (not as an OVI side-car as in `bobtail/main`). The vector ALU datapath is **deferred to Milestone 2**: `IQ_V_ALU` is built but tied off; arithmetic uops are decoded, renamed, and queued but never execute, and the CII coprocessor is not attached. **Vector loads and stores must execute end-to-end.**

**Reference.** The architecture spec is `docs_caracal/src/*.rst` (built HTML under `docs_caracal/_build/`) with figures in `docs_caracal/figures/` (`vector_mapper`, `dispatch_issue`, `execution_stage`). v4 baseline documented in `docs/boom-v4-architecture.md`. Prior OVI-based vector integration on `bobtail/main` is reference material for SystemVerilog primitives to port (`src/main/resources/vsrc/vpu/`) and for one already-Chisel module (`src/main/scala/exu/ovi_wrapper/ls_decode.scala`).

**Branch strategy.** Each step below is a `Caracal/addvector/<step-slug>` feature branch that merges into `Caracal/addvector` and ultimately into `Caracal/main`. Steps are ordered so that every intermediate state compiles, elaborates, and (when `usingRVV=false`) is bit-identical to baseline.

Whenever you are uncertain about implementation details and planning, ask/prompt the user.

---

## Global ground rules

1. **Feature flag.** All vector logic gated by `enableVector: Boolean = false` (`BoomCoreParams`), surfaced as the derived `usingRVV`. Default off — every existing config and test must remain bit-identical to pre-Caracal output.
2. **Code location.** All new Chisel under `src/main/scala/v4/vec/`. Subdirs: `decode/`, `rename/`, `issue/`, `regfile/`, `lsu/`, `common/`. Package `boom.v4.vec.{decode,rename,issue,regfile,lsu,common}`.
3. **Minimize intrusive edits.** Whenever logic *can* live in `v4/vec/` rather than in a baseline file, put it there — even if it duplicates a small amount of BOOM code. Touch baseline files only where unavoidable (`MicroOp`, `BoomCoreParams`, `config-mixins.scala`, `core.scala`, `decode.scala`, `rob.scala`, the integer ALU EU, and the `IssueSlot`/`IssueUnit` bundles).
4. **Parameters.** Defaults: `VLEN = 256`, `ELEN = 64`, `numVecPhysRegisters = 128`, `numVlPhysRegisters = 64`. Everything else parameterized (`numVecLoadQueueEntries`, `numVecStoreQueueEntries`, issue-queue widths/entries, LCB depth, SSI/US queue depths, `dcacheArbiterMode`, …). **No cracker parameters** — the atomic mapper removes frontend cracking, so any `crackerWidth`/`numDecodeToCrackerBufferEntries` left from earlier drafts are removed or repurposed (the mapper's group allocation is `coreWidth*8` wide). The default tier under test is `WithNSmallBooms ++ WithVector`, but the implementation must elaborate cleanly at Medium and Large widths too.
5. **No frontend cracking; atomic group rename; OoO across queues.** A vector instruction is a **single `OP.v`** through decode/rename/ROB/issue. The vector mapper renames a whole `LMUL`/`EMUL` group atomically (up to 8 PRNs per `vdest`); element/segment cracking into `nOP.v` happens only in the Vector LS AGEN. `IQ_V_LOAD`/`IQ_V_STORE` are **age-ordered collapsing** (vector memory is OoO); **`IQ_V_ALU` is an in-order, non-speculative FIFO** that feeds the in-order CII coprocessor in program order and — RoCC-style — only issues instructions **past the PNR** (known-safe, guaranteed to commit), so the CII needs no branch-kill/replay. Inter-queue ordering is enforced exclusively by the ROB.
6. **Group-done completion.** Every vector producer signals completion of a whole destination group **once** — the LCB (loads) emits one group-done after the last destination PRN lands; the (future) CII (arith) emits one per `OP.v`. That single event carries the group's **member-PRN vector** and drives the ROB single-shot busy-clear, the Busy-Table clear, and the VECTOR wakeup. No per-entry ROB completion counter.
7. **Precise exceptions.** Vector loads/stores update `vstart` on element-level faults (oldest faulting element) so the trap handler can resume mid-vector. Stores translate/disambiguate all active elements pre-commit and do not write the DCache until ROB-committed.
8. **Reuse the existing wakeup machinery; add only what the spec requires.** Scalar feeders (base/stride/`.vx`) ride the existing INT network; `.vf` rides FP. Two new networks are added: **VL** (`pvl`, its own register space) and **VECTOR** (group-done, member-PRN vector). There is no VLBU/value-broadcast — `pvl` is a plain readiness wakeup and the VL value is read from the VL RF at execute.
9. **Per-step verification gate (universal).** Every step is "done" only when **all** of the following pass:

   a. `sbt compile` clean inside `generators/boom/`.
   b. `make checkstyle` clean inside `generators/boom/`.
   c. **Base Chipyard build with vector enabled** — proves Chisel still elaborates end-to-end and catches diplomacy / parameter-ripple breakage an isolated `sbt compile` misses:
      ```bash
      cd /root/my-chipyard/sims/vcs
      make CONFIG=MediumBoomV4VectorConfig -j$(nproc) debug
      ```
   d. **Scalar performance regression with vector enabled** — proves the scalar datapath is unbroken by live vector plumbing. Must pass at *every* step (including steps that add no executable vector behavior). Doubles as the per-step perf baseline:
      ```bash
      cd /root/my-chipyard/sims/vcs
      ./run_regr_rvv_scalar.sh MediumBoomV4VectorConfig
      ```
   e. **Vector regression — gated in two phases.** Run in order; the L/S phase may not start until the vset smoke passes:
      - **(e1) `vset` smoke test** — required from **Step 9 (Integer ALU EU for `vset`) onward**. A single ELF that exercises `vsetivli`/`vsetvli`/`vsetvl` and reads back `vl`/`vtype` (via `csrr`) must PASS **before** any load/store test is attempted — `vset` is the prerequisite for every vector LS test, so prove it in isolation first:
        ```bash
        cd /root/my-chipyard/sims/vcs
        ./run_regr.sh tests/rvv/vset_test/vset_test.elf MediumBoomV4VectorConfig
        ```
      - **(e2) `vset` + load/store regression** — required from **Step 11 (Unified LSU) onward** and on any later step that can touch the vector LS datapath. Gated behind (e1) passing:
        ```bash
        cd /root/my-chipyard/sims/vcs
        ./run_regr.sh tests_regr/vset_loadstore_tests.txt MediumBoomV4VectorConfig
        ```
      Earlier steps may skip (e); (a)–(d) and (f) still apply.
   f. **Baseline `usingRVV=false` is bit-identical to pre-Caracal v4.** Elaborate `MediumBoomV4Config` (vector mixin not applied); RTL diff against the pre-Caracal v4 baseline must be empty.
   g. The step's listed step-specific verification artifacts pass.

   Don't merge a step branch into `Caracal/addvector` until (a)–(g) are all green.

10. **No unit tests — module `printf` tracing instead.** |caracal| does **not** add unit-level testbenches: no `generators/boom/src/tests/`, no `chiseltest`/`ChiselSim`, no per-module `*Spec.scala`/`run.sh`, no extra test source root. Every module is validated **only** through the end-to-end VCS+Whisper regressions (rule 11) plus the per-step gate (9). To make those runs debuggable, **every new vec module emits guarded `printf` trace statements** so a single instruction can be followed stage-by-stage through the simulation log:

    - **Gated, off by default.** All trace prints are conditioned on a debug signal (a `vecTrace` plusarg / `DEBUG`-style `Bool`), so production RTL — and the `usingRVV=false` bit-identical baseline (9f) — are unaffected.
    - **Tagged for correlation.** Each line carries the module name and the uop's `rob_idx` (plus, for vector ops, `pvdest`/`pvl`/`vl`/`v_emul` as relevant) so events from different stages line up with each other and with the Whisper cosim trace, and are greppable.
    - **One line per key event** per stage, e.g.: decode (`opcode → is_vec`/`iq_type`/`v_eew`/`v_emul`), VCFG (`vtype`/`vl` update), mapper (group alloc `pvdest[*]`, stale group, non-whole-group counter), VL rename (`pvl`), issue (grant, operands-ready, PNR-pass for `IQ_V_ALU`), AGEN/DGEN (per-`nOP.v` element/PRN/offset), LCB (per-PRN fill, group-done), ROB (alloc/commit/group-done/`vstart`).
    - **These prints are the per-step (9g) artifact:** a step is "done" when its module's trace shows the expected instruction flow in the (9d)/(9e) logs (cross-checked against Whisper).

11. **End-to-end ELF-test convention.** Every test that runs a real RISC-V ELF must go through VCS with the Whisper cosim sidecar — never Verilator. The two regression scripts in (9d)/(9e) are the only sanctioned ELF harnesses; new ELFs are appended to `sims/vcs/tests_regr/*.txt`. Whisper cosim catches per-instruction functional divergence.

---

## Step 0 — Project scaffolding and feature flag

**Scope.** Directory structure, feature flag, config mixin shell. No functional behavior.

**Files created.**
- `src/main/scala/v4/vec/common/VectorParams.scala` —
  ```scala
  case class VectorParams(
    vLen: Int = 256,
    numVecPhysRegisters: Int = 128,        // vector PRF depth
    numVlPhysRegisters: Int = 64,          // VL register file depth (own rename space)
    numVecLoadQueueEntries: Int = 64,
    numVecStoreQueueEntries: Int = 64,
    numVecTmpGroups: Int = 4,              // reserved headroom for pvtmp (segmented LS)
    ssiQueueEntries: Int = 512,            // worst-case single-store element count
    lcbEntries: Int = 8,                   // VLEN-wide load assembly entries
    vecIssueGrantWidth: Int = 1,           // 1 on Medium, 2 on Mega
    dcacheArbiterMode: String = "single",  // "single" | "dual-dynamic" (Mega)
    vecScalarSnoopEnable: Boolean = false,
    mshrAllocPolicy: String = "fair-floor"
  )
  ```
- `src/main/scala/v4/vec/common/package.scala` — package object.
- `src/main/scala/v4/vec/{decode,rename,issue,regfile,lsu}/.keep`.

**Files modified.**
- `parameters.scala` — add `enableVector`/`vector: Option[VectorParams]` to `BoomCoreParams`; derive `usingRVV`; derive `numVecPhysRegs`/`vecPregSz`, `numVlPhysRegs`/`vlPregSz`, `vecVLen`, `vecLregSz`, `maxVecVL`/`vecVLSz`. **Do not** alter any `require()` yet.
- `config-mixins.scala` — `class WithVector extends Config(...)` setting `enableVector=true`, `vector=Some(VectorParams())`; `WithNSmallBoomsVector`; Mega overrides (`vecIssueGrantWidth=2`, `dcacheArbiterMode="dual-dynamic"`).

**Verification (9g).** Add Chipyard configs `MediumBoomV4VectorConfig` (= `MediumBoomV4Config + WithVector`) and `MegaBoomV4VectorConfig` so later steps' (9c)/(9d)/(9e) have valid CONFIGs. No trace yet (scaffolding only) — exercised by the (9c) build.

---

## Step 1 — `MicroOp` vector field extension

**Scope.** Extend `MicroOp` so every downstream stage sees vector state. No logic; all new fields inert (zero/`false`) for scalar uops.

**Files modified.**
- `src/main/scala/v4/common/micro-op.scala`:
  - `is_vec: Bool`.
  - **Widen `dst_rtype`/`lrs1_rtype`/`lrs2_rtype` to 3 bits** (already done) and add `RT_VEC = 4.U(3.W)` in `consts.scala`.
  - Logical vector regs: `lvs1`/`lvs2`/`lvs3`/`lvd`/`lvm` (V0 mask).
  - Physical (post-rename): `pvs1`/`pvs2`/`pvs3`/`pvdest`/`stale_pvdest`/`pvm` (`vecPregSz`), `pvl` (`vlPregSz` — indexes the **VL register file**, not the integer RF). Busy bits `pvs*_busy`/`pvm_busy`/`pvl_busy`.
  - **No** `pvtype` (VTYPE is not renamed). **No** `stale_pvl` (single-entry VL space frees the outgoing committed pointer). **No** `vl_is_known` (VL is always renamed into the VL RF).
  - `is_shared: Bool` and `pvtmp: Vec(8, UInt(vecPregSz.W))` — the segmented-LS temp group, allocated from the **main** vector free list (no TVRB).
  - `vconfig: VConfig` — **vtype snapshot only** (`vlmax`/`vsew`/`vlmul`/`vma`/`vta`). `vstart`/`vxrm`/`vxsat` live in the CSR file, read at execute.
  - `v_eew`/`v_emul` (UInt(3.W)).
  - **nOP.v cursor** `v_split_first/last/idx/total` — inert on the OP.v; populated only by the Vector LS AGEN when it emits element/segment `nOP.v` (these are **not** frontend-cracker state).
  - Segment fields `v_seg_nf`/`v_seg_idx`.
- `consts.scala` — `RT_VEC`; widen `iq_type` one-hot Vec to `IQ_SZ = 7` and add `IQ_V_LOAD/IQ_V_STORE/IQ_V_ALU = 4/5/6`.

**Verification (9g).** Bundle-only change — no trace. Exercised by the (9c) elaboration. **Risk:** the `*_rtype` widening ripples — audit every `*_rtype === RT_*` site; (9d) is load-bearing.

---

## Step 2 — RVV 1.0 decode + Vector Config Unit (VCFG)

**Scope.** Recognize RVV 1.0 in `decode.scala`, populate `MicroOp` vector fields, and stand up the VCFG `vtype` mirror. No mapper yet.

**Files created.**
- `vec/decode/VDecode.scala` — vector decode table (arith → `IQ_V_ALU`, loads → `IQ_V_LOAD`, stores → `IQ_V_STORE`; sets `is_vec`, `lvs*`/`lvd`/`lvm`, `v_eew`/`v_emul`, `v_seg_nf`).
- `vec/decode/VLSDecode.scala` — unit-stride / strided / indexed (ordered+unordered) / segment / fault-only-first; encodes `mop`/`lumop`/`nf`/`width`.
- `vec/decode/VsetDecode.scala` — classifies the three vset forms and marks routing:
  - **`vsetivli`** (immediate VTYPE + AVL): VCFG updates the `vtype` mirror at decode; computed VL written to the VL RF in the front-end (no back-end EU).
  - **`vsetvli`** (immediate VTYPE, register AVL): VTYPE mirror at decode; **executes on the integer ALU EU** (computes `min(rs1, VLMAX)` → VL RF). Not serialized.
  - **`vsetvl`** (register VTYPE + AVL): **`is_unique`** (mapper needs VTYPE for EMUL at decode); executes on the integer ALU EU; updates the VCFG mirror post-drain.
- `vec/decode/VConfigUnit.scala` (VCFG) — speculative `vtype` mirror + committed `vtype` shadow + per-`br_tag` snapshot. Per-lane **nearest-preceding-vset prefix select** for NxWide decode.

**Files modified.**
- `decode.scala` — gate behind `usingRVV`; hook `VDecode`/`VLSDecode`/`VsetDecode` after the scalar tables (last-connect-wins). Mark **explicit vector-CSR accesses** (`csrr`/`csrw`/`csrrw` to `vstart`/`vxrm`/`vxsat`/`vcsr`/`vl`/`vtype`/`vlenb`) `is_unique`.

**Spec refs.** `frontend.rst` (Vector Decode, VSET Special Handling, VCFG Mirror Recovery, Explicit vector-CSR accesses).

**Verification (9g).** `VDecode`/`VsetDecode`/VCFG emit a per-uop trace (`opcode → is_vec`/`iq_type`/`v_eew`/`v_emul`, and `vtype`/`vl` mirror updates); confirm the expected decode for `vsetivli`/`vsetvli`/`vsetvl`/`vle32.v`/`vse64.v`/`vlseg3e8.v`/`vluxei8.v`/`vadd.vv`/widening `vwadd.vv` in the (9d) log (cross-checked vs Whisper). **Risk:** widening/narrowing EEW is where bugs hide — make sure the trace prints `v_eew`/`v_emul` for those.

---

## Step 3 — Atomic Vector Mapper (group rename)

**Scope.** The replacement for the old cracker: a vector RMT + free list + busy table that rename a whole `EMUL` group atomically. One `OP.v`, one ROB entry.

**Files created.**
- `vec/rename/VecMapTable.scala` — 32 entries, **1 PRN per architectural vreg**. An `EMUL`-wide group read returns up to 8 member PRNs per source (`pvs1[8]`/`pvs2[8]`/`pvs3[8]`) + the stale group, correct under arbitrary fragmentation. Includes the **LMUL Tag Table (32×2b)** + combinational whole-group checkers (`is_whole_vg_{1,2,4,8}`) feeding a **non-whole-group perf counter** (observability only — the read is always correct). Group-aware intra-bundle **reg-src / stale bypass** (per-member: forward an older same-bundle lane's freshly-allocated PRN for overlapping members).
- `vec/rename/VecFreeList.scala` — `SelectFirstN`, `allocWidth = coreWidth*8`, **non-contiguous** `EMUL`-group allocation; `deallocWidth = commitWidth*8` (free the whole stale group at commit); `br_alloc_lists` reclaim. Reserve `numVecTmpGroups` headroom so a segmented LS can always allocate `pvdest`+`pvtmp` (forward-progress guarantee).
- `vec/rename/VecBusyTable.scala` — per-PRN over `numVecPhysRegs`. **Set:** all member bits of the dest group (up to 8/OP.v). **Source reads:** per-member bits of each source group, **AND-ed into one group-ready bit** (woken on the last member). **Clear:** group-done carries the member-PRN vector (up to 8 bits/port).

**Spec refs.** `midcore.rst` (Rename Map Table, LMUL Tag checker, Free List, Busy Table, `group-done`); `vector_mapper` figure.

**Verification (9g).** The mapper emits a per-`OP.v` trace: the allocated `pvdest[*]` group + stale group, the EMUL-wide source group reads, the non-whole-group counter, and (Busy Table) per-member set / group-done clear / group-ready-on-last-member. Confirm a vector op's group alloc and a sub-range consumer waking on group-done in the (9e) log. (No standalone unit test — driven by the e2e regressions.)

---

## Step 4 — Single-stage rename integration + VL register-file rename

**Scope.** Wire the vector mapper to rename **in parallel with the scalar (INT/FP) rename in the same cycle** (no second pipeline stage, no delayed pipeline register), and add the VL register file with its own rename. VTYPE is **not** renamed (rides the VConfig snapshot).

**Files created.**
- `vec/rename/VecRenameStage.scala` — wraps `VecMapTable`/`VecFreeList`/`VecBusyTable`.
- `vec/rename/VlRename.scala` — VL register-file rename: 1-entry `VlMapTable` (current `pvl` pointer) + per-`br_tag` snapshots, `VlFreeList` (64), `VlBusyTable`, and the **VL wakeup network**. Commit frees the **outgoing committed pointer** (no stale field). Producers: VCFG (`vsetivli`), integer ALU EU (`vsetvli`/`vsetvl`), LSU (`vleff`).

**Files modified.**
- `core.scala` — pipeline order: decode → rename → dispatch (a **single** rename stage). For a vector `OP.v`, the scalar `rename_stage`/`fp_rename_stage`, the vector mapper, and the VL mapper all run **in parallel** in that one cycle (independent register spaces). Everything the rename→dispatch boundary does is atomic and in program order in that cycle:
  - **In-order reservations:** ROB entry, `br_tag`, **LDQ/STQ slot reservation** (each vector load/store `OP.v` = a single LDQ/STQ entry, `ldq_idx`/`stq_idx` as program-age stamp).
  - **Parallel rename:** scalar INT/FP PRNs; the vector group (`pvdest`/`pvs*`/`pvm` + `EMUL` `stale_pvdest` + `v_emul` + `pvtmp`); and `pvl`.
  - **IQ-slot writes:** every uop (scalar and vector) writes its IQ slot this cycle — no vector lag.
  - **Branch snapshots:** vector RMT, VL map table, and VCFG mirror snapshot on the **same `ren_br_tags` event** as the scalar RMT — **no delayed-`br_tag` path**, no 1-cycle pipeline register. The VCFG mirror (decode-stage) snapshots from the branch's carried `vconfig`.
  - **Trade-off:** the vector group mapper is now on the rename critical path (in parallel with scalar rename → cost = the slower of the two, not the sum).

**Spec refs.** `midcore.rst` (`rename-stage`, `vl-vtype-rename`); `frontend.rst` (VL delivery, VCFG Mirror Recovery).

**Verification (9g).** `VecRenameStage`/`VlRename` emit a trace: per-`OP.v` group alloc and `pvl` assignment, branch snapshot/restore events (same-cycle `br_tag`), and commit frees (stale group / outgoing VL pointer). Confirm a mixed scalar+vector dispatch group renames in one cycle and a mispredict restores the maps, in the (9e) log.

---

## Step 5 — ROB extension (group-done completion)

**Scope.** ROB receives vector completions; single-shot busy-clear via group-done; group stale-free at commit.

**Files modified.**
- `rob.scala`:
  - `dst_rtype` 3 bits (done) to encode `RT_VEC`.
  - **Completion:** one group-done per `OP.v` clears `rob_bsy` (single-shot, like a scalar writeback — **no per-entry counter**). The **shared instruction** (segmented LS) is the only multi-source case: a **1-bit "other half pending" flag** clears only when both the LSU group-done and the (future) CII group-done arrive.
  - `rob_unsafe` cleared on a single **group-safe** event (last element address disambiguated).
  - Commit drives `dealloc_vec_pregs` with the **`EMUL` stale group** (`stale_pvdest`) to the vector free list; drives the **commit-time `vtype`/`vl` CSR write** and the **`vstart` clear** (or set, on trap, from the LSU exception port's faulting-element index).
  - Add vector writeback / group-done ports to `wb_resps`/`lsu_clr_bsy`.

**Spec refs.** `midcore.rst` (ROB Completion-tracking, Commit, Precise exceptions).

**Verification (9g).** The ROB emits a trace per vector entry: alloc, the single group-done clearing `rob_bsy`, `stale_pvdest` group dealloc on commit, the shared-instruction both-halves wait, and `vstart`/CSR writes. Confirm a vector load commits on one group-done and a segmented LS waits for both halves, in the (9e) log.

---

## Step 6 — Vector issue queues + vector issue slot

**Scope.** Three age-ordered collapsing queues and the superset vector slot. Single scheduling stage.

**Files created.**
- `vec/issue/VecIssueSlot.scala` — extends `IssueSlot`:
  - **Scalar feeders** (reuse `prs1`/`prs2`): base/stride/`.vx` on the **INT** network; `pvl` on the **VL** network; `.vf` scalar FP on the **FP** network (`IQ_V_ALU` only).
  - **Vector operands** on the **VECTOR** network: `pvs1`/`pvs2`/`pvs3`/`pvm`, each holding its **member PRNs** (up to `EMUL`); per-member busy bits AND-ed into a group-ready bit (woken on last member). `pvm` participates in `request` only when masked.
  - `request := slot_valid && !iw_issued && scalar_operands_ready && vector_operands_ready`. Granted **once**.
  - Store AGEN/DGEN split: DGEN gated on `pvs3` (VECTOR network) instead of `prs2`.
- `vec/issue/VecIssueUnit.scala` — `IQ_V_LOAD`/`IQ_V_STORE` wrap `IssueUnitCollapsing` (age-ordered) for the insert/grant widths; **`IQ_V_ALU` is an in-order, non-speculative FIFO** variant: head-only select, granted only when the head's operands are ready **and** the head is **past the PNR** (`is_older(rob_idx, rob.io.rob_pnr_idx)`), RoCC-style. Wire `rob_pnr_idx` into `IQ_V_ALU`. This delivers arithmetic `OP.v`'s to the CII in program order and only once non-speculative, so the in-order CII needs no branch-kill/replay (squashed entries are dropped from the FIFO before issue).

**Files modified.**
- `parameters.scala` — append three `IssueParams(iqType = IQ_V_*)`; relax the "exactly one of each" `require`s to be `usingRVV`-aware.
- `dispatch.scala` — bitwise `iq_type` routing extends once widened; route shared instructions to **both** the CII IQ and the LSU path.
- `core.scala` — instantiate the three units; wire INT + FP + **VL** + **VECTOR** wakeup networks.

**Spec refs.** `issue.rst` (Issue/Scheduling, Wakeup Networks, Vector Issue Slot); `dispatch_issue` figure.

**Verification (9g).** Dispatch + the vector slots emit a trace: `iq_type` routing (vle/vse/vadd → V_LOAD/V_STORE/V_ALU; shared → two queues), per-slot operand-ready / grant, and for `IQ_V_ALU` the in-order head + PNR-pass. Confirm correct routing and age-ordered grant (and `IQ_V_ALU` granting only past-PNR) in the (9d)/(9e) logs.

---

## Step 7 — Vector L/S decoder (port `ls_decode.scala`)

**Scope.** Port bobtail's Chisel `ls_decode.scala` → `VecLsDecode`; finishes LS decoding (stride/segment/EEW, AGEN parameters) once operands are resolved.

**Files created.** `vec/lsu/VecLsDecode.scala`, `vec/lsu/ConfigInfo.scala` (port the `ConfigInfo` bundle; `vl`/`vstart` fields are filled at execute from the VL RF / CSR file).

**Files modified.** `core.scala` — instantiate between issue and the V-LSU.

**Verification (9g).** `VecLsDecode` emits a trace of the decoded `ConfigInfo` (mop/stride/`nf`/EEW/access-class) per issued LS uop; confirm correct fields for unit-stride/strided/indexed/segment/masked in the (9e) log (cross-checked vs Whisper / the bobtail reference).

---

## Step 8 — Vector register file (VRF) + VL register file

**Scope.** The vector PRF and the VL RF.

**Files created.**
- `vec/regfile/VecRegFile.scala` — `numVecPhysRegisters=128`, `VLEN=256`, **8 read / 4 write ports**, banked 4×64b, single-cycle read with write-forwarding. Ports: CoProc 4R/2W (Milestone 2), Store 2R/0W, Load 2R/2W. `pvtmp` groups live here (no separate temp file). Tail/mask-undisturbed and the `VL==0`/`vta=0` group copy reuse the Load Unit ports.
- `vec/regfile/VlRegFile.scala` — `numVlPhysRegisters=64`. Write ports for VCFG (`vsetivli`), integer ALU EU (`vsetvli`/`vsetvl`), and LSU (`vleff`); read ports for every vector EU (`pvl`).

**Files modified.** `core.scala` — instantiate; wire load writeback (W0), AGU read (R0), drain read (R1), etc.

**Spec refs.** `midcore.rst` (Register Files, VRF port table); `case_study.rst` (VL==0 group copy).

**Verification (9g).** The VRF/VL RF emit a trace on each write/read port (PRN index, port, group member); confirm load writebacks (W0) and VL-RF writes land at the expected PRNs in the (9e)/(e1) logs. Read-during-write behavior is pinned in the spec, not unit-tested.

---

## Step 9 — Integer ALU EU extension for `vset` (+ `vset` smoke test)

**Scope.** Extend the integer ALU EU (gated by `usingRVV`) to execute `vsetvli`/`vsetvl`, completing the `vset` path **end-to-end so it can be verified in isolation, before any load/store**. (`vsetivli` is already front-end/VCFG from Step 2; this step adds the register-sourced forms.)

**Why this comes before the LSU.** `vset` is the prerequisite for *every* vector load/store (you must set `vtype`/`vl` before a `vle`/`vse`), so it must be proven working before the LSU's L/S regression. Its full path is now in place: decode + VCFG (Step 2), VL rename (Step 4), ROB commit-time `vtype`/`vl` CSR write (Step 5), VRF/VL RF (Step 8), and the integer-ALU execution added here. `vsetvli`/`vsetvl` ride the **scalar** integer issue queue and execute on the int ALU EU (they carry no vector register operands), so this step does not depend on the AGEN/LSU.

**Files modified.**
- `exu/execution-units/*` (integer ALU) — for a `vset` uop: read `rs1` (and `rs2` for `vsetvl`) from the integer RF/bypass, compute `VL = min(rs1, VLMAX)` (and VTYPE for `vsetvl`), and on writeback **target the VL RF** + drive the **VL wakeup network** (and update the VCFG `vtype` mirror for `vsetvl` under `is_unique`). When `rd != x0`, also write `rd` normally. Bit-identical to BOOM when `usingRVV=false`.
- `core.scala` — wire the ALU's VL-RF write port + VL-wakeup driver.

**Spec refs.** `frontend.rst` (Where each vset executes); `overview.rst` (Execute / Integer execution row).

**Verification (9g).** The **(e1) `vset` smoke test** is the gate for this step and **must PASS before Step 11's (e2) L/S regression**:
```bash
cd /root/my-chipyard/sims/vcs
./run_regr.sh tests/rvv/vset_test/vset_test.elf MediumBoomV4VectorConfig
```
It exercises `vsetivli`/`vsetvli`/`vsetvl` and reads back `vl`/`vtype` via `csrr` (committed CSR values), proving `vset` works end-to-end — VCFG `vtype` mirror, VL-RF write, and commit-time CSR update — **independent of the LSU**. The int ALU EU emits a `vset` trace (computed `vl`/`vtype`, VL-RF write) so the smoke run is debuggable. Do not start Step 11 until this is green.

---

## Step 10 — Vector LS AGEN stage (two-stage)

**Scope.** Crack L/S `OP.v` into `nOP.v` at element/segment granularity; track dest/src PRN + offset.

**Files created (under `vec/lsu/` — port from bobtail).**
- `Vec{Load,Store}Packer.scala`, `Vec{Load,Store}Skipper.scala`, `Vec{Load,Store}Walker.scala` — the Packer/Skipper/Walker generators.
- `VecAgenStage1.scala` (`ld_vAGEN_1`/`st_vagen_1`): **Skipper + Walker only**. SSI (strided/indexed/segmented) expanded to `nOP.v` here; unit-stride encoded as a **single** `nOP.v` (base, stride, `is_unit_stride`).
- `VecDgen.scala` (`st_vDGEN`): reads VRF (vector src) or INT/FP RF (scalar src, e.g. `vfmul.vf`); SSI per-element ELEN, unit-stride whole VLEN.
- The stage-2 Packer lives at the LSU queues (Step 11), expanding the unit-stride `nOP.v` per-element just-in-time.

**Files modified.** `core.scala` — wire issue → `VecLsDecode` → AGEN.

**Spec refs.** `execution.rst` (Vector LS AGEN, Vector DGEN); `execution_stage` figure.

**Verification (9g).** The AGEN/DGEN emit a per-`nOP.v` trace (element index, PRN + offset, address, access class); confirm the expected element stream for a unit-stride and an indexed op in the (e2) log. Covered e2e by the Step 11 LSU regression.

---

## Step 11 — Unified LSU (Chisel ports of bobtail VPU memory primitives)

**Scope.** The largest step: vector load/store path end-to-end. **Prerequisite: the Step 9 (e1) `vset` smoke test is green.** References `bobtail:tt_lq.sv`/`tt_store_buffer.sv`/`tt_mem.sv`/`tt_memop_fsm.sv`/`tt_idxldst_fsm.sv`/`tt_mask_fsm.sv`.

**Files created (under `vec/lsu/`).**
- Vector address/data queues: `ld_SSI_ADDR_Q`, `st_SSI_ADDR_Q`/`st_SSI_DATA_Q` (ELEN-wide), `ld_US_ADDR_Q`, `st_US_ADDR_Q`/`st_US_DATA_Q` (VLEN-wide). SSI sized to the worst-case single-store element count (`ssiQueueEntries`, default 512); stores back-pressure the vAGEN when full.
- `VecLoadCoalescingBuffer.scala` (LCB) — VLEN-wide assembly per destination PRN; per-PRN byte-valid bitmap; fills inactive lanes per `vta`/`vma`; one VRF write per PRN; **emits one group-done (member-PRN vector) when the last PRN of the group lands**.
- Per-element progress (`elem_next`/`elem_done`/`fault_elem`) for precise `vstart`; fault-only-first (`vleff`) trims VL and writes the VL RF (Step 4 producer path).
- `CrossLsuSnoop.scala` — **bidirectional** disambiguation: vector store addr searches the LDQ (scalar + vector); scalar/vector loads search the STQ + vector store address queues (forward from `st_*_DATA_Q`). US = one range-overlap check; SSI = per-element through the arbiter.
- `DcacheArbiter.scala` — priority round-robin (scalar floor + anti-starvation) over the shared D$ lane(s), LCAM, and TLB ports; `single` (Medium) / `dual-dynamic` (Mega).
- The stage-2 Packer hook at the US queues (JIT element expansion).

**Files modified.**
- `lsu/lsu.scala` — vector LDQ/STQ as a **single entry** per `OP.v`; cross-LSU snoop wiring; replace any "vector-as-tertiary-priority" with `DcacheArbiter`.
- `lsu/dcache.scala` — no internal change; arbiter sits outside and presents the same `dmem.req`.
- `rob.scala` — `fence_pending` handshake; fence retires only when **both** LSUs drain.
- `core.scala` — instantiate the V-LSU, snoop, arbiter; wire W0/R0/R1, group-done, fence handshake; `pvtmp` handoff for segmented LS (LSU half ↔ coprocessor half via the VRF — the coprocessor half is stubbed in Milestone 1, so segmented LS that needs the transpose is **not** functional until Milestone 2; unit/strided/indexed loads/stores and pure scatter/gather are).

**Spec refs.** `loadstore.rst` (entire), `overview.rst` (Memory).

**Verification (9e2 + 9g).** The (e1) `vset` smoke must already pass. Sub-step gating then mirrors the spec:
- **11a** LQ/SQ/AGU + unit-stride (`single` arbiter) — unit-stride PASS subset.
- **11b** strided + indexed.
- **11c** segment (LSU memory-movement half; transpose deferred — gate only the memory portion).
- **11d** mask + whole-register + mask-load + fault-only-first (`vleff`); add `vsetvl` coverage ELFs.
- **11e** Mega + `dual-dynamic` arbiter + cross-LSU snoop under 2-lane traffic.
All ELF runs via VCS+Whisper (rule 11). The V-LSU modules emit trace (LCB per-PRN fill + group-done, per-element address/disambiguation, store drain, fence handshake) so an element stream can be followed and lined up against Whisper. **Risk:** largest step — split as above; 11a is the minimum viable deliverable.

---

## Step 12 — Tie off `IQ_V_ALU` / CII (defer arithmetic to Milestone 2)

**Scope.** Vector arithmetic is decoded/renamed/queued but never issues; the CII coprocessor is not attached.

**Files modified.**
- `core.scala` — `IQ_V_ALU` grant tied off (`issueWidth=0` in Milestone-1 mode via an `enableVectorArith` sub-flag, default off). Hard `assert(!iq_v_alu.iss_valid)`.

**Verification (9g).** (9d) scalar regression (no vector arith queued for scalar workloads) and the (e2) LS regression stay green. A single-`vadd.vv` sanity ELF may be added expecting the tie-off assertion.

---

## Step 13 — `core.scala` integration sweep

**Scope.** Final wiring sweep — wakeup port counts, branch-update fan-out, commit fan-out, `require()` invariants.

**Files modified.** `core.scala`, `rob.scala`, `parameters.scala`.

**Verification.** (9c)/(9d)/(9e) clean, no `WARN` rows; (9f) baseline diff strictly empty.

---

## Step 14 — Verification and sign-off

1. **Scalar baseline (vector off)** — `MediumBoomV4Config`, full csmith + Linux boot; byte-for-byte vs pre-Caracal.
2. **Scalar perf (vector on)** — `./run_regr_rvv_scalar.sh MediumBoomV4VectorConfig` all PASS; cycle delta reviewed (single-stage rename adds no scalar bubble; confirm the combined rename cycle doesn't regress fmax).
3. **vset + load/store regression** — `./run_regr.sh tests_regr/vset_loadstore_tests.txt MediumBoomV4VectorConfig` all PASS.
4. **Vector ISA cosim** (VCS+Whisper) — `rv64uv-p-*` LS subset at LMUL ∈ {1,2,4,8}, SEW ∈ {8,16,32,64}, ±mask; arithmetic tests expected to fail (V-ALU tied off); LS tests must pass.
5. **Performance counters** exposed: `vec_retire_count`, `vec_load_q_occ`/`vec_store_q_occ`, **non-whole-vector-group reads** (mapper observability), TLB-pressure split (`tlb_miss_scalar`/`tlb_miss_vec`/`tlb_port_stall_*`), MSHR-policy counters, and the **memory-bandwidth-ceiling** monitors (`lsuWidth × ELEN`).
6. **Docs** — keep `docs_caracal/src/*.rst` and the figures in sync; refresh `docs/boom-v4-architecture.md` (or a sister vec-architecture doc).

**Acceptance for Milestone 1.**
- vset + load/store regression all PASS; scalar perf all PASS (cycle delta reviewed).
- All vector load/store ISA tests pass Whisper cosim at LMUL ≤ 8.
- Atomic mapper handles widening (`vw*`) and segment LS group sizing; `vstart` precise on element faults.
- `usingRVV=false` RTL diff vs pre-Caracal v4 is empty.
- Every implemented module emits its gated `printf` trace (off by default), and the expected instruction flow — decode → mapper/VL rename → issue → AGEN → LCB/group-done → ROB commit — is visible and self-consistent (and consistent with Whisper) in the (9d)/(9e) simulation logs.

---

## Non-goals for Milestone 1 (deferred to Milestone 2+)

- **Vector arithmetic execution (CII coprocessor / VPU).** `IQ_V_ALU` built but tied off; reductions, permutes, mask-logic ops, and FP vector ops depend on it.
- **Segmented-LS transpose** — the LSU memory-movement half lands in Milestone 1; the coprocessor transpose half (and thus full segmented LS) waits on the CII attach.
- **VRF banking / port tuning** beyond 8R/4W.
- **Wide / line-granular vector cache port** — Milestone 1 reuses the scalar D$ port (`lsuWidth × ELEN` ceiling); a wider vector cache interface is the highest-leverage follow-on if memory-bound.
- **Performance tuning.** Milestone 1 is functional correctness.

---

## Cross-cutting risks and mitigations

| Risk | Mitigation |
|------|-----------|
| `*_rtype` 2→3-bit widening ripples | Done in Step 1; audited + tested before any vec logic |
| Atomic group rename free-list pressure | `SelectFirstN allocWidth=coreWidth*8`; reserve `numVecTmpGroups` so a segmented LS always gets `pvdest`+`pvtmp` |
| Per-member group-ready CAM is the timing pole | Accept the per-member match (unavoidable: source groups can be sub-ranges/fragmented); keep `EMUL ≤ 8`; group-done collapses the *count* into one event |
| Single-stage rename timing | Scalar and vector-group rename run **in parallel** in one cycle (independent register spaces) → cost = slower of the two, not the sum; vector group mapper (EMUL read + 8-PRN alloc + per-member bypass) must fit the rename cycle |
| Dispatch-group atomicity | Whole group (scalar + vector) renames + reserves ROB/LDQ-STQ/`br_tag` atomically in the single cycle; branch snapshots on the same `ren_br_tags` event — no skew, no delayed `br_tag`, no bubble |
| VCFG `vtype` mirror corrupted by mispredicted `vset` | Per-`br_tag` snapshot + committed shadow recovery (Step 2/4) |
| VL correctness | VL renamed into its own RF; producers VCFG / int-ALU-EU / LSU(`vleff`); explicit vector-CSR `is_unique` |
| `vstart` trap-resume coherence | `vstart` lives in the CSR file, read at execute (not snapshotted); set precisely on trap, cleared at commit |
| Segmented-LS temp lifetime | `pvtmp` is an ordinary VRF group from the main free list; freed at commit, reclaimed on branch — no TVRB, no free-on-consume |
| RVWMO scalar↔vector ordering | Bidirectional cross-LSU snoop + memory-dependency speculation + fence-drains-both (Step 11) |
| D$ / LCAM / TLB contention | `DcacheArbiter` priority-round-robin; `single` (Medium) / `dual-dynamic` (Mega) |
| Vector memory bandwidth ceiling | `lsuWidth × ELEN` is the cap; recommend Large/Mega; counters in Step 14; wide port is Milestone 2 |
| `IQ_V_ALU` accidentally issuing | Hard assertion in Step 12 |
| Largest-step risk (Unified LSU) | Split 11a–11e; 11a (unit-stride, single arbiter) is minimum viable |
| `vset` not proven before L/S bring-up | Step 9 lands the integer-ALU `vset` execution and gates on the **(e1) `vset` smoke test** (`tests/rvv/vset_test/vset_test.elf`) before any Step 11 L/S regression |

---

## File-touch summary

**Created (all under `src/main/scala/v4/vec/`):**

| Step | Path |
|------|------|
| 0 | `common/VectorParams.scala`, `common/package.scala` |
| 2 | `decode/VDecode.scala`, `decode/VLSDecode.scala`, `decode/VsetDecode.scala`, `decode/VConfigUnit.scala` |
| 3 | `rename/VecMapTable.scala`, `rename/VecFreeList.scala`, `rename/VecBusyTable.scala` |
| 4 | `rename/VecRenameStage.scala`, `rename/VlRename.scala` |
| 6 | `issue/VecIssueSlot.scala`, `issue/VecIssueUnit.scala` |
| 7 | `lsu/VecLsDecode.scala`, `lsu/ConfigInfo.scala` |
| 8 | `regfile/VecRegFile.scala`, `regfile/VlRegFile.scala` |
| 10 | `lsu/VecAgenStage1.scala`, `lsu/VecDgen.scala`, `lsu/Vec{Load,Store}{Packer,Skipper,Walker}.scala` |
| 11 | `lsu/VecLoadCoalescingBuffer.scala`, `lsu/CrossLsuSnoop.scala`, `lsu/DcacheArbiter.scala`, vector address/data queues, FSMs (`VecMemopFsm`/`VecIdxLsFsm`/`VecMaskFsm`) |

**Modified (baseline files — kept minimal):**

| File | Steps | Reason |
|------|-------|--------|
| `v4/common/parameters.scala` | 0, 6, 13 | `enableVector`/`VectorParams`, derived sizes, `require`s |
| `v4/common/config-mixins.scala` | 0 | `WithVector`, Mega overrides |
| `v4/common/consts.scala` | 1, 6 | `RT_VEC`; widen `iq_type`; `IQ_V_*` |
| `v4/common/micro-op.scala` | 1 | vector fields (`is_vec`, `pvs*`, `pvl`, `pvtmp`, `VConfig` vtype-only, nOP.v cursor) |
| `v4/exu/decode.scala` | 2 | hook `VDecode`/`VLSDecode`/`VsetDecode`; vector-CSR `is_unique` |
| `v4/exu/dispatch.scala` | 6 | route to `IQ_V_*`; dual-route shared instructions |
| `v4/exu/rob.scala` | 5, 11 | `dst_rtype`, group-done completion, stale-group free, CSR/vstart writes, fence handshake |
| `v4/exu/execution-units/*` (int ALU) | 9 | execute `vsetvli`/`vsetvl` → VL RF (gated `usingRVV`) |
| `v4/exu/core.scala` | 3,4,5,6,7,8,9,10,11,12,13 | top-level wiring of every new module; single-stage parallel scalar+vector rename |
| `v4/lsu/lsu.scala` | 11 | single-entry vector LDQ/STQ, cross-LSU snoop, arbiter |

---

## Implementation notes & deviations (Milestone 1 as-built)

This section records what actually changed during implementation of Steps 9–14 —
bugs found and fixed, design decisions/deviations from the plan above, and known
gaps. All RTL changes are `usingRVV`-gated; `MediumBoomV4Config` (vector-off) stays
byte-for-byte identical to pre-Caracal v4 (verified per-commit by re-elaborating and
diffing gen-collateral, ignoring only `@[...]` source-locators and `$error`
assertion-message line numbers).

### Step 11 (Unified LSU) — as-built scope

Delivered end-to-end (execute + Whisper cosim + scalar bit-identical), all on
`MediumBoomV4VectorConfig`:

- **11a unit-stride** vle/vse, incl. **LMUL>1** (multi-member groups), **masked**
  (mask-undisturbed), **tail-undisturbed** (partial vl), and the **whole 64b lane
  vs per-byte** write path.
- **11b strided** (vlse/vsse — Packer good-stride + Walker arbitrary-stride) and
  **indexed** (vluxei/vloxei/vsuxei/vsoxei gather/scatter).
- **11d whole-register** (vl1re*/vs1r), **mask load/store** (vlm/vsm), **fault-
  only-first** (vle*ff).

**Deviations from the plan:**

1. **REUSE the scalar LDQ/STQ (plan was right; an early impl pass was wrong and was
   reverted).** A vector load holds ONE scalar LDQ entry, a store ONE STQ entry, as
   the ordering + commit placeholder; the cracked 64b beats are VecLSU's own queue,
   NOT LDQ/STQ slots. `VDecode` sets `uses_ldq := is_load` / `uses_stq := is_store`.
   Dual completion on the last beat: `ld_done`/`st_done` (writes the placeholder
   entry succeeded ONCE, so it retires at the ROB head) AND `clr_rob` →
   `rob.vec_clr_bsy` (a `RT_VEC` load has no iresp writeback to clear `rob_bsy`) plus
   a `VecGroupDone` (prn) that clears the dest-group busy bits in rename/issue.

2. **M1 beat EA is PHYSICAL (bare-mode), NOT virtual.** The plan anticipated the
   Packer EA going through the TLB; M1 instead reuses the scalar 64b D$ port at
   LOWEST priority (appended-last `lsu_sched`, DC-only, `uses_tlb=false`,
   `uses_lcam=false`). Each 64b beat is one VRF lane. (Full virtual EA + LCAM
   disambiguation vs younger scalar stores is post-M1.)

3. **`group_done` / `clr_rob` / the LCB beat are driven from the REGISTERED beat,
   never combinationally from `load_nop`/`store_nop`** — else
   issue.vec_wakeup → iss_uops → decode → AGEN → nop → group_done forms a comb loop.
   A fake/bypass beat is finalized one cycle later in a dedicated `sFin` state.

4. **Tail-/mask-undisturbed needs a stale-group COPY.** With renaming, a masked or
   partial-vl load writes a *fresh* physical dest, so the undisturbed lanes would be
   garbage. VecLSU copies each `stale_pvdest_grp` member → `pdst_grp` member (full
   256b) BEFORE placing beats, gated on `!v_unmasked || tail_undist`
   (`tail_undist = loaded_bytes < group_capacity`, computed in the AGEN remap). A
   full unmasked load skips the copy (and never reads the uninitialized stale reg).

5. **Per-BYTE VRF write mask.** The VecRegFile write port masks per byte
   (`vecVLen/8` bits), not per 64b lane — required so a sub-lane write (vlm's
   `ceil(vl/8)` bytes, a partial tail) leaves the rest of the lane undisturbed.

6. **Masked STORE routes to the Skipper, not the Packer.** The store Packer has no
   mask support (`is_fake` hardcoded false); the store Skipper does. `packable` was
   made to exclude masked stores (`isStore && is_mask`). Loads are unchanged (the
   load Packer is mask-aware).

7. **Masked/indexed operands are read late, in `VecLSRegRead`.** The mask (v0) and
   the index vector (vs2 → `pvs2`) are read from dedicated VecRegFile read ports in
   the operand-read stage and streamed to the AGEN; a new `VecIdxGen` module
   serializes the index vector into per-element signed offsets for the Walker.

8. **`vleff` is a plain load in M1.** M1 bare-mode addressing cannot fault, so vleff
   can never trim vl → it behaves as a normal vle that leaves vl unchanged, which is
   architecturally correct for M1 and matches Whisper. It is deliberately NOT marked
   a VL producer (the `is_vleff` producer hooks in VlRename/ROB stay dormant); the
   fault-trim path is Milestone 2 (needs faults = the TLB/virtual addressing).

9. **Segment LS (11c) is Milestone-2-blocked.** Caracal segment LS is a two-half op
   (LSU writes an intermediate `pvtmp` group; the CII coprocessor transposes
   `pvtmp`→`pvdest`). `is_shared` stays false and the ROB waits for a second
   group-done from the not-yet-existent CII, so full segment LS cannot complete in
   M1. Only the memory-movement half is buildable groundwork.

10. **Misaligned unit-stride deferred** (rare; needs cross-beat byte assembly the M1
    "one beat = one clean lane" LCB does not do).

### Bugs found and fixed during bring-up (not anticipated by the plan)

- **VecFreeList double-free / off-by-one (Step 4/5 latent, exposed by the first
  committing vector dest op):** the vector rename (`VecRenameStage`/`VlRename`) was
  single-stage (combinational off `dec_uops`), a cycle AHEAD of the scalar
  `RenameStage`'s registered ren1→ren2 pipeline (which allocates at `dis_fire`). At
  dispatch the vec fields reflected the NEXT cycle's (bubble) uop → both ops freed
  PRN 0. **Fix:** feed the vec renames `rename_stage.io.ren2_uops/ren2_mask` +
  `dis_fire` so they alloc/read in lockstep with dispatch. Also `rob_unsafe` was
  never cleared for vec ops (tripped the PNR assert) → `vec_clr_bsy` now clears it;
  and VL producer-pvl / intra-bundle pvl-busy fixes.

- **Cosim couldn't see vector results (Step 11a.2):** `harness.commit.uops[w].
  debug_vec_wdata/wmask` were hardwired 0. Added combinational VecRegFile debug read
  ports; the commit trace reads back `vec_regfile[vec_remap.pdst]` so the loaded
  result is visible to the arch checker. (Debug-only; not synthesized.)

- **vse teardown hang:** the vector-store STQ placeholder never executes via the
  normal path (addr/data never set), so `can_enq_store_execute` was false for it and
  `stq_execute_head` never advanced past it — stranding every younger scalar store
  (incl. the HTIF tohost write) → sim ran to timeout even though the test passed.
  **Fix:** like a fence, advance `stq_execute_head` when a committed+succeeded
  vector-store placeholder is cleared.

- **vec-LS stale scalar base/stride (prs1 RAW race):** the vector LS is woken
  speculatively, so `VecLSRegRead`'s int-RF read fired the SAME cycle the base GPR's
  writeback committed; the int RF is a registered `Mem` read with no read-during-
  write bypass → stale base → wrong/zero address. Masked in every test by ≥2 instr
  of `la→use` slack. **Fix:** `VecLSRegRead` snoops the int writeback bus and
  forwards on a same-cycle match.

- **`vsetvli` AVL > VLMAX wrapped (found by the Step-14 SEW/LMUL matrix):** the vl
  compute truncated AVL to `vecVLSz+1` bits, so a large AVL wrapped instead of
  saturating to VLMAX (e.g. AVL=2048 → vl=0). Breaks the canonical strip-mining
  pattern (AVL = remaining count). **Fix:** compare the FULL-width AVL vs VLMAX.

- **vec-store issue grant dropped on simultaneous load grant (Step 14):**
  `VecLSRegRead` is a serial, fire-and-forget consumer shared by the V-LOAD and
  V-STORE issue units with a LOAD-priority mux; when both granted the same cycle the
  store's grant was silently lost → hang. **Fix:** the store does not advertise
  `FC_AGEN` in a cycle the load is granting.

- **back-to-back vector-store data corruption (found by the strip-mine memcpy):**
  `VecDgen.num_members` was hardcoded 8, so after a 1-member store it streamed
  phantom members and stalled `active`; and the vec-LS serialization did not include
  `vec_dgen.active`, so the next store issued while VecDgen was still busy and reused
  the previous store's stale `pvs3` → garbage. **Fix:** VecDgen completes by TOTAL
  bytes (`vl<<eew`, handles partial members) and `agen_active` ORs `vec_dgen.active`.

### CSR / misa

`misa.V` is now advertised for vector configs (`override def hasV = enableVector`
in `BoomCoreParams`; `hasV` feeds only the misa string). The Whisper cosim reference
(`boom.json`) misa reset was corrected to match (drop the spurious `X` bit, keep
`V`), so tests that probe `csrr misa` no longer mismatch. `misa` itself is built
normally (not hardcoded); the missing `V` was a consequence of `vfLen=0` making the
default `hasV = vLen>=128 && eLen>=64 && vfLen>=64` false.

### Step 12 — IQ_V_ALU tie-off

`IQ_V_ALU` `fu_types` stay 0 (never grants). Added the hard invariant
`assert(!valu_iss_unit.iss_uops(0).valid)` to catch any future accidental grant
wiring. A vector-arith op (vadd/vmv) in M1 is decoded/renamed/queued but never
executes → the pipeline hangs at `boom_timeout` (the observable, expected M1
outcome; there is no clean fail-fast without false-firing on a squashed speculative
op). Milestone 2 attaches the CII and gates this behind an `enableVectorArith` flag.

### Verification (Step 14) — status

Passing on `MediumBoomV4VectorConfig` via VCS+Whisper cosim:
- Full **SEW {8,16,32,64} × LMUL {1,2,4,8}** unit-stride vle/vse matrix (16 tests).
- **Strip-mined memcpy** (AVL > VLMAX loop, vl = 4,4,2).
- unit-stride / strided / indexed / whole-register / mask-load / vleff, ± mask,
  tail-undisturbed (the `ms11*` + `ms14*` suites in `tests_regr/vset_loadstore_tests.txt`).
- **vset smoke** (16 configs) and **scalar perf** regression (`run_regr_rvv_scalar.sh`,
  9 kernels) on the vector config.
- `MediumBoomV4Config` bit-identical throughout.

**Performance counters (14.5) — done.** A 4th `perfEvents` EventSet (index 3,
`usingRVV`-only) exposes 6 vector HPM events via `mhpmevent`/`mhpmcounter`: vec
insn retire, vec ld/st retire, vec load issued, vec store issued, vec LS pipe busy,
vset retire. Vector-off configs keep the original 3 sets (bit-identical). The
plan's TLB-split / MSHR-policy / bandwidth-ceiling monitors are Milestone 2 (M1 vec
LS is bare-mode DC-only, reusing the scalar D$ port — no separate vec TLB/MSHR);
non-whole-group-read observability needs VecMapTable plumbing (M2). HPM counters are
micro-architectural, so they are NOT cosim-comparable (a software read-back would
mismatch Whisper under lockstep); validation is by elaboration + wiring +
bit-identical.

Remaining M1 item: a **docs refresh** (14.6) of the sister architecture doc — this
"as-built" section serves as the plan-doc refresh; `docs/boom-v4-architecture.md`
may still want a vector-pipeline addendum.

**Known verification gap:** the Whisper cosim does NOT deeply compare vector-STORE
memory data — a garbage vector store passes the per-instruction check and is only
caught by a subsequent scalar load-back (this is how the back-to-back store-data bug
above was found). A memory-compare cosim hook is a recommended follow-on.

### Old bring-up tests that legitimately fail in M1

`tests_regr/vset_loadstore_tests.txt` also lists older `ms2/ms4/ms5` bring-up tests
and `stress_seg`. These use vector ARITHMETIC (`vmv`, etc.) or segment LS, so they
hang/fail in M1 by design (IQ_V_ALU tied off; segment transpose is M2). Only the
`ms11*`/`ms14*` section is the validated Caracal M1 LSU suite.

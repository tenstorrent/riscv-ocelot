# Caracal Milestone 2 Implementation Plan

**Objective.** Complete the BOOM v4 vector datapath on top of Milestone 1. M2 has three
tracks:

1. **Track A — Complete the unified LSU (memory ordering & disambiguation).** M1 shipped a
   working unit-stride/strided/indexed vector load/store path, but its beats are
   *bare-physical* (`uses_tlb=false`, `uses_lcam=false`) and hold only a single
   ordering/commit *placeholder* entry in the scalar LDQ/STQ whose address is never
   registered — so the scalar LCAM cannot see vector addresses and no cross-LSU
   disambiguation happens (M1 as-built deviation #2). M2 implements the full
   `loadstore.rst` §Memory Ordering and Disambiguation design: real element EAs through the
   DTLB, **US range-overlap** and **SSI per-element** cross-LSU snooping, store→load
   ordering with the existing `order_fail` replay path, and load→store data forwarding from
   the vector store data queues.

2. **Track B — Vector arithmetic via the CII coprocessor: the host side.** M1 built
   `IQ_V_ALU` but tied it off. The TT-CII spec + RTL now exist (`src/main/sv/v4/tt-cii`),
   so this track is fully specified: attach `IQ_V_ALU` to the CII Issue channel, service
   operand pulls from the VRF, and write results back — the BOOM-side adapter (`VecCiiHost`).

3. **Track C — Vector arithmetic via the CII coprocessor: the coprocessor (VPU) side.**
   Refactor the SV VPU (`src/main/sv/v4/vpu`) so its top level speaks the CII protocol —
   receive instructions, pull source data, push results — with the execution datapaths left
   largely unchanged. The VPU keeps its vector regfile **as a per-instruction CII staging
   buffer** (the architectural vector RF lives in BOOM).

The M1-deferred items that unblock on the above (segmented-LS transpose, real `vleff`
fault-trim, wide vector cache port) are folded into these tracks.

**Reference.** The architecture spec is `docs_caracal/src/*.rst` (built HTML under
`docs_caracal/_build/`). The LSU spec of record for this milestone is `loadstore.rst`,
specifically §*Memory Ordering and Disambiguation* (`.. _mem-order:`) including the
`order-fail-replay` subsection and the SSI store→younger-load serialization edge case.
The **CII coprocessor spec is TBD** (to be added as `docs_caracal/src/cii.rst`); Track B
cannot be finalized until it exists. M1 as-built state is
`caracal-milestone1-plan.md` §*Implementation notes & deviations* — treat its deviations
#2 (bare-physical beats), #8 (`vleff` is a plain load), and #9 (segment LS is
M2-blocked) as the M2 starting point. Prior OVI vector integration on `bobtail/main`
remains reference material for SystemVerilog primitives (`src/main/resources/vsrc/vpu/`).

**Branch strategy.** Each step below is a `Caracal/m2/<step-slug>` feature branch that
merges into `Caracal/m2` and ultimately into `Caracal/main`. Steps are ordered so that
every intermediate state compiles, elaborates, and (when `usingRVV=false`) is
bit-identical to baseline.

Whenever you are uncertain about implementation details and planning, ask/prompt the user.

---

## Global ground rules

1. **Feature flags.** All vector logic stays gated by the M1 `usingRVV` derived flag.
   Two sub-flags gate the M2 tracks so each can land independently and stay off by
   default until proven:
   - `vecScalarSnoopEnable` (already in `VectorParams`, default `false`) → flipped `true`
     to turn on the LSU cross-LSU disambiguation (Track A). Off = M1 bare-physical
     behavior, bit-identical to M1.
   - `enableVectorArith` (M1 Step 12 sub-flag, default `false`) → gates the CII attach and
     `IQ_V_ALU` grant (Track B). Off = M1 tie-off (`assert(!valu_iss_unit.iss_uops(0).valid)`).
   Every existing config and test must remain bit-identical with both sub-flags off.
2. **Code location.** All new Chisel under `src/main/scala/v4/vec/`. Track A adds to
   `vec/lsu/`; Track B adds `vec/cii/`. Package `boom.v4.vec.{lsu,cii,…}`. Touch baseline
   files (`lsu/lsu.scala`, `core.scala`, `rob.scala`, `tlb`) only where unavoidable.
3. **Minimize intrusive edits.** Whenever logic *can* live in `v4/vec/` rather than a
   baseline file, put it there — even at the cost of a little duplication. The LCAM
   comparison framework in `lsu/lsu.scala` is the one unavoidable baseline surface
   (Track A extends it).
4. **Parameters.** Reuse M1 `VectorParams`. The M1-reserved-but-unused knobs now go live:
   `ssiQueueEntries` (default 512, worst-case single-store element count), the US/SSI
   queue depths, and `dcacheArbiterMode` (`single` on Medium, `dual-dynamic` on Mega).
   Add CII-side parameters only once the CII spec fixes them (TBD).
5. **Completion model unchanged.** Every vector producer signals completion of a whole
   destination group **once** via group-done (member-PRN vector). Loads: the LCB emits it
   (M1). Arith: the CII emits one per `OP.v` (Track B). No per-entry ROB completion counter.
6. **Precise exceptions become real.** Once vector beats translate through the DTLB
   (Track A, A0), element-level faults are possible: vector LS updates `vstart` to the
   oldest faulting element; stores translate/disambiguate all active elements pre-commit
   and do not write the DCache until ROB-committed. `vleff` trims VL on a fault at
   element > 0 (A5). (M1 bare-mode could not fault, so `vleff` was a plain load.)
7. **RVWMO ordering is the correctness bar.** Within a hart a younger load must observe an
   older overlapping store in program order without a fence. Track A closes the M1 hole
   where a younger scalar load could bypass an older vector store. The mechanism is the
   `loadstore.rst` §mem-order design: register vector addresses into the LCAM and reuse
   BOOM's `order_fail` → refetch replay (see `order-fail-replay` subsection).
8. **Reuse the existing LSU machinery.** Track A reuses the scalar LCAM comparison
   framework and its age helpers rather than a parallel CAM: `GenByteMask`,
   `IdxAgeOt`/`IdxAgeYoungerThan`, `EntryValidFromAge`, `IsOlderLSU`, `GetRealLSQIdx`,
   `ForwardingAgeLogic`, `LSUAgePriorityEncoder` (all in `lsu/lsu.scala`). Vector US
   accesses add a *range-overlap* predicate alongside the scalar dword-match; SSI adds a
   per-element search stream through the arbiter.
9. **Per-step verification gate (universal).** Every step is "done" only when **all** pass
   (identical to M1's (9a)–(9g)):

   a. `sbt compile` clean inside `generators/boom/`.
   b. `make checkstyle` clean inside `generators/boom/`.
   c. **Base Chipyard build with vector enabled** — `make CONFIG=MediumBoomV4VectorConfig
      -j$(nproc) debug` in `sims/vcs`.
   d. **Scalar performance regression with vector enabled** —
      `./run_regr_rvv_scalar.sh MediumBoomV4VectorConfig` all PASS.
   e. **Vector regression — phased.** Run in order:
      - **(e1)** `vset` smoke — `./run_regr.sh tests/rvv/vset_test/vset_test.elf …`.
      - **(e2)** `vset` + load/store — `./run_regr.sh tests_regr/vset_loadstore_tests.txt …`.
      - **(e3) LS disambiguation / ordering stress (NEW)** — required from the Track-A
        steps onward: a new `tests_regr/vec_mem_order_tests.txt` of directed alias ELFs
        (vector-store→younger-scalar-load, scalar-store→younger-vector-load, SSI
        store→younger SSI load, partial-overlap forward, page-crossing beat) run under
        VCS+Whisper; PASS + 0 MISMATCH.
      - **(e4) vector-arith regression (NEW, TBD)** — gated behind the CII attach; the
        suite and gate are defined once the CII spec lands.
   f. **Baseline `usingRVV=false` bit-identical to pre-Caracal v4.**
   g. The step's listed step-specific verification artifacts pass (the module `printf`
      trace shows the expected flow in the (9d)/(9e) logs, cross-checked vs Whisper).

   Don't merge a step branch until (a)–(g) are green.
10. **No unit tests — module `printf` tracing instead.** Same as M1: no
    `chiseltest`/`*Spec.scala`; every new module emits gated, tagged, one-line-per-event
    `printf` traces (conditioned on a `vecTrace` plusarg, off by default and out of the
    bit-identical baseline) carrying the module name + `rob_idx` (+ `ldq_idx`/`stq_idx`,
    element index, matched entry, `order_fail` for LSU events). These traces are the (9g)
    artifact.
11. **End-to-end ELF-test convention.** Every ELF test goes through VCS with the Whisper
    cosim sidecar — never Verilator. New ELFs are appended to `sims/vcs/tests_regr/*.txt`.
    **Note the M1 known gap:** Whisper cosim does not deeply compare vector-STORE memory
    data, so a store-ordering bug can pass the per-instruction check and only surface via
    a subsequent scalar load-back — the (e3) alias tests are deliberately built as
    store-then-load-back so the load-back catches ordering errors. A memory-compare cosim
    hook remains a recommended follow-on.

---

## Track A — Complete LSU: memory ordering & disambiguation

Track A realizes `loadstore.rst` §mem-order. It reuses the scalar LCAM comparison
framework in `lsu/lsu.scala` (search pipeline: S0 addr-gen/TLB → S1 register → S2 search;
`do_st_search`/`do_ld_search` gate the per-entry loops; `ldq_order_fail` drives the
`MINI_EXCEPTION_MEM_ORDERING` → refetch replay). The M1 vector datapath (`VecLSU` serial
single-beat FSM presenting one physical beat address at a time on `vec_dmem.req`) is the
substrate the snoop taps.

### Step A0 — Vector beat EA through the DTLB (replace bare-physical)

**Scope.** Route `VecLSU` beats through the shared DTLB so both sides of every comparison
are translated physical addresses (a correctness prerequisite for disambiguation and for
real faults). Handle per-beat page-crossing.

**Files modified.**
- `vec/lsu/VecLSU.scala` — beat request carries a virtual EA; add a translate state so a
  beat waits on TLB response before the D$ request; on TLB miss/fault, latch the faulting
  element index for `vstart`/`vleff`.
- `lsu/lsu.scala` — `can_fire_vec_load`/`can_fire_vec_store` now set `uses_tlb=true`
  (still appended-last / scalar-floor priority); feed the vec beat vaddr into the TLB
  arbiter alongside scalar; return `paddr`/miss/fault to `vec_dmem`.
- `vec/lsu/VecLoadWalker.scala`/`VecStoreWalker.scala`/`Packer` — no longer assume
  identity mapping; a page-crossing unit-stride beat splits at the page boundary.

**Verification (9g).** Trace shows each beat's vaddr→paddr and any TLB miss/fault with the
element index; a page-crossing `vle` in the (e2) log translates each half. (e1)/(e2) stay
green; (e3) not yet required.

### Step A1 — Disambiguation substrate: address/data queues + CrossLsuSnoop + DcacheArbiter

**Scope.** Build the structures `loadstore.rst` §mem-order and the M1 plan (Step 11) named
but never implemented.

**Files created (under `vec/lsu/`).**
- Vector address/data queues: `st_SSI_ADDR_Q`, `st_SSI_DATA_Q` (ELEN-wide), `st_US_ADDR_Q`,
  `st_US_DATA_Q` (VLEN-wide), `ld_SSI_ADDR_Q`, `ld_US_ADDR_Q`. SSI sized to
  `ssiQueueEntries` (512); stores back-pressure the vAGEN when full. Store data is captured
  from `VecDgen` at execute (so the source `pvs3` needs no pin — frees with the stale group
  at commit, per `loadstore.rst` Store Data Queue).
- `CrossLsuSnoop.scala` — **bidirectional** disambiguation, reusing the `lsu.scala` age
  helpers: (i) a vector store address searches the LDQ (scalar + in-flight vector loads);
  (ii) a scalar/vector load address searches the STQ + vector store address queues.
  **US = one range-overlap check** of `[base, base+VL*EEW)`; **SSI = per element** as beats
  drain. Emits `order_fail` set / forward-select back into the `lsu.scala` framework.
- `DcacheArbiter.scala` — priority round-robin (scalar floor + anti-starvation) over the
  shared D$ lane(s), the LCAM search port, and the TLB port; `single` (Medium) /
  `dual-dynamic` (Mega). Replaces M1's "vector-as-lowest-priority appended `lsu_sched`
  term" so vector element searches contend fairly without starving scalar disambiguation.

**Files modified.**
- `lsu/lsu.scala` — expose the LCAM search port and the `ldq_order_fail`/forward-select
  hooks to `CrossLsuSnoop`; feed vector addresses into `lcam_addr`/`lcam_mask` muxes.
- `core.scala` — instantiate the queues, `CrossLsuSnoop`, `DcacheArbiter`; wire to
  `VecLSU`, `VecDgen`, and the scalar LSU. Gate all of it on `vecScalarSnoopEnable`.

**Verification (9g).** Substrate only — no behavior change until A2. Trace shows queue
enqueue/drain and snoop search fire; (e1)/(e2) green with the snoop enabled but before the
ordering action is armed; (9f) bit-identical with `vecScalarSnoopEnable=false`.

### Step A2 — ST→LD ordering (minimum-viable correctness): US range-overlap + order_fail replay

**Scope.** The correctness fix. A vector store address (US range or SSI element) that
matches a **younger, already-executed** load sets that load's `ldq_order_fail`, driving
BOOM's existing `MINI_EXCEPTION_MEM_ORDERING` → `flush_typ=refetch` replay (see
`loadstore.rst` `order-fail-replay`). US uses one range-overlap test; the load and
everything younger squash and refetch, re-rename, re-execute.

**Files modified.** `vec/lsu/CrossLsuSnoop.scala` (arm the ST→LD path, US granularity);
`lsu/lsu.scala` (accept the vector-driven `order_fail`).

**Verification (9e3 + 9g).** Directed alias ELF: vector store then younger scalar load to
the same address; without the fix the load-back reads stale data (or Whisper MISMATCH),
with it the load replays and reads correct data. PASS + 0 MISMATCH; (e1)/(e2) green.
**This step alone closes the documented M1 memory-ordering hole.**

### Step A3 — SSI per-element search + SSI→SSI serialization edge case

**Scope.** Extend the ST→LD search to SSI per element through the `DcacheArbiter`.
Implement the `loadstore.rst` edge case: an SSI store → younger overlapping SSI load
**serializes** (the load is held until the older SSI store's elements resolve/drain,
because a not-yet-generated store element may alias and a later element may hold the
youngest byte — forwarding from a partial scatter is unsafe). The `order_fail` replay
remains the correctness floor if a load slips through.

**Files modified.** `vec/lsu/CrossLsuSnoop.scala` (SSI per-element stream, the hold);
`vec/lsu/VecLSU.scala` (respect the hold on a younger SSI load).

**Verification (9e3 + 9g).** SSI store (scatter) then younger SSI load (gather) overlapping
one element; trace shows the load held until the store drains, then correct data; PASS.

### Step A4 — LD→ST forwarding from the vector store data queues

**Scope.** Performance, not correctness. A load address search that finds an older vector
store fully covering it forwards data from `st_US_DATA_Q`/`st_SSI_DATA_Q` (US range /
SSI per element) via the existing `ForwardingAgeLogic` + StoreGen/LoadGen forward mux in
`lsu.scala`. Partial cover → replay (as scalar partial-forward does).

**Files modified.** `vec/lsu/CrossLsuSnoop.scala` (forward-select from vector store data);
`lsu/lsu.scala` (route forwarded data into the load response / `vec_dmem.resp`).

**Verification (9e3 + 9g).** Store-then-load with full overlap forwards (no D$ round-trip
in trace); partial overlap replays; PASS + 0 MISMATCH.

### Step A5 — Real `vleff` fault-trim + precise `vstart`

**Scope.** With beats now faulting (A0), make `vleff` a real VL producer: a fault at
element 0 is a precise trap (`vstart=0`); a fault at element i>0 trims VL to i and
completes (writes the VL RF, wakes `pvl` dependents, updates the `vl` CSR at commit — the
producer hooks M1 left dormant). All vector LS write precise `vstart` on element faults.

**Files modified.** `vec/lsu/VecLSU.scala` (latch oldest faulting element, drive VL-RF
write for `vleff`); `rob.scala` (`vstart` set on trap from the LSU faulting-element port);
`vec/rename/VlRename.scala` (re-arm the dormant `is_vleff` producer path).

**Verification (9e3 + 9g).** A `vleff` crossing an unmapped page trims VL to the fault
element and matches Whisper; a mid-vector faulting `vle` resumes at `vstart`.

### Step A6 — Wide vector cache port + Mega dual-dynamic arbiter (bandwidth)

**Scope.** Lift the M1 `lsuWidth × ELEN` bandwidth ceiling (M1 non-goal). A wider /
line-granular vector cache interface and the `dual-dynamic` arbiter on Mega. Performance
work; correctness already established by A2–A4.

**Files modified.** `vec/lsu/DcacheArbiter.scala`, `vec/lsu/VecLSU.scala`, `lsu/dcache.scala`
(port width), `core.scala`.

**Verification (9g).** Bandwidth counters improve on a streaming `vle`/`vse` kernel; all of
(e1)–(e3) stay green; scalar (9d) unaffected.

---

## Track B — Vector arithmetic via the CII coprocessor

The TT-CII spec + RTL now exist at `src/main/sv/v4/tt-cii` (see `docs_caracal/src/cii.rst`
for the pointer). Track B attaches the CII to `IQ_V_ALU` so vector arithmetic executes,
and wires the VRF/VL-RF read (operand pull) and write (result) ports.

### The CII interface (generic, 4 credit-metered channels — `tt_cii_interface.sv`)

The CII is loosely-coupled: four unidirectional, credit-metered channels (each a
`tt_cii_channel` credit FIFO, default depth 16). Handshake is **credit-based** (no
`ready`); a sender holds a free-running credit counter and stalls at zero.


Type params (`cii_coproc_pkg_t`): `INSTR_T=32b`, `TAG_T=8b` (≤256 in-flight),
`SRC/DST_DATA_T=256b` (= VLEN), `SRC_ID/OFFSET=8b`, `WB_STATUS=6b` (FP flags). The CII is
variable-latency and may complete internally out-of-order; results are correlated by
`tag`. LMUL groups are handled as **one transaction per instruction** with per-member
operand/result beats selected by `op_offset`/`wb_dst_offset`. (The verification tree
references CV-X-IF, but TT-CII is its own 4-channel protocol, not CVXIF.)

### What M1 already fixed (the CII contract)

- **`IQ_V_ALU` is age-ordered collapsing with a per-entry past-PNR gate.** Grants only when
  the entry's operands are ready **and** the entry is **past the PNR**
  (`is_older(rob_idx, rob.io.rob_pnr_idx)`, applied **per entry**, not head-only), RoCC-style.
  Squashed entries drop from the queue before issue, so the CII needs no **branch**-kill —
  **this lets M2 skip the speculative *rename* model** (shadow table, `cv0–cv31` copy regs)
  described in `tt-cii/docs/specs/protocol/interface_details.adoc`. It does **not** skip
  drain-REQ/drop-WB on redirect: past-PNR is not a commit guarantee, so a ROB-head flush can
  squash accepted CII work — that is exactly what Step A9 implements.
- **One group-done per `OP.v`** (member-PRN vector) drives the ROB single-shot busy-clear,
  Busy-Table clear, and VECTOR wakeup — identical to the LCB's load completion
  (`VecGroupDone{prn[8],mask[8]}`; `core.scala:851,888`).
- **Segmented-LS transpose is the CII's other half.** `is_shared` uops wait for a
  group-done from **both** halves via `rob_other_half_pending` (`rob.scala:426,482-492`);
  the CII driving the second group-done unblocks full segment LS (M1 deviation #9).
- **Feeders/wakeup:** `.vx` on INT, `.vf` on FP, vector operands on VECTOR, `pvl` on VL —
  all already delivered to `VecIssueSlot`.
- **VRF/VL-RF headroom:** `VecRegFile(7R,4W)`, statically partitioned and **0-based**
  (canonical table in `midcore.rst` §Port assignment). M1 owns Load/LCB `R0`–`R2` + `W0`–`W1`
  and Store `R3`–`R4`; **the CII owns `R5`–`R6` and `W2`–`W3`**. Static partitioning (not
  arbitration) is required: the Writeback channel is credit-metered with no back-pressure line,
  so the CII must never stall on a VRF port.
- **Gating:** behind a new `enableVectorArith` flag (default off); the M1 tie-off
  `assert(!valu_iss_unit.iss_uops(0).valid)` (`core.scala:1491-1501`) is removed only when
  the CII attaches under that flag.

### Design decisions (M2)

1. **Non-speculative past-PNR issue** — no shadow table / copy regs (see contract above).
   *(Speculative issue for perf is deferred to M3.)*
2. **Chisel↔SV bridge = flatten wrapper + BlackBox.** The CII SV uses interfaces/modports/
   parameterized packed structs that Chisel `BlackBox` cannot bind directly. Add a thin SV
   wrapper `tt_cii_host_wrap.sv` exposing the *host* modport as flat ports; BOOM
   instantiates it as a `BlackBox` behind a flat `CiiHostIO` Chisel Bundle. *(Alt: reimpl
   the credit relay in Chisel — more work, no SV dep.)*
3. **vtype/vl/vxrm carried in the extended Issue packet.** The SV has no CSR/config
   channel, so the shared ``cii_issue_req_t`` gains `{vtype{sew,lmul,vta,vma}, vl, vxrm}`
   (host fills from `uop.vconfig` + `pvl` read + `csr.io.vector.vxrm` at issue); the
   coprocessor latches it per-op. This is a coordinated **tt-cii SV struct change** (host +
   VPU + `tt_cii_interface.sv`), not a BOOM re-architecture. *(Rejected alt: a serializing
   VCONFIG config-instruction on `vset` — adds a serialization point + coprocessor config
   state.)*
4. **Scalar `.vx`/`.vf` operands captured at issue** into the tag side-table (INT/FP value
   from bypass at grant) and served when the CII requests that `op_id`.
5. **The VPU (coprocessor) applies tail/mask — not the host.** The VPU pulls the `v0` mask
   (and the old destination group, for undisturbed) as CII operands and applies `vta`/`vma`
   from the instruction encoding, returning **fully-formed VLEN results**. The host writes
   CII writeback data back to the VRF **verbatim** (`wb_wr_en`/full mask) — no host-side
   vta/vma. (Realized on the coprocessor side by Track C.)

### EMUL / dest-mask correctness (validated against decode)

`pvdest_grp_mask = memberMask(v_emul)` with `v_emul` derived at `core.scala:804-810`. Result
placement + completion depend on it being the true destination member count. Validation:

- **Widening** (`v_emul = lmul+1`) and **narrowing** (`v_emul = lmul`; `vtype.SEW`/`LMUL`
  describe the *narrow* operands, so dest EMUL = LMUL) — **both correct.** No change needed.
- **Reductions** and **mask-writing compares** (`vmseq`/`vmslt`/…) write a **single**
  destination register, but the derivation has no case → it would claim `2^lmul` members.
  Neither is decoded in M1. **B1/B-arith prerequisite:** add an `is_reduction` /
  mask-writing decode flag that forces `v_emul := 0` (single-member dest). With the `last`
  bit this no longer hangs completion, but it is required for correct placement.
- **Source-side asymmetry (operand pull):** the *same* `v_emul` (= dest emul) also drives
  the source group reads (`VecRenameStage:90/97/137`), so for widen (sources over-read at
  2×) / narrow (wide source under-read at ½) the **per-source** member count is wrong.
  Latent in M1 (arith never executes). **B2 prerequisite:** carry a separate **source EMUL**
  (or per-source EEW→EMUL) so the CII operand pull requests/serves the right member count
  per source. (Shared with Track C's iterate↔offset mapping.)

### Steps

#### Step B0 — CII host adapter skeleton + SV bridge + credit channels

**Files created.** `src/main/sv/v4/tt-cii/src/tt_cii_caracal_pkg.svh` (**done** — the
Caracal parameterization of `tt_cii_interface`: concrete type params + sizes; folds
vtype/vl/vxrm into `INSTR_T`, `last`/`dst_kind` into `WB_STATUS_T`, op-id slot enum; shared
by the host adapter and the VPU wrapper), `vec/cii/CiiBundles.scala` (flat Chisel mirror of
the four channel structs), `vec/cii/VecCiiHost.scala` (the host adapter),
`src/main/resources/vsrc/tt_cii_host_wrap.sv` (flatten wrapper over `tt_cii`'s host modport),
a `BlackBox TTCii`. Per-channel credit counters (depth 16). All gated on `enableVectorArith`
(`VectorParams`, default false → M1 tie-off stays bit-identical). `VecCiiHost` mirrors the
`VecLSU` IO shape (`group_done`, `clr_rob`, `kill`, `busy`, VRF read/write handles) so it
drops into the same `core.scala` wiring pattern.

**Why a bridge.** A Chisel `BlackBox` can only bind **flat `Bits` ports** — it cannot bind a
SV `interface`/`modport` port, nor ports typed by `parameter type` packed structs (exactly
what `tt_cii_interface` and the VPU use). So the Chisel side speaks a **flat** 4-channel
Bundle to a `BlackBox`, and a thin SV wrapper repacks flat ⇄ structs and hides the interface,
the credit relay, and the VPU coprocessor inside.

**Topology (hierarchy).**
```
CHISEL (BOOM)                              |  SV (inside BlackBox TTCii)
valu_iss_unit (IQ_V_ALU)                   |
  .io.iss_uops(0): Valid[MicroOp] ─┐       |
  .io.fu_types(0)  ◄────────────┐  │       |
                   ┌────────────▼──▼────┐  |
                   │     VecCiiHost      │  |  Chisel adapter (instantiated like VecLSU)
                   │  tag alloc +        │  |
                   │  tag side-table +   │  |
                   │  per-channel credit │  |
                   └─────────┬───────────┘  |
                    flat CiiHostIO Bundle    |
                    (iss_*, req_*, dat_*, wb_*|
                     + *_credit; plain UInts) |
                   ┌─────────▼───────────┐   |
                   │   BlackBox TTCii     │───┼─► tt_cii_host_wrap.sv:
                   └─────────────────────┘   |     • repack flat ⇄ cii_caracal_* structs (pkg)
                                             |     • tt_cii_interface cii_h(), cii_c()
                                             |     • tt_cii u_relay(.cii_host(cii_h),
                                             |                       .cii_coproc(cii_c))  // credit FIFOs
                                             |     • tt_vpu_cii_wrapper_top u_vpu(.cii_intf(cii_c))
```
Everything SV (the `tt_cii` credit relay + the VPU) lives **inside** the BlackBox; Chisel
sees only the flat host-side wires. The wrapper packs the flat `iss_*` inputs into
`cii_h.iss_data[0]` (= `cii_caracal_issue_req_t`) and unpacks `req_data`/`wb_data` back to
flat outputs. The 8→4-bit `tag` threads end-to-end: it tags every `req`/`wb` beat, and
`VecCiiHost` keys its side-table on it (operand-pull → PRN, result placement, `clr_rob` on
`last`).

**Flat `CiiHostIO` Bundle** (Chisel ↔ BlackBox; one plain-`UInt` field per struct member):
`iss_valid`/`iss_tag`/`iss_insn`/`iss_vtype`/`iss_vl`/`iss_vstart`/`iss_vxrm`/`iss_frm`/
`iss_hint` (out) + `iss_credit` (in); `req_valid`/`req_tag`/`req_op_id`/`req_op_offset` (in) +
`req_credit` (out); `dat_valid`/`dat_data` (out) + `dat_credit` (in); `wb_valid`/`wb_tag`/
`wb_data`/`wb_dst_offset`/`wb_wr_en`/`wb_status` (in) + `wb_credit` (out); plus `clk`/`reset`.

**Credit ↔ fire-and-forget caveat.** `iss_uops(0)` is a `Valid` (no `ready`) — once granted
the uop is gone. `VecCiiHost` must advertise `fu_types(0)` only when an issue credit is truly
available: it mirrors the SV issue-FIFO occupancy in a **local credit counter** (`+1` per
`iss_credit` pulse, `−1` per `iss_valid` fire) and gates `fu_types` on it. `fu_types` is
**registered** (a combinational `fu_types→grant→iss_valid→counter` path would loop), so the
counter accounting must be exact — never advertise with zero credits (same failure class as
M1's fixed "dropped vec-store grant" bug).

**Verification (9g).** Adapter elaborates and ties off cleanly; (e1)/(e2)/(e3) unaffected;
(9f) bit-identical with `enableVectorArith=false`.

#### Step B1 — Issue path (`IQ_V_ALU` → Issue channel)

**Scope.** Remove the M1 assert; advertise `valu_iss_unit.io.fu_types(0)` when CII
issue-credit > 0 (registered, no comb loop). On grant: allocate a 4b `tag` (`ciiTagBits`, 16 in
flight), emit the
**extended Issue packet** `{tag, instr = uop.debug_inst[31:0], vtype (from uop.vconfig), vl
(from pvl read), vxrm (from csr.io.vector), src_reuse_hint = 0}` (decision #3 — no separate
VCONFIG packet), and write a **tag side-table** entry: `{rob_idx, pvdest_grp[8],
pvdest_grp_mask, pvs1/2/3_grp, pvm, dst_rtype, is_shared, scalar_operands}`. (Completion is
now driven by the writeback `last` bit — decision #2 — so no `expected_wb` counter is
needed; keep `pvdest_grp`/mask for placement.)

**Files modified.** `vec/issue/VecIssueUnit.scala` (advertise `fu_types`), `exu/core.scala`
(remove assert; drive the extended Issue packet from `valu_iss_unit.io.iss_uops(0)`; VL-RF
read for `pvl→vl`; `csr.io.vector` read for `vxrm`).

**Verification (9e4 + 9g).** Trace shows a `vadd.vv` granting past-PNR, its tag, and the
Issue beat carrying the correct vtype/vl.

#### Step B2 — Operand pull (Src-Request → VRF read → Src-Data)

**Scope.** Service Src-Request: decode `{tag, op_id, op_offset}`. `op_id` →
{`NONE`=0, `VS1`=pvs1_grp, `VS2`=pvs2_grp, `VS3`=pvs3_grp, `VM`=pvm, `SCALAR`,
`STALE_VD`=stale_pvdest_grp} (canonical `cii_caracal_srcid_e` names — earlier
`SRC1/SRC2/SRC3/MASK` spellings are retired).

> **Normative (resolved).** `VS3` and `STALE_VD` are **two distinct slots naming two distinct
> groups**, and the host performs **no** instruction-dependent reinterpretation — it serves whichever
> slot is requested straight from the per-tag side-table. `pvs3` is an explicitly encoded third
> source; `stale_pvdest` is the group that held the destination arch vreg before this `OP.v` renamed
> it. **The coprocessor decides what to pull.** For RMW arithmetic (`vfmacc.vv vd, vs1, vs2` →
> `vd += vs1*vs2`) the two name the *same* group and the VPU pulls one; when they differ (masked
> `vadd.vv` under `vma=0`, `vslideup` prefix, `vcompress` tail) the VPU pulls **both**, spending an
> additional lane. An earlier draft had one `VS3` slot that the *host* resolved to `stale_pvdest`
> "when the instruction encodes no third source" — that put instruction decoding in the host adapter
> and made the two-different-groups case **unrepresentable**. Adding slot 6 is a one-value addition
> to `cii_caracal_srcid_e` (the wire type is already `logic [2:0]`, so 6 and 7 were free; no payload
> width changes), but it does require the VPU decoder to emit it — a small **SV-side** delta, so this
> step is not strictly host-only.

`op_offset` = member index. Read the VRF on **read ports `R5`–`R8`** — four lanes, because
`CII_NUM_SRC_REQ = 4` and the VPU wrapper returns member `k` of `{VS1,VS2,VS3,VM}` in one dat beat,
so all four can be live in the same cycle (registered, 1-cycle) — or serve the captured scalar from
the side-table; push 256b on Src-Data **in the exact order requests arrived** (a small in-flight
ordering FIFO, since VRF reads are registered). Reserve the `NONE` op_id encoding.

**Files modified.** `vec/cii/VecCiiHost.scala`; `exu/core.scala` (VRF read ports `R5`–`R8`).

**Verification (9e4 + 9g).** Trace shows each Src-Request → VRF member read → Src-Data,
ordering preserved; masked op pulls `pvm`.

#### Step B3 — Writeback (WB → VRF write + completion)

**Scope.** Consume Writeback. Two completion paths, routed by the side-table's `dst_rtype`
(decision: scalar-dest ops in scope):

- **Vector-dest ops** (`RT_VEC`) — map `{tag, wb_dst_offset}` → `pvdest_grp(wb_dst_offset)`;
  write the beat **verbatim** to the VRF on **write ports `W2`–`W3`** (full-mask; the VPU already
  applied vta/vma per decision #5). On the beat with the **`last` bit set** (decision #2),
  emit `group_done{prn = pvdest_grp, mask = pvdest_grp_mask}` + `clr_rob{rob_idx}`.
- **Scalar-dest ops** (`vmv.x.s`, `vfmv.f.s`, `vcpop.m`, `vfirst.m` → `RT_FIX`/`RT_FLT`) —
  route `wb_data[XLEN-1:0]` to the **INT/FP register file** write + a **scalar wakeup**
  (not the VRF / vec group_done); complete the ROB entry on the `last` beat. Uses the
  existing scalar writeback/wakeup ports (one dedicated `usingRVV`-gated wb port).

Accrue `wb_status` FP flags toward `fflags`; free the `tag` on `last`.

**Files modified.** `vec/cii/VecCiiHost.scala`; `exu/core.scala` (VRF write port `W2` +
INT/FP-RF scalar wb port; group_done/clr_rob to the new wakeup/clr ports); `exu/rob.scala`
(accept CII clr_bsy).

**Verification (9e4 + 9g).** `vadd.vv` at LMUL=2 shows two WB beats, `last` on the second,
then one group-done + clr_rob; a `vmv.x.s` writes the INT RF + wakes a scalar dependent;
results match Whisper; fflags accrue on an FP op.

#### Step B4 — VRF/VL-RF port allocation + wakeup/clr port counts

**Scope.** Concretely allocate CII ports (VRF reads 5,6 + write 1; VL-RF read port for
`pvl`), and bump the completion fan-in so the LSU and CII complete independently the same
cycle: `numVecWbPorts`/`numVecWakeupPorts` 1→2, add the second `vec_clr_bsy` +
`vec_wakeup` port, wire CII `group_done`/`clr_rob` alongside `VecLSU`.

**Files modified.** `common/parameters.scala` (`enableVectorArith`, port/wakeup counts,
`require`s), `common/config-mixins.scala`, `exu/core.scala`, `exu/rob.scala`.

**Verification (9g).** LSU + CII completing the same cycle both clear correctly; (e2)/(e3)
LS regression stays green.

#### Step B5 — Flush, exceptions, `vxrm`/fflags, segment-LS transpose half

**Scope.** Past-PNR issue ⇒ no **branch**-kill of in-flight CII ops (`is_br`/`is_jalr` set
`starts_unsafe`, so the PNR never sweeps past an unresolved branch; the SV has no kill line, which
is consistent). It does **not** remove flush recovery: past-PNR is *not* a commit guarantee — a
ROB-head flush (`MINI_EXCEPTION_MEM_ORDERING`, CSR replay, exception, ERET) squashes accepted CII
work, per `rob.scala:438-441` (*"the failing load will have been marked safe already"*). The
contract below is now normative in `cii.rst` §Flush recovery; **all live tags are killed** with no
age compare, because `io.flush.bits` carries no `rob_idx` and both flush sources fire at the ROB
head, which no in-flight tag can be older than. The **coprocessor never flushes** (Track C C5): on a flush it keeps
running accepted ops to completion. So flush handling lives entirely on the **host** — the
adapter tracks which in-flight tags are killed (`RegNext(rob.io.flush.valid)` marks them),
then **drops/ignores their writebacks** (and satisfies any src-data still owed with
don't-care beats) while continuing to sink every channel beat so CII credits are returned
and neither side stalls. A killed tag is freed from the side-table on its (dropped) `last`

> **Normative (resolved).** Two points that earlier drafts got wrong and this step gets right:
> the src-data beat owed to a killed tag is **mandatory**, not optional — the VPU is in-order with
> no kill line, so withholding it makes the VPU wait forever, never emit `last`, never free its tag,
> and wedge every *surviving* instruction behind it. And the suppression on a killed writeback must
> cover **all four** effects: the VRF/INT/FP write, `clr_rob`, `VecGroupDone`, and `fflags`/`vxsat`
> accrual.
>
> **Do not reuse this drain-and-discard pattern for the vector LSU.** CII *tags* are not recycled
> during the drain, but vector destination *PRNs* are returned to the free list on the flush and
> reallocated within a few cycles — a late LCB write would corrupt a live, unrelated instruction.
> The LSU path needs real squashing (pointer rollback), see `loadstore.rst` §Squashing the in-flight
> vector LSU.
beat. `vxrm`/`vxsat` are read from `csr.io.vector`
at issue and travel in the **extended Issue packet** (decision #3); `wb_status` fflags
accrue at commit. For `is_shared` (segment LS), the CII drives the **second** group-done
that clears `rob_other_half_pending` → unblocks full segment LS (M1 deviation #9).
Vector-arith precise `vstart` resume, if the CII faults, is wired here.

**Files modified.** `vec/cii/VecCiiHost.scala`, `exu/rob.scala`, `exu/core.scala`.

**Verification (9e4 + 9g).** A segment LS completes on both halves; a flush mid-arith
leaves no orphaned tags; `vxrm`-rounded fixed-point op matches Whisper.

#### Step B6 — Vector-arith regression (e4)

**Scope.** New `tests_regr/vec_arith_tests.txt`: integer/FP `.vv`/`.vx`/`.vi`, reductions,
mask-logic, permutes/gathers, ± mask, LMUL ∈ {1,2,4,8}, SEW ∈ {8,16,32,64}, under
VCS+Whisper cosim. Scalar (9d) and LS (e2/e3) stay green; `enableVectorArith=false`
bit-identical.

**As-built — first integrated elaboration (C6/B6 in progress).** Added
`MediumBoomV4VectorArithConfig` (chipyard `BoomConfigs.scala`) =
`WithVector(VectorParams(enableVectorArith = true))`. `make CONFIG=… verilog` **elaborates
cleanly** — the first time the whole Track B host wiring (the `usingVectorArith` block in
`core.scala`, VRF read 5/6 + write 1, VL-RF read 1, the `numVecWbPorts`/`numVecWakeupPorts`=2
completion fan-in, `VecCiiHost` + the `TTCii` BlackBox) elaborates. `VecCiiHost.sv` and the
`tt_cii_host_wrap.sv` bridge (via `addResource`) are emitted into gen-collateral. Bug fixed:
`VecRenameStage` was instantiated with `numWbPorts=1` hard-coded, so `io.wakeups(1)` (the CII
group-done port) was out of bounds — now `numVecWakeupPorts` (1 under M1 → bit-identical, 2
under arith). **Open gap for the VCS *compile*:** only `tt_cii_host_wrap.sv` is on the sim
file list; the modules it instantiates (`tt_cii` relay, `tt_cii_interface`, `tt_cii_channel`,
`tt_cii_fifo`, `rv_async_rst_dff`, the `tt_cii_caracal_pkg` svh) — and, for full C6, the whole
VPU tree — are not yet pulled into the Chipyard build. Wiring those in (and swapping the B0
null coprocessor in the bridge for `tt_vpu_cii_wrapper_top`) is the remaining C6 work.

**As-built — full VCS compile PASSES.** The TT-CII relay stack is now pulled into the build:
`VecCiiHost` `addResource`s the five relay modules (`rv_async_rst_dff`, `tt_cii_fifo`,
`tt_cii_channel`, `tt_cii_interface`, `tt_cii`) + `tt_cii_caracal_pkg.svh`, via symlinks under
`resources/vsrc/` → the vendored `sv/v4/tt-cii/src` (no dup; a fresh checkout must populate the
nested tt-cii repo). `make CONFIG=MediumBoomV4VectorArithConfig` builds a full `simv` cleanly —
VCS binds the `TTCii` BlackBox to `tt_cii_host_wrap.sv` with matching port widths (**confirms
the `NUM_DST_WB=2` alignment**) and resolves the whole relay stack. This simv is host + bridge
+ relay + **B0 null coprocessor**, so it validates the host-side CII path structurally; a
vector-*arith* instruction would issue but never retire (the null cop drops it), so functional
cosim awaits swapping in `tt_vpu_cii_wrapper_top` (+ its VPU SV tree on the filelist) — the
final C6 step.

**As-built — REAL VPU integrated, full VCS compile PASSES (C6 build gate).** The B0 null
coprocessor is replaced by `tt_vpu_cii_wrapper_top #(.VLEN(CII_VLEN)) u_vpu(.cii_intf(iface_B),
…)` on the relay's coprocessor side. The whole VPU SV tree is pulled into the build via
`VecCiiHost` `addResource`s (symlinks under `resources/vsrc` → `sv/v4/{vpu,common}` +
`resources/HardFloat`): 51 compiled `.v`/`.sv` (common util/arith, HardFloat, decoder,
execution units, regfile, `tt_vec_top`, the CII wrapper) + 6 include-only `.svh`/`.h`/`.vi`.
Two enablers: (a) HardFloat `.v` self-include their `.vi` macros via the gen-collateral incdir,
but `.vi` had to be added to the compile-filelist exclusion in `common.mk` (local patch:
`grep -v '…\|vi)$'`) so VCS doesn't compile the macro headers; (b) no HardFloat name collision
(raw lowercase `mulAddRecFN` vs rocketchip's Chisel-mangled `MulAddRecFNPipe_l2_e11_s53`).
`make CONFIG=MediumBoomV4VectorArithConfig` builds a full `simv` with **0 errors** (only benign
interface inout-no-load warnings); the VPU + relay elaborate fully (no unresolved/blackbox
modules). The vector-arith datapath is now structurally complete host↔VPU. Remaining:
**functional cosim** — a vector-arith test on the `-debug` simv under Whisper (e4 / B6), the
first true end-to-end result.

**As-built — FIRST FUNCTIONAL COSIM (C6/B6 bringup).** Built the `-debug` simv and ran under
Whisper (`USE_IMAGE_WHISPER=1 make … run-binary-debug-hex`). `ms11a2_pure_vle` (vector load)
PASSES → the arith build does not regress the general/LS path. `ms12_vadd_expecthang`
(`vle64 v4,v8` + `vadd.vv v12,v4,v8`, the M1 "expected-hang" arith test) now **runs
end-to-end — no hang** (issue → operand-pull → VPU compute → writeback → ROB-retire all work,
wb lands in the correct v12). Two bugs found and one fixed:
  1. **`vstart` (FIXED):** the vector `vstart` CSR is not architecturally maintained, so
     `VecCiiHost` forwarded a stale `vstart=32`; `vstart>vl` made the VPU treat every element
     as pre-start (keep old dest) → result = old dest. Fix: `VecCiiHost` forces `g_vstart=0`
     (M2 issues past-PNR, no mid-instruction fault resume). Result now computes `p0+p1`.

     > **Root cause and proper fix (resolved).** The stale value was a symptom, not the bug: the
     > `vstart` CSR genuinely was not architecturally maintained. Forcing `vstart=0` in the packet
     > hides it but is **not** correct — software may set `vstart` with `csrw vstart`, and RVV
     > requires elements below it to be left unmodified (observable whenever `vd` overlaps a source).
     > The resolution is: (a) delegate `vstart` to rocket's `usingVector` `CSRFile`, which maintains
     > it architecturally and clears it on every non-trapping vector instruction; (b) forward the
     > **real** value in the issue packet and require the VPU to honour it; (c) note that *hardware*
     > never produces a non-zero `vstart`, because vector LS faults now trap with `vstart = 0` and
     > restart the whole instruction. See `frontend.rst` §Vector architectural state and
     > `loadstore.rst` §Per-Element Progress Tracking.
  2. **source-readiness interlock (FIXED).** A `vadd` reading a just-loaded vreg via the CII
     read the VRF before the vector load's write landed → garbage source (passed only with a
     NOP barrier). Root cause: `VecBusyTable`'s source read used only the *registered*
     `busy_table` (+ same-cycle clear) and MISSED the same-cycle *set* from an older lane in
     the same rename packet — so when `vle v8` (older) and the `vadd` (younger) renamed
     together, the vadd read `v8` not-busy and issued without the dependency. The vec MAP
     table already bypasses older-lane allocations (probe: vadd `map(v8)` = `vle v8` pdst=33),
     but the busy table did not. Fix: OR the older lanes' set masks into the lane-i source
     read (`VecBusyTable.groupBusy`), mirroring the map-table bypass. This also explains the
     `v4`-correct/`v8`-garbage asymmetry (`v4` in an earlier packet → busy registered;
     `v8` same packet → missed). **`ms12_vadd_expecthang` (no barrier) now `*** PASSED ***`
     — the first end-to-end host↔VPU vector-arith cosim with no workaround**; the NOP variant
     and `ms11a2_pure_vle` still pass.

---

## Verification and sign-off

1. **Scalar baseline (vector off)** — `MediumBoomV4Config`, full csmith + Linux boot;
   byte-for-byte vs pre-Caracal (and vs M1).
2. **Scalar perf (vector on)** — `./run_regr_rvv_scalar.sh MediumBoomV4VectorConfig` all
   PASS; cycle delta reviewed (the arbiter must not regress scalar D$/TLB latency).
3. **vset + load/store regression** — `tests_regr/vset_loadstore_tests.txt` all PASS
   (unchanged from M1).
4. **LS disambiguation / ordering stress (e3)** — `tests_regr/vec_mem_order_tests.txt` all
   PASS + 0 MISMATCH: vector-store→younger-scalar-load, scalar-store→younger-vector-load,
   SSI store→younger SSI load (serialized), partial-overlap forward, page-crossing beat.
5. **Vector arith cosim (e4, TBD)** — once the CII lands: `rv64uv-p-*` arithmetic subset
   at LMUL ∈ {1,2,4,8}, SEW ∈ {8,16,32,64}, ±mask, under Whisper cosim.
6. **Performance counters** — go live now that vector LS uses the TLB/arbiter: TLB-pressure
   split (`tlb_miss_scalar`/`tlb_miss_vec`/`tlb_port_stall_*`), MSHR-policy counters, the
   `lsuWidth × ELEN` bandwidth-ceiling monitors, and (with the CII) vec-arith retire/occupancy.
7. **Docs** — keep `loadstore.rst` §mem-order in sync with the as-built snoop; add
   `cii.rst` when the CII spec is written and back-fill Track B.

**Acceptance for Milestone 2 (Track A — the committed deliverable):**
- All (e3) disambiguation/ordering tests PASS + 0 MISMATCH; the M1 memory-ordering hole is
  closed (a younger scalar load never bypasses an older aliasing vector store).
- US range-overlap and SSI per-element cross-LSU snooping both functional; LD→ST forwarding
  from the vector store data queues functional; `vleff`/`vstart` precise on real faults.
- vset + load/store regression and scalar perf all PASS; `usingRVV=false` RTL diff empty;
  bit-identical with `vecScalarSnoopEnable=false`.

**Acceptance for Milestone 2 (Track B — pending CII spec):**
- Deferred until the CII spec exists; acceptance criteria written then. Track B is not a
  blocker for the Track-A LSU deliverable.

---

## Non-goals for Milestone 2 (deferred to M3+)

- Anything requiring the CII spec that isn't captured by the placeholder contract above.
- VRF banking / port tuning beyond the M1 9R/3W (unless the CII forces it).
- Multi-hart / coherence stress beyond the single-hart RVWMO bar.
- A memory-compare Whisper cosim hook (recommended follow-on; the (e3) load-back pattern
  is the M2 workaround for the M1 store-data cosim gap).

---

## Cross-cutting risks and mitigations

| Risk | Mitigation |
|------|-----------|
| Vector addresses invisible to scalar LCAM (the M1 hole) | Track A registers US ranges / SSI elements into the LCAM; `order_fail` replay is the correctness floor (A2/A3) |
| US range-overlap vs a per-dword scalar CAM | Add a range-overlap predicate beside the dword match; reuse `GenByteMask`/age helpers, don't fork the CAM |
| SSI per-element search starving scalar disambiguation | `DcacheArbiter` scalar-floor + anti-starvation; SSI elements contend through the same arbiter |
| SSI store→younger SSI load forwarding hazard | Serialize (hold the load until the store resolves); no partial-scatter forward (A3) |
| Bare→TLB transition breaks M1 bit-identical | All Track A behind `vecScalarSnoopEnable`; A0 TLB routing gated too; (9f) enforced per step |
| Page-crossing unit-stride beat | Split at the page boundary in the Walker/Packer (A0) |
| Whisper doesn't compare vector-store data | (e3) tests are store-then-scalar-load-back so the load-back catches ordering/data errors (rule 11) |
| CII spec not ready | Track B is an explicit placeholder; Track A is independent and is the committed M2 deliverable |
| `IQ_V_ALU` accidentally issuing before CII attach | M1 hard assertion stays until `enableVectorArith` + CII wiring land together (Track B) |
| Segment LS still blocked | Unblocks only with the CII transpose half (B4); documented as CII-dependent |

---

## File-touch summary

**Created (all under `src/main/scala/v4/vec/`):**

| Step | Path |
|------|------|
| A1 | `lsu/CrossLsuSnoop.scala`, `lsu/DcacheArbiter.scala`, US/SSI address+data queues (`st_US/SSI_ADDR_Q`, `st_US/SSI_DATA_Q`, `ld_US/SSI_ADDR_Q`) |
| B0+ (TBD) | `cii/*` — CII coprocessor attach + arith EUs (pending CII spec) |

**Modified (baseline + M1 files — kept minimal):**

| File | Steps | Reason |
|------|-------|--------|
| `v4/lsu/lsu.scala` | A0–A4 | vec beat TLB use; feed vec addrs into `lcam_addr`/`lcam_mask`; accept vec-driven `ldq_order_fail`; route forwarded data |
| `v4/vec/lsu/VecLSU.scala` | A0,A3,A5,A6 | beat translate state; SSI-load hold; `vleff` fault latch; wide port |
| `v4/vec/lsu/Vec{Load,Store}Walker.scala`, `*Packer.scala` | A0 | page-crossing split; no identity-map assumption |
| `v4/exu/core.scala` | A1,A6,B0+ | instantiate queues/snoop/arbiter; wire TLB/arbiter; CII attach (TBD) |
| `v4/exu/rob.scala` | A5,B4 | `vstart` on element fault; both-halves group-done for segment LS (CII) |
| `v4/vec/rename/VlRename.scala` | A5 | re-arm dormant `is_vleff` VL-producer path |
| `v4/lsu/dcache.scala` | A6 | vector cache port width |
| `v4/common/parameters.scala`, `config-mixins.scala` | A1,B0 | activate `vecScalarSnoopEnable`/`dcacheArbiterMode`; CII params (TBD) |

---

## Implementation notes & deviations (Milestone 2 as-built)

*(To be filled during implementation, as the M1 plan's as-built section was — record
bugs found and fixed, design deviations, and known gaps here per step.)*

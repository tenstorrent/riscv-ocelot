# Caracal Goal 1 Implementation Plan

**Objective.** Add RVV 1.0 instruction *support* to BOOM v4 — decode, vector uop cracking, vector rename, ROB allocation, vector issue queues, vector register file, and vector load/store unit — all integrated into the BOOM out-of-order core itself (not as an OVI side-car as in `bobtail/main`). The vector ALU datapath is **deferred to Goal 2**: V-ALU issue ports are tied off; arithmetic uops are decoded, cracked, renamed, and queued, but never execute. Vector loads and stores **must** execute end-to-end.

**Reference.** Architecture diagram at `./caraval.png` (XML at `./caraval.drawio.xml`). v4 baseline documented in `docs/boom-v4-architecture.md`. Prior OVI-based vector integration on `bobtail/main` is reference material for SystemVerilog primitives to port (`src/main/resources/vsrc/vpu/`) and for one already-Chisel module (`src/main/scala/exu/ovi_wrapper/ls_decode.scala`).

**Branch strategy.** Each step below is a `Caracal/addvector/<step-slug>` feature branch that merges into `Caracal/addvector` and ultimately into `Caracal/main`. Steps are ordered so that every intermediate state compiles, elaborates, and (when `enableVector=false`) is bit-identical to baseline.

---

## Global ground rules

1. **Feature flag.** All vector logic gated by `enableVector: Boolean = false` (new `BoomCoreParams` field). Default off — every existing config and test must remain bit-identical to pre-Caracal output.
2. **Code location.** All new Chisel under `src/main/scala/v4/vec/`. Subdirs: `decode/`, `rename/`, `issue/`, `regfile/`, `lsu/`, `common/`. Package `boom.v4.vec.{decode,rename,issue,regfile,lsu,common}`.
3. **Minimize intrusive edits.** Whenever logic *can* live in `v4/vec/` rather than in a baseline file, put it there — even if it duplicates a small amount of BOOM code. Touch baseline files only where unavoidable (`MicroOp`, `BoomCoreParams`, `config-mixins.scala`, `core.scala`, `decode.scala`, `rob.scala`, and the `IssueSlot`/`IssueUnit` bundles).
4. **Parameters.** Defaults: `VLEN = 256`, `VLMAX = 256` bits, `numVecPhysRegisters = 128`. Everything else parameterized (`numVecLoadQueueEntries`, `numVecStoreQueueEntries`, issue-queue widths/entries, `numDecodeToCrackerBufferEntries`, etc.). The default tier under test is `WithNSmallBooms ++ WithVector`, but the implementation must elaborate cleanly at Medium and Large widths too.
5. **In-order within queue, OoO across queues.** Each new `IQ_V_*` is age-ordered (collapsing) just like existing queues. Inter-queue ordering is enforced exclusively by the ROB.
6. **Precise exceptions.** Vector loads/stores must update `vstart` on element-level exceptions so the trap handler can resume mid-vector. Stores do not retire to the DCache until ROB-committed.
7. **Don't fight the existing wakeup network.** Whenever possible, treat new operands (VL, V0, vector pregs) as additional source operands on the issue slot using the *existing* preg-match wakeup network. Avoid building a parallel broadcast bus unless absolutely necessary (see Step 9).
8. **Verification gate per step.** Each step is "done" only when (a) `sbt compile` passes; (b) `make checkstyle` passes; (c) `enableVector=false` elaboration is unchanged (RTL diff is empty for the baseline configs); (d) the step's listed verification artifacts pass.

---

## Step 0 — Project scaffolding and feature flag

**Scope.** Create the empty directory structure, the feature flag, and the config mixin shell so subsequent steps have a home. No functional behavior.

**Files created.**
- `src/main/scala/v4/vec/common/VectorParams.scala` — `case class VectorParams(vLen: Int = 256, numVecPhysRegisters: Int = 128, numVecLoadQueueEntries: Int = 32, numVecStoreQueueEntries: Int = 32, numDecodeToCrackerBufferEntries: Int = 4, vecIssueParams: Seq[IssueParams] = …)`.
- `src/main/scala/v4/vec/common/package.scala` — package object declaring `boom.v4.vec.common`.
- `src/main/scala/v4/vec/{decode,rename,issue,regfile,lsu}/.keep` — placeholders.

**Files modified.**
- `src/main/scala/v4/common/parameters.scala` — add `enableVector: Boolean = false` and `vector: Option[VectorParams] = None` to `BoomCoreParams`. Add derived `usingVector` to `HasBoomCoreParameters`. **Do not** alter any `require()` yet.
- `src/main/scala/v4/common/config-mixins.scala` — add `class WithVector extends Config(...)` that sets `enableVector = true` and `vector = Some(VectorParams())`. Add `WithNSmallBoomsVector` convenience composition (`new WithNSmallBooms ++ new WithVector`).

**Verification.**
- `sbt compile` clean.
- Elaborate `WithNSmallBoomsVector` and `WithNSmallBooms` separately; both produce RTL (Verilog diff against the v4 baseline for `WithNSmallBooms` is empty).
- `make checkstyle` passes.

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
  - Add cracker bookkeeping: `v_split_first`, `v_split_last`, `v_split_idx (UInt(log2Ceil(8).W))`, `v_split_total (UInt(log2Ceil(8).W))`.
  - Add segment LS fields: `v_seg_nf (UInt(3.W))`, `v_seg_idx`.
  - Add `vl_is_known: Bool` — true when VL is determined at decode (`vsetivli` or const after broadcast); false when waiting on a renamed scalar.
- `src/main/scala/v4/common/consts.scala` — `RT_VEC = 3.U(3.W)`; widen the `dst_rtype` Mux constants.

**Verification.**
- `sbt compile` clean.
- Existing v4 unit elaboration unchanged when `enableVector=false` — confirm with `diff` against pre-Step-1 generated Verilog.
- New Bundle width is what we expect (one-shot Chisel printout test under a tiny test wrapper).

**Risks.** Widening `*_rtype` from 2 to 3 bits is a ripple change. Audit every `*_rtype === RT_…` and `*_rtype =/= RT_X` site in `decode.scala`, `rename-stage.scala`, `dispatch.scala`, `rob.scala`, the issue-units, and `core.scala`. Most will be untouched by widening, but a few `UInt(2.W)` casts will need to become `UInt(3.W)`.

---

## Step 2 — RVV 1.0 decode tables

**Scope.** Recognize RVV 1.0 instructions in `decode.scala`. Populate the new `MicroOp` vector fields. Handle `vsetvli`/`vsetivli`/`vsetvl` semantics. No cracker yet — for now decode emits a single uop per instruction.

**Files modified.**
- `src/main/scala/v4/exu/decode.scala` — gate behind `usingVector`: add new tables.
- New file `src/main/scala/v4/vec/decode/VDecode.scala` — vector decode table: vector arithmetic ops (set `iq_type = IQ_V_ALU`, `is_vec = true`), vector loads (`IQ_V_LOAD`), vector stores (`IQ_V_STORE`).
- New file `src/main/scala/v4/vec/decode/VLSDecode.scala` — vector load/store table with unit-stride, strided, indexed (ordered + unordered), segment, fault-only-first variants. Encodes `mop`, `lumop`, `nf`, `width` into `v_eew`/`v_emul`/`v_seg_nf`.
- New file `src/main/scala/v4/vec/decode/VsetDecode.scala` — three uop forms:
  - `vsetivli` → all immediate; set `vl_is_known = true`, fill `VConfig` directly.
  - `vsetvli` → `rs1` is scalar VL source (renamed by sRRU). Decode `vtype` from immediate. Mark `is_vsetvli`, set `vl_is_known = false`.
  - `vsetvl` → both rs1 and rs2 are scalars. **Decoder stalls** for this opcode until the older in-flight scalar producers of rs1/rs2 complete and broadcast their results back to decode. Implement the stall as a `dec_hazard` condition keyed on `is_vsetvl && ((io.rs1_busy && in_flight_count > 0) || (io.rs2_busy && in_flight_count > 0))`. Once unblocked, fill `VConfig` directly.

**Configuration tracking at decode.** Maintain a "current architectural vtype/vl" register at decode that the cracker (Step 3) consults to know LMUL/EMUL. For `vsetivli` and `vsetvl`-after-stall, this register updates immediately at decode. For `vsetvli` (VL not known yet), the cracker uses a *predicted* VL — but per the user's design, vsetvli's VL comes from the renamed rs1, and downstream vector uops carry `pvl_busy=true` until the VL broadcast clears them. Decode is allowed to proceed past a `vsetvli` with VL marked unknown; the cracker uses `vlmax` as an upper bound for the cracker's outermost LMUL loop (cracking is conservative — produce uops as if LMUL=full; the issue-queue side will only execute up to actual VL elements).

**Verification.**
- Unit test in `src/test` (re-introduce a minimal test directory under `src/test/scala/v4/vec/` if needed): drive instruction bit-patterns at the decode input; check that `is_vec`, `iq_type`, `v_eew`, `v_emul`, `vl_is_known`, etc. match a golden Scala model.
- Test cases: `vsetivli x0, 4, e32, m2`, `vsetvli a0, a1, e16, m4`, `vsetvl a0, a1, a2`, `vle32.v v0, (a0)`, `vse64.v v8, (a0)`, `vlseg3e8.v v0, (a0)`, `vlsseg2e16.v v0, (a0), a1`, `vluxei8.v v0, (a0), v8`, `vadd.vv v0, v1, v2` (just to confirm `iq_type=IQ_V_ALU`).

**Risks.** RVV decode is one of the larger opcode spaces in RISC-V; getting `v_eew`/`v_emul` right for widening/narrowing arithmetic (e.g., `vwadd.vv` has EEW=2×SEW for the destination) is the part where bugs hide. Cover widening cases explicitly in the unit test.

---

## Step 3 — Vector UOP cracker

**Scope.** A per-decode-lane cracker that expands one input vector uop into `LMUL` (or `EMUL` for widening) sequential uops. One cracker per decode lane.

**Files created.**
- `src/main/scala/v4/vec/decode/VecUopCracker.scala`.

**Behavior.**
- One cracker module per decode lane. Input: `Decoupled[MicroOp]`. Output: `Decoupled[MicroOp]`.
- FSM with states `s_idle`, `s_cracking`.
- On entry of a vector uop with `v_emul_count > 1`: cracker enters `s_cracking`, emits N uops on successive cycles. Each emitted uop has:
  - `lvs1`, `lvs2`, `lvs3`, `lvd` incremented by the iteration index (e.g., `lvd_new = lvd + i`).
  - `v_split_idx = i`, `v_split_total = N`, `v_split_first = (i == 0)`, `v_split_last = (i == N-1)`.
  - All other fields copied from the source uop, including `rob_idx` — **all sub-uops of one architectural instruction share the same `rob_idx`** (the ROB allocates one entry per architectural instruction, not per cracked uop). The ROB busy bit is cleared only when the last sub-uop writes back.
- **Decode stall.** When cracker is in `s_cracking`, it deasserts `io.in.ready` so the decoder/buffer pair holds the source uop's "next" uop until cracker finishes the current expansion.
- Widening (`v_emul > v_lmul`): cracker emits `v_emul` uops for the destination. Sources may be replicated (the source EMUL differs from destination EMUL). This is captured by per-operand emul-derived stride values in the uop.
- Segmented LS (`v_seg_nf > 1`): cracker emits `nf × emul` uops for segment loads/stores. The user said cracker cracks based on LMUL/EMUL; for segments this generalizes to `nf × EMUL`. Confirm exact spec semantics in the test.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — instantiate `Seq.fill(coreWidth)(Module(new VecUopCracker))` only when `usingVector`. Wire between the decode→cracker buffer (Step 4) and rename.

**Verification.**
- Unit test: feed a single `vle32.v v4, (a0)` at LMUL=2, expect 2 output uops with `lvd = 4, 5`, `v_split_idx = 0, 1`, both with the same `rob_idx`.
- LMUL=8 → 8 outputs over 8 cycles.
- Widening `vwadd.vv v4, v2, v1` at LMUL=2 (EMUL_dest=4) → 4 output uops; sources stride differently from destinations.
- Segment `vlseg3e16.v v0, (a0)` at LMUL=1 → 3 output uops (one per field), all writing to fields of `v0`/`v1`/`v2`.
- Scalar instructions pass through unchanged in 1 cycle.

**Risks.** Branch-kill mid-crack: if the originating architectural instruction is mispredicted and killed while the cracker is in the middle of expansion, the cracker must drop all unsent sub-uops. Handle via the standard `br_mask` propagation — sub-uops carry the original instruction's `br_mask`.

---

## Step 4 — Decoder→Cracker buffer

**Scope.** A small per-lane FIFO between `DecodeUnit` and `VecUopCracker` to absorb cracker stalls.

**Files created.**
- `src/main/scala/v4/vec/decode/DecodeToCrackerBuffer.scala` — a queue of `MicroOp`, depth `numDecodeToCrackerBufferEntries` (default 4).

**Why a separate buffer rather than relying on the FetchBuffer.** The FetchBuffer holds *fetched instructions*, not decoded uops. We don't want the cracker's back-pressure to stall the decode lane — that wastes decode bandwidth for scalar instructions that don't need cracking.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — when `usingVector`, insert one buffer per decode lane between `decode_units(w).io.deq` and `cracker(w).io.in`. When `!usingVector`, the buffer is bypassed entirely.

**Verification.**
- Synthetic stress test: drive decoder with a sequence of 8 vector uops at LMUL=8 (each crack takes 8 cycles); confirm decoder can issue one per cycle until the buffer fills, then back-pressures; no uops lost; commit order matches issue order.
- Branch-kill drains the buffer.

---

## Step 5 — Vector Rename Unit (vRRU)

**Scope.** A second rename stage that runs *after* the scalar rename stage (sRRU). All scalar register renaming (int, FP, mask source if it's scalar) happens in sRRU. The vRRU then renames vector pregs (`vs1`/`vs2`/`vs3`/`vd`/`vm = V0`).

**Files created.**
- `src/main/scala/v4/vec/rename/VecRenameStage.scala` — module that wraps a `VecMapTable`, `VecFreeList`, `VecBusyTable`.
- `src/main/scala/v4/vec/rename/VecMapTable.scala` — 32 logical vector regs → physical pregs (size `numVecPhysRegisters`, default 128). Branch snapshots `[maxBrCount][32]`. The V0 mask register is logical reg 0 in this table — it gets the same maptable treatment as any other vector reg.
- `src/main/scala/v4/vec/rename/VecFreeList.scala` — bitvector of free vector pregs; per-branch allocation lists for branch-mispredict reclaim.
- `src/main/scala/v4/vec/rename/VecBusyTable.scala` — bitvector of in-flight vector pregs.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — instantiate `Module(new VecRenameStage)` only when `usingVector`. Pipeline order: cracker out → sRRU → (if `is_vec`) vRRU → dispatch. Scalar uops bypass vRRU entirely (single cycle pass-through).
- Wire vector wakeup ports (Step 11 vector LSU writeback + Step 12 tied-off V-ALU placeholder) into `VecBusyTable`.

**Mask register handling.** When a vector uop carries `lvm != 0` (no actual mask register exists outside V0 in RVV), `lvm` is always 0; vRRU looks up `map_table(0)` to get `pvm`. When `lvm = 0` is interpreted as "no mask" (the encoded `vm` bit in the instruction), set `pvm_busy = false` directly so it doesn't block issue.

**Snapshotting.** vRRU consumes the *same* `br_tag` that sRRU allocates. The vector maptable snaps on the same branch-tag allocation event as the int/FP maptables; restore on `brupdate.b2.mispredict` is parallel to the existing int/FP restore.

**Verification.**
- Unit test: allocate vector destinations across 10 uops; confirm freelist decrement and busytable set.
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
- `src/main/scala/v4/exu/core.scala` — wire `rob.io.dealloc_vec_pregs` into `vRRU.io.dealloc_pregs`. Wire vector-LSU writeback into `rob.io.wb_resps`.

**Verification.**
- Elaboration: `require()` checks on writeback port counts pass.
- Cracked-uop commit test: dispatch a vector load at LMUL=4; confirm ROB entry doesn't commit until all 4 sub-uop writebacks land.

---

## Step 7 — Vector issue queues

**Scope.** Three new issue queues: `IQ_V_LOAD`, `IQ_V_STORE`, `IQ_V_ALU`. Each is age-ordered (collapsing) like the existing queues. Default sizing (Small): each `1×32` per the diagram.

**Files created.**
- `src/main/scala/v4/vec/issue/VecIssueSlot.scala` — extension of `IssueSlot` with operand slots for `pvs1`/`pvs2`/`pvs3`/`pvm`/`pvl` plus their busy bits. (Alternatively: add these fields directly to the base `IssueSlot` and gate with `usingVector` — decide based on how much copy-paste duplication results. Recommend: extend the base slot with `Option`-wrapped vector operand slots so a single `IssueUnit` codepath handles both kinds.)
- `src/main/scala/v4/vec/issue/VecIssueUnit.scala` — thin wrapper around `IssueUnitCollapsing` parameterized for vector slot type. May not need to exist if the base IssueUnit is sufficient with the extended slot.

**Files modified.**
- `src/main/scala/v4/common/consts.scala` — add `IQ_V_LOAD`, `IQ_V_STORE`, `IQ_V_ALU` to the `iq_type` bitfield. Widen `iq_type` from 3 to 4 bits.
- `src/main/scala/v4/common/parameters.scala`:
  - Add the three new `require(issueParams.count(_.iqType == IQ_V_*) == (if usingVector) 1 else 0)` invariants.
  - Update `WithVector` mixin's `issueParams` to append `IssueParams(issueWidth=1, numEntries=32, iqType=IQ_V_LOAD, dispatchWidth=1)`, etc.
- `src/main/scala/v4/exu/dispatch.scala` — the existing bitwise `iq_type & issueParam.iqType` routing should "just work" once `iq_type` is widened. Verify with a simulation.
- `src/main/scala/v4/exu/core.scala` — instantiate the three new issue units; wire their wakeups into the integer wakeup network (so a vector preg becoming ready broadcasts back) plus the new vector wakeup network.

**In-order within queue.** Use the existing age-ordered collapsing queue — it's already in-order. Confirm by reading `issue-unit-age-ordered.scala`; no change needed.

**Verification.**
- Elaboration of `WithNSmallBoomsVector` with all four+three queues instantiated.
- Dispatch routing test: a sequence of `vle32.v / vse32.v / vadd.vv` goes to V_LOAD / V_STORE / V_ALU respectively.
- In-order test: two V_LOAD uops with different ROB indices arrive — they must be issued in dispatch order even if their operands wake in reverse order. (This is the standard age-ordered behavior; just confirm.)

---

## Step 8 — Vector L/S decoder (port of `ls_decode.scala`)

**Scope.** Bobtail's `src/main/scala/exu/ovi_wrapper/ls_decode.scala` is already Chisel. Port to v4 and rename to `VecLsDecode`. It receives uops issued from `IQ_V_LOAD`/`IQ_V_STORE` and finishes the LS-specific decoding (stride, segment, EEW disambiguation, address-generation parameters) since by the time the uop issues, all scalar operands (including stride, base, VL) are resolved.

**Files created.**
- `src/main/scala/v4/vec/lsu/VecLsDecode.scala` — direct port. Replace `boom.exu` package with `boom.v4.vec.lsu`; replace `boom.common._` with `boom.v4.common._`; adapt `EnhancedFuncUnitReq` (port the bundle as well, or substitute the v4 equivalent).
- `src/main/scala/v4/vec/lsu/ConfigInfo.scala` — port the `ConfigInfo` bundle from bobtail.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — instantiate one `VecLsDecode` between issue and the VLSU.

**Verification.**
- Unit test parity with bobtail: drive a handful of known LS uops through both bobtail's `OviLsDecode` and the new `VecLsDecode` (in isolation); confirm identical `ConfigInfo` outputs.

---

## Step 9 — VL broadcast (reuse existing wakeup network)

**Scope.** Make the issue queues hold vector uops until VL is resolved.

**Design choice.** The user offered two options: (a) build a `VLBroadcastManager` that listens on int writebacks, or (b) reuse the existing IQ slot register bookkeeping by treating VL as just another source operand. **Choose (b)** — it's strictly cheaper. The integer writeback path already broadcasts `(valid, pdst)` to every issue slot in every queue; we just need vector slots to have an extra source operand (`pvl`) participating in that match.

**Files modified.**
- `VecIssueSlot` (Step 7): the `pvl` operand and its `pvl_busy` bit are *part of* the slot's operand-ready mask. The slot's `request := all_ready` already gates issue; just include `pvl_ready` in the AND.
- `core.scala`: when adding wakeup ports to the V_LOAD / V_STORE / V_ALU queues, include the integer EU's writeback ports (since VL is an *integer* writeback — `vsetvli` writes back to the integer regfile).
- For `vsetivli` (constant VL): decode sets `pvl_busy = false`. For `vsetvl` (decoder stalled until scalars known): also `pvl_busy = false` at dispatch since the scalar value is captured into `vconfig.vl` directly.

**Verification.**
- Test sequence: `vsetvli a0, a1, e32, m2; vle32.v v0, (a0)`. Confirm the `vle32.v` uop sits in IQ_V_LOAD with `pvl_busy=true` until the `vsetvli` writeback completes; then issues.
- Confirm that *no* new broadcast bus was added — only existing wakeup ports are consumed.

---

## Step 10 — Vector register file

**Scope.** Implement the vector physical register file in Chisel, sized for `numVecPhysRegisters = 128`, width `VLEN = 256` bits. Reference `bobtail/main:src/main/resources/vsrc/vpu/tt_vec_regfile.sv`.

**Files created.**
- `src/main/scala/v4/vec/regfile/VecRegFile.scala`.

**Ports (Goal 1 only).**
- Write ports: 1 (vector LSU load writeback). Goal 2 adds a vector ALU write port.
- Read ports: 1 (vector LSU store-data read). Goal 2 adds vector ALU read ports.
- Implementation: a `Mem(numVecPhysRegisters, UInt(VLEN.W))` is fine for Goal 1. Banking deferred to Goal 2 once port counts grow.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — instantiate; wire load writeback in, store data read out.

**Verification.**
- Unit test: deterministic writes to a sequence of pregs; read back; confirm values.

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
- `VecLoadQueue.scala` (32 entries default; one entry per dispatched vector load uop; tracks element-level progress and `vstart` on faults).
- `VecStoreQueue.scala` (32 entries).
- `VecLoadBuffer.scala` (line-fill / element-coalescing buffer in front of the DCache).
- `VecStoreBuffer.scala` (commit-deferred store buffer — only drains when ROB marks the store committed).
- `VecAgu.scala` (one for loads, one for stores — generates per-element addresses based on `ConfigInfo` from Step 8).
- `VecMemUnit.scala` (top-level, owns the DCache request port for vector traffic).

**Behavior requirements.**
- Within a single vector uop: process elements in program/address order. On exception (page fault, misalign, etc.), set `vstart` to the failing element index; raise the exception against the uop's ROB entry.
- Across uops: V_LOAD and V_STORE may execute OoO with respect to each other and to scalar memops. The vector store buffer holds store data until the ROB marks the store committed (point-of-no-return).
- Supported types: unit-stride, strided (positive and negative stride), indexed (ordered and unordered), unit-stride segment, strided segment, indexed segment, fault-only-first (`vleff`).
- Mask support: per-element masking via `pvm`-pointed mask register; masked-off elements skip the cache.
- DCache port: route vector requests through the existing scalar LSU's DCache port via arbitration — vector traffic and scalar traffic share `lsu_io.dmem.req`. **Do not** add a separate DCache port for vector traffic in Goal 1 (would require DCache changes that complicate Goal 2).

**Files modified.**
- `src/main/scala/v4/lsu/lsu.scala` — add arbitration with the new vector LSU on the DCache request port. Vector traffic is lower priority than scalar by default (configurable).
- `src/main/scala/v4/exu/core.scala` — instantiate `VecMemUnit`; wire issue → `VecLsDecode` → `VecMemUnit`; wire load writeback into the vector regfile and the ROB.

**Verification.**
- Goldens against the spike RISC-V ISA simulator:
  - Unit-stride load/store: `vle{8,16,32,64}.v` / `vse*.v` at LMUL ∈ {1, 2, 4, 8}.
  - Strided: `vlse32.v / vsse32.v` with stride ∈ {0, 4, -8, 256}.
  - Indexed: `vluxei8.v / vsuxei8.v`.
  - Segmented: `vlseg3e16.v / vsseg2e32.v`.
  - Masked: each of the above with a `v0` mask register holding a non-trivial bitmask.
  - Fault-only-first: `vleff.v` crossing a page boundary; confirm `vstart` is updated and a precise trap is taken.

**Risks.** This is the largest step by far. Consider splitting into 11a (LQ/SQ/AGU bring-up, unit-stride only), 11b (strided + indexed), 11c (segment), 11d (mask + fault-only-first). Treat 11a as the minimum viable for the Goal 1 deliverable; 11b–d can land before the Goal 1 closeout but are not blockers for downstream integration testing.

---

## Step 12 — Tie off `IQ_V_ALU`

**Scope.** Vector arithmetic uops are decoded, cracked, renamed, and queued in `IQ_V_ALU`, but **must not** issue (no V-ALU exists yet). Goal 2 will replace this with a real VPU attach.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — for the IQ_V_ALU issue port, drive `grant := false.B`; assert that `IQ_V_ALU.io.iss_uops(0).valid` is never raised. Or equivalently: set `issueWidth = 0` for the V_ALU queue when in Goal 1 mode (require `if (goal1Only) issueWidth == 0`). Use an `enableVectorArith` sub-flag (default off in Goal 1, on in Goal 2).
- Add an assertion (`assert(!io.iq_v_alu.iss_valid, "Goal 1: V-ALU should not issue")`) so simulations explicitly trip if a vector ALU op accidentally gets through.

**Verification.**
- Goal 1 tests use only vector loads/stores and vset*. Vector arithmetic instructions in a test will cause the assertion to fire (expected for Goal 2; not for Goal 1).
- Boot a scalar program (Linux smoke test or equivalent) with `enableVector=true`; confirm zero vector ops are queued.

---

## Step 13 — `core.scala` integration sweep

**Scope.** Final wiring sweep. By this point every individual module has been instantiated and wired piecewise across Steps 0–12. Step 13 reviews `core.scala` end-to-end and fixes any lingering inconsistencies in wakeup port counts, branch-update fan-out, commit fan-out, and require() invariants.

**Files modified.**
- `src/main/scala/v4/exu/core.scala` — final cleanup.
- `src/main/scala/v4/exu/rob.scala` — if any port-count adjustment is still needed.
- `src/main/scala/v4/common/parameters.scala` — finalize `usingVector` derived parameter, finalize require() set.

**Verification.**
- Full elaboration of `WithNSmallBoomsVector`.
- All require() in core.scala and parameters.scala pass.
- Verilog diff against `WithNSmallBooms` (vector disabled): empty.

---

## Step 14 — Verification and sign-off

**Scope.** Sign off Goal 1 as a coherent feature.

**Tasks.**
1. **Regression**: `WithNSmallBooms` (vector off) — full csmith run, Linux boot test in Chipyard. Must match pre-Caracal.
2. **Vector ISA spike-comparison**: with Chipyard's spike-compare harness, run the riscv-tests vector subset (`rv64uv-p-*`) at LMUL ∈ {1, 2, 4, 8}, SEW ∈ {8, 16, 32, 64}, with and without mask. The arithmetic tests will fail (expected — V-ALU is tied off); the load/store tests should pass.
3. **Microarchitectural smoke tests**: hand-crafted assembly snippets for each cracker case, each LS variant, each `vset*` form. Place under `src/test/scala/v4/vec/sim/` or invoke from a Chipyard test target.
4. **Performance counters**: confirm vector-op counters (vector retirement, V_LOAD queue occupancy, V_STORE queue occupancy) are exposed via `BoomCustomCSRs` or perf-counter infrastructure for future tuning.
5. **Documentation update**: refresh `docs/boom-v4-architecture.md` to reflect the new vec pipeline (or write `docs/caracal-vec-architecture.md` as a sister doc).

**Acceptance criteria for Goal 1 sign-off.**
- All vector load/store ISA tests pass spike comparison at LMUL ≤ 8.
- Cracker handles widening (`vw*`) and segment (`vl*seg*`/`vs*seg*`) correctly.
- `vstart` is precise on element-level exceptions.
- Scalar regression unaffected.
- `enableVector=false` RTL diff vs. pre-Caracal v4 is empty.

---

## Non-goals for Goal 1 (deferred to Goal 2+)

- Vector arithmetic execution (V-ALU). `IQ_V_ALU` is built but tied off.
- VPU interface (custom interface between BOOM and an external vector unit).
- Reduction ops, mask logic ops, permutation ops.
- FP vector ops (vfadd, vfmul, ...) — depend on V-ALU.
- Vector register file banking — single-port suffices for LSU-only operation in Goal 1.
- DCache modifications for vector-friendly bandwidth — Goal 1 uses the existing single 64-bit DCache request port via arbitration.
- Performance tuning. Goal 1 is functional correctness; Goal 3+ will tune.

---

## Cross-cutting risks and mitigations

| Risk | Mitigation |
|------|-----------|
| Widening `*_rtype` from 2→3 bits ripples into many files | Done in Step 1; audited and tested before any vec logic lands |
| Cracker × branch-kill correctness | Sub-uops inherit parent's `br_mask`; tested in Step 3 verification |
| V-LSU and scalar LSU DCache contention | Step 11 adds arbitration with scalar priority by default; configurable |
| `vstart` precision on faults | Spec compliance is non-trivial; element-level testing in Step 11 verification |
| Cracker explosion at LMUL=8 with segment+widening (up to 64 sub-uops per architectural op) | Buffer in Step 4 absorbs the back-pressure; ROB stays at 1 entry per architectural op via shared `rob_idx` |
| Test coverage gap for indexed/segment/FoF loads | Step 14 explicitly enumerates these as required ISA-test coverage |
| `IQ_V_ALU` tied off accidentally allowing a vector ALU op to issue and corrupt state | Hard assertion in Step 12 |

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
| 11 | `lsu/VecLoadQueue.scala`, `lsu/VecStoreQueue.scala`, `lsu/VecLoadBuffer.scala`, `lsu/VecStoreBuffer.scala`, `lsu/VecAgu.scala`, `lsu/VecMemUnit.scala`, `lsu/VecMemopFsm.scala`, `lsu/VecIdxLsFsm.scala`, `lsu/VecMaskFsm.scala` |

**Modified (baseline files — kept minimal):**

| File | Steps touching | Reason |
|------|----------------|--------|
| `v4/common/parameters.scala` | 0, 7, 13 | `enableVector`, `VectorParams`, new `require`s, derived `usingVector` |
| `v4/common/config-mixins.scala` | 0 | `WithVector` mixin |
| `v4/common/consts.scala` | 1, 7 | `RT_VEC`; widen `iq_type`; new `IQ_V_*` |
| `v4/common/micro-op.scala` | 1 | Vector fields (`is_vec`, `pvs*`, `VConfig`, cracker bookkeeping, segment fields) |
| `v4/exu/decode.scala` | 2 | Hook in `VDecode`/`VLSDecode`/`VsetDecode`; `vsetvl` decoder stall |
| `v4/exu/dispatch.scala` | 7 | Route to new vector queues (no change if bitwise routing already works post-widening) |
| `v4/exu/rob.scala` | 6 | New writeback ports, `stale_pvdest`, cracked-uop busy-counter |
| `v4/exu/core.scala` | 3, 4, 5, 7, 8, 10, 11, 12, 13 | Top-level wiring of every new module |
| `v4/lsu/lsu.scala` | 11 | DCache port arbitration with V-LSU |

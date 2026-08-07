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
  VecPipeline — the whole Caracal vector subsystem behind ONE bundle: decode,
  two rename spaces, three issue queues, two register files, the vector LSU and
  the coprocessor host. It contains every vector module and owns no datapath of
  its own; what it owns is TOPOLOGY and the design-wide rulings that no single
  child could settle.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/VecPipeline.scala,
  package boom.v4.vec.generated, group vec.
  depends_on MicroOp, VecBundles, VectorParams, VecTrace.
  Instantiated ONCE, in `core.scala`, as the sole vector instance. It exists only
  when `usingRVV` is true — a Scala `Boolean` of `BoomCoreParams`, NOT a hardware
  `Bool` and NOT rocket's `usingVector` — so a vectors-off build has this entire
  subtree ABSENT rather than tied off.

  Nine children, eleven instances: `vdec` (VecDecode); `vec_rename` and
  `vl_rename` (two instances of ONE VecRenameSpace definition); `iq_v_load`,
  `iq_v_store`, `iq_v_alu` (three instances of ONE VecIssueUnit definition);
  `vrf` (VecRegFile); `vlrf` (VlRegFile); `vlsu` (VecLsu); `cii` (VecCiiHost).

  ===> THIS FILE IS WHERE THE CROSS-NODE RULINGS LIVE. Eleven questions were
       deferred here by children that could each only see one side. Each is
       settled in the logic section, in one place, with the reason:
         1. VRF read latency and WHO HOLDS THE FLOP (part 8) — VecRegFile holds
            it, one flop per read port; VecCiiOperandServer deletes its payload
            register. This file states that ONCE, as the canonical answer.
         2. VL-RF read timing (part 9) — combinational, unlike the VRF, and why
            that is not an inconsistency.
         3. The three wakeup networks and their exact fan-out (part 6), including
            the `aluWidth + 1` VL lanes of decision D8.
         4. The group-done fan-out to three consumers, and lanes-not-arbiters on
            every completion path (part 7).
         5. The decode-to-ren2 shadow register for `vl_imm` (part 3).
         6. `numVecWbPorts` / `numVecClrPorts` / `numVlWakeupPorts` — now DECLARED
            in `VectorParams` and BOUND here, not defaulted (parameters section).
         7. Where the four homeless bundles live (part 13), `VecRobFlags`
            included (A34).
         8. `dec_fire`, and `MicroOp.vconfig` on every br_tag-allocating uop
            (part 2).
         9. `fu_types` toward the load/store queues is a CONSTANT (part 11) —
            the vector LSU has no back-pressure into issue, by ground rule.
        10. The FIFTH group-readiness matcher's side channel (decision D6):
            `rdy_vold` on the `IQ_V_LOAD` and `IQ_V_ALU` slots means the per-member
            channel carries FIVE groups, not four, and this container routes it
            (parts 5 and 13).
        11. What `dis_ready` still means now that the vector queues are wired
            NATIVELY through `CompactingDispatcher` (decisions D2/D3) — whole-bundle
            allocation only, never queue capacity (part 4).

  ===> GATE (f) DEPENDENCY, stated so no reviewer re-derives it. Every gating
       claim here is the structural form of `usingRVV`: instance ABSENCE, never a
       tied-off instance. That is independent of the open plan-level ruling on gate
       (f) bit-identity (SEAM_NOTES, "TOP OPEN RULING"), which concerns the WIDTH
       of the shared `RT_*`/`IQ_SZ` encodings, not this container. Two choices here
       touch it and BOTH hold either way: part 5's `iq_type(IQ_V_*)` routing needs
       those three positions to exist and be defaulted on scalar uops (A23), and
       part 5's `dst_rtype === RT_VEC` assertion needs the 3-bit `RT_*` encoding —
       under option (a) both exist unconditionally, under option (b) both exist
       exactly when `usingRVV`, which is when this module exists at all. No ruling
       in this file changes with the outcome.

  ===> AND THE ONE THING THIS CONTAINER MUST NEVER BECOME. `addvector` put 1101
       lines of vector wiring into `core.scala`, braided line-by-line through
       scalar wiring, and the M1 free-list double-free hid in exactly that
       braiding: vector rename ran off `dec_uops`, one cycle ahead of the scalar
       `RenameStage`'s registered ren1-to-ren2 pipeline. Every lockstep input on
       this boundary is therefore named `ren2_*`/`dis_*`, so connecting a decode
       signal to one is visibly wrong AT THE CONNECTION SITE.

  Governing spec anchors: overview.rst `boom-relationship`, `caracal-pipeline`;
  glossary.rst `glossary-terms`; execution.rst `execution-pipelines`,
  `vector-execution`; midcore.rst `rename-stage`, `cii-shared-mapping`,
  `vl-vtype-rename`, `regfiles-bypass`, `old-vd`, `group-done-wb`, `vrf-ports`,
  `spec-wakeups`; issue.rst `cii-shared-sched`, `shared-store-chain`,
  `issue-sched-stage`, `issue-vl-delivery`; frontend.rst `vector-rvv-decode`,
  `vset-dual-dest`, `vl-delivery`; cii.rst `cii-mem-order`.
*/

<|begin_module|>

  <|begin_parameters|>
  This container introduces no tuning knob. It BINDS numbers that the corpus
  names and no written file declares, and it forwards the rest. Every value is a
  Scala `Int`/`Boolean` resolved at elaboration; nothing here is a hardware
  signal, so a sizing change ripples through elaboration instead of desynchro-
  nising one side of a seam.

  ---- Bound from `VectorParams`, which now declares them (A35 CLOSED) ----

  These three were the A35 gap — named by requirements, declared by no file, and
  defaulted independently by three children. `VectorParams` DECLARES ALL THREE NOW,
  so this container BINDS to those names and DECLARES NOTHING: no literal 3, no
  local `val`, no re-default. A child that still defaults one is a bug — the count
  must come down the constructor from here, and here it comes from `VectorParams`.

  `numVecWbPorts` (`VectorParams`, 3). The width of the VECTOR (group-done) wakeup
  network, and the producer count that sizes `vec_rename`'s busy-table clear side
  and every vector issue slot's per-member matcher. What this container still owns
  is the LANE ORDER, which no parameter can express, and it is ENUMERATED here and
  used everywhere below: lane 0 the Load-Coalescing Buffer, lane 1 VecCiiComplete,
  lane 2 VecGroupCopy.

  `numVecClrPorts` (`VectorParams`, 3) — the same number for a DIFFERENT reason,
  which is why it is a separate name: the ROB busy-clear is one lane per completion
  producer, in the same lane order, and the two counts would legitimately diverge
  the day a producer completes a ROB entry with no vector destination group. That
  already happens for a scalar-dest CII op (`vmv.x.s`, `vcpop.m`, `vfirst.m`,
  `vfmv.f.s`), where lane 1 clears the ROB with no group-done at all. Require
  `numVecClrPorts >= numVecWbPorts`.

  `numVlWakeupPorts` (`VectorParams`) = `aluWidth + 1`, NOT 2. See part 6 for the
  network and decision D8 for the count: one lane per integer ALU EU, because the
  `vset` writeback is REPLICATED per ALU EU and `aluWidth` is never 1 in the vector
  matrix, plus one lane for the LSU's `vleff` trim. `vl_rename`'s `numWbPorts`
  binds to this, and so does the `vl_wakeup` seam member's lane count.

  ---- Forwarded, not re-declared ----

  `coreWidth` is `plWidth`/`dispatchWidth` for every child. With SmallBoom REMOVED
  from the vector configuration matrix (decision D3) its reachable values are 2
  (Medium), 3 (Large) and 4 (Mega) — never 1 — and two facts this container relies
  on follow from that and from nowhere else: `allocWidth = coreWidth * 8 >= 16`, so
  a shared `OP.v` needing `2 * maxGroupSize` PRNs all-or-nothing can always
  allocate (that was a DEADLOCK at `coreWidth = 1`, not a stall), and
  `aluWidth == coreWidth >= 2`, which is why the VL wakeup network and VlRegFile's
  `W_alu` are replicated on every tier rather than only on wide ones.
  `numIntWakeupPorts` and `numFpWakeupPorts` come from the core's existing counts —
  the vector side adds no scalar wakeup port. `aluWidth` comes from
  `HasBoomCoreParameters` (`aluIssueParam.issueWidth`) and sizes `vset_resp`,
  `numVlWakeupPorts` and `vlrf`'s `numAluWritePorts`. `lsuWidth` selects whether the
  VRF has W1, and it is read here ONLY to size the connection arrays; `W_CII =
  lsuWidth` (A17), never a literal 2, because the write array is compacted in
  canonical W0,W1,W2 order and W1 is ABSENT — not tied off — at `lsuWidth = 1`.
  `vecIssueEntries` (16), `vecIssueGrantWidth` (1) and every VRF/VL-RF size come
  from `VectorParams`.

  ---- The child bindings, verbatim from hierarchy.yaml ----

  `vec_rename`: numArchRegs 32, maxGroupSize `maxMembers` (8), numPhysRegs
  `numVecPhysRegisters`, numWbPorts `numVecWbPorts`, freeDiscipline "stale_group",
  wakeupKind "group_done", hasRenameWrite false, exportMemberRdy TRUE.
  `vl_rename`: numArchRegs 1, maxGroupSize 1, numPhysRegs `numVlPhysRegisters`,
  numWbPorts `numVlWakeupPorts`, freeDiscipline "committed_ptr", wakeupKind
  "ready_bit", hasRenameWrite TRUE, exportMemberRdy false.
  `iq_v_load`: iqType IQ_V_LOAD, pnrGate false. `iq_v_store`: IQ_V_STORE, pnrGate
  false. `iq_v_alu`: IQ_V_ALU, pnrGate TRUE. All three: numEntries
  `vecIssueEntries`, issueWidth `vecIssueGrantWidth` (require 1 for `iq_v_alu`),
  numVecWbPorts as above, numFpWakeupPorts non-zero for `iq_v_alu` ONLY.
  `vlrf`: numExeReadPorts 3 (one per vector issue queue: the load AGEN's operand
  read, the store AGEN's, and `cii`), numAluWritePorts `aluWidth` (`vec_pipeline_io`
  presents `vset_resp` as `Vec(aluWidth, Valid(ExeUnitResp))` — decision D8, one
  writeback per ALU EU, never arbitrated), numLsuWritePorts 1, W_ren replicated
  `coreWidth`. Total VL-RF write ports `coreWidth + aluWidth + 1`: 5 at Medium,
  9 at Mega.
  `vrf`: the canonical partition, 9R and 2W at `lsuWidth = 1`, 9R/3W at 2.

  `srcReadLatency` = 1 is passed to `cii` and is the parameterised form of the
  part-8 ruling: it is the OBSERVABLE request-to-beat latency, and the flop that
  implements it lives in `vrf`.

  Require `usingRVV` at the top of this module's elaboration and nowhere inside
  it: the gate is instance existence, checked once.
  <|end_parameters|>

  <|begin_ports|>
  CLOCK AND RESET. One `core_clk` domain, POSEDGE clock, ACTIVE-HIGH SYNCHRONOUS
  `core_reset`, both implicit through `BoomModule` — the hierarchy.yaml defaults.
  No second clock domain, no asynchronous reset and no CDC anywhere in this
  subtree. The single active-low crossing in the design is produced inside
  `cii`'s SV shim from the `core_reset` it is handed, un-inverted.

  ---- The whole interface is ONE port ----

  `io : VecPipelineIO` (declared in `pkg/VecBundles.nlhdl.scala`, realizing the
  `vec_pipeline_io` interface entry of hierarchy.yaml). Forty-odd members, and the
  interface entry is authoritative over any child's recollection of it — it has
  been amended repeatedly. This module declares NO port outside that bundle. In
  particular it declares no `busy`, no `active`, no `vec_ready` and no
  per-instruction status of any kind: the only backward signals on this boundary
  are `dis_ready` (a WHOLE-BUNDLE ALLOCATION answer, consumed in program order —
  part 4, and no longer a queue-capacity answer), the per-lane vector-queue
  readiness the dispatcher needs (part 5), `lsu_fencei_rdy_vec` (feeding BOOM's
  pre-existing `fencei_rdy`) and the writeback/completion/exception paths.

  Five groups, by what this container does with each: (1) DECODE FEED straight to
  `vdec` — `dec_insns`, `dec_valids`, `dec_fire`, `dec_uops_in` in,
  `dec_uops_out`/`dec_vec_illegal` out; (2) LOCKSTEP RENAME/DISPATCH —
  `ren2_uops`, `ren2_mask`, `dis_fire` in, `dis_ready` and `dis_uops_out` out,
  REGISTERED ren2 values and never decode values; (3) SPECULATION/RECOVERY/COMMIT, fanned to every child
  that needs them — `brupdate`, `rob_pnr_idx`, `rob_head_idx`, `rob_flush`,
  `rob_flush_kill`, `commit_valids`, `commit_uops`, `commit_rollback`; (4) THE
  SCALAR SEAMS — `int_rf_read_req`/`_rsp` (5 lanes), `fp_rf_read_req`/`_rsp` (ONE
  lane, not two — decision D4 deleted the store-side FP reader),
  `int_wakeups`, `fp_wakeups`, `int_wb_snoop`, `int_wb`, `fp_wb`, `vset_resp`,
  `csr_vector`, `csr_frm`, `csr_vs_dirty`, `vl_wakeup`, `lsu_vec`,
  `lsu_fencei_rdy_vec`; (5) COMPLETION AND OBSERVABILITY — `vec_clr_bsy`
  (`numVecClrPorts` lanes), `vec_clr_unsafe`, `vec_rob_flags`, `vec_xcpt`,
  `vec_trace_en`, `debug_vrf_read`.

  ---- Interface members this container REQUIRED: three APPLIED, one OUTSTANDING ----

  Named here, at the boundary that needs them, rather than resolved silently.
  The first three are now PRESENT in the `vec_pipeline_io` entry and are recorded
  so no reviewer re-derives why they exist:

  - `rob_empty` (fwd, 1 bit, `rob.io.empty`). `VConfigUnit` names it as a
    CHECK-ONLY input for its quiescent-state assertion (`vcfg_shadow ===
    csr_vtype` whenever the ROB is empty). Tying it false would leave the
    assertion elaborated and vacuous, which is worse than not having it: a vtype
    divergence from Whisper then surfaces as a wrong EMUL many stages later.
  - `commit_vl` (bwd, `Valid(UInt(vecVLSz.W))`). The data side of VlRegFile's
    single commit read port. See part 9 for why the ADDRESS side needs no member.
  - `vl_wakeup` WIDENED to `numVlWakeupPorts` lanes, i.e. `(vlPregSz + 1) *
    (aluWidth + 1)`. See part 6 and decision D8. `vset_resp` is likewise now
    `ExeUnitResp * aluWidth` — a Vec, not a single response.

  STILL OUTSTANDING, and it is the one thing decision D2 obliges on this boundary:
  the three vector queues' PER-LANE dispatch handshake. With `CompactingDispatcher`
  the vector queues are wired natively, so each queue's per-lane `ready` must reach
  the dispatcher and the dispatcher's per-lane `valid` must reach the queue. That is
  `dis_vec_ready` (bwd, `coreWidth * 3`) and `dis_vec_valids` (fwd, `coreWidth * 3`),
  one lane group per `IQ_V_*` queue in the fixed order load, store, ALU. Part 5 has
  the mechanism and the one trap that comes with it.

  ---- Three interface names with no declaration, bound here ----

  `IntWakeupBus` is the aggregate of the three scalar broadcast terms every issue
  queue needs, and it is one bundle because all three are the same fan-out with
  the same timing: `wakeups: Vec(numIntWakeupPorts, Valid(new Wakeup))`,
  `child_rebusys: UInt(aluWidth.W)` and `squash_grant: Bool` (BoomCore's
  `alu_exe_units.map(_.io_squash_iss).reduce(_||_) || io.lsu.iwakeups.map(_.bits.
  rebusy).reduce(_||_)`, exactly the term the scalar queues get). `FpWakeupBus`
  carries `wakeups` ONLY — BOOM v4 drives `rebusy := false.B` on every FP wakeup
  port and has no FP speculative-load-hit wakeup at all, so an FP rebusy term
  would be dead wires (A33). `IntWbSnoop` is `{addr: UInt(pregSz.W), data:
  UInt(xLen.W)}` per INT write port, and the array MUST include the write port
  `enableVectorArith` adds, or a base address produced by `vmv.x.s` is missed.
  All three are members of `VecPipelineIO` and belong in `VecBundles` beside it.
  <|end_ports|>

  <|begin_logic|>
  No datapath, no arithmetic and — with exactly two exceptions, both named — no
  state. The two are the per-lane `vl_imm` shadow register of part 3 and nothing
  else. Everything else in this section is a connection, a select, an assertion or
  a ruling. If a future edit adds a third register here, it is almost certainly a
  child's job.

  ---- PART 1. What this container is, and the three streams ----

  //@req-spec-core.a2
  //@req-spec-core.a3
  Caracal supports RVV 1.0 with out-of-order vector load/store, and the RVV state
  and datapath that implements it is exactly this subtree: nothing vector-visible
  exists outside it except the architectural CSR cells (rocket's `CSRFile`) and
  the deltas to BOOM's own shared files.

  //@req-spec-core.a5
  //@req-spec-core.a6
  //@req-spec-core.d5
  BOOM v4's out-of-order pipeline is kept INTACT and the vector path is threaded
  THROUGH it: this container consumes BOOM's decode bundle, BOOM's registered ren2
  bundle, BOOM's `dis_fire`, `brupdate`, PNR, flush and commit ports, and BOOM's
  INT/FP wakeup networks — and adds no second frontend, no second ROB, no second
  commit point and no second scheduling stage. BOOM remains the out-of-order
  scalar host, the scheduler and the VRF owner; the coprocessor executes RVV
  arithmetic and holds no architectural state. That is why the seam is one bundle
  with a rename lockstep contract rather than a coprocessor port: the vector path
  is scheduled by the host, not handed to a device.

  //@req-spec-core.i3
  //@req-spec-cii.d14
  The execution of vector instructions is split into THREE STREAMS, and the split
  is visible in the instance map rather than implied: vector arithmetic /
  reduction / permutation through `iq_v_alu` to `cii`; vector loads through
  `iq_v_load` to `vlsu`'s load direction; vector stores through `iq_v_store` to
  `vlsu`'s store direction. The two memory streams are OUT OF ORDER with respect
  to each other and to program order; the arithmetic stream is IN ORDER inside the
  coprocessor. That asymmetry is deliberate and is the reason `iq_v_alu` is
  age-ordered-collapsing with a per-entry PNR gate rather than a head-only FIFO:
  the VPU being in-order requires one-at-a-time EXECUTION, not program-order
  ISSUE, and a head-only queue would let a segmented store's coprocessor half
  block every younger vector arithmetic op for hundreds of cycles.

  //@req-spec-cii.j1
  //@req-spec-cii.j2
  //@req-spec-cii.j3
  The CII moves NO memory traffic of its own, and the topology is what proves it:
  `cii` has no D$ port, no TLB port, no LSQ port and no member of `lsu_vec`. Every
  vector load and store is a host LSU operation issued from `iq_v_load`/
  `iq_v_store` into `vlsu`. Arithmetic offloaded to the coprocessor is therefore
  ordered against memory purely by the ROB (program order at commit, and the PNR
  gate before issue) and by the register dependences rename already resolved —
  there is no memory fence, no address comparison and no ordering channel between
  `cii` and `vlsu`. The one shared resource is the VRF, and the port partition of
  part 8 keeps even that contention-free.

  ---- PART 2. The decode arm ----

  `vdec` receives `dec_insns`, `dec_valids`, `dec_fire`, `dec_uops_in` and returns
  `dec_uops_out`, `dec_vec_illegal`, plus `dec_vl_imm`/`dec_vl_imm_valid` for part
  3. Its `vcfg` pass-throughs are supplied from here:

  - `ren_br_tags` — `Vec(coreWidth + 1, Valid(UInt(brTagSz.W)))`, BUILT HERE, not
    taken off the seam: entry 0 tied invalid, entry `w + 1` is
    `{valid = io.dis_fire(w) && io.ren2_uops(w).allocate_brtag, bits =
    io.ren2_uops(w).br_tag}`. That is baseline `rename-stage.scala:117-118`
    verbatim with `ren2_fire = io.dis_fire`, so the vtype snapshot table takes its
    snapshots on exactly the cycles the scalar branch snapshots are taken. Deriving
    it here costs two gates and removes a seam member that could disagree.
  - `ren_br_vconfig` — index-aligned with it, entry `w + 1` is
    `io.ren2_uops(w).vconfig`.

  //@req-spec-decode.c22
  ===> TWO OBLIGATIONS ON THE MicroOp DELTA FALL OUT OF THOSE TWO LINES, and both
       are stated here because this is the only place both sides are visible.
       (a) `MicroOp.vconfig` must be written on EVERY uop that can allocate a
       `br_tag`, not only on vector ones. A branch taken between two `vset`s
       snapshots `ren_br_vconfig(w+1)` from whatever uop allocated the tag — a
       scalar branch, usually — and an unwritten `vconfig` there restores a
       don't-care `vtype` on the mispredict. (b) `dec_fire` is a REQUIRED member:
       a decode bundle can fire partially, so a mirror keyed on `dec_valids` would
       absorb a `vset` from a lane that did not advance and then absorb it again
       when the bundle re-presents. Also on the same seam: a register-sourced
       `vset` sets BOTH busy bits and BOTH wakeup networks fire (part 6) — integer
       busy for `pdst` through the scalar rename, VL busy for `pvl` through
       `vl_rename` — and the two are independent by construction here, because
       they live in different rename spaces with different networks.

  `csr_vtype` is `io.csr_vector.vconfig.vtype`; `rob_empty` is the required new
  member. Both are check-only and must reach no mirror, shadow or output.
  `brupdate`, `commit_rollback` (as `rollback`) and `commit_valids` (as
  `com_valids`) pass through. `com_vtype` is `io.commit_uops(w).vconfig`, which
  the ROB delta OVERRIDES at commit from its per-row `rob_vconfig` latch, so a
  register-sourced `vsetvl` commits the vtype its ALU RESOLVED and not its decode
  snapshot. `com_is_vset` is derived here as `io.commit_valids(w) &&
  io.commit_uops(w).is_vl_producer && !io.commit_uops(w).is_vec` — there is no
  `is_vset` field and none is needed: a `vset` is modelled as a SCALAR uop, and
  the only other VL producer is `vleff`, which has `is_vec` set. Deriving it from
  `is_vl_producer` alone would make every `vleff` commit overwrite the committed
  vtype shadow with its own snapshot.

  ---- PART 3. The one shadow pipeline, and why it is here ----

  //@req-spec-decode.i4
  `vdec` computes the VL of a front-end-only `vsetivli` at DECODE
  (`dec_vl_imm`/`dec_vl_imm_valid`), because `min(uimm, VLMAX)` needs the
  decode-cycle vtype mirror; but the VL-RF write and the `pvl` broadcast happen at
  RENAME. This container therefore holds a per-lane register pair, `ren2_vl_imm`
  and `ren2_vl_imm_valid`, ENABLED BY `io.dec_fire(w)` — the same lane-fire that
  admits the uop into the scalar `RenameStage`'s ren1-to-ren2 register — and feeds
  them to `vl_rename`'s same-named inputs, where consumption is qualified by
  `ren2_mask(w) && dis_fire(w)`.

  // ===> `dec_fire(w)` IS THE CORRECT ENABLE AND `ren2_ready` IS NOT AVAILABLE
  // HERE, and the two are equivalent for this purpose: baseline loads `r_uop`
  // when `ren2_ready` and sets `r_valid := ren1_fire(w) = dec_fire(w)`, so on a
  // `ren2_ready` cycle with no `dec_fire` the lane is INVALID at ren2 and the
  // stale shadow value is never consumed. `dec_fire` implies `ren2_ready`.
  // A shadow pipe that ran a cycle ahead of `ren2_uops` — or was recomputed at
  // ren2 from the uop's immediate — is the M1 free-list double-free in a second
  // place: it would pair a VL value with the NEXT cycle's (bubble) uop and write
  // it into another instruction's `pvl`. No kill term is needed on the pair,
  // because every consumer is qualified by the ren2 valid the scalar stage
  // already branch-masks.

  ---- PART 4. The rename arm: two instances of one definition, CHAINED ----

  //@req-spec-rename.h15
  `vec_rename` and `vl_rename` are two instances of ONE `VecRenameSpace`, and they
  are CHAINED, not joined field-by-field: `io.ren2_uops -> vec_rename.ren2_uops`,
  `vec_rename.ren2_uops_out -> vl_rename.ren2_uops`, and `vl_rename.ren2_uops_out`
  IS the dispatch bundle this container routes in part 5. Each instance writes only
  the fields it renamed, so the chain is the join and there is no re-derivation of
  which space owns which field. `br_mask` is not touched by either — the scalar
  stage owns it. `ren2_mask`, `dis_fire`, `brupdate`, `commit_rollback` (as
  `rollback`), `commit_valids`/`commit_uops` go to both. `vl_rename` is the VL
  rename space in full: its own map table, free list and busy table over one
  architectural register — which is what makes the dedicated VL wakeup network of
  part 6 a network over a real register space and not a bolt-on ready bit.

  `dis_ready` (one bit for the whole bundle) is the AND of exactly THREE
  WHOLE-BUNDLE ALLOCATION CONDITIONS and nothing else: `vec_rename.alloc_ok`,
  `vl_rename.alloc_ok`, and `vlsu.dis_ok` reduced over the lanes that request a
  vector memory reservation. One bit, broadcast, because a partial vector
  allocation does not exist: the free list grants a whole bundle or none, and a
  shared `OP.v` needs `2 * maxGroupSize` PRNs atomically. BoomCore ORs the inverse
  into `ren_stalls(w)` for EVERY lane.

  ===> ISSUE-QUEUE CAPACITY IS NOT IN THAT AND ANY MORE (decision D2). An earlier
       draft folded each lane's `dis_uops(w).ready` — the readiness of every queue
       that lane's `iq_type` names — into this one bit. That is exactly what D2
       rejected: collapsing per-lane back-pressure onto a broadcast bit makes a full
       `IQ_V_LOAD` stall PURE-SCALAR LANES carrying no vector uop at all, which
       threatens gate (d) and target P6. Queue readiness is now NATIVE and PER LANE
       through `CompactingDispatcher` (part 5), whose `rdy := ren.ready || !uses_iq`
       masks a queue's ready by whether the lane actually uses it. `dis_ready`
       answers only "can the whole bundle's vector RESOURCES be allocated", which is
       genuinely a whole-bundle question and genuinely cannot be per-lane. Do not
       merge the two answers back together in either direction: a per-lane
       allocation answer would leak half-allocated groups, and a broadcast capacity
       answer is the stall D2 removed.

  // ===> `alloc_fire` IS PER LANE EVEN THOUGH `alloc_ok` IS NOT (A1). BOOM's
  // `dis_stalls` is a prefix scan, so a NON-vector hazard — `ldq_full` on lane 2 —
  // still lets lanes 0-1 dispatch. Consumption must therefore be qualified by
  // `io.dis_fire(w)` per lane; a single fire bit consumes the non-firing lane's
  // window PRNs, that lane retries and allocates a second group, and the first is
  // owned by nobody. The `reqs` side is driven FIRE-INDEPENDENTLY, off the ren2
  // uops, or `alloc_ok -> dis_ready -> dis_fire -> reqs` closes a combinational
  // loop.

  `vec_rename.member_rdy` (`exportMemberRdy`) goes to part 5's dispatch routing as
  the per-member readiness side channel, carrying FIVE groups (D6, part 5) — `vs1`,
  `vs2`, `vs3`, `vtmp`, `vold` — plus the single mask bit. `vec_rename.wakeups` is the three-lane
  VECTOR network of part 6; `vl_rename.wakeups` is the `aluWidth + 1`-lane VL
  network of the same part.
  `vl_rename.vl_rf_write` is VlRegFile's `W_ren`, `coreWidth` lanes, unarbitrated.

  ---- PART 5. Dispatch: routing, and the single-cycle IQ slot write ----

  //@req-spec-issue.e1
  //@req-spec-issue.e3
  Caracal uses SPLIT ISSUE QUEUES, as BOOM does, and adds exactly three:
  `iq_v_load`, `iq_v_store`, `iq_v_alu`. The four scalar queues are untouched and
  are not visible here. Each vector queue may hold any datatype — there is no
  per-datatype queue and no width class in the routing.

  //@req-spec-issue.e4
  //@req-spec-issue.e5
  All queues issue in a SINGLE scheduling stage. This container adds no second
  issue stage, no pre-issue arbitration, no cross-queue grant and no register
  between a grant and its consumer: `iq_v_load.iss_uops(0)` reaches
  `vlsu.iss_ld`, `iq_v_store.iss_uops(0)` reaches `vlsu.iss_st` and
  `iq_v_alu.iss_uops(0)` reaches `cii.iss`, each combinationally.

  //@req-spec-rename.b10
  //@req-spec-rename.b11
  EVERY uop OF A DISPATCH GROUP WRITES ITS ISSUE-QUEUE SLOT IN THE RENAME CYCLE,
  vector and scalar alike, and no uop lags its group-mates. The topology is what
  guarantees it: rename here is single-stage and combinational from `ren2_uops`
  (part 4), the routing below is a fan-out of that same combinational bundle
  qualified by `dis_fire`, and `dis_ready` is one bit for the whole group. So the
  vector half of a mixed group cannot be one cycle behind the scalar half — there
  is no register anywhere between `ren2_uops` and a slot write, and no lane-private
  stall that could hold one member back.

  //@req-spec-core.c9
  Routing is by the `iq_type` bitmask on the dispatch uop: lane `w` is presented to
  queue `q` when `iq_type(q)` is set, with `valid` qualified by `io.dis_fire(w)`.
  A `Vec(IQ_SZ, Bool)` mask — not a queue ID — is what lets a SHARED OP.v name TWO
  queues at once (part 10) while remaining ONE uOP: an OP.v stays a single uOP
  through decode, rename, ROB and issue, and is expanded into `nOP.v` only inside
  the vector LS AGEN, deep inside `vlsu`. Nothing in this container splits, cracks
  or replicates a uop, and there is no port on which a cracked uop could leave it.

  ===> THE QUEUES ARE WIRED NATIVELY, NOT TIED READY (decision D2). Vector configs
       instantiate `CompactingDispatcher`, and the three `IQ_V_*` `issueParams`
       entries are real dispatch clients: the dispatcher applies the SAME `iq_type`
       predicate this paragraph describes, per lane, and computes each lane's
       readiness as `rdy := ren.ready || !uses_iq` — "ready if this lane does not use
       the queue" — so a full vector queue back-pressures only the lanes holding
       vector uops. Each vector queue's per-lane `ready` therefore leaves this
       container (the `dis_vec_ready` member) and the dispatcher's per-lane decision
       comes back in (`dis_vec_valids`); neither may be tied off, and the earlier
       plan of tying the dispatcher's vector lanes `ready := true.B` is dead — it
       was only ever needed because `BasicDispatcher` ANDs every queue's ready into
       every lane.
       // ===> THE TRAP THAT COMES WITH IT, and it must not be discovered in
       // simulation: `CompactingDispatcher` COMPACTS, so a queue's dispatch lane is
       // not necessarily the rename lane of the uop occupying it. Whatever payload a
       // vector queue latches must be the CHAINED-RENAME output — the bundle with
       // `pvdest`, `pvs*`, `pvl` and `stale_pvdest` written (part 4) — so the
       // dispatcher must be fed `dis_uops_out`, NOT the raw scalar `ren2_uops`
       // bundle. Compacting the scalar bundle and handing a vector queue lane `w`
       // one of its entries delivers a uop with unwritten vector fields and no
       // width error anywhere. If the compaction is instead applied to a payload
       // this container selects internally, then the compaction PERMUTATION must
       // cross the seam too — one of the two, never neither.
       // ===> RESOLVED A THIRD WAY, by BoomCore's delta, and this is why the trap
       //      does not fire: an added `require(ip.dispatchWidth == coreWidth)` on
       //      the three IQ_V_* entries. At `dispatchWidth == coreWidth` a
       //      `Compactor` degenerates to `io.out <> io.in` (util.scala:458) — there
       //      IS no compaction and no permutation — so a vector queue's dispatch
       //      lane IS its rename lane, and each queue keeps latching THIS
       //      container's own lane-`w` chained-rename uop. Neither `dis_uops_out`
       //      nor a permutation crosses the seam; only the per-lane
       //      `dis_vec_valids`/`dis_vec_ready` wires do.
       //      The `require` is what keeps that sound: it holds on every tier in the
       //      matrix (Medium/Large/Mega all set `dispatchWidth = coreWidth`), and a
       //      future config that narrowed a vector queue's dispatch width would
       //      FAIL ELABORATION rather than silently deliver a uop with unwritten
       //      vector fields. Do not relax it without re-opening this trap.
  The member-readiness side channel is routed with the uop, lane for lane, into
  each queue's `dis_member_rdy`; it travels BESIDE the uop and never inside it, and
  it carries FIVE per-member groups, not four — see the next paragraph.

  ===> THE FIFTH GROUP IS ROUTED HERE (decision D6). `IQ_V_LOAD` and `IQ_V_ALU`
       slots each instantiate a FIFTH `VecGroupReady`, `rdy_vold`, gating issue on
       the readiness of the `stale_pvdest` GROUP; `IQ_V_STORE` keeps four. So the
       per-member side channel `vec_rename.member_rdy` exports, and this container
       routes into `dis_member_rdy`, has FIVE per-member group vectors — `vs1`,
       `vs2`, `vs3`, `vtmp` and now `vold` — plus the single mask bit. Wire all five;
       a channel sized four leaves `rdy_vold` with no dispatch-time initial state,
       which is a lost single-shot wakeup, i.e. a hang.
       WHY THE FIFTH GROUP EXISTS, since this is the container that has to pay for
       it: `stale_pvdest` is the PREVIOUS mapping of the destination arch vregs, so
       its producer is an OLDER instruction — and age-ordered issue grants the
       oldest READY entry, which does not imply an older producer has finished. Two
       consumers read the group anyway: the LCB pre-loads inactive-lane data from it
       on R2 for a `vta=0`/`vma=0` load (part 8), and the coprocessor may pull it as
       the `STALE_VD` source slot. PER-MEMBER matching is required and an aggregate
       `stale_pvdest_busy` bit was rejected: the group can span UP TO EIGHT
       PRODUCERS (an `LMUL=1` op wrote `v0`, a later `LMUL=8` op renames `v0..v7`),
       and one aggregate bit can neither express "waiting on producer 3 of 8" nor be
       cleared correctly by one group-done. Same reason `pvs*` needs per-member
       matching (`rename.g20`). The conservatism is accepted knowingly: no VPU-side
       signal says whether the coprocessor will actually pull `STALE_VD`, so every
       CII op with a vector destination waits on `stale_pvdest`.

  Assert at this boundary, rather than trusting the decoder: a uop presented to any
  vector queue has `is_vec` set and has this queue's `iq_type` position set; a uop
  with `dst_rtype === RT_VEC` names at least one vector queue; and no uop presented
  here has `ppred_busy` set. Also assert that the three `iq_type(IQ_V_*)` positions
  are false on every uop that reaches no vector queue — this is the cheap end of
  A23, whose fix (six default assignments in `DecodeUnit`) is invisible to gate
  (f): a don't-care bit elaborates bit-identically and still mis-routes.

  ---- PART 6. The three wakeup networks, and the partition that owns them ----

  //@req-spec-issue.f1
  //@req-spec-issue.f7
  BOOM wires each issue queue only to the wakeup network of its own register
  space, so that operand matches across spaces cannot collide; Caracal PRESERVES
  that partitioning and extends it with exactly TWO NEW NETWORKS — VL (`pvl`) and
  VECTOR (group-done) — plus the connections each vector queue needs to whichever
  SCALAR networks supply its scalar feeders. Two networks, and no more: there is
  no VLBU network, no value-broadcast network and no vector bypass network
  anywhere in this design.

  The table below is the whole of it, and it is normative:

    network   producers                          consumers
    INT       existing BOOM int wakeup ports     iq_v_load, iq_v_store, iq_v_alu
    FP        existing BOOM fp wakeup ports      iq_v_alu ONLY
    VL        aluWidth + 1 lanes (see below)     all three, + vl_rename busy
    VECTOR    3 group-done lanes                 all three, + vec_rename busy

  //@req-spec-issue.f8
  //@req-spec-issue.f9
  `io.int_wakeups` is fanned to ALL THREE queues, because the integer network is
  what delivers the base address, the stride and the GPR-sourced `.vx` scalar
  operand of an OP.v — every vector memory op has a base, a strided one has a
  stride, and a `.vx` arithmetic op has an integer source. `child_rebusys` and
  `squash_grant` ride the same bundle to all three, unchanged from the scalar
  queues' terms.

  //@req-spec-issue.f12
  //@req-spec-issue.f13
  `io.fp_wakeups` is fanned to `iq_v_alu` ONLY, for the scalar-FP source of `.vf`
  ops and `vfmv.*.f`. `iq_v_load` and `iq_v_store` elaborate `numFpWakeupPorts = 0`
  and have no FP comparator at all — vector memory addressing uses only GPRs, so
  an FP connection there would be dead silicon in every slot of two queues.

  // ===> THE RESIDUAL FP WINDOW IS CLOSED, AND A30 IS CLOSED WITH IT (decision
  // D4). It was a real exposure while there were TWO FP readers:
  // `FPExeUnit.io_wakeup` is a FAST wakeup with `bypassable := true` and a
  // writeback presented at T+3, so a slot woken at T that drove its FP read
  // address combinationally in the grant cycle could read data one cycle STALE.
  // D4 DELETED the store-side FP reader — no RVV store form takes an FP scalar
  // operand, since store data is `vs3` and base and stride are both integer — so
  // the ONLY FP reader left is `VecCiiIssue`, reached through `iq_v_alu`, and
  // `IQ_V_ALU` IS PAST-PNR GATED. A granted CII op is therefore OLDER THAN THE
  // PNR, which means its FP producer has genuinely written back: there is no
  // residual window between a bypassable match and the read, on any path.
  // DO NOT hold a BYPASSABLE FP wakeup match back one cycle inside
  // `VecIssueSlot`. That earlier recommendation is WITHDRAWN, not deferred —
  // implementing it now would cost a cycle on every `.vf` op to close a window
  // that cannot occur, and it would be invisible in simulation as a bug.
  // What D4 explicitly RETAINS is the MATCH ITSELF (`spec-vrf.e4`,
  // `spec-issue.g8/g9/g10`): the slot still watches `.vf` on the FP wakeup
  // network, because that is how it learns the scalar operand is ready. Only the
  // store-side reader, and the claim that a bypass window exists, are gone.

  //@req-spec-issue.f10
  //@req-spec-issue.h1
  THE VL NETWORK. All three queues connect to it, because any OP.v may depend on
  `pvl`. It is a PLAIN READINESS wakeup — a bare `pvl` physical register number,
  no value capture in the slot — and the VL VALUE is read from the VL register file
  at EXECUTE through the uop's `pvl` (part 9). Carrying the value on the network
  would mean 9 bits times slots times ports of capture registers to hold a number
  that a 64-entry file answers combinationally.

  //@req-spec-decode.i4
  A VL-producing instruction allocates a fresh VL PRN at rename, writes the new VL
  into the VL RF, and broadcasts `pvl` on this network. This container FORMS the
  network from its producers, because none of them can see the others. It is
  `numVlWakeupPorts` = `aluWidth + 1` lanes wide (decision D8):
    lanes 0 .. aluWidth-1 = `io.vset_resp(i)`, the writeback of integer ALU EU `i`:
             `{valid = vset_resp(i).valid && vset_resp(i).bits.uop.is_vl_producer,
             bits = vset_resp(i).bits.uop.pvl}`. Note the write enable is
             `is_vl_producer` and NEVER `dst_rtype` — `vsetvli x0, rs1` discards its
             integer destination and must still write VL.
    lane aluWidth = `vlsu.vl_wb`: the `vleff` trimmed count.
  A rename-cycle `vsetivli` needs NO lane: `vl_rename` leaves its `pvl_busy` clear,
  so it is born ready and there is nothing to wake.

  // ===> WHY ONE LANE PER ALU EU AND NOT ONE SHARED VSET LANE (D8). The vset
  // writeback is REPLICATED, not arbitrated, and this holds on EVERY tier rather
  // than only on wide ones: `aluWidth == coreWidth` on every tier and SmallBoom is
  // out of the vector matrix (D3), so the matrix is Medium(2)/Large(3)/Mega(4) and
  // `aluWidth` is NEVER 1. `ALUExeUnit` advertises the vset FU on every ALU EU
  // instance and its reject list forbids making that advertisement conditional on
  // the EU's `id`, so TWO vsets really can write back in one cycle. Add the `vleff`
  // trim, which is an independent pipeline, and the concurrency is genuine on both
  // axes.
  // ARBITRATION IS NOT AN OPTION, and the reason is correctness: a readiness wakeup
  // is SINGLE-SHOT in BOOM's slot model — the slot clears its busy bit on the match
  // and never looks again — so A VL WAKEUP LOST TO ARBITRATION IS A PERMANENT HANG,
  // never a stall. That is the same unrecoverable-clear argument that made
  // `vec_clr_bsy` one lane per producer, and it is already `VlRegFile`'s stated
  // "replicate, never arbitrate" discipline. It is cheap HERE SPECIFICALLY because
  // the VL RF is `64 x 9b`: write ports are Medium `2+2+1 = 5`, Mega `4+4+1 = 9`,
  // i.e. nine decoders on 576 flops — not twelve ports on 24 kbit, which is why the
  // identical argument is refused on the VRF.
  // REJECTED, and it is the plausible alternative: route vsets to `IQ_UNQ`.
  // `unqWidth = 1` on every tier, so a single writeback falls out by construction,
  // and the throughput cost is ~nil since a strip-mined iteration is 6+
  // instructions. It would require `UniqueExeUnit` — NOT A NODE IN THIS MAP — to
  // advertise the vset capability, and it stretches `spec-decode.c7`, which says a
  // vset must execute on an integer ALU execution unit.
  // If any lane of this network is ever narrowed, the ONLY acceptable fallback is a
  // one-deep skid register per producer inside this container with an assertion on
  // overflow (a readiness wakeup may be DELAYED safely, since the VL RF has no
  // read-during-write bypass and every wakeup-to-execute distance is already at
  // least one cycle) — never a dropped valid.

  //@req-spec-core.f6
  //@req-spec-issue.f3
  //@req-spec-issue.f4
  //@req-spec-issue.f5
  THE VECTOR NETWORK. `Vec(numVecWbPorts, Valid(new VecGroupDone))`, driven by
  GROUP-DONE events — one per completed destination group, each carrying that
  group's FULL member-PRN vector and its `members` count. ONLY the three `IQ_V_*`
  queues connect to it, and this container is where that is enforced: the bundle is
  fanned to `iq_v_load`, `iq_v_store`, `iq_v_alu` and to `vec_rename`'s busy-table
  clear side, and to nothing else. It crosses this boundary in NO direction — no
  scalar queue, no scalar rename space and no external consumer sees it, so the
  cross-space collision BOOM's partitioning prevents cannot occur. It is broadcast
  UNREGISTERED, so a slot and the busy table see the same completion in the same
  cycle and set beats clear.

  Lane assignment, fixed: lane 0 `vlsu`'s LCB group-done, lane 1 `cii.group_done`,
  lane 2 `vlsu`'s VecGroupCopy group-done. Assert `numVecWbPorts` equals the
  connected length in every consumer — a matcher sized smaller than the network
  examines fewer ports than are driven, misses a single-shot group-done, and hangs.

  // ===> `VecLsu` MUST EXPORT THE GROUP-DONE BUNDLES, NOT ONLY THE ROB CLEAR.
  // Its written port list declares `vec_clr_bsy` lanes 0 and 2 (a `rob_idx`) but
  // no `VecGroupDone` output, and the wakeup and busy-table consumers need the
  // MEMBER-PRN VECTOR. Required amendment: `vlsu.group_done : Vec(2,
  // Valid(VecGroupDone))`, lane-aligned with its two clear lanes.

  ---- PART 7. The group-done fan-out: one event, three consumers ----

  //@req-spec-core.g1
  //@req-spec-core.g3
  //@req-spec-rob.c5
  A vector producer emits EXACTLY ONE group-done per OP.v per completing
  destination group, and that ONE event drives THREE consumers at once: the ROB's
  single-shot `rob_bsy` clear (through `io.vec_clr_bsy`), the vector Busy-Table
  clear (`vec_rename.wakeups`), and the vector wakeup network (the three queues).
  The three structures then stay consistent BY CONSTRUCTION rather than by three
  producers agreeing. This container makes that structural in the cheapest possible
  way: for each producer lane the ROB clear is DERIVED from that lane's group-done
  (`vec_clr_bsy(i) := Valid(group_done(i).bits.rob_idx)`) rather than taken from a
  second port, so the two cannot be emitted in different cycles. There is no
  per-entry ROB completion counter anywhere in the design and no port on which a
  per-member completion could leave this container.

  The one exception, and it is why `numVecClrPorts` is its own name: a CII op with
  a SCALAR destination completes a ROB entry with no vector destination group. Lane
  1 is therefore the UNION of `cii.group_done`'s `rob_idx` and `cii.clr_rob`, and
  the assertion to carry is the IMPLICATION `group_done.valid -> clr_rob.valid &&
  same rob_idx in the same cycle`, never an equality.

  `vec_clr_bsy` is one lane per producer and is NEVER ARBITRATED. A clear lost to
  arbitration is UNRECOVERABLE: `cii` frees its tag in the same cycle it completes,
  and the LCB releases its assembly entry, so neither can regenerate the event and
  the ROB entry never retires. `vec_rob_flags` accompanies the same lanes (`fflags`
  and `vxsat` accrue per ROB entry and are applied at COMMIT, not at writeback,
  because a past-PNR CII op can still be squashed by a ROB-head flush); only lane 1
  carries anything today. `vec_clr_unsafe` is the single group-safe event from
  `vlsu` — the LSU half's FIRST address translation, one pulse per ROB entry, never
  per sub-access. `vec_xcpt` is `vlsu.vec_xcpt`, carrying no element index: a
  faulting vector memory op traps with `vstart = 0` and restarts whole.

  `csr_vs_dirty` is owned here because no child claims it: OR over commit lanes of
  `commit_valids(w) && (commit_uops(w).is_vec || commit_uops(w).is_vl_producer)`.
  Commit-sourced, for the same flush-safety reason as `vec_rob_flags` — a writeback
  pulse would dirty VS for an op that is later squashed.

  ---- PART 8. THE VRF RULING: read latency, the flop, and the partition ----

  ===> CANONICAL, AND THIS IS THE SINGLE ANSWER THE WHOLE SUBSYSTEM BINDS TO — no
       child may state a different one and no reviewer need look further. FOR EVERY
       PORT R0 THROUGH R8: THE VRF READ IS A REGISTERED ONE-CYCLE READ, AND THE
       OUTPUT FLOP IS INSTANTIATED IN `VecRegFile`, ONE PER READ PORT. A request
       presented in cycle N returns its data on `read_data` in cycle N+1, once, for
       every read port without exception. The bank's combinational array access sits
       INSIDE that envelope, and `VecRegFileBank`'s "0 cycles, may not be pipelined"
       is BANK-INTERNAL ONLY: it describes the array access, it stops at the bank
       boundary, and IT MUST NOT BE PROPAGATED OUTWARD to any consumer, seam note or
       sibling spec. Same-PRN write forwarding is unconditional and merges on the
       response, which is what makes `spec-vrf.f8`'s "single cycle reads" and
       `spec-cii.f23`'s "the registered, one-cycle VRF read" the SAME statement
       rather than two.

  WHY THIS WAY. Five consumers already assume registered-at-t+1 (the LCB's
  `stale_resp`, `VecDgen`'s `vrf_r3`, `VecMaskStream`, `VecIdxGen`,
  `VecGroupCopy`); one assumed combinational-plus-its-own-register
  (`VecCiiOperandServer`). Putting the flop in the file makes ONE timing contract
  for eleven or twelve ports instead of a per-client convention, and it keeps the
  9-read-port mux out of the consumer's own critical path.

  ===> THE CONSEQUENCE, AND IT IS MANDATORY (being applied in parallel):
       `VecCiiOperandServer` MUST DELETE ITS vLen-WIDE PAYLOAD REGISTER. If both
       it and `VecRegFile` register, observable latency becomes 2,
       `srcReadLatency` still says 1, and every Src-Data beat answers the request
       one beat late — the positional channel is then offset FOREVER, for every
       surviving instruction, with no error anywhere. What `opnd` keeps is a
       one-deep CONTROL register per lane — the `Valid` bit, the killed/scalar
       select and the `xLen` scalar payload — so that the beat arriving from the
       VRF flop at t+1 can be muxed against a scalar or don't-care beat in the same
       cycle it appears. That is the ordering FIFO its own file already describes,
       minus the 256-bit payload. Under no ruling may both sides register. If a
       future edit genuinely needs two cycles, `srcReadLatency` goes to 2 and the
       per-lane ordering depth follows it IN THE SAME EDIT.

  //@req-spec-vrf.i4
  THE PORT PARTITION IS CANONICAL AND NOTHING IN THIS CONTAINER ADDS A PORT.
  R0 load index, R1 load mask, R2 `stale_pvdest`, R3 store data, R4 store mask AND
  store index (one port, two readers, muxed inside `vlsu`), R5-R8 the four CII
  source lanes, W0/W1 load and LCB, W2 the CII writeback at physical index
  `lsuWidth` — A17, never a literal 2, because the write array is compacted in
  canonical W0,W1,W2 order and W1 is ABSENT rather than tied off at `lsuWidth = 1`,
  so a hard-coded 2 drives nothing on the one write path the CII has. The
  connections made here are exactly that list and no other. R2 is load-bearing:
  `stale_pvdest` is the ONLY source of undisturbed lanes — the `vta = 0` tail, the
  `vma = 0` masked-off elements and any `vstart > 0` prefix all come from it and
  from nothing else. `pvs3` and `stale_pvdest` are separate fields naming separate
  groups; they coincide for RMW arithmetic and DIVERGE for masked non-RMW ops,
  `vslideup` and `vcompress`, so this container never substitutes one for the other
  and never merges them.

  The R2/W0 second request slot (VecGroupCopy versus the LCB) is RESOLVED INSIDE
  `vlsu`, and A16 is closed in `VecGroupCopy`'s favour on requirement allocation:
  `spec-lsu.m13`/`m14` — the strict-priority mux, and "an active load drain always
  wins" — are allocated to `VecGroupCopy`, while `VecRegFile` holds only `m6`/`m7`.
  So the LCB's R2 request, its returning R2 data and its W0 write pass through
  `gcopy` COMBINATIONALLY, with no grant, no `ready` and no nack in either
  direction; the LCB drives unconditionally and is NEVER TOLD IT LOST. Consequently
  `VecRegFile`'s `gcopy_r2`/`gcopy_w0` fields and their grant outputs are DELETED,
  that file now sees exactly ONE R2 reader and ONE W0 writer, and this container
  drives one R2 request and one W0 write from `vlsu` with no grant wire anywhere on
  this boundary. Two muxes in series would be worse than one in the wrong place: the
  outer one would re-qualify an already-resolved request and could drop a granted
  copy with nothing reporting it.

  //@req-spec-vrf.c11
  MASKING SEMANTICS ARE HANDLED IN THE EXECUTION UNITS, and this container applies
  none. `cii` writes `wb_data` verbatim on W2 with the coprocessor's own byte mask;
  the coprocessor pulls `v0` on the `VM` slot and old-`vd` on `STALE_VD` and
  applies tail and mask policy internally. On the memory side the byte mask travels
  with the `nOP.v` or on the range entry, and the LCB overlays arriving elements
  onto the R2 pre-load. There is no mask register, no mask mux and no active-lane
  computation at this level — which is also why no mask needs a VRF port beyond R1
  and R4.

  `debug_vrf_read` is `vrf`'s, built from its ordinary read ports (no thirteenth
  port). `vec_trace_en` is ANDed into `vrf`'s trace gate and fanned to nothing
  else — every other module reaches the `vecTrace` plusarg through `VecTrace`.

  ---- PART 9. THE VL-RF RULING, and the commit read ----

  ===> CANONICAL AND BINDING: THE VL REGISTER FILE READ IS COMBINATIONAL — address
       presented, data valid in the SAME cycle, no valid, no ready, no enable, no
       output flop and no read-during-write bypass. It DIFFERS FROM THE VRF RULING
       DELIBERATELY, and the asymmetry is a SIZING FACT rather than an inconsistency
       for someone to tidy: the VL RF is 64 entries of 9 bits with three execute
       readers and one commit reader; the VRF is 96 entries of 256 bits with nine
       readers, banked four ways, where the port count is the dominant area term.
       64x9b with three readers is not 96x256b with nine, and the two files may
       therefore answer differently without either being wrong. A registered VL read
       would wrap a flop and a valid protocol around a 6-bit decode and a 9-bit mux
       to save nothing, and it would push VL onto the issue stage's critical path
       exactly where it must not be. `VlRegFile` states the same ruling in its own
       ports section; the two texts must stay identical.

  Consequences, both mandatory: `VecCiiIssue`'s reading (combinational on the
  presented address, registering the value itself into its emit stage) is CORRECT AS
  WRITTEN and stands. `VecScalarOperandRead`'s reading (address registered, data
  next cycle) is being CORRECTED in parallel — it may register the value on its own
  side if it wants it a cycle later, but it must not expect the file to. The
  no-bypass rule holds because every VL wakeup-to-execute distance is at least one
  cycle (part 6); if anyone ever adds a fast or speculative VL wakeup, `vlrf` needs
  a write forward and this paragraph is the place that must change.

  Write ports, statically partitioned, never arbitrated, `coreWidth + aluWidth + 1`
  of them: `W_ren` from `vl_rename.vl_rf_write` (one per rename lane); `W_alu`
  REPLICATED PER ALU EU (decision D8), lane `i` from `io.vset_resp(i)` with
  `addr = vset_resp(i).bits.uop.pvl` and `data =
  vset_resp(i).bits.data(vecVLSz-1,0)` — the new VL is the same value that ALU
  writes to its integer destination — enabled by `is_vl_producer` and never by
  `dst_rtype`; and `W_lsu` from `vlsu.vl_wb`. No mux, no arbiter and no ordering
  between `W_alu` lanes: two ALU EUs retiring vsets in one cycle write two distinct
  entries, because each vset renamed its own `pvl`. Read ports: three `R_exe`, one
  each to `vlsu`'s two operand-read instances and one to `cii`.

  THE COMMIT READ. `R_commit`'s ADDRESS is selected HERE, from `io.commit_valids`
  and `io.commit_uops`: the YOUNGEST committing uop with `is_vl_producer` set
  drives it. That is the same information the ROB would select from, so it needs no
  new input, and it removes an address member from the seam; only the DATA leaves,
  on the required new `commit_vl` member, whose `valid` is "some lane committed a
  VL producer". BoomCore composes `csr.io.vector.set_vconfig` from it and from the
  commit uop's `vconfig`. ONE port, not `coreWidth`: when several VL producers
  retire together only the youngest becomes architectural. `io.csr_vector` is
  consumed here for its OUTPUT-direction fields only (`vconfig`, `vstart`, `vxrm`);
  every input-direction field of rocket's bundle is driven by BoomCore/Rob, which
  is why `csr_vs_dirty` is a separate bwd bit at all.

  ---- PART 10. `pvtmp`: the rendezvous, and the chain that must not close ----

  //@req-spec-core.i6
  //@req-spec-decode.b4
  A SHARED INSTRUCTION REQUIRES MORE THAN ONE EXECUTION UNIT TO ACTIVATE. A
  segmented load or store needs both the vector LSU and the coprocessor: the LSU
  moves memory, the coprocessor transposes between segment layout and register
  layout. It stays ONE uOP with `is_shared` set and ONE ROB entry, and it names
  TWO issue queues through `iq_type` (part 5).

  //@req-spec-core.i7
  //@req-spec-core.i8
  //@req-spec-rename.e11
  //@req-spec-rob.d10
  //@req-spec-rob.d11
  //@req-spec-issue.c9
  THE TWO HALVES RENDEZVOUS ENTIRELY THROUGH THE `pvtmp` GROUP IN THE VRF: the
  PRODUCER half writes it as a destination, the CONSUMER half reads it as a source,
  and the consumer is woken by `pvtmp`'s GROUP-DONE on the vector wakeup network
  like any other vector operand. For a segmented LOAD the producer is the LSU and
  the consumer is the coprocessor; for a segmented STORE it is the other way round.
  There is no side channel, no handoff FIFO, no cross-unit valid and no port
  between `vlsu` and `cii` in either direction — check the instance map: they share
  only the VRF and the wakeup network. That is the whole point of routing the
  handoff through a renamed group: the rendezvous inherits speculation recovery,
  wakeup and completion from mechanisms that already exist.

  Two things this container must get right about `pvtmp`, both of which have bitten
  before:
  - The `IQ_V_ALU` slot's third-source matcher must select `Mux(is_shared &&
    uses_ldq, pvtmp, pvs3)` — DIRECTION-QUALIFIED. A bare `Mux(is_shared, ...)`
    makes a segmented STORE's coprocessor half wait on the very group it is about
    to write: immediate self-deadlock, exercised only by segmented stores.
  - The per-member readiness side channel must carry `pvtmp`'s members too, or the
    same matcher waits on group-dones that already fired. See part 13.

  //@req-spec-issue.d1
  //@req-spec-issue.d14
  THE SEGMENTED-STORE CHAIN IS SIX STEPS AND THE TOPOLOGY RESPECTS ALL SIX:
  (1) the LSU half AGENs, from operands older than the store; (2) its first address
  translation drives `vec_clr_unsafe`, clearing the entry's single `rob_unsafe`
  bit; (3) the PNR advances past the entry; (4) the coprocessor half becomes
  PNR-eligible in `iq_v_alu` — which is why `pnrGate` is true there and why
  `rob_pnr_idx` AND `rob_head_idx` are both forwarded: BOOM's age comparison is
  the three-argument `IsOlder(a, b, head)` and without the head the gate INVERTS
  across a ROB wrap and lets a younger-than-PNR op reach the coprocessor;
  (5) the coprocessor transposes, writes `pvtmp` on W2 and emits its group-done;
  (6) the LSU half's DGEN reads `pvtmp` on R3 as store data. AGEN and DGEN of the
  same slot are independently grantable, in that order, separated by a long and
  variable delay.
  THERE IS NO CIRCULAR WAIT. Every operand step 1 waits on is OLDER than the
  store, and step 4 depends only on this store's own address translation — which
  step 2 has already delivered, because ONE half clearing `rob_unsafe` is
  sufficient and nothing waits on the coprocessor half. Requiring both halves safe
  is exactly what would close the cycle, and this container must never add a term
  that makes the PNR wait on the coprocessor half.

  //@req-spec-core.i4
  //@req-spec-core.i5
  An OP.v may execute out of order relative to program order but ALWAYS EXECUTES
  ATOMICALLY AND IN ELEMENT ORDER, with no partial result architecturally visible.
  Atomicity is enforced structurally at this level rather than by any check: the
  destination group is renamed WHOLE (a group of `v_emul` PRNs, all-or-nothing, at
  rename), completion is a SINGLE group-done for the whole group, and the ROB
  busy-clear is single-shot — so no dependent can observe a half-written group and
  a trap unwinds to `vstart = 0` and re-executes the whole instruction. Element
  order is the producers' obligation (the AGENs emit in element order, the drain
  cursor advances in order, the coprocessor is in-order internally); what this
  container owes is not to reorder anything on top of it, and it does not: no
  grant, completion or writeback path here reorders or buffers.

  ---- PART 11. The coprocessor and LSU arms ----

  `cii` gets: `iss` from `iq_v_alu.iss_uops(0)`; VRF R5-R8 and W2; VlRegFile
  `R_exe(2)`; INT read lane 4 and THE ONLY FP read lane of the scalar seams (the
  fifth INT lane was added by amendment; the FP lane is lane 0 and is the ONLY FP
  lane on this boundary, because decision D4 deleted the store-side reader — a
  past-PNR CII op's scalar producer has usually RETIRED, so the value exists only
  in the register file and a bypass-only capture would read something unrelated
  with no error); `int_wb_snoop`; `csr_vstart`
  (8 bits, zero-extended to the packet's 9 by `iss` — an index needs one bit fewer
  than a count), `csr_vxrm` from `io.csr_vector`, `csr_frm` from `io.csr_frm`;
  `rob_flush`, `rob_flush_kill`, and `brupdate.b2.mispredict` as a single
  assertion-only bit. It returns `group_done`, `clr_rob`, `rob_flags`, `int_wb`,
  `fp_wb` and `fu_types`.

  `vlsu` gets: the dispatch bundle and `dis_fire`, `iss_ld`/`iss_st`, the full
  recovery set, VRF R0-R4 and W0 (+W1 at `lsuWidth = 2`), VlRegFile `R_exe(0..1)`,
  INT read lanes 0-3 and NO FP read lane AT ALL (decision D4: no RVV store form
  takes an FP scalar operand, so the store-side FP reader is deleted rather than
  tied off), `int_wb_snoop`, and `lsu_vec` straight
  through to BOOM's Unified LSU. It returns `dis_ok`, the two group-done lanes,
  `vl_wb`, `vec_clr_unsafe`, `vec_xcpt` and `lsu_fencei_rdy_vec`.

  ===> `fu_types` TOWARD `iq_v_load` AND `iq_v_store` IS A COMPILE-TIME CONSTANT:
       FC_AGEN and FC_DGEN advertised every cycle, unconditionally. `VecIssueUnit`'s
       port note says "the vector LSU's units drive theirs the same way" as
       `VecCiiIssue` does; that is WRONG and is settled here, because `VecLsu`
       correctly exports nothing that could drive it. No module in the vector LSU
       may export a `busy` reaching an issue unit (plan ground rule 6) — gate H4
       greps the subtree for exactly that — and qualifying the FC_AGEN grant is the
       SAME thing under another name, which is why it was rejected as a fix for the
       mid-walk hazard. Issue eligibility for a vector memory OP.v is "a
       reservation exists", decided at dispatch by `resv` through `dis_ok`, and
       nowhere else. `iq_v_alu.fu_types(0)` is the ONE dynamic advertisement in the
       whole vector design: `cii.fu_types`, which is CII Issue credit AND free-tag
       availability, both computed from NEXT state, and it is a functional-unit
       advertisement rather than a busy — the same mechanism baseline uses for a
       busy divider.

  `flush_pipeline` for all three queues is `io.rob_flush_kill`, which IS
  `RegNext(rob.io.flush.valid)`. Do NOT build a second `RegNext` of `rob_flush`
  here: a divergent copy would kill slots a cycle away from the LSQ pointer
  rollback and from the CII kill window. `rob_flush` (unregistered) goes only where
  a child asked for it by that name — `vlsu` and `cii`.

  ---- PART 12. `usingRVV`, tracing and the assertion set ----

  This whole module, and therefore every vector module, is elaborated ONLY when
  `usingRVV` is true: in a vectors-off build there is no `VecPipeline` instance, no
  `VecPipelineIO` bundle, no vector wire in `core.scala` and no vector module in
  the filelist — ABSENT, not tied off. `usingRVV` is a Scala `Boolean` of
  `BoomCoreParams`, never a hardware `Bool` and never rocket's `usingVector`, which
  is a DIFFERENT gate: rocket's controls the architectural CSR cells, which Caracal
  does need (part 9), while `usingRVV` controls Caracal's own logic. There is
  exactly ONE gate in this file, at the instantiation site.

  There are NO unit tests in this project — validation is end-to-end VCS plus
  Whisper cosim only — so guarded trace is the primary debug surface. This
  container emits three `VecTrace` lines, all at seams no child can see: one per
  dispatched vector uop (`rob_idx` plus the queue set it was routed to), one per
  group-done lane (`rob_idx` plus member count), one per VL-network beat (the
  producing lane). Gated on the `vecTrace` plusarg and `!reset`, off by default,
  emitting only — no register or counter that functional logic reads.

  The assertions this container owns, each of which catches a defect class that
  already occurred once: routing (part 5); `numVecWbPorts` versus the connected
  network length (part 6); group-done implies the ROB clear in the same cycle
  (part 7); at most one VRF write per port per cycle, and no two connected clients
  on one port number (part 8); `commit_vl` fires only when a VL producer commits
  (part 9); no two valid `W_alu` lanes carrying the same `pvl` in one cycle, which
  is the D8 replication's only correctness precondition (part 9); and, for the
  mid-walk hazard, that no second `iss_ld`/`iss_st` grant reaches a direction whose
  pending table says it cannot accept one — the assertion is the visible form of a
  bound that `vlsu`'s per-queue-entry pending table makes structural.
  Two ELABORATION-time checks belong beside them, because what they catch is a
  silent hang rather than a width error: the per-member side channel routed in part 5
  must carry as many groups as the slots' `VecGroupReady` instances consume (five on
  `iq_v_load`/`iq_v_alu`, four on `iq_v_store` — D6), and the VL wakeup network's
  lane count must equal `aluWidth + 1` (D8).

  ---- PART 13. Where the homeless bundles live ----

  Four bundles cross this container's boundary and no written file owns them
  cleanly. Settled, so that Phase R has one answer to apply:

  - `VecScalarOperands` -> `VecBundles`. It crosses four nodes (operand read, both
    agens, the store data path); a bundle declared inside its producer is readable
    from one side only.
  - `VecRobFlags` -> `VecBundles`, as `{rob_idx, fflags: UInt(FLAGS_SZ.W), vxsat:
    Bool}`. THIS IS A34, AND IT IS ASSIGNED HERE: the name appears in the
    `vec_pipeline_io` entry (`VecRobFlags * numVecClrPorts`) and in this container's
    completion group, but NO node declared it — `VecBundles`, already authored, does
    not. It goes to `VecBundles` for exactly the reason `VecScalarOperands` and
    `VecMemberRdy` do, and by the same test: it crosses a boundary both sides must
    review (this container emits it, `Rob` consumes it and applies the flags at
    COMMIT, not at writeback), and `VecBundles` already declares the bundle it is a
    member of. A declaration inside a producer would be readable from one side only.
  - `VecCiiTagEntry` -> `VecBundles` (named there already), single declaration, no
    copy. `VLSAccessDesc`, `LcbAllocReq`/`LcbBeat`/`LcbTrim`, `LsuResourceClaim`,
    `VecLsuCoreIO`, `VecVrfReadReq`/`VecVrfWrite`, `VecBusyResp` and
    `VecMapReq`/`VecMapResp`/`VecRemapReq` STAY LOCAL to their producers, mirroring
    baseline BOOM (which declares `class BusyResp` beside its user and the register
    file's port IO beside the file). The test is whether the bundle crosses a
    boundary that both sides must review; a bundle with one producer and one
    consumer in the same subtree does not.
  - `VecSlotMemberRdy` and `VecMemberRdy` ARE ONE BUNDLE WITH TWO NAMES, and that
    is a defect, not a synonym. Canonical: ONE declaration in `VecBundles`, named
    `VecMemberRdy`, with FIVE PER-MEMBER GROUPS plus the mask bit — `vs1_rdy`,
    `vs2_rdy`, `vs3_rdy`, `vtmp_rdy` and `vold_rdy`, each `Vec(maxMembers, Bool)`,
    plus `vm_rdy: Bool` — and `VecIssueUnit`/`VecIssueSlot` bind to it. `vold_rdy`
    is the FIFTH group of decision D6, feeding the `rdy_vold` matcher on `IQ_V_LOAD`
    and `IQ_V_ALU` slots (part 5); the mask stays a single `Bool` because `pvm` names
    one register, not a group, so five groups and six fields are the same statement.
    `VecRenameSpace`/`VecBusyTable` must WIDEN their export to it — they currently
    say four groups — and this container routes all five into `dis_member_rdy`.
    The multi-group shape (VecRenameSpace's) WINS over
    the four-field one (VecIssueSlot's): the slot's third-source matcher tracks
    `pvtmp` whenever the direction-qualified select of part 10 fires, so it needs
    `pvtmp`'s PER-MEMBER readiness and the four-field bundle cannot supply it —
    the matcher would fall back to the aggregate for exactly the case a group's
    members come from different producers, which is the permanent hang this channel
    exists to prevent. On the OUTPUT side the slot writes its next-state into the
    FIELD IT SELECTED (`vtmp_rdy` when the select took `pvtmp`, `vs3_rdy`
    otherwise), so the receiving slot's identical select reads it back after a
    collapse move.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
THIS CONTAINER ADDS ZERO PIPELINE STAGES AND ZERO CYCLES OF LATENCY TO EVERY PATH
THAT CROSSES IT. That is a constraint, not an aspiration, and it is what makes the
timing claims of eleven child specs composable:

- Decode: `dec_uops_in` to `dec_uops_out` is combinational through `vdec`, and
  nothing here exports a signal that could stall decode.
- Rename and dispatch: `ren2_uops` to the issue-slot write is combinational through
  TWO chained rename spaces and the routing fan-out, in ONE cycle. This is the
  container's longest combinational path and the reason the chain order is
  `vec_rename` then `vl_rename` (the VL space is one architectural register, so its
  map lookup is a 1-of-1 select and adds almost nothing after the 32-entry one). If
  it ever fails timing the fix is NOT a pipeline register here — that breaks
  single-cycle rename and dispatch-group atomicity with it — but inside a space.
- Grant to consumer: `iss_uops(0)` to `vlsu`/`cii` is combinational; single issue
  stage, and no second stage may appear here.
- Wakeup broadcast: group-done and VL beats reach slots and both busy tables
  UNREGISTERED in the same cycle. A register on one of those paths and not the
  other is a lost single-shot wakeup, i.e. a hang.
- Register file: VRF reads latency exactly 1 with the flop in `VecRegFile`, one per
  read port and nowhere else; VL RF reads combinational, same cycle. Obligations,
  not targets, and deliberately different from each other — see parts 8 and 9.
- Completion: group-done, the ROB clear and the `W2`/`W0` write that produced it
  are the SAME cycle. Skewed ahead, a dependent reads stale data; skewed behind, it
  costs a cycle on every dependency.

Throughput targets belong to the children (one grant per queue per cycle; four
Src-Data beats per cycle; the LSU's element rate). The container's own budget is
the fan-out cost it must not exceed: `VecGroupDone` is matched per member against
`numVecWbPorts` ports in every slot of three queues, so a field added to that
bundle is paid `slots * ports * members` times. Keep it to the member PRNs, the
count and the ownership fields.

The one budget this container knowingly SPENDS is D6's: routing a fifth group into
`dis_member_rdy` adds a fifth `VecGroupReady` to every `IQ_V_LOAD` and `IQ_V_ALU`
slot — 32 instances, ~6k comparators — in the wakeup-to-grant stage that is already
this design's #1 timing risk. It was accepted against a silent hang and silent
`stale_pvdest` corruption, and the mitigation if the path fails is inside
`VecGroupReady` (the shared one-hot decode), never a register on the wakeup path,
which part 6 forbids.
<|end_perf|>

<|begin_dependencies|>
Instantiates, per hierarchy.yaml: `VecDecode` (`vdec`); `VecRenameSpace`
(`vec_rename`, `vl_rename` — ONE definition, two instances, as BOOM already does
for INT and FP; `addvector` hand-wrote two spaces and `VlRename` reached 405 lines
reimplementing map, free, busy, wakeup and commit for a space with one
architectural register); `VecIssueUnit` (`iq_v_load`, `iq_v_store`, `iq_v_alu` —
ONE definition, three instances, differing only in `iqType` and `pnrGate`);
`VecRegFile` (`vrf`); `VlRegFile` (`vlrf`); `VecLsu` (`vlsu`); `VecCiiHost`
(`cii`). Nothing else, and no module is re-implemented inline.

Declaration dependencies: `MicroOp` (every bundle on this boundary carries it, and
part 2 places two obligations on its delta), `VecBundles` (`VecPipelineIO`,
`VecGroupDone`, `VecMemberRdy`, `VecRobFlags`, `VecException`), `VectorParams`
(every size), `VecTrace` (the three trace lines of part 12). Binds to
`freechips.rocketchip.rocket.{VConfig, VType, CSRVectorIO}` for the CSR seam,
because rocket's `CSRFile` owns architectural vector CSR state and Caracal owns
only the speculative vtype mirror and the VL register file.

Its parent is `BoomCore` (`core.scala`), which instantiates it once, under
`usingRVV`, and fans out `int_wakeups`/`fp_wakeups` as taps of the EXISTING
wakeup buses (no new network on the scalar side).

---- AMENDMENTS THIS FILE REQUIRES OF OTHERS (for Phase R step R2) ----

1. `vec_pipeline_io` + `VecPipelineIO`: `rob_empty`, `commit_vl`, the widened
   `vl_wakeup` (`numVlWakeupPorts` = `aluWidth + 1`), the `Vec(aluWidth)`
   `vset_resp` and `dis_uops_out` are APPLIED in the interface entry — no action
   left. STILL REQUIRED: `dis_vec_ready` (bwd, `coreWidth * 3`) and
   `dis_vec_valids` (fwd, `coreWidth * 3`), the per-lane dispatch handshake D2
   obliges (part 5). Also still required: declare `IntWakeupBus`, `FpWakeupBus`,
   `IntWbSnoop` and `VecRobFlags` in `VecBundles`, with the shapes given in the
   ports section and part 13 (`VecRobFlags` is A34).
2. `VecCiiOperandServer`: DELETE the `vLen`-wide payload register (part 8). Keep
   the one-deep per-lane control register. In progress.
3. `VecScalarOperandRead`: the VL-RF read is COMBINATIONAL (part 9); correct the
   address-registered assumption. `VecCiiIssue` needs no change. In progress.
4. `VecLsu`: export `group_done : Vec(2, Valid(VecGroupDone))` (part 6), and derive
   its two `vec_clr_bsy` lanes from it rather than emitting them independently.
5. `VecRegFile`: delete `gcopy_r2`/`gcopy_w0` and their grants (A16, part 8) — DONE
   in that file; `VecGroupCopy` holds `spec-lsu.m13`/`m14`.
6. `VecIssueUnit`: correct the port note claiming the vector LSU drives its
   `fu_types` — those two lanes are constants (part 11) — and WIDEN
   `dis_member_rdy` to the five-group bundle (D6, part 5).
7. `VecIssueSlot`: bind to the canonical `VecMemberRdy` (five per-member groups plus
   `vm_rdy`), add the `pvtmp` member vector to its third-source select (part 13,
   part 10), and instantiate the FIFTH `VecGroupReady` (`rdy_vold`) on the
   `IQ_V_LOAD`/`IQ_V_ALU` configurations — its own file still records this as out of
   scope, which D6 supersedes.
8. `VecRenameSpace`/`VecBusyTable`: export the FIFTH per-member group (`vold_rdy`,
   the `stale_pvdest` mapping) on `member_rdy`; both still describe four.
9. `VecRenameSpace`/`VecIssueUnit`/`VecGroupReady`/`VecIssueSlot`: take
   `numVecWbPorts` from this container's constructor argument; no child re-defaults
   it. `VectorParams` now DECLARES it, and `numVecClrPorts` and `numVlWakeupPorts`
   with it, so the chain is `VectorParams` -> here -> child and nowhere else.
10. `BoomCore`: its dispatch section still describes `BasicDispatcher` and tying
   `dispatcher.io.dis_uops(i)(w).ready := true.B` for the three `IQ_V_*` entries —
   the alternative D2 REJECTED. It must instantiate `CompactingDispatcher` for
   vector configs, wire the three vector queues natively through the two new seam
   members, and feed the dispatcher the CHAINED-RENAME bundle (part 5's trap).
   `WithNSmallBoomsVector` is also dropped from the config matrix (D3).
<|end_dependencies|>

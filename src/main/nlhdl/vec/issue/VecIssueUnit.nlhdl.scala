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
  VecIssueUnit — one vector issue queue: BOOM v4's age-ordered COLLAPSING issue
  queue and its priority-encoder select, reused unchanged, holding VecIssueSlot
  entries instead of scalar IssueSlot entries.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/issue/VecIssueUnit.scala,
  package boom.v4.vec.generated.issue.
  depends_on MicroOp, VecBundles, VectorParams, VecTrace.
  instantiates VecIssueSlot as `slots`, count `numEntries`.

  ONE definition, THREE instances, all created by VecPipeline:
    `iq_v_load`  iqType = IQ_V_LOAD  (4)  pnrGate = false  numEntries = 16
    `iq_v_store` iqType = IQ_V_STORE (5)  pnrGate = false  numEntries = 16
    `iq_v_alu`   iqType = IQ_V_ALU   (6)  pnrGate = true   numEntries = 16
  All three are age-ordered collapsing and grant the OLDEST READY entry. The only
  difference between them is `pnrGate` and which optional slot ports exist.

  A NEW module, not a delta on `v4/exu/issue-units/issue-unit-age-ordered.scala`,
  but it deliberately COPIES that file's structure and every name a reader already
  knows — `dis_uops`, `slots`/`issue_slots`, `vacants`, `shamts_oh`,
  `SaturatingCounterOH`, `will_be_valid`, `uops`, `is_available`, `requests`,
  `port_issued`, `iss_uops` — so the two diff line for line. Baseline
  `IssueUnitCollapsing` is untouched and the scalar queues keep using it. Exactly
  three things change: the slot type, the signal the encoder selects on, and the
  added per-member readiness side channel.

  ===> THE PRIORITY ENCODER SELECTS ON `eligible`, NOT ON `request`. VecIssueSlot
       computes past-PNR eligibility ITSELF, per entry, and exports BOTH bits.
       Selecting on `request` here silently deletes the past-PNR gate on
       `iq_v_alu` and hands speculative work to the coprocessor. Baseline's own SNI
       block is NOT reproduced, and its extra `| (rob_idx === rob_pnr_idx)` term
       must not be copied either: `rob_pnr_idx` names the OLDEST UNSAFE entry
       (`rob.scala:531`), so admitting equality admits the entry whose speculation
       is NOT YET RESOLVED. The past-PNR test is a STRICT
       `IsOlder(rob_idx, rob_pnr_idx, rob_head_idx)` with nothing OR-ed onto it.

  ===> `out_member_rdy` MUST BE ROUTED TO `in_member_rdy` ON THE SAME COLLAPSE
       MOVE THAT ROUTES `out_uop` TO `in_uop`. Folding this per-member readiness
       side channel into `out_uop.pvs*_busy` collapses partial readiness to the
       aggregate and HANGS: a group's members can come from different producers, so
       it can be not-ready in aggregate while members 0..2 are already done, and
       the destination slot would then wait for group-dones that already fired.

  ===> `IQ_V_ALU` IS DELIBERATELY NOT A HEAD-ONLY FIFO. The VPU being in-order
       requires one-at-a-time EXECUTION, not program-order ISSUE; ordering is
       enforced exclusively by the ROB. And NO `busy` FROM `VecLsu` MAY BE
       CONSULTED (the vector-LSU invariant): this unit has no port to it at all.

  Governing spec anchors: issue.rst `issue-sched-stage`, `shared-store-chain`,
  `cii-shared-sched`, `vec-queue-reservation`; cii.rst `cii-issue`,
  `cii-segmented`, `cii-flush`; execution.rst `vector-execution`;
  overview.rst `caracal-pipeline`; glossary.rst `glossary-terms`;
  midcore.rst `midcore-segmented-store`.

<|begin_module|>

  <|begin_parameters|>
  The unit takes BOOM's OWN `IssueParams` case class
  (`v4/exu/issue-units/issue-unit.scala`) rather than a vector-specific twin, so
  the vector queues are configured by the same record the scalar queues are and a
  reader has one shape to learn. Fields bound per instance:

  `iqType` — `IQ_V_LOAD` (4), `IQ_V_STORE` (5) or `IQ_V_ALU` (6) from the
  ScalarOpConstants delta. Used only to derive the ELABORATION-TIME Scala
  Booleans `isLoadQueue`/`isStoreQueue`/`isAluQueue`, which are forwarded to the
  slots; no hardware compares `iqType`.

  `numEntries` — slots per queue, default `vecIssueEntries` = 16.
  `dispatchWidth` — default `coreWidth` (3). `issueWidth` — grants per cycle,
  default `vecIssueGrantWidth` = 1; the widest tier raises it to 2.
  `VecCiiIssue` requires the ALU queue's grant width to be exactly 1 and fails
  elaboration otherwise: the CII Issue channel carries one beat per cycle and
  has no ready line, so a second grant would be dropped, not stalled. If a tier
  raises vecIssueGrantWidth it must raise it for the load/store queues only.

  `numSlowEntries` — bound to 0, so every slot is a fast slot and the queue
  collapses at full dispatch throughput. Slow slots are baseline's critical-path
  mitigation for its much deeper scalar queues, and at 16 entries the vector
  queue's critical path is the group-ready AND-reduce inside the slots, not the
  shift-amount chain — so it stays a parameter (the right knob if that ever
  changes) but is zero here. `useFullIssueSel` true, `useMatrixIssue` FALSE: the
  age-ordered COLLAPSING queue is what is named, so the matrix variant is not an
  alternative implementation of this node.
  Require `numEntries - numSlowEntries >= dispatchWidth`, as baseline does.

  `pnrGate` — Boolean, default false, true only for `iq_v_alu`. Adds the
  `rob_pnr_idx` and `rob_head_idx` inputs and forwards them, plus `pnrGate`
  itself, to every slot. It adds NOTHING to this module's select logic.

  `numIntWakeupPorts` — width of the INTEGER wakeup network, taken from the
  core's existing wakeup-port count and not re-declared for vectors. All three
  queues listen to it.
  `numFpWakeupPorts` — default 0, non-zero ONLY when `isAluQueue`. Vector memory
  addressing uses only GPRs, so the load and store queues elaborate no FP
  comparators.

  `numVecWbPorts` — Int, BOUND TO `VectorParams.numVecWbPorts` (3), the width of
  the VECTOR (group-done) wakeup network: the Load Coalescing Buffer, the CII
  writeback completion, and VecGroupCopy's complete-without-execute path.

  ===> THE SINGLE BINDING SITE IS NOW `VectorParams`, WHICH DECLARES IT. This
  module previously claimed the binding site because the corpus named the number
  (issue.g33, rename.g25) and VectorParams did not declare it, leaving
  VecIssueSlot and VecGroupReady each defaulting it to 3 independently.
  VectorParams now declares `numVecWbPorts` (3), `numVecClrPorts` (3, the ROB
  busy-clear lane count) and `numVlWakeupPorts` (`aluWidth + 1`). BIND TO THOSE
  FIELDS; DO NOT RE-DEFAULT LOCALLY. VecPipeline still passes the actual producer
  count to this constructor and this module still forwards it verbatim to every
  slot, which forwards it verbatim to its VecGroupReady instances — but the
  literal 3 exists in exactly one file now. If the numbers ever disagree, a
  matcher examines fewer ports than the network drives, misses a single-shot
  group-done, and the consumer hangs forever. Keep
  `require(numVecWbPorts == io.vec_group_done.length)` as the check.

  `usingRVV` is a Scala `Boolean` of `BoomCoreParams`, not a hardware `Bool`.
  All three instances exist only in a `usingRVV` build; in a vectors-off build
  they are ABSENT rather than tied off, so the emitted RTL stays bit-identical to
  pre-Caracal BOOM v4. The gate is `usingRVV`, never rocket's `usingVector`.

  Every width comes from `MicroOp`'s field types, from `HasBoomCoreParameters`
  (`robAddrSz`, `FC_SZ`, `aluWidth`) or from VectorParams through
  `HasVectorParams`. No literal.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel default and hierarchy.yaml's `defaults`: one
  `core_clk` domain, posedge-triggered, ACTIVE-HIGH SYNCHRONOUS `core_reset`.
  Both implicit; no explicit clock or reset port. The only reset-sensitive state
  in this module is baseline's registered `is_available` vector and the
  `RegNext`ed dispatch readies; all entry state lives in the slots.

  The port list is baseline `IssueUnit`'s, plus the vector wakeup networks and
  the side channel, minus the three things a vector queue must not have.

  `dis_uops` — `Vec(dispatchWidth, Flipped(Decoupled(new MicroOp())))`, baseline's
  dispatch interface with its `ready` back-pressure unchanged.

  `dis_member_rdy` — `Input(Vec(dispatchWidth, new VecMemberRdy))`, the per-member
  readiness side channel at dispatch, produced by VecRenameSpace from the
  per-member vector Busy-Table read it already performs, and qualified by the same
  `dis_uops(w).valid`. It travels BESIDE the uop, never inside it.

  ===> IT IS WIDER THAN IT WAS, BY ONE GROUP (decision D6). `VecMemberRdy` now
  carries `vs1_rdy`, `vs2_rdy`, `vs3_rdy`, `vtmp_rdy` and `vold_rdy`, each
  `Vec(maxMembers, Bool)`, plus `vm_rdy: Bool`. `vold_rdy` is the `stale_pvdest`
  group's per-member readiness, feeding the slot's FIFTH VecGroupReady instance
  (`rdy_vold`) in `iq_v_load` and `iq_v_alu`. VecBusyTable is amended in parallel
  to export that group's per-member read; this module only routes it.
  `iq_v_store` receives the field and leaves it unread rather than getting a
  narrower bundle — one declaration, no per-queue variant, so the collapse move
  stays ONE mux over ONE wire (part 3).
  `VecSlotMemberRdy` was this bundle's second name, declared in
  VecIssueSlot.nlhdl.scala. It is a DEFECT, not a synonym: bind to the single
  `VecMemberRdy` in VecBundles and do not copy it.

  `iss_uops` — `Output(Vec(issueWidth, Valid(new MicroOp())))`, the grants. On
  `iq_v_alu` lane 0 is `VecCiiIssue.io.iss`, which has no `ready` and never
  refuses a grant; on `iq_v_load`/`iq_v_store` the lanes reach the vector LSU's
  AGEN/DGEN paths.

  `int_wakeup_ports` — `Flipped(Vec(numIntWakeupPorts, Valid(new Wakeup)))`, the
  EXISTING BOOM integer wakeup network, broadcast to every slot and also read by
  the dispatch-cycle pre-correction below.
  `fp_wakeup_ports` — the same `Wakeup` bundle on the FP network, elaborated only
  when `isAluQueue`.
  `vl_wakeup` — `Flipped(Vec(numVlWakeupPorts, Valid(UInt(vlPregSz.W))))`, the
  dedicated VL network: `VectorParams.numVlWakeupPorts` = `aluWidth + 1` lanes, each
  carrying a bare VL physical register number. All three queues connect.
  It was ONE lane before decision D8, which REPLICATES the vset writeback per
  integer ALU rather than arbitrating it (a single-shot VL wakeup lost to
  arbitration is a permanent hang), plus one lane for `vleff`'s trimmed VL. Both
  this module's dispatch pre-correction (part 2) and every slot's comparator set
  must scan all lanes; sizing either from a literal 1 drops a VL wakeup.
  `vec_group_done` — `Flipped(Vec(numVecWbPorts, Valid(new VecGroupDone)))` from
  VecBundles, broadcast unregistered to every slot. This module never inspects a
  group-done itself.

  `child_rebusys` — `Input(UInt(aluWidth.W))`, baseline's speculative-child
  retraction bus, broadcast to every slot and used by the dispatch pre-correction.

  `fu_types` — `Input(Vec(issueWidth, Vec(FC_SZ, Bool())))`, baseline's per-port
  functional-unit advertisement, consumed by the select exactly as baseline
  consumes it. This is the design's ONLY back-pressure into issue: `VecCiiIssue`
  drives `iq_v_alu`'s lane to zero when it has no CII Issue credit, and the
  vector LSU's units drive theirs the same way.
  SEAM NOTE (A51) — A TYPE MISMATCH THAT IS SETTLED, NOT OUTSTANDING. This port
  is baseline's `Vec(FC_SZ, Bool())`. `VecCiiIssue` spells its side as
  `Output(UInt(FC_SZ.W))`. Same information, different Chisel type, and BASELINE'S
  TYPE WINS HERE so this module's diff against `IssueUnitCollapsing` stays clean.
  ===> THE CONVERSION POINT IS `VecPipeline`, WHICH CONNECTS THE TWO WITH
  `.asBools` — one named place, recorded so nobody "fixes" it by changing either
  declaration. Neither side changes type; the container converts.
  Separately flagged by VecPipeline and NOT settled here: the sentence above
  claiming the vector LSU's units drive their `fu_types` lanes the same way is
  wrong — VecPipeline reports those two lanes are CONSTANTS. That correction
  belongs to whoever owns the LSU seam; it does not change this port's type.

  `brupdate` — `Input(new BrUpdateInfo())`. `flush_pipeline` — `Input(Bool())`,
  driven from BoomCore's `RegNext(rob.io.flush.valid)` exactly as the scalar
  queues are, and wired to each slot's `kill`. `squash_grant` — `Input(Bool())`.

  `rob_pnr_idx` and `rob_head_idx` — `Input(UInt(robAddrSz.W))`, elaborated ONLY
  when `pnrGate`, forwarded unmodified to every slot and read nowhere in this
  module. Both are needed because BOOM's age comparison is the three-argument
  `IsOlder(a, b, head)`: the ROB is circular, and without the head the gate
  INVERTS across a wrap boundary and lets a younger-than-PNR op reach the
  coprocessor. `rob_head_idx` was added to `vec_pipeline_io` for exactly this.

  Deliberately ABSENT, and a reviewer should reject any of them:
  - `pred_wakeup_port` and every `ppred` term. A vector OP.v is never an SFB
    shadow; the dispatch path asserts `!ppred_busy` and ties it clear instead.
  - `tsc_reg`. Baseline carries it for a debug string; vector tracing goes through
    VecTrace, and an unused `xLen` input invites a lint waiver.
  - Any port to or from `VecLsu`, any `busy`/`fu_ready` input, and any
    element-queue RESERVATION port (see part 8 of the logic section).
  - Any `rob_flush` input taken combinationally from `rob.io.flush.valid` (part
    10), any second `iss_uops` stage, and any cross-queue grant or kill signal.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. The frame: baseline's collapsing queue, copied ----

  //@req-spec-core.e12
  //@req-spec-issue.e9
  //@req-spec-issue.e10
  The age-ordered collapsing queue and its priority-encoder select are REUSED,
  not reinvented. `vacants`, `shamts_oh` with `SaturatingCounterOH`, the
  `will_be_valid` array, the `uops` array, the `clear` derivation, the registered
  `is_available` dispatch-readiness calculation and the `port_issued`/`uop_issued`
  select loop are all taken verbatim from `IssueUnitCollapsing`, with the same
  names, so the vector unit diffs cleanly against the scalar one. `IQ_V_LOAD` and
  `IQ_V_STORE` are two of the six queues that reuse it unchanged; `IQ_V_ALU`
  reuses the identical structure and adds only the per-entry eligibility term the
  slot already computes (part 5). Caracal adds three queue INSTANCES to BOOM, not
  a new scheduling mechanism.

  `slots` is `numEntries` VecIssueSlot instances and `issue_slots` is
  `VecInit(slots.map(_.io))`, mirroring baseline. Every slot receives
  `int_wakeup_ports`, `fp_wakeup_ports` (when elaborated), `vl_wakeup`,
  `vec_group_done`, `child_rebusys`, `squash_grant`, `brupdate`,
  `kill := io.flush_pipeline` and, when `pnrGate`, `rob_pnr_idx`/`rob_head_idx`.

  ---- 2. The dispatch cycle: pre-correct the SCALAR half only ----

  Baseline pre-corrects the dispatching uop's busy bits against the wakeup ports
  before it enters a slot, because a slot reads its REGISTERED `prs*_busy`; that
  block is kept as-is for `prs1`/`prs2` — `prs1_matches`/`prs2_matches`,
  `prs1_wakeups`/`prs2_wakeups`, `prs1_rebusys`/`prs2_rebusys`, `bypassables`,
  `speculative_masks`, the `iw_p*_speculative_child`/`iw_p*_bypass_hint` updates
  and the `child_rebusys` term — sourced from the INT network, and from the FP
  network when `isAluQueue` and `lrs1_rtype === RT_FLT`. The VL comparators are
  added in the same shape as baseline's `pred_wakeup_port` handling: clear
  `dis_uops(w).pvl_busy` when ANY VL lane matches, i.e.
  `io.vl_wakeup.map(l => l.valid && l.bits === pvl_src).reduce(_ || _)` over all
  `numVlWakeupPorts` lanes — not lane 0 alone.
  `iw_issued`, `iw_issued_partial_agen`, `iw_issued_partial_dgen` and the three
  bypass hints are cleared, as baseline clears them.

  NO VECTOR PRE-CORRECTION HAPPENS HERE, and that asymmetry is deliberate.
  VecGroupReady applies the group-done match to the LOADED value in the load
  cycle (`(load ? in_member_rdy : member_rdy) || member_hit`), so a group-done
  arriving in the dispatch cycle is captured at the slot's port. Correcting it
  here as well would put the same correction in two places — this path AND the
  collapse path — and completion is single-shot, so the two disagreeing by one
  cycle is a lost wakeup and a permanent hang.

  //@req-spec-cii.d1
  Vector arithmetic reaches the coprocessor only through `iq_v_alu`, and the
  routing is checked at this boundary rather than trusted: assert that every
  accepted dispatch has `is_vec` set and has `iq_type(iqType)` set for THIS
  instance's `iqType`. `MicroOp.iq_type` is a `Vec(IQ_SZ, Bool())` bitmask, which
  is what lets a shared OP.v name two queues at once (part 8) with no new field.
  `iq_v_alu` is also the only instance whose `fu_types` lane is driven by
  `VecCiiIssue`, so a vector-arithmetic op dispatched elsewhere would additionally
  never find a matching functional unit.

  ===> BASELINE'S `iqType`-SPECIFIC DISPATCH FIXUPS ARE ON A REJECT LIST, AND THE
       LIST IS THE POINT (A49). Each entry is a block a generator told to "reuse
       baseline's dispatch path" would copy, and each is actively wrong here:

  (a) IQ_MEM's `when (uses_stq && lrs2_rtype === RT_FLT) { lrs2_rtype := RT_X;
      prs2_busy := false }` — baseline's FP-STORE fixup. ===> IN A VECTOR STORE
      `prs2` CARRIES THE STRIDE. It must be waited on and delivered intact.
      Copying this clears the busy bit of a live integer operand and clobbers its
      rtype, turning every strided store into a wrong-address store — a silent
      wrong-data bug, not a stall.
  (b) IQ_MEM's `dis_uops(w).prs3_busy := false` — the same class, one operand
      over. Vector store data is a VRF group read on port R3, never a scalar
      operand, so `prs3` has no vector meaning and force-clearing its busy bit
      teaches a reader that the third operand is always ready. Likewise IQ_FP's
      `prs1` rewrite.
  (c) IQ_UNQ's `prs2 := Cat(fp_rm, fp_typ)` and `pimm := mem_size` rewrites —
      they overwrite two fields the vector AGEN reads (`prs2` is the stride again,
      `pimm` is not repurposed here).
  What IS copied is baseline's `iqType != IQ_ALU` clause: assert
  `!(dis_uops(w).valid && ppred_busy)` and tie `ppred_busy := false.B`. That is
  how the slot's "no ppred term" claim is discharged without a port.
  The FOURTH member of this reject list is baseline's SNI block, which is not a
  dispatch fixup and is rejected in part 5 where the eligibility term lives.

  ---- 3. The collapse move, and the side channel that must ride with it ----

  Baseline builds `uops = issue_slots.map(_.out_uop) ++ dis_uops` and drives
  `issue_slots(i).in_uop.bits := uops(i+1)`, overridden inside the
  `when (shamts_oh(i+j) === (1 << (j-1)).U)` loop together with
  `in_uop.valid := will_be_valid(i+j)`. The member-readiness channel is carried
  the same way, from the parallel array
  `member_rdys = issue_slots.map(_.out_member_rdy) ++ io.dis_member_rdy` — same
  indexing, same array length, so index `i+j` names the same entry in both.

  ===> IMPLEMENT THIS AS ONE MUX OVER ONE COMBINED WIRE, not two muxes that
  happen to share a condition. Declare a local wire vector of a two-field
  bundle {uop, member_rdy}, fill it from the two arrays, and let the collapse
  `when` select that bundle once — then no future edit can add a term to one
  path and forget the other. Desynchronised, the migrated slot reloads its
  matchers with the WRONG slot's partial readiness and waits on group-dones that
  already fired: a silent permanent hang with no assertion anywhere.
  Routing partial readiness through `out_uop.pvs*_busy` is the same bug with a
  tidier spelling — the uop carries ONE aggregate bit per operand by MicroOp's
  design, so members 0..2 done with member 3 in flight is indistinguishable from
  nothing done.
  The bundle gained `vold_rdy` with D6, so the combined wire is `maxMembers` bits
  wider — `5*maxMembers + 1` = 41 bits at the defaults, against 25 for the
  original four-group shape. That widening is exactly why the one-mux rule is
  stated as a rule: a bundle gaining a field is the moment a two-mux
  implementation loses one.

  `issue_slots(i).clear := shamts_oh(i) =/= 0.U` and the `is_available` /
  `io.dis_uops(w).ready` calculation are baseline's, unchanged, including its
  `RegNext` and its `assert(!ready || (shamts_oh(w+numEntries) >> w) =/= 0.U)`.

  ===> SEAM, UPDATED BY DECISION D2: THIS QUEUE'S READINESS IS NATIVE PER LANE
  AND NO LONGER TRAVELS THROUGH THE SINGLE `dis_ready` BIT. Vector configs
  instantiate `CompactingDispatcher` (not `BasicDispatcher`), and the three vector
  queues are wired NATIVELY from `dispatcher.io.dis_uops(i)` — not via a
  `ready := true.B` hack with fullness re-routed through the seam. So this
  module's `io.dis_uops(w).ready` IS the back-pressure the dispatcher consumes,
  per lane, unchanged from baseline.
  
  WHY IT MATTERS, mechanically: `BasicDispatcher` computes
  `ren_readys = io.dis_uops.map(d => VecInit(d.map(_.ready)).asUInt).reduce(_&_)`
  — the ready is NOT masked by `iq_type`, so EVERY queue's ready ANDs into EVERY
  lane and a full `IQ_V_LOAD` would stall pure-scalar lanes carrying no vector uop
  at all. `CompactingDispatcher` already masks it:
  `rdy := ren zip uses_iq map { case (u,q) => u.ready || !q }` — "the queue is
  considered ready if the uop doesn't use it." That is why a full vector queue no
  longer stalls scalar dispatch.
  
  What DOES still ride the seam's single `dis_ready` bit is the WHOLE-BUNDLE
  allocation answer: VecFreeList's `alloc_ok` and VecQueueReservation's `dis_ok`,
  which are all-or-nothing by construction and must never fire a subset of lanes.
  Queue fullness is not one of them any more. Do not re-fold this module's
  per-lane ready into that bit.

  ---- 4. Select: the oldest READY entry, out of order among ready ops ----

  //@req-spec-issue.e11
  //@req-spec-issue.e12
  //@req-spec-cii.d2
  The select is baseline's priority encoder over the slots in age order, index 0
  oldest, granting the OLDEST ELIGIBLE entry and freely SKIPPING a not-ready older
  one — out-of-order issue among ready ops. All three queues do this:
  `iq_v_load`/`iq_v_store` because the vector LSU is out-of-order, and `iq_v_alu`
  because age-ordered collapsing with a per-entry past-PNR gate is exactly the
  specified policy. The collapse keeps the array age ordered, so "first in index
  order" IS "oldest", and the gate narrows which entries the encoder may pick
  without changing the encoder. `port_issued`/`uop_issued` and the `fu_code_match`
  against `io.fu_types(w)` are baseline's verbatim, so a queue with
  `issueWidth > 1` fills its lanes oldest-first;
  `assert(PopCount(issue_slots.map(_.grant)) <= issueWidth)`,
  `io.iss_uops := iss_uops` and `when (io.squash_grant) { lane valid := false }`
  are all kept as baseline has them.

  ---- 5. `eligible`, not `request` — the per-entry past-PNR gate ----

  //@req-spec-issue.e13
  //@req-spec-cii.d3
  //@req-spec-cii.d4
  The array the encoder scans is `eligibles = issue_slots.map(_.eligible)`, NOT
  `requests`. On a `pnrGate` slot the slot has already formed
  `request && IsOlder(slot_uop.rob_idx, rob_pnr_idx, rob_head_idx)` — its ROB
  entry compared against `rob.io.rob_pnr_idx`, per entry, on every slot and not
  only the head — so every op this unit hands the CII is individually
  non-speculative, RoCC-style. On a non-`pnrGate` slot `eligible` is `request`
  unchanged, so the load and store queues issue speculatively and ordering and
  replay stay the LSU's business. `requests` is still exported by the slots and is
  read here for TRACING and assertions only, never by the select; keeping both
  visible is what makes "did it request but not qualify" a one-line waveform
  question instead of an inference.

  ===> DO NOT RE-ADD BASELINE'S SNI BLOCK — REJECT LIST ENTRY, SHARPENED (A49).
  `issue_slot_past_pnr`, `issue_past_pnr`, `can_issue_sni` and
  `enableConservativeSNI` are a DIFFERENT MECHANISM: speculative
  non-interference, off by default, per-opcode via `MuxCase`, permissive by
  construction. Two gates computing overlapping conditions in two places is how
  one of them ends up disabled by a config flag nobody connected.
  
  AND ONE OF ITS TERMS IS ACTIVELY WRONG HERE, not merely redundant.
  `issue_slot_past_pnr` includes `| (iss_uop.rob_idx === io.rob_pnr_idx)`, but
  `rob_pnr_idx` NAMES THE OLDEST **UNSAFE** ENTRY (`rob.scala:531`) — so equality
  ADMITS THE UNRESOLVED ENTRY. In the scalar SNI context that is intended, because
  that mechanism is deliberately permissive. For `IQ_V_ALU`'s past-PNR gate it
  would hand a STILL-SPECULATIVE op to the coprocessor, which defeats the entire
  reason `pnrGate` exists — and the CII has no branch-kill path to recover with,
  precisely because the gate was supposed to make one unnecessary.
  ===> THE TEST IS THE STRICT `IsOlder(rob_idx, rob_pnr_idx, rob_head_idx)` THAT
  THE SLOT COMPUTES, WITH NOTHING OR-ED ONTO IT. A generator that "restores the
  missing equality case" is reintroducing the bug.

  //@req-spec-cii.d13
  A squashed `IQ_V_ALU` entry never issues. Two mechanisms, and both are needed:
  `kill := io.flush_pipeline` clears `slot_valid` in the slot, dropping the entry
  from the queue before any select can see it; and a branch kill cannot reach an
  ELIGIBLE entry at all, because a past-PNR entry's `br_mask` is necessarily clear
  — the PNR cannot sweep past an unresolved branch, `is_br`/`is_jalr` setting
  `starts_unsafe`. Assert that: on a `pnrGate` instance, an eligible entry is
  never `IsKilledByBranch`. That assertion is the whole of "the CII needs no
  branch-kill path", stated where it is checkable rather than in a comment.

  ---- 6. Grant requires ALL operands ready ----

  //@req-spec-issue.e8
  //@req-spec-cii.d12
  //@req-spec-cii.i9
  A grant requires every operand class ready, and none may be skipped: the scalar
  feeders (base and stride on `prs1`/`prs2`, the `.vx` integer operand, the `.vf`
  scalar on the FP network), `pvl` on the VL network, and the vector source groups
  `pvs1`/`pvs2`/`pvs3` and the mask `pvm` on the vector network. This unit
  contributes no readiness term of its own — `request` arrives from the slot with
  all of them already ANDed, which is what keeps the wakeup-to-grant path one
  combinational cone with no place to lose a term. On `iq_v_alu` the conjunction
  is `past-PNR AND operands ready`, both required and neither sufficient; the
  coprocessor half of a shared op is issued by this same conjunction, when its
  source operands are ready, with no additional condition.

  ---- 7. One slot, one grant, one select ----

  //@req-spec-core.f10
  //@req-spec-core.f11
  //@req-spec-issue.e6
  //@req-spec-issue.e7
  A non-shared vector OP.v occupies exactly ONE slot in exactly ONE queue and is
  granted ONCE. Mechanically: dispatch writes one slot in the queue its `iq_type`
  bit names; the encoder's `uop_issued` term admits that slot at most once per
  cycle; the grant sets `next_uop.iw_issued`, which drops `request` on the
  following cycle; and `next_valid := rebusied` then frees the entry.

  ===> `spec-core.f11` IS KEPT WITH ITS ID AND ITS READING ANNOTATED (A48 /
  decision D12 case 2). "Each issue slot must be granted once" is FALSE AS
  LITERALLY WRITTEN. The requirement is true in spirit and loosely worded, so it
  is annotated rather than retired — retiring a true-but-loosely-worded
  requirement loses coverage. `issue.rst` has been amended to match this reading.
  
  THE CORRECT READING IS "ONCE PER EXECUTION RESOURCE, NOT A LITERAL GRANT
  COUNT". The obligation being protected is that an OP.v is ALLOCATED and SELECTED
  once, with NO SECOND ISSUE STAGE — which is what `spec-core.f14` below states
  directly, and which this module honours exactly.
  
  /!\ WARNING TO ANY GENERATOR OR REVIEWER: A CHECK OF THE FORM
      `assert(PopCount(grants for this slot over the entry's lifetime) == 1)`
      WOULD BREAK EVERY VECTOR STORE. Do not emit it, in any spelling.
  
  TWO EXCEPTIONS, neither of which is a second scheduling decision:
  (a) `squash_grant` and BOOM's speculative load-hit RE-BUSY can retract a
      grant, after which the entry re-requests and IS GRANTED AGAIN — a second
      grant of the same select. Baseline machinery, reused unchanged, present only
      on the SCALAR half (vector operands never wake speculatively): a replay of
      one select, not a second select.
  (b) A vector STORE slot is granted TWICE, once for AGEN and once for DGEN
      (`spec-issue.d10`): two independently grantable PATHS of one slot
      (baseline's mem slot, extended by VecStoreDgenPath). Their order and the
      unbounded gap between them belong to VecStoreDgenPath. This is the case the
      literal reading of f11 forbids and the design requires.

  //@req-spec-core.f14
  An OP.v is ALLOCATED and SELECTED once: this unit has no second issue stage, no
  second priority-encoder select, no re-select of an already-granted entry and no
  intermediate buffer between `iss_uops` and the execution units. `iss_uops` is
  the grant, driven combinationally in the request cycle.

  ===> THE GRANT MUST BE COMBINATIONAL IN THE REQUEST CYCLE, and for the store
  queue that is load-bearing rather than a performance choice.
  VecStoreDgenPath registers NOTHING — a registered grant bit would be stranded
  by the collapse shift when the uop migrates to another slot instance — so the
  AGEN/DGEN grant state rides the migrating uop's `fu_code(FC_AGEN)` /
  `fu_code(FC_DGEN)` pair. Consequently the slot's `iss_fu_code_agen` /
  `iss_fu_code_dgen` overrides must already be visible on `iss_uop` when this
  unit's `fu_code_match` and the grant resolve. Inserting any register between
  `request` and `grant` here desynchronises the path bits from the grant and a
  store either replays its AGEN or never issues its DGEN.

  ---- 8. Shared instructions: two slots, two queues, no coupling ----

  //@req-spec-core.f12
  //@req-spec-core.h6
  A shared instruction — an `is_shared` segmented load or store — occupies TWO
  slots: one in `iq_v_alu` (the coprocessor half) and one in `iq_v_load` or
  `iq_v_store` (the LSU half), while sharing a SINGLE ROB entry. Dispatch presents
  the same uop to both instances in the same cycle, `iq_type` naming both queues,
  and each instance allocates one slot and grants it once through its own select.
  There is deliberately NO cross-queue signal of any kind: no shared grant, no
  cross-queue kill, no cross-queue ready. The two halves rendezvous only on the
  `pvtmp` group's group-done on the ordinary vector wakeup network, and the single
  ROB entry's completion is tracked by the ROB's 1-bit "other half pending" flag.

  //@req-spec-issue.d6
  //@req-spec-issue.d7
  //@req-spec-cii.i5
  //@req-spec-cii.i13
  For a segmented STORE the two halves are serially dependent, and this unit's
  gate is the whole of that dependency: the LSU half's AGEN translates the address
  set, that first translation clears `unsafe` on the shared ROB entry, the PNR
  advances past the entry, and only then does the coprocessor half sitting in
  `iq_v_alu` become eligible — through the SAME per-entry past-PNR gate every
  other CII op passes, with no translation-complete signal and no private path.
  Once eligible (and with its operands ready) it is granted from `iq_v_alu` like
  any other entry.

  //@req-spec-issue.d15
  There is no circular wait, and the reason is visible in this module's port list:
  the eligibility term reads ONLY `rob_pnr_idx`, `rob_head_idx` and the entry's own
  `rob_idx`. It depends on that store's own address translation and on nothing the
  coprocessor half produces. The shared ROB entry carries ONE `rob_unsafe` bit and
  the LSU half clearing it is sufficient — nothing waits on the coprocessor half.
  Gating the PNR on BOTH halves being safe is what would close the cycle, since
  the coprocessor half cannot issue until the PNR passes the entry. The Rob
  delta owns that; this unit must never be given a signal that would let it
  wait on the other half.
  NOTE the direction qualification the SLOT applies and this unit must not
  undo: an ALU slot points `rdy_vs3` at `Mux(is_shared && uses_ldq, pvtmp,
  pvs3)`. A bare `Mux(is_shared, ...)` makes a segmented STORE's coprocessor
  half wait on the very group it is about to write — immediate self-deadlock,
  exercised only by segmented stores.

  //@req-spec-issue.e14
  //@req-spec-cii.i7
  `IQ_V_ALU` is NOT a head-only FIFO and nothing may make it one: no term in the
  select may reference index 0, `rob_head_idx` (the slots read it, only for wrap
  resolution inside `IsOlder`), or "the oldest valid entry". Program-order issue is
  not required, because rename has resolved every register dependence before an op
  crosses the CII, and the three things that might have required it do not:
  `vtype`/`vl`/`vstart`/`vxrm` ride the per-instruction CII issue packet,
  `vxsat`/`fflags` accumulate at commit in ROB order, and CII tags are opaque with
  results already allowed to return out of order. The cost of getting it wrong is
  concrete: step 4 of the chain above can stall for a 256-element translation pass,
  and in a head-only queue that stall blocks EVERY younger vector arithmetic op.
  Ordering between the vector queues, and between vector and scalar work, is
  enforced exclusively by the ROB.

  ---- 9. The element-queue reservation is an ASSERTION, not a stall ----

  For `iq_v_load`/`iq_v_store`, granting an entry additionally requires that the
  dispatch-time element-queue RESERVATION exists. It always does — capacity was
  claimed in program order at dispatch and `VecQueueReservation` refuses the
  dispatch otherwise — so it is CHECKED, not waited on: assert that a valid entry
  in `iq_v_load` has `uses_ldq` and in `iq_v_store` has `uses_stq`, and declare NO
  reservation port. A `Bool` from the reservation unit into this select would be a
  stall condition that can never be false: dead logic that reads as a dependency
  and invites someone to make it a real one.

  ===> AND NOTHING HERE MAY CONSULT A `busy` FROM `VecLsu` (invariant 3). The
  previous attempt exported `io.busy := grp_active || (state =/= sIdle)` from a
  single vector-LSU FSM straight into `fu_ready`, capping vector memory
  concurrency at one op machine-wide. This unit has no port to the vector LSU.
  Its ONLY back-pressure is `fu_types`, which is credit- and capacity-metered
  per execution resource and is not instruction-scoped state.

  ---- 10. Flush, and the one-cycle window this unit deliberately keeps ----

  `flush_pipeline` is BoomCore's `RegNext(rob.io.flush.valid)`, wired to every
  slot's `kill`, exactly as the scalar queues receive it. That leaves a real
  one-cycle window: a grant CAN fire in the `rob.io.flush.valid` cycle itself.
  This unit does NOT close it, on purpose — a combinational `rob_flush` input here
  would give the vector queues different flush timing from the scalar queues and
  break the "reuse unchanged" claim. The window is absorbed on the consumer side by
  `VecCiiHost`'s kill contract, whose `kill_all` asserts from both terms of the
  same flush event so a tag allocated during the window carries `killed`. Stated
  here so the other end of the seam is not "fixed" by deleting that term.

  ---- 11. Assertions ----

  Synthesizable checks on hardware conditions: `PopCount(grants) <= issueWidth`;
  baseline's dispatch-ready/shamt assertion; every accepted dispatch has `is_vec`
  and `iq_type(iqType)` set; `!(dis_uops(w).valid && ppred_busy)`; `uses_ldq` /
  `uses_stq` per queue (part 9); a grant only ever lands on a slot whose
  `eligible` was high this cycle; and, when `pnrGate`, that no eligible entry is
  `IsKilledByBranch`. Also, at ELABORATION:
  `require(numVecWbPorts == io.vec_group_done.length)`, a `require` that this
  parameter equals the `numVecWbPorts` field reached through `HasVectorParams`, and
  a `require` that `io.vl_wakeup.length` equals that trait's `numVlWakeupPorts` —
  the three checks that keep the port counts bound to their single declaration
  rather than to a local literal — plus a check that a non-`pnrGate` instance
  elaborates no `rob_pnr_idx` port, and that an `isStoreQueue` instance's slots
  elaborate no `rdy_vold` (part 13).

  ---- 12. Tracing ----

  Guarded tracing through the shared VecTrace package, one line per key event,
  each tagged `VecIssueUnit` with the queue name and `rob_idx`, gated on the
  `vecTrace` plusarg and off by default so the traced and measured builds are the
  same machine. Three events: dispatch accept (slot index, `is_shared`), grant
  (issue lane and, on the store queue, which of AGEN/DGEN was offered), and — on
  `pnrGate` instances only — the rising edge of `request && !eligible`, the
  past-PNR stall. That third line is what makes CII issue latency attributable in
  a cosim log; without it a PNR stall and an operand stall look identical from
  outside the queue.

  ---- 13. Stale-destination readiness: RESOLVED as a fifth matcher (D6) ----

  ===> PENDING REQUIREMENT — NO `req` TAG YET, ON PURPOSE. This is a CORPUS GAP,
       not a spec-to-code defect: the requirement is being added to the `spec-issue`
       group `g` family via `/spec-to-reqs`, and no ID has been allocated. Do not
       invent one and do not tag this section until the extract lands and
       `hierarchy.yaml` carries the new ID in the affected nodes' `reqs:` lists.
       The behaviour below is BINDING on the generated RTL regardless.

  `stale_pvdest` is the PREVIOUS mapping of the destination arch vregs, so its
  producer is an OLDER instruction — and age-ordered issue grants the oldest READY
  entry, which does NOT guarantee an older producer has completed. Two consumers of
  this unit's grants read the group anyway: the LCB pre-loads inactive-lane data
  from it on VRF port `R2` for a `vta=0` / `vma=0` load (and VecGroupCopy copies the
  whole group for a VL=0 load), and the coprocessor may pull it as the `STALE_VD`
  source slot. Both can otherwise read a BUSY group and get garbage, silently. FOUR
  NODES REPORTED THIS INDEPENDENTLY, and the prior M2 implementation needed exactly
  this term (`pvold_busy`) to avoid a hang.

  This module's share of the fix is ROUTING ONLY, and there are exactly two pieces
  of it:
  - `dis_member_rdy` is one group wider (`vold_rdy` — see the ports section), and
    the collapse move must carry that group like every other, through the ONE
    combined mux of part 3.
  - `numVecWbPorts` is forwarded to the slot's FIFTH VecGroupReady instance
    (`rdy_vold`) exactly as to the other four. The slot owns `prns :=
    stale_pvdest`, `members := v_emul` and `used := dst_rtype === RT_VEC`; the
    select and the assertions here are unchanged.

  `iq_v_store` slots get NO fifth instance — a store has no `stale_pvdest` reader —
  so this module's three instances are no longer symmetric in slot content, only in
  slot interface. Nothing in the select, the shamt chain or the dispatch path
  distinguishes them.

  ===> AN AGGREGATE `stale_pvdest_busy` BIT WAS REJECTED, AND IT IS THE TEMPTING
  WRONG ANSWER: one MicroOp bit instead of 32 added matchers. It is UNSAFE.
  `stale_pvdest` can span UP TO EIGHT PRODUCERS — an `LMUL=1` op writes `v0`, then
  an `LMUL=8` op renames `v0..v7`, so the younger op's stale mapping is the current
  mappings of eight arch vregs installed by up to eight different instructions. One
  bit cannot express "waiting on producer 3 of 8" and no single group-done can
  clear it correctly. Same argument that forces per-member matching for `pvs*`
  (rename.g20). MicroOp must not gain such a field.
  
  ACCEPTED CONSERVATISM: the host cannot know whether the VPU will actually pull
  `STALE_VD` — the coprocessor decides and NO VPU-SIDE SIGNAL EXISTS — so any CII
  op with a vector destination waits on `stale_pvdest`.
  
  COST: +1 matcher x (16 `iq_v_load` + 16 `iq_v_alu`) slots, IN THE STAGE THAT IS
  ALREADY THIS DESIGN'S #1 TIMING RISK. Accepted against a silent wrong-data
  alternative. The mitigation order in the perf section is unchanged and the fifth
  instance shares the shared one-hot decode with the other four.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
ONE SCHEDULING STAGE, ZERO ADDED CYCLES, and this is a constraint rather than a
target. A group-done in cycle N must be able to produce a grant in cycle N:
nothing between `vec_group_done`/`int_wakeup_ports`/`vl_wakeup` and `iss_uops`
may be registered in this module. The only registers here are baseline's
`is_available` vector and the `RegNext` on `dis_uops(w).ready`; all entry state
is in the slots. A pipeline register on the grant path would also strand
VecStoreDgenPath's path bits (logic part 7).

Throughput: `dispatchWidth` (3) accepts and `issueWidth` (1, 2 on the widest
tier) grants per cycle per queue, with `numSlowEntries = 0` so compaction keeps
up with full-rate dispatch. `iq_v_alu`'s grant width is pinned at 1 by
`VecCiiIssue`'s single Issue beat per cycle.

Critical path at the 1 GHz `core_clk` target, and it is the vector issue stage's
critical path overall: group-done PRN fan-out into
`EMUL x numVecWbPorts x maxMembers` comparators inside each slot's VecGroupReady
instances — FIVE of them in `iq_v_load`/`iq_v_alu` since D6, four in
`iq_v_store` — the per-member OR-trees, the AND-reduce, the slot's operand and
eligibility ANDs, then THIS module's priority encoder and the grant.
This module contributes only the encoder and the `fu_code_match` AND. If the path
fails timing, the mitigations in order are the shared one-hot completing-PRN
decode described in VecGroupReady, then raising `numSlowEntries` to shorten the
shift chain. Registering the match, matching fewer group-done ports, or adding a
second scheduling stage are NOT options: the first two lose single-shot wakeups
and the third is forbidden outright.

Area is essentially all slots — about 792 PRN comparators each in
`iq_v_load`/`iq_v_alu` and 600 in `iq_v_store` at the defaults — x 16 entries x
three instances, roughly 34.9k comparators. The D6 `rdy_vold` matcher accounts for
+6.1k of that (+192 on each of 32 slots), about +21%, spent to remove a silent read
of a BUSY stale group. This module ITSELF is only the shamt chain, one priority
encoder per issue lane, and the collapse muxes — now widened by the
member-readiness bundle riding alongside the uop, `5*maxMembers + 1` = 41 bits with
`vold_rdy` included.
<|end_perf|>

<|begin_dependencies|>
VecIssueSlot — instantiated as `slots`, `count: numEntries`, in all three
instances. This module owns the collapse-move wiring and must route
`out_member_rdy` to `in_member_rdy` on the SAME move that routes `out_uop` to
`in_uop` — every group of the bundle, `vold_rdy` included. It forwards `iqType`,
`pnrGate`, `numIntWakeupPorts`, `numFpWakeupPorts` and `numVecWbPorts`, and consumes
`valid`, `will_be_valid`, `request`, `eligible`, `iss_uop`, `out_uop` and
`out_member_rdy`. The select reads `eligible`; `request` is read only for tracing
and assertions. `VecGroupReady` (five instances per load/ALU slot, four per store
slot since D6) and `VecStoreDgenPath` are the slot's children and this module has no
port to either.

MicroOp — `is_vec`, `is_shared`, `iq_type`, `uses_ldq`/`uses_stq`, `rob_idx`,
`fu_code`, `pvl`/`pvl_busy`, `iw_issued`, `iw_issued_partial_agen`/`_dgen`, the
`iw_p*` hint and speculative-child fields, `prs1`/`prs2`, `lrs*_rtype`,
`ppred_busy`, and baseline's `exception`/`is_fence`/`is_fencei` for the
`will_be_valid` term. This module adds no field to MicroOp — in particular no
per-member busy vector, which is exactly why the side channel exists.

VecBundles — `VecGroupDone` on the vector wakeup ports, passed straight through to
the slots, and `VecMemberRdy`, the per-member side channel, which belongs THERE and
carries `vs1_rdy`/`vs2_rdy`/`vs3_rdy`/`vtmp_rdy`/`vold_rdy` plus `vm_rdy` since D6.
Bind to that single declaration; `VecSlotMemberRdy` (the copy in
VecIssueSlot.nlhdl.scala) is the same bundle under a second name and must not
survive.

VectorParams — `vecIssueEntries` (numEntries), `vecIssueGrantWidth` (issueWidth),
`maxMembers`/`vecPregSz` through the bundles, and the THREE PORT COUNTS IT NOW
DECLARES: `numVecWbPorts` (3), `numVecClrPorts` (3) and `numVlWakeupPorts`
(`aluWidth + 1`). This module's `numVecWbPorts` constructor parameter BINDS to that
field and no longer claims to be the settling site, and `vl_wakeup` is sized from
`numVlWakeupPorts`. `numVecClrPorts` is not used here — this unit has no ROB clear
port — and is named only so the two 3s are not mistaken for one parameter.

VecTrace — the guarded emit helper used in logic part 12.

Binds to EXISTING BOOM v4 declarations rather than re-spelling them: `IssueParams`,
the `IQ_V_LOAD`/`IQ_V_STORE`/`IQ_V_ALU`/`IQ_SZ`/`FC_SZ` constants
(ScalarOpConstants delta), `Wakeup`, `BrUpdateInfo`, `IsKilledByBranch`, `IsOlder`,
`RT_FIX`/`RT_FLT`, `FC_AGEN`/`FC_DGEN`, and `IssueUnitCollapsing`'s
`SaturatingCounterOH` helper.

Instantiated by VecPipeline three times — `iq_v_load`, `iq_v_store`, `iq_v_alu`.
VecPipeline drives `dis_uops`/`dis_member_rdy` from the registered
`ren2_uops`/`dis_fire` path (never from `dec_uops`), `vec_group_done` from the
group-done producers, `rob_pnr_idx`/`rob_head_idx`/`brupdate`/`flush_pipeline`
from `vec_pipeline_io`, and `fu_types` from `VecCiiIssue` (converted with
`.asBools` — the A51 conversion point) and the vector LSU side. It consumes
`iss_uops`: `iq_v_alu` lane 0 into `VecCiiIssue.io.iss`, the other two queues'
lanes into `VecLsu`.

Under decision D2 a vector config's dispatch source is `CompactingDispatcher`, and
`dis_uops` is wired NATIVELY from `dispatcher.io.dis_uops(i)` — no `ready := true.B`
hack, no queue fullness re-routed through the seam's single `dis_ready` bit. This
module's `io.dis_uops(w).ready` is per-lane back-pressure the dispatcher masks by
`iq_type`, which is why a full vector queue no longer stalls a pure-scalar lane
(logic part 3).
<|end_dependencies|>

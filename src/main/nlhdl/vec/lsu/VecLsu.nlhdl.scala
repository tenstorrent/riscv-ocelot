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
VecLsu — the vector load/store unit container: the vector LS AGEN stage, the six
element queues, the drain side, the memory-ordering mechanisms, and the wiring
that binds them to BOOM's Unified LSU. It contains all vector memory datapath
and owns no datapath of its own.
*/

hierarchy.yaml: kind: module, mode: new,
output src/main/scala/v4/vec/generated/lsu/VecLsu.scala,
package boom.v4.vec.generated.lsu, group vec_lsu.
depends_on MicroOp, VecBundles, VectorParams, VecTrace.
Instantiated ONCE, as `vlsu`, inside VecPipeline — which exists only when
`usingRVV` is true, so in a vectors-off build this whole subtree is ABSENT, not
tied off, and the emitted RTL is bit-identical to pre-Caracal BOOM v4.
`usingRVV` is a Scala `Boolean` of `BoomCoreParams`, never a hardware `Bool`,
and it is not rocket's `usingVector`.

Seventeen children: `resv`; `ld_opnd`/`st_opnd`; `ld_msk`/`st_msk`;
`ld_elem_agen`/`st_elem_agen`; `ld_range_agen`/`st_range_agen`; the six
`VecElemQueue` instances; `ld_beat`/`st_beat`; `dgen`; `lcb`; `gcopy`; `arb`;
`snoop`; `fwd`; `hold`; `squash`.

===> THE MODULE EXISTS TO UPHOLD ONE INVARIANT, AND EVERY STRUCTURAL CHOICE
     BELOW IS DOWNSTREAM OF IT. Address generation is cut BY PIPELINE POSITION,
     not by the inherited Packer/Skipper/Walker x load/store cross-product
     (six modules, 2110 lines, whose duplication had already produced divergent
     mask support between the load and store Packers). FILL side: `VecElemAgen`
     (SSI, one address per active element) and `VecRangeAgen` (unit-stride, ONE
     range entry). DRAIN side: `VecBeatExpander`, which coalesces just in time.
     DIRECTION IS A PARAMETER, NEVER A SEPARATE MODULE — each agen, each operand
     read, each mask streamer and each beat expander is instantiated twice so the
     two directions never arbitrate. The load-priority mux over a single shared
     unit is what silently dropped store grants in `addvector`.

===> NO MODULE IN THIS SUBTREE MAY HOLD STATE SCOPED TO "THE CURRENT
     INSTRUCTION", AND NONE MAY EXPORT A `busy` THAT GATES ISSUE. In-flight
     state lives in exactly THREE places, and plan GROUND RULE 6 WAS AMENDED to
     say three rather than two: (a) the six element queues — whose capacity is
     reserved at dispatch, in program order — (b) the LCB's per-PRN assembly
     entries, and (c) THIS MODULE'S PER-LDQ/STQ-ENTRY DESCRIPTOR PENDING TABLE
     (section 3b). (c) is the same KIND of state as (a) and that is why the rule
     admits it rather than being bent around it: one row per LDQ/STQ entry, the
     row indexed by a placeholder the dispatch-time reservation already
     guaranteed, so it is structurally un-overflowable; it is not scoped to "the
     current instruction" (a row belongs to a queue entry, and rows for several
     ops coexist); and it exports no `busy`. The rule is amended IN THE TEXT
     because gate H4 reviews against that text — a table appearing under a rule
     that enumerated two homes would be right to reject.
     This module therefore exports no `busy`, no `active`, no `fu_ready` and no
     credit toward any issue unit. Issue eligibility for a vector memory OP.v is
     "a reservation exists", decided at dispatch by `resv` and nowhere else.
     Gate H4 greps this whole subtree for a `busy` reaching an issue unit; a hit
     is a failed review regardless of measured performance. `vec_lsu_empty`, the
     LCB's `free_count`, `hold_ldq` and `dis_ok` are each explained below in the
     terms that distinguish them from a `busy`.

Governing spec anchors: execution.rst `vector-ls-agen` and `vector-agen`;
loadstore.rst `lsu-unified`, `ssi-queues`, `us-queue`, `store-data-queue`,
`vec-load-algo`, `vec-store-algo`, `mem-order`, `vec-squash`, `elem-progress`,
`dcache-arbiter`, `vector-bw-ceiling`; midcore.rst `group-done-wb`, `vrf-ports`,
`midcore-segmented-load`; issue.rst `shared-store-chain`,
`vec-queue-reservation`; case_study.rst `case-segmented-ls`, `case-vl-zero`;
glossary.rst `glossary-terms`.

<|begin_module|>

  <|begin_parameters|>
  This module introduces no tuning knob of its own. It exists to DERIVE the
  children's parameters from one place so two siblings cannot be configured
  inconsistently, and every value below is an elaboration-time Scala value.

  From `HasBoomCoreParameters`: `coreWidth`, `lsuWidth`, `numLdqEntries`,
  `numStqEntries`, `ldqAddrSz`, `stqAddrSz`, `robAddrSz`, `xLen`,
  `numIrfWritePorts`, `coreDataBytes`. From VectorParams through
  `HasVectorParams`: `vLen`, `eLen`, `vecPregSz`, `vlPregSz`, `maxMembers`,
  `vecVLSz`, `ssiQueueEntries`, `usQueueEntries`, `lcbEntries`,
  `dcacheArbiterMode`.

  ---- Derived, and passed down ----

  `queuePorts` = `lsuWidth` — the fill/consume lane count of every VecElemQueue
  instance, and the same number reaches `ld_beat`/`st_beat` as `nLanes`, `snoop`
  as `searchPorts` and `arb` as its lane count. `readPorts` is left at the
  child's default `ports + 1`: one read per drain lane plus the ONE shared read
  port the disambiguation and forwarding consumers share (see the shared-port mux
  in the logic section).

  //@req-spec-lsu.a13
  On `LargeBoomV4Config` and `MegaBoomV4Config` the dual-port L1 D$ lets two
  memory operations issue per cycle, and the vector address and data queues must
  be `2 x nOP.v` wide so two concurrent element accesses can be presented. That
  obligation is discharged HERE, by setting VecElemQueue's `ports` parameter to
  `lsuWidth` on all six instances rather than leaving it at its default of 1 — the
  queue itself is written against the parameter and cannot satisfy the requirement
  alone. `nLanes` on the beat expanders and `searchPorts` on the snoop are set
  from the same value, because a queue lane with no drain lane, or a search lane
  with no drain lane, would present an address the machine never translated.
  Require `dcacheArbiterMode` to agree ("single" implies `lsuWidth == 1`,
  "dual-dynamic" implies 2), failing elaboration on a mismatch rather than
  silently picking one.

  Queue payload widths, all derived at elaboration and never written as literals:
  `addrWidth` = `(new VecElemAccess).getWidth` for the two SSI address queues,
  `rangeWidth` = `(new VecRangeEntry).getWidth` for the two US address queues,
  `eLen` for `st_SSI_DATA_Q`, `vLen` for `st_US_DATA_Q`. Depths are
  `ssiQueueEntries` and `usQueueEntries`. `reserved` is true on all six;
  `isStore` is stated explicitly on all six (it has no default, deliberately);
  `hasXlatePass` is true on `st_SSI_ADDR_Q` and `st_US_ADDR_Q` only.

  `nKillClients` = 5 — NOT the child's default of 8 — and the ORDER IS FIXED
  because `kill_uop` and `kill` are positionally paired: 0 `ld_msk`, 1 `st_msk`,
  2 `ld_beat`, 3 `st_beat`, 4 `gcopy`. Those are exactly the children that expect
  a RESOLVED kill `Bool` "from the parent". `ld_elem_agen`, `st_elem_agen`,
  `ld_range_agen`, `st_range_agen`, `dgen` and `st_opnd`/`ld_opnd` take
  `brupdate`/`rob_flush` directly and evaluate BOOM's `IsKilledByBranch`
  themselves — VecElemAgen MUST, because it holds two uops with different
  `br_mask`s and one pre-resolved Bool can only ever be right for one of them —
  and `idx` (VecIdxGen) is killed by its parent agen, never by this unit.

  ===> `VecSquashUnit`'s OWN PARAMETER COMMENT IS STALE AND THIS SITE IS
       AUTHORITATIVE. It still enumerates eight clients — "the four fill-side
       agens, `idx_gen`, `mask_stream` and the two `VecBeatExpander`s" — which
       predates the mask-streamer hoist to this level and predates the four
       agens taking `brupdate` directly. Corrected client set: the two mask
       streamers (now this module's instances, so they DO need a resolved kill
       from here), the two beat expanders, and `gcopy`. Five. When
       VecSquashUnit's text is next regenerated its comment must be brought to
       this list; until then, do not read a default of 8 as an instruction to
       wire three dangling clients.

  `isStore` is forwarded on both `VecScalarOperandRead`, both `VecElemAgen`, both
  `VecRangeAgen`, both `VecMaskStream` and both `VecBeatExpander` instances. It is
  stated at every instantiation site, never defaulted, because a defaulted
  direction gives one path the other's freeing semantics.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit pair, matching the hierarchy defaults:
  single `core_clk` domain, posedge `clock`, ACTIVE-HIGH SYNCHRONOUS `reset`. No
  second clock domain and no asynchronous reset appears anywhere in this subtree;
  the one active-low crossing in the design is in `tt_cii_host_wrap`, far from
  here.

  ---- Dispatch, issue and recovery (from VecPipeline, off `vec_pipeline_io`) ----

  `dis_uops` (`Vec(coreWidth, Valid(MicroOp))`) and `dis_fire`
  (`Vec(coreWidth, Bool)`) in, `dis_ok` (`Vec(coreWidth, Bool)`) out — passed
  straight to `resv`. These are the REGISTERED `ren2`/dispatch uops, never
  `dec_uops`.

  `iss_ld`, `iss_st` — `Input(Valid(MicroOp))`, the grants from `iq_v_load` and
  `iq_v_store`. NO READY, NO CREDIT, NO `busy` IN EITHER DIRECTION, and after
  decision D5 the reason is stronger than "two operand-read instances buy it": a
  grant is written unconditionally into section 3b's per-LSQ-entry table, whose
  row is indexed by a placeholder the reservation already guaranteed, so refusing
  a grant is not merely undesirable but unnecessary — the full state is
  unrepresentable.

  `brupdate` (`BrUpdateInfo`), `rob_flush` (unregistered `rob.io.flush.valid`),
  `rob_flush_kill` (its registered form, the cycle baseline lsu.scala rolls its
  LSQ pointers), `commit_valids`/`commit_uops` and `rob_head_idx`/`rob_pnr_idx`.

  ---- Register-file seams (cited by port number, none added) ----

  `int_rf_read_req`/`int_rf_read_rsp` — four lanes, two per operand-read instance
  (base `prs1` and stride `prs2` concurrently). EACH REQUEST LANE IS `Decoupled`,
  not a bare `Valid`, and that is the one interface change decision D5 makes on
  this seam. `PartiallyPortedRF` denies by INDEX PRIORITY —
  `ready := PopCount(io.arb_read_reqs.take(i).map(_.valid)) < numPhysicalReadPorts`
  — the vector lanes are appended LAST, and on Medium there are ~7-9 existing
  logical readers plus five vector lanes against FIVE physical ports. Denial is
  therefore ROUTINE, not exceptional, and a lane whose `ready` is ignored returns
  the wrong base or stride with no assertion anywhere. The `ready`s are consumed
  by section 3b's table, which simply keeps the row presented; they reach no issue
  queue. `int_wb_snoop` (`numIrfWritePorts` of {addr, data}) for the stale-base
  forward.
  NO `fp_rf_read_req`/`fp_rf_read_rsp` ON THIS MODULE AT ALL. Decision D4 DELETED
  the store-side FP reader `st_opnd` used to carry: no RVV store form takes an FP
  scalar operand — store data is always `vs3`, and a store's scalar operands are
  `rs1` (base) and `rs2` (stride), both integer. The single FP read lane left on
  `vec_pipeline_io` belongs to `VecCiiIssue`, and neither operand-read instance
  here has one.
  `vl_read_addr`/`vl_read_data` — two lanes, one per operand-read instance.
  `vl_wb` — `Output(Valid({pvl, vl}))`, VlRegFile's `W_lsu` port, with EXACTLY ONE
  driver inside this module (see the `vleff` paragraph).

  VRF ports, per the canonical table in midcore.rst `vrf-ports`, by number:
  `vrf_r0` (load index), `vrf_r1` (load mask), `vrf_r2` (`stale_pvdest`),
  `vrf_r3` (store data), `vrf_r4` (store mask AND store index — one port, two
  readers, muxed here), `vrf_w0` and, at `lsuWidth = 2`, `vrf_w1`. Each read is a
  `Valid(addr)` out with the data returned REGISTERED one cycle later; each write
  is `Valid(addr, data, byte-mask)`. NOTHING HERE ADDS A VRF PORT.

  ---- The Unified LSU seam ----

  `lsu_vec` — the `VecLsuCoreIO` bundle named by the `vec_pipeline_io` interface
  entry. IT IS DECLARED IN `lsu.scala`, BESIDE `LSUCoreIO`, IN PACKAGE
  `boom.v4.lsu`, AND THIS MODULE BINDS TO THAT ONE DECLARATION BY NAME. It is not
  declared here and a second declaration anywhere is a duplicate to reject. Two
  reasons settle it that way rather than the other: `VecBundles.VecPipelineIO`
  already carries a `lsu_vec: VecLsuCoreIO` field typed against a `boom.v4.lsu`
  type, and the LSU owns the LDQ/STQ pointers and placeholders the bundle
  carries — the producer of the state should own the type that exposes it. This
  module is a CONSUMER of the type and the counterparty on the bundle, nothing
  more. It remains an AGGREGATE of bundles the children already declared, not a
  new vocabulary: `LsuResourceClaim` (declared in VecDcacheArbiter's file) x
  {`scalar_demand`, `scalar_avail`, `vec_claim`},
  `vec_fire`, `dmem_req_ready`, the LCAM presentation `VecLcamSearch` and the
  load-side search context, `stq_addr_matches`/`stq_forward_matches`,
  `vst_addr_match`, `ldq_valid`/`ldq_alloc`/`stq_vec_valid`/`stq_alloc`, the four
  LSQ pointers `ldq_head`/`ldq_tail`/`stq_commit_head`/`stq_tail`, the LDQ/STQ
  element-cursor read/write pair (`elem_next`, `elem_done`, `fault_elem`), the
  per-STQ-entry committed flag, `fwd_resp`, `replay`, `clr_bsy` and
  `fencei_rdy_vec`. A second declaration of any of those bundles is a duplicate to
  reject.

  ---- Completion and exception, out ----

  `vec_clr_bsy` — two of the three `numVecClrPorts` lanes are driven from here:
  lane 0 by `lcb.io.group_done`, lane 2 by `gcopy.io.group_done`. Lane 1 is
  VecCiiComplete's and is not visible here. Lanes, never an arbiter: neither
  producer can be back-pressured.
  `vec_rob_flags` accompanies those lanes and carries nothing from the LSU today.
  `vec_clr_unsafe` — `Output(Valid(robAddrSz))`, the single group-safe event.
  `vec_xcpt` — `Output(Valid(VecException))` {uop, cause, badvaddr}. No element
  index: a faulting vector memory op traps with `vstart = 0`. `uop` is the FAULTING
  `OP.v`'s own uop, taken from the queue entry that faulted — NOT rebuilt from a
  `rob_idx`, and NOT the uop currently at some cursor. The ROB latches it and reads
  `uop.br_mask` for `GetNewBrMask`, so a wrong or empty `br_mask` here drops the
  fault or attributes it to another instruction. (Re-declared at E-prep; the bundle
  previously carried a bare `rob_idx` and could not drive `rob.io.lxcpt` at all —
  see `VecBundles`, `---- VecException ----`.) `cause` is rocket's cause space,
  `log2Ceil(Causes.all.max + 2)` bits, not `xLen`.
  `lsu_fencei_rdy_vec` — `Output(Bool)`, the `vec_lsu_empty` term.

  ===> AND THAT IS THE WHOLE INTERFACE. There is deliberately no `busy`, no
  `active`, no `grp_active`, no `fu_ready` contribution, no per-instruction status
  and no ready line toward any issue queue. `dis_ok` is a DISPATCH-stage capacity
  answer consumed in program order; `lsu_fencei_rdy_vec` gates only `is_unique`
  dispatch through BOOM's pre-existing `fencei_rdy`. Neither is scoped to an
  instruction and neither reaches an issue unit.
  <|end_ports|>

  <|begin_logic|>

  ---- 0. INTEGRATION CONTRACT: what the sub-modules already built require ----

  Written at E7 from the thirteen sub-module generation reports. Every item below is a
  seam where a sub-module made a choice this container must match, or left an obligation
  this container must discharge. None of them are optional and none are visible from
  this file's own prose alone.

  (a) MASK GRANULARITY IS CONVERTED HERE, AND NOWHERE ELSE. `VecRangeEntry.mask` is
  ELEMENT-granular (`vLen` bits, one bit per element, produced by `VecMaskStream` as the
  tail-cleared `us_mask`). `VecSnoopCandidate.active_mask` is BYTE-granular
  (`maxVecMembers * vLen / 8` bits). When this module builds a snoop candidate from a
  queue entry it MUST expand each mask bit into `1 << eew` byte bits. The expansion used
  to be the forwarder's, and moved here when `VecSnoopCandidate` was given a byte-granular
  field — so `VecStoreForward` no longer does it and will read the mask verbatim. Two
  fields of the same width are not the same quantity: at the defaults both are 256 bits,
  which is exactly why getting this wrong is invisible.

  (b) `io.op` TO EACH AGEN MUST BE ONE CYCLE PER GRANT, AND THIS MODULE IS WHAT MAKES IT
  SO. `VecScalarOperandRead.io.out.valid` has NO self-clear: it holds the same resolved
  op until the issue queue grants a different one. `VecElemAgen`'s accept is level-
  triggered against the ports section's "one cycle per granted OP.v" promise, so the
  per-LDQ/STQ-entry descriptor table here must convert the persistent valid into a
  single-cycle presentation. `VecElemAgen` carries `assert(!(accept && w_valid))` as the
  canary; if that fires, this is why.

  (c) `nKillClients` — RECONCILE THE TWO ROSTERS. `VecSquashUnit` defaults to 8 and
  enumerates {4 fill-side agens, idx_gen, mask_stream, 2 beat expanders}; this file's own
  text says 5 and names {ld_msk, st_msk, ld_beat, st_beat, gcopy}. They disagree on both
  count and membership. THE CRITERION: a kill client is a module THIS FILE INSTANTIATES
  DIRECTLY that exposes a `kill` input. `VecIdxGen` is therefore NOT one — it is
  instantiated inside `VecElemAgen`, which owns forwarding kill to it. Settle the roster
  against the generated interfaces, not against either prose list, and fix whichever
  prose is wrong.

  (d) QUEUE-POINTER NARROWING IS THIS MODULE'S JOB. `VecQueueReservation.rollback_tail`
  and `VecSquashUnit.q_squash` are `resvPtrSz` (= `log2Ceil(ssiQueueEntries) + 1`) —
  one uniform width for all six queues. Each `VecElemQueue` takes its OWN `ptrW`
  (`log2Ceil(entries) + 1`), which is narrower for the three US queues. Narrow by
  truncation when connecting. This is exact, not a cast, and only because `VectorParams`
  now requires both depths to be powers of two with `2*ssiQueueEntries` a multiple of
  `2*usQueueEntries` — do not remove those requires.

  (e) THE DRAIN-READ `xlated` ASSERTION IS THIS MODULE'S. `VecElemQueue` cannot make it:
  its `io.rd` is one anonymous indexed-read array and nothing on it distinguishes an
  execute-time translate read (which expects `xlated = 0`) from a post-commit drain read
  (which requires 1). Assert it where the drain read is issued, which is here. Do NOT
  "fix" this by adding an `is_drain` flag to the shared read port.

  (f) CONSTRUCTOR SIGNATURES AS BUILT, which differ from this file's prose:
  `VecElemQueue`'s FIRST parameter is `queueName: String` — pass the six normative names
  (`ld_SSI_ADDR_Q` etc.) exactly as the enumeration spells them, because all six share
  every trace tag and assertion message and the name is the only thing that tells them
  apart. Its `readPorts` is DERIVED (`ports + 1`), not a parameter.
  `VecBeatExpander` takes `(isStore, nLanes, dmemBeatBytes)` — pass `lsuWidth` and
  `coreDataBytes`; neither has a default, deliberately.
  The LCB credit toward the beat expander is THREE RAW TERMS —
  `lcb_free_nonzero`, `lcb_walk_active`, `lcb_walk_rob` — not a reduced ready bit
  and not a per-lane vector of them. Per-lane was the old shape and it solved the
  wrong half of the problem: at `lsuWidth = 2` two beats in one cycle can indeed
  target two different destination members, but what actually decides the credit
  is WHICH OP each beat belongs to, and that is `us_head` for the unit-stride path
  and `ssi_head(i)` for each SSI lane. Only the expander holds both, so it forms
  the test; see its `lcbRdyFor`.

  (g) HOISTED INSTANCES AND THEIR ROUTING. `VecMaskStream` is instantiated HERE, once per
  direction, not inside the agens. Route its `us_mask` to `VecRangeAgen.io.mask` (the range
  agen has no mask reader and must not grow one), and its `staged`/`ahead`/`skip_*` cursor
  to `VecElemAgen`. A unit-stride OP.v retires the streamer's cursor in the cycle the latch
  loads, so `us_mask` is available immediately — the range agen is combinational and
  one-shot and samples it right away.
  IMMEDIATELY MEANS THE LAUNCH CYCLE ITSELF WHEN THE OP.v IS UNMASKED, and that case is
  the one a registered "now awaiting the mask" bit gets wrong. An unmasked op elides the
  mask read, so the streamer's latch loads combinationally from `op` and drives both
  `us_mask.valid` and `done` in the SAME cycle the start pulse is asserted; a masked op
  reads the VRF first and presents them one cycle later. Qualify the range agen's accept
  with `us_mask.valid` ORed against the start pulse, never with a register set BY that
  pulse — the latter samples one cycle late, `us_mask` has already cleared, and the range
  is silently never pushed. The failure is worse than a dropped op because the await bit
  stays set: the NEXT OP.v's `us_mask` fires it instead, so a range is pushed carrying the
  wrong uOP's base and length while the first OP.v never completes and wedges the ROB head.

  (h) THE ORDERING TRIANGLE, RESTATED BECAUSE THREE MODULES DEPEND ON IT. Route
  `fwd.known_overlap` into `hold.known_overlap`; `hold` computes no overlap test of its
  own, so this routing IS the "exact complements" guarantee. Route `hold.hold_ldq` into
  `arb.hold_ldq` — the hold decides who, the arbiter owns suppression. Give `snoop` strict
  priority over `fwd` on each store queue's shared read port; `fwd` is the correct loser
  because it has a declared replay path, while `snoop`'s read is tied to an LCAM grant the
  arbiter cannot retract.

  (h2) THE THREE ORDERING SWITCHES, AND WHICH ONE `vecScalarSnoopEnable` IS. This
  container is the only instantiator of the three ordering nodes, so it is where the
  config sub-flag becomes hardware, and the mapping is NOT one flag to three modules:

    snoop  — instantiated and active UNCONDITIONALLY. It has no correct `false`
             setting and `vecScalarSnoopEnable` must not reach it. A vector store
             drains POST-COMMIT, so its addresses are the ONLY thing that orders a
             younger scalar load against it; with the search off that load reads a
             line the store has not written yet and nothing replays it. The plan's
             "the flag gates their behaviour, not their existence" is right about
             `fwd` and `hold` and wrong about `snoop`, and this is where that is
             resolved rather than in the config.
    fwd    — `enableVecStoreForward = vecScalarSnoopEnable`, the scalar-consumer
             forward. Genuinely optional: with it false every eligible scalar
             forward becomes a replay, which is slower and correct.
             `enableVecBeatForward = false` at every tier, per that module's
             logic paragraph 9 and the ownership question recorded there.
    hold   — `enableOrderHold = true` at every tier, and
             `forwardingEnabled = enableVecBeatForward` — i.e. false — so the hold
             covers the US/US pair `fwd` is not building. The two flags are asserted
             equal inside the modules; this file must pass the SAME Scala value to
             both and must not spell the constant twice.

  ===> DO NOT DERIVE `enableOrderHold` FROM `vecScalarSnoopEnable`. Turning the
       hold off while the scalar forward is also off is harmless (the LSU's own
       `ldst_addr_matches` kill/replay is the floor for every class), but turning
       the hold off is the one change that converts a correctness mechanism's
       absence into a replay storm on multi-element loads, and the flag's stated
       purpose is to stage the PERFORMANCE mechanisms, not the floor.

  (h3) `data_filled` ON A SNOOP CANDIDATE IS ANSWERED FROM THE DATA QUEUES' OWN FILLED
  BITS, AND MAY NOT BE A CONSTANT. `VecCrossLsuSnoop` gates a store presentation on
  `c.ready := !is_store || data_filled` (spec-memord.a22), and `VecStoreForward` ¶2 relies
  on that gate to call "matched a store whose data is not captured yet" UNREACHABLE —
  which is why it checks `resp.filled` with an assertion rather than a stall. Driving
  `data_filled` from a constant true defeats the gate and makes the unreachable case
  reachable: a store address reaches the LCAM ahead of its bytes and the forward reads an
  unwritten entry. That is silent corruption wherever the assertion is compiled out.

  ===> THE CONSTANT-TRUE FORM WAS TRIED AND IS WRONG, so do not re-derive it. The argument
       for it — `VecQueueReservation` claims a store's address and data regions together
       with equal counts and bases, so the data is always written first — is plausible and
       false. With the scalar forward enabled, `ms4p5_vle64_2` trips VecStoreForward's
       "SSI forward read an unfilled data-queue entry" assertion directly.
  The two classes are answered differently and the difference is not cosmetic. A US
  candidate describes ONE range whose data occupies `members` entries of `st_US_DATA_Q`
  starting at `us_data_base`, and a future load may address ANY of them, so the answer is
  the AND over that whole window — a per-member answer is not available at presentation
  time, because which member a load will want is not known until the load searches. An SSI
  candidate is a single element, at the SAME ordinal in `st_SSI_DATA_Q` as its address
  entry. A LOAD candidate has no data half and answers true.
  This needs a combinational read of the filled state, which the registered `rd` port
  cannot give, so `VecElemQueue` exposes `filled_vec` — a read-only view of the per-entry
  register it already keeps, declaring no new state and adding no read port.

  (i) PORTS THE SUB-MODULES NEED FROM `lsu.scala`, VIA `VecLsuCoreIO`, THAT THE LSU DELTA
  MUST THEREFORE EXPOSE: `VecQueueReservation` needs `ldq_head` and `stq_head` (for
  `IsOlderLSU`); `VecOrderHold` needs `stq_head` and `ldq_next_stq_idx`; `VecSquashUnit`
  needs `ldq_head`/`ldq_tail`/`stq_commit_head`/`stq_tail`. None of these modules may
  track LDQ/STQ pointers locally — a second copy drifts on precisely the mispredict cycle
  it is needed. The store tag pool additionally needs `store_failed`, the D$'s
  `s2_store_failed` forwarded unfiltered — see the store-squash entry in section (j).

  (i2) D$-ACCEPTANCE WATCHDOG ON THE LOAD BEAT PATH. Parameter `ldAcceptWatchdog`
  (Int, default 4096, 0 disables and emits no register). Per load lane, count cycles for
  which a beat is pending and the arbiter refuses it — the same condition the `ld_blocked`
  trace already reports, `(ld_beat.req(w).valid || ldRpyValid(w)) && !arb.ld_req(w).ready`
  — and assert the count stays within the bound.

  A refused lane is normal for a few cycles: the scalar side wins the port, or an MSHR is
  filling. Permanently refused is a different failure. The beats stuck behind it are the
  ones some destination group is still waiting on, so that group never completes and the
  deadlock surfaces TWO MODULES AWAY, in rename, as an empty free list — which is where
  the reader ends up looking. This assertion is the one that names the actual stall point.

  Keep `beat_v` and `avail` in the message. On `axpy-vector` the lane sat here with
  `beat_v=1 avail=1` — a beat ready to send and a tag free to carry it — and that pair
  being high is exactly what rules out the vector side and puts the fault at the cache
  interface. Pairs with VecLoadCoalescingBuffer's per-entry completion watchdog: this one
  says WHERE the beats stopped, that one says WHICH member went short.

  (k) ONE GRANT PER DIRECTION PER CYCLE, AND `vecIssueGrantWidth` DOES NOT APPLY TO THE
  MEMORY QUEUES AS BUILT. Found at E7 on the wide tier. `iss_ld`/`iss_st` are each a
  single `Valid(MicroOp)`, `VecScalarOperandRead` states "at most one grant per cycle
  arrives on it", and the descriptor table accepts one row per direction per cycle. But
  `VecPipeline` was constructing `iq_v_load`/`iq_v_store` with `issueWidth =
  vecIssueGrantWidth`, which is 2 on Mega — so the queue could grant two loads in one
  cycle and the container would consume only lane 0. The second grant is DROPPED: the
  issue slot believes it issued and frees itself, the LSU never sees the op, and the
  load never completes. A hang, not a slowdown, and only on a tier with
  `vecIssueGrantWidth > 1`.
  Both memory queues are therefore constructed with `issueWidth = 1` until the container
  accepts a vector of grants. THIS IS A THROUGHPUT LIMIT, NOT A CORRECTNESS CHOICE, and
  it is deliberately NOT hidden by widening the port and dropping the extra grant.
  Note what is still wide on Mega, because the limit is narrower than it sounds: ONE
  `OP.v` starts per direction per cycle, but its elements drain at `lsuWidth` beats per
  cycle. P1/P2 live on the drain side, which is unaffected. What is capped is
  instruction-level overlap between two loads, not element bandwidth within one.
  Lifting it means widening `iss_ld`/`iss_st` to `Vec(vecIssueGrantWidth, ...)` AND
  giving the descriptor table, both operand readers and both mask streamers a second
  accept path — a real change, not a port widening. Re-check this at H1 against the
  measured P-targets before deciding it is worth it.

  (k4) THE TRANSLATE PASS REWRITES THE RANGE ENTRY'S `base`, SO EXACTLY ONE BEAT MAY
  DO IT — THE FIRST. Every beat of a US store carries `uses_tlb`, so a translation
  comes back for each, and each is `base + offset`. Writing every response back into
  the entry leaves `base` holding whichever beat translated LAST; the write pass then
  reads that entry and starts there. Symptom on a 4-beat store: the translate pass
  covers base..base+24 correctly, the write pass covers base+8..base+32 — the first
  element is never stored and one dword PAST THE RANGE is, which is memory corruption
  outside the instruction's own footprint. Qualify the write-back with the beat's
  `first` marker. Note this records only the FIRST beat's translation, so a range
  spanning a page boundary would need per-beat physical addresses that a single `base`
  field cannot hold — out of scope here, but the reason this field is not a general
  physical-address cache.

  (k3) `st_beat.is_write_pass` IS ONE PORT SHARED BY THE US AND SSI HEADS, SO IT HAS
  EXACTLY ONE DRIVER. The US store's pass state (`stUsWritePass`) and the SSI store's
  (`stSsiDoPass2Prev`) are separate registers, and assigning the port from each in turn
  is not two drivers in Chisel — it is one, the LAST. Written that way the SSI value
  wins unconditionally and a unit-stride store is stranded in its translate pass: the
  expander gates `us_pop` on `is_write_pass` for a store, so the range never pops,
  `st_drain_done` never fires, the STQ placeholder never succeeds, and the first
  `fence` after the store hangs — AFTER the store has already committed and matched
  the reference, which is what makes it look like a scalar-fence bug. Select on which
  head is staged (`stUsStagedValid`), matching the expander's own US-first priority.
  ===> A SINGLE PASS BIT FOR BOTH CLASSES IS A LATENT SEAM DEFECT, not just an
  assignment order to get right: a US store draining pass 1 while an SSI store drains
  pass 2 needs two different values on the same wire in the same cycle. The Mux is
  correct whenever one class is in flight, which is every case the LS suite reaches;
  making it correct in general means giving VecBeatExpander a pass bit per head.

  (k2b) ...AND THE DGEN GRANT IS WHAT STARTS `dgen`. The two grants are not
  bookkeeping noise to be filtered away: `VecStoreDgenPath` withholds the DGEN offer
  until `!dgen_operand_busy && dgen_operand_ready`, so the grant IS the statement
  "this store's data register has been written." Driving `dgen.io.req` from the AGEN
  pulse instead reads `pvs3` out of the VRF at address-generation time — for
  `vle v3 / vse v3` that is before the load has written it, and the store silently
  writes whatever the register file happened to hold. It corrupts nothing structural,
  commits, drains, and matches the reference AT COMMIT, because the checker compares
  the store's own architectural effect and not the bytes that reached memory.
  `data_base` is still sampled at the AGEN pulse, where its `resv_lookup` lane is
  live, and held for the later grant.

  ===> "ONLY ONE STORE IS IN FLIGHT PER DIRECTION" IS FALSE, AND `uop`/`vl` MAY NOT
       COME FROM `xx_opnd.out`. A younger store's AGEN pulse can land BEFORE an older
       store's DGEN grant, so at the grant `st_opnd.out` names the younger store while
       `iss_st` names the older one — bits and valid drawn from two different
       instructions. Measured on `ms14_stripmine`, whose third iteration overlaps the
       second: `VecDgen accept` fired twice for the same rob_idx, the older store's
       data stream was built from the younger store's `vl` and base, and the older
       store never completed. `vec_lsu_dgen_request_matches_granted_store`
       (spec-lsu.d7) is exactly this property and it fires on the counterexample.
       Take `uop` from `iss_st.bits`, and hold `vl` and `data_base` in tables INDEXED
       BY `stq_idx`, written at each store's own AGEN pulse — a single held register
       has the same defect, one cycle later.

  (k2) A STORE ARRIVES ON `iss_st` TWICE, AND ONLY THE AGEN PASS STARTS AN OP.v HERE.
  `VecStoreDgenPath` offers the address pass and then the data pass as two separate
  issue grants, exclusive on `fu_code`: the AGEN grant carries `FC_AGEN` alone, the
  DGEN grant `FC_DGEN` alone. This container drives `dgen.io.req` off the AGEN pulse
  (it gates `stPresent` on `dgen.io.req.ready`, so dgen is provably free by then), so
  the DGEN grant is bookkeeping for the ISSUE SLOT and carries no work for this
  module. Qualify the descriptor-table write, the branch-kill exclusion and the
  fresh-candidate term with `FC_AGEN`; an unqualified `iss_st.valid` treats the data
  pass as a second OP.v and re-runs the whole address chain — a second operand read, a
  second mask stream, and a second range push for an op whose region already holds one,
  which the element queue then refuses and `VecRangeAgen` reports as a `range.ready`
  violation. That assertion is correct and names the wrong module: the duplicate
  originates here. Invisible until `VecDecode` set `FC_AGEN`/`FC_DGEN`, because before
  that a store never issued at all.

  (j) `st_drained` IS A LEVEL, ONE BIT PER STQ ENTRY, AND THIS MODULE DRIVES IT. Never a
  pulse: a pulse never arrives for a predicted event naming a store that already drained,
  hanging that load forever. Key it on the post-commit WRITE cursor, not the TRANSLATE
  cursor — translate-keyed releases the hold before any byte reaches the cache and the
  released load reads the stale line.

  ---- 1. What this stage is, and where it sits ----

  //@req-spec-agen.a1
  //@req-spec-agen.b2
  The vector LS AGEN stage sits BETWEEN the `IQ_V_LOAD`/`IQ_V_STORE` issue queues
  and the Unified Load/Store Unit: its only inputs from the issue side are
  `iss_ld` and `iss_st`, and its only outputs toward memory are the arbiter's
  `vec_fire` and the LCAM presentation, both through `lsu_vec`. The first vAGEN
  stage is consequently AFTER the issue stage — `ld_opnd`/`st_opnd` are granted by
  those two queues directly, with no intervening buffer.

  //@req-spec-core.c10
  //@req-spec-agen.a2
  An `OP.v` is a SINGLE uOP through decode, rename, the ROB and issue; Caracal
  performs no frontend cracking. The expansion of one `OP.v` into `nOP.v` happens
  INSIDE THIS MODULE AND NOWHERE ELSE — in `ld_elem_agen`/`st_elem_agen` on the
  fill side for the strided/indexed/segmented classes, and in `ld_beat`/`st_beat`
  on the drain side for unit-stride. The cracked accesses reach memory at ELEMENT
  granularity. No other module in the machine may expand a vector memory uop, and
  nothing outside this subtree may read the `nOP.v`-scoped cursor fields of a uop.

  //@req-spec-agen.a3
  Cracking is by EMUL as well as by element: every emitted `nOP.v` carries WHICH
  destination PRN and WHICH BYTE OFFSET WITHIN THAT PRN it reads or writes. The
  member number is `byte_pos >> log2(vLen/8)` and the offset is
  `byte_pos & (vLen/8 - 1)`, both stamped by the agen that emitted the access and
  read back by the LCB (loads) or the store data path. Those two fields, not the
  element index, are what let an out-of-order response be placed, and the group
  they index is `pvdest` normally and `pvtmp` for a segmented access.

  //@req-spec-agen.a6
  This one stage contains the LOAD vAGEN, the STORE vAGEN and the store vDGEN:
  `ld_opnd` + `ld_elem_agen` + `ld_range_agen` are `ld_vAGEN_1`; `st_opnd` +
  `st_elem_agen` + `st_range_agen` are `st_vagen_1`; `dgen` is `st_vdgen` and runs
  ALONGSIDE the store vAGEN, granted separately on `FC_DGEN`. Vector `OP.v`s
  issued by the CII IQ are forwarded to the coprocessor and never enter here.

  ---- 2. Two stages, and the class-to-agen routing ----

  //@req-spec-agen.b1
  //@req-spec-agen.b4
  The vAGEN is SPLIT INTO TWO STAGES, and the split is the reason this module has
  a fill side and a drain side rather than one address generator. Stage 1 — the
  four agen instances — contains ONLY the Skipper and Walker generators, absorbed
  into `VecElemAgen`: in stage 1 only strided, indexed and segmented accesses have
  effective addresses calculated and expanded into `nOP.v` bundles. A unit-stride
  `OP.v` is encoded by `VecRangeAgen` into a single `nOP.v` carrying the effective
  base, the effective stride and `is_unit_stride`, and the stage-2 Packer —
  `VecBeatExpander` — generates its effective accesses JUST IN TIME at the queues.
  There is no Packer on the fill side; putting it there is what made every
  `addvector` access one beat per element.

  //@req-spec-agen.b3
  Load `OP.v`s are issued to `ld_vAGEN_1` and store `OP.v`s to `st_vagen_1`:
  `iss_ld` reaches `ld_opnd` only and `iss_st` reaches `st_opnd` only, and each
  operand-read result is broadcast to that direction's TWO agens, which select on
  the static access class carried by the uop. `VecElemAgen` takes
  `v_is_strided || v_is_indexed || v_is_segment`; `VecRangeAgen` takes
  `v_is_unit_stride || v_is_whole_reg || v_is_mask`. Assert the two selections are
  mutually exclusive and jointly exhaustive over `is_vec && (uses_ldq ||
  uses_stq)`, and assert each instance sees only its own direction.

  THERE IS NO ROUTING MUX IN THIS MODULE AND NO AGEN MAY EXPECT PRE-ROUTING. Both
  agens of a direction observe the SAME broadcast descriptor and each self-selects
  on the access class. That is cheaper than a demux, and it is what execution.rst
  means by "the generator is selected by access class, not by direction"; a
  pre-routing mux here would be a SECOND decision about the same class bits, taken
  in a module that has no business knowing the RVV encoding. Nothing here
  re-decodes `uop.inst` — the class fields come from `VLSDecode` on the uop. Note
  the consequence for `vleff`, which is architecturally UNIT-STRIDE: its `OP.v`
  self-selects into `VecRangeAgen`, not into `VecElemAgen`.

  ---- 3. The mask streamers live at THIS level ----

  `ld_msk` and `st_msk` are instantiated here, not inside the agens, because the
  unit-stride path needs a mask reader (spec-agen.e12, VecRangeAgen has none) and
  `R1`/`R4` may have EXACTLY ONE reader each (spec-vrf.g18). This module therefore
  owns their start interface: on an accepted `OP.v` of either class it drives
  `op`/`op_masked`/`op_vl` from that direction's `xx_opnd.out` (`op_masked` from
  `v_is_masked`, `op_vl` from the resolved `vl`), and routes
  `staged`/`ahead`/`skip_log2`/`all_inactive` to that direction's element agen,
  taking `step`/`skip` back from it, while `us_mask` goes to that direction's
  range agen as the `mask`/`vm` half of its `io.scalar` input.

  A streamer holds ONE `OP.v`'s cursor per direction, so a direction's element
  walk and its range entry cannot be in the streamer at the same time. That makes
  a direction's `OP.v` hand-offs serialize at THIS level, which is why the
  descriptor table of section 3b exists rather than an assertion that it cannot
  happen. For a unit-stride op the streamer loads its latch and retires the next
  cycle, since there is nothing to walk, so its occupancy is one cycle.

  The `R4` MUX IS THIS MODULE'S, AND IT IS THE ONLY BACK-PRESSURE ON THE STORE
  INDEX READ. `R4` serves BOTH the store mask read (`st_msk.mask_rd_req`) and the
  store index read (`idx` inside `st_elem_agen`, passed through the agen's
  `vrf_read_req`/`vrf_read_gnt` pair), and it must reach `VecRegFile` as EXACTLY
  ONE request — `VecRegFile` provides no `ready` on a read port, so the hold-off
  cannot live there. Static priority: the once-per-`OP.v` mask read wins, so
  `st_elem_agen.vrf_read_gnt` is `!st_msk.owns_port` and `VecIdxGen`'s request
  simply waits a cycle. That cannot starve the index read, and the argument is
  structural rather than statistical — the agen may not start an element access
  whose mask bit is not staged, so the first index member cannot be needed before
  the cycle after the mask read completes, and the mask read happens once and never
  contends again. On the load path the two reads are on different ports (`R0`
  index, `R1` mask), `R0` has exactly one reader, and
  `ld_elem_agen.vrf_read_gnt` is constant true.

  ---- 3b. The descriptor pending table: ONE structure, TWO hazards, and the
           amendment to ground rule 6 that admits it ----

  THE `iss` TO `xx_opnd` TO `io.op` CHAIN HAD NO `ready` ANYWHERE, AND THAT LEFT
  TWO INDEPENDENT HOLES WITH ONE CAUSE — no back-pressure on the operand-read to
  agen chain. Both are closed here, by one structure, and it is worth naming them
  separately because they were reported separately and either alone would justify
  the table:

  (i) MID-WALK ARRIVAL. `VecScalarOperandRead` overwrites its stage register every
  cycle and publishes no readiness, so an agen must latch unconditionally; but an
  SSI element walk takes up to `vl` cycles, and `IQ_V_LOAD`/`IQ_V_STORE` grant the
  oldest READY entry with nothing stopping a second grant on the very next cycle.
  A second descriptor can therefore arrive at an agen still walking the first, and
  `VecElemAgen`'s one-deep pending slot covers exactly ONE such arrival. The
  dispatch-time reservation does NOT cover this: it guarantees QUEUE CAPACITY,
  which is a different resource from "the agen can accept a new descriptor
  mid-walk", and conflating the two is what makes the hole easy to miss.

  (ii) INT-RF READ DENIAL, WHICH IS THE ROUTINE CASE AND NOT THE EXOTIC ONE.
  `PartiallyPortedRF` denies reads BY INDEX PRIORITY —
  `ready := PopCount(io.arb_read_reqs.take(i).map(_.valid)) < numPhysicalReadPorts`
  — the vector lanes are appended LAST, and on Medium ~7-9 existing logical
  readers plus five vector lanes contend for FIVE physical ports. So a denial is
  the expected steady-state event, not an exception, while
  `VecScalarOperandRead` was written for "UNARBITRATED ports with no ready line".
  A denied read that is not retried yields a WRONG `base` or `stride` — every
  element address of that `OP.v` off by an unknown amount, with no assertion
  anywhere and no hang to notice.

  THE STRUCTURE. One PENDING DESCRIPTOR TABLE at this level: one row per LDQ entry
  on the load side, one row per STQ entry on the store side, in the same shape as
  `VecQueueReservation`'s table.

  WHAT A ROW HOLDS, ENUMERATED, BECAUSE A BLANKET `MicroOp` COPY PER LSQ ENTRY IS
  REAL AREA FOR FIELDS THIS PATH NEVER READS: `valid`, `br_mask`, the per-lane
  read-grant bits (see the accumulation rule below: the AUTHORITATIVE per-lane
  outstanding state is `VecScalarOperandRead`'s `rr_need`; what a row holds is the
  presented/accepted status this table needs to pick the next row, never a second
  copy of the consumer's progress), and the uop fields the operand read and the agens actually
  consume — `prs1`, `prs2`, `pvl`, `pvm`, `pvs2`, `pvs3`, `pvdest`, `pvtmp`,
  `stale_pvdest`, the `v_*` width/class fields (`v_eew`, `v_idx_eew`, `v_emul`,
  `v_seg_nf`, `v_is_*`, `v_is_masked`), `is_vec`/`is_shared`, `uses_ldq`/
  `uses_stq`, `ldq_idx`/`stq_idx` and `rob_idx`. The presented payload is
  reconstituted into the `MicroOp` shape `xx_opnd` already takes. IF A DOWNSTREAM
  CONSUMER NEEDS A FIELD NOT ON THAT LIST, ADD IT TO THE LIST — a field left
  don't-care in the reconstituted uop is exactly the failure this enumeration
  exists to prevent, and it presents as a mis-routed or mis-widthed access rather
  than as an X.

  ACCEPT IS UNCONDITIONAL. `iss_ld`/`iss_st` are written into the row named by
  that uop's `ldq_idx`/`stq_idx` in the grant cycle, with NO qualification of any
  kind: no `busy`, no ready toward the issue queue, no dropped grant. That is
  sound BY CONSTRUCTION rather than by argument — the row is indexed by an LDQ/STQ
  placeholder the dispatch-time reservation ALREADY guaranteed for this `OP.v`, and
  one placeholder holds one op, so "the table is full when a grant arrives" is not
  a representable state. Assert on a write to an already-valid row: that would mean
  two `OP.v`s claimed one LSQ placeholder, which is a reservation bug, not a
  capacity event. Each row holds `valid`, the granted uop (its `br_mask` included)
  and the per-row presented/accepted status below; nothing else. It does NOT hold a
  per-lane read-progress mirror — that would be two places recording one fact, and
  the consumer's copy is the one the RF handshake actually advances.

  PRESENTATION IS QUALIFIED, AND THAT IS WHERE BOTH HAZARDS ARE ABSORBED. Each
  cycle this module presents AT MOST ONE row per direction, choosing the OLDEST
  valid row with BOOM's existing `IsOlderLSU`/`EntryValidFromAge` against that
  queue's head, and presents it only when BOTH:
    - that direction's MASK STREAMER IS FREE (its cursor retired, or never
      started) — which covers hazard (i), because every `OP.v` of a direction
      passes through that direction's streamer whatever its class, so streamer
      occupancy IS the direction's hand-off gate and no new port on any agen is
      needed to observe it. The term is DERIVED HERE from the `op` this module
      drives and the `done` it receives — one tracking bit per direction —
      because `VecMaskStream` exports no `busy` or `occupied` and must not gain
      one; and
    - that row's INT AND VL READS HAVE BEEN GRANTED — which covers hazard (ii).
      The reads are still ISSUED BY `xx_opnd`, combinationally off the presented
      uop, exactly as it already does off `iss.bits`; what changes is that its
      `ready` toward this table IS "every read lane this descriptor needs has now
      fired", which is NOT the same as "all granted in one cycle" — see the
      accumulation rule immediately below.

      ===> A PARTIAL GRANT ACCUMULATES; IT IS NOT RETRIED IN FULL (decision
           D5, retry model (a)). Lane 0 (`prs1`, the base) sits ahead of lane 1
           (`prs2`, the stride) in `PartiallyPortedRF`'s index priority, so
           "base granted, stride denied" is an ordinary cycle. `prs1` fires and
           STAYS fired; only the lanes still outstanding re-request. The
           per-lane hold lives in `VecScalarOperandRead` (`rr_need` per lane,
           `rr_data` per lane, the address held from `rr_uop`), which is the
           idiom every scalar EU already uses — hold the address until `fire`.
      
      ===> AND THIS TABLE MUST THEREFORE DROP `valid` TOWARD THE OPERAND READ
           ONCE A ROW IS ACCEPTED, rather than leaving it presented. An earlier
           revision said a denied row "simply stays presented and re-requests
           next cycle"; combined with the accumulating hold that is not merely
           redundant, it BREAKS the consumer's own check — it keeps `iss.valid`
           high on a descriptor already latched in `rr_uop`, and
           `VecScalarOperandRead` asserts `!(iss.valid && rr_valid &&
           rr_need.orR)` precisely to catch a second descriptor arriving while
           one is still outstanding. Presentation is a HAND-OFF, not a
           continuous request: present until accepted, then go quiet and let
           the consumer finish its lanes.
      
      WHY ACCUMULATE RATHER THAN RETRY, since retry is the simpler state. The
      vector lanes are appended LAST in the RF's index priority and there are
      ~7-9 existing logical readers against 5 physical ports on Medium, so
      denial is routine, not exceptional. Retry-in-full requires base AND
      stride to win in the SAME cycle against all of that; under sustained
      scalar pressure that is a livelock, not a slow path, and it also
      re-serializes the very read the 3 -> 5 seam widening existed to
      parallelize. Accumulation makes progress MONOTONIC: each lane fires once
      and stays fired, so the worst case is bounded by the unluckiest single
      lane instead of by the coincidence of all of them.
      The "half-read descriptor" objection is real but is a KILL question, not
      a correctness-of-read question, and it is answered where the state lives:
      `VecScalarOperandRead`'s kill clears `rr_need` so a squashed uop stops
      consuming arbitration, and a read has no side effect, so a lane that
      fired for a killed descriptor has simply wasted a port cycle.

  THE GRANT CYCLE IS BYPASSED SO THE TABLE COSTS NO LATENCY IN THE COMMON CASE. A
  grant whose row is being written this cycle is ALSO eligible for presentation
  this cycle, combinationally around the row, whenever both qualifications above
  already hold — which is the steady state, since the streamer is normally free
  and a read is normally granted. The read request is therefore still driven in
  the grant cycle, as it was before this table existed, and the row is consulted
  only when the fast path could not fire. The bypass adds no path that did not
  exist: `VecScalarOperandRead` already drove `int_rf_read_req` combinationally
  from `iss.bits`. Express the row's next state as UNCONDITIONAL WRITE ON GRANT
  followed by CLEAR ON FIRE, in that priority, so a bypassed grant's row is
  written and cleared in the same cycle and never becomes visibly valid. Do NOT
  instead gate the write on "the bypass did not fire": that makes the descriptor's
  survival depend on two combinational decisions agreeing, and the first time they
  disagree — a read denied in the very cycle the bypass looked free — the
  descriptor is gone with no assertion.

  So the link UPSTREAM of `VecScalarOperandRead` — this table to `xx_opnd` — is
  the one that becomes `Decoupled`: the table presents, `xx_opnd` accepts, the row
  clears on the fire. THE DOWNSTREAM CONTRACT IS UNCHANGED AND MUST STAY SO:
  THE LOAD RESPONSE ALIGNMENT TABLE IS THE ONE PLACE A BEAT MUST BE HELD BACK.
  A response carries its request's uop back unchanged, so the DESTINATION —
  `v_split_dst_prn`, `v_split_dst_byte_off`, `ldq_idx` — is read off the response
  itself and needs no side structure. Its `vaddr`/`byte_en`/`eew` are not carried,
  so each in-flight beat reserves one of `ldRespTags` tags, stamps it into the
  request's `uop.v_mem_tag`, and parks that alignment in a table indexed by the tag;
  the response reads the table at the tag it returns with.

  ===> KEY IT BY TAG, NOT BY LANE, AND NOT BY ARRIVAL ORDER. `ll_resp` — the miss
       return — always arrives on lane `lsuWidth-1` whatever lane issued the
       request. A per-lane FIFO replayed in request order therefore aligns a miss
       against a DIFFERENT lane's request the moment `lsuWidth > 1`: on MegaBoom
       each 16-byte beat placed only its upper 8 bytes, at `dst_byte` +8, so every
       group stalled half-filled, `ld_msk.done` never fired, `ldStreamerBusy`
       latched, and every later vector memory op hung behind it. Order-based
       recovery also cannot survive a nack, which returns on a different port
       entirely and would desync the queue permanently.

  Nothing else bounds how many beats are outstanding, so the beat request MUST be
  qualified by tag availability, exactly as it already is by the LCB credit: same
  rule, same reason, a second response-tracking resource that cannot back-pressure
  once the beat has fired. PICK EVERY LANE'S TAG FROM THE BUSY REGISTER ALONE,
  never from another lane's fire in the same cycle, and require `lsuWidth` free
  tags before any lane proceeds: the arbiter's round-robin makes lane 0's grant
  depend on lane 1's request, so a `valid` gated on a same-cycle allocation closes
  a combinational loop.

  ===> PARK THE WHOLE REQUEST, NOT ONLY THE ALIGNMENT, AND REPLAY A NACKED BEAT
       FROM IT. A nack frees nothing: it marks the tag REPLAY-PENDING and keeps
       both the tag and its parked request alive, because the beat expander has
       already advanced its cursor past that beat and will never re-emit it. On
       MegaBoom a nack is STRUCTURAL, not an edge case — the MSHR accepts one
       request per cycle, so two lanes issuing beats of the same line guarantee
       one, and measured on `ms11a2_pure_vle` exactly half the beats (4 of 8) were
       nacked. Drop them and every group stalls half-filled forever.
       A replay OUTRANKS a fresh beat on its lane: it is older, its tag is already
       spent, and its group cannot complete until it lands. Pick replay candidates
       from the pending register alone, for the same loop reason as the tag pick.
       Clear replay-pending when the replayed beat re-fires, and free the tag only
       on a response. Replay the request BIT-IDENTICALLY, `uses_tlb`/`uses_lcam`
       included, rather than suppressing its side channels: re-running them repeats
       work already done, but suppressing them would make a replayed beat and a
       first-issue beat two different requests, and only one of the two paths would
       ever be exercised by a test that does not nack.

  ===> THE LCB CREDIT IS NOT `free_count =/= 0`. VecDcacheArbiter’s spec calls this
       "the exact per-PRN test", and as built it was not: driving it from the LCB's
       coarse `free_count` deadlocks at `EMUL = lcbEntries`, where ONE group owns
       every entry, `free_count` is 0 for the op's whole lifetime, and the beat
       expander therefore stops emitting the very beats that would fill those
       entries. Measured on `ms14_vls_e64_m8`: 8 entries allocated, 7 of 32
       placements, one VRF write, `group_done` never fired. A free entry is needed
       only while the allocation WALK is still running; once it has finished, this
       op's entries exist and its beats need no new one. Do NOT reduce the credit
       to a ready bit here. Export the THREE RAW TERMS to the beat expander —
       `lcb_free_nonzero` (`free_count =/= 0`), `lcb_walk_active`
       (`lcbWalkActive`) and `lcb_walk_rob` (`lcbWalkUop.rob_idx`) — and let it
       form `free_nonzero || !walk_active || beat_rob =/= walk_rob` per path.
       These assignments must sit BESIDE the walk they read — placing them earlier
       in the file is a Scala forward reference that elaborates as a null and
       fails at FIRRTL time, not at compile.

       THE `rob_idx` TERM IS NOT OPTIONAL — it covers the CROSS-OP case, which the
       other two do not and which deadlocks identically. `lcbWalkActive` is ONE
       GLOBAL register, so a LATER op's walk stalling at `free_count = 0` gates
       beat production for EVERY in-flight load, including an EARLIER op whose
       walk already finished and whose entries already exist. That is a circular
       wait: the new walk needs a free entry, entries are freed only when an
       in-flight op completes and RETIRES, and that op's completion needs exactly
       the beats this gate is blocking. Walks are serialised in op order (the
       `ldLcbTrigger && lcbWalkActive` assert guarantees it), so a head whose
       `rob_idx` differs from the walking op's has provably finished allocating and
       can never need a free entry — the term is safe as well as necessary.
       Measured on `axpy-vector` (LMUL=8, e64, `lcbEntries = 2*maxVecMembers`): two
       groups owned all 16 entries, a third op's walk stalled, and the second
       group's cursor froze mid-stream at element 25 of 32, leaving members 6 and 7
       unfilled. Note the shape of the symptom — `group_done` never fires, the
       destination PRNs stay busy, commit stops, and the VECTOR FREE LIST drains
       until rename deadlocks, so what you actually see is an allocation stall in
       a different module tens of thousands of cycles later.

       AND THE COMPARISON IS PER PATH, WHICH IS WHY THE TERMS GO DOWN RAW. The
       expander's unit-stride path reads `us_head` while each SSI lane reads its
       own `ssi_head(i)`, and those are DIFFERENT ops in flight simultaneously. A
       single ready bit reduced here carries one of those identities and answers
       the wrong question for the other — the unit-stride path was fixed first
       against `us_head.rob_idx` alone and the strided (`vlse`) path went on
       starving until the comparison moved into the expander.

  ===> THE WALK IS A ONE-OP RESOURCE AND MUST BE BACK-PRESSURED AT ADMISSION.
       `lcbWalkActive` holds a single op, and the walk STALLS whenever the LCB has
       no free entry, so a second load admitted meanwhile begins a second walk and
       the first's remaining members are simply lost. Gate `ldPresent` — the point
       where a winning load enters the streamer — on `!lcbWalkActive` as well as
       `!ldStreamerBusy`. Nothing is lost by stalling there: only `ldPresent`
       clears the op's `ldRows` entry, so it just waits. Progress is guaranteed
       because the blocking walk completes as soon as an older in-flight op
       retires and frees its entries.

       DO NOT back-pressure by gating `ldMaskFire` instead. For a unit-stride op
       VecMaskStream retires the instant its latch loads
       (`done_unit_stride = fresh_this_cycle && us_now`), so `us_mask.valid` is a
       ONE-CYCLE PULSE — that is what the "launch cycle must be able to fire" note
       protects — and deferring it DROPS the op rather than stalling it, turning a
       stall bug into a silent lost-instruction bug.

       `lcbWalkActive` is defined beside the walk, hundreds of lines after the
       admission point, so declare a `Wire(Bool())` at the admission point and
       connect it at the walk. Referencing the register directly reads a Scala
       null and fails at FIRRTL time, not at compile time.

  ===> THE WALK'S TERMINATION TEST MUST USE THE WIDTH-EXPANDING `+&`. The walk index
       is `log2Ceil(maxVecMembers)` bits and the member count is one bit wider, so at
       `EMUL = maxVecMembers` a truncating `(idx + 1.U) === members` wraps 7 -> 0,
       compares against 8, and NEVER completes. That leaves `lcbWalkActive` high with
       `free_count = 0` forever, which defeats the `|| !lcbWalkActive` escape above
       and reinstates the very deadlock it exists to prevent — every `_m8` test in
       `vset_loadstore_noarith.txt` hung on exactly this, at all four EEWs, while
       `_m1`/`_m2`/`_m4` passed because they never reach the wrap. Write
       `(idx +& 1.U) === members`, and assert the walk never revisits member 0: the
       failure has no signature of its own downstream, it only looks like a hang.

  ===> `active_bytes` IS THE COMPLETION CONTRACT, NOT A CONVENIENCE. The LCB holds an
       entry until every bit of `active_bytes` is covered, so a byte marked active
       that no beat will ever deliver is a HANG, not a wrong value. Allocating it as
       all-ones — "every byte of every member is active" — is therefore only correct
       for an access that fills whole registers; it hung `ms11a4_vle_tail` (vl=2 of
       4: 16 bytes delivered against 32 claimed), `ms11d_vlm` (2 bytes of 32) and
       `ms14_stripmine` (its vl=2 remainder iteration). Compute it per member from
       the SAME three quantities the range length comes from: all bytes for a
       whole-register access; `ceil(vl/8)` bytes in member 0 alone for a mask access,
       whose vtype elements do not describe the transfer at all; otherwise the bytes
       of the elements that are both below `vl` and set in the staged mask, which
       is the per-EEW expansion of that member's slice of the mask. `inactive_bytes`
       is the complement, and `undisturbed` says whether those bytes are preloaded
       from `stale_pvdest` or written as ones — `vta`/`vma` are the only inputs that
       answer it, and neither is visible here unless decode carries them on the uop.

  ===> STORE BEATS NEED THE SAME POOL AND THE SAME REPLAY, IN A SEPARATE INSTANCE.
       A store returns no data, so its tag holds no alignment and exists only to
       name a nacked beat for replay; it is freed by `store_ack` rather than by a
       response. Only a WRITE-pass beat reaches the D$, so the translate pass
       allocates nothing — gate allocation on `uses_dcache`, or a two-pass store
       burns two tags per beat and the pool halves. Measured on `ms11a2_vse` at
       `lsuWidth = 2`: 2 store acks on lane 0 and 2 store NACKS on lane 1, so
       without store replay a 4-element store writes only the dwords lane 0 owned
       and the rest of the range keeps its old contents. Loads hang when a beat is
       dropped, because the group never completes; stores instead COMMIT SILENTLY
       WRONG, because nothing downstream is waiting for the missing write.

  ===> A WRITE-PASS BEAT MAY ONLY FIRE ON LANE 0, AND A STORE REPLAY LIKEWISE.
       `dcache.scala` gates the store acknowledgement with `&& (w == 0).B`
       (`io.lsu.store_ack(w).valid := s2_valid(w) && s2_send_store_ack(w) && (w ==
       0).B`), so a beat that SUCCEEDS on any other lane is never acknowledged: the
       per-lane result is computed inside the D$ and then discarded. Its tag is
       therefore never freed, and after `ldRespTags` such beats the pool starves and
       the write pass spins forever — measured on `ms14_vls_e64_m4` as `busy=247`
       (7 of 8 tags held), `rpy_mask=0`, and 16369 blocked cycles. This mirrors
       `ll_resp` always returning on lane `lsuWidth-1`, and matches BOOM's own
       scalar store path, which commits on pipe 0 only; it is a property of the
       baseline D$ interface, not of this module. Lanes above 0 may still carry
       TRANSLATE-pass beats, which need no acknowledgement. Do not "fix" this by
       ungating `store_ack` in `dcache.scala` without deciding what per-lane store
       acknowledgement means for the scalar STQ, which shares that port.

  ===> A NACKED STORE ALSO SQUASHES THE TWO STORE BEATS BEHIND IT, SILENTLY, AND
       THOSE MUST BE REPLAYED TOO. `dcache.scala` computes `s2_store_failed` from a
       nacked store at s2 and feeds it into BOTH `s1_valid` and `s2_valid` as
       `!(s2_store_failed && ... && uop.uses_stq)`, so the store beats then in s0 and
       s1 are dropped with NEITHER `store_ack` NOR `nack`. The scalar LSU needs no
       notification because its recovery is coarser — a nack rewinds
       `stq_execute_head` to the nacked store and everything behind it re-executes —
       but this module tracks beats individually by tag and learns of a beat's fate
       ONLY from ack or nack. A squashed beat therefore holds its tag forever and its
       bytes never reach memory: the silently-wrong-store class again, and the one the
       `store_ack` lane gate above does not cover. Measured on `ms14_vls_e8_m8`
       (LMUL=8, 32 write beats): 26 acks, 9 nacks, and 6 beats with no response at
       all, `stTagBusy` stuck at 126 — exactly those 6 tags — and `dst` correct for
       144 of 256 bytes.
       So `dcache.scala` must EXPORT `s2_store_failed` (as `io.lsu.store_failed`) and
       the LSU must forward it here. Do NOT infer it from `vec.nack`: the squash is
       not `is_vec`-filtered, so a SCALAR store's nack kills vector beats while this
       module sees no nack of its own. `vec_fire` drives `dmem_req` combinationally,
       which makes the two victims the beat firing in the `store_failed` cycle and the
       beat that fired the cycle before, so register one cycle of fired-tag one-hots
       and mark both for replay. OR that kill set in AFTER the replay-clear terms: a
       victim that was itself a replay already has its bit in `stRpyClr` from firing,
       and the squash has to win or the beat is lost exactly as before. A victim can
       never be acked in the same cycle — an ack lands at s2, two cycles after its
       fire, and both victims are younger than that — so assert the kill set and the
       ack-clear set are disjoint rather than defining a priority between them.
  Assert a response's tag is allocated: with the gate in place that is an
  invariant, and it is the check that catches any later path which drops or
  duplicates a response before the stale alignment it hands the LCB turns into
  silently misplaced load data.
  `xx_opnd.out` to the agens, to the mask streamer's start and to the range agen
  remains a `Valid` that its consumer LATCHES UNCONDITIONALLY. Every agen was
  written against exactly that, and adding a ready there would rebuild the
  queues-to-issue back-pressure path the dispatch-time reservation exists to
  remove. `VecElemAgen`'s own pending slot is then provably never occupied —
  assert it stays free rather than deleting it, so a wiring regression shows up as
  a firing assertion instead of a silently lost descriptor.

  ===> GROUND RULE 6 IS AMENDED TO THREE LEGAL STATE LOCATIONS, AND THIS TABLE IS
       THE THIRD. The rule previously enumerated exactly two homes for in-flight
       vector-LSU state; it now reads: (a) the six `VecElemQueue` instances,
       (b) the LCB's per-PRN assembly entries, and (c) this table. (c) is
       admissible because it is the SAME KIND of state as (a), on all four terms
       the rule cares about: it is scoped to a QUEUE ENTRY and not to "the current
       instruction" (rows for several ops coexist, and a row outlives no
       placeholder); its capacity is RESERVED AT DISPATCH by the same mechanism;
       it is STRUCTURALLY UN-OVERFLOWABLE, so it needs no occupancy answer; and it
       EXPORTS NO `busy`. The amendment is stated in the plan text and restated
       here on purpose — gate H4 reviews against that text, and a reviewer meeting
       a table under a rule that named two homes would be right to reject it.

  ===> AND THE TABLE EXPORTS NOTHING. It does not reach `iq_v_load`/`iq_v_store`,
       it produces no ready, no credit and no occupancy, and a grant is never
       refused because of it. QUALIFYING THE `FC_AGEN` GRANT IS THE REJECTED
       ALTERNATIVE — the tempting one, since it needs no table at all. It is a
       `busy` reaching an issue unit under another name, which is EXACTLY what
       gate H4 greps for, and it is a failed review regardless of what it
       measures. Rows carry `br_mask`, updated by `GetNewBrMask` every cycle, and
       are invalidated by `IsKilledByBranch`/`rob_flush` exactly as an issue slot's
       uop is — the same discipline, no new kill mechanism. A row killed with reads
       outstanding drops its requests in the same cycle; a response returning for
       a killed row is discarded.

  ---- 4. Reservation and the six queues ----

  `resv` is driven from `dis_uops`/`dis_fire` and its `resv_out` regions are
  routed to the six queues' reservation claim ports; its four lookup and release
  lanes are FIXED — 0 `ld_elem_agen`, 1 `st_elem_agen`, 2 `ld_range_agen`,
  3 `st_range_agen` — and a denied release is never retried. `resv`'s own
  occupancy counters are AUTHORITATIVE for the grant; each queue's `avail`/`tail`
  outputs are consumed here only as assertions that the two agree, because two
  sources of truth for free space is how a queue comes to be over-subscribed.

  LOADS UNDER-RESERVE AND STREAM; STORES RESERVE THE WORST CASE. Nothing in this
  module may assume a load's region is big enough for its whole active element
  set. A LOAD's reservation is `min(worstCase, ldResvMembers * vLen/eew)` entries
  with `ldResvMembers` = 4 from VectorParams (decision D9/D10); a STORE still
  reserves the full `worstCase = EMUL * vLen/eew`, because
  `VecQueueReservation`'s four-step deadlock argument depends on it. The
  consequence here is that a load's fill side can run out of region while
  elements remain, and the ONLY thing that makes room is the DRAIN freeing
  entries — `VecElemQueue` does within-region circular reuse and NEVER extends
  past its tail, which is what keeps this deadlock-free: an older load's region
  sits ahead of a younger one's, drains first, and refills into its OWN region, so
  no younger reservation can block it. That is `spec-lsu.b11`'s streaming
  precondition, and it is live only because loads under-reserve.

  ===> SO DO NOT ADD AN ASSERTION THAT A LOAD'S `resv_count` COVERS ITS
       ELEMENT COUNT. It was the natural check while both directions reserved
       the worst case, and it now fires on every long load. The equal-count /
       one-shared-base assertion of the SSI pair is a STORE-SIDE assertion and
       is stated as such below. Squashability is untouched: a load's region is
       still contiguous and program-ordered, just smaller, so section 9's
       rollback arithmetic is unchanged.

  THE ADDRESS AND DATA REGIONS OF A US STORE ARE NOT IN IDENTITY CORRESPONDENCE,
  and this is the seam most likely to be mis-generated. `st_US_ADDR_Q` holds ONE
  range entry per store while `st_US_DATA_Q` holds one full `vLen` entry PER GROUP
  MEMBER, so a US store claims 1 address entry and `v_emul * nf` data entries. The
  reservation therefore carries a base PER QUEUE SLOT (`resv_out` slot 0 address,
  slot 1 data) and the equal-count/one-shared-base assertion holds for the SSI
  pair ONLY. `us_data_base` on the range entry is filled from slot 1's base, and
  the surplus release for a US store trims the two queues by different counts.

  THE ADDRESS AND DATA HALVES OF AN SSI STORE ARE PAIRED BY ABSOLUTE INDEX, NOT BY
  A COUNT EACH SIDE KEEPS. `st_elem_agen.elem_pub` publishes, per surviving
  element, `{q_idx, elem_idx, seg_idx, active, first, last}` where `q_idx` is the
  ABSOLUTE entry index it wrote the address to; this module routes that stream into
  `dgen.io.cursor` and `dgen` WRITES `st_SSI_DATA_Q` AT THAT SAME ABSOLUTE INDEX
  rather than deriving an ordinal from its own emitted count. The two are
  interchangeable only while both sides agree about which elements survive, and the
  whole point of publishing the cursor is that DGEN never re-evaluates the mask: a
  derived ordinal reintroduces exactly the drift `VecDgen`'s own spec warns about,
  where one disagreement shifts every later ordinal on one side and every
  subsequent element writes the wrong address. The identity is sound because
  `resv` claims an SSI store's address and data regions with equal counts and equal
  bases; assert those bases agree, and assert every accepted `elem_pub` beat is
  matched by exactly one data enqueue at the same index.

  THE US STORE'S PER-MEMBER DATA CAPTURE HAS ONE MECHANISM, AND IT IS `VecDgen`'S.
  `st_range_agen.io.st_data` is an AGREEMENT ASSERTION ONLY: this module ties its
  `ready` PERMANENTLY HIGH and uses the port to check that the range agen and
  `dgen` name the same member sequence. IT CARRIES NO DATA PATH — nothing
  downstream consumes it, no queue is written from it, and removing its consumer
  would change no stored byte. `VecDgen`'s own `members_used`-derived sequence is
  the single mechanism and must remain so, because that TOTAL-BYTE bound is the M1
  phantom-member fix: a hardcoded or externally-supplied 8-member sequence streamed
  members past the end of a 1-member op and then stalled. Two mechanisms for one
  member sequence is how they come to disagree, and the disagreement corrupts store
  data rather than hanging.

  The port is declared `DecoupledIO` with a permanently-high `ready`, which is
  the same shape A57 condemned on `io.req`. It is left as declared because this
  file is the side that states the tie-off and the two must agree; narrowing it
  to a `Valid` is a joint VecLsu/VecRangeAgen/VecDgen change, flagged in the
  report, not taken unilaterally here.

  Head-side reclamation is driven from here: on the deallocation of a vector LDQ
  or STQ placeholder, this module presents that index to `resv`, which invalidates
  the row and echoes the region's base and count, and the echo drives each queue's
  region-free command and `resv`'s own occupancy decrement in the SAME cycle. That
  keeps reclamation in program order at the queue head with no new state here —
  the retiring index is the LSU delta's `ldq_head`/`stq_head`, not a counter of
  ours.
  ON THE DEALLOCATION OF A **VECTOR** PLACEHOLDER, AND ONLY THOSE. The LDQ and STQ
  are shared with scalar memory, so most of what `ldq_head`/`stq_head` walk past
  never reserved anything — the bootrom's first `sw` is one, and it deallocates
  before any vector op has dispatched. The index being presented is therefore
  qualified by `resv`'s `retire_row_valid`, which reports whether the row that
  index addresses holds a live reservation; the walk still advances past a scalar
  entry in the cycle it deallocates, it just does not pulse `retire`. Reading the
  bit off `resv` rather than tracking vector-ness here is deliberate: a local copy
  of which LSQ indices hold reservations is a second source of truth, and it drifts
  on exactly the mispredict cycle where `resv`'s own rollback clears a row.

  ---- 5. Drain, and the store's TWO passes ----

  //@req-spec-lsu.j5
  The vector LOAD path is IDENTICAL to the scalar load path apart from exactly
  three things, and no fourth difference may creep in: addresses are drained from
  the dedicated vector queues (per element out of `ld_SSI_ADDR_Q`, or by running
  the stage-2 Packer over the single `ld_US_ADDR_Q` entry), responses are
  coalesced in the LCB, and per-element progress is tracked in the LDQ entry's
  element cursor. Dispatch, the eligibility rules for a queue entry to drain, the
  fire (`will_fire_load_agen_exec`, TLB + D$ + LCAM together on one grant) and
  writeback all reuse the scalar structures unchanged.

  //@req-spec-lsu.j7
  //@req-spec-lsu.j8
  A vector store's EXECUTE stage drains addresses from `st_SSI_ADDR_Q` /
  `st_US_ADDR_Q` and does TLB AND LCAM ONLY: `st_beat`'s request asserts
  `uses_tlb` and `uses_lcam` and CLEARS `uses_dcache` while `is_write_pass` is
  low, the translated physical address is written back into the same entry through
  its translate-pass update port, and the entry is retained. ALL ACTIVE ELEMENT
  ADDRESSES ARE TRANSLATED PRE-COMMIT, so any page or access fault is detected and
  reported precisely BEFORE the store commits; the completion condition of section
  6 is what makes "all" true rather than "as many as were drained".

  A US STORE RUNS TWO PASSES OVER THE SAME RETAINED RANGE ENTRY, and this module
  owns the transition. Pass 1 is pre-commit TLB + LCAM; pass 2, entered when the
  STQ entry's committed flag rises, is TLB + D$. This module RESETS the entry's
  element cursor between the passes and drives `is_write_pass` from the committed
  flag; `st_beat` only ever advances a cursor and holds no notion of commit. THE
  EXECUTE PASS IS NOT A FREEING EVENT: `us_pop` stays low through pass 1, the
  entry's `filled` bit is untouched (a store consume never clears it), and the
  region is reclaimed only by section 4's post-commit reclamation. Assert that
  `is_write_pass` never falls for an entry that has already raised it, and that
  no D$ write is ever requested for an uncommitted STQ entry.

  //@req-spec-lsu.j11
  The vector STORE path is otherwise identical to the scalar store path: the only
  differences are draining address and data from the dedicated vector queues and
  the pre-commit translation of the whole active element range. Commit, the
  `stq_execute_queue` drain and the D$ write itself are BOOM's, unchanged.

  //@req-spec-lsu.k10
  //@req-spec-rob.d13
  Both drains present their beats to `arb`, which reuses THE SCALAR CACHE PORT
  rather than a `VLEN`-wide vector cache interface: one `dmem.req` lane per
  `lsuWidth`, the same interface, the memory subsystem unchanged. Peak vector
  memory bandwidth is therefore `lsuWidth * coreDataBytes` per cycle and this
  module cannot raise it — it exists so the ceiling is shared fairly. A segmented
  store's actual D$ writes are POST-COMMIT exactly as for any store: it takes the
  same two passes, so its coprocessor round trip changes when its data arrives,
  never when its writes are permitted.

  The load drain's LCB credit starts from ONE NUMBER: `lcb.io.free_count`, the
  same quantity `arb` consumes on `io.lcb_free_count` for its coarse suppression.
  Export it to `ld_beat` as `lcb_free_nonzero` (`free_count =/= 0`) alongside
  `lcb_walk_active` and `lcb_walk_rob`, so both consumers rest on the same
  underlying count rather than each computing its own.

  The per-PRN refinement — "a beat landing in an already-allocated entry needs no
  new credit" — is NOT published by the LCB and must not be. It cannot be: it is a
  question about the BEAT (which op does it belong to, and has that op's walk
  finished?), not about the buffer, and the two live in different modules. The LCB
  publishes the count; VecLsu publishes the walk's identity; the expander, which
  is the only module holding the beat's own head, forms the answer.

  ---- 6. Completion: one event per shape, and nothing streamed ----

  //@req-spec-rob.e1
  //@req-spec-rob.e2
  //@req-spec-rob.e3
  A multi-access vector load is NOT memory-safe until ALL of its element addresses
  have disambiguated, so this module reports a SINGLE group-safe event on
  `vec_clr_unsafe` when the LAST element address of that entry has been
  LCAM-checked, and never a per-sub-access clear. The condition is read off the
  entry's own element cursor: the LCAM pass's cursor has reached the entry's active
  element count, and for a US entry the one range-overlap query has been granted.
  No new counter is added — the cursor lives in the LDQ/STQ placeholder, so the
  event is a comparison, not state of ours.

  //@req-spec-rob.e5
  A vector STORE clears `rob_unsafe` on the same shape of event: one group-safe
  when the last of its element addresses has been LCAM-checked, which for a store
  is the completion of the pre-commit translate pass. A shared instruction needs
  only the LSU half's group-safe; the coprocessor half performs no memory access,
  and demanding a second one would deadlock the chain of section 8.

  //@req-spec-rob.c6
  //@req-spec-rob.c7
  //@req-spec-rob.d17
  A NON-SHARED vector store clears its ROB busy bit through a SINGLE
  `lsu_clr_bsy` — it writes no VRF, so it has no group-done to emit — driven on
  the LSU's existing `clr_bsy` port through `lsu_vec` rather than on a new
  completion lane. It is NOT asserted until that store's WHOLE ACTIVE ELEMENT SET
  has translated and disambiguated, i.e. the same instant section 6's store
  group-safe fires; a clear at the first translated element would let the ROB
  retire a store with untranslated elements and forfeit precise exceptions. A
  segmented store's LSU half writes no VRF either and so likewise signals
  `lsu_clr_bsy`, with the deferral below.

  //@req-spec-rob.c9
  A SHARED store's LSU half DEFERS ITS COMPLETION UNTIL AFTER DGEN HAS READ
  `pvtmp` — not to the translate/disambiguate point that governs a non-shared
  store. Concretely, `lsu_clr_bsy` for an `is_shared` store fires on `dgen`'s
  `last` push into `st_SSI_DATA_Q` for that `stq_idx`, which is step 6 of the
  chain. Signalling at translate would claim the half is done while it has still
  to obtain its store data from the coprocessor, and would invert the fixed
  producer-then-consumer completion order the ROB's one-bit "other half pending"
  flag relies on. Assert that a shared store's clear never precedes its DGEN
  stream's last push, and that a non-shared store's never waits for one.

  A load's completion is the LCB's single group-done on lane 0 of `vec_clr_bsy`,
  and a no-execution op's is `gcopy`'s on lane 2. Nothing here streams a per-PRN
  writeback into the ROB and no per-entry completion counter exists anywhere in
  this subtree.

  ---- 6b. `vleff`: the fault interface belongs to the RANGE agen, and this
            module is the only place its three signals meet ----

  THE FAULT-ONLY-FIRST POLICY IS `VecRangeAgen`'S, NOT `VecElemAgen`'S, AND THAT
  RE-ALLOCATION CHANGES THE WIRING HERE. `vle<eew>ff.v` is architecturally
  UNIT-STRIDE, so its `OP.v` self-selects into `VecRangeAgen` (section 2), and
  `spec-lsu.g1/g2/g3/g4/g9/g10` are allocated there. The three signals this module
  must route, all combinational and all on the LOAD instance only (a store has no
  fault-only-first form):

  - `ld_range_agen.io.fault` (INPUT to the agen) — the DRAIN-side fault report,
    raised against the RETAINED RANGE ENTRY, carrying
    {`elem_idx`, `is_ff`, `rob_idx`, `ldq_idx`} read out of that entry. Routing it
    off the entry rather than off a latch is what keeps `VecRangeAgen` free of
    per-instruction state: the entry is what remembers the instruction. This
    module QUALIFIES it against the walking `rob_idx` exactly as it qualifies the
    element agens' fault reports — same comparison, same kill terms — so a fault
    for a squashed op cannot classify against a live one.

    ===> AND THIS MODULE FORMS THAT REPORT ITSELF; `ld_beat` DOES NOT RAISE IT.
         Earlier text here said "raised by `ld_beat`", and that is not
         implementable: a beat's fault is a TLB exception returned on
         `xlate_resp` ONE CYCLE AFTER the arbiter granted the beat, and
         `VecBeatExpander` sees neither `xlate_resp` nor any other fault input —
         it has a `stop` INPUT and no fault OUTPUT, by its own ports section. So
         the report is built HERE, from the exception shadow this module already
         keeps for `vec_xcpt` (per-lane {valid, uop, vaddr}, registered on a
         granted `uses_tlb` beat, matched against `xlate_resp(w).xcpt_valid`):
           `valid`    = that same exception hit, restricted to the LOAD side and
                        to the unit-stride path (`lcam_range_len` class), so an
                        SSI fault does not reach the range agen;
           `elem_idx` = `(shadow.vaddr - retained_range.base) >> eew`, the
                        faulting beat's element index. Both addresses are
                        VIRTUAL on the load side (`ld_US_ADDR_Q` has no translate
                        pass, so its `base` is never overwritten with a paddr),
                        so the subtraction is the architecturally meaningful one
                        and stays correct across a page boundary — which is the
                        boundary a `vleff` exists to fault on;
           `is_ff`, `rob_idx`, `ldq_idx` = read off the retained range entry, per
                        the paragraph above.
         Deriving `elem_idx` from the address rather than adding an element index
         to `VecMemAccess` is deliberate: the beat is already the unit of the
         cursor, the arithmetic is exact for a unit-stride range by construction,
         and widening the drain-side bundle for one classification would put the
         index on every beat of every op.

  - `ld_beat.io.stop` — asserted by the same qualified fault hit, for the same
    cycle onward, so the cursor stops and no beat past the faulting element is
    requested. Without it the trim is meaningless: elements above `elem_idx`
    would keep issuing and landing in the LCB, and the trimmed VL would describe
    a group that had already been written past. This is the input the "Faults"
    paragraph of `VecBeatExpander` is written against, and it currently has no
    driver.

    ===> AND `stop` IS A LEVEL WHILE THE FAULT HIT IS A ONE-CYCLE PULSE, so it
         needs a latch and the latch must be sized honestly. Keep ONE register
         {valid, `rob_idx`}, set by the qualified fault hit and cleared by the
         US load's `us_pop` or by `squash`/`kill`, and drive
         `stop = faulted.valid && faulted.rob_idx === us_head.rob_idx`. One entry
         is not an approximation: `io.stop` gates `usGate` alone, which serves the
         SINGLE retained unit-stride head, so there is never a second faulted US
         load to track. This is state scoped to the QUEUE HEAD, not to an
         instruction — the same shape as the exception shadow and the LCB walk
         registers already in this file — and §5 rule 6's prohibition is on a
         module owning per-op state that an ISSUE UNIT then waits on, which this
         drives nothing of. A pulse-only `stop` looks correct in the TRAP case
         (the ROB flush arrives within a few cycles and `kill` finishes the job)
         and is wrong in exactly the case the mechanism exists for: a `vleff`
         trim raises no flush, so the cursor would resume on the next cycle and
         run past the fault.

  - A TRIMMED `vleff` RETIRES ITS RANGE ENTRY HERE, and nothing else can do it.
    `VecBeatExpander.io.us_pop` requires `usAnyAdvance`, which requires `usGate`,
    which requires `!io.stop` — so an entry this module has STOPPED can never pop
    itself. For a TRAP that is correct and invisible: the ROB flush squashes the
    op and `kill` retires the entry. For a TRIM it is a HANG, and a silent one:
    a `vleff` trimmed at element `i > 0` raises no trap, the instruction is
    architecturally complete at the trimmed VL, and its entry has no further beats
    to give — but `us_pop` never fires, so `ldUsDrainPtr` never advances, the
    queue entry is never consumed, its reservation is never released, and the next
    unit-stride op never stages. `VecBeatExpander`'s own "Faults" paragraph is
    silent on who retires a stopped entry, which is how this was missed.
    So the retire event is `us_pop OR ff_trim`, and it drives all three of the
    cursor reset, the `ldUsDrainPtr` advance and `ld_US_ADDR_Q.consume` — this
    module already owns that pointer, so no port moves.
    The faulted-head latch must be cleared by that same retire event with
    PRIORITY OVER ITS OWN SET, because `ff_trim` is combinational from `io.fault`
    and therefore fires in the SAME cycle as the fault: an `elsewhen`'d clear
    leaves the latch holding a retired op's `rob_idx`.
    And for the same reason `stop` is driven combinationally from the fault hit as
    well as from the latch. The latch alone asserts a cycle late, which is one
    more beat past the fault, and it would make the `ff_trim => stop` assertion
    below fire on every correct trim.

  - `vec_xcpt.valid` is QUALIFIED BY `fault_trap`, not merely accompanied by it.
    A `vleff` faulting at element `i > 0` must NOT trap; if `vec_xcpt` fires
    generically off `xlate_resp` while `ff_trim` also fires, the machine both
    traps and trims, and the trap wins — which is the architectural violation the
    form exists to prevent, silently, on exactly the `strlen` loop it is for. So
    on the unit-stride load path `vec_xcpt.valid` takes `fault_trap`; every other
    path keeps the generic derivation unchanged.
  - `ld_range_agen.io.fault_trap` (output) — an element-0 fault. Becomes
    `vec_xcpt` with `vstart = 0` and no element index.
  - `ld_range_agen.io.ff_trim` (output) — an element-`i > 0` fault. Goes to
    `lcb.io.trim`, and NOWHERE ELSE.

  ===> AND THE ELEMENT AGENS ARE NOT ON THIS PATH. `VecElemAgen` asserts
       `!v_is_ff` on accept and keeps its own `ff_trim` port for symmetry with
       an assertion that it never fires; DO NOT ROUTE IT. Earlier text here
       accepted `fault_trap`/`ff_trim` from EITHER agen "so neither path is a
       special case" — that predates the re-allocation, and it left the
       unit-stride path (the only path a `vleff` can take) with no fault route
       at all while wiring one for a form that cannot be encoded.

  THE `vleff` VL WRITE STILL HAS EXACTLY ONE DRIVER, and settling that is this
  module's call because it is the only place both candidate producers are visible.
  The trimmed element count reaches `lcb.io.trim` — this module converts the trim
  ELEMENT INDEX into the LCB's {member, `keep_bytes`} form using `v_eew` — and the
  VL register file's `W_lsu` port is driven ONLY by `lcb.io.vl_wb`, on the
  group-done. Routing an agen's `ff_trim` straight to `W_lsu` as well would put TWO
  PRODUCERS on a statically partitioned, never-arbitrated write port, and it would
  publish a trimmed VL — waking every `pvl` dependent — before the group that VL
  describes had been assembled.

  TRACE AND ASSERT THIS PATH HEAVILY, because every one of its outcomes is
  invisible in a passing run and none of them has a natural failure signature. One
  guarded `VecTrace` line per formed fault report (`rob_idx`, `ldq_idx`,
  `elem_idx`, `is_ff`, the shadow's `vaddr`, the range's `base` and `eew`), one per
  `fault_trap`, one per `ff_trim` (with the derived `member`/`keep_bytes`), and one
  per fault hit that was DROPPED by the `rob_idx` qualification — that last one is
  how a squashed-op fault mis-attributed to a live op is caught, and it is the only
  outcome with no downstream effect to observe. Assertions: `is_ff` implies
  `is_unit_stride` on the retained entry — QUALIFIED ON THAT ENTRY BEING STAGED,
  because the staged bits are raw queue read data and mean nothing while the
  queue is empty, so the unqualified form fires at time zero on every test that
  never issues a unit-stride load, which is most of them; `fault_trap` and
  `ff_trim` never valid in
  the same cycle; `elem_idx` strictly below the entry's element count
  (`len >> eew`), which fires if the address arithmetic above ever underflows or
  the entry is the wrong one; and `ff_trim` valid implies `ld_beat.io.stop` in the
  same cycle, which is the coupling a later edit is most likely to break.

  `VecRangeEntry` MUST CARRY `is_ff`. The drain side raises its fault against the
  entry and has no other way to tell a fault-only-first load from an ordinary one:
  without the field, an element-`i > 0` fault on a `vleff` traps, which is the
  exact architectural violation the form exists to prevent, and it fails on the
  `strlen`-shaped loop that is the feature's whole reason for existing. The field
  is being added to `VecBundles` alongside `stride`, `is_unit_stride`, the active
  byte mask, `us_data_base` and `members`; this module reads it only to route, and
  asserts `is_unit_stride` on every entry with `is_ff` set.

  ---- 7. VecGroupCopy's launch, and the R2/W0 mux ----

  A launch is presented to `gcopy` when, and only when, the granted `OP.v` is on
  the LOAD side, has a vector destination, is NOT `v_is_whole_reg`, AND WILL
  GENERATE NO MEMORY ACCESS AT ALL — `vl_zero` (taken from `ld_opnd.out`) or
  `all_inactive` (from `ld_msk`). Neither value is re-derived here.
  ===> THOSE TWO ARE LAUNCH TERMS, NOT MERELY PAYLOAD FIELDS. This paragraph used
  to name them only as values to pass through, and the RTL implemented exactly
  that: every non-whole-register load launched a copy. `VecGroupCopy` is the
  completion producer for the no-execution case ONLY — it is the one writer that
  can finish a group without a D$ request — so launching it for an ordinary load
  makes it a SECOND completion producer racing the LCB. With `ta`/`ma` its
  `must_preserve` is false, so it fires `group_done` immediately at launch, clears
  the ROB busy bit, and the load COMMITS BEFORE ITS OWN D$ ACCESS ISSUES; the
  scalar LSU catches it one instruction later as "trying to commit an un-executed
  load entry", which reads as an LSU bug and is not one. With `tu`/`mu` it is worse
  and silent: `must_preserve` is true, so a real copy of the stale group is written
  over the destination the in-flight load is also writing.
  The launch condition is therefore the EXACT COMPLEMENT of the LCB trigger under
  `ldMaskFire`, and the two must be read as a pair — every granted load reaches
  exactly one completion producer, never both and never neither.
  The whole-register exclusion is a correctness term, not a filter: a `vl1re*` with
  `vl = 0` still transfers its full length from memory, so it is not a
  no-execution op at all, and a copy launched for it would race a real load for the
  same PRNs. A store never launches — it has no VRF destination — and for a shared
  load the group completed is `pvtmp`, which needs no data copy.

  THE STRICT-PRIORITY MUX OVER `R2` AND `W0` LIVES INSIDE `gcopy`. Both
  `VecGroupCopy` and `VecRegFile` declared it, and it is settled by REQUIREMENT
  ALLOCATION: `spec-lsu.m13` (the Load Unit's VRF ports are arbitrated by a
  strict-priority mux) and `spec-lsu.m14` (an active load drain always wins) are
  allocated to `VecGroupCopy`, while `VecRegFile` holds only `m6`/`m7` (the copy
  reuses the Load Unit's ports and adds none). The owner of the obligation owns the
  logic. So this module wires `lcb.io.stale_req`/`stale_resp`/`vrf_write(0)`
  THROUGH `gcopy` combinationally, and `gcopy.vrf_r2_req`/`vrf_r2_data`/`vrf_w0`
  are what reach `VecRegFile`, which therefore sees exactly ONE `R2` reader and ONE
  `W0` writer and must carry no `gcopy` port and no grant logic. The LCB is slot 0,
  drives unconditionally and is NEVER TOLD IT LOST — there is no signal by which it
  could be — while `gcopy` qualifies each of its two requests independently on the
  LCB's valid being low. Assert that a cycle in which the LCB requests `R2` or `W0`
  never carries a `gcopy` request on that same port. At `lsuWidth = 2` the LCB's
  second write reaches `W1` directly; `gcopy` uses `W0` only, since its rate is
  bounded by the single `R2` read.

  ---- 8. Segmented (shared) load/store: two halves, one pvtmp ----

  //@req-spec-lsu.l7
  //@req-spec-lsu.l8
  //@req-spec-lsu.l9
  A segmented load or store is marked `is_shared` and SPLIT INTO TWO HALVES: this
  module executes the half that MOVES THE DATA TO OR FROM MEMORY, and the
  coprocessor executes the half that transposes it. The two HAND OFF THROUGH
  `pvtmp` — the rendezvous group named by the `pvtmp` field of the one shared uop,
  with no table and no temp register file anywhere — and the handoff is a real VRF
  group in both directions: the producer writes it, its group-done wakes the
  consumer's issue slot. Nothing here consults `pvs3` for a shared op and nothing
  here allocates or frees `pvtmp`.

  //@req-spec-lsu.l1
  For a segmented LOAD the `pvtmp` group is the DESTINATION of the LSU half: the
  element agen stamps `pvtmp` members as the placement group on every emitted
  `nOP.v`, the LCB assembles them as an ordinary vector destination, and its
  group-done is what wakes the coprocessor half. `pvdest` correctness for that
  instruction belongs to the coprocessor half under its own `vta`/`vma` policy.
  For a segmented STORE the direction reverses and `pvtmp` is the SOURCE `dgen`
  reads, which is why `dgen_operand` is `Mux(is_shared, pvtmp, pvs3)`.

  //@req-spec-issue.d2
  //@req-spec-issue.d4
  The LSU half of a segmented store runs AGEN AS THE FIRST STEP of the six-step
  chain, and its group-safe is the SECOND: `st_opnd` + `st_elem_agen` source every
  operand of the address path — the base GPR, the index vector and the mask — from
  instructions OLDER than the store, so step 1 is unblockable; the group-safe of
  section 6 then clears `rob_unsafe`, the PNR advances past the entry, the
  coprocessor half becomes eligible and writes `pvtmp`, and only then does step 6
  — `dgen` reading `pvtmp` on a separate, later `FC_DGEN` grant — run. AGEN and
  DGEN of the same slot are independently granted and may be hundreds of cycles
  apart; between them this module holds nothing for that store beyond its retained
  queue region. That is why the chain is a long serial dependency and not a cycle.

  ---- 9. Kill, and the rollback index convention ----

  `squash` is THE RESOLVER of every kill in this subtree. This module routes
  `brupdate`, `rob_flush`, `rob_flush_kill` and the four LSQ pointers into it,
  pairs `kill_uop`/`kill` with the five clients in the fixed order of the
  parameters section, fans `q_squash` out to the six queues in enumeration order,
  drives `resv.rollback` from `squash.resv_rollback` and returns
  `resv.rollback_tail` to `squash.resv_rollback_tail` in the same cycle, and
  routes `squash.kill_ldq` to `lcb.io.kill_ldq` and `squash.resv_rollback.ldq_idx`
  to `gcopy.squash`. Nothing here computes a kill locally, so `spec-lsu.i6` keeps
  its owner.

  THE ROLLBACK INDEX IS BOOM'S EXCLUSIVE TAIL. This is the highest-risk seam in
  the subtree and it is settled here: the `ldq_idx`/`stq_idx` on `resv.rollback`
  names the FIRST DEAD entry, so entries `[head, idx)` survive and
  `[idx, old_tail)` die, exactly as `brupdate.b2.uop.ldq_idx` is the value
  `ldq_tail` had when the branch dispatched. Each queue's new tail is therefore
  `base + count` of the reservation row owned by the YOUNGEST LSQ entry STRICTLY
  OLDER than the driven index, or that queue's head if no such row exists — NOT
  the row of the driven index itself. Reading it as an inclusive survivor keeps one
  killed instruction's entries alive, and the entries it keeps are the ones whose
  destination PRNs have just been returned to the free list. Assert, on every
  rollback, that no surviving row's `base + count` exceeds the new tail and that
  the driven index's own row is invalidated.

  ---- 10. Memory ordering: three mechanisms, one match ----

  `snoop` presents vector addresses to the LCAM and searches loads against the
  vector store address queues; `fwd` decides and performs LD->ST forwarding out of
  the store DATA queues and publishes `known_overlap`; `hold` consumes that same
  classification and decides which younger load waits. This module routes
  `fwd.io.known_overlap` into `hold.known_overlap` and does NOT let `hold` build a
  second overlap test — the forwarding and hold predicates must stay exact
  complements, and one match is what keeps them so. `hold.hold_ldq`
  (`UInt(numLdqEntries.W)`, registered, bit-indexed by the real LDQ index) goes to
  `arb.io.hold_ldq`, which owns the suppression; the hold decides who, the arbiter
  decides what a hold does.

  THE SHARED QUEUE READ PORT IS MUXED HERE, WITH SNOOP WINNING. Both `snoop` and
  `fwd` consume the one read port each store queue reserves beyond its drain lanes
  (`readPorts = ports + 1`). Strict priority to `snoop`: its read is tied to an
  LCAM/TLB grant the arbiter has ALREADY made this cycle, so deferring it would
  strand a grant the arbiter cannot retract, whereas `fwd` has a declared replay
  path — the loser asserts `io.replay` and the load retries, costing one replay and
  no data. `snoop`'s per-candidate `data_filled` gate is satisfied by the same
  muxed read of the paired data queue, so a store candidate that loses the mux is
  simply not presented that cycle and the "no address without its data" gate stays
  sound. Assert that a granted LCAM lane is never left unpresented because of this
  mux.

  `st_drained` IS A LEVEL, ONE BIT PER STQ ENTRY, AND MUST NOT BE A PULSE. This
  module drives it: `st_drained(i)` is ASSERTED whenever entry `i` is not a live
  vector store with active elements still to write — invalid, squashed, scalar,
  never-allocated and fully-drained entries all read asserted — and DEASSERTS only
  while `i` holds a live vector store whose element cursor has not completed its
  active element set ON THE POST-COMMIT WRITE PASS. Two failure modes are being
  avoided at once. A pulse would never arrive for a predicted event naming a store
  that had already drained, hanging that load forever. And keying it on the
  TRANSLATE cursor instead of the WRITE cursor would release the hold before any
  byte reached the cache, so the released load would read the stale line and the
  hold would have added latency and nothing else. Derive it from the write-pass
  cursor and the committed flag only.

  ---- 11. `vec_lsu_empty`, and why it is not a busy ----

  `vec_lsu_empty` is the AND of the four ADDRESS queues' `io.empty`, both DATA
  queues' `io.empty` and `lcb.io.empty`. It leaves on `lsu_fencei_rdy_vec` and
  folds into the LSU delta's
  `io.core.fencei_rdy := !stq_nonempty && io.dmem.ordered && vec_lsu_empty`,
  replacing the old head-side handshake that deadlocks on younger vector stores.
  It is NOT a `busy`: it is not scoped to an instruction, it names no `rob_idx`,
  and its consumer is a `fence.i` waiting at DISPATCH with the ROB already
  draining.

  `gcopy`'s PENDING WORK LIST IS DELIBERATELY EXCLUDED, AND THE EXCLUSION IS
  CONFIRMED RATHER THAN AN OVERSIGHT — it is stated here precisely so it does not
  read as one. A pending `VecGroupCopy` is a VRF WRITE, not memory state: the copy
  performs no D$ access, occupies no element queue, holds no LCB entry and issues
  no TLB or LCAM request, so there is nothing about it for a memory fence to
  order. `fence`/`fence.i` order memory operations against each other, and the
  architectural visibility of a vector register write is ordered by the ROB and
  the group-done/wakeup networks, which the copy already participates in — its
  `vec_clr_bsy` lane 2 is what makes its instruction retire. Folding it into
  `vec_lsu_empty` would therefore not add an ordering guarantee; it would only
  delay `fencei_rdy` behind a VRF write that no consumer of the fence can observe,
  and it would put a non-memory event in a term whose whole definition is "the
  vector memory datapath is empty". Assert the negative form instead: a cycle in
  which `vec_lsu_empty` is high may still have `gcopy` work outstanding, and that
  is legal.

  ---- 12. Trace and assertions ----

  There are no unit tests in this project; validation is end-to-end VCS plus
  Whisper cosim. Emit guarded `VecTrace` lines, gated on the `vecTrace` plusarg
  and `!reset`, off by default, each tagged with this module's name and `rob_idx`:
  one per accepted `OP.v` naming WHICH agen took it and why (the class flags), one
  per store pass transition (`stq_idx`, pass number, cursor reset), one per
  completion event emitted (group-safe, `lsu_clr_bsy`, which lane of
  `vec_clr_bsy`), one per shared-port mux loss, one per `st_drained`
  transition, and — for section 3b — one per row write (direction, `ldq_idx`/
  `stq_idx`, bypassed or not), one per cycle a presentation is WITHHELD naming
  WHICH qualification was missing (streamer busy, or which read lane was denied),
  and one per fire. That "why was it withheld" line is the whole diagnosis of an
  operand-read stall, and it distinguishes an INT-port shortfall from a streamer
  serialization without a waveform. The routing lines are the ones that matter here: a vector memory op
  that goes to the wrong agen, or a completion that is never emitted, is otherwise
  invisible in a log, and this module's own bugs are all wiring bugs.

  Whole-subtree assertions this module owns because only it can see both sides:
  the two agen selections are exclusive and exhaustive; `resv`'s occupancy agrees
  with every queue's `avail`; a store's address and data regions have equal counts
  for the SSI class and the documented 1-to-`v_emul` ratio for the US class (a
  STORE-side check only — a load's region may legitimately be smaller than its
  element count, per section 4); section 3b's table is never written to an already
  valid row, never presents while its direction's streamer is busy or its reads are
  ungranted, and `VecElemAgen`'s own pending slot is always free on an accept; a
  drain-side fault report reaches `ld_range_agen` only while its `rob_idx` matches
  the live unit-stride load, `VecElemAgen.ff_trim` NEVER fires, and every range
  entry with `is_ff` set also has `is_unit_stride`; `st_range_agen.io.st_data`
  agrees with `dgen`'s member sequence on every beat; no
  output of this module reaches an issue unit (a review obligation, checked by
  grep gate H4, stated here so the generated file carries the reminder).
  <|end_logic|>

<|end_module|>

<|begin_perf|>
This module adds NO pipeline stage of its own. Every path through it is either a
direct connection or a mux described above, and the two muxes it owns — `R4` and
the shared queue read port — are single-level static or strict-priority
selections. A registered stage inserted here would land in the middle of one of
the children's own one-cycle contracts (the operand read's grant-to-result, the
beat expander's cursor loop, the forward's search-to-response) and break it.

Section 3b's table is the one structure that HOLDS state, and it is deliberately
not a stage: its grant-cycle bypass means an `OP.v` whose streamer is free and
whose INT/VL reads are granted reaches `xx_opnd` in the grant cycle exactly as it
did before the table existed, so the table costs latency only in the cycles it is
absorbing a hazard that would otherwise have been a lost descriptor or a wrong
base. Two paths to watch, and both are one level: the oldest-valid-row select
(`numLdqEntries` / `numStqEntries` wide, using BOOM's existing age comparators,
and the same shape as `resv`'s own select), and the AND of the streamer-free and
read-granted terms, whose inputs must be flop reads plus the RF's `ready`. Area:
`numLdqEntries + numStqEntries` rows of the ENUMERATED field set, not of a blanket
`MicroOp` — small beside the three 512-entry queues, and the enumeration is what
keeps it so.

Concurrency is the target, and it is structural rather than tuned. A `vle` and a
`vse` must overlap with neither's issue gated by the other (target P4): the two
directions share no operand read, no agen, no mask streamer, no beat expander and
no queue, and they meet only at `arb`, where they are two independent requestors.
Any cycle in which a load grant suppresses a store grant outside the arbiter is a
regression against the whole point of the decomposition.

Throughput obligations passed down: one `OP.v` accepted per direction per cycle;
one element access emitted per cycle per fill-side agen; `lsuWidth` beats granted
per cycle on the drain side; sustained at least one D$ access per cycle per
granted lane while a queue is non-empty (P2); and the unit-stride access count at
the floor `ceil(active_bytes / coreDataBytes)` independent of SEW (P1). This
module's contribution to all five is to add no serialization between them.

Critical paths it must not lengthen: `brupdate.b2` into the six rollback commands
(one comparator level and one mask; the rollback tails arrive precomputed from
`resv`), and the arbiter's grant expression, whose suppression terms must be flop
reads. Area is dominated by the children — three 512-entry SSI queues, the LCB's
assembly entries, the snoop's summary — and this module adds only wiring plus the
assertion logic of section 12.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the dispatch, issue and commit uops, and the `nOP.v`-scoped cursor
fields nothing outside this subtree may read. This module writes NO uop field and
adds none.
VecBundles — `VecElemAccess`, `VecRangeEntry`, `VecReservation`, `VecGroupDone`,
`VecException` and the normative six-queue enumeration, which is the only legal
source of a queue name here. `VecRangeEntry` must carry `is_ff` (section 6b) along
with `stride`, `is_unit_stride`, the active byte mask, `us_data_base` and
`members`; the drain side cannot classify a fault-only-first load without it.
VectorParams — every depth and width passed down, including `ldResvMembers` (the
load reservation quantum, default 4) which reaches `resv`, not this module's own
logic. `VecLsuCoreIO` is NOT declared here: it is declared in `lsu.scala` beside
`LSUCoreIO` and bound by name (see the ports section), and this module declares no
bundle of its own at all.
VecTrace — the guarded trace helpers.

Binds to BOOM's existing `BrUpdateInfo`, `IsKilledByBranch`, `GetNewBrMask`,
`EntryValidFromAge`, `GetRealLSQIdx` and `SelectFirstN` rather than reimplementing
any of them, and introduces no new speculation, wakeup, replay or recovery
mechanism. The only two new networks in the design are VL (`pvl`) and VECTOR
(group-done), and neither originates here.

Instantiates seventeen children, listed in the header. Its counterparties outside
the subtree, and what each owes this seam:
  VecPipeline — instantiates this module as `vlsu`; supplies the dispatch, issue,
    recovery, commit and register-file halves of `vec_pipeline_io`.
  VecRegFile — the seven ports cited by number, and two settled points.
    (a) LATENCY: every read port is ONE REGISTERED CYCLE at the `VecRegFile`
    boundary — request in cycle t, `read_data` in t+1, with the output flop
    instantiated in `VecRegFile`, one per read port. `VecRegFileBank`'s
    "0 cycles, may not be pipelined" is BANK-INTERNAL and sits inside that
    envelope; it must not be propagated outward. This module and every consumer
    below it — `VecIdxGen`, `VecMaskStream`, `VecDgen` on `R3`, the LCB on `R2` —
    are written against exactly that, so the element and beat pipelines are
    correct as specified.
    (b) ARBITRATION: `VecRegFile` sees exactly one requester per port and must add
    none, because the `R2`/`W0` mux is `gcopy`'s and the `R4` mux is this module's.
    Its `gcopy_r2`/`gcopy_w0` ports and their grant logic are a required DELETION.
  VlRegFile — one `W_lsu` write, driven only by `lcb.io.vl_wb`, and two reads, one
    per operand-read instance.
  LSU (edit_existing, src/main/scala/v4/lsu/lsu.scala) — the `VecLsuCoreIO`
    DECLARER and this module's peer on it: the LDQ/STQ placeholders and their
    element cursors, the LCAM/TLB/D$ resource claim and fire,
    `ldst_addr_matches`, the four LSQ pointers, the committed flag, `clr_bsy`, and
    folding `vec_lsu_empty` into `fencei_rdy`. This module binds to that
    declaration and adds none of its own.
  RegisterFile / `PartiallyPortedRF` (host) — the four `Decoupled` INT read lanes
    whose `ready` is index-priority and routinely low, consumed by section 3b's
    table and by nothing else.
  VecSquashUnit (`squash`) — five kill clients in the fixed order of the
    parameters section; its own `nKillClients` default of 8 and its client list
    are stale after the mask-streamer hoist, and this file is authoritative.
  Rob — consumes `vec_clr_bsy` lanes 0 and 2, `lsu_clr_bsy`, `vec_clr_unsafe` and
    `vec_xcpt`, each as an ordinary single-shot event with no vector special case.
  VecIssueUnit (`iq_v_load`, `iq_v_store`) — grants `iss_ld`/`iss_st`. It receives
    NOTHING back from this module: no ready, no credit, no busy.
<|end_dependencies|>

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
  LSU — DELTA SPEC. This file is NOT a description of BOOM's load/store unit. It
  describes only what Caracal ADDS to the existing hand-written
  src/main/scala/v4/lsu/lsu.scala (`class LSU`, `class LSUCoreIO`, `LDQEntry`,
  `STQEntry`), which stays in place as baseline BOOM v4.
*/

  hierarchy.yaml: kind: module, mode: edit_existing,
  target src/main/scala/v4/lsu/lsu.scala. No `output:` — the pre-existing file is
  the artifact. group host. depends_on MicroOp, VecBundles, VectorParams.
  `instantiates:` is EMPTY: this edit adds no submodule. `VecLsu` and everything
  under it is instantiated by `VecPipeline` and reaches this file only across the
  `VecLsuCoreIO` tap. Budget: ~350 added lines (plan section 11), the
  second-largest delta in the design.

  Everything already in lsu.scala is unchanged and is NOT restated: the LDQ/STQ
  register arrays and their `ldq_read`/`stq_read` accessors, `retry_queue`,
  `stq_execute_queue`, the hellacache state machine, the `dtlb` instance,
  `wakeupArbs`, the branch-kill loops, and the helper objects at the bottom of
  the file. Anything this file does not mention keeps its current declaration,
  timing and meaning exactly.

  ===> A VECTOR LOAD HOLDS EXACTLY ONE LDQ ENTRY AND A VECTOR STORE EXACTLY ONE
       STQ ENTRY, AND THAT ENTRY IS A PLACEHOLDER. It carries the program-age
       stamp, the ordering identity and the commit hook, and NONE of the op's
       per-element addresses or data. The cracked element accesses live in
       VecLsu's six element queues and NEVER in an LDQ/STQ slot. Every rule below
       follows from that one sentence.

  ===> THIS IS THE DESIGN'S #1 edit_existing SCOPE-CREEP RISK. lsu.scala is a
       2184-line hot file whose scalar behaviour is the P6 control arm. Read the
       edit-scope must-not-regress list as a hard boundary: with `usingRVV` false
       the emitted RTL must be bit-identical to pre-Caracal BOOM v4, term for
       term, including the ORDER of the `lsu_sched` chain.

  Governing spec anchors: loadstore.rst `lsu-unified`, `mem-order`,
  `order-fail-replay`, `fences`, `dcache-arbiter`, `mem-subsystem`,
  `vector-bw-ceiling`, `vec-store-algo`, `vec-squash`; midcore.rst
  `rename-stage`; overview.rst `caracal-pipeline`;
  caracal-milestone-plan-v2.md section 2 (structural changes + bug list), steps
  E5/E8/G1.

<|begin_module|>

  <|begin_parameters|>
  NO NEW CONSTRUCTOR PARAMETER: `class LSU(implicit p: Parameters, edge:
  TLEdgeOut)` keeps its signature so every instantiation site is untouched.

  //@req-spec-lsu.a2
  Every addition is gated on `usingRVV`, a Scala `Boolean` off `BoomCoreParams`
  via the existing `HasBoomCoreParameters` mixin — never a hardware `Bool`, never
  rocket's `usingVector`. With `usingRVV` false the underlying BOOM LSU is
  untouched: the tap is ABSENT rather than tied off, no wire or register this file
  introduces is elaborated, and the `lsu_sched` chain is textually the baseline
  chain. That is the control arm gate (f) diffs, so "absent, not zero" is a
  requirement, not a style preference.

  Values reused unchanged, all in scope already: `lsuWidth` (1 Small/Medium, 2
  Large/Mega, still bounded by the existing `require(lsuWidth <= 2)`),
  `coreWidth`, `numLdqEntries`, `numStqEntries`, `ldqAddrSz`, `stqAddrSz`,
  `robAddrSz`, `coreMaxAddrBits`, `corePAddrBits`, `xLen`, `coreDataBytes`; from
  VectorParams `vLen`, `eLen`, `maxMembers`, `vecPregSz`. One derived value is
  named rather than inlined: `searchPorts`, the number of LCAM lanes vector
  traffic may occupy, which MUST equal VecDcacheArbiter's grantable LCAM lane
  count and VecCrossLsuSnoop's `searchPorts`. It is `lsuWidth`, not an
  independent knob — a search lane with no matching drain lane would present an
  address the machine never translated.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are unchanged: Chisel's implicit posedge `clock` and ACTIVE-HIGH
  SYNCHRONOUS `reset`, one domain. No new clock, no second reset.

  The delta adds exactly TWO fields to `class LSUCoreIO`, both `usingRVV`-gated,
  and touches no other port of `LSUIO`, `LSUDMemIO` or `io.ptw`.

  //@req-spec-lsu.a3
  `io.core.lsu_vec : VecLsuCoreIO` — the tap, the field `vec_pipeline_io` names
  `lsu_vec`. Declare `class VecLsuCoreIO` in lsu.scala beside `LSUCoreIO`, in
  package `boom.v4.lsu`, so the host owns the bundle that exposes host state; it
  COMPOSES sibling-declared payloads (`LsuResourceClaim` from VecDcacheArbiter;
  `VecMemAccess`, `VecLcamSearch`, `VecVstMatch` from VecBundles) and re-declares
  none. This is how the six vector address/data queues — initialized inside
  `VecLsu` when `usingRVV` is enabled, not here — attach to the LDQ/STQ, the
  LCAM, the DTLB and the D$ port.

  Outputs (LSU to VecLsu), every one a tap on state that already exists here:
  `ld_alloc` / `st_alloc`, `Vec(coreWidth, Valid(idx))`, the placeholder
  allocation pulse (`dis_ldq_idx(w)` qualified by the existing `dis_ld_val`);
  `ldq_head`, `ldq_tail`, `stq_head`, `stq_commit_head`, `stq_tail`, each carried
  at its full `1 + ldqAddrSz` / `1 + stqAddrSz` width WITH BOOM's wrap/carry bit,
  because truncating it silently breaks every age comparison on the far side;
  `ldq_valid`, `Vec(numLdqEntries, Bool)`; `stq_vec_valid` = `stq_valid(i) &&
  stq_uop(i).is_vec`; `scalar_demand` and `scalar_avail`, `Vec(lsuWidth, new
  LsuResourceClaim)`, the arbiter seam, with `scalar_demand` additionally carrying
  `agen_incoming` and `sfence_incoming`; `dmem_req_ready` = `io.dmem.req.ready`;
  `xlate_resp`, `Vec(lsuWidth, Valid({paddr, miss, uncacheable, xcpt_valid,
  xcpt_cause}))`, REGISTERED and presented in the same cycle the scalar path has
  `exe_tlb_paddr`/`exe_tlb_miss`, because the grant promised a port and not a
  translation; `ld_search`, `Vec(lsuWidth, Valid(...))`, the LCAM-stage tap
  carrying `lcam_addr(w)`, `lcam_mask(w)`, `uop`, `lcam_ldq_idx(w)`,
  `next_stq_idx`, `can_forward(w)`, `kill_forward(w)`, the existing
  `age_matches(w)` as `stq_age_mask`, plus `is_range`/`range_lo`/`range_hi` for a
  vector unit-stride load; `stq_addr_matches` / `stq_forward_matches`, the
  existing `ldst_addr_matches` / `ldst_forward_matches` unmodified;
  `pred_overlap`, `Vec(lsuWidth, Valid({stq_idx, is_unit_stride}))`, from BOOM's
  EXISTING memory-dependence machinery in this file; and `resp` / `nack` /
  `store_ack`, the share of `io.dmem.resp` / `nack` / `store_ack` whose
  `uop.is_vec` is set, forwarded verbatim so element responses reach the LCB and
  the drains.

  Inputs (VecLsu to LSU): `vec_claim`, `Vec(lsuWidth, LsuResourceClaim)`, the
  HEAD-INSERTION pre-claim; `vec_fire`, `Vec(lsuWidth, Valid(VecMemAccess))`, the
  granted beat; `lcam`, `Vec(searchPorts, Valid(VecLcamSearch))`,
  VecCrossLsuSnoop's presentation; `vst_match`, `Vec(lsuWidth,
  Valid(VecVstMatch))`; `fwd_resp` and `replay`, `Vec(lsuWidth, ...)`,
  VecStoreForward's result and its partial-cover replay; `ld_group_done`,
  `Valid(ldq_idx)`, the LCB's one group-done per vector load retargeted at the
  placeholder; `st_pass_done`, `Valid(stq_idx)`, the pre-commit translate + DGEN
  pass complete; `st_drain_done`, `Valid(stq_idx)`, the last element irrevocably
  accepted by the cache; and `group_safe`, `Valid({idx, is_load})`, ONE
  speculation-safe event per vector memory op.

  //@req-spec-memord.f10
  `io.core.vec_lsu_empty : Input(Bool())` — the second added port, bound to
  `vec_pipeline_io`'s `lsu_fencei_rdy_vec`. It is separate from the tap because
  the map declares it separately, and because it is the only vector signal
  consumed by a term (`fencei_rdy`) living in the dispatch stage rather than the
  memory pipeline.

  ===> AND THAT IS THE WHOLE INTERFACE DELTA. No second `dmem` port and no
       widened `io.dmem.req` — the arbiter sits outside the cache and the lane
       count is still `lsuWidth`. No `busy`, `vec_busy`, `vec_active` or
       `fu_ready` in either direction: nothing exchanged here may reach an issue
       unit. No `fence_pending` and no `drain_done` — that pair is the deadlock
       the fences section deletes. No VRF port. No `vstart`, no `fault_elem`.
  <|end_ports|>

  <|begin_logic|>

  ---- 1. What the delta is, and what is reused rather than rebuilt ----

  //@req-spec-lsu.a1
  This edit extends BOOM's load/store unit so ONE unit serves scalar and vector
  loads and stores — the Unified Load/Store Unit. Extension in the literal sense:
  no parallel vector LSU appears in this file, and every vector mechanism below
  reaches an EXISTING signal.

  //@req-spec-core.e19
  Reused unchanged, named so a generator cannot mistake any of them for something
  to reimplement: the scalar LDQ and STQ (register arrays, pointers, enqueue
  logic, commit walk), store-to-load forwarding (the `ldst_forward_matches` /
  `ForwardingAgeLogic` select and the `wb_ldst_forward_*` writeback path), and the
  three-stage D$ pipeline s0/s1/s2 (`dmem_req` in s0, `io.dmem.s1_kill` and the
  LCAM in s1, `resp`/`nack`/`store_ack` in s2). The vector path rides these
  three; it does not duplicate them.

  ---- 2. Placeholder allocation: one slot, in the rename cycle, no payload ----

  //@req-spec-rename.b5
  //@req-spec-lsu.a4
  A vector load or store `OP.v` is allocated a SINGLE entry in the existing LDQ or
  STQ using the dispatch-stage logic already here: `is_vec` ops arrive with
  `uses_ldq` / `uses_stq` set exactly as scalar ones do, so `dis_ld_val` /
  `dis_st_val`, `dis_ldq_oh` / `dis_stq_oh`, `ldq_tail_oh` / `stq_tail_oh` and the
  `enableCompactingLSUDuringDispatch` rotation need NO vector term at all. That is
  the largest saving in the delta and it must not be spent: an `is_vec` special
  case in that loop would be a second allocation path.

  //@req-spec-rename.b6
  //@req-spec-rename.b7
  The slot is claimed at the in-order `ldq_tail` / `stq_tail` in the single
  rename/dispatch cycle — Caracal renames scalar and vector in parallel in one
  cycle, so there is no group skew to reconcile — and `io.core.dis_ldq_idx(w)` /
  `dis_stq_idx(w)` carry the index back so rename writes it into the `OP.v` as
  `ldq_idx` / `stq_idx`, the PROGRAM-AGE STAMP every cross-queue comparison in
  sections 5 and 6 keys on. The existing assertions `actual_ld_enq_idx ===
  io.core.dis_uops(w).bits.ldq_idx` and its store twin check that the two sides
  agree and are load-bearing for vector ops too — do not weaken them.

  //@req-spec-rename.b8
  THE ENTRY HOLDS NO PER-ELEMENT ADDRESS AND NO PER-ELEMENT DATA. `LDQEntry` and
  `STQEntry` gain no vector payload field: no address vector, no `vLen`-wide data
  field, no per-element mask. `ldq_addr`, `stq_addr` and `stq_data` are never
  written with element values — the addresses are produced later at the vector LS
  AGEN/DGEN and live in the SSI/US queues. Only the SLOT is reserved here. A
  `vLen`-wide payload per STQ slot is not a feasible structure, which is the whole
  reason those queues exist.

  ---- 3. Where a vector entry's address and data actually come from ----

  //@req-spec-lsu.a6
  When a vector LDQ or STQ entry becomes ready for execution it reads its
  effective address, and a store its data, from the separate address and data
  queues rather than from a uOP — the opposite of the scalar path, where
  `agen(w).bits.data` and `io.core.dgen` carry them. Concretely, `vec_fire(w)` —
  the arbiter's granted beat, already read out of a queue — is muxed in as the
  OUTERMOST arm of existing Mux chains, under `usingRVV`: into `exe_tlb_uop(w)`,
  `exe_tlb_vaddr(w)`, `exe_size(w)`, `exe_cmd(w)` for the DTLB request (with
  `exe_passthr` / `exe_kill` keeping their defaults); into
  `dmem_req(w).bits.addr` / `.data` / `.uop` for the D$ access; and into
  `lcam_addr(w)` with the search terms of section 5. The beat's `uses_tlb` /
  `uses_dcache` / `uses_lcam` bits select which of those it drives, so a store's
  pre-commit translate pass takes TLB+LCAM and not the D$, and its post-commit
  write pass takes the D$ alone.

  //@req-spec-lsu.k5
  Each `dmem.req` lane carries AT MOST ONE ELEMENT per cycle (at most `eLen` = 64
  bits) and there are `lsuWidth` lanes. `vec_fire(w)` is a single beat and this
  file never coalesces two elements into one request, so vector memory bandwidth
  is the scalar port's — 64 bits/cycle on Medium, 128 on Large/Mega — regardless
  of `vLen`. Nothing in this delta may try to raise that ceiling.

  ---- 4. The head-insertion hook, and why a tail-only insertion is wrong ----

  //@req-spec-lsu.k2
  //@req-spec-lsu.k3
  The D$ interface arbiter sits OUTSIDE the cache and presents the same
  `dmem.req` interface: `io.dmem` is unmodified, the memory subsystem below is
  unchanged from BOOM v4, and `VecDcacheArbiter` reaches this file only through
  `vec_claim` / `scalar_avail` / `vec_fire`.

  //@req-spec-lsu.a11
  Element accesses contend for the shared D$ port THROUGH that arbiter, so a
  vector drain does not monopolize the cache. In the per-lane controller loop
  (currently lsu.scala:645-692) the mechanism is: at the HEAD of the chain, BEFORE
  the first `lsu_sched` call, initialize the three vars from the pre-claim instead
  of from `true.B` — `tlb_avail = !vec_claim(w).tlb`, `dc_avail =
  !vec_claim(w).dcache`, `lcam_avail = !vec_claim(w).lcam` (literal `true.B` when
  `usingRVV` is false); then the twelve `lsu_sched` calls follow in their EXISTING
  ORDER, unchanged, term for term; then after the chain export `scalar_avail(w) :=
  {tlb_avail, dc_avail, lcam_avail}`, the residual the arbiter's default path
  consumes.

  ===> A TAIL-ONLY INSERTION CANNOT DELIVER THE ANTI-STARVATION GUARANTEE, AND
       APPENDING THE VECTOR REQUESTS LAST AT LOWEST PRIORITY IS EXACTLY THE
       PREVIOUS ATTEMPT'S DEFECT. `addvector` added the vector requests as extra
       `lsu_sched` calls at the end of this chain with `uses_tlb = false` and
       `uses_lcam = false`. Both halves were wrong and each failed differently:
       lowest static priority with no escape valve made vector accesses
       arbitrarily slow behind any scalar stream, and declaring no TLB/LCAM use
       made every vector element access INVISIBLE to disambiguation. The head
       insertion is what lets the arbiter's one-cycle elevation actually take a
       resource; the residual export is what keeps the ordinary case at the tail,
       with the scalar chain at baseline priority.

  `exe_tlb_valid(w) := !tlb_avail` keeps its exact current form and now correctly
  reads "lane `w` issues a DTLB request this cycle", the requester being the
  vector beat when it pre-claimed — which is why section 3 muxes the beat into
  `exe_tlb_vaddr` at the head of the chain rather than later.

  ===> THE PRE-CLAIM MUST NOT BLOCK AN INCOMING agen OR AN sfence, and this is
  the sharpest hazard in the delta. `io.core.agen` and `io.core.sfence` are
  bare Valid inputs with NO back-pressure path — lsu.scala:682 asserts that a
  valid agen always fires — so a pre-claim that stole the TLB or LCAM from an
  incoming agen would DROP a scalar memory operation, not delay it. The veto
  therefore belongs in the arbiter's elevation decision, which is why
  `scalar_demand` carries `agen_incoming` / `sfence_incoming`. This stays
  acyclic: those valids and every `can_fire_*` term are computed BEFORE the
  chain and depend on none of the three avail vars. What must never feed
  `vec_claim` is `scalar_avail`, the post-chain residual — that path is
  combinationally cyclic through this file.

  ---- 5. Routing vector addresses through the LCAM, in BOTH directions ----

  //@req-spec-memord.a1
  Vector element addresses live in the SSI/US address queues, so the scalar LCAM
  cannot see them by default and a vector store would silently fail to order
  against a scalar load. The delta routes them through the disambiguation
  machinery explicitly and in BOTH directions, so RVWMO ordering between scalar
  and vector memory ops holds. The vehicle is `lcam(i)`, which drives the SAME
  signals a scalar addr-gen drives: `do_st_search(w)` / `do_ld_search(w)`,
  `lcam_addr(w)`, `lcam_uop(w)`, `lcam_mask(w)`, `lcam_stq_idx(w)`,
  `lcam_ldq_idx(w)`, `lcam_next_stq_idx(w)`.

  //@req-spec-memord.a3
  //@req-spec-memord.a5
  ST to LD: each vector store address presented for execute becomes an ordinary
  store searcher. `do_st_search(w)` gains the vector presentation as an OR term
  and nothing else changes, so the existing per-LDQ-entry comparator loop
  (lsu.scala:1183-1271) does the work with its existing
  `IdxAgeOt(lcam_stq_idx, l_next_stq_idx)` age test — indistinguishable from a
  scalar store addr-gen apart from the unit-stride range predicate, which replaces
  the entry's `dword_addr_matches` equality with the presented
  `range_lo`/`range_hi` magnitude pair when `lcam(i).is_range` is set.

  //@req-spec-memord.a4
  That LDQ search must cover BOTH scalar LDQ entries AND in-flight vector loads. A
  vector load's placeholder holds no address, so the comparator has nothing to
  compare unless this file gives it something: maintain a per-LDQ-entry
  dword-granular ADDRESS BOUND for vector loads, `ldq_vec_lo` / `ldq_vec_hi`,
  widened on each element or range address that entry presents as a load searcher
  and cleared on `ld_alloc`. For a vector-load entry the comparator uses range
  overlap against that bound in place of the equality. The bound is a strict
  SUPERSET of the addresses actually presented, which is what makes it sound: a
  superset yields no false negatives, so no aliasing load is missed, and a false
  positive costs a replay and never data. It is the mirror image of the
  per-STQ-entry summary VecCrossLsuSnoop keeps for the other direction, and it
  belongs here because this file owns the LDQ.

  A vector-load placeholder's `ldq_executed` is set when its FIRST element
  access fires (the same event that sets `s0_executing_loads` for a scalar
  load), because `l_executed || l_succeeded` is what admits it to the ST->LD
  match. Symmetrically the nack handler's `ldq_executed(...) := false.B` is
  gated `!is_vec`: one nacked element must not un-execute a whole vector load,
  since the drain re-offers that beat by itself.

  //@req-spec-memord.a18
  //@req-spec-memord.a20
  LD to ST: each load address presented for execute is searched against the scalar
  STQ exactly as today — the per-STQ-entry loop, `addr_matches`,
  `forward_matches`, `age_matches` and the `ldst_addr_matches` /
  `ldst_forward_matches` reduction are untouched — and additionally against the
  vector store address queues. That second half arrives already computed as
  `vst_match(w).addr_match`, which ORs into `ldst_addr_matches(w)`, so ONE
  `ForwardingAgeLogic` ranks scalar and vector stores in a single
  `numStqEntries`-wide space. It is correctness-bearing, not an optimization: a
  vector store drains POST-COMMIT, so a load that missed an older vector store's
  already-generated address would read a line the store has not yet written and
  nothing would replay it.

  A vector store address queue entry is NOT presented to the LCAM until its
  paired `st_*_DATA_Q` entry is valid. That gate lives in VecCrossLsuSnoop and
  this file must not second-guess it: the LSU never synthesizes a store
  searcher out of a queue entry itself, it only consumes `lcam(i)`. Assert that
  an accepted store presentation had its data half filled, so a regression on
  the far side fails loudly here rather than forwarding a byte that does not
  exist yet.

  ---- 6. order_fail, the replay path, and the one lxcpt ----

  //@req-spec-memord.a6
  //@req-spec-memord.c4
  A vector store address matching a YOUNGER, ALREADY-EXECUTED load sets that
  load's LDQ `order_fail` bit — `ldq_order_fail(i) := true.B` and `failed_load :=
  true.B`, the existing statements in the existing `when` block, now reachable
  from a vector searcher. Mark, do not act: this file sets a bit and raises a
  mini-exception, it does not squash.

  //@req-spec-memord.a7
  //@req-spec-memord.a14
  The load is then REPLAYED by BOOM's existing ordering-violation path, and the
  extension is only in what FIRES it. `MINI_EXCEPTION_MEM_ORDERING` at the ROB
  head produces a flush with `flush_typ = refetch`; the load refetches at its own
  PC, re-renames and re-executes. Memory-dependency speculation likewise reuses
  the existing predictor and this same replay path, now firing on the cross-queue
  matches of section 5 as well as on scalar-versus-scalar ones. NO new predictor,
  no new replay mechanism, no selective replay of the load and its dependents.

  //@req-spec-memord.c5
  //@req-spec-memord.c6
  The LSU broadcasts the OLDEST failing load to the ROB as an `lxcpt` with cause
  `MINI_EXCEPTION_MEM_ORDERING`, and this needs no new logic: `l_idx =
  LSUAgePriorityEncoder(ldq_valid(i) && ldq_order_fail(i), ldq_head)` already
  selects the oldest, `r_xcpt.cause := Mux(use_mem_xcpt, mem_xcpt_cause,
  MINI_EXCEPTION_MEM_ORDERING)` already assigns the cause, and the `use_mem_xcpt`
  priority against a translation fault is unchanged. A vector-load placeholder
  participates in that priority encoder like any other entry.

  //@req-spec-memord.e1
  //@req-spec-memord.e2
  For a VECTOR load that order-fails the same refetch/re-rename path applies at
  the granularity of the WHOLE VECTOR INSTRUCTION: the single LDQ placeholder
  entry drives ONE `lxcpt`. There is no per-element and no per-member `lxcpt`, and
  this file exposes no partial-group rewind — on replay the op re-renames its
  whole destination group and re-drains from element zero through the LCB, which
  is what keeps it consistent with the one-group-done completion model.

  ---- 7. Commit, drain eligibility, and the vse teardown hang ----

  //@req-spec-lsu.a7
  //@req-spec-lsu.j9
  The conditions that make a vector LDQ or STQ entry eligible to drain are THE
  SAME as scalar; no new eligibility condition is introduced. A store becomes
  eligible only once the ROB retires it: the existing commit walk sets
  `stq_committed(temp_stq_commit_head)` and `stq_can_execute(...)`, and only then
  may its elements be written. For the placeholder's ROB busy bit to clear at all
  the existing `clr_bsy` walk must advance past it, and that walk requires
  `stq_addr(idx).valid && stq_data(idx).valid`. So on `st_pass_done(idx)` — the
  pre-commit translate pass plus DGEN complete — set `stq_addr(idx).valid :=
  true.B` with `stq_addr_is_virtual(idx) := false.B`, and `stq_data(idx).valid :=
  true.B`, WITHOUT meaningful bits: for a vector entry those registers are
  don't-care and are never read. They are pass-complete markers, and treating them
  as anything else is the mis-generation to watch for.

  ===> M1 BUG — THE `vse` TEARDOWN HANG. THIS IS THE MOST IMPORTANT PARAGRAPH IN
       THIS FILE. Those markers make `can_enq_store_execute` true, which must NOT
       happen: pushing a vector placeholder into `stq_execute_queue` would issue
       one bogus scalar-shaped D$ write from a don't-care address. So qualify it,
       `can_enq_store_execute := <existing terms> && !stq_enq_e.bits.uop.is_vec`
       — and a vector-store placeholder then NEVER executes via the normal path,
       so `stq_execute_head` NEVER ADVANCES PAST IT. That is the observed M1 bug:
       every YOUNGER SCALAR STORE is stranded behind it, including the HTIF
       `tohost` write, so the simulation runs to the `+max-cycles` timeout even
       though the test itself passed. THE FIX, and it is structurally the fence's
       fix: LIKE A FENCE, ADVANCE `stq_execute_head` WHEN A COMMITTED-AND-
       SUCCEEDED VECTOR-STORE PLACEHOLDER IS CLEARED. In the existing
       `when (clear_store)` block, where the file already writes
       `when (stq_head_is_fence) { stq_execute_head := WrapIncWCarry(...) }`,
       extend the condition to `stq_head_is_fence || stq_head_is_vec_store`.
       `clear_store` itself needs no new term, because `stq_succeeded` for a
       vector placeholder is defined next.

  `stq_succeeded(idx)` for a vector placeholder is set by `st_drain_done(idx)`
  — the LAST element irrevocably accepted — and NOT by `io.dmem.store_ack`,
  whose handler is gated `!is_vec`. Setting it on the first element's ack would
  let `clear_store` fire, free the entry and advance `stq_head` while most of
  the store was still in the queues. Same shape on the load side:
  `ld_group_done(idx)` sets `ldq_executed(idx)` and `ldq_will_succeed(idx)`,
  which is what satisfies the existing commit assertion
  `(ldq_executed || ldq_forward_std_val) && ldq_succeeded`; a vector load's
  completion is the LCB's group-done, never an `iresp` beat.

  //@req-spec-lsu.a8
  //@req-spec-lsu.a9
  A draining entry drains ALL of the element accesses it owns, in program and
  element order, with no other op interleaving AT THAT ENTRY. Two structural facts
  give this and no arbiter state is needed for it: an LDQ/STQ entry belongs to
  exactly one `OP.v` for its whole lifetime, and the element cursor lives in that
  entry, so `VecBeatExpander` peeks the queue head and read-modify-writes
  `elem_next` in one cycle and can only advance. This file's contribution is
  negative and therefore easy to break: it must not free, re-present or reorder a
  placeholder mid-drain. Interleaving with OTHER entries is expected and is the
  arbiter's business — atomicity is per entry, not per port.

  //@req-spec-lsu.a10
  A faulting vector load or store commits NO elements. NO RTL IS ADDED HERE FOR THIS:
  a page/access fault on any element arrives through this file's own DTLB response,
  and the EXISTING, UNTOUCHED `mem_xcpt_valids` machinery writes it to the placeholder
  because the beat's uop carries the placeholder's `ldq_idx`/`stq_idx` (VecBeatExpander
  copies them onto every beat it composes, so the attribution is by construction):
  `ldq_uop(idx).exception := true.B`
  or `stq_uop(idx).exception := true.B`, one exception on one entry. The
  placeholder never commits, its fresh destination group is reclaimed by the ROB
  rollback, and the op traps with `vstart = 0` and restarts whole from element 0.
  `fault_elem` stays in the vector LSU as the cursor's stop signal and a debug
  counter — it reaches neither the ROB nor `vstart` from here.

  `group_safe` is the ONE speculation-safe event per vector memory op:
  `io.core.clr_unsafe` for a vector placeholder is driven from it and NOT from
  each element's `RegNext(do_ld_search)` / `RegNext(do_st_search)`, so a vector
  store's `rob_unsafe` clears exactly once, when its LAST element address has been
  LCAM-checked. Per-element pulses would clear it after the first element and let
  the PNR sweep past an op most of whose addresses were still unchecked. The
  existing `&& !RegNext(failed_load)` qualification is kept.

  ---- 8. Fences: fold vec_lsu_empty into fencei_rdy, and nothing else ----

  //@req-spec-memord.f1
  //@req-spec-memord.f2
  //@req-spec-memord.f3
  `fence` (RVWMO `fence rw,rw` and friends), `fence.i` and `sfence.vma` must order
  vector memory ops alongside scalar ones.

  //@req-spec-memord.f5
  //@req-spec-memord.f11
  Caracal gets that WITHOUT ADDING ANY MECHANISM, by extending the signal BOOM
  already gates fences on. All three are decoded `is_unique`, and an `is_unique`
  uop cannot DISPATCH until the ROB is empty and `fencei_rdy`. So the entire fence
  delta is one term on one line — lsu.scala:439 becomes

      io.core.fencei_rdy := !stq_nonempty && io.dmem.ordered && vec_lsu_empty

  with `vec_lsu_empty` tied to `true.B` when `usingRVV` is false, so the line is
  bit-identical to baseline in a vectors-off build.

  //@req-spec-memord.f19
  //@req-spec-memord.f20
  //@req-spec-memord.f21
  `vec_lsu_empty` asserts only when ALL vector LSU state is empty: the four
  address queues (`ld_SSI_ADDR_Q`, `st_SSI_ADDR_Q`, `ld_US_ADDR_Q`,
  `st_US_ADDR_Q`), BOTH store data queues (`st_SSI_DATA_Q`, `st_US_DATA_Q`), and
  the Load Coalescing Buffer. `VecLsu` computes the AND of those seven `io.empty`
  outputs; this file consumes the single bit and must not re-derive it, so there
  is exactly one definition of "the vector LSU is empty". In-flight D$ responses
  are already covered by the `io.dmem.ordered` term in the same expression.

  //@req-spec-memord.f14
  The conservative breadth costs nothing, for a structural reason rather than a
  sizing argument: the wait happens at DISPATCH with the ROB already empty, so
  every older vector store has already COMMITTED and will drain unconditionally —
  the section-7 `stq_execute_head` advance is what makes "unconditionally" true
  for the store at the head — and nothing younger exists yet, because dispatch is
  in program order.

  //@req-spec-memord.f15
  //@req-spec-memord.f16
  ===> DO NOT IMPLEMENT THE HEAD-SIDE HANDSHAKE. Earlier drafts had the ROB raise
       `fence_pending` when a fence reached the head, with the scalar and vector
       LSUs driving `drain_done` when their queues were empty. IT DEADLOCKS: a
       YOUNGER vector store, dispatched after the fence, fills `st_SSI_*_Q`, and
       those entries can only be freed at COMMIT-DRAIN — which cannot happen,
       because the fence is at the ROB head. The fence cannot retire because the
       queues are not empty, and neither side moves. So fence retirement is NOT
       conditional on the vector store queues being empty anywhere in this file:
       `clear_store`, `stq_head_is_fence` and the `io.dmem.force_order` /
       `store_needs_order` pair keep their exact current form, and no
       `fence_pending` or `drain_done` port exists. Ordering is established before
       the fence ever enters the machine.

  ---- 9. Squash: the four pointers out, and the store-side asymmetry ----

  `VecSquashUnit` owns the vector rollback policy and needs this file to export
  `ldq_head`, `ldq_tail`, `stq_commit_head` and `stq_tail`, each with BOOM's
  wrap/carry bit, so its age comparisons run on `EntryValidFromAge` /
  `IsOlderLSU` / `GetRealLSQIdx` rather than on a hand-written compare. This file
  keeps rolling its OWN pointers exactly as it does now — `stq_tail :=
  io.core.brupdate.b2.uop.stq_idx` and `ldq_tail := ...ldq_idx` on a mispredict;
  `ldq_head := 0`, `ldq_tail := 0` and `stq_tail := stq_commit_head` on
  `io.core.exception` — and the vector rollback lands on the SAME cycles.

  ===> ON A FLUSH THE STORE-SIDE VECTOR QUEUES ROLL BACK TO `stq_commit_head`,
       NOT TO `stq_head`, and getting this wrong loses committed store data. A
       COMMITTED vector store has not yet written the D$: its translated element
       addresses live in `st_SSI_ADDR_Q` / `st_US_ADDR_Q` and its data in
       `st_SSI_DATA_Q` / `st_US_DATA_Q` until the post-commit drain. Emptying all
       six queues would discard the addresses and data of stores that are already
       architecturally committed and can no longer be re-executed. This mirrors
       this file's own `stq_tail := stq_commit_head`; the load side is the
       asymmetric case that DOES empty, mirroring `ldq_head := 0; ldq_tail := 0`.

  ---- 10. Tracing ----

  There are no unit tests in this project — no `chiseltest`, no `*Spec.scala` —
  and validation is end-to-end VCS plus Whisper cosim only, so every vector term
  added here emits a guarded trace line through the shared `VecTrace` package,
  one line per key event, tagged with the module name and `rob_idx`, gated on the
  `vecTrace` plusarg and `!reset` and therefore off by default: a placeholder
  allocation, a granted vector beat (lane, resources claimed, elevated or not), a
  vector LCAM presentation, an `order_fail` set by a vector searcher, the
  `stq_execute_head` advance of section 7, and the `fencei_rdy` transition with
  the `vec_lsu_empty` term broken out. The last two are what a teardown hang is
  diagnosed from, and both are invisible in a waveform without a name.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
The binding constraint on this file is not throughput, it is that the scalar
machine keeps its exact cycle behaviour. Four constraints follow, each forcing an
implementation choice rather than describing one.

TARGET P6 — no scalar regression against the vectors-off build. With the drains
idle `vec_claim` is all-zero, the three avail vars start at `true.B`, and the
`lsu_sched` chain resolves cycle-for-cycle as baseline. The scalar path must
acquire no new dependency on a vector signal: `vec_fire` enters as the outermost
arm of Muxes that already existed, `vec_claim` only as the chain's initial value.

NO NEW PIPELINE STAGE IN THE LCAM CYCLE. `ldq_order_fail`, `ldst_addr_matches`
and `wb_ldst_forward_valid` resolve in the cycle after translation today, and the
vector terms — the range predicate, the `vst_match` OR, the per-LDQ-entry vector
bound — must resolve in that same cycle. Registering the match would let a load
write back and be marked safe a cycle before the failure is known, widening the
past-PNR window `VecCiiFlush` exists to cover, for no benefit. Precompute
`range_lo`/`range_hi` once per presentation at the searcher, never once per
entry, so `base + len - 1` stays out of the per-entry loop.

NO COMBINATIONAL LOOP THROUGH THIS FILE, one-directionally: `scalar_avail`
depends on `vec_claim`, so `vec_claim` may depend only on the arbiter's
registered state and on signals computed BEFORE the chain — the `can_fire_*`
terms, `agen(w).valid`, `io.core.sfence.valid`, `dmem_req_ready` — and never on
`scalar_avail`. `dmem_req_ready` is a cache input and outside the loop.

`fencei_rdy` STAYS COMBINATIONAL. It is consumed in the dispatch stage, so
`vec_lsu_empty` must arrive as a level and must not be registered on the way in;
one extra AND term on that line is the whole timing cost.

Area: the additions are the per-LDQ-entry vector address bound
(`numLdqEntries` x 2 dword addresses, roughly 1.4 kbit at Medium sizing) and the
per-entry comparator growing from one dword equality to two magnitude compares of
`corePAddrBits - 3` bits on the already-late TLB-response path — the plausible
critical path in this file. Nothing else adds storage: `LDQEntry` and `STQEntry`
gain no field.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the delta reads `is_vec`, `is_shared`, `ldq_idx`, `stq_idx`, `rob_idx`,
`v_eew` and the access-class flags `v_is_unit_stride` / `v_is_strided` /
`v_is_indexed` / `v_is_segment` off the uop, and re-decodes `uop.inst` NOWHERE.
It writes only uop fields this file already writes (`pdst`, `exception`,
`br_mask`).
VecBundles — `VecMemAccess`, `VecLcamSearch`, `VecVstMatch` and the element-queue
enumeration the fences section names. `VecLsuCoreIO` is declared HERE, beside
`LSUCoreIO`, because it exposes this file's own state; it composes those and
`LsuResourceClaim`, which VecDcacheArbiter declares.
VectorParams — `vLen`, `eLen`, `maxMembers`, `vecPregSz`.

Instantiates nothing new. Existing instances (`dtlb`, `bkptu`, `retry_queue`,
`stq_execute_queue`, `wakeupArbs`, `ForwardingAgeLogic`) are untouched.

Counterparties, each of which Phase R must check from the other side:
  - VecDcacheArbiter — `vec_claim` in at the chain HEAD, `scalar_demand` /
    `scalar_avail` / `dmem_req_ready` out, `vec_fire` in. It must let
    `scalar_demand.agen_incoming` / `.sfence_incoming` veto an elevation; its
    current text says the grant does not depend on `scalar_demand` at all, which
    would drop a non-back-pressurable scalar agen. Reported as a conflict.
  - VecSquashUnit — the four pointers out, and the store-side rollback to
    `stq_commit_head` of section 9.
  - VecCrossLsuSnoop — `lcam` in, `ld_search` / `stq_vec_valid` / `st_alloc` out;
    it owns the "no address without its data" presentation gate.
  - VecStoreForward — `ld_search` / `stq_addr_matches` / `stq_forward_matches`
    out, `fwd_resp` / `replay` in, consumed on the existing `iresp`/`fresp` and
    `s1_kill` paths.
  - VecOrderHold — `ldq_valid` / `ld_alloc` / `pred_overlap` out. Its `ld_ctx`
    declares four fields and no carrier for the store-age boundary its own age
    rule needs; this file exports `stq_head` and the per-lane `stq_age_mask` so
    either resolution is available. Reported as a seam gap.
  - VecLoadCoalescingBuffer and VecBeatExpander (x2) — `resp` / `nack` /
    `store_ack` / `xlate_resp` out, `ld_group_done` in.
  - VecLsu — `st_pass_done`, `st_drain_done`, `group_safe` in, and
    `vec_lsu_empty` on the separate `lsu_fencei_rdy_vec` port.
  - Rob (edit_existing) — consumes the unchanged `io.core.lxcpt`, `clr_bsy` and
    `clr_unsafe`; no new port between the two files.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File   src/main/scala/v4/lsu/lsu.scala
    Classes `class LSU(implicit p: Parameters, edge: TLEdgeOut) extends
    BoomModule with rocket.HasL1HellaCacheParameters`, and `class LSUCoreIO` (two
    added fields only). Package boom.v4.lsu. Hand-written baseline BOOM v4.

  In scope, and nothing outside this list:
    - `class LSUCoreIO`: add `lsu_vec: VecLsuCoreIO` and `vec_lsu_empty:
      Input(Bool())`, both `usingRVV`-gated, plus the `class VecLsuCoreIO`
      declaration beside it.
    - The per-lane controller loop: initialize `tlb_avail` / `dc_avail` /
      `lcam_avail` from `vec_claim(w)` before the first `lsu_sched` call, and
      export `scalar_demand(w)` / `scalar_avail(w)` after the chain.
    - `exe_tlb_uop`, `exe_tlb_vaddr`, `exe_size`, `exe_cmd` and
      `dmem_req(w).bits.addr` / `.data` / `.uop`: one added arm each for
      `vec_fire(w)`.
    - `do_ld_search`, `do_st_search`, `lcam_addr`, `lcam_uop`, `lcam_mask`,
      `lcam_ldq_idx`, `lcam_stq_idx`, `lcam_next_stq_idx`: added vector OR terms
      and Mux arms from `lcam(i)`.
    - The per-LDQ-entry and per-STQ-entry comparator loops: the range-overlap
      predicate under `lcam(i).is_range`, plus the new `ldq_vec_lo` /
      `ldq_vec_hi` registers and their widen/clear.
    - `ldst_addr_matches`: OR in `vst_match(w).addr_match`.
    - `io.core.fencei_rdy`: add the `&& vec_lsu_empty` term.
    - `can_enq_store_execute`: add the `&& !uop.is_vec` qualification.
    - The `when (clear_store)` block: extend the `stq_execute_head` advance
      condition with the committed vector-store-placeholder case.
    - `stq_addr(idx).valid` / `stq_addr_is_virtual(idx)` / `stq_data(idx).valid`
      from `st_pass_done`; `stq_succeeded` from `st_drain_done` with the
      `store_ack` handler gated `!is_vec`; `ldq_executed` / `ldq_will_succeed`
      from `ld_group_done`, with the nack handler's `ldq_executed := false.B`
      gated `!is_vec`.
    - `io.core.clr_unsafe`: for a vector placeholder, source the valid from
      `group_safe` instead of the per-search terms.
    - The tap's remaining output assignments (pointers, valid vectors, alloc
      pulses, `ld_search`, `xlate_resp`, `pred_overlap`, response forwarding),
      plus the guarded `VecTrace` lines and the new assertions.

  Must not regress — bit- and cycle-identical, by their real names:
    - THE `lsu_sched` CHAIN'S TERM ORDER AND ITS TWELVE `will_fire_*`
      ASSIGNMENTS, and each one's `(uses_tlb, uses_dc, uses_lcam)` column. Only
      the three vars' INITIAL VALUE changes. The priority comment above the chain
      stays.
    - `exe_tlb_valid(w) := !tlb_avail` keeps its exact form, as do
      `dtlb.io.req(w).*`, `dtlb.io.kill`, `dtlb.io.sfence`, the `bkptu` wiring
      and `io.ptw <> dtlb.io.ptw`.
    - The dispatch loop: `dis_ldq_oh` / `dis_stq_oh`, `dis_uops`,
      `live_store_mask`, `ldq_tail_oh` / `stq_tail_oh`, the
      `enableCompactingLSUDuringDispatch` rotation, `WrapIncWCarry`, and both
      enqueue-tag assertions.
    - `retry_queue` and its feeders: `ldq_enq_retry_idx`, `stq_enq_retry_idx`,
      `can_enq_load_retry`, `can_enq_store_retry`, the `addr.valid := false`
      handoff.
    - `stq_execute_queue` (the 4-deep `Queue`), `stq_execute_queue_flush`, and
      the nack rewind `stq_execute_head := io.dmem.nack(w).bits.uop.stq_idx`
      under `IsOlderLSU`.
    - The whole hellacache shim: `hella_state` and its seven states,
      `can_fire_hella_incoming` / `can_fire_hella_wakeup`, every
      `io.hellacache.*` assignment.
    - `block_load_mask` / `p1_block_load_mask` / `p2_block_load_mask`,
      `block_load_wakeup`, `stq_almost_full`, `store_needs_order`, and the
      `lsuWidth == 1` deadlock avoidance including `store_blocked_counter`.
    - The forwarding path: `addr_matches`, `forward_matches`, `prs2_matches`,
      `age_matches`, `ldst_forward_matches`, `stld_prs2_matches`,
      `ForwardingAgeLogic`, `wb_ldst_forward_*`, `can_forward`, `kill_forward`,
      and the `enableStLdForwarding` load-to-store-data path.
    - `s0_kills`, `io.dmem.s1_kill`, `s0_executing_loads`, `s1_executing_loads`,
      `s1_set_execute`.
    - The exception network: `mem_xcpt_valids` / `_uops` / `_causes` / `_vaddrs`,
      the oldest-xcpt selection loop, `use_mem_xcpt`, `r_xcpt`, and
      `io.core.lxcpt`'s shape and timing.
    - `wakeupArbs`, `enableFastLoadUse` timing, `spec_wakeups`, `slow_wakeups`,
      `iresp` / `fresp`, `io.core.iwakeups`.
    - The `clr_bsy` walk (`stq_clr_head_idx`, `clr_valid` / `clr_valid_1`,
      `stq_cleared`) and `clr_unsafe` for SCALAR ops.
    - The LDQ and STQ branch-kill loops, `GetNewBrMask`, `IsKilledByBranch`,
      `UpdateBrMask`, and the reset/exception block including
      `stq_tail := stq_commit_head`.
    - `clear_store`, `stq_head_is_fence`, `io.dmem.force_order` and the fence
      drain — only the `stq_execute_head` advance CONDITION widens.
    - Every helper object below `class LSU` (`GenByteMask`,
      `LSUAgePriorityEncoder`, `ForwardingAgeLogic`, `GetRealLSQIdx`,
      `GetLSQIdxCarry`, `IdxAge*`, `IsOlderLSU`, `SafeRegNext`,
      `EntryValidFromAge`, `WrapAddWCarryWidth`) — reuse, do not modify, and do
      not reimplement an age compare inline.
    - `require(lsuWidth <= 2)`, `require(xLen >= fLen)`, and the "Some operations
      is proceeding down multiple pipes" assertion.
    - With `usingRVV = false`: no added port, wire, register or Mux arm is
      elaborated and the emitted RTL is bit-identical to the current file's. No
      reformatting, no reordering, no renaming, and the existing Regents/SiFive
      copyright header and comment blocks are preserved verbatim.

  Interface delta:
    NEW on `class LSUCoreIO`, both `usingRVV`-gated:
      lsu_vec        : VecLsuCoreIO   (fields as listed in the ports section)
      vec_lsu_empty  : Input(Bool())  (bound to `lsu_fencei_rdy_vec`)
    NEW type declared in this file: `class VecLsuCoreIO extends BoomBundle`.
    WIDENED: nothing. `io.dmem`, `io.ptw`, `io.hellacache`, `io.core.agen`,
    `io.core.dgen`, `io.core.iresp` / `fresp` / `iwakeups`, `io.core.clr_bsy`,
    `io.core.clr_unsafe`, `io.core.lxcpt`, `io.core.commit` and every other
    existing port keep their current declaration exactly.
    NEW parameters: none.

    Explicitly NOT added, and a reviewer should reject them on sight: any `busy` /
    `vec_busy` / `vec_active` / `fu_ready` output; `fence_pending` or `drain_done`
    in either direction; a second `dmem` request port or a widened `dmem.req`
    lane; any per-element or `vLen`-wide field on `LDQEntry` / `STQEntry`; a
    separate vector LDQ or vector STQ; an LCAM, DTLB or D$ port beyond the
    existing `lsuWidth`; a VRF port; a `vstart` or `fault_elem` path to the ROB; a
    per-element `lxcpt` or a partial-group rewind; a `RegNext` on
    `vec_lsu_empty` or on any LCAM match term.
<|end_edit_scope|>

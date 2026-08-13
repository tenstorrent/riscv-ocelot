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
  VecCrossLsuSnoop — cross-queue disambiguation SEARCH: it presents vector memory
  addresses to the scalar LCAM, and it searches load addresses against the vector
  store address queues, at the granularity each access class demands.
*/

hierarchy.yaml: kind: module, mode: new,
output src/main/scala/v4/vec/generated/lsu/VecCrossLsuSnoop.scala,
package boom.v4.vec.generated.lsu.
depends_on MicroOp, VecBundles, VectorParams, VecTrace. Instantiates nothing.
Instantiated ONCE by VecLsu as `snoop`. Plan step G1.

===> IT SEARCHES. IT DOES NOT FORWARD AND IT DOES NOT WAIT. Two siblings own the
     other two mechanisms and this file must not re-specify either.
     VecStoreForward owns LD->ST forwarding — selecting the winning store,
     mask-qualifying the bytes, and returning DATA out of `st_*_DATA_Q` — and it
     alone drives VecOrderHold's `known_overlap`. VecOrderHold owns the
     predicted-overlap hold: which younger vector load waits behind which older
     draining vector store. What this node owes them is a MATCH, correctly aged,
     at the right granularity, plus the identity of the entry that matched. The
     ST->LD direction, the one that raises `order_fail`, is this node's own.

===> IT IS BIDIRECTIONAL, AND THAT IS THE POINT OF THE NODE. Vector element
     addresses live in the SSI/US address queues, NOT in the scalar STQ, so
     BOOM's LCAM cannot see them and a vector store would silently fail to order
     against a scalar load. Addresses are therefore routed through the
     disambiguation machinery in BOTH directions: a vector store address drives
     the LCAM exactly as a scalar store addr-gen does, and a load address is
     searched against the vector store address queues exactly as a scalar load
     drives the LCAM today.

Governing spec anchors: loadstore.rst `mem-order`, `order-fail-replay`,
`dcache-arbiter`, `us-queue`, `store-data-queue`;
caracal-milestone-plan-v2.md Phase G step G1, section 5 rules 6, 10 and 11.

<|begin_module|>

  <|begin_parameters|>
  Elaborated only when `usingRVV` is true — a Scala `Boolean` from
  `BoomCoreParams`, not a hardware `Bool` and not rocket's `usingVector`. With
  vectors off this module is ABSENT, not tied off, and every term it adds inside
  `lsu.scala` disappears with it, so a non-vector build stays bit-identical to
  pre-Caracal BOOM v4.

  `searchPorts` (Int, default 1) — how many LCAM search lanes vector traffic may
  occupy in one cycle. It must equal the number of LCAM lanes VecDcacheArbiter can
  grant to vector, which VecLsu derives from `lsuWidth` and `dcacheArbiterMode`
  ("single" -> 1, "dual-dynamic" -> 2). It mirrors VecElemQueue's `ports` and must
  not be set independently: a search lane with no matching drain lane would
  present an address the machine never translated.

  `ssiSnoopWindow` (Int, default 16) — how many recently presented SSI store
  element addresses are held here in FLOPS so a load can compare against them
  exactly and in one cycle. Legal range 0 to `ssiQueueEntries`; 0 is legal and
  costs performance only (see the logic section, paragraph 5b). It is NOT a
  correctness knob: the coarse bound of paragraph 5 is the correctness backstop
  whatever this is set to.

  Every other figure comes from `HasBoomCoreParameters` and `HasVectorParams` —
  `lsuWidth`, `numLdqEntries`, `numStqEntries`, `stqAddrSz`, `ldqAddrSz`,
  `robAddrSz`, `corePAddrBits`, `pgIdxBits`, `coreWidth`, `vLen`, `eLen`,
  `maxMembers`, `usQueueEntries`, `ssiQueueEntries` — and no width below is a
  literal. Two derived values are used in the logic section:

    `maxUsBytes   = maxMembers * vLen / 8`                     (256 at defaults)
    `maxRangeSegs = ceil(maxUsBytes / pageBytes) + 1`

  Require `maxUsBytes <= pageBytes`, which pins `maxRangeSegs` at 2: a unit-stride
  range is at most one whole LMUL=8 group of bytes and so spans at most two pages.
  The require is what stops a future `vLen` from turning the per-page presentation
  of paragraph 3 into an unbounded walk.

  There is deliberately no FSM, timeout or threshold parameter.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow Chisel's implicit convention: posedge `clock`,
  ACTIVE-HIGH SYNCHRONOUS `reset`. Every compare path is combinational within the
  existing LCAM cycle; reset reaches only the summary and window registers, and
  clears them to "nothing generated", which is the safe pole for the summary
  (empty means no match, and a match is re-established by the store's next
  presentation, which must happen before it commits).

  `io.cand` — `Vec(searchPorts, Flipped(Decoupled(VecSnoopCandidate)))`. On each
  granted lane VecLsu offers the entry that pass read out of one of the four
  vector ADDRESS queues (`st_SSI_ADDR_Q`, `st_US_ADDR_Q` for the ST->LD direction;
  `ld_SSI_ADDR_Q`, `ld_US_ADDR_Q` for the load-initiated one). Fields: `is_store`;
  `is_unit_stride` (the payload is a `VecRangeEntry`, i.e. US) else one element
  `nOP.v`; the TRANSLATED physical `addr`; `len` in bytes (from the range entry);
  `eew`; `active_mask`, the entry's active BYTE mask; the owning `uop` (`rob_idx`,
  `ldq_idx`, `stq_idx`, `next_stq_idx`, `br_mask`, `mem_size`); the entry's
  absolute `queue_idx`, its `ordinal` within the store's reservation region and
  that region's `q_base`; `members` for a US store; and `data_filled`, the paired
  `st_*_DATA_Q` entry's `filled` bit, meaningful only when `is_store`. `ready` is
  the arbiter's LCAM grant, AND-ed on a store candidate with `data_filled`
  (paragraph 2). This module adds NO read port to any VecElemQueue — the candidate
  arrives already read, on the shared `io.rd` port VecElemQueue reserved for the
  disambiguation and forwarding consumers.

  `io.lcam` — `Vec(searchPorts, Valid(VecLcamSearch))`, reaching `lsu.scala`
  through `VecLsuCoreIO`. The vector equivalent of what `fired_store_agen` /
  `fired_load_agen` produce for a scalar op, driving the SAME signals:
  `do_st_search` / `do_ld_search`, `lcam_addr`, `lcam_uop`, `lcam_mask`,
  `lcam_stq_idx`, `lcam_ldq_idx`. Fields: `is_store_search`, `is_load_search`,
  `paddr`, `byte_mask`, `is_range`, the dword-granular bounds `range_lo` /
  `range_hi`, and the placeholder `uop` carrying the age identity. `is_range` is
  the same bit as the candidate's `is_unit_stride`, spelled for what the consumer
  does with it: the LSU selects a range comparator, VecStoreForward selects a data
  queue.

  `io.ld_search` — `Vec(lsuWidth, Flipped(Valid(...)))`, the LCAM-stage load
  searcher: `paddr`, `byte_mask` (`lcam_mask`), `uop`, `ldq_idx` (`lcam_ldq_idx`),
  `next_stq_idx`, plus `is_vec` / `is_unit_stride` / `range_base` / `range_len` for
  a vector load, and `stq_age_mask` — the LSU's EXISTING per-STQ-entry "older than
  me" mask (`age_matches`, from `stq_head` and `lcam_next_stq_idx`). This is the
  SAME tap the LSU delta gives VecStoreForward as its `io.ld_search`; both modules
  must see a given lane in the same cycle. Reusing `stq_age_mask` rather than
  recomputing age is deliberate — a second age comparator that disagreed with the
  first would forward from the wrong side of a store.

  `io.stq_vec_valid` — `Vec(numStqEntries, Bool())`, the LSU's
  `stq_valid && stq_uop.is_vec`. It qualifies every summary and window entry, so
  this module keeps no second copy of STQ validity and a squashed placeholder
  cannot produce a match.

  `io.snoop_cand` — `Vec(lsuWidth, Vec(numStqEntries, Valid(...)))`, out, the
  contract VecStoreForward declares: ONE candidate per in-flight vector store,
  INDEXED BY THAT STORE'S STQ PLACEHOLDER, already reduced to the store's youngest
  hitting entry. Fields as that module enumerates them — `is_unit_stride`,
  `queue_idx`, `ordinal`, `entry_paddr`, `entry_active_mask`, and for the
  unit-stride class `us_data_base` and `members`. Valid means EXACT (paragraph 6);
  a coarse-only match is reported on the next port instead.

  `io.vst_addr_match` — `Vec(lsuWidth, UInt(numStqEntries.W))`, out: the
  CONSERVATIVE superset of older vector stores this load may overlap. It ORs into
  the LSU's existing `ldst_addr_matches`, which is also how it reaches
  VecStoreForward — through that module's existing `io.stq_addr_matches` input,
  with no interface change on its side.

  `io.stq_alloc` — `Vec(coreWidth, Valid(stq_idx))`, the LSU's allocation of a
  vector store's STQ placeholder, which clears that summary entry and invalidates
  any window entry tagged with it.
  `io.brupdate` / `io.rob_flush` — BOOM's existing `BrUpdateInfo` and flush pulse,
  used only to suppress a presentation for a uop killed in the presentation cycle,
  via the existing `IsKilledByBranch` helper.

  ===> NOT PORTS, and each absence is a decision. No port toward VecOrderHold:
  its `known_overlap` comes verbatim from VecStoreForward, whose forwarding and
  hold predicates must stay exact complements, and a second source would let the
  two disagree. Nothing here may be gated by a hold either — `order_fail` must
  never be delayed or suppressed by one. No `busy`, `active` or "search in
  progress" output: nothing here may reach an issue unit (plan section 5 rule 6;
  grep gate H4). No `order_fail` output — the bit lives in the LDQ and `lsu.scala`
  sets `ldq_order_fail(i)` off the presentation above, so spec-memord.a6/a7 are
  the LSU delta's obligations, not this node's. No `lxcpt` or ROB port: the
  mini-exception rides the LSU's existing `r_xcpt`. No D$, TLB or VRF port, and no
  queue read port of its own.
  <|end_ports|>

  <|begin_logic|>
  //@req-spec-memord.a2
  ---- 1. Both directions, and neither half is optional ----

  Vector element and range addresses are explicitly routed through the
  disambiguation machinery so RVWMO ordering between scalar and vector memory ops
  holds in BOTH directions.

  ST->LD (this node's own): every vector store address presented for execute
  becomes an ordinary store searcher — same `do_st_search`, same `lcam_addr`, same
  age comparators — so it is compared against every LDQ entry, scalar entries and
  vector-load placeholders alike, and a match on a younger already-executed load
  sets that load's `order_fail`. The presentation is INDISTINGUISHABLE from a
  scalar store addr-gen's apart from the range fields of paragraph 3, which is
  what lets the LSU's existing per-entry loop do the work unchanged.

  LD->ST (paragraph 5): every load address presented for execute is searched
  against the vector store address queues as well as the scalar STQ. This half is
  correctness-bearing, not an optimization: a vector store drains POST-COMMIT, so
  a load that missed an older vector store's already-generated address would read
  stale memory from a D$ that does not yet hold the store, and nothing would ever
  replay it.

  Scalar-vs-scalar disambiguation is UNCHANGED. The vector terms are additive,
  and a vectors-off build has none of them.

  //@req-spec-memord.a22
  ---- 2. The presentation gate: no address without its data ----

  A vector store address queue entry is NOT presented to the LCAM until its
  corresponding `st_*_DATA_Q` entry is valid. `io.cand[i].ready` is the arbiter's
  LCAM grant AND-ed with `data_filled` for a store candidate, so an entry whose
  data half was not yet captured at DGEN does not win the port and is re-offered
  later.

  This is not conservatism for its own sake — it makes a failure mode unreachable.
  Without it a load could match a store address whose data does not exist yet, and
  every consumer would need a third answer besides "forward" and "no match":
  "match, but come back later". With the gate, every store address a load can
  match is data-backed by construction, so VecStoreForward never has to represent
  a data-pending store and paragraph 5's summary can never name one.

  LIVENESS, and ORDER_FAIL STILL IN TIME — the two objections to the gate. It
  cannot deadlock: the data half is written by VecDgen out of VRF port R3/R4
  into the data queue, a path needing neither LCAM, TLB nor D$, so it completes
  independently of every grant this module competes for. And the delay cannot
  lose an order_fail: a vector store's address pass is PRE-COMMIT (its fault
  must be reported precisely before the store) and the store is older than any
  load it can fail, so it presents before it commits, and it commits before
  that younger load can reach the ROB head where the flush fires.

  //@req-spec-memord.b1
  //@req-spec-memord.b2
  //@req-spec-memord.b3
  //@req-spec-memord.b4
  //@req-spec-memord.b5
  ---- 3. Unit-stride: ONE range-overlap test, not per element ----

  A US access, load or store, is the CONTIGUOUS BYTE RANGE
  `[base, base + VL*EEW)`. Take `base` and the byte length from the
  `VecRangeEntry` the candidate carries and do NOT re-derive `VL*EEW` here:
  VecRangeAgen already normalized plain unit-stride (`vl << eew`), whole-register
  (`nregs * vLen / 8`) and mask (`ceil(vl/8)`) onto that one `len` field, and a
  second derivation would get the two length-independent classes wrong.

  The LCAM comparison for that access is a RANGE-OVERLAP TEST of the whole range
  against each queue entry's address, PERFORMED ONCE when the US entry executes.
  Present `range_lo = base >> 3` and `range_hi = (base + len - 1) >> 3` and let
  each per-entry comparator evaluate

      overlap = (entry_addr >> 3) >= range_lo && (entry_addr >> 3) <= range_hi

  in place of the baseline's `dword_addr_matches` equality. Dword granularity is
  chosen because it is exactly the granularity BOOM's comparator already works at,
  so the range form costs two magnitude compares per entry instead of one equality
  and introduces no new address decomposition.

  ANY OVERLAP ANYWHERE IN THE RANGE triggers the ordering or forwarding action.
  There is no partial-coverage answer on this path: coverage is VecStoreForward's
  problem — it replays a load a store cannot fully cover — and for ordering a
  single overlapping byte is a dependence.

  ===> THE RANGE TEST MUST NOT EXPAND INTO PER-ELEMENT SEARCHES, AND MUST NOT
       EXPAND INTO PER-BEAT SEARCHES EITHER. The second half is the likelier
       mis-generation: VecBeatExpander legitimately performs one TLB lookup per
       beat at drain, and it is a short step from there to one LCAM search per
       beat. That reintroduces exactly the cost this requirement removes — up to
       `vLen` searches for one instruction, 256 at the defaults — and starves
       scalar disambiguation through the shared port. The disambiguation pass and
       the drain passes are DIFFERENT PASSES: the search happens once, at the
       entry's execute-time LCAM/TLB pass, and is never repeated when the beats
       later fire.

  The one bounded exception is translation, not element expansion. The comparison
  is on PHYSICAL addresses (BOOM's LCAM compares physical addresses and guards on
  `addr_is_virtual`), and a range of up to `maxUsBytes` bytes can straddle a page
  boundary, so a page-crossing range presents one search per page-resident
  segment — at most `maxRangeSegs` = 2, taking the second segment's translation
  from the same DTLB port the arbiter grants alongside the LCAM port. Two is
  bounded by PAGES SPANNED and is independent of VL, EEW and the mask; an
  implementation that turns it into a loop over elements has failed this paragraph.

  //@req-spec-memord.b20
  ---- 3b. Mask-obliviousness, and the asymmetry with forwarding ----

  The range-overlap test is deliberately MASK-OBLIVIOUS for ordering: the bounds
  come from `base` and `len` alone and the entry's element-granular mask is not
  consulted, not even to trim the last active element. It over-approximates on
  purpose — a masked-off lane inside the range still produces a match.

  ===> DO NOT "FIX" THIS BY AND-ING IN THE MASK. Note the asymmetry with
  VecStoreForward, which IS byte-mask qualified (memord.b21): the two paths fail
  differently. A conservative ordering match costs a replay or a hold, i.e.
  cycles; a wrongly forwarded byte is silent data corruption. So the ordering
  path rounds outward and the forwarding path rounds inward, and the same range
  entry is read both ways by design. The entry's `active_mask` is still carried
  through to `io.snoop_cand.entry_active_mask` — unused for ordering, and the
  only consumer that may qualify with it is the forwarding path.

  //@req-spec-memord.b8
  ---- 4. Strided / indexed / segmented: per element, through the arbiter ----

  Scattered addresses cannot be range-folded, so an SSI access searches the LCAM
  PER ELEMENT `nOP.v` as its elements drain: one candidate, one presentation, one
  set of comparisons per element per granted lane, with `is_unit_stride` low and
  the element's own byte mask. No folding and no summarizing on this path — for a
  store the per-element search is what raises `order_fail` against loads that
  speculated past it.

  Those per-element searches contend for the shared LCAM through the SAME arbiter
  that gates the D$ port, and this module holds no private path around it: it
  requests, waits for the grant, and accepts one candidate per granted lane per
  cycle. That is what stops a 256-element gather from starving scalar
  disambiguation, and it is why "as they drain" is the right description — the
  search rate IS the grant rate. VecDcacheArbiter owns the policy.

  For a STORE, "as they drain" means as elements drain through the shared
  LCAM/TLB port on the PRE-COMMIT translate pass, not on the post-commit write
  pass: the physical address exists only after translation, and order_fail must
  be discovered long before the store's data reaches memory.

  //@req-spec-memord.a19
  ---- 5. The load-initiated search against the vector store queues ----

  Each load address presented for execute is searched against the vector store
  address queues in addition to the scalar STQ, and the search has two tiers
  because one tier cannot be both exact and affordable.

  //@req-spec-memord.a26
  Tier 1, the correctness backstop, is one summary per STQ entry indexed by the
  vector store's EXISTING STQ placeholder: `numStqEntries` entries of
  { `valid`, `is_ssi`, `lo`, `hi`, `q_base`, `q_gen_end` }, where `lo`/`hi` are the
  dword-granular bound over every address that store has PRESENTED so far.
  Updated on each accepted store candidate of paragraph 2 (widen the bound,
  advance `q_gen_end`), cleared by `io.stq_alloc`, qualified at every read by
  `io.stq_vec_valid`. The load-side test is, per STQ entry:
  `valid && io.stq_vec_valid(i) && io.ld_search[w].stq_age_mask(i)` and an overlap
  of the load's access — or, for a vector US load, its whole range — against
  `[lo, hi]`. That is `io.vst_addr_match`.

  For a US store the bound is EXACT: one presentation, and the bound IS the range.
  For an SSI store it is a strict SUPERSET of the generated element addresses,
  which is what makes it sound. A superset yields no false negatives, so no
  aliasing load is ever missed; a false positive costs a sleep-and-retry, never
  data, exactly as in paragraph 3b.

  ===> WHY A SUMMARY AND NOT A CAM OVER THE QUEUE. `st_SSI_ADDR_Q` is 512
  entries in a `SyncReadMem` — VecElemQueue's decision, because a flop array
  plus a 512:1 mux is the wrong structure — and it carries no per-entry age. A
  broadcast compare against it is therefore not possible, not merely expensive,
  so SOME bound is forced. This summary is also the one piece of state in this
  file and it does NOT break the vector-LSU invariant: it is scoped to an STQ
  ENTRY, not to "the current instruction", with no FSM, no cursor, nothing to
  retire and no `busy` export. It is the same shape as the per-entry address
  state the STQ already keeps for scalar stores.

  ---- 5b. Tier 2: the exact comparator arrays ----

  //@req-spec-memord.a27
  Tier 2 answers WHICH entry hit, which is what forwarding needs. Two flop-backed
  comparator arrays, both evaluated in the same LCAM cycle:

    - All `usQueueEntries` entries of `st_US_ADDR_Q` (16 at the defaults, already a
      `Reg(Vec(...))` in VecElemQueue), each compared as a range-overlap per
      paragraph 3. Exact for the whole class, which is where vector-to-vector
      forwarding is permitted at all.
    - A `ssiSnoopWindow`-entry window of the most recently presented SSI store
      element addresses, held HERE in flops, each tagged with `stq_idx`,
      `ordinal`, absolute `queue_idx` and `active_mask`, written on each accepted
      SSI store candidate and invalidated on `io.stq_alloc` for its `stq_idx` or
      when that store's region is freed.

  A store's hits are reduced to ONE candidate — the highest `ordinal`, because
  ordinals rise monotonically with program order inside a store's program-ordered
  reservation region, so "youngest matching element" is "highest matching
  ordinal". That reduction happens here, not in VecStoreForward, and the rule it
  implements is that module's requirement, not this file's.

  The window is a PERFORMANCE structure with a correctness-free failure mode,
  and `ssiSnoopWindow = 0` is a legal configuration. A load that tier-1 matches
  but tier-2 misses simply gets `io.vst_addr_match` set with no
  `io.snoop_cand`, and joins BOOM's EXISTING "matched a store I cannot forward
  from" path: not marked executed, sleeps, retries — the same treatment a
  scalar load already gets against an unforwardable STQ entry, and no new
  mechanism (plan section 5 rule 10). Sizing it is a measurement question for
  Phase H, not a correctness one.

  //@req-spec-memord.a15
  ---- 6. What a candidate hands to the forwarding path ----

  A match on an older vector store names WHERE the data is, and the answer is
  never the STQ: a vector store's STQ entry holds no store data at all, because a
  `vLen`-wide payload per STQ slot is not a feasible structure. Forwarded vector
  store data is read from the `st_*_DATA_Q`, which already holds it, captured at
  DGEN. So the candidate carries the DATA-QUEUE addressing and nothing else could
  serve: `is_unit_stride` selects `st_US_DATA_Q` (a full `vLen` member, located by
  `us_data_base` and `members`) over `st_SSI_DATA_Q` (one `eLen` element, located
  by `ordinal`, which is the same ordinal as the matched ADDRESS entry because
  VecQueueReservation claims a store's address and data regions together, in
  program order, with equal counts).

  Everything downstream is VecStoreForward's: ranking stores by age, the store's
  active byte mask, partial cover, the US/US-only restriction on vector-to-vector
  forwarding, and driving VecOrderHold's `known_overlap`. None of it is specified
  here, and this module exports no opinion about which pairs may forward.

  ---- 7. Speculation and recovery are reused, not rebuilt ----

  Memory-dependency speculation — a load issuing past a store whose address is not
  yet known — REUSES BOOM's existing memory-dependence predictor and its existing
  ordering-violation replay path unchanged. This node extends only what FIRES that
  path: the cross-queue matches above. No new predictor, no new replay mechanism,
  no selective replay.

  ===> AND THAT MAKES A CII HOLE REACHABLE — a dependency, not a side note. An
  order-fail replay raises MINI_EXCEPTION_MEM_ORDERING at the ROB HEAD and
  flushes with flush_typ = refetch. A load's `rob_unsafe` is cleared on its
  first address translation while `order_fail` is discovered later, so the PNR
  sweeps past it, vector arithmetic issues to the coprocessor, and the flush
  then squashes that past-PNR work. Extending order_fail to cross-queue matches
  makes this MORE frequent than in baseline BOOM, not less. VecCiiFlush closes
  the hole; this node is not a correctness story on its own.

  ---- 8. Tracing and assertions ----

  With no unit tests anywhere in this project (plan section 5 rule 11) these
  searches are only observable from a trace. Emit guarded `VecTrace` lines, one per
  key event, each tagged with the module name and `rob_idx`, gated on the
  `vecTrace` plusarg and `!reset` and so off by default: one per accepted
  presentation (direction, class, `paddr` or the range bounds, segment index); one
  per ST->LD match that sets a load's `order_fail`, carrying both identities; one
  per LD->ST tier-1 match, with the matched `stq_idx` and whether tier 2 produced a
  candidate; and one per candidate refused by the paragraph-2 data gate. Insist on
  the last two: a search that never happened, a search that found nothing, and a
  hit the window had evicted look identical in a waveform, and the difference
  between them is the whole bug class this node can produce.

  Assert rather than assume: a candidate accepted with `is_store` set always had
  `data_filled`; a presentation with `is_range` set always has
  `range_hi >= range_lo`; and a tier-2 candidate's `stq_idx` always has its tier-1
  summary bit set, which is the invariant that makes the superset claim testable
  rather than merely asserted in prose.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput is set by the arbiter, not here: one search per granted LCAM lane per
cycle, `searchPorts` lanes. The figures that matter are SEARCHES PER INSTRUCTION,
and they are constraints rather than targets:

  unit-stride, any VL / EEW / mask : 1 per page spanned (<= 2)
  SSI                              : 1 per active element

An implementation whose unit-stride count scales with VL, EEW or beat count has
failed this file regardless of what it measures — that is the `addvector`
behaviour (one access, and one search, per element) that targets P1 and P4 exist
to delete.

Latency: both tiers must resolve within the EXISTING LCAM cycle, and this node may
not add a pipeline stage there. Baseline BOOM sets `ldq_order_fail` and computes
`ldst_addr_matches` in the cycle after translation; a registered match would let a
load write back and be marked safe a cycle before the failure is known, widening
the past-PNR window of paragraph 7 for no benefit.

Timing risk worth naming, because it is the plausible critical path: the per-entry
comparator grows from one dword equality to two magnitude compares of
`corePAddrBits - 3` bits, on a path that already starts at the TLB response.
Precompute `range_lo`/`range_hi` at the searcher — once per presentation, never
once per entry — so `base + len - 1` stays out of every per-entry loop.

Area, and the reason for two tiers: tier 1 is `numStqEntries` x (two dword
addresses + two queue indices + two bits), ~3 kbit at default sizing; tier 2 is
`usQueueEntries + ssiSnoopWindow` compared entries, ~32. The rejected alternative
is a 512-entry flop CAM over `st_SSI_ADDR_Q`, unaffordable and incompatible with
VecElemQueue's `SyncReadMem`. `ssiSnoopWindow` is the knob to move if the SSI
forward-hit rate measures low in Phase H, and moving it cannot affect correctness.
<|end_perf|>

<|begin_dependencies|>
MicroOp — every candidate and searcher carries `new MicroOp()`; this module reads
`rob_idx`, `ldq_idx`, `stq_idx`, `br_mask`, `mem_size`, `is_vec`, `v_eew` and the
access-class flags `v_is_unit_stride` / `v_is_strided` / `v_is_indexed` /
`v_is_segment`, re-decodes `uop.inst` NOWHERE, and writes no uop field.

VecBundles — `VecRangeEntry` (the range candidate's payload; `base`, `len`, `eew`,
`is_unit_stride` and the element mask are what paragraphs 3 and 6 consume) and
`VecElemAccess` (the SSI element candidate). `VecSnoopCandidate` and
`VecLcamSearch` are contracts with two sides — VecLsu and the LSU delta on one,
VecStoreForward on the other — so they belong in VecBundles rather than here.
Reported as a seam addition. The `io.snoop_cand` payload is VecStoreForward's
declared bundle, adopted field for field rather than re-invented.

VectorParams — `vLen`, `maxMembers`, `eLen`, `usQueueEntries`, `ssiQueueEntries`
and the `maxUsBytes` / `maxRangeSegs` derivations. VecTrace — the guarded trace
helpers.

Instantiates nothing. Its seams, and what each side owes:
  - LSU (`src/main/scala/v4/lsu/lsu.scala`, edit_existing) consumes `io.lcam` into
    `do_st_search` / `do_ld_search` / `lcam_*`, extends its per-LDQ-entry and
    per-STQ-entry comparators with the range predicate of paragraph 3, ORs
    `io.vst_addr_match` into `ldst_addr_matches`, drives `io.ld_search`,
    `io.stq_vec_valid` and `io.stq_alloc`, and owns setting `ldq_order_fail` and
    raising the `lxcpt`.
  - VecStoreForward consumes `io.snoop_cand` — one candidate per in-flight vector
    store, indexed by STQ placeholder, already reduced to the youngest hitting
    entry — and sees coarse-only matches through the LSU's `ldst_addr_matches`.
    It, not this module, drives VecOrderHold.
  - VecOrderHold is deliberately NOT a counterparty in either direction, matching
    its own dependency note: `order_fail` must never be gated, delayed or
    suppressed by a hold.
  - VecDcacheArbiter grants the LCAM lane and owns the priority-floor policy.
  - VecElemQueue supplies the candidate on the shared `io.rd` port
    (`readPorts = ports + 1`), whose `resp.filled` bit is the paragraph-2 gate.
    NOTE the sharing: this node and VecStoreForward both consume that one port on
    the store queues, so VecLsu must mux them — two independent readers would be a
    port conflict.
  - VecRangeAgen / VecElemAgen / VecDgen produce the entries searched here;
    `is_unit_stride`, an exact `len`, and the positional address/data pairing by
    ordinal are what make paragraphs 3, 5b and 6 correct.
  - VecCiiFlush is the dependency of paragraph 7 — plan step G2 must not land
    before F6.
<|end_dependencies|>

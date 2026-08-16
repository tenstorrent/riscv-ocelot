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
  VecStoreForward — the LD->ST forwarding decision and data return for a load
  that matched an OLDER VECTOR store: which pairs may forward, which element
  supplies the bytes, and when the load is replayed instead.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecStoreForward.scala,
  package boom.v4.vec.generated.lsu, group vec_lsu.
  depends_on MicroOp, VecBundles, VectorParams, VecTrace.
  Instantiated once by VecLsu as `fwd`, inside the `usingRVV` gate — in a
  vectors-off build the module is ABSENT, not tied off.

  ===> IT RETURNS DATA. THAT IS WHY IT IS NOT VecCrossLsuSnoop. The two
       directions of cross-queue ordering were one node until the v2 amendment:
       the ST->LD direction raises `order_fail` and is pure control, while this
       direction hands the load ACTUAL BYTES. A wrong ordering decision costs a
       replay; a wrong forwarded byte is silent data corruption that no replay
       recovers. Every asymmetry below follows from that, and the mask asymmetry
       of logic paragraph 5 is the sharpest case.

  ===> THE DATA COMES OUT OF `st_SSI_DATA_Q` / `st_US_DATA_Q`, NEVER OUT OF THE
       STQ ENTRY. A vector store's STQ slot is an ordering and commit PLACEHOLDER
       holding no store data at all — a `vLen`-wide payload per STQ slot is not a
       feasible structure, which is why the data queues exist. Code copied from
       baseline `lsu.scala`'s forward path reads `stq_data(s_idx)`; for a vector
       store that register is meaningless, and reading it is the likeliest
       mis-generation of this file.

  ===> IT OWNS NO COMPARATOR ARRAY. Searching each load address against the vector
       store address queues is spec-memord.a19, allocated to VecCrossLsuSnoop.
       This module consumes that search's candidates and owns the DECISION and the
       DATAPATH; the dependencies section states the contract exactly.

  Governing spec anchors: loadstore.rst `mem-order` (which pairs forward, search
  granularity, mask qualification) and `order-fail-replay`; midcore.rst
  `vrf-ports` — where this module deliberately does not appear, because it adds
  no VRF port; caracal-milestone-plan-v2.md section 5 rules 3 and 10.

<|begin_module|>

  <|begin_parameters|>
  Elaborated only when `usingRVV` is true — a Scala `Boolean` from
  `BoomCoreParams`, never a hardware `Bool`, and never rocket's `usingVector`.

  Two constructor parameters, one per CONSUMER CLASS, because the two consumers
  have different partners and collapsing them into one switch over-couples this
  module to VecOrderHold. Forwarding is a PERFORMANCE mechanism layered on the
  `order_fail` floor, so the machine must be correct with either off; with no unit
  tests in this project, switches that isolate this module are the only way to
  attribute a cosim `MISMATCH` to it rather than to the snoop or the queues.

  `enableVecStoreForward` (Boolean, default true): the SCALAR-consumer forward of
  logic paragraphs 3-8, `io.fwd_resp`. When false every eligible scalar forward
  becomes the replay of logic paragraph 6. It has NO partner in VecOrderHold —
  the hold only ever admits a load with `is_vec` set, so a scalar load's forward
  and the hold can never both apply to the same load, and the "exact complements"
  argument below does not reach this switch.

  `enableVecBeatForward` (Boolean, default true): the UNIT-STRIDE VECTOR-consumer
  forward of logic paragraph 9, `io.fwd_beat`. THIS is the switch that must equal
  VecOrderHold's `forwardingEnabled`, and only this one: paragraph 9's US->US pair
  is exactly the pair the hold excludes when its `forwardingEnabled` is true. With
  the two disagreeing in the direction (hold excludes, beat forward off) a US load
  overlapping a US store neither forwards nor waits and reads stale data.
  Elaboration asserts the pairing rather than trusting the instantiator, since
  VecLsu is the only instantiator and a silent mismatch here is data corruption.

  Everything else comes from the existing traits and no width below is a literal:
  `lsuWidth`, `numStqEntries`, `corePAddrBits`, `coreDataBytes`, `xLen`,
  `stqAddrSz`, `ldqAddrSz` from `HasBoomCoreParameters`; `vLen`, `eLen`,
  `maxMembers`, `vecPregSz`, `ssiQueueEntries`, `usQueueEntries` from
  `HasVectorParams`. `vLenBytes = vLen / 8` and `elenBytes = eLen / 8` are
  derived — never written as 32 or 8.

  `lsuWidth` must equal the LSU's own `lsuWidth` (1 on Medium, 2 on Mega) and
  VecDcacheArbiter's lane count. It is not an independent knob: a mismatch
  silently drops a lane's forward.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the map's `defaults:` and Chisel's implicit convention:
  posedge `core_clk`, ACTIVE-HIGH SYNCHRONOUS `core_reset`, both implicit. No
  second clock and no `reset_n` — the active-low inversion for the SV coprocessor
  happens once, in `tt_cii_host_wrap`. Every port is `Vec(lsuWidth, ...)`: one
  independent lane per LCAM search lane, with no arbitration between lanes here.

  `io.ld_search` — Flipped Valid, the load-side search context, tapped from the LSU
  delta's existing LCAM stage and carrying only signals that already exist there:
  `paddr` (physical `lcam_addr`), `ld_mask` (the load's byte mask), `uop`,
  `ldq_idx` (`lcam_ldq_idx`), `next_stq_idx`, `can_forward` (the uncacheable
  qualification) and `kill_forward`. A VECTOR load additionally carries `is_vec`,
  `is_unit_stride`, `range_base` and `range_len` — one range, because a unit-stride
  access is one `VecRangeEntry` and its search is one range-overlap test.

  `io.snoop_cand` — Flipped, `Vec(numStqEntries, Valid(...))` per lane: the
  candidates VecCrossLsuSnoop's search produced, indexed BY THE MATCHING VECTOR
  STORE'S STQ PLACEHOLDER INDEX. Each carries `is_unit_stride`, `queue_idx` (the
  absolute `st_SSI_ADDR_Q` / `st_US_ADDR_Q` index of the youngest hitting entry of
  that store), `ordinal` (its position in the store's reservation region),
  `entry_paddr`, `entry_active_mask` (paragraph 5) and, for the unit-stride class,
  `us_data_base` and `members`.

  Indexing candidates by STQ index is what lets this module reuse BOOM's
  existing age network instead of adding one. A vector store holds exactly ONE
  STQ placeholder, so scalar and vector candidates share one numStqEntries-wide
  space and a single ForwardingAgeLogic ranks both.

  `io.stq_addr_matches`, `io.stq_forward_matches` — inputs, `UInt(numStqEntries.W)`
  per lane: the LSU's existing `ldst_addr_matches` and `ldst_forward_matches`, so
  the select of paragraph 4 sees scalar and vector stores in ONE comparison rather
  than picking a winner per class and reconciling afterwards.

  `io.st_ssi_rd`, `io.st_us_rd` — the SHARED indexed read port of `st_SSI_DATA_Q`
  and `st_US_DATA_Q` (`VecElemQueue`'s `io.rd`, the extra port beyond the drain
  lanes): `req.valid` + `req.idx` out, `resp.data` (`eLen` and `vLen`) +
  `resp.filled` back ONE REGISTERED CYCLE LATER. No port is added and neither queue
  is ever written from here.

  `io.fwd_resp` — Valid out, a SCALAR load's forwarded result: `ldq_idx`, `uop`,
  `data` (`xLen`, aligned and extended), plus the LDQ bookkeeping the replay path
  needs — `forward_std_val` and the forwarding store's `forward_stq_idx`. The LSU
  delta drives `iresp`/`fresp` and the slow wakeup from it, on the existing
  writeback path and in the same cycle as a scalar forward.

  `io.fwd_beat` — Valid out, a VECTOR load's forwarded beat, shaped exactly like a
  D$ response beat so the LCB needs no second placement path: destination `prn`,
  byte offset within that PRN, `data`, byte mask, `rob_idx`, `ldq_idx`, `last`.

  `io.replay` — Valid out, `ldq_idx`: "matched an older vector store but cannot
  fully forward". Reuses the LSU's existing suppression (`io.dmem.s1_kill`,
  `ldq_executed` left clear); no new replay mechanism.

  `io.known_overlap` — Valid out, the overlapped store's `stq_idx` and
  `is_unit_stride`: the KNOWN classification of paragraph 2, published for
  VecOrderHold so the hold and the forward run off ONE match rather than two
  searches that can disagree.

  `io.brupdate`, `io.rob_flush` — BOOM's existing `BrUpdateInfo` and the ROB flush
  pulse, qualifying the outputs for a load killed between the search and response
  cycles through the existing `IsKilledByBranch` helper.

  ===> AND THAT IS THE WHOLE INTERFACE. No `busy`, no `active`, no "current load"
  identifier, no VRF port, no D$ port, no TLB port, no LCAM port of its own. It
  holds no state scoped to an instruction: its only registers are the one-cycle
  search-to-response pipeline registers of paragraph 1, overwritten every cycle.
  <|end_ports|>

  <|begin_logic|>

  ---- 1. Where it sits, and the timing it must not extend ----

  Two cycles wide, reusing baseline BOOM's forward timing exactly. In the SEARCH
  cycle it takes `io.ld_search` and `io.snoop_cand`, selects the winning store,
  decides forwardability, and issues the data-queue read — the read index comes
  combinationally from the winner's `queue_idx`, so no cycle is spent finding it.
  In the RESPONSE cycle the payload returns, is aligned, and drives `io.fwd_resp`
  or `io.fwd_beat` in the same cycle baseline BOOM drives `wb_ldst_forward_valid`
  and its `LoadGen`. The registers between are the pipelined search context only —
  uop, `ldq_idx`, load address, load mask, winning `stq_idx` — the direct
  counterparts of `wb_ldst_forward_e`, `wb_ldst_forward_ldq_idx`,
  `wb_ldst_forward_ld_addr` and `wb_ldst_forward_stq_idx`. A D$ response on the
  same lane in the response cycle TAKES PRECEDENCE and the forward is dropped, as
  in baseline BOOM; resolving that collision with a third cycle instead would put
  this module on the load-to-use path of every scalar load in the machine.

  ---- 2. What counts as a KNOWN overlap ----

  //@req-spec-memord.b15
  An overlap with an older vector store is KNOWN when the LCAM matches an
  ALREADY-GENERATED store element address, or the store's unit-stride RANGE,
  against the load's address. Both halves matter. The SSI form is per element and
  covers only elements the store's AGEN has actually pushed into `st_SSI_ADDR_Q` —
  a not-yet-generated element is unknown by construction, which is the whole
  reason paragraph 7 exists. The US form is one range-overlap test of
  `[base, base + total_bytes)` against the load's bytes, evaluated once, because a
  unit-stride access is a single `VecRangeEntry`.

  This classification is exported on `io.known_overlap` and has two consumers: the
  forwarding decision below, and VecOrderHold's "Known" input for the three
  combinations that do not forward. A second, independently-built overlap test
  could hold a load this module had already forwarded, or forward one the hold
  believed was waiting.

  A store address entry is not presented to the LCAM until its st_*_DATA_Q
  entry is valid (the snoop's obligation, not this module's), so "matched a
  store whose data is not captured yet" is UNREACHABLE here. `resp.filled` is
  checked as an assertion, never as a functional stall condition.

  ---- 3. Which pairs are eligible ----

  //@req-spec-memord.a23
  //@req-spec-memord.b10
  //@req-spec-memord.b14
  Decided from the two access classes alone, before any data is read:

    load     store    forward?
    scalar   US       YES, via io.fwd_resp   (gated by enableVecStoreForward)
    scalar   SSI      YES, via io.fwd_resp   (gated by enableVecStoreForward)
    US       US       YES, via io.fwd_beat   (gated by enableVecBeatForward)
    US       SSI      NO   -> VecOrderHold
    SSI      US       NO   -> VecOrderHold
    SSI      SSI      NO   -> VecOrderHold

  A row whose switch is false falls to VecOrderHold if the load is a vector load,
  and to logic paragraph 6's replay if it is scalar. Those two are not
  interchangeable and the choice is forced by which one the load can survive: a
  held SCALAR load would occupy an LDQ entry the hold has no release event for
  (the hold releases on the older store's drain, keyed on a vector store, and its
  `ld_ctx` admission requires `is_vec`), and a REPLAYED vector load re-fails on
  every retry until the store commits, which is the replay storm the hold exists
  to remove.

  ===> SO THERE ARE TWO ELIGIBILITY PREDICATES HERE, NOT ONE, AND COLLAPSING THEM
       IS THE MIS-GENERATION TO WATCH FOR. The table above selects the FORWARDING
       POOL — the candidate bits that feed the youngest-forwarder reduction — and
       its scalar row IS gated by `enableVecStoreForward`. The predicate that
       gates `io.replay` is a DIFFERENT one and its scalar row is NOT gated by any
       switch: with the scalar forward off, a scalar load with a known overlap
       must STILL replay, because replay is the floor the forward sits on top of
       and turning the forward off may not turn the floor off. Written out, the
       replay predicate is
         `!is_vec || (ld.is_unit_stride && winner.is_unit_stride &&
                      enableVecBeatForward)`
       — scalar always, US/US only while the beat forward owns that pair, and
       never for the three rows VecOrderHold owns. Deriving `io.replay` from the
       pool predicate instead makes a scalar load with the switch off proceed to
       the D$ silently, which is the one outcome neither mechanism catches.

  A SCALAR load forwards from an older vector store of EITHER class. A
  vector-to-vector forward is attempted ONLY when BOTH sides are unit-stride, and
  in particular NEVER between two SSI accesses. The asymmetry is deliberate, not
  an unfinished generalization: a scalar load costs at most ONE replay if the
  forward is declined, so declining is cheap and forwarding is worth building,
  whereas a multi-element vector load would re-fail on every replay until the
  store commits — so for it the design WAITS instead, which is a different
  mechanism in a different module.

  The two reasons the vector-consumer rule is this narrow differ and must not be
  collapsed. An SSI STORE cannot be forwarded from at vector granularity because it
  generates element addresses incrementally, so until it has resolved every active
  element a younger load cannot know whether a not-yet-generated element aliases
  it, nor which element holds the youngest byte for an aliased address. A US store
  to an SSI LOAD is safe in principle and is declined only to avoid building a path
  that range-checks and slices every load element against the store's range.

  The three ineligible pairs produce io.known_overlap and NOTHING else — no
  io.replay and no data read. Suppressing the load is the hold's business, and
  asserting io.replay here would replay a load VecOrderHold is already holding,
  turning a clean serialization back into the replay storm the hold removes.

  ---- 4. Selecting the winner, and where the bytes come from ----

  //@req-spec-memord.a21
  //@req-spec-memord.a24
  A load-address match on an older vector store forwards THAT STORE'S DATA to the
  load. Selection is two-level and both levels are age-ordered.

  Across stores the winner is the YOUNGEST store older than the load, chosen by
  `ForwardingAgeLogic` over the union of `io.stq_addr_matches` and the valid bits
  of `io.snoop_cand`, keyed on the load's `stq_idx`. Because vector and scalar
  stores share one STQ index space this is the existing comparison, unchanged. The
  forward is taken only when the youngest ADDRESS match and the youngest
  FORWARDABLE match are the same store — baseline BOOM's rule, which is what stops
  an intervening non-forwardable store being skipped over.

  Within a single SSI store, where several ALREADY-GENERATED elements alias the
  load, the YOUNGEST MATCHING ELEMENT supplies the data. An older element's value
  is stale: a strided or ordered-indexed store may write one address twice, and
  duplicate indices make that common rather than exotic. Ordinals rise
  monotonically with program order inside a store's program-ordered reservation
  region, so "youngest" is "highest matching ordinal" — a priority select over the
  region, not a new age network. That reduction is evaluated inside the snoop's
  comparator array; this module states the rule the array owes it and consumes one
  candidate per store, and it is the single most important item for Phase R to
  confirm from the snoop's side.

  The bytes are then read from the store DATA queue at the winner's index, never
  from the STQ entry:
    - SSI store: one `eLen` entry of `st_SSI_DATA_Q`, holding exactly one element.
      Its index is the same ORDINAL as the matched address entry, because
      VecQueueReservation claims a store's address and data regions together, in
      program order, with equal entry counts — assert that the two bases agree
      rather than assuming it silently.
    - US store: `st_US_DATA_Q` holds `members` full `vLen`-wide entries for the one
      range entry, so the member holding the load's bytes is
      `(paddr - range_base) >> log2(vLenBytes)` and the read index is
      `us_data_base + member`.
  Alignment into the load's result reuses rocket's `StoreGen`/`LoadGen` pair on the
  load's own `mem_size` and `mem_signed`, exactly as the scalar forward does.

  ---- 5. Mask qualification, and the asymmetry with ordering ----

  //@req-spec-memord.b21
  Forwarding from a unit-stride store is qualified by the store's ACTIVE BYTE
  MASK. Bytes of inactive elements — masked off by `v0`, or beyond `vl` in the tail
  — still hold whatever memory held before and are never forwarded. The mask
  arrives element-granular on the range entry (VecRangeAgen copies the
  once-per-OP.v latched mask in, all-ones for `vm = 1`), so the qualification is:
  expand each mask bit into `1 << eew` byte bits, select the bytes at the load's
  offset within the range, and require the load's byte mask to be a SUBSET of
  them. Anything short of a subset is a partial cover and goes to paragraph 6.

  ===> AND NOTE WHAT IS *NOT* MASK-QUALIFIED. The ORDERING check on the same range
       is deliberately mask-OBLIVIOUS (the snoop's rule, not this module's): it
       over-approximates, and a false positive there costs one replay. Here the
       same over-approximation would hand the load a byte the store never wrote —
       data corruption with no detection anywhere downstream. The two paths
       evaluate the SAME range against DIFFERENT predicates on purpose, and a
       "simplification" that shares one predicate re-opens the masked-store
       forwarding hazard the split was written to close.

  The SSI path needs no equivalent, and it is worth knowing why: a masked-off
  element generates no nOP.v, so it has no st_SSI_ADDR_Q entry and can never be
  a candidate. Absence of the entry IS the mask qualification there.

  ---- 6. Partial cover replays the load ----

  //@req-spec-memord.b6
  //@req-spec-memord.b7
  A forwarding overlap that only PARTIALLY covers the load is handled like any
  partial-forward case in BOOM: the load is REPLAYED rather than given half an
  operand. `io.replay` asserts, the lane's D$ access is suppressed through the
  LSU's existing `s1_kill`, `ldq_executed` is left clear, and the load retries —
  by which time the store has typically drained and it reads the updated line.
  Four conditions reach this path, all partial covers rather than errors:

    - the load's byte mask is not a subset of the store's ACTIVE bytes (para. 5);
    - the matched SSI element is NARROWER than the load, so one `eLen` entry
      cannot cover it;
    - the load's bytes straddle two `st_US_DATA_Q` members, which the single
      shared read port cannot deliver in one cycle;
    - the load's bytes straddle the end of the store's range.

  Replay is the right answer to all four precisely because it is not the
  correctness mechanism — order_fail is. A load that replays without forwarding
  is slow; a load that forwards a wrong byte is broken.

  ---- 7. Forwarding is not a commitment ----

  //@req-spec-memord.a25
  A store element generated AFTER a load has forwarded, and aliasing it, must set
  that load's `order_fail` — through THAT ELEMENT'S OWN ST->LD ordering search, not
  through anything here. This is the consequence of paragraph 2: the forward could
  only consider elements that existed when the load searched, so a later element
  of the same store, or of a younger store that had not yet generated addresses,
  is invisible to it. The replay path stays the correctness floor and the forward
  is a speculation above it.

  What this module owes that mechanism is bookkeeping, and getting it wrong makes
  the failure undetectable. `io.fwd_resp` reports `forward_std_val` and
  `forward_stq_idx`, and `forward_stq_idx` is the forwarding vector store's STQ
  PLACEHOLDER index — never an element-queue index. Baseline `lsu.scala` decides
  whether an incoming store search must fail an already-forwarded load with
  `!l_forward_std_val || ((l_forward_stq_idx =/= lcam_stq_idx) && forwarded_is_older)`;
  that comparison is against STQ indices, so a queue index recorded there would
  make every vector-forwarded load compare against an unrelated store — failing
  loads that were fine, or worse sparing loads that were not.

  A later element of the SAME store the load forwarded from still fails the
  load, because the comparison is stq_idx equality and cannot see that a
  different element of that store is the one now aliasing. Conservative in the
  safe direction: an extra replay, never a missed one.

  ---- 8. The load after an order-fail replay ----

  //@req-spec-memord.c21
  //@req-spec-memord.c22
  A load that order-failed is squashed, refetched at its own PC, re-renamed and
  re-executed. On that execution it obtains its data by exactly one of two routes,
  and this module is one of them: it forwards the store data out of
  `st_SSI_DATA_Q` / `st_US_DATA_Q` if the store is still in flight and now covers
  it, or it reads the DRAINED CACHE LINE if the store has since committed and
  written memory. Both converge on the same writeback, and the result is written
  into the FRESH physical destination the re-rename allocated — for a scalar load
  through `io.fwd_resp` and the LSU's ordinary `iresp`/`fresp`, for a vector load
  through `io.fwd_beat` into the LCB, which writes the re-renamed `pvdest` group
  and emits one group-done.

  Nothing here knows the load is a re-execution and it must not try to: the
  fresh destination arrives on the uop like any other. A "was replayed" bit
  reaching this module would be a second source of truth for something rename
  already settled.

  ---- 9. The unit-stride vector consumer ----

  This whole paragraph is under `enableVecBeatForward`. With it false `io.fwd_beat`
  is held invalid, no US/US candidate is ever selected as a forwarder, and the pair
  is left to VecOrderHold — which is why VecOrderHold's `forwardingEnabled` must
  carry the SAME value and why elaboration asserts it.

  ===> AND `enableVecBeatForward` IS FALSE AT EVERY INSTANTIATION TODAY, BECAUSE
       THIS PARAGRAPH CONTRADICTS THE PORTS SECTION. Read literally it needs
       per-instruction state and this module is forbidden to hold any ("It holds
       no state scoped to an instruction: its only registers are the one-cycle
       search-to-response pipeline registers of paragraph 1"). The contradiction
       is structural, not a wording slip: a unit-stride vector load presents ONE
       range search to the LCAM, on its FIRST beat only, so the cover decision is
       taken once — but the paragraph then requires every SUBSEQUENT beat of that
       load to be answered from the store data queue instead of the D$, and
       nothing in this interface tells a later beat which store, if any, its
       instruction decided to forward from. Carrying that decision means a
       per-load row keyed on `ldq_idx`, which is the state the ports section
       forbids and which plan section 5 rule 6 forbids more generally.
       There is a second, independent blocker one level up: the LCAM presentation
       and the D$ request for a beat are ONE arbiter grant in VecDcacheArbiter, so
       "search, then decline the access" cannot be expressed — suppressing the
       access needs a predicate evaluated BEFORE the grant, which neither this
       spec nor the arbiter's defines.
       Until both are resolved the pair is covered by VecOrderHold, which is
       correct and costs only the forward's latency win. DO NOT GENERATE THIS
       PARAGRAPH from the text above; generate the switch and the invalid
       `io.fwd_beat`, and leave the mechanism to whoever resolves the ownership
       question. The rest of the paragraph is retained verbatim below because it
       states the intended datapath, and it is the starting point for that work.

  When both sides are unit-stride the decision is made ONCE, at the load's
  range-overlap check, and is all-or-nothing: the store's active bytes must fully
  cover the load's active bytes, or paragraph 6 replays the WHOLE vector load (a
  vector order-fail is at whole-instruction granularity — there is no partial-group
  rewind). When it does cover, the load issues no D$ access at all: VecBeatExpander
  forms the same beats it would have requested from the D$ and this module supplies
  each beat's data from the corresponding `st_US_DATA_Q` member, byte-selected by
  `(beat_addr - store_range_base)` within that member. The beat is presented on
  `io.fwd_beat` in the D$ response beat's shape, so the LCB places it by `prn` plus
  byte offset with no idea it was forwarded.

  ---- 10. Gating, assertions, tracing ----

  The whole module is inside `usingRVV`. With vectors disabled it does not
  elaborate, the LSU delta's taps are absent, and the RTL is bit-identical to
  pre-Caracal BOOM v4.

  Assertions, each catching something that would otherwise appear as a wrong value
  rather than a failure: a candidate whose store is not older than the searching
  load; a data-queue read whose `resp.filled` is clear; an SSI candidate whose
  address-region and data-region bases disagree; a US candidate whose `member` is
  at or above `members`; `io.fwd_resp` and `io.fwd_beat` valid on one lane in one
  cycle; a forward asserted for one of paragraph 3's three ineligible pairs.

  Two further assertions guard the switch split, because a wrong switch is silent:
  `io.fwd_beat` valid while `enableVecBeatForward` is false, and `io.fwd_resp`
  valid for a load with `is_vec` set (a vector load's result never returns on the
  scalar response port). Both are structurally unreachable, which is the point —
  they cost nothing and they fail loudly if a later edit re-couples the classes.

  Tracing through the shared `VecTrace` helpers, gated on the `vecTrace` plusarg
  and `!reset`, off by default:
    - one line per taken scalar forward (`rob_idx`, `ldq_idx`, winning `stq_idx`,
      queue and entry index, ordinal, forwarded byte mask);
    - one line per taken beat forward (same, plus `member` and `byte_offset`);
    - one per declined forward naming WHICH of paragraph 6's four conditions
      declined it, and additionally which of the two switches declined it when the
      decline was a switch rather than a cover failure;
    - one per exported `known_overlap`;
    - one per SEARCH that found a tier-1 match but NO usable candidate, carrying
      the load's class and the winner's class. This is the "the snoop matched but
      nothing forwarded" case, which is otherwise indistinguishable from "the
      snoop never matched" and is the single most likely shape of a Phase-G
      regression: the load quietly falls back to the replay floor and only shows
      up as a performance loss or, if the floor is also broken, as stale data.
  The declined-reason field earns its keep — "the load replayed" is otherwise
  indistinguishable between a mask hole, a straddle and a snoop that never
  produced a candidate.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Latency: ONE cycle from search to forwarded result, matching baseline BOOM's
forward exactly. A constraint, not a target — the module sits on the load-to-use
path of every scalar load in a vector build, and a second cycle here would
lengthen that path for scalar code that touches no vector state.

Throughput: one decision and one result per lane per cycle, `lsuWidth` lanes, with
no arbitration between lanes and no shared resource except the two data-queue read
ports — one shared port per queue, so a two-lane machine needing both in one cycle
serves one and replays the other, which is the outcome the partial-cover rule
already produces.

Area: this module contains NO comparator array; the compares against the store
address queues are VecCrossLsuSnoop's and are counted there. Counted here are the
youngest-store select (the existing `ForwardingAgeLogic`), the mask expansion and
subset test (up to `vLen` mask bits expanded to byte granularity) and the
byte-select mux out of a `vLen`-wide store member. The mask expansion is the widest
term and is combinational in the search cycle, so it is the path to watch if this
file fails timing; the fix is to register the expanded active-byte vector on the
range entry when it is pushed — never to drop the qualification.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the load's uop supplies `mem_size`, `mem_signed`, `dst_rtype`,
`stq_idx`, `ldq_idx`, `rob_idx`; a vector load's uop supplies
`v_is_unit_stride`, `v_eew` and `pvdest`. Read those class fields off the uop —
do NOT re-decode `uop.inst`.

VecBundles — `VecRangeEntry` (base, total byte length, `eew`, `is_unit_stride`,
element-granular mask) and `VecElemAccess` are the payloads the candidates
describe; this module reads their fields and declares none of them.
VectorParams, VecTrace — widths and the guarded trace lines.

Instantiates nothing. Instantiated once by VecLsu as `fwd`. Counterparties, each
of which Phase R should check from the other side:

  - VecCrossLsuSnoop (`snoop`) — owns the search (spec-memord.a19) and supplies
    `io.snoop_cand`: one candidate per in-flight vector store, indexed by STQ
    placeholder index, already reduced to that store's youngest hitting entry.
    This is the seam most likely to have been specified differently over there.
  - VecElemQueue instances `st_SSI_DATA_Q` and `st_US_DATA_Q` — the shared indexed
    read port, one registered cycle of latency. No write port, no second read port.
  - VecOrderHold (`hold`) — consumes `io.known_overlap` as its "Known" input.
  - VecLoadCoalescingBuffer (`lcb`) and VecBeatExpander (`ld_beat`) — the
    unit-stride vector consumer path of logic paragraph 9.
  - The LSU delta (`src/main/scala/v4/lsu/lsu.scala`) — provides `io.ld_search`,
    `io.stq_addr_matches`, `io.stq_forward_matches`, and consumes `io.fwd_resp`
    and `io.replay` on its existing writeback and `s1_kill` paths.
  - VecDcacheArbiter (`arb`) — grants the LCAM lane a vector load's range search
    uses. Suppressing a held load's grant is its obligation, not this module's.
<|end_dependencies|>

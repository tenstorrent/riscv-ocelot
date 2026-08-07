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
  VecOrderHold — decides WHICH younger vector load is held behind WHICH older
  in-flight vector store, for the class combinations that do not forward.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecOrderHold.scala,
  package boom.v4.vec.generated.lsu. depends_on MicroOp, VecBundles,
  VectorParams, VecTrace. Instantiated once by VecLsu as `hold`. Elaborated only
  under `usingRVV`; in a vectors-off build the module is ABSENT, not tied off.

  It is the third of the three memory-ordering mechanisms and the only one that
  is neither a search nor a forward: `VecCrossLsuSnoop` searches ST->LD and
  raises `order_fail`, `VecStoreForward` returns DATA, and this module returns
  NOTHING — it makes a younger load WAIT. There is consequently no data path in
  this file and no address comparator: the overlap is decided elsewhere and
  arrives here as an event. That is also why it is small.

  ===> IT IS NOT A CORRECTNESS MECHANISM, and the distinction changes how it is
       reviewed. The hold is a PERFORMANCE mechanism whose only job is to avoid a
       squash; the ordering-violation replay path (`order_fail` ->
       MINI_EXCEPTION_MEM_ORDERING at the ROB head) remains the correctness floor
       for any load that slips through before the dependence is detected. A
       conservative hold, or an outright WRONG prediction, costs cycles and never
       data. Hold this file to a liveness and a bandwidth standard, not to a
       data-integrity one: a build with `enableOrderHold = false` must still pass
       the full cosim regression.

  ===> AND IT MAY NOT INTRODUCE A DEADLOCK, which is the one way it can really
       break the machine. It holds a YOUNGER load behind an OLDER store and never
       the reverse, so every edge of the wait graph points backwards in program
       order and the graph cannot cycle. The reverse edge deadlocks outright: a
       held store never finishes draining, so it never reaches commit-drain, so
       the in-order commit that frees its reserved element-queue region never
       happens, and dispatch wedges behind a reservation it can no longer make.
       The age test in the logic section is a HARD RULE, not an accuracy filter.

  Governing spec anchors: loadstore.rst `mem-order` ("Memory Ordering and
  Disambiguation", the Known / Predicted / Held-by / Released-by table),
  `order-fail-replay`, `dcache-arbiter`, `elem-progress` (the element cursor this
  module waits on).
*/

<|begin_module|>

  <|begin_parameters|>
  No sizing knob of its own, because there is no structure to size. Widths come
  from `HasBoomCoreParameters` on the enclosing core: `numLdqEntries` /
  `numStqEntries` (16/16 Medium, 24/24 Large, 32/32 Mega), `ldqAddrSz` /
  `stqAddrSz`, `robAddrSz`, `lsuWidth` (LCAM search lanes, 1 or 2) and `coreWidth`.
  LDQ and
  STQ indices are carried at BOOM's full `1 + ldqAddrSz` / `1 + stqAddrSz` width:
  the extra bit is the wrap-carry bit and the age comparisons below are wrong
  without it, so it must not be truncated at this boundary.

  One Scala parameter. `enableOrderHold` (Boolean, default true) — when false the
  module elaborates to a constant-zero hold mask and no state. It is the
  EXECUTABLE FORM of the header's correctness claim: with the hold off the machine
  must still be correct, differing only in how often a vector load order-fails, so
  "is this a performance mechanism?" is answerable by a run rather than by a
  review, and a suspected hold-related hang can be bisected without editing RTL.
  A Scala `Boolean` and not a plusarg, so a build that does not want the mechanism
  does not carry its flops.

  No threshold, no counter width and no timeout parameter — there is no timer
  anywhere in this module.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit pair: posedge `clock`, ACTIVE-HIGH
  SYNCHRONOUS `reset`. No second clock, no asynchronous reset. Reset clears the
  state to "nothing held", which is the safe pole — a spurious release costs a
  replay, a spurious hold can wedge.

  ---- The load-side context, and the store-side event ----

  Every input below is `Vec(lsuWidth, ...)`: one independent lane per LCAM search
  lane, no arbitration between lanes. The load side and the store side arrive as
  two separate ports and are JOINED PER LANE, which is what keeps this module's
  payload identical to what `VecStoreForward` already declares rather than asking
  a written sibling to widen its output.

  `ld_ctx` (Input, `Vec(lsuWidth, Valid(new VecHoldLdCtx))`) — the load-side
  search context, the same LCAM-stage tap the LSU delta already gives
  `VecStoreForward` as its `io.ld_search`, reduced to the four fields needed here:
  `ldq_idx` (`lcam_ldq_idx`, `1 + ldqAddrSz` bits), `rob_idx` (trace only, see the
  logic section), `is_vec` and `is_unit_stride`. The class bits must be taken from
  the `MicroOp` fields `is_vec` and `v_is_unit_stride` and never by re-decoding
  `uop.inst` — the mistake the MicroOp delta's access-class section exists to
  prevent. `VecStoreForward` and this module must see the same lane in the SAME
  cycle, or the join below pairs a store event with the wrong load.

  `known_overlap` (Input, `Vec(lsuWidth, Valid(new VecHoldStEvent))`) — an overlap
  the LCAM actually observed, taken verbatim from `VecStoreForward`'s
  `io.known_overlap`: `stq_idx` (the matching store's STQ PLACEHOLDER index) and
  `is_unit_stride`. The two modules run off ONE match by construction, which is
  the point of consuming their classification instead of repeating the comparison.

  `pred_overlap` (Input, same shape) — the same payload out of BOOM's existing
  memory-dependence machinery in the host LSU. Two ports rather than one because
  their trust levels differ and the trace must distinguish them; they are
  otherwise handled identically.

  // Neither event carries an "is a vector store" bit and none is needed: a store
  // entry that is not a live vector store with elements left to write reads
  // `st_drained` ASSERTED, so the admission term below rejects it. One definition
  // does two jobs, and there is no second place for the two to disagree.

  `st_drained` (Input, `Vec(numStqEntries, Bool)`) — the release, a LEVEL, one bit
  per STQ entry, driven by `VecLsu`, indexed by the REAL STQ index so every lookup
  here goes through `GetRealLSQIdx(stq_idx)`. Its definition is exact and getting it wrong
  is the deadlock: `st_drained(i)` is asserted whenever entry `i` is NOT a vector
  store with active elements still to write — invalid, squashed, scalar,
  never-allocated and fully-drained entries all read asserted. It deasserts ONLY
  while `i` holds a live vector store whose element cursor has not yet completed
  its active element set on the post-commit WRITE pass.

  `ldq_valid` (Input, `Vec(numLdqEntries, Bool)`) — the LSU's existing LDQ valid
  vector. `ldq_alloc` (Input, `Vec(coreWidth, Valid(UInt((1 + ldqAddrSz).W)))`) —
  LDQ placeholder allocation, the `dis_ldq_idx` qualified by the `dis_ld_val` the
  LSU already forms at `lsu.scala:381`. Both are needed: the level covers commit
  and branch kill, the allocate pulse covers the same-cycle kill-and-reallocate
  case in which the level never shows a low cycle.

  `hold_ldq` (Output, `UInt(numLdqEntries.W)`) — THE decision and the only output:
  one registered level bit per LDQ placeholder, packed, bit-indexed by the REAL
  LDQ index (`GetRealLSQIdx`) with no wrap-carry bit. `VecDcacheArbiter` indexes it
  with the requesting load-drain lane's `uop.ldq_idx` and suppresses that request's
  grant — D$ lane, LCAM port and TLB port together, since the arbiter grants all
  three as one. The width and packing match `VecDcacheArbiter`'s already-declared
  `io.hold_ldq` input and the LCB's `io.kill_ldq` exactly, and are the same shape
  as baseline `block_load_mask` (`lsu.scala:481`), so the arbiter can OR them.

  ===> THE SUPPRESSION IS NOT IN THIS FILE. This module decides who is held;
       `VecDcacheArbiter` owns what a hold DOES. Two modules editing one grant
       expression would make it impossible to say which dropped a request.

  ===> AND THERE IS NO OTHER OUTPUT: no `busy`, no `active`, no ready
       contribution, and above all NOTHING REACHING AN ISSUE UNIT. A held load has
       already issued and already has an address; it stalls at the memory-port
       grant. Routing this mask to an issue queue would violate the vector-LSU
       invariant (plan rule 6) and would also make one held load block every
       younger unrelated vector load behind it in the queue.
  <|end_ports|>

  <|begin_logic|>
  //@req-spec-memord.b11
  ---- What is held, and in which combinations ----

  A younger vector load overlapping an older in-flight vector store is held until
  that store completes, in EVERY combination that does not forward. Joining lane
  `w`'s `ld_ctx` with the same lane's `known_overlap` or `pred_overlap`, an event
  is admitted when all of: the load is vector, `ld_ctx.is_vec`; the pair does not
  forward, `!(ld_ctx.is_unit_stride && ev.is_unit_stride)`; the store is still
  draining, `!st_drained(ev.stq_idx)` (which is also what makes the store a vector
  store); the load's entry is live, `ldq_valid(ld_ctx.ldq_idx)`; and the store is
  OLDER than the load by the age rule below.

  The forwarding term is the exact complement of the vector-to-vector forwarding
  rule owned by `VecStoreForward`, so the three non-forwarding combinations — SSI
  store to SSI load, SSI store to US load, US store to SSI load — are covered and
  a US/US pair is never held. The two predicates must stay complements: a pair
  that neither forwards nor holds drops to the replay floor and pays a squash. A
  US/US pair whose forward only PARTIALLY covers the load is still not this
  module's case — that is a partial-forward replay, and `VecStoreForward` owns it.

  // A SCALAR load is never held; it forwards out of the store data queues at
  // either class. That asymmetry is the reason this module exists: a scalar load
  // costs at most ONE replay, whereas a multi-element vector load would re-fail
  // on every replay until the store commits.

  ---- The age rule: the hard direction constraint ----

  The event's `stq_idx` must be OLDER than the load, tested with the LSU's
  existing age machinery and never with a bare index compare: the store must lie
  in `EntryValidFromAge(stq_head, ldq_next_stq_idx(ld_ctx.ldq_idx), stq_idx)` — exactly
  the `age_matches(w)(i)` term at `lsu.scala:1358`, whose own comment reads
  "stq(i) is older than the searcher". `ldq_next_stq_idx` is the per-load
  store-age boundary BOOM already records at dispatch (`lsu.scala:383`), so the
  test adds no state.

  An event failing the age test is DROPPED and asserted on. It is not clamped,
  not inverted and never held the other way round. This is also why no
  cycle-detection logic is needed: with every edge pointing backwards in program
  order the wait graph is a forest rooted at the oldest store, and that store
  always makes progress because commit is in order and nothing this module drives
  can stop it.

  //@req-spec-memord.b17
  ---- The hold itself: the load's existing store-dependency block ----

  The hold is the younger load's per-load store-dependency block, pointed at the
  older vector store entry — two functional fields per LDQ entry and nothing else
  (the tracing paragraph adds one trace-only latch and no functional state):

    hold_valid    Reg(Vec(numLdqEntries, Bool))
    hold_stq_idx  Reg(Vec(numLdqEntries, UInt((1 + stqAddrSz).W)))

  6 bits x 16 entries = 96 flops on Medium. Bit `i` of the packed `hold_ldq` output
  is `hold_valid(i) && ldq_valid(i)`, registered. `hold_stq_idx` never leaves the
  module; only the one-bit verdict does.

  ===> NO NEW PREDICTOR, NO NEW CAM, NO NEW QUEUE — that is the whole state. It
       suffices because a vector load holds exactly ONE LDQ placeholder however
       many elements it cracks into (the element accesses live in
       `ld_SSI_ADDR_Q` / `ld_US_ADDR_Q`), so one bit per LDQ entry already says
       "this whole vector load waits". A per-element hold would be a structure the
       size of the element queues and would buy nothing, since a held load cannot
       make partial progress past the store anyway.

  One pointer, not a mask, and the consequence is stated rather than glossed. A
  second admitted event for an already-held entry overwrites the pointer only if
  the new store is YOUNGER than the pointed-to one (the same age test applied
  between the two stores), so the pointer names the youngest older overlapping
  store known so far. That is not the same as waiting for every store the load
  overlaps: an overlap discovered later may still be draining when the hold
  releases. The gap is accepted deliberately — the load is simply re-held by the
  next event naming that store, and anything escaping in between lands on the
  replay floor. A per-load store mask would close it at `numStqEntries` bits per
  LDQ entry, i.e. the CAM-shaped structure this module was specified not to build.

  //@req-spec-memord.b16
  ---- Where the prediction comes from ----

  Overlap prediction is BOOM's existing memory-dependence predictor, reached
  through `pred_overlap`. This module contains no predictor, no training, no table
  and no PC-indexed state, and adds none. In the vendored BOOM v4 baseline that
  machinery is the address-match-driven blocking at `lsu.scala:1413-1431` — an
  `ldst_addr_matches` hit that failed to forward raises `block_load_wakeup`, and a
  store starved 15 cycles (`store_blocked_counter`) does the same; the host LSU
  delta extends what feeds those matches to the cross-queue vector case and
  presents the outcome here.

  // Read the port, not the predictor. If BOOM later gains a trained store-set
  // predictor (an SSIT/LFST pair), it drives this same port with this same
  // payload and nothing in this file changes. That is the point of taking the
  // prediction as an input instead of growing one here.

  Accuracy is explicitly not this module's problem in either direction: a false
  positive costs the wait, a false negative costs a replay, neither can produce a
  wrong architectural value. So there is no accuracy threshold to meet and no
  reason for confidence bits.

  `known_overlap` carries the other row of the spec's table — an overlap is KNOWN
  when the LCAM matches an already-generated store element address, or the store's
  US range, against the load's address. That determination belongs to the search
  side and is tagged there, not here. This module treats a known event exactly
  like a predicted one, differing only in the trace tag and in the assertion that
  a known event's age test MUST pass: a known match with a bad age is a search
  bug, not a mispredict.

  //@req-spec-memord.b19
  ---- Release: the store's element cursor. Not commit, and not a timer ----

  A hold releases when the store entry it points at completes its ACTIVE ELEMENT
  SET, i.e. when `st_drained(hold_stq_idx(i))` asserts: `hold_valid(i)` clears and
  the load is grantable the next cycle. Nothing else releases a hold except the
  entry-lifetime clears below. Three things the release is NOT:

  1. NOT the store COMMITTING. Commit is what lets the store START its write pass;
     no element has reached the cache yet. Releasing there would let the load read
     the stale line, leaving the hold to add latency and nothing else.
  2. NOT the store's TRANSLATE pass finishing. That pass resolves and translates
     element addresses pre-commit and writes no data. This is the subtle one,
     because the translate pass is also "an element cursor completing its active
     element set" — for the OTHER of the store's two cursors. `st_drained` is
     defined on the post-commit WRITE pass for exactly this reason.
  3. NOT A TIMER. No countdown, no retry interval, no maximum hold length.

  ===> `st_drained` IS A LEVEL AND MUST NOT BE NARROWED TO A PULSE. A predicted
       event can name a store that has ALREADY drained — the prediction is not
       synchronised with the store's progress — and a pulse-only release would
       then never arrive, hanging that load forever. With a level, such an event is
       either rejected at admission by the `!st_drained(stq_idx)` term or released
       on its first evaluation. This is the single most likely way to turn this
       performance feature into a hang.

  //@req-spec-memord.b12
  ---- After the release ----

  The released load then reads the UPDATED cache line: it re-requests through
  `VecDcacheArbiter` and its element accesses go to the D$ normally, having waited
  precisely long enough for the older store's elements to be there. No data is
  handed over and no bypass or buffer is needed — the updated line is in the cache
  because the store's write pass put it there, which is what release condition 2
  protects. A released load gets no priority boost; it re-enters the same
  priority-round-robin. Where both sides are SSI the two element streams also
  share the LCAM and D$ port through that arbiter, so the pair executes
  effectively serially, which is the intended outcome rather than a shortfall.

  //@req-spec-memord.b13
  ---- The correctness floor stays underneath ----

  Nothing here is load-bearing for correctness. A load that slips through before
  the dependence is detected — every `pred_overlap` false negative, plus the
  unavoidable window where a load is granted in the same cycle its match is being
  computed — is caught by the ordering-violation replay path: the store's own
  ST->LD search sets that load's `ldq_order_fail`, the LSU broadcasts the oldest
  failing load as an `lxcpt` with `MINI_EXCEPTION_MEM_ORDERING`, and the flush
  fires at the ROB head. This module therefore never retracts a grant it lost the
  race for, never cancels an in-flight element access, and NEVER gates
  `order_fail`. Two consequences to review against: `enableOrderHold = false` must
  pass the full cosim regression, and a hold-related failure signature is a hang
  or lost bandwidth, never a data mismatch — a `MISMATCH` line in a cosim log is
  evidence against `VecCrossLsuSnoop` or `VecStoreForward`, not against this file.

  ---- Entry lifetime, squash and flush ----

  `hold_valid(i)` is additionally cleared whenever `!ldq_valid(i)` (commit, branch
  kill, exception flush) and whenever `ldq_alloc` allocates entry `i`. The
  allocate clear is what makes index reuse safe: a fresh load inheriting a dead
  predecessor's pointer would wait on an unrelated store, and on an
  already-drained one it would wait forever. No `brupdate` or flush port is needed
  — LDQ validity already encodes every kill BOOM performs, and this state is pure
  shadow state over the LDQ.

  Per-cycle update, in one place: clear from `!ldq_valid`, `ldq_alloc` and
  `st_drained`, then set from the admitted events. CLEAR BEATS SET on the same
  entry in the same cycle, so an event for an entry being reallocated, or for a
  store draining this cycle, cannot install a stale hold.

  ---- Tracing and assertions ----

  Emit one guarded `VecTrace` line per key event — hold-admitted (tagged `known`
  or `pred`), hold-released, hold-dropped-on-age, hold-cleared-on-realloc — each
  carrying the load's `rob_idx`, its `ldq_idx`, the `stq_idx` held on, and on
  release the elapsed cycle count, gated on the `vecTrace` plusarg and off by
  default. `ld_ctx` carries `rob_idx` for this and for nothing else: `VecTrace`
  mandates `rob=<rob_idx>` on every line, and this module holds no `MicroOp` to
  take it from. A release line therefore needs the `rob_idx` latched alongside
  `hold_stq_idx`, `robAddrSz` bits per LDQ entry. That latch is TRACE-ONLY state:
  no functional logic may read it, and tracing is a run-time plusarg, so it exists
  in every build with `enableOrderHold` true. It is called out here rather than
  hidden because `VecTrace`'s own rule — helpers declare no state — is about the
  helper, and a caller latching an identifier the helper mandates is the honest
  cost of that mandate. With no unit tests in this project these four lines are the only way to
  tell a working hold from a hang, and the elapsed count is what separates "held
  as designed behind a long store drain" from "never released". The elapsed
  counter is TRACE-ONLY and must not be read by functional logic, or it becomes
  the timer this design does not have.

  Runtime assertions: an admitted event failing the age test; a `known_overlap`
  event failing it (the stronger form — a known match cannot legitimately be
  mis-aged); a hold on an invalid LDQ entry; a hold whose `hold_stq_idx` has read
  `st_drained` for many consecutive cycles without the hold clearing (a liveness
  tripwire, `assert` only, no functional effect); and a hold admitted for a US/US
  pair, which would mean the forwarding predicate and this one have drifted out of
  complement.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
The hold mask is a REGISTERED output. `VecDcacheArbiter`'s grant expression is
already one of the deeper combinational paths in the vector LSU (priority floor,
round-robin, three resource availabilities), so the suppression term must be a
flop read — never an age comparison and address match resolved in the grant
cycle. That fixes the timing of the mechanism: an event admitted in cycle N
suppresses grants from N+1, and a load granted in cycle N is NOT retracted, the
replay floor covering that one-cycle window by design.

Release latency is one cycle from `st_drained` rising to `hold_ldq` falling, and
must not be longer: the store's drain has just finished, so every added cycle is
dead memory-level parallelism on a load that is now free to go.

Throughput: up to `lsuWidth` events admitted per cycle (one per search lane), and
any number of releases per cycle — release is a per-entry level test, not an
arbitrated walk, because several holds may point at one store and all must clear
together.

Area: `numLdqEntries * (2 + stqAddrSz)` functional flops — 96 at Medium — plus the
trace-only `rob_idx` latch and `lsuWidth` age comparators reusing the LSU's
existing `EntryValidFromAge`. No CAM, no per-element state, no predictor storage.
That figure is the budget; materially more means the module grew a structure it was
specified not to have.

Expected hold length is the older store's remaining drain, which for the
post-commit write pass of a wide SSI store can be hundreds of cycles. That is
accepted: the alternative for the same pair is a replay STORM — one full
squash-and-refetch per attempt until the store commits anyway. Two second-order
costs to watch in the LS regression rather than design around: a held load does
not translate, so its `rob_unsafe` bit stays set and the PNR does not sweep past
it, delaying past-PNR CII issue; and a held load's element-queue reservation stays
occupied, reducing the effective in-flight vector load count. Both show up as
throughput, neither as a stall that cannot break.
<|end_perf|>

<|begin_dependencies|>
MicroOp — indirectly, and worth stating precisely: this module reads no `MicroOp`.
The `is_vec` / `v_is_unit_stride` qualifiers and the `rob_idx` its trace lines
quote are extracted by the producer and arrive on the event. The edge exists so a
rename of those fields is visible from here.

VecBundles, VectorParams — declared dependencies used only for the `usingRVV`
elaboration gate. No vector width appears in this file at all: `vLen`, `eLen`,
`vecPregSz` and the element-index widths are absent because the module never names
an element, a register or a byte. If a later edit needs one, that is a signal it
has taken on work belonging to the search or the forward.

VecTrace — the four guarded lines above.

Instantiates nothing. Instantiated once by VecLsu. Counterparties, each of which
Phase R should check from the other side:
  - `VecStoreForward` drives `known_overlap` with its already-declared
    `io.known_overlap` (`stq_idx` + `is_unit_stride`, per lane, unwidened) and owns
    the complementary forwarding predicate; the two predicates must remain exact
    complements.
  - the host LSU delta drives `pred_overlap` from BOOM's existing
    memory-dependence machinery, `ld_ctx` from the same LCAM-stage tap it gives
    `VecStoreForward` as `io.ld_search`, and `ldq_valid` / `ldq_alloc`.
  - `VecLsu` drives `st_drained` from the store entries' element cursors on the
    post-commit write pass, and routes `hold_ldq` onward.
  - `VecDcacheArbiter` consumes `hold_ldq` and owns the actual suppression of the
    D$, LCAM and TLB grant.
  - `VecCrossLsuSnoop` is NOT a counterparty in either direction, and the absence
    is deliberate: `order_fail` must never be gated, delayed or suppressed by a
    hold.
<|end_dependencies|>

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
  VecCiiFlush — the CII's entire misspeculation-recovery mechanism: it decides
  WHEN in-flight coprocessor work is doomed, broadcasts that as one `kill_all`
  bit, and holds the assertions that make the whole no-branch-kill argument
  checkable rather than merely asserted in prose.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/cii/VecCiiFlush.scala,
  package boom.v4.vec.generated.cii, group vec_cii.
  depends_on VecBundles, VecTrace. Instantiates nothing.
  Instantiated once, as `flush`, inside VecCiiHost alongside `tags`, `iss`,
  `opnd`, `wb`, `done` and the `coproc` BlackBox.

  ===> THIS NODE OWNS THE KILL CONTRACT BUT NOT THE KILL STATE. The per-tag
       `killed` bit vector is `tag_killed` inside VecCiiTagTable, which ORs
       `tag_valid` into it on `kill_all` and clears a bit only at allocation.
       VecCiiOperandServer, VecCiiWriteback and VecCiiComplete each read `killed`
       out of their own side-table lookup, NOT from this module — a second,
       direct view of the flush in those nodes would be a second kill path that
       could disagree by a cycle. So this file specifies the contract and the
       trigger; the four siblings execute it. Do not add a per-channel kill
       output here, and do not duplicate `tag_killed` here.

  WHY THE PATH EXISTS AT ALL, given that IQ_V_ALU grants only past-PNR entries:
  past the PNR does NOT mean "will commit". `rob.scala:436-442` says so in its
  own assertion, which is explicitly waived for `MINI_EXCEPTION_MEM_ORDERING`
  because "the failing load will have been marked safe already" — `clr_unsafe`
  fires on a load's FIRST ADDRESS TRANSLATION (`lsu.scala:1443`), while
  `order_fail` is discovered much later, when a store's LCAM search hits that
  load. Load goes safe, PNR sweeps past it, CII ops issue, THEN the load
  order-fails, and the ROB-head flush squashes everything younger including
  in-flight CII work. Caracal makes this MORE frequent than baseline BOOM, not
  less: `mem-order` deliberately extends `order_fail` to fire on cross-queue
  vector/scalar matches.

  ===> THE SRC-DATA BEAT IS MANDATORY. A killed tag's Src-Request must STILL be
       answered with a beat. "Ignore the request and hand a credit back" is not
       available on that channel, because credit ownership is asymmetric — the
       RECEIVER owns each channel's FIFO and returns its credits, so on Src-Data
       the HOST IS THE SENDER and holds no credit to return. The VPU is in-order
       and has NO kill line: it would wait forever for a beat that never comes,
       never emit `last`, never free its tag, and every SURVIVING instruction
       behind it would be stuck too. A squash would become a permanent hang.

  ===> DO NOT REUSE THIS DRAIN-AND-DISCARD PATTERN IN THE VECTOR LSU. See the
       contrast paragraph in the logic section — it is the most important
       paragraph in this file. `VecSquashUnit` uses pointer rollback, and must.

  Governing spec anchors: cii.rst `cii-flush` (past-PNR is not a commit
  guarantee) and `cii-kill-contract` (trigger, scope, behaviour, tag lifetime,
  idempotence), cii.rst `cii-issue` (the side-table membership list, in which
  `killed` is the field that "routes the drain-on-flush"), execution.rst
  `vector-execution` (the VPU needs no branch-kill path), plan v2 §5.

<|begin_module|>

  <|begin_parameters|>
  The module is elaborated only when `usingRVV` is set — a Scala `Boolean` from
  `BoomCoreParams`, never a hardware `Bool`, and never rocket's `usingVector` —
  and, like its siblings, inside VecCiiHost's additional `enableVectorArith`
  gate. In a build with either off it is ABSENT, not tied off, so that a
  vectors-off build is bit-identical to pre-Caracal BOOM v4.

  `nTags` — Int, `1 << ciiTagBits` from VectorParams, i.e. 16 at the default
  `ciiTagBits = 4`. Not a free choice: it must equal `CII_N_TAGS` in
  `tt_cii_caracal_pkg.svh`. It is needed here only to size the two
  occupancy-vector inputs, and an elaboration `require` must check that it
  matches VecCiiTagTable's `nTags`, because a width mismatch between the two
  would silently misalign the watchdog's view of which tags are draining.

  `drainWatchdog` — Int, default 8192 cycles, 0 disables. Bound on how long a
  tag may remain killed-and-live before an assertion fires. It exists because the
  failure mode this contract guards against is a HANG, not a wrong value, and a
  hang has no other signature: the machine simply looks idle until
  `boom_timeout`. The counter is declared inside the assertion's elaboration
  guard, drives nothing functional, and follows BOOM's own `boom_timeout`
  precedent. With `drainWatchdog = 0` no counter is emitted at all.

  There is deliberately NO parameter and no port carrying a `rob_idx` width, a
  ROB-index comparator depth, or any age-ordering knob. See the scope paragraph
  in the logic section — the absence is the design, not an omission.

  Clock/reset: Chisel default — posedge `clock`, active-high SYNCHRONOUS
  `reset`, both implicit via `BoomModule`. This module holds no functional
  register at all (see the state paragraph), so reset affects only the watchdog
  counter, which is `RegInit(0.U)`.
  <|end_parameters|>

  <|begin_ports|>
  ---- The trigger, from the core seam (`vec_pipeline_io`, via VecCiiHost) ----

  `rob_flush` — Input Bool. `vec_pipeline_io.rob_flush`, which is
  `rob.io.flush.valid` itself, unregistered.
  `rob_flush_kill` — Input Bool. `vec_pipeline_io.rob_flush_kill`, which is
  `RegNext(rob.io.flush.valid)` — the same signal the scalar execution units take
  as `io_kill` and the issue units as `flush_pipeline`. Both ports carry terms of
  the SAME event; the kill-window paragraph says why both are consumed.
  `brupdate_mispredict` — Input Bool, `brupdate.b2.mispredict`.
  **ASSERTION-ONLY.** Taken as one bit rather than the whole `BrUpdateInfo`
  bundle on purpose: this node's `depends_on` is VecBundles and VecTrace, it has
  no business decoding branch state, and a one-bit assertion input cannot quietly
  grow into a kill term. Note that VecCiiTagTable's reject list forbids a
  `brupdate` input there for the same reason — this is the ONE place in the CII
  host where branch state is even visible, and only to be contradicted.

  ---- The broadcast output ----

  `kill_all` — Output Bool. The whole functional interface of this module. Wired
  by VecCiiHost to VecCiiTagTable's `kill_all` input (which does
  `tag_killed := tag_killed | tag_valid`) and to VecCiiWriteback's `io.kill_all`
  (which needs the SAME-CYCLE pulse, because a writeback beat can arrive in the
  very cycle the flush fires, one cycle before `killed` is readable from the
  entry). It is a bare Bool: no tag, no mask, no `rob_idx`, no age comparator.

  ---- Assertion and trace inputs (nothing functional may be derived from these) ----

  `tag_valid` — Input `UInt(nTags.W)`, from VecCiiTagTable's `debug.valid`.
  `tag_killed` — Input `UInt(nTags.W)`, from VecCiiTagTable's `debug.killed`.
  Those two outputs are declared debug-only by their owner and this module
  honours that: they feed the watchdog, the trace lines and the protocol
  assertions, and no expression that reaches `kill_all`.
  `alloc` — Input `Valid(UInt(ciiTagBits.W))`, from VecCiiIssue: the tag
  allocated this cycle.
  `alloc_br_mask` — Input `UInt(maxBrCount.W)`, the granted uop's `br_mask`.
  `alloc_flush_on_commit` — Input Bool, the granted uop's `flush_on_commit`. NOT
  a side-table field and it must not become one: it is needed for exactly one
  cycle, at allocation.
  `free` — Input `Valid(UInt(ciiTagBits.W))`, from VecCiiComplete: the tag freed
  this cycle on its `last` beat, killed or live alike.

  What must NOT appear on this interface, as a reviewer's reject list: any port
  toward the coprocessor or the `coproc` BlackBox (the SV stack has no kill line
  and none is being added); a per-channel `src_drain`/`wb_suppress` output (the
  siblings read `killed` from their own lookup — see the ownership callout);
  a local copy of `tag_killed`; any `ready`, `busy` or stall output — this module
  never gates issue and never back-pressures a channel; a `rob_idx` or any
  ROB-index comparison input; any field of `rob.io.flush.bits` (`badvaddr`,
  `cause`, `ftq_idx`, `pc_lob`, `edge_inst`, `is_rvc`, `flush_typ`) — none is
  read, see the scope paragraph; a VRF port; a `MicroOp`.
  <|end_ports|>

  <|begin_logic|>
  ---- What the recovery mechanism is, in full ----

  //@req-spec-cii.d5
  The state this design adds for CII misspeculation recovery is `nTags` bits —
  VecCiiTagTable's `tag_killed` — plus the one-bit trigger computed here. That is
  the whole of it. Caracal does NOT implement the speculative RENAME model from
  `interface_details.adoc`: there is no shadow rename table, no `cv0`–`cv31` copy
  registers, no architectural-state snapshot on the coprocessor side, and no
  re-execution of accepted work. The past-PNR issue gate in `IQ_V_ALU` is what
  buys that simplification, because it makes every op the coprocessor accepts
  individually non-speculative with respect to BRANCHES; the only recovery case
  left is the rarer ROB-head flush, and a rare case whose work merely has to be
  discarded needs a kill bit, not a rename shadow.

  ---- The trigger ----

  //@req-spec-cii.e1
  //@req-spec-cii.e5
  //@req-spec-cii.e6
  Kill is triggered by a ROB-head flush and by nothing else:

    kill_all = rob_flush || rob_flush_kill

  Both terms are the SAME event — `rob.io.flush.valid` and its `RegNext` — so
  this remains "`rob.io.flush.valid` only" as the contract requires; it is not a
  second trigger source, and no other signal in the machine may be added to this
  expression. `brupdate_mispredict` appears in NO functional expression in this
  module. A generated implementation in which it appears anywhere outside an
  `assert` has failed review regardless of simulation results.

  WHY THE WINDOW IS TWO CYCLES WIDE. The CII's issue grant and the core's
  flush plumbing are one cycle apart: IQ_V_ALU takes
  `flush_pipeline = RegNext(rob.io.flush.valid)`, so a grant can still fire in
  the cycle `rob.io.flush.valid` is high, and VecCiiIssue will allocate a tag
  for it — a tag that is wrong-path and whose `killed` bit allocation has just
  CLEARED. The second cycle of the window catches exactly that tag, because it
  is live by then. Widening the window is free precisely because the bit is
  idempotent and sticky (below), so this costs one OR gate and closes a
  one-cycle hole. VecCiiTagTable currently asserts that `kill_all` never
  coincides with `alloc.valid`; with this window that coincidence is BENIGN
  and that assertion needs relaxing to "a tag allocated during a kill window
  carries `killed` by the following cycle". Flagged, not resolved here.

  //@req-spec-cii.e2
  No flush cause is decoded and `flush_typ` is not read. That is what makes the
  path cover every source uniformly: `flush_val = exception_thrown ||
  flush_commit` in `rob.scala`, so an ordinary exception, a
  `MINI_EXCEPTION_MEM_ORDERING` order-fail, a `MINI_EXCEPTION_CSR_REPLAY` and an
  ERET all present identically here. Distinguishing them would add logic whose
  only possible effect is to miss one of them.

  ---- The scope: every live tag, with no age comparator ----

  //@req-spec-cii.e8
  //@req-spec-cii.e9
  //@req-spec-cii.e10
  `kill_all` carries no tag and no age, and its meaning is "set `killed` on every
  LIVE side-table entry" — which VecCiiTagTable implements as the single
  `nTags`-bit OR `tag_killed := tag_killed | tag_valid`. There is no age
  comparison anywhere on this path: no per-tag `rob_idx` compare, no CAM, no
  youngest-survivor select, and no ROB index on the wire to compare with.

  Kill-all is CORRECT, not merely cheap, and the argument has two halves. First,
  the flush always fires at the ROB HEAD (`exception_thrown` and `flush_commit`
  both do), so nothing in flight can be OLDER than it: commit requires `rob_bsy`
  cleared, and for a CII op `rob_bsy` is cleared only by that op's `last` beat,
  so a tag that is still live has not committed and is at-or-younger-than the
  head. Second, `rob.io.flush.bits` carries no `rob_idx` to compare against in
  the first place (`rob.scala:613-622` drives only `badvaddr`, `cause`,
  `ftq_idx`, `pc_lob`, `edge_inst`, `is_rvc` and `flush_typ`), so a comparator
  here would first require a new ROB output. Cheaper and correct is a rare
  combination; take it.

  //@req-spec-cii.e26
  `killed` is IDEMPOTENT. A second flush arriving while a tag is still draining
  raises `kill_all` again and ORs `tag_valid` in again, which changes nothing for
  an entry that already carries the bit. There is no per-tag flush counter, no
  re-initialization on the second flush and no restart of the drain: the drain
  continues uninterrupted and the tag is freed on its `last` beat however many
  flushes intervened. This is also what makes the two-cycle kill window free, and
  what makes a `kill_all` that stays high for several consecutive cycles (a
  flush storm) harmless rather than something to filter or edge-detect. Do NOT
  edge-detect `kill_all`.

  //@req-spec-cii.e27
  The bit is CLEARED ONLY AT ALLOCATION, and never while the tag is live: not on
  the `free` of its `last` beat, not at the end of a drain, not on a later flush,
  and not by this module — which has no write path to it at all. Because `free`
  does not clear it, a freed tag sits idle carrying a stale set bit; that is
  harmless, since every consumer qualifies `killed` with the entry's validity and
  the allocation write is what cleans it. The corollary this module must respect:
  `kill_all` is a broadcast set, so it may never be gated on "no allocation this
  cycle" as a way of protecting a fresh tag. Ordering the two writes so that a
  wrong-path tag ends the cycle with a clear bit is the one way to get a tag that
  writes the VRF and clears a ROB entry that no longer exists.

  ---- The behaviour: drain and discard, never ignore ----

  //@req-spec-cii.e12
  A killed instruction is ALLOWED TO FINISH, on junk data. The VPU is in-order
  and the SV stack has no kill line, so the only way a killed tag leaves the
  coprocessor is by running to its own `last` beat, and the host's job is to keep
  feeding it. The contract per channel, which this node owns and the siblings
  execute:

    Src-Request  (host owns the FIFO)  pop, return `req_credit`, AND RETURN A
                                      DON'T-CARE SRC-DATA BEAT. No VRF read —
                                      that is the real saving, the CII read ports
                                      R5-R8 stay free for survivors.
    Writeback    (host owns the FIFO)  pop, return `wb_credit`, suppress all four
                                      effects: the VRF/INT/FP write, `clr_rob`,
                                      the `VecGroupDone`, and `fflags`/`vxsat`
                                      accrual.
    Issue        (coproc owns it)      nothing to do; a killed tag was already
                                      issued.
    Src-Data     (coproc owns it)      the host is the SENDER — see below.

  The manufactured Src-Data beat is what the danger admonition in
  `cii-kill-contract` is about, and it is the one place where the obvious
  optimisation is a permanent hang rather than a wasted cycle. Credit ownership
  is asymmetric: the RECEIVER owns each channel's FIFO and returns its credits,
  so on Src-Data the host holds no credit and "return a credit instead of a beat"
  is not an available move. Omitting the beat leaves the in-order VPU waiting
  forever, so it never emits `last`, never frees its tag, and every SURVIVING
  instruction queued behind it is stuck too. The suppression must therefore be at
  the EFFECT, never at the beat.

  Neither the drain nor the suppression may be registered relative to the beat
  it applies to. VecCiiOperandServer's drained beat is due in the cycle the
  request is serviced, and VecCiiWriteback takes `kill_all` combinationally
  precisely to cover the cycle in which `killed` is not yet readable from the
  entry. A pipelined kill decision reintroduces exactly the one-cycle hole the
  two-cycle window and the direct `kill_all` term exist to close.

  Tag lifetime is IDENTICAL for a killed tag: it is freed on its dropped `last`
  beat, exactly as a live one is. Only the effects differ. Freeing it earlier —
  the tempting "reclaim it now, it is dead anyway" — would let it be reallocated
  while its own beats were still arriving, and those beats would then be
  interpreted against a different instruction's side-table entry.

  ---- THE CONTRAST THAT MATTERS MOST: unsafe in the vector LSU ----

  Drain-and-discard works here for one specific reason, and that reason does not
  hold in the vector memory path. A CII tag is OPAQUE — it names a transaction and
  nothing else — and it is NOT REUSED until it is reclaimed on its own `last`
  beat, so a late beat for a killed tag can only ever land on the entry that is
  still, correctly, its own. A squashed vector LOAD has no such protection: its
  destination PRNs are returned to the vector free list ON THE FLUSH and are
  promptly reallocated, so a late response draining into a stale `pvdest`
  corrupts whatever register now owns that PRN — silently, and arbitrarily far in
  time from the flush that caused it. `VecSquashUnit` therefore does REAL
  squashing: pointer rollback on all six `VecElemQueue` instances plus LCB entry
  invalidation by `ldq_idx`. If a future change ever makes CII tags recyclable
  before their `last` beat, this contract collapses into the LSU's and must be
  rewritten, not patched.

  ---- No branch-kill path, and the assertions that keep it that way ----

  //@req-spec-cii.e4
  The VPU needs no branch-kill path and this module provides none: it drives
  nothing toward the coprocessor, and a branch mispredict changes no state
  anywhere in the CII host. Branch recovery costs the CII exactly zero cycles and
  zero wires. That is a claim about the MACHINE rather than about this module, so
  it is discharged by the assertions below rather than by construction — which is
  the whole reason this node takes assertion-only inputs at all.

  //@req-spec-cii.e3
  //@req-spec-cii.e7
  Assert, on every allocation, that `alloc_br_mask === 0.U`. This is the exact and
  cheap form of "no in-flight tag is ever younger than an unresolved branch":
  `is_br`/`is_jalr` set `starts_unsafe` (`micro-op.scala:164`), so the PNR can
  never sweep past an unresolved branch, so a past-PNR grant has no unresolved
  older branch and therefore an empty `br_mask`. With that assertion holding,
  "kill is never triggered by `brupdate.b2.mispredict`" becomes a theorem rather
  than a hope. The second assertion states it directly, as a check on the
  BROADCAST rather than on the expression: assert that `kill_all` is never high
  while `brupdate_mispredict` is high and both flush terms are low, with
  `brupdate_mispredict` named in the failure message. Written this way it is not
  a tautology a synthesis pass folds away, and it survives a future edit that
  adds a mispredict term to the kill expression.

  //@req-spec-cii.e11
  Assert, on every allocation, that `!alloc_flush_on_commit`. A CII tag that set
  `flush_on_commit` would be the head instruction of its own flush — a SURVIVOR
  that kill-all would wrongly kill, because it commits and the redirect happens
  after it. The assertion is not vacuous: `vsetvl` and the explicit vector-CSR
  writes DO set `flush_on_commit`, and it holds only because those are decoded as
  scalar uops issued to the integer ALU and never reach `IQ_V_ALU` or the CII. If
  a future decode change routes one of them over the CII, this assertion is what
  fires, and the fix is a survivor exemption in the kill scope — not deleting the
  assertion.

  Also assert the protocol invariants this module is positioned to see cheaply,
  since it holds both occupancy vectors: `free` for a tag whose `tag_valid` bit is
  clear is a double free; `alloc` for a tag whose `tag_valid` bit is SET is the
  reallocation-during-drain the tag-lifetime clause forbids; and a set bit in
  `tag_killed & ~tag_valid` outside the allocation cycle means somebody cleared
  validity without clearing kill, i.e. the "invalidate on flush" bug the tag
  table's callout warns against. And, when `drainWatchdog > 0`, assert that no
  tag stays in `tag_killed & tag_valid` for more than that many cycles — the
  swallowed-beat hang has no other signature, and this assertion is the only
  thing in the design that would name it.

  ---- State summary and tracing ----

  This module holds NO functional sequential state. `kill_all` is a two-input OR
  of two ports; everything else here is an assertion or a trace statement. The
  only register it may emit is the watchdog counter, inside the
  `drainWatchdog > 0` elaboration guard. There is no FSM, no per-tag storage and
  no shadow of `tag_killed` — that vector has exactly one owner.

  Guarded trace lines via the shared `VecTrace` package, gated on the `vecTrace`
  plusarg and `!reset`, off by default: one line on the rising edge of the kill
  window carrying `tag_valid`, its population count and which flush term fired;
  one line when `tag_killed & tag_valid` goes empty (the drain is complete and
  the machine is clean again); and one line if the watchdog trips.

  ===> TRACE-KEY DEVIATION, DELIBERATE. VecTrace's `trace` helper mandates a
  `rob_idx` on every line, and this module has none: it holds no `MicroOp` and
  no side-table entry, and a kill-all is a SET event with no single owning
  instruction. These lines are keyed on the `tag_valid` mask instead. The
  rob_idx-keyed line for each killed instruction is emitted by VecCiiComplete
  when it drops that tag's `last` beat, and by VecCiiOperandServer on each
  drained lane, both of which have the side-table entry and so the rob_idx.
  Do NOT add nTags x robAddrSz debug wires here to satisfy the convention
  literally — the correlatable line already exists downstream.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Recovery latency: ZERO cycles of dedicated recovery. There is no rollback
sequence, no replay walk and no drain state machine; the flush redirect is never
held waiting on the CII. The kill broadcast is one OR gate here and one
`nTags`-bit OR in the tag table, deliberately not an age comparison, so it adds
no comparator to the flush path and requires no new ROB output.

`kill_all` must be combinational from `rob_flush`. It is consumed in the same
cycle by VecCiiWriteback's write-suppression term, whose entire purpose is to
cover the cycle before `killed` is readable from the side table; registering it
here would reopen that hole. It is a fanout-2 single bit, so this costs nothing
in timing.

What a flush does cost is throughput, indirectly: a surviving instruction cannot
issue until an issue credit returns, and a killed tag's credit returns only when
the VPU reaches its `last` beat. Worst case is one full-length in-flight vector
operation's remaining latency. That is the accepted price of the SV stack having
no kill line; the alternative is a protocol change to a frozen, verified
interface.

Note that `CII_N_TAGS == CII_N_ISS_CREDITS` (16) in `tt_cii_caracal_pkg.svh`, so
the tag space cannot be exhausted independently of issue credits and this module
can never be what blocks an issue. It exports no `busy` and gates nothing — per
plan §5, a `busy`-style signal reaching an issue unit is a failed review.

Area: one OR gate, plus the watchdog counter when enabled. With
`drainWatchdog = 0` and tracing off, this module synthesizes to a wire.
<|end_perf|>

<|begin_dependencies|>
VecCiiTagTable — the state this module's output acts on. `kill_all` drives its
`kill_all` input; its `debug.valid`/`debug.killed` outputs come back as
`tag_valid`/`tag_killed` for assertions and trace only. The `killed` bit is read
by VecCiiOperandServer, VecCiiWriteback and VecCiiComplete out of their own
side-table lookups, never from here.

VecCiiWriteback — the second consumer of `kill_all`, which it needs in the flush
cycle itself for write-suppression.

VecCiiIssue — drives `alloc`, `alloc_br_mask` and `alloc_flush_on_commit`. Those
last two are new assertion-only outputs on that node; it is not yet written, so
this is a seam it must pick up.

VecCiiComplete — drives `free`.

VecBundles — for the CII tag width and the channel payload types. Notably NOT for
a bundle of this module's own: every port here is a plain `Bool`, `UInt` or
`Valid(tag)`, because a wider bundle would let instruction-dependent fields leak
into a module that must stay blind to them.

VecTrace — the guarded-printf convention, with the keying deviation noted above.

MicroOp is intentionally NOT a dependency, matching this node's `depends_on` in
hierarchy.yaml: the two facts it needs about the granted uop (`br_mask`,
`flush_on_commit`) arrive as bare assertion-only bits.

Instantiates nothing.
<|end_dependencies|>

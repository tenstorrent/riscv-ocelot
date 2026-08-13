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
  VecDcacheArbiter — the priority round-robin that shares the D$ request lane(s),
  the DTLB port and the LCAM port between the scalar LSU and the two vector
  drains.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecDcacheArbiter.scala,
  package boom.v4.vec.generated.lsu. group vec_lsu.
  depends_on VecBundles, VectorParams, VecTrace. Instantiated once, as `arb`
  inside VecLsu. Instantiates nothing.

  It sits OUTSIDE the cache and presents the SAME `dmem.req` interface: the
  memory subsystem (D$, MSHRs, DTLB, TileLink) is unchanged from BOOM v4, which
  is why no node in the map targets it.

  ===> THIS IS A REWRITE, NOT A PORT, AND P6 (no scalar regression) DEPENDS ON
       IT. `addvector` appended the vector requests LAST in lsu.scala's
       `lsu_sched` chain at the LOWEST static priority, with `uses_tlb = false`
       and `uses_lcam = false`. Both halves of that were wrong and each caused a
       different failure: lowest-static-priority with no anti-starvation made
       vector accesses arbitrarily slow behind any scalar stream, and declaring
       no TLB/LCAM use made every vector element access INVISIBLE to
       disambiguation. This module replaces the static tail with a priority
       round-robin, and every vector request declares its real resource use.

  ===> VECTOR ELEMENT ADDRESSES GO THROUGH THE DTLB LIKE ANY OTHER ACCESS.
       Bare-physical addressing for the vector drains is not an option, and not
       for a translation reason: a physically-addressed vector access cannot be
       compared against the virtually-indexed scalar LCAM traffic, so
       cross-queue disambiguation becomes impossible. TLB use is not an
       optimization knob here, it is a correctness precondition of `mem-order`.

  ===> ONE GRANT COVERS ALL THREE RESOURCES. The D$ lane, the LCAM port and the
       TLB port are gated by the SAME policy decision in the SAME cycle, and all
       three revert to scalar-first ordering on the round-robin boundary. Three
       independent handshakes would let a vector stream hold disambiguation or
       translation while yielding the cache port, which is exactly the
       monopolization the spec forbids.

  Governing spec anchors: loadstore.rst `dcache-arbiter` (the whole policy),
  loadstore.rst `mem-order` (per-element LCAM contention, the predicted-overlap
  hold), loadstore.rst `mem-subsystem` and `vector-bw-ceiling` (it sits outside
  the cache; bandwidth is the scalar port's), loadstore.rst `vec-load-algo` step
  3 and `vec-store-algo` steps 2 and 4 (who the requestors are).
  Plan v2 targets P2 and P6, plan step E5.

<|begin_module|>

  <|begin_parameters|>
  The whole module is elaborated only under `usingRVV` — a Scala `Boolean`
  derived from `BoomCoreParams`, never a hardware `Bool`. In a vectors-off build
  it is ABSENT, not tied off, and lsu.scala's `lsu_sched` chain is textually the
  baseline chain, so the emitted RTL is bit-identical to pre-Caracal BOOM v4. Do
  not gate on rocket-chip's `usingVector`; that is a different switch.

  `lsuWidth: Int` — the number of D$ request lanes, taken from BOOM's existing
  core parameter: 1 on Small/Medium, 2 on Large/Mega. Legal range 1..2. This is
  also the width of `dtlb.io.req`, of the LCAM search ports and of `dmem_req` in
  lsu.scala, which is why one parameter names all four.

  `dcacheArbiterMode: String` — from VectorParams, either "single" or
  "dual-dynamic". It must agree with `lsuWidth` ("single" implies 1,
  "dual-dynamic" implies 2) and elaboration must fail on a mismatch rather than
  silently picking one, because a mistyped mode that fell back to "single" would
  read as a performance bug months later.

  "dual-dynamic" means the two lanes form ONE DYNAMIC POOL, not a static
  scalar-lane-0 / vector-lane-1 split. A static split is the tempting
  simplification and it breaks work conservation in both directions: an
  all-scalar workload would be capped at one lane, and an all-vector workload
  likewise — the spec requires both to reach two.

  `rrPhases: Int` — the length of the round-robin rotation. FIXED AT 4, matching
  the spec's "at least one grant every 4 cycles" verbatim. It is a named
  parameter only so the bound is greppable next to the counter that implements
  it; it is not a tuning knob, because changing it changes a published bound.

  `numVecRequestors: Int` — 2, derived and not chosen: the vector load drain and
  the vector store drain. The scalar LSU is the third requestor but is not
  counted here because it is not a port of this module — it arbitrates in
  lsu.scala's own chain (see the ports section).

  From VectorParams: `lcbEntries` (for the credit-count width) and
  `dcacheArbiterMode`. From `HasBoomCoreParameters`: `lsuWidth`, `numLdqEntries`,
  `robAddrSz`. Derived, as named vals rather than inline arithmetic:
    `rrPtrSz      = log2Ceil(rrPhases)`             2 bits
    `lcbCreditSz  = log2Ceil(lcbEntries + 1)`
  No width in this file may be a literal.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit pair, following the hierarchy.yaml
  defaults: posedge `core_clk`, ACTIVE-HIGH SYNCHRONOUS `core_reset`, single
  clock domain. Both are used: the round-robin pointer and the two blocked flags
  reset to a defined state (see the logic section).

  ---- From the two drains: the requests ----

  `io.ld_req : Vec(lsuWidth, Flipped(Decoupled(VecMemAccess)))` — from VecLsu's
  `ld_beat` (VecBeatExpander, `isStore = false`).
  `io.st_req : Vec(lsuWidth, Flipped(Decoupled(VecMemAccess)))` — from `st_beat`.

  `ready` IS THE GRANT, and it is the grant for all three resources at once —
  that is the contract VecBeatExpander is written against. The payload is the
  drain-to-arbiter bundle declared in VecBundles beside `VecElemAccess`: virtual
  address, `eew`, byte enable, first/last markers, the wrapped `MicroOp` (whose
  cursor fields carry destination PRN and byte offset), store `data`
  (`dmemBeatBytes * 8` bits), the three resource-use bits `uses_tlb`,
  `uses_dcache`, `uses_lcam`, and `lcam_range_len` for the one-shot unit-stride
  range query.

  The three use bits are DATA, not constants, and that is the whole point.
  A load beat asserts all three (the vector analogue of
  `will_fire_load_agen_exec`). A store's pre-commit translate pass asserts
  `uses_tlb` and `uses_lcam` but NOT `uses_dcache`; its post-commit write pass
  asserts `uses_dcache` only. A US entry raises `uses_lcam` on its FIRST beat
  only, with `lcam_range_len` — the whole contiguous range is disambiguated by
  one range-overlap query — while every SSI beat queries per element.

  ---- To and from lsu.scala: the shared-resource seam (VecLsuCoreIO) ----

  `LsuResourceClaim` is a three-field bundle { `tlb`, `dcache`, `lcam` : Bool },
  declared IN THIS FILE and bound by name from the LSU delta. It mirrors the
  three local `var`s `tlb_avail` / `dc_avail` / `lcam_avail` that lsu.scala's
  `lsu_sched` already threads through its chain, so the seam is expressed in the
  baseline's own terms.

  `VecLsuCoreIO` itself is declared in `lsu.scala` beside `LSUCoreIO`, NOT here
  and NOT in VecLsu — it exposes the host's own LDQ/STQ state, so the host owns
  the type. It COMPOSES `LsuResourceClaim` (this file's declaration) and never
  re-declares it. One declaration each way, bound by name.

  - `io.scalar_demand : Vec(lsuWidth, Input(new LsuResourceClaim))` — the raw
    scalar claim on lane `w` this cycle, i.e. the OR of the `can_fire_*` terms
    weighted by the `uses_*` columns of the `lsu_sched` table. Combinational,
    available before the chain resolves. Used ONLY to detect contention for the
    trace and the assertions; the grant does not depend on it.
  - `io.scalar_avail  : Vec(lsuWidth, Input(new LsuResourceClaim))` — the
    RESIDUAL availability after the scalar chain has finished claiming on lane
    `w`: the final values of `tlb_avail`, `dc_avail`, `lcam_avail`. This is what
    the low-priority (default) path consumes.
  - `io.vec_claim     : Vec(lsuWidth, Output(new LsuResourceClaim))` — the
    ELEVATED-path pre-claim. The LSU must clear the corresponding `*_avail` vars
    at the HEAD of the chain, before its first `lsu_sched` call, so that every
    scalar term on that lane sees the resource already taken.
  - `io.vec_fire      : Vec(lsuWidth, Valid(VecMemAccess))` — Output. The
    granted beat, forwarded verbatim. lsu.scala muxes it into `dmem_req(w)`,
    `exe_tlb_valid(w)`/`exe_tlb_vaddr(w)` and `lcam_addr(w)`/`do_ld_search(w)` /
    `do_st_search(w)` under the same `usingRVV` gate.
  - `io.dmem_req_ready : Input(Bool)` — `io.dmem.req.ready`. A grant is
    qualified by it, so a granted beat is a FIRED beat and the drain never has
    to remember an ungranted one.

  ---- The two inbound suppression seams ----

  - `io.lcb_free_count : Input(UInt(lcbCreditSz.W))` — VecLoadCoalescingBuffer's
    `io.free_count`, the number of free assembly entries.
  - `io.hold_ldq : Input(UInt(numLdqEntries.W))` — one bit per LDQ entry, from
    VecOrderHold: this load is held behind an older overlapping vector store.
    Same shape as the LCB's `io.kill_ldq`, deliberately, so the two masks are
    built the same way.

  ---- Trace ----

  `io.vec_trace_en` is not a port: tracing is gated by the shared `VecTrace`
  plusarg, which is read inside the helper.

  ===> THERE IS NO `busy`, NO `active`, NO `vec_fu_ready` AND NO FLUSH PORT.
       No output of this module reaches an issue unit (plan rule 6 / the
       vector-LSU invariant), and it needs no `brupdate` or `rob_flush` input
       because it holds NO state scoped to an instruction — only a free-running
       phase counter and two per-requestor blocked flags, none of which is
       speculative and none of which needs undoing. A reviewer should reject any
       addition to this list.
  <|end_ports|>

  <|begin_logic|>
  ---- The requestor set ----

  //@req-spec-lsu.h1
  Three requestors share the `dmem.req` lane(s) — `lsuWidth` wide, 1 on Medium
  and 2 on Large/Mega — together with the DTLB port and the LCAM port that are
  the same width: (a) scalar LSU fire, load or store, which is every `can_fire_*`
  term of lsu.scala's existing `lsu_sched` table; (b) the vector LOAD drain, US
  beat-expanded or SSI per-element, arriving on `io.ld_req`; (c) the vector STORE
  drain, arriving on `io.st_req` — the pre-commit translate pass and the
  post-commit write pass out of `stq_execute_queue` are the same requestor on
  this port, distinguished only by their `uses_*` bits.

  The two drains are SEPARATE requestors and never arbitrate against each
  other outside this module. `addvector`'s load-priority mux in the shared
  register-read stage silently dropped store grants whenever a load was
  granting in the same cycle, and was worked around by making the store not
  advertise its agen. Target P4 (a vle and a vse overlap) is a direct
  consequence of them being two ports here.

  ---- The rotation ----

  //@req-spec-lsu.h3
  //@req-spec-lsu.h6
  A single `rr_ptr` register of `rrPtrSz` bits, reset to 0, increments modulo
  `rrPhases` EVERY cycle, unconditionally — it is a free-running phase counter,
  not a last-granted pointer. The phase assigns the HEAD position of the
  priority order:

    phase 0 : scalar
    phase 1 : vector load drain  (store drain second, if lsuWidth = 2)
    phase 2 : vector store drain (load drain second, if lsuWidth = 2)
    phase 3 : scalar

  Because the counter is free-running, every requestor owns the head position at
  least once in any window of `rrPhases` = 4 consecutive cycles, which is the
  published bound stated as a hardware property rather than as an argument about
  traffic. Advancing on grant instead would let a requestor that is never ready
  freeze the rotation and starve the others.

  The bound is over cycles in which the requestor CAN be granted. A phase
  whose owner is suppressed by `io.dmem_req_ready`, by an LCB credit or by an
  order hold, or whose translation misses in the DTLB, does not consume the
  guarantee — those are structural, not arbitration, and pretending otherwise
  would make the bound unfalsifiable.

  ---- The priority floor ----

  //@req-spec-lsu.h4
  //@req-spec-lsu.h5
  Scalar memory ops carry the higher BASE priority, expressed structurally: in
  the default (non-elevated) case the vector requestors sit at the TAIL of
  lsu.scala's existing chain, so they can only take what the scalar chain left in
  `io.scalar_avail`, and the baseline scalar ordering is unchanged term for term.
  Scalar additionally owns 2 of the 4 phases outright. A vector requestor
  reaches the head position only in its own phase and only for ONE cycle, so a
  burst of vector element accesses can delay a scalar load or store by at most
  one cycle per rotation and can never indefinitely block one. This is what
  keeps scalar memory latency at baseline, which is target P6, and it is the one
  property to measure first if a scalar regression appears.

  ---- Anti-starvation: when elevation actually happens ----

  Two registered flags, `blocked_ld` and `blocked_st`, reset to false. A flag is
  SET at the end of any cycle in which that requestor had a valid request on any
  lane and received no grant, and CLEARED on any grant. A vector requestor is
  elevated to the head position only in its own phase AND only while its flag is
  set. In steady state, therefore, no elevation happens at all: the drains are
  served opportunistically from the tail out of the lanes the scalar chain did
  not use, and the scalar chain runs exactly as in baseline. Elevation is the
  escape valve, not the normal path — which is why the floor and the 4-cycle
  bound do not fight each other.

  //@req-spec-lsu.h9
  //@req-spec-memord.b9
  Elevation asserts `io.vec_claim` on that lane for exactly the resources the
  winning beat's `uses_*` bits name, and the LSU clears the matching `*_avail`
  vars at the head of its chain. So the SAME policy decision gates the D$ lane,
  the LCAM port and the TLB port — one grant, three resources, never three
  handshakes. That is what puts an SSI access's PER-ELEMENT LCAM searches under
  this arbiter rather than beside it: a scatter/gather consumes disambiguation
  bandwidth at exactly the rate it consumes cache bandwidth, and unmetered
  per-element searches would starve scalar disambiguation while the D$ port
  looked healthy.

  //@req-spec-lsu.h10
  //@req-spec-lsu.h11
  Elevation lasts EXACTLY ONE CYCLE. There is no lock, no burst counter, no
  "vector owns the port until this OP.v is done" state, and no per-instruction
  state of any kind: every beat re-arbitrates from scratch, and at the phase
  boundary all three resources revert to scalar-first ordering. That is why a
  single vector `OP.v` — even one draining 256 element accesses — cannot
  monopolize disambiguation or translation. The absence of that state is also
  what makes this module squash-free and flush-free.

  ---- The grant, per lane ----

  //@req-spec-lsu.h7
  //@req-spec-lsu.h8
  The grant is COMBINATIONAL and single-cycle; the arbiter adds no pipeline
  stage and holds no payload register, because a beat must be able to fire every
  cycle. Per lane `w`, in order:

  1. Build the eligible set: `ld_req(w).valid && !ld_suppressed(w)` and
     `st_req(w).valid`.
  2. Order it by the current phase — elevated requestor first if its blocked
     flag is set, then the other drain, and the scalar chain's residual is what
     remains.
  3. A requestor wins lane `w` when, for every resource its `uses_*` bits name,
     that resource is still available on that lane: `io.scalar_avail` on the
     default path, or unconditionally on the elevated path (nothing scalar can
     have taken a pre-claimed resource). `uses_dcache` additionally requires
     `io.dmem_req_ready`.
  4. Drive `ld_req(w).ready` / `st_req(w).ready` for the winner and
     `io.vec_fire(w)` with its payload, unmodified — the arbiter is
     payload-transparent and must not rewrite a field.

  At `lsuWidth = 2` the two lanes are resolved as an independent pass over the
  same eligible set with lane 0's winner removed, so up to TWO pending requests
  are granted per cycle and the assignment of requestor to lane is dynamic. The
  policy is WORK-CONSERVING in both directions and in both modes: a lane is
  never left idle while an eligible request exists that could use it, so an
  all-scalar workload uses both lanes for scalar, an all-vector workload uses
  both for vector, and a mix splits them under the floor plus rotation above. In
  particular a phase whose owner is not requesting is immediately reassigned —
  the phase grants a POSITION, never a reservation.

  ---- Suppression seam 1: the LCB credit ----

  ===> DO NOT SUPPRESS A LOAD-DRAIN REQUEST ON `io.lcb_free_count`. An earlier
       version of this spec required exactly that, on the reasoning that
       "over-granting is not recoverable while under-granting only costs a cycle".
       **The second half is false, and the first half does not apply to a beat.**
       A beat always lands in an entry its own op allocated at LAUNCH, so it needs
       no new credit and can never over-grant; and when `free_count` reaches 0 with
       every outstanding beat belonging to an already-allocated entry, under-granting
       costs FOREVER, not a cycle — nothing can fire, so nothing frees an entry.
       This is not a rare case either: at `EMUL = lcbEntries` ONE group owns every
       entry, so `free_count` is 0 for that op's entire lifetime. Measured on
       `ms14_vls_e64_m8` (LMUL 8, `lcbEntries` 8): 8 entries allocated, 5 of 32
       placements, `group_done` never fired, and the dependent store never started.
       The binding guard is the per-PRN `lcb_alloc_rdy` test that VecBeatExpander
       already applies to every beat; this coarse one is redundant for correctness.

  `io.lcb_free_count` is therefore still an input (VecLoadCoalescingBuffer owns
  publishing it, and may never back-pressure a response, because a blocked response
  holds an MSHR against the very drain that would free the entry) but it must NOT
  feed load-drain eligibility. A load-drain request is suppressed only by
  `io.hold_ldq`.

  ===> AND THE LCB CREDIT IS THE ONLY THING THAT MAY THROTTLE THE LOAD DRAIN.
  A LOAD NOW UNDER-RESERVES ITS ELEMENT-QUEUE REGION — decision D9/D10:
  `min(worstCase, ldResvMembers * vLen/eew)` entries with `ldResvMembers` = 4,
  while stores still reserve the worst case — so for a long load THE DRAIN IS
  WHAT FREES THE ROOM ITS OWN FILL SIDE IS WAITING FOR. Two consequences for
  this module, and both are "do not add anything":
    (1) Nothing here may suppress or de-prioritize a load-drain request on the
        grounds that its reservation is small, nearly full, or exhausted. A
        suppression term derived from region occupancy would close the loop
        fill -> region-full -> drain-suppressed -> region-never-freed, which is
        a DEADLOCK rather than a slowdown, and it would be invisible in the
        arbiter's own trace.
    (2) The anti-starvation flag matters more than it did, not less: a load
        drain that goes ungranted is now blocking its own agen, not just its
        own completion. `blocked_ld` already covers it — it is set by "valid
        and no grant" with no notion of why — and that is exactly the property
        to preserve.
  The arbiter needs no new port for any of this: it has no notion of a
  reservation and must not acquire one.

  ---- Suppression seam 2: the predicted-overlap hold ----

  //@req-spec-memord.b18
  A load-drain request whose `uop.ldq_idx` bit is set in `io.hold_ldq` is NOT
  GRANTED THE LCAM OR THE D$ PORT: it is masked out of the eligible set on the
  same terms as a credit-suppressed request, so neither resource is claimed for
  it and no LCAM query is issued on its behalf. VecOrderHold decides WHO is
  held — the younger load overlapping an older still-draining vector store, in
  the SSI/SSI and US/SSI combinations that do not forward — and this module is
  where the hold physically takes effect. The release is that store's element
  cursor completing its active set, which reaches here only as `io.hold_ldq`
  dropping the bit; the arbiter has no notion of stores completing and must not
  acquire one.

  The hold is a PERFORMANCE mechanism, not the correctness floor — the
  `order_fail` replay path is. So a conservative or even a wrong bit in
  `io.hold_ldq` may only cost cycles here, and this module must never treat a
  held load as an error or drop its request. It stays pending and is granted
  as soon as the bit clears.

  ---- Assertions and trace ----

  Assert, per lane: at most one requestor is granted; a default-path grant never
  takes a resource `io.scalar_avail` said was gone; `io.vec_claim` is asserted
  only in an elevation cycle and only for bits the winning beat's `uses_*` name;
  and a granted request always has `io.vec_fire(w).valid`. Assert that no grant
  ever goes to a load whose `hold_ldq` bit is set — that one is the seam most
  likely to be mis-wired, and it fails silently as a data corruption rather than
  a hang.

  Emit guarded trace lines through the shared `VecTrace` helper, gated on the
  `vecTrace` plusarg (off by default) and `!reset`, one line per key event:
  a grant (module, `rob_idx`, lane, requestor, phase, and the three `uses_*`
  bits), and an elevation (`rob_idx`, requestor, phase, how many cycles the
  blocked flag had been set). Two lines are enough to reconstruct the whole
  policy from a run, which matters because there are no unit tests — validation
  is end-to-end VCS plus Whisper cosim only.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
TARGET P2 — sustained >= 1 D$ access per cycle per lane while the arbiter grants
and the queue is non-empty. This is a constraint on the implementation, and it
forces two choices rather than leaving them open. First, the grant path is
purely combinational: no request queue, no payload register, no handshake
pipeline stage inside this module, so a beat granted in cycle t is a
`dmem.req` in cycle t. Second, `blocked_ld` / `blocked_st` and `rr_ptr` are the
only registers, and neither is in the grant's combinational path — they select an
ordering, they do not sequence it. An arbiter that registered its grant would
halve vector memory throughput and would satisfy nothing else in exchange.

TARGET P6 — scalar kernels show no regression against the vector-disabled build.
The scalar-priority floor is what buys this and it must actually hold: with the
drains idle the module contributes nothing but a mod-4 counter, and with the
drains saturated a scalar op waits at most one cycle per rotation. `usingRVV =
false` removes the module entirely and leaves lsu.scala's chain textually
identical to baseline, so P6's control arm is the same RTL.

NO COMBINATIONAL LOOP THROUGH lsu.scala, and this is a real hazard worth stating
because the obvious wiring creates one. `io.scalar_avail` depends on the scalar
`can_fire_*` terms AND on `io.vec_claim`. `io.vec_claim` therefore must depend
only on registered state (`rr_ptr`, the blocked flags) and on the drains' request
valids and `uses_*` bits — never on `io.scalar_avail`. The default-path grant may
consume `io.scalar_avail` freely, since it drives no claim back. Keeping the
elevation decision registered-only is what makes the two directions acyclic.

Bandwidth ceiling, for calibration rather than as a target: peak vector memory
throughput is `lsuWidth * dmemBeatBytes` per cycle — 64 bits/cycle on Medium,
128 on Large/Mega — because the cache and its request interface are unchanged.
This module cannot raise that ceiling; it exists so the ceiling is SHARED
fairly. A wider vector cache port is out of scope for the whole plan.
<|end_perf|>

<|begin_dependencies|>
VecBundles — the drain-to-arbiter request payload (`VecElemAccess` plus the
`data`, `uses_tlb`/`uses_dcache`/`uses_lcam` and `lcam_range_len` fields that
VecBeatExpander's spec records as belonging there). The arbiter is
payload-transparent, so it binds to that bundle and declares no view of its own.
`LsuResourceClaim`, by contrast, is declared in THIS file: it is the
arbiter-to-lsu.scala seam and has no other producer, and the LSU delta binds to
it by name.

VectorParams — `dcacheArbiterMode` and `lcbEntries`.
VecTrace — the guarded trace helper.
Binds to `HasBoomCoreParameters` for `lsuWidth`, `numLdqEntries`, `robAddrSz`.

Instantiates nothing, and must not: an arbiter that instantiated a queue would
be holding in-flight state, which the vector-LSU invariant forbids.

Its counterparties, each of which must be checked from the other side in plan
step R2:
  VecBeatExpander x2 (`req` in, `ready` out — `ready` IS the grant, covering D$
    plus TLB plus LCAM together),
  VecLoadCoalescingBuffer (`io.free_count` in),
  VecOrderHold (the held-load mask in),
  LSU / lsu.scala (the `LsuResourceClaim` pair, `io.vec_fire`,
    `io.dmem_req_ready`) — the edit_existing node that owns folding this into
    `lsu_sched`, `dmem_req`, `dtlb.io.req` and `lcam_addr`,
  VecLsu (the parent that wires all of the above).
<|end_dependencies|>

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
  VecFreeList — the group-granular rename free list for the vector and VL
  physical register spaces.
*/
  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/rename/VecFreeList.scala,
  package boom.v4.vec.generated.rename.
  depends_on VecBundles, VectorParams, VecTrace.
  Instantiated TWICE by VecRenameSpace: as the vector space
  (numPhysRegs = numVecPhysRegisters, maxGroupSize = 8,
  freeDiscipline = "stale_group") and as the VL space
  (numPhysRegs = numVlPhysRegisters, maxGroupSize = 1,
  freeDiscipline = "committed_ptr"). The two spaces differ only in those
  parameters.

  This is the vector analogue of `v4/exu/rename/rename-freelist.scala`
  (`RenameFreeList`) and deliberately keeps that module's structure and signal
  names (`free_list`, `spec_alloc_list`, `br_alloc_lists`, `sels`, `sel_fire`,
  `alloc_masks`, `sel_mask`, `dealloc_mask`, `debug_freelist`, ...). The ONLY
  structural change is that a request is a GROUP of up to `maxGroupSize` PRNs
  instead of one PRN. Wherever this file says "unchanged" it means: the scalar
  file's expression, only wider.

  ===> ALL-OR-NOTHING IS THE WHOLE FORWARD-PROGRESS ARGUMENT. A shared
       (segmented) OP.v needs a `pvdest` group AND a `pvtmp` group, and both come
       from THIS free list — no temp pool, no second free list. It gets both in
       one cycle or neither, and on neither the WHOLE dispatch group stalls and
       retries intact. No headroom reservation and deliberately no
       `numVecTmpGroups` parameter: rename is in program order, so an op that
       cannot allocate stalls while every older op keeps committing and freeing
       its stale group, and the oldest op never starves. What WOULD deadlock is a
       PARTIAL allocation — PRNs held until the op commits, and the op unable to
       commit until it allocates the rest.

  ===> ALL-OR-NOTHING IS A PROPERTY OF THE GRANT, NEVER OF DISPATCH. `alloc_ok`
       is one bit and cannot be partial. `alloc_fire` is PER LANE and routinely
       IS partial, because BOOM's `dis_stalls` is a PREFIX SCAN
       (`core.scala:773`): a purely NON-vector hazard on lane 2 — `ldq_full`, say
       — still lets lanes 0..1 dispatch while lane 2 retries next cycle. A single
       whole-bundle fire bit here would consume lane 2's window PRNs anyway, lane
       2 would allocate a SECOND group on its retry, and the first group would be
       owned and freed by nobody: the M1 leak/double-allocate class, reached with
       no vector-side mistake at all. See the `alloc_fire` port and "Partial-prefix
       fire and per-lane consumption" in the logic section.

  Governing spec anchors: midcore.rst `free-list`, `cii-shared-mapping`,
  `vl-vtype-rename`, `regfiles-bypass`; issue.rst `cii-shared-sched`.

<|begin_module|>

  <|begin_parameters|>
  Constructor parameters, all Scala `Int`/`String` resolved at elaboration. The
  module is elaborated only under `usingRVV`; a vectors-off build instantiates
  neither instance, so nothing here appears in that build's RTL.

  `numPhysRegs` — the size of the physical register space this instance manages:
  `numVecPhysRegisters` (default 96) for the vector instance,
  `numVlPhysRegisters` (default 64) for the VL one. Both come from VectorParams;
  neither is written as a literal here.

  //@req-spec-rename.h11
  State is one bit per PRN, so the VL instance's free vector is exactly 64 bits
  wide at the default — a plain bit vector over `numPhysRegs`, identical in kind
  to the vector instance's 96-bit one. A VL producer allocates one fresh PRN out
  of it, because `maxGroupSize` is 1 there and a group of one IS a single PRN.
  That is the whole reason the VL space needs no free list of its own design.

  `numArchRegs` — the PRNs the committed map table holds permanently: 32 for the
  vector space (the RMT maps all 32 architectural vregs at all times, whatever
  the current LMUL), 1 for the VL space. Used only by the capacity `require` and
  an assertion on `initial_allocation`; it indexes nothing.

  `maxGroupSize` — the largest member count of one EMUL group, VectorParams'
  `maxMembers`. 8 for the vector space, 1 for the VL space.

  `freeDiscipline` — a `String`, "stale_group" or "committed_ptr", selecting which
  commit-side free rule applies. `require` it is one of those two spellings: a
  mistyped discipline falling back to the other would leak or double-free PRNs
  thousands of cycles later. Explicit rather than derived from `maxGroupSize`,
  because the two are unrelated decisions that merely correlate in today's two
  instances.

  //@req-spec-rename.f4
  //@req-spec-rename.f5
  `allocWidth` is DERIVED, not passed: `coreWidth * 2 * maxGroupSize`, i.e.
  `coreWidth * 16` for the vector instance. It is the number of prioritized
  outputs taken from BOOM's existing `SelectFirstN` selector
  (`boom.v4.util.SelectFirstN`, reused as-is, not reimplemented) and hence the
  number of distinct free PRNs this list can hand out per cycle. That formula,
  because every lane of a dispatch group may be an LMUL=8 OP.v taking a whole
  group at once — AND a SHARED OP.v takes TWO whole groups (`pvdest` + `pvtmp`),
  so the worst-case lane demand is `2 * maxGroupSize`, not `maxGroupSize`.

  ===> CORRECTED AT E-PREP. This read `coreWidth * maxGroupSize`, which is the
  worst case for an UNSHARED group and exactly HALF the real worst case. It was not
  a sizing preference; it silently defeated the all-or-nothing check below and
  DOUBLE-ALLOCATED PRNs. Once `total_demand` exceeds `allocWidth`, the `alloc_ok`
  condition `forall i < allocWidth: r_valid(i) || i >= total_demand` degenerates to
  `forall i: r_valid(i)` — the `i >= total_demand` escape can never fire, so the
  check cannot represent "demand exceeds what one cycle's selection can supply" and
  reports OK anyway. The trailing lane's window then starts at or past `allocWidth`,
  its `r_sel` index truncates to the low bits, and it is handed a PRN ALREADY
  GRANTED to an older lane in the same cycle. Two live vector groups then share
  physical registers, and the first free returns a PRN the other still owns.

  Reachable on 2 shared LMUL=8 ops plus any third requesting lane at
  `coreWidth = 4`. Segment load/stores are exactly the shared-op producer, so this
  is reachable from the E regression rather than theoretical — but it needs a
  specific co-dispatch, which is why no test found it and only the index-width
  warning pointed at it.

  Note the corroborating evidence that this was a slip rather than a decision:
  `baseW` below is already `log2Ceil(coreWidth * 2 * maxGroupSize + 1)`, i.e. the
  prefix sum was sized for the TRUE worst case by the same author in the same file.

  Rejected: adding `total_demand <= allocWidth` to `alloc_ok` instead. It converts
  corruption into a LIVELOCK — `alloc_ok` is one bit for the whole bundle and
  dispatch re-presents the identical group next cycle, so an over-demanding group
  never makes progress. Fixing that would need the per-lane stall mask this spec
  forbids two paragraphs down. Widening the selector keeps the grant
  all-or-nothing, which is the property the deadlock argument rests on.

  //@req-spec-rename.f12
  `deallocWidth` is likewise derived: `commitWidth * maxGroupSize` (BOOM's
  `retireWidth`, which equals `coreWidth`, times 8) — wide enough for every
  commit lane to return a whole stale group in one cycle. Commit cannot be
  back-pressured, so this width is a correctness requirement, not a throughput
  choice.

  `pregSz` = `log2Ceil(numPhysRegs)`; `n` = `numPhysRegs`, named as in the
  scalar file so the expressions read the same. The lane count is `coreWidth`,
  which is the spelling the spec's `allocWidth = coreWidth*8` uses and is the same
  number that the rename-stage modules (and VecMapTable) call `plWidth` — one
  quantity, two inherited names, not two parameters.

  Two elaboration-time requires:
  `numPhysRegs >= numArchRegs + allocWidth + 2 * maxGroupSize`, so at least one shared
  OP.v can always be renamed; and `allocWidth >= 2 * maxGroupSize`, so a shared OP.v
  can always draw BOTH of its groups from one cycle's selector output. KEEP THE
  SECOND even though the corrected `allocWidth` formula now satisfies it for every
  `coreWidth >= 1`: it is the assertion that states the property the deadlock
  argument depends on, and it is the check that fires if the formula is ever
  narrowed back. A require that is currently trivially true is not a dead require.

  ===> THE `allocWidth` TERM IN THE FIRST REQUIRE IS NOT SLACK, AND OMITTING IT
  DEADLOCKS THE IDLE MACHINE. Found at E7 on MegaBoom. The pre-selection stage
  below parks one PRN per port in a holding register and refills a port ONLY from
  `free_list`, so `allocWidth` PRNs are permanently out of the list — the reachable
  pool at rest is `numPhysRegs - numArchRegs - allocWidth`, not
  `numPhysRegs - numArchRegs`. At `numVecPhysRegisters = 96`, `coreWidth = 4`,
  `maxGroupSize = 8` that reachable pool was EXACTLY ZERO: the 64 free PRNs all sat
  in the 64 ports, `free_list` was 0 with the machine idle, and the first OP.v to
  consume port 0 left it unrefillable. Because lane windows start at `base(0) = 0`,
  every later request needs port 0, so `alloc_ok` was false forever with 63 free PRNs
  stranded in ports 1..63 — a hang, at the first vector load, with no assertion.
  Note what the old arithmetic was really claiming: that `numArchRegs + 2*maxGroupSize`
  PRNs let "one shared OP.v always be renamed". That is true only of a list whose
  free bits are all reachable, which this one's are not. The prefix invariant stated
  two paragraphs down — `r_valid(i)` implies `r_valid(j)` for `j < i` — holds only
  while refills succeed; sizing the file is what keeps it true, since the scalar
  pipeline this file keeps verbatim does not re-compact the ports to restore it.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel/BOOM default and are implicit: single
  `clock`, rising edge; single `reset`, ACTIVE HIGH and SYNCHRONOUS. Every
  register below is reset-initialized from `initial_allocation` or to zero.

  `initial_allocation` — Input, `numPhysRegs` bits: the reset value of
  `free_list`, driven by VecRenameSpace with PRNs `numArchRegs` and above free.
  Same port and meaning as the scalar free list's.

  Request side — ONE request per rename LANE, not per PRN, unlike the scalar file
  where a lane and an allocation port are the same thing:

  - `reqs` — Input `Vec(coreWidth, Bool())`: lane `w` needs a destination group.
    ===> `reqs` IS FIRE-INDEPENDENT. VecRenameSpace drives it from
    `ren2_alloc_reqs`, NEVER from `ren2_alloc_fire`: `alloc_ok -> dis_ready ->
    dis_fire -> reqs` would be a combinational loop. Consumption is qualified
    by `alloc_fire` instead, which is exactly baseline's split — there
    `can_allocate` comes from the pre-selection REGISTER and never from `reqs`.
  - `req_members` — Input `Vec(coreWidth, UInt((log2Ceil(maxGroupSize)+1).W))`,
    the group's member count, i.e. the uop's `v_emul`. Legal 1..`maxGroupSize`
    when `reqs(w)`; always 1 in the VL instance.
  - `req_shared` — Input `Vec(coreWidth, Bool())`: lane `w` needs TWO groups of
    `req_members(w)` members this cycle rather than one. Tied false in the VL
    instance.
    ===> `req_shared` DOES NOT MEAN `uop.is_shared`, and the parent must not
    connect `is_shared` to it. It means "needs two groups". A segmented STORE
    is `is_shared` and does need a `pvtmp` group, but it has NO vector
    destination — its `dst_rtype` is not `RT_VEC` — so it needs exactly ONE
    group. Requesting two and using one LEAKS A GROUP FOREVER: commit frees
    only `stale_pvdest` and `pvtmp`, and neither would ever name the unused
    one. VecRenameSpace therefore raises `req_shared(w)` only when the lane
    needs BOTH groups (`needs_pvdest && needs_pvtmp`) and, in the tmp-only
    case, routes the single granted group into `uop.pvtmp` — see the grant side.

  Grant side:

  - `alloc_pvdest` — Output `Vec(coreWidth, Vec(maxGroupSize, UInt(pregSz.W)))`,
    lane `w`'s FIRST granted group, member by member. `alloc_pvtmp` is the same
    shape and carries the SECOND granted group, meaningful only where
    `req_shared(w)`.
    The names describe the two-group case. This module does not know, and must
    not try to infer, which architectural role a granted group plays: in the
    tmp-only case (a segmented store) the parent takes the single group off
    `alloc_pvdest` and writes it into `uop.pvtmp`, leaving `pvdest` unwritten.
    Do NOT add an `is_shared` or `tmp_only` input to re-derive that here — one
    decision, one place, and that place is VecRenameSpace.
  - `alloc_ok` — Output `Bool()`, ONE bit for the WHOLE bundle: every requesting
    lane's full demand is available. VecRenameSpace consumes it as the vector
    half of `dis_ready`. It is the GRANT, and a grant cannot be partial.
  - `alloc_fire` — Input `Vec(coreWidth, Bool())`: lane `w` actually renamed AND
    dispatched this cycle, i.e. `dis_fire(w) && reqs(w)`. PER LANE, NOT one bit —
    see the header's second `===>` block for why a single bit is the M1
    double-allocate. Allocation state changes only for the lanes whose bit is
    high, so a cycle in which the scalar side stalls SOME lanes for its own
    reasons consumes only the dispatched lanes' PRNs; a cycle with no bit set
    consumes nothing.

  Reclaim side:

  - `dealloc` — Input `Vec(deallocWidth, Valid(UInt(pregSz.W)))`. Under
    "stale_group", the committing uops' `stale_pvdest` members, flattened
    `maxGroupSize` slots per commit lane; under "committed_ptr", only slot 0 of
    each lane is ever valid and carries the outgoing committed pointer.
  - `dealloc_tmp` — Input, same shape: the committing uops' `pvtmp` members.
    Present only when `maxGroupSize > 1`.
  - `ren_br_tags` — Input `Vec(coreWidth+1, Valid(UInt(brTagSz.W)))`. Note
    `coreWidth+1`, NOT `allocWidth+1`: a snapshot is taken between rename LANES,
    and the scalar file's `allocWidth+1` is the same thing only because there
    `allocWidth == plWidth`.
  - `brupdate` (`new BrUpdateInfo`) and `rollback` (`Bool`) — BOOM's existing
    ports, unchanged in name and meaning.

  Observability: `debug_freelist`, Output `UInt(numPhysRegs.W)`, as in the scalar
  file; and `stall_cnt_inc`, Output `Bool()`, high when a requesting lane was
  denied. The latter feeds a `perfEvents` counter and no functional logic — the
  stall rate is the headline number for vector-PRN sizing, so it is a counter and
  not only a trace line.
  <|end_ports|>

  <|begin_logic|>
  ---- State ----

  //@req-spec-rename.f1
  Three registers, exactly as in the scalar free list: `free_list`, a
  `numPhysRegs`-bit vector reset to `initial_allocation` whose bit `i` is set when
  PRN `i` is currently un-used and therefore available to rename;
  `spec_alloc_list`, same width, reset to zero, holding everything allocated since
  the last commit point so a rollback can return it; and `br_alloc_lists`, a
  `Vec(maxBrCount, UInt(numPhysRegs.W))` of per-branch-tag snapshots.

  ---- Selection and the registered pre-selection stage ----

  //@req-spec-rename.f2
  //@req-spec-rename.f7
  `sels = SelectFirstN(free_list, allocWidth)` produces `allocWidth` one-hot
  selections of DISTINCT free PRNs in priority order. Each output is an
  independent one-hot over the whole space, so the PRNs it names bear no relation
  to one another: a group assembled from them is NON-CONTIGUOUS by construction,
  and no contiguous-run allocator, run-length search or fragmentation-recovery
  walk exists anywhere in this module. The vector map table stores one PRN per
  architectural vreg precisely so that this is sufficient.

  Keep the scalar file's per-port pre-selection pipeline verbatim: port `i` holds
  `r_valid(i)` and `r_sel(i) = RegEnable(OHToUInt(sels(i)), sel_fire(i))`,
  refilling when it is empty or its held PRN was consumed. Two consequences are
  load-bearing.

  The PRN leaves free_list at PRE-SELECT time (sel_mask), not at grant time.
  debug_freelist therefore adds the held-but-ungranted PRNs back before the
  "returning a free physical register" assertion, as the scalar file does. Do
  not "fix" this by moving the removal to grant time.

  Second, capacity is judged from the `r_valid` REGISTER outputs, so the stall
  decision is available at the start of the cycle rather than after the
  selector's priority chain — which is what keeps `SelectFirstN` over 96 bits
  with 24 outputs, the deepest structure here, off the rename critical path.
  Because the selector is priority-ordered the valid ports form a PREFIX:
  `r_valid(i)` implies `r_valid(j)` for all `j < i` in steady state.

  ---- Demand, and the all-or-nothing decision ----

  //@req-spec-rename.f3
  //@req-spec-rename.f6
  Per-lane demand is `demand(w) = req_members(w) * (1 + req_shared(w))` when
  `reqs(w)`, else 0 — an entire EMUL group per OP.v, doubled when the lane asked
  for TWO groups (`req_shared`, which is NOT `is_shared`: a segmented store asks
  for one). A prefix sum over `demand` assigns each lane a contiguous WINDOW of
  pre-selection PORTS: lane `w` takes ports `base(w) until base(w)+demand(w)`,
  where `base(w)` is the sum of the younger lanes' demands. The window is
  contiguous in PORT INDEX only — the PRNs those ports hold are not. Lane `w`'s
  `alloc_pvdest` members are its window's first `req_members(w)` ports and,
  when shared, `alloc_pvtmp` is the remainder. This port-window assignment, not
  a static port-per-lane binding, is what lets one lane draw two groups.

  TWO INDEX BASES ARE FORCED TO ZERO RATHER THAN LEFT TO WRAP, and both are about
  the port index being WIDER than `r_sel` is deep — `base` is sized for
  `total_demand`, which reaches `allocWidth` inclusive, so an index one past the end
  wraps onto port 0 instead of faulting, and a wrap is indistinguishable from the
  double-allocation the corrected `allocWidth` exists to prevent. First: a lane with
  `reqs(w)` LOW has demand 0, so its base equals the running total and can be
  `allocWidth` exactly; its grants are unconsumed, but the index must still be legal,
  so read the base as 0 when the lane is not requesting. Second: `pvtmp`'s base is
  `base(w) + members`, which for an UNSHARED lane is the NEXT lane's base — and for
  the youngest unshared lane, one past the end. `alloc_pvtmp` is already masked to 0
  when `!req_shared(w)`, so zero that base too and the value is unchanged.

  Assert per lane, on the worst-case index of each window, that it lies within
  `allocWidth`; do NOT clamp. A clamp silently reproduces the wrap it is hiding,
  whereas the assertion is what fires if the demand arithmetic above is ever
  changed. The assertion is not redundant with `alloc_ok`: `alloc_ok` is about
  whether enough PRNs are FREE, this is about whether the windows FIT.

  //@req-spec-rename.e14
  //@req-spec-issue.c3
  //@req-spec-rename.e15
  `alloc_ok` is the single all-or-nothing verdict: the total demand
  `base(coreWidth-1) + demand(coreWidth-1)` is covered by valid pre-selection
  ports, i.e. for every port `i`, `r_valid(i) || i.U >= total_demand`. Demand is
  computed from `reqs`, which is fire-independent, so `alloc_ok` carries NO
  dependence on `alloc_fire` and the loop the ports section warns about cannot
  form. Both of a
  shared OP.v's groups sit inside its own window, so `alloc_ok` cannot be true
  for a partial pair — there is NO encoding here for "pvdest granted, pvtmp
  pending", and none may be added. A partially allocated shared op would hold
  PRNs it cannot free until it commits, and cannot commit until it gets the rest;
  that is the deadlock this rule removes, which is why it is a property of the
  GRANT rather than of a retry loop.

  //@req-spec-vrf.b11
  When `alloc_ok` is low no lane allocates and nothing is consumed: the whole
  dispatch group stalls and retries intact next cycle — trivially
  order-preserving, no per-lane stall mask. This is the ONLY place the vector-PRN
  capacity limit is enforced, and it is enforced as a consequence of the free
  vector being short, not by any explicit bound. There is NO group-credit counter
  and none may be added: it would duplicate state `free_list` already holds and
  the two copies would disagree on a mispredict. Raise `stall_cnt_inc` and emit
  one VecTrace line (module "VecFreeList", event "stall", the oldest requesting
  lane's `rob_idx`, `total_demand`, free count) so a dispatch stall is
  attributable to PRN exhaustion rather than guessed at.

  Capacity, visible where it bites: 32 of 96 vector PRNs are permanently held
  by the committed map table, so (96-32)/8 = 8 LMUL=8 groups may be in flight
  and at most 4 of those segmented. Read VectorParams' maxRenamableGroups /
  maxRenamableSegGroups; do not recompute them here.

  ---- Partial-prefix fire and per-lane consumption ----

  A high `alloc_ok` says the demand IS available; it does not say the bundle
  dispatched. `dis_stalls` is a PREFIX SCAN in `core.scala:773`, so a hazard that
  has nothing to do with vectors — `ldq_full`, `stq_full`, a ROB-full edge on lane
  2 — dispatches lanes 0..1 and stalls lane 2. `alloc_fire` reports that per lane,
  and EVERY consumption term below is qualified by `alloc_fire(w)`, never by an
  OR-reduction of it and never by `alloc_ok`:
    - port `i` is CONSUMED only when `i` lies inside the window of a lane with
      `alloc_fire(w)` high;
    - lane `w` contributes to `alloc_masks` (hence to `spec_alloc_list` and the
      branch snapshots) only when `alloc_fire(w)` is high;
    - a non-firing lane's window ports keep their held PRNs in the pre-selection
      registers, so its retry next cycle gets PRNs from the same pool and
      allocates ONE group, not a second one.

  A single fire bit is the M1 leak/double-allocate. With one bit, lane 2's
  window PRNs leave `free_list` in a cycle lane 2 did not dispatch; lane 2
  retries and is granted a SECOND group; the first is named by no uop, so no
  stale_pvdest and no pvtmp ever frees it and it is gone until reset. Nothing
  vector-specific is needed to reach it.

  Two properties make per-lane consumption safe and both are worth asserting.
  First, a fire implies the grant: `alloc_ok` gates `dis_ready`, which gates every
  lane's `dis_fire`, so `alloc_fire(w)` may never be high while `alloc_ok` is low.
  Second, fires are PREFIX-SHAPED for the same reason `dis_stalls` is: if lane `w`
  fires then every REQUESTING lane `k < w` fires too. Together they mean the set of
  consumed ports is exactly the prefix `[0, sum of the firing lanes' demands)` —
  the surviving held ports are the high, contiguous remainder, and `sel_mask`
  remains a PREFIX-shaped OR over ports, just a shorter prefix. Assert
  both; a non-prefix fire pattern is not expressible by this port-window
  assignment, and would silently hand two lanes overlapping windows on the retry.

  ---- The temp group ----

  //@req-spec-rename.e4
  //@req-spec-vrf.d7
  The `pvtmp` group is allocated FROM THIS FREE LIST — the main vector free
  list — out of the same cycle's `SelectFirstN` output as `pvdest`, through the
  same port window, with the same member count. It is not drawn from a reserved
  region, a separate pool or a temp register file; no such thing exists in this
  design. Every downstream reclaim path therefore treats a `pvtmp` PRN as an
  ordinary vector PRN, and this module holds no `pvtmp` special case beyond the
  doubled demand above.

  Disjoint ports each holding a distinct PRN mean a lane's `pvdest` and `pvtmp`
  groups cannot overlap and two lanes cannot be handed the same PRN. Assert it:
  no PRN appears twice across the granted members of the lanes whose
  `alloc_fire(w)` is high (a non-firing lane's members are not allocated, so
  including them would make the assertion fire on a legal partial-prefix cycle). A
  double-allocated PRN corrupts architectural vector state with no other symptom,
  so the assertion is cheap relative to the bug.

  For members beyond `req_members(w)`, pad the output group with member 0's PRN.
  NEVER pad with 0.U. PRN 0 is a live register belonging to someone else, and a
  consumer that iterated all maxGroupSize slots without masking by v_emul would
  free it — exactly the M1 double-free of PRN 0. Padding with the group's own
  member 0 makes that same mistake harmless.

  ---- Free-list update: bit-vector ORs only ----

  //@req-spec-rename.f10
  //@req-spec-rename.f11
  Every mask below is a `numPhysRegs`-bit vector and every combination of them is
  an OR or an AND-NOT, so setting 8 bits for an OP.v is the same expression as
  setting 1 with more terms: no new logic, only wider ports.

  `sel_mask` is the OR of `sels(i)` over ports that fire, where port `i` fires
  when `i` is inside the window of a lane whose `alloc_fire(w)` is high, or when
  the port is empty and refilling. // PER LANE: a bundle-wide `alloc_fire.orR`
  here is the leak of the partial-prefix section. `dealloc_mask` is
  `com_deallocs | br_deallocs | rollback_deallocs`, unchanged in form. The state
  update is the scalar file's verbatim:
  `free_list := (free_list & ~sel_mask) | dealloc_mask` and
  `spec_alloc_list := (spec_alloc_list | alloc_masks(0)) & ~dealloc_mask & ~com_despec`.

  There IS a `despec` port, `Vec(deallocWidth, Valid(UInt(pregSz.W)))`, paired
  1:1 with `dealloc`: the same lane and the same member predicate, but carrying
  the committing group's OWN `pvdest` members where `dealloc` carries the stale
  ones it frees. `com_despec` is their OR of one-hots, and it appears ONLY in the
  `spec_alloc_list` update -- it frees nothing. Not registered, unlike
  `com_deallocs`: it only ever removes bits, so acting a cycle early can at worst
  decline to roll back an already-committed PRN, which is correct.

  This port is REQUIRED, and omitting it is a silent-corruption bug, not a
  simplification. `rollback_deallocs` returns `spec_alloc_list` WHOLESALE; if a
  committed PRN is never retired out of that set it stays speculative forever,
  and the first rollback after it commits hands a live architectural register
  back to the free list to be re-allocated under a second name. The symptom is
  the leak assertion in VecRenameSpace firing on the population check -- in the
  over-free direction, since that check is an equality.

  (An earlier revision of this spec said "there is no `despec` port... those
  serve the scalar immediate free list". That was backwards and is corrected
  here: `despec` is driven by the GENERIC `RenameStage`, and it is
  `ImmRenameStage` that ties it off. `isImm` remains genuinely absent -- that
  mode only adds `& ~com_deallocs` to the branch snapshots, which a vector group
  does not need because its busy bits are cleared by group-done in
  VecBusyTable, which never touches the free list.)

  ---- Branch reclaim: unchanged ----

  //@req-spec-rename.f8
  //@req-spec-rename.e8
  //@req-spec-rename.h22
  `alloc_masks` is the scalar file's `scanRight`, over LANES instead of ports,
  each lane contributing — QUALIFIED BY ITS OWN `alloc_fire(w)`, exactly as the
  scalar file qualifies by its per-port `sel_fire` — the OR of the one-hots of ALL
  PRNs it granted: `pvdest` members and `pvtmp` members together. A lane that did
  not fire contributes ZERO, so its window's PRNs enter neither `spec_alloc_list`
  nor any branch snapshot; if they did, a rollback or mispredict would "return"
  PRNs that were never taken and the leak assertion would fire on a legal
  partial-prefix cycle. `br_alloc_lists` is snapshotted
  on `ren_br_tags` and maintained as `br_alloc_lists(i) & ~br_deallocs |
  alloc_masks(0)`, with `br_deallocs = br_alloc_lists(brupdate.b2.uop.br_tag) &
  Fill(n, brupdate.b2.mispredict)`. Both the `enableSuperscalarSnapshots`
  multi-snapshot path and the single-snapshot path (with its
  `PopCount(ren_br_tags.map(_.valid)) <= 1` assertion) carry over as written. A
  mispredict thus reclaims a wrong-path group whole — including a `pvtmp` group
  and a wrong-path VL producer's PRN — with no vector-specific mechanism:
  `pvtmp` is in `alloc_masks` because it was granted, which is the only property
  the branch machinery ever needed.

  ---- Commit reclaim ----

  //@req-spec-rename.f9
  //@req-spec-rename.f13
  `com_deallocs = RegNext(dealloc).map(d => UIntToOH(d.bits) & Fill(n, d.valid))
  .reduce(_|_)`, unchanged from the scalar file INCLUDING the `RegNext` — that
  commit-side pipe stage is part of the existing timing contract, not an
  accident. With `deallocWidth = commitWidth * maxGroupSize` a commit lane
  presents every member of its `stale_pvdest` group in one cycle and the whole
  stale group is freed together. Under "committed_ptr" the same OR tree serves a
  single valid slot per lane.

  //@req-spec-rename.e9
  `dealloc_tmp` is OR-ed into `com_deallocs` by the identical expression, so a
  committing shared OP.v frees its `pvtmp` group in the SAME cycle as its stale
  `pvdest` group. `pvtmp` has no stale predecessor of its own and was never
  installed in the RMT, so commit is the only event that can free it; freeing it
  earlier — at the rendezvous, say — would hand it to a younger op while a
  mispredict could still need it back.

  //@req-spec-rename.h17
  Under `freeDiscipline == "committed_ptr"` the PRN freed at commit is the
  OUTGOING committed pointer — the PRN the committing producer displaces in the
  single-entry committed VL map table — driven onto `dealloc(w*maxGroupSize)` from
  VecMapTable's `com_stale_resps` (its `exportComStale` output, which presents the
  displaced value BEFORE the commit install). There is no `stale_pvl` uop field
  and this module
  must not expect one: the committed map table already holds the value, so a
  per-uop copy could only go stale. The two disciplines differ ONLY in who drives
  `dealloc`; everything below it is shared, which is why one definition serves
  both spaces.

  `rollback_deallocs = spec_alloc_list & Fill(n, rollback)` returns everything
  allocated since the last commit point, unchanged.

  ---- Assertions and trace ----

  Assert `!(debug_freelist & dealloc_mask).orR` ("returning a free physical
  register") as the scalar file does — the cheapest detector of the double-free
  class. Assert `1 <= req_members(w) <= maxGroupSize` whenever `reqs(w)`, and
  that no `dealloc`/`dealloc_tmp` slot beyond a committing group's member count
  is valid.

  //@req-spec-rename.f5
  STARVATION WATCHDOG, parameter `allocStarveWatchdog` (Int, default 4096, 0
  disables and emits no register). Count cycles of uninterrupted `!alloc_ok` and
  assert the count stays within it. `!alloc_ok` is ordinary back-pressure for a
  few cycles — a wide group waiting on the pre-selection stage to refill — but it
  can never be the steady state, because a rename stall blocks dispatch, which
  stops commit, which is the only thing that returns PRNs: the stall sustains
  itself and the machine is DEADLOCKED, not slow. Report `total_demand` and
  `PopCount(free_list)`, since "free=0 while demand=16" is the entire diagnosis.

  Without this assertion the only symptom is core.scala's generic "Pipeline has
  hung" firing `boom_timeout` cycles later, naming nothing about vector rename
  and pointing the reader at the LSU. Real case: at LMUL=8 an in-flight load
  group starved at the D$ interface, so its `group_done` never broadcast, its
  PRNs stayed busy and the free list drained to zero. Size the threshold far
  above any legitimate refill — one selection port refills per cycle, so even a
  full `allocWidth` reservoir reloads in `allocWidth` cycles. Same shape as
  VecCiiFlush's `drainWatchdog`.

  Three assertions guard the per-lane fire specifically:
    - `alloc_fire(w)` implies `reqs(w)` — a lane cannot consume a window it never
      asked for;
    - `alloc_fire(w)` implies `alloc_ok` — the grant gates `dis_ready`, so a fire
      without a grant means the parent wired `alloc_ok` past `dis_ready`;
    - the fires are PREFIX-SHAPED: `alloc_fire(w)` implies `alloc_fire(k)` for
      every requesting lane `k < w`. Violating it means dispatch is no longer a
      prefix scan, and the port-window assignment above must be revisited before
      anything else is.
  Also assert that `ren_br_tags(w+1).valid` implies `dis_fire(w)`, so the snapshot
  boundary and the allocation it must contain are the same event per lane.

  Emit one guarded VecTrace line per granted lane (event "alloc": `rob_idx`,
  `v_emul`, the granted `pvdest` members, the `pvtmp` members when shared) and
  one per commit-side free (event "free": `rob_idx`, the freed members), through
  the shared `VecTrace` helpers, gated on the `vecTrace` plusarg and `!reset`,
  off by default. This project has no unit tests, so these lines plus
  `debug_freelist` are the only way a PRN leak becomes attributable to a cycle.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput: one dispatch group per cycle, up to `coreWidth` groups and
`allocWidth` PRNs granted per cycle, with no allocation latency — the grant is
combinational from `reqs` through the pre-selection registers, because rename and
dispatch are a single cycle.

TIMING: this module and VecMapTable are the top timing risk in the design. The
critical path is `free_list` (register) into `SelectFirstN`'s priority chain over
`numPhysRegs` bits with `allocWidth` outputs, and it must NOT extend into the
grant path: the pre-selection registers cut it, so `reqs` to `alloc_pvdest` is
the prefix sum over `coreWidth` demands plus a mux per member, while the
selector's depth is paid on the following cycle's refill. Run the timing spike on
this arrangement before the rename integration step lands; if a deeper cut is
needed, add it on the refill side, never between `reqs` and `alloc_ok`.

Area: `numPhysRegs * (2 + maxBrCount)` bits of state plus the selector; the
branch-snapshot array dominates and is unchanged in kind from the scalar file.

The `stall_cnt_inc` rate is a first-class performance number, not diagnostics:
with 8 renamable LMUL=8 groups and 4 segmented ones this module is the expected
limiter on vector memory-level parallelism, and the counter is how that is tested
rather than assumed.
<|end_perf|>

<|begin_dependencies|>
VectorParams — `numVecPhysRegisters`, `numVlPhysRegisters`, `maxMembers`,
`vecPregSz`, `vlPregSz`, and the derived `maxRenamableGroups` /
`maxRenamableSegGroups` quoted in the capacity note. VecTrace — the guarded
trace helpers, emit-only. VecBundles is a `depends_on` edge for compile order,
but note that this module's request and grant ports are plain `Vec`s of `UInt`
and `Valid(UInt)` and it consumes NO bundle declared there: a free list traffics
in PRNs, and wrapping them in a named bundle would only give VecRenameSpace a
second place to get the member ordering wrong.

Reused from BOOM unchanged, not reimplemented: `boom.v4.util.SelectFirstN`,
`boom.v4.common.BrUpdateInfo`, and `HasBoomCoreParameters` for `coreWidth`,
`retireWidth`, `maxBrCount`, `brTagSz`, `enableSuperscalarSnapshots`.
Instantiates nothing.

Instantiated by VecRenameSpace as `freelist`, twice. It owns three obligations on
this seam, all of them correctness: `reqs` from `ren2_alloc_reqs` and therefore
FIRE-INDEPENDENT; `alloc_fire` as the PER-LANE `dis_fire(w) && ren2_alloc_reqs(w)`;
and `req_shared(w)` raised only when lane `w` needs BOTH groups, with the tmp-only
case taking the single granted group off `alloc_pvdest` into `uop.pvtmp`.

VecRenameSpace also owns the
LOCKSTEP CONTRACT on this seam: it must drive this module from the REGISTERED
`ren2_uops`/`dis_fire`, never combinationally from `dec_uops`. A cycle-early
drive is the M1 free-list double-free — the vector fields at dispatch described
the next cycle's bubble uop and two ops freed PRN 0. This module cannot detect
that from inside; VecRenameSpace must not create it.
<|end_dependencies|>

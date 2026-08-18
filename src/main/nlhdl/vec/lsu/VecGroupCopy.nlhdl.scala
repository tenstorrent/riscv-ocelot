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
  VecGroupCopy — makes the freshly renamed destination group of a vector memory
  OP.v that executes NO element architecturally correct, by copying
  `pvdest <- stale_pvdest` one VLEN-wide member at a time, with no memory
  traffic of any kind.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecGroupCopy.scala,
  package boom.v4.vec.generated.lsu, group vec_lsu.
  depends_on MicroOp, VecBundles, VectorParams, VecTrace.
  Instantiated ONCE, as `gcopy` inside VecLsu. It is a load-path structure and
  has no store-direction twin, because a vector store has no VRF destination.

  WHY THE NODE EXISTS. Only an immediate-AVL `vset` resolves VL in the front end;
  for every other form VL is renamed, so a `VL = 0` OP.v reaches issue and
  resolves `pvl` to 0 at execute. It executes no element — yet its `pvdest` group
  was already renamed and freshly allocated, and its ROB entry cannot commit until
  that group holds the architecturally correct value. Nothing else in the machine
  would ever write it. This module is that writer.

  ===> IT MUST NEVER STEAL BANDWIDTH FROM A REAL LOAD. It borrows the Load Unit's
       existing VRF ports and adds NONE: it reads `stale_pvdest` on `R2` and
       writes on `W0`, the same two ports the Load Coalescing Buffer uses. Those
       ports are therefore arbitrated by a STRICT-PRIORITY mux in which an active
       load drain ALWAYS wins. The copy is pure catch-up work with nothing
       waiting on its latency, so it is the correct loser. A round-robin here
       would let a `VL = 0` op displace a real element return.

  ===> SCOPE, AND THIS IS THE EASY MISTAKE. The standalone copy is ONLY for the
       no-execution case: `VL = 0`, or a group in which every element is
       inactive. The ORDINARY masked / partial-tail load is NOT this module —
       there the inactive lanes are pre-loaded from `stale_pvdest` on `R2` INSIDE
       VecLoadCoalescingBuffer, overlapped with the load's memory latency, and
       merged into its single `W0` write. Routing the ordinary case through here
       would resurrect exactly the serial `sCopyRd`/`sCopyWr` copy prologue that
       v2 exists to delete, and would break performance target P5.

  Governing spec anchors: case_study.rst `case-vl-zero` (every requirement in
  this file), midcore.rst `vrf-ports` (the canonical port table: `R2`, `W0`),
  midcore.rst `group-done-wb` (completion), midcore.rst `old-vd` (`stale_pvdest`
  is a group, distinct from `pvs3`), loadstore.rst `load-coalesce` (who owns the
  ordinary masked/tail case), plan v2 section 5 rules 3, 4 and 9.

<|begin_module|>

  <|begin_parameters|>
  The whole module is elaborated only when `usingRVV` is true (a Scala Boolean of
  `BoomCoreParams`, never a hardware `Bool`, and never rocket's `usingVector`).
  In a vectors-off build it is ABSENT, not tied off, so the emitted RTL stays
  bit-identical to pre-Caracal BOOM v4.

  `gcopyEntries` — Int, the depth of the pending-copy work list, counted in GROUP
  MEMBERS and not in instructions. Default `numVecPhysRegisters - 32` rounded up to
  a power of two (128 at the default sizing; it was 64 while that parameter was 96,
  and tracks it). It has a hard floor rather than being a tuning knob, and the
  reason is in the logic section: there is no back-pressure path out of this
  module, so the depth must be the structural bound on simultaneously pending
  member copies. Require at elaboration
  `gcopyEntries >= min(numVecPhysRegisters - 32, numLdqEntries * maxMembers)`,
  naming the offending parameter on failure.

  Everything else comes from VectorParams through `HasVectorParams` — `vLen`,
  `vecPregSz`, `maxMembers`, `vecVLSz` — and from `HasBoomCoreParameters` for
  `robAddrSz`, `ldqAddrSz` and `numLdqEntries`. No width in this file is a
  literal.

  There is deliberately NO parameter selecting which VRF ports to use, and no
  `lsuWidth`-dependent second write lane. The ports are `R2` and `W0`, fixed by
  the canonical table in midcore.rst `vrf-ports`, and at `lsuWidth = 2` this
  module still uses `W0` only: the copy rate is bounded by the single `R2` read,
  so a second write port would buy nothing and would put a third writer on `W1`.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the hierarchy default and Chisel's implicit convention:
  single `core_clk` domain, posedge `clock`, ACTIVE-HIGH SYNCHRONOUS `reset`. No
  second domain, no asynchronous reset, no gated clock.

  ---- the launch, from the load side of VecLsu ----

  `launch` — Flipped(Valid(...)), no `ready`. Its `bits` carry the OP.v's
  `MicroOp` (fields read: `pvdest`, `stale_pvdest`, `v_emul`, `v_seg_nf`,
  `vconfig` for `vta`/`vma`, `is_shared`, `pvtmp`, `rob_idx`, `ldq_idx`,
  `v_is_whole_reg`), plus `vl_zero` (a Bool) and `all_inactive` (a Bool).

  There is no `ready` for the same reason VecScalarOperandRead publishes none:
  the producing stage overwrites its register every cycle and cannot hold a
  grant, so the launch MUST be accepted unconditionally. That is what forces the
  overflow-free work list below, and it is not negotiable at this seam.

  `vl_zero` is taken from the load instance of VecScalarOperandRead
  (`ld_opnd.out.bits.vl_zero`) and `all_inactive` from VecMaskStream inside
  `ld_elem_agen`. Both are consumed as given: this module evaluates neither `vl`
  against zero nor the mask against `v0`, because those two values have a single
  owner each and a second derivation is how two units come to disagree about
  which elements survive.

  `kill` — Bool, valid with `launch`. The parent's already-resolved kill for the
  launching OP.v (branch mispredict through `brupdate`, or `rob_flush`), resolved
  outside so this module holds no `br_mask` comparison of its own.

  ---- the Load Unit's VRF ports, and the strict-priority mux over them ----

  `lcb_r2_req` — Input Valid(addr: UInt(vecPregSz.W)). The Load Coalescing
  Buffer's `stale_pvdest` read request.
  `lcb_r2_data` — Output UInt(vLen.W). The `R2` result returned to the LCB.
  `lcb_w0` — Input Valid(addr: UInt(vecPregSz.W), data: UInt(vLen.W),
  mask: UInt((vLen/8).W)). The LCB's `W0` write.

  `vrf_r2_req` — Output Valid(addr: UInt(vecPregSz.W)) to VecRegFile read port
  `R2`.
  `vrf_r2_data` — Input UInt(vLen.W), the `R2` result.
  `vrf_w0` — Output Valid(addr, data, mask) to VecRegFile write port `W0`.

  The LCB's three signals pass THROUGH this module. There is no grant, ready or
  nack back to the LCB in either direction, by design — see the logic section.

  ---- completion ----

  `group_done` — Output Valid(VecGroupDone) (declared in VecBundles): the member
  PRN vector, the member count and the owning `rob_idx`. ONE port. It is one of
  exactly three group-done producers in the design, alongside the LCB and the CII
  writeback completion, which is the `numVecWbPorts = 3` that VecGroupReady's
  matcher and VecPipeline's wakeup network are sized to.

  ---- squash ----

  `squash` — Input Valid(ldq_idx: UInt((1 + ldqAddrSz).W)): the youngest surviving
  LDQ index broadcast by VecSquashUnit. Entries younger than it are discarded by
  pointer rollback, the same mechanism the element queues use and the same
  `ldq_idx` key by which LCB entries are invalidated.
  `flush` — Input Bool (`rob_flush`): discard everything in one cycle.

  ===> `1 + ldqAddrSz`, NOT `ldqAddrSz`, corrected at E6. Every other `ldq_idx` in this
  design — `MicroOp.ldq_idx`, `VecRangeEntry.ldq_idx`, the rollback request, the LCB's
  key — carries the wrap-disambiguating carry bit, and `IdxAgeYt` (the comparator this
  port exists to feed) requires equal widths. At the literal width this either fails to
  elaborate or silently aliases two different LDQ generations onto one index.

  ===> SQUASH REACHES THE EXPANSION REGISTER, NOT ONLY THE ENQUEUED ROWS. Corrected at
  E6, where the logic section was found to scope squash to rows already in the FIFO while
  scoping only `flush` to the expansion/staging registers. A launch still MID-EXPANSION
  when its `ldq_idx` is squashed would keep pushing the remaining members of a group that
  no longer exists — and this subsystem's whole reason for doing pointer rollback rather
  than drain-and-discard is that a squashed load's `pvdest` PRNs are returned to the free
  list within a few cycles. Those pushes would then write registers that already belong to
  another instruction. Kill the expansion register too, on the same age test.

  ===> FLUSH ALSO SUPPRESSES THE ONE-CYCLE NO-COPY COMPLETION. Same correction. The
  `vta = 1` path decides in the launch cycle and pulses `group_done` the next one; the
  logic section's flush list enumerated the FIFO and the expansion register but not that
  pending pulse, so a flush landing exactly one cycle after a no-copy launch still emitted
  a completion — clearing a ROB entry the flush had just retired.

  ---- what is deliberately absent ----

  NO `busy`, NO `full`, NO `ready`, and no signal of any kind that reaches
  VecIssueUnit, VecQueueReservation or a dispatch gate. That is the vector-LSU
  group invariant (plan section 5 rule 3), and a back-pressure output added here
  would be a failed review regardless of measured performance.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. What launches a copy, and what must not ----

  //@req-spec-lsu.m2
  //@req-spec-lsu.m3
  A launch is a vector memory OP.v on the LOAD side that will generate NO memory
  access at all — `vl_zero`, or `all_inactive`. With no active element the LSU
  executes nothing: the agens push no element access and no range entry, the drain
  side sees no beat, and VecQueueReservation releases the op's whole reserved
  region as surplus. Nothing on that path ever writes the VRF, yet the group was
  freshly allocated at rename and the ROB entry cannot clear its busy bit and
  commit until the group is architecturally correct. This module supplies the
  write, and it is the only producer that can: it is reached without a D$ request,
  a translation or an LCAM lookup.

  Three exclusions, each a real bug if it is got wrong:

  Whole-register accesses (`v_is_whole_reg`: `vl1re*`/`vs1r*`) NEVER launch, even
  with `vl == 0`: their length does not depend on VL, so the op is not a
  no-execution op at all — it writes its whole group from memory, and a copy
  launched for it would race a real load for the same PRNs.

  A vector STORE never launches: it has no VRF destination and clears its ROB busy
  bit through the single `lsu_clr_bsy` VecLsu already emits once its active element
  set has translated, which for an empty set is immediate.

  For a SHARED (segmented) load the group completed here is `pvtmp`, not `pvdest`,
  because `pvtmp` is what the LSU half produces and its group-done is what wakes
  the coprocessor half. `pvtmp` is a rendezvous temporary and not architectural
  state, so its DATA need not be copied — the launch emits the `pvtmp` group-done
  with no VRF traffic, and `pvdest` correctness for that instruction belongs to
  the coprocessor half under its own `vta`/`vma` policy.

  ---- 2. The `vta`/`vma` decision: copy, or complete without executing ----

  //@req-spec-lsu.m4
  Tail-undisturbed (`vta = 0`) means the new destination group must equal the OLD
  value of the architectural vreg group — bit for bit, every member. `pvdest` and
  `stale_pvdest` are two DIFFERENT physical groups (see the MicroOp delta: two
  fields naming two groups, never to be merged), so "equal the old value" is a
  physical data movement and not a rename trick.

  //@req-spec-lsu.m9
  //@req-spec-lsu.m10
  Tail-agnostic (`vta = 1`) with nothing else to preserve means the tail may hold
  ANY value, so no copy is required: the entry takes the complete-without-execute
  path and emits its group-done IMMEDIATELY — the cycle after launch, out of the
  registered launch, with no VRF read, no VRF write and no work-list entry.
  Registered rather than combinational because `group_done` feeds the vector
  wakeup, and driving it combinationally from the launch closes the loop issue
  wakeup to `iss_uops` to decode to AGEN to launch and back to `group_done`. One
  register breaks it; it is the same rule the LCB obeys for its own group-done.

  The full decision is one predicate:
    `must_preserve = (vta == 0) || (vma == 0 && !vl_zero)`
  Read it as: with `vl_zero` the whole register group is tail, governed by `vta`
  alone. With `all_inactive` and `vl > 0` the in-VL lanes are inactive BODY
  lanes, governed by `vma`, and the lanes above VL are tail, governed by `vta`.

  //@req-spec-lsu.m5
  When `must_preserve` holds, the OP.v performs a GROUP COPY
  `pvdest <- stale_pvdest`: up to `v_emul` (equivalently `v_emul * v_seg_nf` for
  a segmented form) whole-register, `vLen`-wide copies, ONE PER GROUP MEMBER,
  reading member `k` of the stale group and writing member `k` of the new group.
  Member `k` of `stale_pvdest` pairs with member `k` of `pvdest` and with no
  other; the two groups need not be contiguous in the PRN space and neither is
  addressed as a base plus an offset.

  A WHOLE-REGISTER copy, with an all-ones W0 byte mask, and no lane merging of
  any kind. That is not laziness: with no element written there is nothing to
  merge, so the byte-granular mask and the overlay logic that the LCB needs are
  both absent here. When `must_preserve` holds only because of ONE of vta/vma,
  copying the whole register is still correct, because "agnostic" permits any
  value INCLUDING the old one. That single observation is what keeps this
  module a whole-register mover and keeps every partial-lane case in the LCB.

  ---- 3. The pending work list: member-granular, and overflow-free ----

  Launches that need a copy are pushed into `pending`, a FIFO of `gcopyEntries`
  rows, ONE ROW PER GROUP MEMBER: `{ dst_prn, src_prn, last, rob_idx, ldq_idx }`.
  A group's members are pushed as a consecutive run in member order, and the run's
  final row carries `last`.

  THE FIFO HOLDS PRN NAMES ONLY — NEVER vLen-WIDE DATA. Storing register data
  per pending member would be `gcopyEntries * vLen` bits (16 kbit at the
  defaults, two thirds of the whole VRF) for a rare event. The data exists only
  in the single staging register of section 4.

  There is no `full` output because there can be no back-pressure (see the ports
  section), so the depth must be a BOUND rather than a choice, and there is one: a
  row exists only for a vector PRN that has been renamed and has not committed,
  and the free list cannot have more than `numVecPhysRegisters - 32` such PRNs
  outstanding, because the committed map table maps all 32 architectural vregs at
  all times. A group whose copy is still pending has not committed, so its members
  are still allocated and still inside that bound. `gcopyEntries` at its default
  therefore cannot overflow. Assert on overflow anyway, naming `rob_idx`: it can
  only fire if that argument is wrong, and a dropped copy leaves a wrong
  architectural register value, the hardest class of bug to find in a cosim log.

  A launch expands into its member rows at one row per cycle, held meanwhile in an
  expansion register with a member counter, so an EMUL=8 group occupies the push
  path for 8 cycles. Launches cannot arrive fast enough to overrun it — the load
  side grants at most one OP.v per cycle — but assert that no launch is dropped
  rather than trusting that.

  ---- 4. The copy datapath: two stages, one member per cycle ----

  The head member of `pending` is copied in two stages. Stage A requests `R2` at
  `src_prn`. Stage B captures the `vLen`-bit result into `copy_data` (with
  `copy_dst`, `copy_last`, `copy_rob_idx`) and requests `W0` at `copy_dst` with
  that data and an all-ones byte mask. Each stage's request is granted only in a
  cycle the mux of section 5 leaves the respective port free.

  The staging register is 1 deep and holds the ONLY `vLen`-wide data in the
  module. Its point is that losing `W0` is cheap: the member has already been
  read, so the request is re-presented next cycle and no `R2` cycle is wasted. The
  head row pops when its `W0` write is granted — never when its `R2` read is —
  so a lost write cannot lose the row.

  Do NOT collapse this into a single cycle by wiring R2's combinational read
  result straight into W0's data. It would put a vLen-wide read-decode plus
  write-decode path in one cycle at 1 GHz, and it would also force the mux to
  grant BOTH ports in the same cycle — turning two independent lowest-priority
  requests into one paired request that a busy LCB can starve for far longer.

  Sustained rate with the ports idle is one member per cycle: `R2` for member
  `k+1` overlaps `W0` for member `k`, because the staging register frees in the
  cycle its write is granted.

  ---- 5. The strict-priority mux, and why it lives here ----

  //@req-spec-lsu.m13
  //@req-spec-lsu.m14
  The Load Unit's `R2` and `W0` are arbitrated by a STRICT-PRIORITY mux, declared
  in this module because this module owns the obligation and a single owner is the
  only way the priority cannot be disputed. The LCB is slot 0 and this module is
  slot 1. Slot 0 is a pure COMBINATIONAL PASS-THROUGH: `vrf_r2_req` is
  `lcb_r2_req` whenever `lcb_r2_req.valid`, `vrf_w0` is `lcb_w0` whenever
  `lcb_w0.valid`, `lcb_r2_data` is `vrf_r2_data` unconditionally, and the LCB
  receives no grant, ready or nack. An active load drain therefore ALWAYS wins,
  by construction rather than by an arbiter decision the LCB must observe: it
  cannot be delayed, masked or made to wait, because there is no signal by which
  it could be.

  This module's `R2` request is qualified by `!lcb_r2_req.valid` and its `W0`
  request by `!lcb_w0.valid`, each independently, in the same cycle.

  NO anti-starvation counter, and that absence is deliberate. Liveness here is
  STRUCTURAL: the VL = 0 op holds a ROB entry, commit is in program order, so a
  steady stream of younger loads fills the ROB, dispatch stalls, the in-flight
  drains finish and the ports fall idle. The copy is rare and nothing waits on
  it, so eventual progress without a cycle bound is sufficient. This is the
  exact OPPOSITE of the D$ lane in VecDcacheArbiter, which carries scalar
  traffic and therefore does need a bounded round-robin guarantee. Do not
  copy that arbiter's shape into this mux.

  ---- 6. Completion: one group-done, once ----

  //@req-spec-lsu.m8
  A copy completes with a SINGLE group-done, exactly like a real load, so the
  ROB single-shot busy-clear, the vector Busy-Table clear and the vector wakeup
  path are unchanged and need no per-entry completion counter. As each member's
  `W0` write is granted, its `dst_prn` is accumulated into `done_members`, a
  `Vec(maxMembers, UInt(vecPregSz.W))` register, with `done_count` counting them.
  When the row marked `last` is written, `group_done.valid` rises for one cycle
  carrying `done_members`, `done_count` and the row's `rob_idx`; the accumulator
  then clears. Members are copied in order and one group at a time, so one
  accumulator suffices and no per-group tag is needed.

  `group_done` has one port and two sources — the no-copy path of section 2 and
  the copy-completion path above. They are arbitrated strict-priority with the
  NO-COPY path winning, because it is a bare wire out of the registered launch
  with nowhere to wait, whereas the copy path holds its completed row at the FIFO
  head (unpopped) until it wins the port, which costs it a cycle and costs
  nothing else. `pvl` in the emitted `VecGroupDone` is invalid: a vector load is
  never a VL producer, so nothing here writes the VL register file.

  ---- 7. Scope, restated where a generator will read it ----

  //@req-spec-lsu.m11
  //@req-spec-lsu.m12
  On EVERY masked or partial-tail vector op the inactive element lanes of the
  destination must be preserved or filled per `vta`/`vma`. That obligation is
  design-wide, and it is discharged in three different places, of which this
  module is only the third and smallest:
    - a load that executes AT LEAST ONE element: the LCB pre-loads the inactive
      lanes of the affected members from `stale_pvdest` on `R2` and overlays the
      arriving elements before its single `W0` write, overlapping the load's
      memory latency (loadstore.rst `load-coalesce`);
    - arithmetic: the CII coprocessor pulls old-`vd` on its `STALE_VD` source
      slot and merges the undisturbed lanes itself, writing `W2`;
    - THIS module, and only for the degenerate `elem_start >= vl` corner: the
      no-execution `VL = 0` or fully-inactive `vta = 0` group, where there is no
      element and therefore no merge — just a copy.
  The standalone copy is needed for that case ALONE. Do not route the ordinary
  masked or partial-tail load through here to "reuse the copier": it would
  serialize a whole-group copy in front of every masked load, which is precisely
  the `sCopyRd`/`sCopyWr` prologue v2 deletes and precisely what performance
  target P5 measures.

  ---- 8. Squash, and what this module must never grow ----

  A launch is dropped in its own cycle when `kill` holds. Enqueued rows are
  discarded on `squash` by rolling the FIFO tail back past every row whose
  `ldq_idx` is younger than the broadcast index, and on `flush` by clearing the
  FIFO, the expansion register, the staging register and the accumulator in one
  cycle. Abandoning a copy — even a half-finished one — is always safe: the
  instruction is discarded, its ROB entry never commits and its `pvdest` group
  returns to the free list, so no reader can observe the group it did not finish
  writing. There is no undo path and there must not be one.

  The only state is the FIFO of PRN names, the expansion register, the one
  `vLen`-wide staging register and the member accumulator. NONE of it is scoped to
  "the current instruction" in the sense the invariant forbids: it is a program-
  order work list that many instructions may occupy at once, it never gates a
  grant, and it exports no readiness. No FSM, no concurrency ceiling of one,
  nothing an issue unit can see.

  ---- 9. Trace ----

  There are no unit tests in this project; validation is end-to-end VCS plus
  Whisper cosim. Emit `VecTrace` lines, gated on the `vecTrace` plusarg and
  `!reset`, off by default, tagged with the module name and `rob_idx`: one on
  launch (`vl_zero`, `all_inactive`, `vta`, `vma`, `must_preserve`, member count),
  one per member write (`src_prn`, `dst_prn`, member index), one per cycle a
  request is BLOCKED by the mux, and one on group-done. The blocked-cycle line is
  what distinguishes "losing the ports as designed" from "stuck", which are
  otherwise indistinguishable in a cosim log; no non-trace logic may read it.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Latency, `vta = 1` no-copy path: group-done one cycle after launch,
unconditionally. It uses no VRF port, so it cannot be blocked and must never be
placed behind the copy work list.

Throughput, copy path: one `vLen`-wide member per cycle while the Load Unit's
ports are idle, so an EMUL=8 group costs 8 port-cycles plus the pipeline fill.
There is no cycle bound when the ports are contended, by design.

THE HARD CONSTRAINT, and the one to measure: this module must add ZERO cycles to
any real vector load. It is gated by performance targets P4 (a `vle` and a `vse`
overlap, neither's issue gated by the other) and P5 (a masked/tail-undisturbed
load costs no extra cycles vs unmasked at equal active-byte count); both regress
the moment the mux grants this module a port a drain wanted, or the ordinary
masked case is routed here.

Frequency: single `core_clk` domain, 1 GHz target. Two paths to keep short — `R2`
result into `copy_data`, and `copy_data` out to `W0`. Two cycles on purpose:
there must be no combinational `R2`-to-`W0` path.

Area: `gcopyEntries * (2*vecPregSz + 1 + robAddrSz + ldqAddrSz)` bits of work list
(about 1.8 kbit at the defaults) plus ONE `vLen`-wide staging register and a
`maxMembers * vecPregSz` accumulator. The depth is a correctness bound, not a
knob: shrinking it means giving this module a dispatch-time capacity reservation
the way VecQueueReservation does for the element queues, which is a plan
amendment and not a local change.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the launch carries a `MicroOp`; the fields read are `pvdest`,
`stale_pvdest`, `pvtmp`, `v_emul`, `v_seg_nf`, `vconfig` (`vta`/`vma`),
`is_shared`, `v_is_whole_reg`, `rob_idx`, `ldq_idx`.
VecBundles — `VecGroupDone` is the completion event, declared there.
VectorParams — `vLen`, `vecPregSz`, `maxMembers`, `vecVLSz`, and
`numVecPhysRegisters` for the work-list bound.
VecTrace — the guarded trace helpers.

Instantiates nothing. Instantiated by VecLsu as `gcopy`, once.

Couples, without instantiating, to:
- VecRegFile — read port `R2` and write port `W0`, cited by number from
  midcore.rst `vrf-ports`. THIS MODULE ADDS NO VRF PORT; that is what makes it
  compatible with the canonical table rather than an amendment to it.
- VecLoadCoalescingBuffer — shares `R2` and `W0` through the strict-priority mux
  of section 5, in which the LCB is an unqualified pass-through, and owns the
  ordinary masked/partial-tail case that this module must not be given.
- VecScalarOperandRead (`ld_opnd`) — source of `vl_zero`; VecMaskStream inside
  `ld_elem_agen` — source of `all_inactive`. Neither value is re-derived here.
- VecRangeAgen / VecElemAgen — the launch's counterpart: they push no entry for a
  zero-length or fully-inactive access. VecQueueReservation releases that op's
  whole reserved region as surplus, consistent with generating no memory traffic.
- VecSquashUnit — broadcasts the surviving `ldq_idx` used for pointer rollback.
- VecGroupReady / VecPipeline / Rob — this is the third group-done producer, part
  of the `numVecWbPorts = 3` the wakeup network is sized to, and the ROB consumes
  the event as an ordinary single-shot busy-clear with no special case.
<|end_dependencies|>

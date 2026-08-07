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
  VecGroupReady — the per-source-group readiness matcher of a vector issue slot:
  it AND-reduces one vector source operand's per-member wakeup state into the
  single group-ready bit that operand contributes to `request`.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/issue/VecGroupReady.scala,
  package boom.v4.vec.generated.issue.
  depends_on VecBundles, VectorParams, VecTrace.

  ONE definition, FIVE instances in an `IQ_V_LOAD` or `IQ_V_ALU` slot and FOUR in
  an `IQ_V_STORE` slot — `rdy_vs1`, `rdy_vs2`, `rdy_vs3` and (load/ALU only)
  `rdy_vold`, all `isMask = false`, plus `rdy_vm` (`isMask = true`). Split out of
  VecIssueSlot because the same per-member match was being restated once per
  operand; the four non-mask instances differ in nothing.

  ===> `rdy_vold` (decision D6) MATCHES THE `stale_pvdest` GROUP, and it is an
       ORDINARY instance of this module with no new port and no new behaviour —
       that is the point of the decision. `stale_pvdest` is the PREVIOUS mapping
       of the destination arch vregs, so its producer is an OLDER instruction, and
       age-ordered issue grants the oldest READY entry, which does not guarantee
       an older producer has completed. Two consumers read the group anyway: the
       LCB pre-loads inactive-lane data from it on VRF port `R2` for a `vta=0` /
       `vma=0` load, and the coprocessor may pull it as the `STALE_VD` source
       slot. Without this instance both can read a BUSY group and get garbage.
       An AGGREGATE `stale_pvdest_busy` bit could not have done the job — see the
       sub-range section — which is why the fifth instance is the fix.

  ===> WHY THIS IS A SEPARATE MODULE FROM THE SLOT'S OTHER COMPARATORS: a POLICY
       split, not a tidiness one. Vector operands wake ONLY on actual completion,
       never speculatively. The slot's scalar feeders — base address, stride and
       the `.vx` operand on the INT network, the `.vf` scalar on FP — do ride
       BOOM's existing speculative load-hit wakeup unchanged, and so need the
       re-busy and replay machinery that comes with it. Nothing here may ever
       gain a speculative-wakeup input or a re-busy path; see the reject list in
       the logic section.

  ===> THE MATCH IS PER MEMBER, NOT A SINGLE BASE COMPARATOR. A consumer may read
       a SUB-RANGE of a producer's in-flight group, or a group fragmented across
       two producers, so base-PRN equality is not merely pessimistic — it never
       matches at all.

  ===> ONE GROUP-DONE PER GROUP, EVER: no repeat, no retry. A match dropped for
       one cycle is a permanent hang, so the load path, the collapse-move path and
       the match are ONE next-state expression below, never sequenced.

  Governing spec anchors: issue.rst `issue-sched-stage` ("The Vector Issue Slot"
  and "The match-port budget"), midcore.rst `group-done` ("Busy Table") and
  `spec-wakeups` ("Speculative Wakeups").
*/

<|begin_module|>

  <|begin_parameters|>
  `isMask` — Boolean, default false. True for the single `rdy_vm` instance only.
  A mask is ONE physical register, never a group: `pvm` is a plain
  `UInt(vecPregSz.W)` in the uop, not a `Vec`. With `isMask` true the member
  count collapses to the elaboration-time constant 1, the member-count port is
  not elaborated, and the AND-reduce degenerates to a single bit, so the mask
  instance costs one eighth of a pvs instance. A Scala parameter and not a
  hardware mode bit precisely so that collapse happens at elaboration.

  `numVecWbPorts` — Int, BOUND TO `VectorParams.numVecWbPorts` (3), legal range 1
  and above. The width of the vector wakeup network: how many group-done events
  may arrive in one cycle. NOT a local tuning knob — it must equal the width
  VecPipeline drives, i.e. the number of group-done producers (the Load
  Coalescing Buffer, the CII writeback completion, VecGroupCopy's
  complete-without-execute path). Set below the network width, the matcher can
  MISS a group-done, and because completion is single-shot the consumer then
  waits forever.

  // ===> IT IS NO LONGER DEFAULTED HERE. `VectorParams` now DECLARES
  // `numVecWbPorts` (3), alongside `numVecClrPorts` (3) and `numVlWakeupPorts`
  // (`aluWidth + 1`). This parameter binds to that field and re-defaults nothing;
  // VecIssueUnit and VecIssueSlot bind to the same field, so the three numbers
  // that used to be three independent literal 3s are now one declaration. If they
  // ever disagree, the matcher examines fewer ports than the network drives,
  // misses a single-shot group-done, and the consumer hangs forever.

  // Do NOT confuse numVecWbPorts with the VRF's three write ports. W0/W1 both
  // belong to the load path and the LCB aggregates them into ONE group-done, so
  // the two counts are equal at the default by coincidence, not construction.

  `groupMembers` — derived, `if (isMask) 1 else maxMembers`: the member lanes
  this instance elaborates. `maxMembers` (8) and `vecPregSz` come from
  VectorParams through `HasVectorParams`; no width here is a literal.

  `usingRVV` is a Scala Boolean of `BoomCoreParams`: this module is instantiated
  only inside VecIssueSlot, itself elaborated only in the three IQ_V_* queues of
  a `usingRVV` build. In a vectors-off build it is ABSENT, not tied off, so the
  RTL stays bit-identical to pre-Caracal BOOM v4. No hardware enable input here,
  now or ever. The gate is `usingRVV`, not rocket's `usingVector`.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel default and hierarchy.yaml's `defaults`:
  single `core_clk` domain, posedge-triggered, with ACTIVE-HIGH SYNCHRONOUS
  `core_reset`. Both are implicit; no explicit clock or reset port is declared.

  Inputs from the enclosing slot:

  `load` — Bool, write this matcher's state this cycle. Asserted for BOTH reasons
  a slot's occupant changes: a dispatch into an empty slot, and a collapse move
  shifting the slot above down into this one. One port for both because BOOM's
  age-ordered collapsing queue treats them identically (`io.in_uop.valid` in the
  scalar IssueSlot); two would invite two subtly different update expressions.

  `in_member_rdy` — `Vec(groupMembers, Bool)`, the per-member state to load: on a
  dispatch, the INVERSE of the vector Busy-Table's per-member source read for this
  operand; on a collapse move, the upstream slot's `out_member_rdy` for the same
  operand index.

  `prns` — `Vec(groupMembers, UInt(vecPregSz.W))`, this operand's member PRNs:
  `pvs1`, `pvs2` or `pvs3` of the resident uop, `stale_pvdest` on the `rdy_vold`
  instance, or `pvm` in lane 0 when `isMask`. `members` —
  `UInt((log2Ceil(maxMembers) + 1).W)`, its EMUL as a 1..8 count (`v_emul` of the
  resident uop), not elaborated when `isMask`. `used` — Bool, does this operand
  participate in readiness this cycle; the slot owns the decision, this module
  owns the consequence (see the logic section).

  // `stale_pvdest` is a GROUP, `Vec(maxMembers, UInt(vecPregSz.W))` on the uop,
  // and it reaches `rdy_vold`'s `prns` port exactly like a source group. Its
  // `members` is the same `v_emul`: the stale mapping covers the same arch vregs
  // the destination does, so it has the same member count by construction.

  ===> `prns`, `members` and `used` must be the values of the uop RESIDENT AT THE
       END OF THIS CYCLE — the slot drives them from its `in_uop` payload while
       `load` is high and from its `slot_uop` register otherwise. BOOM's scalar
       path instead pre-corrects `dis_uops.prs*_busy` against the wakeup ports
       inside IssueUnit; doing it at this port keeps the correction in one place
       instead of duplicating it in the unit's dispatch AND collapse paths.

  `group_done` — `Vec(numVecWbPorts, Valid(new VecGroupDone))` (VecBundles owns
  the bundle). Each valid port carries the completing group's full member-PRN
  vector, `Vec(maxMembers, UInt(vecPregSz.W))`, plus its `members` count.
  Broadcast, unregistered, identical at every slot and instance.

  Outputs — `ready`, a Bool: this operand's group-ready bit, one term of the
  slot's `vector_operands_ready`, combinational and valid in the same cycle as
  the group-done that completes the group. And `out_member_rdy`,
  `Vec(groupMembers, Bool)`: the NEXT-STATE per-member vector, exported so a
  collapse move carries partial readiness to the slot below. Next state and not
  the register value on purpose — a group-done landing in the cycle of the move
  would otherwise be lost by both slots.

  ===> THERE IS NO `MicroOp` PORT, and the entry's `depends_on:` omits MicroOp to
       enforce it. A MicroOp port would be a wide bundle replicated up to five
       times per slot in all three queues, and it would let a future edit resolve
       `is_shared` or `vm` here instead of once in the slot.
  <|end_ports|>

  <|begin_logic|>
  ---- What is held, and where ----

  //@req-spec-issue.g13
  The operand's member PRNs — up to EMUL of them, `Vec(maxMembers)` wide with
  `v_emul` valid — are held in the enclosing slot's `slot_uop` register and READ
  COMBINATIONALLY through the `prns` port; this module keeps no copy. A copy
  would add `maxMembers * vecPregSz` flops per operand, up to five times per slot,
  duplicating state the slot already holds and only the slot can update on a
  collapse move. The `rdy_vold` instance reads `stale_pvdest` through the same port
  for the same reason — the stale group lives in `slot_uop` too.

  //@req-spec-issue.g14
  What this module DOES hold is one register bit per member, `member_rdy(i)`:
  each member PRN has its own readiness bit rather than the group sharing one.
  Only these `groupMembers` bits are state; the rest is combinational.

  // The uop carries one AGGREGATED busy bit per operand (`pvs1_busy` and
  // friends) and no per-member vector, by MicroOp's design. That aggregate
  // cannot initialize these bits: a group whose members come from two producers
  // can have members 0..2 complete while member 3 is in flight, and the
  // aggregate then says only "not ready". Broadcasting it to all members would
  // make this matcher wait on group-dones for 0..2 that already fired and will
  // never fire again — a permanent hang. Hence the per-member `in_member_rdy`
  // port, driven from the same Busy-Table read the aggregate is derived from.

  ---- The per-member match ----

  //@req-spec-rename.g19
  //@req-spec-issue.g12
  //@req-spec-vrf.e8
  The only wakeup this module listens to is the VECTOR network: `pvs1`, `pvs2`,
  `pvs3` and `pvm` are matched here against group-done events and never against
  the integer or FP networks. The network carries a single completion event per
  group, yet the readiness it produces stays PER MEMBER — the event's member-PRN
  vector is exactly what makes that possible.

  //@req-spec-issue.g15
  //@req-spec-issue.g18
  For each member lane `i` of this operand and each wakeup port `w`, compare
  `prns(i)` against EVERY member slot `j` of `group_done(w).bits.prns`, and take
  the hit when the port is valid and `j` is within that port's `members` count:

    member_hit(i) := OR over w, j of
                     ( group_done(w).valid &&
                       (j.U < group_done(w).bits.members) &&
                       (group_done(w).bits.prns(j) === prns(i)) )

  // A single base comparator — this operand's member 0 against the group-done's
  // member 0 — cannot be substituted. An LMUL=8 write to v0..v7 followed by an
  // LMUL=2 read at v4 sources {p4, p5}; the producer's base is p0, so base
  // equality never fires and the consumer hangs. The per-member match is the
  // area and timing cost of the vector slot and it is not removable.

  Comparison is on the full `vecPregSz` bits of the PRN — never against a group
  base, a member index or an architectural register number.

  Lanes `i >= members` hold DON'T-CARE PRNs — vector rename writes only the
  group's first EMUL entries — so treat lane `i` as ready whenever
  `i.U >= members` and ignore its matches. Both failure directions are real: a
  stale PRN in an unused lane that never completes hangs the operand forever, and
  one that happens to equal a live PRN completing elsewhere wakes the operand
  EARLY, reading a register its producer has not written. With `isMask` there is
  one lane and this term elaborates away.

  ---- AND-reduce to one bit ----

  //@req-spec-rename.g13
  //@req-spec-issue.g16
  The per-member bits are AND-ed into ONE group-ready bit for this operand. No
  partial-readiness output and no per-member signal leaves this module towards
  `request`; the slot sees one bit per operand.

  //@req-spec-rename.g14
  //@req-spec-issue.g17
  Because the reduction is an AND over all valid members, the operand wakes only
  when its LAST member becomes ready: the group-ready bit rises in the cycle the
  final outstanding member matches, not before — and that holds even when several
  members complete in the same cycle on different ports.

  The next state and both outputs are one expression, so a load, a collapse move
  and a match landing in the same cycle compose:

    member_rdy_next(i) := (load ? in_member_rdy(i) : member_rdy(i)) ||
                          member_hit(i)
    member_rdy(i)      <= member_rdy_next(i)               // posedge core_clk
    out_member_rdy(i)  := member_rdy_next(i)
    group_all_rdy      := AND over i of
                          ( member_rdy_next(i) || (i.U >= members) )
    ready              := !used || group_all_rdy

  // The OR of member_hit into the LOADED value is the load-bearing part. A
  // group-done fires exactly once, so applying the match only to the already-
  // registered state would drop an event arriving in a dispatch or collapse-move
  // cycle and the consumer would never issue. Hence `ready` and `out_member_rdy`
  // come from the next state, not from the register.

  A member's ready bit is STICKY: once set, only a `load` overwrites it. There is
  no re-busy path.

  ---- Sub-range and fragmented source groups ----

  //@req-spec-rename.g20
  A consumer reading a sub-range of a producer's in-flight group wakes when that
  producer's group-done fires, because each of the consumer's members appears in
  that event's member-PRN vector and matches its own lane. The same mechanism
  covers a group fragmented across two producers with no extra logic: each lane
  latches its own hit from whichever port and cycle carries it, and the
  AND-reduce waits for the later. This is conservative but correct — the consumer
  may wake LATER than strictly necessary (it waits on the whole producing group,
  not only the members it reads), never earlier.

  // ===> THIS IS ALSO WHY A SINGLE AGGREGATE `stale_pvdest_busy` BIT WAS REJECTED
  // (D6) AND A FIFTH INSTANCE OF THIS MODULE ACCEPTED INSTEAD, and it is the
  // tempting wrong answer, so the reasoning is recorded where the mechanism lives.
  // A `stale_pvdest` group can span UP TO EIGHT PRODUCERS: an `LMUL=1` op writes
  // `v0`, then an `LMUL=8` op renames `v0..v7`, so the younger op's stale mapping
  // is the current mappings of eight arch vregs installed by up to eight different
  // instructions. One bit cannot express "waiting on producer 3 of 8", and one
  // group-done cannot clear it correctly — clearing it on the first arrival wakes
  // the consumer EARLY (it reads a register a later producer has not written),
  // while requiring all eight is unrepresentable in one bit. This is the same
  // argument that forces per-member matching for `pvs*` (rename.g20) and it
  // applies unchanged to the stale group.

  ---- Conditional participation: the mask, and the deselected operand ----

  `used` low forces `ready` high regardless of every member bit. That is what
  makes the mask instance correct: `pvm`'s busy bit participates only when the
  OP.v is actually masked (encoded `vm` bit clear); for an unmasked op `pvm` is
  DON'T-CARE — it names whatever physical register V0 was last mapped to, perhaps
  one never written again, so waiting on it would hang. The slot drives `used`
  from the resident uop for `rdy_vm`, uses the same port on `rdy_vs3` to drop that
  operand from the LSU half's readiness when `is_shared` is set, uses it on
  `rdy_vs1`/`rdy_vs2`/`rdy_vs3` to drop a source the instruction does not encode
  (`v_uses_vs1`/`v_uses_vs2`/`v_uses_vs3`, decision D11), and uses it on
  `rdy_vold` to drop the stale group when the OP.v has no vector destination. This
  module decodes none of those conditions — it is given the bit — but must honour
  it by GATING, never by freezing the state: the member bits keep updating while
  `used` is low, so an operand that becomes used later cannot have missed an event.

  ---- No speculative wakeup, therefore no re-busy ----

  //@req-spec-vrf.e7
  //@req-spec-vrf.e9
  Every input that can set a member bit is an ACTUAL COMPLETION — a group-done
  from a real writeback. Vector operands are never woken speculatively: a
  vector-load producer completes through the LCB after a long, variable,
  streaming latency and a vector-arithmetic producer completes over the CII in
  program order, so neither has the fixed short load-use latency that makes
  speculation profitable. Waking on real writeback is also what lets this module
  own no re-busy or replay machinery at all.

  // REJECT LIST for this port set: no speculative-wakeup input, no load-hit or
  // load-miss input, no re-busy / clear-ready input, no brupdate or flush input
  // (the slot's valid bit and the queue's compaction handle a squash, and every
  // new occupant arrives through `load`, which overwrites all member bits), and
  // no `busy` output of any kind — the last is the vector-LSU invariant seen
  // from the issue side.

  ---- The match-port budget ----

  //@req-spec-issue.g33
  Each member PRN is matched against ALL `numVecWbPorts` group-done ports EVERY
  cycle, each port carrying up to `maxMembers` PRNs. Matching fewer ports, or one
  port per cycle in rotation, is not an option: the network is `numVecWbPorts`
  wide and completion is single-shot, so a port not examined in the cycle it is
  valid is a lost wakeup. The cost is `EMUL * numVecWbPorts * maxMembers`
  `vecPregSz`-wide equality comparators per instance — 64 per wakeup port per
  group at EMUL = maxMembers = 8, hence 192 at `numVecWbPorts` = 3. Multiplied by
  FIVE operand instances per load/ALU slot (four in a store slot),
  `vecIssueEntries` slots and three IQ_V_* queues, this is the dominant area term
  of the vector issue stage and it sits in the wakeup-to-grant critical path.

  // The fifth instance (`rdy_vold`, D6) is +1 matcher x (16 IQ_V_LOAD + 16
  // IQ_V_ALU) slots = 32 added instances, i.e. +6144 comparators, and it lands in
  // the stage that is ALREADY this design's #1 timing risk. That cost was accepted
  // deliberately against a silent-hang/silent-corruption alternative; the
  // mitigation if the path fails is the shared one-hot decode below, which the
  // fifth instance shares with the other four at no extra decode cost.

  // PERMITTED FACTORING, semantically identical, and the escape valve if that
  // path fails timing: OR the group-done ports' member PRNs into one
  // numVecPhysRegisters-wide one-hot "completing this cycle" vector shared by
  // every instance in a queue, and read member_hit(i) as an indexed lookup of it
  // at prns(i). Same function — still per member against every port — trading
  // the comparator array for one decode plus a wide mux per member. It must stay
  // COMBINATIONAL within the group-done cycle; registering the shared vector adds
  // a cycle to every dependent vector op and loses the same-cycle dispatch match.

  ---- pvl is deliberately not matched here ----

  //@req-spec-issue.g34
  //@req-spec-vrf.e10
  The per-slot match budget is these instances — five in a load/ALU slot, four in
  a store slot — PLUS the VL wakeup network for `pvl`, which is NOT an instance of
  this module but a plain equality comparator per VL lane in the enclosing slot's
  scalar-feeder set. `pvl` is one register in its own register space with exactly
  one busy bit, so an instance here would spend `maxMembers * numVecWbPorts`
  comparators to model one bit, and would connect the wrong network.

  // g34 is KEPT, with its reading annotated: its force is "`pvl` is not a group,
  // so it costs a plain comparator and no matcher instance", not the literal count
  // one. The VL network is now `VectorParams.numVlWakeupPorts` = `aluWidth + 1`
  // lanes (decision D8 replicates the vset writeback per integer ALU rather than
  // arbitrating it, because a single-shot VL wakeup lost to arbitration is a
  // permanent hang), so the slot spends `numVlWakeupPorts` comparators — 3 at
  // Medium, 5 at Mega — still one per lane and still no instance of this module.

  The POLICY is nonetheless this module's and not the INT feeders': `pvl` wakes on ACTUAL
  COMPLETION on its own VL network, not on the integer network and not
  speculatively. No VL producer suits speculation (`vsetivli`'s `pvl` is born
  ready, `vsetvli`/`vsetvl` have deterministic ALU latency, `vleff` publishes its
  trimmed VL only after a long variable load stream), so the VL network needs no
  re-busy machinery either.

  ---- Reset, assertions and tracing ----

  On synchronous active-high `reset`, initialize every `member_rdy` bit to READY.
  The value is functionally irrelevant — `ready` is consumed only while the slot
  is valid and a valid slot always arrived through `load` — and ready-at-reset is
  chosen so an idle slot shows no phantom stall at time 0. Assert that `members`
  is 1..`maxMembers` whenever the operand is used, that
  `group_done(w).bits.members` is likewise in range on every valid port, and that
  an `isMask` instance has one member lane: checks, not behaviour.

  This module emits NO trace line of its own, deliberately: VecTrace's helper
  requires a `MicroOp` so every line carries `rob_idx`, and adding one here for
  debug would put a wide port on a module instantiated four or five times per slot
  in all three queues. VecIssueSlot traces instead, holding the uop — it watches the
  rising edge of each instance's `ready` and emits one guarded line naming the
  operand and the exported `out_member_rdy`, so the last member to complete the
  group is visible. Same convention, same plusarg gate, off by default.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
PURELY COMBINATIONAL FROM GROUP-DONE TO `ready`, ZERO CYCLES. A group-done in
cycle N must be able to produce a grant in cycle N — the slot's `request`, the
queue's select and the grant all happen in that cycle. A pipeline register
between `group_done` and `ready` would add a cycle to EVERY dependent vector op
and would break the same-cycle dispatch and collapse-move captures. There is
exactly one register stage here, `member_rdy`, and it is on the state, not the
output. No ready/valid handshake and no back-pressure in either direction:
`load` is a write enable and `group_done` is a broadcast this module must never
be able to refuse, because completion is single-shot.

Critical path at the 1 GHz `core_clk` target: group-done PRN fan-out to
`groupMembers * numVecWbPorts * maxMembers` comparators, an OR-tree per member,
the AND-reduce, then the slot's operand AND, the queue's priority encoder and the
grant. This IS the vector issue stage's critical path. The shared one-hot decode
in the logic section is the intended mitigation and changes no specified
behaviour; matching fewer ports, rotating ports, or registering the match are not.

Area at the defaults (`maxMembers` 8, `vecPregSz` 7, `numVecWbPorts` 3 from
`VectorParams`, `vecIssueEntries` 16): 192 comparators per non-mask instance and
24 for the mask, so 792 per IQ_V_LOAD / IQ_V_ALU slot (four non-mask instances
including `rdy_vold`, plus the mask) and 600 per IQ_V_STORE slot — about 34.9k
over the three IQ_V_* queues — plus 33 flops of member state per load/ALU slot and
25 per store slot. Stated so that a config change multiplying them is visible at
review rather than at synthesis.

// The delta from the four-instance version is the D6 `rdy_vold` instance: +192
// comparators and +8 flops on each of the 16 IQ_V_LOAD and 16 IQ_V_ALU slots, so
// +6.1k comparators and +256 flops, about +21% on this stage's dominant area
// term. It buys removal of a silent read of a BUSY stale group by the LCB's R2
// pre-load and by the CII's STALE_VD pull.
<|end_perf|>

<|begin_dependencies|>
VecBundles — `VecGroupDone` is this module's input contract: its member-PRN
`Vec(maxMembers, UInt(vecPregSz.W))` plus the `members` count is exactly what the
per-member match consumes. Only those two fields are read here; the event's
`rob_idx` and `pvl` belong to the ROB busy-clear and the VL network and must not
be examined in this module.

VectorParams — `maxMembers`, `vecPregSz` and now `numVecWbPorts`. That field is
DECLARED THERE (3), so this module's constructor parameter BINDS to it and
re-defaults nothing; the same field also settles `numVecClrPorts` (3) and
`numVlWakeupPorts` (`aluWidth + 1`), the latter sizing the VL comparator set the
enclosing slot owns.

VecTrace — bound by the enclosing slot rather than called here (logic section).

Instantiated BY VecIssueSlot FIVE times in an IQ_V_LOAD or IQ_V_ALU slot and FOUR
times in an IQ_V_STORE slot — `rdy_vs1`/`rdy_vs2`/`rdy_vs3` and `rdy_vold`
(load/ALU only) with `isMask = false`, `rdy_vm` with `isMask = true`; instantiates
nothing itself. VecPipeline drives the wakeup ports from the group-done producers —
VecLoadCoalescingBuffer, the CII writeback completion path and VecGroupCopy — and
the per-member load values come from the vector Busy-Table source read in
VecRenameSpace, which is being amended to export a FIFTH group (the `stale_pvdest`
member vector) for `rdy_vold`'s `in_member_rdy`.
<|end_dependencies|>

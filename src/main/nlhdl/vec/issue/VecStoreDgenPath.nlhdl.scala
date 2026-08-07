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
  VecStoreDgenPath — the vector STORE slot's second grant path: the AGEN/DGEN
  path-sequencing rule plus the `is_shared` store-data operand mux.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/issue/VecStoreDgenPath.scala,
  package boom.v4.vec.generated.issue. depends_on MicroOp, VecBundles, VecTrace.
  Instantiated by VecIssueSlot as `dgen_path` with
  `count: isStoreSlot ? 1 : 0` — it exists ONLY in `iq_v_store` slots. That is
  the reason it is a separate node: it is the one part of the vector slot with
  store-only sequencing state, and folding it back in would put that state in
  every load and ALU slot.

  ===> THE DGEN GATED OPERAND IS SELECTED BY `is_shared`, AND GETTING THIS WRONG
       IS ONE OF THE MILESTONE-1 BUGS. The gated operand is `pvtmp` when
       `is_shared` is set and `pvs3` otherwise. Gating DGEN unconditionally on
       `pvs3` reads the wrong group for a segmented store and DEADLOCKS the
       six-step chain, because for such an op `pvs3` is the COPROCESSOR half's
       source group — ready long before the transpose has run — while the group
       the LSU half must read, `pvtmp`, is written *by* that coprocessor half.

  ===> THE TWO PATHS ARE NEVER ONE GRANT. AGEN is granted first, DGEN second,
       and the gap between them is unbounded: for a segmented store DGEN cannot
       fire until the coprocessor half has written `pvtmp`, which is step 6 of a
       chain whose step 1 is this same slot's AGEN.

  This node ADDS NO NEW MECHANISM. It extends BOOM v4's existing mem-slot
  AGEN/DGEN split (src/main/scala/v4/exu/issue-units/issue-slot.scala, the
  `isMem` block) and reuses its state encoding and its names —
  `iw_issued_partial_agen` / `iw_issued_partial_dgen` already exist in
  `MicroOp`, and the durable "which path is still outstanding" state is already
  the `fu_code(FC_AGEN)`/`fu_code(FC_DGEN)` pair. Only the gating operand and
  the offer ORDER change.

  Governing spec anchors: issue.rst `shared-store-chain`,
  issue.rst `issue-sched-stage` ("The Vector Issue Slot"),
  issue.rst `cii-shared-sched` ("Segmented Store"), cii.rst `cii-segmented`,
  midcore.rst `midcore-segmented-store` and `vrf-ports`,
  glossary.rst `glossary-terms` ("Scheduler").
*/

<|begin_module|>

  <|begin_parameters|>
  No constructor parameters, and deliberately not even an `isStoreSlot` one. A
  module that could be parameterized into doing nothing invites being
  instantiated in a load or ALU slot with the parameter false; instead the
  instance count is zero there, so a misplacement is a missing instance rather
  than a silently inert one.

  Elaborated only when `usingRVV` is true — a Scala `Boolean` from
  `BoomCoreParams`, not a hardware `Bool`. With vectors off this module does not
  exist and is not tied off, so a non-vector build stays bit-identical to
  pre-Caracal BOOM v4.

  Every width is taken from the corresponding `MicroOp` field's own Chisel type
  (`chiselTypeOf(uop.pvs3)` for the operand group). No width is a literal and
  none is re-derived from `maxMembers` or `vecPregSz` here: the group this module
  muxes IS a `MicroOp` field, so deriving the port type from the field is what
  keeps the two from desynchronising if the group sizing changes.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel default and the map's defaults: posedge
  `core_clk`, ACTIVE-HIGH SYNCHRONOUS `core_reset`, implicit. This module
  declares no register (see the logic section), so reset reaches nothing inside
  it; the convention is stated because the enclosing slot's state, which this
  module's outputs drive, obeys it.

  Directions are from this module's point of view. The single peer is the
  enclosing VecIssueSlot instance; nothing else connects to it.

  Inputs from the slot:
    - `slot_valid`, `grant`, `squash_grant` : Bool, the slot's own signals of
      those names, `grant`/`squash_grant` originating in the issue unit.
    - `slot_uop` : MicroOp, the slot's registered `slot_uop`. Read for
      `fu_code(FC_AGEN)`, `fu_code(FC_DGEN)`, `iw_issued`,
      `iw_issued_partial_agen`, `iw_issued_partial_dgen`, `is_shared`, `pvs3`,
      `pvs3_busy`, `pvtmp`, `pvtmp_busy`, `v_emul`, and `rob_idx` (trace only).
    - `agen_operands_ready` : Bool, the slot's readiness term for the ADDRESS
      path ONLY — base GPR on `prs1` (integer network), `pvl` (VL network), the
      index group for indexed forms, `pvm` when masked. It must not include the
      store-data operand; see part 5 of the logic section.
    - `dgen_operand_ready` : Bool, group-ready for the operand this module
      selected, from the slot's `rdy_vs3` VecGroupReady instance — the instance
      whose member inputs this module's `dgen_operand` output drives.
    - `agen_rebusied` : Bool, the slot's `rebusied_prs1` equivalent: the AGEN
      path's scalar feeder was re-busied by a speculative-wakeup retraction.

  Outputs to the slot, the operand mux:
    - `dgen_operand` : the selected member-PRN vector, typed from
      `slot_uop.pvs3`; drives `rdy_vs3`'s member inputs.
    - `dgen_operand_members` : the selected group's valid member count.
    - `dgen_operand_busy` : the selected group's busy bit — `pvtmp_busy` or
      `pvs3_busy`, never both.
    - `dgen_operand_is_pvtmp` : Bool, the mux select itself, exported so the slot
      can exclude the DESELECTED operand's busy bit from
      `vector_operands_ready` and so the store-data VRF read is checkable
      against it.

  Outputs to the slot, path sequencing (all Bool):
    - `agen_request`, `dgen_request` : offer the address / data path this cycle.
      At most one is ever true; the slot ORs them into its `request`.
    - `iss_fu_code_agen`, `iss_fu_code_dgen` : override the same two bits of
      `iss_uop.fu_code`, naming the path being offered.
    - `next_fu_code_agen`, `next_fu_code_dgen` : drive the same two bits of
      `next_uop.fu_code` — the durable path-remaining encoding.
    - `issued_partial_agen`, `issued_partial_dgen` : drive
      `next_uop.iw_issued_partial_agen` / `_dgen`.
    - `keep_valid` : forces the slot's `next_valid` true while a path is still
      outstanding after a grant.
    - `both_paths_done` : both paths granted; the slot retires the entry on its
      normal `iw_issued` path.

  Deliberately absent: any `VecGroupDone` port or wakeup-port array. The
  per-member match against every group-done port belongs to VecGroupReady, one
  instance per operand in the slot; this module only chooses WHICH group that
  matcher looks at, and adding ports here would duplicate the slot's comparator
  budget for the one operand that already has a matcher. Also absent: any `busy`
  output toward an issue unit and any port reaching the vector LSU — this module
  observes grants and never learns whether the LSU has drained anything.
  <|end_ports|>

  <|begin_logic|>
  This module is COMBINATIONAL. It declares no register and no counter.

  ---- 1. Where the two grant bits live, and why not here ----

  //@req-spec-core.f13
  //@req-spec-issue.d10
  //@req-spec-issue.d13
  The two paths are tracked as a two-bit "still outstanding" encoding held in the
  pair `fu_code(FC_AGEN)` / `fu_code(FC_DGEN)` of the slot's uop, exactly as
  baseline BOOM's mem slot already does it: both bits set means neither path is
  granted yet, the DGEN bit alone means AGEN is granted and DGEN outstanding,
  neither means both are granted and the entry retires, and the AGEN bit alone is
  unreachable for a vector store (part 2).

  Those two bits are the independently-grantable paths. They are two bits and not
  one, and the slot presents ONE of them per grant, so a vector store slot is
  granted twice and never once. `iw_issued_partial_agen` and
  `iw_issued_partial_dgen` are the single-cycle post-grant markers that carry a
  grant into the following cycle's encoding update — the same two `MicroOp`
  fields the scalar mem slot uses, reused rather than duplicated.

  // ===> THE STATE MUST NOT BE A PRIVATE REGISTER IN THIS MODULE. The vector
  // issue queues are BOOM's age-ordered COLLAPSING queues, so a uop migrates
  // between slot instances whenever a vacancy opens below it
  // (issue-unit-age-ordered.scala drives each slot's `in_uop` from the next
  // slot's `out_uop`). A grant bit registered here would be left behind by that
  // shift: the migrated store would arrive reading "neither path granted" and
  // re-issue its AGEN, or reading "both granted" and never issue its DGEN — a
  // duplicate element address pass, or a store that never completes. Because
  // the bits ride the migrating `MicroOp` and this module is combinational,
  // they follow the uop for free.

  ---- 2. Offer order: AGEN strictly before DGEN ----

  //@req-spec-issue.d11
  When both bits are set the module offers the ADDRESS path and only that path:
  `agen_request` is `slot_valid && !iw_issued && fu_code(FC_AGEN) &&
  agen_operands_ready`, and `dgen_request` is held false while the AGEN bit is
  still set, even if the store data is already ready. This is the one behavioural
  divergence from the scalar mem slot, which offers whichever path is ready and
  will happily issue DGEN first. For a vector store that freedom is wrong: the
  data path's element enumeration is paired in order with the address path's
  across `st_*_ADDR_Q` and `st_*_DATA_Q` (loadstore.rst `store-data-queue`), and
  for a segmented store the data cannot exist before the address pass has run at
  all, AGEN being step 1 of the chain that produces `pvtmp`. The AGEN-bit-only
  encoding is therefore unreachable, and is asserted so rather than assumed.

  ---- 3. What DGEN is gated on ----

  //@req-spec-issue.g22
  //@req-spec-issue.g23
  //@req-spec-issue.g24
  `dgen_request` is `slot_valid && !iw_issued && fu_code(FC_DGEN) &&
  !fu_code(FC_AGEN) && !dgen_operand_busy && dgen_operand_ready`. The gating term
  is the VECTOR store-data operand's group-ready bit, matched on the VECTOR wakeup
  network through the slot's `rdy_vs3` matcher. It is NOT `prs2_busy`: the scalar
  mem slot's DGEN waits on `prs2` because its store data is one integer or FP
  register, whereas a vector store's data is a group of up to EMUL member PRNs in
  the VRF, read at execute on VRF read port `R3` (midcore.rst `vrf-ports`,
  canonical — this module adds no VRF port and changes only which group R3
  addresses). `prs2` plays no part here and must not appear in the expression.

  // Baseline's scalar DGEN operand rewrite (`io.iss_uop.prs1 := slot_uop.prs2`)
  // has no counterpart here and must not be re-added: vector store data is not
  // delivered through a scalar operand slot.

  ---- 4. The operand mux — the M1 bug ----

  //@req-spec-issue.g25
  //@req-spec-issue.g26
  The gated operand is selected by `is_shared`:

      dgen_operand := Mux(uop.is_shared, uop.pvtmp, uop.pvs3)

  and the same select drives the companion outputs, so the member count and the
  busy bit always come from the SAME group as the member PRNs:

      dgen_operand_busy    := Mux(uop.is_shared, uop.pvtmp_busy, uop.pvs3_busy)
      dgen_operand_is_pvtmp := uop.is_shared

  //@req-spec-issue.g27
  //@req-spec-issue.g28
  //@req-spec-cii.i6
  /* WARNING — do not "simplify" this to pvs3. Gating DGEN unconditionally on
     pvs3 is incorrect for a segmented store and earlier Caracal drafts did
     exactly that. For such an op pvs3 is the COPROCESSOR half's source group and
     is ready long before the transpose has run, while the LSU half's data source
     is pvtmp, written BY the coprocessor half. A DGEN woken on pvs3 reads pvtmp
     before it exists and stores garbage; and pvs3's group-done never arrives at
     this slot at all, so the chain deadlocks. Whenever is_shared is set, DGEN
     wakes on pvtmp's group-done and on nothing else. */

  Because the mux sits UPSTREAM of the matcher, the exclusion of the deselected
  operand is structural rather than an extra AND term someone can forget: with
  `is_shared` set, `rdy_vs3` matches `pvtmp`'s members, so `pvs3`'s busy bit is
  not merely ignored, it is not in the readiness cone. That discharges the
  slot-side obligation (issue g32) that `pvs3` be excluded from
  `vector_operands_ready` for the LSU half of a shared store. Keeping it in that
  term would happen to work today, purely because it is ready early, and would
  turn a timing accident into a correctness dependence.

  // ===> `pvs3` AND `pvtmp` ARE NOT MERGED AND NOT REINTERPRETED. This module
  // selects between two fields that both stay intact on the issued uop; it never
  // overwrites one with the other. The downstream store-data read therefore has
  // both, plus `is_shared`, and MUST address VRF port R3 with the same selection.
  // Reading `uop.pvs3` unconditionally at the read site reintroduces this bug one
  // stage later, where it looks like a VRF problem rather than an issue problem.

  ---- 5. Which path a `pvtmp` group-done may wake ----

  //@req-spec-issue.c12
  //@req-spec-cii.i12
  For a segmented store, `pvtmp`'s group-done is what makes this slot's DGEN
  path grantable: it clears the selected group's busy bit and raises
  `dgen_operand_ready` through `rdy_vs3`, which is step 2 of the segmented-store
  wakeup list and the only wakeup the store IQ slot receives from the
  coprocessor half.

  //@req-spec-issue.c13
  It must NOT wake the AGEN path, and the port list is what enforces that:
  `agen_request` is a function of `agen_operands_ready` alone and there is no
  connection in this module from `dgen_operand_ready` or `dgen_operand_busy` to
  `agen_request`. The slot must correspondingly keep the store-data operand out
  of `agen_operands_ready`. The chain's ordering makes the mistake
  harmless-looking, which is why it is asserted at the boundary instead: AGEN has
  always already been granted by the time any `pvtmp` group-done can exist.

  ---- 6. Capturing a grant ----

  On `grant && !squash_grant` exactly one partial marker is set, chosen by the
  path offered this cycle (`issued_partial_agen := grant && !squash_grant &&
  agen_request`, and likewise for DGEN). The encoding update happens the
  following cycle off that marker in `slot_uop`, mirroring baseline:

      AGEN marker set, `agen_rebusied` false:
          next_fu_code_agen := false, next_fu_code_dgen := true
          keep_valid        := true          // the slot survives the grant
      AGEN marker set, `agen_rebusied` true:
          encoding left unchanged, so AGEN re-offers next cycle
      DGEN marker set:
          both_paths_done := true; the slot retires normally

  The AGEN side keeps baseline's re-busy term because its scalar feeders DO ride
  BOOM's speculative load-hit wakeup and can be retracted. The DGEN side has no
  re-busy term at all, and that asymmetry is deliberate: vector operands are
  never woken speculatively (midcore.rst `spec-wakeups`), so a DGEN grant can
  never be invalidated by a retraction, and a re-busy term there would be dead
  logic implying a wakeup policy the design does not have.

  ---- 7. Tolerating an unbounded AGEN-to-DGEN gap ----

  //@req-spec-issue.d12
  Nothing here measures, bounds or times out the interval between the two grants.
  There is no counter, no shift register and no watchdog; the DGEN-outstanding
  encoding is level-held in the migrating uop and is equally valid one cycle or
  ten thousand cycles after the AGEN grant. The spec bounds the delay only
  informally ("hundreds of cycles"), so no cycle count appears here as a
  constraint. Two consequences:

  - The slot occupied by a DGEN-pending store is NOT reclaimable. `keep_valid`
    forces the slot valid across the AGEN grant, and the entry is freed only by
    the DGEN grant, a branch kill or a flush. A store waiting for its
    coprocessor half therefore occupies an `IQ_V_STORE` slot for the whole chain
    — real occupancy pressure, and the honest cost of not cracking.
  - This module must not be the reason the wait ends. It cannot poll and has no
    path to the coprocessor or the LSU; it waits for a group-done on the vector
    network like any other consumer.

  Kill, flush and squash need nothing from it either, for the same reason it
  holds no state: the two bits die with the uop in the slot's own registers, and
  element accesses an already-granted AGEN pushed into the store queues are
  reclaimed by tail-pointer rollback (loadstore.rst `vec-squash`), not from here.
  `squash_grant` is honoured on both paths, so a squashed grant advances neither.

  ---- 8. Assertions (synthesizable, on hardware conditions) ----

  - Never both markers: `!(issued_partial_agen && issued_partial_dgen)`.
  - Never both offers: `!(agen_request && dgen_request)`.
  - Never DGEN before AGEN: `!(dgen_request && fu_code(FC_AGEN))`, and
    `!(issued_partial_dgen && fu_code(FC_AGEN))`.
  - `agen_request` implies `agen_operands_ready` (part 5).
  - A uop occupying a store slot advertises both `FC_AGEN` and `FC_DGEN` on
    arrival: `!(slot_valid && !iw_issued && !fu_code(FC_AGEN) &&
    !fu_code(FC_DGEN))`.
  - The selected group's member count is non-zero whenever `dgen_request` is
    asserted, so a DGEN never issues against an empty group.
  - `!(grant && !slot_valid)`.

  ---- 9. Tracing ----

  Guarded tracing via the shared VecTrace package, one line per key event, each
  tagged with this module's name and `rob_idx` and gated on the `vecTrace`
  plusarg (off by default, so the traced and measured builds are the same
  machine). Three events, no more: `agen_grant`, `dgen_grant`, and
  `dgen_operand_select` on the cycle the slot is filled, carrying `is_shared` and
  which of `pvtmp`/`pvs3` was chosen. The third is what makes the bug in part 4
  visible in a cosim log when the choice is made, rather than hundreds of cycles
  later as wrong store data.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Zero cycles. This module is combinational and sits inside the slot's existing
request/grant loop; it must not add a pipeline stage between `slot_valid` and
`request`, or the vector issue queues would need a second scheduling stage,
which the single-stage design forbids.

It adds NO comparator to the match-port budget. The per-member match for the
store-data operand is the slot's existing `rdy_vs3` VecGroupReady instance; this
module contributes one `maxMembers`-wide PRN mux and a handful of gates in front
of it. That is the reason the mux is placed upstream of the matcher rather than
matching both groups and selecting the results: the latter would double the
`EMUL x numVecWbPorts x MAX_MEMBERS` comparator count for the one operand that
already dominates the slot's area.

The depth added to the request path is one 2:1 mux plus the ordering AND term, so
the slot's critical path is unchanged in character: it remains the group-ready
AND-reduce, not this module.

Throughput target: two grants per store, one per path, with no bubble imposed
between them beyond the operand's own readiness — when the store data is already
ready at the AGEN grant, the common NON-shared case, DGEN must be grantable on
the very next cycle. The strict ordering rule must not cost a cycle beyond that.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the whole interface is expressed in its fields. `is_shared`, `pvs3`,
`pvs3_busy`, `pvtmp`, `pvtmp_busy` and `v_emul` are the mux inputs, and
`iw_issued`, `iw_issued_partial_agen` and `iw_issued_partial_dgen` are the
sequencing state this module owns the next-state function for. The operand port
types are taken from `chiselTypeOf` on those fields rather than re-derived.

VecTrace — the guarded trace helpers used in part 9 of the logic section.

VecBundles — declared as a dependency in hierarchy.yaml, but no bundle from it
appears in this module's port list: the readiness this module consumes is a
single `Bool` from VecGroupReady, not a `VecGroupDone`, precisely because the
per-member match is not done here. The dependency is nominal and is reported as
such rather than satisfied by inventing a port for it.

Binds to the EXISTING `boom.v4.common` functional-unit codes `FC_AGEN` and
`FC_DGEN`. No new `fu_code` is added for the vector store's data path — the
ScalarOpConstants delta adds none, and the whole point of this node is that the
baseline pair is reused.

Instantiates nothing. Its only instantiator is VecIssueSlot, as `dgen_path`,
with count zero outside `iq_v_store`.
<|end_dependencies|>

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
  VecIssueSlot — one entry of a vector issue queue: a SUPERSET of BOOM v4's
  `IssueSlot` that tracks BOTH operand classes, scalar feeders and vector source
  groups, and asserts `request` only when every one of them is ready.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/issue/VecIssueSlot.scala,
  package boom.v4.vec.generated.issue.
  depends_on MicroOp, VecBundles, VecTrace.
  instantiates VecGroupReady x5 in an `iq_v_load` or `iq_v_alu` slot (`rdy_vs1`,
  `rdy_vs2`, `rdy_vs3`, `rdy_vm`, `rdy_vold`) and x4 in an `iq_v_store` slot
  (`rdy_vold` has `count: isStoreSlot ? 0 : 1`), plus VecStoreDgenPath as
  `dgen_path` with count zero outside `iq_v_store`.

  It is a NEW module and not a delta on `src/main/scala/v4/exu/issue-units/
  issue-slot.scala`, but it deliberately REUSES that file's structure and every
  name a reader already knows — `slot_valid`, `slot_uop`, `next_valid`,
  `next_uop`, `in_uop`/`out_uop`, `will_be_valid`, `iw_issued`, `killed`,
  `rebusied_prs1`/`rebusied_prs2` — so the two can be diffed. The scalar
  `IssueSlot` is left untouched: only the IQ_V_* queues instantiate this module.

  ===> THE TWO OPERAND CLASSES HAVE DIFFERENT WAKEUP POLICIES, AND THAT IS THE
       WHOLE REASON THIS FILE HAS A CHILD MODULE. The scalar feeders ride BOOM's
       existing SPECULATIVE load-hit wakeup unchanged, re-busy machinery
       included; the vector source groups wake ONLY on actual completion. The
       per-member vector match therefore lives in VecGroupReady, which owns no
       speculation input at all, and the speculative comparators stay here.

  ===> THE SLOT MUST NOT CAPTURE THE VL VALUE. `pvl` is matched on the VL
       network as a plain readiness wakeup; the value is read from the VL RF at
       execute. And `vtype` is not an operand at all — it rides the per-uOP
       `VConfig` snapshot and is never woken on any network.

  ===> NOTHING HERE MAY CONSULT A `busy` FROM THE VECTOR LSU (invariant 3). This
       slot learns of progress only through group-done events on the vector
       wakeup network, exactly like every other consumer.

  Governing spec anchors: issue.rst `issue-sched-stage` ("Wakeup Networks",
  "The Vector Issue Slot"), issue.rst `issue-vl-delivery`, issue.rst
  `cii-shared-sched`, midcore.rst `spec-wakeups`, `vl-vtype-rename`,
  `group-done-wb`, `midcore-segmented-load`, overview.rst `caracal-pipeline`,
  glossary.rst `glossary-terms`, frontend.rst `vl-delivery`.

<|begin_module|>

  <|begin_parameters|>
  `iqType` — which of the three vector queues this slot belongs to, passed down
  from the enclosing VecIssueUnit's own `iqType`. It is used only to derive three
  ELABORATION-TIME Scala Booleans, `isLoadSlot`, `isStoreSlot` and `isAluSlot`;
  no hardware ever compares it. Those Booleans decide which optional ports and
  which child instances exist, so a slot in the wrong queue is a missing
  instance or an unconnected port, not a silently mis-behaving mux.

  `pnrGate` — Boolean, default false, true only for `iq_v_alu`. Adds the
  per-entry past-PNR eligibility term and, with it, the `rob_pnr_idx` and
  `rob_head_idx` inputs. Same value the enclosing unit is parameterized with.

  `numIntWakeupPorts` — Int, the width of the INTEGER wakeup network this queue
  listens to. Same value BOOM passes to the scalar `IssueSlot` as
  `numWakeupPorts`; take it from the core's existing wakeup-port count rather
  than declaring a vector-specific number.

  `numFpWakeupPorts` — Int, default 0, and non-zero ONLY when `isAluSlot`. The FP
  network reaches `IQ_V_ALU` alone; `IQ_V_LOAD`/`IQ_V_STORE` get zero ports and
  therefore no FP comparators, because vector memory addressing uses only GPRs.

  `numVecWbPorts` — Int, BOUND TO `VectorParams.numVecWbPorts` (3), forwarded
  verbatim to every VecGroupReady instance. It is the width of the vector wakeup
  network (the LCB, the CII writeback completion, VecGroupCopy).

  ===> SETTLED: `VectorParams` NOW DECLARES IT, so nothing re-defaults it here.
  The same declaration site also owns `numVecClrPorts` (3, the ROB busy-clear
  lane count) and `numVlWakeupPorts` (`aluWidth + 1`, which sizes this slot's VL
  comparator set — see the `vl_wakeup` port). Three files used to carry an
  independent literal 3; they are one field now, and a disagreement would make a
  matcher examine fewer group-done ports than the network drives, miss a
  single-shot completion, and hang the consumer forever.

  `usingRVV` is a Scala `Boolean` of `BoomCoreParams`, not a hardware `Bool`.
  This module is elaborated only inside the three IQ_V_* units of a `usingRVV`
  build; in a vectors-off build it is ABSENT rather than tied off, so the
  emitted RTL stays bit-identical to pre-Caracal BOOM v4. The gate is `usingRVV`
  and never rocket's `usingVector`.

  All widths come from `MicroOp`'s own field types and from VectorParams
  (`vecPregSz`, `vlPregSz`, `maxMembers`, `numVecWbPorts`, `numVecClrPorts`,
  `numVlWakeupPorts`) through `HasVectorParams`. No literal.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel default and hierarchy.yaml's `defaults`: one
  `core_clk` domain, posedge-triggered, with ACTIVE-HIGH SYNCHRONOUS
  `core_reset`. Both implicit; no explicit clock or reset port.

  The port list is baseline `IssueSlotIO` plus the vector additions. Baseline,
  unchanged in name and meaning: `valid`, `will_be_valid`, `request`, `grant`,
  `squash_grant`, `iss_uop`, `in_uop` (a `Valid(new MicroOp())` — if valid it
  WILL overwrite this entry), `out_uop`, `brupdate`, `kill`, `clear`, and
  `child_rebusys`.

  //@req-spec-issue.g5
  //@req-spec-vrf.e3
  `int_wakeup_ports` — `Flipped(Vec(numIntWakeupPorts, Valid(new Wakeup)))`, the
  existing BOOM integer wakeup network, carrying `bits.uop.pdst`, `bits.rebusy`,
  `bits.bypassable` and `bits.speculative_mask` exactly as the scalar slot
  consumes them. All three vector queues connect to it: the base address, the
  stride and the `.vx` integer operand are GPR-sourced.

  //@req-spec-issue.g9
  //@req-spec-issue.g10
  `fp_wakeup_ports` — the same `Wakeup` bundle on the FP network, elaborated ONLY
  when `isAluSlot`. A `.vf`-form vector floating-point op or `vfmv.*.f` sources
  one scalar FP register and is matched here; a load or store slot has no such
  port and pays for no FP comparator.

  //@req-spec-rename.h16
  //@req-spec-issue.g6
  `vl_wakeup` — `Flipped(Vec(numVlWakeupPorts, Valid(UInt(vlPregSz.W))))`, the
  dedicated VL wakeup network: `VectorParams.numVlWakeupPorts` = `aluWidth + 1`
  lanes, each carrying a bare VL physical register number. All three queues
  connect to it. Each lane is shaped after baseline's `pred_wakeup_port`, which is
  also a `Valid(index)` matched against one uop field, because that is exactly the
  pattern needed — a readiness event with no payload.

  It was ONE lane before decision D8. The vset writeback is REPLICATED per
  integer ALU rather than arbitrated (`aluWidth` lanes) plus one lane for
  `vleff`'s trimmed VL, because a single-shot VL wakeup lost to arbitration is a
  permanent hang. So the slot's VL match is `numVlWakeupPorts` comparators OR-ed
  together — 3 at Medium, 5 at Mega — not one. Do not size it from a literal and
  do not re-derive the count here.

  `vec_group_done` — `Flipped(Vec(numVecWbPorts, Valid(new VecGroupDone)))` from
  VecBundles, broadcast to every VecGroupReady instance and read nowhere else in
  this module. This slot never inspects a group-done itself.

  `in_member_rdy` / `out_member_rdy` — the per-member readiness SIDE CHANNEL,
  carrying one entry per group this slot's matchers can point at: `vs1_rdy`,
  `vs2_rdy`, `vs3_rdy`, `vtmp_rdy` and `vold_rdy`, each `Vec(maxMembers, Bool)`,
  plus `vm_rdy`, a single `Bool`. `in_member_rdy` is an Input qualified by the same
  `in_uop.valid`; `out_member_rdy` is an Output carrying the instances'
  NEXT-STATE vectors.

  ===> THE BUNDLE IS `VecMemberRdy`, ONE DECLARATION, AND IT GAINED `vold_rdy`
  WITH DECISION D6. Three shapes have existed: VecIssueSlot's original four
  groups (vs1/vs2/vs3 + vm), VecRenameSpace's five (adding `vtmp_rdy`, which
  wins because the third-source matcher tracks `pvtmp` whenever the
  direction-qualified select of part 7 fires and the aggregate cannot supply
  that group's per-member state), and now the D6 shape with `vold_rdy` for
  `stale_pvdest`. GROUPS CARRIED AND MATCHER INSTANCES ARE NOT THE SAME COUNT:
  six fields feed five matchers, because `rdy_vs3` selects between `vs3_rdy` and
  `vtmp_rdy`. `VecSlotMemberRdy` is the same bundle under a second name and is a
  defect, not a synonym — bind to the single `VecMemberRdy` in `VecBundles`.
  On the OUTPUT side each matcher writes its next state into the FIELD IT
  SELECTED (`vtmp_rdy` when the part-7 select took `pvtmp`, `vs3_rdy` otherwise),
  so the receiving slot's identical select reads it back after a collapse move.
  `vold_rdy` is never part of a select — `rdy_vold` always points at
  `stale_pvdest` — so it passes through unmuxed. A STORE slot still receives the
  whole bundle (one declaration, no per-queue variant) and simply leaves
  `vold_rdy` unread, driving it through to `out_member_rdy` unchanged so the
  collapse move stays one mux over one wire.

  ===> THIS CHANNEL MAY NOT BE FOLDED INTO `out_uop.pvs*_busy`. The uop carries
       one AGGREGATED busy bit per operand by MicroOp's design, and a group whose
       members come from two producers can have members 0..2 done while member 3
       is in flight. Collapsing the vector to the aggregate on a collapse move
       makes the destination slot wait again for group-dones that already fired
       and will never fire again — a permanent hang. The channel therefore
       travels beside the uop, through the same collapse move, and VecRenameSpace
       exports the dispatch-time values from the per-member busy-table read it
       already performs.

  `rob_pnr_idx` and `rob_head_idx` — `UInt(robAddrSz.W)`, elaborated only when
  `pnrGate`. Both are needed: BOOM's `IsOlder(a, b, head)` takes the ROB head to
  resolve wraparound, so the two-argument `is_older(rob_idx, rob_pnr_idx)` of the
  spec is realized as `IsOlder(slot_uop.rob_idx, rob_pnr_idx, rob_head_idx)`.

  `eligible` — Output `Bool`, the per-entry selection input the issue unit's
  priority encoder uses. It is `request` on a non-`pnrGate` slot and
  `request && IsOlder(...)` on an `IQ_V_ALU` slot. `request` itself keeps the
  spec's exact meaning and is exported unqualified so that the two obligations
  stay separable at review.

  //@req-spec-issue.f15
  Deliberately ABSENT: `pred_wakeup_port` and any `ppred` term — a vector OP.v is
  never an SFB shadow, and the omission is asserted rather than assumed. Also
  absent: any speculative-wakeup or re-busy port on the VECTOR side, any `busy`
  input from VecLsu or from any vector datapath, any `vtype` operand or `vtype`
  wakeup port, and any VL VALUE input. The converse also holds and is what
  issue.f15 asks for: this module is the only slot with vector match ports, and
  BOOM's scalar `IssueSlot` gains none of them — the scalar queues keep their
  exact baseline port list.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. The frame: baseline's slot, unchanged ----

  //@req-spec-core.e13
  //@req-spec-issue.g1
  //@req-spec-issue.g3
  The state is baseline's and nothing more: `slot_valid` (`RegInit(false.B)`) and
  `slot_uop` (`Reg(new MicroOp())`), with `next_valid`,
  `next_uop := UpdateBrMask(io.brupdate, slot_uop)`,
  `killed := IsKilledByBranch(io.brupdate, io.kill, slot_uop)`, baseline's
  four-way priority on the valid bit (`kill`, then `in_uop.valid`, then `clear`,
  else `next_valid && !killed`), its uop-register load with the
  `assert(!slot_valid || io.clear || io.kill)` overwrite check, and
  `io.out_uop := next_uop` / `io.will_be_valid := next_valid && !killed` so the
  enclosing age-ordered COLLAPSING queue performs the collapse move exactly as it
  does for a scalar slot. This is the whole of the widening: the queue SET grows
  by three and operand-readiness tracking grows to cover vector physical
  registers. No new pipeline stage, no second scheduling stage, no second select.

  //@req-spec-issue.f14
  The vector additions cannot collide with the scalar ones because they are
  matched on DIFFERENT networks against DIFFERENT physical numbering spaces:
  `prs1`/`prs2` against integer `pdst`s, the FP scalar against FP `pdst`s, `pvl`
  against VL PRNs on `vl_wakeup`, and `pvs*`/`pvm` against VRF PRNs inside the
  VecGroupReady instances. This is the same partitioning BOOM already relies on
  for int versus FP, extended by two networks rather than replaced.

  ---- 2. Scalar feeders: baseline comparators, verbatim ----

  //@req-spec-issue.g4
  The base address, the stride and any `.vx` integer operand REUSE the existing
  `prs1` and `prs2` operand slots — no new operand field is added to `MicroOp` for
  them. A unit-stride or indexed access uses `prs1` for the base and leaves
  `prs2` unused; a strided access uses both. The `prs1_matches`/`prs2_matches`,
  `prs1_wakeups`/`prs2_wakeups`, `prs1_rebusys`/`prs2_rebusys`, `bypassables` and
  `speculative_masks` expressions are copied from the scalar slot unchanged,
  including the `next_uop.iw_p1_bypass_hint` / `iw_p1_speculative_child` updates.

  `iw_p1_bypass_hint` / `iw_p2_bypass_hint` ARE A CONTRACT, NOT DEAD BAGGAGE —
  NAME THE READER. An earlier revision of this paragraph asked for the hint to
  be copied "verbatim" and named no consumer, and for a long time it HAD none:
  the field was carried through every vector slot and read by nothing. That is
  how the stale-scalar-base hole survived review — the bit that says "this
  operand is not written back yet" was present, correct, and ignored.
  THE HINT IS NOT A READINESS SIGNAL FOR A CONSUMER WITH NO BYPASS PORT. That is
  the whole content of this paragraph. A scalar EU treats a bypassable wakeup as
  "ready" because it has a bypass network to read the value from at RRD; the
  vector memory path has none — it has a register file read and a writeback
  snoop — so for it a set hint means precisely the opposite: THE VALUE IS NOT
  READABLE YET. On `iq_v_load` and `iq_v_store`, which are NOT past-PNR gated
  (VecPipeline part 3), that state is reachable.

  THE OBLIGATION IS NOT ON THIS MODULE, AND THE GRANT IS NOT DELAYED. It belongs
  to `VecScalarOperandRead`: a set hint means the producer's integer regfile
  write **has not yet landed**. `VecScalarOperandRead` therefore holds that lane
  until it observes the write on `int_wb_snoop`, rather than assuming any
  particular distance. The distance happens to be uniform across bypassable
  producers and that uniformity is what makes a one-cycle arm sufficient — but
  the reasoning for it lives in `VecScalarOperandRead.nlhdl`, not here, and this
  slot must not encode a number. **The slot's only obligation is to deliver the
  hint truthfully on `iss_uop`.** A wrong hint is no longer a lost optimisation;
  it is a wrong address or a stalled lane.

  **`bypass_hint` may be used as a "is the write pending?" predicate, never as a
  latency constant.**
  ===> OPEN DEFECT (named, NOT fixed here): THE `prs2` WAKEUP AND REBUSY ARMS
  ARE GATED DIFFERENTLY, SO SET AND CLEAR DISAGREE ABOUT WHAT THEY COVER.

  Read the four arms together, because no single one of them looks wrong:

    | arm                                   | line  | gated on `lrs*_rtype === RT_FIX`? |
    |---------------------------------------|-------|-----------------------------------|
    | `prs1` wakeup  (set ready)            | :164  | YES                               |
    | `prs1` rebusy  (retract)              | :173  | YES                               |
    | `prs2` wakeup  (set ready)            | :178  | **NO**                            |
    | `prs2` `iw_p2_speculative_child` set  | :180  | **NO**                            |
    | `prs2` rebusy + child clear (retract) | :183  | YES                               |

  The `prs2` group is INCONSISTENT WITH ITSELF: two ungated SET arms feeding one
  gated CLEAR arm. A state that can be set and never cleared.

  THE `:180` ARM IS THE MORE SERIOUS HALF, AND IT IS THE ONE TO FIX FIRST.
  `iw_p2_speculative_child`'s consumer is the RETRACTION PREDICATE at `:183`
  (`io.child_rebusys & slot_uop.iw_p2_speculative_child`). Set ungated at `:180`
  and cleared only through the gated path, a slot whose `prs2` is architecturally
  ABSENT can accumulate a speculative-child mask that nothing clears — and is
  then RE-BUSIED REPEATEDLY by unrelated child rebusys that have nothing to do
  with any operand it actually reads.

  ⇒ **THE SIGNATURE IS STARVATION / LIVELOCK, NOT SILENT CORRUPTION.** That is
  why nothing has caught it, and it is why it will eventually appear as an
  unexplained HANG in some future test rather than as a data mismatch — the same
  "fires ten cycles downstream of the error, in the wrong module" pattern as
  every other defect in this campaign.

  WHICH DIRECTION IS CORRECT: gate **both `:178` and `:180`** on
  `lrs2_rtype === RT_FIX`, matching `:183`. This is RESTORING AN EXISTING
  PRECEDENT, not inventing a rule — the `prs1` pair already has set (`:164`) and
  clear (`:173`) both gated, and that is the evidence that gating is the intended
  discipline. Four arms, one rule.

  WHY IT IS NOT FIXED IN THIS PASS. It lives in the shared scalar-slot logic, so
  changing it perturbs every slot on a regression that has to stay clean for the
  A+B stale-base fix. `VecScalarOperandRead` compensates LOCALLY in the meantime
  by re-qualifying both hints with `lrs*_rtype === RT_FIX` before arming its
  wait — that neutralises the consequence for the vector memory path and for
  nothing else. It does NOT touch the `:180` speculative-child arm at all, so the
  starvation shape above is UNMITIGATED anywhere. **The slot-level defect is live
  and will bite the next consumer of `iw_p2_bypass_hint`, of `prs2_busy`, or of
  `iw_p2_speculative_child`.** Tracked as a follow-up work item; the local
  compensation is not a fix and must not be read as one.
  A real defect with a plausible hang signature and no known reproducer deserves
  its own change and its own clean run — which is the other reason it is not in
  the A+B pass.

  //@req-spec-issue.g8
  The scalar FP source of a `.vf`-form op or `vfmv.*.f` rides `prs1` as well, with
  `lrs1_rtype === RT_FLT` selecting the FP network for that comparator and
  `RT_FIX` selecting the integer one. One scalar source is enough because the two
  forms are mutually exclusive — a `.vf` op has no `.vx` operand and no address —
  so no third operand slot and no new field is needed, and the FP rename stage
  already renames `prs1` for a uop whose `lrs1_rtype` is `RT_FLT`. Assert that a
  slot never sees `RT_FLT` on `lrs1_rtype` unless `isAluSlot`.

  //@req-spec-vrf.e1
  //@req-spec-vrf.e2
  //@req-spec-vrf.e5
  //@req-spec-vrf.e6
  Because the comparators are baseline's, the SPECULATIVE load-hit wakeup reaches
  them unchanged: a vector uOP waiting on a scalar-load result is woken
  speculatively just like any integer or FP consumer, and the retraction path is
  baseline's too — `prs1_rebusys`/`prs2_rebusys` and the
  `child_rebusys & iw_p*_speculative_child` term set `next_uop.prs*_busy` back to
  true and raise `rebusied_prs1`/`rebusied_prs2`, so the entry re-requests. That
  is the point of the operand-class split: speculation and re-busy exist HERE, on
  the scalar half, and nowhere on the vector half. Both `rebusy` terms keep
  baseline's `lrs*_rtype === RT_FIX` qualification, so the FP-sourced case is not
  re-busied by an integer retraction.

  Readiness on the scalar half reads the REGISTERED `slot_uop.prs*_busy`, as
  baseline does, so an INT wakeup at cycle N produces a request at N+1. The
  vector half is same-cycle (part 3). The asymmetry is deliberate: reading the
  registered bit is what makes the speculative-wakeup/re-busy race safe in
  baseline, and "improving" the scalar half to be combinational would change
  the timing of machinery this design promised to reuse unchanged.

  ---- 3. Vector operands: five matchers, one bit each ----

  //@req-spec-issue.g11
  The vector source operands are `pvs1`, `pvs2`, `pvs3` and the mask `pvm` (V0),
  each held as its member PRNs in `slot_uop` and each given one VecGroupReady
  instance: `rdy_vs1`, `rdy_vs2`, `rdy_vs3` (`isMask = false`) and `rdy_vm`
  (`isMask = true`). A LOAD or ALU slot adds a FIFTH instance, `rdy_vold`
  (`isMask = false`), pointed at the `stale_pvdest` group — see part 4 and part 11.
  Each returns ONE group-ready bit, high only when the operand's LAST valid member
  has completed.

  Every instance's `prns`, `members` and `used` are END-OF-CYCLE values: driven
  from `io.in_uop.bits` while `io.in_uop.valid` is high and from `slot_uop`
  otherwise, and its `load` input is `io.in_uop.valid`. `in_member_rdy` is routed
  per operand from this slot's `in_member_rdy` side channel, and each instance's
  `out_member_rdy` is collected into `io.out_member_rdy` so a collapse move
  carries partial readiness down. `members` is `slot_uop.v_emul` for the pvs
  instances AND for `rdy_vold` (the stale mapping covers the same arch vregs the
  destination does, so the member count is identical by construction); the mask
  instance elaborates no member-count port.

  `vector_operands_ready` is the AND of those `ready` bits — five terms in a load
  or ALU slot, four in a store slot — and nothing else. The slot never looks at a
  `vec_group_done` port itself and holds no per-member state; both live in the
  child.

  The slot also mirrors each matcher's result into the outgoing uop's
  aggregate bit (`next_uop.pvs1_busy := !rdy_vs1.io.ready`, and so on) so that
  a downstream reader of `iss_uop`/`out_uop` never sees a stale busy bit. That
  mirror is NOT the readiness path — readiness is the matcher output — and it
  must not be read back into any request term in this file.

  `rdy_vold` HAS NO MIRROR, and that is deliberate: MicroOp declares no
  `stale_pvdest_busy` field and must not gain one (part 11 — an aggregate stale
  busy bit is unsafe, not merely redundant). The stale group's readiness exists
  only as this matcher's output and as `vold_rdy` on the side channel.

  ---- 4. Which operands participate: `used`, driven per queue ----

  ===> EVERY `used` BELOW IS ADDITIONALLY QUALIFIED BY SLOT LIVENESS —
       `active_valid = in_uop.valid || slot_valid` — AND THE TABLE THAT FOLLOWS
       OMITS THAT TERM ONLY FOR READABILITY. It is not optional.

       `slot_uop` is a plain `Reg` with no reset value, so in an EMPTY slot every
       field the table reads is garbage: X at time 0, and the previous occupant's
       stale fields afterwards. Unqualified, an idle slot presents e.g.
       `v_uses_vs2 = 1` with `v_emul = 0`, and `VecGroupReady`'s in-range
       assertion on `members` fires on a slot holding nothing.

       This is not hypothetical — it is what stopped the FIRST cosim run of gate
       (e1): `iq_v_load.slots_3.rdy_vs2` tripped "members out of range
       1..maxVecMembers while the operand is used" at 785 ns while running
       `vset_test.elf`, a test containing **no vector loads at all**, so that slot
       had been empty for the whole run.

       Qualify `used`; do NOT weaken `VecGroupReady`'s assertion, which is right
       about the invariant. The qualification is functionally free: `used` feeds
       only `ready := !used || group_all_rdy` and that assertion, and `ready` is
       consumed only while the slot is valid — so an idle slot reports ready,
       which is what it already effectively did. Weakening the assertion instead
       would keep a live slot's out-of-range `members` from ever being caught.

  //@req-spec-issue.g29
  //@req-spec-issue.g30
  //@req-spec-issue.g31
  `rdy_vm.used := slot_uop.v_is_masked`, read from the `MicroOp` field and NEVER
  re-derived from `inst(25)` — the instruction word does not reach issue. So
  `pvm`'s busy bit participates in `request` only when the OP.v is actually
  masked; an unmasked OP.v leaves `pvm` DON'T-CARE and its matcher forces ready.
  This is the load-bearing case, not a tidiness one: for an unmasked op `pvm`
  names whatever physical register V0 was last mapped to, and the vector mapper
  renames `lvm` only for a masked op, so waiting on it would wait forever on a
  STALE mask physical register that no producer will ever complete.

  //@req-spec-issue.g32
  //@req-spec-issue.c14
  //@req-spec-lsu.l2
  The other `used` inputs are elaboration-time constants or one-gate functions of
  the resident uop, chosen per queue so that the ADDRESS path waits on exactly the
  operands issue.rst names — the scalar base and stride, the index vector for
  indexed forms, `pvm` when masked, and `pvl` — and so that no path waits on a
  group the instruction does not read:

    ALL QUEUES  rdy_vs1.used  := v_uses_vs1
                rdy_vs2.used  := v_uses_vs2
                rdy_vm.used   := v_is_masked      (as stated above)
    IQ_V_LOAD   rdy_vs3.used  := v_uses_vs3 && !is_shared
                rdy_vold.used := dst_rtype === RT_VEC
    IQ_V_STORE  rdy_vs3.used  := true             (it tracks the SELECTED DGEN group)
                (no rdy_vold instance)
    IQ_V_ALU    rdy_vs3.used  := (is_shared && uses_ldq) || v_uses_vs3
                rdy_vold.used := dst_rtype === RT_VEC

  ===> THE TABLE IS EXPRESSED IN `v_uses_vs*`, NOT IN `is_shared`/`uses_ldq`, AND
       THAT IS THE FIX FOR THE MOST COMMON VECTOR INSTRUCTION IN THE DESIGN. The
       previous version read `IQ_V_LOAD rdy_vs3.used := !is_shared`. But per
       `VLSDecode` a load NEVER encodes `lvs3` — `uses_vs3` is `is_store` — so a
       plain `vle64.v`, with `is_shared` false, took `used := true` and gated on
       `pvs3`, i.e. on the current mapping of **v0**. That is D11's hang reached by
       an ordinary unit-stride load with no segmented op and no corner case: if
       v0's producer's group-done had already fired before the slot captured its
       member bits, no future group-done clears them and the slot waits forever.

  The authoritative per-format values live in the decode specs and must be read
  there, not reconstructed here:
  - `VLSDecode`: `uses_vs1` is FALSE ALWAYS (that field position holds `rs1`, the
    base address); `uses_vs2` is `is_indexed` (otherwise the field is the `rs2`
    stride or `umop`); `uses_vs3` is `is_store` (store data is `vs3`). So A
    NON-INDEXED LOAD HAS ALL THREE FALSE, and the load slot's vs1/vs2/vs3 matchers
    are all gated off.
  - `VDecode`: `uses_vs3` is "has a vector destination"; `uses_vs1`/`uses_vs2` have
    real per-format exceptions — the funct5-selector families where `vs1` is a
    sub-opcode, and the reserved-zero-`vs2` families. Read `VDecode` part 3b for the
    exact predicates.

  The `!is_shared` term on the load half's `rdy_vs3` is the named structural
  discharge of issue.g32: the LSU half of a segmented load never reads `pvs3` —
  that is the COPROCESSOR half's source group — so its busy bit is dropped from the
  readiness cone entirely rather than ANDed in and hoped to be early. With
  `v_uses_vs3` in front of it that term is now REDUNDANT on the load path (a load's
  `uses_vs3` is already false), and it is KEPT ANYWAY: g32 is a named obligation and
  a future change to what decode puts in `uses_vs3` must not silently re-admit
  `pvs3` into the LSU half's cone. In the store slot the same obligation is
  discharged upstream instead, by VecStoreDgenPath's mux (part 6), which leaves the
  deselected group out of the cone at zero comparator cost. `IQ_V_STORE`'s `true`
  is not a special case either: `uses_vs3` is true for every store by construction,
  and when `is_shared` the mux has re-pointed the matcher at `pvtmp`, which is also
  used.

  ===> THE ALU SLOT'S `(is_shared && uses_ldq) ||` TERM IS LOAD-BEARING AND MUST
  NOT BE SIMPLIFIED AWAY TO `v_uses_vs3`. For a segmented LOAD both halves carry
  the SAME uop, decoded by VLSDecode, so `v_uses_vs3` is false — yet part 7's
  select has re-pointed the ALU slot's `rdy_vs3` at `pvtmp`, which the
  coprocessor half genuinely must wait on. `v_uses_vs3` describes the ENCODED
  `lvs3` field and says nothing about `pvtmp`, so the OR term is what keeps the
  matcher live for the re-pointed group. Dropping it silently deletes the
  coprocessor half's only rendezvous with the LSU half — the milestone-1 DGEN bug
  class, in the load direction.

  The previous `!(is_shared && uses_ldq)` exclusions on `rdy_vs1`/`rdy_vs2` are
  DROPPED as unnecessary: for a segmented load's coprocessor half `v_uses_vs1` is
  already false (VLSDecode), and `v_uses_vs2` is true only for an indexed form,
  where `pvs2` is a genuinely encoded source whose producer will complete. Waiting
  on it is correct, if marginally conservative; gating it off would be a second
  place where a real source can be dropped from the cone.

  ===> `v_uses_vs1`/`v_uses_vs2`/`v_uses_vs3` ARE MicroOp FIELDS (decision D11),
  set by `VDecode`/`VLSDecode`, and the mechanism is ONE mechanism on both sides
  of the rename/issue seam: the vector MAPPER skips renaming an unencoded source
  and leaves its busy clear, and this SLOT additionally gates that operand out of
  the readiness cone with `used`. Read together, an unencoded source is neither
  renamed nor waited on.

  This REPLACES the sentence this file used to carry — "rename delivers an
  operand the instruction does not encode with all member-rdy bits SET", stated as
  a contract on VecRenameSpace. That contract could not be discharged, and
  VecRenameSpace said so: it has no `lvs*_rtype` to test, and only decode knows
  the instruction format. The hang it left open is the `vadd.vx` case (`vd, vs2,
  rs1`, `lvs1` unencoded, `pvs1` resolving to v0's mapping) and the `vle64.v` case
  above. Do NOT re-derive these bits from `inst`: the instruction word does not
  reach issue.

  `rdy_vm.used := v_is_masked` is UNCHANGED by D11 — it gates on a different
  question (does this op read the mask) and stays exactly as part 4 states it.

  `rdy_vold.used` is the single coarse term `dst_rtype === RT_VEC`: any OP.v with a
  vector destination waits on its `stale_pvdest` group, in both the load and the
  ALU queue. That is DELIBERATELY CONSERVATIVE in both queues, and neither
  conservatism is removable from here:

  - IQ_V_ALU: the host cannot know whether the coprocessor will actually pull the
    `STALE_VD` source slot. The VPU decides, and NO VPU-SIDE SIGNAL EXISTS to ask.
    So the op waits whenever the pull is possible, i.e. whenever it has a vector
    destination.
  - IQ_V_LOAD: `vta`/`vma` are visible in the per-uOP `VConfig` snapshot, so the
    LCB's undisturbed pre-load could in principle be predicted — but it is NOT the
    only reader. VecGroupCopy performs the whole `pvdest <- stale_pvdest` group copy
    for a VL=0 load regardless of `vta`/`vma`, and VL is not known at issue (the
    value is read from the VL RF at execute, part 5). Narrowing this term to
    `vta === 0 || (v_is_masked && vma === 0)` would therefore let a VL=0 load copy
    from a BUSY stale group.

  A segmented STORE's coprocessor half is excluded for free by the same term: it
  writes `pvtmp` and its `dst_rtype` is not `RT_VEC`, so it has no `stale_pvdest`
  group to wait on and `used` is low.

  ---- 5. `pvl` and `vtype` ----

  //@req-spec-decode.i6
  //@req-spec-issue.h2
  Every vector uOP carries `pvl_src` as an implicit operand and is woken on the VL
  network: one comparator PER VL LANE, OR-ed —
  `io.vl_wakeup.map(w => w.valid && w.bits === slot_uop.pvl_src).reduce(_ || _)`
  over `numVlWakeupPorts` lanes — clearing `next_uop.pvl_busy`. That is the whole
  of it. MATCH ON `pvl_src`, NOT `pvl`: on a VL producer that also reads VL — only
  `vle*ff.v` — `pvl` is the uOP's OWN destination, so matching it makes the slot
  wait for a wakeup only that uOP can produce and only after it issues. A
  permanent hang; see the `pvl_src` entry in VecRenameSpace's part 3.
  A per-lane match with no priority is correct because the lanes are one network
  and `pvl_src` matches at most one of them: the VL RF's write ports are statically
  partitioned per producer class and never arbitrated (D8), so two lanes cannot
  carry the same `pvl` in one cycle. NO VALUE
  IS CAPTURED — the slot holds no VL register, no `vl` field and no width for
  one; the value is read from the VL RF by the vector EU at execute, when the
  AGEN cracks the OP.v into element accesses. `pvl` also gets no speculative
  wakeup and no re-busy term, because no VL producer has a load-use latency worth
  speculating on.

  //@req-spec-issue.g7
  //@req-spec-issue.f11
  `vtype` is NOT an issue-slot operand and is never woken on any network. It
  rides the per-uOP `VConfig` snapshot taken at decode from the speculative VCFG
  mirror, so there is no `pvtype`, no `vtype` busy bit, no `vtype` comparator and
  no `vtype` wakeup port anywhere in this file. A reviewer finding any of those
  should reject the change.

  ---- 6. `request` and eligibility ----

  //@req-spec-core.f5
  //@req-spec-rename.g2
  //@req-spec-issue.g19
  //@req-spec-issue.g20
  A non-shared OP.v occupies ONE slot and is granted ONCE, when ALL of its
  physical operands are ready — scalar feeders on the integer network, the `.vf`
  scalar on FP, `pvl` on the VL network, and the vector groups on the vector
  network. There is no partial grant and no operand class that may be skipped:

    scalar_operands_ready := !(prs1_busy && prs1 participates)
                          && !(prs2_busy && prs2 participates)
                          && !pvl_busy
    vector_operands_ready := rdy_vs1.ready && rdy_vs2.ready &&
                             rdy_vs3.ready && rdy_vm.ready &&
                             (rdy_vold.ready, in a load or ALU slot only)
    request := slot_valid && !slot_uop.iw_issued &&
               scalar_operands_ready && vector_operands_ready

  The `rdy_vold` term is elaborated away in a store slot rather than tied true,
  so a store slot's readiness cone is literally baseline's four terms and the
  fifth instance's area is not paid where nothing reads the stale group.

  In a STORE slot `request` is instead `dgen_path.agen_request ||
  dgen_path.dgen_request`, which is the SAME expression specialized per path: the
  slot hands `dgen_path` an `agen_operands_ready` built from
  `scalar_operands_ready` and the vs1/vs2/vm matchers with `rdy_vs3` EXCLUDED,
  and hands it `rdy_vs3.io.ready` as `dgen_operand_ready`. The store-data operand
  therefore cannot leak into the address path's readiness, which is the
  obligation VecStoreDgenPath asserts at its boundary.

  //@req-spec-issue.g21
  On a `pnrGate` slot — `IQ_V_ALU` only — eligibility is
  `request && IsOlder(slot_uop.rob_idx, io.rob_pnr_idx, io.rob_head_idx)`,
  applied PER ENTRY and not only to the queue head, so every op handed to the CII
  is individually non-speculative. `IQ_V_LOAD`/`IQ_V_STORE` set `eligible :=
  request`: vector memory may issue speculatively, and ordering and replay are
  the LSU's business. The gate lives in the slot precisely because it is
  per-entry; putting it in the unit's select would tempt a head-only
  implementation, whose cost is a segmented store's coprocessor half blocking
  every younger vector arithmetic op.

  ===> THE TEST IS STRICT `IsOlder`, AND BASELINE'S EXTRA EQUALITY TERM MUST NOT
  BE COPIED. BOOM's SNI block writes `issue_slot_past_pnr` with
  `| (rob_idx === rob_pnr_idx)`, but `rob_pnr_idx` names the OLDEST UNSAFE entry
  (`rob.scala:531`), so admitting equality admits exactly the entry whose
  speculation is UNRESOLVED. That is intended in the scalar SNI context, which is
  permissive by construction and off by default; here it would hand a
  still-speculative op to the coprocessor and defeat the entire reason `pnrGate`
  exists. `IsOlder(rob_idx, rob_pnr_idx, rob_head_idx)` and nothing OR-ed onto it.

  `io.iss_uop := slot_uop`, plus, in a store slot, the `fu_code(FC_AGEN)` /
  `fu_code(FC_DGEN)` overrides `dgen_path` supplies. Baseline's grant bookkeeping
  is reused verbatim: `next_uop.iw_issued := io.grant && !io.squash_grant`, the
  two `iw_issued_partial_*` markers defaulted false and driven from `dgen_path`,
  and `next_valid := rebusied` when `slot_valid && slot_uop.iw_issued`, ORed with
  `dgen_path.keep_valid` so a DGEN-pending store survives its AGEN grant.

  ===> TWO PIECES OF BASELINE'S `isMem` BLOCK MUST NOT BE COPIED.
  (a) `io.iss_uop.prs1 := slot_uop.prs2` — the scalar DGEN operand rewrite.
      Vector store data is a VRF group read on port R3, not a scalar operand.
  (b) `io.iss_uop.lrs2_rtype := RT_X; io.iss_uop.prs2 := io.iss_uop.prs1` —
      baseline's DCE helper. In a vector slot `prs2` carries the STRIDE and
      must reach the AGEN intact; clobbering it silently turns every strided
      access into a wrong-address access.

  ---- 7. `pvtmp`: how the consumer half of a shared op wakes ----

  //@req-spec-issue.c11
  //@req-spec-rob.d12
  //@req-spec-lsu.l5
  The two halves of a shared OP.v rendezvous on the `pvtmp` group, and the
  consumer half's IQ slot is woken by `pvtmp`'s group-done on the ORDINARY vector
  wakeup network — no private path, no cross-queue signal, no poll. Mechanically
  that means one of the consumer slot's SOURCE matchers must be pointed at `pvtmp`
  (`rdy_vs3` in both cases; `rdy_vold` is never re-pointed):

  - Segmented STORE, consumer = the LSU half's DGEN path in `iq_v_store`:
    VecStoreDgenPath selects `pvtmp` into `rdy_vs3` whenever `is_shared`.
  - Segmented LOAD, consumer = the coprocessor half in `iq_v_alu`: this slot
    performs the mirror-image selection itself, since `dgen_path` does not exist
    in an ALU slot — `rdy_vs3.prns := Mux(is_shared && uses_ldq, pvtmp, pvs3)`,
    with the companion `members` taken from the same select.

  WARNING — THE SELECT IN AN ALU SLOT IS QUALIFIED BY DIRECTION, and dropping
  that qualification is the mirror image of the milestone-1 DGEN bug. For a
  segmented STORE the coprocessor half WRITES pvtmp and READS pvs3; a bare
  `Mux(is_shared, pvtmp, pvs3)` would make that half wait for the group-done
  of the group it is itself about to produce — an immediate self-deadlock, and
  one that only a segmented store exercises. `uses_ldq`/`uses_stq` are the
  baseline MicroOp fields that carry the direction; they are already set on a
  vector load/store OP.v because it reserves an LDQ or STQ slot at dispatch.

  When `is_shared && uses_ldq` in an ALU slot the transpose reads `pvtmp` and, when
  masked, `pvm`; `rdy_vs1` is gated off by `v_uses_vs1` (false for any LS-decoded
  uop) and `rdy_vs2` only participates for an indexed form, where `pvs2` is a real
  encoded source (part 4). `pvs3` and `pvtmp` stay two intact fields on the issued
  uop — this file selects between
  them and never overwrites one with the other, so VecCiiOperandServer still has
  both plus `is_shared` when it decides which CII source slot to serve.

  ---- 8. Assertions ----

  Synthesizable checks on hardware conditions, not behaviour: `!(io.grant &&
  !slot_valid)`; `!(slot_valid && slot_uop.is_vec === false.B)` — a scalar uop in
  a vector queue is a dispatch-routing bug; `!(slot_valid && slot_uop.is_sfb_shadow)`;
  `slot_uop.v_emul` in 1..`maxMembers` while valid; `lrs1_rtype =/= RT_FLT` unless
  `isAluSlot`; and, when `pnrGate`, that a granted entry was `eligible` in that
  cycle. On a store slot, `dgen_path`'s own assertions cover path ordering. Two
  more for the fifth matcher: a store slot elaborates no `rdy_vold` (an
  elaboration-time check, not a hardware one), and `rdy_vold.used` implies
  `slot_uop.pvdest` is a valid group, i.e. `dst_rtype === RT_VEC` and `v_emul` in
  range — the stale group and the destination group always have the same extent.

  ---- 9. Tracing ----

  Guarded tracing through the shared VecTrace package, one line per key event,
  each tagged `VecIssueSlot` with `rob_idx`, gated on the `vecTrace` plusarg and
  off by default. Four events: slot fill (with `v_emul`, `v_is_masked`,
  `is_shared`), the rising edge of `request`, the grant, and — on the rising edge
  of each matcher's `ready`, `rdy_vold` included — one line naming the operand and
  its exported `out_member_rdy`. A `rdy_vold` line is what makes a stale-group
  stall attributable in a cosim log; without it a wait on `stale_pvdest` and a wait
  on a real source group look identical from outside the slot.
  VecGroupReady deliberately emits nothing itself, because the
  trace helper requires a `MicroOp` and only this module holds one; the last
  member to complete a group is visible here instead.

  ---- 10. `numVecClrPorts`, and what this slot does NOT own of completion ----

  `VectorParams` declares `numVecClrPorts` (3) beside `numVecWbPorts` (3): the ROB
  busy-clear lane count, one lane per group-done producer, never arbitrated,
  because a lost clear is unrecoverable and the ROB entry then never retires. It is
  named here only so that the two 3s are not mistaken for one parameter — this slot
  has NO clear port and no ROB-facing port at all. The single group-done event
  drives three consumers (this slot's matchers, the busy table, the ROB clear) in
  the SAME cycle; skewing them is a lost wakeup or a stale read.

  ---- 11. Stale-destination readiness: RESOLVED as a fifth matcher (D6) ----

  The corpus gap that used to sit here is CLOSED: `issue.rst` "The Vector Issue
  Slot" now states the obligation, and it was extracted as spec-issue.g35-g39.
  The prose below is the same behaviour; it is now traceable rather than merely
  binding.

  //@req-spec-issue.g35
  //@req-spec-issue.g36
  `stale_pvdest` is the PREVIOUS mapping of the destination arch vregs, so its
  producer is an OLDER instruction — and that is precisely why the readiness term
  is needed rather than implied. Age-ordered issue grants the oldest READY entry,
  which does NOT mean an older producer has completed; a younger op can be granted
  while the instruction that installed its stale mapping is still in flight.

  Two consumers read the group on that path:
  - the LCB pre-loads inactive-lane data from it on VRF port `R2` for a `vta=0` /
    `vma=0` load, and VecGroupCopy copies the whole group for a VL=0 load;
  - the coprocessor may pull it as the `STALE_VD` source slot.

  Both would otherwise read a BUSY group and get garbage — silently, with no
  assertion. FOUR NODES REPORTED THIS INDEPENDENTLY during authoring, and it
  matches the prior M2 implementation, which needed exactly this term (`pvold_busy`)
  to avoid a hang.

  //@req-spec-issue.g37
  A STORE slot carries no such matcher: a store has no vector destination, so it has
  no stale group, and `rdy_vold` is elaborated with `count: 0` there.

  //@req-spec-issue.g38
  The match is PER MEMBER, never a single aggregate busy bit. `stale_pvdest` may
  span up to `maxMembers` producers — an `LMUL=1` op writes `v0`, then an `LMUL=8`
  op renames `v0..v7`, so the latter's stale group is the current mapping of eight
  arch vregs installed by up to eight instructions. An aggregate bit cannot express
  "waiting on the third of eight" and one group-done cannot clear it correctly,
  which is the same argument that forces per-member matching on `pvs*`.

  //@req-spec-issue.g39
  The `IQ_V_ALU` gate is deliberately CONSERVATIVE: `rdy_vold.used` is
  `dst_rtype === RT_VEC`, so every CII op with a vector destination waits, including
  the ops that will never pull `STALE_VD`. Whether the coprocessor pulls it is the
  VPU decoder's decision, the issue packet carries no hint of it, and no
  back-channel exists — so the host cannot narrow the gate.

  The fix is the fifth `VecGroupReady` instance, `rdy_vold`, in `IQ_V_LOAD` and
  `IQ_V_ALU` slots only, with `prns := slot_uop.stale_pvdest`,
  `members := slot_uop.v_emul`, `used := dst_rtype === RT_VEC` (part 4), and a
  fifth group `vold_rdy` on the per-member side channel. `VecBusyTable` is amended
  in parallel to export that group's per-member read.

  ===> AN AGGREGATE `stale_pvdest_busy` BIT IS UNSAFE, AND IT IS THE TEMPTING
  WRONG ANSWER — one bit in MicroOp instead of 192 comparators x 32 slots.
  `stale_pvdest` can span UP TO EIGHT PRODUCERS: an `LMUL=1` op writes `v0`, then
  an `LMUL=8` op renames `v0..v7`, so the younger op's stale mapping is the
  current mappings of EIGHT arch vregs, installed by up to eight different
  instructions. One bit cannot express "waiting on producer 3 of 8", and no
  single group-done can clear it correctly — clear on the first arrival and the
  consumer wakes EARLY on a register a later producer has not written; require
  all eight and the bit has no encoding for the intermediate states. This is the
  same argument that forces per-member matching for `pvs*` (rename.g20), and it
  applies unchanged here. MicroOp must therefore NOT gain a `stale_pvdest_busy`
  field, and part 3's busy mirror deliberately has no `rdy_vold` counterpart.

  ACCEPTED CONSERVATISM, recorded so it is not "optimized" later: on the CII
  side the host cannot know whether the VPU will actually pull `STALE_VD` — the
  coprocessor decides and NO VPU-SIDE SIGNAL EXISTS — so any CII op with a vector
  destination waits on `stale_pvdest`. On the load side VL is not known at issue,
  so the VL=0 group-copy reader cannot be predicted either (part 4).

  COST: +1 matcher x (16 IQ_V_LOAD + 16 IQ_V_ALU) slots = 32 instances, +192
  comparators and +8 flops each, IN THE STAGE THAT IS ALREADY THIS DESIGN'S #1
  TIMING RISK. Accepted against a silent wrong-data alternative. If the stage
  fails timing, the mitigation is the shared one-hot completing-PRN decode
  described in VecGroupReady — which the fifth instance shares with the other
  four at no extra decode cost — never a pipeline register here and never
  dropping this matcher.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
ONE SCHEDULING STAGE, ZERO ADDED CYCLES. Wakeup to grant is combinational
through this slot: a group-done in cycle N must be able to produce a grant in
cycle N, so nothing between `vec_group_done` and `request` may be registered. The
only registers here are baseline's `slot_valid` and `slot_uop`, plus the member
bits inside the matcher children. A vector OP.v is allocated and selected ONCE;
there is no second priority-encoder select and no cross-queue kill to keep
consistent.

The critical path is the children's: group-done PRN fan-out into
`EMUL x numVecWbPorts x maxMembers` comparators, the OR-tree, the AND-reduce,
then this module's five-input operand AND (four in a store slot), the unit's
priority encoder and the grant. This module contributes only that AND, the `used`
gates, `numVlWakeupPorts` VL comparators OR-ed, and — on a store slot — one 2:1
group mux. If the path fails timing the mitigation is the shared one-hot decode
described in VecGroupReady, never a pipeline register here.

Area at the defaults: about 792 PRN comparators per IQ_V_LOAD / IQ_V_ALU slot and
600 per IQ_V_STORE slot, essentially all of them in the matcher children, plus
baseline's two integer comparator sets, the FP set in ALU slots only, and
`numVlWakeupPorts` VL comparators (3 at Medium) — times `vecIssueEntries` times
three queues, the dominant area term of the vector issue stage. The D6 `rdy_vold`
instance is +192 comparators and +8 flops on the 32 load and ALU slots, about +21%
on that term; it removes a silent read of a BUSY stale group. Scalar slots
instantiate none of it, which is what keeps the promise that a pure-scalar slot
pays nothing for vector matching.
<|end_perf|>

<|begin_dependencies|>
VecGroupReady — instantiated FIVE times in an `iq_v_load` or `iq_v_alu` slot and
FOUR times in an `iq_v_store` slot: `rdy_vs1`/`rdy_vs2`/`rdy_vs3` with
`isMask = false`, `rdy_vm` with `isMask = true`, and `rdy_vold` (D6) with
`isMask = false` and `count: isStoreSlot ? 0 : 1`. This module owns their `load`,
`in_member_rdy`, `prns`, `members` and `used` inputs and consumes `ready` and
`out_member_rdy`. `numVecWbPorts` is forwarded, not re-declared — both this module
and the child bind to `VectorParams.numVecWbPorts`, so the literal 3 exists in one
place only.

VecStoreDgenPath — instantiated as `dgen_path`, count 1 in `iq_v_store` and 0
elsewhere. This module supplies `slot_valid`, `grant`, `squash_grant`,
`slot_uop`, `agen_operands_ready` (which EXCLUDES the store-data operand),
`dgen_operand_ready` (= `rdy_vs3.io.ready`) and `agen_rebusied` (=
`rebusied_prs1`), routes its `dgen_operand`/`_members`/`_busy` into `rdy_vs3`'s
inputs, and consumes `agen_request`/`dgen_request`, `iss_fu_code_*`,
`next_fu_code_*`, `issued_partial_*`, `keep_valid` and `both_paths_done`. On the
two-term `!dgen_operand_busy && dgen_operand_ready` gate it specifies: this module
drives the busy bits it hands `dgen_path` from the live matcher state, so the two
terms can never disagree and the redundant term cannot re-introduce a stall.

MicroOp — `is_vec`, `is_shared`, `uses_ldq`/`uses_stq`, `pvs1`/`pvs2`/`pvs3`,
`pvm`, `pvtmp`, `stale_pvdest`, `pvl`, `dst_rtype`, the aggregate busy bits,
`v_emul`, `v_is_masked`, `v_uses_vs1`/`v_uses_vs2`/`v_uses_vs3`,
`rob_idx`, plus baseline's `prs1`/`prs2`, `lrs*_rtype`, `iw_issued`,
`iw_issued_partial_agen`/`_dgen` and the `iw_p*` hint fields. Operand port types
come from `chiselTypeOf` on those fields. This module adds no field to MicroOp; in
particular it needs no per-member busy vector there (which is exactly why the side
channel exists) and no `stale_pvdest_busy` aggregate (logic part 11).

`v_uses_vs1`/`v_uses_vs2`/`v_uses_vs3` are MicroOp's D11 addition, three Bools
written by `VDecode`/`VLSDecode`. They are the `used` inputs for
`rdy_vs1`/`rdy_vs2`/`rdy_vs3` (logic part 4), and they are the SAME bits the
vector mapper uses to skip renaming an unencoded source — one mechanism read from
two sides, not two. They DISCHARGE the obligation this file previously placed on
VecRenameSpace, which could not meet it (no `lvs*_rtype` exists), and they mirror
`v_is_masked`, which exists for the identical reason. The slot reads them and does
not re-derive them; `v_is_indexed` is no longer read here at all, because for an
LS-decoded uop `v_uses_vs2` IS the indexed predicate.

VecBundles — `VecGroupDone` on the wakeup ports (passed through to the children
only), and `VecMemberRdy`, the per-member side-channel bundle, which now belongs
THERE and not in this file: it crosses VecRenameSpace, VecIssueUnit and this
module, VecPipeline settled it as one declaration under that name, and D6 widened
it with `vold_rdy`. Bind to the single declaration; `VecSlotMemberRdy` is the same
bundle under a second name and must not survive as a copy.

VecTrace — the guarded emit helper used in part 9 of the logic section.

Binds to EXISTING BOOM v4 declarations rather than re-spelling them: `Wakeup`
(v4/exu/execution-units/execution-unit.scala), `BrUpdateInfo`, `UpdateBrMask`,
`IsKilledByBranch`, `IsOlder`, `FC_AGEN`/`FC_DGEN`, `RT_FIX`/`RT_FLT`/`RT_X`, and
`RT_VEC` from the ScalarOpConstants delta (the `rdy_vold.used` term).

VectorParams — `vecPregSz`, `vlPregSz`, `maxMembers`, and the three port counts it
now declares: `numVecWbPorts` (3), `numVecClrPorts` (3, named but not used here)
and `numVlWakeupPorts` (`aluWidth + 1`, which sizes `vl_wakeup`). Bind to those
fields; do not re-default any of them locally.

Instantiated by VecIssueUnit as `slots`, `count: numEntries`, in all three
IQ_V_* instances. The unit owns the collapse-move wiring, and must route
`out_member_rdy` to `in_member_rdy` — EVERY group, `vold_rdy` included — on the
same move it routes `out_uop` to `in_uop`. The dispatch-time values of those groups
come from VecRenameSpace's per-member busy-table read; `VecBusyTable` is amended in
parallel to export the stale-destination group's per-member read alongside the ones
it already exports, which is what makes this the FIFTH matcher and not a fifth
mechanism.
<|end_dependencies|>

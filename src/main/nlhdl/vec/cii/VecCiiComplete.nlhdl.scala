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
  VecCiiComplete — the completion half of the CII Writeback direction: on the
  beat the coprocessor MARKS `last` it emits, exactly once, one group-done, one
  ROB busy-clear, the accrued `fflags`/`vxsat`, and the tag free.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/cii/VecCiiComplete.scala,
  package boom.v4.vec.generated.cii, group vec_cii.
  depends_on VecBundles, VecTrace. Instantiated ONCE, as `done`, by VecCiiHost.
  Instantiates nothing.

  ===> WHY THIS IS A SEPARATE NODE FROM VecCiiWriteback, and why the split is
       clean. THE SEAM IS THE `last` BIT. The coprocessor marks the final beat of
       a tag; the host reads that one bit. Completion therefore needs nothing
       from beat PLACEMENT — not the 256-bit `wb_data`, not `wb_dst_offset`, not
       the W2 byte enables — only {`tag`, `wb_status`} plus the tag's side-table
       entry. The Writeback channel deliberately carries NO expected-count field
       and the host must never derive one: a widening or narrowing op emits a
       member count that DIFFERS from its source EMUL, so a host-side beat count
       would be right everywhere except exactly those instructions.
       This low coupling is what makes the split clean, and it is why the
       equivalent split was NOT made inside VecLoadCoalescingBuffer: there,
       assembly and completion share the per-entry byte-valid bitmap, so cutting
       between them would put a shared counter on a module boundary. Here there
       is no shared counter to cut.

  ===> ALL FOUR EFFECTS OF THE KILL CONTRACT ARE SUPPRESSED, AND THE TAG FREE IS
       NOT ONE OF THEM. This module is the second half of the drain contract
       whose first half is VecCiiFlush: a killed tag's `last` beat is still
       POPPED and its `wb_credit` still returned — VecCiiWriteback's obligation —
       but no group-done, no ROB clear, no CSR side effect and no `fflags`/`vxsat`
       accrual happen. The tag is freed on its DROPPED `last` beat exactly as a
       live one is: the tag LIFETIME is identical, only the effects differ.

  Governing spec anchors: cii.rst `cii-writeback`, `cii-kill-contract`,
  `cii-segmented`; midcore.rst `group-done-wb`, `precise-vec-exc`;
  execution.rst `vector-execution` ("What the coprocessor provides");
  issue.rst `shared-store-chain`.
*/

<|begin_module|>

  <|begin_parameters|>
  ---- ELABORATION GATE ----

  Elaborated only when `usingRVV` is true — a Scala `Boolean` from
  `BoomCoreParams`, not a hardware `Bool`, and NOT rocket's `usingVector`. With
  vectors disabled VecCiiHost does not exist, so this module is ABSENT rather
  than tied off, and a non-vector build stays bit-identical to pre-Caracal
  BOOM v4.

  ---- Sizing (all derived; nothing here is a literal) ----

  Every size comes from `VectorParams`, reached through the VecBundles edge
  hierarchy.yaml grants; none is re-derived and none is a literal. `ciiTagBits` —
  the tag width, default 4, mirroring `CII_TAG_W` in the frozen
  `tt_cii_caracal_pkg.svh` and not a free choice; `numCiiTags = 1 << ciiTagBits`
  = 16 (`CII_N_TAGS`) sizes the per-tag flag accumulator and nothing else.
  `maxMembers` — 8 (`MAX_MEMBERS`), the width of the group-done's member-PRN
  vector. `vecPregSz` — the vector PRN width (7 at 96 PRNs). `robAddrSz` — the ROB
  index width. `FLAGS_SZ` — 5, from
  `freechips.rocketchip.tile.FPConstants.FLAGS_SZ`, never written as 5.

  There is deliberately no expected-beat-count parameter, no per-tag beat-counter
  width and no completion timeout; that absence is the specification, not an
  omission — see part 1 of the logic section.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel and hierarchy.yaml defaults: posedge `clock`,
  ACTIVE-HIGH SYNCHRONOUS `reset`, single `core_clk` domain. Both are used — the
  per-tag flag accumulator is the module's only state.

  - `io.beat`       — Flipped(Valid{ `tag`: `ciiTagBits`, `status`: the
                      `wb_status` sub-bundle of `CiiWriteback` (VecBundles), i.e.
                      { `last`, `dst_kind`, `vxsat`, `fflags` } }). The beat
                      VecCiiWriteback has ACCEPTED AND POPPED this cycle, live or
                      killed alike, and it is driven COMBINATIONALLY — NEITHER
                      SIDE REGISTERS IT, so a beat's placement and its completion
                      land in the same cycle. NOT "Writeback's registered beat":
                      that phrasing appeared in an earlier draft of this port
                      comment and contradicted this file's own performance section,
                      which requires same-cycle. The two files now say the same
                      thing in the same words, and the failure the agreement
                      prevents is a register on ONE side only — which would skew
                      group-done, and therefore a dependent issue, one cycle AHEAD
                      of the final `W2` write. There is deliberately NO `ready`:
                      the credit-metered channel has no back-pressure line and
                      this module always absorbs a beat.
                      // `wb_data` and `wb_dst_offset` are NOT ports here: a
                      // 256-bit fan-out into a module with no datapath is pure
                      // cost, and their absence is what makes the split
                      // reviewable — everything read here is 15 bits wide.
  - `io.wb_lookup`   — This module's reader of VecCiiTagTable's declared
                      `wb_lookup` port: combinational, no handshake, answering in
                      the SAME cycle as the beat. Of the response it reads five
                      fields — `pvdest_grp` (Vec(`maxMembers`,
                      UInt(`vecPregSz`.W)), the destination group's member PRNs),
                      `members` (log2Ceil(`maxMembers`+1) bits), `rob_idx`,
                      `is_shared` and `killed`; `prn`, `wr_en` and `pdst` belong
                      to the port's other reader, VecCiiWriteback. The request
                      side is { `tag`, `wb_dst_offset` } off the same beat: both
                      readers present the SAME tag in the same cycle, so if the
                      table implements one shared lane this module drives no
                      address at all. Either wiring is acceptable; a SECOND
                      16-to-1 entry mux for this module is not.
  - `io.kill_all`    — Input Bool from VecCiiFlush, its whole functional output —
                      a bare combinational bit, which is all `flush` exports and
                      all its reject list permits it to export. Needed IN ADDITION
                      to the looked-up `killed` bit: a beat can arrive in the very
                      cycle the flush fires, one cycle before `killed` is readable
                      from the entry, so the suppression term of part 5 is
                      `io.kill_all || io.wb_lookup.killed`.
                      // The ASYMMETRY WITH VecCiiWriteback IS DELIBERATE, and a
                      // reader should not "unify" the two: this module forms the OR
                      // itself because it needs `killed` from the same lookup
                      // anyway, while `wb` takes ONE pre-computed `wb_suppress`
                      // that VecCiiHost forms from the identical two terms, so that
                      // `wb` needs no flush port at all. Same two terms, two homes,
                      // one settled naming — `flush` exports `kill_all` and
                      // exports no per-channel suppress output to either of us.
  - `io.group_done`  — Output Valid(`VecGroupDone`, from VecBundles). ONE per
                      completing vector destination group.
  - `io.clr_rob`     — Output Valid(UInt(`robAddrSz`.W)). The single-shot ROB
                      busy-clear for the owning `rob_idx`.
  - `io.rob_flags`   — Output Valid{ `rob_idx`, `fflags`: `FLAGS_SZ`, `vxsat`:
                      Bool }. The accrued FP exception flags and the sticky
                      fixed-point saturation bit, handed to the owning ROB entry
                      so both are applied AT COMMIT.
  - `io.free_tag`    — Output Valid(UInt(`ciiTagBits`.W)). The tag free, to
                      VecCiiTagTable's free port. Fires on EVERY `last` beat.

  There are no other ports, and these two absences are load-bearing:

  ===> NO `busy`, NO `ready` TO ANY ISSUE UNIT, NO PER-INSTRUCTION STATUS, AND NO
       PER-MEMBER OUTPUT. Issue credit is VecCiiIssue's registered FIFO-occupancy
       mirror; nothing about completion gates issue, and there is no port on which
       a per-member completion could leave this module even by mistake.
  ===> NO EXCEPTION PORT. This module drives no `vec_xcpt` and no `vstart`: a CII
       arithmetic op completes atomically, so nothing partial exists for a trap to
       describe (part 8).
  <|end_ports|>

  <|begin_logic|>
  ---- 0. The two bundles declared here ----

  hierarchy.yaml grants this node no MicroOp edge and VecBundles declares no
  CII-completion bundle, so declare `CiiCompleteBeat` ({ `tag`, `status` }) and
  the `CiiTagCompletionView` of the side-table entry locally, with the field
  lists given in the ports section. `status` REUSES the `wb_status` sub-bundle
  type from VecBundles by name — never re-spelled, so no consumer can slice the
  9-bit status by hand and get `last` wrong. `VecGroupDone` likewise comes from
  VecBundles unchanged.

  ---- 1. The `last` bit is the ONLY completion signal ----

  //@req-spec-cii.g9
  Completion is driven by the writeback `last` bit. The beat the coprocessor
  marks `last` for a `tag` IS that tag's final beat, and the host treats the
  marking as authoritative: it performs no validation of it, has nothing to
  validate it against, and derives no completion condition of its own.

  //@req-spec-cii.g15
  //@req-spec-cii.g23
  THE HOST MUST NOT INFER COMPLETION BY COUNTING BEATS, and this module is where
  that promise is kept: it declares NO per-tag beat counter, NO expected-count
  register and no comparison of any count against anything. The Writeback channel
  carries no expected-count field to compare against either, which is the same
  decision seen from the coprocessor side.
  // WHY, concretely: the number of writeback beats is the DESTINATION member
  // count, and a widening op (EEW doubling) or a narrowing op emits a member
  // count that differs from the source EMUL the host renamed against. A
  // host-derived count would therefore be correct for every ordinary op and
  // wrong for exactly the widening/narrowing ones — a bug that passes the smoke
  // test and fails in a kernel. One marked bit costs nothing and cannot drift.

  For bring-up an `assert` may compare the VEC-destination beats observed for a
  tag against `io.wb_lookup.members`. It lives entirely inside the assertion — no
  functional output, no completion term, readable by nothing — so it is a
  cross-check on the coprocessor, never a completion mechanism.

  ---- 2. The three effects this module emits on a live `last` beat ----

  //@req-spec-cii.g10
  //@req-spec-cii.g11
  //@req-spec-cii.g12
  When `io.beat.valid && io.beat.bits.status.last` and the entry is NOT `killed`
  and `status.dst_kind` is `CII_DST_VEC`, this module emits in that ONE cycle,
  each as a single-cycle pulse and each exactly once per tag:
    - `io.group_done.valid`, carrying the member-PRN vector (part 3);
    - `io.clr_rob.valid` with `io.wb_lookup.rob_idx` — a SINGLE ROB busy-clear;
    - `io.free_tag.valid` with the beat's `tag`.
  Nothing is queued and nothing is deferred. Because `CII_NUM_DST_WB = 1` the
  coprocessor cannot present two result beats in a cycle, so at most one tag
  completes per cycle: all three ports are single-ported and NO arbitration or
  hold-one-back logic exists here, unlike the LCB at `lsuWidth = 2`.

  Outputs are combinational in the popped beat and the combinational side-table
  read — one level of AND/mux, and NO register on either side of the seam. That
  is safe here, unlike on the load path, because the only loop through this event
  (group-done to vector wakeup to issue to CII Issue to Writeback) traverses the
  coprocessor and is multi-cycle by construction. Adding a register "for safety"
  is not free: it lands on the segmented-store serial chain (part 7).

  ---- 3. The group-done payload, and why there is no per-member completion ----

  //@req-spec-rob.c1
  The tt_CII coprocessor is IN-ORDER, so it knows when an entire OP.v has retired
  and signals completion ONCE PER INSTRUCTION — which maps straight onto BOOM's
  existing single-writeback busy-clear in `rob.scala`. It is the same group-done
  contract the LCB honours on the load path: one contract, two producers, so the
  ROB needs no CII-specific completion path. `io.group_done.bits` carries
  `io.wb_lookup.pvdest_grp` as the member-PRN vector, `members` from the same
  entry, and the shared `rob_idx`. `members` is the RENAMED destination group's
  size — what the free list actually allocated — never a beat count (part 1). The
  `pvl` field of `VecGroupDone` is left invalid: a CII op is never a VL producer,
  since the `vset` family is a scalar uop whose VL write comes from the integer
  ALU, not over this interface.

  //@req-spec-cii.g14
  THERE IS NO PER-MEMBER ROB COMPLETION. The per-member effect of a writeback
  beat is a VRF `W2` write, and that is VecCiiWriteback's, visible only to the
  register file; no beat other than `last` produces any ROB-facing or
  wakeup-facing event, and this module has no port on which one could appear.
  // This is exactly why the ROB needs NO per-entry completion counter
  // (midcore.rst `group-done-wb`). Streaming per-member completions in is what
  // a single-shot `rob_bsy` clear and a per-PRN vector Busy Table cannot
  // absorb, so adding one is a design-invariant violation, not an optimization.
  The one event drives three consumers — the ROB single-shot busy-clear, the
  vector Busy-Table clear and the VECTOR wakeup network — so all three see the
  same completion in the same cycle and cannot drift.

  ---- 4. Flags accrual, and the scalar-destination case ----

  //@req-spec-cii.g5
  `wb_status` FP flags ACCRUE TOWARD `fflags` AT COMMIT. Per tag, keep a
  `numCiiTags`-entry accumulator of { `fflags` (`FLAGS_SZ`), `vxsat` (1) },
  OR-ing in `status.fflags` and `status.vxsat` on every accepted beat of a
  non-killed tag — `vxsat` is the sticky fixed-point saturation bit, so OR is its
  defining behaviour, and OR is also correct for the five FP flags. On the `last`
  beat the accumulated pair is emitted once on `io.rob_flags` with the entry's
  `rob_idx`, and the accumulator entry is CLEARED in the same cycle as the tag
  free (equivalently, at allocation) so no residue leaks into the next user of
  that tag number. Neither flag is applied to a CSR here: `fflags` lands in the
  owning ROB entry's `rob_fflags` slot and reaches `csr.io.fcsr_flags` through
  BOOM's existing commit path, and `vxsat` reaches `csr.io.vector.set_vxsat` from
  the same entry at commit. Accruing in ROB order at commit is what makes both
  correct under a flush, and it is also why exactly ONE flags write per
  instruction is permitted: `rob.scala:405` asserts `!rob_fflags(row_idx).valid`
  on a second write to the same entry.

  A handful of vector instructions write a SCALAR register instead (`vmv.x.s`,
  `vcpop.m`, `vfirst.m` to an integer register; `vfmv.f.s` to an FP register),
  selected by `status.dst_kind`. For those, on `last`: the tag is freed and
  `io.rob_flags` is emitted as above, but `io.group_done` and `io.clr_rob` are
  BOTH suppressed — there is no vector destination group to announce, and the
  ROB busy-clear rides the ordinary INT/FP writeback (`int_wb`/`fp_wb`, an
  `ExeUnitResp`) that VecCiiWriteback already drives, per the reuse ground rule.
  // TWO SINGLE-SOURCE RULES FOLLOW, and both are seam obligations on
  // VecCiiWriteback: exactly one ROB busy-clear per instruction (its
  // ExeUnitResp for a scalar dest, this module's `clr_rob` for a vector dest),
  // and exactly one fflags write per instruction (always this module's) — so
  // the scalar-dest `ExeUnitResp.fflags` must be driven to zero/invalid, or the
  // rob_fflags assert above fires on the second write.

  ---- 5. The kill contract: suppress, do not skip ----

  //@req-spec-cii.g13
  //@req-spec-cii.e18
  //@req-spec-cii.e19
  //@req-spec-cii.e20
  Every effect above is qualified by `!killed_now`, where `killed_now =
  io.kill_all || io.wb_lookup.killed` — BOTH terms, because the registered
  `killed` bit is not readable until the cycle AFTER the flush and a beat can
  arrive in the flush cycle itself. For a killed tag the four effects named by the
  kill contract are suppressed: the VRF/INT/FP write
  (VecCiiWriteback's), `io.clr_rob`, `io.group_done`, and the `fflags`/`vxsat`
  accrual — for a killed tag the accumulator is not updated at all AND
  `io.rob_flags` is not asserted on its `last` beat, so neither a beat-by-beat
  accrual nor a final one can reach a CSR by any route. The beat itself is still
  POPPED and its `wb_credit` still returned; that is VecCiiWriteback's obligation
  and not optional, because the channels have no ready line and a swallowed beat
  stalls the channel for every surviving instruction.
  // Suppression is at the EFFECT, not at the beat. A killed instruction must be
  // allowed to FINISH ON JUNK: the VPU is in-order and the SV has no kill line,
  // so there is no way to stop the beats — only to make them inert.

  ---- 6. Tag lifetime is identical for a killed tag ----

  //@req-spec-cii.e21
  A KILLED TAG IS FREED ON ITS (DROPPED) `last` BEAT, EXACTLY LIKE A LIVE ONE:
  `io.free_tag` is the one output NOT qualified by `killed_now`, and it is not
  qualified by `dst_kind` either. `killed` is idempotent — a second flush while
  the tag is draining re-sets a bit that is already set — so no re-initialization
  and no per-tag flush counter exist here.
  // FREEING EARLY IS THE BUG THIS PARAGRAPH EXISTS TO PREVENT. If a flush freed
  // its tags immediately, a tag could be REALLOCATED to a new instruction while
  // the killed instruction's beats were still arriving; those beats would then
  // resolve against the new tag's side-table entry and be placed into a live,
  // unrelated `pvdest` group — silent corruption of a correctly-executing
  // instruction. The CII avoids it by NOT recycling the tag during the drain,
  // which is why the LSU's PRN-recycling squash needs a different mechanism
  // entirely (see VecSquashUnit); do not port this pattern there.

  ---- 7. Segmented (shared) instructions: one shape, no special case ----

  //@req-spec-cii.i3
  //@req-spec-rob.d18
  THE COPROCESSOR HALF ALWAYS EMITS A GROUP-DONE, and this module needs no
  `is_shared` special case to do it. The group-done names the member PRNs of the
  ONE group field the side table resolves for the coprocessor's destination
  (`io.wb_lookup.pvdest_grp` as VecCiiTagTable declares it), whatever group that
  is — `pvtmp` for a segmented store's coprocessor half, `pvdest` otherwise —
  chosen ONCE at tag allocation, by a mux VecCiiTagTable emits there
  (`Mux(uop.is_shared && uop.uses_stq, uop.pvtmp, uop.pvdest)`). This module applies
  no instruction-dependent reinterpretation of it, exactly as VecCiiOperandServer
  applies none to `pvs3` versus `stale_pvdest`, and it must NOT re-derive the choice
  from `is_shared` (which it reads only to trace and assert): two places deciding
  which group a group-done names is how a segmented store announces the load-side
  group. Equally, the single field must genuinely BE `pvtmp` in that case — if the
  table loaded it unconditionally from `uop.pvdest`, this module's group-done would
  announce a group rename never allocated, and cii.i4/i8/i11 would be unsatisfiable
  with no port here on which the shortfall could be repaired.
  For a segmented LOAD the CII transposes `pvtmp` into `pvdest`
  and its group-done is the SECOND of the two halves; the ROB's one-bit "other
  half pending" flag pairs it with the LSU half's earlier `pvtmp` group-done and
  clears `rob_bsy` on the second arrival. With this wired, full segmented LS —
  deferred in Milestone 1 — completes.

  //@req-spec-issue.d8
  For a segmented STORE the roles reverse and this is step 5 of the six-step
  chain in issue.rst `shared-store-chain`: the coprocessor transposes, writes
  `pvtmp`, and emits its group-done — which is what wakes the store slot's DGEN
  path so step 6 can read `pvtmp` as store data. The group-done fires in the SAME
  cycle the last `pvtmp` member's `W2` write is presented, which is safe for the
  same reason it is on the load path: the wakeup-to-VRF-read distance is at least
  one cycle (issue select, then register read), so the write has committed before
  DGEN reads it. Do NOT delay the group-done a cycle to "order" it after the
  write — the ordering is a consequence of the pipeline, and that cycle sits on a
  serial chain whose steps 1 and 6 can already be hundreds of cycles apart. Note
  also that the handoff IS this group-done, not a translation-complete signal;
  the segmented store's second ROB completion is the LSU half's `lsu_clr_bsy`,
  after DGEN.

  ---- 8. Atomic completion and instruction-granularity exceptions ----

  //@req-spec-rob.g14
  //@req-spec-rob.g15
  //@req-spec-rob.g16
  CII ARITHMETIC OPS COMPLETE ATOMICALLY. The `last`-beat event of part 2 is the
  ONLY completion the ROB ever sees for a CII op, so from the ROB's point of view
  the instruction is wholly incomplete or wholly complete — there is no partial
  state, the intermediate per-member VRF writes being invisible outside the
  register file. Consequently a CII op raises exceptions at INSTRUCTION
  GRANULARITY and its exception report carries NO ELEMENT INDEX: no field of this
  module's state and no field of any port it drives names an element. It holds no
  element cursor, no `fault_elem` and no `vstart` contribution, and drives no
  exception port at all; should the VPU ever report a fault it is a plain precise
  exception on the shared `vec_xcpt` path, whose `VecException` bundle has no
  element field either.
  // The two properties are the same property. A per-member ROB completion
  // (part 3) would make an instruction look resumable mid-group, and then an
  // element index would be needed to describe where to resume — which
  // midcore.rst `precise-vec-exc` shows to be unimplementable here, because the
  // partially-written `pvdest` group is returned to the free list on the trap.

  ---- 9. Trace ----

  Emit one guarded `VecTrace` line per key event, tagged with module name
  `VecCiiComplete` and with `rob_idx`, gated on the `vecTrace` plusarg (off by
  default) and on `!reset`: `last_live` (tag, rob_idx, members, dst_kind),
  `last_killed` (tag, rob_idx), `free` (tag) and `flags` (fflags, vxsat). With no
  unit tests in this project these four lines are the only way to tell this
  module's three failure modes apart in a cosim divergence — a completion that
  never arrived, one that arrived for a killed tag, and a tag freed without one.
  Tracing declares no state, so deleting every call site leaves the design
  bit-identical.
  // NOTE FOR THE VecTrace SEAM: this module holds a real `rob_idx` (from the
  // side-table) but NO MicroOp, while the `trace` helper's signature takes a
  // MicroOp. It needs the rob_idx-keyed form of the helper — the analogue of the
  // `traceDecode` variant that exists because decode has no rob_idx. It must
  // neither fabricate a MicroOp nor fall back to `rob=?`, since it knows the
  // real value.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Completion must cost ZERO cycles beyond the pop: the group-done, the ROB clear,
the flags and the tag free are all asserted in the same cycle VecCiiWriteback
pops the `last` beat, WHICH REQUIRES THAT NEITHER SIDE REGISTERS `io.beat` — the
port comment and this section state the one rule, not two. That is a constraint,
not a preference, in two places — the
tag free is the only thing that returns an in-flight tag to VecCiiIssue's credit
pool, so latency here is issue-credit latency at 16 tags; and the group-done on a
segmented store sits on the six-step chain of issue.rst `shared-store-chain`.

Throughput: one completion per cycle is both sufficient and the architectural
maximum, because `CII_NUM_DST_WB = 1` means the coprocessor cannot present two
result beats — hence two `last` beats — in one cycle. No port here may be widened
to chase throughput; the beat rate is the coprocessor's.

Area is deliberately tiny: `numCiiTags * (FLAGS_SZ + 1)` flops of flag
accumulator (96 at the defaults) and no other state. There are NO per-tag beat
counters, which would have cost `numCiiTags * log2Ceil(maxMembers+1)` flops plus
a per-tag comparator to buy a completion signal already on the wire — and, per
part 1, would be wrong for widening and narrowing ops.

The path from `io.beat` to `io.group_done` is one side-table read plus a two-term
AND, but it fans out in that cycle to the ROB, the vector Busy Table and every
vector issue slot's wakeup comparators: that fan-out, not this module's logic, is
the term to watch in timing closure.
<|end_perf|>

<|begin_dependencies|>
VecBundles — `VecGroupDone` (the completion payload this module fills) and the
`wb_status` sub-bundle of `CiiWriteback` ({ `last`, `dst_kind`, `vxsat`,
`fflags` }), reused by name rather than re-spelled.
VecTrace — the guarded trace helper; emit-only, declares no state.

Binds to `freechips.rocketchip.tile.FPConstants.FLAGS_SZ` for the fflags width,
and to the `CII_DST_VEC`/`CII_DST_INT`/`CII_DST_FP` and `CII_N_TAGS`/`CII_TAG_W`
constants DERIVED from `tt_cii_caracal_pkg.svh` — never redeclared here, since
the SV package is the authoritative side of that contract.

Instantiates NOTHING. It is instantiated once, as `done`, by VecCiiHost, which
wires: `io.beat` from `wb` (VecCiiWriteback), in the cycle `wb` pops the beat and
returns its `wb_credit`; `io.wb_lookup` and `io.free_tag` to `tags`
(VecCiiTagTable), sharing the `wb_lookup` port with `wb`; `io.kill_all` from
`flush` (VecCiiFlush), the same bare Bool that sets `tags`' `tag_killed`;
`io.group_done` out through VecPipeline to the ROB's
`vec_clr_bsy`, the VecBusyTable clear and the vector wakeup in VecIssueUnit — one
event, three consumers; and `io.clr_rob`/`io.rob_flags` into the Rob delta's
single-shot `vec_clr_bsy` and the per-entry `rob_fflags`/sticky-`vxsat` slots.

===> THE CII'S ROB CLEAR NEEDS ITS OWN LANE. `vec_pipeline_io` declares
     `vec_clr_bsy` as ONE Valid(rob_idx), but there are two independent
     producers of it — the LSU-side group-done and this module — and NEITHER can
     be back-pressured: this one frees its tag in the same cycle, so a clear it
     could not present would be lost with no way to regenerate it. VecPipeline
     must therefore present one `vec_clr_bsy` lane PER PRODUCER (the ROB already
     takes multiple writeback ports), never arbitrate a shared one. This module
     drives its port unconditionally and is never told it lost.

Deliberately NOT depended on: MicroOp (hierarchy.yaml grants no such edge — the
`rob_idx` and the destination group arrive from the side-table, not as a uop) and
VecRegFile (this module touches no VRF port and adds none to the canonical table
in midcore.rst `vrf-ports`).
<|end_dependencies|>

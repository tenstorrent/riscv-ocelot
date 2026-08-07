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
  Rob — DELTA SPEC. This file is NOT a description of BOOM v4's reorder buffer.
  It describes only the change Caracal applies to the existing `class Rob` and
  `class RobIo` in src/main/scala/v4/exu/rob.scala, which is hand-written
  baseline BOOM v4 and stays in place.

  hierarchy.yaml: kind: module, mode: edit_existing,
  target src/main/scala/v4/exu/rob.scala, group host. No `output:` — the
  pre-existing file is the artifact. depends_on MicroOp, ScalarOpConstants.
  Instantiates nothing new. Budget (plan section 11): ~110 added lines.

  Everything already in `class Rob` is UNCHANGED and is not restated here: the
  four-state FSM (`s_reset`/`s_normal`/`s_wait_till_empty`/`s_rollback`), the
  `rob_head`/`rob_tail`/`rob_pnr` pointer trio and their `_lsb` fields, the
  per-bank `rob_val`/`rob_bsy`/`rob_unsafe`/`rob_uop`/`rob_exception`/
  `rob_predicated`/`rob_fflags` arrays, `RobCompactUop` and its SRAM plus the
  two-deep bypass, the `io.wb_resps` writeback loop, `io.lsu_clr_bsy`,
  `io.lsu_clr_unsafe`, `io.lxcpt`, `io.csr_replay`, the commit/exception/flush
  logic, `can_commit`/`will_commit`/`block_commit`, the PNR block and both of its
  pointer-ordering asserts, and the tail/full/empty logic. Anything this file
  does not mention keeps its current declaration, wiring and cycle behaviour.

  THE WHOLE DELTA, in one paragraph: one existing field widens (`dst_rtype`, to
  hold `RT_VEC`); three input ports are added (`vec_clr_bsy` laned per producer,
  `vec_clr_unsafe`, `vec_rob_flags`) and one output (`com_vxsat`); and three
  narrow per-entry state items are added — `rob_other_half` (1 bit, the shared
  instruction), `rob_vconfig` (the executed-`vtype` latch for `vsetvl`) and
  `rob_vxsat` (1 bit). Nothing else. Almost every requirement allocated to this
  node is a MUST-NOT-CHANGE constraint, and those are discharged by the
  edit_scope section at the end of this file, which is the enforceable artifact —
  not by logic added here.

  ===> DO NOT ADD A PER-ENTRY GROUP COMPLETION COUNTER. A vector destination
       group of up to EMUL = 8 physical registers completes as ONE group-done
       event carrying the group's member-PRN vector, and the ROB's side of that
       is the SAME single-shot `rob_bsy` clear a scalar op uses. Per-PRN counting
       lives in the producer (the LCB's assembly entries, the CII tag table). A
       counter here would be a second, disagreeing copy of state the producer
       already owns, in the one structure whose entries are the most numerous.

  ===> A VECTOR OP MUST CLEAR `rob_unsafe`, OR THE PNR ASSERT AT rob.scala:436-442
       TRIPS AND THE MACHINE DEADLOCKS. A vector load/store sets `starts_unsafe`
       through `uses_ldq`/`uses_stq` exactly like a scalar one, but it has no
       scalar `lsu_clr_unsafe` firing for it, so without the new port the PNR
       parks on the entry forever, `IQ_V_ALU`'s past-PNR gate never opens, and the
       first exception reaching that row asserts. This was observed in Milestone 1
       (plan section 2 bug list, and the plan's Phase C note).

  Governing spec anchors: midcore.rst `rob-vec`, `group-done-wb`, `vec-commit`,
  `precise-vec-exc`; issue.rst `cii-shared-sched`, `shared-store-chain`;
  loadstore.rst `order-fail-replay`, `elem-progress`; frontend.rst
  `vcfg-recovery`; overview.rst `caracal-pipeline`; glossary.rst.
*/

<|begin_module|>

  <|begin_parameters|>
  NO NEW CONSTRUCTOR PARAMETERS. `class Rob(numWakeupPorts: Int, usingTrace:
  Boolean)` keeps exactly that signature. In particular `numWakeupPorts` is NOT
  raised: vector completion does not arrive on `io.wb_resps`, and a vector
  producer must never be given a wakeup port here (see logic part 3).

  One value is READ from the vector parameters to size the new completion ports:
  `numVecClrPorts`, the number of vector completion lanes, default 3 — lane 0 the
  LCB group-done, lane 1 VecCiiComplete, lane 2 VecGroupCopy, in that fixed
  order. It is a LANE COUNT AND NOT AN ARBITER WIDTH; see the ports section for
  why. Note that no node in the map currently declares `numVecClrPorts`;
  hierarchy.yaml's `vec_pipeline_io` uses it as a width and VectorParams should
  declare it, with this default, so the two sides cannot drift.

  ---- THE ELABORATION GATE ----

  Every addition below is conditional on `usingRVV` — a Scala `Boolean` derived
  from `BoomCoreParams.enableVector`, NOT a hardware `Bool` and NOT rocket's
  `usingVector`. With vectors disabled the new ports and the three new state
  items are ABSENT, not tied off, and every `when` condition reverts to the exact
  baseline expression, so the elaborated ROB is the pre-Caracal one. Write the
  qualifications as Scala `if`, never as a hardware mux against a constant, so
  the emitted RTL is textually the baseline rather than something a folder has to
  rescue.

  Widths used, none of them a literal: `robAddrSz`, `numRobRows`, `coreWidth`,
  `maxMembers` and `vecPregSz` (from VectorParams, reached only indirectly
  through `MicroOp`'s fields), and the register-type width from
  ScalarOpConstants.
  <|end_parameters|>

  <|begin_ports|>
  DELTA ONLY. Clock and reset are unchanged and are the Chisel and hierarchy.yaml
  defaults: posedge `clock`, ACTIVE-HIGH SYNCHRONOUS `reset`, single `core_clk`
  domain. The ROB is `BoomModule` and takes the implicit clock/reset as today.

  Three inputs and one output are added to `class RobIo`, all `usingRVV`-gated,
  all matching the `vec_pipeline_io` signals of the same name in hierarchy.yaml
  except the last, which does not cross that seam:

  - `io.vec_clr_bsy`    — `Input(Vec(numVecClrPorts, Valid(UInt(robAddrSz.W))))`.
                          The single-shot busy-clear for a vector destination
                          group, ONE LANE PER PRODUCER: lane 0 the LCB's
                          `group_done` (a vector load, including a segmented
                          load's `pvtmp` half), lane 1 VecCiiComplete's `clr_rob`
                          (a vector arithmetic op, including a segmented store's
                          coprocessor half), lane 2 VecGroupCopy. Each lane is a
                          plain ROB index, NOT a `VecGroupDone`: the member-PRN
                          vector on that bundle is for the Busy Table and the
                          wakeup network, and the ROB has no use for it.
  - `io.vec_clr_unsafe` — `Input(Valid(UInt(robAddrSz.W)))`. The group-safe
                          event: all of this instruction's element addresses have
                          been translated and LCAM-checked. Shaped exactly like
                          the existing `io.lsu_clr_unsafe` element so the two are
                          visibly the same kind of thing, but a single port rather
                          than a `Vec(lsuWidth, ...)`, because a group-safe is per
                          instruction and not per D$ lane, and the LSU is its only
                          producer.
  - `io.vec_rob_flags`  — `Input(Vec(numVecClrPorts, Valid(VecRobFlags)))`, the
                          same lane assignment, where `VecRobFlags` is
                          {`rob_idx`, `fflags`: `FLAGS_SZ`, `vxsat`: Bool}. The
                          CSR side effects of vector arithmetic, accrued per ROB
                          entry and applied at COMMIT (logic part 10). In practice
                          only lane 1 ever fires — the LSU and VecGroupCopy raise
                          no FP flags — but the lane structure mirrors
                          `vec_clr_bsy` so the two are wired from the same index.
  - `io.com_vxsat`      — `Output(Bool())`. Asserted for one cycle when a
                          committing entry had `vxsat` set; wired in core.scala to
                          `csr.io.vector.set_vxsat`, exactly as the existing
                          `io.commit.fflags` is wired to `csr.io.fcsr_flags`. It
                          is a ROB-to-CSRFile signal inside BoomCore and therefore
                          NOT a `vec_pipeline_io` member.

  ===> LANES, NOT AN ARBITER, AND THIS WAS AMENDED FOR A REASON. `vec_clr_bsy`
       began as a single `Valid` with two unarbitrable producers. NEITHER can be
       back-pressured: VecCiiComplete frees its tag in the same cycle it
       completes, so a clear lost to arbitration is UNRECOVERABLE — the entry
       stays busy forever and the machine hangs at the ROB head. A lane per
       producer removes the arbiter and the failure with it. The cost is one
       `MatchBank` compare and one busy-clear write enable per lane per bank,
       which is the same cost the existing `io.lsu_clr_bsy` pays `coreWidth`
       times over.

  ===> `VecRobFlags` IS NAMED BY hierarchy.yaml BUT DECLARED BY NO NODE. It
       crosses a boundary, so it belongs in VecBundles, which is already authored
       and does not declare it. Field list as above; resolve the placement in
       Phase R rather than declaring it a second time here.

  NO OTHER PORT IS ADDED, and each absence is load-bearing:
  - NO vector exception port. A vector memory fault arrives on the EXISTING
    `io.lxcpt` (see logic part 7). `VecBundles.VecException` is merged onto that
    single port upstream, so the age-ordered `IsOlder` selection at
    rob.scala:667-689 stays single-sourced.
  - NO extra `wb_resps` entry, no per-PRN writeback input, no group-done bundle,
    no `vstart` output, no faulting-element-index input, and no VL or `vtype`
    port (logic part 9 reuses `io.commit`, `io.rollback` and the existing
    `io.wb_resps` for all three).
  - NO new pointer output. `io.rob_pnr_idx` AND `io.rob_head_idx` already exist
    and are exported to VecPipeline unchanged — the past-PNR eligibility gate
    needs the head because BOOM's `IsOlder` is 3-arg and a 2-arg comparison
    inverts across a ROB wrap. Both are pre-existing outputs, so this is a wiring
    obligation on BoomCore and not a delta here.
  <|end_ports|>

  <|begin_logic|>
  ---- PART 1. The one field change: `dst_rtype` widens to 3 bits ----

  //@req-spec-core.e23
  //@req-spec-rob.a2
  //@req-spec-rob.a3
  //@req-spec-rob.a4
  Widen `dst_rtype` in `class RobCompactUop` (rob.scala:309) from `UInt(2.W)` to
  3 bits, and update the `compactUopWidth` sum on rob.scala:314 so its
  `dst_rtype` term is 3 instead of 2. `RT_VEC = 4` then fits alongside the
  existing `RT_FIX = 0`, `RT_FLT = 1`, `RT_X = 2` and `RT_ZERO = 3`, which keep
  those exact values, so every existing comparison — including the
  `temp_uop.dst_rtype =/= RT_X` term of the wrong-pdst assert at rob.scala:550 —
  yields the result it does today. This is a value-preserving WIDENING, not a
  re-encoding, and no existing meaning is overloaded.

  Derive the field's width from `MicroOp`'s `dst_rtype` width rather than writing
  a second literal 3. `uop_to_compact`/`compact_to_uop` (rob.scala:315-338) copy
  the field verbatim in both directions, so if the two widths ever disagree the
  compact path silently TRUNCATES a register type — and truncating `RT_VEC = 4`
  yields `RT_FIX = 0`, i.e. a vector entry that commits as an integer one.

  //@req-spec-rob.a1
  //@req-spec-core.e22
  THIS IS THE ONLY FIELD CHANGE, AND THERE IS NO STRUCTURAL CHANGE AT ALL.
  Caracal reuses BOOM v4's ROB as it stands: same banking, same row/bank index
  arithmetic (`GetRowIdx`/`GetBankIdx`), same compact-uop SRAM plus its two-deep
  bypass, same commit bundle, same FSM. No array is resized, no array is added,
  no pointer is added and the entry count is unchanged.

  ---- PART 2. One ROB entry per OP.v, through the scalar path ----

  //@req-spec-core.c6
  //@req-spec-core.c14
  //@req-spec-rob.a8
  //@req-spec-rob.a9
  //@req-spec-rename.b4
  A single OP.v — an entire LMUL/EMUL vector-register group — occupies EXACTLY
  ONE ROB entry, regardless of EMUL, and is not cracked into one uop per
  destination register at decode. The ROB requires nothing for this: dispatch is
  the untouched `io.enq_valids`/`io.enq_uops` path at rob.scala:377-391, one
  entry per uop reserved in program order by Rename. Element expansion into
  nOP.v happens only in the vector LS AGEN and is invisible here — no nOP.v ever
  reaches a ROB port.

  //@req-spec-core.c15
  //@req-spec-rob.a5
  //@req-spec-rob.a6
  //@req-spec-rob.a7
  Vector uops therefore allocate, commit and roll back through the SAME
  head-pointer and exception machinery as scalar ops: the same `rob_tail`
  allocation, the same `can_commit`/`will_commit`/`block_commit` cascade at
  rob.scala:574-581, the same in-order `rob_head` advance, the same
  `s_rollback` unwind. Inter-queue ordering between the vector and scalar
  pipelines is enforced exclusively by this in-order commit, which is why no
  vector queue needs an ordering mechanism of its own (plan section 5 rule 5).

  //@req-spec-core.h5
  //@req-spec-rename.e2
  //@req-spec-rob.d1
  //@req-spec-rob.d2
  //@req-spec-issue.c5
  //@req-spec-issue.c6
  A SHARED instruction (`is_shared`, the segmented load/store forms) is no
  exception to this: it needs two execution resources and gets TWO ISSUE SLOTS —
  one in `IQ_V_ALU` for the coprocessor half and one in `IQ_V_LOAD`/`IQ_V_STORE`
  for the LSU half — but Rename allocates it PRNs and a SINGLE ROB entry exactly
  as for any other instruction, and the two halves share that one entry. The ROB
  has no notion of a half at dispatch; it sees one uop with `is_shared` set.

  ---- PART 3. Vector completion: the single-shot busy-clear ----

  //@req-spec-rob.b1
  //@req-spec-rob.b2
  A ROB entry that owns an EMUL-wide destination group must not clear `rob_bsy`,
  and therefore must not commit, until ALL of that group's writes are done. What
  makes this affordable is where the "all" is evaluated.

  //@req-spec-rob.b3
  //@req-spec-rob.b4
  //@req-spec-rob.b5
  //@req-spec-rob.b6
  //@req-spec-rob.b7
  //@req-spec-rob.c4
  THE ROB SIDE OF VECTOR COMPLETION IS IDENTICAL TO SCALAR: one single-shot
  `rob_bsy` clear per entry. Every vector producer is required to aggregate its
  whole destination group into ONE group-done event rather than streaming per-PRN
  writebacks into the ROB, so the per-PRN counting lives in the producer and NO
  NEW ROB COUNTER EXISTS. Concretely, inside the existing per-bank loop, add one
  block shaped exactly like the `io.lsu_clr_bsy` block at rob.scala:415-423, but
  iterated over the lanes: `for (clr <- io.vec_clr_bsy)`, and when `clr.valid` and
  `MatchBank(GetBankIdx(clr.bits))`, let `cidx = GetRowIdx(clr.bits)` and clear
  `rob_bsy(cidx)` (subject to part 4's one qualification) and `rob_unsafe(cidx)`.
  The lanes are symmetric and the ROB never names a producer: which lane a clear
  arrived on has no effect on anything the ROB does.

  //@req-spec-rob.c3
  //@req-spec-rob.c8
  The vector memory path's element and segment sub-accesses complete OUT OF ORDER
  through the LSU and DO NOT report to the ROB individually — there is no port on
  which they could. For a vector load the Load Coalescing Buffer assembles the
  group and emits the one event when the LAST destination PRN is written. A
  non-shared vector STORE writes no VRF and completes on the existing
  `io.lsu_clr_bsy` once its whole active element set has translated and
  disambiguated. The scalar completion path — `io.wb_resps`, `io.lsu_clr_bsy` and
  their three writeback asserts at rob.scala:544-552 — is left exactly as it is.

  //@req-spec-rob.c2
  A vector ARITHMETIC instruction executes in program order on the in-order tt_CII
  coprocessor, which knows when the whole OP.v has retired and signals completion
  once, on the writeback beat marked `last`. That single wakeup clears `rob_bsy`
  through the same new port, so the ROB needs no CII-specific completion path.
  // An RT_VEC entry has NO iresp writeback: this port is the ONLY thing that can
  // clear its rob_bsy. That is why the port is an addition and not an
  // optimization — without it every vector op with a vector destination hangs at
  // the ROB head, which is the first failure a bring-up test hits.
  A vector op with a SCALAR destination (`vmv.x.s`, `vcpop.m`, `vfirst.m`,
  `vfmv.f.s`) is the reverse case: it clears through the ordinary INT/FP
  `ExeUnitResp` on `io.wb_resps` and its `vec_clr_bsy` is suppressed at the
  source, so exactly one busy-clear reaches each entry.

  Add one assert mirroring the store one: a `vec_clr_bsy` must name an entry that
  is `rob_val` and `rob_bsy`. // A clear landing on an invalid or already-cleared
  // entry means a wrong-path group-done survived its producer's kill logic, which
  // is the failure mode VecBusyTable has no flush port to catch either.

  ---- PART 4. The shared instruction: ONE BIT, NOT A COUNTER ----

  //@req-spec-rob.d3
  //@req-spec-rob.d4
  //@req-spec-rob.d5
  //@req-spec-issue.c8
  The shared instruction is THE ONLY case in which a ROB entry waits for more
  than one completion, and it needs exactly a 1-BIT "OTHER HALF PENDING" FLAG,
  NOT A COUNTER. Add one per-bank register array `rob_other_half =
  Reg(Vec(numRobRows, Bool()))` beside `rob_bsy`, elaborated only under
  `usingRVV`. One bit is sufficient because the number of completions is two, and
  it is fixed by the instruction form rather than by EMUL.

  //@req-spec-rob.d19
  //@req-spec-rob.d20
  THE FLAG IS SET AT DISPATCH, from the Decoder's `is_shared` on the incoming
  uop, in the existing `when (io.enq_valids(w))` block:
  `rob_other_half(rob_tail) := io.enq_uops(w).is_shared`. It is CLEARED WHEN THE
  SECOND COMPLETION ARRIVES — implemented as: the FIRST completion to reach the
  entry clears the flag and does not clear `rob_bsy`; a completion reaching an
  entry whose flag is already clear clears `rob_bsy` as normal.

  //@req-spec-rob.d15
  //@req-spec-issue.c7
  //@req-spec-cii.i2
  So a shared entry stays busy until BOTH halves have reported, and clears
  `rob_bsy` only on the second event. The qualification is one term added to the
  busy-clear in the two blocks a shared instruction can complete through — the
  new `vec_clr_bsy` block and the existing `io.lsu_clr_bsy` block — of the form
  "clear `rob_bsy` only if `!rob_other_half(cidx)`, else clear
  `rob_other_half(cidx)`". Under `usingRVV = false` that term is a Scala-level
  `true.B` and both blocks are textually the baseline again.
  // Which event each half emits depends on whether it writes the VRF: a
  // segmented LOAD's LSU half writes pvtmp and emits a real group-done
  // (vec_clr_bsy); a segmented STORE's LSU half writes no VRF and signals
  // lsu_clr_bsy, deferred until after DGEN has read pvtmp. The coprocessor half
  // always emits a group-done. The flag is indifferent to which port an arrival
  // came in on, which is precisely why one bit suffices for both forms.

  //@req-spec-core.h7
  //@req-spec-rob.d7
  //@req-spec-rob.d8
  //@req-spec-rob.d14
  //@req-spec-rob.d21
  THE TWO HALVES ALWAYS COMPLETE IN A FIXED ORDER, PRODUCER HALF THEN CONSUMER
  HALF, and the two events CAN NEVER LAND IN THE SAME CYCLE. The halves hand off
  through `pvtmp`: the consumer half is woken BY the producer's `pvtmp`
  group-done, so it issues and executes strictly afterwards. For a segmented LOAD
  the order is LSU then CII; for a segmented STORE it is CII then LSU (the
  six-step chain in issue.rst `shared-store-chain`). The flag is therefore
  equivalent to "wait for the consumer half", and no same-cycle arbitration case
  needs handling — do not write one.
  // If both arrivals COULD coincide, one bit would be insufficient and this
  // would need a counter. The absence of that case is a property of the pvtmp
  // handoff, not an assumption about latency, so ASSERT IT: at most one
  // completion event — across all `vec_clr_bsy` lanes and `io.lsu_clr_bsy`
  // together — may name a given entry in a cycle. The assert is not decoration.
  // Two arrivals in one cycle both read the OLD `rob_other_half` register, so
  // both take the flag-clear branch, `rob_bsy` is never cleared, and the entry
  // hangs at the ROB head — a silent deadlock that only this assert localises.

  The flag needs NO flush or rollback path. It is written unconditionally at
  every dispatch into the row, exactly like `rob_bsy` and `rob_unsafe`, so a
  stale value cannot survive into a reallocated entry, and while `rob_val` is
  false its value is don't-care. Adding it to the `s_rollback` clearing loop at
  rob.scala:473-478 would be dead logic.

  ---- PART 5. `rob_unsafe`, group-safe, and the PNR ----

  //@req-spec-rob.e4
  `rob_unsafe` IS CLEARED BY ONE GROUP-SAFE EVENT AND NEVER PER SUB-ACCESS. A
  multi-access vector load or store is not memory-safe until ALL of its element
  addresses have been LCAM-checked, so the LSU reports a single group-safe on
  `io.vec_clr_unsafe` when the LAST element address has been checked, and the ROB
  clears `rob_unsafe(GetRowIdx(...))` on it in a block shaped like
  `io.lsu_clr_unsafe` (rob.scala:424-429). A per-sub-access clear would declare
  the instruction safe while later elements could still fault or alias.
  // This is also what keeps the exception assert at rob.scala:436-442 true for
  // vector ops: the element cursor STOPS at a fault, so the last element is never
  // checked and group-safe never fires, so `rob_unsafe` is still set when the
  // lxcpt arrives. Requirement on the producer, stated here because the assert
  // lives here: never raise vec_clr_unsafe for a group whose element stream
  // faulted. Even in the degenerate case where the LAST element faults, the
  // assert reads the pre-clear register value and holds.

  //@req-spec-rob.e6
  //@req-spec-issue.d16
  //@req-spec-issue.d17
  //@req-spec-issue.d5
  A SHARED entry carries ONE `rob_unsafe` bit and needs ONLY THE LSU HALF's
  group-safe to clear it; nothing waits on the coprocessor half, which performs
  no memory access and so carries no speculation hazard. The PNR advances past a
  segmented store's entry as soon as that clear arrives (step 3 of the six-step
  chain), which is what lets the coprocessor half become PNR-eligible at all.
  Requiring a second group-safe from the coprocessor half WOULD DEADLOCK: that
  half cannot issue until the PNR has passed the entry, which needs `rob_unsafe`
  already cleared.
  // issue.rst says "first address translation" where midcore.rst says "the last
  // element address has been LCAM-checked". The distinction issue.rst is drawing
  // is WHICH HALF, not which element; for a vector op midcore.rst governs, since
  // spec-rob.e4 forbids a per-sub-access clear.

  //@req-spec-issue.d18
  //@req-spec-core.f7
  THE PNR LOGIC ITSELF IS NOT TOUCHED — neither the `enableFastPNR` branch nor
  the incremental one, nor `rob_unsafe_masked`, nor `rob_pnr_unsafe`, nor either
  pointer-ordering assert. The PNR tracks the resolution of SPECULATION, not
  completion, so an entry with a half still un-issued is treated no differently
  from any other safe entry — completion is what part 4's flag tracks,
  separately. `starts_unsafe` is likewise unmodified: `is_br`/`is_jalr` still set
  it, so the PNR still never sweeps past an unresolved branch, and a vector
  load/store still sets it through `uses_ldq`/`uses_stq` because it holds one
  LDQ/STQ placeholder entry. A vector ARITHMETIC op sets neither and is safe from
  dispatch, which is correct: it accesses no memory.

  ---- PART 6. Commit: the whole stale destination group ----

  //@req-spec-rob.f1
  //@req-spec-rob.f2
  //@req-spec-rob.f4
  At retirement a vector entry frees its ENTIRE stale destination group — EMUL
  stale vector pregs, not one — and the ROB is the structure that carries them:
  the EMUL stale PRNs captured at rename ride in the committing uop's
  `stale_pvdest` field, alongside the new `pvdest`, and both are presented on
  `io.commit.uops(w)` at retirement. The actual deallocation is VecFreeList's
  commit dealloc bank, driven from that bundle, exactly as the scalar
  `RenameFreeList`'s `com_deallocs` is driven from `stale_pdst`.

  //@req-spec-rob.f3
  //@req-spec-rob.f5
  //@req-spec-rob.f6
  THE STALE GROUP IS EXPLICIT, NOT A BASE PLUS COUNT: every ROB entry provides
  `MAX_MEMBERS` = 8 stale-PRN slots, of which a vector entry uses EMUL (per
  `v_emul`) and leaves the remainder DON'T-CARE, while a scalar entry uses the
  one existing `stale_pdst`. An explicit vector is required because the vector
  free list allocates a group WITHOUT requiring contiguous PRNs, so a base+count
  encoding could not name the group that was actually allocated.

  This costs the ROB NO new storage and NO new port. The slots already exist:
  `rob_uop` (rob.scala:364) is `Reg(Vec(numRobRows, new MicroOp()))` and holds
  the whole uop, whose `stale_pvdest` is the `Vec(maxMembers, UInt(vecPregSz.W))`
  the MicroOp delta declares; `compact_to_uop` overrides only its eight listed
  fields, so `stale_pvdest`, `pvdest`, `pvtmp` and `v_emul` reach
  `io.commit.uops(w)` straight out of `rob_uop(rob_head)`.
  // DELIBERATE NON-GOAL: do NOT move stale_pvdest into the compact SRAM to
  // recover the ~56 bits/entry. That means widening compactUopWidth and touching
  // the two-deep bypass at rob.scala:344-353 — the highest-risk lines in the file
  // for a saving midcore.rst `vec-commit` already prices in and accepts.

  ---- PART 7. Precise vector exceptions: `vstart = 0`, restart whole ----

  //@req-spec-rob.g1
  //@req-spec-rob.g12
  Because the whole vector instruction is one ROB entry, its trap must be
  RESTARTABLE, and Caracal makes it so the simplest correct way. The LSU
  exception port reports a vector fault as A PLAIN PRECISE EXCEPTION, LIKE A
  SCALAR LOAD: it arrives on the existing `io.lxcpt` as {uop, cause, badvaddr},
  is recorded by the untouched block at rob.scala:436-443, and is thrown at the
  head through the untouched `can_throw_exception`/`io.com_xcpt` path with the
  OP.v's own `ftq_idx`/`pc_lob`, so the handler returns to the instruction rather
  than into it. NO ROB LOGIC IS ADDED FOR VECTOR EXCEPTIONS AT ALL.

  //@req-spec-rob.g2
  //@req-spec-rob.g3
  //@req-spec-rob.g4
  //@req-spec-rob.g10
  //@req-spec-rob.g11
  //@req-spec-lsu.f10
  //@req-spec-lsu.f11
  //@req-spec-lsu.f12
  //@req-spec-core.i10
  A FAULTING VECTOR LOAD/STORE TRAPS WITH `vstart = 0` AND RESTARTS THE WHOLE
  INSTRUCTION FROM THE BEGINNING; IT MUST NOT RESUME AT `vstart = k`. The
  faulting-element index is NOT carried to the ROB and NOT written to `vstart`,
  and this delta is what guarantees it: no port carries an element index (see the
  ports section), `VecBundles.VecException` declares no such field, and the ROB
  has no write path to `vstart` — which is rocket's `CSRFile` state, not
  Caracal's. `fault_elem` survives only as the element cursor's stop signal and a
  debug counter, inside the LSU.
  // WHY vstart = k IS A SILENT-CORRUPTION BUG, not a lost optimization: elements
  // 0..k-1 were written into pvdest, a FRESHLY renamed group. The instruction
  // never commits, so pvdest is never installed in com_map_table and architectural
  // vd still maps to stale_pvdest — those k elements are GONE. Resuming at k
  // re-executes only k..VL-1 and leaves 0..k-1 holding pre-instruction values.
  // Any reviewer seeing an element index appear on a ROB port should reject it.

  //@req-spec-core.i9
  //@req-spec-rob.g5
  //@req-spec-rob.g6
  //@req-spec-rob.g9
  NO PARTIAL RESULT OF AN OP.v IS EVER ARCHITECTURALLY VISIBLE, and the existing
  machinery is what makes that true. A faulting instruction NEVER COMMITS —
  `will_commit(w)` is already gated by `!can_throw_exception(w)` at
  rob.scala:577 — so `io.commit.valids(w)` never rises for it, so its `pvdest` is
  NEVER installed in `com_map_table` (the commit remap requests are qualified by
  the commit valids, on both the scalar and the vector mapper). Vector stores
  drain to memory only POST-COMMIT, so a faulting store has touched no memory
  either; loads are idempotent. Together, `vstart = 0` is a legal trap value.

  //@req-spec-rob.g7
  //@req-spec-rob.g8
  The exception path RESTORES `map_table := com_map_table` AND RETURNS THE WHOLE
  DESTINATION GROUP TO THE FREE LIST, and the ROB's entire contribution is the
  EXISTING `io.rollback` output (`rob_state === s_rollback`, rob.scala:624) plus
  `io.flush`. VecMapTable's restore and VecFreeList's rollback-dealloc are wired
  from it exactly as `rename-maptable.scala:120-121` and
  `rename-freelist.scala:79` wire the scalar ones. No new output, no vector
  special case, and no change to the two-cycle `exception_thrown` to `s_rollback`
  sequencing in the FSM.

  //@req-spec-rob.h1
  //@req-spec-rob.h2
  //@req-spec-rob.h3
  //@req-spec-rob.h4
  A FAULTING SHARED INSTRUCTION NEEDS NO SPECIAL `pvtmp` CLEANUP. Both `pvdest`
  and `pvtmp` are uncommitted physical groups and a segmented store's memory
  writes are post-commit, so a faulting segmented load/store COMMITS NO PARTIAL
  DATA and leaves no architectural trace; `pvtmp` is reclaimed by the same
  standard free-list flush/rollback path as any uncommitted allocation. The ROB
  is not `pvtmp`-aware anywhere — it carries the field on the uop and does
  nothing with it — and its one-bit flag needs no cleanup either (part 4).

  ---- PART 8. The ordering-violation `lxcpt`: record, defer to the head ----

  //@req-spec-memord.c7
  //@req-spec-memord.c8
  //@req-spec-memord.c9
  When the LSU broadcasts the oldest failing load as an `lxcpt` with cause
  `MINI_EXCEPTION_MEM_ORDERING`, THE ROB RECORDS IT AS AN EXCEPTION ON THAT
  LOAD'S ROW AND TAKES NO ACTION WHEN IT IS REPORTED: rob.scala:436-437 sets
  `rob_exception(GetRowIdx(...))` and nothing else fires. THE FLUSH FIRES ONLY
  WHEN THE FAILING LOAD REACHES THE ROB HEAD, through
  `can_throw_exception(w) := rob_val(rob_head) && rob_exception(rob_head)`, which
  is what makes the replay precise and what lets in-order commit guarantee the
  younger work woken on bad data never reaches architectural state. All of this
  is BASELINE BEHAVIOUR AND IS NOT MODIFIED — including the deliberate
  `cause =/= MINI_EXCEPTION_MEM_ORDERING` exemption in the `rob_unsafe` assert,
  which exists because a failing load has already been marked safe.
  A vector load that order-fails is the same case at whole-instruction
  granularity: its single LDQ placeholder drives ONE `lxcpt`, the whole OP.v is
  refetched and re-renamed, and there is no partial-group rewind — consistent
  with the one-group-done completion model.

  ---- PART 9. Commit-time architectural vector state ----

  //@req-spec-decode.h5
  THE COMMITTED VCFG SHADOW IS UPDATED ONLY BY THE ROB, WHEN A `vset*` uOP
  COMMITS. The ROB is the sole owner of that retire event, and it supplies it
  through `io.commit.valids` and `io.commit.uops` (from which VConfigUnit derives
  its `com_valids`, `com_is_vset` and `com_vtype` inputs) plus `io.rollback`, all
  crossing on `vec_pipeline_io`'s `commit_valids`/`commit_uops`/
  `commit_rollback`. It is the same retire event that writes the architectural
  `vtype` CSR, so the shadow and the CSR cannot disagree. The ROB arbitrates
  nothing: when several `vset*` uops commit in one cycle the newest lane wins,
  and that selection is VConfigUnit's, on the receiving side.

  ---- THE ONE NEW STATE ITEM PART 9 NEEDS: the executed-`vtype` latch ----

  A ROB entry captures its uop at DISPATCH, and for `vsetvl` — whose `vtype`
  comes from a GPR — the decode-time `uop.vconfig` snapshot CANNOT hold the value
  the instruction installs, because it is not known until execute. ALUUnit's delta
  resolves the new `VType` and OVERWRITES `io.resp.bits.uop.vconfig` on its
  writeback (chosen over widening `ExeUnitResp`, which every execution unit
  shares). So the ROB must LATCH IT:

    - add a narrow per-bank array `rob_vconfig = Reg(Vec(numRobRows, VType))`,
      written at dispatch from `io.enq_uops(w).vconfig`;
    - in the EXISTING `io.wb_resps` loop, when the responding uop is a `vset*`
      (test its `is_vl_producer`), also write
      `rob_vconfig(row_idx) := wb_resp.bits.uop.vconfig`;
    - at commit, override that one field of the commit uop:
      `io.commit.uops(w).vconfig := rob_vconfig(rob_head)`, using the same
      override pattern the `debug_fsrc`/`taken` mispredict fix-up already uses at
      rob.scala:466-471.

  `com_vtype` and the architectural `vtype` CSR write are then BOTH sourced from
  that latched value — one value, two consumers, so the committed shadow and the
  CSR cannot disagree, which is what VConfigUnit requires of `com_vtype`.
  ALUUnit must drive the resolved `vconfig` on EVERY `vset*` writeback, not only
  `vsetvl`, so that the ROB needs no per-form case; for the immediate forms it is
  the same value the decode snapshot already held.
  // WHY A SEPARATE 9-BIT ARRAY AND NOT A WRITE INTO `rob_uop`. `rob_uop` is a
  // Reg(Vec(numRobRows, MicroOp)) with two writers today (dispatch, brupdate).
  // Adding a writeback-indexed writer would add numWakeupPorts write enables to
  // the WIDEST register array in the ROB. A dedicated VType-wide array is the
  // same function at a fraction of the cost, and it is also why relying on
  // `vsetvl`'s `is_unique` to get away with a single register was rejected:
  // `vsetvli`/`vsetivli` are NOT is_unique, so several can be in flight, and one
  // uniform per-row rule is cheaper to review than two form-dependent paths.
  ===> NO REQUIREMENT IN THIS NODE'S ALLOCATION COVERS THIS LATCH. It is a corpus
       gap: `spec-decode.c11` puts the computation in the ALU and `spec-decode.h5`
       puts the commit write in the ROB, and nothing states how the value gets
       from one to the other. Specified here because omitting it makes `vsetvl`
       commit the wrong `vtype`; flagged for Phase R rather than tagged.

  //@req-spec-rename.h21
  AT COMMIT OF A VL PRODUCER THE ARCHITECTURAL `vl` CSR IS WRITTEN PRECISELY, and
  again the ROB's contribution is the commit event alone. The committing uop
  carries `is_vl_producer` and `pvl`; the youngest committing VL producer's `pvl`
  drives VlRegFile's SINGLE COMMIT READ PORT, whose data becomes
  `csr.io.vector.set_vconfig.bits.vl`. Precision follows from properties the ROB
  already guarantees and this delta must not weaken: commit is in program order,
  a uop appears on `io.commit.valids` at most once, and it appears only when it
  is architecturally retiring — so the CSR write can neither be speculative nor
  duplicated. The ROB adds no VL storage, no VL port and no `vl` datapath.

  ---- PART 10. `fflags` and `vxsat`: accrue per entry, apply at COMMIT ----

  Vector arithmetic has two CSR side effects and BOTH ARE APPLIED AT COMMIT, NOT
  AT WRITEBACK. VecCiiComplete accumulates `wb_status`'s five FP flags and the
  sticky `vxsat` per tag and emits them once, on the `last` beat, as
  `io.vec_rob_flags`. The ROB holds them per entry until retirement:

    - `fflags` goes into the EXISTING per-bank `rob_fflags` Valid array — the same
      slot an FP writeback fills — so it reaches `csr.io.fcsr_flags` through
      BOOM's existing commit path (`fflags_val`/`io.commit.fflags`,
      rob.scala:630-654) with no new commit output;
    - `vxsat` needs a new 1-bit per-bank array `rob_vxsat = Reg(Vec(numRobRows,
      Bool()))`, cleared at dispatch and set by the flags port, read at the head
      and OR-reduced across retiring lanes onto the new `io.com_vxsat` output.
      It cannot ride in `rob_fflags`, which is `FLAGS_SZ` = 5 bits wide and holds
      FP flags only; `vxsat` is a fixed-point saturation bit and a different CSR.

  ===> WRITE `rob_fflags` EXACTLY ONCE PER ENTRY, OR rob.scala:405 ASSERTS.
       `assert(!rob_fflags(row_idx).valid)` guards the FP writeback path against a
       second write, and the new flags block must carry the same assert. Two
       seam obligations follow, both on the CII side: a CII SCALAR-destination
       writeback must drive `ExeUnitResp.fflags` INVALID (its flags come in on
       `vec_rob_flags` instead, so leaving them on the `ExeUnitResp` is the second
       write), and VecCiiComplete must emit at most one `vec_rob_flags` per tag.

  ===> AND WHY NOT AT WRITEBACK: pulsing `csr.io.vector.set_vxsat` when the
       coprocessor completes is NOT FLUSH-SAFE. A CII op is issued past the PNR,
       and past-PNR is not a commit guarantee (rob.scala:436-442) — an
       order-fail or an older exception can still flush it at the ROB head. A
       sticky CSR bit set by a squashed instruction can never be un-set. Accruing
       in the entry and applying in ROB order at commit is what makes both flags
       correct under a flush, and it is the same discipline `fflags` has always
       had in BOOM.

  One EXISTING assert must be given a `usingRVV`-gated exemption term, and it is
  the only assert this delta weakens: "Committed non-FP instruction has non-zero
  fflag bits" (rob.scala:643-646) fires when a vector FP op accrues flags without
  setting `fp_val`. Exempt a committing uop with `is_vec` set. Vector ops must NOT
  set `fp_val` — that would route them into FP writeback accounting they do not
  use — so the companion assert "Committed FP instruction did not set fflag bits"
  (rob.scala:637-641) is left exactly as it is.

  ---- PART 11. Trace ----

  With no unit tests in this project, add guarded trace lines via the shared
  `VecTrace` package, gated on the `vecTrace` plusarg (off by default) and on
  `!reset`, tagged with module name `Rob` and `rob_idx`: one on a `vec_clr_bsy`
  arrival (with whether it cleared `rob_bsy` or only the other-half flag), one on
  a `vec_clr_unsafe` arrival, and one at commit of an entry whose `dst_rtype` is
  `RT_VEC` (with `v_emul`, the latched `vconfig` and `vxsat`). Those three lines
  are what distinguish "the group never completed" from "one half never reported"
  from "the entry never reached the head" — the three ways this delta can hang the
  machine. Tracing declares no state; deleting every call site leaves the design
  bit-identical.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
This delta must be free. The ROB is a commit-critical structure with
`numRobEntries` copies of everything it holds, and every constraint below is a
constraint on the implementation rather than a preference.

- NO CHANGE TO THE COMMIT CRITICAL PATH. The one term added to `rob_bsy`'s clear
  is `!rob_other_half(cidx)`, read from a register array indexed by the same
  `GetRowIdx` the block already computes. `can_commit` (rob.scala:454) reads
  `rob_bsy(rob_head)` as it does today; nothing new appears between a completion
  and the commit decision.
- AREA: `numVecClrPorts` busy-clear lanes and as many flags lanes, one
  `vec_clr_unsafe` input, `2 * numRobRows` flops per bank for `rob_other_half`
  and `rob_vxsat`, a `VType`-wide `rob_vconfig` array per bank, and one bit per
  bank per compact-SRAM row for the widened `dst_rtype`. The stale-PRN slots are
  `maxMembers * vecPregSz` bits per entry (~56 at the defaults) inside `rob_uop`,
  which midcore.rst `vec-commit` prices and accepts as the cost of freeing a
  group with no second structure.
- ONE `vec_clr_unsafe` PORT IS ENOUGH AT EVERY TIER: group-safe is per
  instruction, not per element and not per D$ lane, and the LSU is its only
  producer. `vec_clr_bsy` and `vec_rob_flags` are sized by PRODUCER COUNT and not
  by a throughput target — do not add a lane for bandwidth, and do not remove one
  by arbitrating, since a lost clear is unrecoverable.
- `rob_vconfig` IS READ ONLY AT THE HEAD and written only at dispatch and at a
  `vset*` writeback, so it adds nothing to the completion or commit critical
  paths; it is a narrow array precisely so it does not become a third writer of
  the much wider `rob_uop`.
- With `usingRVV = false` the ROB's added logic is ABSENT and its timing and area
  are the baseline's, with the single exception recorded in the edit_scope
  section: the unconditional `dst_rtype` widening inherited from the MicroOp and
  ScalarOpConstants deltas.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the ROB reads `is_shared` at dispatch and `is_vl_producer` on a
writeback, and carries `dst_rtype`, `stale_pvdest`, `pvdest`, `pvtmp`, `v_emul`,
`is_vl_producer`, `pvl` and `vconfig` through `rob_uop`/`rob_vconfig` to
`io.commit.uops`. This delta adds no field to that bundle; every field it uses is
declared by the MicroOp delta. It also binds by name to
`freechips.rocketchip.rocket.VType`, the type of `rob_vconfig`, for the same
reason MicroOp's `vconfig` does — rocket's `CSRFile` owns architectural `vtype`.
ScalarOpConstants — `RT_VEC` and the 3-bit register-type width.
VecTrace — the guarded trace helper in part 10. Emit-only, no state.

Instantiates nothing, and adds no instance: `hierarchy.yaml` lists no
`instantiates:` for this node.

Seam-mates, none of which is a compile dependency:
- VecPipeline / BoomCore drive `io.vec_clr_bsy`, `io.vec_clr_unsafe` and
  `io.vec_rob_flags`, merge `VecException` onto the existing `io.lxcpt`, take
  `io.com_vxsat` to `csr.io.vector.set_vxsat`, and consume `io.commit`,
  `io.rollback`, `io.rob_pnr_idx`, `io.rob_head_idx`, `io.flush` and `io.empty`
  unchanged.
- VecLoadCoalescingBuffer's `group_done` (lane 0), VecCiiComplete's `clr_rob`
  (lane 1) and VecGroupCopy (lane 2) are the three sources of `vec_clr_bsy`;
  VecCiiComplete's `rob_flags` is the only expected source of `vec_rob_flags`,
  and its `group_done` is the coprocessor half of a shared instruction.
- ALUUnit overwrites `ExeUnitResp.uop.vconfig` with the resolved `VType` on a
  `vset*` writeback; `rob_vconfig` latches it. That is the seam that makes
  `vsetvl` commit the executed configuration rather than the decode snapshot.
- LSU's group-safe drives `vec_clr_unsafe` and its vector `lxcpt` drives the
  existing exception port; a segmented store's LSU half completes on the existing
  `lsu_clr_bsy`.
- VConfigUnit takes its commit inputs and `rollback` from `io.commit`/
  `io.rollback`; VlRegFile's single commit read is addressed with the youngest
  committing VL producer's `pvl` off the same bundle.
- VecMapTable and VecFreeList consume `io.rollback` for the exception restore and
  the group return, and `io.commit.uops`' `stale_pvdest` for the commit dealloc.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File   src/main/scala/v4/exu/rob.scala
    Classes `class RobIo(numWakeupPorts: Int)` and
            `class Rob(numWakeupPorts: Int, usingTrace: Boolean)`
    (package boom.v4.exu). Hand-written baseline BOOM v4.

  In scope — the edit may touch ONLY these, and nothing else in the file:
    - `class RobIo`: add the `usingRVV`-gated ports
      `vec_clr_bsy   : Input(Vec(numVecClrPorts, Valid(UInt(robAddrSz.W))))`,
      `vec_clr_unsafe: Input(Valid(UInt(robAddrSz.W)))`,
      `vec_rob_flags : Input(Vec(numVecClrPorts, Valid(VecRobFlags)))`,
      `com_vxsat     : Output(Bool())`.
    - `class RobCompactUop` (rob.scala:309): widen `dst_rtype` to 3 bits, and the
      matching `dst_rtype` term of `compactUopWidth` (rob.scala:314) from 2 to 3.
    - Inside the per-bank `for (w <- 0 until coreWidth)` loop only:
        * declare `rob_other_half = Reg(Vec(numRobRows, Bool()))`,
          `rob_vxsat = Reg(Vec(numRobRows, Bool()))` and
          `rob_vconfig = Reg(Vec(numRobRows, VType))`, all `usingRVV` only;
        * in the existing `when (io.enq_valids(w))` block, three added
          assignments: `rob_other_half(rob_tail) := io.enq_uops(w).is_shared`,
          `rob_vxsat(rob_tail) := false.B`,
          `rob_vconfig(rob_tail) := io.enq_uops(w).vconfig`;
        * in the EXISTING `io.wb_resps` loop, one added conditional write
          `rob_vconfig(row_idx) := wb_resp.bits.uop.vconfig` qualified by that
          uop's `is_vl_producer`. NOTHING ELSE in that loop may change;
        * one new `when` block over the `io.vec_clr_bsy` lanes, shaped like the
          `io.lsu_clr_bsy` block at rob.scala:415-423, plus its `rob_val`/
          `rob_bsy` assert and the one-completion-per-entry-per-cycle assert;
        * one new `when` block for `io.vec_clr_unsafe`, shaped like the
          `io.lsu_clr_unsafe` block at rob.scala:424-429;
        * one new `when` block over the `io.vec_rob_flags` lanes, writing
          `rob_fflags(row)` (with the rob.scala:405-style single-write assert) and
          `rob_vxsat(row)`;
        * one added `!rob_other_half(cidx)` qualification on the `rob_bsy` clear
          in the existing `io.lsu_clr_bsy` block and in the new `vec_clr_bsy`
          block, with the flag-clear in the `else` case;
        * one added commit-uop field override
          `io.commit.uops(w).vconfig := rob_vconfig(rob_head)`.
    - At bank scope: the OR-reduction of the retiring lanes' `rob_vxsat` onto
      `io.com_vxsat`, alongside the existing `io.commit.fflags` reduction.
    - One `usingRVV`-gated `!is_vec` exemption term on the "Committed non-FP
      instruction has non-zero fflag bits" assert at rob.scala:643-646, and
      nothing else in that block.
    - The three `VecTrace` call sites and the new asserts named in the logic
      section.
    - Whatever `import` the `VecTrace` helper and rocket's `VType` need.

  Must not regress — these stay BIT- AND CYCLE-IDENTICAL, and they are named
  because they are what a plausible edit would damage:
    - THE COMPLETION PATH FOR SCALAR OPS. `io.wb_resps` and its loop
      (rob.scala:396-412), including the `rob_bsy`/`rob_unsafe`/`rob_predicated`
      and `rob_fflags` writes and the `assert(!rob_fflags(row_idx).valid)`; the
      `io.lsu_clr_bsy` block's `rob_unsafe` clear and BOTH of its asserts;
      `io.lsu_clr_unsafe`; the three writeback asserts at rob.scala:544-552;
      `rob_debug_wdata` and `io.commit.debug_wdata`. The ONLY changes to any of
      these are the single `!rob_other_half` term on one `rob_bsy` clear and the
      `is_vl_producer`-qualified `rob_vconfig` write in the writeback loop, both
      of which vanish when `usingRVV` is false.
    - `numWakeupPorts` and the shape of `io.wb_resps`. Vector completion must NOT
      be routed through a wakeup port.
    - THE PNR BLOCK IN ITS ENTIRETY: both the `enableFastPNR` and the incremental
      branch, `rob_unsafe_masked`, `rob_pnr_unsafe`, `rob_pnr`/`rob_pnr_lsb`, and
      both pointer-ordering asserts at rob.scala:770-773.
    - THE EXCEPTION AND FLUSH LOGIC IN ITS ENTIRETY: the `io.lxcpt` and
      `io.csr_replay` recording blocks and the `MINI_EXCEPTION_MEM_ORDERING`
      exemption in the `rob_unsafe` assert; `can_throw_exception`;
      `r_xcpt_val`/`r_xcpt_uop`/`r_xcpt_badvaddr` and the age-ordered `IsOlder`
      selection at rob.scala:667-689; `exception_thrown`, `io.com_xcpt`,
      `io.flush`, `FlushTypes.getType`, `insn_sys_pc2epc`, `refetch_inst`,
      `flush_commit_mask`, `io.flush_frontend`.
    - THE COMMIT LOGIC AND BUNDLE: `can_commit`, `will_commit`, `block_commit`,
      `block_xcpt`, `io.commit.valids`/`arch_valids`/`fflags`/`debug_insts`,
      `finished_committing_row`, `r_partial_row`, the `rob_head` advance,
      `io.com_load_is_at_rob_head`, `io.rollback`, `io.empty`, `io.ready`,
      `io.rob_head_idx`/`rob_tail_idx`/`rob_pnr_idx`. `class CommitSignals` gains
      NO field — the `vconfig` override rewrites one existing field of
      `io.commit.uops(w)`, and `vxsat` leaves on its own output rather than being
      folded into this widely-shared bundle.
    - THE FSM: `s_reset`/`s_normal`/`s_wait_till_empty`/`s_rollback`, the
      two-cycle `RegNext(RegNext(exception_thrown))` sequencing, the `is_unique`
      transition, and the `s_rollback` clearing loop at rob.scala:473-478 (which
      must NOT gain the new flag).
    - `RobCompactUop`'s other seven fields, `uop_to_compact`/`compact_to_uop`,
      the SRAM write/read and the two-deep `rob_compact_uop_bypassed` bypass at
      rob.scala:344-353. `compactUopWidth` changes by exactly one bit.
    - The branch-kill blocks (`brupdate_b2_rob_*`, the `rob_val(i) := false.B`
      loop, the `debug_fsrc`/`taken` updates) and `rob_fflags` at bank scope.
    - The FP-exception block at rob.scala:630-654: `fflags_val`, `fflags`,
      `io.commit.fflags`, and the "Committed FP instruction did not set fflag
      bits" and "Committed FP load or store has non-zero fflag bits" asserts. The
      ONE permitted change in that block is the `!is_vec` exemption term on the
      "Committed non-FP instruction has non-zero fflag bits" assert.
    - `usingTrace`, `rob_debug_inst_mem`, `DebugRobBundle`, `debug_entry`,
      `toString`.
    - Entry count, row count, banking and index arithmetic are unchanged; no
      array is resized except the one-bit compact `dst_rtype`.
    - KNOWN, ACCEPTED DEVIATION FROM `usingRVV = false` BIT-IDENTITY, recorded so
      gate (f) is not surprised by it: the `dst_rtype` widening is UNCONDITIONAL,
      because the MicroOp and ScalarOpConstants deltas widen the register-type
      encoding unconditionally and the compact field must track MicroOp's width
      or silently truncate. A vectors-off build therefore differs from
      pre-Caracal BOOM by one bit per bank in the compact SRAM and one bit per
      entry in `rob_uop`. Everything else in this file is absent when
      `usingRVV = false`.

  Interface delta:
    NEW ports (all `usingRVV`-gated, all on `class RobIo`):
      vec_clr_bsy      : Input(Vec(numVecClrPorts, Valid(UInt(robAddrSz.W))))
      vec_clr_unsafe   : Input(Valid(UInt(robAddrSz.W)))
      vec_rob_flags    : Input(Vec(numVecClrPorts, Valid(VecRobFlags)))
      com_vxsat        : Output(Bool())
    WIDENED field:
      RobCompactUop.dst_rtype : UInt(2.W) -> 3 bits (tracking MicroOp's width),
      with `compactUopWidth` adjusted by +1.
    NEW state, all per bank and `usingRVV` only:
      rob_other_half : Reg(Vec(numRobRows, Bool()))
      rob_vxsat      : Reg(Vec(numRobRows, Bool()))
      rob_vconfig    : Reg(Vec(numRobRows, VType))
    UNCHANGED parameters: `numWakeupPorts`, `usingTrace`.

    Explicitly NOT added, and a reviewer should REJECT any of these on sight:
      - any per-entry group completion counter, member-done bitmap, or
        `members_target`-style field;
      - a `VecGroupDone` bundle on any ROB port (the member-PRN vector belongs to
        the Busy Table and the wakeup network, not here);
      - a vector exception port, a faulting-element-index input, or any `vstart`
        output or write path;
      - an ARBITER in front of `vec_clr_bsy`, or a reduction of it to a single
        `Valid` — a clear lost to arbitration is unrecoverable because the CII
        frees its tag in the same cycle;
      - an extra `wb_resps` entry, or any per-PRN writeback input;
      - a `vl` or `pvl` port or register, or any `vtype` port (part 9 latches
        `vconfig` off the EXISTING `io.wb_resps` and reuses `io.commit` and
        `io.rollback`);
      - a `vxsat` field inside `rob_fflags`, or a `set_vxsat` pulse driven from
        the writeback rather than from commit;
      - a new field on `class CommitSignals`;
      - a VLEN-wide `rob_debug_wdata`;
      - a writeback-indexed write into `rob_uop` (that is what `rob_vconfig`
        exists to avoid).
<|end_edit_scope|>

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
  VecSquashUnit — the one module that decides what the vector LSU throws away on
  a branch mispredict or a ROB-head flush, and how.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecSquashUnit.scala,
  package boom.v4.vec.generated.lsu. group vec_lsu.
  depends_on MicroOp, VecBundles, VecTrace. Instantiated once, as `squash`,
  inside VecLsu. Instantiates nothing. RTL lands in plan step E8.

  It is a POLICY module: it holds essentially no state. It converts BOOM's
  existing recovery events into three commands — a tail rollback for each of the
  six VecElemQueue instances, an LCB assembly-entry invalidation mask keyed on
  `ldq_idx`, and a resolved per-client crack kill — and owns the age comparison
  so that no other vector-LSU module has to.

  ===> DRAIN-AND-DISCARD IS UNSAFE HERE, AND THIS IS THE ONE PLACE THE CII'S
       CONTRACT MUST NOT BE COPIED. A squashed vector load's destination PRNs are
       recycled by the free list immediately, so a late response draining into a
       stale `pvdest` corrupts whatever now owns that PRN. Squashed element
       accesses must be dropped by POINTER ROLLBACK, not drained. Section 7 of
       the logic body states the contrast with VecCiiFlush in full; it is the
       most important paragraph in this file.

  ===> AND THE PRICE IS ACCEPTED DELIBERATELY. Pointer rollback is chosen over
       per-entry `br_mask` + `IsKilledByBranch` on ~1000 element-queue and LCB
       entries; the cost is that reservation depth, not MLP, bounds vector-load
       memory-level parallelism. Do not "fix" that by adding a per-entry
       br_mask — that is the rejected alternative, not an oversight.

  Governing spec anchors: loadstore.rst `vec-squash`, `order-fail-replay`,
  `elem-progress`, `ssi-queues`; issue.rst `vec-queue-reservation`;
  cii.rst `cii-flush` (the contract this module must NOT copy);
  plan v2 sections 5.6 (the vector-LSU invariant), 5.8 (precise exceptions),
  5.10 (reuse existing machinery) and step E8.
*/

<|begin_module|>

  <|begin_parameters|>
  `nQueues` (Int, default 6, legal value 6 only) — the number of VecElemQueue
  instances this unit rolls back. It is a parameter rather than a literal so the
  six squash commands are generated from VecBundles' normative queue enumeration
  (`ld_SSI_ADDR_Q`, `st_SSI_ADDR_Q`, `st_SSI_DATA_Q`, `ld_US_ADDR_Q`,
  `st_US_ADDR_Q`, `st_US_DATA_Q`); require it to equal that enumeration's size,
  so a queue outside the enumeration fails elaboration instead of staying live
  through a squash.

  `nKillClients` (Int, default 8, legal range 1..16) — the number of in-flight
  crack sites whose kill this unit resolves: the four fill-side agens
  (`ld_elem_agen`, `st_elem_agen`, `ld_range_agen`, `st_range_agen`), `idx_gen`,
  `mask_stream` and the two `VecBeatExpander` instances. VecLsu owns the
  ordering of the vector and must connect it consistently in both directions.

  Everything else comes from `HasBoomCoreParameters`: `numLdqEntries`,
  `numStqEntries`, `ldqAddrSz`, `stqAddrSz`, `coreWidth`. There is no depth, no
  latency and no width parameter of this module's own, because it owns no
  storage.

  This module is elaborated only when `usingRVV` is true. With vectors off it is
  ABSENT — not instantiated and not tied off — so a non-vector build is
  bit-identical to pre-Caracal BOOM v4. `usingRVV` is a Scala `Boolean` from
  `BoomCoreParams`, never a hardware `Bool`, and it is not rocket's
  `usingVector`.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset: Chisel's implicit `clock` (posedge) and `reset` (ACTIVE-HIGH,
  SYNCHRONOUS), per the hierarchy defaults. One clock domain, no second reset.

  ---- Recovery events in (all from vec_pipeline_io, via VecLsu) ----

  `brupdate` — `Input(new BrUpdateInfo)`, BOOM's existing branch-resolution
  bundle. This unit reads `brupdate.b2.mispredict`, `brupdate.b2.uop.ldq_idx`
  and `brupdate.b2.uop.stq_idx`, and passes the whole bundle to
  `IsKilledByBranch`. No new speculation-tracking mechanism is added.

  `rob_flush` — `Input(Bool())`, `rob.io.flush.valid` UNREGISTERED. Used only for
  the crack kill, so a crack stops pushing in the flush cycle itself.

  `rob_flush_kill` — `Input(Bool())`, the REGISTERED form the baseline LSU already
  consumes as `io.core.exception` (`io.lsu.exception := RegNext(rob.io.flush.valid)`,
  core.scala:1205). This, not `rob_flush`, is the queue/reservation/LCB rollback
  event, because it is the cycle on which lsu.scala rolls `ldq_tail`/`stq_tail` and
  clears `ldq_valid`/`stq_valid`. Two ports, not one, so the vector rollback lands
  on the SAME cycle as BOOM's own LSQ rollback rather than one cycle early.

  `ldq_head`, `ldq_tail` — `Input(UInt((1 + ldqAddrSz).W))`, and
  `stq_commit_head`, `stq_tail` — `Input(UInt((1 + stqAddrSz).W))`. The live
  LDQ/STQ pointers, tapped from the LSU delta through `VecLsuCoreIO`. All four
  carry BOOM's extra wrap/carry bit (`WrapIncWCarry`), so a real array index is
  `GetRealLSQIdx(...)` and every age comparison uses BOOM's existing
  `EntryValidFromAge` / `IsOlderLSU` rather than a hand-written compare.

  `kill_uop` — `Input(Vec(nKillClients, Valid(new MicroOp)))`: the uop each
  in-progress crack site is currently walking.

  `resv_rollback_tail` — `Input(Vec(nQueues, UInt))`: the recomputed allocation
  tail per queue, driven back by VecQueueReservation in the SAME cycle it applies
  the rollback to its own table. This unit forwards it to the queues rather than
  recomputing it, so the reservation table and the queue pointers cannot drift.

  ---- Commands out ----

  `q_squash` — `Output(Vec(nQueues, Valid(UInt())))`, matching VecElemQueue's
  `io.squash` exactly: `valid` plus the ABSOLUTE index to roll that queue's
  reservation tail back to.

  `resv_rollback` — `Output(Valid({ ldq_idx: UInt((1 + ldqAddrSz).W), stq_idx:
  UInt((1 + stqAddrSz).W) }))`, driving VecQueueReservation's `rollback` port.
  This unit owns the branch-versus-flush policy; VecQueueReservation owns only
  the translation from an LSQ index to a queue index.

  `kill_ldq` — `Output(UInt(numLdqEntries.W))`, one bit per LDQ entry, driving
  the LCB's `io.kill_ldq`. An LCB assembly entry is invalidated when the bit for
  its owning `ldq_idx` is set.

  `kill` — `Output(Vec(nKillClients, Bool()))`: the resolved per-client crack
  kill, positionally paired with `kill_uop`.

  ===> AND THAT IS THE WHOLE INTERFACE. There is deliberately NO `busy`, no
  `squash_pending`, no `ready` and no back-pressure of any kind, in either
  direction. Every command above is a single-cycle pulse that its consumer must
  accept unconditionally. A handshake here would be a stall path from branch
  resolution into the vector LSU, and a `busy`-style output reaching an issue
  unit is a failed review under plan rule 6 regardless of measured performance.
  <|end_ports|>

  <|begin_logic|>

  ---- 1. Why this module exists at all ----

  //@req-spec-lsu.i1
  `IQ_V_LOAD` and `IQ_V_STORE` issue SPECULATIVELY: unlike `IQ_V_ALU`, which
  carries a per-entry past-PNR eligibility gate so every op handed to the
  coprocessor is individually non-speculative, the two vector memory queues do
  not gate on the PNR at all. That is deliberate — a PNR gate on vector memory
  would serialize the load stream behind the oldest unresolved branch and destroy
  the memory-level parallelism the element queues exist to provide — and it is
  precisely what makes this module necessary: a vector memory `OP.v` can be
  mid-drain, with element accesses in flight, when a branch mispredicts. The
  speculation must not be traded away by adding a PNR gate to those two queues.

  The module is otherwise stateless: every output is a combinational function of
  the event inputs, the LSQ pointers and `resv_rollback_tail`. The only registers
  are section 9's debug counters, which nothing functional reads.

  ---- 2. Two events, one mechanism ----

  Exactly two events drive a squash, and they differ ONLY in the survivor
  indices they compute:

    branch  = `brupdate.b2.mispredict && !rob_flush_kill`
    flush   = `rob_flush_kill`

  The flush term wins when both are asserted in the same cycle — the same
  priority baseline lsu.scala uses at `when (io.core.brupdate.b2.mispredict &&
  !io.core.exception)` — because a flush is strictly more destructive and its
  survivor set is a subset of any branch's.

  ---- 3. Branch mispredict: pointer rollback, no per-entry compare ----

  //@req-spec-lsu.i2
  On `branch`, the surviving LSQ indices are `brupdate.b2.uop.ldq_idx` and
  `brupdate.b2.uop.stq_idx`, which is exactly what baseline lsu.scala assigns to
  `ldq_tail` / `stq_tail`. Drive them on `resv_rollback` and assert every
  `q_squash(q).valid` with `q_squash(q).bits := resv_rollback_tail(q)`, so each
  element queue's tail rolls back to the branch's reservation index in the same
  cycle BOOM rolls the LSQ tails and in the same cycle VecQueueReservation rolls
  its table.

  // CONVENTION, and it must match VecQueueReservation's reading of `rollback`:
  // the index driven here is BOOM's EXCLUSIVE tail. A branch is not a memory
  // op, so `uop.ldq_idx` holds the value `ldq_tail` had when the branch
  // dispatched; entries `[head, ldq_idx)` survive and `[ldq_idx, ldq_tail)` die.
  // The new element-queue tail is therefore `base + count` of the reservation
  // row owned by the YOUNGEST LSQ entry strictly OLDER than the driven index, or
  // the queue's head if that range is empty. Reading the driven index as an
  // inclusive survivor would keep one killed instruction's entries alive.

  //@req-spec-lsu.i4
  //@req-spec-lsu.i10
  Killed entries VANISH WITH NO PER-ENTRY COMPARE. This module emits no
  `br_mask`, computes no `IsKilledByBranch` against a queue entry, and reads no
  per-entry age — the element queues carry none of those fields, by design. The
  rollback is correct because each queue's occupied region is PROGRAM-ORDERED:
  capacity is claimed only through VecQueueReservation, only at dispatch, and
  dispatch is in program order, so a single tail pointer separates the survivors
  from the dead. Per-entry `br_mask` + `IsKilledByBranch` on every element-queue
  and LCB entry is the rejected alternative: it preserves free-running load
  streaming and full MLP, but puts `maxBrCount` bits plus kill logic on roughly a
  thousand entries to serve the common case where nothing is squashed at all.

  //@req-spec-lsu.i5
  The RESERVATIONS ARE RELEASED IN THE SAME EVENT, not in a follow-up cycle and
  not by a separate teardown: `resv_rollback` and the six `q_squash` pulses are
  one cycle, and VecQueueReservation invalidates every row younger than the
  survivor and recomputes each queue's occupancy on that same cycle. There is no
  intermediate state in which the entries are dead but their capacity is still
  claimed, so a squash can never starve dispatch while it is "in progress".

  ---- 4. ROB-head flush: the load side empties, the store side does not ----

  //@req-spec-memord.c23
  //@req-spec-memord.c24
  A ROB-head flush — an architectural exception, `MINI_EXCEPTION_MEM_ORDERING`,
  `MINI_EXCEPTION_CSR_REPLAY` or an ERET — discards the offending instruction AND
  EVERYTHING YOUNGER: ROB rows, issue-queue entries and the pipeline, with their
  speculatively-allocated physical registers freed by the same ROB rollback. This
  module implements the vector-LSU share of that scope, and it is a strict
  superset of any branch rollback, so it needs no age comparator of its own:

    - The LOAD-side queues (`ld_SSI_ADDR_Q`, `ld_US_ADDR_Q`) roll back to EMPTY.
      Drive `resv_rollback.ldq_idx := ldq_head`, which under the exclusive-tail
      convention above names an empty surviving range. This mirrors baseline
      lsu.scala, which sets `ldq_head := 0; ldq_tail := 0` and clears every
      `ldq_valid` on `io.core.exception`.
    - The STORE-side queues (`st_SSI_ADDR_Q`, `st_SSI_DATA_Q`, `st_US_ADDR_Q`,
      `st_US_DATA_Q`) roll back to `stq_commit_head`, NOT to the head. Drive
      `resv_rollback.stq_idx := stq_commit_head`.

  // ===> THE ASYMMETRY IS LOAD-BEARING AND GETTING IT WRONG LOSES COMMITTED
  // STORE DATA. A committed vector store has not yet written the D$: its
  // translated element addresses live in st_*_ADDR_Q and its data in
  // st_*_DATA_Q until the post-commit drain, which is why stores RETAIN their
  // entries rather than streaming them. Emptying the store queues on a flush
  // would discard the addresses and data of stores that are already
  // architecturally committed and can no longer be re-executed. Baseline
  // lsu.scala says the same thing in its own terms: on an exception it rolls
  // `stq_tail := stq_commit_head` and clears only entries that are neither
  // `stq_committed` nor `stq_succeeded`.

  ---- 5. LCB assembly entries: invalidate by owning ldq_idx ----

  //@req-spec-lsu.i7
  `kill_ldq` is the per-LDQ-entry invalidation mask the LCB applies to its
  assembly entries. Every entry whose owning `ldq_idx` bit is set clears `valid`,
  `preload_pending`, `written` and its `byte_valid` in that cycle. The mask, not
  the LCB, carries the age decision:

    - on `branch`: `kill_ldq(i) := EntryValidFromAge(brupdate.b2.uop.ldq_idx,
      ldq_tail, GetRealLSQIdx(i))` — the killed region `[new_tail, old_tail)`,
      computed with the same helper lsu.scala already uses for its younger-load
      masks, so there is exactly one age-comparison idiom in the machine;
    - on `flush`: all ones. Kill-all needs no comparator: a flush fires at the
      ROB head and an in-flight LCB entry can never belong to an instruction
      older than the head, because commit requires `rob_bsy` cleared, which for a
      vector load requires the LCB's group-done.

  The age comparison lives here and only here — the LCB matches a bit, and
  VecQueueReservation translates an index. One owner of the policy, three
  consumers of its result.

  //@req-spec-lsu.i8
  A LATE ELEMENT RESPONSE FOR A ROLLED-BACK `ldq_idx` IS DROPPED, and nothing
  extra is needed to make that true. The LCB places a beat only into an entry
  that is `valid` and matches on both `prn` AND `ldq_idx`; once the entry is
  invalidated the response has nowhere to land and is dropped in the cycle it
  arrives — never buffered, never retried, never redirected. Same shape as BOOM
  dropping a D$ response for an invalidated LDQ entry, and it needs no
  cancellation of the outstanding access itself: a rolled-back `ldq_idx` is simply
  dead. Symmetrically, the `W0` VRF write and the group-done are generated from
  LCB entry state alone, so an invalidated entry can generate neither, in that
  cycle or any later one.

  ---- 6. In-progress cracks are killed like any pipeline stage ----

  //@req-spec-lsu.i6
  A stage-1 vAGEN crack in progress is killed on `brupdate` or on a flush, with
  no rollback bookkeeping of its own: for each client `c`,
  `kill(c) := kill_uop(c).valid && IsKilledByBranch(brupdate, rob_flush,
  kill_uop(c).bits)`. That is BOOM's EXISTING predicate, unmodified, in the
  position baseline code uses it — the crack is treated exactly like any other
  pipeline stage carrying a uop. A killed crack must stop pushing in that cycle;
  whatever it already pushed is recovered by the pointer rollback of section 3,
  which is why nothing here needs to be undone and no client needs a rewind port.

  The kill is resolved HERE rather than at each client so the branch-versus-flush
  policy has one owner, and so a client that latches an index or mask lookahead
  (`idx_gen`, `mask_stream`) drops it on the same cycle the agen it feeds drops
  its cursor. `rob_flush` is used in the kill term rather than `rob_flush_kill`
  because stopping a push a cycle early is free, whereas stopping it a cycle late
  is a push into a region the rollback has already released.

  // Note on duplication: VecRangeAgen, VecDgen and VecScalarOperandRead take
  // `brupdate`/`rob_flush` directly and evaluate the same `IsKilledByBranch`
  // themselves. That is a pure function of its inputs, so the two cannot
  // disagree — it is not a second kill mechanism. What IS forbidden is a client
  // inventing its own kill condition, or holding killed work.

  ---- 7. Why drain-and-discard is unavailable, and why the CII differs ----

  //@req-spec-lsu.i9
  //@req-spec-memord.c16
  //@req-spec-memord.c17
  //@req-spec-memord.c18
  THIS IS THE CENTRAL DESIGN DECISION OF THE MODULE. VecCiiFlush handles a killed
  coprocessor instruction by DRAINING it: the tag's remaining beats are accepted,
  its writeback effects are suppressed, and the tag is freed on its own `last`
  beat exactly as a live one would be. That is safe there for one specific
  reason — a CII tag is OPAQUE, names nothing in any register file, and is not
  recycled while the drain is in progress.

  The vector LSU has no such property, because a squashed vector load's
  destination names REAL PHYSICAL REGISTERS THAT ARE REUSED IMMEDIATELY. On the
  recovery event the physical register file is deliberately LEFT UNTOUCHED — no
  value is rolled back, no write is undone — and correctness comes from renaming
  instead: the ROB rollback restores the rename map to the committed
  architectural state and returns EVERY speculatively-allocated physical register
  to the free list, the killed load's `pvdest` group included, through
  `br_alloc_lists`. The free list then hands those PRNs to a different
  instruction within a few cycles.

  //@req-spec-memord.c19
  //@req-spec-memord.c20
  For the killed instruction itself that is exactly right and is why no RF
  rollback is needed: the stale value is simply ORPHANED, since after the
  rollback nothing maps the load's logical destination to the old `Pd`, and on
  refetch the load renames to a FRESH physical destination, executes correctly
  and writes that register instead.

  // ===> BUT IT IS PRECISELY WHAT MAKES A DRAIN FATAL. A late LCB write into a
  // stale `pvdest` would not be a harmless write to an orphaned register: by the
  // time it lands, that PRN belongs to a live, unrelated, correctly-executing
  // instruction, and the write is a full VLEN wide. The corruption is silent —
  // no exception, no assertion, a wrong architectural result many instructions
  // later. So the vector LSU must make a late write IMPOSSIBLE rather than
  // merely ineffective, and pointer rollback plus LCB invalidation by `ldq_idx`
  // is how: the accesses are dropped, not drained. DO NOT port VecCiiFlush's
  // drain-and-discard pattern into any vector-LSU module, and do not "simplify"
  // this unit into a suppression of effects at the VRF write port — the write
  // port is downstream of the reallocation, so suppression there cannot
  // distinguish the killed load's PRN from its new owner's.

  ---- 8. Ordering-violation replay: whole instructions only ----

  //@req-spec-memord.e3
  //@req-spec-memord.e4
  //@req-spec-memord.e5
  A vector load that order-fails takes the same refetch/re-rename path as a
  scalar one, but AT THE GRANULARITY OF THE WHOLE VECTOR INSTRUCTION. The single
  LDQ placeholder entry drives one `lxcpt`; the resulting flush reaches this unit
  as `rob_flush_kill` and is handled by section 4 with no special case. On replay
  the vector op RE-RENAMES ITS WHOLE DESTINATION GROUP and RE-DRAINS ITS ELEMENT
  ACCESSES THROUGH THE LCB from element zero. THERE IS NO PARTIAL-GROUP REWIND:
  this unit exposes no per-member and no per-element rollback port, and it must
  never gain one. That is consistent with the one-group-done completion model — a
  group completes exactly once, so a partially-rewound group would have no
  representable completion state — and it is the same reason the flush drops the
  whole `pvdest` group rather than the members that had not yet been written.

  //@req-spec-lsu.f6
  //@req-spec-lsu.f7
  Correspondingly, `fault_elem` APPEARS ON NO PORT OF THIS MODULE AND ON NO PORT
  IT DRIVES. It is the element cursor's stop signal and a debug/performance
  counter, nothing more: it is NOT carried to the ROB — `VecException` carries
  `valid`, `rob_idx`, `cause` and `badvaddr` and deliberately no element index —
  and it is NEVER written to `vstart`. A faulting vector memory op traps with
  `vstart = 0` and restarts WHOLE, which is what makes this unit's coarse
  rollback correct: the trapping op never commits, its fresh `pvdest` group is
  reclaimed, and elements `0..k-1` were never architecturally visible, so there
  is nothing for a mid-vector resume to resume from. A `vstart = k` restart would
  require the surviving element writes to be architecturally visible, which is
  exactly what section 7 shows cannot be arranged.

  ---- 9. Trace and assertions ----

  Emit one guarded `VecTrace` line per squash event — `branch` and `flush`
  separately — tagged with the module name and the `rob_idx` of `brupdate.b2.uop`
  (or of the flushing head where available), carrying the two survivor indices,
  the six rolled-back tails and the population count of `kill_ldq`. With no unit
  tests in this project these lines are the only way to see a squash in a log, and
  the `kill_ldq` popcount is what distinguishes "rolled back nothing" from "rolled
  back the wrong region".

  Debug counters (registered, read by nothing functional): squashes by cause, and
  element-queue entries reclaimed by rollback. If the second is persistently large
  relative to entries retired normally, reservation depth — not this unit — is
  what limits throughput.

  Assertions, all synthesizable and all cheap:
    - `q_squash(q).bits` never rolls a tail behind that queue's head (the queue
      asserts the same thing locally; assert it at the source too, because here
      the survivor index is available to name in the message).
    - `resv_rollback.valid` implies `branch || flush`, and `branch && flush` never
      both drive a command — the flush must have won.
    - On `flush`, `kill_ldq` is all ones. A flush that killed a proper subset of
      LDQ entries means the head was wrongly treated as a survivor.
    - `kill(c)` is never asserted while `kill_uop(c).valid` is false — a kill
      pulse with no uop behind it means VecLsu miswired the client vector.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
A SQUASH COMPLETES IN ONE CYCLE, and that is a constraint on the implementation,
not an aspiration. All six queue rollbacks, the reservation-table rollback and
the LCB invalidation are applied on the same cycle as the event. No multi-cycle
teardown, no drain state, no walk over entries, no state machine — a squash
taking N cycles would have to hold back dispatch or the fill side for N cycles,
reintroducing the concurrency ceiling the v2 element-queue substrate exists to
remove.

The LCB invalidation must land before the ROB rollback can return the killed
group's PRNs to the free list. Using the registered `rob_flush_kill` for the
rollback keeps one cycle of margin, which is safe because the ROB's rollback walk
takes multiple cycles; it must not be delayed further than that.

Combinational cost: `numLdqEntries` age comparators for `kill_ldq` plus
`nKillClients` `IsKilledByBranch` terms, all driven from the already-late
`brupdate.b2` — so keep this unit to one comparator level and one mask, with no
chained arithmetic on the branch-resolve path. The six rollback tails are NOT
computed here; they arrive on `resv_rollback_tail`, which keeps the reservation
table's adder out of this budget as well as guaranteeing the two agree.

Zero cost when nothing is squashed: no event, no pulse, no state change anywhere.
That is the whole economic case for pointer rollback over per-entry `br_mask`.
<|end_perf|>

<|begin_dependencies|>
MicroOp — `kill_uop` carries `new MicroOp()`; `brupdate.b2.uop` supplies
`ldq_idx`/`stq_idx`/`br_mask`.
VecBundles — the normative six-queue enumeration that `nQueues` is checked
against, and `VecException` (referenced in section 8 for what it must NOT carry).
VecTrace — the guarded trace lines and their `rob_idx` tagging convention.

Binds to BOOM's existing helpers rather than reimplementing them, per plan rule
10: `BrUpdateInfo`, `IsKilledByBranch`, `EntryValidFromAge`, `IsOlderLSU` and
`GetRealLSQIdx` from `boom.v4.util` / `boom.v4.common`. No new speculation,
kill, wakeup or recovery mechanism is introduced anywhere in this file.

Instantiates nothing. Its counterparties, and what each side owes:
  VecLsu — instantiates it as `squash`; routes `brupdate`, `rob_flush`,
    `rob_flush_kill` and the four LSQ pointers in, fans `q_squash` out to the six
    VecElemQueue instances in enumeration order, and pairs `kill_uop`/`kill` with
    the eight crack clients in a fixed order.
  VecElemQueue (x6) — accepts `q_squash` on its `io.squash` (`valid` + `tail`)
    unconditionally, rolls `tail` and clears `filled`/`xlated` over the abandoned
    range with one mask, and lets the squash WIN over a colliding
    `io.resv.release_tail`.
  VecQueueReservation — receives `resv_rollback` and returns `rollback_tail` in
    the same cycle, applying the identical value to its own `alloc_tail(q)` and
    invalidating every row younger than the survivor.
  VecLoadCoalescingBuffer — receives `kill_ldq` on `io.kill_ldq` and invalidates
    matching assembly entries in that cycle.
  LSU (edit_existing, src/main/scala/v4/lsu/lsu.scala) — exports `ldq_head`,
    `ldq_tail`, `stq_commit_head` and `stq_tail` through `VecLsuCoreIO`, and rolls
    its own LDQ/STQ pointers on the same two events, on the same cycles.
  VecCiiFlush — the DELIBERATE CONTRAST, not a dependency: no signal crosses
    between them and neither may adopt the other's contract. Section 7 is the
    written reason.
<|end_dependencies|>

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
  VecQueueReservation — the dispatch-time capacity allocator for the six vector
  element queues: it claims capacity IN PROGRAM ORDER at dispatch — the worst case
  for a store, a capped quantum for a load — hands each OP.v the index region its
  AGEN will later write, and reclaims that region when the op's LDQ/STQ placeholder
  deallocates.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecQueueReservation.scala,
  package boom.v4.vec.generated.lsu. depends_on MicroOp, VecBundles,
  VectorParams, VecTrace. Instantiated once, as `resv` inside VecLsu.
  Elaborated only when `usingRVV` is true — absent, not tied off — so a
  vectors-off build emits RTL bit-identical to pre-Caracal BOOM v4.
  Plan step E2: built first, together with VecElemQueue, so that nothing
  downstream can be written against a `busy` that does not exist.

  ===> THE RESERVATION RULE SPLITS BY DIRECTION. STORES reserve the full
       worst-case active element count. LOADS reserve
       `min(worstCase, ldResvMembers * vLen/eew)` and STREAM the remainder
       through their region. Logic section 1 states both rules and section 3 the
       common grant discipline; the two callouts below are the reasons the store
       rule cannot be relaxed and the load rule can.

  ===> FOR STORES THIS IS A CORRECTNESS MECHANISM, NOT A TUNING KNOB, AND THE
       DEADLOCK ARGUMENT IS WHY IT EXISTS. EVERY STEP BELOW IS ABOUT STORES ONLY
       — it is the argument that pins the store rule to the worst case, and a
       reader who generalizes it to loads will over-reserve the load queues for
       no reason. Suppose store capacity were claimed opportunistically at
       execute, with a full queue back-pressuring the store vAGEN:
         1. Older store A's operands are not ready, so younger B and C are
            granted first — age-ordered-READY issue from IQ_V_STORE may skip a
            not-ready older entry — and they fill the queue.
         2. A becomes ready, issues, and back-pressures at the vAGEN: no room.
         3. A cannot finish translating its element set, so A cannot commit.
         4. Commit is in-order, so B and C never reach commit-drain and never
            free their entries. Nothing moves. It wedges on a partial fill too:
            A can occupy part of the queue and jam against B.
       Liveness under that scheme would need
       depth >= max_elements_per_store * max_in_flight_stores = 64 * 256 = 16384
       entries. SIZING CANNOT FIX IT. The reservation is what fixes it, because
       the oldest store always already holds its capacity.

  ===> AND THAT IS WHY A LOAD MAY UNDER-RESERVE. LOADS HAVE NO SUCH FAILURE MODE
       — a load completes out of the LCB without waiting for commit
       (`spec-lsu.b10`), so a younger load drains and an older one merely stalls.
       ONLY STORES HAVE THE DEADLOCK EXPOSURE. A load reserves in program order
       for the OTHER reason entirely: SQUASHABILITY. An in-order allocation
       history is what makes every queue squashable by pointer rollback, with no
       per-entry br_mask on ~1000 entries — and squashability SURVIVES a smaller
       reservation, because the region is still contiguous and still
       program-ordered, merely smaller.
       The under-reservation is deadlock-free for a second, separate reason, and
       it is a property of VecElemQueue rather than of this module: a load region
       is reused CIRCULARLY WITHIN ITSELF and is NEVER EXTENDED PAST `tail`. An
       older load's region sits AHEAD of a younger one's, so it drains first and
       refills into its OWN indices; no younger reservation can ever sit between
       an older load and the entries it needs. Break that one property — let a
       streaming load extend past its region — and the load rule becomes unsafe
       immediately.

  ===> NO `busy` LEAVES THIS MODULE. `dis_ok` is a dispatch-stage capacity check,
       not an execution-busy signal: it is consumed by the in-order dispatch stage
       and never by an issue unit. Issue eligibility for a vector memory OP.v is
       purely "a reservation exists", decided here at dispatch. That is the
       vector-LSU invariant (plan section 5 rule 6), and a `busy` reaching an
       issue unit is a failed review regardless of measured performance.

  Governing spec anchors: issue.rst `vec-queue-reservation` and
  `cii-shared-sched`, loadstore.rst `ssi-queues`, `store-data-queue`,
  `us-queue`, `vec-store-algo`, `vec-load-algo` and `vec-squash`.

<|begin_module|>

  <|begin_parameters|>
  No new tuning knobs DECLARED HERE, and no literal widths. From VectorParams
  through `HasVectorParams`: `ssiQueueEntries` (512) and `usQueueEntries` (16)
  give the six queue depths, `vLen`/`maxMembers`/`vecVLSz` size the element-count
  arithmetic, `maxInflightWorstCaseStores` is READ rather than recomputed (see the
  derived-limit paragraph in the logic section), and `ldResvMembers` (default 4)
  is the LOAD reservation quantum in destination MEMBERS. From
  `HasBoomCoreParameters`: `coreWidth` dispatch lanes per cycle, `lsuWidth` for
  the quantum's sanity check, plus `robAddrSz`, `ldqAddrSz` and `stqAddrSz` for
  the ownership fields and the reservation table.

  `ldResvMembers` is BOUND FROM VectorParams, never re-defaulted here and
  never compared against a literal 4. Elaboration must hold
    require(ldResvMembers * vLen/eew_min >= lsuWidth * 2)
  — the agen fills one element per cycle while the drain consumes up to
  `lsuWidth` per cycle, so a too-small quantum lets the agen starve the drain.
  VectorParams is the single owner of that `require` (it owns the parameter);
  it is written out here because it is THIS module's arithmetic it protects,
  and a generator that reads only this file must still know the constraint
  exists rather than inventing a second, weaker one.

  Derived, as named values so no expression is repeated at a use site: `nQueues`
  = 6, fixed by VecBundles' queue enumeration rather than chosen here; `qDepth(q)`
  (SSI `ssiQueueEntries`, US `usQueueEntries`); `qIdxSz(q)` = `log2Ceil(qDepth(q))`;
  `vLenBytes` = `vLen / 8`, which is also the worst-case element count of one
  OP.v, reached at EEW=8 with EMUL=8; and `ldResvCap(eew)` = `ldResvMembers *
  (vLenBytes >> eew)`, the load reservation ceiling, one elaboration-time constant
  per EEW so the `min` in logic section 1 costs a compare and a mux and no
  multiplier.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit pair, matching the hierarchy defaults:
  posedge `clock`, ACTIVE-HIGH SYNCHRONOUS `reset`. On reset every allocation
  pointer and occupancy counter is zero and every reservation-table row is
  invalid.

  ---- Dispatch side (to and from the in-order dispatch stage) ----

  `dis_uops` — input, `Vec(coreWidth, Valid(new MicroOp()))`: the dispatch-group
  uops in PROGRAM ORDER, lane 0 oldest. These are the REGISTERED ren2 uops the
  vec_pipeline_io seam calls `ren2_uops`/`dis_fire`, not `dec_uops` — the
  reservation must be computed on the same cycle the LDQ/STQ slot is claimed, and
  a combinational feed from `dec_uops` runs a cycle ahead of it. Only `is_vec`
  uops with `uses_ldq` or `uses_stq` request anything; every other lane requests
  nothing and is granted unconditionally.

  `dis_ok` — output, `Vec(coreWidth, Bool)`: per-lane "capacity is available for
  this lane, given every older lane in the same group", combinational in the
  dispatch cycle. VecPipeline reduces these into vec_pipeline_io's single-bit
  `dis_ready`.

  `dis_fire` — input, `Vec(coreWidth, Bool)`: the lane actually dispatched. State
  changes only on `dis_fire`; `dis_ok` alone commits nothing, and that split is
  what makes an all-or-nothing multi-resource acquisition possible.

  `resv_out` — output, `Vec(coreWidth, Vec(2, Valid(new VecReservation())))`: the
  granted regions, slot 0 the address queue and slot 1 the data queue (valid for
  stores only). Fields are VecBundles' `VecReservation` — queue id, entry count,
  index region, owning `rob_idx`, reserving `ldq_idx`/`stq_idx` — and VecLsu
  routes each to a VecElemQueue by the `queue` field.
  THE TWO SLOTS CARRY INDEPENDENT BASES AND INDEPENDENT COUNTS. They agree for
  an SSI store and they DO NOT for a US store (logic section 2). `us_data_base`
  on the range entry is slot 1's base — that is the consumer which makes the
  second base load-bearing rather than decorative.

  ---- Execute side (four lanes, one per AGEN instance) ----

  Lane assignment is FIXED and matches VecLsu's instances: 0 = `ld_elem_agen`,
  1 = `st_elem_agen`, 2 = `ld_range_agen`, 3 = `st_range_agen`.

  `resv_lookup` — input, `Vec(4, Valid({ is_store: Bool, q_idx: UInt }))`, with
  `resv_resp` — output, `Vec(4, Vec(2, { base: UInt, count: UInt }))`: a
  combinational read of the reservation table keyed on `ldq_idx` (loads) or
  `stq_idx` (stores). BOTH SLOTS ARE RETURNED — a US store's DGEN needs slot 1's
  base and it is not derivable from slot 0's (logic section 2); a load simply
  ignores slot 1.

  `release` — input, `Vec(4, Valid({ is_store: Bool, q_idx: UInt, used_count:
  Vec(2, UInt) }))`, with `release_ok` — output, `Vec(4, Bool)`: the surplus-return
  request and its grant. `used_count` is PER QUEUE SLOT for the same reason
  `resv_out` carries two bases — a US store's address and data regions are trimmed
  by different amounts (logic section 7).

  ---- Reclamation side (the retire pair) ----

  `retire` — input, `Valid({ is_store: Bool, q_idx: UInt })`: the LDQ or STQ index
  of the vector placeholder being DEALLOCATED this cycle, presented by VecLsu from
  the LSU's `ldq_head`/`stq_head`. One per cycle is enough — an LSQ deallocates at
  most one entry per direction per cycle and a region is freed whole.
  A DEDICATED PORT PAIR, NOT A FIFTH `resv_lookup` LANE AND NOT A REUSED ONE.
  The four lookup lanes are permanently assigned to the four AGENs (see the
  execute side above), so there is no free lane; and reclamation must not be
  able to lose a cycle to an AGEN's read, because a dropped retire is capacity
  never returned — the same hang by a slower route.

  `retire_row_valid` — output, `Bool`: whether the row addressed by `retire.bits`
  (`is_store` selects the table, `q_idx` the row) currently holds a live
  reservation. Combinational and INDEPENDENT of `retire.valid`, so the driver may
  use it to decide that valid in the same cycle.
  ===> THIS EXISTS BECAUSE THE LDQ AND STQ ARE SHARED WITH SCALAR MEMORY OPS. A
  vector placeholder is a minority of the entries VecLsu's head-side walk passes
  over; every scalar load and store deallocates through the same `ldq_head`/
  `stq_head` and reserved nothing. Without this bit VecLsu's only options are to
  pulse `retire` on every deallocation — which retires against an empty row, the
  invalid-row assertion below and, unasserted, a corrupt `occ(q)` — or to shadow
  this module's validity bits locally, a second source of truth that drifts on the
  mispredict cycle rollback clears a row here and not there. It is NOT a
  replacement for that assertion: the assertion still fires if a retire arrives
  against a row this output reports empty.

  `region_free` — output, `Vec(nQueues, Valid({ base: UInt, count: UInt }))`: the
  echo. Looking `retire.q_idx` up in the reservation table yields that op's base
  and count PER QUEUE SLOT; this module invalidates the row, subtracts each count
  from the corresponding `occ(q)`, and drives the same base/count out here for
  VecLsu to apply to each queue's `io.resv.free` port IN THE SAME CYCLE. One
  lookup, one echo, two appliers, no second source of truth about which entries
  just became free.

  ===> THIS PAIR IS THE ONLY HEAD-SIDE RECLAMATION PATH IN THE SUBTREE, AND ITS
  ABSENCE WAS A HANG. It replaces an earlier `q_free` input which reported
  "entries reclaimed from that queue's head this cycle" and WHICH NOTHING DROVE:
  no module in the vector-LSU subtree produced head-side reclamation, VecElemQueue
  exports only `io.empty` and CONSUMES a region-free command rather than producing
  one, so the six element queues would have filled exactly once and never freed —
  reached by any program issuing more vector memory ops than one reservation's
  worth, i.e. the first real vector test.
  WHY NO PER-FILE REVIEW COULD HAVE CAUGHT IT, which is the part worth
  remembering: every file was individually self-consistent. This file correctly
  said the queues report reclamation on `q_free`; VecElemQueue correctly
  consumed a region free at its head. The signal simply had no producer. A
  missing driver is invisible to any check that looks at one file at a time,
  and is exactly what a cross-file seam pass exists to find.

  ---- Squash side ----

  `rollback` — input, `Valid({ ldq_idx: UInt((1 + ldqAddrSz).W), stq_idx:
  UInt((1 + stqAddrSz).W) })`: the rolled-back LDQ/STQ tail indices, whatever their
  cause. THESE ARE BOOM'S EXCLUSIVE TAILS — each names the FIRST DEAD entry, not
  the youngest survivor; see logic section 8, which is where getting it wrong
  keeps a killed instruction's entries alive. VecSquashUnit owns the
  branch-versus-flush policy; this module owns only the translation from an
  LDQ/STQ index to a queue index.

  ===> THE WIDTHS INCLUDE THE CARRY BIT — `1 + ldqAddrSz`, not `ldqAddrSz` —
  corrected at E2, where the narrow declaration was found to make logic section 8
  unimplementable. The value driven here is `brupdate.b2.uop.ldq_idx`, which this
  spec's own section 8 names as its source, and `MicroOp.ldq_idx` is
  `UInt((1 + ldqAddrSz).W)`. Declaring the port one bit narrower silently discards
  the wrap-disambiguating bit, and WITHOUT IT NO AGE COMPARISON IS POSSIBLE: real
  indices alone cannot tell "younger than the pivot" from "older and wrapped", so
  the kill sweep degenerates into scanning for contiguously-valid rows — which
  stops at the first interleaved SCALAR entry and leaves younger vector
  reservations alive. Those rows are never freed, so the queue leaks capacity
  permanently and the machine deadlocks after enough mispredicts: the same failure
  class as an unreserved store queue, reached by a different route. A port width is
  not a detail when the bit being dropped is the one that orders the events.

  `ldq_head` — input, `UInt((1 + ldqAddrSz).W)` — and `stq_head`,
  `UInt((1 + stqAddrSz).W)`, tapped from the LSU's own pointer registers through
  `VecLsuCoreIO`. These exist so section 8 can call `IsOlderLSU(a, b, head)` with
  the head argument it actually takes, which is how the rest of `lsu.scala`,
  `VecLsu` and `VecSquashUnit` all spell an LSQ age comparison (ground rule 10:
  reuse BOOM's machinery rather than a private comparator). They are pure reads of
  state the LSU already maintains — this module adds no pointer tracking of its
  own for the LDQ/STQ, and must not: a second copy would drift on exactly the
  mispredict cycle it is needed.

  TWO PORTS, NOT FOUR: the heads only. An earlier E2 revision of this paragraph
  also declared `ldq_tail`/`stq_tail` "for symmetry with the `VecLsuCoreIO` tap".
  Nothing here reads them — `IsOlderLSU` takes a head, the no-survivor fallback
  drives the VEC queue's head, and `occ(q)` is recomputed from the vec queue's own
  pointers. An input port no logic consumes is not free: mode `new` forbids "extra",
  VecLsu would wire two signals to nothing, and the next reader has to work out
  whether the omission is a bug. Declare exactly what is read.

  `rollback_tail` — output, `Vec(nQueues, UInt)`: the recomputed allocation tail
  per queue, for VecSquashUnit to apply to the six queues in the same cycle this
  module applies it to its own state, so the two cannot drift.

  There is deliberately NO `busy`, no `full` and no ready/valid handshake toward
  any issue queue. See the header callout.
  <|end_ports|>

  <|begin_logic|>

  ---- 1. The requested count: worst case for stores, capped for loads ----

  //@req-spec-lsu.b19
  //@req-spec-issue.b2
  A STORE requests the FULL WORST CASE, in both its address and its data queue or it
  does not dispatch. That is not a sizing preference: the four-step deadlock argument
  in the header depends on the oldest store already holding all the capacity it will
  ever need, so a store that under-reserved could not finish translating its element
  set and could never commit.

  //@req-spec-lsu.b18
  //@req-spec-issue.b9
  A LOAD requests `min(worstCase, ldResvCap(v_eew))` with
  `ldResvCap(eew) = ldResvMembers * (vLenBytes >> eew)` and `ldResvMembers` bound
  from VectorParams (default 4), and STREAMS the remainder in waves. A load may
  under-reserve because it reserves for SQUASHABILITY, not deadlock avoidance — it
  completes out of the LCB without gating on commit — so it has no deadlock exposure
  to protect. The cap is EEW-relative rather than a flat entry count because the agen
  produces one element per cycle while the drain consumes up to `lsuWidth`, so too
  small a cap starves the drain and undercuts P2.

  //@req-spec-issue.b3
  For a strided, indexed or segmented (SSI) access the WORST CASE count is
  `emul_total * elemsPerReg`, where `elemsPerReg` = `vLenBytes >> v_eew` is the
  number of EEW-wide elements in one register and `emul_total` is the TOTAL
  member count the access touches: `v_emul * v_seg_nf` when segmented, `v_emul`
  otherwise, with `v_emul` the PER-FIELD EMUL. Both terms come from the static
  access descriptor VLSDecode wrote into the uop — `v_eew`, `v_emul`, `v_seg_nf`
  — so the count needs NO VL and is fully known at decode. At the defaults EEW=8
  with EMUL=8 gives `8 * (32 >> 0)` = 256 elements, the worst case; EEW=64 with
  EMUL=8 gives 32.

  RVV guarantees NF*EMUL <= 8, so assert emul_total <= maxMembers. Not
  redundant: if VLSDecode ever folded nf into v_emul this expression would
  over-reserve by a factor of nf, and the assertion is what catches the
  double-count instead of a machine that mysteriously admits fewer stores.
  IMPLEMENT THE PRODUCT AS A CONSTANT TABLE, NOT A MULTIPLIER — it is needed
  coreWidth times per cycle in the dispatch stage's combinational path, and
  nf is not always a power of two (3, 5, 6, 7 are legal) so it is not a
  shift. A 4 x 8 table indexed by {v_eew, emul_total} of elaboration-time
  constants puts no arithmetic in the dispatch path.

  A STORE REQUESTS THAT WORST CASE, UNMODIFIED. The four-step argument in the
  header is why, and it is the whole reason the number is computed from EMUL/EEW
  instead of from VL.

  A LOAD REQUESTS `min(worstCase, ldResvCap(v_eew))`, where `ldResvCap(v_eew)` =
  `ldResvMembers * (vLenBytes >> v_eew)` — `ldResvMembers` destination members'
  worth of entries, four by default. A load whose ACTIVE element count exceeds its
  reservation is not denied and is not deferred: it is STREAMED through
  `ld_SSI_ADDR_Q` in waves, filling its region, draining, and refilling the same
  indices as the drain side clears them, which is `spec-lsu.b11`'s streaming path
  and the only reason that path is reachable at all. Nothing else about a load's
  reservation changes: it is still claimed at dispatch, still in program order,
  still contiguous, still rolled back by a tail move.
  ===> AND THAT IS WHY THE MECHANISM IS EEW-RELATIVE RATHER THAN A FLAT ENTRY
       COUNT. The agen produces ONE element per cycle while the drain consumes
       up to `lsuWidth` per cycle, so a region too small to hold more than a
       few beats lets the agen STARVE THE DRAIN — the queue empties faster than
       it refills, and target P2 is undercut by a reservation policy rather than
       by memory bandwidth. Scaling by `vLen/eew` keeps the wave a fixed number
       of MEMBERS regardless of SEW, which is the unit the drain rate is in.
  ===> THE TRAP, RECORDED SO A FUTURE TUNER DOES NOT WALK INTO IT:
       `ldResvMembers >= 8` MAKES THIS MECHANISM DEAD AGAIN. `worstCase =
       emul_total * elemsPerReg` and EMUL <= 8 always, so at 8 the `min` would
       always select `worstCase`, no load could ever exceed its reservation,
       and `spec-lsu.b11`'s precondition would be unreachable — identical in
       effect to deleting the streaming path. At 512 entries with EMUL=8 and
       SEW=8: 2 gives 8 loads in flight, 4 gives 4 loads, 8 gives 2 loads and
       no streaming. 4 is the shipped point: double the in-flight loads of the
       worst-case rule, with margin above the starvation floor the `require` in
       the parameters section enforces.

  A unit-stride, whole-register (`vl1re*`/`vs1r`) or mask (`vlm`/`vsm`) ADDRESS
  entry is exactly ONE entry regardless of EMUL and EEW, because a US queue entry
  is a whole contiguous byte range — one `nOP.v` per instruction rather than one
  per element — so the load cap never binds on the US address queues. Its DATA
  side is a different count entirely; section 2 says what. VectorParams'
  element-count floor consequently applies to the SSI queues only.

  ---- 2. Which queues a lane requests, and in what counts ----

  //@req-spec-lsu.d12
  //@req-spec-lsu.d13
  Direction and access class select the queues. A vector load requests its
  address queue only (`ld_SSI_ADDR_Q` or `ld_US_ADDR_Q`) — there is no load data
  queue, because a load's returning data goes to the LCB for assembly. A vector
  store requests an address queue AND a data queue (`st_SSI_ADDR_Q` with
  `st_SSI_DATA_Q`, or `st_US_ADDR_Q` with `st_US_DATA_Q`) on exactly the same
  terms, because the store data is captured from the source vPRN at DGEN and must
  be held until the post-commit drain just as the translated addresses are. A
  VECTOR STORE RESERVES IN BOTH QUEUES OR DOES NOT DISPATCH: one indivisible
  grant, never two independent ones, so no state exists in which a store owns
  addresses it has nowhere to put data for.

  ===> THE TWO COUNTS ARE EQUAL FOR AN SSI STORE AND ARE NOT EQUAL FOR A US STORE.
       This is the seam most likely to be mis-generated, so it is stated as a rule
       and not as a note: `st_US_ADDR_Q` holds ONE range entry per store, while
       `st_US_DATA_Q` holds one full `vLen` entry PER GROUP MEMBER — so a US store
       claims 1 address entry and `v_emul * v_seg_nf` data entries. THE ADDRESS AND
       DATA REGIONS OF A US STORE ARE THEREFORE NOT IN IDENTITY CORRESPONDENCE:
       different counts, different bases, different pointers.
       CONSEQUENTLY A TABLE ROW CARRIES A BASE AND A COUNT PER QUEUE SLOT, not one
       shared pair — `resv_out` already has two slots and each is filled
       independently — and `us_data_base` on the range entry comes from SLOT 1.
       Both `VecStoreForward` and `VecLsu` reported this gap independently, which
       is a fair signal of how easy the identity reading is to fall into.

  The equal-count / one-shared-base rule holds for the SSI PAIR ONLY, and there
  it is worth asserting: the two SSI queues have EQUAL DEPTH and are allocated
  by the same event with the same count, so their allocation pointers are
  provably identical — assert slot 0's and slot 1's bases and counts agree
  whenever the class is SSI, and do NOT assert it for US. Their HEADS are never
  identical in either class, since address and data entries retire
  independently, so free space is always checked PER QUEUE against that queue's
  own count and a lane is granted only if both fit.

  ---- 3. The grant rule, and program order inside a dispatch group ----

  //@req-spec-issue.b1
  //@req-spec-issue.b2
  Dispatch acquires the element-queue reservation in the SAME in-order action
  that claims the LDQ/STQ slot: `dis_ok` for a lane is false unless every queue
  that lane requests has at least THAT QUEUE'S requested count of free entries. A
  vector load or store therefore cannot dispatch unless its target address queue
  — and, for a store, its data queue — has room for the count section 1 and
  section 2 computed for it: the worst-case active element count for a store, the
  capped count for a load, and a per-queue count for a US store. This is BOOM's
  existing LDQ/STQ discipline extended to the element queues, not a new mechanism.
  The grant rule itself does NOT know about the load cap or the US asymmetry —
  it compares a requested count against free space per queue and nothing else.
  Keeping the direction-and-class arithmetic entirely inside section 1/2 is what
  keeps the dispatch-critical-path structure (prefix sum, compare) unchanged by
  decision D9/D10.

  //@req-spec-lsu.b4
  //@req-spec-lsu.b5
  Capacity is reserved in program order at dispatch, never claimed
  opportunistically at execute (the header callout says why), so the lanes of a
  dispatch group are granted IN AGE ORDER rather than independently: take a
  per-queue running prefix sum over the lanes, compare each prefix against that
  queue's free space, and AND each lane's own fit with every older lane's. Lane 1
  may not be granted while lane 0 is denied even if lane 1 alone would fit. A
  younger op reserving ahead of an older one leaves the occupied region out of
  program order, which forfeits both properties the discipline buys — deadlock
  freedom for stores, and squash by tail rollback instead of a br_mask compare on
  every one of hundreds of entries.

  ---- 4. Shared (segmented) instructions ----

  //@req-spec-issue.c1
  A shared instruction — currently only a segmented load or store, marked
  `is_shared` by the decoder — needs more than one execution resource, and ALL of
  them are acquired at DISPATCH time in the one in-order dispatch action: the CII
  IQ slot for the coprocessor half, the LDQ/STQ slot, the `pvdest` and `pvtmp`
  groups, and this module's element-queue reservation. The all-or-nothing part is
  enforced structurally rather than by negotiating with the other resource
  owners: `dis_ok` is a pure availability answer that claims nothing, and state
  changes only on `dis_fire`, which dispatch asserts only when every resource for
  that lane is available. A partially acquired shared op is unrepresentable.

  The two halves share ONE reservation, the LS half's — the coprocessor half
  consumes no element-queue capacity, because it rendezvouses through the `pvtmp`
  group in the VRF. The reservation is sized from the LS half's access
  descriptor, which is where `v_seg_nf` enters `emul_total`.

  ---- 5. State ----

  Per queue: an allocation tail pointer `alloc_tail(q)` of `qIdxSz(q)` bits and
  an occupancy counter `occ(q)` of `qIdxSz(q)+1` bits; free space is
  `qDepth(q) - occ(q)`.
  An explicit counter, not a maybe-full bit over a wrapped pointer compare:
  this tail moves BACKWARD on a release and on a rollback as well as forward
  on a reservation, and a wrap-compare full/empty scheme is not robust to a
  backward-moving pointer. The counter costs ten flops per queue.

  A reservation table with one row per LDQ entry and one per STQ entry, indexed
  by `ldq_idx`/`stq_idx`, each holding { valid, released, and PER QUEUE SLOT a
  { queue, base, count } }: two slots, slot 0 the address queue and slot 1 the
  data queue, exactly as `resv_out` carries them. Slot 1 is invalid for a load.
  TWO BASES AND TWO COUNTS, NOT ONE OF EACH — section 2 says why: a US store's
  address and data regions have different counts and therefore different bases.
  Sharing one pair was the original reading and it is wrong; it would place
  `us_data_base` at the address queue's base and scatter a US store's data
  writes over another op's region.
  This table is per-LDQ/STQ-ENTRY state, NOT state scoped to "the current
  instruction", and it exports no busy — it is what the vector-LSU invariant
  relies on, not a violation of it, and it is the only place in the machine
  that knows which queue indices an OP.v owns. Ground rule 6 has since been
  AMENDED to enumerate THREE legal homes for in-flight vector-LSU state rather
  than two — (a) the six VecElemQueue instances, (b) the LCB's per-PRN assembly
  entries, (c) VecLsu's per-LDQ/STQ-entry descriptor pending table (decision
  D5) — and this table is the same KIND as (a) and (c): indexed by an LSQ entry
  the dispatch-time allocation already guaranteed, structurally
  un-overflowable, and exporting no busy. Do not read the amendment as a
  licence for a fourth: it names three, and a `busy` from any of them is still
  a failed review.

  On `dis_fire` for a granted lane: write the row, emit the reservation on
  `resv_out`, and for EACH queue slot the lane requested add that slot's count to
  its `occ(q)` and advance its `alloc_tail(q)`. A `retire` echo (section 6)
  subtracts each slot's count from the corresponding `occ(q)` in the same cycle and
  never touches `alloc_tail`.

  ---- 6. Region handoff, and why the base must be published ----

  `resv_resp` returns the row's per-slot `base` and `count` to the AGEN at
  execute — both slots, since a US store's DGEN needs slot 1's base and an SSI
  store's assertion needs to see that the two agree. THE BASE IS LOAD-BEARING, and
  this is the subtle reason the table exists: issue is age-ordered-READY and may
  skip a not-ready older entry, so a younger OP.v's AGEN can fill its region before
  an older one's has started. Allocation is in program order; FILL IS NOT. A
  VecElemQueue therefore cannot append at a single write pointer — it writes at
  `base + element_index` within the region handed out here. A STREAMING LOAD IS THE
  SAME RULE APPLIED CYCLICALLY: it writes at `base + (element_index mod count)`,
  inside the region it already owns, and never past `base + count`.

  //@req-spec-issue.b7
  Space comes back only at a queue's HEAD, in program order, and the `retire` pair
  is the ONLY path by which it does. VecLsu presents the LDQ/STQ index being
  deallocated — the LSU's `ldq_head`/`stq_head`, so it is the OLDEST vector op of
  that direction by construction — this module looks the row up, invalidates it,
  subtracts each slot's count from that slot's `occ(q)`, and drives the same
  base/count on `region_free` for VecLsu to hand to each queue's `io.resv.free`
  in the SAME CYCLE. A younger region that has finished draining therefore does
  not return its space until every older region has been reclaimed, because the
  retiring index advances in program order. This is the same rule the release obeys
  — unused entries that cannot be handed back at the tail stay held and are freed
  IN ORDER with the rest of the region — and it is what keeps each occupied region
  contiguous. The cost is head-of-line blocking on RECLAMATION only; draining is
  never blocked by it.
  Drive `retire_row_valid` from the same row this section looks up, unconditionally
  in `retire.valid` — it is how the driver tells a vector placeholder apart from the
  scalar entries that share the LDQ and STQ, and it must therefore be readable on
  the cycle the driver is still deciding whether to pulse.
  Assert the retiring row is VALID and that its slot-0 base equals that queue's
  head. A retire against an invalid row means an LSQ placeholder deallocated
  without ever having reserved; a base that is not the head means reclamation
  has gone out of program order, which is the one thing the region discipline
  cannot survive. Both are silent corruptions otherwise.
  A streaming load changes NOTHING here: its region is freed whole, once, when
  its LDQ entry deallocates, however many waves it took to fill.

  ---- 7. The surplus release: tail-only ----

  //@req-spec-issue.b4
  //@req-spec-issue.b5
  //@req-spec-lsu.j6
  At execute, once VL has been read from the VL register file, the unused portion
  of the reservation is released. THE VECTOR AGEN PERFORMS IT — it is the stage
  that reads VL — by driving `release` with the count it will actually use: `vl`
  for a non-segmented SSI access, `vl * nf` for a segmented one, one entry for a
  unit-stride range, and zero when VL is zero or every element is inactive (that
  OP.v takes the VecGroupCopy no-execution path and issues no memory traffic at
  all, so its whole region is surplus). For a store the one release event trims
  the address and data queues together, which is sound because the pair is always
  allocated by the same event and so always has the same youngest owner.
  ===> BUT THE TWO QUEUES ARE TRIMMED BY DIFFERENT AMOUNTS FOR A US STORE, so
       `release.used_count` is PER QUEUE SLOT — a `Vec(2, UInt)` — not one
       number. SSI: both slots carry the same value, and assert that they do.
       US store: slot 0 is 1 (a range entry never has address surplus) and slot
       1 is the number of group MEMBERS actually written, which VecRangeAgen
       knows once VL is read. A single `used_count` would trim the US data queue
       by the address queue's count, i.e. release almost the entire data region
       while it is still in use — a silent overwrite of live store data, not a
       lost-performance bug. This is the same non-identity as section 2.
  Assert each slot's used_count <= that slot's count: a violation means the
  AGEN's element walk and this module's count computation disagree, and that
  must not be discovered later as a silent queue overrun.
  A LOAD'S RELEASE MAY BE A NO-OP AND THAT IS EXPECTED. With the load cap, a
  load whose active count meets or exceeds its reservation has no surplus at
  all; it drives `used_count` equal to its count and the release changes
  nothing. Do not treat "loads rarely release" as evidence of a broken release
  path — under decision D9/D10 it is the normal case for a large VL.

  //@req-spec-issue.b6
  //@req-spec-issue.b8
  The release is granted ONLY while the reserving OP.v's region is still the
  YOUNGEST in EVERY queue it holds, i.e. only when each slot satisfies
  `base + count == alloc_tail(q)` and the row has not already been released. Then,
  and only then, the release moves each slot's tail pointer back by that slot's
  surplus and subtracts it from that slot's `occ(q)`. Both slots or neither: a
  store that could trim its address queue but not its data queue is not permitted
  to trim either, or its row would stop describing its own regions.
  A MID-QUEUE RELEASE IS NOT PERMITTED. If a younger OP.v has already reserved
  past this region, `release_ok` is false and the surplus stays held.
  A mid-queue release would punch a hole in the occupied region and break the
  program-ordered-tail invariant that pointer-rollback squash depends on: the
  rolled-back tail would no longer bound exactly the killed entries. This is
  the one rule in this file that a plausible-looking "optimization" would
  break invisibly — nothing observable goes wrong until a mispredict lands on
  top of a hole.

  A release colliding with a reservation for the same queue in the same cycle is
  DENIED, not retried — the reservation has priority, so the region is no longer
  youngest — and it fires at most once per reservation (the `released` bit). It
  trims to the VL-derived count only, never to the mask-derived active count,
  because the mask arrives element by element and a second, later trim would
  necessarily be a mid-queue release. No retry mechanism is needed for either
  case: the surplus is simply freed in order with the rest of the region at
  reclamation, exactly as above. Denying rather than retrying also keeps the
  release out of the dispatch grant's combinational path.
  The spec records the honest cost: under a stream of vector memory ops the
  release usually cannot fire, so the RESERVED count rather than VL is what
  bounds in-flight vector memory — the worst case for stores, the capped count
  for loads. That is precisely why the load cap exists: it lowers the number the
  release cannot be relied upon to lower.

  ---- 8. Squash: pointer rollback, against BOOM's EXCLUSIVE tail ----

  ===> `rollback.ldq_idx`/`rollback.stq_idx` NAME THE FIRST DEAD ENTRY, NOT THE
       YOUNGEST SURVIVOR. This is BOOM's existing convention and the whole seam
       turns on it: `[head, idx)` survive and `[idx, old_tail)` die, exactly as
       `brupdate.b2.uop.ldq_idx` is the value `ldq_tail` held when the branch
       dispatched. THE DRIVEN INDEX'S OWN ROW IS KILLED.

  On `rollback.valid`, find the row owned by the YOUNGEST LSQ entry STRICTLY OLDER
  than the driven index — using BOOM's `IsOlderLSU`/`EntryValidFromAge` against
  that queue's head, taken from the `ldq_head`/`stq_head` ports the ports section
  declares for exactly this call, the same comparators VecLsu and VecSquashUnit use
  — and drive
  each queue's new allocation tail on `rollback_tail` as THAT row's
  `base + count`, per queue slot. If no such row exists (the surviving range is
  empty, which is what a ROB-head flush of the load side looks like) drive the
  queue's HEAD. Apply the same value to `alloc_tail(q)`, recompute `occ(q)` as
  `new_tail - head` in the same cycle, and invalidate the driven index's row AND
  every row younger than it. Because allocation was in program order this drops
  exactly the killed entries — the same shape as BOOM rolling `stq_tail` — and the
  reservations are released in the same event.
  Drain-and-discard is not an alternative: a squashed load's `pvdest` PRNs are
  recycled within a few cycles.

  ===> THE VALIDITY SWEEP IS AN AGE COMPARISON PER ROW, NOT A WALK OVER A CONTIGUOUS
       RUN OF VALID ROWS. Corrected at E2, where this paragraph's former phrase "with
       no per-entry comparison" was read as licensing a walk outward from the pivot
       that stops at the first invalid row. That is wrong, and the reason is that THE
       LDQ AND STQ ARE SHARED WITH SCALAR MEMORY OPS: one scalar load between two
       vector loads leaves an invalid row in THIS module's table, the walk halts
       there, and every younger vector reservation survives a squash that should have
       killed it. Nothing then ever frees those regions — the owning instruction no
       longer exists to retire them — so each mispredict leaks queue capacity until
       the machine deadlocks. Kill row `i` iff
       `row(i).valid && !IsOlderLSU(row_idx(i), pivot, head)`, evaluated
       INDEPENDENTLY per row.
       "No per-entry comparison" was only ever true of the TAIL computation, which is
       a single selection; the phrase has been removed because it does not survive
       contact with the sweep.
  ===> AN INCLUSIVE READING IS THE FAILURE THIS PARAGRAPH EXISTS TO PREVENT, and
       it is the reading the text originally had. Taking the driven index as a
       SURVIVOR and rolling to ITS `base + count` leaves ONE KILLED
       INSTRUCTION'S ENTRIES ALIVE in every one of the six queues — and they are
       the entries whose destination PRNs have just been returned to the free
       list, so the surviving addresses will be drained against re-allocated
       registers. Nothing asserts, nothing hangs; a later load simply reads
       someone else's data.
  ASSERT IT, both directions: assert the driven index's row is invalid after the
  rollback, and assert no surviving valid row has `base + count` greater than the
  new tail. VecSquashUnit drives the exclusive tail and asserts the same
  property from its side; the two assertions are cheap and they are what pins a
  convention that no width or type check can pin.
  Streaming loads need no special case: a killed load's region is bounded by its
  `base + count` however many waves it had run, because streaming never extends
  past `tail`.

  ---- 9. Depth is an architectural limit ----

  //@req-spec-lsu.b14
  //@req-spec-lsu.b15
  Because reservation happens at dispatch, the occupancy accounting above IS the
  mechanism by which queue depth bounds in-flight vector memory ops: an op that
  cannot reserve does not dispatch, so at most `qDepth / reserved_count` ops of a
  given shape are ever in flight. The two directions land on different numbers, and
  that asymmetry is the point of decision D9/D10:

    - STORES: 512 entries / 256 worst-case elements = TWO worst-case vector stores
      in flight. Read that figure from VectorParams' `maxInflightWorstCaseStores`
      rather than recomputing it, or the published limit and the real one can
      drift.
    - LOADS: 512 entries / `ldResvCap` = 512 / (4 * 32) = FOUR loads in flight at
      EMUL=8, SEW=8 — twice the store figure, bought by streaming rather than by
      depth. `ldResvMembers` of 2 would give 8 and 8 would give 2 (see the trap in
      section 1).

  Typical VL reserves far less and gets more overlap; both numbers are worst-case
  MLP bounds, not throughput predictions.

  ---- 10. Trace ----

  One guarded VecTrace line per key event, off by default behind the `vecTrace`
  plusarg, each carrying `rob_idx`: `resv_grant` (queue, base, count, per slot, and
  whether the load cap bound), `resv_deny` with the blocking queue, `resv_release`
  with the per-slot surplus and whether it was granted, `resv_retire` with the
  retiring LSQ index and the per-slot base/count it echoed, and `resv_rollback`
  with each queue's new tail. There are no unit tests here — validation is VCS plus
  Whisper cosim end to end — and a deadlock investigation begins by asking which
  queue refused which `rob_idx`, so the deny line is the one that must not be
  omitted. `resv_retire` is the second: a machine that stops dispatching vector
  memory with no deny line and no retire line is one whose reclamation path is not
  connected, which is exactly the failure this seam had.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Adds NO pipeline stage: the grant is combinational inside the existing dispatch
cycle and the release is a single-cycle decision. Throughput is `coreWidth`
reservations per cycle, matching dispatch, plus one release per AGEN lane, and no
arbitration between release lanes is needed because each owns distinct queues.

The dispatch cycle is the module's critical path (worst at `coreWidth = 4`): it
carries the count table lookup, the load-cap `min`, the per-queue prefix sums over
the lanes and the compare against free space. Keep the count and `ldResvCap` tables
elaboration-time constants and the prefix sums adds of at most `coreWidth` terms —
the load cap costs one compare and one mux per lane, no multiplier, and no serial
dependence on the release or retire path belongs in this cycle.

The retire path is off the dispatch critical path by construction: it is one table
read plus a subtract per queue, and its result is registered occupancy that
dispatch reads next cycle. It must NOT be forwarded combinationally into the same
cycle's grant — a retire that arrives one cycle late costs one dispatch slot; a
retire in the grant's cone costs the design its timing closure.

Area is dominated by the reservation table, (`numLdqEntries` + `numStqEntries`)
rows of roughly 40 bits at the default sizing — TWO base/count pairs per row rather
than one, which is the price of the US store's non-identity (logic section 2) and
of publishing region bases at all.

The performance limit this module imposes is stated rather than hidden (logic
section 9): if a workload is limited by dispatch refusals rather than D$
bandwidth, the answers are `ssiQueueEntries` and, for loads only, `ldResvMembers`
within the bounds section 1 records — never a relaxed allocation policy, and never
a relaxed STORE reservation.
<|end_perf|>

<|begin_dependencies|>
MicroOp — reads `is_vec`, `is_shared`, `uses_ldq`, `uses_stq`, the static access
descriptor `v_eew`/`v_emul`/`v_seg_nf`, the unit-stride/whole-register/mask class
flags, and `rob_idx`/`ldq_idx`/`stq_idx`. It writes none of them and ADDS NO
FIELD to the bundle: the region base lives in this module's table, not on the
uop, because the uop is replicated across every pipeline register and ROB row
while the region is needed only by the AGEN and the squash unit.

VecBundles — `VecReservation` is the output payload, field for field, and the
six-member queue enumeration is the only legal source of a queue id here.

VectorParams — the queue depths, `vLen`, `maxMembers`, `vecVLSz`, the derived
`maxInflightWorstCaseStores`, and `ldResvMembers` (default 4) with its
`require(ldResvMembers * vLen/eew_min >= lsuWidth * 2)`. The elaboration-time floor
requiring each element-granular queue to hold at least `vLen` entries lives THERE
and is not restated here; below that floor a single store could never complete its
reservation — the header's wedge, reached by undersizing instead. Both parameters
are BOUND, never re-defaulted: a second default for `ldResvMembers` in this file
would be a second policy.

VecTrace — the guarded printf helpers.

Instantiates nothing. Its counterparties, and what each owes this seam:
  VecLsu — instantiates it as `resv`; routes `resv_out` to the six queues and the
    lookup/release lanes to the four AGENs; DRIVES `retire` from the LSU's
    `ldq_head`/`stq_head` on vector placeholder deallocation and applies the
    `region_free` echo to each queue's `io.resv.free` in the same cycle. That
    driver is the only reclamation path in the subtree — without it the queues fill
    once and never free.
  VecPipeline — drives `dis_uops`/`dis_fire` from the registered ren2/dispatch
    pipeline and reduces `dis_ok` into vec_pipeline_io's `dis_ready`.
  VecElemQueue (x6) — records each region, writes at `base + element_index` inside
    it, CONSUMES the region free on `io.resv.free`, and reports NO reclamation of
    its own: it exports only `io.empty`, which is why the reclamation command has
    to be manufactured here from the retiring LSQ index.
  VecElemAgen / VecRangeAgen (x4) — read the region at execute and drive the
    per-slot surplus release once VL is known; a streaming load's agen reuses its
    own region cyclically and never releases below its reservation.
  VecSquashUnit — supplies the rolled-back (EXCLUSIVE) LDQ/STQ indices and applies
    `rollback_tail` to the queues in the same cycle this module applies it here,
    and asserts the exclusive-tail convention from its side.
<|end_dependencies|>

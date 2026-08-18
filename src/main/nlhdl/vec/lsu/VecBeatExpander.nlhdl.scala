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
  VecBeatExpander — the DRAIN side of the vector LSU: pops an element-queue head
  and issues D$ accesses, coalescing a unit-stride range into D$-port-width beats
  just-in-time.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecBeatExpander.scala,
  package boom.v4.vec.generated.lsu, group vec_lsu.
  depends_on VecBundles, VectorParams, VecTrace. Instantiates nothing.
  Instantiated TWICE by VecLsu: `ld_beat` (isStore = false) and `st_beat`
  (isStore = true). Direction is a parameter, never a separate module, so the two
  directions never arbitrate against each other — the load-priority mux is what
  silently dropped store grants in `addvector`.

  ===> THIS IS WHERE UNIT-STRIDE THROUGHPUT COMES FROM. Plan targets P1 and P2
       land in this file and nowhere else. P1: the D$ access count for a
       unit-stride op must equal the floor ceil(active_bytes / dmemBeatBytes) —
       4 accesses for an LMUL=1 full-vector op at vLen=256 with a 64-bit port,
       32 for LMUL=8, INDEPENDENT OF SEW. P2: sustained >= 1 access per cycle per
       granted lane while the queue is non-empty. `addvector` put the Packer on
       the FILL side, where coalescing cannot help, and emitted one beat per
       element: 8x the floor at SEW=8, 4x at SEW=32.

  ===> IT RUNS FROM QUEUE STATE ONLY, HOLDS NO PER-INSTRUCTION STATE, AND
       EXPORTS NO `busy` (vector-LSU invariant, plan rule 6). This is the module
       most at risk of violating that invariant, because a "current op" FSM with
       a beat counter is exactly what a reader would expect here and exactly what
       gave the previous attempt a concurrency ceiling of 1. The element cursor
       this module advances is a FIELD OF THE QUEUE/LDQ ENTRY it is draining
       (MicroOp's `elem_next`), read in and written back out; the module itself
       declares no state scoped to an instruction. See the logic section, which
       states the register budget explicitly.

  Governing spec anchors: loadstore.rst `us-queue`, `store-data-queue`,
  `vec-load-algo`, `elem-progress`, `vector-bw-ceiling`;
  execution.rst `vector-agen` (the Packer).

<|begin_module|>

  <|begin_parameters|>
  `isStore: Boolean` — required, no default. Selects which queue set is drained
  (`ld_US_ADDR_Q`/`ld_SSI_ADDR_Q` versus `st_US_ADDR_Q`/`st_SSI_ADDR_Q` plus the
  two store data queues) and enables the store-only ports. It changes no part of
  the coalescing algorithm: expansion is a property of the ACCESS CLASS, not of
  the direction.

  The whole module is elaborated only under `usingRVV` (a Scala `Boolean` derived
  from `BoomCoreParams`, never a hardware `Bool`). In a vectors-off build it is
  ABSENT, not tied off, so the emitted RTL is bit-identical to pre-Caracal
  BOOM v4. Do not gate on rocket's `usingVector`.

  `nLanes: Int` — the number of D$ request lanes this drain may present per cycle.
  Bound to BOOM's `lsuWidth` (1 on Medium, 2 on Large/Mega), legal range 1..2, and
  required at elaboration to agree with `VectorParams.dcacheArbiterMode` ("single"
  implies 1, "dual-dynamic" implies 2) rather than tolerating a mismatch.

  `dmemBeatBytes: Int` — the byte width of one D$ request lane, derived from
  rocket's `coreDataBytes` (`xLen/8`, 8 today). This is THE bound on beat width.
  ===> IT MUST NOT BE WRITTEN AS 8 ANYWHERE, and no expression may assume 3
       offset bits. A wider vector cache port raises this ceiling and every
       width, mask and shift below must follow it automatically. Require it to be
       a power of two and no greater than the page size, which is what makes the
       "a beat never crosses a page" argument in the logic section true.

  From VectorParams: `vLen`, `eLen`, `maxMembers`, `usQueueEntries`, `vecPregSz`.
  Derived, all as named vals rather than inline arithmetic:
    `vLenBytes    = vLen / 8`            bytes per destination member (PRN)
    `beatOffBits  = log2Ceil(dmemBeatBytes)`
    `maxElems     = vLen`                worst-case active elements (SEW=8, LMUL=8)
    `elemIdxSz    = log2Ceil(maxElems + 1)`
    `sizeBits`    the width of the D$ request's `mem_size` field

  ===> `mem_size` in MicroOp is 2 bits, so the largest expressible beat is 8
  bytes today. dmemBeatBytes larger than 8 therefore ALSO requires widening
  mem_size, which is a MicroOp/ScalarOpConstants change outside this node's
  scope. Assert `log2Ceil(dmemBeatBytes) < (1 << sizeBits)` at elaboration so
  the configuration fails the build instead of silently truncating a beat.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit pair, following the hierarchy defaults:
  posedge `core_clk`, ACTIVE-HIGH SYNCHRONOUS `core_reset`. The module declares
  no registers in its functional path (see the logic section), so reset reaches
  only the trace gate.

  The unit-stride source is PEEKED, not popped: `us_head` (Input,
  `Valid(VecRangeEntry)`) presents the head of `{ld,st}_US_ADDR_Q` continuously
  and the entry stays there across all of its beats. `us_cursor` (Input,
  `UInt(elemIdxSz.W)`) is that entry's `elem_next` and `us_cursor_wr` (Output,
  `Valid(UInt(elemIdxSz.W))`) writes the advanced value back in the same cycle
  the beats fire — the register lives in the queue entry, not here. `us_pop`
  (Output, `Bool`) asserts for one cycle when the last active element has been
  consumed and the entry may be freed.

  The SSI source is `ssi_head` (Input, `Vec(nLanes, Valid(VecElemAccess))`), the
  oldest `nLanes` already-element-wise entries of `{ld,st}_SSI_ADDR_Q`, with
  `ssi_pop` (Output, `Vec(nLanes, Bool)`).

  Store-only ports, elaborated only when `isStore`: `st_us_data` (Input,
  `Valid(UInt(vLen.W))`, the head of `st_US_DATA_Q` — one whole source vPRN) with
  `st_us_data_pop`; `st_ssi_data` (Input, `Vec(nLanes, Valid(UInt(eLen.W)))`)
  with `st_ssi_data_pop`; and `is_write_pass` (Input, `Bool`) — false for the
  pre-commit translate pass, true for the post-commit write pass. VecLsu drives it
  from the entry's committed flag; this module holds no notion of commit.

  `req` (Output, `Vec(nLanes, Decoupled(...))`) is the request toward
  VecDcacheArbiter, and `ready` IS THE GRANT — one grant covers the D$ lane, the
  TLB port and the LCAM port together, which is why they cannot be three
  handshakes. The payload is `VecElemAccess` (virtual address, `eew`, byte enable,
  first/last markers, and the wrapped `MicroOp` whose `prn` and
  byte-offset-within-PRN fields carry the placement) plus four fields it needs and
  does not yet have: `data` (`dmemBeatBytes*8`, stores only),
  `uses_tlb`/`uses_dcache`/`uses_lcam`, and `lcam_range_len` for the one-shot US
  range query. Those belong in VecBundles beside `VecElemAccess`, NOT declared
  privately here — a payload declared inside its producer is readable from one
  side only, which is what VecBundles exists to prevent.

  ===> THE US LANE SYNTHESIZES ITS `MicroOp`; THE SSI LANE COPIES ONE. That
  asymmetry is forced — an SSI beat comes from a `VecElemAccess` that already
  carries the whole uOP, while a US beat comes from a `VecRangeEntry`, which
  carries `rob_idx`/`ldq_idx`/`stq_idx`/`pvdest` and NO uOP — so the US lane must
  build one from `DontCare` and is responsible for every field anything downstream
  reads. That is not just the placement fields: `lsu.scala` puts this uOP on
  `dmem_req.bits.uop` and takes `exe_cmd`/`exe_size` off it, so `mem_cmd` and
  `mem_size` reach the TLB and D$ from here, and its nack and response paths route
  on `uses_ldq`/`uses_stq` and guard on `is_vec`. Set `is_vec`, `uses_ldq`/
  `uses_stq` and `mem_cmd` from the instance's own `isStore` parameter alongside
  the placement fields. Leave any of them at `DontCare` and the access is issued
  with a command the minimizer chose and an identity the LSU cannot route: the
  visible symptom is the scalar nack handler's `assert(uses_stq)` firing on a
  vector beat, which reads as an LSU bug and is not one.

  `st_us_data` (Input, `Valid`, stores only) IS A `Valid` AND ITS `valid` IS LOAD-BEARING,
  not decoration. The US data queue is a synchronous read: VecLsu issues the read when
  the write pass starts, so the bytes arrive a cycle later. A write-pass beat composed
  before then carries whatever the read port held — zero out of reset — straight to the
  D$ with `uses_dcache` set. Gate the US beat on it: during the write pass a store beat
  requires `st_us_data.valid`; during the translate pass no data is read and the term is
  vacuous. Consuming only `.bits` and ignoring `.valid` sends whatever the read port
  happens to hold, and nothing asserts: the store commits, drains, retires, and the
  corruption is found only by reading the memory back.

  `st_us_data_pop` (Output, stores only) ADVANCES THE DATA MEMBER POINTER AND IS
  THEREFORE A WRITE-PASS-ONLY EVENT, qualified by `is_write_pass` exactly as `us_pop`
  already is. A unit-stride store walks its range TWICE — translate, then write — and
  only the second pass consumes data. Leave the pop unqualified and the TRANSLATE pass
  advances the pointer as well, so by the time the write pass runs it reads
  `us_data_base + <members already walked>`, an index nothing ever filled. The queue
  reports `filled = 0` forever: with the `.valid` gate above the store stalls, and
  without it EVERY member is written from a stale read port, not merely the first.
  These two are one defect seen from both ends — the missing gate makes it silent, the
  unqualified pop makes it total — and fixing only the gate converts corruption into a
  hang rather than into correct data.

  `lcb_free_nonzero`, `lcb_walk_active` (Inputs, `Bool`) and `lcb_walk_rob`
  (Input, `UInt(robAddrSz.W)`), loads only, are the LCB allocation credit as
  three RAW terms rather than one pre-reduced ready bit. A load beat is not
  requested without an assembly entry for the PRN it targets, because the LCB may
  never back-pressure a response. Form the test HERE, per path:

      def lcbRdyFor(rob) = lcb_free_nonzero || !lcb_walk_active || rob =/= lcb_walk_rob

  and apply it as `lcbRdyFor(us_head.rob_idx)` in `usGate` and
  `lcbRdyFor(ssi_head(i).uop.rob_idx)` in `ssiLaneFire(i)`.

  ===> THE OP IDENTITY MUST COME FROM THE PATH'S OWN HEAD, WHICH IS WHY THE TERMS
       ARRIVE RAW. A beat needs a free entry only while ITS OWN op is still
       allocating; the walk is one global resource, so without the `rob_idx` term
       a later op's stalled walk blocks beats belonging to an earlier op whose
       entries already exist — a circular wait, since those beats are what lets
       that op complete, retire and free what the walk is waiting for. The
       unit-stride path and each SSI lane carry DIFFERENT ops at the same time, so
       a ready bit reduced upstream necessarily carries one identity and
       mis-answers for the other. That is not hypothetical: the unit-stride path
       was fixed first with the comparison done in VecLsu against
       `us_head.rob_idx`, and the strided (`vlse`) path went on starving until the
       comparison moved in here.

  `stop` (Input, `Bool`) says
  the head's cursor has latched `fault_elem`. `kill` (Input, `Bool`) is
  `brupdate`/`rob_flush` on the head and suppresses requests this cycle; pointer
  rollback is VecSquashUnit's, and nothing is undone here because nothing is held
  here.

  ===> THERE IS NO `busy` PORT, NO `active`, NO `grp_active` AND NO `fu_ready`
       CONTRIBUTION. Any output of this module reaching an issue unit is a failed
       review regardless of measured performance (plan rule 6). Issue eligibility
       is decided at dispatch by VecQueueReservation and nowhere else.
  <|end_ports|>

  <|begin_logic|>
  ---- Register budget, stated first because the invariant depends on it ----

  The functional path declares NO state scoped to an instruction: no direction
  FSM, no beat counter, no "current op" latch, no operand shadow copy. Every value
  the expander needs is either a field of the queue head presented this cycle or
  the head entry's `elem_next`, which it reads and writes back. Two registers are
  permitted and both are position-independent: an optional registered NEXT-BEAT
  descriptor (see the performance section) and the round-robin pointer assigning
  work to lanes. Both are invalidated unconditionally on `kill` or on any change
  of the head entry, so neither can survive a squash or outlive the instruction it
  was computed for.

  ---- Two input classes, one output stream ----

  //@req-spec-agen.c1
  The SSI path is a pass-through: a strided, indexed or segmented access arrives
  from `VecElemAgen` already expanded to one `VecElemAccess` per ACTIVE element,
  so each granted lane drains exactly one entry and no coalescing is attempted —
  scattered addresses cannot be range-folded. INDEXED ACCESSES ARE NOT THIS
  MODULE'S BUSINESS: the per-element offsets come from `VecIdxGen` on the fill
  side, and this module never reads an index vector, never reads the VRF and has
  no VRF port. The unit-stride path is the dense, contiguous, non-indexed case,
  and it is the only one that coalesces.

  //@req-spec-agen.b8
  //@req-spec-lsu.c5
  //@req-spec-lsu.c6
  //@req-spec-lsu.j2
  The unit-stride path fires when a unit-strided LDQ/STQ entry is ACTIVATED and
  expands that single `nOP.v` — the one `VecRangeEntry` describing
  `[base, base + VL*EEW)` that `VecRangeAgen` pushed — into D$ accesses JUST IN
  TIME, at the queue, as lanes are granted. Nothing is expanded at fill time and
  nothing is buffered per element: the expansion exists only as the cursor value
  plus this cycle's beat descriptor. Expanding early is what `addvector` did, and
  an early expansion cannot coalesce, because it knows neither the port width it
  will be granted nor how many elements the arbiter will let it retire together.

  ---- The beat composer: one cursor, five constraints ----

  //@req-spec-agen.c2
  //@req-spec-agen.c3
  A single element counter — the Packer's `EEW_CTR`, realized as the head entry's
  `elem_next` — walks the FLATTENED contiguous element stream of the range. Each
  cycle the composer computes, for the byte position `cur_byte = elem_next << eew`
  and address `addr = base + cur_byte`, the run length in bytes as the MINIMUM of
  five constraints, and advances the cursor by whichever is hit FIRST:

    1. VL        `active_bytes - cur_byte`, from the entry's byte length.
    2. vreg      `vLenBytes - (cur_byte % vLenBytes)` — bytes to the destination
                 member boundary.
    3. DMEM      `dmemBeatBytes - (addr % dmemBeatBytes)` — bytes to the D$ lane
                 boundary.
    4. segment   `1 << eew` when the entry's `nf > 1`, unbounded when `nf == 1`.
    5. mask      the length of the run of consecutive ACTIVE mask bits starting
                 at `elem_next`, converted to bytes by `<< eew`.

  Constraints 2 and 4 are representability, not optimization: the beat carries ONE
  placement (`prn`, byte-offset-within-PRN), so a beat straddling two destination
  members would make the LCB write the wrong register, and for `nf > 1`
  consecutive memory bytes belong to DIFFERENT destination registers, so the
  segment term degenerates the walk to one element per beat instead of emitting a
  beat with two placements. (`nf > 1` does not in practice reach the US queue —
  `execution.rst`'s selection rule sends segmented accesses to the Skipper /
  `VecElemAgen` — so term 4 is a safety net, not a throughput case.) Constraint 3
  is structural too: a beat is one `dmem.req`, and because `dmemBeatBytes` divides
  the page size a lane-aligned beat can never cross a page, which is what makes
  one TLB lookup per beat sufficient.

  //@req-spec-lsu.k8
  The result is that CONTIGUOUS BYTES ARE COALESCED UP TO THE LANE WIDTH: eight
  SEW=8 elements become ONE beat on a 64-bit port instead of eight, and the access
  count for an aligned dense unit-stride op is `ceil(active_bytes / dmemBeatBytes)`
  regardless of SEW. Be precise about what that buys: it reduces
  ADDRESS-GENERATION and DISAMBIGUATION cost, not cache-port width. One lane still
  carries at most one beat of at most `eLen` bits per cycle — a coalesced beat is
  one D$ transaction, not several.

  Convert the minimum back to whole elements (`elems = run_bytes >> eew`) so the
  cursor never lands mid-element. If `elems` computes to zero the element itself
  straddles a lane boundary (a misaligned element under a misaligned base): emit
  it as a single element-sized access at its own address and let it take exactly
  the path a misaligned SCALAR access of that size takes. No vector-specific
  misalignment mechanism is added.

  ===> BUG WARNING, and it has already been paid for once: every offset on the
  beat is in BYTES. The placement offset is `(elem_idx << eew) % vLenBytes`
  plus the intra-beat byte position, never an element index. Handing the LCB an
  element count where it expects bytes corrupted element 0 of every misaligned
  load in the M1 bring-up and looked like an LSU fault, not an offset bug.

  For a LOAD, issue the beat at the lane-aligned address with the full lane size
  and let the byte enable say which bytes are wanted. Over-fetching inside an
  aligned window is safe — the window lies entirely within the page the access
  already touches — and it is what gives the naturally-aligned power-of-two
  request the D$ requires. For a STORE it is NOT safe and the rule is different:
  BOOM's D$ derives the write byte mask from `mem_size` and `addr` alone
  (`StoreGen` in `dcache.scala`), so a store beat must be the LARGEST
  NATURALLY-ALIGNED POWER-OF-TWO block that the run covers. An aligned dense
  store therefore still reaches the P1 floor; only a misaligned head or tail
  costs extra beats. Expressing an arbitrary byte-enable store would require a
  partial-write path in the cache, and the cache is unchanged by construction.

  ---- Mask handling: suppress, never flag ----

  //@req-spec-agen.c23
  The mask arrives on the range entry (`VecRangeAgen` performed the single `v0`
  read on the load path's `R1` / the store path's `R4`; this module reads no VRF
  port and must not). Where the bit at the cursor is CLEAR the access is
  SUPPRESSED — no D$ request, no TLB lookup, no LCAM search — and the cursor skips
  the whole run of consecutive clear bits in one cycle, found with a priority
  encoder over the mask slice, so a sparsely-masked op costs no cycle per inactive
  element. The inherited bobtail Packer instead fetched masked-off lanes and
  merely flagged them; suppression is the extension this module owes, and it is
  what makes masked vector memory correct in both directions — a masked-off store
  byte must leave memory unmodified, and a masked-off load must not be able to
  fault on an address the instruction never architecturally touches. Inactive
  DESTINATION lanes are the LCB's concern (pre-loaded from `stale_pvdest` on
  `R2`), never a reason to issue an access here.

  ---- Store data slicing at drain ----

  //@req-spec-lsu.d6
  The `st_US_DATA_Q` entry is a full `vLen` wide — `VecDgen` read the whole source
  vPRN in one access at execute — so the per-element store data is SLICED OUT
  HERE, AT DRAIN, in two byte-granular steps: extract `run_bytes` from the
  `vLen`-wide entry at the member-relative offset `cur_byte % vLenBytes`, then
  rotate that field to the beat's lane position `addr % dmemBeatBytes`.
  `st_us_data_pop` fires when a beat consumes the last active byte of the current
  member, which is exactly when constraint 1 or 2 bites — the vreg-boundary stop
  is what keeps the data queue's head aligned with the address cursor without a
  second counter. On the SSI store path the data is already per element.

  ---- The fire ----

  //@req-spec-lsu.j3
  A beat is presented to `VecDcacheArbiter`, and on grant THE FIRE PERFORMS THE
  TLB, D$ AND LCAM LOOKUPS TOGETHER — the vector analogue of
  `will_fire_load_agen_exec`. One grant, three ports, never three independent
  handshakes: the arbiter gates all three under one priority round-robin policy
  precisely so a vector stream cannot monopolize translation or disambiguation.
  Vector element addresses go through the DTLB like any other access;
  bare-physical addressing is not available, because it would make cross-queue
  disambiguation impossible. LCAM use differs by class: a US entry raises
  `uses_lcam` with `lcam_range_len` on its FIRST beat only, so the whole range is
  disambiguated by one range-overlap query, while every SSI beat queries per
  element. In a store's pre-commit translate pass the beat asserts `uses_tlb` and
  `uses_lcam` and clears `uses_dcache`, and `us_pop` stays low — the cursor
  advances but the entry is retained until commit-drain. Only the post-commit
  write pass writes the cache and pops. Resetting the cursor between the two
  passes belongs to VecLsu, which owns the committed flag; this module only ever
  advances a cursor.

  ---- THE LCAM PRESENTATION IS A PROPERTY OF THE RANGE, NOT OF A BEAT ----

  //@req-spec-lsu.j3
  This paragraph was ABSENT, and its absence is the direct cause of a suite-wide
  defect: the sentence above says "on its FIRST beat only" and nothing said WHICH
  lane holds the first beat, or what happens if that lane does not fire. It was
  implemented as `(i == 0).B && (us_cursor === 0.U)` — the duty pinned to lane
  INDEX 0 while the cursor is advanced by ANY lane. At `nLanes = 2` that is a
  coin flip on the first cycle of every unit-stride vector store. Measured on
  `ms4p9_vl`: at 3735000 both store lanes are refused and both load lanes
  granted; at 3737000 LANE 1's grant advances the shared cursor; at 3739000
  `isFirstBeat` has fallen, so lane 0 is finally granted carrying `uses_lcam=0`.
  The presentation is lost forever.
  **Lane 0 alone can discharge the duty; lane 1 can destroy the opportunity; and
  nothing remembers the duty was owed.**

  THE RULE. The presentation duty belongs to the RANGE. Exactly one GRANTED
  pass-1 beat per range carries `uses_lcam`, and it must be the first beat that
  actually FIRES, not the beat held by a particular lane index. Two corollaries,
  both of which the old form violated:
    - the duty must be expressible per lane. Identify beat 0 by THIS LANE'S OWN
      start element (`usStartElem(i)`, already computed as the serial-prefix
      input to lane `i`), never by the lane index. At most one FIRING lane can
      start at element 0: if lane 0 fires it advances, so lane 1 starts above 0;
      if lane 0 does not fire, lane 1 inherits 0 and correctly owns the duty.
    - a shared cursor advanced by any lane must not retract another lane's
      pending presentation.

  ⚠ THIS PARAGRAPH WAS CORRECT AND THE IMPLEMENTATION DEVIATED FROM IT. When the
  duty moved off `usStartElem(i)` onto the latch plus a serial-prefix accumulator,
  the "beat that actually FIRES" requirement was silently dropped: the accumulator
  suppressed the later lane on `dutyThisLane`, which is built from `laneWillReq`
  and carries NO `req.ready` term. That expresses "this lane was CHOSEN", not
  "this lane carried it". Lane 0 refused + lane 1 granted ⇒ lane 0 holds the duty
  and presents nothing, lane 1 is suppressed and fires anyway, and pass 1 ends
  still owing. Fired `:1658` on five rows (`ms2p5_loadblock`, `ms4p5_vle64_2`,
  `ms4p6_vle32_2`, `ms4p6_vle32_8`, `ms4p9_vl`).
  ⇒ ANY implementation of this rule -- `usStartElem`, latch+accumulator, or a
  priority encoder -- MUST couple the suppression to `io.req(i).fire`, not to the
  selection. `usStartElem(i)` satisfied this only incidentally, because a
  non-firing lane 0 does not advance the cursor and so lane 1 inherits element 0;
  any form that keys off a CHOICE has to state the coupling explicitly.
  ⇒ AND THIS IS WHY THE `PriorityEncoder` FOLLOW-UP WAS REJECTED rather than
  deferred: it picks the lane from mask state, that lane can still be refused, so
  it needs the same fire-coupling -- which reintroduces the `req.ready` dependence
  it was proposed to remove. Uniqueness-by-inspection is worth less than
  correctness-by-coupling. Do not re-propose it as a free improvement.

  ⇒ **AND THEREFORE THE PRESENTED ADDRESS MUST COME FROM THE RANGE, NEVER FROM
  THE BEAT CARRYING IT.** This is the invariant the old lane-0 pin preserved BY
  ACCIDENT, and it is the trap waiting for anyone who makes the duty mobile
  without noticing. `VecLsu` sets the candidate's `paddr` from the granted
  beat's `vaddr`, which equals the range base ONLY when the duty rides element 0.
  Let it ride a later beat and `dwordBounds(paddr, len)` describes a window
  shifted forward by the elements already consumed: it MISSES loads to the low
  part of the range and FALSELY MATCHES past its end. That is worse than losing
  the presentation, because a lost presentation is silent absence while this is a
  confident wrong answer. Take the base from the staged `VecRangeEntry.base`,
  which is the range's own effective base address and is already in hand at that
  seam.

  ⚠ AND NOTE WHAT THE STAGED BASE ACTUALLY CONTAINS, because it is NOT one thing.
  `VecLsu` REWRITES the staged range entry's `base` to the translated PHYSICAL
  address once the first pass-1 beat's `xlate_resp` returns. So the base is
  VIRTUAL before that point and PHYSICAL after it, and which one a presentation
  sees depends on WHICH BEAT carried the duty — element 0's beat sees virtual, a
  later beat (the skip case) may see physical. An earlier note here said "virtual,
  exactly as the beat's `vaddr` was"; that is only half true and is corrected
  here. It is benign today because these tests are bare-mode with VA == PA, and
  the physical one is arguably the more correct of the two, but a value whose
  address space depends on arbitration timing is not something a later reader can
  reason about. This SHARPENS the untranslated-`paddr` open item rather than
  belonging to it: the real resolution is to present the range base AFTER
  translation, which is the same design question — a presentation that must wait
  for `xlate_resp` is a RETRYABLE PRESENTATION, the exact shape deliberately
  removed from this fix — and it must be decided as one.
  ⇒ **AND THAT IS THE TRAP: it must NOT be decided piecemeal by whoever next
  touches `cand.paddr`.** The change looks local and one-line from where that
  code sits — swap the base for the translated one — and it silently
  reintroduces a retry path into a seam whose whole design rests on the
  presentation never needing one. Anyone reaching for that edit is holding one
  end of an open architectural question, not fixing an oversight. Decide the
  retryable-presentation question first, in one place, then change `cand.paddr`
  as a consequence of the decision.

  AND A SKIP CAN STILL STEAL IT, WHICH IS WHY A LATCH IS REQUIRED AND NOT ONLY A
  PER-LANE TERM. `advOk = laneSel && Mux(isSkip, true, req.fire)` — a SKIPPED
  element advances the cursor with NO request and therefore NO presentation. A
  masked unit-stride store whose element 0 is inactive loses the duty even under
  the corrected per-lane term. So the range carries an `lcamOwed` latch: set when
  the US range is staged for pass 1, ANDed into `uses_lcam` on any firing pass-1
  beat, cleared when the candidate fires, and re-armed on `rob_flush`/branch kill
  so a squashed store cannot strand it. Reset value is OWED, which is also the
  safe direction for any state the design cannot account for.

  ONE BIT PER DIRECTION, NOT ONE PER STQ ROW. `VecLsu` stages exactly one US
  range per direction — `st_US_ADDR_Q.io.rd(0)` at `stUsDrainPtr`, which advances
  only at `us_pop`, and `us_pop` for a store requires the write pass. Pass 1 of
  range N therefore strictly precedes pass 2 of range N, which strictly precedes
  the staging of range N+1, so two US ranges can never be in pass 1 together and
  back-to-back stores are already serialized by the drain pointer. A per-row
  array would spend `numStqEntries` bits re-encoding a mutual exclusion one
  pointer already guarantees. If the drain pointer is ever allowed to run ahead,
  this is the decision to revisit, and `lcamOwedSingleRange` is the assertion
  that will catch it.

  ⚠ THE LATCH SELECTS WHICH BEAT CARRIES THE DUTY. IT MUST NOT BLOCK ANYTHING.
  An earlier design for this made the latch an INTERLOCK — "the write pass may
  not be entered while a presentation is owed". IT WAS REJECTED FOR TWO
  INDEPENDENT REASONS, AND EITHER ALONE IS SUFFICIENT: it DEADLOCKS (below), and
  it COSTS A STALL on every unit-stride vector store even when it does not.
  A spec correction is required to leave the design FASTER, not merely correct;
  the selector form adds one comparison and one AND term, stalls nothing, and
  refuses no grant, while the interlock held a pass and added a starvation door
  that then needed arbiter anti-starvation priority to reopen. When two forms are
  both correct, that difference decides it — and here only one of them is even
  correct.

  THE DEADLOCK, precisely, because it must not be re-derived. `VecLsu.scala:802` promotes to the write pass
  when the cursor reaches the end of the range; block that and the cursor is
  parked at the end with the write pass still false, and a lane fires only while
  bytes remain (`res.fire`), so NO further pass-1 beat is ever produced. The
  retry has nothing left to ride: the store never drains, `stq_head` never
  advances, commit blocks. On a single-beat range that is one cycle from staging
  to wedged. **A liveness argument must show an opportunity still EXISTS, not
  merely that nothing blocks it — the interlock satisfied the second and failed
  the first.**
  ⇒ THE GENERAL LESSON, WORTH MORE THAN THIS DEFECT: **where a duty rides a
  FINITE EVENT STREAM, an interlock that stalls the stream can never be the
  enforcement mechanism** — stalling the stream removes the very opportunity the
  duty needs, so the interlock destroys what it was meant to protect. DETECT, DO
  NOT BLOCK. Keep the safety property as an ASSERTION on the state the interlock
  would have gated (`!(write_pass && lcam_owed)`): it catches exactly the same
  violation, at the same instant, and cannot wedge.

  ARM BY IDENTITY, NOT BY EVENT, AND TAG THE LATCH. Key the arm on the staged
  entry's `stq_idx`/`ldq_idx` CHANGING, never on one retire event: a range
  retired by any other path (a trimmed `vleff` retires without a `us_pop`) would
  otherwise leave the next range stranded with a CLEAR latch, silently
  reintroducing the lane-0 defect for every range after the first. Arm
  unconditionally on an identity change, never `when (!owed)`. Carry the tag and
  assert at the clear that the firing candidate's `stq_idx` matches it, so a
  presentation can never discharge a different range's duty.

  With the latch as a pure selector, termination is trivial and needs no
  arbiter anti-starvation and no write-pass coupling: if pass 1 has at least one
  firing beat, the duty rides the first one; an arbiter refusal means no fire
  hence no advance, so the range re-offers the duty next cycle by construction.
  If pass 1 has NO firing beat, every element was skipped, the store touches no
  memory and owes no ordering — clear `lcamOwed` at `us_pop` and assert the range
  had no active bytes. `lcamOwedWatchdog` (4096 default, `0` disables and emits
  no register) is the tripwire on that last piece of reasoning.

  ---- Faults ----

  //@req-spec-lsu.f13
  When `stop` asserts, that entry's cursor stops advancing and no further beat is
  requested for it. THERE IS DELIBERATELY NO CANCELLATION PATH, and its absence is
  the specification rather than an omission: because beats fire in element order,
  no access beyond `fault_elem` was ever issued, so every outstanding access
  belongs to an element BELOW it. Those are LEFT TO RETURN and are dropped exactly
  as on a branch squash — the trap invalidates the load's LCB assembly entry by its
  owning `ldq_idx`, so a late response has nowhere to land. A kill-on-response path
  here would be useless and unsafe: the faulting op's `pvdest` group is reclaimed
  to the free list, so a late write must be made IMPOSSIBLE by invalidating the
  landing site, not merely harmless. The trap takes `vstart = 0` and restarts the
  whole instruction; `fault_elem` never reaches the ROB, and this module neither
  reads nor produces `vstart`.

  ---- Trace ----

  Emit one `VecTrace` line per key event, gated on the `vecTrace` plusarg (off by
  default) and `!reset`: beat granted (with `rob_idx`, element index, run length in
  BYTES and beats-so-far), mask run skipped, entry popped, stop asserted. The beat
  line is how a P1 measurement is read off a waveform-free run, so the run length
  must be on it — a line logging only the address cannot distinguish the floor from
  8x the floor. Tracing declares no counter and no wire a functional path reads.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
TARGET P1 — access-count floor. For a unit-stride op the number of D$ accesses
must equal `ceil(active_bytes / dmemBeatBytes)`: 4 for an LMUL=1 full-vector op
at `vLen = 256` with a 64-bit port, 32 for LMUL=8, INDEPENDENT OF SEW. This is a
constraint on the implementation, not an aspiration. It forces the composer to be
able to retire up to `dmemBeatBytes >> eew` elements in ONE cycle at the narrow
SEWs — an implementation that advanced the cursor by one element per grant would
be functionally correct and would miss the target by 8x at SEW=8, which is
exactly the failure this module exists to fix.

TARGET P2 — sustained >= 1 access per cycle per granted lane while the arbiter
grants and the queue is non-empty. This forbids any multi-cycle round trip per
beat, which makes two things design decisions rather than options. First, the
cursor-advance loop must close in ONE cycle — read `elem_next`, compose, grant,
write back — with a same-cycle bypass so the next beat sees the updated cursor;
if the queue's cursor register cannot be read and written in one cycle, compute
beat n+1's descriptor while beat n is being granted (a one-deep registered
look-ahead, invalidated on `kill` or a head change) rather than inserting a stall
state. Second, `nLanes = 2` computes the two lanes as a serial prefix over the
same cursor, lane 1 starting where lane 0 ended; that adder chain is the only
cross-lane dependency in the ADDRESS COMPOSITION and must not become an
arbitration there.

===> CORRECTION: the premise "since both lanes are the same drain" is FALSE, and
believing it is what made a lane-0-pinned duty look safe. The two lanes are the
same drain in the sense that they share one cursor, but they are NOT granted
together: `VecDcacheArbiter` has independent per-lane `ready`, and it is
MEASURED that lane 1 was granted while lane 0 was refused in the same cycle
(`ms4p9_vl`, 3737000). Any reasoning of the form "lane 0 will get its turn
because the lanes move together" is therefore unsound. The lanes share STATE and
are arbitrated INDEPENDENTLY, which is the worst combination and the reason the
lane-discipline rule below exists.

The critical path is queue-head read to `dmem.req`: mask priority encode, a 5-way
min over byte counts bounded by `vLenBytes`, one add, and for stores a
`dmemBeatBytes`-wide rotate (the `vLen`-wide store-data extract is a mux, not a
shifter chain). If timing forces a cut, take it on the registered look-ahead
above, never by inserting a wait state between grants — that would trade a P2 miss
for a frequency win.

Peak throughput remains `nLanes * dmemBeatBytes` per cycle — 64 bits/cycle on
Medium, 128 on Large/Mega — because the D$ and its request interface are
unchanged. A wider vector cache port raises that ceiling and every width here
follows `dmemBeatBytes` automatically; nothing in this file assumes 64 bits.
<|end_perf|>

<|begin_dependencies|>
VecBundles — `VecRangeEntry` on the unit-stride input, `VecElemAccess` on the SSI
input and on the request toward the arbiter. Two field-level needs are recorded
against VecBundles rather than satisfied privately here: `VecRangeEntry` must
carry the mask bits, `nf` and the active byte length the five constraints read
(the mask travels with the US `nOP.v`, per `execution.rst`), and the
drain-to-arbiter payload must carry store `data`, the
`uses_tlb`/`uses_dcache`/`uses_lcam` qualifiers and `lcam_range_len`.

VectorParams — `vLen`, `eLen`, `maxMembers`, `usQueueEntries`, `vecPregSz` and the
`dcacheArbiterMode` cross-check. Every width derives from it or from
`coreDataBytes`; no literal widths. VecTrace — the guarded-printf convention;
with no unit tests in this project those lines are the only per-cycle visibility
into whether P1 is met.

Instantiates nothing. Its seams, each to be checked from the other side in
Phase R: VecElemQueue (peek/pop plus the cursor read-modify-write on the US head),
VecDcacheArbiter (`req.ready` is the grant covering D$ + TLB + LCAM),
VecLoadCoalescingBuffer (`lcb_free_nonzero`/`lcb_walk_active`/`lcb_walk_rob` in, placement `prn` + byte offset out),
VecCrossLsuSnoop / VecStoreForward (the LCAM query, one range check for US),
VecDgen (the `vLen`-wide `st_US_DATA_Q` entry this module slices), and VecLsu
(`is_write_pass`, `stop`, `kill`, and the cursor reset between store passes).
<|end_dependencies|>

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
  VecElemQueue — the reserved, program-ordered, pointer-rollback-squashable
  element buffer that decouples vector address/data generation from vector
  memory drain. ONE definition, SIX instances.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecElemQueue.scala,
  package boom.v4.vec.generated.lsu. depends_on VecBundles, VectorParams,
  VecTrace. Instantiated six times by VecLsu as `ld_SSI_ADDR_Q`,
  `st_SSI_ADDR_Q`, `st_SSI_DATA_Q`, `ld_US_ADDR_Q`, `st_US_ADDR_Q` and
  `st_US_DATA_Q`. Elaborated only when `usingRVV`; in a vectors-off build the
  module is absent, not tied off.

  ===> THIS IS WHERE THE VECTOR-LSU INVARIANT IS MADE TRUE. The fill side pushes
       into a region it already owns and RETIRES; the drain side works that
       region independently; NEITHER SIDE KNOWS THE OTHER'S PROGRESS. Because the
       in-flight ELEMENT state lives here, no vector-LSU module needs state scoped
       to "the current instruction" and none needs to export a `busy` that gates
       issue. This module exports no `busy` either — see the ports section, which
       says so explicitly and says what it exports instead.
       Ground rule 6 has been AMENDED (decision D5) and now enumerates THREE
       legal homes for in-flight vector-LSU state, not two: (a) these six
       instances, (b) the LCB's per-PRN assembly entries, and (c) VecLsu's
       per-LDQ/STQ-entry descriptor pending table, which absorbs the mid-walk
       hazard and INT-RF read denial. (c) is the same KIND of state as (a) —
       per-queue-entry, capacity guaranteed at dispatch, structurally
       un-overflowable, exporting no `busy`. So "all element state is here" is
       the right reading and "no other module holds any state" is not. What is
       unchanged, and is the part the rule actually protects, is that NONE of the
       three exports a `busy` toward an issue unit.

  ===> IT IS NOT A FIFO, and reading it as one is the mistake that produces the
       wrong RTL. It is a RESERVED-REGION RANDOM-ACCESS BUFFER: capacity is
       claimed in PROGRAM ORDER at dispatch, so the occupied region is
       program-ordered and a squash is a tail-pointer rollback with no per-entry
       compare and no `br_mask`; but within its own region an op is addressed by
       absolute entry index, because age-ordered-ready issue lets a younger op
       fill and drain its region before an older one.

  Governing spec anchors: loadstore.rst `lsu-unified`, `ssi-queues`, `us-queue`,
  `store-data-queue`, `vec-squash`, `vec-store-algo`, `vector-bw-ceiling`;
  issue.rst `vec-queue-reservation`.

<|begin_module|>

  <|begin_parameters|>
  Six constructor parameters. Two have no default on purpose: a wrong default
  for either is a silent correctness bug rather than a mis-tuning, so an
  instantiation that omits them must fail to elaborate.

  `entries` (Int, required) — depth. `ssiQueueEntries` (default 512) for the
  three SSI instances, `usQueueEntries` (default 16) for the three US instances.
  Must be a power of two: the circular pointer arithmetic and the range masks in
  the logic section wrap by truncation, and a non-power-of-two depth would put a
  modulo in the dispatch timing path. Depth is an ARCHITECTURAL LIMIT and not a
  tuning knob — it bounds in-flight vector memory ops, because capacity is
  reserved at dispatch. The bound differs by direction, because the reservation
  rule does (decision D9/D10): a STORE reserves its worst-case element count, so
  512/256 = 2 worst-case stores in flight; a LOAD reserves
  `min(worstCase, ldResvMembers * vLen/eew)` and streams the rest, so at
  `ldResvMembers = 4` it is 512/(4*32) = 4 loads in flight at EMUL=8, SEW=8.
  VectorParams owns `ldResvMembers`, the depths, and the `entries >= vLen` liveness
  floor for the element-granular instances; do not restate those checks, and do not
  re-derive the load cap here — this module never sees a reservation's arithmetic,
  only the region it produced.

  `width` (Int, required) — payload width in bits. The queue NEVER INTERPRETS
  the payload: it stores and returns `width` opaque bits. That is what lets one
  definition serve six instances with four different entry shapes. Per instance:
  `addrWidth` = `(new VecElemAccess).getWidth` for the SSI address queues,
  `rangeWidth` = `(new VecRangeEntry).getWidth` for the US address queues, `eLen`
  (64) for `st_SSI_DATA_Q`, `vLen` (256) for `st_US_DATA_Q` — every one derived
  at elaboration from VecBundles or VectorParams, no literal widths.

  `isStore` (Boolean, required, NO default) — selects the freeing discipline: a
  store instance retains every filled entry until its region is released at
  commit-drain, a load instance lets the drain side free an entry for refill.
  Defaulting this would give a store queue load semantics, i.e. store state
  discarded before commit.

  `hasXlatePass` (Boolean, default false) — true only for `st_SSI_ADDR_Q` and
  `st_US_ADDR_Q`. Adds the in-place payload update port and the second
  (translation) cursor described in the logic section. Requires `isStore`.

  `reserved` (Boolean, default true) — capacity is claimed at dispatch rather
  than opportunistically at execute. All six instances set it true. It is a
  parameter and not a hard-coded `true` so that the property is visible at the
  instantiation site; elaboration requires `reserved || !isStore`, because an
  unreserved store queue is exactly the configuration whose partial-fill
  deadlock is documented in the danger note of loadstore.rst `ssi-queues`.

  `ports` (Int, default 1) — fill/consume lanes, one per D$ request lane: 1 on
  Medium, 2 on Large/Mega where the queues are "2 x nOP.v wide". Derived by
  VecLsu from `lsuWidth`/`dcacheArbiterMode`; the two-lanes-per-cycle
  requirement itself is VecLsu's obligation, not tagged here. `readPorts` is the
  indexed-read port count: one per drain lane, plus one shared port for the
  disambiguation and store-forwarding consumers, which read a store queue at an
  index the LCAM match produced.

  `readPorts` IS A DERIVED BODY VALUE, `ports + 1`, NOT A CONSTRUCTOR PARAMETER,
  and the reason is a Scala restriction rather than a design choice: a default
  argument may not reference an earlier parameter of the SAME parameter list, so
  `(ports: Int = 1, readPorts: Int = ports + 1)` does not compile. The two ways to
  keep it a parameter are both worse — a second parameter list forces every call
  site to write an empty `()` before the implicit list, and an `Option` override
  parameter adds a configuration knob no instantiation uses, which mode `new`
  forbids as "extra". No instantiation overrides it and the formula is exact (one
  drain lane each plus the one shared match-read port), so derive it. KEEP the
  `readPorts >= ports` require: it is trivially true today and it is what fires if
  the formula is ever changed.
  <|end_parameters|>

  <|begin_ports|>
  //@req-spec-lsu.b3
  Fill: `io.enq`, a `Vec(ports, Flipped(Decoupled(...)))` whose bits are `idx`
  (`log2Ceil(entries)`) and `data` (`width`). This is the port the STAGE 1
  vAGENs drive — VecElemAgen into the SSI address queues, VecRangeAgen into the
  US address queues, VecDgen into the store data queues. `idx` is ABSOLUTE, not
  a "next" position: the producer forms it as its reservation base plus its
  element cursor, so two ops may fill in any interleaving. `ready` is a function
  of `bits.idx` (the entry is inside the occupied region and not already
  filled), which is legal Chisel and is the streaming back-pressure for loads.

  Read: `io.rd`, a `Vec(readPorts, ...)` of `req.valid` + `req.idx` with
  `resp.data` + `resp.filled` returned ONE CYCLE LATER — uniformly, across all
  six instances and both storage styles (see the logic section), so that every
  consumer compiles against one timing contract.

  //@req-spec-lsu.k7
  Consume: `io.consume`, a `Vec(ports, Valid(idx))`. One lane per granted D$
  lane, and one entry per lane per cycle — which is what "SSI accesses drain one
  element per granted lane" means structurally: there is no wider path out of
  this module, so a pathological gather is element-serial no matter how the rest
  of the machine is sized. VecDcacheArbiter owns the grant; this module only
  bounds the width. For a load instance a consume clears the entry's filled bit;
  for a store instance it does not (see the logic section).

  Update: `io.update`, a `Vec(ports, Valid(idx, data))`, present only when
  `hasXlatePass`. Replaces a filled entry's payload in place with the TRANSLATED
  address and marks it translated.

  Reservation (present when `reserved`), driven by VecQueueReservation:
  `io.resv.avail` (registered free-entry count), `io.resv.tail` (current
  reservation tail index), `io.resv.claim` (`valid` + `entries`, the summed
  program-ordered claim of one dispatch group, all-or-nothing) and
  `io.resv.release_tail` (`valid` + `tail`, the tail-only surplus release at
  execute once VL is known). `io.resv.free` (`valid` + `base` + `entries`)
  retires the OLDEST region and advances the head. The per-slot prefix sum that
  turns a dispatch group into per-op base indices belongs to
  VecQueueReservation, which owns those requirements; this module exposes the
  tail and the availability it needs to compute them.

  ===> `io.resv.free` IS AN INPUT AND THIS MODULE PRODUCES NO RECLAMATION REPORT OF
  ITS OWN. That is worth stating because the seam was once wired the other way
  round in intent: the reservation was written expecting the queues to REPORT
  head-side reclamation, and nothing here ever did — the six queues would have
  filled exactly once and never freed. The settled direction is the one above.
  VecLsu presents the retiring LDQ/STQ index to VecQueueReservation, which looks up
  that op's region and echoes its base and count; VecLsu applies the echo to this
  port while the reservation decrements its own occupancy in the SAME cycle. So
  `io.resv.free` always names a region that starts at `head`, and the assertion
  below to that effect is checking a property of the whole reclamation path, not a
  local arithmetic slip.

  Squash: `io.squash` (`valid` + `tail`), driven by VecSquashUnit — the absolute
  index to roll the reservation tail back to. It is ONE PAST the youngest surviving
  region (`base + count` of that region), computed by VecQueueReservation from
  BOOM's EXCLUSIVE LSQ tail; this module applies it verbatim and interprets nothing.

  Status: `io.empty`, asserted when the occupied region is empty. VecLsu ANDs all
  six instances (and the LCB) into `vec_lsu_empty`, which folds into
  `io.core.fencei_rdy`.

  `io.filled_vec`, `UInt(entries.W)`: a READ-ONLY VIEW of the per-entry `filled`
  register this module already keeps, in PHYSICAL index order (index it with
  `phys()`, i.e. the low `idxW` bits of a pointer). It declares no state, adds no
  read port and costs no cycle. It exists for one caller and one reason: the
  snoop's store-presentation data gate (spec-memord.a22) must be answered
  COMBINATIONALLY, in the cycle the candidate is built, and the `rd` port cannot
  do it — that port's `resp.filled` is registered and arrives one cycle late,
  which is already after the presentation it was meant to gate. See VecLsu
  section (h3) for what goes wrong when the caller substitutes a constant.

  ===> AND `filled_vec` IS NOT AN EXCEPTION TO THE STATUS RULE BELOW, because it
       is per-ENTRY and carries no instruction identity: it says which slots hold
       bytes, never which op owns them or whether that op is done. A reduction of
       it routed to an issue unit would be exactly the `busy` the rule forbids.

  Clock and reset: Chisel's implicit `clock` (posedge) and `reset` (ACTIVE-HIGH,
  SYNCHRONOUS). No second clock or reset domain appears here.

  ===> AND THAT IS THE WHOLE STATUS EXPORT. There is deliberately NO `busy`, no
  `active`, no "current instruction" identifier and no per-instruction done bit
  on this interface. `io.empty` is safe precisely because its consumer waits at
  DISPATCH with the ROB already empty; a signal of the same shape routed to an
  issue unit would be the failed review the plan's ground rules describe.
  <|end_ports|>

  <|begin_logic|>
  //@req-spec-lsu.a5
  ---- What the six instances are for ----

  Vector memory `OP.v`s hold ONE placeholder LDQ or STQ entry each, for ordering
  and commit. The effective `nOP.v` element operations they crack into are
  buffered HERE, in address and data queues separate from the LDQ/STQ, so that a
  single wide vector access cannot pollute the scalar load/store queues and
  throttle scalar memory performance. Address and data are separate queues
  because they are produced by different stages (AGEN vs DGEN) and consumed at
  different times (translate vs post-commit write).

  //@req-spec-lsu.b1
  //@req-spec-lsu.c2
  The address instances split by ADDRESSING CLASS, not by direction. The SSI
  pair (`ld_SSI_ADDR_Q`, `st_SSI_ADDR_Q`) holds the effective addresses computed
  for strided, indexed and segmented accesses — one entry per active element,
  which is why they are 512 entries deep. The US ADDRESS pair (`ld_US_ADDR_Q`,
  `st_US_ADDR_Q`) holds unit-stride transactions as ONE `VecRangeEntry` per
  instruction describing the whole contiguous range, which is why 16 entries
  suffice and why the element-count depth floor does not apply to them. (The third
  US instance, `st_US_DATA_Q`, is NOT one entry per instruction — see the store-data
  paragraph below, which is the seam most often read wrong.)

  //@req-spec-lsu.d1
  //@req-spec-lsu.d2
  //@req-spec-lsu.d3
  //@req-spec-lsu.d4
  Vector store data is NOT held in the STQ entry — a `vLen`-wide payload per STQ
  slot is not a feasible structure — so `st_SSI_DATA_Q` and `st_US_DATA_Q` buffer
  it instead. Their entry widths differ because their producers differ: an SSI
  store presents data element by element, so an entry is ONE element (`eLen`, 64
  bits); a unit-stride store reads its entire source `vPRN` in a single VRF
  access, so an entry is a FULL `vLen` (256 bits) and VecBeatExpander slices out
  per-element bytes at drain. Take both widths from `eLen` and `vLen`, never
  from the literals.

  ===> `st_US_DATA_Q` HOLDS ONE ENTRY PER GROUP MEMBER, NOT ONE PER INSTRUCTION,
       and it is the one place where a US instance's entry count is not 1. Its
       partner `st_US_ADDR_Q` holds a single `VecRangeEntry` for the whole access,
       but the data side needs `v_emul * nf` whole-`vPRN` entries because that is
       how many source registers there are — so at `usQueueEntries = 16` it holds
       two worst-case (EMUL*NF = 8) unit-stride stores.
       THE ADDRESS AND DATA REGIONS OF A US STORE ARE THEREFORE NOT IN IDENTITY
       CORRESPONDENCE: different counts, different bases, different pointers, and
       the reservation carries a base and a count PER QUEUE SLOT for exactly this
       reason. This module is oblivious to that — it is handed a region and stores
       opaque payloads at absolute indices — but a generator that "simplifies" by
       assuming a US store's two regions match will place its data writes in another
       op's region, and that assumption is easiest to make right here, where the two
       queues are described together.

  //@req-spec-lsu.j10
  A drain consumer therefore reads store data from `st_US_DATA_Q` for a
  unit-stride store — whole-`vPRN`, sliced by the Packer — and from
  `st_SSI_DATA_Q` per element otherwise. The two are never mixed for one
  instruction: the addressing class selects address queue AND data queue
  together, so which queue is read is a routing decision, never a mode of this
  module.

  ---- State ----

  Payload storage is one array of `entries` x `width` bits. Elaborate it as a
  `SyncReadMem` when `entries` is large (the 512-entry SSI instances, where a
  flop array plus a 512:1 mux is the wrong structure) and as a `Reg(Vec(...))`
  for the small US instances. Both are wrapped so that `io.rd` has the same
  one-cycle latency either way.

  Two per-entry bit-vectors, held as `Reg(UInt(entries.W))` rather than
  `Reg(Vec(entries, Bool))` so a whole index range can be cleared in one cycle
  with a mask: `filled` (payload written by the fill side) and, when
  `hasXlatePass`, `xlated` (payload replaced by its translated address).

  Two pointers, both `log2Ceil(entries)+1` bits wide with the usual extra bit to
  distinguish full from empty: `head` (oldest live region's base) and `tail`
  (reservation tail). The occupied region is `[head, tail)` circularly, and
  `io.empty` is `head === tail`. `io.resv.avail` is registered — dispatch reads
  it, so it must not be a fresh subtraction on the dispatch critical path.

  There is NO per-entry `br_mask`, no owner `rob_idx`, no per-entry age and no
  per-instruction record anywhere in this module — the region discipline below
  removes the need for all four, and per-entry `br_mask` on ~1000 entries is the
  alternative the design explicitly rejected on area.

  //@req-spec-lsu.i3
  ---- Program order, and why the occupied region is contiguous ----

  Capacity is claimed only through `io.resv.claim`, only at dispatch, and
  dispatch is in program order — so `tail` advances in program order and each
  queue's occupied region is PROGRAM-ORDERED by construction. Two properties
  fall out of that single fact, and they are the reason the discipline exists:

  1. A mispredict is a POINTER ROLLBACK. On `io.squash.valid`, set `tail` to
     `io.squash.tail` — ONE PAST the youngest surviving region, i.e. that region's
     `base + count` — and clear `filled` (and `xlated`) over the abandoned range
     with one range mask. Killed entries vanish with no per-entry compare, no age
     walk and no multi-cycle drain. This is why LOADS reserve in program order too:
     they have no deadlock exposure, but without an ordered region their entries
     could not be killed this way.
     THE CONVENTION UPSTREAM IS BOOM'S EXCLUSIVE TAIL, and it matters here only
     as a statement of what this module may NOT do: `io.squash.tail` is already
     the exclusive boundary of the surviving range, so apply it as `tail :=
     io.squash.tail` and do NOT add the killing instruction's region back, do
     NOT round up to a region boundary, and do NOT treat it as "the last
     surviving entry" and add one. VecQueueReservation derived it from the LSQ
     index of the FIRST DEAD entry by taking `base + count` of the youngest row
     STRICTLY OLDER than that index (or the queue head if none), and it asserts
     that no surviving region extends past it. An off-by-one-region here keeps
     one killed instruction's entries live and drains them against PRNs already
     returned to the free list.
  2. The oldest region is always at `head`, so releasing capacity is a single
     pointer move.

  A mid-region release is NOT permitted and elaboration cannot catch it:
  `io.resv.release_tail` may only move `tail` backwards, and `io.resv.free`
  may only free at `head`. Either would punch a hole in the occupied region
  and break property 1 for every op behind it. Assert both.
  When `io.squash.valid` and `io.resv.release_tail.valid` collide, the SQUASH
  wins — the release describes a surplus belonging to an op that may itself have
  just been killed.

  //@req-spec-lsu.b6
  //@req-spec-lsu.b7
  //@req-spec-lsu.b8
  ---- Stores retain; the retention is structural, not a policy ----

  A store cannot write memory before it commits, so `isStore` entries are frozen
  once filled: a consume on a store instance does NOT clear `filled`, and the
  only thing that frees a store's entries is `io.resv.free` for its whole region
  at commit-drain. A store therefore holds its FULL ACTIVE ADDRESS SET (and, in
  `st_SSI_DATA_Q`, its full active element data) for the entire pre-commit
  window. At execute the store walks its region through `io.rd`, translates, and
  writes the PHYSICAL address back through `io.update` into the same entry — so
  the translated addresses are retained from execute until commit-drain and are
  never streamed or freed mid-instruction. Assert that a fill never targets an
  already-`filled` entry on a store instance: such a fill is precisely the
  mid-instruction reuse this paragraph forbids, and it would overwrite an
  address whose fault check has already been reported.

  //@req-spec-lsu.b9
  //@req-spec-lsu.d8
  //@req-spec-lsu.d11
  Retention is what makes precise exceptions reachable: every active element's
  address must be translated, and its fault detected, BEFORE the store commits.
  The queue's contribution is the fence between its two store cursors — the
  commit-drain read may not pass the translation cursor, so no element is
  written to memory before its own translation happened. Raising the exception
  and holding commit are the ROB's and VecLsu's obligations, not this module's.
  Store data is subject to the same window from the other end: it is read from
  the source `vPRN` at DGEN (execute time) and held here until the post-commit
  drain, which is exactly why the source `vPRN` needs no pin and frees with the
  rest of the stale group at commit. `st_SSI_DATA_Q` consequently pays the same
  cost as the SSI address queue — a full store's active element data resident
  pre-commit — and reserves in both queues or the store does not dispatch.

  //@req-spec-lsu.b10
  ---- Loads may stream; that is a different entry lifetime ----

  A load completes out of the Load Coalescing Buffer and does not gate on
  commit, so a load's entries have no reason to outlive their access. On a load
  instance (`!isStore`) `io.consume` CLEARS `filled(idx)`, which re-opens that
  index to `io.enq` — so a load region may be drained in waves and a load whose
  active element count exceeds its reservation is streamed through the region it
  already owns. Streaming reuses the op's OWN indices; it never extends past
  `tail`, because a younger reservation may already own the entries there and
  extending would break the ordered-region property above.

  ===> THIS PATH IS REACHED IN NORMAL OPERATION, NOT ONLY IN A CORNER CASE, AND
       DECISION D9/D10 IS WHAT MAKES IT SO. A load reserves
       `min(worstCase, ldResvMembers * vLen/eew)` entries rather than the full
       worst case, so ANY load whose active element count exceeds four members'
       worth streams: `ld_SSI_ADDR_Q` is the instance that does it, and this
       paragraph is its specification. Under the earlier worst-case-for-everything
       rule the precondition was unreachable — `worstCase = EMUL * vLen/eew` is
       already `VLMAX >= VL >= active count` — so streaming would have been dead
       code in the most-instantiated leaf of the subtree. Generate it as a live
       path and exercise it: a wave boundary is where a fill/consume race on one
       index would first show up.

  ===> AND WITHIN-REGION CIRCULAR REUSE IS WHAT MAKES THE UNDER-RESERVATION
       DEADLOCK-FREE. It is the reason the rule "never extend past `tail`" above is
       a correctness rule and not a tidiness rule. An OLDER load's region sits
       AHEAD of every younger one's in the circular order, so it drains first and
       refills into its OWN indices; no younger reservation can ever occupy an entry
       an older load is waiting for, and there is no path by which an older load
       waits on a younger one. Let a streaming load extend past its region — the
       obvious "optimization" once entries are seen to be free — and that property
       is gone: the older load would then be waiting for a younger op's entries
       while the younger op waits behind it at the head for reclamation, which is
       the store deadlock reintroduced on the load side, where no reservation rule
       is guarding against it.
       Squashability is untouched by the smaller reservation: the region is still
       contiguous and still program-ordered, merely shorter, so the pointer
       rollback above is unchanged.

  The fill side does not poll and does not track drain progress: it simply
  sees `enq.ready` deassert on an index still holding an undrained access,
  and resumes when it clears. That absence of a progress channel between the
  two sides IS the decoupling — a single element cursor shared between fill
  and drain is what capped the previous attempt's concurrency at one
  instruction.

  ---- Per-cycle update ----

  Each cycle, in one place, apply: `filled := (filled & ~clear_mask) |
  set_mask`, where `set_mask` is the OR of the accepted `io.enq` indices and
  `clear_mask` is the OR of the load consumes, the freed region's range mask and
  the squashed range mask. Update `xlated` the same way from `io.update`. Move
  `head` on `io.resv.free`, and `tail` on `io.resv.claim` /
  `io.resv.release_tail` / `io.squash` with the priority stated above. Nothing
  else writes state.

  Elaboration checks: `isPow2(entries)`; `width > 0`; `ports >= 1` and
  `readPorts >= ports`; `!hasXlatePass || isStore`; `reserved || !isStore`. Each
  names the offending parameter, because every one of them is a configuration
  error that would otherwise appear as a deadlock or as data written before
  commit rather than as a build failure.

  Runtime assertions (BOOM already uses `assert` widely; they carry no synthesis
  cost): a claim larger than `avail`; a fill whose `idx` is outside `[head,
  tail)`; a fill onto a filled entry when `isStore`; a `free` whose base is not
  `head`; a `release_tail` or `squash` that would move `tail` behind `head`.

  ===> THE "commit-drain read of an entry whose `xlated` bit is clear" ASSERTION IS
  NOT THIS MODULE'S, corrected at E2. `io.rd` is one anonymous indexed-read array
  and nothing on it distinguishes an EXECUTE-time translate read, which is supposed
  to see `xlated = 0`, from a post-commit DRAIN read, which must see `xlated = 1`.
  Asserted here it would false-fire on every legitimate translate read. Do not
  "fix" this by adding an `is_drain` flag to the read port: that pushes a caller's
  pipeline phase into a shared port, and every future reader of the port has to get
  it right. The assertion belongs to the DRAIN CONSUMER, which knows its own phase
  by construction — it is `VecLsu`'s obligation, checked where the drain read is
  issued, and it is listed there.

  ORDERING WHEN `claim` COINCIDES WITH `release_tail`: `squash` overrides both. With
  `release_tail` and `claim` together, REBASE THEN ADD — the tail becomes the
  released tail plus the new claim. They are different instructions in different
  pipeline stages (one retiring its surplus once VL is known, one reserving at
  dispatch) and both must take effect, so neither may be dropped and the order is
  the only composition that conserves both.

  Tracing: emit one guarded VecTrace line per key event — claim, fill, consume,
  update, free, squash-rollback — each tagged with the instance name, the entry
  index and the resulting occupancy, gated on the `vecTrace` plusarg and off by
  default. With no unit tests in this project these six lines are the only way to
  see a queue wedge in a cosim run, and the occupancy field is what separates a
  real stall from a reservation that was never released.

  THE INSTANCE NAME IS A CONSTRUCTOR PARAMETER, `queueName: String`, and it is not
  optional decoration. One definition is instantiated SIX times; a trace corpus in
  which every line says "VecElemQueue" cannot answer the first question anyone asks
  of a wedged LSU — which queue. Chisel's own `instanceName` is not safely readable
  from inside a module's own constructor, so there is no way to recover this after
  the fact and it has to be passed in. VecLsu passes the normative queue name
  (`ld_SSI_ADDR_Q` and the other five, spelled exactly as the six-queue enumeration
  spells them). It is `queueName` and NOT `name` because `name` is an inherited
  member of Chisel's `BaseModule`; shadowing it with a constructor `val` would
  either fail to compile or quietly displace the naming Chisel uses for the emitted
  module, which is a high price for a shorter identifier.

  The elaboration-`require` and runtime-`assert` messages carry `queueName` too, for
  the same reason: six instances share every check, so "claim exceeds avail" without
  a queue name states the symptom and withholds the only fact needed to act on it.

  ===> `rob_idx` IS DELIBERATELY *NOT* ON THESE LINES, corrected at E2 after the
  first generation attempt found the contradiction: this paragraph asked for the
  requester's `rob_idx` on every line, and NO port of this module carries one — the
  ports section's bit-lists have none, and section "There is NO per-entry `br_mask`,
  no owner `rob_idx`..." explicitly forbids storing one per entry. The two
  statements cannot both hold.
  The one that gives way is this one, because the alternative is worse: a TRACE-ONLY
  `rob_idx` on `enq`/`consume` would be a per-ELEMENT field carrying
  per-INSTRUCTION information, i.e. exactly the owner field the entry format forbids,
  reintroduced through the trace port where no assertion polices it. Instead, trace
  the ENTRY INDEX here and let `VecQueueReservation` — which holds `rob_idx` against
  the region it granted — trace the `rob_idx`↔region binding at claim and free. The
  join is then offline and exact: an index in a traced region belongs to that region's
  `rob_idx`, and no new port exists to go stale.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput: `ports` fills, `ports` consumes, `ports` updates and `readPorts`
indexed reads accepted per cycle, with no cycle in which a fill and a consume to
different indices conflict — the two sides must be able to run at full rate
simultaneously, since that concurrency is the module's entire purpose.

Latency: `io.rd` is one cycle, uniformly. `io.enq.ready` is the only
combinational output, and it is deliberately shallow: one bit selected out of
`filled` plus a range compare. `io.resv.avail` is REGISTERED, not combinational,
because it is read at dispatch and must not put a fresh subtraction on that path.
It is nonetheless never stale in a way that lets a claim over-commit: it is
registered from `next_tail`/`next_head`, so a claim granted in one cycle is
already reflected in the value the next cycle's claim reads.

A squash must complete in ONE cycle. Rolling back by walking entries would stall
the pipeline behind a mispredict for hundreds of cycles on the 512-entry
instances, and the walk would race the fill side.

Area is dominated by the three SSI instances (512 entries each), which makes two
things constraints rather than commentary: the payload must stay narrow, so
nothing instruction-scoped may be widened into the entry; and at that depth the
storage must elaborate to a memory rather than a flop array plus a wide mux.
<|end_perf|>

<|begin_dependencies|>
VecBundles — `VecElemAccess` and `VecRangeEntry` give `addrWidth` and
`rangeWidth`, and `VecReservation` is the shape of the dispatch-time claim. This
module stores those payloads OPAQUELY and must not re-declare or slice any of
their fields.

VectorParams — `ssiQueueEntries`, `usQueueEntries`, `eLen`, `vLen`, the
`entries >= vLen` liveness floor which VectorParams checks, not this module, and
`ldResvMembers`, which this module never reads but which is what determines how
often the load streaming path above is entered.

VecTrace — the guarded trace lines above.

Instantiates nothing. Instantiated six times by VecLsu. Its counterparties are
VecElemAgen and VecRangeAgen plus VecDgen on the fill port, VecBeatExpander on
the read/consume ports, VecQueueReservation on the reservation port,
VecSquashUnit on the squash port, VecCrossLsuSnoop and VecStoreForward on the
shared read port, and VecLsu on `io.empty` for `vec_lsu_empty`. Every one of
those seams is an index-plus-payload contract with no progress channel back the
other way, which is the property Phase R should check from both sides.

Two of those seams are DIRECTIONAL in a way worth restating, because both were
mis-read once: `io.resv.free` is an INPUT, manufactured by VecQueueReservation from
the retiring LDQ/STQ index VecLsu presents to it and applied here by VecLsu — this
module reports no reclamation and never did, which is why the reservation's original
expectation of a report from the queues had no producer at all. And `io.squash.tail`
is an already-EXCLUSIVE boundary computed by VecQueueReservation from BOOM's
exclusive LSQ tail — applied verbatim, never adjusted here.
<|end_dependencies|>

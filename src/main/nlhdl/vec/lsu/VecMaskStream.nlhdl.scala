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
  VecMaskStream — reads `v0` once per OP.v and streams its mask bits to that
  direction's address generators at ELEMENT granularity, with the one-entry
  lookahead the element agen's start rule depends on.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecMaskStream.scala,
  package boom.v4.vec.generated.lsu, group vec_lsu.
  depends_on VecBundles, VectorParams, VecTrace.

  ===> INSTANTIATED IN `VecLsu`, ONE PER DIRECTION, AS `ld_msk` AND `st_msk`. It
       is NOT instantiated inside `VecElemAgen`; earlier revisions of this file
       said `msk` inside the element agen and that is corrected here. The hoist to
       VecLsu is not cosmetic and it is not reversible without breaking two
       requirements at once:
         - `spec-agen.e12` requires the mask to be read in stage 1 for EVERY
           access class INCLUDING UNIT-STRIDE, and `VecRangeAgen` — which owns the
           unit-stride class — has no mask reader of its own and adds no VRF port.
           With the streamer inside the element agen, a unit-stride `OP.v` had no
           mask reader at all.
         - `spec-vrf.g18` requires `R1` and `R4` to have EXACTLY ONE reader each,
           so duplicating the streamer into the range agen was never an option
           either.
       One instance per direction at VecLsu level satisfies both: it feeds BOTH
       that direction's element agen (the streamed `staged`/`ahead` cursor, with
       `step`/`skip` back) AND that direction's range agen (the latched `us_mask`,
       carried on the range entry to the drain side). VecLsu owns the start
       interface and the resolved `kill`; this module's ports are unchanged by the
       hoist, only its parent is.
       Two instances, distinguished only by `isStore`, for the same reason the
       agens come in pairs: the load-priority mux over a single shared unit is
       what silently dropped store grants in `addvector`.

  ===> ONE OP.v PER DIRECTION AT A TIME, AND VecLsu IS WHERE THAT SERIALIZES.
       The cursor is single, so a direction's element walk and its range entry
       cannot occupy this module at once, and a direction's `OP.v` hand-offs
       therefore serialize at VecLsu level. That is absorbed by VecLsu's
       per-LDQ/STQ-entry descriptor pending table, which presents a descriptor
       only while this module is free — NOT by any `busy` or `ready` exported from
       here toward an issue unit. For a unit-stride `OP.v` there is nothing to
       walk: the latch loads and the cursor retires the next cycle, so occupancy
       is one cycle.

  ===> THE PORT NUMBERS ARE CANONICAL AND THIS MODULE ADDS NONE.
       midcore.rst `vrf-ports` is the single source of truth: the mask read is
       `R1` on the load path and `R4` on the store path. `R1`'s one reader is
       `ld_msk`. `R4` serves the store mask AND the store index, and it reaches
       `VecRegFile` as EXACTLY ONE request because `VecLsu` owns a 2:1 mux over it
       — mask wins, index waits, with this module's `owns_port` as the hold-off.
       So the file still sees one reader per port, which is what `spec-vrf.g18`
       requires: VRF ports are statically partitioned and never arbitrated inside
       `VecRegFile`, so a second requester arriving there is not a performance
       problem, it is an unimplementable one.

  ===> THE CURSOR THIS MODULE DERIVES IS THE SINGLE OWNER OF "WHICH ELEMENTS ARE
       ACTIVE". The store DGEN and the Load Coalescing Buffer consume this same
       cursor and must NOT evaluate the mask a second time. Two evaluations is
       precisely how the address and data streams drift apart: they agree until
       the first place their tail/vl handling differs, and then a store pairs
       element k's address with element k+1's data.

  Governing spec anchors: execution.rst `vector-agen` (the mask rule, the
  Skipper's priority encoder, the once-per-OP.v latch, the stage-1/stage-2
  split), midcore.rst `vrf-ports` (the canonical port table and the
  one-reader-per-port statement).
*/

<|begin_module|>

  <|begin_parameters|>
  `isStore` — Boolean, no default (VecLsu, the instantiator, must state a
  direction at both sites). False selects the load path and VRF read port `R1`;
  true selects the store path and `R4`. It also selects whether the read port is
  SHARED with the index read (store only) — see the logic section.

  `maxSkipLog2` — Int, default 3, legal range 0 to log2Ceil(vLen). The largest
  power-of-two element skip the priority encoder may emit, i.e. the encoder
  examines a window of `1 << maxSkipLog2` mask bits ahead of the cursor rather
  than all `vLen` of them. It is a knob because it is the one real timing/area
  trade here: the window IS the encoder's fan-in. Masking is an optimization and
  not a selection criterion, so `maxSkipLog2 = 0` — walk every element, skip
  nothing — is a legal and functionally correct bring-up fallback.

  Everything else comes from VectorParams through `HasVectorParams` — `vLen`,
  `vecPregSz`, `vecVLSz` — and from `HasBoomCoreParameters` for `robAddrSz`. No
  width in this file is a literal.

  //@req-spec-agen.e14
  There is no parameter for the mask width and must never be one: the latch is
  exactly `vLen` bits, which is not a chosen size but a proof. The worst-case
  element count of any encodable RVV access is
  VLMAX = LMUL_max * vLen / SEW_min = 8 * vLen / 8 = `vLen`, and a mask is one
  bit per element held in ONE register of `vLen` bits. So a single `vLen`-wide
  read of `v0` covers every element at every SEW and LMUL. That is what makes
  "read once per OP.v and latch" a complete solution rather than an optimization:
  no second read can ever be required, so there is no mid-stream refill path to
  get wrong.

  The whole module is elaborated only when `usingRVV` is set (a Scala Boolean
  from `BoomCoreParams`, never a hardware `Bool`, and never rocket's
  `usingVector`). In a vectors-off build it is ABSENT, not tied off.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the hierarchy default and Chisel's implicit
  convention: single `core_clk` domain, posedge `clock`, ACTIVE-HIGH SYNCHRONOUS
  `reset`. No second domain, no asynchronous reset, no gated clock.

  ---- from VecLsu: the OP.v being cracked ----

  `op` — Valid(MicroOp). Asserted for one cycle when the owning OP.v is granted
  into stage 1 (`ld_vAGEN_1` on the load path, `st_vagen_1` on the store path).
  VecLsu drives it from that direction's `VecScalarOperandRead` output, for an
  `OP.v` of EITHER class — this is a per-direction port, not a per-agen one.
  Fields read: `pvm` (the renamed mask PRN — one register, never a group),
  `rob_idx`, `v_eew`, `v_emul`, `v_seg_nf`.

  `op_masked` — Bool, valid with `op`. True when the instruction actually has a
  mask operand (`vm = 0` in the encoding). VecLsu supplies it already resolved
  from `v_is_masked`; this module does not decode instruction bits.

  `op_vl` — UInt(vecVLSz.W), valid with `op`. This OP.v's VL, read from the VL
  register file through `pvl` at execute. VL is renamed and is not a uop field,
  so it must arrive as a port.

  `kill` — Bool. An ALREADY-RESOLVED kill (branch mispredict via `brupdate`, or
  `rob_flush`), and after the hoist its resolver is named: `VecSquashUnit`, whose
  `kill` client 0 is `ld_msk` and client 1 is `st_msk` in VecLsu's fixed ordering.
  Resolved outside so this module carries no `br_mask` comparison of its own and
  cannot disagree with the agens about whether the same OP.v is dead. (The four
  agens take `brupdate`/`rob_flush` directly instead, because an element agen holds
  two uops with two different `br_mask`s and one pre-resolved Bool could only ever
  be right for one of them. This module holds one OP.v, so a resolved Bool is
  exactly right for it.)

  `step` — Bool, the element agen's accept of the staged element: advance by one.
  `skip` — Bool, the element agen's accept of a whole disabled run: advance by
  `skip_log2` instead. VecLsu routes both back from that direction's
  `VecElemAgen`. The amount is an output and never an input, so the agen cannot
  advance the cursor by a distance this module did not sanction.

  ---- the VRF mask read port (R1 load / R4 store) ----

  `mask_rd_req` — output, `valid` plus `addr` of `vecPregSz` bits. Bound by VecLsu
  to VRF read port `R1` when `isStore` is false and `R4` when true. The port number
  is a static elaboration-time choice, not a runtime selection. There is no `ready`
  on it and there must not be: `VecRegFile` provides none, and on the store path
  this module WINS `R4` unconditionally (see rule 1).

  `mask_rd_data` — input, `vLen` bits. THE READ IS A REGISTERED ONE-CYCLE PORT:
  the result arrives the cycle after `mask_rd_req.valid`, with the output flop
  sitting in `VecRegFile`, one per read port. `VecRegFileBank`'s "0-cycle,
  unpipelined" array read is BANK-INTERNAL and lives inside that one cycle — it
  must not be read as a 0-cycle port here, and this module must not add a second
  register on the path either, or the observable latency becomes 2 and every
  element pipeline above is off by a cycle. The VRF provides read-during-write
  forwarding so a same-cycle write to `pvm` is seen; nothing here re-implements
  that.

  `owns_port` — output Bool, high exactly in the cycle `mask_rd_req.valid` is
  high. It is the HOLD-OFF for the 2:1 `R4` mux that VecLsu owns: on the store
  path VecLsu grants `R4` to this module whenever `owns_port` is high and to
  `VecIdxGen`'s request otherwise. It is a PORT-CYCLE indication and nothing else:
  it must not be routed to `IQ_V_LOAD`/`IQ_V_STORE`, to VecQueueReservation, or to
  any dispatch gate.

  ---- the streamed cursor: what the agen consumes ----

  `staged` — output bundle {`valid`, `elem`, `active`, `last`}. `elem` is the
  ELEMENT index, `log2Ceil(vLen + 1)` bits wide; `active` is this element's
  effective mask bit; `last` marks `elem == op_vl - 1`.

  `ahead` — output bundle of the SAME shape, one element further on.
  `ahead.valid` low is the "next mask bit is not yet staged" condition the agen's
  start rule tests.

  `skip_log2` — output UInt(log2Ceil(maxSkipLog2 + 1).W) with `skip_valid`: the
  power-of-two element distance the agen may jump when the staged element is
  inactive.

  `all_inactive` — output Bool, valid once the latch is loaded: this OP.v has
  zero active elements (VL = 0, or every in-VL mask bit clear). Exported rather
  than left to VecGroupCopy to re-derive, for the single-owner reason above.

  `done` — output Bool, one cycle, when the cursor passes the last element,
  retiring the cursor. NOT a completion signal; it does not reach the ROB.

  `us_mask` — output bundle {`valid`, `bits` (`vLen`), `rob_idx`}: the latched,
  tail-cleared mask vector, published for the unit-stride path (logic rule 5).

  There is deliberately NO `busy` output, no `occupied` output and no `ready`
  output that any issue unit sees. The only back-pressure this module presents is
  `ahead.valid` / `staged.valid` to that direction's element agen. VecLsu's
  "streamer is free" term is DERIVED from the `op` it drives and the `done` it
  receives — a single tracking bit per direction at that level, no port here —
  which is what keeps the hand-off gate out of this module's interface.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. The read: once per OP.v, on the direction's canonical port ----

  //@req-spec-agen.c28
  //@req-spec-agen.e4
  //@req-spec-vrf.g3
  On the load path (`isStore = false`) the read is issued on VRF read port `R1`
  at `op.valid`, addressed by `op.bits.pvm`. `ld_vAGEN_1` is therefore the single
  reader of `R1` — the `ld_msk` instance IS that reader, for every access class of
  the load direction, which is exactly what the hoist bought. The
  `vrf-ports` row that names "Load Unit / LCB" as one functional unit does not
  make the LCB a second reader of `R1`: the LCB never reads the mask at all, it
  consumes the cursor published below. Its own VRF traffic is `R2`
  (`stale_pvdest`) and `W0`/`W1`.

  //@req-spec-agen.e5
  //@req-spec-vrf.g8
  On the store path (`isStore = true`) the same read is issued on `R4`, which
  also carries the store INDEX reads that VecIdxGen needs for an indexed store —
  `idx` inside `st_elem_agen`, no longer a sibling of this module but still the
  other reader of the same port. THE 2:1 MUX IS `VecLsu`'S, not this module's and
  not the agen's, because `VecRegFile` publishes no read `ready` and `R4` must
  reach it as exactly one request. Static priority there: MASK WINS, INDEX WAITS —
  the mask read takes `R4` in the cycle it asserts `owns_port`, and `idx`'s request
  (which holds its `valid` until granted) is held off for that one cycle. This
  cannot starve the index read, and the argument is structural rather than
  statistical — the agen may not start an element access whose mask bit is not yet
  staged (rule 3), so the first index member cannot be needed before the cycle
  after the mask read completes. The mask read is strictly first by construction,
  happens once, and never contends again; index members are read as the walk
  advances.

  //@req-spec-agen.e14
  The read happens ONCE per OP.v and the result is LATCHED in `mask_q`, a
  `vLen`-bit register. It is never re-read per element and there is no
  re-read path. The latch is loaded from `mask_rd_data` the cycle after the
  request; `mask_valid` gates every output until then.

  When `op_masked` is false the VRF read is ELIDED and `mask_q` is loaded with
  all ones. That is correctness, not an optimization: an unmasked instruction has
  no mask operand, `v0` is not one of its sources, and the read would return
  whatever `v0` happens to hold rather than the architectural mask, which for
  `vm = 1` is all ones. Whole-register (`vl1re*`/`vs1r*`) and mask (`vlm`/`vsm`)
  accesses are unmasked by definition and take this path.

  ---- 2. The latch, the tail, and the cursor ----

  //@req-spec-agen.e1
  `mask_q` is post-processed once at load time and never again: bits at or above
  `op_vl` are FORCED TO ZERO. That single AND is what lets every consumer test
  one predicate, `active`, instead of pairing a mask lookup with its own `elem <
  vl` comparison — and a private `vl` comparison in a second consumer is the
  same drift bug as a second mask evaluation. Element `i` is active iff
  `mask_q(i)` after tail clearing. This is the value both AGEN stages apply
  during address generation: stage 1 applies it per element here, and stage 2
  receives it by carriage rather than by a VRF read of its own (rule 5).

  // ELEMENT granularity, not byte granularity. mask_q(i) is element i's bit at
  // EVERY eew — the mask index is NOT scaled by v_eew. This is the exact
  // inverse of the index/address side, where the per-element offset MUST be
  // shifted by eew, and getting the two the same way round is a bug that
  // corrupts only element 0 of a misaligned access and hides everywhere else.

  For a SEGMENTED access the cursor granularity is the SEGMENT: one mask bit
  governs all `v_seg_nf` fields of segment `i` — "packs whole segments per
  element". The cursor counts segments and the ELEMENT AGEN expands fields within
  one; `nf` does not divide the mask.

  `elem_ptr` starts at 0 on every OP.v. There is no `vstart` offset and no input
  for one: a faulting vector memory op traps with `vstart = 0` and restarts the
  WHOLE instruction, so mid-vector resume is not representable here and a
  `vstart` start point could only ever be wrong.

  ---- 3. The one-entry lookahead — a carried-over bring-up hazard ----

  `staged` publishes the cursor's element and `ahead` publishes the next one, one
  element READY-AHEAD, both combinationally from `mask_q` and `elem_ptr`. The
  ELEMENT AGEN's rule (the consumer of this cursor, reached through VecLsu),
  inherited verbatim from the OVI Skipper and kept because
  bring-up found it rather than because design predicted it, is: DO NOT START,
  AND DO NOT EMIT, AN ELEMENT ACCESS WHOSE MASK BIT IS NOT YET STAGED. The agen
  gates both its start and its emit on `ahead.valid`; when it is low the agen
  waits at the element boundary instead of entering wait states mid-stream with
  a half-formed packet.

  // Because the mask is read whole and latched (rule 1), `ahead.valid` can only
  // be low in the load cycle or after a kill — never mid-stream. Publish it
  // anyway, and do not "simplify" it away: it is the same shape as VecIdxGen's
  // staging signal, which genuinely can go low mid-stream, so the agen's start
  // gate stays one AND of two identically-shaped conditions. A start gate whose
  // two halves have different shapes is how the index case gets forgotten.

  ---- 4. The priority encoder over the mask bits ----

  //@req-spec-agen.c9
  When the staged element is inactive, a priority encoder over the mask bits
  finds the next ACTIVE element so the agen jumps past runs of disabled elements
  instead of walking them. Implementation: take the `1 << maxSkipLog2`-bit
  window of `mask_q` above `elem_ptr`, and use BOOM's existing `SelectFirstN` /
  `PriorityEncoder` utilities over it (reuse, per the ground rules; do not add a
  parallel find-first). If the window is entirely clear, publish
  `skip_log2 = maxSkipLog2`; otherwise publish the largest `k` such that all of
  elements `[elem_ptr, elem_ptr + (1 << k))` are inactive.

  // The jump is a POWER OF TWO and that is a datapath requirement, not a
  // rounding convenience. The agen advances a strided address by
  // `addr += stride << k`, a shifted add; an arbitrary jump distance would need
  // `addr += stride * d`, a multiplier in the AGEN's critical loop. The encoder
  // knows the exact distance and deliberately gives back less.

  The window — rather than all `vLen` bits — is also what keeps this
  synthesizable at the target frequency: the encoder's fan-in is
  `1 << maxSkipLog2` bits, a small flat tree, not a `vLen`-bit find-first inside
  the cursor's own feedback loop.

  Skipped elements produce NO output packet. The inherited Skipper emitted "fake"
  packets for skipped regions because the bobtail unit fetched masked-off lanes
  and merely flagged them; Caracal suppresses the access outright, so a skip
  advances the cursor and emits nothing. The suppression rule itself belongs to
  VecElemAgen; this module owns only the cursor movement that makes it free.

  ---- 5. Every access class, including unit-stride ----

  //@req-spec-agen.e12
  The read above is performed for EVERY access class, unit-stride included. A US
  OP.v passes through stage 1 too, where it is encoded as a single range entry,
  so the mask is read here for it as well and then TRAVELS WITH THAT ENTRY to the
  drain-side coalescer; the stage 2 Packer reads no VRF port at all. `us_mask` is
  the publication point for that carriage — the tail-cleared `mask_q` plus its
  `rob_idx` — held while VecLsu routes it into that direction's `VecRangeAgen` as
  the `mask`/`vm` half of its `io.scalar` input, which attaches it to the range
  entry.

  // ===> THIS RULE IS WHY THE MODULE SITS AT VecLsu LEVEL. Inside the element
  // agen, a unit-stride OP.v never reached a mask reader, because unit-stride
  // self-selects into `VecRangeAgen` — which has no reader and adds no VRF port.
  // The requirement is spec-agen.e12 and it is not satisfiable from inside the
  // element agen at all.

  Were stage 2 to read the mask itself, a US OP.v in stage 2 and an SSI OP.v in
  stage 1 would be two concurrent readers of `R1`, which a statically partitioned
  file cannot serve at all. Duplicating the reader into the range agen fails the
  same way, and against a named requirement: `spec-vrf.g18` gives `R1` and `R4`
  EXACTLY ONE reader each. The cost is carrying up to VLMAX mask bits on one
  bundle per OP.v — wide, but one bundle, and the alternative does not exist.

  ---- 6. Single owner of "which elements are active" ----

  The `staged`/`ahead` cursor and `us_mask` are the ONLY mask-derived values in
  the vector LSU. The store DGEN pairs its data beats to this cursor rather than
  re-deriving survival from `v0`; the LCB places responses by the destination PRN
  and byte offset the cursor stamped on the nOP.v; the beat expander coalesces
  from the range entry's carried mask. None of them reads a mask register, and
  none applies its own `vl` comparison.

  ---- 7. State, kill, and what this module must never grow ----

  The only state is `mask_q`, `mask_valid`, `elem_ptr` and the latched `op_vl` /
  `rob_idx` needed to interpret them — the streaming cursor and nothing else. On
  `kill` or on `done` all of it clears in one cycle, with no drain, no queue and
  no multi-cycle teardown, so a squash costs nothing here.

  IN PARTICULAR, DO NOT ADD A SECOND OP.v SLOT HERE. The next `OP.v` of this
  direction is held in `VecLsu`'s per-LDQ/STQ-entry descriptor pending table — the
  THIRD legal home for in-flight vector-LSU state, which plan ground rule 6 was
  amended to permit for exactly this hand-off — and it is presented only once this
  cursor has retired. A queue of pending descriptors here would be per-instruction
  state in a module that has no LSQ index to bound it, which is the shape the rule
  still forbids.

  It exports no `busy` and no ready signal that any issue unit sees. That is the
  vector-LSU group invariant, and a `busy` added here would be a failed review
  regardless of measured performance: the previous attempt's single-FSM AGEN,
  with its concurrency ceiling of one, is what this structure exists to avoid.

  ---- 8. Trace ----

  There are no unit tests in this project; validation is end-to-end VCS plus
  Whisper cosim, so emit `VecTrace` lines — gated on the `vecTrace` plusarg and
  `!reset`, off by default, tagged with the module name and `rob_idx` — one each
  on the mask read (`pvm`, elided or not), the latch load (`op_vl` and the
  active-element popcount), every skip (`elem_ptr`, `skip_log2`), and `done`. The
  popcount and skip lines are what make a masked-store divergence identifiable
  from the cosim log without a waveform. No non-trace logic may read them.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput: one element decision per cycle when elements are active, and
`1 << maxSkipLog2` elements per cycle across a fully-masked-off run. The cursor
must never insert a bubble between two consecutive active elements — a wait
state mid-stream is the failure the lookahead exists to prevent.

Latency: the mask is available to the agens 1 cycle after the stage-1 grant — the
VRF read port is a REGISTERED ONE-CYCLE port at the `VecRegFile` boundary, and the
bank's 0-cycle array read is internal to that cycle — or in the grant cycle itself
when `op_masked` is false and the read is elided. That single cycle is the whole
mask cost of an OP.v.

Occupancy, which is a VecLsu-level property and stated here so both sides agree:
ONE OP.v per direction at a time. A unit-stride OP.v occupies the cursor for one
cycle (load the latch, publish `us_mask`, retire); an SSI OP.v occupies it for the
walk. VecLsu's descriptor table is what holds the next OP.v of that direction in
the meantime, and no `ready`, `busy` or occupancy bit leaves this module toward an
issue queue to express it.

Frequency: single `core_clk` domain, 1 GHz target. Two paths to watch — the
priority-encoder window (bounded by `maxSkipLog2`, deliberately not `vLen`), and
the `vLen`-bit tail-clearing AND at latch load, which is outside the cursor loop,
done once per OP.v, and must stay there.

Area: `vLen` mask flops plus the cursor, per direction, two instances. That is
the entire storage cost, and it is why re-reading the mask per element was never
the cheaper option.
<|end_perf|>

<|begin_dependencies|>
VectorParams — `vLen`, `vecPregSz`, `vecVLSz`, `maxMembers`.
VecBundles — the cursor fields this module stamps are the nOP.v-scoped fields of
`VecElemAccess`/`MicroOp`, declared there and in the MicroOp delta, not here.
VecTrace — the guarded trace helpers.

Instantiates nothing. Instantiated by VecLsu as `ld_msk` and `st_msk` — one per
direction, two per core. It is NOT instantiated by VecElemAgen: that was the
pre-hoist arrangement and it left the unit-stride path with no mask reader.

Couples, without instantiating, to:
- VecRegFile — read port `R1` (load) or `R4` (store), by number, per
  midcore.rst `vrf-ports`. Adds no port. Its read is one REGISTERED cycle at that
  boundary.
- VecIdxGen — shares `R4` on the store path only. VecLsu owns the 2:1 mux (mask
  wins, index waits) and `owns_port` is the hold-off; VecIdxGen holds its request
  until granted.
- VecElemAgen — consumes `staged`/`ahead`/`skip_log2`/`all_inactive` and returns
  `step`/`skip`, routed by VecLsu rather than by a parent-child port.
- VecRangeAgen / VecBeatExpander — receive `us_mask` by carriage on the range
  entry, routed by VecLsu into `VecRangeAgen.io.scalar`.
- VecSquashUnit — resolves `kill`; this module is client 0 (`ld_msk`) / 1
  (`st_msk`) of VecLsu's five, an order that is positional and fixed.
- VecLsu — drives `op`/`op_masked`/`op_vl`, owns the `R4` mux, and holds the next
  OP.v of the direction in its descriptor table while this cursor is occupied.
- VecDgen and VecLoadCoalescingBuffer — consume the published cursor and must
  not evaluate the mask again.
<|end_dependencies|>

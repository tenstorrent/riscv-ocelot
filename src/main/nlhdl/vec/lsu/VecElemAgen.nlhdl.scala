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
  VecElemAgen — the FILL-side stage-1 address generator for the STRIDED, INDEXED
  and SEGMENTED (SSI) vector memory classes: it walks the element stream of one
  OP.v and pushes ONE element access (nOP.v) per ACTIVE element into that
  direction's `*_SSI_ADDR_Q`.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecElemAgen.scala,
  package boom.v4.vec.generated.lsu, group vec_lsu.
  depends_on MicroOp, VecBundles, VectorParams, VecTrace.
  Instantiates ONLY `idx` (VecIdxGen), with `isStore` forwarded. Instantiated
  TWICE by VecLsu — `ld_elem_agen` and `st_elem_agen` — so the two directions
  never arbitrate for anything: the load-priority mux over a shared unit is what
  silently dropped store grants in `addvector` (plan section 2, change 3).
  VecMaskStream is NOT instantiated here. It was HOISTED to VecLsu (`ld_msk` /
  `st_msk`), because the unit-stride path also needs a mask reader
  (spec-agen.e12) while `R1`/`R4` may have exactly one reader each
  (spec-vrf.g18). This module RECEIVES that streamer's cursor as an input.

  ===> IT ABSORBS THE INHERITED SKIPPER AND WALKER, AND THAT IS ONE DATAPATH, NOT
       TWO. The Skipper (strided / segmented) and the Walker (indexed) differ in
       exactly one place — WHERE THE PER-ELEMENT ADDRESS COMES FROM — and in
       nothing else: both pack whole segments per element, both apply the same
       mask cursor, both suppress masked-off elements, both emit the same nOP.v
       into the same queue at the same cadence. The inherited six modules
       (Packer/Skipper/Walker x load/store, 2110 lines) had already drifted into
       DIVERGENT MASK SUPPORT between the load and store Packers, worked around
       by forcing masked stores onto the slower Skipper. One walk with a 2:1
       address-source mux is the fix; two walks would reintroduce the drift with
       better names.

  ===> THREE CARRY-OVER LOOKAHEAD HAZARDS. Found by bring-up, not by design, not
       optional, each specified below where it bites: (1) do not start, and do not
       emit, an element access whose MASK BIT is not yet staged; (2) the same rule
       for its INDEX entry, gating both "start" and "release the final segment" on
       `idx.stall`; (3) complete by TOTAL BYTES (`vl << eew`) with a partial final
       member, never by a hardcoded 8-member walk — the phantom-member walk is the
       back-to-back vector-store data-corruption bug.

  ===> IT RETIRES ITS ELEMENT CURSOR THE MOMENT THE LAST ELEMENT IS PUSHED, AND
       EXPORTS NO `busy`. It never waits on a D$ response, a translation or a
       completion, so it cannot gate the next instruction. Plan ground rule 6
       names this module in the invariant's own text; a `busy`-shaped output here
       is a failed review regardless of measured performance.

  ===> FAULT-ONLY-FIRST IS NOT THIS MODULE'S POLICY, AND THE SIX OBLIGATIONS THAT
       STATE IT HAVE MOVED. `vle<eew>ff.v` is architecturally UNIT-STRIDE, so its
       OP.v self-selects into this direction's `VecRangeAgen` by the class rule in
       logic section 1 and NEVER REACHES THE ELEMENT AGEN AT ALL. The element-0
       trap versus trim-at-element-`i` policy is therefore allocated to
       `VecRangeAgen`; an earlier revision of this file carried it only because
       the architect pass mis-allocated it, which is a routing error and not a
       disagreement about the mechanism. What stays here is the ELEMENT-WALK fault
       path of logic section 8 — latch the first faulting element, stop the fill
       cursor, trap with `vstart = 0` — and the `ff_trim` port, which survives for
       symmetry only (VecLsu accepts a trim from EITHER agen so neither direction
       is a special case) and can fire only for a hypothetical non-unit-stride
       fault-only-first form that RVV 1.0 does not define.

  Governing spec anchors: execution.rst `vector-agen` (selection rule, Skipper,
  Walker, mask rule) and `vector-dgen`; loadstore.rst `ssi-queues`,
  `elem-progress` (per-element progress and the precise-exception rule; the
  fault-only-first policy in that same anchor is `VecRangeAgen`'s),
  `vec-load-algo`, `vec-store-algo`; midcore.rst `precise-vec-exc` and
  `vrf-ports`; glossary.rst `glossary-terms`; issue.rst
  `vec-queue-reservation` and `shared-store-chain`.
*/

<|begin_module|>

  <|begin_parameters|>
  `isStore` — Boolean, REQUIRED, no default. VecLsu states it on both instances;
  defaulting it would quietly give the store path load semantics, a correctness
  bug rather than a mis-tuning. It selects four things: which
  `VecScalarOperandRead` feeds this instance, which `VecMaskStream` cursor
  arrives, which `*_SSI_ADDR_Q` is filled, and which VRF read port the child uses.
  It selects NOTHING about the walk — execution.rst: "the generator is selected by
  access class, not by direction".

  `isStore` is FORWARDED to `idx` and is that child's only parameter. Its VRF read
  port is derived there (`R0` load, `R4` store, shared with that path's mask read)
  per the canonical 0-based table in midcore.rst `vrf-ports`. Nothing here adds a
  VRF port and this module owns none itself — it only passes the child's request
  and response through to VecLsu.

  Sizes come from VectorParams via `HasVectorParams` (`vLen`, `eLen`,
  `maxMembers`, `vecPregSz`, `vecVLSz`, `ssiQueueEntries`) and from
  `HasBoomCoreParameters` (`xLen`, `vaddrBitsExtended`, `robAddrSz`, `ldqAddrSz`,
  `stqAddrSz`). No width is a literal. Derived, as named values:
    `vLenBytes = vLen / 8`                   bytes per destination member (PRN)
    `elemIdxSz = vecVLSz`                    element cursor width (9 bits)
    `qIdxSz    = log2Ceil(ssiQueueEntries)`  absolute queue index width
    `segIdxSz  = 3`                          field cursor; `nf` is 3 bits in RVV

  There is deliberately NO skip-window parameter. The power-of-two skip distance
  is an OUTPUT of VecMaskStream (its `maxSkipLog2` sizes its own priority
  encoder), so this module consumes a sanctioned distance and can never advance
  the cursor by a distance the streamer did not grant. Two parameters naming one
  window is how the two sides come to disagree about it.

  No `usingRVV` parameter: the module is elaborated only from inside VecPipeline,
  which exists only when `usingRVV` is set, so a vectors-off build has no instance
  at all — ABSENT, not tied off (ground rule 1). Do not gate on rocket's
  `usingVector`, a different gate. Elaboration requires `vLen % eLen == 0` so the
  member/offset split below is a shift and a mask; the `ssiQueueEntries >= vLen`
  liveness floor is VectorParams' check and is not restated.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit pair, per the hierarchy defaults: posedge
  `core_clk`, ACTIVE-HIGH SYNCHRONOUS `core_reset`. One domain, no asynchronous
  reset, no latch. The walk-active bit, the pending-descriptor valid and the fault
  latch reset to 0.

  ---- `io.op`, Input(Valid(new VecScalarOperands)) ----

  One cycle per granted OP.v, from this direction's `VecScalarOperandRead`
  (`ld_opnd` / `st_opnd`). Fields read: `uop`, `base`, `stride` (both `xLen`),
  `vl` (`vecVLSz`), `vl_zero`. THERE IS NO `ready` ON THIS PORT and there must not
  be — a ready here is a back-pressure path from the element queues into the issue
  unit under another name, which is exactly what the dispatch-time reservation
  exists to remove. See the logic section's accept rule for what happens instead.

  Every static access property arrives on the wrapped uop and is NEVER re-decoded
  here: `v_mop`, the `v_is_*` class flags, `v_is_masked`, `v_eew` (the DATA
  element width), `v_idx_eew` (the INDEX width), `v_emul`, `v_seg_nf`, `pvs2`,
  `pvdest`, `pvtmp`, `rob_idx`, `ldq_idx`, `stq_idx`, `br_mask`. A second decoder
  disagreeing with `VLSDecode` would send an op to the wrong agen.

  ---- The mask cursor, from the parent's `ld_msk` / `st_msk` ----

  Inputs, exactly the shape VecMaskStream publishes: `msk_staged` and `msk_ahead`
  ({`valid`, `elem`, `active`, `last`} each), `msk_skip_log2` with
  `msk_skip_valid`, and `msk_all_inactive`. Outputs back: `msk_step` (advance one
  element) and `msk_skip` (advance by `msk_skip_log2`), at most one per cycle.

  This module does NOT drive the streamer's `op`/`op_masked`/`op_vl`/`kill` start
  interface — VecLsu does, since the same streamer also serves that direction's
  `VecRangeAgen`. Consequence, stated from this side: the streamer holds ONE
  OP.v's cursor per direction, so a direction's element walk and its range entry
  cannot be in the streamer at once.

  ---- The child's VRF index read, passed through ----

  `vrf_read_req` (Output, `valid` + `prn` of `vecPregSz`), `vrf_read_gnt` (Input
  Bool) and `vrf_read_rsp` (Input, `valid` + `data` of `vLen`) wire straight to
  `idx.io.vrf_read` with no logic between. VecLsu binds them to `R0` on the load
  instance and `R4` on the store instance, where the grant is that path's static
  hold-off in favour of the once-per-OP.v mask read.

  ---- Reservation lanes, to `resv` (VecQueueReservation) ----

  `resv_lookup` (Output Valid({`is_store`, `q_idx`})) with `resv_resp` (Input
  `Vec(2, { base, count })`): the combinational read of the region this OP.v owns,
  keyed on `uop.ldq_idx` for a load and `uop.stq_idx` for a store. `release`
  (Output Valid({`is_store`, `q_idx`, `used_count`: `Vec(2, UInt)`})) with
  `release_ok` (Input Bool): the surplus return. Lanes are fixed by VecLsu — 0 is
  `ld_elem_agen`, 1 is `st_elem_agen`. A denied release is NEVER retried.

  // ===> BOTH OF THESE ARE PER QUEUE SLOT, AND THE WIDENING IS DELIBERATE —
  // SLOT 0 IS THE ADDRESS QUEUE, SLOT 1 THE DATA QUEUE. `VecQueueReservation`
  // declares `resv_resp` as `Vec(4, Vec(2, {base, count}))` and
  // `release.used_count` as `Vec(2, UInt)`, because a US store's address and data
  // regions are NOT in identity correspondence and therefore need a base and a
  // count each. THE CHOICE MADE HERE IS TO WIDEN THIS MODULE'S DECLARATION TO
  // MATCH, not to have VecLsu fan slot 0 out to it — recorded so the next reader
  // does not have to infer it. The two sides then declare one shape, and a
  // scalar-versus-`Vec(2, ...)` mismatch cannot generate a file that fails to
  // elaborate.
  // ===> FOR AN SSI ACCESS THE TWO SLOTS ARE EQUAL, AND THIS MODULE ASSERTS IT
  // RATHER THAN ASSUMING IT. The two SSI queues have equal depth and are claimed
  // by one event with one count, so their allocation pointers are provably
  // identical; assert `resv_resp(lane)(0)` equals `resv_resp(lane)(1)` on every
  // store lookup, then read SLOT 0 for the address arithmetic below. That
  // assertion is what licenses the whole absolute-index pairing of logic rule 7 —
  // VecDgen writes `st_SSI_DATA_Q` at the SAME absolute index this module wrote
  // the address to, which is only meaningful while the two regions start at the
  // same base. Do NOT assert slot equality for a unit-stride op: that is the
  // range agen's class, and there the slots genuinely differ.

  ---- The queue fill, to `{ld,st}_SSI_ADDR_Q` ----

  `addr_enq` — Output `Decoupled({ idx: UInt(qIdxSz.W), data: VecElemAccess })`,
  driving fill lane 0. `idx` is ABSOLUTE — the reservation base plus this OP.v's
  own emitted-access counter, never a FIFO write pointer, because issue is
  age-ordered-READY and a younger OP.v's agen may fill its region before an older
  one has started. `ready` is the streaming back-pressure of logic rule 7.

  Only ONE fill lane is driven even at `lsuWidth = 2`, a property of the class
  rather than a shortcut: an SSI address stream is a serial chain (each strided
  address is the previous plus a stride; each indexed address needs the next index
  entry), so no second address exists in the same cycle. Unit-stride bandwidth
  comes from the DRAIN side instead.

  ---- `elem_pub`, store instance only ----

  Output(Valid({ `q_idx`, `elem_idx`, `seg_idx`, `active`, `first`, `last` })),
  elaborated only when `isStore`. VecDgen consumes it to write `st_SSI_DATA_Q` at
  the SAME absolute index this module wrote the address to. Not elaborated on the
  load instance, whose carriage of the same information is the pushed
  `VecElemAccess` itself; a dangling port would invite a second consumer.

  ---- Faults ----

  `fault` — Input(Valid(UInt(elemIdxSz.W))): the faulting ELEMENT index the drain
  side reported for the OP.v this instance is walking, already qualified by VecLsu
  against the walking `rob_idx`. `fault_trap` — Output(Bool): raise this fault to
  the ROB as a plain precise exception. `fault_elem` IS NOT A PORT (ground rule 8).

  `ff_trim` — Output(Valid(UInt(vecVLSz.W))) — EXISTS FOR SYMMETRY AND IS EXPECTED
  NEVER TO FIRE. A fault-only-first load is unit-stride, so its OP.v is
  `VecRangeAgen`'s and the trim policy is stated there; this port lets VecLsu wire
  one shape to both agens rather than special-casing a direction. When it does
  fire, VecLsu routes it to `lcb.io.trim`, NOT to VlRegFile's `W_lsu` — the VL
  register file's `W_lsu` port has exactly ONE producer, `lcb.io.vl_wb` on the
  group-done, because a trimmed VL published before the group it describes has
  been assembled would wake every `pvl` dependent too early. Assert this port
  stays invalid for every op this instance actually walks.

  ---- Kill ----

  `brupdate` — Input(new BrUpdateInfo) — and `rob_flush` — Input(Bool), resolved
  HERE with BOOM's existing `IsKilledByBranch`/`GetNewBrMask` (ground rule 10, no
  new squash mechanism) rather than arriving pre-resolved, because this module
  holds TWO uops with DIFFERENT `br_mask`s and one resolved Bool from the parent
  could only be right for one of them. The walking descriptor's resolved kill goes
  to `idx.io.kill`.

  ===> AND THAT IS THE WHOLE INTERFACE. No `busy`, no `active`, no `fu_ready`
  contribution, no credit, no "current instruction" identifier. Issue eligibility
  for a vector memory OP.v is decided at dispatch by VecQueueReservation and
  nowhere else.
  <|end_ports|>

  <|begin_logic|>
  ---- 0. State: the complete list ----

  A walking descriptor (`w_valid` plus latched `uop`, `base`, `stride`, `vl`,
  `resv_base`, `resv_count`); the element cursor `elem_ptr`; the field cursor
  `seg_ptr`; the strided address accumulator `addr_acc` (`xLen`); the
  emitted-access counter `emit_ctr` (`qIdxSz`); a one-deep PENDING descriptor of
  the same shape, now an ASSERTION RATHER THAN A MECHANISM (see rule 1); and the
  fault latch `fault_seen` with `fault_elem`. All of it retires with the walk, none
  of it outlives the OP.v, and none of it is a `busy`.

  ---- 1. Which OP.v this instance takes, and the accept rule ----

  //@req-spec-agen.b5
  //@req-spec-agen.c24
  //@req-spec-agen.c6
  //@req-spec-agen.c14
  This instance takes an `io.op` whose uop is `is_vec`, belongs to this direction
  (`uses_stq` when `isStore`, else `uses_ldq`) and is SSI-class:
  `v_is_strided || v_is_indexed || v_is_segment`. That is exactly execution.rst's
  selection — INDEXED to the Walker, non-indexed-and-not-unit-stride (strided,
  segmented) to the Skipper, unit-stride to the stage-2 Packer — so stage 1
  expands ONLY strided, indexed and segmented load/stores into nOP.v bundles, and
  the Skipper path is used instead of the Packer for every access that is not
  unit-stride. Both classes are taken WHETHER OR NOT A MASK IS PRESENT: masking is
  an optimization here and not a selection criterion, and with no mask active the
  walk simply visits every element at `base + i*stride`.
  The complementary set — `v_is_unit_stride`, `v_is_whole_reg`, `v_is_mask` — goes
  to this direction's `VecRangeAgen`. Both agens see the same broadcast `io.op`
  and each takes its own class, so there is no routing mux to get wrong; assert
  the two selections are mutually exclusive and jointly exhaustive.
  `v_is_ff` LIVES INSIDE `v_is_unit_stride`, so a fault-only-first load is on the
  range agen's side of that line and never on this one — RVV 1.0 defines the
  fault-only-first form only for unit-stride loads. Assert `!v_is_ff` on every
  accept here: the assertion is the standing proof that this module's fault path
  need not implement the trim, and it converts a future decode regression into a
  loud failure instead of a `vleff` walked element-by-element with no trim owner.

  Given no `ready` on `io.op`, an accepted OP.v enters the walking descriptor when
  idle and the one-deep PENDING descriptor while a walk runs. That slot covers a
  single back-to-back grant (at `vecIssueGrantWidth = 1` at most one FC_AGEN grant
  per direction per cycle reaches `VecScalarOperandRead`), and it is what this
  module invented before the hazard had an owner.

  ===> THE MID-WALK ARRIVAL IS NOW VecLsu's, AND THE SLOT HERE IS AN ASSERTION.
  VecLsu holds ONE per-LDQ/STQ-entry PENDING DESCRIPTOR TABLE — one row per LSQ
  entry, written from `xx_opnd.out`, presenting at most one descriptor per
  direction and only in a cycle that direction's mask streamer is free. That one
  structure absorbs BOTH hazards that reach this port: the second grant arriving
  mid-walk, and `PartiallyPortedRF` denying the INT read that produces `base` and
  `stride` (the vector lanes are appended last against five physical ports, so
  denial is routine, not exotic). Plan ground rule 6 was AMENDED to permit that
  table as a THIRD legal home for in-flight vector-LSU state — alongside the six
  `VecElemQueue` instances and the LCB's per-PRN assembly entries — because it is
  the same kind of state: per-queue-entry, capacity reserved at dispatch,
  structurally un-overflowable, exporting no `busy`.

  So DO NOT DUPLICATE IT HERE. This module keeps its downstream contract unchanged
  — `io.op` is a `Valid` latched unconditionally, and only the link UPSTREAM of
  `VecScalarOperandRead` gained a `ready` — and the pending slot survives as a
  CHECK: assert it is free on every accept, and trace it if not. With VecLsu's
  table wired the slot is provably never occupied, and a wiring regression then
  shows up as a firing assertion instead of a silently lost descriptor. Answering
  the hazard here with a ready line on `io.op` would reintroduce the
  queues-to-issue back-pressure path, which is what the dispatch-time reservation
  exists to remove.

  //@req-spec-lsu.j1
  What the accept produces is the point of the stage: the addresses a vector load
  drains PER ELEMENT out of `ld_SSI_ADDR_Q` when it issues are PRECOMPUTED HERE,
  by stage 1, one queue entry per active element. The drain side computes no SSI
  address and reads no index vector. `resv_lookup` fires in the accept cycle, its
  per-slot equality is asserted for a store, and SLOT 0 is latched as
  `resv_base`/`resv_count` — one pair of registers, not two, because for this
  module's classes the second slot is provably the same numbers and latching both
  would create a second copy that could only ever disagree by being wrong.

  A `vl_zero` OP.v, and equally one the streamer reports `msk_all_inactive` for,
  emits NO element access at all: no address, no data pairing, no memory traffic.
  It releases its ENTIRE region (`used_count` zero IN BOTH SLOTS) and retires in
  its accept cycle. Making its freshly renamed `pvdest` group architecturally correct is
  VecGroupCopy's job; nothing here retracts an access, because none was emitted.

  ---- 2. The address: one adder, a 2:1 source mux, no multiplier ----

  //@req-spec-agen.c19
  //@req-spec-agen.c8
  //@req-spec-agen.c18
  The access address is `elem_base + (seg_ptr << v_eew)`, narrowed to
  `vaddrBitsExtended` only at the queue payload so wraparound happens at `xLen`
  and matches the ISA. `elem_base` has two sources, and they are the whole
  difference between the inherited Skipper and the inherited Walker:

  - STRIDED / SEGMENTED: the running accumulator `addr_acc`, initialized to `base`
    at accept and advanced by `stride` per step (rule 4 for a skip). No
    `elem_ptr * stride` product is ever formed.
  - INDEXED: `base + idx.offset`, added FRESH PER ELEMENT at `xLen`, so each
    element gets its OWN computed address as the Walker requires. `idx.offset`
    arrives from the child ALREADY ZERO-EXTENDED from `v_idx_eew` to `xLen`; do
    NOT re-extend it and do not open-code a second extension — two extension sites
    is two chances to get the narrow-EEW arm wrong.

  // ===> RVV 1.0 INDEXED OFFSETS ARE UNSIGNED, NOT SIGNED. Spike's
  // `VI_LDST_GET_INDEX` reads them as `uint8_t`/`uint16_t`/`uint32_t`, and Whisper
  // — the cosim reference this design is validated against — agrees. A
  // sign-extension would diverge on EVERY index whose top `v_idx_eew` bit is set,
  // and the divergence would present as an LSU addressing bug rather than as an
  // extension bug, which is exactly the kind of mis-diagnosis that costs days.
  // Earlier revisions of the map and the plan said "signed"; that was a spec
  // defect and is corrected. The extension lives in ONE named function inside
  // VecIdxGen so a future correction stays a one-line change; this side asserts
  // nothing about the extension beyond not repeating it.

  // ===> AND THE INDEX STRIDE COMES FROM `uop.v_idx_eew`, NEVER FROM `uop.v_eew`.
  // For an indexed access the two are INDEPENDENT: `v_eew` is the DATA element
  // width (how wide each loaded or stored element is, and therefore how the
  // destination bytes are placed) while `v_idx_eew` is the INDEX element width
  // (how wide each entry of the index vector is, and therefore the stride at which
  // `idx` walks `pvs2`). Sourcing the child's descriptor field from `v_eew` walks
  // the index vector at the WRONG STRIDE and silently reads the wrong offsets —
  // it produces plausible addresses, which is why it survives a waveform glance.
  // `v_eew` is used HERE, for `seg_ptr << v_eew` and for the byte placement, and
  // NOWHERE in the index descriptor.

  SEGMENTS ARE PACKED PER ELEMENT on both paths. For a segmented access the cursor
  is the pair {element, field}: element `i` contributes `v_seg_nf` accesses at
  `elem_base + f*(1 << v_eew)`, emitted over `nf` consecutive cycles, and only
  then does `elem_ptr` advance. `nf` does not divide the mask — one mask bit
  governs the whole segment (rule 3).

  The destination placement stamped on each nOP.v is the {PRN, byte-offset-within-
  PRN} pair derived from the access's byte position: member `byte_pos >>
  log2(vLenBytes)`, offset `byte_pos & (vLenBytes-1)`. For a non-segmented SSI
  access the group is `pvdest`; for a SEGMENTED access it is the `pvtmp`
  rendezvous group, the field transpose being the coprocessor's half.

  // ===> EVERY OFFSET ON THE nOP.v IS IN BYTES, never an element index. The mask
  // index is the exact inverse — element-granular, NOT scaled by eew — and
  // getting the two the same way round corrupts only element 0 of a misaligned
  // access and hides everywhere else. Paid for once in M1 bring-up, where it read
  // as an LSU fault rather than as an offset bug.

  //@req-spec-core.c13
  The nOP.v-scoped cursor fields of the wrapped MicroOp — `v_split_first`,
  `v_split_last`, `v_split_idx`, `v_split_total`, plus the target PRN and byte
  offset — are POPULATED HERE AND ONLY HERE, in the cycle the access is pushed.
  They are inert on the OP.v itself: flowing through decode, rename, the ROB and
  issue they are don't-care and nothing outside the vector LSU may read them.
  This module being their sole writer is what makes "populated only when the AGEN
  emits element accesses" structural rather than conventional.

  ---- 3. The mask: suppress the access, never flag it ----

  //@req-spec-agen.e2
  //@req-spec-agen.e6
  //@req-spec-agen.e7
  The mask is APPLIED DURING ADDRESS GENERATION, from the streamer's cursor:
  `msk_staged.active` is the single predicate. The streamer already cleared the
  tail above `vl`, so no private `elem_ptr < vl` comparison is made here — a
  second `vl` comparison drifts exactly as a second mask evaluation does. When the
  bit is clear the access is SUPPRESSED: `addr_enq.valid` stays low for every
  field of that element, so a masked-off element produces NO nOP.v and therefore
  no D$ access, no TLB translation and no LCAM search. The cursor still advances
  and the accumulator still adds, because the element occupies address space even
  though it is not accessed.

  //@req-spec-agen.c7
  //@req-spec-agen.e8
  //@req-spec-agen.e9
  The inherited Skipper SKIPPED masked-off elements rather than fetching them;
  Caracal keeps the skip and deletes the "fake" packets the bobtail unit emitted
  for skipped regions, because an access that reached memory would be WRONG and
  not merely wasteful. A masked-off STORE element must leave memory unmodified,
  and the source vPRN's inactive lanes hold coprocessor data rather than the
  memory's prior contents, so an access that reached the D$ would clobber bytes
  the instruction never wrote. A masked-off LOAD element must not raise a memory
  exception, so an access that reached the TLB could fault on an address the
  instruction never architecturally touches. Suppressing the access satisfies
  both, which is why the suppression lives here and not downstream.

  //@req-spec-agen.e11
  The resulting cursor is the SINGLE OWNER of which elements survive, and
  `elem_pub` is its publication point. VecDgen pairs its store data beat to this
  cursor, and the LCB counts the bytes it waits for from the same cursor carried
  on the nOP.v; NEITHER MAY EVALUATE THE MASK AGAIN. Two evaluations agree until
  the first place their tail or `vl` handling differs, and then a store pairs
  element k's address with element k+1's data — invisible in every unmasked test.

  ---- 4. Skipping runs of disabled elements, in power-of-two strides ----

  //@req-spec-agen.c10
  THE POWER-OF-TWO SKIP IS CLASS-CONDITIONAL: it is enabled on the STRIDED and
  SEGMENTED paths and is NEVER enabled on the INDEXED path. Gate it on
  `!uop.v_is_indexed`, in one place, as a condition on `msk_skip` itself rather
  than as a scattered set of exceptions. When the staged element is INACTIVE on a
  skip-enabled path the walk jumps past the whole run instead of walking it:
  assert `msk_skip`, advance `elem_ptr` by `1 << msk_skip_log2` and `addr_acc` by
  `stride << msk_skip_log2`. The distance comes from the streamer's priority
  encoder over the mask bits — the inherited Skipper's mechanism — and is
  consumed, never recomputed.

  // The class condition is not a tuning choice, it is what makes this requirement
  // and `spec-agen.c16` below SIMULTANEOUSLY SATISFIABLE: taken unconditionally
  // they contradict each other on the indexed path, because `VecIdxGen.taken`
  // pulses once per element INCLUDING masked-off ones. Both IDs are kept and both
  // are read class-conditionally; execution.rst's selection text has been amended
  // to say so, so the corpus and the RTL now agree rather than the RTL carrying a
  // silent local exception to a requirement that reads as unconditional.

  // The jump is a POWER OF TWO for a datapath reason, not for rounding
  // convenience: `stride << k` is a shifted add, while an arbitrary distance `d`
  // would put `stride * d` — a multiplier — inside the cursor's feedback loop.

  //@req-spec-agen.c16
  ON THE INDEXED PATH THERE IS NO SKIP, and that is the Walker's defining property
  rather than an omission: `idx.taken` pulses ONCE PER ELEMENT including masked-off
  ones (the index vector has an entry for a masked-off element too), so a jump of
  `1 << k` elements would need `k` index hand-offs in one cycle. The Walker
  therefore WALKS THROUGH ELEMENTS ONE AT A TIME, `msk_step` and `idx.taken`
  together, and a masked-off indexed element costs one cycle and emits nothing.
  Making the skip conditional on the class — the single `!v_is_indexed` gate above,
  and no second exception anywhere — is what keeps this requirement and the
  power-of-two skip from contradicting each other in the RTL.

  ---- 5. The lookahead hazards. Carried over from bring-up; not optional. ----

  //@req-spec-agen.c20
  Two conditions gate the walk and each gates BOTH "start" and "release":
  - MASK: do not start, and do not emit, an element access whose mask bit is not
    yet staged. The gate is `msk_staged.valid && msk_ahead.valid`.
  - INDEX: on the indexed path do not START, and do not RELEASE THE FINAL SEGMENT
    of the element in flight, while `idx.stall` is high. `idx.stall` is VecIdxGen's
    own exported name for `!valid || !next_valid`, so this module gates on one
    signal instead of re-deriving the hazard and disagreeing about it.

  With either gate low the walk WAITS AT AN ELEMENT (or segment) BOUNDARY with
  nothing half-formed in flight: it never enters a wait state mid-stream with a
  partially released segment, and never emits an address built from a staging
  register that has not been written. The distinction matters because the failure
  is a WRONG ADDRESS, not a slow one. The inherited Walker's rule was "to avoid
  stall/wait states it won't release the final segment or start until the next
  index arrives", and an agen that walked ahead of the index stream computed
  addresses from stale or undriven staging registers.

  // VecIdxGen FORCES `next_valid` high at the last element. Rely on that; do NOT
  // add a local "except at the end" term. The exception belongs on the producing
  // side, and duplicating it here is how the final element of every indexed
  // access comes to wait forever for an index entry that does not exist.

  `idx.io.start` is handed {`pvs2`, `v_idx_eew`, `vl`, `rob_idx`} in the accept
  cycle for an indexed op and ONLY for an indexed op — a strided or segmented
  access has no index vector, and starting the child would occupy a VRF port for
  nothing. Its `idx_eew` descriptor field IS `uop.v_idx_eew`, per rule 2's note;
  assert `v_is_indexed` on every `start` so a class the descriptor cannot describe
  can never reach the child.

  ---- 6. Completion by TOTAL BYTES, with a partial final member ----

  The walk completes when `elem_ptr` reaches `vl`, and every member-boundary
  quantity is derived from TOTAL BYTES rather than from a member count: total
  bytes per field are `vl << v_eew`, so the number of destination members actually
  touched is `ceil((vl << v_eew) / vLenBytes)` and a PARTIAL FINAL MEMBER is the
  normal case, not a special case — the walk stops at element `vl - 1` wherever
  inside a member that falls.

  // ===> A HARDCODED 8-MEMBER WALK IS THE BACK-TO-BACK VECTOR-STORE
  // DATA-CORRUPTION BUG from the M1 bring-up log. It streamed phantom members
  // after a 1-member op and then stalled, and the corruption showed on the NEXT
  // store rather than on the short one — which is why simulation found it and
  // review did not. Deriving the count from total bytes makes the phantom member
  // unrepresentable. `v_emul` may size a group; it may never bound the walk.

  ---- 7. The surplus release, and streaming the fill ----

  The release fires ONCE per OP.v, in the accept cycle, because that is the cycle
  VL becomes known (`VecScalarOperandRead` read it from the VL register file): the
  count is `vl` for a non-segmented SSI access, `vl * v_seg_nf` for a segmented
  one, 0 for the VL = 0 / all-inactive case, and it is DRIVEN ONTO BOTH SLOTS OF
  `used_count` — identically, since for an SSI store the address and data regions
  have the same base and the same count, and the reservation asserts as much. That
  is the one place this module's per-slot port shape does real work: the shape
  exists for the US store's non-identity, and stating "both slots, same value" here
  is what stops a reader from concluding an SSI store should trim only one of its
  two regions. The `nf` product is formed
  once per OP.v and never per element, so a narrow multiplier is acceptable here —
  unlike in the dispatch path, where VecQueueReservation must use a constant
  table. It trims to the VL-derived count only and NEVER to the mask-derived
  active count, since a later second trim could only be a mid-queue release. A
  denied release is not retried; the surplus frees in order with the region.

  //@req-spec-lsu.b11
  `addr_enq.idx` is `resv_base + emit_ctr`, where `emit_ctr` counts EMITTED
  accesses, so the filled region is COMPACT — masked-off elements leave no holes
  and the drain side never stalls on an unfilled entry inside a live region. When
  `addr_enq.ready` is low the walk stalls at that access and resumes when it
  clears; on a LOAD instance the drain's consume clears the entry's filled bit and
  re-opens the index, which is the mechanism by which a load's region is drained
  in waves and a load whose active element count exceeds its reservation is
  STREAMED through `ld_SSI_ADDR_Q`. There is no progress channel from the drain
  side; the back-pressure IS the channel. Assert `emit_ctr < resv_count`.

  // A store never stalls here in practice: it retains every filled entry until
  // commit-drain, and its reservation is the WORST-CASE count, which is >= vl >=
  // the active count — so it fills its region once, monotonically, and the
  // streaming path is load-only. By the same argument the active count can never
  // exceed the region at all, so the streaming clause may be dead code. It is
  // specified rather than deleted because the mechanism is what makes the wave
  // drain legal, and deleting it would silently couple fill to drain.

  //@req-spec-lsu.f8
  Accesses are emitted STRICTLY IN ELEMENT ORDER — and within a segmented element
  in field order — into a compact ascending index region. That ordering is what
  lets the drain side fire TLB and LCAM in element order, so a fault at element k
  is always detected before any element greater than k fires. The fire itself is
  VecBeatExpander's, driven by its in-order cursor advance over this region; this
  module owns the emission order, and the two sides together discharge the
  requirement.

  ---- 8. Faults: stop the cursor, and nothing else ----

  //@req-spec-lsu.f5
  //@req-spec-lsu.f9
  //@req-spec-rob.g13
  `io.fault` can only reach a walk still in progress, i.e. the streaming-load case
  above; a store has finished filling long before its translate pass runs. When it
  arrives, `fault_elem` is LATCHED ON THE FIRST FAULT — the index of the OLDEST
  faulting element, never overwritten by a later report — and from that cycle the
  ENTRY STOPS ADVANCING: no further access is emitted and
  `msk_step`/`msk_skip`/`idx.taken` stop pulsing.
  `fault_elem` survives ONLY as (a) that stop signal and (b) a debug/performance
  counter in the trace lines below. IT IS NOT A PORT, never reaches the ROB and is
  never written to `vstart`. The instruction traps with `vstart = 0` and restarts
  WHOLE, legally, because nothing was architecturally written: the faulting OP.v
  never commits, so its fresh `pvdest` group is reclaimed and elements `0..k-1`
  were never visible — resuming at `vstart = k` would leave them holding
  pre-instruction values, which is silent wrong data on any page-crossing gather.
  No path here can produce a non-zero `vstart`. Outstanding accesses below
  `fault_elem` need no cancellation and get none; the trap invalidates their
  landing site (the LCB assembly entry, by its owning `ldq_idx`).

  `fault_trap` therefore FOLLOWS `io.fault` UNCONDITIONALLY on this path, with no
  element-index split and no trim arm, because every class this instance walks —
  strided, indexed, segmented — takes the plain precise trap. The two-way split on
  `fault_elem === 0` belongs to fault-only-first, which is unit-stride and hence
  `VecRangeAgen`'s; keeping a copy of that policy here would give one architectural
  rule two implementations that could drift, and the class rule guarantees the copy
  here could never be exercised to catch the drift. `ff_trim` stays invalid.

  ---- 9. Kill and retire ----

  Both descriptors are killed independently, each on its own `br_mask` (updated by
  `GetNewBrMask` every cycle). A kill drops the descriptor, the cursors and the
  accumulator in one cycle and asserts `idx.io.kill`; there is no drain and nothing
  to retract, because queue tail rollback is VecSquashUnit's.

  THE WALK RETIRES IN THE CYCLE THE LAST ELEMENT'S LAST FIELD IS ACCEPTED BY THE
  QUEUE. It does not wait for a D$ response, a translation, an LCB assembly, a
  group-done or a commit, so the next descriptor — presented by VecLsu's pending
  table the cycle this direction's streamer frees — starts immediately and the next
  OP.v costs no dead cycle. TWO distinct element cursors exist and must not
  be confused: THIS module's FILL cursor, which lives here and retires early, and
  the LDQ/STQ entry's DRAIN cursor `elem_next`, which lives in the queue entry and
  is read-modify-written by VecBeatExpander. Nothing here touches `elem_next`.

  ---- 10. Why the segmented-store chain cannot close a cycle through here ----

  For a segmented store this module is STEP 1 of the six-step chain, and every
  operand of its address path comes from an instruction OLDER than the store: the
  base and stride GPRs read by `st_opnd`, the index vector read by `idx`, and the
  mask read by the parent's `st_msk` — all named by fields rename resolved before
  the store was dispatched. It waits on nothing produced by the store itself or by
  the store's coprocessor half, and it never consults `pvtmp` as a source. That is
  what makes step 1 unblockable and the chain a long serial dependency rather than
  a cycle: step 1's first translation clears `rob_unsafe`, the PNR advances, the
  coprocessor half becomes eligible and writes `pvtmp`, and only then does step 6
  — VecDgen reading `pvtmp` on a separate, later `FC_DGEN` grant — run.

  ---- 11. Trace ----

  There are no unit tests here; validation is end-to-end VCS plus Whisper cosim,
  so emit guarded `VecTrace` lines — one per key event, tagged with this module's
  name and `rob_idx`, gated on the `vecTrace` plusarg and `!reset`, off by
  default: `accept` (class flags, `vl`, `v_eew`, region base and count), `release`
  (`used_count`, granted or not), `emit` (`elem_idx`, `seg_idx`, address, `q_idx`,
  target PRN and BYTE offset), `suppress`, `skip` (`elem_idx`, `msk_skip_log2`),
  `stall` (which of the mask gate, the index gate or `addr_enq.ready` was low —
  that distinction is the whole diagnosis; first cycle of a run only), `fault`
  (`fault_elem`, always a trap on this path), `retire` (accesses emitted versus
  `vl`) and
  `kill`. The `emit` line's byte offset is what makes rule 2's
  bytes-versus-elements bug visible without a waveform, and `retire`'s
  emitted-versus-`vl` pair is what makes rule 6's phantom member visible. These
  lines declare no register and no wire a functional path reads.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput: ONE element access emitted per cycle, sustained, on both the strided
and the indexed path, with no wait state at a member or segment boundary at any
legal EEW. That constraint is what forces both lookahead gates to be tested one
element AHEAD rather than on the element in flight — a gate evaluated on the
current element inserts a bubble between every pair of consecutive active
elements. Across a fully masked-off run the rate is `1 << msk_skip_log2` elements
per cycle on the strided path, one per cycle on the indexed path (rule 4 says why
the latter cannot improve without a second index hand-off per cycle).

Latency: an OP.v accepted in cycle N emits its first access in N+1 for a strided
or segmented access, N+2 at the earliest for an indexed one (the child needs one
cycle to request its first index member and one for the VRF response). Paid once
per OP.v.

Critical path: the strided accumulator loop, `addr_acc + (stride << skip_log2)` at
`xLen`, in series with the mask gate. It must close in ONE cycle because the loop
is the cursor. Do not pipeline inside it; if timing forces a cut, take it on the
descriptor latch at accept, which is off the loop.

Area: one `xLen` accumulator, two cursors, one `qIdxSz` counter and two descriptor
registers (each a MicroOp), per instance, two instances. The second descriptor is
the pending slot — one MicroOp, not a queue. It is now an ASSERTION rather than a
mechanism (VecLsu's per-LDQ/STQ-entry pending table absorbs the mid-walk arrival),
so a synthesis-time evaluation may find it optimizes away entirely once the check
is the only reader; that is the intended end state, not a lost feature.

What this module deliberately does NOT try to be fast at is bandwidth. An SSI
element stream is one access per cycle by construction (scattered addresses cannot
be range-folded), so a pathological gather is element-serial however the rest of
the machine is sized; unit-stride throughput (plan targets P1 and P2) lands in
VecBeatExpander. The one headroom item worth recording: a SEGMENTED element's `nf`
accesses ARE consecutive and could fill two lanes per cycle at `lsuWidth = 2` —
deliberately not done, because a second fill lane would have to be specified
against VecElemQueue's second port and no requirement asks for it.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the wrapped uop on `io.op` and on every emitted `VecElemAccess`. Reads
`is_vec`, `uses_ldq`/`uses_stq`, `v_mop` and the `v_is_*` class flags,
`v_is_masked`, `v_eew`, `v_idx_eew`, `v_emul`, `v_seg_nf`, `pvs2`, `pvdest`,
`pvtmp`, `br_mask`, `rob_idx`, `ldq_idx`, `stq_idx`. It WRITES the nOP.v-scoped
cursor fields (`v_split_*`, target PRN, byte offset) and no others, and ADDS NO
FIELD to the bundle.

VecBundles — `VecElemAccess` is the emitted payload; `VecScalarOperands` is the
input hand-off (currently declared inside VecScalarOperandRead; Phase R should
move it here unchanged, since it crosses four nodes). The mask-cursor and
element-publication bundles sit on VecLsu-internal seams and should follow the
same rule.

VectorParams — `vLen`, `eLen`, `maxMembers`, `vecPregSz`, `vecVLSz`,
`ssiQueueEntries`; every width derives from it or from `HasBoomCoreParameters`.
VecTrace — the guarded trace helpers (`trace`, plus `traceElem`).
Binds to BOOM's existing `BrUpdateInfo`, `IsKilledByBranch` and `GetNewBrMask` in
`boom.v4.common`. No new squash or wakeup mechanism is introduced.

Instantiates `VecIdxGen` as `idx`, once, with `isStore` forwarded — and nothing
else. Its counterparties, each to be checked from the other side in Phase R:
  VecScalarOperandRead (`ld_opnd`/`st_opnd`) — the `io.op` hand-off, no ready.
  VecMaskStream (`ld_msk`/`st_msk`, at VecLsu level) — `staged`/`ahead`/
    `skip_log2` in, `step`/`skip` out; its start interface is VecLsu's.
  VecIdxGen (`idx`) — `io.start` Decoupled in, the index interface plus `stall`
    out, `taken` back; the base-plus-offset add is on THIS side, at `xLen`. Its
    descriptor's `idx_eew` is `uop.v_idx_eew` (NOT `v_eew`) and its `offset` is
    ZERO-extended, both checked from that side too.
  VecQueueReservation (`resv`) — lookup lane 0 (load) / 1 (store), plus the
    once-per-OP.v surplus release. Both directions of that seam are PER QUEUE SLOT
    (`resv_resp` a `Vec(2, {base, count})` per lane, `release.used_count` a
    `Vec(2, UInt)`); this module declares the same shape rather than relying on a
    fan-out at VecLsu, reads slot 0, asserts the slots agree, and drives both.
  VecElemQueue (`ld_SSI_ADDR_Q` / `st_SSI_ADDR_Q`) — fill lane 0 at an absolute
    index, with `ready` as the only back-pressure.
  VecDgen — consumes `elem_pub` on the store instance to pair data at the same
    absolute index, and must not re-evaluate the mask.
  VecBeatExpander / VecLoadCoalescingBuffer — consume the emitted nOP.v's
    placement and element cursor, and must not re-evaluate the mask either.
  VecRangeAgen — the complementary class on the same broadcast `io.op`, and the
    owner of the fault-only-first trap-versus-trim policy (`vleff` is unit-stride).
  VecLsu — the VRF read-port binding, the `io.fault` qualification, the
    per-LDQ/STQ-entry pending descriptor table that makes this module's pending
    slot an assertion, and routing `fault_trap` to the exception report and any
    `ff_trim` to `lcb.io.trim` (the LCB, not this module, reaches VlRegFile's
    `W_lsu`, and it does so through `lcb.io.vl_wb`).
  VecSquashUnit — owns queue pointer rollback; this module only drops state.
<|end_dependencies|>

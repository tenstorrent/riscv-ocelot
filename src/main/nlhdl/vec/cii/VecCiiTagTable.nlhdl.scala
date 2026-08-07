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
  VecCiiTagTable — the host's 16-entry per-tag side-table: the ONE place where a
  CII `{tag, op_id, op_offset}` or `{tag, wb_dst_offset}` triple becomes a
  physical register number.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/cii/VecCiiTagTable.scala,
  package boom.v4.vec.generated.cii, group vec_cii.
  depends_on MicroOp, VecBundles, VecTrace.

  Instantiated once, by VecCiiHost as `tags`. Its five clients are all siblings in
  that container: VecCiiIssue writes an entry on grant, VecCiiOperandServer
  resolves Source-Request pulls, VecCiiWriteback resolves result placement,
  VecCiiComplete reads the member-PRN vector on the `last` beat and frees the tag,
  VecCiiFlush sets `killed`. This module is storage and lookup only — it owns no
  channel, pops no FIFO and returns no credit.

  ===> FOUR THINGS A READER MUST NOT GET WRONG.
       (1) THE CII NEVER CARRIES A REGISTER NUMBER, in either direction. The
           coprocessor names operands by an abstract slot plus a member offset and
           results by a tag plus a member offset; this table is the whole of the
           translation and it translates INWARD only. A PRN or a rename bit
           appearing in a channel payload is a protocol break.
       (2) THE TAG IS OPAQUE. Not the `rob_idx`, not an architectural register, not
           a queue index, not derived from anything — a 4-bit index into this
           array, allocated by VecCiiIssue and echoed back by the coprocessor,
           which never inspects what it selects. THIS MODULE DOES NOT CHOOSE THE
           TAG — it records the entry at the index it is handed (part 2).
       (3) `pvs3_grp` AND `stale_pvdest_grp` ARE TWO FIELDS SERVED ON TWO SLOTS,
           equal for read-modify-write arithmetic and different for a masked
           non-RMW op under `vma = 0`, `vslideup`'s prefix and a `vcompress` tail.
           Merging them, or letting one field serve both slots, makes the diverging
           case unrepresentable (see part 2 of the logic section).
       (4) THE STORED DESTINATION GROUP IS WHATEVER GROUP THE COPROCESSOR HALF
           WRITES, WHICH IS `pvtmp` FOR A SEGMENTED STORE AND `pvdest` OTHERWISE.
           The choice is a MUX MADE HERE, ONCE, AT ALLOCATION. Loading the field
           unconditionally from `uop.pvdest` — which an earlier draft of this file
           did — makes cii.i4/i8/i11 (the coprocessor half writes the `pvtmp`
           group) UNSATISFIABLE, because a segmented store has no `pvdest` at all:
           its `dst_rtype` is not `RT_VEC` and the one group rename granted it
           landed in `pvtmp`. Both downstream readers — VecCiiWriteback and
           VecCiiComplete — are written against a SINGLE stored destination field
           and explicitly FORBID re-deriving the choice from `is_shared`, so if it
           is not made here it is made nowhere.

  Governing spec anchors: cii.rst `cii-issue` (the side-table membership list,
  which is its single source of truth), `cii-kill-contract`, `cii-operands`,
  `cii-writeback`; execution.rst `vector-execution` (channel overview) and
  `cii-prn-arn`; midcore.rst `old-vd`. Plan v2 step F3, ground rules 1, 7 and 11.
*/

<|begin_module|>

  <|begin_parameters|>
  Every parameter is a Scala `Int` resolved at elaboration; no width below may be
  a literal. The first three mirror the frozen SV contract in
  `tt_cii_caracal_pkg.svh`, which is authoritative — derive, never duplicate.

  `nTags` — side-table entries, 16 from `CII_N_TAGS`. Not a free choice:
  `CII_N_TAGS = CII_N_ISS_CREDITS`, so a mismatch desynchronises tag identity from
  the Issue channel's credit space.

  `tagBits` — `ciiTagBits` from VectorParams, 4. Require `tagBits ==
  log2Ceil(nTags)` and, at the binding site, `== CII_TAG_W`.

  `numSrcReqLanes` — resolve lanes on the Source-Request side: 4, from
  `CII_NUM_SRC_REQ`, one per lane of that channel and hence one per CII VRF read
  port `R5`-`R8`. Legal 1..4.

  `numWbLanes` — resolve lanes on the Writeback side: fixed 1, from
  `CII_NUM_DST_WB` (the coprocessor cannot present two result beats in a cycle).
  Named, and the port made a `Vec` of one, so a later widening restructures
  nothing; require 1 until the SV contract says otherwise.

  `maxMembers` — members in one register group, fixed 8. Derived `memberBits =
  log2Ceil(maxMembers)` = 3 = `CII_MEMBER_W`, the wire width of `op_offset` and
  `wb_dst_offset`. Remaining field widths come from `HasBoomCoreParameters` and
  VectorParams: `robAddrSz`, `pregSz` (the scalar `pdst`), `vecPregSz` (7 at 96
  vector PRNs) per vector member, `eLen` (64) for the captured scalar value.

  Elaborated only when `usingRVV` is true — a Scala `Boolean` from
  `BoomCoreParams`, NOT a hardware `Bool` and NOT rocket's `usingVector`. The CII
  attach carries the additional `enableVectorArith` sub-flag, applied by
  VecCiiHost; this module is instantiated inside that gate and is ABSENT
  otherwise, not tied off, so a vectors-off build stays bit-identical to
  pre-Caracal BOOM v4.
  <|end_parameters|>

  <|begin_ports|>
  One `core_clk` domain, POSEDGE clock, ACTIVE-HIGH SYNCHRONOUS `core_reset`, both
  implicit via `BoomModule`. Reset initializes the `valid` and `killed` bit
  vectors to zero and nothing else: the payload array is a plain `Reg`, written
  only at allocation, so a reset value would cost `nTags` x entry-width of reset
  fan-out for state never read before it is written.

  ---- Allocation, from VecCiiIssue ----

  `alloc.valid` — Input `Bool`, raised on the cycle `IQ_V_ALU` grants and the
  Issue packet is emitted, so the entry exists from the first cycle the tag is on
  the wire.
  `alloc.uop` — Input `new MicroOp`. Only `rob_idx`, `is_shared`, `uses_stq`,
  `pdst`, `v_emul`, `pvdest`, `pvtmp`, `stale_pvdest`, `pvs1`, `pvs2`, `pvs3`,
  `pvm` are read.
  `alloc.scalar_operands` — Input `UInt(eLen.W)`: the `.vx`/`.vf` operand captured
  BY VALUE from the INT/FP bypass by VecCiiIssue. Stored verbatim; any hardfloat
  unrecoding belongs to the capturing node, not to storage.
  `alloc.tag` — INPUT `UInt(tagBits.W)`, the entry index this allocation occupies,
  CHOSEN BY VecCiiIssue and handed here with the payload. This module performs no
  free-tag select; see part 2.

  `tag_free_mask` — Output `UInt(nTags.W)`, `~tag_valid`: bit `t` set when tag `t`
  is not live. Combinational and FUNCTIONAL (unlike `debug.*` below), consumed by
  VecCiiIssue, which combines it with its own accept-cycle shadow to pick the tag
  and to compute the registered `fu_types` advertise bit.

  // ===> THERE IS NO `alloc.tag` OUTPUT AND NO REGISTERED `tag_avail` HERE, and
  // both absences are load-bearing rather than tidying. VecCiiIssue picks the tag
  // in its ACCEPT cycle and lands the entry in its EMIT cycle, so `tag_valid`
  // does not show a tag taken until a cycle after it was chosen: a back-to-back
  // grant with no shadow term would pick the SAME TAG TWICE, two instructions
  // would share one entry, and one `last` beat would free a tag whose other owner
  // is still running. The shadow (`s1_pending_mask`, the one-hot of the tag in the
  // accept stage) can only live where the accept cycle is visible, and this module
  // structurally never sees it. A registered `tag_avail` computed here would
  // likewise be computed from CURRENT state — the off-by-one that keeps
  // `fu_types` advertised one cycle too long, and with no `ready` line on the
  // Issue channel that is a DROPPED INSTRUCTION, not a stall. The requirement
  // split already reads this way: allocating the tag is VecCiiIssue's (cii.d6),
  // recording the entry is this module's (cii.d8).

  ---- Source-Request resolve, to VecCiiOperandServer ----

  `src_lookup` — `Vec(numSrcReqLanes, ...)`. Input request `{tag:
  UInt(tagBits.W), op_id: UInt(3.W), op_offset: UInt(memberBits.W)}`, taken
  straight off the channel beat; Output response `{prn: UInt(vecPregSz.W),
  read_vrf: Bool, scalar_data: UInt(eLen.W), killed: Bool, rob_idx:
  UInt(robAddrSz.W)}`. Combinational, NO handshake, no back-pressure either way.

  ---- Writeback resolve, to VecCiiWriteback and VecCiiComplete ----

  `wb_lookup` — `Vec(numWbLanes, ...)`. Input `{tag, wb_dst_offset:
  UInt(memberBits.W)}`; Output `{prn: UInt(vecPregSz.W), wr_en: Bool, pvdest_grp:
  Vec(maxMembers, UInt(vecPregSz.W)), members: UInt((log2Ceil(maxMembers)+1).W),
  rob_idx, pdst: UInt(pregSz.W), is_shared: Bool, killed: Bool}`. One port with
  two readers, not two narrower ports: both read the same beat's tag in the same
  cycle, and a second 16-to-1 entry mux would double this module's dominant cost
  for nothing.

  `free` — Input `Valid(UInt(tagBits.W))`. VecCiiComplete frees the tag on the
  beat marked `last`, for a killed tag exactly as for a live one.

  ---- Kill, from VecCiiFlush ----

  `kill_all` — Input `Bool`. One bit, no tag and no age comparator: flushes fire
  at the ROB head and `rob.io.flush.bits` carries no `rob_idx` to compare against,
  so kill-all is both correct and cheaper than any qualified form.

  `debug.valid` / `debug.killed` — Outputs, `nTags` bits each, raw occupancy for
  waveform inspection. Nothing functional may read them.

  There is deliberately NO port for any of the following, and a reviewer should
  reject each: a `ready`/`valid` handshake or back-pressure on a lookup; a
  `brupdate` input or branch-mask field (branch mispredicts cannot reach the CII —
  `is_br`/`is_jalr` set `starts_unsafe`, so the PNR never sweeps past an
  unresolved branch); the flush `bits` payload; a `vl`/`vtype`/`vstart`/`vxrm`/
  `frm` write port; a per-tag beat counter; any input carrying a register number
  FROM the coprocessor; a free-tag select or an `alloc.tag` OUTPUT; and a `busy`,
  `ready` or registered availability output of ANY kind. `tag_free_mask` is raw
  next-cycle-visible occupancy, not an advertise bit: the module that advertises
  is the module that holds the accept-cycle shadow.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. State: one direct-mapped array, indexed by the tag ----

  //@req-spec-cii.b11
  The tag INDEXES this table directly: `nTags` entries, `tag` is the array index,
  and there is no CAM, no tag-to-entry mapping and no content-addressed search
  anywhere here — a lookup is `entries(req.tag)`. Tag width and entry count are
  the same fact stated twice, which is why the SV package defines `CII_TAG_W =
  log2Ceil(CII_N_TAGS)`.

  Three pieces of state: `entries`, a `Reg(Vec(nTags, new VecCiiTagEntry))` with
  no reset; `tag_valid`, `RegInit(0.U(nTags.W))`, allocated-and-not-yet-freed; and
  `tag_killed`, `RegInit(0.U(nTags.W))` (part 6). `killed` is its own bit vector
  rather than a payload field because it is the only part of an entry written by
  something other than allocation, and separating it makes "the flush touches this
  vector and nothing else" structural rather than a review promise.

  //@req-spec-cii.d16
  //@req-spec-cii.d17
  //@req-spec-cii.d18
  //@req-spec-cii.d19
  //@req-spec-cii.d20
  //@req-spec-cii.d21
  //@req-spec-cii.d22
  //@req-spec-cii.d23
  `VecCiiTagEntry`'s field list is exactly the membership list of cii.rst
  `cii-issue` — that list is the single source of truth and this bundle is its
  Chisel spelling, field for field, name for name, each retained because a host
  job needs it after issue:
    `rob_idx` `UInt(robAddrSz.W)` — completes the ROB entry on `last`.
    `is_shared` `Bool` — marks one half of a shared (segmented) instruction, for
      the ROB's "other half pending" flag.
    `pvdest_grp` `Vec(maxMembers, UInt(vecPregSz.W))` and `pvdest_grp_mask`
      `UInt(maxMembers.W)` — resolve `wb_dst_offset` to a physical destination
      member and its write enable.
    `pvs1_grp`, `pvs2_grp`, `pvs3_grp` `Vec(maxMembers, UInt(vecPregSz.W))` and
      `pvm` `UInt(vecPregSz.W)` — resolve `VS1`/`VS2`/`VS3`/`VM` pulls to a
      physical member. `pvm` is ONE register and never a group.
    `stale_pvdest_grp` `Vec(maxMembers, UInt(vecPregSz.W))` — resolves the
      `STALE_VD` pull.
    `pdst` `UInt(pregSz.W)` — completes a scalar result (`vmv.x.s`, `vcpop.m`,
      `vfirst.m`, `vfmv.f.s`) to the renamed physical scalar destination.
    `scalar_operands` `UInt(eLen.W)` — serves the `SCALAR` slot by value, no VRF
      read. One `eLen` register suffices because an RVV arithmetic op encodes at
      most one scalar source (`rs1` as `.vx` or `.vf`, never both); a custom
      instruction needing two widens this to a `Vec` indexed by the `op_offset` the
      slot already carries.

  // The plan's side-table sketch also lists a "dest-group size". It is PopCount
  // of pvdest_grp_mask, derived where needed and NOT a second field: two fields
  // expressing one fact are two fields that can disagree, and the mask is what
  // the writeback path needs per beat anyway.

  ---- 2. Allocation: record the entry at the index handed in ----

  //@req-spec-cii.d8
  //@req-spec-cii.f11
  On `alloc.valid`, write `entries(alloc.tag)`, set that bit of `tag_valid`, and
  CLEAR that bit of `tag_killed`. `alloc.tag` ARRIVES AS AN INPUT — the free-tag
  select is VecCiiIssue's (cii.d6), for the accept-cycle-shadow reason spelled out
  in the ports section; this module contains no `PriorityEncoder` over `~tag_valid`
  and exports `tag_free_mask` so that the select can be built where the shadow is.
  The array write is the whole of "records a tag side-table entry, used later to
  service operand pulls and to place the result": every later beat for this tag
  reads only what is written here. The mapping is stated explicitly because the
  side-table spells its fields with a `_grp` suffix and `MicroOp` does not, so a
  transposition would be silent corruption rather than a compile error — `rob_idx
  := uop.rob_idx`, `is_shared := uop.is_shared`, `pdst := uop.pdst`, `pvs1_grp :=
  uop.pvs1`, `pvs2_grp := uop.pvs2`, `pvs3_grp := uop.pvs3`, `pvm := uop.pvm`,
  `stale_pvdest_grp := uop.stale_pvdest`, `scalar_operands :=
  alloc.scalar_operands`, and `pvdest_grp_mask := (1.U << uop.v_emul) - 1.U` (the
  low `v_emul` members enabled).

  ===> THE ONE FIELD THAT IS A MUX, NOT A COPY, IS THE DESTINATION GROUP:

           cop_writes_pvtmp = uop.is_shared && uop.uses_stq
           pvdest_grp      := Mux(cop_writes_pvtmp, uop.pvtmp, uop.pvdest)

       `pvdest_grp` NAMES THE GROUP THE COPROCESSOR HALF WRITES, which for a
       SEGMENTED STORE is `pvtmp` and for everything else — including a segmented
       LOAD's coprocessor half, which transposes `pvtmp` INTO `pvdest` — is
       `pvdest`. The predicate is the "tmp-only" case VecRenameSpace already names:
       a segmented store's `dst_rtype` is not `RT_VEC`, so rename granted it ONE
       group and routed it into `uop.pvtmp`, and `uop.pvdest` holds nothing. Both
       spellings of the predicate are the same set — `is_shared && uses_stq` uses
       only baseline `MicroOp` fields and needs no `RT_*` encoding here, and
       dispatch presents the SAME uop to `iq_v_alu` and `iq_v_store` (VecIssueUnit
       part 8), so `uses_stq` is set on the granted coprocessor half.

  // ===> WHY THE MUX IS HERE AND NOWHERE ELSE. Without it, cii.i4/i8/i11 — the
  // coprocessor half writes the `pvtmp` group — are UNSATISFIABLE, because the
  // only destination field a beat can resolve against would hold a group rename
  // never allocated. It cannot be fixed downstream either: VecCiiWriteback writes
  // `wb_lookup.resp.prn` and holds no uop, VecCiiComplete announces
  // `wb_lookup.resp.pvdest_grp` and holds no uop, and BOTH of them explicitly
  // forbid re-deriving the choice from `is_shared`/`is_store` — two places
  // deciding which group a beat lands in is exactly how a segmented store
  // silently writes the load-side group. One mux, one cycle, one place.
  // The entry field keeps the name `pvdest_grp`: it describes the common case, and
  // renaming it would touch four sibling specs to say nothing new. What the name
  // is NOT is a licence to load it unconditionally from `uop.pvdest`.

  ===> `pvs3_grp` AND `stale_pvdest_grp` ARE CAPTURED FROM TWO DIFFERENT `MicroOp`
       FIELDS INTO TWO DIFFERENT ENTRY FIELDS, and this module cannot tell the
       coinciding case from the diverging one — nor does it need to. Do not add a
       comparator, do not fold them, do not add an "is RMW" bit.

  // Members at or beyond v_emul are captured verbatim rather than zeroed: PRN 0 is
  // a real allocatable vector PRN, so zeroing would make an out-of-range offset
  // read a legitimate register belonging to someone else — harder to spot in a
  // waveform than a stale member.

  `alloc.valid` on a tag whose `tag_valid` bit is already set must be impossible,
  because VecCiiIssue's registered `fu_types` gate consumes `tag_free_mask` minus
  its own accept-cycle shadow; assert it rather than trust it. Note that tag
  availability is TIGHTER than the issue credit even though both are 16: the credit
  returns when the coprocessor pops the Issue FIFO, whereas a tag is held until its
  `last` writeback beat. Gating `fu_types` on the credit alone over-advertises, and
  the CII channels have no `ready` line to catch it.

  ---- 3. Source-Request resolve ----

  //@req-spec-cii.d20
  //@req-spec-cii.d21
  //@req-spec-cii.d23
  //@req-spec-cii.f5
  //@req-spec-cii.f6
  //@req-spec-cii.f7
  //@req-spec-cii.f8
  //@req-spec-cii.f10
  Per lane, select `e = entries(req.tag)`, then the response by `op_id` against
  the `cii_caracal_srcid_e` encoding and the member by `op_offset`. The five
  per-slot resolutions below ARE `spec-cii.f5`-`f8` and `f10` — `f5`/`f6`/`f7` the
  `VS1`/`VS2`/`VS3` lines, `f8` the `VM` line, `f10` the `STALE_VD` line:
    `NONE` (0) — reserved, "no source needed": `read_vrf` false, data don't-care.
    `VS1` (1) / `VS2` (2) / `VS3` (3) — `prn := e.pvs{1,2,3}_grp(op_offset)`,
      `read_vrf` true.
    `VM` (4) — `prn := e.pvm`, `read_vrf` true, `op_offset` IGNORED.
    `SCALAR` (5) — `scalar_data := e.scalar_operands`, `read_vrf` FALSE: the value
      was captured at issue, so no VRF and no scalar regfile read happens at pull
      time.
    `STALE_VD` (6) — `prn := e.stale_pvdest_grp(op_offset)`, `read_vrf` true.
  Every lane also drives `killed := tag_killed(req.tag)` and `rob_idx :=
  e.rob_idx`. `killed` is what lets VecCiiOperandServer suppress the VRF read
  while still returning the mandatory don't-care Src-Data beat — the read ports
  staying free is the real saving of the drain contract.

  //@req-spec-cii.f38
  //@req-spec-cii.f11
  The request carries `{tag, op_id, op_offset}` — a slot and a member index — with
  no register number, architectural or physical, and this module is the only place
  that pair becomes a PRN. The reverse never happens: nothing here drives a PRN or
  a rename bit toward an output that reaches a CII channel. `VS3` and `STALE_VD`
  are served from their own fields with no instruction-dependent reinterpretation,
  so the coprocessor may pull one (RMW) or both (diverging) and get the right group
  either way. Two lanes naming the same member in one cycle is not this module's
  concern: each is answered independently and nothing coalesces.

  // ===> TRACEABILITY WRINKLE, NOW CLOSED. Phase R took the first of the two
  // options this note used to offer: decision D12 case 3 (allocation wrong,
  // requirement fine) RE-ALLOCATED `spec-cii.f5`-`f8` and `f10` from
  // VecCiiOperandServer to this node, because the per-slot resolutions above are
  // where they are emitted. hierarchy.yaml now allocates the five IDs here and
  // they are tagged here; VecCiiOperandServer keeps the prose describing the
  // encoding, with its tags removed and a pointer to this site.
  //
  // What did NOT change, and is why the split existed: `opnd` still owns the
  // OUTCOME of each slot — the `R5`-`R8` address drive, the `SCALAR` no-read path,
  // the returned beat, the request order and the killed drain — and those are
  // carried by its own IDs (`f4`, `f9`, `f12`, `vrf.j13`/`j14`), which stayed put.
  // The narrow `src_lookup` response still exists so the 16-to-1 entry mux is
  // emitted once, here, instead of crossing four ~376-bit entries to be recomputed
  // at the same logic depth on the far side.

  // Only the DESTINATION offset is bound-checked (part 4). A source offset is
  // served verbatim, because the membership list stores no per-source member count
  // and deliberately so: widening and narrowing make source and destination counts
  // differ, and the coprocessor owns the member walk. An out-of-range source
  // offset is a VPU decode bug, and the part-8 trace line is what localises it
  // here rather than at the VRF.

  ---- 4. Writeback resolve ----

  //@req-spec-cii.d19
  //@req-spec-cii.d22
  Select `e = entries(req.tag)`, then `prn := e.pvdest_grp(req.wb_dst_offset)` and
  `wr_en := e.pvdest_grp_mask(req.wb_dst_offset)`. Those two fields answer both
  halves of what a beat asks — which physical member, and whether it is a member at
  all — so a beat naming an offset outside the destination group writes nothing
  instead of corrupting a PRN belonging to another instruction. Export the WHOLE
  `pvdest_grp` vector plus `members = PopCount(e.pvdest_grp_mask)` alongside,
  because VecCiiComplete builds one `VecGroupDone` from the full member-PRN vector,
  not from the member this beat placed. Export `pdst` for a scalar-destination op,
  completed to the renamed physical scalar register recorded at issue, and
  `is_shared` for the ROB's "other half pending" flag.

  // Both readers resolve against the ONE destination group part 2 chose, so a
  // segmented store's beats land in `pvtmp` and its group-done names `pvtmp`'s
  // members with no case analysis on either reader's side. `is_shared` is exported
  // for the ROB flag, for trace and for assertions ONLY — neither reader may use
  // it to re-select the group, which was already decided at allocation.

  // This module does not interpret `wb_status.dst_kind` and stores no writeback
  // routing — routing arrives per beat, and VecCiiWriteback selects VRF W2 / INT
  // RF / FP RF from it. The table supplies both candidate destinations.

  ---- 5. Free, and the no-recycling rule ----

  //@req-spec-cii.e23
  //@req-spec-cii.e24
  `free.valid` clears `tag_valid(free.bits)` and nothing else — payload and
  `killed` are left exactly as they are. The tag becomes reallocatable one cycle
  later, when VecCiiIssue's select sees the bit cleared in the exported
  `tag_free_mask`: there is NO same-cycle free-to-allocate bypass, and this module
  supplies none. That costs at most one cycle of availability
  at full occupancy and buys a lifetime invariant checkable by inspection — the
  entry a beat reads cannot have been overwritten by an allocation made in the same
  cycle its last beat arrived.

  ===> A KILLED TAG IS NOT RECYCLED DURING THE DRAIN. Its lifetime is IDENTICAL to
       a live tag's — allocated at grant, freed on the `last` writeback beat, held
       in between however many flushes intervene. Only the EFFECTS differ, and they
       are suppressed downstream by VecCiiWriteback and VecCiiComplete, not here.
       Freeing on the flush would let the tag be reallocated while the coprocessor
       is still emitting beats for it, and those beats would resolve against the NEW
       occupant's `pvdest_grp` and write a PRN owned by a surviving instruction.
       This is also exactly why drain-and-discard is safe here and unsafe in the
       vector LSU, where destination PRNs return to the free list on the flush and
       are promptly reallocated (see VecSquashUnit). Do not generalise the contract
       in that direction.

  ---- 6. Kill: what survives a flush ----

  //@req-spec-cii.e22
  On `kill_all`, `tag_killed := tag_killed | tag_valid`: every LIVE entry gains the
  bit, free entries do not, and the payload array is not written at all. That is
  the required "the `killed` bit and enough side-table state to route the drain must
  survive the flush" — and here "enough state" is the whole entry, because a drain
  needs the same fields a live tag does: `rob_idx` and `pvdest_grp` to identify
  what is being suppressed, and every source group to answer the pulls still
  coming.

  `tag_killed` is CLEARED ONLY BY ALLOCATION (part 2), so setting it is idempotent:
  a second flush mid-drain ORs the same bits again and changes nothing for an entry
  that already carries it. No re-initialization on the second flush, no per-tag
  flush counter, no drain restart.

  // ===> DO NOT CLEAR tag_valid ON kill_all, and add no flush term to any
  // next-state expression except tag_killed. The BOOM idiom for a flush is to
  // invalidate, and here invalidating IS the bug: it frees tags whose beats are
  // still in flight (part 5) and discards the routing state the drain needs.

  ---- 7. No buffering: what this table is not ----

  //@req-spec-cii.b8
  //@req-spec-cii.b11
  The relay between host and coprocessor is pure latency pipes with no buffering,
  and each channel's RECEIVER owns the only FIFO. This table must stay consistent
  with that: it is a random-access side-table of per-tag CONTEXT, indexed by the
  tag — not a packet buffer, not a reorder queue, not a shadow copy of any channel.
  Concretely: every lookup in parts 3 and 4 is combinational, ZERO cycles, adding
  no pipeline stage to any channel; no beat, request or response is stored here;
  and with nothing stored there is nothing to back up, which is why no lookup port
  has a `ready`. That is a requirement, not a convenience — on Source-Data the host
  is the SENDER and holds no credit, so a host-side stall has no legal expression
  in the protocol. The ordering FIFO covering the registered one-cycle VRF read
  belongs to VecCiiOperandServer, which owns request order; here it would make this
  module stateful per beat and split that reasoning across two modules.

  ---- 8. Trace and assertions ----

  There are NO unit tests in this project, so guarded tracing is the debug surface.
  One `VecTrace` line per key event, gated on the `vecTrace` plusarg and `!reset`,
  off by default: "alloc" with tag, `rob_idx`, `v_emul` and the `pvdest_grp` member
  list; "src" per active lane with tag, `op_id`, `op_offset`, resolved `prn` and
  `killed`; "wb" with tag, `wb_dst_offset`, resolved `prn`, `wr_en`; "kill" with
  the live-tag bitmap; "free" with the tag. Emit-only: no register and no counter
  that functional logic reads.

  // The alloc line has the granted MicroOp in hand and uses VecTrace.trace plus
  // the traceTag wrapper directly. Every later line has only the STORED rob_idx,
  // because a tag outlives the uop that created it — so those lines need a
  // rob_idx-keyed entry point, the same gap traceDecode fills at the other end of
  // the pipeline. Fabricating a uop to satisfy the helper's signature would put an
  // invented rob_idx in the trace, which is worse than no line.

  Assertions, in the `usingRVV` build only: `alloc.valid` implies
  `!tag_valid(alloc.tag)` — the index handed in is genuinely free, which is the
  check that catches a broken select on the other side of the seam; every lookup,
  source or writeback, hits a tag whose `tag_valid` bit is set (a beat for an
  unallocated tag is a protocol break, and resolving it silently against stale
  payload is how it reaches the VRF); `free.valid` implies `tag_valid(free.bits)`,
  so no double free; `alloc.uop.v_emul` is within 1..maxMembers; and
  `alloc.uop.is_vec` is set on every allocation.

  ===> THE KILL-WINDOW ASSERTION IS AN EVENTUAL PROPERTY, NOT AN EXCLUSION. The
       obligation is "A TAG ALLOCATED DURING A KILL WINDOW CARRIES `killed` BY THE
       FOLLOWING CYCLE" — i.e. `alloc.valid && kill_all` implies
       `RegNext(tag_killed(alloc.tag))` — and NOT "`kill_all` never coincides with
       `alloc.valid`", which an earlier draft of this file asserted and which fires
       on a real, benign case: `IQ_V_ALU` gates on `flush_pipeline =
       RegNext(rob.io.flush.valid)`, so a grant CAN fire in the flush cycle itself
       and VecCiiIssue will land a wrong-path entry whose `killed` bit its own
       allocation write has just cleared. The kill window spans two cycles
       (VecCiiFlush ORs `rob_flush` with `rob_flush_kill`), so the second cycle
       re-sets the bit on a now-live entry and the drain proceeds normally.
       RELAXED, NOT DELETED. Deleting it turns the hole into a wrong-path VRF write
       plus a `clr_rob` for a dead ROB entry, with nothing to report either.

  ---- 9. What is NOT stored here ----

  // NOT SPECIFIED, AND NOT TO BE ADDED BY A GENERATOR:
  //  - `vl`, `vconfig`/`vtype`, `vstart`, `vxrm`, `frm`. cii.rst `cii-issue` states
  //    explicitly that these are NOT side-table fields: they ride the
  //    per-instruction Issue packet, the coprocessor applies vta/vma and rounding
  //    itself, and the host never re-reads them. Storing them would add ~24 bits x
  //    16 entries of flops for a value with no reader.
  //  - a writeback routing field: `dst_kind` arrives per beat.
  //  - a SECOND destination-group field. `uop.pvtmp` is read at allocation, but it
  //    is muxed INTO the single `pvdest_grp` field (part 2), never stored beside
  //    it. Two destination fields would need a per-beat selector to choose
  //    between them, and that selector is exactly the re-derivation from
  //    `is_shared` that VecCiiWriteback and VecCiiComplete forbid. Where the
  //    transpose half needs `pvtmp` as a SOURCE it arrives as one of the pvs*
  //    slots at issue, on the ordinary group-done machinery, and the binding from
  //    the abstract rendezvous to real PRNs is the MicroOp field itself.
  //  - a beat counter or expected-member count: `last` is the only completion
  //    signal and the host must never infer completion by counting beats —
  //    widening and narrowing ops emit a member count that differs from the source
  //    EMUL, so any count derived here is wrong for exactly those cases.
  //  - `v_eew` or any other access-descriptor field.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Zero-cycle lookup, single-cycle state update, no pipeline stage. Parts 3 and 4 must
answer combinationally from the requested tag and the current array, because the
operand server pops a Source-Request and starts its registered VRF read in the same
cycle, and the writeback path places a beat in the cycle it pops it. Registering a
lookup would add a cycle to every pull and every placement and force a deeper
ordering FIFO in the operand server.

Storage: one entry is `robAddrSz + 2 + 5 x maxMembers x vecPregSz + maxMembers +
vecPregSz + pregSz + eLen` bits — about 376 at the defaults (7 + 2 + 280 + 8 + 7 +
7 + 64) — so roughly 6 kbit of flops over 16 entries. That is the budget argument
behind part 9 of the logic: every entry field costs 16 copies.

Dominant timing cost is entry read multiplexing: `numSrcReqLanes + numWbLanes` = 5
independent 16-to-1 selects over a ~376-bit entry per cycle. Structure each lane as
SELECT THE ENTRY BY TAG FIRST, then the slot by `op_id`, then the member by the
offset — 16-to-1, then 6-to-1, then 8-to-1 over `vecPregSz` bits. A generator must
NOT invert that order and replicate the slot/member mux inside all 16 entries: that
is 16x the logic for the same function. If timing demands more, narrowing a lane by
selecting `{op_id, op_offset}` ahead of the tag mux is the escape hatch — a
restructuring, not a spec change, and not specified here.

Exactly one array write port (allocation, one entry per cycle), addressed by the
INPUT `alloc.tag`. Kill and free write only the two `nTags`-wide bit vectors, so a
flush is an OR over 16 bits and costs the same at any occupancy — which is what
makes kill-all cheaper than any age-qualified alternative. The destination-group
mux of logic part 2 is `maxMembers x vecPregSz` = 56 bits of 2-to-1 select on the
write path only, off every read path and out of every lookup's timing cone.

No throughput target of its own: the table must never be why a grant, a pull or a
writeback beat stalls, hence no handshake anywhere. The only rate it bounds is issue
— 16 concurrent CII instructions — and it bounds it by EXPORTING `tag_free_mask`,
never by back-pressure and never by an availability flop of its own: the registered
advertise bit belongs to VecCiiIssue, which is the only module that can compute it
from NEXT state (see the ports section).
<|end_perf|>

<|begin_dependencies|>
MicroOp — `alloc.uop` supplies `rob_idx`, `is_shared`, `uses_stq`, `pdst`,
`v_emul` and the renamed groups `pvdest`, `pvtmp`, `stale_pvdest`, `pvs1`, `pvs2`,
`pvs3`, `pvm`. The `pvs3`-versus-`stale_pvdest` separation this module depends on is
that bundle's own invariant; merged there, this table could not serve two slots.
`pvtmp` and `uses_stq` are read for ONE purpose — the destination-group mux of
logic part 2 — and `uses_stq` is the baseline field deliberately chosen over
`dst_rtype =/= RT_VEC`, which names the identical set but would drag a
ScalarOpConstants edge into a node hierarchy.yaml grants none.

VecBundles — the CII channel payloads (`CiiSrcReq`, `CiiWriteback`) whose field
widths the lookup request ports match beat for beat. The entry and lookup bundles
themselves — `VecCiiTagEntry` and the source/writeback request-response pairs — are
declared in THIS file, following the `VecBusyResp` precedent in VecBusyTable: they
cross only the boundaries to this module's own siblings inside VecCiiHost.
hierarchy.yaml's VecBundles comment lists `VecCiiTagEntry` among that package's
declarations and the authored VecBundles spec does not declare it; that discrepancy
is reported rather than resolved by declaring the type twice.

VecTrace — the guarded helpers of part 8, plus the rob_idx-keyed entry point noted
there as missing.

VectorParams — `ciiTagBits`, `maxMembers`, `vecPregSz`, and through them the
mirrors of `CII_TAG_W`, `CII_MAX_MEMBERS` and `CII_N_TAGS` in
`tt_cii_caracal_pkg.svh`, DERIVED from the SV contract and never redeclared here.

VecCiiIssue (`iss`) — the allocating sibling, and the seam that changed. It owns
the free-tag select (cii.d6) and hands this module `{alloc.valid, alloc.tag,
alloc.uop, alloc.scalar_operands}`; this module owns the record (cii.d8) and hands
back `tag_free_mask`. // REPORTED, NOT RESOLVED: VecCiiIssue's own file spells its
side of that seam as `tag_alloc — Output(Valid(new VecCiiTagEntry))` "carrying the
allocated tag and the whole entry content", i.e. with the ENTRY pre-built on its
side. This file keeps the uop-shaped payload, because the destination-group mux
(logic part 2) is required to be HERE — moving it into `iss` puts it back in the
node whose reject list has no place for it and leaves this module's cii.d8 with
nothing to do but copy. The two spellings must be reconciled in one direction or
the other before generation; doing it by pre-building the entry means moving the
mux, which is the thing A3 forbids.

Instantiates nothing; its only parent is VecCiiHost, once, as `tags`. A change to
the entry field list is a change to all five client siblings at once.
<|end_dependencies|>

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
  VecRenameSpace — ONE parameterized rename space: map table + free list + busy
  table + the in-bundle prefix bypass, wired into a single rename cycle.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/rename/VecRenameSpace.scala,
  package boom.v4.vec.generated.rename, group vec_rename.
  depends_on MicroOp, VecBundles, VectorParams, VecTrace.
  instantiates VecMapTable as `maptable`, VecFreeList as `freelist`,
  VecBusyTable as `busytable` — one of each, per instance of this module.

  ===> ONE DEFINITION, TWO INSTANCES. VecPipeline instantiates this module twice:
       `vec_rename` (numArchRegs 32, maxGroupSize 8, numPhysRegs
       numVecPhysRegisters, freeDiscipline "stale_group", wakeupKind
       "group_done") and `vl_rename` (numArchRegs 1, maxGroupSize 1, numPhysRegs
       numVlPhysRegisters, freeDiscipline "committed_ptr", wakeupKind
       "ready_bit"). Baseline BOOM already does exactly this for INT and FP with
       one `RenameStage` class; `addvector` instead hand-wrote two spaces and its
       `VlRename` reached 405 lines reimplementing map + free + busy + wakeup +
       commit for a space with ONE architectural register. Every VL construct in
       this file is a `maxGroupSize == 1` / `numArchRegs == 1` degeneration of the
       vector construct beside it. DO NOT ADD A SECOND MODULE FOR VL.

  ===> THE LOCKSTEP CONTRACT — this is the M1 free-list double-free, and this
       module is where it happened. Allocation is driven from `ren2_uops` and
       `dis_fire`: the REGISTERED output of the scalar `RenameStage`'s ren1->ren2
       pipeline, and the dispatch fire of that same registered bundle. It must
       NEVER be driven combinationally from `dec_uops`. Doing so runs this module
       one cycle AHEAD of the scalar rename, so at dispatch the uop's vector
       fields describe the NEXT cycle's (usually bubble) uop, and two ops end up
       freeing the same PRN — in M1, PRN 0. The seam bundle `vec_pipeline_io`
       names these ports `ren2_uops`/`dis_fire` precisely so that connecting
       `dec_uops` is visibly wrong at the connection site.

  This module is the vector analogue of `class RenameStage` in
  src/main/scala/v4/exu/rename/rename-stage.scala (which stays untouched and keeps
  serving INT and FP) and deliberately keeps its structure and names —
  `ren2_alloc_reqs`, `ren2_alloc_fire`, `ren2_br_tags`, `map_reqs`, `remap_reqs`,
  `com_remap_reqs`, `com_valids`, `BypassAllocations`. Read that file alongside
  this spec: everything below is either that file one member wider, or a named
  deletion. The one baseline output NOT reproduced is the per-lane `ren_stalls`
  vector — this space stalls whole bundles, so it exports the single `alloc_ok`.

  Governing spec anchors: midcore.rst `midcore-rename`, `rename-stage`, `rmt`,
  `cii-shared-mapping`, `vl-vtype-rename`, `snapshots`; frontend.rst
  `vset-dual-dest`, `vl-delivery`.

<|begin_module|>

  <|begin_parameters|>
  All constructor parameters are Scala `Int`/`String`/`Boolean` resolved at
  elaboration. The whole module is elaborated only when `usingRVV` is true — a
  Scala `Boolean` from `BoomCoreParams`, NOT a hardware `Bool` and NOT rocket's
  `usingVector`. With vectors off neither instance exists, so a non-vector build
  emits RTL bit-identical to pre-Caracal BOOM v4. No width below is a literal.

  `plWidth` — rename lanes. Default `coreWidth` (3), legal 1..4. Every per-lane
  port is this wide, and the free list's `allocWidth` derives from it.

  `numArchRegs` — architectural registers in this space: 32 (fixed by RVV, not a
  knob) for `vec_rename`, 1 for `vl_rename`. `maxGroupSize` — the most registers
  one instruction renames atomically: `maxMembers` (8) and 1 respectively.
  `numPhysRegs` — `numVecPhysRegisters` (96) or `numVlPhysRegisters` (64), from
  VectorParams. Derived `pregSz = log2Ceil(numPhysRegs)` (7 and 6) and
  `emulSz = log2Ceil(maxGroupSize) + 1`.

  `numWbPorts` — completion ports: `numVecWbPorts` (3: LCB, CII completion,
  VecGroupCopy) for the vector instance, the VL writeback port count for the VL
  one. Passed straight to the busy table; it is also the width every vector issue
  slot's per-member matcher is sized from, so both must come from this one name.

  //@req-spec-rename.h5
  `freeDiscipline` — "stale_group" or "committed_ptr". `wakeupKind` —
  "group_done" or "ready_bit". Both are EXPLICIT named parameters, `require`d to
  be one of their two spellings, and neither is derived from `maxGroupSize`. They
  are what let one definition serve both spaces: the VL rename is a
  one-architectural-register rename using the same map table, free list and busy
  table the scalar rename already provides, just one ARN wide — the only genuine
  differences are which PRN commit releases (the whole `stale_pvdest` group,
  versus the outgoing committed pointer, since no `stale_pvl` exists) and what a
  wakeup carries (a member-PRN vector, versus a plain readiness bit whose value is
  read at execute). Those two decisions are independent of group size and of each
  other; deriving either from `maxGroupSize == 1` would couple them silently.

  `hasRenameWrite` — Boolean, default false, true for `vl_rename` only: gates the
  rename-cycle register-file write port (the born-ready `vsetivli` path) and its
  two input ports. `exportMemberRdy` — Boolean, default false, true for
  `vec_rename` only: gates the per-member source-readiness export to the issue
  stage, and is PASSED DOWN TO THE BUSY TABLE UNDER THE SAME NAME, where it gates
  both the pre-reduction `member_busy_resps` tap and the `stale_pvdest` read that
  feeds its fifth group (D6). Both are presence gates in exactly
  the sense of VecMapTable's `exportComStale` — they keep dead wires out of the
  instance with no use for them — and neither may be derived from `maxGroupSize`.

  `bypass` — passed to VecMapTable, default true for both instances.
  `exportComStale` is passed to VecMapTable as `freeDiscipline == "committed_ptr"`;
  that ONE derivation is legitimate, because the committed-pointer free rule is the
  only consumer of the committed table's displaced mapping.

  Requires: `maxGroupSize <= numArchRegs`; both discipline strings legal;
  `hasRenameWrite` implies `maxGroupSize == 1` (a rename-cycle write of a whole
  group is not defined and no register file offers it); `exportMemberRdy` implies
  `maxGroupSize > 1` (a one-member group's per-member vector carries no
  information the aggregate does not).
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit `clock`/`reset` through `BoomModule`:
  single `core_clk` domain, POSEDGE clock, ACTIVE-HIGH SYNCHRONOUS `core_reset`.
  No second clock domain and no asynchronous reset. This module holds NO state of
  its own — every register in this space lives in one of the three children.

  ---- The lockstep inputs ----

  `ren2_uops` — Input(Vec(plWidth, new MicroOp)), the scalar `RenameStage`'s
  `io.ren2_uops`: REGISTERED, already branch-masked, already carrying its scalar
  PRNs. `ren2_mask` — Input(Vec(plWidth, Bool())), that stage's `io.ren2_mask`.
  `dis_fire` — Input(Vec(plWidth, Bool())), BoomCore's `dis_fire`. These three
  names are the lockstep contract of the file header; there is deliberately no
  `dec_uops` and no `dec_fire` port on this module, and no `kill` port either
  (there are no pipeline registers here to kill).

  `ren2_vl_imm` — Input(Vec(plWidth, UInt(vecVLSz.W))) and `ren2_vl_imm_valid` —
  Input(Vec(plWidth, Bool())), only when `hasRenameWrite`: VConfigUnit's
  `dec_vl_imm` and its `frontend_only`-qualified valid, REGISTERED by VecPipeline
  into the same ren2 stage as the uop it belongs to. A shadow pipeline for this
  pair that ran a cycle ahead of `ren2_uops` would be the free-list double-free in
  a second place, so VecPipeline must register it off the same enable.

  `brupdate` — Input(new BrUpdateInfo), `rollback` — Input(Bool()) (the ROB's
  exception/flush pulse), `com_valids` — Input(Vec(retireWidth, Bool())) and
  `com_uops` — Input(Vec(retireWidth, new MicroOp)) from the ROB commit ports.
  `wakeups` — Input(Vec(numWbPorts, Valid(...))): `VecGroupDone` from VecBundles
  under "group_done", `UInt(pregSz.W)` under "ready_bit". Forwarded unmodified to
  the busy table; this module reads no field of them.

  ---- The outputs ----

  //@req-spec-rename.a7
  //@req-spec-rename.b1
  `ren2_uops_out` — Output(Vec(plWidth, new MicroOp)): the input uops with THIS
  SPACE'S renamed fields written and every other field passed through untouched.
  The whole dispatch group, scalar and vector halves alike, leaves rename in this
  one cycle: this output is combinational from `ren2_uops`, so there is no bubble
  between the halves and nothing downstream has to realign two differently-timed
  bundles. The two instances are CHAINED — `vec_rename`'s `ren2_uops_out` is
  `vl_rename`'s `ren2_uops`, and `vl_rename`'s output is the dispatch bundle. That
  chain IS the join of the two spaces' responses, and it is safer than a
  field-by-field join in the parent (baseline `core.scala:694-725`, which has to
  re-derive which space owns each field): each instance writes only what it
  renamed. `br_mask` is NOT re-derived and `GetNewUopAndBrMask` is NOT re-applied —
  the scalar RenameStage already did it and owns that field.

  //@req-spec-rename.e16
  `alloc_ok` — Output(Bool()), ONE bit for the WHOLE bundle: every requesting lane
  of this space can have its full demand this cycle. VecPipeline ANDs both
  instances' `alloc_ok` into `vec_pipeline_io.dis_ready`, and BoomCore ORs the
  inverse into `ren_stalls(w)` for EVERY lane `w`, not only the requesting ones.
  That broadcast is what makes a failed allocation stall the whole dispatch group.
  There is deliberately no per-lane grant, no per-lane stall mask and no partial
  grant encoding anywhere in this module's interface.

  `member_rdy` — Output(Vec(plWidth, new VecMemberRdy)), only when
  `exportMemberRdy`. **`VecMemberRdy` is declared in `VecBundles`, NOT in this
  file — bind to it and do not declare a local copy.** It has `vs1_rdy`,
  `vs2_rdy`, `vs3_rdy`, `vtmp_rdy` and — per decision D6 — `vold_rdy`, each
  `Vec(maxMembers, Bool())`, plus `vm_rdy`, a single `Bool`, and it takes no
  parameter. Sense is READY, not busy, to match VecGroupReady's `in_member_rdy`
  input exactly.

  ===> CORRECTED 2026-08-10. This paragraph used to say the bundle was declared
  IN THIS FILE, contradicting `VecIssueSlot`, `VecIssueUnit` and `VecPipeline`
  part 13, which all place the single declaration in `VecBundles` — part 13
  calls the two-name situation "a defect, not a synonym". Because `VecBundles`
  did not actually declare it, generation produced two structurally identical
  types facing each other across one seam (this file's `VecMemberRdy` and
  `VecIssueSlot`'s local shim), which `VecIssueUnit` cannot connect. The
  declaration now lives in `VecBundles`; the `maxGroupSize` parameter is gone
  because the export exists only on the vector instance, where it is `maxMembers`
  by definition. `VecBusyResp`/`VecMemberBusyResp` DO stay local to
  `VecBusyTable` — that is the busy sense, one producer and one consumer in the
  same subtree, and this file converts busy to ready.

  `vold_rdy` is `stale_pvdest`'s per-member readiness, the FIFTH group in the
  side channel. `IQ_V_LOAD` and `IQ_V_ALU` slots gate issue on it through a
  fifth VecGroupReady instance (`rdy_vold`): `stale_pvdest` is the PREVIOUS
  mapping of the destination arch vregs, so its producer is OLDER — and
  age-ordered issue grants the oldest READY entry, which does not mean an older
  producer has finished. The LCB pre-loads from it on R2 for `vta=0`/`vma=0`,
  and the coprocessor may pull `STALE_VD`. It is PER MEMBER for the same reason
  `pvs*` is: an LMUL=1 op writes v0, then an LMUL=8 op renames v0..v7, so its
  stale group is the current mappings of eight arch vregs installed by up to
  eight different instructions. An aggregate `stale_pvdest_busy` bit cannot
  express "waiting on producer 3 of 8" and no single group-done clears it
  correctly, so no such bit exists here or in `VecBusyResp`.

  NAMING, FLAGGED NOT FIXED: VecIssueSlot declares `VecSlotMemberRdy` for the
  consumer end of this same channel, with four groups (pvs1/pvs2/pvs3 + pvm) and
  no `vtmp`/`vold`. Seam review ROUND 5 item 15 rules that ONE declaration
  belongs in VecBundles. When that promotion happens, this bundle and that one
  must become the same type with SIX fields; two near-identical bundles across
  this seam is a mis-connection waiting to happen.

  `vl_rf_write` — Output(Vec(plWidth, Valid(addr: UInt(pregSz.W), data:
  UInt(vecVLSz.W)))), only when `hasRenameWrite`: VlRegFile's `W_ren` port,
  replicated per rename lane and never arbitrated.

  Observability: `debug_freelist` and `debug_busytable` are passed straight out
  from the children so both spaces look like the scalar tables in Verdi. Nothing
  functional may read either.

  NO ready/valid handshake anywhere. Reads are combinational, writes are
  registered, and the only back-pressure this module can exert is `alloc_ok`.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. One stage, three children, no state of its own ----

  //@req-spec-rename.a1
  //@req-spec-rename.a5
  //@req-spec-rename.a6
  //@req-spec-rename.d1
  This module extends the existing RenameStage implementation to vector renaming
  in a SINGLE PIPELINE STAGE. It instantiates `maptable`, `freelist` and
  `busytable`, wires them exactly as `RenameStage` wires its scalar three, and
  declares NO register on the path from `ren2_uops` to `ren2_uops_out`.
  Consequently there is no separate vector-mapping pipeline stage and no
  1-cycle-delayed pipeline register: the ren1->ren2 registers this module reads
  belong to the scalar RenameStage and are shared, not duplicated. A generator that
  adds a `RegNext` anywhere between the input uops and the output uops has
  reintroduced the two-stage rename this design exists to delete, along with all of
  its alignment machinery.

  //@req-spec-rename.a2
  //@req-spec-rename.a3
  //@req-spec-rename.a4
  For a vector instruction the scalar (INT/FP) rename and this vector group rename
  run IN PARALLEL IN THE SAME CYCLE, over independent register spaces, free lists
  and busy tables: nothing in this module reads `pdst`, `prs*`, `stale_pdst` or any
  scalar busy bit, and nothing in the scalar RenameStage reads a `pv*` field. So
  for a given OP.v neither rename depends on the other and the combined cycle costs
  the MAX of the two latencies, not their sum. The one shared signal is
  `dis_fire`/`dis_ready`, a bundle-level stall and not a data dependence.

  //@req-spec-rename.a11
  //@req-spec-rename.h9
  The two instances likewise run in parallel with each other: `vec_rename` renames
  `lvd`/`lvs*`/`lvm` to `pvdest`/`pvs*`/`pvm` while `vl_rename` renames VL into the
  VL register file, both in this same single rename cycle. The output chaining
  above is a WIRE pass-through of fields the second instance never reads, so it
  adds no logic depth and does not serialize the two group reads.

  ---- 2. Requests: which lanes rename in this space ----

  Following baseline exactly, per lane `w`:
    `ren2_alloc_reqs(w)` = `ren2_mask(w)` AND this space's destination predicate.
    `ren2_alloc_fire(w)` = `dis_fire(w) && ren2_alloc_reqs(w)`.
  For `vec_rename` the destination predicate is `needs_pvdest(w) || needs_pvtmp(w)`
  where `needs_pvdest(w)` is `ren2_uops(w).dst_rtype === RT_VEC` (the RT_VEC
  encoding the ScalarOpConstants delta adds — the direct analogue of baseline's
  `dst_rtype === rtype` test) and `needs_pvtmp(w)` is `is_vec && is_shared`. For
  `vl_rename` it is `ren2_uops(w).is_vl_producer`, which is ORTHOGONAL to
  `dst_rtype` and must never be inferred from it: `vsetvli x0, rs1, vtype` has
  `dst_rtype === RT_ZERO` and still writes the VL RF.

  ===> reqs INTO THE FREE LIST ARE NOT QUALIFIED BY dis_fire; alloc_fire IS.
  `freelist.io.reqs` takes `ren2_alloc_reqs` (fire-independent) so that
  `alloc_ok` -> `dis_ready` -> `dis_fire` -> `reqs` is not a combinational
  loop. Consumption of the selected PRNs is qualified by the fire instead.
  This is precisely baseline's split, where `can_allocate` comes from the
  free list's pre-selection REGISTER and never from `reqs`.

  ---- 3. Map table wiring ----

  `map_reqs(w)` takes the lane's `lvd`/`lvs1`/`lvs2`/`lvs3`/`lvm` and `emul :=
  ren2_uops(w).v_emul` (EMUL is derived at DECODE from the VCFG mirror; this
  module does not recompute it), with `valid := ren2_mask(w) && is_vec`. For
  `vl_rename` all five specifiers are tied to 0 — the space has one row — and the
  response's `stale_pvdest(0)` IS the current VL pointer.

  `remap_reqs(w)` takes `lvd`, `emul`, `pvdest := freelist.io.alloc_pvdest(w)` and
  `valid := ren2_alloc_fire(w) && needs_pvdest(w)`. `com_remap_reqs(w)` takes the
  committing uop's `lvd`/`pvdest`/`v_emul` with `com_valids(w)` below. The map
  table owns the EMUL-wide group read, the atomic group install and the PRN half
  of the in-bundle prefix bypass; this module must not re-mux a PRN it returned.

  Response into the uop, per lane: `pvs1`, `pvs2`, `pvs3`, `pvm`, `stale_pvdest`
  and `v_emul` are taken from `map_resps(w)` verbatim, and `pvdest` from the free
  list. `stale_pvdest` is written for every lane with `needs_pvdest`, including
  ops that encode no third source — it is a SEPARATE field from `pvs3` naming a
  separate group and the two must never be merged or collapsed here.

  UNENCODED SOURCES ARE SKIPPED, using the `v_uses_vs1`/`v_uses_vs2`/`v_uses_vs3`
  bits `MicroOp` now carries (decision D11, set by `VDecode`/`VLSDecode`, which are
  the only nodes that know the instruction FORMAT). Per lane and per source `s` in
  {1,2,3}, when `!v_uses_vs<s>(w)` this mapper does NOT rename that source: the
  map response for it is ignored, `pvs<s>_busy` is forced CLEAR, and every member
  of `member_rdy(w).vs<s>_rdy` is forced READY. `lvs<s>` is a don't-care field in
  such an encoding, so the map table's answer for it is a real PRN belonging to
  somebody else and its busy bit is somebody else's business.

  ===> THIS IS WHY IT MATTERS, AND IT IS A HANG. For `vadd.vx` (`vd, vs2, rs1`)
  `lvs1` is unencoded, so `pvs1` resolves to the CURRENT MAPPING OF v0 — the mask
  register, which is written constantly. If v0's producer's group-done fired
  BEFORE the slot captured its member-ready bits, no future group-done clears
  them and THE SLOT WAITS FOREVER. VecIssueSlot placed this obligation on this
  module and this module previously could not discharge it, because no
  `lvs*_rtype` existed; D11 supplies the three bits, so it is discharged HERE,
  at the source, and `VecIssueSlot` additionally consumes the same bits as
  VecGroupReady's `used` input. Mirrors `v_is_masked`, which exists for exactly
  this reason on `pvm`. Do NOT instead special-case an opcode list here.

  //@req-spec-decode.i5
  //@req-spec-decode.i2
  For `vl_rename`, `uop.pvl := Mux(is_vl_producer && ren2_alloc_fire(w),
  freelist.io.alloc_pvdest(w)(0), map_resps(w).stale_pvdest(0))`. So every VL
  PRODUCER allocates a FRESH VL PRN, and every younger vector uOP carries the
  CURRENT `pvl` read from the VL map table at rename as an implicit operand — one
  read, one mux, no second structure. The non-producing `vsetvli x0, x0` keep-VL
  form takes the consumer arm by construction, because `is_vl_producer` is clear
  for it, so the surviving `pvl` is preserved rather than replaced by a VLMAX
  recomputation.

  ---- 4. Free list wiring, and the shared instruction ----

  `freelist.io.initial_allocation := Cat(~0.U((numPhysRegs-numArchRegs).W),
  0.U(numArchRegs.W))` — the low `numArchRegs` PRNs are the identity mapping the
  committed table holds permanently and are never free.

  ===> `req_members(w) := v_emul` FOR `vec_rename` AND THE CONSTANT `1` FOR
       `vl_rename` — NOT `v_emul` IN BOTH. An earlier revision of this sentence
       read "`req_members(w) := ren2_uops(w).v_emul` (always 1 in the VL space)".
       The parenthetical states the right fact and draws the wrong conclusion:
       **because** a `vset` carries no EMUL group, `v_emul` there is **0**, not 1.
       A vset is modelled as a scalar uop, so nothing ever sets its `v_emul`.

       Reading `v_emul` in the VL space therefore requests a ZERO-member
       allocation, and `VecFreeList` grants nothing for `pvl`. Its in-range
       assertion caught it at 3035 ns of the gate-(e1) cosim ("req_members out of
       [1, maxGroupSize] range while reqs is asserted"). The SAME false premise
       produced the same bug in the busy-table shim of part 5 — where the
       consequence is worse, because an empty set mask is a silent stale-VL read
       rather than an assertion. Note `map_reqs(w).emul` already used `1.U` for
       this space, so the two sites disagreed with each other.

       THE MODULE THEREFORE DECLARES TWO NAMED HELPERS — `renMembers(w)` and
       `comMembers(w)` — each `v_emul` in the vector instance and the constant `1`
       in the VL one. Use them in ANY code that elaborates in both instances;
       read `v_emul` directly only inside an `if (vectorInstance)` arm.

       ===> AND CHECK REACHABILITY FROM THE BLOCK STRUCTURE, NOT FROM NEARBY
       CONTEXT. A third instance of this same bug (the dealloc-range
       ASSERTIONS, at 3049 ns) survived an audit that concluded "all other
       reads are gated", because those assertions sit a few lines from the
       `stale_group` dealloc LOGIC and read the same expression under a
       DIFFERENT condition — the logic is vec-only, the assertions elaborate in
       both. Grepping a window around a read is not sufficient evidence about
       which space it belongs to; the helpers exist so the question does not
       have to be re-answered.

  `ren_br_tags`, `brupdate` and
  `rollback` are forwarded. `alloc_fire(w) := ren2_alloc_fire(w)`, i.e. the
  PER-LANE `dis_fire(w) && ren2_alloc_reqs(w)`.

  ===> `alloc_fire` IS Vec(coreWidth, Bool), one bit per lane, and VecFreeList's
  spec now declares that shape (seam review A1 — APPLIED, no longer open). It is
  a CORRECTNESS amendment, not style. BOOM's
  `dis_stalls` is a PREFIX SCAN (core.scala:773), so a partial-prefix fire is
  reachable from any NON-vector hazard — `ldq_full` on lane 2 lets lanes 0..1
  dispatch while lane 2 does not. With one bit the free list would consume lane
  2's window PRNs anyway; lane 2 then retries next cycle and allocates a SECOND
  group, and the first is held by nothing and freed by nothing. That is the M1
  double-allocate/leak class, reached without any vector-side mistake. The
  whole-bundle rule this module enforces is on the GRANT (`alloc_ok`), which
  still cannot be partial; it was never a claim that DISPATCH cannot be.
  Do not "restore" the single bit by OR-reducing this vector, and do not
  re-qualify it with `alloc_ok` — `alloc_ok` already gates every lane's
  `dis_fire` through `dis_ready`, so a fire implies the grant by construction.

  //@req-spec-rename.e1
  //@req-spec-rename.e3
  //@req-spec-rename.e5
  A shared (segmented) instruction is allocated PRNs and a single ROB entry as an
  ordinary instruction — one uOP, no cracking, the same request path. In ADDITION,
  when a lane needs BOTH a `pvdest` group and a `pvtmp` group it raises
  `req_shared(w)` so the free list hands back a SECOND group of the same member
  count from the main vector free
  list, and it writes those PRNs into that OP.v's `pvtmp` field, up to EMUL
  members. `pvtmp` is never offered to the map table: it has no architectural
  name, so no ordinary `lvs*` read can alias it.

  Concretely, per lane, and this is the whole of the mapping (seam review A6 —
  APPLIED):
    `req_shared(w) := needs_pvdest(w) && needs_pvtmp(w)` TWO groups
    two-group case  : `pvdest := alloc_pvdest(w)`, `pvtmp := alloc_pvtmp(w)`
    tmp-only case   : `pvtmp  := alloc_pvdest(w)`, `pvdest` left UNWRITTEN
    dest-only case  : `pvdest := alloc_pvdest(w)`, `pvtmp` left unwritten
  `reqs(w)` is high in all three (it is `needs_pvdest || needs_pvtmp`, part 2).

  ===> `req_shared` IS NOT `is_shared`, AND CONNECTING `is_shared` TO IT LEAKS A
  GROUP FOREVER. A SEGMENTED STORE NEEDS pvtmp AND NO pvdest — it has no vector
  destination, so its `dst_rtype` is not RT_VEC and `needs_pvdest` is false —
  yet it IS `is_shared`. Request two groups there and one of them is named by no
  uop field: commit frees only `stale_pvdest` and `pvtmp`, so neither ever names
  it, no branch reclaim covers it (it was granted, so it IS in `alloc_masks`,
  but only a mispredict would return it), and on the committing path it is gone
  until reset. The free list cannot detect this — it grants what it is asked
  for — so the check lives here: `req_shared` counts GROUPS, and the tmp-only
  lane takes its single group off `alloc_pvdest`.

  //@req-spec-rename.e13
  //@req-spec-rename.e16
  //@req-spec-rename.h26
  `alloc_ok` is the free list's whole-bundle verdict, unmodified. An OP.v that
  cannot allocate its vector group, and a VL producer that finds no free VL PRN,
  therefore simply STALL AT RENAME, and the failure stalls the WHOLE dispatch
  group: no uop in the bundle renames that cycle and the bundle retries intact on
  the next. This is deadlock-free without any headroom reservation because rename
  is in program order — every older instruction keeps committing and freeing the
  group (or the pointer) it displaces, so the oldest op is never the one that
  starves. The cost is dispatch bandwidth on the retry, which is accepted
  deliberately: a per-lane stall mask would have to be reconciled with the atomic
  ROB / LDQ / STQ / br_tag reservations of part 6.

  ---- 5. Busy table wiring, and the two busy bits of a vset ----

  `busytable.io.ren_uops` is driven from a PRIVATE wire `bt_uops`, not from
  `ren2_uops_out`: it is the output uop with THIS SPACE'S destination group placed
  in `pvdest`, which for `vl_rename` means `bt_uops(w).pvdest(0) := uop.pvl`
  narrowed to `pregSz`, because the busy table's set path reads `pvdest`.

  ===> AND `vl_rename`'s SHIM MUST ALSO FORCE `bt_uops(w).v_emul := 1`. THE VL
       "GROUP" IS ONE REGISTER, AND WITHOUT THIS `pvl` IS NEVER MARKED BUSY.

       `VecBusyTable`'s set path is generic across both instances and qualifies
       each member with `j < uop.v_emul` (its part 6). The uop renamed in this
       space is a `vset`, which is modelled as a SCALAR uop — `is_vec` clear,
       `v_emul` **0**, because a vset has no vector destination group. So
       `j < 0` is false for every j, the set mask is EMPTY, and the VL busy bit
       is silently never set.

       That is a CORRECTNESS bug and not merely an assertion failure: a
       register-sourced `vsetvli`/`vsetvl` writes `pvl` at ALU writeback, so a
       younger vtype-dependent OP.v must wait on that busy bit. With the bit
       never set the dependent is ready immediately and reads a STALE VL out of
       the VL RF. Found by `VecBusyTable`'s own "rebusy_reqs asserted with
       v_emul == 0" assertion at 3035 ns of the gate-(e1) cosim.

       Force it in the shim that already exists to adapt this space's uop to the
       generic table. Do **not** relax that assertion (it hides the empty set
       mask) and do **not** special-case `maxGroupSize == 1` inside
       `VecBusyTable` (that puts VL knowledge into a module whose whole point is
       having none). Drive `is_shared := false` in the same place, which
       `VecBusyTable`'s part 6 already asks of this file — a vset never sets it
       today, so that one is defensive rather than a fix.

  ===> `vl_rename` MUST NOT WRITE `pvdest` ON ITS OUTPUT PATH. `pvdest` there
  belongs to the vector space and holds a real 7-bit vector group; a 6-bit VL
  PRN dropped into member 0 of the output uop would corrupt the destination
  group of every vector instruction that is also a VL producer.

  //@req-spec-decode.c21
  //@req-spec-decode.i3
  `rebusy_reqs(w) := ren2_alloc_fire(w)`, the same event the free list allocates
  on, EXCEPT that `vl_rename` additionally suppresses it when
  `ren2_vl_imm_valid(w)`. For a register-sourced `vset` rename therefore sets BOTH
  busy bits — the integer busy for `pdst` in the untouched scalar RenameStage and
  the VL busy for `pvl` here — and both wakeup networks later fire. For the
  front-end-only `vsetivli` the VL value arrives WITH the allocation:
  `vl_rf_write(w)` is driven with `valid := dis_fire(w) && ren2_vl_imm_valid(w)`,
  `addr := uop.pvl`, `data := ren2_vl_imm(w)`, writing the new VL into the VL
  register file in this same rename cycle on VlRegFile's statically partitioned
  rename-side port, and no busy bit is set — that `pvl` is BORN READY. There is no
  decode-time VL fast path and no bypass around the VL RF for consumers.

  Busy responses reach the uop with the qualifications only this module can apply:
  `pvs<s>_busy := v_uses_vs<s> && busy_resps(w).pvs<s>_busy` for `s` in {1,2,3}
  (decision D11 — an unencoded source is not renamed and its busy bit stays CLEAR;
  see part 3, and note this is the SAME shape as the mask qualifier beside it),
  `pvm_busy := v_is_masked && busy_resps(w).pvm_busy` (without that qualifier every
  unmasked vector op gains a false dependency on the last writer of `v0`),
  `pvtmp_busy := is_shared && busy_resps(w).pvtmp_busy` (`is_shared` HERE, not
  `req_shared`: every shared op HAS a `pvtmp` group and so has a real `pvtmp` busy
  lifetime; `req_shared` is narrower and counts GROUPS TO ALLOCATE, and a segmented
  store has one of those and a `pvtmp` — do not unify the two predicates),
  and on the VL instance
  `pvl_busy := is_vec && busy_resps(w).pvl_busy`. The vector instance has no
  `pvl` read port and the VL instance has no vector source ports, so reading a
  `pvl` bit out of the wrong table — the M1 bug — is unspellable.

  CLOSED (was OPEN for Phase R; seam review A7, settled by decision D11). The
  per-source USE predicate now exists as three `MicroOp` Bools,
  `v_uses_vs1`/`v_uses_vs2`/`v_uses_vs3`, set by VDecode/VLSDecode — the only
  nodes that know the instruction format. It is consumed in TWO places on
  purpose: HERE, to skip renaming and leave the busy bit clear (part 3), and in
  VecIssueSlot, as VecGroupReady's `used` input. Neither alone is sufficient —
  the slot's `used` cannot repair a member-ready vector that was captured with a
  stale v0 mapping in it, and this module cannot stop a matcher from watching a
  group it was told about. Rejected alternatives, for the record: a reserved
  sentinel in `lvs*` (free at `lregSz = 6`, but every future reader must
  remember the convention and the failure mode is a silent hang) and full
  `lvs*_rtype` fields (3x the bits for no extra information).

  ---- 6. The in-bundle prefix bypass — the part this module owns ----

  //@req-spec-rename.b2
  //@req-spec-rename.b3
  //@req-spec-rename.h10
  Everything the rename->dispatch boundary does happens atomically and IN PROGRAM
  ORDER inside this one cycle, so lane `i` must see what lanes `0..i-1` renamed in
  the same cycle. `BypassAllocations` is the vector override of baseline's method
  of the same name, and it does exactly ONE thing here: it ORs the in-bundle
  dependence into the READINESS fields. Per lane `i`, over older lanes `k < i`
  with `ren2_alloc_fire(k)`, a per-member RANGE hit is computed — "does row
  `lvs* + m` fall inside lane `k`'s destination group `[lvd, lvd + v_emul)`" — and
  where it hits, that member is NOT ready and the operand's aggregate busy bit is
  forced high. For `vl_rename` the same construction is one row wide, which is
  what lets a `vset` and its dependent share a dispatch group: the dependent picks
  up the just-renamed `pvl` and, unless the older producer was a born-ready
  `vsetivli`, also picks up its busy bit.

  Two traps in one paragraph.
  (1) THE PRN HALF OF THE BYPASS IS VecMapTable'S, not this module's — it
      folds over `remap_reqs` internally with the same range test. Do NOT also
      re-mux `pvs*`/`stale_pvdest`/`pvl` here; a second bypass can only mask a
      bug in the first. Assert agreement instead: on a hit, the map response's
      member must equal the older lane's `pvdest(row - lvd)`.
  (2) THE COMPARE IS PER MEMBER AND AGAINST A RANGE, NEVER BASE-TO-BASE. An
      older LMUL=8 write to v0..v7 must bypass into a younger LMUL=2 read at
      v4. Baseline's `r.ldst === uop.lrs1` is correct only because a scalar
      mapping is one register; copied verbatim it misses every sub-range read
      and the younger op issues against a stale PRN pair.
  (3) A born-ready `vsetivli` MUST NOT set the dependent's `pvl_busy`. The
      bypass term is qualified by `!ren2_vl_imm_valid(k)`. Without that
      qualification the dependent waits on a wakeup that will never come,
      because no busy bit was ever set for that producer to clear.
  (4) THE BYPASS COVERS `stale_pvdest` TOO, and its range test is on `lvd`, not
      on an `lvs*`: lane `i`'s stale group IS the current mapping of its own
      destination rows, so if an older lane `k < i` renames any row in
      `[lvd_i, lvd_i + v_emul_i)` then lane `i`'s stale group members ARE lane
      `k`'s freshly allocated PRNs, which are busy by definition. Two writers of
      v0 in one bundle is the everyday case (mask updates), so omitting this term
      hands `rdy_vold` an all-ready vector for a group nothing has written yet.

  `member_rdy(i)` is the per-member READY vector: the busy table's PRE-REDUCTION
  per-member read, inverted, ANDed with the negation of this bypass's per-member
  hit. It is exported alongside the uop and NOT placed in `MicroOp`, whose reject
  list forbids a per-member busy vector. Six fields, five of them vectors:
  `vs1_rdy`, `vs2_rdy`, `vs3_rdy`, `vtmp_rdy`, `vold_rdy` (D6) and the single
  `vm_rdy`. `vs<s>_rdy` is forced ALL-READY when `!v_uses_vs<s>` (D11, part 3), and
  `vtmp_rdy`/`vold_rdy` carry no information on a lane that has no such group — the
  consumer's `used` input is what suppresses them there, exactly as for `vm`.

  The pre-reduction read arrives on a SECOND busy-table output,
  `member_busy_resps: Vec(plWidth, per-operand Vec(maxGroupSize, Bool))`, gated
  by the same `exportMemberRdy` this module passes down. That is an AMENDMENT to
  VecBusyTable (seam review A2 — APPLIED in its file), whose earlier spec exported
  only the AND-reduced `busy_resps`: it is a fan-out of the
  wires its own per-member read (rename.g11) already computes, before the
  reduction — zero new comparators, zero new state — and `busy_resps` keeps its
  stated aggregated-only shape. Two nodes (VecIssueSlot, VecGroupReady) already
  depend on this data existing; the alternative is a second busy table.
  Its FIFTH group, `pvold_busy` from `stale_pvdest`, is the one part that is a
  genuine added read (8 bit-reads per lane) rather than a fan-out — the cost D6
  accepts, in the stage that is already this design's #1 timing risk.

  ===> WHY PER MEMBER MUST LEAVE THIS MODULE. A source group's members can come
  from DIFFERENT producers, so the group can be not-ready in aggregate while
  members 0..2 are already complete. VecGroupReady initializes its per-member
  state from `in_member_rdy`; loading the AGGREGATE into all members makes the
  slot wait for group-dones that have already fired — a PERMANENT HANG, and
  exactly the failure two sibling nodes now depend on this port to avoid.
  `vold_rdy` is the sharpest case of it: `stale_pvdest` can span up to
  `maxGroupSize` producers (an LMUL=1 op writes v0, then an LMUL=8 op renames
  v0..v7, so its stale group is eight arch vregs' current mappings installed by
  up to eight instructions), which is precisely why D6 rejects a single
  aggregate `stale_pvdest_busy` bit as UNSAFE rather than merely conservative.

  ---- 7. Branch tags, snapshots and recovery ----

  //@req-spec-rename.i2
  //@req-spec-rename.i3
  //@req-spec-rename.i4
  //@req-spec-decode.i8
  `ren2_br_tags` is COMPUTED HERE, not received: `Vec(plWidth + 1,
  Valid(UInt(brTagSz.W)))` with entry 0 tied invalid, and entry `w+1` valid on
  `dis_fire(w) && ren2_uops(w).allocate_brtag` carrying `ren2_uops(w).br_tag` —
  character for character baseline's `ren2_br_tags`. Because both derive from the
  SAME registered `ren2_uops` and the SAME `dis_fire`, the vector RMT and the VL
  map table are snapshotted on the same `ren_br_tags` event as the scalar RMT by
  construction rather than by review, and the VL map table is branch-snapshotted
  per `br_tag` like any other. There is NO delayed-`br_tag` path here: deriving
  the event locally is what makes one impossible to add by accident, whereas an
  input port for it would invite a parent to register it "for timing".

  //@req-spec-rename.i12
  //@req-spec-rename.i13
  //@req-spec-decode.i9
  Recovery is entirely the children's, driven from two forwarded signals and
  nothing else. On `brupdate.b2.mispredict` the map table restores
  `br_snapshots(brupdate.b2.uop.br_tag)` and the free list reclaims
  `br_alloc_lists(br_tag)` — ONE CYCLE, flushing only state younger than the
  branch, so a `vset` on a squashed path cannot corrupt the surviving path's VL
  mapping. On `rollback` the map table copies `com_map_table` and the free list
  returns `spec_alloc_list` — one cycle, flushing everything in flight. This
  module adds no third recovery arm, no ROB walk-back and no periodic snapshot.

  ---- 8. Commit wiring, and the two free paths ----

  //@req-spec-decode.c24
  Per commit lane `w`, `com_valids(w)` is `io.com_valids(w)` AND this space's
  commit predicate: `com_uops(w).dst_rtype === RT_VEC` for `vec_rename`,
  `com_uops(w).is_vl_producer` for `vl_rename`. Under "stale_group" the free
  list's `dealloc` slots `w*maxGroupSize + j` take `com_uops(w).stale_pvdest(j)`
  valid for `j < v_emul`, and `dealloc_tmp` takes `pvtmp(j)` under the separate
  gate `io.com_valids(w) && is_vec && is_shared` — separate because a segmented
  STORE has a `pvtmp` group to free and no vector destination at all. Under
  "committed_ptr" slot `w*maxGroupSize` takes `maptable.io.com_stale_resps(w)(0)`,
  the pointer the commit install DISPLACES, read from the committed table before
  the update. A register-sourced `vset` therefore runs BOTH free paths at commit:
  its stale integer `pdst` through the untouched scalar path, and its outgoing
  committed VL pointer through this one. No `stale_pvl` field exists and none is
  needed.

  Carry over baseline's leak assertion, per space: `!RegNext(rollback) ||
  PopCount(freelist.io.debug_freelist) === (numPhysRegs - numArchRegs).U`. It is
  the cheapest detector for the whole class of bug this module's history is made
  of, and it fires within a few cycles of the leak rather than thousands.

  ---- 9. What this module deliberately does NOT do ----

  No ROB-safety field is written here. `rob_unsafe` for vector ops is cleared
  by the Rob delta's group-safe path (`vec_clr_unsafe`), and MicroOp's edit
  scope freezes `starts_unsafe`. If the PNR assertion in rob.scala trips at
  bring-up on a vector op — the M2 symptom — the fix is that path, NOT a
  tie-off in the rename stage. Reported rather than silently implemented.

  No `child_rebusys` port and no speculative rebusy: vector operands are never
  woken speculatively, so there is nothing to cancel. No `despec` port and no
  `isImm` mode. No LDQ/STQ or ROB index assignment — those reservations happen
  in this same cycle but in BoomCore and VecQueueReservation, and duplicating
  the index arithmetic here would give two structures an opinion on program age.

  ---- 10. Trace ----

  There are NO unit tests in this project — validation is end-to-end VCS plus
  Whisper cosim — so emit guarded `VecTrace` lines, gated on the `vecTrace` plusarg
  and `!reset`, off by default, all keyed on `rob_idx`: `tracePrn` per renamed lane
  (event "ren": `v_emul`, the `pvdest` members, the `pvtmp` members when shared),
  `traceVl` per VL rename (event "vl": `pvl`, the written value, born-ready or not),
  one line when `alloc_ok` is low (event "stall"), one per recovery event naming the
  arm that fired and the `br_tag`. Emit-only: no register and no counter that
  functional logic reads. The stall line is what makes a dispatch bubble
  attributable to vector-PRN exhaustion instead of guessed at from a cycle count.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput: one full dispatch bundle per cycle, `plWidth` group renames, with no
stall of this module's own beyond `alloc_ok`. Latency: ZERO cycles — reads and
`alloc_ok` are combinational from `ren2_uops`, writes are registered inside the
children. A pipeline register anywhere on the input-to-output path is a functional
error here, not a timing trade: it would re-create the cross-group skew that
`spec-rename.a5`/`a6`/`b1` exist to forbid.

===> THIS IS THE TOP TIMING RISK IN THE WHOLE DESIGN, and it is not yet bounded.
One cycle carries, in parallel with scalar rename: `plWidth * 4` EMUL-wide map
reads plus the mask read, each with a prefix-bypass fold; up to
`coreWidth * maxGroupSize` = `coreWidth * 8` PRN allocations from a single
`SelectFirstN` over `numPhysRegs` bits; on the order of 25 to 41 per-member busy
reads per lane (41 with `pvtmp` and, per D6, `stale_pvdest`) AND-reduced to
group-ready bits; this module's own per-member range
comparisons for the readiness bypass, now including the `lvd`-based term for
`vold_rdy`; and the whole VL instance beside it. The
spec's argument — "cost = the slower of the two, not the sum" — is the right SHAPE
but it is not a bound.

A TIMING SPIKE ON THIS MODULE AND ITS THREE CHILDREN IS REQUIRED BEFORE THE RENAME
INTEGRATION STEP LANDS. If the vector side loses, THE WHOLE CORE'S RENAME STAGE
PAYS — this is not a vector-only regression, because the stage is shared. The known
fallback is splitting rename back into two stages with all the alignment machinery
this design deleted, and it must stay a known-cost decision rather than a late
surprise. Cheap levers already taken: the deleted LMUL tag checker (one comparator
tree off the path), member selection as a mux over the legal counts 1/2/4/8 rather
than dynamic adders, and the free list's pre-selection registers, which keep the
selector's priority chain out of the grant path and make `alloc_ok` available at the
start of the cycle.

Area: this module adds essentially none of its own — per instance the state is
`numArchRegs * pregSz * (2 + maxBrCount)` bits of map tables and snapshots plus
`numPhysRegs * (2 + maxBrCount)` free-list bits plus `numPhysRegs` busy bits, the
snapshot terms dominating. The free list's `stall_cnt_inc` rate is a first-class
number here too: with 8 renamable LMUL=8 groups and 4 segmented ones, whole-bundle
stalling at rename is the expected limiter on vector memory-level parallelism.
<|end_perf|>

<|begin_dependencies|>
Instantiates, once each, per instance of this module:
  VecMapTable  as `maptable`  — parameters `plWidth`, `numArchRegs`,
    `maxGroupSize`, `numPhysRegs`, `bypass`, and `exportComStale =
    (freeDiscipline == "committed_ptr")`. Bundles `VecMapReq`/`VecMapResp`/
    `VecRemapReq` on that seam.
  VecFreeList  as `freelist`  — parameters `numPhysRegs`, `numArchRegs`,
    `maxGroupSize`, `freeDiscipline`; `allocWidth`/`deallocWidth` are DERIVED
    inside it and must not be passed.
  VecBusyTable as `busytable` — parameters `plWidth`, `numPregs = numPhysRegs`,
    `maxGroupSize`, `numWbPorts`, `wakeupKind`, and `exportMemberRdy` passed
    straight through from this module's own parameter of the same name. Declares
    `VecBusyResp` AND `VecMemberBusyResp` itself; this file must not redeclare
    either, and declares `VecMemberRdy` (the READY-sense export) itself in turn.

depends_on: MicroOp (every `pv*` field, `stale_pvdest`, `v_emul`, `is_vec`,
`is_shared`, `is_vl_producer`, `v_is_masked`, and the three D11 bits
`v_uses_vs1`/`v_uses_vs2`/`v_uses_vs3`), VecBundles (`VecGroupDone` on the wakeup port),
VectorParams (`numVecPhysRegisters`, `numVlPhysRegisters`, `maxMembers`,
`vecPregSz`, `vlPregSz`, `vecVLSz`), VecTrace (the guarded helpers).

Reused from baseline BOOM unchanged, not reimplemented: `BrUpdateInfo`,
`brTagSz`, `maxBrCount`, `enableSuperscalarSnapshots`, `retireWidth`, `coreWidth`,
`RT_VEC`/`RT_ZERO` from ScalarOpConstants, and `BoomModule`/`BoomBundle` for the
implicit `Parameters`. `GetNewUopAndBrMask` is deliberately NOT called here.

Its only parent is VecPipeline, which instantiates it twice, chains the two
outputs, ANDs the two `alloc_ok`s into `dis_ready`, routes `vl_rename`'s
`vl_rf_write` to VlRegFile's `W_ren`, routes `vec_rename`'s `member_rdy` to the
vector issue slots' `in_member_rdy` — all SIX fields, including `vold_rdy` into the
`IQ_V_LOAD`/`IQ_V_ALU` slots' fifth matcher `rdy_vold` (D6) — and — the one
obligation that matters most —
drives `ren2_uops`/`dis_fire` from the scalar RenameStage's REGISTERED ren2 stage
and never from `dec_uops`.
<|end_dependencies|>

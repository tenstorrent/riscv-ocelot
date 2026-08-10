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
  VecMapTable — the vector Rename Map Table: architectural vreg -> physical
  GROUP, read EMUL-wide and written atomically per group.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/rename/VecMapTable.scala,
  package boom.v4.vec.generated.rename, group vec_rename,
  depends_on VecBundles, VectorParams, VecTrace. Instantiated by VecRenameSpace
  as instance `maptable`, once per rename space.

  It is the vector analogue of `class RenameMapTable` in
  src/main/scala/v4/exu/rename/rename-maptable.scala and keeps that module's
  structure and names — `map_table`, `com_map_table`, `br_snapshots`,
  `remap_table`, `map_reqs`/`map_resps`, `remap_reqs`/`com_remap_reqs`,
  `ren_br_tags`, `rollback`. Diffed against it there should be exactly two
  differences: mappings are read and written EMUL-wide, and the stale read is a Vec.

  ===> ONE PRN PER ARCHITECTURAL VREG. That single property is the whole design.
       An EMUL-wide read is just EMUL adjacent row reads, correct however badly
       the physical registers are fragmented, so THREE structures are ABSENT BY
       CONSTRUCTION and their absence is specified below rather than merely
       omitted: a contiguous-run allocator, a whole-group validity check, and a
       fragmentation-recovery walk. Adding any of the three back means the table
       has been misread as storing a group descriptor.

  ===> NO LMUL TAG TABLE AND NO WHOLE-GROUP CHECKER. An earlier draft carried a
       32x2-bit per-ARN tag table plus a comparator tree testing every valid base
       ARN at each LMUL. Deleted: it gated nothing — that draft concluded a FALSE
       result needed neither exception nor recovery — and its test was wrong
       anyway, classifying a group written by eight independent LMUL=1
       instructions as fragmented when that group reads back perfectly. Deleting
       it removes 64 bits of state and a wide comparator tree from the
       single-cycle rename critical path, this design's principal timing risk.

  ===> ONE DEFINITION, TWO INSTANCES. VecRenameSpace is instantiated twice, so
       this module elaborates twice: the vector space (numArchRegs = 32,
       maxGroupSize = 8, numPhysRegs = numVecPhysRegs) and the VL space
       (numArchRegs = 1, maxGroupSize = 1, numPhysRegs = numVlPhysRegs). The VL
       instance is not a special case in the source — every group construct below
       degenerates at elaboration when maxGroupSize is 1. Do not write a second
       module for it.

  Governing spec anchors: midcore.rst `rmt`, `rename-stage`, `snapshots`,
  `cii-shared-mapping`, `vl-vtype-rename`, `old-vd`; glossary.rst
  `glossary-terms`.
*/

<|begin_module|>

  <|begin_parameters|>
  All Scala values resolved at elaboration. The whole module is elaborated only
  when `usingRVV` is true — with vectors off it is absent, not tied off, so a
  non-vector build emits RTL bit-identical to pre-Caracal BOOM v4.

  `plWidth` — rename lanes per cycle. Default `coreWidth` (3), legal 1..`coreWidth`.
  Every port is `plWidth` wide, with `plWidth + 1` intermediate table states so a
  snapshot can be taken between any two lanes.

  `numArchRegs` — architectural registers in this space: 32 for the vector space
  (fixed by RVV, not a knob), 1 for the VL space. `maxGroupSize` — most registers
  one instruction may rename atomically: 8 (LMUL/EMUL <= 8) and 1 respectively.
  Require both powers of two and `maxGroupSize <= numArchRegs`, so a row index is
  a bit-select and not a modulo.

  `numPhysRegs` — `numVecPhysRegisters` (96) or `numVlPhysRegisters` (64), from
  VectorParams; derived `pregSz = log2Ceil(numPhysRegs)`, 7 and 6 bits. No width
  in this file is a literal.

  `bypass` — build the in-bundle prefix bypass. Default true for both instances; a
  parameter only so a bring-up build can rule it in or out as the cause of a bug.
  Without it a `vset` and its dependent, or a producer and consumer, in one dispatch
  group would read a stale mapping.

  `exportComStale` — expose the committed table's displaced mapping (logic section).
  Default false; VecRenameSpace sets it true for `vl_rename` only, whose free
  discipline is `committed_ptr`. Gating it keeps `plWidth * maxGroupSize` dead read
  wires out of the vector instance.

  Derived: `lvregSz = max(log2Ceil(numArchRegs), 1)` for a row index, `emulSz =
  log2Ceil(maxGroupSize) + 1` for a member COUNT in 1..8. Snapshot depth and
  snapshot-write style are BOOM's existing `maxBrCount` and
  `enableSuperscalarSnapshots`, not new parameters — this table must snapshot on
  exactly the events the scalar table does.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit `clock` and `reset`: posedge `clock`,
  ACTIVE-HIGH SYNCHRONOUS `reset`. No second clock domain, no asynchronous reset.

  NO ready/valid handshake on any port, deliberately. This is a combinational read
  plus a registered write driven in lockstep with the rename stage; when a bundle
  cannot allocate, VecRenameSpace stalls the WHOLE dispatch group and does not
  assert the remap valids that cycle. A `ready` here would let one lane install a
  mapping its group-mates did not.

  `map_reqs` — Input(Vec(plWidth, VecMapReq)). Per lane: specifiers `lvd`, `lvs1`,
  `lvs2`, `lvs3`, `lvm`, each `lregSz` wide to match the MicroOp fields of the same
  names (only the low `lvregSz` bits index the table); `emul`, `emulSz` wide, the
  group member COUNT as a 1..maxGroupSize value; `valid`, the lane's `is_vec` (for
  the VL instance, "produces or reads VL").

  // ===> `lregSz` IS INHERITED, NOT A CONSTRUCTOR PARAMETER. `BoomBundle` mixes in
  // `HasBoomCoreParameters`, which already declares `val lregSz`
  // (`parameters.scala:430`), so a `class VecMapReq(val lregSz: Int, ...)` does not
  // compile — scalac demands an `override` modifier for a concrete inherited member.
  // Adding `override` would be worse than the error: it introduces a second source
  // of truth for a width whose whole purpose is "match the MicroOp fields of the
  // same names", and a caller could then pass something else. The same applies to
  // `VecRemapReq`'s `lvd`. Take `lregSz` from the trait; parameterize only
  // `pregSz`, `maxGroupSize` and `emulSz`, which the trait does NOT provide and
  // which genuinely differ between the vector and VL instances.
  //
  // Found at Phase C's gate (a) — the first generation produced constructor
  // parameters and every call site then passed the trait's own `lregSz` into them,
  // which is the proof they were redundant.

  `map_resps` — Output(Vec(plWidth, VecMapResp)). Per lane: `pvs1`, `pvs2`, `pvs3`,
  `stale_pvdest`, each `Vec(maxGroupSize, UInt(pregSz.W))`; `pvm`, a single
  `UInt(pregSz.W)`; `v_emul`, `emulSz` wide. These land in the OP.v fields of the
  same names.

  `remap_reqs` — Input(Vec(plWidth, VecRemapReq)): `lvd`, `pvdest` as
  `Vec(maxGroupSize, UInt(pregSz.W))` (the freshly allocated group, from
  VecFreeList via VecRenameSpace), `emul`, `valid`; speculative installation.
  `com_remap_reqs` — the same bundle from the ROB's commit ports; architectural
  installation. `com_stale_resps` — Output(Vec(plWidth, Vec(maxGroupSize,
  UInt(pregSz.W)))), only when `exportComStale`: per commit lane, the mapping the
  committed table held BEFORE that lane's `com_remap_req` is applied.

  `ren_br_tags` — Input(Vec(plWidth + 1, Valid(UInt(brTagSz.W)))): the same event
  the scalar RMT snapshots on, same `plWidth + 1` shape so a branch's snapshot
  reflects the mappings as of its own position in the bundle. `brupdate` —
  Input(new BrUpdateInfo); only `b2.mispredict` and `b2.uop.br_tag` are read.
  `rollback` — Input(Bool()), the ROB's exception/flush pulse.

  `vec_trace_en` is NOT a port: tracing reads the `vecTrace` plusarg through the
  shared VecTrace package.
  <|end_ports|>

  <|begin_logic|>
  ---- State ----

  //@req-spec-rename.d7
  `map_table` is a `Reg` of `numArchRegs` entries, each `pregSz` wide: ONE PRN PER
  ARCHITECTURAL VREG and nothing else — no per-entry group size, no base+count
  descriptor, no validity bit, no LMUL tag. `com_map_table` is a second array of
  the same shape holding the committed mapping, and `br_snapshots` is
  `Reg(Vec(maxBrCount, Vec(numArchRegs, UInt(pregSz.W))))`, one copy of the
  speculative table per outstanding branch. All reset to the identity mapping,
  ARN i -> PRN i, as the scalar table does — which is also why the low
  `numArchRegs` PRNs are permanently committed state and never enter the free list.

  //@req-spec-rename.h7
  //@req-spec-rename.h19
  With `numArchRegs = 1` and `maxGroupSize = 1` all three collapse at elaboration:
  `map_table` IS the VL space's current-PRN pointer (the renamed `VL`) and
  `com_map_table` IS its single-entry committed map table, the row decode becomes
  constant 0, `emul` constant 1, and every group construct below folds away. No
  separate VlMapTable module exists and none should be written. That single
  committed entry is also what makes a per-uop `stale_pvl` field unnecessary: it
  already holds the PRN each new producer displaces.

  ---- The EMUL-wide group read ----

  //@req-spec-rename.d8
  //@req-spec-rename.d9
  A source group read is `maxGroupSize` adjacent row reads: `pvs1(m) =
  map_table(lvs1 + m)` for `m < emul`, identically for `pvs2`, `pvs3` and
  `stale_pvdest`. Members at `m >= emul` are don't-care and consumers must ignore
  them (`v_emul` says how many are valid). Because each row independently names one
  PRN, the read returns the group's CURRENT mappings DIRECTLY and is correct under
  ARBITRARY FRAGMENTATION — the members returned need bear no relation to each
  other in PRN space, and typically do not.

  // RVV requires a group's base register to be a multiple of EMUL, so `lvs + m`
  // needs no adder: row i is in the group iff i's bits above log2(emul) match the
  // base's, and the member index is i's low log2(emul) bits. Implement the read as
  // a mux over the four legal member counts (1, 2, 4, 8) rather than maxGroupSize
  // dynamic adders — this read is on the rename critical path.

  A fractional EMUL (1/2, 1/4, 1/8) is a member count of 1; the mapper sees only
  the count, never the fraction, so it needs no case for it.

  //@req-spec-rename.a10
  These four reads plus the mask read ARE the mapper's renaming of
  `lvd`/`lvs*`/`lvm` to `pvdest`/`pvs*`/`pvm`: sources and the stale destination
  come from the reads, and `pvdest` is the freshly allocated group arriving on
  `remap_reqs`, which this table installs and passes through unmodified. `pvm` is a
  SINGLE row read, `map_table(lvm)` — the mask is one register (architecturally
  `v0`), never a group, so it gets no member loop and no `emul`.

  //@req-spec-rename.d14
  `v_emul` is echoed from the request onto `map_resps` in the same cycle and
  written into the OP.v's `v_emul` field. EMUL itself is derived at DECODE from the
  VCFG mirror; the mapper does not recompute it. Routing it back out through the
  map response rather than straight from decode to dispatch makes exactly ONE
  structure the authority on how many members of `pvdest`, `pvs*` and
  `stale_pvdest` are meaningful — a second opinion on the member count is the
  cheapest possible way to read seven valid PRNs and one garbage one.

  ---- The stale destination read ----

  //@req-spec-rename.d3
  //@req-spec-rename.d4
  //@req-spec-vrf.i1
  //@req-spec-vrf.i2
  //@req-spec-vrf.i3
  The lane reads its OWN destination specifier `lvd` through that same structure,
  and the result goes into the OP.v's `stale_pvdest` field as a `Vec` of up to EMUL
  stale PRNs — not a single register, since a vector destination renames a whole
  group and, allocation being non-contiguous, base-plus-count could not name the
  group actually mapped. The read is taken BEFORE this lane's own remap, so it
  returns the ARCHITECTURAL OLD-`vd` GROUP: under renaming `pvdest` is a FRESH group
  and the destination's previous contents live in `stale_pvdest`. That is what makes
  `stale_pvdest` the only legitimate source of undisturbed lanes (`vta = 0` tail,
  `vma = 0` masked-off, any `vstart > 0` prefix) and what lets commit free the group.

  //@req-spec-rename.d6
  This capture happens IN THE SINGLE RENAME CYCLE, in parallel with scalar rename —
  not in a second stage, not a cycle later: one more EMUL-wide read of the same
  table concurrent with the three source reads, with no second table, no dependency
  on the scalar side and no pipeline register between the read and the OP.v.

  //@req-spec-vrf.j7
  //@req-spec-vrf.j8
  `pvs3` and `stale_pvdest` come from TWO INDEPENDENT reads at two independent
  specifiers, `lvs3` and `lvd`. For read-modify-write arithmetic the third source IS
  the old destination — `vfmacc.vv vd, vs1, vs2` computes `vd += vs1 * vs2` — so
  `lvs3 == lvd`, both reads return the same PRNs, and the two fields name the same
  group with no special case. When an op needs old-`vd` for merging but its third
  source is something else or absent (a masked `vadd.vv` under `vma = 0`, a
  `vslideup` prefix, a `vcompress` tail) the specifiers differ and the reads return
  different groups, again with no special case. The table compares `lvs3` and `lvd`
  NOWHERE and must never collapse the two responses: the coprocessor decides which
  slots to pull, and pulls both when they differ.

  ---- In-bundle prefix bypass ----

  Rename is in program order within a bundle, so lane i must see the mappings
  installed by lanes 0..i-1 in the same cycle. As in the scalar table each read is a
  `foldLeft` over the older lanes' `remap_reqs`, taking the older lane's `pvdest`
  where it overrides the row read. The `stale_pvdest` read is bypassed identically:
  two ops writing the same architectural vreg in one bundle means the younger one's
  stale group is the older one's `pvdest`, and getting that wrong frees a live group.

  // ===> THE BYPASS COMPARE IS PER MEMBER AND AGAINST A RANGE, NOT AGAINST THE
  // BASE. An older LMUL=8 write to v0..v7 must bypass into a younger LMUL=2 read at
  // v4, delivering that write's members 4 and 5. Comparing `remap.lvd` to
  // `map_req.lvs*` — the scalar table's test, correct there because a scalar mapping
  // is one register — misses every sub-range read and the younger op issues against
  // a stale PRN pair. Per read member m the test is "does row (lvs + m) fall inside
  // older lane k's destination group", and the bypassed value is that lane's
  // `pvdest(row - remap.lvd)`. Same sub-range argument the group-done wakeup rests on.

  //@req-spec-rename.h20
  For the VL instance that same prefix bypass lets a `vset` and its dependent share
  a dispatch group, and the same construction on the COMMITTED side advances VL's
  committed pointer: at commit of a VL producer the `com_remap_req` for the single
  ARN installs the producer's new PRN, so the committed pointer is advanced to the
  new PRN that cycle. With `exportComStale` set, `com_stale_resps` presents the
  value that install DISPLACES, before the update — the PRN VecFreeList frees.
  Reading it from the committed table here is what keeps a `stale_pvl` field out of
  every ROB entry.

  ---- The group write ----

  //@req-spec-rename.d2
  //@req-spec-rename.i15
  Installation is ATOMIC PER LMUL/EMUL GROUP: a lane with `valid` set writes all
  `emul` rows `lvd .. lvd + emul - 1` in one cycle from `pvdest(0..emul-1)`. No state
  exists in which part of a group is installed, and no lane may install a partial
  group — VecRenameSpace stalls the entire bundle when the free list cannot supply a
  whole one, so a partially-mapped group is unrepresentable rather than merely
  avoided. Build the write as the scalar table does, generalized from a bit to a
  member: `remap_table` is a `Wire(Vec(plWidth + 1, Vec(numArchRegs,
  UInt(pregSz.W))))` from a `scanLeft` per row over the lanes, lane k overriding row
  i when row i is inside lane k's destination group (the range test above) with value
  `pvdest(i - lvd)`; `com_remap_table` is the same over `com_remap_reqs`. Normally
  `map_table` takes `remap_table(plWidth)` — the vector RMT simply ADVANCES THROUGH
  ITS REMAP REQUESTS, no recovery source involved — and `com_map_table` always takes
  `com_remap_table(plWidth)`.

  //@req-spec-core.h4
  //@req-spec-rename.e10
  `pvtmp` IS NEVER INSTALLED IN THIS TABLE. The temp group a shared (segmented)
  instruction allocates is an ordinary VRF group with a real busy lifetime, but it
  has no architectural name — no ARN to install it under, no reserved row, no third
  remap port. The binding lives only in the OP.v's own `pvtmp` field: both halves
  derive from that one uop and one ROB entry, so each half's issue slot carries the
  member PRNs directly. Excluding it therefore costs nothing and buys the guarantee
  that NO ARCHITECTURAL READ CAN ALIAS IT — a `pvtmp` row would be reachable by an
  ordinary `lvs*` read, making the rendezvous buffer readable as program state.

  ---- Snapshots and recovery ----

  //@req-spec-rename.i9
  //@req-spec-rename.i10
  Reuse BOOM's branch snapshot mechanism unchanged in structure: on
  `ren_br_tags(i).valid` write `remap_table(i)` into `br_snapshots(tag)`, one
  speculative-table copy per outstanding branch up to `maxBrCount`. Honour the
  existing `enableSuperscalarSnapshots` both ways — when false, assert at most one
  valid per cycle and select the table with a `Mux1H`, as the scalar table does.
  Reusing the mechanism instead of adding a vector-private one keeps the tables
  provably in step; a separate one would have its own tag allocation to get wrong.

  //@req-spec-rename.i6
  Snapshots are taken on the SAME `ren_br_tags` event as the scalar RMT and the
  restore is indexed by the SAME `br_tag`. No delayed-`br_tag` path, no
  vector-private tag space: all three tables update in the same cycle, so skew
  between their snapshot events would be a bug with nothing to hide it.

  //@req-spec-rename.i5
  //@req-spec-rename.i7
  //@req-spec-rename.h8
  Recovery is a three-way priority on the `map_table` write: on
  `brupdate.b2.mispredict`, `map_table := br_snapshots(brupdate.b2.uop.br_tag)`, a
  whole-table restore in ONE CYCLE flushing only state younger than the branch; else
  on `rollback`, `map_table := com_map_table`, a single-cycle copy flushing
  everything in flight; else the normal advance above. The exception path is a COPY,
  never a ROB one-entry-per-cycle walk-back — the committed table already holds the
  newest correct mapping before the trapping instruction, so a walk could only reach
  the same answer more slowly. For the VL instance these are the same two arms one
  entry wide: the pointer restores from its snapshot on mispredict, from the
  committed pointer on exception or flush.

  //@req-spec-rename.i11
  NO PERIODIC SNAPSHOT, and no snapshot on any event but `ren_br_tags`. A periodic
  checkpoint belongs to designs that must reconstruct group structure after a
  rollback; a per-ARN table has none to reconstruct, so it would be pure area plus a
  second, rarely exercised recovery path.

  ---- The structures that are deliberately absent ----

  //@req-spec-rename.d10
  NO CONTIGUOUS-RUN ALLOCATOR. Nothing here requires a group's PRNs to be adjacent
  or even ordered: rows are read and written independently and the group travels as
  an explicit member vector. VecFreeList accordingly selects with BOOM's existing
  `SelectFirstN` and its members are non-contiguous by design. Any code here
  computing a member PRN as base + m rather than reading `pvdest(m)` reintroduces
  the requirement silently.

  //@req-spec-rename.d11
  NO WHOLE-GROUP VALIDITY CHECK. No signal, register or comparator asks whether a
  read returned a whole group, because a per-ARN table cannot return anything else:
  every row read is a live mapping, whatever wrote it at whatever LMUL. Such a check
  would have no failure case to report and no recovery to trigger.

  //@req-spec-rename.d12
  NO FRAGMENTATION-RECOVERY WALK. No recovery path here iterates — both restores
  are single-cycle whole-table writes. Fragmentation is not a state this table can
  be in, so there is nothing for a walk to repair.

  //@req-spec-rename.d13
  NO LMUL TAG WHOLE VECTOR GROUP CHECKER: no 32x2-bit per-ARN tag array, no
  per-LMUL base-ARN comparator tree, no tag write on the remap path. If that
  observability is ever wanted, add a per-ARN "last-write EMUL" PERFORMANCE COUNTER
  — architectural state in a `perfEvents` EventSet, off the rename critical path —
  not a checker whose result nothing consumes.

  ---- Assertions and trace ----

  Carry over the scalar table's duplicate-mapping assertion, generalized to
  members: for every valid `remap_req` member PRN, assert that PRN is not already in
  `map_table` — the cheapest detector for a free-list double-allocation, the M1 bug
  class most worth catching early. It is an `assert`, simulation-only in the emitted
  Verilog, and must feed no functional signal: the comparison is `numArchRegs *
  plWidth * maxGroupSize` wide. Suppress it for duplicate identity mappings shortly
  after reset, as the scalar table does, and also assert `1 <= emul <= maxGroupSize`
  and `lvd + emul <= numArchRegs` on every valid request.

  There are no unit tests in this design — validation is end-to-end VCS plus Whisper
  cosim — so emit guarded VecTrace lines, gated on the `vecTrace` plusarg, off by
  default: one per remap (`tracePrn` with `rob_idx`, `lvd`, `emul`, installed
  members), one per stale-group capture, one per recovery event naming the arm that
  fired and the `br_tag`. The recovery line matters most: a mapping that survived a
  mispredict it should not have is invisible in a waveform until many cycles later,
  in another module.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput: `plWidth` group renames per cycle, sustained, with no stall of this
module's own. Latency: reads COMBINATIONAL, writes registered — the map response
must be valid in the SAME cycle as the request, because scalar and vector rename run
in parallel in one cycle and the dispatch group leaves rename together. A pipeline
register here would reintroduce the two-stage rename this design exists to delete,
along with all of its alignment machinery.

THIS MODULE SITS ON THE DESIGN'S PRINCIPAL TIMING RISK. Per cycle: `plWidth * 4`
EMUL-wide reads (three sources plus the stale destination), `plWidth` mask reads,
each with a prefix-bypass fold over the older lanes, and `plWidth` group writes of
up to `maxGroupSize` rows. The bypass fold, not the storage, is the long path —
`O(plWidth)` deep with a range comparison per member per stage. So keep member
selection a mux over the legal counts (1, 2, 4, 8) rather than a dynamic add per
member, and do not add the deleted checker back as a "cheap" comparator or widen the
fold with any comparison the reads do not need. The plan calls for a timing spike on
this module plus the busy table BEFORE the rename integration step lands, so the
fallback (splitting rename into two stages again) stays a known-cost decision.

Area: `numArchRegs * pregSz` bits speculative, the same again committed, and
`maxBrCount * numArchRegs * pregSz` bits of snapshots — the last term dominates and
is the price of single-cycle branch recovery.
<|end_perf|>

<|begin_dependencies|>
VectorParams — `numVecPhysRegisters`, `numVlPhysRegisters`, `maxMembers` and the
derived `vecPregSz`/`vlPregSz`. Every width here derives from these.
VecBundles — `VecMapReq`, `VecMapResp` and `VecRemapReq` are declared there, not
here, because each is a contract with VecRenameSpace on the other side.
VecTrace — the guarded `trace`/`tracePrn` helpers.

Binds to baseline BOOM declarations without modifying them: `BrUpdateInfo` and
`brTagSz` for the branch ports, `maxBrCount` and `enableSuperscalarSnapshots` for
the snapshot array, `lregSz` for the specifier widths, `BoomModule`/`BoomBundle` for
the implicit `Parameters`.

Instantiates nothing. Its only parent is VecRenameSpace, which instantiates it as
`maptable` in each of its two instances and owns the drive of `remap_reqs` from
VecFreeList, of `com_remap_reqs` from the ROB commit ports, and the whole-bundle
stall that keeps group installation atomic. This module must not reach around it to
either.
<|end_dependencies|>

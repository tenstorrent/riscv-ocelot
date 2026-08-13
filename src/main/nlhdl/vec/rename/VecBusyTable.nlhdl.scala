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
  VecBusyTable — the per-PRN readiness bit vector of one renamed vector-side
  register space, set on group allocation and cleared by group-done.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/rename/VecBusyTable.scala,
  package boom.v4.vec.generated.rename, group vec_rename.
  depends_on VecBundles, VectorParams, VecTrace.

  Instantiated by VecRenameSpace as `busytable`, and VecRenameSpace itself has
  TWO instances (VecPipeline: `vec_rename`, `vl_rename`), so this module is
  elaborated twice: once as the VECTOR busy table over `numVecPhysRegisters` with
  groups of up to `maxMembers`, and once as the VL busy table over
  `numVlPhysRegisters` with a degenerate group size of 1.

  It is the vector analogue of `RenameBusyTable`
  (src/main/scala/v4/exu/rename/rename-busytable.scala), which stays untouched
  and keeps serving INT and FP. Read that file alongside this spec: the
  three-part shape is deliberately the same, and every DIFFERENCE below exists
  because a vector operand is a GROUP, not a register.

  ===> TWO THINGS A READER MUST NOT GET WRONG.
       (1) The table is indexed PER MEMBER PRN. No group-base bit, no
           base-plus-size encoding, anywhere in this module. A consumer may read
           a source group that is a SUB-RANGE of a larger in-flight destination
           group — an LMUL=8 write to v0..v7 then an LMUL=2 read at v4 sources
           {p4, p5}, not the producer base p0 — and a group's members may come
           from different producers, so a base-only bit would wake that consumer
           early on a stale read.
       (2) `busy_resps` carries the per-member bits AND-ed into ONE group-ready
           bit per operand. That aggregation is for the RENAME-STAGE read only.
           It does NOT replace the issue-slot wakeup, which stays PER MEMBER and
           is VecGroupReady's job inside the slot. Two consumers of the same
           completion event at two granularities, on purpose.
           AMENDED (seam review A2): the PRE-REDUCTION per-member bits ALSO leave
           this module, on a SECOND output `member_busy_resps` gated by
           `exportMemberRdy`, because VecGroupReady must INITIALIZE its per-member
           state and the aggregate would hang it (part 5b). `busy_resps` keeps its
           aggregated-only shape; the second port is a fan-out of wires the
           per-member read of `rename.g11` already computes, not a second read.

  Governing spec anchors: midcore.rst `busy-table` and `group-done` (the
  set/read/clear model and every width), `vl-vtype-rename` (the VL instance),
  `cii-shared-mapping` and `regfiles-bypass` (pvtmp is an ordinary group),
  glossary.rst `glossary-terms` (pvtmp).

<|begin_module|>

  <|begin_parameters|>
  Six constructor parameters, all Scala `Int`/`String`/`Boolean` resolved at
  elaboration. No width below may be written as a literal — each comes from
  VectorParams or from `HasBoomCoreParameters`.

  `plWidth` — rename lanes served, i.e. `coreWidth`. Default 3, legal 1..4 (the
  BoomConfigs tiers). It multiplies the source-read port count directly.

  `numPregs` — size of the space this instance covers: `numVecPhysRegisters`
  (default 96) for the vector instance, `numVlPhysRegisters` (default 64) for the
  VL one. Derived: `pregSz = log2Ceil(numPregs)`.

  `maxGroupSize` — largest member count of one group: `maxMembers` (fixed 8) for
  the vector instance, 1 for the VL instance. Legal values 1 and powers of two up
  to `maxMembers`.

  `numWbPorts` — completion ports on the clear side: `numVecWbPorts` for the
  vector instance, the VL writeback port count for the VL one. This is the width
  of the vector wakeup network, and it is also the multiplier on every per-member
  comparator in every issue slot, so both must be sized from this one name.

  `wakeupKind` — a `String`, "group_done" or "ready_bit", passed down from
  VecRenameSpace unchanged; it selects the clear-port shape (see the ports
  section). Kept as an explicit named parameter rather than derived from
  `maxGroupSize == 1` for the reason hierarchy.yaml gives on VecRenameSpace: free
  discipline and wakeup kind are independent decisions and deriving one from the
  other silently couples them. Require one of the two strings, else fail
  elaboration.

  `exportMemberRdy` — Boolean, default FALSE, true for the vector instance only
  (VecRenameSpace passes its own identically-named parameter straight down). It
  gates the second output `member_busy_resps` and the `stale_pvdest` read that
  feeds its fifth group. A PRESENCE gate in exactly the sense of VecMapTable's
  `exportComStale`: with it false the port and the extra read do not exist, so the
  VL instance carries no dead wires. `require(!exportMemberRdy || maxGroupSize > 1)`
  — a one-member group's per-member vector carries nothing the aggregate does not,
  and the VL space's readiness reaches execute as a value, not as member matching.
  Not derived from `maxGroupSize > 1`, for the reason the paragraph above gives.

  Elaborated only when `usingRVV` is true — a Scala `Boolean` from
  `BoomCoreParams`, NOT a hardware `Bool` and NOT rocket's `usingVector`. With
  vectors off the module is ABSENT, not instantiated-and-tied-off, so a
  non-vector build emits RTL bit-identical to pre-Caracal BOOM v4. Require
  `numPregs >= 32 + maxGroupSize` on the vector instance so at least one whole
  group is representable; the tighter capacity requires live in VectorParams.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel and hierarchy.yaml defaults: one `core_clk`
  domain, POSEDGE clock, ACTIVE-HIGH SYNCHRONOUS `core_reset`, both implicit via
  `BoomModule` rather than declared ports. Reset initializes the busy vector to
  all-zero — every PRN ready — because no producer is in flight at reset and a
  table coming up all-busy would deadlock the first vector consumer.

  //@req-spec-rename.g22
  `ren_uops` — Input, `Vec(plWidth, new MicroOp)`. One uop per lane carrying the
  GROUP MEMBER PRNs plus EMUL, not a base and a size: `pvdest`, `pvs1`, `pvs2`,
  `pvs3`, `pvtmp` and — only when `exportMemberRdy` — `stale_pvdest` are each
  `Vec(maxMembers, UInt(vecPregSz.W))`, `pvm` is a
  single PRN, `pvl` a single VL PRN, and `v_emul` the 1..8 member count. Member
  PRNs and not a base because the free list allocates a group WITHOUT requiring
  contiguous PRNs, so base-plus-count could not name the group actually
  allocated. Only these fields are read; the rest of the uop is ignored here.

  //@req-spec-rename.g23
  `rebusy_reqs` — Input, `Vec(plWidth, Bool)`. One bit per lane: this lane
  allocated a destination group and its members must be marked busy. One request
  per OP.v setting up to 8 bits, not one request per member — the group is
  renamed atomically and a partial group is never allocated. VecRenameSpace
  drives it from `ren2_alloc_fire`, the same event the free list allocates on, so
  the set and the allocation cannot disagree.

  //@req-spec-rename.g21
  `busy_resps` — Output, `Vec(plWidth, new VecBusyResp)`. Per-lane, PER-GROUP
  source readiness ALREADY AGGREGATED TO GROUP-READY BITS: one `Bool` per
  operand, never a per-member vector. Vector instance fields: `pvs1_busy`,
  `pvs2_busy`, `pvs3_busy`, `pvm_busy`, `pvtmp_busy`. VL instance: the single
  field `pvl_busy`. Names match the `MicroOp` busy fields exactly so
  VecRenameSpace's connection is field-for-field and cannot be mis-paired.
  `VecBusyResp` is declared in this file, not VecBundles, because it crosses only
  the boundary to this module's own parent — the same locality decision baseline
  BOOM makes with `class BusyResp`. It gains NO field from the amendments below:
  there is no aggregated `pvold_busy` here, deliberately (part 10).

  //@req-spec-rename.g11
  `member_busy_resps` — Output, `Vec(plWidth, new VecMemberBusyResp)`, elaborated
  only when `exportMemberRdy`. AMENDMENT per seam review A2 and decision D6. It is
  the SAME per-member read as `busy_resps`, tapped BEFORE the AND-reduction, in
  BUSY sense like its sibling. `VecMemberBusyResp` is declared in this file beside
  `VecBusyResp`, for the same locality reason, with fields
    `pvs1_busy`, `pvs2_busy`, `pvs3_busy`, `pvtmp_busy`, `pvold_busy`
      — each `Vec(maxGroupSize, Bool)`, member `j` of that source group; and
    `pvm_busy` — a single `Bool`, because `pvm` is one register and never a group.
  Member slots at or beyond `v_emul` are driven CLEAR (not busy), not left
  don't-care. The consumer masks by `members` anyway — VecGroupReady is
  parameterised by the group's member count and never inspects a slot above it —
  but the asymmetry matters: a stale BUSY bit in an unused slot HANGS a matcher
  that forgot to mask, while a clear bit in a slot nobody reads is harmless. Same
  fail-safe direction as VecFreeList padding a short group with member 0 instead of
  PRN 0.
  Sense is BUSY here and READY at VecRenameSpace's `member_rdy`, which inverts
  these bits and ORs in its own in-bundle prefix hit. Two names, two senses, one
  conversion site — do not also invert here.
  `pvold_busy` is the per-member readiness of `stale_pvdest` (D6): `IQ_V_LOAD`
  (the LCB pre-loads from it on R2 when vta=0/vma=0) and `IQ_V_ALU` (the
  coprocessor may pull STALE_VD) gate issue on it through a FIFTH VecGroupReady
  instance, `rdy_vold`. See part 10 for why it exists ONLY per member.

  //@req-spec-rename.g24
  //@req-spec-rename.g25
  `wakeups` — Input, `Vec(numWbPorts, Valid(...))`: exactly `numWbPorts` wide, no
  wider, and the same bus the issue slots match on. With `wakeupKind =
  "group_done"` each port carries `VecGroupDone` from VecBundles, whose payload is
  the COMPLETING GROUP'S MEMBER-PRN VECTOR — `Vec(maxMembers, UInt(vecPregSz.W))`
  plus a `members` count and the owning `rob_idx`. With "ready_bit" the member
  vector degenerates to one PRN and the port is `Valid(UInt(pregSz.W))`, the PRN
  the producer's VL writeback completed. Both shapes share all the clear logic
  below; only member-list extraction differs.

  `debug.busytable` — Output, `Bits(numPregs.W)`: raw state for waveform
  inspection only, nothing functional may read it. Mirrors the identically-named
  port of the scalar `RenameBusyTable` so both tables look the same in Verdi.

  There is deliberately NO port for any of the following, and a reviewer should
  reject each if it appears: a `brupdate`/flush input (see part 8), a
  `child_rebusys` input or any speculative rebusy path, a per-PRN clear port
  beside the group-done ports, a `busy` output of any kind, and an AGGREGATED
  `stale_pvdest` busy bit in `VecBusyResp` (see part 10 — the per-member
  `pvold_busy` above is the whole of what D6 permits).
  <|end_ports|>

  <|begin_logic|>
  ---- 1. State ----

  //@req-spec-rename.g1
  //@req-spec-rename.g6
  //@req-spec-rename.g7
  One register `busy_table`, `RegInit(0.U(numPregs.W))`: the readiness status of
  EACH physical register in this space, bit `p` set meaning "PRN p has an
  in-flight producer and is not yet readable". Readiness is tracked PER MEMBER
  PRN, not by a group base, so the storage is a flat per-PRN bit vector over
  `numPregs` — 96 bits on the vector instance at the default sizing — with no
  group structure in it at all. Group structure exists only in the ADDRESS lists
  that index it, which arrive on the ports above.

  Flat, not Vec(numGroups, Bits): a group's members are non-contiguous PRNs
  from the free list, so no grouping of the storage could match a group that
  is actually allocated.

  ---- 2. Set-busy on allocation ----

  //@req-spec-rename.g8
  //@req-spec-rename.g9
  //@req-spec-rename.g10
  Per lane `i`, form the set mask as the OR over members `j` of
  `UIntToOH(io.ren_uops(i).pvdest(j))`, qualified by `io.rebusy_reqs(i)` and by
  `j < io.ren_uops(i).v_emul`. ALL member bits of the destination group are set —
  up to 8 per OP.v — in the one cycle the group is renamed. The full mask is the
  OR of the `plWidth` lane masks, so the set-busy path is up to `plWidth *
  maxGroupSize` = `coreWidth*8` bits per cycle wide. Structurally this is the
  free list's allocation path: a bit-vector OR, so 8 bits per OP.v costs no new
  logic beyond the wider port. A shared instruction sets TWO groups this way,
  `pvdest` and `pvtmp` (part 6).

  The j < v_emul qualifier keeps an EMUL=1 op from marking 7 unrelated PRNs
  busy. Do NOT instead rely on unused pvdest members being zero: PRN 0 is a
  real allocatable vector PRN, so a stale member would mark it busy forever
  and hang the first consumer of whatever op later owns it.

  ---- 3. Clear-busy on group-done ----

  //@req-spec-rename.g15
  //@req-spec-rename.g18
  //@req-spec-rename.g16
  //@req-spec-rename.g17
  Per wakeup port `w`, form the clear mask as the OR over that port's members of
  `UIntToOH(member_prn)`, qualified by `io.wakeups(w).valid` and by the port's
  `members` count. A producer emits exactly ONE group-done per OP.v — the CII
  once per instruction, the LCB once after the last destination PRN lands — and
  that single event carries the completing group's FULL MEMBER-PRN VECTOR, so
  this module clears the busy bits of ALL of its member PRNs in that one cycle.
  No per-PRN completion port and no per-entry counter exists on this path. The
  clear side is therefore `numWbPorts` by up-to-8 bits wide: `numVecWbPorts` ×
  `maxGroupSize`.

  ONE EVENT, THREE CONSUMERS: the ROB's single-shot rob_bsy clear, this
  table's clear, and the vector wakeup network into the issue slots — same
  group-done, same cycle. Do not re-time this clear against the ROB's: a table
  clearing a cycle later would let a dependent's rename read busy for a group
  whose producer the machine has already retired.

  ---- 4. Next state, and set beats clear ----

  `busy_table_clr` is `busy_table & ~(OR of the per-port clear masks)`;
  `busy_table_next` is `busy_table_clr | (OR of the per-lane set masks)`. SET
  WINS over CLEAR in the same cycle, stated as an ordering rather than left to
  expression order: a set is a PRN acquiring a NEW producer this cycle, and a
  clear from its previous owner winning would bring the new group up ready with
  nothing written.

  Unlike the scalar `RenameBusyTable` there is NO `RegNext` on the wakeup ports,
  no `speculative_mask`/`child_rebusys` cancellation, and consequently no rebusy
  term in `busy_table_next` at all. Vector operands are never woken
  speculatively — a group's readiness is only ever asserted by a real group-done
  — so the speculative-wakeup machinery the scalar table needs for load-hit
  speculation has nothing to cancel here. That deletion is why this module is
  materially smaller than its scalar sibling despite the wider ports; the policy
  itself is stated on VecGroupReady, which owns it.

  ---- 5. Source reads and per-operand aggregation ----

  //@req-spec-rename.g11
  Per lane `i` and per source group, read the busy bit of EACH MEMBER:
  `busy_table(io.ren_uops(i).pvs1(j))` for every `j` in `pvs1`, likewise `pvs2`,
  `pvs3`, and the mask `pvm` — which is a single register, never a group, so it
  contributes exactly one read. That is 3 × 8 + 1 = 25 bit-reads per lane for the
  encoded sources, times `plWidth`, and it is the dominant cost of this module.
  When `exportMemberRdy`, `stale_pvdest` is read the same way, per member — that is
  a REAL added read group (+8 per lane), not a fan-out, and it is the one part of
  D6 that costs something in this module. `pvtmp` adds 8 more (part 6).

  Aggregate each group's per-member bits into ONE GROUP-READY BIT per operand by
  AND-ing the members' readiness (equivalently OR-ing their busy bits into one
  group-busy bit), masking out members at or beyond `v_emul` so an absent member
  neither blocks nor falsely readies the operand. The operand is ready only when
  its LAST member is ready. Drive `busy_resps(i).pvs1_busy` and its peers from
  those reductions. `stale_pvdest` is NOT aggregated and reaches `busy_resps`
  nowhere (part 10).

  Reads are BYPASSED against the clear masks of part 3: a member whose PRN
  matches a group-done firing this cycle reads READY. Without the bypass the
  dependent stalls an extra cycle for nothing, and worse on the vector side a
  whole operand would report busy because one member's clear had not yet landed
  in the register. With no speculative rebusy the bypass is a plain force to
  ready, not baseline's `Mux1H` over the ports' rebusy bits.

  Because both the tracking and the reads are per member, a consumer whose source
  group is a SUB-RANGE of a larger in-flight destination group needs no extra
  logic: its members are exactly the sub-range's PRNs and they go ready when that
  producer's group-done fires. Conservative — the whole group readies together —
  but never early. The corresponding per-member match at the issue slot belongs
  to VecGroupReady, not here.

  This module does NOT do the in-bundle prefix bypass for a younger lane
  depending on an older lane of the same dispatch group: that is
  VecRenameSpace's `BypassAllocations` equivalent, which ORs its term into the
  uop's busy field after reading this table, as `rename-stage.scala` lines
  181-183 do. Doing it here too could only mask a bug in the other one.

  ---- 5b. The per-member export (AMENDMENT: seam review A2, decision D6) ----

  //@req-spec-rename.g11
  When `exportMemberRdy`, drive `member_busy_resps(i)` from the SAME bypassed
  per-member wires the reductions above consume, tapped BEFORE the AND-tree:
  `pvs1_busy(j)`, `pvs2_busy(j)`, `pvs3_busy(j)`, `pvtmp_busy(j)`, `pvold_busy(j)`
  from `stale_pvdest`, and the single `pvm_busy`. No new comparator, no new state,
  no second read of `busy_table`, and no re-timing: if a generator emits a separate
  indexed read for this port it has doubled the read multiplexing that part 5's
  budget calls this module's dominant cost. `busy_resps` is unchanged — the two
  outputs are the SAME data at two granularities, so they can never disagree, and
  that is the point of taking one from the other rather than computing both.

  The ONE term this port adds is the `j < v_emul` qualifier that drives slots at or
  above the member count CLEAR. The raw read of a padded slot is member 0's PRN's
  bit, which can legitimately be BUSY, so passing it through would advertise a busy
  member that does not exist. Part 5's reduction masks the same slots for the same
  reason — one term, applied on both paths, not a new comparator.

  ===> WHY THE AGGREGATE ALONE HANGS THE MACHINE. A source group's members can
  come from DIFFERENT producers, so a group can be not-ready in AGGREGATE while
  members 0..2 are already done. VecGroupReady INITIALIZES its per-member state
  from `in_member_rdy` and thereafter only ORs in matches against live
  group-dones. Load the aggregate into all members and the slot waits for
  group-dones that ALREADY FIRED and will never fire again — a permanent hang,
  with no assertion. Two nodes (VecIssueSlot, VecGroupReady) depend on this port
  existing; the only alternative is a second busy table.

  With `exportMemberRdy` false the port and the `stale_pvdest` read are absent, so
  the VL instance is exactly the module part 7 describes.

  ---- 6. pvtmp is an ordinary operand with a real busy lifetime ----

  //@req-spec-core.h3
  //@req-spec-rename.e7
  //@req-spec-rename.e12
  //@req-spec-vrf.d6
  `pvtmp` — the rendezvous group of a shared (segmented) instruction — is an
  ORDINARY VRF GROUP here and has a REAL BUSY LIFETIME. No separate temp register
  file, no separate temp busy table, and no special case in any path above:
    - the producer half's `pvtmp` members are SET busy by part 2, same cycle and
      same OR-of-`UIntToOH` structure as `pvdest`, qualified by the same
      `rebusy_reqs(i)` and additionally by `is_shared`;
    - the consumer half READS `pvtmp` per member in part 5 and aggregates it into
      `busy_resps(i).pvtmp_busy` alongside the encoded sources;
    - it is CLEARED by the producer half's group-done in part 3,
      indistinguishably from any other group-done — the two halves rendezvous
      through the VRF on this same busy-table and group-done machinery;
    - branch reclaim and the commit free alongside `stale_pvdest` are the free
      list's business and touch nothing here.
  The one asymmetry is that `pvtmp` is never installed in the vector map table so
  no architectural read can alias it — a VecMapTable property, not a busy-table
  one, and free for this module.

  Reading pvtmp takes the per-lane read count from 25 to up to 33, and
  `stale_pvdest` (part 5b, vector instance only) to up to 41. The spec's
  "~25 bit-reads per lane" counts the ENCODED sources only; pvtmp is the fifth
  operand, read on lanes whose uop is_shared.

  ---- 7. The VL instance, and where the pvl bit comes from ----

  //@req-spec-rename.h12
  //@req-spec-rename.h13
  //@req-spec-rename.h14
  With `maxGroupSize = 1`, `numPregs = numVlPhysRegisters` and `wakeupKind =
  "ready_bit"`, this same module IS the VL busy table: ONE BIT PER VL PRN, SET ON
  ALLOCATION of that PRN (part 2, with a single-member `pvdest` list — the
  freshly allocated `pvl`), CLEARED BY THE PRODUCER'S VL WRITEBACK arriving on a
  wakeup port (part 3, single-member list). Every reduction in part 5 degenerates
  to one bit, the AND-trees collapse, and no VL-specific code path is needed.
  `vsetivli` is the one producer whose `pvl` busy bit is never set — the VCFG
  computed VL at decode and the VL RF is written in the rename cycle, so `pvl` is
  born ready — which VecRenameSpace expresses simply by not raising
  `rebusy_reqs` for that lane.

  //@req-spec-rename.g12
  //@req-spec-rename.g26
  Each rename lane's busy read ALSO includes the `pvl` read, so a lane's full read
  is its four (or five) vector source groups plus one VL bit. That bit is read
  FROM THE VL BUSY TABLE — the `vl_rename` instance of this module — and NOT from
  the integer table and NOT from the vector one. VL is renamed into its own
  register space, so `pvl` indexes `VL_RF` and has exactly one busy bit, in that
  table. Concretely: the vector instance has no `pvl` read port and its
  `VecBusyResp` has no `pvl_busy` field; the VL instance answers
  `io.ren_uops(i).pvl_src` -- the READ PRN, never `pvl`, which on a `vle*ff.v` is
  that uOP's own destination -- from its own state and drives `pvl_busy`; VecRenameSpace
  joins the two responses when it writes the uop's busy fields.

  The M1 pvl busy-bit bugs came from reading pvl out of the wrong table. Two
  instances of one module with one read port each makes the mistake
  unspellable: neither instance has an index wide enough, or a state vector
  large enough, to answer the other's read.

  ---- 8. Misprediction and flush: no port, on purpose ----

  No `brupdate` and no flush input, matching the scalar `RenameBusyTable`. A
  wrong-path OP.v leaves its `pvdest` (and `pvtmp`) members set busy; those PRNs
  return to the free list through the branch reclaim lists, and the next
  allocation SETS them busy again while their new producer's group-done CLEARS
  them. A stale busy bit on a free PRN is self-correcting and never observable:
  nothing reads a PRN's bit unless some uop names it, and naming it requires
  having allocated it.

  ASSUMPTION TO CHECK AT BRING-UP: the wakeup ports carry no wrong-path
  group-done for a PRN already reallocated. Vector completions are killed at
  their source (VecCiiFlush answers and drops killed tags; the LSU squash path
  drops killed element accesses) and rename does not restart inside the flush
  window. If that stops holding, the fix belongs at the producer — a
  flush-clear of the whole table would also clear surviving older producers.

  ---- 9. Trace and assertions ----

  There are NO unit tests in this project — no chiseltest, no per-module spec
  class — so guarded tracing is the debug surface. Emit one `VecTrace` line per
  key event, gated on the `vecTrace` plusarg and `!reset`, off by default: "set"
  with the lane's `pvdest` member list and `v_emul`, "clr" with the completing
  port's member list, "read" with the aggregated group-ready bits. Every line
  carries `rob_idx` so it correlates with the ROB's busy-clear line and with the
  Whisper cosim trace for the same instruction. Emit-only: no register, no
  counter that functional logic reads.

  Assert that a group-done never clears an already-clear bit on the vector
  instance — that is a double completion for one OP.v and would corrupt a later
  owner of the PRN — and that no lane raises `rebusy_reqs` with `v_emul === 0.U`.

  ---- 10. stale_pvdest: read PER MEMBER, exported PER MEMBER, never aggregated ----

  The earlier revision of this section said there is no busy read for
  `stale_pvdest`, because midcore.rst `group-done`'s source-read list names
  pvs1/pvs2/pvs3 and pvm only, and reported the old-dest RAW hazard as needing a
  SPEC AMENDMENT rather than a local fix. DECISION D6 IS THAT AMENDMENT, and it
  lands here as part 5b's `pvold_busy`:

  `stale_pvdest` is the PREVIOUS mapping of the destination arch vregs, so its
  producer is an OLDER instruction — and age-ordered issue grants the oldest READY
  entry, which does not mean an older producer has finished. Two consumers read the
  group anyway: the LCB PRE-LOADS from `stale_pvdest` on R2 for `vta=0`/`vma=0`
  (`IQ_V_LOAD`), and the coprocessor may pull `STALE_VD` (`IQ_V_ALU`). Both
  therefore gate issue on a fifth `VecGroupReady` instance, `rdy_vold`, fed from
  this module's per-member export. This matches the prior M2 implementation, whose
  hang needed exactly this term.

  ===> AND A SINGLE AGGREGATE `stale_pvdest_busy` BIT IS THE TEMPTING WRONG
  ANSWER — it is UNSAFE, not merely conservative. `stale_pvdest` can span up to
  `maxGroupSize` DIFFERENT producers: an LMUL=1 op writes v0, then an LMUL=8 op
  renames v0..v7, so that op's stale group is the CURRENT mappings of eight arch
  vregs, installed by up to eight different instructions. One bit cannot express
  "waiting on producer 3 of 8" and one group-done cannot clear it correctly.
  That is the same argument that makes `pvs*` per-member (rename.g20), and it is
  why `VecBusyResp` gains no field: the aggregate would be a legal-looking bit
  that is wrong in the multi-producer case, which is the common case at LMUL>1.

  Still NOT read here, and not to be added by a generator: any per-PRN read
  driven by anything other than a `ren_uops` field, and any read on the VL
  instance beyond `pvl` (part 7).
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Fully combinational read, single-cycle state update, one pipeline stage: the
table is read and answered inside the same rename cycle VecRenameSpace allocates
in, so `busy_resps` must be valid combinationally from `ren_uops` and the current
`busy_table`. Registering the response would break the lockstep contract with the
scalar RenameStage's registered ren1-to-ren2 pipeline.

THIS MODULE SITS ON THE DESIGN'S PRINCIPAL TIMING PATH with VecFreeList and
VecMapTable — hierarchy.yaml names atomic group rename as the top timing risk,
and midcore.rst names this table's per-member match as its dominant cost. Budget
per cycle at the default tier (coreWidth 3, maxMembers 8, 96 PRNs): up to 24
one-hot decoders OR-reduced to a 96-bit set mask; `numVecWbPorts` × 8 decoders
likewise on the clear side; and up to 41 indexed bit-reads per lane × 3 lanes,
each bypassed against the clear mask, then AND-reduced in trees of depth
log2(8) = 3. The trees are shallow — the risk is read multiplexing over a 96-bit
vector at that fan-out. Run the timing spike hierarchy.yaml calls for on
VecRenameSpace BEFORE the rename integration step lands; if the vector side
loses, the whole core's rename stage pays.

41, not 33: `stale_pvdest`'s 8 members (D6) are the only NEW reads either
amendment adds. `member_busy_resps` itself costs NOTHING in this budget — it is
a fan-out of the bypassed per-member wires ahead of the AND-trees, so it adds
fan-out load and no depth, and it removes nothing either: `busy_resps` still
needs the reduction for the rename-cycle read.

No throughput target beyond one full dispatch bundle per cycle. This module must
never be the reason a bundle fails to rename, so it has no ready/valid handshake
and no back-pressure output of any kind.
<|end_perf|>

<|begin_dependencies|>
VecBundles — `VecGroupDone` for the "group_done" clear port; its member-PRN
vector plus `members` count is that bundle's contract and is not re-declared here.
VectorParams — `numVecPhysRegisters`, `numVlPhysRegisters`, `maxMembers`,
`vecPregSz`, `vlPregSz`.
VecTrace — the guarded trace helpers of part 9.
MicroOp, transitively through `ren_uops` — the `pvdest`/`pvs1`/`pvs2`/`pvs3`/
`pvm`/`pvtmp`/`pvl`/`stale_pvdest` group fields and `v_emul`. Note MicroOp is NOT in this node's
`depends_on:` list in hierarchy.yaml even though the port type needs it; the edge
arrives through VecRenameSpace, which does depend on it.

Instantiates nothing. Its only parent is VecRenameSpace, twice. That parent drives
`ren_uops` from a private `bt_uops` wire, so it — not this module — is responsible
for placing this space's freshly renamed destination in `pvdest` and the map
table's response in `stale_pvdest` before this table reads them.

`member_busy_resps` has THREE readers downstream of that parent, and they are the
reason the port exists: VecRenameSpace inverts it into `member_rdy`, VecIssueUnit
carries it as `dis_member_rdy`, and VecGroupReady loads it as `in_member_rdy` —
including its `rdy_vold` instance, which has no other source of `stale_pvdest`
readiness anywhere in the design.

The group-done it consumes is the SAME event consumed by the Rob delta's
`vec_clr_bsy` and by every VecGroupReady in every vector issue slot. A change to
how this module interprets a group-done is a three-way change; check those two
before touching it.
<|end_dependencies|>

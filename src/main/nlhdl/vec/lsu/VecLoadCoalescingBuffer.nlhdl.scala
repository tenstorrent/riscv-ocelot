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
  VecLoadCoalescingBuffer — the response side of the vector load path: it turns
  out-of-order, at-most-one-element (<= ELEN) D$ responses into ONE VRF write per
  destination PRN and ONE group-done per instruction.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecLoadCoalescingBuffer.scala,
  package boom.v4.vec.generated.lsu, group vec_lsu.
  depends_on VecBundles, VectorParams, VecTrace. Instantiated ONCE, as `lcb`, by
  VecLsu. Instantiates nothing.

  ===> BUG NOT TO RE-INTRODUCE (plan section 2, M1 bug 4): COMBINATIONAL LOOP.
       `group_done` — and every other output that leaves this module — must be
       driven from the REGISTERED beat and from REGISTERED entry state, never
       combinationally from the incoming nOP.v. The path
       `issue.vec_wakeup -> iss_uops -> decode -> AGEN -> nop -> group_done ->
       vec_wakeup` closes a loop, and the M1 LCB closed it exactly this way by
       computing `io.group_done.valid` from `io.beat.bits.last`. A fake / bypass
       entry (all-inactive member, zero active bytes) is therefore finalized one
       cycle LATER, not in its allocation cycle.

  ===> THE UNDISTURBED PRE-LOAD IS THE WHOLE POINT OF THIS MODULE'S SHAPE
       (plan section 2, structural change 4). `addvector` had no VRF read port
       for `stale_pvdest`, so it copied the whole stale group with a serial
       sCopyRd/sCopyWr prologue — 2 cycles per member, x8 members — in front of
       every masked or tail-undisturbed load, on the load's own critical path.
       Here the stale data is read on VRF port `R2` into the assembly entry while
       the load's own memory accesses are outstanding, and it lands in a byte set
       DISJOINT from the arriving elements, so the two need no ordering between
       them at all. Performance target P5 (a masked / tail-undisturbed load costs
       no extra cycles versus unmasked at equal active-byte count) lands here.

  Governing spec anchors: loadstore.rst `load-coalesce`, `elem-progress`
  (including "Fault-only-first (``vleff.v``)"), `vec-load-algo`, `vec-squash`;
  midcore.rst `vrf-ports`, `old-vd`, `group-done-wb`, `midcore-segmented-load`.

<|begin_module|>

  <|begin_parameters|>
  ---- ELABORATION GATE ----

  The entire module is elaborated only when `usingRVV` is true — a Scala
  `Boolean` derived from `BoomCoreParams`, not a hardware `Bool`, and NOT
  rocket's `usingVector`. With vectors disabled VecLsu does not exist, so this
  module is absent rather than tied off, and a non-vector build stays
  bit-identical to pre-Caracal BOOM v4.

  ---- Sizing ----

  `lcbEntries` — the number of VLEN-wide assembly entries, taken from
  `VectorParams.lcbEntries`, default 8 (one whole LMUL=8 destination group in
  flight). It is the module's one real knob.

  Require `lcbEntries >= maxMembers`. This is a liveness check, not a style
  check: entries are RETAINED until their whole group retires (see the logic
  section), so an entry array smaller than one destination group could not hold
  all the members of a single LMUL=8 load, and that load could never complete —
  a deadlock reached by undersizing, exactly as `ssiQueueEntries` can be
  undersized. Fail elaboration naming the offending parameter.

  `lsuWidth` — the number of D$ lanes, 1 on Small/Medium and 2 on Large/Mega.
  It sizes BOTH the response ports and the VRF write ports: `W0` only at
  `lsuWidth = 1`, `W0` and `W1` at `lsuWidth = 2`, per the canonical port table
  in midcore.rst `vrf-ports`. This module ADDS NO VRF PORT and must not.

  Widths used below, all derived and none written as a literal: `vLen`,
  `vLenBytes = vLen/8` (the per-byte mask width), `coreDataBits` (the D$ response
  width, <= `eLen`), `vecPregSz`, `vlPregSz`, `maxMembers`, `vecVLSz`,
  `ldqAddrSz`, `numLdqEntries`, `robAddrSz`.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel and hierarchy.yaml defaults: posedge
  `clock`, ACTIVE-HIGH SYNCHRONOUS `reset`, single `core_clk` domain. Both are
  used — this module is almost entirely registered state.

  - `io.alloc`        — Flipped(Decoupled(`LcbAllocReq`)). One allocation per
                        destination PRN, from the load drain (VecBeatExpander).
                        `ready` means an entry is free; see the credit protocol.
  - `io.free_count`   — Output, `log2Ceil(lcbEntries+1)` bits. The number of free
                        entries, published to VecDcacheArbiter so it can suppress
                        a load-drain grant in the same cycle the drain would need
                        a new entry.
  - `io.resp`         — Vec(`lsuWidth`, Flipped(Valid(`LcbBeat`))). Placed D$
                        responses. There is deliberately NO `ready`: the LCB may
                        never back-pressure a response (a blocked response holds
                        an MSHR against the very drain that would free the entry),
                        and after allocation-before-access it never needs to.
  - `io.vrf_write`    — Vec(`lsuWidth`, Valid{ `addr`: `vecPregSz`,
                        `data`: `vLen`, `mask`: `vLenBytes` }). VRF ports `W0`
                        (index 0) and, at `lsuWidth = 2`, `W1` (index 1). `mask`
                        is PER BYTE.
  - `io.stale_req`    — Output Valid(`vecPregSz`). VRF read port `R2`: the
                        `stale_pvdest` member read for an undisturbed pre-load.
  - `io.stale_resp`   — Input, `vLen` bits. The `R2` data, valid one cycle after
                        the request (VecRegFile reads are registered,
                        single-cycle).
  - `io.group_done`   — Output Valid(`VecGroupDone`, from VecBundles). ONE per
                        completing destination group.
  - `io.vl_wb`        — Output Valid{ `pvl`: `vlPregSz`, `vl`: `vecVLSz` }. The
                        `vleff` VL-register-file write.
  - `io.elem_done`    — Vec(`lsuWidth`, Valid{ `ldq_idx`, `nelem` }). Element
                        assembly progress, added into the owning LDQ entry's
                        `elem_done` cursor field.
  - `io.trim`         — Flipped(Valid(`LcbTrim`)). A `vleff` VL trim from the
                        element cursor.
  - `io.kill_ldq`     — Input, `numLdqEntries` bits. One bit per LDQ entry: an
                        assembly entry is invalidated when the bit for its owning
                        `ldq_idx` is set. VecSquashUnit computes it for branch
                        rollback, order-fail, trap and flush alike.
  - `io.empty`        — Output, 1 bit. No valid entries. Consumed only by
                        VecLsu's `vec_lsu_empty` term of `fencei_rdy`.

  ===> `io.free_count` AND `io.empty` ARE NOT A `busy`, and the distinction is
       the vector-LSU invariant, not pedantry. Neither is scoped to "the current
       instruction" and neither reaches an issue unit: `free_count` is a credit on
       a shared resource that stalls a D$ GRANT at the arbiter, and `empty` feeds
       BOOM's pre-existing `fencei_rdy`, which gates only `is_unique` DISPATCH.
       This module exports no `busy`, no `ready` to any issue queue and no
       per-instruction status of any kind, and a reviewer should reject one.

  There are no other ports. Notably absent, and each absence is load-bearing:
  any per-PRN writeback or wakeup output (see part 6), any second group-done
  port, any VRF port beyond `R2`/`W0`/`W1`, and any back-pressure on `io.resp`.
  <|end_ports|>

  <|begin_logic|>
  ---- 0. The three interface bundles, declared here ----

  hierarchy.yaml grants this node no MicroOp edge, and the already-authored
  VecBundles declares no LCB-facing bundle, so declare these three here. Their
  placement fields are COPIES of the nOP.v cursor fields the MicroOp delta owns
  (the destination PRN and the byte offset within it); the drain side lifts them
  out of the `VecElemAccess` it consumed rather than re-deriving them.

  `LcbAllocReq`: `prn` (`vecPregSz`), `ldq_idx`, `rob_idx`, `member_idx`
  (`log2Ceil(maxMembers)`), `members_target` (`log2Ceil(maxMembers)+1`),
  `active_bytes` (`vLenBytes`), `inactive_bytes` (`vLenBytes`), `undisturbed`
  (Bool), `stale_prn` (`vecPregSz`), `is_ff` (Bool), `pvl` (`vlPregSz`),
  `vl_final` (`vecVLSz`, the untrimmed VL, used only when `is_ff`).

  `LcbBeat`: `data` (`coreDataBits`), `prn`, `ldq_idx`, `dst_byte`
  (`log2Ceil(vLenBytes+1)`), `src_off` (byte offset of the valid bytes within the
  response word), `nbytes`, `nelem`.

  `LcbTrim`: `ldq_idx`, `vl_final` (`vecVLSz`), `member` (the member holding the
  stop point), `keep_bytes` (`vLenBytes`).

  ---- 1. The assembly entry array: the module's only state ----

  //@req-spec-lsu.e1
  //@req-spec-lsu.e2
  //@req-spec-lsu.e3
  This module is the Load Coalescing Buffer that sits in front of the VRF write
  port: nothing on the load path writes the VRF except through it. Its storage is
  `lcbEntries` VLEN-wide assembly entries, ONE PER DESTINATION PRN currently
  being filled, and the count is the parameter `lcbEntries` (default 8, i.e. one
  full LMUL=8 group) rather than a fixed number, because it is a memory-level
  parallelism knob and not a correctness figure.

  //@req-spec-lsu.e20
  Each entry is a register bundle:
    `valid`, `prn`, `ldq_idx`, `rob_idx`, `member_idx`, `members_target`,
    `data` (`vLen`), `byte_valid` (`vLenBytes`), `active_bytes` (`vLenBytes`),
    `own_bytes` (`vLenBytes`), `undisturbed`, `stale_prn`, `preload_pending`,
    `preload_done`, `written`, `is_ff`, `pvl`, `vl_final`.
  `byte_valid` is the per-entry BYTE-VALID BITMAP that tracks presence, and it is
  bounded by `active_bytes` — the active-element set the load vAGEN selected —
  which is the entry's completion target. Presence is tracked per byte and not
  per 64-bit lane because the active/inactive boundary is byte-granular: `vlm.v`
  covers `ceil(vl/8)` bytes, an `SEW=8` partial tail ends mid-lane, and a
  misaligned unit-stride beat starts mid-lane.

  ===> ENTRIES ARE RETAINED AFTER THEIR OWN VRF WRITE, until their whole group
       retires. That is what lets the per-group PRN-done count live IN the
       assembly entries (part 7) rather than in a second, per-instruction table,
       which the vector-LSU invariant forbids: in-flight vector LSU state may
       live only in the six VecElemQueue instances and in these entries. The
       price is that `lcbEntries` bounds concurrent destination PRNs including
       already-written ones; the default of 8 is exactly one LMUL=8 group.

  //@req-spec-lsu.e21
  //@req-spec-lsu.e22
  //@req-spec-lsu.e23
  THE LCB DOES NOT RE-DERIVE WHICH ELEMENTS ARE ACTIVE. `active_bytes` arrives
  on the allocation request, computed from the SAME mask-derived element cursor
  that `ld_vAGEN_1` used to decide which element accesses to emit at all
  (execution.rst `vector-agen`: that cursor is the single owner of which elements
  survive, and both the store DGEN and the LCB consume it rather than evaluating
  the mask again). So the bytes the LCB waits for are exactly the bytes the AGEN
  generated accesses for, and a masked-off element — which produced no nOP.v, no
  D$ access, no TLB translation and no LCAM search — is never counted as
  outstanding here either. A second, independent mask evaluation in this
  module would be the classic way to hang a masked load: any disagreement with
  the AGEN leaves an entry waiting forever on a byte no access will ever
  return, and there is no timeout anywhere on this path.

  ---- 2. Allocation: a credit protocol, before the access ----

  //@req-spec-lsu.e24
  An assembly entry is allocated for a destination PRN BEFORE any element access
  for that PRN is issued to the D$. The load drain presents `io.alloc` when its
  element walk crosses into a new destination PRN; the LCB picks a free entry
  (`SelectFirstN` over the `!valid` vector — BOOM's existing utility, per the
  reuse ground rule), writes the request's fields into it and raises
  `io.alloc.ready`. A returning element therefore always has an entry waiting.

  //@req-spec-lsu.e25
  When no entry is free, `io.alloc.ready` is low and `io.free_count` is zero, and
  THE LOAD DRAIN STALLS AT THE ARBITER: VecDcacheArbiter withholds the load-drain
  grant for an access that would need a new entry. The stall is at the D$ grant
  and never at issue. This ordering is what makes the no-back-pressure promise on
  `io.resp` safe — the LCB can afford never to refuse a response precisely
  because it refused the ACCESS earlier, and a blocked response would otherwise
  hold an MSHR against the drain that frees the entry.

  Every member of the destination group is allocated an entry, INCLUDING a member
  that is entirely inactive (a fully-covered tail member of a partial-VL load,
  `active_bytes == 0`). Such an entry has no memory traffic at all, but it must
  still be written for the group to be architecturally correct, and it must still
  be counted by the PRN-done target. // This is the "fake / bypass" beat of the M1
  design, and it is exactly the case that must be finalized ONE CYCLE LATER
  rather than in its allocation cycle — see the combinational-loop callout.
  The standalone VL = 0 / fully-inactive-GROUP case is not this module at all: it
  allocates no entry and is VecGroupCopy's.

  ---- 3. Placing a returning beat ----

  //@req-spec-lsu.e4
  //@req-spec-lsu.e5
  Each returning element is written into ITS BYTE OFFSET within the assembly
  entry for its destination PRN. Both coordinates come from the nOP.v: the
  destination PRN and the byte offset within that PRN are cursor fields the AGEN
  wrote onto the element access, carried through the D$ side-table and presented
  on `LcbBeat` as `prn` and `dst_byte`. They are NOT recoverable from the element
  index at this point without re-deriving EMUL and the mask, which part 1 forbids.

  The datapath, per response port, in two registered stages:
    stage A — register the whole `LcbBeat`. Nothing is computed from the
              unregistered response beyond capturing it.
    stage B — from the REGISTERED beat: select the entry whose `valid`, `prn` and
              `ldq_idx` all match; shift the response's valid bytes down by
              `src_off` and up by `dst_byte`; write those `nbytes` bytes into the
              entry's `data` under a per-byte enable; OR the same byte mask into
              `byte_valid`.
  If no entry matches, the beat is DROPPED silently and nothing is written. That
  is the only correct behaviour and it is load-bearing — see part 10.

  //@req-spec-lsu.j4
  In the same stage-B cycle, pulse `io.elem_done` with the beat's `ldq_idx` and
  its `nelem`, so the owning LDQ entry's `elem_done` cursor advances as element
  responses are assembled. `nelem` is supplied by VecBeatExpander, which knows
  how many elements it coalesced into the access; the LCB does not recompute it
  from `nbytes` and an EEW it would have to be told. `fault_elem` is latched by
  the cursor's owner on the fault, not here — this module never sees a fault.

  ---- 4. Inactive lanes: the overlapped `R2` pre-load ----

  //@req-spec-lsu.e8
  Inactive byte lanes — masked-off, tail, or a `vstart > 0` prefix, i.e. the
  complement of the active cursor — are filled per `vta`/`vma` policy BEFORE the
  entry's write, so the single VRF write leaves no stale bytes anywhere in the
  member. The allocator classifies them once, at allocation:
    - `undisturbed` set (`vta = 0` for tail bytes, `vma = 0` for masked-off body
      bytes, and ANY `vstart > 0` prefix, which is undisturbed unconditionally):
      the bytes are pre-loaded from `stale_pvdest`.
    - `undisturbed` clear (agnostic): the bytes are filled with ALL-ONES locally,
      needing no VRF read. Agnostic permits undisturbed-or-ones; ones is the
      cheap legal choice, and leaving them unwritten is NOT legal here, because
      `pvdest` is a FRESH renamed group whose current contents are an unrelated
      instruction's data rather than the old `vd`.
  In both cases `own_bytes = active_bytes | inactive_bytes` — the complete byte
  set of the member — so the `W0` write covers every byte it is responsible for.

  //@req-spec-lsu.e9
  //@req-spec-lsu.e10
  //@req-spec-vrf.g4
  //@req-spec-vrf.i5
  //@req-spec-vrf.i6
  //@req-spec-vrf.i7
  Under undisturbed policy the inactive-lane data is PRE-LOADED FROM
  `stale_pvdest` ON VRF READ PORT `R2`, before the arriving elements are
  overlaid. `stale_pvdest` is the architectural old-`vd` group and is the ONLY
  source of undisturbed lanes — `vta = 0` tail bytes, `vma = 0` masked-off body
  bytes and any `vstart > 0` prefix bytes all come from it, and NEVER from
  `pvs3`, which is an independent field naming an independent group (midcore.rst
  `old-vd`). The entry carries the corresponding member of `stale_pvdest` as
  `stale_prn`; the LCB drives one `io.stale_req` per cycle, picked from the
  entries with `preload_pending`, and on the following cycle merges
  `io.stale_resp` into the entry's `data` UNDER THE INACTIVE BYTE MASK ONLY,
  clearing `preload_pending` and setting `preload_done`.

  ===> THE PRE-LOAD WRITES A BYTE SET DISJOINT FROM THE ARRIVING ELEMENTS, and
       that is what makes "pre-loaded before the elements are overlaid" true
       without serializing anything. The two writers commute, so a response that
       lands before the `R2` data is not clobbered by it; the required ORDER is
       expressed as a completion condition (part 6 waits for `preload_done`)
       instead of as a datapath sequence. A pre-load that wrote the full VLEN
       would destroy already-arrived elements — which is precisely why the M1
       design had to serialize the copy in front of the load, and why this one
       must not be "simplified" back to a full-width copy.

  //@req-spec-lsu.e11
  //@req-spec-lsu.e12
  ONLY MEMBERS THAT ACTUALLY CONTAIN INACTIVE LANES ARE PRE-LOADED:
  `preload_pending` is set at allocation only when `undisturbed` is set and
  `inactive_bytes` is non-zero. With VL known at execute that is typically the
  single partially-covered member plus any fully-inactive tail members, not all 8.
  And because allocation strictly precedes the element accesses that fill the
  entry (part 2), the `R2` read is already in flight while those accesses are
  outstanding: THE PRE-LOAD OVERLAPS THE LOAD'S MEMORY LATENCY, costs no extra
  cycles, and preserves one `W0` write per PRN. That is performance target P5,
  and it replaces the serial sCopyRd/sCopyWr prologue described in the header.
  One `R2` request per cycle is sufficient at this rate; a member's pre-load is
  never on the critical path of the member's last response except in the
  degenerate case of a D$ hit returning before the `R2` read completes.

  ---- 5. The `vleff` trim ----

  On `io.trim`, for entries of the named `ldq_idx`: the entry whose `member_idx`
  equals `trim.member` narrows `active_bytes` to `active_bytes & keep_bytes`, and
  entries above it narrow `active_bytes` to zero. In both, the bytes given up
  move into the inactive set — they are now TAIL bytes — so `own_bytes` is
  unchanged and `preload_pending` is raised if the entry is `undisturbed` and did
  not already have a pre-load. Every affected entry also latches `trim.vl_final`
  over the untrimmed value it was allocated with. `members_target` is NOT reduced:
  trimming VL does not shrink the destination group, every member of which must
  still be written under the tail policy. No entry is invalidated by a trim.

  ---- 6. One VRF write per destination PRN, on `W0` ----

  //@req-spec-lsu.e6
  When ALL ACTIVE BYTES of a destination PRN are present — `byte_valid` covers
  `active_bytes`, and `preload_done` is set if a pre-load was required — the LCB
  issues a SINGLE VRF write for that PRN: `addr = prn`, `data = entry.data`,
  `mask = own_bytes`, on port `W0` (`io.vrf_write(0)`). The entry then sets
  `written` and issues nothing further. The completion test reads REGISTERED
  entry state, so a fully-inactive entry becomes complete on the cycle after its
  allocation, never in it.

  At `lsuWidth = 2` a second write port `W1` (`io.vrf_write(1)`) exists and up to
  two entries may complete per cycle, chosen with `SelectFirstN`; further
  completions wait a cycle, which costs nothing because no consumer is waiting on
  a per-PRN write. Two write ports can never name the same PRN — one entry per
  PRN and one write per entry — which is the VRF's own no-arbitration invariant.

  //@req-spec-lsu.e19
  //@req-spec-lsu.e17
  //@req-spec-lsu.e18
  THE INTERMEDIATE PER-PRN WRITES ARE VISIBLE ONLY TO THE REGISTER FILE. There
  is NO per-PRN ROB writeback and NO per-PRN vector wakeup: the only ports a
  per-PRN completion drives are `W0`/`W1`, and this module has no other output
  that could carry one. // Streaming per-PRN completions into the ROB or the
  wakeup network is what the single-shot rob_bsy clear and the per-PRN vector
  Busy Table cannot absorb (midcore.rst `group-done-wb`), so adding such a
  port is a design-invariant violation, not an optimization.

  ---- 7. The per-group PRN-done count and the ONE group-done ----

  //@req-spec-lsu.e13
  //@req-spec-lsu.e14
  The LCB owns the per-group PRN-done count. The TARGET is the destination-group
  size derived from `v_emul` and `v_seg_nf` — the true member count, accounting
  for widening/narrowing EEW and for a segmented access's `nf * emul` — computed
  by the allocator and carried on every allocation as `members_target`. The count
  itself is the number of entries with a matching `ldq_idx` and `written` set:
  because entries are retained until the group retires (part 1), the entry array
  IS the counter, and no per-instruction structure is added.

  //@req-spec-lsu.e15
  //@req-spec-lsu.e16
  When the LAST destination PRN of the group is written — the written count for
  that `ldq_idx` reaches `members_target` — the LCB emits ONE group-done, and in
  the same cycle frees every entry of that group. `io.group_done.bits` carries
  the GROUP'S MEMBER-PRN VECTOR, gathered from the retiring entries themselves:
  each entry contributes its `prn` at index `member_idx`, with `members` set to
  `members_target`, plus the `rob_idx` all members share. Assembling the vector
  from the entries rather than replicating a group descriptor in each of them is
  the second reason entries are retained. That single event is what the ROB
  single-shot busy-clear, the vector Busy-Table clear and the vector wakeup all
  consume.

  //@req-spec-lsu.e15
  There is exactly ONE group-done port. If two groups would complete in the same
  cycle (possible only at `lsuWidth = 2`), one is emitted and the other's entries
  stay `written` for one more cycle and retire next cycle. Serializing here is
  free — both groups' VRF data is already committed — and it keeps every consumer
  of the event single-ported.

  ===> DRIVE `group_done` FROM REGISTERED STATE ONLY. Its valid is a function of
       the entry array's `written` bits and nothing else; no term of it may reach
       back to `io.alloc` or `io.resp` in the same cycle. That is the M1
       combinational loop from the header callout, and the shape that caused it —
       `group_done.valid := beat.valid && beat.bits.last` — must not reappear in
       any form, including a "bypass" for the single-member case.

  ---- 8. Segmented loads: `pvtmp` is an ordinary vector destination ----

  //@req-spec-lsu.l3
  //@req-spec-lsu.l4
  //@req-spec-rob.d16
  For a segmented load the LSU half's destination group is `pvtmp`, and this
  module treats it as an ORDINARY vector destination: the allocator puts the
  `pvtmp` members in `prn`, they are assembled, written on `W0` and counted
  against `members_target` with no special case anywhere in the entry logic. So
  the LSU half of a segmented load writes the loaded data into the `pvtmp` group
  in the VRF and emits a REAL group-done — which is what wakes the coprocessor
  half's IQ slot and what the ROB's one-bit "other half pending" flag consumes.
  The absence of a special case here IS the requirement: a distinct pvtmp
  completion path would be a second completion mechanism for the ROB to
  reconcile, and midcore.rst `group-done-wb` gives it exactly one.

  ---- 9. `vleff`: the VL write and the `pvl` wakeup ----

  //@req-spec-lsu.g7
  A `vleff` load is a VL PRODUCER. On its group-done, if the retiring entries have
  `is_ff` set, the LCB also drives `io.vl_wb` with the group's `pvl` and its
  `vl_final` — the FINAL ELEMENT COUNT, which is the full VL as allocated if no
  fault occurred, or `i` if element `i > 0` faulted and `io.trim` overwrote it
  (part 5).

  ===> READ `vl_final` AND `pvl` FROM THE GROUP'S **LAST** MEMBER
  (`member_idx == members_target - 1`), not from an arbitrary or the first one.
  Resolved at E6; the requirement was unstated and the two readings differ only after
  a fault. Part 5's trim updates `vl_final` on the trim-member AND every member ABOVE
  it, leaving members BELOW holding their allocation-time value — so the last member
  is provably always current (it is either the trim target itself or strictly above
  it), while a lower member still holds the UNTRIMMED VL. Publishing from the wrong
  member writes a stale, too-large VL into the VL register file after a mid-group
  fault, and every dependent then reads it through the ordinary `pvl` path with
  nothing to flag the discrepancy. Only a directed `vleff`-with-fault test finds this,
  which is precisely why the choice is written down rather than left to the reader. That write goes to the VL register file
  and WAKES `pvl` IN DEPENDENT VECTOR SLOTS ON THE VL WAKEUP NETWORK exactly as a
  `vset`'s VL write does, so dependents pick up the possibly-trimmed VL through
  the normal `pvl` path and need no `vleff` special case. The architectural `vl`
  CSR is updated at commit, by the ROB, not from here.

  The `vl_wb` event is a plain VL-RF write plus the existing VL wakeup broadcast;
  it is NOT a second completion event. Only the element-0 fault raises
  `rob_exception`, and that path is the ROB's — `vleff` needs no serialization
  and this module gives it none.

  ---- 10. Trap and squash: a late write must be IMPOSSIBLE ----

  //@req-spec-lsu.f14
  //@req-spec-lsu.f15
  A trap or a branch squash INVALIDATES THE FAULTING (or killed) LOAD'S ASSEMBLY
  ENTRIES BY THEIR OWNING `ldq_idx`: every entry whose `io.kill_ldq(ldq_idx)` bit
  is set clears `valid` in that cycle, together with `preload_pending`,
  `written`, and its `byte_valid`. VecSquashUnit computes that bit vector, so the
  age comparison for a pointer rollback lives in the one module that owns it.
  A LATE LCB WRITE FOR A KILLED LOAD IS THEREBY IMPOSSIBLE RATHER THAN MERELY
  HARMLESS, and the distinction is the whole point: the killed instruction's
  `pvdest` group is reclaimed to the free list and reallocated within a few
  cycles, so a late write would land a full VLEN into a live, unrelated
  destination and silently corrupt a correctly-executing instruction. Two
  properties together make it impossible, and BOTH are required:
    - a beat is placed only into an entry that is `valid` and matches on `prn`
      AND `ldq_idx`, so after invalidation an in-flight response has nowhere to
      land and is dropped — never buffered, never retried;
    - the `W0` write and the group-done are generated from entry state alone, so
      an invalidated entry cannot generate either, in that cycle or any later one.
  Drain-and-discard, which is what the CII does on a flush, is NOT available
  here: a CII tag is opaque and is not recycled during the drain, whereas
  vector PRNs are recycled immediately (loadstore.rst `vec-squash` danger
  note).

  Outstanding accesses need no cancellation. Accesses fire in element order, so
  no access beyond the faulting element was ever issued, and the ones below it are
  simply dropped by the rule above.

  ---- 11. Trace ----

  Emit one guarded `VecTrace` line per key event, tagged with module name
  `VecLoadCoalescingBuffer` and `rob_idx`, gated on the `vecTrace` plusarg (off by
  default) and on `!reset`: `alloc` (entry, prn, member_idx, members_target,
  active byte count, undisturbed), `preload` (entry, stale_prn), `place` (entry,
  dst_byte, nbytes), `prn_write` (prn, own byte count), `group_done` (member PRNs,
  members) and `kill` (entry, ldq_idx). With no unit tests in this project, these
  six lines are the only way to tell "the group never completed" from "one byte
  never arrived" — the two failure modes of this module. Tracing declares no
  state, so deleting every call site leaves the design bit-identical.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Performance target P5 lands here: a masked or tail-undisturbed load must cost NO
EXTRA CYCLES versus an unmasked load at equal active-byte count. The
implementation choices that discharge it are constraints, not preferences:

- The `stale_pvdest` pre-load must be issued at ALLOCATION time, so it overlaps
  the load's memory latency. It may not be sequenced after, or in front of, the
  element accesses. Any structure in which no element access may be issued until
  a copy completes re-creates the M1 prologue and misses P5 by 2 cycles per
  member.
- Only members with inactive bytes may be pre-loaded. Pre-loading all `EMUL`
  members unconditionally is correct but costs `R2` bandwidth linear in EMUL and
  is a P5 miss on wide groups.

Timing and throughput:
- Response to VRF write is TWO registered stages (capture, then place-and-test);
  the write of a PRN's final byte therefore appears on `W0` two cycles after its
  response. There is no consumer waiting on a per-PRN write, so this latency is
  not on any critical path; the group-done that IS consumed follows the last
  member's write in the same cycle.
- Sustained one placed beat per response port per cycle, `lsuWidth` ports, with
  no back-pressure on `io.resp` ever.
- One `R2` read and up to `lsuWidth` VRF writes per cycle. No VRF port beyond
  those in midcore.rst `vrf-ports` may be added to reach any target here.

Area: `lcbEntries x (vLen + 3*vLenBytes)` bits of flops for data and the three
byte masks — about 2.8 kbit at `lcbEntries = 8`, `vLen = 256` — plus the small
per-entry tag fields. The `lcbEntries`-wide byte-mask comparators and the
per-`ldq_idx` written-count reduction are the combinational cost, and both scale
with `lcbEntries`, which is the reason to raise that parameter for MLP only
deliberately.
<|end_perf|>

<|begin_dependencies|>
VecBundles — `VecGroupDone` (the group-done payload, whose member-PRN vector and
`members` count this module fills) and, indirectly, the nOP.v definition whose
cursor fields the allocation and beat bundles copy.
VectorParams — `lcbEntries`, `vLen`, `maxMembers`, `vecPregSz`, `vlPregSz`,
`vecVLSz`, `elenBytes`.
VecTrace — the guarded trace helper. Emit-only, no state.

Binds to BOOM's existing `SelectFirstN` for free-entry and completion selection,
per the reuse ground rule; it adds no new selection utility.

Instantiates NOTHING. It is instantiated once, as `lcb`, by VecLsu, which wires:
- `io.alloc` and `io.resp` from the load-side VecBeatExpander (`ld_beat`);
- `io.free_count` to VecDcacheArbiter (`arb`), which owns the load-drain stall;
- `io.stale_req`/`io.stale_resp` to VecRegFile port `R2`, and `io.vrf_write` to
  `W0`/`W1` — the canonical partition of midcore.rst `vrf-ports`, to which this
  module adds nothing;
- `io.group_done` out through VecPipeline to the ROB's `vec_clr_bsy`, the
  VecBusyTable clear and the vector wakeup in VecIssueUnit — one event, three
  consumers;
- `io.vl_wb` to VlRegFile and the VL wakeup network;
- `io.elem_done` and `io.trim` to the LDQ-resident element cursor owned by the
  LSU delta;
- `io.kill_ldq` from VecSquashUnit (`squash`);
- `io.empty` into VecLsu's `vec_lsu_empty`.

VecGroupCopy shares this module's `R2` and `W0` ports through a strict-priority
mux in which an active load drain always wins. That mux is VecLsu's, not this
module's: the LCB drives its requests unconditionally and is never told it lost,
because it never does.

Deliberately NOT depended on: MicroOp (hierarchy.yaml grants no such edge, so the
placement and allocation fields arrive as plain bundles rather than as a whole
uop) and VecRegFile (the VRF ports are a wiring contract, not a compile edge).
<|end_dependencies|>

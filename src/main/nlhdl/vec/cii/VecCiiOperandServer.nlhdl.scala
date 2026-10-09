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
  VecCiiOperandServer — the host's Source-Request / Source-Data server: it turns
  a coprocessor pull of {tag, op_id, op_offset} into a physical VRF (or scalar
  side-table) read and returns one vLen-wide beat per request, in request order.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/cii/VecCiiOperandServer.scala,
  package boom.v4.vec.generated.cii. group vec_cii.
  depends_on VecBundles, VecTrace. Instantiated once, as `opnd`, inside
  VecCiiHost, alongside `tags` (VecCiiTagTable), `iss`, `wb`, `done`, `flush`.

  It owns VRF read ports R5, R6, R7 and R8 — one per Src-Request lane,
  CII_NUM_SRC_REQ = 4 — and nothing else. It adds no VRF port and it never
  stalls on one, because the partition in midcore.rst `vrf-ports` is static and
  those four ports have no other reader.

  ===> VS3 AND STALE_VD ARE TWO DISTINCT SLOTS NAMING TWO DISTINCT GROUPS.
       `pvs3_grp` is an explicitly encoded third source; `stale_pvdest_grp` is the
       group that held the destination architectural vreg before this OP.v renamed
       it. This module performs NO instruction-dependent reinterpretation: it
       serves whichever slot was asked for, straight from the per-tag side-table,
       and never guesses which was meant. THE COPROCESSOR DECIDES WHAT TO PULL —
       one request when the two coincide (RMW arithmetic such as vfmacc.vv), both
       when they differ (masked vadd.vv under vma=0, vslideup's untouched prefix,
       a vcompress tail).
       An earlier draft had ONE VS3 slot that the HOST resolved to stale_pvdest
       "when the instruction encodes no third source". That put instruction
       decoding in the host adapter and made the two-different-groups case
       UNREPRESENTABLE. Do not reintroduce it.

  ===> A KILLED TAG'S REQUEST IS STILL ANSWERED, with a don't-care beat and no
       VRF read. Src-Data is the one channel on which the HOST is the sender, so
       it holds no credit to hand back; swallowing a request stalls the channel
       for every SURVIVING instruction and turns a squash into a permanent hang.
       See VecCiiFlush.

  Governing spec anchors: cii.rst `cii-operands` and `cii-kill-contract`,
  midcore.rst `vrf-ports` and `old-vd`, execution.rst `vector-execution`
  ("What the host provides" / "What the coprocessor provides") and `cii-prn-arn`.

<|begin_module|>

  <|begin_parameters|>
  Constructor parameters, all resolved at elaboration. Nothing here is a tuning
  knob: each is either a mirror of the frozen SV contract in
  `tt_cii_caracal_pkg.svh` or a VectorParams field reached through
  `HasVectorParams`.

  `numSrcLanes` — the number of Src-Request/Src-Data lanes. Default and only
  legal value 4, DERIVED from `CII_NUM_SRC_REQ` = `CII_NUM_SRC_DAT_RSP` = 4.
  Require the two SV constants are equal and that this equals both: a host with
  fewer data lanes than request lanes could not answer a full cycle of pulls,
  and the channel has no back-pressure line to tell the coprocessor so.

  `vLen` (256), `vecPregSz` (7 at 96 PRNs), `maxMembers` (8), `ciiTagBits` (4)
  and `eLen` (64) come from VectorParams. No width below is a literal — the
  Src-Data payload is `vLen` bits, never 256, because the SV package
  parameterizes its payload on VLEN too.

  `srcReadLatency` — request-to-beat latency in cycles. Fixed at 1, the
  "registered, one-cycle VRF read" of cii.rst `cii-operands`. It is a parameter
  for one reason: the ordering-FIFO depth in the logic section is derived from
  it, so an extra pipeline stage cannot be added without the ordering structure
  following it. Legal range 1..2.

  ===> THE ONE CYCLE IS THE VRF'S, NOT THIS MODULE'S. VecPipeline's ruling is
  canonical for every port R0-R8: THE VRF READ IS A REGISTERED ONE-CYCLE READ
  AND THE FLOP LIVES IN `VecRegFile`. So `srcReadLatency = 1` is spent entirely
  inside the register file, and this module adds NO payload register — see stage
  1. If both sides registered, observable latency would be 2 while
  `srcReadLatency` still said 1, and every Src-Data beat would answer the
  request one beat late, FOREVER, for every surviving instruction, with no error
  signal anywhere. The bank's "0-cycle, may not be pipelined" figure is
  BANK-INTERNAL and must not be read as a consumer-visible latency.

  This module is elaborated only when `usingRVV` is true. The gate lives in the
  parent (VecCiiHost) instantiation, not in a local `Bool`, so a vectors-off
  build emits no instance at all and stays bit-identical to pre-Caracal BOOM v4.
  The gate is BOOM's `usingRVV`, not rocket's `usingVector`.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit pair: posedge `clock`, ACTIVE-HIGH
  SYNCHRONOUS `reset`. The active-low `reset_n` the SV coprocessor stack takes is
  inverted exactly once, in `tt_cii_host_wrap`, and never here.

  `src_req` — `Vec(numSrcLanes, Flipped(Valid(new CiiSrcReq)))`. Lane `i` carries
  {`tag`, `op_id`, `op_offset`} popped from the host-side Src-Request FIFO. There
  is deliberately NO `ready`: the CII channels carry no back-pressure line
  (cii.b3) and this module cannot stall (see the logic section), so `Valid` is
  the honest shape and `Decoupled` would advertise a stall that does not exist.

  ===> THE CREDIT GRAIN IS THE BEAT, NOT THE LANE, and the port list must say so.
       `tt_cii_interface.sv` carries ONE `req_valid` and ONE `req_credit` for a
       beat of `numSrcLanes` lanes — there is no per-lane valid and no per-lane
       credit anywhere in the interface. VecCiiHost therefore drives ALL FOUR of
       `src_req(i).valid` from that single beat valid (the `Vec`-of-`Valid` shape
       is kept only because each lane's payload is independent), and PER-LANE
       ACTIVITY IS ENCODED IN THE PAYLOAD, as `op_id = CII_SRC_NONE` on an unused
       lane. That is what `NONE` is for, and it is why an inactive lane is answered
       with a defined don't-care beat rather than swallowed.

  `req_credit` — `Output(Bool())`, ONE bit, not a `Vec`: one credit per BEAT
  consumed, in the cycle it is consumed, because the channel meters beats. Each
  CII channel's RECEIVER owns its FIFO and returns credits; the host is the
  receiver on Src-Request. Assert the four lane valids agree — they must, one
  valid qualifies them all — and drive the credit off the beat, never off a
  per-lane reduction that could return four credits for one beat.

  `src_lookup` — `Vec(numSrcLanes, ...)`, this module's reader of VecCiiTagTable's
  declared `src_lookup` port. Request `{tag, op_id, op_offset}` driven straight off
  `src_req(i).bits`; response `{prn: UInt(vecPregSz.W), read_vrf: Bool,
  scalar_data: UInt(eLen.W), killed: Bool, rob_idx: UInt(robAddrSz.W)}` returned
  COMBINATIONALLY in the same cycle. Four independent lanes, no handshake, no
  back-pressure either way.

  ===> THE NARROW RESPONSE, NOT THE WHOLE ENTRY. An earlier draft of this file
  took `tag_entry: Vec(numSrcLanes, Input(new VecCiiTagEntry))` and did the
  slot-and-member mux locally. That crosses FOUR copies of a ~376-bit entry
  between two ADJACENT modules to compute the identical function at the
  identical logic depth on the far side, where the 16-to-1 entry mux already is.
  The resolve is emitted ONCE, in VecCiiTagTable; this module consumes `{prn,
  read_vrf, scalar_data, killed, rob_idx}` and owns everything downstream of it.
  Do not reintroduce a whole-entry port, and do not add a local class mux.

  `vrf_read_addr` / `vrf_read_data` — `Vec(numSrcLanes, Output(UInt(vecPregSz.W)))`
  and `Vec(numSrcLanes, Input(UInt(vLen.W)))`. Lane `i` drives VRF read port
  `R(5+i)`, 0-based per midcore.rst `vrf-ports`; cite the port by number at the
  connection site. There is no read enable and no read handshake — VecRegFile's
  read ports are unconditional. `vrf_read_data` is REGISTERED INSIDE VecRegFile
  and arrives at t+1 for an address presented at t, per VecPipeline's canonical
  ruling for R0-R8; this module presents it, muxed, in that same t+1 cycle and
  holds no copy of it. A duplicate read is still free — the ports are statically
  partitioned and unconditional — but it is now one flop per port inside the
  register file rather than one here.

  `src_data` — `Vec(numSrcLanes, Valid(new CiiSrcData))`, output. The payload is
  `data` (`vLen` bits) and NOTHING ELSE — no tag, no `op_id`. The coprocessor
  correlates a beat with its request purely by position in the global order, so
  the ordering rules in the logic section are load-bearing, not cosmetic.

  There is deliberately NO flush port. Kill state arrives as the `killed` bit of
  the `src_lookup` response, which VecCiiFlush sets in the table; a second, direct
  view of the flush would create two kill paths that could disagree by a cycle.

  There is likewise NO Src-Data credit input and NO occupancy counter here. The
  `dat_credits` counter lives in VecCiiHost, and its exhaustion is an ASSERTION on
  that side, never a gate on this module's `valid` — see the logic section.
  <|end_ports|>

  <|begin_logic|>
  ---- The pull model, and why there are four independent lanes ----

  //@req-spec-cii.f1
  //@req-spec-vrf.h7
  The CII PULLS operands: there is no push path in this module and no vector
  register-read stage that broadcasts a fixed operand set at issue. All four
  lanes may be live in the same cycle — the VPU wrapper returns "member k of
  {VS1, VS2, VS3, VM} in one dat beat" — so the lanes are structurally
  independent, each with its own VRF read port, side-table read port and pipeline
  register. No lane arbiter, no serialization, no resource shared between lanes.
  That is why `CII_NUM_SRC_REQ = 4` buys four VRF ports rather than two: a lane
  waiting on another lane's port would stall a channel that has no way to express
  a stall.

  ---- Stage 0: pop, credit, look up, decode ----

  //@req-spec-cii.f2
  //@req-spec-cii.f3
  In the cycle a beat arrives, the module pops it and, per lane, drives
  `src_lookup(i).req` with `{tag, op_id, op_offset}` straight off the payload. The
  table answers combinationally in the same cycle: `op_id`
  (`cii_caracal_srcid_e`) selects a SOURCE CLASS and `op_offset` selects the GROUP
  MEMBER INDEX within that class's PRN vector, and the response is the resolved
  `{prn, read_vrf, scalar_data, killed, rob_idx}`. The resolve depends on nothing
  else: not on the instruction word, not on `vtype`, not on `vl`. THE CLASS AND
  MEMBER MUXES ARE EMITTED IN VecCiiTagTable, behind the entry mux they share; this
  module holds no copy of them and no entry field.

  //@req-spec-cii.e13
  `req_credit` is asserted unconditionally whenever the beat is valid, in the same
  cycle — ONE credit for the beat, never one per active lane. Because the module
  can never stall, "pop" and "arrived" are the same event, so credit accounting
  needs no occupancy counter and no pending queue. This holds for a KILLED tag
  exactly as for a live one, and for a beat whose lanes are all `NONE`: the beat is
  popped and its credit returned regardless.

  ---- The op_id map: seven slots, one line each ----

  //@req-spec-cii.f4
  `NONE` (0) is RESERVED AND REQUESTS NO SOURCE — and it is a value that
  LEGITIMATELY APPEARS ON THE WIRE, because with one `valid` per beat it is the
  only way to spell an inactive lane. Do not assert against it. Handle it exactly
  like a killed lane: the beat's credit is returned, no VRF read is performed, and
  a DEFINED DON'T-CARE beat is still returned on that lane one cycle later. The
  Src-Data channel is positional, so an omitted or shifted beat would desynchronise
  every surviving instruction behind it. Value 7 is unassigned and is handled
  identically; unlike `NONE`, value 7 IS assertable, since nothing on the VPU side
  should ever emit it.

  `VS1` (1) resolves to the tag's `pvs1_grp(op_offset)`.

  `VS2` (2) resolves to the tag's `pvs2_grp(op_offset)`.

  `VS3` (3) resolves to the tag's `pvs3_grp(op_offset)` — the EXPLICITLY
  ENCODED third source, and nothing else. It is never re-pointed at any other
  field under any condition.

  `VM` (4) resolves to the tag's `pvm`, the renamed `v0` mask. `op_offset` is
  IGNORED for this slot: a mask always occupies a single vector register, never a
  group, whatever the current LMUL, so `pvm` is a single PRN and there is no
  member mux on this path.

  //@req-spec-cii.f9
  `SCALAR` (5) resolves to the tag's captured scalar operand, arriving as the
  response's `scalar_data` with `read_vrf` FALSE — the `.vx`/`.vf` VALUE captured
  from the INT/FP bypass at issue by VecCiiIssue — and performs NO VRF READ.
  `op_offset` is ignored here too. The value occupies the low `eLen` bits
  of the beat, upper bits zero, so a scalar beat is defined rather than an X
  pattern in cosim. The coprocessor can address no register file (execution.rst
  `cii-prn-arn`), so a late read is not available and the value must already be
  in the side-table. This module applies no format conversion of any kind to it.

  `STALE_VD` (6) resolves to the tag's `stale_pvdest_grp(op_offset)` — the
  old-`vd` group, for merging. This is the slot that carries the one-value SV
  delta to `cii_caracal_srcid_e`; the wire is already `logic [2:0]`, so values 6
  and 7 were free and no payload width changes.

  ===> WHERE THESE SEVEN LINES ARE ACTUALLY EMITTED, STATED HONESTLY. The `op_id`
       mux they describe lives in VecCiiTagTable, behind the narrow `src_lookup`
       port. Phase R closed this the explicit way, via decision D12 case 3
       (allocation wrong, requirement fine): `spec-cii.f5`-`f8` and `f10` are now
       ALLOCATED AND TAGGED IN VecCiiTagTable, at the per-slot resolutions that
       emit them. The seven lines above stay here as the description of the
       encoding this module speaks — they are prose, not a coverage claim, which
       is why they no longer carry tags.

       This module still owns the OUTCOME of every slot — the `R5`-`R8` address
       drive, the `SCALAR` slot's no-read path, the returned beat, the request
       order, and the killed drain — and those obligations are carried by the IDs
       that stayed: `f4`, `f9`, `f12`, `vrf.j13`/`j14`. So the re-allocation moved
       the resolution mapping to where the mux is without stranding anything here.

  ---- The two-slot invariant: the reason this module exists in this shape ----

  //@req-spec-cii.f12
  //@req-spec-vrf.j13
  //@req-spec-vrf.j14
  `VS3` and `STALE_VD` are two distinct slots naming two distinct groups, and
  this module never conflates them, never collapses them into one field, and
  never guesses which of the two was meant. There is no condition — not "the
  instruction encodes no third source", not `v_is_masked`, not `vma` — under
  which a `VS3` request reads `stale_pvdest_grp` or a `STALE_VD` request reads
  `pvs3_grp`.

  //@req-spec-vrf.i8
  //@req-spec-vrf.i9
  Correspondingly, a consumer needing old-`vd` — a `vta = 0` tail, a `vma = 0`
  masked-off lane, a `vstart > 0` prefix — must request `STALE_VD` and read
  `stale_pvdest_grp`, and must NEVER read `pvs3_grp`. `stale_pvdest` is the only
  source of undisturbed lanes in the machine. The two fields coincide for RMW
  arithmetic and diverge for masked non-RMW ops, `vslideup` and `vcompress`; a
  design serving old-`vd` from `pvs3` would be silently right in the common case
  and silently wrong in exactly the merging cases that need it.

  //@req-spec-cii.f13
  //@req-spec-cii.f14
  //@req-spec-vrf.j15
  The module serves whichever slot is requested STRAIGHT FROM THE PER-TAG
  SIDE-TABLE and applies no instruction-dependent reinterpretation. Concretely:
  the class mux is selected by `op_id` alone, and this module does not even hold
  that mux — it forwards `op_id` unaltered on `src_lookup(i).req` and consumes the
  single `prn` that comes back, so there is no field here to re-point and nothing
  to reinterpret. No instruction bits reach this
  module — the issue packet's `instr` field is not an input here — which makes
  the property structural rather than a coding convention a later edit could
  erode.

  The coprocessor decides what to pull. With four lanes it issues one request
  per slot it actually needs: ONE when pvs3 and stale_pvdest name the same
  group (vfmacc.vv vd,vs1,vs2), BOTH when they differ, spending another lane.
  Moving that choice to the side that decodes the instruction is the point.

  ---- op_offset: the member index, unchecked on purpose ----

  //@req-spec-cii.f32
  `op_offset` is the LMUL MEMBER INDEX, 0..`maxMembers`-1. The field is 3 bits
  and `maxMembers` is 8, so every encodable value is in range and no clamping,
  saturation or default-to-member-0 behaviour is needed or permitted. Require
  `op_offset`'s width equals `log2Ceil(maxMembers)` at elaboration, so a future
  `maxMembers` change fails the build instead of silently truncating an index.

  //@req-spec-cii.f33
  //@req-spec-cii.f34
  For a register group of `NM = EMUL` members the coprocessor walks
  `op_offset = 0..NM-1`, and the host does NOT bound-check the offset against the
  tag's recorded group size. Widening DOUBLES the destination and relevant-source
  member count, so a widening op legitimately pulls more members than the narrow
  source's EMUL suggests; a host-side check against the wrong one of those two
  counts would substitute member 0 and corrupt exactly the widening cases. The
  host serves any offset named; the coprocessor owns the walk.

  ---- No policy is applied on the read ----

  //@req-spec-cii.f31
  The host applies NO `vta`/`vma` masking on the operand read, and reads neither
  `vl` nor `vtype` on this path. A full, raw `vLen`-bit register member is
  returned every time. The coprocessor pulls the `v0` mask itself on the `VM`
  slot and old-`vd` on `STALE_VD`, and applies tail/mask policy internally —
  which is the same division of labour as the Writeback direction, where the host
  writes `wb_data` verbatim.

  ---- Stage 1: the VRF's registered read, and the ordering structure ----

  //@req-spec-cii.f21
  //@req-spec-cii.f22
  One beat is returned per request, `vLen` bits wide for a vector slot, exactly
  `srcReadLatency` = 1 cycle after the request, IN THE EXACT ORDER THE REQUESTS
  ARRIVED. Timing: in cycle t the resolved PRN drives `vrf_read_addr(i)`;
  VecRegFile REGISTERS the read INTERNALLY and presents `vrf_read_data(i)` in
  cycle t+1; this module muxes that arriving value against the scalar/don't-care
  alternatives using its one-deep control register and drives `src_data(i)` in that
  same cycle t+1. The Src-Data output is therefore register-driven — by the flop in
  the register file — and the combinational path is split in two by it: address
  resolve up to the VRF in cycle t, and one narrow mux after it in cycle t+1.

  ===> THIS MODULE DECLARES NO `vLen`-WIDE PAYLOAD REGISTER AND NO ADDRESS
       REGISTER, AND ADDING EITHER IS A CORRECTNESS BUG, NOT AN AREA COST.
       MEASURED: a generated revision latched the resolved PRN into a
       `vrfAddrReg` before driving `vrf_read_addr`. Observable latency became 2
       while `srcReadLatency` still said 1, so every Src-Data beat carried the
       PREVIOUS request's operand. On `axpy-vector` (LMUL=8) that silently
       corrupted MEMBER 0 of every `vfmacc` group and nothing else -- members 1-7
       were also shifted by one beat but their operands happened to be equal, so
       the whole class of bug showed up as one wrong register. VecPipeline's ruling puts the read flop
       inside `VecRegFile` for all of R0-R8; a second flop here makes observable
       latency 2 while `srcReadLatency` still says 1, and the positional Src-Data
       channel is then offset by exactly one beat FOREVER — every surviving
       instruction gets the previous request's operand, with no error signal
       anywhere in the machine. Under no ruling may both sides register.

  //@req-spec-cii.f23
  //@req-spec-cii.f44
  //@req-spec-cii.f45
  The ordering FIFO that covers that registered read is, at `srcReadLatency` = 1,
  exactly one stage deep PER LANE, and it is a CONTROL register only: the `Valid`
  bit, the killed / scalar / don't-care select, and the `xLen` scalar payload that
  must be presented instead of VRF data. No `vLen` field, no reordering network.
  It is correct because the global order is
  `(cycle, lane)` lexicographic and BOTH coordinates are preserved structurally.
  Requests arriving in the same cycle are ordered by ASCENDING LANE INDEX; lane
  `i`'s beat is returned on Src-Data lane `i` one cycle later, so the set of
  valid data lanes in a cycle is exactly the set of valid request lanes of the
  previous cycle, and reading them in ascending lane index reproduces the request
  order. Cycles stay ordered because all four lanes share one fixed latency.

  WHY A CONTROL REGISTER IS STILL NEEDED once the payload one is gone: the
  arriving VRF word is unqualified. Whether lane `i`'s beat at t+1 is real data,
  the low-`eLen` scalar value, or a defined don't-care depends on the REQUEST
  that was popped at t, which is no longer on the wire. One `Valid` bit plus a
  2-bit select plus `xLen` bits of scalar payload per lane carries exactly that
  and nothing more. The scalar payload is registered rather than re-read because
  the side-table lookup for cycle t's request is gone by t+1.

  ===> DO NOT COMPACT THE BEATS ONTO THE LOW LANES. THE LANE MAPPING IS
  STRAIGHT-THROUGH. A sparse request set — lanes 0 and 2 active, lanes 1 and 3
  carrying `NONE` — is answered on DATA LANES 0 AND 2, with lanes 1 and 3
  carrying the defined don't-care; nothing shifts down to lanes 0 and 1. With
  one `valid` per beat and no per-lane count on the wire, compaction is not even
  representable, and mixing the two conventions between host and VPU would
  offset the whole positional channel by one beat for every sparse cycle — a
  corruption with no error signal anywhere.
  If `srcReadLatency` is ever raised to 2, the per-lane FIFO deepens to 2 with
  it; the depth is derived from the parameter and never written as a literal.

  ---- Duplicate members are read twice, deliberately ----

  //@req-spec-cii.f46
  Two lanes may name the same member in the same cycle, and the host performs ONE
  READ PER LANE. Nothing coalesces the duplicate and no comparator looks for one.
  `vadd.vv v1, v2, v2`, whose two sources rename to a single PRN, reads that PRN
  on two ports in the same cycle at no cost, because each lane owns its own
  statically partitioned port. A dedupe network would add a cross-lane comparison
  and a beat-replication mux to save nothing.

  ---- The kill drain ----

  //@req-spec-cii.e13
  //@req-spec-cii.e14
  //@req-spec-cii.e15
  When the lookup response has `killed` set, the beat is still POPPED and its
  `req_credit` still returned, a don't-care Src-Data beat IS STILL RETURNED one
  cycle later with `valid` set, and NO VRF READ is performed. The read is
  suppressed by holding `vrf_read_addr(i)` at its previous value (the read
  decoder does not toggle, so no read energy is spent), and the beat is formed at
  t+1 by the control register's select choosing the DEFINED DON'T-CARE CONSTANT —
  zeros — over the arriving `vrf_read_data(i)`. A constant, not "whatever the port
  happens to present": with the payload register gone there is no previous contents
  to hold, and an X or a stale live operand would propagate into the VPU and trip
  its own assertions, turning a correct drain into a false failure in cosim. The
  same select serves `NONE`, the unassigned `op_id` 7, and every lane of an
  all-idle beat, for the same reason and with the same value.

  The beat is MANDATORY. On Src-Data the HOST is the sender, so it holds no
  credit to return in lieu of a beat; the VPU has no kill line, would wait
  forever for the missing beat, never emit `last`, never free its tag, and
  every surviving instruction behind it would hang. Never "ignore the request
  and hand back a credit" on this channel. See VecCiiFlush.

  Correctness does not depend on WHEN the kill becomes visible here. `killed` is a
  registered per-tag bit, so a request arriving in the same cycle as the flush
  sees it clear, performs a real read and returns real data — harmless, because
  the tag's writeback effects are suppressed downstream by VecCiiWriteback and
  VecCiiComplete regardless. A late kill costs one wasted VRF read, never a
  correctness failure. Do not add a bypass to close that cycle.

  ---- The Src-Data credit counter is an ASSERTION, never a gate ----

  The Src-Data credit counter is VecCiiHost's (`dat_credits`), and this module's
  obligation toward it is stated here because this module is what would be
  tempted to obey it: `dat_valid` MUST NEVER BE GATED ON THE COUNTER, and this
  module offers no input on which such a gate could be wired. A beat is due
  exactly one cycle after its request, by the positional contract above; holding
  one back or dropping one desynchronises the channel PERMANENTLY, for every
  surviving instruction, and the VPU has no way to notice or recover. Exhaustion
  is unreachable absent a VPU-side protocol violation — it needs 16 outstanding
  requests whose data the coprocessor has not popped — and the counter exists
  solely to make that violation VISIBLE: assert `dat_credits =/= 0` when a beat is
  presented, and trace the zero. Converting a detectable protocol violation into a
  silent permanent hang is the strictly worse trade.

  ---- Assertions and tracing ----

  Assert, all cheap and all synthesizable: `op_id` is never the unassigned value 7
  (but `NONE` is legal and must NOT be asserted against — it is the inactive-lane
  encoding); the `numSrcLanes` request valids agree, since one beat valid qualifies
  them all; `src_data(i).valid` in cycle t+1 equals `src_req(i).valid` of cycle t
  (the one-in-one-out property the positional channel rests on); a `SCALAR` request
  never drives a VRF read; and exactly one `req_credit` pulse per accepted beat.

  Emit one guarded VecTrace line per served beat, gated on the `vecTrace` plusarg
  and `!reset` as every vec module is:
    [vec] VecCiiOperandServer src_serve rob=<rob_idx> tag=<tag> lane=<i>
          op_id=<op_id> off=<op_offset> prn=<resolved> killed=<killed>
  `rob_idx` comes from the `src_lookup` response — this module holds no `MicroOp`, so
  it needs VecTrace's tag-keyed entry point rather than the `MicroOp`-taking
  `trace` helper. Tracing the resolved PRN alongside `op_id` is what makes a
  wrong-slot bug visible in one grep.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput: one Src-Data BEAT per cycle sustained, `numSrcLanes` lanes wide, i.e.
`4 * vLen` = 1024 bits of operand bandwidth per cycle. This is a constraint, not an
aspiration — the module must never present fewer ACTIVE LANES in cycle t+1 than the
beat it accepted in cycle t named, and it must present them on the SAME lane
indices.

Latency: exactly 1 cycle from Src-Request to Src-Data, fixed and identical on all
four lanes. Equal latency is a correctness property, not a performance one: it is
what lets the ordering structure be a per-lane register instead of a reorder
buffer.

Back-pressure: NONE, in either direction. The module has no stall condition, no
ready output and no occupancy counter, and can have none: the CII channels have
no back-pressure line and the VRF ports it reads are statically partitioned with
no other reader. An edit that introduces a stall here would first have to add a
channel-level mechanism that does not exist. In particular `dat_valid` is not
gated on the parent's Src-Data credit counter — that counter is an assertion, not
a throttle (see the logic section).

Critical path: `src_req(i).bits.tag` into the sibling's entry mux, through the
class and member muxes, out as `prn`, into the VRF read decoder — ending at the
read flop INSIDE `VecRegFile`. It crosses a module boundary twice and is the
timing risk in this module's cone, and the figure worth measuring first. What it
no longer contains is the `vLen`-wide read mux, which now sits after the VRF's own
flop. IF IT FAILS TIMING the fix is to raise `srcReadLatency` to 2 — registering
the resolved PRN before the VRF read — and to deepen the per-lane ordering FIFO by
the same amount in the same edit. Inserting a stall is not legal; neither is
deepening the pipeline without the FIFO (the positional channel would then reorder
against itself on any cycle where lanes carried different depths); and neither is
recovering the slack by registering the read data here, which is the one edit
whose failure mode is silent.

Area: four one-deep CONTROL registers — a `Valid` bit, a 2-bit select and `xLen`
bits of scalar payload per lane, ~268 flops at the defaults — plus four narrow
side-table read ports. The `vLen`-wide payload registers an earlier draft declared
(4 x 256 = 1024 flops) ARE DELETED, not moved: their function is served by the read
flop already inside `VecRegFile`. No storage of its own — operand data is never
buffered at all here, which is why this module holds no instruction-scoped state.
<|end_perf|>

<|begin_dependencies|>
VecBundles — `CiiSrcReq` ({tag, op_id, op_offset}) and `CiiSrcData` ({data}) are
the Chisel view of the frozen SV payloads and are declared there, once. This
module no longer binds `VecCiiTagEntry` at all: with the narrow `src_lookup`
response replacing the whole-entry port, the entry type crosses this boundary
nowhere, and the request/response pair is declared by VecCiiTagTable alongside the
mux that answers it.

VecTrace — the guarded trace line above.

VectorParams reaches this module transitively, through VecBundles and
`HasVectorParams`, for `vLen`, `vecPregSz`, `maxMembers`, `ciiTagBits` and
`eLen`. `tt_cii_caracal_pkg` is the authority for `CII_NUM_SRC_REQ` and
`CII_NUM_SRC_DAT_RSP`; derive `numSrcLanes` from it rather than writing 4.

Instantiates nothing.

Instantiated by VecCiiHost as `opnd`. Its three seams, each owned by a named
node: VecCiiTagTable supplies the four NARROW `src_lookup` lanes — combinational,
`{prn, read_vrf, scalar_data, killed, rob_idx}`, the `op_id` mux emitted on its
side (including `killed`, which VecCiiFlush sets and this module only reads);
VecRegFile supplies read ports R5-R8, unconditional, with no enable and no
handshake, and REGISTERED at t+1 with the flop on its side per VecPipeline's
ruling; `tt_cii_host_wrap` carries the Src-Request beats in and the Src-Data beats
out, and owns the reset-polarity inversion. VecCiiHost owns the beat-grain
unpacking (one `req_valid` fanned to four lane valids, one `req_credit` returned)
and the Src-Data credit counter this module never reads.
<|end_dependencies|>

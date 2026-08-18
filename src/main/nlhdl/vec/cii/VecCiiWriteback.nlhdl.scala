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
  VecCiiWriteback — the CII Writeback channel's BEAT PLACER: it pops one result
  beat per cycle, returns its credit, and steers the payload to exactly one of
  three destinations — VRF write port `W2`, the INT register file, or the FP
  register file.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/cii/VecCiiWriteback.scala,
  package boom.v4.vec.generated.cii.
  depends_on VecBundles, VecTrace. Instantiated once, as `wb` inside VecCiiHost.

  ===> THIS NODE PLACES BEATS AND NOTHING ELSE. Last-beat completion belongs to
       its sibling `VecCiiComplete` (instance `done`): group-done, the ROB
       busy-clear, the tag free and the `fflags`/`vxsat` accrual are ITS ports.
       The seam between the two is the single `last` bit of `wb_status` — which
       is what makes the split clean, since completion needs one bit off the beat
       plus the tag's side-table entry and nothing this module computes. Do not
       add a completion output here; see the reject list in the ports section.

  ===> THE BEAT IS WRITTEN VERBATIM. The VPU pulls `v0` (the `VM` slot) and the
       old destination group (`STALE_VD`) itself and applies `vta`/`vma`
       internally, so `wb_data` is a FULLY-FORMED VLEN result. This module must
       not re-apply `vl` or `vtype` in any form — no element mask, no tail
       zeroing, no byte mask derived from `vl`. `vl`, `vtype`, `vstart` and
       `vxrm` are not even side-table fields (cii.rst `cii-issue`): they rode the
       per-instruction Issue packet, so this side of the interface cannot reach
       them, and that structural absence is what must stay true.

  ===> A KILLED TAG IS DRAINED, NOT IGNORED. The beat is still popped and
       `wb_credit` still returned — otherwise the channel stalls for every
       SURVIVING instruction — while the VRF / INT / FP write is suppressed. The
       other three suppressions (group-done, `clr_rob`, CSR side effects) are
       VecCiiComplete's half of the same contract.

  Governing spec anchors: cii.rst `cii-writeback`, `cii-kill-contract`,
  `cii-segmented`; execution.rst `vector-execution` ("What the coprocessor
  provides") and `cii-prn-arn`; midcore.rst `vrf-ports` and
  `midcore-segmented-store`.

<|begin_module|>

  <|begin_parameters|>
  No case-class parameters of its own. Every width comes from VectorParams
  through `HasVectorParams` and from `HasBoomCoreParameters`, so a sizing change
  ripples instead of desynchronising one side of the frozen CII contract:
  `vLen` (256), `vLenBytes` = `vLen/8` (32, the VRF write-mask width),
  `maxMembers` (8), `vecPregSz` (7), `ciiTagBits` (4), `xLen` (64), `pregSz` (the
  scalar physical register width, the same name VecCiiTagTable uses for the `pdst`
  it stores) and `robAddrSz`. No literal widths anywhere in this module.

  `memberBits` — derived, `log2Ceil(maxMembers)` = 3 bits, the width of
  `wb_dst_offset`; same name and value VecCiiTagTable uses for the `op_offset` of a
  Src-Request, because both are member indices into a group of the same size.

  `ciiNumDstWb` — derived from `CII_NUM_DST_WB` in `tt_cii_caracal_pkg.svh`,
  value 1. Require it to be exactly 1 at elaboration.

  WHY THE `require`: one writeback lane is what makes ONE VRF write port
  sufficient. The coprocessor cannot present two result beats in a cycle, so
  W2 is never contended and never arbitrated (VecRegFile owns that
  requirement). If the SV package ever raised CII_NUM_DST_WB, this module
  would silently drop a beat per cycle, and the fix is NOT a second write port
  — the vrf-ports partition is canonical and adds none. Fail the build.

  `usingRVV` is a Scala `Boolean` of `BoomCoreParams`, not a hardware signal.
  This module is elaborated only inside VecCiiHost, which exists whenever
  `usingRVV` is set and carries no `enableVectorArith` sub-gate (see that file's
  elaboration-gate callout); in a vectors-off build it is ABSENT rather than
  tied off, so the RTL stays bit-identical to pre-Caracal BOOM v4. There is no
  hardware enable input here, now or ever.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel default and hierarchy.yaml's `defaults`:
  one `core_clk` domain, posedge-triggered, with an ACTIVE-HIGH SYNCHRONOUS
  `core_reset`. Both implicit; no explicit clock or reset port is declared.
  This module holds no state, so reset reaches nothing but the trace gate.

  ---- The channel (input) ----

  `io.wb` — `Flipped(Valid(new CiiWriteback))`, the beat presented by the parent
  after it unpacks the flat `wb_*` wires from the `tt_cii_host_wrap` BlackBox.
  There is NO `ready`: the CII is credit-metered and has no back-pressure line,
  and the receiver returns one credit per pop. Fields, per VecBundles:
  `tag` (`ciiTagBits`), `wb_data` (`vLen`), `wb_dst_offset` (`memberBits`),
  `wb_wr_en`, and the `wb_status` sub-bundle.

  //@req-spec-cii.g19
  //@req-spec-cii.g21
  //@req-spec-cii.g22
  `wb_status` is consumed as the named sub-bundle `{last, dst_kind, vxsat,
  fflags}` declared in VecBundles, never sliced out of a bare 9-bit field by
  hand. `last` marks the final beat of the `tag` and is read here ONLY to qualify
  trace and assertions — it is VecCiiComplete's input. `dst_kind` is the routing
  selector below. `vxsat` is the STICKY fixed-point saturation bit; `fflags` are
  the five FP exception flags `{NV, DZ, OF, UF, NX}` in that order, matching
  `tile.FPConstants.FLAGS_SZ`. This module DECODES both and accrues NEITHER —
  accrual toward `vxsat`/`fflags` at commit is VecCiiComplete's, and a second
  accumulator here would double-count every beat.

  //@req-spec-cii.g17
  `wb_dst_offset` is a MEMBER INDEX within the destination register group — a
  group offset, not a register number and not a PRN. The coprocessor addresses
  no register file, architecturally or physically (execution.rst `cii-prn-arn`).

  //@req-spec-cii.g18
  `wb_wr_en` is a PER-BEAT write enable. It gates the register-file write for
  this beat alone and has no effect on popping, on the credit, or on `last`: a
  beat with `wb_wr_en` low still consumes a credit and may still be the beat
  that carries `last` (a status-only final beat is legal and must not be
  mistaken for an idle cycle).

  ---- The side-table resolve (the `wb_lookup` port of VecCiiTagTable) ----

  `io.wb_lookup` — this module's reader of the `tags` sibling's declared
  `wb_lookup` port, combinational, no handshake, valid in the SAME cycle as the
  beat. Request `{tag, wb_dst_offset}`, driven straight off the beat. Response, of
  which this module reads four fields: `prn` (`UInt(vecPregSz.W)`, the destination
  member PRN the table already resolved), `wr_en` (Bool, that member exists in the
  destination group), `pdst` (`UInt(pregSz.W)`, the renamed scalar physical
  destination) and `rob_idx`. The remaining response fields (`pvdest_grp`,
  `members`, `is_shared`, `killed`) belong to the other reader of the same port,
  VecCiiComplete; the port is shared deliberately, since both readers present the
  same tag in the same cycle and a second 16-to-1 entry mux would be pure cost.

  ===> `wr_en` (the table's "this member is real" bit, i.e. bit
  `wb_dst_offset` of `pvdest_grp_mask`) IS NOT `wb_wr_en` (the beat's own write
  enable). Both gate the same VRF write and they mean different things; the
  similar names are the tag table's and are kept rather than re-spelled, so the
  distinction is stated here instead.

  `io.wb_suppress` — Input Bool, meaning exactly "the writeback beat being
  consumed belongs to a killed tag: pop it, return its credit, suppress its
  effects". IT IS DRIVEN BY THE PARENT, VecCiiHost, AS A TWO-TERM OR:

      wb.io.wb_suppress := tags.wb_lookup.resp.killed || flush.io.kill_all

  and both terms are needed because they cover different cycles. `killed` covers
  every beat from the cycle AFTER the flush onward, which is almost all of them;
  `kill_all` covers the beat arriving IN the flush cycle, one cycle before `killed`
  is readable out of the registered table. Neither alone closes the window.

  ===> THE NAMING IS SETTLED AND THE OR BELONGS TO THE PARENT. VecCiiFlush
  exports exactly one bare combinational bit, `kill_all`; its own reject list
  forbids it — correctly — from exporting any per-channel suppress output. So
  there is no `wb_suppress` output anywhere to import, and this module does not
  take `kill_all` directly either: it reads ONE pre-computed decision, which is
  what lets it have no flush port, no `killed` vector and no age comparator.
  Earlier drafts of this file described the input as coming "from the flush
  sibling" and, in the dependencies section, called it `io.kill_all` — both are
  wrong about the same seam, and the container is the authority on it.

  ---- Placement outputs ----

  `io.vrf_write` — `Valid{ addr: UInt(vecPregSz.W), data: UInt(vLen.W),
  mask: UInt(vLenBytes.W) }`, VRF write port `W2`, the CII's single write port
  in the canonical `vrf-ports` partition. Same bundle shape as the LCB's
  `io.vrf_write` on `W0`/`W1`.

  `io.int_wb`, `io.fp_wb` — `Valid(new ExeUnitResp(xLen))`, the scalar-dest
  writeback paths of the `vec_pipeline_io` seam. Fire-and-forget with no `ready`,
  because the Writeback channel cannot be back-pressured: they must land on the
  DEDICATED scalar-dest write port and wakeup slot `usingRVV` adds (the
  port the `int_wb_snoop` width comment in hierarchy.yaml requires be counted,
  and which `numVecIrfWritePorts` already sizes off `usingRVV` alone),
  never on an arbitrated share of BOOM's `ll_arb`. `data` is in the register
  file's ARCHITECTURAL encoding — IEEE-754 for the FP case, whose recode into
  hardfloat is the consumer's (FpPipeline already recodes its long-latency write
  path). No format conversion happens in this module.

  `io.wb_credit` — Output Bool, one credit returned per popped beat.

  ---- Sibling-facing outputs (the `last`-bit seam) ----

  `io.beat` — Output `Valid{ tag, status }`, where `status` is the `wb_status`
  sub-bundle, matching VecCiiComplete's declared `io.beat` input field for field.
  Asserted in the cycle this module pops the beat, for a killed tag exactly as for
  a live one. It carries NEITHER `wb_data` NOR `wb_dst_offset`: fanning 256 bits
  into a module with no datapath is pure cost, and the 15-bit width is what makes
  the placement/completion split reviewable.

  ===> DRIVEN COMBINATIONALLY, NOT REGISTERED, for the reason in the
  performance section: the `last` beat's placement and its completion must be
  the same cycle. NEITHER SIDE REGISTERS IT — VecCiiComplete's port comment now
  says the same thing in the same words, and an earlier draft of that file
  calling this "Writeback's registered beat" is the wording this note exists to
  keep retired. A register on ONE side only is the failure being prevented: it
  would skew group-done a cycle AHEAD of the final `W2` write.

  `io.wb_beat` — Output `Valid(UInt(ciiTagBits.W))`, and `io.wb_last` — Output
  Bool. The tag of the beat being consumed and whether it is that tag's final
  beat, matching VecCiiFlush's declared inputs of the same names. VecCiiFlush uses
  the pair for its drain bookkeeping and watchdog; nothing here reads them back.

  ---- Not ports of this module (reviewer's reject list) ----

  No `VecGroupDone`, no `clr_rob`, no tag-free, no `fflags`/`vxsat` accumulator
  output — all four are VecCiiComplete's. No second VRF write port and no VRF read
  port of any number. No `ready` on any channel and no back-pressure input. No
  `vl`, `vtype`, `vstart` or `vxrm` input. No `rob_flush` and no `killed` vector.
  No `MicroOp` input: this module never needs the issued instruction, which is
  precisely why it cannot re-decode it.
  <|end_ports|>

  <|begin_logic|>
  Purely combinational, zero state, zero pipeline stages. `io.wb.valid` in a
  cycle produces the credit and the placement in that same cycle.

  ---- 1. Pop and credit, unconditionally ----

  //@req-spec-cii.e16
  `io.wb_credit := io.wb.valid`. Every presented beat is consumed the cycle it is
  presented — live tag or killed tag, `wb_wr_en` set or clear, `last` or not. A
  killed tag's beats drain on exactly the schedule a live one's do, so the pop
  never distinguishes them. // Unlike Src-Data, the host is the RECEIVER here, so
  draining costs one credit and no manufactured beat. Withholding the credit
  would stall the channel for every SURVIVING instruction behind the killed
  one and turn a squash into a permanent hang.

  ---- 2. Route by dst_kind ----

  //@req-spec-cii.g8
  //@req-spec-cii.g20
  `wb_status.dst_kind` is the ONLY routing selector, decoded one-hot into three
  mutually exclusive write enables: `CII_DST_VEC` enables `io.vrf_write`,
  `CII_DST_INT` enables `io.int_wb`, `CII_DST_FP` enables `io.fp_wb`. The selector
  arrives PER BEAT and is deliberately not a side-table field (cii.rst
  `cii-issue` says so explicitly), so this module never consults the entry to
  decide where a beat goes and cannot drift out of step with the VPU. Assert at
  most one enable per cycle and that `dst_kind` is never the reserved encoding.

  ---- 3. Vector destination: place the member into the group ----

  //@req-spec-cii.g1
  For `CII_DST_VEC`, `io.vrf_write.bits.addr := io.wb_lookup.resp.prn` — the
  member PRN the tag table resolved from `{tag, wb_dst_offset}` by INDEXING ITS
  MEMBER-PRN VECTOR, never by `pvdest_grp_base + wb_dst_offset`. A group's members
  need not be contiguous (the free list allocates a group without requiring
  contiguous PRNs), so a base-plus-offset address is not merely pessimistic, it
  names the wrong register. The resolve lives in the table because that is where
  the entry mux already is; this module contributes the offset and must not
  re-implement the mux from a group vector. Assert `io.wb_lookup.resp.wr_en` on
  every enabled vector write.

  That assertion guards a re-introducible bring-up bug: a SINGLE-REGISTER-
  DESTINATION op (vmv.s.x, reductions, mask-producing ops) emits exactly one
  beat at offset 0 and the upper members of the renamed group must be left
  alone. Placing beats strictly at the offsets the beats name — never
  broadcasting one across the group, never synthesising a write for a member
  no beat arrived for — is what keeps that case correct; the M2 cosim caught
  the corruption only once it began checking members beyond member 0.

  //@req-spec-cii.g3
  //@req-spec-cii.g4
  `io.vrf_write.bits.data := io.wb.bits.wb_data`, all `vLen` bits, unmodified.
  `io.vrf_write.bits.mask` is the ALL-ONES `vLenBytes` constant: the beat is
  fully formed, so every byte of the destination member is written. The mask
  field exists because the LCB needs per-byte assembly on `W0`/`W1`; on `W2` it
  is a constant and must stay one.

  ===> DO NOT DERIVE THIS MASK. A byte mask built from `vl`, `vtype.vta`,
  `vtype.vma` or `vstart` is the forbidden re-application of vl/vtype wearing a
  different hat, and it would corrupt every tail-/mask-undisturbed result the
  VPU already filled: the VPU applied the policy against the vtype it was
  issued with, and a second application here has no way to agree with it. This
  module has no `vl` or `vtype` input, so writing this bug means adding a port
  — which is on the reject list.

  ---- 4. Per-beat write enable ----

  All three destination enables are additionally qualified by
  `io.wb.bits.wb_wr_en`. A beat with it clear places nothing anywhere, while still
  popping, still returning its credit, and still appearing on `io.beat` — so it
  still completes the tag if it carries `last`. So the vector write enable is the
  four-term AND `io.wb.valid && wb_wr_en && dst_kind === CII_DST_VEC &&
  io.wb_lookup.resp.wr_en`, then gated by part 6.

  ---- 5. Scalar destination: INT or FP register file ----

  //@req-spec-cii.g6
  //@req-spec-cii.g7
  //@req-spec-cii.g16
  //@req-spec-cii.f42
  A handful of vector instructions write a scalar register instead of the VRF:
  `vmv.x.s`, `vcpop.m` and `vfirst.m` to an integer register, `vfmv.f.s` to an FP
  register. For those the beat goes to `io.int_wb` / `io.fp_wb` and NOT to the
  VRF — `io.vrf_write.valid` stays low, so no VRF port is consumed at all.
  `data := io.wb.bits.wb_data(xLen-1, 0)`: a scalar result occupies the LOW `xLen`
  bits and the upper `vLen - xLen` bits are DON'T-CARE — discarded, never
  sign-extended from bit `xLen-1` and never reduced in. The write is directed at
  `io.wb_lookup.resp.pdst`, the renamed PHYSICAL scalar destination recorded at
  issue, with `uop.rob_idx` from the same response; the coprocessor named no
  register, so the side-table is the only source. The `ExeUnitResp` carries the
  SCALAR WAKEUP with it — `uop.pdst` and `uop.dst_rtype` are what BOOM's INT and FP
  networks match on, so a consumer of a `vmv.x.s` result wakes through that
  machinery and no vector-specific wakeup path is added for scalar destinations.
  Drive `predicated` low, and `fflags` from `wb_status.fflags` for the FP case
  only; the INT case reports no FP flags.

  Base the carried `uop` on `NullMicroOp`, NOT on `DontCare`. This is a real
  `ExeUnitResp` and its consumers read uop fields this module has no opinion
  about, so an invalidated uop propagates X into live control: `core.scala`
  feeds `uop.br_mask` to `IsKilledByBranch` on the ROB writeback response, and
  `rob.scala` gates `rob_vconfig := uop.vconfig` on `uop.is_vl_producer`, where
  an X latches a garbage vtype into the committing row. Zero is not merely a
  safe default for `br_mask` but the CORRECT value — VecCiiFlush asserts
  `alloc_br_mask === 0` because IQ_V_ALU only grants past the PNR — and a CII
  arith op is never a vl producer.

  On the FP leg additionally drive `uop.v_eew` from `wb_lookup.resp.v_eew`.
  FpPipeline recodes this beat as `v_eew =/= 2` (single vs double) at its write
  port, and a `vfmv.f.s` result is SEW wide. `v_eew` CANNOT be taken from the
  issuing uop: VecDecode assigns it only on the memory lane, so an arith uop
  carries its decode default. The tag table carries the issuing op's SEW for
  exactly this purpose. Getting it wrong silently recodes an e32 result as a
  double — no assertion, cosim mismatch only.

  ---- 6. Killed-tag suppression ----

  //@req-spec-cii.e17
  All three destination enables are suppressed when `io.wb_suppress` is set. That
  is the ONLY effect this module suppresses; the beat is still popped, still
  credited, and still presented on `io.beat` and `io.wb_beat`. Because that term is
  the OR of the table's registered `killed` bit AND VecCiiFlush's combinational
  `kill_all`, no cycle exists in which a doomed tag's beat reaches `W2`, the INT RF
  or the FP RF — not even the flush cycle, which the `killed` term alone would miss.
  `killed` is idempotent and never cleared while a tag is live, so a second flush
  during a drain changes nothing here.

  The `wb_lookup` response also carries a `killed` bit. This module reads
  exactly ONE suppression source — `io.wb_suppress` — because two suppression
  terms that could disagree is worse than either alone: the failure is a silent
  VRF write to a reallocated PRN.

  ===> THE SELF-CHECK IS AN IMPLICATION, NEVER AN EQUALITY:
           assert(!io.wb_lookup.resp.killed || io.wb_suppress)
  i.e. `killed -> wb_suppress`. An EQUALITY (`killed === wb_suppress`) FIRES ON A
  CORRECT CASE: in the flush cycle the parent's `kill_all` term already sets
  `wb_suppress` while the registered `killed` bit is still clear, which is
  exactly the window the OR exists to cover. The implication is the whole of
  what is checkable here — that no beat the table has marked killed ever gets
  through — and the converse is not a property of this design.

  ---- 7. The segmented halves ----

  //@req-spec-cii.i1
  A segmented LOAD's coprocessor half TRANSPOSES `pvtmp` into `pvdest`: the LSU
  half writes the intermediate `pvtmp` group, `pvtmp`'s group-done wakes the
  coprocessor's `IQ_V_ALU` slot, and the coprocessor pulls `pvtmp` on the Src-Data
  side and returns transposed members. On THIS side that half is an ordinary
  vector-destination writeback — its beats name offsets into `pvdest` and parts 3
  and 4 place them with no special case at all. The transpose shows up here only
  as the fact that the sources came from one group and the destination is another.

  //@req-spec-cii.i4
  //@req-spec-cii.i8
  //@req-spec-cii.i11
  For a segmented STORE the roles reverse: the `pvtmp` group is the DESTINATION of
  the coprocessor half and the source of the LSU half, so that half's beats must
  land in `pvtmp`, not `pvdest`. This module needs no logic for the reversal
  because it writes `io.wb_lookup.resp.prn` and never names a group at all — the
  REQUIREMENT it places on the `tags` sibling is that the destination group an
  entry resolves against is whatever group THE COPROCESSOR writes: the granted
  uop's `pvtmp` for a segmented store's coprocessor half, `pvdest` otherwise,
  chosen once at tag allocation. VecCiiTagTable DISCHARGES THIS with a mux at
  allocation — `Mux(uop.is_shared && uop.uses_stq, uop.pvtmp, uop.pvdest)` into its
  single `pvdest_grp` field — so from here it is an ordinary `W2` write to an
  ordinary renamed group, and that group's group-done (VecCiiComplete's) is what
  later wakes the store's DGEN slot.

  ===> THE DESTINATION-GROUP CHOICE IS THE TAG TABLE'S, MADE ONCE AT ISSUE, AND
  MUST NOT BE RE-DERIVED HERE FROM `is_shared`/`is_store`. Two places deciding
  which group a beat lands in is how a segmented store silently writes the
  load-side group, and re-deriving it would need the uop, which this module
  deliberately does not have. The `wb_lookup` response's `is_shared` bit is for
  the ROB's "other half pending" flag and for trace — reading it to select a
  group is the reject. Note that the entry field is SPELLED `pvdest_grp` — the
  name is the tag table's and describes the common case; the mux behind it is
  what makes cii.i4/i8/i11 satisfiable at all, and without it the requirement
  this paragraph tags could not be met from anywhere in the design.

  //@req-spec-cii.i10
  The transpose itself happens INSIDE the coprocessor (its data transpose unit),
  so a segmented store's `wb_data` is ALREADY transposed and is written verbatim
  by part 3. This module performs no transposition, no lane crossing and no
  re-layout of `wb_data` under any `dst_kind` — the same verbatim rule as for
  arithmetic results, for the same reason: the host has no model of the segment
  layout and could not agree with the VPU's.

  ---- 8. Tracing and assertions ----

  Emit one guarded `VecTrace` line per beat, gated on the `vecTrace` plusarg and
  `!reset`, carrying the module name, `io.wb_lookup.resp.rob_idx`, the `tag`,
  `dst_kind`, `wb_dst_offset`, the resolved destination (`prn` or `pdst`) and the
  `last` / `wb_wr_en` / suppressed flags. A suppressed (killed) beat is traced
  too, marked suppressed — a drained beat leaving no trace is indistinguishable
  from a beat that never arrived, which is the hardest CII failure to debug.

  Assertions: the beat's tag must name a live entry (a beat for an unallocated tag
  is a protocol break, not a droppable event); at most one destination enable per
  cycle; `io.wb_lookup.resp.wr_en` set on an enabled vector write; the kill
  self-check as the IMPLICATION `io.wb_lookup.resp.killed -> io.wb_suppress` and
  never as an equality (part 6); and a
  scalar-destination beat must carry `last` with `wb_dst_offset === 0` — a scalar
  destination is one register, so a second scalar beat for one tag would mean the
  VPU is writing a group into a scalar register.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
One beat per cycle, sustained, with no cycle of latency and no stall condition.
Both halves of that are structural rather than aspirational: `CII_NUM_DST_WB`
is 1 so no more than one beat can ever be offered, and `W2` is statically
partitioned so it can never be denied. This is the property the credit-metered
Writeback channel depends on — it has no back-pressure line, so a module that
could stall would drop a result rather than delay it.

NO PIPELINE STAGE MAY BE ADDED HERE, AND `io.beat` IS NOT REGISTERED. The
placement of the `last` beat and the completion VecCiiComplete derives from it
must occur in the SAME cycle: VecCiiComplete's own performance section requires
its group-done, ROB clear, flags and tag free in "the same cycle VecCiiWriteback
pops the `last` beat", and the LSU-side analogue behaves the same way. A register
on `io.beat` would let a group-done — and therefore a dependent issue — precede
the `W2` write of the group's final member by a cycle. The VRF write lands at the
end of the pop cycle and the earliest dependent read is a cycle later, so
same-cycle completion is safe; one cycle of skew is not.

Area is negligible: a handful of enable terms, one `xLen` slice, and NO STATE at
all. The `vLen`-wide path is pure fan-out from the channel to the `W2` port, and
the member mux that would otherwise dominate lives in the tag table.
<|end_perf|>

<|begin_dependencies|>
VecBundles — `CiiWriteback` and its `wb_status` sub-bundle ({ `last`, `dst_kind`,
`vxsat`, `fflags` }), reused by name rather than re-spelled, so that this module
and VecCiiComplete cannot slice the 9-bit status differently.

VecTrace — the guarded trace helper. This module has a `rob_idx` (from the
side-table) but NO `MicroOp`, so it needs the bare-`rob_idx` form of the helper
rather than the `MicroOp` form — the analogue of the `traceDecode` variant that
exists because decode has no `rob_idx`. VecCiiComplete records the same need.

Binds to the `CII_DST_VEC`/`CII_DST_INT`/`CII_DST_FP` and `CII_NUM_DST_WB`
constants DERIVED from `tt_cii_caracal_pkg.svh` — the SV package is the
authoritative side of that contract.

===> THEIR CHISEL DECLARATION SITE IS NAMED HERE, because "derived, never
     redeclared" without a site is what produces the failure it forbids. They are
     declared ONCE, as a package-level `object VecCiiDstKind` in THIS file:

         object VecCiiDstKind {
           val VEC = 0.U(2.W)
           val INT = 1.U(2.W)
           val FP  = 2.U(2.W)
           val NUM_DST_WB = 1
         }

     Package-level, NOT `val`s inside the module body — VecCiiComplete needs the
     same encodings to tell a vector-destination completion from a scalar one,
     and a module-local `val` is unreachable from there, so it would have to
     mirror them a second time. Two independent mirrors of one SV enum drift
     silently and misroute a writeback rather than failing a build. The 2-bit
     width is `CiiWbStatus.dst_kind`'s, which VecBundles already derives from
     `VecBundlesConsts.ciiWbStatusBits`; do not restate it as a literal.

     Ideally these would sit in `VecBundlesConsts` beside `ciiWbStatusBits`,
     which exists for exactly this purpose. They do not, and the reason is blast
     radius, not principle: VecBundles is a dependency of every vector node in the
     tree, so adding three constants there forces a regeneration Phases A through
     E would have to be revalidated against. KNOWN GAP — migrate them at the next
     VecBundles regeneration, and delete this object in the same change.

VecRegFile — `io.vrf_write` binds to write port `W2` of the canonical `vrf-ports`
partition. Nothing here adds a port.

VecCiiHost — the parent. It unpacks the flat `wb_*` wires from
`tt_cii_host_wrap` into `io.wb`, forwards `io.wb_credit` back to the BlackBox,
carries `io.int_wb`/`io.fp_wb` out onto the `vec_pipeline_io` scalar-dest
writeback ports, and DRIVES `io.wb_suppress` as
`tags.wb_lookup.resp.killed || flush.io.kill_all`. That OR is the container's, not
this module's and not `flush`'s.

VecCiiTagTable (`tags`) — supplies the `wb_lookup` resolve. It owns the entry mux,
the `killed` bit this module's self-check implies against, and the requirement that
an entry's destination group is the group the COPROCESSOR writes — discharged by
its allocation-time mux, `pvtmp` for a segmented store's coprocessor half.

VecCiiComplete (`done`) — the sibling on the other side of the `last` bit,
consuming `io.beat`, which NEITHER of them registers. It owns group-done,
`clr_rob`, the tag free and the `fflags`/`vxsat` accrual, and their suppression for
a killed tag.

VecCiiFlush (`flush`) — exports the bare combinational `kill_all` that forms one
of the two terms of `io.wb_suppress`, and consumes `io.wb_beat`/`io.wb_last`. It
owns the kill window; the `killed` vector itself lives in `tags`. It exports NO
per-channel suppress output and this module takes NO `io.kill_all` — the input here
is named `wb_suppress` and is the parent's OR, nothing else.

Instantiates nothing.
<|end_dependencies|>

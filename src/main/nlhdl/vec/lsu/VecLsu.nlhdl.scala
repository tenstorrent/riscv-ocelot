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
  VecLsu — the vector load/store unit container: the vector LS AGEN stage, the six
  element queues, the drain side, the memory-ordering mechanisms, and the wiring
  that binds them to BOOM's Unified LSU. It contains all vector memory datapath
  and owns no datapath of its own.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecLsu.scala,
  package boom.v4.vec.generated.lsu, group vec_lsu.
  depends_on MicroOp, VecBundles, VectorParams, VecTrace.
  Instantiated ONCE, as `vlsu`, inside VecPipeline — which exists only when
  `usingRVV` is true, so in a vectors-off build this whole subtree is ABSENT, not
  tied off, and the emitted RTL is bit-identical to pre-Caracal BOOM v4.
  `usingRVV` is a Scala `Boolean` of `BoomCoreParams`, never a hardware `Bool`,
  and it is not rocket's `usingVector`.

  Seventeen children: `resv`; `ld_opnd`/`st_opnd`; `ld_msk`/`st_msk`;
  `ld_elem_agen`/`st_elem_agen`; `ld_range_agen`/`st_range_agen`; the six
  `VecElemQueue` instances; `ld_beat`/`st_beat`; `dgen`; `lcb`; `gcopy`; `arb`;
  `snoop`; `fwd`; `hold`; `squash`.

  ===> THE MODULE EXISTS TO UPHOLD ONE INVARIANT, AND EVERY STRUCTURAL CHOICE
       BELOW IS DOWNSTREAM OF IT. Address generation is cut BY PIPELINE POSITION,
       not by the inherited Packer/Skipper/Walker x load/store cross-product
       (six modules, 2110 lines, whose duplication had already produced divergent
       mask support between the load and store Packers). FILL side: `VecElemAgen`
       (SSI, one address per active element) and `VecRangeAgen` (unit-stride, ONE
       range entry). DRAIN side: `VecBeatExpander`, which coalesces just in time.
       DIRECTION IS A PARAMETER, NEVER A SEPARATE MODULE — each agen, each operand
       read, each mask streamer and each beat expander is instantiated twice so the
       two directions never arbitrate. The load-priority mux over a single shared
       unit is what silently dropped store grants in `addvector`.

  ===> NO MODULE IN THIS SUBTREE MAY HOLD STATE SCOPED TO "THE CURRENT
       INSTRUCTION", AND NONE MAY EXPORT A `busy` THAT GATES ISSUE. In-flight
       state lives in exactly THREE places, and plan GROUND RULE 6 WAS AMENDED to
       say three rather than two: (a) the six element queues — whose capacity is
       reserved at dispatch, in program order — (b) the LCB's per-PRN assembly
       entries, and (c) THIS MODULE'S PER-LDQ/STQ-ENTRY DESCRIPTOR PENDING TABLE
       (section 3b). (c) is the same KIND of state as (a) and that is why the rule
       admits it rather than being bent around it: one row per LDQ/STQ entry, the
       row indexed by a placeholder the dispatch-time reservation already
       guaranteed, so it is structurally un-overflowable; it is not scoped to "the
       current instruction" (a row belongs to a queue entry, and rows for several
       ops coexist); and it exports no `busy`. The rule is amended IN THE TEXT
       because gate H4 reviews against that text — a table appearing under a rule
       that enumerated two homes would be right to reject.
       This module therefore exports no `busy`, no `active`, no `fu_ready` and no
       credit toward any issue unit. Issue eligibility for a vector memory OP.v is
       "a reservation exists", decided at dispatch by `resv` and nowhere else.
       Gate H4 greps this whole subtree for a `busy` reaching an issue unit; a hit
       is a failed review regardless of measured performance. `vec_lsu_empty`, the
       LCB's `free_count`, `hold_ldq` and `dis_ok` are each explained below in the
       terms that distinguish them from a `busy`.

  Governing spec anchors: execution.rst `vector-ls-agen` and `vector-agen`;
  loadstore.rst `lsu-unified`, `ssi-queues`, `us-queue`, `store-data-queue`,
  `vec-load-algo`, `vec-store-algo`, `mem-order`, `vec-squash`, `elem-progress`,
  `dcache-arbiter`, `vector-bw-ceiling`; midcore.rst `group-done-wb`, `vrf-ports`,
  `midcore-segmented-load`; issue.rst `shared-store-chain`,
  `vec-queue-reservation`; case_study.rst `case-segmented-ls`, `case-vl-zero`;
  glossary.rst `glossary-terms`.
*/

<|begin_module|>

  <|begin_parameters|>
  This module introduces no tuning knob of its own. It exists to DERIVE the
  children's parameters from one place so two siblings cannot be configured
  inconsistently, and every value below is an elaboration-time Scala value.

  From `HasBoomCoreParameters`: `coreWidth`, `lsuWidth`, `numLdqEntries`,
  `numStqEntries`, `ldqAddrSz`, `stqAddrSz`, `robAddrSz`, `xLen`,
  `numIrfWritePorts`, `coreDataBytes`. From VectorParams through
  `HasVectorParams`: `vLen`, `eLen`, `vecPregSz`, `vlPregSz`, `maxMembers`,
  `vecVLSz`, `ssiQueueEntries`, `usQueueEntries`, `lcbEntries`,
  `dcacheArbiterMode`.

  ---- Derived, and passed down ----

  `queuePorts` = `lsuWidth` — the fill/consume lane count of every VecElemQueue
  instance, and the same number reaches `ld_beat`/`st_beat` as `nLanes`, `snoop`
  as `searchPorts` and `arb` as its lane count. `readPorts` is left at the
  child's default `ports + 1`: one read per drain lane plus the ONE shared read
  port the disambiguation and forwarding consumers share (see the shared-port mux
  in the logic section).

  //@req-spec-lsu.a13
  On `LargeBoomV4Config` and `MegaBoomV4Config` the dual-port L1 D$ lets two
  memory operations issue per cycle, and the vector address and data queues must
  be `2 x nOP.v` wide so two concurrent element accesses can be presented. That
  obligation is discharged HERE, by setting VecElemQueue's `ports` parameter to
  `lsuWidth` on all six instances rather than leaving it at its default of 1 — the
  queue itself is written against the parameter and cannot satisfy the requirement
  alone. `nLanes` on the beat expanders and `searchPorts` on the snoop are set
  from the same value, because a queue lane with no drain lane, or a search lane
  with no drain lane, would present an address the machine never translated.
  Require `dcacheArbiterMode` to agree ("single" implies `lsuWidth == 1`,
  "dual-dynamic" implies 2), failing elaboration on a mismatch rather than
  silently picking one.

  Queue payload widths, all derived at elaboration and never written as literals:
  `addrWidth` = `(new VecElemAccess).getWidth` for the two SSI address queues,
  `rangeWidth` = `(new VecRangeEntry).getWidth` for the two US address queues,
  `eLen` for `st_SSI_DATA_Q`, `vLen` for `st_US_DATA_Q`. Depths are
  `ssiQueueEntries` and `usQueueEntries`. `reserved` is true on all six;
  `isStore` is stated explicitly on all six (it has no default, deliberately);
  `hasXlatePass` is true on `st_SSI_ADDR_Q` and `st_US_ADDR_Q` only.

  `nKillClients` = 5 — NOT the child's default of 8 — and the ORDER IS FIXED
  because `kill_uop` and `kill` are positionally paired: 0 `ld_msk`, 1 `st_msk`,
  2 `ld_beat`, 3 `st_beat`, 4 `gcopy`. Those are exactly the children that expect
  a RESOLVED kill `Bool` "from the parent". `ld_elem_agen`, `st_elem_agen`,
  `ld_range_agen`, `st_range_agen`, `dgen` and `st_opnd`/`ld_opnd` take
  `brupdate`/`rob_flush` directly and evaluate BOOM's `IsKilledByBranch`
  themselves — VecElemAgen MUST, because it holds two uops with different
  `br_mask`s and one pre-resolved Bool can only ever be right for one of them —
  and `idx` (VecIdxGen) is killed by its parent agen, never by this unit.

  // ===> `VecSquashUnit`'s OWN PARAMETER COMMENT IS STALE AND THIS SITE IS
  //      AUTHORITATIVE. It still enumerates eight clients — "the four fill-side
  //      agens, `idx_gen`, `mask_stream` and the two `VecBeatExpander`s" — which
  //      predates the mask-streamer hoist to this level and predates the four
  //      agens taking `brupdate` directly. Corrected client set: the two mask
  //      streamers (now this module's instances, so they DO need a resolved kill
  //      from here), the two beat expanders, and `gcopy`. Five. When
  //      VecSquashUnit's text is next regenerated its comment must be brought to
  //      this list; until then, do not read a default of 8 as an instruction to
  //      wire three dangling clients.

  `isStore` is forwarded on both `VecScalarOperandRead`, both `VecElemAgen`, both
  `VecRangeAgen`, both `VecMaskStream` and both `VecBeatExpander` instances. It is
  stated at every instantiation site, never defaulted, because a defaulted
  direction gives one path the other's freeing semantics.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit pair, matching the hierarchy defaults:
  single `core_clk` domain, posedge `clock`, ACTIVE-HIGH SYNCHRONOUS `reset`. No
  second clock domain and no asynchronous reset appears anywhere in this subtree;
  the one active-low crossing in the design is in `tt_cii_host_wrap`, far from
  here.

  ---- Dispatch, issue and recovery (from VecPipeline, off `vec_pipeline_io`) ----

  `dis_uops` (`Vec(coreWidth, Valid(MicroOp))`) and `dis_fire`
  (`Vec(coreWidth, Bool)`) in, `dis_ok` (`Vec(coreWidth, Bool)`) out — passed
  straight to `resv`. These are the REGISTERED `ren2`/dispatch uops, never
  `dec_uops`.

  `iss_ld`, `iss_st` — `Input(Valid(MicroOp))`, the grants from `iq_v_load` and
  `iq_v_store`. NO READY, NO CREDIT, NO `busy` IN EITHER DIRECTION, and after
  decision D5 the reason is stronger than "two operand-read instances buy it": a
  grant is written unconditionally into section 3b's per-LSQ-entry table, whose
  row is indexed by a placeholder the reservation already guaranteed, so refusing
  a grant is not merely undesirable but unnecessary — the full state is
  unrepresentable.

  `brupdate` (`BrUpdateInfo`), `rob_flush` (unregistered `rob.io.flush.valid`),
  `rob_flush_kill` (its registered form, the cycle baseline lsu.scala rolls its
  LSQ pointers), `commit_valids`/`commit_uops` and `rob_head_idx`/`rob_pnr_idx`.

  ---- Register-file seams (cited by port number, none added) ----

  `int_rf_read_req`/`int_rf_read_rsp` — four lanes, two per operand-read instance
  (base `prs1` and stride `prs2` concurrently). EACH REQUEST LANE IS `Decoupled`,
  not a bare `Valid`, and that is the one interface change decision D5 makes on
  this seam. `PartiallyPortedRF` denies by INDEX PRIORITY —
  `ready := PopCount(io.arb_read_reqs.take(i).map(_.valid)) < numPhysicalReadPorts`
  — the vector lanes are appended LAST, and on Medium there are ~7-9 existing
  logical readers plus five vector lanes against FIVE physical ports. Denial is
  therefore ROUTINE, not exceptional, and a lane whose `ready` is ignored returns
  the wrong base or stride with no assertion anywhere. The `ready`s are consumed
  by section 3b's table, which simply keeps the row presented; they reach no issue
  queue. `int_wb_snoop` (`numIrfWritePorts` of {addr, data}) for the stale-base
  forward.
  NO `fp_rf_read_req`/`fp_rf_read_rsp` ON THIS MODULE AT ALL. Decision D4 DELETED
  the store-side FP reader `st_opnd` used to carry: no RVV store form takes an FP
  scalar operand — store data is always `vs3`, and a store's scalar operands are
  `rs1` (base) and `rs2` (stride), both integer. The single FP read lane left on
  `vec_pipeline_io` belongs to `VecCiiIssue`, and neither operand-read instance
  here has one.
  `vl_read_addr`/`vl_read_data` — two lanes, one per operand-read instance.
  `vl_wb` — `Output(Valid({pvl, vl}))`, VlRegFile's `W_lsu` port, with EXACTLY ONE
  driver inside this module (see the `vleff` paragraph).

  VRF ports, per the canonical table in midcore.rst `vrf-ports`, by number:
  `vrf_r0` (load index), `vrf_r1` (load mask), `vrf_r2` (`stale_pvdest`),
  `vrf_r3` (store data), `vrf_r4` (store mask AND store index — one port, two
  readers, muxed here), `vrf_w0` and, at `lsuWidth = 2`, `vrf_w1`. Each read is a
  `Valid(addr)` out with the data returned REGISTERED one cycle later; each write
  is `Valid(addr, data, byte-mask)`. NOTHING HERE ADDS A VRF PORT.

  ---- The Unified LSU seam ----

  `lsu_vec` — the `VecLsuCoreIO` bundle named by the `vec_pipeline_io` interface
  entry. IT IS DECLARED IN `lsu.scala`, BESIDE `LSUCoreIO`, IN PACKAGE
  `boom.v4.lsu`, AND THIS MODULE BINDS TO THAT ONE DECLARATION BY NAME. It is not
  declared here and a second declaration anywhere is a duplicate to reject. Two
  reasons settle it that way rather than the other: `VecBundles.VecPipelineIO`
  already carries a `lsu_vec: VecLsuCoreIO` field typed against a `boom.v4.lsu`
  type, and the LSU owns the LDQ/STQ pointers and placeholders the bundle
  carries — the producer of the state should own the type that exposes it. This
  module is a CONSUMER of the type and the counterparty on the bundle, nothing
  more. It remains an AGGREGATE of bundles the children already declared, not a
  new vocabulary: `LsuResourceClaim` (declared in VecDcacheArbiter's file) x
  {`scalar_demand`, `scalar_avail`, `vec_claim`},
  `vec_fire`, `dmem_req_ready`, the LCAM presentation `VecLcamSearch` and the
  load-side search context, `stq_addr_matches`/`stq_forward_matches`,
  `vst_addr_match`, `ldq_valid`/`ldq_alloc`/`stq_vec_valid`/`stq_alloc`, the four
  LSQ pointers `ldq_head`/`ldq_tail`/`stq_commit_head`/`stq_tail`, the LDQ/STQ
  element-cursor read/write pair (`elem_next`, `elem_done`, `fault_elem`), the
  per-STQ-entry committed flag, `fwd_resp`, `replay`, `clr_bsy` and
  `fencei_rdy_vec`. A second declaration of any of those bundles is a duplicate to
  reject.

  ---- Completion and exception, out ----

  `vec_clr_bsy` — two of the three `numVecClrPorts` lanes are driven from here:
  lane 0 by `lcb.io.group_done`, lane 2 by `gcopy.io.group_done`. Lane 1 is
  VecCiiComplete's and is not visible here. Lanes, never an arbiter: neither
  producer can be back-pressured.
  `vec_rob_flags` accompanies those lanes and carries nothing from the LSU today.
  `vec_clr_unsafe` — `Output(Valid(robAddrSz))`, the single group-safe event.
  `vec_xcpt` — `Output(VecException)` {valid, rob_idx, cause, badvaddr}. No
  element index: a faulting vector memory op traps with `vstart = 0`.
  `lsu_fencei_rdy_vec` — `Output(Bool)`, the `vec_lsu_empty` term.

  ===> AND THAT IS THE WHOLE INTERFACE. There is deliberately no `busy`, no
  `active`, no `grp_active`, no `fu_ready` contribution, no per-instruction status
  and no ready line toward any issue queue. `dis_ok` is a DISPATCH-stage capacity
  answer consumed in program order; `lsu_fencei_rdy_vec` gates only `is_unique`
  dispatch through BOOM's pre-existing `fencei_rdy`. Neither is scoped to an
  instruction and neither reaches an issue unit.
  <|end_ports|>

  <|begin_logic|>

  ---- 1. What this stage is, and where it sits ----

  //@req-spec-agen.a1
  //@req-spec-agen.b2
  The vector LS AGEN stage sits BETWEEN the `IQ_V_LOAD`/`IQ_V_STORE` issue queues
  and the Unified Load/Store Unit: its only inputs from the issue side are
  `iss_ld` and `iss_st`, and its only outputs toward memory are the arbiter's
  `vec_fire` and the LCAM presentation, both through `lsu_vec`. The first vAGEN
  stage is consequently AFTER the issue stage — `ld_opnd`/`st_opnd` are granted by
  those two queues directly, with no intervening buffer.

  //@req-spec-core.c10
  //@req-spec-agen.a2
  An `OP.v` is a SINGLE uOP through decode, rename, the ROB and issue; Caracal
  performs no frontend cracking. The expansion of one `OP.v` into `nOP.v` happens
  INSIDE THIS MODULE AND NOWHERE ELSE — in `ld_elem_agen`/`st_elem_agen` on the
  fill side for the strided/indexed/segmented classes, and in `ld_beat`/`st_beat`
  on the drain side for unit-stride. The cracked accesses reach memory at ELEMENT
  granularity. No other module in the machine may expand a vector memory uop, and
  nothing outside this subtree may read the `nOP.v`-scoped cursor fields of a uop.

  //@req-spec-agen.a3
  Cracking is by EMUL as well as by element: every emitted `nOP.v` carries WHICH
  destination PRN and WHICH BYTE OFFSET WITHIN THAT PRN it reads or writes. The
  member number is `byte_pos >> log2(vLen/8)` and the offset is
  `byte_pos & (vLen/8 - 1)`, both stamped by the agen that emitted the access and
  read back by the LCB (loads) or the store data path. Those two fields, not the
  element index, are what let an out-of-order response be placed, and the group
  they index is `pvdest` normally and `pvtmp` for a segmented access.

  //@req-spec-agen.a6
  This one stage contains the LOAD vAGEN, the STORE vAGEN and the store vDGEN:
  `ld_opnd` + `ld_elem_agen` + `ld_range_agen` are `ld_vAGEN_1`; `st_opnd` +
  `st_elem_agen` + `st_range_agen` are `st_vagen_1`; `dgen` is `st_vdgen` and runs
  ALONGSIDE the store vAGEN, granted separately on `FC_DGEN`. Vector `OP.v`s
  issued by the CII IQ are forwarded to the coprocessor and never enter here.

  ---- 2. Two stages, and the class-to-agen routing ----

  //@req-spec-agen.b1
  //@req-spec-agen.b4
  The vAGEN is SPLIT INTO TWO STAGES, and the split is the reason this module has
  a fill side and a drain side rather than one address generator. Stage 1 — the
  four agen instances — contains ONLY the Skipper and Walker generators, absorbed
  into `VecElemAgen`: in stage 1 only strided, indexed and segmented accesses have
  effective addresses calculated and expanded into `nOP.v` bundles. A unit-stride
  `OP.v` is encoded by `VecRangeAgen` into a single `nOP.v` carrying the effective
  base, the effective stride and `is_unit_stride`, and the stage-2 Packer —
  `VecBeatExpander` — generates its effective accesses JUST IN TIME at the queues.
  There is no Packer on the fill side; putting it there is what made every
  `addvector` access one beat per element.

  //@req-spec-agen.b3
  Load `OP.v`s are issued to `ld_vAGEN_1` and store `OP.v`s to `st_vagen_1`:
  `iss_ld` reaches `ld_opnd` only and `iss_st` reaches `st_opnd` only, and each
  operand-read result is broadcast to that direction's TWO agens, which select on
  the static access class carried by the uop. `VecElemAgen` takes
  `v_is_strided || v_is_indexed || v_is_segment`; `VecRangeAgen` takes
  `v_is_unit_stride || v_is_whole_reg || v_is_mask`. Assert the two selections are
  mutually exclusive and jointly exhaustive over `is_vec && (uses_ldq ||
  uses_stq)`, and assert each instance sees only its own direction.

  THERE IS NO ROUTING MUX IN THIS MODULE AND NO AGEN MAY EXPECT PRE-ROUTING. Both
  agens of a direction observe the SAME broadcast descriptor and each self-selects
  on the access class. That is cheaper than a demux, and it is what execution.rst
  means by "the generator is selected by access class, not by direction"; a
  pre-routing mux here would be a SECOND decision about the same class bits, taken
  in a module that has no business knowing the RVV encoding. Nothing here
  re-decodes `uop.inst` — the class fields come from `VLSDecode` on the uop. Note
  the consequence for `vleff`, which is architecturally UNIT-STRIDE: its `OP.v`
  self-selects into `VecRangeAgen`, not into `VecElemAgen`.

  ---- 3. The mask streamers live at THIS level ----

  `ld_msk` and `st_msk` are instantiated here, not inside the agens, because the
  unit-stride path needs a mask reader (spec-agen.e12, VecRangeAgen has none) and
  `R1`/`R4` may have EXACTLY ONE reader each (spec-vrf.g18). This module therefore
  owns their start interface: on an accepted `OP.v` of either class it drives
  `op`/`op_masked`/`op_vl` from that direction's `xx_opnd.out` (`op_masked` from
  `v_is_masked`, `op_vl` from the resolved `vl`), and routes
  `staged`/`ahead`/`skip_log2`/`all_inactive` to that direction's element agen,
  taking `step`/`skip` back from it, while `us_mask` goes to that direction's
  range agen as the `mask`/`vm` half of its `io.scalar` input.

  A streamer holds ONE `OP.v`'s cursor per direction, so a direction's element
  walk and its range entry cannot be in the streamer at the same time. That makes
  a direction's `OP.v` hand-offs serialize at THIS level, which is why the
  descriptor table of section 3b exists rather than an assertion that it cannot
  happen. For a unit-stride op the streamer loads its latch and retires the next
  cycle, since there is nothing to walk, so its occupancy is one cycle.

  The `R4` MUX IS THIS MODULE'S, AND IT IS THE ONLY BACK-PRESSURE ON THE STORE
  INDEX READ. `R4` serves BOTH the store mask read (`st_msk.mask_rd_req`) and the
  store index read (`idx` inside `st_elem_agen`, passed through the agen's
  `vrf_read_req`/`vrf_read_gnt` pair), and it must reach `VecRegFile` as EXACTLY
  ONE request — `VecRegFile` provides no `ready` on a read port, so the hold-off
  cannot live there. Static priority: the once-per-`OP.v` mask read wins, so
  `st_elem_agen.vrf_read_gnt` is `!st_msk.owns_port` and `VecIdxGen`'s request
  simply waits a cycle. That cannot starve the index read, and the argument is
  structural rather than statistical — the agen may not start an element access
  whose mask bit is not staged, so the first index member cannot be needed before
  the cycle after the mask read completes, and the mask read happens once and never
  contends again. On the load path the two reads are on different ports (`R0`
  index, `R1` mask), `R0` has exactly one reader, and
  `ld_elem_agen.vrf_read_gnt` is constant true.

  ---- 3b. The descriptor pending table: ONE structure, TWO hazards, and the
           amendment to ground rule 6 that admits it ----

  THE `iss` TO `xx_opnd` TO `io.op` CHAIN HAD NO `ready` ANYWHERE, AND THAT LEFT
  TWO INDEPENDENT HOLES WITH ONE CAUSE — no back-pressure on the operand-read to
  agen chain. Both are closed here, by one structure, and it is worth naming them
  separately because they were reported separately and either alone would justify
  the table:

  (i) MID-WALK ARRIVAL. `VecScalarOperandRead` overwrites its stage register every
  cycle and publishes no readiness, so an agen must latch unconditionally; but an
  SSI element walk takes up to `vl` cycles, and `IQ_V_LOAD`/`IQ_V_STORE` grant the
  oldest READY entry with nothing stopping a second grant on the very next cycle.
  A second descriptor can therefore arrive at an agen still walking the first, and
  `VecElemAgen`'s one-deep pending slot covers exactly ONE such arrival. The
  dispatch-time reservation does NOT cover this: it guarantees QUEUE CAPACITY,
  which is a different resource from "the agen can accept a new descriptor
  mid-walk", and conflating the two is what makes the hole easy to miss.

  (ii) INT-RF READ DENIAL, WHICH IS THE ROUTINE CASE AND NOT THE EXOTIC ONE.
  `PartiallyPortedRF` denies reads BY INDEX PRIORITY —
  `ready := PopCount(io.arb_read_reqs.take(i).map(_.valid)) < numPhysicalReadPorts`
  — the vector lanes are appended LAST, and on Medium ~7-9 existing logical
  readers plus five vector lanes contend for FIVE physical ports. So a denial is
  the expected steady-state event, not an exception, while
  `VecScalarOperandRead` was written for "UNARBITRATED ports with no ready line".
  A denied read that is not retried yields a WRONG `base` or `stride` — every
  element address of that `OP.v` off by an unknown amount, with no assertion
  anywhere and no hang to notice.

  THE STRUCTURE. One PENDING DESCRIPTOR TABLE at this level: one row per LDQ entry
  on the load side, one row per STQ entry on the store side, in the same shape as
  `VecQueueReservation`'s table.

  WHAT A ROW HOLDS, ENUMERATED, BECAUSE A BLANKET `MicroOp` COPY PER LSQ ENTRY IS
  REAL AREA FOR FIELDS THIS PATH NEVER READS: `valid`, `br_mask`, the per-lane
  read-grant bits (see the accumulation rule below: the AUTHORITATIVE per-lane
  outstanding state is `VecScalarOperandRead`'s `rr_need`; what a row holds is the
  presented/accepted status this table needs to pick the next row, never a second
  copy of the consumer's progress), and the uop fields the operand read and the agens actually
  consume — `prs1`, `prs2`, `pvl`, `pvm`, `pvs2`, `pvs3`, `pvdest`, `pvtmp`,
  `stale_pvdest`, the `v_*` width/class fields (`v_eew`, `v_idx_eew`, `v_emul`,
  `v_seg_nf`, `v_is_*`, `v_is_masked`), `is_vec`/`is_shared`, `uses_ldq`/
  `uses_stq`, `ldq_idx`/`stq_idx` and `rob_idx`. The presented payload is
  reconstituted into the `MicroOp` shape `xx_opnd` already takes. IF A DOWNSTREAM
  CONSUMER NEEDS A FIELD NOT ON THAT LIST, ADD IT TO THE LIST — a field left
  don't-care in the reconstituted uop is exactly the failure this enumeration
  exists to prevent, and it presents as a mis-routed or mis-widthed access rather
  than as an X.

  ACCEPT IS UNCONDITIONAL. `iss_ld`/`iss_st` are written into the row named by
  that uop's `ldq_idx`/`stq_idx` in the grant cycle, with NO qualification of any
  kind: no `busy`, no ready toward the issue queue, no dropped grant. That is
  sound BY CONSTRUCTION rather than by argument — the row is indexed by an LDQ/STQ
  placeholder the dispatch-time reservation ALREADY guaranteed for this `OP.v`, and
  one placeholder holds one op, so "the table is full when a grant arrives" is not
  a representable state. Assert on a write to an already-valid row: that would mean
  two `OP.v`s claimed one LSQ placeholder, which is a reservation bug, not a
  capacity event. Each row holds `valid`, the granted uop (its `br_mask` included)
  and the per-row presented/accepted status below; nothing else. It does NOT hold a
  per-lane read-progress mirror — that would be two places recording one fact, and
  the consumer's copy is the one the RF handshake actually advances.

  PRESENTATION IS QUALIFIED, AND THAT IS WHERE BOTH HAZARDS ARE ABSORBED. Each
  cycle this module presents AT MOST ONE row per direction, choosing the OLDEST
  valid row with BOOM's existing `IsOlderLSU`/`EntryValidFromAge` against that
  queue's head, and presents it only when BOTH:
    - that direction's MASK STREAMER IS FREE (its cursor retired, or never
      started) — which covers hazard (i), because every `OP.v` of a direction
      passes through that direction's streamer whatever its class, so streamer
      occupancy IS the direction's hand-off gate and no new port on any agen is
      needed to observe it. The term is DERIVED HERE from the `op` this module
      drives and the `done` it receives — one tracking bit per direction —
      because `VecMaskStream` exports no `busy` or `occupied` and must not gain
      one; and
    - that row's INT AND VL READS HAVE BEEN GRANTED — which covers hazard (ii).
      The reads are still ISSUED BY `xx_opnd`, combinationally off the presented
      uop, exactly as it already does off `iss.bits`; what changes is that its
      `ready` toward this table IS "every read lane this descriptor needs has now
      fired", which is NOT the same as "all granted in one cycle" — see the
      accumulation rule immediately below.

      // ===> A PARTIAL GRANT ACCUMULATES; IT IS NOT RETRIED IN FULL (decision
      //      D5, retry model (a)). Lane 0 (`prs1`, the base) sits ahead of lane 1
      //      (`prs2`, the stride) in `PartiallyPortedRF`'s index priority, so
      //      "base granted, stride denied" is an ordinary cycle. `prs1` fires and
      //      STAYS fired; only the lanes still outstanding re-request. The
      //      per-lane hold lives in `VecScalarOperandRead` (`rr_need` per lane,
      //      `rr_data` per lane, the address held from `rr_uop`), which is the
      //      idiom every scalar EU already uses — hold the address until `fire`.
      //
      // ===> AND THIS TABLE MUST THEREFORE DROP `valid` TOWARD THE OPERAND READ
      //      ONCE A ROW IS ACCEPTED, rather than leaving it presented. An earlier
      //      revision said a denied row "simply stays presented and re-requests
      //      next cycle"; combined with the accumulating hold that is not merely
      //      redundant, it BREAKS the consumer's own check — it keeps `iss.valid`
      //      high on a descriptor already latched in `rr_uop`, and
      //      `VecScalarOperandRead` asserts `!(iss.valid && rr_valid &&
      //      rr_need.orR)` precisely to catch a second descriptor arriving while
      //      one is still outstanding. Presentation is a HAND-OFF, not a
      //      continuous request: present until accepted, then go quiet and let
      //      the consumer finish its lanes.
      //
      // WHY ACCUMULATE RATHER THAN RETRY, since retry is the simpler state. The
      // vector lanes are appended LAST in the RF's index priority and there are
      // ~7-9 existing logical readers against 5 physical ports on Medium, so
      // denial is routine, not exceptional. Retry-in-full requires base AND
      // stride to win in the SAME cycle against all of that; under sustained
      // scalar pressure that is a livelock, not a slow path, and it also
      // re-serializes the very read the 3 -> 5 seam widening existed to
      // parallelize. Accumulation makes progress MONOTONIC: each lane fires once
      // and stays fired, so the worst case is bounded by the unluckiest single
      // lane instead of by the coincidence of all of them.
      // The "half-read descriptor" objection is real but is a KILL question, not
      // a correctness-of-read question, and it is answered where the state lives:
      // `VecScalarOperandRead`'s kill clears `rr_need` so a squashed uop stops
      // consuming arbitration, and a read has no side effect, so a lane that
      // fired for a killed descriptor has simply wasted a port cycle.

  THE GRANT CYCLE IS BYPASSED SO THE TABLE COSTS NO LATENCY IN THE COMMON CASE. A
  grant whose row is being written this cycle is ALSO eligible for presentation
  this cycle, combinationally around the row, whenever both qualifications above
  already hold — which is the steady state, since the streamer is normally free
  and a read is normally granted. The read request is therefore still driven in
  the grant cycle, as it was before this table existed, and the row is consulted
  only when the fast path could not fire. The bypass adds no path that did not
  exist: `VecScalarOperandRead` already drove `int_rf_read_req` combinationally
  from `iss.bits`. Express the row's next state as UNCONDITIONAL WRITE ON GRANT
  followed by CLEAR ON FIRE, in that priority, so a bypassed grant's row is
  written and cleared in the same cycle and never becomes visibly valid. Do NOT
  instead gate the write on "the bypass did not fire": that makes the descriptor's
  survival depend on two combinational decisions agreeing, and the first time they
  disagree — a read denied in the very cycle the bypass looked free — the
  descriptor is gone with no assertion.

  So the link UPSTREAM of `VecScalarOperandRead` — this table to `xx_opnd` — is
  the one that becomes `Decoupled`: the table presents, `xx_opnd` accepts, the row
  clears on the fire. THE DOWNSTREAM CONTRACT IS UNCHANGED AND MUST STAY SO:
  `xx_opnd.out` to the agens, to the mask streamer's start and to the range agen
  remains a `Valid` that its consumer LATCHES UNCONDITIONALLY. Every agen was
  written against exactly that, and adding a ready there would rebuild the
  queues-to-issue back-pressure path the dispatch-time reservation exists to
  remove. `VecElemAgen`'s own pending slot is then provably never occupied —
  assert it stays free rather than deleting it, so a wiring regression shows up as
  a firing assertion instead of a silently lost descriptor.

  ===> GROUND RULE 6 IS AMENDED TO THREE LEGAL STATE LOCATIONS, AND THIS TABLE IS
       THE THIRD. The rule previously enumerated exactly two homes for in-flight
       vector-LSU state; it now reads: (a) the six `VecElemQueue` instances,
       (b) the LCB's per-PRN assembly entries, and (c) this table. (c) is
       admissible because it is the SAME KIND of state as (a), on all four terms
       the rule cares about: it is scoped to a QUEUE ENTRY and not to "the current
       instruction" (rows for several ops coexist, and a row outlives no
       placeholder); its capacity is RESERVED AT DISPATCH by the same mechanism;
       it is STRUCTURALLY UN-OVERFLOWABLE, so it needs no occupancy answer; and it
       EXPORTS NO `busy`. The amendment is stated in the plan text and restated
       here on purpose — gate H4 reviews against that text, and a reviewer meeting
       a table under a rule that named two homes would be right to reject it.

  ===> AND THE TABLE EXPORTS NOTHING. It does not reach `iq_v_load`/`iq_v_store`,
       it produces no ready, no credit and no occupancy, and a grant is never
       refused because of it. QUALIFYING THE `FC_AGEN` GRANT IS THE REJECTED
       ALTERNATIVE — the tempting one, since it needs no table at all. It is a
       `busy` reaching an issue unit under another name, which is EXACTLY what
       gate H4 greps for, and it is a failed review regardless of what it
       measures. Rows carry `br_mask`, updated by `GetNewBrMask` every cycle, and
       are invalidated by `IsKilledByBranch`/`rob_flush` exactly as an issue slot's
       uop is — the same discipline, no new kill mechanism. A row killed with reads
       outstanding drops its requests in the same cycle; a response returning for
       a killed row is discarded.

  ---- 4. Reservation and the six queues ----

  `resv` is driven from `dis_uops`/`dis_fire` and its `resv_out` regions are
  routed to the six queues' reservation claim ports; its four lookup and release
  lanes are FIXED — 0 `ld_elem_agen`, 1 `st_elem_agen`, 2 `ld_range_agen`,
  3 `st_range_agen` — and a denied release is never retried. `resv`'s own
  occupancy counters are AUTHORITATIVE for the grant; each queue's `avail`/`tail`
  outputs are consumed here only as assertions that the two agree, because two
  sources of truth for free space is how a queue comes to be over-subscribed.

  LOADS UNDER-RESERVE AND STREAM; STORES RESERVE THE WORST CASE. Nothing in this
  module may assume a load's region is big enough for its whole active element
  set. A LOAD's reservation is `min(worstCase, ldResvMembers * vLen/eew)` entries
  with `ldResvMembers` = 4 from VectorParams (decision D9/D10); a STORE still
  reserves the full `worstCase = EMUL * vLen/eew`, because
  `VecQueueReservation`'s four-step deadlock argument depends on it. The
  consequence here is that a load's fill side can run out of region while
  elements remain, and the ONLY thing that makes room is the DRAIN freeing
  entries — `VecElemQueue` does within-region circular reuse and NEVER extends
  past its tail, which is what keeps this deadlock-free: an older load's region
  sits ahead of a younger one's, drains first, and refills into its OWN region, so
  no younger reservation can block it. That is `spec-lsu.b11`'s streaming
  precondition, and it is live only because loads under-reserve.

  // ===> SO DO NOT ADD AN ASSERTION THAT A LOAD'S `resv_count` COVERS ITS
  //      ELEMENT COUNT. It was the natural check while both directions reserved
  //      the worst case, and it now fires on every long load. The equal-count /
  //      one-shared-base assertion of the SSI pair is a STORE-SIDE assertion and
  //      is stated as such below. Squashability is untouched: a load's region is
  //      still contiguous and program-ordered, just smaller, so section 9's
  //      rollback arithmetic is unchanged.

  THE ADDRESS AND DATA REGIONS OF A US STORE ARE NOT IN IDENTITY CORRESPONDENCE,
  and this is the seam most likely to be mis-generated. `st_US_ADDR_Q` holds ONE
  range entry per store while `st_US_DATA_Q` holds one full `vLen` entry PER GROUP
  MEMBER, so a US store claims 1 address entry and `v_emul * nf` data entries. The
  reservation therefore carries a base PER QUEUE SLOT (`resv_out` slot 0 address,
  slot 1 data) and the equal-count/one-shared-base assertion holds for the SSI
  pair ONLY. `us_data_base` on the range entry is filled from slot 1's base, and
  the surplus release for a US store trims the two queues by different counts.

  THE ADDRESS AND DATA HALVES OF AN SSI STORE ARE PAIRED BY ABSOLUTE INDEX, NOT BY
  A COUNT EACH SIDE KEEPS. `st_elem_agen.elem_pub` publishes, per surviving
  element, `{q_idx, elem_idx, seg_idx, active, first, last}` where `q_idx` is the
  ABSOLUTE entry index it wrote the address to; this module routes that stream into
  `dgen.io.cursor` and `dgen` WRITES `st_SSI_DATA_Q` AT THAT SAME ABSOLUTE INDEX
  rather than deriving an ordinal from its own emitted count. The two are
  interchangeable only while both sides agree about which elements survive, and the
  whole point of publishing the cursor is that DGEN never re-evaluates the mask: a
  derived ordinal reintroduces exactly the drift `VecDgen`'s own spec warns about,
  where one disagreement shifts every later ordinal on one side and every
  subsequent element writes the wrong address. The identity is sound because
  `resv` claims an SSI store's address and data regions with equal counts and equal
  bases; assert those bases agree, and assert every accepted `elem_pub` beat is
  matched by exactly one data enqueue at the same index.

  THE US STORE'S PER-MEMBER DATA CAPTURE HAS ONE MECHANISM, AND IT IS `VecDgen`'S.
  `st_range_agen.io.st_data` is an AGREEMENT ASSERTION ONLY: this module ties its
  `ready` PERMANENTLY HIGH and uses the port to check that the range agen and
  `dgen` name the same member sequence. IT CARRIES NO DATA PATH — nothing
  downstream consumes it, no queue is written from it, and removing its consumer
  would change no stored byte. `VecDgen`'s own `members_used`-derived sequence is
  the single mechanism and must remain so, because that TOTAL-BYTE bound is the M1
  phantom-member fix: a hardcoded or externally-supplied 8-member sequence streamed
  members past the end of a 1-member op and then stalled. Two mechanisms for one
  member sequence is how they come to disagree, and the disagreement corrupts store
  data rather than hanging.

  // The port is declared `DecoupledIO` with a permanently-high `ready`, which is
  // the same shape A57 condemned on `io.req`. It is left as declared because this
  // file is the side that states the tie-off and the two must agree; narrowing it
  // to a `Valid` is a joint VecLsu/VecRangeAgen/VecDgen change, flagged in the
  // report, not taken unilaterally here.

  Head-side reclamation is driven from here: on the deallocation of a vector LDQ
  or STQ placeholder, this module presents that index to `resv`, which invalidates
  the row and echoes the region's base and count, and the echo drives each queue's
  region-free command and `resv`'s own occupancy decrement in the SAME cycle. That
  keeps reclamation in program order at the queue head with no new state here —
  the retiring index is the LSU delta's `ldq_head`/`stq_head`, not a counter of
  ours.

  ---- 5. Drain, and the store's TWO passes ----

  //@req-spec-lsu.j5
  The vector LOAD path is IDENTICAL to the scalar load path apart from exactly
  three things, and no fourth difference may creep in: addresses are drained from
  the dedicated vector queues (per element out of `ld_SSI_ADDR_Q`, or by running
  the stage-2 Packer over the single `ld_US_ADDR_Q` entry), responses are
  coalesced in the LCB, and per-element progress is tracked in the LDQ entry's
  element cursor. Dispatch, the eligibility rules for a queue entry to drain, the
  fire (`will_fire_load_agen_exec`, TLB + D$ + LCAM together on one grant) and
  writeback all reuse the scalar structures unchanged.

  //@req-spec-lsu.j7
  //@req-spec-lsu.j8
  A vector store's EXECUTE stage drains addresses from `st_SSI_ADDR_Q` /
  `st_US_ADDR_Q` and does TLB AND LCAM ONLY: `st_beat`'s request asserts
  `uses_tlb` and `uses_lcam` and CLEARS `uses_dcache` while `is_write_pass` is
  low, the translated physical address is written back into the same entry through
  its translate-pass update port, and the entry is retained. ALL ACTIVE ELEMENT
  ADDRESSES ARE TRANSLATED PRE-COMMIT, so any page or access fault is detected and
  reported precisely BEFORE the store commits; the completion condition of section
  6 is what makes "all" true rather than "as many as were drained".

  A US STORE RUNS TWO PASSES OVER THE SAME RETAINED RANGE ENTRY, and this module
  owns the transition. Pass 1 is pre-commit TLB + LCAM; pass 2, entered when the
  STQ entry's committed flag rises, is TLB + D$. This module RESETS the entry's
  element cursor between the passes and drives `is_write_pass` from the committed
  flag; `st_beat` only ever advances a cursor and holds no notion of commit. THE
  EXECUTE PASS IS NOT A FREEING EVENT: `us_pop` stays low through pass 1, the
  entry's `filled` bit is untouched (a store consume never clears it), and the
  region is reclaimed only by section 4's post-commit reclamation. Assert that
  `is_write_pass` never falls for an entry that has already raised it, and that
  no D$ write is ever requested for an uncommitted STQ entry.

  //@req-spec-lsu.j11
  The vector STORE path is otherwise identical to the scalar store path: the only
  differences are draining address and data from the dedicated vector queues and
  the pre-commit translation of the whole active element range. Commit, the
  `stq_execute_queue` drain and the D$ write itself are BOOM's, unchanged.

  //@req-spec-lsu.k10
  //@req-spec-rob.d13
  Both drains present their beats to `arb`, which reuses THE SCALAR CACHE PORT
  rather than a `VLEN`-wide vector cache interface: one `dmem.req` lane per
  `lsuWidth`, the same interface, the memory subsystem unchanged. Peak vector
  memory bandwidth is therefore `lsuWidth * coreDataBytes` per cycle and this
  module cannot raise it — it exists so the ceiling is shared fairly. A segmented
  store's actual D$ writes are POST-COMMIT exactly as for any store: it takes the
  same two passes, so its coprocessor round trip changes when its data arrives,
  never when its writes are permitted.

  The load drain's LCB credit is ONE NUMBER READ TWICE. `ld_beat.lcb_alloc_rdy` is
  driven from `lcb.io.alloc.ready`, which the LCB defines as "an entry is free",
  i.e. `free_count =/= 0` — the same quantity `arb` consumes on
  `io.lcb_free_count` for its coarse suppression. Deriving the per-beat gate FROM
  the credit rather than computing it independently is what makes the two
  consistent; any per-PRN refinement (a beat landing in an already-allocated entry
  needs no new credit) must be published by the LCB, never recomputed here.

  ---- 6. Completion: one event per shape, and nothing streamed ----

  //@req-spec-rob.e1
  //@req-spec-rob.e2
  //@req-spec-rob.e3
  A multi-access vector load is NOT memory-safe until ALL of its element addresses
  have disambiguated, so this module reports a SINGLE group-safe event on
  `vec_clr_unsafe` when the LAST element address of that entry has been
  LCAM-checked, and never a per-sub-access clear. The condition is read off the
  entry's own element cursor: the LCAM pass's cursor has reached the entry's active
  element count, and for a US entry the one range-overlap query has been granted.
  No new counter is added — the cursor lives in the LDQ/STQ placeholder, so the
  event is a comparison, not state of ours.

  //@req-spec-rob.e5
  A vector STORE clears `rob_unsafe` on the same shape of event: one group-safe
  when the last of its element addresses has been LCAM-checked, which for a store
  is the completion of the pre-commit translate pass. A shared instruction needs
  only the LSU half's group-safe; the coprocessor half performs no memory access,
  and demanding a second one would deadlock the chain of section 8.

  //@req-spec-rob.c6
  //@req-spec-rob.c7
  //@req-spec-rob.d17
  A NON-SHARED vector store clears its ROB busy bit through a SINGLE
  `lsu_clr_bsy` — it writes no VRF, so it has no group-done to emit — driven on
  the LSU's existing `clr_bsy` port through `lsu_vec` rather than on a new
  completion lane. It is NOT asserted until that store's WHOLE ACTIVE ELEMENT SET
  has translated and disambiguated, i.e. the same instant section 6's store
  group-safe fires; a clear at the first translated element would let the ROB
  retire a store with untranslated elements and forfeit precise exceptions. A
  segmented store's LSU half writes no VRF either and so likewise signals
  `lsu_clr_bsy`, with the deferral below.

  //@req-spec-rob.c9
  A SHARED store's LSU half DEFERS ITS COMPLETION UNTIL AFTER DGEN HAS READ
  `pvtmp` — not to the translate/disambiguate point that governs a non-shared
  store. Concretely, `lsu_clr_bsy` for an `is_shared` store fires on `dgen`'s
  `last` push into `st_SSI_DATA_Q` for that `stq_idx`, which is step 6 of the
  chain. Signalling at translate would claim the half is done while it has still
  to obtain its store data from the coprocessor, and would invert the fixed
  producer-then-consumer completion order the ROB's one-bit "other half pending"
  flag relies on. Assert that a shared store's clear never precedes its DGEN
  stream's last push, and that a non-shared store's never waits for one.

  A load's completion is the LCB's single group-done on lane 0 of `vec_clr_bsy`,
  and a no-execution op's is `gcopy`'s on lane 2. Nothing here streams a per-PRN
  writeback into the ROB and no per-entry completion counter exists anywhere in
  this subtree.

  ---- 6b. `vleff`: the fault interface belongs to the RANGE agen, and this
            module is the only place its three signals meet ----

  THE FAULT-ONLY-FIRST POLICY IS `VecRangeAgen`'S, NOT `VecElemAgen`'S, AND THAT
  RE-ALLOCATION CHANGES THE WIRING HERE. `vle<eew>ff.v` is architecturally
  UNIT-STRIDE, so its `OP.v` self-selects into `VecRangeAgen` (section 2), and
  `spec-lsu.g1/g2/g3/g4/g9/g10` are allocated there. The three signals this module
  must route, all combinational and all on the LOAD instance only (a store has no
  fault-only-first form):

  - `ld_range_agen.io.fault` (INPUT to the agen) — the DRAIN-side fault report,
    raised by `ld_beat` against the RETAINED RANGE ENTRY, carrying
    {`elem_idx`, `is_ff`, `rob_idx`, `ldq_idx`} read out of that entry. Routing it
    off the entry rather than off a latch is what keeps `VecRangeAgen` free of
    per-instruction state: the entry is what remembers the instruction. This
    module QUALIFIES it against the walking `rob_idx` exactly as it qualifies the
    element agens' fault reports — same comparison, same kill terms — so a fault
    for a squashed op cannot classify against a live one.
  - `ld_range_agen.io.fault_trap` (output) — an element-0 fault. Becomes
    `vec_xcpt` with `vstart = 0` and no element index.
  - `ld_range_agen.io.ff_trim` (output) — an element-`i > 0` fault. Goes to
    `lcb.io.trim`, and NOWHERE ELSE.

  // ===> AND THE ELEMENT AGENS ARE NOT ON THIS PATH. `VecElemAgen` asserts
  //      `!v_is_ff` on accept and keeps its own `ff_trim` port for symmetry with
  //      an assertion that it never fires; DO NOT ROUTE IT. Earlier text here
  //      accepted `fault_trap`/`ff_trim` from EITHER agen "so neither path is a
  //      special case" — that predates the re-allocation, and it left the
  //      unit-stride path (the only path a `vleff` can take) with no fault route
  //      at all while wiring one for a form that cannot be encoded.

  THE `vleff` VL WRITE STILL HAS EXACTLY ONE DRIVER, and settling that is this
  module's call because it is the only place both candidate producers are visible.
  The trimmed element count reaches `lcb.io.trim` — this module converts the trim
  ELEMENT INDEX into the LCB's {member, `keep_bytes`} form using `v_eew` — and the
  VL register file's `W_lsu` port is driven ONLY by `lcb.io.vl_wb`, on the
  group-done. Routing an agen's `ff_trim` straight to `W_lsu` as well would put TWO
  PRODUCERS on a statically partitioned, never-arbitrated write port, and it would
  publish a trimmed VL — waking every `pvl` dependent — before the group that VL
  describes had been assembled.

  `VecRangeEntry` MUST CARRY `is_ff`. The drain side raises its fault against the
  entry and has no other way to tell a fault-only-first load from an ordinary one:
  without the field, an element-`i > 0` fault on a `vleff` traps, which is the
  exact architectural violation the form exists to prevent, and it fails on the
  `strlen`-shaped loop that is the feature's whole reason for existing. The field
  is being added to `VecBundles` alongside `stride`, `is_unit_stride`, the active
  byte mask, `us_data_base` and `members`; this module reads it only to route, and
  asserts `is_unit_stride` on every entry with `is_ff` set.

  ---- 7. VecGroupCopy's launch, and the R2/W0 mux ----

  A launch is presented to `gcopy` when, and only when, the granted `OP.v` is on
  the LOAD side, has a vector destination, and is NOT `v_is_whole_reg`, with
  `vl_zero` taken from `ld_opnd.out` and `all_inactive` from `ld_msk`. Neither
  value is re-derived here. The whole-register exclusion is a correctness term,
  not a filter: a `vl1re*` with `vl = 0` still transfers its full length from
  memory, so it is not a no-execution op at all, and a copy launched for it would
  race a real load for the same PRNs. A store never launches — it has no VRF
  destination — and for a shared load the group completed is `pvtmp`, which needs
  no data copy.

  THE STRICT-PRIORITY MUX OVER `R2` AND `W0` LIVES INSIDE `gcopy`. Both
  `VecGroupCopy` and `VecRegFile` declared it, and it is settled by REQUIREMENT
  ALLOCATION: `spec-lsu.m13` (the Load Unit's VRF ports are arbitrated by a
  strict-priority mux) and `spec-lsu.m14` (an active load drain always wins) are
  allocated to `VecGroupCopy`, while `VecRegFile` holds only `m6`/`m7` (the copy
  reuses the Load Unit's ports and adds none). The owner of the obligation owns the
  logic. So this module wires `lcb.io.stale_req`/`stale_resp`/`vrf_write(0)`
  THROUGH `gcopy` combinationally, and `gcopy.vrf_r2_req`/`vrf_r2_data`/`vrf_w0`
  are what reach `VecRegFile`, which therefore sees exactly ONE `R2` reader and ONE
  `W0` writer and must carry no `gcopy` port and no grant logic. The LCB is slot 0,
  drives unconditionally and is NEVER TOLD IT LOST — there is no signal by which it
  could be — while `gcopy` qualifies each of its two requests independently on the
  LCB's valid being low. Assert that a cycle in which the LCB requests `R2` or `W0`
  never carries a `gcopy` request on that same port. At `lsuWidth = 2` the LCB's
  second write reaches `W1` directly; `gcopy` uses `W0` only, since its rate is
  bounded by the single `R2` read.

  ---- 8. Segmented (shared) load/store: two halves, one pvtmp ----

  //@req-spec-lsu.l7
  //@req-spec-lsu.l8
  //@req-spec-lsu.l9
  A segmented load or store is marked `is_shared` and SPLIT INTO TWO HALVES: this
  module executes the half that MOVES THE DATA TO OR FROM MEMORY, and the
  coprocessor executes the half that transposes it. The two HAND OFF THROUGH
  `pvtmp` — the rendezvous group named by the `pvtmp` field of the one shared uop,
  with no table and no temp register file anywhere — and the handoff is a real VRF
  group in both directions: the producer writes it, its group-done wakes the
  consumer's issue slot. Nothing here consults `pvs3` for a shared op and nothing
  here allocates or frees `pvtmp`.

  //@req-spec-lsu.l1
  For a segmented LOAD the `pvtmp` group is the DESTINATION of the LSU half: the
  element agen stamps `pvtmp` members as the placement group on every emitted
  `nOP.v`, the LCB assembles them as an ordinary vector destination, and its
  group-done is what wakes the coprocessor half. `pvdest` correctness for that
  instruction belongs to the coprocessor half under its own `vta`/`vma` policy.
  For a segmented STORE the direction reverses and `pvtmp` is the SOURCE `dgen`
  reads, which is why `dgen_operand` is `Mux(is_shared, pvtmp, pvs3)`.

  //@req-spec-issue.d2
  //@req-spec-issue.d4
  The LSU half of a segmented store runs AGEN AS THE FIRST STEP of the six-step
  chain, and its group-safe is the SECOND: `st_opnd` + `st_elem_agen` source every
  operand of the address path — the base GPR, the index vector and the mask — from
  instructions OLDER than the store, so step 1 is unblockable; the group-safe of
  section 6 then clears `rob_unsafe`, the PNR advances past the entry, the
  coprocessor half becomes eligible and writes `pvtmp`, and only then does step 6
  — `dgen` reading `pvtmp` on a separate, later `FC_DGEN` grant — run. AGEN and
  DGEN of the same slot are independently granted and may be hundreds of cycles
  apart; between them this module holds nothing for that store beyond its retained
  queue region. That is why the chain is a long serial dependency and not a cycle.

  ---- 9. Kill, and the rollback index convention ----

  `squash` is THE RESOLVER of every kill in this subtree. This module routes
  `brupdate`, `rob_flush`, `rob_flush_kill` and the four LSQ pointers into it,
  pairs `kill_uop`/`kill` with the five clients in the fixed order of the
  parameters section, fans `q_squash` out to the six queues in enumeration order,
  drives `resv.rollback` from `squash.resv_rollback` and returns
  `resv.rollback_tail` to `squash.resv_rollback_tail` in the same cycle, and
  routes `squash.kill_ldq` to `lcb.io.kill_ldq` and `squash.resv_rollback.ldq_idx`
  to `gcopy.squash`. Nothing here computes a kill locally, so `spec-lsu.i6` keeps
  its owner.

  THE ROLLBACK INDEX IS BOOM'S EXCLUSIVE TAIL. This is the highest-risk seam in
  the subtree and it is settled here: the `ldq_idx`/`stq_idx` on `resv.rollback`
  names the FIRST DEAD entry, so entries `[head, idx)` survive and
  `[idx, old_tail)` die, exactly as `brupdate.b2.uop.ldq_idx` is the value
  `ldq_tail` had when the branch dispatched. Each queue's new tail is therefore
  `base + count` of the reservation row owned by the YOUNGEST LSQ entry STRICTLY
  OLDER than the driven index, or that queue's head if no such row exists — NOT
  the row of the driven index itself. Reading it as an inclusive survivor keeps one
  killed instruction's entries alive, and the entries it keeps are the ones whose
  destination PRNs have just been returned to the free list. Assert, on every
  rollback, that no surviving row's `base + count` exceeds the new tail and that
  the driven index's own row is invalidated.

  ---- 10. Memory ordering: three mechanisms, one match ----

  `snoop` presents vector addresses to the LCAM and searches loads against the
  vector store address queues; `fwd` decides and performs LD->ST forwarding out of
  the store DATA queues and publishes `known_overlap`; `hold` consumes that same
  classification and decides which younger load waits. This module routes
  `fwd.io.known_overlap` into `hold.known_overlap` and does NOT let `hold` build a
  second overlap test — the forwarding and hold predicates must stay exact
  complements, and one match is what keeps them so. `hold.hold_ldq`
  (`UInt(numLdqEntries.W)`, registered, bit-indexed by the real LDQ index) goes to
  `arb.io.hold_ldq`, which owns the suppression; the hold decides who, the arbiter
  decides what a hold does.

  THE SHARED QUEUE READ PORT IS MUXED HERE, WITH SNOOP WINNING. Both `snoop` and
  `fwd` consume the one read port each store queue reserves beyond its drain lanes
  (`readPorts = ports + 1`). Strict priority to `snoop`: its read is tied to an
  LCAM/TLB grant the arbiter has ALREADY made this cycle, so deferring it would
  strand a grant the arbiter cannot retract, whereas `fwd` has a declared replay
  path — the loser asserts `io.replay` and the load retries, costing one replay and
  no data. `snoop`'s per-candidate `data_filled` gate is satisfied by the same
  muxed read of the paired data queue, so a store candidate that loses the mux is
  simply not presented that cycle and the "no address without its data" gate stays
  sound. Assert that a granted LCAM lane is never left unpresented because of this
  mux.

  `st_drained` IS A LEVEL, ONE BIT PER STQ ENTRY, AND MUST NOT BE A PULSE. This
  module drives it: `st_drained(i)` is ASSERTED whenever entry `i` is not a live
  vector store with active elements still to write — invalid, squashed, scalar,
  never-allocated and fully-drained entries all read asserted — and DEASSERTS only
  while `i` holds a live vector store whose element cursor has not completed its
  active element set ON THE POST-COMMIT WRITE PASS. Two failure modes are being
  avoided at once. A pulse would never arrive for a predicted event naming a store
  that had already drained, hanging that load forever. And keying it on the
  TRANSLATE cursor instead of the WRITE cursor would release the hold before any
  byte reached the cache, so the released load would read the stale line and the
  hold would have added latency and nothing else. Derive it from the write-pass
  cursor and the committed flag only.

  ---- 11. `vec_lsu_empty`, and why it is not a busy ----

  `vec_lsu_empty` is the AND of the four ADDRESS queues' `io.empty`, both DATA
  queues' `io.empty` and `lcb.io.empty`. It leaves on `lsu_fencei_rdy_vec` and
  folds into the LSU delta's
  `io.core.fencei_rdy := !stq_nonempty && io.dmem.ordered && vec_lsu_empty`,
  replacing the old head-side handshake that deadlocks on younger vector stores.
  It is NOT a `busy`: it is not scoped to an instruction, it names no `rob_idx`,
  and its consumer is a `fence.i` waiting at DISPATCH with the ROB already
  draining.

  `gcopy`'s PENDING WORK LIST IS DELIBERATELY EXCLUDED, AND THE EXCLUSION IS
  CONFIRMED RATHER THAN AN OVERSIGHT — it is stated here precisely so it does not
  read as one. A pending `VecGroupCopy` is a VRF WRITE, not memory state: the copy
  performs no D$ access, occupies no element queue, holds no LCB entry and issues
  no TLB or LCAM request, so there is nothing about it for a memory fence to
  order. `fence`/`fence.i` order memory operations against each other, and the
  architectural visibility of a vector register write is ordered by the ROB and
  the group-done/wakeup networks, which the copy already participates in — its
  `vec_clr_bsy` lane 2 is what makes its instruction retire. Folding it into
  `vec_lsu_empty` would therefore not add an ordering guarantee; it would only
  delay `fencei_rdy` behind a VRF write that no consumer of the fence can observe,
  and it would put a non-memory event in a term whose whole definition is "the
  vector memory datapath is empty". Assert the negative form instead: a cycle in
  which `vec_lsu_empty` is high may still have `gcopy` work outstanding, and that
  is legal.

  ---- 12. Trace and assertions ----

  There are no unit tests in this project; validation is end-to-end VCS plus
  Whisper cosim. Emit guarded `VecTrace` lines, gated on the `vecTrace` plusarg
  and `!reset`, off by default, each tagged with this module's name and `rob_idx`:
  one per accepted `OP.v` naming WHICH agen took it and why (the class flags), one
  per store pass transition (`stq_idx`, pass number, cursor reset), one per
  completion event emitted (group-safe, `lsu_clr_bsy`, which lane of
  `vec_clr_bsy`), one per shared-port mux loss, one per `st_drained`
  transition, and — for section 3b — one per row write (direction, `ldq_idx`/
  `stq_idx`, bypassed or not), one per cycle a presentation is WITHHELD naming
  WHICH qualification was missing (streamer busy, or which read lane was denied),
  and one per fire. That "why was it withheld" line is the whole diagnosis of an
  operand-read stall, and it distinguishes an INT-port shortfall from a streamer
  serialization without a waveform. The routing lines are the ones that matter here: a vector memory op
  that goes to the wrong agen, or a completion that is never emitted, is otherwise
  invisible in a log, and this module's own bugs are all wiring bugs.

  Whole-subtree assertions this module owns because only it can see both sides:
  the two agen selections are exclusive and exhaustive; `resv`'s occupancy agrees
  with every queue's `avail`; a store's address and data regions have equal counts
  for the SSI class and the documented 1-to-`v_emul` ratio for the US class (a
  STORE-side check only — a load's region may legitimately be smaller than its
  element count, per section 4); section 3b's table is never written to an already
  valid row, never presents while its direction's streamer is busy or its reads are
  ungranted, and `VecElemAgen`'s own pending slot is always free on an accept; a
  drain-side fault report reaches `ld_range_agen` only while its `rob_idx` matches
  the live unit-stride load, `VecElemAgen.ff_trim` NEVER fires, and every range
  entry with `is_ff` set also has `is_unit_stride`; `st_range_agen.io.st_data`
  agrees with `dgen`'s member sequence on every beat; no
  output of this module reaches an issue unit (a review obligation, checked by
  grep gate H4, stated here so the generated file carries the reminder).
  <|end_logic|>

<|end_module|>

<|begin_perf|>
This module adds NO pipeline stage of its own. Every path through it is either a
direct connection or a mux described above, and the two muxes it owns — `R4` and
the shared queue read port — are single-level static or strict-priority
selections. A registered stage inserted here would land in the middle of one of
the children's own one-cycle contracts (the operand read's grant-to-result, the
beat expander's cursor loop, the forward's search-to-response) and break it.

Section 3b's table is the one structure that HOLDS state, and it is deliberately
not a stage: its grant-cycle bypass means an `OP.v` whose streamer is free and
whose INT/VL reads are granted reaches `xx_opnd` in the grant cycle exactly as it
did before the table existed, so the table costs latency only in the cycles it is
absorbing a hazard that would otherwise have been a lost descriptor or a wrong
base. Two paths to watch, and both are one level: the oldest-valid-row select
(`numLdqEntries` / `numStqEntries` wide, using BOOM's existing age comparators,
and the same shape as `resv`'s own select), and the AND of the streamer-free and
read-granted terms, whose inputs must be flop reads plus the RF's `ready`. Area:
`numLdqEntries + numStqEntries` rows of the ENUMERATED field set, not of a blanket
`MicroOp` — small beside the three 512-entry queues, and the enumeration is what
keeps it so.

Concurrency is the target, and it is structural rather than tuned. A `vle` and a
`vse` must overlap with neither's issue gated by the other (target P4): the two
directions share no operand read, no agen, no mask streamer, no beat expander and
no queue, and they meet only at `arb`, where they are two independent requestors.
Any cycle in which a load grant suppresses a store grant outside the arbiter is a
regression against the whole point of the decomposition.

Throughput obligations passed down: one `OP.v` accepted per direction per cycle;
one element access emitted per cycle per fill-side agen; `lsuWidth` beats granted
per cycle on the drain side; sustained at least one D$ access per cycle per
granted lane while a queue is non-empty (P2); and the unit-stride access count at
the floor `ceil(active_bytes / coreDataBytes)` independent of SEW (P1). This
module's contribution to all five is to add no serialization between them.

Critical paths it must not lengthen: `brupdate.b2` into the six rollback commands
(one comparator level and one mask; the rollback tails arrive precomputed from
`resv`), and the arbiter's grant expression, whose suppression terms must be flop
reads. Area is dominated by the children — three 512-entry SSI queues, the LCB's
assembly entries, the snoop's summary — and this module adds only wiring plus the
assertion logic of section 12.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the dispatch, issue and commit uops, and the `nOP.v`-scoped cursor
fields nothing outside this subtree may read. This module writes NO uop field and
adds none.
VecBundles — `VecElemAccess`, `VecRangeEntry`, `VecReservation`, `VecGroupDone`,
`VecException` and the normative six-queue enumeration, which is the only legal
source of a queue name here. `VecRangeEntry` must carry `is_ff` (section 6b) along
with `stride`, `is_unit_stride`, the active byte mask, `us_data_base` and
`members`; the drain side cannot classify a fault-only-first load without it.
VectorParams — every depth and width passed down, including `ldResvMembers` (the
load reservation quantum, default 4) which reaches `resv`, not this module's own
logic. `VecLsuCoreIO` is NOT declared here: it is declared in `lsu.scala` beside
`LSUCoreIO` and bound by name (see the ports section), and this module declares no
bundle of its own at all.
VecTrace — the guarded trace helpers.

Binds to BOOM's existing `BrUpdateInfo`, `IsKilledByBranch`, `GetNewBrMask`,
`EntryValidFromAge`, `GetRealLSQIdx` and `SelectFirstN` rather than reimplementing
any of them, and introduces no new speculation, wakeup, replay or recovery
mechanism. The only two new networks in the design are VL (`pvl`) and VECTOR
(group-done), and neither originates here.

Instantiates seventeen children, listed in the header. Its counterparties outside
the subtree, and what each owes this seam:
  VecPipeline — instantiates this module as `vlsu`; supplies the dispatch, issue,
    recovery, commit and register-file halves of `vec_pipeline_io`.
  VecRegFile — the seven ports cited by number, and two settled points.
    (a) LATENCY: every read port is ONE REGISTERED CYCLE at the `VecRegFile`
    boundary — request in cycle t, `read_data` in t+1, with the output flop
    instantiated in `VecRegFile`, one per read port. `VecRegFileBank`'s
    "0 cycles, may not be pipelined" is BANK-INTERNAL and sits inside that
    envelope; it must not be propagated outward. This module and every consumer
    below it — `VecIdxGen`, `VecMaskStream`, `VecDgen` on `R3`, the LCB on `R2` —
    are written against exactly that, so the element and beat pipelines are
    correct as specified.
    (b) ARBITRATION: `VecRegFile` sees exactly one requester per port and must add
    none, because the `R2`/`W0` mux is `gcopy`'s and the `R4` mux is this module's.
    Its `gcopy_r2`/`gcopy_w0` ports and their grant logic are a required DELETION.
  VlRegFile — one `W_lsu` write, driven only by `lcb.io.vl_wb`, and two reads, one
    per operand-read instance.
  LSU (edit_existing, src/main/scala/v4/lsu/lsu.scala) — the `VecLsuCoreIO`
    DECLARER and this module's peer on it: the LDQ/STQ placeholders and their
    element cursors, the LCAM/TLB/D$ resource claim and fire,
    `ldst_addr_matches`, the four LSQ pointers, the committed flag, `clr_bsy`, and
    folding `vec_lsu_empty` into `fencei_rdy`. This module binds to that
    declaration and adds none of its own.
  RegisterFile / `PartiallyPortedRF` (host) — the four `Decoupled` INT read lanes
    whose `ready` is index-priority and routinely low, consumed by section 3b's
    table and by nothing else.
  VecSquashUnit (`squash`) — five kill clients in the fixed order of the
    parameters section; its own `nKillClients` default of 8 and its client list
    are stale after the mask-streamer hoist, and this file is authoritative.
  Rob — consumes `vec_clr_bsy` lanes 0 and 2, `lsu_clr_bsy`, `vec_clr_unsafe` and
    `vec_xcpt`, each as an ordinary single-shot event with no vector special case.
  VecIssueUnit (`iq_v_load`, `iq_v_store`) — grants `iss_ld`/`iss_st`. It receives
    NOTHING back from this module: no ready, no credit, no busy.
<|end_dependencies|>

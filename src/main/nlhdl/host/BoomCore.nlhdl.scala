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
  BoomCore — DELTA SPEC. This file is NOT a description of BOOM's core. It
  describes only the change applied to `class BoomCore` in
  src/main/scala/v4/exu/core.scala, which is hand-written baseline BOOM v4 and
  stays in place. Everything the file already does — the frontend redirect
  logic, the branch-resolution reduction, `dec_hazards`/`dis_hazards`, the FTQ
  arbiter, `ll_arb`, the four scalar issue units, the register-read arbitration,
  the CSR/PTW/RoCC/trace plumbing — is unchanged and is NOT restated below.
  Anything this file does not mention keeps its current form exactly.

  hierarchy.yaml: kind: module, mode: edit_existing,
  target src/main/scala/v4/exu/core.scala, group host,
  depends_on MicroOp, ScalarOpConstants, BoomCoreParams, VecBundles.
  Adds exactly ONE instance: `vec` (VecPipeline), connected by the
  `vec_pipeline_io` interface.

  ===> BUDGET: 200 ADDED LINES, AND IT IS THE POINT OF THE FILE, NOT A
       COURTESY. Plan v2 §5 rule 3 and §11's file-touch table both state
       "core.scala <= 200 added lines (was 1101)". `addvector` braided 1101
       lines of vector wiring line-by-line through scalar wiring, and the M1
       free-list double-free hid inside that braiding. The shape of this delta is
       therefore: ONE gated instantiation, ONE bundle connect, and the taps the
       bundle names. If a reviewer finds vector arbitration, vector selection, a
       vector register, a vector queue or a per-field vector fixup here, the
       container boundary has leaked and the logic belongs inside `VecPipeline`.

  ===> THE THREE CONNECTION-SITE RULES THAT CARRY THE OLD BUGS. (1) The rename
       lockstep inputs take `dis_uops`/`dis_valids`/`dis_fire` — the REGISTERED
       ren1-to-ren2 bundle — and never `dec_uops`/`dec_valids`/`dec_fire`; the
       port names on the seam exist so the wrong wiring is visibly wrong here.
       (2) The decode seam runs THROUGH `decode_units(w)`, unconditionally on
       every lane, because `MicroOp.vconfig` must be populated on every uop that
       can allocate a `br_tag` — usually a SCALAR branch. (3) `vset_resp` taps
       `io_alu_resp` WITHOUT the `dst_rtype === RT_FIX` qualifier used at
       core.scala:965 and :975, or `vsetvli x0, rs1` loses its VL write.

  Governing spec anchors: overview.rst `caracal-pipeline`; issue.rst
  `dispatch-stage`, `cii-shared-sched`; frontend.rst `vector-csr-ownership`,
  `vcfg-recovery`; midcore.rst `rename-stage`, `spec-wakeups`; loadstore.rst
  `fences`.
*/

<|begin_module|>

  <|begin_parameters|>
  No new constructor parameter and no new `BoomCoreParams` field: `usingRVV`,
  `vLen`, `maxMembers` and every vector size come from `BoomCoreParams` /
  `VectorParams` through `HasBoomCoreParameters`, already in scope. The delta
  changes exactly four existing elaboration values, each by an amount that is
  ZERO when `usingRVV` is false, and adds three `require`s.

  `numIrfWritePorts` becomes `aluWidth + lsuWidth + 1 + numVecIrfWritePorts`,
  where `numVecIrfWritePorts` is 1 when `usingRVV` and 0 otherwise. The added
  port is the vector subsystem's scalar-destination writeback (`vmv.x.s`,
  `vcpop.m`, `vfirst.m`) and it is APPENDED, so every existing write-port index
  — the `lsuWidth` LSU ports, the `ll_arb` port, the `aluWidth` ALU ports — keeps
  the number it has today. `iregfileBankedWriteArray` gains `None` at the END
  for the same reason: inserting anywhere else renumbers the column-ALU write
  entries and changes `BankedRF`'s per-bank write mapping.

  `numIntWakeups` becomes `coreWidth + lsuWidth + 1 + numVecIrfWritePorts`, the
  wakeup slot that accompanies that write port. It is appended after the loop
  that fills the existing slots, so `require(wu_idx == numIntWakeups)` and
  `require(wb_idx == numIrfWritePorts)` (core.scala:984-985) still hold with the
  scalar indices untouched.

  ===> THE `Rob` CONSTRUCTOR ARGUMENT MUST NOT FOLLOW THE WIDENED
       `numIrfWritePorts`. `rob` is built with
       `numIrfWritePorts + numFpWakeupPorts`; if the first term is allowed to
       follow the widening above, `Rob` gains a `wb_resps` entry that
       its own delta explicitly rejects, and nothing would drive it. The vector
       scalar-destination writeback reaches the ROB through `vec_clr_bsy` lane 1
       and `vec_rob_flags`, never through `wb_resps`. So the Rob instantiation
       passes the SCALAR INT count (`aluWidth + lsuWidth + 1`) plus
       `numFpWakeupPorts`.
       The SECOND term, by contrast, is left to track: `numFpWakeupPorts` is
       `fp_pipeline.io.wakeups.length` (core.scala:116), and decision D7 adds ONE
       `usingRVV`-gated FP wakeup slot inside `FpPipeline` to land `fp_wb`, so it
       rises by one in a vector config and the existing FP loop over
       `fp_pipeline.io.wb` drives the entry it creates (part 8). `Rob`'s
       `numWakeupPorts` is therefore bit-identical whenever `usingRVV` is false —
       which is the gate (f) claim — and one larger, fully driven, when it is true.

  `numIrfLogicalReadPorts` becomes `all_exe_units.map(_.nReaders).reduce(_+_) +
  numVecIrfReadPorts`, with `numVecIrfReadPorts` = 5 when `usingRVV` else 0. The
  value is raised in the one `val`, so both `BankedRF` arguments that take it
  (`numLogicalReadPortsPerBank` and `numLogicalReadPorts`) track it together —
  `BankedRF` only connects its banked read path when those two are EQUAL, so
  raising one alone silently leaves the read ports undriven.

  Three added `require`s. The third is part 4's
  `require(ip.dispatchWidth == coreWidth)` on the three `IQ_V_*` `issueParams`
  entries, which is what keeps `CompactingDispatcher`'s per-queue `Compactor` an
  identity and the dispatch payload off the seam. The other two:
  `require(!usingRVV || usingVector)`, because
  `csr.io.vector` is `usingVector.option(...)` in rocket's `CSRFileIO` and a
  `.get` on `None` is an inscrutable elaboration crash rather than a message; and
  `require(!usingRVV || boomParams.vector.isDefined)`.
  <|end_parameters|>

  <|begin_ports|>
  CLOCK AND RESET: unchanged. `BoomCore` is a `BoomModule` with Chisel's implicit
  POSEDGE `clock` and ACTIVE-HIGH SYNCHRONOUS `reset`, one domain; `vec` inherits
  both implicitly. No clock, reset, gate or CDC is added anywhere.

  NO PORT IS ADDED TO `class BoomCore`'s `io`, and none is widened. The vector
  subsystem is entirely internal to this module plus the two seams that already
  exist as bundles owned by other nodes:

  - `io.lsu` gains `lsu_vec` and `vec_lsu_empty` because `class LSUCoreIO` gains
    them in the `LSU` delta. This file only connects them; it does not declare
    them and must not restate their fields.
  - `csr.io.vector` is rocket's, present whenever `usingVector`. Nothing is added
    to `CSRFileIO`.

  ---- The one added instance, and its one connection ----

  `val vec = if (usingRVV) Some(Module(new VecPipeline)) else None`, an
  `Option`-wrapped instance so a vectors-off build has the whole subtree ABSENT
  rather than tied off. `vec.get.io` is a `VecPipelineIO`, the realization of
  hierarchy.yaml's `vec_pipeline_io`, and it is the ONLY vector interface in this
  file. The interface entry is authoritative over anything below: it has been
  amended eleven times and currently carries 42 members. Every connection in the
  logic section names a member of it; there is no second vector wire, no vector
  `Wire`, no vector `Reg` and no vector arbiter in `core.scala`.

  ---- Members of the seam this delta requires and the entry lacks ----

  Named here rather than resolved silently. Each is a one-line addition to
  `vec_pipeline_io` and `VecPipelineIO`, and each is REQUIRED for this file to
  elaborate a correct machine:

  - `dis_uops_out` (bwd, `MicroOp * coreWidth`) — the chained rename output.
    `Rob` stores a full `MicroOp` per entry (`rob_uop`, rob.scala:364) and
    `VecRenameSpace` frees `stale_pvdest` and `pvtmp` at COMMIT off
    `commit_uops`, so the renamed vector fields must be in `rob.io.enq_uops`. The
    seam as written has no path back, which would leave every committing uop's
    vector PRNs undefined and leak the vector free list on the first vector
    instruction. It must be a WHOLE uop (VecPipeline's chain passes non-vector
    fields through untouched), because the alternative — patching fifteen vector
    fields into `dis_uops` one line at a time, the way core.scala:703-723 patches
    the FP/pred/imm spaces — is the 1101-line braiding this design exists to
    delete, and driving `dis_uops` from the seam output instead closes a
    per-field combinational loop through `vec`.
  - `int_rf_read_req` must gain a per-lane `ready`, i.e. become
    `Vec(5, Decoupled(UInt(maxPregSz.W)))`. See part 7: the INT register file is
    deliberately PARTIALLY ported and denies reads by design, so a read seam with
    no back-pressure is unimplementable.
  - `vset_resp` must become `Vec(aluWidth, Valid(new ExeUnitResp(xLen)))`. See
    part 9.
  - `dis_vec_valids` (fwd, `coreWidth * 3`) and `dis_vec_ready` (bwd,
    `coreWidth * 3`) — the per-lane dispatch handshake the three `IQ_V_*` queues
    need now that decision D2 makes them REAL dispatch clients rather than
    tied-off lanes. Indexed load/store/ALU, matching `WithVector`'s
    `issueParams` append order. See part 4; the payload deliberately does not
    cross, and the `require` that makes that sound is stated there.

  ---- Seam members this delta does NOT consume, deliberately ----

  `vl_wakeup` is an output of `VecPipeline` and every VL consumer is inside
  `VecPipeline` (its part 6 forms the network from `vset_resp` and its own
  `vleff` trim). Nothing in `core.scala` reads or drives it. `debug_vrf_read` is
  `dontTouch`ed for the waveform and the Whisper cosim and reaches no functional
  logic.
  <|end_ports|>

  <|begin_logic|>
  Twelve connection groups. No arithmetic, no state, no arbitration except the
  one two-input age priority of part 6, which exists only because two nodes'
  reject lists forbid it anywhere else.

  ---- PART 1. The gate ----

  There are exactly TWO `if (usingRVV)` decisions in this file: the
  `Option`-wrapped instantiation, and the dispatcher CLASS selection of part 4
  (D2) — no third one is permitted. Every connection below sits inside a
  single `vec.foreach { v => ... }` block or in an existing loop guarded by the
  same `Option`. `usingRVV` is a Scala `Boolean` of `BoomCoreParams`, never a
  hardware `Bool` and never rocket's `usingVector`, which is a DIFFERENT gate
  that controls the architectural CSR cells this design deliberately delegates.

  ---- PART 2. The decode seam runs THROUGH the decode units ----

  Inside the existing decode loop (core.scala:524-535), per lane, four lines:
  `v.io.dec_uops_in(w) := decode_units(w).io.vec.get.uop_to_vdec`;
  `decode_units(w).io.vec.get.uop_from_vdec := v.io.dec_uops_out(w)`;
  `decode_units(w).io.vec.get.illegal := v.io.dec_vec_illegal(w)`; and
  `v.io.dec_insns(w) := dec_fbundle.uops(w).bits.inst`. `dec_valids(w)` and
  `dec_fire(w)` are forwarded from the wires of the same name.

  `dec_uops(w) := decode_units(w).io.deq.uop` stays VERBATIM. The merge of the
  vector fields into the uop, and the OR of vector illegality into
  `id_illegal_insn`, are `DecodeUnit`'s — it owns the three `vec` ports and does
  the merge itself. NOTHING is merged, muxed or overridden in `core.scala`, and
  the connection is unconditional over lanes: no `is_vec` predicate, no
  `Mux(v_legal, ...)`. That is what puts `vconfig` on every lane's uop.

  // ===> TWO COMBINATIONAL LOOPS ARE ONE CARELESS LINE AWAY HERE, AND NEITHER IS
  // OBVIOUS FROM EITHER SIDE ALONE.
  // (a) `dec_fire` MUST reach only registers inside `VecPipeline` (the vtype
  //     mirror and the `vl_imm` shadow). BOOM's `dec_stalls` is a prefix scan
  //     over `dec_hazards`, which reads `dec_xcpts` <- `dec_uops(w).exception`
  //     and `branch_mask_full` <- `dec_uops(w).allocate_brtag`. So a
  //     combinational path from `dec_fire` to `dec_uops_out` or
  //     `dec_vec_illegal` closes dec_fire -> vec -> dec_uops -> dec_stalls ->
  //     dec_fire.
  // (b) No vector decoder may READ `uop.exception` / `uop.exc_cause` off
  //     `uop_to_vdec` (A27): `uop_to_vdec.exception` already depends on
  //     `io.vec.illegal` through `id_illegal_insn`.

  //@req-spec-decode.g2
  Vector architectural CSR state is reached only through `csr.io.vector`, and it
  is rocket's `CSRFile` that holds it — `vtype` including `vill`, `vl`, `vstart`,
  `vxrm`, `vxsat`, `vcsr`, `vlenb` and `mstatus.VS`. This file adds no vector CSR
  register, no `vtype` mirror and no `vl` register; Caracal owns only the
  speculative VCFG mirror and the VL register file, both inside `VecPipeline`.
  The decode-side consumer of that ownership is already present and unchanged:
  `decode_units(w).io.csr_decode <> csr.io.decode(w)` (core.scala:529), whose
  `vector_illegal` bit is rocket's `mstatus.VS == 0` gate and is newly READ by
  `DecodeUnit`. See part 10 for the write-side fields.

  ---- PART 3. The rename / dispatch lockstep ----

  //@req-spec-rename.b9
  //@req-spec-rename.i1
  //@req-spec-decode.h7
  Three connections, and the choice of source wire is the whole content of this
  part: `v.io.ren2_uops := dis_uops`, `v.io.ren2_mask := dis_valids`,
  `v.io.dis_fire := dis_fire`. `dis_uops` IS `rename_stage.io.ren2_uops` (the
  registered ren1-to-ren2 `r_uop`, rename-stage.scala:104-116) after the
  cross-space patches and after `rob_idx`, `ldq_idx`, `stq_idx` and `rxq_idx` are
  written into it, so it is the only bundle that is both REGISTERED and complete.
  `VecPipeline` builds `ren_br_tags` from these two signals as
  `{valid = dis_fire(w) && ren2_uops(w).allocate_brtag, bits = ren2_uops(w).br_tag}`
  — which is `ren2_br_tags(w+1)` at rename-stage.scala:117-118 verbatim, because
  `ren2_fire` there IS `io.dis_fire`. So `br_tag` allocation, the scalar and
  vector RMT snapshots and the speculative VCFG mirror snapshot all happen on the
  SAME event in the SAME cycle, by construction rather than by agreement.

  // ===> WIRING `dec_uops`/`dec_fire` TO THESE THREE PORTS IS THE M1 FREE-LIST
  // DOUBLE-FREE. Vector rename then runs one cycle AHEAD of the scalar
  // RenameStage's ren1-to-ren2 register, so at dispatch the vector fields
  // describe the NEXT cycle's (bubble) uop and two ops free the same PRN. The
  // ports are named `ren2_*`/`dis_*` precisely so that this line is visibly
  // wrong where it is written.

  `rob.io.enq_uops := v.io.dis_uops_out` replaces `rob.io.enq_uops := dis_uops`
  (core.scala:790) when `usingRVV`. `rob.io.enq_valids := dis_fire` is unchanged.
  This is the ONLY place the chained rename output is consumed outside
  `VecPipeline`, and it must not be written back into `dis_uops`: `dis_uops`
  feeds `v.io.ren2_uops`, so a return assignment is a per-field combinational
  loop. `io.lsu.dis_uops` keeps taking `dis_uops` — the scalar LSU needs
  `uses_ldq`/`uses_stq`/`is_vec`, all decode-stage fields, and no vector PRN.

  The vector ALLOCATION answer enters through ONE existing term. In the per-lane
  loop at core.scala:725, `ren_stalls(w)` gains `|| vec_stall`, where `vec_stall`
  is `!v.io.dis_ready` (a single bit, broadcast to every lane, `false.B` when
  vectors are off). One bit for the whole bundle, because the vector free list
  grants a whole dispatch group or none — a per-lane vector stall cannot express
  that, and BOOM's own `dis_stalls` prefix scan then lets the surviving prefix
  dispatch, which is exactly why `VecPipeline` consumes per lane (`alloc_fire` is
  per lane even though `alloc_ok` is not, A1).

  ===> AND `dis_ready` IS NOW ALLOCATION ONLY (decision D2). It is the AND of
       exactly THREE whole-bundle allocation conditions — `vec_rename.alloc_ok`,
       `vl_rename.alloc_ok`, `vlsu.dis_ok` — and ISSUE-QUEUE CAPACITY IS NOT ONE OF
       THEM any more. Vector queue fullness is per-lane and reaches the pipeline
       through the EXISTING `dis_hazards` term `!dispatcher.io.ren_uops(w).ready`,
       natively, because part 4's dispatcher masks each queue's ready by whether
       that lane's `iq_type` names the queue. Folding capacity back into
       `vec_stall` is exactly what D2 rejected: a broadcast bit makes a full
       `IQ_V_LOAD` stall pure-scalar lanes. Do not merge the two answers in either
       direction — allocation is genuinely whole-bundle and genuinely cannot be
       per-lane, capacity is the reverse.

  // ===> DO NOT CONNECT ANYTHING TO core.scala's OWN `dis_ready` OR `dec_ready`
  // WIRES. They are `!dis_stalls.last` and `dec_fire.last` and keep those
  // definitions exactly; the seam member of the same name is the VECTOR
  // subsystem's answer and reaches the pipeline only through `ren_stalls(w)`.
  // The name collision is real and a generator will fall into it.
  // The reverse obligation is `VecPipeline`'s: `dis_ready` must be computed
  // FIRE-INDEPENDENTLY from `ren2_uops`, or dis_ready -> ren_stalls ->
  // dis_hazards -> dis_fire -> dis_ready closes a loop.

  ---- PART 4. Dispatch: the dispatcher is CONFIG-SELECTED and the three vector
       queues are wired NATIVELY ----

  //@req-spec-core.e9
  //@req-spec-issue.a5
  `dispatcher` becomes `Module(if (usingRVV) new CompactingDispatcher else new
  BasicDispatcher)` — decision D2 — and that one-line CHOICE is the entire edit.
  `dispatch.scala` is NOT touched: both dispatchers already exist in it
  (`BasicDispatcher` at dispatch.scala:43, `CompactingDispatcher` at :70) and BOOM
  v4's own `DispatchIO` already carries every port this needs. A NON-VECTOR BUILD
  STILL INSTANTIATES `BasicDispatcher` AND MUST STAY BIT-IDENTICAL, which is why
  the selection is config-driven and not unconditional: the dispatcher is NOT one
  of D1's enumerated encoding-width exceptions, so gate (f) may not be made to
  depend on it. How the dispatcher is fed is unchanged, verbatim
  (core.scala:829-832): `ren_uops(w).valid := dis_fire(w)` and
  `ren_uops(w).bits := dis_uops(w)`.

  //@req-spec-core.e10
  //@req-spec-core.e11
  //@req-spec-issue.a3
  //@req-spec-issue.a6
  //@req-spec-issue.a7
  Routing is by the `iq_type` bitmask and nothing else, on both sides of the
  seam, and BOTH dispatchers express it with the SAME predicate:
  `BasicDispatcher` drives `dis(w).valid := ren_uops(w).valid &&
  ren_uops(w).bits.iq_type(issueParam.iqType)`, and `CompactingDispatcher` gates
  each lane's request with the identical bit test as `uses_iq`. Neither needs any
  change to admit `IQ_V_LOAD`/`IQ_V_STORE`/`IQ_V_ALU`; `VecPipeline`'s part 5
  applies that same predicate to the same uop for its three queues. So there is no
  vector-specific dispatch logic anywhere: no vector decode of the uop at
  dispatch, no queue-ID field, no vector priority, no vector `Wire`.

  What the delta adds here is three branches in the dispatch loop and nothing
  more. `WithVector` appends three `IssueParams` entries to the tier's
  `issueParams`, and core.scala:837-849 ends in `require(false)` for an
  unrecognised `iqType` — SO A VECTOR CONFIG FAILS ELABORATION TODAY. Add one
  `else if` per `IQ_V_*` type, each wiring that entry as a real dispatch client:
    `v.io.dis_vec_valids(q)(w) := dispatcher.io.dis_uops(i)(w).valid`
    `dispatcher.io.dis_uops(i)(w).ready := v.io.dis_vec_ready(q)(w)`
  for every lane `w`, with `q` = 0/1/2 for load/store/ALU. Nothing is tied
  `true.B`; each entry's `ready` is a real per-lane back-pressure line out of
  `VecPipeline`.

  // ===> WHY `ready := true.B` UNDER `BasicDispatcher` WAS REJECTED (D2), AND IT
  // IS THE WHOLE REASON THE DISPATCHER CLASS CHANGES AT ALL. `BasicDispatcher`
  // computes
  // `ren_readys = io.dis_uops.map(d => VecInit(d.map(_.ready)).asUInt).reduce(_&_)`
  // — the ready is NOT masked by `iq_type`, so EVERY queue's ready ANDs into
  // EVERY lane. Wiring the vector queues in under it would let a full
  // `IQ_V_LOAD` stall PURE-SCALAR LANES CARRYING NO VECTOR UOP AT ALL, directly
  // threatening gate (d) and target P6. Tying those lanes ready avoids that stall
  // but throws the back-pressure away, so queue capacity would then have to ride
  // the seam's single broadcast `dis_ready` bit — the same scalar stall by a
  // longer route, plus a hack left in place. `CompactingDispatcher` already
  // implements the masking correctly:
  // `rdy := ren zip uses_iq map {case (u,q) => u.ready || !q}` — "the queue is
  // considered ready if the uop doesn't use it."
  // THE COST, RECORDED HONESTLY BECAUSE IT IS NOT FREE: a `Compactor` per queue,
  // SEVEN of them in a vector build, and THE SCALAR DISPATCH PATH DIFFERS IN A
  // VECTOR BUILD. A scalar-only regression that appears only on a vector config
  // must be attributed HERE and not to vector logic. What bounds that: at
  // `dispatchWidth == coreWidth` (true on every tier — see the `require` below)
  // each `Compactor` degenerates to `io.out <> io.in` (util.scala:458), so every
  // scalar queue's per-lane `valid`/`bits` are bit-identical to
  // `BasicDispatcher`'s and the ONLY behavioural difference is the `iq_type`
  // masking of `ren_uops(w).ready` — strictly FEWER stalls, never more.

  // ===> THE PAYLOAD DOES NOT CROSS THE SEAM, AND ONE `require` IS WHAT MAKES
  // THAT SOUND. The uop a vector queue latches is `VecPipeline`'s own lane-`w`
  // CHAINED-RENAME uop (the bundle with `pvdest`/`pvs*`/`pvl`/`stale_pvdest`
  // written), not `dispatcher.io.dis_uops(i)(w).bits`, so only `valid` and
  // `ready` cross. That is correct ONLY because dispatch lane `w` IS rename lane
  // `w`: every tier sets `dispatchWidth = coreWidth` on every queue
  // (config-mixins.scala:160/211/260) and `Compactor` with `n == k` is a straight
  // `io.out <> io.in`, so NO PERMUTATION EXISTS. Add
  // `require(ip.dispatchWidth == coreWidth)` for the three vector entries. If a
  // future tier narrows one, the compaction permutes and either the compacted
  // PAYLOAD or the permutation itself must cross the seam (`VecPipeline` part 5's
  // trap); the `require` turns that into a build failure instead of a queue
  // quietly latching a uop with unwritten vector PRNs and no width error.
  // The reverse obligation on `VecPipeline`: `dis_vec_ready` must be computed
  // FIRE-INDEPENDENTLY from registered queue occupancy, exactly as `dis_ready`
  // must be. `n == k` also makes `ren_uops(w).ready` independent of any `valid`,
  // so nothing closes dis_vec_ready -> ren_uops.ready -> dis_hazards -> dis_fire
  // -> ren_uops.valid; a fire-dependent `dis_vec_ready` would close it.

  // ===> AND THIS IS WHY SmallBoom IS NOT IN THE VECTOR MATRIX (D3).
  // `CompactingDispatcher` carries a constraint `BasicDispatcher` does not:
  // `issueParams.map(ip => require(ip.dispatchWidth >= ip.issueWidth))`. At
  // `coreWidth = 1` the `IQ_MEM` entry is forced to `issueWidth >= 2` by
  // `require(memWidth >= 2)` (parameters.scala:272) while
  // `dispatchWidth = coreWidth = 1`, so it becomes `require(1 >= 2)` — and
  // widening `dispatchWidth` is blocked by `require(dispatchWidth <= coreWidth)`
  // (parameters.scala:275). Small+vector therefore cannot elaborate at all. THE
  // VECTOR MATRIX IS MEDIUM/LARGE/MEGA and `WithNSmallBoomsVector` DOES NOT
  // EXIST. Medium(2)/Large(3)/Mega(4) all satisfy the constraint as written
  // (`IQ_MEM` issueWidth 2/2/3, `IQ_ALU` 2/3/4). Independent second reason Small
  // is out: `allocWidth = coreWidth*8 = 8`, but a shared `OP.v` needs
  // `2*maxGroupSize = 16` PRNs all-or-nothing — a DEADLOCK, not a stall.
  // The other alternative — drop the three `issueParams` entries in `WithVector`
  // — was rejected because `VecIssueUnit` is parameterised from them.

  //@req-spec-issue.a1
  //@req-spec-issue.a2
  Dispatch keeps its position and its meaning: it is the stage between rename and
  the issue queues, and the last IN-ORDER stage before instructions go
  out-of-order. The vector subsystem attaches to that stage and does not move,
  duplicate or bypass it — `ren2_uops`/`dis_fire` are dispatch-cycle signals, the
  vector queue slot write happens in that same cycle (`VecPipeline` part 5), and
  no register, queue or second dispatch stage is inserted between rename and any
  queue, vector or scalar.

  //@req-spec-issue.a4
  The LDQ/STQ reservation stays where only an in-order stage can do it and stays
  BYTE-FOR-BYTE as it is: `dis_uops(w).ldq_idx := io.lsu.dis_ldq_idx(w)` and
  `dis_uops(w).stq_idx := io.lsu.dis_stq_idx(w)` (core.scala:780-784), with
  `io.lsu.ldq_full(w) && dis_uops(w).uses_ldq` still in `dis_hazards`. A vector
  memory OP.v takes ONE placeholder LDQ or STQ entry through this same path, in
  program order, and carries the index onward on the uop; the vector element
  queues are reserved against that index inside `VecPipeline`, so no second
  allocator and no vector-specific reservation term appears here.

  //@req-spec-issue.c2
  A segmented load or store is dispatched to BOTH the CII issue queue and its own
  load/store path, and dispatch time is where that happens — with no logic,
  because `iq_type` is a MASK and not an identifier. One uOP with `is_shared` set
  has two `IQ_V_*` positions set and is presented to both queues in the same
  cycle by the same fan-out that presents any other uop to one queue. It also
  takes its LDQ/STQ entry from the path above. Nothing splits, cracks or
  replicates the uop in this file, and there is no port here on which a cracked
  uop could exist.

  ---- PART 5. Speculation, recovery and commit fan-out ----

  Nine straight connections, every one of them from a signal that already exists
  and drives the scalar equivalents in the same cycle, so no vector consumer can
  see a different recovery view than the scalar pipeline does:
  `brupdate` (the merged `BrUpdateInfo`); `rob_pnr_idx := rob.io.rob_pnr_idx`;
  `rob_head_idx := rob.io.rob_head_idx`; `rob_empty := rob.io.empty`;
  `rob_flush := rob.io.flush.valid`; `rob_flush_kill :=
  RegNext(rob.io.flush.valid)`; `commit_valids := rob.io.commit.valids`;
  `commit_uops := rob.io.commit.uops`; `commit_rollback := rob.io.rollback`.

  `rob_head_idx` is load-bearing and pre-existing (it already feeds
  `iss_unit.io.rob_head` and `io.lsu.rob_head_idx`): BOOM's age test is the
  three-argument `IsOlder(a, b, head)`, so `IQ_V_ALU`'s past-PNR gate is not
  computable from `rob_idx` and `rob_pnr_idx` alone and would INVERT across a ROB
  wrap, handing a still-speculative op to the coprocessor.

  // `rob_flush_kill` is `RegNext(rob.io.flush.valid)`, the SAME expression the
  // scalar issue units, `fp_pipeline` and every `eu.io_kill` use. Compute it
  // once at the connection and do not let `VecPipeline` build a second
  // `RegNext` of `rob_flush`: a divergent copy kills vector slots a cycle away
  // from the LSQ pointer rollback and from the CII kill window.

  ---- PART 6. Completion into the ROB, and the one merge this file owns ----

  Three lane-preserving connections: `rob.io.vec_clr_bsy := v.io.vec_clr_bsy`
  (`numVecClrPorts` = 3 lanes), `rob.io.vec_rob_flags := v.io.vec_rob_flags`
  (same lanes) and `rob.io.vec_clr_unsafe := v.io.vec_clr_unsafe`. These are
  connections, not reductions: NEVER arbitrate, OR-reduce or `Mux1H` the clear
  lanes. A clear lost to arbitration is unrecoverable, because `VecCiiComplete`
  frees its tag and the LCB releases its assembly entry in the same cycle they
  complete, so the ROB entry would never retire and the machine would hang with
  no assertion.

  `vec_xcpt` has nowhere to go but a merge, and that is why the merge is here:
  `Rob`'s reject list forbids a second exception port and `LSU`'s forbids a
  vector fault path of its own, while `rob.io.lxcpt` is a single
  `Flipped(Valid(new Exception))` (rob.scala:75). So this file drives
  `rob.io.lxcpt` from `io.lsu.lxcpt` and `v.io.vec_xcpt` by AGE, oldest wins,
  using `IsOlder(a.uop.rob_idx, b.uop.rob_idx, rob.io.rob_head_idx)` — the same
  three-argument form the ROB itself uses at rob.scala:670 — and translating
  `VecException` into `{uop, cause, badvaddr}`. The loser is DROPPED, and that is
  safe for one specific reason worth writing down: the exception that is taken
  flushes everything younger, so the dropped (younger) faulting op is squashed and
  re-executes, and re-faults. `vstart` takes no part in this: a faulting vector
  access traps with `vstart = 0` and restarts whole, and `fault_elem` never
  reaches the ROB.

  ---- PART 7. The scalar feeder seams: INT reads, the writeback snoop, wakeups ----

  //@req-spec-core.e15
  Every vector execution unit reaches its scalar feeders — the base address, the
  stride, the `.vx` integer operand and the `.vf` FP operand — through read ports
  into the SCALAR register files and the integer bypass information, and this file
  is where those ports are physically attached. Five INT lanes and ONE FP lane,
  appended AFTER every existing exe-unit port so no scalar arbitration index
  moves: `iregfile.io.arb_read_reqs(numScalarLogicalReadPorts + i) <>
  v.io.int_rf_read_req(i)` for i in 0 until 5, with
  `v.io.int_rf_read_rsp(i) := iregfile.io.rrd_read_resps(numScalarLogicalReadPorts + i)`.
  The response is REGISTERED: `PartiallyPortedRF` reads
  `regfile(RegNext(arb_read_reqs.bits))`, so an address presented in cycle N
  returns data in cycle N+1, once. Every vector consumer must match that, and it
  is the same latency the scalar exe units already live with.

  ONE FP LANE, NOT TWO: `fp_rf_read_req` is 1, because decision D4 DELETED the
  store-side FP reader — no RVV store form takes an FP scalar operand, since store
  data is always `vs3` and the scalar operands are `rs1` (base) and `rs2` (stride),
  both integer. `VecCiiIssue`'s `.vf` read is the only FP reader on this seam, so
  no tier needs an extra physical FP read port for it.

  // ===> DECIDED (D5), AND THIS FILE COULD NEVER HAVE PAPERED OVER IT: THE INT
  // READ SEAM NEEDS A `ready` AND HAD NONE. `arb_read_reqs` is `Flipped(Decoupled(...))` and
  // `PartiallyPortedRF` sets `ready(i) := PopCount(earlier valids) <
  // numPhysicalReadPorts` — the INT file is DELIBERATELY partially ported
  // (`numIrfReadPorts` is 3 on Medium, 4-6 elsewhere, against roughly ten
  // logical readers), so a read IS denied whenever enough earlier ports are
  // active. There is no placement that fixes it: putting the vector lanes first
  // both steals ports from the scalar pipeline and still fails, since 5 > 3.
  // Raising `numIrfReadPorts` to the logical count is the most expensive change
  // available in this core. The only sound answer is to expose the existing
  // BOOM mechanism: make the seam per-lane `Decoupled` and let
  // `VecScalarOperandRead` / `VecCiiIssue` hold the address until `fire`,
  // exactly as every scalar exe unit holds in its arb stage. That is ground
  // rule 10 (reuse BOOM's machinery) and it is NOT a `busy` reaching an issue
  // unit — the grant has already happened. That is exactly what D5 ADOPTED:
  // `int_rf_read_req` is per-lane `Decoupled`, the holding is in
  // `VecScalarOperandRead` / `VecCiiIssue`, and the absorbing structure is
  // `VecLsu`'s per-LDQ/STQ-entry descriptor pending table, written AT THE GRANT
  // so the `FC_AGEN`/`FC_DGEN` grant itself stays unqualified. So the amendment
  // to `vec_pipeline_io` and to those two child specs is DECIDED, not merely
  // reported. Its FP counterpart A32 is MOOT (D4/D3): the FP read-port shortfall
  // only ever arose from needing two FP lanes on a Giga tier that is not in the
  // vector matrix.

  `int_wb_snoop` is tapped from the write ports themselves, one lane per INT
  write port including the vector one added in part 8:
  `{valid, addr, data} := iregfile.io.write_ports(i).{valid, bits.addr, bits.data}`,
  with NO `RegNext` anywhere on the path. This single tap discharges both halves
  of e15's "register file and integer bypass network", and the reason is a timing
  coincidence in the baseline worth stating: the ALU bypass entry
  (core.scala:965) and the ALU write port (core.scala:975) are driven from the
  SAME `unit.io_alu_resp` in the SAME cycle, and the LSU's bypass entry and write
  port are both `RegNext(io.lsu.iresp(i))`, so the write-port snoop carries the
  bypass network's information at the bypass network's cycle, plus the `ll_arb`
  port that the bypass network does not have. A separate `int_bypasses` member on
  the seam would be the same wires under a second name. It is also the only fix
  for the M1 stale-scalar-base bug: `FullyPortedRF` is a plain `Mem` with a
  registered address and NO read-during-write forwarding, so a read landing in
  the same cycle as the base GPR's write returns the stale value, and closing
  that needs the DATA, which `int_wakeups` does not carry.

  `int_wakeups` is aggregated into the seam's `IntWakeupBus` from three signals
  that already exist, with no new network and no register: `wakeups :=
  int_wakeups` (the `numIntWakeups`-wide vector, after part 8 appends its slot),
  `child_rebusys := alu_exe_units.map(_.io_child_rebusy).reduce(_|_)` and
  `squash_grant := alu_exe_units.map(_.io_squash_iss).reduce(_||_) ||
  io.lsu.iwakeups.map(_.bits.rebusy).reduce(_||_)` — character-for-character the
  term `alu_iss_unit.io.squash_grant` gets at core.scala:1050-1053, so a vector
  slot and a scalar ALU slot re-busy on the same cycle for the same reason.

  `fp_wakeups := fp_pipeline.io.wakeups` is a pure tap of the existing FP wakeup
  vector, no `RegNext`, no added port in `fp-pipeline.scala`: without it there is
  no FP readiness path at all and a `.vf` op can never become ready (A29). The FP
  read seam is `fp_pipeline.io.vec_frf_read_req` / `vec_frf_read_rsp`.

  ---- PART 8. Writeback: the BOOM v4 design, widened ----

  //@req-spec-core.e21
  The writeback arbitration is BOOM v4's, WIDENED for the vector write ports and
  not replaced. Concretely: `ll_arb` keeps its exact input list and priority
  order; the `lsuWidth` LSU write ports and the `aluWidth` ALU write ports keep
  their indices, their `RegNext` structure and their `dst_rtype === RT_FIX`
  qualifiers; the FP writeback loop into `rob.io.wb_resps` (core.scala:1227-1231)
  is untouched. The delta appends ONE INT write port and ONE wakeup slot after
  the existing loop:
  `iregfile.io.write_ports(wb_idx) := {valid = v.io.int_wb.valid, addr =
  v.io.int_wb.bits.uop.pdst, data = v.io.int_wb.bits.data}` and
  `int_wakeups(wu_idx) := {valid = v.io.int_wb.valid, uop = v.io.int_wb.bits.uop,
  rebusy = false, bypassable = false, speculative_mask = 0}`, then both indices
  advance and the two existing `require`s still hold. `bypassable` is false and
  no `int_bypasses` entry is added: this result comes from a coprocessor
  completion, not from an ALU, and there is no cycle in which a dependent could
  use a bypass.

  A dedicated port rather than another `ll_arb` input, deliberately: `ll_arb` can
  DENY, and `VecCiiWriteback` pops its beat and returns its credit in the cycle it
  writes back, so a denied writeback is unrecoverable. This port is also why
  `int_wb_snoop` must be sized `numIrfWritePorts` and not the scalar count — a
  base address produced by `vmv.x.s` is otherwise invisible to the stale-base
  forward of part 7.

  // ===> `fp_wb` NOW HAS A LANDING SITE, AND A31 IS CLOSED (decision D7).
  // `vfmv.f.s` needs an FP register-file write port plus an FP wakeup slot, both
  // of which live inside `fp-pipeline.scala`. D7 gives it exactly that: a
  // DEDICATED FP write port plus a dedicated wakeup slot, added in `FpPipeline`
  // under `usingRVV`, and `FpPipeline`'s reject list — which previously refused
  // this port — was FORMALLY AMENDED to permit it. So `fp_wb` is a connected
  // member of this seam like any other, and this file neither ties it off nor
  // waits on an owner.
  // THE `ll_wports` ROUTE WAS REJECTED, and the reason is the same one that made
  // `int_wb` a dedicated port two paragraphs up: `ll_wbarb` is an `Arbiter`
  // (in(0) = mem, in(1) = ifpu, in(2) = fdiv) into `write_ports(0)`, so a fourth
  // input CAN BE DENIED, the CII channel has NO back-pressure, and no bound on
  // the denial is constructible — the arbiter can lose to mem AND ifpu AND fdiv
  // while consecutive scalar-FP CII ops produce back-to-back beats. `vfmv.f.s`
  // being RARE is what makes that dangerous rather than acceptable: the failure
  // is a once-in-a-blue-moon wrong FP register value with no assertion anywhere.
  // Second, independent reason: everything joining `ll_wbarb` is hardfloat-
  // `recode`d, while `VecCiiWriteback` emits IEEE.
  // ===> THE WIRING OBLIGATION THIS PUTS ON THIS FILE, and it is two connections,
  // not one. (1) Drive `FpPipeline`'s new dedicated vector FP writeback input
  // from `vec.get.io.fp_wb`. (2) The extra `fp_pipeline.io.wakeups` / `io.wb`
  // entry must REACH THE ROB: `numFpWakeupPorts` is `fp_pipeline.io.wakeups.
  // length` (core.scala:116), so it follows D7's added slot automatically, the
  // `Rob` constructor argument of the parameters section keeps its
  // `+ numFpWakeupPorts` term unchanged, and the existing FP loop over
  // `fp_pipeline.io.wb` at core.scala:1227-1231 carries the new entry into
  // `rob.io.wb_resps` with `require(cnt == rob.numWakeupPorts)` still holding.
  // That loop is NOT special-cased: it applies `ieee(wb.bits.data)` uniformly, so
  // the new entry must be presented recoded like every other FP wb entry, and the
  // IEEE-to-recode conversion of the CII payload belongs on `FpPipeline`'s input
  // side. This is the ONE place a `wb_resps` entry is added — the INT vector
  // writeback still reaches the ROB only through `vec_clr_bsy` lane 1 and
  // `vec_rob_flags`, and the `Rob` argument still must NOT follow the widened
  // `numIrfWritePorts`.

  ---- PART 9. `vset_resp` and the multi-ALU ruling ----

  In the existing per-ALU loop (core.scala:961-983), one line per column:
  `v.io.vset_resp(i).valid := unit.io_alu_resp.valid` and
  `v.io.vset_resp(i).bits := unit.io_alu_resp.bits`.

  // ===> TAP `io_alu_resp` RAW. The two existing uses at core.scala:965 and :975
  // qualify it with `dst_rtype === RT_FIX`; reusing that qualifier here DROPS
  // the VL write of `vsetvli x0, rs1`, which discards its integer destination
  // and is the common idiom. The write enable on the VL side is
  // `bits.uop.is_vl_producer` and never a register type — that is `ALUUnit`'s
  // ruling and it is only visible from this end. The tap is also unqualified by
  // `IsKilledByBranch`, matching the integer write port it accompanies: the VL
  // write lands in a renamed `pvl` that rollback reclaims, and killing one of
  // the two writes but not the other is the inconsistency to avoid.

  // ===> RESOLVED HERE: THE PORT IS REPLICATED, NEVER ARBITRATED (A32/the
  // aluWidth question). `ALUExeUnit` advertises the vset functional unit on
  // EVERY ALU EU and its reject list forbids an `id`-conditional advertisement,
  // so at `aluWidth >= 2` two `vsetvli`s CAN write back in the same cycle
  // (`vsetvli`/`vsetivli` are not `is_unique`; only `vsetvl` is). A single
  // `vset_resp` would therefore have to arbitrate, and the losing VL wakeup is
  // single-shot in BOOM's slot model — a lost wakeup is a permanent hang, the
  // same argument that made `vec_clr_bsy` and `vl_wakeup` per-producer lanes.
  // Settled: `vset_resp` becomes `Vec(aluWidth, Valid(ExeUnitResp))`,
  // `VlRegFile.numAluWritePorts` becomes `aluWidth` (its own file already says
  // "replicate the port, never arbitrate" for exactly this case), and
  // `numVlWakeupPorts` becomes `aluWidth + 1`. At `aluWidth = 1` this is
  // bit-identical to the single-port shape, so Small/Medium/Large are unaffected
  // and only Mega/Giga pay for it. Restricting vset to one ALU column is the
  // rejected alternative: it is a change to `ALUExeUnit`'s advertisement, which
  // that node forbids, and it would serialize a common scalar instruction.

  ---- PART 10. The CSR seam ----

  //@req-spec-decode.g2
  `csr_vector` carries rocket's OUTPUT-direction vector state into `VecPipeline`
  — `vconfig` (with `vtype`), `vstart` and `vxrm` — and the four INPUT-direction
  fields are driven from here, all four from COMMIT rather than from writeback,
  because a past-PNR coprocessor op can still be squashed by a ROB-head flush:

    `csr.io.vector.get.set_vconfig.valid := v.io.commit_vl.valid`
    `csr.io.vector.get.set_vconfig.bits.vl := v.io.commit_vl.bits`
    `csr.io.vector.get.set_vconfig.bits.vtype := <the same committing uop's vconfig>`
    `csr.io.vector.get.set_vxsat := rob.io.com_vxsat`
    `csr.io.vector.get.set_vs_dirty := v.io.csr_vs_dirty`
    `csr.io.vector.get.set_vstart := {valid = a vector uop commits, bits = 0}`

  `commit_vl` is the VL register file's commit read DATA; its ADDRESS was
  selected inside `VecPipeline` from `commit_uops`, so the `vtype` half must use
  the SAME selection — the youngest committing lane with `is_vl_producer` set,
  taking `rob.io.commit.uops(w).vconfig`, which the `Rob` delta has already
  overwritten from its per-row `rob_vconfig` so a register-sourced `vsetvl`
  commits the vtype its ALU RESOLVED. Assert that `commit_vl.valid` agrees with
  the local selection's validity, because the two selects are the same expression
  written in two files and that is the only thing keeping them equal.

  `set_vstart` is written as a constant ZERO on every vector commit, and it is
  the CSR-side half of the design's precise-exception rule: Caracal never writes
  a non-zero `vstart`, so the only way `vstart` becomes non-zero is a software
  write, and a vector instruction that honours that prefix must clear it when it
  retires. `csr_frm := csr.io.fcsr_rm`, the same value `all_exe_units` and
  `fp_pipeline` already get. Note that rocket asserts `set_vconfig.bits.vl <=
  set_vconfig.bits.vtype.vlMax` (CSR.scala), which is a live check on the pair
  crossing this seam.

  ---- PART 11. The memory seam and the fence fold ----

  `io.lsu.lsu_vec <> v.io.lsu_vec` — one bundle connect; `VecLsu` owns its
  contents and this file inspects no member of it. `io.lsu.vec_lsu_empty :=
  v.io.lsu_fencei_rdy_vec`.

  //@req-spec-memord.f8
  //@req-spec-memord.f9
  The dispatch-side fence condition is UNCHANGED and that is how these two are
  discharged: `wait_for_empty_pipeline` at core.scala:739-740 stays
  `(dis_uops(w).is_unique || !custom_csrs.enableOOO) && (!rob.io.empty ||
  !io.lsu.fencei_rdy || dis_prior_slot_valid(w))`, so an `is_unique` uop still
  cannot dispatch until the ROB is empty AND `fencei_rdy` is asserted. The vector
  LSU's drain state reaches that same term by being folded into
  `io.lsu.fencei_rdy` INSIDE the LSU (`vec_lsu_empty` is an input of
  `LSUCoreIO`), not by a new term at the dispatch site. That placement is
  deliberate: it keeps one fence condition instead of two, and a head-side
  handshake with the vector subsystem would deadlock against an `is_unique` uop
  that is itself waiting to dispatch.

  ---- PART 12. Trace, debug, and what is NOT here ----

  `v.io.vec_trace_en` is driven from the `vecTrace` plusarg that `VecTrace`
  defines, read ONCE in this file and fanned in as a single bit, off by default.
  `dontTouch(v.io.debug_vrf_read)` so the waveform and the Whisper cosim can read
  the VRF; it reaches no functional logic. `v.io.vl_wakeup` is read by nothing
  here (part 6 of `VecPipeline` forms and consumes the VL network internally),
  and is left as an observability output.

  The pre-existing `boom_timeout` hang assertion is untouched — with no unit
  tests anywhere in this project it is the primary bring-up signal for every
  vector deadlock the ground rules exist to prevent.

  The assertions this delta adds, three: `commit_vl.valid` agrees with the local
  youngest-VL-producer select (part 10); the `rob.io.lxcpt` merge keeps the older
  of its two inputs (part 6); and no uop reaches `rob.io.enq_uops` with
  `dst_rtype === RT_VEC` unless it also names a vector queue in `iq_type` — the
  cheap end of A23, whose real fix is `DecodeUnit`'s six default assignments and
  which gate (f) cannot see, because a don't-care bit elaborates bit-identically
  and still mis-routes.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
THIS DELTA ADDS NO PIPELINE STAGE AND NO CYCLE OF LATENCY TO ANY EXISTING PATH,
and that is a constraint on the generated edit rather than a target:

- Decode: `dec_uops(w)` gains the vector merge's logic depth and NOTHING else. If
  that fails timing the fix is inside `VecDecode`, never a register here: a
  register between `decode_units` and `dec_uops` breaks BOOM's one-cycle decode
  contract and the `dec_stalls` prefix scan with it.
- Rename/dispatch: `dis_uops` to `rob.io.enq_uops` now passes through two chained
  vector rename spaces — the delta's longest new combinational path, and the
  reason vector rename must be single-stage, since `rob.io.enq_valids` is
  `dis_fire` and the enqueued uop must be valid in the dispatch cycle.
- Dispatch (vector builds only, D2): `dispatcher.io.ren_uops(w).ready` gains ONE
  OR level per queue — `u.ready || !uses_iq` before the cross-queue AND — on a
  path that already feeds the `dis_hazards` scan. That is the price of not
  stalling scalar lanes on a full vector queue, and it is why the payload stays
  off the seam: at `dispatchWidth == coreWidth` each `Compactor` is a wire, so
  nothing else in the dispatch path gets deeper. All of it disappears when
  `usingRVV` is false, because the class does.
- The four widened counts cost area, not time: one INT write port, one INT wakeup
  comparator column, five INT logical read ports (mux width per
  `PartiallyPortedRF` bank, no storage). All zero when `usingRVV` is false.
- Every recovery and completion connection is a wire, unregistered, in the same
  cycle the scalar consumers see it. A `RegNext` on any of them skews a
  single-shot clear or wakeup by a cycle, which is a hang, not a slowdown.
- The part-6 merge adds one `IsOlder` and a 2:1 mux in front of `rob.io.lxcpt`, a
  path that already had a mux (`lxcpt_older`, rob.scala:670).
<|end_perf|>

<|begin_dependencies|>
Instantiates, and this is the whole instantiation delta: `VecPipeline` as `vec`,
once, `Option`-wrapped under `usingRVV`, connected by `vec_pipeline_io`. Every
pre-existing instantiation in this file — `decode_units`, `rename_stage` and its
three siblings, `dispatcher`, the four issue units, `iregfile`/`pregfile`/
`immregfile`/`bregfile`, `rob`, `csr`, `fp_pipeline`, `ll_arb`, `ftq_arb`, the
ALU/Mem/Unique exe units — is untouched in count, parameters and order, with ONE
named exception: `dispatcher`'s CLASS is now config-selected —
`CompactingDispatcher` when `usingRVV`, `BasicDispatcher` otherwise (D2, part 4).
Its instance name, position, constructor arguments (none) and feed are unchanged,
and a non-vector build gets the same class it gets today.

Declaration dependencies: `MicroOp` (the seam's uop members, and `vconfig` on
every br_tag-allocating uop), `ScalarOpConstants` (the `IQ_V_*` positions used by
the dispatcher's bit test and by the added assertion, and `RT_VEC`),
`BoomCoreParams` (`usingRVV`, `numVecIrfWritePorts`, the vector sizes),
`VecBundles` (`VecPipelineIO`, `VecException`, `VecRobFlags`, `IntWakeupBus`,
`FpWakeupBus`, `IntWbSnoop`). Binds to `freechips.rocketchip.rocket.VConfig` and
to rocket's `CSRFileIO.vector` option for the CSR seam.

Sibling deltas this file connects to, and the port names it relies on:
`DecodeUnit` (`io.vec.uop_to_vdec` / `uop_from_vdec` / `illegal`), `ALUUnit` and
`ALUExeUnit` (the value on `io_alu_resp` for a `is_vl_producer` uop), `Rob`
(`io.vec_clr_bsy`, `io.vec_clr_unsafe`, `io.vec_rob_flags`, `io.com_vxsat`),
`LSU` (`io.lsu_vec`, `io.vec_lsu_empty`), `FpPipeline`
(`io.vec_frf_read_req`/`_rsp`, and the DEDICATED vector FP write port plus wakeup
slot that lands `fp_wb` — decision D7, its reject list amended to permit them).

---- AMENDMENTS THIS FILE REQUIRES OF OTHERS (Phase R step R2) ----

1. `vec_pipeline_io` + `VecPipelineIO`: ADD `dis_uops_out` (bwd,
   `MicroOp * coreWidth`); give `int_rf_read_req` a per-lane `ready`; widen
   `vset_resp` to `Vec(aluWidth, Valid(ExeUnitResp))`; ADD `dis_vec_valids` (fwd,
   `coreWidth * 3`) and `dis_vec_ready` (bwd, `coreWidth * 3`) for D2's native
   dispatch handshake (part 4).
2. `VlRegFile`: `numAluWritePorts = aluWidth` (its own text already prescribes
   replication for this case); `VecPipeline`: `numVlWakeupPorts = aluWidth + 1`
   and part 6 lane list re-indexed accordingly.
3. `VecScalarOperandRead` and `VecCiiIssue`: hold the INT read address until
   `fire`, as every scalar exe unit does; the INT-RF response is registered at
   t+1 relative to a GRANTED address, not to the presented one.
4. `FpPipeline`: a DEDICATED FP write port plus a dedicated wakeup slot to land
   `fp_wb`, `usingRVV`-gated, with its reject list amended to permit them (D7,
   A31 CLOSED). NOT `io.ll_wports`, which is arbitrated and can deny (part 8).
   The write-port count is `numFrfWritePorts = fpWidth + lsuWidth +
   (if (usingRVV) 1 else 0)` — APPEND one, NEVER assign a literal 2: the baseline
   is 2 on Medium/Large but 3 on Mega, so a hard-coded 2 silently DROPS Mega's
   second exe-unit write port. There is also just ONE `vec_frf_read_req` lane, so
   no extra physical FP read port is needed in any tier (`numFrfReadPorts` stays
   3 on Medium/Large): D4 deleted the store-side FP reader, since no RVV store
   form takes an FP scalar operand.
5. `BoomConfigMixins`: the three `IQ_V_*` `issueParams` entries stay (they
   parameterise `VecIssueUnit`) and each must be appended with
   `dispatchWidth = coreWidth` and `issueWidth <= coreWidth`, because a vector
   config now instantiates `CompactingDispatcher` (D2) and this file `require`s
   both (part 4). DROP `WithNSmallBoomsVector`: the vector matrix is
   MEDIUM/LARGE/MEGA (D3).
6. `VecPipeline`: consume `dis_vec_valids` / drive `dis_vec_ready` per lane, and
   note that with `dispatchWidth == coreWidth` the `Compactor` is an identity
   `<>`, so part 5's compaction trap is discharged by the `require` in part 4 —
   neither the payload nor a permutation crosses the seam, and the vector queues
   keep latching this container's own lane-`w` chained-rename uop.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File   src/main/scala/v4/exu/core.scala
    Class  `class BoomCore(roccCSRs: Seq[Seq[CustomCSR]])(implicit p: Parameters)
            extends BoomModule with HasBoomFrontendParameters with HasFPUParameters`
    (package boom.v4.exu). Hand-written baseline BOOM v4.
    Added-line budget: 200 (plan v2 §5 rule 3 and §11; `addvector` added 1101).

  In scope:
    - The elaboration values `numIrfWritePorts`, `numIntWakeups`,
      `numIrfLogicalReadPorts`, `iregfileBankedWriteArray` (append `None` at the
      END only), and the `Module(new Rob(...))` argument expression, which must
      keep its present VALUE.
    - One added instance: `val vec = if (usingRVV) Some(Module(new VecPipeline))`
      and its single `io` connect block.
    - Inside the decode loop (524-535): three `decode_units(w).io.vec.get`
      connections plus the four decode-feed members. `dec_uops(w) :=
      decode_units(w).io.deq.uop` unchanged.
    - `ren_stalls(w)` at 725: one appended `|| vec_stall` term.
    - `rob.io.enq_uops` at 790: source changed to `vec.get.io.dis_uops_out`.
    - The `dispatcher` INSTANTIATION SITE: `Module(if (usingRVV) new
      CompactingDispatcher else new BasicDispatcher)` (D2). Instance name,
      position and feed unchanged.
    - The dispatcher loop at 837-849: three added `else if` branches, one per
      `IQ_V_*` iqType, each wiring that entry NATIVELY from
      `dispatcher.io.dis_uops(i)` — per-lane `valid` out to `dis_vec_valids`,
      per-lane `ready` in from `dis_vec_ready`. No `ready := true.B` anywhere.
      Plus the `require(ip.dispatchWidth == coreWidth)` those three entries need.
    - The per-ALU loop at 961-983: two added lines for `vset_resp(i)`.
    - After the wakeup/writeback loops: one appended `iregfile.io.write_ports`
      entry and one appended `int_wakeups` entry, with `wb_idx`/`wu_idx`
      advancing.
    - The register-read arbitration loop at 1064-1073 and the response loop at
      1088-1096: five appended INT lanes after `arb_idx`/`rd_idx` reach the
      scalar count, and the two `require`s updated to the new totals.
    - `rob.io.lxcpt`: replaced by the two-input age merge of part 6.
    - `rob.io.vec_clr_bsy`, `rob.io.vec_rob_flags`, `rob.io.vec_clr_unsafe`,
      `io.lsu.lsu_vec`, `io.lsu.vec_lsu_empty`, `csr.io.vector.get`'s four input
      fields, `fp_pipeline.io.vec_frf_read_req`/`_rsp` and `FpPipeline`'s new
      dedicated vector FP writeback input (D7, part 8): new connections only.
    - Three added `require`s, three added assertions, one `dontTouch`.

  Must not regress — bit- and cycle-identical, named because they are the parts
  most at risk:
    - `dec_valids`, `dec_fire`, `dec_ready`, `dec_finished_mask`, `dec_xcpts`,
      `dec_prior_slot_valid`, `dec_xcpt_stall`, `dec_hazards`, `dec_stalls`, and
      the whole of `dec_brmask_logic`'s wiring including
      `dec_uops(w).br_tag`/`br_mask`. In particular `dec_ready := dec_fire.last`
      and the scan structure of `dec_stalls` are untouched, and no vector signal
      appears in `dec_hazards`.
    - `dis_valids`, `dis_fire`, `dis_ready := !dis_stalls.last`, `dis_hazards`
      and every one of its fourteen terms — including
      `wait_for_empty_pipeline` (739-740) verbatim, `!dispatcher.io.ren_uops(w).
      ready`, the `ldq_full`/`stq_full` terms, `brtag_stall`, the RoCC terms.
      Every term keeps its TEXT; no vector term is added to `dis_hazards`. The
      vector ALLOCATION answer enters only through `ren_stalls(w)`, and vector
      QUEUE CAPACITY enters only through the pre-existing
      `!dispatcher.io.ren_uops(w).ready` term, whose VALUE differs in a vector
      build because of the dispatcher class (D2) and not because anything was
      added here.
    - The cross-rename patch block at 703-723: every `prs1`/`prs2`/`prs3`/
      `ppred`/`pdst`/`stale_pdst`/`imm_sel`/`pimm` mux and every `*_busy`
      expression, unchanged. No vector field is patched into `dis_uops`.
    - `dis_uops(w).ldq_idx`/`stq_idx`/`rob_idx`/`rxq_idx` assignments (780-813)
      and `rob.io.enq_valids`/`enq_partial_stall`.
    - `dispatch.scala`: NOT EDITED AT ALL — neither `BasicDispatcher` nor
      `CompactingDispatcher` nor `DispatchIO` changes, because both dispatchers
      already exist there and already carry every port D2 needs. What DOES change
      is `core.scala`'s dispatcher CHOICE, which now depends on `usingRVV`. The
      regression obligation is therefore narrower and sharper than "not edited":
      with `usingRVV = false` this file must still instantiate `BasicDispatcher`
      and the emitted dispatch logic must be BIT-IDENTICAL — same class, same
      `ren_readys` reduction, same three `else if` scalar branches, same
      `require(false)` fallthrough reachable by nothing. With `usingRVV = true`
      the scalar dispatch path deliberately DIFFERS (`CompactingDispatcher`'s
      `iq_type`-masked ready); that difference is D2's accepted cost, is bounded
      to the ready masking at `dispatchWidth == coreWidth` (part 4), and must not
      be "fixed" by reverting to `BasicDispatcher`.
    - `ll_arb`: input count, input order, priority, `ready := true.B`, and the
      `csr`/`mul`/`div`/`f2i`/`rocc` entries. No vector input.
    - The existing INT write ports and wakeup slots: the `lsuWidth` LSU entries
      (877-889), the `ll_arb` entry (944-957) and the `aluWidth` entries
      (961-983), each keeping its index, its `RegNext` structure, its
      `IsKilledByBranch` term and its `dst_rtype === RT_FIX` qualifier — the raw
      `io_alu_resp` tap of part 9 is ADDITIONAL and changes neither line 965 nor
      line 975.
    - `int_bypasses` (width and contents), `pred_wakeups`/`pred_wakeup`,
      `pregfile`/`bregfile`/`immregfile` writes, `imm_rename_stage`'s wakeups.
    - `rob.io.wb_resps` in its entirety, including the FP loop at 1227-1231 and
      `require(cnt == rob.numWakeupPorts)` — the loop's TEXT is untouched; it
      simply iterates one entry further in a vector config, because D7's added FP
      wakeup slot raises `numFpWakeupPorts` (part 8). At `usingRVV = false` the
      entry count is identical; `rob.io.lsu_clr_bsy`,
      `rob.io.lsu_clr_unsafe`, `rob.io.csr_replay`.
    - `brupdate`/`b1`/`b2`/`brinfos` and the frontend redirect block (404-488).
    - `csr.io` in every existing respect: `fcsr_flags`, `set_fs_dirty`, `retire`,
      `exception`, `pc`, `cause`, `tval`, `decode(w)`, `counters`, `customCSRs`.
    - The `trace`, `COMMIT_LOG_PRINTF`, `BRANCH_PRINTF`, `TraceCoreIngress`,
      PTW and RoCC blocks, and the `boom_timeout` hang assertion.
    - With `usingRVV = false`: every value, index, port and instance above is
      literally what it is today, since each added quantity is
      `if (usingRVV) ... else 0` and the instance is `None`.

    // ===> AND THE ONE HONEST QUALIFICATION ON THAT LAST LINE. This
    // must-not-regress list is the artifact that carries plan gate (f) for
    // core.scala, so it must say plainly what gate (f) now means.
    //
    // RESOLVED by decision D1: gate (f) is NO LONGER "bit-identical to
    // pre-Caracal BOOM v4". It is "identical to the RE-BASELINED REFERENCE,
    // except for the enumerated encoding widths", and the diff target is
    // `docs_caracal/v2-rebaseline/` rather than pre-Caracal RTL. The exception
    // is exactly four items, enumerated in plan section 6a: the `RT_*` group
    // 2b -> 3b, `IQ_SZ` 4 -> 7 (hence `MicroOp.iq_type`), `MicroOp`'s three
    // `*_rtype` fields, and `Rob`'s compact `dst_rtype` tracking `MicroOp`.
    // The cause is that `ScalarOpConstants` is a bare Scala trait with no
    // `Parameters` in scope, so it cannot gate those widths on `usingRVV`; the
    // alternative of parameterizing them was rejected because that trait is
    // consumed during `issueParams` construction before `Parameters` exists.
    //
    // WHAT THAT MEANS FOR THIS FILE, unchanged by the ruling: every quantity
    // here is gated on `usingRVV` and the instance is absent, so a vectors-off
    // build contributes ZERO to the diff from this delta. The widened encodings
    // a reviewer will see in `MicroOp` and `Rob` are NOT caused by this file
    // and cannot be fixed here.
    // ===> SO DO NOT ATTRIBUTE A GATE (f) FAILURE HERE, and do not check this
    //      list against pre-Caracal RTL — that comparison will show the four
    //      enumerated width differences and they are expected. The check is
    //      against the re-baselined reference, which is a checked-in artifact
    //      that must EXIST before gate (f) means anything at all (plan section
    //      6a makes generating it a step, analogous to A0).
    // Note separately that a PASSING gate (f) does not prove the `IQ_V_*`
    // defaulting obligation is met: a don't-care bit can elaborate
    // bit-identically and still mis-route. That check is a read of the
    // `DecodeUnit` delta plus an assertion, never a clean diff.

  Interface delta:
    NEW ports on `class BoomCore`'s `io`: NONE. WIDENED: NONE.
    NEW parameters: none. CHANGED elaboration values: the four in the parameters
    section, each zero-delta when `usingRVV` is false.
    NEW instance: `vec` (VecPipeline), `Option`-wrapped, connected by
    `vec_pipeline_io`.
    CHANGED instance CLASS: `dispatcher` is `CompactingDispatcher` when `usingRVV`
    and `BasicDispatcher` otherwise (D2). No new instance, no new argument, and
    `dispatch.scala` unedited.
    NEW SEAM CONNECTIONS the native dispatch wiring needs: the three `IQ_V_*`
    entries of `dispatcher.io.dis_uops` — per-lane `valid` into
    `v.io.dis_vec_valids(q)(w)`, per-lane `ready` from `v.io.dis_vec_ready(q)(w)`
    — plus the `require(ip.dispatchWidth == coreWidth)` on those entries. The
    dispatch PAYLOAD does not cross (part 4).
    NEWLY CONNECTED existing ports (declared by other nodes' deltas):
    `decode_units(w).io.vec`, `rob.io.vec_clr_bsy`, `rob.io.vec_clr_unsafe`,
    `rob.io.vec_rob_flags`, `rob.io.com_vxsat`, `io.lsu.lsu_vec`,
    `io.lsu.vec_lsu_empty`, `fp_pipeline.io.vec_frf_read_req`/`_rsp` and its
    dedicated vector FP writeback input (D7),
    `csr.io.vector.get.{set_vconfig, set_vstart, set_vxsat, set_vs_dirty}`.

    Explicitly NOT added, and a reviewer should reject any of these on sight:
      - a second vector instance of any kind, or any NEW `Module(...)` other than
        `VecPipeline` — the pre-existing `dispatcher`'s class selection is a
        changed argument to an existing `Module(...)`, not an added instance;
      - `dispatcher.io.dis_uops(i)(w).ready := true.B` on any `IQ_V_*` entry, or
        any other tie-off of a vector dispatch lane: that is the alternative D2
        REJECTED;
      - `Module(new CompactingDispatcher)` UNCONDITIONALLY. A non-vector build
        must still get `BasicDispatcher`, or gate (f) breaks on a change that D1's
        enumerated exception does not cover;
      - a vector `Wire`, `Reg`, `Vec` or `Arbiter` in `core.scala` — the only
        added selection logic is the two-input `IsOlder` merge into
        `rob.io.lxcpt`, and the youngest-VL-producer lane select of part 10;
      - any per-field patching of vector `MicroOp` fields into `dis_uops`
        (`dis_uops(w).pvdest := ...` and its fourteen siblings) — the M1
        braiding, and a combinational loop as well;
      - `dec_uops`, `dec_valids` or `dec_fire` wired to `ren2_uops`,
        `ren2_mask` or `dis_fire`, in either direction;
      - core's own `dis_ready` or `dec_ready` driven from the seam's `dis_ready`;
      - a `dst_rtype === RT_FIX` qualifier on the `vset_resp` tap, or any
        change to that qualifier at line 965 or 975;
      - a vector term inside `dec_hazards`, `dis_hazards`,
        `wait_for_empty_pipeline`, or a second fence/drain handshake anywhere;
      - an arbiter, OR-reduction or `Mux1H` on `vec_clr_bsy`, `vec_rob_flags` or
        `vl_wakeup`; a `RegNext` on any completion, wakeup or clear path;
      - a HAND-ADDED `rob.io.wb_resps` entry, or a Rob constructor argument that
        follows the widened `numIrfWritePorts` — the one new entry a vector config
        gets is the D7 FP slot, and it arrives through the EXISTING FP loop with
        no new line of its own;
      - a `busy`, `vec_busy`, `vec_active` or `fu_ready` input from the vector
        subsystem, or any vector term in an issue unit's `fu_types`;
      - a vector `iq_type` decode, queue-ID field or routing table here;
      - `usingVector` used as the gate anywhere in this file (it appears only as
        the subject of the added `require`);
      - a vector printf outside the `vec_trace_en` / `VecTrace` guard, or any
        change to the `boom_timeout` assertion.
<|end_edit_scope|>

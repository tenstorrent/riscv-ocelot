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
  DecodeUnit — DELTA SPEC. Not a description of BOOM's decoder: it describes only
  the change Caracal applies to `class DecodeUnit` / `class DecodeUnitIo` in
  src/main/scala/v4/exu/decode.scala, which is hand-written baseline BOOM v4 and
  stays in place.

  hierarchy.yaml: kind: module, mode: edit_existing,
  target src/main/scala/v4/exu/decode.scala, group host.
  depends_on MicroOp, ScalarOpConstants. Instantiates nothing.
  No `output:` — the pre-existing file is the artifact.
  Budget (plan v2 section 11): ~40 added lines. One of the smallest deltas in the
  map, and four of its fifteen requirements are discharged by changing NOTHING
  (logic part 6).

  Everything else in the file is unchanged and is not restated here — the decode
  tables, `class CtrlSigs`, `class BranchDecode`, `class BranchMaskGenerationLogic`
  and the whole scalar body. Anything not mentioned keeps its expression exactly;
  the edit scope section enumerates what is most at risk.

  ===> HIGHEST-RISK OBLIGATION HERE, AND IT IS NOT ABOUT VECTOR UOPS.
       `MicroOp.vconfig` must be written on EVERY uop that can allocate a
       `br_tag`, not only on `is_vec` uops. VConfigUnit sources its per-`br_tag`
       VCFG snapshot from `ren2_uops(w).vconfig`, and `allocate_brtag` is
       `(is_br && !is_sfb) || is_jalr` — nearly every branch tag in the machine
       is allocated by a SCALAR branch. Populate `vconfig` only for vector uops
       and every snapshot taken on a scalar branch is garbage, so the mirror is
       restored to nonsense on the mispredict of an ordinary `beq`. This delta
       therefore makes the vector merge UNCONDITIONAL over lanes (logic part 4)
       instead of a `Mux(v_legal, ...)`, which is the shape that has the bug.

  ===> `vsetvl` IS MARKED BOTH `is_unique` AND `flush_on_commit`, HERE, FROM THE
       INSTRUCTION WORD. `is_unique` alone does not order the speculative vtype
       mirror against younger decode: `core.scala:739-740` gates only the unique
       uop's OWN dispatch and says nothing about younger uops, which decode one
       cycle later against a stale mirror and hand the mapper the wrong EMUL.

  Governing spec anchors: frontend.rst `vector-rvv-decode` ("VSET Special
  Handling"), `vector-csr-explicit`, `vector-csr-ownership`, `frontend-stages`;
  loadstore.rst `fences`; overview.rst `caracal-pipeline`; glossary.rst
  `glossary-terms`.
*/

<|begin_module|>

  <|begin_parameters|>
  No new constructor parameter and no new elaboration knob. Two existing values
  decide what the delta must do.

  `usingRVV` — the Scala `Boolean` from `BoomCoreParams`, NOT a hardware `Bool`.
  Every added line is inside `if (usingRVV)` or is a term multiplied by a
  Scala-constant false, so with vectors off the elaborated module is bit-identical
  to pre-Caracal BOOM v4: the new ports are ABSENT (an `Option` that is `None`),
  the recognition terms constant-fold away, and the decode table's
  illegal-instruction behaviour for RVV opcodes is exactly what it is today. Do
  not gate on rocket's `usingVector` — that enables the architectural vector CSRs
  in rocket's `CSRFile` and is a different gate with a different owner.

  `IQ_SZ` — raised 4 to 7 by the ScalarOpConstants delta, unconditionally (that
  trait has no `Parameters` to gate on). Not free for this file: see logic part 3.

  //@req-spec-decode.a5
  `coreWidth` is unchanged and must not be touched. `core.scala:120` instantiates
  one `DecodeUnit` per decode lane, and every port added below is a scalar port on
  that per-lane module, so parameterizable super-scalar decode is maintained by
  construction — widening `coreWidth` replicates the new ports with the units. NO
  cross-lane logic may be added here. The machine's only cross-lane vector decode
  logic is VConfigUnit's nearest-preceding-`vset` prefix scan, inside VecPipeline;
  a lane consumes its result and never computes it.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are unchanged: implicit Chisel `clock`/`reset`, posedge,
  ACTIVE-HIGH SYNCHRONOUS, one core clock domain. `DecodeUnit` holds no register
  and this delta adds none, so neither reaches the added logic; they serve only
  the added assertion.

  ---- ONE new Option-wrapped sub-bundle on `class DecodeUnitIo` ----

  Add a single field `vec`, present only when `usingRVV`, so a vectors-off build's
  `DecodeUnitIo` is unchanged field-for-field. Three scalar members, per-lane
  because the module is per-lane:

  - `uop_to_vdec`   — `Output(new MicroOp())`. This lane's BASELINE decode result,
                      before any vector field is merged. BoomCore wires it to
                      `vec_pipeline_io.dec_uops_in(w)`.
  - `uop_from_vdec` — `Input(new MicroOp())`. VecDecode's result: `uop_to_vdec`
                      with the vector fields overridden and every other field
                      passed through. From `vec_pipeline_io.dec_uops_out(w)`.
  - `illegal`       — `Input(Bool())`. Raise illegal-instruction for a
                      VECTOR-SPECIFIC reason the scalar tables cannot see: a
                      poisoned mirror reaching a vtype-dependent op, a
                      vtype-derived EMUL above `maxMembers`, the reserved keep-VL
                      encoding. From `vec_pipeline_io.dec_vec_illegal(w)`.

  ===> THERE IS DELIBERATELY NO `dec_vec_legal` INPUT; reject one. "Is this
       encoding in the vector opcode space" is answered LOCALLY from `inst` (logic
       part 1), not imported: it keeps the illegal-instruction gate a function of
       `inst` alone so a mis-wired seam cannot defeat it, it adds no signal to an
       already-wide interface, and it is the COARSE question whose answer is a
       fixed opcode property — while `illegal` carries the FINE question, whose
       answer depends on the speculative mirror. Different lifetimes, different
       owners.

  ---- One existing port newly READ, not widened ----

  `io.csr_decode` is `Flipped(new rocket.CSRDecodeIO)` and already carries
  `vector_illegal` (CSR.scala:246), driven as
  `io.status.vs === 0.U || reg_mstatus.v && reg_vsstatus.vs === 0.U ||
  !usingVector.B`. Baseline BOOM never reads it; the delta does. No port is added,
  widened or re-flipped for this.

  Nothing else on `DecodeUnitIo` changes: `enq`, `deq`, `status`, `fcsr_rm`,
  `interrupt`, `interrupt_cause` keep their exact declarations. In particular NO
  `vec_lsu_empty`, `fencei_rdy` or drain-handshake port appears — logic part 5
  says why one would be a design error rather than merely redundant.
  <|end_ports|>

  <|begin_logic|>
  All of the below is inside the `usingRVV` gate. The delta is one recognition
  term, two terms appended to two existing expressions, six field defaults, and
  one output merge.

  //@req-spec-core.c1
  //@req-spec-core.c2
  //@req-spec-decode.a1
  `DecodeUnit` STAYS a single-cycle purely combinational function emitting EXACTLY
  ONE `MicroOp` per instruction; the delta adds no register, no state and no
  second output beat. The merge in part 4 is a FIELD OVERRIDE on the one outgoing
  uop, not a second uop, so no configuration of this file can emit more uops than
  it consumes: no cracking and no micro-op expansion happens here, and element
  expansion into nOP.v happens only inside the vector LS AGEN. The frontend
  therefore keeps its two stages — instruction fetch and a ONE-cycle decoder —
  because the vector decoders feeding this merge are themselves combinational and
  sit in the same decode cycle. Nothing here may become a pipeline stage.

  ---- 1. `v_legal`: recognizing the vector opcode space ----

  Add one local predicate, computed from `inst` alone, as three positive matches:

    OP-V            `inst(6,0) === "b1010111".U`
    vector LOAD-FP  `inst(6,0) === "b0000111".U` and
                    `inst(14,12).isOneOf(0.U, 5.U, 6.U, 7.U)`
    vector STORE-FP `inst(6,0) === "b0100111".U` and the same width set

  `v_legal` is that predicate ANDed with `!io.csr_decode.vector_illegal`.

  // ===> MATCH THE LOAD/STORE WIDTHS POSITIVELY. The tempting form is "LOAD-FP
  // with a width other than FLW's 010 or FLD's 011", and it is wrong: width 001
  // is FLH and 100 is FLQ, neither in `F_table`, so both are illegal today. The
  // negative form silently admits them as vector, stops them trapping, and routes
  // an FLH encoding into VLSDecode. RVV's data widths are exactly the four above.

  // ===> THE `vector_illegal` TERM IS LOAD-BEARING, NOT DEFENSIVE. `mstatus.VS`
  // and the VS=Off illegal-instruction gate are rocket's (frontend.rst
  // `vector-csr-ownership`). Without this term a vector instruction executed with
  // VS=Off is admitted by the gate below and never traps — an architectural hole
  // no vector test can find, because vector tests enable VS first.

  ---- 2. The two terms added to `id_illegal_insn` ----

  An RVV encoding matches no row of any table in `object DecodeTables`, so
  `cs_legal` is false for it and `!cs_legal` — the first term of the existing
  disjunction — is exactly what traps vector instructions today. Two changes, no
  others: qualify the FIRST term only, as `!cs_legal && !v_legal`, and append
  `|| io.vec.get.illegal`.

  The other five terms are untouched, and qualifying them would change scalar
  behaviour: `cs.fp_val` cannot fire on an RVV encoding (no row matched, so
  `fp_val` takes `decode_default`'s `N`), and `cs.is_amo`, `uop.is_rocc`, `csr_en`
  and `sfence || system_insn` are all false for it.

  Folding into `id_illegal_insn` rather than adding a sixth `checkExceptions` row
  keeps the cause and the PRIORITY identical: interrupt, `bp_debug_if`,
  `bp_xcpt_if`, `xcpt_pf_if`, `xcpt_ae_if`, illegal. A vector-specific trap IS an
  illegal-instruction trap and gets no new cause code.

  // ===> NO COMBINATIONAL LOOP, BUT IT IS CONDITIONAL ON A NEIGHBOUR.
  // `uop.exception` depends on `illegal`, which comes from VecDecode, which is fed
  // `uop_to_vdec` — a bundle that CONTAINS `exception`. That is a loop if and only
  // if a vector decoder derives its illegal output from `uop_in.exception`. None
  // does (VDecode: `vtype.vill` and the EMUL bound; VsetDecode: the keep-VL VLMAX
  // comparison; VLSDecode: an encoding check), so the vector decoders must NOT
  // read `uop_in.exception`/`exc_cause`. Recorded here, host-side.

  ---- 3. Vector-field defaults: this file is the LAST-RESORT WRITER ----

  //@req-spec-core.e6
  Decode sets the `iq_type` routing bits and dispatch routes purely on them, with
  no vector-specific logic of its own. Baseline `DecodeUnit` writes
  `iq_type(IQ_UNQ)`, `(IQ_ALU)`, `(IQ_MEM)` and `(IQ_FP)` individually — four
  assignments that covered a `Vec(IQ_SZ, Bool())` completely at `IQ_SZ = 4` and no
  longer do at 7. Drive `iq_type(IQ_V_LOAD)`, `iq_type(IQ_V_STORE)` and
  `iq_type(IQ_V_ALU)` to `false.B` here; VecDecode overrides them through the
  merge for a recognized RVV lane, routing loads and stores on the first two,
  arithmetic on the third, and a segmented access on TWO bits (two issue slots,
  one ROB entry). The default is all this file owns.

  // ===> NOT BELT-AND-BRACES: THE UNCOVERED POSITIONS ARE `DontCare`. The body
  // begins `uop := io.enq.uop`, and `io.enq.uop` comes from
  // `dec_fbundle.uops(w).bits`, whose producer does `f2_fetch_bundle := DontCare`
  // (frontend.scala:480). Without these assignments a SCALAR uop carries three
  // don't-care queue-routing bits into dispatch, and whatever FIRRTL's
  // invalidation folds them to decides whether ordinary integer code is dispatched
  // into the vector load queue. Same argument for `is_vec`, `is_shared` and
  // `is_vl_producer` — drive all three `false.B`. `is_vl_producer` alone would
  // allocate a `pvl` and write the VL register file for a scalar uop.

  The other added `MicroOp` fields — the `lv*` specifiers, the `pv*` groups,
  `v_emul`/`v_eew`/`v_seg_nf`/`v_idx_eew`, the `v_mop`/`v_is_*` access class and
  the `v_split_*` cursor — get NO default and are documented don't-care while
  `is_vec` is clear: zeroing them is half the line budget spent on fields whose
  every reader is already gated on `is_vec`.

  ---- 4. The merge, and why it is unconditional ----

  Replace the final `io.deq.uop := uop` with, under `usingRVV`, a drive from
  `io.vec.get.uop_from_vdec`, and export `uop` on `io.vec.get.uop_to_vdec`. The
  merge is a WHOLE-BUNDLE assignment applied on EVERY LANE, vector or not — never
  `Mux(v_legal, uop_from_vdec, uop)`.

  That is sound because VecDecode is a pass-through with overrides: each of its
  four children gates its field writes on its own recognition predicate, so on a
  scalar lane the only field differing from `uop_to_vdec` is `vconfig`, which
  VConfigUnit drives for every lane from its per-lane prefix select regardless of
  what the lane holds.

  ===> AND `vconfig` ON EVERY LANE IS EXACTLY WHY THE MERGE MUST BE
       UNCONDITIONAL. It is the source of the per-`br_tag` VCFG snapshot:
       VConfigUnit captures `ren_br_vconfig(w+1) = ren2_uops(w).vconfig` on the
       same allocation event that snapshots the rename map tables, and branch tags
       are allocated by `allocate_brtag = (is_br && !is_sfb) || is_jalr` — a
       property of SCALAR control flow. A vector kernel's loop-back `bne` is a
       scalar branch, and it is the tag that matters. Gate the merge on `v_legal`
       and every such snapshot holds a don't-care `vtype`; the mirror is restored
       to garbage on an ordinary branch mispredict, every surviving younger vector
       op derives the wrong EMUL, and the mapper allocates a mis-sized PRN group —
       silent corruption on a path with no vector instruction near the failure.
       Settled here in favour of the unconditional merge.

  Assert that, on a lane where the local RVV opcode predicate is false,
  `uop_from_vdec` equals `uop` in every field EXCEPT `vconfig`. This project has no
  unit tests — validation is end-to-end VCS plus Whisper cosim — so this assertion
  is what turns "a vector decoder wrote a field on a lane it does not own" from a
  scalar miscompare hundreds of cycles downstream into a named failure in the cycle
  it happened. It is the price of the unconditional merge, and worth paying.

  //@req-spec-core.e5
  No scalar decoder is modified. `object DecodeTables` keeps all seven tables and
  `decode_default` unchanged and NO RVV row is added to any of them — RVV decode
  is VecDecode's, and a host-side RVV table would be a second thing to keep in
  agreement with the VPU's own decoder. `class CtrlSigs` gains no field and its
  `decode` method is untouched; `class BranchDecode` and
  `class BranchMaskGenerationLogic` are not touched at all. Every scalar
  instruction's control signals are bit-identical before and after, in a vector
  build as well as a non-vector one, because `v_legal` is false for every encoding
  any table row matches.

  //@req-spec-core.f8
  `uop.br_type` keeps its eight-way reduction over BEQ/BNE/BGE/BGEU/BLT/BLTU/
  JAL/JALR and gains no vector term. `is_br`, `is_jalr` and `starts_unsafe` are
  `def`s on `MicroOp` (`micro-op.scala:117,119,164` —
  `starts_unsafe = uses_ldq || (uses_stq && !is_fence) || is_br || is_jalr`) whose
  bodies the MicroOp delta freezes, so a uop with `is_br` or `is_jalr` set still
  sets `starts_unsafe` and the PNR still never sweeps past an unresolved branch. A
  vector uop sets no `br_type`, so it is never `is_br`/`is_jalr`; ROB safety for
  vector ops is the Rob delta's business and no term for it appears here.

  ---- 5. `vsetvl`: both bits, decided from the instruction word ----

  //@req-spec-decode.e3
  //@req-spec-decode.e4
  Recognize `vsetvl` locally as one comparison against rocket's
  `Instructions.VSETVL` pattern (already in scope via the file's
  `import freechips.rocketchip.rocket.Instructions._`) and add that predicate to
  BOTH existing expressions, leaving their right-hand sides otherwise verbatim:

    `uop.is_unique       := cs.inst_unique || is_vsetvl`
    `uop.flush_on_commit := cs.flush_on_commit ||
                            (csr_en && !csr_ren && io.csr_decode.write_flush) ||
                            is_vsetvl`

  `is_unique` makes the `vsetvl` itself dispatch only once everything older has
  retired, so it computes its VTYPE against a settled machine. `flush_on_commit`
  makes everything YOUNGER observe the result: those uops decoded a cycle behind
  it, against a mirror a `vsetvl` cannot update at decode because its VTYPE lives
  in `rs2` and does not exist yet. Refetching them is what makes VConfigUnit's
  committed-shadow restore the recovery path, and why no execute-time mirror write
  exists anywhere in the design.

  // ===> THE PROOF THAT `is_unique` ALONE IS INSUFFICIENT IS IN THE BASELINE.
  // `core.scala:739-740` reads
  //   wait_for_empty_pipeline(w) = (dis_uops(w).is_unique || !enableOOO) &&
  //                                (!rob.io.empty || !io.lsu.fencei_rdy ||
  //                                 dis_prior_slot_valid(w))
  // — a condition on ONE uop's own DISPATCH. Nothing in it, and nothing else in
  // BOOM, holds back the DECODE of the next bundle, and the vtype mirror is
  // written at decode, a stage earlier, so it is not ordered by `is_unique` at
  // all. An earlier draft of frontend.rst claimed it was. VsetDecode sets the same
  // pair from the other side of the merge; that duplication is intentional, so the
  // property holds from the instruction word alone and does not depend on
  // VecPipeline being present, elaborated or correctly wired.

  //@req-spec-memord.f17
  //@req-spec-memord.f18
  `is_unique` is ALSO how a `vsetvl` and the explicit vector-CSR accesses wait for
  in-flight VECTOR memory to settle before dispatching. No new handshake, port or
  drain protocol is added here for it. The mechanism is the LSU delta's one-term
  extension `io.core.fencei_rdy := !stq_nonempty && io.dmem.ordered &&
  vec_lsu_empty` (lsu.scala:439), which `wait_for_empty_pipeline` above already
  consumes: an `is_unique` uop cannot dispatch until the ROB is empty AND
  `fencei_rdy`, so folding `vec_lsu_empty` in makes every `is_unique` uop wait for
  the four vector address queues, both store data queues and the Load Coalescing
  Buffer to drain. This decoder's whole contribution is asserting `is_unique` on
  the right instructions.

  // A head-side handshake is worse than redundant, it deadlocks: a YOUNGER
  // vector store fills `st_SSI_*_Q`, whose entries free only at commit-drain,
  // which cannot happen while the fence sits at the ROB head. The dispatch-side
  // wait cannot deadlock — at dispatch the ROB is already empty, so every older
  // vector store has committed and drains unconditionally, and nothing younger
  // exists yet. Do not add a `vec_lsu_empty` port here.

  ---- 6. What this delta does NOT add, and the four reqs that discharges ----

  //@req-spec-decode.f1
  //@req-spec-decode.f2
  Every explicit vector-CSR access — any `csrr`/`csrw`/`csrrw`-family instruction
  targeting `vstart`, `vxrm`, `vxsat`, `vcsr`, `vl`, `vtype` or `vlenb` — is
  ALREADY decoded `is_unique`, and every such write already `flush_on_commit`, in
  unmodified baseline BOOM v4: the rows `CSRRW`, `CSRRS`, `CSRRC`, `CSRRWI`,
  `CSRRSI`, `CSRRCI` (decode.scala:149-155) each carry `inst_unique = Y` and
  `flush_on_commit = Y`, and `uop.flush_on_commit` ORs in
  `csr_en && !csr_ren && io.csr_decode.write_flush` on top. Rocket's `CSRFile` puts
  all seven vector CSR addresses in `read_mapping` under `usingVector`, returns
  `write_flush` true for every one (each `addr | (PRV.M << modeLSB)` falls outside
  the `mscratch`..`mtval` exemption window), and raises `read_illegal` while
  `vector_illegal` holds. The required decode therefore comes from machinery that
  already exists: this delta ADDS NOTHING here, and what discharges the two
  requirements is the must-not-regress list — those six rows, those two columns and
  that expression must survive the edit unchanged.

  // For the reader checking spec against file: frontend.rst
  // `vector-csr-explicit` argues as though the CSR rows were `flush_on_commit = N`
  // and the `write_flush` term supplied it for writes. In THIS baseline the row
  // already forces both bits, so a vector-CSR READ is also flush_on_commit — a
  // superset of the requirement, costing a refetch on a rare instruction. Do not
  // "optimize" by clearing the row bit: that would make the property depend on
  // another repo asserting `write_flush`, for a saving nobody measured.

  //@req-spec-memord.f6
  //@req-spec-memord.f7
  Likewise `fence`, `fence.i` and `sfence.vma` are all already decoded
  `is_unique`, and `FENCE` and `SFENCE_VMA` also `flush_on_commit`, in the
  unmodified table: `SFENCE_VMA` (line 157), `FENCE_I` (166) and `FENCE` (167) all
  carry `Y, Y` in those two columns. Caracal orders vector memory against all three
  WITHOUT adding a mechanism, exactly as for `vsetvl` above — the `is_unique`
  decode is untouched and the vector path is folded into `fencei_rdy`. Nothing is
  added for these two either; the must-not-regress list discharges them.

  // Observation, not a defect: the spec's "`FENCE` and `SFENCE_VMA` are also
  // `flush_on_commit`" reads as though `FENCE_I` were not, but row 166 sets it
  // too. The requirement demands the bit on FENCE and SFENCE_VMA and forbids it
  // nowhere, so the baseline satisfies it; the extra bit on `fence.i` is
  // pre-existing BOOM behaviour and must not be "corrected" here.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
`DecodeUnit` is purely combinational, one cycle, `coreWidth` copies, in the DECODE
critical path, and the delta must not change any of that: no pipeline stage, no
register, no `RegNext`. Decode stays the one-cycle stage the frontend's two-stage
structure assumes, and rename — already the design's top timing risk, with up to
`coreWidth * maxMembers` PRN allocations in a cycle — must see `v_emul` in the
cycle after decode.

Added host-side depth is small and bounded: three opcode equalities plus a
four-way width membership test for `v_legal`, one bit-pattern compare for
`vsetvl`, and two extra OR terms on already-wide disjunctions.

The real added depth is NOT in this file. `io.deq.uop` now comes from
`uop_from_vdec`, so the decode cycle's critical path runs through VecDecode's
children, including VConfigUnit's `coreWidth`-deep prefix scan over
`VtypeTable.decode`. That is the path to watch; if decode timing fails the fix
belongs there or in the mapper, never in a register added here.

With `usingRVV = false` none of the above is elaborated — no port, no comparison,
no merge — and the emitted RTL is bit-identical to pre-Caracal BOOM v4, which gate
(f) checks by diffing it.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the delta writes fields it adds (`is_vec`, `is_shared`,
`is_vl_producer`) and carries `vconfig` through the merge. It also depends on that
delta FREEZING the `is_br` / `is_jalr` / `starts_unsafe` / `allocate_brtag` method
bodies, which is what makes the `starts_unsafe` claim a no-change claim.

ScalarOpConstants — for `IQ_V_LOAD`, `IQ_V_STORE`, `IQ_V_ALU` and the raised
`IQ_SZ`, which is precisely why the three new `iq_type` positions need a default
here. `RT_VEC` and the widened `*_rtype` encodings are the vector decoders' to use.

VecDecode — not a `depends_on:` edge (it is inside VecPipeline, across
`vec_pipeline_io`) but the counterparty of the merge, and through it this delta is
coupled to VDecode, VLSDecode, VsetDecode and VConfigUnit: VConfigUnit supplies the
`vconfig` propagated on every lane, VsetDecode sets the same `vsetvl` bit pair from
the other side, and all four contribute to `illegal`.

BoomCore — owns the wiring: `dec_uops_in(w)` from `uop_to_vdec`, `uop_from_vdec`
from `dec_uops_out(w)`, `illegal` from `dec_vec_illegal(w)`, and
`dec_uops(w) := decode_units(w).io.deq.uop` (core.scala:534) unchanged.

Lsu — counterparty of the memory-ordering requirements: this file asserts
`is_unique`, the LSU delta folds `vec_lsu_empty` into `fencei_rdy`, and
`core.scala:739-740` joins them. No signal passes directly between the two.

Instantiates nothing, today or after the delta.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File   src/main/scala/v4/exu/decode.scala
    Classes `class DecodeUnitIo(implicit p: Parameters) extends BoomBundle` and
            `class DecodeUnit(implicit p: Parameters) extends BoomModule`
    (package boom.v4.exu). Hand-written baseline BOOM v4.

  In scope:
    - `class DecodeUnitIo`: ONE `Option`-wrapped `vec` sub-bundle, conditional on
      `usingRVV`, with members `uop_to_vdec` (Output MicroOp), `uop_from_vdec`
      (Input MicroOp), `illegal` (Input Bool).
    - `class DecodeUnit`: the local RVV opcode-space predicate, `v_legal`, and the
      `vsetvl` predicate.
    - `id_illegal_insn`: qualifying its FIRST term with `&& !v_legal` and
      appending `|| io.vec.get.illegal`. No other term of it, and no row of the
      `checkExceptions` list, may change.
    - `uop.is_unique` and `uop.flush_on_commit`: appending one `is_vsetvl` term to
      each; existing right-hand sides stay verbatim.
    - Default assignments for `uop.iq_type(IQ_V_LOAD)`, `uop.iq_type(IQ_V_STORE)`,
      `uop.iq_type(IQ_V_ALU)`, `uop.is_vec`, `uop.is_shared`,
      `uop.is_vl_producer`.
    - The final `io.deq.uop := uop`: exporting `uop` on `uop_to_vdec` and driving
      `io.deq.uop` from `uop_from_vdec` when `usingRVV`.
    - One added assertion (the scalar-lane pass-through check).
    - Reading `io.csr_decode.vector_illegal`.

  Must not regress:
    - `object DecodeTables` in its entirety: `decode_default` and all seven tables
      (`X_table`, `X32_table`, `X64_table`, `F_table`, `FDivSqrt_table`, `B_table`,
      `RoCC_table`), every row and column, in order. NO RVV row is added to any
      table. In particular the `inst_unique` and `flush_on_commit` columns of
      `CSRRW`/`CSRRS`/`CSRRC`/`CSRRWI`/`CSRRSI`/`CSRRCI` (149-155), `SFENCE_VMA`
      (157) and `FENCE_I`/`FENCE` (166-167) stay `Y, Y` — four of this node's
      fifteen requirements are discharged by their being unchanged.
    - `class CtrlSigs` gains no field; its `decode` method, `sigs` sequence and
      `fp.vec := false.B` line are unchanged. `class BranchDecode`,
      `class BranchDecodeSignals` and `class BranchMaskGenerationLogic` are not
      touched at all.
    - `csr_en`, `csr_ren`, `system_insn`, `sfence`, `illegal_rm`, `cs_legal` keep
      their exact expressions, and `io.csr_decode.inst := inst` is unchanged.
    - The `checkExceptions` list keeps its six rows in order, so interrupt beats
      breakpoint beats fetch fault beats illegal-instruction, and a vector trap
      still reports `Causes.illegal_instruction`.
    - `uop.br_type`, `op1_sel`, `op2_sel`, `csr_cmd`, `mem_size`, `mem_cmd`,
      `is_fence`, `is_fencei`, `is_sfence`, `is_eret`, `is_sys_pc2epc`, `is_rocc`,
      `is_mov`, the immediate block (`di24_20`, `imm_packed`, `short_imm`,
      `imm_rename`, `imm_sel`, `pimm`), the SFB `is_sfb_shadow` rewrites and
      `ldst_is_rs1` are bit-identical.
    - The four existing `uop.iq_type(...)` assignments keep their exact right-hand
      sides; the three new positions are ADDITIONAL assignments. `uop.dst_rtype`,
      `uop.lrs1_rtype`, `uop.lrs2_rtype` keep their `RT_ZERO`-on-x0 muxes — this
      delta introduces `RT_VEC` nowhere, the vector decoders do via the merge.
    - With `usingRVV = false`: `DecodeUnitIo` has today's field set and widths,
      `io.deq.uop := uop` is the emitted assignment, and the module is
      bit-identical to the current file. The decode table's illegal-instruction
      behaviour for RVV opcodes is unchanged in that build — an RVV encoding still
      traps via `!cs_legal`.
    - Copyright header, `// scalastyle:off/on` markers, comment style, table column
      alignment and declaration order are preserved.

  Interface delta:
    NEW on `class DecodeUnitIo`, only when `usingRVV`:
      vec.uop_to_vdec   : Output(new MicroOp())
      vec.uop_from_vdec : Input(new MicroOp())
      vec.illegal       : Input(Bool())
    NEWLY READ (no declaration change): io.csr_decode.vector_illegal
    WIDENED: nothing.  NEW PARAMETERS: none.

    Explicitly NOT added; reject them if they appear:
      - a `dec_vec_legal` / `vec_legal` INPUT (the opcode-space question is
        answered locally from `inst`);
      - a `vec_lsu_empty`, `fencei_rdy`, `drain_done` or any memory-drain
        handshake port (the ordering is `is_unique` plus the LSU's `fencei_rdy`
        fold, at dispatch; a head-side handshake deadlocks);
      - a `vtype`/`vl`/`vstart`/`vxrm` input or any vtype mirror register (VCFG
        state is VConfigUnit's; `vconfig` arrives on `uop_from_vdec`);
      - any RVV row in any `DecodeTables` table, or a `vec`/`is_vec` field on
        `class CtrlSigs`;
      - a new `checkExceptions` row or a new exception cause for vector traps;
      - `Mux(v_legal, uop_from_vdec, uop)` as the output merge — the merge is
        unconditional over lanes, because `vconfig` must reach every uop that can
        allocate a `br_tag`;
      - any register, pipeline stage or `RegNext` anywhere in `DecodeUnit`.
<|end_edit_scope|>

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
  VsetDecode — the per-lane decoder that splits the three `vset` encodings into
  the three different pipeline paths they take, and drives the uOP fields that
  commit each one to its path.
*/

hierarchy.yaml: kind: module, mode: new,
output src/main/scala/v4/vec/generated/decode/VsetDecode.scala,
package boom.v4.vec.generated.decode. depends_on MicroOp, VecTrace,
VtypeTable. Instantiated ONCE, as `vset`, inside VecDecode; its ports are
`coreWidth`-wide vectors, one element per decode lane.

THE SPLIT IS THE SUBSTANCE OF THIS FILE:

  vsetivli  both VTYPE and AVL immediate. With `rd == x0` the uOP is FRONT-END
            ONLY: no issue queue, no EU. VL is computed at decode; the VL-RF
            write happens in the RENAME cycle, where `pvl` exists. No busy bit,
            `pvl` born ready, ROB entry dispatched non-busy.
  vsetvli   VTYPE immediate, AVL from `rs1` -> integer ALU EU.
  vsetvl    VTYPE from `rs2` AND AVL from `rs1` -> integer ALU EU, plus
            `is_unique` AND `flush_on_commit`.

===> `vsetvli rd = x0, rs1 = x0` IS THE RESERVED KEEP-VL FORM, NOT `AVL = 0`.
     Reading the zero `rs1` field as an AVL of zero sets VL to 0 and silently
     turns every following vector instruction into a no-op — a failure that
     looks like a masking or tail-policy bug, miles from its cause.

===> A REGISTER-SOURCED `vset` HAS TWO DESTINATIONS IN TWO RENAME SPACES:
     `pdst` in the integer RF and `pvl` in the VL RF. That is what
     `is_vl_producer` exists for; it is ORTHOGONAL to `dst_rtype`, which reads
     `RT_ZERO` when `rd == x0` while the VL RF is still written.

Governing spec anchors: frontend.rst `vector-rvv-decode` ("VSET Special
Handling"), `vset-dual-dest`, `vl-delivery`. Plan v2 ground rules 1 (usingRVV),
9 (rocket owns the architectural vector CSRs), 10 (reuse the existing wakeup
networks), 11 (guarded tracing).

<|begin_module|>

  <|begin_parameters|>
  No constructor parameters. Everything comes implicitly through Chisel's
  `Parameters`: `coreWidth` and `lregSz` from `HasBoomCoreParameters`, `vecVLSz`
  from VectorParams, `xLen` for the width of rocket's `VType`.

  Elaborated only when `usingRVV` is true, and it needs no `usingRVV` term of its
  own: it lives under VecPipeline, whose single `core.scala` instantiation is
  already gated, so a vectors-off build contains no instance at all and emits RTL
  bit-identical to pre-Caracal BOOM v4 (gate (f)). Absent, not tied off.

  //@req-spec-decode.c13
  No parameter selects a wakeup network and none here could add one. A
  register-sourced `vset` CONSUMES the existing integer network for `rs1`/`rs2`
  and PRODUCES on the VL network for `pvl`. No VCFG wakeup network exists anywhere
  in the design and nothing in this file may imply one; the only two new networks
  in v2 are VL and VECTOR (group-done), neither created here.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel and hierarchy.yaml default: posedge
  `core_clk`, ACTIVE-HIGH SYNCHRONOUS `core_reset`. Both are implicit and unused —
  this module is purely combinational — and exist only because a Chisel `Module`
  carries them and because trace statements are `!reset`-gated.

  Every port is `Vec(coreWidth, ...)`, index 0 = oldest lane in program order. The
  names are the ones VConfigUnit's port list already uses for the same wires, since
  that unit declares them as inputs it takes "from the sibling decoders (VsetDecode
  for the `vset` shapes)". One wire, one name on both sides.

  Inputs:

  - `dec_valid`   — this lane holds a valid instruction. This module qualifies on
                    validity; the MIRROR is updated by VConfigUnit on `dec_fire`,
                    which is that unit's business and is not replicated here.
  - `inst`        — 32 bits, the raw instruction word. All field extraction is by
                    bit position on it, matching rocket-chip `RocketCore`'s
                    `ex_new_vtype`/`ex_avl` extraction exactly.
  - `uop_in`      — the `MicroOp` the baseline `DecodeUnit` produced. For an OP-V
                    opcode the baseline tables have no entry, so its scalar control
                    fields are meaningless; this module drives a complete override
                    of the fields it owns (enumerated in the logic section) and
                    passes every other field through untouched.
  - `dec_vconfig` — rocket `VType` from VConfigUnit: this lane's prefix-select
                    result, which for a `vset` lane is the SELF-INCLUSIVE value,
                    i.e. its own new vtype.
  - `prev_vtype`  — rocket `VType` from VConfigUnit: the SELF-EXCLUSIVE value, the
                    vtype in effect BEFORE this lane's own instruction. Read for
                    one thing only, the keep-VL VLMAX comparison.

  Outputs:

  - `dec_is_vset`, plus `is_vsetivli` / `is_vsetvli` / `is_vsetvl` as three
    mutually exclusive predicates rather than an encoded field, so a consumer
    cannot mis-decode the shape. Asserted only with `dec_valid`.
  - `dec_vtype_is_imm` — this `vset`'s vtype is an immediate. False for `vsetvl`.
  - `dec_vtype_imm`    — its encoded vtype bits, RAW: legality is VtypeTable's
    answer and VConfigUnit is the caller that needs it, so nothing is pre-decoded.
  - `dec_avl_imm`      — the 5-bit `uimm` AVL field of a `vsetivli`.
  - `keep_vl_illegal`  — the reserved keep-VL encoding was used with a VLMAX change
    (logic part 5). A separate output because it is NOT derivable from
    `dec_vtype_imm` alone — it needs the `prev_vtype` comparison — and it must
    reach the `vill` of both the mirror and the uOP.
  - `frontend_only`    — this uOP occupies no issue queue and no functional unit;
    true for `vsetivli` with `rd == x0` and nothing else. This, and NOT merely "is
    a `vsetivli`", is what qualifies VConfigUnit's `dec_vl_imm_valid` and therefore
    the rename-cycle VL-RF write.
  - `uop_out`          — the overridden `MicroOp`, merged by the `DecodeUnit` delta.

  Deliberately absent: the computed VL value (VConfigUnit owns `dec_vl_imm`; this
  module supplies its operands), any vtype-mirror state, and any handshake or
  back-pressure line in either direction — decode is a fixed one-cycle stage and
  this is a combinational function of its inputs.
  <|end_ports|>

  <|begin_logic|>
  Purely combinational, evaluated independently per lane. NO register, memory or
  state may be declared here: the two cross-lane values consumed, `dec_vconfig`
  and `prev_vtype`, arrive already prefix-selected from VConfigUnit, which is the
  sole owner of vtype state in the machine.

  ---- 1. Recognition and the three-way split ----

  A lane is a `vset` when its opcode is OP-V (`inst[6:0] = 0b1010111`) with
  `funct3 = 0b111`. The shapes separate on the top bits, matching rocket's
  `Instructions.VSETIVLI`/`VSETVLI`/`VSETVL` patterns, and the field positions are
  deliberately identical to rocket's:

    vsetivli  `inst[31:30] === 0b11`     vtype = `inst[29:20]`, AVL = `inst[19:15]`
                                         zero-extended (an immediate, not a regno)
    vsetvli   `!inst[31]`                vtype = `inst[30:20]`, AVL = `rs1`
    vsetvl    `inst[31:25] === 0b1000000` vtype = `rs2`,        AVL = `rs1`

  Take the field positions from rocket, not from a fresh reading of the ISA
  manual. rocket's CSRFile owns architectural vtype (ground rule 9), so a
  disagreement about where the vtype bits live would leave the speculative
  mirror and the architectural CSR in different configurations with no
  exception raised anywhere.

  Legality is NOT computed here, and the extracted vtype bits leave as RAW
  `dec_vtype_imm`. VConfigUnit is the caller that turns them into a checked
  `VType` through VtypeTable's `decode(bits)`, and this module reads the result
  back as `dec_vconfig` rather than decoding a second copy — three copies of a
  `vill` rule is three chances to disagree, which is why that table is one shared
  package and why this file calls it for exactly one thing, part 5's VLMAX
  comparison.

  ---- 2. `vsetivli`, `rd == x0`: the front-end-only path ----

  //@req-spec-decode.c3
  Both VTYPE and AVL are immediate, so nothing is left for a back-end unit to
  compute. Drive `iq_type` ALL-CLEAR and `fu_code` ALL-CLEAR: the uOP is
  dispatched to no issue queue and reaches no functional unit; it consumes an ROB
  entry and nothing else. `frontend_only` is asserted for exactly this case and
  `dst_rtype` is `RT_ZERO`.

  //@req-spec-decode.c26
  //@req-spec-decode.c5
  Both operands of the VL computation are exported: `dec_avl_imm` (the `uimm`
  field) and `dec_vtype_imm`. VConfigUnit evaluates
  `VtypeTable.computeVL(avl, bits, _, useCurrentVL = false, useMax = false,
  useZero = false)` — which is `min(uimm, VLMAX)` — and publishes it as
  `dec_vl_imm`, qualified by `frontend_only` from this module. That pair IS the
  rename-side VL-RF write request: `pvl` is allocated by the VL mapper in the
  rename cycle and written in that same cycle on the VL RF's statically
  partitioned rename-side port. Because the value arrives WITH the allocation, NO
  VL busy bit is set — this `pvl` is BORN READY, so a dependent vector uOP in the
  same or the next dispatch group never waits on it. `is_vl_producer` is
  nevertheless SET: the VL still lives in the VL register file and is still renamed
  through `pvl` exactly like any other VL producer. There is NO decode-time VL fast
  path, no bypass around the VL RF and no statically-known-VL route for consumers.

  Delegating the computation is also what keeps the M1 AVL bug out: that
  implementation truncated AVL before comparing it against VLMAX, so a large AVL
  WRAPPED instead of saturating (AVL = 2048 gave vl = 0) and the canonical
  strip-mining loop broke. VtypeTable tests the high bits instead of truncating.

  ===> DO NOT REINTRODUCE THE M1 `vl_is_known` MicroOp FIELD. The MicroOp delta
  forbids it by name and it is unnecessary: born-readiness is a property of the
  WRITE REQUEST, seen by the rename cycle as it allocates. The VL value must
  also ride the SAME registered decode -> ren2 stage as its uop; a shadow
  pipeline for it is the shape of the M1 free-list double-free, where vector
  logic ran a cycle ahead and acted on the next cycle's bubble.

  //@req-spec-decode.c6
  This uOP has no writeback, so nothing exists that could ever clear its ROB busy
  bit: its ROB entry must be dispatched NON-BUSY and commits as soon as it reaches
  the head, where it writes the architectural `vtype`/`vl` and the committed VCFG
  shadow. The encoding the ROB keys on is structural rather than a special case —
  `iq_type` all-clear means the uop issues nowhere and no producer is owed.
  Nothing here modifies `MicroOp.starts_bsy`, whose body the MicroOp delta
  freezes; non-busy dispatch belongs to the Rob delta and this paragraph is the
  contract it reads.

  ===> `vsetivli` WITH `rd != x0` IS NOT FRONT-END ONLY — the one place this file
       departs from the letter of "only vsetivli is front-end only". `rd` must
       receive the new `vl` and the front end has no integer-RF write port to
       deliver it with; rename-side integer writes do not exist and integer rename
       is explicitly not modified. So for `rd != x0` the shape takes the
       register-sourced path below with `dst_rtype = RT_FIX`, the ALU computing VL
       from the IMMEDIATE AVL rather than from `rs1`, `frontend_only` LOW, and the
       VL RF written by the ALU writeback like any register-sourced vset — which
       keeps `is_vl_producer` at ONE meaning ("writes the VL RF"), never qualified
       by shape at the writeback. One condition, not a per-shape table: the
       front-end-only path is taken exactly when the VL is immediate AND no
       integer destination is needed.

  ---- 3. `vsetvli`, `vsetvl` (and `vsetivli` with `rd != x0`): the ALU path ----

  //@req-spec-decode.c8
  Set `iq_type` to `IQ_ALU` — the SCALAR INTEGER issue queue — and `fu_code` to
  `FC_ALU`. From dispatch's point of view these are ordinary integer uOPs; none of
  the three `IQ_V_*` queues is involved, and `is_vec` stays CLEAR because a `vset`
  is not an OP.v: no vector operand, no vector destination group, nothing for the
  group-done completion path to complete. Its vector character is carried entirely
  by `vconfig` and `is_vl_producer`.

  //@req-spec-decode.c9
  Wakeup is the existing integer network, unmodified: `lrs1 = rs1` with
  `lrs1_rtype = RT_FIX`, and for `vsetvl` additionally `lrs2 = rs2` with
  `lrs2_rtype = RT_FIX`, so the uOP is woken by `rs1` (and `rs2`) exactly like any
  integer op. `lrs3_rtype` and `frs3_en` are cleared.

  ===> AN `x0` SOURCE MUST DECODE RT_ZERO, NOT RT_FIX: drive
  `Mux(field === 0.U, RT_ZERO, RT_FIX)`, mirroring the scalar decoder, because
  `rename-stage.scala` asserts `!(r_valid && lrs1_rtype === RT_FIX && lrs1 ===
  0.U)` and it fires on the very first `vsetvli rd, x0, vtypei` otherwise. Both
  fields need it — `rs1 == x0` is a defined and common vset form, and
  `rs2 == x0` on a `vsetvl` legally requests the all-zero vtype.

  //@req-spec-decode.c16
  These uOPs have TWO destinations in TWO independent rename spaces. Drive
  `dst_rtype = Mux(rd === 0.U, RT_ZERO, RT_FIX)` for `pdst` in the integer RF and
  SEPARATELY drive `is_vl_producer` for `pvl` in the VL RF. `is_vl_producer` must
  not be inferred from `dst_rtype` by anyone: with `rd == x0` `dst_rtype` is
  `RT_ZERO` and the VL RF is still written, so a consumer inferring "writes VL"
  from `dst_rtype` would drop the write and every dependent would read a stale VL.
  Both destinations take the same value off one result bus, which is what makes
  the second rename space cheap.

  The remaining integer control fields are driven to a defined, quiescent
  selection rather than left at the meaningless baseline decode: `fcn_op` to the
  ALU's `FN_ADD` identity at full `fcn_dw` width and `op1_sel`/`op2_sel` to
  `OP1_RS1`/`OP2_ZERO`. The ALU's own result mux replaces its output with the
  computed VL, exactly as rocket does with
  `mem_reg_wdata := Mux(set_vconfig, new_vl, alu.io.out)`.

  Set `imm_rename = false` and `imm_sel = IS_N`: a `vset` consumes NO immediate
  rename resource, because the AVL and vtype fields are read at execute straight
  off `uop.inst`, which every uOP already carries. Set `csr_cmd = CSR.N` — a
  `vset` is not a CSR access; the architectural `vtype`/`vl` update happens at
  COMMIT through the ROB's path to rocket's `CSRFile` and the committed VCFG
  shadow (ground rule 9), and marking `csr_cmd` would serialize every `vset` for
  no gain. Clear `uses_ldq`/`uses_stq` and leave `exception`/`exc_cause` as the
  baseline produced them — the `mstatus.VS = Off` gate is rocket's.

  ---- 4. `vsetvl` must serialize ----

  //@req-spec-decode.e1
  For the `vsetvl` shape ONLY, raise BOTH `is_unique` AND `flush_on_commit`. The
  reason it must serialize where `vsetvli` need not is EMUL: the vector mapper
  needs VTYPE at rename to size a destination group; `vsetvli` has VTYPE as an
  immediate so EMUL is known at decode, and `vsetvl` does not — its VTYPE is in
  `rs2` and exists only after execute.

  `is_unique` ALONE IS NOT ENOUGH, and an earlier draft of the chapter claimed
  it was. It gates only this uop's OWN dispatch (`core.scala:739-740`) until
  everything OLDER has retired, and says nothing about YOUNGER uops, which
  decode against a stale vtype mirror, derive the wrong EMUL and make the
  mapper allocate the wrong number of vector PRNs — silent group mis-sizing
  with no misprediction involved, so the per-br_tag VCFG snapshot recovery
  never fires. `flush_on_commit` is what makes younger code observe the effect:
  everything younger is refetched once the `vsetvl` commits, and that flush
  reloads the speculative mirror from the committed shadow. So NO execute-time
  mirror write is needed and none may be added.

  Both bits are also set by the `DecodeUnit` delta for this encoding: the SAME
  signal reached from the other side of the merge, not a second mechanism. If the
  two ever disagree, this file is the origin.

  ---- 5. The four `vsetvli` sub-cases ----

  Split on the two register fields, `rs1 = inst[19:15]` and `rd = inst[11:7]`:

  //@req-spec-decode.i15
  (a) `rs1 != x0` — VL = `min(rs1, VLMAX)`, computed by the ALU from the integer
      RF or bypass. A VL producer: `is_vl_producer` is SET.
  (b) `rs1 == x0, rd != x0` — VL = VLMAX. Still a producer, still computed at
      execute and written to the VL RF from the ALU: `is_vl_producer` is SET.
      There is no decode-time fast path here either, even though VLMAX is a
      function of an immediate vtype alone.

  //@req-spec-decode.i11
  //@req-spec-decode.i12
  //@req-spec-decode.i16
  (c) `rs1 == x0, rd == x0` — the RESERVED KEEP-VL FORM. VL is KEPT UNCHANGED.
      This is NOT a VL producer: `is_vl_producer` is CLEAR, so no `pvl` is
      allocated, the VL MAP TABLE IS LEFT UNTOUCHED, and every younger uOP keeps
      reading the existing `pvl`. `frontend_only` is low and `dst_rtype` is
      `RT_ZERO`. The uop still takes the integer-ALU path with both source types
      `RT_ZERO`, retiring through an ordinary writeback that writes neither
      register file — uniform with the other sub-cases, and cheaper than a fourth
      dispatch path for an instruction whose only effect is a vtype update.
      The zero in the rs1 FIELD selects a behaviour; it is not an operand value.

  //@req-spec-decode.i13
      RESERVED-ENCODING CHECK for case (c): if the new immediate vtype changes
      VLMAX relative to `prev_vtype`, the encoding is reserved (RVV 1.0 section
      6.2) and Caracal sets `vill`. Compute
      `VtypeTable.decode(dec_vtype_imm).vlmax =/= VtypeTable.decode(prev_vtype).vlmax`
      and drive it on `keep_vl_illegal`, which VConfigUnit must OR into the mirror's
      `vill`; OR it into `uop_out.vconfig.vill` here as well, so the uOP that must
      trap carries its own poison and does not depend on the neighbour's fold.
      VLMAX equality IS the SEW/LMUL-ratio test, since VLMAX = VLEN*LMUL/SEW.
      Compare the derived VLMAX rather than rebuilding the ratio from the
      fields, so this check cannot drift from VtypeTable's legality rules. An
      already-illegal vtype is caught by its own `vill` term and has `vlmax`
      driven to zero, so the comparison cannot manufacture a spurious pass. The
      `vill` set here poisons the VCFG mirror, which is what makes younger
      vtype-dependent uOPs trap at decode instead of allocating a mis-sized PRN
      group.

  //@req-spec-decode.i14
  (d) Otherwise — `rd == x0` together with a VL CHANGE, i.e. `rd == x0` and
      `rs1 != x0`. A VL PRN IS STILL ALLOCATED to hold the new VL:
      `is_vl_producer` is SET even though `dst_rtype` is `RT_ZERO`. This case is
      precisely why the two cannot be one field — discarding the integer
      destination says nothing about the VL destination, and the value must be
      somewhere for younger uOPs to read.

  The same rd/rs1 split governs `vsetvl`, whose AVL is also `rs1`, with one
  difference: its VTYPE is not immediate, so the keep-VL reserved check cannot be
  evaluated at decode and is left to execute.

  ---- 6. `vconfig` and the mirror update ----

  //@req-spec-decode.d8
  A `vset`'s OWN `vconfig` field holds its NEW vtype, not the vtype it was decoded
  under, because that is what the ROB writes to the architectural `vtype` CSR and
  the committed VCFG shadow at commit. Drive `uop_out.vconfig := dec_vconfig` with
  `keep_vl_illegal` ORed into its `vill`. This is correct only if VConfigUnit's
  per-lane prefix select is SELF-INCLUSIVE for a `vset` lane and self-exclusive for
  a consumer lane, which is the contract that unit states; taking the value from
  there rather than re-decoding it locally is what keeps the uOP snapshot and the
  mirror from ever disagreeing about the same instruction.

  `vsetvl` is the exception and cannot satisfy the requirement as written: its new
  vtype is a runtime value in `rs2` and does not exist at decode. For that shape
  `dec_vtype_is_imm` is LOW and `uop_out.vconfig` carries the incoming
  `prev_vtype` unchanged, as a documented don't-care — the architectural update at
  commit takes the EXECUTED vtype off the ALU result path (VConfigUnit's
  `com_vtype` input says the same thing from the commit side), and
  `flush_on_commit` guarantees no younger uOP can have observed the placeholder.

  ---- 7. Assertions and tracing ----

  Assert that at most one shape predicate is set per lane; that `frontend_only`
  implies `iq_type` all-clear, `is_vl_producer` set and `is_vsetivli` set; that
  `keep_vl_illegal` implies `is_vsetvli` with both register fields zero; and that
  `dec_vtype_is_imm` is false for `vsetvl` and true for the other two shapes. Also
  assert `dec_avl_imm` is only consumed when `is_vsetivli` — it occupies the same
  bit positions as `rs1`, so a consumer that read it for the wrong shape would
  quietly use a register NUMBER as an element COUNT.

  With no unit tests in this project (ground rule 11), emit guarded VecTrace
  lines: one per decoded `vset`, emitted through `VecTrace`'s **`traceDecode`**
  variant — keyed on `ftq_idx`/`pc_lob`, NOT on `rob_idx`, because the ROB entry is
  allocated at DISPATCH and no `rob_idx` exists at decode. Inventing a zero would
  alias with real ROB entry 0 in every grep, which is worse than admitting the
  identifier is not yet known. Tagged with module name, carrying
  the shape, the raw immediate vtype, `keep_vl_illegal`, `is_vl_producer` and
  `frontend_only`, gated on the `vecTrace` plusarg and off by default. This is the
  only record of which path a `vset` took, and what a VL mismatch against the
  Whisper cosim trace is diagnosed from.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Single-cycle combinational, `coreWidth` copies, in the DECODE critical path. It
must add no pipeline stage: the shape, the extracted fields and the reserved-case
check are all available in the same cycle as the instruction word.

The path worth watching is the cross-lane one. `dec_vconfig` and `prev_vtype` for
lane `w` depend on the immediate vtype decoded in lanes `0..w-1`, so decode carries
a prefix scan of depth `coreWidth` through VConfigUnit's per-lane select. The chain
is acyclic — a lane depends only on OLDER lanes — but this module sits on it twice
(the raw fields go out, the selected vtype comes back), so keep its own
contribution to bit-slicing plus ONE `VtypeTable.decode` and ONE VLMAX equality
comparator. Do not rebuild the SEW/LMUL ratio as a second comparator tree, and do
not re-decode `dec_vconfig` locally.

No register, no memory, no arbitration and no handshake anywhere here. Any of
those would mean per-lane state had crept into decode, where the only owner of
vtype state is VConfigUnit.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the `uop_in`/`uop_out` bundle, specifically the added `is_vl_producer`
and `vconfig` fields and the widened three-bit `dst_rtype`/`lrs1_rtype`/
`lrs2_rtype`. This module writes `is_vl_producer` and `vconfig`; it reads neither
`pvl` nor any vector group field.

VtypeTable — `decode(bits)`, used for ONE thing: the keep-VL VLMAX comparison.
Every other legality and VL rule is evaluated by VConfigUnit from the raw fields
this module exports, so no rule is restated here and none is computed twice.

VecTrace — the guarded trace convention.

Binds by name to `freechips.rocketchip.rocket.VType` for the `dec_vconfig` and
`prev_vtype` payloads, to rocket's `Instructions.VSETIVLI`/`VSETVLI`/`VSETVL` bit
patterns so recognition shares rocket's encoding rather than re-deriving it, and
to `boom.v4.common.constants.ScalarOpConstants` for `IQ_ALU`, `FC_ALU`, `RT_FIX`,
`RT_ZERO`, `IS_N` and the `OP1_*`/`OP2_*` selections.

Instantiates nothing. Instantiated once, as `vset`, by VecDecode, which wires it.
Its counterparties are VConfigUnit (both directions: the shape/field/illegal
outputs go there, `dec_vconfig` and `prev_vtype` come back, and that unit's
`dec_vl_imm` is qualified by `frontend_only` from here), the `DecodeUnit` delta
(the `uop_out` merge), the `ALUUnit` delta (which executes every shape that is not
`frontend_only`), VlRegFile (the rename-side write port that `frontend_only`
enables) and the `Rob` delta (non-busy dispatch of the `frontend_only` shape).
<|end_dependencies|>

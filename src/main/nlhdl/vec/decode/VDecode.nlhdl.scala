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
  VDecode — the arithmetic RVV opcode decoder: one combinational lane per decode
  lane that turns an OP-V instruction word into the vector fields of ONE MicroOp.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/decode/VDecode.scala,
  package boom.v4.vec.generated.decode. group vec_decode.
  depends_on MicroOp, VecTrace, VtypeTable. Instantiated once, as `arith`, by
  VecDecode alongside VLSDecode (`ls`), VsetDecode (`vset`) and VConfigUnit
  (`vcfg`). Instantiates nothing.

  ===> ONE uOP PER INSTRUCTION, AND NOTHING HERE MAY EXPAND ONE. This module is
       purely combinational, holds no register, and emits exactly one uOP per
       lane per cycle. Element expansion into nOP.v happens in the vector LS
       AGEN and nowhere else; a destination-per-register expansion at decode
       would multiply the ROB entry count of every vector program by up to 8 and
       is the single thing this node exists to not do.

  ===> IT IS DELIBERATELY NOT A FULL RVV DECODER. The CII issue packet carries
       the raw 32-bit instruction word (`CiiIssueReq.instr`) plus vtype/vl/
       vstart/vxrm, and the coprocessor decodes the operation itself. So VDecode
       decodes only what the HOST needs: which architectural registers are read
       and written, which rename space each destination belongs to, which issue
       queue the uOP is routed to, how many registers the destination group has,
       and whether a second execution resource is needed. It reads funct6 ONLY
       for the destination-SHAPE classes (widening, single-register destination,
       whole-register move); there is no operation table, no ALU function code
       and no immediate extraction. A second full decoder of the same instruction
       would be a second thing to keep in agreement with the VPU's decoder, which
       is the class of divergence the CII's raw-instruction field exists to avoid.

  Governing spec anchors: frontend.rst `vector-rvv-decode` (the decoder itself,
  "CII Shared Instruction Decoding", "VSET Special Handling" for the EMUL
  obligation, `vector-csr-ownership` for the vill poison and its whole-register
  exemption), glossary.rst `glossary-terms` (uOP / OP.v / nOP.v / shared
  instruction), issue.rst `cii-shared-sched`, midcore.rst `old-vd`.
*/

<|begin_module|>

  <|begin_parameters|>
  `usingRVV` — the Scala `Boolean` derived from `BoomCoreParams`, not a hardware
  `Bool`. This whole module is elaborated only when it is true; with vectors off
  it is ABSENT, not tied off, so a non-vector build stays bit-identical to
  pre-Caracal BOOM v4 (plan gate (f)). Do not gate on rocket-chip's
  `usingVector`, which is a different switch with a different owner.

  `coreWidth` — the number of decode lanes, from `HasBoomCoreParameters`, default
  3 (`MediumBoomV4VectorConfig`). ONE instance covers all lanes, which is why
  hierarchy.yaml lists a single `arith` instance and not one per lane. The lanes
  are fully independent and there is no cross-lane logic here at all: the only
  cross-lane decode logic in the machine is the nearest-preceding-`vset` prefix
  select, which VConfigUnit owns and whose result a lane merely consumes.

  `maxMembers` — from VectorParams, fixed at 8, the largest legal EMUL group.
  Used as the reserved-encoding bound below, never written as a literal 8.

  No other parameters. Every field width comes from `MicroOp` or VectorParams
  (`lregSz`, `log2Ceil(maxMembers) + 1`); no width here is a literal.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel and hierarchy.yaml defaults: posedge `clock`,
  ACTIVE-HIGH SYNCHRONOUS `reset`, single `core_clk` domain. Both are the
  implicit Chisel signals and neither reaches the datapath — this module declares
  no register and no memory, so they are used only by the guarded trace
  statements. There is no ready/valid handshake and no back-pressure line in
  either direction: decode cannot stall on this module, and stalling the decode
  stage stays DecodeUnit's business.

  Every port below is a `Vec(coreWidth, ...)`, lane-indexed identically. Inputs:

  - `inst`      — `UInt(32.W)`, the lane's instruction word.
  - `valid`     — `Bool`, the lane's decode valid. It qualifies the trace and the
                  illegal output only; the decoded fields are don't-care rather
                  than zeroed when it is low, since gating them would add a mux
                  layer to the decode critical path for no consumer.
  - `uop_in`    — `new MicroOp()`, the uOP as baseline `DecodeUnit` decoded it.
  - `vtype_in`  — `freechips.rocketchip.rocket.VType`, the 9 mirrored bits
                  {vlmul, vsew, vta, vma, vill} for THIS lane, already resolved by
                  VConfigUnit's nearest-preceding-`vset` prefix select. Rocket's
                  type, not a re-spelled one, for the reason VtypeTable gives: the
                  speculative mirror and the architectural CSR must not have two
                  encodings of `vtype`.
  - `ls_in`     — the part of VLSDecode's static access descriptor this module
                  needs: `is_mem`, `is_load`, `is_store`, `is_whole_reg` and the
                  raw `nf` field. It is an input rather than a local re-decode so
                  that `nf` is decoded exactly once in the machine; the
                  `is_shared` paragraph says why that is a live hazard and not
                  merely redundancy.

  Outputs:

  - `uop_out`   — `new MicroOp()`, `uop_in` with the fields enumerated in the
                  logic section overridden and every other field passed through.
  - `is_arith`  — `Bool`, this lane holds vector ARITHMETIC. The parent folds it
                  into its RVV-legality term and owns `dec_vec_illegal`.
  - `vill_trap` — `Bool`, this lane must raise illegal-instruction at decode. It
                  is an output because the two terms that set it are decidable
                  only here, while raising the trap is the parent's job.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. Recognition ----

  A lane holds vector arithmetic when its opcode is OP-V (`inst(6,0) === 0x57`)
  and its funct3 (`inst(14,12)`) is NOT `0b111`. Funct3 `0b111` is OPCFG, the
  `vset` class, which is VsetDecode's; the seven other funct3 values are the
  arithmetic forms OPIVV/OPFVV/OPMVV/OPIVI/OPIVX/OPFVF/OPMVX. Vector loads and
  stores use the LOAD-FP/STORE-FP opcodes and are never matched here.

  //@req-spec-core.c3
  //@req-spec-decode.a4
  //@req-spec-decode.a6
  //@req-spec-decode.a7
  Each lane is a single-cycle combinational function producing exactly ONE
  decoded uOP packet. There is no state, no sequencing, no iteration and no
  second output beat: `uop_out(i)` is a function of `inst(i)`, `uop_in(i)`,
  `vtype_in(i)` and `ls_in(i)` in the same cycle. A vector instruction is one
  uOP exactly as a scalar instruction is; this module performs no cracking and
  no micro-op expansion of any kind, so no configuration of it can emit more
  uOPs than the lane count.

  //@req-spec-core.c5
  //@req-spec-core.c7
  //@req-spec-core.c8
  The one uOP represents the WHOLE `LMUL`/`EMUL` destination register group and
  will occupy ONE ROB entry. Nothing here indexes a group member, and no field
  written here names a single member of a group: the group is named by its
  architectural number (`lvd`) and its size (`v_emul`), and the member PRNs are
  the vector mapper's to allocate atomically at rename. The absence of a member
  index in this module's outputs is the structural reason frontend cracking
  cannot creep back in — an OP.v stays one uOP through decode, rename, the ROB
  and issue, and is expanded into nOP.v only inside the vector LS AGEN, which is
  a load/store path this module never feeds.

  ---- 2. The fields this module writes ----

  For a lane where `is_arith` holds:

  - `is_vec := true.B`. An OP.v is an ordinary uOP with this bit set; it is not
    a separate bundle type.
  - `lvd := inst(11,7)` and `dst_rtype := RT_VEC` for a vector destination.
  - `lvs2 := inst(24,20)`, with `lrs2_rtype := RT_X` — the vs2 field is always a
    vector source for arithmetic.
  - `lvm := 0.U`. The mask register is architecturally always `v0`; carry it as a
    field so the mask reads through the same map-table path as any other source.
  - `iq_type(IQ_V_ALU) := true.B` and `fu_code(FC_ALU) := true.B`, so the uOP
    matches the `fu_types` the CII host advertises to `IQ_V_ALU`.

  // This module sets ONLY the IQ_V_ALU position and does not clear the others.
  // Clearing the scalar-derived iq_type/fu_code bits for a recognized RVV
  // instruction is VecDecode's single assignment, made once before the four
  // children's field writes compose. If this module cleared them too, the result
  // would depend on the order the parent chains its children — a last-connect
  // ordering dependence that survives review and breaks on the next reorder.

  `lvs1` is the vs1 field `inst(19,15)` when it names a vector register, i.e. the
  OPIVV/OPFVV/OPMVV forms; for the scalar-feeder and funct5-selector forms it is
  not a vector source at all (next two paragraphs).

  `lvs3 := inst(11,7)`, the destination architectural register, for every
  arithmetic lane with a vector destination. This is how "`pvs3` and
  `stale_pvdest` coincide for read-modify-write arithmetic" (midcore.rst `old-vd`)
  is actually produced: the map-table read of `lvs3` and the old-`vd` lookup are
  reads of the same architectural register in the same rename cycle. It costs
  nothing on an op with no encoded third source, whose VS3 slot the coprocessor
  never requests, and adds no dependency that was not there already — `IQ_V_ALU`
  must gate on the old destination group regardless, since the coprocessor pulls
  STALE_VD for merges.
  // Do NOT instead have rename copy stale_pvdest into pvs3, and do NOT add a
  // funct6 table of the RMW families here. The first makes two fields alias
  // through a hidden assignment, which is what design invariant 7 forbids; the
  // second is another copy of the VPU's decoder.

  ---- 3. Scalar feeders, and the x0 rule that bit us ----

  The vs1 field is a SCALAR source for the `.vx`/`.vf` forms, and the scalar
  value is read from the integer or FP register file through the existing INT/FP
  wakeup and read paths — no new network:

  - funct3 OPIVX (`0b100`) or OPMVX (`0b110`): `lrs1 := inst(19,15)` with
    `lrs1_rtype := Mux(rs1 === 0.U, RT_ZERO, RT_FIX)`.
  - funct3 OPFVF (`0b101`): `lrs1 := inst(19,15)` with `lrs1_rtype := RT_FLT`.
  - otherwise: `lvs1 := inst(19,15)` and `lrs1_rtype := RT_X`.

  // ===> THE x0 CONVERSION IS MANDATORY, NOT DEFENSIVE. RVV permits `.vx` and
  // `vmv.s.x` to read x0 (value 0), and rename-stage.scala:109 asserts
  // `!(r_valid && lrs1_rtype === RT_FIX && lrs1 === 0.U)`. An earlier
  // implementation omitted the conversion and the assertion fired on the first
  // real kernel that used `vmv.s.x v12, zero`. Mirror the scalar decoder
  // (decode.scala:505) exactly: RT_FIX becomes RT_ZERO when the specifier is 0.
  // RT_FLT needs no such conversion — f0 is a real register.

  Scalar-DESTINATION arithmetic (funct6 `0b010000`): under OPMVV this is
  `vmv.x.s`/`vcpop.m`/`vfirst.m`, which write an integer GPR, and under OPFVV it
  is `vfmv.f.s`, which writes an FP register. For these set `ldst := inst(11,7)`
  and `dst_rtype := RT_FIX` or `RT_FLT` so the destination is renamed in the
  scalar space and written back through the CII's scalar writeback port, and do
  NOT set `RT_VEC`. `is_vec` stays true — these still execute on the coprocessor.
  In this family the vs1 field is a funct5 selector rather than a register, so
  `lrs1_rtype` stays `RT_X` and no scalar source is read.

  ---- 3b. `v_uses_vs*` — which vector sources the FORMAT encodes ----

  Every arithmetic lane also writes `v_uses_vs1`, `v_uses_vs2` and `v_uses_vs3`,
  the three `MicroOp` Bools that say whether the format encodes that vector
  source at all. Only decode knows the format, and the instruction word does not
  reach rename or issue, so these bits cannot be recovered anywhere else.

  ===> AN UNENCODED SOURCE LEFT UNMARKED IS A HANG, NOT A LOST OPTIMIZATION. For
       `vadd.vx` the vs1 field is unencoded, `lvs1` is meaningless, and the
       mapper's map-table read returns the current mapping of `v0` — the MASK
       register, rewritten constantly by any masked program. If that mapping's
       producer fired its group-done BEFORE the issue slot captured its
       member-ready bits, nothing will ever clear them again and the slot waits
       forever on an operand the instruction does not read. The MicroOp delta
       (section 4) carries the full argument; this module is where the bits are
       produced. The two consumers are the vector mapper (skip renaming an
       unencoded source, leave its busy bit clear) and `VecIssueSlot` (drive the
       bit onto the matching `used` input).

  `v_uses_vs1` is true for the VECTOR-VECTOR forms — funct3 OPIVV (`0b000`),
  OPFVV (`0b001`) and OPMVV (`0b010`) — and false everywhere else. Two
  qualifications, one per direction:

  - false for OPIVX/OPMVX/OPFVF, where the vs1 field is `rs1` and part 3 has
    already routed it to `lrs1` in the integer or FP space, and false for OPIVI,
    where it is `simm5`;
  - false, even inside those three funct3 values, for the FUNCT5-SELECTOR
    families under OPMVV/OPFVV, which are exactly
    the funct6 values matching `funct6(5,2) === 0b0100`: `0b010000`
    (`VWXUNARY0`/`VWFUNARY0` — part 3's scalar-destination ops), `0b010010`
    (`VXUNARY0`/`VFUNARY0` — `vzext`/`vsext`, the FP convert family), `0b010011`
    (`VFUNARY1`) and `0b010100` (`VMUNARY0` — `vmsbf`/`vmsif`/`vmsof`/`viota.m`/
    `vid.v`). In every one of those the vs1 field is a 5-bit SUB-OPCODE, not a
    register number, so renaming it would rename an arbitrary architectural vreg
    named by an opcode.

  `v_uses_vs2` is true for every arithmetic form EXCEPT the families whose vs2
  field is architecturally reserved-zero. There are three and they must each be
  matched:

  - funct6 `0b010111` WITH `vm = 1`, under OPIVV/OPIVX/OPIVI/OPFVF only —
    `vmv.v.v`/`vmv.v.x`/`vmv.v.i`/`vfmv.v.f`. BOTH qualifiers are load-bearing:
    the same funct6 with `vm = 0` is `vmerge.v*m`/`vfmerge.vfm`, which DOES read
    vs2, and the same funct6 with `vm = 1` under OPMVV is `vcompress.vm`, which
    reads vs2 AND vs1. A funct6-only test breaks every `vmerge`; a funct6-plus-vm
    test that forgets the funct3 restriction breaks `vcompress`, and both break it
    in the silent-corruption direction rather than the hang direction.
  - funct6 `0b010000` under OPMVX/OPFVF — `vmv.s.x` and `vfmv.s.f`. Note the
    funct3 restriction here too: the same funct6 under OPMVV/OPFVV is part 3's
    scalar-destination family (`vmv.x.s`/`vcpop.m`/`vfirst.m`/`vfmv.f.s`), whose
    vs2 IS the source vector.
  - `vid.v` alone, matched by its full sub-opcode (OPMVV, funct6 `0b010100`, vs1
    selector `0b10001`). Its `VMUNARY0` siblings `vmsbf`/`vmsif`/`vmsof` and
    `viota.m` all read vs2, so this one cannot be matched at funct6 granularity.

  `vmv<n>r.v` (funct6 `0b100111` under OPIVI) DOES read vs2 — vs2 names its
  source group — so it is not in the list.

  `v_uses_vs3` is true exactly when the lane has a VECTOR destination, i.e.
  wherever part 2 writes `lvs3 := inst(11,7)` and `dst_rtype := RT_VEC`. It is
  false for part 3's scalar-destination family, where `inst(11,7)` names a GPR or
  an FP register and `lvs3` names no vector register at all. There is no separate
  "encodes a third source" test, and part 2 says why: `lvs3` is deliberately the
  destination for every vector-destination arithmetic op, because the coprocessor
  may pull STALE_VD for a merge and the slot must gate on that group regardless.

  On a lane where `is_arith` is false this module writes none of the three; they
  are defaulted false by the `DecodeUnit` delta (see the MicroOp delta's edit
  scope), which is the only place a *scalar* uop's copies can be driven.

  // ===> THE AUTHORITY FOR THESE LISTS IS THE RVV 1.0 ENCODING TABLE PLUS THE
  // VPU's own decode, NOT intuition about which operands an instruction "uses".
  // A wrongly-true bit hangs; a wrongly-false bit drops a real dependency and
  // reads a stale group, which is silent corruption. Review both directions
  // against the same tables part 5's single-register-destination list is
  // reviewed against.

  Also write `v_is_masked := !inst(25)` on every arithmetic lane. RVV's `vm` bit
  is 1 for UNMASKED, so the field is the COMPLEMENT of the encoded bit, and this
  module owns the arithmetic side of it exactly as VecDecode owns the memory side
  (`!desc.vm` qualified by the two non-maskable unit-stride forms). The field is
  required because `inst` does not reach the issue slot or the mapper, both of
  which must know whether `pvm` participates — an unmasked op whose `pvm` is
  ANDed into readiness waits forever on whatever physical register `v0` last
  mapped to, the same failure the `v_uses_vs*` bits above exist to prevent.

  ---- 4. EMUL — the one number the mapper cannot derive for itself ----

  //@req-spec-decode.d1
  Every uOP this module emits carries `v_emul`, the DESTINATION group's member
  count as a 1..`maxMembers` value, obtained from the VtypeTable package and from
  nowhere else: evaluate `VtypeTable.decode(vtype_in(i))` for the lane and then
  `VtypeTable.emul(info, dest_eew)`. Because vector uOP cracking is deferred to
  execute, the vector mapper must allocate the whole PRN group up front, and
  `v_emul` is the only input it has for the size. An EMUL that disagreed with the
  one the coprocessor derives from the vtype in the same issue packet would
  corrupt member indexing on the CII, which is why the derivation is bound to the
  shared package rather than open-coded here.

  `dest_eew` for arithmetic is SEW, i.e. `vtype_in.vsew`, with two exceptions
  that are statically decodable from the instruction word:

  - The WIDENING families produce a 2*SEW destination, so their group is twice as
    many registers: funct6 `0x30`..`0x3F` under OPMVV/OPMVX (vwadd/vwsub/vwmul/
    vwmacc and the `.w` forms) and the corresponding OPFVV/OPFVF widening
    families. Pass `vsew + 1` as the destination element width for these.
  - The NARROWING families (funct6 `0x2C`..`0x2F` under OPIVV/OPIVX/OPIVI —
    vnsrl/vnsra/vnclip) keep a SEW-wide destination; it is their SOURCE that is
    2*SEW. They need no adjustment here.

  A source-side group size is deliberately NOT carried, and no second field is
  added for one. The only host consumers of a group size are the mapper's atomic
  destination allocation and the group-done clear over the destination members; a
  SOURCE group's member count belongs to whichever older op allocated it and is
  recorded in the vector map table. The coprocessor names each source member it
  wants by `op_offset` on the CII source-request channel, so nothing on the host
  needs a per-source EMUL.

  ---- 5. Single-register destinations: EMUL is 1 regardless of LMUL ----

  //@req-spec-decode.d1
  Several arithmetic classes write ONE vector register whatever the current LMUL,
  and for them `v_emul` must be 1, overriding the vtype-derived value:

  - the MASK-RESULT ops, which write one mask register: the integer and FP
    compares (`vmseq`..`vmsgt`, `vmfeq`..`vmfge`), `vmadc`/`vmsbc`, the
    mask-logical family, and `vmsbf`/`vmsif`/`vmsof` from `VMUNARY0`. Note that
    `viota.m` and `vid.v` are NOT in this set — they write a full EMUL group and
    take the vtype-derived count;
  - the reductions (`vred*`, `vfred*`, `vfwred*`), which write element 0 of one
    register;
  - `vmv.s.x` and `vfmv.s.f`, which write element 0 of one register;
  - the scalar-destination ops of part 3, which allocate no vector group at all
    and take 1 so that nothing downstream reads a larger count.

  // ===> GET THIS WRONG AND THE MACHINE HANGS, IT DOES NOT MISCOMPUTE. The
  // coprocessor emits `last` on the writeback beat of member (dst_nm - 1). If
  // the host sizes an LMUL>1 `vmv.s.x` as an 8-member group while the VPU emits
  // one result beat, the `last` beat never arrives, the CII tag is never freed,
  // the ROB entry never clears and in-order commit stalls until the
  // "Pipeline has hung" assertion fires. This was first seen on a `vmv.s.x` in
  // conv1d. The host's classification must therefore agree with the VPU's own
  // `o_ignore_lmul` (vmv.x.s|vmv.s.x|vmv.s.f|vmv.f.s) and `o_ignore_dstincr`
  // (mask_only|reductop) — those two signals are the authoritative definition
  // and this list must be reviewed against them, not against the ISA manual.

  ---- 6. Whole-register moves, and the two illegal-instruction terms ----

  `vmv<n>r.v` (funct6 `0b100111` under OPIVI) takes its group size from the NREG
  field of its OWN encoding — `simm5 + 1`, legal values 1, 2, 4, 8 — and NOT from
  the vtype mirror. That is why it stays decodable while the mirror is poisoned:
  it cannot allocate a mis-sized group, because the size did not come from the
  mirror. Drive `v_emul` from NREG for this form, and exclude it from the
  vtype-dependency term below.

  `vill_trap(i)` is the OR of exactly two terms, and both are computable only
  here:

  - `vtype_in(i).vill && is_arith(i) && !is_whole_reg_move(i)` — a poisoned
    mirror reaching a vtype-DEPENDENT arithmetic op. VConfigUnit owns setting the
    poison; this module owns the "is it vtype-dependent" qualifier, because the
    whole-register exemption is an opcode property.
  - the vtype-derived destination EMUL exceeding `maxMembers`, which is how a
    widening op at a large LMUL presents (LMUL=8 widening asks for 16 registers).
    That is a reserved encoding and must trap. Do NOT clamp it silently: a
    clamped group would be allocated at 8 members while the coprocessor computed
    16, and the mismatch would surface as corrupt data much later.

  ---- 7. is_shared — the second execution resource ----

  //@req-spec-decode.b1
  //@req-spec-decode.b2
  //@req-spec-decode.b3
  `is_shared` marks an instruction needing BOTH the vector load/store unit and
  the coprocessor, and this module owns the field because it owns the
  `IQ_V_ALU` routing bit that the coprocessor half needs. Assert it exactly for a
  SEGMENTED load or store: `ls_in.is_mem && !ls_in.is_whole_reg &&
  ls_in.nf =/= 0`. When it is asserted, also set `iq_type(IQ_V_ALU)` on that
  lane — VLSDecode sets `IQ_V_LOAD`/`IQ_V_STORE`, so the uOP carries two queue
  bits, takes two issue slots and still occupies ONE ROB entry. The two halves
  rendezvous through the `pvtmp` group, which the vector mapper allocates
  all-or-nothing at rename; nothing about that allocation is decided here beyond
  raising the flag that requests it. Only RVV 1.0 instructions can be marked:
  the flag exists because the CII-attached coprocessor may need to read the
  LSU's intermediate results and vice versa, which is meaningless for a scalar
  uOP, so `is_shared` is false on every lane where neither `is_arith` nor
  `ls_in.is_mem` holds.

  // ===> TWO ENCODING TRAPS IN ONE THREE-TERM EXPRESSION. (a) `nf` is
  // NFIELDS-1, so "segmented" is `nf =/= 0`, not `nf > 1`; testing `> 1` marks
  // no 2-field access shared and testing the count marks every access shared.
  // (b) The whole-register forms `vl<n>r.v`/`vs<n>r.v` REUSE the nf field to
  // encode NREG-1, so `vl8r.v` carries nf = 7 and would be misread as an
  // 8-field segmented access — dispatching a coprocessor half that transposes
  // nothing and never completes. This is why `nf` arrives from VLSDecode's
  // descriptor together with `is_whole_reg` instead of being sliced out of the
  // instruction word here: the two bits must be read by the same decoder or
  // they will eventually disagree.

  //@req-spec-issue.c4
  A vector ARITHMETIC instruction is NEVER `is_shared`, in any configuration.
  The coprocessor manages its own internal resources for arithmetic, so no
  second execution resource is dispatched and no `pvtmp` rendezvous group is
  needed; shared handling applies only to vector load/store. Express this as the
  `is_mem` term in the assignment above rather than as a downstream assertion,
  so that an arithmetic lane cannot set the bit even transiently.

  ---- 8. What this module deliberately does NOT write ----

  Anything not listed above passes through from `uop_in` unchanged. These fields
  in particular have another owner, and writing one here would give it two writers
  whose disagreement no test would localize:

  - `v_eew` and `v_seg_nf` — VLSDecode's, part of the memory access descriptor.
    An arithmetic op's element width is SEW, reachable from `vconfig`.
  - `vconfig` — VConfigUnit's, which snapshots the mirror it also selects.
  - `uses_ldq` / `uses_stq` — VLSDecode's.
  - `is_vl_producer` and everything about VL — VsetDecode's and ALUUnit's. No
    arithmetic op is a VL producer.
  - all `pv*` fields, `stale_pvdest` and `pvtmp` — the vector mapper's, at rename.
    This module writes only the LOGICAL specifiers.
  - `v_split_*` and the element cursor — nOP.v-scoped, inert on an OP.v, the
    vector LS AGEN's; their staying zero here is what makes a stray reader's
    mistake consistent rather than intermittent.
  - `is_unique` / `flush_on_commit` — DecodeUnit's, and needed by no arithmetic
    op. Marking arithmetic unique would serialize the whole vector pipeline.

  // CORRECTED, and the old text was factually false rather than merely stale: it
  // said no mask-enable bit exists and that the `IQ_V_ALU` slot's `pvm` gate
  // reads `inst(25)` for itself. `MicroOp` DOES carry `v_is_masked`, its delta
  // forbids re-deriving it from `inst(25)` at a use site, and `MicroOp` does not
  // carry `inst` to issue at all — `VecIssueSlot` reads the field. VecDecode
  // already assigns the arithmetic side of that field to this module, so it is
  // written in part 3b above rather than disowned here.

  ---- 9. Trace ----

  Emit one guarded trace line per valid recognized lane through the shared
  VecTrace package using its DECODE-STAGE entry point,
  `traceDecode(module, event, ftq_idx, pc_lob, extra)` — module `VDecode`, event
  `arith`, carrying `v_emul`, `lvd`, `is_shared` and the three `v_uses_vs*` bits
  — plus one on `vill_trap` naming which term fired. Gated on the `vecTrace`
  plusarg and `!reset`, off by default.

  // ===> NOT the `rob_idx`-keyed entry point, which an earlier draft of this
  // section named. THERE IS NO `rob_idx` AT DECODE: the ROB entry is allocated at
  // DISPATCH, so this module has none to tag a line with, and a line claiming
  // `rob=0` would silently alias with real ROB entry 0 in every grep.
  // `traceDecode` is keyed on `ftq_idx`/`pc_lob` — the identifier this stage does
  // have — and prints `rob=?` in that position, which is why VecDecode,
  // VLSDecode, VsetDecode and VConfigUnit all use the same variant. Correlating
  // a decode line with a later pipeline line is then a two-step join through the
  // dispatch line; that is the honest cost of the ROB entry not existing yet.

  There are no unit tests in this project and validation is end-to-end VCS plus
  Whisper cosim, where a wrong decode field is first visible as a mismatch
  hundreds of cycles downstream; these two lines are what localize it.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Single-cycle combinational, zero pipeline stages, `coreWidth` independent copies,
and it sits directly in the DECODE stage critical path. Constraints that follow:

- No register, no memory, no multiplier and no divider. The deepest term is the
  VtypeTable evaluation, a shift-and-compare tree by construction; the widening
  adjustment is an increment of `vsew`, not a multiply.
- `v_emul` must be available in the same cycle as the `vtype` it derives from. It
  feeds atomic group rename, the design's top timing risk (up to
  `coreWidth * maxMembers` PRN allocations in one cycle), so this module must add
  no depth ahead of it. If decode timing fails, the fix is in the mapper, never a
  pipeline stage here: a registered stage would break the one-uOP-per-instruction
  -per-cycle property the rest of the front end assumes.
- With `usingRVV = false` this module is not elaborated at all, so it contributes
  nothing to the baseline build's area or timing.
<|end_perf|>

<|begin_dependencies|>
MicroOp — `uop_in`/`uop_out` are `new MicroOp()`, and every field this module
writes is one the MicroOp delta adds (`is_vec`, `is_shared`, `lvd`, `lvs1`,
`lvs2`, `lvs3`, `lvm`, `v_emul`, `v_is_masked`, `v_uses_vs1`, `v_uses_vs2`,
`v_uses_vs3`) or widens (`dst_rtype`, `lrs1_rtype`, `lrs2_rtype` to 3 bits, which
`RT_VEC` requires). The three `v_uses_vs*` bits are the delta's D11 addition and
this module is one of their two producers; their consumers (the vector mapper and
`VecIssueSlot`) reach them through the uop, never through this node.

VtypeTable — `decode` and `emul`, the ONLY EMUL derivation path. This module must
not compute a group size any other way: VConfigUnit, VsetDecode and ALUUnit bind
to the same functions, and three copies of the LMUL rule is three chances to
disagree.

VecTrace — the guarded trace helpers.

It also binds by name to `trait ScalarOpConstants` for `RT_VEC`/`RT_FIX`/
`RT_FLT`/`RT_ZERO`/`RT_X`, `IQ_V_ALU`, `IQ_SZ`, `FC_ALU` and `FC_SZ`, and to
`freechips.rocketchip.rocket.VType` for `vtype_in`. Neither appears in this
node's `depends_on:` — the first because baseline `MicroOp` already mixes it in,
the second because rocket is upstream and unmodified.

Instantiates nothing. Its parent VecDecode supplies two inputs from its siblings
(`vtype_in` from VConfigUnit's prefix select, `ls_in` from VLSDecode's access
descriptor), which are wiring edges owned by the parent rather than `depends_on:`
edges here. `uop_out` is consumed by the DecodeUnit delta, which merges the vector
fields back into the scalar decode result.
<|end_dependencies|>

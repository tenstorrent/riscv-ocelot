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
  VecDecode — the RVV decode CONTAINER: it holds the four decode-stage units and
  MERGES their per-lane results into one outgoing uOP per lane.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/decode/VecDecode.scala,
  package boom.v4.vec.generated.decode. group vec_decode.
  depends_on MicroOp, VecBundles, VecTrace (and, unavoidably, VtypeTable — see
  the dependencies section, which records that as a map defect rather than
  hiding it). Instantiated once, as `vdec`, by VecPipeline.
  Instantiates: VDecode as `arith`, VLSDecode as `ls`, VsetDecode as `vset`,
  VConfigUnit as `vcfg`. All four are single instances with `coreWidth`-wide
  ports; there is no per-lane instantiation anywhere in decode.

  ===> THE MERGE IS FIELD-WISE, NOT LANE-WISE. Do NOT implement this module as a
       per-lane mux that selects ONE child's `uop_out`. On a SEGMENTED vector
       load/store lane, `arith` writes `is_shared` and the `IQ_V_ALU` routing bit
       while this module writes the whole memory field group for the same lane —
       a whole-uop select would silently drop one of the two, and the surviving
       half would issue to one queue, wait forever for a rendezvous partner that
       was never dispatched, and hang at commit. Field ownership is disjoint (the
       table in logic part 3), so a field-wise union is well defined and
       order-independent. That order-independence is bought by part 2's single
       zeroing and by every writer of `iq_type`/`fu_code` being a SET, never a
       clear.

  ===> `MicroOp.vconfig` IS WRITTEN ON EVERY LANE, INCLUDING PURELY SCALAR ONES.
       This is the highest-risk wire in the node. `VConfigUnit` snapshots the
       vtype mirror per `br_tag` from `ren_br_vconfig`, which is
       `ren2_uops(w).vconfig` — the BRANCH's carried snapshot. Branches are
       overwhelmingly scalar. If `vconfig` were written only on `is_vec` lanes,
       every snapshot taken on a scalar branch would be an undriven or stale
       value, and every mispredict restore would install garbage into the mirror,
       mis-sizing PRN groups for surviving younger vector uOPs with no
       misprediction left to recover from. Unconditional, every lane, every
       cycle.

  Governing spec anchors: frontend.rst `vector-rvv-decode` (including "CII
  Shared Instruction Decoding", "VSET Special Handling", `vset-dual-dest`),
  glossary.rst `glossary-terms` (uOP / OP.v / nOP.v). Plan v2 section 5 ground
  rules 1 (usingRVV), 2 (one uOP per instruction), 9 (rocket owns the
  architectural vector CSRs), 11 (guarded tracing).
*/

<|begin_module|>

  <|begin_parameters|>
  `usingRVV` — the Scala `Boolean` from `BoomCoreParams`, NOT a hardware `Bool`
  and NOT rocket-chip's `usingVector`. This module and all four children are
  elaborated only when it is true; with vectors off VecPipeline is not
  instantiated at all, so nothing here exists — absent, not tied off — and a
  non-vector build emits RTL bit-identical to pre-Caracal BOOM v4 (plan gate
  (f)). No port here is defaulted or tied when the switch is off, because no
  port here exists when the switch is off.

  `coreWidth` — the number of decode lanes, from `HasBoomCoreParameters`.
  Default 3 (`MediumBoomV4VectorConfig`); legal 1..4. Every port below is a
  `Vec(coreWidth, ...)` and lane 0 is the OLDEST in program order. This module
  passes `coreWidth` to nothing: all four children take it from the same
  implicit `Parameters`, which is what keeps the lane indexing of their ports
  aligned with each other and with `dec_uops_in`.

  `maxBrCount` — BOOM's branch-tag count, needed only to size the `ren_br_tags`
  pass-through that reaches `vcfg`.

  `maxMembers` — from VectorParams, fixed at 8. Used as the reserved-encoding
  bound on a memory access's destination group size in logic part 5, never
  written as a literal.

  No parameter of this module's own. It introduces no knob: a container that
  added a configuration point would be a place for the four children to be
  configured inconsistently.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel and hierarchy.yaml defaults: single
  `core_clk` domain, posedge-triggered, with an ACTIVE-HIGH SYNCHRONOUS
  `core_reset`. This module declares NO register and no memory of its own — all
  decode-stage state in the machine lives in `vcfg` — so both are used here only
  to reach the children and to gate the trace statements.

  There is NO handshake, NO `ready` and NO `busy` in either direction. Decode
  cannot be stalled by the vector subsystem and this module exports nothing that
  could stall it; stalling decode remains `DecodeUnit`'s and `BoomCore`'s
  business. A `busy` out of any vector node is a design-invariant violation.

  ---- Decode feed, per lane ----

  - `dec_insns`    — Input `Vec(coreWidth, UInt(32.W))`, the raw instruction
                     words. Fanned to `arith.inst`, `ls.lanes(i).inst` and
                     `vset.inst` unchanged, and read directly by this module for
                     the register-specifier extraction of logic part 5.
  - `dec_valids`   — Input `Vec(coreWidth, Bool)`, the lane holds a decoded
                     instruction. A qualifier, not a handshake.
  - `dec_fire`     — Input `Vec(coreWidth, Bool)`, the lane ADVANCES out of
                     decode this cycle (BOOM's `dec_fire(w)`). Wired to
                     `vcfg.dec_fire` and to nothing else.
                     // ===> `dec_valids` IS NOT A SUBSTITUTE. A decode bundle
                     // can fire partially, so a mirror keyed on validity would
                     // absorb a `vset` from a lane that did not advance and then
                     // see the same `vset` re-presented next cycle. This signal
                     // is NOT currently on the `vec_pipeline_io` interface —
                     // flagged in the authoring report as a required addition.
  - `dec_uops_in`  — Input `Vec(coreWidth, new MicroOp())`, the uOP as baseline
                     `DecodeUnit` decoded it. The merge BASE.

  ---- Decode results, per lane ----

  - `dec_uops_out` — Output `Vec(coreWidth, new MicroOp())`, the merged uOP.
  - `dec_vec_illegal` — Output `Vec(coreWidth, Bool)`, raise illegal-instruction
                     on this lane at decode. The single aggregation point for
                     every decode-stage vector legality term (logic part 6).
  - `dec_vl_imm`, `dec_vl_imm_valid` — Output, width `vecVLSz` (9 bits, per the
                     corrected VectorParams sizing) and 1 bit. The decode-computed
                     VL of a front-end-only `vsetivli` and its qualifier, leaving
                     for the RENAME-cycle VL-RF write. They must be REGISTERED by
                     VecPipeline through the same decode-to-ren2 stage as their
                     uOP and qualified there by `dis_fire(w)`; running them a
                     cycle ahead of the uOP is the shape of the M1 free-list
                     double-free.

  ---- Pass-throughs that exist only to reach `vcfg` ----

  This module reads none of these; each is wired straight to the correspondingly
  named `vcfg` port. They surface on this boundary only because hierarchy.yaml
  places `vcfg` inside this node: `ren_br_tags`
  (`Vec(coreWidth + 1, Valid(UInt(brTagSz.W)))`, entry 0 tied invalid, entry
  `w + 1` for rename lane `w`), `ren_br_vconfig` (`Vec(coreWidth + 1, VType)`,
  index-aligned with it, entry `w + 1` being `ren2_uops(w).vconfig`), `brupdate`,
  `rollback`, `com_valids`, `com_is_vset`, `com_vtype`, and the two check-only
  inputs `csr_vtype` and `rob_empty`.

  Deliberately absent, and a reviewer should reject them: any `vtype`/`vl`
  output other than `dec_vl_imm`, any execute-time vtype write port, any VRF
  port, any element-cursor or nOP.v-scoped port, and any `stall`/`ready`/`busy`.
  <|end_ports|>

  <|begin_logic|>
  Single-cycle, purely combinational apart from the state inside `vcfg`. One uOP
  in, one uOP out, per lane, per cycle. Nothing here expands, cracks, buffers or
  reorders a uOP: element expansion into nOP.v happens only in the vector LS
  AGEN, and a decode-stage container that emitted two uOPs for one instruction
  would break the property every stage after it assumes.

  ---- 1. Recognition: the three-way opcode split ----

  //@req-spec-decode.a3
  RVV 1.0 opcodes reach exactly three of the four children, partitioned by opcode
  so that no instruction is claimed by two of them: OP-V (`inst(6,0)` = 0x57)
  with `funct3` other than `0b111` is vector ARITHMETIC and is `arith`'s;
  OP-V with `funct3` = `0b111` is OPCFG, the `vset` class, and is `vset`'s;
  LOAD-FP (`0b0000111`) and STORE-FP (`0b0100111`) with a vector `width` encoding
  are vector memory and are `ls`'s. Each child performs its own recognition from
  the instruction word — this module does not pre-classify and hand out enables,
  because a fourth copy of the opcode map is a fourth thing to keep in agreement.
  What this module owns is the union: `rvv_recognized(w)` is
  `arith.is_arith(w) || ls.lanes(w).desc.is_vls || vset.dec_is_vset(w)`, and it
  is the term that gates part 2's zeroing, part 5's memory field writes and the
  trace. Assert that at most one of the three predicates holds per lane; they are
  mutually exclusive by opcode, so the assertion is a check on the children's
  recognition rather than behaviour of its own.

  // `rvv_recognized` is NOT the same thing as DecodeUnit's `v_legal` gate. That
  // gate decides whether the RVV extension is enabled and reachable at all
  // (including rocket's `mstatus.VS = Off` trap, which is the CSR file's and is
  // not evaluated anywhere in this node). This term only says which of the three
  // decoders owns the lane.

  ---- 2. The single zeroing, and why it is exactly one assignment ----

  On a lane where `rvv_recognized` holds, `iq_type` and `fu_code` are cleared to
  all-zero ONCE, on the merge base, BEFORE any child's field writes are applied.
  Baseline `DecodeUnit` has no table entry for an RVV opcode, so the scalar
  routing bits it produced for such a lane are meaningless; left in place they
  would dispatch a vector uOP to a scalar issue queue and hand it to a scalar
  functional unit.

  // ===> THE ZEROING LIVES HERE AND NOWHERE ELSE, AND THAT IS A CONTRACT WITH
  // ALL THREE DECODERS. `VDecode` sets only `IQ_V_ALU`/`FC_ALU` and deliberately
  // does not clear the others; `VsetDecode` drives `IQ_ALU`/`FC_ALU`; this module
  // sets `IQ_V_LOAD`/`IQ_V_STORE`. Because the field arrives already zero, every
  // one of those writes is a SET and the composition is order-independent. If a
  // child also cleared the field, the result would depend on the order this
  // module chained its children — a last-connect ordering dependence that
  // survives review and breaks silently on the next reorder. `VsetDecode`'s
  // "all-clear" for the front-end-only shape is a no-op re-assertion of this
  // zeroing, not a second mechanism.

  On a lane where `rvv_recognized` is false, nothing in this module writes any
  field: `dec_uops_out(w)` is `dec_uops_in(w)` verbatim, with the ONE exception
  of `vconfig` (part 4). A scalar lane must leave decode bit-identical to a
  vectors-off build apart from that snapshot.

  ---- 3. The merge, and the field-ownership table ----

  Build `dec_uops_out(w)` as `dec_uops_in(w)`, then apply part 2's zeroing, then
  take each field from its single owner. The children's `uop_out` bundles are
  read FIELD-WISE. Ownership is disjoint, so no arbitration, priority or
  last-connect ordering is needed anywhere in the merge:

    from `arith.uop_out(w)`  — `is_vec`, `dst_rtype`, `lvd`, `lvs1`, `lvs2`,
        `lvs3`, `lvm`, `ldst`, `lrs1`, `lrs1_rtype`, `lrs2_rtype`, `v_emul`, and
        the `IQ_V_ALU`/`FC_ALU` bits, on ARITHMETIC lanes; plus `is_shared` and
        the `IQ_V_ALU` bit on a SEGMENTED MEMORY lane, which is the only lane
        where two owners write the same uOP.
    from `vset.uop_out(w)`   — `is_vl_producer`, `dst_rtype`, `lrs1`/`lrs2` and
        their rtypes, `is_unique`, `flush_on_commit`, the `IQ_ALU`/`FC_ALU` bits,
        and the integer control selection (`fcn_op`, `fcn_dw`, `op1_sel`,
        `op2_sel`, `imm_sel`, `imm_rename`, `csr_cmd`), on `vset` lanes.
    from `ls.lanes(w).desc`  — the whole memory field group, mapped by part 5.
    from `vcfg.dec_vconfig(w)` — `vconfig`, on EVERY lane (part 4).

  Fields NO owner writes stay as `dec_uops_in(w)` produced them. In particular
  the nOP.v-scoped `v_split_*` fields, the element cursor and the nOP.v target
  PRN / byte-offset fields are driven to ZERO on every recognized lane rather
  than left undriven, so that a stage reading them off an OP.v by mistake gets a
  consistently wrong answer instead of an intermittently plausible one; the
  MicroOp delta asks for exactly that. All `pv*` fields, `stale_pvdest` and
  `pvtmp` are the vector mapper's at rename and are not touched here.

  ---- 4. `vconfig`: the unconditional write ----

  `dec_uops_out(w).vconfig := vcfg.dec_vconfig(w)`, for every lane, with
  `vset.keep_vl_illegal(w)` ORed into its `vill` field. One expression, one
  writer, no `is_vec` qualifier and no `rvv_recognized` qualifier.

  The `vill` fold is done here rather than left to `vcfg` alone so that the uOP
  which must trap carries its own poison and does not depend on a neighbour's
  fold; `VsetDecode` states the same obligation from its side, and both
  statements describe THIS assignment rather than two mechanisms.

  // ===> THE `DecodeUnit` DELTA MUST MERGE `vconfig` UNCONDITIONALLY TOO. That
  // delta gates its merge of the vector MicroOp fields on its `v_legal` /
  // RVV-opcode term. `vconfig` must be OUTSIDE that gate. If it is inside,
  // scalar branches carry no snapshot, `ren_br_vconfig` is garbage, and the
  // per-`br_tag` restore corrupts the mirror on the first mispredicted branch of
  // any program that has ever executed a `vset`. Strictly only uops that can
  // allocate a `br_tag` need it; writing it on every lane is cheaper than the
  // predicate and cannot be got wrong by a later change to which uops allocate
  // tags.

  ---- 5. The memory field group: descriptor to uOP ----

  `VLSDecode` emits a `VLSAccessDesc` and NO uOP, so mapping the descriptor onto
  the uOP is this module's, on a lane where `ls.lanes(w).desc.is_vls` holds:

  Class carriage, straight across, one field per descriptor flag: `v_mop` from
  `desc.mop`, and `v_is_unit_stride`, `v_is_strided`, `v_is_indexed`,
  `v_is_segment`, `v_is_whole_reg`, `v_is_mask`, `v_is_ff` from the
  correspondingly named descriptor flags. These ride the uOP because none of
  their consumers (`VecLsu`, `VecElemAgen`, `VecRangeAgen`, `VecIdxGen`,
  `VecBeatExpander`) depends on `VLSDecode`; no consumer may re-decode
  `uop.inst`.

  `v_seg_nf := Mux(desc.is_segment, desc.nf, 0.U)`.
  // ===> QUALIFY IT. `nf` is NFIELDS-1 for a segmented access and NREG-1 for
  // `vl<n>re<eew>`, so passing `desc.nf` through raw makes `vl8re64` present as
  // an 8-field segmented access to any reader of `v_seg_nf`.

  `v_is_masked := !desc.vm && !desc.is_whole_reg && !desc.is_mask`.
  // ===> THE SENSE IS INVERTED. RVV's `vm` bit is 1 for UNMASKED, so the field
  // is the COMPLEMENT of `inst(25)`, and copying `desc.vm` into it gives every
  // masked op an unmasked descriptor and vice versa. The two non-maskable
  // unit-stride forms are forced false so a reader never consults `pvm` for
  // them. The arithmetic side of this field is `arith`'s.

  The two element widths, which is where an indexed access is got wrong:
  `v_eew := Mux(desc.eew_is_index, vcfg.dec_vconfig(w).vsew, desc.eew)` and
  `v_idx_eew := desc.eew` (meaningful only when `v_is_indexed`).

  ---- Source-use bits (D11), and the lane rule that makes them safe ----

  Copy the three source-use bits straight across from whichever child claimed the
  lane: `v_uses_vs1/2/3 := desc.uses_vs1/2/3` on a memory lane, and the
  correspondingly-named outputs of `arith` on an arithmetic lane.

  ===> AND DRIVE THEM FALSE ON EVERY LANE THAT CLAIMS NEITHER — the `vset` lanes
       and the front-end-only lanes included. This is the whole point of the
       field. A `vset` is a scalar uOP with `is_vec` clear; it encodes no vector
       source, so leaving these bits at their incoming (don't-care) value would
       hand the vector mapper a request to rename `lvs*` for an instruction that
       has none, and hand the issue slot a `used` bit that makes it wait on
       whatever `v0` currently maps to. The failure is a HANG, not a stall: if
       that producer's group-done already fired before the slot captured its
       member-ready bits, no future group-done clears them.
       The zeroing rule stated in part 1 for `iq_type`/`fu_code` therefore extends
       to these three bits and to `v_is_masked`: cleared ONCE on the merge base
       for every lane, so each child's write is a SET and composition stays
       order-independent.
       // Note this is the container's job precisely because no child sees the
       // lanes it did not claim. VLSDecode cannot zero an arithmetic lane's bits
       // and VDecode cannot zero a vset's.
  // `v_eew` is the DATA width by definition. For an indexed access the
  // instruction's `width` field describes the INDEX elements and the data width
  // is `vtype.vsew`; `VLSDecode` cannot see vtype and correctly refuses to guess,
  // which is why this combination happens here, in the one module that has both
  // the descriptor and the vtype snapshot.

  //@req-spec-decode.a3
  The DESTINATION GROUP SIZE of a memory access, which is the number the vector
  mapper allocates atomically at rename and the one quantity in the descriptor
  path that is not a bit-slice. Evaluate `VtypeTable.decode` on this lane's
  selected `vconfig` once, then:
    - whole-register: `v_emul := desc.nregs`, taken from the instruction's own
      NREG field and NOT from vtype, which is why the form stays decodable while
      the mirror is poisoned;
    - mask (`vlm.v`/`vsm.v`): `v_emul := 1.U`, one byte-granular register;
    - indexed: `v_emul := VtypeTable.emul(info, info.vsew)`, i.e. LMUL. The DATA
      group is LMUL members; it is the INDEX group that scales with the index
      width, and the index group's size is not carried on the uOP at all — the
      coprocessor and `VecIdxGen` name index members by offset;
    - otherwise: `v_emul := VtypeTable.emul(info, desc.eew)`, i.e.
      `LMUL * EEW / SEW`, clamped to at least 1 by that function;
  and for a SEGMENTED access multiply the result by `desc.nf + 1`, because a
  segmented access's destination is NFIELDS consecutive groups and the mapper
  must allocate all of them as one atomic request.

  //@req-spec-decode.a3
  RESERVED-ENCODING CHECK, this module's own: if the derived destination group
  size exceeds `maxMembers`, the encoding is reserved (RVV 1.0's
  `EMUL * NFIELDS <= 8` constraint, and the `LMUL * EEW / SEW <= 8` constraint
  for a plain access) and the lane raises illegal-instruction. Do NOT clamp it:
  a clamped group would be allocated at 8 members while the coprocessor and the
  agens computed the true count, and the mismatch surfaces as corrupt data or a
  never-completing group-done many hundreds of cycles later. This check is the
  memory-side twin of `VDecode`'s widening-EMUL term and cannot live in
  `VLSDecode`, which reads no vtype.

  Routing and the queues: `is_vec := true.B`; `iq_type(IQ_V_LOAD) := !is_store`
  and `iq_type(IQ_V_STORE) := is_store`, as SET operations onto the zeroed field
  so that a segmented access carries `IQ_V_ALU` from `arith` alongside and takes
  two issue slots while occupying ONE ROB entry; `uses_ldq := !is_store` and
  `uses_stq := is_store`, one placeholder entry per vector memory instruction for
  ordering and commit only — the cracked element accesses go to `VecLsu`'s own
  six queues and never to an LDQ or STQ slot.

  Register specifiers, extracted here because `VLSDecode` writes no uOP and
  `VDecode` does not claim these lanes:
    - a LOAD writes a vector group: `lvd := inst(11,7)`, `dst_rtype := RT_VEC`;
    - a STORE reads one: `lvs3 := inst(11,7)`, `dst_rtype := RT_X`. The store
      data group is an explicitly ENCODED third source and is `pvs3`, never
      `stale_pvdest` — the two remain separate fields naming separate groups;
    - the base address is always the integer `rs1`: `lrs1 := inst(19,15)` with
      `lrs1_rtype := Mux(inst(19,15) === 0.U, RT_ZERO, RT_FIX)`;
    - `inst(24,20)` is THREE different things by class and must be decoded as
      such: the STRIDE integer register for a strided access
      (`lrs2 := inst(24,20)`, `lrs2_rtype := Mux(... === 0.U, RT_ZERO, RT_FIX)`),
      the INDEX vector group for an indexed access (`lvs2 := inst(24,20)`,
      `lrs2_rtype := RT_X`), and the `umop` sub-opcode for a unit-stride access,
      which names no register at all (`lrs2_rtype := RT_X`);
    - `lvm := 0.U`, the mask being architecturally `v0`, so it reads through the
      same map-table path as any other source.

  // ===> THE x0 CONVERSION IS MANDATORY ON BOTH INTEGER SOURCES.
  // `rename-stage.scala:109` asserts
  // `!(r_valid && lrs1_rtype === RT_FIX && lrs1 === 0.U)`, and a vector load
  // based at `x0` or strided by `x0` is legal RVV. The same omission fired on
  // `vmv.s.x v12, zero` in the arithmetic path; mirror the scalar decoder
  // (`decode.scala:505`) rather than rediscovering it a third time.

  `is_vl_producer := desc.is_ff`. `vleff` writes its trimmed element count to the
  VL register file and wakes `pvl` in its dependents, so it IS a VL producer and
  the VL-RF write port for it is declared from day one even though it does not
  fire until the fault-trim path lands.
  // This contradicts `VLSDecode`'s prose, which keeps `is_vl_producer` clear for
  // `vleff` on the strength of a hierarchy.yaml comment that has since been
  // corrected. `VLSDecode` writes no uOP, so this module is the writer and
  // settles it in favour of the corrected comment, loadstore.rst and
  // spec-lsu.g6/g7. Flagged in the authoring report.

  ---- 6. `dec_vec_illegal`: the aggregation ----

  `dec_vec_illegal(w)` is the OR of exactly FOUR terms, qualified by
  `dec_valids(w)`:
    - `ls.lanes(w).illegal` — a reserved vector memory encoding;
    - `arith.vill_trap(w)` — a poisoned mirror reaching a vtype-DEPENDENT
      arithmetic op, or an arithmetic destination EMUL past `maxMembers`;
    - `vcfg.dec_vtype_illegal(w)` — a poisoned mirror reaching a lane whose
      `dec_uses_vtype` is set;
    - this module's memory-side EMUL-bound term from part 5.
  And nothing else. `vset.keep_vl_illegal(w)` is deliberately NOT a term here:
  the reserved keep-VL encoding SETS `vill`, it does not raise
  illegal-instruction, and the trap arrives later on the first younger
  vtype-dependent uOP.

  On an arithmetic lane the second and third terms describe the same condition
  and both fire. That redundancy is deliberate and harmless — same lane, same
  cycle, same value — whereas dropping either one is not, because `arith` owns
  the whole-register-move exemption and `vcfg` owns the poison.

  `mstatus.VS = Off` is NOT evaluated here and no term for it may be added: that
  gate is rocket's `CSRFile`, reached through the baseline decode path.

  ---- 7. The four seams this module exists to close ----

  Each of the following is a wire the children specified from one side only. This
  module is where they are joined, and the joins are named so a reviewer can
  check both ends.

  (a) `ls` to `arith`. `arith.ls_in(w)` takes `is_mem := desc.is_vls`,
      `is_load := desc.is_vls && !desc.is_store`, `is_store := desc.is_store`,
      `is_whole_reg := desc.is_whole_reg` and the RAW `nf := desc.nf`
      (= `inst(31,29)`, NFIELDS-1). `arith` needs all five to compute
      `is_shared` as `is_mem && !is_whole_reg && nf =/= 0`.
      // The whole point of routing `nf` through the descriptor instead of
      // letting `arith` slice it out of `inst` is that `nf` and `is_whole_reg`
      // must be read by the SAME decoder: `vl8r.v` carries `nf = 7` and a
      // second reader would eventually mark it an 8-field segmented access,
      // dispatching a coprocessor half that transposes nothing and never
      // completes.

  (b) `vcfg` to `arith`. `arith.vtype_in(w) := vcfg.dec_vconfig(w)` — the
      self-EXCLUSIVE value on a consumer lane, which is what an arithmetic op
      needs, and the same wire the uOP snapshot comes from, so the EMUL `arith`
      derives and the vtype the coprocessor is handed cannot disagree.

  (c) `vcfg` to `vset`, and back. `vset.dec_vconfig(w) := vcfg.dec_vconfig(w)`
      (self-INCLUSIVE on a `vset` lane, which is what makes a `vset`'s own
      snapshot its NEW vtype) and `vset.prev_vtype(w) := vcfg.dec_prev_vconfig(w)`,
      the SELF-EXCLUSIVE value. That second output is an ADDITION to
      `VConfigUnit`'s port list: it is the per-lane INPUT side of the prefix
      `scanLeft` that already computes `dec_vconfig`, so exporting it costs one
      wire and no logic.
      // The identity that bounds the cost of that addition:
      // `dec_prev_vconfig(w)` equals `dec_vconfig(w-1)` for every `w >= 1` — on a
      // `vset` lane the self-inclusive select IS the running value out, and on a
      // non-`vset` lane in and out are the same value — so the only genuinely new
      // information is lane 0's, the mirror register. Exporting the whole vector
      // is still preferred over reconstructing it here, which would put a second
      // copy of the prefix convention in a second file.

  (d) `vset` to `vcfg`. The shape and field outputs go across by name —
      `dec_is_vset`, `dec_vtype_is_imm`, `dec_vtype_imm`, `dec_avl_imm`,
      `dec_is_vsetivli` — plus `keep_vl_illegal`, which is a second ADDITION to
      `VConfigUnit`'s inputs: it must be ORed into that lane's contribution to
      the mirror's `vill`, or the reserved keep-VL encoding poisons the uOP but
      not the mirror and younger ops decode against a configuration the
      architecture calls illegal.
      // ===> THIS IS NOT A COMBINATIONAL LOOP, AND IT IS WORTH PROVING RATHER
      // THAN ASSUMING. `keep_vl_illegal(w)` depends on `prev_vtype(w)`, which
      // depends on the mirror and on lanes `0..w-1` only — including their own
      // `keep_vl_illegal`. The dependence is strictly triangular in the lane
      // index, so the prefix chain remains acyclic; `dec_vtype_imm` is pure
      // bit-slicing of `inst` and closes no loop either. A generator that
      // implements the fold as a whole-vector reduction instead of per-lane
      // inside the `scanLeft` WOULD create a loop.

  `dec_vl_imm_valid` is where the fifth seam is settled: `vcfg` raises its own
  qualifier on `dec_is_vsetivli`, and this module publishes
  `dec_vl_imm_valid(w) := vcfg.dec_vl_imm_valid(w) && vset.frontend_only(w)`.
  The distinction is load-bearing: a `vsetivli` with `rd != x0` is NOT front-end
  only — `rd` must receive the new VL and the front end has no integer-RF write
  port — so it takes the ALU path and its VL RF write comes from the ALU
  writeback. Qualifying the rename-cycle write by shape alone would give that
  encoding TWO VL writers for one instruction, on two statically partitioned and
  deliberately unarbitrated ports.

  `vcfg.dec_uses_vtype(w)` is driven by this module as
  `(arith.is_arith(w) && !is_whole_reg_move(w)) ||
   (desc.is_vls && !desc.is_whole_reg && !desc.is_mask)`, where
  `is_whole_reg_move` is the `vmv<n>r.v` encoding (OP-V, `funct3` = OPIVI,
  `funct6` = `0b100111`).
  // The whole-register forms MUST be exempt: `vmv<n>r.v`, `vl<n>r.v` and
  // `vs<n>r.v` take their group size from their own NREG field, so the mis-sized
  // group hazard cannot arise for them — and they are how vector state is saved
  // and restored, so trapping them under poison would leave the machine unable
  // to recover from a `vill` at all. `is_whole_reg_move` is a two-comparator
  // predicate that `VDecode` also computes internally for its own `vill_trap`;
  // re-deriving it here is a duplicated comparator, not a duplicated decoder,
  // and the alternative was adding an output to an already-written child.
  // Flagged in the authoring report: the right long-term fix is for `VDecode` to
  // export it.

  ---- 8. Assertions and trace ----

  Assert, per lane and only under `dec_valids(w)`: at most one of the three
  recognition predicates; `iq_type` non-zero on every recognized lane EXCEPT
  `vset.frontend_only`, which legitimately issues nowhere; `is_shared` implies
  the lane carries both `IQ_V_ALU` and one of `IQ_V_LOAD`/`IQ_V_STORE`;
  `v_emul` in 1..`maxMembers` on any lane with `is_vec` set and no illegal term;
  and the nOP.v cursor fields zero on every recognized lane. These are checks,
  not behaviour: deleting every one leaves the emitted datapath bit-identical.

  There are no unit tests in this project — validation is end-to-end VCS plus
  Whisper cosim — so emit guarded trace lines through the shared `VecTrace`
  package using the `traceDecode(module, event, ftq_idx, pc_lob, extra)` variant,
  NEVER the `rob_idx` one: the ROB entry is allocated at DISPATCH, so no
  decode-stage caller has a `rob_idx`, and a line claiming `rob=0` would alias
  with real ROB entry 0 in every grep. Two events and no more, both gated on the
  `vecTrace` plusarg and `!reset`, off by default: one per valid recognized lane
  carrying which decoder claimed it, the merged `iq_type`, `v_emul`, `v_eew` and
  `is_shared`; and one on `dec_vec_illegal` naming WHICH of part 6's terms fired.
  // The children each trace their own decision. This module's line is the only
  // record of the MERGED result, which is what a wrong `iq_type` or a dropped
  // `is_shared` is diagnosed from — and those are container bugs, invisible in
  // any child's trace.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Single-cycle combinational, zero pipeline stages, and it sits directly in the
DECODE critical path together with all four children. No pipeline stage may be
added anywhere in this node: the merged `v_emul` must be available in the same
cycle as the instruction because it feeds atomic group rename in the next, and
rename is the design's top timing risk (up to `coreWidth * maxMembers` PRN
allocations in one cycle). If decode timing fails, the fix is in the mapper.

The depth this module adds on top of its children is bounded by construction: a
per-lane field-wise mux tree, ONE `VtypeTable.decode` plus ONE `VtypeTable.emul`
per lane for the memory path, one small EMUL-by-`nf + 1` product (3 bits by 4, or
a shift-add of at most three terms), and the OR tree for `dec_vec_illegal`. That
`emul` evaluation is in PARALLEL with `arith`'s, not in series: both read the same
`dec_vconfig(w)`.

The one genuinely `coreWidth`-deep path is `vcfg`'s prefix scan, which this module
sits across twice — raw `vset` fields out, selected and previous vtypes back, and
`keep_vl_illegal` forward into the same scan (acyclicity argued in logic part
7(d)). Keep this module's contribution to that chain at zero: no comparator
between `vset`'s outputs and `vcfg`'s inputs, and no re-decode of `dec_vconfig`.

No register, no memory, no arbitration, no handshake, no back-pressure. With
`usingRVV = false` nothing here is elaborated, so the contribution to a
non-vector build's area and timing is exactly zero.
<|end_perf|>

<|begin_dependencies|>
VDecode, instance `arith` — vector arithmetic. This module supplies its
`vtype_in` from `vcfg` and its `ls_in` from `ls`, and consumes `uop_out`
field-wise plus `is_arith` and `vill_trap`.

VLSDecode, instance `ls` — the static access descriptor. It emits `desc` and
`illegal` and NO uOP, which is why logic part 5 is as long as it is: the
descriptor-to-MicroOp mapping and every vector memory register specifier are
this module's.

VsetDecode, instance `vset` — the three `vset` shapes. Two of its inputs
(`dec_vconfig`, `prev_vtype`) and two of its outputs (`keep_vl_illegal`,
`frontend_only`) are seams settled in logic part 7.

VConfigUnit, instance `vcfg` — the speculative vtype mirror and its committed
shadow. It requires TWO port additions this module depends on:
`dec_prev_vconfig` (per-lane, the prefix scan's input side) and
`keep_vl_illegal` (per-lane input, ORed into that lane's mirror `vill`
contribution).

MicroOp — `dec_uops_in`/`dec_uops_out` and every field the merge writes; the
memory field group is exactly what sections 4 and 4b of that delta add.

VecBundles — for the `vec_pipeline_io` field names this node's decode ports must
match on the seam above it.

VecTrace — the guarded `traceDecode` helper.

VtypeTable — `decode` and `emul`, for the MEMORY-side destination group size,
which no other node can compute: `VLSDecode` reads no vtype and `VDecode` claims
only arithmetic lanes. This binding is NOT in this node's `depends_on:` list and
should be — reported as a map defect rather than worked around, because the
alternative (a local LMUL/EEW rule) is a fourth copy of the EMUL derivation, the
one thing VtypeTable exists to prevent.

Also binds by name, without a `depends_on:` edge, to `ScalarOpConstants` for
`RT_VEC`/`RT_FIX`/`RT_ZERO`/`RT_X`, `IQ_V_LOAD`/`IQ_V_STORE`/`IQ_V_ALU`, `IQ_SZ`
and `FC_SZ` (baseline `MicroOp` already mixes it in), and to
`freechips.rocketchip.rocket.VType` for every vtype-valued port (rocket is
upstream and unmodified, and the mirror must hold the same encoding the
architectural CSR does).

Instantiated once, as `vdec`, by VecPipeline, which owns: the construction of
`ren_br_tags`/`ren_br_vconfig` from `ren2_uops`, the registering of `dec_vl_imm`
through the decode-to-ren2 stage and its qualification by `dis_fire(w)`, and the
routing of `dec_uops_out`/`dec_vec_illegal` back onto `vec_pipeline_io`. The
`DecodeUnit` delta is the other end of the `dec_uops_out` merge and carries the
`vconfig`-outside-the-`v_legal`-gate obligation stated in logic part 4.
<|end_dependencies|>

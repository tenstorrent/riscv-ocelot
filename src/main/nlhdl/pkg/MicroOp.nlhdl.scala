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
  MicroOp — DELTA SPEC. This file is NOT a description of BOOM's MicroOp
  bundle.
*/

  It describes only the vector fields Caracal ADDS to the existing
  `class MicroOp` in src/main/scala/v4/common/micro-op.scala, which is
  hand-written baseline BOOM v4 and stays in place.

  hierarchy.yaml: kind: package, mode: edit_existing,
  target src/main/scala/v4/common/micro-op.scala. No `output:` — the
  pre-existing file is the artifact. Budget: ~170 added lines.

  Everything already in `class MicroOp` — `inst`, `iq_type`, `fu_code`, the
  `iw_*` issue-window fields, `br_mask`, `pdst`/`prs1`/`prs2`/`prs3`,
  `stale_pdst`, `uses_ldq`/`uses_stq`, the `is_br`/`is_jal`/`is_jalr`
  predicates, `starts_bsy`, `starts_unsafe` — is unchanged and is not restated
  below. Anything this file does not mention keeps its current declaration and
  meaning exactly.

  ===> WHY THIS FILE IS THE MOST DANGEROUS DELTA IN THE MAP. `MicroOp` is the
       single most depended-upon declaration in BOOM: it is instantiated in
       every pipeline stage, every queue entry and every ROB row, so each field
       added here costs area in dozens of places at once and the fan-in over
       `depends_on:` is the blast radius. The previous attempt added 162 lines
       here, and its `pvl` busy-bit bugs traced back to this bundle. Add the
       fields the design needs and not one more.

  Governing spec anchors: overview.rst and glossary.rst (the uOP model),
  midcore.rst `old-vd` and `vl-vtype-rename`, frontend.rst `vset-dual-dest`,
  loadstore.rst `elem-progress`.

<|begin_module|>

  <|begin_parameters|>
  No new parameters. The added fields take their widths from values that already
  exist or that VectorParams introduces: `vecPregSz`, `vlPregSz`, `maxMembers`,
  `vecVLSz` and `vLen`. No width below is a literal.

  ---- EVERY ADDED FIELD IS GATED ----

  All fields described here are elaborated only when `usingRVV` is true. With
  vectors disabled the bundle must be bit-identical to baseline BOOM v4 — same
  fields, same order, same total width — because gate (f) diffs the generated
  RTL of a non-vector config against the pre-Caracal baseline, and a bundle that
  gained even a zero-width field would perturb every module that carries a uop.
  Use the standard Chisel idiom for conditional bundle fields so that the
  disabled case emits nothing at all rather than a tied-off field.
  <|end_parameters|>

  <|begin_ports|>
  Not applicable. `MicroOp` is a Chisel `Bundle`, not a module: it has no I/O,
  no clock and no reset of its own. It inherits `BoomBundle`'s implicit
  `Parameters` exactly as it does today, and this delta does not change that.
  <|end_ports|>

  <|begin_logic|>
  The delta adds field declarations to `class MicroOp`, in the groups below. It
  adds no methods except where stated, and changes no existing field except the
  three register-type fields called out at the end.

  ---- 1. The vector marker ----

  //@req-spec-core.c4
  Add `is_vec`, a `Bool`. An OP.v is an ordinary uOP with `is_vec` set, carrying
  the vector-specific fields below; it is not a separate bundle type and not a
  variant. This is what lets a vector instruction flow through decode, rename,
  the ROB and issue on the same paths as a scalar one, with only the stages that
  care inspecting `is_vec`.

  Add `is_shared`, a `Bool`, marking an instruction whose execution needs both
  the vector LSU and the coprocessor — the segmented load/store forms. It gates
  the `pvtmp` allocation and the two-half issue behaviour described below.

  ---- 2. Logical vector specifiers (decode to rename only) ----

  Add `lvd`, `lvs1`, `lvs2`, `lvs3` and `lvm`, each `lregSz` wide, alongside the
  existing scalar `ldst`/`lrs1`/`lrs2`/`lrs3`. These are the architectural
  vector register numbers the decoder extracts, consumed by the vector mapper
  and dead thereafter, exactly as the scalar logical specifiers are.

  `lvm` names the mask register, which is architecturally always `v0`. Carry it
  as a field rather than hard-wiring zero at every read site so that the mask
  travels the same path as any other source and needs no special case in the
  map table read.

  ---- 3. Renamed vector operands: groups, not registers ----

  //@req-spec-rename.d5
  Add `pvdest` and `stale_pvdest`, each a `Vec(maxMembers, UInt(vecPregSz.W))`.
  `stale_pvdest` is a VECTOR of up to EMUL stale PRNs and NOT a single register:
  the group it names need not be contiguous, because the free list allocates a
  group without requiring contiguous PRNs, so a base-plus-count encoding could
  not name the group that was actually allocated.

  Add `pvs1`, `pvs2`, `pvs3` as `Vec(maxMembers, UInt(vecPregSz.W))` and `pvm`
  as a single `UInt(vecPregSz.W)` — the mask is one register, never a group.

  //@req-spec-core.h8
  //@req-spec-rename.e6
  Add `pvtmp`, also a `Vec(maxMembers, UInt(vecPregSz.W))`, holding up to EMUL
  members. This field IS the binding from the abstract `pvtmp` rendezvous to
  real PRNs — there is no separate table anywhere that records it, and no temp
  register file. The two halves of a shared instruction find each other by
  reading this field off the same OP.v.

  Add per-operand group busy bits: `pvs1_busy`, `pvs2_busy`, `pvs3_busy`,
  `pvm_busy`, `pvtmp_busy` and `pvl_busy`, each a single `Bool`. One bit per
  OPERAND, not per member: the busy table AND-reduces a group's per-member bits
  into one group-ready bit before it reaches the uop, because an operand wakes
  only when its LAST member is ready. Carrying per-member bits here would
  duplicate state the busy table already owns and let the two disagree.

  //@req-spec-vrf.j1
  //@req-spec-vrf.j2
  //@req-spec-vrf.j4
  //@req-spec-vrf.j5
  //@req-spec-vrf.j6
  ===> `pvs3` AND `stale_pvdest` ARE TWO INDEPENDENT FIELDS NAMING TWO
       INDEPENDENT PHYSICAL GROUPS, AND MUST NOT BE MERGED. `pvs3` names the
       group holding an explicitly ENCODED third source operand — the store data
       of a `vse`, the addend of an `vfmacc`. `stale_pvdest` names the group that
       held the destination architectural vreg BEFORE this OP.v renamed it, and
       it exists for every vector op with a destination whether or not that op
       encodes a third source at all.
       They COINCIDE for read-modify-write arithmetic, where the third source
       *is* the old destination. They DIVERGE for a masked non-RMW op under
       `vma = 0`, for `vslideup`'s untouched prefix, and for `vcompress`'s tail.
       Merging them into one field makes the diverging case unrepresentable:
       there would be no way to obtain both pvs3 and old-vd for the same
       instruction. Both are exposed to the coprocessor as distinct CII source
       slots (VS3 and STALE_VD) and the VPU chooses which to pull, so a merge
       here would also break the operand server, which performs no
       instruction-dependent reinterpretation.

  //@req-spec-rename.h18
  Add `pvl`, a single `UInt(vlPregSz.W)`: the renamed VL this uop reads.

  ===> DO NOT ADD `stale_pvl`. A VL producer carries NO per-uop stale VL field.
       The VL space has one architectural register, so its free discipline is to
       release the OUTGOING COMMITTED POINTER at commit — a single-entry
       committed map table, not a per-uop stale value. A `stale_pvl` field would
       be `numRobEntries` copies of a value that already exists once.
       Also DO NOT ADD `pvtype`: VTYPE is not renamed. And DO NOT ADD a
       `vl_is_known` fast-path flag: VL is read from the VL register file at
       execute, with no statically-known bypass.

  ---- 4. The static access descriptor ----

  Add `v_eew`, 2 bits, the element width of the DATA a memory access moves.
  For every non-indexed form this is the instruction's own encoded EEW and not
  `vtype.vsew`. For an INDEXED form it is `vtype.vsew`, because there the
  instruction's encoded width field describes the INDEX elements, not the data —
  see `v_idx_eew` below.
  Add `v_emul`, `log2Ceil(maxMembers) + 1` bits, the group's member count as a
  1..8 value, written by the vector mapper in the rename cycle.
  Add `v_seg_nf`, 3 bits, the segment field count `nf` for a segmented access.

  Add `v_idx_eew`, 2 bits, the element width of the INDEX vector, meaningful only
  when `v_is_indexed` is set and don't-care otherwise.

  Add `v_is_masked`, a `Bool`: the decoded sense of the instruction's `vm` bit,
  set when the OP.v is actually masked.

  ===> THIS MUST BE A FIELD, not re-derived from `inst(25)` at each use site.
       Three separate consumers need it and none of them should be decoding an
       instruction word: the issue slot must include `pvm`'s busy bit in
       readiness ONLY when the op is masked (otherwise every unmasked vector op
       gains a false dependency on the last writer of `v0`, and a stale mask PRN
       gets waited on forever); `VecMaskStream` must know whether to read the
       mask at all; and `VecElemAgen` must know whether to consult mask bits when
       skipping. `MicroOp` does not carry the full `inst` to those stages, and the
       vector mapper only renames `lvm` for a masked op, so `pvm` alone cannot
       distinguish "unmasked" from "masked by whatever v0 last held".

  Two element widths, not one, and conflating them is a silent corruption
  rather than a compile error: for `vluxei32.v` with SEW=8 the index elements
  are 32b while the data elements are 8b, so the index group size and the data
  group size differ. VLSDecode resolves the instruction's raw `width` field
  into these two fields; no consumer re-derives them.

  Add `v_uses_vs1`, `v_uses_vs2` and `v_uses_vs3`, each a `Bool`: "this
  instruction's FORMAT actually encodes that vector source". `VDecode` sets them
  for the arithmetic forms and `VLSDecode`'s descriptor supplies them for the
  memory forms; nothing downstream re-derives them, because only decode knows the
  instruction format and the instruction word does not reach rename or issue.

  ===> THESE THREE BITS PREVENT A HANG, NOT A SLOWDOWN. Take `vadd.vx`
       (`vd, vs2, rs1`): the `vs1` field is UNENCODED, so nothing sensible is in
       `lvs1`, the vector mapper's map-table read returns the CURRENT MAPPING OF
       `v0` — the MASK register, which a masked program rewrites constantly — and
       `pvs1` names a physical group this instruction will never read. That group
       may well be busy. And if its producer's group-done already fired BEFORE
       this slot captured its member-ready bits, no future group-done will ever
       clear them: the group is complete and nothing re-announces a completion.
       The slot then waits forever, on a source the instruction does not have.
       It is not a false dependency that eventually resolves; it is a permanent
       stall, with no assertion of its own, on a common instruction form.

  Two consumers, and the bit must come from decode for both:
    - the VECTOR MAPPER skips renaming an unencoded source and leaves that
      operand's busy bit CLEAR, so it arrives ready instead of pointing at a
      stale group;
    - `VecIssueSlot` drives them onto its per-operand `used` inputs, exactly as
      it drives `v_is_masked` onto `rdy_vm.used`, so an unencoded operand is
      dropped from the readiness cone rather than ANDed in and hoped to be early.
  Both, deliberately: the mapper makes the operand ready and the slot removes it
  from the cone, so neither is singly load-bearing.

  This MIRRORS `v_is_masked` earlier in this section, which exists for the
  identical reason — an unmasked op whose `pvm` names whatever `v0` last mapped
  to. The same failure, one field short of being covered for the three source
  groups; `pvtmp` is covered by `is_shared` for the same reason again.

  REJECTED: a reserved SENTINEL value in `lvs1`/`lvs2`/`lvs3` meaning
  "unencoded". Encodings 32..63 are free at `lregSz = 6`, so it costs zero new
  bits — which is exactly why it is tempting. It makes correctness depend on
  every future reader of those fields remembering an implicit convention that
  no type expresses, and the failure mode of a reader that forgets is the
  silent hang above rather than a compile error.
  ALSO REJECTED: full `lvs1_rtype`/`lvs2_rtype`/`lvs3_rtype` fields. Three
  times the bits for no additional information: the only question any consumer
  asks is "is this source encoded", never "which rename space is it in",
  because a vector source is always in the vector space.

  COST, stated plainly because this bundle's fan-in IS the review list: 3 bits
  replicated across every pipeline register, queue entry and ROB row, in a
  bundle with 49 transitive dependents. That is the price, and it is paid
  because the alternative is a hang rather than a slowdown.

  ---- 4b. The access CLASS — which agen this op goes to ----

  Add the access-class fields that `VLSDecode` decodes and the vector LSU routes
  on: `v_mop` (2 bits, the raw `mop` field) plus the derived one-hot-ish flags
  `v_is_unit_stride`, `v_is_strided`, `v_is_indexed`, `v_is_segment`,
  `v_is_whole_reg`, `v_is_mask` and `v_is_ff`.

  ===> THESE MUST BE MicroOp FIELDS, AND THE REASON IS STRUCTURAL. The access
       class is decoded ONCE, at decode, by `VLSDecode`. Its consumers are
       `VecLsu` (which routes the op to one of four agen instances),
       `VecElemAgen`, `VecRangeAgen`, `VecIdxGen` and `VecBeatExpander` — and NONE
       of them depends on `VLSDecode`, so the descriptor has no path to them
       except riding the uop. The only alternative is for a downstream module to
       re-decode `uop.inst`, which would duplicate the one decision `VLSDecode`
       exists to own, in a module that has no business knowing the RVV encoding.
       A second decoder that disagrees with the first sends an op to the wrong
       agen, and unit-stride arriving at the element agen is precisely the
       one-beat-per-element behaviour v2 exists to delete.

  Class is decoded from `mop` and `umop` ALONE — never from the direction.
  Direction reaches the agens as the `isStore` module PARAMETER, which is why
  each agen is instantiated twice and the two directions never arbitrate.
  The flags are mutually exclusive by construction; assert one-hot rather than
  relying on it.

  ---- 5. The vtype snapshot ----

  //@req-spec-vrf.c7
  Add `vconfig`, holding the `vtype` snapshot taken at DECODE from the
  speculative VCFG mirror. This is how VTYPE reaches the execution units: per
  uOP, carried with the instruction, rather than read from a CSR at execute or
  renamed. Declare it as rocket-chip's `VType` so it is the same encoding
  rocket's `CSRFile` uses for architectural `vtype` — see VtypeTable for why the
  two must not have separate encodings.

  Its `vill` field doubles as the mirror's poison flag, so a `vill` snapshot
  travels with the uop that must trap on it.

  //@req-spec-decode.d6
  ===> `vconfig` CARRIES VTYPE ONLY. Do NOT add fields for `vstart`, `vxrm` or
       `vxsat`, and do not snapshot them into the uop. Those three are read from
       the CSR file at EXECUTE. Snapshotting them would be wrong rather than
       merely wasteful: they are not part of the speculative configuration a
       branch must restore, and a stale snapshot of `vxsat` — a sticky
       accumulating flag — would lose accrued saturation.
       `vl` is likewise absent from the uop: VL is renamed, so it is reached
       through `pvl` and read from the VL register file.

  ---- 6. The dual-destination bit ----

  //@req-spec-decode.c18
  Add `is_vl_producer`, a `Bool`, set on any instruction that writes the VL
  register file. It is ORTHOGONAL to `dst_rtype` and must be a field of its own,
  not an encoding within it, because a register-sourced `vset` has TWO
  destinations in TWO independent rename spaces: `pdst` in the integer file and
  `pvl` in the VL file. `dst_rtype` is single-valued and cannot express both.

  The x0 case is exactly why this cannot be folded into dst_rtype: for
  `vsetvli x0, rs1, vtype` the integer destination is discarded, so dst_rtype
  reads RT_ZERO — and the VL register file must STILL be written. A consumer
  that inferred "writes VL" from dst_rtype would drop that write and every
  dependent would read a stale VL.

  ---- 7. nOP.v-scoped element cursor fields ----

  //@req-spec-core.c11
  Add the element and segment cursor fields `v_split_first`, `v_split_last`,
  `v_split_idx` and `v_split_total`. These are NOP.V-SCOPED: they describe one
  cracked element access, not the whole instruction.

  //@req-spec-agen.a4
  Alongside them add the two fields that say which register the access targets:
  the destination `prn` it will write and the BYTE OFFSET within that PRN. A
  response can return out of order, so the Load Coalescing Buffer places it by
  those two fields alone and cannot recover them from the element index without
  re-deriving EMUL and the mask.

  Add one more field beside them, `v_mem_tag`, `UInt(ldRespTagSz.W)`: which of the
  outstanding load beats a D$ response belongs to. The D$ hands the request's uop
  back unchanged, so carrying the tag here is what lets `VecLsu` key its
  response-alignment table by REQUEST rather than by lane — necessary because
  `ll_resp` always returns on lane `lsuWidth-1` no matter which lane issued the
  request. It is meaningful only on a vector load beat in flight.

  //@req-spec-lsu.f1
  //@req-spec-lsu.f2
  //@req-spec-lsu.f3
  //@req-spec-lsu.f4
  A vector LDQ or STQ entry carries an ELEMENT CURSOR in addition to the scalar
  fields it already has. Declare the cursor as one named sub-bundle so the LDQ
  and STQ entries include it as a unit rather than as three loose fields, with
  exactly three members:
    - `elem_next`  — the index of the next element to drain.
    - `elem_done`  — the count or bitmap of completed elements.
    - `fault_elem` — the index of the OLDEST faulting element, latched on the
                     first fault.

  fault_elem is retained ONLY as the element cursor's stop signal and as a
  debug/performance counter. It is never carried to the ROB and never written
  to vstart: a faulting vector op traps with vstart = 0 and restarts whole,
  because its fresh pvdest group is reclaimed and elements 0..k-1 were never
  architecturally visible. See the Rob delta.

  //@req-spec-core.c12
  ===> ALL OF THESE FIELDS ARE INERT ON THE OP.v ITSELF. They are populated only
       when the vector LS AGEN emits element accesses, and on the OP.v flowing
       through decode, rename, the ROB and issue they are don't-care. Nothing
       outside the vector LSU may read them. Assert this rather than trusting
       it: on a uop with `is_vec` set that has not been through AGEN, the cursor
       fields must be zero, so that a stage reading them by mistake gets a
       consistently wrong answer instead of an intermittently plausible one.

  ---- 8. The one change to an EXISTING field ----

  //@req-spec-decode.c25
  Widen `dst_rtype`, `lrs1_rtype` and `lrs2_rtype` from `UInt(2.W)` to
  `UInt(3.W)` so the register-type space can hold the new `RT_VEC` encoding
  (added by the ScalarOpConstants delta). This is a WIDENING ONLY: `dst_rtype`
  keeps its unmodified integer meaning end to end, `RT_FIX`/`RT_FLT`/`RT_X`/
  `RT_ZERO` keep their current values, and no existing comparison against them
  changes behaviour. Nothing is overloaded and no new meaning is attached to an
  existing encoding.

  This is the only existing declaration the delta touches. In particular the
  `starts_unsafe` and `starts_bsy` methods, the `is_br`/`is_jal`/`is_jalr`
  predicates and `allocate_brtag` are NOT modified here — the vector-related
  changes to ROB safety live in the Rob delta, not in this bundle.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
This bundle has no timing behaviour of its own, but it has a hard area
constraint, and it is the reason the field list above is as short as it is.

A `MicroOp` is replicated across every pipeline register, issue slot, queue
entry and ROB row in the machine. The group fields alone are five
`Vec(8, UInt(7.W))` = 280 bits at the default sizing, so a careless addition
here is multiplied by the whole uop population. Two consequences the
implementation must respect:

- The group fields must be sized by `maxMembers` and `vecPregSz`, never by a
  literal, so a smaller configuration actually gets a smaller bundle.
- With `usingRVV = false` the bundle must emit exactly the baseline field set.
  Not a zero-width field, not a tied-off one: absent. Gate (f) compares RTL.
<|end_perf|>

<|begin_dependencies|>
VectorParams — for `vecPregSz`, `vlPregSz`, `maxMembers`, `vecVLSz`.

Binds to `freechips.rocketchip.rocket.VType` for the `vconfig` field.

Instantiates nothing; it is a Bundle. Its dependents are essentially the whole
design — BoomCore, Rob, LSU, DecodeUnit, ALUUnit, FpPipeline, VecBundles,
VecTrace, VecPipeline and most `vec/**` modules all declare `depends_on:
MicroOp`. Before adding a field, take that fan-in as the review list.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File   src/main/scala/v4/common/micro-op.scala
    Class  `class MicroOp(implicit p: Parameters) extends BoomBundle`
    (package boom.v4.common). This is hand-written baseline BOOM v4.

  In scope:
    - Adding the new `val` field declarations listed in the logic section to
      `class MicroOp`, each conditional on `usingRVV`.
    - Widening exactly three existing declarations from `UInt(2.W)` to
      `UInt(3.W)`: `dst_rtype`, `lrs1_rtype`, `lrs2_rtype`.
    - Adding the `import` needed for rocket's `VType` and for VectorParams.
    - Adding one named cursor sub-bundle declaration for the element cursor.

  Must not regress:
    - Every existing field of `class MicroOp` keeps its name, width, order and
      meaning. In particular `pdst`, `prs1`, `prs2`, `prs3`, `stale_pdst` and
      `ppred` are UNCHANGED — the vector groups are new fields beside them, never
      a reinterpretation of them.
    - The existing methods `is_br`, `is_jal`, `is_jalr`, `is_sfb_br`,
      `is_sfb_shadow`, `allocate_brtag`, `starts_bsy` and `starts_unsafe` keep
      their current bodies exactly. `starts_unsafe` in particular must NOT gain
      a vector term here; ROB safety for vector ops is the Rob delta's business.
    - `abstract trait HasBoomUOP` and its `val uop = new MicroOp()` are untouched.
    - The three widened `*_rtype` fields keep the existing `RT_FIX`, `RT_FLT`,
      `RT_X` and `RT_ZERO` values and comparison semantics. Widening must not
      change the result of any existing comparison.
    - With `usingRVV = false`, the elaborated bundle is bit-identical to the
      current file's: identical field set, identical widths, identical total
      width, and no field present-but-zero-width.
    - The file's existing copyright header and comment style are preserved. No
      reformatting, no reordering of existing fields, no renaming.

  Interface delta:
    NEW fields (all `usingRVV`-gated):
      is_vec, is_shared, is_vl_producer                                   : Bool
      lvd, lvs1, lvs2, lvs3, lvm                                : UInt(lregSz.W)
      pvdest, stale_pvdest, pvs1, pvs2, pvs3, pvtmp
                                          : Vec(maxMembers, UInt(vecPregSz.W))
      pvm                                                  : UInt(vecPregSz.W)
      pvl                                                   : UInt(vlPregSz.W)
      pvs1_busy, pvs2_busy, pvs3_busy, pvm_busy, pvtmp_busy, pvl_busy     : Bool
      v_eew                                                          : UInt(2.W)
      v_idx_eew                                                      : UInt(2.W)
      v_emul                          : UInt((log2Ceil(maxMembers) + 1).W)
      v_seg_nf                                                       : UInt(3.W)
      v_is_masked                                                         : Bool
      v_mop                                                          : UInt(2.W)
      v_is_unit_stride, v_is_strided, v_is_indexed, v_is_segment,
        v_is_whole_reg, v_is_mask, v_is_ff                                 : Bool
      v_uses_vs1, v_uses_vs2, v_uses_vs3                                  : Bool
      vconfig                                        : rocketchip.rocket.VType
      v_split_first, v_split_last                                         : Bool
      v_split_idx, v_split_total                             : UInt(vecVLSz.W)
      the nOP.v target PRN and byte-offset-within-PRN fields
      v_mem_tag                                        : UInt(ldRespTagSz.W)
      the element cursor sub-bundle {elem_next, elem_done, fault_elem}

    WIDENED fields:
      dst_rtype, lrs1_rtype, lrs2_rtype              : UInt(2.W) -> UInt(3.W)

    Explicitly NOT added, and a reviewer should reject them if they appear:
      pvtype, stale_pvl, vl_is_known, any per-member busy bit vector, any
      vstart / vxrm / vxsat snapshot field, and `lvs1_rtype`/`lvs2_rtype`/
      `lvs3_rtype` — the three `v_uses_vs*` Bools carry the only part of that
      information any consumer asks for, at a third of the bits.

  Obligation this delta places on the `DecodeUnit` delta:
    `is_vec`, `is_shared`, `is_vl_producer` and `v_uses_vs1`/`v_uses_vs2`/
    `v_uses_vs3` MUST BE EXPLICITLY DEFAULTED — driven false on every uop that no
    vector decoder claims. This is not defensive tidiness. `DecodeUnit` does
    `uop := io.enq.uop`, and `io.enq.uop` arrives from a bundle the FRONTEND sets
    `:= DontCare`, so a field this delta adds is UNDRIVEN on a scalar uop unless
    something drives it. Without those defaults every *scalar* uop carries
    don't-care vector bits into dispatch, where `is_vec` mis-routes it,
    `is_shared` asks the free list for a `pvtmp` rendezvous group it will never
    use, `is_vl_producer` claims a VL register file write, and a stray
    `v_uses_vs*` bit puts a garbage source group into an issue slot's readiness
    cone — the hang described in the logic section, reached from a scalar
    instruction.

    ⇒ THE DEFAULTING OBLIGATION COVERS EVERY ADDED BOOL, not only the four named
    above. `v_is_masked`, `v_mop`, the seven `v_is_*` class flags and the three
    `v_uses_vs*` bits all reach `DecodeUnit` through the same `uop := io.enq.uop`
    from a frontend bundle set `:= DontCare`, so all of them need an explicit
    default on the scalar path. A don't-care CLASS flag is worse than a don't-care
    routing bit: it would send a scalar uop into an agen.
    The list above was previously incomplete — it omitted v_is_masked,
    v_idx_eew, v_mop and the seven class flags even though the logic section
    adds them. Since a reviewer is instructed to REJECT any field absent from
    this list, an incomplete list is a live trap in both directions: it invites
    rejecting a field the design needs, and it hides a field nobody audited.

    ===> A PASSING GATE (f) DOES NOT PROVE THE DEFAULTS EXIST. A don't-care bit
         can elaborate BIT-IDENTICALLY to the baseline — the compiler is free to
         resolve it either way — and still mis-route at run time. The check is a
         read of the `DecodeUnit` delta plus an assertion that these bits are
         false on every uop with `is_vec` clear; it is never a clean (f) diff.
<|end_edit_scope|>

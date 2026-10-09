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
  VLSDecode — the STATIC ACCESS DESCRIPTOR of a vector load or store: `v_eew`,
  `mop`, `nf`, the unit-stride / strided / indexed / segment / whole-register /
  mask flags, and which vector source operands the form actually encodes, decoded
  from the instruction word alone.
*/

hierarchy.yaml: kind: module, mode: new,
output src/main/scala/v4/vec/generated/decode/VLSDecode.scala,
package boom.v4.vec.generated.decode, group vec_decode.
depends_on MicroOp, VecTrace. Instantiated once, as `ls`, by VecDecode.

===> WHAT THIS DESCRIPTOR IS FOR, AND IT IS NOT A DETAIL. The descriptor is
     what selects the FILL-side address generator downstream: `VecRangeAgen`
     (one range entry for the whole access) for unit-stride, `VecElemAgen`
     (one element access per ACTIVE element) for strided, segmented and
     indexed. That selection is by ACCESS CLASS and NEVER by direction —
     load and store take the identical three rules, and direction is a
     PARAMETER of the agen instances (`isStore`), not an input to the choice.
     Everything in the logic section below exists to make that selection a
     decode-time, one-hot, exhaustive function of fixed instruction bits, so
     that no downstream module ever has to guess a class or fall back to one.

===> THIS MODULE READS `vtype` FOR NOTHING. Every field it produces comes from
     the 32-bit instruction word — EEW, `mop` and `nf` are ENCODED, whereas
     SEW, LMUL and EMUL are CONFIGURATION. That is why its `depends_on` list
     has neither VtypeTable nor VectorParams. Mixing the two here is the
     classic vector-memory decode bug; the indexed case below is where it bites.

Governing spec anchors: execution.rst `vector-agen` (the selection rules and
the Packer / Skipper / Walker paragraphs), loadstore.rst `elem-progress`
("Fault-only-first (``vleff.v``)"), frontend.rst `vector-rvv-decode`. Plan
section 2 deletes the six inherited OVI Packer/Skipper/Walker modules in favour
of VecElemAgen / VecRangeAgen / VecBeatExpander cut by pipeline position; their
selection OBLIGATIONS are discharged here, which is why this file names them.

<|begin_module|>

  <|begin_parameters|>
  `coreWidth` — the number of decode lanes. Default 3; legal 1 to 8. The module
  is a plain per-lane replication of one pure combinational decoder with NO
  cross-lane logic whatsoever: no arbitration, no shared resource, no state, so
  N lanes are N independent copies of a small comparator network. VecDecode
  instantiates this module ONCE and it fans out internally, matching how
  VtypeTable is evaluated once per lane.

  No other parameters. In particular the whole-register register count is NOT
  sized from `maxMembers`: its legal values are 1, 2, 4 and 8 by the RVV
  encoding itself, so the field is 4 bits and no VectorParams dependency is
  needed to size a width the ISA already fixes.

  The entire module is elaborated only when `usingRVV` is true (a Scala
  `Boolean` from `BoomCoreParams`, not a hardware `Bool`, and NOT rocket's
  `usingVector`). With vectors disabled VecDecode does not instantiate it and no
  RTL is emitted at all — absent, not tied off — so a non-vector build stays
  bit-identical to pre-Caracal BOOM v4.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel default and the hierarchy.yaml default:
  posedge `clock`, ACTIVE-HIGH SYNCHRONOUS `reset`, single `core_clk` domain.
  Both are present because this is a `Module`, and both are functionally UNUSED:
  the decoder is purely combinational and holds no register. `reset` is read
  only by the trace gate described in the logic section.

  Per lane `i` in 0 until `coreWidth`:

  - `io.lanes(i).valid`   — Input, 1 bit. This lane holds a decoded instruction
                            this cycle. It is a QUALIFIER, not a handshake:
                            there is no `ready` anywhere in this module, because
                            decode does not back-pressure and a combinational
                            decoder cannot stall. The outputs are still driven
                            when `valid` is low; a consumer must qualify them.
  - `io.lanes(i).inst`    — Input, 32 bits. The raw instruction word. The ONLY
                            functional input.
  - `io.lanes(i).uop`     — Input, `MicroOp`. Context for the trace line only;
                            the logic section states which fields it may read.
  - `io.lanes(i).desc`    — Output, `VLSAccessDesc` (declared in this file, see
                            the logic section). The static access descriptor.
  - `io.lanes(i).illegal` — Output, 1 bit. This lane holds a vector load/store
                            with a RESERVED encoding. Feeds VecDecode's
                            `dec_vec_illegal` lane bit on the `vec_pipeline_io`
                            seam, which BoomCore turns into an illegal-instruction
                            trap. Asserted only when `is_vls` is set: a
                            non-vector instruction must never be reported
                            illegal from here.

  There are no other ports. Notably absent, and a reviewer should reject them:
  any `busy`, `ready` or `stall` output; any `vtype`/`vl`/`vstart` input; any
  VRF port. A `busy` from anywhere on the vector path is a design-invariant
  violation, and this unit has no reason to want one.
  <|end_ports|>

  <|begin_logic|>
  ---- 0. The output bundle ----

  Declare `class VLSAccessDesc` in this file (it is the only bundle here, is
  produced by this module and is consumed through the uop's static
  access-descriptor fields, so it does not belong in VecBundles):
    `is_vls`, `is_store`                                              : Bool
    `mop`                                                        : UInt(2.W)
    `vm`                                                              : Bool
    `eew`                                                        : UInt(2.W)
    `eew_is_index`                                                    : Bool
    `nf`                                                         : UInt(3.W)
    `is_unit_stride`, `is_strided`, `is_indexed`                      : Bool
    `is_segment`, `is_whole_reg`, `is_mask`, `is_ff`                  : Bool
    `nregs`                                                      : UInt(4.W)
    `uses_vs1`, `uses_vs2`, `uses_vs3`                                : Bool

  `mop` is carried VERBATIM rather than being reduced to the three class flags,
  so a consumer that needs the indexed-ordered versus indexed-unordered
  distinction (the memory-ordering units) reads `mop` instead of asking for
  another synthesized flag here.

  ---- 1. Is this even a vector load/store ----

  A vector load/store is `opcode == LOAD-FP (0000111)` or
  `opcode == STORE-FP (0100111)` — the same two opcodes as scalar `flw`/`fld`
  and `fsw`/`fsd` — disambiguated by the `width` field `inst(14,12)`: a VECTOR
  access uses `000` (EEW 8), `101` (16), `110` (32) or `111` (64), while scalar
  FP uses `010` and `011`. Set `is_vls` from exactly that conjunction.

  Wrong in the permissive direction, every fld acquires a vector access
  descriptor; wrong in the restrictive direction, vector loads decode as
  scalar FP. Both fail silently at decode and surface far away.

  `is_store` comes from the opcode and NOTHING else. It is recorded in the
  descriptor for the LSU's direction routing and for the trace line. It is
  never an input to any class decision below.

  ---- 2. Field extraction: fixed bits, no arithmetic ----

  From `inst`, unconditionally: `nf = inst(31,29)`, `mew = inst(28)`,
  `mop = inst(27,26)`, `vm = inst(25)`, the unit-stride sub-opcode
  `umop = inst(24,20)` (the encoding's `lumop`/`sumop`), and `width` as above.
  `eew` is the 2-bit log2 element-size that `width` names: `000` to 0,
  `101` to 1, `110` to 2, `111` to 3. `mew` is the reserved EEW extension and
  must be 0.

  ---- 3. ACCESS CLASS: one-hot, exhaustive, from `mop` alone ----

  //@req-spec-agen.c26
  The three class flags are decoded from `mop` and from `mop` only, and this is
  the requirement the whole file is shaped around: the generator is selected by
  ACCESS CLASS, NOT BY DIRECTION.
    `is_unit_stride` = (`mop` == 00)
    `is_indexed`     = (`mop` == 01) or (`mop` == 11)   // unordered / ordered
    `is_strided`     = (`mop` == 10)
  `is_store` appears in none of these expressions, and the load and store paths
  consume the identical descriptor. The only real load/store asymmetries in
  address generation are which VRF port carries the mask and index (R0 index /
  R1 mask on the load path, R4 on the store path — `midcore.rst` `vrf-ports` is
  the authority and this module adds no port to that table) and that the store
  path also runs the DGEN. Neither is visible here. Direction reaches the agens
  as the `isStore` PARAMETER of their two instances, so a load and a store of
  the same class run structurally identical generators — what the deleted OVI
  cross-product failed to guarantee, having drifted into divergent mask support
  between its load and store Packers.

  //@req-spec-agen.c25
  The class decode is TOTAL and there is NO DEFAULT ARM. Every value of the
  2-bit `mop` field selects exactly one class, so an access is never assigned a
  generator by falling through to one. In particular the index-driven generator
  is NOT the fallback for a non-indexed access: its per-element offset comes
  from the index interface, and a strided access has no index vector to drive
  it, so a fallback would drive that interface with an undriven or stale
  offset. Emit a Chisel `assert` that the three flags are exactly one-hot
  whenever `valid && is_vls`. The assert is a check, not behaviour: deleting
  every one leaves the emitted datapath bit-identical.

  ---- 4. What each class selects downstream ----

  //@req-spec-agen.c5
  `is_unit_stride` selects `VecRangeAgen`, which emits ONE range entry
  describing `[base, base + VL*EEW)` into the `*_US_ADDR_Q`, and the drain-side
  `VecBeatExpander` (the stage-2 Packer) coalesces that range into D$-width
  beats just in time. Because `is_unit_stride` and `is_indexed` are decoded from
  disjoint values of the same 2-bit field, they are MUTUALLY EXCLUSIVE BY
  CONSTRUCTION: the range/Packer path can never be selected for an indexed
  access, which it does not support. This is a structural guarantee, not a
  checked one — there is no encoding of `mop` that sets both.

  //@req-spec-agen.c13
  //@req-spec-agen.c22
  A non-indexed, non-unit-stride access — strided or segmented, masked or not —
  selects `VecElemAgen` in its NO-INDEX mode (the absorbed Skipper): the
  element address is `base + i*stride`, walked under the mask. It must NOT
  select the index-driven mode, which does not support non-indexed accesses.
  The descriptor makes that enforceable rather than conventional: `VecIdxGen`'s
  enable is `is_indexed` and nothing else, so on a strided access the index
  interface is not merely ignored, it is never started.

  //@req-spec-agen.c21
  An INDEXED access selects `VecElemAgen` in its index-driven mode (the
  absorbed Walker), for `mop` == 01 and `mop` == 11 alike, and REGARDLESS OF
  `vm`: indexed and indexed-masked take the same generator. Masking is an
  optimization inside the generator — the mask-derived cursor decides which
  elements survive — and is NOT a selection criterion. `vm` therefore appears
  in the descriptor as plain data and in none of the class expressions.

  In v2 the Skipper/Walker distinction is no longer a module boundary:
  masked-skip and index-driven differ only in where the per-element address
  comes from, so they are two modes of VecElemAgen. The selection OBLIGATION
  survives the merge and lives here, in the one place that decides the class.

  ---- 5. Unit-stride sub-forms: the `umop` decode ----

  When `is_unit_stride`, `umop` names which unit-stride form this is, and all
  four remain the unit-stride access class — they are contiguous byte ranges and
  all take `VecRangeAgen`:
    `00000` — ordinary unit-stride (`vle*.v` / `vse*.v`).
    `01000` — WHOLE REGISTER (`vl<nregs>re<eew>.v` / `vs<nregs>r.v`). Set
              `is_whole_reg`, and set `nregs` to `nf + 1`, whose only legal
              values are 1, 2, 4 and 8.
    `01011` — MASK (`vlm.v` / `vsm.v`). Set `is_mask`.
    `10000` — FAULT-ONLY-FIRST (`vleff.v`), load only. Set `is_ff`.
  Anything else is reserved; see the illegal rules below.

  ===> `nf` DOES NOT MEAN SEGMENT COUNT ON A WHOLE-REGISTER ACCESS. The same
       three bits carry NFIELDS-1 for a segmented access and the register count
       minus one for `vl<n>re<eew>`. So `is_segment` is NOT `nf =/= 0`; it is
       `nf =/= 0 && !is_whole_reg && !is_mask && !is_ff`. A bare `nf =/= 0`
       makes `vl8re64` look like an 8-field segmented load, which would demand
       a `pvtmp` rendezvous group and the coprocessor transpose datapath for an
       instruction that is a plain contiguous copy. Compute `is_segment` from
       the qualified expression and nowhere else.

  Whole-register and mask accesses have a FIXED byte length rather than one
  derived from VL and EEW. This module does not compute those lengths —
  `VecRangeAgen` owns the range derivation — it only classifies, so that
  VecRangeAgen switches on a flag instead of re-decoding `umop`.

  ---- 6. `vleff` — decoded as an ordinary speculative load ----

  //@req-spec-lsu.g5
  `is_ff` marks `vleff.v` and marks nothing else. `vleff` IS NOT SERIALIZED: it
  is an ordinary speculative vector unit-stride load. This module sets no
  fence, no barrier, no ordering and no serialization attribute on it, and its
  descriptor is bit-identical to that of the corresponding `vle*.v` except for
  the `is_ff` bit itself. `is_ff` exists so the LSU knows to trim rather than
  trap on a fault at element `i > 0`; it must not become a decode-stage
  ordering property.

  //@req-spec-lsu.g11
  `vleff` MUST NOT BE MARKED `is_unique`. This module never sets `uop.is_unique`
  and asserts nothing VecDecode could turn into it. The earlier draft's
  reasoning was wrong twice over: `is_unique` does not make an instruction the
  only one in flight — it stalls only the unique uop's OWN dispatch until the
  ROB drains, after which younger uops dispatch a cycle later — and the
  serialization was never needed, because VL is renamed into the VL register
  file, so a trimmed VL reaches consumers through `pvl` and the VL wakeup
  network with correct ordering by construction.
  vleff is hot in strlen/memchr-style loops, so any is_unique term reachable
  from is_ff is a reject, not a conservatism.

  `vleff` IS A VL PRODUCER. It writes its final (possibly trimmed) element count
  to its VL register file destination and wakes `pvl` in its dependents, per
  loadstore.rst, `spec-lsu.g6`/`g7` and the plan's Phase G. `VecDecode` is the
  module that writes the uOP and it sets `is_vl_producer := desc.is_ff`; this
  module contributes the `is_ff` bit that decision is made from, and nothing here
  may suggest otherwise.

  CORRECTED. An earlier draft of this paragraph said `is_ff` does NOT make the
  uop a VL producer "until the real fault-trim path exists", citing a
  hierarchy.yaml comment that has since been corrected. That was a STAGING note
  that read as a design statement, and it contradicted two requirements
  allocated to sibling nodes (`VlRegFile`'s write port and
  `VecLoadCoalescingBuffer`'s trim report) as well as `VecDecode`, the actual
  writer. Staging is real but belongs in the plan's step table: the VL-RF write
  port for `vleff` is declared from day one and simply never fires until step
  G4 lands. It must NOT be retrofitted as an arbiter on the ALU write port —
  `spec-decode.c27` forbids arbitrated VL-RF write ports.

  The `vleff` FAULT AND TRIM POLICY is not this module's either, and no longer
  lives in the file that used to claim it: `vleff` is architecturally unit-stride,
  so it takes `VecRangeAgen`, and `spec-lsu.g1-g4`/`g9`/`g10` are re-allocated
  there. What stays here is exactly `spec-lsu.g5` and `spec-lsu.g11` above — that
  `vleff` is not serialized and is not `is_unique` — which are decode-stage
  properties and the only ones this module can hold.

  ---- 7. Indexed accesses: `eew` names the INDEX, not the data ----

  For every non-indexed class, `eew` is the MEMORY DATA element width and
  `eew_is_index` is false. For an INDEXED access the `width` field encodes the
  INDEX element width, and the data element width is `vtype.vsew` — which this
  module deliberately cannot see. So drive `eew` from `width` unconditionally
  and set `eew_is_index = is_indexed`, making the reinterpretation explicit in
  the descriptor.

  ===> A CONSUMER THAT READS `eew` AS THE DATA WIDTH ON AN INDEXED ACCESS IS
       WRONG, and so is any EMUL derived from it: for a non-indexed access the
       data group is `LMUL * EEW / SEW` members, whereas for an indexed access
       the DATA group is `LMUL` members and it is the INDEX group that is
       `LMUL * idxEEW / SEW`. Combining `eew` with the `vconfig` snapshot to
       reach a data EEW and an EMUL happens in VecDecode, which has both, guarded
       by `eew_is_index`.

  ---- 7b. `uses_vs*` — which vector sources a MEMORY form encodes ----

  The descriptor also carries `uses_vs1`, `uses_vs2` and `uses_vs3`, which
  VecDecode copies onto the uOP's `v_uses_vs1`/`v_uses_vs2`/`v_uses_vs3` fields
  exactly as it copies the class flags. They are decoded here because they are a
  function of `mop` and the opcode direction and of nothing else — no vtype, no
  arithmetic funct6.

    `uses_vs1` = false, always. NO vector load or store form encodes a vs1
                 vector source. The vs1 field position holds `rs1`, the integer
                 BASE address, on every one of them.
    `uses_vs2` = `is_indexed`. On an indexed access the vs2 field names the INDEX
                 vector group. On a strided access the same field is `rs2`, the
                 integer stride; on a unit-stride access it is `umop`, a
                 sub-opcode naming no register at all.
    `uses_vs3` = `is_store`. Store DATA is `vs3`, read from the vector register
                 file by the DGEN. A load encodes no vector source operand
                 whatever — for a non-indexed load all three bits are false.
                 This matches which specifiers `VecDecode` actually extracts: a
                 store sets `lvs3 := inst(11,7)`, a load sets `lvd` and leaves
                 `lvs3` unwritten, which is exactly the case that must not be
                 renamed.

  The three bits are per-uOP, not per-execution-half. A SEGMENTED store is still a
  store, and its coprocessor half reads `pvs3` as well, so `uses_vs3` is set for
  it; WHICH half consults the bit is `VecIssueSlot`'s per-queue business and is
  decided there, from these bits plus `is_shared`, not here.

  ===> WITHOUT THESE BITS A PLAIN `vle64.v` HANGS. Its vs1/vs2/vs3 fields hold an
       integer base and a sub-opcode, so the mapper reading them as vector
       specifiers renames whatever architectural vregs those bit patterns happen
       to name — frequently `v0`, the mask register, which a masked program
       rewrites constantly. A group whose group-done already fired is never
       re-announced, so an issue slot that ANDed its busy bit in waits forever.
       The MicroOp delta (section 4) carries the full argument. The bits are the
       load side of the same fix `v_is_masked` already provides for `pvm`.

  ===> `uses_vs3` IS THE DIRECTION BIT, AND THIS IS THE ONE PLACE IN THIS FILE
  WHERE `is_store` LEGITIMATELY APPEARS ON THE RIGHT-HAND SIDE. It is NOT a
  class decision — part 3's rule that the agen is selected by class and never
  by direction is untouched, since `uses_vs3` selects no generator and reaches
  no agen enable. It says only which register file the DGEN reads, which is a
  property of the direction by definition.

  And the store's SCALAR operands are INTEGER ONLY: `rs1` (base) and `rs2`
  (stride). No RVV store form takes an FP scalar operand — store data is
  always `vs3` — so nothing in this descriptor implies an FP read on the store
  path, and nothing downstream should provide one. `vfmul.vf`, the instruction
  once cited for a store-side FP read, is vector-scalar ARITHMETIC dispatched
  to the coprocessor and is not a store at all; the store-side FP read lane is
  deleted and the only remaining FP reader is the CII issue path.

  ---- 8. Reserved encodings ----

  Assert `illegal` on a lane when `is_vls` and any of:
    - `mew` is 1 (the reserved EEW extension beyond 64 bits);
    - `is_unit_stride` and `umop` is none of the four forms in part 5;
    - `is_whole_reg` and `nf + 1` is not 1, 2, 4 or 8;
    - `is_mask` and (`nf` is nonzero or `eew` is not 0) — a mask access is one
      byte-granular register's worth;
    - `is_ff` and `is_store` — there is no store counterpart to `vleff`;
    - `is_whole_reg` or `is_mask`, and `vm` is 0 — neither form is maskable.
  A segmented INDEXED access is LEGAL (`vluxseg`/`vsuxseg`) and must not appear
  in this list. Everything else legal decodes with `illegal` low.

  Reporting a reserved encoding here rather than letting it reach rename is
  what stops a mis-sized PRN group being allocated for an instruction with no
  defined meaning.

  ---- 9. Trace ----

  Emit ONE guarded trace line per valid vector load/store lane through the
  shared `VecTrace` helper's DECODE-STAGE entry point,
  `traceDecode(module, event, ftq_idx, pc_lob, extra)` — module name `VLSDecode`,
  event `desc`, with the extra fields `mop`, `eew`, `nf`, the class flags and
  `uses_vs2`/`uses_vs3` — gated on the `vecTrace` plusarg (off by default) and on
  `!reset`. There are no unit tests in this project; end-to-end VCS plus Whisper
  cosim is the only validation, so this is the first line to grep when an access
  takes the wrong agen.

  `io.lanes(i).uop` is read for the trace context only, and this module reads no
  other field of it — specifically not `vconfig`.
  The `traceDecode` variant is mandatory here, not a preference: at DECODE the
  uop's rob_idx is not yet assigned (the ROB allocates at DISPATCH), so a
  rob_idx-keyed line would claim `rob=0` and alias with real ROB entry 0 in
  every grep. `traceDecode` prints `rob=?` and keys on `ftq_idx`/`pc_lob`, so
  correlation with later stages is by PC/ftq_idx.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Single-cycle, purely combinational, zero registers, replicated `coreWidth`
times in the DECODE stage critical path — which is shared with VDecode,
VsetDecode and VConfigUnit's mirror update, so the budget here is small.

Structural consequences, treated as constraints rather than aspirations:
- Every output is a function of FIXED instruction bit ranges. No adder, no
  shifter, no multiplier, no barrel select, and nothing that synthesizes to a
  ROM. `nregs` is `nf + 1` on 3 bits and is the only arithmetic in the file.
- No pipeline stage may be added. The descriptor must be available in the same
  cycle as the instruction, because VecDecode combines `eew` with the `vconfig`
  snapshot to reach EMUL in that same cycle, and EMUL feeds atomic group rename
  — the design's top timing risk.
- No state, so no flush, squash or branch-kill path: a mis-speculated lane
  simply is not `valid` and the outputs are discarded.
<|end_perf|>

<|begin_dependencies|>
MicroOp — for the `MicroOp` type on the per-lane `uop` context input, and
because the descriptor's fields land in the uop's static access-descriptor field
group (`v_eew`, `v_seg_nf`, and the `v_uses_vs1`/`v_uses_vs2`/`v_uses_vs3` bits
part 7b decodes) which that delta owns.

VecTrace — for the guarded trace helper. Emit-only; it declares no state, so
removing every call site leaves this module bit-identical.

Instantiates NOTHING. It is instantiated once, as `ls`, by VecDecode, which
merges this descriptor into `dec_uops_out` and ORs `illegal` into
`dec_vec_illegal` on the `vec_pipeline_io` seam.

Deliberately NOT depended on: VtypeTable and VectorParams (this module decodes
the instruction word only, and the `depends_on` list is what keeps that boundary
honest), and VecBundles (`VLSAccessDesc` is declared here, this module being its
sole producer). The descriptor's downstream consumers — VecLsu's agen selection,
VecElemAgen, VecRangeAgen, VecIdxGen, VecMaskStream — reach it through the
MicroOp, not through a dependency on this module.
<|end_dependencies|>

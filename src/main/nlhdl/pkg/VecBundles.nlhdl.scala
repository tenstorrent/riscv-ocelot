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
  VecBundles — every bundle that crosses a boundary between two vector nodes.

  hierarchy.yaml: kind: package, mode: new,
  output src/main/scala/v4/vec/generated/VecBundles.scala,
  package boom.v4.vec.generated. depends_on MicroOp, VectorParams.

  PACKAGE NODE CONVENTION. A `kind: package` emitting Chisel `Bundle` classes
  and nothing else — no Module, no I/O of its own, no state.
  The parameters section states what the bundles are parameterized on, the
  ports section is explicitly empty, and the logic section declares the bundles.
  A bundle's field list IS its specification, so the logic section is a set of
  field declarations plus the invariant each bundle carries.

  This file exists because a bundle is a CONTRACT WITH TWO SIDES. Declaring
  each one once, here, is what makes the seam reviewable from both ends in plan
  step R2; a bundle declared inside its producer would be readable only from
  the producing side.

  Governing spec anchors: midcore.rst (group-done and the completion model),
  loadstore.rst `ssi-queues` and `elem-progress` (the element access and the
  queue set), cii.rst `cii-interface` (the four channel payloads),
  issue.rst `vec-queue-reservation`.
*/

<|begin_module|>

  <|begin_parameters|>
  No constructor parameters of its own. Each bundle is parameterized implicitly
  through Chisel's `Parameters` on the sizes it needs from VectorParams:
  `vLen`, `eLen`, `vecPregSz`, `vlPregSz`, `maxMembers`, `vecVLSz`,
  `ciiTagBits`, `numSrcSlots`, and from `HasBoomCoreParameters` on `robAddrSz`,
  `ldqAddrSz`, `stqAddrSz` and `coreWidth`.

  No width in this file may be a literal. Every one is derived from a
  VectorParams field or a BOOM core parameter, so that a sizing change ripples
  instead of desynchronising one side of a seam. The CII widths are a stricter
  case still: they must derive from the VectorParams MIRRORS of
  `tt_cii_caracal_pkg.svh`'s localparams (`CII_TAG_W`, `CII_NUM_SRC_SLOTS`,
  `CII_VL_W`, `CII_MEMBER_W`), because a flat BlackBox port disagreeing by one bit
  shifts a whole payload without a width error — see `src_reuse_hint` below.
  <|end_parameters|>

  <|begin_ports|>
  None. This is a declaration unit: no I/O, no clock, no reset.
  <|end_ports|>

  <|begin_logic|>
  ---- VecGroupDone: the completion event ----

  //@req-spec-core.g2
  //@req-spec-issue.f6
  `VecGroupDone` announces that one whole destination group has completed. It
  carries the completing group's FULL MEMBER-PRN VECTOR — a `Vec(maxMembers,
  UInt(vecPregSz.W))` of member PRNs together with a `members` count giving how
  many of them are valid — plus the `rob_idx` of the owning OP.v and an
  `is_vl_producer`-style `pvl` field where the producer also wrote VL.

  It carries the member PRNs and not a group base plus a size because a group's
  members need not be contiguous: the free list allocates a group without
  requiring contiguous PRNs, so a base-plus-count encoding could not name the
  group that was actually allocated. A consumer matches each of its source
  group's members against this vector.

  // ONE event, THREE consumers: the ROB's single-shot rob_bsy clear, the vector
  // Busy-Table clear, and the vector wakeup network. That is why it is one
  // bundle rather than three narrower ones — the three consumers must see the
  // same completion in the same cycle, and a split bundle would let them drift.

  ---- VecMemberRdy: the per-member readiness side channel ----

  `VecMemberRdy` carries per-member operand readiness alongside a uOP, from the
  rename space to the issue queues. **FIVE per-member groups plus the mask bit:**
  `vs1_rdy`, `vs2_rdy`, `vs3_rdy`, `vtmp_rdy` and `vold_rdy`, each
  `Vec(maxMembers, Bool)`, plus `vm_rdy: Bool`. Take NO parameter — size the
  groups from `maxMembers`, which this package already has in scope.

  The mask stays a single `Bool` because `pvm` names ONE register, not a group,
  so "five groups and six fields" is the same statement, not a discrepancy.
  `vold_rdy` is `stale_pvdest`'s per-member readiness (decision D6), feeding the
  `rdy_vold` matcher on `IQ_V_LOAD` and `IQ_V_ALU` slots — the CII reads the old
  destination as a source, so it is a real dependency and not a duplicate of
  `vs3_rdy`. This is the READY sense; `VecBusyTable`'s `VecMemberBusyResp` is the
  BUSY sense and stays local to it, with `VecRenameSpace` converting between them.

  // ===> IT IS DECLARED HERE, ONCE, AND EVERY OTHER SITE BINDS TO IT. This bundle
  // is the one `VecPipeline` part 13 rules on: "`VecSlotMemberRdy` and
  // `VecMemberRdy` ARE ONE BUNDLE WITH TWO NAMES, and that is a defect, not a
  // synonym." Three specs (`VecIssueSlot`, `VecIssueUnit`, `VecPipeline`) already
  // said the single declaration belongs in `VecBundles` — and this package never
  // declared it, so the two consumers each declared their own: `VecRenameSpace`
  // emitted `VecMemberRdy(maxGroupSize)` and `VecIssueSlot` a local
  // `VecIssueSlotMemberRdyShim`, structurally identical types facing each other
  // across one seam that `VecIssueUnit` must connect. It would not have compiled.
  // Added here 2026-08-10; `VecRenameSpace`'s spec amended to bind rather than
  // declare. Both generated shapes had already converged on the five-group layout
  // above, so this promotion is a rename, not a redesign.
  //
  // It belongs here by the same test as `VecScalarOperands` and `VecRobFlags`: it
  // crosses a boundary BOTH sides must review. A declaration inside a producer is
  // readable from one side only.

  ---- VecElemAccess: the nOP.v ----

  `VecElemAccess` is the cracked element access — the "nOP.v" — that address
  generation emits and the drain side consumes. It wraps the originating OP.v's
  `MicroOp` and adds the access payload: the virtual address, the `eew`, a byte
  enable, and `first`/`last` markers for the group.

  The fields identifying WHICH register the access targets — the destination PRN
  and the byte offset within it — are the nOP.v-scoped cursor fields of the
  wrapped `MicroOp`, not new fields here; see the MicroOp delta spec, which owns
  that obligation. Declaring them a second time in this bundle would give the
  drain side two places to read the same thing from, and one of them would go
  stale.

  ---- The element queue set ----

  //@req-spec-lsu.a14
  //@req-spec-lsu.a15
  //@req-spec-agen.a5
  Declare the queue set as a named enumeration rather than leaving the six
  queues as six unrelated instances, so that a module naming a queue cannot
  name one that does not exist. The set comprises exactly six members and the
  names are normative, following the `{ld,st}_{SSI,US}_{ADDR,DATA}_Q`
  convention: `ld_SSI_ADDR_Q`, `st_SSI_ADDR_Q`, `st_SSI_DATA_Q`,
  `ld_US_ADDR_Q`, `st_US_ADDR_Q` and `st_US_DATA_Q`.

  // There is deliberately no ld_*_DATA_Q in either class: a load's returning
  // data goes to the LCB for assembly, not into a queue. The asymmetry is real
  // and the enumeration should make it obvious rather than leave a reader
  // wondering which two entries are missing.

  Address generation delivers its nOP.v bundles into these dedicated queues and
  nowhere else — never into an LDQ or STQ slot, which hold one placeholder
  entry per vector instruction for ordering and commit only.

  `VecRangeEntry` is the unit-stride counterpart of `VecElemAccess`: one entry
  describing a whole contiguous byte range. One of these stands for what would
  otherwise be up to `vLen` element accesses.

  ===> THE FIELD LIST BELOW IS THE COMPLETE AUTHORITATIVE ENUMERATION, AND IT IS
       STATED IN FULL FOR A REASON. The original list here was written from the
       PRODUCER's point of view — what `VecRangeAgen` needs to emit — and FOUR
       separate consumers then each discovered, independently, a field the drain
       side cannot do without. That pattern is the warning: the entry is the ONLY
       thing that remembers a unit-stride instruction (which is precisely how
       `VecRangeAgen` stays free of per-instruction state), so anything the drain
       side, the LCB, the forwarder or the squash unit needs about that instruction
       must be ON THE ENTRY. A generator must emit exactly this list; a consumer
       needing a fifth thing amends this list rather than deriving it locally.

    `base`            — the effective base virtual address of the range.
    `len`             — TOTAL ACTIVE BYTE LENGTH. Sized for a whole LMUL=8 group of
                        BYTES, `log2Ceil(maxMembers * vLen / 8 + 1)` = 9 bits, and
                        NEVER from `vecVLSz`: the two are the same width and a
                        different QUANTITY (`len` counts bytes, `vecVLSz` sizes an
                        element count), coinciding only because the narrowest SEW is
                        one byte.
    `eew`             — the element width. It travels even though the address
                        arithmetic no longer needs it once `len` is known, because
                        the drain side scales a mask bit into a byte enable with it
                        and the LCB needs it for placement.
    `stride`          — the EFFECTIVE byte stride (`1 << eew` for unit-stride, 1 for
                        the whole-register and mask forms). Carried explicitly even
                        though it is derivable from `eew`, so the drain side decodes
                        ONE self-describing entry format instead of special-casing
                        three access classes (spec-agen.b7).
    `is_unit_stride`  — set high on every entry in these queues. It is what tells
                        `VecBeatExpander` to coalesce rather than issue one access
                        per entry, and what tells `VecCrossLsuSnoop` and
                        `VecStoreForward` that this entry is a RANGE (spec-agen.b7).
    `is_ff`           — FAULT-ONLY-FIRST. `vle<eew>ff.v` is architecturally
                        unit-stride, so its OP.v self-selects into `VecRangeAgen` and
                        never reaches the element agen — but the FAULT is raised on
                        the DRAIN side, against this RETAINED entry. Without this bit
                        the drain side cannot tell a fault-only-first load from an
                        ordinary one and would TRAP on an element `i > 0` fault that
                        must instead TRIM VL. Assert `is_unit_stride` on every entry
                        carrying it.
    `nf`              — the segment field count, read by `VecBeatExpander`'s segment
                        constraint (`1 << eew` when `nf > 1`, unbounded when
                        `nf == 1`). In practice `nf > 1` does not reach the US
                        queues, so this is a representability safety net rather than
                        a throughput case — but the constraint cannot be written
                        without the field.
    `mask`            — the access's ACTIVE MASK (see the note below on its
                        granularity, which is an open cross-file disagreement).
    `pvdest_base`,
    `members`         — the destination group's base PRN and member count, from
                        `pvdest`/`v_emul`.
    `us_data_base`    — for a STORE, the absolute base INDEX of this access's region
                        in `st_US_DATA_Q`, filled from reservation slot 1.
    `rob_idx`,
    `ldq_idx`/`stq_idx` — the ownership fields, so the drain side, the LCB and the
                        squash unit can each name the instruction from the entry
                        alone.

  ===> `us_data_base` IS NOT DERIVABLE FROM THE ADDRESS REGION'S BASE, and this is
       the seam most likely to be mis-generated. A US store's ADDRESS and DATA
       regions are NOT in identity correspondence: `st_US_ADDR_Q` holds ONE range
       entry per store while `st_US_DATA_Q` holds one full `vLen` entry PER GROUP
       MEMBER, so the store claims 1 address entry and `v_emul * nf` data entries and
       the reservation carries a base PER QUEUE SLOT. `VecStoreForward` reads the
       forwarding member at `us_data_base + member` with
       `member = (paddr - base) >> log2(vLenBytes)`; the equal-count /
       one-shared-base assertion holds for the SSI pair ONLY.

  ===> THE MASK MUST TRAVEL ON THE RANGE ENTRY, and this is the one field a
       reader is most likely to think redundant. The mask is read ONCE per OP.v on
       the fill side and latched, but a unit-stride access is not expanded until
       the DRAIN side coalesces it — so the drain-side coalescer needs the mask
       and has no other source for it: `VecRangeAgen` instantiates no mask reader,
       and re-reading `v0` at drain would need a VRF port the canonical table does
       not grant. Without this field the masked unit-stride case has no carrier at
       all.
       It is a BYTE mask rather than an element mask so that one field serves both
       purposes: the coalescer suppresses accesses for masked-off lanes with it,
       and `VecStoreForward` qualifies a forward with it. Note that ORDERING is
       deliberately mask-oblivious and must not consult it.

  // ===> REPORTED, NOT RESOLVED — THE MASK'S GRANULARITY AND WIDTH ARE CONTESTED
  // ACROSS FOUR FILES, and a generator must not pick a side silently. This file
  // says BYTE mask, `vLen/8` = 32 bits (the paragraph above). Three consumers read
  // it as ELEMENT-granular and wider: `VecRangeAgen` states "the mask is
  // ELEMENT-granular, so the drain side needs `eew` to scale a mask bit into a byte
  // enable"; `VecBeatExpander`'s constraint 5 measures "the run of consecutive
  // ACTIVE mask bits starting at `elem_next`, converted to bytes by `<< eew`";
  // `VecMaskStream` sizes its carriage as "up to VLMAX mask bits on one bundle per
  // OP.v", i.e. 256. The two readings are not interchangeable — a byte mask needs no
  // `eew` to become a byte enable, which is the very reason this list carries `eew`
  // — and `vLen/8` cannot cover a range that spans up to `maxMembers * vLen / 8`
  // bytes at LMUL=8. One granularity and one width must be chosen for all four
  // files in one edit, with the affected `spec-agen.e13`/`spec-agen.e12` text
  // checked against it. Recorded here because this is the declaration site, and a
  // field silently declared at 32 bits that three consumers index past is a
  // truncation with no error anywhere.

  // ===> CONSIDERED AND NOT ADDED: the two RETAINED PPNs `VecBeatExpander` asked
  // for, to spare its post-commit write pass a re-translation of an address its
  // pre-commit translate pass already resolved. Not added because it makes the
  // entry hold POST-TRANSLATION state, which changes what a squash and the
  // `is_write_pass` cursor reset must invalidate, and because the bound "two" rests
  // on a range of at most `maxMembers * vLen / 8` = 256 bytes crossing at most one
  // page boundary — an argument that must be written down and checked against the
  // page size, not assumed. Two lookups per beat-pass remains the specified
  // behaviour until that review happens; this note exists so the next reader knows
  // it was weighed rather than missed.

  ---- VecReservation ----

  `VecReservation` is the dispatch-time capacity claim: which queue, how many
  entries, and the owning `rob_idx` plus the reserving `ldq_idx`/`stq_idx`. It
  carries the reservation's index region so a squash can roll a queue's tail
  pointer back to the youngest surviving reservation without a per-entry
  comparison.

  ---- VecException ----

  `VecException` reports a vector memory fault to the ROB as a plain precise
  exception: `valid`, `rob_idx`, `cause`, `badvaddr`. It deliberately carries NO
  element index — a faulting vector op traps with `vstart = 0` and restarts
  whole, so an element index reaching the ROB could only be misused.

  ---- The four CII channel payloads ----

  These four bundles are the Chisel view of the frozen SV contract in
  `tt_cii_caracal_pkg.svh`. Every width below is derived from `ciiTagBits`,
  `vLen` or a named constant of that package — never written as a literal —
  because the SV side is authoritative and a disagreement here is a silent
  protocol break, not a compile error.

  ===> ALL FOUR ARE PER-LANE PAYLOADS, AND THE CHANNEL'S GRAIN IS THE BEAT.
       `tt_cii_interface.sv` gives each channel exactly ONE `valid` and ONE
       `credit` for a beat of N lanes (`CII_NUM_SRC_REQ` = `CII_NUM_SRC_DAT_RSP` =
       4 on the two source channels, 1 elsewhere); there is no per-lane valid and no
       per-lane credit anywhere in the interface. So NO bundle here may grow a
       `valid`, `credit`, `ready` or lane-index field: per-lane ACTIVITY is encoded
       in the payload, as `op_id = CII_SRC_NONE` on an unused Src-Request lane, and
       an inactive lane is answered with a defined don't-care beat on the SAME lane
       index rather than being compacted away. The lane mapping is STRAIGHT-THROUGH
       in both directions; the only index a payload carries is a MEMBER index
       (`op_offset`, `wb_dst_offset`), never a lane index and never a register
       number.

  //@req-spec-cii.a6
  //@req-spec-cii.a14
  `CiiIssueReq` (host to coprocessor) carries `tag` (`ciiTagBits`), `instr`
  (32 bits, the raw RVV word), a `vtype` field holding {vsew, vlmul, vta, vma},
  `vl`, `vstart`, `vxrm`, `frm`, and a per-source `src_reuse_hint` of
  `numSrcSlots` = 4 BITS, derived from `CII_NUM_SRC_SLOTS`.
  The `vtype`, `vl` and `vxrm` fields are the Caracal EXTENSION to the generic
  `tt_cii_interface.sv` issue struct: the generic struct carries tag and
  instruction only, and these three are what let the coprocessor hold no
  cross-instruction configuration state.

  ===> `src_reuse_hint` IS FOUR BITS, NOT THREE. The frozen package types the
       corresponding field as `cii_caracal_frwd_hint_t = logic
       [CII_NUM_SRC_SLOTS-1:0]`, i.e. 4 — one bit per hintable source slot
       (VS1, VS2, VS3, VM) — and the flat `iss_hint` port of the BlackBox is 4 bits
       wide. An earlier draft of this file said 3. The host drives the field to ZERO
       on every source either way and the VPU ignores it in M2, so nothing about
       BEHAVIOUR depends on the width — but the ISSUE PACKET IS A FLAT BLACKBOX
       PORT, so a one-bit disagreement shifts every field packed above it and the
       coprocessor decodes a different instruction than the host sent, with no
       width error and no assertion. THE FROZEN SV VALUE WINS, the same precedent as
       `spec-vrf.h6`; derive from `CII_NUM_SRC_SLOTS` and never write 3 or 4.
       cii.rst's `src_reuse_hint[3]` row is wrong for the same reason and is a
       `/spec-to-reqs` follow-up, not something to mirror here.

  //@req-spec-cii.a8
  `CiiSrcReq` (coprocessor to host) carries `tag` (`ciiTagBits`), `op_id` (3
  bits, the abstract source slot) and `op_offset` (3 bits, the group member
  index). Note what is absent: no register number of any kind, architectural or
  physical. The coprocessor names operands by slot and member only.

  //@req-spec-cii.a10
  `CiiSrcData` (host to coprocessor) carries `data`, `vLen` bits wide, and
  nothing else — not even the tag. The channel is ordered, so the coprocessor
  correlates a beat with its request by arrival order rather than by a field.
  // That is exactly why a killed tag's request must still be answered: an
  // omitted beat would desynchronise the channel for every surviving
  // instruction. See VecCiiFlush.

  //@req-spec-cii.a12
  //@req-spec-cii.a15
  `CiiWriteback` (coprocessor to host) carries `tag`, `wb_data` (`vLen` bits),
  `wb_dst_offset` (3 bits, the destination member index), `wb_wr_en` (per-beat
  write enable) and `wb_status` (9 bits). Declare `wb_status` as a named
  sub-bundle of {`last`, `dst_kind`, `vxsat`, `fflags`} rather than a bare
  9-bit field, so a consumer cannot slice it by hand and get the fields wrong.
  The `last` bit is the Caracal extension to the generic writeback struct and
  marks the final beat of a tag.

  // `last` is the ONLY completion signal: the channel carries no expected-count
  // field, and the host must never infer completion by counting beats. Widening
  // and narrowing ops emit a member count that differs from the source EMUL, so
  // a count derived on the host would be wrong for exactly those cases.

  ---- VecPipelineIO ----

  `VecPipelineIO` realizes the `vec_pipeline_io` interface declared in
  hierarchy.yaml — the single bundle across which BoomCore and VecPipeline
  communicate. Declare it here, field for field, matching that interface entry.

  // ===> THE RENAME INPUTS ARE NAMED ren2_uops AND dis_fire, NOT dec_uops, and
  // the name is load-bearing rather than cosmetic. Vector rename must allocate
  // in lockstep with the scalar RenameStage's REGISTERED ren1->ren2 pipeline.
  // Driving it combinationally from dec_uops runs it one cycle ahead, so at
  // dispatch the vector fields describe the NEXT cycle's (bubble) uop and two
  // ops free the same PRN. With these port names, connecting dec_uops here is
  // visibly wrong at the connection site.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
No behaviour, so no throughput target. Two structural constraints do belong
here, because both are about what the bundles must not force:

Nothing declared here may imply a pipeline stage. These are wire bundles; any
registering is the choice of the module that uses them, not of the declaration.

`VecGroupDone` is matched per member against every wakeup port in every vector
issue slot each cycle, so its width is multiplied by (slots x ports) in the
issue-stage comparator budget. Keep it to the member PRNs, the count and the
ownership fields — a field added here is paid for many times over in issue.
<|end_perf|>

<|begin_dependencies|>
MicroOp — `VecPipelineIO` and several bundles carry `new MicroOp()`, and
`VecElemAccess` copies the OP.v's cursor fields.
VectorParams — every width derives from it.

Binds to `freechips.rocketchip.rocket.VConfig`/`VType` for the `vtype` field of
`CiiIssueReq`, for the same reason VtypeTable does: rocket's `CSRFile` owns
architectural `vtype`, so the issue packet must carry rocket's encoding of it
rather than a re-spelled one. Note that this binding is a REPACK and not a slice:
rocket's field order is `{vill, reserved, vma, vta, vsew, vlmul_sign, vlmul_mag}`
and its deprecated `vlmul` accessor returns only `vlmul_mag`, so building the 8-bit
`{vsew, vlmul, vta, vma}` field through that accessor silently drops the
fractional-LMUL sign and turns every mf2/mf4/mf8 op into m1/m2/m4 with no width
error.

// ===> REPORTED, NOT RESOLVED — `VecCiiTagEntry` HAS NO DECLARATION SITE.
// hierarchy.yaml's entry for this node lists `VecCiiTagEntry` among this package's
// declarations, and VecCiiIssue, VecCiiOperandServer, VecCiiWriteback and
// VecCiiComplete all bind it expecting to find it here — but this file does not
// declare it, and VecCiiTagTable declares it locally instead, on the
// `VecBusyResp`-in-`VecBusyTable` precedent, arguing that it crosses only
// boundaries between siblings inside VecCiiHost. Both positions are defensible and
// they cannot both be generated: one declaration, one home. Recorded from this end
// as well as the other four so the discrepancy cannot be closed by each side
// assuming the other did it. It is NOT resolved by declaring the type twice.

Instantiates nothing. Its dependents are VecPipeline and effectively every
`vec/**` module, so a field change here has the widest blast radius of any node
in the map after MicroOp — check the `depends_on:` fan-in before editing.
<|end_dependencies|>

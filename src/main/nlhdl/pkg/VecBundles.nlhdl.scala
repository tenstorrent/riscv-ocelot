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

/* VecBundles — every bundle that crosses a boundary between two vector nodes. */

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

  ===> `VecGroupDone` IS `rob_idx`-KEYED, AND THAT IS CORRECT — BUT IT IS NOT SUFFICIENT
  ON ITS OWN. Found at E7. A completing vector load has TWO consumers that key on
  DIFFERENT identifiers: the ROB clears busy by `rob_idx`, and `lsu.scala` marks the load
  executed by `ldq_idx` (`GetRealLSQIdx(...)` indexes `ldq_executed`/`ldq_will_succeed`
  directly). Neither identifier is derivable from the other without a shadow table, which
  ground rule 6 forbids.
  Do NOT add `ldq_idx` to this bundle: it is the ROB-facing completion event and a
  memory-subsystem index has no meaning to the other producers that drive it (the CII
  writeback has no LDQ entry at all). The LOAD-SIDE producer publishes the LDQ-keyed
  completion SEPARATELY — `VecLoadCoalescingBuffer.group_done_ldq`, asserted in the same
  cycle from the same winner — and `VecLsu` routes that to the host seam.
  This mattered: with only the rob_idx form available, the container tied the host's
  `ld_group_done` off rather than wire it wrong. That is the right call and it is also a
  MACHINE THAT NEVER RETIRES A VECTOR LOAD — the LDQ entry is never marked executed, so
  the first vector load hangs at the ROB head. A completion path that is correct for one
  of its two consumers and absent for the other looks like a wiring gap and behaves like
  a deadlock.

  ONE event, THREE consumers: the ROB's single-shot rob_bsy clear, the vector
  Busy-Table clear, and the vector wakeup network. That is why it is one
  bundle rather than three narrower ones — the three consumers must see the
  same completion in the same cycle, and a split bundle would let them drift.

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

  ===> IT IS DECLARED HERE, ONCE, AND EVERY OTHER SITE BINDS TO IT. This bundle
  is the one `VecPipeline` part 13 rules on: "`VecSlotMemberRdy` and
  `VecMemberRdy` ARE ONE BUNDLE WITH TWO NAMES, and that is a defect, not a
  synonym." Three specs (`VecIssueSlot`, `VecIssueUnit`, `VecPipeline`) already
  said the single declaration belongs in `VecBundles` — and this package never
  declared it, so the two consumers each declared their own: `VecRenameSpace`
  emitted `VecMemberRdy(maxGroupSize)` and `VecIssueSlot` a local
  `VecIssueSlotMemberRdyShim`, structurally identical types facing each other
  across one seam that `VecIssueUnit` must connect. It would not have compiled.
  Added here 2026-08-10; `VecRenameSpace`'s spec amended to bind rather than
  declare. Both generated shapes had already converged on the five-group layout
  above, so this promotion is a rename, not a redesign.

  It belongs here by the same test as `VecScalarOperands` and `VecRobFlags`: it
  crosses a boundary BOTH sides must review. A declaration inside a producer is
  readable from one side only.

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

  There is deliberately no ld_*_DATA_Q in either class: a load's returning
  data goes to the LCB for assembly, not into a queue. The asymmetry is real
  and the enumeration should make it obvious rather than leave a reader
  wondering which two entries are missing.

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
    `pvdest`,
    `members`         — the destination group's FULL PER-MEMBER PRN VECTOR
                        (`Vec(maxVecMembers, UInt(vecPregSz.W))`) and member count,
                        copied verbatim from `MicroOp.pvdest` / `v_emul`.

  ===> THIS FIELD WAS `pvdest_base`, A SINGLE BASE PRN, AND THAT WAS THE MOST SERIOUS
  DEFECT FOUND IN PHASE E. Corrected at E6. **A RENAMED GROUP'S MEMBER PRNs ARE NOT
  CONTIGUOUS.** `VecFreeList` says so explicitly in its own prose — it hands each lane a
  contiguous window of SELECTOR PORTS, and "the window is contiguous in PORT INDEX only —
  the PRNs those ports hold are not" — which is exactly why `MicroOp.pvdest` is a `Vec`
  and why `VecGroupDone` carries a full member vector rather than base+count.
  With a single base, `VecBeatExpander` computed `pvdest_base + memberIdx` and
  `VecRangeAgen` computed `srcGroupBase + (membersUsed - 1)`. For member 0 both are right;
  for every member above it they name AN UNRELATED PHYSICAL REGISTER belonging to some
  other in-flight instruction. So every LMUL>1 unit-stride load wrote its upper members
  into someone else's registers, and every multi-member unit-stride store read its data
  from them — with no assertion, no width error, and a passing LMUL=1 test suite.
  ALWAYS INDEX THE GROUP VECTOR. Never base plus offset, anywhere in this design, for any
  of `pvdest`/`pvs3`/`pvtmp`/`stale_pvdest`. If a bundle needs a group, it carries the
  vector; the width cost is `maxVecMembers * vecPregSz` bits and it buys the only
  representation that is correct.
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

  ===> REPORTED, NOT RESOLVED — THE MASK'S GRANULARITY AND WIDTH ARE CONTESTED
  ACROSS FOUR FILES, and a generator must not pick a side silently. This file
  says BYTE mask, `vLen/8` = 32 bits (the paragraph above). Three consumers read
  it as ELEMENT-granular and wider: `VecRangeAgen` states "the mask is
  ELEMENT-granular, so the drain side needs `eew` to scale a mask bit into a byte
  enable"; `VecBeatExpander`'s constraint 5 measures "the run of consecutive
  ACTIVE mask bits starting at `elem_next`, converted to bytes by `<< eew`";
  `VecMaskStream` sizes its carriage as "up to VLMAX mask bits on one bundle per
  OP.v", i.e. 256. The two readings are not interchangeable — a byte mask needs no
  `eew` to become a byte enable, which is the very reason this list carries `eew`
  — and `vLen/8` cannot cover a range that spans up to `maxMembers * vLen / 8`
  bytes at LMUL=8. One granularity and one width must be chosen for all four
  files in one edit, with the affected `spec-agen.e13`/`spec-agen.e12` text
  checked against it. Recorded here because this is the declaration site, and a
  field silently declared at 32 bits that three consumers index past is a
  truncation with no error anywhere.

  ===> CONSIDERED AND NOT ADDED: the two RETAINED PPNs `VecBeatExpander` asked
  for, to spare its post-commit write pass a re-translation of an address its
  pre-commit translate pass already resolved. Not added because it makes the
  entry hold POST-TRANSLATION state, which changes what a squash and the
  `is_write_pass` cursor reset must invalidate, and because the bound "two" rests
  on a range of at most `maxMembers * vLen / 8` = 256 bytes crossing at most one
  page boundary — an argument that must be written down and checked against the
  page size, not assumed. Two lookups per beat-pass remains the specified
  behaviour until that review happens; this note exists so the next reader knows
  it was weighed rather than missed.

  ---- VecReservation ----

  `VecReservation` is the dispatch-time capacity claim: which queue, how many
  entries, and the owning `rob_idx` plus the reserving `ldq_idx`/`stq_idx`. It
  carries the reservation's index region so a squash can roll a queue's tail
  pointer back to the youngest surviving reservation without a per-entry
  comparison.

  ---- VecException ----

  `VecException` reports a vector memory fault to the ROB as a plain precise
  exception, and it is DECLARED TO BE FIELD-FOR-FIELD THE SAME SHAPE AS
  `rob.scala`'s `class Exception`: `uop` (a full `MicroOp`), `cause`
  (`log2Ceil(Causes.all.max + 2)` bits — rocket's cause space, NOT `xLen`; the
  bundle's only consumer assigns it straight onto `rob.io.lxcpt.bits.cause` and a
  wider field there would silently truncate), and `badvaddr` (`coreMaxAddrBits`).
  It deliberately carries NO element index — a faulting vector op traps with
  `vstart = 0` and restarts whole, so an element index reaching the ROB could
  only be misused.

  ===> RESOLVED AT E-PREP; THE HISTORY IS KEPT BECAUSE THE FAILURE MODE IS
  INVISIBLE. As originally declared — `{valid, rob_idx, cause, badvaddr}` — this
  bundle COULD NOT drive `rob.io.lxcpt`, and was inert only because of D2 staging.
  Found by the BoomCore delta at D2, which has to merge it with `io.lsu.lxcpt` by
  age into `rob.io.lxcpt`. That port is `Valid(new Exception)` and the ROB's latch
  does `next_xcpt_uop := new_xcpt.uop` and then reads `uop.br_mask` for
  `GetNewBrMask` (`rob.scala`) — i.e. the ROB needs a MicroOp, and the bundle had
  none, so BoomCore populated `.rob_idx` and left the rest `DontCare`. Safe only
  while `VecPipeline`'s D2 staging tied `vec_xcpt.valid` false.

  WHY IT HAD TO BE FIXED BEFORE `VecLsu` LANDS AT E7, i.e. what a future edit must
  not undo: an unpopulated `br_mask` on a latched exception makes `GetNewBrMask`
  compute against garbage, so a vector fault taken while a branch is in flight is
  either dropped or attributed to the wrong instruction — a precise-exception bug
  that no width check and no assertion catches. The `uop` is the faulting `OP.v`'s,
  which the LSU has in hand. Do NOT reconstruct one in BoomCore from `rob_idx`:
  `rob_idx` cannot recover `br_mask`.

  Two fields were DELETED in the same edit, and neither may come back. `valid` is
  redundant: every site nests this bundle inside a `Valid(...)` wrapper and nothing
  read the inner bit. `rob_idx` is redundant *and dangerous*: `uop.rob_idx` is the
  same number, and two independently-driven copies of it is exactly the divergence
  class that `MicroOp.v_vl_imm` was introduced at D3 to close.

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
  That is exactly why a killed tag's request must still be answered: an
  omitted beat would desynchronise the channel for every surviving
  instruction. See VecCiiFlush.

  //@req-spec-cii.a12
  //@req-spec-cii.a15
  `CiiWriteback` (coprocessor to host) carries `tag`, `wb_data` (`vLen` bits),
  `wb_dst_offset` (3 bits, the destination member index), `wb_wr_en` (per-beat
  write enable) and `wb_status` (9 bits). Declare `wb_status` as a named
  sub-bundle of {`last`, `dst_kind`, `vxsat`, `fflags`} rather than a bare
  9-bit field, so a consumer cannot slice it by hand and get the fields wrong.
  The `last` bit is the Caracal extension to the generic writeback struct and
  marks the final beat of a tag.

  `last` is the ONLY completion signal: the channel carries no expected-count
  field, and the host must never infer completion by counting beats. Widening
  and narrowing ops emit a member count that differs from the source EMUL, so
  a count derived on the host would be wrong for exactly those cases.

  ---- The host-seam declarations A2 deferred to D2 ----

  Five members of `vec_pipeline_io` were OMITTED at A2 rather than declared wrong,
  each with a note in the generated file naming step D2 as its owner. This section
  settles all five. It is where `VecPipeline`'s amendment 1 ("STILL REQUIRED:
  declare `IntWakeupBus`, `FpWakeupBus`, `IntWbSnoop` and `VecRobFlags` in
  `VecBundles`") is discharged, and the A34 assignment with it.

  ===> THREE OF THE FIVE NAMES IN hierarchy.yaml ARE DESCRIPTIONS OF A SHAPE, NOT
       REQUESTS FOR A NEW TYPE, and generating a class for them would be the WORSE
       outcome. `IntWakeupBus`, `FpWakeupBus` and `DecoupledReadReq` each describe
       something BOOM already declares:
         - a wakeup bus is `Vec(n, Valid(new Wakeup))`, and `Wakeup`
           (`execution-unit.scala`) carries `uop`, `bypassable`,
           `speculative_mask` and `rebusy`. A structurally-equal Caracal copy
           would FORK that bundle: a field added to BOOM's `Wakeup` would reach
           the scalar issue units and not the vector ones, with no width error.
           ===> BUT THE BUS IS MORE THAN THE `Vec`, AND THE FIRST VERSION OF
           THIS PARAGRAPH GOT THAT WRONG. `IntWakeupBus` in `VecPipeline`'s
           ports section is an AGGREGATE of three things: the wakeup `Vec`,
           `child_rebusys: UInt(aluWidth.W)` and `squash_grant: Bool`. This
           section originally declared only the `Vec`, on the reasoning above
           — which is right about the `Vec` and silently dropped the other
           two. `VecPipeline` then found that every `VecIssueUnit` instance
           declares both as unconditional inputs, could not compute either
           (both are BoomCore-internal:
           `alu_exe_units.map(_.io_squash_iss).reduce(_||_)`), and tied them
           to `0.U`/`false.B` as "safe, no-effect defaults".

           THEY ARE NOT NO-EFFECT, AND THE TIE-OFF IS A SILENT-CORRUPTION
           BUG. `VecIssueUnit` uses `child_rebusys` to RE-MARK A SLOT BUSY
           when a speculatively-woken scalar `.vx`/`.vf` feeder's parent load
           misses — it is the retraction half of BOOM's speculative wakeup.
           Held at zero, the retraction never arrives and the vector op
           issues against a stale GPR, with no assertion anywhere.
           `squash_grant` is the same shape of mechanism one stage later.

           So declare BOTH as their own seam members, driven by BoomCore:
             val int_child_rebusys = Input(UInt(aluWidth.W))
             val int_squash_grant  = Input(Bool())
           separate members rather than a wrapper bundle, for the same reason
           the `Vec` is bare: they are BOOM's own terms, and a Caracal
           aggregate around them would be a second place to keep in step.
           There is deliberately no FP counterpart — BOOM has no FP analogue
           of either term.
         - `DecoupledReadReq` is `Decoupled(UInt(addrWidth.W))` — the exact type of
           `RegisterFile.io.arb_read_reqs`, which is `Flipped(Decoupled(UInt(
           log2Ceil(numRegisters).W)))`. Its whole purpose is to be assignable to
           that port with `<>`; a bespoke bundle could not be.
       So these three are declared AS THE SEAM MEMBERS THEMSELVES, in
       `VecPipelineIO`, in terms of BOOM's own types. The two that DO carry new
       structure — `VecRobFlags` and `IntWbSnoop` — become real classes here.
       This is the resolution, not a deferral: the requirement was that the shapes
       be declared in this file, and they are.

  ---- VecRobFlags (this is A34) ----

  `VecRobFlags` is the CSR side effect of a vector instruction on its way to
  COMMIT: `{rob_idx: robAddrSz, fflags: FPConstants.FLAGS_SZ, vxsat: Bool}`. It
  crosses as `Vec(numVecClrPorts, Valid(new VecRobFlags))`, one lane per completion
  producer, matching `vec_clr_bsy` lane for lane — lane 0 the LCB group-done, lane 1
  `VecCiiComplete`, lane 2 `VecGroupCopy`.

  It lives here, not inside its producer, by part 13's test: `VecPipeline` emits it
  and `Rob` consumes it, so both sides must review the declaration, and `VecBundles`
  already declares `VecGroupDone`, the bundle it travels beside.

  ===> IT IS APPLIED AT COMMIT AND NEVER AT WRITEBACK, and the ROB has an assertion
       that will catch a generator which forgets: `rob.scala:405` asserts
       `!rob_fflags(row_idx).valid` on a write, so a vector op must write the
       per-entry fflags slot AT MOST ONCE. That is also why a CII scalar-dest
       writeback must leave `ExeUnitResp.fflags` invalid and route its flags
       through this bundle instead — two paths into one slot fires the assert.
       Pulsing `csr.io.vector.set_vxsat` at writeback is separately wrong: a
       past-PNR CII op can still be squashed by a ROB-head flush, so a writeback
       pulse dirties architectural state for an instruction that never retires.

  ---- IntWbSnoop ----

  `IntWbSnoop` is the INT-writeback tap: `{addr, data}`, crossing as
  `Vec(numIrfWritePorts, Valid(new IntWbSnoop))`. It is NOT `int_wakeups` — a
  wakeup carries readiness without a value, and this carries the value.

  ===> `addr` IS `maxPregSz` WIDE, NOT `ipregSz`, because this bundle exists to be
       compared against `RegisterFile.io.write_ports(i).bits.addr`, which
       `regfile.scala:35` declares as `UInt(maxPregSz.W)`. Taking the narrower
       INT-specific width would be arithmetically sufficient on every config and
       still wrong: the comparison would then be between two different widths and
       Chisel would zero-extend one side silently. Match the port being snooped.

  Without this tap the M1 stale-scalar-base bug has no fix. A vector load/store is
  woken speculatively, so `VecScalarOperandRead`'s INT-RF read can fire in the same
  cycle the base GPR's writeback commits — and the INT RF is a registered `Mem`
  read with no read-during-write bypass, so the read returns the OLD base. The
  forward therefore needs the DATA, and the port count must include the
  scalar-dest write port, or a base produced by `vmv.x.s` is missed.

  ---- The CSR seam: there is no `CSRVectorIO` ----

  hierarchy.yaml types `csr_vector` as `CSRVectorIO`. NO SUCH CLASS EXISTS. Rocket
  declares the port anonymously —
  `val vector = usingVector.option(new Bundle { ... })` at
  `rocket-chip/src/main/scala/rocket/CSR.scala:310` — so there is no name to
  reference. Two ways out were weighed and both rejected: restating rocket's bundle
  field-for-field here would drift from it silently, and inventing a Chisel type in
  rocket's name would claim ownership of state ground rule 9 says is rocket's.

  Declare instead `VecCsrRead`, Caracal's own READ-DIRECTION VIEW of that port:
  `{vconfig: freechips.rocketchip.rocket.VConfig, vstart: UInt(maxVLMax.log2.W),
  vxrm: UInt(2.W)}`. Three properties make this the right shape:

    - It uses rocket's `VConfig` for the field that has a name, so `vtype`/`vl`
      cannot drift. Only the two bare-`UInt` fields are restated, and each restates
      rocket's own EXPRESSION (`maxVLMax.log2`, `2`) rather than a literal.
      `.log2` is rocket's `IntToAugmentedInt.log2` (`util/package.scala:236`) —
      `log2Ceil` plus `require(isPow2)` — and it is NOT in scope in this package
      by default. IMPORT IT BY NAME (`freechips.rocketchip.util.
      IntToAugmentedInt`); do not switch the field to `log2Ceil(maxVLMax)` and do
      not wildcard-import rocket's `util`. Gate (a) caught the missing import as
      `value log2 is not a member of Int`, and rewriting it to `log2Ceil` was the
      wrong fix: the point of this field is that it is the same expression rocket
      writes at `CSR.scala:312`, and the `isPow2` require rides along with it.
    - It carries ONLY the output-direction fields, which is exactly what part 8 of
      `VecPipeline` says the container consumes: `vconfig`, `vstart`, `vxrm`. The
      input-direction fields of rocket's bundle — `set_vconfig`, `set_vstart`,
      `set_vxsat`, `set_vs_dirty` — are driven by BoomCore and the `Rob` delta and
      must NOT appear here. That is why `csr_vs_dirty` is a separate backward bit
      on this seam rather than a member of this bundle.
    - Because it holds no write path, it CANNOT be used to write architectural
      vector CSR state, which is the property ground rule 9 actually wants. A
      faithful copy of rocket's bundle would have handed the container one.

  The seam member keeps its name and direction: `csr_vector`, forward,
  `Input(new VecCsrRead)`. BoomCore wires its three fields from
  `csr.io.vector.get`, field by field, at the one site where that port is in scope.

  ---- VecPipelineIO ----

  `VecPipelineIO` realizes the `vec_pipeline_io` interface declared in
  hierarchy.yaml — the single bundle across which BoomCore and VecPipeline
  communicate. Declare it here, field for field, matching that interface entry.
  The five members A2 omitted are now part of that field-for-field obligation:

    val vec_rob_flags   = Output(Vec(numVecClrPorts, Valid(new VecRobFlags)))
    val int_rf_read_req = Vec(5, Decoupled(UInt(ipregSz.W)))
    val int_wakeups       = Input(Vec(numIntWakeupPorts, Valid(new Wakeup)))
    val int_child_rebusys = Input(UInt(aluWidth.W))
    val int_squash_grant  = Input(Bool())
    val fp_wakeups        = Input(Vec(numFpWakeupPorts,  Valid(new Wakeup)))
    val int_wb_snoop    = Input(Vec(numIrfWritePorts,  Valid(new IntWbSnoop)))
    val csr_vector      = Input(new VecCsrRead)

  ===> THE THREE PORT COUNTS ARE CONSTRUCTOR PARAMETERS OF THIS BUNDLE, SUPPLIED BY
       BoomCore FROM THE LENGTH OF THE ACTUAL BUS. `VecPipelineIO` takes
       `numIntWakeupPorts`, `numFpWakeupPorts` and `numIrfWritePorts`, and
       `VecPipeline` takes the same three and passes them through. BoomCore
       constructs it as `new VecPipelineIO(int_wakeups.length,
       fp_pipeline.io.wakeups.length, numIrfWritePorts)`.

       Do NOT re-derive the formulas. `core.scala:113,116` compute
       `numIrfWritePorts = aluWidth + lsuWidth + 1` and
       `numIntWakeups = coreWidth + lsuWidth + 1` INSIDE the `BoomCore` class body,
       so they are not core parameters and this file cannot read them. Copying the
       arithmetic would put a second copy of a scalar-side decision in the vector
       package, where a change to BOOM's writeback set would leave it silently
       stale and mis-size a snoop `Vec` — a truncation with no error. Taking
       `.length` off the bus itself is the only form that cannot drift, and it is
       why these are parameters rather than derived `val`s.
       `numFpWakeupPorts` is `fp_pipeline.io.wakeups.length` for the same reason
       (`core.scala:117` computes it that way already).

  `int_rf_read_req` is declared WITHOUT an explicit direction wrapper: the
  members of `Decoupled` already carry their own directions, and it is connected
  to `iregfile.io.arb_read_reqs` with `<>`. Wrapping it in `Output(...)` would
  flip `ready` the wrong way.

  ===> AND IT MUST BE `Decoupled`, NOT A BARE ADDRESS. `PartiallyPortedRF` DENIES a
       read by index priority, and the INT RF is deliberately partially ported
       (`numIrfReadPorts = 3` on Medium against ~10 logical readers), so denial is
       ROUTINE. Five vector readers against three ports cannot be fixed by any
       affordable port count. Every scalar EU already holds its address until fire;
       these must too, with the response registered at t+1 off the GRANTED address.
       A bare address here would silently read whatever the arbiter granted instead
       — the same class of bug as the stale base above, and just as quiet.

  ===> THE RENAME INPUTS ARE NAMED ren2_uops AND dis_fire, NOT dec_uops, and
  the name is load-bearing rather than cosmetic. Vector rename must allocate
  in lockstep with the scalar RenameStage's REGISTERED ren1->ren2 pipeline.
  Driving it combinationally from dec_uops runs it one cycle ahead, so at
  dispatch the vector fields describe the NEXT cycle's (bubble) uop and two
  ops free the same PRN. With these port names, connecting dec_uops here is
  visibly wrong at the connection site.
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

Binds to `boom.v4.exu.Wakeup` (the two wakeup-bus seam members are taps of the
EXISTING scalar buses, not new types — see the host-seam section) and to
`freechips.rocketchip.tile.FPConstants.FLAGS_SZ` for `VecRobFlags.fflags`, the
same expression `rob.scala:369` uses for the slot those flags land in.

===> RESOLVED 2026-08-10 (step D2) — the five A2-deferred host-seam members.
`vec_rob_flags` (A34), `int_rf_read_req`, `int_wakeups`, `fp_wakeups`,
`int_wb_snoop` and the `csr_vector` type are all settled in the host-seam
section of the logic block above. Three of the six needed no new type; two
became classes (`VecRobFlags`, `IntWbSnoop`); the sixth replaced a reference to
a nonexistent `rocket.CSRVectorIO` with `VecCsrRead`, a read-direction view.

===> REPORTED, NOT RESOLVED — `VecCiiTagEntry` HAS NO DECLARATION SITE.
hierarchy.yaml's entry for this node lists `VecCiiTagEntry` among this package's
declarations, and VecCiiIssue, VecCiiOperandServer, VecCiiWriteback and
VecCiiComplete all bind it expecting to find it here — but this file does not
declare it, and VecCiiTagTable declares it locally instead, on the
`VecBusyResp`-in-`VecBusyTable` precedent, arguing that it crosses only
boundaries between siblings inside VecCiiHost. Both positions are defensible and
they cannot both be generated: one declaration, one home. Recorded from this end
as well as the other four so the discrepancy cannot be closed by each side
assuming the other did it. It is NOT resolved by declaring the type twice.

Instantiates nothing. Its dependents are VecPipeline and effectively every
`vec/**` module, so a field change here has the widest blast radius of any node
in the map after MicroOp — check the `depends_on:` fan-in before editing.
<|end_dependencies|>

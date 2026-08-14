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
  VecCiiIssue — the Issue direction of the CII host: on an `IQ_V_ALU` grant it
  allocates a 4-bit `tag`, assembles the per-tag side-table entry, emits the
  extended Issue packet, and meters `fu_types` against the Issue channel's
  credits.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/cii/VecCiiIssue.scala,
  package boom.v4.vec.generated.cii. group vec_cii. depends_on MicroOp,
  VecBundles, VectorParams, VecTrace, tt_cii_caracal_pkg. Instantiated ONCE by
  VecCiiHost as `iss`. A NEW node of the v2 amendment: the CII host had a module
  per channel direction except this one, which left the whole issue-packet
  contract inside the container next to the BlackBox binding.

  ===> CALLOUT 1 — THE PACKET IS THE CONFIGURATION CHANNEL. The CII has no CSR
       channel and a 32-bit RVV word does not encode `vtype`, so the full dynamic
       vector context rides in every packet. That buys the design out of a
       serializing configuration write, lets the VPU hold no cross-instruction
       state, and therefore lets `IQ_V_ALU` issue age-ordered rather than in
       program order. Every field comes from a DIFFERENT place, and a substitution
       is silent corruption rather than a compile error: `insn` from the uop's raw
       word, `vtype` from the DECODE-time speculative mirror snapshot on the uop
       (not a CSR read), `vl` from the VL register file at the RENAMED `pvl_src` (not
       VLMAX derived from `vtype`), `vstart`/`vxrm` from the architectural CSR
       file at issue (not snapshotted at decode), `frm` from `fcsr`.

  ===> CALLOUT 2 — AN OVER-ADVERTISED CREDIT IS A DROPPED INSTRUCTION. The grant
       is a fire-and-forget `Valid` and the CII channels have no `ready` line, so
       there is nothing to back-pressure and nothing to notice. `fu_types` may be
       advertised only when an issue credit AND a free tag are both genuinely
       available, from a REGISTERED gate, exact to the beat.

  Governing spec anchors: cii.rst `cii-host-bridge` (the credit-metered gate),
  `cii-issue` (tag allocation, side-table membership, no serializing VCONFIG
  write), `cii-operands`, `cii-flush`; execution.rst `vector-execution`,
  `cii-prn-arn`, `cii-issue-packet`; frontend.rst `vector-rvv-decode` (CSR read
  at execute); plan §5 rules 1, 5, 8 and 10.

<|begin_module|>

  <|begin_parameters|>
  No functional parameters. The CII figures are derived from the frozen SV
  contract `tt_cii_caracal_pkg.svh` through VectorParams and never redeclared — a
  Chisel-side literal disagreeing with the package is a silent protocol break,
  not a compile error: `ciiTagBits` = `CII_TAG_W` = 4 (16 tags), `numCiiTags` =
  `CII_N_TAGS` = 16, `ciiIssueCredits` = `CII_N_ISS_CREDITS` = 16 (the Issue
  receiver's depth), `numIssueLanes` = `CII_NUM_INST_ISSUE` = 1, `numSrcSlots` =
  `CII_NUM_SRC_SLOTS` = 4 (the `src_reuse` width).

  ===> "NEVER REDECLARED" NEEDS A NAMED SOURCE, or it becomes three bare literals
       that happen to be right today. The Chisel mirror of the SV package is
       `object TtCiiCaracalPkg`, declared in `VecCiiHost.scala` and visible to
       this file without an import (same package). Default the constructor
       parameters FROM IT — `numCiiTags = TtCiiCaracalPkg.CII_N_TAGS`,
       `ciiIssueCredits = ..CII_N_ISS_CREDITS`,
       `numIssueLanes = ..CII_NUM_INST_ISSUE` — not from `16`, `16`, `1`.
       `ciiTagBits` and `ciiNumSrcSlots` continue to come from `VectorParams`,
       which mirrors the same SV values; the two mirrors are cross-checked by the
       existing `require(numCiiTags == 1 << ciiTagBits)`, which is what makes a
       drift between them a build failure instead of a protocol break.

  Other widths come from the implicit `Parameters`: `vecVLSz` (9), `vlPregSz`,
  `vecPregSz`, `maxMembers` through VecBundles/VectorParams, and `xLen`,
  `maxPregSz`, `robAddrSz`, `numIrfWritePorts`, `FC_SZ` from
  `HasBoomCoreParameters`. No width here may be a literal — in particular `vl`
  and `vstart` are `vecVLSz` = 9 bits, never 6: 6 bits is VLMAX for LMUL=1 only
  and truncates a real VL of 256 at LMUL=8/SEW=8 to zero.

  Elaborated only when `usingRVV`; in a vectors-off build it is ABSENT, not tied
  off (plan §5 rule 1). There is no `enableVectorArith` sub-gate — see the
  elaboration-gate callout in VecCiiHost for why it was removed.

  Require `numIssueLanes == 1` and that `iq_v_alu`'s grant width is 1: the Issue
  channel carries one beat per cycle, so a second grant in a cycle has nowhere to
  go and, with no ready line, is lost silently. If a wide tier raises
  `vecIssueGrantWidth` to 2 for the ALU queue, that must fail elaboration here
  rather than drop instructions at run time.

  Deliberately absent — any queue, any FSM, any instruction-scoped state beyond
  the single stage below, and any `busy`/`fu_ready` output other than the
  credit-metered `fu_types` (plan §5 rule 6).
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are the hierarchy.yaml defaults and Chisel's implicit
  convention: posedge `core_clk`, ACTIVE-HIGH SYNCHRONOUS `core_reset`. Three
  pieces of state are reset-sensitive: the stage valid bit and the registered
  advertise bit reset to 0; the credit counter resets to the FULL complement
  `ciiIssueCredits` (see the credit section — 0 deadlocks the machine silently).

  `iss` — `Input(Valid(new MicroOp))`, the grant from `iq_v_alu`. Fire and
  forget: no `ready`, and this module never refuses an asserted grant.

  `fu_types` — `Output(UInt(FC_SZ.W))`, wired to `iq_v_alu.io.fu_types(0)`,
  carrying the FU-code mask VDecode puts on a CII op (the EXISTING `FC_ALU` bit;
  ScalarOpConstants' delta adds no new `FC_*` code and `FC_SZ` stays 10), or zero
  when no credit is available. It must be exactly VDecode's mask: wider grants
  ops the CII cannot accept, different means nothing ever issues.

  `iss_pkt` — `Output(Valid(new CiiIssueReq))` (VecBundles), the extended Issue
  packet; VecCiiHost flattens it onto the BlackBox's `iss_valid`/`iss_data`, which
  `tt_cii_host_wrap` repacks into `cii_caracal_issue_req_t`. No `ready`.
  `iss_credit` — `Input(Bool())`, one credit per beat the receiver pops, arriving
  late through the relay's registered credit-return pipe.

  The IO bundle takes `numCiiTags` as its OWN constructor parameter —
  `class VecCiiIssueIO(val numCiiTags: Int)(implicit p: Parameters)`, instantiated
  as `IO(new VecCiiIssueIO(numCiiTags))` — because a module-level `val` is not in
  scope inside the IO class and referencing it there is a `not found: value
  numCiiTags` compile error. This is the same idiom `VecCiiTagTableIO` already
  uses for `nTags`; match it.

  `tag_free_mask` — `Input(UInt(numCiiTags.W))`, bit `t` set when tag `t` is not
  live, exported by `tags` (VecCiiTagTable), which owns the live bit because it
  owns the entry and the `last`-beat free. `tag_alloc` —
  `Output(Valid(new VecCiiTagEntry))` carrying the allocated `tag` and the whole
  entry content: VecCiiTagTable records it (its cii.d8), this module produces it.

  `alloc_br_mask` — `Output(UInt(maxBrCount.W))`, and `alloc_flush_on_commit` —
  `Output(Bool())`: the granted uop's `br_mask` and `flush_on_commit`, taken from
  `s1_uop` and therefore valid in the SAME cycle as `tag_alloc`. They exist solely
  to feed VecCiiFlush's two allocation-time assertions (cii.e3/e4 and cii.e11) and
  no functional path reads either one.

  ===> THEY MUST COME FROM `s1_uop`, NOT FROM `io.iss.bits`, AND THEY MUST NOT BE
       TIED OFF. VecCiiFlush's dependency section already names this module as
       their driver; an earlier revision of THIS file's port list omitted them,
       and the container consequently tied both to zero — which leaves all three
       assertions well-typed, permanently true and therefore silently dead. A
       tied-off assertion input is worse than a missing assertion: it reports
       PASS. `br_mask` must be sampled in the same stage as the allocation because
       the assertion is about the uop THIS entry was built from; reading
       `io.iss.bits` in the emit cycle samples the NEXT grant. The entry itself
       stores neither field — they are assertion-only and must not be added to
       `VecCiiTagEntry`.

  `vl_read_addr` — `Output(UInt(vlPregSz.W))` and `vl_read_data` —
  `Input(UInt(vecVLSz.W))`: this node's dedicated VL register file read port on
  `vlrf`, the one VlRegFile allocates per vector issue queue. Unconditional, no
  valid/ready. Assumed COMBINATIONAL on the presented address, per VlRegFile, so
  the value is registered into the stage below; if that read turns out to be
  address-registered instead, drop that register rather than adding a cycle.

  `int_scalar_read_req` — `Output(UInt(maxPregSz.W))` / `int_scalar_read_rsp` —
  `Input(UInt(xLen.W))`, plus `fp_scalar_read_req`/`fp_scalar_read_rsp` of the
  same shape: address in the grant cycle, data the next, the discipline
  VecScalarOperandRead's INT read already uses. `int_wb_snoop` —
  `Input(Vec(numIrfWritePorts, Valid(new Bundle { addr: UInt(maxPregSz.W); data:
  UInt(xLen.W) })))`, a fan-out of the existing `vec_pipeline_io` writeback tap.

  ===> SEAM GAP, NOW CLOSED BY AMENDMENT. `vec_pipeline_io` used to give four
  INT read ports (two per VecScalarOperandRead instance) and one FP port, and
  its comment said this node needs none because it captures scalars "from the
  bypass". A bypass carries only values in flight, and a past-PNR CII op's
  scalar producer has usually retired, so the value exists ONLY in the register
  file — which is what execution.rst `cii-prn-arn` states ("the host reads the
  INT/FP RF at `prs1` at grant"). The seam is therefore FIVE INT reads (the
  fifth is this node's) and ONE FP read, which is likewise this node's: decision
  D4 deleted the store-side FP reader, so no second FP lane is needed or wanted.
  Do not "fix" this by deleting the read, which delivers an undefined `.vx`
  operand.

  `csr_vstart` — `Input(UInt(...))` from `csr.io.vector.vstart`, rocket's width
  `maxVLMax.log2` = 8 bits at VLEN=256; `csr_vxrm` — `Input(UInt(2.W))` from
  `csr.io.vector.vxrm`; `csr_frm` — `Input(UInt(3.W))` from `csr.io.fcsr_rm`.
  `csr_vstart` is 8 bits and the packet field is 9: ZERO-EXTEND. The widths
  differ legitimately — `vstart` is an element INDEX (0..VLMAX-1) while `vl` is
  a COUNT (0..VLMAX), so only `vl` needs the extra bit.

  `rob_flush` — `Input(Bool())`, `rob.io.flush.valid`, used only to decide the
  `killed` value written with a fresh entry; this module owns no other part of
  the kill contract.

  Nothing here is a `Decoupled` and there is no `busy` output. The design's only
  back-pressure flows the other way, through `fu_types`.
  <|end_ports|>

  <|begin_logic|>
  ---- Structure: one pipeline stage, nothing instruction-scoped ----

  ACCEPT cycle: `vl_read_addr` := `iss.bits.pvl_src` and the scalar read address :=
  the renamed physical scalar source, both combinational off `iss.bits`; the grant
  is captured into one stage — `s1_valid` (reset 0), `s1_uop`, `s1_tag` and
  `s1_vl`. There is NO `s1_killed`: kill state lives in VecCiiTagTable's
  `tag_killed`, never in the entry or in this stage — see the flush-race callout
  below. EMIT cycle: `iss_pkt.valid` and `tag_alloc.valid` both assert
  off `s1_valid`, in the SAME cycle, so the coprocessor never sees a packet whose
  side-table entry is not yet readable. The stage exists because the INT/FP
  register-file read answers in the cycle after its address and the entry must be
  written once and complete; emitting the packet one cycle earlier than the entry
  was rejected because it would be correct only under an unstated minimum request
  latency across the relay, whose pipe depths this side does not own.

  Nothing else is stored and the stage is overwritten every cycle, which is what
  makes a grant in every consecutive cycle legal and makes it impossible for this
  module to hold "the current coprocessor instruction".

  ---- Credit metering ----

  //@req-spec-cii.c11
  One counter, `iss_credits`, of width `log2Ceil(ciiIssueCredits + 1)` (5 bits),
  mirroring the room left in the Issue receiver's FIFO: `+1` per returned
  `iss_credit`, `−1` per beat accounted, `RegInit(ciiIssueCredits.U)`. The full
  complement at reset is required because `tt_cii_channel` keeps no counter of its
  own and the receiver returns credits only on pops, never an initial batch.
  Initialising to 0 does not fail safe: `fu_types` would never be advertised,
  `IQ_V_ALU` would never grant, and it would present as a vector hang with no
  assertion anywhere.

  //@req-spec-cii.c14
  Account at ACCEPT, not at emit: debit on `iss.valid`, not on `iss_pkt.valid`.
  Every accepted grant emits exactly one beat (even a killed one — see the flush
  race), so this is the same accounting one cycle earlier, and it is the only
  spelling that counts the beat sitting in the stage register. Debiting at emit
  leaves that beat unaccounted for one cycle, an over-advertisement by one, which
  with no ready line is a DROPPED INSTRUCTION rather than a stall.

  //@req-spec-cii.c10
  //@req-spec-cii.c12
  //@req-spec-cii.c13
  The gate is one REGISTERED bit, `advertise`, and `fu_types` is that bit
  selecting the CII FU-code mask, else zero. Registered because a combinational
  `fu_types` → grant → `iss_valid` path closes a loop through the issue unit's
  select logic. Its next value comes from the counter's NEXT value, not its
  current one, and from tag availability as well:
  `advertise := (iss_credits_next =/= 0.U) && (tag_avail_next =/= 0.U)`, with
  `tag_avail = tag_free_mask & ~s1_pending_mask`. Both terms are needed because a
  credit returns when the receiver POPS the issue beat while a tag stays live
  until its `last` writeback beat, far later, so tag availability is the tighter
  resource and the same fire-and-forget argument applies to it.

  ===> THE OFF-BY-ONE THAT DROPS AN INSTRUCTION. `RegNext(iss_credits =/= 0)`
  looks equivalent and is not: at one credit the consuming grant fires in the
  same cycle the register samples, so the stale sample keeps `fu_types`
  advertised one cycle longer, a second grant arrives at zero credits, and its
  beat is lost with nothing to report it.

  Assert synthesizably `!(iss.valid && iss_credits === 0.U)`, `!(iss.valid &&
  tag_avail === 0.U)` and `iss_credits <= ciiIssueCredits.U` (the last catches a
  spurious or doubled credit return corrupting the mirror). These assertions are
  the exactness obligation made checkable.

  ---- Tag allocation ----

  //@req-spec-cii.d6
  The `tag` is a 4-bit opaque handle, allocated at grant as the lowest set bit of
  `tag_avail` using BOOM's existing `SelectFirstN`/`PriorityEncoder` (plan §5 rule
  10, no new selector). It is not the `rob_idx` and not a register number; the
  coprocessor only echoes it on every Src-Request and Writeback beat.
  `s1_pending_mask` is the one-hot of `s1_tag` qualified by `s1_valid`.
  ===> WITHOUT THAT SHADOW THE SAME TAG IS ALLOCATED TWICE. `tags` sets the
  live bit when `tag_alloc` lands, one cycle after the tag was chosen, so
  `tag_free_mask` still shows it free during the emit cycle; a back-to-back
  grant picks it again, two instructions share one entry, and one `last` beat
  frees a tag whose other owner is still running. One bit covers the one cycle
  of exposure and duplicates no state — the stage already holds the tag.

  ---- The issue packet, field by field ----

  //@req-spec-cii.d7
  //@req-spec-cii.k1
  //@req-spec-cii.k10
  In the emit cycle drive `iss_pkt.valid` and assemble `iss_pkt.bits`: the full
  dynamic vector context of ONE instruction, from five independent sources,
  prefixed with the 4-bit `tag`. Field order and widths are
  `cii_caracal_instr_t`'s, and Chisel bundle fields are MSB-first, so the `vtype`
  sub-bundle must be declared `{vsew, vlmul, vta, vma}` in that order.

  //@req-spec-cii.k2
  `insn`, 32 b: `s1_uop.debug_inst`, the RAW RVV word, whose `vs1`/`vs2`/`vd`
  fields are ARCHITECTURAL specifiers the VPU decodes to derive its operand set
  and semantics, never to address a register file. Assert `debug_inst === inst` on
  an `is_vec` uop — RVV has no compressed encoding so they coincide today, and the
  assertion catches a future decode path that rewrote `inst` and thereby silently
  changed what the coprocessor executes.

  //@req-spec-cii.k3
  `vtype`, 8 b: `{vsew, vlmul, vta, vma}` repacked FIELD BY FIELD from
  `s1_uop.vconfig`, the decode-time snapshot of the speculative VCFG mirror riding
  the uop — NOT a read of the architectural `vtype` CSR. Two traps:
  (1) Do NOT `asUInt` rocket's `VType` and slice it. Its layout is
      {vill, reserved, vma, vta, vsew, vlmul_sign, vlmul_mag} — a different
      order, plus a `vill` bit and a reserved field the packet has no room for.
      Its low 8 bits are NOT this field.
  (2) `vlmul` is 3 b and must be `Cat(vlmul_sign, vlmul_mag)`. Rocket's
      deprecated `vlmul` accessor returns `vlmul_mag` ALONE (2 b), so using it
      drops the fractional-LMUL sign and turns every mf2/mf4/mf8 op into an
      m1/m2/m4 one, with no width mismatch to catch it.
  Assert `!s1_uop.vconfig.vill`: a `vill` uop must have trapped at decode, and the
  packet has nowhere to carry the poison bit onward.

  //@req-spec-cii.k4
  //@req-spec-cii.f27
  `vl`, 9 b: `s1_vl`, read out of the VL REGISTER FILE at the RENAMED `pvl_src` in
  the accept cycle. VL is renamed, so there is no VL value on the uop and no
  decode-time VL to snapshot: the VL PRN is a plain readiness wakeup and the value
  is resolved here, at execute. It is the architectural VL, not VLMAX derived from
  `vtype`, and `vl = 0` is an ordinary value that issues normally.

  ===> THE READ ADDRESS IS `pvl_src`, NOT `pvl`, and `MicroOp` says so in as many
       words: "Read-side consumers (VL-RF read address, source busy bit,
       issue-slot VL wakeup match) must use THIS field, never `pvl`." `pvl` is the
       PRN a VL PRODUCER WRITES; `pvl_src` is the one this uop READS. They are
       equal for every instruction that reaches this module today — only `vle*ff.v`
       is both a VL reader and a VL writer, and that is an LSU op which never
       issues to `IQ_V_ALU` — so the two spellings are indistinguishable in
       simulation right now and would diverge silently the day that stops being
       true. The already-built sibling `VecScalarOperandRead` reads `pvl_src` for
       the identical role; match it.

  //@req-spec-cii.k5
  //@req-spec-decode.d7
  `vstart`, 9 b: the zero-extended `csr_vstart`, READ FROM THE CSR FILE AT ISSUE.
  `vstart`, `vxrm` and `vxsat` are deliberately not snapshotted into the uop —
  they are not part of the speculative configuration a branch restores, and a
  stale `vxsat` snapshot would lose accrued saturation — so they are read at
  execute. This module reads `vstart` and `vxrm`; `vxsat` is never read here, it is
  the sticky bit VecCiiWriteback accrues FROM the writeback beats. Reading a CSR
  at execute is sound in an out-of-order machine only because BOOM decodes CSR
  writers as `is_unique`: a younger `csrw vstart` cannot take effect while this
  instruction's ROB entry is open, and an older one has already committed.

  //@req-spec-cii.k6
  //@req-spec-cii.k7
  ===> `vstart` IS PASSED THROUGH AND MUST NOT BE FORCED TO 0. Hardware never
       PRODUCES a non-zero `vstart` — a faulting vector load or store traps with
       `vstart = 0` and restarts whole (plan §5 rule 8) — so it is tempting to
       conclude the field is always zero and tie it off. Software can write it:
       `csrw vstart` before a vector op is architectural RVV, and the VPU honours
       it by leaving elements below it untouched. Tying it to 0 makes a resumed
       instruction recompute elements it was required to leave alone, visible only
       when `vd` overlaps a source — data corruption with no exception, no trace,
       and no unmodified test that exercises it.

  //@req-spec-cii.k8
  //@req-spec-cii.k9
  `vxrm`, 2 b: `csr_vxrm`, straight from `csr.io.vector` — the fixed-point
  rounding mode is applied inside the VPU, so it must travel per instruction.
  `frm`, 3 b: `csr_frm`, from `fcsr.frm`. The two are distinct modes in distinct
  CSRs and are adjacent in the packet, so a swap is a plausible mis-wiring that no
  width check would catch.

  //@req-spec-cii.k11
  //@req-spec-cii.d9
  //@req-spec-cii.d10
  `src_reuse` (the `instr_src_valid` hint, `numSrcSlots` bits) is driven to ZERO
  on every source: the VPU ignores it and re-pulls every operand, and honouring it
  to elide redundant pulls is a later optimisation. Everything above travels PER
  INSTRUCTION in this packet — there is NO serializing VCONFIG CSR write and no
  configuration path of any other kind. That is not tidiness: out-of-band
  configuration would impose program-order issue on the CII, which is exactly the
  constraint `IQ_V_ALU` relies on not existing in order to be age-ordered rather
  than a head-only FIFO.

  ---- The side-table entry ----

  //@req-spec-cii.f40
  In the same cycle `tag_alloc.bits` snapshots the RENAMED PHYSICAL groups off
  `s1_uop` — `pvdest_grp` plus its member mask, `pvs1_grp`, `pvs2_grp`,
  `pvs3_grp`, `pvm`, `stale_pvdest_grp` — with `rob_idx`, `is_shared`, `pdst` and
  the captured scalar value. The host owns the whole ARN→PRN mapping and the
  coprocessor names operands only by slot and member offset, so this snapshot is
  the only thing that can later resolve a pull or a writeback. `pvs3_grp` and
  `stale_pvdest_grp` are copied as TWO INDEPENDENT FIELDS from two independent
  MicroOp fields, never merged or cross-assigned: they coincide for RMW arithmetic
  and diverge for a masked non-RMW op, `vslideup` and `vcompress`.

  The mapping is stated field by field because the side-table spells its fields
  with a `_grp` suffix and `MicroOp` does not, so a transposition would be silent
  corruption rather than a compile error — `rob_idx := s1_uop.rob_idx`,
  `is_shared := s1_uop.is_shared`, `pdst := s1_uop.pdst`,
  `pvs1_grp := s1_uop.pvs1`, `pvs2_grp := s1_uop.pvs2`,
  `pvs3_grp := s1_uop.pvs3`, `pvm := s1_uop.pvm`,
  `stale_pvdest_grp := s1_uop.stale_pvdest`, `scalar_operands :=` the captured
  scalar, and `tag :=` the tag chosen in the accept cycle.

  ===> THE ONE FIELD THAT IS A MUX, NOT A COPY, IS THE DESTINATION GROUP:

           cop_writes_pvtmp = s1_uop.is_shared && s1_uop.uses_stq
           pvdest_grp      := Mux(cop_writes_pvtmp, s1_uop.pvtmp, s1_uop.pvdest)

       `pvdest_grp` NAMES THE GROUP THE COPROCESSOR HALF WRITES, which for a
       SEGMENTED STORE is `pvtmp` and for everything else — including a segmented
       LOAD's coprocessor half, which transposes `pvtmp` INTO `pvdest` — is
       `pvdest`. The predicate is the "tmp-only" case VecRenameSpace already names:
       a segmented store's `dst_rtype` is not `RT_VEC`, so rename granted it ONE
       group and routed it into `uop.pvtmp`, and `uop.pvdest` holds nothing. Both
       spellings of the predicate are the same set — `is_shared && uses_stq` uses
       only baseline `MicroOp` fields and needs no `RT_*` encoding here, and
       dispatch presents the SAME uop to `iq_v_alu` and `iq_v_store` (VecIssueUnit
       part 8), so `uses_stq` is set on the granted coprocessor half.

  ===> WHY THE MUX IS HERE AND NOWHERE ELSE. Without it, cii.i4/i8/i11 — the
  coprocessor half writes the `pvtmp` group — are UNSATISFIABLE, because the
  only destination field a beat can resolve against would hold a group rename
  never allocated. It cannot be fixed downstream either: VecCiiTagTable performs a
  whole-bundle copy and derives nothing, VecCiiWriteback writes
  `wb_lookup.resp.prn` and holds no uop, VecCiiComplete announces
  `wb_lookup.resp.pvdest_grp` and holds no uop, and ALL THREE explicitly forbid
  re-deriving the choice from `is_shared`/`is_store` — two places deciding which
  group a beat lands in is exactly how a segmented store silently writes the
  load-side group. One mux, one cycle, one place, and this is the place.
  The entry field keeps the name `pvdest_grp`: it describes the common case, and
  renaming it would touch four sibling specs to say nothing new. What the name
  is NOT is a licence to load it unconditionally from `uop.pvdest`.

  Members at or beyond `v_emul` are captured verbatim rather than zeroed: PRN 0 is
  a real allocatable vector PRN, so zeroing would make an out-of-range offset
  read a legitimate register belonging to someone else — harder to spot in a
  waveform than a stale member.

  The member mask is a PREFIX (thermometer) mask of the DESTINATION group's size,
  `(1 << v_emul) - 1`, not a one-hot of `v_emul`.
  Two observed ways to get this wrong: `UIntToOH` instead of the prefix form
  enables one member and drops the rest; and taking the count from a SOURCE
  EMUL makes the mask too wide for a single-register-destination op (`vmv.s.x`,
  the reductions, the mask-producing compares), which hangs the coprocessor
  waiting to place members that are never written. `v_emul` is the destination
  group's member count as the mapper allocated it, already 1 for those ops.

  //@req-spec-cii.f24
  //@req-spec-cii.f41
  A `.vx`/`.vf` scalar source is delivered BY VALUE, not by address: the
  coprocessor can address no register file, so a late read is not available to it.
  In the accept cycle the renamed physical scalar register drives
  `int_scalar_read_req`, or `fp_scalar_read_req` when the source's register type is
  `RT_FLT` — the uop's register-type field selects the file, nothing here decodes
  the instruction — and in the emit cycle the returned value is written into the
  entry's scalar field, from which VecCiiOperandServer later serves the `SCALAR`
  slot with no VRF read. That read carries the same RAW window as
  VecScalarOperandRead's base-GPR read and for the same reason: BOOM's integer
  register file is a `Mem` read of a registered address with no read-during-write
  bypass, so a write presented in the EMIT cycle is not reflected in that cycle's
  data. Compare every valid `int_wb_snoop` port's `addr` against the
  stage-registered scalar PRN and on a hit substitute that port's `data`; assert
  `PopCount(hits) <= 1` and use a `Mux1H`, not a priority mux.
  The window is exactly the emit cycle — a cycle earlier is redundant (the
  write took effect at the intervening edge), a cycle later is too late.

  ---- What this module does not look at, and the flush race ----

  //@req-spec-cii.d11
  All vector arithmetic, reduction and permutation instructions, and the
  coprocessor half of the shared (segmented load/store) instructions, reach the
  VPU through this one path, and this module distinguishes none of them: it
  performs no RVV decode whatever, since the raw word goes in the packet and the
  VPU's decoder derives the operand set and the semantics. That is what keeps the
  host adapter free of a second RVV decoder that could disagree with VDecode's.
  STAGING NOTE, not a behaviour: a shared instruction's coprocessor half
  addresses the `pvtmp` rendezvous group on the side it transposes (source side
  for a segmented load, destination side for a segmented store), so the entry
  content substitutes `pvtmp` on that side. No requirement in the corpus names
  that substitution and plan step F7 owns the transpose half; recorded here so
  it is not silently dropped when F7 lands.

  A ROB-head flush can arrive in the accept or the emit cycle — after the tag is
  chosen, before the entry exists — and a grant CAN fire in the flush cycle
  itself, because `IQ_V_ALU` gates on `flush_pipeline = RegNext(rob.io.flush.valid)`
  rather than on the flush cycle. So this module WILL sometimes land a wrong-path
  entry, and that is handled, but NOT here.

  ===> THIS MODULE ADDS NO KILL LOGIC, AND `rob_flush` IS ASSERTION/TRACE-ONLY.
       An earlier revision of this paragraph said to carry an `s1_killed` bit and
       "write the entry with `killed` already set". THAT MECHANISM IS SUPERSEDED
       and `VecCiiTagEntry` deliberately has no `killed` field — kill state lives
       in VecCiiTagTable's own `tag_killed` register, never in the entry. The race
       is closed by the KILL WINDOW BEING TWO CYCLES WIDE: VecCiiFlush drives
       `kill_all = rob_flush || rob_flush_kill`; an allocation can only occur in
       the FIRST of those two cycles (the second is exactly `flush_pipeline`,
       which gates the grant), the allocation write clears that tag's kill bit in
       cycle one, and cycle two re-sets it on the now-live entry. The drain then
       proceeds normally. VecCiiTagTable's "kill-window assertion is an eventual
       property" callout is the authority on this and states the obligation as
       `alloc.valid && kill_all` implies `RegNext(tag_killed(alloc.tag))`.
       Do not re-add a `killed` field, do not gate the packet on `rob_flush`, and
       do not assert that `kill_all` and `alloc.valid` are mutually exclusive —
       they legitimately coincide.

  The beat is emitted regardless — suppressing
  it would strand the tag, because the coprocessor would never receive the
  instruction, never emit a `last` beat, and the tag would never be freed. Kill is
  expressed by the side-table bit and by suppressing effects at writeback, never
  by withholding a packet: VecCiiFlush's Src-Data asymmetry, applied to the Issue
  channel. Branch kill is not handled here and must not be — `is_br`/`is_jalr` set
  `starts_unsafe`, so the PNR cannot sweep past an unresolved branch and no
  granted CII op is ever younger than one; the assertion recording that belongs to
  VecCiiFlush.

  ---- Tracing and remaining assertions ----

  Two guarded `VecTrace` lines, gated on the `vecTrace` plusarg and `!reset` and
  adding no logic any functional path reads: `VecTrace.traceTag` per emitted packet
  with `rob_idx`, `tag`, `vl`, the packed `vtype`, `vstart` and the credit count —
  the line that makes a Whisper divergence attributable to a specific packet field
  — and one whenever `iss_credits` reaches zero, the evidence for whether credit
  depth or VPU latency limits throughput. Assert also that a granted uop has
  `is_vec` set and `iq_type` naming `IQ_V_ALU`. Every assertion here is
  synthesizable; the module contains no simulation-only construct.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput: ONE issue beat per cycle, sustained — a grant in every consecutive
cycle must produce a packet in every consecutive cycle. That forbids an FSM here
and makes the credit gate the only thing that may ever stop a grant.

Latency: exactly ONE cycle from grant to packet, fixed and data-independent, set
by the INT/FP register-file read discipline rather than by anything computed here;
the packet assembly is pure wiring plus one `Mux1H`.

The real ceiling is elsewhere: with 16 issue credits and a credit round trip of N
cycles through the relay's registered request and credit-return pipes plus the
receiver's pop, the sustained rate is 16/N per cycle. If the zero-credit trace
line dominates, the fix is credit depth or VPU latency in the SV, not logic here.
Vector arithmetic is in-order in the VPU by deliberate choice, so a long-latency
op blocks younger independent ones however fast this path is.

Critical path to watch: `tag_free_mask` → priority select → `s1_tag`, in series
with the accept-cycle grant. If it fails timing, pre-select the next tag a cycle
early (the mask is known before the grant) — never by deleting the
`s1_pending_mask` term, which is a correctness term. Area is one `MicroOp` stage
register, a 5-bit counter, a 4-bit tag register, a 16-bit shadow mask and one
advertise flop.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the grant payload. Reads `debug_inst`, `inst`, `vconfig`, `pvl_src`
(the READ-side VL PRN — never `pvl`, see the `vl` field note above), `pvdest`,
`pvtmp`, `stale_pvdest`, `pvs1`, `pvs2`, `pvs3`, `pvm`, `v_emul`, `is_vec`,
`is_shared`, `uses_stq`, `iq_type`, `prs1`, the `*_rtype` fields, `pdst`,
`rob_idx`; writes none. `pvtmp` and `uses_stq` are read for ONE purpose, the
destination-group mux of the side-table entry.

VecBundles — `CiiIssueReq` (the packet) and `VecCiiTagEntry` (the entry payload).
BUNDLE-LOCATION NOTE. hierarchy.yaml's VecBundles entry lists `VecCiiTagEntry`
among its declarations, but VecBundles as written does not declare it. It
crosses this node, VecCiiTagTable, VecCiiOperandServer, VecCiiWriteback,
VecCiiComplete and VecCiiFlush, so it belongs there; Phase R should add it
unchanged.
WIDTH DISAGREEMENT, also for Phase R: VecBundles gives `CiiIssueReq` a 3-bit
`src_reuse_hint` while the frozen package types `instr_src_valid` as
`CII_NUM_SRC_SLOTS` = 4 bits. The SV is authoritative and this module drives
zero either way, but the flat BlackBox port must match exactly or the whole
issue payload shifts.

VectorParams — `ciiTagBits`, `maxMembers`, `vecVLSz`, `vecPregSz`, `vlPregSz`,
and the mirrors of `CII_N_TAGS` / `CII_N_ISS_CREDITS` / `CII_NUM_INST_ISSUE` /
`CII_NUM_SRC_SLOTS`. VecTrace — the two guarded lines. tt_cii_caracal_pkg — the
frozen contract every width and field order derives from
(`cii_caracal_instr_t`, `cii_caracal_vtype_t`, `cii_caracal_issue_req_t`,
`CII_TAG_W`, `CII_VL_W`, `CII_N_ISS_CREDITS`).

Binds to `freechips.rocketchip.rocket.VType` through `MicroOp.vconfig` and to
`HasBoomCoreParameters`. Reuses BOOM's `SelectFirstN`/`PriorityEncoder`; adds no
new selector, wakeup or squash mechanism.

Instantiates nothing. Instantiated by VecCiiHost as `iss`. Producers: `iq_v_alu`
(VecIssueUnit), `vlrf` (VlRegFile), BoomCore's INT/FP register files and
writeback tap, rocket's `CSRFile` through `csr_vector`/`csr_frm`, and the Issue
channel's credit return from `tt_cii_host_wrap`. Consumers: `tags`
(VecCiiTagTable) and the coprocessor; VecCiiComplete and VecCiiFlush act on the
tag it allocated and VecCiiOperandServer on the entry it wrote.
<|end_dependencies|>

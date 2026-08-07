# Caracal v2 — Phase R seam review agenda (as-found)

**Status:** Phase N (author all 58 NL_HDL specs) is COMPLETE. `inspect-hierarchy` passes
with 0 errors. **This document is the outstanding work.**

## What this is, and why it exists separately from the specs

`inspect-hierarchy` checks *format* and *requirement-tag coverage*. It cannot check whether
two specs AGREE with each other. Every finding below is a case where two independently
authored specs — each individually well-formed, tag-complete and self-consistent — describe
the same seam differently, or where a signal has a consumer and no producer, or where a
requirement is unimplementable as written.

That class of defect is invisible to per-file review by construction, which is the entire
premise of the plan's §4 "author everything, then review the whole set". This file is the
concrete output of that premise: **60 numbered amendments (A1-A60) plus 6 open rulings.**

## How it was produced

Authored by ~50 parallel agents, one per node, each writing its own `.nlhdl` spec and
reporting (a) its seam assumptions about neighbours and (b) conflicts it found but did NOT
resolve. Findings were accumulated as authoring progressed, so a later node's report often
names a defect in an EARLIER, already-written file. Those are the A-items.

**Read the A-items as a work list, not as a description of the current tree.** Items marked
RESOLVED / CONSISTENT record a decision already reflected in the specs; the rest are edits
still to make.

## How to read the voice

Sections addressed as "you" were written FOR the authoring agents (e.g. "ROUND 3 — For
VecRenameSpace"). They are preserved verbatim rather than rewritten, because they record
what each author was actually told — which matters when auditing why a spec says what it
says. Treat them as a transcript, not as instructions to a human reader.

## ⚠ Two items block Phase A regardless of everything else

1. **A vector config fails elaboration TODAY.** The three `IQ_V_*` `issueParams` entries make
   `core.scala:837-849` hit `require(false)`. `BoomCore`'s spec proposes a one-line fix; it
   needs sign-off (see the BoomCore section).
2. **Gate (f) bit-identity is false by construction** — see "TOP OPEN RULING" below.
   `ScalarOpConstants`/`MicroOp`/`Rob` widen shared encodings unconditionally. Decide before
   generating any RTL, because the first two nodes in build order are the affected ones.

## Highest bring-up risk (silent hang / silent corruption, no assertion fires)

- **A52** — `q_free` on `VecQueueReservation` had NO DRIVER: the six element queues fill
  once and never free. Reached by the first real vector test.
- **A1** — `VecFreeList`'s single-bit `alloc_fire` reintroduces the M1 double-allocate via a
  *non-vector* hazard (prefix-scan `dis_stalls`).
- **A12** — `VecCiiIssue`'s credit counter must reset to 16, not 0; init-0 is a machine-wide
  vector hang with no assertion.
- **A13** — rocket's deprecated `vlmul` accessor returns 2 bits, silently turning every
  fractional-LMUL op into m1/m2/m4 with no width error.
- **A23** — three `IQ_V_*` `iq_type` bits arrive DON'T-CARE on every scalar uop without
  explicit defaults. A passing gate (f) does not prove the defaults exist.
- **A49** — copying baseline's SNI `rob_idx === rob_pnr_idx` term into `IQ_V_ALU`'s past-PNR
  gate admits an unresolved entry to the coprocessor, defeating the gate's whole purpose.

## Open rulings needing a human decision (not mine to make)

| Ruling | Where |
|---|---|
| Gate (f): accept a bounded exception, or parameterize the encodings | "TOP OPEN RULING" |
| `allocWidth = coreWidth*8` deadlocks a shared OP.v at `coreWidth == 1` | hierarchy.yaml `VecFreeList` comment |
| RVV index offsets: signed (map/plan) or unsigned (RVV 1.0)? | "OPEN RULINGS" |
| `stale_pvdest` readiness — a CORPUS GAP; 4 nodes converged on it; matches the prior M2 `pvold_busy` hang | A-list + VecIssueSlot |
| No back-pressure path for a multi-cycle element walk | "OPEN RULING (structural)" |
| A30 — FP fast-wakeup residual window; no node owns the fix | A30 |
| A31 — `vfmv.f.s` has no landing site | A31 |

## Also needing `/spec-to-reqs` follow-up (spec defects, not spec-to-code defects)

`agen.a8`/`c11` (superseded OVI modules — already ledgered); `memord.a19` (not synthesizable
as written); `memord.b16` (names a store-set predictor BOOM v4 does not have);
`spec-core.f11` (false for a vector store, which is granted twice); `spec-agen.d5`;
`spec-decode.d8`/`c3`; `overview.rst:158` vs `frontend.rst`; and `spec-lsu.g1-g4/g9/g10`
mis-allocated to `VecElemAgen` (an `architect`-pass error of mine, not an authoring one).

---

# Index of amendments

| # | Item | Owner file(s) to edit |
|---|---|---|
| A1 | `VecFreeList`: `alloc_fire` must become `Vec(coreWidth, Bool)`, not one bit. | VecFreeList |
| A2 | `VecBusyTable` must add a second output `member_busy_resps` | VecBusyTable |
| A3 | `VecCiiTagTable` must store the coprocessor half's destination group as `pvtmp` for a segmented store. | VecCiiTagTable |
| A4 | `VecCiiWriteback` vs `VecCiiFlush` — one suppression port, two shapes. | VecCiiWriteback, VecCiiFlush |
| A5 | `VecCiiWriteback`/`VecCiiComplete`: neither side registers `io.beat`. | VecCiiWriteback, VecCiiComplete |
| A6 | `req_shared` on `VecFreeList` means "needs TWO groups", NOT literally `is_shared`. | VecFreeList |
| A7 | No per-source USE predicate exists for `pvs1`/`pvs2`/`pvs3`. | — |
| A8 | `VLSDecode`'s prose still says `vleff` is not a VL producer. | VLSDecode |
| A9 | `VecRangeAgen` carries a stale complaint | VecRangeAgen |
| A10 | `VDecode` should export `is_whole_reg_move` | VDecode |
| A11 | `VecBundles.CiiIssueReq.src_reuse_hint` is 3 bits; the frozen SV types `instr_src_valid` as `CII_NUM_SRC_SLOTS` = 4 bits. | — |
| A12 | `VecCiiIssue`'s credit counter must RESET TO THE FULL COMPLEMENT (16), not 0. | VecCiiIssue |
| A13 | Rocket `VType` → CII packet `vtype` is a REPACK, not a slice. | — |
| A14 | `VlRegFile` read timing is still contradicted across three files. | — |
| A15 | `csr.io.vector.vstart` is 8 bits | — |
| A16 | RESOLVED — the R2/W0 strict-priority mux lives in `VecGroupCopy`. | VecGroupCopy |
| A17 | `W2`'s physical write-port index is `lsuWidth`, not a literal 2 | — |
| A18 | `VecRegFile` provides no `ready` on read requests; `VecIdxGen`'s store instance expects one. | VecRegFile, VecIdxGen |
| A19 | `spec-lsu.g1/g2/g3/g4/g9/g10` (the `vleff` fault/trim policy) are MIS-ALLOCATED to `VecElemAgen`. | VecElemAgen |
| A20 | `VecIdxGen`'s descriptor field `idx_eew` must be sourced from `uop.v_idx_eew` | VecIdxGen |
| A21 | `spec-agen.c10` (power-of-2 mask skip) and `spec-agen.c16` (one element at a time) cannot both hold on the INDEXED path | — |
| A22 | `VecMaskStream`'s own file still says it is instantiated inside `VecElemAgen`. | VecMaskStream, VecElemAgen |
| A23 | CONFIRMED REAL BUG — the three new `IQ_V_*` positions must be explicitly defaulted, and only `DecodeUnit` can do it. | DecodeUnit |
| A24 | `io.csr_decode.vector_illegal` must be ANDed into `v_legal`. | — |
| A25 | Vector LOAD-FP/STORE-FP widths must be matched POSITIVELY as `{000,101,110,111}`. | — |
| A26 | `dec_vec_illegal` must cover every RVV encoding the host's COARSE opcode gate admits but the design does not implement. | — |
| A27 | No vector decoder may read `uop_in.exception` / `uop_in.exc_cause` | — |
| A28 | `VDecode` and `VsetDecode` both still say their decode-stage trace lines are tagged with `rob_idx`. | VDecode, VsetDecode |
| A29 | `vec_pipeline_io` gained `fp_wakeups` | vec_pipeline_io |
| A30 | RESIDUAL FP-WAKEUP WINDOW that no response-cycle forward can close. | — |
| A31 | `vfmv.f.s` has no owner. | — |
| A32 | Giga tier will fail elaboration on the added FP read port. | — |
| A33 | `midcore.rst` `spec-wakeups`' FP re-busy clause is vacuous on this baseline. | — |
| A34 | `VecRobFlags` is named in `vec_pipeline_io` but declared by no node | VecRobFlags, vec_pipeline_io |
| A35 | `numVecClrPorts` is declared nowhere | — |
| A36 | `ALUUnit` must drive the resolved `vconfig` on EVERY `vset*` writeback, not only `vsetvl`. | ALUUnit |
| A37 | Wording conflict resolved in midcore's favour | — |
| A38 | One existing assert must be weakened | — |
| A39 | `VecCiiOperandServer`: replace `tag_entry: Vec(4, Input(VecCiiTagEntry))` with the narrow `src_lookup` response | VecCiiOperandServer |
| A40 | `VecCiiTagTable`: drop the `alloc.tag` output and the registered `tag_avail`; export `tag_free_mask = ~tag_valid` instead. | VecCiiTagTable |
| A41 | `VecCiiWriteback`'s kill self-check must be the IMPLICATION `killed -> wb_suppress`, never an equality. | VecCiiWriteback |
| A42 | `VecBundles.CiiIssueReq.src_reuse_hint` must be 4 bits | — |
| A43 | The `addPath` calls must sit in the BlackBox's own CONSTRUCTOR BODY | — |
| A44 | The SV-constant cross-check is a BUILD step, not a Chisel `require`. | — |
| A45 | `src/main/sv/v4/tt-cii` is NOT registered in `.gitmodules` | — |
| A46 | `VecCiiHost` instantiates the receive FIFOs DEGENERATE (`ciiRxDepth = 0`) | VecCiiHost |
| A47 | The Src-Data credit counter's stall is an ASSERTION, not a gate. | — |
| A48 | `spec-core.f11` "each issue slot must be granted once" is FALSE AS LITERALLY WRITTEN for a vector store | — |
| A49 | DO NOT COPY baseline's SNI block into the vector issue unit — one of its terms is actively wrong here. | — |
| A50 | CONSISTENT ACROSS THE SEAM (no action): | — |
| A51 | `fu_types` type mismatch at the CII seam | — |
| A52 | ⇒ `q_free` ON `VecQueueReservation` HAD NO DRIVER AT ALL. | VecQueueReservation |
| A53 | US store address/data regions are NOT in identity correspondence. | — |
| A54 | `VlRegFile.W_lsu` had TWO candidate producers | — |
| A55 | `VecLsuCoreIO` is now declared TWICE. | VecLsuCoreIO |
| A56 | `VecRangeAgen.io.st_data` | — |
| A57 | `VecScalarOperandRead.out` has no `ready`, but `VecRangeAgen.io.req` is `Decoupled`. | — |
| A58 | `VecSquashUnit`'s `nKillClients` comment is stale after the mask-streamer hoist. | VecSquashUnit |
| A59 | ROLLBACK CONVENTION SETTLED — BOOM's EXCLUSIVE tail. | — |
| A60 | `VecGroupCopy` is deliberately EXCLUDED from `vec_lsu_empty` | VecGroupCopy |

---
# Seam notes from completed sibling agents

Read this if any node listed below is one of your `depends_on:` or
`instantiates:` targets, or is on the other side of a seam you touch. These are
**contracts a sibling has already committed to in a written spec**. Honour them,
or if you must disagree, say so loudly in your report rather than silently
specifying something incompatible.

---

## VecRegFileBank  (written — 357 lines)

Consumed by: **VecRegFile** (instantiates it x4).

- **Bank IO shape the parent must match**:
  - `read_addr : Vec(numReadPorts, Input(UInt(vecPregSz.W)))`
  - `read_data : Vec(numReadPorts, Output(UInt(bankWidth.W)))`
  - `write_ports: Vec(numWritePorts, Flipped(Valid(addr: UInt(vecPregSz.W),
     data: UInt(bankWidth.W), mask: UInt(bankBytes.W))))`
  - **No handshake on reads** — no `Decoupled`, no back-pressure anywhere. Unlike
    the scalar regfile's `arb_read_reqs`.
- **The address is the full unshifted PRN.** There is no bank-select bit and no
  `addr >> log2Ceil(numBanks)`. Banking is **by WIDTH, not by register index**,
  so BOOM's scalar `BankedRF` pattern in
  `src/main/scala/v4/exu/register-read/regfile.scala` must NOT be reused —
  it banks by index. Flagged as the likeliest mis-generation.
- **Bit-slice convention**: bank `b` holds register bits
  `[bankWidth*(b+1)-1 : bankWidth*b]`, little-endian. Byte `j` of bank `b` is
  register byte `bankBytes*b + j` (= element index at SEW=8). `VecRegFile` must
  slice write data/mask on this convention, and the LCB and store path index
  against it.
- **`numWritePorts = 1 + lsuWidth`** (2 at `lsuWidth=1` → W0+W2; 3 at
  `lsuWidth=2` → W0+W1+W2). Ports arrive **compacted in canonical W0,W1,W2
  order**. The bank treats indices symmetrically and never names a functional
  unit; `VecRegFile` owns the mapping and must not renumber per bank.
- A write with an all-zero mask **in a given bank still arrives with `valid`
  set** — valid/addr are broadcast unchanged to all four banks.
- **Read latency 0 / "may not be pipelined" is BANK-INTERNAL ONLY — corrected.**
  The `VecRegFile` PORT contract is a **registered ONE-cycle read** (request N,
  `read_data` N+1), with the output flop instantiated in `VecRegFile`, one per read
  port, and the bank's combinational access sitting inside that envelope. Do not
  propagate "combinational port" to any consumer. This is what
  `VecLoadCoalescingBuffer` (`stale_resp`), `VecDgen` (`vrf_r3`), `VecMaskStream` and
  `VecIdxGen` all already assume. Write is 1 cycle; read-during-write forwarding is
  unconditional and merges on the response.
- The array is `Reg`, not `RegInit` — deliberately takes no reset value.
- **Tracing**: the bank does NOT call `VecTrace.trace(...)`, because that helper
  mandates a `MicroOp`/`rob_idx` and a storage bank has neither.
  **`VecRegFile` owns the per-access VRF trace lines** (it has the uop). The bank
  keeps only the write-collision assertion and an optional forward-hit line.
- **`debug_vrf_read`** (from the `vec_pipeline_io` interface) is built by
  `VecRegFile` out of the ordinary read ports. The bank declares no debug port —
  that would be a 13th port.
- Open item for `VecRegFile`: **`spec-vrf.f9`** ("VRF initialized only by the
  usingRVV switch") is allocated to `VecRegFile`, not to the bank, but the
  storage lives in the bank. The bank honoured it untagged by declaring the array
  inside the `usingRVV`-gated instantiation, i.e. `usingRVV` controls the array's
  *existence*. `VecRegFile` should tag `f9` consistently with that reading.
- Note: "12 ports" in the spec/risk table is the `lsuWidth = 2` case; at
  `lsuWidth = 1` it is 11.

---

## CORRECTIONS ALREADY APPLIED to the pkg/ files (re-read them if you read them early)

Three defects were found by sibling agents and are now FIXED in `src/main/nlhdl/pkg/`.
If you read those files before this note appeared, re-read the changed sections.

1. **`VectorParams`: `maxVecVL` / `vecVLSz` corrected to 256 elements / 9 bits.**
   It previously said `maxVecVL = vLen / 8` (32, → 6 bits), which is VLMAX for
   LMUL=1 only and silently truncates a real VL of 256 at LMUL=8/SEW=8 to zero.
   Correct is `vLen * maxMembers / 8` = `vLen` = 256, so **`vecVLSz` = 9 bits**,
   which matches midcore.rst/frontend.rst's "64 entries of ~9 bits".
   **Use 9 bits for every VL-valued field**, including the CII issue packet's `vl`.

2. **`MicroOp` gained the access-class fields** (new section 4b): `v_mop` plus
   `v_is_unit_stride / v_is_strided / v_is_indexed / v_is_segment /
   v_is_whole_reg / v_is_mask / v_is_ff`, and `v_idx_eew`.
   Reason: `VLSDecode` decodes the class once, but `VecLsu`, `VecElemAgen`,
   `VecRangeAgen`, `VecIdxGen` and `VecBeatExpander` do not depend on it, so the
   descriptor has no path to them except on the uop. **Do NOT re-decode
   `uop.inst` downstream** — read these fields.
   Also: **`v_eew` is the DATA element width.** For an INDEXED access the data
   width is `vtype.vsew` and the index width is the separate `v_idx_eew`.

3. **`VecTrace` gained `traceDecode(module, event, ftq_idx, pc_lob, extra)`.**
   `rob_idx` does not exist at decode (the ROB entry is allocated at dispatch),
   so decode-stage callers (VDecode, VLSDecode, VsetDecode, VConfigUnit) use this
   variant, keyed on `ftq_idx`/`pc_lob`, emitting `rob=?`. Never invent a zero
   `rob_idx` — it aliases with real ROB entry 0 in every grep.

4. **`hierarchy.yaml`'s VLSDecode comment corrected**: `vleff` **IS** a VL
   producer (loadstore.rst, spec-lsu.g6/g7, plan §Phase G all agree). The old
   "not a VL producer until the fault-trim path exists" was a staging note that
   read as a design statement. The VL-RF write port for `vleff` is declared from
   day one and simply never fires until step G4.

---

## VlRegFile  (written — 358 lines)

- Write ports are **statically partitioned, one per producer class, never
  arbitrated**: `W_ren` (rename-cycle, for `vsetivli`, **replicated per rename
  lane**), `W_alu` (integer ALU writeback for `vsetvli`/`vsetvl`), `W_lsu`
  (`vleff`'s trimmed count). `numAluWritePorts` defaults to 1 because
  `vec_pipeline_io` presents a single `vset_resp` — if a tier ever lets two
  integer ALUs produce a VL in one cycle, **replicate the port, never arbitrate**.
- Holds **no busy bit**. `pvl_busy` is owned by the `vl_rename` instance of
  `VecRenameSpace`, which leaves it clear for `vsetivli` (born ready).
- `W_ren[w].valid` is assumed qualified by the **same `dis_fire(w)` off
  `ren2_uops`** that qualifies `vl_rename`'s allocation on lane `w`. Wiring it
  off `dec_uops` is the M1 free-list double-free again.
- **Reads are unconditional and combinational**, no valid/ready, no enable, no
  read-during-write bypass — assumes every VL-wakeup→execute-read distance is
  >= 1 cycle. If anyone adds a fast/speculative VL wakeup, this needs a bypass mux.
- Read-port owners: load AGEN, store AGEN, `VecCiiHost` (one per vector issue
  queue), plus a **single commit read** the ROB drives with the youngest
  committing VL producer's `pvl`, feeding `csr.io.vector.set_vconfig.bits.vl`.
  `VecQueueReservation` and `VecGroupCopy` were given **no** port (reservation is
  worst-case EMUL/EEW at dispatch; the VL=0 path is downstream of a load-side
  read). If `VecGroupCopy` needs VL, take a load-side port — do not add one.
- The whole array **resets to 0** (576 flops), unlike the VRF which does not.

## VecBusyTable  (written — 415 lines)

- Instantiated **twice** — the VL busy table is the `maxGroupSize = 1` instance of
  this same module, not a separate module. If any sibling authored a standalone VL
  busy table, that is a duplicate to resolve.
- Declares **`VecBusyResp` in this file, not in VecBundles** (mirroring baseline
  `class BusyResp`). Field names match MicroOp exactly: `pvs1_busy, pvs2_busy,
  pvs3_busy, pvm_busy, pvtmp_busy` on the vector instance, `pvl_busy` on the VL
  instance. If `VecBundles` or `VecRenameSpace` also declares it → duplicate.
- Assumes **`VecRenameSpace` owns the in-bundle prefix bypass** (the
  `BypassAllocations` equivalent), ORing into the uop busy field *after* reading
  this table, and joins the two instances' responses. The vector instance has
  **no `pvl` read port at all**.
- `VecGroupDone` fields assumed: `Vec(maxMembers, UInt(vecPregSz.W))` member PRNs
  + a **`members` count** + `rob_idx`. The clear side uses `members`, not `v_emul`.
- The **VL instance's clear port is `Valid(UInt(pregSz.W))`** — a bare PRN, not a
  degenerate `VecGroupDone`. The VL writeback path must drive that shape.
- **No flush port**: assumes producers kill wrong-path completions at source
  (`VecCiiFlush`, `VecSquashUnit`) so no wrong-path group-done ever reaches a
  reallocated PRN. Flagged as a bring-up check.
- `busy_resps` is **combinational in the rename cycle**; no `RegNext` on wakeups,
  so issue slots and this table see the same group-done in the same cycle as the
  ROB's `vec_clr_bsy`. **Set beats clear** in the same cycle.

## VLSDecode  (written — 365 lines)

- Per-lane interface: in `valid`, `inst[31:0]`, uop context; out `desc:
  VLSAccessDesc`, `illegal`. `coreWidth` lanes inside the single `ls` instance,
  **no ready/handshake**. `VecDecode` is assumed to merge `desc` into
  `dec_uops_out` and OR `illegal` into `dec_vec_illegal`.
- Class is decoded from **`mop`/`umop` alone, never from direction**; `is_store`
  appears in no class expression. Direction reaches the agens as the `isStore`
  module parameter.
- `is_segment` is `nf =/= 0 && !is_whole_reg && !is_mask && !is_ff` — a bare
  `nf =/= 0` would make `vl8re64` demand a `pvtmp` group and the coprocessor
  transpose path, because `nf` means register-count-minus-one for `vl<n>re<eew>`.
- `VecRangeAgen` is assumed to own byte-length derivation for whole-register
  (`nregs * VLEN/8`) and mask (`ceil(vl/8)`) accesses; this node only classifies.

---

## SECOND ROUND OF CORRECTIONS (applied — re-read if you read earlier)

5. **`MicroOp` gained `v_is_masked`** (the decoded `vm` sense). Do NOT re-derive it
   from `inst(25)`: the issue slot must include `pvm`'s busy bit in readiness ONLY
   when masked, or every unmasked op falsely depends on the last writer of `v0`.
6. **`VecBundles.VecRangeEntry` gained the active BYTE mask** (`vLen/8` bits), plus
   it must also carry `stride` and `is_unit_stride` (spec-agen.b7). The US path has
   no other mask carrier — `VecRangeAgen` has no mask reader of its own.
7. **`VecMaskStream` was HOISTED out of `VecElemAgen` up to `VecLsu`**, one per
   direction (`ld_msk`, `st_msk`). Two agents reported this conflict with opposite
   proposed fixes. Adjudicated: the unit-stride path needs a mask reader
   (spec-agen.e12) and R1/R4 may have exactly ONE reader each (spec-vrf.g18), so
   the streamer sits at VecLsu level and feeds BOTH that direction's element agen
   and its range agen. `VecElemAgen` now instantiates only `VecIdxGen`.
8. **The six `VecElemQueue` instances now pass `isStore` and `hasXlatePass`.**
   `hasXlatePass` is true only for `st_SSI_ADDR_Q` / `st_US_ADDR_Q`.
9. **`vec_pipeline_io`: `int_rf_read_req/rsp` widened 3 -> 4**, and a new
   **`int_wb_snoop`** ({addr,data} per INT write port) added — `int_wakeups` carries
   readiness without data, so the stale-base forward was unimplementable without it.
10. **`hierarchy.yaml` comment fixes**: R4 serves mask AND index (two readers, one
    port); `VecScalarOperandRead` supplies the mask/index PRNs but does NOT perform
    those VRF reads; `VecFreeList`'s capacity comment corrected to the 96-PRN
    sizing (8 groups / 4 segmented).

## OPEN RULINGS raised by agents — do not resolve unilaterally, report if you touch them

- **`allocWidth = coreWidth*8` deadlocks a shared OP.v at `coreWidth == 1`** (needs
  16 PRNs all-or-nothing, pool is 8). An elaboration `require` now fails the build.
  Decide before C2: either Small forbids segmented vector LS, or allocWidth becomes
  `max(coreWidth,2)*maxGroupSize`.
- **RVV index offsets: SIGNED or UNSIGNED?** The map and plan say signed; RVV 1.0
  describes indexed offsets as unsigned. `VecIdxGen` isolated the extension into one
  named function so a Whisper divergence localises there.
- **`VlRegFile` says "no read-during-write bypass"**, but `VecScalarOperandRead`
  argues `pvl` is woken by the ALU VL writeback and needs a same-cycle forward.
  One of the two must change.
- **`loadstore.rst` load-streaming clause may be dead** — the reservation is
  worst-case, so a load's active count can never exceed it.
- **`overview.rst:158` contradicts `frontend.rst`** on the execute-time vtype mirror
  write. frontend.rst + the plan win (there is NO execute-time mirror write).
- **`VecScalarOperands` bundle** should move into `VecBundles` (it crosses four nodes).

---
# ROUND 3 — contracts from completed leaves. If a node below is your child or seam-mate, THIS IS BINDING.

## For VecRenameSpace (children VecMapTable / VecFreeList / VecBusyTable all written)
- **VecBusyTable declares `VecBusyResp` in its own file** (not VecBundles). Fields match
  MicroOp: `pvs1_busy pvs2_busy pvs3_busy pvm_busy pvtmp_busy` (vector inst),
  `pvl_busy` (VL inst). Do not redeclare it.
- **You own the in-bundle prefix bypass** (`BypassAllocations` equivalent), ORing into
  the uop busy field AFTER reading the busy table, and you JOIN the two instances'
  responses. The vector busy instance has **no `pvl` port**.
- **VecFreeList expects ONE whole-bundle `alloc_ok` out / `alloc_fire` in** — no per-lane
  grant or stall mask. You must AND `alloc_ok` into `dis_ready` and **never fire a subset
  of lanes**. It needs `reqs`/`req_members`(=`v_emul`)/`req_shared`(=`is_shared`) per lane.
- VecFreeList needs a **second dealloc bank `dealloc_tmp`** for `pvtmp` at commit (commit
  cannot stall). Unused member slots are padded with **member 0, never 0.U**; consumers
  must still mask by `v_emul`.
- **VecMapTable expects bundles `VecMapReq` / `VecMapResp` / `VecRemapReq`** and assumes
  they live in VecBundles (which does not currently declare them — resolve). `emul`
  arrives as a member COUNT 1..8 (fractional EMUL presented as 1). It **echoes `v_emul`
  back out through `map_resps`**, so it is the single authority on member validity.
- VecMapTable has an **`exportComStale` parameter / `com_stale_resps` port** you must set
  true for `vl_rename` and route to VecFreeList (that is how the outgoing committed VL
  pointer is freed; there is no `stale_pvl`).
- `ren_br_tags` is `Vec(plWidth+1, Valid(brTag))`, entry 0 tied invalid, entry `w+1` for
  lane `w` — exactly baseline `ren2_br_tags`. `rollback` is a Bool.
- **⇒ YOU MUST EXPORT PER-MEMBER SOURCE READINESS TO THE ISSUE STAGE.** `VecGroupReady`
  requires an `in_member_rdy: Vec(members, Bool)` per source group, carried alongside the
  uop (not inside MicroOp, whose reject list forbids a per-member busy vector). Reason: a
  group's members can come from DIFFERENT producers, so it can be not-ready in aggregate
  while members 0..2 are already done; broadcasting the aggregate makes the matcher wait
  for group-dones that already fired — a **permanent hang**. rename.g11 already performs
  the per-member read; just export it.

## For VecRegFile (child VecRegFileBank written)
- Bank IO, bit-slice convention, `numWritePorts = 1 + lsuWidth`, W0/W1/W2 canonical order,
  0-cycle unpipelined read: see the VecRegFileBank section above. **Banking is by WIDTH,
  not by register index** — do not reuse BOOM's `BankedRF`.
- **You own**: the per-access VRF trace lines (the bank has no uop), `debug_vrf_read` built
  from the ordinary read ports (no 13th port), and the `spec-vrf.f9` tag.
- **You own the strict-priority mux on the Load Unit's R2/W0** between
  `VecLoadCoalescingBuffer` (always wins) and `VecGroupCopy` (always loses). The LCB drives
  unconditionally and is never told it lost.
- `VecCiiOperandServer` owns R5–R8 outright and must never stall on a port;
  `VecCiiWriteback` owns W2 outright.

## For VecIssueSlot (children VecGroupReady / VecStoreDgenPath written)
- **VecGroupReady needs `in_member_rdy` (per member) AND you must route `out_member_rdy`
  (next-state) through the collapse move as a SIDE CHANNEL** — routing it through
  `out_uop.pvs*_busy` collapses it to the aggregate and reintroduces the hang above.
- Its `prns`/`members`/`used` must be **end-of-cycle values**: from `in_uop` while `load`
  is high, from `slot_uop` otherwise.
- **`used` is an INPUT you drive**: masked-ness (`v_is_masked`) for `rdy_vm`, and
  `!is_shared` on `rdy_vs3` for the LSU half (that is how issue.g32 is discharged
  structurally, at zero comparator cost).
- **VecStoreDgenPath's operand mux sits UPSTREAM of the matcher**: wire its
  `dgen_operand`/`_members`/`_busy` into your **`rdy_vs3`** instance and return that
  instance's group-ready as its `dgen_operand_ready`. In a store slot `rdy_vs3` therefore
  tracks **`pvtmp`** whenever `is_shared`. It registers nothing (collapse would strand a
  registered grant bit) — the grant bits live in the migrating uop's
  `fu_code(FC_AGEN)`/`fu_code(FC_DGEN)` pair with `iw_issued_partial_*` as post-grant markers.
- **`numVecWbPorts` is undeclared anywhere.** VecGroupReady parameterised it with default
  **3** (LCB, CII completion, VecGroupCopy). If you or VectorParams declare it, bind them.
- **OPEN, and two nodes now depend on it**: `IQ_V_LOAD` must gate issue on `stale_pvdest`
  readiness (the LCB pre-loads from it on R2) and `IQ_V_ALU` must too (the CII pulls
  `STALE_VD`). No corpus requirement covers stale-dest readiness. The prior M2
  implementation needed exactly this (`pvold_busy`) to avoid a hang. Report it; do not
  invent a port silently.

## For VecElemAgen (child VecIdxGen written; VecMaskStream HOISTED to VecLsu)
- **You instantiate ONLY `VecIdxGen` now.** The mask streamer moved to VecLsu (one per
  direction); you receive its staged/ahead/skip interface as an INPUT from your parent.
- VecIdxGen: `io.start` Decoupled{`pvs2`, `idx_eew`, `vl`, `rob_idx`}; index interface
  `{valid, offset: SInt(xLen), elem_idx, last, next_valid, stall}` out, `taken` in.
  `next_valid` is forced high at the last element (else deadlock); `taken` pulses per
  element **including masked-off ones**. **Gate BOTH "start" and "release final segment" on
  its exported `stall`.** The base+offset add happens in YOU, at `xLen`, before narrowing.
- **The element cursor lives in the LDQ/STQ placeholder entry, not in you** —
  `VecBeatExpander` PEEKS the queue head and read-modify-writes `elem_next` in the same
  cycle. You write at `base + element_index` (absolute), not at a FIFO write pointer.
- VecQueueReservation drives four fixed release lanes; yours are lane 0 (ld_elem) and
  lane 1 (st_elem), `used_count` = `vl` (SSI) or `vl * nf` (segmented). **A denied release
  is never retried.**
- Open wording conflict to honour: `spec-lsu.f8` ("fire TLB and LCAM in element order") is
  allocated to you but the fire happens on the DRAIN side. VecBeatExpander's in-order
  cursor advance is what makes it true. Specify your side (emit in element order) and say so.

---
# ROUND 4 — FOR THE CONTAINERS (VecLsu, VecCiiHost, VecIssueUnit, VecPipeline, BoomCore)

You are wiring nodes that are ALREADY WRITTEN. Your job is largely to RESOLVE the
mismatches they reported. Each item below is a real reported gap — settle it
explicitly in your file and say which way you settled it in your report.

## Declarations that no written file owns yet (settle where they live)
- **`VecCiiTagEntry`** — hierarchy.yaml's VecBundles comment lists it, but
  `pkg/VecBundles.nlhdl.scala` does not declare it. VecCiiOperandServer binds to
  "the single declaration wherever it lands" and forbids a second copy.
- **`VecBusyResp`** — declared inside `VecBusyTable.nlhdl.scala` (baseline
  `BusyResp` precedent). Do not duplicate.
- **`VLSAccessDesc`** (VLSDecode), **`LcbAllocReq`/`LcbBeat`/`LcbTrim`** (LCB),
  **`LsuResourceClaim`** (VecDcacheArbiter), **`VecScalarOperands`**
  (VecScalarOperandRead), **`VecMapReq`/`VecMapResp`/`VecRemapReq`** (VecMapTable
  assumes VecBundles owns them) — all declared locally by their authors, all cross
  a node boundary. Decide: promote to VecBundles, or keep local and forbid copies.
- **`numVecWbPorts`** is named by `issue.g33`/`rename.g25` but declared NOWHERE.
  VecGroupReady parameterised it, default **3** (LCB, CII completion, VecGroupCopy).
  VecGroupCopy confirms it is producer #3. Bind it.
- **`VecRangeEntry` needs four fields beyond VecBundles' enumeration**: `stride`
  and `is_unit_stride` (spec-agen.b7, VecRangeAgen), the active **byte mask**
  (added), and `us_data_base` + `members` (VecStoreForward — `st_US_ADDR_Q` holds 1
  entry/store while `st_US_DATA_Q` holds `v_emul`, so the two reservation bases are
  NOT in identity correspondence). VecBeatExpander additionally wants `nf` and, to
  avoid post-commit re-translation, up to 2 retained PPNs.
- **`VecElemAccess`** needs, per VecBeatExpander: store `data`,
  `uses_tlb`/`uses_dcache`/`uses_lcam`, `lcam_range_len`.
- **`VecTrace` needs a THIRD entry point keyed on `tag` + side-table `rob_idx`** for
  the CII nodes (they hold no MicroOp), analogous to `traceDecode`. Also a
  `rob_idx`-only overload for VecIdxGen/VecRegFileBank-style nodes.

## OPEN RULING — VRF read latency: WHO OWNS THE FLOP (do not settle unilaterally)
`spec-vrf.f8` says "single cycle reads" with same-PRN write forwarding;
`spec-cii.f23` says "the **registered**, one-cycle VRF read". Both imply
**observable latency 1**, but the written files disagree on where the register sits:
- `VecRegFileBank`: array read is **combinational**, 0-cycle, may not be pipelined.
- `VecCiiOperandServer`: comb address in cycle t, **payload registered by the
  consumer**, beat driven at t+1 → total latency 1.
- `VecIdxGen`, `VecMaskStream`, `VecGroupCopy`, `VecDgen`, LCB (`R2`): assume the
  **VRF returns registered data at t+1**.
⇒ If BOTH VecRegFile and the consumer register, latency becomes **2** and every
element/beat pipeline above is off by a cycle. `VecRegFile` was told (in flight) to
specify a registered 1-cycle port. **VecPipeline must state the single canonical
answer and every consumer must match it.** This is the top R2 agenda item.

## VecLsu — specific obligations from your children
- You instantiate **`VecMaskStream` x2 (`ld_msk`, `st_msk`) at YOUR level** (hoisted
  out of VecElemAgen) and must feed BOTH that direction's element agen AND its range
  agen. `VecElemAgen` instantiates only `VecIdxGen`.
- **You own the strict-priority mux ownership question**: `VecGroupCopy` declares the
  R2/W0 mux INSIDE itself (LCB requests pass through `gcopy` combinationally, no
  grant/ready/nack), while `VecRegFile` was told it owns that mux. **Pick one.**
- You must qualify `VecGroupCopy`'s launch: load direction only, has a vector dest,
  and **exclude `v_is_whole_reg`** (a `vl1re*` with `vl=0` still executes normally;
  launching a copy would race a real load for the same PRNs).
- `VecSquashUnit` expects to be the resolver of the `kill`/`kill_uop` vectors
  (`nKillClients`, default 8) that VecIdxGen/VecMaskStream/VecBeatExpander each
  expect as a resolved Bool "from the parent". If you compute them locally instead,
  `spec-lsu.i6` loses its owner. Route uop in / kill out through the squash unit.
- **Rollback index convention (highest-risk seam in the LSU subtree)**:
  VecSquashUnit drives BOOM's **exclusive** tail; VecQueueReservation's spec reads as
  **inclusive** survivor. An inclusive reading keeps one killed instruction's entries
  alive. Settle it and assert it.
- `VecBeatExpander` **PEEKS** the US queue head and read-modify-writes `elem_next` in
  the same cycle; the cursor lives in the LDQ/STQ placeholder, not in any agen. Both
  sides address the queues by **absolute index** (`reservation base + element
  cursor`), never a FIFO pointer — the queues are NOT FIFOs.
- `st_US_DATA_Q` holds **one entry per source group member** (`vLen` each), not one
  per instruction. VecDgen and VecBeatExpander both assume this.
- A US store runs **two passes over the same retained range entry** (pre-commit
  TLB+LCAM, post-commit TLB+D$); you reset the cursor between them and must not treat
  the execute pass as freeing.
- Double credit check on the LCB: `VecBeatExpander` gates on a per-PRN
  `lcb_alloc_rdy` Bool while `VecDcacheArbiter` gates on `free_count === 0`. Consistent
  only if you derive the former from the latter — say so.
- `VecOrderHold` exports its hold as `hold_ldq : UInt(numLdqEntries.W)` (arbiter
  assumption); `VecStoreForward` exports `known_overlap` for the hold to consume.
  Do not let the hold build a second overlap test.

## VecCiiHost — specific obligations
- **Straight-through Src-Data lane mapping, NOT compacted**: a sparse request set
  (lanes 0 and 2) returns on data lanes 0 and 2. `tt_cii_host_wrap` and the VPU must
  not expect compaction onto lane 0.
- **CORRECTED — THE CREDIT GRAIN IS THE BEAT, NOT THE LANE.** An earlier revision of this
  note said `req_credit` is a per-lane Bool; that is wrong against the SV.
  `tt_cii_interface.sv` gives each channel **one** `valid` and **one** `credit` for a beat
  of N lanes — there is no per-lane valid anywhere, so lane-level compaction is not even
  representable, and per-lane activity is encoded in the payload as
  `op_id = CII_SRC_NONE`. Consequence: `VecCiiHost` drives all four
  `opnd.io.src_req(i).valid` from the single beat valid and reduces `opnd.io.req_credit`
  (a `Vec(4,Bool)` as written) to the one channel bit, asserting the lanes agree.
  `src_req` is `Flipped(Valid)`, **no ready**.
- `VecCiiIssue` must store the captured scalar in the format the VPU consumes (IEEE,
  Hardfloat recode already undone for `.vf`); the operand server applies no conversion.
- The kill path is split three ways and all three must hold: **TagTable** keeps
  `killed` + enough state to route the drain and does not recycle the tag;
  **OperandServer** answers a killed request with a defined don't-care beat + credit
  and no VRF read; **Writeback** pops the beat and returns `wb_credit` but suppresses
  the register write; **Complete** suppresses group-done / ROB clear / CSR effects and
  frees the tag on the dropped last beat.
- The SV keeps the identifier `CII_SRC_VS3_VD` for slot 3 (rename declined on
  evidence: live refs in the verified tb and in a blackbox). **The contract is the
  VALUE 3, not the name.** Any sibling asserting `CII_SRC_VS3` exists in SV is wrong.
- `cii.rst` says `src_reuse_hint[3]`; the SV type is **4** bits. SV value wins.

## VecIssueUnit — specific obligations
- `VecIssueSlot` requires a **per-member readiness side channel** through the collapse
  move (`in_member_rdy` / `out_member_rdy`), NOT routed through
  `out_uop.pvs*_busy` — the aggregate collapses partial readiness and **hangs**.
- `VecStoreDgenPath` **registers nothing** (a registered grant bit would be stranded
  by the collapse shift); grant state rides the migrating uop's
  `fu_code(FC_AGEN)`/`fu_code(FC_DGEN)` with `iw_issued_partial_*` as markers. Grant
  must be combinational in the request cycle and `iss_fu_code_*` visible on `iss_uop`
  before the grant resolves.
- **OPEN (three nodes now depend on it)**: `IQ_V_LOAD` must gate on `stale_pvdest`
  readiness (LCB pre-loads from it on R2) and `IQ_V_ALU` must too (CII pulls
  `STALE_VD`). No corpus requirement covers stale-dest readiness; prior M2 work needed
  `pvold_busy` to avoid a hang. Report, do not invent silently.

---
# ROUND 5 — two NEW bugs found by containers, plus one interface fix

11. **`vec_pipeline_io` gained `rob_head_idx`** (amendment). BOOM's `IsOlder` is
    3-arg `(a, b, head)`; the past-PNR gate is not computable from `rob_idx` and
    `rob_pnr_idx` alone because the ROB is circular. Without the head, the gate
    INVERTS across a wrap boundary and lets a younger-than-PNR op reach the
    coprocessor. `VecIssueSlot` computes `eligible` internally (g21 mandates
    per-entry) and exports BOTH `request` and `eligible`; **`VecIssueUnit`'s
    priority encoder must select on `eligible`, not `request`.**

12. **NEW BUG CLASS — the segmented-store self-deadlock (mirror of the M1 DGEN bug).**
    An `IQ_V_ALU` slot must itself point a matcher at `pvtmp` to wake the
    coprocessor half of a segmented **LOAD** (issue.c11 / rob.d12 / lsu.l5). The
    select **MUST be direction-qualified**: `Mux(is_shared && uses_ldq, pvtmp, pvs3)`.
    A bare `Mux(is_shared, ...)` makes a segmented **STORE**'s coprocessor half wait
    on the very group it is about to write — immediate self-deadlock, and it is
    exercised only by segmented stores, so casual testing will not find it.

13. **NEW — the CII flush-cycle allocation race.** `IQ_V_ALU` gates on
    `flush_pipeline = RegNext(rob.io.flush.valid)`, so a grant CAN fire in the
    `rob.io.flush.valid` cycle and `VecCiiIssue` will allocate a wrong-path tag whose
    `killed` bit has just cleared. Consequences: (a) `VecCiiTagTable`'s assertion that
    `kill_all` never coincides with `alloc.valid` **will fire on a real benign case**
    and must be relaxed to "a tag allocated during a kill window carries `killed` by
    the following cycle"; (b) if that assertion is merely deleted, the hole becomes a
    wrong-path VRF write plus a `clr_rob` for a dead ROB entry. Settled fix:
    `kill_all = rob_flush || rob_flush_kill` — both terms of the SAME event, so
    `spec-cii.e5`'s "on `rob.io.flush.valid` only" still holds, and it is free because
    the kill bit is idempotent. A cheaper alternative is gating `VecCiiIssue`'s grant
    acceptance on `!rob_flush`; `VecCiiHost` should pick one and say which.

14. **`killed` state lives in `VecCiiTagTable`, not `VecCiiFlush`.** `VecCiiFlush`'s
    only functional output is `kill_all: Bool`, **combinational from `rob_flush`** (the
    writeback node needs to suppress a beat one cycle before `killed` is readable).
    `VecCiiOperandServer` deliberately has NO flush port and reads `killed` from its own
    lookup. Do not add per-channel drain/suppress outputs.

15. **`VecSlotMemberRdy`** (the per-member readiness side channel, VecRenameSpace →
    VecIssueUnit → VecIssueSlot) is declared in `VecIssueSlot.nlhdl.scala` but belongs
    in `VecBundles` — same open class as `VecScalarOperands` and `VecCiiTagEntry`.

16. **`memord.b16` names a structure BOOM v4 DOES NOT HAVE.** There is no trained
    store-set predictor (no SSIT/LFST; grep for `store_set`/`MemDep` is empty). What
    exists is match-driven blocking at `lsu.scala:1413-1431` (`ldst_addr_matches` hit
    that failed to forward → `block_load_wakeup`, plus the 15-cycle
    `store_blocked_counter`). `VecOrderHold` discharges b16 by CONSUMING that outcome on
    a port, so a future trained predictor drives the same port unchanged. A generator
    told to "use BOOM's memory-dependence predictor" will look for a table that is not
    there — the `.rst` wording should be corrected.

---
# REQUIRED AMENDMENTS TO ALREADY-WRITTEN FILES
# These are DEFECTS in written specs, found by a later sibling. They must be applied
# in Phase R (step R2) before any RTL is generated from the affected file. Listed here
# rather than patched blind, because each needs its owner's file read in full first.

**A1. `VecFreeList`: `alloc_fire` must become `Vec(coreWidth, Bool)`, not one bit.**
CORRECTNESS, not style. BOOM's `dis_stalls` is a prefix scan (`core.scala:773`), so a
**partial-prefix fire** is reachable from any NON-vector hazard — e.g. `ldq_full` on
lane 2 still lets lanes 0-1 dispatch. With a single `alloc_fire` bit the free list
consumes the non-firing lane's window PRNs; that lane retries and allocates a SECOND
group, and the first is owned and freed by nobody. That is the M1 leak/double-allocate
class, reached **without any vector-side mistake at all**. The whole-bundle rule still
holds on the GRANT (`alloc_ok`), which cannot be partial.
Also: `reqs` must be driven **fire-independent** (from `ren2_alloc_reqs`, not
`ren2_alloc_fire`), or `alloc_ok → dis_ready → dis_fire → reqs` is a combinational
loop. Consumption is qualified by fire. Matches baseline, where `can_allocate` comes
from the pre-selection register.

**A2. `VecBusyTable` must add a second output `member_busy_resps`** (per lane, per
operand, `Vec(maxGroupSize, Bool)`), gated by an `exportMemberRdy` parameter. Its file
currently exports only the AND-reduced `busy_resps` and says "never a per-member
vector" — but `VecGroupReady`/`VecIssueSlot` require `in_member_rdy` per member or they
**hang permanently** (a group whose members come from different producers is not-ready
in aggregate while members 0..2 are already done). This is a fan-out of wires
`rename.g11` already computes, taken PRE-reduction: zero new comparators, and
`busy_resps` keeps its stated shape.

**A3. `VecCiiTagTable` must store the coprocessor half's destination group as `pvtmp`
for a segmented store.** It currently declares `pvdest_grp := uop.pvdest` and lists
`pvtmp` as deliberately not stored, which makes `spec-cii.i4`/`i8`/`i11` (the
coprocessor half writes the `pvtmp` group) **unsatisfiable**. Both `VecCiiWriteback` and
`VecCiiComplete` require the single stored destination field to be `pvtmp` in that case
and explicitly forbid re-deriving it from `is_shared` downstream. Add the mux at alloc.

**A4. `VecCiiWriteback` vs `VecCiiFlush` — one suppression port, two shapes.**
Writeback takes a pre-computed `io.wb_suppress` from Flush; Flush's reject list forbids
any per-channel suppress output and exports only a bare `kill_all`. `VecCiiComplete`
independently chose `kill_all || wb_lookup.killed`. Settle on `kill_all` (+ the entry
bit) and delete `wb_suppress`, or add it to Flush — not both.

**A5. `VecCiiWriteback`/`VecCiiComplete`: neither side registers `io.beat`.**
Writeback's ruling; Complete's port comment still calls it "registered" while its perf
section demands same-cycle. A register on only one side skews group-done AHEAD of the
final `W2` write. Both agreed on "neither"; make the wording match.

**A6. `req_shared` on `VecFreeList` means "needs TWO groups", NOT literally
`is_shared`.** A segmented **store** needs `pvtmp` and has **no** `pvdest` (its
`dst_rtype` is not `RT_VEC`). Requesting two groups and using one **leaks a group
forever**, because commit frees only `stale_pvdest` and `pvtmp`. In the tmp-only case
the single granted group is routed into `uop.pvtmp`.

**A7. No per-source USE predicate exists for `pvs1`/`pvs2`/`pvs3`.** There is no
`lvs*_rtype`, so a `.vx` op's unread `pvs1` can report busy at rename. Currently
harmless ONLY because `VecIssueSlot` drives `VecGroupReady`'s `used`, and the seam notes
specify `used` only for `vm` and `vs3`. Fix: either `VDecode` exports `v_uses_vs*` bits,
or `VecIssueSlot` derives them for all three. Pick one and write it down.

**A8. `VLSDecode`'s prose still says `vleff` is not a VL producer.** Correction 4
reversed that (`loadstore.rst`, `spec-lsu.g6`/`g7`, plan §Phase G all agree it IS), and
`VecDecode` — the actual writer of the uop — settled it as `is_vl_producer := desc.is_ff`.
Update `VLSDecode` part 6 so the two files do not read as contradicting.

**A9. `VecRangeAgen` carries a stale complaint** that `maxVecVL = vLen/8`; correction 1
fixed that to `vLen` / 9 bits. Drop the complaint.

**A10. `VDecode` should export `is_whole_reg_move`** (or a `uses_vtype`). `VecDecode`
currently re-derives the two-comparator `vmv<n>r.v` exemption locally, so the term is
decoded twice.

**A11. `VecBundles.CiiIssueReq.src_reuse_hint` is 3 bits; the frozen SV types
`instr_src_valid` as `CII_NUM_SRC_SLOTS` = 4 bits.** The host drives zero either way, but
the flat BlackBox port must match or **the whole issue payload shifts**. SV value wins
(same precedent as `spec-vrf.h6`). `cii.rst`'s `src_reuse_hint[3]` row is also wrong.

**A12. `VecCiiIssue`'s credit counter must RESET TO THE FULL COMPLEMENT (16), not 0.**
`tt_cii_channel` keeps no counter of its own and the receiver returns credits only on
pops, so a counter initialised to 0 never receives a first credit — a **silent
machine-wide vector hang** with no assertion. Also: debit at **accept**, not at emit
(otherwise the beat sitting in the stage register is unaccounted for one cycle = an
over-advertisement by one), and compute the registered advertise bit from the counter's
**NEXT** value — `RegNext(credits =/= 0)` is the off-by-one that drops an instruction.

**A13. Rocket `VType` → CII packet `vtype` is a REPACK, not a slice.** Rocket's field
order is `{vill, reserved, vma, vta, vsew, vlmul_sign, vlmul_mag}` and its deprecated
`vlmul` accessor returns **2 bits** (`vlmul_mag` alone). Using the accessor silently drops
the fractional-LMUL sign, turning every mf2/mf4/mf8 op into m1/m2/m4 **with no width
error**. Applies to any node building the 8-bit `{vsew, vlmul, vta, vma}` field.

**A14. `VlRegFile` read timing is still contradicted across three files.**
`VecCiiIssue` took SEAM_NOTES' "combinational on the presented address" and registers the
data itself; `VecScalarOperandRead` assumes address-registered, data next cycle. One is
wrong. This is the VL-RF instance of the same flop-ownership question as the VRF (open
ruling above) — settle both together in R2, and note they may legitimately differ, since
the VL RF is 64x9b and the VRF is 96x256b.

**A15. `csr.io.vector.vstart` is 8 bits** (`maxVLMax.log2`) while the CII packet field is
9 (`CII_VL_W`) — **zero-extend**. `vl` genuinely needs 9 bits (a count 0..256); `vstart`
is an index 0..255. Do not size them from one constant.

**A16. RESOLVED — the R2/W0 strict-priority mux lives in `VecGroupCopy`.**
`spec-lsu.m13`/`m14` are allocated to `VecGroupCopy`; `VecRegFile` holds only `m6`/`m7`.
So `VecRegFile` must have its `gcopy_r2`/`gcopy_w0` ports and grant logic **deleted** —
its own file documents the exact deletion. The LCB's `R2` request, `R2` data return and
`W0` write pass through `gcopy` combinationally, no grant/ready/nack; the LCB drives
unconditionally and is never told it lost.

**A17. `W2`'s physical write-port index is `lsuWidth`, not a literal 2** (the write array
is compacted in canonical W0,W1,W2 order per the bank contract). `W1` is **absent, not
tied off**, at `lsuWidth = 1`. Wiring `VecCiiWriteback` to a hard-coded 2 drives nothing
at `lsuWidth = 1`.

**A18. `VecRegFile` provides no `ready` on read requests; `VecIdxGen`'s store instance
expects one.** Resolved: the back-pressure is the **2:1 R4 mux inside `VecLsu`** (R4 serves
both the store mask and the store index and must reach `VecRegFile` as exactly one
request). `VecMaskStream` exports `owns_port` as the hold-off. Mask wins, index waits.

---
# OPEN RULING (structural) — NO BACK-PRESSURE PATH FOR A MULTI-CYCLE ELEMENT WALK

The chain `iss → VecScalarOperandRead → io.op` has **no `ready` anywhere**
(VecScalarOperandRead: "the agens must latch unconditionally, justified by the
dispatch-time queue reservation"). But an SSI element walk takes up to `vl` cycles, and
`IQ_V_LOAD`/`IQ_V_STORE` grant the oldest-ready entry with nothing preventing a second
grant the next cycle. **A second descriptor can arrive at an agen still walking the
first, with nowhere to put it.**

The reservation does not cover this. It guarantees *queue capacity*, not that the agen can
accept a new descriptor mid-walk — two different resources that the spec conflates. This
is the one place v2's vector-LSU invariant is under-specified: it says where in-flight
state may live, but not what throttles the producer.

**Recommended resolution (given to VecLsu): a pending table indexed by LDQ/STQ entry**,
like `VecQueueReservation`'s. Per-queue-entry state is exactly what the invariant PERMITS
(it forbids state scoped to "the current instruction" and forbids a `busy` reaching an
issue unit); sizing by `numLdqEntries`/`numStqEntries` makes the bound structural.
**REJECTED alternative:** qualifying the `FC_AGEN` grant — that is a `busy` reaching an
issue unit under another name, a failed review per ground rule 6, and precisely what gate
H4 greps for.
The same hazard exists one level up: the mask streamer is **one-op-per-direction** and now
serves both that direction's element agen and its range agen, so a direction's OP.v
hand-offs serialize at VecLsu level. One structure should cover both.

**A19. `spec-lsu.g1/g2/g3/g4/g9/g10` (the `vleff` fault/trim policy) are MIS-ALLOCATED
to `VecElemAgen`.** `vle<eew>ff.v` is architecturally **unit-stride**, so its OP.v goes to
`VecRangeAgen` — whose reqs are only b6/b7/e13/c3/c4/d5. No node owns them correctly today.
Either move these six to `VecRangeAgen` and give it a fault/trim interface, or keep them
where the *mechanism* lives and document why. This is an `architect` allocation error, not
an authoring one.

**A20. `VecIdxGen`'s descriptor field `idx_eew` must be sourced from `uop.v_idx_eew`**, not
`uop.v_eew`. VecIdxGen's file says "the uop's `v_eew`", which predates the `v_eew` /
`v_idx_eew` split (correction 2). For an indexed access `v_eew` is the DATA width and
`v_idx_eew` is the index width; using `v_eew` walks the index vector at the wrong stride.
Fix on both sides.

**A21. `spec-agen.c10` (power-of-2 mask skip) and `spec-agen.c16` (one element at a time)
cannot both hold on the INDEXED path**, because `VecIdxGen.taken` pulses once per element
including masked-off ones. `VecElemAgen` resolved it by making the skip
**class-conditional** (strided/segmented only, never indexed). Record it so the `.rst`
selection table matches.

**A22. `VecMaskStream`'s own file still says it is instantiated inside `VecElemAgen`.**
The hoist to `VecLsu` (round 3, item 7) must be applied to its prose too.

**A23. CONFIRMED REAL BUG — the three new `IQ_V_*` positions must be explicitly
defaulted, and only `DecodeUnit` can do it.** `ScalarOpConstants` raises `IQ_SZ` 4→7
unconditionally; baseline `DecodeUnit` does `uop := io.enq.uop` then writes only iq_type
positions 0..3 individually; and `io.enq.uop` originates from a bundle the frontend sets
with `f2_fetch_bundle := DontCare` (frontend.scala:480). Net effect without a fix: **every
SCALAR uop carries three don't-care vector-queue routing bits into dispatch** —
mis-routing, not merely an X in a waveform. Same hazard for `is_vec`, `is_shared`,
`is_vl_producer`. Fixed by six default assignments in the `DecodeUnit` delta (the only
node that sees `io.enq.uop`). **A passing gate (f) does NOT prove the defaults are
present**: a don't-care bit can elaborate bit-identically and still mis-route.

**A24. `io.csr_decode.vector_illegal` must be ANDed into `v_legal`.** It is rocket's
`mstatus.VS == 0` gate (CSR.scala:246) — an existing port BOOM has never read. Without it
a vector instruction executed with VS=Off is admitted and never traps, silently violating
`spec-decode.g5`.

**A25. Vector LOAD-FP/STORE-FP widths must be matched POSITIVELY as `{000,101,110,111}`.**
The natural "width =/= FLW/FLD" form admits width 001 (FLH) and 100 (FLQ), neither of
which is in `F_table` — so it would stop two currently-illegal encodings from trapping AND
route an FLH into `VLSDecode`.

**A26. `dec_vec_illegal` must cover every RVV encoding the host's COARSE opcode gate
admits but the design does not implement.** `DecodeUnit` computes only opcode-space
legality locally and takes fine legality (vill, EMUL bound, keep-VL reserved) back on
`dec_vec_illegal`. If the vector side does not trap an admitted-but-unimplemented
encoding, it silently misbehaves instead of raising illegal-instruction.

**A27. No vector decoder may read `uop_in.exception` / `uop_in.exc_cause`** — that closes a
combinational loop (`uop.exception` ← `illegal` ← VecDecode ← a bundle containing
`exception`). None does today; keep it that way.

**A28. `VDecode` and `VsetDecode` both still say their decode-stage trace lines are tagged
with `rob_idx`.** Contradicts correction 3 — no `rob_idx` exists at decode; they must use
`traceDecode(..., ftq_idx, pc_lob, ...)`.

**A29. `vec_pipeline_io` gained `fp_wakeups`** (amendment). `spec-vrf.e4` requires the `.vf`
scalar feeder to be woken on the FP network and `VecIssueSlot` keeps FP `.vf` comparators,
but only `int_wakeups` was declared — so there was **no FP readiness path at all** and a
`.vf` op could never become ready. FpPipeline adds no new network; this is a tap of the
existing `io.wakeups`, fanned out by BoomCore.

**A30. RESIDUAL FP-WAKEUP WINDOW that no response-cycle forward can close.**
`FPExeUnit.io_wakeup` is a FAST wakeup with `bypassable := true` and
`fastWakeupLatency = dfmaLatency - 3` ("Three stages WAKE-ISS-ARB"), so from wakeup cycle T
the writeback is presented at **T+3**. `VecScalarOperandRead` drives the FP address
combinationally in its grant cycle, so a slot woken at T reads address T+1 / data T+2 —
**stale by exactly one cycle and unfixable inside FpPipeline**, because the write has not
happened in any cycle FpPipeline can see. The CONSUMER must close it: either delay the FP
address one cycle relative to the grant, or hold `bypassable` FP-wakeup readiness back one
cycle in `VecIssueSlot`. `ll_wbarb`-sourced wakeups (FP load / ifpu / fdiv) need none of
this — they fire in the same cycle as `fregfile.io.write_ports(0).valid`. **Pick one side
and write it down; this is the FP twin of the M1 stale-scalar-base bug.**

**A31. `vfmv.f.s` has no owner.** `vec_pipeline_io.fp_wb` requires an **FP register-file
write port plus a wakeup slot** inside `fp-pipeline.scala`, which is where
`VecCiiWriteback`'s "dedicated scalar-dest write port and wakeup slot that
`enableVectorArith` adds" must physically live. No requirement in `FpPipeline`'s allocation
covers it, and adding it changes `numFrfWritePorts`/`numWakeupPorts`. FpPipeline correctly
put it on its reject list rather than inventing it. **Needs allocating before Phase F.**

**A32. Giga tier will fail elaboration on the added FP read port.** The port can never be
denied only while `numFrfReadPorts + 1 >= numFrfLogicalReadPorts`. That holds for
Small/Medium/Large (`fpWidth=1`, `numFrfReadPorts=3`) and Mega (`2`/`6`), but **fails for
Giga** (`fpWidth=2`, `numFrfReadPorts=4`): a Giga vector config must raise
`numFrfReadPorts` to 6. It is an elaboration failure, not silent — but `BoomConfigMixins`
should set it.

**A33. `midcore.rst` `spec-wakeups`' FP re-busy clause is vacuous on this baseline.**
It says the FP scalar feeder is "re-busied through the same machinery if the load later
misses", but BOOM v4 has **no speculative load-hit wakeup on the FP side at all** — no
`fwakeups` analogue of `io.lsu.iwakeups`; `io.lsu.fresp` goes to `ll_wbarb` on actual data
return, and every FP wakeup port drives `rebusy := false.B`. Discharged vacuously; no
re-busy machinery added. Flagged in case the spec meant something stronger.

---
# ===> TOP OPEN RULING — GATE (f) BIT-IDENTITY CANNOT HOLD AS CURRENTLY SPECIFIED
# This supersedes the milder note in ScalarOpConstants. It is a PLAN-LEVEL issue, not a
# file-level one, and it must be decided before ANY Phase-A RTL is generated.

**The problem.** Three nodes widen shared encodings UNCONDITIONALLY, i.e. not under
`usingRVV`:
- `ScalarOpConstants` widens `RT_FIX/RT_FLT/RT_X/RT_ZERO` 2b -> 3b (to make room for
  `RT_VEC = 4`) and raises `IQ_SZ` 4 -> 7. A constants **trait** has no `Parameters` in
  scope, so it *cannot* gate on `usingRVV` as written.
- `MicroOp` widens `dst_rtype`, `lrs1_rtype`, `lrs2_rtype` 2b -> 3b to match.
- `Rob` must therefore widen its **compact** `dst_rtype` too, or `RT_VEC = 4` truncates
  to `RT_FIX = 0` — a silent mis-route of every vector destination.

**The consequence.** A `usingRVV = false` build differs from pre-Caracal BOOM v4 by one bit
per bank in the ROB's compact SRAM and one bit per entry in `rob_uop`, plus three bits of
`iq_type` on every uop. So **plan gate (f) — "usingRVV=false bit-identical to pre-Caracal
v4", asserted at every step and in §6 — is false by construction**, not by a mistake in any
one file.

**The two ways out. Pick one; do not leave it implicit.**
(a) **Accept a bounded, documented exception to gate (f).** Amend §6 gate (f) to "identical
    except for the width of the register-type and iq_type encodings, enumerated here", and
    make the gate a diff against a *re-baselined* reference rather than pre-Caracal RTL.
    Cheap, honest, but it weakens the strongest guarantee the plan makes and every future
    step then diffs against a moving baseline.
(b) **Re-gate all three together** so the widths are `usingRVV`-conditional. This means the
    `RT_*`/`IQ_*` encodings stop being a bare `trait` of literals and become parameterized
    (derived in `BoomCoreParams`, or a small object taking `Parameters`), so a non-vector
    build emits 2-bit rtypes and `IQ_SZ = 4` exactly as today. More work, touches three
    already-written specs, and preserves the promise.

My recommendation is **(b)**, because gate (f) is the mechanism that lets every other step
be reviewed cheaply — a moving baseline makes every subsequent bit-identity claim
unfalsifiable. But it is a scope call, not mine to make.

**A34. `VecRobFlags` is named in `vec_pipeline_io` but declared by no node** (VecBundles,
already authored, does not declare it). Same class as `VecCiiTagEntry`/`VecScalarOperands`.

**A35. `numVecClrPorts` is declared nowhere** (same gap as `numVecWbPorts`). `Rob` used
default 3; `VectorParams` should declare both.

**A36. `ALUUnit` must drive the resolved `vconfig` on EVERY `vset*` writeback, not only
`vsetvl`.** `Rob` implements the executed-VTYPE carrier as a per-row `rob_vconfig`
overwritten from the existing `io.wb_resps` loop when `is_vl_producer` is set. It
explicitly rejected the "rely on `vsetvl`'s `is_unique` and use one register" shortcut,
because `vsetvli`/`vsetivli` are **not** `is_unique`.

**A37. Wording conflict resolved in midcore's favour**: `issue.rst` `shared-store-chain`
says the LSU half's *first* address translation clears `rob_unsafe`; `midcore.rst` says the
*last* element address LCAM-checked. `rob.e4` forbids a per-sub-access clear, so midcore
wins; issue.rst's point is *which half*, not which element. Correct issue.rst.

**A38. One existing assert must be weakened**: rob.scala:643-646 ("Committed non-FP
instruction has non-zero fflag bits") needs a `usingRVV`-gated `!is_vec` exemption, because
a vector FP op accrues fflags without setting `fp_val`. The two companion FP asserts stay.

---
# CII SUBTREE — RESOLVED BY VecCiiHost (with two corrections to MY earlier notes)

**MY ERROR 1, corrected**: I told `VecCiiHost` that `spec-cii.b8` was allocated to it. It is
**`VecCiiTagTable`'s** (check the `reqs:` lists). `VecCiiHost` correctly declined to tag it.
**MY ERROR 2, corrected**: the credit grain is the BEAT, not the lane — see the corrected
bullet above. Both were caught by reading `tt_cii_interface.sv` rather than trusting my note.
Lesson for R2: verify seam claims against the frozen SV, not against this file.

**A39. `VecCiiOperandServer`: replace `tag_entry: Vec(4, Input(VecCiiTagEntry))` with the
narrow `src_lookup` response** `{prn, read_vrf, scalar_data, killed, rob_idx}`. The `op_id`
mux lives in `VecCiiTagTable` (whole-entry export would cross 4 x ~376 bits between adjacent
modules to compute the same function at the same logic depth).
**Traceability wrinkle, deliberately left open:** `cii.f5`-`f8`/`f10` are tagged in
`VecCiiOperandServer` while the mux is emitted in `VecCiiTagTable`. `opnd` still owns the
outcome (R5-R8 drive, the scalar no-read path, the beat, the order, the drain).
**R2 must either move f5-f8/f10 to the table or keep them with an explicit
"resolved via sibling" note. Do NOT resolve it by silently re-tagging.**

**A40. `VecCiiTagTable`: drop the `alloc.tag` output and the registered `tag_avail`; export
`tag_free_mask = ~tag_valid` instead.** The free-tag select must live in `VecCiiIssue`,
where the accept-cycle shadow (`s1_pending_mask`) lives: `iss` picks the tag in its accept
cycle and writes the entry in its emit cycle, so `tag_valid` lags a cycle and a back-to-back
grant with no shadow **allocates the same tag twice**. The table never sees the accept cycle
and cannot hold that shadow. Also the table's registered `tag_avail` is computed from
*current* state — the off-by-one that over-advertises by one cycle = a dropped instruction.
The requirement split already reads this way (d6 allocate = `iss`, d8 record = `tags`).
Good news: `VecCiiIssue` already ANDs tag availability into `fu_types` and computes both
terms from NEXT state, so there is no over-advertisement once the select moves.

**A41. `VecCiiWriteback`'s kill self-check must be the IMPLICATION `killed -> wb_suppress`,
never an equality.** An equality fires in the flush cycle on a correct case. The container
drives `wb.io.wb_suppress := tags.wb_lookup.resp.killed || flush.io.kill_all` — both terms
needed (`killed` covers every beat from the cycle after the flush; `kill_all` covers the beat
arriving *in* the flush cycle). Naming settled: flush exports `kill_all`, the writeback input
is `wb_suppress`. `VecCiiWriteback`'s dependencies section calls it `io.kill_all`,
inconsistent with its own ports section — fix the prose.

**A42. `VecBundles.CiiIssueReq.src_reuse_hint` must be 4 bits** (`cii_caracal_frwd_hint_t` /
`CII_NUM_SRC_SLOTS`). Confirms A11 from the container side; the flat `iss_hint` port is 4.

**A43. The `addPath` calls must sit in the BlackBox's own CONSTRUCTOR BODY**, not in a Scala
`object` initializer. Hoisted, they drag the SV into a **vectors-off** filelist and make that
build require the tt-cii submodule — breaking gate (f) for a reason unrelated to logic.

**A44. The SV-constant cross-check is a BUILD step, not a Chisel `require`.** Chisel cannot
parse the `.svh` at elaboration. One Scala mirror object holds the numbers; the same build
step that verifies the submodule extracts the localparams and fails on drift. A hand-edited
mirror value with no `.svh` change is a reviewer-reject.

**A45. `src/main/sv/v4/tt-cii` is NOT registered in `.gitmodules`** (`git submodule status`
errors on it). The build check must therefore verify **contents present**, not
`submodule status`. Confirm with whoever vendors it. Also confirmed:
`src/main/resources/vsrc/` holds only `btb_harness.v`/`predictor_harness.v` — **no stale CII
copies exist**, so "no copies" is a rule to keep, not a cleanup.

**A46. `VecCiiHost` instantiates the receive FIFOs DEGENERATE (`ciiRxDepth = 0`)** —
unregistered passthrough valid, credits forwarded unaltered. Justified: both consumers are
structurally unstallable (`opnd` owns R5-R8 outright with fixed 1-cycle reads, `wb` owns W2
outright and places combinationally), so the pop IS the arrival. The 16 in
`CII_N_{REQ,WB}_CREDITS` is the **sender's** in-flight allowance, not a depth the host must
build. Make either consumer stallable and the credit must move to the pop of a real buffer.

**A47. The Src-Data credit counter's stall is an ASSERTION, not a gate.** `dat_valid` must
never be gated on the counter — dropping a beat desynchronises the positional channel
**permanently**. Exhaustion is unreachable absent a VPU-side protocol violation; the counter
exists to make that visible.

**A48. `spec-core.f11` "each issue slot must be granted once" is FALSE AS LITERALLY WRITTEN
for a vector store**, which is granted twice (AGEN then DGEN, `spec-issue.d10`). `VecIssueUnit`
resolved it as "once per EXECUTION RESOURCE, not a literal grant count". **A generator that
asserts `PopCount(grants per slot) == 1` over an entry's lifetime would break every vector
store.** The `squash_grant`/re-busy replay is likewise a second grant of the same select.
Reword f11, or keep it with the exception written down.

**A49. DO NOT COPY baseline's SNI block into the vector issue unit — one of its terms is
actively wrong here.** `issue_slot_past_pnr` includes `| (rob_idx === rob_pnr_idx)`, but
`rob_pnr_idx` names the **oldest UNSAFE** entry (rob.scala:531), so equality **admits the
unresolved entry**. In the scalar SNI context that is intended; for `IQ_V_ALU`'s past-PNR gate
it would hand a still-speculative op to the CII, defeating the entire reason `pnrGate` exists.
The past-PNR test must be strict `IsOlder(rob_idx, rob_pnr_idx, rob_head_idx)`.
Three more baseline blocks are on the reject list for the vector slots: IQ_MEM's
`lrs2_rtype := RT_X; prs2_busy := false` FP-store fixup (**vector `prs2` is the STRIDE**),
IQ_MEM's `prs3_busy := false`, and IQ_UNQ's `prs2`/`pimm` rewrites.

**A50. CONSISTENT ACROSS THE SEAM (no action):** `VecIssueUnit` kept baseline
`flush_pipeline = RegNext(rob.io.flush.valid)` and added no combinational `rob_flush` gate,
leaving the one-cycle flush-allocation window (item 13) to be absorbed downstream — and
`VecCiiHost` independently chose exactly that absorption (`kill_all = rob_flush ||
rob_flush_kill`, plus `iss` pre-setting `killed`). The two agree. Verify in R2 rather than
re-opening.

**A51. `fu_types` type mismatch at the CII seam**: `VecIssueUnit` declares baseline's
`Vec(FC_SZ, Bool())`; `VecCiiIssue` declares `UInt(FC_SZ.W)`. Same information — `VecPipeline`
connects with `.asBools`. Keep baseline's type so the diff stays clean.

---
# VecLsu SETTLEMENTS + the last crop of defects

**A52. ⇒ `q_free` ON `VecQueueReservation` HAD NO DRIVER AT ALL.** Nothing anywhere in the
17-node subtree generated head-side reclamation, so the six element queues would fill once
and **never free** — a hang, reached by any program that issues more vector memory ops than
one reservation's worth. `VecLsu` specifies the fix: it presents the RETIRING LDQ/STQ index
to `resv`, which invalidates the row and echoes base/count to drive both the queues'
region-free and its own occupancy decrement. **This needs ONE ADDED PORT PAIR on
`VecQueueReservation`** (`retire` in / `region_free` out) — the four lookup lanes are all
taken by agens. Highest-priority amendment on this list.

**A53. US store address/data regions are NOT in identity correspondence.**
`st_US_ADDR_Q` holds 1 entry per store; `st_US_DATA_Q` holds `v_emul * nf`. So
`VecQueueReservation`'s "one row holds one shared base, assert they agree" and its single
`used_count` release hold for the **SSI pair ONLY**. The reservation must carry a base AND a
count PER QUEUE SLOT (its `resv_out` already has two slots); `us_data_base` on the range
entry comes from slot 1. Confirms `VecStoreForward`'s independent report of the same gap.

**A54. `VlRegFile.W_lsu` had TWO candidate producers** — `lcb.io.vl_wb` and
`ld_elem_agen.ff_trim`. Settled: **`lcb.io.vl_wb` ONLY**; the agen's `ff_trim` routes into
`lcb.io.trim` and the LCB owns the single VL-RF write. `VecElemAgen`'s "routed by VecLsu to
W_lsu" is superseded.

**A55. `VecLsuCoreIO` is now declared TWICE.** `LSU`'s delta says it declares it in
`lsu.scala` beside `LSUCoreIO`; `VecLsu` says it declares it in its own file as an aggregate
of the children's bundles. **Exactly one may survive.** Recommend `lsu.scala`, since
`VecBundles.VecPipelineIO` already has a `lsu_vec: VecLsuCoreIO` field referencing a
`boom.v4.lsu` type and the LSU owns the LDQ/STQ pointers it carries.

**A56. `VecRangeAgen.io.st_data`** (per-member whole-register capture commands to `VecDgen`)
is **REDUNDANT** with VecDgen's own US member sequence derived from `io.req`/`members_used`.
`VecLsu` ties its `ready` high and uses it only as an agreement assertion. Reconcile in one
of the two files — do not leave two mechanisms.

**A57. `VecScalarOperandRead.out` has no `ready`, but `VecRangeAgen.io.req` is `Decoupled`.**
The ready is guaranteed high for a reserved US region (one entry per instruction, never
pre-filled), so `VecRangeAgen.io.req` **should be a `Valid`**, not `Decoupled` — a ready that
is always true invites a generator to add back-pressure that must not exist.

**A58. `VecSquashUnit`'s `nKillClients` comment is stale after the mask-streamer hoist.**
`VecLsu` sets it to **5**, not the default 8, with fixed ordering
`{ld_msk, st_msk, ld_beat, st_beat, gcopy}`. The four agens and `dgen`/`xx_opnd` take
`brupdate`/`rob_flush` DIRECTLY — `VecElemAgen` *must*, because two uops carry two different
`br_mask`s and one pre-resolved Bool cannot serve both — and `idx` is killed by its parent.

**A59. ROLLBACK CONVENTION SETTLED — BOOM's EXCLUSIVE tail.** `[head, idx)` survive. The new
queue tail is `base+count` of the row owned by the youngest LSQ entry **strictly older** than
the driven index, else the queue head — **not** the driven index's own row. Asserted in
`VecLsu`. **`VecQueueReservation` §8 reads as INCLUSIVE and must be corrected**, or one killed
instruction's entries stay live.

**A60. `VecGroupCopy` is deliberately EXCLUDED from `vec_lsu_empty`** (a pending copy is a VRF
write, not memory state) — so it does not hold up `fencei_rdy`. Confirm that is intended:
a fence does not order VRF writes, so it looks right, but it is a judgement call.

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
  ALUUnit — DELTA SPEC. This file is NOT a description of BOOM's integer ALU
  functional unit. It describes only the change Caracal applies to the existing
  `class ALUUnit(dataWidth: Int)` in
  src/main/scala/v4/exu/execution-units/functional-unit.scala, which is
  hand-written baseline BOOM v4 and stays in place.
*/

hierarchy.yaml: kind: module, mode: edit_existing,
target src/main/scala/v4/exu/execution-units/functional-unit.scala,
group: host. No `output:` — the pre-existing file is the artifact.
depends_on MicroOp, ScalarOpConstants, VtypeTable.
Budget (plan v2 §11 file-touch summary): ~70 added lines.

THE CHANGE, IN ONE SENTENCE: under `usingRVV`, when the uOP arriving at the
integer ALU is a register-sourced `vset` (`vsetvli` or `vsetvl`), the unit
resolves that instruction's `vtype`, computes the new VL through the
VtypeTable package, and drives that VL onto its EXISTING result bus in place
of the rocket-ALU output. Nothing else about the unit moves.

Everything already in `class ALUUnit` is unchanged and is not restated below —
the operand selects, the rocket ALU instance, the branch comparators and
`brinfo`, the SFB path. Any signal this file does not name keeps its current
definition and behaviour exactly.

===> THIS DELTA ADDS NO PORT. `vsetvli`/`vsetvl` arrive through the ordinary
     `io.req` with `fu_code(FC_ALU)` set (see ALUExeUnit's delta), and the
     result leaves on the existing `io.resp`. Both destinations — `rd` in the
     integer RF and `pvl` in the VL RF — take the SAME value off that ONE
     result bus, which is what makes the dual-destination rule cheap. A new
     VL-RF port or a mirror-write port appearing here is a failed review.

===> THE BUG NOT TO RE-INTRODUCE: COMPARE THE FULL-WIDTH AVL. `addvector`
     truncated the AVL to `vecVLSz+1` bits before comparing it against VLMAX,
     so a large AVL WRAPPED instead of saturating — AVL=2048 produced vl=0 —
     which breaks the canonical strip-mining loop, where AVL is the whole
     remaining element count and must saturate to VLMAX on every iteration
     but the last. Pass `io.req.bits.rs1_data` at its full `xLen` width. Note
     `vecVLSz` is 9 bits, not 6.

Governing spec anchors: frontend.rst `vector-rvv-decode` ("VSET Special
Handling") and `vset-dual-dest` and `vl-delivery`; midcore.rst
`regfiles-bypass` and `vl-vtype-rename`; issue.rst `issue-vl-delivery`.

<|begin_module|>

  <|begin_parameters|>
  No new constructor parameter. `class ALUUnit(dataWidth: Int)` keeps its single
  argument and its `FunctionalUnit(isAluUnit = true, dataWidth = dataWidth)`
  superclass call unchanged, and it is still instantiated as
  `Module(new ALUUnit(dataWidth = xLen))` by `ALUExeUnit`.

  Two elaboration-time values already in scope through `BoomModule` /
  `HasBoomCoreParameters` are the only new reads:

  - `usingRVV` — the Scala `Boolean` from `BoomCoreParams` (`boomParams.
    enableVector`), NOT a hardware `Bool`, and NOT rocket's `usingVector`.
  - `vecVLSz` — 9 bits at the defaults, the width of a VL value. Used only to
    size the zero-extension of the computed VL up to `dataWidth`; it is never a
    truncation point for the AVL.

  //@req-spec-decode.c14
  //@req-spec-decode.c15
  ---- THE GATE ----

  The whole extension is wrapped in a Scala `if (usingRVV) { ... }`, and the one
  expression it has to modify (the `alu_out` result mux) is written so that the
  `usingRVV = false` elaboration produces the baseline expression TEXTUALLY,
  not a Mux with a constant-false select. With vectors off this unit must be
  BIT-IDENTICAL to BOOM v4: no added mux level in the result path, no added
  comparator, no added wire, no reference to a vector MicroOp field. Gate (f)
  diffs the emitted Verilog of a non-vector config against the pre-Caracal
  baseline, and the ALU result path is the most timing-sensitive place in the
  machine for an unnecessary mux to appear — so "absent, not tied off" is a
  correctness requirement here and not a tidiness preference.
  <|end_parameters|>

  <|begin_ports|>
  NO PORT IS ADDED, WIDENED, REMOVED OR RETYPED. The `io` bundle declared in
  `abstract class FunctionalUnit` — `kill`, `req`, `resp`, `brupdate`,
  `fcsr_rm`, `brinfo` — is untouched, as is `io.req.ready := true.B`.

  The delta reads three things that the existing bundle already carries, and
  writes one field of one bundle it already drives:

  - reads `io.req.bits.rs1_data` (the AVL) and `io.req.bits.rs2_data` (a
    `vsetvl`'s `vtype` word), both at full `dataWidth` (`xLen`) and both read
    DIRECTLY, exactly as the branch comparators already read
    `val rs1 = io.req.bits.rs1_data` — NOT through `op1_data`/`op2_data`, whose
    MuxLookups are scalar-decode paths and must not gain a vector case.
  - reads the vector fields of `io.req.bits.uop`: `is_vl_producer`, `vconfig`,
    and the baseline `lrs1_rtype` / `lrs2_rtype` / `dst_rtype` register types.
  - drives `io.resp.bits.data` (already driven) and `io.resp.bits.uop.vconfig`
    (a field of an already-driven bundle — see the resolved-vtype paragraph in
    the logic section).

  Clock and reset are Chisel's implicit pair and are unchanged: posedge
  `clock`, ACTIVE-HIGH SYNCHRONOUS `reset`, core clock domain (`core_clk` /
  `core_reset` in hierarchy.yaml). This delta adds no state, so it adds no
  reset behaviour at all.

  Deliberately absent: no VL-RF write port, no VL-RF read port, no output to the
  speculative VCFG mirror, no second integer write port, no new or widened
  `ExeUnitResp` field, no `vl`/`vtype` side channel. All fan-out lives one and
  two levels up; the edit_scope section carries the full reject list.
  <|end_ports|>

  <|begin_logic|>
  Everything below is inside the `usingRVV` gate.

  ---- 1. Recognising the uOP, without re-decoding it ----

  A register-sourced `vset` is identified by `io.req.bits.uop.is_vl_producer`
  (the MicroOp field added by the MicroOp delta). On a uOP that reached this
  unit, that bit means `vsetvli` or `vsetvl` and nothing else: `vsetivli` is
  front-end only and never gets an issue slot or an EU, and `vleff` — the third
  VL producer class — is an LSU uOP that never sets `fu_code(FC_ALU)`.

  ===> CORRECTED — THE SENTENCE ABOVE IS WRONG ABOUT `vsetivli`, AND THE ERROR
       PRODUCED A WRONG ARCHITECTURAL RESULT. **`vsetivli` with `rd != x0` DOES
       reach this unit.** `VsetDecode` routes it to `IQ_ALU`/`FC_ALU`, and
       `VecDecode` gives the reason in its own file: "rd needs an integer-RF write
       the front end has no port for". Only an execution unit has that port, so
       "front-end only" can be true of the VL-RF write and of `vsetivli`'s *vtype*
       — never of its `rd`.

       The consequence of believing it: this unit read AVL from
       `io.req.bits.rs1_data`, but `vsetivli`'s AVL is the immediate
       `inst(19,15)`, so `VsetDecode` sets `lrs1_rtype := RT_X` and `imm_sel :=
       IS_N` — `rs1_data` is a register that was never renamed or read. The stale
       value was `>= maxVLMax`, saturating to VLMAX, so
       `vsetivli x11, 1, e16, m2` wrote **32** where `min(1, 32) = 1`. Found as a
       cosim Register Mismatch against Whisper at gate (e1), on the first vector
       instruction the test executes.

       Resolution: `MicroOp` gains `v_vl_imm`, the VL **VConfigUnit already
       computed at decode**, and this unit selects it for a `vsetivli`
       (`lrs1_rtype === RT_X`, exact because the other two shapes are `RT_FIX` or
       the `RT_ZERO` VLMAX request) instead of recomputing. Carrying the RESULT
       and not the AVL is deliberate: `vsetivli` has two destinations that must
       receive the same value — `rd` here and `pvl` in the VL RF at rename — and
       a second `min(AVL, VLMAX)` evaluation in this unit could only create a way
       for them to disagree. `vleff` remains genuinely absent from this unit.

  Which of the two forms it is comes from `uop.lrs2_rtype === RT_FIX`: only
  `vsetvl` encodes a second integer source, so the decoder renames `rs2` for
  `vsetvl` and leaves `lrs2_rtype` non-`RT_FIX` for `vsetvli`. Call that
  `vset_vtype_from_rs2`.

  NO INSTRUCTION RE-DECODE. The unit must NOT look at `uop.inst` to tell the
  forms apart — same structural reason MicroOp's delta gives for the
  access-class fields: a second decoder that disagrees with the first is a
  silent wrong answer, and this one would sit in the ALU's result path.

  //@req-spec-decode.c11
  ---- 2. The `vtype` this instruction configures ----

  One `vtype` word feeds the VL computation:
    `vset_vtype_bits = Mux(vset_vtype_from_rs2, io.req.bits.rs2_data,
                           io.req.bits.uop.vconfig.asUInt)`
  For `vsetvl` the `vtype` is a runtime register value and this unit is the
  first place in the machine that can resolve it — that IS the "computes VTYPE
  for `vsetvl`" obligation. For `vsetvli` the `vtype` is immediate and was
  already resolved at decode, and the uOP's own `vconfig` snapshot holds its NEW
  `vtype` (the per-lane prefix select is self-inclusive for a `vset` — see
  frontend.rst's note under `vset-dual-dest`), so reading `vconfig` reuses that
  result instead of decoding the `zimm` field a second time.

  Resolve legality and VLMAX with exactly ONE call to the VtypeTable package:
  `VtypeTable.decode(vset_vtype_bits)`, giving `vlmax`, `vill`, `vta`, `vma`.
  Both forms go through that single call, so `vsetvli` and `vsetvl` cannot
  disagree about whether a configuration is legal.

  DELEGATE; DO NOT REIMPLEMENT. VtypeTable wraps rocket-chip's
  `freechips.rocketchip.rocket.VType`, the same declaration rocket's CSRFile
  uses for architectural `vtype` under `usingVector`; an ALU-local vill/VLMAX
  rule would let this unit admit a `vtype` the architectural CSR calls
  `vill`. A `vill` result is NOT an exception here — RVV 1.0 says such a vset
  sets `vill` and forces VL to 0 — so this unit never touches
  `uop.exception`/`uop.exc_cause`, and for `vsetvl` the poison reaches
  younger uOPs through `flush_on_commit`, not from here.

  //@req-spec-decode.c10
  ---- 3. The VL computation ----

  `vset_vl = VtypeTable.computeVL(avl = io.req.bits.rs1_data,
                                  bits = vset_vtype_bits,
                                  currentVL = 0, useCurrentVL = false,
                                  useMax = vset_use_max, useZero = false)`
  which is `min(rs1, VLMAX)`, with `vill` forcing 0, delegated to rocket's
  `VType.vl(...)`. The result is `vecVLSz` (9) bits wide.

  ===> THE AVL ARRIVES AT FULL `xLen` WIDTH AND IS NOT NARROWED BEFORE THE
  COMPARE. This is the `addvector` bug verbatim: it truncated AVL to
  `vecVLSz+1` bits first, so AVL=2048 wrapped to 0 and produced vl=0 instead
  of saturating at VLMAX, and the canonical strip-mining loop
  (`vsetvli t0, a0, ...` with a0 = elements remaining) silently made no
  progress. Rocket's `vl(...)` is correct BY CONSTRUCTION and that is the
  reason to delegate: it forms `atLeastMaxVLMax` from the FULL-width
  `avl >= maxVLMax` comparison FIRST and only then indexes the low
  `log2(maxVLMax)` bits, so the truncation it does perform applies to a
  residue already known to be below maxVLMax. Anything narrower than
  `rs1_data` passed here re-creates the bug.

  //@req-spec-decode.i10
  `vset_use_max` is `uop.lrs1_rtype === RT_ZERO`, i.e. the AVL source is
  architecturally `x0`. For `rs1 == x0` with `rd != x0` the new VL is VLMAX,
  computed here from `vtype` — there is no decode-time fast path for it, and
  this unit is where VLMAX is produced.

  ===> THIS FLAG CANNOT BE INFERRED FROM THE DATA. `rs1_data` is 0 both for
       `rs1 == x0` (meaning "use VLMAX") and for a GPR that happens to hold 0
       (meaning "VL = 0"), so the two cases are indistinguishable at execute
       without the decoded register type. Using the value would turn every
       `vsetvli rd, x0` into a zero-length configuration.

  `useCurrentVL` is TIED FALSE and `currentVL` is tied 0, deliberately: the
  keep-VL form `vsetvli x0, x0, vtype` is NOT a VL producer at all
  (frontend.rst `vl-delivery`) — the VL map table is left untouched and younger
  uOPs keep their existing `pvl` — so no VL value has to be preserved here.
  `useZero` is tied false because `vill` already forces VL to 0 inside
  `VType.vl(...)`.

  The M1 implementation gave this unit a VL-RF READ PORT (`pvl_src`) for
  keep-VL and paid for it with an iss+3 pipe-timing fix. v2 deletes that path
  by making keep-VL a non-producer at decode. Do NOT add the read port back:
  VlRegFile's port table has no ALU read port (R_exe is one per vector issue
  queue, R_commit is the ROB's). If a keep-VL uOP does reach this unit, its
  `is_vl_producer` is clear, so the computed VL is discarded by the mux in
  part 4 and no VL-RF write is enabled anywhere.

  //@req-spec-decode.c17
  ---- 4. One result bus, two destinations ----

  Extend the EXISTING `alu_out` result mux with the vset case at highest
  priority, so `io.resp.bits.data` carries the new VL zero-extended from
  `vecVLSz` to `dataWidth` whenever `uop.is_vl_producer` is set, and carries
  the baseline `Mux(is_sfb_shadow && pred_data, ..., Mux(is_mov, rs2_data,
  alu.io.out))` otherwise. The rocket ALU still evaluates for a vset; its
  output is simply not selected. `rd` receives the NEW `vl`, so both
  destinations take the SAME value off this ONE bus — no second result path, no
  arbiter, no extra write port.

  Assert that an `is_vl_producer` uOP is never `is_sfb_br`, `is_sfb_shadow` or
  `is_mov`, rather than relying on the mux priority — a `vset` in an SFB shadow
  would otherwise be silently converted to a predicated move.

  //@req-spec-decode.c12
  //@req-spec-decode.c19
  //@req-spec-vrf.c2
  //@req-spec-vrf.c3
  ---- 5. Fan-out A: the integer register file ----

  When `rd != x0` the `vset` additionally writes `rd` in the integer RF, and that
  write needs NO new logic here or above: `dst_rtype` keeps its unmodified
  integer meaning, so the pre-existing gating in core.scala —
  `iregfile.io.write_ports(wb_idx).valid := unit.io_alu_resp.valid &&
  unit.io_alu_resp.bits.uop.dst_rtype === RT_FIX` — already fans the result out
  for exactly the `rd != x0` case and leaves it un-enabled where the decoder set
  `RT_ZERO` for `rd == x0`. The `int_bypasses` and `int_wakeups` entries gated on
  the same `RT_FIX` term are likewise unchanged, so a scalar dependent of `rd`
  sees the new `vl` at ordinary ALU bypass latency.

  That `rd` GPR write is an ORDINARY integer destination and is the ONLY
  integer-register-file interaction of the entire vector path — which is what
  "integer rename is not modified" actually requires, and what would have been
  impossible had VL been renamed as a GPR. Nothing here may add an integer RF
  port or make an integer RF write conditional on a vector field.

  //@req-spec-decode.c20
  //@req-spec-issue.h5
  //@req-spec-issue.h6
  ---- 6. Fan-out B: the VL register file and the VL wakeup network ----

  The same response, with `uop.is_vl_producer` set on it, is the carrier for the
  VL-RF write: BoomCore taps this unit's `io_alu_resp` (the `ALUExeUnit` output
  that already exists) onto `vec_pipeline_io.vset_resp`, and inside VecPipeline
  it drives VlRegFile's `W_alu` port — `addr` from `uop.pvl`, `data` from the
  low `vecVLSz` bits of `resp.bits.data` — with `is_vl_producer` as the write
  enable, while `vl_rename` broadcasts `pvl` on the VL wakeup network in the
  same cycle so dependent vector uOPs wake.

  ===> THE WRITE ENABLE IS `is_vl_producer`, NEVER `dst_rtype`. For
       `vsetvli x0, rs1, vtype` the integer destination is discarded and
       `dst_rtype` reads `RT_ZERO`, yet the VL RF MUST STILL be written. A
       consumer that inferred "writes VL" from `dst_rtype` would drop that write
       and every dependent vector uOP would read a stale VL. This is exactly why
       `is_vl_producer` is a field of its own and orthogonal to `dst_rtype`, and
       why this unit passes it through untouched on `io.resp.bits.uop`.

  This unit therefore drives the VL RF and the VL wakeup network THROUGH its
  ordinary response and owns neither: it adds no VL-RF port and no wakeup
  broadcaster. `vset_resp` is a single port, so exactly one ALU EU may execute
  a register-sourced `vset` — if a wide tier ever lets two of them, VlRegFile's
  `numAluWritePorts` is REPLICATED, never arbitrated (its own spec says so),
  because this response path has no back-pressure line to report a lost port on.

  ---- 7. The resolved `vtype` on the response ----

  For a `vsetvl` the resolved `VType` (part 2) is written onto
  `io.resp.bits.uop.vconfig`, overwriting the decode-time snapshot, which for
  `vsetvl` could not have held the new value. This is the only carrier the
  instruction's new `vtype` has from execute to commit, where the ROB writes
  the architectural `vtype` CSR and the committed VCFG shadow. For every other
  uOP — including `vsetvli` — `vconfig` passes through unmodified, since the
  decode snapshot is already correct.

  SEAM, not a local decision: the Rob delta must latch `vconfig` from this
  writeback for a `vsetvl`, because a ROB entry captures its uop at dispatch.
  If it does not, `vsetvl`'s architectural `vtype` write has no carrier and
  the ALU's computed VTYPE is dead. Chosen over widening `ExeUnitResp`
  because that bundle is shared by every EU in the machine.

  //@req-spec-decode.e5
  ---- 8. What this unit must NOT do: write the speculative `vtype` mirror ----

  There is NO execute-time write to the speculative VCFG `vtype` mirror, from
  this unit or from anywhere else. `vsetvl` is marked BOTH `is_unique` AND
  `flush_on_commit` by the DecodeUnit delta, so the mirror write disappears
  entirely: the ROB updates the COMMITTED VCFG shadow at `vsetvl`'s commit and
  `flush_on_commit` refetches everything younger against that shadow (row 2 of
  the recovery table, one cycle, already implemented). An execute-time mirror
  write would instead need a new recovery path for a mirror updated out of
  program order by a possibly-wrong-path uOP, and buys nothing.

  KNOWN SPEC DEFECT: overview.rst:158 says the ALU DOES write the mirror at
  execute. frontend.rst `vector-rvv-decode` and plan v2 contradict it and
  WIN. A mirror-write port appearing here is a failed review.

  ---- 9. Non-interference, speculation, and tracing ----

  The branch/jump outputs (`io.brinfo`, `mispredict`, `pc_sel`, `is_taken`,
  `jalr_target`, `cfi_idx`) are left exactly as they are: a `vset` has
  `br_type === B_N` and is neither `is_br` nor `is_jalr`, so `brinfo.valid` is
  false for it with no added term. `io.kill` and `io.brupdate` keep their
  current (unread-by-this-unit) treatment — a wrong-path `vset` is killed like
  any wrong-path ALU op, and its VL-RF write lands in a `pvl` the VL free list
  reclaims and that no surviving consumer names, so the stored value becomes
  unreachable rather than wrong. That is why neither this unit nor VlRegFile
  needs a flush port. The unit stays single-cycle, stateless and combinational
  from `req` to `resp`.

  This delta adds NO VecTrace call — a deliberate exception to the guarded-trace
  ground rule, because the event is already covered from both sides (VlRegFile
  emits `wr_alu` with `pvl`, `vl` and `rob_idx` for precisely this writeback,
  and for `rd != x0` the cosim sees the GPR write), and because a call here
  would add a VecTrace dependency this node's `depends_on` list does not have.
  The SFB-exclusivity assertion in part 4 is the whole added debug surface.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
The integer ALU result path is the machine's tightest timing loop — it feeds the
single-cycle bypass network that back-to-back dependent ALU ops rely on — so
this delta has exactly one performance obligation: ADD AT MOST ONE MUX LEVEL AT
THE OUTPUT, AND NOTHING TO THE ALU'S OWN PATH.

- The VLMAX/VL computation runs in PARALLEL with the rocket ALU, not in series
  with it: a shifter plus a comparator tree on `rs1_data`, `rs2_data` and
  `uop.vconfig` (VtypeTable's perf section commits to that shape and to no
  multiplier or divider), shallower than the 64-bit adder it races. Only the
  final select is in series, and it merges into the existing `alu_out` mux
  chain rather than adding a level after it.
- No new pipeline stage, no new register, no change to `io.resp.valid`: a
  `vset` occupies an ALU EU for one cycle like any integer op.
- The VL wakeup consequence is that a dependent vector uOP wakes in the
  writeback cycle and is granted no earlier than the next, which is what lets
  VlRegFile have NO read-during-write bypass. If this unit were ever pipelined
  or its response registered, that assumption must be re-checked there.
- With `usingRVV = false`: zero added gates, and the emitted Verilog for this
  module is bit-identical to pre-Caracal BOOM v4.
<|end_perf|>

<|begin_dependencies|>
Per hierarchy.yaml, `depends_on: [MicroOp, ScalarOpConstants, VtypeTable]`.
`instantiates:` adds NOTHING — the delta creates no new module instance, and the
existing `Module(new freechips.rocketchip.rocket.ALU())` is pre-existing and
unchanged.

- VtypeTable (`boom.v4.vec.generated`) — `decode(bits)` for `vlmax`/`vill`, and
  `computeVL(...)` for the VL. This unit is one of that package's three callers
  (VConfigUnit and VsetDecode are the others) and must not restate its rules.
  Through it, and only through it, this delta binds to rocket-chip's
  `freechips.rocketchip.rocket.VType`.
- MicroOp — `is_vl_producer`, `vconfig`, `pvl` (carried, not read here), and the
  widened `dst_rtype` / `lrs1_rtype` / `lrs2_rtype`.
- ScalarOpConstants — `RT_FIX` and `RT_ZERO` comparisons (3 bits wide after that
  delta's widening; the comparisons themselves keep their current results).

Seam peers, all wired OUTSIDE this file:
- ALUExeUnit — advertises the widened `iq_type`/`fu_types` so a `vset` routes to
  the ALU EU, and exports this unit's response as `io_alu_resp`. Unchanged by
  this delta.
- BoomCore — taps `io_alu_resp` onto `vec_pipeline_io.vset_resp` (must NOT gate
  that tap on `dst_rtype === RT_FIX`, or the `rd == x0` VL write is lost), and
  keeps the existing `RT_FIX`-gated integer RF write, bypass and wakeup entries
  untouched.
- VecPipeline / VlRegFile — consume `vset_resp` on VlRegFile's `W_alu` port;
  VecRenameSpace's `vl_rename` instance owns the `pvl` allocation, the VL busy
  bit and the VL wakeup broadcast.
- DecodeUnit — sets `is_vl_producer`, `fu_code(FC_ALU)`, the `RT_ZERO` register
  types for `x0` operands, the `vconfig` snapshot, and `vsetvl`'s `is_unique` +
  `flush_on_commit`. Every decode-provided bit this unit reads is that delta's
  obligation to set correctly.
- Rob — the commit-time architectural `vtype`/`vl` write, and the `vconfig`
  latch for `vsetvl` described in part 7 of the logic section.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File  src/main/scala/v4/exu/execution-units/functional-unit.scala
    Class `class ALUUnit(dataWidth: Int)(implicit p: Parameters) extends
          FunctionalUnit(isAluUnit = true, dataWidth = dataWidth)`
          (package boom.v4.exu). Hand-written baseline BOOM v4.
    Budget ~70 added lines, all inside `class ALUUnit`.

  In scope:
    - Adding, inside a Scala `if (usingRVV)` gate in `class ALUUnit`, the
      wires: the `is_vl_producer` recogniser, `vset_vtype_from_rs2`
      (`uop.lrs2_rtype === RT_FIX`), `vset_vtype_bits`, `vset_use_max`
      (`uop.lrs1_rtype === RT_ZERO`), the `VtypeTable.decode` result and
      `vset_vl`.
    - Extending the existing `val alu_out` mux expression with the vset case at
      highest priority, and only that expression in the response block.
    - Driving `io.resp.bits.uop.vconfig` with the resolved `VType` for a
      `vsetvl` (the rest of `io.resp.bits.uop` stays the current
      `:= io.req.bits.uop` pass-through).
    - Adding the SFB-exclusivity assertion named in the logic section.
    - Adding the `import` for `boom.v4.vec.generated.VtypeTable`.

  Must not regress:
    - ⇒ THE MOST IMPORTANT LINE HERE: with `usingRVV = false`, `class ALUUnit`
      elaborates BIT-IDENTICALLY to pre-Caracal BOOM v4 — identical Verilog,
      identical `alu_out` expression, no extra mux, wire, comparator or constant
      in the result path, and no reference to any vector MicroOp field.
    - The rocket ALU instance and its four inputs `alu.io.in1`, `alu.io.in2`,
      `alu.io.fn`, `alu.io.dw` are untouched, including the
      `Mux(uop.op1_sel === OP1_RS1SHL, DW_64, uop.fcn_dw)` term.
    - `imm_xprlen`, `block_pc`, `uop_pc`, `op1_shamt`, `op1_shl`, `op1_data`,
      `op2_oh` and `op2_data` keep their exact current definitions. No new
      `OP1_*` / `OP2_*` case is added to either MuxLookup.
    - The branch/jump path is cycle- and bit-identical: `br_eq`, `br_lt`,
      `br_ltu`, `pc_sel`, `is_taken`, `target_offset`, `encodeVirtualAddress`,
      `jalr_target_base`, `jalr_target_xlen`, `jalr_target`, `cfi_idx`,
      `mispredict`, the whole `brinfo` bundle and `io.brinfo`.
    - The SFB path is unchanged for every non-vset uop: `is_sfb_shadow` with
      `pred_data` still selects `rs1_data`/`rs2_data` per `ldst_is_rs1`,
      `is_mov` still selects `rs2_data`, `is_sfb_br` still selects
      `pc_sel === PC_BRJMP`, and `io.resp.bits.predicated` is unchanged.
    - `io.req.ready := true.B`, `io.resp.valid := io.req.valid`,
      `io.resp.bits.uop := io.req.bits.uop` (apart from the single `vconfig`
      field above) and `assert(io.resp.ready)` are unchanged.
    - `abstract class FunctionalUnit` and its `io` bundle, and the sibling
      classes in the same file (`FuncUnitReq`, `BrInfoBundle`,
      `BrResolutionInfo`, `BrUpdateInfo`, `BrUpdateMasks`, `FPUUnit`,
      `IntToFPUnit`, `DivUnit`, `PipelinedMulUnit`), are NOT touched at all.
    - File header, comment style and declaration order preserved; no
      reformatting of untouched lines.

  Interface delta:
    NONE. No new port, no widened port, no new constructor parameter, no change
    to `ExeUnitResp` or `FuncUnitReq`. The only interface-visible change is the
    VALUE now appearing on `io.resp.bits.data` (the new VL) and on
    `io.resp.bits.uop.vconfig` (the resolved `vtype`) for a uOP with
    `is_vl_producer` set.

    Explicitly NOT added, and a reviewer should reject them if they appear:
      - a VL register file WRITE port on this unit (the write is VecPipeline's,
        off `vset_resp`);
      - a VL register file READ port / `pvl_src` input (the M1 keep-VL path —
        keep-VL is a non-producer in v2 and VlRegFile's port table has no ALU
        read port);
      - any write to the speculative VCFG `vtype` mirror, or any port toward
        VConfigUnit (spec-decode.e5; overview.rst:158 is a known defect);
      - a second integer RF write port, or any new integer-RF interaction
        beyond the existing `RT_FIX`-gated `rd` write;
      - a new field on `ExeUnitResp` or `FuncUnitReq`;
      - a `usingVector` gate anywhere (the gate is `usingRVV`);
      - any use of `uop.inst` to distinguish the vset forms;
      - any narrowing of the AVL before the VLMAX comparison.
<|end_edit_scope|>

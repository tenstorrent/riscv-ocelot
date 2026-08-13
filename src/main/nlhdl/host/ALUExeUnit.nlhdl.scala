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
  ALUExeUnit — DELTA SPEC. Describes only the change that makes a
  register-sourced `vset` (`vsetvli`, `vsetvl`) execute on the integer ALU
  execution unit. The target is `class ALUExeUnit(val id: Int)` in
  src/main/scala/v4/exu/execution-units/execution-unit.scala, which is
  hand-written baseline BOOM v4.
*/

  hierarchy.yaml: kind: module, mode: edit_existing,
  target src/main/scala/v4/exu/execution-units/execution-unit.scala,
  group host, depends_on ScalarOpConstants. No `output:` — nothing is generated
  from this file. Budget (plan §11): ~10 added lines, the SMALLEST delta in the
  design.

  Everything not mentioned here is unchanged by definition. In particular the
  ALU EU's existing ports, its ARB/RRD/EXE pipeline registers (`arb_uop`,
  `rrd_uop`, `exe_uop`), its replay-on-squash behaviour, its `ALUUnit`
  instantiation and its branch-resolution output are NOT restated and NOT
  touched.

  ===> THIS NODE ADDS NO PORT AND NO FUNCTIONAL UNIT. The whole change is an
       ADVERTISEMENT: this EU declares that it supports the `vset` uops, so that
       BOOM's existing `fu_code` / `io_ready_fu_types` matching routes them here.
       The VL/VTYPE computation is the `ALUUnit` delta; the VL-RF write and the
       `pvl` wakeup are downstream of `io_alu_resp`. If a reviewer sees a new
       `Valid(...)` IO, a second `ALUUnit`, or an extra pipeline stage in this
       file, the delta has been mis-generated.

  ===> NO NEW FUNCTIONAL-UNIT CODE EXISTS TO ADVERTISE. `FC_ALU` already covers
       it. The ScalarOpConstants delta adds `RT_VEC` and three `IQ_V_*` queues
       and leaves `FC_SZ = 10` with all ten `FC_*` codes unchanged, so inventing
       an `FC_VSET` here would both exceed `FC_SZ` and break the one-code-per-EU
       routing this file relies on.

  Governing spec anchors: frontend.rst `vector-rvv-decode` ("VSET Special
  Handling"), frontend.rst `vset-dual-dest`.

<|begin_module|>

  <|begin_parameters|>
  No new parameters. `class ALUExeUnit(val id: Int)` keeps its single `id`
  constructor parameter with its existing meaning (the ALU column index, used for
  `io_fast_wakeup.bits.speculative_mask` and `io_child_rebusy`), and `aluWidth`
  instances continue to be built by `core.scala`.

  The delta READS one existing elaboration-time value it does not declare:
  `usingRVV`, the Scala `Boolean` from `HasBoomCoreParameters` (BoomCoreParams
  delta), in scope because `ExecutionUnit` extends `BoomMultiIOModule`. It is not
  a hardware `Bool` and must not become one — with `usingRVV = false` every line
  this delta adds is ABSENT from the emitted Verilog, not tied off. Do not gate on
  rocket-chip's `usingVector`, a different switch.

  `def nReaders = 2` is unchanged and is already sufficient: `vsetvli` needs
  `rs1` and `vsetvl` needs `rs1` and `rs2`, which is exactly the two integer read
  ports this EU already requests through `HasIrfReadPorts`. Raising `nReaders`
  would add integer-RF read ports for the whole machine and is forbidden.
  <|end_parameters|>

  <|begin_ports|>
  None. The delta adds no IO to this module, in either the `usingRVV = true` or
  the `usingRVV = false` build.

  The reason is the dual-destination rule (frontend.rst `vset-dual-dest`): `rd`
  and `vl` receive the SAME value off ONE result bus, and `pvl`, `vconfig` and
  `is_vl_producer` are `MicroOp` fields (MicroOp delta). So the existing
  `io_alu_resp : Output(Valid(new ExeUnitResp(xLen)))` already carries everything
  a VL writeback needs — the value on `.bits.data` and the destination on
  `.bits.uop.pvl`. `BoomCore` taps that existing bus, qualified by
  `.bits.uop.is_vl_producer`, and presents it to the vector pipeline as
  `vec_pipeline_io.vset_resp` (declared in hierarchy.yaml as an `ExeUnitResp`,
  which is this bundle). The VL register file's `W_alu` port and the `pvl` wakeup
  are driven from there, not from here.

  Clock/reset convention (unchanged, stated for completeness): Chisel implicit
  posedge `clock` with active-high SYNCHRONOUS `reset`. This delta adds no state
  element, so it adds no reset behaviour; the existing `arb_uop` / `rrd_uop` /
  `exe_uop` registers keep their current reset semantics exactly.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. The advertisement ----

  //@req-spec-decode.c7
  Every `ALUExeUnit` instance advertises, in the `fu_types` ArrayBuffer it
  already populates, that `FC_ALU` is supported and PERMANENTLY READY:
  `fu_types += ((FC_ALU, true.B, "ALU"))`. That single entry is what discharges
  "a `vsetvli` or `vsetvl` must execute on an integer ALU execution unit":
  `VsetDecode` sets `fu_code(FC_ALU)` on the register-sourced vset uops, the ALU
  issue unit compares `uop.fu_code` against each ALU EU's `io_ready_fu_types`
  (`get_ready_fu_types()`), and the uop can therefore only be granted to an ALU
  EU. The delta's obligation here is to keep that entry unconditional and
  unqualified so a vset can never be starved of an ALU, and to state — as this
  file's contract to the rest of the design — that the ALU EU is where a
  register-sourced vset executes.

  The only textual change the advertisement needs is the descriptive label:
  under `usingRVV` the entry's string becomes "ALU+VSET", so the `toString`
  machine-configuration printout names the vset capability. The label is
  consumed only by `override def toString` / `BoomCoreStringPrefix` and reaches
  no hardware. Keep it gated anyway, so that a non-vector build's config string
  is character-identical to today's.

  Which ISSUE QUEUE a vset is dispatched to (a scalar integer queue) is
  NOT this node's claim — that is the VsetDecode / DecodeUnit iq_type
  decision. This EU is statically bound to `alu_iss_unit` by core.scala and
  has no say in the binding.

  ---- 2. What must NOT be added to the readiness term ----

  `FC_ALU`'s ready `Bool` stays the literal `true.B`. Do not introduce a
  `vset_ready` wire, a busy flag, or any vset-dependent term into it.

  Two reasons, the second load-bearing:
    - The VL RF's write ports are STATICALLY PARTITIONED per producer class and
      never arbitrated (frontend.rst, "VL-RF write ports are statically
      partitioned"), so the integer-ALU port cannot be occupied and there is
      nothing to wait for. Both the ALU and LSU VL writeback paths have no
      back-pressure line at all.
    - `io_ready_fu_types` is read by the issue unit for EVERY scalar ALU
      candidate every cycle. A vector-dependent term there would put vector
      logic on the scalar issue critical path, and would be a
      `busy`-that-gates-issue of exactly the kind the ground rules reject.

  ---- 3. The vset must not lengthen this EU ----

  The vset execution added by the `ALUUnit` delta must complete in the SAME
  combinational EXE cycle as an ordinary ALU op. `ALUUnit` drives
  `io.resp.valid := io.req.valid` with no pipeline stage, this EU forwards it as
  `io_alu_resp` with no register, and `io_fast_wakeup` is asserted three stages
  earlier — at ISS, off `io_iss_uop`, with `bypassable := true.B`. A vset needing
  an extra cycle would therefore not merely be slow: it would falsify that fast
  wakeup and the `int_bypasses` entry `core.scala` builds from `io_alu_resp`, for
  the vset's own `rd` and for whatever consumed the speculative, column-masked
  (`speculative_mask := (1 << id).U`) wakeup. This EU adds no stage and no queue;
  the `min(AVL, VLMAX)` compare must fit the existing cycle.

  The integer fast wakeup keeps its exact current condition
  (`io_iss_uop.bits.dst_rtype === RT_FIX`), which is already correct for a
  vset: with `rd == x0` decode yields RT_ZERO and no integer wakeup fires,
  while the VL RF is still written — that asymmetry is precisely why
  `is_vl_producer` exists as its own MicroOp field. Do NOT add an
  `is_vl_producer` term to `io_fast_wakeup`: the VL RF has no
  read-during-write bypass, so a `pvl` wakeup three cycles ahead of the data
  would be read stale. The `pvl` wakeup is derived downstream, from the
  writeback.

  ---- 4. The one guard outside `class ALUExeUnit` ----

  `UniqueExeUnit` also instantiates an `ALUUnit` (its `hasCSR` block, for
  `FC_CSR`). Once the `ALUUnit` delta adds the `usingRVV` vset path, that second
  instance contains it too, reachable only if a VL producer were ever granted to
  the Unique EU. Add one `usingRVV`-gated assertion in that block stating that
  it never is — `!(exe_uop.valid && exe_uop.bits.is_vl_producer)` — and export
  nothing from it. This is the only line the delta places outside
  `class ALUExeUnit`, and it exists because the vset writeback tap must come
  from the ALU EU's `io_alu_resp` alone; a silent second producer of VL values
  would write the VL RF's single ALU port from a path nobody reviews.

  ---- 5. Non-changes worth stating, because they look like changes ----

  The `RT_*` widening from `UInt(2.W)` to `UInt(3.W)` (ScalarOpConstants +
  MicroOp deltas) requires NO edit here. Every comparison in this file —
  `lrs*_rtype === RT_FIX`, the `RT_ZERO` zeroing muxes, `RT_FLT` in
  `HasFrfReadPorts`, `dst_rtype === RT_FIX` in `io_fast_wakeup` — widens on both
  sides and keeps its current result, because the four existing encodings keep
  the values 0..3. Add no `RT_VEC` term: a uop with an `RT_VEC` destination is
  dispatched to a vector issue queue and can never be granted to a scalar EU.

  No trace statement is added. The vset event is traced once, by the `ALUUnit`
  delta, which is the module holding the computed VL and VTYPE.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
No performance target of its own, and that is the constraint: this delta must be
performance-NEUTRAL and timing-neutral.

- Latency: unchanged. ARB -> RRD -> EXE, `ALUUnit` combinational,
  `io_alu_resp` unregistered. A vset completes in the same cycle count as an
  `addi`, and issue-to-writeback for every existing ALU op is untouched.
- Throughput: unchanged. `aluWidth` ALU ops per cycle; a vset consumes one ALU
  slot for one cycle like any integer op and needs no serialization here
  (`vsetvl`'s serialization is `is_unique` + `flush_on_commit` at decode/ROB, not
  an EU stall).
- Frequency: zero added gates. `io_ready_fu_types` must not gain a level of
  logic, since it feeds issue-stage select.
- With `usingRVV = false`: RTL bit-identical to pre-Caracal BOOM v4. Only
  assertions and one printout label are gated, so this is one of the cheapest
  files to check gate (f) on.
<|end_perf|>

<|begin_dependencies|>
`ScalarOpConstants` (src/main/nlhdl/pkg/ScalarOpConstants.nlhdl.scala) — for the
`FC_ALU` code this EU advertises and the widened `RT_*` encodings the traits in
this file compare against. Note what it does NOT supply: no new `FC_*` code and
no change to `FC_SZ`, so `get_all_fu_types()` / `get_ready_fu_types()` keep their
`Vec(FC_SZ, Bool())` shape.

Reads `usingRVV` (BoomCoreParams delta) and `MicroOp.is_vl_producer` / `pvl`
(MicroOp delta) — the latter only as fields of the uop already riding
`io_alu_resp`, never as new ports.

Instantiates nothing new. `val alu = Module(new ALUUnit(dataWidth = xLen))`
already exists; the `ALUUnit` delta changes what that instance computes and this
file does not re-describe it.

Consumers of this node's advertisement: `core.scala`'s
`alu_iss_unit.io.fu_types := alu_exe_units.map(_.io_ready_fu_types)` (routing),
and `core.scala`'s per-ALU writeback loop, which the BoomCore delta extends to
tap `io_alu_resp` into `vec_pipeline_io.vset_resp`.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File  src/main/scala/v4/exu/execution-units/execution-unit.scala
    Class `class ALUExeUnit(val id: Int)(implicit p: Parameters)
           extends ExecutionUnit("Alu")` (package boom.v4.exu).
    Plus ONE gated assertion inside `class UniqueExeUnit`'s `hasCSR` block in
    the same file (see In scope, item 3). Hand-written baseline BOOM v4.

  In scope:
    1. The `fu_types += ((FC_ALU, true.B, "ALU"))` line in `ALUExeUnit`: the
       `usingRVV`-gated descriptive label only. The code (`FC_ALU`) and the
       ready term (`true.B`) stay exactly as they are.
    2. `usingRVV`-gated assertions and comments inside `ALUExeUnit` documenting
       that a VL producer granted to this EU carries `fu_code(FC_ALU)`.
    3. One `usingRVV`-gated assertion in `UniqueExeUnit`'s `hasCSR` block that
       its `ALUUnit` instance never sees `is_vl_producer`.
    Nothing else in the file may be edited. Total added lines: ~10.

  Must not regress:
    - `io_alu_resp` keeps its exact declaration `Output(Valid(new
      ExeUnitResp(xLen)))` and its exact valid condition
      `io_alu_resp.valid := alu.io.resp.valid`. It is not gated on
      `dst_rtype`, not gated on `is_vl_producer`, and not registered.
    - `io_fast_wakeup` keeps `io_iss_uop.valid && dst_rtype === RT_FIX`,
      `speculative_mask := (1 << id).U`, `rebusy := false.B`,
      `bypassable := true.B`; `io_fast_pred_wakeup` keeps its `is_sfb_br`
      condition. No `is_vl_producer` or `pvl` term is added to either.
    - `io_ready_fu_types := get_ready_fu_types()` is unchanged, and
      `get_all_fu_types()` / `get_ready_fu_types()` in `ExecutionUnit` are not
      touched. `FC_ALU`'s ready term stays the literal `true.B`.
    - `io_squash_iss`, `io_child_rebusy` and the `when (io_squash_iss ||
      arb_rebusied)` replay block keep their current bodies: same
      `will_replay`, same clearing of `iw_p1_bypass_hint` / `iw_p2_bypass_hint`,
      same `rrd_uop.valid := false.B`. A vset replays like any ALU op.
    - `alu.io.req.valid := exe_uop.valid && exe_uop.bits.fu_code(FC_ALU)` and
      the whole `exe_int_req` construction (`rs1_data`, `rs2_data`, `imm_data`,
      `pred_data`, `ftq_info`) are unchanged; `rs3_data` stays `DontCare`.
    - `io_brinfo := alu.io.brinfo` is unchanged; branch resolution latency and
      the `HasBrfReadPort` / `HasFtqReadPort` / `HasPrfReadPort` /
      `HasImmrfReadPort` / `HasIrfReadPorts` trait bodies are untouched. In
      particular no `RT_VEC` term is added to any of them.
    - `def nReaders = 2` and the resulting integer-RF read-port count are
      unchanged.
    - `MemExeUnit`, `FPExeUnit`, `class Wakeup`, `class ExeUnitResp`,
      `class MemGen`, `class CSRResp` and `abstract class ExecutionUnit` are not
      modified at all. `UniqueExeUnit` is touched by exactly one assertion and
      keeps every existing connection, including its `io_csr_resp` / `io_sfence`
      / `io_mul_resp` / `io_ifpu_resp` / `io_div_resp` behaviour and timing.
    - With `usingRVV = false` the elaborated RTL is bit-identical to the current
      file's for every EU class, and the `toString` config string is
      character-identical.
    - Existing copyright header, comment style and declaration order preserved.
      No reformatting.

  Interface delta:
    NEW ports:      none.
    WIDENED ports:  none.
    NEW parameters: none.
    CHANGED types:  none.

    Reject list — if any of these appears, the edit is wrong:
      - a new `io_vset_wb` / `io_vl_resp` / `VsetWbResp`-style output port (that
        was the M1 shape; v2 carries the VL writeback on the existing
        `ExeUnitResp` because `pvl` and `vconfig` are MicroOp fields);
      - a new `io_vl_wakeup` output (the `pvl` wakeup is derived downstream of
        the tapped writeback, not here);
      - a second `ALUUnit`, any other `Module(...)`, or a vector functional unit
        in this file;
      - a new `FC_*` code, a change to `FC_SZ`, or an `id`-conditional
        advertisement that lets only some ALU EUs accept a vset;
      - a vset-dependent term in `io_ready_fu_types` or in `io_squash_iss`;
      - any added pipeline register, `RegNext`, or queue on the `io_alu_resp`
        path;
      - a change to `nReaders`, or a new integer/FP register-file read or write
        port;
      - an `RT_VEC` comparison anywhere in this file.
<|end_edit_scope|>

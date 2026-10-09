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
  ScalarOpConstants — DELTA SPEC. Describes only the constants Caracal ADDS to
  the existing `trait ScalarOpConstants` in
  src/main/scala/v4/common/consts.scala, which is hand-written baseline BOOM v4.
*/

  hierarchy.yaml: kind: package, mode: edit_existing,
  target src/main/scala/v4/common/consts.scala. No `output:`. Budget: ~30 lines.

  This is the smallest delta in the map: one register-type encoding and three
  issue-queue identifiers. It is listed as its own node anyway because both
  changes WIDEN a field that baseline code already compares against, so the
  blast radius is much larger than the line count suggests.

  Nothing else in `consts.scala` is touched — not `trait RISCVConstants`, not
  `trait ExcCauseConstants`, and not the branch, immediate, operand-select or
  functional-unit constants in `ScalarOpConstants` itself.

  Governing spec anchor: overview.rst, the pipeline table's rename/dispatch rows.

<|begin_module|>

  <|begin_parameters|>
  No parameters. `trait ScalarOpConstants` is a plain mixin of Scala `val`s with
  no implicit `Parameters` in scope, and this delta does not add any — every
  value below is a literal encoding, not a derived width.

  Note that these constants are therefore NOT gated on `usingRVV`: a trait of
  constants has no configuration to read. The encodings simply exist; whether
  anything ever uses `RT_VEC` or an `IQ_V_*` queue is decided by the configs and
  by the `usingRVV`-gated logic elsewhere. This is safe, and gate (f) still
  holds, only because both changes are additive in the sense described below.
  <|end_parameters|>

  <|begin_ports|>
  Not applicable. This is a Scala trait of constants: no I/O, no clock, no reset,
  no hardware.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. The third register class ----

  //@req-spec-core.e8
  Add a third architectural register class, `RT_VEC`, to the decode-stage
  register-type encoding, with the value 4. The existing block reads:

    RT_FIX = 0, RT_FLT = 1, RT_X = 2, RT_ZERO = 3, each `UInt(2.W)`

  Adding a fifth value does not fit in 2 bits, so the whole group widens from
  `UInt(2.W)` to `UInt(3.W)` and `RT_VEC = 4.U(3.W)` joins it.

  ===> KEEP THE EXISTING FOUR VALUES AT 0, 1, 2 AND 3. The widening must be
       value-preserving: every existing comparison against `RT_FIX`, `RT_FLT`,
       `RT_X` or `RT_ZERO` must yield the same result it does today. Renumbering
       even one of them would silently change decode behaviour for scalar code,
       and it would do so in a way no vector test could ever detect.

  The matching widening of `dst_rtype`, `lrs1_rtype` and `lrs2_rtype` in the
  MicroOp bundle, and of the ROB entry's `dst_rtype`, are the MicroOp and Rob
  deltas respectively. All three widths must move together: this trait is
  where the encoding lives, but it declares no storage.

  ---- 2. The three vector issue queues ----

  Append `IQ_V_LOAD`, `IQ_V_STORE` and `IQ_V_ALU` to the issue-queue identifier
  space, and raise `IQ_SZ` from 4 to 7. The existing block reads:

    IQ_SZ = 4; IQ_MEM = 0, IQ_UNQ = 1, IQ_ALU = 2, IQ_FP = 3

  so the new identifiers take 4, 5 and 6 and APPEND rather than insert.

  ===> APPEND, NEVER INSERT, and keep IQ_MEM/IQ_UNQ/IQ_ALU/IQ_FP at 0..3. These
       identifiers index `MicroOp.iq_type`, which is a `Vec(IQ_SZ, Bool())`, so
       they are bit POSITIONS in a one-hot-per-queue vector. Inserting a value
       would renumber the existing queues and mis-route every scalar uop, and
       because dispatch routes purely on those bits the failure would look like
       a scheduling bug rather than a constant-numbering bug.

  Raising IQ_SZ widens MicroOp.iq_type by three bits for every uop in the
  machine, including in configs that never enable vectors. That is the one
  place this delta is NOT free, and it is accepted deliberately: gating IQ_SZ
  on usingRVV would make the trait depend on Parameters, which it does not
  today.

  ===> IT IS NOT MERELY AN AREA COST. THE THREE NEW POSITIONS MUST BE
       EXPLICITLY DEFAULTED, AND NOT BY THIS FILE. Baseline `DecodeUnit`
       assigns `uop := io.enq.uop` and then writes only iq_type positions 0..3
       individually, while `io.enq.uop` originates from a bundle the frontend
       sets with `f2_fetch_bundle := DontCare`. So without an explicit default
       every SCALAR uop would carry three DON'T-CARE vector-queue routing bits
       into dispatch — mis-routing, not just an X in a waveform. The same
       hazard applies to `is_vec`, `is_shared` and `is_vl_producer`.
       The fix lives in the DecodeUnit delta (six default assignments), which
       is the only node that sees `io.enq.uop`. This trait must NOT attempt it:
       a constants trait has no uop to default.
       Gate (f) remains the bit-identity check, but note that a PASSING gate
       (f) does NOT prove the defaults are present — a don't-care bit can
       elaborate identically and still mis-route in simulation.

  No functional-unit code is added here. `FC_AGEN` and `FC_DGEN` already exist
  and the vector paths reuse them; `FC_SZ` stays 10.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
No behaviour and no timing target. One constraint: every value here must remain
a Scala-elaboration-time constant so that the emitted comparisons are constant
folded exactly as they are today. Nothing in this trait may become a hardware
signal.
<|end_perf|>

<|begin_dependencies|>
None. `trait ScalarOpConstants` sits at the bottom of the declaration graph and
depends on nothing — it must stay that way, since `MicroOp` mixes it in and a
dependency here would create a cycle.

Its dependents are MicroOp, Rob, DecodeUnit, ALUUnit, ALUExeUnit,
BoomConfigMixins and BoomCore. The `IQ_SZ` change reaches every one of them.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File  src/main/scala/v4/common/consts.scala
    Trait `trait ScalarOpConstants` (package boom.v4.common.constants).
    Hand-written baseline BOOM v4.

  In scope:
    - The "Decode Stage Control Signals" block: widening RT_FIX/RT_FLT/RT_X/
      RT_ZERO from `UInt(2.W)` to `UInt(3.W)` and adding `RT_VEC = 4.U(3.W)`.
    - The "IQT type" block: raising `IQ_SZ` from 4 to 7 and adding `IQ_V_LOAD`,
      `IQ_V_STORE`, `IQ_V_ALU` as 4, 5, 6.

  Must not regress:
    - RT_FIX = 0, RT_FLT = 1, RT_X = 2, RT_ZERO = 3 keep those exact values.
    - IQ_MEM = 0, IQ_UNQ = 1, IQ_ALU = 2, IQ_FP = 3 keep those exact values.
    - Every other constant in the trait is untouched: X/Y/N, the BSRC_*, CFI_*,
      PC_*, B_*, OP1_*, OP2_*, REN_*, SZ_DW/DW_*, MEN_*, IS_* groups, `FC_SZ`
      and all ten `FC_*` codes, and `def NullMicroOp`.
    - `trait RISCVConstants` and `trait ExcCauseConstants` in the same file are
      not touched at all. In particular `MINI_EXCEPTION_MEM_ORDERING` and
      `MINI_EXCEPTION_CSR_REPLAY` keep their values 16 and 17 and their
      `require`s.
    - The file's copyright header, comment style and declaration order are
      preserved. New values are appended within their existing block, not
      interleaved.

  Interface delta:
    NEW:      RT_VEC = 4.U(3.W); IQ_V_LOAD = 4; IQ_V_STORE = 5; IQ_V_ALU = 6
    WIDENED:  RT_FIX, RT_FLT, RT_X, RT_ZERO : UInt(2.W) -> UInt(3.W)
    CHANGED:  IQ_SZ : 4 -> 7
    Nothing else.
<|end_edit_scope|>

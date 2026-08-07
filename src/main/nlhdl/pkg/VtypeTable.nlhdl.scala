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
  VtypeTable — the one place that turns a `vtype` value into {VLMAX, EMUL,
  vill}, and an AVL into a VL.

  hierarchy.yaml: kind: package, mode: new,
  output src/main/scala/v4/vec/generated/VtypeTable.scala,
  package boom.v4.vec.generated. depends_on VectorParams.

  PACKAGE NODE CONVENTION. A `kind: package` emitting a Scala `object
  VtypeTable` of pure combinational functions, plus one small Bundle. No I/O,
  no state, no instance. The parameters section lists what the functions are
  parameterized on, the ports section is explicitly empty, and the logic section
  gives the functions.

  WHY IT IS A PACKAGE AND NOT A MODULE. Its callers evaluate it a different
  number of times each: VConfigUnit once per decode lane, VsetDecode once per
  lane for the immediate forms, and ALUUnit once at execute for the
  register-sourced forms. Expressing a pure function as a Module would force
  `coreWidth + 1` instantiations of something with no state, and — worse —
  would let the three callers drift apart. Three copies of a `vill` rule is
  three chances to disagree about when a configuration is legal.

  ===> IT WRAPPS ROCKET-CHIP'S `VType`; IT DOES NOT REIMPLEMENT IT.
       `freechips.rocketchip.rocket.VType` already provides `lmul_ok`,
       `max_vsew`, `vlMax` and a full-width `vl(...)`, and rocket's `CSRFile`
       is the owner of architectural `vtype` under `usingVector`. If this file
       computed VLMAX its own way, the host and the architectural CSR could
       disagree about whether a configuration is legal — the mirror would admit
       a `vtype` the CSR calls `vill`, and a mis-sized PRN group would be
       allocated for it. Bind to rocket's definitions; add only what Caracal
       needs on top.

  Governing spec anchor: frontend.rst `vector-rvv-decode`, plus the VSET
  handling section for the VL computation.
*/

<|begin_module|>

  <|begin_parameters|>
  No constructor parameters of its own. Every function here is parameterized
  implicitly, through Chisel's `Parameters`, on the machine sizes it needs:
  `vLen` and `eLen` from VectorParams (and hence from rocket's
  `HasCoreParameters`, which is where `VType.max_vsew` and `VType.vlMax` read
  them from), and `maxMembers` from VectorParams for the group-size bound.

  //@req-spec-decode.d10
  The functions are combinational and side-effect-free, which is what lets a
  caller evaluate them once per decode lane at the machine's NxWide decoder
  width. There is no per-lane state here and no arbitration between lanes: N
  evaluations of a pure function are N independent copies of a small comparator
  tree, and each decode lane gets its own. Nothing in this file may introduce
  state, because that would make the lane count load-bearing.
  <|end_parameters|>

  <|begin_ports|>
  None. This is a declaration unit: no I/O, no clock, no reset.
  <|end_ports|>

  <|begin_logic|>
  ---- The bundle ----

  `VtypeInfo` is a small Bundle bundling the decoded results of one `vtype`:
  `vlmax` (width `vecVLSz`), `emul` (width `log2Ceil(maxMembers) + 1`, i.e. the
  group's member count as a 1..8 value), `vill`, plus the `vta` and `vma`
  policy bits passed straight through. It exists so a caller takes one decode
  result rather than four parallel functions that could be called
  inconsistently.

  ---- Legality: three ways a vtype is illegal ----

  //@req-spec-decode.a11
  The supported LMUL settings are 1/8, 1/4, 1/2, 1, 2, 4 and 8, encoded as
  rocket's signed `vlmul` field (`vlmul_sign` with `vlmul_mag`) — but only
  subject to VLMAX >= 1. A fractional LMUL small enough that
  `VLEN * LMUL / SEW` would round to zero elements names no registers at all
  and is not a legal configuration.

  //@req-spec-decode.a12
  //@req-spec-decode.a13
  //@req-spec-decode.a14
  Do NOT hand-write those three checks. Obtain all of them from
  `VType.fromUInt(bits)`, whose returned `vill` is already the disjunction of
  exactly the three conditions Caracal needs:
    - `!lmul_ok` — this is the VLMAX >= 1 constraint. For fractional LMUL,
      rocket's `lmul_ok` is `vlmul_mag =/= 0 && ~vlmul_mag < max_vsew - vsew`,
      which is false precisely when VLEN*LMUL/SEW < 1.
    - `max_vsew < vsew` — SEW greater than ELEN, since
      `max_vsew = log2Ceil(eLen/8)`.
    - `reserved =/= 0`.
  // The reserved vlmul=3'b100 encoding falls out of the same expression rather
  // than needing a case of its own: 3'b100 is vlmul_sign=1 with vlmul_mag=0,
  // so lmul_ok's `vlmul_mag =/= 0` term is false and vill is set. Do not add a
  // separate comparison against 3'b100 — a redundant check that disagrees with
  // rocket's is worse than no check.

  Expose this as `decode(bits: UInt): VtypeInfo`, which calls
  `VType.fromUInt(bits)` and fills `vlmax` from that `VType`'s `vlMax`, `vill`
  from its `vill`, and `vta`/`vma` from its fields. `vlmax` is meaningful only
  when `vill` is clear; when `vill` is set, drive `vlmax` and `emul` to zero so
  a consumer that ignores `vill` gets an obviously-wrong answer rather than a
  plausible one.

  ---- EMUL: the group size a rename must allocate ----

  `emul(info, eew)` returns the number of registers in a group whose elements
  are `eew` bits wide, as a 1..`maxMembers` count. For an arithmetic op the
  element width is SEW and EMUL is LMUL; for a load or store the element width
  is the instruction's EEW and EMUL is `LMUL * EEW / SEW`. Clamp the result to
  at least 1 — a fractional EMUL still occupies one whole register — and assert
  it never exceeds `maxMembers`, since a legal `vtype` cannot produce a group
  wider than 8 and a wider one would silently truncate the rename allocation.

  // This is the value that reaches the vector mapper as v_emul and decides how
  // many PRNs an OP.v allocates atomically. An EMUL that disagrees with the
  // one the VPU derives from the same issue packet would corrupt member
  // indexing on the CII, so it is derived from vtype+eew only — never from a
  // separate decode path.

  ---- VL from AVL ----

  //@req-spec-decode.c2
  `computeVL(avl, bits, currentVL, useCurrentVL, useMax, useZero)` returns the
  new VL. For the immediate form (`vsetivli`) this is `min(uimm, VLMAX)`. Do
  not write that `min` by hand either: delegate to rocket's
  `VType.computeVL`/`VType.vl`, which already implements the RVV rules
  including the `rs1 = x0` max and keep-current cases the register-sourced
  forms need.

  // ===> COMPARE THE FULL-WIDTH AVL. This is where an earlier implementation
  // truncated AVL to vecVLSz+1 bits before comparing against VLMAX, so a large
  // AVL WRAPPED instead of saturating and AVL=2048 produced vl=0. That breaks
  // the canonical strip-mining idiom, where AVL is the whole remaining element
  // count and is expected to saturate to VLMAX on every iteration but the
  // last. Rocket's `vl(...)` handles this correctly by testing the high bits
  // separately (`atLeastMaxVLMax`) instead of truncating, which is the second
  // reason to delegate rather than reimplement.

  A VL of zero is a legal, reachable result and not an error: consumers handle
  it (see VecGroupCopy for the destination-group consequence). This function
  must not special-case it.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Every function here is single-cycle combinational and lands in the DECODE stage
critical path, replicated `coreWidth` times. Keep each one a shallow comparator
and shifter tree — no multipliers, no dividers, no iteration. `VLMAX` and
`EMUL` are computed by shifting `vLen` by SEW/LMUL/EEW exponents, never by
dividing; rocket's `vlMax` is already written this way, which is a third reason
to use it as-is.

The `emul` result feeds atomic group rename, which is the design's top timing
risk (up to `coreWidth * 8` PRN allocations per cycle). This file must not
contribute depth to it: `emul` is available in the same cycle its `vtype`
snapshot is, with no pipeline stage.
<|end_perf|>

<|begin_dependencies|>
VectorParams — for `vLen`, `eLen`, `maxMembers` and `vecVLSz`.

Also binds, deliberately and by name, to two rocket-chip declarations that are
NOT nodes in this map because they are upstream and unmodified:
`freechips.rocketchip.rocket.VType` (for `fromUInt`, `lmul_ok`, `max_vsew`,
`vlMax`, `vl`) and `freechips.rocketchip.rocket.VConfig`. Rocket's `CSRFile`
owns architectural `vtype` under `usingVector`, so these types are the contract
between Caracal's speculative mirror and the architectural state — binding to
them is what keeps the two from disagreeing about legality.

Instantiates nothing. It is a package: callers reach it through `depends_on:`.
Its dependents are VConfigUnit, VsetDecode, VDecode and ALUUnit.
<|end_dependencies|>

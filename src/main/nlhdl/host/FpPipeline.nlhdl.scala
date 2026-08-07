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
  FpPipeline — DELTA SPEC. Not a description of BOOM's FP pipeline: it describes
  only what Caracal ADDS to the existing `class FpPipeline` in
  src/main/scala/v4/exu/fp-pipeline.scala, hand-written baseline BOOM v4 that stays
  in place.

  hierarchy.yaml: kind: module, mode: edit_existing, target
  src/main/scala/v4/exu/fp-pipeline.scala (package boom.v4.exu). No `output:` — the
  pre-existing file is the artifact. depends_on MicroOp. Budget: ~50 added lines
  (plan §11).

  The delta is three things: ONE added FP register-file READ PORT; ONE added
  DEDICATED FP register-file WRITE PORT plus the wakeup slot that goes with it
  (D7, and see the amended reject list — an earlier revision of this file REFUSED
  this port); and the WAKEUP TAP contract on the FP network that a `.vf` scalar
  feeder of a vector slot wakes on. Everything else — the `FPExeUnit` instances,
  `issue_unit`, `fregfile`'s existing read/write ports and their arbitration and
  assignment order, `ll_wbarb`, `fp_bypasses`, `io.to_int`, `io.dgen`, the FMA and
  divSqrt latencies and the `fflags` payload of every existing slot — is UNCHANGED
  and not restated here. `io.wakeups`/`io.wb` gain ONE entry under `usingRVV` and
  are otherwise untouched, entry for entry.

  ===> NO SECOND WAKEUP NETWORK IS CREATED FOR `.vf`. Vector ops ride the FP
       wakeup network that already exists; the "tap" is the EXISTING `io.wakeups`
       port, fanned out to one more consumer by the BoomCore delta. This delta adds
       no wakeup logic and no register on the wakeup path. The one thing a
       duplicated wakeup net cannot be is one cycle late, which is exactly what a
       re-registered copy would be.
       WHAT D7 DOES ADD IS A PRODUCER ON THAT SAME NET, NOT A NET: one slot appended
       last to `fp_wakeups`/`io.wb`, so `numWakeupPorts` does gain one under
       `usingRVV`. An earlier revision of this file said `numWakeupPorts` was
       unchanged; that was true of the read-port-only delta and is no longer true.
       Every existing slot keeps its index and its timing.

  ===> THE ADDED READ PORT IS NOT A BYPASSED REGISTER-FILE READ, AND SAYING SO IS
       THE POINT. `fregfile` bottoms out in `FullyPortedRF`, a `Mem` read of a
       `RegNext`-ed address with NO read-during-write forwarding; the FP pipeline
       covers that gap for its own consumers with `fp_bypasses`, which is wired
       only into `unit.io_rrd_frf_bypasses` and is not exported. So this delta
       builds the response-cycle forward LOCALLY, on the added port only. Leaving
       that unstated is the M1 stale-operand bug in the FP direction.

  ===> AND THERE IS NO RESIDUAL FAST-WAKEUP WINDOW LEFT TO CLOSE (A30, CLOSED BY
       D4). An earlier revision of this file devoted part 3 of its logic section to
       a one-cycle window that no response-cycle forward could close, and handed the
       fix to the consumer. That analysis assumed the added read port's consumer was
       `VecScalarOperandRead`'s store instance, which drives its address
       combinationally in the cycle it is granted. D4 DELETED that consumer: no RVV
       store form takes an FP scalar operand (store data is always `vs3`; the scalar
       operands are `rs1`/`rs2`, both integer), so `vec_pipeline_io.fp_rf_read_req`
       is one lane whose only reader is `VecCiiIssue`. `IQ_V_ALU` is PAST-PNR GATED,
       so a granted CII op is OLDER THAN THE PNR and its scalar-FP producer has
       genuinely written back — there is no fast-wakeup lead to outrun and no
       consumer-side obligation. The response-cycle forward below is retained
       anyway, because it is cheap and because "the producer retired" is an argument
       about the common case while the forward is a proof about every case.

  Governing spec anchors: overview.rst `boom-relationship` ("FP execution —
  Unchanged"), midcore.rst `spec-wakeups` ("the `.vf` scalar on the FP network").
*/

<|begin_module|>

  <|begin_parameters|>
  No new module parameter and no new `BoomCoreParams` field. The delta re-derives
  five existing elaboration-time values inside `class FpPipeline`, each gated on
  `usingRVV` (a Scala `Boolean` from `HasBoomCoreParameters`, never a hardware
  `Bool`):

  - Add `numFrfExeReadPorts = fpWidth * 3` — the share of logical read ports the
    `FPExeUnit`s own. This is the value the existing `fpWidth * 3` expression
    already computes; naming it is what lets the two `require(rd_idx == ...)`
    checks in the arbitrate and read stages keep their current meaning once a
    non-exe-unit port exists.
  - Change `numFrfLogicalReadPorts` to `numFrfExeReadPorts + (if (usingRVV) 1
    else 0)`. The added logical port is the LAST index, `numFrfExeReadPorts`.
  - Pass `numFrfReadPorts + (if (usingRVV) 1 else 0)` as `BankedRF`'s
    `numPhysicalReadPorts` argument. `numFrfReadPorts` itself is not redefined —
    the increment is at the instantiation site.
  - Change `numFrfWritePorts` from `fpWidth + lsuWidth` to `fpWidth + lsuWidth +
    (if (usingRVV) 1 else 0)` (D7). The added write port is the LAST index, so the
    existing `w_cnt` assignment order is untouched and `require(w_cnt ==
    fregfile.io.write_ports.length)` still holds with the loop bodies unchanged.
    `fregfileBankedWriteArray = Seq.fill(numFrfWritePorts){None}` follows the new
    count with no edit of its own.
  - Change `numWakeupPorts` from `fpIssueParams.issueWidth + numLlPorts` to that
    plus `(if (usingRVV) 1 else 0)` (D7). The added slot is the LAST index, so
    `require(idx == numWakeupPorts)` holds with the existing loop bodies unchanged,
    and `issue_unit.io.wakeup_ports := fp_wakeups` picks the extra slot up with no
    edit — which is required, not incidental: a scalar FP consumer of a `vfmv.f.s`
    result must be able to wake.

  // D7 states the change as "`numFrfWritePorts` 1 -> 2". The ACTUAL baseline
  // expression is `fpWidth + lsuWidth`, which is 2 on Medium/Large and 3 on Mega,
  // so the delta is "+1 under `usingRVV`" and D7's literal numbers are the count of
  // added-versus-none rather than the parameter's value. Recorded because a
  // generator that hard-coded `2` would silently drop the Mega tier's second
  // exe-unit write port.

  Unchanged: `numLlPorts`, `fpPregSz`, `numFrfBanks`, `fpIssueParams` and
  `dispatchWidth`.

  ---- Why a PHYSICAL port is added and not just a logical one ----

  `PartiallyPortedRF` grants physical ports to logical ports in ASCENDING LOGICAL
  INDEX ORDER and denies one by dropping its `ready`
  (`arb_read_reqs(i).ready := PopCount(arb_read_reqs.take(i).map(_.valid)) <
  numPhysicalReadPorts`). The consumer of the added port has no `ready` line at
  all — `vec_pipeline_io` declares `fp_rf_read_req`/`fp_rf_read_rsp` and nothing
  else — so a denial would be silent and would return another register's data.
  One added physical port alongside the one added logical port makes the added
  port UNDENIABLE whenever `numFrfReadPorts >= numFrfExeReadPorts`; state that as
  an elaboration `require` under `usingRVV`:
  `require(numFrfReadPorts + 1 >= numFrfLogicalReadPorts)`.

  It holds for every tier in the vector matrix, which after D3 is MEDIUM/LARGE/MEGA
  — SmallBoom is no longer a vector config at all, so its numbers are no longer
  load-bearing here. Medium/Large have `fpWidth = 1`, `numFrfReadPorts = 3`; Mega
  has `fpWidth = 2`, `numFrfReadPorts = 6`.

  // A32, reduced to a note (D4 made it moot — the shortfall only arose from needing
  // TWO read lanes, and Giga is not in the vector matrix): if a Giga vector config
  // is ever added, `numFrfReadPorts` must rise to 6 in its config mixin. Giga
  // deliberately under-ports its FP file (`fpWidth = 2`, `numFrfReadPorts = 4`),
  // and the `require` above fails loudly at elaboration, which is the whole reason
  // it exists.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are unchanged and implicit, as for any `BoomModule`: posedge
  `clock`, ACTIVE-HIGH SYNCHRONOUS `reset`. The delta adds exactly one
  reset-sensitive element, the address register described in the logic section,
  and it needs no reset value (its output is only ever consumed alongside a
  consumer-side valid). The added write port and wakeup slot hold no state of their
  own — they are driven combinationally from `io.vec_fp_wb`.

  ---- Added to `io` (all `usingRVV`-gated, absent otherwise) ----

  `vec_frf_read_req` — `Input(UInt(maxPregSz.W))`. The FP physical register
  number the vector pipeline wants read. This is the seam's `fp_rf_read_req`
  (hierarchy.yaml `vec_pipeline_io`, width `pregSz`), and BoomCore's delta wires
  it straight through from `vec_pipeline.io.fp_rf_read_req`.

  `vec_frf_read_rsp` — `Output(UInt(xLen.W))`. The register's value in
  ARCHITECTURAL IEEE-754 encoding, valid in the cycle AFTER the address, on the
  same address-now / data-next-cycle discipline as every other FP read port.

  ===> EXACTLY ONE READ PORT, BECAUSE EXACTLY ONE CONSUMER EXISTS — AND SINCE D4
       THAT CONSUMER IS `VecCiiIssue`, NOT A STORE. The single driver is
       `VecCiiIssue`'s scalar-FP capture for `.vf` arithmetic (`vfmul.vf` and
       friends): the coprocessor can address no register file, so the host must
       read the value and deliver it BY VALUE across the CII, and in BOOM the
       bypass network sits ON TOP OF a register-file read rather than replacing
       one. `IQ_V_ALU` is past-PNR gated, so a granted CII op's scalar producer
       has usually retired and left the bypass network entirely — capturing from
       the bypass alone would read a stale or unrelated value with no error, which
       is why the port exists at all.
       NEITHER `VecScalarOperandRead` INSTANCE READS THIS PORT. An earlier revision
       named `st_opnd` as the sole driver and said the CII path did not use the port;
       that is inverted, and D4 is why: `execution.rst` cited `vfmul.vf` as a store
       DGEN reading an FP scalar, but `vfmul.vf` is vector-scalar FP ARITHMETIC and
       no RVV store form has an FP scalar operand at all (store data is always
       `vs3`; the scalar operands are `rs1`/`rs2`, both integer). `IQ_V_LOAD`/
       `IQ_V_STORE` slots have no FP wakeup connection and no FP reader.
       Since the seam has a single driver, NO arbitration is added on it, and a
       second port would be a port with no reader.

  `vec_fp_wb` — `Input(Valid(new ExeUnitResp(xLen)))` (D7). The vector pipeline's
  SCALAR-FP DESTINATION writeback: `vfmv.f.s`, and any CII op whose destination is
  an FP scalar register. This is the seam's `fp_wb` (`vec_pipeline_io`), driven by
  `VecCiiWriteback` through `VecPipeline` and wired here by BoomCore's delta. It is
  fire-and-forget — no `ready`, because the CII Writeback channel cannot be
  back-pressured — and `bits.data` is in ARCHITECTURAL IEEE-754 encoding, `xLen`
  wide, with `bits.fflags` already accumulated by `VecCiiWriteback` and
  `bits.uop.pdst`/`dst_rtype` naming the renamed scalar FP destination. The recode
  into hardfloat is this module's, per the seam's stated convention that data
  crosses in the architectural encoding.

  ---- Deliberately NOT added (a reviewer's reject list) ----

  No new wakeup NETWORK and no vector-only wakeup port — the tap for `.vf` READINESS
  is the existing `io.wakeups`, and the one slot D7 appends to it is a producer on
  that same net, not a second net (see the amendment below). No `fp_wb_snoop` output:
  `io.wb` already carries `{uop.pdst, data}` per wakeup port in the same cycle as
  the corresponding `fregfile.io.write_ports` write, so it IS the FP analogue of
  the `int_wb_snoop` the INT side had to add, and this delta consumes it internally
  instead of exporting a second copy. No `valid` on `vec_frf_read_req`, no `ready`
  on `vec_frf_read_req`/`vec_frf_read_rsp`, and no `ready` on `vec_fp_wb`. No vector
  register file port of any number: the `vrf-ports` partition in midcore.rst is
  untouched, this delta ports the SCALAR FP file only. No second FP READ port (D4
  reduced the seam to one lane). No change to `dfmaLatency`, `fastWakeupLatency`, the
  divSqrt latency, the `ll_wbarb` input order or any existing slot's `fflags`.

  ---- ⇒ THE REJECT LIST IS AMENDED (D7). READ THIS BEFORE REVIEWING THE DIFF. ----

  This list PREVIOUSLY REFUSED the thing the delta now adds. Its exact wording was:
  "No FP register-file WRITE port and no added wakeup slot for the vector
  scalar-destination writeback (`vfmv.f.s`, `vec_pipeline_io`'s `fp_wb`): no
  requirement in this node's allocation covers it, and adding one would change
  `numFrfWritePorts` and `numWakeupPorts` — flagged upward, not silently absorbed."
  The flag was raised (A31), it was decided (D7), and the port IS NOW IN SCOPE. This
  is a DELIBERATE WIDENING of an `edit_existing` node's allowed change, recorded in
  the text where the refusal was — plan §10 names an `edit_existing` node quietly
  exceeding its scope as the previous attempt's worst bug site, and the defence
  against that is an amendment a reviewer can see, not a diff that merely does not
  match the list.

  WHY A DEDICATED PORT AND NOT AN ARBITRATED SHARE — a CORRECTNESS argument, not a
  cost one. The natural cheap option is a 4th input on `ll_wbarb`, which is a plain
  `Arbiter` (in(0) = mem, in(1) = ifpu, in(2) = fdiv) feeding
  `fregfile.io.write_ports(0)`. An `Arbiter` input CAN BE DENIED, and the CII
  writeback channel has NO BACK-PRESSURE — `VecCiiWriteback` drives `fp_wb` as a
  fire-and-forget `Valid`, so a denied beat is a LOST WRITE, not a delayed one. No
  bound is constructible either: the arbiter can lose to mem *and* ifpu *and* fdiv,
  and consecutive scalar-FP CII ops can produce back-to-back beats, so there is no
  N for which "denied at most N cycles" is provable. `vfmv.f.s` BEING RARE IS
  EXACTLY WHAT MAKES ARBITRATION DANGEROUS: the failure mode is a once-in-a-blue-moon
  wrong value in an FP register with no assertion anywhere, on a path no test
  exercises densely. And a second, independent reason: everything joining `ll_wbarb`
  is in HARDFLOAT RECODED form (`ll_wbarb.io.in(0).bits.data := recode(...)`,
  `io.from_int` and `io_fdiv_resp` both recoded), while `VecCiiWriteback` emits IEEE
  — so an `ll_wbarb` input would need a recode in front of the arbiter, on the arbiter's
  own critical path, to buy nothing.

  It is `usingRVV`-GATED, so it does not break gate (f): with vectors off the port,
  the slot and the two parameter increments are ABSENT, and the change lands inside
  D1's ENUMERATED EXCEPTION rather than widening it. It stays inside the ~50-line
  budget (plan §11) because it is one write-port assignment, one wakeup-slot
  assignment, one `io.wb` entry and one `recode`.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. Where the port attaches, and why the index matters ----

  In the register-arbitrate stage, after the existing `for (unit <- exe_units)` loop
  has consumed logical indices `0 until numFrfExeReadPorts` (leave that loop and its
  `<>` connections exactly as they are; only retarget its `require`), drive
  `fregfile.io.arb_read_reqs(numFrfExeReadPorts).bits := io.vec_frf_read_req`.

  APPENDING LAST IS LOAD-BEARING, not cosmetic. `PartiallyPortedRF` allocates in
  ascending logical index order, so a port placed after every exe-unit port can
  never take a physical port away from an `FPExeUnit`: `io_arb_frf_reqs(0..2).ready`
  — and therefore `io_squash_iss`, which replays FP issue when an arb request is
  denied — behaves exactly as it does today. Placing the vector port first, or
  interleaving it, would let a vector read squash scalar FP issue: a silent
  throughput regression with vectors on, and a change to the very arbitration order
  this delta promises not to touch.

  `arb_read_reqs(numFrfExeReadPorts).valid` is tied `true.B`: the seam carries no
  valid bit, so the port requests unconditionally, which is what "dedicated,
  unarbitrated, no ready line" means for a `Decoupled` port the elaboration
  `require` guarantees a physical port. Add
  `assert(fregfile.io.arb_read_reqs(numFrfExeReadPorts).ready)` in the same spirit
  as the existing `assert (ll_wbarb.io.in(0).ready)` — a standing check that the
  port-count math is still true after a config change.

  // If the per-cycle read ever matters for power, the fix is a `valid` bit on the
  // vec_pipeline_io seam — a hierarchy.yaml amendment, so not done here.

  ---- 2. The response, and the forward that makes it correct ----

  In the register-read stage, after the existing `for (unit <- exe_units)` loop
  (whose `require` is retargeted the same way), the added port's raw response is
  `fregfile.io.rrd_read_resps(numFrfExeReadPorts)`. That response is STALE with
  respect to a write presented in the response cycle, and this delta repairs it
  here rather than at the consumer:

    - a write presented in the ADDRESS cycle IS reflected in the response-cycle
      data — it took effect at the intervening clock edge, and the `Mem` read is
      combinational off the `RegNext`-ed address. No forward needed;
    - a write presented in the RESPONSE cycle is NOT reflected — it takes effect
      at the end of that cycle. The read returns the pre-write value and it must
      be forwarded.

  So: register the address once, `vec_frf_rd_addr = RegNext(io.vec_frf_read_req)`,
  and in the response cycle compare it against every valid
  `fregfile.io.write_ports(j)` (those valids are already `RT_FLT`-qualified, so no
  re-qualification here), substituting that port's `bits.data` on a hit.
  `RegisterFile`'s existing single-writer assertion already guarantees at most one
  write port targets a register in a cycle, so this is a `Mux1H` over the hit
  vector, not a priority mux; assert `PopCount(hits) <= 1` rather than relying on
  it silently.

  The forward only READS already-driven signals, so the write path stays
  cycle-identical — and it means the CONSUMER NEEDS NO FP WRITEBACK SNOOP PORT,
  which is what makes this seam match hierarchy.yaml, where `vec_pipeline_io`
  declares an FP read request and response and no FP snoop.

  The compare is over ALL of `fregfile.io.write_ports`, so with `usingRVV` it spans
  the port part 3b adds too — a `.vf` operand read in the same cycle a `vfmv.f.s`
  writes that register is forwarded by the same logic, with no special case.

  ---- 3. ⇒ THERE IS NO RESIDUAL WINDOW. A30 IS CLOSED, AND NO CONSUMER-SIDE
          OBLIGATION IS HANDED OUT. ----

  This part previously described a one-cycle hole the forward could not close, and
  told the consumer to fix it. IT NO LONGER EXISTS, and the reason is D4 rather than
  any change to the mechanism here.

  The arithmetic that produced it still holds for the SCALAR pipeline: an
  `FPExeUnit`'s wakeup is a FAST wakeup that LEADS its own register writeback by
  three cycles (`fastWakeupLatency = dfmaLatency - 3 // Three stages WAKE-ISS-ARB`,
  with `bypassable := true.B`), so from wakeup cycle T the write is presented at
  T+3, and a reader whose ADDRESS lands at T+2 or later is fully covered by the
  forward in part 2. The hole was that a consumer could address at T+1: a vector
  slot woken at T, granted at T+1, driving its FP read address COMBINATIONALLY in
  the grant cycle. That consumer was `VecScalarOperandRead`'s store instance.

  D4 DELETED THAT CONSUMER. The only reader of this port is now `VecCiiIssue`, whose
  slots are `IQ_V_ALU`, and `IQ_V_ALU` is PAST-PNR GATED: a granted CII op is OLDER
  THAN THE POINT-OF-NO-RETURN, which means every instruction ahead of it — including
  the FP producer of its `.vf` scalar — is past the PNR too and has genuinely written
  back. The T+1 read cannot happen: the grant is not merely "one cycle after a
  wakeup", it is "after the PNR passed the producer", which is strictly later than
  the producer's writeback. There is no fast-wakeup lead left to outrun, so there is
  nothing for the consumer to delay and no `bypassable`-hold obligation on
  `VecIssueSlot` from this module.

  // The forward in part 2 STAYS anyway, and this is not belt-and-braces for its own
  // sake: past-PNR gating is an argument about the producer having retired, and the
  // forward is a proof about the one cycle the register file itself is incoherent.
  // The long-latency FP producers were never exposed either — `ll_wbarb`'s wakeup
  // fires in the SAME cycle as `fregfile.io.write_ports(0).valid`, with
  // `speculative_mask := 0` and `bypassable := false`, so its write has landed by
  // the end of the wakeup cycle. And note the port added in part 3b is in the same
  // class: same-cycle wakeup and write, no bypass, no window.

  ---- 3b. The DEDICATED scalar-FP write port and its wakeup slot (D7) ----

  Two additions, both appended LAST for the same reason the read port is, and both
  driven combinationally from `io.vec_fp_wb`:

  THE WRITE PORT. After the existing `for (i <- 1 until lsuWidth)` and
  `for (eu <- exe_units)` loops have run `w_cnt` up to `fpWidth + lsuWidth` — leave
  both loop bodies and their order exactly as they are — drive
  `fregfile.io.write_ports(w_cnt)`:
    `.valid     := io.vec_fp_wb.valid && io.vec_fp_wb.bits.uop.dst_rtype === RT_FLT`
    `.bits.addr := io.vec_fp_wb.bits.uop.pdst`
    `.bits.data := recode(io.vec_fp_wb.bits.data, <tag>)`
  then `w_cnt += 1`, so `require(w_cnt == fregfile.io.write_ports.length)` still
  holds against the widened `numFrfWritePorts`. The `RT_FLT` qualification is the
  same discipline every other write port in this file follows, and it is what keeps
  a `vmv.x.s` beat — which goes to the INT side — from writing an FP register.

  NO `RegNext` ON THIS PORT. `write_ports(0)` is delayed a cycle deliberately (to cut
  `ll_wbarb`'s critical path) and the exe-unit ports are not; this one follows the
  exe-unit ports, direct, because the CII beat has already crossed a queue and a
  registered writeback stage inside `VecCiiWriteback` and needs no further pipelining
  here. Adding a delay would also desynchronize it from its wakeup slot below.

  THE WAKEUP SLOT. After the existing commit-stage loops have run `idx` up to
  `fpIssueParams.issueWidth + numLlPorts`, drive `fp_wakeups(idx)` from the same
  beat: `valid` with the same `RT_FLT` qualification, `bits.uop :=
  io.vec_fp_wb.bits.uop`, and `speculative_mask := 0.U`, `bypassable := false.B`,
  `rebusy := false.B`. Then `io.wb(idx) := io.vec_fp_wb` with
  `io.wb(idx).bits.data := recode(io.vec_fp_wb.bits.data, <tag>)` — `io.wb` carries
  `ExeUnitResp(fLen+1)` in RECODED form and core.scala applies `ieee()` on the way
  into the ROB, so the recoded value is what belongs here, exactly as the
  `1 until lsuWidth` slots already do. `fflags` passes through untouched;
  `VecCiiWriteback` has already accumulated it. Then `idx += 1`, so
  `require(idx == numWakeupPorts)` holds. `issue_unit.io.wakeup_ports := fp_wakeups`
  needs no edit and now carries the slot.

  WAKEUP AND WRITE IN THE SAME CYCLE IS WHAT MAKES `bypassable := false` CORRECT AND
  ADDS NO WINDOW. The earliest a consumer woken at T can present an address is T+1,
  and `FullyPortedRF` reads `RegNext`-ed addresses, so its data comes back at T+2;
  the write landed at the end of T. So no `fp_bypasses` entry is added (`fp_bypasses`
  stays `Vec(fpWidth, ...)`, exe-unit sourced, untouched) and no consumer needs a
  bypass from this port. `speculative_mask := 0` and `rebusy := false` are literal
  truth here rather than convention: nothing about a CII scalar-FP result is
  speculative on a load hit, so there is no re-busy case to express.

  NO KILL LOGIC IS ADDED ON THIS PATH. `VecCiiWriteback` suppresses all three of its
  destinations for a killed tag (`io.wb_suppress`, from `VecCiiFlush`'s kill window),
  so a doomed beat never reaches this port in any cycle — including the flush cycle.
  This module therefore applies no `IsKilledByBranch`/`UpdateBrMask` of its own, and
  must not: a second, later kill qualification here would be a different window from
  the one the vector side uses, and the two disagreeing is worse than either alone.

  // ===> THE RECODE TAG IS THE ONE LOOSE END, AND IT IS FLAGGED, NOT GUESSED.
  // `recode(x, tag)` needs a type tag — `0` for single, `1` for double — and BOOM's
  // existing long-latency path derives it as `mem_size =/= 2.U`. A `vfmv.f.s` result
  // is `SEW` wide, so the tag is the same shape off the vector element width on the
  // writeback uop: `<tag> = io.vec_fp_wb.bits.uop.v_eew =/= 2.U` (`v_eew` encodes
  // 8/16/32/64 as 0..3, so `2` is 32-bit = single). `MicroOp`'s `v_eew` is DOCUMENTED
  // as "the element width of the DATA a memory access moves", so `VDecode` must also
  // set it for a scalar-FP-destination CII op or an `SEW=32` `vfmv.f.s` will be
  // recoded as a double and read back as a NaN-boxed wrong value. That is a MicroOp/
  // VDecode obligation this port depends on; reported rather than assumed. No
  // element-width member is added to the SEAM — the tag comes off the uop already on
  // `fp_wb`, which is why the read-seam prohibition on size inputs still stands.

  ---- 4. IEEE, not hardfloat, on the way out ----

  `fregfile` stores hardfloat RECODED values, `fLen+1` bits wide. Apply
  `ieee(...)` — from `tile.HasFPUParameters`, which this module already mixes in —
  ONCE, after the forward mux (both the register-file response and every write
  port's data are in the same recoded encoding), and drive `io.vec_frf_read_rsp`
  with the result, zero-extended if `fLen < xLen`. This matches the vec seam's
  stated convention (data crosses it in the register file's ARCHITECTURAL
  encoding; any recode back into hardfloat is the consumer's) and what core.scala
  already does at the other FP boundary: `rob.io.wb_resps(cnt).bits.data :=
  ieee(wb.bits.data)` on `fp_pipeline.io.wb`.

  `ieee(x)` defaults to `maxType` and performs NaN UNBOXING, so a register written
  as a recoded single returns a NaN-boxed value whose low 32 bits are the correct
  single-precision datum. The vector side therefore takes the low `SEW` bits and
  NO element-width input is needed on this seam — which is why no size signal is
  added, unlike the `recode(..., mem_size =/= 2.U)` the long-latency write path
  needs.

  ---- 5. The wakeup tap ----

  //@req-spec-vrf.e4
  The `.vf` scalar feeder of a vector slot is woken on the FP wakeup network, and
  that network is the one already here: the `fp_wakeups` wire presented on
  `io.wakeups`, unmodified. This delta adds no port and no logic for it. BoomCore's
  delta fans `fp_pipeline.io.wakeups` out to `vec_pipeline_io` alongside its
  existing consumer `fp_rename_stage.io.wakeups`, and the vector issue slot's
  scalar-feeder comparator matches `bits.uop.pdst` against the slot's FP-typed
  scalar source exactly as an FP issue slot does. The obligation this module
  carries is therefore NEGATIVE, and literal: nothing may be inserted between
  `fp_wakeups` and `io.wakeups` — no `RegNext`, no re-qualification, no
  per-consumer variant — because the vector slot and the scalar FP slots must
  observe the same wakeup in the same cycle.

  The slot part 3b appends rides this same wire, which is the point of appending it
  there rather than exporting a separate signal: a `.vf` op whose scalar source is
  the FP destination of an earlier `vfmv.f.s` wakes through the ordinary FP network,
  and a SCALAR FP op consuming that register wakes through the same slot in the same
  cycle. One network, two kinds of consumer, no vector-specific wakeup path — which
  is the same rule the rest of this section states, applied to the producer side.

  // What this network does and does not carry, since the consumer's comparator
  // depends on it: every FP wakeup port drives `rebusy := false.B`. BOOM v4 has
  // NO speculative load-hit wakeup on the FP side — `io.lsu.fresp` feeds
  // `ll_wbarb` on actual data return and there is no `fwakeups` analogue of
  // `io.lsu.iwakeups` — so a `.vf` feeder needs no re-busy or replay machinery
  // and none is added: midcore.rst's "re-busied through the same machinery if the
  // load later misses" is discharged vacuously here, not by new logic. The only
  // `bypassable` FP wakeup is the FMA fast wakeup of part 3, and part 3 explains
  // why no reader of this delta is exposed to its lead.

  ---- 6. Everything else, held still ----

  //@req-spec-core.b4
  The FP execution subsystem is otherwise unchanged from BOOM v4, and that is a
  requirement rather than an aspiration. Cycle for cycle, in a vectors-ON build
  too: the `FPExeUnit` construction and its `hasFDiv`/`hasFpiu` placement; the FMA
  latency (`dfmaLatency`, and with it `fastWakeupLatency` and the WAKE-ISS-ARB
  alignment); the divSqrt unit and its `io_fdiv_resp` arbiter input; the
  `ll_wbarb` input order (mem / fromint / fdiv), its INPUT COUNT — still three, D7
  adds NOTHING to it — and its one-cycle write delay; the existing read-port
  arbitration order; the `fregfile` write-port assignment order for indices `0 until
  fpWidth + lsuWidth` and the `recode(...)` on the long-latency path; `fp_bypasses`;
  every EXISTING entry of the `io.wakeups`/`io.wb` commit-stage assembly and the
  `fflags` payload inside `ExeUnitResp`; `io.to_int`; `io.dgen`; and the issue unit's
  dispatch and wakeup wiring. The two things D7 changes are ADDITIVE and terminal: one
  write-port index after the last existing one, one wakeup/`io.wb` index after the
  last existing one. Nothing existing moves, is renumbered, or is arbitrated against.

  With `usingRVV = false` every item above is not merely preserved but bit-identical,
  because the added read port, the address register, the forward mux, the `ieee`
  instance, the extra physical read port, the added write port, its `recode`, and the
  added wakeup/`io.wb` slot are all ABSENT — not tied off — so the read-port count,
  the write-port count, `numWakeupPorts`, the arbiter and the `io` Bundle are exactly
  the baseline's.

  No `VecTrace` line is added here. The shared helper keys on a `MicroOp`/`rob_idx`;
  the added read port sees only an address, and it is already visible in
  `VecCiiIssue`'s trace line on the consumer side, which has the uop. The added write
  port's beat is likewise already traced by `VecCiiWriteback`.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Latency of the added read: exactly ONE cycle, address to data, identical to every
other FP read port and to the INT reads the same consumer performs in parallel. No
pipeline stage is added, and adding one would MOVE the forward window described in
the logic section — do not. Throughput: one read per cycle, sustained; nothing on
this port can back-pressure, stall or be denied, by construction of the port count.

Latency of the added write: ZERO added cycles — the beat writes the register file in
the cycle it arrives and wakes in that same cycle. Throughput: one beat per cycle,
sustained, WITH NO POSSIBILITY OF DENIAL, which is the entire point of D7: back-to-back
scalar-FP CII beats all land, and no bound on arbitration loss has to be argued
because there is no arbitration.

Area: one physical FP read port and one physical FP write port. `FullyPortedRF`'s cost
model is `(R+W)*(R+2W)`, so at the Medium tier (`R = 3 -> 4`, `W = 2 -> 3`) this is a
real but bounded growth in the FP file, paid only in a vectors-on build — plus one
`maxPregSz`-wide address register, one `numFrfWritePorts`-wide address compare, one
`Mux1H` on `fLen+1` bits, one `ieee` instance, two `recode` instances (write port and
`io.wb` entry) and one wakeup slot's worth of comparators inside the FP issue unit.
The write port is the more expensive half in that cost model — `W` is weighted twice
— and it is bought deliberately, against a correctness argument, not a throughput one.

Critical path to watch: the response-cycle forward plus `ieee`, in series with the
register-file read output and feeding the consumer's operand capture. It is off the
existing FP read ports' paths entirely. If it fails timing the fix is NOT to delete
the forward (that is the stale-operand bug) and NOT to add a stage (that moves the
window); it is to hoist `ieee` across the seam by widening the response to the
recoded `fLen+1` encoding — a `vec_pipeline_io` change, so a hierarchy.yaml
amendment.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the added READ port carries a bare physical register number and reads no
`MicroOp` field (the FP-typed source selection that produces the address happens in
`VecCiiIssue`). The added WRITE port does read the uop on `io.vec_fp_wb`: `pdst`,
`dst_rtype` and the vector element-width field for the `recode` tag (see the flagged
note in logic part 3b). The `depends_on: MicroOp` edge also exists because
`FpPipeline`'s pre-existing ports (`io.dis_uops`, `io.wakeups`, `io.wb`,
`io.ll_wports`) carry `MicroOp`, and the `*_rtype` widening that delta performs must
not change the result of this module's `dst_rtype === RT_FLT` comparisons.

Binds to pre-existing machinery, all unchanged: `BankedRF` / `PartiallyPortedRF` /
`FullyPortedRF` in src/main/scala/v4/exu/register-read/regfile.scala — whose
ascending-index read arbitration and absent read-during-write forwarding are the
two facts this delta is built on — `tile.HasFPUParameters` for `ieee` and `recode`,
and `HasBoomCoreParameters` for `usingRVV`, `fpWidth`, `lsuWidth`, `maxPregSz`,
`xLen`, `fLen`.

Instantiates nothing new. Consumers of the delta: `VecCiiIssue` (sole driver of the
added read port, since D4 deleted the store-side reader), `VecCiiWriteback` (through
`VecPipeline`, sole driver of the added write port), and `VecIssueSlot` for the
wakeup tap — all wired by the BoomCore delta through `vec_pipeline_io`.

// BoomCore's delta must widen with this one. The landing
// site now exists here; the wiring of `fp_pipeline.io.vec_fp_wb`, and of the extra
// `io.wakeups`/`io.wb` entry into `rob.io.wb_resps` and the FP rename/vector wakeup
// fan-out, is BoomCore's to state. Flagged across, not silently assumed.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File   src/main/scala/v4/exu/fp-pipeline.scala
    Class  `class FpPipeline(implicit p: Parameters) extends BoomModule with
            tile.HasFPUParameters` (package boom.v4.exu). Hand-written baseline
            BOOM v4.

  In scope:
    - Adding `vec_frf_read_req` and `vec_frf_read_rsp` to the `io` Bundle, both
      conditional on `usingRVV`.
    - Adding `val numFrfExeReadPorts = fpWidth * 3`; redefining
      `numFrfLogicalReadPorts` as `numFrfExeReadPorts + (if (usingRVV) 1 else 0)`.
    - Changing ONLY the `numPhysicalReadPorts` argument of the existing
      `Module(new BankedRF(...))` call to `numFrfReadPorts + (if (usingRVV) 1
      else 0)`. No other constructor argument moves.
    - Retargeting the two existing `require(rd_idx == numFrfLogicalReadPorts)`
      checks (register-arbitrate stage and register-read stage) to
      `require(rd_idx == numFrfExeReadPorts)`.
    - Adding one `usingRVV`-gated block in the register-arbitrate stage that
      drives `fregfile.io.arb_read_reqs(numFrfExeReadPorts)` and asserts its
      `ready`; and one in the register-read stage holding the address register,
      the write-port forward `Mux1H`, the `PopCount(hits) <= 1` assertion, the
      `ieee` conversion, and the drive of `io.vec_frf_read_rsp`.
    - Adding the `usingRVV`-gated `require(numFrfReadPorts + 1 >=
      numFrfLogicalReadPorts)`.
    - ADDED BY D7 (this widens what was previously refused — see the amended reject
      list in the ports section; it is a deliberate scope change, recorded here so a
      reviewer diffing scope against diff sees it declared):
        * Adding `vec_fp_wb` to the `io` Bundle, `usingRVV`-gated.
        * Redefining `numFrfWritePorts` as `fpWidth + lsuWidth + (if (usingRVV) 1
          else 0)` and `numWakeupPorts` as `fpIssueParams.issueWidth + numLlPorts +
          (if (usingRVV) 1 else 0)`. `fregfileBankedWriteArray` and the `BankedRF`
          write-port argument follow the new count with no edit of their own.
        * Adding one `usingRVV`-gated write-port assignment AFTER the existing
          `w_cnt` loops (index `fpWidth + lsuWidth`), with `w_cnt += 1` so
          `require(w_cnt == fregfile.io.write_ports.length)` still holds.
        * Adding one `usingRVV`-gated `fp_wakeups(idx)` + `io.wb(idx)` assignment
          AFTER the existing commit-stage loops, with `idx += 1` so
          `require(idx == numWakeupPorts)` still holds.

  Must not regress:
    - `require (numFrfReadPorts >= 3)` stays, unchanged and un-relaxed.
    - LATENCIES. `dfmaLatency` and therefore `fastWakeupLatency`, the divSqrt
      unit's latency and its `io_fdiv_resp` handshake, and the `exe_units`
      construction (`fpWidth` instances, `hasFDiv = usingFDivSqrt && (w ==
      fpWidth-1)`, `hasFpiu = (w == fpWidth-1)`, the `fp_exe_unit_${w}` names).
      The three-cycle WAKE-ISS-ARB wakeup lead must still hold exactly — the scalar
      bypass is stated against it, and so is part 3's argument that no vector reader
      is exposed to it.
    - THE EXISTING READ-PORT ARBITRATION ORDER. Logical ports
      `0 until numFrfExeReadPorts` keep their identity, order and `<>` connection
      to `unit.io_arb_frf_reqs(i)` / `unit.io_rrd_frf_resps(i)`; the added port is
      strictly LAST, so no `FPExeUnit` is ever denied a physical port it would
      have won today, and `io_squash_iss` and the FP issue replay it drives keep
      their current behaviour.
    - THE EXISTING WRITEBACK PATH: `ll_wbarb`'s INPUT COUNT (still exactly three —
      D7 adds a register-file port, NOT an arbiter input) and input order (0 = mem
      with its `RegNext` + `recode(..., mem_size =/= 2.U)`, 1 = `io.from_int`,
      2 = fdiv), the one-cycle write delay on `fregfile.io.write_ports(0)` and its
      comment, the `w_cnt` assignment order for indices `0 until fpWidth + lsuWidth`,
      `assert (ll_wbarb.io.in(0).ready)`, and the `RT_FLT` qualification on every
      write port's valid — which the added port also observes. `require (w_cnt ==
      fregfile.io.write_ports.length)` stays, and now checks the widened count.
    - `fflags` ACCUMULATION: the `fflags` field of every EXISTING `io.wb(idx)` entry
      and the `io.wb`/`io.wakeups` commit-stage assembly producing it — including
      `speculative_mask := 0.U`, `rebusy := false.B`, `bypassable := false.B` on
      the `ll_wbarb` and extra-`ll_wport` slots and the `recode` on
      `io.wb(idx).bits.data` for `i >= 1`. `numLlPorts` unchanged;
      `require (idx == numWakeupPorts)` stays and now checks the widened count. No
      existing wakeup or `io.wb` index may be renumbered — the added slot is LAST.
    - `fp_bypasses`, `io.to_int`, `io.dgen`, `io.fcsr_rm`/`io.status` fan-out, the
      `io_kill := io.flush_pipeline` loop, `issue_unit`'s construction and all of
      its dispatch/wakeup/`fu_types` wiring, and `toString`.
    - With `usingRVV = false`: BIT-IDENTICAL to the current file — same
      `numFrfLogicalReadPorts`, same physical read-port count, same
      `numFrfWritePorts`, same `numWakeupPorts`, same arbiter, same `io` Bundle, no
      present-but-unused port or register. Gate (f) diffs this RTL, and everything
      D7 adds is inside the `usingRVV` gate, so it lands in D1's ENUMERATED
      EXCEPTION rather than enlarging it.
    - No reformatting, reordering or renaming of existing statements; the existing
      copyright header and comment style are preserved.

  Interface delta:
    NEW ports on `io` (all `usingRVV`-gated):
      vec_frf_read_req                            : Input(UInt(maxPregSz.W))
      vec_frf_read_rsp                            : Output(UInt(xLen.W))
      vec_fp_wb                                   : Input(Valid(ExeUnitResp(xLen)))

    NEW elaboration values: numFrfExeReadPorts; numFrfLogicalReadPorts,
    numFrfWritePorts and numWakeupPorts redefined; one added `require`.

    WIDENED ports, `usingRVV` ONLY: `io.wakeups` and `io.wb`, by exactly ONE entry
    appended LAST. Every existing entry keeps its index, content and timing. With
    vectors off both are exactly the baseline's width. CHANGED port semantics: none.

    Explicitly NOT added, and a reviewer should reject them if they appear:
      any new wakeup NETWORK or vector-specific wakeup bus (the added slot rides
      `fp_wakeups`/`io.wakeups`, it does not create a second net); any `RegNext` or
      other element on the `fp_wakeups` -> `io.wakeups` path; an `fp_wb_snoop`
      output (use the existing `io.wb`); a `valid` or `ready` on either added read
      signal, or a `ready` on `vec_fp_wb`; a second FP READ port, or any read port
      placed before the exe-unit ports; a SECOND FP write port, a write port placed
      before the existing ones, or a 4th `ll_wbarb` input (D7 rejects arbitration
      explicitly — see the ports section); any element-width or `mem_size`-style
      input on the added read seam (the write path's `recode` tag comes off the uop
      already on `vec_fp_wb`, not from a new seam member); any vector register file
      port; any change to `dfmaLatency`, `fastWakeupLatency`, the divSqrt latency,
      the `ll_wbarb` input order or any existing slot's `fflags` path.

    // NO LONGER ON THIS LIST (D7, and deliberately so): "an FP register-file WRITE
    // port, a change to `numFrfWritePorts`, or an added wakeup slot for the vector
    // scalar-destination writeback". Those three were refused by the previous
    // revision, flagged upward as A31, and granted by D7. A reviewer comparing this
    // node's diff against an older copy of the reject list should read the amendment
    // in the ports section before rejecting them.
<|end_edit_scope|>

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
  VecTrace — the shared guarded-printf tracing convention for every vec module.

  hierarchy.yaml: kind: package, mode: new,
  output src/main/scala/v4/vec/generated/VecTrace.scala,
  package boom.v4.vec.generated. depends_on MicroOp.

  PACKAGE NODE CONVENTION. A `kind: package` emitting a Scala `object VecTrace`
  of helper methods. No Module, no I/O, no state. The ports section is
  explicitly empty.

  ===> NO REQUIREMENTS ARE ALLOCATED TO THIS NODE, and that is a decision rather
       than an oversight. It is the only node in the map with an empty `reqs:`
       list. Tracing is a ground rule of the implementation plan (rule 11), not
       an architectural obligation in the .rst specs, so the requirement corpus
       contains nothing to cite here. There are therefore no requirement tags
       anywhere in this file, and a reviewer should not expect any.

  WHY IT IS A PACKAGE. This project has NO unit tests — no `chiseltest`, no
  per-module spec classes. Validation is end-to-end VCS + Whisper cosim
  regression only. Guarded tracing is consequently the primary debug surface for
  the whole vector subsystem, and a convention that every module reinvented
  would be useless for exactly the thing it exists for: correlating stages with
  each other and with the Whisper trace. One declaration, bound by all of them.
*/

<|begin_module|>

  <|begin_parameters|>
  No constructor parameters. The helpers are parameterized implicitly through
  Chisel's `Parameters` on `robAddrSz`, `vecPregSz` and `vlPregSz` for the field
  widths they format.

  Tracing is enabled at RUN TIME, not elaboration time, by a `vecTrace` plusarg
  read once and defaulting to OFF. It is a plusarg and not a Scala parameter
  because a config-time switch would produce a differently-elaborated machine,
  and then the traced build would not be the build under test.
  <|end_parameters|>

  <|begin_ports|>
  None. This is a declaration unit: no I/O, no clock, no reset.
  <|end_ports|>

  <|begin_logic|>
  ---- The gate ----

  Expose `traceEnabled`: the value of a `vecTrace` plusarg, obtained through
  rocket-chip's `PlusArg` utility, default 0 (off). BOOM already uses `PlusArg`
  for `boom_timeout`, so this follows the existing convention rather than
  introducing a second one.

  Every trace statement is gated on `traceEnabled && !reset`. Both terms are
  required: the plusarg term keeps the default-off promise below, and the reset
  term suppresses the flood of garbage lines that registers emit before they are
  initialized, which otherwise buries the first real event.

  ---- The emit helper ----

  Provide one method, `trace(module: String, event: String, uop: MicroOp, extra:
  Seq[(String, Bits)] = Nil)`, that emits a single line when the gate is true.
  One line per key event. The line format is fixed and greppable:

    [vec] <module> <event> rob=<rob_idx> <key>=<value> ...

  `module` and `event` are Scala strings resolved at elaboration, so they cost
  nothing in hardware. Every line carries `rob_idx`, unconditionally and never
  as an optional field — it is the only identifier common to the decode, rename,
  issue, LSU and CII stages, so it is what makes a line correlatable with
  another module's line and with the Whisper cosim trace. A line without it is
  not useful and the helper must not permit one.

  Provide convenience wrappers that add the fields each family of modules always
  wants, so a caller does not assemble them by hand and get the names
  inconsistent: `tracePrn` adds `pvdest` and the member count, `traceVl` adds
  `pvl` and `vl`, `traceElem` adds the element index and `eew`, and `traceTag`
  adds the CII `tag`.

  ---- The decode-stage variant ----

  Provide a second entry point, `traceDecode(module, event, ftq_idx, pc_lob,
  extra)`, for callers in the DECODE stage. It is identical except that it is
  keyed on `ftq_idx`/`pc_lob` instead of `rob_idx`, and it emits `rob=?` in that
  position so a line is still recognisable by the same grep.

  // ===> THIS EXISTS BECAUSE rob_idx DOES NOT YET EXIST AT DECODE. The ROB entry
  // is allocated at DISPATCH, so VDecode, VLSDecode, VsetDecode and VConfigUnit
  // have no rob_idx to tag a line with. Without this variant a decode-stage
  // caller would either be unable to trace at all or would invent a zero
  // rob_idx, and a line claiming rob=0 is worse than a line admitting it does
  // not know — it would silently alias with the real rob entry 0 in every grep.
  // `ftq_idx`/`pc_lob` is the identifier those stages DO have, and it is enough
  // to correlate a decode line with the Whisper trace by PC. Correlating a
  // decode line with a later pipeline line is then a two-step join through the
  // dispatch line, which is the honest cost of the ROB entry not existing yet.

  ---- The two uOP-less variants, and which to reach for ----

  Some callers have no `MicroOp` at their boundary. They split into two cases and
  get one entry point each. **Use them in this order — the first that applies:**

    1. a `MicroOp` in scope           -> `trace` / `tracePrn` / `traceVl` /
                                          `traceElem` / `traceTag`
    2. no uOP, but a `rob_idx`        -> `traceId(module, event, rob_idx, extra)`
    3. no instruction identity at all -> `traceStruct(module, event, extra)`

  `traceId` emits a real `rob=<rob_idx>` and is therefore fully correlatable; it
  differs from `trace` only in taking the index directly instead of extracting it
  from a uOP. Reach for it whenever a `rob_idx` is genuinely available, because
  the ONLY thing that makes `traceStruct`'s `rob=?` acceptable is that no honest
  answer exists — and an unnecessary `rob=?` throws away the cross-stage and
  Whisper correlation this package exists to provide.

  // ===> A GROUP-DONE WAKEUP IS THE MOTIVATING CASE FOR `traceId`. It carries a
  // bare `rob_idx` and a member-PRN vector, and no `MicroOp` — so `VecBusyTable`
  // could not trace its clear event at all, while having the very identifier the
  // line format wants. That is a missing entry point, not a caller problem.

  Provide `traceStruct(module, event, extra)` for callers whose event is scoped
  to a PHYSICAL RESOURCE rather than to an instruction. It
  emits `rob=?` in the same position as `traceDecode`, so one grep still finds
  every line. It takes no identifier argument of its own: the identifying key —
  `prn`, `port`, `bank`, `entry` — is the caller's to name in `extra`, and
  `extra` must therefore be non-empty (check it at elaboration; a structural
  line with no key identifies nothing).

  // ===> THESE EXIST BECAUSE SOME MODULES HAVE NO uOP AT THEIR BOUNDARY AT ALL,
  // and the original text did not account for them. `trace`/`tracePrn`/
  // `traceVl`/`traceElem`/`traceTag` all take a `MicroOp` to extract `rob_idx`;
  // `traceDecode` covers the decode stage. But `VecMapTable`, `VecFreeList`,
  // `VecBusyTable`, `VecRegFile`, `VecRegFileBank`, `VlRegFile` and
  // `VecGroupReady` are LOOKUP AND STORAGE STRUCTURES, not pipeline stages:
  // their `depends_on` deliberately excludes MicroOp, their ports carry bare
  // addresses and data, and their events are genuinely about a resource, not an
  // instruction — "PRN 37 freed at commit", "bank 2 forwarded a write to read
  // port 5". There is no uOP in scope to extract a `rob_idx` from, and inventing
  // a port to carry one would add a wire that only tracing reads, which the next
  // section forbids.
  //
  // Ground rule 11 of plan v2 requires EVERY vec module to trace, so with only
  // the two entry points above, ground rule 11 was unsatisfiable for most of the
  // Phase C module set. Observed across Phases B and C: `VConfigUnit`,
  // `VlRegFile`, `VecFreeList` and `VecBusyTable` omitted trace calls and
  // reported the gap (`VecFreeList` could tag zero of its three lines), while
  // `VecRegFileBank` hand-rolled a raw `printf` behind the public
  // `traceEnabled`. Five nodes, two incompatible workarounds, one missing pair of
  // entry points — which is why these are named helpers and not a convention.
  //
  // ===> IT DOES NOT RELAX THE rob_idx RULE FOR INSTRUCTION-SCOPED EVENTS. If a
  // module HAS a uOP at its boundary, its instruction events MUST use `trace` or
  // one of its wrappers; reaching for `traceStruct` to avoid threading a uop is
  // a review failure. The test is what the event is ABOUT, not what is
  // convenient to wire: `VecStoreDgenPath` and `VecIssueSlot` hold uOPs and owe
  // real `rob_idx` lines, while a free-list pop owes a `prn` and could not
  // honestly name a `rob_idx` even if one were available, because a group is
  // allocated for one uOP and freed on behalf of another.
  //
  // `emitLine` STAYS PRIVATE and a caller must not hand-roll a `printf` behind
  // `traceEnabled` — the whole point of this package is that the line format is
  // in one place. `traceEnabled` remains public only for gating a caller's own
  // non-emitting debug logic.

  ---- What it must not become ----

  These helpers emit only. They must declare no register, no counter and no
  wire that any non-trace logic reads, so that removing every call site would
  leave the design's behaviour bit-identical. A trace helper that accumulated
  state would make the traced and untraced builds different machines, which is
  the one failure mode that would make tracing worse than nothing.

  Performance counters are a separate concern and are NOT here: they are
  architectural state in a `perfEvents` EventSet, they are read by software, and
  they must exist whether tracing is on or off.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
The gate must be off by default and cost nothing when off. Specifically: with
the `vecTrace` plusarg absent, the emitted RTL must be bit-identical to the same
design with every trace call site deleted — the `printf` statements sit behind
the gate and add no logic to any functional path.

This matters beyond tidiness. Gate (f) of the plan requires that a
`usingRVV = false` build be bit-identical to pre-Caracal BOOM v4. Tracing lives
inside `usingRVV`-gated modules, so it cannot affect that build at all; but it
must equally not perturb the `usingRVV = true` build being measured against the
performance targets, or the traced run and the measured run would not be the
same machine.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the `trace` helper takes a `MicroOp` to extract `rob_idx` and the
vector fields the wrappers format. `traceDecode`, `traceId` and `traceStruct` do
NOT, which
is what lets a module whose `depends_on` excludes MicroOp still satisfy ground
rule 11 without acquiring a MicroOp port purely to be traceable.

Binds to `freechips.rocketchip.util.PlusArg` for the run-time gate.

Instantiates nothing. Every `vec/**` module depends on it, so it must stay
dependency-light in the other direction: it may not depend on VecBundles or on
any module's types, or the declaration graph gains a cycle.
<|end_dependencies|>

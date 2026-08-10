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

package boom.v4.vec.generated

import chisel3._
import chisel3.util._

import freechips.rocketchip.util.PlusArg

import boom.v4.common.MicroOp

// GENERATED from src/main/nlhdl/pkg/VecTrace.nlhdl.scala. Do not hand-edit;
// regenerate via the nlhdl gen-rtl flow instead.
//
// VecTrace -- the shared guarded-printf tracing convention for every vec
// module.
//
// PACKAGE NODE CONVENTION. A `kind: package` emitting a Scala `object
// VecTrace` of helper methods. No Module, no I/O, no state. The nlhdl
// source's ports section is explicitly empty.
//
// NO REQUIREMENTS ARE ALLOCATED TO THIS NODE (hierarchy.yaml `reqs: []`), and
// that is a decision, not an oversight: tracing is ground rule 11 of the
// implementation plan, not an architectural obligation in the .rst corpus, so
// there is nothing in the requirement corpus to cite here and no `//@req-`
// tags appear anywhere in this file.
//
// WHY IT IS A PACKAGE. This project has NO unit tests -- no `chiseltest`, no
// per-module spec classes. Validation is end-to-end VCS + Whisper cosim
// regression only. Guarded tracing is consequently the primary debug surface
// for the whole vector subsystem, and a convention every module reinvented
// would be useless for exactly the thing it exists for: correlating stages
// with each other and with the Whisper trace. One declaration, bound by all
// of them.
object VecTrace {

  // ---- The gate ----
  //
  // Tracing is enabled at RUN TIME, not elaboration time, by a `vecTrace`
  // plusarg read once per call site and defaulting to OFF. It is a plusarg
  // and not a Scala/config parameter because a config-time switch would
  // produce a differently-elaborated machine, and then the traced build would
  // not be the build under test.
  //
  // `def`, not `val`/`lazy val`: `PlusArg`'s underlying `plusarg_reader` is
  // itself a Module, instantiated as a child of whichever module is
  // elaborating at the call site. There is no single global instance that
  // could be shared across the many, unrelated calling modules without
  // fabricating a cross-hierarchy wire, so every reference below (and every
  // caller that reads `traceEnabled` directly) pays for -- and gets -- its
  // own reader. This is the same idiom BOOM already uses for `boom_timeout`
  // (see `v4/exu/core.scala`), just called from more places.
  def traceEnabled: Bool =
    PlusArg("vecTrace", 0, "Enable guarded vec-pipeline debug tracing (VecTrace); off by default, and free when off", width = 1).orR

  // ---- The emit primitive ----
  //
  // Every trace statement is gated on `traceEnabled && !reset`. Both terms
  // are required: the plusarg term keeps the default-off promise (see the
  // nlhdl `perf` section), and the reset term suppresses the flood of garbage
  // lines that registers emit before they are initialized, which would
  // otherwise bury the first real event.
  //
  // `module` and `event` are Scala `String`s resolved at elaboration, so they
  // cost nothing in hardware -- only `robArgs`/`fields` become `printf`
  // arguments. `robFmt` is a literal format fragment ("%d" for a real
  // `rob_idx`, "?" for the decode-stage variant below) rather than a %d
  // argument, so the "rob=?" case needs no dummy UInt to carry.
  //
  // This helper emits ONLY: it declares no register, no counter and no wire
  // that any non-trace logic reads, so that deleting every call site (or
  // running with the plusarg unset, which drives `traceEnabled` to false at
  // runtime) leaves the design's cycle-by-cycle behavior bit-identical. A
  // trace helper that accumulated state would make the traced and untraced
  // builds different machines, which is the one failure mode that would make
  // tracing worse than nothing.
  private def emitLine(
    module:  String,
    event:   String,
    robFmt:  String,
    robArgs: Seq[Bits],
    fields:  Seq[(String, Bits)]): Unit = {
    // `Module.reset` rather than a bare `reset`: this is an `object`, not a
    // Module, so there is no implicit `reset` in lexical scope here. The
    // helper still runs inside the caller's module context at elaboration, so
    // Module.reset resolves to the CALLER's reset -- which is the one we want
    // (suppress trace lines while that module is in reset), and is the same
    // convention BOOM's existing `!reset.asBool` printf guards use.
    when (traceEnabled && !Module.reset.asBool) {
      // [vec] <module> <event> rob=<rob_idx> <key>=<value> ...
      val fmt = s"[vec] $module $event rob=$robFmt" +
        fields.map { case (k, _) => s" $k=%d" }.mkString + "\n"
      printf(fmt, (robArgs ++ fields.map(_._2)): _*)
    }
  }

  /**
   * One line per key event. `rob_idx` is carried unconditionally and never
   * as an optional field -- it is the only identifier common to the decode,
   * rename, issue, LSU and CII stages, so it is what makes a line
   * correlatable with another module's line and with the Whisper cosim
   * trace. A line without it is not useful and this signature does not
   * permit one (contrast [[traceDecode]] below, whose ROB entry genuinely
   * does not exist yet).
   */
  def trace(module: String, event: String, uop: MicroOp, extra: Seq[(String, Bits)] = Nil): Unit =
    emitLine(module, event, "%d", Seq(uop.rob_idx), extra)

  // ---- Convenience wrappers ----
  //
  // Each adds the fields its family of callers always wants, so a caller
  // does not assemble them by hand and get the names inconsistent.

  /**
   * `pvdest` and the member count, for producer-side (rename/dispatch) trace
   * lines. `pvdest` is reported as the group's base PRN (member 0); `nmem`
   * (== `v_emul`, "group member count, 1..8" per MicroOp) tells a reader how
   * many contiguously-numbered members follow it.
   */
  def tracePrn(module: String, event: String, uop: MicroOp, extra: Seq[(String, Bits)] = Nil): Unit =
    trace(module, event, uop, Seq(("pvdest", uop.pvdest.get.head), ("nmem", uop.v_emul.get)) ++ extra)

  /**
   * `pvl` and `vl`. `pvl` (the renamed VL-file pointer this uop reads) comes
   * straight off the `uop`. The numeric VL value itself is NOT a `MicroOp`
   * field -- `MicroOp` carries only the pointer (see
   * `v4/common/micro-op.scala`'s note by `pvl`: "VL has a single
   * committed-map-table pointer released at commit, not a per-uop stale
   * value"), and `vconfig`'s `VType` carries the vtype CSR bits, not a VL
   * count either. A caller that wants the numeric VL in its trace line
   * (typically whatever just read it off the VL file/busy table) supplies it
   * explicitly here.
   */
  def traceVl(module: String, event: String, uop: MicroOp, vl: UInt, extra: Seq[(String, Bits)] = Nil): Unit =
    trace(module, event, uop, Seq(("pvl", uop.pvl.get), ("vl", vl)) ++ extra)

  /**
   * The element index and EEW, for AGEN/LSU-side trace lines. Both are read
   * straight off the `uop`'s static access descriptor / split cursor
   * (`v_split_idx`, `v_eew`) -- valid once the OP.v has been through the
   * vector LS AGEN, per the same fields' own doc comment in `MicroOp`.
   */
  def traceElem(module: String, event: String, uop: MicroOp, extra: Seq[(String, Bits)] = Nil): Unit =
    trace(module, event, uop, Seq(("eidx", uop.v_split_idx.get), ("eew", uop.v_eew.get)) ++ extra)

  /**
   * The CII tag. Unlike `pvdest`/`pvl`/the element cursor, the CII tag is not
   * a `MicroOp` field (`VecCiiTagEntry` is a separate module's bundle, and
   * this package's dependencies are deliberately limited to `MicroOp` -- see
   * the nlhdl `dependencies` section), so the caller -- which already holds
   * the tag it allocated or matched -- passes it in directly. Accepting a
   * bare `UInt` here does not add a type dependency on the CII tag bundle.
   */
  def traceTag(module: String, event: String, uop: MicroOp, tag: UInt, extra: Seq[(String, Bits)] = Nil): Unit =
    trace(module, event, uop, Seq(("tag", tag)) ++ extra)

  // ---- The decode-stage variant ----
  //
  // THIS EXISTS BECAUSE rob_idx DOES NOT YET EXIST AT DECODE. The ROB entry
  // is allocated at DISPATCH, so VDecode, VLSDecode, VsetDecode and
  // VConfigUnit have no rob_idx to tag a line with. Without this variant a
  // decode-stage caller would either be unable to trace at all or would
  // invent a zero rob_idx, and a line claiming rob=0 is worse than a line
  // admitting it does not know -- it would silently alias with the real ROB
  // entry 0 in every grep. `ftq_idx`/`pc_lob` is the identifier those stages
  // DO have, and it is enough to correlate a decode line with the Whisper
  // trace by PC. Correlating a decode line with a later pipeline line is then
  // a two-step join through the dispatch line, which is the honest cost of
  // the ROB entry not existing yet.
  def traceDecode(module: String, event: String, ftq_idx: UInt, pc_lob: UInt, extra: Seq[(String, Bits)] = Nil): Unit =
    emitLine(module, event, "?", Seq.empty, Seq(("ftq_idx", ftq_idx), ("pc_lob", pc_lob)) ++ extra)

  // ---- The two uOP-less variants, and which to reach for ----
  //
  // Some callers have no `MicroOp` at their boundary. They split into two
  // cases and get one entry point each. Use them in this order -- the first
  // that applies:
  //
  //   1. a MicroOp in scope           -> trace / tracePrn / traceVl /
  //                                       traceElem / traceTag
  //   2. no uOP, but a rob_idx        -> traceId(module, event, rob_idx, extra)
  //   3. no instruction identity      -> traceStruct(module, event, extra)
  //
  // A GROUP-DONE WAKEUP IS THE MOTIVATING CASE FOR `traceId`. It carries a
  // bare `rob_idx` and a member-PRN vector, and no `MicroOp` -- so
  // `VecBusyTable` could not trace its clear event at all, while having the
  // very identifier the line format wants. That is a missing entry point,
  // not a caller problem.

  /**
   * `traceId` emits a real `rob=<rob_idx>` and is therefore fully
   * correlatable; it differs from [[trace]] only in taking the index
   * directly instead of extracting it from a uOP. Reach for it whenever a
   * `rob_idx` is genuinely available, because the ONLY thing that makes
   * [[traceStruct]]'s `rob=?` acceptable is that no honest answer exists --
   * an unnecessary `rob=?` throws away the cross-stage and Whisper
   * correlation this package exists to provide.
   */
  def traceId(module: String, event: String, rob_idx: UInt, extra: Seq[(String, Bits)] = Nil): Unit =
    emitLine(module, event, "%d", Seq(rob_idx), extra)

  // THESE EXIST BECAUSE SOME MODULES HAVE NO uOP AT THEIR BOUNDARY AT ALL,
  // and the original text did not account for them. `trace`/`tracePrn`/
  // `traceVl`/`traceElem`/`traceTag` all take a `MicroOp` to extract
  // `rob_idx`; `traceDecode` covers the decode stage. But `VecMapTable`,
  // `VecFreeList`, `VecBusyTable`, `VecRegFile`, `VecRegFileBank`,
  // `VlRegFile` and `VecGroupReady` are LOOKUP AND STORAGE STRUCTURES, not
  // pipeline stages: their `depends_on` deliberately excludes MicroOp, their
  // ports carry bare addresses and data, and their events are genuinely
  // about a resource, not an instruction -- "PRN 37 freed at commit", "bank
  // 2 forwarded a write to read port 5". There is no uOP in scope to extract
  // a `rob_idx` from, and inventing a port to carry one would add a wire
  // that only tracing reads, which "What it must not become" (see [[trace]]
  // and [[emitLine]] above) forbids.
  //
  // Ground rule 11 of plan v2 requires EVERY vec module to trace, so with
  // only `trace`/`traceId` above, ground rule 11 was unsatisfiable for most
  // of the Phase C module set. Observed across Phases B and C: `VConfigUnit`,
  // `VlRegFile`, `VecFreeList` and `VecBusyTable` omitted trace calls and
  // reported the gap (`VecFreeList` could tag zero of its three lines),
  // while `VecRegFileBank` hand-rolled a raw `printf` behind the public
  // `traceEnabled`. Five nodes, two incompatible workarounds, one missing
  // pair of entry points -- which is why these are named helpers and not a
  // convention.
  //
  // IT DOES NOT RELAX THE rob_idx RULE FOR INSTRUCTION-SCOPED EVENTS. If a
  // module HAS a uOP at its boundary, its instruction events MUST use
  // `trace` or one of its wrappers; reaching for `traceStruct` to avoid
  // threading a uop is a review failure. The test is what the event is
  // ABOUT, not what is convenient to wire: `VecStoreDgenPath` and
  // `VecIssueSlot` hold uOPs and owe real `rob_idx` lines, while a free-list
  // pop owes a `prn` and could not honestly name a `rob_idx` even if one
  // were available, because a group is allocated for one uOP and freed on
  // behalf of another.
  //
  // `emitLine` STAYS PRIVATE and a caller must not hand-roll a `printf`
  // behind `traceEnabled` -- the whole point of this package is that the
  // line format is in one place. `traceEnabled` remains public only for
  // gating a caller's own non-emitting debug logic.

  /**
   * For callers whose event is scoped to a PHYSICAL RESOURCE rather than to
   * an instruction. Emits `rob=?` in the same position as [[traceDecode]],
   * so one grep still finds every line. Takes no identifier argument of its
   * own -- the identifying key (`prn`, `port`, `bank`, `entry`) is the
   * caller's to name in `extra`.
   *
   * `extra` must therefore be non-empty: a structural line with no key
   * identifies nothing. Checked with a Scala `require` -- `extra` is a
   * Scala `Seq` known at elaboration, so this is an elaboration-time check,
   * not a runtime one, and it applies unconditionally (i.e. even if tracing
   * ends up gated off at runtime, a call site that could never carry a key
   * is still a caller bug worth catching at elaboration).
   */
  def traceStruct(module: String, event: String, extra: Seq[(String, Bits)] = Nil): Unit = {
    require(extra.nonEmpty, s"VecTrace.traceStruct($module, $event): extra must be non-empty -- a structural line with no key identifies nothing")
    emitLine(module, event, "?", Seq.empty, extra)
  }
}

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

object VecTrace {

  // ---- The gate ----
  def traceEnabled: Bool =
    PlusArg("vecTrace", 0, "Enable guarded vec-pipeline debug tracing (VecTrace); off by default, and free when off", width = 1).orR

  // ---- The emit primitive ----

  private def emitLine(
    module:  String,
    event:   String,
    robFmt:  String,
    robArgs: Seq[Bits],
    fields:  Seq[(String, Bits)]): Unit = {
    when (traceEnabled && !Module.reset.asBool) {
      // [vec] <module> <event> rob=<rob_idx> <key>=<value> ...
      val fmt = s"[vec] $module $event rob=$robFmt" +
        fields.map { case (k, _) => s" $k=%d" }.mkString + "\n"
      printf(fmt, (robArgs ++ fields.map(_._2)): _*)
    }
  }

  def trace(module: String, event: String, uop: MicroOp, extra: Seq[(String, Bits)] = Nil): Unit =
    emitLine(module, event, "%d", Seq(uop.rob_idx), extra)

  // ---- Convenience wrappers ----

  def tracePrn(module: String, event: String, uop: MicroOp, extra: Seq[(String, Bits)] = Nil): Unit =
    trace(module, event, uop, Seq(("pvdest", uop.pvdest.get.head), ("nmem", uop.v_emul.get)) ++ extra)

  def traceVl(module: String, event: String, uop: MicroOp, vl: UInt, extra: Seq[(String, Bits)] = Nil): Unit =
    trace(module, event, uop, Seq(("pvl", uop.pvl.get), ("vl", vl)) ++ extra)

  def traceElem(module: String, event: String, uop: MicroOp, extra: Seq[(String, Bits)] = Nil): Unit =
    trace(module, event, uop, Seq(("eidx", uop.v_split_idx.get), ("eew", uop.v_eew.get)) ++ extra)

  def traceTag(module: String, event: String, uop: MicroOp, tag: UInt, extra: Seq[(String, Bits)] = Nil): Unit =
    trace(module, event, uop, Seq(("tag", tag)) ++ extra)

  // ---- The decode-stage variant ----
  def traceDecode(module: String, event: String, ftq_idx: UInt, pc_lob: UInt, extra: Seq[(String, Bits)] = Nil): Unit =
    emitLine(module, event, "?", Seq.empty, Seq(("ftq_idx", ftq_idx), ("pc_lob", pc_lob)) ++ extra)

  def traceId(module: String, event: String, rob_idx: UInt, extra: Seq[(String, Bits)] = Nil): Unit =
    emitLine(module, event, "%d", Seq(rob_idx), extra)

  def traceStruct(module: String, event: String, extra: Seq[(String, Bits)] = Nil): Unit = {
    require(extra.nonEmpty, s"VecTrace.traceStruct($module, $event): extra must be non-empty -- a structural line with no key identifies nothing")
    emitLine(module, event, "?", Seq.empty, extra)
  }
}

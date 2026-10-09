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

package boom.v4.vec.generated.cii

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common.{BoomBundle, BoomModule}
import boom.v4.vec.generated.VecTrace

// GENERATED from src/main/nlhdl/vec/cii/VecCiiFlush.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecCiiFlushIO(val nTags: Int)(implicit p: Parameters) extends BoomBundle
{
  val rob_flush           = Input(Bool())
  val rob_flush_kill      = Input(Bool())
  val brupdate_mispredict = Input(Bool())

  //@req-spec-cii.e8
  //@req-spec-cii.e9
  //@req-spec-cii.e10
  val kill_all = Output(Bool())

  val tag_valid  = Input(UInt(nTags.W))
  val tag_killed = Input(UInt(nTags.W))

  val alloc                 = Input(Valid(UInt(ciiTagBits.W)))
  val alloc_br_mask         = Input(UInt(maxBrCount.W))
  val alloc_flush_on_commit = Input(Bool())

  val free = Input(Valid(UInt(ciiTagBits.W)))
}

class VecCiiFlush(
  val nTags:         Int = 16,
  val drainWatchdog: Int = 8192)
  (implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecCiiFlush: elaborated only under usingRVV (never rocket's usingVector)")
  require(nTags == (1 << ciiTagBits),
    s"VecCiiFlush: nTags ($nTags) must equal 1 << ciiTagBits (${1 << ciiTagBits}), matching VecCiiTagTable's nTags")

  val io = IO(new VecCiiFlushIO(nTags))

  //@req-spec-cii.d5
  //@req-spec-cii.e1
  //@req-spec-cii.e2
  //@req-spec-cii.e5
  //@req-spec-cii.e6
  //@req-spec-cii.e26
  //@req-spec-cii.e27
  io.kill_all := io.rob_flush || io.rob_flush_kill

  val allocMask = Mux(io.alloc.valid, UIntToOH(io.alloc.bits, nTags), 0.U(nTags.W))

  //@req-spec-cii.e4
  //@req-spec-cii.e3
  assert(!io.alloc.valid || io.alloc_br_mask === 0.U,
    "VecCiiFlush: alloc_br_mask must be 0 on every allocation -- IQ_V_ALU's past-PNR gate forbids an unresolved older branch")

  //@req-spec-cii.e7
  assert(!(io.kill_all && io.brupdate_mispredict && !io.rob_flush && !io.rob_flush_kill),
    "VecCiiFlush: kill_all must never coincide with brupdate_mispredict unless a flush term is also set")

  //@req-spec-cii.e11
  assert(!io.alloc.valid || !io.alloc_flush_on_commit,
    "VecCiiFlush: alloc_flush_on_commit must never be set on a granted CII op -- it would be its own flush's survivor")

  assert(!io.free.valid || io.tag_valid(io.free.bits),
    "VecCiiFlush: free.valid on a tag whose tag_valid bit is not set (double free)")

  assert(!io.alloc.valid || !io.tag_valid(io.alloc.bits),
    "VecCiiFlush: alloc.valid on a tag whose tag_valid bit is already set (reallocation during drain)")

  assert((io.tag_killed & ~io.tag_valid & ~allocMask) === 0.U,
    "VecCiiFlush: tag_killed set while tag_valid is clear outside the allocation cycle -- validity cleared without clearing kill")

  if (drainWatchdog > 0) {
    val draining  = (io.tag_killed & io.tag_valid).orR
    val drain_cnt = RegInit(0.U(log2Ceil(drainWatchdog + 2).W))

    when (draining) {
      drain_cnt := drain_cnt + 1.U
    } .otherwise {
      drain_cnt := 0.U
    }

    //@req-spec-cii.e12
    assert(drain_cnt <= drainWatchdog.U,
      "VecCiiFlush: a tag has remained killed-and-live past drainWatchdog cycles -- a Src-Data or Writeback beat may have been swallowed")

    when (drain_cnt > drainWatchdog.U) {
      VecTrace.traceStruct("VecCiiFlush", "watchdog_trip", Seq(("tag_killed_and_valid", io.tag_killed & io.tag_valid)))
    }

    when (!draining && drain_cnt =/= 0.U) {
      VecTrace.traceStruct("VecCiiFlush", "drain_done", Seq(("tag_valid", io.tag_valid)))
    }
  }

  when (io.rob_flush) {
    VecTrace.traceStruct("VecCiiFlush", "kill_window", Seq(
      ("tag_valid",      io.tag_valid),
      ("npop",           PopCount(io.tag_valid)),
      ("rob_flush",      io.rob_flush),
      ("rob_flush_kill", io.rob_flush_kill)))
  }
}

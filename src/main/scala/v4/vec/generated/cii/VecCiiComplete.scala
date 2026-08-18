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
import freechips.rocketchip.tile.FPConstants

import boom.v4.common._
import boom.v4.vec.generated.{VecGroupDone, VecRobFlags, VecTrace}

// GENERATED from src/main/nlhdl/vec/cii/VecCiiComplete.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecCiiCompleteIO(implicit p: Parameters) extends BoomBundle
{
  val beat      = Flipped(Valid(new VecCiiWbBeat))
  val wb_lookup = Flipped(new VecCiiWbLookupIO)
  val kill_all  = Input(Bool())

  val group_done = Output(Valid(new VecGroupDone))
  val clr_rob     = Output(Valid(UInt(robAddrSz.W)))
  val rob_flags   = Output(Valid(new VecRobFlags))
  val free_tag    = Output(Valid(UInt(ciiTagBits.W)))
}

class VecCiiComplete(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecCiiComplete: elaborated only under usingRVV (never rocket's usingVector)")

  //@req-spec-rob.g14
  //@req-spec-rob.g15
  //@req-spec-rob.g16
  val io = IO(new VecCiiCompleteIO)

  val numCiiTags = 1 << ciiTagBits

  class FlagAccumEntry extends Bundle
  {
    val fflags = UInt(FPConstants.FLAGS_SZ.W)
    val vxsat  = Bool()
  }

  //@req-spec-cii.g5
  val flagAccum = RegInit(VecInit(Seq.fill(numCiiTags)(0.U.asTypeOf(new FlagAccumEntry))))

  val tag    = io.beat.bits.tag
  val status = io.beat.bits.status

  io.wb_lookup.req.tag           := tag
  io.wb_lookup.req.wb_dst_offset := 0.U

  //@req-spec-cii.g13
  //@req-spec-cii.e18
  //@req-spec-cii.e19
  //@req-spec-cii.e20
  val killed_now = io.kill_all || io.wb_lookup.resp.killed

  //@req-spec-cii.g9
  //@req-spec-cii.g15
  //@req-spec-cii.g23
  val live_last = io.beat.valid && status.last && !killed_now

  val is_vec = status.dst_kind === VecCiiDstKind.VEC

  val entry     = flagAccum(tag)
  val accFflags = Mux(killed_now, entry.fflags, entry.fflags | status.fflags)
  val accVxsat  = Mux(killed_now, entry.vxsat,  entry.vxsat  | status.vxsat)

  when (io.beat.valid) {
    when (status.last) {
      flagAccum(tag).fflags := 0.U
      flagAccum(tag).vxsat  := false.B
    } .otherwise {
      flagAccum(tag).fflags := accFflags
      flagAccum(tag).vxsat  := accVxsat
    }
  }

  //@req-spec-cii.g10
  //@req-spec-cii.g11
  //@req-spec-cii.g12
  //@req-spec-cii.g14
  //@req-spec-issue.d8
  io.group_done.valid := live_last && is_vec
  //@req-spec-cii.i3
  //@req-spec-rob.d18
  //@req-spec-rob.c1
  io.group_done.bits.pvdest  := io.wb_lookup.resp.pvdest_grp
  io.group_done.bits.members := io.wb_lookup.resp.members
  io.group_done.bits.rob_idx := io.wb_lookup.resp.rob_idx
  io.group_done.bits.pvl.valid := false.B
  io.group_done.bits.pvl.bits  := 0.U

  io.clr_rob.valid := live_last && is_vec
  io.clr_rob.bits  := io.wb_lookup.resp.rob_idx

  io.rob_flags.valid       := live_last
  io.rob_flags.bits.rob_idx := io.wb_lookup.resp.rob_idx
  io.rob_flags.bits.fflags  := accFflags
  io.rob_flags.bits.vxsat   := accVxsat

  //@req-spec-cii.e21
  io.free_tag.valid := io.beat.valid && status.last
  io.free_tag.bits  := tag

  when (live_last) {
    VecTrace.traceId("VecCiiComplete", "last_live", io.wb_lookup.resp.rob_idx, Seq(
      ("tag",      tag),
      ("members",  io.wb_lookup.resp.members),
      ("dst_kind", status.dst_kind)))
    VecTrace.traceId("VecCiiComplete", "flags", io.wb_lookup.resp.rob_idx, Seq(
      ("fflags", accFflags),
      ("vxsat",  accVxsat)))
  }

  when (io.beat.valid && status.last && killed_now) {
    VecTrace.traceId("VecCiiComplete", "last_killed", io.wb_lookup.resp.rob_idx, Seq(
      ("tag", tag)))
  }

  when (io.free_tag.valid) {
    VecTrace.traceStruct("VecCiiComplete", "free", Seq(("tag", tag)))
  }
}

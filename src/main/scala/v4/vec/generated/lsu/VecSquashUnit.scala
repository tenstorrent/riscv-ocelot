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

package boom.v4.vec.generated.lsu

import chisel3._
import chisel3.util._
import chisel3.layer

import org.chipsalliance.cde.config.Parameters

import boom.v4.common.{BoomModule, BoomBundle, MicroOp}
import boom.v4.exu.BrUpdateInfo
import boom.v4.lsu.EntryValidFromAge
import boom.v4.util.IsKilledByBranch
import boom.v4.vec.generated.{VecQueueId, VecTrace}
import boom.v4.vec.formal.{BoomSvaLayer, VecSquashUnitChecks}

// GENERATED from src/main/nlhdl/vec/lsu/VecSquashUnit.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecSquashRollbackReq(implicit p: Parameters) extends BoomBundle
{
  val ldq_idx = UInt((1 + ldqAddrSz).W)
  val stq_idx = UInt((1 + stqAddrSz).W)
}

class VecSquashUnitIO(val nQueues: Int, val nKillClients: Int)(implicit p: Parameters) extends BoomBundle
{
  val brupdate       = Input(new BrUpdateInfo)
  val rob_flush      = Input(Bool())
  val rob_flush_kill = Input(Bool())

  val ldq_head        = Input(UInt((1 + ldqAddrSz).W))
  val ldq_tail        = Input(UInt((1 + ldqAddrSz).W))
  val stq_commit_head = Input(UInt((1 + stqAddrSz).W))
  val stq_tail        = Input(UInt((1 + stqAddrSz).W))

  val kill_uop = Input(Vec(nKillClients, Valid(new MicroOp)))

  val resv_rollback_tail = Input(Vec(nQueues, UInt(resvPtrSz.W)))

  val q_squash      = Output(Vec(nQueues, Valid(UInt(resvPtrSz.W))))
  val resv_rollback = Output(Valid(new VecSquashRollbackReq))
  //@req-spec-lsu.f6
  //@req-spec-lsu.f7
  val kill_ldq = Output(UInt(numLdqEntries.W))
  val kill     = Output(Vec(nKillClients, Bool()))
}

class VecSquashUnit(
  val nQueues:      Int = 6,
  val nKillClients: Int = 8
)(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecSquashUnit: elaborates only under usingRVV")

  val queueEnum = Seq(
    VecQueueId.ld_SSI_ADDR_Q, VecQueueId.st_SSI_ADDR_Q, VecQueueId.st_SSI_DATA_Q,
    VecQueueId.ld_US_ADDR_Q,  VecQueueId.st_US_ADDR_Q,  VecQueueId.st_US_DATA_Q
  )
  require(nQueues == queueEnum.length,
    s"VecSquashUnit: nQueues ($nQueues) must equal VecQueueId's enumeration size (${queueEnum.length})")
  require(nKillClients >= 1 && nKillClients <= 16,
    s"VecSquashUnit: nKillClients ($nKillClients) must be in 1..16")

  val io = IO(new VecSquashUnitIO(nQueues, nKillClients))

  //@req-spec-lsu.i1
  val branch = io.brupdate.b2.mispredict && !io.rob_flush_kill
  //@req-spec-memord.e3
  //@req-spec-memord.e4
  //@req-spec-memord.e5
  val flush  = io.rob_flush_kill

  //@req-spec-lsu.i2
  //@req-spec-lsu.i4
  //@req-spec-lsu.i10
  //@req-spec-lsu.i5
  //@req-spec-lsu.i9
  //@req-spec-memord.c16
  //@req-spec-memord.c17
  //@req-spec-memord.c18
  io.resv_rollback.valid := branch || flush
  //@req-spec-memord.c23
  //@req-spec-memord.c24
  io.resv_rollback.bits.ldq_idx := Mux(flush, io.ldq_head, io.brupdate.b2.uop.ldq_idx)
  io.resv_rollback.bits.stq_idx := Mux(flush, io.stq_commit_head, io.brupdate.b2.uop.stq_idx)

  for (q <- 0 until nQueues) {
    io.q_squash(q).valid := branch || flush
    io.q_squash(q).bits  := io.resv_rollback_tail(q)
  }

  //@req-spec-lsu.i7
  //@req-spec-lsu.i8
  //@req-spec-memord.c19
  //@req-spec-memord.c20
  val ldq_branch_kill = VecInit((0 until numLdqEntries).map { i =>
    EntryValidFromAge(io.brupdate.b2.uop.ldq_idx, io.ldq_tail, i.U((1 + ldqAddrSz).W))
  }).asUInt
  io.kill_ldq := Mux(flush, Fill(numLdqEntries, 1.U(1.W)), Mux(branch, ldq_branch_kill, 0.U(numLdqEntries.W)))

  //@req-spec-lsu.i6
  for (c <- 0 until nKillClients) {
    io.kill(c) := io.kill_uop(c).valid && IsKilledByBranch(io.brupdate, io.rob_flush, io.kill_uop(c).bits)
  }

  assert(!io.resv_rollback.valid || (branch || flush),
    "VecSquashUnit: resv_rollback asserted with neither branch nor flush")
  assert(!(branch && flush),
    "VecSquashUnit: branch and flush both driving a squash -- flush must win")
  assert(!flush || io.kill_ldq.andR,
    "VecSquashUnit: flush must kill every LDQ entry")
  for (c <- 0 until nKillClients) {
    assert(!(io.kill(c) && !io.kill_uop(c).valid),
      s"VecSquashUnit: kill($c) asserted with no uop behind it")
  }

  val branchSquashes      = RegInit(0.U(32.W))
  val flushSquashes       = RegInit(0.U(32.W))
  val ldqEntriesReclaimed = RegInit(0.U(32.W))
  when (branch) { branchSquashes := branchSquashes + 1.U }
  when (flush)  { flushSquashes  := flushSquashes  + 1.U }
  when (branch || flush) { ldqEntriesReclaimed := ldqEntriesReclaimed + PopCount(io.kill_ldq) }

  val tailFields = (0 until nQueues).map(q => (s"q$q", io.q_squash(q).bits: Bits))
  when (branch) {
    VecTrace.traceId("VecSquashUnit", "squash_branch", io.brupdate.b2.uop.rob_idx,
      Seq(("ldq_idx", io.resv_rollback.bits.ldq_idx), ("stq_idx", io.resv_rollback.bits.stq_idx)) ++
      tailFields ++ Seq(("kill_ldq_cnt", PopCount(io.kill_ldq))))
  }
  when (flush) {
    VecTrace.traceStruct("VecSquashUnit", "squash_flush",
      Seq(("ldq_idx", io.resv_rollback.bits.ldq_idx), ("stq_idx", io.resv_rollback.bits.stq_idx)) ++
      tailFields ++ Seq(("kill_ldq_cnt", PopCount(io.kill_ldq))))
  }

  //@formal-anchor VecSquashUnitChecks
  layer.block(BoomSvaLayer) {
    VecSquashUnitChecks(
      branch        = branch,
      flush         = flush,
      rollbackLdq   = io.resv_rollback.bits.ldq_idx,
      brLdqIdx      = io.brupdate.b2.uop.ldq_idx,
      killLdq       = io.kill_ldq,
      kill0         = io.kill(0),
      killUop0Valid = io.kill_uop(0).valid
    )
  }
}

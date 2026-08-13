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

package boom.v4.vec.generated.issue

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common.{BoomBundle, BoomModule}
import boom.v4.vec.generated.VecGroupDone

// GENERATED from src/main/nlhdl/vec/issue/VecGroupReady.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecGroupReadyIO(val isMask: Boolean)(implicit p: Parameters) extends BoomBundle
{
  private val groupMembers: Int = if (isMask) 1 else maxVecMembers

  val load = Input(Bool())

  val in_member_rdy = Input(Vec(groupMembers, Bool()))

  //@req-spec-issue.g13
  val prns = Input(Vec(groupMembers, UInt(vecPregSz.W)))

  val members = if (isMask) None else Some(Input(UInt((log2Ceil(maxVecMembers) + 1).W)))

  val used = Input(Bool())

  //@req-spec-rename.g19
  //@req-spec-issue.g12
  //@req-spec-vrf.e8
  val group_done = Input(Vec(vectorParams.numVecWbPorts, Valid(new VecGroupDone)))

  val ready = Output(Bool())

  val out_member_rdy = Output(Vec(groupMembers, Bool()))

}

class VecGroupReady(isMask: Boolean = false)(implicit p: Parameters) extends BoomModule
{
  private val groupMembers: Int = if (isMask) 1 else maxVecMembers

  require(!isMask || groupMembers == 1,
    "VecGroupReady: an isMask instance must elaborate exactly one member lane")

  val io = IO(new VecGroupReadyIO(isMask))

  // ---- What is held, and where ----

  //@req-spec-issue.g14
  val member_rdy = RegInit(VecInit(Seq.fill(groupMembers)(true.B)))

  // ---- The per-member match ----

  //@req-spec-issue.g15
  //@req-spec-issue.g18
  //@req-spec-issue.g33
  val member_hit = Wire(Vec(groupMembers, Bool()))
  for (i <- 0 until groupMembers) {
    member_hit(i) := (for {
      w <- 0 until vectorParams.numVecWbPorts
      j <- 0 until maxVecMembers
    } yield {
      io.group_done(w).valid &&
      (j.U < io.group_done(w).bits.members) &&
      (io.group_done(w).bits.pvdest(j) === io.prns(i))
    }).reduce(_ || _)
  }

  // ---- Next state: load, collapse move and match compose in one cycle ----

  //@req-spec-rename.g20
  //@req-spec-rename.g14
  //@req-spec-issue.g17
  val member_rdy_next = Wire(Vec(groupMembers, Bool()))
  for (i <- 0 until groupMembers) {
    member_rdy_next(i) := Mux(io.load, io.in_member_rdy(i), member_rdy(i)) || member_hit(i)
  }
  member_rdy := member_rdy_next

  //@req-spec-issue.g17
  // The collapse chain pairs this with the slot's out_uop, which is the REGISTER
  // occupant -- so this must be too. Exporting member_rdy_next hands the slot below
  // the INCOMING occupant's readiness while it takes the outgoing occupant's uop.
  for (i <- 0 until groupMembers) {
    io.out_member_rdy(i) := member_rdy(i) || (member_hit(i) && !io.load)
  }

  // ---- AND-reduce to one bit, and conditional participation ----

  //@req-spec-rename.g13
  //@req-spec-issue.g16
  val group_all_rdy: Bool = if (isMask) {
    member_rdy_next(0)
  } else {
    (0 until groupMembers)
      .map(i => member_rdy_next(i) || (i.U >= io.members.get))
      .reduce(_ && _)
  }
  io.ready := !io.used || group_all_rdy

  //@req-spec-vrf.e7
  //@req-spec-vrf.e9

  //@req-spec-issue.g34
  //@req-spec-vrf.e10

  // ---- Reset, assertions ----

  if (!isMask) {
    assert(!io.used || (io.members.get >= 1.U && io.members.get <= maxVecMembers.U),
      "VecGroupReady: members out of range 1..maxVecMembers while the operand is used")
  }

  for (w <- 0 until vectorParams.numVecWbPorts) {
    assert(!io.group_done(w).valid ||
      (io.group_done(w).bits.members >= 1.U && io.group_done(w).bits.members <= maxVecMembers.U),
      "VecGroupReady: group_done member count out of range 1..maxVecMembers on a valid port")
  }
}

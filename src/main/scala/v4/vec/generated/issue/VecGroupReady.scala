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
import boom.v4.vec.generated.{VecGroupDone, VecTrace}

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
  // occupant -- so this must be too. Exporting member_rdy_next would hand the slot
  // below the INCOMING occupant's readiness while it takes the outgoing occupant's
  // uop.
  //
  // `member_hit` is included UNCONDITIONALLY, load cycle or not. `io.prns` names the
  // REGISTER occupant (the slot drives them from `slot_uop`, not from the incoming
  // uop), so a hit is always this occupant's wakeup and the slot below is exactly
  // who needs it. The old `&& !io.load` DROPPED it on a shift cycle: a `group_done`
  // is a ONE-CYCLE PULSE, the outgoing occupant had already left the register, and
  // nobody else was comparing its PRNs -- so that wakeup was lost outright and the
  // op could sit ready-less forever. Including it here is also what lets the
  // consumer of `in_member_rdy` skip comparing against the INCOMING prns: the
  // exporter has already folded this cycle's hit in.
  for (i <- 0 until groupMembers) {
    io.out_member_rdy(i) := member_rdy(i) || member_hit(i)
  }

  // Who woke this member. A member that goes ready with no legitimate producer
  // completion is the shape of an early issue, and the only way to tell a real
  // wakeup from a spurious PRN match is to name the group_done that matched.
  for (i <- 0 until groupMembers) {
    for (w <- 0 until vectorParams.numVecWbPorts) {
      for (j <- 0 until maxVecMembers) {
        val hit = io.group_done(w).valid && (j.U < io.group_done(w).bits.members) &&
                  (io.group_done(w).bits.pvdest(j) === io.prns(i)) && io.used
        when (hit && !member_rdy(i)) {
          VecTrace.traceId("VecGroupReady", "wake", io.group_done(w).bits.rob_idx, Seq(
            ("port", w.U), ("gd_member", j.U), ("my_member", i.U),
            ("prn", io.prns(i)), ("gd_members", io.group_done(w).bits.members),
            ("load", io.load.asUInt)))
        }
      }
    }
  }

  // ---- AND-reduce to one bit, and conditional participation ----

  //@req-spec-rename.g13
  //@req-spec-issue.g16
  // THE ISSUE GATE IS THE REGISTER OCCUPANT'S, AND `member_rdy_next` IS NOT IT.
  // On a load cycle `member_rdy_next` is the INCOMING occupant's readiness, while
  // the slot's `request`/`iss_uop` still name the outgoing one -- so gating on it
  // let a slot grant its occupant on the strength of the NEXT occupant's operands.
  // Measured on conv1d-vector (MegaBoom): `vmacc.vv v10,v14,v12` was granted with
  // `members = 1` (the incoming uop's count) against its own `emul = 2`, which
  // collapsed this AND-reduce for members 1..7, and it read a `pvs3` whose member 0
  // was still busy. `member_rdy | member_hit` is the register occupant's own state,
  // including a wakeup landing this cycle.
  val member_rdy_now = (0 until groupMembers).map(i => member_rdy(i) || member_hit(i))
  val group_all_rdy: Bool = if (isMask) {
    member_rdy_now(0)
  } else {
    (0 until groupMembers)
      .map(i => member_rdy_now(i) || (i.U >= io.members.get))
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

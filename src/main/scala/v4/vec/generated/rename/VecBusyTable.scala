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

package boom.v4.vec.generated.rename

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common.{BoomBundle, BoomModule, MicroOp}
import boom.v4.vec.generated.{VecGroupDone, VecTrace}

// GENERATED from src/main/nlhdl/vec/rename/VecBusyTable.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
class VecBusyResp(val vectorInstance: Boolean)(implicit p: Parameters) extends BoomBundle
{
  //@req-spec-rename.g21
  val pvs1_busy  = if (vectorInstance) Some(Bool()) else None
  //@req-spec-rename.g21
  val pvs2_busy  = if (vectorInstance) Some(Bool()) else None
  //@req-spec-rename.g21
  val pvs3_busy  = if (vectorInstance) Some(Bool()) else None
  //@req-spec-rename.g21
  val pvm_busy   = if (vectorInstance) Some(Bool()) else None
  //@req-spec-rename.g21
  val pvtmp_busy = if (vectorInstance) Some(Bool()) else None
  //@req-spec-rename.h12
  //@req-spec-rename.h13
  //@req-spec-rename.h14
  //@req-spec-rename.g12
  //@req-spec-rename.g26
  val pvl_busy   = if (!vectorInstance) Some(Bool()) else None
}

class VecMemberBusyResp(val maxGroupSize: Int)(implicit p: Parameters) extends BoomBundle
{
  //@req-spec-rename.g11
  val pvs1_busy  = Vec(maxGroupSize, Bool())
  //@req-spec-rename.g11
  val pvs2_busy  = Vec(maxGroupSize, Bool())
  //@req-spec-rename.g11
  val pvs3_busy  = Vec(maxGroupSize, Bool())
  //@req-spec-rename.g11
  val pvtmp_busy = Vec(maxGroupSize, Bool())
  //@req-spec-vrf.d6
  val pvold_busy = Vec(maxGroupSize, Bool())
  //@req-spec-rename.g11
  val pvm_busy   = Bool()
}

class VecBusyTableIO(
  val plWidth:         Int,
  val numPregs:        Int,
  val maxGroupSize:    Int,
  val numWbPorts:      Int,
  val wakeupKind:      String,
  val exportMemberRdy: Boolean)
  (implicit p: Parameters) extends BoomBundle
{
  private val pregSz          = log2Ceil(numPregs)
  private val vectorInstance  = wakeupKind == "group_done"

  //@req-spec-rename.g22
  val ren_uops = Input(Vec(plWidth, new MicroOp))

  //@req-spec-rename.g23
  val rebusy_reqs = Input(Vec(plWidth, Bool()))

  //@req-spec-rename.g21
  val busy_resps = Output(Vec(plWidth, new VecBusyResp(vectorInstance)))

  //@req-spec-rename.g11
  val member_busy_resps =
    if (exportMemberRdy) Some(Output(Vec(plWidth, new VecMemberBusyResp(maxGroupSize)))) else None

  //@req-spec-rename.g24
  //@req-spec-rename.g25
  val wakeups = Input(Vec(numWbPorts, Valid(
    (if (vectorInstance) new VecGroupDone else UInt(pregSz.W)): Data
  )))

  val debug = new Bundle { val busytable = Output(Bits(numPregs.W)) }
}

class VecBusyTable(
  val plWidth:         Int,
  val numPregs:        Int,
  val maxGroupSize:    Int,
  val numWbPorts:      Int,
  val wakeupKind:      String,
  val exportMemberRdy: Boolean = false)
  (implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecBusyTable: elaborated only under usingRVV (never rocket's usingVector)")
  require(wakeupKind == "group_done" || wakeupKind == "ready_bit",
    s"""VecBusyTable: wakeupKind ("$wakeupKind") must be "group_done" or "ready_bit"""")
  require(!exportMemberRdy || maxGroupSize > 1,
    "VecBusyTable: exportMemberRdy requires maxGroupSize > 1 -- a one-member group's " +
    "per-member vector carries nothing the aggregate does not")

  val vectorInstance = wakeupKind == "group_done"
  val pregSz         = log2Ceil(numPregs)

  if (vectorInstance) {
    require(numPregs >= 32 + maxGroupSize,
      s"VecBusyTable: numPregs ($numPregs) must be >= 32 + maxGroupSize ($maxGroupSize) " +
      "on the vector instance, or no full group can ever be renamed")
  }

  val io = IO(new VecBusyTableIO(plWidth, numPregs, maxGroupSize, numWbPorts, wakeupKind, exportMemberRdy))

  // ===========================================================================
  // ---- Wakeup-port extraction (shared by clear-mask and the assertion) ----
  // ===========================================================================

  private def wakeupMemberPrns(w: Int): Seq[UInt] =
    if (vectorInstance) io.wakeups(w).bits.asInstanceOf[VecGroupDone].pvdest
    else Seq(io.wakeups(w).bits.asInstanceOf[UInt])

  private def wakeupMemberCount(w: Int): UInt =
    if (vectorInstance) io.wakeups(w).bits.asInstanceOf[VecGroupDone].members
    else 1.U

  // =========================================================================
  // ---- 1. State ----
  // =========================================================================

  //@req-spec-rename.g1
  //@req-spec-rename.g6
  //@req-spec-rename.g7
  val busy_table = RegInit(0.U(numPregs.W))

  // =========================================================================
  // ---- 2. Set-busy on allocation ----
  // =========================================================================

  //@req-spec-rename.g8
  //@req-spec-rename.g9
  //@req-spec-rename.g10
  //@req-spec-core.h3
  //@req-spec-rename.e7
  //@req-spec-rename.e12
  //@req-spec-vrf.d6
  private def setMaskFor(i: Int): UInt = {
    val uop  = io.ren_uops(i)
    val emul = uop.v_emul.get
    val destMask = (0 until maxGroupSize).map { j =>
      UIntToOH(uop.pvdest.get(j), numPregs) & Fill(numPregs, io.rebusy_reqs(i) && (j.U < emul))
    }.reduce(_ | _)
    val tmpMask = (0 until maxGroupSize).map { j =>
      UIntToOH(uop.pvtmp.get(j), numPregs) &
        Fill(numPregs, io.rebusy_reqs(i) && uop.is_shared.get && (j.U < emul))
    }.reduce(_ | _)
    destMask | tmpMask
  }
  val setMaskOR = (0 until plWidth).map(setMaskFor).reduce(_ | _)

  // =========================================================================
  // ---- 3. Clear-busy on group-done ----
  // =========================================================================

  //@req-spec-rename.g15
  //@req-spec-rename.g18
  //@req-spec-rename.g16
  //@req-spec-rename.g17
  private def clearMaskFor(w: Int): UInt = {
    val prns  = wakeupMemberPrns(w)
    val cnt   = wakeupMemberCount(w)
    val valid = io.wakeups(w).valid
    prns.zipWithIndex.map { case (prn, j) =>
      UIntToOH(prn, numPregs) & Fill(numPregs, valid && (j.U < cnt))
    }.reduce(_ | _)
  }
  val clearMaskOR = (0 until numWbPorts).map(clearMaskFor).reduce(_ | _)

  // =========================================================================
  // ---- 4. Next state, and set beats clear ----
  // =========================================================================

  val busy_table_clr  = busy_table & ~clearMaskOR
  val busy_table_next = busy_table_clr | setMaskOR
  busy_table := busy_table_next

  io.debug.busytable := busy_table

  // =========================================================================
  // ---- 5 / 5b. Source reads, per-operand aggregation, and the per-member
  //              export (D6 / seam review A2) ----
  // =========================================================================

  //@req-spec-rename.h12
  //@req-spec-rename.h13
  //@req-spec-rename.h14
  //@req-spec-rename.g12
  //@req-spec-rename.g26
  for (i <- 0 until plWidth) {
    val uop  = io.ren_uops(i)
    val resp = io.busy_resps(i)

    if (vectorInstance) {
      val emul = uop.v_emul.get

      //@req-spec-rename.g11
      def memberBits(vec: Seq[UInt]): Seq[Bool] =
        (0 until maxGroupSize).map(j => busy_table_clr(vec(j)) && (j.U < emul))

      val pvs1Bits  = memberBits(uop.pvs1.get)
      val pvs2Bits  = memberBits(uop.pvs2.get)
      val pvs3Bits  = memberBits(uop.pvs3.get)
      //@req-spec-core.h3
      //@req-spec-rename.e7
      //@req-spec-rename.e12
      //@req-spec-vrf.d6
      val pvtmpBits = memberBits(uop.pvtmp.get)
      val pvmBit    = busy_table_clr(uop.pvm.get)

      //@req-spec-rename.g11
      resp.pvs1_busy.get  := pvs1Bits.reduce(_ || _)
      resp.pvs2_busy.get  := pvs2Bits.reduce(_ || _)
      resp.pvs3_busy.get  := pvs3Bits.reduce(_ || _)
      resp.pvtmp_busy.get := pvtmpBits.reduce(_ || _)
      resp.pvm_busy.get   := pvmBit

      if (exportMemberRdy) {
        //@req-spec-vrf.d6
        val pvoldBits = memberBits(uop.stale_pvdest.get)
        val mresp = io.member_busy_resps.get(i)
        for (j <- 0 until maxGroupSize) {
          //@req-spec-rename.g11
          mresp.pvs1_busy(j)  := pvs1Bits(j)
          mresp.pvs2_busy(j)  := pvs2Bits(j)
          mresp.pvs3_busy(j)  := pvs3Bits(j)
          mresp.pvtmp_busy(j) := pvtmpBits(j)
          //@req-spec-vrf.d6
          mresp.pvold_busy(j) := pvoldBits(j)
        }
        //@req-spec-rename.g11
        mresp.pvm_busy := pvmBit
      }

      //@req-spec-rename.g11
      VecTrace.trace("VecBusyTable", "read", uop, Seq(
        ("pvs1_busy",  resp.pvs1_busy.get),
        ("pvs2_busy",  resp.pvs2_busy.get),
        ("pvs3_busy",  resp.pvs3_busy.get),
        ("pvtmp_busy", resp.pvtmp_busy.get),
        ("pvm_busy",   resp.pvm_busy.get)))
    } else {
      //@req-spec-rename.h12
      //@req-spec-rename.h13
      //@req-spec-rename.h14
      //@req-spec-rename.g12
      //@req-spec-rename.g26
      resp.pvl_busy.get := busy_table_clr(uop.pvl_src.get)

      VecTrace.trace("VecBusyTable", "read", uop, Seq(("pvl_busy", resp.pvl_busy.get)))
    }
  }

  for (i <- 0 until plWidth) {
    when (io.rebusy_reqs(i)) {
      VecTrace.tracePrn("VecBusyTable", "set", io.ren_uops(i))
    }
  }

  for (w <- 0 until numWbPorts) {
    when (io.wakeups(w).valid) {
      if (vectorInstance) {
        val gd = io.wakeups(w).bits.asInstanceOf[VecGroupDone]
        VecTrace.traceId("VecBusyTable", "clr", gd.rob_idx,
          Seq(("pvdest", gd.pvdest.head), ("nmem", gd.members)))
      } else {
        VecTrace.traceStruct("VecBusyTable", "clr",
          Seq(("prn", io.wakeups(w).bits.asInstanceOf[UInt])))
      }
    }
  }

  // =========================================================================
  // ---- 9. Assertions ----
  // =========================================================================

  if (vectorInstance) {
    for (w <- 0 until numWbPorts) {
      val prns = wakeupMemberPrns(w)
      val cnt  = wakeupMemberCount(w)
      for (j <- 0 until maxGroupSize) {
        when (io.wakeups(w).valid && j.U < cnt) {
          assert(busy_table(prns(j)),
            "VecBusyTable: group-done cleared an already-clear bit (double completion)")
        }
      }
    }
  }

  for (i <- 0 until plWidth) {
    assert(!(io.rebusy_reqs(i) && io.ren_uops(i).v_emul.get === 0.U),
      "VecBusyTable: rebusy_reqs asserted with v_emul == 0")
  }
}

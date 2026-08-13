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

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.exu.BrUpdateInfo
import boom.v4.lsu.GetRealLSQIdx
import boom.v4.util.IsKilledByBranch
import boom.v4.vec.generated.{VecSnoopCandidate, VecSnoopHit, VecLcamSearch, VecLdSearch, VecTrace}

// GENERATED from src/main/nlhdl/vec/lsu/VecCrossLsuSnoop.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecCrossLsuSnoopTier1Row(implicit p: Parameters) extends BoomBundle
{
  val valid     = Bool()
  val is_ssi    = Bool()
  val lo        = UInt((corePAddrBits - 3).W)
  val hi        = UInt((corePAddrBits - 3).W)
  val q_base    = UInt(log2Ceil(ssiQueueEntries).W)
  val q_gen_end = UInt(log2Ceil(ssiQueueEntries + 1).W)
}

class VecCrossLsuSnoopTier2UsRow(implicit p: Parameters) extends BoomBundle
{
  val valid        = Bool()
  val stq_idx      = UInt((1 + stqAddrSz).W)
  val queue_idx    = UInt(log2Ceil(usQueueEntries).W)
  val base         = UInt(corePAddrBits.W)
  val len          = UInt(log2Ceil(maxVecMembers * vecVLen / 8 + 1).W)
  val active_mask  = UInt((maxVecMembers * vecVLen / 8).W)
  val us_data_base = UInt(log2Ceil(usQueueEntries).W)
  val members      = UInt(log2Ceil(maxVecMembers + 1).W)
}

class VecCrossLsuSnoopTier2SsiRow(implicit p: Parameters) extends BoomBundle
{
  val valid       = Bool()
  val stq_idx     = UInt((1 + stqAddrSz).W)
  val ordinal     = UInt(log2Ceil(ssiQueueEntries).W)
  val queue_idx   = UInt(log2Ceil(ssiQueueEntries).W)
  val base        = UInt(corePAddrBits.W)
  val eew         = UInt(2.W)
  val active_mask = UInt((vecELen / 8).W)
}

class VecCrossLsuSnoopIO(val searchPorts: Int)(implicit p: Parameters) extends BoomBundle
{
  val cand           = Vec(searchPorts, Flipped(Decoupled(new VecSnoopCandidate)))
  val lcam           = Output(Vec(searchPorts, Valid(new VecLcamSearch)))

  val ld_search      = Input(Vec(lsuWidth, Valid(new VecLdSearch)))
  val stq_vec_valid  = Input(Vec(numStqEntries, Bool()))

  val snoop_cand     = Output(Vec(lsuWidth, Vec(numStqEntries, Valid(new VecSnoopHit))))
  val vst_addr_match = Output(Vec(lsuWidth, UInt(numStqEntries.W)))

  val stq_alloc      = Input(Vec(coreWidth, Valid(UInt((1 + stqAddrSz).W))))

  val brupdate       = Input(new BrUpdateInfo)
  val rob_flush      = Input(Bool())
}

class VecCrossLsuSnoop(val searchPorts: Int = 1, val ssiSnoopWindow: Int = 16)(implicit p: Parameters)
  extends BoomModule
{
  require(usingRVV, "VecCrossLsuSnoop: elaborates only under usingRVV")
  require(searchPorts >= 1, s"VecCrossLsuSnoop: searchPorts ($searchPorts) must be >= 1")
  require(ssiSnoopWindow >= 0 && ssiSnoopWindow <= ssiQueueEntries,
    s"VecCrossLsuSnoop: ssiSnoopWindow ($ssiSnoopWindow) must be in [0, ssiQueueEntries=$ssiQueueEntries]")

  val maxUsBytes = maxVecMembers * vecVLen / 8
  require(maxUsBytes <= (1 << pgIdxBits),
    s"VecCrossLsuSnoop: maxUsBytes ($maxUsBytes) must fit within one page (${1 << pgIdxBits} bytes) " +
    "so a unit-stride range spans at most two pages (maxRangeSegs = 2)")

  val io = IO(new VecCrossLsuSnoopIO(searchPorts))

  private def dwordBounds(base: UInt, lenBytes: UInt): (UInt, UInt) = {
    val lo = base(corePAddrBits - 1, 3)
    val hi = (base + lenBytes - 1.U)(corePAddrBits - 1, 3)
    (lo, hi)
  }

  //@req-spec-memord.b20
  private def rangesOverlap(lo0: UInt, hi0: UInt, lo1: UInt, hi1: UInt): Bool =
    (lo0 <= hi1) && (lo1 <= hi0)

  //@req-spec-memord.a26
  val t1 = RegInit(VecInit(Seq.fill(numStqEntries)(0.U.asTypeOf(new VecCrossLsuSnoopTier1Row))))

  //@req-spec-memord.a27
  val tier2Us = RegInit(VecInit(Seq.fill(usQueueEntries)(0.U.asTypeOf(new VecCrossLsuSnoopTier2UsRow))))

  val tier2Ssi  = if (ssiSnoopWindow > 0) Some(RegInit(VecInit(Seq.fill(ssiSnoopWindow)(0.U.asTypeOf(new VecCrossLsuSnoopTier2SsiRow))))) else None
  val ssiWinPtr = if (ssiSnoopWindow > 0) Some(RegInit(0.U(log2Ceil(ssiSnoopWindow).W))) else None

  // ---- io.stq_alloc: clear the tier-1 summary and invalidate window/tier-2 entries tagged with it ----
  for (w <- 0 until coreWidth) {
    when (io.stq_alloc(w).valid) {
      val row = GetRealLSQIdx(io.stq_alloc(w).bits)
      t1(row).valid := false.B
      t1(row).lo    := 0.U
      t1(row).hi    := 0.U
    }
  }
  for (j <- 0 until usQueueEntries) {
    when ((0 until coreWidth).map(w => io.stq_alloc(w).valid &&
      GetRealLSQIdx(io.stq_alloc(w).bits) === GetRealLSQIdx(tier2Us(j).stq_idx)).reduce(_ || _)) {
      tier2Us(j).valid := false.B
    }
  }
  tier2Ssi.foreach { win =>
    for (j <- 0 until ssiSnoopWindow) {
      when ((0 until coreWidth).map(w => io.stq_alloc(w).valid &&
        GetRealLSQIdx(io.stq_alloc(w).bits) === GetRealLSQIdx(win(j).stq_idx)).reduce(_ || _)) {
        win(j).valid := false.B
      }
    }
  }

  // ---- Presentation lanes: paragraph 1 (both directions), paragraph 2 (data gate), ----
  // ---- paragraph 3 (US range), paragraph 4 (SSI element), paragraph 5/5b (tier updates) ----
  val ssiStorePresent = Wire(Vec(searchPorts, Bool()))
  val ssiStoreCand     = Wire(Vec(searchPorts, new VecSnoopCandidate))

  for (i <- 0 until searchPorts) {
    val c  = io.cand(i)
    val cb = c.bits

    //@req-spec-memord.a22
    c.ready := !cb.is_store || cb.data_filled

    val fire    = c.valid && c.ready
    val killed  = IsKilledByBranch(io.brupdate, io.rob_flush, cb.uop)
    val present = fire && !killed

    //@req-spec-memord.b1
    //@req-spec-memord.b2
    //@req-spec-memord.b3
    //@req-spec-memord.b4
    //@req-spec-memord.b5
    val (presLo, presHi) = dwordBounds(cb.paddr, Mux(cb.is_unit_stride, cb.len, 1.U))

    //@req-spec-memord.a2
    io.lcam(i).valid                := present
    io.lcam(i).bits.is_store_search := cb.is_store
    io.lcam(i).bits.is_load_search  := !cb.is_store
    io.lcam(i).bits.paddr           := cb.paddr
    io.lcam(i).bits.byte_mask       := cb.active_mask(7, 0)
    //@req-spec-memord.b8
    io.lcam(i).bits.is_range        := cb.is_unit_stride
    io.lcam(i).bits.range_lo        := presLo
    io.lcam(i).bits.range_hi        := presHi
    io.lcam(i).bits.uop             := cb.uop

    assert(!(c.fire && cb.is_store) || cb.data_filled,
      "VecCrossLsuSnoop: an accepted store candidate must have data_filled set")
    assert(!(io.lcam(i).valid && io.lcam(i).bits.is_range) ||
      (io.lcam(i).bits.range_hi >= io.lcam(i).bits.range_lo),
      "VecCrossLsuSnoop: a range presentation must have range_hi >= range_lo")

    //@req-spec-memord.a26
    when (present && cb.is_store) {
      val row       = GetRealLSQIdx(cb.uop.stq_idx)
      val prevValid = t1(row).valid
      t1(row).valid     := true.B
      t1(row).is_ssi    := !cb.is_unit_stride
      t1(row).lo        := Mux(prevValid, Mux(presLo < t1(row).lo, presLo, t1(row).lo), presLo)
      t1(row).hi        := Mux(prevValid, Mux(presHi > t1(row).hi, presHi, t1(row).hi), presHi)
      t1(row).q_base    := cb.q_base
      t1(row).q_gen_end := (cb.queue_idx + 1.U)(log2Ceil(ssiQueueEntries + 1) - 1, 0)
    }

    //@req-spec-memord.a27
    when (present && cb.is_store && cb.is_unit_stride) {
      val e = tier2Us(cb.queue_idx)
      e.valid        := true.B
      e.stq_idx      := cb.uop.stq_idx
      e.queue_idx    := cb.queue_idx
      e.base         := cb.paddr
      e.len          := cb.len
      e.active_mask  := cb.active_mask
      e.us_data_base := cb.us_data_base
      e.members      := cb.members
    }

    ssiStorePresent(i) := present && cb.is_store && !cb.is_unit_stride
    ssiStoreCand(i)     := cb

    //@req-spec-memord.a15
    when (c.valid && cb.is_store && !cb.data_filled) {
      VecTrace.trace("VecCrossLsuSnoop", "data_gate_refused", cb.uop,
        Seq(("lane", i.U), ("is_unit_stride", cb.is_unit_stride), ("queue_idx", cb.queue_idx)))
    }
    when (present) {
      VecTrace.trace("VecCrossLsuSnoop", "present", cb.uop,
        Seq(("lane", i.U), ("is_store", cb.is_store), ("is_unit_stride", cb.is_unit_stride),
            ("lo", presLo), ("hi", presHi)))
    }
  }

  // ---- SSI store window write: at most one wraparound per cycle across all lanes ----
  tier2Ssi.zip(ssiWinPtr).foreach { case (win, ptr) =>
    val prefixCnt = (0 until searchPorts).map(i => PopCount(ssiStorePresent.slice(0, i)))
    for (i <- 0 until searchPorts) {
      val slotRaw = ptr + prefixCnt(i)
      val slot    = Mux(slotRaw >= ssiSnoopWindow.U, slotRaw - ssiSnoopWindow.U, slotRaw)
      when (ssiStorePresent(i)) {
        val e  = win(slot)
        val cb = ssiStoreCand(i)
        e.valid       := true.B
        e.stq_idx     := cb.uop.stq_idx
        e.ordinal     := cb.ordinal
        e.queue_idx   := cb.queue_idx
        e.base        := cb.paddr
        e.eew         := cb.eew
        e.active_mask := cb.active_mask(7, 0)
      }
    }
    val totalCnt = PopCount(ssiStorePresent)
    val nextPtrRaw = ptr + totalCnt
    ptr := Mux(nextPtrRaw >= ssiSnoopWindow.U, nextPtrRaw - ssiSnoopWindow.U, nextPtrRaw)
  }

  // ---- Load-initiated search against the vector store queues: paragraph 5/5b ----
  for (w <- 0 until lsuWidth) {
    val ls = io.ld_search(w)
    val isRange = ls.bits.is_vec && ls.bits.is_unit_stride
    val (lsLo, lsHi) = dwordBounds(
      Mux(isRange, ls.bits.range_base, ls.bits.paddr),
      Mux(isRange, ls.bits.range_len, 1.U))

    //@req-spec-memord.a19
    val t1Hit = Wire(Vec(numStqEntries, Bool()))
    for (i <- 0 until numStqEntries) {
      t1Hit(i) := ls.valid && t1(i).valid && io.stq_vec_valid(i) && ls.bits.stq_age_mask(i) &&
        rangesOverlap(lsLo, lsHi, t1(i).lo, t1(i).hi)
    }
    io.vst_addr_match(w) := t1Hit.asUInt

    val usRowHit = Wire(Vec(numStqEntries, Valid(new VecSnoopHit)))
    for (i <- 0 until numStqEntries) { usRowHit(i) := 0.U.asTypeOf(usRowHit(i)) }
    for (j <- 0 until usQueueEntries) {
      val e = tier2Us(j)
      val (elo, ehi) = dwordBounds(e.base, e.len)
      val hit = ls.valid && e.valid && rangesOverlap(lsLo, lsHi, elo, ehi)
      val row = GetRealLSQIdx(e.stq_idx)
      when (hit) {
        usRowHit(row).valid                  := true.B
        usRowHit(row).bits.is_unit_stride    := true.B
        usRowHit(row).bits.ordinal           := 0.U
        usRowHit(row).bits.queue_idx         := e.queue_idx
        usRowHit(row).bits.paddr       := e.base
        usRowHit(row).bits.active_mask := e.active_mask
        usRowHit(row).bits.us_data_base      := e.us_data_base
        usRowHit(row).bits.members           := e.members
        // Only stores are recorded in tier 2, and only after passing the
        // data_filled gate at presentation, so both are true by construction.
        usRowHit(row).bits.is_store          := true.B
        usRowHit(row).bits.data_filled       := true.B
        usRowHit(row).bits.stq_idx           := e.stq_idx
        usRowHit(row).bits.len               := e.len
        usRowHit(row).bits.eew               := 0.U
      }
    }

    // Youngest-ordinal-wins per STQ row, as a STATIC reduction. The row index is a
    // COMPARISON here, never a dynamic write target: writing ssiRowHit(row) inside a
    // `when` whose condition reads ssiRowHit(row).ordinal is a combinational cycle.
    val ssiRowHit = Wire(Vec(numStqEntries, Valid(new VecSnoopHit)))
    for (i <- 0 until numStqEntries) {
      def entryHit(j: Int): (Bool, VecSnoopHit) = {
        val e = tier2Ssi.get(j)
        val elemLen = 1.U << e.eew
        val (elo, ehi) = dwordBounds(e.base, elemLen)
        val hit = ls.valid && e.valid && rangesOverlap(lsLo, lsHi, elo, ehi) &&
          (GetRealLSQIdx(e.stq_idx) === i.U)
        val cand = Wire(new VecSnoopHit)
        cand.is_unit_stride := false.B
        cand.ordinal        := e.ordinal
        cand.queue_idx      := e.queue_idx
        cand.paddr          := e.base
        cand.active_mask    := e.active_mask
        cand.us_data_base   := 0.U
        cand.members        := 0.U
        cand.is_store       := true.B
        cand.data_filled    := true.B
        cand.stq_idx        := e.stq_idx
        cand.len            := elemLen
        cand.eew            := e.eew
        (hit, cand)
      }
      if (ssiSnoopWindow == 0) {
        ssiRowHit(i) := 0.U.asTypeOf(ssiRowHit(i))
      } else {
        val cands = (0 until ssiSnoopWindow).map(entryHit)
        val picked = cands.tail.foldLeft(cands.head) { case ((accHit, accBits), (h, b)) =>
          val take = h && (!accHit || b.ordinal > accBits.ordinal)
          (accHit || h, Mux(take, b, accBits))
        }
        ssiRowHit(i).valid := picked._1
        ssiRowHit(i).bits  := picked._2
      }
    }


    for (i <- 0 until numStqEntries) {
      val winner = Mux(usRowHit(i).valid, usRowHit(i), ssiRowHit(i))
      io.snoop_cand(w)(i).valid := t1Hit(i) && winner.valid
      io.snoop_cand(w)(i).bits  := winner.bits

      assert(!io.snoop_cand(w)(i).valid || t1(i).valid,
        "VecCrossLsuSnoop: tier-2 candidate reported for an STQ entry with no tier-1 summary bit set")

      when (t1Hit(i)) {
        VecTrace.traceId("VecCrossLsuSnoop", "tier1_match", ls.bits.uop.rob_idx,
          Seq(("lane", w.U), ("stq_idx", i.U), ("tier2_hit", winner.valid)))
      }
    }
  }
}

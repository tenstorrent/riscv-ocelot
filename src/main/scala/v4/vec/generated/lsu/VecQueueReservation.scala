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
import boom.v4.lsu.{GetRealLSQIdx, IsOlderLSU}
import boom.v4.vec.generated.{VecReservation, VecQueueId, VecTrace}
import boom.v4.vec.formal.{BoomSvaLayer, VecQueueReservationChecks}

// GENERATED from src/main/nlhdl/vec/lsu/VecQueueReservation.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecResvReq(implicit p: Parameters) extends BoomBundle
{
  val is_store = Bool()
  val q_idx    = UInt(lsuAddrSz.W)
}

class VecResvUsedCount(implicit p: Parameters) extends BoomBundle
{
  val is_store   = Bool()
  val q_idx      = UInt(lsuAddrSz.W)
  val used_count = Vec(2, UInt(log2Ceil(ssiQueueEntries + 1).W))
}

class VecResvSlotResp(implicit p: Parameters) extends BoomBundle
{
  //@req-spec-lsu.b4
  // WIDTH: `resvPtrSz`, NOT `log2Ceil(ssiQueueEntries)`. A base is a QUEUE
  // POINTER and every pointer in this path carries a WRAP BIT: the reservation's
  // own `allocTail`, VecElemQueue's head/tail, VecElemAgen's `resv_base`, and
  // VecDgen's `data_base` are all `resvPtrSz` (= log2Ceil(entries) + 1). Declared
  // one bit narrow, this bundle silently drops that bit on the way OUT of the
  // module, so an op dispatched once the tail passes `ssiQueueEntries` is handed
  // base 0 while the queue's tail sits at 512+: the agen then enqueues at index 0,
  // outside the occupied region, and `ld_SSI_ADDR_Q`'s "enq idx outside occupied
  // region" assertion fires against a head/tail pair that looks perfectly sane.
  // Measured on `transpose-vector` (MegaBoom): base ran 448, 480, then 0 with the
  // queue at head=448 tail=544.
  // The same truncation reached `region_free`, which uses this bundle to hand the
  // queue the base of a retiring region -- a wrapped retire would have freed the
  // wrong 512-entry half.
  val base  = UInt(resvPtrSz.W)
  val count = UInt(log2Ceil(ssiQueueEntries + 1).W)
}

class VecResvRollbackReq(implicit p: Parameters) extends BoomBundle
{
  val ldq_idx = UInt((1 + ldqAddrSz).W)
  val stq_idx = UInt((1 + stqAddrSz).W)
}

class VecQueueReservationIO(implicit p: Parameters) extends BoomBundle
{
  val dis_uops = Input(Vec(coreWidth, Valid(new MicroOp())))
  val dis_ok   = Output(Vec(coreWidth, Bool()))
  val dis_fire = Input(Vec(coreWidth, Bool()))
  val resv_out = Output(Vec(coreWidth, Vec(2, Valid(new VecReservation()))))

  val resv_lookup = Input(Vec(4, Valid(new VecResvReq())))
  val resv_resp   = Output(Vec(4, Vec(2, new VecResvSlotResp())))

  val release    = Input(Vec(4, Valid(new VecResvUsedCount())))
  val release_ok = Output(Vec(4, Bool()))

  val retire           = Input(Valid(new VecResvReq()))
  val retire_row_valid = Output(Bool())
  val region_free      = Output(Vec(6, Valid(new VecResvSlotResp())))

  val rollback      = Input(Valid(new VecResvRollbackReq()))
  val rollback_tail = Output(Vec(6, UInt(resvPtrSz.W)))

  //@req-spec-lsu.b16
  // THE AGE-WINDOW BASE, NOT THE LSQ HEAD. Every `IsOlderLSU` in this module is
  // taken against this pointer, and that comparison is only defined for indices
  // AT OR AFTER it -- an index before the base reads as the youngest in the ring.
  // So the base must be the oldest row THIS module still owns: VecLsu drives it
  // from its retire walk pointer, which lags the LSQ head whenever more than one
  // load or store commits in a cycle. Driven from the LSQ head instead, the rows
  // in that gap -- committed, unkillable, and still holding queue regions -- are
  // swept as young and dropped, and their entries leak out of `occ`. See VecLsu's
  // note at the `resv.io.ldq_head` hookup for the measurement.
  val ldq_head = Input(UInt((1 + ldqAddrSz).W))
  val stq_head = Input(UInt((1 + stqAddrSz).W))
}

class VecQueueReservation(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecQueueReservation: elaborates only under usingRVV")

  val io = IO(new VecQueueReservationIO)

  import VecQueueId._
  val nQueues  = 6
  val qDepth   = Seq(ssiQueueEntries, ssiQueueEntries, ssiQueueEntries, usQueueEntries, usQueueEntries, usQueueEntries)
  val qSz      = resvPtrSz
  val qCntSz   = log2Ceil(ssiQueueEntries + 1)
  val sumSz    = qCntSz + log2Ceil(coreWidth + 1) + 1

  val vLenBytes = vecVLen / 8

  //@req-spec-issue.b3
  val elemsPerRegInt: Seq[Int] = (0 until 4).map(eew => vLenBytes >> eew)

  //@req-spec-lsu.b18
  //@req-spec-issue.b9
  val ldResvCapTbl = VecInit(elemsPerRegInt.map(e => (vectorParams.ldResvMembers * e).U(qCntSz.W)))

  //@req-spec-issue.b3
  val worstCaseTbl = VecInit((0 until 4).map { eew =>
    VecInit((0 to maxVecMembers).map(et => (elemsPerRegInt(eew) * et).U(qCntSz.W)))
  })

  val emulTotalTbl = VecInit((0 until 16).map { vemul =>
    VecInit((0 until 8).map(nf => (vemul * (nf + 1)).U(qCntSz.W)))
  })

  // ---- Per-queue state: this module is the sole owner (VecElemQueue's own
  // head/tail track it independently, driven by the same claim/free/squash
  // events VecLsu forwards from this module's outputs). ----

  val allocTail = RegInit(VecInit(Seq.fill(nQueues)(0.U(qSz.W))))
  val occ       = RegInit(VecInit(Seq.fill(nQueues)(0.U(qCntSz.W))))
  val freeSpace = VecInit((0 until nQueues).map(q => qDepth(q).U(qCntSz.W) - occ(q)))

  class VecResvSlot extends Bundle {
    val valid = Bool()
    val queue = UInt(ld_SSI_ADDR_Q.getWidth.W)
    val base  = UInt(qSz.W)
    val count = UInt(qCntSz.W)
  }
  // full_idx is the row's OWN ldq_idx/stq_idx as captured at dispatch --
  // already carry-inclusive on MicroOp, so no reconstruction against head is
  // needed to feed IsOlderLSU later. Sized per-direction (idxW) rather than
  // shared, since ldqAddrSz and stqAddrSz need not be equal and IsOlderLSU
  // requires matching operand widths.
  class VecResvRow(val idxW: Int) extends Bundle {
    val valid    = Bool()
    val released = Bool()
    val rob_idx  = UInt(robAddrSz.W)
    val full_idx = UInt(idxW.W)
    val slots    = Vec(2, new VecResvSlot)
  }

  //@req-spec-lsu.b4
  // A region's exclusive end, IN POINTER SPACE. `base + count` is one bit wider
  // than a pointer, so every comparison against a tail must be taken on the
  // truncated value or it disagrees with the hardware the moment a region ends at
  // the wrap point.
  def regionEnd(sl: VecResvSlot): UInt = (sl.base + sl.count)(qSz - 1, 0)

  val ldqTable = RegInit(VecInit(Seq.fill(numLdqEntries)(0.U.asTypeOf(new VecResvRow(1 + ldqAddrSz)))))
  val stqTable = RegInit(VecInit(Seq.fill(numStqEntries)(0.U.asTypeOf(new VecResvRow(1 + stqAddrSz)))))
  def pickRow(isStore: Bool, st: VecResvRow, ld: VecResvRow): VecResvRow = {
    val w  = 1 + (ldqAddrSz max stqAddrSz)
    val sw = Wire(new VecResvRow(w)); sw := st
    val lw = Wire(new VecResvRow(w)); lw := ld
    Mux(isStore, sw, lw)
  }

  // =========================================================================
  // ---- 1/2. Per-lane requested count and queue selection ----
  // =========================================================================

  val laneValid   = Wire(Vec(coreWidth, Bool()))
  val laneUsesStq = Wire(Vec(coreWidth, Bool()))
  val laneQueue   = Wire(Vec(coreWidth, Vec(2, UInt(ld_SSI_ADDR_Q.getWidth.W))))
  val laneCount   = Wire(Vec(coreWidth, Vec(2, UInt(qCntSz.W))))
  val laneSlotReq = Wire(Vec(coreWidth, Vec(2, Bool())))
  val laneCapBound = Wire(Vec(coreWidth, Bool()))

  for (w <- 0 until coreWidth) {
    val u = io.dis_uops(w).bits

    //@req-spec-lsu.d12
    //@req-spec-lsu.d13
    laneValid(w)   := io.dis_uops(w).valid && u.is_vec.get && (u.uses_ldq || u.uses_stq)
    laneUsesStq(w) := u.uses_stq

    val isUSClass = u.v_is_unit_stride.get || u.v_is_whole_reg.get || u.v_is_mask.get

    //@req-spec-issue.c1
    val emulTotalSeg = emulTotalTbl(u.v_emul.get)(u.v_seg_nf.get)
    //@req-spec-issue.b3
    val emulTotal    = Mux(u.v_is_segment.get, emulTotalSeg, u.v_emul.get)
    assert(!laneValid(w) || emulTotal <= maxVecMembers.U,
      "VecQueueReservation: emul_total exceeds maxMembers -- NF*EMUL<=8 violated")

    val worstCase = worstCaseTbl(u.v_eew.get)(emulTotal)
    val ldCap     = ldResvCapTbl(u.v_eew.get)

    //@req-spec-lsu.b19
    //@req-spec-issue.b2
    val storeAddrCount = Mux(isUSClass, 1.U(qCntSz.W), worstCase)
    val storeDataCount = Mux(isUSClass, emulTotal, worstCase)
    assert(!(laneValid(w) && u.uses_stq && !isUSClass) || storeAddrCount === storeDataCount,
      "VecQueueReservation: SSI store address/data counts disagree")

    //@req-spec-lsu.b18
    //@req-spec-issue.b9
    val loadCapped     = !isUSClass && (ldCap < worstCase)
    val loadAddrCount  = Mux(isUSClass, 1.U(qCntSz.W), Mux(loadCapped, ldCap, worstCase))
    laneCapBound(w) := loadCapped

    val loadQueueId      = Mux(isUSClass, ld_US_ADDR_Q, ld_SSI_ADDR_Q)
    val storeAddrQueueId = Mux(isUSClass, st_US_ADDR_Q, st_SSI_ADDR_Q)
    val storeDataQueueId = Mux(isUSClass, st_US_DATA_Q, st_SSI_DATA_Q)

    laneSlotReq(w)(0) := laneValid(w)
    laneQueue(w)(0)   := Mux(u.uses_stq, storeAddrQueueId, loadQueueId)
    laneCount(w)(0)   := Mux(u.uses_stq, storeAddrCount, loadAddrCount)

    //@req-spec-lsu.d12
    //@req-spec-lsu.d13
    laneSlotReq(w)(1) := laneValid(w) && u.uses_stq
    laneQueue(w)(1)   := storeDataQueueId
    laneCount(w)(1)   := storeDataCount
  }

  // =========================================================================
  // ---- 3. Age-ordered grant: per-queue prefix sums, chained across lanes ----
  // =========================================================================

  val reqByQueue = Wire(Vec(nQueues, Vec(coreWidth, UInt(qCntSz.W))))
  for (q <- 0 until nQueues) {
    for (w <- 0 until coreWidth) {
      reqByQueue(q)(w) := (0 until 2).map(k => Mux(laneSlotReq(w)(k) && laneQueue(w)(k) === q.U, laneCount(w)(k), 0.U)).reduce(_ + _)
    }
  }

  //@req-spec-lsu.b4
  //@req-spec-lsu.b5
  val baseByQueue = Wire(Vec(nQueues, Vec(coreWidth + 1, UInt(sumSz.W))))
  for (q <- 0 until nQueues) {
    baseByQueue(q)(0) := 0.U
    for (w <- 0 until coreWidth) {
      baseByQueue(q)(w + 1) := baseByQueue(q)(w) + reqByQueue(q)(w)
    }
  }

  val laneFits = Wire(Vec(coreWidth, Bool()))
  for (w <- 0 until coreWidth) {
    laneFits(w) := (0 until nQueues).map { q =>
      (baseByQueue(q)(w) + reqByQueue(q)(w)) <= freeSpace(q)
    }.reduce(_ && _)
  }

  //@req-spec-issue.b1
  //@req-spec-issue.b2
  val chainOk = Wire(Vec(coreWidth, Bool()))
  chainOk(0) := true.B
  for (w <- 1 until coreWidth) {
    chainOk(w) := chainOk(w - 1) && (!laneValid(w - 1) || laneFits(w - 1))
  }
  for (w <- 0 until coreWidth) {
    io.dis_ok(w) := !laneValid(w) || (laneFits(w) && chainOk(w))
  }

  // =========================================================================
  // ---- resv_out: the granted regions ----
  // =========================================================================

  for (w <- 0 until coreWidth) {
    for (k <- 0 until 2) {
      val q = laneQueue(w)(k)
      io.resv_out(w)(k).valid        := laneSlotReq(w)(k) && io.dis_fire(w)
      io.resv_out(w)(k).bits.queue   := q
      io.resv_out(w)(k).bits.base    := allocTail(q) + baseByQueue(q)(w)
      io.resv_out(w)(k).bits.entries := laneCount(w)(k)
      io.resv_out(w)(k).bits.rob_idx := io.dis_uops(w).bits.rob_idx
      io.resv_out(w)(k).bits.ldq_idx := io.dis_uops(w).bits.ldq_idx
      io.resv_out(w)(k).bits.stq_idx := io.dis_uops(w).bits.stq_idx
    }
    when (laneValid(w) && !io.dis_ok(w)) {
      val blockingQueue = PriorityMux((0 until 2).map { k =>
        val q = laneQueue(w)(k)
        (laneSlotReq(w)(k) && ((baseByQueue(q)(w) + reqByQueue(q)(w)) > freeSpace(q)), q)
      } :+ (true.B, laneQueue(w)(0)))
      VecTrace.trace("VecQueueReservation", "resv_deny", io.dis_uops(w).bits, Seq(("queue", blockingQueue)))
    }
    when (io.dis_fire(w) && laneValid(w)) {
      VecTrace.trace("VecQueueReservation", "resv_grant", io.dis_uops(w).bits, Seq(
        ("q0", laneQueue(w)(0)), ("c0", laneCount(w)(0)),
        ("q1", laneQueue(w)(1)), ("c1", Mux(laneSlotReq(w)(1), laneCount(w)(1), 0.U)),
        ("ld_capped", laneCapBound(w).asUInt)))
    }
  }

  // =========================================================================
  // ---- Dispatch-time table write ----
  // =========================================================================

  for (w <- 0 until coreWidth) {
    val u = io.dis_uops(w).bits
    when (io.dis_fire(w) && laneValid(w) && !u.uses_stq) {
      val idx = GetRealLSQIdx(u.ldq_idx)
      //@req-spec-lsu.b16
      // THE LSQ MUST NOT HAND BACK A SLOT WHOSE RESERVATION HAS NOT RETIRED. The
      // row is cleared at retire or at a rollback kill, so a still-valid row here
      // means the LSQ freed the entry and reallocated it while VecLsu's retire
      // walk -- one row per cycle -- was still behind. The write would overwrite a
      // live region's base with a new one, and the pending retire would then free
      // the WRONG extent. Unreachable while the walk's lag stays under the queue's
      // depth (measured max 2 on spmv_small against 32 entries); asserted rather
      // than assumed, because the failure is silent queue corruption.
      assert(!ldqTable(idx).valid,
        "VecQueueReservation: dispatch reused an ldq slot whose reservation had not retired")
      ldqTable(idx).valid    := true.B
      ldqTable(idx).released := false.B
      ldqTable(idx).rob_idx  := u.rob_idx
      ldqTable(idx).full_idx := u.ldq_idx
      ldqTable(idx).slots(0).valid := true.B
      ldqTable(idx).slots(0).queue := laneQueue(w)(0)
      ldqTable(idx).slots(0).base  := allocTail(laneQueue(w)(0)) + baseByQueue(laneQueue(w)(0))(w)
      ldqTable(idx).slots(0).count := laneCount(w)(0)
      ldqTable(idx).slots(1).valid := false.B
      ldqTable(idx).slots(1).queue := 0.U
      ldqTable(idx).slots(1).base  := 0.U
      ldqTable(idx).slots(1).count := 0.U
    }
    when (io.dis_fire(w) && laneValid(w) && u.uses_stq) {
      val idx = GetRealLSQIdx(u.stq_idx)
      //@req-spec-lsu.b16
      assert(!stqTable(idx).valid,
        "VecQueueReservation: dispatch reused a stq slot whose reservation had not retired")
      stqTable(idx).valid    := true.B
      stqTable(idx).released := false.B
      stqTable(idx).rob_idx  := u.rob_idx
      stqTable(idx).full_idx := u.stq_idx
      for (k <- 0 until 2) {
        stqTable(idx).slots(k).valid := true.B
        stqTable(idx).slots(k).queue := laneQueue(w)(k)
        stqTable(idx).slots(k).base  := allocTail(laneQueue(w)(k)) + baseByQueue(laneQueue(w)(k))(w)
        stqTable(idx).slots(k).count := laneCount(w)(k)
      }
    }
  }

  val fireSum = Wire(Vec(nQueues, UInt(sumSz.W)))
  for (q <- 0 until nQueues) {
    fireSum(q) := (0 until coreWidth).map(w => Mux(io.dis_fire(w), reqByQueue(q)(w), 0.U)).reduce(_ + _)
  }

  // =========================================================================
  // ---- 6. resv_lookup / resv_resp: combinational region echo to the AGENs ----
  // =========================================================================

  for (i <- 0 until 4) {
    val l    = io.resv_lookup(i)
    val row  = pickRow(l.bits.is_store, stqTable(l.bits.q_idx(stqAddrSz - 1, 0)), ldqTable(l.bits.q_idx(ldqAddrSz - 1, 0)))
    for (k <- 0 until 2) {
      val slotLive = l.valid && row.valid && row.slots(k).valid
      io.resv_resp(i)(k).base  := Mux(slotLive, row.slots(k).base, 0.U)
      io.resv_resp(i)(k).count := Mux(slotLive, row.slots(k).count, 0.U)
    }
  }

  // =========================================================================
  // ---- 7. release: the surplus trim, tail-only, denied on collision ----
  // =========================================================================

  val releaseSum = Wire(Vec(nQueues, UInt(sumSz.W)))
  val releaseSumTerms = Wire(Vec(4, Vec(nQueues, UInt(qCntSz.W))))

  for (i <- 0 until 4) {
    val r   = io.release(i)
    val idx = r.bits.q_idx
    val row = pickRow(r.bits.is_store, stqTable(idx(stqAddrSz - 1, 0)), ldqTable(idx(ldqAddrSz - 1, 0)))

    //@req-spec-lsu.b4
    // MODULAR: `regionEnd` truncates to qSz. Every base/tail here is a WRAPPING
    // pointer, so the raw `base + count` is one bit wider and compares unequal to
    // the wrapped tail exactly when the region ends at the wrap point -- the
    // surplus trim would then be silently refused for the rest of the run.
    val slot0Fits = regionEnd(row.slots(0)) === allocTail(row.slots(0).queue)
    val slot1Fits = !row.slots(1).valid || (regionEnd(row.slots(1)) === allocTail(row.slots(1).queue))
    val noCollide0 = fireSum(row.slots(0).queue) === 0.U
    val noCollide1 = !row.slots(1).valid || fireSum(row.slots(1).queue) === 0.U

    //@req-spec-issue.b6
    //@req-spec-issue.b8
    //@req-spec-issue.b6
    // ⚠ NOT DURING A ROLLBACK. The surplus trim is a TAIL-SIDE edit, and a
    // rollback cycle rewrites the tail wholesale from the surviving rows' stored
    // regions -- which are read here BEFORE this cycle's trim. Allowed to fire
    // together, the table row keeps the trimmed `count` while `allocTail` keeps
    // the untrimmed end, and the two never agree again: the next retire of that
    // row frees `used_count` entries out of a region that reserved more, `occ`
    // leaks the difference, and the head drifts below the tail until "retiring
    // row's base is not its queue's head" fires on a LATER, innocent row.
    // Refusing the trim costs only capacity -- the full reservation is freed at
    // retire instead -- and `release_ok` has no consumer that must see it granted.
    val ok = r.valid && row.valid && !row.released && slot0Fits && slot1Fits && noCollide0 && noCollide1 &&
             !io.rollback.valid
    io.release_ok(i) := ok

    for (k <- 0 until 2) {
      assert(!(r.valid && row.valid && row.slots(k).valid) || r.bits.used_count(k) <= row.slots(k).count,
        "VecQueueReservation: release used_count exceeds reserved count")
    }
    assert(!(r.valid && row.valid && r.bits.is_store && row.slots(0).valid && row.slots(1).valid &&
             row.slots(0).queue === st_SSI_ADDR_Q) ||
           r.bits.used_count(0) === r.bits.used_count(1),
      "VecQueueReservation: SSI store release used_count disagrees across slots")

    for (q <- 0 until nQueues) {
      releaseSumTerms(i)(q) := 0.U
      for (k <- 0 until 2) {
        when (ok && row.slots(k).valid && row.slots(k).queue === q.U) {
          releaseSumTerms(i)(q) := row.slots(k).count - r.bits.used_count(k)
        }
      }
    }

    when (ok) {
      when (!r.bits.is_store) {
        val lidx = idx(ldqAddrSz - 1, 0)
        ldqTable(lidx).released := true.B
        ldqTable(lidx).slots(0).count := r.bits.used_count(0)
      } .otherwise {
        val sidx = idx(stqAddrSz - 1, 0)
        stqTable(sidx).released := true.B
        stqTable(sidx).slots(0).count := r.bits.used_count(0)
        stqTable(sidx).slots(1).count := r.bits.used_count(1)
      }
      //@req-spec-issue.b4
      //@req-spec-issue.b5
      //@req-spec-lsu.j6
      VecTrace.traceId("VecQueueReservation", "resv_release", row.rob_idx, Seq(
        ("q_idx", idx), ("is_store", r.bits.is_store.asUInt),
        ("surplus0", row.slots(0).count - r.bits.used_count(0)),
        ("surplus1", Mux(row.slots(1).valid, row.slots(1).count - r.bits.used_count(1), 0.U))))
    }
  }

  for (q <- 0 until nQueues) {
    releaseSum(q) := (0 until 4).map(i => releaseSumTerms(i)(q)).reduce(_ + _)
  }

  // =========================================================================
  // ---- Retire: the only head-side reclamation path ----
  // =========================================================================

  val retireRow = pickRow(io.retire.bits.is_store,
    stqTable(io.retire.bits.q_idx(stqAddrSz - 1, 0)),
    ldqTable(io.retire.bits.q_idx(ldqAddrSz - 1, 0)))

  io.retire_row_valid := retireRow.valid

  val retireSum = Wire(Vec(nQueues, UInt(sumSz.W)))
  for (q <- 0 until nQueues) {
    retireSum(q) := (0 until 2).map(k =>
      Mux(io.retire.valid && retireRow.slots(k).valid && retireRow.slots(k).queue === q.U, retireRow.slots(k).count, 0.U)
    ).reduce(_ + _)
  }

  for (q <- 0 until nQueues) {
    io.region_free(q).valid       := false.B
    io.region_free(q).bits.base  := 0.U
    io.region_free(q).bits.count := 0.U
  }

  when (io.retire.valid) {
    //@req-spec-issue.b7
    assert(retireRow.valid, "VecQueueReservation: retire against an invalid reservation row")
    assert(retireRow.slots(0).base === (allocTail(retireRow.slots(0).queue) - occ(retireRow.slots(0).queue))(qSz - 1, 0),
      "VecQueueReservation: retiring row's base is not its queue's head")

    for (k <- 0 until 2) {
      when (retireRow.slots(k).valid) {
        val q = retireRow.slots(k).queue
        io.region_free(q).valid      := true.B
        io.region_free(q).bits.base  := retireRow.slots(k).base
        io.region_free(q).bits.count := retireRow.slots(k).count
      }
    }
    when (io.retire.bits.is_store) {
      stqTable(io.retire.bits.q_idx(stqAddrSz - 1, 0)).valid := false.B
    } .otherwise {
      ldqTable(io.retire.bits.q_idx(ldqAddrSz - 1, 0)).valid := false.B
    }
    VecTrace.traceId("VecQueueReservation", "resv_retire", retireRow.rob_idx, Seq(
      ("q_idx", io.retire.bits.q_idx), ("is_store", io.retire.bits.is_store.asUInt),
      ("base0", retireRow.slots(0).base), ("count0", retireRow.slots(0).count)))
  }

  // =========================================================================
  // ---- 8. Squash: pointer rollback against the exclusive tail ----
  // =========================================================================

  def headOf(q: UInt): UInt = (allocTail(q) - occ(q))(qSz - 1, 0)

  //@req-spec-lsu.b4
  // "The surviving region ends at or before the new tail" -- STATED MODULARLY.
  // A magnitude compare (`end <= newTail`) is only right while the queue has never
  // wrapped: once the tail passes `ssiQueueEntries` a perfectly ordinary surviving
  // region has an end numerically ABOVE the wrapped tail and the assertion fires on
  // correct hardware. Distance from the new tail is a total order over the live
  // window (`head <= end <= newTail` in modular arithmetic), which is the same
  // idiom VecLsu's US drain-pointer rewind uses for exactly this reason.
  def inRolledBackRegion(end: UInt, head: UInt, newTail: UInt): Bool =
    (newTail - end)(qSz - 1, 0) <= (newTail - head)(qSz - 1, 0)

  // Youngest-strictly-older-than-pivot selection: a single reduction over one
  // direction's table, restricted to rows whose slot 0 queue matches cls.
  def selectYoungestOlder(table: Vec[VecResvRow], n: Int, pivot: UInt, head: UInt, cls: UInt): (Bool, UInt) = {
    val found    = Wire(Vec(n, Bool()))
    val bestIdx  = Wire(Vec(n, UInt(log2Ceil(n).W)))
    val bestFull = Wire(Vec(n, UInt(head.getWidth.W)))
    def isCand(i: Int) = table(i).valid && table(i).slots(0).valid && table(i).slots(0).queue === cls &&
      IsOlderLSU(table(i).full_idx, pivot, head)
    found(0)    := isCand(0)
    bestIdx(0)  := 0.U
    bestFull(0) := table(0).full_idx
    for (i <- 1 until n) {
      val cand  = isCand(i)
      val newer = !found(i - 1) || IsOlderLSU(bestFull(i - 1), table(i).full_idx, head)
      val take  = cand && newer
      found(i)    := found(i - 1) || cand
      bestIdx(i)  := Mux(take, i.U, bestIdx(i - 1))
      bestFull(i) := Mux(take, table(i).full_idx, bestFull(i - 1))
    }
    (found(n - 1), bestIdx(n - 1))
  }

  val rollbackTailNext = Wire(Vec(nQueues, UInt(qSz.W)))
  val rollbackOccNext  = Wire(Vec(nQueues, UInt(qCntSz.W)))
  for (q <- 0 until nQueues) {
    rollbackTailNext(q) := allocTail(q)
    rollbackOccNext(q)  := occ(q)
  }

  when (io.rollback.valid) {
    val ldqPivot = io.rollback.bits.ldq_idx
    val stqPivot = io.rollback.bits.stq_idx

    // The validity sweep is an age comparison per row, independently evaluated
    // -- not a walk that halts at the first scalar (already-invalid) entry.
    for (i <- 0 until numLdqEntries) {
      val kill = ldqTable(i).valid && !IsOlderLSU(ldqTable(i).full_idx, ldqPivot, io.ldq_head)
      when (kill) { ldqTable(i).valid := false.B }
    }
    for (i <- 0 until numStqEntries) {
      val kill = stqTable(i).valid && !IsOlderLSU(stqTable(i).full_idx, stqPivot, io.stq_head)
      when (kill) { stqTable(i).valid := false.B }
    }

    for ((cls, qOut) <- Seq((ld_SSI_ADDR_Q, ld_SSI_ADDR_Q), (ld_US_ADDR_Q, ld_US_ADDR_Q))) {
      val (hit, idx) = selectYoungestOlder(ldqTable, numLdqEntries, ldqPivot, io.ldq_head, cls)
      rollbackTailNext(qOut) := Mux(hit, regionEnd(ldqTable(idx).slots(0)), headOf(qOut))
      rollbackOccNext(qOut)  := occ(qOut) - (allocTail(qOut) - rollbackTailNext(qOut))(qSz - 1, 0)
      for (i <- 0 until numLdqEntries) {
        assert(!(ldqTable(i).valid && ldqTable(i).slots(0).valid && ldqTable(i).slots(0).queue === cls &&
                 IsOlderLSU(ldqTable(i).full_idx, ldqPivot, io.ldq_head)) ||
               inRolledBackRegion(regionEnd(ldqTable(i).slots(0)), headOf(qOut), rollbackTailNext(qOut)),
          "VecQueueReservation: a surviving ldq row's region exceeds the rolled-back tail")
      }
    }

    for ((cls, addrQ, dataQ) <- Seq((st_SSI_ADDR_Q, st_SSI_ADDR_Q, st_SSI_DATA_Q), (st_US_ADDR_Q, st_US_ADDR_Q, st_US_DATA_Q))) {
      val (hit, idx) = selectYoungestOlder(stqTable, numStqEntries, stqPivot, io.stq_head, cls)
      rollbackTailNext(addrQ) := Mux(hit, regionEnd(stqTable(idx).slots(0)), headOf(addrQ))
      rollbackTailNext(dataQ) := Mux(hit, regionEnd(stqTable(idx).slots(1)), headOf(dataQ))
      rollbackOccNext(addrQ)  := occ(addrQ) - (allocTail(addrQ) - rollbackTailNext(addrQ))(qSz - 1, 0)
      rollbackOccNext(dataQ)  := occ(dataQ) - (allocTail(dataQ) - rollbackTailNext(dataQ))(qSz - 1, 0)
      for (i <- 0 until numStqEntries) {
        val surviving = stqTable(i).valid && stqTable(i).slots(0).valid && stqTable(i).slots(0).queue === cls &&
          IsOlderLSU(stqTable(i).full_idx, stqPivot, io.stq_head)
        assert(!surviving || inRolledBackRegion(regionEnd(stqTable(i).slots(0)), headOf(addrQ), rollbackTailNext(addrQ)),
          "VecQueueReservation: a surviving stq row's address region exceeds the rolled-back tail")
        assert(!surviving || inRolledBackRegion(regionEnd(stqTable(i).slots(1)), headOf(dataQ), rollbackTailNext(dataQ)),
          "VecQueueReservation: a surviving stq row's data region exceeds the rolled-back tail")
      }
    }

    for (q <- 0 until nQueues) {
      VecTrace.traceStruct("VecQueueReservation", "resv_rollback", Seq(("queue", q.U), ("new_tail", rollbackTailNext(q))))
    }
    // Per-row rollback verdict: which ldq rows the age sweep judged killed vs
    // surviving, and each row's reserved region. A lost queue entry shows up here
    // as a SURVIVING row whose region ends above the new tail, or as a row killed
    // by the index sweep that the br_mask path still considers live.
    for (i <- 0 until numLdqEntries) {
      when (ldqTable(i).valid) {
        VecTrace.traceId("VecQueueReservation", "rollback_row", ldqTable(i).rob_idx, Seq(
          ("row", i.U), ("full_idx", ldqTable(i).full_idx), ("pivot", ldqPivot),
          ("ldq_head", io.ldq_head), ("released", ldqTable(i).released.asUInt),
          ("q0", ldqTable(i).slots(0).queue), ("base0", ldqTable(i).slots(0).base),
          ("cnt0", ldqTable(i).slots(0).count),
          ("older", IsOlderLSU(ldqTable(i).full_idx, ldqPivot, io.ldq_head).asUInt)))
      }
    }
  }

  val rollbackValidPrev = RegNext(io.rollback.valid, false.B)
  val ldqPivotPrev      = RegInit(0.U((1 + ldqAddrSz).W))
  val stqPivotPrev      = RegInit(0.U((1 + stqAddrSz).W))
  ldqPivotPrev := io.rollback.bits.ldq_idx
  stqPivotPrev := io.rollback.bits.stq_idx
  when (rollbackValidPrev) {
    assert(!ldqTable(GetRealLSQIdx(ldqPivotPrev)).valid, "VecQueueReservation: rollback-driven ldq row still valid")
    assert(!stqTable(GetRealLSQIdx(stqPivotPrev)).valid, "VecQueueReservation: rollback-driven stq row still valid")
  }

  io.rollback_tail := rollbackTailNext

  // =========================================================================
  // ---- 5/9. State update ----
  // =========================================================================

  for (q <- 0 until nQueues) {
    when (io.rollback.valid) {
      //@req-spec-lsu.b14
      //@req-spec-lsu.b15
      // A ROLLBACK IS TAIL-SIDE; A RETIRE IS HEAD-SIDE; THEY ARE INDEPENDENT AND
      // CAN LAND IN THE SAME CYCLE. `rollbackOccNext` accounts only for the tail
      // rewind, so writing it bare DROPS a coincident retire: the region is freed
      // to the queue by `region_free` all the same, but `occ` still counts it, so
      // `headOf` (= allocTail - occ) sits `retireSum` entries behind the queue's
      // real head from then on. The next retire trips "retiring row's base is not
      // its queue's head" -- pointing at a row that is perfectly correct.
      // Measured on `transpose-vector` (MegaBoom) at the first wrap of the load
      // SSI queue: a store retire coincided with the mispredict that rolled the
      // tail to 0, and the following load retire asserted.
      // A retiring row is the OLDEST in its direction, so it can never be the row
      // the age sweep kills, and the two updates cannot describe the same entries.
      // THAT HOLDS ONLY BECAUSE `io.ldq_head`/`io.stq_head` CARRY THE RETIRE WALK
      // POINTER, not the LSQ head: driven from the head, the retiring row sits
      // BEFORE the age base, the sweep reads it as the youngest row in the ring and
      // kills it, and this same-cycle pairing stops being safe. See the port.
      allocTail(q) := rollbackTailNext(q)
      occ(q)       := rollbackOccNext(q) - retireSum(q)
    } .otherwise {
      allocTail(q) := allocTail(q) + fireSum(q) - releaseSum(q)
      //@req-spec-lsu.b14
      //@req-spec-lsu.b15
      occ(q) := occ(q) + fireSum(q) - releaseSum(q) - retireSum(q)
    }
  }

  //@formal-anchor VecQueueReservationChecks
  layer.block(BoomSvaLayer) {
    VecQueueReservationChecks(
      laneValid      = laneValid(0),
      laneUsesStq    = laneUsesStq(0),
      laneAddrCount  = laneCount(0)(0),
      laneDataCount  = laneCount(0)(1),
      resvAddrValid  = io.resv_out(0)(0).valid,
      resvDataValid  = io.resv_out(0)(1).valid,
      retireValid    = io.retire.valid,
      retireRowValid = io.retire_row_valid,
      occSsiAddr     = occ(0),
      ssiDepthLit    = qDepth(0).U,
      ldCapLit       = ldResvCapTbl(io.dis_uops(0).bits.v_eew.get)
    )
  }
}

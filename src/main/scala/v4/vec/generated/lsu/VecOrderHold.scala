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

import boom.v4.common.{BoomModule, BoomBundle}
import boom.v4.lsu.{GetRealLSQIdx, EntryValidFromAge, IsOlderLSU}
import boom.v4.vec.generated.VecTrace

// GENERATED from src/main/nlhdl/vec/lsu/VecOrderHold.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecHoldLdCtx(implicit p: Parameters) extends BoomBundle
{
  val ldq_idx        = UInt((ldqAddrSz + 1).W)
  val rob_idx        = UInt(robAddrSz.W)
  val is_vec         = Bool()
  val is_unit_stride = Bool()
}

class VecHoldStEvent(implicit p: Parameters) extends BoomBundle
{
  val stq_idx        = UInt((stqAddrSz + 1).W)
  val is_unit_stride = Bool()
}

class VecOrderHold(
  val enableOrderHold: Boolean = true,
  // Must equal VecStoreForward's enableVecStoreForward. The two predicates are exact
  // complements only if they agree on whether forwarding covers the US->US case; if
  // forwarding is off and this is true, a US load overlapping a US store neither
  // forwards nor waits and reads stale data.
  val forwardingEnabled: Boolean = true
)(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    val ld_ctx           = Input(Vec(lsuWidth, Valid(new VecHoldLdCtx)))
    val known_overlap    = Input(Vec(lsuWidth, Valid(new VecHoldStEvent)))
    val pred_overlap     = Input(Vec(lsuWidth, Valid(new VecHoldStEvent)))
    val st_drained       = Input(Vec(numStqEntries, Bool()))
    val ldq_valid        = Input(Vec(numLdqEntries, Bool()))
    val ldq_alloc        = Input(Vec(coreWidth, Valid(UInt((ldqAddrSz + 1).W))))
    val stq_head         = Input(UInt((stqAddrSz + 1).W))
    val ldq_next_stq_idx = Input(Vec(numLdqEntries, UInt((stqAddrSz + 1).W)))
    val hold_ldq         = Output(UInt(numLdqEntries.W))
  })

  if (enableOrderHold) {

    val ageCtrW   = 20
    val ageCtrMax = ((1 << ageCtrW) - 1).U((ageCtrW).W)

    //@req-spec-memord.b17
    val hold_valid   = RegInit(VecInit(Seq.fill(numLdqEntries)(false.B)))
    val hold_stq_idx = RegInit(VecInit(Seq.fill(numLdqEntries)(0.U((stqAddrSz + 1).W))))
    val hold_rob_idx = RegInit(VecInit(Seq.fill(numLdqEntries)(0.U(robAddrSz.W))))
    val hold_age_ctr = RegInit(VecInit(Seq.fill(numLdqEntries)(0.U(ageCtrW.W))))

    val ldq_invalid_last = RegNext(VecInit(io.ldq_valid.map(!_)), VecInit(Seq.fill(numLdqEntries)(false.B)))

    val alloc_mask = io.ldq_alloc.map(a =>
      Mux(a.valid, UIntToOH(GetRealLSQIdx(a.bits), numLdqEntries), 0.U(numLdqEntries.W))
    ).reduce(_ | _)

    val base_clear = Wire(Vec(numLdqEntries, Bool()))
    val release    = Wire(Vec(numLdqEntries, Bool()))

    for (i <- 0 until numLdqEntries) {
      //@req-spec-memord.b19
      release(i)    := hold_valid(i) && io.st_drained(GetRealLSQIdx(hold_stq_idx(i)))
      base_clear(i) := !io.ldq_valid(i) || alloc_mask(i) || release(i)

      when (base_clear(i)) {
        hold_valid(i) := false.B
      } .elsewhen (hold_valid(i) && hold_age_ctr(i) =/= ageCtrMax) {
        hold_age_ctr(i) := hold_age_ctr(i) + 1.U
      }

      when (hold_valid(i) && alloc_mask(i)) {
        VecTrace.traceId("VecOrderHold", "cleared_realloc", hold_rob_idx(i), Seq(
          ("ldq_idx", i.U), ("stq_idx", hold_stq_idx(i))))
      }
      when (release(i)) {
        VecTrace.traceId("VecOrderHold", "released", hold_rob_idx(i), Seq(
          ("ldq_idx", i.U), ("stq_idx", hold_stq_idx(i)), ("elapsed", hold_age_ctr(i))))
      }

      assert(!(hold_valid(i) && ldq_invalid_last(i)),
        "VecOrderHold: hold_valid held on an LDQ entry that was already invalid last cycle")
      assert(!(hold_valid(i) && hold_age_ctr(i) === ageCtrMax),
        "VecOrderHold: hold has not released for an excessive number of cycles (liveness tripwire)")
    }

    for (w <- 0 until lsuWidth) {
      val ld     = io.ld_ctx(w)
      val k_ev   = io.known_overlap(w)
      val p_ev   = io.pred_overlap(w)
      val target = GetRealLSQIdx(ld.bits.ldq_idx)
      val liveOk = io.ldq_valid(target)

      //@req-spec-memord.b11
      val k_eligible = ld.valid && k_ev.valid && ld.bits.is_vec &&
        (if (forwardingEnabled) !(ld.bits.is_unit_stride && k_ev.bits.is_unit_stride) else true.B)
      val k_base   = k_eligible && !io.st_drained(GetRealLSQIdx(k_ev.bits.stq_idx)) && liveOk
      val k_ageOk  = EntryValidFromAge(io.stq_head, io.ldq_next_stq_idx(target), k_ev.bits.stq_idx)
      val k_admit  = k_base && k_ageOk
      val k_ageFail = k_base && !k_ageOk

      //@req-spec-memord.b16
      val p_eligible = ld.valid && p_ev.valid && ld.bits.is_vec &&
        (if (forwardingEnabled) !(ld.bits.is_unit_stride && p_ev.bits.is_unit_stride) else true.B)
      val p_base   = p_eligible && !io.st_drained(GetRealLSQIdx(p_ev.bits.stq_idx)) && liveOk
      val p_ageOk  = EntryValidFromAge(io.stq_head, io.ldq_next_stq_idx(target), p_ev.bits.stq_idx)
      val p_admit  = p_base && p_ageOk
      val p_ageFail = p_base && !p_ageOk

      val both_admit   = k_admit && p_admit
      val p_is_younger = IsOlderLSU(k_ev.bits.stq_idx, p_ev.bits.stq_idx, io.stq_head)
      val new_valid    = k_admit || p_admit
      val new_is_known = Mux(both_admit, !p_is_younger, k_admit)
      val new_stq = Mux(both_admit, Mux(p_is_younger, p_ev.bits.stq_idx, k_ev.bits.stq_idx),
                         Mux(k_admit, k_ev.bits.stq_idx, p_ev.bits.stq_idx))

      val overwrite_ok = !hold_valid(target) || IsOlderLSU(hold_stq_idx(target), new_stq, io.stq_head)
      val installed     = new_valid && !base_clear(target) && overwrite_ok

      when (installed) {
        hold_valid(target)   := true.B
        hold_stq_idx(target) := new_stq
        hold_rob_idx(target) := ld.bits.rob_idx
        hold_age_ctr(target) := 0.U
      }

      assert(!(k_admit && ld.bits.is_unit_stride && k_ev.bits.is_unit_stride),
        "VecOrderHold: admitted a known US/US pair -- forward/hold predicates have drifted out of complement")
      assert(!(p_admit && ld.bits.is_unit_stride && p_ev.bits.is_unit_stride),
        "VecOrderHold: admitted a predicted US/US pair -- forward/hold predicates have drifted out of complement")
      assert(!k_ageFail,
        "VecOrderHold: a known_overlap event failed the age test -- this is a search bug, not a mispredict")

      when (installed && new_is_known) {
        VecTrace.traceId("VecOrderHold", "admit_known", ld.bits.rob_idx, Seq(
          ("ldq_idx", target), ("stq_idx", new_stq)))
      }
      when (installed && !new_is_known) {
        VecTrace.traceId("VecOrderHold", "admit_pred", ld.bits.rob_idx, Seq(
          ("ldq_idx", target), ("stq_idx", new_stq)))
      }
      when (k_ageFail) {
        VecTrace.traceId("VecOrderHold", "dropped_age", ld.bits.rob_idx, Seq(
          ("ldq_idx", target), ("stq_idx", k_ev.bits.stq_idx)))
      }
      when (p_ageFail) {
        VecTrace.traceId("VecOrderHold", "dropped_age", ld.bits.rob_idx, Seq(
          ("ldq_idx", target), ("stq_idx", p_ev.bits.stq_idx)))
      }
    }

    //@req-spec-memord.b12
    //@req-spec-memord.b13
    io.hold_ldq := VecInit((0 until numLdqEntries).map(i => hold_valid(i) && io.ldq_valid(i))).asUInt

  } else {
    io.hold_ldq := 0.U
  }
}

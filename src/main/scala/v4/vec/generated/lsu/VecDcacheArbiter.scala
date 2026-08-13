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

import boom.v4.common.{BoomModule, BoomBundle}
import boom.v4.lsu.GetRealLSQIdx
import boom.v4.vec.generated.{VecMemAccess, VecTrace}
import boom.v4.vec.formal.{BoomSvaLayer, VecDcacheArbiterChecks}

// GENERATED from src/main/nlhdl/vec/lsu/VecDcacheArbiter.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class LsuResourceClaim(implicit p: Parameters) extends BoomBundle
{
  val tlb    = Bool()
  val dcache = Bool()
  val lcam   = Bool()
}

class VecDcacheArbiterIO(implicit p: Parameters) extends BoomBundle
{
  //@req-spec-lsu.h1
  val ld_req = Vec(lsuWidth, Flipped(Decoupled(new VecMemAccess)))
  val st_req = Vec(lsuWidth, Flipped(Decoupled(new VecMemAccess)))

  val scalar_demand  = Input(Vec(lsuWidth, new LsuResourceClaim))
  val scalar_avail   = Input(Vec(lsuWidth, new LsuResourceClaim))
  val vec_claim      = Output(Vec(lsuWidth, new LsuResourceClaim))
  val vec_fire       = Output(Vec(lsuWidth, Valid(new VecMemAccess)))
  val dmem_req_ready = Input(Vec(lsuWidth, Bool()))

  val lcbCreditSz    = log2Ceil(lcbEntries + 1)
  val lcb_free_count = Input(UInt(lcbCreditSz.W))
  val hold_ldq       = Input(UInt(numLdqEntries.W))
}

class VecDcacheArbiter(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecDcacheArbiter: elaborates only under usingRVV")
  require(
    (dcacheArbiterMode == "single" && lsuWidth == 1) ||
    (dcacheArbiterMode == "dual-dynamic" && lsuWidth == 2),
    s"VecDcacheArbiter: dcacheArbiterMode ($dcacheArbiterMode) must agree with lsuWidth ($lsuWidth)")

  val io = IO(new VecDcacheArbiterIO)

  val rrPhases = 4
  val rrPtrSz  = log2Ceil(rrPhases)

  //@req-spec-lsu.h3
  //@req-spec-lsu.h6
  val rr_ptr = RegInit(0.U(rrPtrSz.W))
  rr_ptr := rr_ptr + 1.U

  //@req-spec-lsu.h10
  //@req-spec-lsu.h11
  val blocked_ld = RegInit(false.B)
  val blocked_st = RegInit(false.B)

  //@req-spec-lsu.h4
  //@req-spec-lsu.h5
  val elevateLoad  = (rr_ptr === 1.U) && blocked_ld
  val elevateStore = (rr_ptr === 2.U) && blocked_st

  // `elevated` is a SCALA Boolean, not a Bool: Chisel's `||` builds hardware rather than
  // short-circuiting, so `elevated || avail.x` keeps avail in the cone even when elevated
  // is true.B -- and vec_claim's cone must be avail-free or the LSU seam forms a cycle.
  def resourceOk(bits: VecMemAccess, avail: LsuResourceClaim, dmemReady: Bool, elevated: Boolean): Bool = {
    val tlbOk    = if (elevated) true.B else !bits.uses_tlb || avail.tlb
    val dcacheOk = if (elevated) !bits.uses_dcache || dmemReady
                   else          !bits.uses_dcache || (avail.dcache && dmemReady)
    val lcamOk   = if (elevated) true.B else !bits.uses_lcam || avail.lcam
    tlbOk && dcacheOk && lcamOk
  }

  val ldSuppressedHold   = Wire(Vec(lsuWidth, Bool()))
  val ldSuppressedCredit = Wire(Vec(lsuWidth, Bool()))
  val ldEligible         = Wire(Vec(lsuWidth, Bool()))
  val stEligible         = Wire(Vec(lsuWidth, Bool()))
  val ldWins             = Wire(Vec(lsuWidth, Bool()))
  val stWins             = Wire(Vec(lsuWidth, Bool()))
  val ldWinsElevated     = Wire(Vec(lsuWidth, Bool()))
  val stWinsElevated     = Wire(Vec(lsuWidth, Bool()))

  //@req-spec-lsu.h7
  //@req-spec-lsu.h8
  for (w <- 0 until lsuWidth) {
    //@req-spec-memord.b18
    ldSuppressedHold(w) := io.hold_ldq(GetRealLSQIdx(io.ld_req(w).bits.uop.ldq_idx))

    // NOT gated on lcb_free_count: a beat always lands in an entry its op allocated
    // at launch, and VecBeatExpander already applies the exact per-PRN lcb_alloc_rdy
    // test. Suppressing on the coarse credit DEADLOCKS at EMUL = lcbEntries, where one
    // group owns every entry and free_count is 0 for the op's whole lifetime.
    ldSuppressedCredit(w) := false.B

    ldEligible(w) := io.ld_req(w).valid && !ldSuppressedHold(w) && !ldSuppressedCredit(w)
    stEligible(w) := io.st_req(w).valid

    val ldOkElevated = ldEligible(w) &&
      resourceOk(io.ld_req(w).bits, io.scalar_avail(w), io.dmem_req_ready(w), true)
    val stOkFallback = stEligible(w) &&
      resourceOk(io.st_req(w).bits, io.scalar_avail(w), io.dmem_req_ready(w), false) && !ldOkElevated

    val stOkElevated = stEligible(w) &&
      resourceOk(io.st_req(w).bits, io.scalar_avail(w), io.dmem_req_ready(w), true)
    val ldOkFallback = ldEligible(w) &&
      resourceOk(io.ld_req(w).bits, io.scalar_avail(w), io.dmem_req_ready(w), false) && !stOkElevated

    val ldOkDefault = ldEligible(w) &&
      resourceOk(io.ld_req(w).bits, io.scalar_avail(w), io.dmem_req_ready(w), false)
    val stOkDefault = stEligible(w) &&
      resourceOk(io.st_req(w).bits, io.scalar_avail(w), io.dmem_req_ready(w), false) && !ldOkDefault

    ldWins(w) := Mux(elevateLoad, ldOkElevated, Mux(elevateStore, ldOkFallback, ldOkDefault))
    stWins(w) := Mux(elevateLoad, stOkFallback, Mux(elevateStore, stOkElevated, stOkDefault))
    ldWinsElevated(w) := elevateLoad  && ldOkElevated
    stWinsElevated(w) := elevateStore && stOkElevated
  }

  val ldValidAny = io.ld_req.map(_.valid).reduce(_ || _)
  val ldGrantAny = ldWins.reduce(_ || _)
  val stValidAny = io.st_req.map(_.valid).reduce(_ || _)
  val stGrantAny = stWins.reduce(_ || _)

  when (ldGrantAny)      { blocked_ld := false.B }
    .elsewhen (ldValidAny) { blocked_ld := true.B }

  when (stGrantAny)      { blocked_st := false.B }
    .elsewhen (stValidAny) { blocked_st := true.B }

  for (w <- 0 until lsuWidth) {
    io.ld_req(w).ready   := ldWins(w)
    io.st_req(w).ready   := stWins(w)
    io.vec_fire(w).valid := ldWins(w) || stWins(w)
    io.vec_fire(w).bits  := Mux(ldWins(w), io.ld_req(w).bits, io.st_req(w).bits)

    // vec_claim's cone must EXCLUDE scalar_avail: the LSU clears *_avail from it, so
    // reading avail here closes scalar_avail -> vec_claim -> scalar_avail across the
    // module boundary. Select the bits from the ELEVATED decision only (which is
    // unconditional by construction), never from vec_fire, whose winner is avail-gated.
    val elevatedGrant = ldWinsElevated(w) || stWinsElevated(w)
    val elevWinBits   = Mux(ldWinsElevated(w), io.ld_req(w).bits, io.st_req(w).bits)

    //@req-spec-lsu.h9
    //@req-spec-memord.b9
    io.vec_claim(w).tlb    := elevatedGrant && elevWinBits.uses_tlb
    io.vec_claim(w).dcache := elevatedGrant && elevWinBits.uses_dcache
    io.vec_claim(w).lcam   := elevatedGrant && elevWinBits.uses_lcam
  }

  for (w <- 0 until lsuWidth) {
    assert(!(io.ld_req(w).ready && io.st_req(w).ready),
      "VecDcacheArbiter: lane granted to both load and store drains")
    assert(!(ldWins(w) && !ldWinsElevated(w) && io.ld_req(w).bits.uses_tlb) || io.scalar_avail(w).tlb,
      "VecDcacheArbiter: default-path load grant claimed a TLB port scalar_avail said was gone")
    assert(!(ldWins(w) && !ldWinsElevated(w) && io.ld_req(w).bits.uses_lcam) || io.scalar_avail(w).lcam,
      "VecDcacheArbiter: default-path load grant claimed an LCAM port scalar_avail said was gone")
    assert(!(ldWins(w) && !ldWinsElevated(w) && io.ld_req(w).bits.uses_dcache) || io.scalar_avail(w).dcache,
      "VecDcacheArbiter: default-path load grant claimed a D$ port scalar_avail said was gone")
    assert(!(stWins(w) && !stWinsElevated(w) && io.st_req(w).bits.uses_tlb) || io.scalar_avail(w).tlb,
      "VecDcacheArbiter: default-path store grant claimed a TLB port scalar_avail said was gone")
    assert(!(stWins(w) && !stWinsElevated(w) && io.st_req(w).bits.uses_lcam) || io.scalar_avail(w).lcam,
      "VecDcacheArbiter: default-path store grant claimed an LCAM port scalar_avail said was gone")
    assert(!(stWins(w) && !stWinsElevated(w) && io.st_req(w).bits.uses_dcache) || io.scalar_avail(w).dcache,
      "VecDcacheArbiter: default-path store grant claimed a D$ port scalar_avail said was gone")
    assert((ldWins(w) || stWins(w)) === io.vec_fire(w).valid,
      "VecDcacheArbiter: a granted lane must present io.vec_fire.valid")
    assert(!(ldWins(w) && ldSuppressedHold(w)),
      "VecDcacheArbiter: granted a load whose hold_ldq bit is set")
    val elevatedGrant = ldWinsElevated(w) || stWinsElevated(w)
    assert(elevatedGrant || (!io.vec_claim(w).tlb && !io.vec_claim(w).dcache && !io.vec_claim(w).lcam),
      "VecDcacheArbiter: vec_claim asserted outside an elevation grant")
  }

  def contended(bits: VecMemAccess, demand: LsuResourceClaim): Bool =
    (bits.uses_tlb && demand.tlb) || (bits.uses_dcache && demand.dcache) || (bits.uses_lcam && demand.lcam)

  for (w <- 0 until lsuWidth) {
    when (ldWins(w)) {
      VecTrace.trace("VecDcacheArbiter", "grant", io.ld_req(w).bits.uop, Seq(
        ("lane", w.U), ("requestor", 0.U),
        ("phase", rr_ptr),
        ("uses_tlb", io.ld_req(w).bits.uses_tlb.asUInt),
        ("uses_dcache", io.ld_req(w).bits.uses_dcache.asUInt),
        ("uses_lcam", io.ld_req(w).bits.uses_lcam.asUInt),
        ("contended", contended(io.ld_req(w).bits, io.scalar_demand(w)).asUInt)))
      when (ldWinsElevated(w)) {
        VecTrace.trace("VecDcacheArbiter", "elevation", io.ld_req(w).bits.uop, Seq(
          ("requestor", 0.U), ("phase", rr_ptr)))
      }
    }
    when (stWins(w)) {
      VecTrace.trace("VecDcacheArbiter", "grant", io.st_req(w).bits.uop, Seq(
        ("lane", w.U), ("requestor", 1.U),
        ("phase", rr_ptr),
        ("uses_tlb", io.st_req(w).bits.uses_tlb.asUInt),
        ("uses_dcache", io.st_req(w).bits.uses_dcache.asUInt),
        ("uses_lcam", io.st_req(w).bits.uses_lcam.asUInt),
        ("contended", contended(io.st_req(w).bits, io.scalar_demand(w)).asUInt)))
      when (stWinsElevated(w)) {
        VecTrace.trace("VecDcacheArbiter", "elevation", io.st_req(w).bits.uop, Seq(
          ("requestor", 1.U), ("phase", rr_ptr)))
      }
    }
  }

  //@formal-anchor VecDcacheArbiterChecks
  layer.block(BoomSvaLayer) {
    VecDcacheArbiterChecks(
      ldReqValid   = io.ld_req(0).valid,
      ldReqDcache  = io.ld_req(0).bits.uses_dcache,
      ldWins       = ldWins(0),
      stWins       = stWins(0),
      ldWinsElev   = ldWinsElevated(0),
      stWinsElev   = stWinsElevated(0),
      ldEligible   = ldEligible(0),
      vecFireValid = io.vec_fire(0).valid,
      claimTlb     = io.vec_claim(0).tlb,
      claimDcache  = io.vec_claim(0).dcache,
      claimLcam    = io.vec_claim(0).lcam,
      availDcache  = io.scalar_avail(0).dcache,
      rrPtr        = rr_ptr,
      blockedLd    = blocked_ld,
      dmemReady    = io.dmem_req_ready(0),
      lcbFree      = io.lcb_free_count
    )
  }
}

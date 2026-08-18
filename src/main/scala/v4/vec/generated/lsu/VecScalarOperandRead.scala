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
import boom.v4.util.{GetNewBrMask, IsKilledByBranch}
import boom.v4.vec.generated.{IntWbSnoop, VecTrace}

// GENERATED from src/main/nlhdl/vec/lsu/VecScalarOperandRead.nlhdl.scala. Do
// not hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecScalarOperands(implicit p: Parameters) extends BoomBundle
{
  val uop          = new MicroOp
  val base         = UInt(xLen.W)
  val stride       = UInt(xLen.W)
  val scalar_data  = UInt(xLen.W)
  val vl           = UInt(vecVLSz.W)
  val vl_zero      = Bool()
}

class VecScalarOperandReadIO(implicit p: Parameters) extends BoomBundle
{
  val iss        = Input(Valid(new MicroOp))
  val brupdate   = Input(new BrUpdateInfo)
  val rob_flush  = Input(Bool())

  val int_rf_read_req = Vec(2, Decoupled(UInt(ipregSz.W)))
  val int_rf_read_rsp = Input(Vec(2, UInt(xLen.W)))
  val int_wb_snoop    = Input(Vec(numIrfWritePorts, Valid(new IntWbSnoop)))

  val vl_read_addr = Output(UInt(vlPregSz.W))
  val vl_read_data = Input(UInt(vecVLSz.W))

  val out = Output(Valid(new VecScalarOperands))
}

class VecScalarOperandRead(
  val isStore:        Boolean = false,
  val wbWaitWatchdog: Int     = 4096)
  (implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecScalarOperandRead: elaborates only under usingRVV")

  val io = IO(new VecScalarOperandReadIO)

  //@req-spec-issue.h3
  val rr_valid       = RegInit(false.B)
  val rr_need        = RegInit(VecInit(Seq.fill(2)(false.B)))
  val rr_uop         = Reg(new MicroOp)
  val rr_data        = Reg(Vec(2, UInt(xLen.W)))
  val rr_vl          = Reg(UInt(vecVLSz.W))
  val rr_held_cycles = Reg(UInt(8.W))

  val killedNow = IsKilledByBranch(io.brupdate, io.rob_flush, rr_uop)

  //@req-spec-issue.d3
  val heldAddr = Seq(rr_uop.prs1, rr_uop.prs2)
  val grantAddr = Seq(io.iss.bits.prs1, io.iss.bits.prs2)
  val heldActive = VecInit((0 until 2).map(n => rr_valid && rr_need(n)))

  //@req-spec-issue.d3
  // DOES THIS LANE ACTUALLY HAVE AN INT SOURCE? Held form, valid for the whole
  // life of the op, as against `grantRtypeFix`'s grant-cycle form.
  //
  // EVERY property in this module that reasons about "our producer" MUST be
  // qualified by this. A unit-stride access uses prs1 only; lane 1's `prs2` is
  // then a LEFTOVER RENAME NUMBER that this op does not use and therefore does
  // not hold. An unused PRN can be freed and reallocated while we are still
  // live, and an unrelated instruction will legitimately write it -- so any
  // property of the form "a later write to this PRN means our producer wrote
  // late" is FALSE on such a lane and fires on correct behaviour.
  val heldRtypeFix = Seq(rr_uop.lrs1_rtype === RT_FIX, rr_uop.lrs2_rtype === RT_FIX)

  //@req-spec-issue.d3
  // WAIT-FOR-WRITEBACK ON A BYPASSABLE INT WAKEUP.
  //
  // A BYPASSABLE wakeup fires BEFORE the producer's integer regfile write, so
  // both the grant-cycle read AND its response-cycle snoop forward MISS: the
  // write is presented on int_wb_snoop at grant+2 (measured on ms2p5_loadblock).
  // iq_v_load and iq_v_store are NOT past-PNR gated (unlike iq_v_alu, which is
  // why the structurally identical single-cycle forward in VecCiiIssue is safe),
  // so this window is real on the INT network reaching this module. Hold the
  // lane until the awaited write is OBSERVED on int_wb_snoop -- wait on the
  // EVENT, never on a constant that bakes a writeback depth into this path.
  //
  // WHY A ONE-CYCLE HINT IS A SUFFICIENT ARMING PREDICATE. Every bypassable
  // integer producer presents its regfile write at the same distance from the
  // hint, and that bound is ENFORCED rather than merely observed: for both
  // producer classes the bypass network and the write port are driven from one
  // node in one cycle -- the load from RegNext(io.lsu.iresp) (core.scala:965 and
  // :969), the ALU from unit.io_alu_resp (core.scala:1060 and :1050; NOTE :1058
  // is rob.io.wb_resps, the ROB writeback, NOT the regfile write port) -- and
  // execution-unit.scala:136/:141 assert that any uop reaching RRD with the hint
  // set MUST hit the bypass. Hint implies bypass-available, and bypass-available
  // IS write-presented. The speculative load-hit wakeup is bypassable too
  // (lsu.scala:1164, :1727), so a load-produced base needs no extra machinery.
  //
  // This wait is armed from the hint pulse, and that is sufficient BECAUSE
  // execution-unit.scala:136/:141 assert hint => bypass-hit while core.scala:965
  // /:969 (load) and :1060/:1050 (ALU) drive bypass and write port from one node
  // in one cycle. A sticky scoreboard is NOT required and must not be added on
  // suspicion -- it costs ~288 wires and re-opens the !pnrGate scoping. Revisit
  // ONLY if that assertion is removed.
  //
  // AND ON THE MISS PATH, WHICH IS THE NON-OBVIOUS HALF. A load that misses
  // retracts its speculative wakeup, and there IS a one-cycle window at X+2
  // (= H+1) where slot_uop.prs1_busy is still 0 -- the re-busy does not land
  // until X+3 -- while the hint has already fallen to 0, so a grant there would
  // arm nothing. That grant is killed by int_squash_grant, because the
  // retraction is broadcast on iwakeups in exactly that cycle -- both derive
  // from spec_wakeups/w2.valid. The hint pulse and the retraction are locked to
  // the same cycle by construction; that is why a one-cycle arm is sufficient on
  // the miss path too.
  // SCOPE THIS PER CYCLE, AND DO NOT READ EITHER TERM AS UNNECESSARY.
  //   X+2        : int_squash_grant (combinational, in the retraction cycle).
  //                Delete it and X+2 reopens -- the original bug.
  //   X+3 onward : the slot re-busy, once prs1_busy is restored. Delete it and
  //                the slot grants with hint=0 AND prs1_busy=0 while the miss is
  //                still outstanding, and NOTHING catches it.
  // The re-busy does NOT close X+2 -- scalar_operands_ready reads the REGISTERED
  // prs1_busy, so it lands one cycle late. That is the ONLY thing wrong with the
  // "the re-busy handles misses" explanation, and stating it unscoped ("that is
  // false") reads as a licence to delete the term that owns everything after.
  // Neither term is redundant; deleting either reopens a different set of cycles.
  //
  // `bypass_hint` is a "is the write pending?" PREDICATE, never a latency
  // constant. Do not turn it back into a number.
  //
  // Qualified by the source register type. An operand that is genuinely RT_ZERO
  // or absent takes no INT wakeup, so this never suppresses a needed wait.
  //
  // THIS IS A DEFECT MITIGATION AND ITS JUSTIFICATION LIVES IN ANOTHER FILE,
  // WHICH IS WHY IT READS AS OPTIONAL HERE. VecIssueSlot has a live OPEN DEFECT:
  // its prs2 wakeup arm (:178) and iw_p2_speculative_child set (:180) are
  // UNGATED while the matching clear (:183) IS gated on lrs2_rtype === RT_FIX --
  // two ungated SET arms feeding one gated CLEAR, so lane 1's hint can be raised
  // off a leftover rename number by a wakeup nothing will ever retract. Keep
  // this qualification until both are gated.
  // WARNING -- IT COVERS THE :178 HALF ONLY. It does not touch :180, so the
  // worse half is UNMITIGATED ANYWHERE: iw_p2_speculative_child feeds the
  // retraction predicate, so a slot whose prs2 is architecturally absent can
  // accumulate a child mask nothing clears and be re-busied repeatedly by
  // unrelated rebusys. That is a starvation/livelock shape, not corruption. Do
  // NOT read this line as evidence the slot defect is handled.
  val rr_wb_wait = RegInit(VecInit(Seq.fill(2)(false.B)))
  val rr_hint    = RegInit(VecInit(Seq.fill(2)(false.B)))
  val rr_wb_seen = RegInit(VecInit(Seq.fill(2)(false.B)))
  val grantRtypeFix = Seq(io.iss.bits.lrs1_rtype === RT_FIX, io.iss.bits.lrs2_rtype === RT_FIX)
  val grantHint     = Seq(io.iss.bits.iw_p1_bypass_hint, io.iss.bits.iw_p2_bypass_hint)
  // A write landing IN the grant cycle takes effect at the intervening edge and
  // is therefore already reflected in that lane's response -- do not arm for it,
  // or the lane would wait for a second write to a PRN that only gets one, which
  // is a hang rather than a stall.
  val grantPrn  = (0 until 2).map(n => grantAddr(n)(ipregSz - 1, 0))
  val grantHits = (0 until 2).map(n =>
    VecInit(io.int_wb_snoop.map(w => w.valid && w.bits.addr === grantPrn(n))))
  val grantArm = (0 until 2).map(n =>
    grantRtypeFix(n) && grantHint(n) && !grantHits(n).asUInt.orR)

  //@req-spec-agen.d16
  for (n <- 0 until 2) {
    io.int_rf_read_req(n).valid := heldActive(n) || io.iss.valid
    io.int_rf_read_req(n).bits  := Mux(heldActive(n), heldAddr(n), grantAddr(n))(ipregSz - 1, 0)
  }

  val fire       = VecInit((0 until 2).map(n => io.int_rf_read_req(n).fire))
  val fired_prev = RegNext(fire, VecInit(Seq.fill(2)(false.B)))

  val prevBits  = RegNext(VecInit(io.int_rf_read_req.map(_.bits)))
  val prevValid = RegNext(VecInit(io.int_rf_read_req.map(_.valid)), VecInit(Seq.fill(2)(false.B)))
  for (n <- 0 until 2) {
    assert(!(prevValid(n) && !fired_prev(n)) ||
           (io.int_rf_read_req(n).valid && io.int_rf_read_req(n).bits === prevBits(n)),
      "VecScalarOperandRead: held INT read request dropped valid or changed address before fire")
  }

  //@req-spec-issue.d3
  val hits = (0 until 2).map(n => VecInit(io.int_wb_snoop.map(w => w.valid && w.bits.addr === heldAddr(n))))
  for (n <- 0 until 2) {
    assert(!rr_valid || PopCount(hits(n)) <= 1.U,
      "VecScalarOperandRead: multiple writeback ports hit the same forward compare")
  }
  // Do not delete this substitution: BOOM's INT Mem read has no
  // read-during-write bypass, so this lane's response is stale without it.
  val fwd = (0 until 2).map(n =>
    Mux(hits(n).asUInt.orR, Mux1H(hits(n), io.int_wb_snoop.map(_.bits.data)), io.int_rf_read_rsp(n)))

  //@req-spec-issue.d3
  for (n <- 0 until 2) {
    when (fired_prev(n) && !rr_wb_wait(n)) { rr_data(n) := fwd(n) }
    // The awaited write, on the snoop. Watched whether or not this lane's read
    // has fired yet: a lane denied at grant may see the write before its held
    // read fires, and either order leaves rr_data correct.
    when (rr_wb_wait(n) && hits(n).asUInt.orR) {
      rr_data(n)    := Mux1H(hits(n), io.int_wb_snoop.map(_.bits.data))
      rr_wb_wait(n) := false.B
      rr_wb_seen(n) := true.B
      VecTrace.traceId("VecScalarOperandRead", if (isStore) "st_wb_seen" else "ld_wb_seen",
        rr_uop.rob_idx, Seq(("lane", n.U), ("prn", heldAddr(n))))
    }
  }

  //@req-spec-issue.d3
  // x0 IS NOT A REGISTER READ, on this path either. Rename maps `x0` to p0 and
  // nothing ever writes p0, so the register file hands back that entry's leftover
  // contents; BOOM's scalar register-read stage forces the zero itself. A
  // `vlse`/`vsse` whose stride register is `x0` is architecturally a ZERO stride
  // -- the idiom for broadcasting one element -- and would otherwise walk from a
  // stale value. Keyed on the register type, which decode already resolves to
  // RT_ZERO for an rs field of 0.
  val presentedRaw = (0 until 2).map(n => Mux(fired_prev(n) && !rr_wb_wait(n), fwd(n), rr_data(n)))
  val heldRtypeZero = Seq(rr_uop.lrs1_rtype === RT_ZERO, rr_uop.lrs2_rtype === RT_ZERO)
  val presented = (0 until 2).map(n => Mux(heldRtypeZero(n), 0.U(xLen.W), presentedRaw(n)))

  when (io.iss.valid) {
    rr_uop         := io.iss.bits
    rr_valid       := true.B
    rr_need(0)     := !fire(0)
    rr_need(1)     := !fire(1)
    rr_held_cycles := 0.U
    //@req-spec-issue.d3
    for (n <- 0 until 2) {
      rr_wb_wait(n) := grantArm(n)
      rr_hint(n)    := grantArm(n)
      rr_wb_seen(n) := false.B
    }
    VecTrace.traceId("VecScalarOperandRead", if (isStore) "st_grant" else "ld_grant",
      io.iss.bits.rob_idx, Seq(
        ("prs1", grantAddr(0)), ("prs2", grantAddr(1)),
        ("hint0", grantHint(0).asUInt), ("hint1", grantHint(1).asUInt),
        ("fix0", grantRtypeFix(0).asUInt), ("fix1", grantRtypeFix(1).asUInt),
        ("gh0", grantHits(0).asUInt.orR.asUInt), ("gh1", grantHits(1).asUInt.orR.asUInt),
        ("arm0", grantArm(0).asUInt), ("arm1", grantArm(1).asUInt)))
  } .otherwise {
    rr_uop.br_mask := GetNewBrMask(io.brupdate, rr_uop)
    when (killedNow) {
      rr_valid   := false.B
      rr_need(0) := false.B
      rr_need(1) := false.B
      //@req-spec-issue.d3
      rr_wb_wait(0) := false.B
      rr_wb_wait(1) := false.B
    } .otherwise {
      when (fire(0)) { rr_need(0) := false.B }
      when (fire(1)) { rr_need(1) := false.B }
      when (rr_valid && (rr_need(0) || rr_need(1))) {
        rr_held_cycles := rr_held_cycles + 1.U
      }
    }
  }

  //@req-spec-issue.h3
  io.vl_read_addr := io.iss.bits.pvl_src.get
  when (io.iss.valid) { rr_vl := io.vl_read_data }

  io.out.bits.uop         := rr_uop
  io.out.bits.base        := presented(0)
  io.out.bits.stride      := presented(1)
  io.out.bits.scalar_data := (if (isStore) presented(1) else 0.U(xLen.W))
  io.out.bits.vl          := rr_vl
  //@req-spec-lsu.m1
  io.out.bits.vl_zero     := rr_vl === 0.U
  //@req-spec-issue.d3
  io.out.valid            := rr_valid && !rr_need(0) && !rr_need(1) &&
                             !rr_wb_wait(0) && !rr_wb_wait(1) && !killedNow

  //@req-spec-issue.d3
  // bypassHintUnhonored. THE CONTRACT with VecIssueSlot's iw_p*_bypass_hint:
  // this module is that hint's only reader. Never present a lane whose wakeup
  // was bypassable without having OBSERVED the producer's write on
  // int_wb_snoop. Deliberately phrased
  // over rr_hint/rr_wb_seen rather than over the rr_wb_wait gate itself, so that
  // any future path clearing the gate WITHOUT the write having been seen (the
  // kill arm does exactly that today) still fires it. After this fix it should
  // be unfirable -- that is the point: it fails at grant+1 in the module that
  // erred instead of ten cycles later in a TLMonitor two hierarchy levels away
  // complaining about a diplomatic parameter.
  for (n <- 0 until 2) {
    assert(!(io.out.valid && rr_hint(n) && !rr_wb_seen(n)),
      s"VecScalarOperandRead: lane $n presented with a BYPASSABLE INT wakeup whose producer writeback was never observed on int_wb_snoop -- the base/stride is the previous tenant of that physical register")
  }

  //@req-spec-issue.d3
  // lateWritebackObserved. GUARDS THE INVARIANT THE TWO PASSIVE CASES REST ON.
  // A lane granted ONE cycle after its bypass hint is covered by the
  // response-cycle forward above; a lane granted TWO cycles after it is covered
  // by the register file itself. Both depend on every BYPASSABLE INT producer
  // sharing ONE wakeup-to-writeback distance -- the invariant asserted in the
  // scalar path by execution-unit.scala:136/:141 (hint => bypass-hit). That is
  // NOT "only the 1-cycle ALU is bypassable" -- the speculative load-hit wakeup
  // IS bypassable (lsu.scala:1164 under enableFastLoadUse, :1727 otherwise) and
  // IS fanned to the vector queues (core.scala:963 into int_wakeups, :1424 into
  // VecPipeline). It holds because enableFastLoadUse shifts the wakeup and the
  // response by the SAME cycle -- true path: wakeup at s0 (:1160) + iresp
  // un-RegNext'd (:1750); false path: wakeup at s1 (:1722) + iresp RegNext'd
  // (:1750) -- so BOTH settings, and the ALU, land at H+2 (measured: iregfile
  // write at wakeup+3 for an ALU producer and a load-hit producer alike). An FU
  // marked bypassable with a DIFFERENT calibration breaks both passive cases
  // SILENTLY, and bypassHintUnhonored cannot see it because rr_hint is already
  // false by then. This is that tripwire: a snoop hit on the PRN a lane just
  // consumed from the register file means that producer's write is further out
  // than the calibration assumes. Compared against the REGISTERED address rather
  // than heldAddr, so an intervening grant cannot alias the compare.
  //
  // SOUNDNESS OF THE CONVERSE -- CORRECTED, AND THE CORRECTION IS THE WHOLE
  // REASON FOR THE heldRtypeFix QUALIFIER BELOW.
  // As originally written this read: "a write to consumedAddr(n) after we read
  // it can only be our own producer writing late, because that PRN cannot be
  // freed and reallocated while a live consumer still holds it as a source."
  // That premise is FALSE for an operand the consumer DOES NOT USE. An unused
  // prs2 on a unit-stride access is not held by anything, so it can be freed and
  // reallocated while we are live, and an unrelated instruction writes it
  // legitimately. This assertion then fires on correct behaviour -- which it did,
  // on ms11a2_pure_vle, ms11a3_vle_lmul2 and ms2_vse64, all unit-stride, all
  // lane 1.
  // CORRECT STATEMENT: the PRN cannot be freed while a live consumer holds it as
  // A SOURCE IT ACTUALLY USES. Hence heldRtypeFix: only track a lane that
  // genuinely has an INT source. Do not drop that qualifier -- it is not a
  // filter for noise, it is the precondition the soundness argument needs.
  // The remaining span is then safe with margin: a PRN is freed only when the
  // next writer of the same architectural register COMMITS, and that instruction
  // is younger than this consumer, which has not executed yet. So the
  // no-reallocation span runs to OUR commit and STRICTLY CONTAINS this window.
  //
  // AND `killedNow` IN THE consumedValid CLEAR BELOW IS LOAD-BEARING, NOT
  // HYGIENE. There is exactly one way to leave that span: a SQUASH, which
  // returns the register to the free list where it can be reallocated on the
  // correct path -- at which point a still-held consumedValid would fire on a
  // perfectly legitimate write to the NEW tenant. Called out because "cleared on
  // kill" reads like boilerplate and is exactly the term a future simplifier
  // deletes.
  //
  // WINDOW: from the consuming read until the op leaves -- deliberately WIDER
  // than a single RegNext peephole, and deliberately DIFFERENT from
  // staleCaptureCheck's two-cycle bound. The reason the two differ: this window
  // terminates while the op is still LIVE, so the no-reallocation argument above
  // holds across all of it; staleCaptureCheck's can outlive the op, so it must
  // stay bounded or it compares against a reallocated register. A one-cycle
  // peephole here would only catch a producer late by exactly one, which is
  // useless against an FU whose calibration we do not know.
  //
  // TWO DEVIATIONS FROM THE ARCHITECT'S DRAFT OF THIS BLOCK, both deliberate,
  // recorded so they can be objected to rather than discovered:
  //   (1) heldAddr is TRUNCATED to ipregSz at the capture. The draft registered
  //       the full prs* width and compared it width-extended against an
  //       ipregSz-wide snoop address. Behaviourally identical while prs* carries
  //       no bits above ipregSz-1, but every other use of heldAddr in this file
  //       truncates (see :134), and an untruncated compare in a file that
  //       truncates everywhere else is a trap for the next reader.
  //   (2) consumedValid is a held register with an explicit clear rather than a
  //       single RegNext. That is what implements the wide window above; the
  //       draft's RegNext gave a one-cycle peephole.
  val consumedFromRf = (0 until 2).map(n =>
    fired_prev(n) && !rr_wb_wait(n) && !hits(n).asUInt.orR && rr_valid &&
    heldRtypeFix(n))
  val consumedAddr  = Reg(Vec(2, UInt(ipregSz.W)))
  val consumedValid = RegInit(VecInit(Seq.fill(2)(false.B)))
  for (n <- 0 until 2) {
    when (consumedFromRf(n)) {
      consumedValid(n) := true.B
      consumedAddr(n)  := heldAddr(n)(ipregSz - 1, 0)
    }
    when (io.iss.valid || killedNow) { consumedValid(n) := false.B }

    val lateHit = consumedValid(n) &&
      VecInit(io.int_wb_snoop.map(w => w.valid && w.bits.addr === consumedAddr(n))).asUInt.orR
    assert(!lateHit,
      s"VecScalarOperandRead: lane $n took its value from the INT register file and that PRN's producer writeback then appeared on int_wb_snoop -- a bypassable producer's wakeup-to-writeback distance exceeds the one this module's forward window is calibrated for (execution-unit.scala:136), and the consumed base/stride is stale")
  }

  //@req-spec-issue.d3
  // staleCaptureCheck. The ONLY property here that does not trust the readiness
  // logic: it compares what was PRESENTED against what the integer register file
  // turned out to hold, whatever mechanism was supposed to guarantee they agree.
  // A write to a presented lane's PRN landing in the presentation cycle or the
  // next one, carrying different data, means the capture was stale -- which is
  // exactly the shape of the defect this commit fixes (write at grant+2, capture
  // at grant+1). Bounded at two cycles deliberately: beyond that a PRN can have
  // been freed and reallocated, and the comparison would stop being meaningful.
  //
  // QUALIFIED BY heldRtypeFix FOR THE SAME REASON AS lateWritebackObserved --
  // audited and found to have the IDENTICAL hole. It also compares against
  // rr_uop.prs*, so on a unit-stride op's unused lane 1 it can fire when the
  // stale PRN is reallocated and written with different data inside its window.
  // It had simply not been hit yet: its window is two cycles where the other's
  // is the op's whole life, so it is the same defect with a much smaller target.
  // Fixed together deliberately -- fixing only the one that fired would have
  // left a latent false positive to surface on an unrelated future run.
  val pres_v     = RegNext(io.out.valid, false.B)
  val pres_prn   = RegNext(VecInit(heldAddr.map(a => a(ipregSz - 1, 0))))
  val pres_data  = RegNext(VecInit(presented))
  val pres_rtype = RegNext(VecInit(heldRtypeFix), VecInit(Seq.fill(2)(false.B)))
  for (n <- 0 until 2) {
    val lateHits = VecInit(io.int_wb_snoop.map(w => w.valid && w.bits.addr === pres_prn(n)))
    val lateData = Mux1H(lateHits, io.int_wb_snoop.map(_.bits.data))
    assert(!(pres_v && pres_rtype(n) && lateHits.asUInt.orR && lateData =/= pres_data(n)),
      s"VecScalarOperandRead: STALE CAPTURE on lane $n -- the integer register file received a different value for this lane's PRN within two cycles of presentation, so the base/stride handed to the AGEN was not the architectural value")
  }

  //@req-spec-issue.d3
  // wbWaitWatchdog. A BACKSTOP, NOT A LOAD-BEARING GUARANTEE -- and the number
  // is still not a pipeline depth, so do not tighten it either.
  //
  // Correction to an earlier rationale in this file, recorded because an
  // OVERSTATED reason is the same defect class as an understated one: this was
  // documented as load-bearing on the theory that an awaited write might never
  // arrive. It cannot. The only way to lose it is a producer squashed while its
  // consumer survives, which cannot happen here -- the consumer is younger and
  // carries a same-or-superset br_mask, so any kill that takes the producer also
  // takes us, and `killedNow` clears the wait.
  //
  // What IS true, and is why the number must stay large: `int_squash_grant`
  // (core.scala:1441-1444 -> VecIssueUnit's iss_uops kill) only suppresses a
  // grant in the SAME cycle as a retraction; a rebusy arriving after this module
  // has been granted does not reach it, and ld_opnd/st_opnd have no kill input
  // beyond brupdate/rob_flush. So on a mis-speculated base the wait legitimately
  // lasts a D$ miss plus refill -- correct behaviour, and a strict improvement
  // over computing an address from the mis-speculated value, but slow. Size for
  // miss+refill, not for the ALU's two cycles.
  if (wbWaitWatchdog > 0) {
    val waiting  = rr_wb_wait.asUInt.orR
    val wait_cnt = RegInit(0.U(log2Ceil(wbWaitWatchdog + 2).W))

    when (waiting) {
      wait_cnt := wait_cnt + 1.U
    } .otherwise {
      wait_cnt := 0.U
    }

    assert(wait_cnt <= wbWaitWatchdog.U,
      "VecScalarOperandRead: a lane has waited for its bypassable producer's int_wb_snoop write past wbWaitWatchdog cycles -- the awaited writeback never reached a snoop port")

    // Trip the trace ONE CYCLE BEFORE the assertion: VCS's $finish on the failing
    // assert pre-empts a printf in the same time step, so a trace armed on the
    // same comparison never reaches the log -- measured on conv1d-vector.
    when (wait_cnt === wbWaitWatchdog.U) {
      VecTrace.traceStruct("VecScalarOperandRead", "wb_wait_watchdog_trip",
        Seq(("lane0_waiting", rr_wb_wait(0)), ("lane1_waiting", rr_wb_wait(1)),
            ("prs1", heldAddr(0)), ("prs2", heldAddr(1)), ("rob_idx", rr_uop.rob_idx)))
    }
  }

  //@req-spec-issue.d3
  // WIDENED to cover rr_wb_wait, and the widening is the point. This guard used
  // to test rr_need alone, which goes to 0 as soon as the read fires -- so once
  // only the writeback wait is outstanding it saw nothing, and the io.iss.valid
  // branch below would silently overwrite rr_uop/rr_wb_wait/rr_hint/rr_wb_seen
  // and re-arm against a different op.
  //
  // That is reachable BECAUSE of this fix, which is why it is asserted rather
  // than reasoned about: on a grant at X+1 for a load that then misses,
  // rr_wb_wait arms and holds correctly, but VecIssueSlot.scala:389-390/:409-410
  // keep the slot valid and clear iw_issued on the rebusy, so the refill's slow
  // wakeup can grant THE SAME OP a second time while this stage is still
  // waiting from the first grant. Static reading says the outcome is probably
  // correct, but the margin is one cycle and it is inferred, not measured, and
  // this state did not exist before. Let the regression answer it.
  assert(!(io.iss.valid && rr_valid &&
           (rr_need(0) || rr_need(1) || rr_wb_wait(0) || rr_wb_wait(1))),
    "VecScalarOperandRead: grant landed on a still-unfired hold or an outstanding writeback wait")

  assert(!io.iss.valid ||
    (io.iss.bits.is_vec.get && (if (isStore) io.iss.bits.uses_stq else io.iss.bits.uses_ldq)),
    "VecScalarOperandRead: granted uop missing is_vec or direction-appropriate uses_ldq/uses_stq")

  when (io.out.valid) {
    VecTrace.traceVl("VecScalarOperandRead", if (isStore) "st_resolve" else "ld_resolve", rr_uop, rr_vl,
      Seq(("base", presented(0)), ("stride", presented(1)), ("v_eew", rr_uop.v_eew.get),
          ("held_cycles", rr_held_cycles)))
  }
  for (n <- 0 until 2) {
    when (fired_prev(n) && hits(n).asUInt.orR) {
      VecTrace.trace("VecScalarOperandRead", "wb_forward", rr_uop,
        Seq(("lane", n.U), ("addr", heldAddr(n)), ("data", fwd(n))))
    }
  }
}

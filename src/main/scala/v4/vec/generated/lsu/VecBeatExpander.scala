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

import freechips.rocketchip.rocket.{M_XRD, M_XWR}

import boom.v4.common.BoomModule
import boom.v4.vec.generated.{VecMemAccess, VecRangeEntry, VecElemAccess, VecTrace}
import boom.v4.vec.formal.{BoomSvaLayer, VecBeatExpanderChecks}

// GENERATED from src/main/nlhdl/vec/lsu/VecBeatExpander.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
class VecBeatExpander(
  val isStore:       Boolean,
  val nLanes:        Int,
  val dmemBeatBytes: Int
)(implicit p: Parameters) extends BoomModule
{
  require(nLanes == 1 || nLanes == 2, s"VecBeatExpander: nLanes ($nLanes) must be 1 or 2")
  require(isPow2(dmemBeatBytes), s"VecBeatExpander: dmemBeatBytes ($dmemBeatBytes) must be a power of two")
  require(dmemBeatBytes <= (1 << corePgIdxBits),
    s"VecBeatExpander: dmemBeatBytes ($dmemBeatBytes) must not exceed the page size")
  require((dcacheArbiterMode == "single" && nLanes == 1) || (dcacheArbiterMode == "dual-dynamic" && nLanes == 2),
    s"VecBeatExpander: nLanes ($nLanes) disagrees with VectorParams.dcacheArbiterMode ($dcacheArbiterMode)")

  val vLenBytes   = vecVLen / 8
  val beatOffBits = log2Ceil(dmemBeatBytes)
  val maxElems    = vecVLen
  val elemIdxSz   = log2Ceil(maxElems + 1)
  val sizeBits    = 2
  require(log2Ceil(dmemBeatBytes) < (1 << sizeBits),
    s"VecBeatExpander: log2Ceil(dmemBeatBytes) (${log2Ceil(dmemBeatBytes)}) must be < (1 << sizeBits) (${1 << sizeBits})")

  val byteW      = log2Ceil(maxVecMembers * vecVLen / 8 + 1)
  val memberIdxW = log2Ceil(vLenBytes)
  val elenBytes  = vecELen / 8

  val io = IO(new Bundle {
    val us_head      = Input(Valid(new VecRangeEntry))
    val us_cursor    = Input(UInt(elemIdxSz.W))
    val us_cursor_wr = Output(Valid(UInt(elemIdxSz.W)))
    val us_pop       = Output(Bool())

    val ssi_head = Input(Vec(nLanes, Valid(new VecElemAccess)))
    val ssi_pop  = Output(Vec(nLanes, Bool()))

    val st_us_data      = if (isStore) Some(Input(Valid(UInt(vecVLen.W))))              else None
    val st_us_data_pop  = if (isStore) Some(Output(Bool()))                             else None
    val st_ssi_data     = if (isStore) Some(Input(Vec(nLanes, Valid(UInt(vecELen.W))))) else None
    val st_ssi_data_pop = if (isStore) Some(Output(Vec(nLanes, Bool())))                else None
    val is_write_pass   = if (isStore) Some(Input(Bool()))                              else None

    // Bare Decoupled, not Output(Decoupled(...)): the wrapper would force `ready` to
    // be an output too, and it is the arbiter's grant coming back.
    val req = Vec(nLanes, Decoupled(new VecMemAccess))

    // The LCB-allocation credit, as three raw terms rather than one pre-reduced
    // ready bit. The escape "this beat's op has already finished allocating, so
    // it can never need a free entry" depends on WHICH op the beat belongs to,
    // and only this module knows that: the unit-stride path reads `us_head`
    // while each SSI lane reads its own `ssi_head(i)`. Reducing it upstream
    // forces one op's identity onto both paths and silently mis-answers for the
    // other -- see `lcbRdyFor`.
    val lcb_free_nonzero = if (!isStore) Some(Input(Bool()))                 else None
    val lcb_walk_active  = if (!isStore) Some(Input(Bool()))                 else None
    val lcb_walk_rob     = if (!isStore) Some(Input(UInt(robAddrSz.W)))      else None
    val stop = Input(Bool())
    val kill = Input(Bool())
  })

  def place(field: UInt, byteOff: UInt, outBytes: Int): UInt =
    (field << (byteOff << 3.U))(outBytes * 8 - 1, 0)

  def extract(field: UInt, byteOff: UInt, outBytes: Int): UInt =
    (field >> (byteOff << 3.U))(outBytes * 8 - 1, 0)

  private case class ComposeResult(
    fire:      Bool,
    isSkip:    Bool,
    corner:    Bool,
    curElem:   UInt,
    curByte:   UInt,
    addr:      UInt,
    runBytes:  UInt,
    runElems:  UInt,
    memberEnd: Bool,
    nextElem:  UInt,
    nextByte:  UInt
  )

  //@req-spec-agen.c2
  //@req-spec-agen.c3
  private def composeLane(elemIn: UInt, byteIn: UInt): ComposeResult = {
    val eew    = io.us_head.bits.eew
    val base   = io.us_head.bits.base
    val nf     = io.us_head.bits.nf
    val active = io.us_head.bits.len
    val mask   = io.us_head.bits.mask
    val maskFieldW = mask.getWidth

    val addr  = base + byteIn
    val remVL = active - byteIn
    val done  = remVL === 0.U

    //@req-spec-agen.c23
    val relMask   = mask >> elemIn
    val curActive = relMask(0)

    val skipRaw    = Mux(relMask === 0.U, maskFieldW.U, PriorityEncoder(relMask))
    val remVLElems = remVL >> eew
    val skipElemsW = Mux(skipRaw.pad(byteW) > remVLElems, remVLElems, skipRaw.pad(byteW))

    val notMask      = ~relMask
    val activeRunRaw = Mux(notMask === 0.U, maskFieldW.U, PriorityEncoder(notMask))
    val maskRunBytes = (activeRunRaw.pad(byteW) << eew)(byteW - 1, 0)

    val remVreg  = vLenBytes.U(byteW.W) - byteIn(memberIdxW - 1, 0)
    val remDmem  = dmemBeatBytes.U(byteW.W) - addr(beatOffBits - 1, 0)
    val segBytes = 1.U(byteW.W) << eew
    val c4       = Mux(nf > 1.U, segBytes, remVL)

    val minBytes    = Seq(remVL, remVreg, remDmem, c4, maskRunBytes).reduce((a, b) => Mux(a < b, a, b))
    val elemBytes   = 1.U(byteW.W) << eew
    val runElemsRaw = minBytes >> eew
    val corner      = runElemsRaw === 0.U
    val runElems    = Mux(corner, 1.U, runElemsRaw)
    val runBytes    = Mux(corner, elemBytes, minBytes)

    val fire   = !done
    val isSkip = fire && !curActive

    val advElems = Mux(isSkip, skipElemsW, runElems)
    val advBytes = Mux(isSkip, (skipElemsW << eew)(byteW - 1, 0), runBytes)

    //@req-spec-lsu.d6
    val memberEnd = fire && !isSkip && ((minBytes === remVL) || (minBytes === remVreg))

    ComposeResult(fire, isSkip, corner, elemIn, byteIn, addr, runBytes, runElems, memberEnd,
      (elemIn + advElems)(elemIdxSz - 1, 0), (byteIn + advBytes)(byteW - 1, 0))
  }

  //@req-spec-agen.b8
  //@req-spec-lsu.c5
  //@req-spec-lsu.c6
  //@req-spec-lsu.j2
  val usInitByte = (io.us_cursor << io.us_head.bits.eew)(byteW - 1, 0)
  // A beat needs a free LCB entry ONLY while its OWN op is the one still
  // allocating. The walk is a single global resource, so without the rob_idx
  // term a LATER op's walk stalling at free_count = 0 blocks beats belonging to
  // an EARLIER op whose entries already exist -- a circular wait, because those
  // beats are what lets that op complete, retire and free the entries the walk
  // is waiting for.
  //
  // Take the op identity PER PATH. `us_head` and `ssi_head(i)` are different
  // ops in flight at the same time, so a single ready bit computed upstream from
  // one of them answers the wrong question for the other: that is why the
  // strided (`vlse`/SSI) path still starved after the unit-stride path was
  // fixed against `us_head.rob_idx` alone.
  def lcbRdyFor(rob: UInt): Bool =
    if (!isStore) io.lcb_free_nonzero.get || !io.lcb_walk_active.get || (rob =/= io.lcb_walk_rob.get)
    else true.B
  // The data queue read has a cycle of latency, so a write-pass beat composed
  // before it lands sends stale bytes to the D$ -- silently, as a store.
  def usDataRdy: Bool = if (isStore) (!io.is_write_pass.get || io.st_us_data.get.valid) else true.B
  val usGate = io.us_head.valid && !io.stop && !io.kill &&
    lcbRdyFor(io.us_head.bits.rob_idx) && usDataRdy

  //@req-spec-agen.c1
  // Per-lane, and per-lane's OWN op: at nLanes>1 two beats in one cycle can target
  // two different destination members, which are two independent allocations.
  val ssiLaneFire = Seq.tabulate(nLanes)(i =>
    io.ssi_head(i).valid && lcbRdyFor(io.ssi_head(i).bits.uop.rob_idx))

  val usWantsAny  = usGate
  val ssiWantsAny = ssiLaneFire.reduce(_ || _)
  val contend     = usWantsAny && ssiWantsAny
  val rr = RegInit(false.B)
  val usSelected = Mux(contend, !rr, usWantsAny)

  private var elemAcc: UInt = io.us_cursor
  private var byteAcc: UInt = usInitByte
  private val usAdvOk  = new Array[Bool](nLanes)
  private val usMemEnd = new Array[Bool](nLanes)
  private val usIsSkip = new Array[Bool](nLanes)
  private val usStartElem = new Array[UInt](nLanes)
  private val useUs  = new Array[Bool](nLanes)

  for (i <- 0 until nLanes) {
    val res  = composeLane(elemAcc, byteAcc)
    val head = io.us_head.bits
    val eew  = head.eew

    usStartElem(i) = elemAcc
    val fireLane = res.fire && usGate
    val laneSel  = usSelected && fireLane
    useUs(i) = laneSel

    val memberIdx = (res.curByte >> memberIdxW)(log2Ceil(maxVecMembers) - 1, 0)
    val byteOff   = res.curByte(memberIdxW - 1, 0)
    //@req-spec-lsu.j3
    val isFirstBeat = (i == 0).B && (io.us_cursor === 0.U)

    val lanePayload = Wire(new VecMemAccess)
    lanePayload := DontCare
    lanePayload.vaddr   := res.addr
    lanePayload.eew     := eew
    lanePayload.byte_en := 0.U
    lanePayload.first   := isFirstBeat
    lanePayload.last    := false.B
    lanePayload.uop.rob_idx := head.rob_idx
    lanePayload.uop.ldq_idx := head.ldq_idx
    lanePayload.uop.stq_idx := head.stq_idx
    lanePayload.uop.mem_size := eew
    // Synthesized from DontCare, unlike the SSI lane below which copies a whole
    // uop: every field the LSU, TLB or D$ reads off this access must be set here.
    lanePayload.uop.is_vec.get := true.B
    lanePayload.uop.uses_ldq   := (!isStore).B
    lanePayload.uop.uses_stq   := isStore.B
    lanePayload.uop.mem_cmd    := (if (isStore) M_XWR else M_XRD)
    lanePayload.uop.v_split_dst_prn.get := head.pvdest(memberIdx)
    lanePayload.uop.v_split_dst_byte_off.get := byteOff
    lanePayload.data           := 0.U
    lanePayload.uses_tlb        := true.B
    lanePayload.uses_dcache     := true.B
    lanePayload.uses_lcam       := isFirstBeat
    lanePayload.lcam_range_len := Mux(isFirstBeat, head.len, 0.U)

    val reqValid = laneSel && !res.isSkip
    var advOk: Bool  = false.B
    var nextElem: UInt = elemAcc
    var nextByte: UInt = byteAcc
    var memberEndOk: Bool = false.B

    if (!isStore) {
      val shiftW    = log2Ceil(dmemBeatBytes + 1)
      val loadAddr  = Mux(res.corner, res.addr, (res.addr >> beatOffBits) << beatOffBits)
      val loadSize  = Mux(res.corner, eew, beatOffBits.U(sizeBits.W))
      val laneOff   = Mux(res.corner, 0.U, res.addr(beatOffBits - 1, 0))
      val activeLen = res.runBytes(shiftW - 1, 0)
      val byteEn    = (((1.U << activeLen) - 1.U) << laneOff)(elenBytes - 1, 0)

      lanePayload.vaddr    := loadAddr
      lanePayload.byte_en  := byteEn
      lanePayload.uop.mem_size := loadSize
      lanePayload.last     := res.fire && !res.isSkip && (res.nextByte >= head.len)

      advOk     = laneSel && Mux(res.isSkip, true.B, io.req(i).fire)
      nextElem  = Mux(advOk, res.nextElem, elemAcc)
      nextByte  = Mux(advOk, res.nextByte, byteAcc)
      memberEndOk = advOk && res.memberEnd
    } else {
      val shiftW    = log2Ceil(dmemBeatBytes + 1)
      val addrLow   = res.addr(beatOffBits - 1, 0)
      val alignLog2 = PriorityEncoder(Cat(1.U(1.W), addrLow))
      val runLog2   = Log2(res.runBytes)
      val sizeLog2  = Mux(alignLog2 < runLog2, alignLog2, runLog2)
      val sizeBytes = (1.U(byteW.W) << sizeLog2)(byteW - 1, 0)
      val storeAddr = (res.addr >> sizeLog2) << sizeLog2
      val storeOff  = storeAddr(beatOffBits - 1, 0)
      val byteEn    = (((1.U << sizeBytes(shiftW - 1, 0)) - 1.U) << storeOff)(elenBytes - 1, 0)

      //@req-spec-lsu.d6
      val memberOff = res.curByte(memberIdxW - 1, 0)
      val extracted = extract(io.st_us_data.get.bits, memberOff, dmemBeatBytes)
      val rotated   = place(extracted, storeOff, dmemBeatBytes)

      val writePass = io.is_write_pass.get
      val composeNextByte = (res.curByte + sizeBytes)(byteW - 1, 0)
      val composeNextElem = (res.curElem + (sizeBytes >> eew))(elemIdxSz - 1, 0)
      val composeMemberEnd = (composeNextByte(byteW - 1, memberIdxW) =/= res.curByte(byteW - 1, memberIdxW)) ||
        (composeNextByte >= head.len)

      lanePayload.vaddr    := storeAddr
      lanePayload.byte_en  := byteEn
      lanePayload.uop.mem_size := sizeLog2
      lanePayload.last     := res.fire && !res.isSkip && (composeNextByte >= head.len)
      lanePayload.data           := rotated
      lanePayload.uses_dcache     := writePass
      lanePayload.uses_lcam       := isFirstBeat && !writePass
      lanePayload.lcam_range_len := Mux(isFirstBeat && !writePass, head.len, 0.U)

      advOk    = laneSel && Mux(res.isSkip, true.B, io.req(i).fire)
      nextElem = Mux(advOk, Mux(res.isSkip, res.nextElem, composeNextElem), elemAcc)
      nextByte = Mux(advOk, Mux(res.isSkip, res.nextByte, composeNextByte), byteAcc)
      // A member consumed entirely by skips still ends, and its st_US_DATA_Q entry
      // must still pop -- gating this on !isSkip desyncs the data queue permanently.
      memberEndOk = advOk && Mux(res.isSkip, res.memberEnd, composeMemberEnd)
    }

    io.req(i).valid := reqValid
    io.req(i).bits  := lanePayload

    elemAcc = nextElem
    byteAcc = nextByte
    usAdvOk(i)  = advOk
    usMemEnd(i) = memberEndOk
    usIsSkip(i) = res.isSkip
  }

  //@req-spec-lsu.k8
  val usAnyAdvance = usAdvOk.reduce(_ || _)
  io.us_cursor_wr.valid := usAnyAdvance
  io.us_cursor_wr.bits  := elemAcc
  //@req-spec-lsu.f13
  val usRangeDoneNow = usAnyAdvance && (byteAcc >= io.us_head.bits.len)
  io.us_pop := usRangeDoneNow && (if (isStore) io.is_write_pass.get else true.B)
  if (isStore) {
    // Only the WRITE pass consumes data. Unqualified, the translate pass walks
    // the member pointer too and the write pass then reads a never-filled index.
    io.st_us_data_pop.get := usMemEnd.reduce(_ || _) && io.is_write_pass.get
  }

  when (contend)   { rr := !rr }
  when (io.us_pop) { rr := false.B }
  when (io.kill)   { rr := false.B }

  private val useSsi = new Array[Bool](nLanes)
  for (i <- 0 until nLanes) {
    val sel = !usSelected && ssiLaneFire(i)
    useSsi(i) = sel

    val ssiBits = io.ssi_head(i).bits
    val payload = Wire(new VecMemAccess)
    payload := DontCare
    payload.uop     := ssiBits.uop
    payload.vaddr   := ssiBits.vaddr
    payload.eew     := ssiBits.eew
    payload.byte_en := ssiBits.byte_en
    payload.first   := ssiBits.first
    payload.last    := ssiBits.last

    val laneOff = ssiBits.vaddr(beatOffBits - 1, 0)
    if (isStore) {
      val writePass = io.is_write_pass.get
      payload.data       := place(io.st_ssi_data.get(i).bits, laneOff, dmemBeatBytes)
      payload.uses_dcache := writePass
      payload.uses_lcam   := !writePass
    } else {
      payload.data       := 0.U
      payload.uses_dcache := true.B
      payload.uses_lcam   := true.B
    }
    payload.uses_tlb       := true.B
    payload.lcam_range_len := 0.U

    when (sel) {
      io.req(i).valid := true.B
      io.req(i).bits  := payload
    }

    io.ssi_pop(i) := sel && io.req(i).fire
    if (isStore) {
      io.st_ssi_data_pop.get(i) := sel && io.req(i).fire
    }
  }

  when (usAnyAdvance) {
    VecTrace.traceId("VecBeatExpander", "beat", io.us_head.bits.rob_idx,
      Seq(("elem", elemAcc), ("run_bytes", byteAcc - usInitByte)))
  }
  when (io.us_head.valid && io.stop) {
    VecTrace.traceId("VecBeatExpander", "stop", io.us_head.bits.rob_idx)
  }
  when (io.us_pop) {
    VecTrace.traceId("VecBeatExpander", "pop", io.us_head.bits.rob_idx)
  }
  for (i <- 0 until nLanes) {
    when (useUs(i) && usIsSkip(i) && usAdvOk(i)) {
      VecTrace.traceId("VecBeatExpander", "mask_skip", io.us_head.bits.rob_idx,
        Seq(("elem", usStartElem(i))))
    }
  }

  //@formal-anchor VecBeatExpanderChecks
  layer.block(BoomSvaLayer) {
    VecBeatExpanderChecks(
      reqValid      = io.req(0).valid,
      reqByteEn     = io.req(0).bits.byte_en,
      usHeadValid   = io.us_head.valid,
      ssiHeadValid  = io.ssi_head(0).valid,
      cursorWrValid = io.us_cursor_wr.valid,
      cursorWrBits  = io.us_cursor_wr.bits,
      usCursor      = io.us_cursor,
      reqFire       = io.req(0).fire
    )
  }
}

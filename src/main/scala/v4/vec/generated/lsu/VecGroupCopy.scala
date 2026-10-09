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
import boom.v4.lsu.IdxAgeYt
import boom.v4.vec.generated.{VecGroupDone, VecTrace}
import boom.v4.vec.formal.{BoomSvaLayer, VecGroupCopyChecks}

// GENERATED from src/main/nlhdl/vec/lsu/VecGroupCopy.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecGroupCopyLaunch(implicit p: Parameters) extends BoomBundle
{
  val uop          = new MicroOp
  val vl_zero      = Bool()
  val all_inactive = Bool()
}

class VecGroupCopyVrfWrite(implicit p: Parameters) extends BoomBundle
{
  val addr = UInt(vecPregSz.W)
  val data = UInt(vecVLen.W)
  val mask = UInt((vecVLen / 8).W)
}

class VecGroupCopyIO(implicit p: Parameters) extends BoomBundle
{
  val launch = Flipped(Valid(new VecGroupCopyLaunch))
  val kill   = Input(Bool())

  val lcb_r2_req  = Input(Valid(UInt(vecPregSz.W)))
  val lcb_r2_data = Output(UInt(vecVLen.W))
  val lcb_w0      = Input(Valid(new VecGroupCopyVrfWrite))

  val vrf_r2_req  = Output(Valid(UInt(vecPregSz.W)))
  val vrf_r2_data = Input(UInt(vecVLen.W))
  val vrf_w0      = Output(Valid(new VecGroupCopyVrfWrite))

  val group_done = Output(Valid(new VecGroupDone))

  val squash = Input(Valid(UInt((1 + ldqAddrSz).W)))
  val flush  = Input(Bool())
}

class VecGroupCopy(val gcopyEntries: Int = 128)(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecGroupCopy: elaborates only under usingRVV")

  require(gcopyEntries >= math.min(numVecPhysRegs - 32, numLdqEntries * maxVecMembers),
    s"VecGroupCopy: gcopyEntries ($gcopyEntries) must be >= " +
    s"min(numVecPhysRegs - 32, numLdqEntries * maxVecMembers) " +
    s"(${math.min(numVecPhysRegs - 32, numLdqEntries * maxVecMembers)})")
  require(isPow2(gcopyEntries), s"VecGroupCopy: gcopyEntries ($gcopyEntries) must be a power of two")

  val io = IO(new VecGroupCopyIO)

  val memberCntW = log2Ceil(maxVecMembers + 1)
  def toMemberCount(x: UInt): UInt =
    if (x.getWidth >= memberCntW) x(memberCntW - 1, 0) else x.pad(memberCntW)

  val emulW = log2Ceil(maxVecMembers) + 1

  class Row extends Bundle {
    val dst_prn = UInt(vecPregSz.W)
    val src_prn = UInt(vecPregSz.W)
    val last    = Bool()
    val rob_idx = UInt(robAddrSz.W)
    val ldq_idx = UInt((1 + ldqAddrSz).W)
  }

  val idxW = log2Ceil(gcopyEntries)
  val ptrW = idxW + 1
  def phys(x: UInt): UInt = x(idxW - 1, 0)
  def wrapAdd(a: UInt, b: UInt): UInt = (a + b)(ptrW - 1, 0)
  def wrapSub(a: UInt, b: UInt): UInt = (a - b)(ptrW - 1, 0)

  val uop          = io.launch.bits.uop
  val launchFire   = io.launch.valid && !io.kill
  val isShared     = uop.is_shared.get
  val vta          = uop.vconfig.get.vta
  val vma          = uop.vconfig.get.vma
  val vlZero       = io.launch.bits.vl_zero
  val allInactive  = io.launch.bits.all_inactive

  //@req-spec-lsu.m4
  val mustPreserve = !vta || (!vma && !vlZero)

  //@req-spec-lsu.m2
  //@req-spec-lsu.m3
  val needCopy   = launchFire && !isShared && mustPreserve
  val noCopyFire = launchFire && (isShared || !mustPreserve)

  assert(!(launchFire && uop.v_is_whole_reg.get),
    "VecGroupCopy: launch fired for a whole-register access")
  assert(!(launchFire && (uop.is_shared.get =/= (uop.v_seg_nf.get =/= 0.U))),
    "VecGroupCopy: is_shared disagrees with v_seg_nf on a launch")

  // ---- the pending work list ----
  val pendRows = Reg(Vec(gcopyEntries, new Row))
  val pendHead = RegInit(0.U(ptrW.W))
  val pendTail = RegInit(0.U(ptrW.W))
  val readHead = RegInit(0.U(ptrW.W))

  // ---- Stage A: the in-flight R2 read ----
  val aValid  = RegInit(false.B)
  val aDst    = Reg(UInt(vecPregSz.W))
  val aSrc    = Reg(UInt(vecPregSz.W))
  val aLast   = Reg(Bool())
  val aRobIdx = Reg(UInt(robAddrSz.W))

  // ---- Stage B: the single vLen-wide staging register ----
  val stageBValid  = RegInit(false.B)
  val stageBData   = Reg(UInt(vecVLen.W))
  val stageBDst    = Reg(UInt(vecPregSz.W))
  val stageBSrc    = Reg(UInt(vecPregSz.W))
  val stageBLast   = Reg(Bool())
  val stageBRobIdx = Reg(UInt(robAddrSz.W))

  // ---- the completion accumulator ----
  val doneMembers = Reg(Vec(maxVecMembers, UInt(vecPregSz.W)))
  val doneCount   = RegInit(0.U(memberCntW.W))

  // ---- the launch expansion register ----
  val expandValid  = RegInit(false.B)
  val expandDst    = Reg(Vec(maxVecMembers, UInt(vecPregSz.W)))
  val expandSrc    = Reg(Vec(maxVecMembers, UInt(vecPregSz.W)))
  val expandCount  = Reg(UInt(emulW.W))
  val expandCursor = Reg(UInt(emulW.W))
  val expandRobIdx = Reg(UInt(robAddrSz.W))
  val expandLdqIdx = Reg(UInt((1 + ldqAddrSz).W))

  //@req-spec-lsu.m9
  //@req-spec-lsu.m10
  val noCopyValid   = RegInit(false.B)
  val noCopyPvdest  = Reg(Vec(maxVecMembers, UInt(vecPregSz.W)))
  val noCopyMembers = Reg(UInt(memberCntW.W))
  val noCopyRobIdx  = Reg(UInt(robAddrSz.W))

  // A flush retires the ROB entry this pulse would clear, so it must suppress it too.
  noCopyValid := noCopyFire && !io.flush
  when (noCopyFire) {
    noCopyPvdest  := Mux(isShared, uop.pvtmp.get, uop.pvdest.get)
    noCopyMembers := toMemberCount(uop.v_emul.get)
    noCopyRobIdx  := uop.rob_idx
  }

  val acceptNewCopy = needCopy && !expandValid
  assert(!(needCopy && expandValid),
    "VecGroupCopy: launch dropped -- gcopy expansion register busy, rob_idx=%d", uop.rob_idx)

  //@req-spec-lsu.m5
  val pushRow = Wire(new Row)
  pushRow.dst_prn := expandDst(expandCursor)
  pushRow.src_prn := expandSrc(expandCursor)
  pushRow.last    := expandCursor === (expandCount - 1.U)
  pushRow.rob_idx := expandRobIdx
  pushRow.ldq_idx := expandLdqIdx

  assert(!(expandValid && wrapSub(pendTail, pendHead) === gcopyEntries.U(ptrW.W)),
    "VecGroupCopy: pending work-list overflow, rob_idx=%d", expandRobIdx)

  // ---- the head-region age scan for squash ----
  val occ = wrapSub(pendTail, pendHead)
  val squashDiscard = VecInit((0 until gcopyEntries).map { o =>
    val slot = pendRows(phys(wrapAdd(pendHead, o.U(ptrW.W))))
    (o.U(ptrW.W) < occ) && IdxAgeYt(slot.ldq_idx, io.squash.bits)
  })
  val keepCount     = Mux(squashDiscard.asUInt.orR, PriorityEncoder(squashDiscard), occ)
  val squashNewTail = wrapAdd(pendHead, keepCount)

  // ---- the strict-priority mux over R2 / W0 ----
  val w0Req              = stageBValid
  val w0HeldForGroupDone = stageBValid && stageBLast && noCopyValid
  val w0MyReq            = w0Req && !w0HeldForGroupDone
  val w0Granted          = w0MyReq && !io.lcb_w0.valid

  val readAvail = readHead =/= pendTail
  val aRow      = pendRows(phys(readHead))
  val myR2Req   = readAvail && (!stageBValid || w0Granted)
  val r2Granted = myR2Req && !io.lcb_r2_req.valid

  //@req-spec-lsu.m13
  //@req-spec-lsu.m14
  io.vrf_r2_req.valid := io.lcb_r2_req.valid || myR2Req
  io.vrf_r2_req.bits  := Mux(io.lcb_r2_req.valid, io.lcb_r2_req.bits, aRow.src_prn)
  io.lcb_r2_data      := io.vrf_r2_data

  io.vrf_w0.valid     := io.lcb_w0.valid || w0MyReq
  io.vrf_w0.bits.addr := Mux(io.lcb_w0.valid, io.lcb_w0.bits.addr, stageBDst)
  io.vrf_w0.bits.data := Mux(io.lcb_w0.valid, io.lcb_w0.bits.data, stageBData)
  //@req-spec-lsu.m11
  //@req-spec-lsu.m12
  val allOnesMask = ((BigInt(1) << (vecVLen / 8)) - 1).U((vecVLen / 8).W)
  io.vrf_w0.bits.mask := Mux(io.lcb_w0.valid, io.lcb_w0.bits.mask, allOnesMask)

  //@req-spec-lsu.m8
  val copyDoneValid   = w0Granted && stageBLast
  val copyDonePvdest  = VecInit((0 until maxVecMembers).map(i =>
    Mux(i.U === doneCount, stageBDst, doneMembers(i))))
  val copyDoneMembers = doneCount + 1.U
  val copyDoneRobIdx  = stageBRobIdx

  io.group_done.valid          := noCopyValid || copyDoneValid
  io.group_done.bits.pvdest    := Mux(noCopyValid, noCopyPvdest, copyDonePvdest)
  io.group_done.bits.members   := Mux(noCopyValid, noCopyMembers, copyDoneMembers)
  io.group_done.bits.rob_idx   := Mux(noCopyValid, noCopyRobIdx, copyDoneRobIdx)
  io.group_done.bits.pvl.valid := false.B
  io.group_done.bits.pvl.bits  := DontCare

  when (io.flush) {
    pendHead     := 0.U
    pendTail     := 0.U
    readHead     := 0.U
    aValid       := false.B
    stageBValid  := false.B
    expandValid  := false.B
    doneCount    := 0.U
  } .otherwise {
    // Squash reaches the EXPANSION REGISTER as well as the enqueued rows: a launch
    // still mid-expansion would otherwise keep pushing members of a killed group,
    // whose pvdest PRNs the free list has already recycled.
    when (io.squash.valid && expandValid && IdxAgeYt(expandLdqIdx, io.squash.bits)) {
      expandValid := false.B
    }
    val tailAfterSquash = Mux(io.squash.valid, squashNewTail, pendTail)
    val tailAfterPush   = Mux(expandValid, wrapAdd(tailAfterSquash, 1.U), tailAfterSquash)
    pendTail := tailAfterPush

    when (expandValid) {
      pendRows(phys(pendTail)) := pushRow
    }

    when (acceptNewCopy) {
      expandValid  := true.B
      expandDst    := uop.pvdest.get
      expandSrc    := uop.stale_pvdest.get
      expandCount  := uop.v_emul.get
      expandCursor := 0.U
      expandRobIdx := uop.rob_idx
      expandLdqIdx := uop.ldq_idx
    } .elsewhen (expandValid) {
      when (pushRow.last) {
        expandValid := false.B
      } .otherwise {
        expandCursor := expandCursor + 1.U
      }
    }

    when (r2Granted) {
      aValid  := true.B
      aDst    := aRow.dst_prn
      aSrc    := aRow.src_prn
      aLast   := aRow.last
      aRobIdx := aRow.rob_idx
      readHead := wrapAdd(readHead, 1.U)
    } .otherwise {
      aValid := false.B
    }

    when (aValid) {
      stageBValid  := true.B
      stageBData   := io.vrf_r2_data
      stageBDst    := aDst
      stageBSrc    := aSrc
      stageBLast   := aLast
      stageBRobIdx := aRobIdx
    } .elsewhen (w0Granted) {
      stageBValid := false.B
    }

    when (w0Granted) {
      pendHead := wrapAdd(pendHead, 1.U)
      when (stageBLast) {
        doneCount := 0.U
      } .otherwise {
        doneMembers(doneCount) := stageBDst
        doneCount := doneCount + 1.U
      }
    }
  }

  when (launchFire) {
    VecTrace.trace("VecGroupCopy", "launch", uop, Seq(
      ("vl_zero", vlZero), ("all_inactive", allInactive),
      ("vta", vta), ("vma", vma), ("must_preserve", mustPreserve),
      ("members", uop.v_emul.get)))
  }
  when (w0Granted) {
    VecTrace.traceId("VecGroupCopy", "member_write", stageBRobIdx,
      Seq(("src_prn", stageBSrc), ("dst_prn", stageBDst), ("idx", doneCount)))
  }
  when (myR2Req && io.lcb_r2_req.valid) {
    VecTrace.traceId("VecGroupCopy", "r2_blocked", aRow.rob_idx, Seq(("src_prn", aRow.src_prn)))
  }
  when (w0MyReq && io.lcb_w0.valid) {
    VecTrace.traceId("VecGroupCopy", "w0_blocked", stageBRobIdx, Seq(("dst_prn", stageBDst)))
  }
  when (io.group_done.valid) {
    VecTrace.traceId("VecGroupCopy", "group_done", io.group_done.bits.rob_idx,
      Seq(("members", io.group_done.bits.members)))
  }

  //@formal-anchor VecGroupCopyChecks
  layer.block(BoomSvaLayer) {
    VecGroupCopyChecks(
      needCopy       = needCopy,
      noCopyFire     = noCopyFire,
      noCopyValid    = noCopyValid,
      vlZero         = io.launch.bits.vl_zero,
      allInactive    = io.launch.bits.all_inactive,
      gcFlush        = io.flush,
      w0Granted      = w0Granted,
      stageBDst      = stageBDst,
      stageBSrc      = stageBSrc,
      copyDoneValid  = copyDoneValid,
      groupDoneValid = io.group_done.valid,
      lcbR2Valid     = io.lcb_r2_req.valid,
      lcbR2Bits      = io.lcb_r2_req.bits,
      vrfR2Bits      = io.vrf_r2_req.bits,
      lcbW0Valid     = io.lcb_w0.valid,
      lcbW0Addr      = io.lcb_w0.bits.addr,
      vrfW0Addr      = io.vrf_w0.bits.addr,
      vrfW0Mask      = io.vrf_w0.bits.mask,
      allOnesMask    = allOnesMask
    )
  }
}

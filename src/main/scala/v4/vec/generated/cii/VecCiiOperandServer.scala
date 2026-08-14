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

package boom.v4.vec.generated.cii

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common.{BoomBundle, BoomModule}
import boom.v4.vec.generated.{CiiSrcReq, CiiSrcData, VecTrace}

// GENERATED from src/main/nlhdl/vec/cii/VecCiiOperandServer.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecCiiSrcCtrl(implicit p: Parameters) extends BoomBundle
{
  val valid  = Bool()
  val sel    = UInt(2.W)
  val scalar = UInt(vecELen.W)
}

class VecCiiOperandServerIO(val numSrcLanes: Int)(implicit p: Parameters) extends BoomBundle
{
  val src_req       = Vec(numSrcLanes, Flipped(Valid(new CiiSrcReq)))
  val req_credit    = Output(Bool())
  val src_lookup    = Vec(numSrcLanes, Flipped(new VecCiiSrcLookupIO))
  val vrf_read_addr = Vec(numSrcLanes, Output(UInt(vecPregSz.W)))
  val vrf_read_data = Vec(numSrcLanes, Input(UInt(vecVLen.W)))
  val src_data      = Vec(numSrcLanes, Output(Valid(new CiiSrcData)))
}

class VecCiiOperandServer(
  val numSrcLanes:    Int = 4,
  val srcReadLatency: Int = 1)
  (implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecCiiOperandServer: elaborated only under usingRVV (never rocket's usingVector)")
  require(numSrcLanes == 4,
    s"VecCiiOperandServer: numSrcLanes ($numSrcLanes) must equal CII_NUM_SRC_REQ = CII_NUM_SRC_DAT_RSP = 4")
  require(srcReadLatency >= 1 && srcReadLatency <= 2,
    s"VecCiiOperandServer: srcReadLatency ($srcReadLatency) must be 1..2")

  //@req-spec-cii.f32
  require(log2Ceil(maxVecMembers) == (new CiiSrcReq).op_offset.getWidth,
    s"VecCiiOperandServer: op_offset width (${(new CiiSrcReq).op_offset.getWidth}) must equal " +
    s"log2Ceil(maxVecMembers) (${log2Ceil(maxVecMembers)})")

  val io = IO(new VecCiiOperandServerIO(numSrcLanes))

  val SEL_VRF      = 0.U(2.W)
  val SEL_SCALAR   = 1.U(2.W)
  val SEL_DONTCARE = 2.U(2.W)

  //@req-spec-cii.f1
  //@req-spec-vrf.h7
  val beatValid = io.src_req(0).valid
  for (i <- 1 until numSrcLanes) {
    assert(io.src_req(i).valid === beatValid,
      "VecCiiOperandServer: lane request valids disagree -- one valid must qualify the whole beat")
  }

  //@req-spec-cii.e13
  io.req_credit := beatValid
  assert(io.req_credit === beatValid,
    "VecCiiOperandServer: req_credit must be asserted exactly once per accepted beat")

  val vrfAddrReg = RegInit(VecInit(Seq.fill(numSrcLanes)(0.U(vecPregSz.W))))
  val ctrlZero   = 0.U.asTypeOf(new VecCiiSrcCtrl)

  //@req-spec-cii.f23
  //@req-spec-cii.f44
  //@req-spec-cii.f45
  val ctrlStages = RegInit(VecInit(Seq.fill(srcReadLatency)(VecInit(Seq.fill(numSrcLanes)(ctrlZero)))))

  //@req-spec-cii.f46
  for (i <- 0 until numSrcLanes) {
    val req  = io.src_req(i).bits
    val resp = io.src_lookup(i).resp

    //@req-spec-cii.f2
    //@req-spec-cii.f3
    io.src_lookup(i).req.tag       := req.tag
    //@req-spec-cii.f12
    //@req-spec-vrf.j13
    //@req-spec-vrf.j14
    //@req-spec-cii.f13
    //@req-spec-cii.f14
    //@req-spec-vrf.j15
    io.src_lookup(i).req.op_id     := req.op_id
    //@req-spec-cii.f33
    //@req-spec-cii.f34
    io.src_lookup(i).req.op_offset := req.op_offset

    //@req-spec-cii.e13
    //@req-spec-cii.e14
    //@req-spec-cii.e15
    val isKilled = resp.killed
    val sel = Wire(UInt(2.W))
    when (isKilled) {
      sel := SEL_DONTCARE
    } .elsewhen (resp.read_vrf) {
      //@req-spec-vrf.i8
      //@req-spec-vrf.i9
      sel := SEL_VRF
    } .elsewhen (req.op_id === 5.U) {
      //@req-spec-cii.f9
      sel := SEL_SCALAR
    } .otherwise {
      //@req-spec-cii.f4
      sel := SEL_DONTCARE
    }

    assert(!(req.op_id === 5.U && resp.read_vrf),
      "VecCiiOperandServer: a SCALAR request must never drive a VRF read")
    assert(!beatValid || req.op_id =/= 7.U,
      "VecCiiOperandServer: op_id 7 is unassigned and must never appear on the wire")

    val doVrfRead = resp.read_vrf && !isKilled

    //@req-spec-cii.f21
    //@req-spec-cii.f22
    when (beatValid && doVrfRead) {
      vrfAddrReg(i) := resp.prn
    }
    io.vrf_read_addr(i) := vrfAddrReg(i)

    val newCtrl = Wire(new VecCiiSrcCtrl)
    newCtrl.valid  := beatValid
    newCtrl.sel    := sel
    newCtrl.scalar := resp.scalar_data
    ctrlStages(0)(i) := newCtrl

    when (beatValid) {
      VecTrace.traceId("VecCiiOperandServer", "src_serve", resp.rob_idx, Seq(
        ("tag",    req.tag),
        ("lane",   i.U),
        ("op_id",  req.op_id),
        ("off",    req.op_offset),
        ("prn",    resp.prn),
        ("killed", isKilled)))
    }
  }

  for (s <- 1 until srcReadLatency) {
    ctrlStages(s) := ctrlStages(s - 1)
  }

  for (i <- 0 until numSrcLanes) {
    val ctrl = ctrlStages(srcReadLatency - 1)(i)

    io.src_data(i).valid := ctrl.valid

    //@req-spec-cii.f31
    io.src_data(i).bits.data := MuxCase(0.U(vecVLen.W), Seq(
      (ctrl.sel === SEL_VRF)    -> io.vrf_read_data(i),
      (ctrl.sel === SEL_SCALAR) -> ctrl.scalar.pad(vecVLen)))

    val reqValidDelayed = ShiftRegister(io.src_req(i).valid, srcReadLatency, false.B, true.B)
    assert(io.src_data(i).valid === reqValidDelayed,
      "VecCiiOperandServer: src_data(i).valid in cycle t+srcReadLatency must equal src_req(i).valid of cycle t")
  }
}

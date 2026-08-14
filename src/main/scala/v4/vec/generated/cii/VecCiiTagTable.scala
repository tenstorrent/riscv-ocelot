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
import boom.v4.vec.generated.VecTrace

// GENERATED from src/main/nlhdl/vec/cii/VecCiiTagTable.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

//@req-spec-cii.d16
//@req-spec-cii.d17
//@req-spec-cii.d18
//@req-spec-cii.d19
//@req-spec-cii.d20
//@req-spec-cii.d21
//@req-spec-cii.d22
//@req-spec-cii.d23
class VecCiiTagEntry(implicit p: Parameters) extends BoomBundle
{
  val tag              = UInt(ciiTagBits.W)
  val rob_idx          = UInt(robAddrSz.W)
  val is_shared        = Bool()
  val pvdest_grp       = Vec(maxVecMembers, UInt(vecPregSz.W))
  val pvdest_grp_mask  = UInt(maxVecMembers.W)
  val pvs1_grp         = Vec(maxVecMembers, UInt(vecPregSz.W))
  val pvs2_grp         = Vec(maxVecMembers, UInt(vecPregSz.W))
  val pvs3_grp         = Vec(maxVecMembers, UInt(vecPregSz.W))
  val pvm              = UInt(vecPregSz.W)
  val stale_pvdest_grp = Vec(maxVecMembers, UInt(vecPregSz.W))
  val pdst             = UInt(maxPregSz.W)
  val scalar_operands  = UInt(vecELen.W)
  // SEW of the issuing op, in MicroOp.v_eew's 2-bit encoding. Carried purely so
  // an FP-scalar-dest writeback (vfmv.f.s) can be recoded at the right width:
  // the coprocessor names no register and the uop is long gone by writeback, so
  // the side-table is the only source. See VecCiiWriteback's fp leg.
  val v_eew            = UInt(2.W)
}

class VecCiiSrcLookupReq(implicit p: Parameters) extends BoomBundle
{
  val tag       = UInt(ciiTagBits.W)
  val op_id     = UInt(3.W)
  val op_offset = UInt(log2Ceil(maxVecMembers).W)
}

class VecCiiSrcLookupResp(implicit p: Parameters) extends BoomBundle
{
  val prn         = UInt(vecPregSz.W)
  val read_vrf    = Bool()
  val scalar_data = UInt(vecELen.W)
  val killed      = Bool()
  val rob_idx     = UInt(robAddrSz.W)
}

class VecCiiWbLookupReq(implicit p: Parameters) extends BoomBundle
{
  val tag           = UInt(ciiTagBits.W)
  val wb_dst_offset = UInt(log2Ceil(maxVecMembers).W)
}

class VecCiiWbLookupResp(implicit p: Parameters) extends BoomBundle
{
  val prn        = UInt(vecPregSz.W)
  val wr_en      = Bool()
  val pvdest_grp = Vec(maxVecMembers, UInt(vecPregSz.W))
  val members    = UInt((log2Ceil(maxVecMembers) + 1).W)
  val rob_idx    = UInt(robAddrSz.W)
  val pdst       = UInt(maxPregSz.W)
  val is_shared  = Bool()
  val killed     = Bool()
  val v_eew      = UInt(2.W)
}

class VecCiiSrcLookupIO(implicit p: Parameters) extends BoomBundle
{
  val req  = Input(new VecCiiSrcLookupReq)
  val resp = Output(new VecCiiSrcLookupResp)
}

class VecCiiWbLookupIO(implicit p: Parameters) extends BoomBundle
{
  val req  = Input(new VecCiiWbLookupReq)
  val resp = Output(new VecCiiWbLookupResp)
}

class VecCiiTagTableIO(
  val nTags:          Int,
  val tagBits:        Int,
  val numSrcReqLanes: Int,
  val numWbLanes:     Int)
  (implicit p: Parameters) extends BoomBundle
{
  val alloc         = Input(Valid(new VecCiiTagEntry))
  val tag_free_mask = Output(UInt(nTags.W))
  val src_lookup    = Vec(numSrcReqLanes, new VecCiiSrcLookupIO)
  val wb_lookup     = Vec(numWbLanes, new VecCiiWbLookupIO)
  val free          = Input(Valid(UInt(tagBits.W)))
  val kill_all      = Input(Bool())
  val debug = new Bundle {
    val valid  = Output(UInt(nTags.W))
    val killed = Output(UInt(nTags.W))
  }
}

class VecCiiTagTable(
  val nTags:          Int = 16,
  val tagBits:        Int = 4,
  val numSrcReqLanes: Int = 4,
  val numWbLanes:     Int = 1,
  val maxMembers:     Int = 8)
  (implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecCiiTagTable: elaborated only under usingRVV (never rocket's usingVector)")
  require(tagBits == log2Ceil(nTags),
    s"VecCiiTagTable: tagBits ($tagBits) must equal log2Ceil(nTags) (${log2Ceil(nTags)})")
  require(numSrcReqLanes >= 1 && numSrcReqLanes <= 4,
    s"VecCiiTagTable: numSrcReqLanes ($numSrcReqLanes) must be 1..4")
  require(numWbLanes == 1,
    s"VecCiiTagTable: numWbLanes ($numWbLanes) must be 1")
  require(maxMembers == vectorParams.maxMembers,
    s"VecCiiTagTable: maxMembers ($maxMembers) must equal vectorParams.maxMembers (${vectorParams.maxMembers})")

  val io = IO(new VecCiiTagTableIO(nTags, tagBits, numSrcReqLanes, numWbLanes))

  //@req-spec-cii.b11
  val entries    = Reg(Vec(nTags, new VecCiiTagEntry))
  val tag_valid  = RegInit(0.U(nTags.W))
  val tag_killed = RegInit(0.U(nTags.W))

  val allocMask = Mux(io.alloc.valid, UIntToOH(io.alloc.bits.tag, nTags), 0.U(nTags.W))
  val freeMask  = Mux(io.free.valid, UIntToOH(io.free.bits, nTags), 0.U(nTags.W))
  val killMask  = Mux(io.kill_all, tag_valid, 0.U(nTags.W))

  //@req-spec-cii.d8
  //@req-spec-cii.f11
  when (io.alloc.valid) {
    entries(io.alloc.bits.tag) := io.alloc.bits
  }

  tag_valid  := (tag_valid | allocMask) & ~freeMask
  tag_killed := (tag_killed | killMask) & ~allocMask

  io.tag_free_mask := ~tag_valid

  assert(!io.alloc.valid || !tag_valid(io.alloc.bits.tag),
    "VecCiiTagTable: alloc.valid on a tag whose tag_valid bit is already set")

  //@req-spec-cii.b8
  //@req-spec-cii.b11
  //@req-spec-cii.d20
  //@req-spec-cii.d21
  //@req-spec-cii.d23
  //@req-spec-cii.f5
  //@req-spec-cii.f6
  //@req-spec-cii.f7
  //@req-spec-cii.f8
  //@req-spec-cii.f10
  //@req-spec-cii.f38
  //@req-spec-cii.f11
  for (i <- 0 until numSrcReqLanes) {
    val req  = io.src_lookup(i).req
    val resp = io.src_lookup(i).resp
    val e    = entries(req.tag)

    resp.prn         := 0.U
    resp.read_vrf    := false.B
    resp.scalar_data := 0.U
    resp.killed       := tag_killed(req.tag)
    resp.rob_idx      := e.rob_idx

    switch (req.op_id) {
      is (0.U) {
        resp.read_vrf := false.B
      }
      is (1.U) {
        resp.prn      := e.pvs1_grp(req.op_offset)
        resp.read_vrf := true.B
      }
      is (2.U) {
        resp.prn      := e.pvs2_grp(req.op_offset)
        resp.read_vrf := true.B
      }
      is (3.U) {
        resp.prn      := e.pvs3_grp(req.op_offset)
        resp.read_vrf := true.B
      }
      is (4.U) {
        resp.prn      := e.pvm
        resp.read_vrf := true.B
      }
      is (5.U) {
        resp.scalar_data := e.scalar_operands
        resp.read_vrf     := false.B
      }
      is (6.U) {
        resp.prn      := e.stale_pvdest_grp(req.op_offset)
        resp.read_vrf := true.B
      }
    }

    VecTrace.traceId("VecCiiTagTable", "src", e.rob_idx, Seq(
      ("tag",       req.tag),
      ("op_id",     req.op_id),
      ("op_offset", req.op_offset),
      ("prn",       resp.prn),
      ("killed",    resp.killed)))
  }

  //@req-spec-cii.d19
  //@req-spec-cii.d22
  for (j <- 0 until numWbLanes) {
    val req  = io.wb_lookup(j).req
    val resp = io.wb_lookup(j).resp
    val e    = entries(req.tag)

    resp.prn        := e.pvdest_grp(req.wb_dst_offset)
    resp.wr_en      := e.pvdest_grp_mask(req.wb_dst_offset)
    resp.pvdest_grp := e.pvdest_grp
    resp.members    := PopCount(e.pvdest_grp_mask)
    resp.rob_idx     := e.rob_idx
    resp.pdst        := e.pdst
    resp.is_shared   := e.is_shared
    resp.killed      := tag_killed(req.tag)
    resp.v_eew       := e.v_eew

    VecTrace.traceId("VecCiiTagTable", "wb", e.rob_idx, Seq(
      ("tag",           req.tag),
      ("wb_dst_offset", req.wb_dst_offset),
      ("prn",           resp.prn),
      ("wr_en",         resp.wr_en)))
  }

  //@req-spec-cii.e23
  //@req-spec-cii.e24
  when (io.free.valid) {
    VecTrace.traceStruct("VecCiiTagTable", "free", Seq(("tag", io.free.bits)))
  }

  assert(!io.free.valid || tag_valid(io.free.bits),
    "VecCiiTagTable: free.valid on a tag whose tag_valid bit is not set (double free)")

  //@req-spec-cii.e22
  when (io.kill_all) {
    VecTrace.traceStruct("VecCiiTagTable", "kill", Seq(("tag_valid", tag_valid)))
  }

  when (io.alloc.valid) {
    val pvdestFields: Seq[(String, UInt)] =
      (0 until maxVecMembers).map(m => (s"pv$m", io.alloc.bits.pvdest_grp(m)))
    VecTrace.traceId("VecCiiTagTable", "alloc", io.alloc.bits.rob_idx,
      Seq(("tag", io.alloc.bits.tag), ("nmem", PopCount(io.alloc.bits.pvdest_grp_mask))) ++ pvdestFields)
  }

  io.debug.valid  := tag_valid
  io.debug.killed := tag_killed
}

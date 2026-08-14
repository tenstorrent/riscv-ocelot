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

import boom.v4.common._
import boom.v4.exu.ExeUnitResp
import boom.v4.vec.generated.{CiiSrcReq, CiiWbStatus, VecGroupDone, VecRobFlags, IntWbSnoop, VecTrace}

// GENERATED from src/main/nlhdl/vec/cii/VecCiiHost.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

object TtCiiCaracalPkg
{
  val CII_VLEN            = 256
  val CII_TAG_W           = 4
  val CII_N_TAGS          = 16
  val CII_MEMBER_W        = 3
  val CII_MAX_MEMBERS     = 8
  val CII_VL_W            = 9
  val CII_NUM_INST_ISSUE  = 1
  val CII_NUM_SRC_REQ     = 4
  val CII_NUM_SRC_DAT_RSP = 4
  val CII_NUM_DST_WB      = 1
  val CII_NUM_SRC_SLOTS   = 4
  val CII_N_ISS_CREDITS   = 16
  val CII_N_REQ_CREDITS   = 16
  val CII_N_DAT_CREDITS   = 16
  val CII_N_WB_CREDITS    = 16
}

//@req-spec-cii.c3
class TTCiiIO extends Bundle
{
  val clk         = Input(Clock())
  val core_reset  = Input(Bool())

  val iss_valid   = Input(Bool())
  val iss_tag     = Input(UInt(4.W))
  val iss_insn    = Input(UInt(32.W))
  val iss_vtype   = Input(UInt(8.W))
  val iss_vl      = Input(UInt(9.W))
  val iss_vstart  = Input(UInt(9.W))
  val iss_vxrm    = Input(UInt(2.W))
  val iss_frm     = Input(UInt(3.W))
  val iss_hint    = Input(UInt(4.W))
  val iss_credit  = Output(Bool())

  val req_valid      = Output(Bool())
  val req_tag        = Output(UInt(16.W))
  val req_op_id      = Output(UInt(12.W))
  val req_op_offset  = Output(UInt(12.W))
  val req_credit     = Input(Bool())

  val dat_valid   = Input(Bool())
  val dat_data    = Input(UInt(1024.W))
  val dat_credit  = Output(Bool())

  val wb_valid       = Output(Bool())
  val wb_tag         = Output(UInt(4.W))
  val wb_data        = Output(UInt(256.W))
  val wb_dst_offset  = Output(UInt(3.W))
  val wb_wr_en       = Output(Bool())
  val wb_status      = Output(UInt(9.W))
  val wb_credit      = Input(Bool())
}

//@req-spec-cii.c1
class TTCii extends BlackBox with HasBlackBoxPath with HasBlackBoxResource
{
  val io = IO(new TTCiiIO)

  override def desiredName = "tt_cii_host_wrap"

  private val chipyardDir = System.getProperty("user.dir")
  private val svRoot = s"$chipyardDir/generators/boom/src/main/sv/v4"

  require(new java.io.File(s"$svRoot/tt-cii/src/tt_cii.sv").exists(),
    "TTCii: the tt-cii submodule is not checked out under generators/boom/src/main/sv/v4/tt-cii -- " +
    "init the submodule before elaborating a usingRVV config")

  addResource("/HardFloat/source/addRecFN.v")
  addResource("/HardFloat/source/compareRecFN.v")
  addResource("/HardFloat/source/divSqrtRecFN_small.v")
  addResource("/HardFloat/source/fNToRecFN.v")
  addResource("/HardFloat/source/HardFloat_primitives.v")
  addResource("/HardFloat/source/HardFloat_rawFN.v")
  addResource("/HardFloat/source/iNToRecFN.v")
  addResource("/HardFloat/source/isSigNaNRecFN.v")
  addResource("/HardFloat/source/mulAddRecFN.v")
  addResource("/HardFloat/source/mulRecFN.v")
  addResource("/HardFloat/source/recFNToFN.v")
  addResource("/HardFloat/source/recFNToIN.v")
  addResource("/HardFloat/source/recFNToRecFN.v")
  addResource("/HardFloat/source/RISCV/HardFloat_specialize.v")
  addResource("/HardFloat/source/HardFloat_consts.vi")
  addResource("/HardFloat/source/HardFloat_localFuncs.vi")
  addResource("/HardFloat/source/RISCV/HardFloat_specialize.vi")

  addPath(s"$svRoot/common/utility/tt_cam_buffer.sv")
  addPath(s"$svRoot/common/utility/tt_compare.sv")
  addPath(s"$svRoot/common/utility/tt_ffs.sv")
  addPath(s"$svRoot/common/utility/tt_fifo.sv")
  addPath(s"$svRoot/common/utility/tt_pipe_stage.sv")
  addPath(s"$svRoot/common/utility/tt_popcnt.sv")
  addPath(s"$svRoot/common/utility/tt_reshape.sv")
  addPath(s"$svRoot/common/utility/tt_rts_rtr_pipe_stage.sv")
  addPath(s"$svRoot/common/utility/tt_skid_buffer.sv")

  addPath(s"$svRoot/common/arithmetic/tt_fp16_div.sv")
  addPath(s"$svRoot/common/arithmetic/tt_fp32_div.sv")
  addPath(s"$svRoot/common/arithmetic/tt_int_div_r2.sv")
  addPath(s"$svRoot/common/arithmetic/tt_int_div_simple.sv")
  addPath(s"$svRoot/common/arithmetic/VecFP16rec7.sv")
  addPath(s"$svRoot/common/arithmetic/VecFP16rsqrt7.sv")
  addPath(s"$svRoot/common/arithmetic/VecFP32rec7.sv")
  addPath(s"$svRoot/common/arithmetic/VecFP32rsqrt7.sv")

  addPath(s"$svRoot/tt-cii/src/rv_async_rst_dff.sv")
  addPath(s"$svRoot/tt-cii/src/rv_async_rst_dff_Tdat.sv")
  addPath(s"$svRoot/tt-cii/src/tt_cii_fifo.sv")
  addPath(s"$svRoot/tt-cii/src/tt_cii_channel.sv")
  addPath(s"$svRoot/tt-cii/src/tt_cii_interface.sv")
  addPath(s"$svRoot/tt-cii/src/tt_cii.sv")

  addPath(s"$svRoot/vpu/decoder/autogen_riscv_imabfv.v")
  addPath(s"$svRoot/vpu/decoder/tt_ascii_instrn_decode.sv")
  addPath(s"$svRoot/vpu/decoder/tt_decoded_mux.sv")
  addPath(s"$svRoot/vpu/decoder/tt_decoder.sv")
  addPath(s"$svRoot/vpu/decoder/tt_id.sv")

  addPath(s"$svRoot/vpu/execution/int_datapath_unit/tt_vec_div_unit.sv")
  addPath(s"$svRoot/vpu/execution/int_datapath_unit/tt_vec_iadd.sv")
  addPath(s"$svRoot/vpu/execution/int_datapath_unit/tt_vec_idp.sv")
  addPath(s"$svRoot/vpu/execution/int_datapath_unit/tt_vec_imul.sv")
  addPath(s"$svRoot/vpu/execution/int_datapath_unit/tt_vec_mul_dp.sv")
  addPath(s"$svRoot/vpu/execution/fp_datapath_unit/tt_vfp_encoder_lane.sv")
  addPath(s"$svRoot/vpu/execution/fp_datapath_unit/tt_vfp_encoder.sv")
  addPath(s"$svRoot/vpu/execution/fp_datapath_unit/tt_vfp_ex_unit.sv")
  addPath(s"$svRoot/vpu/execution/fp_datapath_unit/tt_vfp_fma.sv")
  addPath(s"$svRoot/vpu/execution/fp_datapath_unit/tt_vfp_lane.sv")
  addPath(s"$svRoot/vpu/execution/fp_datapath_unit/tt_vfp_red.sv")
  addPath(s"$svRoot/vpu/execution/fp_datapath_unit/tt_vfp_unit.sv")

  addPath(s"$svRoot/vpu/reg/tt_vec_regfile.sv")
  addPath(s"$svRoot/vpu/tt_vec_top.sv")
  addPath(s"$svRoot/vpu/tt_vpu_cii_wrapper_top.sv")

  addPath(s"$svRoot/generated/tt_cii_host_wrap.sv")
  addPath(s"$svRoot/tt-cii/src/tt_cii_caracal_pkg.svh")
  addPath(s"$svRoot/vpu/packages/tt_briscv_pkg.svh")
  addPath(s"$svRoot/vpu/decoder/briscv_defines.h")
  addPath(s"$svRoot/vpu/decoder/autogen_defines.h")
}

class VecCiiHostIO(implicit p: Parameters) extends BoomBundle
{
  val iss      = Input(Valid(new MicroOp))
  val fu_types = Output(UInt(FC_SZ.W))

  val vl_read_addr = Output(UInt(vlPregSz.W))
  val vl_read_data = Input(UInt(vecVLSz.W))

  val int_scalar_read_req = Decoupled(UInt(maxPregSz.W))
  val int_scalar_read_rsp = Input(UInt(xLen.W))
  val fp_scalar_read_req  = Decoupled(UInt(maxPregSz.W))
  val fp_scalar_read_rsp  = Input(UInt(xLen.W))
  val int_wb_snoop        = Input(Vec(numIrfWritePorts, Valid(new IntWbSnoop)))

  val csr_vstart = Input(UInt(8.W))
  val csr_vxrm   = Input(UInt(2.W))
  val csr_frm    = Input(UInt(3.W))

  val vrf_read_addr = Vec(TtCiiCaracalPkg.CII_NUM_SRC_REQ, Output(UInt(vecPregSz.W)))
  val vrf_read_data = Vec(TtCiiCaracalPkg.CII_NUM_SRC_REQ, Input(UInt(vecVLen.W)))
  val vrf_write     = Output(Valid(new VecCiiVrfWrite))

  val group_done = Output(Valid(new VecGroupDone))
  val clr_rob    = Output(Valid(UInt(robAddrSz.W)))
  val rob_flags  = Output(Valid(new VecRobFlags))
  val int_wb     = Output(Valid(new ExeUnitResp(xLen)))
  val fp_wb      = Output(Valid(new ExeUnitResp(xLen)))

  val rob_flush           = Input(Bool())
  val rob_flush_kill      = Input(Bool())
  val brupdate_mispredict = Input(Bool())
}

class VecCiiHost(implicit p: Parameters) extends BoomModule
{
  //@req-spec-cii.a1
  require(usingRVV, "VecCiiHost: elaborated only under usingRVV (never rocket's usingVector)")

  require(ciiTagBits == log2Ceil(TtCiiCaracalPkg.CII_N_TAGS),
    s"VecCiiHost: ciiTagBits ($ciiTagBits) must equal log2Ceil(CII_N_TAGS) (${log2Ceil(TtCiiCaracalPkg.CII_N_TAGS)})")
  require(TtCiiCaracalPkg.CII_N_TAGS == TtCiiCaracalPkg.CII_N_ISS_CREDITS,
    s"VecCiiHost: CII_N_TAGS (${TtCiiCaracalPkg.CII_N_TAGS}) must equal CII_N_ISS_CREDITS (${TtCiiCaracalPkg.CII_N_ISS_CREDITS})")
  require(TtCiiCaracalPkg.CII_NUM_SRC_REQ == TtCiiCaracalPkg.CII_NUM_SRC_DAT_RSP,
    s"VecCiiHost: CII_NUM_SRC_REQ (${TtCiiCaracalPkg.CII_NUM_SRC_REQ}) must equal CII_NUM_SRC_DAT_RSP (${TtCiiCaracalPkg.CII_NUM_SRC_DAT_RSP})")
  require(TtCiiCaracalPkg.CII_NUM_DST_WB == 1,
    s"VecCiiHost: CII_NUM_DST_WB (${TtCiiCaracalPkg.CII_NUM_DST_WB}) must be 1")
  require(maxVecMembers == TtCiiCaracalPkg.CII_MAX_MEMBERS,
    s"VecCiiHost: maxVecMembers ($maxVecMembers) must equal CII_MAX_MEMBERS (${TtCiiCaracalPkg.CII_MAX_MEMBERS})")
  require(log2Ceil(maxVecMembers) == TtCiiCaracalPkg.CII_MEMBER_W,
    s"VecCiiHost: log2Ceil(maxVecMembers) (${log2Ceil(maxVecMembers)}) must equal CII_MEMBER_W (${TtCiiCaracalPkg.CII_MEMBER_W})")
  require(vecVLSz == TtCiiCaracalPkg.CII_VL_W,
    s"VecCiiHost: vecVLSz ($vecVLSz) must equal CII_VL_W (${TtCiiCaracalPkg.CII_VL_W})")
  require(vecVLen == TtCiiCaracalPkg.CII_VLEN,
    s"VecCiiHost: vecVLen ($vecVLen) must equal CII_VLEN (${TtCiiCaracalPkg.CII_VLEN})")

  val io = IO(new VecCiiHostIO)

  val tags = Module(new VecCiiTagTable(
    nTags          = TtCiiCaracalPkg.CII_N_TAGS,
    tagBits        = ciiTagBits,
    numSrcReqLanes = TtCiiCaracalPkg.CII_NUM_SRC_REQ,
    numWbLanes     = TtCiiCaracalPkg.CII_NUM_DST_WB,
    maxMembers     = maxVecMembers))

  val iss = Module(new VecCiiIssue(
    numCiiTags      = TtCiiCaracalPkg.CII_N_TAGS,
    ciiIssueCredits = TtCiiCaracalPkg.CII_N_ISS_CREDITS,
    numIssueLanes   = TtCiiCaracalPkg.CII_NUM_INST_ISSUE))

  val opnd = Module(new VecCiiOperandServer(
    numSrcLanes    = TtCiiCaracalPkg.CII_NUM_SRC_REQ,
    srcReadLatency = 1))

  val wb = Module(new VecCiiWriteback)

  val done = Module(new VecCiiComplete)

  val flush = Module(new VecCiiFlush(nTags = TtCiiCaracalPkg.CII_N_TAGS))

  val coproc = Module(new TTCii)

  coproc.io.clk        := clock
  coproc.io.core_reset := reset.asBool

  private val srcIdBits  = (new CiiSrcReq).op_id.getWidth
  private val memberBits = (new CiiSrcReq).op_offset.getWidth

  iss.io.iss := io.iss
  io.fu_types := iss.io.fu_types

  //@req-spec-cii.b5
  //@req-spec-cii.b7
  //@req-spec-cii.f35
  //@req-spec-cii.f36
  tags.io.alloc := iss.io.tag_alloc
  iss.io.tag_free_mask := tags.io.tag_free_mask

  iss.io.vl_read_data := io.vl_read_data
  io.vl_read_addr := iss.io.vl_read_addr

  //@req-spec-cii.a18
  //@req-spec-cii.f39
  io.vrf_read_addr := opnd.io.vrf_read_addr
  opnd.io.vrf_read_data := io.vrf_read_data
  io.vrf_write := wb.io.vrf_write

  io.int_scalar_read_req.valid := io.iss.valid && io.iss.bits.lrs1_rtype === RT_FIX
  io.int_scalar_read_req.bits  := iss.io.int_scalar_read_req
  assert(!io.int_scalar_read_req.valid || io.int_scalar_read_req.ready)
  iss.io.int_scalar_read_rsp := io.int_scalar_read_rsp

  io.fp_scalar_read_req.valid := io.iss.valid && io.iss.bits.lrs1_rtype === RT_FLT
  io.fp_scalar_read_req.bits  := iss.io.fp_scalar_read_req
  assert(!io.fp_scalar_read_req.valid || io.fp_scalar_read_req.ready)
  iss.io.fp_scalar_read_rsp := io.fp_scalar_read_rsp

  iss.io.int_wb_snoop := io.int_wb_snoop

  iss.io.csr_vstart := io.csr_vstart
  iss.io.csr_vxrm   := io.csr_vxrm
  iss.io.csr_frm    := io.csr_frm

  iss.io.rob_flush := io.rob_flush

  //@req-spec-core.e18
  //@req-spec-agen.a7
  //@req-spec-cii.a2
  //@req-spec-cii.a4
  //@req-spec-cii.a5
  coproc.io.iss_valid  := iss.io.iss_pkt.valid
  coproc.io.iss_tag    := iss.io.iss_pkt.bits.tag
  //@req-spec-cii.f37
  coproc.io.iss_insn   := iss.io.iss_pkt.bits.instr
  coproc.io.iss_vtype  := iss.io.iss_pkt.bits.vtype
  coproc.io.iss_vl     := iss.io.iss_pkt.bits.vl
  coproc.io.iss_vstart := iss.io.iss_pkt.bits.vstart
  coproc.io.iss_vxrm   := iss.io.iss_pkt.bits.vxrm
  coproc.io.iss_frm    := iss.io.iss_pkt.bits.frm
  coproc.io.iss_hint   := iss.io.iss_pkt.bits.src_reuse_hint
  iss.io.iss_credit    := coproc.io.iss_credit

  when (coproc.io.iss_valid) {
    VecTrace.traceId("VecCiiHost", "iss_flat", iss.io.tag_alloc.bits.rob_idx, Seq(
      ("tag",  coproc.io.iss_tag),
      ("insn", coproc.io.iss_insn)))
  }

  //@req-spec-cii.b9
  //@req-spec-cii.b6
  for (i <- 0 until TtCiiCaracalPkg.CII_NUM_SRC_REQ) {
    opnd.io.src_req(i).valid         := coproc.io.req_valid
    opnd.io.src_req(i).bits.tag       := coproc.io.req_tag(i * ciiTagBits + ciiTagBits - 1, i * ciiTagBits)
    opnd.io.src_req(i).bits.op_id     := coproc.io.req_op_id(i * srcIdBits + srcIdBits - 1, i * srcIdBits)
    opnd.io.src_req(i).bits.op_offset := coproc.io.req_op_offset(i * memberBits + memberBits - 1, i * memberBits)

    tags.io.src_lookup(i).req  := opnd.io.src_lookup(i).req
    opnd.io.src_lookup(i).resp := tags.io.src_lookup(i).resp
  }
  coproc.io.req_credit := opnd.io.req_credit

  coproc.io.dat_valid := opnd.io.src_data(0).valid
  coproc.io.dat_data := Cat((TtCiiCaracalPkg.CII_NUM_SRC_DAT_RSP - 1 to 0 by -1).map(i => opnd.io.src_data(i).bits.data))

  //@req-spec-cii.b1
  //@req-spec-cii.b2
  val dat_credits = RegInit(TtCiiCaracalPkg.CII_N_DAT_CREDITS.U(log2Ceil(TtCiiCaracalPkg.CII_N_DAT_CREDITS + 1).W))
  val dat_beat_fire = opnd.io.src_data(0).valid
  dat_credits := dat_credits + coproc.io.dat_credit.asUInt - dat_beat_fire.asUInt

  assert(!dat_beat_fire || dat_credits =/= 0.U,
    "VecCiiHost: Src-Data beat presented with zero dat_credits -- VPU-side protocol violation")

  when (dat_credits === 0.U) {
    VecTrace.traceStruct("VecCiiHost", "dat_credits_exhausted", Seq(("dat_credits", dat_credits)))
  }

  val wbStatusTyped = coproc.io.wb_status.asTypeOf(new CiiWbStatus)

  wb.io.wb.valid             := coproc.io.wb_valid
  wb.io.wb.bits.tag           := coproc.io.wb_tag
  wb.io.wb.bits.wb_data       := coproc.io.wb_data
  wb.io.wb.bits.wb_dst_offset := coproc.io.wb_dst_offset
  wb.io.wb.bits.wb_wr_en      := coproc.io.wb_wr_en
  wb.io.wb.bits.wb_status     := wbStatusTyped
  coproc.io.wb_credit         := wb.io.wb_credit

  when (coproc.io.wb_valid) {
    VecTrace.traceId("VecCiiHost", "wb_flat", tags.io.wb_lookup(0).resp.rob_idx, Seq(
      ("tag",  coproc.io.wb_tag),
      ("last", wbStatusTyped.last)))
  }

  io.int_wb := wb.io.int_wb
  io.fp_wb  := wb.io.fp_wb

  //@req-spec-cii.c8
  //@req-spec-cii.c9
  tags.io.wb_lookup(0).req := wb.io.wb_lookup.req     // the ONLY driver
  wb.io.wb_lookup.resp     := tags.io.wb_lookup(0).resp
  done.io.wb_lookup.resp   := tags.io.wb_lookup(0).resp
  // done.io.wb_lookup.req is deliberately left unread.

  wb.io.wb_suppress := tags.io.wb_lookup(0).resp.killed || flush.io.kill_all

  assert(!done.io.beat.valid || done.io.wb_lookup.req.tag === wb.io.wb_lookup.req.tag,
    "VecCiiHost: done and wb must observe the same wb_lookup tag in the same cycle")

  done.io.beat := wb.io.beat
  io.group_done := done.io.group_done
  io.clr_rob    := done.io.clr_rob
  io.rob_flags  := done.io.rob_flags

  tags.io.free := done.io.free_tag
  flush.io.free := done.io.free_tag

  flush.io.rob_flush           := io.rob_flush
  flush.io.rob_flush_kill      := io.rob_flush_kill
  flush.io.brupdate_mispredict := io.brupdate_mispredict
  flush.io.tag_valid  := tags.io.debug.valid
  flush.io.tag_killed := tags.io.debug.killed
  flush.io.alloc.valid := iss.io.tag_alloc.valid
  flush.io.alloc.bits  := iss.io.tag_alloc.bits.tag
  flush.io.alloc_br_mask         := iss.io.alloc_br_mask
  flush.io.alloc_flush_on_commit := iss.io.alloc_flush_on_commit

  //@req-spec-cii.f43
  tags.io.kill_all := flush.io.kill_all
  done.io.kill_all := flush.io.kill_all
}

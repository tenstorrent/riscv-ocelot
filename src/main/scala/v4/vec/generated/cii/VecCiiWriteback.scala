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
import freechips.rocketchip.tile.FPConstants

import boom.v4.common._
import boom.v4.exu.ExeUnitResp
import boom.v4.vec.generated.{CiiWbStatus, CiiWriteback, VecBundlesConsts, VecTrace}

// GENERATED from src/main/nlhdl/vec/cii/VecCiiWriteback.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

object VecCiiDstKind
{
  val dstKindBits = VecBundlesConsts.ciiWbStatusBits - 1 - 1 - FPConstants.FLAGS_SZ

  val VEC = 0.U(dstKindBits.W)
  val INT = 1.U(dstKindBits.W)
  val FP  = 2.U(dstKindBits.W)
  val NUM_DST_WB = 1
}

class VecCiiVrfWrite(implicit p: Parameters) extends BoomBundle
{
  val addr = UInt(vecPregSz.W)
  val data = UInt(vecVLen.W)
  val mask = UInt((vecVLen / 8).W)
}

class VecCiiWbBeat(implicit p: Parameters) extends BoomBundle
{
  val tag    = UInt(ciiTagBits.W)
  val status = new CiiWbStatus
}

class VecCiiWritebackIO(implicit p: Parameters) extends BoomBundle
{
  val wb          = Flipped(Valid(new CiiWriteback))
  val wb_lookup   = Flipped(new VecCiiWbLookupIO)
  val wb_suppress = Input(Bool())

  val vrf_write = Output(Valid(new VecCiiVrfWrite))
  val int_wb    = Output(Valid(new ExeUnitResp(xLen)))
  val fp_wb     = Output(Valid(new ExeUnitResp(xLen)))

  val wb_credit = Output(Bool())

  val beat     = Output(Valid(new VecCiiWbBeat))
  val wb_beat  = Output(Valid(UInt(ciiTagBits.W)))
  val wb_last  = Output(Bool())
}

class VecCiiWriteback(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecCiiWriteback: elaborated only under usingRVV (never rocket's usingVector)")
  require(VecCiiDstKind.NUM_DST_WB == 1,
    s"VecCiiWriteback: NUM_DST_WB (${VecCiiDstKind.NUM_DST_WB}) must be 1 -- W2 is never contended and never arbitrated")

  val io = IO(new VecCiiWritebackIO)

  //@req-spec-cii.e16
  // Unlike Src-Data, the host is the RECEIVER here, so draining costs one
  // credit and no manufactured beat. Withholding the credit would stall the
  // channel for every SURVIVING instruction behind the killed one and turn a
  // squash into a permanent hang.
  io.wb_credit := io.wb.valid

  //@req-spec-cii.g19
  //@req-spec-cii.g21
  //@req-spec-cii.g22
  val status = io.wb.bits.wb_status

  //@req-spec-cii.g8
  //@req-spec-cii.g20
  val is_vec = status.dst_kind === VecCiiDstKind.VEC
  val is_int = status.dst_kind === VecCiiDstKind.INT
  val is_fp  = status.dst_kind === VecCiiDstKind.FP

  //@req-spec-cii.g17
  val wb_dst_offset = io.wb.bits.wb_dst_offset

  io.wb_lookup.req.tag           := io.wb.bits.tag
  io.wb_lookup.req.wb_dst_offset := wb_dst_offset

  //@req-spec-cii.g18
  val wb_wr_en = io.wb.bits.wb_wr_en

  //@req-spec-cii.e17
  val not_suppressed = !io.wb_suppress

  //@req-spec-cii.g1
  //@req-spec-cii.i1
  //@req-spec-cii.i4
  //@req-spec-cii.i8
  //@req-spec-cii.i11
  val vec_en = io.wb.valid && wb_wr_en && is_vec && io.wb_lookup.resp.wr_en && not_suppressed

  io.vrf_write.valid      := vec_en
  io.vrf_write.bits.addr  := io.wb_lookup.resp.prn
  //@req-spec-cii.g3
  //@req-spec-cii.g4
  //@req-spec-cii.i10
  io.vrf_write.bits.data  := io.wb.bits.wb_data
  io.vrf_write.bits.mask  := ~(0.U((vecVLen / 8).W))

  //@req-spec-cii.g6
  //@req-spec-cii.g7
  //@req-spec-cii.g16
  //@req-spec-cii.f42
  val int_en = io.wb.valid && wb_wr_en && is_int && not_suppressed
  val fp_en  = io.wb.valid && wb_wr_en && is_fp  && not_suppressed

  // Start the carried uop from NullMicroOp, NOT DontCare. This beat is a real
  // ExeUnitResp and its consumers read uop fields this module has no opinion
  // about, so leaving them invalid propagates X into live control:
  //   - core.scala feeds uop.br_mask to IsKilledByBranch on the rob wb_resp.
  //     Zero is not merely safe here but correct -- VecCiiFlush asserts
  //     alloc_br_mask === 0, since IQ_V_ALU only grants past the PNR.
  //   - rob.scala gates `rob_vconfig := uop.vconfig` on uop.is_vl_producer; an
  //     X there latches a garbage vtype into the committing ROB row. A CII
  //     arith op is never a vl producer.
  // Fields the consumers actually key on are driven explicitly below.
  io.int_wb.valid       := int_en
  io.int_wb.bits         := DontCare
  io.int_wb.bits.uop     := NullMicroOp
  io.int_wb.bits.data    := io.wb.bits.wb_data(xLen - 1, 0)
  io.int_wb.bits.predicated := false.B
  io.int_wb.bits.fflags.valid := false.B
  io.int_wb.bits.fflags.bits  := 0.U
  io.int_wb.bits.uop.pdst    := io.wb_lookup.resp.pdst
  io.int_wb.bits.uop.rob_idx := io.wb_lookup.resp.rob_idx
  io.int_wb.bits.uop.dst_rtype := RT_FIX

  io.fp_wb.valid       := fp_en
  io.fp_wb.bits         := DontCare
  io.fp_wb.bits.uop     := NullMicroOp
  io.fp_wb.bits.data    := io.wb.bits.wb_data(xLen - 1, 0)
  io.fp_wb.bits.predicated := false.B
  io.fp_wb.bits.fflags.valid := fp_en
  io.fp_wb.bits.fflags.bits  := status.fflags
  io.fp_wb.bits.uop.pdst    := io.wb_lookup.resp.pdst
  io.fp_wb.bits.uop.rob_idx := io.wb_lookup.resp.rob_idx
  io.fp_wb.bits.uop.dst_rtype := RT_FLT
  // fp-pipeline recodes this beat as `v_eew =/= 2`, i.e. single vs double, and
  // says so at its write port. A vfmv.f.s result is SEW wide, so the tag table
  // carries the issuing op's SEW for exactly this. Getting it wrong silently
  // recodes an e32 result as double.
  io.fp_wb.bits.uop.v_eew.get := io.wb_lookup.resp.v_eew

  io.beat.valid        := io.wb.valid
  io.beat.bits.tag     := io.wb.bits.tag
  io.beat.bits.status  := status

  io.wb_beat.valid := io.wb.valid
  io.wb_beat.bits  := io.wb.bits.tag
  io.wb_last       := status.last

  assert(PopCount(Seq(vec_en, int_en, fp_en)) <= 1.U,
    "VecCiiWriteback: at most one destination enable may be asserted per cycle")
  assert(!io.wb.valid || is_vec || is_int || is_fp,
    "VecCiiWriteback: dst_kind must never be the reserved encoding")

  assert(!vec_en || io.wb_lookup.resp.wr_en,
    "VecCiiWriteback: an enabled vector write must have wb_lookup.resp.wr_en set")

  //@req-spec-cii.e17
  assert(!io.wb_lookup.resp.killed || io.wb_suppress,
    "VecCiiWriteback: a killed tag's beat must always be suppressed (killed -> wb_suppress)")

  assert(!(io.wb.valid && (is_int || is_fp)) || (status.last && wb_dst_offset === 0.U),
    "VecCiiWriteback: a scalar-destination beat must carry last with wb_dst_offset === 0")

  when (io.wb.valid) {
    VecTrace.traceId("VecCiiWriteback", "wb", io.wb_lookup.resp.rob_idx, Seq(
      ("tag",       io.wb.bits.tag),
      ("dst_kind",  status.dst_kind),
      ("off",       wb_dst_offset),
      ("prn",       io.wb_lookup.resp.prn),
      ("pdst",      io.wb_lookup.resp.pdst),
      ("data",      io.wb.bits.wb_data(63, 0)),
      ("last",      status.last),
      ("wr_en",     wb_wr_en),
      ("suppressed", io.wb_suppress)))
  }
}

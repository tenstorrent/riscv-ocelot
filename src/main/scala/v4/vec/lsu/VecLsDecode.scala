//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Load/Store Decode (address-generation decode)
//------------------------------------------------------------------------------
//
// Combinational decoder that turns one vector memory uop (plus its resolved
// rs1/rs2 operands, vl, and vstart) into a `ConfigInfo` for the Step-10 load/
// store generation path. Reuses the Step-2 static decode (VLSDecode.apply) for
// the instruction-encoded fields (EEW, mop, lumop, NF, vm, unit/strided/whole/
// mask/fof) and folds in the dynamic, vtype/operand-dependent fields (VL, EMUL,
// stride magnitude/direction, segment count, base address).
//
// Ported from bobtail's `OviLsDecode` module (origin/bobtail/main
// src/main/scala/exu/ovi_wrapper/ls_decode.scala:67-237). Caracal differences:
//   - All OVI-specific plumbing is dropped: `sb_id` / the SBIDQ port,
//     `EnhancedFuncUnitReq`, the Decoupled ready/valid OVI handshake, the
//     OviScoreboard, and the `dontTouch` debug taps. Step-10 should remap any
//     `config_info.sb_id` reads to `dec_info.uop.rob_idx`.
//   - Instruction-field extraction is delegated to `VLSDecode.apply(uop.inst)`
//     instead of being re-sliced inline.
//   - Operands arrive as plain Inputs (rs1_data/rs2_data/vl/vstart) rather than
//     through `io.in.req` (EnhancedFuncUnitReq); vtype comes from
//     `uop.vconfig.vtype`.
//   - Not instantiated in core.scala yet (deferred to Step 10), so it is
//     scalac-compiled but not elaborated. The trace printf below will not fire
//     until then.

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.vec.decode.VLSDecode

class VecLsDecode(implicit p: Parameters) extends BoomModule with VecLsConstants
{
  val io = IO(new Bundle {
    val in = Input(new Bundle {
      val valid    = Bool()
      val uop      = new MicroOp()
      val rs1_data = UInt(xLen.W)   // base address
      val rs2_data = UInt(xLen.W)   // stride (for strided ops)
      val vl       = UInt(vecVLSz.W)
      val vstart   = UInt(VSTART_W.W)
    })
    val out = Output(new Bundle {
      val valid    = Bool()
      val is_load  = Bool()         // control signal for loadgen vs storegen
      val dec_info = new ConfigInfo()
    })
  })

  // ========= Static (instruction-encoded) decode (reuse Step-2) =========
  // bobtail ls_decode.scala:87-117 re-sliced the instruction inline; here we
  // reuse the Step-2 static decoder for the same fields.
  val vlsinfo = VLSDecode(io.in.uop.inst)

  // ========= Dynamic config / register values =========
  // vtype snapshot off the uop's vconfig. Caracal's VConfig bundle is FLAT
  // (vsew/vlmul/vta/vma/vlmax directly), unlike rocketchip's VConfig which nests
  // a `.vtype` -- so read the fields straight off vconfig.
  val vtype     = io.in.uop.vconfig
  val rs1_data  = io.in.rs1_data
  val rs2_data  = io.in.rs2_data

  // Caracal's vtype.vlmul IS the signed 3-bit LMUL exponent in two's-complement
  // (000..011 = +0..+3 for m1/m2/m4/m8; 111/110/101 = -1/-2/-3 for mf2/mf4/mf8),
  // which is exactly bobtail's reconstructed Mux(vlmul_sign, -vlmul_mag, vlmul_mag).
  // So the raw field is the signed exponent directly.
  val vtype_vlmul = vtype.vlmul
  val vtype_vsew  = vtype.vsew

  // Static field aliases (bobtail used inst slices; VLSInfo carries them).
  val instWidth = vlsinfo.v_eew(1, 0)  // EEW field, low two bits (bobtail instWidth = inst[13:12])
  val instNf    = vlsinfo.nf           // bobtail instNf = inst[31:29]

  val is_load   = vlsinfo.is_load
  val is_whole  = vlsinfo.is_whole
  val is_mask_ls = vlsinfo.is_mask_ls
  val is_indexed = vlsinfo.is_indexed
  val is_unit   = vlsinfo.is_unit
  val is_strided = vlsinfo.is_strided
  val is_seg    = vlsinfo.is_seg
  val is_fof    = vlsinfo.is_fof

  // seg_count = nf+1 for segmented ops, else 1 (bobtail ls_decode.scala:131).
  val seg_count = Mux(is_seg, instNf +& 1.U, 1.U(SEG_W.W))

  // ========= Good-stride detector (bobtail ls_decode.scala:135-139) =========
  // strideN: rs2 byte stride equals N * (seg_count << EEW), magnitude in either
  // direction. unit/whole/mask force the "1-element" case.
  val strideIs1 = (is_strided && ((rs2_data === (seg_count << (instWidth + 0.U))) ||
                                   (rs2_data.asSInt === -(seg_count << (instWidth + 0.U)).asSInt))) ||
                  is_unit || is_whole || is_mask_ls
  val strideIs2 = is_strided && ((rs2_data === (seg_count << (instWidth + 1.U))) ||
                                 (rs2_data.asSInt === -(seg_count << (instWidth + 1.U)).asSInt))
  val strideIs4 = is_strided && ((rs2_data === (seg_count << (instWidth + 2.U))) ||
                                 (rs2_data.asSInt === -(seg_count << (instWidth + 2.U)).asSInt))
  val strideIsNeg = rs2_data(xLen - 1)  // bobtail used rs2_data(63)

  // ========= Whole load/store evl decode (bobtail ls_decode.scala:124-141) ==
  // evl for whole-register ops = NFIELDS * VLEN / EEW, expressed in elements.
  // The lookup key is Cat(nf, width[2:0]) where width is the 3-bit funct3 EEW
  // field (inst[14:12]: 000=e8,101=e16,110=e32,111=e64). The middle key bit is
  // the width HIGH bit inst[14] (1 for e16/e32/e64), NOT the mem-extended-width
  // bit inst[28] -- using mew here left whole_vl=0 for e16/e32/e64 (-> vl=0 ->
  // bypass -> no load).
  val nf_wth = Cat(instNf, io.in.uop.inst(14), instWidth)  // {nf[2:0], width[2:0]}
  val whole_vl = MuxLookup(nf_wth, 0.U)(Seq(
    0.U  -> (VLEN_BYTES.U),       // nf=0, width=00 (8b):  1*VLEN/8
    5.U  -> (VLEN_BYTES.U >> 1),  // nf=0, width=01 (16b): 1*VLEN/16
    6.U  -> (VLEN_BYTES.U >> 2),  // nf=0, width=10 (32b): 1*VLEN/32
    7.U  -> (VLEN_BYTES.U >> 3),  // nf=0, width=11 (64b): 1*VLEN/64
    8.U  -> (VLEN_BYTES.U << 1),  // nf=1, width=00 (8b):  2*VLEN/8
    13.U -> (VLEN_BYTES.U),       // nf=1, width=01 (16b): 2*VLEN/16
    14.U -> (VLEN_BYTES.U >> 1),  // nf=1, width=10 (32b): 2*VLEN/32
    15.U -> (VLEN_BYTES.U >> 2),  // nf=1, width=11 (64b): 2*VLEN/64
    24.U -> (VLEN_BYTES.U << 2),  // nf=3, width=00 (8b):  4*VLEN/8
    29.U -> (VLEN_BYTES.U << 1),  // nf=3, width=01 (16b): 4*VLEN/16
    30.U -> (VLEN_BYTES.U),       // nf=3, width=10 (32b): 4*VLEN/32
    31.U -> (VLEN_BYTES.U >> 1),  // nf=3, width=11 (64b): 4*VLEN/64
    56.U -> (VLEN_BYTES.U << 3),  // nf=7, width=00 (8b):  8*VLEN/8
    61.U -> (VLEN_BYTES.U << 2),  // nf=7, width=01 (16b): 8*VLEN/16
    62.U -> (VLEN_BYTES.U << 1),  // nf=7, width=10 (32b): 8*VLEN/32
    63.U -> (VLEN_BYTES.U)        // nf=7, width=11 (64b): 8*VLEN/64
  ))

  // whole-register EMUL = log2(NFIELDS) (bobtail ls_decode.scala:143-148).
  val whole_vlmul = MuxLookup(instNf, 0.U)(Seq(
    0.U -> 0.U,  // 1 register  (log2(1) = 0)
    1.U -> 1.U,  // 2 registers (log2(2) = 1)
    3.U -> 2.U,  // 4 registers (log2(4) = 2)
    7.U -> 3.U   // 8 registers (log2(8) = 3)
  ))

  // ========= Outputs =========
  val dec_info = io.out.dec_info

  io.out.valid   := io.in.valid
  io.out.is_load := is_load
  dec_info.uop   := io.in.uop

  // base_v_reg: vd for loads / vs3 for stores (bobtail used inst[11:7];
  // MicroOp.lvd is the same architectural dest, width vecLregSz==5).
  dec_info.base_v_reg := io.in.uop.lvd

  // base_addr from rs1 (bobtail ls_decode.scala:233).
  dec_info.base_addr := rs1_data
  dec_info.vstart    := io.in.vstart

  // evl: whole -> whole_vl, mask-type -> ceil(vl/8), else vl
  // (bobtail ls_decode.scala:155-159).
  dec_info.vl := MuxCase(io.in.vl, Seq(
    is_whole   -> whole_vl,
    is_mask_ls -> ((io.in.vl + 7.U) >> 3)
  ))

  // eew_enc: mask-type fixed 8b (enc 0), indexed uses vtype.vsew, else inst
  // width (bobtail ls_decode.scala:162-166, frozen contract form).
  dec_info.eew_enc := Mux(is_mask_ls, 0.U,
                          Mux(is_indexed, vtype_vsew(1, 0), vlsinfo.v_eew(1, 0)))

  // emul_enc: recomputed (NOT reusing uop.v_emul). Rationale: the Step-2
  // VLSDecode header notes the static decode deliberately does not compute
  // EMUL, and recomputing here keeps VecLsDecode a self-contained 1:1 port of
  // bobtail ls_decode.scala:195-204 (whole -> NF-derived; indexed -> clamped
  // vtype LMUL; mask -> 0; else clamped (LMUL + EEW_width - SEW)).
  val emul_normal     = vtype_vlmul + instWidth - vtype_vsew  // log-domain LMUL*(EEW/SEW)
  val eff_emul_normal = Mux(emul_normal(2), 0.U, emul_normal(1, 0))  // positive-clamp
  val eff_vtype_vlmul = Mux(vtype_vlmul(2), 0.U, vtype_vlmul(1, 0))  // positive-clamp
  dec_info.emul_enc := MuxCase(eff_emul_normal, Seq(
    is_whole   -> whole_vlmul,
    is_indexed -> eff_vtype_vlmul,
    is_mask_ls -> 0.U
  ))

  // ========= Stride block (bobtail ls_decode.scala:206-231) =========
  dec_info.is_good_stride := strideIs1 || strideIs2 || strideIs4
  // force positive direction for whole/unit; else use sign of rs2.
  dec_info.stride_dir := Mux(is_whole || is_unit, false.B, strideIsNeg)
  // force the implied stride for unit (seg*EEW) and whole (1*EEW); else rs2.
  dec_info.stride := MuxCase(rs2_data.asSInt, Seq(
    is_unit  -> (seg_count << instWidth).zext,
    is_whole -> (1.U << instWidth).zext
  ))
  dec_info.stride_is_1 := strideIs1
  // stride_enc: log2 of the good-stride multiple {1,2,4}.
  dec_info.stride_enc := MuxCase(0.U, Seq(
    strideIs1 -> 0.U,
    strideIs2 -> 1.U,
    strideIs4 -> 2.U
  ))

  // ========= Segment outputs (bobtail ls_decode.scala:234-240) =========
  // is_good_seg: not segmented (seg==1) or seg_count is a power of two.
  dec_info.is_good_seg := !is_seg || (((instNf + 1.U) & instNf) === 0.U)
  dec_info.seg_count   := seg_count
  dec_info.seg_enc     := PriorityEncoder(seg_count)

  // ========= Operation-type flags (bobtail ls_decode.scala:243-250) =========
  // is_mask: masked LS (uses the mask buffer), suppressed for whole / mask-type
  // LS. v_unmasked == vm bit; masked iff !v_unmasked.
  dec_info.is_mask  := Mux(is_whole || is_mask_ls, false.B, !io.in.uop.v_unmasked)
  dec_info.is_index := is_indexed
  dec_info.is_fof   := is_fof

  // ========= Trace =========
  // Will not fire until Step 10 wires io.in.valid; gated off for now.
  val vecTrace = false.B
  when (vecTrace && io.in.valid) {
    printf("[vlsdec] rob=%d mop=%d eew_enc=%d emul_enc=%d stride=%d seg_count=%d is_mask=%d is_index=%d\n",
      io.in.uop.rob_idx, vlsinfo.mop, dec_info.eew_enc, dec_info.emul_enc,
      dec_info.stride, dec_info.seg_count, dec_info.is_mask, dec_info.is_index)
  }
}

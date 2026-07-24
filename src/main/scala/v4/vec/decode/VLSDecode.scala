//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Load/Store Decode (decode-time static field extraction)
//------------------------------------------------------------------------------
//
// Extracts the *static* (instruction-encoded) load/store fields from a vector
// memory instruction: element width (EEW), mop, lumop, NF (segments), the vm
// (mask-enable) bit, and the unit-stride sub-class (whole-register, mask, fault-
// only-first). These are all known at decode without vtype.
//
// NOTE: EMUL (= LMUL * EEW/SEW) and the per-uop vconfig snapshot depend on the
// *current vtype*, which is tracked by the VConfigUnit (VCFG) vtype mirror, not
// known at decode. So this object does NOT compute v_emul -- the VCFG / core-level
// EMUL derivation fills it. The full address-generation decode (ports, strides)
// is the Step 8 VecLsDecode port; this is only the decode-stage classification.

package boom.v4.vec.decode

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

/** RVV memory mop encodings (inst[27:26]). */
object VMop
{
  val UNIT       = 0.U(2.W)  // unit-stride (also whole-reg / mask / fault-only-first via lumop)
  val UNORDERED  = 1.U(2.W)  // indexed-unordered
  val STRIDED    = 2.U(2.W)  // strided
  val ORDERED    = 3.U(2.W)  // indexed-ordered
}

/** RVV unit-stride lumop encodings (inst[24:20], only meaningful when mop==UNIT). */
object VLumop
{
  val UNIT  = 0.U(5.W)   // plain unit-stride
  val WHOLE = 8.U(5.W)   // whole-register load/store (vl?re / vs?r)
  val MASK  = 11.U(5.W)  // mask load/store (vlm.v / vsm.v)
  val FOF   = 16.U(5.W)  // fault-only-first (vle?ff.v)
}

/** Decoded static load/store info for one vector memory instruction. */
class VLSInfo(implicit p: Parameters) extends BoomBundle
{
  val is_load    = Bool()
  val is_store   = Bool()
  val v_eew      = UInt(3.W)   // encoded element width: 0=8b,1=16b,2=32b,3=64b
  val mop        = UInt(2.W)
  val lumop      = UInt(5.W)
  val nf         = UInt(3.W)   // NF field (0 => 1 field); seg count = nf+1
  val mew        = Bool()      // memory extended width bit (inst[28]); mew=1 => EEW=128 (reserved in M1)
  val eew_unsup  = Bool()      // mew=1 selects EEW=128, unsupported in Caracal M1
  val vm         = Bool()      // 1 = unmasked, 0 = masked (read v0)
  val is_indexed = Bool()
  val is_strided = Bool()
  val is_unit    = Bool()
  val is_whole   = Bool()
  val is_mask_ls = Bool()
  val is_fof     = Bool()
  val is_seg     = Bool()
}

object VLSDecode
{
  // RVV major opcodes.
  def OP_LOAD_FP  = "b0000111".U(7.W)
  def OP_STORE_FP = "b0100111".U(7.W)

  /** Vector-memory funct3 (width) encodings -- distinguish vector mem ops from
    * scalar FP loads/stores that share the LOAD-FP/STORE-FP opcodes. */
  def isVecMemFunct3(f3: UInt): Bool = f3 === "b000".U || f3 === "b101".U ||
                                       f3 === "b110".U || f3 === "b111".U

  /** Is this a vector load or store (vs a scalar FLW/FLD/FSW/FSD)? */
  def isVecMem(inst: UInt): Bool = {
    val opcode = inst(6, 0)
    val f3     = inst(14, 12)
    ((opcode === OP_LOAD_FP) || (opcode === OP_STORE_FP)) && isVecMemFunct3(f3)
  }

  def apply(inst: UInt)(implicit p: Parameters): VLSInfo = {
    val info   = Wire(new VLSInfo)
    val opcode = inst(6, 0)
    val f3     = inst(14, 12)

    info.is_load  := opcode === OP_LOAD_FP
    info.is_store := opcode === OP_STORE_FP
    // For vector memory ops the encoded EEW = {mew, width[1:0]}, where width is
    // funct3 inst[14:12] and mew is inst[28] (NOT inst[14] -- that earlier comment
    // was wrong). Caracal M1 supports only SEW 8/16/32/64, i.e. mew=0; mew=1 selects
    // EEW=128, which is reserved/unsupported here. We capture the low two width bits
    // as v_eew (0=8b,1=16b,2=32b,3=64b) and flag mew=1 as unsupported rather than
    // silently treating it as a 64b op. The illegal-instruction path is owned by
    // decode.scala (not this object); eew_unsup surfaces the condition for it / the
    // VCFG to act on.
    val mew = inst(28)
    info.mew       := mew
    info.eew_unsup := mew
    info.v_eew    := Cat(0.U(1.W), f3(1, 0))
    info.mop      := inst(27, 26)
    info.lumop    := inst(24, 20)
    info.nf       := inst(31, 29)
    info.vm       := inst(25)

    val is_unit = info.mop === VMop.UNIT
    info.is_indexed := info.mop === VMop.UNORDERED || info.mop === VMop.ORDERED
    info.is_strided := info.mop === VMop.STRIDED
    info.is_unit    := is_unit
    info.is_whole   := is_unit && info.lumop === VLumop.WHOLE
    info.is_mask_ls := is_unit && info.lumop === VLumop.MASK
    info.is_fof     := is_unit && info.lumop === VLumop.FOF
    // Segmented only for the regular (non whole/mask/fof) unit/strided/indexed forms.
    info.is_seg     := (info.nf =/= 0.U) && !info.is_whole && !info.is_mask_ls
    info
  }
}

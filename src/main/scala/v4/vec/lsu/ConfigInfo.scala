//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Load/Store Config Info
//------------------------------------------------------------------------------
//
// `ConfigInfo` is the latched, fully-resolved description of one vector memory
// instruction handed from `VecLsDecode` to the Step-10 load/store generators
// (loadgen/storegen/walkers/packers/skippers under reference/vec-lsgen/). It
// combines the static decode (EEW, mop, NF, vm, unit/strided/whole/mask/fof)
// with the dynamic, vtype/operand-dependent fields (VL, EMUL, base address,
// stride magnitude/direction, segment count).
//
// Ported from bobtail's `OviLsDecode` ConfigInfo (origin/bobtail/main
// src/main/scala/exu/ovi_wrapper/ls_decode.scala:43-65). Differences vs bobtail:
//   - The `sb_id` (OVI scoreboard id) field is DROPPED. Caracal has no OVI
//     scoreboard; Step-10 consumers that read `sb_id` should remap it to
//     `uop.rob_idx` off the embedded `uop` field instead.
//   - `VecLsConstants` extends `HasBoomCoreParameters`, so VLEN/DMEM_WIDTH come
//     from the core `Parameters` rather than being threaded as ctor args. The
//     bobtail/reference `new ConfigInfo(VLEN, DMEM_WIDTH)` becomes a plain
//     `new ConfigInfo` (Step-10 must drop the ctor args when instantiating).

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

/** Shared sizing constants for the Caracal vector load/store generation path.
  *
  * Mixed into both `ConfigInfo` (the bundle) and `VecLsDecode` (the decoder),
  * and intended to be mixed into the Step-10 lsgen modules so they all agree on
  * widths. VLEN and DMEM_WIDTH are derived from the core `Parameters` via
  * `HasBoomCoreParameters` (no constructor args, unlike bobtail's abstract
  * `val VLEN`/`val DMEM_WIDTH`).
  *
  * Ported from bobtail trait `VecLSGenConstants` (ls_decode.scala:18-39).
  */
trait VecLsConstants extends HasBoomCoreParameters
{
  def VLEN             = vecVLen                  // VLEN in bits (default 256)
  // coreDataBits = xLen max fLen max vMemDataBits. BOOM does not set
  // vMemDataBits, and xLen == fLen == 64 here, so this resolves to 64 -- the
  // dcache data-beat width the lsgen path packs against.
  def DMEM_WIDTH       = coreDataBits             // == 64
  def VLEN_BYTES       = VLEN / 8
  def DMEM_BYTES       = DMEM_WIDTH / 8
  def DMEM_ENC         = log2Ceil(DMEM_WIDTH / 8 + 1)  // +1 so the wrap value fits the comparator
  def ADDR_BREAK       = log2Ceil(DMEM_WIDTH / 8)
  def VDB_R_SIZE_BYTES = log2Ceil(DMEM_WIDTH / 8 + 1)
  def EEW_ENC_W        = 2
  def EMUL_ENC_W       = 2
  def STRIDE_ENC_W     = 2
  def SEG_W            = 4
  def SEG_ENC_W        = 3
  def MASK_W           = 64
  def MASK_W_SIZE      = log2Ceil(MASK_W)
  def EL_ID_W          = log2Ceil(VLEN / 8)
  def VSTART_W         = log2Ceil(VLEN)
}

/** Decoded + resolved config for one vector memory instruction.
  *
  * Field-for-field port of bobtail's ConfigInfo (ls_decode.scala:43-65) with
  * `sb_id` removed (see file header). The widths track `VecLsConstants` /
  * the core params so the Step-10 lsgen consumers see identical layouts.
  */
class ConfigInfo(implicit p: Parameters) extends BoomBundle with VecLsConstants
{
  val base_v_reg     = UInt(5.W)              // vd (loads) / vs3 (stores)
  val vl             = UInt(vecVLSz.W)        // effective vl (evl), already adjusted for whole/mask
  val vstart         = UInt(VSTART_W.W)
  val eew_enc        = UInt(EEW_ENC_W.W)      // encoded EEW: 0=8b,1=16b,2=32b,3=64b
  val emul_enc       = UInt(EMUL_ENC_W.W)     // positive-clamped EMUL (log domain)
  val stride         = SInt(xLen.W)           // byte stride (signed)
  val stride_enc     = UInt(STRIDE_ENC_W.W)   // log2 of good-stride multiple {1,2,4}*EEW
  val stride_dir     = Bool()                 // 0: positive, 1: negative
  val is_good_stride = Bool()                 // stride is 1/2/4 * (seg*EEW)
  val stride_is_1    = Bool()                 // stride is exactly 1 element (for storegen)
  val seg_count      = UInt(SEG_W.W)          // number of fields per segment (nf+1)
  val seg_enc        = UInt(SEG_ENC_W.W)      // log2 of seg_count
  val is_good_seg    = Bool()                 // seg_count == 1 or power-of-two
  val is_mask        = Bool()                 // masked LS (uses mask buffer); NOT mask-type LS
  val is_index       = Bool()                 // indexed (ordered/unordered)
  val is_fof         = Bool()                 // fault-only-first unit load
  val base_addr      = UInt(xLen.W)           // rs1 base address
  val uop            = new MicroOp()          // embedded uop (Step-10 reads rob_idx in place of sb_id)
}

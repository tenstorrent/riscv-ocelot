// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

import boom.exu.FUConstants._
import boom.common._
import boom.exu._
import boom.util._

import chisel3.dontTouch // this is for debugging purposes

trait VecLSGenConstants {
  // --- abstract vals ---
  val VLEN: Int
  val DMEM_WIDTH: Int
  // --- constants ---
  val VLEN_BYTES = VLEN/8
  val DMEM_BYTES = DMEM_WIDTH/8
  val DMEM_ENC   = log2Ceil(DMEM_WIDTH/8+1) // +1 to use the wrap value in comparator
  val ADDR_BREAK = log2Ceil(DMEM_WIDTH/8)
  val VDB_R_SIZE_BYTES = log2Ceil(DMEM_WIDTH/8+1)
  val EEW_ENC_W    = 2
  val EMUL_ENC_W   = 2
  val STRIDE_ENC_W = 2
  val SEG_W        = 4
  val SEG_ENC_W    = 3
  val MASK_W       = 64
  val MASK_W_SIZE  = log2Ceil(MASK_W)
  val EL_ID_W      = log2Ceil(VLEN/8)   // should be 11 in OVI (add padding)
  val VSTART_W     = log2Ceil(8*VLEN/8) // should be 13 in OVI (add padding)
}

class ConfigInfo(override val VLEN: Int, override val DMEM_WIDTH: Int)(implicit p: Parameters)
extends Bundle with VecLSGenConstants {
  val sb_id      = UInt(5.W)
  val base_v_reg = UInt(5.W)
  val vl         = UInt(9.W)
  val vstart     = UInt(VSTART_W.W)
  val eew_enc    = UInt(EEW_ENC_W.W)
  val emul_enc   = UInt(EMUL_ENC_W.W)
  val stride     = SInt(64.W)
  val stride_enc = UInt(STRIDE_ENC_W.W)
  val stride_dir = Bool()
  val is_good_stride = Bool()
  val stride_is_1 = Bool() // stride is 1 (for storegen)
  val seg_count  = UInt(SEG_W.W)
  val seg_enc    = UInt(SEG_ENC_W.W)
  val is_good_seg = Bool()
  val is_mask    = Bool()
  val is_index   = Bool()
  val is_fof     = Bool()
  val base_addr  = UInt(64.W)
  val uop        = new MicroOp()
}

class OviLsDecode(override val VLEN: Int, override val DMEM_WIDTH: Int)(implicit p: Parameters)
extends BoomModule with VecLSGenConstants {
  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // Input from the VLSIQ and the SBIDQ
    val in = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val req   = Input(new EnhancedFuncUnitReq(xLen, VLEN))
    }
    // Outputs
    val out = new Bundle {
      val ready    = Input(Bool())
      val valid    = Output(Bool())
      val is_load  = Output(Bool()) // control signal for loadgen vs storegen
      val dec_info = Output(new ConfigInfo(VLEN, DMEM_WIDTH))
    }
  })

  // ======== RV Handshake ========

  // not clocked so simply pass through
  io.in.ready  := io.out.ready
  io.out.valid := io.in.valid

  // ========= Config and Register Values ========

  val rs1_data     = io.in.req.req.rs1_data
  val rs2_data     = io.in.req.req.rs2_data
  val vtype_vl     = io.in.req.vconfig.vl
  val vtype_vstart = io.in.req.vstart
  val vtype_vlmul  = Mux(io.in.req.vconfig.vtype.vlmul_sign, // dealing with fractional LMUL
                         -io.in.req.vconfig.vtype.vlmul_mag(1,0).asTypeOf(UInt(3.W)),
                          io.in.req.vconfig.vtype.vlmul_mag(1,0).asTypeOf(UInt(3.W)))
  val vtype_vsew   = io.in.req.vconfig.vtype.vsew

  // ========= Instruction Fields ========

  val instOP   = io.in.req.req.uop.inst(6, 0)
  val instUMop = io.in.req.req.uop.inst(24, 20)
  val instMop  = io.in.req.req.uop.inst(27, 26)

  val instNf         = io.in.req.req.uop.inst(31, 29)
  val instMaskEnable = !io.in.req.req.uop.inst(25) // 0: enable, 1 disable
  val instElemSize   = io.in.req.req.uop.inst(14, 12)
  val instWidth      = instElemSize(1, 0) // EEW field
  val instMew        = instElemSize(2)    // reserved: must be 0
  val instVldDest    = io.in.req.req.uop.inst(11, 7)

  // ========= Operation type flags ========

  val isLoad   = instOP === 7.U
  val isStore  = instOP === 39.U
  val isWhole  = instMop === 0.U && instUMop === 8.U
  val isMaskLS = instMop === 0.U && instUMop === 11.U // refers to vlm.v and vsm.v (not masked loads/stores)
  val isIndex  = instMop === 1.U ||  instMop === 3.U  // unordered and ordered
  val isUnit   = instMop === 0.U
  val isStride = instMop === 2.U
  val isSeg    = !isWhole && !isMaskLS && (instNf =/= 0.U)  // Segmented: not whole/mask and nf > 0
  val isFoF    = instMop === 0.U && instUMop === 16.U && instOP === 7.U // fault-only-first-unit load
  // val isWholeStore = isWhole && isStore
  // val isWholeLoad  = isWhole && isLoad
  // val isStoreMask  = isMaskLS && isStore
  // val isLoadMask   = isMaskLS && isLoad

  val seg_count = Mux(isSeg, instNf +& 1.U, 1.U(SEG_W.W))

  // ========= Good Stride Detector ========

  val strideIs0   = (isStride && (rs2_data === 0.U))
  val strideIs1   = (isStride && ((rs2_data === (seg_count << (instWidth + 0.U))) || (rs2_data.asSInt === (-(seg_count << (instWidth + 0.U)).asSInt)))) || (isUnit) || (isWhole) || (isMaskLS)
  val strideIs2   = (isStride && ((rs2_data === (seg_count << (instWidth + 1.U))) || (rs2_data.asSInt === (-(seg_count << (instWidth + 1.U)).asSInt))))
  val strideIs4   = (isStride && ((rs2_data === (seg_count << (instWidth + 2.U))) || (rs2_data.asSInt === (-(seg_count << (instWidth + 2.U)).asSInt))))
  val strideIsNeg = rs2_data(63)

  // ========= Whole Load/Store Decoder ========

  val nf_wth = Cat(instNf, instElemSize)
  val whole_vl = MuxLookup(nf_wth, 0.U, Seq(
    0.U  -> (VLEN_BYTES.U),       // nf=0, width=00 (8bit):  1*VLEN/8
    5.U  -> (VLEN_BYTES.U >> 1),  // nf=0, width=01 (16bit): 1*VLEN/16
    6.U  -> (VLEN_BYTES.U >> 2),  // nf=0, width=10 (32bit): 1*VLEN/32
    7.U  -> (VLEN_BYTES.U >> 3),  // nf=0, width=11 (64bit): 1*VLEN/64
    8.U  -> (VLEN_BYTES.U << 1),  // nf=1, width=00 (8bit):  2*VLEN/8
    13.U -> (VLEN_BYTES.U),       // nf=1, width=01 (16bit): 2*VLEN/16
    14.U -> (VLEN_BYTES.U >> 1),  // nf=1, width=10 (32bit): 2*VLEN/32
    15.U -> (VLEN_BYTES.U >> 2),  // nf=1, width=11 (64bit): 2*VLEN/64
    24.U -> (VLEN_BYTES.U << 2),  // nf=3, width=00 (8bit):  4*VLEN/8
    29.U -> (VLEN_BYTES.U << 1),  // nf=3, width=01 (16bit): 4*VLEN/16
    30.U -> (VLEN_BYTES.U),       // nf=3, width=10 (32bit): 4*VLEN/32
    31.U -> (VLEN_BYTES.U >> 1),  // nf=3, width=11 (64bit): 4*VLEN/64
    56.U -> (VLEN_BYTES.U << 3),  // nf=7, width=00 (8bit):  8*VLEN/8
    61.U -> (VLEN_BYTES.U << 2),  // nf=7, width=01 (16bit): 8*VLEN/16
    62.U -> (VLEN_BYTES.U << 1),  // nf=7, width=10 (32bit): 8*VLEN/32
    63.U -> (VLEN_BYTES.U)        // nf=7, width=11 (64bit): 8*VLEN/64
  ))

  val whole_vlmul = MuxLookup(instNf, 0.U, Seq(
    0.U -> 0.U,  // 1 register  (log2(1) = 0)
    1.U -> 1.U,  // 2 registers (log2(2) = 1)  
    3.U -> 2.U,  // 4 registers (log2(4) = 2)
    7.U -> 3.U   // 8 registers (log2(8) = 3)
  ))

  // ========= Outputs ========
  
  // is_load - detect load operations
  io.out.is_load := isLoad

  // sb_id - from the SBIDQ
  io.out.dec_info.sb_id := io.in.req.sb_id
  
  // base_v_reg (Base Vector Register) - vd for loads, vs3 for stores
  io.out.dec_info.base_v_reg := instVldDest  // For loads: vd[4:0], For stores: vs3[4:0] (same bit position)
  
  // vl/evl (Vector Length) - with special cases for both loads and stores
  io.out.dec_info.vl := MuxLookup(Cat(isWhole, isMaskLS), vtype_vl, Seq(
    Cat(true.B, false.B)  -> whole_vl,        // Whole register: NFIELDS * VLEN / EEW
    Cat(false.B, true.B)  -> ((vtype_vl + 7.U) >> 3),  // Mask: ceil(vl/8)
    Cat(false.B, false.B) -> vtype_vl        // Normal: vl CSR value
  ))

  io.out.dec_info.vstart := vtype_vstart
  
  // eew_enc (Encoded Effective Element Width)
  io.out.dec_info.eew_enc := MuxLookup(Cat(isMaskLS, isIndex), instWidth, Seq(
    Cat(true.B, false.B)  -> 0.U,             // Mask: fixed at 8 bits (encoded as 0)
    Cat(false.B, true.B)  -> vtype_vsew,      // Indexed: config sew value
    Cat(false.B, false.B) -> instWidth        // Others: data width from instruction
  ))
  
  // emul_enc (Effective LMUL) - calculated based on instruction type  
  val emul_normal = vtype_vlmul + instWidth - vtype_vsew  // EMUL = LMUL * (EEW / SEW) in log domain
  val eff_emul_normal = Mux(emul_normal(2), 0.U, emul_normal(1, 0)) // effective positive value
  val eff_vtype_vlmul = Mux(vtype_vlmul(2), 0.U, vtype_vlmul(1, 0)) // effective positive value
  io.out.dec_info.emul_enc := MuxLookup(Cat(isWhole, isIndex, isMaskLS), vtype_vlmul, Seq(
    Cat(true.B, false.B, false.B)  -> whole_vlmul,     // Whole:   NF derived
    Cat(false.B, true.B, false.B)  -> eff_vtype_vlmul, // Indexed: LMUL from vtype CSR
    Cat(false.B, false.B, true.B)  -> 0.U,             // Masked:  EMUL = 0 (fixed)
    Cat(false.B, false.B, false.B) -> eff_emul_normal  // Others:  EMUL = LMUL * (EEW / SEW)
  ))
  
  // Stride detection and outputs (for both loads and stores)
  io.out.dec_info.is_good_stride := strideIs1 || strideIs2 || strideIs4
  io.out.dec_info.stride_dir     := Mux((isWhole || isUnit), false.B, strideIsNeg)        // 0: positive, 1: negative (force positive for whole and unit)
  io.out.dec_info.stride         := MuxLookup(Cat(isUnit, isWhole), rs2_data.asSInt, Seq( // need to force the stride value in case of unit stride (rs2 data isnt the right value)
    Cat(true.B, false.B)  -> (seg_count << instWidth).zext.asSInt,    // for unit stride, force the stride to be the seg*eew
    Cat(false.B, true.B)  -> (1.U << instWidth).zext.asSInt, // (chisel tries to find the effective width then extend casuing in negative so force zero extend)
    Cat(false.B, false.B) -> rs2_data.asSInt                 // in other cases, when stride is required, use the rs2 field
  ))
  io.out.dec_info.stride_is_1 := strideIs1

  // stride_enc: log2 of stride magnitude for good strides
  io.out.dec_info.stride_enc := MuxLookup(Cat(strideIs4, strideIs2, strideIs1), 0.U, Seq(
    Cat(false.B, false.B, true.B) -> 0.U,   // stride = 1*EEW
    Cat(false.B, true.B, false.B) -> 1.U,   // stride = 2*EEW  
    Cat(true.B, false.B, false.B) -> 2.U    // stride = 4*EEW
  ))
  
  // Segment outputs (for both loads and stores)
  io.out.dec_info.is_good_seg := !isSeg || (((instNf + 1.U) & instNf) === 0.U) // either not segmented (seg=1) or segment (seg=nf+1) is power of 2
  io.out.dec_info.seg_count   := seg_count
  
  // seg_enc: log2 of segment count
  io.out.dec_info.seg_enc := PriorityEncoder(io.out.dec_info.seg_count)
  
  // Operation type flags (for both loads and stores)
  io.out.dec_info.is_mask  := Mux((isWhole || isMaskLS), false.B, instMaskEnable) // this is for masked LS (use the mask buffer) NOT mask-type LS
  io.out.dec_info.is_index := isIndex

  // fault-only-first (this variant is only for unit loads)
  io.out.dec_info.is_fof := isFoF
  
  // base_addr (Base Memory Address) - from rs1 (for both loads and stores)
  io.out.dec_info.base_addr := rs1_data

  // entire MicroOp
  io.out.dec_info.uop := io.in.req.req.uop

  // ========= Illegal Check ========

  // val is_illegal = mew_violation || eew_violation // TODO: add to outputs

  // ========= Vector Load/Store Illegal Instruction Conditions ========
  // 
  // Per RISC-V Vector specification, several conditions can cause illegal instruction
  // exceptions for vector load/store operations:
  //
  // 1. MEW (Memory Element Width) Violations:
  //    - inst[14] (mew bit) must be 0 for current spec
  //    - Values of 1 are reserved for future 128+ bit extensions
  //
  // 2. EMUL (Effective LMUL) Range Violations:
  //    - EMUL = (EEW/SEW) * LMUL must satisfy: 1/8 <= EMUL <= 8
  //    - Values outside this range use reserved encodings
  //
  // 3. Invalid Instruction Encoding:
  //    - Reserved mop field values (e.g., mop=001 for loads)
  //    - Other reserved field combinations per spec
  //
  // 4. Invalid Configuration (VPU-Detected):
  //    - vill bit set in vtype CSR (previous vset{i}vl{i} failed)
  //    - Unsupported EEW widths for current implementation
  //    - Vector instructions when vector extension disabled
  //
  // 5. Illegal Register Group Specifiers/Alignment:
  //    - LMUL=2: odd-numbered registers reserved (v1, v3, v5, ...)
  //    - LMUL=4: non-multiple-of-4 registers reserved (v1, v2, v3, v5, ...)
  //    - LMUL=8: non-multiple-of-8 registers reserved (v1-v7, v9-v15, ...)
  //
  // 6. Overlap Constraints Violations:
  //    - Load destination cannot overlap source unless specific conditions met
  //    - Masked loads: destination cannot overlap mask register v0
  //    - Indexed segment loads: destination cannot overlap index source vs2
  //
  // 7. Segment Load/Store Specific Constraints:
  //    - EMUL * NFIELDS must be <= 8
  //    - Register numbers cannot exceed v31 (no wraparound to v32+)
  //
  // 8. Whole Register Load Unsupported EEW:
  //    - vl<nf>r instructions with unsupported EEW values
  //
  // Note: The VPU may complete illegal operations by sending completed.illegal
  // without setting memop.sync_start. This triggers an exception during commit,
  // causing pipeline flush and killing younger instructions.

  dontTouch(isWhole)
  dontTouch(whole_vl)
  dontTouch(whole_vlmul)
  dontTouch(io.in)
  dontTouch(io.out)

}

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
  val EL_ID_W      = log2Ceil(VLEN/8) // should be 11 in OVI (add padding)
}

class ConfigInfo(override val VLEN: Int, override val DMEM_WIDTH: Int)
extends Bundle with VecLSGenConstants {
  val sb_id      = UInt(5.W)
  val base_v_reg = UInt(5.W)
  val vl         = UInt(9.W)
  val eew_enc    = UInt(EEW_ENC_W.W)
  val emul_enc   = UInt(EMUL_ENC_W.W)
  val stride     = SInt(64.W)
  val stride_enc = UInt(STRIDE_ENC_W.W)
  val stride_dir = Bool()
  val is_good_stride = Bool()
  val is_unit_stride = Bool() // stride is 1 (for storegen)
  val seg_count  = UInt(SEG_W.W)
  val seg_enc    = UInt(SEG_ENC_W.W)
  val is_good_seg = Bool()
  val is_mask    = Bool()
  val is_index   = Bool()
  val base_addr  = UInt(64.W)
}

class OviLsDecode(override val VLEN: Int, override val DMEM_WIDTH: Int)(implicit p: Parameters)
extends BoomModule with VecLSGenConstants {
  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // Input from the VLSIQ and the SBIDQ
    val deq_data  = Input(new EnhancedFuncUnitReq(xLen, VLEN))
    val deq_sb_id = Input(UInt(5.W))

    // Outputs
    val is_load  = Output(Bool()) // control for loadgen vs storegen
    val dec_info = new ConfigInfo(VLEN, DMEM_WIDTH)
  })

  // ========= Config and Register Values ========

  val rs1_data    = io.deq_data.req.rs1_data
  val rs2_data    = io.deq_data.req.rs2_data
  val vtype_vl    = io.deq_data.vconfig.vl
  val vtype_vlmul = io.deq_data.vconfig.vtype.vlmul_mag
  val vtype_vsew  = io.deq_data.vconfig.vtype.vsew

  // ========= Instruction Fields ========

  val instOP   = io.deq_data.req.uop.inst(6, 0)
  val instUMop = io.deq_data.req.uop.inst(24, 20)
  val instMop  = io.deq_data.req.uop.inst(27, 26)

  val instNf         = io.deq_data.req.uop.inst(31, 29)
  val instMaskEnable = !io.deq_data.req.uop.inst(25) // 0: enable, 1 disable
  val instElemSize   = io.deq_data.req.uop.inst(14, 12)
  val instWidth      = instElemSize(1, 0) // EEW field
  val instMew        = instElemSize(2)    // reserved: must be 0
  val instVldDest    = io.deq_data.req.uop.inst(11, 7)

  // ========= Operation type flags ========

  val isLoad   = instOP === 7.U
  val isStore  = instOP === 39.U
  val isWhole  = instMop === 0.U && instUMop === 8.U
  val isMaskLS = instMop === 0.U && instUMop === 11.U // refers to vlm.v and vsm.v (not masked loads/stores)
  val isIndex  = instMop === 1.U ||  instMop === 3.U  // unordered and ordered
  val isUnit   = instMop === 0.U
  val isStride = instMop === 2.U
  val isSeg    = !isWhole && !isMaskLS && (instNf =/= 0.U)  // Segmented: not whole/mask and nf > 0
  // val isWholeStore = isWhole && isStore
  // val isWholeLoad  = isWhole && isLoad
  // val isStoreMask  = isMaskLS && isStore
  // val isLoadMask   = isMaskLS && isLoad

  // ========= Good Stride Detector ========

  val strideIs0   = (isStride && (rs2_data === 0.U))
  val strideIs1   = (isStride && ((rs2_data === (1.U << (instWidth + 0.U))) || (rs2_data.asSInt === (-(1.U << (instWidth + 0.U)).asSInt)))) || (isUnit)
  val strideIs2   = (isStride && ((rs2_data === (1.U << (instWidth + 1.U))) || (rs2_data.asSInt === (-(1.U << (instWidth + 1.U)).asSInt))))
  val strideIs4   = (isStride && ((rs2_data === (1.U << (instWidth + 2.U))) || (rs2_data.asSInt === (-(1.U << (instWidth + 2.U)).asSInt))))
  val strideIsNeg = rs2_data(63)

  // ========= Whole Load/Store Decoder ========

  val nf_wth = Cat(instNf, instElemSize)
  val whole_vl = MuxLookup(nf_wth, 0.U, Seq(
    0.U  -> (DMEM_BYTES.U),
    5.U  -> (DMEM_BYTES.U >> 1),
    6.U  -> (DMEM_BYTES.U >> 2),
    7.U  -> (DMEM_BYTES.U >> 3),
    8.U  -> (DMEM_BYTES.U << 1),
    13.U -> (DMEM_BYTES.U),
    14.U -> (DMEM_BYTES.U >> 1),
    15.U -> (DMEM_BYTES.U >> 2),
    24.U -> (DMEM_BYTES.U << 2),
    29.U -> (DMEM_BYTES.U << 1),
    30.U -> (DMEM_BYTES.U),
    31.U -> (DMEM_BYTES.U >> 1),
    56.U -> (DMEM_BYTES.U << 3),
    61.U -> (DMEM_BYTES.U << 2),
    62.U -> (DMEM_BYTES.U << 1),
    63.U -> (DMEM_BYTES.U)
  ))

  val whole_vlmul = MuxLookup(instNf, 0.U, Seq(
    0.U -> 0.U,  // 1 register  (log2(1) = 0)
    1.U -> 1.U,  // 2 registers (log2(2) = 1)  
    3.U -> 2.U,  // 4 registers (log2(4) = 2)
    7.U -> 3.U   // 8 registers (log2(8) = 3)
  ))

  // ========= Outputs ========
  
  // is_load - detect load operations
  io.is_load := isLoad

  // sb_id - from the SBIDQ
  io.dec_info.sb_id := io.deq_sb_id
  
  // base_v_reg (Base Vector Register) - vd for loads, vs3 for stores
  io.dec_info.base_v_reg := instVldDest  // For loads: vd[4:0], For stores: vs3[4:0] (same bit position)
  
  // vl/evl (Vector Length) - with special cases for both loads and stores
  io.dec_info.vl := MuxLookup(Cat(isWhole, isMaskLS), vtype_vl, Seq(
    Cat(true.B, false.B)  -> whole_vl,        // Whole register: NFIELDS * VLEN / EEW
    Cat(false.B, true.B)  -> ((vtype_vl + 7.U) >> 3),  // Mask: ceil(vl/8)
    Cat(false.B, false.B) -> vtype_vl        // Normal: vl CSR value
  ))
  
  // eew_enc (Encoded Effective Element Width)
  io.dec_info.eew_enc := MuxLookup(Cat(isMaskLS, isIndex), instWidth, Seq(
    Cat(true.B, false.B)  -> 0.U,             // Mask: fixed at 8 bits (encoded as 0)
    Cat(false.B, true.B)  -> vtype_vsew,      // Indexed: config sew value
    Cat(false.B, false.B) -> instWidth        // Others: data width from instruction
  ))
  
  // emul_enc (Effective LMUL) - calculated based on instruction type  
  val emul_normal = vtype_vlmul + instWidth - vtype_vsew  // EMUL = LMUL * (EEW / SEW) in log domain
  io.dec_info.emul_enc := MuxLookup(Cat(isWhole, isIndex), vtype_vlmul, Seq(
    Cat(true.B, false.B)  -> whole_vlmul,     // Whole register: derived from nf
    Cat(false.B, true.B)  -> vtype_vlmul,     // Indexed: LMUL from vtype CSR
    Cat(false.B, false.B) -> emul_normal      // Others: EMUL = LMUL * (EEW / SEW)
  ))
  
  // Stride detection and outputs (for both loads and stores)
  io.dec_info.is_good_stride := strideIs1 || strideIs2 || strideIs4
  io.dec_info.stride_dir := strideIsNeg  // 0: positive, 1: negative
  io.dec_info.stride := rs2_data.asSInt
  io.dec_info.is_unit_stride := strideIs1

  // stride_enc: log2 of stride magnitude for good strides
  io.dec_info.stride_enc := MuxLookup(Cat(strideIs4, strideIs2, strideIs1), 0.U, Seq(
    Cat(false.B, false.B, true.B) -> 0.U,   // stride = 1*EEW
    Cat(false.B, true.B, false.B) -> 1.U,   // stride = 2*EEW  
    Cat(true.B, false.B, false.B) -> 2.U    // stride = 4*EEW
  ))
  
  // Segment outputs (for both loads and stores)
  io.dec_info.is_good_seg := !isSeg || (((instNf + 1.U) & instNf) === 0.U) // either not segmented (seg=1) or segment (seg=nf+1) is power of 2
  io.dec_info.seg_count   := Mux(isSeg, instNf + 1.U, 1.U)
  
  // seg_enc: log2 of segment count
  io.dec_info.seg_enc := PriorityEncoder(io.dec_info.seg_count)
  
  // Operation type flags (for both loads and stores)
  io.dec_info.is_mask := instMaskEnable // this is for masked LS (use the mask buffer) NOT mask-type LS
  io.dec_info.is_index := isIndex
  
  // base_addr (Base Memory Address) - from rs1 (for both loads and stores)
  io.dec_info.base_addr := rs1_data

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

}

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

package boom.v4.vec.generated.decode

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.VType

import boom.v4.common._
import boom.v4.vec.generated.{VtypeTable, VecTrace}

// GENERATED from src/main/nlhdl/vec/decode/VDecode.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
class VDecodeLsInfo(implicit p: Parameters) extends BoomBundle
{
  val is_mem       = Bool()
  val is_load      = Bool()
  val is_store     = Bool()
  val is_whole_reg = Bool()
  val nf           = UInt(3.W) // raw NFIELDS-1 field, width matches MicroOp.v_seg_nf
}
class VDecodeIO(implicit p: Parameters) extends BoomBundle
{
  val inst      = Input(Vec(coreWidth, UInt(32.W)))
  val valid     = Input(Vec(coreWidth, Bool()))
  val uop_in    = Input(Vec(coreWidth, new MicroOp()))
  val vtype_in  = Input(Vec(coreWidth, new VType()))
  val ls_in     = Input(Vec(coreWidth, new VDecodeLsInfo()))

  val uop_out   = Output(Vec(coreWidth, new MicroOp()))
  val is_arith  = Output(Vec(coreWidth, Bool()))
  val vill_trap = Output(Vec(coreWidth, Bool()))
}

class VDecode(implicit p: Parameters) extends BoomModule
{
  val io = IO(new VDecodeIO())
  private val OPIVV = 0.U(3.W)
  private val OPFVV = 1.U(3.W)
  private val OPMVV = 2.U(3.W)
  private val OPIVI = 3.U(3.W)
  private val OPIVX = 4.U(3.W)
  private val OPFVF = 5.U(3.W)
  private val OPMVX = 6.U(3.W)

  for (w <- 0 until coreWidth) {

    val inst   = io.inst(w)
    val funct6 = inst(31, 26)
    val funct3 = inst(14, 12)
    val vm     = inst(25)
    val vs1f   = inst(19, 15)
    val vs2f   = inst(24, 20)
    val rd     = inst(11, 7)

    // ---- 1. Recognition ----
    val is_OP_V   = inst(6, 0) === 0x57.U
    val is_vset_f3 = funct3 === "b111".U
    val is_arith_w = is_OP_V && !is_vset_f3
    io.is_arith(w) := is_arith_w

    //@req-spec-core.c3
    //@req-spec-decode.a4
    //@req-spec-decode.a6
    //@req-spec-decode.a7
    val uopOut = WireDefault(io.uop_in(w))

    // -- 4/5. dest_eew and the widening/narrowing families --
    val funct3_is_m_or_f = funct3 === OPMVV || funct3 === OPMVX || funct3 === OPFVV || funct3 === OPFVF
    val is_widening = funct6(5, 4) === 3.U && funct3_is_m_or_f
    val is_vfunary0      = funct6 === 0x12.U && funct3 === OPFVV
    val is_vfwcvt        = is_vfunary0 && vs1f(4, 3) === "b01".U
    val is_widening_total = is_widening || is_vfwcvt
    val sew = io.vtype_in(w).vsew
    val dest_eew = Mux(is_widening_total, sew +& 1.U, sew)

    //@req-spec-decode.d1
    val vtypeInfo         = VtypeTable.decode(io.vtype_in(w).asUInt)
    val v_emul_from_vtype = VtypeTable.emul(vtypeInfo, dest_eew)

    // -- 5. Single-register-destination families (v_emul forced to 1) --
    val is_compare = funct6(5, 3) === 3.U &&
      (funct3 === OPIVV || funct3 === OPIVX || funct3 === OPIVI || funct3 === OPFVV || funct3 === OPFVF)
    val funct3_is_ivv_ivx_ivi = funct3 === OPIVV || funct3 === OPIVX || funct3 === OPIVI
    val is_madc_sbc = (funct6 === 0x11.U || funct6 === 0x13.U) && funct3_is_ivv_ivx_ivi
    val is_mask_logical = funct6(5, 3) === 3.U && funct3 === OPMVV
    val is_vmunary0_maskresult = funct6 === 0x14.U && funct3 === OPMVV &&
      (vs1f === "b00001".U || vs1f === "b00010".U || vs1f === "b00011".U)
    val is_reduction = funct6(5, 3) === 0.U && (funct3 === OPMVV || funct3 === OPFVV)
    val is_widening_reduction = (funct6 === 0x30.U || funct6 === 0x31.U || funct6 === 0x33.U) &&
      (funct3 === OPMVV || funct3 === OPFVV)
    val is_vmv_s_x_or_vfmv_s_f = funct6 === 0x10.U && (funct3 === OPMVX || funct3 === OPFVF)
    val is_scalar_dest_family = funct6 === 0x10.U && (funct3 === OPMVV || funct3 === OPFVV)
    val is_single_reg_dest = is_compare || is_madc_sbc || is_mask_logical || is_vmunary0_maskresult ||
      is_reduction || is_widening_reduction || is_vmv_s_x_or_vfmv_s_f || is_scalar_dest_family

    val is_whole_reg_move = funct6 === 0x27.U && funct3 === OPIVI
    val v_emul_whole_reg  = vs1f +& 1.U

    //@req-spec-decode.d1
    val v_emul_final = Mux(is_whole_reg_move, v_emul_whole_reg,
                        Mux(is_single_reg_dest, 1.U, v_emul_from_vtype))

    // -- 3b. v_uses_vs1/vs2/vs3 --
    // SPEC DEFECT (reported, not resolved): nlhdl source states vs1 sub-opcode
    // exclusion two ways that disagree in how funct6 sets are specified, but in
    // practice is inert. This file implements the named enumeration {0x10,0x12,
    // 0x13,0x14} as the unambiguous interpretation.
    val is_vv_form = funct3 === OPIVV || funct3 === OPFVV || funct3 === OPMVV
    val is_funct5_selector_family = (funct3 === OPMVV || funct3 === OPFVV) &&
      (funct6 === 0x10.U || funct6 === 0x12.U || funct6 === 0x13.U || funct6 === 0x14.U)
    val v_uses_vs1_w = is_vv_form && !is_funct5_selector_family

    val is_ivv_ivx_ivi_or_fvf = funct3 === OPIVV || funct3 === OPIVX || funct3 === OPIVI || funct3 === OPFVF
    val vs2_zero_a = funct6 === 0x17.U && vm === 1.U && is_ivv_ivx_ivi_or_fvf
    val vs2_zero_b = funct6 === 0x10.U && vm === 1.U && (funct3 === OPMVX || funct3 === OPFVF)
    val vs2_zero_c = funct3 === OPMVV && funct6 === 0x14.U && vs1f === "b10001".U
    val v_uses_vs2_w = !(vs2_zero_a || vs2_zero_b || vs2_zero_c)

    val v_uses_vs3_w = !is_scalar_dest_family

    //@req-spec-decode.b1
    //@req-spec-decode.b2
    //@req-spec-decode.b3
    //@req-spec-issue.c4
    val is_segmented = io.ls_in(w).is_mem && !io.ls_in(w).is_whole_reg && (io.ls_in(w).nf =/= 0.U)
    uopOut.is_shared.get := is_segmented
    uopOut.iq_type(IQ_V_ALU) := is_arith_w || is_segmented

    when (is_arith_w) {
      // ---- 2. Default: vector destination group ----
      //@req-spec-core.c5
      //@req-spec-core.c7
      //@req-spec-core.c8
      uopOut.is_vec.get   := true.B
      uopOut.lvd.get      := rd
      uopOut.dst_rtype     := RT_VEC
      uopOut.lvs2.get      := vs2f
      uopOut.lrs2_rtype    := RT_X
      uopOut.lvm.get       := 0.U
      uopOut.lvs3.get      := rd
      uopOut.fu_code(FC_ALU) := true.B

      // ---- 3. Scalar feeders, and the x0 rule ----
      when (funct3 === OPIVX || funct3 === OPMVX) {
        uopOut.lrs1       := vs1f
        uopOut.lrs1_rtype := Mux(vs1f === 0.U, RT_ZERO, RT_FIX)
      } .elsewhen (funct3 === OPFVF) {
        uopOut.lrs1       := vs1f
        uopOut.lrs1_rtype := RT_FLT
      } .otherwise {
        uopOut.lvs1.get   := vs1f
        uopOut.lrs1_rtype := RT_X
      }

      when (is_scalar_dest_family) {
        uopOut.ldst      := rd
        uopOut.dst_rtype := Mux(funct3 === OPMVV, RT_FIX, RT_FLT)
      }

      // ---- 3b. v_uses_vs1/vs2/vs3 and v_is_masked ----
      uopOut.v_uses_vs1.get := v_uses_vs1_w
      uopOut.v_uses_vs2.get := v_uses_vs2_w
      uopOut.v_uses_vs3.get := v_uses_vs3_w
      uopOut.v_is_masked.get := !vm

      // ---- 4/5/6. EMUL ----
      uopOut.v_emul.get := v_emul_final
    }

    io.uop_out(w) := uopOut

    // ---- 6 (cont). vill_trap: OR of exactly two terms ----
    val vill_mirror_poisoned = io.vtype_in(w).vill && is_arith_w && !is_whole_reg_move
    val vill_emul_overflow = is_arith_w && !is_whole_reg_move && !is_single_reg_dest &&
      (v_emul_from_vtype === 0.U)
    io.vill_trap(w) := io.valid(w) && (vill_mirror_poisoned || vill_emul_overflow)

    // ---- 9. Trace ----
    when (io.valid(w) && is_arith_w) {
      VecTrace.traceDecode(
        "VDecode", "arith", io.uop_in(w).ftq_idx, io.uop_in(w).pc_lob,
        Seq(
          ("v_emul",     uopOut.v_emul.get),
          ("lvd",        uopOut.lvd.get),
          ("is_shared",  uopOut.is_shared.get),
          ("uses_vs1",   uopOut.v_uses_vs1.get),
          ("uses_vs2",   uopOut.v_uses_vs2.get),
          ("uses_vs3",   uopOut.v_uses_vs3.get)))
    }
    when (io.vill_trap(w)) {
      VecTrace.traceDecode(
        "VDecode", "vill", io.uop_in(w).ftq_idx, io.uop_in(w).pc_lob,
        Seq(
          ("mirror_poisoned", vill_mirror_poisoned),
          ("emul_overflow",   vill_emul_overflow)))
    }
  }
}

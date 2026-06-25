//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Decode (umbrella)
//------------------------------------------------------------------------------
//
// Recognizes RVV 1.0 instructions and populates the MicroOp vector fields. Ties
// together VLSDecode (load/store static fields) and VsetDecode (vset-class).
//
// Classification is opcode/funct3/mop based (matching the RVV encoding and the
// bobtail ls_decode reference) rather than a 381-row BitPat table -- vector
// control is mostly computed from instruction bit-fields, not looked up.
//
// Step 2 scope: decode emits a SINGLE uop per architectural instruction and fills
// only *static* fields. The runtime-vtype-dependent EMUL (= LMUL * EEW/SEW) is NOT
// computed here -- it is derived later in core.scala from vconfig.vlmul + v_eew +
// the widen/narrow class. We DO capture the static widen/narrow class and v_eew so
// that derivation has its inputs. The VConfigUnit (VCFG) owns the speculative
// vtype mirror that supplies vconfig to vector data ops. For immediate-vtype vset
// lanes (vsetivli/vsetvli) we snapshot the full vconfig (incl. vlmax) here; the
// VCFG reads it as dec_vtype_in. Vector arithmetic is decoded and routed to
// IQ_V_ALU but is tied off (never issues) until Goal 2.

package boom.v4.vec.decode

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

/** Tiny params accessor: HasBoomCoreParameters needs an implicit p; this exposes
  * vecVLen / the vlmax field width to the otherwise param-free VDecode helpers. */
private class VDecodeParams(implicit val p: Parameters) extends HasBoomCoreParameters

object VDecode
{
  def OP_V = "b1010111".U(7.W)   // OP-V: vector arithmetic + vset (OPCFG)

  /** Vector arithmetic: OP-V opcode, funct3 != 0b111 (0b111 is vset/OPCFG). */
  def isVecArith(inst: UInt): Bool = (inst(6, 0) === OP_V) && (inst(14, 12) =/= "b111".U)

  /** Vector data op routed to a vector issue queue (load / store / arith). */
  def isVec(inst: UInt)(implicit p: Parameters): Bool =
    VLSDecode.isVecMem(inst) || isVecArith(inst)

  /** Any recognized RVV instruction (data op or vset-class). */
  def isLegal(inst: UInt)(implicit p: Parameters): Bool =
    isVec(inst) || VsetDecode.isVset(inst)

  // OP-V funct3 (inst[14:12]) sub-encodings (RVV 1.0).
  def OPIVV = "b000".U(3.W)
  def OPFVV = "b001".U(3.W)
  def OPMVV = "b010".U(3.W)
  def OPIVI = "b011".U(3.W)
  def OPIVX = "b100".U(3.W)
  def OPFVX = "b101".U(3.W)
  def OPMVX = "b110".U(3.W)
  def OPCFG = "b111".U(3.W)   // vset-class

  /**
   * Decode the widening / narrowing class from funct6 (inst[31:26]) for arithmetic
   * ops. Returns (is_widen, is_narrow). Coverage (RVV 1.0, common families):
   *
   *  Widening (funct3 = OPMVV/OPMVX integer; result EEW = 2*SEW):
   *    vwaddu/vwadd/vwsubu/vwsub        funct6 0b110000..0b110011  (0x30..0x33)
   *    vwaddu.w/vwadd.w/vwsubu.w/vwsub.w funct6 0b110100..0b110111 (0x34..0x37)
   *    vwmulu/vwmulsu/vwmul             funct6 0x38, 0x3A, 0x3B
   *    vwmaccu/vwmacc/vwmaccus/vwmaccsu funct6 0x3C..0x3F
   *    i.e. the whole funct6 block 0x30..0x3F under OPMVV/OPMVX is widening.
   *
   *  Narrowing (funct3 = OPIVV/OPIVX/OPIVI; source EEW = 2*SEW):
   *    vnsrl/vnsra                      funct6 0b101100, 0b101101 (0x2C, 0x2D)
   *    vnclipu/vnclip                   funct6 0b101110, 0b101111 (0x2E, 0x2F)
   *    i.e. funct6 block 0x2C..0x2F under OPIVV/OPIVX/OPIVI is narrowing.
   *
   * NOT covered here (left false; refined in Milestone 2): widening FP (OPFVV/OPFVX
   * vfwadd/vfwmul/...), narrowing FP (vfncvt), the OPFVV widening reductions, and
   * vwmacc fp variants. These need FP-class handling and will be added in M2.
   */
  def widenNarrow(inst: UInt): (Bool, Bool) = {
    val f3     = inst(14, 12)
    val funct6 = inst(31, 26)
    val is_opmv = (f3 === OPMVV) || (f3 === OPMVX)
    val is_opiv = (f3 === OPIVV) || (f3 === OPIVX) || (f3 === OPIVI)
    // Widening integer: funct6 in 0x30..0x3F under OPMVV/OPMVX.
    val is_widen  = is_opmv && (funct6 >= "h30".U) && (funct6 <= "h3F".U)
    // Narrowing integer shift/clip: funct6 in 0x2C..0x2F under OPIVV/OPIVX/OPIVI.
    val is_narrow = is_opiv && (funct6 >= "h2C".U) && (funct6 <= "h2F".U)
    (is_widen, is_narrow)
  }

  /**
   * Compute VLMAX = LMUL * VLEN / SEW for an immediate-vtype vset, where
   * SEW = 8 << vsew and LMUL is encoded in the 3-bit vlmul field:
   *   0=m1, 1=m2, 2=m4, 3=m8, 5=mf8, 6=mf4, 7=mf2 (4 is reserved).
   * For fractional LMUL the spec floors: VLMAX = floor(LMUL * VLEN / SEW).
   *
   * Implementation: VLEN is a Scala Int constant (power of two) and SEW = 8<<vsew,
   * so VLEN/SEW = VLEN >> (3+vsew) is an exact shift. We enumerate the 4 SEW values
   * (vsew 0..3) x 7 valid LMUL settings with a MuxLookup over Cat(vlmul, vsew),
   * folding the fractional-LMUL division (with floor) at Scala-Int compute time.
   * width is the VConfig.vlmax field width (log2Ceil(VLEN+1)). Reserved/unsupported
   * encodings (vsew>=4, vlmul==4) map to 0.
   */
  def vlmaxOf(vsew: UInt, vlmul: UInt, vecVLen: Int, width: Int): UInt = {
    // Integer or fractional multiplier for each vlmul encoding, as (numer, denom).
    val lmulTable = Seq(
      0 -> (1, 1),  // m1
      1 -> (2, 1),  // m2
      2 -> (4, 1),  // m4
      3 -> (8, 1),  // m8
      5 -> (1, 8),  // mf8
      6 -> (1, 4),  // mf4
      7 -> (1, 2)   // mf2
    )
    val cases =
      for {
        sew  <- 0 until 4                 // vsew 0..3 => SEW 8,16,32,64
        (mul, (num, den)) <- lmulTable
      } yield {
        val sewBytes = 8 << sew
        // floor(VLEN * num / den / SEW)
        val vlmax = (vecVLen.toLong * num) / (den.toLong * sewBytes)
        // key = {vlmul[2:0], vsew[2:0]}
        Cat(mul.U(3.W), sew.U(3.W)) -> vlmax.U(width.W)
      }
    MuxLookup(Cat(vlmul, vsew), 0.U(width.W))(cases)
  }

  /**
   * Populate the vector-related MicroOp fields in place. Call only when
   * isLegal(inst) holds; overrides the scalar-decode assignments via Chisel
   * last-connect-wins. For vector data ops the runtime-vtype-dependent v_emul is
   * left for the core.scala derivation (vconfig.vlmul + v_eew + widen/narrow); the
   * VConfigUnit (VCFG) supplies the vconfig snapshot. For immediate-vtype vset
   * lanes we snapshot the full vconfig (incl. vlmax) here for the VCFG to read.
   */
  def decode(uop: MicroOp, inst: UInt)(implicit p: Parameters): Unit = {
    val ls       = VLSDecode(inst)
    val vset     = VsetDecode(inst)
    val is_arith = isVecArith(inst)
    val is_mem   = VLSDecode.isVecMem(inst)
    val params   = new VDecodeParams
    val (is_widen, is_narrow) = widenNarrow(inst)

    val rd  = inst(11, 7)
    val rs1 = inst(19, 15)
    val rs2 = inst(24, 20)
    val vm  = inst(25).asBool   // RVV mask bit: 1 = unmasked, 0 = masked (read v0)

    // Fully own iq_type for every recognized RVV uop: the scalar decode derives
    // iq_type bits 0..3 from cs.fu_code, which is a don't-care default for these
    // instructions (they aren't in the scalar tables). Clear all, then set the
    // one correct bit below. Likewise clear the scalar load/store-queue flags --
    // vector memory ops use the V-LSU, never the scalar LDQ/STQ.
    for (i <- 0 until IQ_SZ) { uop.iq_type(i) := false.B }
    uop.uses_ldq := false.B
    uop.uses_stq := false.B

    when (vset.is_vset) {
      // vset-class: SCALAR integer uop. Reads rs1 (AVL, except vsetivli's uimm)
      // and writes rd (= new VL). Not is_vec; flows through the scalar pipeline.
      // The actual VL computation + vtype/vl update is wired in Steps 3/6; here
      // we only set the scalar operand types + the decode-time classification.
      uop.is_vec       := false.B
      // Classification bits consumed by the VConfigUnit (VCFG) and Step 9 routing.
      uop.is_vsetivli := vset.is_vsetivli
      uop.is_vsetvli  := vset.is_vsetvli
      uop.is_vsetvl   := vset.is_vsetvl
      // F4: vsetvl (register VTYPE) must serialize. Its VTYPE comes from rs2 at
      // execute, so the VCFG vtype mirror cannot be updated at decode -- a younger
      // vector op would be sized (EMUL) against a stale vtype. Mark is_unique so the
      // pipeline drains and the mirror update lands before any younger uop decodes.
      // vsetivli/vsetvli have immediate VTYPE (mirror updates at decode) and must
      // NOT serialize, so leave is_unique false for them.
      uop.is_unique := vset.is_vsetvl
      // F5 routing note: VDecode currently routes ALL three vset forms to the int
      // ALU (IQ_ALU/FC_ALU) below. Per the plan, vsetivli is FRONT-END/VCFG-only:
      // both its VTYPE and AVL are immediate, so the VCFG resolves VL at decode and
      // it needs no back-end EU -- the int-ALU dispatch for vsetivli is redundant.
      // We intentionally do NOT change the routing here (Step 9 finalizes vset
      // execution); this redundancy is reconciled in Step 9. vsetvli/vsetvl genuinely
      // need the int ALU (VL = min(rs1,VLMAX), and vsetvl's VTYPE from rs2).
      // Route as a scalar ALU uop so it allocates an integer rd, writes back and
      // never stalls the ROB. The actual min(AVL,VLMAX) / vtype update datapath
      // is wired in Step 6; until then the written VL value is a placeholder.
      for (i <- 0 until FC_SZ) { uop.fu_code(i) := false.B }
      uop.fu_code(FC_ALU) := true.B
      uop.iq_type(IQ_ALU) := true.B
      uop.dst_rtype    := Mux(vset.rd_is_x0, RT_ZERO, RT_FIX)
      uop.lrs1_rtype   := Mux(vset.is_vsetivli || vset.rs1_is_x0, RT_ZERO, RT_FIX)
      uop.lrs2_rtype   := Mux(vset.is_vsetvl, RT_FIX, RT_X)   // vsetvl reads rs2 (vtype)
      // Snapshot the immediate vtype into this uop's vconfig (valid for
      // vsetvli/vsetivli). vsetvl gets vtype from rs2 at execute (Step 6 / VCFG).
      uop.vconfig.vsew  := vset.vsew
      uop.vconfig.vlmul := vset.vlmul
      uop.vconfig.vta   := vset.vta
      uop.vconfig.vma   := vset.vma
      // F-vlmax: VLMAX = floor(LMUL * VLEN / SEW) from the immediate vtype. The
      // core-level VCFG reads this vconfig (incl. vlmax) as dec_vtype_in for
      // immediate-vtype vset lanes. vsetvl's vlmax is recomputed at execute from
      // the rs2 vtype (its vconfig here is don't-care).
      uop.vconfig.vlmax := vlmaxOf(vset.vsew, vset.vlmul, params.vecVLen,
                                   uop.vconfig.vlmax.getWidth)
    } .otherwise {
      // Vector data op (load / store / arith).
      uop.is_vec := true.B

      // Route to the correct vector issue queue.
      uop.iq_type(IQ_V_LOAD)  := ls.is_load
      uop.iq_type(IQ_V_STORE) := ls.is_store
      uop.iq_type(IQ_V_ALU)   := is_arith

      uop.v_seg_nf := ls.nf

      // F2/R1: capture the widen/narrow class (static, from funct6) for BOTH mem
      // and arith. The final v_emul (= LMUL * EEW/SEW) is NOT computed here -- it is
      // derived later in core.scala from vconfig.vlmul + v_eew + v_widen/v_narrow.
      uop.v_widen  := is_widen
      uop.v_narrow := is_narrow

      // v_eew: for MEMORY ops the element width is encoded in the instruction
      // ({mew, width[1:0]}); we use the low two width bits (mew=0 in M1, see below).
      // For ARITH ops there is no instruction-encoded EEW -- the element width is
      // vtype SEW (widen => 2*SEW dest, narrow => 2*SEW src). Since SEW is not known
      // at decode, we cannot fill an absolute v_eew for arith here; the SEW-derived
      // EEW is filled by the VCFG / core.scala from vconfig.vsew + v_widen/v_narrow.
      // We assign 0 for arith to give the field a defined decode value (it is a
      // don't-care until the VCFG fills it).
      uop.v_eew := Mux(is_mem, ls.v_eew, 0.U)

      // F1: mask flag. vm==1 means UNMASKED in RVV. lvm = v0 (logical reg 0) stays
      // as-is; v_unmasked records whether the op reads v0 as a mask.
      uop.v_unmasked := vm
      // Vector mask source register is always v0.
      uop.lvm := 0.U

      when (is_mem) {
        // F3: mew=1 selects EEW=128, reserved/unsupported in Caracal M1. ls.eew_unsup
        // surfaces this; the illegal-instruction path is owned by decode.scala. We do
        // NOT silently treat mew=1 as a 64b op -- v_eew above uses only width[1:0],
        // and eew_unsup flags the unsupported case for the exception path / VCFG.
        // (No exception is raised from this object; see VLSDecode F3 note.)
        // Memory ops: rs1 is the integer base address (scalar source).
        uop.lrs1       := rs1
        uop.lrs1_rtype := RT_FIX
        // rs2 field: scalar stride (strided) or vector index (indexed) or unused.
        when (ls.is_strided) {
          uop.lrs2       := rs2
          uop.lrs2_rtype := RT_FIX
        } .otherwise {
          uop.lrs2_rtype := RT_X
          uop.lvs2       := rs2     // vector index reg (indexed); don't-care otherwise
        }
        when (ls.is_load) {
          // vd is the vector destination.
          uop.lvd       := rd
          uop.dst_rtype := RT_VEC
        } .otherwise {
          // store: vs3 (= rd field) is the vector data SOURCE; no register dest.
          uop.lvs3      := rd
          uop.dst_rtype := RT_X
        }
      } .otherwise {
        // Vector arithmetic (tied off in Goal 1). vd = dest, vs1/vs2 = sources.
        // .vx/.vi variants substitute a scalar/imm for vs1; handled in Goal 2.
        uop.lvd       := rd
        uop.dst_rtype := RT_VEC
        uop.lvs1      := rs1
        uop.lvs2      := rs2
        uop.lrs1_rtype := RT_X
        uop.lrs2_rtype := RT_X
      }
    }
  }
}

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
//
// VDecode -- the arithmetic RVV opcode decoder: one combinational lane per
// decode lane that turns an OP-V instruction word into the vector fields of
// ONE MicroOp. Single-cycle combinational, holds no register, no memory. ONE
// uOP per instruction: no cracking, no micro-op expansion -- an OP.v stays one
// uOP through decode/rename/ROB/issue and is expanded into nOP.v only inside
// the vector LS AGEN, which this module never feeds.
//
// NOT a full RVV decoder: the CII issue packet carries the raw instruction
// word and the coprocessor decodes the operation itself. This module decodes
// only what the HOST needs -- which architectural registers are read/written,
// which rename space each destination belongs to, which issue queue routes
// the uOP, the destination group size, and whether a second execution
// resource is needed. funct6 is read ONLY for the destination-SHAPE classes
// (widening, single-register destination, whole-register move).
//
// Instantiated once, as `arith`, by VecDecode alongside VLSDecode (`ls`),
// VsetDecode (`vset`) and VConfigUnit (`vcfg`). Instantiates nothing.
//
// Governing spec anchors: frontend.rst `vector-rvv-decode`, glossary.rst
// `glossary-terms`, issue.rst `cii-shared-sched`, midcore.rst `old-vd`.

/**
 * VDecodeLsInfo -- the narrow subset of VLSDecode's static access descriptor
 * this module needs: is_mem, is_load, is_store, is_whole_reg and the raw nf
 * field. This is a LOCAL bundle, not VLSDecode's own descriptor type: VDecode's
 * hierarchy.yaml `depends_on:` lists only [MicroOp, VecTrace, VtypeTable] --
 * VLSDecode is not a dependency edge of this node. VecDecode (the parent) maps
 * VLSDecode's real descriptor fields onto this view when it wires `ls` to
 * `arith`; that mapping is the parent's wiring responsibility, not this file's.
 *
 * is_load/is_store are declared here because the nlhdl ports section names them
 * as part of what this module's `ls_in` port needs, but no assignment below
 * reads either one -- only is_mem/is_whole_reg/nf feed the is_shared formula.
 * Declared, not invented usage for them.
 */
class VDecodeLsInfo(implicit p: Parameters) extends BoomBundle
{
  val is_mem       = Bool()
  val is_load      = Bool()
  val is_store     = Bool()
  val is_whole_reg = Bool()
  val nf           = UInt(3.W) // raw NFIELDS-1 field, width matches MicroOp.v_seg_nf
}

/** VDecode's IO: every port is a `Vec(coreWidth, ...)`, lane-indexed identically. */
class VDecodeIO(implicit p: Parameters) extends BoomBundle
{
  // Clock/reset are the implicit Chisel signals; this module has no register
  // and no memory, so they reach only the guarded trace statements below.
  // No ready/valid handshake: decode cannot stall on this module.
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

  // ---- RVV OP-V funct3 sub-opcodes (RVV 1.0 encoding table). Local to this
  // file: not part of ScalarOpConstants, and no other generated source names
  // them, so they are declared once here rather than re-spelled per branch. ----
  private val OPIVV = 0.U(3.W)
  private val OPFVV = 1.U(3.W)
  private val OPMVV = 2.U(3.W)
  private val OPIVI = 3.U(3.W)
  private val OPIVX = 4.U(3.W)
  private val OPFVF = 5.U(3.W)
  private val OPMVX = 6.U(3.W)
  // funct3 = 0b111 (OPCFG) is VsetDecode's; never matched as is_arith here.

  for (w <- 0 until coreWidth) {

    val inst   = io.inst(w)
    val funct6 = inst(31, 26)
    val funct3 = inst(14, 12)
    val vm     = inst(25)
    val vs1f   = inst(19, 15)
    val vs2f   = inst(24, 20)
    val rd     = inst(11, 7)

    // ==========================================================================
    // ---- 1. Recognition ----
    // ==========================================================================
    // A lane holds vector arithmetic when its opcode is OP-V (0x57) and its
    // funct3 is not 0b111 (OPCFG/vset, VsetDecode's). Vector loads/stores use
    // LOAD-FP/STORE-FP opcodes and are never matched here.
    val is_OP_V   = inst(6, 0) === 0x57.U
    val is_vset_f3 = funct3 === "b111".U
    val is_arith_w = is_OP_V && !is_vset_f3
    io.is_arith(w) := is_arith_w

    //@req-spec-core.c3
    //@req-spec-decode.a4
    //@req-spec-decode.a6
    //@req-spec-decode.a7
    // Each lane is a single-cycle combinational function producing exactly ONE
    // decoded uOP packet: uop_out(w) is a function of inst(w)/uop_in(w)/
    // vtype_in(w)/ls_in(w) in the same cycle. No state, no sequencing, no
    // iteration, no second output beat, and no configuration of this `for`
    // loop can emit more than `coreWidth` uOPs per cycle.
    val uopOut = WireDefault(io.uop_in(w))

    // ==========================================================================
    // ---- Classification wires (funct6/funct3/vs1/vm-based), computed for
    // every lane regardless of is_arith -- cheap combinational logic, gated
    // into uopOut only inside the `when (is_arith_w)` block below. ----
    // ==========================================================================

    // -- 4/5. dest_eew and the widening/narrowing families --
    // WIDENING: funct6 0x30..0x3F (top 2 bits = 11) under OPMVV/OPMVX/OPFVV/OPFVF
    // only (vwadd/vwsub/vwmul/vwmacc, the .w forms, and the OPFVV/OPFVF widening
    // arithmetic/reduction families). NOT OPIVV/OPIVX/OPIVI.
    val funct3_is_m_or_f = funct3 === OPMVV || funct3 === OPMVX || funct3 === OPFVV || funct3 === OPFVF
    val is_widening = funct6(5, 4) === 3.U && funct3_is_m_or_f
    // NARROWING: funct6 0x2C..0x2F under OPIVV/OPIVX/OPIVI (vnsrl/vnsra/vnclip).
    // Dest stays SEW-wide (their SOURCE is 2*SEW) -- no dest_eew adjustment.
    //
    // VFUNARY0 (funct6 0x12, OPFVV): vs1 is an opcode extension whose top two
    // bits (vs1(4,3)) partition the block into vfcvt.*/.rtz (0b00, dest SEW,
    // no adjustment), vfwcvt.* (0b01, dest 2*SEW -- pass vsew+1 exactly like
    // the 0x30..0x3F widening families), and vfncvt.* (0b10, dest SEW, source
    // 2*SEW -- no adjustment, same reasoning as vnsrl/vnsra/vnclip; NOT "fixed"
    // by symmetry with vfwcvt). VXUNARY0 is the SAME funct6 0x12 but under
    // OPMVV (vzext/vsext): its destination is SEW and the source is narrower,
    // so it needs no adjustment either -- gate on funct3 as well as funct6 so
    // the two families behind this one funct6 are not conflated.
    val is_vfunary0      = funct6 === 0x12.U && funct3 === OPFVV
    val is_vfwcvt        = is_vfunary0 && vs1f(4, 3) === "b01".U
    val is_widening_total = is_widening || is_vfwcvt
    val sew = io.vtype_in(w).vsew
    val dest_eew = Mux(is_widening_total, sew +& 1.U, sew)

    // -- EMUL from VtypeTable, the sanctioned derivation path. VtypeTable.decode
    // takes the raw UInt encoding (it calls VType.fromUInt internally), so the
    // VType port bundle is bridged via .asUInt -- a mechanical round-trip
    // (VType.fromUInt itself uses `.asTypeOf(new VType)`), not a reinterpretation.
    //@req-spec-decode.d1
    val vtypeInfo         = VtypeTable.decode(io.vtype_in(w).asUInt)
    val v_emul_from_vtype = VtypeTable.emul(vtypeInfo, dest_eew)
    // ASSUMPTION (not independently re-derivable per the dependency contract):
    // for this design's parameterization (dest_eew is SEW or SEW+1, i.e. a
    // widening factor of at most 2x, and maxMembers a power of two), the ONLY
    // reachable "EMUL exceeds maxMembers" case is exactly LMUL=8 widening,
    // whose true EMUL (2*maxMembers) is exactly 2^emulWidth and therefore
    // truncates to precisely 0 in VtypeTable.emul's returned value (never to
    // a non-zero out-of-range value). Detecting the overflow as
    // `v_emul_from_vtype === 0.U` is therefore complete for this module
    // without recomputing the raw pre-clamp magnitude itself (forbidden by
    // the dependencies section: "must not compute a group size any other
    // way"). If maxMembers/eLen/vLen relationships ever change such that a
    // widening factor > 2x becomes reachable here, this detector would need
    // re-review.

    // -- 5. Single-register-destination families (v_emul forced to 1) --
    // Integer + FP compares: funct6 0x18-0x1F under OPIVV/OPIVX/OPIVI (int) or
    // OPFVV/OPFVF (FP).
    val is_compare = funct6(5, 3) === 3.U &&
      (funct3 === OPIVV || funct3 === OPIVX || funct3 === OPIVI || funct3 === OPFVV || funct3 === OPFVF)
    // vmadc/vmsbc: funct6 0x11/0x13, OPIVV/OPIVX/OPIVI only (no OPMVV/OPFVV
    // forms exist for these, but the funct3 restriction keeps this predicate
    // from also matching funct6 0x13 under OPFVV, which is VFUNARY1 -- an
    // unrelated funct5-selector family, NOT a single-register destination).
    val funct3_is_ivv_ivx_ivi = funct3 === OPIVV || funct3 === OPIVX || funct3 === OPIVI
    val is_madc_sbc = (funct6 === 0x11.U || funct6 === 0x13.U) && funct3_is_ivv_ivx_ivi
    // Mask-logical family (vmand.mm etc): funct6 0x18-0x1F under OPMVV.
    val is_mask_logical = funct6(5, 3) === 3.U && funct3 === OPMVV
    // vmsbf.m/vmsif.m/vmsof.m (VMUNARY0, funct6 0x14, OPMVV): distinguished
    // from viota.m/vid.v (same funct6/funct3) by the vs1 sub-opcode selector.
    // viota.m and vid.v are NOT in this set -- they take the vtype-derived count.
    val is_vmunary0_maskresult = funct6 === 0x14.U && funct3 === OPMVV &&
      (vs1f === "b00001".U || vs1f === "b00010".U || vs1f === "b00011".U)
    // Reductions vred*/vfred*: funct6 0x00-0x07 under OPMVV/OPFVV.
    val is_reduction = funct6(5, 3) === 0.U && (funct3 === OPMVV || funct3 === OPFVV)
    // Widening reductions vwredsum(u)/vfwred*: funct6 in {0x30,0x31,0x33}.
    val is_widening_reduction = (funct6 === 0x30.U || funct6 === 0x31.U || funct6 === 0x33.U) &&
      (funct3 === OPMVV || funct3 === OPFVV)
    // vmv.s.x/vfmv.s.f: funct6 0x10 under OPMVX/OPFVF.
    val is_vmv_s_x_or_vfmv_s_f = funct6 === 0x10.U && (funct3 === OPMVX || funct3 === OPFVF)
    // Part-3's scalar-destination family (vmv.x.s/vcpop.m/vfirst.m/vfmv.f.s):
    // funct6 0x10 under OPMVV/OPFVV -- allocates no vector group at all.
    val is_scalar_dest_family = funct6 === 0x10.U && (funct3 === OPMVV || funct3 === OPFVV)
    val is_single_reg_dest = is_compare || is_madc_sbc || is_mask_logical || is_vmunary0_maskresult ||
      is_reduction || is_widening_reduction || is_vmv_s_x_or_vfmv_s_f || is_scalar_dest_family

    // -- 6. Whole-register move: vmv<n>r.v, funct6 0x27 under OPIVI. Group size
    // is NREG = (unsigned vs1 field) + 1, legal values 1/2/4/8, taken from its
    // OWN encoding rather than the (possibly poisoned) vtype mirror. --
    val is_whole_reg_move = funct6 === 0x27.U && funct3 === OPIVI
    val v_emul_whole_reg  = vs1f +& 1.U

    //@req-spec-decode.d1
    // Final v_emul: whole-register move > single-register-dest override >
    // the general vtype-derived count, in that priority order.
    val v_emul_final = Mux(is_whole_reg_move, v_emul_whole_reg,
                        Mux(is_single_reg_dest, 1.U, v_emul_from_vtype))

    // -- 3b. v_uses_vs1/vs2/vs3 --
    // v_uses_vs1: true for the vector-vector forms (OPIVV/OPFVV/OPMVV), except
    // the FUNCT5-SELECTOR families under OPMVV/OPFVV whose vs1 is a sub-opcode,
    // not a register: VWXUNARY0/VWFUNARY0 (0x10), VXUNARY0/VFUNARY0 (0x12),
    // VFUNARY1 (0x13), VMUNARY0 (0x14).
    // SPEC DEFECT (reported, not resolved): the nlhdl source states this
    // exclusion two ways that disagree -- an algebraic test "funct6(5,2) ===
    // 0b0100" (which selects funct6 {0x10,0x11,0x12,0x13}, NOT 0x14) and a
    // named enumeration "0b010000, 0b010010, 0b010011, 0b010100" (i.e.
    // {0x10,0x12,0x13,0x14}, omitting 0x11). The two sets differ at both
    // ends. In practice this is inert -- funct6 0x11 (VMADC/VMSBC) is never
    // encoded under OPMVV/OPFVV in the RVV 1.0 table, only under
    // OPIVV/OPIVX/OPIVI, so it is out of scope for this test either way --
    // but the formula and the list are not the same set as WRITTEN. This
    // file implements the named enumeration {0x10,0x12,0x13,0x14}, since it
    // is unambiguous and matches the real instruction set; the algebraic
    // formula is not used.
    val is_vv_form = funct3 === OPIVV || funct3 === OPFVV || funct3 === OPMVV
    val is_funct5_selector_family = (funct3 === OPMVV || funct3 === OPFVV) &&
      (funct6 === 0x10.U || funct6 === 0x12.U || funct6 === 0x13.U || funct6 === 0x14.U)
    val v_uses_vs1_w = is_vv_form && !is_funct5_selector_family

    // v_uses_vs2: true except for the three vs2-reserved-zero families:
    //   (a) funct6 0x17 with vm=1 under OPIVV/OPIVX/OPIVI/OPFVF (vmv.v.v/
    //       vmv.v.x/vmv.v.i/vfmv.v.f) -- vm=0 at the same funct6 is vmerge/
    //       vfmerge, which DOES read vs2, so both funct6 AND vm are checked.
    //   (b) funct6 0x10 with vm=1 under OPMVX/OPFVF (vmv.s.x/vfmv.s.f) -- the
    //       same funct6 under OPMVV/OPFVV is the scalar-dest family, whose
    //       vs2 IS the source vector, so it is deliberately excluded here.
    //   (c) vid.v alone: OPMVV, funct6 0x14, vs1 selector 0b10001. Its
    //       VMUNARY0 siblings (vmsbf/vmsif/vmsof/viota.m) all read vs2.
    val is_ivv_ivx_ivi_or_fvf = funct3 === OPIVV || funct3 === OPIVX || funct3 === OPIVI || funct3 === OPFVF
    val vs2_zero_a = funct6 === 0x17.U && vm === 1.U && is_ivv_ivx_ivi_or_fvf
    val vs2_zero_b = funct6 === 0x10.U && vm === 1.U && (funct3 === OPMVX || funct3 === OPFVF)
    val vs2_zero_c = funct3 === OPMVV && funct6 === 0x14.U && vs1f === "b10001".U
    val v_uses_vs2_w = !(vs2_zero_a || vs2_zero_b || vs2_zero_c)

    // v_uses_vs3: true exactly when the lane has a VECTOR destination, i.e.
    // wherever dst_rtype ends up RT_VEC -- every arithmetic lane except the
    // scalar-destination family.
    val v_uses_vs3_w = !is_scalar_dest_family

    // -- 7. is_shared: SEGMENTED load/store only, never arithmetic. Computed
    // for EVERY lane (not gated on is_arith_w) since VDecode is the sole
    // owner of this field regardless of which decoder claimed the lane; the
    // is_mem term is what forces it false for every arithmetic lane. --
    //@req-spec-decode.b1
    //@req-spec-decode.b2
    //@req-spec-decode.b3
    //@req-spec-issue.c4
    // nf is NFIELDS-1, so "segmented" is `nf =/= 0`, not `nf > 1`. The
    // whole-register forms reuse nf to encode NREG-1 (e.g. vl8r.v carries
    // nf=7); ls_in.is_whole_reg excludes them from being misread as an
    // 8-field segmented access.
    val is_segmented = io.ls_in(w).is_mem && !io.ls_in(w).is_whole_reg && (io.ls_in(w).nf =/= 0.U)
    uopOut.is_shared.get := is_segmented
    // A vector ARITHMETIC instruction is NEVER is_shared: is_arith_w and
    // is_segmented are mutually exclusive (OP-V arithmetic vs. LOAD-FP/
    // STORE-FP), so this single merged assignment is the ONLY writer of
    // iq_type(IQ_V_ALU) -- no double-connect with the arithmetic block below.
    uopOut.iq_type(IQ_V_ALU) := is_arith_w || is_segmented

    when (is_arith_w) {
      // ==========================================================================
      // ---- 2. Default: vector destination group ----
      // ==========================================================================
      //@req-spec-core.c5
      //@req-spec-core.c7
      //@req-spec-core.c8
      // The one uOP represents the WHOLE LMUL/EMUL destination register group
      // and occupies ONE ROB entry. Nothing here indexes a group member; the
      // group is named by its architectural number (lvd) and size (v_emul).
      uopOut.is_vec.get   := true.B
      uopOut.lvd.get      := rd
      uopOut.dst_rtype     := RT_VEC
      uopOut.lvs2.get      := vs2f
      uopOut.lrs2_rtype    := RT_X
      uopOut.lvm.get       := 0.U
      uopOut.lvs3.get      := rd
      uopOut.fu_code(FC_ALU) := true.B

      // ==========================================================================
      // ---- 3. Scalar feeders, and the x0 rule ----
      // ==========================================================================
      // ===> THE x0 CONVERSION IS MANDATORY: RVV permits .vx/vmv.s.x to read
      // x0, and rename-stage.scala:109 asserts !(r_valid && lrs1_rtype===RT_FIX
      // && lrs1===0.U). Mirror the scalar decoder (decode.scala:505): RT_FIX
      // becomes RT_ZERO when the specifier is 0. RT_FLT needs no such
      // conversion -- f0 is a real register.
      when (funct3 === OPIVX || funct3 === OPMVX) {
        uopOut.lrs1       := vs1f
        uopOut.lrs1_rtype := Mux(vs1f === 0.U, RT_ZERO, RT_FIX)
      } .elsewhen (funct3 === OPFVF) {
        uopOut.lrs1       := vs1f
        uopOut.lrs1_rtype := RT_FLT
      } .otherwise {
        // OPIVV/OPFVV/OPMVV (real vector source) and OPIVI (vs1 is simm5,
        // don't-care since v_uses_vs1 is false for it -- see part 3b).
        uopOut.lvs1.get   := vs1f
        uopOut.lrs1_rtype := RT_X
      }

      // Scalar-DESTINATION arithmetic (funct6 0b010000): vmv.x.s/vcpop.m/
      // vfirst.m (OPMVV, writes an integer GPR) and vfmv.f.s (OPFVV, writes
      // an FP register). is_vec stays true -- these still execute on the
      // coprocessor. lrs1_rtype already stayed RT_X via the `otherwise`
      // branch above (vs1 here is a funct5 selector, not a register).
      when (is_scalar_dest_family) {
        uopOut.ldst      := rd
        uopOut.dst_rtype := Mux(funct3 === OPMVV, RT_FIX, RT_FLT)
      }

      // ==========================================================================
      // ---- 3b. v_uses_vs1/vs2/vs3 and v_is_masked ----
      // ==========================================================================
      // ===> AN UNENCODED SOURCE LEFT UNMARKED IS A HANG, NOT A LOST
      // OPTIMIZATION -- see the nlhdl source's part 3b note on vadd.vx's
      // unencoded vs1 aliasing the mask register's mapping.
      uopOut.v_uses_vs1.get := v_uses_vs1_w
      uopOut.v_uses_vs2.get := v_uses_vs2_w
      uopOut.v_uses_vs3.get := v_uses_vs3_w
      // RVV's vm bit is 1 for UNMASKED, so v_is_masked is its complement.
      uopOut.v_is_masked.get := !vm

      // ==========================================================================
      // ---- 4/5/6. EMUL ----
      // ==========================================================================
      uopOut.v_emul.get := v_emul_final
    }

    io.uop_out(w) := uopOut

    // ==========================================================================
    // ---- 6 (cont). vill_trap: OR of exactly two terms ----
    // ==========================================================================
    // valid qualifies the illegal output (per the nlhdl ports section): the
    // decoded uOP fields above are deliberately NOT gated by valid (no mux on
    // the decode critical path for no consumer), but vill_trap is.
    val vill_mirror_poisoned = io.vtype_in(w).vill && is_arith_w && !is_whole_reg_move
    // ASSUMPTION: the EMUL-overflow term is additionally excluded for the
    // single-register-destination overrides (part 5), on the same reasoning
    // the nlhdl source gives explicitly for whole-register moves ("exclude it
    // from the vtype-dependency term"). A widening REDUCTION (vwredsum(u),
    // vfwredosum/vfwredusum) always writes exactly one register regardless of
    // LMUL, so its true destination can never overflow maxMembers even though
    // the generic vtype-derived candidate (which assumes a full 2*LMUL-member
    // group) can wrap to 0 for it at LMUL=8. Without this exclusion, a legal
    // vfwredosum.vs at LMUL=8 would be spuriously trapped illegal. The nlhdl
    // source does not spell this interaction out for the reduction case the
    // way it does for whole-register moves; this exclusion is this file's
    // conservative resolution of that gap, applied to avoid the "wrongly-true
    // bit" failure mode the source repeatedly warns about elsewhere.
    val vill_emul_overflow = is_arith_w && !is_whole_reg_move && !is_single_reg_dest &&
      (v_emul_from_vtype === 0.U)
    io.vill_trap(w) := io.valid(w) && (vill_mirror_poisoned || vill_emul_overflow)

    // ==========================================================================
    // ---- 9. Trace ----
    // ==========================================================================
    // Decode-stage variant: there is no rob_idx yet (allocated at dispatch),
    // so this uses traceDecode's ftq_idx/pc_lob keying, gated on the vecTrace
    // plusarg and !reset (both handled inside VecTrace.traceDecode/emitLine).
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

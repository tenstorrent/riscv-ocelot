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
import freechips.rocketchip.rocket.{M_XRD, M_XWR}

import boom.v4.common._
import boom.v4.exu.BrUpdateInfo
import boom.v4.vec.generated.{VtypeTable, VecTrace}

// GENERATED from src/main/nlhdl/vec/decode/VecDecode.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
class VecDecodeIO(implicit p: Parameters) extends BoomBundle
{

  // ---- Decode feed, per lane ----
  val dec_insns    = Input(Vec(coreWidth, UInt(32.W)))
  val dec_valids   = Input(Vec(coreWidth, Bool()))
  val dec_fire     = Input(Vec(coreWidth, Bool()))
  val dec_uops_in  = Input(Vec(coreWidth, new MicroOp()))

  // ---- Decode results, per lane ----
  val dec_uops_out    = Output(Vec(coreWidth, new MicroOp()))
  val dec_vec_illegal = Output(Vec(coreWidth, Bool()))
  val dec_vl_imm       = Output(Vec(coreWidth, UInt(vecVLSz.W)))
  val dec_vl_imm_valid = Output(Vec(coreWidth, Bool()))

  val ren_br_tags    = Input(Vec(coreWidth + 1, Valid(UInt(brTagSz.W))))
  val ren_br_vconfig = Input(Vec(coreWidth + 1, new VType))
  val brupdate       = Input(new BrUpdateInfo)
  val rollback       = Input(Bool())
  val com_valids     = Input(Vec(coreWidth, Bool()))
  val com_is_vset    = Input(Vec(coreWidth, Bool()))
  val com_vtype      = Input(Vec(coreWidth, new VType))
  val csr_vtype      = Input(new VType)
  val rob_empty      = Input(Bool())
}

class VecDecode(implicit p: Parameters) extends BoomModule
{
  val io = IO(new VecDecodeIO())

  val arith = Module(new VDecode())
  val ls    = Module(new VLSDecode())
  val vset  = Module(new VsetDecode())
  val vcfg  = Module(new VConfigUnit())

  // ==========================================================================
  // ---- vcfg: whole-vector pass-throughs (this module reads none of these) --
  // ==========================================================================
  vcfg.io.ren_br_tags    := io.ren_br_tags
  vcfg.io.ren_br_vconfig := io.ren_br_vconfig
  vcfg.io.brupdate       := io.brupdate
  vcfg.io.rollback       := io.rollback
  vcfg.io.com_valids     := io.com_valids
  vcfg.io.com_is_vset    := io.com_is_vset
  vcfg.io.com_vtype      := io.com_vtype
  vcfg.io.csr_vtype      := io.csr_vtype
  vcfg.io.rob_empty      := io.rob_empty
  vcfg.io.dec_valids     := io.dec_valids
  vcfg.io.dec_fire       := io.dec_fire

  for (w <- 0 until coreWidth) {

    // ========================================================================
    // ---- ls: feed VLSDecode's per-lane descriptor ----
    // ========================================================================
    ls.io.lanes(w).valid := io.dec_valids(w)
    ls.io.lanes(w).inst  := io.dec_insns(w)
    ls.io.lanes(w).uop   := io.dec_uops_in(w)
    val desc = ls.io.lanes(w).desc

    // ========================================================================
    // ---- 1. Recognition: the three-way opcode split ----
    // ========================================================================
    //@req-spec-decode.a3
    val isArithLane = arith.io.is_arith(w)
    val isMemLane   = desc.is_vls
    val isVsetLane  = vset.io.dec_is_vset(w)
    val rvvRecognized = isArithLane || isMemLane || isVsetLane
    assert(!io.dec_valids(w) || PopCount(Seq(isArithLane, isMemLane, isVsetLane)) <= 1.U,
      "VecDecode: more than one of {arith, ls, vset} recognized the same lane")

    // ========================================================================
    // ---- arith (VDecode): feed vtype_in (seam b) and ls_in (seam a) ----
    // ========================================================================
    arith.io.inst(w)   := io.dec_insns(w)
    arith.io.valid(w)  := io.dec_valids(w)
    arith.io.uop_in(w) := io.dec_uops_in(w)
    arith.io.vtype_in(w) := vcfg.io.dec_vconfig(w)
    arith.io.ls_in(w).is_mem       := desc.is_vls
    arith.io.ls_in(w).is_load      := desc.is_vls && !desc.is_store
    arith.io.ls_in(w).is_store     := desc.is_store
    arith.io.ls_in(w).is_whole_reg := desc.is_whole_reg
    arith.io.ls_in(w).nf           := desc.nf

    // ========================================================================
    // ---- vset (VsetDecode): feed dec_vconfig (seam c) and prev_vtype ----
    // ========================================================================
    vset.io.dec_valid(w) := io.dec_valids(w)
    vset.io.inst(w)      := io.dec_insns(w)
    vset.io.uop_in(w)    := io.dec_uops_in(w)
    vset.io.dec_vconfig(w) := vcfg.io.dec_vconfig(w)
    vset.io.prev_vtype(w) := vcfg.io.dec_prev_vconfig(w)

    // ========================================================================
    // ---- vcfg: feed the vset-shape fields (seam d), dec_uses_vtype ----
    vcfg.io.dec_is_vset(w)      := vset.io.dec_is_vset(w)
    vcfg.io.dec_vtype_imm(w)    := vset.io.dec_vtype_imm(w).pad(xLen)
    vcfg.io.dec_vtype_is_imm(w) := vset.io.dec_vtype_is_imm(w)
    vcfg.io.dec_is_vsetivli(w)  := vset.io.is_vsetivli(w)
    vcfg.io.dec_avl_imm(w)      := vset.io.dec_avl_imm(w)
    vcfg.io.dec_ftq_idx(w) := io.dec_uops_in(w).ftq_idx
    vcfg.io.dec_pc_lob(w)  := io.dec_uops_in(w).pc_lob

    val isWholeRegMove = (io.dec_insns(w)(6, 0) === 0x57.U) &&
                         (io.dec_insns(w)(14, 12) === 3.U) &&    // OPIVI
                         (io.dec_insns(w)(31, 26) === 0x27.U)    // funct6
    vcfg.io.dec_uses_vtype(w) := (isArithLane && !isWholeRegMove) ||
                                 (desc.is_vls && !desc.is_whole_reg && !desc.is_mask)

    // ========================================================================
    // ---- The memory-side EMUL derivation (this module's own, part 5) -------
    // ========================================================================
    val vtypeInfo = VtypeTable.decode(vcfg.io.dec_vconfig(w).asUInt)
    //@req-spec-decode.a3
    // SPEC DEFECT (reported, not resolved): VtypeTable's own VtypeInfo digest drops vsew entirely.
    val emulPerField = Mux(desc.is_whole_reg, desc.nregs,
                       Mux(desc.is_mask,      1.U,
                       Mux(desc.is_indexed,   vtypeInfo.emul,
                                              VtypeTable.emul(vtypeInfo, desc.eew))))
    val segMultiplier = desc.nf +& 1.U
    val emulTotal      = Mux(desc.is_segment, emulPerField * segMultiplier, emulPerField)

    //@req-spec-decode.a3
    val memEmulIllegal = isMemLane && (emulPerField === 0.U || emulTotal > maxVecMembers.U)

    // ========================================================================
    // ---- The merge base, part 2 (single zeroing) and parts 3/4/5 (merge) --
    // ========================================================================
    val uopOut = WireDefault(io.dec_uops_in(w))

    // ---- Part 4: vconfig, THE UNCONDITIONAL WRITE, every lane, no qualifier.
    val vTypeWithKeepVl = WireDefault(vcfg.io.dec_vconfig(w))
    vTypeWithKeepVl.vill := vcfg.io.dec_vconfig(w).vill || vset.io.keep_vl_illegal(w)
    uopOut.vconfig.get := vTypeWithKeepVl

    when (rvvRecognized) {
      // ---- Part 2: THE SINGLE ZEROING, before any child's SET is applied.
      uopOut.iq_type := VecInit(Seq.fill(IQ_SZ)(false.B))
      uopOut.fu_code := VecInit(Seq.fill(FC_SZ)(false.B))
      uopOut.v_uses_vs1.get  := false.B
      uopOut.v_uses_vs2.get  := false.B
      uopOut.v_uses_vs3.get  := false.B
      uopOut.v_is_masked.get := false.B
      uopOut.v_tail_undist.get := false.B
      uopOut.v_mask_undist.get := false.B
      uopOut.v_split_first.get        := false.B
      uopOut.v_split_last.get         := false.B
      uopOut.v_split_idx.get          := 0.U
      uopOut.v_split_total.get        := 0.U
      uopOut.v_split_dst_prn.get      := 0.U
      uopOut.v_split_dst_byte_off.get := 0.U
      uopOut.v_elem_cursor.get.elem_next  := 0.U
      uopOut.v_elem_cursor.get.elem_done  := 0.U
      uopOut.v_elem_cursor.get.fault_elem := 0.U
      uopOut.is_shared.get     := arith.io.uop_out(w).is_shared.get
      uopOut.iq_type(IQ_V_ALU) := arith.io.uop_out(w).iq_type(IQ_V_ALU)

      // ---- Part 3, arithmetic-lane fields, from arith.uop_out(w) ----
      when (isArithLane) {
        uopOut.is_vec.get     := arith.io.uop_out(w).is_vec.get
        uopOut.dst_rtype       := arith.io.uop_out(w).dst_rtype
        uopOut.lvd.get         := arith.io.uop_out(w).lvd.get
        uopOut.lvs1.get        := arith.io.uop_out(w).lvs1.get
        uopOut.lvs2.get        := arith.io.uop_out(w).lvs2.get
        uopOut.lvs3.get        := arith.io.uop_out(w).lvs3.get
        uopOut.lvm.get         := arith.io.uop_out(w).lvm.get
        uopOut.ldst            := arith.io.uop_out(w).ldst
        uopOut.lrs1            := arith.io.uop_out(w).lrs1
        uopOut.lrs1_rtype      := arith.io.uop_out(w).lrs1_rtype
        uopOut.lrs2_rtype      := arith.io.uop_out(w).lrs2_rtype
        uopOut.v_emul.get      := arith.io.uop_out(w).v_emul.get
        uopOut.v_uses_vs1.get  := arith.io.uop_out(w).v_uses_vs1.get
        uopOut.v_uses_vs2.get  := arith.io.uop_out(w).v_uses_vs2.get
        uopOut.v_uses_vs3.get  := arith.io.uop_out(w).v_uses_vs3.get
        uopOut.v_is_masked.get := arith.io.uop_out(w).v_is_masked.get
        uopOut.fu_code(FC_ALU) := true.B
      }

      // ---- Part 3, vset-lane fields, from vset.uop_out(w) ----
      when (isVsetLane) {
        uopOut.dst_rtype          := vset.io.uop_out(w).dst_rtype
        uopOut.is_vl_producer.get := vset.io.uop_out(w).is_vl_producer.get
        uopOut.lrs1               := vset.io.uop_out(w).lrs1
        uopOut.lrs2               := vset.io.uop_out(w).lrs2
        uopOut.lrs1_rtype         := vset.io.uop_out(w).lrs1_rtype
        uopOut.lrs2_rtype         := vset.io.uop_out(w).lrs2_rtype
        uopOut.is_unique          := vset.io.uop_out(w).is_unique
        uopOut.flush_on_commit    := vset.io.uop_out(w).flush_on_commit
        uopOut.iq_type(IQ_ALU)    := vset.io.uop_out(w).iq_type(IQ_ALU)
        uopOut.fu_code(FC_ALU)    := vset.io.uop_out(w).fu_code(FC_ALU)
        uopOut.fcn_op             := vset.io.uop_out(w).fcn_op
        uopOut.fcn_dw             := vset.io.uop_out(w).fcn_dw
        uopOut.op1_sel            := vset.io.uop_out(w).op1_sel
        uopOut.op2_sel            := vset.io.uop_out(w).op2_sel
        uopOut.imm_sel            := vset.io.uop_out(w).imm_sel
        uopOut.imm_rename         := vset.io.uop_out(w).imm_rename
        uopOut.csr_cmd            := vset.io.uop_out(w).csr_cmd
        uopOut.frs3_en            := vset.io.uop_out(w).frs3_en
        uopOut.uses_ldq           := vset.io.uop_out(w).uses_ldq
        uopOut.uses_stq           := vset.io.uop_out(w).uses_stq
        uopOut.v_vl_imm.get       := vcfg.io.dec_vl_imm(w).bits
      }

      // ========================================================================
      // ---- Part 5, memory-lane fields ----
      when (isMemLane) {
        // Class carriage, straight across, one field per descriptor flag.
        uopOut.v_mop.get            := desc.mop
        uopOut.v_is_unit_stride.get := desc.is_unit_stride
        uopOut.v_is_strided.get     := desc.is_strided
        uopOut.v_is_indexed.get     := desc.is_indexed
        uopOut.v_is_segment.get     := desc.is_segment
        uopOut.v_is_whole_reg.get   := desc.is_whole_reg
        uopOut.v_is_mask.get        := desc.is_mask
        uopOut.v_is_ff.get          := desc.is_ff
        uopOut.v_seg_nf.get := Mux(desc.is_segment, desc.nf, 0.U)
        uopOut.v_is_masked.get := !desc.vm && !desc.is_whole_reg && !desc.is_mask
        //@req-spec-lsu.e8
        uopOut.v_tail_undist.get := !vcfg.io.dec_vconfig(w).vta
        uopOut.v_mask_undist.get := !vcfg.io.dec_vconfig(w).vma
        uopOut.v_eew.get     := Mux(desc.eew_is_index, vcfg.io.dec_vconfig(w).vsew(1, 0), desc.eew)
        uopOut.v_idx_eew.get := desc.eew
        uopOut.v_uses_vs1.get := desc.uses_vs1
        uopOut.v_uses_vs2.get := desc.uses_vs2
        uopOut.v_uses_vs3.get := desc.uses_vs3
        uopOut.v_emul.get := emulTotal(log2Ceil(maxVecMembers), 0)
        uopOut.is_vec.get          := true.B
        uopOut.iq_type(IQ_V_LOAD)  := !desc.is_store
        uopOut.iq_type(IQ_V_STORE) := desc.is_store
        uopOut.fu_code(FC_AGEN)    := true.B
        uopOut.fu_code(FC_DGEN)    := desc.is_store
        uopOut.uses_ldq            := !desc.is_store
        uopOut.uses_stq            := desc.is_store
        // The baseline table has no RVV row, so mem_cmd arrives as M_X (all
        // don't-care) and reaches the TLB and D$ as whatever it minimized to.
        uopOut.mem_cmd             := Mux(desc.is_store, M_XWR, M_XRD)
        when (!desc.is_store) {
          uopOut.lvd.get  := io.dec_insns(w)(11, 7) // rd: destination group
          uopOut.dst_rtype := RT_VEC
        } .otherwise {
          uopOut.lvs3.get := io.dec_insns(w)(11, 7)
          uopOut.dst_rtype := RT_X
        }
        val rs1f = io.dec_insns(w)(19, 15)
        uopOut.lrs1       := rs1f
        uopOut.lrs1_rtype := Mux(rs1f === 0.U, RT_ZERO, RT_FIX)
        val rs2f = io.dec_insns(w)(24, 20)
        when (desc.is_strided) {
          uopOut.lrs2       := rs2f
          uopOut.lrs2_rtype := Mux(rs2f === 0.U, RT_ZERO, RT_FIX)
        } .elsewhen (desc.is_indexed) {
          uopOut.lvs2.get   := rs2f
          uopOut.lrs2_rtype := RT_X
        } .otherwise {
          // unit-stride: the field is umop, no register named at all.
          uopOut.lrs2_rtype := RT_X
        }

        uopOut.lvm.get := 0.U
        uopOut.is_vl_producer.get := desc.is_ff
      }
    }

    io.dec_uops_out(w) := uopOut

    // ==========================================================================
    // ---- 6. dec_vec_illegal: the aggregation, exactly four terms ----
    // ==========================================================================
    io.dec_vec_illegal(w) := io.dec_valids(w) && (
      ls.io.lanes(w).illegal ||
      arith.io.vill_trap(w) ||
      vcfg.io.dec_vtype_illegal(w) ||
      memEmulIllegal)

    // ==========================================================================
    // ---- The fifth seam: dec_vl_imm_valid ----
    // ==========================================================================
    io.dec_vl_imm(w)       := vcfg.io.dec_vl_imm(w).bits
    io.dec_vl_imm_valid(w) := vcfg.io.dec_vl_imm(w).valid && vset.io.frontend_only(w)

    // ==========================================================================
    // ---- 8. Assertions (checks, not behaviour: deletable without changing
    //         the emitted datapath) ----
    // ==========================================================================
    assert(!(io.dec_valids(w) && rvvRecognized && !vset.io.frontend_only(w)) ||
      uopOut.iq_type.asUInt =/= 0.U,
      "VecDecode: a recognized, non-frontend-only lane must route to some issue queue")
    assert(!io.dec_valids(w) || !uopOut.is_shared.get ||
      (uopOut.iq_type(IQ_V_ALU) && (uopOut.iq_type(IQ_V_LOAD) || uopOut.iq_type(IQ_V_STORE))),
      "VecDecode: is_shared must carry both IQ_V_ALU and one of IQ_V_LOAD/IQ_V_STORE")
    assert(!(io.dec_valids(w) && uopOut.is_vec.get && !io.dec_vec_illegal(w)) ||
      (uopOut.v_emul.get >= 1.U && uopOut.v_emul.get <= maxVecMembers.U),
      "VecDecode: v_emul out of [1, maxVecMembers] on a legal is_vec lane")
    assert(!(io.dec_valids(w) && rvvRecognized) ||
      (!uopOut.v_split_first.get && !uopOut.v_split_last.get &&
       uopOut.v_split_idx.get === 0.U && uopOut.v_split_total.get === 0.U &&
       uopOut.v_split_dst_prn.get === 0.U && uopOut.v_split_dst_byte_off.get === 0.U &&
       uopOut.v_elem_cursor.get.elem_next === 0.U &&
       uopOut.v_elem_cursor.get.elem_done === 0.U &&
       uopOut.v_elem_cursor.get.fault_elem === 0.U),
      "VecDecode: nOP.v-scoped cursor fields must be zero on every recognized lane")

    // ========================================================================
    // ---- 8 (cont). Trace ----
    // ========================================================================
    when (io.dec_valids(w) && rvvRecognized) {
      VecTrace.traceDecode(
        "VecDecode", "merge", io.dec_uops_in(w).ftq_idx, io.dec_uops_in(w).pc_lob,
        Seq(
          ("decoder",   Mux(isArithLane, 0.U(2.W), Mux(isMemLane, 1.U(2.W), 2.U(2.W)))),
          ("iq_type",   uopOut.iq_type.asUInt),
          ("v_emul",    uopOut.v_emul.get),
          ("v_eew",     uopOut.v_eew.get),
          ("is_shared", uopOut.is_shared.get)))
    }
    when (io.dec_vec_illegal(w)) {
      VecTrace.traceDecode(
        "VecDecode", "illegal", io.dec_uops_in(w).ftq_idx, io.dec_uops_in(w).pc_lob,
        Seq(
          ("mem_illegal",      ls.io.lanes(w).illegal),
          ("arith_illegal",    arith.io.vill_trap(w)),
          ("vtype_illegal",    vcfg.io.dec_vtype_illegal(w)),
          ("mem_emul_illegal", memEmulIllegal)))
    }
  }
}

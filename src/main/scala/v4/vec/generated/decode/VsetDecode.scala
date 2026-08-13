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
import freechips.rocketchip.rocket.{Instructions, VType, CSR, ALU}

import boom.v4.common._
import boom.v4.vec.generated.{VtypeTable, VecTrace}

// GENERATED from src/main/nlhdl/vec/decode/VsetDecode.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

//@req-spec-decode.c13
class VsetDecode(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    // ---- Decode-stage inputs, one entry per lane ----
    val dec_valid    = Input(Vec(coreWidth, Bool()))
    val inst         = Input(Vec(coreWidth, UInt(32.W)))
    val uop_in       = Input(Vec(coreWidth, new MicroOp()))
    val dec_vconfig  = Input(Vec(coreWidth, new VType))
    val prev_vtype   = Input(Vec(coreWidth, new VType))

    // ---- Outputs, one entry per lane ----
    val dec_is_vset      = Output(Vec(coreWidth, Bool()))
    val is_vsetivli      = Output(Vec(coreWidth, Bool()))
    val is_vsetvli       = Output(Vec(coreWidth, Bool()))
    val is_vsetvl        = Output(Vec(coreWidth, Bool()))
    val dec_vtype_is_imm = Output(Vec(coreWidth, Bool()))
    val dec_vtype_imm    = Output(Vec(coreWidth, UInt(11.W)))
    // The 5-bit uimm AVL field of a vsetivli.
    val dec_avl_imm      = Output(Vec(coreWidth, UInt(5.W)))
    val keep_vl_illegal  = Output(Vec(coreWidth, Bool()))
    val frontend_only    = Output(Vec(coreWidth, Bool()))
    val uop_out          = Output(Vec(coreWidth, new MicroOp()))
  })

  for (w <- 0 until coreWidth) {
    val inst = io.inst(w)

    // ---- Part 1: recognition and the three-way split ----
    val isVsetivliRaw = Instructions.VSETIVLI === inst
    val isVsetvliRaw  = Instructions.VSETVLI  === inst
    val isVsetvlRaw   = Instructions.VSETVL   === inst

    val isVsetivli = io.dec_valid(w) && isVsetivliRaw
    val isVsetvli  = io.dec_valid(w) && isVsetvliRaw
    val isVsetvl   = io.dec_valid(w) && isVsetvlRaw
    val decIsVset  = isVsetivli || isVsetvli || isVsetvl

    io.is_vsetivli(w) := isVsetivli
    io.is_vsetvli(w)  := isVsetvli
    io.is_vsetvl(w)   := isVsetvl
    io.dec_is_vset(w) := decIsVset

    val rd  = inst(11, 7)
    val rs1 = inst(19, 15)
    val rs2 = inst(24, 20)

    //@req-spec-decode.c26
    //@req-spec-decode.c5
    io.dec_vtype_imm(w) := Mux(isVsetivli, inst(29, 20), inst(30, 20))
    io.dec_avl_imm(w)   := inst(19, 15)

    io.dec_vtype_is_imm(w) := isVsetivli || isVsetvli

    // ---- Part 2: vsetivli, rd == x0 -- the front-end-only path ----
    //@req-spec-decode.c3
    val frontendOnly = isVsetivli && rd === 0.U
    io.frontend_only(w) := frontendOnly
    //@req-spec-decode.c6

    // ---- Part 5: the VL-producer rd/rs1 split (register-sourced shapes) ----
    val rs1Zero = rs1 === 0.U
    val rdZero  = rd === 0.U

    //@req-spec-decode.i15
    //@req-spec-decode.i14
    //@req-spec-decode.i11
    //@req-spec-decode.i12
    //@req-spec-decode.i16
    val regSourcedIsVlProducer = !(rs1Zero && rdZero)
    val keepVlCase             = rs1Zero && rdZero

    val isVlProducer = Mux(isVsetivli, true.B, regSourcedIsVlProducer)

    //@req-spec-decode.i13
    val vtypeImmForKeepVlCheck = io.dec_vtype_imm(w).pad(xLen)
    val vlmaxNew  = VtypeTable.decode(vtypeImmForKeepVlCheck).vlmax
    val vlmaxPrev = VtypeTable.decode(io.prev_vtype(w).asUInt).vlmax
    val keepVlIllegal = isVsetvli && keepVlCase && (vlmaxNew =/= vlmaxPrev)
    io.keep_vl_illegal(w) := keepVlIllegal

    //@req-spec-decode.c16
    val dstRtype = Mux(rdZero, RT_ZERO, RT_FIX)

    // ---- Assemble uop_out ----
    io.uop_out(w) := io.uop_in(w)

    when (decIsVset) {
      io.uop_out(w).dst_rtype := dstRtype
      io.uop_out(w).is_vl_producer.get := isVlProducer

      //@req-spec-decode.e1
      io.uop_out(w).is_unique       := io.uop_in(w).is_unique       || isVsetvl
      io.uop_out(w).flush_on_commit := io.uop_in(w).flush_on_commit || isVsetvl

      //@req-spec-decode.d8
      when (io.dec_vtype_is_imm(w)) {
        val vc = WireDefault(io.dec_vconfig(w))
        vc.vill := io.dec_vconfig(w).vill || keepVlIllegal
        io.uop_out(w).vconfig.get := vc
      } .otherwise {
        io.uop_out(w).vconfig.get := io.prev_vtype(w)
      }

      when (frontendOnly) {
        //@req-spec-decode.c3
        io.uop_out(w).iq_type := VecInit(Seq.fill(IQ_SZ)(false.B))
        io.uop_out(w).fu_code := VecInit(Seq.fill(FC_SZ)(false.B))
      } .otherwise {
        //@req-spec-decode.c8
        val iqt = VecInit(Seq.fill(IQ_SZ)(false.B)); iqt(IQ_ALU) := true.B
        val fct = VecInit(Seq.fill(FC_SZ)(false.B)); fct(FC_ALU) := true.B
        io.uop_out(w).iq_type := iqt
        io.uop_out(w).fu_code := fct

        //@req-spec-decode.c9
        // SPEC DEFECT (reported, not resolved): wakeup paragraph contradicts logic part 2's statement about vsetivli AVL source. RT_X used instead of RT_FIX/RT_ZERO.
        io.uop_out(w).lrs1_rtype := Mux(isVsetivli, RT_X, Mux(rs1Zero, RT_ZERO, RT_FIX))
        io.uop_out(w).lrs2_rtype := Mux(isVsetvl, Mux(rs2 === 0.U, RT_ZERO, RT_FIX), RT_X)
        io.uop_out(w).frs3_en    := false.B
        // SPEC DEFECT (reported, not resolved): c9 says "lrs3_rtype and frs3_en are cleared" but MicroOp has no lrs3_rtype field.
        io.uop_out(w).fcn_dw  := DW_XPR
        io.uop_out(w).fcn_op  := ALU.FN_ADD
        io.uop_out(w).op1_sel := OP1_RS1
        io.uop_out(w).op2_sel := OP2_ZERO
        io.uop_out(w).imm_rename := false.B
        io.uop_out(w).imm_sel    := IS_N
        io.uop_out(w).csr_cmd    := CSR.N
        io.uop_out(w).uses_ldq   := false.B
        io.uop_out(w).uses_stq   := false.B
      }
    }

    // ---- Part 7: assertions ----
    assert(PopCount(Seq(isVsetivli, isVsetvli, isVsetvl)) <= 1.U,
      "VsetDecode: more than one vset shape predicate asserted for lane")
    assert(!frontendOnly || (io.uop_out(w).iq_type.asUInt === 0.U &&
                             io.uop_out(w).is_vl_producer.get &&
                             isVsetivli),
      "VsetDecode: frontend_only uop must have iq_type all-clear, " +
      "is_vl_producer set, and be a vsetivli")
    assert(!keepVlIllegal || (isVsetvli && rs1Zero && rdZero),
      "VsetDecode: keep_vl_illegal implies is_vsetvli with both register " +
      "fields zero")
    assert(!decIsVset || (io.dec_vtype_is_imm(w) === !isVsetvl),
      "VsetDecode: dec_vtype_is_imm must be false for vsetvl and true for " +
      "the other two shapes")

    // ---- Part 7: guarded tracing ----
    // rob_idx not allocated at decode, use VecTrace decode-stage variant.
    val shapeCode = Mux(isVsetivli, 0.U(2.W), Mux(isVsetvli, 1.U(2.W), 2.U(2.W)))
    when (decIsVset) {
      VecTrace.traceDecode("VsetDecode", "decode",
        io.uop_in(w).ftq_idx, io.uop_in(w).pc_lob,
        Seq(
          ("shape", shapeCode),
          ("vtype_imm", io.dec_vtype_imm(w)),
          ("keep_vl_illegal", keepVlIllegal),
          ("is_vl_producer", isVlProducer),
          ("frontend_only", frontendOnly)
        ))
    }
  }
}

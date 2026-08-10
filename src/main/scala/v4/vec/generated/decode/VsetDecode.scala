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
//
// VsetDecode -- the per-lane decoder that splits the three `vset` encodings
// into the three different pipeline paths they take, and drives the uOP
// fields that commit each one to its path.
//
// THE SPLIT IS THE SUBSTANCE OF THIS FILE:
//
//   vsetivli, rd == x0   FRONT-END ONLY: no issue queue, no EU. VL is computed
//                        at decode (by VConfigUnit, from the operands this
//                        file exports); the VL-RF write happens in the RENAME
//                        cycle. pvl is born ready.
//   vsetivli, rd != x0   NOT front-end only: rd needs an integer-RF write the
//                        front end cannot deliver, so this takes the
//                        register-sourced ALU path with the AVL still read
//                        from the IMMEDIATE field (never from rs1 -- rs1's bit
//                        position holds the uimm, not a register number).
//   vsetvli              VTYPE immediate, AVL from rs1 -> integer ALU EU.
//   vsetvl                VTYPE from rs2 AND AVL from rs1 -> integer ALU EU,
//                        plus is_unique AND flush_on_commit.
//
// ===> `vsetvli rd = x0, rs1 = x0` IS THE RESERVED KEEP-VL FORM, NOT
//      `AVL = 0`. The same reserved form, without the decode-time illegal
//      check (deferred to execute since its vtype is a register value), also
//      governs `vsetvl`. It does NOT exist for `vsetivli`: that shape's rs1
//      bit position is always an immediate AVL, so an all-zero encoding there
//      just means "AVL = 0", never "keep VL".
//
// ===> A REGISTER-SOURCED `vset` HAS TWO DESTINATIONS IN TWO RENAME SPACES:
//      `pdst` in the integer RF and `pvl` in the VL RF, via `is_vl_producer`,
//      which is ORTHOGONAL to `dst_rtype` (RT_ZERO when rd == x0, independent
//      of whether the VL RF is written).
//
// Governing spec anchors: frontend.rst `vector-rvv-decode` ("VSET Special
// Handling"), `vset-dual-dest`, `vl-delivery`. Plan v2 ground rules 1
// (usingRVV), 9 (rocket owns the architectural vector CSRs), 10 (reuse the
// existing wakeup networks), 11 (guarded tracing).
//
// Elaborated only when `usingRVV` is true, and needs no `usingRVV` term of its
// own: it lives under VecPipeline, whose single `core.scala` instantiation is
// already gated, so a vectors-off build contains no instance of this module at
// all (absent, not tied off).
//
//@req-spec-decode.c13
// No parameter here selects a wakeup network and none here could add one. A
// register-sourced `vset` CONSUMES the existing integer network for
// rs1/rs2 (via lrs1_rtype/lrs2_rtype below) and PRODUCES on the VL network for
// pvl (wired by VecRenameSpace/VlRegFile, not this module). No VCFG wakeup
// network exists anywhere in the design and nothing in this file implies one.
class VsetDecode(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    // ---- Decode-stage inputs, one entry per lane ----
    val dec_valid    = Input(Vec(coreWidth, Bool()))
    val inst         = Input(Vec(coreWidth, UInt(32.W)))
    val uop_in       = Input(Vec(coreWidth, new MicroOp()))
    // Self-inclusive selected vtype for this lane (this lane's own new vtype
    // if it is itself a vset), from VConfigUnit.
    val dec_vconfig  = Input(Vec(coreWidth, new VType))
    // Self-exclusive selected vtype -- the vtype in effect BEFORE this lane's
    // own instruction. Read for one thing only: the keep-VL VLMAX comparison.
    val prev_vtype   = Input(Vec(coreWidth, new VType))

    // ---- Outputs, one entry per lane ----
    val dec_is_vset      = Output(Vec(coreWidth, Bool()))
    val is_vsetivli      = Output(Vec(coreWidth, Bool()))
    val is_vsetvli       = Output(Vec(coreWidth, Bool()))
    val is_vsetvl        = Output(Vec(coreWidth, Bool()))
    val dec_vtype_is_imm = Output(Vec(coreWidth, Bool()))
    // RAW encoded vtype bits, not yet legality-checked. Width assumption
    // (spec silent on this port's width): 11 bits, the wider of the two
    // immediate vtype fields (vsetvli's inst(30,20)); vsetivli's 10-bit
    // inst(29,20) zero-extends into it. A consumer feeding this into
    // VType.fromUInt-family functions (as this file itself does for the
    // keep-VL check) must zero-extend it to xLen first, mirroring how rocket-
    // chip's own RocketCore lets Mux/MuxCase auto-widen the same two fields
    // up to its xLen-wide vsetvl (rs2) case before calling VType.fromUInt.
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
    //
    // Bind by name to rocket's own Instructions.VSETIVLI/VSETVLI/VSETVL
    // BitPat patterns (each already encodes OP-V's opcode and funct3=111)
    // rather than re-deriving the recognition rule locally.
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

    // Field positions, deliberately identical to rocket's own extraction
    // (RocketCore's ex_new_vtype/ex_avl) so a disagreement about where the
    // vtype/AVL bits live can never arise between the speculative mirror and
    // the architectural CSR.
    val rd  = inst(11, 7)
    val rs1 = inst(19, 15)
    val rs2 = inst(24, 20)

    //@req-spec-decode.c26
    //@req-spec-decode.c5
    // Both operands of the (front-end-only) VL computation are exported RAW:
    // legality/VL computation is VConfigUnit's job, not decoded a second time
    // here. vsetvli's field is inst(30,20) (11b); vsetivli's is inst(29,20)
    // (10b) -- Chisel widens the narrower operand of the Mux to match.
    io.dec_vtype_imm(w) := Mux(isVsetivli, inst(29, 20), inst(30, 20))
    // dec_avl_imm is only meaningful for a vsetivli lane (is_vsetivli); it
    // occupies the same bit positions as rs1, so a consumer reading it for
    // any other shape would quietly use a register NUMBER as an element
    // COUNT (logic part 7's fifth assertion). There is no hardware signal in
    // this module representing "consumed by the wrong shape" to assert
    // against; this is a contract on the CONSUMER, documented here rather
    // than enforced, since it cannot be checked from the producer side.
    io.dec_avl_imm(w)   := inst(19, 15)

    io.dec_vtype_is_imm(w) := isVsetivli || isVsetvli

    // ---- Part 2: vsetivli, rd == x0 -- the front-end-only path ----
    //@req-spec-decode.c3
    val frontendOnly = isVsetivli && rd === 0.U
    io.frontend_only(w) := frontendOnly
    //@req-spec-decode.c6
    // This uop's ROB entry must be dispatched non-busy (the Rob delta's job,
    // since nothing will ever clear a busy bit for a uop with no writeback).
    // MicroOp.starts_bsy's body is frozen by the MicroOp delta and is not
    // touched here; the encoding this file drives (iq_type all-clear, below)
    // is what the Rob delta keys its non-busy dispatch decision on.

    // ---- Part 5: the VL-producer rd/rs1 split (register-sourced shapes) ----
    // Governs vsetvli AND vsetvl identically (the closing note of logic part
    // 5: "the same rd/rs1 split governs vsetvl"). vsetivli is exempt -- its
    // rs1 bit position is always an immediate AVL, so it has no keep-VL form
    // and is_vl_producer is unconditionally SET for it (section 2's "ONE
    // meaning" rule).
    val rs1Zero = rs1 === 0.U
    val rdZero  = rd === 0.U

    //@req-spec-decode.i15
    // (a) rs1 != x0: VL = min(rs1, VLMAX), computed by the ALU -- a producer.
    // (b) rs1 == x0, rd != x0: VL = VLMAX -- still computed at execute and
    //     written to the VL RF -- still a producer.
    //@req-spec-decode.i14
    // (d) rd == x0 together with rs1 != x0 (a VL change): a VL PRN is STILL
    //     ALLOCATED even though dst_rtype is RT_ZERO -- the VL destination is
    //     independent of the (discarded) integer destination.
    //@req-spec-decode.i11
    //@req-spec-decode.i12
    //@req-spec-decode.i16
    // (c) rs1 == x0, rd == x0: the RESERVED KEEP-VL FORM. NOT a VL producer --
    //     no pvl allocated, the VL map table is left untouched, every younger
    //     uop keeps reading the existing pvl.
    val regSourcedIsVlProducer = !(rs1Zero && rdZero)
    val keepVlCase             = rs1Zero && rdZero

    val isVlProducer = Mux(isVsetivli, true.B, regSourcedIsVlProducer)

    //@req-spec-decode.i13
    // RESERVED-ENCODING CHECK for case (c), vsetvli only (vsetvl's vtype is a
    // register value, unknown at decode, so this check is left to execute for
    // it -- logic part 5's closing note). VLMAX equality IS the SEW/LMUL-
    // ratio test; go through the one shared VtypeTable.decode for both
    // operands rather than rebuilding the ratio, so an already-illegal vtype
    // (vlmax forced to 0 by decode()) cannot manufacture a spurious pass.
    // Implementation note: the nlhdl source's literal formula reads
    // `VtypeTable.decode(prev_vtype)`, but prev_vtype's port type is VType
    // (a Bundle) while decode() takes a UInt -- `.asUInt` bridges the two, and
    // is exactly the round-trip VtypeTable.scala's own comments describe as
    // intentional (re-decoding an already-legal-or-poisoned VType is
    // idempotent).
    val vtypeImmForKeepVlCheck = io.dec_vtype_imm(w).pad(xLen)
    val vlmaxNew  = VtypeTable.decode(vtypeImmForKeepVlCheck).vlmax
    val vlmaxPrev = VtypeTable.decode(io.prev_vtype(w).asUInt).vlmax
    val keepVlIllegal = isVsetvli && keepVlCase && (vlmaxNew =/= vlmaxPrev)
    io.keep_vl_illegal(w) := keepVlIllegal

    //@req-spec-decode.c16
    // Two destinations in two independent rename spaces: dst_rtype (pdst,
    // integer RF) and is_vl_producer (pvl, VL RF) are driven separately and
    // neither is derivable from the other -- with rd == x0, dst_rtype is
    // RT_ZERO while the VL RF may still be written (cases b/d above).
    val dstRtype = Mux(rdZero, RT_ZERO, RT_FIX)

    // ---- Assemble uop_out: default passthrough, then override owned fields
    //      only for a recognized vset lane ----
    io.uop_out(w) := io.uop_in(w)

    when (decIsVset) {
      io.uop_out(w).dst_rtype := dstRtype
      io.uop_out(w).is_vl_producer.get := isVlProducer

      //@req-spec-decode.e1
      // vsetvl ONLY: both bits, decided from the instruction word. Also set
      // by the DecodeUnit delta from the other side of the merge (the SAME
      // signal, not a second mechanism) -- OR'd here rather than overwritten
      // so the two cannot silently disagree by one clearing what the other
      // set.
      io.uop_out(w).is_unique       := io.uop_in(w).is_unique       || isVsetvl
      io.uop_out(w).flush_on_commit := io.uop_in(w).flush_on_commit || isVsetvl

      //@req-spec-decode.d8
      // A vset's OWN vconfig holds its NEW vtype (what the ROB installs into
      // the architectural CSR and the committed shadow at commit), taken from
      // VConfigUnit's self-inclusive dec_vconfig rather than re-decoded here,
      // with keep_vl_illegal ORed into vill so the uop carries its own poison.
      // vsetvl is the documented exception: its vtype is a runtime rs2 value
      // that does not exist at decode, so dec_vtype_is_imm is low for it and
      // its vconfig instead carries the incoming prev_vtype unchanged, as a
      // don't-care (flush_on_commit above guarantees no younger uop can have
      // observed this placeholder).
      when (io.dec_vtype_is_imm(w)) {
        val vc = WireDefault(io.dec_vconfig(w))
        vc.vill := io.dec_vconfig(w).vill || keepVlIllegal
        io.uop_out(w).vconfig.get := vc
      } .otherwise {
        io.uop_out(w).vconfig.get := io.prev_vtype(w)
      }

      when (frontendOnly) {
        //@req-spec-decode.c3
        // Both VTYPE and AVL are immediate: dispatched to no issue queue,
        // reaches no functional unit -- consumes an ROB entry and nothing
        // else. is_vec stays CLEAR (already false from the DecodeUnit
        // delta's own default; not touched here -- a vset is not an OP.v).
        io.uop_out(w).iq_type := VecInit(Seq.fill(IQ_SZ)(false.B))
        io.uop_out(w).fu_code := VecInit(Seq.fill(FC_SZ)(false.B))
        // lrs1_rtype/lrs2_rtype and the immediate/CSR/ldq/stq quiescent
        // fields section 3 drives for the ALU path are deliberately NOT
        // touched here: section 2 enumerates only iq_type, fu_code,
        // dst_rtype, frontend_only and is_vl_producer as this shape's owned
        // fields, and a front-end-only uop never reaches an issue queue or
        // functional unit that could read the untouched ones.
      } .otherwise {
        //@req-spec-decode.c8
        // The scalar integer ALU path: vsetvli, vsetvl, and vsetivli with
        // rd != x0. None of the three IQ_V_* queues is involved; is_vec
        // stays CLEAR (same DecodeUnit default, not touched here).
        val iqt = VecInit(Seq.fill(IQ_SZ)(false.B)); iqt(IQ_ALU) := true.B
        val fct = VecInit(Seq.fill(FC_SZ)(false.B)); fct(FC_ALU) := true.B
        io.uop_out(w).iq_type := iqt
        io.uop_out(w).fu_code := fct

        //@req-spec-decode.c9
        // Wakeup is the existing integer network, unmodified. An x0 source
        // decodes RT_ZERO, not RT_FIX (mirrors the scalar decoder; guards
        // rename-stage.scala's !(lrs1_rtype===RT_FIX && lrs1===0) assert).
        // vsetivli-with-rd is the one shape in this group whose rs1 bit
        // position is NOT a register: its AVL is immediate (logic part 2's
        // callout), so lrs1_rtype is RT_X ("not-a-register") for it rather
        // than RT_FIX/RT_ZERO off rs1's field -- otherwise this uop would
        // wait at rename on a PRN it will never actually consume.
        //
        // ===> SPEC DEFECT (reported, not resolved): logic part 3's own
        // wakeup paragraph (also tagged c9) literally groups "vsetvli,
        // vsetvl (and vsetivli with rd != x0)" together and states
        // "lrs1 = rs1 with lrs1_rtype = RT_FIX" for the group -- which
        // directly contradicts logic part 2's explicit statement that
        // vsetivli-with-rd computes VL "from the IMMEDIATE AVL rather than
        // from rs1" and never reads a register for it. Resolved here in
        // favor of part 2's more specific, independently-justified text
        // (no integer-RF write port fetches an AVL that was never renamed);
        // RT_X is used for vsetivli's lrs1_rtype instead of RT_FIX/RT_ZERO.
        io.uop_out(w).lrs1_rtype := Mux(isVsetivli, RT_X, Mux(rs1Zero, RT_ZERO, RT_FIX))
        // Only vsetvl reads rs2; vsetvli and vsetivli-with-rd have no second
        // source.
        io.uop_out(w).lrs2_rtype := Mux(isVsetvl, Mux(rs2 === 0.U, RT_ZERO, RT_FIX), RT_X)
        io.uop_out(w).frs3_en    := false.B
        // lrs3_rtype: SPEC DEFECT (reported, not resolved) -- the nlhdl
        // source's c9 paragraph says "lrs3_rtype and frs3_en are cleared",
        // but MicroOp (src/main/scala/v4/common/micro-op.scala) declares no
        // `lrs3_rtype` field anywhere, gated or ungated. Only frs3_en exists
        // and is cleared above; lrs3_rtype is omitted rather than invented.

        // The remaining integer control fields, driven to a defined,
        // quiescent selection: the ALU's own result mux is replaced with the
        // computed VL by the (separate) ALUUnit delta, so the ALU's actual
        // arithmetic result here is never the real answer.
        io.uop_out(w).fcn_dw  := DW_XPR
        io.uop_out(w).fcn_op  := ALU.FN_ADD
        io.uop_out(w).op1_sel := OP1_RS1
        io.uop_out(w).op2_sel := OP2_ZERO
        // A vset consumes NO immediate rename resource -- AVL/vtype are read
        // at execute straight off uop.inst.
        io.uop_out(w).imm_rename := false.B
        io.uop_out(w).imm_sel    := IS_N
        // Not a CSR access -- the architectural vtype/vl update happens at
        // COMMIT through the ROB's path to rocket's CSRFile (ground rule 9).
        io.uop_out(w).csr_cmd    := CSR.N
        io.uop_out(w).uses_ldq   := false.B
        io.uop_out(w).uses_stq   := false.B
        // exception/exc_cause are left as the baseline (DecodeUnit) produced
        // them, per the nlhdl source -- not touched here.
      }
    }

    // ---- Part 7: assertions ----
    // At most one shape predicate per lane (structurally true by
    // construction -- the three BitPat patterns are mutually exclusive on
    // bit 31/30 -- kept as a static sanity check).
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

    // ---- Part 7: guarded tracing (ground rule 11) ----
    // rob_idx does not exist at decode (allocated at dispatch), so this uses
    // VecTrace's decode-stage variant, keyed on ftq_idx/pc_lob instead.
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

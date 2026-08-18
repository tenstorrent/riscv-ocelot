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

package boom.v4.vec.generated.cii

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.vec.generated.{CiiIssueReq, IntWbSnoop, VecTrace}

// GENERATED from src/main/nlhdl/vec/cii/VecCiiIssue.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecCiiIssueIO(val numCiiTags: Int)(implicit p: Parameters) extends BoomBundle
{
  val iss             = Input(Valid(new MicroOp))

  val fu_types        = Output(UInt(FC_SZ.W))

  val iss_pkt         = Output(Valid(new CiiIssueReq))
  val iss_credit      = Input(Bool())

  val tag_free_mask   = Input(UInt(numCiiTags.W))
  val tag_alloc       = Output(Valid(new VecCiiTagEntry))

  val alloc_br_mask          = Output(UInt(maxBrCount.W))
  val alloc_flush_on_commit  = Output(Bool())

  val vl_read_addr    = Output(UInt(vlPregSz.W))
  val vl_read_data    = Input(UInt(vecVLSz.W))

  val int_scalar_read_req = Output(UInt(maxPregSz.W))
  val int_scalar_read_rsp = Input(UInt(xLen.W))
  val fp_scalar_read_req  = Output(UInt(maxPregSz.W))
  val fp_scalar_read_rsp  = Input(UInt(xLen.W))
  val int_wb_snoop        = Input(Vec(numIrfWritePorts, Valid(new IntWbSnoop)))

  val csr_vstart = Input(UInt(8.W))
  val csr_vxrm   = Input(UInt(2.W))
  val csr_frm    = Input(UInt(3.W))

  val rob_flush  = Input(Bool())
}

class VecCiiIssue(
  val numCiiTags:      Int = TtCiiCaracalPkg.CII_N_TAGS,
  val ciiIssueCredits: Int = TtCiiCaracalPkg.CII_N_ISS_CREDITS,
  val numIssueLanes:   Int = TtCiiCaracalPkg.CII_NUM_INST_ISSUE)
  (implicit p: Parameters) extends BoomModule
{
  require(usingRVV, "VecCiiIssue: elaborated only under usingRVV (never rocket's usingVector)")
  require(numCiiTags == 1 << ciiTagBits,
    s"VecCiiIssue: numCiiTags ($numCiiTags) must equal 1 << ciiTagBits (${1 << ciiTagBits})")
  require(numIssueLanes == 1,
    s"VecCiiIssue: numIssueLanes ($numIssueLanes) must be 1 -- the Issue channel carries one beat per cycle")
  require(vectorParams.vecIssueGrantWidth == 1,
    s"VecCiiIssue: vectorParams.vecIssueGrantWidth (${vectorParams.vecIssueGrantWidth}) must be 1")

  val io = IO(new VecCiiIssueIO(numCiiTags))

  val s1_valid = RegInit(false.B)
  val s1_uop   = Reg(new MicroOp)
  val s1_tag   = Reg(UInt(ciiTagBits.W))
  val s1_vl    = Reg(UInt(vecVLSz.W))

  io.vl_read_addr := io.iss.bits.pvl_src.get

  val grantIsFpScalar = io.iss.bits.lrs1_rtype === RT_FLT
  io.int_scalar_read_req := Mux(grantIsFpScalar, 0.U, io.iss.bits.prs1)
  io.fp_scalar_read_req  := Mux(grantIsFpScalar, io.iss.bits.prs1, 0.U)

  //@req-spec-cii.c11
  val iss_credits = RegInit(ciiIssueCredits.U(log2Ceil(ciiIssueCredits + 1).W))

  //@req-spec-cii.d6
  val s1_pending_mask = Mux(s1_valid, UIntToOH(s1_tag, numCiiTags), 0.U(numCiiTags.W))
  val tag_avail       = io.tag_free_mask & ~s1_pending_mask
  val chosen_tag      = PriorityEncoder(tag_avail)

  //@req-spec-cii.c14
  val iss_credits_next = Mux(io.iss.valid, iss_credits + io.iss_credit.asUInt - 1.U,
                                            iss_credits + io.iss_credit.asUInt)
  val grantMask     = Mux(io.iss.valid, UIntToOH(chosen_tag, numCiiTags), 0.U(numCiiTags.W))
  val tag_avail_next = tag_avail & ~grantMask

  iss_credits := iss_credits_next

  //@req-spec-cii.c10
  //@req-spec-cii.c12
  //@req-spec-cii.c13
  val advertise = RegInit(false.B)
  advertise := (iss_credits_next =/= 0.U) && (tag_avail_next =/= 0.U)

  io.fu_types := Mux(advertise, UIntToOH(FC_ALU.U, FC_SZ), 0.U(FC_SZ.W))

  assert(!(io.iss.valid && iss_credits === 0.U),
    "VecCiiIssue: grant accepted with zero issue credits")
  assert(!(io.iss.valid && tag_avail === 0.U),
    "VecCiiIssue: grant accepted with no tag available")
  assert(iss_credits <= ciiIssueCredits.U,
    "VecCiiIssue: iss_credits exceeds ciiIssueCredits -- spurious or doubled credit return")

  s1_valid := io.iss.valid
  s1_uop   := io.iss.bits
  s1_tag   := chosen_tag
  s1_vl    := io.vl_read_data

  //@req-spec-cii.d7
  //@req-spec-cii.k1
  //@req-spec-cii.k10
  io.iss_pkt.valid := s1_valid
  io.iss_pkt.bits.tag := s1_tag

  //@req-spec-cii.k2
  io.iss_pkt.bits.instr := s1_uop.debug_inst
  assert(!s1_valid || !s1_uop.is_vec.get || s1_uop.debug_inst === s1_uop.inst,
    "VecCiiIssue: debug_inst and inst diverge on a is_vec uop")

  //@req-spec-cii.k3
  val s1_vlmul = Cat(s1_uop.vconfig.get.vlmul_sign, s1_uop.vconfig.get.vlmul_mag)
  io.iss_pkt.bits.vtype := Cat(s1_uop.vconfig.get.vsew, s1_vlmul, s1_uop.vconfig.get.vta, s1_uop.vconfig.get.vma)
  assert(!s1_valid || !s1_uop.vconfig.get.vill,
    "VecCiiIssue: a vill uop reached the Issue packet -- it must have trapped at decode")

  //@req-spec-cii.k4
  //@req-spec-cii.f27
  io.iss_pkt.bits.vl := s1_vl

  //@req-spec-cii.k5
  //@req-spec-decode.d7
  //@req-spec-cii.k6
  //@req-spec-cii.k7
  io.iss_pkt.bits.vstart := io.csr_vstart.pad(vecVLSz)

  //@req-spec-cii.k8
  //@req-spec-cii.k9
  io.iss_pkt.bits.vxrm := io.csr_vxrm
  io.iss_pkt.bits.frm  := io.csr_frm

  //@req-spec-cii.k11
  //@req-spec-cii.d9
  //@req-spec-cii.d10
  io.iss_pkt.bits.src_reuse_hint := 0.U

  //@req-spec-cii.f40
  io.tag_alloc.valid           := s1_valid
  io.tag_alloc.bits.tag        := s1_tag
  io.tag_alloc.bits.rob_idx    := s1_uop.rob_idx
  io.tag_alloc.bits.is_shared  := s1_uop.is_shared.get

  val cop_writes_pvtmp = s1_uop.is_shared.get && s1_uop.uses_stq
  io.tag_alloc.bits.pvdest_grp := Mux(cop_writes_pvtmp, s1_uop.pvtmp.get, s1_uop.pvdest.get)

  val prefixMask = ((1.U((maxVecMembers + 1).W) << s1_uop.v_emul.get) - 1.U)(maxVecMembers - 1, 0)
  io.tag_alloc.bits.pvdest_grp_mask := prefixMask

  io.tag_alloc.bits.pvs1_grp         := s1_uop.pvs1.get
  io.tag_alloc.bits.pvs2_grp         := s1_uop.pvs2.get
  io.tag_alloc.bits.pvs3_grp         := s1_uop.pvs3.get
  io.tag_alloc.bits.pvm              := s1_uop.pvm.get
  //@req-spec-cii.f11
  // The PREDICATES, not just the values. Copying the five PRN groups without
  // these left the tag table unable to say which of them exist.
  io.tag_alloc.bits.uses_vs1         := s1_uop.v_uses_vs1.get
  io.tag_alloc.bits.uses_vs2         := s1_uop.v_uses_vs2.get
  io.tag_alloc.bits.uses_vs3         := s1_uop.v_uses_vs3.get
  io.tag_alloc.bits.stale_pvdest_grp := s1_uop.stale_pvdest.get
  io.tag_alloc.bits.pdst             := s1_uop.pdst

  // SEW-derived, NOT s1_uop.v_eew: VecDecode assigns v_eew only on the memory
  // lane, so an arith uop reaching the CII carries its decode default. For an
  // arith op the element width simply IS SEW, which is what the FP recode on
  // the vfmv.f.s writeback needs.
  io.tag_alloc.bits.v_eew            := s1_uop.vconfig.get.vsew(1, 0)

  //@req-spec-cii.f24
  //@req-spec-cii.f41
  val hits = VecInit(io.int_wb_snoop.map(w => w.valid && w.bits.addr === s1_uop.prs1))
  assert(PopCount(hits) <= 1.U,
    "VecCiiIssue: multiple int_wb_snoop ports hit the same stage-registered scalar PRN")
  val intScalarFwd = Mux(hits.asUInt.orR, Mux1H(hits, io.int_wb_snoop.map(_.bits.data)), io.int_scalar_read_rsp)
  val s1IsFpScalar = s1_uop.lrs1_rtype === RT_FLT
  //@req-spec-cii.f24
  // x0 IS NOT A REGISTER READ. Rename maps `x0` to p0 and NOTHING EVER WRITES
  // p0, so the integer register file returns that entry's leftover contents --
  // BOOM's scalar register-read stage forces the zero itself for exactly this
  // reason, and this path is that stage for the CII. Without it `vmv.s.x vd, x0`
  // -- the idiomatic way to clear an accumulator, emitted by every reduction
  // kernel -- writes garbage into element 0: measured on conv1d-vector, DUT
  // 0x0000000080002a88 against an architectural 0.
  // Keyed on the register TYPE, which VecDecode/VDecode already resolve to
  // RT_ZERO when the rs1 field is 0, and not on `prs1 === 0`: p0 is a legal
  // physical register number for a real source only if the free list could hand
  // it out, and reading the type is what the scalar path does.
  val s1IsZeroScalar = s1_uop.lrs1_rtype === RT_ZERO
  io.tag_alloc.bits.scalar_operands := Mux(s1IsFpScalar, io.fp_scalar_read_rsp,
    Mux(s1IsZeroScalar, 0.U, intScalarFwd))

  io.alloc_br_mask         := s1_uop.br_mask
  io.alloc_flush_on_commit := s1_uop.flush_on_commit

  //@req-spec-cii.d11
  assert(!io.iss.valid || (io.iss.bits.is_vec.get && io.iss.bits.iq_type(IQ_V_ALU)),
    "VecCiiIssue: granted uop is missing is_vec or iq_type(IQ_V_ALU)")

  when (io.iss_pkt.valid) {
    VecTrace.traceTag("VecCiiIssue", "issue", s1_uop, s1_tag, Seq(
      ("vl",      s1_vl),
      ("vtype",   io.iss_pkt.bits.vtype),
      ("vstart",  io.iss_pkt.bits.vstart),
      ("credits", iss_credits)))
  }

  when (iss_credits === 0.U) {
    VecTrace.traceStruct("VecCiiIssue", "credits_exhausted", Seq(("iss_credits", iss_credits)))
  }
}

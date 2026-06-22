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
// Step 2 scope: decode emits a SINGLE uop per architectural instruction (no
// cracker yet) and fills only *static* fields. vtype-dependent fields
// (v_emul, the vconfig snapshot, the final VL) are filled by the cracker
// (Step 3) which owns the architectural vtype/vl mirror. Vector arithmetic is
// decoded and routed to IQ_V_ALU but is tied off (never issues) until Goal 2.

package boom.v4.vec.decode

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

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

  /**
   * Populate the vector-related MicroOp fields in place. Call only when
   * isLegal(inst) holds; overrides the scalar-decode assignments via Chisel
   * last-connect-wins. Leaves vtype-dependent fields (v_emul, vconfig) at their
   * inert decode values -- the cracker fills them in Step 3.
   */
  def decode(uop: MicroOp, inst: UInt)(implicit p: Parameters): Unit = {
    val ls       = VLSDecode(inst)
    val vset     = VsetDecode(inst)
    val is_arith = isVecArith(inst)
    val is_mem   = VLSDecode.isVecMem(inst)

    val rd  = inst(11, 7)
    val rs1 = inst(19, 15)
    val rs2 = inst(24, 20)

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
      // vsetvli/vsetivli). vsetvl gets vtype from rs2 at execute (Step 3/6).
      uop.vconfig.vsew  := vset.vsew
      uop.vconfig.vlmul := vset.vlmul
      uop.vconfig.vta   := vset.vta
      uop.vconfig.vma   := vset.vma
    } .otherwise {
      // Vector data op (load / store / arith).
      uop.is_vec := true.B

      // Route to the correct vector issue queue.
      uop.iq_type(IQ_V_LOAD)  := ls.is_load
      uop.iq_type(IQ_V_STORE) := ls.is_store
      uop.iq_type(IQ_V_ALU)   := is_arith

      // Static load/store fields (cracker fills v_emul / vconfig later).
      uop.v_eew      := ls.v_eew
      uop.v_seg_nf   := ls.nf

      // Vector mask: lvm = v0 (logical 0); masked when the vm bit is clear.
      uop.lvm := 0.U

      when (is_mem) {
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

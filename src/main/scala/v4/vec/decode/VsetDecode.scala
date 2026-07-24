//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal vset-class Decode (vsetvli / vsetivli / vsetvl)
//------------------------------------------------------------------------------
//
// Recognizes the three vset-class instructions and classifies them, extracting
// the immediate vtype (vsetvli/vsetivli), the immediate AVL (vsetivli), and the
// rd==x0 / rs1==x0 sub-cases of vsetvli (RVV spec 6.1).
//
// vset-class instructions are *scalar* integer uops -- they read rs1 (AVL) and
// (vsetvl) rs2 (vtype), and write rd (= new VL). They are NOT is_vec uops routed
// to a vector queue. Their job, beyond writing rd, is to update vtype/vl: the
// VConfigUnit (VCFG) speculative vtype mirror and architectural CSR (Step 6)
// consume this info, and the VCFG handles delivery to younger vector uops. The
// vsetvl serialization (is_unique) lets the VCFG mirror update land safely.
// Step 2 only does decode-stage recognition + field extraction.

package boom.v4.vec.decode

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import freechips.rocketchip.rocket.Instructions._

import boom.v4.common._

/** Decoded vset-class info. */
class VsetInfo(implicit p: Parameters) extends BoomBundle
{
  val is_vsetvli  = Bool()
  val is_vsetivli = Bool()
  val is_vsetvl   = Bool()
  val is_vset     = Bool()        // any vset-class
  val rd_is_x0    = Bool()
  val rs1_is_x0   = Bool()
  val avl_imm     = UInt(5.W)     // zimm[4:0] AVL (vsetivli only)
  val vtype_imm   = UInt(11.W)    // vtype immediate (vsetvli: inst[30:20], vsetivli: inst[29:20])
  // vtype fields decoded from vtype_imm (valid only when vtype is immediate).
  val vsew        = UInt(3.W)
  val vlmul       = UInt(3.W)
  val vta         = Bool()
  val vma         = Bool()
}

object VsetDecode
{
  /** Recognize any vset-class instruction. */
  def isVset(inst: UInt): Bool = (inst === VSETVLI) || (inst === VSETIVLI) || (inst === VSETVL)

  def apply(inst: UInt)(implicit p: Parameters): VsetInfo = {
    val info = Wire(new VsetInfo)

    val is_vsetivli = inst === VSETIVLI
    val is_vsetvl   = inst === VSETVL
    val is_vsetvli  = (inst === VSETVLI) && !is_vsetivli && !is_vsetvl

    info.is_vsetvli  := is_vsetvli
    info.is_vsetivli := is_vsetivli
    info.is_vsetvl   := is_vsetvl
    info.is_vset     := is_vsetvli || is_vsetivli || is_vsetvl

    info.rd_is_x0  := inst(11, 7) === 0.U
    info.rs1_is_x0 := inst(19, 15) === 0.U
    info.avl_imm   := inst(19, 15)   // zimm[4:0] for vsetivli

    // vtype immediate: vsetvli uses inst[30:20] (11 bits), vsetivli uses
    // inst[29:20] (10 bits, zero-extended). vsetvl reads vtype from rs2 (dynamic).
    info.vtype_imm := Mux(is_vsetivli, Cat(0.U(1.W), inst(29, 20)), inst(30, 20))

    // vtype field layout: [2:0]=vlmul, [5:3]=vsew, [6]=vta, [7]=vma.
    info.vlmul := info.vtype_imm(2, 0)
    info.vsew  := info.vtype_imm(5, 3)
    info.vta   := info.vtype_imm(6)
    info.vma   := info.vtype_imm(7)

    // Resulting VL statically known at decode:
    // VL is always renamed into the VL register file and read via pvl at execute;
    // there is no decode-time "VL known" fast path. (vsetivli/vsetvli rs1==x0 still
    // compute VL from the immediate/VLMAX, but the producer writes it to the VL RF.)
    info
  }
}

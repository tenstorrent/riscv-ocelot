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

import boom.v4.common.{BoomBundle, BoomModule, MicroOp}
import boom.v4.vec.generated.VecTrace

// GENERATED from src/main/nlhdl/vec/decode/VLSDecode.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VLSAccessDesc(implicit p: Parameters) extends BoomBundle
{
  val is_vls         = Bool()
  val is_store       = Bool()
  val mop            = UInt(2.W)
  val vm             = Bool()
  val eew            = UInt(2.W)
  val eew_is_index   = Bool()
  val nf             = UInt(3.W)
  val is_unit_stride = Bool()
  val is_strided     = Bool()
  val is_indexed     = Bool()
  val is_segment     = Bool()
  val is_whole_reg   = Bool()
  val is_mask        = Bool()
  val is_ff          = Bool()
  val nregs          = UInt(4.W)
  val uses_vs1       = Bool()
  val uses_vs2       = Bool()
  val uses_vs3       = Bool()
}

class VLSDecodeLaneIO(implicit p: Parameters) extends BoomBundle
{
  val valid   = Input(Bool())
  val inst    = Input(UInt(32.W))
  val uop     = Input(new MicroOp())
  val desc    = Output(new VLSAccessDesc)
  val illegal = Output(Bool())
}

class VLSDecodeIO(implicit p: Parameters) extends BoomBundle
{
  val lanes = Vec(coreWidth, new VLSDecodeLaneIO)
}

class VLSDecode(implicit p: Parameters) extends BoomModule
{
  val io = IO(new VLSDecodeIO)

  for (i <- 0 until coreWidth) {
    val lane = io.lanes(i)
    val inst = lane.inst

    // ---- 1. Is this even a vector load/store ----
    val opcode       = inst(6, 0)
    val width        = inst(14, 12)
    val is_load_fp   = opcode === "b0000111".U
    val is_store_fp  = opcode === "b0100111".U
    val is_vec_width = (width === "b000".U || width === "b101".U ||
                        width === "b110".U || width === "b111".U)
    val is_vls = (is_load_fp || is_store_fp) && is_vec_width
    // Wrong in the permissive direction, every fld acquires a vector access
    // descriptor; wrong in the restrictive direction, vector loads decode as
    // scalar FP. Both fail silently at decode and surface far away.

    val is_store = is_store_fp

    // ---- 2. Field extraction: fixed bits, no arithmetic ----
    val nf   = inst(31, 29)
    val mew  = inst(28)
    val mop  = inst(27, 26)
    val vm   = inst(25)
    val umop = inst(24, 20) // lumop / sumop
    val eew = inst(13, 12)

    // ---- 3. ACCESS CLASS: one-hot, exhaustive, from `mop` alone ----
    //@req-spec-agen.c26
    val is_unit_stride = mop === 0.U
    val is_indexed     = mop === 1.U || mop === 3.U // unordered / ordered
    val is_strided     = mop === 2.U

    //@req-spec-agen.c5

    //@req-spec-agen.c13
    //@req-spec-agen.c22

    //@req-spec-agen.c21

    //@req-spec-agen.c25
    assert(!(lane.valid && is_vls) ||
      PopCount(Seq(is_unit_stride, is_strided, is_indexed)) === 1.U,
      "VLSDecode: access class flags must be exactly one-hot")

    // ---- 5. Unit-stride sub-forms: the `umop` decode ----
    val is_ordinary_us = umop === "b00000".U
    val is_whole_reg   = is_unit_stride && umop === "b01000".U

    val nregs = nf +& 1.U

    val is_mask = is_unit_stride && umop === "b01011".U

    //@req-spec-lsu.g5
    //@req-spec-lsu.g11
    val is_ff = is_unit_stride && umop === "b10000".U

    val is_segment = nf =/= 0.U && !is_whole_reg && !is_mask && !is_ff

    // ---- 7. Indexed accesses: `eew` names the INDEX, not the data ----
    val eew_is_index = is_indexed

    // ---- 7b. `uses_vs*` — which vector sources a MEMORY form encodes ----
    val uses_vs1 = false.B
    val uses_vs2 = is_indexed
    val uses_vs3 = is_store

    // ---- 8. Reserved encodings ----
    val illegal = is_vls && (
      mew ||
      (is_unit_stride && !(is_ordinary_us || is_whole_reg || is_mask || is_ff)) ||
      (is_whole_reg && !(nregs === 1.U || nregs === 2.U || nregs === 4.U || nregs === 8.U)) ||
      (is_mask && (nf =/= 0.U || eew =/= 0.U)) ||
      (is_ff && is_store) ||
      ((is_whole_reg || is_mask) && !vm)
    )

    // ---- 0/4/6. Descriptor assembly ----
    lane.desc.is_vls          := is_vls
    lane.desc.is_store        := is_store
    lane.desc.mop             := mop
    lane.desc.vm              := vm
    lane.desc.eew             := eew
    lane.desc.eew_is_index    := eew_is_index
    lane.desc.nf              := nf
    lane.desc.is_unit_stride  := is_unit_stride
    lane.desc.is_strided      := is_strided
    lane.desc.is_indexed      := is_indexed
    lane.desc.is_segment      := is_segment
    lane.desc.is_whole_reg    := is_whole_reg
    lane.desc.is_mask         := is_mask
    lane.desc.is_ff           := is_ff
    lane.desc.nregs           := nregs
    lane.desc.uses_vs1        := uses_vs1
    lane.desc.uses_vs2        := uses_vs2
    lane.desc.uses_vs3        := uses_vs3

    lane.illegal := illegal

    // ---- 9. Trace ----
    when (lane.valid && is_vls) {
      VecTrace.traceDecode("VLSDecode", "desc", lane.uop.ftq_idx, lane.uop.pc_lob, Seq(
        ("mop",      mop),
        ("eew",      eew),
        ("nf",       nf),
        ("us",       is_unit_stride),
        ("strided",  is_strided),
        ("indexed",  is_indexed),
        ("uses_vs2", uses_vs2),
        ("uses_vs3", uses_vs3)
      ))
    }
  }
}

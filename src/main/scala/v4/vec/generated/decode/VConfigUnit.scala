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

import boom.v4.common.{BoomBundle, BoomModule}
import boom.v4.exu.BrUpdateInfo
import boom.v4.vec.generated.{VtypeTable, VecTrace}

// GENERATED from src/main/nlhdl/vec/decode/VConfigUnit.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
//
// SPEC DEFECT (reported, not resolved): rollback restore trace omitted.
// SPEC DEFECT (reported, not resolved): VtypeTable.decode cannot carry vsew/vlmul_sign/vlmul_mag.

class VcfgBits(implicit p: Parameters) extends BoomBundle
{
  val vill        = Bool()
  val vma         = Bool()
  val vta         = Bool()
  val vsew        = UInt(3.W)
  val vlmul_sign  = Bool()
  val vlmul_mag   = UInt(2.W)
}

class VConfigUnitIO(implicit p: Parameters) extends BoomBundle
{
  // ---- Decode-stage inputs, one entry per lane ----
  val dec_valids        = Input(Vec(coreWidth, Bool()))
  val dec_fire          = Input(Vec(coreWidth, Bool()))
  val dec_is_vset        = Input(Vec(coreWidth, Bool()))
  val dec_vtype_imm     = Input(Vec(coreWidth, UInt(xLen.W)))
  val dec_vtype_is_imm  = Input(Vec(coreWidth, Bool()))
  val dec_is_vsetivli   = Input(Vec(coreWidth, Bool()))
  val dec_avl_imm       = Input(Vec(coreWidth, UInt(5.W)))
  val dec_uses_vtype    = Input(Vec(coreWidth, Bool()))
  val dec_ftq_idx       = Input(Vec(coreWidth, UInt(log2Ceil(ftqSz).W)))
  val dec_pc_lob        = Input(Vec(coreWidth, UInt(log2Ceil(icBlockBytes).W)))

  // ---- Decode-stage outputs, one entry per lane ----
  val dec_vconfig        = Output(Vec(coreWidth, new VType))
  val dec_prev_vconfig   = Output(Vec(coreWidth, new VType))
  val dec_vl_imm          = Output(Vec(coreWidth, Valid(UInt(vecVLSz.W))))
  val dec_vtype_illegal   = Output(Vec(coreWidth, Bool()))

  // ---- Rename-stage inputs: the per-br_tag snapshot ----
  val ren_br_tags     = Input(Vec(coreWidth + 1, Valid(UInt(brTagSz.W))))
  val ren_br_vconfig  = Input(Vec(coreWidth + 1, new VType))

  // ---- Recovery inputs ----
  val brupdate  = Input(new BrUpdateInfo)
  val rollback  = Input(Bool())

  // ---- Commit inputs: the only writer of the committed shadow ----
  val com_valids   = Input(Vec(coreWidth, Bool()))
  val com_is_vset  = Input(Vec(coreWidth, Bool()))
  val com_vtype    = Input(Vec(coreWidth, new VType))

  // ---- Check-only inputs: consumed ONLY by the section-7 assertion ----
  val csr_vtype  = Input(new VType)
  val rob_empty  = Input(Bool())
}

class VConfigUnit(implicit p: Parameters) extends BoomModule
{
  val io = IO(new VConfigUnitIO)

  // ---- 9-bit VType <-> VcfgBits helpers ----
  private def compress(v: VType): VcfgBits = {
    val b = Wire(new VcfgBits)
    b.vill       := v.vill
    b.vma        := v.vma
    b.vta        := v.vta
    b.vsew       := v.vsew
    b.vlmul_sign := v.vlmul_sign
    b.vlmul_mag  := v.vlmul_mag
    b
  }
  private def expand(b: VcfgBits): VType = {
    val v = Wire(new VType)
    v.vill       := b.vill
    v.reserved   := 0.U
    v.vma        := b.vma
    v.vta        := b.vta
    v.vsew       := b.vsew
    v.vlmul_sign := b.vlmul_sign
    v.vlmul_mag  := b.vlmul_mag
    v
  }
  private def poisoned(): VcfgBits = {
    val b = WireInit(0.U.asTypeOf(new VcfgBits))
    b.vill := true.B
    b
  }

  // =========================================================================
  // ---- 1. What this unit is, and what it is deliberately not ----
  // =========================================================================
  //@req-spec-decode.d2
  //@req-spec-decode.d3
  //@req-spec-core.b8
  //@req-spec-decode.g6
  //@req-spec-decode.g7
  //@req-spec-core.d1
  //@req-spec-rename.h6
  //@req-spec-vrf.c5
  //@req-spec-vrf.c8
  //@req-spec-vrf.c6
  //@req-spec-decode.d4
  //@req-spec-decode.d5
  //@req-spec-decode.d13

  // =========================================================================
  // ---- 2. State: three elements, and only three ----
  // =========================================================================
  //@req-spec-decode.h3
  val vcfg_mirror = RegInit(poisoned())
  //@req-spec-decode.h4
  //@req-spec-decode.f4
  val vcfg_shadow = RegInit(poisoned())
  //@req-spec-decode.h6
  //@req-spec-decode.h14
  val vcfg_snapshots = RegInit(VecInit(Seq.fill(maxBrCount)(poisoned())))

  // =========================================================================
  // ---- 3. The per-lane select: a prefix, not a broadcast ----
  // =========================================================================

  val decodedImm: Seq[VcfgBits] = (0 until coreWidth).map { w =>
    compress(VType.fromUInt(io.dec_vtype_imm(w)))
  }

  val laneUpdates: Seq[Bool] = (0 until coreWidth).map { w =>
    io.dec_valids(w) && io.dec_is_vset(w) && io.dec_vtype_is_imm(w)
  }
  val laneUpdatesFire: Seq[Bool] = (0 until coreWidth).map { w =>
    io.dec_fire(w) && io.dec_is_vset(w) && io.dec_vtype_is_imm(w)
  }

  //@req-spec-decode.h1
  //@req-spec-decode.d11
  val running: Seq[VcfgBits] =
    (laneUpdates zip decodedImm).scanLeft(vcfg_mirror) { case (prev, (doUpdate, newVal)) =>
      Mux(doUpdate, newVal, prev)
    }
  val runningFire: Seq[VcfgBits] =
    (laneUpdatesFire zip decodedImm).scanLeft(vcfg_mirror) { case (prev, (doUpdate, newVal)) =>
      Mux(doUpdate, newVal, prev)
    }
  //@req-spec-decode.d9
  for (w <- 0 until coreWidth) {
    io.dec_vconfig(w)      := expand(running(w + 1))
    io.dec_prev_vconfig(w) := expand(running(w))
  }

  // =========================================================================
  // ---- 4. Updating the mirror ----
  // =========================================================================
  //@req-spec-decode.d12
  val vcfg_mirror_decode_update: VcfgBits = runningFire(coreWidth)

  val mirror_update_fires: Bool = laneUpdatesFire.reduce(_ || _)
  val mirror_update_ftq = WireInit(io.dec_ftq_idx(0))
  val mirror_update_pc  = WireInit(io.dec_pc_lob(0))
  for (w <- 0 until coreWidth) {
    when (laneUpdatesFire(w)) {
      mirror_update_ftq := io.dec_ftq_idx(w)
      mirror_update_pc  := io.dec_pc_lob(w)
    }
  }

  //@req-spec-decode.c1
  for (w <- 0 until coreWidth) {
    val vl = VtypeTable.computeVL(
      avl          = io.dec_avl_imm(w),
      bits         = io.dec_vtype_imm(w),
      currentVL    = 0.U,
      useCurrentVL = false.B,
      useMax       = false.B,
      useZero      = false.B)
    io.dec_vl_imm(w).valid := io.dec_fire(w) && io.dec_is_vsetivli(w)
    io.dec_vl_imm(w).bits  := vl
  }

  // =========================================================================
  // ---- 5. Poison ----
  // =========================================================================
  //@req-spec-decode.g8
  //@req-spec-decode.g10
  //@req-spec-decode.h15
  //@req-spec-decode.g9
  //@req-spec-decode.g12
  //@req-spec-decode.g13
  for (w <- 0 until coreWidth) {
    io.dec_vtype_illegal(w) := running(w + 1).vill && io.dec_uses_vtype(w)
  }

  // =========================================================================
  // ---- 6. Recovery ----
  // =========================================================================
  //@req-spec-decode.h12
  //@req-spec-decode.h13
  assert(PopCount(io.ren_br_tags.map(_.valid)) <= 1.U,
    "VConfigUnit: more than one ren_br_tags entry valid in the same cycle")
  val do_br_snapshot   = io.ren_br_tags.map(_.valid).reduce(_ || _)
  val br_snapshot_tag  = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_tags.map(_.bits))
  val br_snapshot_val  = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_vconfig.map(compress))
  when (do_br_snapshot) {
    vcfg_snapshots(br_snapshot_tag) := br_snapshot_val
  }
  when (do_br_snapshot) {
    VecTrace.traceStruct("VConfigUnit", "snapshot_write", Seq(
      ("br_tag", br_snapshot_tag),
      ("vtype",  br_snapshot_val.asUInt)))
  }
  //@req-spec-decode.e2
  val shadow_next = WireInit(vcfg_shadow)
  //@req-spec-decode.h4
  //@req-spec-decode.f4
  val shadow_update_fires = (0 until coreWidth).map(w => io.com_valids(w) && io.com_is_vset(w)).reduce(_ || _)
  val shadow_update_lane  = WireInit(0.U(log2Ceil(coreWidth).W))
  for (w <- 0 until coreWidth) {
    when (io.com_valids(w) && io.com_is_vset(w)) {
      shadow_next := compress(io.com_vtype(w))
      shadow_update_lane := w.U
    }
  }
  vcfg_shadow := shadow_next
  when (shadow_update_fires) {
    VecTrace.traceStruct("VConfigUnit", "shadow_update", Seq(
      ("lane",  shadow_update_lane),
      ("vtype", shadow_next.asUInt)))
  }
  //@req-spec-decode.h2
  //@req-spec-core.d2
  //@req-spec-core.d3
  //@req-spec-decode.h8
  //@req-spec-decode.h9
  when (io.brupdate.b2.mispredict) {
    vcfg_mirror := vcfg_snapshots(io.brupdate.b2.uop.br_tag)
    VecTrace.traceTag("VConfigUnit", "recover_mispredict",
      io.brupdate.b2.uop, io.brupdate.b2.uop.br_tag)
  //@req-spec-decode.h10
  //@req-spec-decode.h11
  } .elsewhen (io.rollback) {
    vcfg_mirror := shadow_next
  } .otherwise {
    vcfg_mirror := vcfg_mirror_decode_update
    when (mirror_update_fires) {
      VecTrace.traceDecode("VConfigUnit", "mirror_update",
        mirror_update_ftq, mirror_update_pc,
        Seq(("vtype", vcfg_mirror_decode_update.asUInt),
            ("vill",  vcfg_mirror_decode_update.vill)))
    }
  }

  // =========================================================================
  // ---- 7. Trace and checks ----
  // =========================================================================
  assert(!io.rob_empty || expand(vcfg_shadow).asUInt === io.csr_vtype.asUInt,
    "VConfigUnit: committed shadow disagrees with the architectural vtype CSR while the ROB is empty")
}

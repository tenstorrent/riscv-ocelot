//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Config Unit (VCFG)
//------------------------------------------------------------------------------
//
// The VCFG is the decode-stage *speculative* mirror of the architectural VTYPE
// (the `VConfig` bundle: vlmax/vsew/vlmul/vma/vta). It exists so dependent
// vector uops can snapshot a `vtype` into their `vconfig` and the vector mapper
// can derive EMUL at decode -- a stage ahead of br_tag allocation at rename.
//
// It mirrors VTYPE only; VL is renamed into the VL register file and delivered
// via `pvl` at execute, so there is NO decode-time VL here (see frontend.rst
// "VL delivery"). Three pieces of state, modeled directly on BOOM's RMT
// (rename-maptable.scala):
//
//   - spec_vtype     : speculative working mirror, updated in program order as
//                      immediate-vtype vsets decode.
//   - com_vtype_r    : committed shadow, updated only by the ROB on vset commit.
//   - vcfg_snapshots : per-br_tag snapshot, maxBrCount deep, taken on the same
//                      ren_br_tags event as the RMT snapshots.
//
// Per-lane "nearest-preceding-vset" prefix select (frontend.rst, the
// `[vsetvli, vadd, vsetivli, vadd]` example): each lane's effective vtype is the
// in-bundle vset immediately preceding it in program order, else the mirror.
//
// Recovery is identical in shape to the RMT (rename-maptable.scala:117-126):
//   mispredict -> vcfg_snapshots(br_tag);  rollback -> committed shadow;
//   else       -> end-of-bundle prefix value.
//
// Only immediate-vtype vsets (vsetivli/vsetvli) update the mirror at decode;
// vsetvl (register VTYPE) is `is_unique` and updates the mirror from the EU once
// the pipeline drains, so it is NOT a decode-time mirror update and is not an
// input here.

package boom.v4.vec.decode

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.exu.BrUpdateInfo

class VConfigUnitIO(implicit p: Parameters) extends BoomBundle
{
  // coreWidth comes from HasBoomCoreParameters (via BoomBundle), same as the RMT.
  // Per-lane decode inputs (this cycle's decode bundle, program order = ascending index).
  val dec_valids    = Input(Vec(coreWidth, Bool()))       // lane has a valid uop this cycle
  val dec_is_vset   = Input(Vec(coreWidth, Bool()))       // lane is any vset (vsetivli/vsetvli/vsetvl)
  val dec_imm_vtype = Input(Vec(coreWidth, Bool()))       // vtype is immediate (vsetivli||vsetvli) -> updates mirror at decode
  val dec_vtype_in  = Input(Vec(coreWidth, new VConfig))  // vtype an imm-vtype vset sets (vsew/vlmul/vta/vma/vlmax precomputed by decode)

  // Per-lane output: effective vtype for this lane (nearest preceding in-bundle vset, else mirror).
  val lane_vtype    = Output(Vec(coreWidth, new VConfig))

  // Branch snapshot -- driven from the SAME ren_br_tags event the scalar RMT uses.
  val ren_br_tags      = Input(Vec(coreWidth + 1, Valid(UInt(brTagSz.W))))
  val br_carried_vtype = Input(Vec(coreWidth + 1, new VConfig)) // per-slot prefix-select value the branch "sees"

  // Mispredict / flush restore.
  val brupdate = Input(new BrUpdateInfo)  // use brupdate.b2.mispredict and brupdate.b2.uop.br_tag
  val rollback = Input(Bool())            // exception/flush -> restore committed shadow

  // Commit-time committed-vtype shadow update (driven by ROB on vset commit).
  val com_vset_valid = Input(Bool())
  val com_vtype      = Input(new VConfig)

  // Gated trace.
  val vec_trace = Input(Bool())

  // Optional debug tag for trace (rob_idx not available at decode -- per-lane id for correlation).
  val dec_uop_id = Input(Vec(coreWidth, UInt(32.W)))
}

class VConfigUnit(implicit p: Parameters) extends BoomModule
{
  val io = IO(new VConfigUnitIO)

  // The RVV reset VTYPE. RVV has no architected reset value for vtype other than
  // the `vill` convention; Caracal's VConfig bundle does not carry a vill bit, so
  // we reset every field to 0 (vsew=0 -> SEW=8, vlmul=0 -> LMUL=1, vta=vma=0),
  // matching the codebase's bundle-reset idiom (e.g. fetch-target-queue.scala:142).
  // vlmax rides the VConfig and is supplied precomputed by decode for live vsets;
  // at reset it is 0 and is never consumed before the first vset updates it.
  def resetVConfig: VConfig = (0.U).asTypeOf(new VConfig)

  // Speculative working mirror and committed shadow.
  val spec_vtype  = RegInit(resetVConfig)
  val com_vtype_r = RegInit(resetVConfig)

  // Per-br_tag snapshots -- same depth as the RMT's br_snapshots (maxBrCount).
  val vcfg_snapshots = Reg(Vec(maxBrCount, new VConfig))

  // --------------------------------------------------------------------------
  // Per-lane nearest-preceding-vset prefix select.
  //
  // Mirrors the RMT intra-bundle scanLeft bypass (rename-maptable.scala:88-92):
  // prefix(0) is the incoming mirror, and each lane that performs an imm-vtype
  // vset update overwrites the running value for all *younger* lanes. A vector
  // op at lane i therefore sees prefix(i) -- the nearest PRECEDING in-bundle
  // vset, else the mirror. prefix(coreWidth) is the end-of-bundle value used as
  // the next speculative mirror state.
  // --------------------------------------------------------------------------
  val lane_update = Wire(Vec(coreWidth, Bool()))
  for (i <- 0 until coreWidth) {
    lane_update(i) := io.dec_valids(i) && io.dec_is_vset(i) && io.dec_imm_vtype(i)
  }

  // scanLeft over the lanes: seed with spec_vtype, fold each lane's possible update.
  val prefix = (0 until coreWidth)
    .map(i => (lane_update(i), io.dec_vtype_in(i)))
    .scanLeft(spec_vtype) { case (cur, (upd, vt)) => Mux(upd, vt, cur) }

  for (i <- 0 until coreWidth) {
    io.lane_vtype(i) := prefix(i)
  }

  // --------------------------------------------------------------------------
  // Branch snapshots -- same enableSuperscalarSnapshots / Mux1H pattern as the
  // RMT (rename-maptable.scala:101-115). The snapshot value is the branch's
  // carried vtype (io.br_carried_vtype), the nearest-preceding-vset prefix value
  // the branch sees, captured on the same ren_br_tags event as the RMT.
  // --------------------------------------------------------------------------
  if (enableSuperscalarSnapshots) {
    for (i <- 0 until coreWidth + 1) {
      when (io.ren_br_tags(i).valid) {
        vcfg_snapshots(io.ren_br_tags(i).bits) := io.br_carried_vtype(i)
      }
    }
  } else {
    assert(PopCount(io.ren_br_tags.map(_.valid)) <= 1.U)
    val do_br_snapshot    = io.ren_br_tags.map(_.valid).reduce(_ || _)
    val br_snapshot_tag   = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_tags.map(_.bits))
    val br_snapshot_vtype = Mux1H(io.ren_br_tags.map(_.valid), io.br_carried_vtype)
    when (do_br_snapshot) {
      vcfg_snapshots(br_snapshot_tag) := br_snapshot_vtype
    }
  }

  // --------------------------------------------------------------------------
  // Next-state for the speculative mirror -- same priority as the RMT
  // (rename-maptable.scala:117-126): mispredict restores the br_tag snapshot,
  // rollback restores the committed shadow, otherwise advance to the
  // end-of-bundle prefix value.
  // --------------------------------------------------------------------------
  when (io.brupdate.b2.mispredict) {
    spec_vtype := vcfg_snapshots(io.brupdate.b2.uop.br_tag)
  } .elsewhen (io.rollback) {
    spec_vtype := com_vtype_r
  } .otherwise {
    spec_vtype := prefix(coreWidth)
  }

  // Committed shadow: updated only by the ROB on vset commit.
  com_vtype_r := Mux(io.com_vset_valid, io.com_vtype, com_vtype_r)

  // --------------------------------------------------------------------------
  // Gated trace -- one greppable line per event, prefixed "[VCFG]".
  // --------------------------------------------------------------------------
  when (io.vec_trace) {
    // Per imm-vtype vset lane update (decode-time mirror update).
    for (i <- 0 until coreWidth) {
      when (lane_update(i)) {
        printf("[VCFG] dec lane=%d uop_id=%d vset vsew=%d vlmul=%d vta=%d vma=%d vlmax=%d\n",
          i.U, io.dec_uop_id(i),
          io.dec_vtype_in(i).vsew, io.dec_vtype_in(i).vlmul,
          io.dec_vtype_in(i).vta, io.dec_vtype_in(i).vma, io.dec_vtype_in(i).vlmax)
      }
    }
    // Committed shadow update.
    when (io.com_vset_valid) {
      printf("[VCFG] commit vsew=%d vlmul=%d vta=%d vma=%d vlmax=%d\n",
        io.com_vtype.vsew, io.com_vtype.vlmul, io.com_vtype.vta, io.com_vtype.vma, io.com_vtype.vlmax)
    }
    // Mispredict restore.
    when (io.brupdate.b2.mispredict) {
      printf("[VCFG] mispredict restore br_tag=%d vsew=%d vlmul=%d vta=%d vma=%d vlmax=%d\n",
        io.brupdate.b2.uop.br_tag,
        vcfg_snapshots(io.brupdate.b2.uop.br_tag).vsew, vcfg_snapshots(io.brupdate.b2.uop.br_tag).vlmul,
        vcfg_snapshots(io.brupdate.b2.uop.br_tag).vta, vcfg_snapshots(io.brupdate.b2.uop.br_tag).vma,
        vcfg_snapshots(io.brupdate.b2.uop.br_tag).vlmax)
    } .elsewhen (io.rollback) {
      // Flush to committed shadow.
      printf("[VCFG] rollback flush vsew=%d vlmul=%d vta=%d vma=%d vlmax=%d\n",
        com_vtype_r.vsew, com_vtype_r.vlmul, com_vtype_r.vta, com_vtype_r.vma, com_vtype_r.vlmax)
    }
  }
}

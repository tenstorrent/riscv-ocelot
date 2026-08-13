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

package boom.v4.vec.generated.rename

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common.{BoomBundle, BoomModule}
import boom.v4.exu.BrUpdateInfo
import boom.v4.vec.generated.VecTrace

// GENERATED from src/main/nlhdl/vec/rename/VecMapTable.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
//
// SPEC DEFECT (reported, not resolved): VecMapReq/VecMapResp/VecRemapReq
// declared locally, not in VecBundles per spec.

class VecMapReq(val emulSz: Int)(implicit p: Parameters) extends BoomBundle
{
  val lvd   = UInt(lregSz.W)
  val lvs1  = UInt(lregSz.W)
  val lvs2  = UInt(lregSz.W)
  val lvs3  = UInt(lregSz.W)
  val lvm   = UInt(lregSz.W)
  val emul  = UInt(emulSz.W)
  val valid = Bool()
}

class VecMapResp(val pregSz: Int, val maxGroupSize: Int, val emulSz: Int)(implicit p: Parameters) extends BoomBundle
{
  val pvs1         = Vec(maxGroupSize, UInt(pregSz.W))
  val pvs2         = Vec(maxGroupSize, UInt(pregSz.W))
  val pvs3         = Vec(maxGroupSize, UInt(pregSz.W))
  val stale_pvdest = Vec(maxGroupSize, UInt(pregSz.W))
  val pvm          = UInt(pregSz.W)
  val v_emul       = UInt(emulSz.W)
}

class VecRemapReq(val pregSz: Int, val maxGroupSize: Int, val emulSz: Int)(implicit p: Parameters) extends BoomBundle
{
  val lvd    = UInt(lregSz.W)
  val pvdest = Vec(maxGroupSize, UInt(pregSz.W))
  val emul   = UInt(emulSz.W)
  val valid  = Bool()
}

class VecMapTable(
  val plWidth:        Int,
  val numArchRegs:    Int,
  val maxGroupSize:   Int,
  val numPhysRegs:    Int,
  val bypass:         Boolean = true,
  val exportComStale: Boolean = false)
  (implicit p: Parameters) extends BoomModule
{
  private def isPow2(x: Int): Boolean = x > 0 && (x & (x - 1)) == 0
  require(isPow2(numArchRegs), s"numArchRegs ($numArchRegs) must be a power of two")
  require(isPow2(maxGroupSize), s"maxGroupSize ($maxGroupSize) must be a power of two")
  require(maxGroupSize <= numArchRegs,
    s"maxGroupSize ($maxGroupSize) must be <= numArchRegs ($numArchRegs)")

  // ---- Derived widths ----
  val pregSz  = log2Ceil(numPhysRegs)
  val lvregSz = math.max(log2Ceil(numArchRegs), 1)
  val emulSz  = log2Ceil(maxGroupSize) + 1

  val io = IO(new BoomBundle()(p) {
    val map_reqs       = Input(Vec(plWidth, new VecMapReq(emulSz)))
    val map_resps       = Output(Vec(plWidth, new VecMapResp(pregSz, maxGroupSize, emulSz)))

    val remap_reqs      = Input(Vec(plWidth, new VecRemapReq(pregSz, maxGroupSize, emulSz)))
    val com_remap_reqs  = Input(Vec(plWidth, new VecRemapReq(pregSz, maxGroupSize, emulSz)))
    val com_stale_resps = if (exportComStale) Some(Output(Vec(plWidth, Vec(maxGroupSize, UInt(pregSz.W))))) else None

    val ren_br_tags = Input(Vec(plWidth + 1, Valid(UInt(brTagSz.W))))
    val brupdate    = Input(new BrUpdateInfo)
    val rollback    = Input(Bool())
  })

  // =========================================================================
  // ---- State ----
  // =========================================================================

  //@req-spec-rename.d7
  //@req-spec-rename.h7
  //@req-spec-rename.h19
  val map_table     = RegInit(VecInit((0 until numArchRegs).map(i => i.U(pregSz.W))))
  val com_map_table = RegInit(VecInit((0 until numArchRegs).map(i => i.U(pregSz.W))))
  val br_snapshots  = Reg(Vec(maxBrCount, Vec(numArchRegs, UInt(pregSz.W))))

  //@req-spec-core.h4
  //@req-spec-rename.e10

  // =========================================================================
  // ---- Per-lane destination-group bounds ----
  // =========================================================================

  private def loOf(lvd: UInt): UInt = lvd(lvregSz - 1, 0)
  private def hiOf(lo: UInt, emul: UInt): UInt = lo +& emul

  val remap_lo = io.remap_reqs.map(r => loOf(r.lvd))
  val remap_hi = (remap_lo zip io.remap_reqs).map { case (lo, r) => hiOf(lo, r.emul) }

  val com_remap_lo = io.com_remap_reqs.map(r => loOf(r.lvd))
  val com_remap_hi = (com_remap_lo zip io.com_remap_reqs).map { case (lo, r) => hiOf(lo, r.emul) }

  // =========================================================================
  // ---- The EMUL-wide group read, with the in-bundle prefix bypass ----
  // =========================================================================

  private def groupRow(specBase: UInt, m: Int): UInt =
    loOf(specBase) | m.U(lvregSz.W)

  private def bypassFold(i: Int, row: UInt): UInt = {
    val raw = map_table(row)
    if (!bypass) raw else {
      (0 until i).foldLeft(raw) { case (prev, k) =>
        val inGroup = io.remap_reqs(k).valid && row >= remap_lo(k) && row < remap_hi(k)
        Mux(inGroup, io.remap_reqs(k).pvdest(row - remap_lo(k)), prev)
      }
    }
  }

  //@req-spec-rename.d8
  //@req-spec-rename.d9
  private def groupReadBypassed(i: Int, specBase: UInt): Vec[UInt] =
    VecInit((0 until maxGroupSize).map(m => bypassFold(i, groupRow(specBase, m))))

  private def singleReadBypassed(i: Int, specBase: UInt): UInt =
    bypassFold(i, loOf(specBase))

  for (i <- 0 until plWidth) {
    //@req-spec-rename.a10
    io.map_resps(i).pvs1 := groupReadBypassed(i, io.map_reqs(i).lvs1)
    io.map_resps(i).pvs2 := groupReadBypassed(i, io.map_reqs(i).lvs2)
    //@req-spec-vrf.j7
    io.map_resps(i).pvs3 := groupReadBypassed(i, io.map_reqs(i).lvs3)
    io.map_resps(i).pvm  := singleReadBypassed(i, io.map_reqs(i).lvm)

    //@req-spec-rename.d3
    //@req-spec-rename.d4
    //@req-spec-rename.d6
    //@req-spec-vrf.i1
    //@req-spec-vrf.i2
    //@req-spec-vrf.i3
    //@req-spec-vrf.j8
    io.map_resps(i).stale_pvdest := groupReadBypassed(i, io.map_reqs(i).lvd)

    //@req-spec-rename.d14
    io.map_resps(i).v_emul := io.map_reqs(i).emul
    when (io.map_reqs(i).valid) {
      VecTrace.traceStruct("VecMapTable", "stale_capture", Seq(
        ("lane",         i.U),
        ("lvd",          io.map_reqs(i).lvd),
        ("stale_pvdest", io.map_resps(i).stale_pvdest(0)),
        ("nmem",         io.map_reqs(i).emul)))
    }
  }

  // =========================================================================
  // ---- The group write ----
  // =========================================================================

  //@req-spec-rename.d2
  //@req-spec-rename.i15
  val remap_table     = Wire(Vec(plWidth + 1, Vec(numArchRegs, UInt(pregSz.W))))
  val com_remap_table = Wire(Vec(plWidth + 1, Vec(numArchRegs, UInt(pregSz.W))))

  for (i <- 0 until numArchRegs) {
    val row = i.U(lvregSz.W)

    val remappedRow = (0 until plWidth).scanLeft(map_table(i)) { (pdst, k) =>
      val inGroup = io.remap_reqs(k).valid && row >= remap_lo(k) && row < remap_hi(k)
      Mux(inGroup, io.remap_reqs(k).pvdest(row - remap_lo(k)), pdst)
    }
    val comRemappedRow = (0 until plWidth).scanLeft(com_map_table(i)) { (pdst, k) =>
      val inGroup = io.com_remap_reqs(k).valid && row >= com_remap_lo(k) && row < com_remap_hi(k)
      Mux(inGroup, io.com_remap_reqs(k).pvdest(row - com_remap_lo(k)), pdst)
    }

    for (j <- 0 until plWidth + 1) {
      remap_table(j)(i)     := remappedRow(j)
      com_remap_table(j)(i) := comRemappedRow(j)
    }
  }

  //@req-spec-rename.h20
  if (exportComStale) {
    for (k <- 0 until plWidth) {
      io.com_stale_resps.get(k) := VecInit((0 until maxGroupSize).map { m =>
        com_remap_table(k)(com_remap_lo(k) | m.U(lvregSz.W))
      })
    }
  }

  // =========================================================================
  // ---- Snapshots and recovery ----
  // =========================================================================

  //@req-spec-rename.i9
  //@req-spec-rename.i10
  //@req-spec-rename.i6
  if (enableSuperscalarSnapshots) {
    for (i <- 0 until plWidth + 1) {
      when (io.ren_br_tags(i).valid) {
        br_snapshots(io.ren_br_tags(i).bits) := remap_table(i)
      }
    }
  } else {
    assert(PopCount(io.ren_br_tags.map(_.valid)) <= 1.U,
      "VecMapTable: more than one ren_br_tags entry valid in the same cycle")
    val do_br_snapshot   = io.ren_br_tags.map(_.valid).reduce(_ || _)
    val br_snapshot_tag  = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_tags.map(_.bits))
    val br_snapshot_table = Mux1H(io.ren_br_tags.map(_.valid), remap_table)
    when (do_br_snapshot) {
      br_snapshots(br_snapshot_tag) := br_snapshot_table
    }
  }
  //@req-spec-rename.i11

  //@req-spec-rename.i5
  //@req-spec-rename.i7
  //@req-spec-rename.h8
  when (io.brupdate.b2.mispredict) {
    VecTrace.traceTag("VecMapTable", "recover_mispredict", io.brupdate.b2.uop, io.brupdate.b2.uop.br_tag)
    map_table := br_snapshots(io.brupdate.b2.uop.br_tag)
  } .elsewhen (io.rollback) {
    map_table := com_map_table
  } .otherwise {
    map_table := remap_table(plWidth)
  }
  com_map_table := com_remap_table(plWidth)

  //@req-spec-rename.d10
  //@req-spec-rename.d11
  //@req-spec-rename.d12
  //@req-spec-rename.d13

  // =========================================================================
  // ---- Assertions ----
  // =========================================================================
  for (k <- 0 until plWidth) {
    when (io.remap_reqs(k).valid) {
      VecTrace.traceStruct("VecMapTable", "remap", Seq(
        ("lane",   k.U),
        ("lvd",    io.remap_reqs(k).lvd),
        ("pvdest", io.remap_reqs(k).pvdest(0)),
        ("nmem",   io.remap_reqs(k).emul)))
    }

    for (m <- 0 until maxGroupSize) {
      val memberValid = io.remap_reqs(k).valid && m.U < io.remap_reqs(k).emul
      assert(!memberValid || !map_table.contains(io.remap_reqs(k).pvdest(m)),
        "VecMapTable: trying to write a duplicate mapping.")
    }
    assert(!io.remap_reqs(k).valid ||
      (io.remap_reqs(k).emul >= 1.U && io.remap_reqs(k).emul <= maxGroupSize.U),
      "VecMapTable: emul out of legal range (1..maxGroupSize) on a valid remap request.")
    assert(!io.remap_reqs(k).valid || remap_hi(k) <= numArchRegs.U,
      "VecMapTable: remap request's destination group overflows numArchRegs.")
  }
}

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

package boom.v4.vec.generated

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile.TileKey
import freechips.rocketchip.rocket.VType

import boom.v4.common.BoomCoreParams

// GENERATED from src/main/nlhdl/pkg/VtypeTable.nlhdl.scala. Do not hand-edit;
// regenerate via the nlhdl gen-rtl flow instead.
//@req-spec-decode.d10
object VtypeTable {

  // ---- Binding to VectorParams ----
  private def vectorParams(implicit p: Parameters): VectorParams =
    p(TileKey).core.asInstanceOf[BoomCoreParams].vector.get

  private def vecVLSz(implicit p: Parameters): Int = {
    val vp = vectorParams
    (new HasVectorParams { val vectorParams = vp }).vecVLSz
  }

  // ---- The bundle ----

  class VtypeInfo(implicit p: Parameters) extends Bundle {
    val vlmax = UInt(vecVLSz.W)
    val emul  = UInt((log2Ceil(vectorParams.maxMembers) + 1).W)
    val vill  = Bool()
    val vta   = Bool()
    val vma   = Bool()
  }

  // ---- EMUL: shared core, used by decode()'s default and emul() below ----
  private def emulFromVLMax(vlmax: UInt, eewIn: UInt)(implicit p: Parameters): UInt = {
    val vp = vectorParams
    val vLen = vp.vLen
    val maxMembers = vp.maxMembers
    require(vLen >= 8, s"VtypeTable.emul: vLen ($vLen) must be at least 8")
    val emulWidth = log2Ceil(maxMembers) + 1
    val eew = eewIn(1, 0)
    val vLenShift = log2Ceil(vLen) - 3
    val raw = (vlmax << eew) >> vLenShift
    val clamped = Mux(raw === 0.U, 1.U, raw)
    val result = clamped(emulWidth - 1, 0)
    assert(result === 0.U || result <= maxMembers.U,
      "VtypeTable.emul: computed EMUL is neither 0 nor within 1..maxMembers " +
      "-- raw was not a power of two, which is a derivation bug (an illegal " +
      "vtype/eew combination legally returns 0, not an assertion failure)")
    result
  }

  //@req-spec-decode.a11
  //@req-spec-decode.a12
  //@req-spec-decode.a13
  //@req-spec-decode.a14
  def decode(bits: UInt)(implicit p: Parameters): VtypeInfo = {
    val vt = VType.fromUInt(bits)
    val info = Wire(new VtypeInfo)
    info.vill := vt.vill
    when (vt.vill) {
      info.vlmax := 0.U
      info.emul  := 0.U
    } .otherwise {
      info.vlmax := vt.vlMax
      info.emul  := emulFromVLMax(vt.vlMax, vt.vsew)
    }
    info.vta := vt.vta
    info.vma := vt.vma
    info
  }

  // ---- `resolve`: the FULL VType, for the consumers that need the bundle ----
  def resolve(bits: UInt)(implicit p: Parameters): VType = VType.fromUInt(bits)

  // ---- EMUL: the group size a rename must allocate ----
  def emul(info: VtypeInfo, eew: UInt)(implicit p: Parameters): UInt =
    emulFromVLMax(info.vlmax, eew)

  //@req-spec-decode.c2
  def computeVL(
    avl: UInt,
    bits: UInt,
    currentVL: UInt,
    useCurrentVL: Bool,
    useMax: Bool,
    useZero: Bool)(implicit p: Parameters): UInt = {
    val maxVLMaxSz = log2Ceil(p(TileKey).core.vLen)
    VType.computeVL(avl.pad(maxVLMaxSz), bits, currentVL.pad(maxVLMaxSz),
                    useCurrentVL, useMax, useZero)
  }
}

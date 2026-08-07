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
//
// VtypeTable — the one place that turns a `vtype` value into {VLMAX, EMUL,
// vill}, and an AVL into a VL.
//
// PACKAGE NODE CONVENTION. A `kind: package` emitting a Scala `object
// VtypeTable` of pure combinational functions, plus one small Bundle. No I/O,
// no state, no instance.
//
// WHY IT IS A PACKAGE AND NOT A MODULE. Its callers evaluate it a different
// number of times each: VConfigUnit once per decode lane, VsetDecode once per
// lane for the immediate forms, and ALUUnit once at execute for the
// register-sourced forms. Expressing a pure function as a Module would force
// `coreWidth + 1` instantiations of something with no state, and — worse —
// would let the three callers drift apart. Three copies of a `vill` rule is
// three chances to disagree about when a configuration is legal.
//
// ===> IT WRAPS ROCKET-CHIP'S `VType`; IT DOES NOT REIMPLEMENT IT.
//      `freechips.rocketchip.rocket.VType` already provides `lmul_ok`,
//      `max_vsew`, `vlMax` and a full-width `vl(...)`, and rocket's `CSRFile`
//      is the owner of architectural `vtype` under `usingVector`. If this file
//      computed VLMAX its own way, the host and the architectural CSR could
//      disagree about whether a configuration is legal. Bind to rocket's
//      definitions; add only what Caracal needs on top.
//
// Governing spec anchor: frontend.rst `vector-rvv-decode`, plus the VSET
// handling section for the VL computation.
//
// BINDING ASSUMPTION (documented, not in the nlhdl dependencies list). Every
// machine size this file needs (vLen, eLen, maxMembers) is read implicitly off
// the `Parameters` object, the same way rocket's `HasCoreParameters` reaches
// `coreParams`. There is no `HasVectorParams` mix-in available to a
// free-standing `object`, and this node's `hierarchy.yaml` `depends_on:` names
// only `VectorParams` (not `BoomCoreParams`), so `vectorParams` below reaches
// it exactly the way `HasBoomCoreParameters.boomParams`
// (v4/common/parameters.scala:195) already does: cast `TileKey`'s `core` to
// `BoomCoreParams` and read its `vector` option. `BoomCoreParams.vector` is
// itself a pending sibling delta (see hierarchy.yaml's `BoomCoreParams` node);
// this file assumes that field lands with exactly this name and type, and
// that every caller of this object only evaluates it under `usingRVV`/vector
// configs (an `Option.get` on `vector` elaboration-fails loudly otherwise,
// which is the desired fail-fast behavior rather than a silent default).
//@req-spec-decode.d10
// Every function below is combinational and side-effect-free: `object
// VtypeTable` holds no `Reg`, no `Mem`, and no per-call state, so a caller may
// evaluate it once per decode lane — up to the machine's NxWide decoder width
// — with each evaluation an independent copy of a small comparator/shifter
// tree, and no arbitration between lanes is ever needed. Nothing below may
// introduce state, because that would make the lane count load-bearing.
object VtypeTable {

  // ---- Binding to VectorParams (see the BINDING ASSUMPTION note above) ----
  private def vectorParams(implicit p: Parameters): VectorParams =
    p(TileKey).core.asInstanceOf[BoomCoreParams].vector.get

  // vecVLSz (9 bits at the defaults) is a *derived* HasVectorParams value, not
  // a bare VectorParams field, so borrow the trait's own formula rather than
  // restating `log2Ceil(vLen * maxMembers / 8) + 1` here — one place computes
  // it, same reasoning the rest of this design uses for VType.
  private def vecVLSz(implicit p: Parameters): Int = {
    val vp = vectorParams
    (new HasVectorParams { val vectorParams = vp }).vecVLSz
  }

  // ---- The bundle ----

  /**
   * VtypeInfo — one caller-facing decode result, so a caller takes one Bundle
   * rather than four parallel functions that could be called inconsistently.
   *
   * `vlmax`/`emul` are meaningful only when `vill` is clear; [[decode]] drives
   * both to zero when `vill` is set, so a consumer that ignores `vill` reads
   * an obviously-wrong answer instead of a plausible one. `vta`/`vma` are the
   * policy bits, passed straight through from the decoded `vtype`.
   */
  class VtypeInfo(implicit p: Parameters) extends Bundle {
    val vlmax = UInt(vecVLSz.W)
    val emul  = UInt((log2Ceil(vectorParams.maxMembers) + 1).W)
    val vill  = Bool()
    val vta   = Bool()
    val vma   = Bool()
  }

  // ---- EMUL: shared core, used by decode()'s default and emul() below ----
  //
  // EMUL = LMUL * EEW / SEW. Since VLMAX = vLen*LMUL/SEW, LMUL/SEW = VLMAX/vLen,
  // so EMUL = VLMAX * EEW / vLen — computable from VLMAX and EEW alone, with
  // no separate SEW term needed.
  //
  // `eew` is consulted only in its low 2 bits: a code 0..3 => EEW 8/16/32/64,
  // matching MicroOp.v_eew's own encoding (src/main/nlhdl/pkg/MicroOp.nlhdl.scala,
  // "Add `v_eew`, 2 bits, the element width of the DATA a memory access
  // moves"), so a caller can pass `v_eew` straight through with no
  // reinterpretation, and decode() below can pass the freshly-decoded
  // `vtype.vsew` the same way for the SEW-relative (arithmetic-op) case.
  //
  // `vLen` and the EEW-code-to-bytes conversion are both powers of two, so the
  // whole computation is two shifts — one dynamic, by the low 2 bits of `eew`;
  // one static, by the compile-time `log2Ceil(vLen) - 3` — never a multiply or
  // a divide, per the perf section ("VLMAX and EMUL are computed by shifting
  // vLen by SEW/LMUL/EEW exponents, never by dividing").
  private def emulFromVLMax(vlmax: UInt, eewIn: UInt)(implicit p: Parameters): UInt = {
    val vp = vectorParams
    val vLen = vp.vLen
    val maxMembers = vp.maxMembers
    require(vLen >= 8, s"VtypeTable.emul: vLen ($vLen) must be at least 8")
    val emulWidth = log2Ceil(maxMembers) + 1
    val eew = eewIn(1, 0) // 2-bit EEW code, matching MicroOp.v_eew
    val vLenShift = log2Ceil(vLen) - 3 // log2(vLen/8); vLen is a multiple of 8
    val raw = (vlmax << eew) >> vLenShift
    // Clamp the LOWER bound only: a fractional EMUL still occupies one whole
    // register. The upper bound is an invariant, not a clamp — a legal vtype
    // cannot produce a group wider than `maxMembers`, and silently truncating
    // a too-wide result would corrupt the rename allocation instead of
    // surfacing the bug, so an over-`maxMembers` result is an assertion
    // failure rather than a saturated value.
    val clamped = Mux(raw === 0.U, 1.U, raw)
    assert(clamped <= maxMembers.U,
      "VtypeTable.emul: computed EMUL exceeds maxMembers -- an illegal " +
      "vtype/eew combination reached this function uncaught")
    clamped(emulWidth - 1, 0)
  }

  //@req-spec-decode.a11
  //@req-spec-decode.a12
  //@req-spec-decode.a13
  //@req-spec-decode.a14
  // Do NOT hand-write the vill checks. `VType.fromUInt(bits)`'s returned
  // `vill` is already the disjunction of exactly the three conditions Caracal
  // needs:
  //   - `!lmul_ok` — the VLMAX >= 1 constraint (a11). For fractional LMUL,
  //     rocket's `lmul_ok` is `vlmul_mag =/= 0 && ~vlmul_mag < max_vsew -
  //     vsew`, false precisely when VLEN*LMUL/SEW < 1.
  //   - `max_vsew < vsew` (a12) — SEW greater than ELEN.
  //   - `reserved =/= 0` (a13). The reserved vlmul=3'b100 encoding (a14) falls
  //     out of the same `lmul_ok` expression rather than needing a case of its
  //     own: 3'b100 is vlmul_sign=1, vlmul_mag=0, so `lmul_ok`'s
  //     `vlmul_mag =/= 0` term is false and `vill` is set. No separate
  //     comparison against 3'b100 is added — a redundant check that disagreed
  //     with rocket's would be worse than no check.
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

  // ---- EMUL: the group size a rename must allocate ----
  //
  // `emul(info, eew)` returns the number of registers in a group whose
  // elements are `eew` bits wide, as a 1..`maxMembers` count. For an
  // arithmetic op the element width is SEW and EMUL is LMUL — exactly what
  // `decode()` already bundles as `info.emul` above, by calling this same
  // core with `eew = vtype.vsew`. For a load or store the element width is
  // the instruction's own EEW, and a caller passes that instead.
  //
  // This is the value that reaches the vector mapper as v_emul and decides
  // how many PRNs an OP.v allocates atomically. An EMUL that disagreed with
  // the one the VPU derives from the same issue packet would corrupt member
  // indexing on the CII, so it is derived from vtype+eew only — never from a
  // separate decode path.
  def emul(info: VtypeInfo, eew: UInt)(implicit p: Parameters): UInt =
    emulFromVLMax(info.vlmax, eew)

  //@req-spec-decode.c2
  // `computeVL(avl, bits, currentVL, useCurrentVL, useMax, useZero)` returns
  // the new VL. For the immediate form (`vsetivli`) this is `min(uimm,
  // VLMAX)`. Do not write that `min` by hand either: delegate to rocket's
  // `VType.computeVL`/`VType.vl`, which already implements the RVV rules
  // including the `rs1 = x0` max and keep-current cases the register-sourced
  // forms need.
  //
  // Compare the FULL-WIDTH `avl` — do not truncate it to `vecVLSz+1` bits
  // before comparing against VLMAX, or a large AVL WRAPS instead of
  // saturating (AVL=2048 -> vl=0), breaking the canonical strip-mining idiom
  // where AVL is the whole remaining element count and is expected to
  // saturate to VLMAX on every iteration but the last. `VType.vl(...)`
  // already handles this correctly by testing the high bits separately
  // (`atLeastMaxVLMax`) instead of truncating.
  //
  // A VL of zero is a legal, reachable result and not an error: consumers
  // handle it (see VecGroupCopy for the destination-group consequence). This
  // function does not special-case it.
  def computeVL(
    avl: UInt,
    bits: UInt,
    currentVL: UInt,
    useCurrentVL: Bool,
    useMax: Bool,
    useZero: Bool)(implicit p: Parameters): UInt =
    VType.computeVL(avl, bits, currentVL, useCurrentVL, useMax, useZero)
}

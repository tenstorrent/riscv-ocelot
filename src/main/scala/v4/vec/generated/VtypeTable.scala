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
    // register.
    val clamped = Mux(raw === 0.U, 1.U, raw)

    // ===> AN EMUL ABOVE `maxMembers` IS AN ILLEGAL INSTRUCTION, NOT AN
    // IMPOSSIBLE STATE, AND THIS FUNCTION MUST NOT ASSERT ON IT. A legal
    // `vtype` ALONE cannot produce a group wider than `maxMembers`, but
    // `vtype` PLUS an EEW that differs from SEW can, and routinely does: a
    // widening op (EEW = 2*SEW) at LMUL=8 gives EMUL=16, and an indexed
    // access with EEW=64 against SEW=8 gives EMUL = 8*LMUL. RVV 1.0 reserves
    // exactly those encodings, and Caracal traps them at DECODE — VDecode's
    // EMUL-bound term is the architectural check, and it can only be reached
    // because this function RETURNS the out-of-range case instead of dying
    // on it. An assertion on the untruncated value fires on a machine that
    // is behaving correctly: every vwadd/vwmul at LMUL=8 would abort a cosim
    // run while the DUT was, correctly, raising an illegal-instruction trap.
    //
    // ===> THE OUT-OF-RANGE INDICATION IS THE RETURN VALUE 0, AND THAT IS A
    // CONTRACT, NOT AN ACCIDENT OF TRUNCATION. `raw` is always a power of
    // two — it is `vlmax` shifted up by the EEW code and down by a
    // compile-time constant — so every out-of-range EMUL (16, 32, 64 or 128)
    // is congruent to 0 modulo 2^emulWidth, and truncating to `emulWidth`
    // bits below maps every one of them onto 0. Zero is otherwise
    // unreachable, because the fractional case above is clamped UP to 1.
    // Callers therefore test `emul === 0` for "group too wide", and NO
    // caller may treat 0 as a group size. `decode()` above independently
    // drives `emul` to 0 when `vill` is set, which is the same contract
    // from the other direction.
    //
    // The assertion below is the one that is actually invariant, and it is
    // what makes the 0-contract sound rather than decorative: the returned
    // value is either 0 or in 1..`maxMembers`. It fires precisely when `raw`
    // was not a power of two — i.e. when the shift-only derivation above was
    // broken — which is the bug that would let a genuine group size alias
    // onto the reserved 0 encoding.
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

  // ---- `resolve`: the FULL VType, for the consumers that need the bundle ----
  //
  // `decode` returns a DIGEST (`{vlmax, emul, vill, vta, vma}`) which drops
  // `vsew`, `vlmul_sign` and `vlmul_mag` and therefore cannot reconstruct a
  // `VType`. But `MicroOp.vconfig`, `rob_vconfig`, the VCFG mirror, its
  // committed shadow and the per-br_tag snapshot array all carry the WHOLE
  // bundle -- it is the type rocket's CSRFile holds architectural vtype in.
  // Without an entry point here, each such consumer invents its own, and the
  // second one to do so got it wrong in a way no width check could catch:
  // it reinterpreted the raw bits with `.asTypeOf` and overrode only
  // `vill`/`reserved`, leaving `vsew`/`vlmul_*`/`vta`/`vma` holding whatever
  // `rs2` happened to contain.
  //
  // RVV 1.0 requires that WHEN `vill` IS SET, EVERY OTHER `vtype` FIELD READS
  // AS ZERO. `VType.fromUInt` implements exactly that -- its result starts as
  // `WireInit(0.U.asTypeOf(new VType))` and is assigned only on the `!vill`
  // path -- so a `csrr vtype` after an illegal `vset` returns zero. A
  // hand-built version returns garbage instead, which is a cosim mismatch
  // against the Whisper reference with no elaboration error anywhere.
  //
  // The rule: a consumer needing a full `VType` calls `resolve`; a consumer
  // needing legality plus VLMAX/EMUL calls `decode`; nobody hand-builds a
  // `VType` from raw bits. Both go through `VType.fromUInt`, so they cannot
  // disagree. This is a pure delegation and has no logic of its own on
  // purpose -- its entire value is being the one name consumers can reach.
  def resolve(bits: UInt)(implicit p: Parameters): VType = VType.fromUInt(bits)

  // ---- EMUL: the group size a rename must allocate ----
  //
  // `emul(info, eew)` returns the number of registers in a group whose
  // elements are `eew` bits wide, as a 1..`maxMembers` count when the
  // combination is legal, and 0 when it is not — see the out-of-range
  // contract documented in `emulFromVLMax` above. For an arithmetic op the
  // element width is SEW and EMUL is LMUL — exactly what `decode()` already
  // bundles as `info.emul` above, by calling this same core with
  // `eew = vtype.vsew`. For a load or store the element width is the
  // instruction's own EEW, and a caller passes that instead.
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
  // ===> BOTH `avl` AND `currentVL` ARE PADDED UP TO `maxVLMax.log2` BITS, AND
  //      THIS IS LOAD-BEARING, NOT TIDINESS. rocket's `VType.vl` does
  //      `Mux(useCurrentVL, currentVL, avl)(maxVLMax.log2 - 1, 0)` -- an
  //      unconditional 8-bit slice at VLEN=256 -- so it REQUIRES both operands
  //      to be at least that wide. Two callers are narrower:
  //        - VConfigUnit passes `vsetivli`'s AVL, a 5-bit `uimm[4:0]`;
  //        - every caller passes `currentVL = 0.U` on the paths that do not use
  //          it, and a `0.U` literal is ONE bit.
  //      The Mux takes the max of its operand widths, so both cases produce a
  //      slice of bits 7:0 out of a 5-bit (or 1-bit) value, which is a hard
  //      Chisel elaboration error: "High index 7 is out of range [0, 4]".
  //      Padding here fixes it once for every caller instead of at each call
  //      site, where the next caller would have to rediscover it.
  //
  //      `.pad` ONLY WIDENS -- it can never truncate -- so this cannot
  //      reintroduce the addvector wrap-instead-of-saturate bug described
  //      above. That bug came from NARROWING a wide AVL; this widens a narrow
  //      one. A 64-bit `rs1_data` AVL passes through untouched.
  //
  //      The width is taken from `p(TileKey).core.vLen`, which IS rocket's
  //      `maxVLMax` (`tile/Core.scala`: `def maxVLMax = vLen`) as BOOM
  //      supplies it, rather than from `vectorParams.vLen` -- so if the two
  //      ever diverge this pad follows the one rocket's slice actually uses.
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

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
//
// VLSDecode — the STATIC ACCESS DESCRIPTOR of a vector load or store: `v_eew`,
// `mop`, `nf`, the unit-stride / strided / indexed / segment / whole-register /
// mask flags, and which vector source operands the form actually encodes,
// decoded from the instruction word alone.
//
// WHAT THIS DESCRIPTOR IS FOR. It is what selects the FILL-side address
// generator downstream: `VecRangeAgen` (one range entry for the whole access)
// for unit-stride, `VecElemAgen` (one element access per active element) for
// strided, segmented and indexed. That selection is by ACCESS CLASS and NEVER
// by direction — load and store take the identical three rules; direction is
// a PARAMETER of the agen instances (`isStore`), not an input to the choice.
//
// THIS MODULE READS `vtype` FOR NOTHING. Every field it produces comes from
// the 32-bit instruction word — EEW, `mop` and `nf` are ENCODED, whereas SEW,
// LMUL and EMUL are CONFIGURATION. That is why this module has neither
// VtypeTable nor VectorParams as a dependency.
//
// Elaborated only when `usingRVV` (a Scala Boolean, NOT rocket's
// `usingVector`) is true, by virtue of the parent (`VecDecode`) only
// instantiating it under that condition — this module itself performs no
// internal `usingRVV` gating and holds no register, so it is ABSENT, not
// tied off, in a non-vector build.
//
// Governing spec anchors: execution.rst `vector-agen`, loadstore.rst
// `elem-progress` ("Fault-only-first (`vleff.v`)"), frontend.rst
// `vector-rvv-decode`.

/**
 * VLSAccessDesc — the statically-decoded access descriptor. Declared here
 * because this module is its sole producer.
 *
 * `mop` is carried VERBATIM rather than reduced to the three class flags, so a
 * consumer that needs the indexed-ordered vs. indexed-unordered distinction
 * (the memory-ordering units) reads `mop` instead of asking for another
 * synthesized flag here.
 */
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

/**
 * Per-lane port. `valid` is a QUALIFIER, not a handshake: there is no `ready`
 * anywhere in this module, because decode does not back-pressure and a
 * combinational decoder cannot stall. Outputs are still driven when `valid`
 * is low; a consumer must qualify them.
 *
 * `uop` is read for the trace context only (`ftq_idx`/`pc_lob`) — this module
 * reads no other field of it, specifically not `vconfig`.
 */
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

/**
 * VLSDecode — `coreWidth` (from `HasBoomCoreParameters`, via `BoomModule`)
 * independent copies of one pure combinational decoder; no cross-lane logic
 * whatsoever, no arbitration, no shared resource, no state. Instantiated ONCE
 * by `VecDecode`, as `ls`, and fans out internally.
 *
 * perf: single-cycle, purely combinational, zero registers. Every output is a
 * function of FIXED instruction bit ranges — no adder, no shifter, no
 * multiplier, no barrel select. `nregs` (`nf + 1`, 3 bits in) is the only
 * arithmetic in the file. No pipeline stage may be added: the descriptor must
 * be available in the same cycle as the instruction. No state, so no flush,
 * squash or branch-kill path — a mis-speculated lane simply is not `valid`
 * and the outputs are discarded.
 */
class VLSDecode(implicit p: Parameters) extends BoomModule
{
  val io = IO(new VLSDecodeIO)

  for (i <- 0 until coreWidth) {
    val lane = io.lanes(i)
    val inst = lane.inst

    // ---- 1. Is this even a vector load/store ----
    //
    // A vector load/store is opcode == LOAD-FP (0000111) or opcode ==
    // STORE-FP (0100111) — the same two opcodes as scalar flw/fld and
    // fsw/fsd — disambiguated by the `width` field inst(14,12): a VECTOR
    // access uses 000 (EEW 8), 101 (16), 110 (32) or 111 (64), while scalar
    // FP uses 010 and 011.
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

    // `is_store` comes from the opcode and NOTHING else. It is recorded in
    // the descriptor for the LSU's direction routing and for the trace line.
    // It is never an input to any class decision below.
    val is_store = is_store_fp

    // ---- 2. Field extraction: fixed bits, no arithmetic ----
    val nf   = inst(31, 29)
    val mew  = inst(28)
    val mop  = inst(27, 26)
    val vm   = inst(25)
    val umop = inst(24, 20) // lumop / sumop
    // `eew` is the 2-bit log2 element-size `width` names: for every one of
    // the four legal vector `width` patterns (000/101/110/111) the low two
    // bits of `width` already equal the desired EEW code (0/1/2/3), so no
    // separate lookup/mux is needed — a fixed bit slice of `inst`.
    val eew = inst(13, 12)

    // ---- 3. ACCESS CLASS: one-hot, exhaustive, from `mop` alone ----
    //@req-spec-agen.c26
    // The three class flags are decoded from `mop` and from `mop` only:
    // `is_store` appears in none of these expressions, and the load and
    // store paths consume the identical descriptor — the generator is
    // selected by ACCESS CLASS, NOT BY DIRECTION.
    val is_unit_stride = mop === 0.U
    val is_indexed     = mop === 1.U || mop === 3.U // unordered / ordered
    val is_strided     = mop === 2.U

    //@req-spec-agen.c5
    // `is_unit_stride` and `is_indexed` come from disjoint `mop` encodings,
    // so they are MUTUALLY EXCLUSIVE BY CONSTRUCTION: the range/Packer path
    // (VecRangeAgen) can never be selected for an indexed access, which it
    // does not support. Structural guarantee, not a checked one.

    //@req-spec-agen.c13
    //@req-spec-agen.c22
    // A non-indexed, non-unit-stride access (strided or segmented, masked or
    // not) selects VecElemAgen's NO-INDEX mode. It must NOT select the
    // index-driven mode, which does not support non-indexed accesses:
    // VecIdxGen's enable downstream is `is_indexed` and nothing else, so on a
    // strided access the index interface is never started.

    //@req-spec-agen.c21
    // An INDEXED access selects VecElemAgen's index-driven mode, for
    // `mop` == 01 and `mop` == 11 alike, and REGARDLESS OF `vm`: indexed and
    // indexed-masked take the same generator. Masking is an optimization
    // inside the generator, not a selection criterion — `vm` therefore
    // appears in the descriptor as plain data and in none of the class
    // expressions above.
    //
    // In v2 the Skipper/Walker distinction is no longer a module boundary:
    // masked-skip and index-driven differ only in where the per-element
    // address comes from, so they are two modes of VecElemAgen. The
    // selection OBLIGATION survives the merge and lives here, in the one
    // place that decides the class.

    //@req-spec-agen.c25
    // The class decode is TOTAL and there is NO DEFAULT ARM: every value of
    // the 2-bit `mop` field selects exactly one class, so an access is never
    // assigned a generator by falling through to one.
    assert(!(lane.valid && is_vls) ||
      PopCount(Seq(is_unit_stride, is_strided, is_indexed)) === 1.U,
      "VLSDecode: access class flags must be exactly one-hot")
    // The assert is a check, not behaviour: deleting it leaves the emitted
    // datapath bit-identical.

    // ---- 5. Unit-stride sub-forms: the `umop` decode ----
    //
    // When `is_unit_stride`, `umop` names which unit-stride form this is,
    // and all four remain the unit-stride access class (contiguous byte
    // ranges, all taking VecRangeAgen):
    //   00000 — ordinary unit-stride (vle*.v / vse*.v)
    //   01000 — WHOLE REGISTER (vl<nregs>re<eew>.v / vs<nregs>r.v)
    //   01011 — MASK (vlm.v / vsm.v)
    //   10000 — FAULT-ONLY-FIRST (vleff.v), load only
    val is_ordinary_us = umop === "b00000".U
    val is_whole_reg   = is_unit_stride && umop === "b01000".U

    // `nf` DOES NOT MEAN SEGMENT COUNT ON A WHOLE-REGISTER ACCESS: the same
    // three bits carry NFIELDS-1 for a segmented access and the register
    // count minus one for vl<n>re<eew>. `nregs` (nf + 1) is meaningful only
    // when `is_whole_reg`; its only legal values are 1, 2, 4 and 8 (checked
    // in part 8 below).
    // `+&` (growing add), not `+`: nf's max value 7 plus 1 is 8, which does
    // not fit back in nf's own 3-bit width -- a plain `+` would truncate
    // 8 (0b1000) to 0, silently wrapping nregs to zero instead of 8.
    val nregs = nf +& 1.U

    val is_mask = is_unit_stride && umop === "b01011".U

    //@req-spec-lsu.g5
    //@req-spec-lsu.g11
    // `is_ff` marks vleff.v and marks nothing else. `vleff` IS NOT
    // SERIALIZED: it is an ordinary speculative vector unit-stride load —
    // this module sets no fence, no barrier, no ordering and no
    // serialization attribute on it, and never sets/touches `is_unique`.
    // vleff is hot in strlen/memchr-style loops, so any is_unique term
    // reachable from is_ff is a reject, not a conservatism.
    val is_ff = is_unit_stride && umop === "b10000".U

    // `vleff` IS A VL PRODUCER (writes its trimmed element count to its VL-RF
    // destination and wakes `pvl`), but that decision and the VL-RF write
    // port live in VecDecode/VlRegFile, not here — this module contributes
    // only the `is_ff` bit that decision is made from.
    //
    // CORRECTED. An earlier draft of this paragraph said `is_ff` does NOT make the
    // uop a VL producer "until the real fault-trim path exists", citing a
    // hierarchy.yaml comment that has since been corrected. That was a STAGING note
    // that read as a design statement, and it contradicted two requirements
    // allocated to sibling nodes (`VlRegFile`'s write port and
    // `VecLoadCoalescingBuffer`'s trim report) as well as `VecDecode`, the actual
    // writer. Staging is real but belongs in the plan's step table: the VL-RF write
    // port for `vleff` is declared from day one and simply never fires until step
    // G4 lands. It must NOT be retrofitted as an arbiter on the ALU write port —
    // `spec-decode.c27` forbids arbitrated VL-RF write ports.

    // A bare `nf =/= 0` would make e.g. vl8re64 look like an 8-field
    // segmented load; qualify against the other unit-stride forms.
    val is_segment = nf =/= 0.U && !is_whole_reg && !is_mask && !is_ff

    // ---- 7. Indexed accesses: `eew` names the INDEX, not the data ----
    //
    // For every non-indexed class, `eew` is the MEMORY DATA element width.
    // For an INDEXED access the `width` field encodes the INDEX element
    // width, and the data element width is `vtype.vsew`, which this module
    // deliberately cannot see — so `eew` is driven from `width`
    // unconditionally (above) and the reinterpretation is made explicit via
    // `eew_is_index`.
    val eew_is_index = is_indexed

    // ---- 7b. `uses_vs*` — which vector sources a MEMORY form encodes ----
    //
    // A function of `mop` and the opcode direction only — no vtype, no
    // arithmetic funct6.
    val uses_vs1 = false.B // NO vector load or store form encodes a vs1
                            // vector source; that field position holds rs1
                            // (the integer base address) on every one of them.
    val uses_vs2 = is_indexed // vs2 names the INDEX group on an indexed
                               // access; on a strided access the same field
                               // is rs2 (integer stride), on unit-stride it
                               // is umop (no register at all).
    val uses_vs3 = is_store // Store DATA is vs3, read by the DGEN. A load
                             // encodes no vector source operand whatever.
    // WITHOUT THESE BITS A PLAIN vle64.v HANGS: its vs1/vs2/vs3 fields hold
    // an integer base and a sub-opcode, so a mapper reading them as vector
    // specifiers would rename whatever architectural vreg those bit
    // patterns happen to name — frequently v0, the mask register.
    //
    // ===> `uses_vs3` IS THE DIRECTION BIT, AND THIS IS THE ONE PLACE IN THIS FILE
    // WHERE `is_store` LEGITIMATELY APPEARS ON THE RIGHT-HAND SIDE. It is NOT a
    // class decision — part 3's rule that the agen is selected by class and never
    // by direction is untouched, since `uses_vs3` selects no generator and reaches
    // no agen enable. It says only which register file the DGEN reads, which is a
    // property of the direction by definition.
    //
    // And the store's SCALAR operands are INTEGER ONLY: `rs1` (base) and `rs2`
    // (stride). No RVV store form takes an FP scalar operand — store data is
    // always `vs3` — so nothing in this descriptor implies an FP read on the store
    // path, and nothing downstream should provide one. `vfmul.vf`, the instruction
    // once cited for a store-side FP read, is vector-scalar ARITHMETIC dispatched
    // to the coprocessor and is not a store at all; the store-side FP read lane is
    // deleted and the only remaining FP reader is the CII issue path.

    // ---- 8. Reserved encodings ----
    //
    // A segmented INDEXED access is LEGAL (vluxseg/vsuxseg) and must not
    // appear in this list. Everything else legal decodes with `illegal` low.
    val illegal = is_vls && (
      mew ||
      (is_unit_stride && !(is_ordinary_us || is_whole_reg || is_mask || is_ff)) ||
      (is_whole_reg && !(nregs === 1.U || nregs === 2.U || nregs === 4.U || nregs === 8.U)) ||
      (is_mask && (nf =/= 0.U || eew =/= 0.U)) ||
      (is_ff && is_store) ||
      ((is_whole_reg || is_mask) && !vm)
    )
    // Reporting a reserved encoding here rather than letting it reach rename
    // is what stops a mis-sized PRN group being allocated for an instruction
    // with no defined meaning.

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
    //
    // The `traceDecode` variant is mandatory here, not a preference: at
    // DECODE the uop's rob_idx is not yet assigned (the ROB allocates at
    // DISPATCH), so this line keys on ftq_idx/pc_lob instead and correlates
    // with later stages by PC/ftq_idx through the dispatch line.
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

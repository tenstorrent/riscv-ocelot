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
// VConfigUnit — the Vector Config Unit (VCFG): the speculative decode-stage
// `vtype` mirror, and the committed shadow it is recovered from.
//
// This unit owns EXACTLY TWO pieces of state and nothing else: the
// speculative mirror and the committed shadow (plus the per-`br_tag`
// snapshot array that is how the mirror is branch-recovered — the nlhdl
// source calls this a third element, but architecturally it is the mirror's
// own recovery mechanism, not a third independent piece of ownership). It is
// NOT architectural state and NOT a CSR file: rocket-chip's `CSRFile` under
// `usingVector` owns architectural `vtype` (including `vill`), `vl`,
// `vstart`, `vxrm`, `vxsat`, `vcsr`, `vlenb` and `mstatus.VS`. This unit
// mirrors `vtype` ONLY — never `vl`, never `vstart`, never `vxrm`.
//
// THERE IS NO EXECUTE-TIME MIRROR WRITE. No port here is driven from an
// execution unit. `vsetvl` (vtype from a register) is marked both
// `is_unique` and `flush_on_commit` by the DecodeUnit delta; recovery of the
// mirror for it is committed-shadow + refetch, because `is_unique` alone
// stalls only the unique uop's own dispatch and says nothing about younger
// uops.
//
// THE POISON FLAG IS `vtype.vill` ITSELF, not a bit beside it: the mirror is
// one `VType`, and poisoning it means setting the `vill` field of that same
// value, so poison snapshots, restores and is carried into `vconfig` for
// free, with no recovery path of its own.
//
// Elaborated only when `usingRVV` (a Scala Boolean, NOT rocket's
// `usingVector`) is true, by virtue of the parent (`VecDecode`) only
// instantiating it under that condition, per the sibling decode-stage
// modules' own convention (see `VLSDecode`): this module performs no
// internal `usingRVV` gating of its own.
//
// TRACING. `VecTrace` now offers the full three-step ladder (`trace*` ->
// `traceId` -> `traceStruct`; see `VecTrace.scala`'s "two uOP-less variants"
// note), which closes all but one of the four gaps a prior generation of
// this file reported. Per event, first rung that applies:
//   - "mirror update at decode" (part 6's `.otherwise` arm) is decode-lane-
//     synchronous, so `dec_ftq_idx`/`dec_pc_lob` correctly identify it via
//     `traceDecode`, unchanged from before.
//   - "snapshot write" (part 6, on `ren_br_tags` allocation) has no MicroOp
//     or rob_idx on this module's rename-stage boundary (`ren_br_tags`/
//     `ren_br_vconfig` carry a bare tag and a bare `VType`, nothing else),
//     so this is bottom-rung `traceStruct`, keyed on `br_tag` -- the honest
//     identifier of which branch's snapshot slot was written -- plus the
//     `vtype` value written, the same key+payload shape `VecMapTable`'s own
//     `traceStruct("remap", ...)` uses.
//   - "restore, mispredict arm" (part 6, `io.brupdate.b2.mispredict`) is NOT
//     bottom-rung, and AN EARLIER VERSION OF THIS FILE WRONGLY CLAIMED
//     OTHERWISE. `io.brupdate.b2` is a `BrResolutionInfo`, which mixes in
//     `HasBoomUOP` (`val uop = new MicroOp()`, v4/common/micro-op.scala:23),
//     so `io.brupdate.b2.uop` IS a real MicroOp -- with a real `rob_idx` --
//     already in scope via the existing `brupdate` port. Per the ladder's
//     own ordering ("the FIRST that applies"), rung 1 applies: `traceTag`
//     with that uop and its `br_tag`, exactly the call `VecMapTable`'s and
//     `VecRenameSpace`'s own `recover_mispredict` events already make for
//     the identical shape of event (see those files' headers, which
//     document correcting this same wrong claim). No new port was added to
//     get this identifier -- `brupdate` was already here.
//   - "restore, rollback arm" (part 6, `io.rollback`) has NO equivalent.
//     `rollback` is a bare `Input(Bool())` with no br_tag, no MicroOp, no
//     rob_idx anywhere upstream of it, and the event (`vcfg_mirror :=
//     shadow_next`) is a single whole-mirror overwrite, not a per-branch or
//     per-lane one -- there is no honest identifying key to hand
//     `traceStruct`, and fabricating one (e.g. a constant marker) is exactly
//     what `traceStruct`'s non-empty-`extra` require exists to keep out.
//     SPEC DEFECT (reported, not resolved): this one line remains OMITTED,
//     flagged inline at the `.elsewhen (io.rollback)` arm below, under the
//     same discipline `VecMapTable`'s, `VecFreeList`'s and
//     `VecRenameSpace`'s own generated files already apply to their
//     identical bare-`rollback` sites.
//   - "shadow update at commit" (part 6, on a committing `vset*`) has no
//     MicroOp or rob_idx on this module's commit-stage boundary either
//     (`com_valids`/`com_is_vset`/`com_vtype` carry no `MicroOp`, no
//     `rob_idx`) -- bottom-rung `traceStruct`, keyed on `lane` (the
//     committing slot, highest index wins per program order) and `vtype`
//     (the value installed).
// Section 7 carries no `//@req-` tags of its own, so no requirement is left
// untagged by the one remaining omission. The quiescent-state assertion
// (also section 7) needs only `csr_vtype`/`rob_empty`, both real ports, and
// is untouched by any of this.
//
// SPEC DEFECT (reported, not resolved) -- VtypeTable.decode's return type.
// `VtypeTable.decode(bits): VtypeInfo` reduces a decoded `vtype` to
// `{vlmax, emul, vill, vta, vma}` -- it drops `vsew`, `vlmul_sign` and
// `vlmul_mag` entirely. Those three fields are exactly what the mirror, the
// shadow, the snapshot array and `dec_vconfig` must carry (the ports
// section calls `dec_vconfig` "rocket's VType", the full bundle). The
// dependencies section states VtypeTable is used "for decode (legality...)"
// and separately says the mirror "deliberately holds the same bundle the
// CSRFile holds architectural vtype in" and binds to
// `freechips.rocketchip.rocket.VType` "by name". Taking both at face value
// is impossible: `VtypeInfo` cannot reconstruct a `VType`. Resolution: this
// file calls `freechips.rocketchip.rocket.VType.fromUInt` directly for the
// full decode used by the mirror/`dec_vconfig`/shadow/snapshot path -- the
// exact function `VtypeTable.decode`'s own doc comment says it "delegates
// to" for the `vill` rule, so the legality computation is still the single
// rocket-chip source of truth and is not reimplemented or duplicated.
// `VtypeTable` is still used for its other named purpose, `computeVL`
// (`vsetivli`'s VL, part 4 below). `VtypeTable.decode` itself is never
// called from this file, since its digest return cannot serve this
// module's need and calling it anyway would add nothing but a second,
// disjoint decode of the same bits.
//
// Governing spec anchors: frontend.rst `vector-rvv-decode` (VSET Special
// Handling), `vector-csr-explicit`, `vector-csr-ownership`, `vcfg-recovery`;
// overview.rst `boom-relationship`, `caracal-pipeline`; midcore.rst
// `regfiles-bypass`, `vl-vtype-rename`. Plan v2 section 5 ground rule 9 and
// the Phase B notes.

/**
 * VcfgBits — the nine significant bits of rocket's `VType`
 * (`vlmul_sign`, `vlmul_mag[1:0]`, `vsew[2:0]`, `vta`, `vma`, `vill`), and
 * nothing else. `VType` is nominally `xLen` bits wide because of its
 * `reserved` field, which `VType.fromUInt` always forces to zero -- so
 * storing the full width in the mirror, the shadow and every one of the
 * `maxBrCount` snapshot entries would cost roughly 7x the flops for bits
 * that are always zero. `expand` below reconstructs `reserved = 0` on read,
 * per the nlhdl parameters section's storage-sizing note.
 */
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
  // BOTH validity and fire are needed, and they gate DIFFERENT things. See the
  // "TWO PREFIX SCANS" note in part 3: `dec_valids` gates the combinational
  // prefix whose result LEAVES this module, `dec_fire` gates the one that
  // updates the mirror REGISTER. Using `dec_fire` for the outputs closes a
  // combinational loop through BoomCore's decode-stall logic.
  val dec_valids        = Input(Vec(coreWidth, Bool()))
  val dec_fire          = Input(Vec(coreWidth, Bool()))
  val dec_is_vset        = Input(Vec(coreWidth, Bool()))
  // Raw, not-yet-legality-checked encoded vtype bits for an immediate-form
  // vset. Sized to `xLen`: this is fed straight to
  // `VType.fromUInt`/`VtypeTable.computeVL`, both of which reinterpret their
  // `bits`/`vtype` argument as a full-width `VType` via `.asTypeOf`, so it
  // must be exactly as wide as that bundle (`xLen` bits, per the `reserved`
  // field). ASSUMPTION: the sibling immediate-form decoders (VsetDecode)
  // deliver this already shaped to `xLen`, not the instruction's own
  // (narrower) immediate-field width.
  val dec_vtype_imm     = Input(Vec(coreWidth, UInt(xLen.W)))
  val dec_vtype_is_imm  = Input(Vec(coreWidth, Bool()))
  val dec_is_vsetivli   = Input(Vec(coreWidth, Bool()))
  val dec_avl_imm       = Input(Vec(coreWidth, UInt(5.W)))
  val dec_uses_vtype    = Input(Vec(coreWidth, Bool()))
  // This lane's `MicroOp.ftq_idx`/`MicroOp.pc_lob`, wired by VecDecode from
  // `dec_uops_in(w)`. THESE EXIST ONLY TO TAG THE TRACE LINES OF PART 7 --
  // no functional logic below reads either. They are decode-lane-scoped
  // (this cycle's bundle only), which is why they can tag the decode-lane
  // mirror-update trace of part 4 but not the rename-/mispredict-/
  // commit-stage events of part 7 (see the SPEC DEFECT note in the file
  // header and at those sites).
  val dec_ftq_idx       = Input(Vec(coreWidth, UInt(log2Ceil(ftqSz).W)))
  val dec_pc_lob        = Input(Vec(coreWidth, UInt(log2Ceil(icBlockBytes).W)))

  // ---- Decode-stage outputs, one entry per lane ----
  val dec_vconfig        = Output(Vec(coreWidth, new VType))
  // The `vtype` in effect IMMEDIATELY BEFORE this lane -- the strictly-
  // EXCLUSIVE prefix (the part-3 `scanLeft`'s running value ENTERING lane
  // `w`, which at `w = 0` is `vcfg_mirror` itself). Consumed by VsetDecode's
  // `prev_vtype`. NOT `dec_vconfig` shifted by one lane -- see part 3.
  val dec_prev_vconfig   = Output(Vec(coreWidth, new VType))
  // `valid` is the port the nlhdl body calls `dec_vl_imm_valid`; bundling it
  // with the VL value follows the same `Valid(...)` idiom this module's own
  // `ren_br_tags` port below already uses for a qualified per-lane value.
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

  // Deliberately NO port for: an execute-time vtype write, a VL value
  // input, a vstart/vxrm/vxsat input or output, a separate `mirror_valid`
  // output, a stall/busy output, OR `keep_vl_illegal` (VsetDecode's reserved
  // keep-VL check): that vtype is itself legal, so it reaches the mirror
  // through the ordinary part-4 path and the trap's `flush_on_commit`
  // recovery already covers it -- see the nlhdl ports-section callout. The
  // illegality still reaches the outgoing uop, but through VecDecode ORing
  // it into `dec_vec_illegal`, not through a port here.
}

/**
 * VConfigUnit ("vcfg") — see the file header for the full design rationale.
 * Instantiated once by `VecDecode`.
 */
class VConfigUnit(implicit p: Parameters) extends BoomModule
{
  val io = IO(new VConfigUnitIO)

  // ---- 9-bit VType <-> VcfgBits helpers (storage-sizing note above) ----
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
  // Reset value for every one of the three state elements: all-zero VType
  // with `vill` SET -- i.e. poisoned at reset. Not a free choice: matches
  // rocket's CSRFile reset of `reg_vconfig.vtype` (CSR.scala, the `when
  // (reset)` in the io.vector block: `vtype := 0.U.asTypeOf(new
  // VType); vtype.vill := true.B`), so the very first vector instruction
  // after reset is judged illegal here exactly as the architectural CSR
  // would judge it.
  private def poisoned(): VcfgBits = {
    val b = WireInit(0.U.asTypeOf(new VcfgBits))
    b.vill := true.B
    b
  }

  // =========================================================================
  // ---- 1. What this unit is, and what it is deliberately not ----
  // =========================================================================
  //
  //@req-spec-decode.d2
  //@req-spec-decode.d3
  // This module IS the Vector Config Unit: a local, speculative copy of the
  // `vtype` value most recently set by a `vset`, so younger dependent
  // instructions can derive EMUL and snapshot `vtype` at decode without a
  // CSR read or a back-end round trip. Because uOP cracking is deferred to
  // the vector LS AGEN, the vector mapper must know EMUL at rename to
  // allocate the right number of PRNs -- the reason this copy exists at
  // decode at all.
  //
  //@req-spec-core.b8
  //@req-spec-decode.g6
  //@req-spec-decode.g7
  // Vector architectural CSR state is DELEGATED, not reimplemented here.
  // Architectural `vtype` (with `vill`), `vl`, `vstart`, `vxrm`, `vxsat`,
  // `vcsr`, `vlenb` and `mstatus.VS` dirty tracking / the `VS=Off` gate all
  // live in rocket-chip's `CSRFile` under `usingVector`, reached through
  // `csr.io.vector` -- no such CSR read appears anywhere below; this
  // module's only two inputs that even mention the CSR (`csr_vtype`,
  // `rob_empty`) feed nothing but the section-7 assertion.
  //
  //@req-spec-core.d1
  //@req-spec-rename.h6
  //@req-spec-vrf.c5
  //@req-spec-vrf.c8
  // VTYPE IS NOT RENAMED: no VTYPE map table, no free list, no busy table,
  // no wakeup network and NO VTYPE REGISTER FILE anywhere below -- only VL
  // gets a register file (VecRenameSpace's `vl_rename` instance, elsewhere).
  // VTYPE reaches the execution units by riding the mirror into a per-uOP
  // snapshot instead of a whole rename space.
  //
  //@req-spec-vrf.c6
  //@req-spec-decode.d4
  // That per-uOP delivery IS the `dec_vconfig` output (part 3 below): each
  // lane is handed the `vtype` that applies to it, and `VecDecode` writes it
  // into that uop's `MicroOp.vconfig`, so the EU reads VTYPE off the
  // instruction it is executing -- never from a CSR at execute, never from
  // this module.
  //
  //@req-spec-decode.d5
  //@req-spec-decode.d13
  // THE MIRROR DOES NOT MIRROR `vl`, and VL is never broadcast back into it.
  // There is no decode-time VL value at all: VL is renamed into the VL
  // register file and a younger vector uOP carries `pvl` and reads the
  // value at execute. The one VL quantity computed here (`dec_vl_imm`, part
  // 4) is an OUTPUT; nothing brings it back. A `vl` input would reintroduce
  // exactly the decode-stage VL tracker that the VL rename space replaces.

  // =========================================================================
  // ---- 2. State: three elements, and only three ----
  // =========================================================================

  //@req-spec-decode.h3
  // `vcfg_mirror` -- the working, speculative copy, updated at decode. Read
  // by the per-lane select in part 3.
  val vcfg_mirror = RegInit(poisoned())

  //@req-spec-decode.h4
  //@req-spec-decode.f4
  // `vcfg_shadow` -- the committed shadow, holding the known-good
  // architectural `vtype`. Written ONLY from the commit inputs below, when
  // a `vset*` uop commits (part 6). No other writer exists.
  val vcfg_shadow = RegInit(poisoned())

  //@req-spec-decode.h6
  //@req-spec-decode.h14
  // `vcfg_snapshots` -- one `vtype` snapshot per branch tag. Nine bits times
  // `maxBrCount` is a few tens of flops.
  val vcfg_snapshots = RegInit(VecInit(Seq.fill(maxBrCount)(poisoned())))

  // =========================================================================
  // ---- 3. The per-lane select: a prefix, not a broadcast ----
  // =========================================================================

  // Per-lane immediate decode (SPEC DEFECT above: via VType.fromUInt
  // directly, not VtypeTable.decode, whose VtypeInfo digest cannot carry
  // vsew/vlmul). Legality (`vill`) is exactly rocket's own rule -- no local
  // VLMAX computation, no local reserved-encoding check.
  val decodedImm: Seq[VcfgBits] = (0 until coreWidth).map { w =>
    compress(VType.fromUInt(io.dec_vtype_imm(w)))
  }

  // A lane UPDATES the running value iff it is an immediate-vtype vset
  // (`vsetvli`/`vsetivli`); `vsetvl` never appears here because its
  // `dec_vtype_is_imm` is false (part 4).
  //
  // ===> THERE ARE TWO PREFIX SCANS, AND THE QUALIFIER DIFFERS BETWEEN THEM.
  //      THIS IS NOT REDUNDANCY -- ONE SCAN CLOSES A COMBINATIONAL LOOP.
  //
  //      `laneUpdatesFire` (fire-gated) drives the MIRROR REGISTER, and it must
  //      stay fire-gated for the reason hierarchy.yaml gives when it added
  //      `dec_fire` to the seam: "the vtype mirror must advance only on a lane
  //      that actually leaves decode. On a partially-firing bundle, updating on
  //      `dec_valids` makes the mirror DOUBLE-ABSORB a vset when the bundle
  //      re-presents."
  //
  //      `laneUpdates` (valid-gated) drives everything that LEAVES this module
  //      combinationally -- `dec_vconfig`, `dec_prev_vconfig`,
  //      `dec_vtype_illegal`. Those must NOT depend on `dec_fire`, because
  //      BoomCore computes `dec_fire` FROM them: `dec_vtype_illegal` reaches
  //      `dec_vec_illegal` -> the uop's `exception` -> `dec_xcpts` ->
  //      `dec_hazards` -> `dec_stalls` -> `dec_fire`. firtool's CheckCombLoops
  //      caught exactly that cycle on MegaBoomV4VectorConfig at gate (c):
  //        dec_fire_1 -> vec.io_dec_vec_illegal_1 -> decode_1.io_vec_illegal
  //          -> ... -> dec_hazards_1 -> dec_stalls_1 -> dec_fire_1
  //      Note Chisel elaboration does NOT check for combinational loops, so this
  //      passed elaboration and failed in firtool.
  //
  //      SWAPPING THE OUTPUT SCAN TO `dec_valids` IS BEHAVIOURALLY FREE, and the
  //      reason is a property of BOOM's decode stage rather than of this module:
  //      `core.scala:659` builds `dec_stalls` as a CUMULATIVE `scanLeft` OR, so
  //      `dec_fire(w)` implies every older lane also fires. Therefore a bundle in
  //      which an older vset does not fire has no younger lane that fires, and
  //      every younger lane's outputs are discarded and re-presented next cycle
  //      -- by which time the mirror register holds exactly the fired prefix.
  //      The two scans can only disagree on lanes whose outputs nobody consumes.
  val laneUpdates: Seq[Bool] = (0 until coreWidth).map { w =>
    io.dec_valids(w) && io.dec_is_vset(w) && io.dec_vtype_is_imm(w)
  }
  val laneUpdatesFire: Seq[Bool] = (0 until coreWidth).map { w =>
    io.dec_fire(w) && io.dec_is_vset(w) && io.dec_vtype_is_imm(w)
  }

  //@req-spec-decode.h1
  //@req-spec-decode.d11
  // A decode bundle is `coreWidth` instructions of program order presented
  // at once, so the value each lane sees is a PREFIX SELECT across the
  // bundle: lane `w` takes the nearest PRECEDING `vset` in the bundle,
  // falling back to `vcfg_mirror` if none. Implemented as a `scanLeft`
  // starting from `vcfg_mirror`, one mux stage per lane -- the same shape as
  // the scalar MapTable's `remap_table` `scanLeft`
  // (`v4/exu/rename/rename-maptable.scala`).
  // The OUTPUT scan (valid-gated) -- read by dec_vconfig/dec_prev_vconfig/
  // dec_vtype_illegal, none of which may depend on dec_fire.
  val running: Seq[VcfgBits] =
    (laneUpdates zip decodedImm).scanLeft(vcfg_mirror) { case (prev, (doUpdate, newVal)) =>
      Mux(doUpdate, newVal, prev)
    }
  // The REGISTER scan (fire-gated) -- read ONLY by vcfg_mirror_decode_update
  // (part 4) and the mirror-update trace. Never exported.
  val runningFire: Seq[VcfgBits] =
    (laneUpdatesFire zip decodedImm).scanLeft(vcfg_mirror) { case (prev, (doUpdate, newVal)) =>
      Mux(doUpdate, newVal, prev)
    }
  // running.length == coreWidth + 1; running(0) == vcfg_mirror (pre-bundle),
  // running(w + 1) == the value after lane w has been processed.

  //@req-spec-decode.d9
  // THE SELECT IS SELF-INCLUSIVE FOR A vset AND SELF-EXCLUSIVE FOR A
  // CONSUMER. `running(w + 1)` already reflects lane w's own update when
  // lane w is an immediate vset (self-inclusive: its own `dec_vconfig` is
  // its NEW vtype), and equals `running(w)` (the value from strictly
  // earlier lanes) when lane w is not a vset (self-exclusive: a consumer
  // sees state from BEFORE itself, never a same-cycle sibling's newest
  // value).
  //
  // Both `dec_vconfig` and `dec_prev_vconfig` are TAPS OF THIS ONE `running`
  // ARRAY -- no second prefix computation, no extra mux stage. `running(w)`
  // is the value ENTERING lane `w` (the exclusive prefix: `running(0) ==
  // vcfg_mirror` for `w = 0`); `running(w + 1)` is that same value with lane
  // `w`'s own update applied when lane `w` is a vset (the inclusive prefix).
  // Exporting `running(w)` directly -- rather than a consumer reconstructing
  // it as `dec_vconfig(w - 1)`, which has no `w = 0` case -- is exactly what
  // fixes VecDecode's `prev_vtype(0) := DontCare` (see the nlhdl ports
  // section).
  for (w <- 0 until coreWidth) {
    io.dec_vconfig(w)      := expand(running(w + 1))
    io.dec_prev_vconfig(w) := expand(running(w))
  }

  // =========================================================================
  // ---- 4. Updating the mirror ----
  // =========================================================================

  //@req-spec-decode.d12
  // On the last lane of the bundle, the mirror takes the running value out
  // of the end of the scanLeft -- the ordinary decode-update path, used in
  // the priority chain's `otherwise` arm in part 6. Covers `vsetvli` (VTYPE
  // immediate, VL from `rs1` at the integer ALU EU) and `vsetivli` (both
  // immediate). A `vsetvl` (`dec_vtype_is_imm` false for that lane) leaves
  // the running value unchanged at that lane -- it cannot update the mirror
  // at decode, since its vtype is a register value unknown until execute;
  // it is safe only because of part 6's `flush_on_commit` recovery.
  val vcfg_mirror_decode_update: VcfgBits = runningFire(coreWidth)

  // (trace, part 7) The decode lane RESPONSIBLE for
  // `vcfg_mirror_decode_update` -- the highest-index lane with `laneUpdates`
  // set, mirroring which lane's write the scanLeft's last-connect semantics
  // actually keep. TRACE-ONLY: `dec_ftq_idx`/`dec_pc_lob` are read only to
  // build these two wires, and these two wires feed nothing but the
  // `VecTrace.traceDecode` call in part 6's `.otherwise` arm below.
  val mirror_update_fires: Bool = laneUpdatesFire.reduce(_ || _)
  val mirror_update_ftq = WireInit(io.dec_ftq_idx(0))
  val mirror_update_pc  = WireInit(io.dec_pc_lob(0))
  for (w <- 0 until coreWidth) {
    // `laneUpdatesFire`, matching `mirror_update_fires` above: this tags the
    // trace line for the MIRROR REGISTER update, so it must name the lane that
    // actually advanced it, not a valid-but-stalled one.
    when (laneUpdatesFire(w)) {
      mirror_update_ftq := io.dec_ftq_idx(w)
      mirror_update_pc  := io.dec_pc_lob(w)
    }
  }

  //@req-spec-decode.c1
  // `vsetivli` IS RESOLVED HERE IN A SINGLE CYCLE, front-end only: both its
  // VTYPE and its AVL are immediate, so `VL = min(uimm, VLMAX)` is available
  // in the decode cycle. VLMAX must come from THIS lane's own decoded vtype
  // (self-inclusive), not the incoming running value, so `VtypeTable.
  // computeVL` is applied to `io.dec_vtype_imm(w)` directly (it re-decodes
  // internally, `ignore_vill = true`, matching rocket's own AVL-vs-VLMAX
  // saturation rule) together with `io.dec_avl_imm(w)`. The VL-RF write
  // itself is NOT done here -- `pvl` is allocated by the VL mapper in the
  // RENAME cycle, so at decode there is no index to write to; this output is
  // the value only.
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
  // A vill-setting vset POISONS the mirror simply by being the value that
  // `running`/`vcfg_mirror_decode_update` carries forward: `decodedImm(w)`
  // is `VType.fromUInt`'s own result, whose `vill` is already rocket's
  // disjunction of `!lmul_ok`, `vsew > max_vsew` and `reserved =/= 0` (there
  // is no separate poison register -- see part 2). For `vsetivli`/
  // `vsetvli` this happens DIRECTLY, in the same cycle, through the ordinary
  // path above -- no special case. For `vsetvl` no mechanism is added here:
  // it is `flush_on_commit` (part 6), so younger uops refetch and decode
  // again against the restored mirror.
  //
  //@req-spec-decode.g9
  //@req-spec-decode.g12
  //@req-spec-decode.g13
  // While a lane's selected `vconfig` has `vill` set, that lane raises
  // `dec_vtype_illegal` IFF `dec_uses_vtype` is also set -- the gate is
  // `dec_uses_vtype` and nothing else, so a whole-register move/load/store
  // (`dec_uses_vtype` false, EMUL from its own `NREG` field) decodes and
  // executes normally under a poisoned mirror. A poisoned mirror still
  // propagates into `dec_vconfig` for every lane, including exempt ones;
  // poison suppresses no output.
  for (w <- 0 until coreWidth) {
    io.dec_vtype_illegal(w) := running(w + 1).vill && io.dec_uses_vtype(w)
  }

  // =========================================================================
  // ---- 6. Recovery ----
  // =========================================================================

  //@req-spec-decode.h12
  //@req-spec-decode.h13
  // THE SNAPSHOT SOURCE IS THE BRANCH'S CARRIED `vconfig`
  // (`ren_br_vconfig`), NOT THE MIRROR REGISTER: the mirror lives at decode,
  // one stage ahead of `br_tag` allocation at rename, so by the time a tag
  // is allocated the mirror may already have absorbed `vset*`s younger than
  // that branch. `ren_br_vconfig(i)` is exactly the nearest-preceding-vset
  // value the branch was handed by part 3's prefix select.
  //
  // Capture on the SAME `ren_br_tags` allocation event the scalar RMT snapshots
  // from, with the same one-hot reduction (`rename-maptable.scala`'s
  // `br_snapshots` write): assert at most one tag allocates per cycle, then
  // `Mux1H` the tag and the value into ONE write.
  assert(PopCount(io.ren_br_tags.map(_.valid)) <= 1.U,
    "VConfigUnit: more than one ren_br_tags entry valid in the same cycle")
  val do_br_snapshot   = io.ren_br_tags.map(_.valid).reduce(_ || _)
  val br_snapshot_tag  = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_tags.map(_.bits))
  val br_snapshot_val  = Mux1H(io.ren_br_tags.map(_.valid), io.ren_br_vconfig.map(compress))
  when (do_br_snapshot) {
    vcfg_snapshots(br_snapshot_tag) := br_snapshot_val
  }
  // (trace, part 7) Snapshot write. No MicroOp/rob_idx reaches this
  // module's rename-stage boundary (`ren_br_tags`/`ren_br_vconfig` carry a
  // bare tag and a bare `VType`), so bottom-rung `traceStruct`, keyed on
  // `br_tag` -- the honest identifier of which branch's snapshot slot this
  // write targets -- plus the `vtype` value written. See the file header's
  // TRACING note.
  when (do_br_snapshot) {
    VecTrace.traceStruct("VConfigUnit", "snapshot_write", Seq(
      ("br_tag", br_snapshot_tag),
      ("vtype",  br_snapshot_val.asUInt)))
  }
  //
  // A snapshot write and a mispredict restore never collide for the same
  // tag: a branch cannot resolve in the cycle it renames (`b2` is at least
  // one cycle behind allocation), the same assumption BOOM's MapTable
  // already makes.

  //@req-spec-decode.e2
  // THE FLUSH RESTORE MUST READ THE SHADOW'S NEXT VALUE, NOT THE REGISTER'S
  // CURRENT OUTPUT: `flush_on_commit` raises the flush the SAME cycle
  // `vsetvl`'s new vtype is being written into the shadow (part below), so
  // sourcing the restore from the register would reload the value from
  // BEFORE that `vsetvl` and mis-size the refetched code's EMUL. This wire
  // is also the shadow register's own next-value expression (defined once,
  // used by both), so it is correct whether the ROB raises the flush this
  // cycle or a cycle later.
  val shadow_next = WireInit(vcfg_shadow)
  //@req-spec-decode.h4
  //@req-spec-decode.f4
  // Shadow write: ONLY from commit, taking the NEWEST committing `vset*`
  // (highest lane index among `com_valids && com_is_vset`) since commit is
  // in program order -- later `when`s in this loop win by Chisel's
  // last-connect semantics, so the highest satisfying `w` dominates.
  // Rollback does not write the shadow: rollback retires nothing.
  //
  // (trace, part 7) Track the winning lane the same way, by last-connect
  // over the same loop, so the trace call below reports exactly the lane
  // that dominates `shadow_next` -- TRACE-ONLY: nothing functional reads
  // `shadow_update_lane`.
  val shadow_update_fires = (0 until coreWidth).map(w => io.com_valids(w) && io.com_is_vset(w)).reduce(_ || _)
  val shadow_update_lane  = WireInit(0.U(log2Ceil(coreWidth).W))
  for (w <- 0 until coreWidth) {
    when (io.com_valids(w) && io.com_is_vset(w)) {
      shadow_next := compress(io.com_vtype(w))
      shadow_update_lane := w.U
    }
  }
  vcfg_shadow := shadow_next
  // (trace, part 7) Shadow update at commit. No MicroOp/rob_idx reaches
  // this module's commit-stage boundary (`com_valids`/`com_is_vset`/
  // `com_vtype` carry no `MicroOp`, no `rob_idx`), so bottom-rung
  // `traceStruct`, keyed on `lane` (the committing slot that won the
  // last-connect above) and `vtype` (the value installed). See the file
  // header's TRACING note.
  when (shadow_update_fires) {
    VecTrace.traceStruct("VConfigUnit", "shadow_update", Seq(
      ("lane",  shadow_update_lane),
      ("vtype", shadow_next.asUInt)))
  }

  //@req-spec-decode.h2
  //@req-spec-core.d2
  //@req-spec-core.d3
  // The mirror is speculative, so a `vset*` on a mispredicted path that
  // updated it would corrupt the EMUL of every surviving younger vector
  // uop. It therefore gets THE SAME RECOVERY MACHINERY AS THE RENAME MAP
  // TABLE -- speculative copy, committed copy, per-br_tag snapshot array --
  // and not a mechanism of its own: mispredict first, then flush/rollback,
  // then the ordinary decode update, exactly the scalar MapTable's priority
  // ordering, so the two structures cannot diverge on a cycle where both a
  // mispredict and a flush are asserted.
  //@req-spec-decode.h8
  //@req-spec-decode.h9
  when (io.brupdate.b2.mispredict) {
    // Restore the mirror to a branch snapshot, in ONE CYCLE, in lockstep
    // with the RMT restore -- same trigger signal, same cycle.
    vcfg_mirror := vcfg_snapshots(io.brupdate.b2.uop.br_tag)
    // (trace, part 7) Restore, mispredict arm (source = snapshot). Rung 1
    // of the ladder applies despite this module otherwise having no MicroOp
    // on its boundary: `io.brupdate.b2` mixes in `HasBoomUOP`, so
    // `io.brupdate.b2.uop` is a real MicroOp already in scope via the
    // existing `brupdate` port -- no port was added to get this identifier.
    // `traceTag` with that uop and its own `br_tag` is exactly the call
    // `VecMapTable`'s and `VecRenameSpace`'s `recover_mispredict` events
    // already make for the identical event shape. See the file header's
    // TRACING note (which also corrects an earlier version of this file
    // that wrongly claimed no port here carries an identifier).
    VecTrace.traceTag("VConfigUnit", "recover_mispredict",
      io.brupdate.b2.uop, io.brupdate.b2.uop.br_tag)
  //@req-spec-decode.h10
  //@req-spec-decode.h11
  } .elsewhen (io.rollback) {
    // Restore the mirror from the committed shadow's NEXT value, in ONE
    // CYCLE, in parallel with `map_table := com_map_table`.
    vcfg_mirror := shadow_next
    // (trace, part 7 -- SPEC DEFECT, reported not resolved) Restore,
    // rollback arm (source = shadow): OMITTED. `rollback` is a bare
    // `Input(Bool())` with no br_tag, no MicroOp, no rob_idx anywhere
    // upstream of it, and the event is a single whole-mirror overwrite, not
    // a per-branch or per-lane one -- there is no honest identifying key to
    // hand `traceStruct`, and fabricating one is exactly what its
    // non-empty-`extra` require exists to keep out. Same discipline
    // `VecMapTable`'s, `VecFreeList`'s and `VecRenameSpace`'s own generated
    // files already apply to their identical bare-`rollback` sites. See the
    // file header's TRACING note.
  } .otherwise {
    // Ordinary decode update (part 4).
    vcfg_mirror := vcfg_mirror_decode_update
    // (trace, part 7) Mirror update at decode -- the ONE event of section
    // 7's four that IS decode-lane-synchronous, so `dec_ftq_idx`/
    // `dec_pc_lob` correctly identify the responsible lane (see the ports-
    // section comment on those two inputs and the file header's TRACING
    // note). Gated on `mirror_update_fires` (part 4) so a cycle with no
    // firing vset in the bundle emits nothing.
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
  //
  // TRACING SUMMARY. Section 7 calls for four events, and now three of the
  // four decode/rename/mispredict/commit lines fire: "mirror update at
  // decode" (`traceDecode`, part 6's `.otherwise` arm), "snapshot write"
  // (`traceStruct`, keyed on `br_tag`+`vtype`, part 6), and "shadow update
  // at commit" (`traceStruct`, keyed on `lane`+`vtype`, part 6). The
  // mispredict arm of the fourth event ("restore") also now fires
  // (`traceTag`, using the real MicroOp at `io.brupdate.b2.uop`, part 6) --
  // an earlier version of this file wrongly reported no port here carried
  // an identifier for it; `brupdate` already did.
  //
  // SPEC DEFECT (reported, not resolved) -- NARROWED, NOT CLOSED. Only the
  // rollback arm of "restore" (source = shadow) remains omitted: `rollback`
  // is a bare `Input(Bool())` with no br_tag, no MicroOp, no rob_idx
  // anywhere upstream of it, and the event is a single whole-mirror
  // overwrite with no honest identifying key to hand `traceStruct` -- see
  // the inline note at the `.elsewhen (io.rollback)` arm (part 6) and the
  // file header's TRACING note, which also documents the identical,
  // pre-existing omission in `VecMapTable`'s, `VecFreeList`'s and
  // `VecRenameSpace`'s own generated files. Section 7 carries no `//@req-`
  // tags, so no requirement is left untagged by this one remaining
  // omission.
  //
  // Quiescent-state assertion: with nothing speculative in flight
  // (`rob_empty`), the shadow and the architectural CSR -- written from the
  // same commit event -- must agree; a divergence is otherwise invisible
  // until a flush restores a wrong vtype. This assertion is the only
  // consumer of `csr_vtype` and `rob_empty`.
  assert(!io.rob_empty || expand(vcfg_shadow).asUInt === io.csr_vtype.asUInt,
    "VConfigUnit: committed shadow disagrees with the architectural vtype CSR while the ROB is empty")
}

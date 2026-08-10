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

import boom.v4.common._
import boom.v4.exu.BrUpdateInfo
import boom.v4.vec.generated.{VtypeTable, VecTrace}

// GENERATED from src/main/nlhdl/vec/decode/VecDecode.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
//
// VecDecode -- the RVV decode CONTAINER. It holds the four decode-stage units
// (VDecode as `arith`, VLSDecode as `ls`, VsetDecode as `vset`, VConfigUnit as
// `vcfg` -- all four single, coreWidth-wide instances, no per-lane
// instantiation anywhere in decode) and MERGES their per-lane results into one
// outgoing uOP per lane. Single-cycle, purely combinational apart from the
// state inside `vcfg`. One uOP in, one uOP out, per lane, per cycle -- no
// cracking, no expansion: element expansion into nOP.v happens only in the
// vector LS AGEN.
//
// ===> THE MERGE IS FIELD-WISE, NOT LANE-WISE. This module never selects ONE
// child's whole `uop_out` for a lane. On a segmented vector load/store lane,
// `arith` writes `is_shared`/`IQ_V_ALU` while this module writes the whole
// memory field group for the SAME lane -- a whole-uop select would silently
// drop one of the two halves and hang the survivor waiting for a rendezvous
// partner that was never dispatched. Field ownership is disjoint (see the
// table below the merge loop), so the union is well defined and
// order-independent -- bought by the SINGLE zeroing of `iq_type`/`fu_code`
// (and, per part 5's note, `v_uses_vs1/2/3`/`v_is_masked`) before any child's
// SET is applied, with every writer of those fields a SET, never a clear.
//
// ===> `MicroOp.vconfig` IS WRITTEN ON EVERY LANE, INCLUDING PURELY SCALAR
// ONES, unconditionally, with no `is_vec` and no `rvv_recognized` qualifier.
// `VConfigUnit` snapshots the vtype mirror per `br_tag` from the branch's own
// carried `vconfig` (ultimately `ren2_uops(w).vconfig`), and branches are
// overwhelmingly scalar. Gating this write on `is_vec` would leave every
// scalar branch's snapshot undriven/stale and corrupt the mirror on the first
// mispredict of any program that ever executed a `vset`.
//
// Governing spec anchors: frontend.rst `vector-rvv-decode` (including "CII
// Shared Instruction Decoding", "VSET Special Handling", `vset-dual-dest`),
// glossary.rst `glossary-terms` (uOP / OP.v / nOP.v). Plan v2 section 5 ground
// rules 1 (usingRVV), 2 (one uOP per instruction), 9 (rocket owns the
// architectural vector CSRs), 11 (guarded tracing).
//
// Elaborated only when `usingRVV` is true, by virtue of the parent
// (`VecPipeline`) only instantiating this module (and its four children)
// under that condition -- this module performs no internal `usingRVV` gating
// of its own, per the sibling decode-stage modules' own convention.
//
// ===> VConfigUnit GAINED THREE PORTS ON REGENERATION, CLOSING A DEFECT THIS
// FILE'S PREVIOUS GENERATION REPORTED (the fix lives on VConfigUnit's side;
// this file only wires the new ports):
//   (1) NEW OUTPUT `dec_prev_vconfig` (per-lane, self-EXCLUSIVE vtype -- the
//       prefix scan's INPUT side; at lane 0 this is `vcfg_mirror` itself, the
//       raw pre-bundle register). Wired to `vset.io.prev_vtype(w)` for EVERY
//       lane INCLUDING lane 0 (seam c below). This retires the previous
//       generation's `vset.io.prev_vtype(0) := DontCare`, which fed an
//       undriven value straight into VsetDecode's reserved keep-VL legality
//       comparison. NOT reconstructed as `dec_vconfig(w - 1)`: that identity
//       holds only for `w >= 1`, and VConfigUnit now exports the real
//       prefix-scan tap directly instead.
//   (2) NEW INPUTS `dec_ftq_idx`/`dec_pc_lob` (per-lane) -- trace-tagging
//       only. Wired from `dec_uops_in(w).ftq_idx`/`.pc_lob` below; no
//       functional logic in this file (or VConfigUnit) reads either.
//   `keep_vl_illegal` is DELIBERATELY NOT a VConfigUnit port, and this is not
//   a defect: the vtype a reserved keep-VL `vsetvli` carries is itself legal,
//   so the mirror absorbs a valid configuration and the trap's
//   `flush_on_commit` recovery restores it from the committed shadow, exactly
//   as `vsetvl` already relies on. Part 4 below (ORing
//   `vset.io.keep_vl_illegal(w)` into the OUTGOING UOP's `vconfig.vill`)
//   remains the only path by which the reserved encoding reaches anything,
//   and is unchanged.
class VecDecodeIO(implicit p: Parameters) extends BoomBundle
{
  // Clock/reset are the implicit Chisel signals; this module declares no
  // register and no memory of its own -- all decode-stage state lives in
  // `vcfg` -- so both reach only the guarded trace statements below (via
  // VecTrace) and the children.
  //
  // NO handshake, NO `ready`, NO `busy` in either direction: decode cannot be
  // stalled by the vector subsystem, and this module exports nothing that
  // could stall it.

  // ---- Decode feed, per lane ----
  val dec_insns    = Input(Vec(coreWidth, UInt(32.W)))
  val dec_valids   = Input(Vec(coreWidth, Bool()))
  // The lane ADVANCES out of decode this cycle (BOOM's dec_fire(w)). Wired to
  // vcfg.dec_fire and to nothing else -- NOT a substitute for dec_valids: a
  // partially-firing bundle must not let the vtype mirror double-absorb a
  // vset on re-presentation.
  val dec_fire     = Input(Vec(coreWidth, Bool()))
  val dec_uops_in  = Input(Vec(coreWidth, new MicroOp()))

  // ---- Decode results, per lane ----
  val dec_uops_out    = Output(Vec(coreWidth, new MicroOp()))
  val dec_vec_illegal = Output(Vec(coreWidth, Bool()))
  // The decode-computed VL of a front-end-only vsetivli and its qualifier.
  // VecPipeline must register these through the decode-to-ren2 stage and
  // qualify them there by dis_fire(w) -- running them a cycle ahead of the
  // uOP is the shape of the M1 free-list double-free.
  val dec_vl_imm       = Output(Vec(coreWidth, UInt(vecVLSz.W)))
  val dec_vl_imm_valid = Output(Vec(coreWidth, Bool()))

  // ---- Pass-throughs that exist only to reach vcfg. This module reads none
  // of these; each is wired straight to the correspondingly named vcfg port.
  val ren_br_tags    = Input(Vec(coreWidth + 1, Valid(UInt(brTagSz.W))))
  val ren_br_vconfig = Input(Vec(coreWidth + 1, new VType))
  val brupdate       = Input(new BrUpdateInfo)
  val rollback       = Input(Bool())
  val com_valids     = Input(Vec(coreWidth, Bool()))
  val com_is_vset    = Input(Vec(coreWidth, Bool()))
  val com_vtype      = Input(Vec(coreWidth, new VType))
  val csr_vtype      = Input(new VType)
  val rob_empty      = Input(Bool())

  // Deliberately absent, per the nlhdl source's ports section: any
  // vtype/vl output other than dec_vl_imm, any execute-time vtype write
  // port, any VRF port, any element-cursor or nOP.v-scoped port, and any
  // stall/ready/busy.
}

/**
 * VecDecode ("vdec") -- see the file header for the full design rationale.
 * Instantiated once by VecPipeline.
 */
class VecDecode(implicit p: Parameters) extends BoomModule
{
  val io = IO(new VecDecodeIO())

  val arith = Module(new VDecode())
  val ls    = Module(new VLSDecode())
  val vset  = Module(new VsetDecode())
  val vcfg  = Module(new VConfigUnit())

  // ==========================================================================
  // ---- vcfg: whole-vector pass-throughs (this module reads none of these) --
  // ==========================================================================
  vcfg.io.ren_br_tags    := io.ren_br_tags
  vcfg.io.ren_br_vconfig := io.ren_br_vconfig
  vcfg.io.brupdate       := io.brupdate
  vcfg.io.rollback       := io.rollback
  vcfg.io.com_valids     := io.com_valids
  vcfg.io.com_is_vset    := io.com_is_vset
  vcfg.io.com_vtype      := io.com_vtype
  vcfg.io.csr_vtype      := io.csr_vtype
  vcfg.io.rob_empty      := io.rob_empty
  // BOTH: `dec_valids` gates VConfigUnit's OUTPUT prefix scan, `dec_fire` gates
  // the one that advances the mirror register. See the "TWO PREFIX SCANS" note
  // in VConfigUnit part 3 -- driving the outputs from `dec_fire` closes a
  // combinational loop through BoomCore's decode-stall logic.
  vcfg.io.dec_valids     := io.dec_valids
  vcfg.io.dec_fire       := io.dec_fire

  for (w <- 0 until coreWidth) {

    // ========================================================================
    // ---- ls: feed VLSDecode's per-lane descriptor ----
    // ========================================================================
    ls.io.lanes(w).valid := io.dec_valids(w)
    ls.io.lanes(w).inst  := io.dec_insns(w)
    ls.io.lanes(w).uop   := io.dec_uops_in(w)
    val desc = ls.io.lanes(w).desc

    // ========================================================================
    // ---- 1. Recognition: the three-way opcode split ----
    // ========================================================================
    //@req-spec-decode.a3
    // RVV 1.0 opcodes reach exactly three of the four children, partitioned by
    // opcode so no instruction is claimed by two of them. Each child performs
    // its own recognition from the instruction word; this module owns only the
    // UNION, which gates part 2's zeroing, part 5's memory field writes and
    // the trace.
    val isArithLane = arith.io.is_arith(w)
    val isMemLane   = desc.is_vls
    val isVsetLane  = vset.io.dec_is_vset(w)
    val rvvRecognized = isArithLane || isMemLane || isVsetLane
    assert(!io.dec_valids(w) || PopCount(Seq(isArithLane, isMemLane, isVsetLane)) <= 1.U,
      "VecDecode: more than one of {arith, ls, vset} recognized the same lane")
    // `rvv_recognized` is NOT DecodeUnit's `v_legal` gate (mstatus.VS=Off is
    // rocket's CSRFile and is evaluated nowhere in this node) -- it only says
    // which of the three decoders owns the lane.

    // ========================================================================
    // ---- arith (VDecode): feed vtype_in (seam b) and ls_in (seam a) ----
    // ========================================================================
    arith.io.inst(w)   := io.dec_insns(w)
    arith.io.valid(w)  := io.dec_valids(w)
    arith.io.uop_in(w) := io.dec_uops_in(w)
    // seam (b): vcfg -> arith. Self-EXCLUSIVE on a consumer lane, the same
    // wire the uOP snapshot (part 4) comes from, so arith's derived EMUL and
    // the vtype the coprocessor is handed cannot disagree.
    arith.io.vtype_in(w) := vcfg.io.dec_vconfig(w)
    // seam (a): ls -> arith. arith needs all five to compute is_shared as
    // is_mem && !is_whole_reg && nf =/= 0. Note this real VDecode declares
    // is_load/is_store on VDecodeLsInfo but never actually reads either one
    // (only is_mem/is_whole_reg/nf feed is_shared) -- still driven, since they
    // are real input ports this module must connect.
    arith.io.ls_in(w).is_mem       := desc.is_vls
    arith.io.ls_in(w).is_load      := desc.is_vls && !desc.is_store
    arith.io.ls_in(w).is_store     := desc.is_store
    arith.io.ls_in(w).is_whole_reg := desc.is_whole_reg
    arith.io.ls_in(w).nf           := desc.nf

    // ========================================================================
    // ---- vset (VsetDecode): feed dec_vconfig (seam c) and prev_vtype ----
    // ========================================================================
    vset.io.dec_valid(w) := io.dec_valids(w)
    vset.io.inst(w)      := io.dec_insns(w)
    vset.io.uop_in(w)    := io.dec_uops_in(w)
    // seam (c): vcfg -> vset. Self-INCLUSIVE on a vset lane -- what makes a
    // vset's own snapshot its NEW vtype.
    vset.io.dec_vconfig(w) := vcfg.io.dec_vconfig(w)
    // seam (c), the prefix's INPUT side: self-EXCLUSIVE on every lane
    // (VConfigUnit's real `dec_prev_vconfig` output tap, at lane 0 the raw
    // `vcfg_mirror` register). Wired for EVERY lane, including lane 0 -- no
    // reconstruction as `dec_vconfig(w - 1)`, which has no w==0 case and is
    // explicitly forbidden now that the real port exists. This retires the
    // previous generation's `vset.io.prev_vtype(0) := DontCare`.
    vset.io.prev_vtype(w) := vcfg.io.dec_prev_vconfig(w)

    // ========================================================================
    // ---- vcfg: feed the vset-shape fields (seam d), dec_uses_vtype, and the
    //      trace-tagging identifiers (dec_ftq_idx/dec_pc_lob) ----
    // ========================================================================
    // seam (d): vset -> vcfg, by name.
    vcfg.io.dec_is_vset(w)      := vset.io.dec_is_vset(w)
    // VConfigUnit's dec_vtype_imm is sized xLen; VsetDecode's raw field is 11
    // bits (the wider of the two immediate vtype fields). Zero-extend at this
    // boundary -- a width bridge, not a functional disagreement, and the same
    // `.pad(xLen)` idiom VsetDecode itself uses internally for its own
    // keep-VL VLMAX comparison.
    vcfg.io.dec_vtype_imm(w)    := vset.io.dec_vtype_imm(w).pad(xLen)
    vcfg.io.dec_vtype_is_imm(w) := vset.io.dec_vtype_is_imm(w)
    vcfg.io.dec_is_vsetivli(w)  := vset.io.is_vsetivli(w)
    vcfg.io.dec_avl_imm(w)      := vset.io.dec_avl_imm(w)
    // New ports, trace-tagging only: this lane's MicroOp.ftq_idx/pc_lob,
    // straight off the merge base. No functional logic anywhere in this file
    // reads either back -- VConfigUnit consumes them only to tag its own
    // decode-lane-synchronous mirror-update trace line.
    vcfg.io.dec_ftq_idx(w) := io.dec_uops_in(w).ftq_idx
    vcfg.io.dec_pc_lob(w)  := io.dec_uops_in(w).pc_lob

    // `is_whole_reg_move` (vmv<n>r.v: OP-V, funct3=OPIVI, funct6=0b100111) is
    // re-derived here as a duplicated comparator (not a duplicated decoder --
    // VDecode computes the identical predicate internally for its own
    // vill_trap and does not export it; flagged in the authoring report as
    // the right long-term fix being an added VDecode output).
    val isWholeRegMove = (io.dec_insns(w)(6, 0) === 0x57.U) &&
                         (io.dec_insns(w)(14, 12) === 3.U) &&    // OPIVI
                         (io.dec_insns(w)(31, 26) === 0x27.U)    // funct6
    vcfg.io.dec_uses_vtype(w) := (isArithLane && !isWholeRegMove) ||
                                 (desc.is_vls && !desc.is_whole_reg && !desc.is_mask)

    // ========================================================================
    // ---- The memory-side EMUL derivation (this module's own, part 5) -------
    // ========================================================================
    // Evaluated for every lane (cheap combinational, in PARALLEL with arith's
    // own EMUL derivation -- both read the same dec_vconfig(w)); only
    // consumed under isMemLane / for the illegal aggregation below.
    val vtypeInfo = VtypeTable.decode(vcfg.io.dec_vconfig(w).asUInt)
    //@req-spec-decode.a3
    // The destination group size, the one quantity in the descriptor path
    // that is not a bit-slice:
    //   - whole-register: v_emul := desc.nregs, from the instruction's own
    //     NREG field, NOT vtype (stays decodable while the mirror is
    //     poisoned);
    //   - mask (vlm.v/vsm.v): v_emul := 1, one byte-granular register;
    //   - indexed: v_emul := LMUL. `VtypeTable.emul(info, info.vsew)` as
    //     literally specified is not callable -- SPEC DEFECT (reported, not
    //     resolved): VtypeTable's own VtypeInfo digest drops vsew entirely
    //     (VConfigUnit's file header flags the identical gap independently).
    //     Resolved by using `vtypeInfo.emul` directly: VtypeTable.decode's
    //     own doc defines that field as exactly emulFromVLMax(vlmax, vsew),
    //     i.e. LMUL -- the value the literal formula would have produced --
    //     so no re-derivation or invented field is needed.
    //   - otherwise: v_emul := VtypeTable.emul(info, desc.eew), i.e.
    //     LMUL*EEW/SEW, clamped to at least 1 by that function.
    val emulPerField = Mux(desc.is_whole_reg, desc.nregs,
                       Mux(desc.is_mask,      1.U,
                       Mux(desc.is_indexed,   vtypeInfo.emul,
                                              VtypeTable.emul(vtypeInfo, desc.eew))))
    // Segmented access: NFIELDS consecutive groups, allocated atomically.
    val segMultiplier = desc.nf +& 1.U // NFIELDS, 1..8
    val emulTotal      = Mux(desc.is_segment, emulPerField * segMultiplier, emulPerField)

    //@req-spec-decode.a3
    // RESERVED-ENCODING CHECK, this module's own: NOT clamped -- the full-
    // width emulTotal (computed above, before any truncation for the uOP
    // field) is compared directly against maxVecMembers, so an over-wide
    // result is never silently allocated at 8. The 4-bit truncation applied
    // to uopOut.v_emul below only ever matters on a lane this term already
    // marks illegal.
    //
    // VtypeTable.emul's OUT-OF-RANGE CONTRACT CHANGED: it now RETURNS 0 (never
    // an unclamped magnitude) when the derived group would exceed
    // maxVecMembers, and decode() independently returns emul=0 on a poisoned
    // (vill) vtype. `emulPerField` folds a value from that contract on the
    // indexed and "otherwise" (unit-stride/strided) legs, so an over-wide
    // per-field EMUL now reads as 0, not as some large value -- multiplying it
    // by segMultiplier would then produce emulTotal = 0 too, which is BELOW
    // maxVecMembers and would silently hide the reserved encoding. Neither the
    // whole-register leg (desc.nregs) nor the mask leg (1.U) can legitimately
    // be 0, so checking emulPerField === 0.U is unambiguous: it never fires on
    // a leg that did not consult VtypeTable.emul/decode.
    val memEmulIllegal = isMemLane && (emulPerField === 0.U || emulTotal > maxVecMembers.U)

    // ========================================================================
    // ---- The merge base, part 2 (single zeroing) and parts 3/4/5 (merge) --
    // ========================================================================
    val uopOut = WireDefault(io.dec_uops_in(w))

    // ---- Part 4: vconfig, THE UNCONDITIONAL WRITE, every lane, no qualifier.
    val vTypeWithKeepVl = WireDefault(vcfg.io.dec_vconfig(w))
    // vset.keep_vl_illegal ORed into vill here so the uOP that must trap
    // carries its own poison regardless of the mirror-side defect above.
    vTypeWithKeepVl.vill := vcfg.io.dec_vconfig(w).vill || vset.io.keep_vl_illegal(w)
    uopOut.vconfig.get := vTypeWithKeepVl

    when (rvvRecognized) {
      // ---- Part 2: THE SINGLE ZEROING, before any child's SET is applied.
      // Baseline DecodeUnit has no table entry for an RVV opcode, so the
      // scalar routing bits it produced for this lane are meaningless.
      uopOut.iq_type := VecInit(Seq.fill(IQ_SZ)(false.B))
      uopOut.fu_code := VecInit(Seq.fill(FC_SZ)(false.B))
      // The zeroing rule extends to the three source-use bits and v_is_masked
      // (part 5's closing note): the container is the only place that can
      // zero them, since no child sees the lanes it did not claim.
      uopOut.v_uses_vs1.get  := false.B
      uopOut.v_uses_vs2.get  := false.B
      uopOut.v_uses_vs3.get  := false.B
      uopOut.v_is_masked.get := false.B
      // nOP.v-scoped fields driven to ZERO on every recognized lane (part 3's
      // closing note), not left undriven, so a stage misreading them off an
      // OP.v gets a consistently wrong answer.
      uopOut.v_split_first.get        := false.B
      uopOut.v_split_last.get         := false.B
      uopOut.v_split_idx.get          := 0.U
      uopOut.v_split_total.get        := 0.U
      uopOut.v_split_dst_prn.get      := 0.U
      uopOut.v_split_dst_byte_off.get := 0.U
      uopOut.v_elem_cursor.get.elem_next  := 0.U
      uopOut.v_elem_cursor.get.elem_done  := 0.U
      uopOut.v_elem_cursor.get.fault_elem := 0.U

      // `is_shared`/IQ_V_ALU: arith computes this for EVERY lane regardless of
      // which decoder claimed it (is_mem comes from ls_in, fed above) -- the
      // one field two owners (arith and this module, on a segmented memory
      // lane) legitimately both target, and it is a single SET sourced from
      // arith alone, never a lane-conditional mux.
      uopOut.is_shared.get     := arith.io.uop_out(w).is_shared.get
      uopOut.iq_type(IQ_V_ALU) := arith.io.uop_out(w).iq_type(IQ_V_ALU)

      // ---- Part 3, arithmetic-lane fields, from arith.uop_out(w) ----
      when (isArithLane) {
        uopOut.is_vec.get     := arith.io.uop_out(w).is_vec.get
        uopOut.dst_rtype       := arith.io.uop_out(w).dst_rtype
        uopOut.lvd.get         := arith.io.uop_out(w).lvd.get
        uopOut.lvs1.get        := arith.io.uop_out(w).lvs1.get
        uopOut.lvs2.get        := arith.io.uop_out(w).lvs2.get
        uopOut.lvs3.get        := arith.io.uop_out(w).lvs3.get
        uopOut.lvm.get         := arith.io.uop_out(w).lvm.get
        uopOut.ldst            := arith.io.uop_out(w).ldst
        uopOut.lrs1            := arith.io.uop_out(w).lrs1
        uopOut.lrs1_rtype      := arith.io.uop_out(w).lrs1_rtype
        uopOut.lrs2_rtype      := arith.io.uop_out(w).lrs2_rtype
        uopOut.v_emul.get      := arith.io.uop_out(w).v_emul.get
        uopOut.v_uses_vs1.get  := arith.io.uop_out(w).v_uses_vs1.get
        uopOut.v_uses_vs2.get  := arith.io.uop_out(w).v_uses_vs2.get
        uopOut.v_uses_vs3.get  := arith.io.uop_out(w).v_uses_vs3.get
        uopOut.v_is_masked.get := arith.io.uop_out(w).v_is_masked.get
        uopOut.fu_code(FC_ALU) := true.B
      }

      // ---- Part 3, vset-lane fields, from vset.uop_out(w) ----
      when (isVsetLane) {
        uopOut.dst_rtype          := vset.io.uop_out(w).dst_rtype
        uopOut.is_vl_producer.get := vset.io.uop_out(w).is_vl_producer.get
        // The real VsetDecode never writes lrs1/lrs2 VALUE (only their
        // rtypes) -- baseline DecodeUnit already placed the raw specifier
        // bits there for every instruction, vset or not. Copying them here is
        // therefore a provable no-op (vset.uop_out(w).lrs1/lrs2 always equal
        // dec_uops_in(w).lrs1/lrs2), kept only to match the spec's literal
        // field list; it changes nothing.
        uopOut.lrs1               := vset.io.uop_out(w).lrs1
        uopOut.lrs2               := vset.io.uop_out(w).lrs2
        uopOut.lrs1_rtype         := vset.io.uop_out(w).lrs1_rtype
        uopOut.lrs2_rtype         := vset.io.uop_out(w).lrs2_rtype
        uopOut.is_unique          := vset.io.uop_out(w).is_unique
        uopOut.flush_on_commit    := vset.io.uop_out(w).flush_on_commit
        uopOut.iq_type(IQ_ALU)    := vset.io.uop_out(w).iq_type(IQ_ALU)
        uopOut.fu_code(FC_ALU)    := vset.io.uop_out(w).fu_code(FC_ALU)
        uopOut.fcn_op             := vset.io.uop_out(w).fcn_op
        uopOut.fcn_dw             := vset.io.uop_out(w).fcn_dw
        uopOut.op1_sel            := vset.io.uop_out(w).op1_sel
        uopOut.op2_sel            := vset.io.uop_out(w).op2_sel
        uopOut.imm_sel            := vset.io.uop_out(w).imm_sel
        uopOut.imm_rename         := vset.io.uop_out(w).imm_rename
        uopOut.csr_cmd            := vset.io.uop_out(w).csr_cmd
        uopOut.frs3_en            := vset.io.uop_out(w).frs3_en
        uopOut.uses_ldq           := vset.io.uop_out(w).uses_ldq
        uopOut.uses_stq           := vset.io.uop_out(w).uses_stq

        // The decode-computed VL, carried to the ALU so that a `vsetivli` whose
        // `rd != x0` writes the SAME value into `rd` that the VL RF receives.
        // Sourced from VConfigUnit's single computation (`vcfg.io.dec_vl_imm`),
        // NOT recomputed, which is the whole point of the MicroOp field -- see
        // the field's own note in micro-op.scala.
        //
        // Taken UNCONDITIONALLY on a vset lane rather than qualified by
        // `dec_vl_imm.valid`: `valid` means "this lane is a vsetivli", which is
        // exactly the condition under which ALUUnit reads the field, and
        // ALUUnit's own discriminator (`lrs1_rtype === RT_X`) is derived from the
        // same decode. A second qualifier here could only disagree with it.
        uopOut.v_vl_imm.get       := vcfg.io.dec_vl_imm(w).bits
      }

      // ========================================================================
      // ---- Part 5, memory-lane fields, from ls.lanes(w).desc (this module's
      //      own job -- VLSDecode emits a descriptor and NO uOP) ------------
      // ========================================================================
      when (isMemLane) {
        // Class carriage, straight across, one field per descriptor flag.
        uopOut.v_mop.get            := desc.mop
        uopOut.v_is_unit_stride.get := desc.is_unit_stride
        uopOut.v_is_strided.get     := desc.is_strided
        uopOut.v_is_indexed.get     := desc.is_indexed
        uopOut.v_is_segment.get     := desc.is_segment
        uopOut.v_is_whole_reg.get   := desc.is_whole_reg
        uopOut.v_is_mask.get        := desc.is_mask
        uopOut.v_is_ff.get          := desc.is_ff

        // `nf` is NFIELDS-1 for a segmented access and NREG-1 for
        // vl<n>re<eew> -- qualify it, or vl8re64 presents as an 8-field
        // segmented access to any reader of v_seg_nf.
        uopOut.v_seg_nf.get := Mux(desc.is_segment, desc.nf, 0.U)

        // RVV's vm bit is 1 for UNMASKED, so v_is_masked is its COMPLEMENT.
        // The two non-maskable unit-stride forms are forced false.
        uopOut.v_is_masked.get := !desc.vm && !desc.is_whole_reg && !desc.is_mask

        // The two element widths. For an indexed access `desc.eew` names the
        // INDEX, not the data -- the data width is vtype.vsew. vsew's low two
        // bits ARE the EEW code for supported SEW<=64, the same convention
        // VtypeTable.emulFromVLMax uses internally (eewIn(1,0)).
        uopOut.v_eew.get     := Mux(desc.eew_is_index, vcfg.io.dec_vconfig(w).vsew(1, 0), desc.eew)
        uopOut.v_idx_eew.get := desc.eew // meaningful only when v_is_indexed

        // Source-use bits, straight from the descriptor on a memory lane.
        uopOut.v_uses_vs1.get := desc.uses_vs1
        uopOut.v_uses_vs2.get := desc.uses_vs2
        uopOut.v_uses_vs3.get := desc.uses_vs3

        // The destination group size, computed above (memEmulIllegal /
        // emulTotal); truncated to v_emul's width -- harmless when illegal,
        // since the illegal check itself used the untruncated magnitude.
        uopOut.v_emul.get := emulTotal(log2Ceil(maxVecMembers), 0)

        // Routing and the queues: a segmented access carries IQ_V_ALU from
        // arith (set above, unconditionally) alongside these, taking two
        // issue slots while occupying ONE ROB entry.
        uopOut.is_vec.get          := true.B
        uopOut.iq_type(IQ_V_LOAD)  := !desc.is_store
        uopOut.iq_type(IQ_V_STORE) := desc.is_store
        uopOut.uses_ldq            := !desc.is_store
        uopOut.uses_stq            := desc.is_store

        // Register specifiers, extracted here because VLSDecode writes no
        // uOP and VDecode does not claim these lanes.
        when (!desc.is_store) {
          uopOut.lvd.get  := io.dec_insns(w)(11, 7) // rd: destination group
          uopOut.dst_rtype := RT_VEC
        } .otherwise {
          // Store data group is an explicitly ENCODED third source (pvs3),
          // never stale_pvdest.
          uopOut.lvs3.get := io.dec_insns(w)(11, 7)
          uopOut.dst_rtype := RT_X
        }

        // Base address is always the integer rs1. x0 conversion MANDATORY on
        // both integer sources (rename-stage.scala:109's assert), mirroring
        // the scalar decoder (decode.scala:505).
        val rs1f = io.dec_insns(w)(19, 15)
        uopOut.lrs1       := rs1f
        uopOut.lrs1_rtype := Mux(rs1f === 0.U, RT_ZERO, RT_FIX)

        // inst(24,20) is three different things by class.
        val rs2f = io.dec_insns(w)(24, 20)
        when (desc.is_strided) {
          uopOut.lrs2       := rs2f
          uopOut.lrs2_rtype := Mux(rs2f === 0.U, RT_ZERO, RT_FIX)
        } .elsewhen (desc.is_indexed) {
          uopOut.lvs2.get   := rs2f
          uopOut.lrs2_rtype := RT_X
        } .otherwise {
          // unit-stride: the field is umop, no register named at all.
          uopOut.lrs2_rtype := RT_X
        }

        uopOut.lvm.get := 0.U // mask is architecturally v0

        // vleff IS a VL producer (writes its trimmed count to the VL RF and
        // wakes pvl); this module is the writer of is_vl_producer for memory
        // lanes, settling in favour of the corrected VLSDecode comment,
        // loadstore.rst and spec-lsu.g6/g7.
        uopOut.is_vl_producer.get := desc.is_ff
      }
    }

    io.dec_uops_out(w) := uopOut

    // ==========================================================================
    // ---- 6. dec_vec_illegal: the aggregation, exactly four terms ----
    // ==========================================================================
    io.dec_vec_illegal(w) := io.dec_valids(w) && (
      ls.io.lanes(w).illegal ||       // reserved vector memory encoding
      arith.io.vill_trap(w) ||        // poisoned mirror on a vtype-dependent
                                       // arithmetic op, or arithmetic EMUL
                                       // overflow
      vcfg.io.dec_vtype_illegal(w) || // poisoned mirror reaching a
                                       // dec_uses_vtype lane
      memEmulIllegal)                 // this module's memory-side EMUL bound
    // vset.keep_vl_illegal is deliberately NOT a term here: the reserved
    // keep-VL encoding SETS vill (part 4), it does not raise
    // illegal-instruction; the trap arrives later on the first younger
    // vtype-dependent uOP. mstatus.VS=Off is rocket's CSRFile and is not
    // evaluated here.

    // ==========================================================================
    // ---- The fifth seam: dec_vl_imm_valid ----
    // ==========================================================================
    // A vsetivli with rd != x0 is NOT front-end only (rd needs an integer-RF
    // write the front end has no port for), so qualifying by dec_is_vsetivli
    // alone would give that encoding two VL writers on two unarbitrated ports.
    io.dec_vl_imm(w)       := vcfg.io.dec_vl_imm(w).bits
    io.dec_vl_imm_valid(w) := vcfg.io.dec_vl_imm(w).valid && vset.io.frontend_only(w)

    // ==========================================================================
    // ---- 8. Assertions (checks, not behaviour: deletable without changing
    //         the emitted datapath) ----
    // ==========================================================================
    assert(!(io.dec_valids(w) && rvvRecognized && !vset.io.frontend_only(w)) ||
      uopOut.iq_type.asUInt =/= 0.U,
      "VecDecode: a recognized, non-frontend-only lane must route to some issue queue")
    assert(!io.dec_valids(w) || !uopOut.is_shared.get ||
      (uopOut.iq_type(IQ_V_ALU) && (uopOut.iq_type(IQ_V_LOAD) || uopOut.iq_type(IQ_V_STORE))),
      "VecDecode: is_shared must carry both IQ_V_ALU and one of IQ_V_LOAD/IQ_V_STORE")
    assert(!(io.dec_valids(w) && uopOut.is_vec.get && !io.dec_vec_illegal(w)) ||
      (uopOut.v_emul.get >= 1.U && uopOut.v_emul.get <= maxVecMembers.U),
      "VecDecode: v_emul out of [1, maxVecMembers] on a legal is_vec lane")
    assert(!(io.dec_valids(w) && rvvRecognized) ||
      (!uopOut.v_split_first.get && !uopOut.v_split_last.get &&
       uopOut.v_split_idx.get === 0.U && uopOut.v_split_total.get === 0.U &&
       uopOut.v_split_dst_prn.get === 0.U && uopOut.v_split_dst_byte_off.get === 0.U &&
       uopOut.v_elem_cursor.get.elem_next === 0.U &&
       uopOut.v_elem_cursor.get.elem_done === 0.U &&
       uopOut.v_elem_cursor.get.fault_elem === 0.U),
      "VecDecode: nOP.v-scoped cursor fields must be zero on every recognized lane")

    // ==========================================================================
    // ---- 8 (cont). Trace, guarded per ground rule 11. traceDecode's
    //      ftq_idx/pc_lob variant is mandatory here: the ROB allocates at
    //      DISPATCH, so no decode-stage caller has a rob_idx. ----
    // ==========================================================================
    when (io.dec_valids(w) && rvvRecognized) {
      VecTrace.traceDecode(
        "VecDecode", "merge", io.dec_uops_in(w).ftq_idx, io.dec_uops_in(w).pc_lob,
        Seq(
          ("decoder",   Mux(isArithLane, 0.U(2.W), Mux(isMemLane, 1.U(2.W), 2.U(2.W)))),
          ("iq_type",   uopOut.iq_type.asUInt),
          ("v_emul",    uopOut.v_emul.get),
          ("v_eew",     uopOut.v_eew.get),
          ("is_shared", uopOut.is_shared.get)))
    }
    when (io.dec_vec_illegal(w)) {
      VecTrace.traceDecode(
        "VecDecode", "illegal", io.dec_uops_in(w).ftq_idx, io.dec_uops_in(w).pc_lob,
        Seq(
          ("mem_illegal",      ls.io.lanes(w).illegal),
          ("arith_illegal",    arith.io.vill_trap(w)),
          ("vtype_illegal",    vcfg.io.dec_vtype_illegal(w)),
          ("mem_emul_illegal", memEmulIllegal)))
    }
  }
}

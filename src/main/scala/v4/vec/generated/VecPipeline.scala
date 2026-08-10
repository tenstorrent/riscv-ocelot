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
import freechips.rocketchip.rocket.VType

import boom.v4.common._
import boom.v4.exu.{BrUpdateInfo, ExeUnitResp, IssueParams}
import boom.v4.vec.generated.decode.VecDecode
import boom.v4.vec.generated.rename.VecRenameSpace
import boom.v4.vec.generated.issue.VecIssueUnit
import boom.v4.vec.generated.regfile.{VecRegFile, VlRegFile}

// GENERATED from src/main/nlhdl/vec/VecPipeline.nlhdl.scala. Do not hand-edit;
// regenerate via the nlhdl gen-rtl flow instead.
//
// VecPipeline -- the whole Caracal vector subsystem behind ONE bundle
// (VecPipelineIO): decode, two rename spaces, three issue queues, two
// register files, the vector LSU and the coprocessor host. It owns no
// datapath of its own -- what it owns is TOPOLOGY and the design-wide
// rulings that no single child could settle (see the nlhdl source's
// "===>" notes for the eleven cross-node rulings this file is the single
// answer to).
//
// STEP D2 STAGING (plan v2, docs_caracal/caracal-milestone-plan-v2.md):
// `VecLsu` ("vlsu", step E7) and `VecCiiHost` ("cii", Phase F) DO NOT EXIST
// YET and are NOT instantiated here -- the plan's D2 row is explicit:
// "container only -- LSU and CII tied off". Every seam those two children
// would have owned is driven to a documented default inside ONE
// contiguous block below, bannered
// "D2 STAGING TIE-OFF: vlsu / cii ABSENT UNTIL E7 / PHASE F" --
// grep that banner to find the whole set. Nothing a PRESENT child owns
// (vdec, vec_rename, vl_rename, the three issue queues, vrf, vlrf) is
// tied off anywhere in this file: those seven are wired natively and
// completely. The immediate purpose of this staging is that a vector
// config ELABORATES and that vsetvli/vsetivli/vsetvl work end to end --
// nothing in the vset path touches vlsu or cii.
//
// Elaborated only when `usingRVV` is true (a Scala Boolean of
// BoomCoreParams, never rocket's `usingVector`) -- by virtue of the
// PARENT (BoomCore) only instantiating this module under that gate. This
// module performs no internal `usingRVV` conditional of its own; its
// ABSENCE from a vectors-off build is the gate, checked once below.
//
// Governing spec anchors: overview.rst `boom-relationship`,
// `caracal-pipeline`; glossary.rst `glossary-terms`; execution.rst
// `execution-pipelines`, `vector-execution`; midcore.rst `rename-stage`,
// `cii-shared-mapping`, `vl-vtype-rename`, `regfiles-bypass`, `old-vd`,
// `group-done-wb`, `vrf-ports`, `spec-wakeups`; issue.rst `cii-shared-sched`,
// `shared-store-chain`, `issue-sched-stage`, `issue-vl-delivery`;
// frontend.rst `vector-rvv-decode`, `vset-dual-dest`, `vl-delivery`;
// cii.rst `cii-mem-order`.
//
// SPEC DEFECT, FOUND HERE AND FIXED AT THE SEAM -- `child_rebusys` /
// `squash_grant`. The nlhdl ports section ("Three interface names with no
// declaration, bound here") calls for an `IntWakeupBus` AGGREGATE carrying
// `wakeups` / `child_rebusys: UInt(aluWidth.W)` / `squash_grant: Bool`, and
// this file's "amendments this file requires of others" (item 1) asks for it
// to be declared in VecBundles. D2's first pass declared only the `Vec`, on
// the (correct, as far as it went) reasoning that a Caracal copy of BOOM's
// `Wakeup` would fork it -- and silently dropped the other two, even though
// every `VecIssueUnit` instance declares both as unconditional inputs.
//
// This file's first generation tied them to `0.U`/`false.B` and called them
// "safe, no-effect defaults". THAT WAS WRONG, AND IT WAS THE DANGEROUS KIND OF
// WRONG: `child_rebusys` is the RETRACTION half of BOOM's speculative wakeup.
// VecIssueUnit uses it to re-mark a slot busy when a speculatively-woken
// scalar `.vx`/`.vf` feeder's parent load misses. Held at zero, the retraction
// never arrives, and the OP.v issues against a stale GPR -- no width error, no
// assertion, silent data corruption. A tie-off is not conservative when the
// signal's whole job is to say "take that wakeup back".
//
// Both terms ARE BoomCore-internal (`alu_exe_units.map(_.io_squash_iss)
// .reduce(_||_) || ...`) and genuinely not computable here -- which is an
// argument for a seam member, not for a default. `VecPipelineIO` now declares
// `int_child_rebusys` and `int_squash_grant`, BoomCore drives them, and this
// container routes them to all three queues. See the wiring below, next to the
// three queues' scalar-network fan-out.
class VecPipeline(val numIntWakeupPorts: Int, val numFpWakeupPorts: Int, val numIrfWritePorts: Int)
  (implicit p: Parameters) extends BoomModule
{
  //@req-spec-core.a2
  //@req-spec-core.a3
  // Caracal supports RVV 1.0 with out-of-order vector load/store, and the
  // RVV state and datapath that implements it is exactly this subtree:
  // nothing vector-visible exists outside it except the architectural CSR
  // cells (rocket's CSRFile, reached only through io.csr_vector/csr_frm/
  // csr_vs_dirty) and the deltas to BOOM's own shared files. Discharged by
  // instance existence, checked once: require usingRVV at the top of this
  // module's elaboration and nowhere inside it.
  require(usingRVV, "VecPipeline: elaborated only under usingRVV (never rocket's usingVector)")

  val io = IO(new VecPipelineIO(numIntWakeupPorts, numFpWakeupPorts, numIrfWritePorts))

  // ===========================================================================
  // ---- PART 1. Instantiation map -- nine children named, seven built ----
  // ===========================================================================
  //
  //@req-spec-core.a5
  //@req-spec-core.a6
  //@req-spec-core.d5
  // BOOM v4's out-of-order pipeline stays INTACT: every wire below into
  // `vdec`/`vec_rename`/`vl_rename` comes straight off this module's own
  // `io` (BoomCore's decode bundle, its REGISTERED ren2 bundle, its
  // dis_fire/brupdate/PNR/flush/commit ports) -- no second frontend, no
  // second ROB, no second commit point and no second scheduling stage is
  // declared anywhere in this file.
  val vdec = Module(new VecDecode())

  val vec_rename = Module(new VecRenameSpace(
    plWidth         = coreWidth,
    numArchRegs     = 32,
    maxGroupSize    = maxVecMembers,
    numPhysRegs     = numVecPhysRegs,
    numWbPorts      = vectorParams.numVecWbPorts,
    freeDiscipline  = "stale_group",
    wakeupKind      = "group_done",
    hasRenameWrite  = false,
    exportMemberRdy = true))

  val vl_rename = Module(new VecRenameSpace(
    plWidth         = coreWidth,
    numArchRegs     = 1,
    maxGroupSize    = 1,
    numPhysRegs     = numVlPhysRegs,
    numWbPorts      = numVlWakeupPorts,
    freeDiscipline  = "committed_ptr",
    wakeupKind      = "ready_bit",
    hasRenameWrite  = true,
    exportMemberRdy = false))

  //@req-spec-core.i3
  //@req-spec-cii.d14
  //@req-spec-issue.e1
  //@req-spec-issue.e3
  // THREE STREAMS, visible in the instance map: vector arithmetic /
  // reduction / permutation through `iq_v_alu` (to the coprocessor, once
  // Phase F lands); vector loads through `iq_v_load` (to the vector LSU's
  // load direction, once E7 lands); vector stores through `iq_v_store`
  // (to the LSU's store direction). Caracal adds exactly these three split
  // issue queues; the four scalar queues are untouched and not visible
  // here. All three are ONE parameterized `VecIssueUnit` definition,
  // differing in `iqType`/`pnrGate`/`numFpWakeupPorts` and in ISSUE WIDTH.
  //
  // ===> `vecIssueGrantWidth` SCALES THE LOAD AND STORE QUEUES ONLY;
  //      `iq_v_alu` IS ALWAYS 1. The first generation of this file bound all
  //      three from `vecIssueGrantWidth` and then `require`d that shared value
  //      to be 1 -- which made every wide tier un-elaboratable, because
  //      `WithLargeBoomsVector` and `WithMegaBoomsVector` both set
  //      `vecIssueGrantWidth = 2` (in step with `lsuWidth = 2` and
  //      `dcacheArbiterMode = "dual-dynamic"`). Gate (c) caught it as
  //      "iq_v_alu's issueWidth ... must be 1" on MegaBoomV4VectorConfig.
  //
  //      `VecIssueUnit.nlhdl.scala` states the rule outright: "if a config
  //      raises `vecIssueGrantWidth` it must raise it for the load/store queues
  //      only." The reason is the frozen SV contract, not a tuning choice:
  //      `tt_cii_interface.sv` gives the issue channel exactly ONE valid per
  //      beat, so the coprocessor accepts one issue per cycle no matter how
  //      wide the LSU gets. A second CII grant has nowhere to go.
  //
  //      So the constraint is now STRUCTURAL -- `iq_v_alu` is constructed with
  //      issueWidth 1 -- and the `require` that remains is the one that can
  //      actually be violated by a config: a grant width below 1 would leave
  //      the load/store queues unable to issue at all.
  require(vectorParams.vecIssueGrantWidth >= 1,
    s"VecPipeline: vectorParams.vecIssueGrantWidth (${vectorParams.vecIssueGrantWidth}) must be " +
    ">= 1 -- it is the load/store queues' grants per cycle. iq_v_alu is always 1 by construction " +
    "(the CII issue channel has one valid per beat), so this parameter never applies to it.")

  private def vecIssueParams(iqt: Int, issueWidth: Int): IssueParams = IssueParams(
    dispatchWidth = coreWidth,
    issueWidth    = issueWidth,
    numEntries    = vectorParams.vecIssueEntries,
    iqType        = iqt)

  val iq_v_load = Module(new VecIssueUnit(
    params            = vecIssueParams(IQ_V_LOAD,  vectorParams.vecIssueGrantWidth),
    numIntWakeupPorts = numIntWakeupPorts,
    pnrGate           = false,
    numFpWakeupPorts  = 0,
    numVecWbPorts     = vectorParams.numVecWbPorts))

  val iq_v_store = Module(new VecIssueUnit(
    params            = vecIssueParams(IQ_V_STORE, vectorParams.vecIssueGrantWidth),
    numIntWakeupPorts = numIntWakeupPorts,
    pnrGate           = false,
    numFpWakeupPorts  = 0,
    numVecWbPorts     = vectorParams.numVecWbPorts))

  val iq_v_alu = Module(new VecIssueUnit(
    params            = vecIssueParams(IQ_V_ALU,   1),
    numIntWakeupPorts = numIntWakeupPorts,
    pnrGate           = true,
    numFpWakeupPorts  = numFpWakeupPorts,
    numVecWbPorts     = vectorParams.numVecWbPorts))

  // Fixed order {IQ_V_LOAD, IQ_V_STORE, IQ_V_ALU}, matching
  // io.dis_vec_valids/io.dis_vec_ready's own lane-group ordering.
  val vecQueues = Seq(iq_v_load, iq_v_store, iq_v_alu)

  //@req-spec-vrf.i4
  // THE PORT PARTITION IS CANONICAL AND NOTHING IN THIS CONTAINER ADDS A
  // PORT: `vrf` is instantiated with its own fixed 9R port set and no
  // extra port is declared on it anywhere below.
  val vrf  = Module(new VecRegFile())
  val vlrf = Module(new VlRegFile())

  // `vlsu` (VecLsu) and `cii` (VecCiiHost) are NOT instantiated -- see the
  // file header and the D2 STAGING TIE-OFF block immediately below.
  //@req-spec-cii.j1
  //@req-spec-cii.j2
  //@req-spec-cii.j3
  // The absence is itself the strongest form of "the CII moves no memory
  // traffic of its own": with no `cii` instance in this file there is no
  // D$ port, no TLB port, no LSQ port and no member of `lsu_vec` reaching
  // a coprocessor anywhere in this subtree, by construction rather than by
  // omission.

  // ===========================================================================
  // ======== D2 STAGING TIE-OFF: vlsu / cii ABSENT UNTIL E7 / PHASE F ========
  // ===========================================================================
  //
  // Every seam below is owned by `VecLsu` ("vlsu", lands at step E7) or
  // `VecCiiHost` ("cii", lands in Phase F) and has no other driver in this
  // file. Each line states which absent child owns it. Nothing a PRESENT
  // child owns (vdec, vec_rename, vl_rename, iq_v_load/store/alu, vrf,
  // vlrf) is tied off here or anywhere else in this file.

  // (spec-issue.d1/d14's rob_pnr_idx/rob_head_idx forwarding is tagged at
  // the pnrGate forwarding site below, not here -- those two come straight
  // from io and are NOT a tie-off.)

  // ---- 1. dis_ready's vlsu.dis_ok term: VecLsu (E7). ----
  // `dis_ready` is normally the AND of vec_rename.alloc_ok, vl_rename.alloc_ok
  // and vlsu.dis_ok (reduced over lanes requesting a vector memory
  // reservation). With vlsu absent that third term is a constant TRUE --
  // named here, not silently dropped, per decision D2/rule 5.
  val vlsuDisOkTiedTrue: Bool = true.B // VecLsu.dis_ok, owned by VecLsu (E7)

  // ---- 2. The vector (group-done) wakeup network: lanes owned by VecLsu
  //         (0, 2) and VecCiiHost (1). All three tied invalid. ----
  val vecGroupDoneNetwork = Wire(Vec(vectorParams.numVecWbPorts, Valid(new VecGroupDone)))
  for (i <- 0 until vectorParams.numVecWbPorts) {
    // lane 0 = VecLsu's LCB group-done (E7); lane 1 = VecCiiHost's
    // cii.group_done (Phase F); lane 2 = VecLsu's VecGroupCopy group-done
    // (E7). No producer exists yet for any of the three.
    vecGroupDoneNetwork(i).valid := false.B
    vecGroupDoneNetwork(i).bits  := DontCare
  }

  // ---- 3. The VL wakeup network's LSU lane (the vleff trim): VecLsu (E7).
  //         Lanes 0..aluWidth-1 (the vset writeback) are real -- see Part 6. ----
  val vlWakeupNetwork = Wire(Vec(numVlWakeupPorts, Valid(UInt(vlPregSz.W))))
  vlWakeupNetwork(aluWidth).valid := false.B // VecLsu.vl_wb (vleff trim), owned by VecLsu (E7)
  vlWakeupNetwork(aluWidth).bits  := DontCare

  // ---- 4. iq_v_alu's dynamic fu_types(0): VecCiiHost (Phase F). ----
  // Part 11: this is the ONE dynamic FU advertisement in the vector design
  // (CII issue credit + free-tag availability). With cii absent there is no
  // credit to advertise, so the lane is tied to all-false: iq_v_alu can
  // still accept dispatches (rule 4 -- the queue itself is wired natively)
  // but never grants an entry, since there is nowhere for a grant to go.
  iq_v_alu.io.fu_types(0) := VecInit(Seq.fill(FC_SZ)(false.B)) // cii.fu_types, owned by VecCiiHost (Phase F)

  // ---- 5. VecRegFile ("vrf"): EVERY read/write port is owned by vlsu/cii.
  //         Per the canonical partition (part 8): R0 load index, R1 load
  //         mask, R2 stale_pvdest, R3 store data, R4 store mask+index, W0/W1
  //         load+LCB -- all VecLsu (E7); R5-R8 CII source lanes, W2 CII
  //         writeback -- all VecCiiHost (Phase F). No present child touches
  //         a VRF port, so the whole 9R/(1+lsuWidth)W array ties invalid. ----
  for (rp <- 0 until vrf.numReadPorts) {
    vrf.io.read(rp).valid := false.B // R0-R8, owned by VecLsu (R0-R4, R2 shared) / VecCiiHost (R5-R8)
    vrf.io.read(rp).bits  := DontCare
  }
  for (wp <- 0 until vrf.numWritePorts) {
    vrf.io.write(wp).valid := false.B // W0/W1 owned by VecLsu (E7); W2 owned by VecCiiHost (Phase F)
    vrf.io.write(wp).bits  := DontCare
  }

  // ---- 6. VlRegFile ("vlrf"): the vlsu/cii-owned ports only. w_ren (rename,
  //         vl_rename), w_alu (the ALU vset writeback, Part 9) and r_commit
  //         (this container's own commit-scan) are PRESENT and wired for
  //         real elsewhere -- NOT tied off here. ----
  for (i <- 0 until vlrf.numExeReadPorts) {
    // r_exe(0) = VecLsu's load AGEN (E7); r_exe(1) = VecLsu's store AGEN
    // (E7); r_exe(2) = VecCiiHost (Phase F); any further lane at a wider
    // vecIssueGrantWidth repeats the same {load, store, cii} pattern.
    vlrf.io.r_exe(i).addr := 0.U
  }
  for (i <- 0 until vlrf.numLsuWritePorts) {
    vlrf.io.w_lsu(i).valid := false.B // the vleff completion write, owned by VecLsu (E7)
    vlrf.io.w_lsu(i).bits  := DontCare
  }

  // ---- 7. int_rf_read_req: lanes 0-3 VecLsu (E7), lane 4 VecCiiHost
  //         (Phase F). No present child reads the scalar INT RF through
  //         this seam. ----
  for (i <- 0 until io.int_rf_read_req.length) {
    io.int_rf_read_req(i).valid := false.B
    io.int_rf_read_req(i).bits  := 0.U
  }

  // ---- 8. fp_rf_read_req: the sole reader is VecCiiIssue's `.vf` operand,
  //         inside VecCiiHost (Phase F). ----
  io.fp_rf_read_req := 0.U

  // ---- 9. Scalar-dest writeback (vmv.x.s, vcpop.m, vfirst.m, vfmv.f.s):
  //         both produced entirely inside VecCiiHost (Phase F). ----
  io.int_wb.valid := false.B
  io.int_wb.bits  := DontCare
  io.fp_wb.valid  := false.B
  io.fp_wb.bits   := DontCare

  // ---- 10. Completion group-done -> ROB (part 7): lanes 0 and 2 are
  //          DERIVED from vecGroupDoneNetwork below in Part 7 (real
  //          wiring, not a tie-off, and vacuously invalid today because
  //          the network itself is tied off in item 2 above). Lane 1 is
  //          the union of cii.group_done's rob_idx and cii.clr_rob -- both
  //          owned by VecCiiHost (Phase F) -- and has no other source, so
  //          it is tied here directly. ----
  io.vec_clr_bsy(1).valid := false.B // union of cii.group_done/cii.clr_rob, owned by VecCiiHost (Phase F)
  io.vec_clr_bsy(1).bits  := DontCare

  // ---- 11. vec_rob_flags: today only lane 1 (VecCiiHost, Phase F) ever
  //          carries anything by design; lanes 0/2 (VecLsu's producers)
  //          carry no CSR side effect by design, D2-staged or not. All
  //          three tie invalid. ----
  for (i <- 0 until vectorParams.numVecClrPorts) {
    io.vec_rob_flags(i).valid := false.B // lane 1: VecCiiHost (Phase F); lanes 0/2: never produced, by design
    io.vec_rob_flags(i).bits  := DontCare
  }

  // ---- 12. vec_clr_unsafe / vec_xcpt: both entirely VecLsu's (E7) -- the
  //          LSU half's first address translation and its memory fault
  //          report, respectively. ----
  io.vec_clr_unsafe.valid := false.B // VecLsu's group-safe pulse, owned by VecLsu (E7)
  io.vec_clr_unsafe.bits  := DontCare
  io.vec_xcpt.valid := false.B // VecLsu.vec_xcpt, owned by VecLsu (E7)
  io.vec_xcpt.bits  := DontCare

  // ---- 13. lsu_fencei_rdy_vec: owned by VecLsu (E7). With no vector LSU
  //          instance there are no vector memory ops in flight to block
  //          fencei on, so the safe default is READY (true), not the
  //          reset-value-shaped false a stalled port would suggest. ----
  io.lsu_fencei_rdy_vec := true.B // VecLsu's fencei-ready, owned by VecLsu (E7)

  // ======== END D2 STAGING TIE-OFF ========

  // ===========================================================================
  // ---- PART 2. The decode arm ----
  // ===========================================================================
  vdec.io.dec_insns   := io.dec_insns
  vdec.io.dec_valids  := io.dec_valids
  vdec.io.dec_fire    := io.dec_fire
  vdec.io.dec_uops_in := io.dec_uops_in
  io.dec_uops_out     := vdec.io.dec_uops_out
  io.dec_vec_illegal  := vdec.io.dec_vec_illegal

  //@req-spec-decode.c22
  // `ren_br_tags`/`ren_br_vconfig` are BUILT HERE, off the REGISTERED
  // `ren2_uops`/`dis_fire`, not taken off the seam -- baseline
  // rename-stage.scala:117-118 verbatim with `ren2_fire = io.dis_fire`, so
  // the vtype snapshot table snapshots on exactly the cycles the scalar
  // branch snapshots are taken. `MicroOp.vconfig` is therefore required to
  // be written on EVERY br_tag-allocating uop, not only vector ones (the
  // MicroOp-delta obligation this line makes visible).
  val ren_br_tags = Wire(Vec(coreWidth + 1, Valid(UInt(brTagSz.W))))
  ren_br_tags(0).valid := false.B
  ren_br_tags(0).bits  := DontCare
  val ren_br_vconfig = Wire(Vec(coreWidth + 1, new VType))
  ren_br_vconfig(0) := DontCare
  for (w <- 0 until coreWidth) {
    ren_br_tags(w + 1).valid := io.dis_fire(w) && io.ren2_uops(w).allocate_brtag
    ren_br_tags(w + 1).bits  := io.ren2_uops(w).br_tag
    ren_br_vconfig(w + 1)    := io.ren2_uops(w).vconfig.get
  }
  vdec.io.ren_br_tags    := ren_br_tags
  vdec.io.ren_br_vconfig := ren_br_vconfig
  vdec.io.brupdate       := io.brupdate
  vdec.io.rollback       := io.commit_rollback
  vdec.io.com_valids     := io.commit_valids
  // `com_is_vset` per part 2: a vset is a SCALAR uop (no `is_vset` field
  // exists); the only other VL producer is `vleff`, which carries
  // `is_vec`, so excluding it here is what stops a `vleff` commit from
  // overwriting the committed vtype shadow with its own snapshot.
  vdec.io.com_is_vset := VecInit((0 until coreWidth).map { w =>
    io.commit_valids(w) && io.commit_uops(w).is_vl_producer.get && !io.commit_uops(w).is_vec.get
  })
  // `com_vtype` trusts the ROB delta to have already overridden
  // `commit_uops(w).vconfig` from its per-row `rob_vconfig` latch for a
  // register-sourced vsetvl -- this container reads it, it does not
  // re-derive it.
  vdec.io.com_vtype := VecInit((0 until coreWidth).map(w => io.commit_uops(w).vconfig.get))
  vdec.io.csr_vtype  := io.csr_vector.vconfig.vtype
  vdec.io.rob_empty  := io.rob_empty

  // ===========================================================================
  // ---- PART 3. The one shadow pipeline: dec_vl_imm -> ren2_vl_imm ----
  // ===========================================================================
  //
  //@req-spec-decode.i4
  // A per-lane register pair, ENABLED BY io.dec_fire(w) -- the same
  // lane-fire that admits the uop into the scalar RenameStage's
  // ren1-to-ren2 register. `dec_fire(w)` is the correct enable (and
  // `ren2_ready` is not available here): on a `ren2_ready` cycle with no
  // `dec_fire` the lane is invalid at ren2 and the stale shadow value is
  // never consumed downstream (every consumer is qualified by
  // `ren2_mask(w) && dis_fire(w)`, which the scalar stage already
  // branch-masks). No kill term is needed on the pair for the same reason.
  val ren2_vl_imm       = RegInit(VecInit(Seq.fill(coreWidth)(0.U(vecVLSz.W))))
  val ren2_vl_imm_valid = RegInit(VecInit(Seq.fill(coreWidth)(false.B)))
  for (w <- 0 until coreWidth) {
    when (io.dec_fire(w)) {
      ren2_vl_imm(w)       := vdec.io.dec_vl_imm(w)
      ren2_vl_imm_valid(w) := vdec.io.dec_vl_imm_valid(w)
    }
  }
  vl_rename.io.ren2_vl_imm.get       := ren2_vl_imm
  vl_rename.io.ren2_vl_imm_valid.get := ren2_vl_imm_valid

  // ===========================================================================
  // ---- PART 4. The rename arm: two instances of one definition, CHAINED ----
  // ===========================================================================
  //
  //@req-spec-rename.h15
  // `vec_rename` and `vl_rename` are CHAINED, not joined field-by-field:
  // io.ren2_uops -> vec_rename.ren2_uops, vec_rename.ren2_uops_out ->
  // vl_rename.ren2_uops, and vl_rename.ren2_uops_out IS `chainedUops`,
  // the dispatch bundle Part 5 routes. Each instance writes only the
  // fields it renamed, so the chain is the join.
  vec_rename.io.ren2_uops := io.ren2_uops
  vec_rename.io.ren2_mask := io.ren2_mask
  vec_rename.io.dis_fire  := io.dis_fire
  vec_rename.io.brupdate  := io.brupdate
  vec_rename.io.rollback  := io.commit_rollback
  vec_rename.io.com_valids := io.commit_valids
  vec_rename.io.com_uops   := io.commit_uops
  vec_rename.io.wakeups    := vecGroupDoneNetwork

  vl_rename.io.ren2_uops := vec_rename.io.ren2_uops_out
  vl_rename.io.ren2_mask := io.ren2_mask
  vl_rename.io.dis_fire  := io.dis_fire
  vl_rename.io.brupdate  := io.brupdate
  vl_rename.io.rollback  := io.commit_rollback
  vl_rename.io.com_valids := io.commit_valids
  vl_rename.io.com_uops   := io.commit_uops
  vl_rename.io.wakeups    := vlWakeupNetwork

  val chainedUops = vl_rename.io.ren2_uops_out
  io.dis_uops_out := chainedUops

  //@req-spec-core.i4
  //@req-spec-core.i5
  // Atomicity is enforced STRUCTURALLY here, not by a check: `dis_ready`
  // is one bit, the AND of both spaces' whole-bundle `alloc_ok` and the
  // (tied-true, item 1 above) `vlsu.dis_ok` term -- a destination group is
  // renamed WHOLE, all-or-nothing, or the bundle does not dispatch at all.
  // There is no per-lane vector allocation answer anywhere in this file
  // (decision D2, see part 5's own note) -- queue CAPACITY is native and
  // per-lane, but group ALLOCATION stays this one broadcast bit.
  io.dis_ready := vec_rename.io.alloc_ok && vl_rename.io.alloc_ok && vlsuDisOkTiedTrue

  // ===========================================================================
  // ---- PART 5. Dispatch: routing, and the single-cycle IQ slot write ----
  // ===========================================================================
  //
  //@req-spec-issue.e4
  //@req-spec-issue.e5
  // ALL QUEUES ISSUE IN A SINGLE SCHEDULING STAGE: this container adds no
  // second issue stage, no pre-issue arbitration and no register between
  // dispatch and a vector queue's slot write. `iq_v_load.iss_uops(0)`/
  // `iq_v_store.iss_uops(0)`/`iq_v_alu.iss_uops(0)` are the grants this
  // design defines for `vlsu.iss_ld`/`vlsu.iss_st`/`cii.iss` -- with vlsu
  // and cii absent (D2 staging) those three outputs simply have no
  // consumer in this file yet; that is not a tie-off (nothing is assigned
  // to force them), it is an unrouted output that E7/Phase F connect.
  //
  //@req-spec-rename.b10
  //@req-spec-rename.b11
  // EVERY UOP OF A DISPATCH GROUP WRITES ITS ISSUE-QUEUE SLOT IN THE
  // RENAME CYCLE: the fan-out below is combinational off `chainedUops`
  // (Part 4), qualified only by `io.dis_vec_valids`/`dis_fire`-derived
  // validity -- there is no register anywhere between `chainedUops` and a
  // slot write.
  //
  // ===> THE QUEUES ARE WIRED NATIVELY, NOT TIED READY (decision D2).
  // `io.dis_vec_valids`/`io.dis_vec_ready` are the per-lane, per-queue
  // dispatch handshake `CompactingDispatcher` drives/consumes; because
  // BoomCore's vector `issueParams` entries require `dispatchWidth ==
  // coreWidth` (the part-5 "trap", resolved a third way), a `Compactor`
  // degenerates to `io.out <> io.in` there, so queue lane `w` IS
  // `chainedUops(w)` -- no permutation crosses this seam.
  for (q <- 0 until vecQueues.length) {
    for (w <- 0 until coreWidth) {
      vecQueues(q).io.dis_uops(w).valid  := io.dis_vec_valids(q)(w)
      vecQueues(q).io.dis_uops(w).bits   := chainedUops(w)
      io.dis_vec_ready(q)(w)             := vecQueues(q).io.dis_uops(w).ready
      // The per-member readiness side channel (D6: FIVE groups --
      // vs1/vs2/vs3/vtmp/vold_rdy -- plus vm_rdy) travels BESIDE the uop,
      // lane for lane, into every queue's dis_member_rdy.
      vecQueues(q).io.dis_member_rdy(w)  := vec_rename.io.member_rdy.get(w)
    }
  }

  //@req-spec-core.c9
  // Routing is by the `iq_type` bitmask on the dispatch uop -- a
  // `Vec(IQ_SZ, Bool)`, not a queue ID, which is what lets a SHARED OP.v
  // (part 10, spec-decode.b4) name TWO queues at once while remaining ONE
  // uOP, one ROB entry. Checked at this boundary rather than trusted:
  for (w <- 0 until coreWidth) {
    val u = chainedUops(w)
    val namesAnyVecQueue = u.iq_type(IQ_V_LOAD) || u.iq_type(IQ_V_STORE) || u.iq_type(IQ_V_ALU)
    assert(!(io.dis_fire(w) && u.dst_rtype === RT_VEC) || namesAnyVecQueue,
      "VecPipeline: a dst_rtype===RT_VEC uop names no vector queue")
    assert(!(io.dis_fire(w) && !u.is_vec.get) || !namesAnyVecQueue,
      "VecPipeline: a non-vector uop has a vector iq_type bit set (A23)")
    assert(!(io.dis_fire(w) && namesAnyVecQueue) || !u.ppred_busy,
      "VecPipeline: a uop presented to a vector queue has ppred_busy set")
  }

  // ===========================================================================
  // ---- PART 6. The three wakeup networks ----
  // ===========================================================================
  //
  //@req-spec-issue.f1
  //@req-spec-issue.f7
  // Caracal PRESERVES BOOM's per-space wakeup partitioning and extends it
  // with exactly TWO NEW NETWORKS -- VL (`pvl`) and VECTOR (group-done) --
  // plus the connections each vector queue needs to the SCALAR networks
  // that supply its scalar feeders (INT to all three, FP to iq_v_alu only).
  for (q <- vecQueues) {
    //@req-spec-issue.f8
    //@req-spec-issue.f9
    // Fanned to ALL THREE: the integer network delivers the base address,
    // the stride and the GPR-sourced `.vx` operand of an OP.v.
    q.io.int_wakeup_ports := io.int_wakeups
    q.io.brupdate         := io.brupdate
    q.io.flush_pipeline    := io.rob_flush_kill
    //@req-spec-issue.f10
    //@req-spec-issue.h1
    // THE VL NETWORK: all three queues connect, because any OP.v may
    // depend on `pvl`. Plain readiness only -- no value on this network.
    q.io.vl_wakeup := vlWakeupNetwork
    q.io.vec_group_done := vecGroupDoneNetwork
    // The retraction half of the speculative wakeup, and the grant squash.
    // These were briefly tied to 0/false as "safe, no-effect defaults" when
    // the seam had no member for them. That was WRONG, not conservative:
    // `child_rebusys` is what re-marks this slot busy when a speculatively
    // woken scalar .vx/.vf feeder's parent load misses, so holding it at zero
    // lets the OP.v issue against a stale GPR with no error anywhere. Both are
    // BoomCore-internal terms and are now real seam members it drives.
    q.io.child_rebusys := io.int_child_rebusys
    q.io.squash_grant  := io.int_squash_grant
  }
  //@req-spec-issue.f12
  //@req-spec-issue.f13
  // The FP network reaches iq_v_alu ONLY -- iq_v_load/iq_v_store elaborate
  // numFpWakeupPorts = 0 and have no FP comparator at all.
  iq_v_alu.io.fp_wakeup_ports.get := io.fp_wakeups

  //@req-spec-issue.d1
  //@req-spec-issue.d14
  // pnrGate forwards rob_pnr_idx/rob_head_idx UNMODIFIED to iq_v_alu and
  // reads neither here: the 3-arg IsOlder(a, b, head) age comparison the
  // slot computes needs the head to disambiguate ROB wraparound, and this
  // is the SAME gate every CII op passes through -- no translation-
  // complete signal and no private path from the LSU half exists or is
  // added here.
  iq_v_alu.io.rob_pnr_idx.get  := io.rob_pnr_idx
  iq_v_alu.io.rob_head_idx.get := io.rob_head_idx

  //@req-spec-decode.i4
  // A VL-producing instruction allocates a fresh VL PRN at rename, writes
  // the new VL into the VL RF (Part 9), and broadcasts `pvl` on this
  // network. This container FORMS the network from its producers, because
  // none of them can see the others: lanes 0..aluWidth-1 are the ALU
  // vset writeback (write enable is `is_vl_producer`, NEVER `dst_rtype` --
  // `vsetvli x0, rs1` discards its integer destination and must still
  // write VL); lane `aluWidth` (the vleff trim) is D2-staged off above.
  for (i <- 0 until aluWidth) {
    vlWakeupNetwork(i).valid := io.vset_resp(i).valid && io.vset_resp(i).bits.uop.is_vl_producer.get
    vlWakeupNetwork(i).bits  := io.vset_resp(i).bits.uop.pvl.get
  }
  io.vl_wakeup := vlWakeupNetwork

  //@req-spec-core.f6
  //@req-spec-issue.f3
  //@req-spec-issue.f4
  //@req-spec-issue.f5
  // THE VECTOR NETWORK: `vecGroupDoneNetwork` (D2-staged invalid above) is
  // fanned to all three vector queues AND to vec_rename's busy-table clear
  // side, and to NOTHING ELSE -- no scalar queue, no scalar rename space
  // and no external consumer sees it, so the cross-space collision BOOM's
  // partitioning prevents cannot occur. Already connected to the three
  // queues in the Part 6 loop above; vec_rename's side is Part 4's
  // `vec_rename.io.wakeups := vecGroupDoneNetwork`.

  // ===========================================================================
  // ---- PART 7. The group-done fan-out: one event, three consumers ----
  // ===========================================================================
  //
  //@req-spec-core.g1
  //@req-spec-core.g3
  //@req-spec-rob.c5
  // For each VecLsu-owned lane the ROB clear is DERIVED from that lane's
  // group-done, not taken from a second port, so the two cannot be
  // emitted in different cycles. Lane 1 (VecCiiHost's) is the exception --
  // it is a union of two independent signals (group_done.rob_idx AND
  // clr_rob) and is tied directly in the D2 STAGING TIE-OFF block instead.
  io.vec_clr_bsy(0).valid := vecGroupDoneNetwork(0).valid
  io.vec_clr_bsy(0).bits  := vecGroupDoneNetwork(0).bits.rob_idx
  io.vec_clr_bsy(2).valid := vecGroupDoneNetwork(2).valid
  io.vec_clr_bsy(2).bits  := vecGroupDoneNetwork(2).bits.rob_idx

  //@req-spec-vrf.c11
  // MASKING SEMANTICS ARE HANDLED IN THE EXECUTION UNITS; this container
  // applies none -- no mask register, no mask mux and no active-lane
  // computation anywhere in this file. `vrf`'s trace gate is the only
  // vrf-facing wire this container drives outside the D2 tie-off block.
  vrf.io.trace_en    := io.vec_trace_en
  io.debug_vrf_read  := vrf.io.debug_vrf_read

  // ===========================================================================
  // ---- PART 9. The VL-RF ruling: the ALU writeback and the commit read ----
  // ===========================================================================
  //
  // W_ren (vl_rename's rename-cycle write, one per lane, unarbitrated).
  // Connected leaf-by-leaf rather than as a whole-Bundle `:=`: VecRenameSpace's
  // `vl_rf_write` payload is an ANONYMOUS `{addr, data}` Bundle (its own file,
  // not VlRegFileWriteData), so a whole-aggregate connect would depend on
  // Chisel's Bundle-type-equivalence rules across two distinct classes: safer
  // and equally correct to connect the two leaf UInt/Bool fields directly.
  for (w <- 0 until coreWidth) {
    vlrf.io.w_ren(w).valid     := vl_rename.io.vl_rf_write.get(w).valid
    vlrf.io.w_ren(w).bits.addr := vl_rename.io.vl_rf_write.get(w).bits.addr
    vlrf.io.w_ren(w).bits.data := vl_rename.io.vl_rf_write.get(w).bits.data
  }

  // W_alu, REPLICATED PER ALU EU (decision D8): lane `i` from
  // `io.vset_resp(i)`, enabled by `is_vl_producer` and never `dst_rtype`.
  // No mux, no arbiter between lanes.
  for (i <- 0 until aluWidth) {
    vlrf.io.w_alu(i).valid     := io.vset_resp(i).valid && io.vset_resp(i).bits.uop.is_vl_producer.get
    vlrf.io.w_alu(i).bits.addr := io.vset_resp(i).bits.uop.pvl.get
    vlrf.io.w_alu(i).bits.data := io.vset_resp(i).bits.data(vecVLSz - 1, 0)
  }

  // THE COMMIT READ: the address is selected HERE, from `commit_valids`/
  // `commit_uops` -- the YOUNGEST committing uop with `is_vl_producer` set
  // drives it (a commit bundle's highest lane index is its youngest
  // member). Only the DATA leaves, on `commit_vl`; the address needs no
  // seam member.
  val vlProducerCommits = (0 until coreWidth).map(w => io.commit_valids(w) && io.commit_uops(w).is_vl_producer.get)
  val youngestVlProducerAddr = PriorityMux(
    vlProducerCommits.reverse,
    (0 until coreWidth).reverse.map(w => io.commit_uops(w).pvl.get))
  val anyVlProducerCommits = vlProducerCommits.reduce(_ || _)
  vlrf.io.r_commit.addr := youngestVlProducerAddr
  io.commit_vl.valid    := anyVlProducerCommits
  io.commit_vl.bits     := vlrf.io.r_commit.data
  assert(!io.commit_vl.valid || anyVlProducerCommits,
    "VecPipeline: commit_vl fired with no committing VL producer")

  // `csr_vs_dirty` (owned here -- no child claims it): OR over commit
  // lanes of `is_vec || is_vl_producer`, commit-sourced for the same
  // flush-safety reason as `vec_rob_flags` (a writeback pulse would dirty
  // VS for an op later squashed).
  io.csr_vs_dirty := (0 until coreWidth).map { w =>
    io.commit_valids(w) && (io.commit_uops(w).is_vec.get || io.commit_uops(w).is_vl_producer.get)
  }.reduce(_ || _)

  // ===========================================================================
  // ---- PART 10. `pvtmp`: the rendezvous, and the chain that must not close ----
  // ===========================================================================
  //
  //@req-spec-core.i6
  //@req-spec-decode.b4
  //@req-spec-core.i7
  //@req-spec-core.i8
  //@req-spec-rename.e11
  //@req-spec-rob.d10
  //@req-spec-rob.d11
  //@req-spec-issue.c9
  // A shared instruction's two halves rendezvous ENTIRELY through the
  // `pvtmp` group in the VRF and its group-done on the vector wakeup
  // network -- there is no side channel, no handoff FIFO and no port
  // between `vlsu` and `cii` in this file (and neither is even
  // instantiated yet). The mechanism that makes that possible is already
  // wired above: `vecGroupDoneNetwork` reaches all three vector queues and
  // vec_rename identically (Part 6's loop and Part 4's `wakeups`
  // connection) -- a shared op's coprocessor half (in iq_v_alu) is woken
  // by the SAME broadcast its LSU-side producer half would use, with no
  // extra term added here. `IQ_V_ALU`'s own third-source select
  // (`Mux(is_shared && uses_ldq, pvtmp, pvs3)`) and the `vold_rdy`/
  // `vtmp_rdy` per-member channel are `VecIssueSlot`'s, not this
  // container's, to apply.

  // ===========================================================================
  // ---- PART 11. fu_types toward iq_v_load/iq_v_store: NOT a staging item ----
  // ===========================================================================
  //
  // ===> fu_types TOWARD iq_v_load/iq_v_store IS A COMPILE-TIME CONSTANT
  // (part 11), regardless of vlsu's existence: FC_AGEN and FC_DGEN
  // advertised every cycle, unconditionally. Per ground rule 6 (echoed by
  // the D2 task authorization), no module in the vector LSU may export a
  // busy reaching an issue unit -- issue eligibility for a vector memory
  // OP.v is "a reservation exists", decided at dispatch through `dis_ok`,
  // never a downstream functional-unit-ready signal. This is therefore
  // real, permanent wiring, kept OUT of the D2 STAGING TIE-OFF block.
  val vecMemFuTypesConst = VecInit(Seq.tabulate(FC_SZ)(i => (i == FC_AGEN || i == FC_DGEN).B))
  for (w <- 0 until iq_v_load.io.fu_types.length)  { iq_v_load.io.fu_types(w)  := vecMemFuTypesConst }
  for (w <- 0 until iq_v_store.io.fu_types.length) { iq_v_store.io.fu_types(w) := vecMemFuTypesConst }

  // ===========================================================================
  // ---- PART 12. Tracing (ground rule 11) ----
  // ===========================================================================
  //
  // Three VecTrace lines, all at seams no child can see. Two of the three
  // (group-done, VL-network lane `aluWidth`) are structurally wired but
  // currently dead under D2 staging, since their producers do not exist
  // yet -- that is expected, not a bug, and E7/Phase F need change nothing
  // here to make them live.

  // Event 1: one per dispatched vector uop -- rob_idx plus the queue set
  // it was routed to. Rung 1 (`trace`): a real MicroOp is in hand.
  for (w <- 0 until coreWidth) {
    when (io.dis_fire(w) && chainedUops(w).is_vec.get) {
      VecTrace.trace("VecPipeline", "dispatch", chainedUops(w), Seq(("iq_type", chainedUops(w).iq_type.asUInt)))
    }
  }

  // Event 2: one per group-done lane -- rob_idx plus member count. Rung 2
  // (`traceId`): VecGroupDone carries a real rob_idx but no MicroOp.
  for (i <- 0 until vectorParams.numVecWbPorts) {
    when (vecGroupDoneNetwork(i).valid) {
      VecTrace.traceId("VecPipeline", "group_done", vecGroupDoneNetwork(i).bits.rob_idx,
        Seq(("members", vecGroupDoneNetwork(i).bits.members), ("lane", i.U)))
    }
  }

  // Event 3: one per VL-network beat -- the producing lane. Rung 1
  // (`trace`): `io.vset_resp(i).bits.uop` is a real MicroOp. The tied-off
  // lane `aluWidth` never fires (see the D2 STAGING TIE-OFF block), so it
  // needs, and gets, no trace call here.
  for (i <- 0 until aluWidth) {
    when (vlWakeupNetwork(i).valid) {
      VecTrace.trace("VecPipeline", "vl_wakeup", io.vset_resp(i).bits.uop,
        Seq(("pvl", vlWakeupNetwork(i).bits), ("lane", i.U)))
    }
  }
}

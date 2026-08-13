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
import boom.v4.vec.generated.lsu.{VecLsu, VecLsuVrfWrite}

// GENERATED from src/main/nlhdl/vec/VecPipeline.nlhdl.scala. Do not hand-edit;
// regenerate via the nlhdl gen-rtl flow instead.
//
// SPEC DEFECT (found and fixed at seam): `child_rebusys` / `squash_grant` must be driven
// by BoomCore, not tied to 0/false (was wrong: silent data corruption if retraction is missed).
class VecPipeline(val numIntWakeupPorts: Int, val numFpWakeupPorts: Int)
  (implicit p: Parameters) extends BoomModule
{
  //@req-spec-core.a2
  //@req-spec-core.a3
  require(usingRVV, "VecPipeline: elaborated only under usingRVV (never rocket's usingVector)")

  val io = IO(new VecPipelineIO(numIntWakeupPorts, numFpWakeupPorts))

  // ===========================================================================
  // ---- PART 1. Instantiation map -- nine children named, seven built ----
  // ===========================================================================
  //
  //@req-spec-core.a5
  //@req-spec-core.a6
  //@req-spec-core.d5
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
    params            = vecIssueParams(IQ_V_LOAD,  1),
    numIntWakeupPorts = numIntWakeupPorts,
    pnrGate           = false,
    numFpWakeupPorts  = 0,
    numVecWbPorts     = vectorParams.numVecWbPorts))

  val iq_v_store = Module(new VecIssueUnit(
    params            = vecIssueParams(IQ_V_STORE, 1),
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
  val vrf  = Module(new VecRegFile())
  val vlrf = Module(new VlRegFile())

  //@req-spec-cii.j1
  //@req-spec-cii.j2
  //@req-spec-cii.j3

  // ===========================================================================
  // ======== PART 1b. The vector LSU (E7), and the cii staging that remains ====
  // ===========================================================================

  val vlsu = Module(new VecLsu)

  // ---- 1. dis_ready's vlsu.dis_ok term. ----
  vlsu.io.dis_fire := io.dis_fire
  val vlsuDisOk: Bool = vlsu.io.dis_ok.reduce(_ && _)

  vlsu.io.brupdate       := io.brupdate
  vlsu.io.rob_flush      := io.rob_flush
  vlsu.io.rob_flush_kill := io.rob_flush_kill
  vlsu.io.commit_valids  := io.commit_valids
  vlsu.io.commit_uops    := io.commit_uops
  vlsu.io.rob_head_idx   := io.rob_head_idx
  vlsu.io.rob_pnr_idx    := io.rob_pnr_idx

  // ---- 2. The vector (group-done) wakeup network: lanes 0 and 2 are VecLsu's
  //         (lcb, gcopy); lane 1 is VecCiiHost's and stays staged (Phase F). ----
  val vecGroupDoneNetwork = Wire(Vec(vectorParams.numVecWbPorts, Valid(new VecGroupDone)))
  vecGroupDoneNetwork(0) := vlsu.io.vec_group_done(0)
  vecGroupDoneNetwork(2) := vlsu.io.vec_group_done(1)
  vecGroupDoneNetwork(1).valid := false.B // cii.group_done, owned by VecCiiHost (Phase F)
  vecGroupDoneNetwork(1).bits  := DontCare

  // ---- 3. The VL wakeup network's LSU lane (the vleff trim). ----
  val vlWakeupNetwork = Wire(Vec(numVlWakeupPorts, Valid(UInt(vlPregSz.W))))
  vlWakeupNetwork(aluWidth).valid := vlsu.io.vl_wb.valid
  vlWakeupNetwork(aluWidth).bits  := vlsu.io.vl_wb.bits.pvl

  // ---- 4. iq_v_alu's dynamic fu_types(0): VecCiiHost (Phase F). ----
  iq_v_alu.io.fu_types(0) := VecInit(Seq.fill(FC_SZ)(false.B)) // cii.fu_types, owned by VecCiiHost (Phase F)

  // ---- 5. VecRegFile ("vrf"): R0-R4 and W0/W1 are VecLsu's; R5-R8 and W2 are
  //         VecCiiHost's and stay staged (Phase F). ----
  def vrfRead(port: Int, req: Valid[UInt]): Unit = {
    vrf.io.read(port).valid        := req.valid
    vrf.io.read(port).bits.addr    := req.bits
    vrf.io.read(port).bits.rob_idx := DontCare
  }
  vrfRead(0, vlsu.io.vrf_r0_req)
  vrfRead(1, vlsu.io.vrf_r1_req)
  vrfRead(2, vlsu.io.vrf_r2_req)
  vrfRead(3, vlsu.io.vrf_r3_req)
  vrfRead(4, vlsu.io.vrf_r4_req)
  vlsu.io.vrf_r0_resp.valid := RegNext(vlsu.io.vrf_r0_req.valid)
  vlsu.io.vrf_r0_resp.bits  := vrf.io.read_data(0)
  vlsu.io.vrf_r1_resp       := vrf.io.read_data(1)
  vlsu.io.vrf_r2_resp       := vrf.io.read_data(2)
  vlsu.io.vrf_r3_gnt        := true.B
  vlsu.io.vrf_r3_resp.valid := RegNext(vlsu.io.vrf_r3_req.valid)
  vlsu.io.vrf_r3_resp.bits  := vrf.io.read_data(3)
  vlsu.io.vrf_r4_resp       := vrf.io.read_data(4)
  for (rp <- 5 until vrf.numReadPorts) {
    vrf.io.read(rp).valid := false.B // R5-R8, owned by VecCiiHost (Phase F)
    vrf.io.read(rp).bits  := DontCare
  }

  def vrfWrite(port: Int, w: Valid[VecLsuVrfWrite]): Unit = {
    vrf.io.write(port).valid        := w.valid
    vrf.io.write(port).bits.addr    := w.bits.addr
    vrf.io.write(port).bits.data    := w.bits.data
    vrf.io.write(port).bits.mask    := w.bits.mask
    vrf.io.write(port).bits.rob_idx := DontCare
  }
  vrfWrite(0, vlsu.io.vrf_w0)
  vlsu.io.vrf_w1 match {
    case Some(w1) => vrfWrite(1, w1)
    case None     => vrf.io.write(1).valid := false.B; vrf.io.write(1).bits := DontCare
  }
  for (wp <- 2 until vrf.numWritePorts) {
    vrf.io.write(wp).valid := false.B // W2, owned by VecCiiHost (Phase F)
    vrf.io.write(wp).bits  := DontCare
  }

  // ---- 6. VlRegFile ("vlrf"): VecLsu's execute reads and the vleff write. ----
  for (i <- 0 until 2) {
    vlrf.io.r_exe(i).addr   := vlsu.io.vl_read_addr(i)
    vlsu.io.vl_read_data(i) := vlrf.io.r_exe(i).data
  }
  for (i <- 2 until vlrf.numExeReadPorts) {
    vlrf.io.r_exe(i).addr := 0.U // CII grant lanes (Phase F)
  }
  vlrf.io.w_lsu(0).valid     := vlsu.io.vl_wb.valid
  vlrf.io.w_lsu(0).bits.addr := vlsu.io.vl_wb.bits.pvl
  vlrf.io.w_lsu(0).bits.data := vlsu.io.vl_wb.bits.vl
  for (i <- 1 until vlrf.numLsuWritePorts) {
    vlrf.io.w_lsu(i).valid := false.B
    vlrf.io.w_lsu(i).bits  := DontCare
  }

  // ---- 7. int_rf_read_req: lanes 0-3 VecLsu, lane 4 VecCiiHost (Phase F). ----
  for (i <- 0 until 4) {
    io.int_rf_read_req(i) <> vlsu.io.int_rf_read_req(i)
    vlsu.io.int_rf_read_rsp(i) := io.int_rf_read_rsp(i)
  }
  io.int_rf_read_req(4).valid := false.B // VecCiiIssue's FP/INT lane (Phase F)
  io.int_rf_read_req(4).bits  := 0.U
  vlsu.io.int_wb_snoop := io.int_wb_snoop

  // ---- 7b. The host LSU tap. VecLsu owns its contents. ----
  io.lsu_vec <> vlsu.io.lsu_vec

  // ---- 8. fp_rf_read_req: VecCiiHost (Phase F). ----
  io.fp_rf_read_req := 0.U

  // ---- 9. Scalar-dest writeback: VecCiiHost (Phase F). ----
  io.int_wb.valid := false.B
  io.int_wb.bits  := DontCare
  io.fp_wb.valid  := false.B
  io.fp_wb.bits   := DontCare

  // ---- 10. Completion group-done -> ROB: lanes 0/2 from vecGroupDoneNetwork; lane 1 VecCiiHost. ----
  io.vec_clr_bsy(1).valid := false.B // union of cii.group_done/cii.clr_rob, owned by VecCiiHost (Phase F)
  io.vec_clr_bsy(1).bits  := DontCare

  // ---- 11. vec_rob_flags: lane 1 VecCiiHost, lanes 0/2 never produced by design. ----
  io.vec_rob_flags(0) := vlsu.io.vec_rob_flags(0)
  io.vec_rob_flags(2) := vlsu.io.vec_rob_flags(1)
  io.vec_rob_flags(1).valid := false.B // VecCiiHost (Phase F)
  io.vec_rob_flags(1).bits  := DontCare

  // ---- 12. vec_clr_unsafe / vec_xcpt: VecLsu (E7). ----
  io.vec_clr_unsafe := vlsu.io.vec_clr_unsafe
  io.vec_xcpt       := vlsu.io.vec_xcpt

  // ---- 13. lsu_fencei_rdy_vec: VecLsu (E7). ----
  io.lsu_fencei_rdy_vec := vlsu.io.lsu_fencei_rdy_vec

  // ======== END: cii staging remains; vlsu landed at E7 ========

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
  vdec.io.com_is_vset := VecInit((0 until coreWidth).map { w =>
    io.commit_valids(w) && io.commit_uops(w).is_vl_producer.get && !io.commit_uops(w).is_vec.get
  })
  vdec.io.com_vtype := VecInit((0 until coreWidth).map(w => io.commit_uops(w).vconfig.get))
  vdec.io.csr_vtype  := io.csr_vector.vconfig.vtype
  vdec.io.rob_empty  := io.rob_empty

  // ===========================================================================
  // ---- PART 3. The one shadow pipeline: dec_vl_imm -> ren2_vl_imm ----
  // ===========================================================================

  //@req-spec-decode.i4
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

  //@req-spec-rename.h15
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

  //@req-spec-lsu.b5
  // `ren2_mask` (= BoomCore's dis_valids, straight from rename), never dis_fire and never
  // dis_vec_valids: dis_ok feeds dis_ready feeds dis_fire, and dis_vec_valids comes off the
  // dispatcher whose ren_uops.valid is itself dis_fire-derived. Either one closes
  // dis_hazards -> dis_fire -> dis_ok -> dis_ready -> vec_stall -> ren_stalls -> dis_hazards.
  for (w <- 0 until coreWidth) {
    vlsu.io.dis_uops(w).valid := io.ren2_mask(w) && chainedUops(w).is_vec.get &&
      (chainedUops(w).uses_ldq || chainedUops(w).uses_stq)
    vlsu.io.dis_uops(w).bits  := chainedUops(w)
  }

  // The two memory issue grants are fire-and-forget: no ready, no busy back.
  vlsu.io.iss_ld := iq_v_load.io.iss_uops(0)
  vlsu.io.iss_st := iq_v_store.io.iss_uops(0)

  //@req-spec-core.i4
  //@req-spec-core.i5
  io.dis_ready := vec_rename.io.alloc_ok && vl_rename.io.alloc_ok && vlsuDisOk

  // ===========================================================================
  // ---- PART 5. Dispatch: routing, and the single-cycle IQ slot write ----
  // ===========================================================================

  //@req-spec-issue.e4
  //@req-spec-issue.e5
  //@req-spec-rename.b10
  //@req-spec-rename.b11
  for (q <- 0 until vecQueues.length) {
    for (w <- 0 until coreWidth) {
      vecQueues(q).io.dis_uops(w).valid  := io.dis_vec_valids(q)(w)
      vecQueues(q).io.dis_uops(w).bits   := io.dis_vec_uops(q)(w)
      io.dis_vec_ready(q)(w)             := vecQueues(q).io.dis_uops(w).ready
      // Selected by rob_idx, NOT by lane: under usingRVV the core builds a
      // CompactingDispatcher, which packs uops into low lanes, so dispatch lane w
      // and rename lane w are different instructions whenever compaction moves one.
      val fromLane = (0 until coreWidth).map(k =>
        chainedUops(k).rob_idx === io.dis_vec_uops(q)(w).rob_idx)
      vecQueues(q).io.dis_member_rdy(w) := Mux1H(fromLane,
        (0 until coreWidth).map(k => vec_rename.io.member_rdy.get(k)))
      assert(!io.dis_vec_valids(q)(w) || PopCount(fromLane) === 1.U,
        "VecPipeline: dispatched vector uop matches no rename lane -- member_rdy would be lost")
    }
  }

  //@req-spec-core.c9
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

  //@req-spec-issue.f1
  //@req-spec-issue.f7
  for (q <- vecQueues) {
    //@req-spec-issue.f8
    //@req-spec-issue.f9
    q.io.int_wakeup_ports := io.int_wakeups
    q.io.brupdate         := io.brupdate
    q.io.flush_pipeline    := io.rob_flush_kill
    //@req-spec-issue.f10
    //@req-spec-issue.h1
    q.io.vl_wakeup := vlWakeupNetwork
    q.io.vec_group_done := vecGroupDoneNetwork
    q.io.child_rebusys := io.int_child_rebusys
    q.io.squash_grant  := io.int_squash_grant
  }
  //@req-spec-issue.f12
  //@req-spec-issue.f13
  iq_v_alu.io.fp_wakeup_ports.get := io.fp_wakeups

  //@req-spec-issue.d1
  //@req-spec-issue.d14
  iq_v_alu.io.rob_pnr_idx.get  := io.rob_pnr_idx
  iq_v_alu.io.rob_head_idx.get := io.rob_head_idx

  //@req-spec-decode.i4
  for (i <- 0 until aluWidth) {
    vlWakeupNetwork(i).valid := io.vset_resp(i).valid && io.vset_resp(i).bits.uop.is_vl_producer.get
    vlWakeupNetwork(i).bits  := io.vset_resp(i).bits.uop.pvl.get
  }
  io.vl_wakeup := vlWakeupNetwork

  //@req-spec-core.f6
  //@req-spec-issue.f3
  //@req-spec-issue.f4
  //@req-spec-issue.f5

  // ===========================================================================
  // ---- PART 7. The group-done fan-out: one event, three consumers ----
  // ===========================================================================

  //@req-spec-core.g1
  //@req-spec-core.g3
  //@req-spec-rob.c5
  io.vec_clr_bsy(0).valid := vecGroupDoneNetwork(0).valid
  io.vec_clr_bsy(0).bits  := vecGroupDoneNetwork(0).bits.rob_idx
  io.vec_clr_bsy(2).valid := vecGroupDoneNetwork(2).valid
  io.vec_clr_bsy(2).bits  := vecGroupDoneNetwork(2).bits.rob_idx

  //@req-spec-vrf.c11
  vrf.io.trace_en    := io.vec_trace_en
  io.debug_vrf_read  := vrf.io.debug_vrf_read

  // ===========================================================================
  // ---- PART 9. The VL-RF ruling: the ALU writeback and the commit read ----
  // ===========================================================================

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

  io.csr_vs_dirty := (0 until coreWidth).map { w =>
    io.commit_valids(w) && (io.commit_uops(w).is_vec.get || io.commit_uops(w).is_vl_producer.get)
  }.reduce(_ || _)

  // ===========================================================================
  // ---- PART 10. `pvtmp`: the rendezvous, and the chain that must not close ----
  // ===========================================================================

  //@req-spec-core.i6
  //@req-spec-decode.b4
  //@req-spec-core.i7
  //@req-spec-core.i8
  //@req-spec-rename.e11
  //@req-spec-rob.d10
  //@req-spec-rob.d11
  //@req-spec-issue.c9

  // ===========================================================================
  // ---- PART 11. fu_types toward iq_v_load/iq_v_store: NOT a staging item ----
  // ===========================================================================

  val vecMemFuTypesConst = VecInit(Seq.tabulate(FC_SZ)(i => (i == FC_AGEN || i == FC_DGEN).B))
  for (w <- 0 until iq_v_load.io.fu_types.length)  { iq_v_load.io.fu_types(w)  := vecMemFuTypesConst }
  for (w <- 0 until iq_v_store.io.fu_types.length) { iq_v_store.io.fu_types(w) := vecMemFuTypesConst }

  // ===========================================================================
  // ---- PART 12. Tracing (ground rule 11) ----
  // ===========================================================================

  for (w <- 0 until coreWidth) {
    when (io.dis_fire(w) && chainedUops(w).is_vec.get) {
      VecTrace.trace("VecPipeline", "dispatch", chainedUops(w), Seq(("iq_type", chainedUops(w).iq_type.asUInt)))
    }
  }

  for (i <- 0 until vectorParams.numVecWbPorts) {
    when (vecGroupDoneNetwork(i).valid) {
      VecTrace.traceId("VecPipeline", "group_done", vecGroupDoneNetwork(i).bits.rob_idx,
        Seq(("members", vecGroupDoneNetwork(i).bits.members), ("lane", i.U)))
    }
  }

  for (i <- 0 until aluWidth) {
    when (vlWakeupNetwork(i).valid) {
      VecTrace.trace("VecPipeline", "vl_wakeup", io.vset_resp(i).bits.uop,
        Seq(("pvl", vlWakeupNetwork(i).bits), ("lane", i.U)))
    }
  }
}

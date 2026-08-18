//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Floating Point Datapath Pipeline
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v4.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.{Parameters}
import freechips.rocketchip.rocket
import freechips.rocketchip.tile

import boom.v4.common._
import boom.v4.util._

/**
 * Top level datapath that wraps the floating point issue window, regfile, and arithmetic units.
 */
class FpPipeline(implicit p: Parameters) extends BoomModule with tile.HasFPUParameters
{
  val fpIssueParams = issueParams.find(_.iqType == IQ_FP).get
  val dispatchWidth = fpIssueParams.dispatchWidth
  val numLlPorts = lsuWidth
  // Caracal (D7): +1 wakeup slot under usingRVV for the vector pipeline's
  // dedicated scalar-FP writeback (see the added write port/wakeup slot below).
  val numWakeupPorts = fpIssueParams.issueWidth + numLlPorts + (if (usingRVV) 1 else 0)
  val fpPregSz = log2Ceil(numFpPhysRegs)

  val io = IO(new Bundle {
    val brupdate         = Input(new BrUpdateInfo())
    val flush_pipeline   = Input(Bool())
    val fcsr_rm          = Input(UInt(width=freechips.rocketchip.tile.FPConstants.RM_SZ.W))
    val status           = Input(new freechips.rocketchip.rocket.MStatus())

    val dis_uops         = Vec(dispatchWidth, Flipped(Decoupled(new MicroOp)))

    // +1 for recoding.
    val ll_wports        = Flipped(Vec(lsuWidth, Valid(new ExeUnitResp(fLen+1))))// from memory unit
    val from_int         = Flipped(Decoupled(new ExeUnitResp(fLen+1)))// from integer RF
    val dgen             = Valid(new MemGen)           // to Load/Store Unit
    val to_int           = Decoupled(new ExeUnitResp(xLen))           // to integer RF

    val wakeups          = Vec(numWakeupPorts, Valid(new Wakeup))
    val wb               = Vec(numWakeupPorts, Valid(new ExeUnitResp(fLen+1)))

    // Caracal: the vector pipeline's single dedicated FP RF read port --
    // sole driver is VecCiiIssue's `.vf` scalar-FP operand capture -- and its
    // dedicated scalar-FP destination write port (D7: `vfmv.f.s` and any CII
    // op with an FP scalar destination). Both usingRVV-gated, absent otherwise.
    val vec_frf_read_req = if (usingRVV) Some(Input(UInt(maxPregSz.W))) else None
    val vec_frf_read_rsp = if (usingRVV) Some(Output(UInt(xLen.W))) else None
    val vec_fp_wb        = if (usingRVV) Some(Input(Valid(new ExeUnitResp(xLen)))) else None

    val debug_tsc_reg    = Input(UInt(width=xLen.W))
  })

  //**********************************
  // construct all of the modules

  val exe_units: Seq[FPExeUnit] = (0 until fpWidth) map { w =>
    Module(new FPExeUnit(
      hasFDiv = usingFDivSqrt && (w==fpWidth-1),
      hasFpiu = (w==fpWidth-1)
    )).suggestName(s"fp_exe_unit_${w}")
  }
  require (numFrfReadPorts >= 3)
  // Caracal: the share of logical FP RF read ports the FPExeUnits themselves
  // own -- naming it is what lets the two `require(rd_idx == ...)` checks
  // below keep their current meaning once a non-exe-unit (vector) port exists.
  val numFrfExeReadPorts = fpWidth * 3
  val numFrfLogicalReadPorts = numFrfExeReadPorts + (if (usingRVV) 1 else 0)
  if (usingRVV) {
    //@req-spec-core.b4
    // The added vector FP read port must be UNDENIABLE by PartiallyPortedRF's
    // ascending-logical-index physical-port allocation -- the vec_pipeline_io
    // seam carries no `ready`, so a denial would be silent. This is what keeps
    // every existing FPExeUnit's read-port grant, and the `io_squash_iss`
    // replay it drives, exactly as it behaves in the vectors-off baseline.
    require(numFrfReadPorts + 1 >= numFrfLogicalReadPorts)
  }
  val numFrfWritePorts = fpWidth + lsuWidth + (if (usingRVV) 1 else 0)

  val issue_unit     = IssueUnit(fpIssueParams, numWakeupPorts, false, false)
  issue_unit.suggestName("fp_issue_unit")
  issue_unit.io.rob_head := DontCare
  issue_unit.io.rob_pnr_idx := DontCare
  val fregfileBankedWriteArray = Seq.fill(numFrfWritePorts) { None }
  val fregfile       = Module(new BankedRF(
    UInt((fLen+1).W),
    numFrfBanks,
    numFrfLogicalReadPorts,
    numFpPhysRegs,
    numFrfLogicalReadPorts,
    numFrfReadPorts + (if (usingRVV) 1 else 0),
    numFrfWritePorts,
    fregfileBankedWriteArray,
    "Floating Point"
  ))
  val fp_bypasses = Wire(Vec(fpWidth, Valid(new ExeUnitResp(xLen+1))))
  val fp_wakeups = Wire(Vec(numWakeupPorts, Valid(new Wakeup)))
  io.wakeups := fp_wakeups

  //*************************************************************
  // Issue window logic

  val iss_uops   = issue_unit.io.iss_uops

  issue_unit.io.tsc_reg := io.debug_tsc_reg
  issue_unit.io.brupdate := io.brupdate
  issue_unit.io.flush_pipeline := io.flush_pipeline
  issue_unit.io.squash_grant := exe_units.map(_.io_squash_iss).reduce(_||_)


  //-------------------------------------------------------------
  // **** Dispatch Stage ****
  //-------------------------------------------------------------

  // Input (Dispatch)
  for (w <- 0 until dispatchWidth) {
    issue_unit.io.dis_uops(w) <> io.dis_uops(w)
  }

  //-------------------------------------------------------------
  // **** Issue Stage ****
  //-------------------------------------------------------------

  // Output (Issue)
  for (i <- 0 until issue_unit.issueWidth) {

    var fu_types = exe_units(i).io_ready_fu_types
    issue_unit.io.fu_types(i) := fu_types

  }

  // Wakeup
  issue_unit.io.wakeup_ports := fp_wakeups

  issue_unit.io.pred_wakeup_port.valid := false.B
  issue_unit.io.pred_wakeup_port.bits := DontCare
  issue_unit.io.child_rebusys := 0.U

  issue_unit.io.iss_uops zip exe_units map { case (i, u) => u.io_iss_uop := i }

  //-------------------------------------------------------------
  // **** Register Arbitrate Stage ****
  //-------------------------------------------------------------

  var rd_idx = 0
  for (unit <- exe_units) {
    for (i <- 0 until 3) {
      fregfile.io.arb_read_reqs(rd_idx) <> unit.io_arb_frf_reqs(i)
      rd_idx += 1
    }
  }
  require(rd_idx == numFrfExeReadPorts)
  if (usingRVV) {
    // Caracal: the vector pipeline's dedicated, unarbitrated FP read port --
    // sole consumer is VecCiiIssue's `.vf` operand capture. Appended LAST
    // (index numFrfExeReadPorts) so it can never take a physical port away
    // from an FPExeUnit: PartiallyPortedRF allocates in ascending logical
    // index order. `valid` is tied true.B because the seam carries no valid
    // bit -- this is a dedicated, unarbitrated port, not a shared one.
    fregfile.io.arb_read_reqs(numFrfExeReadPorts).valid := true.B
    fregfile.io.arb_read_reqs(numFrfExeReadPorts).bits  := io.vec_frf_read_req.get
    assert(fregfile.io.arb_read_reqs(numFrfExeReadPorts).ready)
  }

  //-------------------------------------------------------------
  // **** Register Read Stage ****
  //-------------------------------------------------------------

  rd_idx = 0
  for (unit <- exe_units) {
    unit.io_rrd_frf_bypasses := fp_bypasses
    for (i <- 0 until 3) {
      unit.io_rrd_frf_resps(i) := fregfile.io.rrd_read_resps(rd_idx)
      rd_idx += 1
    }
  }
  require(rd_idx == numFrfExeReadPorts)
  if (usingRVV) {
    // Caracal: response-cycle forward for the added read port. `fregfile`
    // bottoms out in a `Mem` read of a `RegNext`-ed address with NO
    // read-during-write forwarding; `fp_bypasses` covers that gap only for
    // the exe units (`unit.io_rrd_frf_bypasses`), so this delta builds the
    // forward locally for its own consumer instead. A write presented in the
    // address cycle is already reflected in the response (it lands at the
    // intervening clock edge); a write presented in the response cycle is
    // not, and that is the case forwarded here.
    val vec_frf_rd_addr  = RegNext(io.vec_frf_read_req.get)
    val vec_frf_wr_hits  = fregfile.io.write_ports.map(w => w.valid && w.bits.addr === vec_frf_rd_addr)
    assert(PopCount(vec_frf_wr_hits) <= 1.U)
    val vec_frf_fwd_data = Mux1H(vec_frf_wr_hits, fregfile.io.write_ports.map(_.bits.data))
    val vec_frf_resp     = Mux(vec_frf_wr_hits.reduce(_||_), vec_frf_fwd_data,
                                fregfile.io.rrd_read_resps(numFrfExeReadPorts))
    // `fregfile` stores hardfloat-recoded values; the vec_pipeline_io seam's
    // stated convention is architectural (IEEE-754) data, so unrecode once,
    // after the forward mux.
    io.vec_frf_read_rsp.get := ieee(vec_frf_resp)
  }


  //-------------------------------------------------------------
  // **** Execute Stage ****
  //-------------------------------------------------------------

  exe_units.map(_.io_brupdate := io.brupdate)



  //-------------------------------------------------------------
  // **** Writeback Stage ****
  //-------------------------------------------------------------

  val ll_wbarb = Module(new Arbiter(new ExeUnitResp(fLen+1), 1 + // mem
                                                             1 + // fromint
                                                             1   // fdiv
  ))


  // Hookup load writeback -- and recode FP values.
  ll_wbarb.io.in(0).valid := RegNext(io.ll_wports(0).valid &&
    !IsKilledByBranch(io.brupdate, io.flush_pipeline, io.ll_wports(0).bits))
  ll_wbarb.io.in(0).bits  := RegNext(UpdateBrMask(io.brupdate, io.ll_wports(0).bits))
  ll_wbarb.io.in(0).bits.data := recode(RegNext(io.ll_wports(0).bits.data),
                                        RegNext(io.ll_wports(0).bits.uop.mem_size =/= 2.U))

  val ifpu_resp = io.from_int
  ll_wbarb.io.in(1) <> ifpu_resp

  ll_wbarb.io.in(2) <> exe_units.find(_.hasFDiv).get.io_fdiv_resp.get


  // Cut up critical path by delaying the write by a cycle.
  // Wakeup signal is sent on cycle S0, write is now delayed until end of S1,
  // but Issue happens on S1 and RegRead doesn't happen until S2 so we're safe.
  fregfile.io.write_ports(0).valid := ll_wbarb.io.out.valid && ll_wbarb.io.out.bits.uop.dst_rtype === RT_FLT
  fregfile.io.write_ports(0).bits.addr := ll_wbarb.io.out.bits.uop.pdst
  fregfile.io.write_ports(0).bits.data := ll_wbarb.io.out.bits.data

  assert (ll_wbarb.io.in(0).ready) // never backpressure the memory unit.
  when (ifpu_resp.valid) { assert (ifpu_resp.bits.uop.dst_rtype === RT_FLT) }

  var w_cnt = 1
  for (i <- 1 until lsuWidth) {
    fregfile.io.write_ports(w_cnt).valid := RegNext(io.ll_wports(i).valid)
    fregfile.io.write_ports(w_cnt).bits.addr := RegNext(io.ll_wports(i).bits.uop.pdst)
    fregfile.io.write_ports(w_cnt).bits.data := recode(RegNext(io.ll_wports(i).bits.data),
                                                       RegNext(io.ll_wports(i).bits.uop.mem_size =/= 2.U))
    w_cnt += 1
  }
  for (eu <- exe_units) {
    fregfile.io.write_ports(w_cnt).valid     := eu.io_fpu_resp.valid && eu.io_fpu_resp.bits.uop.dst_rtype === RT_FLT
    fregfile.io.write_ports(w_cnt).bits.addr := eu.io_fpu_resp.bits.uop.pdst
    fregfile.io.write_ports(w_cnt).bits.data := eu.io_fpu_resp.bits.data
    w_cnt += 1
  }
  if (usingRVV) {
    // Caracal (D7): dedicated scalar-FP destination write port for the vector
    // pipeline's CII writeback (`vfmv.f.s`, and any CII op whose destination
    // is an FP scalar register). No arbitration: VecCiiWriteback drives this
    // beat fire-and-forget with no back-pressure, and an Arbiter input can be
    // denied with no provable bound -- see the amended reject list in
    // FpPipeline.nlhdl.scala. No RegNext: the CII beat has already crossed a
    // queue and a registered writeback stage inside VecCiiWriteback.
    // `v_eew` is the recode tag (SEW-derived; a vfmv.f.s result is SEW wide),
    // the same shape the long-latency path derives from `mem_size`.
    fregfile.io.write_ports(w_cnt).valid     := io.vec_fp_wb.get.valid &&
                                                 io.vec_fp_wb.get.bits.uop.dst_rtype === RT_FLT
    fregfile.io.write_ports(w_cnt).bits.addr := io.vec_fp_wb.get.bits.uop.pdst
    fregfile.io.write_ports(w_cnt).bits.data := recode(io.vec_fp_wb.get.bits.data,
                                                        io.vec_fp_wb.get.bits.uop.v_eew.get =/= 2.U)
    w_cnt += 1
  }
  for (w <- 0 until fpWidth) {
    fp_bypasses(w).valid := exe_units(w).io_fpu_resp.valid && exe_units(w).io_fpu_resp.bits.uop.dst_rtype === RT_FLT
    fp_bypasses(w).bits  := exe_units(w).io_fpu_resp.bits
  }


  require (w_cnt == fregfile.io.write_ports.length)

  val fpiu_unit = exe_units.find(_.hasFpiu).get
  io.to_int <> fpiu_unit.io_fpiu_resp.get
  io.dgen   := fpiu_unit.io_dgen.get

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Commit Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  var idx = 0
  for (w <- 0 until fpWidth) {
    fp_wakeups(idx)  := exe_units(w).io_wakeup
    io.wb(idx) := exe_units(w).io_fpu_resp
    idx += 1
  }


  fp_wakeups(idx).valid    := ll_wbarb.io.out.valid
  fp_wakeups(idx).bits.uop := ll_wbarb.io.out.bits.uop
  fp_wakeups(idx).bits.speculative_mask := 0.U
  fp_wakeups(idx).bits.bypassable := false.B
  fp_wakeups(idx).bits.rebusy := false.B

  io.wb(idx) := ll_wbarb.io.out
  ll_wbarb.io.out.ready := true.B
  idx += 1

  for (i <- 1 until lsuWidth) {
    val wb = RegNext(UpdateBrMask(io.brupdate, io.flush_pipeline, io.ll_wports(i)))
    fp_wakeups(idx).valid := wb.valid && wb.bits.uop.dst_rtype === RT_FLT
    fp_wakeups(idx).bits.uop := wb.bits.uop
    fp_wakeups(idx).bits.speculative_mask := 0.U
    fp_wakeups(idx).bits.bypassable := false.B
    fp_wakeups(idx).bits.rebusy := false.B


    io.wb(idx) := wb
    io.wb(idx).bits.data := recode(
      RegNext(io.ll_wports(i).bits.data),
      RegNext(io.ll_wports(i).bits.uop.mem_size =/= 2.U)
    )
    idx += 1
  }
  if (usingRVV) {
    //@req-spec-vrf.e4
    // Caracal (D7): the wakeup slot that goes with the added write port above,
    // appended LAST so no existing wakeup/io.wb index is renumbered. This
    // slot IS the FP-network tap: `.vf` scalar feeders of a vector issue slot
    // wake on this same, unmodified `fp_wakeups`/`io.wakeups` wire -- no
    // second wakeup network is added, here or anywhere else in this file.
    // Same-cycle wakeup-and-write (no RegNext) is what makes
    // `bypassable := false` correct: the earliest a consumer woken at T can
    // present a read address is T+1, and the RF reads RegNext-ed addresses,
    // so data returns at T+2 -- strictly after this write has landed.
    fp_wakeups(idx).valid    := io.vec_fp_wb.get.valid &&
                                 io.vec_fp_wb.get.bits.uop.dst_rtype === RT_FLT
    fp_wakeups(idx).bits.uop := io.vec_fp_wb.get.bits.uop
    fp_wakeups(idx).bits.speculative_mask := 0.U
    fp_wakeups(idx).bits.bypassable := false.B
    fp_wakeups(idx).bits.rebusy := false.B

    io.wb(idx) := io.vec_fp_wb.get
    io.wb(idx).bits.data := recode(io.vec_fp_wb.get.bits.data,
                                   io.vec_fp_wb.get.bits.uop.v_eew.get =/= 2.U)
    idx += 1
  }
  require (idx == numWakeupPorts)
  exe_units.map(_.io_fcsr_rm := io.fcsr_rm)
  exe_units.map(_.io_status := io.status)

  //-------------------------------------------------------------
  // **** Flush Pipeline ****
  //-------------------------------------------------------------
  // flush on exceptions, miniexeptions, and after some special instructions

  for (w <- 0 until exe_units.length) {
    exe_units(w).io_kill := io.flush_pipeline
  }

  override def toString: String =
    (BoomCoreStringPrefix("===FP Pipeline===") + "\n"
    + exe_units.map(_.toString).mkString("\n") + "\n"
    + fregfile.toString
    + BoomCoreStringPrefix(
      "Num Wakeup Ports      : " + numWakeupPorts))
}

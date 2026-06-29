//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISC-V Processor Core
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// BOOM has the following (conceptual) stages:
//   if0 - Instruction Fetch 0 (next-pc select)
//   if1 - Instruction Fetch 1 (I$ access)
//   if2 - Instruction Fetch 2 (instruction return)
//   if3 - Instruction Fetch 3 (enqueue to fetch buffer)
//   if4 - Instruction Fetch 4 (redirect from bpd)
//   dec - Decode
//   ren - Rename1
//   dis - Rename2/Dispatch
//   iss - Issue
//   rrd - Register Read
//   exe - Execute
//   mem - Memory
//   sxt - Sign-extend
//   wb  - Writeback
//   com - Commit

package boom.v4.exu

import java.nio.file.{Paths}

import chisel3._
import chisel3.util._
import chisel3.experimental.IntParam

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.rocket.{Causes, PRV, CSR, CSRs, TracedInstruction}
import freechips.rocketchip.tile.{HasFPUParameters, TraceBundle, CustomCSR}
import freechips.rocketchip.util.{Str, UIntIsOneOf, CoreMonitorBundle, PlusArg}
import freechips.rocketchip.devices.tilelink.{PLICConsts, CLINTConsts}
import freechips.rocketchip.trace.{TraceCoreIngress, TraceCoreInterface, TraceCoreParams}

import boom.v4.common._
import boom.v4.ifu.{GlobalHistory, HasBoomFrontendParameters}
import boom.v4.util._
import boom.v4.vec.decode.VConfigUnit
import boom.v4.vec.rename.{VecRenameStage, VlRename}
import boom.v4.vec.regfile.{VecRegFile, VlRegFile}

/**
 * Top level core object that connects the Frontend to the rest of the pipeline.
 */
class BoomCore(roccCSRs: Seq[Seq[CustomCSR]])(implicit p: Parameters) extends BoomModule
  with HasBoomFrontendParameters // TODO: Don't add this trait
  with HasFPUParameters
{
  val nTotalRoCCCSRs = roccCSRs.flatten.size
  val traceIngressParams = TraceCoreParams(nGroups = boomParams.retireWidth, iretireWidth = 1, 
                                            xlen = coreParams.xLen, iaddrWidth = vaddrBitsExtended) 
  val io = IO(new freechips.rocketchip.tile.CoreBundle
  {
    val hartid = Input(UInt(hartIdLen.W))
    val interrupts = Input(new freechips.rocketchip.rocket.CoreInterrupts(false))
    val ifu = new boom.v4.ifu.BoomFrontendIO
    val ptw = Flipped(new freechips.rocketchip.rocket.DatapathPTWIO())
    val rocc = Flipped(new freechips.rocketchip.tile.RoCCCoreIO(nTotalRoCCCSRs))
    val lsu = Flipped(new boom.v4.lsu.LSUCoreIO)
    val ptw_tlb = new freechips.rocketchip.rocket.TLBPTWIO()
    val trace = Output(new TraceBundle)
    val fcsr_rm = UInt(freechips.rocketchip.tile.FPConstants.RM_SZ.W)
    val traceStall = Input(Bool())
    val trace_core_ingress = if (boomParams.enableTraceCoreIngress) Some(Output(new TraceCoreInterface(traceIngressParams))) else None
  })
  io.ptw_tlb := DontCare
  io.ptw := DontCare
  io.ifu := DontCare

  //**********************************
  // construct all of the modules

  val mem_exe_units: Seq[MemExeUnit] = (0 until memWidth) map { w =>
    Module(new MemExeUnit(
      hasAGen = w >= (memWidth - lsuWidth),
      hasDGen = true
    )).suggestName(s"mem_exe_unit_${w}")
  }
  val agen_exe_units: Seq[MemExeUnit] = mem_exe_units.filter(_.hasAGen)
  val unq_exe_units: Seq[UniqueExeUnit] = Seq(
    Module(new UniqueExeUnit(
      hasCSR = true,
      hasRocc = usingRoCC,
      hasMul = true,
      hasDiv = true,
      hasIfpu = true
    )).suggestName(s"unique_exe_unit_0")
  )
  val unq_exe_unit = unq_exe_units(0)
  val csr_resp = unq_exe_unit.io_csr_resp.get
  val alu_exe_units: Seq[ALUExeUnit] = (0 until aluWidth) map { w =>
    Module(new ALUExeUnit(
      id             = w
    )).suggestName(s"alu_exe_unit_${w}")
  }
  val all_exe_units = (alu_exe_units ++ mem_exe_units ++ unq_exe_units)

  // Meanwhile, the FP pipeline holds the FP issue window, FP regfile, and FP arithmetic units.
  val fp_pipeline = Module(new FpPipeline)

  // ********************************************************
  // Clear fp_pipeline before use
  fp_pipeline.io.ll_wports := DontCare


  val numIrfWritePorts        = aluWidth + lsuWidth + 1
  // +1 dedicated integer-RF read port for the vector LS base-address read when
  // usingRVV (Step 11a.2). Gated -> the vector-OFF RF is unchanged (bit-identical).
  val numIrfLogicalReadPorts  = all_exe_units.map(_.nReaders).reduce(_+_) + (if (usingRVV) 1 else 0)

  val numIntWakeups           = coreWidth + lsuWidth + 1
  val numFpWakeupPorts        = fp_pipeline.io.wakeups.length

  val numImmReaders     = aluWidth + memWidth + 1 // "Wakeup" immediates when they are read

  val decode_units      = (0 until decodeWidth) map { w => Module(new DecodeUnit).suggestName(s"decode_${w}") }
  val dec_brmask_logic  = Module(new BranchMaskGenerationLogic(coreWidth))
  val rename_stage      = Module(new RenameStage(coreWidth, numIntPhysRegs, numIntWakeups, false))
  val fp_rename_stage   = Module(new RenameStage(coreWidth, numFpPhysRegs, numFpWakeupPorts, true))
  val pred_rename_stage = Module(new PredRenameStage(coreWidth, 1))
  val imm_rename_stage  = Module(new ImmRenameStage(coreWidth, numImmReaders)) // wakeup ports used when insts read imm
  val rename_stages     = Seq(rename_stage, pred_rename_stage, imm_rename_stage) ++ (if (usingFPU) Seq(fp_rename_stage) else Nil)

  // Caracal vector rename (Step 4): VConfig mirror + vector-group rename + VL rename.
  // Instantiated in parallel with scalar rename. NOT added to rename_stages (the
  // scalar drive loop is left byte-identical). All wiring is gated by usingRVV so
  // the vector-OFF RTL is unchanged (gate 9f).
  val vconfig_unit     = if (usingRVV) Some(Module(new VConfigUnit)) else None
  val vec_rename_stage = if (usingRVV) {
    Some(Module(new VecRenameStage(coreWidth, numVecPhysRegs, coreWidth, 1)))
  } else None
  val vl_rename        = if (usingRVV) Some(Module(new VlRename(coreWidth, numVlPhysRegs, 3))) else None

  // Caracal vector register files (Step 8): the vector physical register file
  // (8R/4W, VLEN-wide with per-64b write mask) and the VL physical register
  // file (6R/3W). Both are DORMANT here -- producers (Step 9 register-write)
  // and consumers (Step 11 register-read) are not wired yet. Gated by usingRVV
  // so the vector-OFF RTL is byte-identical (gate 9f). All inputs are tied off
  // below; read_ports.data outputs are left dangling (legal in Chisel).
  // numDebugReadPorts = coreWidth * MAX_MEMBERS: a combinational readback of each
  // committing vector op's full dest group, for the cosim commit trace (Step 11a.2).
  val vec_regfile = if (usingRVV) Some(Module(new VecRegFile(8, 4, coreWidth * boom.v4.vec.rename.VecEmul.MAX_MEMBERS))) else None
  val vl_regfile  = if (usingRVV) Some(Module(new VlRegFile(6, 3)))  else None

  // Caracal vector LSU (Step 11a.2): unit-stride vle beat engine. Declared at
  // top level so the completion datapath (group-done -> vec rename / issue
  // wakeups + rob.vec_clr_bsy) and the dcache / VRF / AGEN wiring can all
  // reference it across the (separate) usingRVV blocks below. Gated by usingRVV.
  val vec_lsu = if (usingRVV) Some(Module(new boom.v4.vec.lsu.VecLSU)) else None

  // Caracal vector LS register-read stage (Step 11a.2, sub-step B): reads the
  // vle base address (rs1) from a dedicated integer-RF read port + vl from the
  // VL-RF, then drives VecLsDecode/AGEN. Top-level so the arb/rrd stages can wire
  // its dedicated iregfile read port. Gated by usingRVV.
  val vec_ls_rr = if (usingRVV) Some(Module(new boom.v4.vec.lsu.VecLSRegRead)) else None

  val mem_iss_unit     = IssueUnit(memIssueParam, numIntWakeups, false, false)
  val unq_iss_unit     = IssueUnit(unqIssueParam, numIntWakeups, false, false)
  val alu_iss_unit     = IssueUnit(aluIssueParam, numIntWakeups, enableColumnALUIssue, enableALUSingleWideDispatch)

  // Caracal vector issue units (Step 6): three DORMANT vector issue queues
  // (V_LOAD / V_STORE / V_ALU). Gated by usingRVV so the vector-OFF RTL is
  // byte-identical (gate 9f). .get is safe because v*IssueParam is Some iff
  // usingRVV (see parameters.scala require). All inputs are driven below;
  // fu_types=0 and tied-off wakeups keep them from ever granting.
  val vload_iss_unit  = if (usingRVV) Some(boom.v4.vec.issue.VecIssueUnit(vLoadIssueParam.get,  numIntWakeups)) else None
  val vstore_iss_unit = if (usingRVV) Some(boom.v4.vec.issue.VecIssueUnit(vStoreIssueParam.get, numIntWakeups)) else None
  val valu_iss_unit   = if (usingRVV) Some(boom.v4.vec.issue.VecIssueUnit(vAluIssueParam.get,   numIntWakeups)) else None

  // Caracal (Step 9): collapse the per-ALU-lane vset writeback responses into a single
  // Valid[VsetWbResp]. Exactly one ALU lane (the int ALU carrying the vset FU) produces a
  // valid response per cycle, so OR-ing valids and Mux1H-ing the bits is exact. This wire
  // is the producer for the ROB vl/vtype stash, the VL-RF write, and the VL wakeup. It is
  // driven at the ALU-resp consumption site below and read in the regfile/rename/issue
  // tie-off blocks. Gated by usingRVV so the vector-OFF RTL is byte-identical (gate 9f).
  val vset_wb = if (usingRVV) Some(Wire(Valid(new boom.v4.common.VsetWbResp))) else None

  val dispatcher       = Module(new BasicDispatcher)
  val iregfileBankedWriteArray = Seq.fill(lsuWidth + 1) { None } ++ ((0 until aluWidth).map { w => if (enableColumnALUWrites) Some(w) else None })
  val iregfile         = Module(new BankedRF(
    UInt(xLen.W),
    numIrfBanks,
    numIrfLogicalReadPorts,
    numIntPhysRegs,
    numIrfLogicalReadPorts,
    numIrfReadPorts,
    numIrfWritePorts,
    iregfileBankedWriteArray,
    "Integer"
  ))
  val pregfile         = Module(new FullyPortedRF(
    Bool(),
    ftqSz,
    aluWidth,
    1,
    "Predicate"
  ))
  val immregfile       = Module(new FullyPortedRF(
    UInt(LONGEST_IMM_SZ.W),
    numImmPhysRegs,
    numImmReaders,
    coreWidth,
    "Immediate"
  ))
  val bregfile         = Module(new FullyPortedRF(
    new BrInfoBundle,
    maxBrCount,
    aluWidth,
    coreWidth,
    "Branch"
  ))
  val rob              = Module(new Rob(
    numIrfWritePorts + numFpWakeupPorts,
    trace
  ))
  // Used to wakeup registers in rename and issue. ROB needs to listen to something else.
  val int_wakeups  = Wire(Vec(numIntWakeups, Valid(new Wakeup)))
  val pred_wakeups = Wire(Vec(aluWidth     , Valid(new Wakeup)))

  // The arb stage guarantees only 1 pred wakeup per cycle
  assert(PopCount(pred_wakeups.map(_.valid)) <= 1.U)
  val pred_wakeup  = Wire(Valid(new Wakeup))
  pred_wakeup.valid         := pred_wakeups.map(_.valid).reduce(_||_)
  pred_wakeup.bits          := DontCare
  pred_wakeup.bits.uop.pdst := Mux1H(pred_wakeups.map(_.valid), pred_wakeups.map(_.bits.uop.pdst))

  val int_bypasses  = Wire(Vec(coreWidth + lsuWidth, Valid(new ExeUnitResp(xLen))))

  //***********************************
  // Pipeline State Registers and Wires

  // Decode/Rename1 Stage
  val dec_valids = Wire(Vec(coreWidth, Bool()))  // are the decoded instruction valid? It may be held up though.
  val dec_uops   = Wire(Vec(coreWidth, new MicroOp()))
  val dec_fire   = Wire(Vec(coreWidth, Bool()))  // can the instruction fire beyond decode?
                                                    // (can still be stopped in ren or dis)
  val dec_ready  = Wire(Bool())
  val dec_xcpts  = Wire(Vec(coreWidth, Bool()))
  val ren_stalls = Wire(Vec(coreWidth, Bool()))

  // Rename2/Dispatch stage
  val dis_valids = Wire(Vec(coreWidth, Bool()))
  val dis_uops   = Wire(Vec(coreWidth, new MicroOp))
  val dis_fire   = Wire(Vec(coreWidth, Bool()))
  val dis_ready  = Wire(Bool())

  // Issue Stage/Register Read
  val mem_iss_uops = mem_iss_unit.io.iss_uops
  val alu_iss_uops = alu_iss_unit.io.iss_uops
  val unq_iss_uops = unq_iss_unit.io.iss_uops

  // --------------------------------------
  // Dealing with branch resolutions

  // The individual branch resolutions from each ALU
  val brinfos = Reg(Vec(coreWidth, Valid(new BrResolutionInfo)))

  // "Merged" branch update info from all ALUs
  // brmask contains masks for rapidly clearing mispredicted instructions
  // brindices contains indices to reset pointers for allocated structures
  //           brindices is delayed a cycle
  val brupdate  = Wire(new BrUpdateInfo)
  val b1    = Wire(new BrUpdateMasks)
  val b2    = Reg(new BrResolutionInfo)

  brupdate.b1 := b1
  brupdate.b2 := b2

  for ((b, a) <- brinfos zip alu_exe_units) {
    b.bits := UpdateBrMask(brupdate, a.io_brinfo.bits)
    b.valid := a.io_brinfo.valid && !rob.io.flush.valid && !IsKilledByBranch(brupdate, RegNext(rob.io.flush.valid), a.io_brinfo.bits)
  }
  b1.resolve_mask := brinfos.map(x => x.valid << x.bits.uop.br_tag).reduce(_|_)
  b1.mispredict_mask := brinfos.map(x => (x.valid && x.bits.mispredict) << x.bits.uop.br_tag).reduce(_|_)

  // Find the oldest mispredict and use it to update indices
  val live_brinfos      = brinfos.map(br => br.valid && br.bits.mispredict && !IsKilledByBranch(brupdate, RegNext(rob.io.flush.valid), br.bits.uop))
  val mispredict_val    = live_brinfos.reduce(_||_)

  b2.mispredict  := mispredict_val
  b2.cfi_type    := Mux1H(live_brinfos, brinfos.map(_.bits.cfi_type))
  b2.taken       := Mux1H(live_brinfos, brinfos.map(_.bits.taken))
  b2.pc_sel      := Mux1H(live_brinfos, brinfos.map(_.bits.pc_sel))
  b2.uop         := UpdateBrMask(brupdate, PriorityMux(live_brinfos, brinfos.map(_.bits.uop)))
  b2.jalr_target := Mux1H(live_brinfos, brinfos.map(_.bits.jalr_target))
  b2.target_offset := Mux1H(live_brinfos, brinfos.map(_.bits.target_offset))

  val oldest_mispredict_ftq_idx = Mux1H(live_brinfos, brinfos.map(_.bits.uop.ftq_idx))


  assert (!((brupdate.b1.mispredict_mask =/= 0.U || brupdate.b2.mispredict)
    && rob.io.rollback), "Can't have a mispredict during rollback.")

  io.ifu.brupdate := brupdate

  for (eu <- all_exe_units) {
    eu.io_brupdate := brupdate
  }

  fp_pipeline.io.brupdate := brupdate

  // Load/Store Unit & ExeUnits
  // val mem_resps = io.lsu.iresp
  var agen_idx = 0
  var dgen_idx = 0
  for (eu <- mem_exe_units) {
    if (eu.hasAGen) {
      io.lsu.agen(agen_idx) := eu.io_agen.get
      agen_idx += 1
    }
    if (eu.hasDGen) {
      io.lsu.dgen(dgen_idx) := eu.io_dgen.get
      dgen_idx += 1
    }
  }
  io.lsu.dgen(dgen_idx) := fp_pipeline.io.dgen



  //-------------------------------------------------------------
  // Uarch Hardware Performance Events (HPEs)

  val perfEvents = new freechips.rocketchip.rocket.EventSets(Seq(
    new freechips.rocketchip.rocket.EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("exception", () => rob.io.com_xcpt.valid),
      ("nop",       () => false.B),
      ("nop",       () => false.B),
      ("nop",       () => false.B))),

    new freechips.rocketchip.rocket.EventSet((mask, hits) => (mask & hits).orR, Seq(
//      ("I$ blocked",                        () => icache_blocked),
      ("nop",                               () => false.B),
      ("branch misprediction",              () => b2.mispredict),
      ("control-flow target misprediction", () => b2.mispredict &&
                                                  b2.cfi_type === CFI_JALR),
      ("flush",                             () => rob.io.flush.valid),
// ("branch resolved",                   () => b2.valid)
    )),

    new freechips.rocketchip.rocket.EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("I$ miss",     () => io.ifu.perf.acquire),
      ("D$ miss",     () => io.lsu.perf.acquire),
      ("D$ release",  () => io.lsu.perf.release),
      ("ITLB miss",   () => io.ifu.perf.tlbMiss),
      ("DTLB miss",   () => io.lsu.perf.tlbMiss),
      ("L2 TLB miss", () => io.ptw.perf.l2miss)))))
  val csr = Module(new freechips.rocketchip.rocket.CSRFile(perfEvents, boomParams.customCSRs.decls, roccCSRs.flatten))
  csr.io.inst foreach { c => c := DontCare }
  csr.io.rocc_interrupt := io.rocc.interrupt
  csr.io.gva := DontCare
  csr.io.htval := DontCare
  csr.io.mhtinst_read_pseudo := false.B

  val custom_csrs = Wire(new BoomCustomCSRs)
  custom_csrs.csrs.foreach { c => c.stall := false.B; c.set := false.B; c.sdata := DontCare }
  (custom_csrs.csrs zip csr.io.customCSRs).map { case (lhs, rhs) => lhs <> rhs }
  io.ifu.enable_bpd := custom_csrs.enableBPD

  //val icache_blocked = !(io.ifu.fetchpacket.valid || RegNext(io.ifu.fetchpacket.valid))
  val icache_blocked = false.B
  csr.io.counters foreach { c => c.inc := RegNext(perfEvents.evaluate(c.eventSel)) }

  //****************************************
  // Time Stamp Counter & Retired Instruction Counter
  // (only used for printf and vcd dumps - the actual counters are in the CSRFile)
  val debug_tsc_reg = RegInit(0.U(xLen.W))
  val debug_irt_reg = RegInit(0.U(xLen.W))
  val debug_brs     = RegInit(VecInit(Seq.fill(5) { 0.U(xLen.W) }))
  val debug_jals    = RegInit(VecInit(Seq.fill(5) { 0.U(xLen.W) }))
  val debug_jalrs   = RegInit(VecInit(Seq.fill(5) { 0.U(xLen.W) }))

  for (j <- 0 until 5) {
    debug_brs(j) := debug_brs(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_br
    }))
    debug_jals(j) := debug_jals(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_jal
    }))
    debug_jalrs(j) := debug_jalrs(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_jalr
    }))
  }

  dontTouch(debug_brs)
  dontTouch(debug_jals)
  dontTouch(debug_jalrs)

  debug_tsc_reg := debug_tsc_reg + 1.U
  debug_irt_reg := debug_irt_reg + PopCount(rob.io.commit.arch_valids.asUInt)
  dontTouch(debug_tsc_reg)
  dontTouch(debug_irt_reg)

  //****************************************
  // Print-out information about the machine

  val issStr = "(Age-based Priority)"



  val fpPipelineStr = fp_pipeline.toString

  override def toString: String =
    (BoomCoreStringPrefix("====Overall Core Params====") + "\n\n"
    + mem_exe_units.map(_.toString).mkString("") + "\n"
    + unq_exe_units.map(_.toString).mkString("") + "\n\n"
    + alu_exe_units.map(_.toString).mkString("") + "\n\n"
    + fpPipelineStr + "\n\n"
    + rob.toString + "\n\n"
    + BoomCoreStringPrefix(
        "===Other Core Params===",
        "Fetch Width           : " + fetchWidth,
        "Decode Width          : " + coreWidth,
        "Issue Width           : " + issueParams.map(_.issueWidth).sum,
        "ROB Size              : " + numRobEntries,
        "Issue Window Size     : " + issueParams.map(_.numEntries) + issStr,
        "Load/Store Unit Size  : " + numLdqEntries + "/" + numStqEntries,
        "Num Int Phys Registers: " + numIntPhysRegs,
        "Num FP  Phys Registers: " + numFpPhysRegs,
        "Load-to-use delay     : " + (if (enableFastLoadUse) 4 else 5),
        "Max Branch Count      : " + maxBrCount)
    + "\n\n" + iregfile.toString + "\n\n"
    + BoomCoreStringPrefix(
        "Num Wakeup Ports      : " + numIntWakeups,
        "Num Bypass Ports      : " + int_bypasses.length) + "\n"
    + BoomCoreStringPrefix(
        "DCache Ways           : " + dcacheParams.nWays,
        "DCache Sets           : " + dcacheParams.nSets,
        "DCache nMSHRs         : " + dcacheParams.nMSHRs,
        "ICache Ways           : " + icacheParams.nWays,
        "ICache Sets           : " + icacheParams.nSets,
        "D-TLB Ways            : " + dcacheParams.nTLBWays,
        "I-TLB Ways            : " + icacheParams.nTLBWays,
        "Paddr Bits            : " + paddrBits,
        "Vaddr Bits            : " + vaddrBits) + "\n"
    + BoomCoreStringPrefix(
        "Using FPU Unit?       : " + usingFPU.toString,
        "Using FDivSqrt?       : " + usingFDivSqrt.toString,
        "Using VM?             : " + usingVM.toString) + "\n")

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Fetch Stage/Frontend ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  io.ifu.redirect_val         := false.B
  io.ifu.redirect_flush       := false.B

  // Breakpoint info
  io.ifu.status  := csr.io.status
  io.ifu.bp      := csr.io.bp
  io.ifu.mcontext := csr.io.mcontext
  io.ifu.scontext := csr.io.scontext

  io.ifu.flush_icache := (0 until coreWidth).map { i =>
    (rob.io.commit.arch_valids(i) && rob.io.commit.uops(i).is_fencei) ||
    (RegNext(dec_valids(i) && dec_uops(i).is_jalr && csr.io.status.debug))
  }.reduce(_||_)

  // TODO FIX THIS HACK
  // The below code works because of two quirks with the flush mechanism
  //  1 ) All flush_on_commit instructions are also is_unique,
  //      In the future, this constraint will be relaxed.
  //  2 ) We send out flush signals one cycle after the commit signal. We need to
  //      mux between one/two cycle delay for the following cases:
  //       ERETs are reported to the CSR two cycles before we send the flush
  //       Exceptions are reported to the CSR on the cycle we send the flush
  // This discrepency should be resolved elsewhere.
  when (RegNext(rob.io.flush.valid)) {
    io.ifu.redirect_val   := true.B
    io.ifu.redirect_flush := true.B
    val flush_typ = RegNext(rob.io.flush.bits.flush_typ)
    // Clear the global history when we flush the ROB (exceptions, AMOs, unique instructions, etc.)
    val new_ghist = WireInit((0.U).asTypeOf(new GlobalHistory))
    new_ghist.current_saw_branch_not_taken := true.B
    new_ghist.ras_idx := io.ifu.rrd_ftq_resps(0).entry.ras_idx
    io.ifu.redirect_ghist := new_ghist
    when (FlushTypes.useCsrEvec(flush_typ)) {
      io.ifu.redirect_pc  := Mux(flush_typ === FlushTypes.eret,
                                 ShiftRegister(csr.io.evec, 3),
                                 csr.io.evec)
    } .otherwise {
      val flush_pc = (AlignPCToBoundary(io.ifu.rrd_ftq_resps(0).pc, icBlockBytes)
                      + RegNext(rob.io.flush.bits.pc_lob)
                      - Mux(RegNext(rob.io.flush.bits.edge_inst), 2.U, 0.U))
      val flush_pc_next = flush_pc + Mux(RegNext(rob.io.flush.bits.is_rvc), 2.U, 4.U)
      io.ifu.redirect_pc := Mux(FlushTypes.useSamePC(flush_typ),
                                flush_pc, flush_pc_next)

    }
    io.ifu.redirect_ftq_idx := RegNext(rob.io.flush.bits.ftq_idx)
  } .elsewhen (brupdate.b2.mispredict && !RegNext(rob.io.flush.valid)) {
    val block_pc = AlignPCToBoundary(io.ifu.rrd_ftq_resps(0).pc, icBlockBytes)
    val uop_maybe_pc = block_pc | brupdate.b2.uop.pc_lob
    val npc = uop_maybe_pc + Mux(brupdate.b2.uop.is_rvc || brupdate.b2.uop.edge_inst, 2.U, 4.U)
    val jal_br_target = Wire(UInt(vaddrBitsExtended.W))
    jal_br_target := (uop_maybe_pc.asSInt + brupdate.b2.target_offset +
      (Fill(vaddrBitsExtended-1, brupdate.b2.uop.edge_inst) << 1).asSInt).asUInt
    val bj_addr = Mux(brupdate.b2.cfi_type === CFI_JALR, brupdate.b2.jalr_target, jal_br_target)
    val mispredict_target = Mux(brupdate.b2.pc_sel === PC_PLUS4, npc, bj_addr)
    io.ifu.redirect_val     := true.B
    io.ifu.redirect_pc      := mispredict_target
    io.ifu.redirect_flush   := true.B
    io.ifu.redirect_ftq_idx := brupdate.b2.uop.ftq_idx
    val use_same_ghist = (brupdate.b2.cfi_type === CFI_BR &&
                          !brupdate.b2.taken &&
                          bankAlign(block_pc) === bankAlign(npc))
    val ftq_entry = io.ifu.rrd_ftq_resps(0).entry
    val cfi_idx = (brupdate.b2.uop.pc_lob ^
      Mux(ftq_entry.start_bank === 1.U, 1.U << log2Ceil(bankBytes), 0.U))(log2Ceil(fetchWidth), 1)
    val ftq_ghist = io.ifu.rrd_ftq_resps(0).ghist
    val next_ghist = ftq_ghist.update(
      ftq_entry.br_mask.asUInt,
      brupdate.b2.taken,
      brupdate.b2.cfi_type === CFI_BR,
      cfi_idx,
      true.B,
      io.ifu.rrd_ftq_resps(0).pc,
      ftq_entry.cfi_is_call && ftq_entry.cfi_idx.bits === cfi_idx,
      ftq_entry.cfi_is_ret  && ftq_entry.cfi_idx.bits === cfi_idx)


    io.ifu.redirect_ghist   := Mux(
      use_same_ghist,
      ftq_ghist,
      next_ghist)
    io.ifu.redirect_ghist.current_saw_branch_not_taken := use_same_ghist
  } .elsewhen (rob.io.flush_frontend || brupdate.b1.mispredict_mask =/= 0.U) {
    io.ifu.redirect_flush   := true.B
  }

  // Tell the FTQ it can deallocate entries by passing youngest ftq_idx.
  val youngest_com_idx = (coreWidth-1).U - PriorityEncoder(rob.io.commit.valids.reverse)
  io.ifu.commit.valid := rob.io.commit.valids.reduce(_|_) || rob.io.com_xcpt.valid
  io.ifu.commit.bits  := Mux(rob.io.com_xcpt.valid,
                             rob.io.com_xcpt.bits.ftq_idx,
                             rob.io.commit.uops(youngest_com_idx).ftq_idx)

  assert(!(rob.io.commit.valids.reduce(_|_) && rob.io.com_xcpt.valid),
    "ROB can't commit and except in same cycle!")




  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Decode Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // track mask of finished instructions in the bundle
  // use this to mask out insts coming from FetchBuffer that have been finished
  // for example, back pressure may cause us to only issue some instructions from FetchBuffer
  // but on the next cycle, we only want to retry a subset
  val dec_finished_mask = RegInit(0.U(coreWidth.W))

  //-------------------------------------------------------------
  // Pull out instructions and send to the Decoders

  io.ifu.fetchpacket.ready := dec_ready
  val dec_fbundle = io.ifu.fetchpacket.bits

  //-------------------------------------------------------------
  // Decoders

  for (w <- 0 until coreWidth) {
    dec_valids(w)                      := io.ifu.fetchpacket.valid && dec_fbundle.uops(w).valid &&
                                          !dec_finished_mask(w)
    decode_units(w).io.enq.uop         := dec_fbundle.uops(w).bits
    decode_units(w).io.status          := csr.io.status
    decode_units(w).io.csr_decode      <> csr.io.decode(w)
    decode_units(w).io.interrupt       := RegNext(csr.io.interrupt)
    decode_units(w).io.interrupt_cause := RegNext(csr.io.interrupt_cause)
    decode_units(w).io.fcsr_rm         := csr.io.fcsr_rm

    dec_uops(w) := decode_units(w).io.deq.uop
  }

  //-------------------------------------------------------------
  // FTQ GetPC Port Arbitration
  // 3 ports
  // port0 goes to flush,mispredict,xcpt
  // port1/2 goes to jmp unit

  val xcpt_pc_req = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))
  val mispredict_pc_req = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))
  val flush_pc_req = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))

  val ftq_arb = Module(new Arbiter(UInt(log2Ceil(ftqSz).W), 3))

  // Order by the oldest. Flushes come from the oldest instructions in pipe
  // Decoding exceptions come from youngest
  ftq_arb.io.in(0) <> flush_pc_req
  ftq_arb.io.in(1) <> mispredict_pc_req
  ftq_arb.io.in(2) <> xcpt_pc_req

  io.ifu.arb_ftq_reqs(0) := ftq_arb.io.out.bits
  ftq_arb.io.out.ready  := true.B

  val ftq_port_issued = Array.fill(2) { false.B }
  val ftq_port_addrs  = Array.fill(2) { 0.U(log2Ceil(ftqSz).W) }
  for (i <- 0 until aluWidth) {
    for (w <- 0 until 2) {
      val req = alu_exe_units(i).io_arb_ftq_reqs(w)
      var read_issued = false.B
      val data_sel = WireInit(0.U(2.W))
      for (j <- 0 until 2) {
        val issue_read = WireInit(false.B)
        val use_port = WireInit(false.B)
        when (!read_issued && !ftq_port_issued(j) && req.valid) {
          issue_read := true.B
          use_port := true.B
          data_sel := UIntToOH(j.U)
        }
        val was_port_issued_yet = ftq_port_issued(j)
        ftq_port_issued(j) = use_port || ftq_port_issued(j)
        ftq_port_addrs(j) = ftq_port_addrs(j) | Mux(was_port_issued_yet || !use_port, 0.U, req.bits)
        read_issued = issue_read || read_issued
      }
      req.ready := read_issued
      alu_exe_units(i).io_rrd_ftq_resps(w) := Mux(RegNext(data_sel(0)),
        io.ifu.rrd_ftq_resps(1), io.ifu.rrd_ftq_resps(2))

    }
  }
  for (j <- 0 until 2) {
    io.ifu.arb_ftq_reqs(j+1)  := ftq_port_addrs(j)
  }



  // Frontend Exception Requests
  val xcpt_idx = PriorityEncoder(dec_xcpts)
  xcpt_pc_req.valid    := dec_xcpts.reduce(_||_)
  xcpt_pc_req.bits     := dec_uops(xcpt_idx).ftq_idx
  rob.io.xcpt_fetch_pc := io.ifu.rrd_ftq_resps(0).pc

  flush_pc_req.valid   := rob.io.flush.valid
  flush_pc_req.bits    := rob.io.flush.bits.ftq_idx

  // Mispredict requests (to get the correct target)
  mispredict_pc_req.valid := mispredict_val
  mispredict_pc_req.bits  := oldest_mispredict_ftq_idx


  //-------------------------------------------------------------
  // Decode/Rename1 pipeline logic

  dec_xcpts := dec_uops zip dec_valids map {case (u,v) => u.exception && v}
  // Frontend exceptions need to shoot straight past dec/dis without stall, to match
  // the timing of the FTQ resp which provides badaddr.
  // Wait for pipeline to empty before letting an exception past dec
  val dec_prior_slot_valid = dec_valids.scanLeft(false.B) ((s,v) => s || v)
  val dec_xcpt_stall = (0 until coreWidth).map(w => dec_xcpts(w) &&
    (!rob.io.empty || !io.lsu.fencei_rdy || dec_prior_slot_valid(w) || dis_valids.reduce(_||_) || !xcpt_pc_req.ready)
  )
  // stall fetch/dcode because we ran out of branch tags
  val branch_mask_full = Wire(Vec(coreWidth, Bool()))

  val dec_hazards = (0 until coreWidth).map(w =>
                      dec_valids(w) &&
                      (  !dis_ready
                      || rob.io.rollback
                      || dec_xcpt_stall(w)
                      || branch_mask_full(w)
                      || brupdate.b1.mispredict_mask =/= 0.U
                      || brupdate.b2.mispredict
                      || io.ifu.redirect_flush))

  val dec_stalls = dec_hazards.scanLeft(false.B) ((s,h) => s || h).takeRight(coreWidth)
  dec_fire := (0 until coreWidth).map(w => dec_valids(w) && !dec_stalls(w))

  // all decoders are empty and ready for new instructions
  dec_ready := dec_fire.last

  when (dec_ready || io.ifu.redirect_flush) {
    dec_finished_mask := 0.U
  } .otherwise {
    dec_finished_mask := dec_fire.asUInt | dec_finished_mask
  }

  //-------------------------------------------------------------
  // Branch Mask Logic

  dec_brmask_logic.io.brupdate := brupdate
  dec_brmask_logic.io.flush_pipeline := RegNext(rob.io.flush.valid)

  for (w <- 0 until coreWidth) {
    dec_brmask_logic.io.is_branch(w) := !dec_finished_mask(w) && dec_uops(w).allocate_brtag
    dec_brmask_logic.io.will_fire(w) :=  dec_fire(w) &&
                                         dec_uops(w).allocate_brtag // ren, dis can back pressure us
    dec_uops(w).br_tag  := dec_brmask_logic.io.br_tag(w)
    dec_uops(w).br_mask := dec_brmask_logic.io.br_mask(w)
  }

  branch_mask_full := dec_brmask_logic.io.is_full

  //-------------------------------------------------------------
  // Caracal vector decode-stage config (VCFG) + lane_vtype merge + v_emul derive.
  // Gated by usingRVV (gate 9f). Runs after dec_uops are finalized (br_tag/br_mask
  // set above) so the merged vtype/v_emul flow into rename below.
  //-------------------------------------------------------------
  if (usingRVV) {
    val vcfg = vconfig_unit.get

    // D.7 -- rebuild the same Vec(coreWidth+1, Valid(brTag)) the scalar RMT builds
    // internally (rename-stage.scala:117-118): from dis_fire + the dispatched uop's
    // allocate_brtag + br_tag. Slot 0 is the incoming (no-branch) slot.
    val vec_br_tags = Wire(Vec(coreWidth + 1, Valid(UInt(brTagSz.W))))
    vec_br_tags(0).valid := false.B
    vec_br_tags(0).bits  := 0.U
    for (w <- 0 until coreWidth) {
      vec_br_tags(w + 1).valid := dis_fire(w) && dis_uops(w).allocate_brtag
      vec_br_tags(w + 1).bits  := dis_uops(w).br_tag
    }

    // D.2 -- VCFG wiring.
    for (w <- 0 until coreWidth) {
      vcfg.io.dec_valids(w)    := dec_valids(w)
      vcfg.io.dec_is_vset(w)   := dec_uops(w).is_vsetivli || dec_uops(w).is_vsetvli || dec_uops(w).is_vsetvl
      vcfg.io.dec_imm_vtype(w) := dec_uops(w).is_vsetivli || dec_uops(w).is_vsetvli
      vcfg.io.dec_vtype_in(w)  := dec_uops(w).vconfig
      vcfg.io.dec_uop_id(w)    := dec_uops(w).debug_inst // trace tag (best-effort; rob_idx not yet known)
    }
    vcfg.io.ren_br_tags    := vec_br_tags
    vcfg.io.brupdate       := brupdate
    vcfg.io.rollback       := rob.io.rollback
    // ROB commit -> vtype shadow update (Step 5). Collapse the per-lane commit Vec to
    // the single VConfigUnit port, taking the YOUNGEST committing vset (vsetivli/vsetvli
    // aren't is_unique, so 2 can commit/cycle; highest committing lane wins).
    val vset_oh = rob.io.commit.vcfg_vset_valid.get
    vcfg.io.com_vset_valid := vset_oh.reduce(_ || _)
    vcfg.io.com_vtype      := PriorityMux(vset_oh.reverse, rob.io.commit.vcfg_vtype.get.reverse)
    vcfg.io.vec_trace      := false.B    // Step-9 enables the vecTrace plusarg
    // br_carried_vtype: slot 0 sees the speculative mirror; slot w+1 sees lane w's
    // effective (nearest-preceding-vset) vtype.
    vcfg.io.br_carried_vtype(0) := vcfg.io.spec_vtype_out
    for (w <- 0 until coreWidth) {
      vcfg.io.br_carried_vtype(w + 1) := vcfg.io.lane_vtype(w)
    }

    // Merge lane_vtype into vector DATA ops so the mapper derives the right EMUL.
    // last-connect-wins: only overwrites the vconfig of is_vec lanes.
    for (w <- 0 until coreWidth) {
      when (dec_uops(w).is_vec) {
        dec_uops(w).vconfig := vcfg.io.lane_vtype(w)
      }
    }

    // D.3 -- derive the 3-bit dest EMUL (encoding 0..3 = m1..m8; fractional/<=m1 -> 0).
    //   emul_log2 = lmul_log2 + (EEW_log2 - SEW_log2)  for vector load/store
    //             = lmul_log2 + 1                       for widening arith
    //             = lmul_log2                           for normal/narrowing arith
    // then clamp to [0,3]. Only magnitude matters for member sizing, so a fractional
    // LMUL (vlmul 4..7) collapses to lmul_log2 = 0.
    for (w <- 0 until coreWidth) {
      when (dec_uops(w).is_vec) {
        val vt        = dec_uops(w).vconfig // already merged with lane_vtype above
        val sew_log2  = vt.vsew.asSInt        // 3-bit, SEW = 8 << vsew
        val eew_log2  = dec_uops(w).v_eew.asSInt
        // vtype LMUL encoding: 0..3 = m1..m8 (log2 = 0..3); 5,6,7 = mf8,mf4,mf2;
        // anything >= 4 collapses to single-member -> lmul_log2 = 0.
        val lmul_log2 = Mux(vt.vlmul < 4.U, vt.vlmul.asSInt, 0.S)
        val is_ls     = dec_uops(w).iq_type(IQ_V_LOAD) || dec_uops(w).iq_type(IQ_V_STORE)
        val emul_log2 = Mux(is_ls, lmul_log2 + (eew_log2 - sew_log2),
                        Mux(dec_uops(w).v_widen, lmul_log2 + 1.S,
                                                 lmul_log2))
        // clamp to [0,3]
        val clamped = Mux(emul_log2 < 0.S, 0.S,
                      Mux(emul_log2 > 3.S, 3.S, emul_log2))
        dec_uops(w).v_emul := clamped.asUInt(2, 0)
      }
    }

    // D.4 -- drive VecRenameStage + VlRename. Every input port MUST be driven.
    val vrs = vec_rename_stage.get
    val vlr = vl_rename.get

    // Step 11a.2 FIX: the scalar RenameStage has a ren1->ren2 pipeline register
    // (its io.ren2_uops are REGISTERED and its alloc happens at ren2 = dis_fire).
    // VecRenameStage is single-stage (combinational off its dec_uops), so feeding
    // it the dec-stage uops made its outputs a CYCLE AHEAD of the dispatched
    // (registered) scalar uop -- the vec fields (pvdest/stale) then reflected the
    // NEXT cycle's dec_uop (a bubble: lvd=0 -> readGroup(0)=identity stale ->
    // double-free; pvdest = next free-list candidate). Drive it from the scalar
    // rename's REGISTERED ren2 outputs + dis_fire so it allocates/reads in lockstep
    // with dispatch.
    vrs.io.dec_fire    := dis_fire
    vrs.io.dec_valids  := rename_stage.io.ren2_mask
    vrs.io.dec_uops    := rename_stage.io.ren2_uops
    vrs.io.ren_br_tags := vec_br_tags
    vrs.io.brupdate    := brupdate
    vrs.io.rollback    := rob.io.rollback
    vrs.io.kill        := io.ifu.redirect_flush
    vrs.io.dis_fire    := dis_fire
    vrs.io.dis_ready   := dis_ready
    vrs.io.com_valids  := rob.io.commit.valids
    vrs.io.vec_trace   := false.B    // Step-9 enables the vecTrace plusarg
    for (w <- 0 until coreWidth) {
      vrs.io.dec_uop_id(w) := rename_stage.io.ren2_uops(w).debug_inst
    }
    // Commit-free path (Step 5): map-table remap + free-list dealloc from the ROB.
    vrs.io.com_remap   := rob.io.commit.vec_remap.get
    vrs.io.com_dealloc := rob.io.commit.vec_dealloc.get
    // Step 11a.2: the vector LSU group-done is the first real busy-clear source.
    // Drive wakeup port 0 from VecLSU.group_done (clears dest-group busy in the
    // vector busy table); remaining ports stay invalid until the CII (Goal 2).
    vrs.io.wakeups := DontCare
    for (k <- 0 until vrs.io.wakeups.length) {
      vrs.io.wakeups(k).valid := false.B
    }
    vrs.io.wakeups(0) := vec_lsu.get.io.group_done

    // Step 11a.2 FIX (same ren2 alignment as vrs above): drive VlRename from the
    // scalar rename's REGISTERED ren2 outputs + dis_fire so the VL alloc/read is
    // in lockstep with dispatch (was a cycle ahead off dec_uops).
    vlr.io.dec_fire    := dis_fire
    vlr.io.dec_valids  := rename_stage.io.ren2_mask
    vlr.io.dec_uops    := rename_stage.io.ren2_uops
    vlr.io.ren_br_tags := vec_br_tags
    vlr.io.brupdate    := brupdate
    vlr.io.rollback    := rob.io.rollback
    vlr.io.kill        := io.ifu.redirect_flush
    vlr.io.com_valids  := rob.io.commit.valids
    vlr.io.vec_trace   := false.B    // Step-9 enables the vecTrace plusarg
    for (w <- 0 until coreWidth) {
      vlr.io.dec_uop_id(w)    := rename_stage.io.ren2_uops(w).debug_inst
      vlr.io.com_is_vlprod(w) := rob.io.commit.vl_is_vlprod.get(w)    // ROB commit -> VL free (Step 5)
      vlr.io.com_pvl(w)       := rob.io.commit.vl_com_pvl.get(w)
    }
    // VL wakeup (Step 9): the int-ALU vset write is the sole VL producer. Drive wakeup
    // port 0 from the collapsed vset response (pvl just written into the VL-RF); other
    // wakeup ports stay invalid. This clears pvl_busy for younger vector consumers.
    vlr.io.wakeups := DontCare
    for (k <- 0 until vlr.io.wakeups.length) {
      vlr.io.wakeups(k).valid := false.B
    }
    vlr.io.wakeups(0).valid    := vset_wb.get.valid
    vlr.io.wakeups(0).bits.pvl := vset_wb.get.bits.pvl

    // ROB vector group-done completion ports (Step 11a.2): VecLSU clears the
    // completing vle's rob_bsy via port 0 (rob_idx-based); other ports stay
    // invalid until the CII (Goal 2). This is the commit path for vector loads
    // (they hold no LDQ entry -- VDecode sets uses_ldq=false).
    rob.io.vec_clr_bsy.get := DontCare
    for (k <- 0 until rob.io.vec_clr_bsy.get.length) {
      rob.io.vec_clr_bsy.get(k).valid := false.B
    }
    rob.io.vec_clr_bsy.get(0) := vec_lsu.get.io.clr_rob
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Register Rename Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Inputs
  for (rename <- rename_stages) {
    rename.io.kill := io.ifu.redirect_flush
    rename.io.brupdate := brupdate

    rename.io.debug_rob_empty := rob.io.empty

    rename.io.dec_fire := dec_fire
    rename.io.dec_uops := dec_uops

    rename.io.dis_fire := dis_fire
    rename.io.dis_ready := dis_ready

    rename.io.com_valids := rob.io.commit.valids
    rename.io.com_uops := rob.io.commit.uops
    rename.io.rollback := rob.io.rollback
  }



  // Outputs
  dis_uops := rename_stage.io.ren2_uops
  dis_valids := rename_stage.io.ren2_mask
  ren_stalls := rename_stage.io.ren_stalls


  /**
   * TODO This is a bit nasty, but it's currently necessary to
   * split the INT/FP rename pipelines into separate instantiations.
   * Won't have to do this anymore with a properly decoupled FP pipeline.
   */
  for (w <- 0 until coreWidth) {
    val i_uop     = rename_stage.io.ren2_uops(w)
    val f_uop     = fp_rename_stage.io.ren2_uops(w)
    val p_uop     = pred_rename_stage.io.ren2_uops(w)
    val imm_uop   = imm_rename_stage.io.ren2_uops(w)
    val f_stall   = fp_rename_stage.io.ren_stalls(w)
    val p_stall   = pred_rename_stage.io.ren_stalls(w)
    val imm_stall = imm_rename_stage.io.ren_stalls(w)

    // lrs1 can "pass through" to prs1. Used solely to index the csr file.
    dis_uops(w).prs1 := Mux(dis_uops(w).lrs1_rtype === RT_FLT, f_uop.prs1,
                        Mux(dis_uops(w).lrs1_rtype === RT_FIX, i_uop.prs1, dis_uops(w).lrs1))
    dis_uops(w).prs2 := Mux(dis_uops(w).lrs2_rtype === RT_FLT, f_uop.prs2, i_uop.prs2)
    dis_uops(w).prs3 := f_uop.prs3
    dis_uops(w).ppred := p_uop.ppred
    dis_uops(w).pdst := Mux(dis_uops(w).dst_rtype  === RT_FLT, f_uop.pdst,
                        Mux(dis_uops(w).dst_rtype  === RT_FIX, i_uop.pdst,
                        Mux(dis_uops(w).is_sfb_br            , p_uop.pdst,
                                                               random.LFSR(maxPregSz) // Random set the PDST so the ALU banked IQ is balanced
                        )))
    dis_uops(w).imm_sel := imm_uop.imm_sel
    dis_uops(w).pimm    := imm_uop.pimm

    dis_uops(w).stale_pdst := Mux(dis_uops(w).dst_rtype === RT_FLT, f_uop.stale_pdst, i_uop.stale_pdst)

    dis_uops(w).prs1_busy := i_uop.prs1_busy && (dis_uops(w).lrs1_rtype === RT_FIX) ||
                             f_uop.prs1_busy && (dis_uops(w).lrs1_rtype === RT_FLT)
    dis_uops(w).prs2_busy := i_uop.prs2_busy && (dis_uops(w).lrs2_rtype === RT_FIX) ||
                             f_uop.prs2_busy && (dis_uops(w).lrs2_rtype === RT_FLT)
    dis_uops(w).prs3_busy := f_uop.prs3_busy && dis_uops(w).frs3_en
    dis_uops(w).ppred_busy := p_uop.ppred_busy && dis_uops(w).is_sfb_shadow

    ren_stalls(w) := rename_stage.io.ren_stalls(w) || f_stall || p_stall || imm_stall
  }

  //-------------------------------------------------------------
  // Caracal vector dis_uops assembly + ren_stalls fold (Step 4).
  // Gated by usingRVV (gate 9f). Placed AFTER the scalar dis_uops / ren_stalls
  // blocks so last-connect-wins overrides the vector fields for is_vec uops. The
  // scalar prs1/prs2/prs3/pdst Mux is intentionally NOT touched -- a vector op's
  // scalar base/stride still take prs1/prs2 from the int-rename Mux.
  //-------------------------------------------------------------
  if (usingRVV) {
    for (w <- 0 until coreWidth) {
      val v_uop  = vec_rename_stage.get.io.ren2_uops(w)
      val vl_uop = vl_rename.get.io.ren2_uops(w)

      // D.5 -- vector group/source/dest fields from the vector mapper + VL rename.
      when (dis_uops(w).is_vec) {
        dis_uops(w).pvdest           := v_uop.pvdest
        dis_uops(w).pvdest_grp       := v_uop.pvdest_grp
        dis_uops(w).pvdest_grp_mask  := v_uop.pvdest_grp_mask
        dis_uops(w).stale_pvdest     := v_uop.stale_pvdest
        dis_uops(w).stale_pvdest_grp := v_uop.stale_pvdest_grp
        dis_uops(w).pvs1             := v_uop.pvs1
        dis_uops(w).pvs1_grp         := v_uop.pvs1_grp
        dis_uops(w).pvs2             := v_uop.pvs2
        dis_uops(w).pvs2_grp         := v_uop.pvs2_grp
        dis_uops(w).pvs3             := v_uop.pvs3
        dis_uops(w).pvs3_grp         := v_uop.pvs3_grp
        dis_uops(w).pvm              := v_uop.pvm
        dis_uops(w).pvtmp            := v_uop.pvtmp
        dis_uops(w).pvtmp_mask       := v_uop.pvtmp_mask
        dis_uops(w).pvs1_busy        := v_uop.pvs1_busy
        dis_uops(w).pvs2_busy        := v_uop.pvs2_busy
        dis_uops(w).pvs3_busy        := v_uop.pvs3_busy
        dis_uops(w).pvm_busy         := v_uop.pvm_busy
        dis_uops(w).pvl              := vl_uop.pvl
        dis_uops(w).pvl_busy         := vl_uop.pvl_busy
        dis_uops(w).v_emul           := v_uop.v_emul
      }

      // VL PRODUCERS (vsetvl*) are scalar (is_vec=false), so the is_vec merge
      // above skips them -- but they allocate a VL dest pvl in VlRename. Give them
      // that pvl so the int-ALU's vset writeback (vset_out.bits.pvl := uop.pvl,
      // functional-unit.scala) lands on the VL-RF entry that younger vector
      // consumers read (Step 11a.2). Without this the producer's pvl stayed 0
      // while consumers read the freshly-allocated pvl -> stale/garbage vl.
      val is_vl_prod = dis_uops(w).is_vsetivli || dis_uops(w).is_vsetvli || dis_uops(w).is_vsetvl
      when (is_vl_prod) {
        dis_uops(w).pvl := vl_uop.pvl
      }

      // D.6 -- fold the vector rename stalls into the per-lane ren_stalls. Re-list
      // the scalar stage stalls (rather than reading ren_stalls(w), which would be a
      // combinational self-loop x := x || y) and OR in the vector terms. Mirrors the
      // scalar assignment at the top of this section; last-connect-wins overrides it.
      ren_stalls(w) := rename_stage.io.ren_stalls(w) || fp_rename_stage.io.ren_stalls(w) ||
                       pred_rename_stage.io.ren_stalls(w) || imm_rename_stage.io.ren_stalls(w) ||
                       vec_rename_stage.get.io.ren_stalls(w) || vl_rename.get.io.ren_stalls(w)
    }
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Dispatch Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  //-------------------------------------------------------------
  // Rename2/Dispatch pipeline logic

  val dis_prior_slot_valid = dis_valids.scanLeft(false.B) ((s,v) => s || v)
  val dis_prior_slot_unique = (dis_uops zip dis_valids).scanLeft(false.B) {case (s,(u,v)) => s || v && u.is_unique}
  val wait_for_empty_pipeline = (0 until coreWidth).map(w => (dis_uops(w).is_unique || !custom_csrs.enableOOO) &&
                                  (!rob.io.empty || !io.lsu.fencei_rdy || dis_prior_slot_valid(w)))
  val rocc_shim_busy = if (usingRoCC) !unq_exe_unit.io_rocc_core.get.rxq_empty else false.B
  val wait_for_rocc = (0 until coreWidth).map(w =>
                        (dis_uops(w).is_fence || dis_uops(w).is_fencei) && (io.rocc.busy || rocc_shim_busy))
  val rxq_full = if (usingRoCC) unq_exe_unit.io_rocc_core.get.rxq_full else false.B
  val block_rocc = (dis_uops zip dis_valids).map{case (u,v) => v && u.is_rocc}.scanLeft(rxq_full)(_||_)
  val dis_rocc_alloc_stall = (dis_uops.map(_.is_rocc) zip block_rocc) map {case (p,r) =>
                               if (usingRoCC) p && r else false.B}

  // Only 1 branch-tag allocating insruction allowed to proceed per cycle unless enableSuperscalarSnapshots
  // This reduces checkpointing complexity
  val block_brtag = (dis_uops zip dis_valids).map{case (u,v) => v && u.allocate_brtag}.scanLeft(false.B)(_||_)
  val brtag_stall = (dis_uops.map(_.allocate_brtag) zip block_brtag) map {case (p,r) =>
    if (enableSuperscalarSnapshots) false.B else (p && r) }
  val dis_hazards = (0 until coreWidth).map(w =>
                      dis_valids(w) &&
                      (  !rob.io.ready
                      || brtag_stall(w)
                      || ren_stalls(w)
                      || io.lsu.ldq_full(w) && dis_uops(w).uses_ldq
                      || io.lsu.stq_full(w) && dis_uops(w).uses_stq
                      || !dispatcher.io.ren_uops(w).ready
                      || wait_for_empty_pipeline(w)
                      || wait_for_rocc(w)
                      || dis_prior_slot_unique(w)
                      || dis_rocc_alloc_stall(w)
                      || brupdate.b1.mispredict_mask =/= 0.U
                      || brupdate.b2.mispredict
                      || io.ifu.redirect_flush))


  io.lsu.fence_dmem := (dis_valids zip wait_for_empty_pipeline).map {case (v,w) => v && w} .reduce(_||_)

  val dis_stalls = dis_hazards.scanLeft(false.B) ((s,h) => s || h).takeRight(coreWidth)
  dis_fire := dis_valids zip dis_stalls map {case (v,s) => v && !s}
  dis_ready := !dis_stalls.last

  //-------------------------------------------------------------
  // LDQ/STQ Allocation Logic

  for (w <- 0 until coreWidth) {
    // Dispatching instructions request load/store queue entries when they can proceed.
    dis_uops(w).ldq_idx := io.lsu.dis_ldq_idx(w)
    dis_uops(w).stq_idx := io.lsu.dis_stq_idx(w)
  }

  //-------------------------------------------------------------
  // Rob Allocation Logic

  rob.io.enq_valids := dis_fire
  rob.io.enq_uops   := dis_uops
  rob.io.enq_partial_stall := dis_stalls.last // TODO come up with better ROB compacting scheme.
  rob.io.debug_tsc := debug_tsc_reg
  rob.io.csr_stall := csr.io.csr_stall
  rob.io.trace_stall := io.traceStall

  // Minor hack: ecall and breaks need to increment the FTQ deq ptr earlier than commit, since
  // they write their PC into the CSR the cycle before they commit.
  // Since these are also unique, increment the FTQ ptr when they are dispatched
  when (RegNext(dis_fire.reduce(_||_) && dis_uops(PriorityEncoder(dis_fire)).is_sys_pc2epc)) {
    io.ifu.commit.valid := true.B
    io.ifu.commit.bits  := RegNext(dis_uops(PriorityEncoder(dis_valids)).ftq_idx)
  }

  for (w <- 0 until coreWidth) {
    // note: this assumes uops haven't been shifted - there's a 1:1 match between PC's LSBs and "w" here
    // (thus the LSB of the rob_idx gives part of the PC)
    if (coreWidth == 1) {
      dis_uops(w).rob_idx := rob.io.rob_tail_idx
    } else {
      dis_uops(w).rob_idx := Cat(rob.io.rob_tail_idx >> log2Ceil(coreWidth).U,
                               w.U(log2Ceil(coreWidth).W))
    }
  }

  //-------------------------------------------------------------
  // RoCC allocation logic
  if (usingRoCC) {
    val rocc_unit = unq_exe_unit
    for (w <- 0 until coreWidth) {
      // We guarantee only decoding 1 RoCC instruction per cycle
      dis_uops(w).rxq_idx := rocc_unit.io_rocc_core.get.rxq_idx(w)
    }
  }

  //-------------------------------------------------------------
  // Dispatch to issue queues

  // Get uops from rename2
  for (w <- 0 until coreWidth) {
    dispatcher.io.ren_uops(w).valid := dis_fire(w)
    dispatcher.io.ren_uops(w).bits  := dis_uops(w)
  }

  var iu_idx = 0
  // Send dispatched uops to correct issue queues
  // Backpressure through dispatcher if necessary
  for (i <- 0 until issueParams.size) {
    if (issueParams(i).iqType == IQ_FP) {
      fp_pipeline.io.dis_uops <> dispatcher.io.dis_uops(i)
    } else if (issueParams(i).iqType == IQ_MEM) {
      mem_iss_unit.io.dis_uops <> dispatcher.io.dis_uops(i)
    } else if (issueParams(i).iqType == IQ_ALU) {
      alu_iss_unit.io.dis_uops <> dispatcher.io.dis_uops(i)
    } else if (issueParams(i).iqType == IQ_UNQ) {
      unq_iss_unit.io.dis_uops <> dispatcher.io.dis_uops(i)
    } else if (issueParams(i).iqType == IQ_V_LOAD) {
      // Caracal (Step 6): only present when usingRVV, so .get is safe.
      vload_iss_unit.get.io.dis_uops <> dispatcher.io.dis_uops(i)
    } else if (issueParams(i).iqType == IQ_V_STORE) {
      vstore_iss_unit.get.io.dis_uops <> dispatcher.io.dis_uops(i)
    } else if (issueParams(i).iqType == IQ_V_ALU) {
      valu_iss_unit.get.io.dis_uops <> dispatcher.io.dis_uops(i)
    } else {
      require(false)
    }
  }

  //-------------------------------------------------------------
  // Write immediates, branches into immediate file
  for (w <- 0 until coreWidth) {
    val uop = RegNext(dis_uops(w))

    immregfile.io.write_ports(w).valid     := RegNext(dis_fire(w)) && !uop.imm_sel.isOneOf(IS_N, IS_SH)
    immregfile.io.write_ports(w).bits.addr := uop.pimm
    immregfile.io.write_ports(w).bits.data := uop.imm_packed

    bregfile.io.write_ports(w).valid               := RegNext(dis_fire(w)) && uop.allocate_brtag
    bregfile.io.write_ports(w).bits.addr           := uop.br_tag
    bregfile.io.write_ports(w).bits.data.ldq_idx   := uop.ldq_idx
    bregfile.io.write_ports(w).bits.data.stq_idx   := uop.stq_idx
    bregfile.io.write_ports(w).bits.data.rxq_idx   := uop.rxq_idx
  }


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Issue Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  var wu_idx = 0
  var wb_idx = 0
  var bypass_idx = 0
  for (i <- 0 until lsuWidth) {
    int_wakeups(wu_idx) := io.lsu.iwakeups(i)
    rob.io.wb_resps(wb_idx) := RegNext(UpdateBrMask(brupdate, RegNext(rob.io.flush.valid), io.lsu.iresp(i)))
    iregfile.io.write_ports(wb_idx).valid := RegNext(io.lsu.iresp(i).valid)
    iregfile.io.write_ports(wb_idx).bits.addr := RegNext(io.lsu.iresp(i).bits.uop.pdst)
    iregfile.io.write_ports(wb_idx).bits.data := RegNext(io.lsu.iresp(i).bits.data)

    int_bypasses(bypass_idx).valid := RegNext(io.lsu.iresp(i).valid)
    int_bypasses(bypass_idx).bits  := RegNext(io.lsu.iresp(i).bits)
    wu_idx += 1
    wb_idx += 1
    bypass_idx += 1
  }
  val ll_arb = Module(new Arbiter(new ExeUnitResp(xLen),
    1 + // Mul
    1 + // F2I
    (if (unq_exe_unit.hasDiv) 1 else 0) +
    (if (unq_exe_unit.hasCSR) 1 else 0) +
    (if (unq_exe_unit.hasRocc) 1 else 0)))

  var arb_idx = 0
  ll_arb.io.in(arb_idx).valid := unq_exe_unit.io_mul_resp.get.valid
  ll_arb.io.in(arb_idx).bits  := unq_exe_unit.io_mul_resp.get.bits
  arb_idx += 1
  ll_arb.io.in(arb_idx) <> fp_pipeline.io.to_int
  arb_idx += 1
  if (unq_exe_unit.hasDiv) {
    ll_arb.io.in(arb_idx) <> unq_exe_unit.io_div_resp.get
    arb_idx += 1
  }
  if (unq_exe_unit.hasCSR) {
    ll_arb.io.in(arb_idx).valid     := csr_resp.valid
    ll_arb.io.in(arb_idx).bits.uop  := csr_resp.bits.uop
    val rdata = WireInit(csr.io.rw.rdata)
    if (enableBPDHPMs) {
      var hpmcounter = CSRs.hpmcounter16
      var mhpmcounter = CSRs.mhpmcounter16
      for (i <- 0 until 5) {
        when (csr.io.rw.addr === hpmcounter.U || csr.io.rw.addr === mhpmcounter.U) {
          rdata := debug_brs(i)
        }
        hpmcounter = hpmcounter + 1
        mhpmcounter = mhpmcounter + 1
        when (csr.io.rw.addr === hpmcounter.U || csr.io.rw.addr === mhpmcounter.U) {
          rdata := debug_jals(i)
        }
        hpmcounter = hpmcounter + 1
        mhpmcounter = mhpmcounter + 1
        when (csr.io.rw.addr === hpmcounter.U || csr.io.rw.addr === mhpmcounter.U) {
          rdata := debug_jalrs(i)
        }
        hpmcounter = hpmcounter + 1
        mhpmcounter = mhpmcounter + 1
      }
    }
    ll_arb.io.in(arb_idx).bits.data := rdata
    ll_arb.io.in(arb_idx).bits.predicated   := false.B
    ll_arb.io.in(arb_idx).bits.fflags.valid := false.B
    ll_arb.io.in(arb_idx).bits.fflags.bits  := false.B
    assert(!(ll_arb.io.in(arb_idx).valid && !ll_arb.io.in(arb_idx).ready))
    arb_idx += 1
  }
  if (unq_exe_unit.hasRocc) {
    ll_arb.io.in(arb_idx) <> unq_exe_unit.io_rocc_resp.get
    arb_idx += 1
  }
  ll_arb.io.out.ready := true.B
  int_wakeups(wu_idx).valid := ll_arb.io.out.valid && ll_arb.io.out.bits.uop.dst_rtype === RT_FIX
  int_wakeups(wu_idx).bits.uop  := ll_arb.io.out.bits.uop
  int_wakeups(wu_idx).bits.speculative_mask := 0.U
  int_wakeups(wu_idx).bits.rebusy := false.B
  int_wakeups(wu_idx).bits.bypassable := false.B
  wu_idx += 1

  rob.io.wb_resps(wb_idx).valid  := RegNext(ll_arb.io.out.valid && !IsKilledByBranch(brupdate, RegNext(rob.io.flush.valid), ll_arb.io.out.bits))
  rob.io.wb_resps(wb_idx).bits   := RegNext(ll_arb.io.out.bits)

  iregfile.io.write_ports(wb_idx).valid     := ll_arb.io.out.valid && ll_arb.io.out.bits.uop.dst_rtype === RT_FIX
  iregfile.io.write_ports(wb_idx).bits.addr := ll_arb.io.out.bits.uop.pdst
  iregfile.io.write_ports(wb_idx).bits.data := ll_arb.io.out.bits.data
  wb_idx += 1


  // loop through each issue-port (exe_units are statically connected to an issue-port)
  for (i <- 0 until aluWidth) {
    val unit = alu_exe_units(i)
    val fast_wakeup = unit.io_fast_wakeup

    int_bypasses(bypass_idx).valid := unit.io_alu_resp.valid && unit.io_alu_resp.bits.uop.dst_rtype === RT_FIX
    int_bypasses(bypass_idx).bits  := unit.io_alu_resp.bits
    bypass_idx += 1

    int_wakeups(wu_idx) := fast_wakeup
    wu_idx += 1

    rob.io.wb_resps(wb_idx).valid  := RegNext(unit.io_alu_resp.valid && !IsKilledByBranch(brupdate, RegNext(rob.io.flush.valid), unit.io_alu_resp.bits))
    rob.io.wb_resps(wb_idx).bits   := RegNext(unit.io_alu_resp.bits)

    iregfile.io.write_ports(wb_idx).valid     := unit.io_alu_resp.valid && unit.io_alu_resp.bits.uop.dst_rtype === RT_FIX
    iregfile.io.write_ports(wb_idx).bits.addr := unit.io_alu_resp.bits.uop.pdst
    iregfile.io.write_ports(wb_idx).bits.data := unit.io_alu_resp.bits.data
    wb_idx += 1

    pred_wakeups(i) := unit.io_fast_pred_wakeup


  }
  require (wu_idx == numIntWakeups)
  require (wb_idx == numIrfWritePorts)

  // ----------------------------------------------------------------
  // Caracal (Step 9): collapse the int-ALU vset writeback responses. Each ALUExeUnit
  // exposes io_vset_wb (Option[Valid[VsetWbResp]], present iff usingRVV). Only the lane
  // running the vset FU asserts valid in a given cycle, so OR the valids and Mux1H the
  // bits across lanes. Gated by usingRVV; .get is safe inside the gate.
  if (usingRVV) {
    val vset_valids = alu_exe_units.map(_.io_vset_wb.get.valid)
    vset_wb.get.valid := vset_valids.reduce(_ || _)
    vset_wb.get.bits  := Mux1H(vset_valids, alu_exe_units.map(_.io_vset_wb.get.bits))
    assert(PopCount(VecInit(vset_valids).asUInt) <= 1.U, "more than one ALU lane produced a vset writeback")

    // ROB vl/vtype stash port (Input Vec(coreWidth, Valid[VsetWbResp])). The int ALU maps to
    // lane 0; the rob_idx inside the response selects the ROB row, so the lane index is only a
    // structural port assignment. Drive every lane: lane 0 = collapsed vset, others invalid.
    for (w <- 0 until coreWidth) {
      if (w == 0) {
        rob.io.vset_wb.get(0) := vset_wb.get
      } else {
        rob.io.vset_wb.get(w).valid := false.B
        rob.io.vset_wb.get(w).bits  := DontCare
      }
    }

    // ----------------------------------------------------------------
    // Caracal (Step 9): architectural vconfig write into the rocket CSRFile. With Option A
    // (usingVector=true when usingRVV), csr.io.vector is Some(...). The ROB drives per-lane
    // commit signals; collapse them to the YOUNGEST committing vset (vsetivli/vsetvli are
    // not is_unique, so up to retireWidth can commit/cycle -- highest committing lane wins),
    // mirroring the VCFG com wiring above. Every INPUT sub-field of io.vector is driven.
    val csr_vec      = csr.io.vector.get
    val csr_vset_oh  = rob.io.commit.csr_vset_valid.get
    val win_vcfg     = PriorityMux(csr_vset_oh.reverse, rob.io.commit.csr_vconfig.get.reverse)
    val win_vl       = PriorityMux(csr_vset_oh.reverse, rob.io.commit.csr_vl.get.reverse)

    csr_vec.set_vconfig.valid    := csr_vset_oh.reduce(_ || _)
    csr_vec.set_vconfig.bits.vl  := win_vl
    // Bit-exact bridge BOOM VConfig -> rocket VType. The rocket VType packs (MSB..LSB):
    //   {vill, reserved, vma, vta, vsew[2:0], vlmul_sign, vlmul_mag[1:0]}
    // so its low byte == {vma, vta, vsew[2:0], vlmul[2:0]} -- exactly the vtype immediate
    // byte. Reconstruct via VType.fromUInt on the packed byte Cat(vma, vta, vsew, vlmul);
    // for a legal config fromUInt leaves vill=0/reserved=0 and copies the fields verbatim,
    // so set_vconfig.bits.vtype.asUInt's low byte round-trips the original immediate.
    //   e32/m1/ta/ma -> vsew=2(010), vlmul=0(000), vta=1, vma=1
    //   => Cat(1,1,010,000) = 0b11010000 = 0xD0  (matches the smoke's `bne t1, 0xD0`).
    val packed_vtype = Cat(win_vcfg.vma, win_vcfg.vta, win_vcfg.vsew, win_vcfg.vlmul)
    csr_vec.set_vconfig.bits.vtype := freechips.rocketchip.rocket.VType.fromUInt(packed_vtype, true)

    // Remaining io.vector INPUT fields: no architectural vstart/vxsat update from the vset
    // path. (set_vstart/set_vconfig are Flipped(Valid), set_vs_dirty/set_vxsat are Input Bool.)
    csr_vec.set_vstart.valid := false.B
    csr_vec.set_vstart.bits  := 0.U
    csr_vec.set_vs_dirty     := false.B
    csr_vec.set_vxsat        := false.B

    // ---- Caracal vector-CSR waveform taps (dontTouch so they survive opt). ----
    // Search these names in Verdi/DVE. Architectural readback (== `csrr vl/vtype/
    // vstart/vxrm`); these mirror rocket CSRFile reg_vconfig/reg_vstart/reg_vxrm.
    val vec_arch_vl     = dontTouch(WireInit(csr_vec.vconfig.vl))
    val vec_arch_vtype  = dontTouch(WireInit(csr_vec.vconfig.vtype.asUInt))
    val vec_arch_vstart = dontTouch(WireInit(csr_vec.vstart))
    val vec_arch_vxrm   = dontTouch(WireInit(csr_vec.vxrm))
    // Commit-time vset write into the CSR (pulses for one cycle when a vset retires):
    val vec_set_valid   = dontTouch(WireInit(csr_vec.set_vconfig.valid))
    val vec_set_vl      = dontTouch(WireInit(win_vl))
    val vec_set_vtype   = dontTouch(WireInit(packed_vtype))
    // Physical/renamed VL produced by the int-ALU vset (before commit): which VL preg
    // (pvl) and the VL value written into the VlRegFile.
    val vec_pvl         = dontTouch(WireInit(vset_wb.get.bits.pvl))
    val vec_pvl_value   = dontTouch(WireInit(vset_wb.get.bits.vl_value))
    val vec_pvl_wr      = dontTouch(WireInit(vset_wb.get.valid))
  }

  // arb stage guarantees 1 preg writer per cycle
  val pregfile_write_valids = alu_exe_units.map(u => u.io_alu_resp.valid && u.io_alu_resp.bits.uop.is_sfb_br)
  assert(PopCount(pregfile_write_valids) <= 1.U)
  pregfile.io.write_ports(0).valid := pregfile_write_valids.reduce(_||_)
  pregfile.io.write_ports(0).bits.addr := Mux1H(pregfile_write_valids, alu_exe_units.map(_.io_alu_resp.bits.uop.pdst))
  pregfile.io.write_ports(0).bits.data := Mux1H(pregfile_write_valids, alu_exe_units.map(_.io_alu_resp.bits.data))

  // Connect the predicate wakeup port
  alu_iss_unit.io.pred_wakeup_port.valid := pred_wakeup.valid
  alu_iss_unit.io.pred_wakeup_port.bits  := pred_wakeup.bits.uop.pdst
  mem_iss_unit.io.pred_wakeup_port.valid := false.B
  mem_iss_unit.io.pred_wakeup_port.bits  := DontCare
  unq_iss_unit.io.pred_wakeup_port.valid := false.B
  unq_iss_unit.io.pred_wakeup_port.bits  := DontCare



  // ----------------------------------------------------------------
  // Connect the wakeup ports to the busy tables in the rename stages

  for ((renport, intport) <- rename_stage.io.wakeups zip int_wakeups) {
    renport <> intport
  }
  if (usingFPU) {
    fp_rename_stage.io.wakeups := fp_pipeline.io.wakeups
  }

  pred_rename_stage.io.wakeups(0) := pred_wakeup
  imm_rename_stage.io.wakeups := all_exe_units.map(_.io_rrd_immrf_wakeup)

  rename_stage.io.child_rebusys := alu_exe_units.map(_.io_child_rebusy).reduce(_|_)
  imm_rename_stage.io.child_rebusys := 0.U
  pred_rename_stage.io.child_rebusys := 0.U
  fp_rename_stage.io.child_rebusys := 0.U

  mem_iss_unit.io.fu_types := mem_exe_units.map(_.io_ready_fu_types)
  alu_iss_unit.io.fu_types := alu_exe_units.map(_.io_ready_fu_types)
  unq_iss_unit.io.fu_types := unq_exe_units.map(_.io_ready_fu_types)

  for (iss_unit <- Seq(mem_iss_unit, alu_iss_unit, unq_iss_unit)) {
    iss_unit.io.tsc_reg  := debug_tsc_reg
    iss_unit.io.brupdate := brupdate
    iss_unit.io.flush_pipeline := RegNext(rob.io.flush.valid)

    // Rebusy children of misspeculated wakeups
    iss_unit.io.child_rebusys := alu_exe_units.map(_.io_child_rebusy).reduce(_|_)

    iss_unit.io.wakeup_ports := int_wakeups

    iss_unit.io.rob_pnr_idx := rob.io.rob_pnr_idx
    iss_unit.io.rob_head    := rob.io.rob_head_idx
  }

  mem_iss_unit.io.squash_grant := (
    mem_exe_units.map(_.io_squash_iss).reduce(_||_) ||
    alu_exe_units.map(_.io_squash_iss).reduce(_||_) ||
    io.lsu.iwakeups.map(_.bits.rebusy).reduce(_||_)
  )
  unq_iss_unit.io.squash_grant := (
    unq_exe_units.map(_.io_squash_iss).reduce(_||_) ||
    alu_exe_units.map(_.io_squash_iss).reduce(_||_) ||
    io.lsu.iwakeups.map(_.bits.rebusy).reduce(_||_)
  )
  alu_iss_unit.io.squash_grant := (
    alu_exe_units.map(_.io_squash_iss).reduce(_||_) ||
    io.lsu.iwakeups.map(_.bits.rebusy).reduce(_||_)
  )

  mem_iss_unit.io.iss_uops zip mem_exe_units map { case (i, u) => u.io_iss_uop := i }
  alu_iss_unit.io.iss_uops zip alu_exe_units map { case (i, u) => u.io_iss_uop := i }
  unq_iss_unit.io.iss_uops zip unq_exe_units map { case (i, u) => u.io_iss_uop := i }

  // ----------------------------------------------------------------
  // Caracal (Step 6): wire the three vector issue units DORMANT. Mirrors the
  // scalar common wiring above. Every input is driven (Chisel errors otherwise).
  // fu_types=0 (per issue port) and all-invalid wakeup networks guarantee no
  // grant -> the units never issue. iss_uops outputs are left dangling: there is
  // no consumer yet (LSU in Step 11, CII in Step 12), which is legal in Chisel.
  // All gated by usingRVV so vector-OFF RTL is byte-identical (gate 9f).
  if (usingRVV) {
    for (iss_unit <- Seq(vload_iss_unit.get, vstore_iss_unit.get, valu_iss_unit.get)) {
      // Mirror the exact RHS expressions the scalar units are given.
      iss_unit.io.tsc_reg        := debug_tsc_reg
      iss_unit.io.brupdate       := brupdate
      iss_unit.io.flush_pipeline := RegNext(rob.io.flush.valid)
      iss_unit.io.child_rebusys  := alu_exe_units.map(_.io_child_rebusy).reduce(_|_)

      // INT feeders (base / stride / .vx scalar operands).
      iss_unit.io.wakeup_ports := int_wakeups

      // No predicate consumer (mirror scalar mem/unq units).
      iss_unit.io.pred_wakeup_port.valid := false.B
      iss_unit.io.pred_wakeup_port.bits  := DontCare

      // SNI gating (used by IQ_V_ALU; ignored by the LS collapsing units).
      iss_unit.io.rob_pnr_idx := rob.io.rob_pnr_idx
      iss_unit.io.rob_head    := rob.io.rob_head_idx

      // DORMANCY: no vector EU exists yet -> never advertise any ready FU.
      iss_unit.io.fu_types.foreach(_.foreach(_ := false.B))

      // No grant squash source yet.
      iss_unit.io.squash_grant := false.B

      // Vector wakeup network: port 0 carries the VecLSU group-done so a younger
      // vector op waiting on this load's dest group is woken (Step 11a.2). Other
      // ports stay invalid until the CII (Goal 2).
      iss_unit.io.vec_wakeup_ports.foreach { p => p.valid := false.B; p.bits := DontCare }
      iss_unit.io.vec_wakeup_ports(0) := vec_lsu.get.io.group_done
      // VL wakeup (Step 9): drive port 0 from the int-ALU vset response so pvl_busy clears
      // for vector consumers waiting on this VL producer; other ports stay invalid.
      iss_unit.io.vl_wakeup_ports.foreach  { p => p.valid := false.B; p.bits := DontCare }
      iss_unit.io.vl_wakeup_ports(0).valid    := vset_wb.get.valid
      iss_unit.io.vl_wakeup_ports(0).bits.pvl := vset_wb.get.bits.pvl
    }
    // Step 11a.2 (sub-step B): UN-TIE the V-LOAD grant. Advertise FC_AGEN on issue
    // port 0 only when the whole vector-LS pipe is idle (VecLSRegRead.fu_ready is
    // registered -> no comb loop with the grant). vle uops carry fu_code(FC_AGEN).
    // vstore / valu stay tied off (no consumer yet).
    vload_iss_unit.get.io.fu_types(0)(FC_AGEN) := vec_ls_rr.get.io.fu_ready
  }

  // ----------------------------------------------------------------
  // Caracal (Step 8): tie off the vector + VL register files DORMANT. No
  // producers (Step 9 reg-write) or consumers (Step 11 reg-read) exist yet, so
  // every input must be driven (Chisel errors otherwise). Writes are held
  // invalid; read addresses are 0. read_ports.data outputs are left dangling
  // (no consumer yet -- legal in Chisel). All gated by usingRVV so vector-OFF
  // RTL is byte-identical (gate 9f).
  if (usingRVV) {
    vec_regfile.get.io.write_ports.foreach { w =>
      w.valid     := false.B
      w.bits.addr := 0.U
      w.bits.data := 0.U
      w.bits.mask := 0.U
    }
    vec_regfile.get.io.read_ports.foreach { r => r.addr := 0.U }

    // VL-RF write ports: tie off all, then un-stub W1 (the int-ALU vset writer). The int ALU
    // is the SOLE VL-RF writer for all three vset forms (vsetvl/vsetvli/vsetivli) -- this
    // avoids the vsetivli double-write that a separate VCFG W0 path would cause; W0/W2 stay
    // tied off. addr = pvl (renamed VL phys reg), data = computed vl_value.
    vl_regfile.get.io.write_ports.foreach { w =>
      w.valid     := false.B
      w.bits.addr := 0.U
      w.bits.data := 0.U
    }
    vl_regfile.get.io.write_ports(1).valid     := vset_wb.get.valid
    vl_regfile.get.io.write_ports(1).bits.addr := vset_wb.get.bits.pvl
    vl_regfile.get.io.write_ports(1).bits.data := vset_wb.get.bits.vl_value
    vl_regfile.get.io.read_ports.foreach { r => r.addr := 0.U }
  }

  //-------------------------------------------------------------
  // Caracal (Step 10): Vector LS AGEN, instantiated DORMANT. The cracking path
  // (VecLsDecode -> VecAgenStage1{load,store} + VecDgen) is wired so every input
  // is driven and the dangling vector-LS issue iss_uops finally have a consumer,
  // but the AGEN output nop.ready is held false so the FSMs never leave IDLE and
  // nothing is cracked. No vector register-read path exists yet (Step 11), so the
  // operand/vl/vstart feeds are 0 and VecDgen's VRF read is tied off. The unit-
  // stride fast-path Packer and the unified LSU consumer arrive in Step 11. All
  // gated by usingRVV so vector-OFF RTL stays byte-identical (gate 9f).
  if (usingRVV) {
    val vec_ls_decode  = Module(new boom.v4.vec.lsu.VecLsDecode)
    val vec_agen_load  = Module(new boom.v4.vec.lsu.VecAgenStage1(isStore = false))
    val vec_agen_store = Module(new boom.v4.vec.lsu.VecAgenStage1(isStore = true))
    val vec_dgen       = Module(new boom.v4.vec.lsu.VecDgen)
    val vec_agen_kill  = RegNext(rob.io.flush.valid)

    // Step 11a.2 (sub-step B): the V-LOAD grant flows through VecLSRegRead, which
    // reads rs1 (base) from a dedicated int-RF port + vl from the VL-RF, then
    // drives VecLsDecode. (Stores stay dormant: vstore never grants yet.)
    vec_ls_rr.get.io.iss             := vload_iss_unit.get.io.iss_uops(0)
    vec_ls_rr.get.io.vl_data         := vl_regfile.get.io.read_ports(0).data
    vec_ls_rr.get.io.agen_active     := vec_agen_load.io.gen_active
    vec_ls_rr.get.io.lsu_busy        := vec_lsu.get.io.busy
    vec_ls_rr.get.io.agen_start_fire := vec_agen_load.io.start.fire
    vec_ls_rr.get.io.kill            := vec_agen_kill
    vl_regfile.get.io.read_ports(0).addr := vec_ls_rr.get.io.vl_addr

    vec_ls_decode.io.in.valid    := vec_ls_rr.get.io.dec.valid
    vec_ls_decode.io.in.uop      := vec_ls_rr.get.io.dec.uop
    vec_ls_decode.io.in.rs1_data := vec_ls_rr.get.io.dec.rs1_data
    vec_ls_decode.io.in.rs2_data := 0.U          // unit-stride: rs2 unused (implied stride)
    vec_ls_decode.io.in.vl       := vec_ls_rr.get.io.dec.vl
    vec_ls_decode.io.in.vstart   := vec_ls_rr.get.io.dec.vstart

    // Load AGEN: start from decode (is_load). Step 11a.2: the cracked beat stream
    // now feeds VecLSU instead of being tied off.
    vec_agen_load.io.start.valid      := vec_ls_decode.io.out.valid && vec_ls_decode.io.out.is_load
    vec_agen_load.io.start.bits       := vec_ls_decode.io.out.dec_info
    vec_agen_load.io.kill             := vec_agen_kill
    vec_agen_load.io.mask_idx.valid   := false.B
    vec_agen_load.io.mask_idx.data    := 0.U
    vec_lsu.get.io.load_nop <> vec_agen_load.io.load_nop.get

    // Store AGEN: start from decode (!is_load); output ready held false (DORMANT).
    vec_agen_store.io.start.valid      := vec_ls_decode.io.out.valid && !vec_ls_decode.io.out.is_load
    vec_agen_store.io.start.bits       := vec_ls_decode.io.out.dec_info
    vec_agen_store.io.kill             := vec_agen_kill
    vec_agen_store.io.mask_idx.valid   := false.B
    vec_agen_store.io.mask_idx.data    := 0.U
    vec_agen_store.io.store_nop.get.ready := false.B

    // Store-data handshake: store AGEN (consumer) <> VecDgen (producer).
    vec_agen_store.io.vdb_data.get <> vec_dgen.io.vdb_data

    // VecDgen dormant inputs: no store ever starts; no VRF read path yet.
    vec_dgen.io.start.valid        := false.B
    vec_dgen.io.start.bits         := DontCare
    vec_dgen.io.kill               := vec_agen_kill
    vec_dgen.io.vrf_read.resp_data := 0.U
    vec_dgen.io.scalar_data        := 0.U

    // VecLSU <-> scalar LSU dedicated vector dcache port + kill.
    vec_lsu.get.io.dmem <> io.lsu.vec_dmem.get
    vec_lsu.get.io.kill := vec_agen_kill

    // VecLSU writes the destination VRF group, one 64b lane per beat. Drive the
    // VRF write port 0 (last-connect-wins over the Step-8 tie-off above).
    vec_regfile.get.io.write_ports(0).valid     := vec_lsu.get.io.vrf_write.valid
    vec_regfile.get.io.write_ports(0).bits.addr := vec_lsu.get.io.vrf_write.bits.addr
    vec_regfile.get.io.write_ports(0).bits.data := vec_lsu.get.io.vrf_write.bits.data
    vec_regfile.get.io.write_ports(0).bits.mask := vec_lsu.get.io.vrf_write.bits.mask
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Register Read Arbitrate Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  arb_idx = 0
  for ((unit, w) <- all_exe_units.zipWithIndex) {
    for (i <- 0 until unit.nReaders) {
      iregfile.io.arb_read_reqs(arb_idx) <> unit.io_arb_irf_reqs(i)
      arb_idx += 1
    }
    immregfile.io.arb_read_reqs(w) <> unit.io_arb_immrf_req
    unit.io_arb_rebusys := io.lsu.iwakeups
  }
  // Step 11a.2: the vector LS base-address read claims the dedicated last int-RF
  // logical read port (added to numIrfLogicalReadPorts under usingRVV).
  if (usingRVV) {
    iregfile.io.arb_read_reqs(arb_idx) <> vec_ls_rr.get.io.irf_req
    arb_idx += 1
  }
  require(arb_idx == numIrfLogicalReadPorts)
  for ((unit, w) <- (alu_exe_units).zipWithIndex) {
    pregfile.io.arb_read_reqs(w) <> unit.io_arb_prf_req
    bregfile.io.arb_read_reqs(w) <> unit.io_arb_brf_req
  }


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Register Read Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Register Read <- Issue (rrd <- iss)
  var rd_idx = 0
  for ((unit, w) <- all_exe_units.zipWithIndex) {
    for (i <- 0 until unit.nReaders) {
      unit.io_rrd_irf_resps(i) := iregfile.io.rrd_read_resps(rd_idx)
      rd_idx += 1
    }
    unit.io_rrd_immrf_resp := immregfile.io.rrd_read_resps(w)
    unit.io_rrd_irf_bypasses := int_bypasses
  }
  // Step 11a.2: vector LS base-address read response from the dedicated port.
  if (usingRVV) {
    vec_ls_rr.get.io.irf_resp := iregfile.io.rrd_read_resps(rd_idx)
    rd_idx += 1
  }
  require (rd_idx == numIrfLogicalReadPorts)
  for ((unit, w) <- alu_exe_units.zipWithIndex) {
    unit.io_rrd_prf_resp := pregfile.io.rrd_read_resps(w)
    unit.io_rrd_brf_resp := bregfile.io.rrd_read_resps(w)
  }

  //-------------------------------------------------------------
  // Privileged Co-processor 0 Register File
  // Note: Normally this would be bad in that I'm writing state before
  // committing, so to get this to work I stall the entire pipeline for
  // CSR instructions so I never speculate these instructions.

  io.lsu.sfence := unq_exe_unit.io_sfence.get
  io.ifu.sfence := unq_exe_unit.io_sfence.get

  // for critical path reasons, we aren't zero'ing this out if resp is not valid
  csr.io.rw.addr        := csr_resp.bits.addr
  csr.io.rw.cmd         := CSR.maskCmd(csr_resp.valid, csr_resp.bits.uop.csr_cmd)
  csr.io.rw.wdata       := csr_resp.bits.data

  rob.io.csr_replay.valid := csr_resp.valid && csr.io.rw_stall
  rob.io.csr_replay.bits.uop := csr_resp.bits.uop
  rob.io.csr_replay.bits.cause := MINI_EXCEPTION_CSR_REPLAY
  rob.io.csr_replay.bits.badvaddr := DontCare

  // Extra I/O
  // Delay retire/exception 1 cycle
  csr.io.retire    := RegNext(PopCount(rob.io.commit.arch_valids.asUInt))
  csr.io.exception := RegNext(rob.io.com_xcpt.valid)
  // csr.io.pc used for setting EPC during exception or CSR.io.trace.

  csr.io.pc        := (boom.v4.util.AlignPCToBoundary(io.ifu.com_pc, icBlockBytes)
                     + RegNext(rob.io.com_xcpt.bits.pc_lob)
                     - Mux(RegNext(rob.io.com_xcpt.bits.edge_inst), 2.U, 0.U))
  // Cause not valid for for CALL or BREAKPOINTs (CSRFile will override it).
  csr.io.cause     := RegNext(rob.io.com_xcpt.bits.cause)
  csr.io.ungated_clock := clock

  val tval_valid = csr.io.exception &&
    csr.io.cause.isOneOf(
      //Causes.illegal_instruction.U, we currently only write 0x0 for illegal instructions
      Causes.breakpoint.U,
      Causes.misaligned_load.U,
      Causes.misaligned_store.U,
      Causes.load_access.U,
      Causes.store_access.U,
      Causes.fetch_access.U,
      Causes.load_page_fault.U,
      Causes.store_page_fault.U,
      Causes.fetch_page_fault.U)

  csr.io.tval := Mux(tval_valid,
    RegNext(encodeVirtualAddress(rob.io.com_xcpt.bits.badvaddr, rob.io.com_xcpt.bits.badvaddr)), 0.U)

  // TODO move this function to some central location (since this is used elsewhere).
  def encodeVirtualAddress(a0: UInt, ea: UInt) =
    if (vaddrBitsExtended == vaddrBits) {
      ea
    } else {
      // Efficient means to compress 64-bit VA into vaddrBits+1 bits.
      // (VA is bad if VA(vaddrBits) != VA(vaddrBits-1)).
      val a = a0.asSInt >> vaddrBits
      val msb = Mux(a === 0.S || a === -1.S, ea(vaddrBits), !ea(vaddrBits-1))
      Cat(msb, ea(vaddrBits-1,0))
    }

  // reading requires serializing the entire pipeline
  csr.io.fcsr_flags.valid := rob.io.commit.fflags.valid
  csr.io.fcsr_flags.bits  := rob.io.commit.fflags.bits
  csr.io.set_fs_dirty.get := rob.io.commit.fflags.valid

  all_exe_units.map(i => i.io_fcsr_rm := csr.io.fcsr_rm)
  io.fcsr_rm := csr.io.fcsr_rm

  fp_pipeline.io.fcsr_rm := csr.io.fcsr_rm

  csr.io.hartid := io.hartid
  csr.io.interrupts := io.interrupts

  // we do not support the H-extension
  csr.io.htval := DontCare
  csr.io.gva := DontCare

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Execute Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Load/Store Unit ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // enqueue basic load/store info in Decode
  for (w <- 0 until coreWidth) {
    io.lsu.dis_uops(w).valid := dis_fire(w)
    io.lsu.dis_uops(w).bits  := dis_uops(w)
  }

  // tell LSU about committing loads and stores to clear entries
  io.lsu.commit                  := rob.io.commit

  // tell LSU that it should fire a load that waits for the rob to clear
  io.lsu.commit_load_at_rob_head := rob.io.com_load_is_at_rob_head

  //com_xcpt.valid comes too early, will fight against a branch that resolves same cycle as an exception
  io.lsu.exception := RegNext(rob.io.flush.valid)

  // Handle Branch Mispeculations
  io.lsu.brupdate := brupdate
  io.lsu.rob_head_idx := rob.io.rob_head_idx
  io.lsu.rob_pnr_idx  := rob.io.rob_pnr_idx

  io.lsu.tsc_reg := debug_tsc_reg

  // Connect IFPU
  fp_pipeline.io.from_int  <> unq_exe_unit.io_ifpu_resp.get

  // Connect FLDs
  fp_pipeline.io.ll_wports <> io.lsu.fresp

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Commit Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  var cnt = numIrfWritePorts
  for (wb <- fp_pipeline.io.wb) {
    rob.io.wb_resps(cnt) := wb
    rob.io.wb_resps(cnt).bits.data := ieee(wb.bits.data)
    cnt += 1
  }

  require (cnt == rob.numWakeupPorts)

  // branch resolution
  rob.io.brupdate <> brupdate

  io.lsu.status := csr.io.status
  io.lsu.bp     := csr.io.bp
  io.lsu.mcontext := csr.io.mcontext
  io.lsu.scontext := csr.io.scontext


  all_exe_units.map(u => u.io_status := csr.io.status)
  fp_pipeline.io.status := csr.io.status

  // LSU <> ROB
  rob.io.lsu_clr_bsy    := io.lsu.clr_bsy
  rob.io.lsu_clr_unsafe := io.lsu.clr_unsafe
  rob.io.lxcpt          <> io.lsu.lxcpt

  assert (!(csr.io.singleStep), "[core] single-step is unsupported.")


  //-------------------------------------------------------------
  // **** Flush Pipeline ****
  //-------------------------------------------------------------
  // flush on exceptions, miniexeptions, and after some special instructions

  fp_pipeline.io.flush_pipeline := RegNext(rob.io.flush.valid)

  for (eu <- all_exe_units)
    eu.io_kill := RegNext(rob.io.flush.valid)


  assert (!(rob.io.com_xcpt.valid && !rob.io.flush.valid),
    "[core] exception occurred, but pipeline flush signal not set!")

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Outputs to the External World ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // detect pipeline freezes and throw error
  val idle_cycles = freechips.rocketchip.util.WideCounter(32)
  when (rob.io.commit.valids.asUInt.orR ||
        csr.io.csr_stall ||
        io.rocc.busy ||
        reset.asBool) {
    idle_cycles := 0.U
  }
  assert (!(idle_cycles.value(PlusArg("boom_timeout", 13, width=5))), "Pipeline has hung.")

  fp_pipeline.io.debug_tsc_reg := debug_tsc_reg


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Handle Cycle-by-Cycle Printouts ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------


  if (COMMIT_LOG_PRINTF) {
    var new_commit_cnt = 0.U

    for (w <- 0 until coreWidth) {
      val priv = ShiftRegister(csr.io.status.prv, 2) // erets change the privilege. Get the old one

      // To allow for diffs against spike :/
      def printf_inst(uop: MicroOp) = {
        when (uop.is_rvc) {
          printf("(0x%x)", uop.debug_inst(15,0))
        } .otherwise {
          printf("(0x%x)", uop.debug_inst)
        }
      }

      when (rob.io.commit.arch_valids(w)) {
        printf("%d 0x%x ",
          priv,
          Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen))
        printf_inst(rob.io.commit.uops(w))
        when (rob.io.commit.uops(w).dst_rtype === RT_FIX && rob.io.commit.uops(w).ldst =/= 0.U) {
          printf(" x%d 0x%x\n",
            rob.io.commit.uops(w).ldst,
            rob.io.commit.debug_wdata(w))
        } .elsewhen (rob.io.commit.uops(w).dst_rtype === RT_FLT) {
          printf(" f%d 0x%x\n",
            rob.io.commit.uops(w).ldst,
            rob.io.commit.debug_wdata(w))
        } .otherwise {
          printf("\n")
        }
      }
    }
  } else if (BRANCH_PRINTF) {
    val debug_ghist = RegInit(0.U(globalHistoryLength.W))
    when (rob.io.flush.valid && FlushTypes.useCsrEvec(rob.io.flush.bits.flush_typ)) {
      debug_ghist := 0.U
    }

    var new_ghist = debug_ghist

    for (w <- 0 until coreWidth) {
      when (rob.io.commit.arch_valids(w) &&
        (rob.io.commit.uops(w).is_br || rob.io.commit.uops(w).is_jal || rob.io.commit.uops(w).is_jalr)) {
        // for (i <- 0 until globalHistoryLength) {
        //   printf("%x", new_ghist(globalHistoryLength-i-1))
        // }
        // printf("\n")
        printf("%x %x %x %x %x %x\n",
          rob.io.commit.uops(w).debug_fsrc, rob.io.commit.uops(w).taken,
          rob.io.commit.uops(w).is_br, rob.io.commit.uops(w).is_jal,
          rob.io.commit.uops(w).is_jalr, Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen))

      }
      new_ghist = Mux(rob.io.commit.arch_valids(w) && rob.io.commit.uops(w).is_br,
        Mux(rob.io.commit.uops(w).taken, new_ghist << 1 | 1.U(1.W), new_ghist << 1),
        new_ghist)
    }
    debug_ghist := new_ghist
  }

  // TODO: Does anyone want this debugging functionality?
  val coreMonitorBundle = Wire(new CoreMonitorBundle(xLen, fLen))
  coreMonitorBundle := DontCare
  coreMonitorBundle.clock  := clock
  coreMonitorBundle.reset  := reset


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Page Table Walker

  io.ptw.ptbr       := csr.io.ptbr
  io.ptw.status     := csr.io.status
  io.ptw.pmp        := csr.io.pmp
  io.ptw.sfence     := io.ifu.sfence

  //-------------------------------------------------------------
  //-------------------------------------------------------------

  io.rocc := DontCare
  io.rocc.exception := csr.io.exception && csr.io.status.xs.orR
  io.rocc.csrs <> csr.io.roccCSRs
  if (usingRoCC) {
    val rocc_unit = unq_exe_unit
    rocc_unit.io_rocc_core.get.rocc         <> io.rocc
    rocc_unit.io_rocc_core.get.dis_uops     := dis_uops
    rocc_unit.io_rocc_core.get.rob_head_idx := rob.io.rob_head_idx
    rocc_unit.io_rocc_core.get.rob_pnr_idx  := rob.io.rob_pnr_idx

    for (w <- 0 until coreWidth) {
      rocc_unit.io_rocc_core.get.dis_rocc_vals(w) := (
        dis_fire(w) &&
        dis_uops(w).is_rocc &&
        !dis_uops(w).exception
      )
    }
  }

  io.trace := DontCare
  io.trace.time := csr.io.time
  io.trace.insns map (t => t.valid := false.B)
  io.trace.custom.get.asInstanceOf[BoomTraceBundle].rob_empty := rob.io.empty

  if (trace) {
    for (w <- 0 until coreWidth) {
      // Delay the trace so we have a cycle to pull PCs out of the FTQ
      io.trace.insns(w).valid      := RegNext(rob.io.commit.arch_valids(w))

      // Recalculate the PC
      io.ifu.debug_ftq_idx(w) := rob.io.commit.uops(w).ftq_idx
      val iaddr = (AlignPCToBoundary(io.ifu.debug_fetch_pc(w), icBlockBytes)
                   + RegNext(rob.io.commit.uops(w).pc_lob)
                   - Mux(RegNext(rob.io.commit.uops(w).edge_inst), 2.U, 0.U))(vaddrBits-1,0)
      io.trace.insns(w).iaddr      := Sext(iaddr, xLen)

      def getInst(uop: MicroOp, inst: UInt): UInt = {
        Mux(uop.is_rvc, Cat(0.U(16.W), inst(15,0)), inst)
      }

      def getWdata(uop: MicroOp, wdata: UInt): UInt = {
        Mux((uop.dst_rtype === RT_FIX && uop.ldst =/= 0.U) || (uop.dst_rtype === RT_FLT), wdata, 0.U(xLen.W))
      }

      // use debug_insts instead of uop.debug_inst to use the rob's debug_inst_mem
      // note: rob.debug_insts comes 1 cycle later
      io.trace.insns(w).insn       := getInst(RegNext(rob.io.commit.uops(w)), rob.io.commit.debug_insts(w))
      io.trace.insns(w).wdata.map { _ := RegNext(getWdata(rob.io.commit.uops(w), rob.io.commit.debug_wdata(w))) }

      // Comment out this assert because it blows up FPGA synth-asserts
      // This tests correctedness of the debug_inst mem
      // when (RegNext(rob.io.commit.valids(w))) {
      //   assert(rob.io.commit.debug_insts(w) === RegNext(rob.io.commit.uops(w).debug_inst))
      // }
      // This tests correctedness of recovering pcs through ftq debug ports
      // when (RegNext(rob.io.commit.valids(w))) {
      //   assert(Sext(io.trace.insns(w).iaddr, xLen) ===
      //     RegNext(Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen)))
      // }

      // These csr signals do not exactly match up with the ROB commit signals.
      io.trace.insns(w).priv       := RegNext(Cat(RegNext(RegNext(csr.io.status.debug)), csr.io.status.prv))
      // Can determine if it is an interrupt or not based on the MSB of the cause
      io.trace.insns(w).exception  := RegNext(rob.io.com_xcpt.valid && !rob.io.com_xcpt.bits.cause(xLen - 1)) && (w == 0).B
      io.trace.insns(w).interrupt  := RegNext(rob.io.com_xcpt.valid && rob.io.com_xcpt.bits.cause(xLen - 1)) && (w == 0).B
      io.trace.insns(w).cause      := RegNext(rob.io.com_xcpt.bits.cause)
      io.trace.insns(w).tval       := RegNext(csr.io.tval)
    }
    dontTouch(io.trace)
  } else {
    io.ifu.debug_ftq_idx := DontCare
  }

  if (boomParams.enableTraceCoreIngress) {
    for (w <- 0 until coreWidth) {
        val trace_ingress = Module(new TraceCoreIngress(traceIngressParams))
        trace_ingress.io.in.valid := RegNext(rob.io.commit.arch_valids(w))
        // this is predicted to be taken, but since it is commited, it must have been predicted correctly
        trace_ingress.io.in.taken := RegNext(rob.io.commit.uops(w).taken) 
        trace_ingress.io.in.is_branch := RegNext(rob.io.commit.uops(w).is_br)
        trace_ingress.io.in.is_jal := RegNext(rob.io.commit.uops(w).is_jal)
        trace_ingress.io.in.is_jalr := RegNext(rob.io.commit.uops(w).is_jalr)
        trace_ingress.io.in.insn := RegNext(rob.io.commit.uops(w).debug_inst)
        trace_ingress.io.in.pc := RegNext(rob.io.commit.uops(w).debug_pc)
        trace_ingress.io.in.is_compressed := RegNext(rob.io.commit.uops(w).is_rvc)
        trace_ingress.io.in.interrupt := RegNext(rob.io.com_xcpt.valid && rob.io.com_xcpt.bits.cause(xLen - 1)) && (w == 0).B
        trace_ingress.io.in.exception := RegNext(rob.io.com_xcpt.valid && !rob.io.com_xcpt.bits.cause(xLen - 1)) && (w == 0).B
        trace_ingress.io.in.trap_return := RegNext(rob.io.commit.uops(w).is_eret)
        io.trace_core_ingress.get.group(w) <> trace_ingress.io.out
    }
    io.trace_core_ingress.get.ctx := RegNext(csr.io.ptbr.asid)
    io.trace_core_ingress.get.tval := RegNext(csr.io.tval)
    io.trace_core_ingress.get.cause := RegNext(csr.io.cause)
    io.trace_core_ingress.get.time := RegNext(csr.io.time)
    io.trace_core_ingress.get.priv := RegNext(csr.io.status.prv)
  }


  //-------------------------------------------------------------
  // Caracal (Step 11a.2): commit-time vector readback for the cosim arch checker.
  // The cosim reads a committed vector op's result from commit.uops[w].
  // debug_vec_{wdata,wmask} (core_harness.v), NOT from the VRF. Populate it by
  // reading vec_regfile[pvdest_grp] for each committing vector dest op via the
  // combinational debug read ports. wmask = pvdest_grp_mask (valid members).
  //-------------------------------------------------------------
  val dbg_vec_wdata = Wire(Vec(coreWidth, UInt(((vLen * 8).max(1)).W)))
  val dbg_vec_wmask = Wire(Vec(coreWidth, UInt(8.W)))
  dbg_vec_wdata.foreach(_ := 0.U)
  dbg_vec_wmask.foreach(_ := 0.U)
  if (usingRVV) {
    val MM = boom.v4.vec.rename.VecEmul.MAX_MEMBERS
    for (w <- 0 until coreWidth) {
      val rmp = rob.io.commit.vec_remap.get(w)
      val dea = rob.io.commit.vec_dealloc.get(w)
      for (m <- 0 until MM) {
        vec_regfile.get.io.debug_read_ports(w * MM + m).addr := rmp.pdst(m)
      }
      // member m occupies debug_vec_wdata[vLen*m +: vLen]; Cat puts the first
      // element at the MSB, so reverse to land member 0 in the low bits.
      dbg_vec_wdata(w) := Cat((0 until MM).reverse.map(m =>
        vec_regfile.get.io.debug_read_ports(w * MM + m).data))
      dbg_vec_wmask(w) := Mux(rmp.valid, dea.mask, 0.U)
    }
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Connect debugging harness for DV COSIM bridge ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  if (DEBUG_HARNESS) {
     if (coreParams.retireWidth == 1) {
       val harness_1 = Module(new BoomCoreHarnessWrapper_1(coreParams.vLen.max(64)))  // safe min for no-VPU configs
                     
       harness_1.io.clock        := clock.asBool
       harness_1.io.reset        := reset.asBool
       harness_1.io.hartid       := io.hartid

       harness_1.io.csrwr.cmd   := csr.io.rw.cmd
       harness_1.io.csrwr.addr  := csr.io.rw.addr
       harness_1.io.csrwr.wdata := csr.io.rw.wdata    
       harness_1.io.csrwr.rdata := csr.io.rw.rdata    

       for (w <- 0 until 1) {
          harness_1.io.commit.arch_valids(w)      := rob.io.commit.arch_valids(w)
          harness_1.io.commit.uops(w).debug_pc    := rob.io.commit.uops(w).debug_pc(vaddrBits-1,0)
          harness_1.io.commit.uops(w).debug_tag   := 0.U  // v4 MicroOp lacks debug_tag (dispatch-assigned MCM ID)
          harness_1.io.commit.uops(w).debug_inst  := rob.io.commit.uops(w).debug_inst
          harness_1.io.commit.uops(w).dst_rtype   := rob.io.commit.uops(w).dst_rtype
          harness_1.io.commit.uops(w).ldst        := rob.io.commit.uops(w).ldst
          harness_1.io.commit.uops(w).debug_wdata := rob.io.commit.debug_wdata(w)
          harness_1.io.commit.uops(w).debug_vec_wdata := 0.U  // no VPU yet
          harness_1.io.commit.uops(w).debug_vec_wmask := 0.U
       }
     } else if (coreParams.retireWidth == 2) {
       val harness_2 = Module(new BoomCoreHarnessWrapper_2(coreParams.vLen.max(64)))  // safe min for no-VPU configs
                     
       harness_2.io.clock        := clock.asBool
       harness_2.io.reset        := reset.asBool
       harness_2.io.hartid       := io.hartid
       
       harness_2.io.csrwr.cmd   := csr.io.rw.cmd
       harness_2.io.csrwr.addr  := csr.io.rw.addr
       harness_2.io.csrwr.wdata := csr.io.rw.wdata    
       harness_2.io.csrwr.rdata := csr.io.rw.rdata    

       for (w <- 0 until 2) {
          harness_2.io.commit.arch_valids(w)      := rob.io.commit.arch_valids(w)
          harness_2.io.commit.uops(w).debug_pc    := rob.io.commit.uops(w).debug_pc(vaddrBits-1,0)
          harness_2.io.commit.uops(w).debug_tag   := 0.U  // v4 MicroOp lacks debug_tag (dispatch-assigned MCM ID)
          harness_2.io.commit.uops(w).debug_inst  := rob.io.commit.uops(w).debug_inst
          harness_2.io.commit.uops(w).dst_rtype   := rob.io.commit.uops(w).dst_rtype
          harness_2.io.commit.uops(w).ldst        := rob.io.commit.uops(w).ldst
          harness_2.io.commit.uops(w).debug_wdata := rob.io.commit.debug_wdata(w)
          if (usingRVV) {  // Step 11a.2: committed vector result, read back from the VRF
            harness_2.io.commit.uops(w).debug_vec_wdata := dbg_vec_wdata(w)
            harness_2.io.commit.uops(w).debug_vec_wmask := dbg_vec_wmask(w)
          } else {
            harness_2.io.commit.uops(w).debug_vec_wdata := 0.U  // no VPU
            harness_2.io.commit.uops(w).debug_vec_wmask := 0.U
          }
       }
     } else if (coreParams.retireWidth == 3) {
       val harness_3 = Module(new BoomCoreHarnessWrapper_3(coreParams.vLen.max(64)))  // safe min for no-VPU configs
                     
       harness_3.io.clock        := clock.asBool
       harness_3.io.reset        := reset.asBool
       harness_3.io.hartid       := io.hartid
       
       harness_3.io.csrwr.cmd   := csr.io.rw.cmd
       harness_3.io.csrwr.addr  := csr.io.rw.addr
       harness_3.io.csrwr.wdata := csr.io.rw.wdata    
       harness_3.io.csrwr.rdata := csr.io.rw.rdata    

       for (w <- 0 until 3) {
          harness_3.io.commit.arch_valids(w)      := rob.io.commit.arch_valids(w)
          harness_3.io.commit.uops(w).debug_pc    := rob.io.commit.uops(w).debug_pc(vaddrBits-1,0)
          harness_3.io.commit.uops(w).debug_tag   := 0.U  // v4 MicroOp lacks debug_tag (dispatch-assigned MCM ID)
          harness_3.io.commit.uops(w).debug_inst  := rob.io.commit.uops(w).debug_inst
          harness_3.io.commit.uops(w).dst_rtype   := rob.io.commit.uops(w).dst_rtype
          harness_3.io.commit.uops(w).ldst        := rob.io.commit.uops(w).ldst
          harness_3.io.commit.uops(w).debug_wdata := rob.io.commit.debug_wdata(w)
          harness_3.io.commit.uops(w).debug_vec_wdata := 0.U  // no VPU yet
          harness_3.io.commit.uops(w).debug_vec_wmask := 0.U
       }
     } else if (coreParams.retireWidth == 4) {
       val harness_4 = Module(new BoomCoreHarnessWrapper_4(coreParams.vLen.max(64)))  // safe min for no-VPU configs
                     
       harness_4.io.clock        := clock.asBool
       harness_4.io.reset        := reset.asBool
       harness_4.io.hartid       := io.hartid
       
       harness_4.io.csrwr.cmd   := csr.io.rw.cmd
       harness_4.io.csrwr.addr  := csr.io.rw.addr
       harness_4.io.csrwr.wdata := csr.io.rw.wdata    
       harness_4.io.csrwr.rdata := csr.io.rw.rdata    

       for (w <- 0 until 4) {
          harness_4.io.commit.arch_valids(w)      := rob.io.commit.arch_valids(w)
          harness_4.io.commit.uops(w).debug_pc    := rob.io.commit.uops(w).debug_pc(vaddrBits-1,0)
          harness_4.io.commit.uops(w).debug_tag   := 0.U  // v4 MicroOp lacks debug_tag (dispatch-assigned MCM ID)
          harness_4.io.commit.uops(w).debug_inst  := rob.io.commit.uops(w).debug_inst
          harness_4.io.commit.uops(w).dst_rtype   := rob.io.commit.uops(w).dst_rtype
          harness_4.io.commit.uops(w).ldst        := rob.io.commit.uops(w).ldst
          harness_4.io.commit.uops(w).debug_wdata := rob.io.commit.debug_wdata(w)
          harness_4.io.commit.uops(w).debug_vec_wdata := 0.U  // no VPU yet
          harness_4.io.commit.uops(w).debug_vec_wmask := 0.U
       }
     } else if (coreParams.retireWidth == 6) {
       val harness_6 = Module(new BoomCoreHarnessWrapper_6(coreParams.vLen.max(64)))  // safe min for no-VPU configs
                     
       harness_6.io.clock        := clock.asBool
       harness_6.io.reset        := reset.asBool
       harness_6.io.hartid       := io.hartid
       
       harness_6.io.csrwr.cmd   := csr.io.rw.cmd
       harness_6.io.csrwr.addr  := csr.io.rw.addr
       harness_6.io.csrwr.wdata := csr.io.rw.wdata    
       harness_6.io.csrwr.rdata := csr.io.rw.rdata    

       for (w <- 0 until 6) {
          harness_6.io.commit.arch_valids(w)      := rob.io.commit.arch_valids(w)
          harness_6.io.commit.uops(w).debug_pc    := rob.io.commit.uops(w).debug_pc(vaddrBits-1,0)
          harness_6.io.commit.uops(w).debug_tag   := 0.U  // v4 MicroOp lacks debug_tag (dispatch-assigned MCM ID)
          harness_6.io.commit.uops(w).debug_inst  := rob.io.commit.uops(w).debug_inst
          harness_6.io.commit.uops(w).dst_rtype   := rob.io.commit.uops(w).dst_rtype
          harness_6.io.commit.uops(w).ldst        := rob.io.commit.uops(w).ldst
          harness_6.io.commit.uops(w).debug_wdata := rob.io.commit.debug_wdata(w)
          harness_6.io.commit.uops(w).debug_vec_wdata := 0.U  // no VPU yet
          harness_6.io.commit.uops(w).debug_vec_wmask := 0.U
       }
     } else if (coreParams.retireWidth == 8) {
       val harness_8 = Module(new BoomCoreHarnessWrapper_8(coreParams.vLen.max(64)))  // safe min for no-VPU configs
                     
       harness_8.io.clock        := clock.asBool
       harness_8.io.reset        := reset.asBool
       harness_8.io.hartid       := io.hartid
       
       harness_8.io.csrwr.cmd   := csr.io.rw.cmd
       harness_8.io.csrwr.addr  := csr.io.rw.addr
       harness_8.io.csrwr.wdata := csr.io.rw.wdata    
       harness_8.io.csrwr.rdata := csr.io.rw.rdata    

       for (w <- 0 until 8) {
          harness_8.io.commit.arch_valids(w)      := rob.io.commit.arch_valids(w)
          harness_8.io.commit.uops(w).debug_pc    := rob.io.commit.uops(w).debug_pc(vaddrBits-1,0)
          harness_8.io.commit.uops(w).debug_tag   := 0.U  // v4 MicroOp lacks debug_tag (dispatch-assigned MCM ID)
          harness_8.io.commit.uops(w).debug_inst  := rob.io.commit.uops(w).debug_inst
          harness_8.io.commit.uops(w).dst_rtype   := rob.io.commit.uops(w).dst_rtype
          harness_8.io.commit.uops(w).ldst        := rob.io.commit.uops(w).ldst
          harness_8.io.commit.uops(w).debug_wdata := rob.io.commit.debug_wdata(w)
          harness_8.io.commit.uops(w).debug_vec_wdata := 0.U  // no VPU yet
          harness_8.io.commit.uops(w).debug_vec_wmask := 0.U
       }
     }
  }

}




//-------------------------------------------------------------
// Below these classes instatiate black box wrappers for the COSIM harness
//-------------------------------------------------------------

// Snapshot of the CSR write port sampled by the cosim harness for differential checking.
class CSRWrite(val xLen: Int) extends Bundle
{
  val cmd   = UInt(3.W) // 0:Nop, 2:Read, 4:SystemInsn, 5:Write, 6:Set, 7:Clear
  val addr  = UInt(12.W)
  val wdata = Bits(xLen.W)
  val rdata = Bits(xLen.W)
}

class BoomCoreHarnessWrapper_1(val vlen: Int) extends BlackBox(Map("VLEN" -> IntParam(vlen)))
with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Bool())
    val reset = Input(Bool())
    val hartid = Input(UInt(8.W))
    val commit = Input(new DebugCommitSignals(40, 1, 64, vlen, 5, 1))
    val csrwr = Input(new CSRWrite(64))
  })
  addResource("/vsrc/core_harness_interface.v")
  addResource("/vsrc/core_harness.v")
  addResource("/vsrc/core_harness_wrapper_1.v")
}

class BoomCoreHarnessWrapper_2(val vlen: Int) extends BlackBox(Map("VLEN" -> IntParam(vlen)))
with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Bool())
    val reset = Input(Bool())
    val hartid = Input(UInt(8.W))
    val commit = Input(new DebugCommitSignals(40, 2, 64, vlen, 5, 1))
    val csrwr = Input(new CSRWrite(64))
  })
  addResource("/vsrc/core_harness_interface.v")
  addResource("/vsrc/core_harness.v")
  addResource("/vsrc/core_harness_wrapper_2.v")
}

class BoomCoreHarnessWrapper_3(val vlen: Int) extends BlackBox(Map("VLEN" -> IntParam(vlen)))
with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Bool())
    val reset = Input(Bool())
    val hartid = Input(UInt(8.W))
    val commit = Input(new DebugCommitSignals(40, 3, 64, vlen, 5, 1))
    val csrwr = Input(new CSRWrite(64))
  })
  addResource("/vsrc/core_harness_interface.v")
  addResource("/vsrc/core_harness.v")
  addResource("/vsrc/core_harness_wrapper_3.v")
}

class BoomCoreHarnessWrapper_4(val vlen: Int) extends BlackBox(Map("VLEN" -> IntParam(vlen)))
with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Bool())
    val reset = Input(Bool())
    val hartid = Input(UInt(8.W))
    val commit = Input(new DebugCommitSignals(40, 4, 64, vlen, 5, 2))
    val csrwr = Input(new CSRWrite(64))
  })
  addResource("/vsrc/core_harness_interface.v")
  addResource("/vsrc/core_harness.v")
  addResource("/vsrc/core_harness_wrapper_4.v")
}

class BoomCoreHarnessWrapper_6(val vlen: Int) extends BlackBox(Map("VLEN" -> IntParam(vlen)))
with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Bool())
    val reset = Input(Bool())
    val hartid = Input(UInt(8.W))
    val commit = Input(new DebugCommitSignals(40, 6, 64, vlen, 5, 2))
    val csrwr = Input(new CSRWrite(64))
  })
  addResource("/vsrc/core_harness_interface.v")
  addResource("/vsrc/core_harness.v")
  addResource("/vsrc/core_harness_wrapper_6.v")
}

class BoomCoreHarnessWrapper_8(val vlen: Int) extends BlackBox(Map("VLEN" -> IntParam(vlen)))
with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Bool())
    val reset = Input(Bool())
    val hartid = Input(UInt(8.W))
    val commit = Input(new DebugCommitSignals(40, 8, 64, vlen, 5, 2))
    val csrwr = Input(new CSRWrite(64))
  })
  addResource("/vsrc/core_harness_interface.v")
  addResource("/vsrc/core_harness.v")
  addResource("/vsrc/core_harness_wrapper_8.v")
}

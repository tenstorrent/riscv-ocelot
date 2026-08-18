//******************************************************************************
// Copyright (c) 2013 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Re-order Buffer
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Bank the ROB, such that each "dispatch" group gets its own row of the ROB,
// and each instruction in the dispatch group goes to a different bank.
// We can compress out the PC by only saving the high-order bits!
//
// ASSUMPTIONS:
//    - dispatch groups are aligned to the PC.
//
// NOTES:
//    - Currently we do not compress out bubbles in the ROB.
//    - Exceptions are only taken when at the head of the commit bundle --
//      this helps deal with loads, stores, and refetch instructions.

package boom.v4.exu

import scala.math.ceil

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.util._
import freechips.rocketchip.rocket.VType

import boom.v4.common._
import boom.v4.util._
import boom.v4.vec.generated.{VecRobFlags, VecTrace}

/**
 * IO bundle to interact with the ROB
 *
 * @param numWakeupPorts number of wakeup ports to the rob
 * @param numFpuPorts number of fpu ports that will write back fflags
 */
class RobIo(
  val numWakeupPorts: Int
  )(implicit p: Parameters)  extends BoomBundle
{
  // Decode Stage
  // (Allocate, write instruction to ROB).
  val enq_valids       = Input(Vec(coreWidth, Bool()))
  val enq_uops         = Input(Vec(coreWidth, new MicroOp()))
  val enq_partial_stall= Input(Bool()) // we're dispatching only a partial packet,
                                       // and stalling on the rest of it (don't
                                       // advance the tail ptr)

  val xcpt_fetch_pc = Input(UInt(vaddrBitsExtended.W))

  val rob_tail_idx = Output(UInt(robAddrSz.W))
  val rob_pnr_idx  = Output(UInt(robAddrSz.W))
  val rob_head_idx = Output(UInt(robAddrSz.W))

  // Handle Branch Misspeculations
  val brupdate = Input(new BrUpdateInfo())

  // Write-back Stage
  // (Update of ROB)
  // Instruction is no longer busy and can be committed
  val wb_resps = Flipped(Vec(numWakeupPorts, Valid(new ExeUnitResp(xLen max fLen+1))))

  // Unbusying ports for stores.
  val lsu_clr_bsy      = Input(Vec(coreWidth, Valid(UInt(robAddrSz.W))))

  // Port for unmarking loads/stores as speculation hazards..
  val lsu_clr_unsafe   = Input(Vec(lsuWidth, Valid(UInt(robAddrSz.W))))

  // Vector completion: single-shot busy-clear, one lane per producer (LCB
  // group-done, VecCiiComplete, VecGroupCopy). usingRVV-gated: with vectors
  // disabled RT_VEC entries do not exist, so this port is ABSENT, not tied
  // off, and the elaborated ROB reverts to the pre-Caracal one.
  val vec_clr_bsy      = if (usingRVV) Some(Input(Vec(vectorParams.numVecClrPorts, Valid(UInt(robAddrSz.W))))) else None

  // Group-safe: all of this instruction's element addresses have been
  // LCAM-checked. Shaped like lsu_clr_unsafe above, but a single port --
  // group-safe is per instruction, not per D$ lane, and the LSU is its
  // only producer.
  val vec_clr_unsafe   = if (usingRVV) Some(Input(Valid(UInt(robAddrSz.W)))) else None

  // CSR side effects of vector arithmetic (fflags/vxsat), applied at
  // commit, lane-for-lane with vec_clr_bsy above.
  val vec_rob_flags    = if (usingRVV) Some(Input(Vec(vectorParams.numVecClrPorts, Valid(new VecRobFlags)))) else None

  // Asserted for one cycle when a committing entry had vxsat set; wired to
  // csr.io.vector.set_vxsat in core.scala, exactly as io.commit.fflags is
  // wired to csr.io.fcsr_flags.
  val com_vxsat        = if (usingRVV) Some(Output(Bool())) else None

  val lxcpt = Flipped(new ValidIO(new Exception())) // LSU
  val csr_replay = Input(Valid(new Exception()))

  // Commit stage (free resources).
  val commit = Output(new CommitSignals())
  val rollback   = Bool()

  // tell the LSU that the head of the ROB is a load
  // (some loads can only execute once they are at the head of the ROB).
  val com_load_is_at_rob_head = Output(Bool())

  // Communicate exceptions to the CSRFile
  val com_xcpt = Valid(new CommitExceptionSignals())

  // Let the CSRFile stall us (e.g., wfi).
  val csr_stall = Input(Bool())

  // Let the TraceEncoder stall us
  val trace_stall = Input(Bool())

  // Flush signals (including exceptions, pipeline replays, and memory ordering failures)
  // to send to the frontend for redirection.
  val flush = Valid(new CommitExceptionSignals)

  // Stall Decode as appropriate
  val empty = Output(Bool())
  val ready = Output(Bool()) // ROB is busy unrolling rename state...

  // Stall the frontend if we know we will redirect the PC
  val flush_frontend = Output(Bool())


  val debug_tsc = Input(UInt(xLen.W))
}

/**
 * Bundle to send commit signals across processor
 */
class CommitSignals(implicit p: Parameters) extends BoomBundle
{
  val valids      = Vec(retireWidth, Bool()) // These instructions may not correspond to an architecturally executed insn
  val arch_valids = Vec(retireWidth, Bool())
  val uops        = Vec(retireWidth, new MicroOp())
  val fflags      = Valid(UInt(5.W))

  // These come a cycle later
  val debug_insts = Vec(retireWidth, UInt(32.W))

  val debug_wdata = Vec(retireWidth, UInt(xLen.W))
}

/**
 * Commit signals for the whisper-cosim Debug Harness.
 * Mirrors the SV interface in `vsrc/core_harness_wrapper_N.v`. Vector fields exist on
 * DebugMicroOp so the SV bit widths stay stable; core.scala ties them to zero until v4 has a VPU.
 */
class DebugCommitSignals(val coreMaxAddrBits: Int, val retireWidth: Int, val xLen: Int, val vLen: Int, val lregSz: Int, val memWidth: Int) extends Bundle
{
  val arch_valids = Vec(retireWidth, Bool())
  val uops        = Vec(retireWidth, new boom.v4.common.DebugMicroOp(coreMaxAddrBits, xLen, vLen, lregSz))
}

/**
 * Bundle to communicate exceptions to CSRFile
 *
 * TODO combine FlushSignals and ExceptionSignals (currently timed to different cycles).
 */
class CommitExceptionSignals(implicit p: Parameters) extends BoomBundle
{
  val ftq_idx    = UInt(log2Ceil(ftqSz).W)
  val edge_inst  = Bool()
  val is_rvc     = Bool()
  val pc_lob     = UInt(log2Ceil(icBlockBytes).W)
  val cause      = UInt(xLen.W)
  val badvaddr   = UInt(xLen.W)
// The ROB needs to tell the FTQ if there's a pipeline flush (and what type)
// so the FTQ can drive the frontend with the correct redirected PC.
  val flush_typ  = FlushTypes()
}

/**
 * Tell the frontend the type of flush so it can set up the next PC properly.
 */
object FlushTypes
{
  def SZ = 3
  def apply() = UInt(SZ.W)
  def none = 0.U
  def xcpt = 1.U // An exception occurred.
  def eret = (2+1).U // Execute an environment return instruction.
  def refetch = 2.U // Flush and refetch the head instruction.
  def next = 4.U // Flush and fetch the next instruction.

  def useCsrEvec(typ: UInt): Bool = typ(0) // typ === xcpt.U || typ === eret.U
  def useSamePC(typ: UInt): Bool = typ === refetch
  def usePCplus4(typ: UInt): Bool = typ === next

  def getType(valid: Bool, i_xcpt: Bool, i_eret: Bool, i_refetch: Bool): UInt = {
    val ret =
      Mux(!valid, none,
      Mux(i_eret, eret,
      Mux(i_xcpt, xcpt,
      Mux(i_refetch, refetch,
        next))))
    ret
  }
}

/**
 * Bundle of signals indicating that an exception occurred
 */
class Exception(implicit p: Parameters) extends BoomBundle
{
  val uop = new MicroOp()
  val cause = Bits(log2Ceil(freechips.rocketchip.rocket.Causes.all.max+2).W)
  val badvaddr = UInt(coreMaxAddrBits.W)
}

/**
 * Bundle for debug ROB signals
 * These should not be synthesized!
 */
class DebugRobSignals(implicit p: Parameters) extends BoomBundle
{
  val state = UInt()
  val rob_head = UInt(robAddrSz.W)
  val rob_pnr = UInt(robAddrSz.W)
  val xcpt_val = Bool()
  val xcpt_uop = new MicroOp()
  val xcpt_badvaddr = UInt(xLen.W)
}

/**
 * Reorder Buffer to keep track of dependencies and inflight instructions
 *
 * @param numWakeupPorts number of wakeup ports to the ROB
 * @param numFpuPorts number of FPU units that will write back fflags
 */
class Rob(
  val numWakeupPorts: Int,
  val usingTrace: Boolean
  )(implicit p: Parameters) extends BoomModule
{
  val io = IO(new RobIo(numWakeupPorts))
  // ROB Finite State Machine
  val s_reset :: s_normal :: s_wait_till_empty :: s_rollback :: Nil = Enum(4)
  val rob_state = RegInit(s_reset)

  //commit entries at the head, and unwind exceptions from the tail
  val rob_head     = RegInit(0.U(log2Ceil(numRobRows).W))
  val rob_head_lsb = RegInit(0.U((1 max log2Ceil(coreWidth)).W)) // TODO: Accurately track head LSB (currently always 0)
  val rob_head_idx = if (coreWidth == 1) rob_head else Cat(rob_head, rob_head_lsb)

  val rob_tail     = RegInit(0.U(log2Ceil(numRobRows).W))
  val rob_tail_lsb = RegInit(0.U((1 max log2Ceil(coreWidth)).W))
  val rob_tail_idx = if (coreWidth == 1) rob_tail else Cat(rob_tail, rob_tail_lsb)

  val rob_pnr      = RegInit(0.U(log2Ceil(numRobRows).W))
  val rob_pnr_lsb  = RegInit(0.U((1 max log2Ceil(coreWidth)).W))
  val rob_pnr_idx  = if (coreWidth == 1) rob_pnr  else Cat(rob_pnr , rob_pnr_lsb)

  val next_rob_head = WireInit(rob_head)
  rob_head := next_rob_head


  val full         = Wire(Bool())
  val empty        = Wire(Bool())

  val will_commit         = Wire(Vec(coreWidth, Bool()))
  val can_commit          = Wire(Vec(coreWidth, Bool()))
  val can_throw_exception = Wire(Vec(coreWidth, Bool()))

  val rob_pnr_unsafe      = Wire(Vec(coreWidth, Bool())) // are the instructions at the pnr unsafe?
  val rob_head_vals       = Wire(Vec(coreWidth, Bool())) // are the instructions at the head valid?
  val rob_tail_vals       = Wire(Vec(coreWidth, Bool())) // are the instructions at the tail valid? (to track partial row dispatches)
  val rob_head_uses_stq   = Wire(Vec(coreWidth, Bool()))
  val rob_head_uses_ldq   = Wire(Vec(coreWidth, Bool()))
  val rob_head_fflags     = Wire(Vec(coreWidth, Valid(UInt(freechips.rocketchip.tile.FPConstants.FLAGS_SZ.W))))
  // The head-indexed vxsat read, mirroring rob_head_fflags above. Declared
  // outside the per-bank loop (like rob_head_fflags) because rob_vxsat
  // itself is bank-local; usingRVV-gated because io.com_vxsat is.
  val rob_head_vxsat      = if (usingRVV) Some(Wire(Vec(coreWidth, Bool()))) else None

  val exception_thrown = Wire(Bool())

  // exception info
  // TODO compress xcpt cause size. Most bits in the middle are zero.
  val r_xcpt_val       = RegInit(false.B)
  val r_xcpt_uop       = Reg(new MicroOp())
  val r_xcpt_badvaddr  = Reg(UInt(coreMaxAddrBits.W))
  io.flush_frontend := r_xcpt_val

  //--------------------------------------------------
  // Utility

  def GetRowIdx(rob_idx: UInt): UInt = {
    if (coreWidth == 1) return rob_idx
    else return rob_idx >> log2Ceil(coreWidth).U
  }
  def GetBankIdx(rob_idx: UInt): UInt = {
    if(coreWidth == 1) { return 0.U }
    else           { return rob_idx(log2Ceil(coreWidth)-1, 0).asUInt }
  }

  // **************************************************************************
  // Debug

  class DebugRobBundle extends BoomBundle
  {
    val valid      = Bool()
    val busy       = Bool()
    val unsafe     = Bool()
    val uop        = new MicroOp()
    val exception  = Bool()
  }
  val debug_entry = Wire(Vec(numRobEntries, new DebugRobBundle))
  debug_entry := DontCare // override in statements below

  // **************************************************************************
  // --------------------------------------------------------------------------
  // **************************************************************************

  // Contains all information the PNR needs to find the oldest instruction which can't be safely speculated past.
  val rob_unsafe_masked = WireInit(VecInit(Seq.fill(numRobRows << log2Ceil(coreWidth)){false.B}))

  val rob_debug_inst_rdata = Wire(Vec(coreWidth, UInt(32.W)))
  val rob_debug_inst_wmask = WireInit(VecInit(0.U(coreWidth.W).asBools))
  val rob_debug_inst_wdata = Wire(Vec(coreWidth, UInt(32.W)))
  // Used for trace port, for debug purposes only
  if (usingTrace) {
    val rob_debug_inst_mem   = SyncReadMem(numRobRows, Vec(coreWidth, UInt(32.W)))
    rob_debug_inst_mem.write(rob_tail, rob_debug_inst_wdata, rob_debug_inst_wmask)
    rob_debug_inst_rdata := rob_debug_inst_mem.read(rob_head, will_commit.reduce(_||_))
  } else {
    rob_debug_inst_rdata := DontCare
  }

  // Branch resolution
  val brupdate_b2_rob_row = GetRowIdx(io.brupdate.b2.uop.rob_idx)
  val brupdate_b2_rob_row_oh = UIntToOH(brupdate_b2_rob_row)
  val brupdate_b2_rob_clr_oh = IsYoungerMask(brupdate_b2_rob_row, rob_head, numRobRows)
  val brupdate_b2_rob_bank_idx = GetBankIdx(io.brupdate.b2.uop.rob_idx)
  val brupdate_b2_rob_bank_clr_oh = ~MaskLower(UIntToOH(brupdate_b2_rob_bank_idx))

  class RobCompactUop extends Bundle {
    val is_fencei = Bool()
    val ftq_idx = UInt(log2Ceil(ftqSz).W)
    val uses_ldq = Bool()
    val uses_stq = Bool()
    //@req-spec-core.e23
    //@req-spec-rob.a2
    //@req-spec-rob.a3
    //@req-spec-rob.a4
    // Tracks MicroOp.dst_rtype, widened 2b -> 3b by the RT_VEC encoding in
    // ScalarOpConstants. Must stay in step with it: a 2b field here would
    // truncate RT_VEC (4) to RT_FIX (0) on the way through the ROB.
    val dst_rtype = UInt(3.W)
    val ldst = UInt(lregSz.W)
    val pdst = UInt(maxPregSz.W)
    val stale_pdst = UInt(maxPregSz.W)
  }
  //@req-spec-rob.a1
  //@req-spec-core.e22
  val compactUopWidth = 1 + log2Ceil(ftqSz) + 1 + 1 + 3 + lregSz + maxPregSz + maxPregSz
  def compact_to_uop(compact: RobCompactUop, uop: MicroOp): MicroOp = {
    val out = WireInit(uop)
    out.is_fencei := compact.is_fencei
    out.ftq_idx   := compact.ftq_idx
    out.uses_ldq  := compact.uses_ldq
    out.uses_stq  := compact.uses_stq
    out.dst_rtype := compact.dst_rtype
    out.ldst      := compact.ldst
    out.pdst      := compact.pdst
    out.stale_pdst := compact.stale_pdst
    out
  }
  def uop_to_compact(uop: MicroOp): RobCompactUop = {
    val out = Wire(new RobCompactUop)
    out.is_fencei := uop.is_fencei
    out.ftq_idx   := uop.ftq_idx
    out.uses_ldq  := uop.uses_ldq
    out.uses_stq  := uop.uses_stq
    out.dst_rtype := uop.dst_rtype
    out.ldst      := uop.ldst
    out.pdst      := uop.pdst
    out.stale_pdst := uop.stale_pdst
    out
  }
  // More efficient rob uop storage in 1R1W masked SRAM
  val rob_compact_uop_mem = SyncReadMem(numRobRows, Vec(coreWidth, UInt(compactUopWidth.W)))
  val rob_compact_uop_wdata = VecInit(io.enq_uops.map(u => uop_to_compact(u).asUInt))
  rob_compact_uop_mem.write(rob_tail, rob_compact_uop_wdata, io.enq_valids)
  val rob_compact_uop_rdata = rob_compact_uop_mem.read(next_rob_head)
  val rob_compact_uop_might_bypass = rob_head === RegNext(rob_tail)
  val rob_compact_uop_bypassed = (0 until coreWidth) map { w =>
    Mux(rob_head === RegNext(rob_tail) && RegNext(io.enq_valids(w)),
      RegNext(rob_compact_uop_wdata(w)),
      Mux(rob_head === ShiftRegister(rob_tail, 2) && ShiftRegister(io.enq_valids(w), 2),
        ShiftRegister(rob_compact_uop_wdata(w), 2),
        rob_compact_uop_rdata(w)
      )
    ).asTypeOf(new RobCompactUop)
  }

  val rob_fflags    = Seq.fill(coreWidth)(Reg(Vec(numRobRows, UInt(freechips.rocketchip.tile.FPConstants.FLAGS_SZ.W))))

  for (w <- 0 until coreWidth) {
    def MatchBank(bank_idx: UInt): Bool = (bank_idx === w.U)

    // one bank
    val rob_val       = RegInit(VecInit(Seq.fill(numRobRows){false.B}))
    val rob_bsy       = Reg(Vec(numRobRows, Bool()))
    val rob_unsafe    = Reg(Vec(numRobRows, Bool()))
    val rob_uop       = Reg(Vec(numRobRows, new MicroOp()))
    val rob_exception = Reg(Vec(numRobRows, Bool()))
    val rob_predicated = Reg(Vec(numRobRows, Bool())) // Was this instruction predicated out?
    val rob_fflags    = Reg(Vec(numRobRows, Valid(Bits(freechips.rocketchip.tile.FPConstants.FLAGS_SZ.W))))

    // usingRVV-gated per-bank vector state (Caracal D3 delta):
    //   rob_other_half -- 1-bit "other half pending" flag for a shared
    //     (segmented load/store) instruction; NOT a counter, see part 4.
    //   rob_vxsat      -- sticky fixed-point saturation bit, accrued from
    //     io.vec_rob_flags and applied at commit onto io.com_vxsat.
    //   rob_vconfig    -- the executed-VType latch for vset*, since a
    //     register-sourced vsetvl's vtype is not known until execute.
    //@req-spec-rob.d3
    //@req-spec-rob.d4
    //@req-spec-rob.d5
    //@req-spec-issue.c8
    val rob_other_half = if (usingRVV) Some(Reg(Vec(numRobRows, Bool()))) else None
    val rob_vxsat       = if (usingRVV) Some(Reg(Vec(numRobRows, Bool()))) else None
    val rob_vconfig     = if (usingRVV) Some(Reg(Vec(numRobRows, new VType))) else None

    val rob_debug_wdata = Mem(numRobRows, UInt(xLen.W))

    //-----------------------------------------------
    // Dispatch: Add Entry to ROB

    rob_debug_inst_wmask(w) := io.enq_valids(w)
    rob_debug_inst_wdata(w) := io.enq_uops(w).debug_inst

    when (io.enq_valids(w)) {
      rob_val(rob_tail)       := true.B
      rob_bsy(rob_tail)       := io.enq_uops(w).starts_bsy
      rob_unsafe(rob_tail)    := io.enq_uops(w).starts_unsafe
      rob_uop(rob_tail)       := io.enq_uops(w)
      rob_exception(rob_tail) := io.enq_uops(w).exception
      rob_predicated(rob_tail)   := false.B
      rob_fflags(rob_tail).valid := false.B
      rob_fflags(rob_tail).bits  := 0.U

      if (usingRVV) {
        //@req-spec-rob.d19
        //@req-spec-rob.d20
        rob_other_half.get(rob_tail) := io.enq_uops(w).is_shared.get
        rob_vxsat.get(rob_tail)      := false.B
        rob_vconfig.get(rob_tail)    := io.enq_uops(w).vconfig.get
      }

      assert (rob_val(rob_tail) === false.B, "[rob] overwriting a valid entry.")
      assert ((io.enq_uops(w).rob_idx >> log2Ceil(coreWidth)) === rob_tail)
    } .elsewhen (io.enq_valids.reduce(_|_) && !rob_val(rob_tail)) {

    }

    //-----------------------------------------------
    // Writeback

    for (i <- 0 until numWakeupPorts) {
      val wb_resp = io.wb_resps(i)
      val wb_uop = wb_resp.bits.uop
      val row_idx = GetRowIdx(wb_uop.rob_idx)
      when (wb_resp.valid && MatchBank(GetBankIdx(wb_uop.rob_idx))) {
        rob_bsy(row_idx)      := false.B
        rob_unsafe(row_idx)   := false.B
        rob_predicated(row_idx)  := wb_resp.bits.predicated
        when (wb_resp.bits.fflags.valid) {
          assert(!rob_fflags(row_idx).valid)

          rob_fflags(row_idx).valid := true.B
          rob_fflags(row_idx).bits  := wb_resp.bits.fflags.bits

        }
        // The one new state item part 9 needs: a register-sourced vsetvl's
        // vtype is not known at dispatch, so ALUUnit overwrites it on the
        // vset* writeback and the ROB must latch it here for commit.
        if (usingRVV) {
          when (wb_uop.is_vl_producer.get) {
            rob_vconfig.get(row_idx) := wb_resp.bits.uop.vconfig.get
          }
        }
      }
    }

    // Stores have a separate method to clear busy bits
    for (clr_rob_idx <- io.lsu_clr_bsy) {
      when (clr_rob_idx.valid && MatchBank(GetBankIdx(clr_rob_idx.bits))) {
        val cidx = GetRowIdx(clr_rob_idx.bits)
        //@req-spec-rob.d15
        //@req-spec-issue.c7
        //@req-spec-cii.i2
        if (usingRVV) {
          // A shared (segmented store) instruction's LSU half is only the
          // FIRST of its two completions: clear the "other half pending"
          // flag instead of rob_bsy, and let the second (coprocessor) half
          // clear rob_bsy. Reverts to the unconditional clear below when
          // usingRVV is false.
          when (!rob_other_half.get(cidx)) {
            rob_bsy(cidx) := false.B
          } .otherwise {
            rob_other_half.get(cidx) := false.B
          }
        } else {
          rob_bsy(cidx) := false.B
        }
        rob_unsafe(cidx) := false.B
        assert (rob_val(cidx) === true.B, "[rob] store writing back to invalid entry.")
        assert (rob_bsy(cidx) === true.B, "[rob] store writing back to a not-busy entry.")
      }
    }
    for (clr <- io.lsu_clr_unsafe) {
      when (clr.valid && MatchBank(GetBankIdx(clr.bits))) {
        val cidx = GetRowIdx(clr.bits)
        rob_unsafe(cidx) := false.B
      }
    }

    // Vector completion: single-shot busy-clear, one lane per producer
    // (LCB group-done, VecCiiComplete, VecGroupCopy). Shaped exactly like
    // the io.lsu_clr_bsy block above, iterated over the vector completion
    // lanes instead of coreWidth. An RT_VEC entry has no iresp writeback,
    // so this port is the ONLY thing that can clear its rob_bsy.
    if (usingRVV) {
      //@req-spec-rob.b1
      //@req-spec-rob.b2
      //@req-spec-rob.b3
      //@req-spec-rob.b4
      //@req-spec-rob.b5
      //@req-spec-rob.b6
      //@req-spec-rob.b7
      //@req-spec-rob.c2
      //@req-spec-rob.c3
      //@req-spec-rob.c4
      //@req-spec-rob.c8
      for (clr <- io.vec_clr_bsy.get) {
        when (clr.valid && MatchBank(GetBankIdx(clr.bits))) {
          val cidx = GetRowIdx(clr.bits)
          val cleared_bsy = !rob_other_half.get(cidx)
          when (cleared_bsy) {
            rob_bsy(cidx) := false.B
          } .otherwise {
            rob_other_half.get(cidx) := false.B
          }
          rob_unsafe(cidx) := false.B
          assert (rob_val(cidx) === true.B, "[rob] vec_clr_bsy naming an invalid entry.")
          assert (rob_bsy(cidx) === true.B, "[rob] vec_clr_bsy naming a not-busy entry.")
          VecTrace.trace("Rob", "vec_clr_bsy", rob_uop(cidx), Seq(("cleared_bsy", cleared_bsy)))
        }
      }

      // At most one completion event -- across all vec_clr_bsy lanes and
      // io.lsu_clr_bsy together -- may name a given entry in this bank in a
      // cycle. Two arrivals in the same cycle would both read the OLD
      // rob_other_half value and both take the flag-clear branch, so
      // rob_bsy would never clear and the entry would hang at the ROB head.
      //@req-spec-core.h7
      //@req-spec-rob.d7
      //@req-spec-rob.d8
      //@req-spec-rob.d14
      //@req-spec-rob.d21
      val vec_completion_valid = io.vec_clr_bsy.get.map(c => c.valid && MatchBank(GetBankIdx(c.bits))) ++
                                  io.lsu_clr_bsy.map(c => c.valid && MatchBank(GetBankIdx(c.bits)))
      val vec_completion_cidx  = io.vec_clr_bsy.get.map(c => GetRowIdx(c.bits)) ++
                                  io.lsu_clr_bsy.map(c => GetRowIdx(c.bits))
      for (i <- vec_completion_valid.indices; j <- (i + 1) until vec_completion_valid.length) {
        assert(!(vec_completion_valid(i) && vec_completion_valid(j) && vec_completion_cidx(i) === vec_completion_cidx(j)),
          "[rob] two completion events named the same entry in the same cycle.")
      }

      // Group-safe: all of this instruction's element addresses have been
      // LCAM-checked. Shaped like the io.lsu_clr_unsafe block above, but a
      // single port -- group-safe is per instruction, not per D$ lane, and
      // the LSU is its only producer. A shared entry's single rob_unsafe
      // bit needs only this (its LSU half); the coprocessor half performs
      // no memory access and carries no speculation hazard.
      //@req-spec-rob.e4
      //@req-spec-rob.e6
      //@req-spec-issue.d16
      //@req-spec-issue.d17
      //@req-spec-issue.d5
      when (io.vec_clr_unsafe.get.valid && MatchBank(GetBankIdx(io.vec_clr_unsafe.get.bits))) {
        val cidx = GetRowIdx(io.vec_clr_unsafe.get.bits)
        rob_unsafe(cidx) := false.B
        VecTrace.trace("Rob", "vec_clr_unsafe", rob_uop(cidx))
      }

      // CSR side effects of vector arithmetic (fflags/vxsat), accrued per
      // entry and applied at COMMIT, never at writeback -- a past-PNR CII
      // op can still be squashed by a ROB-head flush, and a sticky CSR bit
      // set by a squashed instruction could never be un-set.
      for (flags <- io.vec_rob_flags.get) {
        when (flags.valid && MatchBank(GetBankIdx(flags.bits.rob_idx))) {
          val row = GetRowIdx(flags.bits.rob_idx)
          // Mirrors the rob.scala fflags-writeback assert: a per-entry
          // fflags slot may be written at most once.
          assert(!rob_fflags(row).valid, "[rob] vec_rob_flags writing an already-valid fflags slot.")
          rob_fflags(row).valid := true.B
          rob_fflags(row).bits  := flags.bits.fflags
          rob_vxsat.get(row)    := flags.bits.vxsat
        }
      }
    }


    //-----------------------------------------------------
    // Exceptions
    // (the cause bits are compressed and stored elsewhere)

    when (io.lxcpt.valid && MatchBank(GetBankIdx(io.lxcpt.bits.uop.rob_idx))) {
      rob_exception(GetRowIdx(io.lxcpt.bits.uop.rob_idx)) := true.B
      when (io.lxcpt.bits.cause =/= MINI_EXCEPTION_MEM_ORDERING) {
        // In the case of a mem-ordering failure, the failing load will have been marked safe already.
        assert(rob_unsafe(GetRowIdx(io.lxcpt.bits.uop.rob_idx)),
          "An instruction marked as safe is causing an exception")
      }
    }
    when (io.csr_replay.valid && MatchBank(GetBankIdx(io.csr_replay.bits.uop.rob_idx))) {
      rob_exception(GetRowIdx(io.csr_replay.bits.uop.rob_idx)) := true.B
    }
    can_throw_exception(w) := rob_val(rob_head) && rob_exception(rob_head)

    //-----------------------------------------------
    // Commit

    // Can this instruction commit? (the check for exceptions/rob_state happens later).
    // Block commit if there is mispredict
    can_commit(w) := rob_val(rob_head) && !(rob_bsy(rob_head)) && !io.csr_stall && !io.brupdate.b2.mispredict && !io.trace_stall


    // use the same "com_uop" for both rollback AND commit
    // Perform Commit
    io.commit.valids(w)      := will_commit(w)
    io.commit.arch_valids(w) := will_commit(w) && !rob_predicated(rob_head)
    io.commit.uops(w)        := compact_to_uop(rob_compact_uop_bypassed(w), rob_uop(rob_head))
    io.commit.debug_insts(w) := rob_debug_inst_rdata(w)

    // A register-sourced vsetvl's vtype is not known until execute, so the
    // decode-time uop.vconfig snapshot in rob_uop cannot hold it; override
    // with the latch ALUUnit wrote on the vset* writeback (part 9). Same
    // override pattern the debug_fsrc/taken mispredict fix-up below uses.
    if (usingRVV) {
      //@req-spec-decode.h5
      io.commit.uops(w).vconfig.get := rob_vconfig.get(rob_head)
    }

    // We unbusy branches in b1, but its easier to mark the taken/provider src in b2,
    // when the branch might be committing
    when (io.brupdate.b2.mispredict &&
      MatchBank(GetBankIdx(io.brupdate.b2.uop.rob_idx)) &&
      GetRowIdx(io.brupdate.b2.uop.rob_idx) === rob_head) {
      io.commit.uops(w).debug_fsrc := BSRC_C
      io.commit.uops(w).taken      := io.brupdate.b2.taken
    }

    when (rob_state === s_rollback) {
      for (i <- 0 until numRobRows) {
        rob_val(i) := false.B
        rob_bsy(i) := false.B
      }
    }

    // -----------------------------------------------
    // Kill speculated entries on branch mispredict
    for (i <- 0 until numRobRows) {
      val br_mask = rob_uop(i).br_mask

      when (io.brupdate.b2.mispredict && (
        brupdate_b2_rob_clr_oh(i) ||
        (brupdate_b2_rob_row_oh(i) && brupdate_b2_rob_bank_clr_oh(w))
      )) {
        rob_val(i) := false.B
      }

      // //kill instruction if mispredict & br mask match
      // when (IsKilledByBranch(io.brupdate, false.B, br_mask))
      // {
      //   rob_val(i) := false.B
      // } .elsewhen (rob_val(i)) {
      //   // clear speculation bit even on correct speculation
      //   rob_uop(i).br_mask := GetNewBrMask(io.brupdate, br_mask)
      // }
    }


    // Debug signal to figure out which prediction structure
    // or core resolved a branch correctly
    when (io.brupdate.b2.mispredict &&
      MatchBank(GetBankIdx(io.brupdate.b2.uop.rob_idx))) {
      rob_uop(GetRowIdx(io.brupdate.b2.uop.rob_idx)).debug_fsrc := BSRC_C
      rob_uop(GetRowIdx(io.brupdate.b2.uop.rob_idx)).taken      := io.brupdate.b2.taken
    }

    // -----------------------------------------------
    // Commit
    when (will_commit(w)) {
      rob_val(rob_head) := false.B
    }

    // -----------------------------------------------
    // Outputs
    rob_head_vals(w)     := rob_val(rob_head)
    rob_tail_vals(w)     := rob_val(rob_tail)
    rob_head_fflags(w)   := rob_fflags(rob_head)
    rob_head_uses_stq(w) := io.commit.uops(w).uses_stq
    rob_head_uses_ldq(w) := io.commit.uops(w).uses_ldq
    if (usingRVV) {
      rob_head_vxsat.get(w) := rob_vxsat.get(rob_head)
    }

    //------------------------------------------------
    // Invalid entries are safe; thrown exceptions are unsafe.
    for (i <- 0 until numRobRows) {
      rob_unsafe_masked((i << log2Ceil(coreWidth)) + w) := rob_val(i) && (rob_unsafe(i) || rob_exception(i))
    }
    // Read unsafe status of PNR row.
    rob_pnr_unsafe(w) := rob_val(rob_pnr) && (rob_unsafe(rob_pnr) || rob_exception(rob_pnr))


    //--------------------------------------------------
    // Debug: for debug purposes, track side-effects to all register destinations

    for (i <- 0 until numWakeupPorts) {
      val rob_idx = io.wb_resps(i).bits.uop.rob_idx
      when (io.wb_resps(i).valid && MatchBank(GetBankIdx(rob_idx))) {
        rob_debug_wdata(GetRowIdx(rob_idx)) := io.wb_resps(i).bits.data
      }
      val temp_uop = rob_uop(GetRowIdx(rob_idx))

      assert (!(io.wb_resps(i).valid && MatchBank(GetBankIdx(rob_idx)) &&
               !rob_val(GetRowIdx(rob_idx))),
               "[rob] writeback (" + i + ") occurred to an invalid ROB entry.")
      assert (!(io.wb_resps(i).valid && MatchBank(GetBankIdx(rob_idx)) &&
               !rob_bsy(GetRowIdx(rob_idx))),
               "[rob] writeback (" + i + ") occurred to a not-busy ROB entry.")
      assert (!(io.wb_resps(i).valid && MatchBank(GetBankIdx(rob_idx)) &&
               temp_uop.dst_rtype =/= RT_X && temp_uop.pdst =/= io.wb_resps(i).bits.uop.pdst),
               "[rob] writeback (" + i + ") occurred to the wrong pdst.")
    }
    io.commit.debug_wdata(w) := rob_debug_wdata(rob_head)

    // Trace: distinguishes "the group never completed" / "one half never
    // reported" / "the entry never reached the head" -- the three ways
    // this delta can hang the machine. Emit-only; deleting this call site
    // leaves the design bit-identical.
    if (usingRVV) {
      when (io.commit.valids(w) && io.commit.uops(w).dst_rtype === RT_VEC) {
        VecTrace.trace("Rob", "commit_vec", io.commit.uops(w), Seq(
          ("v_emul",  io.commit.uops(w).v_emul.get),
          ("vconfig", rob_vconfig.get(rob_head).asUInt),
          ("vxsat",   rob_vxsat.get(rob_head))))
      }
    }

  } //for (w <- 0 until coreWidth)

  // **************************************************************************
  // --------------------------------------------------------------------------
  // **************************************************************************

  // -----------------------------------------------
  // Commit Logic
  // need to take a "can_commit" array, and let the first can_commits commit
  // previous instructions may block the commit of younger instructions in the commit bundle
  // e.g., exception, or (valid && busy).
  // Finally, don't throw an exception if there are instructions in front of
  // it that want to commit (only throw exception when head of the bundle).

  var block_commit = (rob_state =/= s_normal) && (rob_state =/= s_wait_till_empty) || RegNext(exception_thrown) || RegNext(RegNext(exception_thrown))
  var will_throw_exception = false.B
  var block_xcpt   = false.B

  for (w <- 0 until coreWidth) {
    will_throw_exception = (can_throw_exception(w) && !block_commit && !block_xcpt) || will_throw_exception

    will_commit(w)       := can_commit(w) && !can_throw_exception(w) && !block_commit
    block_commit         = (rob_head_vals(w) &&
                           (!can_commit(w) || can_throw_exception(w))) || block_commit
    block_xcpt           = will_commit(w)
  }

  // Note: exception must be in the commit bundle.
  // Note: exception must be the first valid instruction in the commit bundle.
  exception_thrown := will_throw_exception
  val is_mini_exception = io.com_xcpt.bits.cause.isOneOf(MINI_EXCEPTION_MEM_ORDERING, MINI_EXCEPTION_CSR_REPLAY)
  io.com_xcpt.valid := exception_thrown && !is_mini_exception
  io.com_xcpt.bits := DontCare
  io.com_xcpt.bits.cause := r_xcpt_uop.exc_cause

  io.com_xcpt.bits.badvaddr := Sext(r_xcpt_badvaddr, xLen)
  val insn_sys_pc2epc =
    rob_head_vals.reduce(_|_) && PriorityMux(rob_head_vals, io.commit.uops.map{u => u.is_sys_pc2epc})

  val refetch_inst = exception_thrown || insn_sys_pc2epc
  val com_xcpt_uop = PriorityMux(rob_head_vals, io.commit.uops)
  io.com_xcpt.bits.ftq_idx   := com_xcpt_uop.ftq_idx
  io.com_xcpt.bits.edge_inst := com_xcpt_uop.edge_inst
  io.com_xcpt.bits.is_rvc    := com_xcpt_uop.is_rvc
  io.com_xcpt.bits.pc_lob    := com_xcpt_uop.pc_lob

  val flush_commit_mask = Range(0,coreWidth).map{i => io.commit.valids(i) && io.commit.uops(i).flush_on_commit}
  val flush_commit = flush_commit_mask.reduce(_|_)
  val flush_val = exception_thrown || flush_commit

  assert(!(PopCount(flush_commit_mask) > 1.U),
    "[rob] Can't commit multiple flush_on_commit instructions on one cycle")

  val flush_uop = Mux(exception_thrown, com_xcpt_uop, Mux1H(flush_commit_mask, io.commit.uops))

  // delay a cycle for critical path considerations
  io.flush.valid          := flush_val
  io.flush.bits.badvaddr  := DontCare
  io.flush.bits.cause     := DontCare
  io.flush.bits.ftq_idx   := flush_uop.ftq_idx
  io.flush.bits.pc_lob    := flush_uop.pc_lob
  io.flush.bits.edge_inst := flush_uop.edge_inst
  io.flush.bits.is_rvc    := flush_uop.is_rvc
  io.flush.bits.flush_typ := FlushTypes.getType(flush_val,
                                                exception_thrown && !is_mini_exception,
                                                flush_commit && flush_uop.is_eret,
                                                refetch_inst)

  io.rollback := rob_state === s_rollback

  // -----------------------------------------------
  // FP Exceptions
  // send fflags bits to the CSRFile to accrue

  val fflags_val = Wire(Vec(coreWidth, Bool()))
  val fflags     = Wire(Vec(coreWidth, UInt(freechips.rocketchip.tile.FPConstants.FLAGS_SZ.W)))

  for (w <- 0 until coreWidth) {
    fflags_val(w) := rob_head_fflags(w).valid && io.commit.valids(w)
    fflags(w) := Mux(fflags_val(w), rob_head_fflags(w).bits, 0.U)

    assert (!(io.commit.valids(w) &&
              io.commit.uops(w).fp_val &&
             !(io.commit.uops(w).uses_stq || io.commit.uops(w).uses_ldq) &&
             !rob_head_fflags(w).valid),
             "Committed FP instruction did not set fflag bits")

    // Vector arithmetic accrues fflags without fp_val (fp_val would route
    // it into FP writeback accounting it does not use), so exempt a
    // committing vector uop from this otherwise-non-FP-instruction check.
    // Scala-level false.B when usingRVV is false, so the assert reverts to
    // exactly its baseline form.
    val committing_is_vec = if (usingRVV) io.commit.uops(w).is_vec.get else false.B
    assert (!(io.commit.valids(w) &&
             !io.commit.uops(w).fp_val &&
             !committing_is_vec &&
             rob_head_fflags(w).valid),
             "Committed non-FP instruction has non-zero fflag bits.")
    assert (!(io.commit.valids(w) &&
             io.commit.uops(w).fp_val &&
             (io.commit.uops(w).uses_ldq || io.commit.uops(w).uses_stq) &&
             (rob_head_fflags(w).bits =/= 0.U && rob_head_fflags(w).valid)),
             "Committed FP load or store has non-zero fflag bits.")
  }
  io.commit.fflags.valid := fflags_val.reduce(_|_)
  io.commit.fflags.bits  := fflags.reduce(_|_)

  // vxsat: the same commit-time accrual discipline as fflags above, applied
  // to the fixed-point saturation flag instead of the FP flags -- see the
  // vec_rob_flags block for why this cannot be set at writeback.
  if (usingRVV) {
    val vxsat_val = (0 until coreWidth).map(w => rob_head_vxsat.get(w) && io.commit.valids(w))
    io.com_vxsat.get := vxsat_val.reduce(_ || _)
  }

  // -----------------------------------------------
  // Exception Tracking Logic
  // only store the oldest exception, since only one can happen!

  val next_xcpt_uop = Wire(new MicroOp())
  next_xcpt_uop := r_xcpt_uop
  val enq_xcpts = Wire(Vec(coreWidth, Bool()))
  for (i <- 0 until coreWidth) {
    enq_xcpts(i) := io.enq_valids(i) && io.enq_uops(i).exception
  }

  when (!(io.flush.valid || exception_thrown)) {
    val new_xcpt_valid = io.lxcpt.valid || io.csr_replay.valid

    val lxcpt_older = !io.csr_replay.valid || (IsOlder(io.lxcpt.bits.uop.rob_idx, io.csr_replay.bits.uop.rob_idx, rob_head_idx) && io.lxcpt.valid)
    val new_xcpt = Mux(lxcpt_older, io.lxcpt.bits, io.csr_replay.bits)

    when (new_xcpt_valid) {
      when (!r_xcpt_val || IsOlder(new_xcpt.uop.rob_idx, r_xcpt_uop.rob_idx, rob_head_idx)) {
        r_xcpt_val              := true.B
        next_xcpt_uop           := new_xcpt.uop
        next_xcpt_uop.exc_cause := new_xcpt.cause
        r_xcpt_badvaddr         := new_xcpt.badvaddr
      }
    } .elsewhen (!r_xcpt_val && enq_xcpts.reduce(_|_)) {
      val idx = enq_xcpts.indexWhere{i: Bool => i}

      // if no exception yet, dispatch exception wins
      r_xcpt_val      := true.B
      next_xcpt_uop   := io.enq_uops(idx)
      r_xcpt_badvaddr := AlignPCToBoundary(io.xcpt_fetch_pc, icBlockBytes) | io.enq_uops(idx).pc_lob

    }
  }

  r_xcpt_uop         := next_xcpt_uop
  r_xcpt_uop.br_mask := GetNewBrMask(io.brupdate, next_xcpt_uop)
  when (IsKilledByBranch(io.brupdate, io.flush.valid, next_xcpt_uop)) {
    r_xcpt_val := false.B
  }

  assert (!(exception_thrown && !r_xcpt_val),
    "ROB trying to throw an exception, but it doesn't have a valid xcpt_cause")

  assert (!(empty && r_xcpt_val),
    "ROB is empty, but believes it has an outstanding exception.")

  assert (!(will_throw_exception && (GetRowIdx(r_xcpt_uop.rob_idx) =/= rob_head)),
    "ROB is throwing an exception, but the stored exception information's " +
    "rob_idx does not match the rob_head")

  // -----------------------------------------------
  // ROB Head Logic

  // remember if we're still waiting on the rest of the dispatch packet, and prevent
  // the rob_head from advancing if it commits a partial parket before we
  // dispatch the rest of it.
  // update when committed ALL valid instructions in commit_bundle

  val r_partial_row = RegInit(false.B)

  val finished_committing_row =
    (io.commit.valids.asUInt =/= 0.U) &&
    ((will_commit.asUInt ^ rob_head_vals.asUInt) === 0.U) &&
    !(r_partial_row && rob_head === rob_tail && !io.brupdate.b2.mispredict)

  when (finished_committing_row) {
    next_rob_head     := WrapInc(rob_head, numRobRows)
    rob_head_lsb := 0.U
  } .elsewhen (rob_state === s_rollback) {
    rob_head_lsb := 0.U
  } .otherwise {
    rob_head_lsb := OHToUInt(PriorityEncoderOH(rob_head_vals.asUInt))
  }

  // -----------------------------------------------
  // ROB Point-of-No-Return (PNR) Logic
  // Acts as a second head, but only waits on busy instructions which might cause misspeculation.
  // TODO is it worth it to add an extra 'parity' bit to all rob pointer logic?
  // Makes 'older than' comparisons ~3x cheaper, in case we're going to use the PNR to do a large number of those.
  // Also doesn't require the rob tail (or head) to be exported to whatever we want to compare with the PNR.

  if (enableFastPNR) {
    val unsafe_entry_in_rob = rob_unsafe_masked.reduce(_||_)
    val next_rob_pnr_idx = Mux(unsafe_entry_in_rob,
                               AgePriorityEncoder(rob_unsafe_masked, rob_head_idx),
                               rob_tail << log2Ceil(coreWidth) | PriorityEncoder(~rob_tail_vals.asUInt))
    rob_pnr := next_rob_pnr_idx >> log2Ceil(coreWidth)
    if (coreWidth > 1)
      rob_pnr_lsb := next_rob_pnr_idx(log2Ceil(coreWidth)-1, 0)
  } else {
    val safe_to_inc = rob_state === s_normal || rob_state === s_wait_till_empty
    val do_inc_row = !rob_pnr_unsafe.reduce(_||_) && !(rob_pnr === rob_tail && !io.brupdate.b2.mispredict)
    when (rob_state === s_rollback) {
      assert(rob_pnr === rob_head)
      rob_pnr_lsb := 0.U
    } .elsewhen (empty && io.enq_valids.asUInt =/= 0.U) {
      // Unforunately for us, the ROB does not use its entries in monotonically
      //  increasing order, even in the case of no exceptions. The edge case
      //  arises when partial rows are enqueued and committed, leaving an empty
      //  ROB.
      rob_pnr     := rob_head
      rob_pnr_lsb := PriorityEncoder(io.enq_valids)
    } .elsewhen (safe_to_inc && do_inc_row) {
      rob_pnr     := WrapInc(rob_pnr, numRobRows)
      rob_pnr_lsb := 0.U
    } .elsewhen (safe_to_inc && (rob_pnr =/= rob_tail)) {
      rob_pnr_lsb := PriorityEncoder(rob_pnr_unsafe)
    } .elsewhen (safe_to_inc && !full && !empty) {
      rob_pnr_lsb := PriorityEncoder(rob_pnr_unsafe.asUInt | ~MaskLower(rob_tail_vals.asUInt))
    }
  }

  // Head overrunning PNR likely means an entry hasn't been marked as safe when it should have been.
  assert(!IsOlder(rob_pnr_idx, rob_head_idx, rob_tail_idx) || rob_pnr_idx === rob_tail_idx)

  // PNR overrunning tail likely means an entry has been marked as safe when it shouldn't have been.
  assert(!IsOlder(rob_tail_idx, rob_pnr_idx, rob_head_idx) || full)

  // -----------------------------------------------
  // ROB Tail Logic

  when (io.brupdate.b2.mispredict) {
    rob_tail      := WrapInc(GetRowIdx(io.brupdate.b2.uop.rob_idx), numRobRows)
    rob_tail_lsb  := 0.U
    r_partial_row := false.B
  } .elsewhen (io.enq_valids.asUInt =/= 0.U && !io.enq_partial_stall) {
    rob_tail      := WrapInc(rob_tail, numRobRows)
    rob_tail_lsb  := 0.U
    r_partial_row := false.B
  } .elsewhen (io.enq_valids.asUInt =/= 0.U && io.enq_partial_stall) {
    rob_tail_lsb  := PriorityEncoder(~MaskLower(io.enq_valids.asUInt))
    r_partial_row := true.B
  }


  // -----------------------------------------------
  // Full/Empty Logic

  full       := WrapInc(rob_tail, numRobRows) === rob_head
  empty      := (rob_head === rob_tail) && (rob_head_vals.asUInt === 0.U)

  io.rob_head_idx := rob_head_idx
  io.rob_tail_idx := rob_tail_idx
  io.rob_pnr_idx  := rob_pnr_idx
  io.empty        := empty
  io.ready        := (rob_state === s_normal) && !full && !r_xcpt_val

  //-----------------------------------------------
  //-----------------------------------------------
  //-----------------------------------------------

  // ROB FSM
  switch (rob_state) {
    is (s_reset) {
      rob_state := s_normal
    }
    is (s_normal) {
      when (RegNext(RegNext(exception_thrown))) {
        rob_state := s_rollback
      } .otherwise {
        for (w <- 0 until coreWidth) {
          when (io.enq_valids(w) && io.enq_uops(w).is_unique) {
            rob_state := s_wait_till_empty
          }
        }
      }
    }
    is (s_rollback) {
      rob_tail     := rob_head
      rob_tail_lsb := 0.U
      rob_state    := s_normal
    }
    is (s_wait_till_empty) {
      when (RegNext(RegNext(exception_thrown))) {
        rob_state := s_rollback
      } .elsewhen (empty) {
        rob_state := s_normal
      }
    }
  }

  // -----------------------------------------------
  // Outputs

  io.com_load_is_at_rob_head := RegNext(rob_head_uses_ldq(PriorityEncoder(rob_head_vals.asUInt)) &&
                                        !will_commit.reduce(_||_))



  override def toString: String = BoomCoreStringPrefix(
    "==ROB==",
    "Machine Width      : " + coreWidth,
    "Rob Entries        : " + numRobEntries,
    "Rob Rows           : " + numRobRows,
    "Rob Row size       : " + log2Ceil(numRobRows),
    "log2Ceil(coreWidth): " + log2Ceil(coreWidth))
}

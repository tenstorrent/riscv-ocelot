// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

import boom.exu.FUConstants._
import boom.common._
import boom.exu._
import boom.util._

import chisel3.dontTouch // this is for debugging purposes


// Scoreboard (a queue of MicroOps with holes) to track vector operations
// and their ordering across the CPU-VPU interface..
class OviScoreboard(val SB_SIZE: Int = 32)(implicit p: Parameters) extends BoomModule {
  val io = IO(new Bundle {
    // insert MicroOp
    val insert  = Flipped(Decoupled(new MicroOp))
    // remove MicroOp by index
    val remove = new Bundle {
      val idx   = Input(UInt(log2Ceil(SB_SIZE).W))
      val valid = Input(Bool())
      val ready = Output(Bool()) // simply a "NOT empty" signal
      val uop   = Output(new MicroOp)
    }
    // expose the tail as sb_id
    val next_sb_id = Output(UInt(log2Ceil(SB_SIZE).W))
    // signals from the core to help speculation
    val core = new Bundle {
      val rob_pnr_idx  = Input(UInt(robAddrSz.W)) // point of no return index
      val rob_head_idx = Input(UInt(robAddrSz.W)) // ROB head index
      val brupdate     = Input(new BrUpdateInfo()) // branch update info
      val exception    = Input(Bool()) // exception signal
    }
    // dispatch signals into VPU
    val dispatch = new Bundle {
      val dispatch_sb_id       = Output(UInt(log2Ceil(SB_SIZE).W)) // dispatch id
      val dispatch_next_senior = Output(Bool()) // next senior pending entry
      val dispatch_kill        = Output(Bool()) // next kill pending entry
    }
    // debug signals
    val debug = new Bundle {
      val debug_head     = Output(UInt((log2Ceil(SB_SIZE)+1).W))
      val debug_tail     = Output(UInt((log2Ceil(SB_SIZE)+1).W))
      val debug_sb_state = Output(Vec(SB_SIZE, UInt(3.W))) // 3 bits for up to 8 states
      val debug_sb_uop   = Output(Vec(SB_SIZE, new MicroOp))
    }
  })

  // ============================================================
  // helper definitions and checks

  // error if SB_SIZE is not a power of 2 (wrap bit logic wont work)
  require(isPow2(SB_SIZE), "SB_SIZE must be a power of 2")

  object SBState extends ChiselEnum {
    val INVALID, DISPATCH, SENIOR_PENDING, SENIOR, KILL_PENDING = Value
  }
  def wrapInc(idx: UInt, max: Int): UInt = Mux((idx === (max-1).U), 0.U, idx + 1.U)

  def is_to_be_killed(uop: MicroOp): Bool = {
    (io.core.exception && !IsOlder(uop.rob_idx, io.core.rob_pnr_idx, io.core.rob_head_idx)) ||
    IsKilledByBranch(io.core.brupdate, uop)
  }

  def is_to_be_senior(uop: MicroOp): Bool = {
    IsOlder(uop.rob_idx, io.core.rob_pnr_idx, io.core.rob_head_idx) ||
    (uop.rob_idx === io.core.rob_pnr_idx)
  }

  // ============================================================
  // initializations

  // scoreboard (basically a uop array) container data
  val sb_uop   = Reg(Vec(SB_SIZE, new MicroOp()))
  val sb_state = RegInit(VecInit(Seq.fill(SB_SIZE)(SBState.INVALID)))

  val head = RegInit(0.U((log2Ceil(SB_SIZE)+1).W)) // read ptr with wrap state
  val tail = RegInit(0.U((log2Ceil(SB_SIZE)+1).W)) // write ptr with wrap state

  val head_ptr = head(log2Ceil(SB_SIZE)-1, 0)
  val tail_ptr = tail(log2Ceil(SB_SIZE)-1, 0)

  val empty = (head_ptr === tail_ptr) && (head(log2Ceil(SB_SIZE)) === tail(log2Ceil(SB_SIZE)))
  val full  = (head_ptr === tail_ptr) && (head(log2Ceil(SB_SIZE)) =/= tail(log2Ceil(SB_SIZE)))

  // dispatch signals
  val pending_entries = VecInit(
    (0 until SB_SIZE).map(i => (sb_state(i) === SBState.SENIOR_PENDING) ||
                               (sb_state(i) === SBState.KILL_PENDING))
  )
  val pending_psel = PriorityEncoderOH(pending_entries)

  // ============================================================
  // scoreboard interface logic

  // io connections
  io.insert.ready := !full  // insert ready if not full
  io.remove.ready := !empty // remove ready if not empty
  io.next_sb_id := tail_ptr // next available id is tail for a queue-like structure
  io.remove.uop := sb_uop(io.remove.idx) // expose the "response uop"
  // default dispatch signals (will be updated in state machine)
  io.dispatch.dispatch_sb_id := 0.U
  io.dispatch.dispatch_next_senior := false.B
  io.dispatch.dispatch_kill := false.B

  // per-entry state machine
  for (i <- 0 until SB_SIZE) {
    switch (sb_state(i)) {

      // INV to DIS during insert at tail_ptr
      is (SBState.INVALID) {
        when (io.insert.fire && (tail_ptr === i.U)) {
          sb_uop(i) := io.insert.bits
          sb_uop(i).br_mask := GetNewBrMask(
            io.core.brupdate,
            io.insert.bits
          ) // same cycle enq-br update
          sb_state(i) := Mux(
            is_to_be_killed(io.insert.bits),
            SBState.KILL_PENDING,
            SBState.DISPATCH
          ) // same cycle enq-kill
          tail := wrapInc(tail, SB_SIZE)
        }
      }

      // DIS to SEN/KILL PENDING due to core decisions
      is (SBState.DISPATCH) {
        when (is_to_be_senior(sb_uop(i))) {
          sb_state(i) := SBState.SENIOR_PENDING
        } .elsewhen (is_to_be_killed(sb_uop(i))) {
          sb_state(i) := SBState.KILL_PENDING
        } .otherwise {
          sb_uop(i).br_mask := GetNewBrMask(io.core.brupdate, sb_uop(i))
        }
      }

      // SEN.P to SEN on next_senior signal sent
      is (SBState.SENIOR_PENDING) {
        when (pending_psel(i)) {
          sb_state(i) := SBState.SENIOR
          io.dispatch.dispatch_next_senior := true.B
          io.dispatch.dispatch_sb_id := i.U
        }
      }

      // SEN to INV on remove
      is (SBState.SENIOR) {
        when (io.remove.valid && (io.remove.idx === i.U)) {
          sb_state(i) := SBState.INVALID
        }
      }

      // KILL.P to INV on kill signal sent
      is (SBState.KILL_PENDING) {
        when (pending_psel(i)) {
          sb_state(i) := SBState.INVALID
          io.dispatch.dispatch_kill := true.B
          io.dispatch.dispatch_sb_id := i.U
        }
      }
    }
  }

  // track head for in-orderness
  // NOTE: head only moves a single entry at a time so there
  //       could be delays due to holes in the queue
  when (!empty && (
    (sb_state(head_ptr) === SBState.INVALID) ||       // to skip invalid entries only
    (io.remove.valid && (io.remove.idx === head_ptr)) // to bypass removed uop in same cycle
  )) {
    head := wrapInc(head, SB_SIZE)
  }

  // ============================================================
  // assertions for debugging

  when (io.remove.valid) {
    assert(sb_state(io.remove.idx) === SBState.SENIOR,
      p"[SB] Remove valid but entry ${io.remove.idx} is not in SENIOR state! State=${sb_state(io.remove.idx)}")
  }

  // Assertion: If any entry is SENIOR_PENDING or KILL_PENDING, then at least one dispatch signal is active
  val any_pending = (0 until SB_SIZE).map { i =>
    (sb_state(i) === SBState.SENIOR_PENDING) || (sb_state(i) === SBState.KILL_PENDING)
  }.reduce(_||_)

  when (any_pending) {
    assert(io.dispatch.dispatch_next_senior ^ io.dispatch.dispatch_kill,
      "[SB] There is a pending entry but both next_sen and kill are either on or off!")
  }

  // Assert: For all entries from head_ptr to tail_ptr in DISPATCH state,
  // no instruction after should be IsOlder than any uop before it.
  // Only check entries in DISPATCH state.
  for (i <- 0 until SB_SIZE) {
    for (j <- (i + 1) until SB_SIZE) {

      val old   = (head + i.U)(log2Ceil(SB_SIZE), 0)
      val young = (head + j.U)(log2Ceil(SB_SIZE), 0)
      val old_ptr   =   old(log2Ceil(SB_SIZE)-1, 0)
      val young_ptr = young(log2Ceil(SB_SIZE)-1, 0)

      when ((sb_state(old_ptr) === SBState.DISPATCH) && (sb_state(young_ptr) === SBState.DISPATCH)) {
        // younger is returning older in function
        when (IsOlder(sb_uop(young_ptr).rob_idx, sb_uop(old_ptr).rob_idx, io.core.rob_head_idx)) {
          // assert false
          val young_rob_idx = sb_uop(young_ptr).rob_idx
          val old_rob_idx   = sb_uop(old_ptr).rob_idx
          val rob_head_idx = io.core.rob_head_idx
          // print debug info
          assert(false.B, p"[SB] Entry $young_ptr after $old_ptr in queue is older (IsOlder) than $old_ptr! young_rob_idx=$young_rob_idx, old_rob_idx=$old_rob_idx, rob_head_idx=$rob_head_idx")
        }
      }
    }
  }

  // debug signals
  io.debug.debug_head     := head_ptr
  io.debug.debug_tail     := tail_ptr
  io.debug.debug_sb_state := sb_state.map(_.asUInt)
  io.debug.debug_sb_uop   := sb_uop

  // Prevent Chisel from optimizing away these signals
  dontTouch(io.insert.valid)
  dontTouch(io.insert.bits.rob_idx)
  dontTouch(io.insert.bits.debug_inst)
  dontTouch(io.insert.bits.debug_pc)
  dontTouch(io.remove.valid)
  dontTouch(io.remove.idx)
  // dontTouch(io.core)
  dontTouch(io.dispatch)
  dontTouch(io.debug.debug_head)
  dontTouch(io.debug.debug_tail)
  dontTouch(io.debug.debug_sb_state)

}

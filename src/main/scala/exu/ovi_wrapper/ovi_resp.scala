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
import boom.lsu.VGenResp

import chisel3.dontTouch // this is for debugging purposes

// Response handler for OVI wrapper
// NOTE: there are some hard-coded constants in this module
class OviLSURespHandler(
  val MAX_OUTSTANDING_VMEMOPS: Int,
  val vpuVlen: Int,
  val oviWidth: Int,
  val lsuDmemWidth: Int
)(implicit p: Parameters) extends BoomModule {
  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // from vlsiq
    val enq = Flipped(ValidIO(new EnhancedFuncUnitReq(xLen, vLen)))
    // from lsu
    val lsu_resp = Input(new VGenResp(coreDataBits))
    // from fake load return queue
    val fake_load_return_data = Flipped(new DecoupledIO(UInt(34.W)))
    // from core (to maintain speculation)
    val core_in = new Bundle {
      val rob_pnr_idx  = Input(UInt(robAddrSz.W))
      val rob_head_idx = Input(UInt(robAddrSz.W))
      val brupdate     = Input(new BrUpdateInfo())
      val exception    = Input(Bool())
    }
    // to core (to report exceptions and clear busy)
    val core_out = ValidIO(new VecMemClrUnsafe())
    // part of the core_out bundle but need to send this info to sb
    val core_out_sb_id = Output(UInt(5.W))
    // to vpu
    val vpu = new Bundle {
      // sync end
      val sync_end     = Output(Bool())
      val sb_id        = Output(UInt(5.W))
      val vstart_vlfof = Output(UInt(15.W))
      // data
      val load_valid   = Output(Bool())
      val load_seq_id  = Output(UInt(34.W))
      val load_data    = Output(UInt(512.W))
      val load_mask_valid = Output(Bool())
      val load_mask    = Output(UInt(64.W))
    }
  })

  // ========== Variable Definitions ==========

  object State extends ChiselEnum {
    val WAIT, PENDING, DONE = Value
  }

  class VstartVlfofTrackerEntry extends BoomBundle {
    val valid = Bool()
    val core_report = State()
    val vpu_report = State()
    val exception = Bool()
    val xcpt_cause = UInt(xLen.W)
    val vstart_vlfof = UInt(15.W)
    val badvaddr = UInt(coreMaxAddrBits.W)
    val uop = new MicroOp()
    val sb_id = UInt(5.W)
  }

  // there could be a port from the decoder for this, but this will do for now
  def uop_is_fof(uop: MicroOp): Bool = {
    val instMop  = uop.inst(27, 26) // should be "unit"
    val instUMop = uop.inst(24, 20) // should be "fof"
    val instOP   = uop.inst(6, 0)   // should be "load"
    return (instMop === 0.U) && (instUMop === 16.U) && (instOP === 7.U)
  }

  val vsvlf_tracker = RegInit(VecInit.fill(MAX_OUTSTANDING_VMEMOPS)(0.U.asTypeOf(new VstartVlfofTrackerEntry)))
  
  // ========== default outputs ==========

  io.vpu.sync_end := false.B
  io.vpu.sb_id := 0.U
  io.vpu.vstart_vlfof := 0.U
  io.core_out.valid := false.B
  io.core_out.bits := DontCare
  io.core_out_sb_id := DontCare
  io.fake_load_return_data.ready := false.B
  io.vpu.load_valid := false.B
  io.vpu.load_seq_id := 0.U
  io.vpu.load_data := 0.U
  io.vpu.load_mask_valid := false.B
  io.vpu.load_mask := 0.U

  // ========== Data Handler ==========

  // ready-valid logic
  val lsu_load_data_resp = io.lsu_resp.vectorDataBack && io.lsu_resp.s0l1
  io.fake_load_return_data.ready := !lsu_load_data_resp

  // propogate data from LSU to VPU
  when (lsu_load_data_resp) {
    io.vpu.load_valid := true.B
    io.vpu.load_seq_id := Cat(
      io.lsu_resp.sbId(4, 0),
      io.lsu_resp.elemCount(6, 0),
      io.lsu_resp.elemOffset(5, 0),
      0.U((11-log2Ceil(vpuVlen/8)).W), // pad to 11
      io.lsu_resp.elemID(log2Ceil(vpuVlen/8)-1, 0),
      io.lsu_resp.vRegID(4, 0)
    )
    io.vpu.load_data := Mux(
      io.lsu_resp.strideDir,
      Cat(io.lsu_resp.data ((lsuDmemWidth-1), 0), 0.U((oviWidth-lsuDmemWidth).W)), // negative stride
      Cat(0.U, io.lsu_resp.data ((lsuDmemWidth-1), 0))                             // positive stride
    )
    io.vpu.load_mask_valid := io.lsu_resp.isMask
    io.vpu.load_mask := io.lsu_resp.Mask

  // send data from fake queue to VPU
  }.elsewhen (io.fake_load_return_data.fire) {
    io.vpu.load_valid := true.B
    io.vpu.load_seq_id := io.fake_load_return_data.bits
    io.vpu.load_data := 0.U
    io.vpu.load_mask_valid := false.B
    io.vpu.load_mask := 0.U
  }

  // ========== Update vsvlf_tracker with core signals ==========
  // NOTE: the core being marked as DONE earlier is similar to the poison bit functionality here

  val core_report_vec = WireInit(VecInit.fill(MAX_OUTSTANDING_VMEMOPS)(State.WAIT))
  val enq_uop = Wire(new MicroOp())
  val enq_core_report = WireInit(State.WAIT)

  for (i <- 0 until MAX_OUTSTANDING_VMEMOPS) {
    core_report_vec(i) := vsvlf_tracker(i).core_report
    when (
        (vsvlf_tracker(i).valid) &&
        (vsvlf_tracker(i).core_report =/= State.DONE) &&
        ((io.core_in.exception && !IsOlder(vsvlf_tracker(i).uop.rob_idx, io.core_in.rob_pnr_idx, io.core_in.rob_head_idx)) ||
         (IsKilledByBranch(io.core_in.brupdate, vsvlf_tracker(i).uop)))
    ) {
      core_report_vec(i) := State.DONE // this will be seen in the same cycle
      vsvlf_tracker(i).core_report := State.DONE // this is to update the reg
    }
    vsvlf_tracker(i).uop.br_mask := GetNewBrMask(io.core_in.brupdate, vsvlf_tracker(i).uop)
  }
  enq_uop := io.enq.bits.req.uop
  enq_uop.br_mask := GetNewBrMask(io.core_in.brupdate, io.enq.bits.req.uop)
  enq_core_report := Mux((
    (io.enq.bits.poison) ||
    (io.core_in.exception && !IsOlder(io.enq.bits.req.uop.rob_idx, io.core_in.rob_pnr_idx, io.core_in.rob_head_idx)) ||
    (IsKilledByBranch(io.core_in.brupdate, io.enq.bits.req.uop))
  ), State.DONE, State.WAIT)


  // ========== Vstart Vlfof Tracker Logic ==========

  // find the next available slot
  val invalid_vec = VecInit(vsvlf_tracker.indices.map { i => !vsvlf_tracker(i).valid })
  val vsvlf_next_available_vec = Mux(invalid_vec.asUInt.orR, PriorityEncoderOH(invalid_vec.asUInt), 0.U)

  for (i <- 0 until MAX_OUTSTANDING_VMEMOPS) {
    // when a memop ends (last possible el_id for vstart/vlfof has been sent)
    // we need to send the final value to the VPU and reset the tracker
    when (
      (vsvlf_tracker(i).valid) &&
      ((io.lsu_resp.vectorDoneSt && vsvlf_tracker(i).sb_id === io.lsu_resp.sbIdDoneSt) ||
       (io.lsu_resp.vectorDoneLd && vsvlf_tracker(i).sb_id === io.lsu_resp.sbIdDoneLd))
    ) {
      // if theres a valid excepted element on same cycle as the memop ends, update the value
      // (dont have to worry about "early_report" since we will are reporting rn anyways)
      when (
        (io.lsu_resp.vectorDataBack && vsvlf_tracker(i).sb_id === io.lsu_resp.sbId) &&
        (io.lsu_resp.exception) &&
        (vsvlf_tracker(i).vstart_vlfof > io.lsu_resp.elemID)
      ) {
        vsvlf_tracker(i).exception    := true.B
        vsvlf_tracker(i).xcpt_cause   := io.lsu_resp.xcpt_cause
        vsvlf_tracker(i).vstart_vlfof := io.lsu_resp.elemID
        vsvlf_tracker(i).badvaddr     := io.lsu_resp.badvaddr
        assert(core_report_vec(i) === State.WAIT, "ERROR: smaller VstartVlfof after reporting to core?!")
      }
      // -- move statuses from wait to pending (if necessary) --
      when (core_report_vec(i) === State.WAIT) {
        vsvlf_tracker(i).core_report := State.PENDING
      }
      assert(vsvlf_tracker(i).vpu_report === State.WAIT, "ERROR: VstartVlfof tracker slot is already marked pending!")
      vsvlf_tracker(i).vpu_report := State.PENDING
    }

    // when theres a valid memory packet that is being transferred
    // we need to update the tracker with the minimum value
    .elsewhen (
      (vsvlf_tracker(i).valid) &&
      (io.lsu_resp.vectorDataBack && vsvlf_tracker(i).sb_id === io.lsu_resp.sbId) &&
      (io.lsu_resp.exception)
    ) {
      // update the value if the new exception is smaller
      when (vsvlf_tracker(i).vstart_vlfof > io.lsu_resp.elemID) {
        vsvlf_tracker(i).exception    := true.B
        vsvlf_tracker(i).xcpt_cause   := io.lsu_resp.xcpt_cause
        vsvlf_tracker(i).vstart_vlfof := io.lsu_resp.elemID
        vsvlf_tracker(i).badvaddr     := io.lsu_resp.badvaddr
        assert(core_report_vec(i) === State.WAIT, "ERROR: smaller VstartVlfof after reporting to core?!")
      }
      // check for early report requests
      when (io.lsu_resp.xcpt_early_report && core_report_vec(i) === State.WAIT) {
        vsvlf_tracker(i).core_report := State.PENDING
      }
      vsvlf_tracker(i).exception := true.B
      assert(vsvlf_tracker(i).vpu_report === State.WAIT, "ERROR: VstartVlfof tracker slot is already marked pending!")
    }

    // when a new memory transaction is starting
    // reserve a slot for the new transaction (valid + sb_id)
    .elsewhen (
      (io.enq.valid) &&
      (vsvlf_next_available_vec(i))
    ) {
      assert(vsvlf_tracker(i).valid === false.B, "ERROR: VstartVlfof tracker slot is already marked valid!")
      assert(!(VecInit(vsvlf_next_available_vec).asUInt & (VecInit(vsvlf_next_available_vec).asUInt - 1.U)), "ERROR: VstartVlfof tracker has multiple slots marked available!")
      vsvlf_tracker(i).valid := true.B
      vsvlf_tracker(i).core_report := enq_core_report
      vsvlf_tracker(i).vpu_report := State.WAIT
      vsvlf_tracker(i).exception := false.B
      vsvlf_tracker(i).xcpt_cause := 0.U
      vsvlf_tracker(i).vstart_vlfof := 0.U
      vsvlf_tracker(i).uop := enq_uop
      vsvlf_tracker(i).sb_id := io.enq.bits.sb_id
    }
  }


  // ========== "Deq" or Output/Report Logic ==========

  // select one of the done slots to report output
  val vsvlf_reportable_vec = VecInit(vsvlf_tracker.indices.map { i =>
    vsvlf_tracker(i).valid && ((vsvlf_tracker(i).vpu_report === State.PENDING) || (core_report_vec(i) === State.PENDING))
  })
  val vsvlf_report_candidate_vec = Mux(vsvlf_reportable_vec.asUInt.orR, PriorityEncoderOH(vsvlf_reportable_vec.asUInt), 0.U)

  // select an entry and report either just vpu or both vpu and core
  for (i <- 0 until MAX_OUTSTANDING_VMEMOPS) {
    when (vsvlf_report_candidate_vec(i)) {
      // report memop_sync end to VPU
      when (vsvlf_tracker(i).vpu_report === State.PENDING) {
        io.vpu.sync_end             := true.B
        io.vpu.sb_id                := vsvlf_tracker(i).sb_id
        io.vpu.vstart_vlfof         := vsvlf_tracker(i).vstart_vlfof
        vsvlf_tracker(i).vpu_report := State.DONE // unnecessary: gonna be cleared anyways
      }
      // report xcpt or safe signal to core
      when (core_report_vec(i) === State.PENDING) {
        io.core_out.valid := true.B
        io.core_out.bits.uop          := vsvlf_tracker(i).uop
        io.core_out.bits.exception    := vsvlf_tracker(i).exception
        io.core_out.bits.xcpt_cause   := vsvlf_tracker(i).xcpt_cause
        io.core_out.bits.vstart_vlfof := vsvlf_tracker(i).vstart_vlfof
        io.core_out.bits.is_fof       := uop_is_fof(vsvlf_tracker(i).uop)
        io.core_out.bits.badvaddr     := vsvlf_tracker(i).badvaddr
        io.core_out_sb_id             := vsvlf_tracker(i).sb_id
        vsvlf_tracker(i).core_report  := State.DONE // unnecessary: gonna be cleared anyways
      }
      // clear the entry (0ing sets the valid to 0)
      vsvlf_tracker(i) := 0.U.asTypeOf(new VstartVlfofTrackerEntry)

      // assertions
      assert(!(VecInit(vsvlf_report_candidate_vec).asUInt & (VecInit(vsvlf_report_candidate_vec).asUInt - 1.U)), "ERROR: VstartVlfof tracker has multiple slots marked reportable!")
      assert(vsvlf_tracker(i).vpu_report =/= State.DONE, "ERROR: VstartVlfof tracker slot is marked done!")
      assert(core_report_vec(i) === State.PENDING || core_report_vec(i) === State.DONE, "ERROR: VstartVlfof tracker slot is not marked pending or done!")
    }
  }

  // ========== extra assertions ==========

  for (i <- 0 until MAX_OUTSTANDING_VMEMOPS) {
    assert(!(vsvlf_tracker(i).valid && (vsvlf_tracker(i).vpu_report === State.DONE)), "ERROR: VstartVlfof tracker: valid entry cannot have vpu_report in DONE state!")
  }
  assert(!(io.enq.valid && vsvlf_tracker.map(_.valid).reduce(_ && _)), "ERROR: VstartVlfof tracker: enq valid fired when all entries are full!")
  // Assert: no two valid entries can have the same sb_id
  for (i <- 0 until MAX_OUTSTANDING_VMEMOPS) {
    for (j <- i + 1 until MAX_OUTSTANDING_VMEMOPS) {
      assert(!(vsvlf_tracker(i).valid && vsvlf_tracker(j).valid && (vsvlf_tracker(i).sb_id === vsvlf_tracker(j).sb_id)),
        s"ERROR: VstartVlfof tracker: Entries $i and $j have the same sb_id!")
    }
  }

}


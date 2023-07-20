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
import boom.lsu.{LSUExeIO}

import hardfloat._

class CombMemory[T <: Data](size: Int, gen: T) extends Module {
  val io = IO(new Bundle {
    val wrAddr = Input(UInt(log2Ceil(size).W))
    val wrData = Input(gen.cloneType)
    val wrEn   = Input(Bool())
    val rdAddr = Input(UInt(log2Ceil(size).W))
    val rdData = Output(gen.cloneType)
  })

  val mem = RegInit(VecInit(Seq.fill(size)(0.U.asTypeOf(gen))))

  when(io.wrEn) {
    mem(io.wrAddr) := io.wrData
  }

  io.rdData := mem(io.rdAddr)
}

class EnhancedFuncUnitReq(xLen: Int, vLen: Int)(implicit p: Parameters) extends Bundle {
  val vconfig = new VConfig()
  val vxrm = UInt(2.W)
  val fcsr_rm = UInt(3.W)
  val req = new FuncUnitReq(xLen)
}

class OviWrapper(xLen: Int, vLen: Int)(implicit p: Parameters)
    extends BoomModule
    with freechips.rocketchip.rocket.constants.MemoryOpConstants {
  val io = IO(new Bundle {
    val vconfig = Input(new VConfig())
    val vxrm = Input(UInt(2.W))
    val fcsr_rm = Input(UInt(3.W))
    val req = Flipped(new DecoupledIO(new FuncUnitReq(xLen)))
    val resp = new DecoupledIO(new FuncUnitResp(xLen))
    val set_vxsat = Output(Bool())
    val vGenIO = Flipped(new boom.lsu.VGenIO)
    
    val debug_wb_vec_valid = Output(Bool())
    val debug_wb_vec_wdata = Output(UInt((coreParams.vLen * 8).W))
    val debug_wb_vec_wmask = Output(UInt(8.W))
  })

  val reqQueue = Module(new Queue(new EnhancedFuncUnitReq(xLen, vLen), 4))
  val uOpMem = Module(new CombMemory(32, new MicroOp()))
  val vpuModule = Module(new tt_vpu_ovi(vLen))
  val maxIssueCredit = 16
  val issueCreditCnt = RegInit(maxIssueCredit.U(log2Ceil(maxIssueCredit + 1).W))
  issueCreditCnt := issueCreditCnt + vpuModule.io.issue_credit - vpuModule.io.issue_valid

  reqQueue.io.enq.valid := io.req.valid
  reqQueue.io.enq.bits.req := io.req.bits
  reqQueue.io.enq.bits.vconfig := io.vconfig
  reqQueue.io.enq.bits.vxrm := io.vxrm
  reqQueue.io.enq.bits.fcsr_rm := io.fcsr_rm

  val sbId = RegInit(0.U(5.W))
  sbId := sbId + vpuModule.io.issue_valid

  uOpMem.io.wrEn := reqQueue.io.deq.valid
  uOpMem.io.wrAddr := sbId
  uOpMem.io.wrData := reqQueue.io.deq.bits.req.uop
  uOpMem.io.rdAddr := vpuModule.io.completed_sb_id

  val respUop = uOpMem.io.rdData

  io := DontCare
  io.req.ready := reqQueue.io.deq.ready
  io.resp.valid := vpuModule.io.completed_valid
  io.resp.bits.data := vpuModule.io.completed_dest_reg
  io.resp.bits.uop := respUop
  io.resp.bits.uop.dst_rtype := Mux(respUop.dst_rtype === RT_VEC,
                                    RT_X,
                                    respUop.dst_rtype)
  io.resp.bits.uop.uses_stq := 0.B // Trick Rob to acknowledge Vector Store
  io.resp.bits.fflags.valid := vpuModule.io.completed_fflags.orR
  io.resp.bits.fflags.bits.uop.rob_idx := io.resp.bits.uop.rob_idx
  io.resp.bits.fflags.bits.flags := vpuModule.io.completed_fflags

  io.set_vxsat := DontCare
  io.debug_wb_vec_valid := vpuModule.io.debug_wb_vec_valid
  io.debug_wb_vec_wdata := vpuModule.io.debug_wb_vec_wdata
  io.debug_wb_vec_wmask := vpuModule.io.debug_wb_vec_wmask

/*
  faking mem sync start
*/
  //val fakeLoadStart = ShiftRegister (reqQueue.io.deq.valid && reqQueue.io.deq.bits.req.uop.uses_ldq, 200)

   val internalMemSyncStart = vpuModule.io.memop_sync_start
   val tryDeqVLSIQ = RegInit(false.B)
   val internalStoreWrite = vpuModule.io.store_valid
   

/*
  vLSIQ start
*/


  val inMiddle = RegInit(false.B)
  val outStandingReq = RegInit(0.U)
  val vOSud = Cat (vpuModule.io.memop_sync_start, vpuModule.io.memop_sync_end)
  when (vOSud === 1.U) {
    outStandingReq := outStandingReq - 1.U
  }.elsewhen (vOSud === 2.U) {
    outStandingReq := outStandingReq + 1.U 
  }

  val vLSIQueue = Module(new Queue(new EnhancedFuncUnitReq(xLen, vLen), 2))
  val sbIdQueue = Module(new Queue(UInt(5.W), 2))

  // this needs to be changed in the future to include load, just keep it this way for now
  vLSIQueue.io.enq.valid := issueCreditCnt =/= 0.U && reqQueue.io.deq.valid && (reqQueue.io.deq.bits.req.uop.uses_stq || reqQueue.io.deq.bits.req.uop.uses_ldq)
  vLSIQueue.io.enq.bits := reqQueue.io.deq.bits
  vLSIQueue.io.deq.ready := !inMiddle && ((outStandingReq =/= 0.U || internalMemSyncStart) || tryDeqVLSIQ)
  sbIdQueue.io.enq.valid := reqQueue.io.deq.valid && (reqQueue.io.deq.bits.req.uop.uses_stq || reqQueue.io.deq.bits.req.uop.uses_ldq)
  sbIdQueue.io.enq.bits := sbId
  sbIdQueue.io.deq.ready := !inMiddle && ((outStandingReq =/= 0.U || internalMemSyncStart) || tryDeqVLSIQ)
  when (!inMiddle) {
  when (vLSIQueue.io.deq.valid && vLSIQueue.io.deq.ready) {
    inMiddle := true.B 
  }.elsewhen ((outStandingReq =/= 0.U || internalMemSyncStart) && !vLSIQueue.io.deq.valid) {
    tryDeqVLSIQ := true.B 
    inMiddle := true.B 
  }.elsewhen (tryDeqVLSIQ && vLSIQueue.io.deq.valid) {
    tryDeqVLSIQ := false.B
    inMiddle := true.B 
  }
  }.elsewhen (vpuModule.io.memop_sync_end) {
    inMiddle := false.B
  }
  reqQueue.io.deq.ready := issueCreditCnt =/= 0.U && vLSIQueue.io.enq.ready
  val newVGenConfig = vLSIQueue.io.deq.valid && vLSIQueue.io.deq.ready && (vLSIQueue.io.deq.bits.req.uop.uses_stq || vLSIQueue.io.deq.bits.req.uop.uses_ldq)
/*
  vLSIQ end
*/

/*
  VDB start
*/
  val vdb = Module (new VDB(512, 64, 4))
  
  vdb.io.writeValid := false.B 
//  vdb.io.writeData := 0.U 
  vdb.io.pop := false.B 
  vdb.io.last := false.B  
  vdb.io.configValid := false.B  

  vdb.io.writeValid := internalStoreWrite
  vdb.io.writeData := vpuModule.io.store_data
  vdb.io.sliceSize := 8.U 

 

/*
   VDB end
*/
/*
   Vid Gen Start
*/
val vIdGen = Module (new VIdGen(32, 8))
 vIdGen.io.configValid := false.B 
 vIdGen.io.startID := DontCare
 vIdGen.io.startVD := DontCare  
 vIdGen.io.pop := false.B
 vIdGen.io.sliceSize := 8.U

val vAGen = Module (new VAgen ())

  vAGen.io.configValid := false.B 

  vAGen.io.startAddr := DontCare
  vAGen.io.stride := DontCare
  vAGen.io.isStride := DontCare 
  vAGen.io.sliceSize := 8.U 
  vAGen.io.vl := 4.U
  vAGen.io.pop := false.B  
  /*
      Decode Start
  */

  io.vGenIO.req.valid := false.B 
  io.vGenIO.req.bits := DontCare
  io.vGenIO.reqHelp.valid := false.B
  io.vGenIO.reqHelp.bits := DontCare 

  val vGenEnable  = RegInit(false.B)
  val vGenHold = Reg(new EnhancedFuncUnitReq(xLen, vLen))
  val sbIdHold = RegInit(0.U)
  val vDBcount = RegInit(0.U(3.W))
  val vDBud = Cat (vpuModule.io.store_valid, vdb.io.release)
  when (vDBud === 1.U) {
    vDBcount := vDBcount - 1.U
  }.elsewhen (vDBud === 2.U) {
    vDBcount := vDBcount + 1.U 
  }
  


  val s0l1 = RegInit(false.B)

  val strideDirHold = RegInit(true.B)

  val sliceSizeHold = RegInit(0.U)


  when (newVGenConfig && !vGenEnable) {
    vGenEnable := true.B 
    vGenHold.req.uop := vLSIQueue.io.deq.bits.req.uop
    sbIdHold := sbIdQueue.io.deq.bits 
    vdb.io.configValid := vLSIQueue.io.deq.bits.req.uop.uses_stq
    vIdGen.io.configValid := vLSIQueue.io.deq.bits.req.uop.uses_ldq
    vIdGen.io.startID := 0.U
    s0l1 := vLSIQueue.io.deq.bits.req.uop.uses_ldq
    vAGen.io.configValid := true.B
    vAGen.io.vl := vLSIQueue.io.deq.bits.vconfig.vl
    vAGen.io.startAddr := vLSIQueue.io.deq.bits.req.rs1_data
    vAGen.io.stride := vLSIQueue.io.deq.bits.req.rs2_data 
    
    // this is fine for now, change later for index store
    val instElemSize = vLSIQueue.io.deq.bits.req.uop.inst(14, 12)
    val vldDest = vLSIQueue.io.deq.bits.req.uop.inst(11, 7)
    strideDirHold := vLSIQueue.io.deq.bits.req.rs2_data(31)
    vIdGen.io.startVD := vldDest
    
    when (instElemSize === 0.U) {
    //  vdb.io.sliceSize := 1.U
    //  vAGen.io.sliceSize := 1.U
    //  vIdGen.io.sliceSize := 1.U
    sliceSizeHold := 1.U   
    }.elsewhen (instElemSize === 5.U){
   // vdb.io.sliceSize := 2.U
   // vAGen.io.sliceSize := 2.U
   // vIdGen.io.sliceSize := 2.U
   sliceSizeHold := 2.U 
    }.elsewhen (instElemSize === 6.U){
    //  vdb.io.sliceSize := 4.U
    //  vAGen.io.sliceSize := 4.U
    //  vIdGen.io.sliceSize := 4.U
    sliceSizeHold := 4.U 
    }.otherwise{
    //   vdb.io.sliceSize := 8.U 
    //   vAGen.io.sliceSize := 8.U
    //   vIdGen.io.sliceSize := 8.U
    sliceSizeHold := 8.U 
    }
  
    
    val instMop = vLSIQueue.io.deq.bits.req.uop.inst(27, 26)
    when (instMop === 0.U) {
      vAGen.io.isStride := false.B 
    }.otherwise {
      vAGen.io.isStride := true.B 
    }
  }

    vdb.io.sliceSize := sliceSizeHold 
    vAGen.io.sliceSize := sliceSizeHold 
    vIdGen.io.sliceSize := sliceSizeHold 
    
  

  io.vGenIO.req.valid := vGenEnable && ((!s0l1 && vDBcount =/= 0.U) || s0l1)
  io.vGenIO.req.bits.uop := vGenHold.req.uop
  io.vGenIO.req.bits.data := Mux(s0l1, 0.U, vdb.io.outData) 
  io.vGenIO.req.bits.last := false.B 
  io.vGenIO.req.bits.addr := vAGen.io.outAddr

  io.vGenIO.reqHelp.bits.elemID := vIdGen.io.outID 
  io.vGenIO.reqHelp.bits.vRegID := vIdGen.io.outVD
  io.vGenIO.reqHelp.bits.sbId   := sbIdHold
  io.vGenIO.reqHelp.bits.strideDir := strideDirHold 

  val MemSyncEnd = io.vGenIO.resp.bits.vectorDone && io.vGenIO.resp.valid && inMiddle 
  val MemSbId = io.vGenIO.resp.bits.sbId
  val MemCredit = vdb.io.release 
  val MemVstart = 0.U

  val MEMLoadValid = WireInit(false.B)
  val MEMLoadData = WireInit(0.U(512.W))
  val MEMSeqId    = WireInit(0.U(34.W))

    
  val seqSbId = io.vGenIO.resp.bits.sbId
  val seqElCount = WireInit(1.U(7.W))
  val seqElOff = WireInit(0.U(6.W))
  val seqElId = io.vGenIO.resp.bits.elemID
  val seqVreg = io.vGenIO.resp.bits.vRegID

  MEMSeqId := Cat (seqSbId, seqElCount, seqElOff, 0.U(3.W), seqElId, seqVreg)

  MEMLoadValid := io.vGenIO.resp.valid && io.vGenIO.resp.bits.s0l1 && !io.vGenIO.resp.bits.vectorDone  // needs fixing later if we are overlapping
  when (MEMLoadValid) {
    when (io.vGenIO.resp.bits.strideDir){  // negative
       when (io.vGenIO.resp.bits.memSize === 0.U) {
        MEMLoadData := Cat(io.vGenIO.resp.bits.data (7, 0), 0.U) 
       }.elsewhen (io.vGenIO.resp.bits.memSize === 1.U) {
        MEMLoadData := Cat(io.vGenIO.resp.bits.data (15, 0), 0.U)       
       }.elsewhen (io.vGenIO.resp.bits.memSize === 2.U) {
        MEMLoadData := Cat(io.vGenIO.resp.bits.data (31, 0), 0.U)
       }.elsewhen (io.vGenIO.resp.bits.memSize === 3.U) {
        MEMLoadData := Cat(io.vGenIO.resp.bits.data (63, 0), 0.U)
       }
    }.otherwise {
      when (io.vGenIO.resp.bits.memSize === 0.U) {
        MEMLoadData := Cat(0.U, io.vGenIO.resp.bits.data (7, 0)) 
       }.elsewhen (io.vGenIO.resp.bits.memSize === 1.U) {
        MEMLoadData := Cat(0.U, io.vGenIO.resp.bits.data (15, 0))       
       }.elsewhen (io.vGenIO.resp.bits.memSize === 2.U) {
        MEMLoadData := Cat(0.U, io.vGenIO.resp.bits.data (31, 0))
       }.elsewhen (io.vGenIO.resp.bits.memSize === 3.U) {
        MEMLoadData := Cat(0.U, io.vGenIO.resp.bits.data (63, 0))
       }
    }
  }

  val MEMReturnMaskValid = WireInit(false.B) 
  val MEMReturnMask = WireInit(0.U(64.W))
  val MEMMaskCredit = WireInit(false.B)

  io.vGenIO.req.bits.last := vAGen.io.last 
  when (io.vGenIO.req.valid && io.vGenIO.req.ready && 
                              ((io.vGenIO.req.bits.uop.uses_stq && !io.vGenIO.resp.bits.dsqFull) || (io.vGenIO.req.bits.uop.uses_ldq && !io.vGenIO.resp.bits.dlqFull))) {
    vdb.io.pop := io.vGenIO.req.bits.uop.uses_stq
    vAGen.io.pop := true.B 
    vIdGen.io.pop := io.vGenIO.req.bits.uop.uses_ldq
    when (vAGen.io.last) {
      vGenEnable := false.B         
      vdb.io.last := io.vGenIO.req.bits.uop.uses_stq
    }
  }

  
  /*
      Decode End
  */

  vpuModule.io := DontCare
  vpuModule.io.clk := clock
  vpuModule.io.reset_n := ~reset.asBool
  vpuModule.io.issue_valid := reqQueue.io.deq.valid && reqQueue.io.deq.ready
  vpuModule.io.issue_inst := reqQueue.io.deq.bits.req.uop.inst
  vpuModule.io.issue_sb_id := sbId
  vpuModule.io.issue_scalar_opnd := Mux( (reqQueue.io.deq.bits.req.uop.uses_ldq || reqQueue.io.deq.bits.req.uop.uses_stq), reqQueue.io.deq.bits.req.rs2_data, reqQueue.io.deq.bits.req.rs1_data)
  vpuModule.io.issue_vcsr := Cat(
    0.U(1.W), // vill
    reqQueue.io.deq.bits.vconfig.vtype.vsew, // vsew
    reqQueue.io.deq.bits.vconfig.vtype.vlmul_mag, // vlmul
    reqQueue.io.deq.bits.fcsr_rm, // frm
    reqQueue.io.deq.bits.vxrm, // vxrm
    Cat(0.U((15-log2Ceil(vLen+1)).W),
        reqQueue.io.deq.bits.vconfig.vl), // vl
    0.U(14.W) // vstart
  )
  vpuModule.io.issue_vcsr_lmulb2 := reqQueue.io.deq.bits.vconfig.vtype.vlmul_sign
  vpuModule.io.dispatch_sb_id := sbId
  vpuModule.io.dispatch_next_senior := reqQueue.io.deq.valid
  vpuModule.io.dispatch_kill := 0.B
  vpuModule.io.memop_sync_end := MemSyncEnd
  vpuModule.io.store_credit := MemCredit

  vpuModule.io.load_seq_id := MEMSeqId
  vpuModule.io.load_data := MEMLoadData
  vpuModule.io.load_valid := MEMLoadValid
  vpuModule.io.load_mask := MEMReturnMask
  vpuModule.io.load_mask_valid := MEMReturnMaskValid


}

class tt_vpu_ovi (vLen: Int)(implicit p: Parameters) extends BlackBox(Map("VLEN" -> IntParam(vLen))) with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clk = Input(Clock())
    val reset_n = Input(Bool())
    val issue_inst = Input(UInt(32.W))
    val issue_sb_id = Input(UInt(5.W))
    val issue_scalar_opnd = Input(UInt(64.W))
    val issue_vcsr = Input(UInt(40.W))
    val issue_vcsr_lmulb2 = Input(Bool()) // Added 1 more bit for vlmul
    val issue_valid = Input(Bool())
    val issue_credit = Output(Bool())
    val dispatch_sb_id = Input(UInt(5.W))
    val dispatch_next_senior = Input(Bool())
    val dispatch_kill = Input(Bool())
    val completed_valid = Output(Bool())
    val completed_sb_id = Output(UInt(5.W))
    val completed_fflags = Output(UInt(5.W))
    val completed_dest_reg = Output(UInt(64.W))
    val completed_vxsat = Output(Bool())
    val completed_vstart = Output(UInt(14.W))
    val completed_illegal = Output(Bool())
    val store_valid = Output(Bool())
    val store_data = Output(UInt(512.W))
    val store_credit = Input(Bool())
    val memop_sync_end = Input(Bool())
    val memop_sync_start = Output(Bool())
    val debug_wb_vec_valid = Output(Bool())
    val debug_wb_vec_wdata = Output(UInt((vLen * 8).W))
    val debug_wb_vec_wmask = Output(UInt(8.W))
    val load_seq_id = Input(UInt(34.W))
    val load_data = Input(UInt(512.W))
    val load_valid = Input (Bool())
    val load_mask = Input(UInt(64.W))
    val load_mask_valid = Input (Bool())
  })
  // addResource("/vsrc/vpu_ovi/tt_vpu_ovi.sv")
  addResource("/vsrc/vpu/briscv_defines.h")
  addResource("/vsrc/vpu/tt_briscv_pkg.vh")
  addResource("/vsrc/vpu/autogen_riscv_imabfv.v")
  addResource("/vsrc/vpu/autogen_defines.h")
  addResource("/vsrc/vpu/vfp_pipeline.sv")
  addResource("/vsrc/vpu/tt_id.sv")
  addResource("/vsrc/vpu/tt_ex.sv")
  addResource("/vsrc/vpu/tt_lq.sv")
  addResource("/vsrc/vpu/tt_mem.sv")
  addResource("/vsrc/vpu/tt_vec.sv")
  addResource("/vsrc/vpu/tt_vec_iadd.sv")
  addResource("/vsrc/vpu/tt_vec_idp.sv")
  addResource("/vsrc/vpu/tt_vec_imul.sv")
  addResource("/vsrc/vpu/tt_vec_mul_dp.sv")
  addResource("/vsrc/vpu/tt_vec_regfile.sv")
  addResource("/vsrc/vpu/tt_vfp_unit.sv")
  addResource("/vsrc/vpu/tt_vfp_ex_unit.sv")
  addResource("/vsrc/vpu/tt_vfp_lane.sv")
  addResource("/vsrc/vpu/tt_vfp_encoder.sv")
  addResource("/vsrc/vpu/tt_vfp_encoder_lane.sv")
  addResource("/vsrc/vpu/tt_vfp_fma.sv")
  addResource("/vsrc/vpu/tt_vfp_red.sv")
  addResource("/vsrc/vpu/tt_popcnt.sv")
  addResource("/vsrc/vpu/tt_pipe_stage.sv")
  addResource("/vsrc/vpu/tt_rts_rtr_pipe_stage.sv")
  addResource("/vsrc/vpu/tt_cam_buffer.sv")
  addResource("/vsrc/vpu/tt_skid_buffer.sv")
  addResource("/vsrc/vpu/tt_store_fifo.sv")
  addResource("/vsrc/vpu/tt_ffs.sv")
  addResource("/vsrc/vpu/tt_ascii_instrn_decode.sv")
  addResource("/vsrc/vpu/tt_compare.sv")
  addResource("/vsrc/vpu/tt_decoded_mux.sv")
  addResource("/vsrc/vpu/tt_decoder.sv")
  addResource("/vsrc/vpu/tt_reshape.sv")
  addResource("/vsrc/vpu/tt_memop_fsm.sv")
  addResource("/vsrc/vpu/tt_fifo.sv")
  addResource("/vsrc/vpu/tt_vpu_ovi.sv")  
  addResource("/vsrc/HardFloat/source/RISCV/HardFloat_specialize.v")
  addResource("/vsrc/HardFloat/source/RISCV/HardFloat_specialize.vi")
  addResource("/vsrc/HardFloat/source/HardFloat_consts.vi")
  addResource("/vsrc/HardFloat/source/HardFloat_localFuncs.vi")
  addResource("/vsrc/HardFloat/source/HardFloat_primitives.v")
  addResource("/vsrc/HardFloat/source/HardFloat_rawFN.v")
  addResource("/vsrc/HardFloat/source/addRecFN.v")
  addResource("/vsrc/HardFloat/source/compareRecFN.v")
  addResource("/vsrc/HardFloat/source/divSqrtRecFN_small.v")
  addResource("/vsrc/HardFloat/source/fNToRecFN.v")
  addResource("/vsrc/HardFloat/source/iNToRecFN.v")
  addResource("/vsrc/HardFloat/source/isSigNaNRecFN.v")
  addResource("/vsrc/HardFloat/source/mulAddRecFN.v")
  addResource("/vsrc/HardFloat/source/mulRecFN.v")
  addResource("/vsrc/HardFloat/source/recFNToFN.v")
  addResource("/vsrc/HardFloat/source/recFNToIN.v")
  addResource("/vsrc/HardFloat/source/recFNToRecFN.v")
  
}


class VAgen(implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val sliceSize = Input(UInt(4.W))
    val vl = Input(UInt(9.W))
    val configValid = Input(Bool())
    val startAddr = Input(UInt(64.W))
    val stride = Input(UInt(64.W))
    val isStride = Input(Bool())
    val pop = Input(Bool())
    val outAddr = Output(UInt(40.W))
    val last = Output(Bool())
  })

//  val sliceSizeHold = Reg(UInt(4.W))
  val vlHold = Reg(UInt(9.W))

  val currentIndex = Reg(UInt(9.W))
  val currentAddr  = Reg(UInt(64.W))
  val stride  = Reg(UInt(64.W))
  val working = RegInit(false.B)
  val isStride = Reg(Bool())
  

  io.outAddr := currentAddr(39, 0)

  when (io.configValid) {
  //  sliceSizeHold := io.sliceSize
    vlHold := io.vl - 1.U
    currentIndex := 0.U 
    currentAddr := io.startAddr
    isStride := io.isStride
    stride := io.stride 
    working := true.B 
  }
  
  io.last := (currentIndex === vlHold) && working

  when (io.pop) {
    when (io.last) {
      working := false.B 
      currentIndex := 0.U
    }.otherwise {
      currentIndex := currentIndex + 1.U
      when (isStride) {
          currentAddr := currentAddr + stride 
      }.otherwise { 
          currentAddr := currentAddr + io.sliceSize 
      }    
    }
  }


}

// M is max number of byte per VLEN (32), N is max number of byte per memory interface (8) 
class VIdGen(val M: Int, val N: Int)(implicit p: Parameters) extends Module {
  require(isPow2(M), "M must be a power of 2")
  require(isPow2(N), "N must be a power of 2")
  require(M >= N, "M must be greater than or equal to N")
  require(M % 8 == 0, "M must be a multiple of 8")
  require(N % 8 == 0, "N must be a multiple of 8")
  val S = log2Ceil(M + 1)
  val I = log2Ceil(M) + 3
  val K = log2Ceil(M)
  

  val io = IO(new Bundle {
    val configValid = Input(Bool())
    val startID = Input(UInt(I.W))
    val startVD = Input(UInt(5.W))
    val pop = Input(Bool())
    val sliceSize = Input(UInt(S.W))
    val outID = Output(UInt(I.W))
    val outVD = Output(UInt(5.W))
  }) 

  val currentID = RegInit(0.U(I.W))
  val currentVD = RegInit(0.U(5.W))
  val count = RegInit(0.U(S.W))
//  val step = RegInit(0.U(S.W))

  when (io.configValid) {
    currentID := io.startID
    currentVD := io.startVD
    count := 0.U  // for now
//    step := io.sliceSize
  }
  io.outID := currentID
  io.outVD := currentVD
  when (io.pop) {
    currentID := currentID + 1.U 
    when (count + io.sliceSize === M.U) {
      count := 0.U
      currentVD := currentVD + 1.U
    }.otherwise{
      count := count + io.sliceSize
    }
  }

}  


class VDB(val M: Int, val N: Int, val Depth: Int)(implicit p: Parameters) extends Module {
  require(isPow2(M), "M must be a power of 2")
  require(isPow2(N), "N must be a power of 2")
  require(M >= N, "M must be greater than or equal to N")
  require(M % 8 == 0, "M must be a multiple of 8")
  require(N % 8 == 0, "N must be a multiple of 8")
  val S = log2Ceil(N / 8 + 1)
  val I = log2Ceil(M / 8 + 1)
  val maxIndex = (M / 8)

  val io = IO(new Bundle {
    val configValid = Input(Bool())
    val writeValid = Input(Bool())
    val writeData = Input(UInt(M.W))
    val pop = Input(Bool())
    val last = Input(Bool())
    val sliceSize = Input(UInt(S.W))
    val release = Output(Bool())
    val outData = Output(UInt(N.W))
  })

  val buffer = RegInit(VecInit(Seq.fill(Depth)(0.U(M.W))))
  val readPtr = RegInit(0.U(log2Ceil(Depth).W))
  val writePtr = RegInit(0.U(log2Ceil(Depth).W))
  val currentIndex = Reg(UInt(I.W))

  when(io.configValid) {
   currentIndex := 0.U
  }

  val currentEntry = buffer(readPtr)

  when(io.writeValid) {
    buffer(writePtr) := io.writeData
    writePtr := WrapInc(writePtr, Depth)
  }

  val paddedEntry = Cat(0.U((N-8).W), currentEntry)
  val slices = VecInit(Seq.tabulate(M/N)(i => paddedEntry((i+1)*N-1, i*N)))
  io.outData := slices(currentIndex)

  io.release := false.B 
  when (io.pop) {
    when (io.last) {
      readPtr := WrapInc(readPtr, Depth)
      currentIndex := 0.U
      io.release := true.B 
    } .otherwise {
      when (currentIndex + io.sliceSize === maxIndex.U) {
        currentIndex := 0.U
        readPtr := WrapInc(readPtr, Depth)
        io.release := true.B 
      } .otherwise {
        currentIndex := currentIndex + io.sliceSize
      }
    }
  }
}


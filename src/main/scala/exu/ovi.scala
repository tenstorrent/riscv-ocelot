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

/** Request queue that supports speculation.
 * 
 * This allows requests to be killed much closer to the OVI boundary.
 */ 
class OviReqQueue(val num_entries: Int)(implicit p: Parameters) extends BoomModule {
  val io = IO(new Bundle {
    val enq = DeqIO(new EnhancedFuncUnitReq(xLen, vLen))
    val deq = EnqIO(new EnhancedFuncUnitReq(xLen, vLen))
    val count = Output(UInt(log2Ceil(num_entries + 1).W))

    val rob_pnr_idx, rob_head_idx = Input(UInt(robAddrSz.W))
    val brupdate = Input(new BrUpdateInfo())
    val exception = Input(Bool())
  })

  val entries = Reg(Vec(num_entries, new EnhancedFuncUnitReq(xLen, vLen)))
  val entries_valid = RegInit(VecInit.fill(num_entries)(false.B))
  val enq_ptr, deq_ptr = RegInit(0.U(log2Ceil(num_entries).W))

  ////////////////////////////////////////////////////////////////

  io.count := PopCount(entries_valid)

  // Allow enqueue when not full
  io.enq.ready := entries_valid.asUInt =/= Fill(num_entries, 1.U)

  // Allow dequeue when not empty and past PNR
  io.deq.valid := entries_valid.asUInt =/= 0.U && (
    IsOlder(io.deq.bits.req.uop.rob_idx, io.rob_pnr_idx, io.rob_head_idx)
    || io.deq.bits.req.uop.rob_idx === io.rob_pnr_idx
  )

  ////////////////////////////////////////////////////////////////

  // Keep branch masks up-to-date
  for (idx <- 0 until num_entries) {
    val new_mask = GetNewBrMask(io.brupdate, entries(idx).req.uop)
    entries(idx).req.uop.br_mask := new_mask
  }

  def is_killed(uop: MicroOp): Bool = (
    // For exceptions, kill everything past the PNR
    (io.exception && !IsOlder(uop.rob_idx, io.rob_pnr_idx, io.rob_head_idx))
    // For branch mispredictions, kill based on branch masks
    || IsKilledByBranch(io.brupdate, uop)
  )
  val killed_entries = VecInit.tabulate(num_entries)(idx =>
    entries_valid(idx) && is_killed(entries(idx).req.uop)
  )

  for (idx <- 0 until num_entries)
    when(killed_entries(idx)) { entries_valid(idx) := false.B }

  // Take killed entries into account when finding a place for the next entry
  val new_enq_ptr = Mux(
    killed_entries.asUInt === 0.U,
    enq_ptr,
    AgePriorityEncoder(killed_entries, deq_ptr)
  )

  // Instructions might already be killed by the time they arrive
  val do_enq = io.enq.fire && !(
    IsKilledByBranch(io.brupdate, io.enq.bits.req.uop) ||
    io.exception || RegNext(io.exception)
  )
  when(do_enq) {
    entries(new_enq_ptr) := io.enq.bits
    entries_valid(new_enq_ptr) := true.B
  }
  enq_ptr := Mux(do_enq, WrapInc(new_enq_ptr, num_entries), new_enq_ptr)

  when(io.deq.fire) {
    entries_valid(deq_ptr) := false.B
    deq_ptr := WrapInc(deq_ptr, num_entries)
  }
  io.deq.bits := entries(deq_ptr)
}

////////////////////////////////////////////////////////////////

class EnhancedFuncUnitReq(xLen: Int, vLen: Int)(implicit p: Parameters) extends Bundle {
  val vconfig = new VConfig()
  val vxrm = UInt(2.W)
  val fcsr_rm = UInt(3.W)
  val req = new FuncUnitReq(xLen)
}

class OviWrapperCoreIO(implicit p: Parameters) extends BoomBundle {
  val vconfig = Input(new VConfig())
  val vxrm = Input(UInt(2.W))
  val set_vxsat = Output(Bool())

  val vGenIO = Flipped(new boom.lsu.VGenIO)

  val rob_pnr_idx, rob_head_idx = Input(UInt(robAddrSz.W))
  val exception = Input(Bool())
  
  val debug_wb_vec_valid = Output(Bool())
  val debug_wb_vec_wdata = Output(UInt((vLen * 8).W))
  val debug_wb_vec_wmask = Output(UInt(8.W))
}

class OviWrapper(implicit p: Parameters) extends BoomModule
    with freechips.rocketchip.rocket.constants.MemoryOpConstants {
  val io = IO(new Bundle {
    val fcsr_rm = Input(UInt(3.W))
    val req = Flipped(new DecoupledIO(new FuncUnitReq(xLen)))
    val resp = new DecoupledIO(new FuncUnitResp(xLen))
    val core = new OviWrapperCoreIO
    val brupdate = Input(new BrUpdateInfo())
  })

  io := DontCare

  val reqQueue = Module(new OviReqQueue(8))

  // The request pipeline takes several cycles to stall
  io.req.ready := reqQueue.io.count <= (reqQueue.num_entries - 3).U
  assert(!(reqQueue.io.enq.valid && !reqQueue.io.enq.ready),
         "OviReqQueue overflow")
  
  reqQueue.io.enq.valid := io.req.valid
  reqQueue.io.enq.bits.req := io.req.bits
  reqQueue.io.enq.bits.vconfig := io.core.vconfig
  reqQueue.io.enq.bits.vxrm := io.core.vxrm
  reqQueue.io.enq.bits.fcsr_rm := io.fcsr_rm

  reqQueue.io.rob_pnr_idx := io.core.rob_pnr_idx
  reqQueue.io.rob_head_idx := io.core.rob_head_idx
  reqQueue.io.brupdate := io.brupdate
  reqQueue.io.exception := io.core.exception
  
  val uOpMem = Mem(32, new MicroOp())
  val vpuModule = Module(new tt_vpu_ovi(vLen))
  val maxIssueCredit = 16
  val issueCreditCnt = RegInit(maxIssueCredit.U)
  issueCreditCnt := issueCreditCnt + vpuModule.io.issue_credit - vpuModule.io.issue_valid

  val sbId = Counter(0 until 32, vpuModule.io.issue_valid)._1
  when(reqQueue.io.deq.valid) { uOpMem.write(sbId, reqQueue.io.deq.bits.req.uop) }
  val respUop = uOpMem.read(vpuModule.io.completed_sb_id)

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

  io.core.set_vxsat := DontCare
  io.core.debug_wb_vec_valid := vpuModule.io.debug_wb_vec_valid
  io.core.debug_wb_vec_wdata := vpuModule.io.debug_wb_vec_wdata
  io.core.debug_wb_vec_wmask := vpuModule.io.debug_wb_vec_wmask

/*
   OVI LS helper start
*/


/*
  Input / Output with VPU should be defined here
*/
  

   val MemSyncStart = vpuModule.io.memop_sync_start
   val MemStoreValid = vpuModule.io.store_valid 
   val MemStoreData = vpuModule.io.store_data
   val MemMaskValid = vpuModule.io.mask_idx_valid
   val MemMaskId    = Cat(vpuModule.io.mask_idx_last_idx, vpuModule.io.mask_idx_item)
//   val MemLastId    = false.B 


  val MemSyncEnd = WireInit(false.B)
  val MemSbId = WireInit(0.U(5.W))
  val MemVstart = 0.U
  val MemLoadValid = WireInit (false.B)
  val MemSeqId = WireInit(0.U(34.W))
  val MemLoadData = WireInit(0.U(512.W))
  val MemReturnMaskValid = WireInit(false.B)
  val MemReturnMask = WireInit(0.U(64.W))
  val MemStoreCredit = WireInit(false.B)
  val MemMaskCredit = WireInit(false.B) 

  val seqSbId = WireInit(0.U(5.W))    // 5
  val seqElCount = WireInit(1.U(7.W)) // 7
  val seqElOff = WireInit(0.U(6.W))   // 6
  val seqElId = WireInit(0.U(11.W))   // 11
  val seqVreg = WireInit(0.U(5.W))    // 5



/*
   Constants Definition
*/  
   val vlsiQDepth = 4
   val oviWidth   = 512
   val lsuDmemWidth = 64
   val vpuVlen = 256
   val vdbDepth = 4
   val byteVreg = vpuVlen / 8
   val byteDmem = lsuDmemWidth / 8
   val vAGenDepth = 4

/*
  vLSIQ start
*/

  // in the middle of handling vector load store
  val inMiddle = RegInit(false.B)
  // trying to dequeue VLSIQ 
  val tryDeqVLSIQ = RegInit(false.B)
  // this chunk is checking the number of outstanding mem_sync_start
  val outStandingReq = RegInit(0.U(3.W))
  val vOSud = Cat (vpuModule.io.memop_sync_start, vpuModule.io.memop_sync_end)
  when (vOSud === 1.U) {
    outStandingReq := outStandingReq - 1.U
  }.elsewhen (vOSud === 2.U) {
    outStandingReq := outStandingReq + 1.U 
  }

  val vLSIQueue = Module(new Queue(new EnhancedFuncUnitReq(xLen, vLen), vlsiQDepth))
  val sbIdQueue = Module(new Queue(UInt(5.W), vlsiQDepth))

  // enqueue whenever VPU is ready && reqQueue is valid && there is ld or st 
  vLSIQueue.io.enq.valid := issueCreditCnt =/= 0.U && reqQueue.io.deq.valid && (reqQueue.io.deq.bits.req.uop.uses_stq || reqQueue.io.deq.bits.req.uop.uses_ldq)
  // grab everything from reqQueue
  vLSIQueue.io.enq.bits := reqQueue.io.deq.bits
  // can only dequeue vLSIQueue when it is not in the middle of handling vector load store and
  // 1. memSyncStart 2. pop out outStanding 3. previously tried
  vLSIQueue.io.deq.ready := !inMiddle && (outStandingReq =/= 0.U || MemSyncStart || tryDeqVLSIQ)
  // sbIdQueue is the same as vLSIQueue
  sbIdQueue.io.enq.valid := issueCreditCnt =/= 0.U && reqQueue.io.deq.valid && (reqQueue.io.deq.bits.req.uop.uses_stq || reqQueue.io.deq.bits.req.uop.uses_ldq)
  sbIdQueue.io.enq.bits := sbId
  sbIdQueue.io.deq.ready := !inMiddle && (outStandingReq =/= 0.U || MemSyncStart || tryDeqVLSIQ)
  when (!inMiddle) {
    // success transaction
     when (vLSIQueue.io.deq.valid && vLSIQueue.io.deq.ready) {
       inMiddle := true.B 
    // active pop failed
     }.elsewhen ((outStandingReq =/= 0.U || MemSyncStart) && !vLSIQueue.io.deq.valid) {
       tryDeqVLSIQ := true.B 
       inMiddle := true.B 
    // passive pop successful
     }.elsewhen (tryDeqVLSIQ && vLSIQueue.io.deq.valid) {
       tryDeqVLSIQ := false.B
       inMiddle := true.B 
     }
  // finish one vector load stroe
  }.elsewhen (MemSyncEnd) {
    inMiddle := false.B
  }
  reqQueue.io.deq.ready := issueCreditCnt =/= 0.U && vLSIQueue.io.enq.ready
  // a new set has dequeued from vLSIQueue
  val newVGenConfig = vLSIQueue.io.deq.valid && vLSIQueue.io.deq.ready 


/*
   v-Helper Start
*/


  val vAGen = Module (new VAgen (lsuDmemWidth, 66, vAGenDepth))

  vAGen.io.configValid := false.B 
  vAGen.io.maskData := MemMaskId  
  vAGen.io.maskValid := MemMaskValid  
  vAGen.io.startAddr := vLSIQueue.io.deq.bits.req.rs1_data 
  vAGen.io.stride := vLSIQueue.io.deq.bits.req.rs2_data
  vAGen.io.isStride := false.B 
  vAGen.io.isIndex := false.B 
  vAGen.io.isMask := false.B 
  vAGen.io.vl := vLSIQueue.io.deq.bits.vconfig.vl
  vAGen.io.pop := false.B  
  vAGen.io.initialSliceSize := 0.U
  vAGen.io.memSize := 0.U
  MemMaskCredit := vAGen.io.release

  val vdb = Module (new VDB(oviWidth, lsuDmemWidth, vpuVlen, vdbDepth))
  
  vdb.io.writeValid := false.B 
  vdb.io.pop := false.B 
  vdb.io.last := false.B  
  vdb.io.configValid := false.B  

  vdb.io.writeValid := MemStoreValid
  vdb.io.writeData := MemStoreData
  vdb.io.sliceSize := vAGen.io.sliceSizeOut 
  vdb.io.vlmul := vLSIQueue.io.deq.bits.vconfig.vtype.vlmul_mag
  MemStoreCredit := vdb.io.release

  
  // make sure that we have enough data
  val vDBcount = RegInit(0.U(3.W))
  val vDBud = Cat (MemStoreValid, MemStoreCredit)
  when (vDBud === 1.U) {
    vDBcount := vDBcount - 1.U
  }.elsewhen (vDBud === 2.U) {
    vDBcount := vDBcount + 1.U 
  }
  assert (vDBcount < 5.U)

   

val vIdGen = Module (new VIdGen(byteVreg, byteDmem))
 vIdGen.io.configValid := false.B 
 vIdGen.io.startID := 0.U 
 vIdGen.io.startVD := 0.U  
 vIdGen.io.pop := false.B
 vIdGen.io.sliceSize := vAGen.io.sliceSizeOut



  val vwhls = Module (new VWhLSDecoder (vpuVlen))
  vwhls.io.nf := 0.U 
  vwhls.io.wth := 0.U
  /*
      Decode Start
  */

  io.core.vGenIO.req.valid := false.B 
  io.core.vGenIO.req.bits := DontCare
  io.core.vGenIO.reqHelp.valid := false.B
  io.core.vGenIO.reqHelp.bits := DontCare 

  // need to send vGenIO request
  val vGenEnable  = RegInit(false.B)
  // holding the microOp from vLSIQueue
  val vGenHold = Reg(new EnhancedFuncUnitReq(xLen, vLen))
  // holding the sbId from sbIdQueue
  val sbIdExQueue = Module(new Queue(UInt(5.W), vlsiQDepth))
  sbIdExQueue.io.enq.valid := newVGenConfig
  sbIdExQueue.io.enq.bits := sbIdQueue.io.deq.bits 
  sbIdExQueue.io.deq.ready := MemSyncEnd 
  MemSbId := sbIdExQueue.io.deq.bits 
  // holding the sbId from sbIdQueue(vLSIQueue)
  val sbIdHold = RegInit(0.U)
  // holding wheter it is store or load
  val s0l1 = RegInit(false.B)
  // holding the direction of the stride, only useful for strided
  val strideDirHold = RegInit(true.B)
  val vlIsZero = RegInit(false.B)

  // Instruction Decoding
  
  val instNf = vLSIQueue.io.deq.bits.req.uop.inst(31, 29)
  val instMop = vLSIQueue.io.deq.bits.req.uop.inst(27, 26)
  val instMaskEnable = !vLSIQueue.io.deq.bits.req.uop.inst(25)  // 0: enable, 1 disable
  val instUMop = vLSIQueue.io.deq.bits.req.uop.inst(24, 20)
  val instElemSize = vLSIQueue.io.deq.bits.req.uop.inst(14, 12)
  val instVldDest = vLSIQueue.io.deq.bits.req.uop.inst(11, 7)
  val instOP = vLSIQueue.io.deq.bits.req.uop.inst(6, 0)
  val isWholeStore = instOP === 39.U && instUMop === 8.U  && instMop === 0.U
  val isWholeLoad =  instOP === 7.U  && instUMop === 8.U  && instMop === 0.U
  val isStoreMask =  instOP === 39.U && instUMop === 11.U && instMop === 0.U
  val isLoadMask =   instOP === 7.U  && instUMop === 11.U && instMop === 0.U
  val isIndex =                         instMop === 1.U || instMop === 3.U

  // start of a new round of vector load store
  when (newVGenConfig && !vGenEnable) {
    vGenEnable := true.B 
    vGenHold.req.uop := vLSIQueue.io.deq.bits.req.uop
    // Override mem_size for indexed load/store with sew from vtype
    vGenHold.req.uop.mem_size := Mux(isIndex, vLSIQueue.io.deq.bits.vconfig.vtype.vsew,
                                               vLSIQueue.io.deq.bits.req.uop.mem_size)
    sbIdHold := sbIdQueue.io.deq.bits 
    vlIsZero := vLSIQueue.io.deq.bits.vconfig.vl === 0.U &&
               !isWholeLoad &&
               !isWholeStore
    s0l1 := vLSIQueue.io.deq.bits.req.uop.uses_ldq    
    // only use vdb when it is store
    vdb.io.configValid := vLSIQueue.io.deq.bits.req.uop.uses_stq
       // default lmul
    vdb.io.vlmul := vLSIQueue.io.deq.bits.vconfig.vtype.vlmul_mag
    // only use vID when it is load
    vIdGen.io.configValid := vLSIQueue.io.deq.bits.req.uop.uses_ldq
       // vstart = 0 for now
    vIdGen.io.startID := 0.U
       // write back V dest
    vIdGen.io.startVD := instVldDest
    // vAGen always on
    vAGen.io.configValid := true.B
       // default vl    
    vAGen.io.vl := vLSIQueue.io.deq.bits.vconfig.vl
       // default base address and stride
    vAGen.io.startAddr := vLSIQueue.io.deq.bits.req.rs1_data
    vAGen.io.stride := vLSIQueue.io.deq.bits.req.rs2_data 
    vAGen.io.isMask := instMaskEnable
    vAGen.io.memSize := Mux(isIndex, vLSIQueue.io.deq.bits.vconfig.vtype.vsew,
                                               vLSIQueue.io.deq.bits.req.uop.mem_size)
    // special case of whole load and whole store
    when (isWholeLoad || isWholeStore) {
    vwhls.io.nf := instNf
    vwhls.io.wth := instElemSize 
    vAGen.io.vl := vwhls.io.overVl 
    vdb.io.vlmul := vwhls.io.overVlmul
    }.elsewhen (isStoreMask || isLoadMask) {
      vAGen.io.vl := (vLSIQueue.io.deq.bits.vconfig.vl + 7.U) >> 3
      vdb.io.vlmul := 0.U
    }
    // initial SliceSize    
    when (isWholeStore || isStoreMask || isLoadMask) {
      vAGen.io.initialSliceSize := 1.U 
    }.elsewhen (isIndex) {
      vAGen.io.initialSliceSize := 1.U << vLSIQueue.io.deq.bits.vconfig.vtype.vsew 
    }.otherwise {
    when (instElemSize === 0.U) {
      vAGen.io.initialSliceSize := 1.U 
    }.elsewhen (instElemSize === 5.U){
      vAGen.io.initialSliceSize := 2.U 
    }.elsewhen (instElemSize === 6.U){
      vAGen.io.initialSliceSize := 4.U 
    }.otherwise{
      vAGen.io.initialSliceSize := 8.U 
    }
    }
    // check unit stride / strided / index
    strideDirHold := 0.U 
    when (instMop === 0.U) {
      vAGen.io.isStride := false.B 
    }.elsewhen(instMop === 2.U) {
      vAGen.io.isStride := true.B 
      strideDirHold := vLSIQueue.io.deq.bits.req.rs2_data(31)
    }.otherwise {
      vAGen.io.isIndex := true.B
    }
  }

/*
   Fake load response for masked-off elements
*/

  val fakeLoadReturnQueue = Module(new Queue(UInt(34.W), 8))
  fakeLoadReturnQueue.io.enq.valid := s0l1 && (vAGen.io.popForce || vAGen.io.popForceLast)
  fakeLoadReturnQueue.io.enq.bits := Cat(sbIdHold, vAGen.io.elemCount, vAGen.io.elemOffset, 0.U(3.W), vIdGen.io.outID, vIdGen.io.outVD)
  fakeLoadReturnQueue.io.deq.ready := false.B 





  /*
     Output to LSU
  */ 

  io.core.vGenIO.req.valid := vGenEnable && ((!s0l1 && ((!vlIsZero && vDBcount =/= 0.U) || vlIsZero)) || s0l1) && vAGen.io.canPop
  io.core.vGenIO.req.bits.uop := vGenHold.req.uop
  io.core.vGenIO.req.bits.uop.mem_size := Mux(s0l1, 3.U, vGenHold.req.uop.mem_size)
  io.core.vGenIO.req.bits.data := Mux(s0l1, 0.U, vdb.io.outData) 
  io.core.vGenIO.req.bits.last := vAGen.io.last 
  io.core.vGenIO.req.bits.addr := Mux(s0l1, Cat(vAGen.io.outAddr(39, 3), 0.U(3.W)), vAGen.io.outAddr)

  io.core.vGenIO.reqHelp.bits.elemID := vIdGen.io.outID 
  io.core.vGenIO.reqHelp.bits.elemOffset := vAGen.io.elemOffset
  io.core.vGenIO.reqHelp.bits.elemCount := vAGen.io.elemCount  
  io.core.vGenIO.reqHelp.bits.vRegID := vIdGen.io.outVD
  io.core.vGenIO.reqHelp.bits.sbId   := sbIdHold
  io.core.vGenIO.reqHelp.bits.strideDir := strideDirHold 
  io.core.vGenIO.reqHelp.bits.isMask := vAGen.io.isMaskOut
  io.core.vGenIO.reqHelp.bits.Mask := vAGen.io.currentMaskOut
  io.core.vGenIO.reqHelp.bits.isFake := vAGen.io.isFake

/*
   FSM for V-helper
*/

  when ((io.core.vGenIO.req.valid && io.core.vGenIO.req.ready) || vAGen.io.popForce) {                                                         
    vdb.io.pop := io.core.vGenIO.req.bits.uop.uses_stq && !vlIsZero
    vAGen.io.pop := true.B 
    vIdGen.io.pop := io.core.vGenIO.req.bits.uop.uses_ldq && !vlIsZero
    when (vAGen.io.last) {
      vGenEnable := false.B         
      vdb.io.last := io.core.vGenIO.req.bits.uop.uses_stq && !vlIsZero
    }
  }



/*
   Parse LSU response
*/
  val LSUReturnLoadValid = WireInit(false.B)
  LSUReturnLoadValid := io.core.vGenIO.resp.valid && io.core.vGenIO.resp.bits.s0l1 && !io.core.vGenIO.resp.bits.vectorDone  // needs fixing later if we are overlapping
  val vReturnData = Module(new VReturnData(512, 64))
  vReturnData.io.memSize := io.core.vGenIO.resp.bits.memSize
  vReturnData.io.lsuData := io.core.vGenIO.resp.bits.data
  vReturnData.io.strideDir := io.core.vGenIO.resp.bits.strideDir

/*
   Data back to VPU
*/

  
  MemSyncEnd := io.core.vGenIO.resp.bits.vectorDone && io.core.vGenIO.resp.valid && inMiddle
  MemLoadValid := LSUReturnLoadValid || fakeLoadReturnQueue.io.deq.valid
  MemSeqId := Cat (seqSbId, seqElCount, seqElOff, seqElId, seqVreg) 

  when (LSUReturnLoadValid) {
    MemLoadData := vReturnData.io.oviData    
    seqSbId := io.core.vGenIO.resp.bits.sbId
    seqElCount := io.core.vGenIO.resp.bits.elemCount
    seqElOff := io.core.vGenIO.resp.bits.elemOffset
    seqElId := Cat(0.U(3.W), io.core.vGenIO.resp.bits.elemID)
    seqVreg := io.core.vGenIO.resp.bits.vRegID
    
    MemReturnMaskValid := io.core.vGenIO.resp.bits.isMask
    MemReturnMask := io.core.vGenIO.resp.bits.Mask
  }.elsewhen (fakeLoadReturnQueue.io.deq.valid) {    
    MemLoadData := 0.U     
    seqSbId := fakeLoadReturnQueue.io.deq.bits (33, 29)
    seqElCount := fakeLoadReturnQueue.io.deq.bits (28, 22)
    seqElOff := fakeLoadReturnQueue.io.deq.bits (21, 16)
    seqElId := fakeLoadReturnQueue.io.deq.bits (15, 5)
    seqVreg := fakeLoadReturnQueue.io.deq.bits (4, 0)
    MemReturnMaskValid := true.B 
    MemReturnMask := false.B 
    fakeLoadReturnQueue.io.deq.ready := true.B 
  }

  
  /*
      OVI LS helper end
  */

  vpuModule.io := DontCare
  vpuModule.io.clk := clock
  vpuModule.io.reset_n := ~reset.asBool
  vpuModule.io.issue_valid := reqQueue.io.deq.valid && reqQueue.io.deq.ready
  vpuModule.io.issue_inst := reqQueue.io.deq.bits.req.uop.inst
  vpuModule.io.issue_sb_id := sbId
  vpuModule.io.issue_scalar_opnd := Mux( (reqQueue.io.deq.bits.req.uop.lrs1_rtype === RT_FLT), reqQueue.io.deq.bits.req.rs3_data,
                                    Mux( (reqQueue.io.deq.bits.req.uop.uses_ldq ||
                                          reqQueue.io.deq.bits.req.uop.uses_stq             ), reqQueue.io.deq.bits.req.rs2_data,
                                                                                               reqQueue.io.deq.bits.req.rs1_data))
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
  vpuModule.io.dispatch_next_senior := reqQueue.io.deq.valid && reqQueue.io.deq.ready
  vpuModule.io.dispatch_kill := 0.B
  
     vpuModule.io.memop_sync_end := MemSyncEnd
// vpuModule.io.mem_sb_id := MEMSbId 
// vpuModule.io.mem_vstart := MEMVstart
   vpuModule.io.load_valid := MemLoadValid
   vpuModule.io.load_seq_id := MemSeqId
   vpuModule.io.load_data := MemLoadData
   vpuModule.io.load_mask_valid := MemReturnMaskValid
   vpuModule.io.load_mask := MemReturnMask
   vpuModule.io.store_credit := MemStoreCredit
   vpuModule.io.mask_idx_credit := vAGen.io.release




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
    val mask_idx_credit = Input(Bool())
    val mask_idx_item = Output (UInt(65.W))
    val mask_idx_valid = Output(Bool())
    val mask_idx_last_idx = Output(Bool())

  })
  addResource("/vsrc/vpu/briscv_defines.h")
  addResource("/vsrc/vpu/tt_briscv_pkg.vh")
  addResource("/vsrc/vpu/autogen_riscv_imabfv.v")
  addResource("/vsrc/vpu/autogen_defines.h")
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
  addResource("/vsrc/vpu/tt_ffs.sv")
  addResource("/vsrc/vpu/tt_ascii_instrn_decode.sv")
  addResource("/vsrc/vpu/tt_compare.sv")
  addResource("/vsrc/vpu/tt_decoded_mux.sv")
  addResource("/vsrc/vpu/tt_decoder.sv")
  addResource("/vsrc/vpu/tt_reshape.sv")
  addResource("/vsrc/vpu/tt_memop_fsm.sv")
  addResource("/vsrc/vpu/tt_mask_fsm.sv")
  addResource("/vsrc/vpu/lrm_model.sv")
  addResource("/vsrc/vpu/tt_fifo.sv")
  addResource("/vsrc/vpu/tt_vpu_ovi.sv")  
  addResource("/vsrc/vpu/tt_vpu_ovi_assert.sv")  
  addResource("/vsrc/HardFloat/source/RISCV/HardFloat_specialize.v")
  addResource("/vsrc/HardFloat/source/RISCV/HardFloat_specialize.vi")
  addResource("/vsrc/HardFloat/source/HardFloat_consts.vi")
  addResource("/vsrc/HardFloat/source/HardFloat_localFuncs.vi")
  addResource("/vsrc/HardFloat/source/HardFloat_primitives.v")
  addResource("/vsrc/HardFloat/source/HardFloat_rawFN.v")
  addResource("/vsrc/HardFloat/source/addRecFN.v")
  addResource("/vsrc/HardFloat/source/compareRecFN.v")
  addResource("/vsrc/HardFloat/source/fNToRecFN.v")
  addResource("/vsrc/HardFloat/source/iNToRecFN.v")
  addResource("/vsrc/HardFloat/source/isSigNaNRecFN.v")
  addResource("/vsrc/HardFloat/source/mulAddRecFN.v")
  addResource("/vsrc/HardFloat/source/recFNToFN.v")
  addResource("/vsrc/HardFloat/source/recFNToIN.v")
  addResource("/vsrc/HardFloat/source/recFNToRecFN.v")
  
}

// M is dmem bandwidth, N is mask interface width (66), Depth is mask buffer width (4)
class VAgen(val M: Int, val N: Int, val Depth: Int)(implicit p: Parameters) extends Module {
    val k = log2Ceil(M/8+1)
    val seg = k - 1  
  val io = IO(new Bundle {    
    // inteface with the VPU
    val maskData = Input(UInt(N.W))
    val maskValid = Input(Bool())
    val release = Output (Bool())
    // interface with OVI decode
    val configValid = Input(Bool())
    val initialSliceSize = Input(UInt(k.W))
    val sliceSizeOut = Output(UInt(k.W))
    val vl = Input(UInt(9.W))
    val memSize = Input(UInt(2.W))
      // which types of load store
    val isStride = Input(Bool())
    val isMask = Input(Bool())
    val isIndex = Input(Bool())
      // base address and stride    
    val startAddr = Input(UInt(64.W))
    val stride = Input(UInt(64.W))
    // interface with OVI VAGen
      // pop when there is handshake with LSU
    val pop = Input(Bool())
      // payloads
    val outAddr = Output(UInt(40.W))
    val last = Output(Bool())
    val isFake = Output (Bool())
    val isMaskOut   = Output (Bool())
    val currentMaskOut = Output (UInt(N.W))
    val elemOffset = Output(UInt(6.W))
    val elemCount = Output(UInt(7.W))
      // control, canPop needs handshaking, popForce is masked off
    val popForce = Output (Bool())
    val canPop = Output (Bool())
    val popForceLast = Output (Bool())

  })

  
  val vlHold = Reg(UInt(9.W))
  // points at element
  val currentIndex = Reg(UInt(9.W))
  // points at mask element in buffer
  val currentMaskIndex = Reg(UInt(9.W))

  // in strided, this is used to calculate and accumulate address
  // in index, this just stores the base address
  val currentAddr  = Reg(UInt(64.W))
  val stride  = Reg(UInt(64.W))
  // working or not
  val working = RegInit(false.B)
  // type of load / store
  val isStride = Reg(Bool())
  val isIndex = Reg(Bool())
  val isMask = Reg(Bool())
  // special case for vl = 0
  val fakeHold = RegInit(false.B)
  // holding the initial sliceSize, become more useful in V1
  val sliceSizeHold = RegInit(0.U(k.W))
  val memSizeHold = RegInit(0.U(2.W))
  val negativeStrideElemOffset = WireInit(0.U(k.W))
  val positiveStrideElemOffset = WireInit(0.U(k.W))
  negativeStrideElemOffset := k.U - memSizeHold - 1.U 
  positiveStrideElemOffset := io.outAddr(k-2, 0) >> memSizeHold 

  val isNegStride = isStride && stride(63)
  
  io.elemOffset := Mux(isNegStride, (negativeStrideElemOffset - positiveStrideElemOffset), positiveStrideElemOffset)
 
//  io.elemOffset := 0.U 
  io.elemCount := 1.U 
  

  when (io.configValid) {
     sliceSizeHold := io.initialSliceSize
     memSizeHold := io.memSize
     when (io.vl === 0.U) {
       vlHold := 0.U
       fakeHold := true.B
     }.otherwise {
       vlHold := io.vl - 1.U
     }
     working := true.B 
     isStride := io.isStride
     isIndex := io.isIndex 
     isMask := io.isMask
     currentIndex := 0.U 
     currentMaskIndex := 0.U
     currentAddr := io.startAddr     
     stride := io.stride     
  }
  
  io.sliceSizeOut := sliceSizeHold  // this is only for V0

  
  io.isMaskOut := isMask
  io.currentMaskOut := false.B 

  /*
     Mask Buffer to handle the mask input
  */

  val buffer = RegInit(VecInit(Seq.fill(Depth)(0.U(N.W))))
  val readPtr = RegInit(0.U(log2Ceil(Depth).W))
  val writePtr = RegInit(0.U(log2Ceil(Depth).W))
  when(io.maskValid) {
    buffer(writePtr) := io.maskData
    writePtr := WrapInc(writePtr, Depth)
  }
  
  /*
     Check if there is valid mask
  */ 
  val vMaskcount = RegInit(0.U(3.W))
    val vMaskud = Cat (io.maskValid, io.release)
    when (vMaskud === 1.U) {
      vMaskcount := vMaskcount - 1.U
    }.elsewhen (vMaskud === 2.U) {
      vMaskcount := vMaskcount + 1.U 
    }
  val hasMask = WireInit(false.B)
    hasMask := vMaskcount =/= 0.U
  assert (vMaskcount < (Depth+1).U)
  
  /*
     CurrentEntry and Current Mask
  */

  val currentEntry = buffer(readPtr)
  val currentMask = WireInit (true.B)
  // current mask is 1 bit for now, will change for V1  
  currentMask := Mux(isIndex, currentEntry(64), currentEntry(currentMaskIndex))
  io.currentMaskOut := currentMask

  /*
     Calculate Index Address
  */

  val indexAddr = WireInit(0.U(64.W))
  indexAddr := currentAddr + currentEntry(63, 0)
  io.outAddr := Mux(isIndex, indexAddr(39, 0), currentAddr(39, 0))  
  

    
  // popForce will happen when hasMask, masked-off, not last
  io.popForce := working && !currentMask && isMask && hasMask && !io.last 
  // this happens when the last element is masked-off, still need to send something
  val lastFake = working && !currentMask && isMask  && hasMask && io.last 
  io.popForceLast := lastFake
  // can Pop happens when lastFake, !isMask && !isIndex, !isMask && isIndex && hasMask, isMask && hasMask && currentMask, fakeHold
  io.canPop := working && ((currentMask && isMask && hasMask) || (!isMask && !isIndex) || (!isMask && isIndex && hasMask) || lastFake || fakeHold)
//  io.canPop := working && ((currentMask && ((isMask || isIndex) && hasMask)) || (!isMask && !isIndex) || lastFake)
  val isLastIndex = currentEntry(65)
  
  // fake happens when: no mask but vl = 0, with mask but last one masked off
  io.isFake := fakeHold || lastFake 
  // last one happens either currentIndex touch vl or isIndex && isLastIndex
  io.last := ((currentIndex === vlHold) || (isIndex && isLastIndex)) && working
  
  io.release := false.B 
  when (isIndex) {
      // index increase readPtr anyway, last or not
       when(io.pop || io.popForce) {
        readPtr := WrapInc(readPtr, Depth)
        io.release := true.B 
        currentIndex := currentIndex + 1.U
        // turn off working
       // precaution for vl = 0 index load store
         when(io.last) {
           fakeHold := false.B 
           working := false.B 
        }
       }
       
    }.otherwise{
     when (io.pop || io.popForce) {
       when (io.last) {
          fakeHold := false.B
          working := false.B 
          currentIndex := 0.U
          // pop off the current Mask regardless
          when (isMask) {
            readPtr := WrapInc(readPtr, Depth)
            currentMaskIndex := 0.U
            io.release := true.B 
           }
          
       }.otherwise {
        // pop off current mask if it reaches end
         when (currentMaskIndex === 63.U && isMask) {
          readPtr := WrapInc(readPtr, Depth)
          currentMaskIndex := 0.U
          io.release := true.B 
         }.elsewhen(isMask) {
          currentMaskIndex := currentMaskIndex + 1.U
         }
         currentIndex := currentIndex + 1.U 
         when (isStride) {
            currentAddr := currentAddr + stride 
         }.otherwise { 
            currentAddr := currentAddr + io.sliceSizeOut 
         }    
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

  when (io.configValid) {
    currentID := io.startID
    currentVD := io.startVD
    count := 0.U  // for now
  }
  io.outID := currentID
  io.outVD := currentVD
  when (io.pop) {
    when (count + io.sliceSize === M.U) {
      count := 0.U
      currentVD := currentVD + 1.U
      currentID := 0.U
    }.otherwise{
      count := count + io.sliceSize
      currentID := currentID + 1.U 
    }
  }

}  


class VDB(val M: Int, val N: Int, val Vlen: Int, val Depth: Int)(implicit p: Parameters) extends Module {
  require(isPow2(M), "M must be a power of 2")
  require(isPow2(N), "N must be a power of 2")
  require(isPow2(Vlen), "Vlen must be a power of 2")
  require(M >= N, "M must be greater than or equal to N")
  require(M % 8 == 0, "M must be a multiple of 8")
  require(N % 8 == 0, "N must be a multiple of 8")
  val S = log2Ceil(N / 8 + 1)
  val I = log2Ceil(M / 8 + 1)
  val maxIndex = (M / 8)
  val safeLmul = log2Ceil(M/Vlen)  // 2 in our case, however this needs to be logged (LMUL = 2, but vlmul = 1)
  val preshift = log2Ceil(M/Vlen) // 1 in our case

  val io = IO(new Bundle {
    val configValid = Input(Bool())
    val writeValid = Input(Bool())
    val writeData = Input(UInt(M.W))
    val pop = Input(Bool())
    val last = Input(Bool())
    val vlmul = Input(UInt(3.W))
    val sliceSize = Input(UInt(S.W))
    val release = Output(Bool())
    val outData = Output(UInt(N.W))
  })

  val buffer = RegInit(VecInit(Seq.fill(Depth)(0.U(M.W))))
  val readPtr = RegInit(0.U(log2Ceil(Depth).W))
  val writePtr = RegInit(0.U(log2Ceil(Depth).W))
  val finalJump = RegInit(0.U((log2Ceil(Depth+1)).W))
  val currentIndex = Reg(UInt(I.W))
  val needJump = RegInit(false.B)
  val jumping = RegInit(false.B)
  val miniIndex = RegInit(0.U(log2Ceil(N+1).W))

  when(io.configValid) {
   currentIndex := 0.U
   miniIndex := 0.U
   needJump := false.B 
  }

  val currentEntry = buffer(readPtr)

  when(io.writeValid) {
    buffer(writePtr) := io.writeData
    writePtr := WrapInc(writePtr, Depth)
  }

 val paddedEntry = Cat(0.U((N-8).W), currentEntry)
 val slices = VecInit(Seq.tabulate(M/8)(i => paddedEntry(i*8+N-1, i*8)))
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
        }.otherwise {
          currentIndex := currentIndex + io.sliceSize
        }
    }
  }
}

class VWhLSDecoder(val M: Int) extends Module {
  val Mbyte = M/8
  val io = IO(new Bundle {
    val nf = Input(UInt(3.W))
    val wth = Input(UInt(3.W))
    val overVl = Output(UInt(9.W))
    val overVlmul = Output(UInt(3.W))
  })

  val nf_wth = Cat(io.nf, io.wth)

  io.overVl := MuxLookup(nf_wth, 0.U, Array(
    0.U  -> (Mbyte.U),
    5.U  -> (Mbyte.U >> 1),
    6.U  -> (Mbyte.U >> 2),
    7.U  -> (Mbyte.U >> 3),
    8.U  -> (Mbyte.U << 1),
    13.U -> (Mbyte.U),
    14.U -> (Mbyte.U >> 1),
    15.U -> (Mbyte.U >> 2),
    24.U -> (Mbyte.U << 2),
    29.U -> (Mbyte.U << 1),
    30.U -> (Mbyte.U),
    31.U -> (Mbyte.U >> 1),
    56.U -> (Mbyte.U << 3),
    61.U -> (Mbyte.U << 2),
    62.U -> (Mbyte.U << 1),
    63.U -> (Mbyte.U)
  ))
  io.overVlmul := 0.U
  switch(io.nf) {
    is (0.U) { io.overVlmul := 0.U }
    is (1.U) { io.overVlmul := 1.U }
    is (3.U) { io.overVlmul := 2.U }
    is (7.U) { io.overVlmul := 3.U }
  }
}


// M is the return bus width: 512 for now, N is the DMEM width: 64 for now
class VReturnData (val M: Int, val N: Int) (implicit p: Parameters) extends Module {
    val io = IO(new Bundle {
    val lsuData = Input(UInt(N.W))
    val oviData = Output(UInt(M.W))
    // 0 for pos, 1 for neg
    val strideDir     = Input (Bool())
    val memSize = Input (UInt(2.W))
  })
    io.oviData := 0.U 
    when (io.strideDir){  // negative
       when (io.memSize === 0.U) {
        io.oviData := Cat(io.lsuData (7, 0), 0.U(504.W)) 
       }.elsewhen (io.memSize === 1.U) {
        io.oviData := Cat(io.lsuData (15, 0), 0.U(496.W))       
       }.elsewhen (io.memSize === 2.U) {
        io.oviData := Cat(io.lsuData (31, 0), 0.U(480.W))
       }.elsewhen (io.memSize === 3.U) {
        io.oviData := Cat(io.lsuData (63, 0), 0.U(448.W))
       }
    }.otherwise {
      when (io.memSize === 0.U) {
        io.oviData := Cat(0.U, io.lsuData (7, 0)) 
       }.elsewhen (io.memSize === 1.U) {
        io.oviData := Cat(0.U, io.lsuData (15, 0))       
       }.elsewhen (io.memSize === 2.U) {
        io.oviData := Cat(0.U, io.lsuData (31, 0))
       }.elsewhen (io.memSize === 3.U) {
        io.oviData := Cat(0.U, io.lsuData (63, 0))
       }
    }
}

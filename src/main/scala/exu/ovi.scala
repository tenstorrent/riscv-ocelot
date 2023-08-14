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
  when(reqQueue.io.deq.valid && reqQueue.io.deq.ready) { uOpMem.write(sbId, reqQueue.io.deq.bits.req.uop) }
  val respUop = uOpMem.read(vpuModule.io.completed_sb_id)

  io.resp.valid := vpuModule.io.completed_valid
  io.resp.bits.data := vpuModule.io.completed_dest_reg
  io.resp.bits.uop := respUop
  io.resp.bits.uop.dst_rtype := Mux(respUop.dst_rtype === RT_VEC,
                                    RT_X,
                                    respUop.dst_rtype)
  io.resp.bits.uop.uses_stq := 0.B // Trick Rob to acknowledge Vector Store
  io.resp.bits.fflags.valid := vpuModule.io.completed_valid && vpuModule.io.completed_fflags.orR
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
   val outStandingLSCount = 32
   val vpuVlen = 256
   val vdbDepth = 4
   val vAGenDepth = 4
   val fakeLoadDepth = 8

   val lsuDmemWidth = coreDataBits
   val byteVreg = vpuVlen / 8
   val byteDmem = lsuDmemWidth / 8
   val addrBreak = log2Ceil(lsuDmemWidth/8)
   

/*
  vLSIQ start
*/

  // in the middle of handling vector load store
  val inMiddle = RegInit(false.B)
  // trying to dequeue VLSIQ 
  val tryDeqVLSIQ = RegInit(false.B)
  // this chunk is checking the number of outstanding mem_sync_start
  val outStandingReq = RegInit(0.U(log2Ceil(outStandingLSCount).W))
  val canStartAnother = WireInit(false.B)
  val vOSud = Cat (vpuModule.io.memop_sync_start, canStartAnother)
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
  }.elsewhen (canStartAnother) {
    inMiddle := false.B
  }
  reqQueue.io.deq.ready := issueCreditCnt =/= 0.U && vLSIQueue.io.enq.ready
  // a new set has dequeued from vLSIQueue
  val newVGenConfig = vLSIQueue.io.deq.valid && vLSIQueue.io.deq.ready 


/*
   v-Helper Start
*/


  val vAGen = Module (new VAgen (lsuDmemWidth, 66, vAGenDepth, vpuVlen, oviWidth))

  vAGen.io.configValid := false.B 
  vAGen.io.maskData := MemMaskId  
  vAGen.io.maskValid := MemMaskValid  
  vAGen.io.startAddr := vLSIQueue.io.deq.bits.req.rs1_data 
  vAGen.io.stride := vLSIQueue.io.deq.bits.req.rs2_data
  vAGen.io.isUnit := false.B
  vAGen.io.isStride := false.B 
  vAGen.io.isIndex := false.B 
  vAGen.io.isMask := false.B 
  vAGen.io.isLoad := false.B 
  vAGen.io.vl := vLSIQueue.io.deq.bits.vconfig.vl
  vAGen.io.pop := false.B  
  vAGen.io.initialSliceSize := 0.U
  vAGen.io.memSize := 0.U
  MemMaskCredit := vAGen.io.release
// VDB stuff
  val vdb = Module (new VDB(oviWidth, lsuDmemWidth, vpuVlen, vdbDepth))
  
  vdb.io.writeValid := false.B 
  vdb.io.pop := false.B 
  vdb.io.last := false.B  
  vdb.io.configValid := false.B  

  vdb.io.writeValid := MemStoreValid
  vdb.io.writeData := MemStoreData
  vdb.io.sliceSize := vAGen.io.sliceSizeOut 
  vdb.io.packOveride := vAGen.io.spackOveride
  vdb.io.packSkipVDB := vAGen.io.packSkipVDB
  vdb.io.packId := vAGen.io.packVDBId 
  MemStoreCredit := vdb.io.release

  
  // make sure that we have enough data
  val vDBcount = RegInit(0.U(log2Ceil(vdbDepth+1).W))
  val vDBud = Cat (MemStoreValid, MemStoreCredit)
  when (vDBud === 1.U) {
    vDBcount := vDBcount - 1.U
  }.elsewhen (vDBud === 2.U) {
    vDBcount := vDBcount + 1.U 
  }
  assert (vDBcount <= (vdbDepth).U)

   

val vIdGen = Module (new VIdGen(byteVreg, byteDmem))
 vIdGen.io.configValid := false.B 
 vIdGen.io.startID := 0.U 
 vIdGen.io.startVD := 0.U  
 vIdGen.io.pop := false.B
 vIdGen.io.sliceSize := vAGen.io.sliceSizeOut
 vIdGen.io.packOveride := vAGen.io.packOveride
 vIdGen.io.packSkipVreg := vAGen.io.packSkipVreg
 vIdGen.io.packId := vAGen.io.packId 


// extra decoder for whole register / mask
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
  val isUnit =                                             instMop === 0.U 

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
    vAGen.io.isLoad := vLSIQueue.io.deq.bits.req.uop.uses_ldq
    // special case of whole load and whole store
    when (isWholeLoad || isWholeStore) {
    vwhls.io.nf := instNf
    vwhls.io.wth := instElemSize 
    vAGen.io.vl := vwhls.io.overVl 
    }.elsewhen (isStoreMask || isLoadMask) {
      vAGen.io.vl := (vLSIQueue.io.deq.bits.vconfig.vl + 7.U) >> 3
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
      vAGen.io.isUnit := true.B 
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

  val fakeLoadReturnQueue = Module(new Queue(UInt(34.W), fakeLoadDepth))
  fakeLoadReturnQueue.io.enq.valid := s0l1 && (vAGen.io.popForce || vAGen.io.popForceLast)
  fakeLoadReturnQueue.io.enq.bits := Cat(sbIdHold, vAGen.io.elemCount, vAGen.io.elemOffset, 0.U((11 - log2Ceil(byteVreg + 1)).W), vIdGen.io.outID, vIdGen.io.outVD)
  fakeLoadReturnQueue.io.deq.ready := false.B 





  /*
     Output to LSU
  */ 

  io.core.vGenIO.req.valid := vGenEnable && ((!s0l1 && ((!vlIsZero && vDBcount =/= 0.U) || vlIsZero)) || s0l1) && vAGen.io.canPop
  io.core.vGenIO.req.bits.uop := vGenHold.req.uop
  io.core.vGenIO.req.bits.uop.mem_size := Mux(s0l1, addrBreak.U, vAGen.io.memSizeOut)
  io.core.vGenIO.req.bits.data := Mux(s0l1, 0.U, vdb.io.outData) 
  io.core.vGenIO.req.bits.last := vAGen.io.last 
  io.core.vGenIO.req.bits.addr := Mux(s0l1, Cat(vAGen.io.outAddr(39, addrBreak), 0.U(addrBreak.W)), vAGen.io.outAddr)

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
      canStartAnother := true.B 
    }
  }



/*
   Parse LSU response
*/
  val LSUReturnLoadValid = WireInit(false.B)
  LSUReturnLoadValid := io.core.vGenIO.resp.valid && io.core.vGenIO.resp.bits.s0l1  // needs fixing later if we are overlapping
  val LSUReturnData = WireInit(0.U(oviWidth.W))
  when (io.core.vGenIO.resp.bits.strideDir){  // negative
       LSUReturnData := Cat(io.core.vGenIO.resp.bits.data ((lsuDmemWidth-1), 0), 0.U((oviWidth-lsuDmemWidth).W))
    }.otherwise {
      LSUReturnData := Cat(0.U, io.core.vGenIO.resp.bits.data ((lsuDmemWidth-1), 0))
    }

/*
   Data back to VPU
*/

MemSbId := io.core.vGenIO.resp.bits.sbIdDone 
val MemSb = RegInit(0.U(32.W))
val vectorDone = WireInit(0.U(2.W))
val MemSbResidue = WireInit(false.B)
vectorDone := Cat (io.core.vGenIO.resp.bits.vectorDoneLd, io.core.vGenIO.resp.bits.vectorDoneSt)
when (vectorDone === 1.U) {
   MemSbId := io.core.vGenIO.resp.bits.sbIdDoneSt  
}.elsewhen (vectorDone === 2.U) {
   MemSbId := io.core.vGenIO.resp.bits.sbIdDoneLd 
}.elsewhen (vectorDone === 3.U) {
   MemSbId := io.core.vGenIO.resp.bits.sbIdDoneLd
   MemSb := MemSb.bitSet (io.core.vGenIO.resp.bits.sbIdDoneSt, true.B)
}.elsewhen (MemSb =/= 0.U) {
   MemSbResidue := true.B 
   MemSbId := PriorityEncoder (MemSb)
   MemSb := MemSb.bitSet (MemSbId, false.B)
//   MemSb(MemSbId) := 0.U 
}
MemSyncEnd := (io.core.vGenIO.resp.bits.vectorDone && io.core.vGenIO.resp.valid) || MemSbResidue

  
//  MemSyncEnd := io.vGenIO.resp.bits.vectorDone && io.vGenIO.resp.valid && inMiddle
  
  MemLoadValid := LSUReturnLoadValid || fakeLoadReturnQueue.io.deq.valid
  MemSeqId := Cat (seqSbId, seqElCount, seqElOff, seqElId, seqVreg) 

  when (LSUReturnLoadValid) {
    MemLoadData := LSUReturnData    
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
   vpuModule.io.memop_sb_id := MemSbId  
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
    val memop_sb_id = Input(UInt(5.W))
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
  addResource("/vsrc/vpu/tt_store_fsm.sv")
  addResource("/vsrc/vpu/tt_scoreboard_ovi.sv") 
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

// M is dmem bandwidth, N is mask interface width (66), Depth is mask buffer width (4), VLEN is 256 for now
class VAgen(val M: Int, val N: Int, val Depth: Int, val VLEN: Int, val OVILEN: Int)(implicit p: Parameters) extends Module {
    val k = log2Ceil(M/8+1)
    val I = log2Ceil(VLEN)
    val T = log2Ceil(OVILEN + 1)
    val numByte = (M/8)
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
    val memSizeOut = Output(UInt(3.W))
      // which types of load store
    val isUnit = Input (Bool())
    val isStride = Input(Bool())
    val isMask = Input(Bool())
    val isIndex = Input(Bool())
    val isLoad = Input (Bool())
      // base address and stride    
    val startAddr = Input(UInt(64.W))
    val stride = Input(UInt(64.W))
    // interface with OVI Vhelper
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
    // interface with VIdGen
    val packOveride = Output (Bool())
    val spackOveride = Output (Bool())
    val packId = Output (UInt(I.W))
    val packVDBId = Output (UInt(T.W))
    val packSkipVreg = Output (Bool()) 
    val packSkipVDB = Output (Bool())   
  })

  

  val vPacker = Module (new Vpacker (M, VLEN))

  vPacker.io.configValid := io.configValid     
  vPacker.io.vl := io.vl
  vPacker.io.memSize := io.memSize
  vPacker.io.isUnit := io.isUnit
  vPacker.io.isStride := io.isStride
  vPacker.io.isMask := io.isMask
  vPacker.io.isLoad := io.isLoad 
  vPacker.io.startAddr := io.startAddr
  vPacker.io.stride := io.stride 
  vPacker.io.pop := io.pop 
    
  io.packOveride := vPacker.io.packOveride
  io.packId := vPacker.io.packId 
  io.packSkipVreg := vPacker.io.packSkipVreg   


  val sPacker = Module (new Spacker (M, OVILEN))  
  sPacker.io.configValid := io.configValid     
  sPacker.io.vl := io.vl
  sPacker.io.memSize := io.memSize
  sPacker.io.isUnit := io.isUnit
  sPacker.io.isMask := io.isMask
  sPacker.io.isLoad := io.isLoad 
  sPacker.io.startAddr := io.startAddr
  
  sPacker.io.pop := io.pop 
  io.spackOveride := sPacker.io.packOveride
  io.packVDBId := sPacker.io.packVDBId 
  io.packSkipVDB := sPacker.io.packSkipVDB
  
    
  
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
  val isUnit = Reg(Bool())
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
  val internalElemOffset = WireInit(0.U(6.W))
  internalElemOffset := Mux(isNegStride, (negativeStrideElemOffset - positiveStrideElemOffset), positiveStrideElemOffset)
  io.elemOffset := Mux(vPacker.io.packOveride, vPacker.io.elemOffset, internalElemOffset)
  io.elemCount := Mux(vPacker.io.packOveride, vPacker.io.elemCount, 1.U) 
  
  io.memSizeOut := Mux (io.spackOveride, sPacker.io.memSizeOut, memSizeHold)


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
     isUnit := io.isUnit 
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
  val vMaskcount = RegInit(0.U(log2Ceil(Depth+1).W))
  val vMaskud = Cat (io.maskValid, io.release)
  when (vMaskud === 1.U) {
    vMaskcount := vMaskcount - 1.U
  }.elsewhen (vMaskud === 2.U) {
    vMaskcount := vMaskcount + 1.U 
  }
  val hasMask = WireInit(false.B)
  hasMask := vMaskcount =/= 0.U
  assert (vMaskcount <= Depth.U)
  
  /*
     CurrentEntry and Current Mask
  */

  val currentEntry = buffer(readPtr)
  val currentMask = WireInit (true.B)
  // current mask is 1 bit for now, will change for V1  
  currentMask := Mux(isIndex, currentEntry(64), currentEntry(currentMaskIndex))
  io.currentMaskOut := Mux(vPacker.io.packOveride, currentEntry((VLEN/8 - 1), 0), currentMask)

  val vPackMask = Module (new VPackMask (VLEN))
  vPackMask.io.elemCount := io.elemCount
  vPackMask.io.currentMask := currentEntry((VLEN/8 - 1), 0)


  /*
     Calculate Index Address
  */

  val indexAddr = WireInit(0.U(64.W))
  indexAddr := currentAddr + currentEntry(63, 0)
  io.outAddr := Mux(isIndex, indexAddr(39, 0), currentAddr(39, 0))  
  

    
  // popForce will happen when hasMask, masked-off, not last
  io.popForce := working && !io.last && isMask && hasMask && ((!vPacker.io.packOveride && !currentMask) || (vPacker.io.packOveride && !vPackMask.io.validMask)) 
  // this happens when the last element is masked-off, still need to send something
  val lastFake = working && io.last && isMask  && hasMask && ((!vPacker.io.packOveride && !currentMask) || (vPacker.io.packOveride && !vPackMask.io.validMask)) 
  io.popForceLast := lastFake
  // can Pop happens when lastFake, !isMask && !isIndex, !isMask && isIndex && hasMask, isMask && hasMask && currentMask, fakeHold
  io.canPop := working && (((!vPacker.io.packOveride || sPacker.io.packOveride) && ((currentMask && isMask && hasMask) || (!isMask && !isIndex) || (!isMask && isIndex && hasMask))) || 
                            lastFake || fakeHold ||
                            (vPacker.io.packOveride && (!isMask || (hasMask && isMask && vPackMask.io.validMask))))
  val isLastIndex = currentEntry(65)
  
  // fake happens when: no mask but vl = 0, with mask but last one masked off
  io.isFake := fakeHold || lastFake 
  // last one happens either currentIndex touch vl or isIndex && isLastIndex
  io.last := ((!vPacker.io.packOveride && ((currentIndex === vlHold) || (isIndex && isLastIndex))) || (vPacker.io.packOveride && vPacker.io.packLast) 
                                                                                                   || (sPacker.io.packOveride && sPacker.io.packLast)) && working
  
  io.release := false.B 
  when (isIndex) {
      // index increase readPtr anyway, last or not
    when(io.pop || io.popForce) {
      when (!fakeHold) {
        readPtr := WrapInc(readPtr, Depth)
        io.release := true.B 
        currentIndex := currentIndex + 1.U
      }
      // turn off working, precaution for vl = 0 index load store
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
          when (isMask && !fakeHold) {
            readPtr := WrapInc(readPtr, Depth)
            currentMaskIndex := 0.U
            io.release := true.B 
           }          
       }.otherwise {
        // pop off current mask if it reaches end
          when (vPacker.io.packOveride) {
            when (vPacker.io.packSkipVMask) {
              readPtr := WrapInc(readPtr, Depth)
              io.release := true.B 
            }.otherwise {
              buffer(readPtr) := buffer(readPtr) >> io.elemCount 
            }
          }.otherwise {when (currentMaskIndex === 63.U && isMask) {
            readPtr := WrapInc(readPtr, Depth)
            currentMaskIndex := 0.U
            io.release := true.B 
          }.elsewhen(isMask) {
            currentMaskIndex := currentMaskIndex + 1.U
          }
        }
          currentIndex := currentIndex + 1.U 
          when (vPacker.io.packOveride){
            when (vPacker.io.packIncrement) {
              when(vPacker.io.packDir) {
                currentAddr := currentAddr - numByte.U 
              }.otherwise {
                currentAddr := currentAddr + numByte.U 
              }
            }
          }.elsewhen(sPacker.io.packOveride){
            currentAddr := currentAddr + sPacker.io.packIncrement
          }.otherwise {
            when (isStride) {
              currentAddr := currentAddr + stride 
            }.elsewhen(isUnit) { 
              currentAddr := currentAddr + io.sliceSizeOut 
            }    
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
  

  val io = IO(new Bundle {
    val configValid = Input(Bool())
    val startID = Input(UInt(S.W))
    val startVD = Input(UInt(5.W))
    val pop = Input(Bool())    
    val outID = Output(UInt(S.W))
    val outVD = Output(UInt(5.W))
    // interface with VAGen
    val sliceSize = Input(UInt(S.W))
    val packOveride = Input (Bool())
    val packId = Input (UInt(S.W))
    val packSkipVreg = Input (Bool()) 
  }) 

  val currentID = RegInit(0.U(S.W))
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
    when (io.packOveride) {
      when (io.packSkipVreg) {
        currentID := 0.U 
        currentVD := currentVD + 1.U
      }.otherwise {
        currentID := currentID + io.packId
      }

    }.otherwise{
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

  val io = IO(new Bundle {
    val configValid = Input(Bool())
    val writeValid = Input(Bool())
    val writeData = Input(UInt(M.W))
    val pop = Input(Bool())
    val last = Input(Bool())
//    val vlmul = Input(UInt(3.W))
    val sliceSize = Input(UInt(S.W))
    val release = Output(Bool())
    val outData = Output(UInt(N.W))
    val packOveride = Input (Bool())
    val packId = Input (UInt(I.W))
    val packSkipVDB = Input (Bool())
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
 val slices = VecInit(Seq.tabulate(M/8)(i => paddedEntry(i*8+N-1, i*8)))
 io.outData := slices(currentIndex) 
 
  io.release := false.B 
  when (io.pop) {
    when (io.packOveride) {
        when (io.packSkipVDB || io.last) {
          readPtr := WrapInc(readPtr, Depth)
          currentIndex := 0.U
          io.release := true.B
        }.otherwise {
          currentIndex := currentIndex + io.packId
        }
    }.otherwise {
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


class Vpacker(val M: Int, val VLEN: Int) extends Module {
   val k = log2Ceil(M/8+1)
    val I = log2Ceil(VLEN)
    val MByte = M/8
    val VLENByte = VLEN/8
  val io = IO(new Bundle {    
    // interface with vAGen at start
    val configValid = Input(Bool())
    val vl = Input(UInt(9.W))
    val memSize = Input(UInt(2.W))
      // which types of load store
    val isUnit = Input (Bool())
    val isStride = Input(Bool())
    val isMask = Input(Bool())
    val isLoad = Input(Bool())
      // base address and stride    
    val startAddr = Input(UInt(64.W))
    val stride = Input(UInt(64.W))
    // interface with OVI Vhelper
    val elemOffset = Output(UInt(6.W))
    val elemCount = Output(UInt(7.W))
     // interface with VIdGen / VAGen
    val packOveride = Output (Bool())
    val packId = Output (UInt(I.W))
    val packSkipVreg = Output (Bool())
    val packSkipVMask = Output (Bool())  
    val packIncrement = Output (Bool())
    val packLast = Output (Bool())
    val packDir = Output (Bool())
     // pop 
    val pop = Input (Bool())
  })

    
  // decode the stride to see if we can do packing
    val canPack = WireInit (false.B)
    val strideDetector = Module (new StrideDetector())
    strideDetector.io.mem_size := io.memSize
    strideDetector.io.stride := io.stride
    val isZero = WireInit(false.B)
    val isOne = WireInit(false.B)
    val isTwo = WireInit(false.B)
    val isFour = WireInit(false.B)
    val isMask = RegInit(false.B)
    val logStride = WireInit(3.U(2.W)) // this is the log2 of stride
    isZero := strideDetector.io.isZero
    isOne := strideDetector.io.isOne
    isTwo := strideDetector.io.isTwo
    isFour := strideDetector.io.isFour
    logStride := strideDetector.io.logStride 
    canPack := io.configValid && (io.isUnit || (io.isStride && (isOne || isTwo || isFour))) && io.isLoad && (io.vl =/= 0.U)
    

  // initializing element offset and logStride
    val currentOffset = RegInit(0.U(6.W)) // offset, marking the position in cache line
    val currentLogStride = RegInit(0.U(2.W))  // logStride, probably don't need to store
    val currentLogStrideTemp = WireInit(0.U(2.W))
    val currentMax = RegInit(0.U(6.W)) // max element count for now (for each transaction)
    val currentVMax = RegInit(0.U(6.W)) // max element count per vReg
    val currentDir = RegInit(false.B) // direction, might be useful
    val currentIndex = RegInit(0.U(9.W)) // the index of element (as a whole in the vector)
    val currentVIndex = RegInit(0.U(6.W))
    val currentMIndex = RegInit(0.U(7.W))
    val vlHold = RegInit(0.U(9.W))
    val isPacking = RegInit(false.B)  // drive override
    
    io.packOveride := isPacking
  //  io.packOveride := false.B 
    io.packDir := currentDir

    // calculating initial element offset
    val internalConfigValid = WireInit (false.B)
    val initialNegativeStrideElemOffset = WireInit(0.U(k.W))
    val initialPositiveStrideElemOffset = WireInit(0.U(k.W))

    when (canPack) {
       initialNegativeStrideElemOffset := k.U - io.memSize - 1.U 
       initialPositiveStrideElemOffset := io.startAddr(k-2, 0) >> io.memSize       
       
       currentOffset := Mux ((io.isStride && io.stride(63)), (initialNegativeStrideElemOffset - initialPositiveStrideElemOffset), initialPositiveStrideElemOffset)
       currentLogStride := Mux(io.isUnit, 0.U(2.W), logStride)
       currentLogStrideTemp := Mux(io.isUnit, 0.U(2.W), logStride)
       currentDir := io.isStride && io.stride(63)
       currentMax := MByte.U >> (io.memSize + currentLogStrideTemp)
       currentVMax := VLENByte.U >> (io.memSize)
       currentIndex := 0.U 
       currentVIndex := 0.U 
       currentMIndex := 0.U
       vlHold := io.vl 
       isPacking := true.B 
       isMask := io.isMask
    }

    // calculation of offset and count
    io.elemOffset := currentOffset
    val theoreticalCount = WireInit(0.U(6.W)) //max count in this transaction if ignore vl and vReg boundary
    theoreticalCount := currentMax - (currentOffset >> currentLogStride)
    val vRegDistant = WireInit(0.U(6.W))
    vRegDistant := currentVMax - currentVIndex
    val vlDistant = WireInit(0.U(9.W))
    vlDistant := vlHold - currentIndex 
    val afterVRegCount = WireInit(0.U(6.W)) // we consider VReg first
    afterVRegCount := Mux((theoreticalCount > vRegDistant), vRegDistant, theoreticalCount)
    val afterVLCount = WireInit(0.U(6.W))
    afterVLCount := Mux((afterVRegCount > vlDistant), vlDistant, afterVRegCount)

    io.packId := afterVLCount 
    io.elemCount := afterVLCount 
    io.packSkipVreg := (afterVLCount === vRegDistant) && isPacking
    io.packLast := (afterVLCount === vlDistant) && isPacking
    io.packIncrement := ((currentOffset >> currentLogStride) + io.elemCount === currentMax) && isPacking
    io.packSkipVMask := ((currentMIndex + io.elemCount) === 64.U) && isMask && isPacking 
    when(io.pop) {
      when(io.packLast) {
        isPacking := false.B 
      }
      currentIndex := currentIndex + io.elemCount
      when (io.packSkipVreg) {
        currentVIndex := 0.U 
      }.otherwise {
        currentVIndex := currentVIndex + io.elemCount 
      }
      when (io.packSkipVMask) {
        currentMIndex := 0.U 
      }.elsewhen(isMask) {
        currentMIndex := currentMIndex + io.elemCount 
      }
      when ((currentOffset >> currentLogStride) + io.elemCount === currentMax) {
        currentOffset := 0.U 
      }.otherwise {
        currentOffset := currentOffset + (io.elemCount << currentLogStride)
      }
    }
}


class StrideDetector extends Module {
  val io = IO(new Bundle {
    val mem_size = Input(UInt(2.W))
    val stride = Input(UInt(64.W))
    val isZero = Output(Bool())
    val isOne = Output(Bool())
    val isTwo = Output(Bool())
    val isFour = Output(Bool())
    val logStride = Output(UInt(2.W))
  })

  io.isZero := io.stride === 0.U

  io.isOne := (io.mem_size === 0.U && (io.stride === 1.U || io.stride === "hFFFFFFFFFFFFFFFF".U)) ||
              (io.mem_size === 1.U && (io.stride === 2.U || io.stride === "hFFFFFFFFFFFFFFFE".U)) ||
              (io.mem_size === 2.U && (io.stride === 4.U || io.stride === "hFFFFFFFFFFFFFFFC".U)) ||
              (io.mem_size === 3.U && (io.stride === 8.U || io.stride === "hFFFFFFFFFFFFFFF8".U))

  io.isTwo := (io.mem_size === 0.U && (io.stride === 2.U || io.stride === "hFFFFFFFFFFFFFFFE".U)) ||
              (io.mem_size === 1.U && (io.stride === 4.U || io.stride === "hFFFFFFFFFFFFFFFC".U)) ||
              (io.mem_size === 2.U && (io.stride === 8.U || io.stride === "hFFFFFFFFFFFFFFF8".U))

  io.isFour := (io.mem_size === 0.U && (io.stride === 4.U || io.stride === "hFFFFFFFFFFFFFFFC".U))
  io.logStride := 3.U 
  when (io.isTwo) {
    io.logStride := 1.U 
  }.elsewhen(io.isFour){
    io.logStride := 2.U 
  }.elsewhen(io.isOne){
    io.logStride := 0.U 
  }
}

class VPackMask (val VLEN: Int) extends Module {
  val EffMaskLength = VLEN / 8
  val io = IO (new Bundle{
    val elemCount = Input(UInt(7.W))
    val currentMask = Input (UInt(EffMaskLength.W))
    val validMask = Output (Bool())
  })
  val internalMask = WireInit(0.U(EffMaskLength.W))
  internalMask := (1.U << io.elemCount) - 1.U 
  io.validMask := (io.currentMask & internalMask).orR
}



class Spacker(val M: Int, val VDBLEN: Int) extends Module {
   val k = log2Ceil(M/8+1)
    val I = log2Ceil(VDBLEN + 1)
    val MByte = M/8
    val VDBLENByte = VDBLEN/8
  val io = IO(new Bundle {    
    // interface with vAGen at start
    val configValid = Input(Bool())
    val vl = Input(UInt(9.W))
    val memSize = Input(UInt(2.W))
    val memSizeOut = Output(UInt(3.W))
      // which types of load store
    val isUnit = Input (Bool())
    val isMask = Input(Bool())
    val isLoad = Input(Bool())
      // base address and stride    
    val startAddr = Input(UInt(64.W))
      // interface with OVI Vhelper
  //  val elemOffset = Output(UInt(6.W))
  //  val elemCount = Output(UInt(7.W))
     // interface with VIdGen / VAGen
    val packOveride = Output (Bool())
    val packVDBId = Output (UInt(I.W))
    val packSkipVDB = Output (Bool())  
    val packIncrement = Output (UInt(64.W))
    val packLast = Output (Bool())
     // pop 
    val pop = Input (Bool())
  })

   val canPack = WireInit (false.B)
   canPack := io.configValid && io.isUnit && !io.isMask && !io.isLoad && (io.vl =/= 0.U)

   val currentOffset = RegInit(0.U(6.W)) // offset, marking the position in cache line
   val currentMax = RegInit(0.U(6.W)) // max element count for now (for each transaction)
   val currentVMax = RegInit(0.U(7.W)) // max element count per vReg
   val currentIndex = RegInit(0.U(9.W)) // the index of element (as a whole in the vector)
   val currentVIndex = RegInit(0.U(7.W))
   val vlHold = RegInit(0.U(9.W))
   val memSize = RegInit(0.U(2.W))
   val isPacking = RegInit(false.B)  // drive override


   io.packOveride := isPacking
  // io.packOveride := false.B 

    when (canPack) {
       currentOffset := io.startAddr(k-2, 0) >> io.memSize          
       currentMax := MByte.U >> (io.memSize)
       currentVMax := VDBLENByte.U >> (io.memSize)
       currentIndex := 0.U 
       currentVIndex := 0.U        
       vlHold := io.vl 
       memSize := io.memSize
       isPacking := true.B        
    }

    // calculation of offset and count
  //  io.elemOffset := currentOffset
    val theoreticalCount = WireInit(0.U(6.W)) //max count in this transaction if ignore vl and vReg boundary
    theoreticalCount := currentMax - currentOffset
    val actualTheoreticalCount = WireInit(0.U(6.W)) //max count in this transaction if ignore vl and vReg boundary
    actualTheoreticalCount := PriorityEncoderOH (theoreticalCount)
    val vRegDistant = WireInit(0.U(7.W))
    vRegDistant := currentVMax - currentVIndex
    val actualVRegDistant = WireInit(0.U(7.W))
    val vRegPower2 = Module (new SmallPowerOfTwo(7))
    vRegPower2.io.inData := vRegDistant 
    actualVRegDistant := vRegPower2.io.outData 

    val vlDistant = WireInit(0.U(9.W))
    vlDistant := vlHold - currentIndex 
    val actualVlDistant = WireInit(0.U(9.W))
    val vlPower2 = Module (new SmallPowerOfTwo(9))
    vlPower2.io.inData := vlDistant 
    actualVlDistant := vlPower2.io.outData 
    
    
    val afterVRegCount = WireInit(0.U(6.W)) // we consider VReg first
    afterVRegCount := Mux((actualTheoreticalCount > actualVRegDistant), actualVRegDistant, actualTheoreticalCount)
    val afterVLCount = WireInit(0.U(6.W))
    afterVLCount := Mux((afterVRegCount > actualVlDistant), actualVlDistant, afterVRegCount)


    io.packVDBId := afterVLCount << memSize
  //  io.elemCount := afterVLCount 
    io.packSkipVDB := (afterVLCount === vRegDistant) && isPacking
    io.packLast := (afterVLCount === vlDistant) && isPacking
    io.packIncrement := afterVLCount << memSize 

    when(io.pop) {
      when(io.packLast) {
        isPacking := false.B 
      }
      currentIndex := currentIndex + afterVLCount
      when (io.packSkipVDB) {
        currentVIndex := 0.U 
      }.otherwise {
        currentVIndex := currentVIndex + afterVLCount
      }
      when (currentOffset  + afterVLCount === currentMax) {
        currentOffset := 0.U 
      }.otherwise {
        currentOffset := currentOffset + afterVLCount
      }
    }

    val countLog = WireInit(0.U(3.W))
    countLog := PriorityEncoder(afterVLCount)
    io.memSizeOut := memSize + countLog 


}




// this will return the smallest power of 2
// used for tracking VL distant and VReg distant in packed store

class SmallPowerOfTwo(bitWidth: Int) extends Module {
  val io = IO(new Bundle {
    val inData = Input(UInt(bitWidth.W))
    val outData = Output(UInt(bitWidth.W))
  })

  when(io.inData === 0.U) {
    io.outData := 0.U
  } .otherwise {
    when(PopCount(io.inData) === 1.U) {
      io.outData := io.inData
    } .otherwise {
      // Reverse the bits of the input and take the position of the most significant '1' bit.
    val reversedBits = Reverse(io.inData)
    val positionOfMSB = PriorityEncoder(reversedBits)
    
    // Shift 1 left by the position of the most significant '1' bit to get the closest power of 2.
    io.outData := 1.U << (bitWidth.U - 1.U - positionOfMSB)
    }
  }
}

// PriorityEncoderOH 


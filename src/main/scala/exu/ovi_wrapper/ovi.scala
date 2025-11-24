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
import boom.exu.OviScoreboard // moved the SB to a different file
import chisel3.dontTouch // this is for debugging purposes

class EnhancedFuncUnitReq(xLen: Int, vLen: Int)(implicit p: Parameters) extends Bundle {
  val vconfig = new VConfig()
  val vxrm = UInt(2.W)
  val fcsr_rm = UInt(3.W)
  val vstart = UInt(log2Ceil(vLen+1).W)
  val sb_id = UInt(5.W)
  val poison = Bool()
  val req = new FuncUnitReq(xLen)
}

class OviWrapper(implicit p: Parameters) extends BoomModule
with freechips.rocketchip.rocket.constants.MemoryOpConstants {

  // =============== IO Ports Definition ===============
  val io = IO(new Bundle {
    val req  = Flipped(new DecoupledIO(new FuncUnitReq(xLen)))
    val resp = new DecoupledIO(new FuncUnitResp(xLen))

    val vconfig = Input(new VConfig())
    val vxrm    = Input(UInt(2.W))
    val fcsr_rm = Input(UInt(3.W))
    val vstart  = Input(UInt(log2Ceil(vLen+1).W))

    val vGenIO = Flipped(new boom.lsu.VGenIO)

    val debug_wb_vec_valid = Output(Bool())
    val debug_wb_vec_wdata = Output(UInt((vLen * 8).W))
    val debug_wb_vec_wmask = Output(UInt(8.W))

    val core = new Bundle {
      val rob_pnr_idx  = Input(UInt(robAddrSz.W))
      val rob_head_idx = Input(UInt(robAddrSz.W))
      val brupdate     = Input(new BrUpdateInfo())
      val exception    = Input(Bool())
    }
  })

  // =============== Constants Definition ===============

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
  val MAX_ISSUE_CREDIT = 32
  val MAX_OUTSTANDING_VMEMOPS = 8 // max number of vector memory operations that can be outstanding

  io := DontCare
  val vpu = Module(new tt_vpu_ovi(vLen))
  val scoreboard = Module(new OviScoreboard(32))

  // == sb connections ==
  scoreboard.io.insert.bits := io.req.bits.uop // insert uop
  scoreboard.io.insert.valid := io.req.fire   // use fire instead of valid so that other readys are also taken into account
  val sb_ready = scoreboard.io.insert.ready // io.req.ready := <...> && sb_ready

  scoreboard.io.remove.idx := vpu.io.completed_sb_id // remove idx
  val resp_valid = vpu.io.completed_valid // remove valid
  scoreboard.io.remove.valid := resp_valid 
  val sb_remove_ready = scoreboard.io.remove.ready // remove ready (!empty)
  val resp_uop = scoreboard.io.remove.uop // output uop

  val next_sb_id = scoreboard.io.next_sb_id // next available sb id

  scoreboard.io.core.rob_pnr_idx := io.core.rob_pnr_idx
  scoreboard.io.core.rob_head_idx := io.core.rob_head_idx
  scoreboard.io.core.brupdate := io.core.brupdate
  scoreboard.io.core.exception := io.core.exception

  val dispatch_sb_id = scoreboard.io.dispatch.dispatch_sb_id
  val dispatch_next_senior = scoreboard.io.dispatch.dispatch_next_senior
  val dispatch_kill = scoreboard.io.dispatch.dispatch_kill

  // == vpu connections ==
  io.resp.valid := resp_valid
  io.resp.bits.data := vpu.io.completed_dest_reg
  io.resp.bits.uop := resp_uop
  io.resp.bits.uop.dst_rtype := Mux(resp_uop.dst_rtype === RT_VEC, RT_X, resp_uop.dst_rtype)
  io.resp.bits.uop.uses_stq := 0.B // Trick Rob to acknowledge Vector Store
  io.resp.bits.fflags.valid := vpu.io.completed_valid && vpu.io.completed_fflags.orR
  io.resp.bits.fflags.bits.uop.rob_idx := io.resp.bits.uop.rob_idx
  io.resp.bits.fflags.bits.flags := vpu.io.completed_fflags

  io.debug_wb_vec_valid := vpu.io.debug_wb_vec_valid
  io.debug_wb_vec_wdata := vpu.io.debug_wb_vec_wdata
  io.debug_wb_vec_wmask := vpu.io.debug_wb_vec_wmask

  val issue_credit_cnt = RegInit(MAX_ISSUE_CREDIT.U)
  issue_credit_cnt := issue_credit_cnt + vpu.io.issue_credit - vpu.io.issue_valid 
  val vpu_ready = (issue_credit_cnt =/= 0.U) && sb_ready // <== !full

/*
   OVI LS helper start
*/


/*
  Input / Output with VPU should be defined here
*/
  

  val MemSyncStart = vpu.io.memop_sync_start
  val MemStoreValid = vpu.io.store_valid 
  val MemStoreData = vpu.io.store_data
  val MemMaskValid = vpu.io.mask_idx_valid
  val MemMaskId    = Cat(vpu.io.mask_idx_last_idx, vpu.io.mask_idx_item)

  val MemSyncEnd = WireInit(false.B)
  val MemSbId = WireInit(0.U(5.W))
  val VstartVlfof = WireInit(0.U(15.W))
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
  vLSIQ start
*/

  val vlsi_queue = Module(new OviQueue(vlsiQDepth))
  vlsi_queue.io.core := io.core

  // Dequeue a request whenever VPU and v.ls i-queue are ready
  io.req.ready := vpu_ready && vlsi_queue.io.enq.ready

  // v.ls i-queue enq logic
  vlsi_queue.io.enq.valid := io.req.fire && (io.req.bits.uop.uses_stq || io.req.bits.uop.uses_ldq)
  vlsi_queue.io.enq.bits.req     := io.req.bits
  vlsi_queue.io.enq.bits.vconfig := io.vconfig
  vlsi_queue.io.enq.bits.vxrm    := io.vxrm
  vlsi_queue.io.enq.bits.fcsr_rm := io.fcsr_rm
  vlsi_queue.io.enq.bits.vstart  := io.vstart
  vlsi_queue.io.enq.bits.sb_id   := next_sb_id
  vlsi_queue.io.enq.bits.poison  := false.B // the que will automatically update the poison bit

  // ===============  OSC3 LSGEN DEQ CODE START ===============
  
  // 1-element deep buffer for decoder inputs (it was 3 but we really only need a flip-flop)
  val decoder_buffer = Module(new OviQueue(1))
  decoder_buffer.io.core := io.core

  // Counter-based VLSIQ dequeue logic with outstanding operation tracking
  // Outstanding counter: accumulate MemSyncStart signals
  // NOTE: technically this is a "outstanding v.mem_op AND started decoding" counter
  val outstanding_ctr = RegInit(0.U(log2Ceil(MAX_OUTSTANDING_VMEMOPS).W))

  // reset the deq logic
  vlsi_queue.io.deq.ready := false.B
  decoder_buffer.io.enq.valid := false.B

  // Counter update logic
  val memsync_arrives = MemSyncStart
  val vlsiq_fire = (outstanding_ctr =/= 0.U) && (vlsi_queue.io.deq.valid) && (decoder_buffer.io.enq.ready)

  when (memsync_arrives && !vlsiq_fire) {
    when (outstanding_ctr =/= (MAX_OUTSTANDING_VMEMOPS-1).U) {
      outstanding_ctr := outstanding_ctr + 1.U
    } .otherwise {
      assert(false.B, "ERROR: Outstanding counter overflow! Increase MAX_OUTSTANDING_VMEMOPS.")
    }
  } .elsewhen (!memsync_arrives && vlsiq_fire) {
    outstanding_ctr := outstanding_ctr - 1.U
  }

  // Override VLSIQ and decoder buffer signals
  when (vlsiq_fire) {
    vlsi_queue.io.deq.ready := true.B
    decoder_buffer.io.enq.valid := true.B
  }
  decoder_buffer.io.enq.bits := vlsi_queue.io.deq.bits

  // Debug signals
  dontTouch(outstanding_ctr)
  dontTouch(memsync_arrives)
  dontTouch(vlsiq_fire)

  // ===============  OSC3 LSGEN DEQ CODE END ===============


/*
   v-Helper Start
*/


  // ===============  OSC3 LSGEN INSTANTIATION CODE START ===============
  // code written by Kishore S (8/5/2025)

  // Instantiate the new LS decoder - connect to buffer output
  val lsDecoder = Module(new OviLsDecode(vpuVlen, lsuDmemWidth))
  // Instantiate load and store generators
  val loadGen = Module(new LoadGen(vpuVlen, lsuDmemWidth))
  val storeGen = Module(new StoreGen(vpuVlen, lsuDmemWidth))
  // connect core signals to generators (for poison bit updates)
  loadGen.io.core := io.core
  storeGen.io.core := io.core

  // Use OR of both generators' gen_active signals instead of separate register
  val gen_active = (loadGen.io.gen_active || storeGen.io.gen_active)

  lsDecoder.io.in.req := decoder_buffer.io.deq.bits
  lsDecoder.io.in.valid := decoder_buffer.io.deq.valid


  // Instantiate mask/index buffer (for small control data)
  val maskIdxBuffer = Module(new MaskIdxBuff(WIDTH = 66, DEPTH = 4)) // 66-bit width, 4 entries deep
  
  // Instantiate Vector Data Buffer (for large store data)
  val vecDataBuffer = Module(new VecDataBuffer(R_WIDTH = lsuDmemWidth, W_WIDTH = oviWidth, DEPTH = vdbDepth))


  // Connect mask/index buffer to VPU mask/index data
  maskIdxBuffer.io.mask_idx_in.valid := MemMaskValid
  maskIdxBuffer.io.mask_idx_in.bits := MemMaskId
  
  // Connect Vector Data Buffer to VPU store data
  vecDataBuffer.io.data_in.valid := MemStoreValid
  vecDataBuffer.io.data_in.bits := MemStoreData
  
  // Connect Decoder Buffer to Generators
  // Dequeue from buffer when generators accept and when no generators are currently active
  val load_gen_handshake = lsDecoder.io.out.valid && lsDecoder.io.out.is_load && loadGen.io.start.ready && !gen_active
  val store_gen_handshake = lsDecoder.io.out.valid && !lsDecoder.io.out.is_load && storeGen.io.start.ready && !gen_active
  lsDecoder.io.out.ready := load_gen_handshake || store_gen_handshake
  decoder_buffer.io.deq.ready := lsDecoder.io.in.ready
  
  // Connect decoder outputs to generators
  loadGen.io.start.valid := lsDecoder.io.out.valid && lsDecoder.io.out.is_load && !gen_active
  loadGen.io.start.bits := lsDecoder.io.out.dec_info
  // Connect mask/index buffer to load generator
  loadGen.io.mask_idx.valid := maskIdxBuffer.io.mask_idx_out.valid
  loadGen.io.mask_idx.data := maskIdxBuffer.io.mask_idx_out.bits

  storeGen.io.start.valid := lsDecoder.io.out.valid && !lsDecoder.io.out.is_load && !gen_active
  storeGen.io.start.bits := lsDecoder.io.out.dec_info
  // Connect mask/index buffer to store generator
  storeGen.io.mask_idx.valid := maskIdxBuffer.io.mask_idx_out.valid
  storeGen.io.mask_idx.data := maskIdxBuffer.io.mask_idx_out.bits
  // Connect Vector Data Buffer to store generator
  storeGen.io.vdb_data.valid_bytes := vecDataBuffer.io.data_out.valid_bytes
  storeGen.io.vdb_data.data := vecDataBuffer.io.data_out.bits

  // Shadow outputs - connect to actual LSU ready signals
  loadGen.io.load_packet.ready := io.vGenIO.req.ready
  storeGen.io.store_packet.ready := io.vGenIO.req.ready
  
  // Connect buffer ready signals (shadow connections)
  maskIdxBuffer.io.mask_idx_out.ready := loadGen.io.mask_idx.ready || storeGen.io.mask_idx.ready
  vecDataBuffer.io.data_out.read_bytes := storeGen.io.vdb_data.read_bytes
  vecDataBuffer.io.data_out.read_all := storeGen.io.vdb_data.read_all
  
  // Add dontTouch to preserve signals for observation (shadow code)
  dontTouch(loadGen.io.start.valid)
  dontTouch(loadGen.io.start.bits)
  dontTouch(storeGen.io.start.valid)
  dontTouch(storeGen.io.start.bits)
  dontTouch(loadGen.io.load_packet.valid)
  dontTouch(loadGen.io.load_packet.bits)
  dontTouch(storeGen.io.store_packet.valid) 
  dontTouch(storeGen.io.store_packet.bits)
  dontTouch(lsDecoder.io.out.is_load)
  dontTouch(lsDecoder.io.out.dec_info)
  
  // Add dontTouch for buffer observation
  dontTouch(maskIdxBuffer.io.mask_idx_in.valid)
  dontTouch(maskIdxBuffer.io.mask_idx_in.bits)
  dontTouch(maskIdxBuffer.io.mask_idx_out.valid)
  dontTouch(maskIdxBuffer.io.mask_idx_out.bits)
  dontTouch(vecDataBuffer.io.data_in.valid)
  dontTouch(vecDataBuffer.io.data_in.bits)
  dontTouch(vecDataBuffer.io.data_out.valid_bytes)
  dontTouch(vecDataBuffer.io.data_out.bits)
  dontTouch(vecDataBuffer.io.credit)

  // ===============  OSC3 LSGEN INSTANTIATION CODE END ===============


  // Latch the load/store type when decoder fires to know which generator to use
  val gen_is_load = RegInit(false.B)
  
  // Set the operation type when decoder handshake occurs
  when (load_gen_handshake) {
    gen_is_load := true.B
  } .elsewhen (store_gen_handshake) {
    gen_is_load := false.B  
  }

  // Generate the sequence ID for the fake load return queue
  val seq_id = Wire(UInt(34.W))
  seq_id := Cat(
    loadGen.io.load_packet.bits.sb_id(4, 0),                           // bits 33:29 (5 bits)
    loadGen.io.load_packet.bits.el_count(6, 0),                        // bits 28:22 (7 bits)  
    loadGen.io.load_packet.bits.el_off(5, 0),                          // bits 21:16 (6 bits)
    Cat(0.U((11-log2Ceil(vpuVlen/8)).W), loadGen.io.load_packet.bits.el_id), // bits 15:5  (11 bits total, padded)
    loadGen.io.load_packet.bits.v_reg(4, 0)                            // bits 4:0   (5 bits)
  )

  // =============== Route load/store to LSU or fake queue ===============
  // NOTE: this section is only long becuase of difference in naming (LSU vs OVI)

  val fakeLoadReturnQueue = Module(new Queue(UInt(34.W), fakeLoadDepth))
  fakeLoadReturnQueue.io.deq.ready := false.B 
  fakeLoadReturnQueue.io.enq.valid := gen_active && gen_is_load && loadGen.io.load_packet.valid && loadGen.io.load_packet.bits.is_fake
  fakeLoadReturnQueue.io.enq.bits := seq_id

  // Override the original OVI→LSU outputs with generator outputs
  when (gen_active && gen_is_load && loadGen.io.load_packet.valid) {
    // Load packet → LSU req interface
    io.vGenIO.req.valid := loadGen.io.load_packet.valid
    io.vGenIO.req.bits.uop := loadGen.io.load_packet.bits.uop
    // i dont like the idea of changing the uop itself but i will reverse this when reconstructing the LSU
    io.vGenIO.req.bits.uop.mem_size := addrBreak.U  // For loads: use DMEM width
    io.vGenIO.req.bits.data := 0.U  // Loads don't send data
    io.vGenIO.req.bits.last := loadGen.io.load_packet.bits.last
    io.vGenIO.req.bits.addr := Cat(loadGen.io.load_packet.bits.addr(39, addrBreak), 0.U(addrBreak.W))  // Crop lower address bits for loads
    io.vGenIO.req.bits.predicated := false.B
    io.vGenIO.req.bits.fflags.valid := false.B
    io.vGenIO.req.bits.mxcpt.valid := false.B
    io.vGenIO.req.bits.sfence.valid := false.B
    
    // Load packet → LSU reqHelp interface
    io.vGenIO.reqHelp.valid := loadGen.io.load_packet.valid
    io.vGenIO.reqHelp.bits.uop := loadGen.io.load_packet.bits.uop
    io.vGenIO.reqHelp.bits.poison := loadGen.io.load_packet.bits.poison
    io.vGenIO.reqHelp.bits.elemID := loadGen.io.load_packet.bits.el_id
    io.vGenIO.reqHelp.bits.elemOffset := loadGen.io.load_packet.bits.el_off
    io.vGenIO.reqHelp.bits.elemCount := loadGen.io.load_packet.bits.el_count
    io.vGenIO.reqHelp.bits.vRegID := loadGen.io.load_packet.bits.v_reg
    io.vGenIO.reqHelp.bits.sbId := loadGen.io.load_packet.bits.sb_id
    io.vGenIO.reqHelp.bits.strideDir := loadGen.io.load_packet.bits.dir
    io.vGenIO.reqHelp.bits.isMask := loadGen.io.load_packet.bits.mask_valid
    io.vGenIO.reqHelp.bits.Mask := loadGen.io.load_packet.bits.mask_data(31, 0)  // Truncate to 32 bits
    io.vGenIO.reqHelp.bits.isFake := loadGen.io.load_packet.bits.is_fake
    io.vGenIO.reqHelp.bits.misaligned := loadGen.io.load_packet.bits.misaligned
  }.elsewhen (gen_active && !gen_is_load && storeGen.io.store_packet.valid) {
    // Store packet → LSU req interface  
    io.vGenIO.req.valid := storeGen.io.store_packet.valid
    io.vGenIO.req.bits.uop := storeGen.io.store_packet.bits.uop
    // i dont like the idea of changing the uop itself but i will reverse this when reconstructing the LSU
    io.vGenIO.req.bits.uop.mem_size := storeGen.io.store_packet.bits.mem_size  // For stores: use actual memory operation size
    io.vGenIO.req.bits.data := storeGen.io.store_packet.bits.data
    io.vGenIO.req.bits.last := storeGen.io.store_packet.bits.last
    io.vGenIO.req.bits.addr := storeGen.io.store_packet.bits.addr  // For stores: use full address
    io.vGenIO.req.bits.predicated := false.B
    io.vGenIO.req.bits.fflags.valid := false.B
    io.vGenIO.req.bits.mxcpt.valid := false.B
    io.vGenIO.req.bits.sfence.valid := false.B
    
    // Store packet → LSU reqHelp interface
    io.vGenIO.reqHelp.valid := storeGen.io.store_packet.valid
    io.vGenIO.reqHelp.bits.uop := storeGen.io.store_packet.bits.uop
    io.vGenIO.reqHelp.bits.poison := storeGen.io.store_packet.bits.poison
    io.vGenIO.reqHelp.bits.elemID := storeGen.io.store_packet.bits.elem_id
    io.vGenIO.reqHelp.bits.elemOffset := 0.U  // Store generators don't track element offsets the same way
    io.vGenIO.reqHelp.bits.elemCount := 1.U   // Stores typically handle 1 element at a time
    io.vGenIO.reqHelp.bits.vRegID := 0.U      // Not applicable for stores
    io.vGenIO.reqHelp.bits.sbId := storeGen.io.store_packet.bits.sb_id
    io.vGenIO.reqHelp.bits.strideDir := false.B  // Not applicable for stores
    io.vGenIO.reqHelp.bits.isMask := false.B     // Stores don't use mask interface the same way
    io.vGenIO.reqHelp.bits.Mask := 0.U           // Not applicable for stores  
    io.vGenIO.reqHelp.bits.isFake := storeGen.io.store_packet.bits.is_fake
    io.vGenIO.reqHelp.bits.misaligned := storeGen.io.store_packet.bits.misaligned
  }.otherwise {
    // When generators are not active, set LSU outputs to invalid
    io.vGenIO.req.valid := false.B
    io.vGenIO.req.bits := DontCare
    io.vGenIO.reqHelp.valid := false.B
    io.vGenIO.reqHelp.bits := DontCare
  }



  // =============== Mem Response Handler ===============

  val ovi_resp_handler = Module(new OviLSURespHandler(MAX_OUTSTANDING_VMEMOPS))

  // new entry for memop tracker from vlsiq
  ovi_resp_handler.io.enq.valid := vlsiq_fire
  ovi_resp_handler.io.enq.bits  := vlsi_queue.io.deq.bits
  // response from LSU
  ovi_resp_handler.io.lsu_resp  := io.vGenIO.resp
  // fake load return queue
  ovi_resp_handler.io.fake_load_return_data <> fakeLoadReturnQueue.io.deq
  // core signals
  ovi_resp_handler.io.core_in    := io.core
  // core reports
  // val TODO_core_out := ovi_resp_handler.io.core_out
  // vpu reports
  MemSyncEnd  := ovi_resp_handler.io.vpu.sync_end
  MemSbId     := ovi_resp_handler.io.vpu.sb_id
  VstartVlfof := ovi_resp_handler.io.vpu.vstart_vlfof
  // vpu load data
  MemLoadValid       := ovi_resp_handler.io.vpu.load_valid
  MemSeqId           := ovi_resp_handler.io.vpu.load_seq_id
  MemLoadData        := ovi_resp_handler.io.vpu.load_data
  MemReturnMaskValid := ovi_resp_handler.io.vpu.load_mask_valid
  MemReturnMask      := ovi_resp_handler.io.vpu.load_mask

  // // -- Memop Sync End logic --
  
  // val PendingSBIDMap = RegInit(0.U(32.W)) // like a bool map of pending sb_ids
  // val vstDone = io.vGenIO.resp.vectorDoneSt
  // val vldDone = io.vGenIO.resp.vectorDoneLd

  // when (vstDone && !vldDone) {
  //   MemSbId := io.vGenIO.resp.sbIdDoneSt  
  //   MemSyncEnd := true.B
  // }.elsewhen (!vstDone && vldDone) {
  //   MemSbId := io.vGenIO.resp.sbIdDoneLd 
  //   MemSyncEnd := true.B
  // }.elsewhen (vstDone && vldDone) {
  //   MemSbId := io.vGenIO.resp.sbIdDoneLd
  //   PendingSBIDMap := PendingSBIDMap.bitSet (io.vGenIO.resp.sbIdDoneSt, true.B)
  //   MemSyncEnd := true.B
  // }.elsewhen (PendingSBIDMap =/= 0.U) {
  //   MemSbId := PriorityEncoder (PendingSBIDMap)
  //   PendingSBIDMap := PendingSBIDMap.bitSet (MemSbId, false.B)
  //   MemSyncEnd := true.B
  // }.otherwise {
  //   MemSbId := 0.U
  //   MemSyncEnd := false.B
  // }

  // // -- Handle load data response --

  // // Parse the LSU response (invert for neg stride)
  // val LSUReturnLoadValid = io.vGenIO.resp.vectorDataBack && io.vGenIO.resp.s0l1 // only consider loads
  // val LSUReturnData = Mux(
  //   io.vGenIO.resp.strideDir,
  //   Cat(io.vGenIO.resp.data ((lsuDmemWidth-1), 0), 0.U((oviWidth-lsuDmemWidth).W)), // negative stride
  //   Cat(0.U, io.vGenIO.resp.data ((lsuDmemWidth-1), 0))                             // positive stride
  // )

  // // Mux the response from fake queue or the real load..
  // // TODO: what if we have fake responses left in the queue when we get a memop_sync_end? Need to check for this case..
  // MemLoadValid := LSUReturnLoadValid || fakeLoadReturnQueue.io.deq.valid
  // MemSeqId := Cat (seqSbId, seqElCount, seqElOff, seqElId, seqVreg) 

  // when (LSUReturnLoadValid) {
  //   MemLoadData := LSUReturnData    
  //   seqSbId := io.vGenIO.resp.sbId
  //   seqElCount := io.vGenIO.resp.elemCount
  //   seqElOff := io.vGenIO.resp.elemOffset
  //   seqElId := Cat(0.U(3.W), io.vGenIO.resp.elemID)
  //   seqVreg := io.vGenIO.resp.vRegID
    
  //   MemReturnMaskValid := io.vGenIO.resp.isMask
  //   MemReturnMask := io.vGenIO.resp.Mask
  // }.elsewhen (fakeLoadReturnQueue.io.deq.valid) {    
  //   MemLoadData := 0.U     
  //   seqSbId := fakeLoadReturnQueue.io.deq.bits (33, 29)
  //   seqElCount := fakeLoadReturnQueue.io.deq.bits (28, 22)
  //   seqElOff := fakeLoadReturnQueue.io.deq.bits (21, 16)
  //   seqElId := fakeLoadReturnQueue.io.deq.bits (15, 5)
  //   seqVreg := fakeLoadReturnQueue.io.deq.bits (4, 0)
  //   MemReturnMaskValid := true.B 
  //   MemReturnMask := false.B 
  //   fakeLoadReturnQueue.io.deq.ready := true.B 
  // }

  // // =============== Vstart Bookkeeping ===============

  // class VstartVlfofTrackerEntry extends Bundle {
  //   val valid = Bool()
  //   val exception = Bool()
  //   val xcpt_cause = UInt(xLen.W)
  //   val sb_id = UInt(5.W)
  //   val vstart_vlfof = UInt(15.W)
  // }
  // val vsvlf_tracker = RegInit(VecInit.fill(MAX_OUTSTANDING_VMEMOPS)(0.U.asTypeOf(new VstartVlfofTrackerEntry)))
  // val vsvlf_next_available_vec = PriorityEncoderOH(vsvlf_tracker.map(~_.valid))

  // for (i <- 0 until MAX_OUTSTANDING_VMEMOPS) {
  //   // when a memop ends (last possible el_id for vstart/vlfof has been sent)
  //   // we need to send the final value to the VPU and reset the tracker
  //   when (
  //     (MemSyncEnd) &&
  //     (vsvlf_tracker(i).valid) &&
  //     (vsvlf_tracker(i).sb_id === MemSbId)
  //   ) {
  //     // sometimes the memop ends on the same cycle as the last element is being
  //     // transferred. forward the exception
  //     when (
  //       (io.vGenIO.resp.vectorDataBack) &&
  //       (vsvlf_tracker(i).valid) &&
  //       (io.vGenIO.resp.exception) &&
  //       (vsvlf_tracker(i).sb_id === io.vGenIO.resp.sbId) &&
  //       (vsvlf_tracker(i).vstart_vlfof > io.vGenIO.resp.elemID)
  //     ) {
  //       VstartVlfof := io.vGenIO.resp.elemID
  //     }
  //     // otherwise, just send the final value from the table
  //     .otherwise {
  //       VstartVlfof := vsvlf_tracker(i).vstart_vlfof
  //     }
  //     // reset the tracker
  //     vsvlf_tracker(i).valid := false.B
  //     vsvlf_tracker(i).exception := false.B
  //     vsvlf_tracker(i).xcpt_cause := 0.U
  //     vsvlf_tracker(i).vstart_vlfof := 0.U
  //   }

  //   // when theres a valid memory packet that is being transferred
  //   // we need to update the tracker with the minimum value
  //   .elsewhen (
  //     (io.vGenIO.resp.vectorDataBack) &&
  //     (vsvlf_tracker(i).valid) &&
  //     (io.vGenIO.resp.exception) &&
  //     (vsvlf_tracker(i).sb_id === io.vGenIO.resp.sbId) &&
  //     (vsvlf_tracker(i).vstart_vlfof > io.vGenIO.resp.elemID)
  //   ) {
  //     vsvlf_tracker(i).exception := true.B
  //     vsvlf_tracker(i).xcpt_cause := io.vGenIO.resp.xcpt_cause
  //     vsvlf_tracker(i).vstart_vlfof := io.vGenIO.resp.elemID
  //   }

  //   // when a new memory transaction is starting
  //   // reserve a slot for the new transaction (valid + sb_id)
  //   .elsewhen (
  //     (vlsiq_fire) &&
  //     (vsvlf_next_available_vec(i))
  //   ) {
  //     assert(!vsvlf_tracker(i).valid, "ERROR: VstartVlfof tracker slot is already marked valid!")
  //     assert(!(VecInit(vsvlf_next_available_vec).asUInt & (VecInit(vsvlf_next_available_vec).asUInt - 1.U)), "ERROR: VstartVlfof tracker has multiple slots marked available!")
  //     vsvlf_tracker(i).valid := true.B
  //     vsvlf_tracker(i).exception := false.B
  //     vsvlf_tracker(i).xcpt_cause := 0.U
  //     vsvlf_tracker(i).sb_id := vlsi_queue.io.deq.bits.sb_id
  //     vsvlf_tracker(i).vstart_vlfof := 0.U
  //   }
  // }


  // =============== Send Data to VPU ===============

  vpu.io := DontCare
  vpu.io.clk := clock
  vpu.io.reset_n := ~reset.asBool
  vpu.io.issue_valid := io.req.fire
  vpu.io.issue_inst := io.req.bits.uop.inst
  vpu.io.issue_sb_id := next_sb_id
  vpu.io.issue_scalar_opnd := Mux(
    io.req.bits.uop.lrs1_rtype === RT_FLT,
    io.req.bits.rs3_data,
    Mux(
      io.req.bits.uop.uses_ldq || io.req.bits.uop.uses_stq,
      io.req.bits.rs2_data,
      io.req.bits.rs1_data
    )
  )
  vpu.io.issue_vcsr := Cat(
    0.U(1.W), // vill
    io.vconfig.vtype.vsew, // vsew
    io.vconfig.vtype.vlmul_mag, // vlmul
    io.fcsr_rm, // frm
    io.vxrm, // vxrm
    Cat(0.U((15-log2Ceil(vLen+1)).W), io.vconfig.vl), // vl
    Cat(0.U((14-log2Ceil(vLen+1)).W), io.vstart), // vstart
  )
  vpu.io.issue_vcsr_lmulb2 := io.vconfig.vtype.vlmul_sign
  vpu.io.dispatch_sb_id := dispatch_sb_id
  vpu.io.dispatch_next_senior := dispatch_next_senior
  vpu.io.dispatch_kill := dispatch_kill
  vpu.io.memop_sync_end := MemSyncEnd
  vpu.io.memop_sb_id := MemSbId  
  vpu.io.memop_vstart_vlfof := VstartVlfof
  vpu.io.load_valid := MemLoadValid
  vpu.io.load_seq_id := MemSeqId
  vpu.io.load_data := MemLoadData
  vpu.io.load_mask_valid := MemReturnMaskValid
  vpu.io.load_mask := MemReturnMask
  vpu.io.store_credit := vecDataBuffer.io.credit
  vpu.io.mask_idx_credit := maskIdxBuffer.io.credit
  
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
    val memop_vstart_vlfof = Input(UInt(15.W))
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
  addResource("/vsrc/vpu/tt_briscv_pkg.svh")
  addResource("/vsrc/vpu/autogen_riscv_imabfv.v")
  addResource("/vsrc/vpu/autogen_defines.h")
  addResource("/vsrc/vpu/tt_id.sv")
  addResource("/vsrc/vpu/tt_ex.sv")
  addResource("/vsrc/vpu/tt_lq.sv")
  addResource("/vsrc/vpu/tt_vec.sv")
  addResource("/vsrc/vpu/tt_vec_iadd.sv")
  addResource("/vsrc/vpu/tt_vec_idp.sv")
  addResource("/vsrc/vpu/tt_vec_imul.sv")
  addResource("/vsrc/vpu/tt_vec_mul_dp.sv")
  addResource("/vsrc/vpu/tt_vec_div_unit.sv")
  addResource("/vsrc/vpu/VecFP16rsqrt7.sv")
  addResource("/vsrc/vpu/VecFP32rsqrt7.sv")
  addResource("/vsrc/vpu/VecFP32rec7.sv")
  addResource("/vsrc/vpu/VecFP16rec7.sv")
  addResource("/vsrc/vpu/tt_int_div_simple.sv")
  addResource("/vsrc/vpu/tt_fp16_div.sv")
  addResource("/vsrc/vpu/tt_fp32_div.sv")
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
  addResource("/vsrc/vpu/tt_idxldst_fsm.sv")
  addResource("/vsrc/vpu/tt_reshape.sv")
  addResource("/vsrc/vpu/tt_memop_fsm.sv")
  addResource("/vsrc/vpu/tt_mask_fsm.sv")
  addResource("/vsrc/vpu/tt_store_buffer.sv")
  addResource("/vsrc/vpu/tt_load_buffer.sv")
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
  addResource("/vsrc/HardFloat/source/divSqrtRecFN_small.v")
}



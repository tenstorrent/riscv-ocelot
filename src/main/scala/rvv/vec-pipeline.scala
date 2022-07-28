// See LICENSE.TT for license details.
package boom.rvv

import chisel3._
import chisel3.util._
import chisel3.experimental._

import freechips.rocketchip.config.{Parameters}
import freechips.rocketchip.rocket.{VConfig}

import boom.exu.FUConstants._
import boom.common._
import boom.exu._
import boom.util._
import boom.lsu.{LSUExeIO}

import hardfloat._

class VecPipeline (xLen: Int, vLen: Int)(implicit p: Parameters) extends BoomModule
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
   val io = IO(new Bundle {
      val vconfig = Input(new VConfig())
      val vxrm    = Input(UInt(2.W))
      val fcsr_rm = Input(UInt(3.W))
      val req    = Flipped(new DecoupledIO(new FuncUnitReq(xLen)))
      val resp   = new DecoupledIO(new FuncUnitResp(xLen))
      val set_vxsat = Output(Bool())

      // To LSU
      val vec_dis_uops    = Valid(new MicroOp)
      val vec_dis_ldq_idx = Input(UInt(ldqAddrSz.W))
      val vec_dis_stq_idx = Input(UInt(stqAddrSz.W))
    
      val vec_ldq_full    = Input(Bool())
      val vec_stq_full    = Input(Bool())
    
      val vec_lsu_io      = Flipped(Vec(2, new LSUExeIO))
      val vec_lsu_stall   = Input(Bool())
   })

   val vfp_pipeline = Module(new vfp_pipeline(coreParams.vLen, coreMaxAddrBits))
   val req_queue    = Module(new Queue(new FuncUnitReq(xLen), 4))
   val uop_queue    = Module(new Queue(new MicroOp(), 20))
   val lsu_queue    = Module(new Queue(new FuncUnitResp(xLen), 2))

   io.req.ready := vfp_pipeline.io.o_id_instrn_rtr &&
                   req_queue.io.enq.ready

   req_queue.io.enq.valid := io.req.valid
   req_queue.io.enq.bits  := io.req.bits
   req_queue.io.deq.ready := vfp_pipeline.io.o_id_instrn_rtr

   vfp_pipeline.io.i_clk   := clock.asBool
   vfp_pipeline.io.i_reset := reset.asBool
   vfp_pipeline.io.i_csr_vl := io.vconfig.vl
   vfp_pipeline.io.i_csr_vsew := io.vconfig.vtype.vsew
   vfp_pipeline.io.i_csr_vlmul := io.vconfig.vtype.vlmul_signed.asUInt
   vfp_pipeline.io.i_csr_vxrm := io.vxrm
   vfp_pipeline.io.i_csr_frm  := io.fcsr_rm
   vfp_pipeline.io.i_if_instrn_rts := req_queue.io.deq.valid
   vfp_pipeline.io.i_if_instrn := req_queue.io.deq.bits.uop.inst
   vfp_pipeline.io.i_if_pc := 0.U
   vfp_pipeline.io.i_rf_vex_p0 := req_queue.io.deq.bits.rs1_data
   vfp_pipeline.io.i_rf_vex_p1 := req_queue.io.deq.bits.rs2_data
   vfp_pipeline.io.i_fprf_vex_p0 := req_queue.io.deq.bits.rs3_data

   vfp_pipeline.io.i_data_req_rtr      := lsu_queue.io.enq.ready
   vfp_pipeline.io.i_rd_data_vld_0     :=     io.vec_lsu_io(0).vresp.valid
   vfp_pipeline.io.i_rd_data_resp_id_0 := Cat(io.vec_lsu_io(0).vresp.bits.uop.rob_idx(3,0), io.vec_lsu_io(0).vresp.bits.uop.pdst(5,0))
   vfp_pipeline.io.i_rd_data_0         :=     io.vec_lsu_io(0).vresp.bits.data
   vfp_pipeline.io.i_rd_data_vld_1     :=     io.vec_lsu_io(1).vresp.valid
   vfp_pipeline.io.i_rd_data_resp_id_1 := Cat(io.vec_lsu_io(1).vresp.bits.uop.rob_idx(3,0), io.vec_lsu_io(1).vresp.bits.uop.pdst(5,0))
   vfp_pipeline.io.i_rd_data_1         :=     io.vec_lsu_io(1).vresp.bits.data

   uop_queue.io.enq.valid := req_queue.io.deq.valid && req_queue.io.deq.ready
   uop_queue.io.enq.bits  := req_queue.io.deq.bits.uop
   uop_queue.io.deq.ready := vfp_pipeline.io.o_instrn_commit_valid

   // Allocate LDQ for Vector Loads
   io.vec_dis_uops.valid                 :=  lsu_queue.io.deq.valid && lsu_queue.io.deq.ready
   io.vec_dis_uops.bits.pdst             :=  lsu_queue.io.deq.bits.uop.pdst
   io.vec_dis_uops.bits.ldq_idx          :=  io.vec_dis_ldq_idx
   io.vec_dis_uops.bits.stq_idx          :=  lsu_queue.io.deq.bits.uop.stq_idx
   io.vec_dis_uops.bits.rob_idx          :=  lsu_queue.io.deq.bits.uop.rob_idx
   io.vec_dis_uops.bits.uses_ldq         :=  lsu_queue.io.deq.bits.uop.uses_ldq
   io.vec_dis_uops.bits.uses_stq         :=  lsu_queue.io.deq.bits.uop.uses_stq
   io.vec_dis_uops.bits.dst_rtype        :=  RT_VEC
   io.vec_dis_uops.bits.mem_size         :=  lsu_queue.io.deq.bits.uop.mem_size
   io.vec_dis_uops.bits.mem_cmd          :=  lsu_queue.io.deq.bits.uop.mem_cmd
   io.vec_dis_uops.bits.is_vec           :=  1.B
   io.vec_dis_uops.bits.last_vec_stq     :=  lsu_queue.io.deq.bits.uop.last_vec_stq
   io.vec_dis_uops.bits.is_empty_st      :=  lsu_queue.io.deq.bits.uop.is_empty_st 

   lsu_queue.io.enq.valid                 :=     vfp_pipeline.io.o_data_req &&
                                              (  vfp_pipeline.io.o_data_byten.orR ||
                                               (!vfp_pipeline.io.o_mem_load &&
                                                 vfp_pipeline.io.o_mem_last   )     )
   lsu_queue.io.enq.bits                  := DontCare
   lsu_queue.io.enq.bits.addr             :=     vfp_pipeline.io.o_data_addr
   lsu_queue.io.enq.bits.data             :=     vfp_pipeline.io.o_wr_data(63,0)
   lsu_queue.io.enq.bits.uop.pdst         :=     vfp_pipeline.io.o_data_req_id(5,0)
   lsu_queue.io.enq.bits.uop.rob_idx      :=     vfp_pipeline.io.o_data_req_id(9,6)
   lsu_queue.io.enq.bits.uop.uses_ldq     :=     vfp_pipeline.io.o_mem_load
   lsu_queue.io.enq.bits.uop.uses_stq     :=    !vfp_pipeline.io.o_mem_load
   lsu_queue.io.enq.bits.uop.dst_rtype    := RT_VEC
   lsu_queue.io.enq.bits.uop.mem_size     :=     vfp_pipeline.io.o_mem_size
   lsu_queue.io.enq.bits.uop.mem_cmd      := Mux(vfp_pipeline.io.o_mem_load, M_XRD, M_XWR)
   lsu_queue.io.enq.bits.uop.ctrl.is_load :=     vfp_pipeline.io.o_mem_load
   lsu_queue.io.enq.bits.uop.ctrl.is_sta  :=    !vfp_pipeline.io.o_mem_load
   lsu_queue.io.enq.bits.uop.ctrl.is_std  :=    !vfp_pipeline.io.o_mem_load
   lsu_queue.io.enq.bits.uop.is_vec       := 1.B
   lsu_queue.io.enq.bits.uop.last_vec_stq :=    !vfp_pipeline.io.o_mem_load &&
                                                 vfp_pipeline.io.o_mem_last
   lsu_queue.io.enq.bits.uop.is_empty_st  :=    !vfp_pipeline.io.o_mem_load &&
                                                !vfp_pipeline.io.o_data_byten.orR
   lsu_queue.io.enq.bits.uop.stq_idx      := uop_queue.io.deq.bits.stq_idx

   io.vec_lsu_io(0).req.valid                 := RegNext(lsu_queue.io.deq.valid &&
                                                         lsu_queue.io.deq.ready &&
                                                        !lsu_queue.io.deq.bits.uop.is_empty_st)
   io.vec_lsu_io(0).req.bits                  :=         DontCare
   io.vec_lsu_io(0).req.bits.addr             := RegNext(lsu_queue.io.deq.bits.addr)
   io.vec_lsu_io(0).req.bits.data             := RegNext(lsu_queue.io.deq.bits.data)
   io.vec_lsu_io(0).req.bits.uop.pdst         := RegNext(lsu_queue.io.deq.bits.uop.pdst)
   io.vec_lsu_io(0).req.bits.uop.rob_idx      := RegNext(lsu_queue.io.deq.bits.uop.rob_idx)
   io.vec_lsu_io(0).req.bits.uop.uses_ldq     := RegNext(lsu_queue.io.deq.bits.uop.uses_ldq)
   io.vec_lsu_io(0).req.bits.uop.uses_stq     := RegNext(lsu_queue.io.deq.bits.uop.uses_stq)
   io.vec_lsu_io(0).req.bits.uop.dst_rtype    := RegNext(lsu_queue.io.deq.bits.uop.dst_rtype)
   io.vec_lsu_io(0).req.bits.uop.mem_size     := RegNext(lsu_queue.io.deq.bits.uop.mem_size)
   io.vec_lsu_io(0).req.bits.uop.mem_cmd      := RegNext(lsu_queue.io.deq.bits.uop.mem_cmd)
   io.vec_lsu_io(0).req.bits.uop.ctrl.is_load := RegNext(lsu_queue.io.deq.bits.uop.ctrl.is_load)
   io.vec_lsu_io(0).req.bits.uop.ctrl.is_sta  := RegNext(lsu_queue.io.deq.bits.uop.ctrl.is_sta)
   io.vec_lsu_io(0).req.bits.uop.ctrl.is_std  := RegNext(lsu_queue.io.deq.bits.uop.ctrl.is_std)
   io.vec_lsu_io(0).req.bits.uop.is_vec       := RegNext(lsu_queue.io.deq.bits.uop.is_vec)
   io.vec_lsu_io(0).req.bits.uop.last_vec_stq := RegNext(lsu_queue.io.deq.bits.uop.last_vec_stq)
   io.vec_lsu_io(0).req.bits.uop.is_empty_st  := RegNext(lsu_queue.io.deq.bits.uop.is_empty_st )
   io.vec_lsu_io(0).req.bits.uop.ldq_idx      := RegNext( io.vec_dis_ldq_idx)
   io.vec_lsu_io(0).req.bits.uop.stq_idx      := RegNext(lsu_queue.io.deq.bits.uop.stq_idx)

   lsu_queue.io.deq.ready                     := !io.vec_ldq_full && !io.vec_stq_full && !io.vec_lsu_stall
   io.vec_lsu_io(1).req                       := DontCare

   io.resp.valid                        := vfp_pipeline.io.o_instrn_commit_valid
   io.resp.bits.data                    := vfp_pipeline.io.o_instrn_commit_data(63,0)
   io.resp.bits.fflags.valid            := vfp_pipeline.io.o_instrn_commit_fflags.orR
   io.resp.bits.fflags.bits.uop.rob_idx := uop_queue.io.deq.bits.rob_idx
   io.resp.bits.fflags.bits.flags       := vfp_pipeline.io.o_instrn_commit_fflags
   io.resp.bits.uop                     := uop_queue.io.deq.bits
   io.resp.bits.uop.dst_rtype           := Mux(uop_queue.io.deq.bits.dst_rtype === RT_VEC,
                                               RT_X,
                                               uop_queue.io.deq.bits.dst_rtype)
   io.resp.bits.uop.uses_stq            := 0.B // Allow iresp pop ROB entry

   io.set_vxsat                         := vfp_pipeline.io.o_sat_csr

}

class vfp_pipeline(val vlen: Int, val addrWidth: Int) extends BlackBox(Map("VLEN" -> IntParam(vlen), "ADDRWIDTH" -> IntParam(addrWidth)))
with HasBlackBoxResource with HasBlackBoxPath {
  val io = IO(new Bundle {
    val i_clk       = Input(Bool())
    val i_reset     = Input(Bool())
    val i_csr_vl    = Input(UInt(log2Ceil(vlen+1).W))
    val i_csr_vsew  = Input(UInt(3.W))
    val i_csr_vlmul = Input(UInt(3.W))
    val i_csr_vxrm  = Input(UInt(2.W))
    val i_csr_frm   = Input(UInt(3.W))
    val i_if_instrn_rts = Input(Bool())
    val o_id_instrn_rtr = Output(Bool())
    val i_if_instrn     = Input(UInt(32.W))
    val i_if_pc         = Input(UInt(32.W))
    val i_rf_vex_p0     = Input(UInt(64.W))
    val i_rf_vex_p1     = Input(UInt(64.W))
    val i_fprf_vex_p0   = Input(UInt(64.W))
    val o_data_req      = Output(Bool())
    val o_data_addr     = Output(UInt(addrWidth.W))
    val o_data_byten    = Output(UInt((vlen/8).W))
    val o_wr_data       = Output(UInt(vlen.W))
    val o_data_req_id   = Output(UInt(10.W))
    val o_mem_load      = Output(Bool())
    val o_mem_size      = Output(UInt(3.W))
    val o_mem_last      = Output(Bool())
    val i_data_req_rtr  = Input(Bool())
    val i_rd_data_vld_0 = Input(Bool())
    val i_rd_data_resp_id_0 = Input(UInt(10.W))
    val i_rd_data_0         = Input(UInt(64.W))
    val i_rd_data_vld_1 = Input(Bool())
    val i_rd_data_resp_id_1 = Input(UInt(10.W))
    val i_rd_data_1         = Input(UInt(64.W))
    val o_instrn_commit_valid = Output(Bool())
    val o_instrn_commit_data  = Output(UInt((vlen*8).W))
    val o_instrn_commit_mask  = Output(UInt(8.W))
    val o_instrn_commit_fflags = Output(UInt(5.W))
    val o_sat_csr = Output(Bool())
  })
  addResource("/vsrc/rvv/briscv_defines.h")
  addResource("/vsrc/rvv/tt_briscv_pkg.vh")
  addResource("/vsrc/rvv/autogen_riscv_imabfv.v")
  addResource("/vsrc/rvv/autogen_defines.h")
  addResource("/vsrc/rvv/vfp_pipeline.sv")
  addResource("/vsrc/rvv/tt_id.sv")
  addResource("/vsrc/rvv/tt_ex.sv")
  addResource("/vsrc/rvv/tt_lq.sv")
  addResource("/vsrc/rvv/tt_mem.sv")
  addResource("/vsrc/rvv/tt_vec.sv")
  addResource("/vsrc/rvv/tt_vec_iadd.sv")
  addResource("/vsrc/rvv/tt_vec_idp.sv")
  addResource("/vsrc/rvv/tt_vec_imul.sv")
  addResource("/vsrc/rvv/tt_vec_mul_dp.sv")
  addResource("/vsrc/rvv/tt_vec_regfile.sv")
  addResource("/vsrc/rvv/tt_vfp_unit.sv")
  addResource("/vsrc/rvv/tt_vfp_ex_unit.sv")
  addResource("/vsrc/rvv/tt_vfp_lane.sv")
  addResource("/vsrc/rvv/tt_vfp_encoder.sv")
  addResource("/vsrc/rvv/tt_vfp_encoder_lane.sv")
  addResource("/vsrc/rvv/tt_vfp_fma.sv")
  addResource("/vsrc/rvv/tt_vfp_red.sv")
  addResource("/vsrc/rvv/tt_popcnt.sv")
  addResource("/vsrc/rvv/tt_pipe_stage.sv")
  addResource("/vsrc/rvv/tt_rts_rtr_pipe_stage.sv")
  addResource("/vsrc/rvv/tt_cam_buffer.sv")
  addResource("/vsrc/rvv/tt_skid_buffer.sv")
  addResource("/vsrc/rvv/tt_store_fifo.sv")
  addResource("/vsrc/rvv/tt_ffs.sv")
  addResource("/vsrc/rvv/tt_ascii_instrn_decode.sv")
  addResource("/vsrc/rvv/tt_compare.sv")
  addResource("/vsrc/rvv/tt_decoded_mux.sv")
  addResource("/vsrc/rvv/tt_decoder.sv")
  addResource("/vsrc/rvv/tt_reshape.sv")
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



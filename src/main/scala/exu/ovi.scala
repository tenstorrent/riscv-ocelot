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

class OviWrapper (xLen: Int, vLen: Int)(implicit p: Parameters) extends BoomModule
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
   val io = IO(new Bundle {
      val vconfig = Input(new VConfig())
      val vxrm    = Input(UInt(2.W))
      val fcsr_rm = Input(UInt(3.W))
      val req    = Flipped(new DecoupledIO(new FuncUnitReq(xLen)))
      val resp   = new DecoupledIO(new FuncUnitResp(xLen))
      val set_vxsat = Output(Bool())

      val debug_wb_vec_valid = Output(Bool())
      val debug_wb_vec_wdata = Output(UInt((coreParams.vLen*8).W))
      val debug_wb_vec_wmask = Output(UInt(8.W))
   })
   io := DontCare

   val vpuModule = Module(new Vpu)

   vpuModule.io := DontCare
   vpuModule.io.clk := clock
   vpuModule.io.reset_n := ~reset.asBool
   vpuModule.io.issue_valid := io.req.valid
   io.resp.valid := vpuModule.io.completed_valid
   io.set_vxsat := io.req.valid

}

class Vpu
extends BlackBox with HasBlackBoxResource{
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
  })
  addResource("/vsrc/vpu_ovi/tt_vpu_ovi.sv")
}
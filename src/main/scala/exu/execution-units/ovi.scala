// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig, VType}

import boom.common._
import boom.util._

/** Request queue that supports speculation.
 * 
 * This allows requests to be killed much closer to the OVI boundary.
 */ 
class OviReqQueue(val num_entries: Int)(implicit p: Parameters)
  extends BoomModule
{
  val io = IO(new Bundle {
    val enq = DeqIO(new FuncUnitReq(xLen))
    val deq = EnqIO(new FuncUnitReq(xLen))
    val count = Output(UInt(log2Ceil(num_entries + 1).W))

    val rob_pnr_idx, rob_head_idx = Input(UInt(robAddrSz.W))
    val brupdate = Input(new BrUpdateInfo())
    val exception = Input(Bool())
  })

  val entries = Reg(Vec(num_entries, new FuncUnitReq(xLen)))
  val entries_valid = RegInit(VecInit.fill(num_entries)(false.B))
  val enq_ptr, deq_ptr = RegInit(0.U(log2Ceil(num_entries).W))

  ////////////////////////////////////////////////////////////////

  io.count := PopCount(entries_valid)

  // Allow enqueue when not full
  io.enq.ready := entries_valid.asUInt =/= Fill(num_entries, 1.U)

  // Allow dequeue when not empty and past PNR
  io.deq.valid := entries_valid.asUInt =/= 0.U && (
    IsOlder(io.deq.bits.uop.rob_idx, io.rob_pnr_idx, io.rob_head_idx)
    || io.deq.bits.uop.rob_idx === io.rob_pnr_idx
  )

  ////////////////////////////////////////////////////////////////

  // Keep branch masks up-to-date
  for (idx <- 0 until num_entries) {
    val new_mask = GetNewBrMask(io.brupdate, entries(idx).uop)
    entries(idx).uop.br_mask := new_mask
  }

  def is_killed(uop: MicroOp): Bool = (
    // For exceptions, kill everything past the PNR
    (io.exception && !IsOlder(uop.rob_idx, io.rob_pnr_idx, io.rob_head_idx))
    // For branch mispredictions, kill based on branch masks
    || IsKilledByBranch(io.brupdate, uop)
  )
  val killed_entries = VecInit.tabulate(num_entries)(idx =>
    entries_valid(idx) && is_killed(entries(idx).uop)
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
    IsKilledByBranch(io.brupdate, io.enq.bits.uop) ||
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

class VecConfigUnit(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    val req  = Flipped(new DecoupledIO(new FuncUnitReq(xLen)))
    val resp = new DecoupledIO(new FuncUnitResp(xLen))

    val set_vtype = Output(Valid(new VType))
    val set_vl    = Output(Valid(UInt(log2Up(maxVLMax + 1).W)))
  })

  val avl_imm = ImmGen(io.req.bits.uop.imm_packed, io.req.bits.uop.ctrl.imm_sel).asUInt
  val vtype_imm = io.req.bits.uop.imm_packed(19) ## 0.U(xLen.W) ## io.req.bits.uop.imm_packed(15,8)

  val vtype_raw = Wire(UInt())
  val avl = Wire(UInt(xLen.W))
  when (io.req.bits.uop.ctrl.imm_sel === IS_IVLI) {
      vtype_raw := vtype_imm
      avl       := avl_imm
  } .elsewhen (io.req.bits.uop.ctrl.imm_sel === IS_VLI) {
      vtype_raw := vtype_imm
      avl       := io.req.bits.rs1_data
  } .otherwise {
      vtype_raw := io.req.bits.rs2_data
      avl       := io.req.bits.rs1_data
  }

  val vl    = Wire(UInt())
  val vtype = VType.fromUInt(vtype_raw, false)
  // vsetvl(i) with rd=x0 and rs1=x0 doesn't update vl CSR
  val set_vl =
    io.req.bits.uop.ctrl.imm_sel =/= IS_IVLI &&
    io.req.bits.uop.ldst =/= 0.U &&
    io.req.bits.uop.lrs1 =/= 0.U
  // vsetvl(i) with rd!=x0 and rs1=x0 sets vl to VLMAX
  val set_vlmax =
    io.req.bits.uop.ctrl.imm_sel =/= IS_IVLI &&
    io.req.bits.uop.ldst =/= 0.U &&
    io.req.bits.uop.lrs1 === 0.U
  when (set_vlmax) {
    vl := VType.computeVL(avl, vtype_raw, 0.U, 0.B, 1.B, 0.B)
  } .otherwise {
    vl := VType.computeVL(avl, vtype_raw, 0.U, 0.B, 0.B, 0.B)
  }

  io.req.ready  := true.B
  io.resp.valid := io.req.fire

  io.resp.bits.uop          := io.req.bits.uop
  io.resp.bits.data         := vl
  io.resp.bits.fflags.valid := false.B

  io.set_vtype.valid        := io.resp.valid
  io.set_vtype.bits         := vtype

  io.set_vl.valid           := set_vl && io.resp.valid
  io.set_vl.bits            := vl
}

class OviWrapperCoreIO(implicit p: Parameters) extends BoomBundle
{
  val rob_pnr_idx, rob_head_idx = Input(UInt(robAddrSz.W))
  val exception = Input(Bool())

  val set_vtype = Output(Valid(new VType))
  val set_vl    = Output(Valid(UInt(log2Up(maxVLMax + 1).W)))

  val vconfig = Input(new VConfig())
  val vxrm    = Input(UInt(2.W))
  val vGenIO  = Flipped(new boom.lsu.VGenIO)
  val debug_wb_vec_valid = Output(Bool())
  val debug_wb_vec_wdata = Output(UInt((vLen * 8).W))
  val debug_wb_vec_wmask = Output(UInt(8.W))

  val set_vxsat = Output(Bool())
}

class OviWrapperWrapper(implicit p: Parameters) extends BoomModule // Yeah...
{
  val io = IO(new Bundle {
    val fcsr_rm = Input(UInt(3.W))
    val req = Flipped(new DecoupledIO(new FuncUnitReq(xLen)))
    val resp = new DecoupledIO(new FuncUnitResp(xLen))
    val core = new OviWrapperCoreIO
    val brupdate = Input(new BrUpdateInfo())
  })

  ////////////////////////////////////////////////////////////////

  val req_queue = Module(new OviReqQueue(8))
  
  req_queue.io.enq <> io.req
  // The request pipeline takes several cycles to stall
  io.req.ready := req_queue.io.count <= (req_queue.num_entries - 3).U
  assert(!(req_queue.io.enq.valid && !req_queue.io.enq.ready),
         "OviReqQueue overflow")

  req_queue.io.rob_pnr_idx  := io.core.rob_pnr_idx
  req_queue.io.rob_head_idx := io.core.rob_head_idx
  req_queue.io.brupdate     := io.brupdate
  req_queue.io.exception    := io.core.exception

  ////////////////////////////////////////////////////////////////

  val vec_config_unit = Module(new VecConfigUnit())

  io.core.set_vtype := vec_config_unit.io.set_vtype
  io.core.set_vl    := vec_config_unit.io.set_vl

  ////////////////////////////////////////////////////////////////

  val ovi_wrapper = Module(new OviWrapper())

  ovi_wrapper.io.vconfig <> io.core.vconfig
  ovi_wrapper.io.vxrm    <> io.core.vxrm
  ovi_wrapper.io.fcsr_rm <> io.fcsr_rm

  ovi_wrapper.io.vGenIO <> io.core.vGenIO

  ovi_wrapper.io.debug_wb_vec_valid <> io.core.debug_wb_vec_valid
  ovi_wrapper.io.debug_wb_vec_wdata <> io.core.debug_wb_vec_wdata
  ovi_wrapper.io.debug_wb_vec_wmask <> io.core.debug_wb_vec_wmask

  ////////////////////////////////////////////////////////////////

  req_queue.io.deq.nodeq()
  vec_config_unit.io.req.noenq()
  ovi_wrapper.io.req.noenq()

  val uopc = req_queue.io.deq.bits.uop.uopc
  when(uopc === uopVEC) {
    ovi_wrapper.io.req     <> req_queue.io.deq
  }.elsewhen((uopc === uopVSETVL || uopc === uopVSETVLI || uopc === uopVSETIVLI) && !ovi_wrapper.io.resp.valid) {
    vec_config_unit.io.req <> req_queue.io.deq
  }

  when(ovi_wrapper.io.resp.valid) {
    io.resp <> ovi_wrapper.io.resp
    vec_config_unit.io.resp.nodeq()
  }.otherwise {
    io.resp <> vec_config_unit.io.resp
    ovi_wrapper.io.resp.nodeq()
  }

  ////////////////////////////////////////////////////////////////

  io.core.set_vxsat := DontCare
}

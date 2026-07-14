//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal CII host adapter (Milestone 2, Track B)
//------------------------------------------------------------------------------
//
// VecCiiHost is the BOOM-side (host) adapter for the TT-CII vector-arithmetic
// coprocessor. It sits between the in-order IQ_V_ALU issue unit and the SV
// coprocessor (the VPU), bridged by the `TTCii` BlackBox (which hides the SV
// `tt_cii_interface`, its credit relay, and the VPU). Chisel can only bind flat
// `Bits` ports, so the BlackBox presents the four CII channels as flat, packed
// buses; the wrapper SV repacks them into the interface structs.
//
// Responsibilities (filled in across B1-B5; B0 is the tied-off skeleton):
//   B1  issue    : advertise fu_types on issue credit; on grant, tag + emit the
//                  extended issue packet (insn + vtype/vl/vxrm/frm); side-table.
//   B2  operand  : service Src-Request -> VRF/scalar read -> Src-Data (in order).
//   B3  writeback: place wb beats (VRF vector-dest, or INT/FP scalar-dest);
//                  on `last`, emit group_done + clr_rob.
//   B5  flush    : kill quiesces the adapter (past-PNR issue => no rollback).
//
// Wired in core.scala exactly like VecLSU (group_done -> vec wakeups, clr_rob ->
// rob.vec_clr_bsy, kill = RegNext(rob.flush)). Instantiated ONLY under
// usingVectorArith, so with the flag off there is zero elaboration / RTL impact.

package boom.v4.vec.cii

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.vec.rename.VecGroupDone

/** BlackBox over `tt_cii_host_wrap.sv`: flat host-side view of the four CII
  * channels (multi-lane fields are packed lane-major UInts), plus clk/rst_n.
  * Port names here MUST match the SV wrapper's port list exactly. Widths that
  * depend on core parameters (`vLen` = VLEN, `vlSz` = VL field width) are passed
  * as explicit Ints -- a raw BlackBox cannot mix in HasBoomCoreParameters. */
class TTCii(vLen: Int, vlSz: Int) extends BlackBox with HasBlackBoxResource
{
  override def desiredName = "tt_cii_host_wrap"   // must match the SV module name
  import CiiConsts._
  val io = IO(new Bundle {
    val clk   = Input(Clock())
    val rst_n = Input(Bool())

    // Instruction issue  (host -> cop): SV inputs, iss_credit is the SV output.
    val iss_valid  = Input(Bool())
    val iss_tag    = Input(UInt(TAG_W.W))
    val iss_insn   = Input(UInt(32.W))
    val iss_vtype  = Input(UInt(VTYPE_W.W))
    val iss_vl     = Input(UInt(vlSz.W))
    val iss_vstart = Input(UInt(vlSz.W))
    val iss_vxrm   = Input(UInt(VXRM_W.W))
    val iss_frm    = Input(UInt(FRM_W.W))
    val iss_hint   = Input(UInt(NUM_SRC_SLOTS.W))
    val iss_credit = Output(Bool())

    // Source-operand request  (cop -> host): SV outputs, req_credit is SV input.
    val req_valid     = Output(Bool())
    val req_tag       = Output(UInt((NUM_SRC_REQ * TAG_W).W))
    val req_op_id     = Output(UInt((NUM_SRC_REQ * SRCID_W).W))
    val req_op_offset = Output(UInt((NUM_SRC_REQ * MEMBER_W).W))
    val req_credit    = Input(Bool())

    // Source-operand data  (host -> cop): SV inputs, dat_credit is SV output.
    val dat_valid  = Input(Bool())
    val dat_data   = Input(UInt((NUM_SRC_DAT_RSP * vLen).W))
    val dat_credit = Output(Bool())

    // Result writeback  (cop -> host): SV outputs, wb_credit is SV input.
    val wb_valid      = Output(Bool())
    val wb_tag        = Output(UInt((NUM_DST_WB * TAG_W).W))
    val wb_data       = Output(UInt((NUM_DST_WB * vLen).W))
    val wb_dst_offset = Output(UInt((NUM_DST_WB * MEMBER_W).W))
    val wb_wr_en      = Output(UInt(NUM_DST_WB.W))
    val wb_status     = Output(UInt((NUM_DST_WB * WB_STATUS_W).W))
    val wb_credit     = Input(Bool())
  })

  // The flatten wrapper + the TT-CII relay stack it instantiates. The .sv are
  // symlinks under resources/vsrc -> ../sv/v4/tt-cii/src (the vendored source of
  // truth). The .svh package is include-only (found via the gen-collateral
  // incdir; excluded from the compile filelist). TRACK C adds the VPU tree here.
  addResource("/vsrc/tt_cii_caracal_pkg.svh")
  addResource("/vsrc/rv_async_rst_dff.sv")
  addResource("/vsrc/tt_cii_fifo.sv")
  addResource("/vsrc/tt_cii_channel.sv")
  addResource("/vsrc/tt_cii_interface.sv")
  addResource("/vsrc/tt_cii.sv")
  addResource("/vsrc/tt_cii_host_wrap.sv")

  // ---- Track C (C6): the VPU coprocessor tree, instantiated inside the bridge
  // as tt_vpu_cii_wrapper_top on iface_B. All .sv are symlinks under
  // resources/vsrc -> the vendored sv/v4/{vpu,common} + resources/HardFloat.
  // .svh/.h/.vi are include-only (gen-collateral incdir; filtered off the
  // compile list -- .vi needs the common.mk patch). HardFloat modules are raw
  // lowercase names (addRecFN, mulAddRecFN, ...) that do NOT collide with
  // rocketchip's Chisel-mangled CamelCase HardFloat (MulAddRecFNPipe_l2_e11_s53).
  // common util + arithmetic
  addResource("/vsrc/tt_cam_buffer.sv"); addResource("/vsrc/tt_compare.sv")
  addResource("/vsrc/tt_ffs.sv"); addResource("/vsrc/tt_fifo.sv")
  addResource("/vsrc/tt_pipe_stage.sv"); addResource("/vsrc/tt_popcnt.sv")
  addResource("/vsrc/tt_reshape.sv"); addResource("/vsrc/tt_rts_rtr_pipe_stage.sv")
  addResource("/vsrc/tt_skid_buffer.sv")
  addResource("/vsrc/tt_fp16_div.sv"); addResource("/vsrc/tt_fp32_div.sv")
  addResource("/vsrc/tt_int_div_r2.sv"); addResource("/vsrc/tt_int_div_simple.sv")
  addResource("/vsrc/VecFP16rec7.sv"); addResource("/vsrc/VecFP16rsqrt7.sv")
  addResource("/vsrc/VecFP32rec7.sv"); addResource("/vsrc/VecFP32rsqrt7.sv")
  // HardFloat (raw Verilog; .vi macros are include-only)
  addResource("/vsrc/addRecFN.v"); addResource("/vsrc/compareRecFN.v")
  addResource("/vsrc/divSqrtRecFN_small.v"); addResource("/vsrc/fNToRecFN.v")
  addResource("/vsrc/HardFloat_primitives.v"); addResource("/vsrc/HardFloat_rawFN.v")
  addResource("/vsrc/iNToRecFN.v"); addResource("/vsrc/isSigNaNRecFN.v")
  addResource("/vsrc/mulAddRecFN.v"); addResource("/vsrc/mulRecFN.v")
  addResource("/vsrc/recFNToFN.v"); addResource("/vsrc/recFNToIN.v")
  addResource("/vsrc/recFNToRecFN.v"); addResource("/vsrc/HardFloat_specialize.v")
  addResource("/vsrc/HardFloat_consts.vi"); addResource("/vsrc/HardFloat_localFuncs.vi")
  addResource("/vsrc/HardFloat_specialize.vi")
  // VPU decoder (+ .h/.svh include-only)
  addResource("/vsrc/tt_briscv_pkg.svh")
  addResource("/vsrc/autogen_defines.h"); addResource("/vsrc/briscv_defines.h")
  addResource("/vsrc/autogen_riscv_imabfv.v"); addResource("/vsrc/tt_ascii_instrn_decode.sv")
  addResource("/vsrc/tt_decoded_mux.sv"); addResource("/vsrc/tt_decoder.sv")
  addResource("/vsrc/tt_id.sv")
  // VPU execution units
  addResource("/vsrc/tt_vfp_encoder_lane.sv"); addResource("/vsrc/tt_vfp_encoder.sv")
  addResource("/vsrc/tt_vfp_ex_unit.sv"); addResource("/vsrc/tt_vfp_fma.sv")
  addResource("/vsrc/tt_vfp_lane.sv"); addResource("/vsrc/tt_vfp_red.sv")
  addResource("/vsrc/tt_vfp_unit.sv")
  addResource("/vsrc/tt_vec_div_unit.sv"); addResource("/vsrc/tt_vec_iadd.sv")
  addResource("/vsrc/tt_vec_idp.sv"); addResource("/vsrc/tt_vec_imul.sv")
  addResource("/vsrc/tt_vec_mul_dp.sv")
  // VPU regfile + top + CII wrapper
  addResource("/vsrc/tt_vec_regfile.sv"); addResource("/vsrc/tt_vec_top.sv")
  addResource("/vsrc/tt_vpu_cii_wrapper_top.sv")
}

class VecCiiHost(implicit p: Parameters) extends BoomModule
{
  import CiiConsts._

  val io = IO(new Bundle {
    // Granted IQ_V_ALU head (fire-and-forget Valid from the vector ALU issue unit).
    val iss_uop = Flipped(Valid(new MicroOp))
    // Advertised to the issue unit's fu_types: "a CII issue credit is available".
    // Registered (see credit counter) to avoid a fu_types->grant->iss comb loop.
    val fu_rdy  = Output(Bool())

    // VRF read ports (operand pull): CII read ports 5,6. Registered-address read.
    val vrf_read = Vec(NUM_SRC_REQ, new Bundle {
      val req_addr  = Output(UInt(vecPregSz.W))
      val resp_data = Input(UInt(vecVLen.W))
    })
    // VL register-file read: pvl -> vl at issue.
    val vl_read = new Bundle {
      val req_addr  = Output(UInt(vlPregSz.W))
      val resp_data = Input(UInt(vecVLSz.W))
    }
    // CSR reads used to fill the issue payload (vxrm/frm) and vstart.
    val csr_vxrm   = Input(UInt(VXRM_W.W))
    val csr_frm    = Input(UInt(FRM_W.W))
    val csr_vstart = Input(UInt(vecVLSz.W))
    // Scalar (.vx/.vf) operand value captured at issue (from INT/FP RF bypass).
    val scalar_rs1 = Input(UInt(xLen.W))

    // VRF write port (vector-dest results): CII write port 1. Per-byte mask.
    val vrf_write = Valid(new Bundle {
      val addr = UInt(vecPregSz.W)
      val data = UInt(vecVLen.W)
      val mask = UInt((vecVLen / 8).W)
    })
    // Scalar-dest results (vmv.x.s / vfmv.f.s / vcpop.m / vfirst.m).
    val scalar_wb = Valid(new Bundle {
      val rob_idx  = UInt(robAddrSz.W)
      val data     = UInt(xLen.W)
      val to_fp    = Bool()                 // false=INT RF, true=FP RF
    })

    // Completion: one group-done per vector-dest OP.v + a single ROB busy-clear.
    val group_done = Valid(new VecGroupDone)
    val clr_rob    = Valid(UInt(robAddrSz.W))

    val kill = Input(Bool())
    val busy = Output(Bool())
  })

  // ---- SV bridge -----------------------------------------------------------
  val bb = Module(new TTCii(vecVLen, vecVLSz))
  bb.io.clk   := clock
  bb.io.rst_n := !reset.asBool   // BOOM reset is sync active-high; tt_cii wants active-low

  // Preserve every CII interface port across firtool: many are tied to
  // constants in B0 (and stay lightly-driven through later steps), so without
  // dontTouch they could be constant-folded / pruned -- desyncing the BlackBox
  // port binding to the SV wrapper and hiding them from waveforms. Mark the
  // whole four-channel boundary as dontTouch.
  dontTouch(bb.io)

  // =========================================================================
  // Issue path (B1): un-tie IQ_V_ALU. Advertise fu_rdy on issue credit; on a
  // grant, present pvl to the (registered) VL-RF read, then one cycle later emit
  // the extended issue packet {tag, insn, vtype, vl, vstart, vxrm, frm} and
  // record a tag side-table entry. The sIdle->sEmit stage covers the 1-cycle
  // VL-RF read latency. Issue is fire-and-forget, so fu_rdy is deasserted while a
  // grant is being consumed (and while emitting).
  // =========================================================================
  object IState extends ChiselEnum { val sIdle, sEmit = Value }
  val istate = RegInit(IState.sIdle)

  // Issue credit counter (host-held): +1 per returned iss_credit, -1 per emit.
  val iss_credit_cnt = RegInit(ISS_CREDITS.U(log2Ceil(ISS_CREDITS + 1).W))

  // Latched grant, waiting one cycle for the registered VL-RF read.
  val g_uop    = Reg(new MicroOp)
  val g_vxrm   = Reg(UInt(VXRM_W.W))
  val g_frm    = Reg(UInt(FRM_W.W))
  val g_vstart = Reg(UInt(vecVLSz.W))
  val g_scalar = Reg(UInt(xLen.W))
  val g_tag    = Reg(UInt(TAG_W.W))

  // Tag free-list (1 = free): allocate at issue emit, free on the wb `last`.
  // A monotonic counter could alias -- issue credits return at issue-accept, far
  // earlier than completion, so in-flight tags can exceed a wrap window.
  val free     = RegInit(((BigInt(1) << N_TAGS) - 1).U(N_TAGS.W))
  val free_tag = PriorityEncoder(free)
  val has_free = free.orR

  // Side-table: tag -> {rob_idx, dest/src groups, dst_rtype, is_shared}.
  val sidetable = Reg(Vec(N_TAGS, new CiiTagEntry))

  val grant   = (istate === IState.sIdle) && io.iss_uop.valid && !io.kill
  val do_emit = (istate === IState.sEmit) && !io.kill

  // VL-RF read: present pvl at the grant cycle; data valid next cycle (sEmit).
  io.vl_read.req_addr := io.iss_uop.bits.pvl

  switch (istate) {
    is (IState.sIdle) {
      when (grant) {
        g_uop    := io.iss_uop.bits
        g_vxrm   := io.csr_vxrm
        g_frm    := io.csr_frm
        // M2 stopgap: the vector vstart CSR is not yet architecturally maintained
        // (core.scala: "no architectural vstart/vxsat update"), so csr_vstart is
        // stale. M2 issues past-PNR and does not resume mid-instruction on a fault,
        // so vstart is always 0 for these ops. Forcing 0 avoids a stale non-zero
        // vstart making the VPU treat every element as pre-start (skip -> old dest).
        g_vstart := 0.U   // was: io.csr_vstart (stale; see above)
        g_scalar := io.scalar_rs1
        g_tag    := free_tag
        istate   := IState.sEmit
      }
    }
    is (IState.sEmit) { istate := IState.sIdle }
  }
  when (io.kill) { istate := IState.sIdle }

  // ---- issue beat (sEmit) --------------------------------------------------
  bb.io.iss_valid  := do_emit
  bb.io.iss_tag    := g_tag
  bb.io.iss_insn   := g_uop.debug_inst
  bb.io.iss_vtype  := Cat(g_uop.vconfig.vsew,    // {vsew[3],vlmul[3],vta,vma} = 8b,
                          g_uop.vconfig.vlmul,   // matches cii_caracal_vtype_t packed order
                          g_uop.vconfig.vta,
                          g_uop.vconfig.vma)
  bb.io.iss_vl     := io.vl_read.resp_data       // registered read of g_uop.pvl (addr driven last cycle)
  bb.io.iss_vstart := g_vstart
  bb.io.iss_vxrm   := g_vxrm
  bb.io.iss_frm    := g_frm
  bb.io.iss_hint   := 0.U                        // src_reuse ignored in M2

  when (do_emit) {
    val e = Wire(new CiiTagEntry)
    e.rob_idx         := g_uop.rob_idx
    e.pvdest_grp      := g_uop.pvdest_grp
    e.pvdest_grp_mask := g_uop.pvdest_grp_mask
    e.pvs1_grp        := g_uop.pvs1_grp
    e.pvs2_grp        := g_uop.pvs2_grp
    e.pvs3_grp        := g_uop.pvs3_grp
    e.pvm             := g_uop.pvm
    e.scalar          := g_scalar
    e.dst_rtype       := g_uop.dst_rtype
    e.is_shared       := g_uop.is_shared
    sidetable(g_tag)  := e
    // tag allocation (clear free bit) is handled in the free-list update below.
  }

  // Credit counter: +1 per returned credit, -1 per emitted issue beat.
  when (bb.io.iss_credit && !do_emit)      { iss_credit_cnt := iss_credit_cnt + 1.U }
  .elsewhen (!bb.io.iss_credit && do_emit) { iss_credit_cnt := iss_credit_cnt - 1.U }

  // fu_rdy: idle, credit available, and not already consuming a grant. Registered
  // to break the fu_types -> grant -> iss_valid combinational loop.
  io.fu_rdy := RegNext((istate === IState.sIdle) && (iss_credit_cnt > 0.U) && has_free &&
                       !io.iss_uop.valid && !io.kill, false.B)

  // =========================================================================
  // Operand pull (B2): serve coprocessor Src-Request from the VRF / side-table,
  // returning Src-Data IN REQUEST ORDER (per-beat, lane i -> lane i). Channel
  // handshake (from tt_cii_channel): the req channel is a NON-FWFT receiver --
  // the host asserts req_credit to pop, and the popped beat's req_valid+req_data
  // arrive registered ONE CYCLE LATER. VRF read is registered (1-cycle). The dat
  // channel is a sender (local credit counter).
  //   pIdle(assert req_credit 1 cyc) -> pPend(await popped beat, drive VRF addr)
  //   -> pCap(latch VRF/scalar data) -> pSend(emit dat when credit)
  //
  // !! OPEN (validate with the real coprocessor + sim): req_credit is asserted
  // once per FSM pass, so credit-return rate == drain rate (self-limiting). But
  // tt_cii_channel returns a credit for EVERY req_credit pulse, incl. popping an
  // empty FIFO (spurious idle credits), and this FSM drains ~1 beat / 4 cyc. A
  // sustained request burst larger than the credit FIFO depth could overflow
  // (do_push drops when full). Track C's coprocessor request behavior + a cosim
  // run must confirm this handshake / retune depth or throughput.
  // =========================================================================
  object PState extends ChiselEnum { val pIdle, pPend, pCap, pSend = Value }
  val pstate = RegInit(PState.pIdle)

  val dat_credit_cnt = RegInit(DAT_CREDITS.U(log2Ceil(DAT_CREDITS + 1).W))

  val l_isvec  = Reg(Vec(NUM_SRC_REQ, Bool()))
  val l_isscl  = Reg(Vec(NUM_SRC_REQ, Bool()))
  val l_scalar = Reg(Vec(NUM_SRC_REQ, UInt(xLen.W)))
  val d_lane   = Reg(Vec(NUM_SRC_DAT_RSP, UInt(vecVLen.W)))

  // per-lane req field extractors (valid the cycle bb.io.req_valid is high)
  def reqTag(i: Int) = bb.io.req_tag      ((i + 1) * TAG_W    - 1, i * TAG_W)
  def reqId (i: Int) = bb.io.req_op_id    ((i + 1) * SRCID_W  - 1, i * SRCID_W)
  def reqOff(i: Int) = bb.io.req_op_offset((i + 1) * MEMBER_W - 1, i * MEMBER_W)

  bb.io.req_credit := (pstate === PState.pIdle) && !io.kill

  // default VRF read addrs (overridden while serving a request beat)
  for (r <- io.vrf_read) { r.req_addr := 0.U }

  switch (pstate) {
    is (PState.pIdle) { pstate := PState.pPend }            // 1-cycle pop request
    is (PState.pPend) {
      when (bb.io.req_valid) {
        for (i <- 0 until NUM_SRC_REQ) {
          val ent  = sidetable(reqTag(i))
          val opid = reqId(i)
          val off  = reqOff(i)
          io.vrf_read(i).req_addr := MuxLookup(opid, 0.U)(Seq(
            CiiConsts.SRC_VS1.U -> ent.pvs1_grp(off),
            CiiConsts.SRC_VS2.U -> ent.pvs2_grp(off),
            CiiConsts.SRC_VS3.U -> ent.pvs3_grp(off),
            CiiConsts.SRC_VM.U  -> ent.pvm))
          l_isvec(i)  := (opid === CiiConsts.SRC_VS1.U) || (opid === CiiConsts.SRC_VS2.U) ||
                         (opid === CiiConsts.SRC_VS3.U) || (opid === CiiConsts.SRC_VM.U)
          l_isscl(i)  := opid === CiiConsts.SRC_SCALAR.U
          l_scalar(i) := ent.scalar
        }
        pstate := PState.pCap
      } .otherwise {
        pstate := PState.pIdle                              // pop hit empty FIFO; retry
      }
    }
    is (PState.pCap) {
      for (i <- 0 until NUM_SRC_REQ) {
        d_lane(i) := Mux(l_isvec(i), io.vrf_read(i).resp_data,
                     Mux(l_isscl(i), l_scalar(i).pad(vecVLen), 0.U))
      }
      pstate := PState.pSend
    }
    is (PState.pSend) {
      when (dat_credit_cnt > 0.U) { pstate := PState.pIdle }
    }
  }
  when (io.kill) { pstate := PState.pIdle }

  val dat_send = (pstate === PState.pSend) && (dat_credit_cnt > 0.U) && !io.kill
  bb.io.dat_valid := dat_send
  bb.io.dat_data  := d_lane.asUInt          // Vec.asUInt: lane 0 in the low bits

  when (bb.io.dat_credit && !dat_send)      { dat_credit_cnt := dat_credit_cnt + 1.U }
  .elsewhen (!bb.io.dat_credit && dat_send) { dat_credit_cnt := dat_credit_cnt - 1.U }

  // =========================================================================
  // Writeback + completion (B3). The wb channel is a NON-FWFT receiver like req:
  // assert wb_credit to pop; the beat's wb_valid+wb_data arrive registered next
  // cycle. Processing is single-cycle (one VRF/INT/FP write), so the host accepts
  // every cycle and never stalls -> no overflow.
  //   - vector-dest (dst_kind==VEC): write pvdest_grp(wb_dst_offset) on VRF write
  //     port 1, verbatim (the VPU already applied vta/vma). On `last`, emit
  //     group_done + clr_rob.
  //   - scalar-dest (vmv.x.s/vfmv.f.s/vcpop/vfirst): route to the INT/FP RF via
  //     scalar_wb. STRUCTURAL only -- needs the VDecode dst_rtype fix + rename
  //     handling (B3b) to work end-to-end.
  // group_done/clr_rob/scalar_wb are CONSUMED by the ROB/wakeups in B4 (needs the
  // numVecWbPorts/numVecWakeupPorts bump); here they are generated and (except
  // vrf_write, wired in core.scala) left unconsumed.
  // =========================================================================
  bb.io.wb_credit := !io.kill                          // always ready (1-cycle processing)
  val wb_v    = bb.io.wb_valid && !io.kill
  // Host consumes wb lane 0 only (bus is NUM_DST_WB lanes wide; lane 1 unused).
  val wb_tag  = bb.io.wb_tag(TAG_W - 1, 0)
  val wb_off  = bb.io.wb_dst_offset(MEMBER_W - 1, 0)
  val wb_datv = bb.io.wb_data(vecVLen - 1, 0)
  val wb_wren = bb.io.wb_wr_en(0)
  val wb_st   = bb.io.wb_status(WB_STATUS_W - 1, 0).asTypeOf(new CiiWbStatus)
  val went    = sidetable(wb_tag)
  val is_vecd = wb_st.dst_kind === CiiConsts.DST_VEC.U

  io.vrf_write.valid     := wb_v && wb_wren && is_vecd
  io.vrf_write.bits.addr := went.pvdest_grp(wb_off)
  io.vrf_write.bits.data := wb_datv
  io.vrf_write.bits.mask := Fill(vecVLen / 8, 1.U(1.W))   // full; VPU applied vta/vma

  io.scalar_wb.valid        := wb_v && wb_wren && !is_vecd
  io.scalar_wb.bits.rob_idx := went.rob_idx
  io.scalar_wb.bits.data    := wb_datv(xLen - 1, 0)
  io.scalar_wb.bits.to_fp   := wb_st.dst_kind === CiiConsts.DST_FP.U

  val wb_last = wb_v && wb_st.last
  io.group_done.valid     := wb_last && is_vecd
  io.group_done.bits.prn  := went.pvdest_grp
  io.group_done.bits.mask := went.pvdest_grp_mask
  io.clr_rob.valid := wb_last
  io.clr_rob.bits  := went.rob_idx

  // Tag free-list update: allocate at issue emit, free on wb `last`.
  val alloc_oh = Mux(do_emit, UIntToOH(g_tag, N_TAGS),  0.U(N_TAGS.W))
  val free_oh  = Mux(wb_last, UIntToOH(wb_tag, N_TAGS), 0.U(N_TAGS.W))
  free := (free & (~alloc_oh).asUInt) | free_oh

  io.busy := (istate =/= IState.sIdle) || (pstate =/= PState.pIdle)

  dontTouch(sidetable)
}

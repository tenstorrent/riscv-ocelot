//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector LS register-read stage (Step 11a.2, sub-step B)
//------------------------------------------------------------------------------
//
// Tiny operand-read front-end for the vector load path. When the V-LOAD issue
// unit grants a vle (Valid, fire-and-forget), this stage reads the integer base
// address (rs1) from a DEDICATED usingRVV integer-regfile read port and the
// effective vl from the VL register file, then drives VecLsDecode -> AGEN.start
// for ONE instruction. It advertises a single `fu_ready` bit (gated on the whole
// vector-LS pipe being idle, all REGISTERED state -> no comb loop with the issue
// unit's grant) so the issue unit only grants when this stage and VecLSU/AGEN are
// free; iss_uops is Valid (no back-pressure), so a grant must never be dropped.
//
// M1 scope: unit-stride vle only. rs2 (stride) is NOT read -- VecLsDecode forces
// the implied unit stride. vstart is 0 (M1 has no fault-only-first resume), so the
// architectural vstart CSR is not consulted here.
//
// Read timing: the integer RF read is the banked/arbitrated 2-stage read (drive
// arb_read_reqs addr + valid; when ready, data lands in rrd_read_resps the NEXT
// cycle). The VL RF read is a registered-address read (drive addr; data next
// cycle). Both addrs are driven in sReq, so in sRrd both responses are valid.

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

class VecLSRegRead(implicit p: Parameters) extends BoomModule with VecLsConstants
{
  // matches core.scala's numIrfWritePorts = aluWidth + lsuWidth + 1.
  val numIrfWritePorts = aluWidth + lsuWidth + 1

  val io = IO(new Bundle {
    // V-LOAD issue grant (Valid, fire-and-forget).
    val iss = Input(Valid(new MicroOp))
    // single FU-ready bit advertised to the V-LOAD issue unit's fu_types.
    val fu_ready = Output(Bool())

    // dedicated integer-RF read port (drive arb req; data next cycle).
    val irf_req  = DecoupledIO(UInt(log2Ceil(numIntPhysRegs).W))
    val irf_resp = Input(UInt(xLen.W))
    // int writeback ports (same signals that write iregfile). The int RF read is
    // a registered read of a Mem with NO read-during-write bypass, and the V-LOAD
    // is woken SPECULATIVELY -- so a base/stride GPR produced 0-1 instrs before the
    // vle can be written the SAME cycle this stage reads it, and the Mem returns
    // the stale (pre-write) value. Snoop the write ports and forward on a match
    // (timing-aligned: the Mem write and this bypass both use the write bus this
    // cycle; the Mem only reflects it next cycle).
    val irf_wb   = Input(Vec(numIrfWritePorts, Valid(new Bundle {
      val addr = UInt(maxPregSz.W)                 // matches iregfile write-port addr / uop.prs1
      val data = UInt(xLen.W)
    })))

    // VL register-file read (registered address).
    val vl_addr = Output(UInt(vlPregSz.W))
    val vl_data = Input(UInt(vecVLSz.W))

    // Vector mask (v0) read for masked LS: registered-address VecRegFile read of
    // pvm. The low MASK_W bits (one bit per element) feed the AGEN mask stream.
    val vrf_mask_addr = Output(UInt(vecPregSz.W))
    val vrf_mask_data = Input(UInt(vecVLen.W))

    // pipe-busy feedback (registered) from the downstream AGEN + VecLSU.
    val agen_active = Input(Bool())
    val lsu_busy    = Input(Bool())

    // driven to VecLsDecode.io.in; AGEN latches it when start fires.
    val dec = Output(new Bundle {
      val valid    = Bool()
      val uop      = new MicroOp()
      val rs1_data = UInt(xLen.W)
      val rs2_data = UInt(xLen.W)               // stride (vlse/vsse); ignored otherwise
      val vl       = UInt(vecVLSz.W)
      val vstart   = UInt(VSTART_W.W)
      val mask     = UInt(MASK_W.W)             // v0[MASK_W-1:0], one bit per element
    })
    // AGEN.start.fire -- tells us VecLsDecode->AGEN latched this instruction.
    val agen_start_fire = Input(Bool())

    val kill = Input(Bool())
  })

  // rs1 (base) and rs2 (stride) are read serially on the single dedicated int-RF
  // port: sReq1 reads rs1, sReq2 reads rs2. rs1's data lands the cycle after its
  // read fires (= the first sReq2 cycle), latched into rs1_q; rs2's data lands in
  // sRrd. Serial avoids a 2nd int-RF read port (no regfile resize) and the arb
  // bank-conflict timing of two parallel reads.
  object State extends ChiselEnum {
    val sIdle, sReq1, sReq2, sRrd = Value
  }
  val state  = RegInit(State.sIdle)
  val rr_uop = Reg(new MicroOp)
  val rs1_q  = Reg(UInt(xLen.W))

  // pipe idle => safe to grant another vle. All terms are registered.
  val pipe_idle = (state === State.sIdle) && !io.agen_active && !io.lsu_busy
  io.fu_ready := pipe_idle && !io.kill

  // Writeback bypass: return the write-port data instead of the (stale) Mem read
  // when a same-cycle int writeback targets the physical reg being read. `preg`
  // is the physical reg whose read data is landing THIS cycle (prs1 in sReq2,
  // prs2 in sRrd -- both one cycle after their addr was driven).
  def bypassed(preg: UInt, rf_data: UInt): UInt = {
    val hit  = io.irf_wb.map(w => w.valid && (w.bits.addr === preg))
    val data = Mux1H(hit, io.irf_wb.map(_.bits.data))
    Mux(hit.reduce(_ || _), data, rf_data)
  }

  // latch rs1 the cycle its data is valid (one cycle after sReq1 fired), applying
  // the same-cycle writeback bypass so a just-produced base is not read stale.
  when (RegNext(state === State.sReq1 && io.irf_req.fire, false.B)) {
    rs1_q := bypassed(rr_uop.prs1, io.irf_resp)
  }

  // defaults
  io.irf_req.valid := false.B
  io.irf_req.bits  := Mux(state === State.sReq2, rr_uop.prs2, rr_uop.prs1)
  io.vl_addr       := rr_uop.pvl
  io.vrf_mask_addr := rr_uop.pvm                 // read v0 (registered; data valid in sRrd)
  io.dec.valid     := false.B
  io.dec.uop       := rr_uop
  io.dec.rs1_data  := rs1_q
  io.dec.rs2_data  := bypassed(rr_uop.prs2, io.irf_resp)  // rs2 data live in sRrd
  io.dec.vl        := io.vl_data
  io.dec.vstart    := 0.U                       // M1: unit-stride starts at element 0
  io.dec.mask      := io.vrf_mask_data(MASK_W - 1, 0)

  switch (state) {
    is (State.sIdle) {
      when (io.iss.valid && pipe_idle) {
        rr_uop := io.iss.bits
        state  := State.sReq1
      }
    }
    is (State.sReq1) {
      // read rs1 (base). VL-RF + mask reads run in parallel (addrs held from now).
      io.irf_req.valid := true.B
      when (io.kill) {
        state := State.sIdle
      } .elsewhen (io.irf_req.fire) {
        state := State.sReq2
      }
    }
    is (State.sReq2) {
      // read rs2 (stride). rs1 data was latched into rs1_q this cycle.
      io.irf_req.valid := true.B
      when (io.kill) {
        state := State.sIdle
      } .elsewhen (io.irf_req.fire) {
        state := State.sRrd
      }
    }
    is (State.sRrd) {
      // all reads' data valid now -> present the decoded instruction.
      io.dec.valid := !io.kill
      when (io.kill) {
        state := State.sIdle
      } .elsewhen (io.agen_start_fire) {
        state := State.sIdle
      }
    }
  }

  when (io.kill) { state := State.sIdle }
}

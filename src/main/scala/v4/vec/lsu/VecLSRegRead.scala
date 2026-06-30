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
  val io = IO(new Bundle {
    // V-LOAD issue grant (Valid, fire-and-forget).
    val iss = Input(Valid(new MicroOp))
    // single FU-ready bit advertised to the V-LOAD issue unit's fu_types.
    val fu_ready = Output(Bool())

    // dedicated integer-RF read port (drive arb req; data next cycle).
    val irf_req  = DecoupledIO(UInt(log2Ceil(numIntPhysRegs).W))
    val irf_resp = Input(UInt(xLen.W))

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
      val vl       = UInt(vecVLSz.W)
      val vstart   = UInt(VSTART_W.W)
      val mask     = UInt(MASK_W.W)             // v0[MASK_W-1:0], one bit per element
    })
    // AGEN.start.fire -- tells us VecLsDecode->AGEN latched this instruction.
    val agen_start_fire = Input(Bool())

    val kill = Input(Bool())
  })

  object State extends ChiselEnum {
    val sIdle, sReq, sRrd = Value
  }
  val state  = RegInit(State.sIdle)
  val rr_uop = Reg(new MicroOp)

  // pipe idle => safe to grant another vle. All terms are registered.
  val pipe_idle = (state === State.sIdle) && !io.agen_active && !io.lsu_busy
  io.fu_ready := pipe_idle && !io.kill

  // defaults
  io.irf_req.valid := false.B
  io.irf_req.bits  := rr_uop.prs1
  io.vl_addr       := rr_uop.pvl
  io.vrf_mask_addr := rr_uop.pvm                 // read v0 (registered; data valid in sRrd)
  io.dec.valid     := false.B
  io.dec.uop       := rr_uop
  io.dec.rs1_data  := io.irf_resp
  io.dec.vl        := io.vl_data
  io.dec.vstart    := 0.U                       // M1: unit-stride starts at element 0
  io.dec.mask      := io.vrf_mask_data(MASK_W - 1, 0)

  switch (state) {
    is (State.sIdle) {
      when (io.iss.valid && pipe_idle) {
        rr_uop := io.iss.bits
        state  := State.sReq
      }
    }
    is (State.sReq) {
      // drive the int-RF arb read + the VL-RF read address; advance when the
      // arbitrated read is accepted (data then lands next cycle).
      io.irf_req.valid := true.B
      when (io.kill) {
        state := State.sIdle
      } .elsewhen (io.irf_req.ready) {
        state := State.sRrd
      }
    }
    is (State.sRrd) {
      // both reads' data are valid now -> present the decoded instruction.
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

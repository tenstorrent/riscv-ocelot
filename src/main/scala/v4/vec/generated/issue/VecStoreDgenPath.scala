/*
Tenstorrent CONFIDENTIAL
__________________
 Tenstorrent Inc.
 All Rights Reserved.

NOTICE:  All information contained herein is, and remains
the property of Tenstorrent Inc.  The intellectual
and technical concepts contained
herein are proprietary to Tenstorrent Inc.
and may be covered by U.S., Canadian and Foreign Patents,
patents in process, and are protected by trade secret or copyright law.
Dissemination of this information or reproduction of this material
is strictly forbidden unless prior written permission is obtained
from Tenstorrent Inc.
*/

package boom.v4.vec.generated.issue

import chisel3._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.vec.generated.VecTrace

// GENERATED from src/main/nlhdl/vec/issue/VecStoreDgenPath.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecStoreDgenPathIO(implicit p: Parameters) extends BoomBundle
{
  private val uopT = new MicroOp

  // ---- Inputs from the slot ----
  val slot_valid          = Input(Bool())
  val grant                = Input(Bool())
  val squash_grant         = Input(Bool())
  val slot_uop             = Input(new MicroOp)
  val agen_operands_ready  = Input(Bool())
  val dgen_operand_ready   = Input(Bool())
  val agen_rebusied        = Input(Bool())

  // ---- Outputs to the slot: the operand mux ----
  val dgen_operand          = Output(uopT.pvs3.get.cloneType)
  val dgen_operand_members  = Output(uopT.v_emul.get.cloneType)
  val dgen_operand_busy     = Output(Bool())
  val dgen_operand_is_pvtmp = Output(Bool())

  // ---- Outputs to the slot: path sequencing ----
  val agen_request         = Output(Bool())
  val dgen_request         = Output(Bool())
  val iss_fu_code_agen     = Output(Bool())
  val iss_fu_code_dgen     = Output(Bool())
  val next_fu_code_agen    = Output(Bool())
  val next_fu_code_dgen    = Output(Bool())
  val issued_partial_agen  = Output(Bool())
  val issued_partial_dgen  = Output(Bool())
  val keep_valid           = Output(Bool())
  val both_paths_done      = Output(Bool())
}

/**
 * VecStoreDgenPath: vector STORE slot's second grant path (AGEN/DGEN path sequencing and operand mux).
 */
class VecStoreDgenPath(implicit p: Parameters) extends BoomModule
{
  val io = IO(new VecStoreDgenPathIO)

  val uop = io.slot_uop

  // =========================================================================
  // ---- 1. Where the two grant bits live, and why not here ----
  // =========================================================================
  //
  //@req-spec-core.f13
  //@req-spec-issue.d10
  //@req-spec-issue.d13
  val fu_agen = uop.fu_code(FC_AGEN)
  val fu_dgen = uop.fu_code(FC_DGEN)

  // =========================================================================
  // ---- 2. Offer order: AGEN strictly before DGEN ----
  // =========================================================================
  //
  //@req-spec-issue.d11
  //@req-spec-issue.c13
  val agen_request = io.slot_valid && !uop.iw_issued && fu_agen && io.agen_operands_ready

  // =========================================================================
  // ---- 3. What DGEN is gated on ----
  // =========================================================================
  //
  //@req-spec-issue.g22
  //@req-spec-issue.g23
  //@req-spec-issue.g24
  //@req-spec-issue.c12
  //@req-spec-cii.i12
  val dgen_request = io.slot_valid && !uop.iw_issued && fu_dgen && !fu_agen &&
    !io.dgen_operand_busy && io.dgen_operand_ready

  // =========================================================================
  // ---- 4. The operand mux — the M1 bug ----
  // =========================================================================
  //
  //@req-spec-issue.g25
  //@req-spec-issue.g26
  //@req-spec-issue.g27
  //@req-spec-issue.g28
  //@req-spec-cii.i6
  /* WARNING — do not "simplify" this to pvs3. Gating DGEN unconditionally on
     pvs3 is incorrect for a segmented store and earlier Caracal drafts did
     exactly that. For such an op pvs3 is the COPROCESSOR half's source group
     and is ready long before the transpose has run, while the LSU half's
     data source is pvtmp, written BY the coprocessor half. A DGEN woken on
     pvs3 reads pvtmp before it exists and stores garbage; and pvs3's
     group-done never arrives at this slot at all, so the chain deadlocks.
     Whenever is_shared is set, DGEN wakes on pvtmp's group-done and on
     nothing else. */
  io.dgen_operand          := Mux(uop.is_shared.get, uop.pvtmp.get, uop.pvs3.get)
  io.dgen_operand_busy     := Mux(uop.is_shared.get, uop.pvtmp_busy.get, uop.pvs3_busy.get)
  io.dgen_operand_is_pvtmp := uop.is_shared.get
  io.dgen_operand_members  := uop.v_emul.get

  io.agen_request := agen_request
  io.dgen_request := dgen_request

  // =========================================================================
  // ---- Path naming for the issued uop ----
  // =========================================================================
  //
  //@req-spec-issue.d11
  io.iss_fu_code_agen := agen_request
  io.iss_fu_code_dgen := dgen_request

  // =========================================================================
  // ---- 6. Capturing a grant ----
  // =========================================================================
  //
  //@req-spec-issue.d13
  val grantedThisCycle = io.grant && !io.squash_grant
  io.issued_partial_agen := grantedThisCycle && agen_request
  io.issued_partial_dgen := grantedThisCycle && dgen_request

  val agenMarker = uop.iw_issued_partial_agen
  val dgenMarker = uop.iw_issued_partial_dgen

  val next_fu_code_agen_w = WireInit(fu_agen)
  val next_fu_code_dgen_w = WireInit(fu_dgen)
  //@req-spec-issue.d13
  when (agenMarker && !io.agen_rebusied) {
    next_fu_code_agen_w := false.B
    next_fu_code_dgen_w := true.B
  }
  io.next_fu_code_agen := next_fu_code_agen_w
  io.next_fu_code_dgen := next_fu_code_dgen_w

  io.keep_valid      := agenMarker
  io.both_paths_done := dgenMarker

  // =========================================================================
  // ---- 7. Tolerating an unbounded AGEN-to-DGEN gap ----
  // =========================================================================
  //
  //@req-spec-issue.d12

  // =========================================================================
  // ---- 8. Assertions (synthesizable, on hardware conditions) ----
  // =========================================================================

  assert(!(io.issued_partial_agen && io.issued_partial_dgen),
    "VecStoreDgenPath: both partial markers set in the same cycle")
  assert(!(io.agen_request && io.dgen_request),
    "VecStoreDgenPath: both paths offered in the same cycle")
  //@req-spec-issue.c13
  assert(!(io.dgen_request && fu_agen),
    "VecStoreDgenPath: DGEN offered while AGEN bit still set")
  assert(!(io.issued_partial_dgen && fu_agen),
    "VecStoreDgenPath: DGEN granted while AGEN bit still set")
  assert(!io.agen_request || io.agen_operands_ready,
    "VecStoreDgenPath: agen_request asserted without agen_operands_ready")
  assert(!(io.slot_valid && !uop.iw_issued && !fu_agen && !fu_dgen),
    "VecStoreDgenPath: a live, un-issued store slot advertises neither FC_AGEN nor FC_DGEN")
  assert(!io.dgen_request || (io.dgen_operand_members =/= 0.U),
    "VecStoreDgenPath: dgen_request asserted against an empty operand group")
  assert(!(io.grant && !io.slot_valid),
    "VecStoreDgenPath: grant asserted against an invalid slot")

  // =========================================================================
  // ---- 9. Tracing ----
  // =========================================================================
  when (io.issued_partial_agen) {
    VecTrace.trace("VecStoreDgenPath", "agen_grant", uop)
  }
  when (io.issued_partial_dgen) {
    VecTrace.trace("VecStoreDgenPath", "dgen_grant", uop)
  }
  // SPEC DEFECT (reported, not resolved) -- `dgen_operand_select` trace event
  // omitted; cannot fire "on slot fill" without a register (forbidden by module design).
}

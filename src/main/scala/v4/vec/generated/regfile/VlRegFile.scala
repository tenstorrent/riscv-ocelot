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

package boom.v4.vec.generated.regfile

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common.{BoomBundle, BoomModule}
import boom.v4.vec.generated.VecTrace

// GENERATED from src/main/nlhdl/vec/regfile/VlRegFile.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

/**
 * VlRegFileWriteData -- the `addr`/`data` pair every write port's `Valid`
 * bundle carries. `addr` is the destination `pvl` (a VL physical register
 * number allocated by `vl_rename`); `data` is the new VL value.
 */
class VlRegFileWriteData(implicit p: Parameters) extends BoomBundle
{
  val addr = UInt(vlPregSz.W)
  val data = UInt(vecVLSz.W)
}

/**
 * VlRegFileReadPort -- a combinational read port: `addr` is presented this
 * cycle, `data` is valid the SAME cycle. No `valid`, no `ready`, no enable,
 * no output flop.
 */
class VlRegFileReadPort(implicit p: Parameters) extends BoomBundle
{
  val addr = Input(UInt(vlPregSz.W))
  val data = Output(UInt(vecVLSz.W))
}

/**
 * VlRegFileIO -- the full port list. Write ports are a fixed table, one
 * class per producer, never an arbiter (decode.c27); read ports are
 * combinational and are the only way a VL value leaves this module
 * (decode.i17/i18, rename.h25).
 *
 * `numAluWritePorts`, `numLsuWritePorts` and `numExeReadPorts` are computed
 * once by the enclosing `VlRegFile` module and threaded through here so the
 * IO bundle and the module body can never disagree on a port count.
 */
class VlRegFileIO(numAluWritePorts: Int, numLsuWritePorts: Int, numExeReadPorts: Int)
                  (implicit p: Parameters) extends BoomBundle
{
  // ---- Write ports (statically partitioned, one class per producer) ----

  //@req-spec-decode.c27

  //@req-spec-decode.c29
  val w_ren = Input(Vec(coreWidth, Valid(new VlRegFileWriteData)))

  //@req-spec-decode.c28
  val w_alu = Input(Vec(numAluWritePorts, Valid(new VlRegFileWriteData)))

  val w_lsu = Input(Vec(numLsuWritePorts, Valid(new VlRegFileWriteData)))

  // ---- Read ports ----
  val r_exe    = Vec(numExeReadPorts, new VlRegFileReadPort)
  val r_commit = new VlRegFileReadPort
}

class VlRegFile(implicit p: Parameters) extends BoomModule
{
  // ---- Port-count parameters (all derived, none a literal chosen here) ----

  //@req-spec-decode.c28
  val numAluWritePorts: Int = aluWidth

  val numLsuWritePorts: Int = 1

  val numExeReadPorts: Int = 3 * vectorParams.vecIssueGrantWidth

  val io = IO(new VlRegFileIO(numAluWritePorts, numLsuWritePorts, numExeReadPorts))

  // =========================================================================
  // ---- Storage ----
  // =========================================================================

  //@req-spec-decode.i1
  //@req-spec-rename.h4
  //@req-spec-vrf.c1
  val vl_rf = RegInit(VecInit(Seq.fill(numVlPhysRegs)(0.U(vecVLSz.W))))

  require(vecVLSz >= log2Ceil(maxVecVL + 1),
    s"VlRegFile: vecVLSz ($vecVLSz) is too narrow to hold maxVecVL ($maxVecVL)")

  // =========================================================================
  // ---- The rename-cycle write (`vsetivli`) ----
  // =========================================================================

  //@req-spec-decode.c4
  //@req-spec-issue.h4
  for (w <- 0 until coreWidth) {
    when (io.w_ren(w).valid) {
      vl_rf(io.w_ren(w).bits.addr) := io.w_ren(w).bits.data
      VecTrace.traceStruct("VlRegFile", "wr_ren",
        Seq(("prn", io.w_ren(w).bits.addr), ("vl", io.w_ren(w).bits.data)))
    }
  }

  // =========================================================================
  // ---- The ALU writeback (`vsetvli` / `vsetvl`) ----
  // =========================================================================

  for (w <- 0 until numAluWritePorts) {
    when (io.w_alu(w).valid) {
      vl_rf(io.w_alu(w).bits.addr) := io.w_alu(w).bits.data
      VecTrace.traceStruct("VlRegFile", "wr_alu",
        Seq(("prn", io.w_alu(w).bits.addr), ("vl", io.w_alu(w).bits.data)))
    }
  }

  // =========================================================================
  // ---- The `vleff` writeback ----
  // =========================================================================

  //@req-spec-lsu.g6
  for (w <- 0 until numLsuWritePorts) {
    when (io.w_lsu(w).valid) {
      vl_rf(io.w_lsu(w).bits.addr) := io.w_lsu(w).bits.data
      VecTrace.traceStruct("VlRegFile", "wr_lsu",
        Seq(("prn", io.w_lsu(w).bits.addr), ("vl", io.w_lsu(w).bits.data)))
    }
  }

  val allWritePorts: Seq[Valid[VlRegFileWriteData]] = io.w_ren ++ io.w_alu ++ io.w_lsu
  for (i <- allWritePorts.indices; j <- (i + 1) until allWritePorts.length) {
    assert(!(allWritePorts(i).valid && allWritePorts(j).valid) ||
           (allWritePorts(i).bits.addr =/= allWritePorts(j).bits.addr),
      s"VlRegFile: write ports $i and $j targeted the same pvl in the same cycle")
  }

  // =========================================================================
  // ---- The read side is the ONLY way VL leaves this module ----
  // =========================================================================

  //@req-spec-decode.i17
  //@req-spec-decode.i18
  //@req-spec-rename.h25

  //@req-spec-decode.i7
  //@req-spec-rename.h23
  for (i <- 0 until numExeReadPorts) {
    io.r_exe(i).data := vl_rf(io.r_exe(i).addr)
  }

  //@req-spec-lsu.g8
  io.r_commit.data := vl_rf(io.r_commit.addr)
}

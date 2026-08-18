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

// GENERATED from src/main/nlhdl/vec/regfile/VecRegFile.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
//
//@req-spec-core.d4

//@req-spec-vrf.g1
//@req-spec-cii.f29
object VrfPort {
  //@req-spec-vrf.g18
  //@req-spec-vrf.g19
  val R_LD_IDX      = 0
  val R_LD_MASK     = 1
  val R_LD_STALE    = 2
  val R_ST_DATA     = 3
  val R_ST_MASK_IDX = 4

  //@req-spec-vrf.g10
  //@req-spec-vrf.h3
  //@req-spec-cii.f25
  val R_CII_BASE    = 5

  val W_LD_LCB0 = 0
  val W_LD_LCB1 = 1

  //@req-spec-vrf.g11
  //@req-spec-vrf.h4
  //@req-spec-cii.f26
  //@req-spec-vrf.g17
  //@req-spec-cii.g2
  //@req-spec-vrf.f3
  //@req-spec-vrf.g20
  def W_CII(lsuWidth: Int): Int = lsuWidth
}

class VecVrfReadReq(implicit p: Parameters) extends BoomBundle
{
  //@req-spec-vrf.f9
  val addr    = UInt(vecPregSz.W)
  val rob_idx = Valid(UInt(robAddrSz.W)) // TRACE-ONLY, see class doc.
}

class VecVrfWrite(implicit p: Parameters) extends BoomBundle
{
  val addr    = UInt(vecPregSz.W)
  val data    = UInt(vecVLen.W)

  //@req-spec-vrf.g9
  val mask    = UInt((vecVLen / 8).W)
  val rob_idx = Valid(UInt(robAddrSz.W)) // TRACE-ONLY, see VecVrfReadReq doc.
}

class VecRegFileIO(numReadPorts: Int, numWritePorts: Int, numDebugReadPorts: Int)(implicit p: Parameters) extends BoomBundle
{
  //@req-spec-vrf.f2
  //@req-spec-vrf.g15
  //@req-spec-cii.f30
  val read           = Vec(numReadPorts, Flipped(Valid(new VecVrfReadReq)))
  val read_data      = Vec(numReadPorts, Output(UInt(vecVLen.W)))

  //@req-spec-vrf.f3
  //@req-spec-vrf.g9
  //@req-spec-vrf.g16
  val write          = Vec(numWritePorts, Flipped(Valid(new VecVrfWrite)))

  // COMMIT-TIME architectural read, for the Whisper cosim vector compare. Address
  // per (commit port, member); `numDebugReadPorts == 0` emits no ports at all.
  val debug_read_addr = Vec(numDebugReadPorts, Input(UInt(vecPregSz.W)))
  val debug_read_data = Vec(numDebugReadPorts, Output(UInt(vecVLen.W)))

  val trace_en       = Input(Bool())
}

class VecRegFile(implicit p: Parameters) extends BoomModule
{
  //@req-spec-vrf.f1
  //@req-spec-vrf.f9
  require(usingRVV, s"VecRegFile instantiated with usingRVV=false")

  //@req-spec-vrf.f2
  val numReadPorts: Int = 9
  require(numReadPorts == 9,
    s"VecRegFile: numReadPorts ($numReadPorts) must be 9 (R0-R8 is a fixed port count)")

  //@req-spec-vrf.f3
  //@req-spec-vrf.g20
  val numWritePorts: Int = 1 + lsuWidth
  require(lsuWidth >= 1 && lsuWidth <= 2,
    s"VecRegFile: lsuWidth ($lsuWidth) must be in [1, 2] (numWritePorts = 1 + lsuWidth must be 2 or 3)")

  val numBanks  = 4
  val bankWidth = vecVLen / numBanks
  val bankBytes = bankWidth / 8

  // One address per (commit port, group member). Gated off entirely for a physical
  // build; on for every simulation config, because a checker that is off by default
  // is the failure this facility exists to remove.
  val numDebugReadPorts: Int = if (enableVecCosimCheck) coreWidth * maxVecMembers else 0

  val io = IO(new VecRegFileIO(numReadPorts, numWritePorts, numDebugReadPorts))

  // ---- 1. Structure: four banks, sliced by width ----
  //
  //@req-spec-vrf.g21
  //@req-spec-vrf.g22
  //@req-spec-vrf.g23
  val bank = Seq.tabulate(numBanks) { b =>
    Module(new VecRegFileBank(
      bankId        = b,
      numBanks      = numBanks,
      numReadPorts  = numReadPorts,
      numWritePorts = numWritePorts,
      numDebugReadPorts = numDebugReadPorts))
  }

  // ---- 2. Why a static partition is sound: no arbitration is required ----
  //
  //@req-spec-vrf.g1
  //@req-spec-cii.f29
  //@req-spec-core.h1
  //@req-spec-core.h2
  //@req-spec-vrf.d2
  //@req-spec-vrf.d3
  //@req-spec-vrf.d4
  //@req-spec-vrf.d5
  //@req-spec-vrf.g13
  //@req-spec-vrf.g14
  //@req-spec-vrf.c9
  //@req-spec-vrf.c10
  //@req-spec-lsu.m6
  //@req-spec-lsu.m7
  for (b <- bank; p <- 0 until numReadPorts) {
    b.io.read_addr(p) := io.read(p).bits.addr
  }

  // ---- 3. The vector bypass network IS this file's read-during-write forwarding ----
  //
  //@req-spec-vrf.d1
  for (p <- 0 until numReadPorts) {
    io.read_data(p) := RegNext(Cat(
      bank(3).io.read_data(p), bank(2).io.read_data(p),
      bank(1).io.read_data(p), bank(0).io.read_data(p)))
  }

  // ---- Tracing: per-access read trace lines (logic §7) ----
  //
  for (p <- 0 until numReadPorts) {
    when (io.trace_en && io.read(p).valid) {
      when (io.read(p).bits.rob_idx.valid) {
        VecTrace.traceId("VecRegFile", "read_req", io.read(p).bits.rob_idx.bits,
          Seq(("port", p.U), ("prn", io.read(p).bits.addr)))
      } .otherwise {
        VecTrace.traceStruct("VecRegFile", "read_req",
          Seq(("port", p.U), ("prn", io.read(p).bits.addr)))
      }
    }
  }

  val read_req_valid_r = RegNext(VecInit(io.read.map(_.valid)))
  val read_rob_r        = RegNext(VecInit(io.read.map(_.bits.rob_idx)))
  for (p <- 0 until numReadPorts) {
    when (io.trace_en && read_req_valid_r(p)) {
      when (read_rob_r(p).valid) {
        VecTrace.traceId("VecRegFile", "read_rsp", read_rob_r(p).bits,
          Seq(("port", p.U), ("data", io.read_data(p)(31, 0))))
      } .otherwise {
        VecTrace.traceStruct("VecRegFile", "read_rsp",
          Seq(("port", p.U), ("data", io.read_data(p)(31, 0))))
      }
    }
  }

  //@req-spec-lsu.m7
  //@req-spec-lsu.m6
  for (b <- 0 until numBanks; w <- 0 until numWritePorts) {
    bank(b).io.write_ports(w).valid      := io.write(w).valid
    bank(b).io.write_ports(w).bits.addr  := io.write(w).bits.addr
    bank(b).io.write_ports(w).bits.data  := io.write(w).bits.data(bankWidth * (b + 1) - 1, bankWidth * b)
    bank(b).io.write_ports(w).bits.mask  := io.write(w).bits.mask(bankBytes * (b + 1) - 1, bankBytes * b)
  }

  // ---- Tracing: per-access write trace line (logic §7) ----
  //
  for (w <- 0 until numWritePorts) {
    when (io.trace_en && io.write(w).valid) {
      when (io.write(w).bits.rob_idx.valid) {
        VecTrace.traceId("VecRegFile", "write", io.write(w).bits.rob_idx.bits,
          Seq(("port", w.U), ("prn", io.write(w).bits.addr),
              ("mask", io.write(w).bits.mask), ("data", io.write(w).bits.data(31, 0))))
      } .otherwise {
        VecTrace.traceStruct("VecRegFile", "write",
          Seq(("port", w.U), ("prn", io.write(w).bits.addr),
              ("mask", io.write(w).bits.mask), ("data", io.write(w).bits.data(31, 0))))
      }
    }
  }

  // ---- Debug ----
  //
  // Combinational, NOT RegNext: the cosim compare happens in the commit cycle, so
  // this path is deliberately one cycle shorter than the functional reads above.
  for (p <- 0 until numDebugReadPorts) {
    for (b <- 0 until numBanks) {
      bank(b).io.debug_read_addr(p) := io.debug_read_addr(p)
    }
    io.debug_read_data(p) := Cat((0 until numBanks).reverse.map(b => bank(b).io.debug_read_data(p)))
  }
}

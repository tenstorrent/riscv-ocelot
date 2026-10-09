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

// GENERATED from src/main/nlhdl/vec/regfile/VecRegFileBank.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.

class VecRegFileBankWritePort(bankWidth: Int, bankBytes: Int)(implicit p: Parameters) extends BoomBundle
{
  val addr = UInt(vecPregSz.W)
  val data = UInt(bankWidth.W)
  val mask = UInt(bankBytes.W)
}

class VecRegFileBankIO(bankWidth: Int, bankBytes: Int, numReadPorts: Int, numWritePorts: Int, numDebugReadPorts: Int)(implicit p: Parameters) extends BoomBundle
{
  // Read ports: no handshake
  val read_addr   = Vec(numReadPorts, Input(UInt(vecPregSz.W)))
  val read_data   = Vec(numReadPorts, Output(UInt(bankWidth.W)))

  // Write ports
  val write_ports = Vec(numWritePorts, Flipped(Valid(new VecRegFileBankWritePort(bankWidth, bankBytes))))

  // Debug reads: storage is a register array, so these are muxes and cannot
  // contend with the functional ports. Zero ports emits no hardware.
  val debug_read_addr = Vec(numDebugReadPorts, Input(UInt(vecPregSz.W)))
  val debug_read_data = Vec(numDebugReadPorts, Output(UInt(bankWidth.W)))
}

class VecRegFileBank(
  val bankId:        Int,
  val numBanks:      Int = 4,
  val numReadPorts:  Int = 9,
  val numWritePorts: Int = 3,
  val numDebugReadPorts: Int = 0
)(implicit p: Parameters) extends BoomModule
{
  require(usingRVV, s"VecRegFileBank (bankId=$bankId) instantiated with usingRVV=false")

  //@req-spec-vrf.f5
  //@req-spec-vrf.f10
  require(numBanks == 4, s"VecRegFileBank (bankId=$bankId): numBanks ($numBanks) must be 4")

  require(bankId >= 0 && bankId < numBanks,
    s"VecRegFileBank: bankId ($bankId) out of range [0, $numBanks)")

  require(numWritePorts >= 2 && numWritePorts <= 3,
    s"VecRegFileBank (bankId=$bankId): numWritePorts ($numWritePorts) must be in [2, 3]")

  //@req-spec-vrf.f11
  require(vecVLen % numBanks == 0,
    s"VecRegFileBank (bankId=$bankId): vecVLen ($vecVLen) must be a multiple of numBanks ($numBanks)")
  val bankWidth = vecVLen / numBanks
  val bankBytes = bankWidth / 8

  val io = IO(new VecRegFileBankIO(bankWidth, bankBytes, numReadPorts, numWritePorts, numDebugReadPorts))

  // ---- Storage ----

  //@req-spec-vrf.f4
  //@req-spec-vrf.f12
  val vrf_bank = Reg(Vec(numVecPhysRegs, UInt(bankWidth.W)))

  //@req-spec-vrf.f6
  //@req-spec-vrf.f11
  private def expandByteMask(mask: UInt): UInt =
    Cat((0 until bankBytes).reverse.map(j => Fill(8, mask(j))))

  // ---- Write path (sequential) ----

  //@req-spec-vrf.f7
  val write_one_hot: Seq[UInt] = io.write_ports.map { wp =>
    Mux(wp.valid, UIntToOH(wp.bits.addr, numVecPhysRegs), 0.U(numVecPhysRegs.W))
  }

  //@req-spec-vrf.f13
  //@req-spec-vrf.f7
  for (r <- 0 until numVecPhysRegs) {
    val portEnables = io.write_ports.zip(write_one_hot).map { case (wp, oh) =>
      Mux(oh(r), expandByteMask(wp.bits.mask), 0.U(bankWidth.W))
    }
    val portDatas = io.write_ports.zip(write_one_hot).map { case (wp, oh) =>
      Mux(oh(r), wp.bits.data & expandByteMask(wp.bits.mask), 0.U(bankWidth.W))
    }
    val enable = portEnables.reduce(_ | _)
    val newData = portDatas.reduce(_ | _)
    vrf_bank(r) := (vrf_bank(r) & ~enable) | newData
  }

  //@req-spec-vrf.f13
  for (i <- 0 until numWritePorts; j <- (i + 1) until numWritePorts) {
    assert(!(io.write_ports(i).valid && io.write_ports(j).valid &&
      io.write_ports(i).bits.addr === io.write_ports(j).bits.addr),
      s"VecRegFileBank bankId=$bankId: write ports $i and $j target the same PRN " +
      "in the same cycle")
  }

  // ---- Read path (combinational, single cycle) ----

  val read_forwards = Wire(Vec(numReadPorts, Bool()))

  //@req-spec-vrf.f7
  //@req-spec-vrf.f8
  //@req-spec-vrf.f14
  private def readAt(addr: UInt): (UInt, Bool) = {
    val arrayData = vrf_bank(addr)
    val fwdHits = io.write_ports.map(wp => wp.valid && wp.bits.addr === addr)
    val fwdEnable = io.write_ports.zip(fwdHits).map { case (wp, hit) =>
      Mux(hit, expandByteMask(wp.bits.mask), 0.U(bankWidth.W))
    }.reduce(_ | _)
    val fwdData = io.write_ports.zip(fwdHits).map { case (wp, hit) =>
      Mux(hit, wp.bits.data & expandByteMask(wp.bits.mask), 0.U(bankWidth.W))
    }.reduce(_ | _)
    ((arrayData & ~fwdEnable) | fwdData, fwdHits.reduce(_ || _))
  }

  for (p <- 0 until numReadPorts) {
    val (data, fwd) = readAt(io.read_addr(p))
    io.read_data(p)  := data
    read_forwards(p) := fwd
  }

  // Debug reads share `readAt` so a debug read of a PRN can never disagree with a
  // functional read of it. Not traced: a commit-time compare must not perturb the
  // read-forward trace the functional ports own.
  for (p <- 0 until numDebugReadPorts) {
    io.debug_read_data(p) := readAt(io.debug_read_addr(p))._1
  }

  // ---- Tracing ----

  val any_forward  = read_forwards.reduce(_ || _)
  val fwd_port_idx = PriorityEncoder(read_forwards)
  when (any_forward) {
    VecTrace.traceStruct("VecRegFileBank", "read_fwd",
      Seq(("bank", bankId.U), ("port", fwd_port_idx), ("prn", io.read_addr(fwd_port_idx))))
  }
}

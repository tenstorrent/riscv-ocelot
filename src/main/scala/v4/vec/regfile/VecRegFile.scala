//******************************************************************************
// Copyright (c) 2013 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Vector Register File (Caracal)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v4.vec.regfile

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

// Standalone vector register file. Does NOT extend the scalar RegisterFile abstract
// class, whose addr width is INT/FP-specific (maxPregSz). Holds numVecPhysRegs entries
// of vecVLen bits each, presented as whole-register (256b) read/write ports.
//
// pvtmp groups are ordinary PRNs and use these same ports (no special wiring). The
// 4x64b lane banking is purely an internal datapath detail enabling per-lane masked
// writes; reads always return the full vecVLen-bit register. Forwarding/bypass is
// handled by the vec bypass network, not here -- matching scalar FullyPortedRF timing
// (registered read address, no same-cycle write-forward).
class VecRegFile(numReadPorts: Int = 8, numWritePorts: Int = 4, numDebugReadPorts: Int = 0)(implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    val read_ports = Vec(numReadPorts, new Bundle {
      val addr = Input(UInt(vecPregSz.W))
      val data = Output(UInt(vecVLen.W))
    })
    val write_ports = Vec(numWritePorts, Flipped(Valid(new Bundle {
      val addr = UInt(vecPregSz.W)
      val data = UInt(vecVLen.W)
      val mask = UInt((vecVLen / 8).W)             // per-BYTE write enable (one bit/byte)
    })))
    // COMBINATIONAL debug-only read ports (no RegNext): used by the cosim commit
    // trace to read back a committed vector op's dest group the SAME cycle it
    // commits, so the loaded/computed result is visible to the arch checker.
    // Not synthesized for real hardware; present only when numDebugReadPorts > 0.
    val debug_read_ports = Vec(numDebugReadPorts, new Bundle {
      val addr = Input(UInt(vecPregSz.W))
      val data = Output(UInt(vecVLen.W))
    })
  })

  val nLanes = vecVLen / 64 // 4 lanes of 64b for VLEN=256

  // [prn][lane]; lane 0 is bits[63:0], so .asUInt reconstructs the register with lane 0
  // as the least-significant bits (matches the per-lane write slicing below).
  val vrf = Reg(Vec(numVecPhysRegs, Vec(nLanes, UInt(64.W))))

  // Registered read address, no same-cycle write-forward (mirror FullyPortedRF, regfile.scala:206).
  for (r <- 0 until numReadPorts) {
    io.read_ports(r).data := vrf(RegNext(io.read_ports(r).addr)).asUInt
  }

  // Combinational debug reads (cosim commit trace).
  for (r <- 0 until numDebugReadPorts) {
    io.debug_read_ports(r).data := vrf(io.debug_read_ports(r).addr).asUInt
  }

  // Per-BYTE masked write. Each lane is 8 bytes; write byte (8*l + k) from the
  // data iff mask bit (8*l + k) is set, else keep the old byte. Per-byte (not
  // per-lane) granularity is required for sub-lane writes: a mask load (vlm) or
  // a partial-lane tail writes only some bytes of a lane and must leave the rest
  // undisturbed (a per-lane write would zero them).
  for (w <- io.write_ports) {
    when (w.valid) {
      for (l <- 0 until nLanes) {
        val old_lane = vrf(w.bits.addr)(l)
        val new_lane = w.bits.data(64 * l + 63, 64 * l)
        vrf(w.bits.addr)(l) := Cat((0 until 8).reverse.map { k =>
          Mux(w.bits.mask(8 * l + k), new_lane(8 * k + 7, 8 * k), old_lane(8 * k + 7, 8 * k))
        })
      }
    }
  }

  // ensure there is only 1 writer per register (copy of regfile.scala:42-51)
  if (numWritePorts > 1) {
    for (i <- 0 until (numWritePorts - 1)) {
      for (j <- (i + 1) until numWritePorts) {
        assert(!io.write_ports(i).valid ||
               !io.write_ports(j).valid ||
               (io.write_ports(i).bits.addr =/= io.write_ports(j).bits.addr),
          "[vecregfile] too many writers a register")
      }
    }
  }
}

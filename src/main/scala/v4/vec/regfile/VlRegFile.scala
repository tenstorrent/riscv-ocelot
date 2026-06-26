//******************************************************************************
// Copyright (c) 2013 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// VL Register File (Caracal)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v4.vec.regfile

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

// Standalone register file holding the VL VALUE (0..maxVecVL = vecVLen elements), one
// vecVLSz-bit entry per VL preg. Does NOT extend the scalar RegisterFile abstract class
// (its addr width is INT/FP-specific). Read by EUs via the pvl operand.
//
// Write ports are wired in Steps 9/11: W0 = VCFG (vsetivli), W1 = int-ALU (vsetvli/vsetvl),
// W2 = LSU (vleff). Timing matches scalar FullyPortedRF: registered read address, no
// same-cycle write-forward.
class VlRegFile(numReadPorts: Int = 6, numWritePorts: Int = 3)(implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    val read_ports = Vec(numReadPorts, new Bundle {
      val addr = Input(UInt(vlPregSz.W))
      val data = Output(UInt(vecVLSz.W))
    })
    val write_ports = Vec(numWritePorts, Flipped(Valid(new Bundle {
      val addr = UInt(vlPregSz.W)
      val data = UInt(vecVLSz.W)
    })))
  })

  val vlrf = Reg(Vec(numVlPhysRegs, UInt(vecVLSz.W)))

  // Registered read address, no same-cycle write-forward (mirror FullyPortedRF, regfile.scala:206).
  for (r <- 0 until numReadPorts) {
    io.read_ports(r).data := vlrf(RegNext(io.read_ports(r).addr))
  }

  // Write (mirror FullyPortedRF write, regfile.scala:208).
  for (w <- io.write_ports) {
    when (w.valid) {
      vlrf(w.bits.addr) := w.bits.data
    }
  }

  // ensure there is only 1 writer per register (copy of regfile.scala:42-51)
  if (numWritePorts > 1) {
    for (i <- 0 until (numWritePorts - 1)) {
      for (j <- (i + 1) until numWritePorts) {
        assert(!io.write_ports(i).valid ||
               !io.write_ports(j).valid ||
               (io.write_ports(i).bits.addr =/= io.write_ports(j).bits.addr),
          "[vlregfile] too many writers a register")
      }
    }
  }
}

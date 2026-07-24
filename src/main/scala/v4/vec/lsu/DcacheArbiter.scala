//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal D$ Interface Arbiter (Track A / A1)
//------------------------------------------------------------------------------
//
// loadstore.rst §"D$ Interface Arbiter". The unified LSU shares the D$ request lane(s)
// (`lsuWidth` wide -- 1 on Medium, 2 on Large/Mega) plus the LCAM and TLB ports among
// several requestors: scalar LSU fire, vector LOAD drain (US Packer / SSI per-element),
// and vector STORE drain (post-commit). This is the priority-round-robin grant policy:
//
//   - PRIORITY FLOOR: scalar memory ops carry a higher base priority than vector
//     drains, so a burst of vector element accesses never indefinitely blocks a scalar
//     load/store (keeps scalar memory latency near baseline). Modeled here as a `hi`
//     class per requestor -- hi requestors always outrank lo (vector) requestors.
//   - ROUND-ROBIN ANTI-STARVATION: among the lo (vector) requestors a rotating pointer
//     guarantees each contending requestor a grant within N cycles, so a long vector
//     drain yields the lane periodically and a steady scalar stream still lets the
//     vector drain advance.
//   - WORK-CONSERVING, up to `grantWidth` (= lsuWidth) grants/cycle: an all-scalar
//     workload uses every lane for scalar, all-vector uses every lane for vector, a mix
//     splits under the floor + round-robin. (Dual-lane on Large/Mega; single on Medium.)
//
// The same grant vector gates the shared LCAM and TLB ports (applied at the wiring, so
// no single vector OP.v monopolizes disambiguation/translation). Pure combinational
// policy + one RR register; instantiated only when `usingVecSnoop`.

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

class DcacheArbReq(implicit p: Parameters) extends BoomBundle {
  val valid = Bool()   // this requestor wants a lane this cycle
  val hi    = Bool()   // priority class: true = scalar floor, false = vector drain
}

/** Priority-round-robin arbiter over `grantWidth` lanes for `nReq` requestors.
  * Grants the `grantWidth` lowest-priority-score valid requestors, where hi requestors
  * score below (win over) lo requestors, and lo requestors are ranked by a rotating
  * round-robin pointer. Scores are unique, so grants are unambiguous. */
class DcacheArbiter(nReq: Int, grantWidth: Int)(implicit p: Parameters) extends BoomModule
{
  require(nReq >= 2 && grantWidth >= 1)
  val io = IO(new Bundle {
    val req   = Input(Vec(nReq, new DcacheArbReq))
    val grant = Output(Vec(nReq, Bool()))
    val kill  = Input(Bool())
  })

  // Round-robin pointer over the lo (vector) requestors.
  val rr = RegInit(0.U(log2Ceil(nReq).W))

  // Rotated distance of index i from rr (0 == highest lo priority), in [0, nReq).
  def rotDist(i: Int): UInt = {
    val d = Wire(UInt(log2Ceil(2 * nReq + 1).W))
    d := Mux(i.U >= rr, (i.U - rr), ((i + nReq).U - rr))
    d
  }

  // Unique priority score: hi requestors 0..nReq-1 (by index); lo requestors
  // nReq + rotDist (always above all hi). Lower score = higher priority.
  val score = VecInit((0 until nReq).map { i =>
    Mux(io.req(i).hi, i.U(log2Ceil(2 * nReq + 1).W), nReq.U + rotDist(i))
  })

  // Grant the requestor iff fewer than grantWidth valid requestors outrank it.
  for (i <- 0 until nReq) {
    val nOutrank = PopCount((0 until nReq).map { j =>
      io.req(j).valid && (score(j) < score(i))
    })
    io.grant(i) := io.req(i).valid && (nOutrank < grantWidth.U)
  }

  // Advance the RR pointer past the lo requestors served this cycle (they rotate to the
  // back), so anti-starvation holds. hi grants do not move the pointer.
  val loGrants = PopCount((0 until nReq).map(i => io.grant(i) && !io.req(i).hi))
  val nextRr   = rr +& loGrants
  when (!io.kill) {
    rr := Mux(nextRr >= nReq.U, nextRr - nReq.U, nextRr)(log2Ceil(nReq) - 1, 0)
  } .otherwise {
    rr := 0.U
  }
}

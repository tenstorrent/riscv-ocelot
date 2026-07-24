//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Cross-LSU Disambiguation Snoop (Track A / A1)
//------------------------------------------------------------------------------
//
// loadstore.rst §mem-order. Bidirectional cross-LSU disambiguation, reusing the
// scalar `lsu.scala` age helpers (IdxAgeOt / EntryValidFromAge):
//
//   (i)  STORE -> LDQ  [implemented here, = the Track-A A2 behavior]: a vector store's
//        element/US-range address searches the LDQ (scalar + in-flight vector loads);
//        a YOUNGER, already-executed load it overlaps has its `order_fail` set, driving
//        BOOM's MINI_EXCEPTION_MEM_ORDERING refetch replay. US = one range-overlap
//        check of [base, base+VL*EEW); SSI = per element as beats drain.
//   (ii) LOAD -> STQ + store address queues [scaffolded; forward-select armed in A4]:
//        a scalar/vector load address searches older stores for a forwarding source.
//
// This module encapsulates the vector STORE->LDQ range search that Track A A2 currently
// does in-line in `lsu.scala`; `lsu.scala` feeds it the per-pipe vector-store searcher
// + an LDQ range snapshot and ORs `order_fail` back into `ldq_order_fail`. It is
// instantiated only when `usingVecSnoop`; the in-line A2 path is used otherwise, so the
// disambiguation behavior is identical whether or not this module is present.

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.lsu.{IdxAgeOt}

/** Per-pipe searcher: a vector store beat's translated dword address + its STQ index
  * (for the program-order age comparison against a load's older-store boundary). */
class SnoopStoreSearch(implicit p: Parameters) extends BoomBundle with VecLsConstants {
  val valid   = Bool()
  val addr    = UInt(coreMaxAddrBits.W)         // store beat dword paddr (M1 bare-physical)
  val stq_idx = UInt((1 + stqAddrSz).W)         // searcher's STQ index (+carry)
}

/** LDQ range snapshot entry the snoop compares against (the fields the store->LDQ
  * search needs), one per LDQ entry. Populated from lsu.scala's ldq_* state. */
class SnoopLdqEntry(implicit p: Parameters) extends BoomBundle with VecLsConstants {
  val valid        = Bool()
  val eos          = Bool()                     // executed || succeeded (has read the D$)
  val next_stq_idx = UInt((1 + stqAddrSz).W)    // older-store boundary
  val rng_v        = Bool()                     // vector-load [lo,hi) range registered
  val lo           = UInt(coreMaxAddrBits.W)
  val hi           = UInt(coreMaxAddrBits.W)
}

class CrossLsuSnoop(implicit p: Parameters) extends BoomModule with VecLsConstants
{
  val io = IO(new Bundle {
    // (i) STORE -> LDQ search: one searcher per D$ pipe.
    val st_search  = Input(Vec(lsuWidth, new SnoopStoreSearch))
    val ldq        = Input(Vec(numLdqEntries, new SnoopLdqEntry))
    // set on any LDQ entry that must replay (OR into lsu.scala ldq_order_fail).
    val order_fail = Output(Vec(numLdqEntries, Bool()))
    val kill       = Input(Bool())
  })

  // (i) STORE -> LDQ range disambiguation (= Track-A A2). For each LDQ entry, fail it if
  // an older store beat (this pipe's searcher) overlaps its registered [lo,hi) range and
  // the load already executed. 8B store beat [addr, addr+8) overlaps [lo,hi). IdxAgeOt
  // (searcher STQ idx older than the load's next_stq_idx boundary) => store is
  // program-order-older than the load, i.e. a genuine ST->LD ordering violation.
  for (i <- 0 until numLdqEntries) {
    val e = io.ldq(i)
    val hit = (0 until lsuWidth).map { w =>
      val s = io.st_search(w)
      s.valid && e.valid && e.rng_v && e.eos &&
        IdxAgeOt(s.stq_idx, e.next_stq_idx) &&
        (s.addr < e.hi) && ((s.addr + 8.U) > e.lo)
    }.reduce(_ || _)
    io.order_fail(i) := hit && !io.kill
  }

  // (ii) LOAD -> STQ + store-address-queue forward search: scaffolded for A4. The
  // structure (a load searcher + STQ/store-queue snapshot -> forward-select) attaches
  // here; the forwarding action is armed in Step A4, so nothing is driven yet.
}

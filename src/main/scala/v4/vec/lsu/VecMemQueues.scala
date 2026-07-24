//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Memory Address/Data Queues (Track A / A1)
//------------------------------------------------------------------------------
//
// The disambiguation substrate from loadstore.rst §"vector address and data queues".
// A vector memory OP.v holds a SINGLE placeholder entry in the scalar LDQ/STQ; the
// many effective element accesses (nOP.v) it expands into are buffered HERE instead
// of polluting the scalar LDQ/STQ. Six queues, named {ld,st}_{SSI,US}_{ADDR,DATA}_Q:
//
//   - SSI (strided/segmented/indexed): element-wise effective addresses. Sized to the
//     worst-case single-instruction element count (`ssiQueueEntries`, 512): a STORE
//     must retain its full active address+data set from execute until commit-drain
//     (precise faults), so it cannot stream mid-instruction; additional in-flight
//     stores back-pressure the store vAGEN/vDGEN. A LOAD may stream (drains in waves).
//   - US (unit-stride): a single nOP.v `[base, base + VL*EEW)` per OP.v -- one entry
//     per in-flight US load/store, expanded to per-element D$ accesses just-in-time by
//     the Packer AGEN at drain. Sized to the LDQ/STQ depth.
//   - DATA queues hold store data captured from the source vPRN at DGEN (execute), so
//     the vPRN needs no pin (frees with the stale group at commit). SSI data = one
//     element (ELEN); US data = a full VLEN (the Packer slices per-element at drain).
//
// This module is pure storage + handshakes (chisel Queues) plus a combinational
// address-search port for CrossLsuSnoop. It is instantiated only when `usingVecSnoop`
// (vecScalarSnoopEnable); when off it does not exist and the datapath is unchanged.

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._

/** One SSI (strided/segmented/indexed) element access: a translated effective address
  * plus the owning LDQ/STQ placeholder index and per-element bookkeeping. */
class VecSsiAddrEntry(implicit p: Parameters) extends BoomBundle with VecLsConstants
{
  val addr    = UInt(coreMaxAddrBits.W)   // translated element effective address (M1: bare-physical)
  val nbytes  = UInt(4.W)                 // access size in bytes (1..8 = EEW)
  val lsq_idx = UInt((1 + math.max(ldqAddrSz, stqAddrSz)).W) // owning LDQ (load) / STQ (store) entry
  val elem_id = UInt(9.W)                 // element index within the OP.v (0..255)
  val last    = Bool()                    // last element access of this OP.v
}

/** One US (unit-stride) OP.v: the whole contiguous range `[base, base + nbytes)` in a
  * single entry; the Packer AGEN expands it to per-element D$ accesses at drain. */
class VecUsAddrEntry(implicit p: Parameters) extends BoomBundle with VecLsConstants
{
  val base    = UInt(coreMaxAddrBits.W)   // range base (translated; M1 bare-physical)
  val nbytes  = UInt(log2Ceil(VLEN_BYTES + 1).W) // VL*EEW total bytes of the range
  val eew     = UInt(2.W)                 // encoded EEW (for per-element slicing at drain)
  val lsq_idx = UInt((1 + math.max(ldqAddrSz, stqAddrSz)).W)
  val dir     = Bool()                    // negative-stride direction (padding)
}

/** The six vector memory queues + a combinational address-overlap search port used by
  * CrossLsuSnoop. `ssiEntries` sizes the SSI queues (worst-case store element count);
  * the US queues hold one entry per in-flight US OP.v. */
class VecMemQueues(implicit p: Parameters) extends BoomModule with VecLsConstants
{
  val ssiEntries = vectorParams.ssiQueueEntries
  val ldUsEntries = numLdqEntries
  val stUsEntries = numStqEntries
  val eLenBits    = 64

  val io = IO(new Bundle {
    // ---- enqueue (from the vAGENs / vDGEN) ----
    val ld_ssi_addr_enq = Flipped(Decoupled(new VecSsiAddrEntry))
    val st_ssi_addr_enq = Flipped(Decoupled(new VecSsiAddrEntry))
    val st_ssi_data_enq = Flipped(Decoupled(UInt(eLenBits.W)))
    val ld_us_addr_enq  = Flipped(Decoupled(new VecUsAddrEntry))
    val st_us_addr_enq  = Flipped(Decoupled(new VecUsAddrEntry))
    val st_us_data_enq  = Flipped(Decoupled(UInt(vecVLen.W)))

    // ---- dequeue (to the DcacheArbiter for D$ access / drain) ----
    val ld_ssi_addr_deq = Decoupled(new VecSsiAddrEntry)
    val st_ssi_addr_deq = Decoupled(new VecSsiAddrEntry)
    val st_ssi_data_deq = Decoupled(UInt(eLenBits.W))
    val ld_us_addr_deq  = Decoupled(new VecUsAddrEntry)
    val st_us_addr_deq  = Decoupled(new VecUsAddrEntry)
    val st_us_data_deq  = Decoupled(UInt(vecVLen.W))

    // ---- occupancy / back-pressure (stores must not overflow; gate the vAGEN) ----
    val st_ssi_addr_full = Output(Bool())
    val st_ssi_data_full = Output(Bool())

    val kill = Input(Bool())
  })

  // Store queues cannot free mid-instruction (precise faults) and are past-commit /
  // non-speculative, so a branch/flush kill must NOT drop them -- plain reset. LOAD
  // queues are speculative and flush on kill (withReset). Drain/back-pressure policy
  // lives in the arbiter + the vAGEN gate.
  val killLoad = reset.asBool || io.kill
  val ld_ssi_addr_q = withReset(killLoad) { Module(new Queue(new VecSsiAddrEntry, ssiEntries, flow = false)) }
  val ld_us_addr_q  = withReset(killLoad) { Module(new Queue(new VecUsAddrEntry,  ldUsEntries, flow = false)) }
  val st_ssi_addr_q = Module(new Queue(new VecSsiAddrEntry, ssiEntries, flow = false))
  val st_ssi_data_q = Module(new Queue(UInt(eLenBits.W),    ssiEntries, flow = false))
  val st_us_addr_q  = Module(new Queue(new VecUsAddrEntry,  stUsEntries, flow = false))
  val st_us_data_q  = Module(new Queue(UInt(vecVLen.W),     stUsEntries, flow = false))

  ld_ssi_addr_q.io.enq <> io.ld_ssi_addr_enq;  io.ld_ssi_addr_deq <> ld_ssi_addr_q.io.deq
  st_ssi_addr_q.io.enq <> io.st_ssi_addr_enq;  io.st_ssi_addr_deq <> st_ssi_addr_q.io.deq
  st_ssi_data_q.io.enq <> io.st_ssi_data_enq;  io.st_ssi_data_deq <> st_ssi_data_q.io.deq
  ld_us_addr_q.io.enq  <> io.ld_us_addr_enq;   io.ld_us_addr_deq  <> ld_us_addr_q.io.deq
  st_us_addr_q.io.enq  <> io.st_us_addr_enq;   io.st_us_addr_deq  <> st_us_addr_q.io.deq
  st_us_data_q.io.enq  <> io.st_us_data_enq;   io.st_us_data_deq  <> st_us_data_q.io.deq

  // A store's address/data queues must hold its full active set pre-commit; expose
  // "almost full" (< one max OP.v of headroom) so the store vAGEN/vDGEN stalls rather
  // than overflowing. maxVecVL element-accesses is the worst case per OP.v.
  io.st_ssi_addr_full := st_ssi_addr_q.io.count > (ssiEntries - maxVecVL).U
  io.st_ssi_data_full := st_ssi_data_q.io.count > (ssiEntries - maxVecVL).U
}

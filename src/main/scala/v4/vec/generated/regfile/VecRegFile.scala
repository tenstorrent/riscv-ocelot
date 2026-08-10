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
// VecRegFile -- the vector physical register file as seen by the rest of the
// machine: 96 PRNs x `vLen` bits presented through a STATICALLY PARTITIONED
// 9-read / 3-write port set, assembled from four `VecRegFileBank` width
// slices. Instantiated once, as `vrf`, by VecPipeline.
//
// THIS NODE OWNS WHO OWNS WHICH PORT. The companion module VecRegFileBank
// owns HOW the array is built (flops, banking by width, per-port decoders,
// the per-byte write merge, read-during-write forwarding). This file owns the
// canonical port partition from midcore.rst `vrf-ports`, the fan-out of a
// vLen-wide write into four bank slices, the concatenation of four read
// slices back into a vLen-wide result, and `debug_vrf_read`. It owns NO
// multiplexing at all: every port here has exactly one driver by the time it
// reaches this module (A16), and the only strict-priority mux in the vector
// register path lives in VecGroupCopy, one level up.
//
//@req-spec-core.d4
// The VRF is instantiated by VecPipeline as `vrf` and lives in the host. The
// coprocessor reads/writes it through the CII as a CLIENT (R5-R8, W2) -- it
// holds no VRF storage, no shadow copy and no private address space.
//
// Governing spec anchors: midcore.rst `vrf-ports` (the canonical partition),
// midcore.rst `vector-regfile` (96 PRNs / 9R / 3W / banked / usingRVV),
// midcore.rst `regfiles-bypass` (the vector bypass network, no temp file, no
// mask file), midcore.rst `old-vd` (why R2 exists), cii.rst `cii-operands`
// and `cii-writeback` (R5-R8 / W2, never-stall), case_study.rst
// `case-vl-zero` (the group copy borrowing R2/W0).
//
// TRACING: VecTrace's three-step ladder -- trace(uop, ...) -> traceId (bare
// rob_idx) -> traceStruct (neither) -- is now fully generated, which
// unblocks every per-access VRF trace line (request-cycle read,
// response-cycle read, write) described in the nlhdl `logic` section
// paragraph 7 that a prior generation of this module had to omit. This
// module still has no MicroOp (rung 1 never applies here), so every line
// below reaches for rung 2 or 3: each port's trace-only `rob_idx:
// Valid(UInt)` (see VecVrfReadReq/VecVrfWrite) is the genuine identifier
// when valid -- `traceId`, a real `rob=<idx>` -- and there is no honest
// identifier when it is not -- `traceStruct`, `rob=?`, keyed on `port`/
// `prn` (matching VecRegFileBank's own `traceStruct` convention for a
// port/PRN-scoped event rather than an instruction-scoped one). No printf is
// hand-rolled anywhere here; every line goes through VecTrace's public
// entry points. `io.trace_en` is ANDed into the gate at every call site
// below, as the ports section requires.

/**
 * VrfPort -- named, 0-based constants for every read/write port index, so a
 * name and a number can never disagree. Ports are STATICALLY PARTITIONED and
 * never arbitrated: no arbiter, no round-robin, no request/grant and no
 * `ready` anywhere on this interface.
 *
 * Two independent reasons license the total absence of arbitration: (1) two
 * write ports can never target the same PRN (rename always allocates a fresh
 * group per destination), so no write priority is needed; (2) a read is
 * always the full vLen width, spanning all four banks, so several read ports
 * addressing one bank in the same cycle is the normal case, never a hazard.
 * Neither conflict a naively wider port count would invite can arise here.
 */
//@req-spec-vrf.g1
//@req-spec-cii.f29
object VrfPort {
  //@req-spec-vrf.g18
  //@req-spec-vrf.g19
  // R0-R4: each has exactly one client on its own side (R4's one reader
  // multiplexes two upstream jobs -- store mask and store index -- but that
  // resolution happens in VecLsu, one level up, never here).
  val R_LD_IDX      = 0 // R0: load index read (VecIdxGen load instance)
  val R_LD_MASK     = 1 // R1: load mask v0 read (VecMaskStream ld_msk)
  val R_LD_STALE    = 2 // R2: LCB's stale_pvdest member read
  val R_ST_DATA     = 3 // R3: store-data read (VecDgen)
  val R_ST_MASK_IDX = 4 // R4: store mask v0 AND store index (one port, two readers upstream)

  //@req-spec-vrf.g10
  //@req-spec-vrf.h3
  //@req-spec-cii.f25
  // R5-R8: the four CII source pull lanes, one per Src-Request pull lane.
  // FOUR, not two, and DERIVED rather than chosen: CII_NUM_SRC_REQ ==
  // CII_NUM_SRC_DAT_RSP == 4 in tt_cii_caracal_pkg.svh (the VPU wrapper
  // returns member k of {VS1, VS2, VS3, VM} in one dat beat, so all four
  // lanes can be live in one cycle). Owned outright by VecCiiOperandServer.
  val R_CII_BASE    = 5 // R5-R8 == R_CII_BASE + 0 .. R_CII_BASE + 3

  // W0/W1: the LCB's write lanes. W1 exists only at lsuWidth == 2.
  val W_LD_LCB0 = 0
  val W_LD_LCB1 = 1

  //@req-spec-vrf.g11
  //@req-spec-vrf.h4
  //@req-spec-cii.f26
  //@req-spec-vrf.g17
  //@req-spec-cii.g2
  // W2: the coprocessor writeback, owned outright by VecCiiWriteback. ONE
  // write port suffices precisely because CII_NUM_DST_WB == 1 in
  // tt_cii_caracal_pkg.svh: the coprocessor cannot present two result beats
  // in one cycle, so a second write port would never be driven.
  //
  //@req-spec-vrf.f3
  //@req-spec-vrf.g20
  // W2's PHYSICAL INDEX IS `lsuWidth`, NEVER A LITERAL 2 (A17): the write
  // array is compacted in canonical W0,W1,W2 order and W1 is ABSENT (not
  // tied off) at lsuWidth == 1, so W2 is physical entry 1 there and physical
  // entry 2 only at lsuWidth == 2. Expressed as a function of lsuWidth,
  // never a bare constant, so a caller cannot spell the wrong index.
  def W_CII(lsuWidth: Int): Int = lsuWidth
}

/**
 * One read-port request: the PRN to read, plus a trace-only `rob_idx`.
 * `rob_idx` carries NO functional meaning and must not reach the datapath,
 * the decoders or the forwarding compare -- it exists only so a per-access
 * VRF trace line can be tied to an instruction. `Valid` rather than a bare
 * UInt so a client with no uop in hand leaves it invalid rather than
 * inventing a zero that aliases real ROB entry 0.
 */
class VecVrfReadReq(implicit p: Parameters) extends BoomBundle
{
  //@req-spec-vrf.f9
  val addr    = UInt(vecPregSz.W)
  val rob_idx = Valid(UInt(robAddrSz.W)) // TRACE-ONLY, see class doc.
}

/**
 * One write-port payload: destination PRN, full vLen-wide data, a per-BYTE
 * write mask, and the same trace-only `rob_idx`.
 */
class VecVrfWrite(implicit p: Parameters) extends BoomBundle
{
  val addr    = UInt(vecPregSz.W)
  val data    = UInt(vecVLen.W)

  //@req-spec-vrf.g9
  // PER BYTE (vLenBytes bits), NOT per 64-bit lane: vlm.v writes ceil(vl/8)
  // bytes, and a partial tail writes fewer -- every byte the mask does not
  // select must be left undisturbed. Per-lane granularity inside a 64-bit
  // bank would be indistinguishable from no mask at all.
  val mask    = UInt((vecVLen / 8).W)
  val rob_idx = Valid(UInt(robAddrSz.W)) // TRACE-ONLY, see VecVrfReadReq doc.
}

/**
 * VecRegFile's IO. No handshake anywhere: ports are statically partitioned,
 * so this module presents no `ready`, no `Decoupled` and no grant on any
 * port, and cannot refuse a request.
 *
 * BECAUSE THE PARTITION IS STATIC, THE CII CAN NEVER STALL ON A VRF PORT: it
 * owns R5-R8 and W2 outright, this module presents no back-pressure on
 * either, and nothing else in the machine can drive them, so a CII access is
 * accepted in the cycle it is presented, always -- load-bearing because the
 * CII Writeback channel is credit-metered with no back-pressure line of its
 * own, so a refused write would have nowhere to go.
 */
class VecRegFileIO(numReadPorts: Int, numWritePorts: Int)(implicit p: Parameters) extends BoomBundle
{
  //@req-spec-vrf.f2
  //@req-spec-vrf.g15
  //@req-spec-cii.f30
  // Fixed at 9 (R0-R8); see the `require` in VecRegFile for why this is a
  // spec obligation and not a tuning knob. `Flipped(Valid(...))`, never
  // `Decoupled` -- no ready, no grant, so R5-R8 (owned by the CII) can never
  // be refused.
  val read           = Vec(numReadPorts, Flipped(Valid(new VecVrfReadReq)))
  val read_data      = Vec(numReadPorts, Output(UInt(vecVLen.W)))

  //@req-spec-vrf.f3
  //@req-spec-vrf.g9
  //@req-spec-vrf.g16
  // 2 at lsuWidth == 1 (W0, W2), 3 at lsuWidth == 2 (W0, W1, W2). The mask
  // inside VecVrfWrite is per-byte (see that class), not per bank lane.
  // `Flipped(Valid(...))`, never `Decoupled` -- no ready, no grant, so W2
  // (owned by the CII writeback, credit-metered with no back-pressure line
  // of its own) can never be refused.
  val write          = Vec(numWritePorts, Flipped(Valid(new VecVrfWrite)))

  // Built from the ordinary read ports, no address of its own: a private
  // debug address port would be a thirteenth port on the array, which the
  // canonical table forbids.
  val debug_vrf_read = Vec(coreWidth, Output(UInt(vecVLen.W)))

  // ANDed into VecTrace's gate at every per-access trace call site below
  // (read_req, read_rsp, write) -- see the file-header TRACING note.
  val trace_en       = Input(Bool())
}

/**
 * VecRegFile -- see file header for the full design rationale. Instantiated
 * once, as `vrf`, by VecPipeline.
 */
class VecRegFile(implicit p: Parameters) extends BoomModule
{
  // The module's existence, not its content, is gated on usingRVV (the
  // Scala Boolean from BoomCoreParams -- NOT rocket's `usingVector`):
  // VecPipeline, this module's only instantiator, exists only under that
  // gate, so a vectors-off build contains no instance of this module and no
  // vector flops at all -- absent, not tied off.
  //@req-spec-vrf.f1
  //@req-spec-vrf.f9
  require(usingRVV, s"VecRegFile instantiated with usingRVV=false")

  //@req-spec-vrf.f2
  // Fixed at 9 (R0-R8): a spec obligation (the canonical partition), not a
  // tier knob. It is 9 at every lsuWidth -- the second LCB write lane adds a
  // WRITE port, never a read.
  val numReadPorts: Int = 9
  require(numReadPorts == 9,
    s"VecRegFile: numReadPorts ($numReadPorts) must be 9 (R0-R8 is a fixed port count)")

  //@req-spec-vrf.f3
  //@req-spec-vrf.g20
  // numWritePorts = 1 + lsuWidth: 2 at lsuWidth == 1, 3 at lsuWidth == 2
  // (the default tier this design targets). Bound to lsuWidth -- BOOM's
  // existing D$-lane parameter -- rather than a list of config names, so
  // every tier is covered without this file tracking a config list.
  val numWritePorts: Int = 1 + lsuWidth
  require(lsuWidth >= 1 && lsuWidth <= 2,
    s"VecRegFile: lsuWidth ($lsuWidth) must be in [1, 2] (numWritePorts = 1 + lsuWidth must be 2 or 3)")

  // vLen/bank geometry restated here (the bank's own parameters) only
  // because this node does the fan-out/concatenation and must use the same
  // values as VecRegFileBank.
  val numBanks  = 4
  val bankWidth = vecVLen / numBanks
  val bankBytes = bankWidth / 8

  val io = IO(new VecRegFileIO(numReadPorts, numWritePorts))

  // ---- 1. Structure: four banks, sliced by width ----
  //
  //@req-spec-vrf.g21
  //@req-spec-vrf.g22
  //@req-spec-vrf.g23
  // At lsuWidth == 1 the write array has two entries (W0, W2) and W1 is
  // ABSENT -- not elaborated, not a tied-off third entry -- so the file is
  // 9R/2W. At lsuWidth == 2 the array has three entries and the file is
  // 9R/3W. Elaborating a dead third write port would contradict the 9R/2W
  // total and would pay four more per-bank decoders for a port nothing
  // drives. `numWritePorts` above already reflects this; nothing further is
  // needed here beyond instantiating each bank with that count.
  val bank = Seq.tabulate(numBanks) { b =>
    Module(new VecRegFileBank(
      bankId        = b,
      numBanks      = numBanks,
      numReadPorts  = numReadPorts,
      numWritePorts = numWritePorts))
  }

  // ---- 2. Why a static partition is sound: no arbitration is required ----
  //
  //@req-spec-vrf.g1
  //@req-spec-cii.f29
  // Two write ports never target the same PRN: every PRN is the destination
  // of exactly one producer and rename always allocates a FRESH group, so
  // W0/W1 (the LCB) and W2 (the coprocessor) cannot name the same register,
  // including for a shared instruction (one half writes pvtmp, the other
  // pvdest -- two distinct groups). So no write priority and no write
  // arbitration exists anywhere in this module; the assertion enforcing that
  // property lives in VecRegFileBank (one per bank, checked against the
  // same broadcast `io.write` below) and is DELIBERATELY NOT DUPLICATED
  // here -- see the bank-invariant note in the final report for the
  // OR-vs-priority-mux consequence of that choice.
  //
  // Read ports never conflict over a bank either: a read is a full vLen, so
  // every port spans all four banks, and several read ports addressing one
  // bank is the normal case rather than a hazard.

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
  // There is no separate temporary register file and no separate mask
  // register file anywhere in this design. `pvtmp` (a shared/segmented op's
  // intermediate-result group) and `v0` (a vector mask) are read and
  // written on the SAME ports as any other PRN, below -- this module cannot
  // tell them apart from any other PRN, which is the property being
  // specified. Concretely, and adding no port: a segmented LOAD writes
  // pvtmp on W0/W1 and the coprocessor reads it on R5-R8; a segmented STORE
  // has the coprocessor write pvtmp on W2 and VecDgen read it as store data
  // on R3. `v0` is read as a whole vLen-wide PRN on R1 (load path) or R4
  // (store path) and delivered to the coprocessor as the VM source on one
  // of R5-R8; masking semantics are applied in the execution units, never
  // here -- this module has no notion of a mask register, only of the
  // per-byte WRITE mask on VecVrfWrite, a different thing entirely.
  //
  //@req-spec-lsu.m6
  //@req-spec-lsu.m7
  // R2's stale_pvdest read and W0's write are reused, unmodified, by the
  // VL == 0 / fully-inactive group copy (VecGroupCopy) -- the same two ports
  // the LCB uses, adding NO new VRF port. The strict-priority resolution
  // between the LCB and the group copy happens entirely in VecGroupCopy, one
  // level up (A16): what reaches `io.read(R_LD_STALE)` / `io.write(W_LD_LCB0)`
  // below is one already-resolved address and one already-resolved write, so
  // this module needs no mux for either.
  //
  // Read address: the FULL, UNSHIFTED PRN, to every bank, for every port.
  // There is no bank-select bit and no per-bank port subset -- banking is BY
  // WIDTH, not by register index (the opposite of BOOM's scalar BankedRF).
  for (b <- bank; p <- 0 until numReadPorts) {
    b.io.read_addr(p) := io.read(p).bits.addr
  }

  // ---- 3. The vector bypass network IS this file's read-during-write forwarding ----
  //
  //@req-spec-vrf.d1
  // BOOM's INT/FP bypass network is realized here as PER-PORT
  // READ-DURING-WRITE FORWARDING INSIDE THE REGISTER FILE (in the banks, on
  // every read port), not as a separate muxing network between execution
  // units -- a design decision, not an omission (see file header). The
  // `RegNext` below IS the read-port output flop, one per read port and the
  // only state in this module: it captures the bank's ALREADY-FORWARDED
  // value at the end of the request cycle, so the registered response is
  // coherent with a same-cycle write rather than in tension with it.
  // little-endian: bank 0 in the least-significant bits, matching the
  // bank's bit-slice convention (bank b holds register bits
  // [bankWidth*(b+1)-1 : bankWidth*b]).
  for (p <- 0 until numReadPorts) {
    io.read_data(p) := RegNext(Cat(
      bank(3).io.read_data(p), bank(2).io.read_data(p),
      bank(1).io.read_data(p), bank(0).io.read_data(p)))
  }

  // ---- Tracing: per-access read trace lines (logic §7) ----
  //
  // Request-cycle line: one per valid read port, in its REQUEST cycle
  // ("[vec] VecRegFile read_req port=.. prn=.. rob=.."). Rung 2 (`traceId`)
  // when the port's trace-only `rob_idx` is valid, else rung 3
  // (`traceStruct`, `rob=?`) -- see file header.
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

  // Response-cycle line: one per valid read port, in its RESPONSE cycle,
  // with the low bits of the returned data, correlated by port number
  // ("[vec] VecRegFile read_rsp port=.. data=.. rob=.."). `read_req_valid_r`
  // and `read_rob_r` carry the request-cycle fields one cycle forward so
  // this line can be tagged to the same request -- TRACE-ONLY registers per
  // the nlhdl logic section ("any register carrying the request fields into
  // the response cycle is trace-only"): they feed no functional logic, so
  // deleting this tracing block (or running with the plusarg unset) leaves
  // the design's cycle-by-cycle behavior bit-identical.
  //
  // ASSUMPTION: the spec asks for "the low bits of the returned data" but
  // does not size them; 32 bits is chosen as enough to distinguish values in
  // a debug trace without printing a full vLen=256-bit decimal every line.
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

  // For write port w: `valid` and `bits.addr` are broadcast unchanged to all
  // four banks, while `bits.data` and `bits.mask` are SLICED per bank. A
  // write whose mask is all zero in a given bank still arrives there with
  // `valid` set; the bank enables no byte.
  //
  //@req-spec-lsu.m7
  //@req-spec-lsu.m6
  // ===> NO MUX ANYWHERE IN THIS LOOP, in either direction. `io.write` is
  // wired unqualified: no grant term, no `!valid` qualification, no priority
  // encoder. R2/W0's group-copy sharing (A16) and any other upstream
  // arbitration is fully resolved before it reaches `io.write` -- one
  // address and one write arrive here, never two competing ones.
  for (b <- 0 until numBanks; w <- 0 until numWritePorts) {
    bank(b).io.write_ports(w).valid      := io.write(w).valid
    bank(b).io.write_ports(w).bits.addr  := io.write(w).bits.addr
    bank(b).io.write_ports(w).bits.data  := io.write(w).bits.data(bankWidth * (b + 1) - 1, bankWidth * b)
    bank(b).io.write_ports(w).bits.mask  := io.write(w).bits.mask(bankBytes * (b + 1) - 1, bankBytes * b)
  }

  // ---- Tracing: per-access write trace line (logic §7) ----
  //
  // One line per valid write port ("[vec] VecRegFile write port=.. prn=..
  // mask=.. data=.. rob=.."). A write has no response and needs no second
  // line; `mask` is included because it is what distinguishes a sub-lane
  // `vlm.v`-style write from a corrupted full-width one. Same rung rule as
  // the read lines: `traceId` when the port's trace-only `rob_idx` is
  // valid, else `traceStruct` (`rob=?`).
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
  // A plain wire off the registered response, independent of the trace
  // gate -- an observation of ports that already exist, feeding nothing
  // inside the design. ASSUMPTION: this assumes coreWidth <= numReadPorts
  // (true at every tier this design targets); the nlhdl source states the
  // mapping directly ("lane i is io.read_data(i)") without itself bounding
  // coreWidth against numReadPorts, so no additional require is added here
  // beyond what the Vec index bounds already enforce at elaboration.
  for (i <- 0 until coreWidth) {
    io.debug_vrf_read(i) := io.read_data(i)
  }
}

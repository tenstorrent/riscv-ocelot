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
//
// VecRegFileBank -- one width slice of the vector register file: a
// `numVecPhysRegisters x bankWidth` array of standard-cell flip-flops
// carrying all `numReadPorts` reads and `numWritePorts` writes at
// `bankWidth = vLen / numBanks` bits. Instantiated FOUR times, as `bank`, by
// VecRegFile, which owns the vLen-wide concatenation of the four banks' data
// and the slicing of a whole-VRF write's data/mask into per-bank fields. This
// module treats its ports symmetrically and by index only -- it does not name
// any functional unit, and the 9R/3W port partition (midcore.rst
// `vrf-ports`) is VecRegFile's contract, not this module's.
//
// ===> BANKING IS BY WIDTH, NOT BY REGISTER INDEX -- the opposite of BOOM's
//      scalar `BankedRF` (v4/exu/register-read/regfile.scala), which banks
//      the register *count*. Here every bank holds a `bankWidth`-bit slice of
//      EVERY PRN, so the bank's address is the full `vecPregSz`-bit PRN,
//      unshifted, with NO bank-select comparison and NO per-bank port
//      subset. Several read ports addressing this bank in the same cycle is
//      the normal case, not a conflict.
//
// ===> READ-DURING-WRITE FORWARDING IS A CORRECTNESS REQUIREMENT, NOT AN
//      OPTIMIZATION. Reads are single-cycle and combinational; there is no
//      bypass network above the vector register file to cover for its
//      absence.
//
// Governing spec anchors: midcore.rst `vector-regfile` (the whole of this
// file), midcore.rst `vrf-ports` (the partition this bank serves but does not
// define), midcore.rst `regfiles-bypass` (the 96-PRN / 24 kbit sizing
// figures), loadstore.rst `load-coalesce` (the LCB, sole driver of W0/W1).

/**
 * One write-port payload: destination PRN, this bank's slice of the write
 * data, and a per-BYTE write-enable mask (`bankBytes` bits, NOT one bit per
 * 64b lane -- required so a sub-lane write, e.g. `vlm.v`'s `ceil(vl/8)`
 * bytes, or a partial tail, leaves the rest of the destination undisturbed).
 * This is the `.bits` payload of each `Flipped(Valid(...))` write port.
 */
class VecRegFileBankWritePort(bankWidth: Int, bankBytes: Int)(implicit p: Parameters) extends BoomBundle
{
  val addr = UInt(vecPregSz.W)
  val data = UInt(bankWidth.W)
  val mask = UInt(bankBytes.W)
}

class VecRegFileBankIO(bankWidth: Int, bankBytes: Int, numReadPorts: Int, numWritePorts: Int)(implicit p: Parameters) extends BoomBundle
{
  // Read ports: no handshake -- no valid, no ready, no Decoupled. Ports are
  // statically partitioned by VecRegFile, so a read is an unconditional
  // combinational lookup of whatever address is presented; an idle port's
  // result is simply ignored by the caller.
  val read_addr   = Vec(numReadPorts, Input(UInt(vecPregSz.W)))
  val read_data   = Vec(numReadPorts, Output(UInt(bankWidth.W)))

  // Write ports: VecRegFile broadcasts `valid`/`addr` unchanged to all four
  // banks and slices `data`/`mask` per bank. A write whose mask is zero in
  // this bank still arrives with `valid` set; the bank simply enables no
  // byte -- this keeps the four banks identical and keeps the write decoder
  // out of the mask's timing path.
  val write_ports = Vec(numWritePorts, Flipped(Valid(new VecRegFileBankWritePort(bankWidth, bankBytes))))

  // Deliberately no debug read port here: VecRegFile builds `debug_vrf_read`
  // from the ordinary read ports; a private debug port here would be a
  // thirteenth port on the array, which the port table forbids.
}

/**
 * VecRegFileBank -- see file header for the full design rationale.
 * Instantiated FOUR times, as `bank`, by VecRegFile.
 *
 * `bankId` identifies which width slice this instance is. It is permitted in
 * exactly two places: elaboration-time `require`s and trace/assertion
 * message strings -- it must never appear in the datapath, since the four
 * instances are structurally identical and VecRegFile distinguishes them
 * purely by which bit slice it wires to each.
 *
 * `numBanks`, `numReadPorts` and `numWritePorts` are Scala values resolved at
 * elaboration and supplied by VecRegFile, which derives them from the
 * canonical port table -- this module does not recompute them and does not
 * know which unit any read/write index belongs to.
 */
class VecRegFileBank(
  val bankId:        Int,
  val numBanks:      Int = 4,
  val numReadPorts:  Int = 9,
  val numWritePorts: Int = 3
)(implicit p: Parameters) extends BoomModule
{
  // The module's existence, not its content, is gated on usingRVV (the
  // Scala Boolean from BoomCoreParams -- NOT rocket's `usingVector`):
  // VecRegFile, this module's only instantiator, exists only under that
  // gate, so a vectors-off build contains no instance of this module at all.
  require(usingRVV, s"VecRegFileBank (bankId=$bankId) instantiated with usingRVV=false")

  //@req-spec-vrf.f5
  //@req-spec-vrf.f10
  // numBanks is fixed at 4: the banked architecture, and specifically the
  // count 4, is a spec obligation and not a tuning knob. Exposed as a
  // parameter only so bankWidth/bankBytes derive from one name instead of a
  // literal 4; checked here rather than assumed.
  require(numBanks == 4, s"VecRegFileBank (bankId=$bankId): numBanks ($numBanks) must be 4")

  // bankId in 0 until numBanks -- one of the two places bankId is permitted
  // to appear (an elaboration-time require).
  require(bankId >= 0 && bankId < numBanks,
    s"VecRegFileBank: bankId ($bankId) out of range [0, $numBanks)")

  // ASSUMPTION: the nlhdl parameters section states "Legal range for writes
  // is 2 to 3" for numWritePorts but does not explicitly say to `require` it
  // (unlike numBanks, which does say "checked with require(numBanks == 4)").
  // Following this file's own VectorParams convention -- every other stated
  // "Legal range" in this codebase is enforced by a require -- this is
  // checked too rather than left as an unenforced comment.
  require(numWritePorts >= 2 && numWritePorts <= 3,
    s"VecRegFileBank (bankId=$bankId): numWritePorts ($numWritePorts) must be in [2, 3]")

  //@req-spec-vrf.f11
  // bankWidth = vLen / numBanks (64 b at vLen = 256, via HasBoomCoreParameters'
  // `vecVLen`, itself derived from VectorParams). The 64-bit figure quoted in
  // the spec is this formula's vLen=256 instance, not an independent
  // constant, so nothing below writes 64 literally. vLen is already required
  // to be a power of two by rocket-chip, so bankWidth is too.
  require(vecVLen % numBanks == 0,
    s"VecRegFileBank (bankId=$bankId): vecVLen ($vecVLen) must be a multiple of numBanks ($numBanks)")
  val bankWidth = vecVLen / numBanks

  // bankBytes = bankWidth / 8 is the number of write-mask bits (8 at default).
  val bankBytes = bankWidth / 8

  val io = IO(new VecRegFileBankIO(bankWidth, bankBytes, numReadPorts, numWritePorts))

  // ---- Storage ----

  //@req-spec-vrf.f4
  //@req-spec-vrf.f12
  // Reg(Vec(...)) -- an array of standard-cell flip-flops. No SyncReadMem, no
  // Mem, no vendor SRAM macro and no latch array: an SRAM's registered read
  // would break the single-cycle read contract below, and a memory macro
  // cannot present numReadPorts + numWritePorts independent ports.
  // numVecPhysRegisters (from VectorParams, reached via the
  // HasBoomCoreParameters mixin as `numVecPhysRegs`) is the array depth --
  // the SAME depth in every bank; the four banks together form the
  // numVecPhysRegisters x vLen array the spec calls for by each holding a
  // numVecPhysRegisters x bankWidth slice of it. Banking divides the WIDTH,
  // never the depth.
  //
  // No reset value (Reg, not RegInit): RVV requires software to establish
  // vector state before reading it, and mstatus.VS gates the file's use
  // until it does, so reset-initializing this array would fan the reset net
  // into every bit and buy nothing architecturally. usingRVV controls the
  // array's EXISTENCE, not its content.
  val vrf_bank = Reg(Vec(numVecPhysRegs, UInt(bankWidth.W)))

  //@req-spec-vrf.f6
  //@req-spec-vrf.f11
  // Bit-slice contract, stated because both this module and VecRegFile index
  // it: a vector register splits across the four banks of bankWidth bits,
  // little-endian, bank b holding register bits
  // [bankWidth*(b+1)-1 : bankWidth*b]. Byte j of bank b is byte
  // bankBytes*b + j of the register, i.e. element bankBytes*b + j at SEW=8 --
  // element order and byte order agree. `expandByteMask` encodes that same
  // byte convention: mask bit 0 covers the LOWEST byte of this bank's slice
  // (bits [7:0]).
  private def expandByteMask(mask: UInt): UInt =
    Cat((0 until bankBytes).reverse.map(j => Fill(8, mask(j))))

  // ---- Write path (sequential) ----

  //@req-spec-vrf.f7
  // Each write port has its OWN decoder in this bank: a one-hot
  // UIntToOH(addr, numVecPhysRegisters) qualified by that port's valid. No
  // shared decoder and no shared address bus across ports.
  val write_one_hot: Seq[UInt] = io.write_ports.map { wp =>
    Mux(wp.valid, UIntToOH(wp.bits.addr, numVecPhysRegs), 0.U(numVecPhysRegs.W))
  }

  //@req-spec-vrf.f13
  //@req-spec-vrf.f7
  // On the rising edge of clock, for every register r and every byte j, byte
  // j of vrf_bank(r) updates from write port w if that port's decoder
  // selects r and its mask bit j is set. Because two write ports can never
  // target the same PRN (asserted below), the per-byte enables of different
  // ports are mutually exclusive at any given r, so this is a plain
  // per-byte-masked OR-reduction over the write ports -- NOT a priority mux
  // and NOT an arbiter, and no write port can be refused or delayed.
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
  // That exclusivity is imposed on this bank from outside, so it is
  // ASSERTED, not assumed silently -- mirrors the scalar RegisterFile's "too
  // many writers a register" assertion. If it fires, the defect is in rename
  // or in the LCB/CII write routing, never here, and the fix is not to add
  // priority to this module. (bankId in a message string -- the second of
  // the two places it is permitted to appear.)
  for (i <- 0 until numWritePorts; j <- (i + 1) until numWritePorts) {
    assert(!(io.write_ports(i).valid && io.write_ports(j).valid &&
      io.write_ports(i).bits.addr === io.write_ports(j).bits.addr),
      s"VecRegFileBank bankId=$bankId: write ports $i and $j target the same PRN " +
      "in the same cycle")
  }
  // The assert is the enforcement of the property that licenses the
  // OR-reduce write path. Weakening it to a priority mux would hide a rename
  // bug.

  // ---- Read path (combinational, single cycle) ----

  // Per-port "did this read forward this cycle" wire, consumed only by the
  // tracing block below -- functional logic never reads it (see Tracing).
  val read_forwards = Wire(Vec(numReadPorts, Bool()))

  for (p <- 0 until numReadPorts) {
    //@req-spec-vrf.f7
    //@req-spec-vrf.f14
    // Each read port also has its own decoder in this bank: port p's data is
    // vrf_bank(read_addr(p)), a numVecPhysRegisters-to-1 select of bankWidth
    // bits, independent of every other port's. All numReadPorts exist in
    // every bank simultaneously; no port is dropped, gated by an address
    // bit, or shared with another port.
    val arrayData = vrf_bank(io.read_addr(p))

    //@req-spec-vrf.f8
    // Read-during-write forwarding, per read port, combinationally in the
    // same cycle: if any write port is valid with addr === read_addr(p),
    // read_data(p) is the byte-wise merge of that port's data (bytes whose
    // mask bit is set) with the array output (the rest) -- the same
    // per-byte merge the write path commits at the end of the cycle. Two
    // writers cannot name one PRN, so at most one write port can hit any
    // given read address: an OR of mutually exclusive per-byte selects, not
    // a priority chain.
    val fwdHits = io.write_ports.map(wp => wp.valid && wp.bits.addr === io.read_addr(p))
    val fwdEnable = io.write_ports.zip(fwdHits).map { case (wp, hit) =>
      Mux(hit, expandByteMask(wp.bits.mask), 0.U(bankWidth.W))
    }.reduce(_ | _)
    val fwdData = io.write_ports.zip(fwdHits).map { case (wp, hit) =>
      Mux(hit, wp.bits.data & expandByteMask(wp.bits.mask), 0.U(bankWidth.W))
    }.reduce(_ | _)

    //@req-spec-vrf.f8
    // The forwarding is unconditional, not predicated on any claim about how
    // far apart a producer's write and a consumer's read must be -- it is
    // part of the port's definition; removing it because a regression still
    // passes would be a failed review.
    io.read_data(p) := (arrayData & ~fwdEnable) | fwdData

    read_forwards(p) := fwdHits.reduce(_ || _)
  }

  // ---- What this module does not contain ----
  //
  // No state other than vrf_bank. No busy output, no credit, no reservation
  // counter, nothing scoped to "the current instruction" -- this module
  // cannot stall or be stalled and exports no signal that could gate an
  // issue unit. No pipeline register on either path.

  // ---- Tracing ----
  //
  // Ground rule 11 (guarded printf, `vecTrace` plusarg, off by default). This
  // bank has no MicroOp context, so it cannot use VecTrace.trace or its
  // wrappers -- only the shared VecTrace.traceEnabled gate (per the
  // dependencies section). Emits at most one line per cycle, for the
  // lowest-indexed read port that forwarded this cycle, tagged with module
  // name, bankId, read-port index and PRN. Neither `read_forwards` above nor
  // the wires below feed any functional logic -- deleting this block (or
  // running with the plusarg unset) leaves the design's cycle-by-cycle
  // behavior bit-identical.
  //
  // ASSUMPTION: the spec says "at most one line per cycle in which a read
  // forwards" but does not specify a selection rule when more than one read
  // port forwards in the same cycle. Lowest-index-wins (PriorityEncoder) is
  // chosen as the most conservative deterministic rule.
  val any_forward  = read_forwards.reduce(_ || _)
  val fwd_port_idx = PriorityEncoder(read_forwards)
  when (VecTrace.traceEnabled && !reset.asBool) {
    when (any_forward) {
      printf("[vec] VecRegFileBank bankId=%d read_fwd port=%d prn=%d\n",
        bankId.U, fwd_port_idx, io.read_addr(fwd_port_idx))
    }
  }
}

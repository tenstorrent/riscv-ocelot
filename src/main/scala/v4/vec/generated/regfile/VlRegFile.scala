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
//
// VlRegFile -- the VL physical register file: `numVlPhysRegs` entries of
// `vecVLSz` bits, the ONLY place a vector VL value is ever stored or read
// from. Instantiated exactly once, by VecPipeline, as `vlrf`. Elaborated
// only when `usingRVV` is true -- by virtue of the parent only instantiating
// it under that condition (Gate (f)); this module performs no internal
// `usingRVV` gating of its own, the same convention the decode-stage
// siblings use.
//
// WHY THIS IS A SEPARATE MODULE FROM VecRegFile. The two share a
// rename-space DEFINITION (VecRenameSpace, instantiated twice by VecPipeline
// as `vec_rename` and `vl_rename`) and nothing else. `numVlPhysRegs x
// vecVLSz` of flops with a handful of trivial ports has no structure in
// common with the VRF's `numVecPhysRegs x vLen`, banked four ways with
// twelve ports, where port count is the dominant area term and the
// flop-vs-latch-vs-SRAM question is the design's #2 risk. Rename logic is
// shared; STORAGE is not. Do NOT merge these, and do NOT copy
// VecRegFileBank's read-during-write bypass in here -- see the no-bypass
// note below for why this file does not need one.
//
// WHAT THIS MODULE IS NOT. It holds storage and nothing else. No map table,
// no free list, no busy table, no wakeup broadcast, no branch snapshot, no
// `brupdate` port and no flush port. All of that is VecRenameSpace's
// `vl_rename` instance. This module exports no `busy` and no `ready` of any
// kind.
//
// TRACING -- the three mandated WRITE lines are emitted via VecTrace's
// three-step ladder (`trace*` -> `traceId` -> `traceStruct`; VecTrace.scala's
// "two uOP-less variants" note). This module's ports, per the nlhdl ports
// section, are bare `Valid` bundles of `addr`/`data` -- no MicroOp anywhere
// on any write or read port, and this module's `depends_on` (VectorParams,
// VecTrace) deliberately does not include MicroOp -- so the first rung
// (`trace`/`traceVl`/...) is unreachable without inventing a MicroOp, which
// this flow's ground rules forbid. There is also no `rob_idx` on any port,
// so the second rung (`traceId`) is unreachable too, for the same reason
// `VecRegFileBank` (a sibling storage/lookup structure with the same
// uOP-less, rob_idx-less boundary) uses the third rung. Every one of the
// three write events -- `wr_ren`, `wr_alu`, `wr_lsu` -- is therefore
// emitted with `VecTrace.traceStruct`, keyed on `prn` (the write/read
// `addr`, i.e. the `pvl` the event concerns) and `vl` (the `data`
// written/read). No port was added to reach a higher rung -- see the ports
// section's binding note and the nlhdl's own admonition against acquiring a
// port purely to be traceable.
//
// Governing spec anchors: frontend.rst `vector-rvv-decode` ("VSET Special
// Handling") and `vl-delivery`, midcore.rst `vl-vtype-rename` and
// `regfiles-bypass`, issue.rst `issue-vl-delivery`, loadstore.rst
// `elem-progress` ("Fault-only-first").

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
  // The write side is a fixed table, not an arbiter: exactly one write port
  // per PRODUCER CLASS, and within a class one port per producer instance.
  // No `ready` on any of them -- the ALU and LSU writeback paths have NO
  // back-pressure line, so a shared or arbitrated write port would be a
  // structural hazard with nowhere to report it. REPLICATE, NEVER ARBITRATE.

  //@req-spec-decode.c29
  // W_ren is replicated PER RENAME LANE because one dispatch bundle may
  // legally hold several `vsetivli` -- `[vsetivli, vadd, vsetivli, vadd]` is
  // the spec's own example -- and each lane allocates its own distinct
  // `pvl` in the same cycle. A single shared rename-side port would drop
  // the second one silently.
  val w_ren = Input(Vec(coreWidth, Valid(new VlRegFileWriteData)))

  //@req-spec-decode.c28
  // D8: ONE write port per integer ALU EU, lane-aligned with `vset_resp`
  // (`Vec(aluWidth, Valid(ExeUnitResp))`) -- `numAluWritePorts == aluWidth`,
  // never one, because `aluWidth` is never 1 anywhere in the vector matrix
  // (Small is out per D3). Two `vsetvli`/`vsetvl` writebacks in the same
  // cycle is a NORMAL case that arbitration must never be allowed to drop
  // (a lost single-shot VL wakeup is a permanent hang, not a stall).
  val w_alu = Input(Vec(numAluWritePorts, Valid(new VlRegFileWriteData)))

  // `vleff` completion write port(s). Declared from day one (decode.c27's
  // "one port per producer class", never retrofit an arbiter later); see
  // the module body for why it never fires yet.
  val w_lsu = Input(Vec(numLsuWritePorts, Valid(new VlRegFileWriteData)))

  // ---- Read ports ----
  val r_exe    = Vec(numExeReadPorts, new VlRegFileReadPort)
  val r_commit = new VlRegFileReadPort

  // Deliberately absent: no `brupdate` input, no `rob_flush` input, no busy
  // or ready output, no wakeup output, no debug read port. Speculation
  // recovery is entirely a rename-space (`vl_rename`) concern -- a
  // wrong-path write lands in a PRN no surviving consumer names, so the
  // stored value becomes unreachable rather than wrong.
}

/**
 * VlRegFile ("vlrf") -- see the file header for the full design rationale.
 * Instantiated once by `VecPipeline`.
 */
class VlRegFile(implicit p: Parameters) extends BoomModule
{
  // ---- Port-count parameters (all derived, none a literal chosen here) ----

  //@req-spec-decode.c28
  // `numAluWritePorts` is NOT a default this file chooses: it is `aluWidth`
  // from `HasBoomCoreParameters` (`aluIssueParam.issueWidth`), because the
  // BoomCore-to-VecPipeline interface presents `vset_resp` as
  // `Vec(aluWidth, Valid(ExeUnitResp))` and every lane of it lands on its
  // own W_alu port here (decision D8).
  val numAluWritePorts: Int = aluWidth

  // `vleff` completion write ports. Default 1; VecLsu retires at most one
  // `vleff` group-done per cycle.
  val numLsuWritePorts: Int = 1

  // Execute-stage read ports: one per vector issue queue (IQ_V_LOAD -> load
  // AGEN, IQ_V_STORE -> store AGEN, IQ_V_ALU -> the CII host), i.e. one per
  // grant lane that can start a vector uOP in a cycle. A tier raising
  // `vecIssueGrantWidth` raises this proportionally (nlhdl parameters
  // section); at `vecVLSz` bits per entry a read port is nearly free, so
  // replicate rather than share.
  val numExeReadPorts: Int = 3 * vectorParams.vecIssueGrantWidth

  val io = IO(new VlRegFileIO(numAluWritePorts, numLsuWritePorts, numExeReadPorts))

  // =========================================================================
  // ---- Storage ----
  // =========================================================================

  //@req-spec-decode.i1
  //@req-spec-rename.h4
  //@req-spec-vrf.c1
  // A single register array of `numVlPhysRegs` entries, each `vecVLSz` bits:
  // a dedicated VL register file, in its own rename space, with exactly one
  // architectural register (`vl`) behind it. It is NOT a slice of the
  // integer file, NOT a slice of the VRF, and NOT a CSR mirror -- BOOM's
  // integer rename, free list, busy table and bypass network are untouched
  // by Caracal, which could not have been true if VL had been renamed as a
  // GPR. Plain flops with per-port decoders; no banking (each entry is only
  // `vecVLSz` bits -- there is nothing to bank) and no SRAM.
  //
  // Reset the ENTIRE array to 0 on `reset`: negligible cost at these sizes,
  // and it buys two things -- the entry the committed VL pointer names
  // after reset reads as VL=0, matching `vl`'s architectural reset value,
  // and no read can ever return X. (Unlike the VRF, which deliberately does
  // NOT reset its storage -- the trade is different at this size, and
  // X-cleanliness matters more here: a VL of X does not corrupt one lane, it
  // makes every downstream element count meaningless.)
  val vl_rf = RegInit(VecInit(Seq.fill(numVlPhysRegs)(0.U(vecVLSz.W))))

  // ---- Capacity check on the entry width (elaboration-time, not a req) ----
  // vecVLSz must be wide enough to hold the largest representable VL
  // (maxVecVL = vLen * maxMembers / 8, i.e. 256 at the defaults, needing 9
  // bits). HasVectorParams already enforces this on `vecVLSz` itself
  // (VectorParams.scala); restated here, naming `vecVLSz` directly, per this
  // file's own elaboration-time contract -- a too-narrow entry does not fail
  // loudly, it truncates VL=256 to 0 and the machine silently executes a
  // zero-length vector op.
  require(vecVLSz >= log2Ceil(maxVecVL + 1),
    s"VlRegFile: vecVLSz ($vecVLSz) is too narrow to hold maxVecVL ($maxVecVL)")

  // =========================================================================
  // ---- The rename-cycle write (`vsetivli`) ----
  // =========================================================================

  //@req-spec-decode.c4
  //@req-spec-issue.h4
  // `vsetivli` is FRONT-END ONLY: both `vtype` and AVL are immediate, so
  // VConfigUnit resolves it at decode and computes VL = min(uimm, VLMAX).
  // The resulting value is written to VL_RF[pvl] in the RENAME cycle, NOT at
  // decode, for a mechanical reason: `pvl` is allocated by `vl_rename` at
  // rename, so at decode there is no VL-RF index to write to. W_ren[w]
  // therefore carries the decode-computed VL forward one stage and writes
  // it where the index becomes known. No back-end issue slot and no EU are
  // involved.
  //
  // LOCKSTEP (upstream contract, not enforced by this module): `io.w_ren(w)
  // .valid` must already be qualified, by the producer, with the SAME
  // `dis_fire(w)` that qualifies `vl_rename`'s allocation on lane w -- this
  // module applies no gating of its own beyond `valid` and trusts that
  // qualification.
  //
  // The write sets no busy bit -- there is no busy state in this module at
  // all -- and `vl_rename` leaves `pvl_busy` clear for a `vsetivli`, so its
  // `pvl` is BORN READY and a dependent vector uOP never waits on it; the
  // uOP has no writeback, so its ROB entry is dispatched non-busy.
  for (w <- 0 until coreWidth) {
    when (io.w_ren(w).valid) {
      vl_rf(io.w_ren(w).bits.addr) := io.w_ren(w).bits.data
      // Tracing (see file header): no MicroOp/rob_idx on this port ->
      // traceStruct, keyed on the written `pvl` (`prn`) and its new value.
      VecTrace.traceStruct("VlRegFile", "wr_ren",
        Seq(("prn", io.w_ren(w).bits.addr), ("vl", io.w_ren(w).bits.data)))
    }
  }

  // =========================================================================
  // ---- The ALU writeback (`vsetvli` / `vsetvl`) ----
  // =========================================================================

  // `vsetvli` and `vsetvl` take VL (and, for `vsetvl`, `vtype`) from a GPR,
  // so they execute on an integer ALU EU, woken by `rs1`/`rs2` on the
  // INTEGER wakeup network. The ALU computes VL = min(rs1, VLMAX) and its
  // writeback targets this port with `is_vl_producer` (not `dst_rtype`) as
  // the write enable upstream -- with `rd == x0` the uOP's `dst_rtype` is
  // `RT_ZERO` and the VL RF is still written. The same result bus feeds the
  // integer RF when `dst_rtype === RT_FIX`, because `rd` receives the new
  // `vl`; that write is unrelated to this module and is not visible here.
  //
  // There are `numAluWritePorts` (== `aluWidth`) of these, not one (D8), and
  // lane `i` is wired one-to-one from ALU EU `i`'s `vset_resp`, with NO mux
  // and NO arbiter between lanes: both writes land at the same posedge into
  // two different entries when two `vset`s retire together, and both
  // `pvl`s go out on the VL wakeup network (elsewhere) on their own lanes,
  // so neither single-shot wakeup can be lost.
  for (w <- 0 until numAluWritePorts) {
    when (io.w_alu(w).valid) {
      vl_rf(io.w_alu(w).bits.addr) := io.w_alu(w).bits.data
      // Tracing (see file header): traceStruct, keyed on the written `pvl`
      // (`prn`) and its new value. Both lanes may fire the same cycle (D8);
      // each lane's call is independently guarded on its own `.valid`, so
      // two same-cycle `vset` writebacks emit two distinct lines.
      VecTrace.traceStruct("VlRegFile", "wr_alu",
        Seq(("prn", io.w_alu(w).bits.addr), ("vl", io.w_alu(w).bits.data)))
    }
  }

  // =========================================================================
  // ---- The `vleff` writeback ----
  // =========================================================================

  //@req-spec-lsu.g6
  // `vleff.v` is the third producer class. On completion VecLsu writes the
  // final element count through W_lsu to the `vleff`'s own VL-RF
  // destination -- the full VL if no element faulted, or `i` if element
  // `i > 0` faulted and VL was trimmed. Bit-for-bit the same transaction a
  // `vset` performs. A fault on element 0 is a normal precise trap instead
  // and writes nothing here.
  //
  // STAGING, not a contradiction: this port is declared from day one and
  // stays permanently invalid until the real fault-trim path lands in step
  // G4 (VLSDecode's hierarchy.yaml note) -- decode.c27 asks for one port per
  // producer CLASS, and retrofitting this port later would be the change
  // most likely to be done as an arbiter on W_alu instead, which decode.c27
  // forbids.
  for (w <- 0 until numLsuWritePorts) {
    when (io.w_lsu(w).valid) {
      vl_rf(io.w_lsu(w).bits.addr) := io.w_lsu(w).bits.data
      // Tracing (see file header): traceStruct, keyed on the written `pvl`
      // (`prn`) and its new (trimmed) value. Currently dead in any case --
      // this port never fires until step G4 lands.
      VecTrace.traceStruct("VlRegFile", "wr_lsu",
        Seq(("prn", io.w_lsu(w).bits.addr), ("vl", io.w_lsu(w).bits.data)))
    }
  }

  // ---- No two ports may target the same entry (assertion, not a req) ----
  // In any cycle every valid write port carries a DISTINCT `pvl`: each
  // producer writes a PRN `vl_rename`'s free list handed it, not reused
  // until commit frees it. The write path relies on that -- a per-port
  // decoder feeding a plain per-entry enable, no priority mux and no
  // defined winner -- so assert it pairwise over ALL write ports, W_alu
  // lanes against each other included (two ALU EUs retiring `vset`s in the
  // same cycle, D8, is the case most likely to be omitted from a
  // hand-written check).
  val allWritePorts: Seq[Valid[VlRegFileWriteData]] = io.w_ren ++ io.w_alu ++ io.w_lsu
  for (i <- allWritePorts.indices; j <- (i + 1) until allWritePorts.length) {
    assert(!(allWritePorts(i).valid && allWritePorts(j).valid) ||
           (allWritePorts(i).bits.addr =/= allWritePorts(j).bits.addr),
      s"VlRegFile: write ports $i and $j targeted the same pvl in the same cycle")
  }

  // ---- No read-during-write bypass (documentation, not a req) ----
  // Writes are registered; a read sees a written entry from the NEXT cycle.
  // No forwarding from any write port into any read port is needed, because
  // every producer-to-consumer path has at least one cycle between the
  // write and the consumer's execute read: a rename-cycle write is followed
  // by dispatch, issue and register-read, and an ALU or LSU writeback
  // broadcasts `pvl` on the VL wakeup network in the cycle it writes, after
  // which the woken slot still has to be granted. This is the OPPOSITE of
  // VecRegFileBank, where read-during-write forwarding is a stated
  // requirement -- do not copy that bypass here.
  //
  // SEAM CONSTRAINT, not an optimization: if any tier ever adds a
  // speculative or fast VL wakeup that lets a consumer read in the SAME
  // cycle as the write, this module needs a bypass mux and that change must
  // come back here.

  // =========================================================================
  // ---- The read side is the ONLY way VL leaves this module ----
  // =========================================================================

  //@req-spec-decode.i17
  //@req-spec-decode.i18
  //@req-spec-rename.h25
  // No broadcast, no snapshot, no side channel into the uOP, no path into
  // the VCFG mirror, and NO statically-known-VL shortcut for `vsetivli`'s
  // compile-time-constant AVL case (a `vl_is_known`/`vl_imm` MicroOp field is
  // on MicroOp's reject list, for exactly this reason: it would create a
  // second source of truth for VL that the keep-VL form of `vsetvli`, a
  // mispredict, or a `vleff` trim could each falsify independently). One
  // producer path, one storage array, one read path.

  //@req-spec-decode.i7
  //@req-spec-rename.h23
  // Execute-stage reads are COMBINATIONAL: address presented in cycle N,
  // data valid in cycle N, the SAME cycle, with no `valid`, no `ready`, no
  // enable, no output flop and no read-during-write bypass. An EU that does
  // not need VL this cycle simply ignores the data; a consumer that wants
  // the value a cycle later registers it on its OWN side. This is
  // deliberately different from the VRF's registered one-cycle read -- a
  // sizing fact, not an inconsistency, at `numVlPhysRegs x vecVLSz`. NOT
  // traced: these reads are unconditional and would emit every cycle per
  // port, burying the events that matter (nlhdl "Tracing" section).
  for (i <- 0 until numExeReadPorts) {
    io.r_exe(i).data := vl_rf(io.r_exe(i).addr)
  }

  //@req-spec-lsu.g8
  // The commit-side read: at commit of a VL producer the ROB reads
  // VL_RF[committed pvl] here, and that value is what updates the
  // architectural `vl` CSR in rocket's CSRFile (through `csr.io.vector`'s
  // `set_vconfig`). A separate port from R_exe rather than a borrowed one
  // because commit must never contend with execute. ONE port, not
  // `coreWidth` of them: when a commit bundle retires several VL producers
  // only the youngest one's VL becomes architectural, so the ROB side
  // selects that lane's `pvl` and drives this single address -- that select
  // is the ROB's, not this module's.
  io.r_commit.data := vl_rf(io.r_commit.addr)
  // READS ARE NOT TRACED -- neither R_exe nor R_commit. Both are bare
  // `addr`/`data` pairs with no valid or enable, so a line on either fires
  // every cycle per port and buries the events that matter. The nlhdl used to
  // mandate an `rd_commit` line here and generation implemented it literally,
  // which made it fire unconditionally; gating it would need an enable added
  // SOLELY to make a trace line emit, which the VecTrace spec forbids. The
  // commit event is observable from the consumer side, where a real `rob_idx`
  // exists (the ROB / VConfigUnit commit path can use `traceId`). Spec
  // corrected 2026-08-10; see the nlhdl Tracing paragraph's `// ===>` note.
}

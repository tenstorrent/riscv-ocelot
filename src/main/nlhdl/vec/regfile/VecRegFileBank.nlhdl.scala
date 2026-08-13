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

/*
  VecRegFileBank — one width slice of the vector register file: a
  `numVecPhysRegisters x (vLen/4)` array of standard-cell flip-flops carrying all
  12 VRF ports at `vLen/4` bits.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/regfile/VecRegFileBank.scala,
  package boom.v4.vec.generated.regfile,
  depends_on VectorParams, VecTrace.
  Instantiated FOUR times, as `bank`, by VecRegFile.

  This node owns HOW the array is built — flops, banking, per-port decoders,
  read-during-write forwarding, the per-byte write mask. It does NOT own WHO owns
  which port: the 9R/3W partition (R0 index, R1 load mask, R2 stale_pvdest,
  R3 store data, R4 store mask/index, R5-R8 CII, W0/W1 load-LCB, W2 CII) is
  canonical in midcore.rst `vrf-ports` and is VecRegFile's contract. A bank
  treats its ports symmetrically and by index only; it must not name a
  functional unit anywhere. Nothing here adds a port.

  ===> BANKING IS BY WIDTH, NOT BY REGISTER INDEX. This is the opposite of
       BOOM's scalar `BankedRF` in v4/exu/register-read/regfile.scala, which
       banks the register *count* and therefore computes a bank select
       (`bankIdx(addr)`) and shifts the address (`addr >> log2Ceil(numBanks)`).
       Here every bank holds a 64-bit slice of EVERY PRN, so the bank's address
       is the full `vecPregSz`-bit PRN, unshifted, and there is NO bank-select
       comparison and NO per-bank port subset. Reusing the scalar pattern here
       would be functionally wrong and would quietly drop three quarters of
       every read. This is also what makes the whole read-port-conflict question
       vacuous: several read ports addressing one bank is the normal case.

  ===> READ-DURING-WRITE FORWARDING IS A CORRECTNESS REQUIREMENT, NOT AN
       OPTIMIZATION. Reads are single-cycle and combinational, so a write
       landing on the PRN being read in the same cycle must be forwarded to that
       read port's output. There is no bypass network above the vector register
       file to cover for its absence — vector operands wake on actual
       completion and their consumers read the VRF directly.

  Governing spec anchors: midcore.rst `vector-regfile` (the whole of this file),
  midcore.rst `vrf-ports` (the partition this bank serves but does not define),
  midcore.rst `regfiles-bypass` (the 96-PRN / 24 kbit sizing figures),
  loadstore.rst `load-coalesce` (the LCB, sole driver of W0/W1).

<|begin_module|>

  <|begin_parameters|>
  All parameters are Scala values resolved at elaboration, read from
  VectorParams / `HasVectorParams` rather than restated as literals. The module
  is elaborated only inside a `usingRVV` build: VecRegFile, its only
  instantiator, exists only under that gate, so a vectors-off build contains no
  instance of this module at all — absent, not tied off.

  //@req-spec-vrf.f12
  `numVecPhysRegisters` is the number of vector physical registers, default 96
  from VectorParams. It is the array depth of this bank, and it is the SAME
  depth in every bank: the four banks together form the
  `numVecPhysRegisters x vLen` array the spec calls for, by each holding a
  `numVecPhysRegisters x (vLen/4)` slice of it. Banking does not divide the
  depth. `vecPregSz = log2Ceil(numVecPhysRegisters)` is 7 at the default.

  //@req-spec-vrf.f5
  //@req-spec-vrf.f10
  `numBanks` is fixed at 4. It is exposed as a parameter only so that the bank
  width and the mask width can be derived from one name instead of a literal 4,
  and it is checked with `require(numBanks == 4)`: the banked architecture, and
  specifically the count 4, is a spec obligation and not a tuning knob. A bank
  therefore never sees `numBanks` in its datapath; only in derived widths.

  //@req-spec-vrf.f11
  `bankWidth = vLen / numBanks` is the bank's data width in bits — 64 at
  `vLen = 256`. The 64-bit figure quoted in the spec is the `vLen = 256`
  instance of this formula, not an independent constant, so nothing in this file
  may write 64. `require(vLen % numBanks == 0)`; `vLen` is already required to
  be a power of two by rocket-chip, so `bankWidth` is too.

  `bankBytes = bankWidth / 8` is the number of write-mask bits, 8 at default.

  `bankId` in `0 until numBanks` identifies the instance. It is permitted in
  exactly two places: elaboration-time `require`s and trace/assertion message
  strings. If `bankId` appears in any datapath expression the file has been
  mis-generated — the four instances are structurally identical, and VecRegFile
  distinguishes them purely by which bit slice it wires to each.

  `numReadPorts` is 9, and `numWritePorts` is `1 + lsuWidth`: 2 at
  `lsuWidth = 1` (W0, W2) and 3 at `lsuWidth = 2` (W0, W1, W2). Defaults 9 and
  3. Legal range for writes is 2 to 3. Both counts come from VecRegFile, which
  derives them from the canonical table; the bank does not recompute them and
  does not know which unit any index belongs to.

  `usingRVV` is the Scala `Boolean` from `BoomCoreParams` (NOT rocket's
  `usingVector`, and NOT a hardware `Bool`). It is not a functional parameter of
  this module — it is the gate on the module's existence, asserted here with a
  `require(usingRVV)` so that an accidental instantiation in a vectors-off build
  fails elaboration instead of silently emitting 6 kbit of flops.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are the Chisel implicits: single `clock`, rising edge, and a
  single active-high SYNCHRONOUS `reset`, shared with the rest of the core. No
  second clock domain, no asynchronous reset, no clock enable port.

  //@req-spec-vrf.f14
  //@req-spec-vrf.f7
  Read ports — the bank carries ALL `numReadPorts` of them, at `bankWidth` bits
  each, because a VRF read is a full `vLen` and every read port therefore spans
  all four banks. `read_addr` is `Vec(numReadPorts, Input(UInt(vecPregSz.W)))`,
  the PRN, and `read_data` is `Vec(numReadPorts, Output(UInt(bankWidth.W)))`,
  this bank's slice of it. VecRegFile drives the same address to all four banks
  and concatenates the four `read_data` slices into the `vLen`-wide result it
  hands its client.

  There is deliberately NO handshake on a read: no `valid`, no `ready`, no
  `Decoupled` (again unlike the scalar `arb_read_reqs` in
  v4/exu/register-read/regfile.scala). Ports are statically partitioned, so
  there is nothing to arbitrate and nothing that can be refused; a read is an
  unconditional combinational lookup of whatever address is presented, and an
  idle port's result is simply ignored by VecRegFile.

  Write ports — `Vec(numWritePorts, Flipped(Valid(...)))` with three fields:
    `addr` : `UInt(vecPregSz.W)` — the destination PRN, unshifted.
    `data` : `UInt(bankWidth.W)` — this bank's slice of the write data.
    `mask` : `UInt(bankBytes.W)` — per-BYTE write enable within the slice.

  The mask is per byte and NOT per 64-bit lane. Per-lane granularity would be
  indistinguishable from no mask at all in a 64-bit bank, and would make a
  sub-lane write impossible: a mask-register load (`vlm.v`, which writes
  `ceil(vl/8)` bytes) or a partial tail must leave every other byte of the
  destination undisturbed, and the LCB writes only the byte lanes it has
  assembled. A byte is the finest granularity RVV ever writes at — at SEW=8
  element `e` is byte `e` — so one mask bit per byte, and no finer.

  VecRegFile slices a whole-VRF write into the four banks: bank `b` receives
  data bits `[bankWidth*(b+1)-1 : bankWidth*b]` and mask bits
  `[bankBytes*(b+1)-1 : bankBytes*b]` of the `vLen`-wide payload and its
  `vLen/8`-bit mask, with `valid` and `addr` broadcast unchanged to all four.
  A write whose mask is zero in this bank still arrives with `valid` set; the
  bank simply enables no byte. That is intentional — it keeps the four banks
  identical and keeps the write decoder out of the mask's timing path.

  Debug — there is no debug read port. VecRegFile builds the `debug_vrf_read`
  output of `vec_pipeline_io` from the ordinary read ports; a private debug port
  here would be a thirteenth port on the array, which the table forbids.
  <|end_ports|>

  <|begin_logic|>

  ---- Storage ----

  //@req-spec-vrf.f4
  //@req-spec-vrf.f12
  The storage is `Reg(Vec(numVecPhysRegisters, UInt(bankWidth.W)))` — an array of
  standard-cell flip-flops, named `vrf_bank`. This is the binding implementation
  choice: no `SyncReadMem`, no `Mem`, no vendor SRAM macro and no latch array is
  permitted here, because an SRAM's registered read would break the single-cycle
  read contract below and a memory macro cannot present 12 independent ports.
  Four such banks are `numVecPhysRegisters x vLen` in total, 96 x 256 b = 24 kbit
  at the defaults, of which this bank holds 6 kbit.

  The array takes NO reset value: it is `Reg`, not `RegInit`. Reset-initializing
  24 kbit would fan the reset net into every bit of the file and buy nothing
  architecturally — RVV requires software to establish vector state before
  reading it, and `mstatus.VS` gates the file's use until it does. The array's
  existence, not its content, is what `usingRVV` controls (the spec's
  "initialized only by the usingRVV switch"): under a vectors-off build the
  registers are never elaborated. For deterministic simulation use the
  simulator's own initialization (`+vcs+initreg+random`, Verilator's zero init)
  rather than paying reset in silicon.

  //@req-spec-vrf.f6
  //@req-spec-vrf.f11
  Bit-slice contract, which must be stated because both sides of the seam index
  it: a vector register splits across the four banks of `bankWidth = vLen/4`
  bits, little-endian, with bank `b` holding register bits
  `[bankWidth*(b+1)-1 : bankWidth*b]`. Byte `j` of bank `b` is byte
  `bankBytes*b + j` of the register, which is element `bankBytes*b + j` at
  SEW=8. Element order and byte order therefore agree, so the LCB's element
  cursor and the store path's element index can be turned into a bank index plus
  a byte-mask position by division and remainder alone, with no lane swizzle.

  ---- Write path (sequential) ----

  //@req-spec-vrf.f7
  Each write port has its OWN decoder in this bank. Port `w`'s decoder is a
  one-hot `UIntToOH(write_ports(w).bits.addr, numVecPhysRegisters)` qualified by
  `write_ports(w).valid`; crossing it with the per-byte mask gives that port's
  per-register, per-byte enable. There is no shared decoder and no shared
  address bus across ports. Twelve decoders per bank, 48 across the file, is the
  cost the port count imposes and it is deliberate — see the perf section.

  //@req-spec-vrf.f13
  //@req-spec-vrf.f7
  On the rising edge of `clock`, for every register `r` and every byte `j`, byte
  `j` of `vrf_bank(r)` is updated from write port `w` if that port's decoder
  selects `r` and its mask bit `j` is set. Because two write ports can never
  target the same PRN, the per-byte enables of different ports are mutually
  exclusive at any given `r`, so this is a plain per-byte-masked OR-reduction
  over the write ports — a byte's next value is the OR of (each port's data byte
  ANDed with that port's enable for this byte) with the old byte where no port
  enables it. It is NOT a priority mux and NOT an arbiter, and no write port can
  be refused or delayed.

  //@req-spec-vrf.f13
  That exclusivity is imposed on this bank from outside, so it is ASSERTED, not
  assumed silently: for every pair of write ports `i < j`,
  `assert(!(write_ports(i).valid && write_ports(j).valid &&
  write_ports(i).bits.addr === write_ports(j).bits.addr))`, with a message
  naming both port indices and `bankId`. This mirrors the "too many writers a
  register" assertion the scalar `RegisterFile` already carries, and it holds by
  construction upstream: every PRN is the destination of exactly one producer,
  rename always allocates a fresh group, and a shared (segmented) instruction
  writes `pvtmp` from one half and `pvdest` from the other — two distinct
  groups. If it fires, the defect is in rename or in the LCB/CII write routing,
  never here, and the fix is not to add priority to this module.

  The assert is the enforcement of the property that licenses the OR-reduce
  write path. Weakening it to a priority mux would hide a rename bug.

  ---- Read path (combinational, single cycle) ----

  //@req-spec-vrf.f7
  //@req-spec-vrf.f14
  Each read port also has its own decoder in this bank: port `p`'s data is
  `vrf_bank(read_addr(p))`, a `numVecPhysRegisters`-to-1 select of `bankWidth`
  bits, independent of every other port's. All nine exist in every bank
  simultaneously, at `bankWidth` bits each. No port is dropped, gated by an
  address bit, or shared with another port in any bank.

  //@req-spec-vrf.f8
  Read-during-write forwarding, per read port, combinationally in the same
  cycle: for read port `p`, if any write port `w` is valid with
  `write_ports(w).bits.addr === read_addr(p)`, then `read_data(p)` is the
  byte-wise merge of that port's `data` (for bytes whose mask bit is set) with
  the array output (for the rest) — the same per-byte merge the write path will
  commit at the end of the cycle. Since two writers cannot name one PRN, at most
  one write port can hit any given read address, so the forward select is again
  an OR of mutually exclusive per-byte selects rather than a priority chain.
  Byte granularity matters here too: a partially masked write must forward only
  the bytes it writes and leave the rest reading out of the array, or a
  `vlm.v`-style sub-lane write would corrupt the untouched bytes of a
  same-cycle reader.

  //@req-spec-vrf.f8
  The forwarding is unconditional, not predicated on any claim about how far
  apart a producer's write and a consumer's read must be. One could argue it is
  unreachable — vector operands wake on group-done, after the last member is
  written — but the LCB's `stale_pvdest` pre-load on R2, the store path's R3/R4
  reads and the CII's R5-R8 pull lanes all choose addresses without observing
  the write ports, and any later shortening of the wakeup path would turn a
  same-cycle case on with no signal that it had. The forwarding mux is part of
  the port's definition; removing it because a regression still passes is a
  failed review.

  ---- What this module does not contain ----

  No state other than `vrf_bank`. No `busy` output, no credit, no reservation
  counter, and nothing scoped to "the current instruction" — this module cannot
  stall or be stalled and exports no signal that could gate an issue unit. No
  pipeline register on either path, so the bank has a fixed,
  parameter-independent timing shape and needs no latency parameter.

  ---- Tracing ----

  Tracing follows the shared VecTrace convention (`vecTrace` plusarg, off by
  default). This bank cannot use `VecTrace.trace`: that helper requires a
  `MicroOp` so every line carries `rob_idx`, and a storage bank has no uop
  context and **must not be given one** just to be traceable. Per-access VRF
  trace lines that need instruction identity are therefore VecRegFile's, which
  has the `rob_idx`.

  Use **`VecTrace.traceStruct("VecRegFileBank", "read_fwd", ...)`** — the
  structure-scoped rung of VecTrace's three-step ladder — with `bank`, `port`
  and `prn` as the `extra` keys. This module emits only the write-collision
  assertion above plus, at most, one line per cycle in which a read forwards,
  because a forwarding bug is otherwise indistinguishable from a stale-operand
  bug several stages downstream. Neither may introduce a register or a wire that
  functional logic reads.

  ===> DO NOT HAND-ROLL A `printf` BEHIND `VecTrace.traceEnabled`. An earlier
  version of THIS paragraph said "guarded printf ... gated on
  `VecTrace.traceEnabled && !reset`", and the first generation duly emitted a
  raw `printf` with its own format string — the only node in the design to do
  so, which is exactly the divergence the shared package exists to prevent.
  `traceStruct` did not exist at the time; it does now, it applies the
  `traceEnabled && !reset` gate itself inside the private `emitLine`, and the
  VecTrace spec now forbids hand-rolled emission outright. `traceEnabled`
  remains public only for gating a caller's own NON-emitting debug logic.
  Corrected 2026-08-10; the generated RTL was fixed in the same pass.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput and latency, as constraints rather than commentary:

- Nine reads and up to three writes EVERY cycle, unconditionally, with no
  arbitration, no back-pressure and no stall path. This is the property that
  makes VecRegFile's static partition true, so it is a constraint on this node,
  not an aspiration.
- Read latency is ZERO cycles: address presented and `bankWidth` bits of data
  returned in the same cycle, including the forwarded case. The read path may
  not be pipelined or registered. A registered read would break the spec's
  single-cycle read, and covering it would require a vector bypass network,
  which the design deliberately does not have.
- Write latency is one cycle: data presented in cycle `n` is readable from the
  array in `n+1`, and is visible to a same-cycle reader in `n` only through the
  forwarding path.

AREA AND TIMING ARE WHY THIS NODE EXISTS SEPARATELY, and the open risk is
confined to it. The cross-cutting risk table in the milestone plan lists VRF
area as a HIGH risk: 96 x 256 b = 24 kbit of flops with 12 ports, where the PORT
COUNT now dominates the storage term — cutting 128 PRNs to 96 saved 8 kbit while
the port count went 11 to 12, so the trade moved the wrong way. Per bank the
cost is 6 kbit of flops plus 9 read muxes of 96:1 x 64 b, 3 write decoders of
96 one-hot outputs, and the byte-merge and forward-compare logic on every port.

The mitigation is an area/timing estimate at gate C3 BEFORE committing to a
flop-based file; the open question is whether flops survive or whether this
becomes latch-based or SRAM-banked. That decision changes THIS MODULE ONLY: the
port partition above it (midcore.rst `vrf-ports`), the bit-slice contract and
the latencies stated here are the interface, and any replacement storage must
meet them — in particular the zero-cycle read, which rules out an ordinary
single-cycle-latency SRAM.

The critical path to budget: `vrf_bank` flop Q -> the port's
`numVecPhysRegisters`:1 read mux -> the forward byte-merge mux -> the module
output -> the consumer's own logic in the same cycle. The forward compare
(`write_addr === read_addr`, `vecPregSz` = 7 bits) runs in parallel with the
read mux and should not be on the critical path; if it becomes so, the fix is a
faster comparator, not a pipeline stage.
<|end_perf|>

<|begin_dependencies|>
VectorParams — `numVecPhysRegisters`, `vLen` and the derived `vecPregSz`. Every
width in this file derives from those; no literal 4, 64, 96 or 256 appears
anywhere in the emitted Chisel.

VecTrace — the `traceEnabled` gate only. This module does not use the
`trace(uop, ...)` helper, because it has no `MicroOp` (see the logic section).

INSTANTIATED BY VecRegFile, four times, as `bank`. VecRegFile owns everything
this module deliberately does not: which functional unit holds which read or
write port, the `vLen`-wide concatenation of the four `read_data` slices, the
slicing of write data and of the `vLen/8`-bit write mask into per-bank fields,
and the `debug_vrf_read` output.

Instantiates nothing itself. In particular it instantiates no memory primitive:
the array is `Reg(Vec(...))` and must stay so unless the C3 area estimate
overturns the flop decision, which is a change to this file and to
midcore.rst `vector-regfile`, not to any neighbour.

Not a dependency of, and not aware of, the VL register file: VlRegFile is a
separate 64 x ~9 b structure that shares only the rename-space definition, never
storage. Do not generalize this module to serve both.
<|end_dependencies|>

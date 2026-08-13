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
  VecRegFile — the vector physical register file as seen by the rest of the
  machine: 96 PRNs x `vLen` bits presented through a STATICALLY PARTITIONED
  9-read / 3-write port set, assembled from four `VecRegFileBank` width slices.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/regfile/VecRegFile.scala,
  package boom.v4.vec.generated.regfile,
  depends_on VectorParams, VecTrace.
  Instantiates VecRegFileBank as `bank`, count 4.
  Instantiated once, as `vrf`, by VecPipeline.

  THIS NODE OWNS WHO OWNS WHICH PORT. The companion node VecRegFileBank owns HOW
  the array is built (flops, banking by width, per-port decoders, the per-byte
  write merge, read-during-write forwarding). This file owns the canonical port
  partition from midcore.rst `vrf-ports`, the fan-out of a `vLen`-wide write into
  four bank slices, the concatenation of four read slices back into a `vLen`-wide
  result, the per-access trace lines and `debug_vrf_read`. It owns NO
  multiplexing at all: after A16 every port here has exactly one driver, and the
  only strict-priority mux in the vector register path lives in `VecGroupCopy`.

  ===> THE PARTITION IS CANONICAL, 0-BASED, AND NOTHING MAY ADD A PORT.
       R0 load index | R1 load mask v0 | R2 load `stale_pvdest` |
       R3 store data | R4 store mask AND store index (ONE port, TWO readers) |
       R5-R8 CII source pull lanes 0-3 | W0 load/LCB | W1 second LCB write
       (exists only at `lsuWidth = 2`) | W2 CII writeback.
       Totals: 9R / 2W at `lsuWidth = 1`, 9R / 3W at `lsuWidth = 2`. A reader or
       writer that cannot name its number does not have a port. Earlier drafts
       said 7R/4W on the argument that a pull interface "needs 2 reads, not 4":
       wrong in both directions, and settled by the frozen SV contract, not by
       argument — `CII_NUM_SRC_REQ = 4` gives R5-R8 and `CII_NUM_DST_WB = 1`
       gives W2 alone. `tt_cii_caracal_pkg.svh`'s own inline comment still reads
       "2 VRF read ports for CII (5,6)"; the VALUE is authoritative, not the
       comment.

  ===> THE VECTOR BYPASS NETWORK IS THE REGISTER FILE'S READ-DURING-WRITE
       FORWARDING, not a separate structure. See the logic section; this is a
       recorded design decision, not an omission.

  ===> THE READ PORT CONTRACT IS A REGISTERED, ONE-CYCLE READ: request in cycle
       N, response in N+1, forwarding applied to the response. The bank's
       combinational array access sits INSIDE that envelope and the output flop is
       here, ONE PER READ PORT. Do NOT read VecRegFileBank's "0-cycle, may not be
       pipelined" as this contract — it describes the ARRAY ACCESS only, it is
       BANK-INTERNAL, and it must not be propagated outward to any consumer. Every
       client already assumes N+1 (spec-cii.f23's ordering FIFO exists to cover
       it; VecMaskStream, VecIdxGen, the LCB's `stale_resp` and VecDgen's
       `vrf_r3` all say so).
       VecPipeline part 8 states this ONCE as the design's single canonical answer
       and it is binding: THIS file holds the flop, and NO consumer may hold a
       second one. Its mandatory consequence is on a sibling —
       VecCiiOperandServer deletes its `vLen`-wide payload register — because if
       both sides register, the observable latency is 2 while `srcReadLatency`
       still says 1 and the positional Src-Data channel is offset by a beat
       forever, silently.

  Governing spec anchors: midcore.rst `vrf-ports` (the canonical partition — the
  single source of truth, which cii.rst and loadstore.rst defer to),
  midcore.rst `vector-regfile` (96 PRNs / 9R / 3W / banked / usingRVV),
  midcore.rst `regfiles-bypass` (the vector bypass network, no temp file, no mask
  file), midcore.rst `old-vd` (why R2 exists), cii.rst `cii-operands` and
  `cii-writeback` (R5-R8 / W2 and the never-stall obligation),
  case_study.rst `case-vl-zero` (the group copy borrowing R2/W0),
  glossary.rst `glossary-terms` (`pvtmp` is an ordinary VRF group).

<|begin_module|>

  <|begin_parameters|>
  Every parameter is a Scala value resolved at elaboration, read from
  VectorParams / `HasVectorParams` / `HasBoomCoreParameters`. No literal 4, 9,
  64, 96 or 256 appears in the emitted Chisel.

  //@req-spec-vrf.f1
  //@req-spec-vrf.f9
  `numVecPhysRegisters` — the number of vector PRNs, default 96 from
  VectorParams, and `vecPregSz = log2Ceil(numVecPhysRegisters)` = 7 bits is the
  width of every address on every port. The file is parameterizable in this
  count, in `vLen` and in the write-port count; it is NOT parameterizable in the
  read-port count or in the port assignment, because the partition is a contract
  between eleven other modules rather than a tuning knob. `usingRVV` (the Scala
  `Boolean` from `BoomCoreParams`, NOT rocket's `usingVector`, NOT a hardware
  `Bool`) is the only thing that initializes the VRF into existence: VecPipeline
  instantiates this module only under that gate, so a vectors-off build contains
  no VecRegFile, no VecRegFileBank and no vector flops at all — absent, not tied
  off. `require(usingRVV)` here so an accidental instantiation fails elaboration.
  The bank honours the same reading by declaring its array inside this gated
  instantiation, which is why `usingRVV` controls the array's EXISTENCE and not
  its reset value.

  //@req-spec-vrf.f2
  `numReadPorts` is 9, fixed, with `require(numReadPorts == 9)`. It is a named
  parameter only so the port array and the bank instantiation read one name; the
  count is a spec obligation (R0-R8) and does not track any tier knob. It is 9 at
  every `lsuWidth`: the second LCB write lane adds a WRITE port, never a read.

  //@req-spec-vrf.f3
  //@req-spec-vrf.g20
  `numWritePorts = 1 + lsuWidth`, i.e. 2 at `lsuWidth = 1` and 3 at
  `lsuWidth = 2`, default 3 on the tiers this design targets. Legal range 2..3,
  checked with `require(lsuWidth >= 1 && lsuWidth <= 2)`. The count is bound to
  `lsuWidth` — BOOM's existing D$-lane parameter — rather than to a list of
  config names, so any tier including those below Medium is covered without this
  file tracking a config list. Default support is "3 write ports" in the sense
  the spec means it: three at the default `lsuWidth = 2` tier.

  `vLen` (256) is the register width and `vLenBytes = vLen / 8` (32) is the width
  of a write mask. `numBanks` is 4 and `bankWidth = vLen / numBanks`,
  `bankBytes = bankWidth / 8`; those three are the bank's parameters, restated
  here only because this node does the slicing and must use the same values.
  `coreWidth` sizes `debug_vrf_read`, and `robAddrSz` sizes the trace side-band.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are the Chisel implicits: one `clock`, rising edge, one
  active-high SYNCHRONOUS `reset`, shared with the core. No second clock domain,
  no asynchronous reset, no clock enable.

  ---- The port-number object ----

  //@req-spec-vrf.g1
  //@req-spec-cii.f29
  Emit a Scala `object VrfPort` of named 0-based constants and index every port
  through it, so a name and a number can never disagree: `R_LD_IDX = 0`,
  `R_LD_MASK = 1`, `R_LD_STALE = 2`, `R_ST_DATA = 3`, `R_ST_MASK_IDX = 4`,
  `R_CII_BASE = 5` (lanes `R_CII_BASE + 0 .. + 3` = R5-R8), `W_LD_LCB0 = 0`,
  `W_LD_LCB1 = 1`, `W_CII = lsuWidth` — the LAST of those is A17 and is never a
  literal 2, for the reason spelled out under the write ports below. Ports are
  STATICALLY PARTITIONED and never arbitrated: no arbiter, no round-robin, no
  request/grant and no `ready` anywhere in this interface, so a client's port is
  available to it every cycle by construction. There is NO exception and no second
  request slot: after A16 even the VL == 0 group copy presents nothing extra here
  (see the R2/W0 subsection below and logic paragraph 6).

  ---- Bundles declared in this file ----

  `VecVrfReadReq`  : `addr` (`UInt(vecPregSz.W)`), plus `rob_idx`
  (`Valid(UInt(robAddrSz.W))`) which is TRACE-ONLY.
  `VecVrfWrite`    : `addr` (`UInt(vecPregSz.W)`), `data` (`UInt(vLen.W)`),
  `mask` (`UInt(vLenBytes.W)`), plus the same trace-only `rob_idx`.

  Declared here rather than in VecBundles, mirroring baseline BOOM, whose
  `RegisterFile` declares its own read/write port IO next to the file; if
  VecBundles also declares them that is a duplicate, not two types. `addr`,
  `data` and `mask` are the exact field names the LCB's `io.vrf_write` uses.

  The trace `rob_idx` carries NO functional meaning and must not reach the
  datapath, the decoders, the forwarding compare or any functional register. It
  exists because per-access VRF trace lines are this node's job (the bank has no
  uop context) and a line without an instruction identifier joins to nothing. It
  is a `Valid` so a client with no uop in hand leaves it invalid and the line
  prints `rob=?`, as `VecTrace.traceDecode` does at decode, rather than inventing
  a zero that aliases real ROB entry 0.

  ---- Read ports ----

  //@req-spec-vrf.f2
  `io.read : Vec(numReadPorts, Flipped(Valid(new VecVrfReadReq)))` and
  `io.read_data : Vec(numReadPorts, Output(UInt(vLen.W)))`. Port `p`'s data is
  the full `vLen`-wide register named by `io.read(p).bits.addr`, REGISTERED:
  presented in cycle N, `io.read_data(p)` is valid in cycle N+1, once, and holds
  only until the next response. Exactly one flop stage, never two — see the
  perf section, where this is a named obligation rather than a target.

  `valid` is a port-cycle indication only — it gates the trace line and the R2
  request slot mux, nothing else. The address goes to all four banks
  unconditionally, an idle port's `read_data` is ignored by its client, and there
  is no `ready`, no `Decoupled` and no way for this module to refuse a read. There
  is no response-side `valid` either: a client that requested in N samples in N+1,
  and each is a single-outstanding reader of its own port (VecCiiOperandServer
  covers the same fact with an ordering FIFO, because it interleaves four lanes'
  responses into one channel). Where a client expects a `ready` on its request —
  VecIdxGen's store instance does, R4 being shared upstream — that `ready` comes
  from the 2:1 mux inside VecLsu, NEVER from here.

  Owners, by number, each with exactly one client on its own side:
  //@req-spec-vrf.g18
  //@req-spec-vrf.g19
  R0 the load index read (VecIdxGen's load instance). R1 the load mask read of
  `v0` (VecMaskStream's `ld_msk`) — R1 has EXACTLY ONE reader, which is why the
  mask streamer was hoisted to VecLsu level and carries the mask forward with the
  `nOP.v` rather than letting the stage-2 Packer read the VRF itself; a US op in
  stage 2 and an SSI op in stage 1 would otherwise be two concurrent readers,
  which a statically partitioned file cannot serve. R2 the LCB's `stale_pvdest`
  member read. R3 the store-data read (VecDgen). R4 also has exactly one reader on
  its own side, but that reader multiplexes TWO jobs upstream: the store mask
  (`st_msk`) and the store index (VecIdxGen's store instance). One `vLen`-wide
  read of `v0` yields the mask bits of every element, so the mask is read once per
  `OP.v` and latched while index members are read as the walk advances — the two
  never need the same cycle, so an indexed masked store needs no second port.
  R5-R8 the four CII source pull lanes.

  ---- Write ports ----

  //@req-spec-vrf.f3
  //@req-spec-vrf.g9
  `io.write : Vec(numWritePorts, Flipped(Valid(new VecVrfWrite)))`. The mask is
  PER BYTE (`vLenBytes` bits), not per 64-bit lane: `vlm.v` writes `ceil(vl/8)`
  bytes, a partial tail writes fewer, and every byte the mask does not select must
  be left undisturbed. Per-lane granularity would be indistinguishable from no
  mask at all inside a 64-bit bank.

  Owners: index `W_LD_LCB0` is the LCB's first write lane (W0), index
  `W_LD_LCB1` its second (W1, elaborated only at `lsuWidth = 2`), index `W_CII`
  is the coprocessor writeback (W2). The Store Unit appears in this list nowhere
  and gets no write port at all: a store READS the VRF (R3, R4) and writes
  memory; it never writes the VRF.

  //@req-spec-vrf.g21
  //@req-spec-vrf.g22
  //@req-spec-vrf.g23
  At `lsuWidth = 1` the write array has two entries, W0 and W2, and W1 is ABSENT
  — not elaborated, not a tied-off third entry — so the file is 9R/2W. Elaborating
  a dead third write port would contradict the 9R/2W total and would pay 4 more
  per-bank decoders for a port nothing drives. At `lsuWidth = 2` the array has
  three entries and the file is 9R/3W.

  ===> A17, AND IT IS A WIRING CONTRACT, NOT A NOTE. W2's PHYSICAL INDEX IS
       `lsuWidth`, NEVER A LITERAL 2. The array is COMPACTED in canonical
       W0,W1,W2 order — that is the bank's contract too, which never renumbers
       per bank — and W1 is ABSENT rather than tied off at `lsuWidth = 1`, so the
       CII's port is physical entry 1 there and physical entry 2 only at
       `lsuWidth = 2`. W2's NAME (`W_CII`) and its owner (VecCiiWriteback) never
       move; only the index does. Wiring VecCiiWriteback to a hard-coded 2 drives
       NOTHING at `lsuWidth = 1` — an out-of-range or dead connection on the one
       write path the CII has, on a channel with no back-pressure to report it.
       VecPipeline expresses the same index as `lsuWidth` at the connection site.

  ---- R2 and W0: exactly one client each (A16) ----

  //@req-spec-lsu.m6
  //@req-spec-lsu.m7
  The VL == 0 / fully-inactive group copy (VecGroupCopy) writes a freshly renamed
  `pvdest` group with no memory traffic at all, and it does so by REUSING the Load
  Unit's ports — `stale_pvdest` on R2, the copy on W0, the same two the LCB uses —
  adding NO new VRF port. That is what makes it compatible with the canonical
  table rather than an amendment to it, and it is affordable because those ports
  are idle whenever no load is draining. Even at `lsuWidth = 2` the copy uses W0
  only; its rate is bounded by the single R2 read anyway. Those two requirements
  are the whole of what THIS file owes the group copy, and both are discharged by
  the port table above: the totals stay 9R/2-3W and no name is added.

  ===> AND THAT IS ALL. THIS MODULE SEES EXACTLY ONE R2 READER AND EXACTLY ONE W0
       WRITER. A16 is RESOLVED against this file: the strict-priority mux that
       resolves the LCB against the group copy lives in `VecGroupCopy`, which
       holds `spec-lsu.m13`/`m14` (the mux itself, and "an active load drain
       always wins"). So the `io.gcopy_r2`, `io.gcopy_r2_grant`, `io.gcopy_w0` and
       `io.gcopy_w0_grant` fields earlier drafts declared here are DELETED, along
       with every grant term behind them. R2 and W0 are now ordinary
       single-client ports, indistinguishable in this interface from R0 or W2.
       The LCB's `R2` request, its `R2` data return and its `W0` write reach this
       module having passed THROUGH `gcopy` COMBINATIONALLY, with no grant, no
       `ready` and no nack in either direction, and the LCB is never told it lost
       — it drives unconditionally and this file cannot refuse it. Nothing about
       the counts, the port numbers or the timing changes with the deletion; a
       reviewer should reject any `gcopy_*` field, grant wire or second request
       slot reappearing in this interface.

  ---- Debug ----

  `io.debug_vrf_read : Vec(coreWidth, Output(UInt(vLen.W)))`, the `debug_vrf_read`
  member of the `vec_pipeline_io` interface. It is built from the ORDINARY read
  ports and has no address of its own: lane `i` is `io.read_data(i)`. A private
  debug address port would be a thirteenth port on the array, which the table
  forbids, and the bank correctly declares none. `io.trace_en` (`Input(Bool())`,
  the `vec_trace_en` member of the same interface) is ANDed into the trace gate.
  <|end_ports|>

  <|begin_logic|>

  ---- 1. Structure: four banks, sliced by width ----

  Instantiate `VecRegFileBank` four times as `bank(0..3)` with the same
  `numVecPhysRegisters`, `numReadPorts` and `numWritePorts`, and `bankId = b`.
  Banking is BY WIDTH, NOT BY REGISTER INDEX, so:

  - `bank(b).read_addr(p) := io.read(p).bits.addr` — the FULL, UNSHIFTED PRN, to
    every bank, for every port. There is no bank-select bit, no
    `addr >> log2Ceil(numBanks)` and no per-bank port subset.
  - `io.read_data(p) := RegNext(Cat(bank(3).read_data(p), bank(2).read_data(p),
    bank(1).read_data(p), bank(0).read_data(p)))` — little-endian, bank 0 in the
    least significant bits, matching the bank's stated bit-slice convention (bank
    `b` holds register bits `[bankWidth*(b+1)-1 : bankWidth*b]`; byte `j` of bank
    `b` is register byte `bankBytes*b + j`, which is element `bankBytes*b + j` at
    SEW=8).
  - For write port `w`: `valid` and `bits.addr` are broadcast unchanged to all
    four banks, while `bits.data` and `bits.mask` are SLICED —
    `data(bankWidth*(b+1)-1, bankWidth*b)` and
    `mask(bankBytes*(b+1)-1, bankBytes*b)`. A write whose mask is all zero in a
    given bank still arrives there with `valid` set; the bank enables no byte.

  ===> DO NOT reuse BOOM's scalar `BankedRF` (v4/exu/register-read/regfile.scala).
  It banks the register COUNT and therefore computes a bank select and shifts
  the address. Here every bank holds a slice of EVERY PRN. Applying the scalar
  pattern would quietly drop three quarters of every read.

  That `RegNext` is THE read-port output flop, one per read port, and the only
  state in this module. It is what makes the PORT a registered one-cycle read while
  the bank's array access stays combinational: it captures the bank's
  already-forwarded value at the end of the request cycle, so the response is the
  value of that PRN AS OF THE REQUEST CYCLE, write-during-read included, and a
  write landing in N+1 is correctly not reflected. `numReadPorts * vLen` = 2304
  flops, the price of the envelope every client is written against.

  Beyond those flops this module holds NO storage: no array of its own, no FSM, no
  counter, no state scoped to an instruction, and no signal that could gate an
  issue unit.

  ---- 2. Why a static partition is sound: no arbitration is required ----

  //@req-spec-vrf.g1
  //@req-spec-cii.f29
  Neither conflict a 12-port file invites can arise, for two independent reasons,
  and that is what licenses the absence of every arbiter above.

  Two write ports never target the same PRN: every PRN is the destination of
  exactly one producer and rename always allocates a FRESH group, so W0/W1 (the
  LCB's lanes, one write per destination PRN) and W2 (the coprocessor) cannot name
  the same register — including for a shared instruction, where one half writes
  `pvtmp` and the other `pvdest`, two distinct groups. So no write priority and no
  write arbitration exists here; the assertion enforcing the property lives in
  VecRegFileBank and is deliberately not duplicated.

  Read ports never conflict over a bank: a read is a full `vLen`, so every port
  spans all four banks, and several read ports addressing one bank is the normal
  case rather than a hazard.

  ---- 3. The vector bypass network IS this file's read-during-write forwarding ----

  //@req-spec-vrf.d1
  BOOM has a full operand-forwarding bypass network for the integer and FP
  pipelines, and Caracal owes the vector pipeline an equivalent. THAT EQUIVALENT
  IS REALIZED AS PER-PORT READ-DURING-WRITE FORWARDING INSIDE THE REGISTER FILE
  (in the banks, on every one of the nine read ports), NOT as a separate network
  of muxes between execution units. This is a design decision and is recorded as
  such: an INT/FP-style network pays off because a scalar ALU has a fixed
  one-cycle producer latency worth catching before writeback, whereas every vector
  producer here — the LCB after a long streaming load, the CII in program order
  over a credit-metered channel — has a long, variable latency and wakes its
  consumers on actual group-done, never speculatively, so a separate network would
  be `vLen`-wide muxing for a case the wakeup discipline has already resolved. The
  file's own forwarding covers the one case that remains observable, a write
  landing on the PRN being read in the request cycle, and because the output flop
  captures the bank's forwarded value the registered read is COHERENT with a
  same-cycle write rather than in tension with it. Without it a registered read
  could return a byte the array was overwriting in the very cycle it was sampled,
  and there is no network above the VRF to repair it.

  ---- 4. pvtmp and masks are ordinary vector registers ----

  //@req-spec-core.h1
  //@req-spec-core.h2
  //@req-spec-vrf.d2
  //@req-spec-vrf.d3
  //@req-spec-vrf.d4
  //@req-spec-vrf.d5
  There is no separate temporary register file anywhere in this design and no
  second array in this module. The intermediate results of a shared (segmented)
  instruction live in the MAIN VRF as the `pvtmp` group: an ordinary VRF group of
  ordinary PRNs, with a real busy lifetime, reclaimed on mispredict and freed at
  commit alongside `stale_pvdest`. Every vector EU addresses `pvtmp` exactly as it
  addresses any other vector PRN — it arrives on `bits.addr` of an ordinary port
  and THIS MODULE CANNOT TELL IT APART from any other PRN, which is the property
  being specified. Its reads, writes, busy/wakeup and branch/commit reclaim all go
  through the existing vector register machinery: these ports, VecBusyTable, the
  vector group-done wakeup and VecRenameSpace.

  //@req-spec-vrf.g13
  //@req-spec-vrf.g14
  Concretely, on the ports already listed and adding none. Segmented LOAD: the
  LSU writes the loaded data into `pvtmp` on W0/W1 (it is the LSU half's
  destination) and the coprocessor reads `pvtmp` on R5-R8 to transpose it (it is
  the coprocessor half's source). Segmented STORE: the roles reverse — the
  coprocessor writes `pvtmp` on W2 and VecDgen reads it as store data on R3, via
  `dgen_operand = Mux(uop.is_shared, uop.pvtmp, uop.pvs3)` upstream in the issue
  slot. Either way `pvtmp` is a destination for one half and a source for the
  other, on ports that already exist.

  //@req-spec-vrf.c9
  //@req-spec-vrf.c10
  There is likewise NO dedicated mask register file. A vector mask is treated like
  any other vector register: `v0` is read as a whole `vLen`-wide PRN on R1 (load
  path) or R4 (store path), and is delivered to the coprocessor as the `VM`
  source on one of R5-R8. Masking semantics — which elements are active, `vma`
  merging — are applied in the execution units and in the LCB, never here. This
  module has no notion of a mask register, only of a per-byte WRITE mask, which
  is a different thing entirely.

  ---- 5. The coprocessor is a client of this file, not its owner ----

  //@req-spec-core.d4
  The VRF is instantiated by VecPipeline as `vrf` and lives in the host. The
  coprocessor reads and writes it through the CII as a CLIENT — it presents PRNs
  on R5-R8 in answer to its own Src-Requests and results on W2 — and holds no VRF
  storage, no shadow copy and no private address space. VecCiiOperandServer
  resolves an `op_id`/`op_offset` pull to a physical member PRN from its
  side-table and presents that PRN here; VecCiiWriteback presents W2. A
  coprocessor-owned file would also make `pvtmp`'s rendezvous between the LSU half
  and the coprocessor half impossible.

  //@req-spec-vrf.g10
  //@req-spec-vrf.h3
  //@req-spec-cii.f25
  The four CII source read ports are R5-R8, one per Src-Request pull lane, owned
  outright by VecCiiOperandServer. Four, not two, and DERIVED rather than chosen:
  `CII_NUM_SRC_REQ` = `CII_NUM_SRC_DAT_RSP` = 4 in `tt_cii_caracal_pkg.svh`, and
  the VPU wrapper returns "member `k` of {VS1, VS2, VS3, VM} in one dat beat", so
  all four lanes can be live in one cycle and each needs its own port.
  `stale_pvdest` (the `STALE_VD` pull) and `pvs3` are independent source slots
  naming independent groups, served on two of these same four lanes — never
  merged, never reinterpreted as one another.

  //@req-spec-vrf.g11
  //@req-spec-vrf.h4
  //@req-spec-cii.f26
  The single CII write port is W2, index `W_CII` = `lsuWidth` and never a literal
  2 (A17), owned outright by VecCiiWriteback. One, not two, and again derived:
  `CII_NUM_DST_WB = 1`, a constant flagged
  must-match-the-`tt_cii_interface`-default because the relay's `type(wb_data)`
  resolves against it — an override that differs is a port-connection mismatch,
  not a performance choice.

  //@req-spec-vrf.g15
  //@req-spec-vrf.g16
  //@req-spec-cii.f30
  BECAUSE THE PARTITION IS STATIC, THE CII CAN NEVER STALL ON A VRF PORT. It owns
  R5-R8 and W2 outright, this module presents no `ready` and no grant on any of
  them, and nothing else in the machine can drive them, so a CII access is
  accepted in the cycle it is presented, always. That is load-bearing rather than
  convenient: the CII Writeback channel is credit-metered with NO back-pressure
  line, so a refused write would have nowhere to go and the beat would be lost. It
  is also why W2 must not be "shared when idle" with anything — the moment a
  second client can take it the never-stall property is gone.

  //@req-spec-vrf.g17
  //@req-spec-cii.g2
  One write port suffices precisely because `CII_NUM_DST_WB = 1` means the
  coprocessor cannot present two result beats in one cycle, so a second would
  never be driven and would be pure area. It follows that VecCiiWriteback must
  never merge two beats into one cycle even if a future VPU could produce them.

  ---- 6. NO MUX ON R2 / W0: the group copy is an upstream client (A16) ----

  //@req-spec-lsu.m6
  //@req-spec-lsu.m7
  THERE IS NO MULTIPLEXING ANYWHERE IN THIS FILE. R2 and W0 each have exactly ONE
  request slot, driven by exactly one wire, treated exactly like R0 or W2:
  `io.read(R_LD_STALE).bits.addr` goes to the banks' `read_addr(R_LD_STALE)`
  unconditionally, and `io.write(W_LD_LCB0)` reaches the banks unqualified. No
  grant term, no `!valid` qualification, no priority encoder, no anti-starvation
  counter, and no signal on this boundary telling any client it lost.

  The sharing the group copy needs is real but it happens ONE LEVEL UP. A16 is
  resolved in `VecGroupCopy`'s favour on requirement allocation: `spec-lsu.m13`
  (the strict-priority mux) and `spec-lsu.m14` (an active load drain always wins)
  are allocated THERE, and this file holds only `m6`/`m7` — reuse the Load Unit's
  ports, add no new VRF port — both of which the port table alone discharges. So
  the LCB's request, the R2 data going back to it and its W0 write pass THROUGH
  `gcopy` combinationally; `gcopy` inserts its own request only in a cycle the LCB
  is not using the port, and this module never learns that two clients existed.
  What arrives here is one address and one write, already resolved.

  ===> DO NOT RE-ADD THE MUX HERE, IN EITHER DIRECTION. Two copies of a
  strict-priority mux in series is not redundancy: the outer one would qualify
  an already-resolved request against a `valid` that the inner one has already
  consumed, so a granted copy could be dropped with nothing reporting it. The
  functional description was IDENTICAL in both files, which is exactly why
  exactly one may be built.
  Note also: the ORDINARY masked / partial-tail case is NOT the group-copy
  mechanism. Those inactive lanes are pre-loaded from `stale_pvdest` on R2
  INSIDE the LCB, overlapped with memory latency and merged into its single W0
  write. Routing the ordinary case through the copy resurrects the serial
  prologue v2 deletes.

  ---- 7. Tracing and debug ----

  Per-access trace lines are THIS node's job — the bank has no uop context and
  must not be given one — and follow the shared VecTrace convention: a guarded
  `printf` gated on `VecTrace.traceEnabled && io.trace_en && !reset`, off by
  default, in VecTrace's fixed greppable
  `[vec] <module> <event> rob=<...> <key>=<value>` format. Having no `MicroOp`,
  this module cannot call `VecTrace.trace(uop, ...)`; it emits the same format
  from the port's trace-only `rob_idx`, printing `rob=?` when that is invalid,
  exactly as `traceDecode` does at decode. If VecTrace later grows a
  `Valid(rob_idx)` entry point, use it instead of the raw printf.

  One line per valid read port in its REQUEST cycle (`port`, `prn`, `rob`) and one
  in its response cycle with the low bits of the returned data, correlated by port
  number; any register carrying the request fields into the response cycle is
  trace-only. One line per valid write port (`port`, `prn`, `mask`, low data
  bits) — a write has no response and needs no second line, and the `mask` value
  is what distinguishes a sub-lane `vlm.v`-style write from a corrupted
  full-width one. There is NO lost-request line to emit any more: with the
  `gcopy_*` slots deleted (A16) nothing presented to this module can lose, and the
  trace that catches a group copy making no progress belongs to `VecGroupCopy`,
  which is where the losing request now exists.

  `io.debug_vrf_read` is a plain wire off the registered response, lane `i` from
  `io.read_data(i)`, independent of the trace gate — an observation of ports that
  already exist, feeding nothing inside the design.

  Removing every trace call site and the debug output must leave behaviour
  bit-identical: no functional register, no counter, no wire functional logic reads.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Constraints on the implementation, not commentary:

- NINE reads and up to THREE writes EVERY cycle, unconditionally, with no
  arbitration, no back-pressure and no stall path anywhere in this module. This
  is what makes the static partition true and what makes the CII's never-stall
  obligation structural rather than probabilistic.
- THE NAMED READ-LATENCY OBLIGATION: read latency is EXACTLY ONE CYCLE, no more
  and no less — request in N, `io.read_data(p)` in N+1, through exactly one flop
  stage (the per-port output register of logic paragraph 1). The generator may
  neither add a stage for timing nor remove the one that is there. Two stages
  break every client (VecMaskStream's one-entry lookahead, VecIdxGen's
  single-outstanding member read, VecCiiOperandServer's ordering FIFO, the LCB's
  overlap of the R2 pre-load with memory latency). Zero would push a `vLen`-wide
  96:1 mux plus the forward merge into the consumer's cycle. If the path misses
  1 GHz the fix is a faster read mux or a shallower consumer, NOT a second flop.
- Inside that cycle: bank flop Q, the port's `numVecPhysRegisters`:1 read mux, the
  byte-wise forward merge, and the `Cat` of four slices (pure wiring). Nothing
  else: with A16 applied there is no address-side select in front of the banks on
  any port, so no port's path is longer than any other's.
- Write latency is one cycle, and a write in N is seen by a read REQUESTED in N
  through the banks' forwarding, which the output flop then captures — so a
  producer's write and a consumer's read of one PRN in one cycle are coherent at
  no extra latency to either.
- The strict-priority resolution of the LCB against the group copy costs this
  module NOTHING, because it is `VecGroupCopy`'s (A16); what it costs there is one
  AND term on the loser's `valid`, ahead of this file's address input, and no cycle
  on the winner. There must still be no path from R2's read data to W0's write
  data inside the group copy — two cycles on purpose, and with a registered read
  it could not be collapsed even deliberately.

RESOLVED SEAM TIMING CONFLICT — SETTLED, NOT OPEN, and recorded because siblings
stated it in incompatible words. VecRegFileBank's "0 cycles, may not be pipelined"
describes the BANK-INTERNAL ARRAY ACCESS only, and that statement stops at the
bank boundary: it may not be propagated to any consumer of this module. The flop
that makes the PORT a registered one-cycle read lives HERE, one per read port, and
is counted here. The corpus settles it: spec-cii.f23 requires an ordering FIFO to
cover "the registered, one-cycle VRF read", and midcore.rst's "forwarding for
single cycle reads if a write port writes to the same PRN" is a read that
completes in one cycle AND stays coherent with a same-cycle write. The LCB
(`stale_resp`), VecDgen (`vrf_r3`), VecMaskStream and VecIdxGen all already say
N+1 and need no change; VecGroupCopy registers the R2 result itself, so it is
correct here too. The ONE file that was not — VecCiiOperandServer, which took the
read as combinational and registered the payload itself — is being corrected the
only way that keeps latency at 1: it DELETES its `vLen`-wide payload register and
keeps a one-deep per-lane CONTROL register. Both sides registering is the
forbidden combination, and it fails silently: `srcReadLatency` would still say 1
while every positional Src-Data beat answers its request one beat late, forever.

AREA is the design's #2 risk and it is confined to the BANK, not to this node:
96 x 256 b = 24 kbit of flops carrying 12 ports, where the port count now
dominates the storage term (128 -> 96 PRNs cut 8 kbit while the port count went
11 -> 12, so the trade moved the wrong way). The mitigation is an area/timing
estimate at gate C3 before committing to a flop-based file, and if that forces a
latch- or SRAM-banked array THIS FILE'S INTERFACE DOES NOT CHANGE. One
consequence for that decision, since the port is registered: an SRAM with a
one-cycle registered read would REPLACE this node's output flop, not add to it,
and the read-during-write forwarding merge would have to move up here — the port
must still answer in exactly N+1 and must still be coherent with a write in N.

Capacity, stated beside the port count because both follow from the same 96: 32
PRNs are always held by the committed map table, leaving (96 - 32) / 8 = 8 LMUL=8
groups in flight and at most 4 segmented ops (`pvdest` + `pvtmp` = 16 PRNs).
Nothing enforces that bound — an `OP.v` finds too few free PRNs and stalls at
rename — so watch the free-list stall rate in the LS regression.
<|end_perf|>

<|begin_dependencies|>
VectorParams — `numVecPhysRegisters`, `vLen`, and the derived `vecPregSz`;
`lsuWidth`, `coreWidth` and `robAddrSz` come from `HasBoomCoreParameters`.

VecTrace — the `traceEnabled` gate and the line format. The `trace(uop, ...)`
helper is NOT usable here (no `MicroOp`), as in the bank.

INSTANTIATES VecRegFileBank four times as `bank`, per the bank's own IO contract:
`read_addr : Vec(numReadPorts, Input(UInt(vecPregSz.W)))`, `read_data :
Vec(numReadPorts, Output(UInt(bankWidth.W)))`, `write_ports : Vec(numWritePorts,
Flipped(Valid(addr, data: bankWidth, mask: bankBytes)))`, no handshake anywhere,
`numWritePorts = 1 + lsuWidth` compacted in canonical W0,W1,W2 order and never
renumbered per bank, combinational array read, 1-cycle write, unconditional
read-during-write forwarding, array is `Reg` with no reset value. The bank's
0-cycle figure is INTERNAL: this module's read PORT is registered and one cycle,
and the flop that makes it so is instantiated here, not there.

CLIENTS, each by port number, none of which is a `depends_on:` of this node —
they wire to this module inside VecPipeline / VecLsu / VecCiiHost. EVERY ENTRY IS
ONE DRIVER, because every upstream sharing is resolved before it reaches here:
  R0 VecIdxGen (load instance) · R1 VecMaskStream `ld_msk` ·
  R2 VecGroupCopy's `vrf_r2_req`, which is the LCB's `stale_req` passed through
  combinationally with `gcopy`'s own request behind it (A16) · R3 VecDgen
  (`vrf_r3`) · R4 the 2:1 mux inside VecLsu over VecMaskStream `st_msk` and
  VecIdxGen (store instance), which is also where those two clients' `ready` comes
  from · R5-R8 VecCiiOperandServer · W0 VecGroupCopy's `vrf_w0` (the LCB's
  `vrf_write` lane 0 passed through, same rule as R2) · W1
  VecLoadCoalescingBuffer's second write lane, at `lsuWidth = 2` only · W2
  VecCiiWriteback, at physical index `lsuWidth` (A17).
`stale_resp` reaches the LCB back through `gcopy` as well; this module drives one
`read_data(R_LD_STALE)` and has no return mux and no idea who consumes it.

Field-name note for the wiring: VecIdxGen and VecMaskStream call the address
`prn` and this file calls it `addr` (matching the bank and the LCB). Same thing;
rename at the wiring, do not add a second bundle.

NOT a dependency of, and not aware of, VlRegFile: that is a separate
`numVlPhysRegisters` x `vecVLSz` structure sharing only the rename-space
definition, never storage or ports. Do not generalize this module to serve both.
<|end_dependencies|>

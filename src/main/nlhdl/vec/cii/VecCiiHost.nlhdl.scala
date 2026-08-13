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
  VecCiiHost — the BOOM side of the coprocessor interface: the container that
  joins six Chisel per-direction nodes to the SystemVerilog TT-CII stack through
  one flat BlackBox, owns the four channels' credit accounting, and is the whole
  of the machine's ARN->PRN translation for vector arithmetic.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/cii/VecCiiHost.scala,
  package boom.v4.vec.generated.cii, group vec_cii.
  depends_on MicroOp, VecBundles, VectorParams, VecTrace.
  Instantiated ONCE, as `cii`, by VecPipeline. Instantiates seven things:
  `tags` (VecCiiTagTable), `iss` (VecCiiIssue), `opnd` (VecCiiOperandServer),
  `wb` (VecCiiWriteback), `done` (VecCiiComplete), `flush` (VecCiiFlush) and
  `coproc` (tt_cii_host_wrap, the SV shim, whose reset is `core_rst_n`).

  It is a CONTAINER. It holds almost no logic of its own: the flat-bundle pack
  and unpack, the two sender-side credit counters' home, the receive-side credit
  returns, and the handful of seam decisions the six children could not each make
  alone. Where a child owns a behaviour this file references it by node name
  rather than restating it — a second description is a second thing to keep in
  step.

  ===> CALLOUT 1 — THE SV IS BOUND WHERE IT LIVES. The BlackBox binds through
       `HasBlackBoxPath` with `addPath` against `src/main/sv/v4/**` DIRECTLY.
       NOTHING is copied into `src/main/resources/vsrc/`. `addvector` copied seven
       SV files there, one of them submodule content, which silently falsifies the
       premise v2 rests on: the SV is refactored and separately verified, so a
       stale copy means you verify one file and simulate another with nothing to
       tell you. Single source of truth, no copies, and a build check that the
       tt-cii submodule is present.

  ===> CALLOUT 2 — THE RESET PORT IS `core_reset`, ACTIVE-HIGH, DRIVEN
       UN-INVERTED. The design's only reset-polarity crossing is inside
       `tt_cii_host_wrap`, which inverts once into `core_rst_n`. The `addvector`
       reference named this port `rst_n` and inverted on the Chisel side; writing
       that here gives either two inversions (the SV stack held in reset forever)
       or the crossing in the wrong file. `coproc.io.core_reset := reset.asBool`.

  ===> CALLOUT 3 — CREDIT OWNERSHIP IS ASYMMETRIC AND THE CHANNELS HAVE NO
       `ready`. The host is the RECEIVER on Src-Request and Writeback, so it owns
       those two channels' buffering and returns one credit per pop; it is the
       SENDER on Issue and Src-Data, so it holds a free-running credit counter on
       each and stalls at zero. Getting the direction backwards on any one channel
       produces a hang or a dropped beat with no error signal anywhere.

  Governing spec anchors: cii.rst `cii-interface`, `cii-host-bridge`,
  `cii-operands`, `cii-writeback`, `cii-kill-contract`, `cii-mem-order`;
  execution.rst `vector-execution` (channel overview, what each side provides),
  `cii-prn-arn`, `cii-issue-packet`; overview.rst `caracal-pipeline`;
  midcore.rst `vrf-ports`. Plan v2 section 5 rules 1, 4, 6 and 11.

<|begin_module|>

  <|begin_parameters|>
  No case-class parameters of its own, and no tuning knob. Every figure is
  DERIVED. The Chisel side of the CII has exactly one place where the frozen SV
  numbers appear — a single Scala object in this file, `TtCiiCaracalPkg` — and
  every child reads them through it or through VectorParams. A second literal
  anywhere on the Chisel side is a silent protocol break rather than a compile
  error, which is why the direction of the dependency is one-way and stated here.

  From `tt_cii_caracal_pkg.svh` (authoritative): `CII_VLEN` = 256, `CII_TAG_W`
  = 4, `CII_N_TAGS` = 16, `CII_MEMBER_W` = 3, `CII_MAX_MEMBERS` = 8, `CII_VL_W`
  = 9, `CII_NUM_INST_ISSUE` = 1, `CII_NUM_SRC_REQ` = 4, `CII_NUM_SRC_DAT_RSP`
  = 4, `CII_NUM_DST_WB` = 1, `CII_NUM_SRC_SLOTS` = 4, and the four depths
  `CII_N_{ISS,REQ,DAT,WB}_CREDITS` = 16.

  These are cross-checked, not assumed. The build step of the logic section's
  part 2 — the one that verifies the submodule is checked out — ALSO extracts
  those localparams from the `.svh` and fails the build if any disagrees with the
  Scala mirror. Chisel cannot parse SystemVerilog at elaboration, so a `require`
  alone can only compare Chisel to Chisel; the comparison that matters is against
  the file, and it belongs in the build. A hand-edited value in the mirror object
  with no matching `.svh` change is the one thing a reviewer must reject outright.

  Elaboration `require`s that CAN be expressed in Chisel, all of them here rather
  than scattered through the children: `ciiTagBits == log2Ceil(numCiiTags)`;
  `numCiiTags == ciiIssueCredits` (tag identity and the Issue credit space are
  the same 16, and a divergence desynchronises them); `numSrcReqLanes ==
  numSrcDatLanes` (a host with fewer data lanes than request lanes cannot answer
  a full cycle of pulls and has no way to say so); `numWbLanes == 1`; `maxMembers
  == 8` and `log2Ceil(maxMembers) == ciiMemberBits`; `vecVLSz == 9` (not 6 — 6
  bits is VLMAX for LMUL=1 only and truncates a real VL of 256 to zero); and
  `vLen == 256` matching `CII_VLEN`, since both the Src-Data and Writeback
  payload widths derive from it.

  `ciiRxDepth` — the depth of the host's Src-Request and Writeback receive
  buffers. Default 0, legal range 0..16. See part 4: at 0 the buffer is a wire
  and the credit is returned in the arrival cycle, which is legal precisely
  because both consumers are structurally unstallable. The parameter exists so
  that making either consumer stallable cannot be done without noticing that the
  credit must then move to the pop of a real buffer.

  //@req-spec-cii.c1
  There is no parameter selecting a Chisel or a behavioural model of the
  coprocessor, because there is no second implementation to select. Caracal's core
  is Chisel; the CII interface, its credit relay (`tt_cii`, four
  `tt_cii_channel`s) and the VPU are SystemVerilog, and they stay SystemVerilog —
  frozen, separately verified by `src/main/sv/v4/vpu/tb/cii_fv_tb.sv`, and reached
  only through the flat BlackBox of part 3. This node is the entire Chisel-side
  surface of that split.

  //@req-spec-cii.a1
  ELABORATION GATE. The whole node, all six children and the BlackBox exist only
  when `usingRVV && enableVectorArith`, both Scala `Boolean`s of `BoomCoreParams`
  — never hardware `Bool`s, and never rocket's `usingVector`, which is a different
  gate. Vector arithmetic executing on an in-order vector unit attached through
  the TT-CII is therefore a structural property of the elaborated machine: with
  either flag false there is no attach, no coprocessor and no vector ALU of any
  kind, and the emitted RTL is bit-identical to pre-Caracal BOOM v4. Absent, not
  tied off.
  <|end_parameters|>

  <|begin_ports|>
  CLOCK AND RESET. One `core_clk` domain, POSEDGE clock, ACTIVE-HIGH SYNCHRONOUS
  `core_reset`, both implicit via `BoomModule` — the hierarchy.yaml defaults. The
  active-low `core_rst_n` the SV stack needs is produced INSIDE `coproc` from the
  `core_reset` port this module drives with `reset.asBool`, un-inverted. There is
  no second reset, no CDC and no reset sequencing anywhere in this subtree.

  ---- Issue, to and from `iq_v_alu` (VecIssueUnit) ----

  `io.iss` — `Input(Valid(new MicroOp))`, the `IQ_V_ALU` grant, fanned straight to
  `iss`. Fire-and-forget: no `ready`, and this module never refuses it.
  `io.fu_types` — `Output(UInt(FC_SZ.W))`, to `iq_v_alu.io.fu_types(0)`, straight
  from `iss`. THIS IS THE ONLY BACKWARD SIGNAL THIS NODE PRESENTS TO ANY ISSUE
  UNIT. There is no `busy`, no `ready` and no stall output of any kind — a
  `busy`-style signal reaching an issue unit is a failed review regardless of
  measured performance (plan section 5 rule 6).

  ---- Operand context read at issue ----

  `io.vl_read_addr` / `io.vl_read_data` — `Output(UInt(vlPregSz.W))` /
  `Input(UInt(vecVLSz.W))`, this node's dedicated VlRegFile read port (VlRegFile
  grants one per vector issue queue; this is the `IQ_V_ALU` one). Unconditional
  and combinational, no valid and no ready.
  `io.int_scalar_read_req` / `io.int_scalar_read_rsp`, `io.fp_scalar_read_req` /
  `io.fp_scalar_read_rsp` — the renamed physical scalar source address out,
  `xLen` of data back the following cycle, for the `.vx`/`.vf` capture.
  `io.int_wb_snoop` — `Input(Vec(numIrfWritePorts, Valid({addr, data})))`, the
  existing INT writeback tap, needed for `iss`'s response-cycle forward.

  ===> SEAM GAP, NOW CLOSED BY AMENDMENT. `vec_pipeline_io` used to size
  `int_rf_read_req/rsp` at 4 (two per VecScalarOperandRead instance) and
  `fp_rf_read_req/rsp` at 1, and its comment said this node needs none because
  it captures scalars "from the bypass". A bypass carries only values in
  flight, and a past-PNR CII op's scalar producer has usually retired, so the
  value exists only in the register file — execution.rst `cii-prn-arn` says so
  directly. The seam is now 5 INT read ports, the fifth being this node's, and
  1 FP read port, which is ALSO this node's and the only one on the seam:
  decision D4 deleted the store-side FP reader, so 2 FP ports are not needed.
  Do not "fix" this by deleting the read: that delivers an undefined `.vx`
  operand, silently.

  ---- Architectural CSR state (rocket's, read at issue) ----

  `io.csr_vstart` (rocket's `vstart` width, 8 bits at VLEN=256),
  `io.csr_vxrm` (2 bits) from `csr.io.vector`; `io.csr_frm` (3 bits) from
  `vec_pipeline_io.csr_frm`. All three are wired to `iss` unchanged. `vstart` is
  zero-extended to the packet's 9 bits by `iss`, which owns the reason (an element
  INDEX needs one bit fewer than a COUNT).

  ---- Vector register file (the canonical static partition) ----

  `io.vrf_read_addr` / `io.vrf_read_data` — `Vec(4, Output(UInt(vecPregSz.W)))` /
  `Vec(4, Input(UInt(vLen.W)))`, VRF READ PORTS `R5`, `R6`, `R7`, `R8`, one per
  Src-Request lane, owned outright by `opnd`. Cite the number at the connection
  site.
  `io.vrf_write` — `Valid({addr, data, mask})`, VRF WRITE PORT `W2`, owned
  outright by `wb`. NOTHING HERE ADDS A VRF PORT and no port is arbitrated: the
  partition of midcore.rst `vrf-ports` is static, which is what lets the
  credit-metered channels never stall on a register file.

  ---- Completion and scalar writeback, out through VecPipeline ----

  `io.group_done` — `Valid(new VecGroupDone)` from `done`.
  `io.clr_rob` — `Valid(UInt(robAddrSz.W))`, this producer's OWN `vec_clr_bsy`
  lane. `io.rob_flags` — `Valid({rob_idx, fflags, vxsat})`.
  `io.int_wb` / `io.fp_wb` — `Valid(new ExeUnitResp(xLen))` from `wb`, landing on
  the dedicated scalar-dest write port and wakeup slot `enableVectorArith` adds,
  never on an arbitrated share of `ll_arb`.

  `vec_pipeline_io` declares ONE `vec_clr_bsy`, and there are two independent
  producers of it (the LSU-side group-done and this node's `done`), NEITHER of
  which can be back-pressured — `done` frees its tag in the same cycle, so a
  clear it could not present would be lost with no way to regenerate it.
  VecPipeline must present one lane PER PRODUCER; this node drives its lane
  unconditionally and is never told it lost.

  ---- Flush ----

  `io.rob_flush`, `io.rob_flush_kill` — `Input(Bool())` each, from
  `vec_pipeline_io`: `rob.io.flush.valid` and its `RegNext`. Both go to `flush`
  and `io.rob_flush` additionally to `iss`.
  `io.brupdate_mispredict` — `Input(Bool())`, ASSERTION-ONLY, to `flush`. Taken as
  one bit rather than the `BrUpdateInfo` bundle so it cannot quietly grow into a
  kill term. No field of `rob.io.flush.bits` is a port of this node.

  ---- The flat BlackBox bundle (internal, and a hard contract) ----

  //@req-spec-cii.c3
  This node drives a FLAT four-channel bundle to a `BlackBox` — plain `UInt`
  fields, no `Decoupled`, no bundle-typed port, no `ready` on any channel —
  because a Chisel BlackBox can bind flat `logic` ports only: not an SV
  `interface`/`modport` port, and not a port whose type comes from a `parameter
  type` packed struct, which is exactly what `tt_cii_interface` and the VPU
  wrapper use. The port list below IS the contract with
  `src/main/sv/v4/generated/tt_cii_host_wrap.sv`; a BlackBox port list is
  type-checked against nothing, so a mismatch is a link failure at best and a
  silently mis-wired channel at worst. Names, directions and widths, with
  direction stated from the BlackBox's own point of view exactly as the SV
  declares them:

    clk, core_reset                                    Input  1, 1
    iss_valid, iss_tag, iss_insn, iss_vtype            Input  1, 4, 32, 8
    iss_vl, iss_vstart, iss_vxrm, iss_frm, iss_hint    Input  9, 9, 2, 3, 4
    iss_credit                                         Output 1
    req_valid, req_tag, req_op_id, req_op_offset        Output 1, 16, 12, 12
    req_credit                                         Input  1
    dat_valid, dat_data                                Input  1, 1024
    dat_credit                                         Output 1
    wb_valid, wb_tag, wb_data, wb_dst_offset            Output 1, 4, 256, 3
    wb_wr_en, wb_status                                Output 1, 9
    wb_credit                                          Input  1

  `iss_hint` is `CII_NUM_SRC_SLOTS` = 4 bits, NOT 3. VecBundles gives
  `CiiIssueReq.src_reuse_hint` 3 bits; the SV type `cii_caracal_frwd_hint_t` is 4
  and the SV is authoritative. This node drives zero either way, but the flat port
  must match exactly or the whole Issue payload shifts by a bit.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. What this node is, and what vector arithmetic therefore is not ----

  //@req-spec-core.e18
  //@req-spec-agen.a7
  VECTOR ARITHMETIC IS NOT EXECUTED ON A BOOM EXECUTION UNIT, and this file is
  where that is true rather than merely intended: there is no vector functional
  unit anywhere in this subtree, no vector ALU, no vector datapath and no
  execution stage. An `OP.v` granted by the CII issue queue `iq_v_alu` arrives on
  `io.iss`, is turned into an Issue packet by `iss` in one cycle, and is FORWARDED
  DIRECTLY to the coprocessor over the CII — it visits no `ExeUnit`, is never
  written into a BOOM issue-to-execute register-read stage, and produces no
  `ExeUnitResp` except in the one case where its DESTINATION is a scalar register.
  The only arithmetic the integer ALU keeps under `usingRVV` is the `vset` family,
  which is decoded as a scalar uop and never reaches `IQ_V_ALU`.

  //@req-spec-cii.a2
  ONLY THE ARITHMETIC `OP.v` STREAM IS OFFLOADED. Vector loads and stores stay in
  the unified LSU (`VecLsu`, a sibling under VecPipeline) and never appear on any
  channel of this node. Correspondingly the CII moves NO MEMORY TRAFFIC OF ITS
  OWN: this node has no D$ port, no TLB port, no LDQ/STQ index and no address of
  any kind on any port. Arithmetic offloaded here is ordered relative to memory
  purely by the ROB and by the register dependences rename resolved, so no
  cross-unit memory-ordering machinery exists here — the two exceptions that look
  like memory are the segmented halves, and even they exchange data through the
  VRF `pvtmp` group, not through memory.

  ---- 2. Binding the SystemVerilog: single source of truth ----

  The `TTCii` BlackBox extends `HasBlackBoxPath` and calls `addPath` on the SV
  files it needs, by their real paths under `src/main/sv/v4/`:
  `generated/tt_cii_host_wrap.sv` (this flow's output), `tt-cii/src/tt_cii.sv`,
  `tt_cii_channel.sv`, `tt_cii_fifo.sv`, `tt_cii_interface.sv`,
  `rv_async_rst_dff.sv`, `rv_async_rst_dff_Tdat.sv`, and the VPU subtree rooted
  at `vpu/tt_vpu_cii_wrapper_top.sv`. `tt_cii_caracal_pkg.svh` is INCLUDED rather
  than compiled, so the build must put `src/main/sv/v4/tt-cii/src` on the include
  path; it is reached through the gen-collateral incdir, as the rest of the stack
  already is.

  ===> NOTHING IS COPIED INTO src/main/resources/vsrc/, EVER. `addvector`
  copied seven files there, one of them submodule content, and the copies then
  drifted. That does not merely duplicate a file: it falsifies v2's central
  premise, because the artifact that was verified and the artifact that is
  simulated become different files with nothing to report the difference. A
  build check must confirm the tt-cii submodule is checked out and fail with a
  named message if it is not. If `addPath` fights the Chipyard flow, the ONLY
  permitted fallback is a sync step guarded by a checksum that FAILS THE BUILD
  ON DRIFT — never a silent copy, and never a copy that is refreshed
  best-effort.

  The paths are added in the BlackBox's own constructor body, which is what makes
  the vectors-off promise hold as a build property too: with `usingRVV` or
  `enableVectorArith` false the BlackBox is never constructed, so no `addPath`
  runs, no SV file enters the filelist, nothing references the SV package, and the
  build does not even require the submodule to be present. An `addPath` hoisted
  into a Scala `object` initializer would break that silently.

  ---- 3. The four channels, and who owns which end ----

  //@req-spec-cii.a4
  //@req-spec-cii.a5
  The coprocessor is reached over FOUR UNIDIRECTIONAL, CREDIT-METERED CHANNELS —
  Issue (host to cop, 1 lane), Src-Request (cop to host, 4 lanes), Src-Data (host
  to cop, 4 lanes) and Writeback (cop to host, 1 lane). Each is one
  `tt_cii_channel` credit FIFO inside the `relay` instance of `tt_cii` within
  `coproc`, at the default depth 16 taken from `CII_N_ISS_CREDITS`,
  `CII_N_REQ_CREDITS`, `CII_N_DAT_CREDITS` and `CII_N_WB_CREDITS`. Those four
  instances are the channels; this node presents one end of each and adds none of
  its own. Every channel carries exactly one `valid` and one payload beat forward
  and exactly one `credit` bit back, and NO CHANNEL HAS A `ready` LINE — which is
  why a mistake in this section is a dropped beat or a permanent hang rather than
  a stall.

  //@req-spec-cii.b1
  //@req-spec-cii.b2
  ON THE TWO CHANNELS WHERE THE HOST IS THE SENDER IT HOLDS A FREE-RUNNING CREDIT
  COUNTER AND STALLS AT ZERO. Issue's counter lives in `iss` (`iss_credits`,
  `RegInit(ciiIssueCredits.U)`, `+1` per returned `iss_credit`, `-1` per accepted
  grant) and its stall is expressed by withholding `fu_types`, which is the only
  legal way to stall a fire-and-forget grant. Src-Data's counter lives HERE:
  `dat_credits`, same width and same full-complement reset, `+1` on
  `coproc.io.dat_credit` and `-1` per beat this node presents. Both reset to the
  FULL complement, not to zero: `tt_cii_channel` keeps no counter of its own and
  the receiver returns credits only on pops, so a zero-initialised counter never
  recovers and the machine looks like a vector hang with no assertion anywhere.

  ===> THE SRC-DATA STALL IS AN ASSERTION, NOT A GATE, AND THAT IS DELIBERATE.
  `opnd` has no stall condition and can have none: its beat is due exactly one
  cycle after the request it answers, and the positional channel would
  desynchronise for every surviving instruction if a beat were held back or
  dropped. So `dat_valid` is NEVER gated on `dat_credits`. The counter exists
  to make exhaustion VISIBLE — assert `dat_credits =/= 0` whenever a beat is
  presented, and trace the zero — because exhaustion is unreachable unless the
  coprocessor has 16 requests outstanding whose data it has not popped, which
  is a VPU-side protocol violation. Gating would convert a detectable
  violation into a silent, permanent hang.

  //@req-spec-cii.b9
  ON THE TWO CHANNELS WHERE THE HOST IS THE RECEIVER IT OWNS THE FIFO AND RETURNS
  ONE CREDIT PER POP. Those two are Src-Request and Writeback, and the FIFOs are
  THIS NODE'S — not the shim's. `tt_cii_host_wrap` instantiates only `tt_cii` and
  the VPU: the relay is pure latency pipes with no buffering, so `req_valid` and
  `wb_valid` arrive as UNREGISTERED passthroughs of the relay beat, and
  `req_credit` and `wb_credit` are TRUE CREDIT RETURNS this node drives and the
  shim forwards unaltered. The old "assert credit, then a registered valid and
  data next cycle" timing of the `addvector` shim does not exist any more; do not
  write a consumer that waits for it.

  ---- 4. The receive buffers are degenerate, and the parameter says why ----

  Both receive buffers are instantiated at `ciiRxDepth` = 0, i.e. the beat is
  presented to its consumer combinationally in the arrival cycle and the credit is
  returned in that same cycle, because the pop IS the arrival. That is legal for
  exactly one reason, and it is the same reason on both channels: THE CONSUMER
  CANNOT STALL. `opnd` owns `R5`-`R8` outright, statically partitioned, with a
  fixed one-cycle read and no other reader, so it accepts a Src-Request beat every
  cycle unconditionally. `wb` owns `W2` outright and places a beat combinationally,
  so it accepts a Writeback beat every cycle unconditionally. A buffer that can
  never be occupied is a wire, and the 16 in `CII_N_REQ_CREDITS` /
  `CII_N_WB_CREDITS` is the SENDER's allowance — how many beats may be in flight
  across the relay before a credit comes back — not a depth the host must build.

  The parameter is the guard rail. If a later edit ever makes either consumer
  stallable — an arbitrated VRF port, a pipelined placement, anything — then
  `ciiRxDepth` becomes non-zero, the credit MOVES to the pop of the real
  buffer, and the beat presented to the consumer is the buffer's head rather
  than the wire. Leaving the credit on `valid` while adding a stall is the
  overflow bug this parameter exists to make impossible to write by accident.

  BEAT GRAIN, and it settles a mismatch between two children. The SV carries ONE
  `req_valid` and ONE `req_credit` for a beat of four lanes, and one `dat_valid`
  and one `dat_credit` likewise; there is no per-lane valid and no per-lane credit
  anywhere in `tt_cii_interface`. So a beat is the unit of credit on both
  channels. This node therefore drives all four of `opnd.io.src_req(i).valid` from
  the single beat valid, and reduces `opnd.io.req_credit` to the one channel bit
  with an assertion that the four lanes agree — they must, since one valid
  qualifies them all. PER-LANE ACTIVITY IS ENCODED IN THE PAYLOAD, by `op_id =
  CII_SRC_NONE` on an unused lane. That is what `NONE` is for, and it is why
  `opnd` must answer a `NONE` lane with a defined don't-care beat rather than
  swallowing it.

  ---- 5. Straight-through lane mapping, and the packing convention ----

  //@req-spec-cii.b6
  THE SRC-DATA LANE MAPPING IS STRAIGHT-THROUGH, NOT COMPACTED, and
  `tt_cii_host_wrap` agrees: a sparse request set — lanes 0 and 2 active, lanes 1
  and 3 carrying `NONE` — is answered on DATA LANES 0 AND 2, with lanes 1 and 3
  carrying the defined don't-care. Nothing shifts down to lanes 0 and 1. This is
  not a preference: with one valid per beat and no per-lane count on the wire,
  compaction is not representable, and mixing the two conventions between host and
  VPU would offset the whole positional channel by one beat for every sparse
  cycle. Within a beat, individual GROUP MEMBERS are selected by `op_offset` on a
  pull and by `wb_dst_offset` on a writeback — a member index, carried per lane,
  never a lane index and never a register number.

  MULTI-LANE PACKING, one rule for all four channels and it must match the SV
  bit for bit: LANE-MAJOR, LANE 0 IN THE LOW BITS, so lane `i` of a field with
  per-lane width `W` occupies `[i*W +: W]`. Chisel slices identically — lane `i`
  is `flat(i*W + W - 1, i*W)`, and the packing direction is `Cat` of the lanes in
  DESCENDING index order. Fields within a lane are assigned individually, never by
  whole-lane bit-cast.

  Three field layouts this node assembles or takes apart, each of which has no
  error detection at all if it is wrong:
    `iss_vtype`, 8 bits, `{vsew[2:0], vlmul[2:0], vta, vma}` with VSEW IN THE HIGH
      BITS. Chisel bundle `asUInt` is MSB-first in declaration order, so
      `CiiIssueReq`'s `vtype` sub-bundle must be DECLARED in that order. Do not
      `asUInt` rocket's `VType` and slice it — its layout is different and carries
      a `vill` bit the packet has no room for.
    `wb_status`, 9 bits, `{last, dst_kind[1:0], vxsat, fflags[4:0]}`, unpacked
      into the named sub-bundle of `CiiWriteback` and never sliced by hand
      downstream.
    `iss_vl` and `iss_vstart`, 9 bits each (`CII_VL_W`), matching the corrected
      `vecVLSz`.

  NAMING TRAP INSIDE THE SHIM, recorded here so nobody re-derives it from the
  package: `tt_cii_interface`'s own `cii_result_t` calls the appended-status
  field `wb_fp_flags`, while the package's `cii_caracal_result_t` calls the same
  bits `wb_status`. Reached through an interface instance it is
  `ifh.wb_data[k].wb_fp_flags`. The Chisel side sees only the flat `wb_status`
  port and is unaffected — but a reader chasing the bits into the SV will meet
  both names for one field.

  ---- 6. The tag: one transaction, threaded end to end ----

  //@req-spec-cii.b5
  //@req-spec-cii.b7
  AN LMUL REGISTER GROUP IS ONE TRANSACTION, identified by its `tag`. One tag is
  allocated per granted `OP.v`, whatever its EMUL: a group of eight members is one
  transaction with one Issue packet, up to four pulls per cycle over as many
  cycles as the coprocessor needs, and up to eight Writeback beats — not eight
  transactions. The CII is variable-latency and may complete internally out of
  order, so RESULTS ARE CORRELATED BACK TO THE ISSUING INSTRUCTION BY TAG and by
  nothing else: not by arrival order, not by an index this node keeps, and not by
  a `rob_idx` on the wire. The only channel that is positional is Src-Data, and it
  is positional in the other direction (request order), which is why it needs no
  tag at all.

  //@req-spec-cii.c8
  //@req-spec-cii.c9
  THE TAG THREADS END TO END AND LABELS EVERY `req` AND `wb` BEAT, and this node
  KEYS ITS SIDE-TABLE ON IT. The table is `tags` (VecCiiTagTable), 16 entries
  direct-mapped by the tag with no CAM anywhere; the tag is opaque — not the
  `rob_idx`, not an architectural register, not a queue index, just an index into
  that array, allocated by the host and echoed back unexamined by the coprocessor.
  Three jobs read it, all of them keyed on the tag off the beat in the beat's own
  cycle: the operand-pull resolve to a PRN, the result placement, and the ROB
  busy-clear on the `last` beat. This node's wiring is what makes "end to end"
  literal — the tag on the Issue packet, the tag on a Src-Request beat and the tag
  on a Writeback beat are the same four bits, copied and never re-encoded, in both
  the flat bundle and the shim.

  ---- 7. Indirection: the coprocessor addresses no register file ----

  //@req-spec-cii.a18
  //@req-spec-cii.f39
  THE COPROCESSOR DRIVES ALL REGISTER-FILE READS AND WRITES INDIRECTLY, THROUGH
  THIS NODE, using abstract operand slots and member offsets rather than register
  numbers — and THE HOST OWNS THE ENTIRE ARN-TO-PRN MAPPING. Every VRF access in
  the CII arm is performed by a child of this container on a port of this
  container: reads on `R5`-`R8` by `opnd`, the write on `W2` by `wb`. The
  coprocessor has no VRF port, no VRF address and no path to one. The mapping it
  is kept away from lives in exactly two places, both here: `VecCiiTagTable`'s
  per-tag entry (the renamed groups snapshotted at issue) and `MicroOp`'s renamed
  fields on the way in.

  //@req-spec-cii.f35
  //@req-spec-cii.f36
  RENAME IS FULLY RESOLVED BEFORE AN OP CROSSES THE CII. By the time `iss` emits a
  packet, `pvdest`, `stale_pvdest`, `pvs1`, `pvs2`, `pvs3`, `pvm` and the scalar
  `pdst` on the granted uop are physical, and the `.vx`/`.vf` source has been read
  from the INT/FP register file and captured BY VALUE. Nothing renameable crosses
  afterwards. Correspondingly THE COPROCESSOR NEVER ADDRESSES A REGISTER FILE BY
  NUMBER, ARCHITECTURAL OR PHYSICAL, and this node's port list is the proof: no
  payload in either direction on any of the four channels carries a register
  number, and there is no input on which one could arrive. A PRN or a rename bit
  appearing in a channel payload is a protocol break, not an optimisation.

  //@req-spec-cii.f37
  The one place architectural specifiers DO cross is the Issue packet's raw 32-bit
  `insn`, whose `vs1`/`vs2`/`vd` fields the coprocessor decodes ONLY to derive its
  operand SET — which `op_id`s to pull, and how many members — and the op
  semantics. It must not, and structurally cannot, use them to address any
  register file: the register a slot resolves to is chosen by `tags` from the
  snapshot, and the coprocessor never learns it. This is also why no RVV decode
  exists on this side of the interface: a second decoder here could disagree with
  `VDecode`'s, and the packet carries the raw word precisely so it does not have
  to.

  //@req-spec-cii.f43
  WHAT THE COPROCESSOR IS THEREFORE FREE OF: rename, wakeup and replay logic. No
  shadow rename table, no `cv0`-`cv31` copy registers, no architectural snapshot
  and no re-execution exist on the SV side, and this node provides no port toward
  them. The three functions live on the host instead — rename in `VecRenameSpace`,
  wakeup in the VECTOR group-done network `done` drives, and recovery in the
  drain-and-discard contract of part 8 — which is what the past-PNR issue gate on
  `IQ_V_ALU` buys. Consistent with that, the SV stack has NO KILL LINE and none is
  being added.

  ---- 8. The kill path, split four ways, and every part holds ----

  `flush` (VecCiiFlush) exports exactly one functional bit, `kill_all`,
  combinational from `io.rob_flush || io.rob_flush_kill` — both terms of the SAME
  event, so the trigger is still `rob.io.flush.valid` only. The state it acts on
  lives in `tags`. Four consumers execute the contract and all four are wired here:

    `tags`  gets `kill_all` and does `tag_killed := tag_killed | tag_valid`. It
            does NOT clear `tag_valid`, does not free on the flush, and does not
            recycle a killed tag: the lifetime is identical to a live tag's.
    `opnd`  reads `killed` from its OWN side-table lookup — it has no flush port —
            and answers a killed pull with a credit plus a defined don't-care
            Src-Data beat and NO VRF read. The beat is mandatory: on Src-Data the
            host is the sender and holds no credit to hand back instead.
    `wb`    pops the beat, returns `wb_credit`, and suppresses the VRF, INT and FP
            writes. Its suppression input is driven HERE — see below.
    `done`  suppresses `group_done`, `clr_rob` and the `fflags`/`vxsat` accrual for
            a killed tag, and frees the tag on the dropped `last` beat anyway.

  THE ONE PIECE OF KILL LOGIC THIS CONTAINER OWNS is the writeback suppression
  term: `wb.io.wb_suppress := tags.wb_lookup.resp.killed || flush.io.kill_all`.
  Both terms are needed and cover different cycles. `killed` covers every beat
  from the cycle after the flush onward, which is almost all of them; `kill_all`
  covers the beat arriving IN the flush cycle, one cycle before `killed` is
  readable out of the registered table. Neither term alone closes the window, and
  the OR belongs here because `flush`'s reject list — correctly — forbids it from
  exporting a per-channel suppress output, while `wb` deliberately reads a single
  pre-computed decision so it needs no flush port, no `killed` vector and no age
  comparator.

  ===> CONSEQUENCE FOR `wb`'s SELF-CHECK: the assertion relating its
  suppression input to the `killed` bit of its lookup response must be the
  IMPLICATION `killed -> wb_suppress`, never an equality. In the flush cycle
  `wb_suppress` is set while `killed` is still clear, and an equality assertion
  fires on that entirely correct case.

  THE FLUSH-CYCLE ALLOCATION RACE is covered twice, and both covers are kept.
  `IQ_V_ALU` gates on `flush_pipeline = RegNext(rob.io.flush.valid)`, so a grant
  can still fire in the `rob_flush` cycle and `iss` will allocate a wrong-path tag
  whose `killed` bit has just been cleared by its own allocation write. Cover one
  is the two-cycle kill window above, which catches that tag on the second cycle
  when it is live. Cover two is `iss` writing the entry with `killed` ALREADY SET
  when `io.rob_flush` was high in either of its two cycles. They are independently
  sufficient; both are cheap and the bit is idempotent, so a doubled set changes
  nothing.

  The alternative — gating `iss`'s grant ACCEPTANCE on `!rob_flush` — is
  REJECTED. It would be correct, but it adds a second recovery shape (drop at
  accept) beside the one that must exist and be exercised anyway (accept, kill,
  drain), and two shapes means two sets of states to verify for a case that
  already costs one OR gate. The corollary for `tags`: its assertion that
  `kill_all` never coincides with `alloc.valid` WILL fire on a real and benign
  case and must be relaxed to "a tag allocated during a kill window carries
  `killed` by the following cycle". Relaxed, not deleted — deleting it turns
  the hole into a wrong-path VRF write plus a `clr_rob` for a dead ROB entry.

  ---- 9. Where the slot-to-PRN mux lives, and where the tag is chosen ----

  TWO RESOLUTIONS THIS CONTAINER MAKES BETWEEN CHILDREN THAT DISAGREED. Both are
  wiring-shape decisions, so they belong here and nowhere else.

  FIRST, the Source-Request resolve uses `tags`' NARROW `src_lookup` port, four
  lanes of request `{tag, op_id, op_offset}` in and response `{prn, read_vrf,
  scalar_data, killed, rob_idx}` out, combinational and unhandshaked. The 16-to-1
  entry mux and the slot-and-member mux behind it are emitted ONCE, inside
  `VecCiiTagTable`. The whole-entry export the operand server assumed is not
  wired: it would cross four copies of a ~376-bit entry between two adjacent
  modules to compute the identical function on the far side, at identical logic
  depth. `opnd` consumes the resolve and owns everything downstream of it — the
  `R5`-`R8` drive, the scalar slot's no-read path, the beat, the order, and the
  killed drain — which is where its own requirements sit. Writeback resolves the
  same way, through `wb_lookup`, whose response `wb` and `done` read in the same
  cycle from one port rather than two.

  SECOND, THE FREE-TAG SELECT LIVES IN `iss`, NOT IN `tags`. `tags` exports
  `tag_free_mask` = `~tag_valid`, `nTags` bits and functional; `iss` picks the
  lowest set bit of `tag_free_mask & ~s1_pending_mask` and presents the chosen tag
  with the entry on `tag_alloc`, which `tags` records at that index. The select
  must sit where the shadow bit does: `iss` chooses a tag in its ACCEPT cycle and
  writes the entry in its EMIT cycle, so `tag_valid` does not show the tag taken
  until a cycle after it was chosen, and a back-to-back grant with no shadow term
  would allocate the same tag twice — two instructions sharing one entry, the
  second overwriting the first's groups. `tags` cannot hold that shadow because it
  never sees the accept cycle. The requirement split already reads this way:
  allocating the tag is `iss`'s (cii.d6), recording the entry is `tags`' (cii.d8).

  ===> REQUIRED EDITS THAT FOLLOW, small and mechanical: `tags` drops its
  `alloc.tag` OUTPUT and its registered `tag_avail` output, and exports
  `tag_free_mask` instead; `opnd` replaces its `tag_entry: VecCiiTagEntry`
  input with the narrow `src_lookup` response. The registered advertise bit is
  `iss`'s single `advertise` flop, computed from the NEXT-state values of BOTH
  resources — the credit AND the free-tag mask. `tags`' own registered
  `tag_avail` would be computed from CURRENT state and is exactly the
  off-by-one that over-advertises `fu_types` by one cycle; with no `ready` line
  on the Issue channel that is a DROPPED INSTRUCTION, not a stall. `iss` as
  written already ANDs both terms and already computes them from next state, so
  nothing about the gate changes — only where the mask comes from.

  ---- 10. Group completion and the one-cycle rule ----

  The Writeback beat this node unpacks is fanned to `wb` and `done` as the same
  wire in the same cycle, and NEITHER registers it. That is a joint constraint,
  not a preference: `wb` places the `last` beat into `W2` and `done` derives the
  group-done from that same beat, and a register on one side only would let a
  group-done — and therefore a dependent issue — precede the write of the group's
  final member by a cycle. Same-cycle is safe because the `W2` write lands at the
  end of the cycle and the earliest dependent read is a cycle later, through issue
  select and register read. Completion is one event per destination group, driven
  by the `last` bit and never by a beat count; no per-member ROB completion leaves
  this subtree and there is no port on which one could.

  ---- 11. VRF read latency: the one thing this node cannot settle alone ----

  The CII arm is wired for OBSERVABLE LATENCY EXACTLY 1 from Src-Request to
  Src-Data, because `srcReadLatency = 1` is what makes the positional channel
  one-beat-in-one-beat-out with a single register per lane and no reordering
  network. WHERE THE FLOP SITS IS VecPipeline's RULING TO MAKE, and this node
  binds either way with a one-line consequence:
    if `VecRegFile` presents COMBINATIONAL read data in the address cycle, `opnd`
      keeps its own payload register and the total is 1;
    if `VecRegFile` presents REGISTERED data at t+1, `opnd` MUST DELETE that
      register, or the total becomes 2, every Src-Data beat arrives a cycle late
      against the request it answers, and the channel is off by one beat forever.
  Under no ruling may both register. If a future edit genuinely needs two cycles,
  `srcReadLatency` goes to 2 and the per-lane ordering depth follows it in the
  same edit — the parameter exists so those two cannot drift apart.

  ---- 12. Trace, and the one open item this node inherits ----

  Guarded `VecTrace` lines, gated on the `vecTrace` plusarg and `!reset`, off by
  default, emitting only — no register and no counter that functional logic reads.
  This node adds exactly three, all at the seam its children cannot see: one per
  Issue beat crossing the flat bundle, one on `dat_credits` reaching zero (the
  protocol violation of part 3), and one per Writeback beat crossing inward with
  its `tag` and `last`. Everything else is traced by the child that holds the
  event. The CII nodes hold no `MicroOp`, so they need `VecTrace`'s tag-keyed
  entry point — a real `rob_idx` out of the side table, never a fabricated uop and
  never `rob=?`.

  OPEN, INHERITED, AND NOT INVENTED AWAY HERE: `IQ_V_ALU` must gate issue on
  `stale_pvdest` READINESS, because the coprocessor pulls `STALE_VD` and this
  node serves it straight from the entry with no busy check of its own. No
  corpus requirement covers stale-dest readiness; the prior M2 implementation
  needed exactly this (`pvold_busy`) to avoid a hang. It is VecIssueSlot's port
  to add, not this node's, and it is reported rather than silently added.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
This container must add ZERO cycles of latency to every channel and zero to the
grant path. Everything in it is combinational: the flat pack and unpack, the
degenerate receive buffers, the credit returns and the kill OR. The only flops it
owns are the Src-Data credit counter and the trace-free assertion state, so the
end-to-end latency of a channel is the relay's pipe depths plus the VPU's own
pipeline plus the one cycle each of `iss` and `opnd` deliberately spends.

Throughput, all sustained and all set by the lane counts rather than by anything
here: one Issue packet per cycle, one Src-Request beat (4 lanes) per cycle, one
Src-Data beat (4 lanes, `4 * vLen` = 1024 bits) per cycle, one Writeback beat per
cycle. This node must never be the reason a beat is not presented in the cycle it
is due.

The two widest paths are the ones to watch: `dat_data` at 1024 bits packed
combinationally out to the BlackBox, and `wb_data` at 256 bits unpacked
combinationally in and fanned to `wb`. If either fails timing the fix is a
registered stage on the CHISEL side with the matching change to `srcReadLatency`
and the ordering depth, or a deeper relay pipe — never a register in the shim,
which would desynchronise a beat from its credit and from its `last` marker, and
never a register on one of `wb`/`done` alone.

The real throughput ceiling is the credit round trip, not this logic: with 16
credits per channel and a round trip of N cycles through the relay's registered
forward and credit-return pipes, the sustained rate is 16/N beats per cycle. If
the zero-credit trace lines dominate, the answer is credit depth or VPU latency in
the SV. Note also that vector arithmetic is in-order inside the VPU by deliberate
choice, so a long-latency op blocks younger independent ones however fast this
attach is.

Area: the flat bundle is wires. `tags`' 16 entries dominate the whole subtree at
roughly 6 kbit of flops; everything else here is a 5-bit counter and a handful of
gates.
<|end_perf|>

<|begin_dependencies|>
Children, one instance each, all elaborated only inside this node's
`usingRVV && enableVectorArith` gate:
  `tags`  VecCiiTagTable      — the 16-entry side-table; owns `tag_valid`,
                                `tag_killed`, and the slot/member resolve behind
                                the narrow `src_lookup`/`wb_lookup` ports.
  `iss`   VecCiiIssue         — the Issue direction: the free-tag select, the
                                packet, the entry content, the Issue credit
                                mirror and the registered `fu_types` gate.
  `opnd`  VecCiiOperandServer — Src-Request in, Src-Data out; owns VRF read ports
                                R5-R8 and the positional ordering.
  `wb`    VecCiiWriteback     — the beat placer; owns VRF write port W2 and the
                                INT/FP scalar-dest writebacks.
  `done`  VecCiiComplete      — the `last`-beat completion: group-done, the ROB
                                clear, the flags accrual and the tag free.
  `flush` VecCiiFlush         — the kill trigger, one combinational `kill_all`.
  `coproc` tt_cii_host_wrap   — the SV flatten shim, reset `core_rst_n`, holding
                                `tt_cii` (the relay, four `tt_cii_channel`s) and
                                `tt_vpu_cii_wrapper_top`. Bound as the `TTCii`
                                BlackBox.

Declared types. `VecCiiTagEntry` and the two lookup request/response pairs are
declared ONCE, in `VecCiiTagTable.nlhdl.scala`, following the `VecBusyResp`
precedent in VecBusyTable: with the narrow-port resolution of part 9 the entry
type crosses exactly one boundary (`iss` to `tags`, same Chisel package, no
import), and the lookup bundles cross only to siblings in this container.
VecBundles must NOT declare it; hierarchy.yaml's VecBundles comment listing it
there is stale. A second copy anywhere is a defect.

VecBundles — `CiiIssueReq`, `CiiSrcReq`, `CiiSrcData`, `CiiWriteback` with its
`wb_status` sub-bundle, `VecGroupDone`, and `VecPipelineIO`. One required edit:
`CiiIssueReq.src_reuse_hint` must widen from 3 to 4 bits to match
`cii_caracal_frwd_hint_t`.

MicroOp — the granted uop on `io.iss`: the renamed groups, `v_emul` (the
DESTINATION group's member count, including the widening doubling and the
single-register-destination override to 1 — confirmed against VDecode, which is
what makes `tags`' prefix mask correct), `is_shared`, `pdst`, `rob_idx`,
`vconfig`, `pvl`, `prs1` and the register-type fields.
VectorParams — `ciiTagBits`, `maxMembers`, `vecPregSz`, `vlPregSz`, `vecVLSz`,
`vLen`, `eLen`, and through them the mirrors of the SV constants.
VecTrace — the three guarded lines of part 12, plus the tag-keyed entry point the
CII nodes need.

tt_cii_caracal_pkg — the frozen contract every width above derives from. Not a
`depends_on` edge in hierarchy.yaml for this node, but the authority behind
VectorParams' CII figures, and the file the build step of part 2 checks against.

Its parent VecPipeline supplies the grant, the VL read, the INT/FP reads and
snoop, the CSR state, the VRF ports and the flush terms, and consumes the
completion, the scalar writebacks and `fu_types`. VecRegFile owns the R5-R8/W2
ports this node drives; the ROB consumes the clear and the flags; VlRegFile
supplies the `pvl` read.
<|end_dependencies|>

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
  VlRegFile — the VL physical register file: 64 entries of ~9 bits, the ONLY
  place a vector VL value is ever stored or read from.
*/

hierarchy.yaml: kind: module, mode: new,
output src/main/scala/v4/vec/generated/regfile/VlRegFile.scala,
package boom.v4.vec.generated.regfile,
depends_on VectorParams, VecTrace. Instantiated exactly once, by VecPipeline
as `vlrf`. Elaborated only when `usingRVV` is true — with vectors off this
module does not exist, it is not tied off, so a non-vector build stays
bit-identical to pre-Caracal BOOM v4.

===> WHY THIS IS A SEPARATE MODULE FROM VecRegFile. The two share a
     rename-space DEFINITION (VecRenameSpace, instantiated twice by
     VecPipeline as `vec_rename` and `vl_rename`) and nothing else. 64x9b of
     flops with a handful of trivial ports has no structure in common with
     96x256b banked four ways with 12 ports, where port count is the dominant
     area term and the flop-vs-latch-vs-SRAM question is the design's #2 risk.
     Rename logic is shared; STORAGE is not. Do not merge these, and do not
     copy VecRegFileBank's read-during-write bypass in here — the logic
     section says why this file does not need one.

===> WHAT THIS MODULE IS NOT. It holds storage and nothing else. No map
     table, no free list, no busy table, no wakeup broadcast, no branch
     snapshot, no `brupdate` port and no flush port. All of that is
     VecRenameSpace's `vl_rename` instance. This module exports no `busy` and
     no `ready` of any kind.

Governing spec anchors: frontend.rst `vector-rvv-decode` ("VSET Special
Handling") and `vl-delivery`, midcore.rst `vl-vtype-rename` and
`regfiles-bypass`, issue.rst `issue-vl-delivery`, loadstore.rst
`elem-progress` ("Fault-only-first").

<|begin_module|>

  <|begin_parameters|>
Every sizing value comes from VectorParams / `BoomCoreParams`; no width below
is a literal.

  `numVlPhysRegisters` — the number of entries. Default 64, straight from
  VectorParams, which also carries the `numVlPhysRegisters >= 1 + coreWidth`
  floor (one committed pointer plus a full dispatch group of producers). This
  module does not restate that require.

  `vlPregSz` — the index width, `log2Ceil(numVlPhysRegisters)`, 6 bits at the
  default. Every address port below is exactly this wide and is a `pvl`, i.e. a
  VL physical register number produced by `vl_rename`.

  `vlWidth` — the entry width, `vecVLSz` from VectorParams. It must hold the
  largest representable VL, which is 256 at VLEN=256 (LMUL=8, SEW=8), i.e. 9
  bits — the spec's "64 entries of about 9 bits". The logic section checks it.

  `coreWidth` — the machine's decode/rename/dispatch width, setting the number of
  rename-side write ports. Inherited, not a knob of this module. With SmallBoom
  removed from the vector configuration matrix (decision D3) the reachable values
  are 2 (Medium), 3 (Large) and 4 (Mega); `coreWidth = 1` does not occur in any
  vector build.

  //@req-spec-decode.c28
  `numAluWritePorts` — the number of integer ALU EUs that may execute a
  register-sourced `vset`. It is `aluWidth`, from `HasBoomCoreParameters`
  (`aluIssueParam.issueWidth`), and it is NOT a default this file chooses: the
  BoomCore-to-VecPipeline interface presents `vset_resp` as `Vec(aluWidth,
  Valid(ExeUnitResp))` and every lane of it lands on its own `W_alu` port here
  (decision D8).

  ===> D8: THE VSET WRITEBACK IS REPLICATED PER ALU EU, ON EVERY TIER, AND THIS IS
       NOT A WIDE-TIER-ONLY PATH. `aluWidth == coreWidth` on every tier, so with
       Small out of the matrix (D3) the matrix is Medium(2)/Large(3)/Mega(4) and
       `aluWidth` is NEVER 1 — there is no configuration in which this collapses to
       a single port, so the replicated form is the only form that is ever built
       and it must not be written as an "if a wide tier ever..." contingency.
       `ALUExeUnit` advertises the `vset` functional unit on EVERY ALU EU instance
       — its reject list forbids advertising the capability conditionally on the
       EU's `id`, because that would make one EU's `fu_types` differ from its
       siblings' for a unit they all contain — so TWO `vset`s genuinely can write
       back in the same cycle.
       ARBITRATION IS NOT AN OPTION, and this is a correctness argument rather than
       a cost one: the VL wakeup a `vset` writeback drives is SINGLE-SHOT, so a
       wakeup lost to arbitration is a PERMANENT HANG, never a stall. That is this
       file's stated "replicate, never arbitrate" discipline (see the write-port
       table), applied where it is cheap: at `numVlPhysRegisters x vlWidth` =
       `64 x 9b`, write ports cost decoders on 576 flops. Totals, so the cost is
       visible rather than asserted: `coreWidth + aluWidth + numLsuWritePorts` —
       Medium `2 + 2 + 1 = 5`, Large `3 + 3 + 1 = 7`, Mega `4 + 4 + 1 = 9`. Nine
       ports on 576 flops is not twelve ports on 24 kbit; the identical discipline
       would be indefensible in the VRF and is free here.
       REJECTED ALTERNATIVE, recorded because it is the plausible one: route
       `vset`s to `IQ_UNQ` instead. `unqWidth = 1` on every tier, so exactly one
       writeback falls out by construction, and the throughput cost is ~nil since a
       strip-mined iteration is 6+ instructions. It was rejected because it would
       require `UniqueExeUnit` — which is not a node in the module map — to
       advertise the `vset` capability, and because it stretches `spec-decode.c7`,
       which says a `vset` must execute on an integer ALU execution unit.

  `numLsuWritePorts` — `vleff` completion write ports. Default 1; VecLsu retires
  at most one `vleff` group-done per cycle.

  `numExeReadPorts` — execute-stage read ports. Default 3, one per vector issue
  queue (`IQ_V_LOAD` -> load AGEN, `IQ_V_STORE` -> store AGEN, `IQ_V_ALU` -> the
  CII host), i.e. one per grant lane that can start a vector uOP in a cycle. A
  tier raising `vecIssueGrantWidth` to 2 raises this proportionally; at 9 bits
  per entry a read port is nearly free, so replicate rather than share.

  `usingRVV` — the Scala `Boolean` from `BoomCoreParams`, not a hardware
  `Bool`. It is the elaboration gate on the whole module.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit pair: posedge `clock`, ACTIVE-HIGH
  SYNCHRONOUS `reset`, in the core clock domain (`core_clk` / `core_reset` in
  hierarchy.yaml). There is no second clock, no asynchronous reset and no
  crossing here — the active-low `reset_n` inversion in the map belongs to the
  SV coprocessor stack, not to this file.

  ---- Write ports (statically partitioned, one class per producer) ----

  //@req-spec-decode.c28
  //@req-spec-decode.c27
  The write side is a fixed table, not an arbiter. There is exactly one write
  port per PRODUCER CLASS, and within a class one port per producer instance:

    W_ren[0 .. coreWidth-1]   rename cycle       `vsetivli`
    W_alu[0 .. aluWidth-1]          int ALU EU writeback  `vsetvli` / `vsetvl`
                                    ONE PORT PER ALU EU, lane-aligned with
                                    `vset_resp` (D8)
    W_lsu[0 .. numLsuWritePorts-1]  LSU writeback         `vleff` trimmed VL

  Each is a `Valid` bundle carrying `addr: UInt(vlPregSz.W)` (the destination
  `pvl`) and `data: UInt(vlWidth.W)` (the new VL). There is no `ready` on any of
  them, and that absence is the point: the ALU and LSU writeback paths have NO
  back-pressure line, so a shared or arbitrated write port would be a structural
  hazard with nowhere to report it. Partitioning makes "no producer can ever
  stall on a VL-RF port" true by construction. At 64x9b the extra decoders are
  negligible — the same discipline as the VRF port table (midcore.rst
  `vrf-ports`), applied where it happens to be cheap. REPLICATE, NEVER ARBITRATE
  is therefore this file's standing rule and not a per-port judgement: it is why
  W_ren is per rename lane, and it is the whole argument for D8's W_alu per ALU EU.

  //@req-spec-decode.c29
  W_ren is replicated PER RENAME LANE because one dispatch bundle may legally
  hold several `vsetivli` — `[vsetivli, vadd, vsetivli, vadd]` is the spec's
  own example — and each lane allocates its own distinct `pvl` in the same
  cycle. A single shared rename-side port would drop the second one silently.

  ---- Read ports ----

  //@req-spec-decode.i7
  //@req-spec-rename.h23
    R_exe[0 .. numExeReadPorts-1]   `addr: UInt(vlPregSz.W)` input,
                                    `data: UInt(vlWidth.W)` output
  Every vector EU reads VL from `VL_RF[pvl]` at EXECUTE, through one of these,
  using the `pvl` the uOP has carried since rename.

  ===> BINDING, FOR EVERY READ PORT INCLUDING `R_commit`: THE VL-RF READ IS
       COMBINATIONAL. Address presented in cycle N, data valid in cycle N — the
       SAME cycle — with no `valid`, no `ready`, no enable, no output flop and no
       read-during-write bypass. An EU that does not need VL this cycle simply
       ignores the data. A consumer that wants the value a cycle later registers it
       ON ITS OWN SIDE; it must not expect this file to.
       THIS IS DELIBERATELY DIFFERENT FROM THE VRF, whose ports are a REGISTERED
       one-cycle read with the flop in `VecRegFile`, and the asymmetry is a SIZING
       FACT, not an inconsistency to be tidied away: this file is `64 x 9b` with
       three execute readers and one commit reader, the VRF is `96 x 256b` with
       nine readers banked four ways. A registered VL read would wrap a flop and a
       valid protocol around a 6-bit decode and a 9-bit mux to save nothing, and it
       would put VL a cycle behind the execute stage that consumes it. Keeping it
       combinational also keeps this file off the issue-stage critical path and
       means it can never be a source of back-pressure.
       Consequences, both settled: `VecCiiIssue`'s reading — combinational on the
       presented address, registering the value into its own emit stage — is
       CORRECT AS WRITTEN and needs no change. `VecScalarOperandRead`'s reading —
       address registered, data next cycle — is WRONG and is being corrected.
       VecPipeline part 9 states the same ruling once for the whole subsystem; the
       two texts must stay identical, and if a fast or speculative VL wakeup is
       ever added, BOTH change together and this file gains a write forward.

  //@req-spec-lsu.g8
    R_commit                        `addr` input, `data` output
  The commit-side read port. At commit of a VL producer the ROB reads
  `VL_RF[committed pvl]` here and that value is what updates the architectural
  `vl` CSR, precisely, in rocket's `CSRFile` (through `csr.io.vector`'s
  `set_vconfig`). It is a separate port from R_exe rather than a borrowed one
  because commit must never contend with execute; and it is a READ of this file
  rather than a value carried in the ROB entry because the ROB entry does not
  hold VL — see the no-bypass rule in the logic section. The same port serves
  `vset` and `vleff` commits identically; nothing about the commit path knows
  which producer wrote the entry. ONE port, not `coreWidth` of them: when a
  commit bundle retires several VL producers only the youngest one's VL becomes
  architectural, so the ROB side selects that lane's `pvl` and drives this
  single address. That select is the ROB's, not this module's — a per-lane read
  port here would invite the wrong `vl` to win the CSR write.

  ---- What is deliberately absent ----

  No `brupdate` input, no `rob_flush` input, no busy or ready output, no wakeup
  output, no debug read port. A reviewer should reject any of these appearing
  here. Speculation recovery is entirely a rename-space concern: a wrong-path
  write lands in a PRN that the free list reclaims and that no surviving
  consumer names, so the stored value becomes unreachable rather than wrong.
  Tracing needs no port either — VecTrace's gate is a `vecTrace` plusarg read
  inside the helper.
  <|end_ports|>

  <|begin_logic|>
  ---- Storage ----

  //@req-spec-decode.i1
  A single register array of `numVlPhysRegisters` entries, each `vlWidth` bits:
  a dedicated VL register file, in its own rename space, with exactly one
  architectural register (`vl`) behind it. It is not a slice of the integer
  file, not a slice of the VRF, and not a CSR mirror. Plain flops with per-port
  decoders; no banking (the whole entry is 9 bits — there is nothing to bank)
  and no SRAM.

  //@req-spec-rename.h4
  //@req-spec-vrf.c1
  The VL value is NOT held in the integer register file. This is the whole
  reason the module exists: BOOM's integer rename, free list, busy table and
  bypass network are untouched by Caracal, and they could not have been if VL
  had been renamed as a GPR. A `vset` with `rd != x0` writes `rd` in the
  integer RF as an ordinary integer destination from the same result bus — that
  write is unrelated to this file and is not visible here.

  Reset the ENTIRE array to 0 on `reset`. That is 576 flops at the defaults, so
  the cost is nil, and it buys two things: the entry the committed VL pointer
  names after reset reads as VL=0, matching `vl`'s architectural reset value,
  and no read can ever return X. The VRF deliberately does NOT reset its 24 kbit
  of storage; the trade is different at 576 bits, and X-cleanliness matters more
  here because a VL of X does not corrupt one lane — it makes every downstream
  element count meaningless and gives the cosim no usable first mismatch.

  ---- The rename-cycle write (`vsetivli`) ----

  //@req-spec-decode.c4
  //@req-spec-issue.h4
  A `vsetivli` is FRONT-END ONLY: both `vtype` and AVL are immediate, so
  VConfigUnit resolves it at decode and computes `VL = min(uimm, VLMAX)`. The
  resulting value is written to `VL_RF[pvl]` in the RENAME cycle, NOT at
  decode, for a mechanical reason: `pvl` is allocated by `vl_rename` at rename,
  so at decode there is no VL-RF index to write to. W_ren[w] therefore carries
  the decode-computed VL forward one stage and writes it where the index
  becomes known. No back-end issue slot and no EU are involved.

  LOCKSTEP: W_ren[w].valid must be qualified by the SAME `dis_fire(w)` that
  qualifies vl_rename's allocation on lane w, off `ren2_uops` — never
  combinationally off `dec_uops`. The M1 free-list double-free was exactly
  this: vector rename running one cycle ahead of the scalar RenameStage's
  registered ren1->ren2 pipeline, so the fields seen at dispatch belonged to
  the next cycle's bubble uop. A W_ren write that fires on a lane whose
  allocation did not would scribble a PRN this producer does not own.
  The write sets no busy bit — there is no busy state in this module at all —
  and `vl_rename` leaves `pvl_busy` clear for a `vsetivli`, so its `pvl` is
  BORN READY and a dependent vector uOP in the same or the next dispatch group
  never waits on it. The uOP has no writeback, so its ROB entry is dispatched
  non-busy.

  ---- The ALU writeback (`vsetvli` / `vsetvl`) ----

  `vsetvli` and `vsetvl` take VL (and, for `vsetvl`, `vtype`) from a GPR, so
  they execute on an integer ALU EU, woken by `rs1`/`rs2` on the INTEGER wakeup
  network. The ALU computes `VL = min(rs1, VLMAX)` and its writeback targets
  W_alu with `is_vl_producer` as the write enable. `is_vl_producer` and not
  `dst_rtype`: with `rd == x0` the uOP's `dst_rtype` is `RT_ZERO` and the VL RF
  is still written, so VL-producing is not inferable from `dst_rtype` at all.
  The same result bus feeds the integer RF when `dst_rtype === RT_FIX`, because
  `rd` receives the new `vl` — one value, two destinations, two rename spaces.

  There are `aluWidth` of these ports, not one (D8), and lane `i` is driven from
  `vset_resp(i)` — the writeback of ALU EU `i` — with NO mux, NO arbiter and no
  ordering between lanes. Two `vset`s in one cycle is a NORMAL case, not an
  exceptional one: every ALU EU advertises the `vset` FU, and `aluWidth >= 2` on
  every configuration in the vector matrix. Both writes land at the same posedge
  into two different entries, and both `pvl`s go out on the VL wakeup network on
  their own lanes (`numVlWakeupPorts = aluWidth + 1`, sized in VectorParams), so
  neither single-shot wakeup can be lost. The regression must actually exercise two
  same-cycle `vset`s — `[vsetvli, vadd, vsetvli, vadd]` in one bundle is the spec's
  own example and it reaches this file as two concurrent W_alu lanes.

  ---- The `vleff` writeback ----

  //@req-spec-lsu.g6
  `vleff.v` is the third producer class. On completion VecLsu writes the final
  element count through W_lsu to the `vleff`'s own VL-RF destination — the full
  VL if no element faulted, or `i` if element `i > 0` faulted and VL was
  trimmed. That is bit-for-bit the same transaction a `vset` performs, which is
  the design's reason for not serializing `vleff`: its trimmed VL reaches
  consumers through `pvl` and the VL wakeup network with correct ordering by
  construction, so `is_unique` buys nothing. A fault on element 0 is a normal
  precise trap instead and writes nothing here.

  STAGING, not a contradiction: VLSDecode's hierarchy.yaml entry says `vleff`
  is NOT a VL producer until the real fault-trim path exists. W_lsu therefore
  exists in the port table from day one and stays permanently invalid until
  that step lands. Structuring it that way is deliberate — c28 asks for one
  port per producer CLASS, and retrofitting a third write port later is the
  change most likely to be done as an arbiter on W_alu instead.

  ---- No two ports may target the same entry ----

  In any cycle every valid write port carries a DISTINCT `pvl`: each producer
  writes a PRN `vl_rename`'s free list handed it, not reused until commit frees
  it. Rely on that — the write path is a per-port decoder feeding a plain
  per-entry enable, no priority mux and no defined winner — and ASSERT it over
  all valid write-port address pairs, because a violation would show up as a
  silently wrong VL rather than as a structural error. The assertion must range
  over ALL `coreWidth + aluWidth + numLsuWritePorts` ports pairwise, W_alu lanes
  against each other included: two ALU EUs retiring `vset`s in the same cycle
  (D8) is the case most likely to be omitted from a hand-written check, and it is
  also the one the free list makes safe — each `vset` renamed its own `pvl`.

  ---- No read-during-write bypass, and why that is safe ----

  Writes are registered; a read sees a written entry from the NEXT cycle. No
  forwarding from any write port into any read port is needed, because every
  producer-to-consumer path has at least one cycle between the write and the
  consumer's execute read: a rename-cycle write is followed by dispatch, issue
  and register-read, and an ALU or LSU writeback broadcasts `pvl` on the VL
  wakeup network in the cycle it writes, after which the woken slot still has to
  be granted. This is the OPPOSITE of VecRegFileBank, where read-during-write
  forwarding is a stated requirement (vrf.f8). Do not copy that bypass here.

  SEAM CONSTRAINT, not an optimization: if any tier ever adds a speculative
  or fast VL wakeup that lets a consumer read in the SAME cycle as the
  write, this module needs a bypass mux and that change must come back here.
  Assert the distance instead of assuming it silently.

  ---- The read side is the ONLY way VL leaves this module ----

  //@req-spec-decode.i18
  //@req-spec-rename.h25
  Vector consumers read VL only from here. The module has no other output: no
  broadcast, no snapshot, no side channel into the uOP, no path into the VCFG
  mirror. VConfigUnit mirrors `vtype` and never `vl`, so there is no
  decode-stage VL value for anyone to read, and a `vsetvli`'s computed VL is
  never broadcast back to the front end.

  //@req-spec-decode.i17
  There is NO statically-known-VL path that bypasses this file. In every
  producing case — immediate (`vsetivli`), register (`vsetvli`/`vsetvl`), and
  trimmed (`vleff`) — VL is written to an entry here and read back via `pvl`.
  The tempting shortcut is the `vsetivli` case, where VL is a compile-time
  constant known at decode: a `vl_is_known`/`vl_imm` field on the MicroOp would
  let a consumer skip the read. It is deliberately absent (MicroOp's reject
  list names it), because it creates a second source of truth for VL that the
  keep-VL form of `vsetvli`, a mispredict, or a `vleff` trim can each falsify
  independently. One producer path, one storage array, one read path.

  ---- Tracing ----

  Emit one guarded VecTrace line per WRITE event through `traceStruct` — this
  module has neither a `MicroOp` nor a `rob_idx` on its boundary, so the
  structure-scoped rung is the honest one; key each line on `prn` and `vl`.
  Module `VlRegFile`, events `wr_ren`, `wr_alu` and `wr_lsu`, each gated on its
  port's own `valid`. Gated additionally on VecTrace's `vecTrace` plusarg and
  `!reset`, off by default, and adding no state — with the plusarg absent the
  emitted RTL is identical to the same design with the calls deleted.

  **READS ARE NOT TRACED — neither `R_exe` NOR `R_commit`.** Both are bare
  `addr`/`data` pairs with no valid or enable, so a line on either fires every
  cycle per port and buries the events that matter.

  ===> `rd_commit` WAS MANDATED HERE AND IS NOW REMOVED, because the reason
  this paragraph already gives for excluding `R_exe` applies verbatim to
  `R_commit`: it has no valid bit, so the module cannot distinguish a genuine
  VL-producer commit from any other cycle. Generation implemented it literally
  and reported the tension — the line fired unconditionally. Gating it would
  need an enable input added SOLELY to make a trace line emit, which the
  VecTrace spec explicitly forbids ("a module should not acquire a port purely
  to be traceable"), and the commit event is already observable from the
  consumer side, where a `rob_idx` genuinely exists: the ROB / VConfigUnit
  commit path can trace it with `traceId`. An every-cycle line in the only
  debug instrument the plan has is worse than no line. Corrected 2026-08-10.

  These three write lines are the whole debug surface for VL from this side — the
  project has no unit tests, validation is end-to-end VCS plus Whisper cosim
  only, and a wrong VL is otherwise indistinguishable at the cosim boundary from
  an AGEN element-count bug.

  ---- Capacity check on the entry width ----

  Require `vlWidth >= log2Ceil(maxVLMAX + 1)`, where
  `maxVLMAX = vLen * maxMembers / 8` is the largest VL the machine can produce
  (LMUL=8 at SEW=8) — 256 at VLEN=256, needing 9 bits to hold 0..256
  inclusive. Check it AT ELABORATION with a `require` that names `vecVLSz`,
  because a too-narrow entry does not fail: it truncates, and VL=256 stored in
  a narrower field reads back as 0, which the machine then executes as a
  zero-length vector op. That is a silent-wrong-answer failure mode reachable
  only at the widest LMUL, i.e. exactly the case a short regression misses.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Area: `numVlPhysRegisters * vlWidth` = 576 flops at the defaults, plus decoders
for `coreWidth + aluWidth + numLsuWritePorts` write ports — Medium `2+2+1 = 5`,
Large `3+3+1 = 7`, Mega `4+4+1 = 9` (D8) — and 4 read ports (3 execute + 1
commit). Negligible against the VRF's 24 kbit even at nine write ports, which is
why the static port partition is affordable here and replication always beats
sharing. It is the ONLY reason: the same nine-port replication on the VRF would be
unbuildable, so this argument must never be carried across to that file.

Latency: reads are COMBINATIONAL — address to data in the SAME cycle, inside the
consumer's register-read stage, a 6-bit decode and a 9-bit mux. That is a binding
contract, not a target (see the ports section), and it is deliberately unlike the
VRF's registered one-cycle port. This module must not be pipelined and must not
appear in a critical-path report; if it does, the fault is in the addressing logic
feeding it, not here.

Writes take effect at the next posedge. Throughput is one write per port per
cycle with no arbitration and no stall cycle, ever: `coreWidth` rename-side
writes plus `aluWidth` ALU writes plus one LSU write can ALL land in the same
cycle — 9 at Mega — and that case must be exercised in the regression rather than
assumed.

Gate (f): with `usingRVV = false` this module is not elaborated at all.
<|end_perf|>

<|begin_dependencies|>
VectorParams — `numVlPhysRegisters`, `vlPregSz`, `vecVLSz`, `vLen` and
`maxMembers` (the last two only for the elaboration-time width check). Nothing
here re-derives a width that file already names. `numVlWakeupPorts` = `aluWidth + 1`
is declared there too; this file does not own the VL wakeup network but its write
ports are what that count is derived FROM (per-ALU `vset` writeback plus the
`vleff` trim), so the two must move together. `coreWidth` and `aluWidth` come from
`HasBoomCoreParameters`.

VecTrace — the guarded-printf helpers, `traceVl` in particular.

Instantiates nothing. This is a leaf.

Instantiated by VecPipeline exactly once, as `vlrf`, alongside `vrf`
(VecRegFile) and the two VecRenameSpace instances. Its port peers, all wired at
that level: VecRenameSpace `vl_rename` supplies every `pvl` this file is
addressed by and owns the map table, free list, busy table and VL wakeup network
that this file deliberately does not; VConfigUnit/VecDecode supply the
decode-computed `vsetivli` VL that arrives on W_ren one stage later; EACH of
BoomCore's `aluWidth` integer ALU EUs drives ITS OWN W_alu lane through the
interface's `vset_resp`, which is `Vec(aluWidth, Valid(ExeUnitResp))`; VecLsu drives
W_lsu with the `vleff` trimmed count; the load AGEN, store AGEN and VecCiiHost
take the R_exe ports; and the ROB / `csr.io.vector` commit path takes R_commit.
<|end_dependencies|>

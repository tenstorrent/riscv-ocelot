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
  VecScalarOperandRead — the operand-read stage of ONE vector memory direction:
  it reads the scalar feeders (base GPR and stride GPR) out of the INT register
  file and resolves VL out of the VL register file, then hands the result to that
  direction's address generators.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecScalarOperandRead.scala,
  package boom.v4.vec.generated.lsu. group vec_lsu.
  depends_on MicroOp, VecBundles, VecTrace.
  Instantiated TWICE by VecLsu: `ld_opnd` (isStore = false) and `st_opnd`
  (isStore = true). Direction is a PARAMETER, never a separate module.

  ===> WHY TWO INSTANCES AND NOT ONE SHARED STAGE. `addvector` had a single
       shared `VecLSRegRead` serving both directions behind a LOAD-PRIORITY MUX.
       When both queues granted in the same cycle the store's grant was SILENTLY
       LOST — a fire-and-forget `Valid` has no ready line, so there was nothing
       to back-pressure and nothing to notice. It was worked around by making the
       store not advertise `FC_AGEN` in any cycle the load was granting, which is
       a throughput ceiling dressed up as a fix. Two instances delete the
       arbitration instead of tuning it: structural change 3 of the v2 plan (§2,
       "Direction is a parameter").

  ===> WHY THERE IS NO FSM HERE. The predecessor was a four-state machine
       (sIdle/sReq1/sReq2/sRrd) reading `rs1` then `rs2` serially on one shared
       INT port, exporting a `fu_ready` gated on the whole vector-LS pipe being
       idle — the vector-LSU invariant's forbidden `busy`, and the reason the
       machine held one vector memory op at a time. This module is a ONE-STAGE
       PIPELINE with two PARALLEL INT reads, exporting no readiness of any kind
       toward the issue unit (plan §5 rule 6). Its only variability is INT
       register-file arbitration (D5 below): the reads are `Decoupled` and a
       denied read repeats, so latency is one cycle plus the number of cycles the
       INT file denied. That is arbitration on a read port, NOT a state machine
       over the instruction and NOT a `busy`.

  ===> D4 — THERE IS NO FP READ HERE ANY MORE, AND THE DELETION IS THE POINT.
       An earlier revision of this file read the FP register file on the store
       instance, because `execution.rst` said the store DGEN reads "the FP/INT
       register file for scalar source operands, for instance `vfmul.vf`".
       `vfmul.vf` is a vector-scalar floating-point MULTIPLY — arithmetic,
       dispatched to the CII — and it is NOT A STORE. No RVV store form takes an
       FP scalar operand at all: store data is always `vs3`, and a store's scalar
       operands are `rs1` (base) and `rs2` (stride), both INTEGER. So the FP leg
       was unreachable logic, and `vec_pipeline_io.fp_rf_read_req` is back to ONE
       lane, whose single reader is `VecCiiIssue`'s `.vf` capture. `execution.rst`
       is amended (D12 case 1) and `spec-agen.d3` is re-worded onto the INT read,
       which is the part of it that exists.
       WHAT IS *NOT* DELETED: `spec-vrf.e4` and `spec-issue.g8/g9/g10` are
       UNCHANGED. The vector issue slot still matches `.vf` on the FP wakeup
       network, because that is how it learns a scalar-FP operand is ready. Only
       the store-side READER is gone.

  ===> D5 — THE INT READ IS PER-LANE `Decoupled` AND HOLDS ITS ADDRESS UNTIL
       `fire`. An earlier revision claimed "DEDICATED, UNARBITRATED ports with no
       ready line — address driven in the grant cycle". That is false about the
       machine it connects to. `iregfile.io.arb_read_reqs` is
       `Flipped(Decoupled(...))` and `PartiallyPortedRF` denies BY INDEX PRIORITY
       (`ready := PopCount(io.arb_read_reqs.take(i).map(_.valid)) <
       numPhysicalReadPorts`); the vector lanes are appended LAST, and there are
       roughly 7-9 existing logical readers plus 5 vector lanes against FIVE
       physical ports on Medium. DENIAL IS ROUTINE, not exceptional, and no
       affordable port count or placement fixes it. A bare address would silently
       return whatever register the arbiter granted instead — a wrong base
       address with no error anywhere.
       THE DOWNSTREAM CONTRACT IS UNCHANGED: `out` stays a `Valid` that the agen
       latches unconditionally. Only the UPSTREAM link becomes handshaked.

  ===> AND THE VL-RF READ IS COMBINATIONAL, which this file also had wrong.
       `VecPipeline` part 9 is canonical: address presented, data valid in the
       SAME cycle, no valid, no ready, no enable, no read-during-write bypass. It
       differs from the VRF deliberately — 64 entries of 9 bits with three
       execute readers is not 96 entries of 256 bits with nine. This module may
       register the returned value on ITS OWN side (and does, below), but it must
       not expect the file to.

  Governing spec anchors: issue.rst `shared-store-chain` and
  `issue-vl-delivery`, execution.rst `vector-dgen` (as amended by D4),
  case_study.rst `case-vl-zero`, and plan §2 bug (3) "stale scalar base (`prs1`
  RAW race)".

<|begin_module|>

  <|begin_parameters|>
  `isStore: Boolean`, default `false` — the direction this instance serves.
  `false` is the load path (granted by `iq_v_load`, feeding
  `ld_elem_agen`/`ld_range_agen`); `true` is the store path (granted by
  `iq_v_store`, feeding `st_elem_agen`/`st_range_agen` and `dgen`). Exactly those
  two values are legal; VecLsu passes it explicitly on both instances.

  It is the ONLY parameter. Everything else derives from the implicit
  `Parameters`: `xLen` and `maxPregSz` from `HasBoomCoreParameters`, `vlPregSz`
  and `vecVLSz` reached through VecBundles (which owns the VectorParams
  binding), and BoomCore's `numIrfWritePorts` for the writeback-snoop width. No
  width here may be a literal. The module is elaborated only under `usingRVV`,
  because it exists only under VecPipeline — absent in a vectors-off build, not
  tied off.

  Deliberately ABSENT, because either would be the shared-stage mistake in a new
  form: any notion of which direction has priority, and any queue depth. This
  module holds no queue.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel default and the hierarchy.yaml default:
  posedge `core_clk`, ACTIVE-HIGH SYNCHRONOUS `core_reset`, implicit as usual
  for a `BoomModule`. The module's only reset-sensitive state is the stage-valid
  bit and the two per-lane `rr_need` bits, all of which must reset to 0 — a
  `rr_need` bit surviving reset would drive a held INT read request out of a module
  with no grant behind it.

  ---- the grant ----

  `iss` — `Input(Valid(new MicroOp))`, this direction's issue grant from
  `iq_v_load` or `iq_v_store`. STILL fire-and-forget after D5: there is NO ready
  line and NO `fu_ready`, `busy` or credit output anywhere on this module, so no
  back-pressure can reach the issue unit. The handshake D5 adds is on the INT
  register-file side only and points away from the issue unit; the one case where
  that leaves a grant unabsorbed is flagged explicitly in the logic section and
  owned by `VecLsu`'s descriptor table, not hidden behind this sentence. The granted
  `uop.fu_code` distinguishes BOOM's existing `FC_AGEN` and `FC_DGEN` paths; a
  store slot is granted for each independently (VecStoreDgenPath), and both use
  this one port because at most one grant per cycle arrives on it.

  `brupdate` — `Input(new BrUpdateInfo)`, and `rob_flush` — `Input(Bool())`.
  Used with BOOM's existing `IsKilledByBranch` to kill the in-flight read; no
  new squash mechanism is added (plan §5 rule 10).

  ---- scalar feeders: the INT register file ----

  `int_rf_read_req` — `Output(Vec(2, Decoupled(UInt(ipregSz.W))))`: index 0 the
  base register `uop.prs1`, index 1 the stride register `uop.prs2`. PER-LANE
  `Decoupled` (D5), connected by the BoomCore delta to
  `iregfile.io.arb_read_reqs(numScalarLogicalReadPorts + n)`, which is
  `Flipped(Decoupled)` and denies by index priority. Each lane asserts `valid`
  from the grant cycle and HOLDS ITS ADDRESS STABLE UNTIL `fire` — exactly what
  every scalar execution unit already does in its own arbitrate stage, so this is
  ground rule 10 (reuse BOOM's machinery), not a new protocol. The two lanes are
  CONCURRENT and NOT serialized, and they fire INDEPENDENTLY: index priority can
  grant lane 0 and deny lane 1 in the same cycle, so each carries its own held
  address and its own fired bit. Serializing them onto one port is what forced
  the predecessor's FSM to exist.

  ON THE ADDRESS WIDTH, corrected at E-prep: `ipregSz`, not `maxPregSz`, and the
  distinction is not cosmetic. `maxPregSz` is `ipregSz max fpregSz` and is the width
  of the uop FIELD (`MicroOp.prs1` holds an INT, FP or vector PRN in one field);
  `ipregSz` is the width of the INT register file's actual `arb_read_reqs` port,
  which is what this output BINDS to with `<>`. They are equal only while
  `numIntPhysRegs >= numFpPhysRegs`, so writing `maxPregSz` here builds today and
  fails to connect on the first config that renames more FP registers than INT.
  Take `uop.prs1`/`uop.prs2` and narrow at this port. `int_wb_snoop.addr` below
  legitimately stays `maxPregSz`: it is compared against the uop field, not bound to
  a register-file port.
  `int_rf_read_rsp` — `Input(Vec(2, UInt(xLen.W)))`. `PartiallyPortedRF` reads
  `regfile(RegNext(arb_read_reqs.bits))`, so a lane's data is valid in the cycle
  AFTER THAT LANE FIRED, off the GRANTED address — never off an address that was
  presented and denied. It is registered here in that cycle, per lane.

  The `ready` on these lanes is NOT back-pressure toward the issue unit and
  must never be turned into any. The grant has already happened; this is a
  read port losing an arbitration inside the register file, and nothing about
  it reaches `iq_v_load`/`iq_v_store`. Ground rule 6 and gate H4 are about a
  `busy`/credit/ready presented to an ISSUE UNIT, which this module still does
  not have in any form.

  `int_wb_snoop` — `Input(Vec(numIrfWritePorts, Valid(new Bundle { addr:
  UInt(maxPregSz.W); data: UInt(xLen.W) })))`. THE SAME SIGNALS THAT WRITE
  `iregfile`'s write ports, not the wakeup bus: the forward below needs write
  DATA, which a wakeup port does not carry. The width must be BoomCore's full
  `numIrfWritePorts`, INCLUDING the vector scalar-destination write port added
  under `enableVectorArith` — otherwise a `vmv.x.s` result used as a base
  address is the one case the forward misses.

  ---- NO FP PORT ON THIS MODULE (D4), and VL ----

  There is no `fp_rf_read_req`/`fp_rf_read_rsp` here, on either instance. An
  earlier revision elaborated them when `isStore`; D4 deleted them, because no
  RVV store form has an FP scalar operand (see the D4 note in the header). The
  one FP read lane on `vec_pipeline_io` belongs to `VecCiiIssue`, and the port
  FpPipeline's delta adds is ITS port, not this module's. This module reads the
  INT register file and the VL register file, and nothing else.

  `vl_read_addr` — `Output(UInt(vlPregSz.W))`, driven with `uop.pvl_src` — and
  `vl_read_data` — `Input(UInt(vecVLSz.W))`, the VL RF's COMBINATIONAL read
  result, valid in the SAME CYCLE as the address (VecPipeline part 9: no valid,
  no ready, no enable). Connected to `vlrf` inside VecPipeline. There is no
  handshake to lose and no denial to survive here — the VL RF is fully ported by
  construction (three `R_exe` ports, one per reader), which is exactly why it can
  be combinational while the INT file cannot be unarbitrated.

  ---- the result ----

  `out` — `Output(Valid(new VecScalarOperands))`, asserted for exactly ONE CYCLE
  per grant: the cycle after the LAST of that grant's INT lanes fired, which is
  the cycle after the grant whenever the INT file granted immediately. Fields:
  `uop` (the granted `MicroOp`, `fu_code` preserved so the consumer knows whether
  this is the AGEN or the DGEN half), `base` and `stride` (`UInt(xLen.W)`),
  `scalar_data` (`UInt(xLen.W)`, store instance only), `vl` (`UInt(vecVLSz.W)`)
  and `vl_zero` (`Bool`).

  THIS SIDE OF THE MODULE IS UNCHANGED BY D5, DELIBERATELY. No ready line here
  either: the consumer must latch it unconditionally, which it can, because the
  element-queue capacity was already reserved at dispatch (VecQueueReservation). A
  ready here would be a back-pressure path from the queues to the issue unit
  wearing a different name. `out.valid` now IMPLIES every lane of that grant
  fired, so a descriptor can never leave this module carrying an un-granted read —
  the handshake is absorbed entirely upstream of `out`.

  WHERE THE DESCRIPTOR THEN WAITS, AND WHY IT IS NOT HERE. `VecLsu` owns a
  PER-LDQ/STQ-ENTRY DESCRIPTOR PENDING TABLE (its section 3b) which presents a
  descriptor to an agen only when that direction's mask streamer is free and
  the INT/VL reads have been granted. D5 amends GROUND RULE 6 to permit that
  table as the THIRD legal home for in-flight vector-LSU state — (a) the six
  `VecElemQueue`s, (b) the LCB's per-PRN assembly entries, (c) that table —
  because it is the same kind of state as (a): per-queue-entry, capacity
  reserved at dispatch, no `busy` exported, structurally un-overflowable. That
  structure is REFERENCED here and duplicated nowhere: this module keeps no
  queue, no second copy of the descriptor and no per-instruction row.

  `out.bits` carries no mask, index-group, EEW or EMUL field — those ride the
  wrapped `MicroOp` as `pvm`, `pvs2`, `v_eew` and `v_emul`, and restating them
  would give the consumer two places to read the same thing from, one of which
  would go stale (the rule VecBundles applies to `VecElemAccess`). No `vstart`
  field either: per plan §5 rule 8 a vector memory access begins at element 0.

  BUNDLE-LOCATION NOTE. `VecScalarOperands` crosses a node boundary (into
  VecElemAgen, VecRangeAgen, VecDgen and VecGroupCopy), so by the convention
  VecBundles states it belongs in VecBundles. VecBundles as written does not
  declare it. Declared here for now; Phase R should move it, unchanged.
  <|end_ports|>

  <|begin_logic|>
  ---- Structure: one stage, no instruction-scoped state, a held read ----

  Cycle 0 (grant): `int_rf_read_req(0).bits` := `iss.bits.prs1`,
  `int_rf_read_req(1).bits` := `iss.bits.prs2` with both `valid`s asserted, and
  `vl_read_addr` := `iss.bits.pvl_src` — all driven COMBINATIONALLY from `iss.bits`,
  so no cycle is spent deciding to read. The grant is captured into the stage
  register set: `rr_uop` (a `MicroOp`), `rr_valid` (a `Bool`, reset to 0),
  `rr_need` (2 bits, one per INT lane, set for the lanes that have not yet
  fired), `rr_data` (2 x `xLen`, the per-lane captured response) and `rr_vl`.

  BECAUSE THE VL RF IS COMBINATIONAL, VL IS RESOLVED IN CYCLE 0 AND NEVER WAITS.
  `vl_read_data` returns in the same cycle as the address, and its value is
  captured straight into `rr_vl` alongside the uop. VL therefore cannot be
  desynchronized from the grant by an INT denial, and the VL RF needs no address
  hold, no valid and no second read.

  While `rr_valid` holds and `rr_need(n)` is still set, lane `n` re-presents the
  HELD address from `rr_uop` (`prs1` for lane 0, `prs2` for lane 1) with `valid`
  high; on `int_rf_read_req(n).fire` the lane's `rr_need(n)` clears and the
  NEXT cycle captures `int_rf_read_rsp(n)` (post-forward, below) into `rr_data(n)`.
  `out.valid` asserts in the first cycle in which every lane's data has been
  captured and `!killed` — one cycle after the grant when both lanes fired in the
  grant cycle, which is the uncontended case.

  The address a lane presents is therefore `Mux(rr_valid && rr_need(n), <held from
  rr_uop>, <from iss.bits>)`, with `valid := (rr_valid && rr_need(n)) || iss.valid`.
  The two terms are MUTUALLY EXCLUSIVE by the assertion in the flagged note below,
  so the `Mux`'s priority is not a design decision hiding in an implementation
  detail — a cycle in which both were live would be a lost grant, which is exactly
  what that assertion catches.

  //@req-spec-issue.h3
  Nothing else is stored: no queue, no element cursor, no per-instruction row, and
  nothing that could be called "the current vector memory instruction" beyond the
  one grant whose reads are outstanding. It exports no `busy`, no `fu_ready` and no
  credit to any issue unit, satisfying the vector-LSU invariant by construction
  rather than by review (plan §5 rule 6) — the `ready` it CONSUMES on the INT lanes
  points the other way, into the register file, and is never re-exported. Two
  instances exist, one per direction, and neither can see the other, so there is no
  priority mux between directions and no grant lost to one.

  ===> CLOSED ON THE OTHER SIDE OF THE SEAM, AND RECORDED BECAUSE THE CHECK
  STAYS HERE: A GRANT LANDING ON A
  STILL-UNFIRED HOLD. The hold above is ONE grant deep, and this module exports
  no readiness, so `iq_v_load`/`iq_v_store` may grant again in the very next
  cycle while a lane is still denied. D5 states the grant is "accepted
  unconditionally (no `busy`, no dropped grant — the row is indexed by an
  LDQ/STQ entry the reservation already guaranteed, so overflow is
  unrepresentable)", which is only true if the absorbing row EXISTS FROM THE
  GRANT CYCLE. `VecLsu`'s section 3b now does exactly that: the row is written
  AT THE GRANT, unconditionally, with clear-on-fire behind it, and the held read
  address is driven from the PRESENTED ROW — so the window is covered where the
  state legitimately lives. It was never closable here, without either a second
  copy of the descriptor table (which ground rule 6(c) puts in `VecLsu`,
  singular) or a readiness output (which gate H4 rejects). What stays here is
  the CHECK: this module ASSERTS `!(iss.valid && rr_valid && rr_need.orR)`, and
  it firing means a fresh descriptor reached this module while its one-deep hold
  was still unfired — i.e. the grant-cycle row write upstream did not do its
  job. Checked, not assumed away.

  ===> SETTLED, decision D5 retry model (a): THE PER-LANE HOLD LIVES HERE, AND
       THE TABLE GOES QUIET ONCE IT HAS HANDED A ROW OVER. A partial grant
       ACCUMULATES — `prs1` fires and stays fired while only the still-
       outstanding lanes re-request — because the vector lanes are appended LAST
       in `PartiallyPortedRF`'s index priority and denial is routine, so
       requiring base AND stride to win in one cycle would be a livelock under
       sustained scalar pressure rather than a slow path, and would re-serialize
       the read the 3 -> 5 seam widening existed to parallelize. Accumulation
       makes progress MONOTONIC: the worst case is the unluckiest single lane,
       not the coincidence of all of them.
       The reciprocal obligation on `VecLsu`, which its file now states: a
       presented row is a HAND-OFF, so the table drops `valid` toward this
       module once the row is accepted and must NOT leave it presented and
       re-requesting. Leaving it high is exactly what the assertion above
       catches, and under the accumulating model it would fire on every denial.

  ---- VL is resolved HERE, at execute ----

  //@req-spec-issue.h3
  VL is a source operand in its own rename space and there is no decode-time VL
  value. `pvl_src` is a PLAIN READINESS WAKEUP on the VL network — the issue slot
  captures no VL value, only the ready bit — and the VALUE is read from the VL
  register file AT EXECUTE, here, at the point where the granted `OP.v` is about
  to be cracked into element accesses. Hence `vl_read_addr` from the READ PRN
  `iss.bits.pvl_src` and NOT `pvl`, which on a `vle*ff.v` is that uOP's own
  as-yet-unwritten VL destination (see VecRenameSpace part 3),
  in the grant cycle, `rr_vl` from the SAME-CYCLE `vl_read_data`, and
  `out.bits.vl` from `rr_vl`; hence also `MicroOp` carrying no `vl` field and no
  `vl_is_known` flag.

  Two consumers depend on VL becoming known exactly here and no earlier: the
  agens, which size the element walk by total bytes (`vl << eew`), and
  VecQueueReservation, which releases the SURPLUS of the worst-case capacity it
  claimed at dispatch. `out.bits.vl` is the single source for both, so the
  release and the walk cannot disagree about how many elements exist.

  CORRECTED (VecPipeline part 9, canonical): the VL RF is COMBINATIONAL and
  has NO read-during-write forward, and this module must not expect one. An
  earlier revision of this file asked for both a registered address with
  next-cycle data AND a same-cycle write forward; neither is what `vlrf`
  provides. The no-forward rule is sound because every VL wakeup-to-execute
  distance is at least one cycle (VecPipeline part 6), so a `pvl` this module
  reads was written no later than the previous cycle. If a fast or speculative
  VL wakeup is ever added, `VlRegFile` needs a write forward and VecPipeline
  part 9 is the paragraph that has to change — not this one.

  This module registers `vl_read_data` on ITS OWN side (`rr_vl`) purely to line
  VL up with the INT responses on `out`. That is a local convenience, permitted
  explicitly by part 9, and it is the reason an INT denial cannot leave `vl`
  and `base`/`stride` describing different instructions.

  ---- ⇒ THE STALE SCALAR BASE FORWARD. NOT AN OPTIMIZATION. ----

  //@req-spec-issue.d3
  The vector LS is woken SPECULATIVELY, so this INT read can fire in the same
  cycle the base GPR's writeback commits. BOOM's integer register file is a
  combinational `Mem` read of a REGISTERED ADDRESS with NO read-during-write
  bypass on this port, so:

    - a write presented in the cycle a lane FIRED is reflected in that lane's
      response-cycle data (it took effect at the intervening clock edge) — no
      forward needed;
    - a write presented in that lane's RESPONSE cycle is NOT reflected (it takes
      effect at the end of that cycle) — the read returns the STALE pre-write
      value and it MUST be forwarded.

  So, in each lane's response cycle, compare every valid `int_wb_snoop` port's
  `addr` against the STAGE-REGISTERED `rr_uop.prs1` and `rr_uop.prs2`
  independently and on a hit substitute that port's `data` for the corresponding
  `int_rf_read_rsp` entry before it is captured into `rr_data`.

  ===> A LANE WHOSE REGISTER TYPE IS `RT_ZERO` PRESENTS A LITERAL ZERO, and no
       amount of forwarding substitutes for it. Rename maps `x0` to p0, nothing
       ever writes p0, and the register file therefore returns that entry's
       leftover contents — BOOM's scalar register-read stage forces the zero
       itself, and this module is that stage for the vector LS. It matters on lane
       1: a `vlse`/`vsse` whose stride register is `x0` is architecturally a ZERO
       stride, the idiom for broadcasting one element, and would otherwise walk
       memory from whatever p0 last held. Key it on the type, which decode already
       resolves to `RT_ZERO` for an rs field of 0, and apply it to the PRESENTED
       value rather than to `rr_data`, so the forward and the wait logic stay
       written in terms of what the register file actually returned.

  ===> RETRACTED, AND IT MANDATED A BUG. An earlier revision of this paragraph
  ended: "The window is exactly that lane's response cycle: a cycle earlier is
  redundant, A CYCLE LATER IS TOO LATE." That sentence is WRONG and the RTL
  generated from it computed vector addresses from the previous tenant of the
  base physical register. It enumerated only two cases -- write at the FIRE
  cycle (already reflected) and write at the RESPONSE cycle (forwarded) -- and
  silently assumed no third. There is a third, and it is the common one:

    - a wakeup that arrives with `iw_p*_bypass_hint` SET fires one cycle BEFORE
      the producer's regfile write. The slot is granted at T, the read fires at
      T, the response arrives at T+1 -- and the producer's write is presented on
      `int_wb_snoop` at T+2. BOTH the read AND the response-cycle forward miss.
      "A cycle later" is not too late; a cycle later is exactly where the data
      is.

  Measured on `ms2p5_loadblock` (2000 ps clock) -- every number in this
  paragraph is from `regr_G_full/bringup_tests/ms2p5_loadblock/ms2p5_loadblock.fsdb`,
  so cite that file and not one of the other four failures: T=3729000 `iss_valid=1`,
  `prs1=0x33`, `iw_p1_bypass_hint=1`, read fires; T+1=3731000 response
  `0xa5d7d000` -- the stale prior tenant of p51 -- and the snoop carries addr
  `0x14`, so the forward MISSES; T+2=3733000 the snoop carries addr `0x33`,
  data `0x80002010`. Rename was CORRECT throughout: the iregfile word for the
  renamed PRN holds the right value at T+2. This is a bypass-window RAW hazard,
  not a stale architectural read. `ms4_vle64` at t=3735000 shows it in one
  cycle across two lanes: lane0 `addr=0x0080002010 is_vec=0` (the scalar `ld`,
  which HAS a bypass network) and lane1 `addr=0x00a5d7d000 is_vec=1` (this
  module, which does not).

  THE WINDOW, CORRECTLY STATED. A lane's forward window OPENS at that lane's
  response cycle and STAYS OPEN until the write it is waiting for has been
  observed. Concretely: arm a per-lane `rr_wb_wait` bit at the grant; while that
  bit is set do not capture the regfile response into `rr_data` and do not
  present `out.valid`; clear it, and capture `Mux1H(hits, int_wb_snoop.data)`
  into `rr_data`, on the cycle the awaited write appears on the snoop. Watch the
  snoop whether or not that lane's read has fired yet -- a lane denied at grant
  may see the write first, and both orders leave `rr_data` correct. A branch
  kill or flush clears the wait bits with the rest of the stage state.

  ---- WHAT ARMS THE WAIT: THE GRANT-CYCLE HINT ----

  Arm from `iss.bits.iw_p1_bypass_hint` / `iw_p2_bypass_hint`, latched per lane
  at the grant.

  WHY ONE WAIT-CYCLE IS ENOUGH, AND WHAT WOULD BREAK IT. The wait is armed from
  the grant-cycle hint, which is set for exactly one cycle. That is sufficient
  only because every bypassable integer producer presents its register-file write
  at the SAME distance from the hint -- measured `H+2` on `ms2p5_loadblock` for
  an ALU producer. **This bound is enforced, not observed.** For both producer
  classes the bypass network and the write port are driven from one node in one
  cycle -- the load from `RegNext(io.lsu.iresp)` (`core.scala:965` and `:969`),
  the ALU from `unit.io_alu_resp` (`core.scala:1058` and `:1050`) -- and
  `execution-unit.scala:136`/`:141` assert that any uop reaching RRD with
  `iw_p*_bypass_hint` set MUST hit the bypass. Hint implies bypass-available;
  bypass-available IS write-presented; therefore the distance is uniform.

  **The load wakeup is bypassable too** (`lsu.scala:1164` and `:1727`, both
  settings of `enableFastLoadUse`), so a load-produced base is covered by the
  same bound and needs no extra machinery. An earlier review believed only the
  1-cycle ALU was marked bypassable; that is FALSE, and the fix survives the
  correction only because of the uniformity above. Do not restate the narrower
  claim.

  ---- AND ON THE MISS PATH: THE WINDOW EXISTS, AND `int_squash_grant` CLOSES IT ----

  A load that misses retracts its speculative wakeup, and there IS a one-cycle
  exposure. At X+2 (= H+1) `slot_uop.prs1_busy` is still 0 -- the re-busy does
  not land until X+3 -- while the hint has already fallen to 0. A grant taken in
  that cycle arms nothing.

  What closes it is `int_squash_grant`: the retraction is broadcast on
  `iwakeups` in EXACTLY that cycle, and `VecIssueUnit.scala:403-406` kills the
  grant COMBINATIONALLY, same cycle. And it is not luck -- the hint falling and
  the retraction firing both derive from the same term, `w2.valid` at X+2, so
  they are locked together by construction, on ALL FIVE TIERS and not only on
  the one that sets `enableFastLoadUse`. (`int_squash_grant` reads `.bits.rebusy`
  without qualifying on `.valid`; that is X-safe because the arbiter's fallback
  carries a constant false and all four slow-wakeup sites drive
  `rebusy := false.B`.)

  ⇒ **SCOPE THE CLAIM PER CYCLE, AND DO NOT SAY EITHER TERM IS UNNECESSARY.**
  The re-busy does NOT close X+2 -- it lands at X+3, because
  `scalar_operands_ready` reads the REGISTERED `prs1_busy`. The combinational
  `int_squash_grant` closes X+2. The re-busy is nonetheless REQUIRED from X+3
  onward. **Neither term is redundant; deleting either reopens a different set of
  cycles:**

    | cycle       | what holds the line        | if you delete it                    |
    |-------------|----------------------------|-------------------------------------|
    | X+2         | `int_squash_grant` (comb.) | X+2 reopens -- the original bug     |
    | X+3 onward  | slot re-busy (`prs1_busy`) | the slot grants with hint=0 AND     |
    |             |                            | prs1_busy=0 while the miss is still |
    |             |                            | outstanding, and NOTHING catches it |

  An earlier revision of this paragraph said flatly that the re-busy explanation
  "is FALSE". RETRACTED as written: it is false ONLY for X+2, and stating it
  unscoped reads as "the re-busy is not needed" -- which would license deleting
  exactly the term that owns X+3 onward. Naming which cycles each term owns is
  what makes this paragraph unusable as a licence to delete either one.

  ⇒ Separately, and NOT in conflict with the above: `int_squash_grant`
  suppresses only SAME-CYCLE grants. A retraction arriving after this module has
  already been granted does not reach it, so an armed lane's wait can legitimately
  last a miss plus a refill. That is the post-grant case and it is what sizes
  `wbWaitWatchdog`; see that paragraph.

  **THE LOAD-BEARING ASSUMPTION, STATED SO IT CAN BE CHECKED:** every bypassable
  integer producer's regfile write lands no later than the second cycle after the
  hint. A producer that violates it would break `execution-unit.scala:136` in the
  scalar path first, which is the reason no sticky per-source wait bit is carried
  here. If that scalar assertion is ever weakened or removed, THIS paragraph is
  the dependent and the wait must become sticky -- set on the bypassable wakeup,
  cleared on the observed `int_wb_snoop` hit -- which requires routing the
  write-port addresses into every vector issue slot. `lateWritebackObserved` and
  `staleCaptureCheck` are the detectors that make the violation loud rather than
  silent in the meantime.

  **`bypass_hint` may be used as a "is the write pending?" predicate, never as a
  latency constant.**

  IF THE STICKY FORM IS EVER NEEDED, BUILD IT HERE, NOT IN THE SLOT. The sentence
  above says the sticky bit "requires routing the write-port addresses into every
  vector issue slot" -- that is the obvious construction and it is the WORSE one:
  it roughly doubles the comparator array each slot already pays for
  `int_prs1_matches`, and it re-opens the `pnrGate` scoping question by putting
  vector-only state in a module shared with `iq_v_alu`. There is a better place.
  `VecPipeline` is the ONLY scope that already holds BOTH the integer wakeup
  network (`io.int_wakeups`, which it fans to the three queues) and the writeback
  snoop (`io.int_wb_snoop`). Build one `Vec(numIntPhysRegs, Bool)` register there
  -- SET on a bypassable INT wakeup naming that PRN, CLEARED when that PRN's
  write is seen on the snoop, all bits cleared on `rob_flush` -- and fan it
  through a `VecLsu` port to `ld_opnd` and `st_opnd`, which then arm from
  `pending(prn)` instead of the hint. One array for the whole vector pipeline,
  no new comparators in any slot, no `pnrGate` interaction. This construction was
  built and elaborated cleanly before C1 was withdrawn; it is recorded so the
  next person does not re-derive the slot-based version.

  AND HERE IS ITS PRICE, MEASURED, SO THE TRADE CAN BE MADE WITH A NUMBER RATHER
  THAN RE-DERIVED: at `MegaBoomV4VectorConfig` the scoreboard is 144 bits of
  state in `VecPipeline` plus `io_int_wb_pending_0..143` on BOTH
  `VecScalarOperandRead` instances -- 288 wires of fanout. That was judged too
  much to spend defending a bound that already has an enforcing assertion one
  level down, and it is the whole reason the hint-armed form ships. If the
  enforcement goes away the number stops being an argument against it.

  DO NOT ARM FOR A WRITE THAT LANDS IN THE GRANT CYCLE. Such a write takes effect
  at the intervening clock edge and is already reflected in that lane's response
  (the first case above). Suppress arming on a grant-cycle `int_wb_snoop` hit
  against the granted PRN, or the lane waits for a second write to a physical
  register that only ever gets one.

  RE-QUALIFY BY SOURCE TYPE -- AND THIS IS A DEFECT MITIGATION, NOT
  BELT-AND-BRACES. AND the arming predicate with `lrs1_rtype`/`lrs2_rtype ===
  RT_FIX`. An operand that is genuinely `RT_ZERO` or absent takes no INT wakeup
  at all, so this never suppresses a wait that was needed.
  ⇒ THE REASON IT IS REQUIRED LIVES IN ANOTHER FILE, WHICH IS WHY IT READS AS
  OPTIONAL HERE AND WILL BE DELETED BY SOMEONE TIDYING. `VecIssueSlot` has a live
  OPEN DEFECT: its `prs2` wakeup arm (`:178`) and `iw_p2_speculative_child` set
  (`:180`) are UNGATED while the matching rebusy/clear (`:183`) IS gated on
  `lrs2_rtype === RT_FIX`. So on a form with no stride, lane 1's hint can be
  raised off a leftover rename number by an unrelated wakeup that nothing will
  ever retract. This qualification is the LOCAL MITIGATION for that defect and
  must stay until `VecIssueSlot:178`/`:180` are gated. See the open-defect entry
  in `VecIssueSlot.nlhdl`.

  KEEP THE ONE-CYCLE SNOOP FORWARD AT THE RESPONSE CYCLE. It is not made
  redundant by the wait and must not be "simplified" away: it is the mechanism
  the wait itself uses to observe the write, and it is what covers a write
  arriving at exactly the response cycle on an UNARMED lane.

  ASSERT THE CONTRACT: `bypassHintUnhonored`. Per lane, never present with
  `out.valid` while that lane's grant carried a hint and the producer's write
  has not been observed. Phrase it over a latched `rr_hint` and a latched
  `rr_wb_seen` rather than over the `rr_wb_wait` gate itself, so that any path
  which clears the gate WITHOUT the write having been seen still trips it.
  It fires at grant+1 in the module that erred; the bug it names surfaced
  instead as a TLMonitor A-channel `AcquireBlock` complaint two hierarchy
  levels away, about a diplomatic parameter, roughly ten cycles later. One
  assertion here is the difference between a five-minute fix and a campaign.

  ASSERT THE CALIBRATION ITSELF: `lateWritebackObserved`. The two PASSIVE cases
  -- a lane granted one cycle after the hint (covered by the response-cycle
  forward) and two cycles after it (covered by the register file) -- both rest on
  every bypassable INT producer sharing ONE wakeup-to-writeback distance, the
  invariant `execution-unit.scala:136`/`:141` assert in the scalar path. An FU
  added later, marked bypassable with a DIFFERENT calibration, breaks both
  silently, and `bypassHintUnhonored` cannot see it because `rr_hint` is already
  false by then. So watch for the tell: a snoop hit on the PRN a lane just
  CONSUMED from the register file means that producer's write is further out than
  the calibration assumes. Compare against a REGISTERED address, not `heldAddr`,
  so an intervening grant cannot alias the compare.

  ITS WINDOW RUNS FROM THE CONSUMING READ UNTIL THE OP LEAVES, AND THAT IS
  DELIBERATELY WIDER THAN `staleCaptureCheck`'s TWO CYCLES. The two bounds differ
  for a reason, so do not "harmonise" them:
    - this one terminates while the op is still LIVE, and a PRN cannot be freed
      and reallocated while a live consumer still holds it as a source -- so the
      no-benign-aliasing argument holds across the whole window and there is no
      false-positive path;
    - `staleCaptureCheck`'s can OUTLIVE the op, so past a couple of cycles it
      would compare against a reallocated register and fire falsely.
  A one-cycle peephole here would catch only a producer late by exactly one,
  which is useless against an FU whose calibration is by definition unknown --
  and one cycle late is very nearly the ONLY distance this assertion does not
  need to catch, because the response-cycle forward already covers a write at
  H+1. A peephole aims the tripwire at the one case that is already handled and
  misses every case it exists for.

  ⇒ **QUALIFY BOTH DETECTORS BY SOURCE TYPE, AND THE SOUNDNESS ARGUMENT IS WHY.**
  An earlier revision stated the no-aliasing bound as: "a write to that PRN after
  we read it can only be our own producer writing late, because the PRN cannot be
  freed and reallocated while a live consumer holds it as a source." RETRACTED AS
  WRITTEN -- it is FALSE for an operand the consumer DOES NOT USE. A unit-stride
  access uses `prs1` only; lane 1's `prs2` is a leftover rename number that the op
  does not hold, so it CAN be freed and reallocated while the op is live, and an
  unrelated instruction writes it legitimately.
  This was not hypothetical: `lateWritebackObserved` fired on lane 1 of
  `ms11a2_pure_vle`, `ms11a3_vle_lmul2` and `ms2_vse64` -- all unit-stride, all
  previously PASSING, all correct behaviour.
  CORRECT STATEMENT: the PRN cannot be freed while a live consumer holds it as A
  SOURCE IT ACTUALLY USES. Hence both `lateWritebackObserved` and
  `staleCaptureCheck` are qualified by the HELD `lrs*_rtype === RT_FIX` -- the
  held form, not the grant-cycle one, because the check spans the op's life. That
  qualifier is not noise-filtering; it is the precondition the soundness argument
  requires, and removing it re-breaks the property.
  `staleCaptureCheck` had the IDENTICAL hole and is fixed in the same change even
  though it never fired: it also compares against `rr_uop.prs*`, and its window is
  merely smaller (two cycles versus the op's life), so it is the same defect with
  a smaller target. Fixing only the one that fired would have left a latent false
  positive to surface on an unrelated future run.

  WHY THE QUALIFIED WINDOW IS SAFE, DERIVED RATHER THAN ASSERTED. A physical
  register is freed only when the NEXT WRITER of the same architectural register
  COMMITS, and that instruction is younger than our consumer -- which has not even
  executed yet. So the span in which the PRN cannot be reallocated runs to our
  consumer's COMMIT, and it STRICTLY CONTAINS this assertion's window. That is
  margin, not a tight fit.

  ⇒ **`killedNow` IN THE `consumedValid` CLEAR IS LOAD-BEARING, NOT HYGIENE.**
  There is exactly ONE way to leave the safe span above, and it is a squash: on
  a flush, rollback returns the register to the free list and it can be
  reallocated on the correct path, at which point a still-held `consumedValid`
  would fire on a perfectly legitimate write to the NEW tenant. `killedNow` is
  what keeps the assertion sound. It is called out here because "cleared on
  kill" reads like boilerplate and is precisely the term a future simplifier
  deletes.

  WAIT ON THE EVENT, NOT ON A COUNT. The obvious alternative -- age the wakeup
  by a constant `intAluWbDelay` -- is REJECTED: it bakes the integer ALU's
  writeback depth into the vector path, where it cannot be checked and can be
  mis-sized silently by any change to the INT pipeline. The event is observable
  on a port this module already has.

  THE HOLD CREATES ONE NEW STATE, AND THE GRANT GUARD MUST BE WIDENED TO SEE IT.
  The pre-existing "grant landed on a still-unfired hold" assertion tests
  `rr_need` ONLY. `rr_need` goes to 0 as soon as the read fires, so once only the
  writeback wait is outstanding that guard sees nothing -- and the `iss.valid`
  branch would then overwrite `rr_uop`, `rr_wb_wait`, `rr_hint` and `rr_wb_seen`,
  silently discarding the wait and re-arming against a different op. Widen it to
  `rr_need(0) || rr_need(1) || rr_wb_wait(0) || rr_wb_wait(1)`.
  This is reachable BECAUSE of the hold, which is why it is asserted rather than
  argued: on a grant at X+1 for a load that then misses, the wait arms and holds
  correctly, but `VecIssueSlot.scala:389-390`/`:409-410` keep the slot valid and
  clear `iw_issued` on the rebusy, so the refill's slow wakeup can grant THE SAME
  OP a second time while this stage is still waiting from the first grant. Static
  reading says the outcome is probably correct; the margin is one cycle and the
  reading is inferred, not measured, and this state did not exist before the
  hold. An earlier note advised against extending the guard to `rr_wb_wait` on
  the grounds that the squash-and-regrant path is legitimate; that advice is
  SUPERSEDED -- the second-grant path above is a different one and is not
  obviously legitimate. If the widened guard fires on a benign case, that is a
  cheap thing to learn from one regression instead of arguing about.

  IT COSTS A CYCLE, AND ONLY ON THE HINTED PATH. An unhinted wakeup is already
  written back, `rr_wb_wait` is never set, and the timing is bit-identical to
  before. Do not "optimise" the hinted path back to the response cycle.

  THE FAILURE MODE INVERTS, WHICH IS THE POINT. Because the hold waits on an
  event, a producer write that never reaches a snoop port now HANGS instead of
  quietly presenting garbage. `wbWaitWatchdog` (default 4096, `0` disables and
  emits no register, following `VecCiiFlush.drainWatchdog`) names that failure
  with the lane, the held PRNs and the `rob_idx`.

  IT IS A BACKSTOP, NOT A LOAD-BEARING GUARANTEE -- AND ITS NUMBER IS STILL NOT A
  PIPELINE DEPTH. An earlier revision of this paragraph called the watchdog
  load-bearing, on the theory that an awaited write might never arrive. RETRACTED:
  it cannot. The only way to lose the write is a producer squashed while its
  consumer survives, and that cannot happen here -- the consumer is younger and
  carries a same-or-superset `br_mask`, so any kill that takes the producer also
  takes us, and `killedNow` clears the wait. An OVERSTATED rationale is the same
  defect class as an understated one, in the other direction; hence this
  correction.
  What remains true, and is what actually sizes the number: `int_squash_grant`
  (`core.scala:1441-1444`, which ORs `io.lsu.iwakeups.map(_.bits.rebusy)`) reaches
  `VecIssueUnit` and kills `iss_uops.valid` in the SAME cycle as the retraction --
  so it protects a grant colliding with a rebusy, and NOTHING MORE. A rebusy
  arriving after this module has already been granted does not reach it:
  `ld_opnd`/`st_opnd` have no kill input beyond `brupdate` and `rob_flush`. On a
  mis-speculated base the wait therefore legitimately lasts a D$ miss plus refill.
  Size for that, not for the ALU's two cycles, and do not tighten it.
  (The silver lining stands: under this form such an op WAITS for the real write
  instead of computing addresses from a mis-speculated base, which is what it did
  before.)

  ASSERT IT INDEPENDENTLY TOO: `staleCaptureCheck`. Every other property here
  trusts the readiness logic. This one does not: it compares the value PRESENTED
  against the value the integer register file turned out to hold -- a write to a
  presented lane's PRN landing in the presentation cycle or the next one with
  different data means the capture was stale, whatever mechanism was supposed to
  prevent it. Bound it at two cycles: beyond that a PRN can have been freed and
  reallocated and the comparison stops meaning anything.
  IT IS PARTIAL BY CONSTRUCTION, AND ITS SILENCE PROVES NOTHING. Because it
  compares a late `int_wb_snoop` hit on `rr_uop.prs*` against a value already
  presented, it only fires while the operand is still held. A SHORT overshoot
  trips it; a LONG one may not. Do not read its silence as evidence that a
  writeback arrived on time -- that is what `lateWritebackObserved` is for.

  BOUNDED AT TWO CYCLES, AND IT MUST STAY BOUNDED. The partiality above reads
  like a weakness to be fixed, and the obvious "improvement" -- widen the window,
  or drop the bound entirely and compare against every later write to that PRN --
  IS WRONG AND WILL FIRE FALSELY. Beyond a couple of cycles the physical register
  can have been freed and reallocated, at which point the write being compared
  belongs to a DIFFERENT instruction and disagreeing with our value is correct
  behaviour, not a defect. Bounded partial detection is the honest form of this
  property. Widen the coverage with `lateWritebackObserved`, which keys off the
  PRN a lane just consumed and is sound for exactly the reason this one is not:
  a PRN cannot be freed and reallocated while a live consumer still holds it as
  a source.

  THE HOLD IS SAFE AT THE VecLsu SEAM. `VecLsu`'s `ldAwaitingPulse` and
  `stAwaitingPulse` are LEVEL handshakes -- a register set by `ldPresent`/
  `stPresent`, cleared on `out.valid` -- and `ldStreamerBusy`/`stStreamerBusy`
  block the next `present` until `msk.io.done`, which is itself launched from
  the op pulse. Arbitrary extra hold cycles were already tolerated (the existing
  `rr_need` hold path relies on the same property), and a register cannot
  false-pulse on a sticky `out.valid` left over from the previous op.

  THE WINDOW IS PER LANE, NOT PER GRANT, AND D5 IS WHY. With `Decoupled` reads
  the two lanes may fire in different cycles, so "the response cycle" is no
  longer one cycle for the whole grant: lane `n`'s forward window is the cycle
  after `int_rf_read_req(n).fire`, and it must be evaluated then and only then.
  A single grant-relative window — the natural way to write this before D5 —
  would miss the forward on any lane that was denied even once, which is the
  stale-base bug reappearing exactly where it is hardest to see. Held cycles
  need NO forward: the read has not happened yet, so there is nothing stale to
  repair, and the address being held is a physical register number that cannot
  change under the hold.

  BUG (3) FROM THE M1 BRING-UP LOG — DO NOT RE-INTRODUCE. A CORRECTNESS
  REQUIREMENT, not a performance feature: without it a vector load or store
  whose base GPR was produced one instruction earlier computes every address
  from a stale base. It is masked in almost every test by the two or more
  instructions of `la`->use slack that compilers and hand-written tests
  naturally emit, so it will NOT surface casually — its absence must be
  positively demonstrated (the forward-hit trace line below, in a directed
  back-to-back test), never assumed.
  THAT DEMAND WAS RIGHT AND WAS NOT MET. The directed back-to-back tests exist:
  `ms3p5_pureload`, `ms4_vle64` and `edge_ls_gen` each overwrite the base GPR
  immediately before the vector access. All three FAILED — an AcquireBlock to a
  non-cacheable address, because the address was the previous tenant of the
  base PRN — and `ms2p5_loadblock` and `stress_seg` failed downstream of the
  same read. The single-cycle window this paragraph specified was demonstrated
  INSUFFICIENT, not demonstrated present. Any future narrowing of the window
  must re-run those five.
  If BoomCore's read port ever becomes a fully registered SyncReadMem read,
  or gains a stage between request and response, THIS WINDOW MOVES WITH IT.

  Two writeback ports never target the same physical register in one cycle, so
  the substitution is a `Mux1H` over the hit vector, not a priority mux; assert
  `PopCount(hits) <= 1` per read rather than relying on it silently.

  ---- The scalar source operands are INTEGER ONLY (D4) ----

  //@req-spec-agen.d16
  This module READS SCALAR SOURCE OPERANDS FROM THE INT REGISTER FILE, and only
  from there: `prs1` (the base address) on lane 0 and `prs2` (the stride) on lane
  1, for every form on either direction. The vector register file serves the
  VECTOR source operands (`st_vdgen` reads store data on VRF port `R3`, VecDgen's
  port, untouched here), and no register file other than those two and the VL RF is
  read on behalf of a vector memory op.

  On the store instance, if an `FC_DGEN` grant's data source is ever scalar-typed,
  the register-type field on the granted uop — set by VecDecode, so nothing here
  re-decodes the instruction — selects the INT file and the value is taken from lane
  1's response, which is idle on a DGEN grant because a DGEN grant reads no stride.
  That result is presented on `out.bits.scalar_data`.

  `scalar_data` IS A RETAINED FIELD, NOT A RETAINED CLAIM. `execution.rst` as
  amended says the DGEN reads its store data from the VECTOR register file only,
  because RVV store data is always `vs3` — so on every store form defined today
  the `RT_FIX` mux above never selects and `scalar_data` is don't-care. It is kept
  because `VecLsu`'s descriptor row lists it and because the mux is a select on a
  value lane 1 already read (no port, no read, no cost), NOT because a store with
  an integer data operand is expected to appear. If one never does, the field and
  the mux are the right thing to delete together, in `VecBundles` and here.

  ===> THE FP LEG OF `spec-agen.d3` IS UNREACHABLE AND IS DELETED (D4). The
  requirement was extracted from `execution.rst`'s "reads the FP/INT register
  file for scalar source operands, for instance `vfmul.vf`", and the example is
  simply the wrong instruction class: `vfmul.vf` is vector-scalar FP
  ARITHMETIC, dispatched to the CII and served by `VecCiiIssue`, not a store.
  Enumerating the RVV store forms settles it — `vse<eew>.v`, `vsse<eew>.v`,
  `vsuxei`/`vsoxei`, `vs<nf>r.v` and their segmented variants — store data is
  always the vector register `vs3`, and the only scalar operands are `rs1`
  (base) and `rs2` (stride), both integer. There is no encoding in which a
  store names an FP scalar register, so an FP reader on this module could never
  fire. `spec-agen.d3` keeps its ID and its INT content (D12 case 1 amends the
  `.rst` text); `execution.rst`'s FP clause is the falsehood, not this file.

  ---- Why the segmented-store chain cannot deadlock through this module ----

  //@req-spec-issue.d3
  The LSU half of a segmented store sources every operand of its address path
  from instructions OLDER than the store itself: the base GPR read here through
  `rr_uop.prs1`, plus the index vector and mask read by `idx` (VecIdxGen) and
  `msk` (VecMaskStream) inside this direction's `st_elem_agen` — all named by
  fields rename resolved before the store was dispatched. This module waits on
  nothing produced BY the store, nothing produced by its coprocessor half, and
  never consults `pvtmp`. That makes step 1 of the six-step segmented-store chain
  (issue.rst `shared-store-chain`) unblockable and the chain a long serial
  dependency rather than a cycle: step 1 clears `rob_unsafe`, the PNR advances,
  the coprocessor half becomes eligible and writes `pvtmp`, and only then does
  step 6 — VecDgen reading `pvtmp` on a separate, later `FC_DGEN` grant — run.

  The mask and index VRF reads are NOT performed here, though both are AGEN
  operands. `vrf-ports` partitions those ports statically (R0/R1 load, R4
  store) and never arbitrates them, so each must have exactly ONE reader, and
  that reader is the agen's VecMaskStream/VecIdxGen, which needs them at
  element cadence rather than once per instruction. This module passes their
  PRNs on the wrapped uop and reads neither.

  ---- VL = 0 is a normal result, not an error ----

  //@req-spec-lsu.m1
  VL is known at decode only for the immediate-AVL form (`vsetivli`), where a
  dependent of a zero AVL can be squashed in the front end. For EVERY other form
  VL is renamed into the VL register file, so a `VL = 0` op reaches issue, is
  granted, and arrives here with `pvl` resolving to 0. This module treats that as
  an ordinary read result: `out.valid` asserts exactly as for any other VL,
  `out.bits.vl` is 0 and `out.bits.vl_zero` is 1. No exception, nothing dropped,
  no stall.

  `vl_zero` is an explicit bit rather than a comparison left to each consumer,
  because the consumers are asymmetric: the agens must emit no element access at
  all, while VecGroupCopy must instead perform the `pvdest <- stale_pvdest` group
  copy that makes the freshly renamed destination group architecturally correct so
  the entry can commit (case_study.rst `case-vl-zero`; VecGroupCopy owns that
  behaviour). A consumer deriving the condition itself could derive it differently.

  ---- Kill, tracing, assertions ----

  A grant captured in the stage register is killed when
  `IsKilledByBranch(brupdate, rob_flush, rr_uop)` holds, suppressing `out.valid`
  that cycle AND clearing `rr_valid`/`rr_need` so a killed grant stops re-presenting
  its held read — a held request for a squashed uop would keep taking an INT
  arbitration slot from live readers for as long as the denial lasted. `rr_uop.br_mask`
  is updated by `GetNewBrMask` as usual. Because the module holds nothing else, kill
  is complete — no leftover state to flush, no counter to rewind, no half-issued
  instruction to retract.

  Two guarded `VecTrace` lines, gated on the `vecTrace` plusarg and `!reset` and
  adding no logic any functional path reads: one per grant, tagged with the
  instance's direction and carrying `rob_idx`, `pvl`, the resolved `vl`, `base`,
  `stride`, `v_eew` and the number of cycles the INT read was HELD (`traceVl`
  supplies the VL fields); and one whenever the writeback FORWARD FIRES, naming the
  read that hit and the substituted value. The second is the evidence that plan §2
  bug (3) is absent — with no unit tests in this project, a directed back-to-back
  `la`/`vle` run plus that line IS the demonstration. The held-cycle count on the
  first is the evidence for D5's premise that INT denial is routine rather than
  theoretical, and it is the number to look at if P2/P3 throughput comes in low.

  Assert that a granted uop has `is_vec` set and belongs to this direction
  (`uses_stq` when `isStore`, `uses_ldq` otherwise); that at most one writeback port
  hits per read; that a lane's `valid` never drops while `rr_need(n)` is set and its
  held `bits` never change between assertion and `fire` (the `Decoupled` producer
  obligation, checked rather than assumed); and `!(iss.valid && rr_valid &&
  rr_need.orR)` per the grant-cycle note above. All are synthesizable `assert`s;
  this module contains no simulation-only construct anywhere.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput: ONE grant per cycle, sustained, per direction — not one per
instruction completion — WHENEVER THE INT FILE GRANTS BOTH LANES. That is the
design target and the reason there is no FSM here and no shared INT port. The one
thing that can slow it is INT read arbitration (D5): a denied lane repeats while
the lanes that fired stay fired, so throughput degrades by the unluckiest lane's
denial count and never stalls outright. A grant arriving while a lane is still
outstanding cannot happen by construction — `VecLsu`'s descriptor table drops
`valid` once it has handed this module a row and re-presents only after the hold
clears — and the assertion in the logic section is the check on that, not a
deferred open item.

Latency: ONE cycle from grant to `out.valid` in the uncontended case, plus one
cycle per cycle the INT file denied a lane. It is no longer data-independent, and
the change is upstream only: the VL RF answers combinationally in the grant cycle
and cannot deny, and the downstream `Valid` is still a single-cycle unhandshaked
pulse the agen latches.

Depth: one stage — one `MicroOp` register, one valid bit, two `xLen` response
registers, two need bits and one `vecVLSz` VL register — times two instances. The
area cost of de-sharing is that second set plus a second pair of INT read ports,
and it buys the removal of an arbiter that was silently dropping store grants. The
FP read port an earlier revision had on the store instance is GONE (D4), so the
store instance is now no more ported than the load instance.

Critical path to watch: the per-lane response-cycle writeback forward. It is a
`numIrfWritePorts`-wide address compare plus a `Mux1H` on `xLen` bits, in series
with the register-file read output, feeding the agen's address adder. If that
path fails timing, the fix is NOT to delete the forward (that reintroduces bug
(3)) and NOT to add a pipeline stage that would move the comparison window
(see the logic section) — it is to narrow the compare by pre-decoding the
writeback addresses one cycle earlier, in the cycle the lane fires, when the read
addresses are already known and held.

Second path to watch, new with D5: `int_rf_read_req(n).ready` is a `PopCount` over
every earlier logical reader's `valid` inside `PartiallyPortedRF`, and it now
feeds this module's `rr_need` update. It is the register file's own existing path,
lengthened by no logic here — but the vector lanes sit at the END of the priority
chain, so they see the deepest `PopCount`. If it fails timing the answer is a
register-file-side one (a registered grant, or fewer logical readers), not a
protocol change here.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the grant payload and the wrapped uop on the result. Reads `prs1`,
`prs2`, `pvl`, `fu_code`, the `*_rtype` register-type fields, `is_vec`,
`uses_ldq`/`uses_stq`, `br_mask` and `rob_idx`; carries `pvm`, `pvs2`, `v_eew`
and `v_emul` through untouched for the agens.

VecBundles — the home for the `VecScalarOperands` result bundle (see the
bundle-location note in the ports section) and the transitive route to
VectorParams for `vlPregSz` and `vecVLSz`: the hierarchy entry lists no direct
VectorParams dependency, so those widths arrive through VecBundles.

VecTrace — the two guarded trace lines.

Binds to BOOM's existing `BrUpdateInfo`, `IsKilledByBranch` and `GetNewBrMask` in
`boom.v4.common`, and to `HasBoomCoreParameters` for `xLen`, `maxPregSz` and
`numIrfWritePorts`. No new squash or wakeup mechanism is introduced.

Instantiates nothing. Instantiated by VecLsu, twice, as `ld_opnd` and `st_opnd`.
Its consumers are
VecElemAgen, VecRangeAgen, VecDgen and (through the `vl_zero` bit) VecGroupCopy
for its own direction only — reached through `VecLsu`'s descriptor pending table,
which is where a produced descriptor waits; its producers are that direction's
VecIssueUnit instance, BoomCore's INT register file read ports (`Decoupled`, two
lanes per instance) and writeback bus, and VlRegFile (combinational read). It has
NO dependency on FpPipeline: the FP read port that delta adds belongs to
`VecCiiIssue` (D4), not to this module.
<|end_dependencies|>

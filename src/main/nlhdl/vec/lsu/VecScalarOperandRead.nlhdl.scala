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
  `int_rf_read_rsp` entry before it is captured into `rr_data`. The window is
  exactly that lane's response cycle: a cycle earlier is redundant, a cycle later
  is too late.

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

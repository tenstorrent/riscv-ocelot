# Caracal Step 11a.2 — Vector unit-stride LDQ/STQ multi-beat mechanism (frozen design)

Resolves how a vector unit-stride load/store `OP.v` (holding ONE LDQ/STQ entry
reserved at dispatch) performs N sequential 64b dcache accesses through BOOM v4's
existing scalar LSU, with ZERO scalar perturbation. Implementation pending.

## Recommended mechanism: direct dcache-request path + entry-as-placeholder

Reject "re-arm `ldq_executed`/`ldq_succeeded` per chunk" (Mechanism 1) — those
per-entry bits feed scalar invariants/asserts (lsu.scala:898,1547,1762) and the
nack/replay logic; re-arming races a late nack and is hard to prove bit-identical.

Instead (Mechanism 2 + Mechanism 3's single-range view for disambiguation):
- The new **VecLSU** block owns the beat loop + chunk accounting. It presents ONE
  beat at a time (serial, single-outstanding) on a NEW `io.core.vec_dmem.req` port.
- lsu.scala forwards that beat into the existing `dmem_req` mux via an **appended,
  lowest-priority** `lsu_sched` term (`will_fire_vec_load`/`will_fire_vec_store`).
  Appended-last ⇒ strict top-down priority ⇒ no scalar `will_fire` changes
  (compile-time scalar floor; vector fills idle dcache cycles only).
- The LDQ/STQ entry is an ordering/disambiguation PLACEHOLDER. Its `executed`/
  `succeeded` (loads) are written **once, monotonically**, on a VecLSU `ld_done`
  pulse after the last beat — never re-armed. Stores: STQ entry = commit/clr_bsy
  placeholder; VecLSU drains beats once `stq_committed` (RVWMO: stores hit D$ only
  after commit) and emits a single vector `clr_bsy` (contract D5) + group-done.

## Serial model ⇒ no per-response tag needed (confirmed)
VecLSU issues beat k, waits for its response, then issues k+1. Its **current-chunk
register** (`cur_prn`, `cur_byte_off`, `cur_ldq/stq_idx`) uniquely identifies any
in-flight response. Route responses to VecLSU by `resp.uop.is_vec`; the VecLSU's
register supplies (PRN, byte-offset) placement into the VecLoadCoalescingBuffer.
On the last chunk of the last member, emit ONE `VecGroupDone{prn=pvdest_grp, mask}`
→ rob.vec_clr_bsy + vec_rename wakeup + vec issue vec_wakeup (all already accept it).

## CORRECTION to the study (TLB)
The study said vector beats are "pre-translated physical, `uses_tlb=false`". WRONG:
the Packer computes a VIRTUAL EA (base+offset); it still needs DTLB translation. So
M1 vector beats MUST use the TLB (`uses_tlb=true`), still appended-last priority
(minor TLB-arb perturbation, still scalar-floor). Unit-stride can cross pages, so
per-beat translation is required for correctness. Re-evaluate whether to translate
once-per-beat or once-per-page-run as an optimization later.

## Exact lsu.scala touch-points (all usingRVV-gated, identity when off)
1. New `vec_dmem` sub-bundle on `LSUCoreIO` (~108): req(Decoupled) / resp(Valid) /
   nack(Valid) / store_ack(Valid) / ld_done(Valid ldqIdx) / st_clr_bsy(Valid robIdx).
   Make it `Option` so the IO port list is byte-identical when `!usingRVV`.
2. Appended `can_fire_vec_load`/`will_fire_vec_load` (+store) after the last
   `lsu_sched` call (~679); `can_fire := usingRVV.B && (w==lsuWidth-1).B &&
   vec_dmem.req.valid && <scalar-floor auto via priority>`.
3. `dmem_req` mux (~958): `.elsewhen (will_fire_vec_load||will_fire_vec_store)` →
   addr/data/uop from `vec_dmem.req`, beat uop `dst_rtype=RT_VEC`/`is_vec`.
4. resp tap (~1564): wrap scalar load-resp `when` with `&& !(usingRVV.B &&
   resp.uop.is_vec)`; sibling `when(is_vec)` → `vec_dmem.resp`. (Keeps vec data out
   of iresp/fresp and out of `ldq_succeeded`.)
5. nack tap (~1546) and store_ack tap (~1613): `is_vec` discrimination → vec_dmem.*.
6. completion: gated `when(vec_dmem.ld_done.valid){ ldq_executed:=true;
   ldq_will_succeed:=true }` (once); route `vec_dmem.st_clr_bsy` to clr_bsy port.
7. picker/can_fire guards: add `&& !(usingRVV.B && is_vec)` to load-wakeup predicate
   (501/618), store-commit can_fire (609), and scalar clr_bsy generator (1089).

## core.scala
- Scalar `agen`/`dgen` drive (~300-310) UNCHANGED — vector beats go via vec_dmem.req,
  store data rides in vec_dmem.req.bits.data (not io.lsu.dgen).
- Instantiate VecLSU (+ VecLoadCoalescingBuffer); un-tie fu_types so V-LOAD/V-STORE
  grant (~1389); wire vec_dmem handshake; route group-done → vec_clr_bsy(827)/
  vec_rename wakeup(796)/vec issue vec_wakeup(1395); VecDgen start/vrf_read; AGEN
  load_nop/store_nop .ready (1469/1477) → VecLSU.
- VecLsDecode.io.in (1458): base/stride from 2 new usingRVV IRF read ports, vl from
  VlRegFile read, vstart from CSR.

## Decomposition for implementation
- VecLoadCoalescingBuffer + VecLSU beat FSM (NEW, vec/lsu/) — the bulk; no scalar touch.
- lsu.scala: the 7 gated edits above — LEAD, careful, scalar 9/9 is the safety net.
- core.scala: un-tie + wire — LEAD.
- New vle/vse e2e test (like vset_test) → gate e2 vs Whisper.
Gate: scalar 9/9 still green + vle/vse e2e + MediumBoomV4Config bit-identical.

## Risk note
High confidence scalar stays green: appended-last priority (compile-time), vector
beats don't touch scalar agen/dgen, per-entry bits written once. Watch: TLB-arb
(now used), LCAM range registration for disambiguation vs younger scalar stores,
and the Option-IO must elaborate to an identical port list when off.

## IMPLEMENTATION STATUS (load path landed; functional vle pending operand-read)

Decomposed into two gated sub-steps. **Sub-step A is DONE and ELABORATES/builds**
(MediumBoomV4VectorConfig simv built clean, no comb cycle). Sub-step B (the
operand read + issue un-tie + e2e test) is the remaining work for a functional vle.

### Key design CORRECTIONS vs the frozen doc above (discovered during impl)
- **Reuse the scalar LDQ/STQ (frozen doc was RIGHT here).** A vector load occupies
  ONE entry in the existing scalar LDQ, a store one STQ entry -- the architectural
  instruction's ordering + commit placeholder. There is NO vector-dedicated
  LDQ/STQ; the cracked 64b beats are VecLSU's separate beat queue and never take
  LDQ/STQ slots. (An earlier impl pass mis-read the Step-2 `uses_ldq := false` line
  as a constraint and routed completion purely through the ROB busy clear -- that
  was WRONG and is reverted.) Fix: VDecode now sets `uses_ldq := ls.is_load` /
  `uses_stq := ls.is_store` for vector mem ops. Completion on the last beat is DUAL:
    * `ld_done` (ldq_idx) -> LSU writes the placeholder LDQ entry's
      executed/succeeded ONCE (never re-armed) -> retires at the ROB head.
    * `clr_rob` (rob_idx) -> `rob.io.vec_clr_bsy(0)` clears rob_bsy (a RT_VEC load
      has NO iresp writeback to clear it) + `VecGroupDone` (prn) -> vec-rename
      wakeup(0) + vec-issue vec_wakeup(0) clear the dest-group busy bits.
  `VecDmemIO` = {req, resp, nack, ld_done}. The beat uop sent to the dcache keeps
  `uses_ldq=false` (it is a nano-op, not an LDQ op; resp routing keys on is_vec).
  The placeholder LDQ entry's `addr.valid` is never set (the vle never goes through
  scalar agen), so it stays invisible to the scalar wakeup/disambiguation pickers
  (which all require addr.valid) -- full LCAM address registration for RVWMO
  disambiguation is post-M1.
- **M1 bare-mode: vec beat EA is PHYSICAL, no TLB.** riscv-tests are identity-
  mapped, so `can_fire_vec_load` uses DC only (uses_tlb=false, uses_lcam=false),
  appended LAST in the `lsu_sched` chain. (The doc's "MUST use TLB" correction is
  a post-M1 refinement for page-crossing/paged mode.)
- **Beat == one 64b VRF lane.** DMEM_WIDTH=64b, so each VecLoadNop beat carries
  <=8 valid bytes = exactly one 64b dest lane (aligned base). The LCB writes one
  VRF lane per beat (no 256b accumulator); placement = `el_id<<eew` (dst byte),
  `el_off` (src byte off within the beat), `el_count<<eew` (nbytes) -> per-lane
  byte->lane mask. Partial-lane tail / straddling misaligned base = post-M1.
- **comb-cycle gotcha (hit + fixed):** `group_done` MUST derive only from the
  registered `cur` beat, never combinationally from `load_nop` -- else
  issue.vec_wakeup -> iss_uops -> decode -> AGEN -> load_nop -> group_done loops.
  Fake/bypass beats finalize one cycle later in a dedicated `sFin` state.

### Sub-step A -- LANDED (elaborates + builds; bit-identical pending verify)
- NEW `v4/vec/lsu/VecLSU.scala`: `VecDmemReq`/`VecDmemResp`/`VecDmemIO` bundles +
  `VecLSU` serial beat FSM (sIdle/sFin/sReq/sResp). Instantiates the LCB.
- NEW `v4/vec/lsu/VecLoadCoalescingBuffer.scala`: per-beat lane write + group_done.
- `lsu/lsu.scala` gated edits (all `if (usingRVV)` / Option -> identity when off):
  LSUCoreIO.vec_dmem Option bundle; will_fire/can_fire_vec_load Options; appended
  `lsu_sched` vec term; dmem_req vec branch; resp routing (`resp_is_vec` guard +
  vec_dmem.resp drive); nack routing (`nack_is_vec` guard + vec_dmem.nack);
  vec_dmem.req.ready drive; `ld_done` -> ldq_executed/ldq_will_succeed write.
- `vec/decode/VDecode.scala`: vector mem uops carry `fu_code(FC_AGEN)` (so the
  V-LOAD/V-STORE issue units match them) AND `uses_ldq := is_load` /
  `uses_stq := is_store` (occupy a scalar LDQ/STQ entry).
- `exu/core.scala`: top-level `vec_lsu` Option; group_done -> vrs.wakeups(0) +
  vec issue vec_wakeup_ports(0); clr_rob -> rob.vec_clr_bsy(0); AGEN load_nop ->
  VecLSU.load_nop; VecLSU.dmem <> io.lsu.vec_dmem; VecLSU.vrf_write -> vec_regfile
  write port 0.

### Sub-step B -- REMAINING for a functional vle (next session)
1. **Operand read** (the blocker): VecLsDecode.io.in feeds are still STUBBED
   (rs1_data/rs2_data/vl/vstart = 0 in core.scala ~1473). Need: rs1 base from a
   NEW usingRVV-gated integer-regfile read port (bump `numIrfLogicalReadPorts`,
   wire arb_read_reqs/rrd_read_resps -- 2-stage banked read w/ ready), vl from a
   VlRegFile read (pvl), vstart from CSR. Drive off the granted vload iss_uop.
2. **Un-tie the V-LOAD grant**: advertise `vload_iss_unit.io.fu_types(0)(FC_AGEN)`
   gated on `!vec_lsu.busy` (REGISTERED -> no comb loop), so a vle grants only
   when VecLSU is idle (iss_uops is Valid/fire-and-forget -> must not drop).
3. **e2e vle test** (like the vset smoke) -> gate e2 vs Whisper.
4. Re-verify: scalar 9/9 + MediumBoomV4Config bit-identical.
Then Step 11a.2-store (vse): vec_dmem.store_ack + st_clr_bsy, STQ-commit drain,
VecDgen VRF read, the store-side picker guards (lsu.scala 609/1089).

## IMPLEMENTATION STATUS -- store path LANDED (PASSES cosim)

The vse (OoO unit-stride vector store) path is now functional and validated end
to end against Whisper. What landed on top of the load path:

- **VecDgen (store-data generator) un-stubbed.** It reads each `pvs3_grp` member
  from a dedicated VecRegFile read port (`read_ports(1)`) and presents 64b slices
  to the store AGEN. The VRF read is REGISTERED (data = vrf[RegNext(addr)]), so a
  capture state (`sCap`) was added between `sFetch` (drive addr) and `sStream`
  (consume `buf`): reading `resp_data` in the same cycle as driving the address
  returned the PREVIOUS member's stale data and corrupted every store beat. Start
  fires from decode when `!is_load`.
- **VecLSU store beats.** Added `sSReq`/`sSAck` states: drive a `M_XWR` beat
  (addr & ~7, data from the store_nop) onto vec_dmem, wait for `store_ack`
  (retry on `nack`), advance. On the LAST beat: `clr_rob(rob_idx)` + `st_done(stq_idx)`.
- **STQ placeholder retirement.** The vse holds ONE entry in the scalar STQ
  (`uses_stq`). Per-beat acks are routed to VecLSU (`vec_dmem.store_ack`, gated by
  `sack_is_vec` so the scalar handler skips them); the STQ entry is marked
  `succeeded` ONCE via `st_done` so it retires (`committed && succeeded`) at the
  STQ head. Its `addr/data` are never set, so the scalar store-commit drain
  self-skips it.
- **Issue un-tie for stores.** `VecIssueSlot` gates a store request on
  `pvs3_ready` (`st_data_ok = !uses_stq || pvs3_ready`) so the store does not
  issue before its data operand group is renamed/ready. `vec_ls_rr` is shared by
  the V-LOAD and V-STORE issue units (load priority in the Mux);
  `vstore_iss_unit.fu_types(0)(FC_AGEN) := vec_ls_rr.fu_ready`.

Gate (all green): vse e2e (correct store data, self-check `gp=1`, 0 cosim
mismatches), vle regression (clean tohost finish), vset smoke 16/16,
MediumBoomV4Config bit-identical (0 non-cosmetic SV diffs -- the only
scalar-reachable change, `store_ack && !sack_is_vec` with `sack_is_vec=false.B`,
folds to the original gate).

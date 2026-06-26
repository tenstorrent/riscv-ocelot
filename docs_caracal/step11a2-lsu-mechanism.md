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

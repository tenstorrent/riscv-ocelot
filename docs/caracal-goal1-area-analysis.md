# Caracal Goal 1 — Area & Complexity Analysis

**Status.** Deferred-optimization tracking doc. Goal 1's mandate is **functional correctness first**; nothing below is a blocker for landing the plan or the implementation. Each item is a hot spot to revisit once the (8d) scalar regression and (8e) vset+LS regression pass on `MediumBoomV4VectorConfig` and `MegaBoomV4VectorConfig`.

Companion to `caracal-goal1-plan.md`. Cross-references to that doc use its step numbers and issue numbers.

**Baseline reference.** All numbers below assume `MediumBoomV4VectorConfig`. Current BOOMv4 baseline (from `src/main/scala/v4/`):
- `MicroOp` ≈ 214 bits
- ROB depth = 64 entries (Medium)
- Issue queue sizes: IQ_MEM 12, IQ_UNQ 12, IQ_ALU 20, IQ_FP 12 (44 total slots)
- Int writeback ports = aluWidth (2) + lsuWidth (2) + 1 = 5
- `numIntPhysRegisters` = 80 → 7-bit preg index
- Int regfile = 80 × 64 bits, 5R + 4W (banked, `numIrfBanks=2`)
- LDQ / STQ = 16 / 16 entries
- DCache MSHRs = 2 (Medium), 8 (Mega)
- TLB = ~32 entries, `lsuWidth` ports
- `maxBrCount` = 12 (Medium), 32 (Mega)

---

## High area / complexity

### 1. MicroOp bundle inflates ~70%

The plan adds ~150 bits to the unified `MicroOp` bundle:
- `is_vec` (1) + 5 logical regs × 5 bits (25)
- 5 physical regs × log2(128) (35)
- 2 mask/VL pregs × 7 (14)
- 6 busy bits
- `VConfig` sub-bundle (~30: vstart 8, vl 8, vlmax 8, vsew/vlmul/vma/vta/vxrm/vxsat ~11)
- `v_eew`/`v_emul` (6)
- `v_split_first/_last/_idx/_total` (10, 4-bit widths post-Issue-5)
- segment fields (6)
- `vsetvl_id` (3)
- `vl_is_known` (1)
- `active_lo`/`active_hi` (6)

Total: 214 → ~365 bits per MicroOp.

Ripple cost (Medium):

| Structure | Entries | Old bits | New bits | Delta |
|---|---|---|---|---|
| ROB | 64 | 13,696 | ~23,360 | +9,664 |
| All IQ slots | 44 | ~9,416 | ~16,060 | +6,644 |
| LSU LDQ + STQ | 32 | ~16k+ | ~21k+ | +5k+ |
| FetchBuffer / dispatch group regs | coreWidth×4 | ~1.7k | ~2.9k | +1.2k |

ROB + IQ alone grows ~16 Kbit. Paid even on scalar-only paths because the bundle is unified.

**Mitigation (deferred).** Split MicroOp into a scalar-core struct and a vector-extension struct; only vector ROB entries / vector issue slots carry the extension. The ROB stores the extension in a side array indexed by `rob_idx`, only allocated when `is_vec=true`. Recovers ~10–14 Kbit on scalar-heavy workloads.

---

### 2. 8-wide collapsing queue insert

Plan: `vecIssueInsertWidth = 8` (cracker burst into IQ_V_LOAD / IQ_V_STORE / IQ_V_ALU). Existing BOOM collapsing queues take 2-wide insert at most (`dispatchWidth=2` on Medium IQ_MEM). Going 2 → 8 means the collapsing-shift logic must handle 8 simultaneous arrivals mixed with N grants per cycle. The shift becomes an N-input compactor instead of a 1–2-input shifter.

This is the **single biggest combinational complexity item** in the plan. Likely fails timing on a wide collapsing queue at any reasonable clock target.

**Mitigation options (deferred — pick at perf-tuning time):**
- (a) Non-collapsing matrix-style queue with a separate age matrix. More area, timing-friendly.
- (b) Cap insert at 4 (cracker takes 2 cycles for LMUL=8 instead of 1). Halves issue-side throughput of the cracker burst; preserves existing IssueSlot codebase.
- (c) Insert a per-lane FIFO at the queue write port so the shift logic isn't combinational with the cracker. 1-cycle pipeline delay between cracker emit and queue insert.

Pick at Step-7 implementation time. None of (a)–(c) block functional correctness — the queue still age-orders and grants in order; only the *insert bandwidth* differs.

---

### 3. VL-data fanout to every vector issue slot

Plan (Issue 6 + Step 9): every vector slot (3 queues × 64 slots = **192 slots**) taps each int writeback's data lane (5 ports × 64 bits = 320 bits) plus the cracker-broadcast bus.

Per-slot cost:
- 5 tag-comparators × 7 bits = 35 bits of comparison
- 64-bit data latch + select mux
- 1 cracker-bus matcher on `vsetvl_id`

Total: 192 × (35-bit tag-cmp + 5 × 64-bit mux + 12-bit bus matcher) ≈ **~80 Kbit of comparator/mux logic**, plus broadcasting 320+ bits of data to 192 receivers as a routing problem.

This is the kind of broadcast network real cores try hard to avoid. Modern OoO cores forward through regfile bypass nets, not slot-side data fanout.

**Mitigation (deferred).** Replace data-fanout with "arbitrated regfile read on wakeup":
- Slot only listens to `(valid, pdst)` on the int wakeup network (standard).
- When `pvl_busy` clears, slot allocates a shared regfile read port; 1-cycle later, it has `vl_captured`.
- Trades 1 cycle of latency + 1 read port for ~80 Kbit of fanout.

Defer until area is measured; functional correctness doesn't depend on the choice.

---

### 4. Cross-LSU snoop CAM

Plan (Step 11): vector LB snoops scalar SQ per-element + vector SB; scalar LQ snoops V-SB; intra-vector snoop in both directions.

Worst case for V-LB → scalar SQ: LMUL=8 / SEW=8 / VLEN=256 = 256 elements × 16 scalar SQ entries = **4,096 40-bit comparators per snoop cycle**.

V-SB → scalar LQ direction is even harder structurally: V-SB entries hold address *descriptors* (base + stride + index + mask + segment info), not single addresses. Each scalar load address must be range-checked against each V-SB entry's footprint — per-entry range arithmetic, not equality CAM.

**Mitigation (deferred):**
- (a) Limit per-cycle snoop to the *active sub-uop* (1 sub-uop × elements_per_subuop × SQ entries) instead of "all in-flight elements." With in-order issue (Issue 1), this is the natural bound — already implied by the architecture. Bounds the CAM to 32 elements × 16 = 512 comparators/cycle on Medium.
- (b) Per-bank store-address filter / bloom in front of the CAM, so the full snoop fires only on a candidate hit.
- (c) For V-SB → scalar LQ direction, gate the per-entry range check on a coarser "page-range" hit first.

(a) is essentially free and should be considered part of the Step-11 implementation, not a perf optimization.

---

### 5. TLB pressure — unaddressed in the plan

The plan never extends the TLB. BOOM's TLB has `lsuWidth` ports (1 on Medium, 2 on Mega) and ~32 entries. Vector AGU generates per-element addresses; strided/indexed traffic can hit a new page every element. Scalar TLB sees vector + scalar traffic on the same ports.

**This is the biggest perf cliff Goal 1 will likely hit in the (8d) scalar regression**, and it isn't called out in the plan's cross-cutting risks table. Strided-load benchmarks will degrade scalar IPC measurably because vector AGU traffic occupies the shared TLB port.

**Mitigation options (deferred):**
- (a) Dedicated V-TLB (~16 entries) wired into PTW alongside the scalar TLB. Adds area but isolates pressure.
- (b) Extend scalar TLB ports (`tlbWidth = 2` even on Medium) and arbitrate. Less area, more contention.
- (c) Accept the perf cliff and document for Goal 2.

Goal 1 should at minimum **measure** this — add a TLB-miss-rate perf counter alongside the others in Step 14 task 5, so the cliff is attributable when it appears.

---

## Medium

### 6. Vector regfile 2W/2R at VLEN=256

128 entries × 256 bits = **32 Kbit storage**. 2W + 2R ports, each 256 bits wide. ~6× the storage of the Medium int regfile (80×64 = 5,120 bits). Reasonable for what it does; the W1 (squash-copy) + R0 (AGU) port-sharing scheme from Step 10 adds arbitration in the read/write path.

**Mitigation (deferred).** Defer W1 by combining it with the future V-ALU write port at the Goal 2 boundary. Until V-ALU lands, slot-squash `vta=0` cases can use the single existing write port and arbitrate against load writebacks — Goal 1's vta=0 path is rare enough that load throughput barely suffers.

---

### 7. V-StoreBuffer per-element bitmaps

64 entries × (~128 addr descriptor + 7 pdst + 32-bit drained bitmap + 32-bit computed bitmap + 7 rob_idx + 12 br_mask + active_mask) ≈ **220 bits/entry → 14 Kbit total**.

Plus a drain FSM walking the bitmap per entry. Branch-kill walks all 64 entries' `br_mask` each cycle (64 × 12-bit AND). Real but manageable.

**Mitigation:** none needed at Goal 1. Acceptable cost for correctness.

---

### 8. ROB busy-split counter + 8-wide rename allocation

ROB row extension: 64 × 4 bits = 256 bits. Cheap by itself.

The real issue is **8-wide rename allocation per cycle**. Cracker emits 8 sub-uops; all share one `rob_idx` (good — single ROB row write), but vRRU still allocates 8 vector pregs in 1 cycle. That's an 8-priority-encoder over a 128-bit freelist bitmap, with per-branch `br_alloc_lists` seeing 8 updates per cycle. Current BOOM freelist handles 2-wide allocation; 8-wide is a real combinational extension.

**Mitigation (deferred):** cap cracker emit to `coreWidth` (4 on Medium) so the freelist allocation fits the existing dispatch-width assumption. LMUL=8 takes 2 cycles to crack. Same outcome as the Step-7 cap, applied to the freelist side.

Both #2 and #8 push the same direction: a `coreWidth`-wide cracker is the simpler design point. Goal 1 can ship at 8-wide and optimize down if timing fails, or vice versa.

---

### 9. vRRU 1-cycle scalar-bypass register

MicroOp × coreWidth = ~365 × 4 (Medium) = **1,460 bits** of pipeline register, paid every cycle on every scalar dispatch. Plus +1 cycle on every branch-mispredict refill.

Small in absolute terms. Measured by the `vec_rename_bubble_cycles` counter (Issue 16).

**Mitigation (deferred to Goal 2):** fold vRRU into the sRRU cycle (combinational vector rename in parallel with scalar rename). Risk of busting timing on rename critical path — exactly why Goal 1 picks the bubble.

---

### 10. DCache arbiter + parametrized MSHR policy

Three MSHR policies coexisting in elaborated RTL (`fair-floor` / `hard-partition` / `fcfs`) = some dead logic per mode. Anti-starvation counters and the fair-floor allocator are small but real.

**Mitigation:** none needed. Parametrization is intentional; pick one at production tape-out time and remove the others if area-critical.

---

## Low / trivial

### 11. `store_pending` bit per pdst

128 bits + 1 AND gate at dealloc. Trivial.

### 12. Cracker vtype mirror + br-snapshot + side buffer

`maxBrCount` × ~30-bit snapshot + 4-entry OoO-broadcast side buffer ≈ **480 bits total**. Trivial.

---

## Hardest-to-implement logic (orthogonal to area)

These are design-complexity hot spots, not area. They'll consume implementation calendar disproportionately.

**A. Wide collapsing queue insert.** Covered in #2. Pick the mitigation upfront — this design decision affects implementation time more than any other in Goal 1.

**B. Cross-LSU memory-dependency speculation.** BOOM's mem-dep predictor and ordering-violation replay machinery is already one of the more intricate parts of the scalar LSU. Extending it cross-LSU (a vector load needs to replay because a scalar store address now matches it) requires:
- Per-uop seqnum visible across both LSUs
- Replay-target lookup that finds the right uop in the right queue
- Partial-state rollback (if the vector load already forwarded N elements and now needs to redo them)

Real implementation effort, probably 2–3× a normal step.

**C. `vta=0` regfile-copy micro-op.** Adds a third uop kind on the writeback bus (`squash=true` / `copy=true` / normal). Needs a writeback arbiter that handles all three. The `W1`/`R0` port pair creates port-conflict scenarios with the AGU. Implementable but introduces a side-path with its own state machine.

**D. 8-wide freelist allocation per cycle.** Same root cause as #8. The bitmap freelist with per-branch `br_alloc_lists` currently handles 2-wide allocate. 8-wide stretches the rename critical path. Capping cracker width is the obvious mitigation.

**E. Conditional read of `pvm` in V-LSU.** Step 5 says "conditional read gated on encoded `vm` bit." In hardware, "conditional read" of a regfile port either always-asserts (wasting bandwidth on unmasked uops) or has a fast skip path (bypass mux + 1-cycle alignment). Recommend Step 5 clarify to **"always read, ignore result on unmasked"** — avoids bypass complexity at the cost of a small power increase.

**F. Element-precise vstart on fault under multi-lane DCache.** Even with in-order issue (Issue 1), Mega's 2-lane V-LSU pipeline can have multiple elements in-flight to DCache simultaneously. If lane-0 element 5 faults while lane-1 element 6 has already returned a DCache response, the in-flight wave must drain in-order and not commit element 6 to the regfile. The "in-order writeback" rule covers this in principle, but requires per-element completion tracking in the V-LSU pipeline that's non-trivial.

---

## Mitigations to consider at the Goal-1 → Goal-2 boundary

Once functional correctness is in (all 17 + 3 vsetvl ELFs PASS in the (8e) regression on both Medium and Mega), revisit in this order:

1. **Cap cracker emit width to `coreWidth`** (closes #2, #8, D simultaneously). Single biggest area/complexity win.
2. **Split MicroOp** into scalar + vector-extension (closes #1).
3. **Add V-TLB** (closes #5 — biggest perf cliff).
4. **Replace VL-data fanout with arbitrated regfile read** (closes #3).
5. **Bound cross-LSU snoop to active sub-uop** (closes #4, mostly).
6. **Fold vRRU into sRRU cycle** (closes #9, blocked on timing).

Items 1, 2, 5 are pure area wins with no behavioral change — Goal 2 can start with them. Items 3, 4 are perf wins. Item 6 is risky and may not close timing.

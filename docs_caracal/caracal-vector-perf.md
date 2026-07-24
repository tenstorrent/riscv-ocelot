# Caracal Vector Memory/Arith Performance — Status & Roadmap

Status of the vector load/store + arith performance work, the measured wins, the
current bottlenecks (with evidence), and the prioritized plan to go faster. Companion to
`caracal-milestone2-plan.md` (Track A/B/C) and `src/loadstore.rst` (§mem-order).

All numbers are VCS + Whisper cosim, `run-binary-debug-hex`, sim-time to `$finish`
(TestDriver.v:158), **0 Register Mismatch** unless noted. Kernels: `tests/rvv/kernels/`
(`axpy`, `_axpy`, `conv1d` — all unit-stride).

---

## 1. Work done (committed)

### Correctness enablers (made the vector-arith kernels run at all)
| Area | boom commit |
|---|---|
| CII 4-lane operand pull + single-reg-dest (`vmv.s.x`/reduction/mask) + x0 vector decode | `5e2960d2` |
| CII load/accumulate correctness (axpy/conv1d pass) | `1bba52d0` |
| `vsetvli x0,x0` keep-vl: preserve VL, not VLMAX | `a33e4ef6` |
| Wide-tier (MegaBoom, decodeWidth=4/memWidth=2): VlRename, store pipe-0, keep-vl RAW stall | `49c3c6d0` |

### Performance features
| # | Feature | boom commit | What it does |
|---|---|---|---|
| P1 | **Multi-outstanding loads** | `a785b7d1` | Descriptor table: up to `vecLoadMaxInflight`(=8) 64b beats in flight on the vec_dmem port (was serial 1-at-a-time). Hides miss latency. |
| P2 | **Dual-dynamic 2-wide loads** | `f4af2a49` | 2nd load beat opportunistically claims an idle scalar D$ pipe (Mega, `lsuWidth=2`); AGEN cracks 2 beats/cycle on the unit-stride fast path. |
| P3 | **Track A A2 — store→load disambiguation** | `60757775` | Vector loads execute **speculatively**; correctness via the LCAM store-search + `order_fail` refetch replay, **replacing** the M1 conservative "load waits for all older stores to *commit*" barrier. **The biggest single win.** Also: AGEN/DGEN issue queues 8→16, D$-traffic debug signals. |
| P4 | **Track A A1 substrate (modules)** | `87c435f0` | `VecMemQueues` (6 spec queues) + `DcacheArbiter` (priority-RR) + `CrossLsuSnoop` (store→LDQ disambig as a module). `CrossLsuSnoop` wired + validated bit-identical; queues/arbiter built, unwired (await multi-group VecLSU). |

### Measured wins
| kernel / config | before | after (P1–P3) | Δ |
|---|---|---|---|
| **Mega** axpy | 46.1 ms | **38.58 ms** | **−16%** |
| **Mega** conv1d | 140.0 ms | **74.69 ms** | **−47%** |
| **Medium** axpy | 57.83 ms | **54.05 ms** | −6.5% |

P3 (A2 speculative loads) is where the big Mega wins come from — conv1d especially was
heavily store→commit serialized (store `output[i]`, then next iteration's `input[i+1+k]`
loads waited for that store to reach the ROB head).

### Test / validation assets
- `tests/rvv/bringup_tests/ms11e_ssi_alias.S/.elf` (superproj `c9b8f299`) — directed SSI
  store→younger-load ordering regression.
- `MediumBoomV4VectorSnoopConfig` — arms the A1 `CrossLsuSnoop` (`vecScalarSnoopEnable`).
- dontTouch D$-traffic debug signals (`dbg_dmem_req_fire_cnt`, `dbg_vec_beats_cnt`,
  `dbg_desc_*` in LSU/VecLSU) for per-cycle transaction inspection in the waveform.

---

## 2. Current bottlenecks (with evidence)

1. **Single-group VecLSU serializes the whole vec-LS front-end.** The pipeline
   `IQ_V_{LOAD,STORE}` → `VecLSRegRead` → `VecAgenStage1` → `VecLSU` handles **one vector
   memory op at a time**: `VecLSRegRead` reads one op's operands per pass and its
   `fu_ready` gates on *full* `VecLSU` idle (`lsu_busy`). So a store's whole beat-drain
   blocks the next iteration's loads. This is the dominant remaining vector-LS limiter.
2. **CII arith latency / issue cadence.** The CII coprocessor does a `vfmacc.vv` (LMUL=1)
   in ~14 cycles issue→writeback (after `CRD_REQ/RETURN_DELAY`→0). Post-A2, arith-heavy
   kernels (conv1d dot-products) are limited by the vfmacc dependency chain + CII issue,
   not the load path.
3. **D$ port bandwidth caps the load wins.** Medium has **1** D$ pipe (beats serialize at
   1/cycle regardless); Mega has 2 (dual-dynamic already uses both for loads). Any
   multi-group overlap is bounded by this.
4. **SSI is conservative but this is currently harmless.** Strided/indexed loads use a
   full-range `[0,MAX)` over-match (replay-heavy in principle). On the current single-group
   VecLSU, SSI ops **serialize naturally** (shared pipe, in order) — the directed test
   showed 0 replays — so precise SSI (A3) is a **no-op here** (see §3).

---

## 3. Roadmap to go faster (prioritized)

### R1 — Multi-group VecLSU  ★ highest impact, highest effort
Re-architect the vec-LS front-end (`VecLSRegRead` gating, core issue gating, `VecLSU`
FSM) for **per-context concurrency**: first a LOAD context + a STORE context running
concurrently (a store drains while the next iteration's loads issue), then N load groups
(cross-iteration overlap). This is the single change that unlocks the remaining vector-LS
throughput — and it makes the **A1 queues + `DcacheArbiter` live** and makes **A3
meaningful**.
- Scope: multi-file, correctness-critical. Needs `VecLSU` split into contexts +
  `io.busy` → `busy_load`/`busy_store` + `VecLSRegRead`/core per-context `fu_ready` +
  vec_dmem store-vs-load port arbitration (untangle from the dual-dynamic `req`/`req2`).
- Expected win: real on **Mega** (store pipe-0 + load pipe-1 concurrently → ~2 beats/cyc
  cross-group); modest on **Medium** (1 port — recovers FSM/setup/dependency overlap only).
- Effort/risk: multi-week, gated (`vecLsuOverlap`, bit-identical off) + staged +
  cosim-validated. **Attempted the bounded increment this session; reverted** on finding
  it's a front-end re-architecture, not a localized VecLSU edit — build it as a designed
  project, not an incremental edit.

### R2 — CII vector-arith throughput
After A2, arith-heavy kernels are CII-bound. Options: deeper/pipelined vfmacc issue,
`src_reuse_hint` (currently ignored — re-pull elision), issuing independent vector-arith
ops back-to-back within the 16-deep CII credit window. Investigate with the waveform
(the ~14-cyc op + the inter-issue gap) before committing.

### R3 — Track A A4: LD→ST forwarding
Forward an older store's data to a younger overlapping load instead of replaying it
(`CrossLsuSnoop` has the load→STQ search scaffold). Cuts the replay cost where aliasing is
real. Correctness floor (`order_fail` replay) already in place.

### R4 — Track A A3: precise SSI (only after R1)
Wire `VecMemQueues` (`SSI_ADDR_Q`) + per-element `CrossLsuSnoop` search + the SSI→SSI hold.
**Provably a no-op until R1 exists** (single-group VecLSU serializes SSI naturally — the
directed test confirmed 0 replays/reorder). Do R1 first, then this earns its keep.

### R5 — Track A A0: DTLB effective addresses
Route vector beats through the shared DTLB (currently bare-physical / identity-mapped).
**Correctness-generality, not perf** — required for real page tables; not needed for the
identity-mapped riscv-tests.

### Cross-cutting: a directed alias test for A2
The kernels don't alias, so A2's `order_fail` replay path is correct-by-construction but
unexercised by a real store→younger-load alias. `ms11e_ssi_alias` covers SSI; add a
US-store→younger-load alias ELF to exercise the US replay directly (spec (9e3)).

---

## 4. Config quick-reference
- `MediumBoomV4VectorArithConfig` / `MegaBoomV4VectorArithConfig` — CII vector-arith
  (`enableVectorArith`); Mega adds `dcacheArbiterMode="dual-dynamic"` (P2).
- `MediumBoomV4VectorSnoopConfig` — arms A1 `CrossLsuSnoop` (`vecScalarSnoopEnable`).
- Build/run: `caracal-vector-lsu-bandwidth` / `caracal-tracka-disambiguation` notes and
  the top-level `CLAUDE.md` (container bootstrap, `USE_IMAGE_WHISPER=1`, `.daidir` relink
  on config switch, `whisper_connect` one-cosim-at-a-time).

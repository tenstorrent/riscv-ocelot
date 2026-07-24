# BOOM v4 Architecture Reference

This document is a reference for the BOOM v4 out-of-order RISC-V core as it lives in this repository. Its purpose is to support **future architectural changes** — adding new instruction classes (e.g., RVV 1.0 for the Caracal project), adding new execution units, retuning sizing parameters, and reasoning about microarchitectural invariants. It is **not** a beginner's tutorial on OoO microarchitecture; it assumes the reader is familiar with rename, ROB-based commit, and speculative execution.

All paths below are relative to `src/main/scala/v4/` unless otherwise noted. Packages are `boom.v4.{common,ifu,exu,lsu,util}`.

---

## 1. Scope and integration

BOOM is a Chisel library, not a self-running design. It is elaborated by a higher-level SoC generator (Chipyard) which attaches the BOOM tile to a memory system, interrupt controller, and bootrom. The integration shape:

- **`common/tile.scala`** defines `BoomTile` (a `LazyModule` extending `BaseTile`) and `BoomTileAttachParams`. `BoomTile` brings in `BoomFrontend` (the IFU + ICache), `BoomNonBlockingDCache`, the PTW, optional RoCC accelerators, and exports a TileLink master node aggregating ICache/DCache/RoCC traffic. The diplomacy graph is built in the lazy phase; module instantiation happens in `BoomTileModuleImp`.
- **`exu/core.scala`** is the synchronous top of the actual core — it instantiates decode, rename, dispatch, issue, register-read, execution units, the FP pipeline, the ROB, and the LSU's core-side interface. Everything below in this document ultimately gets wired together here.

Versioning: `v3/` (SonicBOOM) and `v4/` live side by side; all Caracal work targets `v4/`.

---

## 2. Parameter system

### 2.1 `BoomCoreParams`

Defined as `case class BoomCoreParams(...)` in `src/main/scala/v4/common/parameters.scala` (lines 25–130, between the `// DOC include start` / `// DOC include end` markers). Constraints below come from explicit `require(...)` calls in the same file or in the `HasBoomCoreParameters` trait (lines 193–370); the **Source** column points at the authoritative line(s). Defaults are taken directly from the case-class declaration. "No explicit cap" means there is no `require` enforcing an upper bound — practical limits come from synthesis cost, not elaboration.

#### Pipeline widths and reorder window

| Knob | Default | Valid range | Source |
|------|---------|-------------|--------|
| `fetchWidth` | 4 | Power of 2; `≥ decodeWidth` | parameters.scala:28, 206, 207 |
| `decodeWidth` (aliased `coreWidth`) | 1 | `1 ≤ decodeWidth ≤ fetchWidth` | parameters.scala:29, 204, 207 |
| `numRobEntries` | 32 | `> 0`, multiple of `decodeWidth` | parameters.scala:30, 351 |
| `maxBrCount` | 4 | `≥ 2` | parameters.scala:46, 350 |
| `numFetchBufferEntries` | 8 | `> 0` (no explicit cap) | parameters.scala:47 |
| `ftq.nEntries` | 16 | `> 0` (no explicit cap) | parameters.scala:66 |

#### Issue queues

`issueParams: Seq[IssueParams]` declares one `IssueParams(issueWidth, numEntries, iqType, dispatchWidth)` per queue. The default declares four queues — `IQ_MEM(2×8)`, `IQ_UNQ(1×8)`, `IQ_ALU(1×8)`, `IQ_FP(1×8)` — and the `require`s mandate exactly that *set* of `iqType`s.

| Constraint | Range | Source |
|------------|-------|--------|
| Count of `iqType == IQ_FP`  | `== 1` (or `!usingFPU`) | parameters.scala:255 |
| Count of `iqType == IQ_MEM` | `== 1` | parameters.scala:256 |
| Count of `iqType == IQ_ALU` | `== 1` | parameters.scala:257 |
| Count of `iqType == IQ_UNQ` | `== 1` | parameters.scala:258 |
| `unqIssueParam.issueWidth` | `== 1` | parameters.scala:265 |
| `memIssueParam.issueWidth` (`memWidth`) | `≥ 2` *and* `≥ lsuWidth` | parameters.scala:272–273 |
| `IssueParams.dispatchWidth` (every entry) | `1 ≤ dispatchWidth ≤ coreWidth` | parameters.scala:275 |
| `aluIssueParam.issueWidth` (`aluWidth`) | `> 0`; `enableColumnALUWrites` additionally needs `isPow2(aluWidth) && aluWidth > 1` | parameters.scala:324 |
| `fpIssueParam.issueWidth` (`fpWidth`) | `> 0` | parameters.scala:268 |
| `lsuWidth` | `1 ≤ lsuWidth ≤ memWidth` | parameters.scala:36, 273 |

Per-queue `numEntries` has no `require` floor or cap beyond elaboration cost.

#### LSU queue depths

| Knob | Default | Valid range | Source |
|------|---------|-------------|--------|
| `numLdqEntries` | 8 | `> coreWidth + 1` (i.e. `numLdqEntries − 1 > coreWidth`) | parameters.scala:37, 352 |
| `numStqEntries` | 8 | `> coreWidth + 1` | parameters.scala:38, 353 |

#### Physical register files

| Knob | Default | Valid range | Source |
|------|---------|-------------|--------|
| `numIntPhysRegisters` | 48 | `≥ 32 + coreWidth` | parameters.scala:39, 348 |
| `numFpPhysRegisters` | 48 | `≥ 32 + coreWidth` (unused when `!usingFPU`) | parameters.scala:40, 349 |
| `numImmPhysRegisters` | 32 | `> 0` (no explicit cap) | parameters.scala:41 |
| `numIrfReadPorts` | 3 | `> 0` (no explicit cap; per-bank read ports) | parameters.scala:42 |
| `numFrfReadPorts` | 3 | `> 0` (no explicit cap; per-bank read ports) | parameters.scala:43 |
| `numIrfBanks` | 1 | `≥ 1` | parameters.scala:44 |
| `numFrfBanks` | 1 | `≥ 1` | parameters.scala:45 |

#### Execution latencies

| Knob | Default | Valid range | Source |
|------|---------|-------------|--------|
| `imulLatency` | 3 | `> 0` (pipelined-mul depth) | parameters.scala:68 |
| `intToFpLatency` | 2 | `> 0` | parameters.scala:67 |
| `fpu.sfmaLatency` / `fpu.dfmaLatency` | 4 / 4 | `sfmaLatency == dfmaLatency` (unified write-port schedule) | parameters.scala:93, 245 |

#### RoCC, DCache, ICache, PMP, paging

| Knob | Default | Valid range | Source |
|------|---------|-------------|--------|
| `numRXQEntries` | 4 | `> 0`; small — holds operand snapshots + instruction bits | parameters.scala:70 |
| `numRCQEntries` | 8 | `> 0`; just holds preg pointers | parameters.scala:71 |
| `numDCacheBanks` | 1 | `1` ⇒ duplicated arrays (no bank conflicts); `≥ lsuWidth` ⇒ banked (conflicts NACK) — enforced in `lsu/dcache.scala` | parameters.scala:72 |
| `dcacheSinglePorted` | false | — | parameters.scala:73 |
| `nPMPs` | 8 | `≥ 0`; Rocket Chip PMP infrastructure effectively caps at 16 | parameters.scala:75 |
| `enableICacheDelay` | false | — | parameters.scala:76 |
| `icacheSinglePorted` | true | — | parameters.scala:77 |
| `icacheParams.nSets` (set in Chipyard) | (varies) | `≤ 64` (alias-handling assertion) | parameters.scala:283 |
| `pgLevels` | 3 | `3` = Sv39, `4` = Sv48, `5` = Sv57 | parameters.scala:27 |
| `nL2TLBEntries` / `nL2TLBWays` | 512 / 1 | `> 0` | parameters.scala:102–103 |

#### Branch prediction

| Knob | Default | Valid range | Source |
|------|---------|-------------|--------|
| `enableBranchPrediction` | true | — | parameters.scala:80 |
| `branchPredictor` | identity factory | Set by BPD mixins (TAGEL, Boom2, Alpha21264, SW) | parameters.scala:81 |
| `globalHistoryLength` | 64 | `> 0`; BPD-stack dependent | parameters.scala:82 |
| `localHistoryLength` | 32 | Local history enabled iff `localHistoryNSets > 1 && localHistoryLength > 1` | parameters.scala:83, 311 |
| `localHistoryNSets` | 128 | Local history enabled iff `localHistoryNSets > 1 && localHistoryLength > 1` | parameters.scala:84, 311 |
| `bpdMaxMetaLength` | 120 | Must be `≥` sum of per-tier metadata bits | parameters.scala:85 |
| `numRasEntries` | 32 | `0` disables RAS; nonzero floored to `2` internally (`nRasEntries = numRasEntries max 2`) | parameters.scala:86, 304–305 |
| `enableRasTopRepair` | true | — | parameters.scala:87 |

#### Feature flags (Booleans)

The architecturally interesting flags (semantics in §2.2):

| Flag | Default | Source | Inter-flag constraint |
|------|---------|--------|-----------------------|
| `enableColumnALUIssue` | false | parameters.scala:48 | — |
| `enableALUSingleWideDispatch` | false | parameters.scala:49 | Requires `enableColumnALUIssue` (parameters.scala:323) |
| `enableBankedFPFreelist` | false | parameters.scala:50 | — |
| `enablePrefetching` | false | parameters.scala:51 | — |
| `enableFastLoadUse` | false | parameters.scala:52 | — |
| `enableCompactingLSUDuringDispatch` | true | parameters.scala:53 | — |
| `enableAgenStage` | false | parameters.scala:54 | — |
| `enableStLdForwarding` | false | parameters.scala:55 | — |
| `enableFastPNR` | false | parameters.scala:56 | — |
| `enableSFBOpt` | true | parameters.scala:57 | — |
| `enableGHistStallRepair` | true | parameters.scala:58 | — |
| `enableBTBFastRepair` | true | parameters.scala:59 | — |
| `enableLoadToStoreForwarding` | true | parameters.scala:60 | — |
| `enableSuperscalarSnapshots` | true | parameters.scala:61 | — |
| `enableSlowBTBRedirect` | false | parameters.scala:62 | — |
| `enableBPDHPMs` | false | parameters.scala:63 | — |
| `useAtomicsOnlyForIO` | false | parameters.scala:65 | — |
| `enableConservativeSNI` | false | parameters.scala:125 | Speculative non-interference (side-channel mitigation) |
| `enableCommitLogPrintf` | false | parameters.scala:120 | — |
| `enableBranchPrintf` | false | parameters.scala:121 | — |
| `enableMemtracePrintf` | false | parameters.scala:122 | — |
| `enableTraceCoreIngress` | false | parameters.scala:127 | — |

#### Inherited / overridden from Rocket `CoreParams`

`BoomCoreParams extends freechips.rocketchip.tile.CoreParams` and overrides these in its body (lines 132–148):

| Name | Value | Comment |
|------|-------|---------|
| `xLen` | 64 | RV64 only — `require(!(xLen == 32 && usingFPU))` (parameters.scala:236) |
| `retireWidth` | `= decodeWidth` | parameters.scala:138 |
| `pmpGranularity` | 4 | parameters.scala:135 |
| `lrscCycles` | 80 | Worst-case LR/SC reservation hold (parameters.scala:137) |
| `useZba` / `useZbb` / `useZbs` | true | Bit-manip extensions always enabled (parameters.scala:144–146) |
| `useHypervisor` | false | parameters.scala:140 |

#### Derived widths

Computed in `HasBoomCoreParameters` (`parameters.scala:332–346`); these are the bit-widths future modules must match when extending the `MicroOp` bundle:

```scala
val numRobRows = numRobEntries / coreWidth
val robAddrSz  = log2Ceil(numRobRows) + log2Ceil(coreWidth)
val logicalRegCount = if (usingFPU) 64 else 32
val lregSz     = log2Ceil(logicalRegCount)
val ipregSz    = log2Ceil(numIntPhysRegs)
val fpregSz    = log2Ceil(numFpPhysRegs)
val maxPregSz  = ipregSz max fpregSz
val immPregSz  = log2Ceil(numImmPhysRegs)
val ldqAddrSz  = log2Ceil(numLdqEntries)
val stqAddrSz  = log2Ceil(numStqEntries)
val lsuAddrSz  = ldqAddrSz max stqAddrSz
val brTagSz    = log2Ceil(maxBrCount)
```

### 2.2 Feature flags

Boolean flags on `BoomCoreParams` gate optional behaviors. The non-obvious ones:

- `enableColumnALUIssue` / `enableALUSingleWideDispatch` — column-banked ALU layout with banked dispatch.
- `enableBankedFPFreelist` — banked FP freelist (reduces priority-encoder cost at wide rename).
- `enableAgenStage` — separate AGEN pipeline stage in the LSU instead of combinational addr-gen.
- `enableStLdForwarding` / `enableLoadToStoreForwarding` — two distinct optimizations (see §10).
- `enableFastLoadUse` — speculative load-use wakeup (with poison bits for replay).
- `enableFastPNR` — single-cycle PNR computation via age-priority encoder vs. multi-cycle scan.
- `enableSFBOpt` — Short Forward Branch shadow optimization (predicate-based folding of small forward branches into conditional moves).
- `enableSuperscalarSnapshots` — branch snapshots that can capture a full decode group, not just one branch.
- `enableGHistStallRepair`, `enableBTBFastRepair`, `enableSlowBTBRedirect` — frontend recovery optimizations.
- `enableCompactingLSUDuringDispatch` — let the dispatcher partially fill the LSU when other queues stall.
- `enableCommitLogPrintf`, `enableBranchPrintf`, `enableMemtracePrintf` — debug traces (also exposed as standalone mixins).

### 2.3 Structural invariants (`require(...)`)

The full set of elaboration-time invariants in `src/main/scala/v4/common/parameters.scala` (the `HasBoomCoreParameters` trait body). Any structural change must keep these satisfied — failing one is an elaboration error, not a runtime issue.

```scala
// Fetch / decode widths
require(isPow2(fetchWidth))                                                    // line 206
require(coreWidth <= fetchWidth)                                               // line 207

// XLEN / FPU
require(!(xLen == 32 && usingFPU), "RV32 does not support fp")                 // line 236

// FP latency unification (all FPU ops padded to same write-port schedule)
require(sfmaLatency == dfmaLatency)                                            // line 245

// Issue queues — exactly one of each type
require(issueParams.count(_.iqType == IQ_FP)  == 1 || !usingFPU)               // line 255
require(issueParams.count(_.iqType == IQ_MEM) == 1)                            // line 256
require(issueParams.count(_.iqType == IQ_ALU) == 1)                            // line 257
require(issueParams.count(_.iqType == IQ_UNQ) == 1)                            // line 258

// Issue widths
require(unqIssueParam.issueWidth == 1)                                         // line 265
require(memWidth >= 2)                                                         // line 272
require(memWidth >= lsuWidth)                                                  // line 273

// Dispatch widths (applied to every IssueParams entry)
issueParams.map(x => require(x.dispatchWidth <= coreWidth && x.dispatchWidth > 0))  // line 275

// ICache aliasing (limit imposed because virtual-index alias handling is buggy)
require(icacheParams.nSets <= 64, "Handling aliases in the ICache is buggy.")  // line 283

// Column-ALU dependency
require(!enableALUSingleWideDispatch || enableColumnALUIssue)                  // line 323

// Physical registers / branch / ROB / LSU sizing
require(numIntPhysRegs >= 32 + coreWidth)                                      // line 348
require(numFpPhysRegs  >= 32 + coreWidth)                                      // line 349
require(maxBrCount >= 2)                                                       // line 350
require(numRobEntries % coreWidth == 0)                                        // line 351
require((numLdqEntries - 1) > coreWidth)                                       // line 352
require((numStqEntries - 1) > coreWidth)                                       // line 353
```

Two non-`require` invariants worth knowing because they silently constrain values:

- `nRasEntries = numRasEntries max 2` (line 304) — passing `numRasEntries = 1` does **not** error; it is silently floored to 2. To disable RAS, pass `0` (line 305: `useRAS = numRasEntries > 0`).
- `enableColumnALUWrites = enableColumnALUIssue && isPow2(aluWidth) && aluWidth > 1` (line 324) — column writes are only enabled when `aluWidth` is a non-trivial power of two; otherwise the column-issue mixin elaborates without column-writes even if you asked for them.

The "exactly one of each issue queue" set (lines 255–258) is the load-bearing constraint for any new issue queue type. Adding `IQ_VEC` means relaxing this set and threading the new type through dispatch (§8.1), ROB (§12), and `core.scala` wakeup aggregation (§13). Note that `IQ_UNQ` additionally pins `issueWidth == 1` (line 265) — a vector queue would not have that restriction unless you intend it.

Additional `require`s exist downstream in `exu/core.scala` (wakeup-port counts, write-port matches, bypass-port matches) — those fire when the EU mix changes rather than when the parameter values change; see §13.

### 2.4 Config tiers

`src/main/scala/v4/common/config-mixins.scala` packages parameter sets. Each tier mixin is a `Config((site, here, up) => { ... })` block that overrides `TilesLocated(InSubsystem)` with `BoomTileAttachParams(tileParams = BoomTileParams(core = BoomCoreParams(...)))`. The numbers below are read directly from those overrides; any field not listed inherits the `BoomCoreParams` default from §2.1. All tiers default to `WithTAGELBPD` (TAGE-L) except `WithNMegaTapeoutBooms`, which uses `WithFastTAGEBPD(singlePorted)`.

#### `WithNSmallBooms` — 1-wide baseline (config-mixins.scala:96)

| | |
|---|---|
| `fetchWidth` / `decodeWidth` | 4 / 1 |
| `numRobEntries` | 32 |
| Issue queues | MEM `2×8` · UNQ `1×8` · ALU `1×8` · FP `1×8` (all `dispatchWidth=1`) |
| `lsuWidth` | 1 (default) |
| `numIntPhysRegisters` / `numFpPhysRegisters` | 52 / 48 |
| `numImmPhysRegisters` | 32 (default — not overridden) |
| `numIrfReadPorts` / `numIrfBanks` | 3 / 1 (default) |
| `numFrfReadPorts` / `numFrfBanks` | 3 / 1 |
| `numLdqEntries` / `numStqEntries` | 8 / 8 |
| `maxBrCount` | 8 |
| `numFetchBufferEntries` / `ftq.nEntries` | 8 / 16 |
| `numDCacheBanks` | 1 (default) |
| Notable flags | None overridden (`enableSFBOpt=true`, `enableSuperscalarSnapshots=true` from defaults) |
| DCache / ICache | `nSets=64, nWays=4, nMSHRs=2, fetchBytes=8` |

#### `WithNMediumBooms` — 2-wide (config-mixins.scala:146)

| | |
|---|---|
| `fetchWidth` / `decodeWidth` | 4 / 2 |
| `numRobEntries` | 64 |
| Issue queues | MEM `2×12` · UNQ `1×12` · ALU `2×20` · FP `1×12` (all `dispatchWidth=2`) |
| `numIntPhysRegisters` / `numFpPhysRegisters` | 80 / 64 |
| `numIrfReadPorts` / `numIrfBanks` | 5 / 2 |
| `numFrfReadPorts` / `numFrfBanks` | 3 / 1 |
| `numLdqEntries` / `numStqEntries` | 16 / 16 |
| `maxBrCount` | 12 |
| `numFetchBufferEntries` / `ftq.nEntries` | 16 / 32 |
| DCache / ICache | same as Small |

#### `WithNLargeBooms` — 3-wide, A15-class (config-mixins.scala:197)

| | |
|---|---|
| `fetchWidth` / `decodeWidth` | 8 / 3 |
| `numRobEntries` | 96 |
| Issue queues | MEM `2×16` · UNQ `1×16` (`numSlowEntries=8`) · ALU `3×16` (`numSlowEntries=8`) · FP `1×24` (`numSlowEntries=12`) |
| `numIntPhysRegisters` / `numFpPhysRegisters` | 100 / 96 |
| `numIrfReadPorts` / `numIrfBanks` | 6 / 2 |
| `numFrfReadPorts` / `numFrfBanks` | 3 / 1 |
| `numLdqEntries` / `numStqEntries` | 24 / 24 |
| `maxBrCount` | 16 |
| `numFetchBufferEntries` / `ftq.nEntries` | 24 / 32 |
| Notable flags | `enableColumnALUIssue = true` |
| DCache / ICache | `nSets=64, nWays=8, nMSHRs=4, fetchBytes=16, rowBits=128` |

#### `WithNMegaBooms` — 4-wide (config-mixins.scala:246)

| | |
|---|---|
| `fetchWidth` / `decodeWidth` | 8 / 4 |
| `numRobEntries` | 128 |
| Issue queues | MEM `3×32` · UNQ `1×20` · ALU `4×40` · FP `2×32` (all `dispatchWidth=4`) |
| `lsuWidth` | 2 |
| `numIntPhysRegisters` / `numFpPhysRegisters` | 144 / 128 |
| `numIrfReadPorts` / `numIrfBanks` | 4 / 4 |
| `numFrfReadPorts` / `numFrfBanks` | 6 / 1 |
| `numLdqEntries` / `numStqEntries` | 32 / 32 |
| `maxBrCount` | 20 |
| `numFetchBufferEntries` / `ftq.nEntries` | 32 / 40 |
| `numDCacheBanks` | 4 (banked DCache) |
| Notable flags | `enablePrefetching = true`, `enableFastLoadUse = true`, `enableSuperscalarSnapshots = true` |
| DCache / ICache | `nSets=64, nWays=8, nMSHRs=8, fetchBytes=16` |

#### `WithNMegaTapeoutBooms` — 4-wide ASIC tape-out variant (config-mixins.scala:298)

Mega's general sizing but tuned for single-ported SRAMs and timing closure. Constructor arg `singlePorted: Boolean = true` propagates to `dcacheSinglePorted` / `icacheSinglePorted` and to `WithFastTAGEBPD`.

| | |
|---|---|
| `fetchWidth` / `decodeWidth` | 8 / 4 |
| `numRobEntries` | 128 |
| Issue queues | MEM `2×32` (`numSlowEntries=20`) · UNQ `1×20` (`numSlowEntries=12`) · ALU `4×20` (`numSlowEntries=12`) · FP `2×32` (`numSlowEntries=20`) |
| `lsuWidth` | 2 |
| `imulLatency` | 4 (vs default 3 — slack for single-ported RAMs) |
| `numIntPhysRegisters` / `numFpPhysRegisters` | 144 / 128 |
| `numIrfReadPorts` / `numIrfBanks` | 4 / 4 |
| `numFrfReadPorts` / `numFrfBanks` | 4 / 4 (banked FP RF) |
| `numLdqEntries` / `numStqEntries` | 32 / 32 |
| `maxBrCount` | 20 |
| `numFetchBufferEntries` / `ftq.nEntries` | 32 / 40 |
| `numDCacheBanks` | 4 |
| `dcacheSinglePorted` / `icacheSinglePorted` | `singlePorted` (default true) |
| Notable flags | `enableColumnALUIssue=true`, `enableALUSingleWideDispatch=true`, `enableBankedFPFreelist=true`, `enableAgenStage=true`, `enableSlowBTBRedirect=true`, `enablePrefetching=true`, `enableSuperscalarSnapshots=false`, `enableCompactingLSUDuringDispatch=false`, `enableStLdForwarding=false` |
| BPD | `WithFastTAGEBPD(singlePorted)` (the only tier *not* using `WithTAGELBPD`) |
| DCache | `nSets=128, nWays=4` (wider/shallower vs Mega) |

#### `WithNGigaBooms` — 5-wide widest (config-mixins.scala:363)

| | |
|---|---|
| `fetchWidth` / `decodeWidth` | 8 / 5 |
| `numRobEntries` | 130 |
| Issue queues | MEM `2×32` (`numSlowEntries=12`) · UNQ `1×32` (`numSlowEntries=24`) · ALU `5×20` (`numSlowEntries=10`) · FP `2×32` (`numSlowEntries=20`) |
| `lsuWidth` | 2 |
| `numIntPhysRegisters` / `numFpPhysRegisters` | 128 / 128 |
| `numIrfReadPorts` / `numIrfBanks` | 3 / 1 (defaults — not overridden!) |
| `numFrfReadPorts` / `numFrfBanks` | 6 / 1 |
| `numLdqEntries` / `numStqEntries` | 32 / 32 |
| `maxBrCount` | 20 |
| `numFetchBufferEntries` / `ftq.nEntries` | 40 / 40 |
| `numDCacheBanks` | 1 (back to duplicated arrays — Giga reverts from Mega's banked DCache) |
| Notable flags | `enablePrefetching=true`, `enableColumnALUIssue=true`, `enableSuperscalarSnapshots=false` |
| DCache / ICache | `nSets=64, nWays=8, nMSHRs=8, fetchBytes=16` |

#### Cross-tier observations

- **`numImmPhysRegisters` is never overridden** by any tier — all configs use the default `32`. If immediate-register pressure becomes a bottleneck for wider RVV ops, this is a free knob.
- **`numSlowEntries`** (an `IssueParams` field) first appears at Large and is set on every tier from Large onward. It controls a slow/fast partition inside the collapsing issue queue; entries beyond `numSlowEntries` shift slower or live in a separate physical bank.
- **`lsuWidth`** is `1` for Small/Medium/Large (default) and `2` for Mega/MegaTapeout/Giga.
- **DCache banking** is non-monotonic across tiers: Small/Med/Large/Giga use `numDCacheBanks=1` (duplicated), while Mega/MegaTapeout use `4` (banked). Adding `lsuWidth=2` requires either duplication or `numDCacheBanks ≥ lsuWidth`.
- **`enableSuperscalarSnapshots`** is `true` by default *and* in Small/Medium/Large/Mega, but explicitly `false` in MegaTapeout and Giga (the widest tiers turn it off, likely for area).

#### Branch-predictor mixins

The `branchPredictor` field of `BoomCoreParams` is a factory `(BranchPredictionBankResponse, Parameters) => (Seq[BranchPredictorBank], BranchPredictionBankResponse)` that composes a per-bank predictor stack. Each mixin replaces this field plus the BPD sizing params:

| Mixin | File:Line | Notes |
|-------|-----------|-------|
| `WithFastTAGEBPD(singlePorted)` | config-mixins.scala:541 | Used by `WithNMegaTapeoutBooms`; tuned for single-ported SRAMs |
| `WithTAGELBPD` | config-mixins.scala:587 | Default for every other tier; TAGE with L-loop predictor |
| `WithBoom2BPD` | config-mixins.scala:616 | BOOMv2-era predictor; mostly for comparison |
| `WithAlpha21264BPD` | config-mixins.scala:643 | Hybrid local/global, Alpha 21264-style |
| `WithSWBPD` | config-mixins.scala:672 | Software-driven predictor (research) |

A core gets exactly one BPD mixin; combining tier + BPD is the standard composition pattern (e.g., `new WithNLargeBooms ++ new WithBoom2BPD ++ baseConfig`).

---

## 3. The `MicroOp` bundle

`common/micro-op.scala` defines `class MicroOp` — the in-flight instruction descriptor that flows through every pipeline stage. Almost every architectural change touches this bundle. The fields, grouped by purpose:

**Identity / debug.** `uopc` (uop opcode, ~110 values in `consts.scala`), `inst` (original 32-bit insn), `debug_inst`, `debug_pc`, `is_rvc`, `pc_lob` (low PC bits for full reconstruction with FTQ entry), `edge_inst` (instruction straddled fetch packet).

**Logical registers (set at decode).** `ldst`, `lrs1`, `lrs2`, `lrs3` and their types (`dst_rtype`, `lrs1_rtype`, `lrs2_rtype` — values from `RT_FIX`, `RT_FLT`, `RT_X`, `RT_PAS`). `ldst_val`, `frs3_en`.

**Physical registers (set at rename).** `pdst`, `prs1`, `prs2`, `prs3`, `ppred` (predicate preg, used by SFB); `stale_pdst` (freed at commit); `prs*_busy`, `ppred_busy` (set at ren2 from busytable, consumed at issue).

**Issue / FU routing.** `iq_type` (3-bit mask — `IQ_MEM` / `IQ_UNQ` / `IQ_ALU` / `IQ_FP`, bitwise so the dispatcher routes via `iq_type & issueParam.iqType`), `fu_code` (functional unit bitmask), `iw_state`, `iw_p1_poisoned`, `iw_p2_poisoned` (load-speculation poison).

**ROB and queues.** `rob_idx` (set at dispatch), `ldq_idx`, `stq_idx`, `rxq_idx` (RoCC).

**Branch tracking.** `br_tag` (snapshot ID for this branch, allocated at rename when `allocate_brtag` is true), `br_mask` (bitmask of older speculative branches this uop sits under), `is_br`, `is_jalr`, `is_jal`, `is_sfb`, `taken`, `ftq_idx`.

**Immediate / CSR.** `imm_packed` (densely packed immediate; expanded by FU), `csr_addr`.

**Memory.** `mem_cmd`, `mem_size`, `mem_signed`, `uses_ldq`, `uses_stq`, `is_fence`, `is_fencei`, `is_amo`.

**Serialization.** `is_unique` (ROB drains before this uop executes), `flush_on_commit` (refetch after commit), `is_sys_pc2epc` (ECALL/EBREAK semantics — store PC into EPC).

**Exception state from frontend.** `xcpt_pf_if`, `xcpt_ae_if`, `xcpt_ma_if`, `bp_debug_if`, `bp_xcpt_if`; later `exception`, `exc_cause` carry decoded exceptions.

**FP control / NaN-boxing.** `fp_val`, `fp_single`.

**Helper methods.** `allocate_brtag()` (branches and JALRs that need a snapshot), `rf_wen()` (instruction writes a register), `unsafe()` (can mis-speculate — loads, stores except fences, branches, JALRs); these are queried by rename, ROB, and PNR logic.

**For Caracal:** adding RVV means extending this bundle with at least `pvs1`/`pvs2`/`pvs3`/`pvdst` + busy bits, `vtype`/`vl`/`vstart`/`vm`, `uses_vldq`/`uses_vstq` (if a vector LSU side-car is added), and a new `RT_VEC` for the rtype fields (likely requires widening rtype from 2 to 3 bits).

---

## 4. Pipeline overview

A single uop's life:

```
Fetch (F0..F4) → FetchBuffer → Decode → Rename(ren1, ren2) → Dispatch
              → IssueSlot (MEM/UNQ/ALU/FP) → RegisterRead
              → ExecutionUnit → Writeback → ROB.busy_clear
              → ROB head reaches uop → Commit → free stale_pdst
```

Side channels:
- **BPD training and FTQ deallocation** ride with commit.
- **Branch resolution** in execute broadcasts `BrUpdateInfo` (b1 = kill masks, b2 = oldest mispredict with target). All queues consume this within a cycle.
- **Exceptions** are stored in the ROB entry; the ROB raises `com_xcpt` at commit, which triggers a frontend redirect and rename rollback.

`exu/core.scala` is where this fan-in/fan-out is wired. Read it last — it makes far more sense after the individual stages.

---

## 5. Frontend (`ifu/`)

### 5.1 Fetch pipeline

`ifu/frontend.scala` implements a 5-stage fetch pipeline (F0..F4) with these responsibilities:

- **F0 — NextPC select.** Choose `s0_vpc` from reset vector / F1 prediction / F2-F3 prediction / late redirect. Issue request to ITLB and BPD.
- **F1 — ICache access + ITLB.** Translate VPC→PPC, kick off ICache data read. BPD F1 response available — a fast taken-branch redirect can re-aim F0 here.
- **F2 — ICache response.** Hit/miss resolved. Second early-redirect window from BPD F2.
- **F3 — Predecode + RVC expansion + RAS read + BPD F3 + SFB analysis.** This is the dense stage: per-slot branch decode, CFI index extraction, and FetchBundle assembly. SFB shadow computation happens here when `enableSFBOpt` is on.
- **F4 — Enqueue.** Drive FetchBundle into the FetchBuffer and FTQ in parallel.

The pipeline is decoupled at F3 by a small queue so the BPD F3 response can flow under back-pressure without stalling F0..F2.

### 5.2 Branch predictor stack

`ifu/bpd/predictor.scala` instantiates a top-level `BranchPredictor` per fetch bank (1 or 2 banks depending on `fetchBytes`). Each bank is a *composed* stack constructed via the `branchPredictor` factory in `BoomCoreParams` and built by `ComposedBranchPredictorBank` (`ifu/bpd/composer.scala`).

The TAGE-L default stack is, in priority order:
1. **TAGE** (`tage.scala`) — primary predictor, multiple tagged tables with geometric history.
2. **BIM** (`bim.scala`) — bimodal base predictor (fast fallback).
3. **BTB / SlowBTB / FAuBTB / GuBTB** (`btb.scala`, `slowbtb.scala`, `faubtb.scala`, `gubtb.scala`) — target prediction with varying associativity/latency tradeoffs.
4. **Loop predictor** (`loop.scala`) — predicts counted loop exits.
5. **RAS** (`ras.scala`) — return address stack.
6. **UBTB / hBIM / sw_predictor** — auxiliary tiers.

Each tier wraps the prior tier and may override its prediction in metadata; the FetchBundle carries the merged `bpd_meta` for training. Predictions arrive at F1/F2/F3 latency depending on tier; later predictions can redirect earlier stages.

### 5.3 Fetch Target Queue (FTQ)

`ifu/fetch-target-queue.scala`. The FTQ is the *single source of truth* for a fetch bundle's PC, CFI, predictor metadata, and RAS state once decode has consumed the bundle. Size `ftq.nEntries` (default 16). Each entry carries: `cfi_idx`, `cfi_taken`, `cfi_mispredicted`, `cfi_type`, `cfi_is_call`/`is_ret`, `cfi_npc_plus4`, `ras_top`, `ras_idx`, `br_mask`, `start_bank`, plus archived global+local history and BPD metadata in side SRAMs.

Allocation: F4 pushes one entry per fetch bundle. Deallocation: commit signals the oldest FTQ entry retire-ready, which simultaneously generates the BPD training update. On a mispredict-driven flush, FTQ snaps `enq_ptr` to the corrected index+1 and patches the mispredicting entry with the true CFI before raising training.

The FTQ also serves `get_ftq_pc` reads to the execute stage so JALR units and exception handlers can reconstruct full PCs from `(ftq_idx, pc_lob)`.

### 5.4 Fetch buffer

`ifu/fetch-buffer.scala`. A `numFetchBufferEntries`-deep elastic buffer that decouples fetch (potentially bursty across bank boundaries) from decode. Enqueue accepts up to `fetchWidth` instructions per cycle; dequeue presents `coreWidth` per cycle to decode. Flushed on any frontend redirect.

### 5.5 ICache

`ifu/icache.scala`. Standard set-associative cache (sets/ways set by `ICacheParams` from Chipyard). Single-ported when `icacheSinglePorted=true` (the default for ASIC tape-out friendliness). Refill over the tile's TileLink master node. `enableICacheDelay` adds a cycle on data read for timing closure. No prefetch hooks yet, though the SourceID space (1) is reserved for them.

### 5.6 Redirect sources

The frontend accepts redirects from four places, in priority order:
1. **ROB exception/CSR-write/sfence flush** (highest) — drives `redirect_flush=true` with target PC. Clears F1..F4, FetchBuffer, and (depending on cause) RAS.
2. **Execute-stage branch mispredict** — drives `redirect_val` with corrected PC and `redirect_ftq_idx`. Clears F1..F4 and FetchBuffer; FTQ patches the corresponding entry and trains BPD.
3. **In-stage BPD overrides** (F1 → F0, F2 → F0/F1, F3 → F0..F2) — fast re-aim of younger fetches without flushing downstream.
4. **Reset** (initial).

---

## 6. Decode

`exu/decode.scala`. One `DecodeUnit` per `coreWidth` slot. Each unit drives a `CtrlSigs.decode()` table lookup combining `XDecode` (base ISA), `X64Decode` or `X32Decode` (XLEN-specific), `FDecode` (FP, if `usingFPU`), and `RoCCDecode` (if `usingRoCC`). Each table row produces: `legal`, `fp_val`, `fp_single`, `uopc`, `iq_type`, `fu_code`, `dst_type`/`rs1_type`/`rs2_type`, `frs3_en`, `imm_sel`, `uses_ldq`/`uses_stq`, `is_amo`/`is_fence`/`is_fencei`, `mem_cmd`, `wakeup_delay`, `bypassable`, `is_br`/`is_sys_pc2epc`/`inst_unique`/`flush_on_commit`, `csr_cmd`.

Exception prioritization in `checkExceptions()`: interrupts, breakpoints, page faults, access faults, illegal instructions. `is_unique` and `flush_on_commit` are decided here based on uop class (CSR writes, fences, etc.).

To add a new instruction class, extend a decode table with new `BitPat → List(...)` rows. The "exactly one of each `iq_type`" constraint in `parameters.scala` means new uops either piggy-back on an existing queue or require adding a new queue type.

---

## 7. Rename

`exu/rename/`. The rename stage operates in two cycles (ren1, ren2):

- **ren1.** Read the maptable and allocate from the freelist. The just-allocated pdst gets bypassed to younger uops in the same group (same-cycle WAW forwarding within the rename group).
- **ren2.** Read the busytable for the now-allocated prs1/prs2/prs3; this populates the `prs*_busy` fields the issue slot needs. The 2-cycle separation gives the maptable write enough time to settle before ren2 reads.

### 7.1 Three rename pipelines

Three separate `RenameStage` modules run in parallel — `core.scala` instantiates them side by side:

| Pipeline | Pregs | Logical regs | Notes |
|----------|-------|--------------|-------|
| Integer  | `numIntPhysRegisters` | 32 (x0..x31) | `float=false` |
| FP       | `numFpPhysRegisters`  | 32 (f0..f31) | `float=true`; optional banked freelist via `enableBankedFPFreelist` |
| Immediate | `numImmPhysRegisters` | (synthetic) | v4 novelty — decouples immediate fields from the integer regfile; reduces IRF pressure |
| Predicate | `ftqSz` entries | (FTQ index space) | `PredRenameStage`; only when `enableSFBOpt` is on |

Each pipeline has its own:

- **MapTable** (`rename-maptable.scala`). One entry per logical register holding the current pdst. Branch snapshots: `br_snapshots[maxBrCount][numLregs]` captured on branch rename; restored verbatim on mispredict.
- **FreeList** (`rename-freelist.scala`). Bitvector of free pregs. `SelectFirstN(free_list, plWidth)` finds the next `plWidth` allocatable. Per-branch allocation lists `br_alloc_lists[maxBrCount]` track which pregs were allocated under each branch tag so they can be reclaimed on mispredict.
- **BusyTable** (`rename-busytable.scala`). Bitvector of in-flight pregs. Set on rename (rebusy), cleared on writeback (wakeup). Read at ren2.

### 7.2 Branch snapshots

When a branch or JALR is renamed (`allocate_brtag` is true), it gets a `br_tag` and *all three* rename pipelines snapshot their maptables under that tag. On `brupdate.b2.mispredict`, all three maptables restore from the tagged snapshot in one cycle, and freelists reclaim the per-tag allocation lists. `enableSuperscalarSnapshots` lets one snapshot capture a full rename group rather than a single uop, important at `coreWidth > 1`.

### 7.3 Wakeup ports into the busytable

Each completing instruction broadcasts `(valid, pdst)` to the busytable to clear the bit. The wakeup-port count must match exactly what `core.scala` aggregates — there's a `require(iss_wu_idx == numIntIssueWakeupPorts)` style invariant. Adding a new execution path means widening this fan-in.

---

## 8. Dispatch and issue queues

### 8.1 Dispatcher

`exu/dispatch.scala`. Two implementations:

- **BasicDispatcher.** All queues see `coreWidth` ports; stalls rename if *any* queue is full for *any* uop in the group.
- **CompactingDispatcher.** Queues advertise `dispatchWidth >= issueWidth`; a `Compactor` packs only the uops a queue actually accepts. Combined with `enableCompactingLSUDuringDispatch`, this lets a single backed-up queue not stall the others.

Routing is by bitmask: `dis_uops[i].valid := io.ren_uops[w].valid && ((io.ren_uops[w].bits.iq_type & issueParams[i].iqType.U) =/= 0.U)`. A uop with multiple `iq_type` bits set replicates into multiple queues (used for store-addr/store-data splits).

### 8.2 Issue queue types

Four queues, exactly one of each (the v4 expansion from v3's three):

| Type | Holds | Default issueWidth × entries |
|------|-------|------------------------------|
| `IQ_MEM` | Loads, stores, AMOs, fences | 2 × 8 (Small); up to 2 × 24 (Mega) |
| `IQ_ALU` | Integer ALU, branches, JALRs | 1 × 8 (Small); up to 4 × 24 (Mega) |
| `IQ_UNQ` | Unique / iterative / rare ops (mul, div, CSR, RoCC) | 1 × 8 — typically 1 unit |
| `IQ_FP`  | FP arithmetic | 1 × 8 (gated by `usingFPU`) |

The split of ALU vs. UNQ is the key v4 refinement: separating pipelined common-case ALU/branch traffic from long-latency, non-pipelined, or single-FU ops lets the ALU queue stay narrow and fast.

### 8.3 Issue unit implementations

Three implementations live under `exu/issue-units/`:

- **`issue-unit-age-ordered.scala`** — collapsing-array age-ordered queue. Older entries closer to position 0; on issue, younger entries shift down. Strict age priority; cost is O(N²) wiring. Default.
- **`issue-unit-banked.scala`** — column-banked variant for `enableColumnALUIssue`. Each bank is tied to one register-file bank and one ALU pipe; reduces RF read-port pressure.
- **`issue-unit-matrix.scala`** — matrix scheduler (commit `9cca341c`). Rows × columns layout; relaxes strict age order in exchange for higher issue rates. Currently the *fallback* path (`cf8e6d7a: fix: fall back to non-matrix issue for now`) means matrix is built but not the default — check `useMatrix`/`useBanked` toggles in `core.scala` for the live selection.

### 8.4 Issue slot mechanics

`issue-unit.scala` + `issue-slot.scala`. Slot states: `s_invalid`, `s_valid_1`, `s_valid_2` (the latter for store ops split into addr-gen + data-gen). Per-slot operand-ready flags `p1`, `p2`, `p3`, `ppred`. Wakeup paths:

- **Slow wakeup** from execution-unit writeback: matches `pdst` against `prs*`.
- **Fast wakeup** for bypassable ALU results, broadcast at issue time of the producer (1-cycle ahead of writeback).
- **Speculative load wakeup** (`enableFastLoadUse`): LSU signals `spec_ld_wakeup` when a load issues; dependents may issue speculatively but their operands are *poisoned* (`iw_p*_poisoned`) until cache hit is confirmed. On miss, the LSU signals `ld_miss` and poisoned dependents re-mark their operands busy and re-arbitrate.

The slot requests issue when valid, not killed by branch, and all `p*` are set. The issue unit arbitrates among requesting slots (oldest-first for age-ordered) and grants one per cycle per issue port.

---

## 9. Register files and register read

### 9.1 Register files

`exu/register-read/regfile.scala` — `RegisterFileSynthesizable`. Three files instantiated in `core.scala`:

| File | Size | Read ports | Write ports | Width |
|------|------|------------|-------------|-------|
| Integer (`iregfile`) | `numIntPhysRegisters` | `numIrfReadPorts` × `numIrfBanks` | `numIrfWritePorts` = ΣEUs.writesIrf + lsuWidth | `xLen` |
| FP (`fregfile`) | `numFpPhysRegisters` | `numFrfReadPorts` × `numFrfBanks` | per-FP-EU writes | `fLen+1` (NaN-boxed/recoded) |
| Immediate (`ipregfile`) | `numImmPhysRegisters` | mirrored to IRF readers | 1 (rename-time fill) | `LONGEST_IMM_SZ` |
| Predicate (`pregfile`) | `ftqSz` entries | 1 | 1 | 1 bit |

The integer file supports per-write-port "bypassable" tagging — bypassable writes appear on the bypass network combinationally, non-bypassable ones (load returns, mul, div, CSR) commit through the file. Banking (`numIrfBanks > 1`) is the alternative when read-port count would otherwise blow up the file; reads from each bank are then constrained to operands rename-allocated into that bank.

The **immediate physical register file** is v4-specific. Instead of widening the integer regfile with immediate-write ports, decode allocates an immediate preg in rename, writes the immediate value at rename time (the imm is known statically), and EUs read it through a dedicated read port. This decouples critical path: the imm file is much smaller and simpler, and the IRF doesn't need a write port per immediate-bearing uop.

### 9.2 Register-read stage

`exu/register-read/` — actually rolled into the regfile + bypass logic in v4 (no separate `register-read.scala` file). Per-issue-port: register addresses are emitted at issue; read data is available the next cycle; a bypass mux combines RF data with bypassable write-port forwards. RS3 (FMA third operand) is *not* bypassed — it must come from the RF.

The execute stage receives `(rs1, rs2, rs3, imm, pred)` plus the MicroOp.

---

## 10. Execution units

`exu/execution-units/`. Two layers:

### 10.1 `FunctionalUnit` (the unit of execution)

`functional-unit.scala`. Abstract base; subclasses declare `isPipelined`, `numStages`, `numBypassStages`, `dataWidth`, and capability flags. Two concrete bases:

- **`PipelinedFunctionalUnit`** — always-ready, latency-N pipeline. Branch-kill logic propagates `br_mask` through every stage.
- **`IterativeFunctionalUnit`** — holds one uop; `req.ready` is gated by FSM state. Branch-kill snapshots the held uop's `br_mask`.

Concrete FUs:

| Class | Type | Stages / Latency | Bypassable |
|-------|------|------------------|-----------|
| `ALUUnit` | Pipelined | 1 (configurable) | Yes |
| `MemAddrCalcUnit` | Pipelined | 0 (combinational) — or +1 if `enableAgenStage` | n/a |
| `PipelinedMulUnit` | Pipelined | `imulLatency` (3) | No |
| `DivUnit` | Iterative | ~10–60 (operand-dependent) | No |
| `FPUUnit` | Pipelined | `dfmaLatency` (= `sfmaLatency`) | No |
| `IntToFPUnit` | Pipelined | `intToFpLatency` (2) | No |
| `FDivSqrtUnit` | Iterative | ~20–60 | No |
| CSR handling | Inside `ALUUnit` | 1 | No (serialized) |

### 10.2 `ExecutionUnit` (the wrapper bound to an issue port)

`execution-unit.scala`. Each `ExecutionUnit` bundles one or more FUs behind a single issue-port interface. It advertises capability flags (`hasAlu`, `hasMul`, `hasDiv`, `hasMem`, `hasFpu`, `hasFdiv`, `hasIfpu`, `hasFpiu`, `hasCSR`, `hasJmpUnit`, `hasRocc`) and an `fu_types` UInt that is dynamically masked when an iterative FU is busy (so an in-flight DIV doesn't get dispatched another DIV). It exposes `iresp`, `fresp`, `ll_iresp`, `ll_fresp` for short- and long-latency writeback paths.

`ExecutionUnits` (the collection) decides how many EUs to instantiate based on issue widths and feature flags:

- For `intIssueParam.issueWidth`: one EU with `hasJmpUnit` (port 0), one with `hasCSR + hasRocc` (where applicable), one with `hasMul`, one with `hasDiv`, one with `hasIfpu` (if FPU), the rest pure ALU.
- For `memIssueParam.issueWidth`: that many `hasMem` units (no ALU).
- For `fpIssueParam.issueWidth` (inside `FpPipeline`): each FP EU has `hasFpu`; the first also has `hasFdiv`, `hasFpiu`.

### 10.3 The FP pipeline

`exu/fp-pipeline.scala` instantiates the FP issue queue, FP regfile, FP rename (separate from this file — driven from `core.scala`), and FP EUs as a self-contained subsystem. It exposes `dis_uops` (in), `wakeups` (out for rename), and `to_int` / `from_int` move-and-convert ports that arbitrate with the IRF write port via `ll_wbarb`.

Hardfloat is the underlying arithmetic library (recoded format with the +1 width). Loads from memory go through `recFNFromFN` on arrival; stores go through `ieee()` on the way out.

### 10.4 RoCC

`exu/execution-units/rocc.scala` — `RoCCShim`. RoCC instructions decode like normal uops and ride the `IQ_UNQ` queue. The shim owns two side queues:

- **RXQ** (`numRXQEntries`, default 4) — execute queue. Holds the instruction bits and (when available) `rs1`/`rs2` snapshots. An RXQ entry doesn't fire to the RoCC port until ROB PNR catches up (RoCC ops are non-speculative).
- **RCQ** (`numRCQEntries`, default 8) — commit queue. Holds the MicroOp awaiting a response. RoCC response data routes through `ll_iresp` and the response register matches the head of RCQ.

Branch-kill, exception, and flush all wind RXQ back to `rxq_com_head`. Once an RXQ entry has fired (i.e., is in RCQ), it is uncancellable — exceptions leave it in flight.

**This is the existing OoO-friendly co-processor handshake pattern**: dispatch-time allocation, PNR-gated execute, response-driven commit. Caracal's RVV will likely *not* use RoCC, but the dispatch/commit handshake is informative as a starting point if vector ops need any non-speculative phases.

---

## 11. Load-Store Unit (`lsu/`)

### 11.1 Top-level shape

`lsu/lsu.scala`. The LSU exposes `lsuWidth` request ports to the issue stage. Each request walks: AGEN (combinational or 1-stage with `enableAgenStage`) → TLB (`tlb.scala`) → LCAM (Load Compare-and-Match against the opposing queue) → DCache request. The two main queues:

- **LDQ** (`numLdqEntries`, default 8) — circular queue of in-flight loads.
- **STQ** (`numStqEntries`, default 8) — circular queue of in-flight stores.

### 11.2 LDQ entry

Fields: `addr` (vaddr then paddr), `addr_is_virtual`, `addr_is_uncacheable`, `executed`, `succeeded`, `order_fail`, `observed`, `st_dep_mask` (bitmask of older stores at allocation time), `youngest_stq_idx`, `forward_std_val` + `forward_stq_idx` (which store forwarded).

Allocation at dispatch (gated by `uses_ldq`); deallocation at commit (FIFO at head). Replay paths: `ldq_retry` for TLB-miss loads (oldest virtual-addr load), `ldq_wakeup` for nacked or unexecuted loads.

### 11.3 STQ entry

Fields: `addr`, `addr_is_virtual`, `data` (Valid — addr and data fill independently because of STA/STD split), `committed` (ROB has committed it; eligible to drain to DCache), `succeeded` (DCache acked).

Stores can dispatch as STA, STD, or fused. The store leaves the STQ only after both committed and succeeded — i.e., stores commit *speculatively* into the STQ at ROB commit and drain non-speculatively from STQ to DCache, in order.

### 11.4 Forwarding and ordering

The LSU performs a **Load Compare-and-Match (LCAM)** search every execute cycle:

- For each executing load, scan STQ for older stores with overlapping address → potential **St→Ld forward**. `ForwardingAgeLogic` picks the youngest older overlapping store with data ready. If overlap exists but data not ready, the load is stalled (`block_load_mask`).
- For each executing store, scan LDQ for younger loads with overlapping address that have *already executed* without forwarding from this store → **memory-ordering violation**. Such loads are marked `order_fail`, raising `MINI_EXCEPTION_MEM_ORDERING` at commit (full rollback to the load).

The two flags:
- `enableStLdForwarding` — STQ → load forwarding (the common case above).
- `enableLoadToStoreForwarding` — additionally allow a younger committed store's data to be observed by an older still-buffered store under certain coalescing patterns (rarely useful; conservative default is to leave it off and rely on STQ ordering).

There is no memory-disambiguation wait-bit yet (still on the TODO list).

### 11.5 DCache

`lsu/dcache.scala` — `BoomNonBlockingDCache`. Two layouts:

- **Duplicated** (`numDCacheBanks = 1`, default): full ports per `lsuWidth` request; no bank conflicts.
- **Banked** (`numDCacheBanks ≥ lsuWidth`): each bank single-ported. Conflicting same-bank accesses NACK; loser retries.

`mshrs.scala` — MSHR file. On miss, allocate an MSHR; the load (or any subsequent overlap) goes into the per-MSHR RPQ. On refill, RPQ replays through the cache pipeline. `dcacheSinglePorted` makes the data SRAM single-ported (ASIC-friendly) by serializing refill against demand reads.

### 11.6 TLB

`lsu/tlb.scala` — `NBDTLB` (Non-Blocking DTLB). Sectored + superpage entries + optional special PMP-only entry. On miss, PTW is invoked async; the load retries on completion. `sfence` invalidates per-ASID/per-page or wholesale.

### 11.7 AMO / LR-SC

AMOs go through the STQ but require both addr and data ready and must reach ROB head before firing to DCache. The DCache response brings back the AMO result, which routes via the LSU's load-data path to the integer write port. `useAtomicsOnlyForIO` restricts atomic semantics to I/O regions.

### 11.8 Prefetcher

`lsu/prefetcher.scala`. `NLPrefetcher` (next-line) when `enablePrefetching=true`; otherwise `NullPrefetcher`. Triggered off MSHR commits.

### 11.9 LSU ↔ ROB ↔ dispatch handshake

- **Dispatch**: ROB-side dispatch broadcasts `dis_uops` with `uses_ldq`/`uses_stq`; LSU allocates and reports back `dis_ldq_idx`/`dis_stq_idx`, which dispatch latches into the MicroOp.
- **Execute**: LSU consumes issue-port requests via the AGEN FU's address result.
- **Writeback**: load data appears on `iresp`/`fresp`; stores signal `clr_bsy` to free the ROB busy bit at addr+data ready.
- **Commit**: ROB's `commit.uops[w]` with `uses_ldq`/`uses_stq` set tells the LSU to advance its commit pointers; STQ stores then drain to DCache in FIFO order.

### 11.10 What changes for vector memops

The LSU is currently **strictly 1:1**: one uop, one LDQ/STQ entry, one cache request. RVV unit-stride, strided, indexed, and segment loads/stores break this assumption — a single vector load may produce many cache requests, may span cache lines, and (for indexed/segment) may need element-level ordering tracking. The integration choices:

- **Microcode at dispatch.** Break a vector memop into scalar uops at decode/dispatch — minimal LSU change, but explodes ROB / LDQ / STQ pressure.
- **Multi-entry allocation.** Reserve a contiguous range of LDQ/STQ entries for one vector uop. Forwarding/ordering LCAM logic widens to byte-mask-per-element.
- **Vector LSU side-car.** A separate VLDQ/VSTQ that interfaces with the DCache through dedicated ports and exposes a single ROB-level commit handshake. This is closer to bobtail's OVI approach (where the vector unit owned its own memory queues) and isolates the LSU from VLEN scaling.

Whichever, the touch points are: `MicroOp` (`uses_vldq`/`uses_vstq` and element-tracking fields), the dispatch allocation loop (`lsu.scala` lines around dispatch allocation), the LCAM mask widths, and any new MSHR allocation policy.

---

## 12. Reorder Buffer (`exu/rob.scala`)

### 12.1 Layout

`numRobEntries` arranged as `numRobRows = numRobEntries / coreWidth` rows × `coreWidth` banks. Per-bank arrays: `rob_val`, `rob_bsy`, `rob_unsafe`, `rob_uop`, `rob_exception`, `rob_predicated`, `rob_fflags`. Each rename group occupies one row (one bank per slot).

### 12.2 Pointers

- `rob_head` / `rob_head_lsb` — oldest uncommitted (commit pointer); LSB tracks partial-row commits.
- `rob_tail` / `rob_tail_lsb` — next free row; LSB tracks partial-row dispatch.
- `rob_pnr` / `rob_pnr_lsb` — Point of No Return: oldest *unsafe* uop. Once a uop is older than PNR, it can no longer be killed by a misprediction.

`enableFastPNR` swaps a slow scan for an `AgePriorityEncoder` over `rob_unsafe` — single-cycle PNR computation at the cost of priority-encoder area.

### 12.3 Allocation, writeback, commit

- **Allocate** (dispatch): write `rob_uop`, set `rob_val`, set `rob_bsy` (clear only for fences). The MicroOp's `rob_idx` is set at dispatch and consumed everywhere.
- **Writeback** (from EUs and LSU): clear `rob_bsy[row]`. Stores have a direct `lsu_clr_bsy` path. Loads can clear `rob_unsafe` separately once TLB checks pass even if data is still pending.
- **Commit**: `rob_val(head) && !rob_bsy(head) && !csr_stall` ⇒ commit; up to `coreWidth` per cycle subject to priority logic (earlier in row must commit for later to commit).

### 12.4 Branch mispredict

ROB walks all entries with `IsKilledByBranch(brupdate, rob_uop(i).br_mask)`; matching uops invalidate. `rob_tail` snaps to the row after the mispredicting branch (`WrapInc(GetRowIdx(brupdate.b2.uop.rob_idx), numRobRows)`). The surviving uops update their `br_mask` via `GetNewBrMask(brupdate, br_mask)`.

### 12.5 Exception flow

When a uop with `rob_exception=true` reaches head, ROB drives `com_xcpt.valid` to CSRFile, transitions to `s_rollback`, and walks `rob_tail` backward freeing entries (which the freelist consumes via the `stale_pdst` stream). Frontend redirected to xtvec target. `is_unique` uops route through `s_wait_till_empty` first to drain the ROB before they execute.

### 12.6 What changes for a new uop class

The ROB itself is largely uop-agnostic — it tracks busy/unsafe/exception flags and routes commits. Adding RVV mainly needs:
- `vtype`-changing instructions get `is_unique` so vector state changes serialize.
- Writeback ports for the vector EU(s) added to the `wb_resps` aggregation (and the corresponding `require(cnt == numWakeupPorts)` count updated).
- If vector ops can raise their own exceptions (alignment, illegal `vtype`), wire them into `rob_exception` from the right writeback path.

---

## 13. Top-level wiring (`exu/core.scala`)

The key signal-level structure in `BoomCore`:

1. **Instantiation block** (top of `core.scala`):
   - `decode_units: Seq[DecodeUnit]` (size `coreWidth`).
   - `rename_stage`, `fp_rename_stage`, `imm_rename_stage`, `pred_rename_stage` (the last gated on `enableSFBOpt`).
   - `dispatcher` (BasicDispatcher or CompactingDispatcher).
   - `int_iss_unit`, `mem_iss_unit`, `unq_iss_unit` (and FP issue inside `fp_pipeline`).
   - `iregfile`, `ipregfile`, `pregfile`; `fp_pipeline` owns `fregfile`.
   - `exe_units = new ExecutionUnits(fpu=false)` for the integer side; `fp_pipeline.exe_units` for FP.
   - `lsu_io` connected to the LSU instantiated in `BoomTileModuleImp`.
   - `rob`.
   - `ll_wbarb` — arbiter for the shared long-latency IRF write port (used by load returns, FP→I moves, RoCC responses).

2. **Wakeup port aggregation** is the trickiest piece. For each integer EU that writes IRF:
   - If the EU is bypassable, a *fast wakeup* port is allocated (broadcasts at issue of producer).
   - A *slow wakeup* port is also allocated unless the EU is `alwaysBypassable`.
   - Both feed the integer issue units (`int_iss_unit`, `mem_iss_unit`, `unq_iss_unit`) and the integer busytable in `rename_stage`.
   - The total count must equal `numIntIssueWakeupPorts` (a derived constant); `require(iss_wu_idx == numIntIssueWakeupPorts)` enforces it.

   FP wakeups are entirely internal to `fp_pipeline`, fed to `fp_rename_stage.io.wakeups`. Cross-domain wakeups (FP→I moves) ride `ll_wbarb`.

3. **Branch resolution fan-in**. Every ALU EU produces a `BrResolutionInfo`; `core.scala` reduces them to a single `BrUpdateInfo` (with the oldest mispredict selected) and broadcasts to: IFU, FP pipeline, all EUs, LSU, ROB, every issue unit's slots, every rename pipeline.

4. **Commit fan-out**. `rob.io.commit` drives: CSR (retire count, fflags), LSU (advance commit pointers), all three integer rename stages (free `stale_pdst` to freelists), FP rename, trace.

5. **Redirect fan-out**. `rob.io.flush` and execute mispredict drive `io.ifu.redirect_*` (the frontend) and trigger maptable rollback via branch-tag snapshots.

The `core.scala` `require()` statements form a checklist of structural invariants — any change that adds/removes EUs, issue ports, regfile ports, or wakeup ports will trip them, which is by design.

---

## 14. Extension points for adding instruction support

Adding a new instruction class (the main Caracal use case for RVV) is a multi-file change. The checklist:

### 14.1 Always touched

1. **`common/consts.scala`** — add new `uopXXX` codes, possibly new `FU_VEC`, possibly new `IQ_VEC`, possibly new `RT_VEC` (widening `*_rtype` if needed).
2. **`exu/decode.scala`** — add new decode table rows mapping `BitPat → List(...)`. Set `iq_type`, `fu_code`, register types, control signals.
3. **`common/micro-op.scala`** — add fields for any new state the uop must carry (e.g., `vtype`, `vl`, vector preg fields).
4. **`common/parameters.scala`** — add `numXxxPhysRegisters` and other sizing knobs; relax or extend the `require()` invariants if you're adding an issue queue or register class.
5. **`common/config-mixins.scala`** — add a `WithVector` (or feature-specific) mixin that flips the new parameters on.

### 14.2 If adding a new register class (e.g., vector regfile)

6. **`exu/rename/`** — instantiate a fourth `RenameStage` in `core.scala`; the existing module is generic over preg count and logical count. Wire its `wakeups`, `brupdate`, `dealloc_pregs`, etc. just like the integer/FP pipelines.
7. **`exu/register-read/regfile.scala`** — add a new `RegisterFileSynthesizable` instance, sized appropriately. Consider banking for wide vector regfiles (VLEN can be 128+ bits).
8. **Update `MicroOp`** with `pvX` fields and `pvX_busy` fields.

### 14.3 If adding a new issue queue type

9. **`common/parameters.scala`** — relax `require(issueParams.count(_.iqType == X) == 1)` to include the new type, and add a require for the new type.
10. **`exu/dispatch.scala`** — no change to routing logic itself (it's bitwise-AND-driven), but new queue must be added to `issue_units`.
11. **`exu/core.scala`** — instantiate the new issue unit, wire its `wakeup_ports` into the aggregation, and route the new EU's `iresp` to the IRF (or a new vector RF) write ports.

### 14.4 If adding a new execution unit

12. **`exu/execution-units/`** — define a new `FunctionalUnit` subclass (pipelined or iterative; latency declared). Wrap it in an `ExecutionUnit` subclass advertising the right capability flags and write-port destinations.
13. **`exu/execution-units/execution-units.scala`** — extend `ExecutionUnits` collection logic to instantiate the new EU when the corresponding parameter is set.
14. **`exu/core.scala`** — add the EU's writeback to `wb_resps` aggregation; bump the writeback / wakeup port counts to match the `require()`s.

### 14.5 If adding new memory operations

15. **`lsu/lsu.scala`** — extend dispatch allocation (`uses_ldq`/`uses_stq` are currently 1-bit; vector memops may need multi-entry allocation or a side-car queue).
16. **`lsu/dcache.scala`** — for very high-bandwidth vector memops, consider whether the MSHR file and bank conflicts can sustain the request rate.
17. **`exu/rob.scala`** — if vector memops can raise unique exceptions, wire them in.

### 14.6 If serialization is needed

18. Set `is_unique` and/or `flush_on_commit` in the decode table for any instruction that mutates state subsequent instructions depend on (e.g., `vsetvli`).
19. The ROB FSM (`s_wait_till_empty`) already handles `is_unique` — no ROB-side change.

### 14.7 What's *not* a touch point

The branch tag system, ROB allocation, frontend, and most of the LSU's load/store ordering logic are uop-agnostic — they work off `br_mask`, `uses_ldq/stq`, `rob_idx`, and the busy/unsafe flags, none of which depend on uop class.

---

## 15. Glossary

- **AGEN** — Address generation; the rs1+imm computation for memory ops.
- **BPD** — Branch Predictor Directory; the composed prediction stack.
- **BR tag** — Branch tag; identifies one in-flight speculative branch for snapshot/rollback.
- **CFI** — Control-Flow Instruction (branch, JAL, JALR).
- **EU** — Execution Unit; one issue-port wrapper around one or more FUs.
- **FU** — Functional Unit; one ALU / mul / div / mem-addr-calc / FPU / etc.
- **FTQ** — Fetch Target Queue; per-fetch-bundle metadata, source of PC reconstruction.
- **IQ** — Issue Queue. v4 has four: `IQ_MEM`, `IQ_ALU`, `IQ_UNQ`, `IQ_FP`.
- **LCAM** — Load Compare-and-Match; LSU's per-cycle scan of LDQ vs. STQ.
- **LDQ / STQ** — Load Data Queue / Store Queue.
- **MSHR** — Miss Status Handling Register; one outstanding cache miss.
- **NBDTLB** — Non-Blocking DTLB.
- **PNR** — Point of No Return; oldest unsafe uop in the ROB. Older uops are committed-or-pending-commit, younger uops are speculatively killable.
- **RCQ / RXQ** — RoCC Commit Queue / RoCC Execute Queue.
- **RPQ** — Replay Queue (inside an MSHR).
- **RT_FIX / RT_FLT / RT_X / RT_PAS** — register types (integer / FP / none / passthrough).
- **SFB** — Short Forward Branch optimization; predicate-folds tiny forward branches.
- **uop** — micro-op; `MicroOp` bundle instance.

---

*This document reflects BOOM v4 as of the `Caracal/addvector` branch. Update sections 7–14 as Caracal vector work introduces new pipelines, queues, or register classes.*

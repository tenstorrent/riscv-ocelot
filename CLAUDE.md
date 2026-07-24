# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project: Caracal

Caracal is a new project that builds on the BOOM v4 core to integrate **out-of-order RVV 1.0** vector instruction execution. The current `Caracal/initial` branch is just the project-setup baseline — it forks BOOM at the point where the v3 + v4 cores live side-by-side, with no vector support yet.

Branching model:
- `Caracal/main` — integration branch for all Caracal work.
- `Caracal/<feature>` — feature branches. PRs target `Caracal/main`.
- `Caracal/initial` — starting baseline (the current branch); no Caracal work lives here.
- `bobtail/main` — **reference only**. The Bobtail/Bobcat team integrated RVV into the *older* BOOM (pre-v4) by attaching a vector unit through an OVI (Open Vector Interface) co-processor. When designing Caracal features, consult `bobtail/main` to see how a prior team solved the same plumbing problem, but do not assume their approach is the right one — Caracal targets true OoO RVV inside BOOM v4, not an OVI side-car. Useful files on that branch: `src/main/scala/exu/execution-units/ovi.scala`, `src/main/scala/exu/ovi_wrapper/*`, `WithVector` mixin in `src/main/scala/common/config-mixins.scala`.

## Repository structure

This repo is **not self-runnable**. It's a Chisel library consumed by [Chipyard](https://github.com/ucb-bar/chipyard), which is the only supported way to elaborate, simulate, or build a SoC around the core. The Chipyard commit this repo is pinned against is in `CHIPYARD.hash`.

Two BOOM generations live side-by-side under `src/main/scala/`:
- `v3/` — BOOM v3 (SonicBOOM). Kept for compatibility.
- `v4/` — **BOOM v4. Caracal work happens here.** All file paths below are relative to `src/main/scala/v4/`.

Subdirectories within each version mirror the pipeline:
- `common/` — `parameters.scala` (`BoomCoreParams`, `IssueParams`), `config-mixins.scala` (`WithNSmallBooms`/`Medium`/`Large`/`Mega`/`Giga`, BPD selectors), `micro-op.scala` (the `MicroOp` bundle that flows through the pipeline), `tile.scala` (`BoomTile`, diplomacy attach), `types.scala`, `consts.scala`.
- `ifu/` — frontend: `frontend.scala`, `icache.scala`, `fetch-buffer.scala`, `fetch-target-queue.scala`, and the branch predictor stack under `bpd/` (TAGE, BIM, BTB variants, RAS, loop predictor, composer).
- `exu/` — execution: `core.scala` (top-level pipeline wiring), `decode.scala`, `dispatch.scala`, `rob.scala`, `fp-pipeline.scala`, plus `rename/`, `issue-units/` (age-ordered, banked, matrix), `register-read/regfile.scala`, and `execution-units/` (`functional-unit.scala`, `execution-unit.scala`, `fpu.scala`, `fdiv.scala`, `rocc.scala`).
- `lsu/` — `lsu.scala`, `dcache.scala`, `mshrs.scala`, `tlb.scala`, `prefetcher.scala`, `tracegen.scala`.
- `util/` — shared helpers (`util.scala`, `elastic-reg.scala`, `elastic-sram.scala`, `seqmem-transformable.scala`, `ParallelFindOne.scala`).

Package names are `boom.v4.{common,ifu,exu,lsu,util}`. When adding a Caracal feature, stay inside `v4/` unless you have a specific reason to touch `v3/`.

For a detailed walkthrough of the v4 pipeline (frontend, rename, dispatch, issue, regfiles, EUs, LSU, ROB, wiring, and extension points for new instructions/register classes), see `docs/boom-v4-architecture.md`. Update that doc when Caracal work adds new pipelines, queues, or register classes.

### Issue-queue topology (relevant for RVV planning)
`BoomCoreParams.issueParams` declares the set of issue queues. The v4 baseline has exactly four: `IQ_MEM`, `IQ_UNQ`, `IQ_ALU`, `IQ_FP`, and `parameters.scala` enforces "exactly one of each" via `require(...)`. Adding a vector issue queue means relaxing those `require`s, threading a new `IQ_VEC` (or similar) through dispatch/rename/ROB, and wiring a new execution unit — this is not a localized change.

## Build & checks

Build is driven by either SBT or Mill (both `build.sbt` and `build.sc` are kept in sync — Chisel 6.5.0, Scala 2.13.14, depends on `rocketchip-$chiselVersion` 1.6-SNAPSHOT). Because the core is elaborated by Chipyard, you almost never run `sbt run` here directly; instead:

- **Style check** (the only first-party check target):
  ```
  make checkstyle
  ```
  Runs `sbt scalastyle test:scalastyle`. Rules live in `scalastyle-config.xml` — note the mandatory copyright/license header regex and the 120-col limit.
- **Compile-only sanity check** (useful before pushing): `sbt compile`. Full elaboration, simulation, and tests happen from a Chipyard checkout pointing at this repo as a submodule.
- **csmith fuzzing** harness lives under `util/csmith/` (`install-csmith.sh`, `run-csmith.sh`). It's invoked from Chipyard, not standalone.

There is no in-repo unit-test suite. The `src/test/` Scala tests that existed pre-Caracal were removed; do not add new ones here without first checking that the equivalent flow doesn't already live in Chipyard.

## Coding conventions (from `CONTRIB_AND_STYLE.md`)

- Scala identifiers: `lowerCamelCase`. Chisel signal/wire names: `lower_snake_case`. Classes/objects: `UpperCamelCase`.
- Prefix pipeline-stage signals with their stage: `val f2_valid = ...`, `val s1_uop = ...`.
- Braces: same line for `if`/`for`/`while`/`when`/defs; **next line** for `class`/`object`. `else`/`.otherwise` go on the same line as the preceding `}`.
- Two-space indent, no tabs (enforced by scalastyle).
- Every file begins with the SiFive/Regents copyright header — scalastyle will reject files that don't match the regex in `scalastyle-config.xml`.

 ## Chisel language reference

For any Chisel syntax or language-implementation questions (Bundle composition, `Mux`/`MuxCase`, `SyncReadMem` vs `Mem`, `Decoupled` handshake
semantics, `withClockAndReset`, `chisel3.util` helpers, etc.), consult the official Chisel docs at https://www.chisel-lang.org/docs and fix any
syntax/elaboration issues against that reference. Do not guess at Chisel APIs from training data, confirm against the docs.


## Notes for future work

- `TODO.md` lists pre-existing BOOM TODOs (memory-disambiguation wait-bit, hit-under-miss icache, prefetchers, under-provisioned RF ports). These predate Caracal and remain open.
- The upstream BOOM docs under `docs/sections/` describe the v3-era pipeline; treat them as background, not as a spec for v4 internals.

# Bobtail OVI load/store generators (reference only)

These eight files are the **bobtail OVI-era** vector load/store address-generation
reference material:

- `loadgen.scala`, `loadpacker.scala`, `loadskipper.scala`, `loadwalker.scala`
- `storegen.scala`, `storepacker.scala`, `storeskipper.scala`, `storewalker.scala`

They implement the bobtail **Packer / Skipper / Walker** address-generation
algorithms that the prior OVI co-processor design used to expand a vector
load/store into element/segment accesses.

## Why they live here and not in the source tree

`sbt`/`mill` only compile `src/main/scala`. These files were moved **out** of the
compiled tree (via `git mv`) because they do **not** compile as-is in Caracal:

- They declare `package boom.exu`, which does not exist in BOOM v4 (v4 uses
  `boom.v4.{common,ifu,exu,lsu,vec.*}`).
- They reference undefined symbols `VecLSGenConstants` and `ConfigInfo` that
  were part of the bobtail OVI environment and have no Caracal equivalent yet.

Leaving them under `src/main/scala/v4/vec/lsu/lsgen/` broke the build.

## What they are kept for

Reference for two later Caracal milestone steps:

- **Step 7 (`ConfigInfo`)** — the per-instruction configuration descriptor the
  generators consume.
- **Step 10 (`VecAgenStage1` / `VecDgen`)** — the Caracal vector LS AGEN /
  data-gen datapath.

When reworked, they must be brought into package `boom.v4.vec.lsu` and split per
the milestone plan's **two-stage Skipper + Walker / JIT-Packer** design, rather
than ported verbatim from the OVI single-stage form.

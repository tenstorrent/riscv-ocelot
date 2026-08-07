# Gate (f) — the re-baselined reference

Plan v2 §6 gate (f) originally read *"a `usingRVV=false` build is bit-identical to
pre-Caracal BOOM v4"*. Decision **D1** relaxed it, because it is not achievable:
`ScalarOpConstants` is a bare Scala `trait` with no `Parameters` in scope, so it widens
`RT_*` and `IQ_SZ` **unconditionally**, and `MicroOp`/`Rob` must follow.

The gate now reads: *identical to the re-baselined reference **except for the enumerated
encoding widths below***. This directory is the machinery that makes that statement
falsifiable instead of asserted — which was the explicit obligation D1 created:

> A new plan step, analogous to A0, must generate and check in the re-baselined
> reference. Without it "identical except the enumerated widths" is unfalsifiable, and
> every later step's bit-identity claim rests on it.

## The enumerated exception — and nothing else

- `RT_FIX` / `RT_FLT` / `RT_X` / `RT_ZERO`: `UInt(2.W)` → `UInt(3.W)` (values 0..3 unchanged)
- `IQ_SZ`: 4 → 7, hence `MicroOp.iq_type` 4 → 7 bits
- `MicroOp.dst_rtype`, `lrs1_rtype`, `lrs2_rtype`: 2b → 3b
- `Rob`'s compact `dst_rtype` tracks `MicroOp`'s width

Any other difference is a gate (f) failure.

## Why there are *two* artifacts

D1 asks for a reference "with the widened encodings and no vector logic". On its own that
is an artifact nobody can audit: it already contains the exception, so diffing a later
build against it cannot tell you whether the widening itself dragged anything else along.
So there are two:

| Artifact | boom source | Role |
|---|---|---|
| `prebaseline` | **before** the D1 widening (`Caracal/addvector2`, no `v4/vec/`) | the anchor |
| `rebaseline`  | **after** the D1 widening, still no vector logic (plan step A2) | what gate (f) diffs against thereafter |

The check that discharges D1 is therefore:

```
gate-f-check.py --pre  sims/vcs/generated-src-gatef-prebaseline \
                --post sims/vcs/generated-src-gatef-rebaseline
```

and it must report **every** difference as the enumerated exception. That is the step that
proves the widening is bounded. From then on, every later Phase A/B step re-runs the same
check with `--post` pointing at a fresh vectors-off build, against `rebaseline`.

**Ordering consequence:** `prebaseline` can be generated today. `rebaseline` cannot — the
widening currently exists only in the `edit_existing` nlhdl specs under
`src/main/nlhdl/pkg/`, not in `src/main/scala`. It is generated when step A2 lands. Until
then this directory holds the anchor and the tooling, and `manifest/rebaseline.json` is
absent by design.

## Configs covered

`SmallBoomV4Config`, `MediumBoomV4Config`, `MegaBoomV4Config` — `decodeWidth` 1 / 2 / 4 and
`memWidth` 1 / 1 / 2. Width-dependent regressions are the plausible failure mode for an
encoding change, so the gate spans the tiers rather than checking the default config only.
(`LargeBoomV4Config` sits between Medium and Mega and is omitted for cost; override with
`CONFIGS='...' ./regen.sh`.)

Note these are the *plain* V4 configs. Rule 4 restricts the **vector** matrix to
Medium/Large/Mega and drops Small — that is a separate list, and Small is still a supported
vectors-off config that gate (f) must protect.

## What `regen.sh` neutralizes, and why it must

Caracal added two things to chipyard's `generators/chipyard/src/main/scala/config/BoomConfigs.scala`
that a pre-vector boom cannot compile against:

1. `boom.v4.common.WithBoomDebugHarness` — the whisper-cosim DPI harness. Mixed into the
   plain `SmallBoomV4Config`/`MediumBoomV4Config`/`LargeBoomV4Config`/`MegaBoomV4Config`
   too, so its absence breaks *every* config, not just the vector ones.
2. `class *Vector*Config` — reference `boom.v4.vec.common.VectorParams`.

`regen.sh` strips both, elaborates, and restores the file via an `EXIT` trap. The two rules
are mechanical and there are deliberately no others.

This matters for the comparison, not just for compilation: **both** artifacts must be
generated with the same chipyard-side config content, or the pre↔re diff mixes the encoding
delta with a config delta and the gate stops meaning anything. Hence the rule — *the gate-f
config set is the plain BOOM v4 configs with Caracal's chipyard-side mixins removed* — and
it applies to `rebaseline` as much as to `prebaseline`, even once `BoomConfigs.scala`
compiles again on its own.

## How the check works

Widening `dst_rtype` renumbers every bit position downstream of it in every bundle that
packs a `MicroOp` — ROB entries, issue slots, queue payloads. A textual diff therefore
reports thousands of changed subscripts that are all consequences of the exception, and
buries the one line that matters. So the check is split three ways:

- **Module set (strict).** Neither side may gain or lose a module. This is the "and no
  vector logic" half of D1, and it is the sharpest check here.
- **Tier 1 (strict).** A module mentioning neither `rtype` nor `iq_type` cannot be affected
  by the exception, so its normalized text must be *equal*. Any difference is a violation.
  This covers the majority of modules and is a true equality check.
- **Tier 2 (structural).** For modules that do carry the widened fields, text is expected to
  differ. What must not differ is the module's shape: same ports, same submodule instances,
  same statement histogram. Every width change must land on an allowlisted name (`rtype`,
  `iq_type`) *and* be exactly the enumerated widening — an allowlisted name with an
  unexpected width delta is still a violation.

Normalization strips `@[File.scala 12:34]` source locators (the widening shifts Scala line
numbers, which would otherwise dirty nearly every line), the `firtool` version banner, and
assertion string literals that embed `file:line`.

### Limitation, stated rather than glossed

Tier 2 detects added or removed structure, not rewritten expressions: logic could in
principle be altered while port list, instance list and statement counts all hold. Tier 1 is
a genuine equality check but by construction does not cover the modules the exception
touches. A passing gate is strong evidence, not proof.

Separately, and noted in D1 as **A23**: a passing gate (f) does **not** prove the three new
`IQ_V_*` `iq_type` bits are explicitly defaulted. `DecodeUnit` does `uop := io.enq.uop`
where `io.enq.uop` comes from a bundle the frontend sets `:= DontCare`, so without six
explicit defaults every *scalar* uop carries don't-care vector routing bits into dispatch.
A don't-care bit can elaborate bit-identically and still mis-route. That needs its own
check; gate (f) will not catch it.

## Reproducibility

`manifest/*.json` records the repo and boom SHAs the artifact was generated from, a
per-module normalized hash, and a whole-tree hash. The Verilog itself is **not** checked in
(620+ modules × 3 configs); `regen.sh` reproduces it from the recorded SHAs instead.

This rests on Chisel + firtool elaboration being deterministic for a fixed source tree.
**Verified, not assumed:** `SmallBoomV4Config` was elaborated twice from the same tree and
produced the identical normalized `tree_sha256` `25b82890f43ce8e8`. If `regen.sh` on an
unchanged tree ever produces a differing hash, that property has broken and the manifest —
not the tree — is what to distrust.

Recorded pins for `prebaseline`: repo `48f7c218`, boom `2d7cf02e` (branch
`Caracal/addvector2`), `firtool-1.75.0`, `src/main/scala` clean. Note that boom's
`src/main/scala` at `2d7cf02e` is byte-identical to `09974f14` ("Merge pull request #8 from
tenstorrent/Caracal/initial") — `addvector2`'s three commits add only docs, nlhdl specs and
`src/main/resources`. So the anchor really is pre-vector Caracal BOOM v4.

## Usage

```bash
cd generators/boom/docs_caracal/v2-rebaseline
S=../../../../sims/vcs                 # where the trees land

./regen.sh prebaseline                 # anchor; generatable today
./regen.sh rebaseline                  # only after plan step A2 lands the widening

# did a regeneration reproduce the recorded artifact? (the Verilog is not checked in,
# so this is how the manifest earns its keep)
./gate-f-check.py --verify $S/generated-src-gatef-prebaseline \
                  --against manifest/prebaseline.json

# discharge D1: prove the widening is exactly the enumerated exception
./gate-f-check.py --pre  $S/generated-src-gatef-prebaseline \
                  --post $S/generated-src-gatef-rebaseline -v

# every later vectors-off build is checked against the re-baseline
./gate-f-check.py --pre $S/generated-src-gatef-rebaseline --post <fresh>

./selftest.sh                          # run after touching gate-f-check.py
```

`CONTAINER=<name>` overrides the podman container (default `reverent_turing`);
`CONFIGS='A B'` overrides the config list and rebuilds only those, leaving the other trees
and their manifest entries intact. Exit status is 0 on pass, 1 on violations.

**`regen.sh` mutates a tracked source file** (`BoomConfigs.scala`) and restores it via an
`EXIT` trap, so it takes an `flock` and refuses to run concurrently — two overlapping runs
would have the second back up the first's already-neutralized file and "restore" that, leaving
the repo uncompilable. It also refuses to start if the file is *already* neutralized, which
means a previous run died without restoring; the fix it prints is
`git checkout generators/chipyard/src/main/scala/config/BoomConfigs.scala`.

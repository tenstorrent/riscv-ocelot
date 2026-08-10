# Mode: validate — check the ledger, the checkers, and the build

Invocation: `/req-formal-chisel validate [<family> …] [--no-build]`

Three layers of check, cheapest first. Run them in order and stop reporting a
later layer as passing if an earlier one failed.

---

## Layer 1 — the ledger (mechanical, always run)

```bash
python3 req-formal-chisel/scripts/validate-formal.py [<family> …]
```

Exit `0` = no errors, `1` = errors, `2` = bad invocation. It only ever reads.

What it checks:

| Check | Why it matters |
|---|---|
| `family:` matches filename and `families.yaml` | a ledger for a family that does not exist |
| ID form `formal-<family>.<group><n>`, unique, group letter matches its reqs' group | drifting IDs break tags |
| Every `reqs:` entry is a **live** requirement in `reqs_source` | catches tags pointing at retired/typo'd IDs |
| Every live requirement appears in `coverage:` exactly once | **the gap check** — the one thing no other layer can see |
| Each `coverage:` row has `assertions:` xor `unassertable:` | a row with neither is silent non-coverage |
| Each `assertions:` row cites ≥1 `kind: assert` | covers alone never discharge a requirement |
| Every `\|->` / `\|=>` assert has `reachability:` pointing at a real cover | vacuity |
| Every `kind: assume` has `justification:` | a wrong assume makes the whole file vacuous |
| Non-cover assertions have a non-empty `reqs:` | stops unlabelled "while I was here" properties |
| Labels unique across the family, and none reused from `retired:` | coverage-DB keys |
| Ledger `assertions:` ↔ `//@formal-req-` tags in the checker files agree **both ways** | the plan/claim diff |
| Every `checkers:` `file:` exists and defines `object <name>` | ledger rows for files nobody wrote |
| Each `dut_file:` contains exactly one `//@formal-anchor <name>` | a checker written but never bound in — compiles clean, checks nothing |
| No `//@req-` tag in any checker file | keeps *checks* out of `/nlhdl`'s *implements* namespace |

`--plan-only` skips every check that needs the `.scala` files, for use between
`plan` and `gen`.

---

## Layer 2 — elaboration and the bind (needs the container)

The properties existing in Scala proves nothing about the emitted Verilog. This
layer is what proves the `bind` actually happened.

```bash
CFG=MediumBoomV4VectorConfig
podman exec <container> bash -lc "set -o pipefail; \
  source /opt/conda/etc/profile.d/conda.sh && source /root/my-chipyard/env.sh && \
  cd /root/my-chipyard/sims/vcs && make CONFIG=$CFG verilog 2>&1 | tail -20"
```

Then, in `sims/vcs/generated-src/chipyard.harness.TestHarness.$CFG/gen-collateral/`:

| Expect | Command | A failure means |
|---|---|---|
| one `layers_*_BoomSvaLayer.sv` per bound top | `ls layers_*BoomSvaLayer*.sv` | the layer was stripped — check for `--disable-layers` in the firtool line |
| a `bind <Dut> <Dut>Checks…` statement in it | `grep -h '^bind' layers_*BoomSvaLayer*.sv` | the anchor never elaborated |
| every ledger `label:` present | `grep -ho '^\s*[a-z0-9_]*: \(assert\|assume\|cover\) property' *_BoomSvaLayer.sv` | a property was dropped or renamed |
| **no property folded to a constant** | `grep -nE '(assert\|cover) property.*\((1'"'"'h[01]\|1'"'"'b[01])\);' *_BoomSvaLayer.sv` | tautology — the property is checking nothing. Treat as an error, not a warning. |
| assertion text **absent** from the DUT's own `.sv` | `grep -c 'assert property' <Dut>.sv` → `0` | the bind did not take; properties inlined |

The tautology grep is the highest-value check here. A property built on a
misremembered signal, or one whose antecedent contradicts itself, constant-folds
during lowering and emits `assert property (@(posedge clock) disable iff (x) 1'h1)`
— which passes every run, forever, and looks like coverage in every report.

---

## Layer 3 — simulation (needs the binds on the compile line)

**`layers_*.sv` is not in `filelist.f`.** firtool deliberately omits bind files
from the filelist, and chipyard builds its simulation source list from
`$(ALL_MODS_FILELIST)`, which derives from it (`common.mk:308-316`). So a default
build compiles the checker modules and never binds them: **the build succeeds and
checks nothing.** This is the single most likely way for this skill's output to
silently do nothing.

Add them with `EXTRA_SIM_SOURCES`, which both `sims/vcs/Makefile` and
`sims/verilator/Makefile` append straight to the compiler command line. Quote it
so make expands the wildcard when the recipe runs, after elaboration:

```bash
make CONFIG=$CFG -j$(nproc) debug \
  EXTRA_SIM_SOURCES='$(wildcard $(GEN_COLLATERAL_DIR)/layers_*.sv)'
```

VCS already passes `-assert svaext` (`sims/vcs/vcs.mk:51`), so no other flag is
needed for the SVA itself.

Then confirm the binds are live in a real run — a property that compiles but is
never sampled is still nothing:

```bash
OD=$(pwd)/output/chipyard.harness.TestHarness.$CFG
make CONFIG=$CFG $OD/dhrystone.riscv.out EXTRA_SIM_SOURCES='…'
grep -iE 'assert|Offending' $OD/dhrystone.riscv.log
```

### Verilator is not a validation path

Verilator 5.022 in this container rejects `##` delays and `s_eventually` outright
(`%Error-UNSUPPORTED`). Boolean-only properties and bare `|->`/`|=>` lint clean;
anything temporal does not. Do not gate a Verilator build on this skill's output,
and never report a Verilator pass as evidence the properties work.

---

## Reporting

Report per family:

```
formal-decode: 115 reqs → 98 asserted (14 partial), 17 unassertable
               142 assertions (96 assert / 38 cover / 8 assume) across 6 checkers
               layer 1 PASS   layer 2 PASS   layer 3 not run
```

Then, individually: every `unassertable:` reason (they are the claims most worth a
second opinion), every `partial:`, every property that failed on real RTL with its
counterexample, and every tautology. Never average a partial into a coverage
percentage — report the three buckets separately.

If a property fails, report the failure and stop. Do not weaken it; see the
mandatory stops in `SKILL.md`.

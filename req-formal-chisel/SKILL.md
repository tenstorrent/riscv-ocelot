---
name: req-formal-chisel
description: Generate Chisel SVA assertions and functional coverage from requirement YAMLs. Use when turning src/main/nlhdl/reqs/spec-<family>.yaml plus a Chisel source into bound-in properties — planning which assertion covers which requirement (plan), writing the checker files and binding them (gen), checking the ledger against the code and the build (validate), or answering which assertion covers a requirement (trace).
---

# req-formal-chisel — Requirements → Chisel SVA + coverage SKILL

This skill turns **requirements into properties**. Given a feature family's
`spec-<family>.yaml` and the Chisel that implements it, it produces:

1. `src/main/nlhdl/formal/formal-<family>.yaml` — the ledger: every live
   requirement mapped to the assertions and cover statements that check it.
2. `src/main/scala/v4/vec/formal/<Module>Checks.scala` — the properties
   themselves, in a file **separate from the DUT**.
3. One `layer.block(BoomSvaLayer) { … }` anchor per checked module, which makes
   firtool emit the properties into their own `.sv` and hook them up with a
   SystemVerilog **`bind`** — no assertion text inside the generated core RTL.

```
src/main/nlhdl/reqs/spec-<family>.yaml       the requirements  — the authority
src/main/nlhdl/formal/formal-<family>.yaml   the ledger        — this skill's plan
src/main/scala/v4/vec/formal/*Checks.scala   the properties    — //@formal-req- tags
src/main/scala/**/<Dut>.scala                the DUT           — one anchor line, nothing else
```

The bar is **at least one assertion per live requirement.** A requirement that
genuinely cannot be expressed as a property goes in the ledger as
`unassertable:` with a reason — see the rules below. That escape hatch is the
one thing in this skill that can quietly turn 115 requirements into 12
assertions, so it is a reviewable claim, never a default.

## Two agents, two models

This skill is a **two-tier** flow, and the tiers exist to separate judgement
from transcription:

| Tier         | Model  | Owns                                                                     |
|--------------|--------|--------------------------------------------------------------------------|
| Orchestrator | opus   | Reads the reqs and the Chisel. Decides *what* to assert for each requirement, on which signals, in which checker file. Writes the ledger. Fans out to the coders and reconciles what comes back. |
| Coder        | sonnet | Reads one checker's ledger slice. Writes *that* Chisel file and its anchor. Invents no properties. |

Deciding that `spec-decode.d4` is checked by a two-cycle implication on
`io.vcfg_mirror.valid` is the expensive judgement; typing it in Chisel LTL
syntax is not. Splitting them keeps opus tokens on the first job. The coder's
brief is written to be **self-contained** — it never reads the spec `.rst`, and
never re-decides a property.

## Modes

This skill dispatches on the **first token** of the invocation (`args`):

| Verb       | Purpose                                                                     | Follow                       |
|------------|-----------------------------------------------------------------------------|------------------------------|
| `plan`     | Orchestrator only: write/refresh `formal-<family>.yaml`. No Chisel emitted.  | `references/orchestrate.md`  |
| `gen`      | Orchestrator resumes from an existing ledger and fans out coders to write the checkers. | `references/orchestrate.md` |
| `run`      | `plan` then `gen` in one pass.                                              | `references/orchestrate.md`  |
| `validate` | Mechanically check the ledger against the reqs, the checkers, and the build. | `references/validate.md`     |
| `trace`    | Answer "what checks `spec-decode.d4`?" / "which reqs have no assertion?".    | `references/trace.md`        |

Typical calls:

```
/req-formal-chisel plan decode
/req-formal-chisel plan decode --file src/main/scala/v4/exu/decode.scala
/req-formal-chisel gen decode
/req-formal-chisel run cii
/req-formal-chisel validate decode
/req-formal-chisel trace spec-decode.d4
/req-formal-chisel trace --unasserted decode
```

`plan` comes first: no checker is written before its ledger row exists, because
the ledger row is what states which requirement the property is there to
discharge.

## Dispatch

1. Read the first token of `args`.
   - If it matches a verb above, **read that mode's reference file and follow it**.
   - If there is no verb, infer:
     - "plan/decide assertions for …", "what should we assert" → `plan`
     - "write/generate the assertions", "implement the checkers" → `gen`
     - "assertions for family X" with no other qualifier → `run`
     - "check/verify the assertions", "is the ledger right" → `validate`
     - "what checks …", "which reqs have no assertion" → `trace`
   - If ambiguous across modes, ask before proceeding.
2. **All modes must read `references/schema.md` first.** It is the normative
   contract for `formal-<family>.yaml` and for what does and does not qualify as
   an assertion of a requirement.
3. `gen` additionally requires `references/implement.md` — the Chisel LTL and
   layer/`bind` contract. It is also the text handed to each coder.
4. The second positional token is the **family** (`decode`, `cii`, `lsu` — a key
   in `families.yaml`). `--file <path>` narrows a run to the modules in one
   Chisel source; without it the run covers every module the family's
   requirements are allocated to in `hierarchy.yaml`.
5. Bundled assets, resolved relative to this skill directory:
   - `assets/BoomSvaLayer.scala` — the layer definition. Copy it into
     `src/main/scala/v4/vec/formal/` on first run; never write a second layer.
   - `examples/formal-skid.yaml` + `examples/SkidBufferChecks.scala` — a worked
     ledger and checker for a 1-entry skid buffer. Both are real: the properties
     elaborate, firtool 1.75.0 emits `SkidBuffer_BoomSvaLayer.sv` plus the `bind`
     in `layers_SkidBuffer_BoomSvaLayer.sv`, the DUT's own `.sv` has zero
     assertions, and VCS compiles it with `-assert svaext`. The checker, the
     layer, and an anchored `BoomModule` also compile inside the `boom` sbt
     project as-is.
   - `scripts/validate-formal.py` — the mechanical checker. Self-test it against
     the example with:
     ```
     python3 req-formal-chisel/scripts/validate-formal.py \
         --plan-only --formal-dir req-formal-chisel/examples skid
     ```
     Expect **exactly one** error — `family 'skid' is not declared in
     families.yaml` — because the example uses a deliberately fake family so it
     cannot pollute the real requirement corpus. Any other finding is a
     regression in the script or the example.

## Layout this skill owns

```
src/main/nlhdl/formal/
└── formal-<family>.yaml                  # one per feature family, mirroring reqs/

src/main/scala/v4/vec/formal/
├── BoomSvaLayer.scala                    # the single bind layer, copied from assets/
├── VDecodeChecks.scala                   # one checker object per checked module
└── VecCiiHostChecks.scala
```

- One ledger per **feature family**, named `formal-<family>.yaml`, matching
  `spec-<family>.yaml` one-for-one. Never one per module and never one per spec file.
- Assertion IDs are `formal-<family>.<group><n>` — e.g. `formal-decode.d3`. The
  group letters are **the same letters as the requirement family's `groups:`**,
  so `formal-decode.d*` checks `spec-decode.d*`.
- Checker files are `<DutModule>Checks.scala`, package `boom.v4.vec.formal`.
- Paths in the ledger are **repo-relative from `generators/boom`**, matching
  `hierarchy.yaml` (`src/main/scala/v4/exu/decode.scala`).

## Rules of engagement (all modes)

- **One assertion per requirement, minimum.** Every live requirement in
  `spec-<family>.yaml` appears in the ledger exactly once, carrying either
  `assertions:` (≥1 ID) or `unassertable:` (a reason). A requirement in neither
  is a validation error. Silence is the failure mode this ledger exists to make
  visible.

- **`unassertable:` is a claim, not a shrug.** Legitimate: a requirement about
  code provenance ("must be unchanged from BOOM v4"), a structural requirement
  discharged by elaboration (`require(...)` in Scala, not SVA), a requirement
  whose subject is inside a blackbox this flow cannot see. Not legitimate: "hard
  to express", "needs a testbench", "no signal for it yet". If the signal does
  not exist, that is a finding about the RTL — report it, do not bury it in a
  skip reason. Prefer `partial:` (see `references/schema.md`) over a skip when
  you can check *some* of the obligation.

- **Never weaken a property to make it pass.** An assertion that fails is
  information: either the RTL is wrong, the requirement is wrong, or the
  property is wrong, and which one it is is a question for the user. Relaxing
  the antecedent until it stops firing produces a green run that checks nothing
  and a ledger that lies. Report the failure with the counterexample; do not
  edit the property to fit the DUT.

- **`AssumeProperty` needs a citation.** An assume constrains the environment,
  and a wrong one makes every assertion in the file vacuously true — this is the
  single easiest way to make this skill's output worthless. Emit one only where
  a requirement or an explicit interface contract states the constraint, cite it
  in the ledger row, and pair it with a `CoverProperty` proving the constrained
  state is still reachable.

- **Cover statements measure reachability, not correctness.** A `CoverProperty`
  never discharges a requirement on its own. It appears in a ledger row
  alongside at least one assert, or it is a `reachability:` entry with no
  requirement attached.

- **The DUT gets exactly one anchor and nothing else.** One
  `layer.block(BoomSvaLayer) { <Module>Checks(...) }` block at the end of the
  module body, plus its `//@formal-anchor` comment. No signal renaming, no
  refactoring for "observability", no new ports, no probes, no reformatting. If
  a property needs a signal that does not exist in the module body, say so and
  stop — do not synthesize a helper register inside the DUT to make a property
  writable. Wrapper modules and `BoringUtils` taps are **forbidden here**: they
  change the instance hierarchy that the rest of BOOM, the cosim bridge, and
  every waveform script depend on.

- **`//@formal-req-`, never `//@req-`.** Properties carry
  `//@formal-req-spec-decode.d4` tags, one per line, above the property. The
  `//@req-` namespace belongs to `/nlhdl` and means *this code implements the
  requirement*; `//@formal-req-` means *this code checks it*. Collapsing them
  would let an assertion satisfy `/spec-to-reqs trace`, so a requirement with a
  property and no implementation would report as done. Keep them apart.

- **The ledger is the plan; the tags are the claim.** The validator diffs one
  against the other and errors on either direction of mismatch. This is not
  denormalization: the ledger says what opus decided should be checked, the tags
  say what sonnet actually wrote, and a disagreement between them is precisely
  the bug that a single source of truth would hide.

- **Labels are unique, meaningful, and immutable.** Every property carries
  `label = Some("...")`, which becomes its SVA label in the emitted Verilog and
  its key in VCS coverage databases and failure messages. Renaming one orphans
  historical coverage data. Scope them per checker
  (`vdecode_vsetvli_writes_vl`), not globally generic (`assert_1`).

- **IDs are immutable and never reused**, including after retirement — the same
  rule, and the same reason, as requirement IDs.

- **Only the orchestrator writes the ledger; only coders write checker files.**
  One writer per file. `validate` and `trace` write nothing at all.

- **Never round-trip the ledger through PyYAML.** Write it as text so comments
  and key order survive; `scripts/validate-formal.py` only ever reads.


- **Three mandatory stops.** Everything else runs to completion. Stop and ask when:
  1. a requirement needs a signal the module does not expose (report it as an
     observability gap; never add one);
  2. a requirement's `statement:` is too ambiguous to yield a property without
     choosing between two materially different readings;
  3. an existing ledger row's `assertions:` no longer matches the tags in the
     checker file (drift — someone edited one side by hand).

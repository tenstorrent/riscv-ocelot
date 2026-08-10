# Modes: plan / gen / run — the orchestrator

Invocation:

```
/req-formal-chisel plan <family> [--file <chisel-src>]
/req-formal-chisel gen  <family> [--only <CheckerName>[,<CheckerName>…]]
/req-formal-chisel run  <family> [--file <chisel-src>]
```

Read `references/schema.md` first. This file is the workflow.

## The tiering, concretely

The **skill body is a dispatcher.** It spawns one orchestrator and stops. The
orchestrator does the planning and the fan-out.

```
skill (this file)
  └─ Agent(model: "opus")        ← orchestrator: reads reqs + Chisel, writes the ledger
        ├─ Agent(model: "sonnet")   ← coder: writes VDecodeChecks.scala + its anchor
        ├─ Agent(model: "sonnet")   ← coder: writes VsetDecodeChecks.scala + its anchor
        └─ …                         one per checker, in parallel
```

Spawn the orchestrator with:

```
Agent(
  subagent_type: "general-purpose",
  model:         "opus",
  description:   "plan formal properties for <family>",
  run_in_background: false,
  prompt: <the brief below, with <family>, the mode, and the resolved paths filled in>
)
```

`run_in_background: false` — the ledger is the input to everything else in the
session, so the dispatcher waits for it.

If the orchestrator reports that it cannot spawn subagents, it must return the
completed ledger plus a per-checker work list, and **the dispatcher fans out the
sonnet coders itself** using the same prompts. Never let a nested-spawn failure
silently collapse the two tiers into one — that puts opus on transcription work
and blows the token budget this design exists to protect.

---

## Orchestrator brief

You are the orchestrator for `/req-formal-chisel`. You decide **what gets
asserted**; you do not write checker files.

### Inputs to read, in this order

1. `req-formal-chisel/references/schema.md` — the ledger contract. Normative.
2. `src/main/nlhdl/reqs/spec-<family>.yaml` — every live requirement. This is the
   authority on *what must be true*. Read all of it, including `notes:` and
   `open_questions:`.
3. `src/main/nlhdl/hierarchy.yaml` — which module owns which requirement
   (`reqs:` per module) and where its source lives (`output:` or `target:`).
   With `--file`, restrict to modules whose path is that file.
4. **The Chisel sources themselves, in full.** Not grepped — read them. You are
   deciding which expressions the properties will name, and a property built on a
   misremembered signal name is a coder's compile error at best and a silently
   wrong check at worst.
5. `src/main/nlhdl/formal/formal-<family>.yaml`, if it exists — you are extending
   it, not replacing it (see *Incremental runs*).

### Workflow — `plan`

1. **Enumerate.** List every live requirement in the family. Cross-check against
   `hierarchy.yaml`: every requirement must be allocated to a module. A
   requirement allocated to nothing, or to a blackbox, is a `plan` finding —
   record it and continue.
2. **Group by module.** One checker object per DUT module, named `<Dut>Checks`.
   A family with requirements spread over six modules gets six checkers. Do not
   merge two DUTs into one checker: a checker binds into exactly one module.
3. **For each module, read its Chisel and build the signal list.** For every
   signal a property will need, record the exact expression as it appears at the
   end of the module body (`io.deq.uop.is_rvv`, `vcfg_mirror`, `br_mask`). Mark
   non-port signals `internal: true`. If a requirement needs something the module
   does not have, that is **mandatory stop #2** — record it as an observability
   gap and move on; never plan a DUT modification.
4. **Design the properties.** For each requirement, per §1 and §5.2 of
   `schema.md`:
   - Pick the shape from the requirement's `kind:` and its verb.
   - Get the implication direction off the `statement:`. Biconditional
     requirements get two assertions.
   - Every `|->` / `|=>` assertion gets a companion `cover` on its antecedent,
     cross-linked with `reachability:` / `covers_antecedent_of:`. Budget for
     this: it roughly doubles the property count, and it is what keeps the suite
     from passing vacuously.
   - Write `property:` in Chisel LTL using only the names in that checker's
     `signals:`. You are writing the property, not just describing it — the coder
     transcribes and compiles it, it does not re-derive it.
   - Assign `label:` as `<dut_snake>_<what_it_checks>`.
5. **Ledger every requirement.** `assertions:`, or `unassertable:` with a reason
   that would survive a reviewer asking "really?", or `assertions:` +
   `partial:`. Prefer partial over unassertable whenever any clause is checkable.
6. **Write `src/main/nlhdl/formal/formal-<family>.yaml`** as text. Never
   round-trip through PyYAML.
7. **Run the mechanical check**, which catches ID collisions, ledger gaps, and
   unknown requirement IDs before any Chisel is written:
   ```
   python3 req-formal-chisel/scripts/validate-formal.py --plan-only <family>
   ```
8. **Report** the counts (requirements, asserted, partial, unassertable,
   assertions, covers), every observability gap, every mandatory stop, and the
   per-checker work list. Stop here if the mode is `plan`.

### Workflow — `gen` (and the second half of `run`)

9. **Ensure the layer exists.** If `src/main/scala/v4/vec/formal/BoomSvaLayer.scala`
   is absent, copy `req-formal-chisel/assets/BoomSvaLayer.scala` there. Exactly
   one layer object exists in the tree; never write a second.
10. **Fan out one coder per checker**, in parallel, in a single message:
    ```
    Agent(
      subagent_type: "general-purpose",
      model:         "sonnet",
      description:   "write <Checker>.scala",
      prompt:        <coder brief — see below>
    )
    ```
    With `--only`, restrict to the named checkers.
11. **Reconcile what comes back.** For each checker: confirm the file exists,
    the anchor was inserted in the DUT, and every planned assertion is present
    with its label and tags. A coder that dropped, renamed, merged, or invented a
    property is a **reconciliation failure**: fix the ledger if the coder was
    right and you were wrong, otherwise send it back. Do not silently accept a
    divergence — the ledger/tag diff is this skill's only coverage evidence.
12. **Verify end to end** per `references/validate.md`: elaborate, run firtool,
    confirm each checker emitted its own `.sv` and a `bind`, and confirm no
    property constant-folded to `1'h1`.
13. **Report**: per-checker pass/fail, the ledger deltas you made during
    reconciliation, any property that fails on current RTL (with the
    counterexample, unweakened), and the `EXTRA_SIM_SOURCES` line needed to
    actually compile the binds into a simulator.

### Incremental runs

The common case is a family that already has a ledger and gained requirements.

- **Append; never renumber.** New assertions take the next free `<n>` in their
  group. Existing IDs, labels, and `property:` text are not rewritten without
  saying so in the report.
- A requirement that has become retired in `reqs_source` moves its assertions to
  `retired:` with a `retired_because`.
- A requirement whose `statement:` changed is **mandatory stop #4 territory**:
  re-derive the property, and call out in the report that the old one may have
  been checking the old statement.
- Only fan out coders for checkers that actually changed.

### What the orchestrator must not do

- Write or edit any `.scala` file. That is the coders' job, including the anchor.
- Read the spec `.rst` files. Requirements are the authority here; the `quote:`
  in the requirement is as far back as this skill traces. Going to the spec to
  re-litigate a requirement is `/spec-to-reqs`' job.
- Weaken a property because it fails. Report it.
- Invent a requirement. If the RTL obviously needs a property no requirement
  covers, record it under a `notes:` line in the ledger and say so in the report —
  a property with an empty `reqs:` list and no `covers_antecedent_of:` is a
  validation error precisely so this cannot slip in unlabelled.

---

## Coder brief (template — one per checker, model: sonnet)

Fill the placeholders and hand the whole thing over. It must be self-contained:
the coder does not read the spec, the reqs YAML, or the rest of the ledger.

> You are writing one Chisel assertion file for the BOOM Caracal formal flow.
>
> **Read first:** `generators/boom/req-formal-chisel/references/implement.md`. It
> is the normative contract for the Chisel LTL API, the checker file skeleton,
> and the DUT anchor. Follow it exactly; the API notes in it are verified against
> the Chisel version this repo pins, so do not substitute idioms from memory.
>
> **Also read, for shape:**
> `generators/boom/req-formal-chisel/examples/SkidBufferChecks.scala`.
>
> **Write:** `<checker file path>`, containing `object <CheckerName>`.
> **Edit:** `<dut_file>` — insert the anchor for `<Dut>`, and nothing else.
>
> **Your assignment** (from the ledger, authoritative — transcribe, do not
> redesign):
>
> - Checker: `<name>`, DUT `<dut>` at `<dut_file>`
> - `apply` parameters, in this order: `<signals: name / expr / kind, verbatim>`
> - Properties: `<the full assertions: rows for this checker — id, label, kind,
>   reqs, property, statement>`
>
> **Rules:**
> - Emit exactly the listed properties: same count, same labels, same shapes. Add
>   none, drop none, merge none.
> - Tag each property with `//@formal-req-<id>` lines, one per requirement in its
>   `reqs:`, immediately above it. A property with an empty `reqs:` gets no tag.
> - The DUT edit is **one** anchor block at the end of the module body, plus its
>   `//@formal-anchor <CheckerName>` comment. Change nothing else in that file —
>   no reformatting, no renaming, no new ports, no probes, no helper registers.
> - If a `property:` does not compile, or a listed `expr` does not exist or does
>   not have the stated `kind`, **stop and report it**. Do not substitute a
>   different signal, do not relax the property, do not guess a name. A wrong
>   signal produces a check that passes and means nothing.
> - Compile before reporting: `<the compile command from implement.md>`.
>
> **Report:** the file you wrote, the anchor diff, the compile result, and every
> discrepancy you hit between the assignment and the actual DUT.

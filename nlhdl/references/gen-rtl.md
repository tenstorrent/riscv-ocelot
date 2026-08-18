# Mode: gen-rtl — generate RTL from a `.nlhdl` file

Invocation: `/nlhdl gen-rtl <path/to/module.nlhdl.<hdl>>`

Read `references/format.md` first for file naming, syntax, and section
semantics. This file is the generation workflow and its output bar.

## Inputs

- **Target file** — the `.nlhdl.<hdl>` path (from `args`; if absent, ask).
- **`hierarchy.yaml`** — the project module map, for dependency resolution and
  for the module's **`mode:`** (see step 2 — it decides what you write and where).

## Workflow

Follow these steps in order:

1. **Parse.** Read the whole file. Extract module name and target HDL from the
   filename. Identify every section. Reject unrecognized `<|...|>` tokens.
2. **Determine the mode.** Look up the module in `hierarchy.yaml` and read
   `mode:`. It selects the whole shape of this run — do not proceed without it:
   - `new` → write a complete file at `output:`. If `output:` already exists,
     stop and report: either the mode is stale or the file is unexpected.
   - `edit_generated` → regenerate a complete file at `output:`, replacing what
     is there. The spec remains authoritative over the whole module.
   - `edit_existing` → read `target:` in full **before** editing, then apply
     only the delta the spec describes. Jump to the "Editing existing RTL"
     section below and follow it instead of steps 4–5.
   - `blackbox: true` → nothing to generate; report and stop.
   - `mode:` missing on a non-blackbox module → stop and ask which mode applies.
     Never default to overwriting a file.
3. **Resolve dependencies.** For each entry in `<|begin_dependencies|>`, look it
   up in `hierarchy.yaml` to find its interface (and generated output, if it
   exists). Instantiate against that interface. If a dependency is missing from
   the map, stop and report it rather than inventing a port list.
4. **Reconcile.** Check that ports, parameters, logic, and perf are mutually
   consistent. If the spec is ambiguous or self-contradictory, prefer asking the
   user; only when that is not possible, pick the most conservative
   interpretation and record every such assumption **in the nlhdl source and in
   your report** — not as a comment in the generated RTL.
5. **Generate** (`new` and `edit_generated`). The spec is the complete authority:
   the emitted module must implement **every** parameter, port, and behavior in
   the nlhdl file and nothing beyond it. Emit synthesizable RTL in the target HDL:
   - Preserve the Tenstorrent license header from `template.nlhdl.sv` at the top
     of every generated file.
   - Module and port names must match the spec.
   - Preserve the source's `//` and `/* */` comments in the output, placed next
     to the code they annotate — **all of them, and only them.** The nlhdl file
     decides what prose reaches the RTL; you do not add to it. Do not write
     comments to label blocks, restate logic, describe behavior, cite the spec
     section a block came from, or narrate assumptions. If the generated RTL
     needs explaining beyond what the source's comments say, the explanation
     belongs in the nlhdl body, not in the output. See "Comment policy" in
     `references/format.md`.
   - **Carry every `//@req-<id>` tag into the RTL**, verbatim and one per line,
     immediately above the code implementing that requirement — not collected in
     the header. These tags are the spec → nlhdl → RTL traceability chain and
     `spec-to-reqs` reads them by grep, so a dropped tag reads as an
     unimplemented requirement. Keep the literal `//@req-` token even in Chisel.
     If the nlhdl source is missing a tag for an ID in the module's
     `hierarchy.yaml` `reqs:` list, report it rather than inventing a placement.
   - No unsynthesizable constructs in the RTL path (no `#delay`, no `initial`
     for logic, no non-synthesizable system tasks).
   - Reset every stateful element. Avoid inferred latches. Drive every output on
     every path.
6. **Write output.** Write to the `output:` path from `hierarchy.yaml`, or if it
   is absent, the source file's sibling `generated/` directory (e.g.
   `examples/fifo/generated/sync_fifo.sv`). Do not overwrite the `.nlhdl` source.
7. **Report.** Summarize what was generated, which defaults/assumptions you
   chose, and any perf targets you could not guarantee. For `edit_generated`,
   also diff against the previous output and call out every behavioral change —
   including any hand edits to the generated file that your regeneration
   dropped. If a linter/simulator is available, offer to run it — do not claim
   verification you did not perform.

## Editing existing RTL (`mode: edit_existing`)

Here the nlhdl file is a **delta spec**, and the pre-existing RTL at `target:`
is not yours. The bar is not "matches the spec" but "matches the spec **and**
changes nothing else."

1. **Read the target fully** before writing anything. Understand the existing
   reset convention, naming style, clocking, and the blocks you are about to
   touch. Never edit a file you have only grepped.
2. **Read `<|begin_edit_scope|>`.** It bounds the change. If the section is
   missing, derive the scope from the spec and **state it back to the user for
   confirmation** before editing.
3. **Plan the minimal edit.** List the hunks you intend to touch and check each
   against the scope. Anything outside it is out of bounds — including
   reformatting, renaming, reordering ports, "fixing" unrelated bugs, updating
   headers, or tidying comments. Report such findings separately instead.
4. **Preserve conventions.** Match the file's existing indentation, naming,
   comment style, and reset/clock idioms — even where they differ from what you
   would write for a new module. Keep its license/copyright header as-is; do not
   substitute the template header.
5. **Apply the edit** with targeted edits, not a whole-file rewrite. Extend
   existing structures where the spec extends behavior; replace a block only
   when the spec replaces that block. Carry the delta spec's `//@req-<id>` tags
   onto the lines the edit adds or changes — and only those. Pre-existing code
   the edit did not touch gets no tags: this module's `reqs:` covers its delta,
   not the behavior that was already there.
6. **Check for regression.** Before reporting, verify that:
   - every pre-existing port and parameter still has its original declaration
     and semantics unless the spec changed it;
   - behavior outside the changed hunks is bit- and cycle-identical (no shifted
     pipeline depth, no altered reset values, no new stall or backpressure path
     on an untouched interface);
   - default/`else` paths that previously drove a signal still drive it;
   - nothing became an inferred latch or a multiply-driven net.
7. **Stop on conflict.** If the spec cannot be satisfied without changing
   unspecified behavior (e.g. it needs an extra cycle on an interface the scope
   protects), do not pick a side — report the conflict, the two options, and
   wait.
8. **Report as a diff.** Show the touched hunks, map each to the part of the
   spec it implements, and state explicitly what you left untouched and any
   regression risk you could not rule out by inspection.

## Fixing bugs in RTL

This covers any later touch-up of RTL — a bug found in bring-up, a lint fix, a
patch to a file this flow generated — as opposed to a full regeneration.

- **Add no comments.** Not a note on what changed, not a rationale for the fix,
  not a `// FIXME`/`// NOTE`, not a dated changelog line, not a restatement of
  the bug. The fix is the diff; the explanation goes in your report to the user
  and, where the behavior was mis-specified, back into the nlhdl source.
- **The one exception:** a critical issue a future reader of this code would
  otherwise re-break — a hazard, a required ordering, a workaround for silicon
  or tool behavior that looks wrong locally. **Two lines maximum**, placed on
  the code it concerns. If you cannot say it in two lines, it belongs in the
  nlhdl source or a doc, not here.
- **Do not delete or reword comments already in the file**, including
  `//@req-<id>` tags. If a fix moves the code a tag sits above, move the tag
  with it.
- If the bug reveals that the nlhdl spec is wrong, fix the spec too and say so —
  otherwise the next `gen-rtl` regenerates the bug.

## Output bar

- Implements the spec and nothing more.
- Synthesizable; no latches; every output driven on every path; every state
  element reset.
- Assumptions recorded in the nlhdl source and the report, not in RTL comments.
- Every comment in the RTL came from the `.nlhdl` source; none was added by the
  generator. Every `//@req-` tag in the source survives into the RTL, above the
  code that implements it.
- `new` / `edit_generated`: the module fully matches the spec — no spec item
  unimplemented, no behavior the spec did not ask for.
- `edit_existing`: the diff is confined to the specified scope, existing
  behavior is preserved, and the file's original style and header survive.

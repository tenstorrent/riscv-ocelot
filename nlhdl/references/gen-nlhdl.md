# Mode: gen-nlhdl — author a `.nlhdl` file from a natural-language spec

Invocation: `/nlhdl gen-nlhdl "<spec>"` (or a path to a written spec / a
conversation describing the module).

Read `references/format.md` first — the file you produce must conform to it.
Study `examples/fifo/fifo.nlhdl.sv` as the reference for tone and structure.

## Goal

Turn a user's natural-language module description into a well-formed `.nlhdl`
file: descriptive, concise, unambiguous, and ready for `gen-rtl`.

## Workflow

0. **Decide the mode first.** Is this a brand-new module, or a change to RTL that
   already exists? The answer changes what you write and becomes the module's
   `mode:` in `hierarchy.yaml`:
   - **`new`** — no RTL exists. Write a whole-module spec.
   - **`edit_generated`** — the RTL exists and this flow generated it from an
     nlhdl source. Do not write a new file: **edit the existing nlhdl source**
     so it still describes the complete module, then regenerate. The nlhdl file
     stays the single authority — never describe the change as a patch against
     the generated RTL.
   - **`edit_existing`** — the RTL exists but this flow did not write it
     (hand-written or third-party). Write a **delta spec** (step 3b below).
   If you cannot tell, check `hierarchy.yaml` for an existing entry and look for
   the RTL on disk; ask if it is still unclear.
1. **Understand the module.** Identify its function, parameters, ports, behavior,
   performance intent, and dependencies. If any of these are missing and cannot
   be reasonably defaulted, ask before writing — do not invent requirements.
   For an edit, read the existing RTL first — the spec must be written against
   what is actually there, using its real signal and parameter names.
2. **Choose name and target.** Derive `<module_name>` (descriptive, matches the
   intended RTL module name) and `<target_hdl>`. Filename is
   `<module_name>.nlhdl.<target_hdl>`. Confirm the target HDL if unstated.
3. **Draft the file** from `template.nlhdl.sv`:
   - Keep the license header block.
   - A brief `/* ... */` summary comment describing the module.
   - `<|begin_module|>` containing `<|begin_parameters|>`, `<|begin_ports|>`,
     `<|begin_logic|>`.
   - File-scope `<|begin_perf|>` (optional) and `<|begin_dependencies|>`
     (recommended) outside the module block.
3b. **If `mode: edit_existing`, write a delta spec** instead of a whole-module
   spec. See "Whole-module specs vs delta specs" in `references/format.md`.
   - Open the summary comment by stating this is a modification to an existing
     module, and name the target file.
   - Describe **only the change**: new ports/parameters, new or replaced
     behavior, and how it integrates with the existing logic. Reference existing
     signals by their real names. Never restate the full port list — anything
     unmentioned is unchanged by definition.
   - Include `<|begin_edit_scope|>`: the target file/module, what the change may
     touch, what must not regress, and the interface delta. Be specific about
     what is protected — "the existing TX path timing and the apb read latency
     must be unchanged" is usable; "don't break anything" is not.
   - Keep the delta as small as the requirement allows. If the user's request
     really implies a rewrite, say so and propose `new` (or a new module)
     rather than smuggling a rewrite in as an edit.
4. **Write the body as prose**, not `//` lines. Body text is descriptive and
   concise. Reserve `//` / `/* */` comments for annotations meant to survive
   into generated RTL near the relevant code (e.g. a non-obvious algorithm note).
5. **Be complete and consistent.** Every port referenced by the logic must be
   declared; every parameter used must be defined with a default and, where
   sensible, a legal range. State clock/reset conventions explicitly.
5b. **Cite the module's requirements.** If the module's `hierarchy.yaml` entry
   has a `reqs:` list, read each ID's `statement:` from its family YAML under
   `reqs/` and write a `//@req-<id>` comment line immediately above the part of
   the body that specifies it (see "Requirement tags" in `references/format.md`).
   - Tag the specific description, not the file header. Several IDs on one
     description is fine — one line each.
   - **Every allocated ID must appear at least once.** If one has no home in the
     body, the module is under-specified or the allocation is wrong: say which,
     and do not quietly drop the tag.
   - Do not tag a description with a requirement it does not actually answer,
     and do not invent IDs. The tag is a claim, and `spec-to-reqs`' coverage
     numbers are built from these claims.
6. **List dependencies** by module name. If the module instantiates others,
   name them so they can be resolved against `hierarchy.yaml` later; do not
   describe their internals.
7. **Write output** to the appropriate location (ask if unsure): typically a new
   directory `examples/<name>/<name>.nlhdl.<hdl>` or a path the user gives.
8. **Report** what you wrote, the assumptions you made, and the `hierarchy.yaml`
   entry the module needs — including its `mode:` and the matching `output:`
   (`new`/`edit_generated`) or `target:` (`edit_existing`) path. Offer
   `inspect-hierarchy` to add/check it.

## Quality bar

- Conforms to `references/format.md`; delimiters exact; sections in the right
  scope (perf/dependencies/edit_scope outside the module block).
- Prose body, comments used only for carry-through annotations.
- Scope matches the mode: whole-module specs are complete; delta specs describe
  only the change and carry an `<|begin_edit_scope|>` that a generator can hold
  itself to.
- Every requirement allocated to the module in `hierarchy.yaml` is cited by a
  `//@req-<id>` comment above the description that answers it.
- No unstated ports/parameters; clock and reset conventions explicit. (For a
  delta spec, only the delta's ports/parameters — existing ones are implied.)
- Implementable and synthesizable as written — nothing that would force
  simulation-only constructs.

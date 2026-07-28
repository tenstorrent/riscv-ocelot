# Mode: gen-nlhdl — author a `.nlhdl` file from a natural-language spec

Invocation: `/nlhdl gen-nlhdl "<spec>"` (or a path to a written spec / a
conversation describing the module).

Read `references/format.md` first — the file you produce must conform to it.
Study `examples/fifo/fifo.nlhdl.sv` as the reference for tone and structure.

## Goal

Turn a user's natural-language module description into a well-formed `.nlhdl`
file: descriptive, concise, unambiguous, and ready for `gen-rtl`.

## Workflow

1. **Understand the module.** Identify its function, parameters, ports, behavior,
   performance intent, and dependencies. If any of these are missing and cannot
   be reasonably defaulted, ask before writing — do not invent requirements.
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
4. **Write the body as prose**, not `//` lines. Body text is descriptive and
   concise. Reserve `//` / `/* */` comments for annotations meant to survive
   into generated RTL near the relevant code (e.g. a non-obvious algorithm note).
5. **Be complete and consistent.** Every port referenced by the logic must be
   declared; every parameter used must be defined with a default and, where
   sensible, a legal range. State clock/reset conventions explicitly.
6. **List dependencies** by module name. If the module instantiates others,
   name them so they can be resolved against `hierarchy.yaml` later; do not
   describe their internals.
7. **Write output** to the appropriate location (ask if unsure): typically a new
   directory `examples/<name>/<name>.nlhdl.<hdl>` or a path the user gives.
8. **Report** what you wrote, the assumptions you made, and note that the module
   likely needs a `hierarchy.yaml` entry (offer `inspect-hierarchy` to add/check it).

## Quality bar

- Conforms to `references/format.md`; delimiters exact; sections in the right
  scope (perf/dependencies outside the module block).
- Prose body, comments used only for carry-through annotations.
- No unstated ports/parameters; clock and reset conventions explicit.
- Implementable and synthesizable as written — nothing that would force
  simulation-only constructs.

# Mode: gen-rtl — generate RTL from a `.nlhdl` file

Invocation: `/nlhdl gen-rtl <path/to/module.nlhdl.<hdl>>`

Read `references/format.md` first for file naming, syntax, and section
semantics. This file is the generation workflow and its output bar.

## Inputs

- **Target file** — the `.nlhdl.<hdl>` path (from `args`; if absent, ask).
- **`hierarchy.yaml`** — the project module map, for dependency resolution.

## Workflow

Follow these steps in order:

1. **Parse.** Read the whole file. Extract module name and target HDL from the
   filename. Identify every section. Reject unrecognized `<|...|>` tokens.
2. **Resolve dependencies.** For each entry in `<|begin_dependencies|>`, look it
   up in `hierarchy.yaml` to find its interface (and generated output, if it
   exists). Instantiate against that interface. If a dependency is missing from
   the map, stop and report it rather than inventing a port list.
3. **Reconcile.** Check that ports, parameters, logic, and perf are mutually
   consistent. If the spec is ambiguous or self-contradictory, prefer asking the
   user; only when that is not possible, pick the most conservative
   interpretation and record every such assumption as a comment in the output.
4. **Generate.** Emit synthesizable RTL in the target HDL:
   - Preserve the Tenstorrent license header from `template.nlhdl.sv` at the top
     of every generated file.
   - Module and port names must match the spec.
   - Preserve the source's `//` and `/* */` comments in the output, placed next
     to the code they annotate.
   - No unsynthesizable constructs in the RTL path (no `#delay`, no `initial`
     for logic, no non-synthesizable system tasks).
   - Reset every stateful element. Avoid inferred latches. Drive every output on
     every path.
   - Comment each block with the piece of the spec it implements.
5. **Write output.** Place generated files in the source file's sibling
   `generated/` directory (e.g. `examples/fifo/generated/sync_fifo.sv`) or the target directory specified by hierarchy.yaml. Do not overwrite the `.nlhdl` source.
6. **Report.** Summarize what was generated, which defaults/assumptions you
   chose, and any perf targets you could not guarantee. If a linter/simulator is
   available, offer to run it — do not claim verification you did not perform.

## Output bar

- Implements the spec and nothing more.
- Synthesizable; no latches; every output driven on every path; every state
  element reset.
- Assumptions documented inline where the spec was silent.
- Traceable back to the `.nlhdl` sections and comments.

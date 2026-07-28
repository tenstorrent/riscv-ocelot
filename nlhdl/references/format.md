# NL_HDL File Format (shared reference)

Read this before running `gen-rtl` or `gen-nlhdl`. It defines the file naming,
syntax, and section semantics both modes rely on.

## File naming

```
<module_name>.nlhdl.<target_hdl>
```

- **`<module_name>`** — a descriptive name of the module's function. This is the
  RTL module name (e.g. `sync_fifo` → `module sync_fifo`).
- **`<target_hdl>`** — the extension of the HDL to emit; it selects the target
  language:
  - `.sv` → SystemVerilog
  - `.v` → Verilog
  - `.vhd` → VHDL
  - `.scala` → Chisel

Example: `sync_fifo.nlhdl.sv` → module `sync_fifo`, generated as SystemVerilog.

## Syntax

A file is free-form text plus comments and delimited sections:

```
/*
  Block comments — file-level notes, license header, rationale.
*/

// Inline comments — clarify a single line or section.

<|begin_module|>
  // Module body (see sections below)
  if condA then assign 1 to outA
<|end_module|>
```

- `<|begin_module|>` / `<|end_module|>` delimit the module interface and
  behavior. Everything the module *is* lives inside this block. Supplementary
  sections (`perf`, `dependencies`) live **outside** the module block, at file
  scope.
- **Body text has no special syntax** — it is descriptive, concise prose (and
  optionally pseudo-code, tables, or equations).
- **Comments (`//`, `/* */`) are preserved in the generated RTL, placed near the
  relevant code.** They are author annotations, distinct from body text.
- Delimiters are literal tokens. Match them exactly. Treat any unrecognized
  `<|...|>` token as an error and surface it rather than guessing.

## Sections inside `<|begin_module|>`

### Parameters — `<|begin_parameters|>` … `<|end_parameters|>`
Compile-time parameters and their meaning. Each entry should give a name, an
intent, and where possible a default and legal range. Generate these as
`parameter`s (SystemVerilog/Verilog) or `generic`s/params in the target HDL. If
a default is unstated, choose a safe one and record the choice in a comment.

### Ports — `<|begin_ports|>` … `<|end_ports|>`
The module's I/O. For each port capture direction, width (may be parameterized),
and purpose. Clock and reset conventions (edge, active level, sync vs async)
belong here; if unstated, default to **posedge clock, active-high synchronous
reset** and note the assumption.

### Logic — `<|begin_logic|>` … `<|end_logic|>`
The behavioral heart of the module: what it computes and how state evolves.
When the description implies state, prefer explicit, named state and clearly
separated sequential vs combinational blocks.

## Supplementary sections (file scope, outside the module block)

### Performance — `<|begin_perf|>` … `<|end_perf|>`  *(optional)*
Intended performance targets: throughput, latency/cycle count, target frequency,
pipeline depth. Treat these as **constraints on the implementation**, not
commentary. If a target forces a structural choice (e.g. "1 result/cycle" ⇒
pipelined), make that choice and note it.

### Dependencies — `<|begin_dependencies|>` … `<|end_dependencies|>`  *(recommended)*
The modules this module instantiates or depends on. Cross-reference each against
`hierarchy.yaml` to locate the dependency's own NL_HDL source and interface. Do
**not** re-implement a dependency inline — instantiate it by its declared
interface.

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
  sections (`perf`, `dependencies`, `edit_scope`) live **outside** the module
  block, at file scope.
- **Body text has no special syntax** — it is descriptive, concise prose (and
  optionally pseudo-code, tables, or equations).
- **Comments (`//`, `/* */`) are preserved in the generated RTL, placed near the
  relevant code.** They are author annotations, distinct from body text — and
  because they are the only thing that crosses into the RTL, they are rationed.
  See "Comment policy" below.
- Delimiters are literal tokens. Match them exactly. Treat any unrecognized
  `<|...|>` token as an error and surface it rather than guessing.

## Comment policy

The nlhdl file is where intent, context, and behavior are recorded. The RTL is
the mechanical projection of it. A comment in an nlhdl file is therefore not
documentation — it is an explicit request to push text into the generated RTL,
and generated RTL should carry as little prose as it can.

Only two kinds of comment belong in an nlhdl file:

1. **`//@req-<id>` requirement tags — required.** Every ID allocated to the
   module must appear. These are the traceability chain and are never optional,
   never collected in the header, and never dropped. See below.
2. **A short file description — 1–2 lines.** What the module is, not how it
   works. "Synchronous FIFO, parameterized width/depth, with occupancy count."
   is the right length. A paragraph is not.

Everything else stays in the body prose:

- **Do not** write a comment that describes what the code does, restates the
  logic, labels a block ("// write pointer update"), or explains an algorithm.
  That description is the body text's job, and it is already in this file.
- **Rare exception:** a genuinely critical note a future reader of the RTL alone
  would get wrong — a hazard, an off-by-one that looks like a bug, a timing or
  ordering constraint that is invisible locally. Two lines maximum.
- Assumptions you had to make (a defaulted width, an unstated reset polarity)
  belong in the body prose and the report, not in an RTL comment.

Rule of thumb: if deleting the comment would lose nothing that isn't already in
the nlhdl body, it should not be a comment.

## Requirement tags — `//@req-<id>`

A comment of the form `//@req-spec-<family>.<group><n>`, one ID per line,
immediately above the text it applies to:

```
// @req tags cite the requirement the following description answers.
//@req-spec-cii.e1
//@req-spec-cii.e2
On rob.io.flush.valid, every in-flight tag is killed and its credit returned.
```

- The IDs come from the module's `reqs:` list in `hierarchy.yaml`, which the
  `architect` mode allocated from the requirement corpus under `reqs/`.
- The token is literally `//@req-` in every target language, including Chisel —
  that exact string is what `spec-to-reqs`' validator greps for. In a language
  whose line comment is not `//` (VHDL), write `-- //@req-<id>`.
- A tag is a **claim that this code satisfies that obligation**. Put it on the
  specific description or block that answers the requirement, never on the file
  as a whole — a header full of tags claims everything and locates nothing.
- Tags are comments, so the normal comment rule carries them into the generated
  RTL, next to the code they annotate. That is what makes a requirement
  traceable from spec → nlhdl → RTL by grep.

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

### Edit scope — `<|begin_edit_scope|>` … `<|end_edit_scope|>`  *(required for `mode: edit_existing`)*
Bounds the change when the nlhdl file is a **delta spec** against pre-existing
RTL. Capture, in prose:

- **Target** — the file and module being edited (must match `target:` in
  `hierarchy.yaml`).
- **In scope** — the specific signals, always-blocks, states, or ports the
  change may touch.
- **Out of scope / must not regress** — behavior that must remain bit- and
  cycle-identical, plus interfaces whose semantics must not shift. Existing
  behavior is protected by default; this list calls out the parts most at risk.
- **Interface delta** — new or widened ports/parameters only. Anything not
  listed keeps its current declaration exactly.

The generator treats this section as a hard constraint, not commentary: an edit
that reaches outside "in scope" is a failure, even if it looks like an
improvement.

## Whole-module specs vs delta specs

The `mode:` field on the module's `hierarchy.yaml` entry decides how the file is
read. Author accordingly:

| `mode:`          | The nlhdl file describes…            | Sections                                     |
|------------------|-------------------------------------|----------------------------------------------|
| `new`            | the whole module                    | parameters / ports / logic, fully populated  |
| `edit_generated` | the whole module                    | same — the spec stays the complete authority |
| `edit_existing`  | only the change to apply            | delta parameters/ports/logic + `edit_scope`  |

For `edit_existing`, `<|begin_parameters|>`, `<|begin_ports|>`, and
`<|begin_logic|>` describe **the delta only** — new ports, new/changed behavior,
and how it integrates with what is already there. Say so explicitly in the body
(e.g. "adds…", "replaces the existing single-entry buffer with…") so a reader
cannot mistake a delta for a complete interface. Never restate the entire
existing port list; unmentioned ports are unchanged by definition.

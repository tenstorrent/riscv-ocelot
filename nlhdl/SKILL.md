---
name: nlhdl
description: Author and generate RTL with the NL_HDL flow. Use when the user works with .nlhdl files or hierarchy.yaml — generating synthesizable HDL from a .nlhdl spec (gen-rtl), authoring a .nlhdl file from a natural-language spec (gen-nlhdl), or validating/answering questions about a hierarchy.yaml (inspect-hierarchy).
---

# NL_HDL — Natural Language HDL Design SKILL

NL_HDL describes an RTL module in structured natural language. This skill turns
those descriptions into synthesizable RTL, authors new descriptions, and keeps
the project's module map (`hierarchy.yaml`) well-formed.

## Modes

This skill dispatches on the **first token** of the invocation (`args`):

| Verb                 | Purpose                                                    | Follow                            |
|----------------------|------------------------------------------------------------|-----------------------------------|
| `gen-rtl`            | Generate synthesizable RTL from a `.nlhdl.<hdl>` file.      | `references/gen-rtl.md`           |
| `gen-nlhdl`          | Author a new `.nlhdl` file from a natural-language spec.    | `references/gen-nlhdl.md`         |
| `inspect-hierarchy`  | Validate a `hierarchy.yaml` / answer design-org questions.  | `references/inspect-hierarchy.md` |

Typical calls:

```
/nlhdl gen-rtl examples/fifo/fifo.nlhdl.sv
/nlhdl gen-nlhdl "a synchronous FIFO, WIDTH/DEPTH parameterized, count output"
/nlhdl inspect-hierarchy hierarchy.yaml
```

## Dispatch

1. Read the first token of `args`.
   - If it matches a verb above, **read that mode's reference file and follow it**.
   - If there is no verb (e.g. auto-invoked, or the user described a task in prose),
     **infer** the mode from the request:
     - "generate/build RTL/HDL from …" → `gen-rtl`
     - "write/author an nlhdl for …", "turn this spec into an nlhdl" → `gen-nlhdl`
     - "check/validate/explain the hierarchy" → `inspect-hierarchy`
   - If the request is ambiguous across modes, ask which mode before proceeding.
2. `gen-rtl` and `gen-nlhdl` both depend on the file format — read
   `references/format.md` first when running either.
3. Bundled assets you may rely on, resolved relative to this skill directory:
   - `template.nlhdl.sv` — canonical file skeleton and license header.
   - `hierarchy.yaml` — the project module map and its documented schema.
   - `examples/` — worked examples (e.g. `examples/fifo/`).

## Rules of engagement (all modes)

- **Implement the spec, not more.** Never add ports, features, or "helpful"
  logic the spec does not call for.
- **Ambiguity → ask, then assume.** Prefer clarifying over guessing. When you
  must assume (missing default, reset polarity, width), choose the conservative
  option and document it inline.
- **Synthesizability is non-negotiable.** If a behavior cannot be expressed
  synthesizably in the target HDL, say so instead of emitting simulation-only code.
- **Traceability.** A reader must be able to map generated artifacts back to the
  section of the source they came from.


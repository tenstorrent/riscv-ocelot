---
name: nlhdl
description: Author and generate RTL with the NL_HDL flow. Use when the user works with .nlhdl files or hierarchy.yaml — planning a design's module map from requirements and a plan doc (architect), generating synthesizable HDL from a .nlhdl spec (gen-rtl), authoring a .nlhdl file from a natural-language spec (gen-nlhdl), or validating/answering questions about a hierarchy.yaml (inspect-hierarchy).
---

# NL_HDL — Natural Language HDL Design SKILL

NL_HDL describes an RTL module in structured natural language. This skill plans
a design's module map, turns those descriptions into synthesizable RTL, authors
new descriptions, and keeps the map (`hierarchy.yaml`) well-formed.

## Modes

This skill dispatches on the **first token** of the invocation (`args`):

| Verb                 | Purpose                                                    | Follow                            |
|----------------------|------------------------------------------------------------|-----------------------------------|
| `architect`          | Plan the design: turn requirements + a plan doc into `hierarchy.yaml`. | `references/architect.md` |
| `gen-rtl`            | Generate synthesizable RTL from a `.nlhdl.<hdl>` file.      | `references/gen-rtl.md`           |
| `gen-nlhdl`          | Author a new `.nlhdl` file from a natural-language spec.    | `references/gen-nlhdl.md`         |
| `inspect-hierarchy`  | Validate a `hierarchy.yaml` / answer design-org questions.  | `references/inspect-hierarchy.md` |

They run in that order — the map decides which modules exist and which
requirements each one owns, before any nlhdl or RTL is written:

```
reqs/*.yaml + plan.md --architect--> hierarchy.yaml --gen-nlhdl--> *.nlhdl.* --gen-rtl--> RTL
```

Typical calls:

```
/nlhdl architect docs_caracal/caracal-milestone-plan-v2.md
/nlhdl gen-rtl examples/fifo/fifo.nlhdl.sv
/nlhdl gen-nlhdl "a synchronous FIFO, WIDTH/DEPTH parameterized, count output"
/nlhdl inspect-hierarchy hierarchy.yaml
```

## Dispatch

1. Read the first token of `args`.
   - If it matches a verb above, **read that mode's reference file and follow it**.
   - If there is no verb (e.g. auto-invoked, or the user described a task in prose),
     **infer** the mode from the request:
     - "plan/lay out the design", "what modules do we need", "build the
       hierarchy from the reqs/plan" → `architect`
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

- **Check `mode:` before writing anything.** Every non-blackbox module in
  `hierarchy.yaml` declares whether its RTL is written fresh (`new`), regenerated
  from its nlhdl source (`edit_generated`), or patched in place into pre-existing
  hand-written RTL (`edit_existing`). This decides both what the nlhdl file means
  and what you are allowed to touch:
  - `new` / `edit_generated` — the nlhdl source is authoritative over the **whole
    module**; the RTL must fully match the spec, with nothing missing and nothing
    extra.
  - `edit_existing` — the nlhdl source is a **delta spec**. Apply only the
    specified change, preserve the file's existing style and header, and cause no
    functional regression. If the change cannot be made without touching
    unspecified behavior, stop and report instead of deciding.
  A missing `mode:` is an error, not a default — never overwrite a file to find
  out what it was.
- **Implement the spec, not more.** Never add ports, features, or "helpful"
  logic the spec does not call for. When editing existing RTL this extends to
  the file itself: no reformatting, renaming, reordering, or unrelated fixes —
  report those separately.
- **Ambiguity → ask, then assume.** Prefer clarifying over guessing. When you
  must assume (missing default, reset polarity, width), choose the conservative
  option and record it in the nlhdl source and your report — not as a comment in
  the RTL.
- **Comments are rationed; the nlhdl file holds the intent.** The nlhdl source
  is where behavior, context, and rationale are written; generated RTL is the
  mechanical projection of it and should carry almost no prose. In an nlhdl file
  the only comments are `//@req-<id>` tags (mandatory) and a 1–2 line file
  description; never a comment that describes, labels, or restates logic — that
  text goes in the body prose. `gen-rtl` copies the source's comments through
  verbatim and **adds none of its own**. Later bug fixes to RTL add no comments
  either, unless a critical issue would otherwise be re-broken by the next
  reader — and then two lines maximum. Full policy: "Comment policy" in
  `references/format.md`.
- **Synthesizability is non-negotiable.** If a behavior cannot be expressed
  synthesizably in the target HDL, say so instead of emitting simulation-only code.
- **Traceability.** A reader must be able to map generated artifacts back to the
  section of the source they came from.
- **Carry requirements through to the code.** Where a module's `hierarchy.yaml`
  entry has a `reqs:` list, each of those IDs is cited as a `//@req-<id>` comment
  above the part of the nlhdl file that specifies it, and that comment survives
  into the generated RTL above the code that implements it. Placing a tag is a
  claim that this code satisfies that obligation — it is this skill's to make
  (`spec-to-reqs` writes requirements but never tags), so make it deliberately:
  tag the implementing code, not the file header.


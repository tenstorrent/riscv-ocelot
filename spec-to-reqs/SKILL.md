---
name: spec-to-reqs
description: Derive traceable RTL requirements from architecture specs. Use when curating the feature-family registry and deciding which family owns which spec section (curate), turning a .rst/.md spec into per-family requirement YAMLs under src/main/nlhdl/reqs/ (extract), checking that those YAMLs still match the spec and the RTL tags (validate), or answering what implements a req / which reqs are unimplemented (trace).
---

# spec-to-reqs — Spec → Requirements SKILL

This skill turns an architecture spec into a set of **requirements**: small, numbered,
individually-tagged obligations that the RTL must fulfil, each one grounded in a verbatim
quote from the spec it came from.

A requirement is the unit of traceability between three artifacts:

```
docs_caracal/src/*.rst          the spec         — the authority
src/main/nlhdl/reqs/*.yaml      the requirements — this skill's output
src/main/nlhdl/**/*.nlhdl.*     the RTL          — carries //@req-<id> tags
```

Requirements **are binding on the microarchitecture.** If the spec dictates a structure,
that structure is a requirement, not a suggestion. This is deliberate, and it is also the
skill's central hazard: a tool that may prescribe microarchitecture can dress its own
design instincts up as requirements. The only guard against that is the quote rule —
**every requirement carries verbatim spec text, no exceptions.** If you cannot quote it,
it is not a requirement.

## Modes

This skill dispatches on the **first token** of the invocation (`args`):

| Verb       | Purpose                                                          | Follow                     |
|------------|------------------------------------------------------------------|----------------------------|
| `curate`   | Author and audit `families.yaml` — the family list, the spec corpus, and which family owns which spec section. | `references/curate.md` |
| `extract`  | Mine one feature family's specs into `spec-<family>.yaml`.        | `references/extract.md`    |
| `validate` | Mechanically check the YAMLs against the specs and the RTL tags.  | `references/validate.md`   |
| `trace`    | Answer "what implements X?" and "what is unimplemented?".         | `references/trace.md`      |

Typical calls:

```
/spec-to-reqs curate audit
/spec-to-reqs curate assign docs_caracal/src/midcore.rst
/spec-to-reqs curate add memord
/spec-to-reqs extract lsu
/spec-to-reqs validate
/spec-to-reqs validate cii
/spec-to-reqs trace spec-lsu.f9
/spec-to-reqs trace --unimplemented lsu
```

`curate` comes first in a family's life: a family must exist and own sections before
`extract` can mine them.

## Dispatch

1. Read the first token of `args`.
   - If it matches a verb above, **read that mode's reference file and follow it**.
   - If there is no verb, infer:
     - "add a family", "who owns this section", "audit the registry" → `curate`
     - "turn this spec into reqs", "mine …", "write reqs for …" → `extract`
     - "check/verify the reqs", "are the reqs still right" → `validate`
     - "what implements …", "which reqs are untagged" → `trace`
   - If ambiguous across modes, ask before proceeding.
2. **All modes must read `references/schema.md` first.** It is the normative contract for
   the YAML and for what does and does not qualify as a requirement.
3. Bundled assets, resolved relative to this skill directory:
   - `scripts/validate-reqs.py` — the mechanical checker.
   - `examples/families.yaml` + `examples/spec-cii.yaml` — a worked registry and family,
     quotes checked against the real spec. Validate them with:
     `python3 scripts/validate-reqs.py --reqs-dir <skill-dir>/examples`

## Layout this skill owns

```
src/main/nlhdl/reqs/
├── families.yaml          # the curated registry: families, corpus, section ownership
├── spec-decode.yaml
├── spec-lsu.yaml
└── spec-cii.yaml
```

- One YAML per **feature family**, never per spec file. A family draws from as many spec
  files as it needs, and one spec file feeds as many families as it needs — `families.yaml`
  records which family owns which section, so each family ledgers only its own.
- Requirement IDs are `spec-<family>.<group><n>` — e.g. `spec-lsu.f9`.
- RTL tags are `//@req-spec-lsu.f9`, **one per line**, on the line(s) above the code that
  implements the requirement.
- `source:` paths are **repo-relative** (`docs_caracal/src/cii.rst`).

## Rules of engagement (all modes)

- **Never invent a requirement.** Every `statement:` must be supported by the `quote:`
  beside it. If the spec does not say it, it is not a requirement — no matter how obviously
  correct it seems, and no matter how much the design appears to need it. Design gaps go in
  `open_questions:`, never into a `statement:`.

- **Never paraphrase inside `quote:`.** It is a verbatim substring of the cited file,
  copied exactly. Whitespace may be reflowed (matching is whitespace-normalized); nothing
  else may change. The quote is what makes drift mechanically detectable — a paraphrased
  quote silently disables the only staleness check in the system.

- **No number in a `statement:` unless the spec states that number as a requirement.**
  "A queue absorbs the burst" is a requirement. "A 4-entry queue absorbs the burst" is a
  requirement only if the spec says four.

- **IDs are immutable and never reused.** Not after retirement, not after renumbering, not
  ever. Existing RTL tags depend on it. `extract` appends within a group; it never
  renumbers, and it never rewrites an existing `statement:` or `quote:` without approval.

- **Grep is the truth about implementation.** There is no `implemented_by:`, `status:`, or
  `covered:` field, by design. A denormalized copy of `grep -rn 'req-spec-lsu.f9'
  src/main/nlhdl/` is a field that will be wrong within a week, and authoritatively so.

- **This skill never writes `//@req-` tags.** It reads them. Placing a tag is a claim that
  code satisfies a requirement — an implementation judgment owned by `/nlhdl gen-nlhdl`.
  If the same tool authored both the requirement and the claim it was met, coverage would
  be self-certifying and an untagged requirement would stop being a signal.

- **The only files this skill writes outside `src/main/nlhdl/reqs/`** are anchor insertions
  into spec `.rst` files, batched behind a single approval at the start of an `extract` run
  (see `references/extract.md`). It never edits spec prose, and it never edits RTL.

- **Only `curate` writes `families.yaml`; only `extract` writes `spec-<family>.yaml`.** One
  writer per file. `validate` and `trace` write nothing at all.

- **Family names are permanent.** A family name is inside every requirement ID, and every ID
  is inside RTL tags. There is no rename, split or merge once requirements exist — only
  retire-and-reissue, which invalidates every affected tag. See `references/curate.md`.

- **Four mandatory stops.** Everything else runs to completion. Stop and ask when:
  1. a requirement fits no family declared in `families.yaml`, or falls in a section no
     family owns (hand off to `curate`);
  2. a requirement appears to be superseded and should be retired;
  3. an existing `quote:` no longer occurs in its spec file (drift);
  4. the spec says **should** rather than must/may.

- **Write YAML as text, never round-trip it through a library.** PyYAML would strip
  comments and reorder keys, turning every extraction into an unreviewable whole-file diff.
  `scripts/validate-reqs.py` only ever reads.

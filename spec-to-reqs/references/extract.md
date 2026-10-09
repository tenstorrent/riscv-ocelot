# `extract` — mine one family's specs into requirements

```
/spec-to-reqs extract <family>
```

**Read `references/schema.md` before doing anything else.** It is the normative contract
for what qualifies as a requirement and what the YAML must contain.

`extract` is **idempotent and additive.** Re-running it on a changed spec *is* the update
path — there is no separate `update` verb, because two code paths that must agree on ID
assignment will eventually disagree, and the second one gets less testing.

---

## Scope: one family per invocation

Never "extract everything." Family scope is what keeps the whole source set in attention at
once and the resulting diff reviewable.

---

## Procedure

### Step 0 — Resolve the family and its assigned sections

1. Read `src/main/nlhdl/reqs/families.yaml`.
2. If `<family>` is not declared there: **stop and hand off to `curate add`** — this is
   mandatory stop #1. Do not proceed on assumption, and do not edit `families.yaml`
   yourself; `curate` is its only writer.
3. Determine **which sections this run may mine**: the headings `assignments:` gives this
   family, for each of its `sources:`. Sections owned by a sibling family or listed
   `unowned:` are **out of scope for this run** — do not read them for obligations, and do
   not ledger them.
   - If a source file has no `assignments:` block, the legacy rule applies: every section
     of that file is in scope, and siblings' sections are skipped in the ledger with a
     reason. Prefer running `curate assign <file>` first.
4. Read `src/main/nlhdl/reqs/spec-<family>.yaml` if it exists. Everything in it — IDs,
   groups, statements, quotes — is **existing state to be preserved**, not a draft to be
   rewritten.

### Step 1 — Anchor sweep (one approval, before any extraction)

Parse every file in the family's `sources:` and list every section you intend to mine that
has **no enclosing `.. _label:` anchor**. Present the exact insertions:

```
docs_caracal/src/cii.rst — 9 sections lack an enclosing anchor. Propose inserting:

  before line 210  ".. _cii-kill-contract:"   §"The kill contract"
  before line 341  ".. _cii-writeback:"       §"Result writeback and completion"
  before line 402  ".. _cii-segmented:"       §"Segmented load/store"
  ...

Additive only — no prose is touched. Apply?
```

Rules for this step:

- **Additive only.** Insert an anchor line and a blank line before the section's heading.
  Never touch prose, never reorder, never reword a heading.
- Anchor names are `<family-or-file-stem>-<slug-of-heading>`, lowercase, hyphenated.
- Anchors are semantically inert in Sphinx: they add a label, change no rendering, and
  break no existing `:ref:`.
- **One approval for the whole batch**, before extraction begins — not one per section.
- If declined: proceed with `heading:`-only citation and note in your final summary which
  requirements are citing an unanchored section, so a later pass can fix them in bulk.

This is the **only** place `extract` writes outside `src/main/nlhdl/reqs/`.

### Step 2 — Drift check on existing requirements

For every requirement already in the family file, confirm its `quote:` still occurs in its
`source.file` (whitespace-normalized). If any do not:

**Stop. Report every drifted requirement, and propose — do not apply — an updated quote and
statement.** Silently re-quoting would let a spec rewrite retroactively change what the RTL
was tagged as satisfying, invisibly to review. This is mandatory stop #3.

### Step 3 — Walk this family's assigned sections in document order

For each spec source, in file order, for each **assigned** section in document order:

1. Read the section in full, including tables, notes, warnings and admonitions —
   `.. warning::` and `.. danger::` blocks in these specs carry hard obligations, often the
   sharpest ones in the chapter.
2. Decide: does this section contain obligations at all?
   - **No** → add a `coverage:` entry with `skipped:` and an honest, specific reason.
     "Rationale for §4", "restates the glossary definition". Not "nothing here", and not
     "belongs to another family" — if that is true, the assignment is wrong, so stop and
     say so rather than skipping it.
   - **Yes** → write the requirements.
3. For each obligation found:
   - Assign a group letter from `groups:`; add a new group (with its description) if the
     section opens a genuinely new area of the family.
   - Take the next free `<n>` **within that group**. Never renumber anything.
   - Write `statement:` per the §5 rules — one sentence, must/shall, explicit subject, no
     conjoined verbs, no rationale.
   - Classify `kind:` — `function` / `interface` / `structure` / `timing`.
   - Copy `quote:` **verbatim**.
   - Set `anchor:` to the nearest enclosing label and `heading:` to the exact section
     heading.
4. Append the section's `coverage:` entry listing the IDs just written.
5. **Write to disk before moving to the next section.** A run interrupted at section 12
   leaves sections 1–11 correct on disk, and re-running resumes rather than restarting —
   idempotency guarantees it.

### Step 4 — Report

- Counts by `kind:`, and total added this run.
- New groups opened.
- Sections skipped, with reasons.
- `open_questions:` accumulated.
- Requirements citing unanchored sections, if the Step 1 batch was declined.

---

## The four mandatory stops

Everything else runs to completion without asking. Stop for:

| # | Trigger                                            | Why it cannot be automatic                                        |
|---|----------------------------------------------------|-------------------------------------------------------------------|
| 1 | A requirement fits no declared family, or sits in a section this family does not own | A silently coined family rots every future ID; a silently re-owned section makes two families claim one obligation. Hand off to `curate`. |
| 2 | A requirement appears superseded                    | Retirement destroys existing traceability.                        |
| 3 | An existing `quote:` no longer occurs in the spec   | Silent re-quoting rewrites history invisibly.                     |
| 4 | The spec says **should**                            | Guess `must` and you fabricate; guess `may` and you drop a real one. |

Deliberately *not* a stop: per-section approval. Forty sections means forty round trips,
and by gate fifteen the reviewer is rubber-stamping — which is worse than no gate, because
it launders unreviewed content as reviewed.

---

## Failure modes to guard against

**Inventing microarchitecture.** Requirements are binding on the microarchitecture, so this
skill is licensed to prescribe structure — which makes it very easy to promote your own
design instinct into a requirement. Every time you write a `structure` requirement, check
that the `quote:` beside it actually dictates that structure. If the quote only *implies*
it, or describes a problem the structure would solve, it is not a requirement; it is a
design gap, and it belongs in `open_questions:`.

**Coarse extraction.** Fifteen requirements from a 35 KB spec means whole paragraphs were
compressed into single entries. Each one then gets tagged in one place and the rest of its
content goes untraced. Split until each entry is one idea.

**Laundering description as obligation.** These specs quote BOOM source and explain
existing behaviour at length. `rob.scala:436-442` says X is evidence, not a requirement on
Caracal. The requirement is what the spec says Caracal *must do* in light of it.

**Skip reasons that are excuses.** "Background" applied to a section containing a
`.. danger::` block is how a real obligation disappears. Reasons get reviewed; write ones
that survive review.

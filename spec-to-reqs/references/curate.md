# `curate` — author and audit `families.yaml`

```
/spec-to-reqs curate audit                     # registry health report
/spec-to-reqs curate assign <spec-file>        # assign a file's sections to families
/spec-to-reqs curate add <family>              # add a family (extract's gate #1)
/spec-to-reqs curate survey <glob>             # propose a taxonomy for a new corpus
```

**Read `references/schema.md` §3 before doing anything else.**

`families.yaml` is the ID namespace. Every requirement ID embeds its family name, and every
`//@req-` tag in the RTL embeds that ID, so the keys in this file are **permanent in a way
nothing else in the flow is.** That is what this verb exists to protect: the other three
verbs read the registry and trust it; only `curate` changes it.

---

## What the registry owns

Three things, all of which are judgement calls and none of which belong to `extract`:

1. **The family list** — the architectural boundaries, and therefore the ID namespace.
2. **The corpus** — which spec files are in scope (`spec_corpus:`), and which are
   deliberately out (`excluded_sources:`, with reasons).
3. **The section→family assignment** — for each spec file, which family owns which section,
   and which sections nobody owns.

The third is the one that earns this verb. A spec file is routinely shared by several
families: `midcore.rst` has 22 sections split across `rename`, `rob` and `vrf`. Without a
central assignment, each of those three families has to ledger all 22 sections and skip the
14 that belong to its siblings — 172 ledger entries across the project to cover 108 real
sections, with 64 of them carrying no information and free to disagree. The `assignments:`
block states each section's owner **once**, and each family then ledgers only what it owns.

### Three states, and the difference that matters

| State | Declared in | Meaning |
|---|---|---|
| owned by family F | `assignments: … owned: F: [...]` | F's ledger **must** cover it |
| unowned | `assignments: … unowned: {heading: reason}` | nobody ledgers it; the reason lives in the registry, once |
| owned but not mined | that family's `coverage:` `skipped:` | F owns it and **chose** not to mine it |

The last two look similar and are not. **Unowned is a taxonomy fact** — the section is out
of scope for every family, so recording it once is correct. **Owned-but-skipped is a
family's own judgement** — "this section is pure rationale", "this restates §3" — and it
belongs in that family's ledger where it is reviewed alongside the requirements it
explains. Collapsing the two loses the review.

---

## Modes

### `audit` — the default, and the one to run often

```bash
python3 <skill-dir>/scripts/validate-reqs.py --registry-only
```

Reports, as errors:

- a `spec_corpus:` file that no family claims and `excluded_sources:` does not exclude —
  **a whole spec chapter nobody owns**, which is the failure this verb exists to catch;
- a file both excluded and claimed;
- an `excluded_sources:` entry with no reason;
- a section assigned to no family and not listed `unowned:` — **a gap between families**,
  invisible to `validate` before this block existed;
- a section assigned twice, or both owned and unowned;
- an assignment naming an undeclared family, or a heading that does not exist in the file;
- a family that owns sections in a file it does not declare as a source, or declares a
  source it owns nothing in.

Drop `--registry-only` to get the same checks plus every family file. Families declared but
not yet extracted are reported as a **note**, not an error — that is the normal state
mid-migration.

### `assign <spec-file>` — the main working mode

1. Parse the file's sections in document order.
2. For each section, read enough of it to decide which declared family's boundary it falls
   inside — or that no family owns it.
3. Present the whole proposed assignment for **one approval**, section by section with the
   proposed owner and a one-line justification. Not one round trip per section.
4. On approval, write the `assignments:` block for that file.

Rules:

- **Assignment is per section, never per file.** Assigning a whole file to one family is
  what produces the `midcore.rst` problem in reverse — a family that owns 22 sections of
  which it cares about 6.
- **Propose `unowned:` sparingly and specifically.** "Non-normative framing" for a chapter
  intro is honest. "Not relevant" for a section with a `.. danger::` block is how a real
  obligation disappears — and unlike a family's `skipped:`, an unowned section is never
  reviewed again by anyone.
- **Never reassign a section whose current owner has already mined it.** Moving a section
  from `rename` to `rob` after `spec-rename.yaml` has requirements citing it means those
  requirements must be retired and reissued under new IDs, breaking every tag. Report the
  conflict and stop.

### `add <family>` — `extract`'s gate #1

1. Check the key is not already in use, **including by any retired ID** in any family file.
   A reused family name silently rehomes dead IDs.
2. Write `description:` (one line, the architectural boundary — not a list of modules) and
   `sources:`.
3. If the family takes sections from a file that already has an `assignments:` block, run
   `assign` on that file in the same change. A new family with no assigned sections fails
   `audit`.

### `survey <glob>` — bootstrap a new corpus

For a spec corpus with no registry yet. Read every file's section structure, propose
families, and say which files should be excluded and why. Present for approval; write
nothing until it is given.

Bias the proposal toward boundaries that line up with **both** the spec's chapter structure
and the module names in `src/main/nlhdl/hierarchy.yaml`, so that a family maps to code
someone can own. A taxonomy that matches only the documentation split produces families like
`midcore` that name no real boundary.

---

## What `curate` must refuse

**Rename a family.** The name is inside every ID and every RTL tag. There is no rename;
there is only retire-every-requirement-and-reissue-under-a-new-family, which invalidates
every tag pointing at the old ones. If the name is genuinely wrong and nothing has been
extracted yet, editing the key is free — verify that first, and say so explicitly when
proposing it.

**Split or merge a family with extracted requirements.** Same reason: the resulting
requirements need new IDs. The procedure, when it is genuinely warranted:

1. `curate add` the new family or families.
2. Re-extract the affected sections under the new family, producing new IDs.
3. Retire the old requirements with `retired_because` naming the new IDs as successors.
4. Leave the RTL alone. `validate` will flag every tag pointing at a retired ID, and that
   list is the retagging worklist.

Do that only when the boundary is actually wrong. It costs a churn of every affected tag.

**Delete a family.** Retire its requirements instead. Deleting the key makes every
`//@req-spec-<family>.*` tag unresolvable with no record of why — the same failure as
deleting a requirement instead of retiring it, one level up.

**Edit anything but the registry.** `curate` writes `families.yaml` and nothing else. It
does not create or edit `spec-<family>.yaml` files (that is `extract`'s), it does not touch
spec prose or anchors (that is `extract`'s anchor sweep), and it never touches RTL.
Additive changes only: never reorder families, never rewrite the file's comments.

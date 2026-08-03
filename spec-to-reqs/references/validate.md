# `validate` — mechanical checks

```
/spec-to-reqs validate            # all families
/spec-to-reqs validate cii lsu    # named families only
```

Run the bundled script. Do **not** perform these checks by reading files yourself:

```
python3 <skill-dir>/scripts/validate-reqs.py [family ...]
```

The script finds the repo root by walking up to `.git`, and defaults to
`src/main/nlhdl/reqs/` for the YAMLs and `src/main/nlhdl/` for the RTL tag scan. Override
with `--repo-root`, `--reqs-dir`, `--rtl-dir`. `--registry-only` checks `families.yaml`
alone — that is what `curate audit` runs.

Exit codes: `0` clean (notes and warnings allowed), `1` errors found, `2` bad invocation.

**Why a script and not a careful read:** every check here is string-in-file, set difference,
uniqueness, or regex. A model performing them is slower, non-deterministic, and — the actual
danger — *able to report a pass it did not perform*. A hallucinated quote-match is worse
than no check, because it will be believed. The script exits non-zero with a `file:line`
list, identically, every time.

---

## What it checks

### Errors

| Check | Catches |
|---|---|
| `quote:` occurs in `source.file`, whitespace-normalized | **Drift** — the spec was reworded under a requirement |
| `source.heading` exists as a real section | A reworded heading, or a citation pointing at the wrong file |
| `source.anchor` is defined in that file | A stale or invented anchor |
| `source.file` is listed in `spec_sources` | Citing a document the family does not claim to cover |
| Every section **assigned to this family** appears in `coverage:` | **Unmined sections** |
| No `coverage:` entry for a section assigned elsewhere | A family claiming a sibling's obligations |
| Every `coverage:` entry matches a real section | A renamed or deleted section, before its reqs are orphaned |
| Each `coverage:` entry has `reqs:` xor `skipped:` | A section neither mined nor consciously skipped |
| Each live req appears in exactly one `coverage:` entry | Requirements that escaped the ledger |
| ID format, uniqueness, family match, group declared | ID-namespace corruption |
| `kind` ∈ {function, interface, structure, timing} | Typos and invented kinds |
| `statement` non-empty and contains must/shall | Descriptive text smuggled in as a requirement |
| `spec_sources` equals `families.yaml` `sources:` | The one deliberate redundancy in the schema, guarded |
| Family declared in `families.yaml`; no stray `spec-*.yaml` | Families coined without approval |
| **Registry:** every `spec_corpus:` file is claimed by a family or excluded with a reason | **A whole spec chapter nobody owns** |
| **Registry:** every section of an assigned file is owned or explicitly `unowned:` | **A gap between families** — invisible to every other check |
| **Registry:** no section owned twice, or both owned and unowned | Two families claiming one obligation |
| **Registry:** assignments name declared families and real headings | Typos in the taxonomy |
| **Registry:** a family owns ≥1 section in each source it declares, and owns none outside them | `sources:` drifting from reality |
| **Registry:** no file both excluded and claimed | A contradictory corpus declaration |
| Retired entries have `id`, `statement`, `retired_because`; not also live | Tombstones with no explanation |
| Every `//@req-` tag resolves to a **live** ID | Tags pointing at retired or nonexistent requirements |

### Warnings (do not fail the run)

- `statement` longer than 40 words.
- `statement` containing " and " with more than one modal verb — likely two conjoined
  obligations that should be split.

### Notes

- Live requirements with **no** `//@req-` tag anywhere in the RTL tree. This is
  deliberately *not* an error: early in a subsystem's life every requirement is untagged,
  and an error there would make the check useless exactly when you most want to run it. Use
  `trace --unimplemented` to work the list down.
- Families declared in `families.yaml` with no `spec-<family>.yaml` yet. Also deliberately
  not an error — a curated taxonomy legitimately runs ahead of extraction, and erroring
  here would make `validate` unusable for the whole migration.
- `families.yaml` declaring no `spec_corpus:`, which disables the unclaimed-file check.

---

## Reading the failures

**`DRIFT: quote no longer occurs in …`** — the spec changed. Do **not** silently re-quote:
re-read the section, decide whether the requirement still holds, and then either update it
via an approved `extract` drift stop, or retire it. Re-quoting without reading is how a
spec rewrite retroactively changes what the RTL was tagged as satisfying.

**`section … is absent from … coverage ledger`** — a section this family owns but has not
mined. Run `extract` for the family; it will resume where it stopped.

**`section … is assigned to no family and not listed as unowned`** — usually a *new* spec
section, which means new obligations nobody owns yet. Run `curate assign` on the file. This
is the only check that detects added requirements; quote-matching cannot, because a quote
that still matches says nothing about text added next to it.

**`family 'X' ledgers a section assigned to 'Y'`** — either the ledger entry is wrong, or
the assignment is. If the section really is X's work, fix `families.yaml` via `curate` — but
if Y has already mined it, moving it means retiring Y's requirements and reissuing them
under X, which breaks every tag pointing at them. Decide deliberately.

**`no such section in … (heading reworded or removed)`** — the ledger references a section
that no longer exists. Fix the heading in both the ledger and any `source.heading` citing
it, in the same change.

**`tag references RETIRED req`** — implementation cleanup. The tag is a claim about a
requirement that no longer exists; `retired_because` names the successor, so retag or
delete.

**`tag references UNKNOWN req`** — a typo in the tag, or a requirement deleted instead of
retired. If the latter, the deletion was a mistake: restore it as a tombstone.

---

## Known limits

- Section detection handles reStructuredText **underline-style** headings (the style used
  throughout `docs_caracal/src/`) and Markdown ATX headings. Overline-style RST headings are
  not detected.
- A one-column RST simple-table border could in principle be mistaken for a heading
  underline. Multi-column borders cannot, because they contain interior spaces. If a false
  section appears, give it a `coverage:` entry with a `skipped:` reason noting it is a table
  artifact.
- The script never writes. Fixes are yours or `extract`'s.

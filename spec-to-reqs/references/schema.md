# The requirement YAML contract

Normative. All three verbs depend on this file. `scripts/validate-reqs.py` enforces
everything here that can be checked mechanically; the rest is enforced in review.

---

## 1. What is and is not a requirement

A requirement is **one obligation the RTL must fulfil**, stated at the level of *what the
design must do or contain* — and, because these are microarchitecture specs, that includes
structure. If the spec dictates that a mirror is maintained, a queue exists, or work is
split across particular stages, those are requirements.

The test is not "is this behavioural or structural?" — it is **"does the spec say it?"**

| Spec wording                                | Becomes                                                     |
|---------------------------------------------|-------------------------------------------------------------|
| must / shall / never / always / is required / imperative statements of fact about the design | **a requirement** |
| may / can / is permitted                     | `notes:` on the governing requirement, or a `coverage:` skip |
| should / is recommended                      | **stop and ask** — see below                                  |
| description, motivation, background, history | `coverage:` skip, reason `non-normative`                      |
| citations of existing BOOM code as evidence  | not a requirement on its own; may support one                 |

**`may` is an allowance, not an obligation.** Tagging code that satisfies a permission is
meaningless — it can never fail — and it inflates the count with entries that make coverage
figures look better than they are.

**`should` always stops the run.** In an internal microarchitecture spec, `should` is
usually a `must` written loosely, but sometimes it is a genuine preference. Guessing either
way is wrong: guess `must` and you fabricate an obligation; guess `may` and you drop a real
one. Ask, then record the answer in `notes:` so it is not re-litigated next run.

### Granularity

**One idea per requirement.** The operational form of that rule, and the one a reviewer can
check in seconds:

> If the statement needs " and " between two verbs, it is two requirements.

A 35 KB spec should be expected to yield on the order of 60–100 requirements. If a family
YAML has fifteen, the extraction was too coarse — go back and split.

---

## 2. Identifiers

```
spec-<family>.<group><n>
```

- `<family>` — lowercase, declared in `families.yaml`. Matches the filename
  (`spec-lsu.yaml`) and the `family:` key inside it.
- `<group>` — one or more lowercase letters, declared in that file's `groups:` map. A group
  is a subgroup of related obligations inside the family, so that requirements added later
  land next to their neighbours instead of at the end of a flat list.
- `<n>` — a positive integer, unique within the group.

Examples: `spec-decode.b3`, `spec-lsu.f9`, `spec-cii.a12`.

**IDs are immutable and never reused**, including after retirement. RTL tags depend on it.

RTL tag form, one per line, immediately above the implementing code:

```systemverilog
//@req-spec-cii.b2
//@req-spec-cii.b4
```

---

## 3. The registry: `families.yaml`

Written and audited by `curate` only. See `references/curate.md`.

```yaml
spec_corpus:
  - docs_caracal/src/*.rst

excluded_sources:
  - file: docs_caracal/src/usage.rst
    reason: every section is a bare TODO — no obligations yet
  - file: docs_caracal/src/diagrams.rst
    reason: figure includes only, no prose

families:
  rename:
    description: Vector and VL renaming — map tables, free lists, busy tables, snapshots
    sources:
      - docs_caracal/src/midcore.rst
  rob:
    description: ROB participation of vector uops — group-done, commit, exceptions
    sources:
      - docs_caracal/src/midcore.rst

assignments:
  - file: docs_caracal/src/midcore.rst
    owned:
      rename:
        - "The vector map table"
        - "Free list and refcounting"
      rob:
        - "Group-done tracking"
    unowned:
      "Chapter introduction": non-normative framing, no obligations
```

| Key                | Required | Meaning                                                          |
|--------------------|----------|------------------------------------------------------------------|
| `families`         | yes      | family key → `description` + `sources`. The ID namespace.         |
| `spec_corpus`      | no       | globs bounding the spec corpus. Without it, a spec file that no family claims cannot be detected. |
| `excluded_sources` | no       | corpus files deliberately not mined, each with a reason.          |
| `assignments`      | no       | per spec file, which family owns which section. See below.        |

Families are **architectural boundaries and you own them.** `extract` assigns each
requirement to a family already declared here; if a requirement fits none, it stops and
proposes a new one rather than inventing `spec-misc.a1`. A model that coins families
per-run puts the same obligation in `decode` today and `frontend-decode` next month, and
every existing tag rots.

### `assignments` — section ownership

A spec file is routinely shared by several families: `midcore.rst`'s 22 sections split
across `rename`, `rob` and `vrf`. `assignments:` states each section's owner **once**, and
each family's `coverage:` ledger then covers only what it owns.

Without it, every family sharing a file must ledger *all* of that file's sections and skip
its siblings' — 172 ledger entries across this project to cover 108 real sections, 64 of
them carrying no information and free to disagree with each other.

- Every section of an assigned file must appear exactly once, under `owned:` or `unowned:`.
  A section in neither is an error: it is a **gap between families**, and it is the one
  failure no other check can see.
- `unowned:` needs a reason per section. It means *no family will ever mine this*, so the
  reason is the only record.
- A family may not own sections in a file it does not declare as a source, and may not
  declare a source it owns nothing in.
- **Assignment is per section, never per file.**

`assignments:` is optional **per file**, so a registry can be migrated one spec file at a
time. A file with no assignments block falls back to the legacy rule: every family sourcing
it ledgers all of its sections.

### What the registry deliberately omits

`families.yaml` does **not** map families to RTL directories. `trace` greps the whole of
`src/main/nlhdl/`, which is exact and layout-independent; a path list would couple the
registry to a tree that is still being built and would produce false failures the first
time a file moves.

---

## 4. The family file: `spec-<family>.yaml`

```yaml
family: cii
description: Vector arithmetic offload over the TT-CII coprocessor interface
spec_sources:
  - docs_caracal/src/cii.rst

groups:
  a: channel structure and credit protocol
  b: flush and kill behaviour
  c: operand delivery

reqs:
  - id: spec-cii.b1
    kind: interface
    statement: >
      The CII adapter must trigger kill only on rob.io.flush.valid, never on
      brupdate.b2.mispredict.
    source:
      file: docs_caracal/src/cii.rst
      anchor: cii-flush
      heading: "The kill contract"
      quote: >
        **Trigger — ``rob.io.flush.valid`` only.** Never ``brupdate.b2.mispredict``
    notes: >
      The spec requires this be asserted, not merely assumed.

coverage:
  - file: docs_caracal/src/cii.rst
    heading: "The kill contract"
    anchor: cii-flush
    reqs: [spec-cii.b1]
  - file: docs_caracal/src/cii.rst
    heading: "The VPU (coprocessor) side"
    skipped: coprocessor-internal, outside the host-side scope of this family

open_questions:
  - question: >
      The spec does not state whether a killed tag's side-table entry may be reclaimed
      before its last beat if the VPU is known idle.
    source:
      file: docs_caracal/src/cii.rst
      anchor: cii-flush
      heading: "The kill contract"

retired:
  - id: spec-cii.b7
    statement: >
      The adapter must compare tag age against the flush rob_idx.
    retired_because: >
      cii.rst "The kill contract" establishes that io.flush.bits carries no rob_idx and
      that no age comparison is needed. Superseded by spec-cii.b2.
```

### Top-level keys

| Key             | Required | Meaning                                                        |
|-----------------|----------|----------------------------------------------------------------|
| `family`        | yes      | Must match the filename and a key in `families.yaml`.           |
| `description`   | yes      | One line.                                                       |
| `spec_sources`  | yes      | Must equal the family's `sources:` in `families.yaml`, exactly. |
| `groups`        | yes      | Map of group letter → what that group covers.                   |
| `reqs`          | yes      | The live requirements.                                          |
| `coverage`      | yes      | The section ledger. See §6.                                     |
| `open_questions`| no       | Spec ambiguities. Not requirements.                             |
| `retired`       | no       | Tombstones. See §7.                                             |

`spec_sources` duplicates `families.yaml`. That redundancy is deliberate — the family file
should be readable alone — and the validator errors if the two disagree, so it cannot drift.

---

## 5. The requirement record

| Field       | Required | Meaning                                                       |
|-------------|----------|----------------------------------------------------------------|
| `id`        | yes      | §2.                                                            |
| `kind`      | yes      | One of `function`, `interface`, `structure`, `timing`.          |
| `statement` | yes      | The obligation. Rules below.                                   |
| `source`    | yes      | `file`, `anchor` (where one exists), `heading`, `quote`.        |
| `notes`     | no       | Clarification only. **Never introduces a new obligation.**      |

### `kind`

| Value       | Means                                                                        |
|-------------|-------------------------------------------------------------------------------|
| `function`  | Observable behaviour: given this input or state, this result.                 |
| `interface` | A contract at a named seam — signal semantics, handshake, ordering across a boundary. |
| `structure` | The design must contain this thing, or be organized this way.                 |
| `timing`    | Cycle counts, latency, throughput, pipeline-depth obligations.                |

All four are equally binding. `kind` is a reading and filtering facet, not a statement about
negotiability. It exists so a 90-requirement family stays navigable and so an implementor
can pull every `interface` requirement for the seam they are about to write.

### `statement` rules

1. **One sentence**, `must` or `shall`, present tense.
2. **Explicit subject.** "The vector decoder must …", not "illegal instructions are
   handled". A requirement with no named subject cannot be tagged, because nobody knows
   whose obligation it is.
3. **No conjoined obligations.** Two verbs joined by " and " means two requirements.
4. **No rationale.** Reasons go in `notes:`. Rationale inside a statement is how a
   requirement grows a second idea.
5. **≤ 40 words.** Warned, not errored.
6. **No number the spec did not state as a requirement.**

### `source` rules

- `file` — repo-relative path.
- `anchor` — the **nearest enclosing** `.. _name:` label. It may belong to an ancestor
  section; that is fine and expected. Omit only when the file has no enclosing anchor at
  all (`.md` sources, or a section whose anchor insertion was declined).
- `heading` — the **exact** heading text of the section the quote came from, which may be a
  subsection of the anchored one.
- `quote` — a **verbatim** substring of `file`. Reflowing whitespace is permitted because
  matching is whitespace-normalized. Nothing else may be altered: no ellipses, no
  corrections, no dropping of `**` or ``` `` ``` markup. One to three sentences is the
  useful range — long enough to be found unambiguously, short enough that an unrelated edit
  elsewhere in the paragraph does not trip a false drift report.

---

## 6. The coverage ledger

The ledger covers **exactly the sections `families.yaml` assigns to this family** — no more,
no less. Each entry carries either `reqs:` or `skipped:`; a section with neither is a
**validation error**, and ledgering a section assigned to a sibling family is also an error.

Where `families.yaml` has no `assignments:` block for a source file, the legacy rule
applies: the family ledgers *every* section of that file, skipping the ones that belong to
siblings.

**`skipped:` here means "this family owns the section and chose not to mine it"** — pure
rationale, a restatement of another section, background. It does **not** mean "this belongs
to another family": that is `unowned:`/sibling ownership in `families.yaml`, recorded once.
Keeping the two apart is what makes a skip reason reviewable — an owner's judgement call
sits next to the requirements it explains, rather than being lost among 64 mechanical
sibling-skips.

```yaml
coverage:
  - file: docs_caracal/src/cii.rst
    heading: "The kill contract"
    anchor: cii-flush
    reqs: [spec-cii.b1, spec-cii.b2, spec-cii.b3]
  - file: docs_caracal/src/cii.rst
    heading: "The VPU (coprocessor) side"
    skipped: coprocessor-internal, outside this family's scope
```

This exists so that **silence becomes visible.** Without it, a family with 60 requirements
is indistinguishable from a family where attention ran out at section 9, and "we never got
to §9" masquerades as "§9 had nothing in it."

Three things it catches that nothing else does:

- **Unmined sections** — an assigned section absent from the ledger, or present with
  neither `reqs:` nor `skipped:`. Hard error.
- **Added spec sections** — caught at the registry level: a new section belongs to no
  family, so `curate audit` errors before any family is asked to mine it. This is the only
  mechanism that detects *new* obligations. Quote-checking cannot: a quote that still
  matches says nothing about text added beside it.
- **Renamed headings** — a ledger entry with no matching section in the file is an error,
  which flags a reworded heading before its requirements are silently orphaned.

Skip reasons are claims, and they are reviewable. `skipped: out of family scope` is
something a reader can disagree with; a silent omission is not.

Every live requirement must be listed in exactly one ledger entry's `reqs:`.

---

## 7. Retirement

When a spec change invalidates a requirement, it moves to `retired:` — it is **not**
deleted, and it does **not** get a flag inside `reqs:`.

```yaml
retired:
  - id: spec-lsu.f4
    statement: >
      Unit-stride loads must be split into one request per element.
    retired_because: >
      loadstore.rst "Coalescing unit-stride accesses" replaced per-element splitting with
      cache-line coalescing. Superseded by spec-lsu.f9.
```

Deleting outright leaves any surviving `//@req-spec-lsu.f4` in the RTL resolving to **zero
hits** — indistinguishable from a typo, and answerable only by archaeology through git
history. A tombstone answers it in one line and names the successor.

A flag inside `reqs:` instead of a separate block poisons every count and filter: "how many
`interface` requirements does this family have?" should not depend on remembering to
exclude tombstones, because eventually someone forgets.

Rules:

- Retirement **never edits RTL**. The skill's job stops at the YAML.
- `retired_because` must name the spec change, and the successor requirement where one
  exists.
- The validator errors on any RTL tag pointing at a retired or unknown ID. That is the
  cleanup signal, and it lands on whoever next runs `validate`.

# Mode: architect — plan the design and write `hierarchy.yaml`

Invocation: `/nlhdl architect [<plan-doc>] [--reqs-dir <dir>] [--out <hierarchy.yaml>]`

Defaults: reqs dir `src/main/nlhdl/reqs/`, output `src/main/nlhdl/hierarchy.yaml`.

This is the **first** verb in the flow. It decides what modules exist before
anyone writes an nlhdl file:

```
reqs/*.yaml + plan.md  --architect-->  hierarchy.yaml  --gen-nlhdl-->  *.nlhdl.*  --gen-rtl-->  RTL
```

`architect` writes **only `hierarchy.yaml`**. It writes no nlhdl file, no RTL,
and never edits the requirement YAMLs or the spec.

## Inputs, in authority order

1. **The requirement corpus** — `reqs/families.yaml` plus each
   `reqs/spec-<family>.yaml` (see the `spec-to-reqs` skill). This is the
   authority on **what must be built**. Every live requirement is a claim on the
   design, and the map must say which module answers it.
2. **The plan document** — a milestone/implementation plan, if one exists. This
   is the authority on **how the work is organized**: scope boundary and
   non-goals, phases, the directory/naming layout, which pre-existing files are
   in play, and any ground rules or structural invariants the map must not
   violate. A plan often states the decomposition outright — prefer it.
3. **The spec** (`docs_caracal/src/*.rst` or equivalent) — read **only** where
   1 and 2 leave a structural question unanswered, and cite what you used.

The corpus is derived *from* the spec, so reaching for the spec is a fallback,
and finding an obligation there that no requirement covers is a **gap in the
corpus, not a licence to invent**. Report it and hand off to
`/spec-to-reqs extract`; do not add a module for an untracked obligation, and
never paste spec prose into `hierarchy.yaml` as though it were a requirement.

If the plan and the corpus disagree — the plan omits a structure the reqs
mandate, or adds one nothing requires — **report the conflict**. Do not silently
follow either. The plan can be wrong; so can the extraction.

## Workflow

1. **Inventory the requirements.** Read `families.yaml`, then every family file.
   Build the live-requirement list per family (`reqs:` minus `retired:`). Record
   the counts — they are the denominator of everything you report at the end.
2. **Read the plan.** Extract the scope boundary and non-goals, the layout
   convention (where sources and generated artifacts live, filename form), the
   named phases, the top-level module(s), and any invariant the decomposition
   must respect. Non-goals matter as much as goals: they are what populates the
   out-of-scope ledger.
3. **Inventory what already exists on disk.** For every file the plan implicates,
   check whether it is there. This — not the plan's wording — decides `mode:`:
   - absent → `mode: new`
   - present, written by this flow from an nlhdl source → `mode: edit_generated`
   - present, hand-written or third-party → `mode: edit_existing` (`target:`)
   - present, instantiated but never to be written → `blackbox: true`
   A mode assigned from the plan's prose without checking the filesystem is how
   a hand-written baseline file gets scheduled for overwrite.
4. **Decompose.** Draw module boundaries where the requirements do:
   - Cluster requirements that share state or a data structure into one module.
   - Put a module boundary at each seam that `kind: interface` requirements
     describe — those requirements are contracts, and a contract wants two
     identifiable sides.
   - Give shared declarations (types, constants, parameter bundles) their own
     `kind: package` nodes, since several modules bind to them.
   - Respect the plan's decomposition where it has one. Deviating is allowed
     when the requirements demand it, but say so in the report and leave a
     comment in the file.
   - Size the split so one module is one reviewable nlhdl file. A module that
     ends up owning 40 requirements is under-decomposed.
5. **Allocate every requirement.** Each live req goes in exactly one module's
   `reqs:` — or, if genuinely shared across a seam, in each module that must
   uphold it — or in the top-level `reqs_out_of_scope:` ledger with a reason.
   **A live requirement in none of those three places is an error**, and it is
   the one failure the rest of the flow cannot see: an unallocated requirement
   is never written into an nlhdl file, never tagged, and never missed.
6. **Fill in the structure.** For each node: `kind`, `source`, `mode`,
   `output:`/`target:`, `group`, `depends_on:` for packages, `instantiates:` for
   children, `clock`/`reset` where they differ from the defaults, and `top:`.
   Remember that an `edit_existing` node's `instantiates:` lists only the
   instances the edit *adds*.
7. **Write the file as text.** Do not round-trip through a YAML library — it
   strips comments and reorders keys, and the rationale comments are half the
   value of the map. Comment every non-obvious choice: why a module exists, why
   a boundary sits where it does, what a delta covers.
8. **Validate.** Run `inspect-hierarchy` on what you wrote and fix what it
   reports before handing over.
9. **Report** (see below).

## The `reqs:` field

```yaml
  vec_cii_flush:
    source: src/main/nlhdl/vec/cii/VecCiiFlush.nlhdl.scala
    mode: new
    output: src/main/scala/v4/vec/generated/VecCiiFlush.scala
    group: vec_cii
    reqs: [spec-cii.e1, spec-cii.e2, spec-cii.e3, spec-cii.e7]
```

- **Bare IDs, never tagged.** Write `spec-cii.e1` — **never** the same ID behind
  the tag prefix (`//@req-` immediately followed by it). `spec-to-reqs`'
  validator scans `.yaml` files under `src/main/nlhdl/` for that literal token,
  so a tagged ID here would count as an implementation claim, inflating coverage
  before a line of RTL exists. The same trap catches *comments about* tagging:
  spell a prefixed ID out inside `hierarchy.yaml`, even as a counter-example,
  and the grep will find it.
- **Allocation is not implementation.** `reqs:` here says *this module is
  responsible for this obligation*. A `//@req-<id>` tag in an nlhdl file or its
  generated RTL says *this code claims to satisfy it*. The gap between the two
  is exactly the work outstanding, which is why they are kept as separate
  records and why this one is a plan rather than a status.
- **Prefer one owner.** Allocate to several modules only when the obligation
  really is joint (a handshake contract binding both sides). Two owners means
  two places to keep in step, so leave a comment saying why.
- **Every ID must be live.** An ID that matches nothing in the corpus, or that
  sits in a family's `retired:` block, is an error — not a typo to smooth over.
- **Blackbox nodes take no `reqs:`.** Nothing this flow writes will ever tag
  them. Requirements that fall to external IP belong in the out-of-scope ledger.
- **A module with no `reqs:`** is allowed only for structural glue the plan calls
  for (a wrapper, a flattening shim). Say so in a comment; otherwise the empty
  list reads as an allocation someone forgot.

## The out-of-scope ledger

```yaml
# Live requirements this map deliberately does not allocate. Every live req is
# either in some module's `reqs:` or here — silence is not an option.
reqs_out_of_scope:
  - reqs: [spec-cii.h1, spec-cii.h2, spec-cii.h3]
    reason: coprocessor-internal (VPU side); this map covers the host only.
  - reqs: [spec-lsu.k4]
    reason: >
      Deferred past this milestone — plan §9 lists segmented-store coalescing
      as a non-goal.
```

Same logic as the coverage ledger in `spec-to-reqs`: a design that covers 60 of
90 requirements should not be indistinguishable from one where attention ran out
at requirement 60. A reason is a claim a reviewer can disagree with; an omission
is not.

## Re-running on an existing map

`architect` is **incremental**. An existing `hierarchy.yaml` is a decision
record, not a draft to regenerate:

- **Never rename or re-key a module** that already has an nlhdl source or
  generated RTL. The key is the RTL module name and appears in filenames.
- **Preserve existing comments and hand-edited fields.** Merge; do not rewrite
  the file from scratch.
- **New requirements** (from a later `extract`) get allocated to existing modules
  where they fit; propose a new module only when none does.
- **Retired requirements** are removed from `reqs:` — say which, so the reader
  knows whether the module's job just shrank.
- **Mode changes are load-bearing.** A node that flips `new` → `edit_generated`
  because its output now exists is routine; anything flipping to or from
  `edit_existing` changes what the generator is allowed to touch — call it out.
- Report the change as a diff against the previous map, not as a fresh file.

## Report

Lead with the allocation ledger, because it is the number that matters:

- **Requirements:** N live across F families → A allocated, S out of scope,
  U unallocated. **List every unallocated ID** — that list should be empty.
- **Modules:** count by `mode:` (`new` / `edit_generated` / `edit_existing` /
  `blackbox`), and by `kind:`. Name every `edit_existing` target explicitly:
  those are the pre-existing files this plan will modify.
- **Decomposition notes:** where you departed from the plan, and why.
- **Conflicts and gaps:** plan-vs-corpus disagreements, spec obligations with no
  requirement, requirements too vague to allocate. Each needs an owner and a
  next step (`/spec-to-reqs extract`, or a question to the user).
- **`inspect-hierarchy` result**, and the suggested next step — usually
  `gen-nlhdl` on the first modules in topological order.

## Output bar

- Every live requirement is allocated or ledgered, with none silently dropped.
- Every `mode:` was decided by looking at the filesystem, not by reading prose.
- Every module key matches its `source:` filename and its intended RTL module
  name; the `instantiates:`/`depends_on:` graph is acyclic and reaches `top:`.
- Rationale comments survive; the file explains itself to the next reader.
- `inspect-hierarchy` passes.

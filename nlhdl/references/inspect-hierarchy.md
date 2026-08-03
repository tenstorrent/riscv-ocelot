# Mode: inspect-hierarchy — validate & reason about a `hierarchy.yaml`

Invocation: `/nlhdl inspect-hierarchy [path/to/hierarchy.yaml]`
(defaults to the nearest `hierarchy.yaml` if no path is given).

Two jobs: **(A)** check a `hierarchy.yaml` for formatting/consistency problems,
and **(B)** answer questions about design organization and hierarchy. The
skill's bundled `hierarchy.yaml` is the schema reference — its comments document
every field.

## Schema recap

Top level: `version`, `defaults`, `clocks`, `resets`, `interfaces`, `top`,
`reqs_out_of_scope`, `modules`. Each entry under `modules:` is keyed by module
name and may carry: `kind`, `source`, `mode`, `output`, `target`, `group`,
`clock`, `reset`, `reqs`, `parameters`, `ports`, `blackbox`, `depends_on`, and
`instantiates` (a list of `{module, instance, count?, params?, connect?, clock?,
reset?}`).

`reqs:` lists the requirement IDs (from the corpus under `reqs/`) that a module
is responsible for; `reqs_out_of_scope:` ledgers the live requirements this map
deliberately leaves unallocated, with a reason each. Both are written by
`architect`. They record an **allocation**, not implementation — the
`//@req-<id>` tags in the nlhdl/RTL are the implementation claim.

`kind:` is optional and defaults to `module`. `kind: package` marks a shared
declaration unit (SV `package`/`svh`, Chisel `Bundle`/`object`/params case class)
that is depended on but never instantiated. `depends_on:` lists the packages a
node binds to — declaration-only edges, kept out of `instantiates:` so the
instance tree stays a tree.

`mode:` is **required on every non-blackbox module** and is what tells the
generator whether it is writing a file or editing one:
`new` (whole module, write `output:`), `edit_generated` (whole module, regenerate
`output:` this flow previously wrote), `edit_existing` (delta spec applied
in place to pre-existing `target:` RTL, no regressions).

Principle — **define once, instantiate many**: a module is defined once under
`modules:`; it is instantiated any number of times via other modules'
`instantiates:` lists. For nlhdl-backed modules the `.nlhdl` `<|begin_ports|>` /
`<|begin_parameters|>` sections are the source of truth; declare `ports:` /
`parameters:` inline **only** for `blackbox:` modules.

## Validation checklist (job A)

Report findings grouped as **errors** (block generation) vs **warnings**.

1. **Well-formed YAML** and required keys present (`version`, `top`, `modules`).
2. **Top exists.** Every name in `top` is a key in `modules`.
3. **Reference integrity.** Every `instantiates[].module` resolves to a defined
   module. Every `connect[].interface` resolves to an entry in `interfaces`.
   Every `clock`/`reset` name resolves to `clocks`/`resets` (or `defaults`).
4. **Acyclic.** The `instantiates` edges form a DAG. Report any cycle as an
   error (topological build order must exist; leaves generate first).
5. **Names match.** Each module key matches the RTL module name and the
   `<module_name>` in its `source` filename. Flag mismatches
   (e.g. key `sync_fifo` but `source: fifo.nlhdl.sv`).
6. **Blackbox rules.** `blackbox: true` modules must declare `ports:` (and any
   `parameters:`) inline and have no `source`/`mode`/`instantiates`. Non-blackbox
   modules must have a `source` and must NOT duplicate ports/params that belong
   in the `.nlhdl` file.
6b. **Mode rules.** Every non-blackbox module has a `mode:` of `new`,
   `edit_generated`, or `edit_existing`. A missing or unrecognized value is an
   **error** — the generator must not have to guess whether a file is safe to
   overwrite. Then, per mode:
   - `new` / `edit_generated`: `output:` set, no `target:`. `output:` should land
     under a `generated/` directory.
   - `new`: warn if `output:` already exists on disk — either the mode is stale
     (should be `edit_generated`) or the file is not this flow's to write.
   - `edit_generated`: error if `output:` does **not** exist on disk; the mode
     claims a prior generated artifact. Warn if the file lacks the generated
     header/marker of this flow — that suggests `edit_existing`.
   - `edit_existing`: `target:` set and existing on disk, and `output:` **not**
     set (the pre-existing file is the artifact). Its `source:` must carry an
     `<|begin_edit_scope|>` section — warn if absent, since the generator then
     has no stated bound on the edit. Warn too if the delta spec restates a full
     port list, which usually means it was written as a whole-module spec by
     mistake.
   - Error if a `target:` path is also some other module's `output:`, or if two
     modules name the same `target:` — that is two specs editing one file.
   - **`instantiates:` on an `edit_existing` node lists only the instances the
     edit ADDS**, not the target's complete instance list — the delta spec has no
     business restating instantiations it does not create. Consequence: the
     instance tree below an `edit_existing` node is intentionally incomplete, so
     do **not** report pre-existing sibling modules as "unreachable from `top`".
     Reachability is only meaningful across `mode: new` / `edit_generated` nodes.
6c. **Kind rules.** `kind:` is absent (⇒ `module`), `module`, or `package`; any
   other value is an **error**. For `kind: package`:
   - No `instantiates:` and no appearance in `top:` — a package is never
     instantiated. Either is an **error**.
   - No `ports:` required even when `blackbox: true`; a package declares types,
     not an interface, so there is nothing to instantiate against.
   - Every name in any node's `depends_on:` resolves to a defined `kind: package`
     entry. A `depends_on:` naming a `kind: module` is an **error** — use
     `instantiates:` for module edges.
   - Packages must sort before their dependents. `depends_on:` edges join
     `instantiates:` edges for the topological order, and a cycle across the
     combined graph is an error (rule 4).
   - Warn if a package has no dependents: either the map is incomplete or the
     declaration is dead.
6d. **Requirement allocation.** Only when a requirement corpus exists (default
   `src/main/nlhdl/reqs/`); skip these checks silently when it does not.
   - Every ID in any `reqs:` or `reqs_out_of_scope:` list resolves to a **live**
     requirement — present in some family's `reqs:`, absent from its `retired:`.
     An unknown ID is an **error**; a retired one is an error naming the
     tombstone's successor.
   - **Bare IDs only.** A `//@req-` prefix inside this file is an **error**: the
     `spec-to-reqs` validator greps `.yaml` for that token and would count the
     allocation as an implementation claim.
   - **Every live requirement is allocated exactly once** — to one module's
     `reqs:`, or to `reqs_out_of_scope:`. Unallocated is an **error** (nothing
     downstream can see it). In both places is an error. In several modules'
     `reqs:` is a **warning**: legitimate for an obligation binding both sides of
     a seam, suspicious otherwise.
   - Every `reqs_out_of_scope:` entry carries a non-empty `reason:`.
   - `blackbox: true` nodes carry no `reqs:` (**error** if they do — this flow
     never writes them, so the allocation could never be discharged).
   - **Warn** on a non-blackbox module with no `reqs:` at all: either it is
     structural glue (fine — expect a comment saying so) or an allocation was
     missed.
   - **Cross-check the tags.** Where a module's `source:` exists, grep it for
     `//@req-` tags: an allocated ID with no tag in the source is a **warning**
     (unimplemented, or `gen-nlhdl` has not run); a tag citing an ID *not*
     allocated to that module is a **warning** too — either the map is stale or
     the module is doing another module's job.
7. **Param overrides.** `params:` keys at an instance exist as parameters on the
   target module; expression references (e.g. `NUM_M: NUM_CORES`) resolve in the
   parent's parameter scope.
8. **Arrays.** `count:` references a defined parameter or a literal ≥ 1.
9. **Output paths.** `output:` paths are unique across modules and land under a
   `generated/` directory; no two modules write the same file. `target:` paths
   are likewise unique and must not collide with any `output:`.
10. **Cross-check with sources.** Where a `.nlhdl` source exists, its
    `<|begin_dependencies|>` list should agree with the module's `instantiates`
    targets. Flag dependencies present in one but not the other.

## Answering design questions (job B)

For questions about organization/hierarchy, derive answers from the map:
- **Elaborated tree** — walk `top` down through `instantiates`, expanding arrays
  and applying param overrides, to show the instance hierarchy.
- **Where-used / fan-in** — which modules instantiate a given module.
- **Build order** — the topological order generation would follow.
- **Clock/reset domains** — which instances sit in which domain and where CDC
  crossings occur (an instance whose `clock`/`reset` differs from its parent).
- **Reuse** — modules instantiated in more than one place.
- **Declaration impact** — for a `kind: package`, the fan-in over `depends_on:`
  answers "what breaks if this type/parameter changes?". Transitively closed, it
  is the blast radius of a bundle or constant edit.
- **Requirement allocation** — which module owns a given requirement, what a
  module owes, and which live requirements are allocated nowhere. Report an
  allocation as an allocation: "`vec_cii_flush` owns spec-cii.e1" is a plan,
  not evidence the behavior exists. For that, `/spec-to-reqs trace spec-cii.e1`
  greps the actual tags.
- **Ownership** — which modules this flow writes wholesale (`new`,
  `edit_generated`), which it only patches in place (`edit_existing`), and which
  it never touches (`blackbox`). Useful before a bulk regeneration: only
  `new`/`edit_generated` modules are safe to rebuild unattended.

Ground every answer in the file; if the map is incomplete or inconsistent, say
so and point at the specific entry rather than guessing.

## Report

Lead with a pass/fail summary and the error/warning counts, then the grouped
findings with the offending `modules:` key or line, then any requested
design-organization answers. Offer to fix the errors if the user wants.

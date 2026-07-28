# Mode: inspect-hierarchy — validate & reason about a `hierarchy.yaml`

Invocation: `/nlhdl inspect-hierarchy [path/to/hierarchy.yaml]`
(defaults to the nearest `hierarchy.yaml` if no path is given).

Two jobs: **(A)** check a `hierarchy.yaml` for formatting/consistency problems,
and **(B)** answer questions about design organization and hierarchy. The
skill's bundled `hierarchy.yaml` is the schema reference — its comments document
every field.

## Schema recap

Top level: `version`, `defaults`, `clocks`, `resets`, `interfaces`, `top`,
`modules`. Each entry under `modules:` is keyed by module name and may carry:
`source`, `output`, `target`, `group`, `clock`, `reset`, `parameters`, `ports`,
`blackbox`, and `instantiates` (a list of `{module, instance, count?, params?,
connect?, clock?, reset?}`).

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
   `parameters:`) inline and have no `source`/`instantiates`. Non-blackbox
   modules must have a `source` and must NOT duplicate ports/params that belong
   in the `.nlhdl` file.
7. **Param overrides.** `params:` keys at an instance exist as parameters on the
   target module; expression references (e.g. `NUM_M: NUM_CORES`) resolve in the
   parent's parameter scope.
8. **Arrays.** `count:` references a defined parameter or a literal ≥ 1.
9. **Output paths.** `output:` paths are unique across modules and land under a
   `generated/` directory; no two modules write the same file.
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

Ground every answer in the file; if the map is incomplete or inconsistent, say
so and point at the specific entry rather than guessing.

## Report

Lead with a pass/fail summary and the error/warning counts, then the grouped
findings with the offending `modules:` key or line, then any requested
design-organization answers. Offer to fix the errors if the user wants.

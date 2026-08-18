# The formal ledger contract — `formal-<family>.yaml`

Normative. All modes depend on this file. `scripts/validate-formal.py` enforces
everything here that can be checked mechanically; the rest is enforced in review.

---

## 1. What qualifies as checking a requirement

An assertion checks a requirement when **it can fail on RTL that violates the
requirement.** That is the whole test, and it is stricter than it looks.

| Requirement statement                                                   | A property that checks it                                   | Not a property that checks it |
|-------------------------------------------------------------------------|-------------------------------------------------------------|-------------------------------|
| "The decoder must set `is_rvv` for every RVV opcode."                    | `AssertProperty(isRvvOpcode(inst) \|-> uop.is_rvv)`          | `AssertProperty(uop.is_rvv \|-> isRvvOpcode(inst))` — the converse |
| "vsetvli must write VL in the same cycle it retires."                    | `AssertProperty(retire && isVsetvli \|-> vlWrite.valid)`     | `CoverProperty(retire && isVsetvli)` — reachability, not correctness |
| "The mirror must be restored on misprediction."                          | `AssertProperty(mispredict \|=> mirror === snapshot)`        | `AssertProperty(mispredict \|-> true.B)` — vacuous |

Three failure modes to check every property against before it lands:

- **Vacuity.** If the antecedent can never hold, the assertion passes forever
  and checks nothing. Every implication-shaped property must be paired with a
  `CoverProperty` on its antecedent, recorded in the same ledger row via
  `reachability:`. This is not optional bookkeeping — an unreachable antecedent
  is indistinguishable from a passing check in every report either tool produces.
- **The converse.** `A |-> B` and `B |-> A` are different obligations. Read the
  requirement's direction off its `statement:`, not off what is convenient to
  write. Where the requirement is genuinely biconditional ("if and only if"),
  that is two assertions, and both belong in the row.
- **Tautology.** A property whose expression constant-folds to `1'h1` in the
  emitted Verilog is checking nothing. `validate` greps the emitted `.sv` for
  exactly this and errors on it.

### One requirement, one row; one property, one row

A requirement may take several assertions. An assertion may discharge several
requirements — a single `|->` covering both "must set `is_rvv`" and "must set it
in the same cycle" is legitimate, and both rows cite it. What is *not* legitimate
is an assertion that appears in a row it does not actually bear on, to make the
count look better.

---

## 2. Identifiers

```
formal-<family>.<group><n>
```

- `<family>` — lowercase, matching the filename, the `family:` key inside it, and
  a family declared in `families.yaml`.
- `<group>` — the **same group letter as the requirements it checks.**
  `formal-decode.d3` checks `spec-decode.d*`. When one assertion spans groups,
  use the group of the first requirement in its `reqs:` list.
- `<n>` — a positive integer, unique within the group.

**IDs are immutable and never reused**, including after retirement.

Tag form in the checker file, one per line, immediately above the property:

```scala
//@formal-req-spec-decode.d3
//@formal-req-spec-decode.d4
AssertProperty(mispredict |=> mirror === snapshot, label = Some("vdecode_mirror_restore"))
```

Never `//@req-` — that namespace means *implements* and belongs to `/nlhdl`. See
the rules in `SKILL.md`.

---

## 3. The ledger file

```yaml
family: decode
description: >
  Properties for the frontend family: RVV opcode decode, vset placement, the
  speculative VCFG mirror and its recovery, and VL delivery.
reqs_source: src/main/nlhdl/reqs/spec-decode.yaml
layer: BoomSvaLayer

checkers:
  - name: VDecodeChecks
    file: src/main/scala/v4/vec/formal/VDecodeChecks.scala
    dut: VDecode
    dut_file: src/main/scala/v4/vec/generated/VDecode.scala
    signals:
      - { name: uop,        expr: "io.deq.uop",           kind: bundle }
      - { name: isRvv,      expr: "io.deq.uop.is_rvv",    kind: bool }
      - { name: mirror,     expr: "vcfg_mirror",          kind: bundle, internal: true }
    notes: >
      `vcfg_mirror` is a module-level register, in scope at the anchor.

assertions:
  - id: formal-decode.d3
    checker: VDecodeChecks
    label: vdecode_mirror_restore
    kind: assert
    reqs: [spec-decode.d7]
    statement: >
      On a branch misprediction the VCFG mirror must equal the snapshot value in
      the following cycle.
    property: >
      brupdate.b2.mispredict |=> (mirror.vtype === snapshot.vtype)
    reachability: formal-decode.d4
    notes: >
      Non-overlapping (|=>) because the restore is registered.

  - id: formal-decode.d4
    checker: VDecodeChecks
    label: vdecode_mispredict_seen
    kind: cover
    reqs: []
    covers_antecedent_of: formal-decode.d3
    statement: >
      A branch misprediction occurs while a vset is in flight.
    property: >
      brupdate.b2.mispredict && vset_inflight

coverage:
  - req: spec-decode.d7
    assertions: [formal-decode.d3]
  - req: spec-decode.a2
    unassertable: >
      Provenance requirement — "unchanged from BOOM v4" is a claim about the
      source's history, not about any signal. Checked by review and by git, not
      by a property.
  - req: spec-decode.a8
    assertions: [formal-decode.a2]
    partial: >
      Checks that VLEN is a power of two and ≥ 128; the requirement's full
      obligation includes the ELEN relationship, which the module does not expose.

retired:
  - id: formal-decode.d1
    label: vdecode_mirror_restore_old
    retired_because: >
      spec-decode.d7 was rewritten to a registered restore; superseded by
      formal-decode.d3.
```

### Top-level keys

| Key            | Required | Meaning                                                       |
|----------------|----------|---------------------------------------------------------------|
| `family`       | yes      | Matches the filename and a key in `families.yaml`.             |
| `description`  | yes      | One line, or a short block.                                    |
| `reqs_source`  | yes      | Path to the family's `spec-<family>.yaml`.                      |
| `layer`        | yes      | The bind layer. Always `BoomSvaLayer`; the key exists so a file states its own contract. |
| `checkers`     | yes      | One entry per checker file. See §4.                            |
| `assertions`   | yes      | Every property. See §5.                                        |
| `coverage`     | yes      | The requirement ledger. See §6.                                |
| `retired`      | no       | Tombstones. See §7.                                            |

---

## 4. The checker record

| Field      | Required | Meaning                                                            |
|------------|----------|--------------------------------------------------------------------|
| `name`     | yes      | Scala object name. Must be `<dut>Checks`.                          |
| `file`     | yes      | Repo-relative path under `src/main/scala/v4/vec/formal/`.           |
| `dut`      | yes      | The Chisel module class the checker is bound into.                  |
| `dut_file` | yes      | Repo-relative path of the DUT source — where the anchor goes.       |
| `signals`  | yes      | Every signal the checker's `apply` takes. See below.               |
| `notes`    | no       | Anything a reader needs about scope or naming.                     |

`signals:` is the checker's **interface, and it is the thing that keeps the coder
honest.** Each entry is `{ name, expr, kind }` where `name` is the `apply`
parameter, `expr` is the exact Chisel expression at the anchor site, and `kind`
is `bool`, `uint`, or `bundle`. Mark `internal: true` on anything that is not a
port, so a reader can see at a glance how deep into the module the properties
reach.

Enumerating signals explicitly, rather than passing the module (`Checks(this)`),
is deliberate: it bounds what the properties can observe to a reviewed list, and
it makes the coder's job a mechanical mapping instead of an exploration of the
DUT's internals.

---

## 5. The assertion record

| Field                  | Required | Meaning                                                    |
|------------------------|----------|------------------------------------------------------------|
| `id`                   | yes      | §2.                                                        |
| `checker`              | yes      | A `name` from `checkers:`.                                 |
| `label`                | yes      | SVA label. Unique across the family, `snake_case`, prefixed with the DUT. Immutable. |
| `kind`                 | yes      | `assert`, `assume`, or `cover`.                            |
| `reqs`                 | yes      | Requirement IDs this property **checks**. Non-empty for `assert`. Must be `[]` for `assume` (see below) and for a `cover`. |
| `statement`            | yes      | One sentence, plain English, saying what the property checks. |
| `property`             | yes      | The property in Chisel LTL, using `signals:` names. See §5.1. |
| `reachability`         | no       | For an implication-shaped assert: the ID of the cover proving its antecedent is reachable. **Required** whenever `property` contains `\|->` or `\|=>`. |
| `covers_antecedent_of` | no       | On a cover: the assert it backs.                            |
| `purpose`              | for a `cover` with no `covers_antecedent_of` | `functional` — deliberate scenario coverage, not backing any assert. |
| `justification`        | for `assume` | The requirement or interface contract that licenses the constraint, cited by ID in prose. An assume with no justification is an error. |
| `notes`                | no       | Clarification. Never a second obligation.                  |

### `reqs:` on an assume is always empty

**An assume checks nothing** — it constrains the environment so the asserts
around it mean something. Letting an assume carry `reqs:` would make it look like
coverage, and a requirement "covered" by an assume alone is a requirement whose
violation can never be detected: the property that would have caught it has been
assumed away.

So a requirement that is really an obligation on the *producer* is ledgered
`unassertable:` in this module's family, with the assume named in the reason:

```yaml
  - req: spec-skid.d1
    unassertable: >
      Obligation on the producer, not on this module. Modelled here as the
      environment assume formal-skid.d1 (skid_no_zero_input) so the downstream
      asserts are meaningful; verifying the producer belongs to the producer's
      family.
```

If the requirement is genuinely this module's and you find yourself reaching for
an assume to make it pass, that is mandatory stop #1 — the property is failing.

### 5.1 `property` rules

1. It is a **Chisel expression**, not SystemVerilog: `|->`, `|=>`, `###`,
   `.delay(n)`, `.delayRange(a,b)`, `.eventually`. See `references/implement.md`
   for the verified API surface.
2. It references only names declared in that checker's `signals:`. A property
   naming anything else is an error — that is how a coder ends up reaching into
   the DUT for a signal nobody reviewed.
3. **No `reset`, no `hasBeenReset` handling.** `AssertProperty` gates on the
   implicit disable automatically. Writing reset gating by hand produces a
   double-gated property that also misses the first post-reset cycle.
4. One obligation per property. If it needs `&&` between two unrelated
   consequents, it is two assertions with two labels — otherwise a failure
   report cannot say which half broke.

### 5.2 `kind` and how to pick a property shape

| Requirement `kind:` | Usual property shape                                       | Notes |
|---------------------|------------------------------------------------------------|-------|
| `function`          | `antecedent \|-> consequent`, or a bare invariant           | The default. Get the direction right (§1). |
| `interface`         | Handshake and ordering: `valid && !ready \|-> valid.delay(1)`, `fire \|=> …` | Most naturally expressed, and most valuable — these are the seams that break. |
| `structure`         | Usually **not** SVA. A width/parameter obligation is a Scala `require(...)` at elaboration; a "this thing exists" obligation is discharged by the code existing. | Ledger it `unassertable:` and name the `require` where one applies. Do not fabricate a runtime property for a compile-time fact. |
| `timing`            | `.delay(n)`, `.delayRange(a,b)`, `.eventually` for liveness | A latency requirement with a stated cycle count is exactly `##n`. Unbounded "eventually drains" is `s_eventually` and needs `-assert svaext`. |

---

## 6. The requirement ledger

`coverage:` covers **exactly the live requirements in `reqs_source`** — no more,
no less. Each entry carries `assertions:` or `unassertable:`; an entry with
neither is a hard error, and so is a requirement absent from the ledger entirely.

| Field          | Meaning                                                                |
|----------------|------------------------------------------------------------------------|
| `req`          | The requirement ID. Must be live in `reqs_source` (not retired).         |
| `assertions`   | ≥1 ID from `assertions:`, **all of `kind: assert`**. Covers and assumes never appear here — a cover attaches to the assert it backs via `reachability:`, and an assume discharges nothing at all. |
| `unassertable` | Prose reason. Mutually exclusive with `assertions:`.                    |
| `partial`      | Optional alongside `assertions:` — what part of the obligation is *not* checked. |

`partial:` exists because the honest answer is usually "most of it". A
requirement with three clauses and a property covering two is better recorded as
partial than as either fully covered (a lie) or unassertable (a bigger lie). The
validator reports partial counts separately, so a family's real coverage is
legible instead of averaged away.

This ledger exists so that **silence becomes visible.** Without it, a family
with 40 assertions is indistinguishable from a family where attention ran out at
group `d`, and "group `e` had nothing checkable in it" masquerades as "we never
got to group `e`."

---

## 7. Retirement

A property whose requirement was retired, or which was superseded by a better
property, moves to `retired:` with its `label` and a `retired_because`. It is not
deleted.

Deleting outright leaves any surviving `//@formal-req-` tag and any historical
coverage row keyed on that label resolving to nothing — indistinguishable from a
typo. A tombstone answers it in one line and names the successor.

The validator errors on any `//@formal-req-` tag pointing at a retired or unknown
requirement ID, and on any checker file containing a label that is retired.

# Caracal v2 — the NL_HDL authoring brief used for Phase N

Preserved verbatim. This is the brief every one of the ~50 Phase-N authoring agents was
given, alongside a per-node dossier (its `hierarchy.yaml` entry + its exact requirements
with spec quotes). It is kept because it documents the conventions the 58 specs were written
to — the package-node convention, the delta-spec rules, the requirement-tag contract and the
10 design invariants — which is what a regeneration or a new node must match.

See `caracal-v2-seam-review.md` for the outstanding work.

---

# NL_HDL authoring brief — Caracal Phase N (Stage 1)

You are authoring ONE `.nlhdl.scala` (or `.nlhdl.sv`) file for ONE node of the
Caracal vector design. ~50 sibling agents are authoring the other nodes in
parallel from this same brief. **Consistency with this brief is as important as
being right about your own module**, because Phase R reads all 58 files together
and checks every seam from both sides.

Repo root: `/proj_risc/user_dev/ading/chipyard_current/chipyard_try/generators/boom`
All paths below are relative to it. Work only inside it.

---

## 0. Read these first (in this order)

1. `/home/ading/.claude/skills/nlhdl/references/format.md` — the file format. Non-negotiable.
2. Your dossier: `<DOSSIER>` — your hierarchy.yaml entry + your exact requirements.
3. `src/main/nlhdl/hierarchy.yaml` — at minimum your own entry, your
   `depends_on:`/`instantiates:` targets' entries, and the `interfaces:` block.
   Its comments are **design decisions already made**; honour them, do not re-litigate.
4. The already-written exemplars in `src/main/nlhdl/pkg/` — match their tone,
   density and structure. `VectorParams.nlhdl.scala` is the model for a `new`
   node; `MicroOp.nlhdl.scala` is the model for an `edit_existing` delta spec.
5. The `.rst` sections your dossier cites, under `docs_caracal/src/`.
6. `docs_caracal/caracal-milestone-plan-v2.md` §5 (global ground rules) and §2
   (the five structural changes + the bug list). Skim; cite where relevant.

## 1. Output

Write exactly one file, at the `source:` path in your hierarchy.yaml entry.
Nothing else. Do not create directories elsewhere, do not touch
`hierarchy.yaml`, do not write RTL, do not touch another node's file.

**No RTL is generated in this phase, for any module.** You are writing a
specification in prose, not code.

## 2. File skeleton (copy this shape exactly)

```
/* <the Tenstorrent CONFIDENTIAL license header, verbatim from
   /home/ading/.claude/skills/nlhdl/template.nlhdl.sv> */

/*
  <ModuleName> — one-line statement of what it is.

  hierarchy.yaml: kind: <module|package>, mode: <new|edit_existing>,
  output/target <path>, package boom.v4.vec.generated.<...>
  depends_on <...>

  <For a package node: the PACKAGE NODE CONVENTION paragraph — see §6.>
  <For an edit_existing node: state plainly that this is a DELTA against
   pre-existing hand-written RTL and name the target file. See §7.>

  <===> callouts for the one or two things a reader must not get wrong.>

  Governing spec anchors: <file.rst `anchor`>, ...
*/

<|begin_module|>

  <|begin_parameters|>
  ...
  <|end_parameters|>

  <|begin_ports|>
  ...
  <|end_ports|>

  <|begin_logic|>
  ...
  <|end_logic|>

<|end_module|>

<|begin_perf|>
...
<|end_perf|>

<|begin_dependencies|>
...
<|end_dependencies|>

<|begin_edit_scope|>          <-- ONLY for mode: edit_existing
...
<|end_edit_scope|>
```

### Hard format rules

- `parameters` / `ports` / `logic` go INSIDE the module block. `perf` /
  `dependencies` / `edit_scope` go OUTSIDE it, at file scope. Getting this wrong
  is an error, not a style choice.
- Each delimiter appears **exactly once**. **NEVER write a `<|...|>` token inside
  a comment or prose** — a parser reads it as a real section marker. Refer to
  sections by name in words ("the parameters section"). This is a real trap; it
  already broke four files once.
- Body text is **descriptive prose**, not `//` comment lines and not code.
  Reserve `//` and `/* */` for annotations meant to survive into the generated
  RTL next to the relevant code (a non-obvious algorithm note, a bug warning).
- `edit_scope` is required for `mode: edit_existing` and forbidden otherwise.

## 3. Requirement tags — the thing most likely to be got wrong

Write `//@req-spec-<family>.<group><n>`, **one ID per line**, immediately above
the prose it applies to:

```
//@req-spec-lsu.e9
//@req-spec-lsu.e10
Under undisturbed policy the inactive-lane data is pre-loaded from stale_pvdest
on VRF read port R2, before the arriving elements are overlaid.
```

- **Every ID in your dossier must appear at least once.** A validator checks this
  and it will fail the batch.
- **Never tag an ID that is not in your dossier.** Another node owns it. If your
  module genuinely seems to satisfy someone else's requirement, say so in your
  report instead — do not tag it.
- A tag is a **claim that this description specifies that obligation**. Put it on
  the specific paragraph, never on the file header or on the whole module.
- Several IDs above one paragraph is fine, one line each.
- If an allocated ID has no natural home in your body, the module is
  under-specified or the allocation is wrong. **Say which in your report** and do
  not quietly drop the tag.

## 4. Content bar — what a good spec does here

- **Complete but not padded.** Every port the logic mentions is declared; every
  parameter used has a default and, where sensible, a legal range. State the
  clock/reset convention explicitly (Chisel default: posedge `clock`,
  **active-high synchronous** `reset` — say so).
- **Say WHY, not just what**, where a choice is non-obvious or was got wrong
  before. Your entry's comments and the plan's §2 bug list are the source for
  this. A spec that only restates the requirement teaches the next reader nothing.
- **Name real signals and real module names** — from hierarchy.yaml, from the
  `.rst`, from the baseline Chisel where you are editing it.
- **Synthesizable as written.** No behaviour that would force simulation-only
  constructs. If something cannot be expressed synthesizably, say so rather than
  specifying it anyway.
- **Do not invent scope.** No ports, parameters or features the requirements and
  your entry do not call for. Do not add a module. Do not change the design.
- Length guide: 150–300 lines. The pkg/ exemplars are 130–355. Being thorough is
  right; padding is not.

## 5. Design invariants you must not violate

These are design-wide and Phase R checks them across all files at once.

1. **`usingRVV` gates everything.** Every vector feature is conditional on
   `usingRVV` (a Scala `Boolean` from `BoomCoreParams`, NOT a hardware `Bool`),
   so that a vectors-off build emits RTL bit-identical to pre-Caracal BOOM v4.
   Absent, not tied-off. Do not gate on rocket's `usingVector` — different gate.
2. **One uOP per instruction; no frontend cracking.** An `OP.v` stays a single
   uOP through decode, rename, ROB and issue. Element expansion into `nOP.v`
   happens ONLY in the vector LS AGEN.
3. **⇒ THE VECTOR-LSU INVARIANT.** No module in the vector LSU may hold state
   scoped to "the current instruction", and **no module may export a `busy` that
   gates issue**. In-flight state lives only in the six `VecElemQueue` instances
   (capacity reserved at dispatch in program order) and the LCB's per-PRN
   assembly entries. A `busy`-style signal reaching an issue unit is a failed
   review regardless of measured performance. This is why v2 exists — the
   previous attempt had one FSM with a concurrency ceiling of 1.
4. **Group-done completion.** Every vector producer signals completion of a whole
   destination group **once**, carrying the group's member-PRN vector. One event
   drives three consumers: the ROB single-shot busy-clear, the vector Busy-Table
   clear, and the vector wakeup. No per-entry ROB completion counter.
5. **Precise exceptions, `vstart = 0`.** A faulting vector load/store traps with
   `vstart = 0` and restarts the WHOLE instruction. Mid-vector resume is not
   implementable here. `fault_elem` survives only as the cursor stop signal and a
   debug counter — it never reaches the ROB and never reaches `vstart`.
6. **Vector architectural CSR state is rocket's.** `vtype` (incl. `vill`), `vl`,
   `vstart`, `vxrm`, `vxsat`, `vcsr`, `vlenb` and `mstatus.VS` come from
   rocket-chip's `CSRFile` under `usingVector`. Caracal owns ONLY the speculative
   VCFG `vtype` mirror and the VL register file.
7. **`pvs3` and `stale_pvdest` are separate fields naming separate groups.** They
   coincide for RMW arithmetic and diverge for masked non-RMW ops, `vslideup` and
   `vcompress`. Never merge or reinterpret them.
8. **VRF ports are statically partitioned and the table is canonical**
   (`midcore.rst` `vrf-ports`): R0 index, R1 load mask, R2 `stale_pvdest`,
   R3 store data, R4 store mask+index, R5–R8 CII, W0/W1 load/LCB, W2 CII.
   **Nothing adds a VRF port.** Cite your port by number.
9. **No unit tests exist.** No `chiseltest`, no `*Spec.scala`. Validation is
   end-to-end VCS + Whisper cosim only. So every vec module emits guarded trace
   statements via the shared `VecTrace` package — mention this in your logic
   section where it matters (one line per key event, tagged with module name and
   `rob_idx`, gated on the `vecTrace` plusarg, off by default).
10. **Reuse BOOM's existing machinery** rather than adding parallel mechanisms:
    the INT/FP wakeup networks for scalar feeders, the `order_fail` replay path,
    the memory-dependence predictor, `SelectFirstN`, branch snapshots. Two new
    networks only: VL (`pvl`) and VECTOR (group-done).

## 6. If your node is `kind: package`

It is a shared declaration unit (Chisel `Bundle`/`object`/case class), not an
instantiable module. Follow the convention the pkg/ exemplars set:

- Open the summary with a "PACKAGE NODE CONVENTION" paragraph saying it emits
  declarations, has no I/O, and how to read the three sections.
- `parameters` = the fields/knobs. `ports` = **explicitly "None."** with one
  sentence saying why (a declaration unit has no I/O, no clock, no reset) — state
  it, do not leave the section empty. `logic` = the declarations themselves and
  any elaboration-time `require`s.

## 7. If your node is `mode: edit_existing`

Your file is a **DELTA SPEC**, not a description of the module.

- **Read the target file first.** Use its real class, field and signal names.
  Your dossier gives the `target:` path.
- Open the summary comment by stating this is a delta and naming the target file.
- Describe **only the change**. Never restate the existing port or field list —
  anything you do not mention is unchanged by definition. Say so explicitly.
- Keep the delta as small as the requirements allow. There is a stated
  **added-line budget** per file in the plan's §11 file-touch summary — find
  yours and state it in the summary comment.
- `instantiates:` on an edit_existing node lists only the instances the edit
  ADDS. Do not describe pre-existing instantiations.
- Your `edit_scope` section must have four labelled parts and must be specific
  enough for a generator to hold itself to:
  - **Target** — file and class/trait, matching `target:` exactly (write the path).
  - **In scope** — the specific fields, blocks, signals or ports the edit may touch.
  - **Must not regress** — what stays bit- and cycle-identical. Name the parts
    most at risk, by their real names. "Don't break anything" is not usable.
  - **Interface delta** — new or widened ports/parameters only, plus an explicit
    list of things that must NOT appear (a reviewer's reject list).

## 8. Report back (your final message IS the return value)

Be brief and factual. Include:
1. The path you wrote and its line count.
2. Requirements: N allocated, N tagged. Name any you could not place, and why.
3. **Seam assumptions** — anything you assumed about a NEIGHBOURING node's
   interface (a handshake, a width, a back-pressure direction, a port number, a
   field name). This is the single most valuable thing you can report: it is what
   Phase R will check from the other side, and a wrong assumption here is exactly
   the class of defect v2's methodology exists to catch.
4. Conflicts found: `.rst` vs hierarchy.yaml vs the plan, or a requirement you
   believe is mis-allocated or unimplementable as written. Do not silently
   resolve these — report them.
5. Assumptions you had to make (a default, a width, a reset convention).

Do not report success if you could not place every requirement.

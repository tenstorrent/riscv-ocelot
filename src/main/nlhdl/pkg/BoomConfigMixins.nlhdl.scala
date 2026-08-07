/*
Tenstorrent CONFIDENTIAL
__________________
 Tenstorrent Inc.
 All Rights Reserved.

NOTICE:  All information contained herein is, and remains
the property of Tenstorrent Inc.  The intellectual
and technical concepts contained
herein are proprietary to Tenstorrent Inc.
and may be covered by U.S., Canadian and Foreign Patents,
patents in process, and are protected by trade secret or copyright law.
Dissemination of this information or reproduction of this material
is strictly forbidden unless prior written permission is obtained
from Tenstorrent Inc.
*/

/*
  BoomConfigMixins — DELTA SPEC. Describes only the config fragments Caracal
  ADDS to src/main/scala/v4/common/config-mixins.scala, which is hand-written
  baseline BOOM v4.

  hierarchy.yaml: kind: package, mode: edit_existing,
  target src/main/scala/v4/common/config-mixins.scala. No `output:`.
  Budget: ~60 lines. depends_on VectorParams, BoomCoreParams.

  ===> THE EXISTING TIER MIXINS ARE NOT MODIFIED. `WithNSmallBooms`,
       `WithNMediumBooms`, `WithNLargeBooms`, `WithNMegaBooms`,
       `WithNGigaBooms` and every other fragment in this file keep their current
       bodies exactly. Vectors are added by COMPOSING a new fragment on top of a
       tier, never by editing the tier. This is what keeps every existing
       scalar config bit-identical, and it is the whole reason the delta is
       ~60 lines instead of a rewrite.

  Scope boundary worth stating: the Chipyard-level configs the plan names
  (`MediumBoomV4VectorConfig`, `MegaBoomV4VectorConfig`) live in Chipyard's
  BoomConfigs.scala, which is a DIFFERENT REPOSITORY and not a node in this map.
  This file provides the fragments those configs compose; it does not declare
  them.

  Governing spec anchors: overview.rst (Chipyard relationship), midcore.rst
  (Rename Map Table), loadstore.rst `dcache-arbiter`.
*/

<|begin_module|>

  <|begin_parameters|>
  No parameters of its own. This file declares Chipyard/CDE `Config` fragments,
  each of which sets fields of `BoomCoreParams` (see the BoomCoreParams delta)
  and constructs a `VectorParams`.

  //@req-spec-core.a4
  Every fragment added here must be an ordinary
  `org.chipsalliance.cde.config.Config` following the same shape as the existing
  fragments in this file: a `Config((site, here, up) => { case TilesLocated(...)
  => ... })` that maps over `up(TilesLocated(InSubsystem))` and rewrites the
  `BoomTileAttachParams`. That shape is what keeps Caracal composable with the
  rest of the Chipyard ecosystem — a fragment that replaced the tile list
  instead of mapping over it, or that read a global rather than `up(...)`, would
  work in isolation and break the moment it were composed with another
  Chipyard fragment.

  Concretely this means the vector fragment must be applicable as
  `new WithVector ++ new WithNMediumBooms(1) ++ ...`, in any position where a
  core-modifying fragment is legal, and must not require being first or last.
  <|end_parameters|>

  <|begin_ports|>
  Not applicable. Config fragments are Scala objects: no I/O, no clock, no reset,
  no hardware.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. `WithVector` — the composable vector fragment ----

  Add `class WithVector(...)` — a fragment that maps over the located tiles and,
  for each `BoomTileAttachParams`, rewrites `core` to set `enableVector = true`
  and `vector = Some(VectorParams(...))`, leaving every other field of that
  tier's `BoomCoreParams` untouched.

  It must also add the three vector issue queues to the tier's existing
  `issueParams` sequence — one `IssueParams` entry each for `IQ_V_LOAD`,
  `IQ_V_STORE` and `IQ_V_ALU`, with `numEntries` from `vecIssueEntries`,
  `issueWidth` from `vecIssueGrantWidth`, and `dispatchWidth` matching the
  tier's existing entries. APPEND to the sequence; do not rebuild it, or a tier's
  carefully-sized scalar queues would be silently replaced.

  Expose constructor arguments for the values a caller may reasonably want to
  vary — the physical register counts and the queue depths — each defaulting to
  the `VectorParams` default, so `new WithVector` with no arguments is the
  documented configuration.

  // The existing `require`s in HasBoomCoreParameters check exactly one
  // IQ_MEM/IQ_UNQ/IQ_ALU/IQ_FP entry each. They count by iqType, so appending
  // three entries with new iqTypes cannot trip them. Do not relax them.

  ---- 2. Per-tier width overrides ----

  //@req-spec-lsu.h2
  //@req-spec-lsu.a12
  The D$ request lane count `lsuWidth` must be 1 on the Medium tier and 2 on the
  Large and Mega tiers, so that those two tiers can issue two memory operations
  per cycle. Add tier-specific fragments — `WithLargeBoomsVector` and
  `WithMegaBoomsVector`, or equivalently arguments to `WithVector` — that set
  `lsuWidth = 2` and, in step with it, `dcacheArbiterMode = "dual-dynamic"` and
  `vecIssueGrantWidth = 2`. On Medium, all three keep their defaults (1, 1 and
  "single").

  // ===> DISCREPANCY WITH THE BASELINE, and it must be resolved in the vector
  // config only. `WithNMegaBooms` already sets lsuWidth = 2, but
  // `WithNLargeBooms` does NOT — it leaves lsuWidth at its default of 1, so the
  // scalar Large tier today issues one memory op per cycle. loadstore.rst
  // requires 2 for Large. The fix belongs in the VECTOR fragment, which raises
  // lsuWidth for the Large vector config; `WithNLargeBooms` itself must not be
  // touched, because changing it would alter scalar LargeBoomV4Config RTL and
  // break gate (f) for a config that has nothing to do with vectors.
  // This is legal on Large: `require(memWidth >= lsuWidth)` holds because the
  // Large tier's IQ_MEM issueWidth is already 2. Flag it to the reviewer — if
  // the intent was that scalar Large should also be dual-issue, that is a
  // separate baseline change with its own gate, not part of this delta.

  ---- 3. Sub-flag fragments, so tracks land independently ----

  Add small fragments that set `enableVectorArith` and `vecScalarSnoopEnable`
  independently of `enableVector`, so that an intermediate machine with a working
  vector LSU and no coprocessor attach is expressible as a config rather than as
  a source edit. The BoomCoreParams delta requires that neither can be set
  without `enableVector`.

  ---- 4. The committed rename map table ----

  //@req-spec-rename.c1
  The committed rename map table must always be enabled, and no fragment added
  here may introduce an option that disables it. BOOM v4's
  `rename-maptable.scala` declares `com_map_table` unconditionally — there is no
  knob, unlike BOOM v3's `enableCommitMapTable` — so this obligation is
  discharged by NOT adding one.

  This is not a vacuous statement in context: vector recovery depends on the
  committed map table specifically. On an exception or pipeline flush the vector
  RMT is restored by a single-cycle copy from its committed counterpart, and the
  free list returns a faulting op's whole destination group by the same path. A
  configuration that made the committed table optional would make vector flush
  recovery unimplementable, so it must remain impossible to express.

  // Reviewer note: this is the thinnest requirement tag in the pkg/ set — it is
  // a claim about the ABSENCE of a config option rather than about code. If the
  // reviewer prefers, spec-rename.c1 is a reasonable candidate to move to the
  // hierarchy.yaml out-of-scope ledger as "satisfied by baseline BOOM v4 having
  // no such knob". It is tagged here rather than dropped because this file is
  // where such a knob would be added if anyone ever added one.

  ---- 5. What must NOT change ----

  No existing fragment in this file is edited. In particular the tier mixins
  keep their `fetchWidth`, `decodeWidth`, `numRobEntries`, `issueParams`,
  physical register counts, LDQ/STQ sizes, `maxBrCount`, FTQ, `nPerfCounters`,
  `fpu`, and their `DCacheParams`/`ICacheParams` exactly as they are.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
No behaviour of its own; the fragments only choose parameters. Two constraints
follow from what those parameters cost, and both are the reason the per-tier
overrides exist at all rather than one global setting:

Raising `lsuWidth` to 2 doubles the D$ request lanes, the LCAM ports and the TLB
ports the arbiter drives, and turns on the LCB's second write port (VRF `W1`).
That is area the Medium tier must not pay, which is why the override is per tier
and defaults off.

Raising `vecIssueGrantWidth` to 2 doubles each vector issue queue's grant
comparator tree. Since the vector queues share the single scheduling stage with
the scalar ones, this lands directly in the issue critical path — so it is
enabled only on the tier whose scalar queues are already that wide.
<|end_perf|>

<|begin_dependencies|>
VectorParams — constructed by the `WithVector` fragment.
BoomCoreParams — the fields the fragments set (`enableVector`, `vector`,
`enableVectorArith`, `vecScalarSnoopEnable`) and `lsuWidth`.
ScalarOpConstants — for the `IQ_V_LOAD`/`IQ_V_STORE`/`IQ_V_ALU` identifiers used
in the appended `IssueParams` entries.

Instantiates nothing. Consumed from outside this repository by Chipyard's
BoomConfigs.scala, which composes these fragments into the named configs.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File src/main/scala/v4/common/config-mixins.scala (package boom.v4.common).
    Hand-written baseline BOOM v4.

  In scope:
    - ADDING new `class With...` Config fragments at the end of the file:
      the vector fragment, the per-tier width overrides, and the sub-flag
      fragments described above.
    - Adding the `import` needed for VectorParams.

  Must not regress:
    - NO EXISTING FRAGMENT IS EDITED. `WithNSmallBooms`, `WithNMediumBooms`,
      `WithNLargeBooms`, `WithNMegaBooms`, `WithNMegaTapeoutBooms`,
      `WithNGigaBooms`, the CS152 fragments, `WithTAGELBPD` and every other
      existing fragment keep their current bodies byte-for-byte.
    - In particular `WithNLargeBooms` keeps `lsuWidth` UNSET (defaulting to 1).
      The Large-tier lsuWidth = 2 requirement is satisfied by the new vector
      fragment, not by changing the scalar tier. See the discrepancy note in the
      logic section.
    - `WithNMegaBooms` keeps its existing `lsuWidth = 2` — the vector fragment
      must be idempotent there, not additive.
    - Every existing scalar config composed from this file elaborates to
      bit-identical RTL. This is gate (f) and it covers Small, Medium, Large and
      Mega, not only the tier under test.
    - Existing `issueParams` sequences are appended to, never rebuilt or
      reordered; the four scalar entries keep their widths and entry counts.
    - The file's copyright header, the `// DOC include start/end` markers around
      the tier fragments, and the existing comment and formatting style are
      preserved.

  Interface delta:
    NEW: a composable vector Config fragment; per-tier width-override fragments
         setting lsuWidth = 2, dcacheArbiterMode = "dual-dynamic" and
         vecIssueGrantWidth = 2; and fragments for the `enableVectorArith` and
         `vecScalarSnoopEnable` sub-flags.
    No existing fragment, class name or parameter is changed, widened or removed.
<|end_edit_scope|>

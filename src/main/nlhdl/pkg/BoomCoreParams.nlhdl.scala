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
  BoomCoreParams — DELTA SPEC. Describes only what Caracal ADDS to
  `case class BoomCoreParams` and `trait HasBoomCoreParameters` in
  src/main/scala/v4/common/parameters.scala, which is hand-written baseline
  BOOM v4.

  hierarchy.yaml: kind: package, mode: edit_existing,
  target src/main/scala/v4/common/parameters.scala. No `output:`.
  Budget: ~90 lines. depends_on VectorParams.

  ===> THIS FILE DEFINES `usingRVV`, THE GATE THE ENTIRE DESIGN HANGS OFF. Every
       other vector node's `usingRVV`-conditional behaviour resolves to the one
       derived value declared here, so this delta is what makes the plan's
       central promise — "vectors off means bit-identical to BOOM v4" —
       mechanically true rather than aspirational.

  Nothing existing in the file is touched: none of the ~90 constructor fields,
  none of the derived widths, none of the `require`s. The additions are new
  fields with defaults that preserve current behaviour, plus new derived values.

  Governing spec anchor: overview.rst (the usingRVV gate and BOOM v4
  equivalence), frontend.rst `vector-csr-ownership`.
*/

<|begin_module|>

  <|begin_parameters|>
  ---- New constructor fields on `case class BoomCoreParams` ----

  Add `enableVector: Boolean = false`. This is the master switch for every
  vector feature in the design. It defaults to FALSE, which is what makes the
  addition safe: every existing config constructs `BoomCoreParams` without
  naming it and therefore gets exactly its current machine.

  Add `vector: Option[VectorParams] = None`, carrying the vector sizing
  parameters. Two fields rather than one because they answer different
  questions — whether the vector datapath exists at all, and how big it is —
  and because `Option[VectorParams]` alone could not express "vectors enabled
  with default sizing" without a sentinel.

  ---- Rocket-chip CoreParams fields that must also be set ----

  `BoomCoreParams extends freechips.rocketchip.tile.CoreParams`, and rocket's
  `HasCoreParameters` derives `usingVector` from `useVector`. Caracal delegates
  all vector ARCHITECTURAL CSR state to rocket's `CSRFile`, which only
  instantiates that state under `usingVector`. So a vector-enabled config must
  also set:
    - `useVector = true`   — so `CSRFile` instantiates vtype/vl/vstart/vxrm/
                             vxsat/vcsr/vlenb and the mstatus.VS machinery.
    - `vLen`, `eLen`       — from the `VectorParams`, so rocket's `VType`
                             derives `max_vsew` and `vlMax` from the same
                             numbers Caracal does.

  These are `override def`s in the case class body rather than new constructor
  fields, derived from `vector`, so a config cannot set them inconsistently with
  the `VectorParams` it passed.

  ===> `usingVector` AND `usingRVV` ARE TWO DIFFERENT GATES AND MUST NOT BE
       CONFLATED. `usingVector` is rocket's, and it means "the architectural
       vector CSRs exist". `usingRVV` is Caracal's, declared below, and it means
       "the Caracal vector datapath exists". Caracal's pipeline gates on
       `usingRVV` throughout. They happen to be set together by every config in
       this design, but they are not synonyms: gating vector RTL on
       `usingVector` would couple Caracal's datapath to a rocket-chip
       parameter that a future config could set for an unrelated reason.

  Note that rocket-chip's `if (usingVector)` block in `HasCoreParameters`
  already requires `isPow2(vLen)`, `eLen >= 32`, `vLen % eLen == 0` and
  `eLen == 32 || eLen == 64`. Do not restate those requires — VectorParams says
  so too, and a duplicated require that drifts is worse than none.
  <|end_parameters|>

  <|begin_ports|>
  Not applicable. A Scala case class and a mixin trait: no I/O, no clock, no
  reset.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. `override def hasV` ----

  Add `override def hasV: Boolean = enableVector` to the `BoomCoreParams` body.

  Rocket's `CoreParams` defines `def hasV: Boolean = vLen >= 128 && eLen >= 64
  && vfLen >= 64`, and `hasV` is what drives the `V` letter in the `misa` CSR
  string.

  // ===> BUG FROM THE PREVIOUS ATTEMPT, and it is not obvious from reading
  // either side. Caracal sets vLen = 256 and eLen = 64, so the first two terms
  // hold — but `vfLen` stays 0, because vfLen describes a rocket-style vector
  // FP unit that Caracal does not have (vector FP arithmetic happens in the VPU
  // behind the CII, which rocket knows nothing about). The default therefore
  // evaluates FALSE and misa.V was never advertised, so software could not
  // detect the vector unit. Overriding hasV is the fix; raising vfLen would not
  // be, since `require(vfLen <= eLen)` and the vfh checks would then describe a
  // datapath that is not there.
  // The Whisper reference model's misa reset in boom.json must match this: keep
  // V, drop the spurious X.

  ---- 2. Derived values on `trait HasBoomCoreParameters` ----

  //@req-spec-core.a7
  Add `usingRVV`, a `Boolean`, derived as `boomParams.enableVector`. THIS is the
  gate every vector feature in the design is conditioned on — not `enableVector`
  read directly, and not `usingVector`. One name, declared once, so that a
  reviewer can grep for it and find every vector-conditional site.

  Add `vectorParams`, resolving `boomParams.vector` to a concrete
  `VectorParams`, using the default instance when `enableVector` is true and
  `vector` is `None`. Accessing it when `usingRVV` is false must fail loudly at
  elaboration rather than silently yield defaults — a module reading vector
  sizing in a non-vector build is a gating bug, and the whole point of this file
  is to catch that at compile time.

  Then re-export the sizes the rest of the design reads, so no module reaches
  through `boomParams.vector.get`:
    `numVecPhysRegs`, `vecPregSz`, `numVlPhysRegs`, `vlPregSz`, `vecVLen`,
    `vecELen`, `maxVecVL`, `vecVLSz`, `maxVecMembers`, `ciiTagBits`,
    `ssiQueueEntries`, `usQueueEntries`, `lcbEntries`, `dcacheArbiterMode`.

  These follow the file's existing convention exactly — the surrounding code
  already re-exports `numIntPhysRegs`, `numFpPhysRegs`, `lsuWidth` and
  `maxPregSz` the same way, so this section should read as more of the same
  rather than as a new mechanism.

  ---- 2b. What VectorParams states but cannot itself express (added at A2) ----

  Three obligations are written in the VectorParams spec and DELEGATED here,
  because each is defined in terms of a BoomCoreParams quantity — `aluWidth`,
  `coreWidth`, `lsuWidth` — that a zero-dependency `case class VectorParams`
  cannot see. They are not new obligations and not a scope increase; only their
  location moved, and VectorParams carries a `DELEGATED (A2)` note at each site
  pointing here. All three are `usingRVV`-gated, so a vectors-off build is
  unaffected and gate (f) is untouched.

  1. Derive `numVlWakeupPorts = aluWidth + 1` on the trait — a per-ALU vset
     writeback plus the vleff trim (decision D8, replicate rather than
     arbitrate). It is DERIVED here, never a `VectorParams` field: making it a
     field would force it to be mandatory (no honest tier-independent default),
     and that would make `VectorParams()` — the default instance section 2 above
     is specified to construct — uncompilable.

  2. Require `numVlPhysRegs >= 1 + coreWidth`, so a full dispatch group of VL
     producers can allocate plus the one committed pointer.

  3. Require `ldResvMembers * vecVLen / 8 >= lsuWidth * 2` (the `eew_min = 8`
     case, which is the binding one), so a tier that widens `lsuWidth` without
     revisiting the load reservation quantum fails the build rather than
     silently starving the drain and undercutting target P2.

  // Note the shape difference: (1) is a derived value the design reads, (2) and
  // (3) are pure elaboration checks that read nothing. Both kinds belong on the
  // trait for the same reason — it is the first scope where both operands exist.

  ---- 3. Sub-flags, so tracks can land independently ----

  Add two further `Boolean` fields to `BoomCoreParams`, both defaulting to
  false, and derive a value for each on the trait:
    - `enableVectorArith` — attaches the CII coprocessor and enables the
      `IQ_V_ALU` grant path. With it false the vector LSU works and vector
      arithmetic does not, which is a valid intermediate machine.
    - `vecScalarSnoopEnable` — enables bidirectional cross-LSU disambiguation.

  Require that neither is true unless `enableVector` is: a sub-flag on its own
  would elaborate a coprocessor attach with no vector rename behind it.

  ---- 4. What must NOT be added ----

  ===> No cracker parameters of any kind. There is no frontend cracking to
       configure: a vector instruction is one uOP through decode, rename, the
       ROB and issue, and element expansion happens only in the vector LS AGEN.
  ===> No `numVecTmpGroups` or any other free-list headroom parameter. No
       headroom is needed for forward progress, because rename is in program
       order, so an OP.v that cannot allocate simply stalls. The actual
       requirement is that `pvdest` + `pvtmp` allocation is all-or-nothing,
       which is VecFreeList's business and not a parameter.

  ---- 5. The equivalence property this file is responsible for ----

  //@req-spec-core.a8
  With vectors disabled the core must elaborate as stock BOOM v4. Concretely,
  and this is the acceptance test for the whole delta: a `BoomCoreParams`
  constructed exactly as it is today — no `enableVector`, no `vector` — must
  produce RTL bit-identical to the pre-Caracal baseline, ignoring only `@[...]`
  source locators and `$error` message line numbers.

  That holds if and only if every addition here is either a new field with a
  behaviour-preserving default, or a derived value that nothing reads when
  `usingRVV` is false. It does NOT hold automatically, so the property is a
  gate, not an assumption: this is gate (f), and it is checked on
  `MediumBoomV4Config` at every step, not only at the end.

  // The one place to watch is the IQ_SZ widening in the ScalarOpConstants
  // delta, which is NOT gated (a constants trait has no Parameters in scope).
  // If gate (f) ever fails, that is the first thing to check, and the fallback
  // is to make IQ_SZ derived — which would in turn have to be declared here.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
No behaviour, so no throughput or latency target. The constraint that matters is
elaboration-time: every value added here must be a Scala `Boolean`, `Int` or
`String` resolved before hardware generation, never a signal. `usingRVV` in
particular must be a Scala `Boolean` and not a `Bool`, so that a
`usingRVV`-conditional block is ABSENT from the emitted Verilog rather than
present-and-tied-off. That distinction is the difference between passing and
failing gate (f).
<|end_perf|>

<|begin_dependencies|>
VectorParams — the `vector: Option[VectorParams]` field and every re-exported
size.

Extends `freechips.rocketchip.tile.CoreParams` and
`freechips.rocketchip.tile.HasCoreParameters`, as it already does; the delta
adds `override def hasV` and the `useVector`/`vLen`/`eLen` overrides to that
existing relationship.

Instantiates nothing. Its dependents include BoomConfigMixins and, through
`HasBoomCoreParameters`, essentially every module in BOOM — which is why
`usingRVV` is declared here rather than anywhere more local.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File  src/main/scala/v4/common/parameters.scala
    Both `case class BoomCoreParams(...) extends CoreParams` and
    `trait HasBoomCoreParameters extends HasCoreParameters`
    (package boom.v4.common). Hand-written baseline BOOM v4.

  In scope:
    - Adding four constructor fields to `BoomCoreParams`: `enableVector`,
      `vector`, `enableVectorArith`, `vecScalarSnoopEnable`, each with a
      behaviour-preserving default. Add them inside the existing
      `// DOC include start/end: BOOM Parameters` region, following the file's
      current field ordering convention.
    - Adding to the `BoomCoreParams` body: `override def hasV`, and the
      `useVector`/`vLen`/`eLen` overrides derived from `vector`.
    - Adding a new derived-values block to `HasBoomCoreParameters`: `usingRVV`,
      `vectorParams`, the re-exported sizes, and the two sub-flag values, plus
      the `require`s stated in the logic section.
    - Adding the `import` for VectorParams.

  Must not regress:
    - Every existing constructor field of `BoomCoreParams` keeps its name, type,
      default and position semantics. Since callers use named arguments this is
      about defaults, not order, but do not reorder existing fields regardless.
    - The existing body values are untouched: `xLen`, `haveFSDirty`,
      `pmpGranularity`, `instBits`, `lrscCycles`, `retireWidth`,
      `nPTECacheEntries`, `useHypervisor`, `jumpInFrontend`, `traceHasWdata`,
      `useConditionalZero`, `useZba`/`useZbb`/`useZbs`, `traceCustom` and
      `customCSRs`.
    - `class BoomTraceBundle` and `class BoomCustomCSRs` are not touched at all.
    - In `HasBoomCoreParameters`, every existing derived value keeps its current
      definition: `coreWidth`, the data-structure sizes, `aluWidth`/`memWidth`/
      `fpWidth`/`lsuWidth`, the LSU and branch-prediction blocks, and the
      "Implicitly calculated constants" block — in particular `robAddrSz`,
      `logicalRegCount`, `lregSz`, `ipregSz`, `fpregSz`, `maxPregSz`,
      `immPregSz`, `ldqAddrSz`, `stqAddrSz`, `lsuAddrSz`, `brTagSz`.
    - `maxPregSz` stays `ipregSz max fpregSz`. It must NOT be widened to include
      `vecPregSz`: vector PRNs live in their own fields sized by `vecPregSz`,
      and widening `maxPregSz` would grow `pdst`/`prs*` for every uop in every
      config, breaking gate (f).
    - Every existing `require` keeps its current form and none is weakened —
      including `require(memWidth >= 2)`, `require(memWidth >= lsuWidth)`,
      `require(numIntPhysRegs >= 32 + coreWidth)` and the LDQ/STQ ones.
    - With `enableVector = false` the elaborated core is bit-identical to the
      current file's output.
    - The file's copyright header, section-comment style (`//***...`) and the
      DOC include markers are preserved.

  Interface delta:
    NEW constructor fields on BoomCoreParams:
      enableVector          : Boolean               = false
      vector                : Option[VectorParams]  = None
      enableVectorArith     : Boolean               = false
      vecScalarSnoopEnable  : Boolean               = false
    NEW body overrides on BoomCoreParams:
      override def hasV, override def useVector, override def vLen,
      override def eLen
    NEW derived values on HasBoomCoreParameters:
      usingRVV, vectorParams, and the re-exported vector sizes and sub-flags
      listed in the logic section.
    Nothing is widened and nothing is removed.
<|end_edit_scope|>

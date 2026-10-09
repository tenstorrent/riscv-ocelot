# The implementation contract — Chisel LTL, checker files, and the bind

Normative for coders. Read this before writing a checker. Everything in the *API
surface* section is verified against the Chisel this repo pins (6.7.0, from
`build.sbt`) — prefer it over anything you remember about `chisel3.ltl`.

---

## 1. Where things go

| Artifact          | Path                                                       | Package                |
|-------------------|------------------------------------------------------------|------------------------|
| The bind layer    | `src/main/scala/v4/vec/formal/BoomSvaLayer.scala`            | `boom.v4.vec.formal`   |
| A checker         | `src/main/scala/v4/vec/formal/<Dut>Checks.scala`             | `boom.v4.vec.formal`   |
| The anchor        | inside the DUT's own file, wherever that is                 | the DUT's package      |

Paths are repo-relative from `generators/boom`. sbt only compiles under
`src/main/scala/`, which is why checkers live there and not beside the ledger.

---

## 2. API surface (verified, Chisel 6.7.0)

```scala
import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._   // implicit Bool -> Sequence, and |-> |=> ### ##* ##+
```

| Construct                        | Meaning                                     | Emits |
|----------------------------------|---------------------------------------------|-------|
| `AssertProperty(b)`              | boolean property                            | `assert property` |
| `AssertProperty(p, label = Some("x"))` | labelled                              | `x: assert property` |
| `AssertProperty(p, clock = Some(clock), disable = Some(reset.asDisable))` | explicit gating | `@(posedge clock) disable iff (reset)` |
| `AssumeProperty(p)` / `CoverProperty(p)` | same signatures                     | `assume property` / `cover property` |
| `a \|-> b`                        | overlapping implication                     | `\|->` |
| `a \|=> b`                        | non-overlapping (consequent next cycle)     | `\|=>` |
| `a ### b`                        | concatenation with one cycle of delay        | `a ##1 b` |
| `s.delay(n)`                     | delay by exactly `n`                        | `##n s` |
| `s.delayRange(a, b)`             | delay within `[a,b]`                        | `##[a:b] s` |
| `s.delayAtLeast(n)`              | delay by `n` or more                        | `##[n:$] s` |
| `p.eventually`                   | liveness                                    | `s_eventually` |
| `p.not`, `p.and(q)`, `p.or(q)`   | property algebra                            | — |
| `Sequence(a, Delay(1), b)`       | explicit atom form of `a ### b`              | `a ##1 b` |

Notes that will cost you a debug cycle each if ignored:

- **Never write reset gating by hand.** `AssertProperty` already gates on the
  implicit disable and inserts the `hasBeenReset` guard. Adding
  `when(!reset) { … }` around a property, or `&& !reset` inside one, produces a
  double gate and drops the first post-reset cycle.
- **`|=>` and `.delay(1)` are not the same as `###`.** `a |=> b` is an
  implication; `a ### b` is a two-cycle *sequence* that is itself the property.
  `AssertProperty(a ### b)` asserts "a now and b next cycle, always" — almost
  never what a requirement means. If the ledger says `|=>`, write `|=>`.
- **`circt` collides with `chisel3.util.circt`.** Inside a file that imports
  `chisel3.util._`, spell the emitter `_root_.circt.stage.ChiselStage`.
- **Scalar comparisons need `===`, not `==`.** `a == b` on `Data` is a Scala
  object comparison and silently yields a Scala `Boolean`, which will not compile
  into a property — and in a `|->` position produces a confusing type error, not
  a clear one.
- Chisel 7 renames `layer.Convention.Bind` to `LayerConfig.Extract()`. If
  `USE_CHISEL7` is ever set, `BoomSvaLayer.scala` needs that one-line change —
  and note that the Chisel 7 path in `common.mk` also passes
  `--disable-layers=Verification.Assume,Verification.Cover`, which would strip
  every `AssumeProperty` and `CoverProperty` this skill emits.

---

## 3. The checker file

One `object` per DUT, one `apply` taking the exact parameters the ledger's
`signals:` list declares, in that order.

```scala
// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.VDecode]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in VDecode.scala;
  * firtool emits these into their own .sv and a SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/formal/formal-decode.yaml.
  */
object VDecodeChecks {
  def apply(
    isRvv:      Bool,
    vsetRetire: Bool,
    vlWrite:    Bool,
    mirrorVtype: UInt,
    snapVtype:  UInt
  ): Unit = {

    //@formal-req-spec-decode.c4
    AssertProperty(
      vsetRetire |-> vlWrite,
      label = Some("vdecode_vset_writes_vl")
    )

    // Antecedent reachability for vdecode_vset_writes_vl (formal-decode.c9).
    CoverProperty(vsetRetire, label = Some("vdecode_vset_retire_seen"))

    //@formal-req-spec-decode.d7
    AssertProperty(
      isRvv |=> Sequence.BoolSequence(mirrorVtype === snapVtype),
      label = Some("vdecode_mirror_restore")
    )
  }
}
```

Rules:

- **Every parameter is used.** An unused parameter means the property that needed
  it is missing — report it rather than leaving a dangling argument.
- **`//@formal-req-<id>`, one per line, directly above the property.** Not in the
  file header, not on the same line. `validate` finds them by grep and pairs them
  with the ledger; a tag in the header claims the whole file checks that
  requirement.
- Covers that exist only for reachability get **no** requirement tag — they
  discharge nothing. Name the assert they back in a comment, as above.
- No `printf`, no `when` blocks, no registers, no state. A checker is a set of
  property declarations over signals handed to it. If a property needs history,
  express it as a sequence (`.delay(n)`, `###`), not as a register you declare
  here — a register inside a bound layer is legal Chisel but puts sampling logic
  in a place nobody will look when the property misfires.
- Never mutate an input. These are observations.

---

## 4. The anchor in the DUT

Exactly one block, at the **end of the module body** — where every internal
signal is in scope:

```scala
class VDecode(implicit p: Parameters) extends BoomModule {
  val io = IO(new VDecodeIo)

  // … existing RTL, untouched …

  //@formal-anchor VDecodeChecks
  layer.block(BoomSvaLayer) {
    VDecodeChecks(
      isRvv       = io.deq.uop.is_rvv,
      vsetRetire  = vset_retire,
      vlWrite     = io.vl_write.valid,
      mirrorVtype = vcfg_mirror.vtype,
      snapVtype   = vcfg_snapshot.vtype
    )
  }
}
```

with, at the top of the DUT file:

```scala
import chisel3.layer
import boom.v4.vec.formal.{BoomSvaLayer, VDecodeChecks}
```

This is the **entire** DUT edit. Named arguments are required — a positional list
of eight `Bool`s is exactly the kind of thing that silently transposes two
signals and produces a suite that passes while checking the wrong pair.

### What is forbidden in the DUT

- Adding ports, wires, registers, or `val`s to make a signal observable.
- Renaming, reformatting, reordering, or "tidying" anything.
- A wrapper module or `BoringUtils.tapAndRead`. Both work in isolation and both
  are banned here: a wrapper changes the instance hierarchy that the cosim
  bridge, the waveform scripts, and every hierarchical path in this repo depend
  on, and a tap adds a probe wire to the DUT for the same observability the
  in-place anchor already provides for free.
- More than one anchor per module.

If a needed signal is not in scope at the end of the module body, **stop and
report it.** That is an observability gap, and the decision about how to close it
belongs to whoever owns the RTL.

---

## 5. Compiling

Fast Scala-only check (no elaboration, no firtool) — this is what a coder runs
before reporting:

```bash
podman exec <container> bash -lc 'set -o pipefail; \
  source /opt/conda/etc/profile.d/conda.sh && source /root/my-chipyard/env.sh && \
  cd /root/my-chipyard && \
  make -C sims/vcs launch-sbt SBT_COMMAND=";project boom; compile" 2>&1 | tail -25'
```

`set -o pipefail` is not optional: `make | tail` otherwise reports tail's status
and a failed compile looks clean.

Full elaboration and the `bind` check belong to `validate` — see
`references/validate.md`. A coder reports a successful compile, never a
successful verification.

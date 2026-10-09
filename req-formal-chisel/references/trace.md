# Mode: trace — answer coverage questions

Invocation:

```
/req-formal-chisel trace <req-id>          # what checks this requirement?
/req-formal-chisel trace <formal-id>       # what does this assertion check?
/req-formal-chisel trace --unasserted <family>
/req-formal-chisel trace --unassertable <family>
/req-formal-chisel trace --partial <family>
/req-formal-chisel trace --checker <CheckerName>
```

This mode **writes nothing.** It reads the ledger and greps the tree.

## Grep is the truth about what is checked

There is no `status:` or `verified:` field in the ledger, by design. The ledger
says what was *planned*; the `//@formal-req-` tags say what was *written*; the
emitted `.sv` says what was *bound in*. Answer with whichever of the three the
question is actually about, and say which one you used.

```bash
# what checks spec-decode.d7 — the claim, from the code
grep -rn 'formal-req-spec-decode\.d7' src/main/scala/

# ...and the plan, from the ledger
grep -n -A6 'spec-decode\.d7' src/main/nlhdl/formal/formal-decode.yaml

# ...and whether it survived to Verilog
grep -rn 'vdecode_mirror_restore' <gen-collateral>/layers_*BoomSvaLayer*.sv
```

A requirement whose tag exists but whose label is absent from the emitted `.sv`
is **not checked**, whatever the ledger says. Report it that way.

## Answering `trace <req-id>`

Give, in this order:

1. The requirement's `statement:` from `spec-<family>.yaml` — so the reader can
   judge whether the property actually bears on it.
2. Its `coverage:` row: `assertions:`, `partial:`, or `unassertable:`.
3. For each assertion: `label`, `kind`, `property`, and the checker file:line of
   its tag.
4. Whether the label appears in the emitted `.sv`, if a build exists. If none
   does, say "not built" rather than implying it is live.

## Answering `--unasserted <family>`

Two distinct populations, and collapsing them is the most misleading thing this
mode can do:

- **Unassertable** — ledgered, with a reason. A decision that was made.
- **Missing** — a live requirement with no `coverage:` row at all. A gap.

Report them separately, `missing` first. Then, separately again, requirements
whose row exists but whose tags are absent from the code — planned and never
written.

## Cross-checking against implementation

`/spec-to-reqs trace <id>` answers "what *implements* this" via `//@req-` tags;
this mode answers "what *checks* it" via `//@formal-req-`. The interesting cells
are the off-diagonal ones:

| Implemented | Checked | Means |
|---|---|---|
| yes | yes | the good case |
| yes | no | untested code |
| no | yes | a property waiting for its RTL — expected mid-milestone, suspicious later |
| no | no | not started |

When asked "how is `<family>` doing", give the four counts. A single "coverage
percentage" over a family averages these together and is worse than no number.

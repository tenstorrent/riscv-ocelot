# `trace` — who implements what

```
/spec-to-reqs trace spec-lsu.f9          # what implements this requirement?
/spec-to-reqs trace --unimplemented lsu  # which of this family's reqs are untagged?
/spec-to-reqs trace src/main/nlhdl/vec/lsu/VecLsu.nlhdl.scala   # what does this file claim?
```

Implementation status is **derived by grep, never stored.** There is no `implemented_by:`,
`status:`, or `covered:` field in the schema. A denormalized copy of a fact that `grep`
answers exactly would be wrong within a week — and wrong with the authority of a checked-in
file. The tags in the RTL are the only record, and they cannot go stale relative to
themselves.

---

## Recipes

**What implements a requirement**

```bash
grep -rn '//@req-spec-lsu\.f9\b' src/main/nlhdl/
```

Report the file, line, and the enclosing construct. Zero hits means unimplemented — or, if
`validate` reports the ID as unknown, a typo or an improperly deleted requirement.

**What a file claims to implement**

```bash
grep -on '//@req-\(spec-[a-z0-9_]*\.[a-z]*[0-9]*\)' src/main/nlhdl/vec/lsu/VecLsu.nlhdl.scala
```

Then look each ID up in its family YAML and report the statements, so the reader sees
obligations rather than opaque IDs.

**Which requirements are unimplemented**

Run `scripts/validate-reqs.py` and read its `NOTE` block — it already computes live IDs
minus tagged IDs, for one family or all. Do not re-derive it by hand.

**Coverage figure for a family**

Live requirement count from the YAML, tagged count from the validator's summary line. State
both numbers; do not round to a percentage without them, because the denominator moves every
time the spec grows.

---

## What to report, and what not to claim

A tag means *someone asserted this code satisfies that requirement*. It does not mean the
code is correct, complete, or verified — no tool checked the claim, and this skill never
places tags precisely so the claim comes from the implementor rather than from the tool that
wrote the requirement.

So report tag counts as **tag counts**. "9 of 10 requirements are tagged" is a fact. "9 of
10 requirements are implemented" overstates it, and "the CII kill contract is done" is not
something a grep can establish.

Untagged is the more reliable signal in the other direction: an untagged requirement is
almost certainly unimplemented, because nobody writes the code and then removes the tag.

#!/usr/bin/env python3
"""
validate-formal.py — mechanical checks for src/main/nlhdl/formal/formal-*.yaml

Every check here is deterministic: set difference, uniqueness, regex, string-in-file.
Nothing in this script requires judgement, and nothing in it may be delegated to a
model — a coverage check that hallucinates success is worse than no check at all.

This script only ever READS. The ledgers are written as text by the skill so that
comments and key order survive; round-tripping them through PyYAML would turn every
run into an unreviewable whole-file diff.

Usage:
    validate-formal.py [family ...]
    validate-formal.py --plan-only [family ...]
    validate-formal.py [--repo-root DIR] [--formal-dir DIR] [--reqs-dir DIR]
                       [--rtl-dir DIR] [family ...]

--plan-only skips every check that needs the .scala files, for use between
`/req-formal-chisel plan` and `gen`.

The repo root is the `generators/boom` checkout: found by walking up from the current
directory looking for src/main/nlhdl/, then from this script's own directory as a
fallback. Pass --repo-root to override.

Exit codes:
    0  no errors (warnings and notes may be present)
    1  one or more errors
    2  bad invocation / unreadable inputs
"""

import argparse
import glob as globmod
import os
import re
import sys

try:
    import yaml
except ImportError:
    sys.stderr.write("error: PyYAML is required (python3 -c 'import yaml')\n")
    sys.exit(2)


FORMAL_ID_RE = re.compile(r"^formal-(?P<family>[a-z0-9_]+)\.(?P<group>[a-z]+)(?P<num>[0-9]+)$")
REQ_ID_RE = re.compile(r"^spec-(?P<family>[a-z0-9_]+)\.(?P<group>[a-z]+)(?P<num>[0-9]+)$")
FORMAL_TAG_RE = re.compile(r"//@formal-req-(spec-[a-z0-9_]+\.[a-z]+[0-9]+)")
IMPL_TAG_RE = re.compile(r"//@req-(spec-[a-z0-9_]+\.[a-z]+[0-9]+)")
ANCHOR_RE = re.compile(r"//@formal-anchor\s+([A-Za-z_][A-Za-z0-9_]*)")
LABEL_RE = re.compile(r"""label\s*=\s*Some\(\s*["']([A-Za-z0-9_]+)["']\s*\)""")
OBJECT_RE = re.compile(r"^\s*object\s+([A-Za-z_][A-Za-z0-9_]*)", re.M)
IDENT_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
LABEL_FORM_RE = re.compile(r"^[a-z][a-z0-9_]*$")

KINDS = ("assert", "assume", "cover")
SIGNAL_KINDS = ("bool", "uint", "sint", "bundle", "vec")
IMPLICATION_TOKENS = ("|->", "|=>")

# Identifiers that may appear in a `property:` without being a declared signal:
# Chisel/LTL API surface and literals.
PROPERTY_ALLOWED = set("""
    Sequence BoolSequence Delay AssertProperty AssumeProperty CoverProperty
    delay delayRange delayAtLeast eventually not and or concat clock disable
    asUInt asBool asSInt andR orR xorR
    U S B W true false True False Some None
    Cat Mux Fill VecInit PopCount Reverse Log2 OHToUInt UIntToOH PriorityEncoder
    io reset
""".split())

LIST_LIMIT = 25  # above this, summarize instead of enumerating


class Report(object):
    """Collects findings. Errors set the exit code; warnings and notes do not."""

    def __init__(self):
        self.errors = []
        self.warnings = []
        self.notes = []

    def error(self, where, msg):
        self.errors.append("%s: %s" % (where, msg))

    def warn(self, where, msg):
        self.warnings.append("%s: %s" % (where, msg))

    def note(self, msg):
        self.notes.append(msg)

    def print_and_exit(self):
        for line in self.errors:
            print("ERROR   %s" % line)
        for line in self.warnings:
            print("WARN    %s" % line)
        for line in self.notes:
            print("NOTE    %s" % line)
        print("")
        print("%d error(s), %d warning(s)" % (len(self.errors), len(self.warnings)))
        sys.exit(1 if self.errors else 0)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def read_text(path):
    with open(path, "r") as fh:
        return fh.read()


def load_yaml(path, rep, where):
    try:
        with open(path, "r") as fh:
            return yaml.safe_load(fh) or {}
    except Exception as exc:                                     # noqa: BLE001
        rep.error(where, "parse failed: %s" % exc)
        return None


def summarize(items):
    items = sorted(items)
    if len(items) <= LIST_LIMIT:
        return ", ".join(items)
    return "%s, ... (%d total)" % (", ".join(items[:LIST_LIMIT]), len(items))


def scala_files(rtl_dir):
    out = []
    for root, _dirs, files in os.walk(rtl_dir):
        for name in files:
            if name.endswith(".scala"):
                out.append(os.path.join(root, name))
    return sorted(out)


# ---------------------------------------------------------------------------
# Requirement corpus
# ---------------------------------------------------------------------------

def load_reqs(path, rep, where):
    """Return (live_ids, retired_ids, group_of_id) from a spec-<family>.yaml."""
    doc = load_yaml(path, rep, where)
    if doc is None:
        return set(), set(), {}

    live, retired, groups = set(), set(), {}
    for entry in doc.get("reqs") or []:
        if not isinstance(entry, dict):
            continue
        rid = entry.get("id")
        if not rid:
            continue
        live.add(rid)
        m = REQ_ID_RE.match(rid)
        if m:
            groups[rid] = m.group("group")
    for entry in doc.get("retired") or []:
        if isinstance(entry, dict) and entry.get("id"):
            retired.add(entry["id"])
    return live, retired, groups


# ---------------------------------------------------------------------------
# Ledger validation
# ---------------------------------------------------------------------------

def check_ledger(path, family, repo_root, reqs_dir, rtl_dir, known_families,
                 plan_only, rep):
    where = os.path.basename(path)
    doc = load_yaml(path, rep, where)
    if doc is None:
        return

    # --- top-level keys ----------------------------------------------------
    for key in ("family", "description", "reqs_source", "layer", "checkers",
                "assertions", "coverage"):
        if key not in doc:
            rep.error(where, "missing required top-level key '%s'" % key)

    if doc.get("family") != family:
        rep.error(where, "family '%s' does not match filename (expected '%s')"
                  % (doc.get("family"), family))
    if known_families is not None and family not in known_families:
        rep.error(where, "family '%s' is not declared in families.yaml" % family)
    if doc.get("layer") not in (None, "BoomSvaLayer"):
        rep.error(where, "layer '%s' is not BoomSvaLayer; exactly one layer exists "
                         "in this tree (see assets/BoomSvaLayer.scala)" % doc.get("layer"))

    # --- the requirement corpus this ledger claims to cover ---------------
    reqs_source = doc.get("reqs_source")
    live_reqs, retired_reqs, req_groups = set(), set(), {}
    reqs_readable = False
    if reqs_source:
        reqs_path = os.path.join(repo_root, reqs_source)
        if not os.path.exists(reqs_path):
            # Fall back to --reqs-dir before giving up, so a relocated corpus
            # degrades to a NOTE rather than a wall of false gaps.
            alt = os.path.join(reqs_dir, "spec-%s.yaml" % family)
            reqs_path = alt if os.path.exists(alt) else reqs_path
        if os.path.exists(reqs_path):
            live_reqs, retired_reqs, req_groups = load_reqs(
                reqs_path, rep, "%s -> %s" % (where, os.path.basename(reqs_path)))
            reqs_readable = True
        else:
            rep.note("%s: reqs_source '%s' not found — requirement-coverage checks "
                     "skipped for this family" % (where, reqs_source))

    # --- checkers ---------------------------------------------------------
    checkers = {}
    for entry in doc.get("checkers") or []:
        if not isinstance(entry, dict):
            rep.error(where, "checkers: entry is not a mapping")
            continue
        name = entry.get("name")
        if not name:
            rep.error(where, "checkers: entry with no name")
            continue
        if name in checkers:
            rep.error(where, "duplicate checker '%s'" % name)
        checkers[name] = entry
        for key in ("file", "dut", "dut_file", "signals"):
            if key not in entry:
                rep.error(where, "checker '%s' missing '%s'" % (name, key))
        dut = entry.get("dut")
        if dut and name != "%sChecks" % dut:
            rep.error(where, "checker '%s' must be named '%sChecks' (its dut)"
                      % (name, dut))
        f = entry.get("file") or ""
        if f and "/formal/" not in f:
            rep.error(where, "checker '%s' file '%s' is not under a formal/ directory"
                      % (name, f))
        sig_names = []
        for sig in entry.get("signals") or []:
            if not isinstance(sig, dict) or "name" not in sig or "expr" not in sig:
                rep.error(where, "checker '%s' has a signal without name/expr" % name)
                continue
            if sig["name"] in sig_names:
                rep.error(where, "checker '%s' declares signal '%s' twice"
                          % (name, sig["name"]))
            sig_names.append(sig["name"])
            k = sig.get("kind")
            if k is not None and k not in SIGNAL_KINDS:
                rep.error(where, "checker '%s' signal '%s' has kind '%s' (expected %s)"
                          % (name, sig["name"], k, "/".join(SIGNAL_KINDS)))
        entry["_sig_names"] = sig_names

    # --- assertions -------------------------------------------------------
    assertions = {}
    labels = {}
    for entry in doc.get("assertions") or []:
        if not isinstance(entry, dict):
            rep.error(where, "assertions: entry is not a mapping")
            continue
        aid = entry.get("id")
        if not aid:
            rep.error(where, "assertions: entry with no id")
            continue
        if aid in assertions:
            rep.error(where, "duplicate assertion id '%s'" % aid)
        assertions[aid] = entry

        m = FORMAL_ID_RE.match(aid)
        if not m:
            rep.error(where, "id '%s' is malformed (want formal-<family>.<group><n>)" % aid)
        elif m.group("family") != family:
            rep.error(where, "id '%s' does not belong to family '%s'" % (aid, family))

        for key in ("checker", "label", "kind", "statement", "property"):
            if key not in entry:
                rep.error(where, "%s missing '%s'" % (aid, key))
        if "reqs" not in entry:
            rep.error(where, "%s missing 'reqs' (use [] for a pure cover)" % aid)

        kind = entry.get("kind")
        if kind not in KINDS:
            rep.error(where, "%s kind '%s' is not one of %s" % (aid, kind, "/".join(KINDS)))

        checker = entry.get("checker")
        if checker and checker not in checkers:
            rep.error(where, "%s names checker '%s', which is not declared" % (aid, checker))

        label = entry.get("label")
        if label:
            if label in labels:
                rep.error(where, "label '%s' used by both %s and %s"
                          % (label, labels[label], aid))
            labels[label] = aid
            if not LABEL_FORM_RE.match(label):
                rep.error(where, "%s label '%s' must be lower snake_case" % (aid, label))

        reqs = entry.get("reqs") or []
        if not isinstance(reqs, list):
            rep.error(where, "%s reqs must be a list" % aid)
            reqs = []
        if kind == "assert" and not reqs:
            rep.error(where, "%s is an assert with no reqs — every assert discharges at "
                             "least one requirement, or it is an unlabelled addition" % aid)
        if kind == "assume" and reqs:
            rep.error(where, "%s is an assume with reqs %s — an assume CHECKS nothing, it "
                             "constrains. Ledger the requirement unassertable: and name "
                             "this assume in the reason (schema.md §5)"
                      % (aid, summarize(str(r) for r in reqs)))
        if kind == "cover" and reqs:
            rep.error(where, "%s is a cover with reqs %s — a cover measures reachability, "
                             "not correctness; cite it alongside an assert in the coverage "
                             "row instead" % (aid, summarize(str(r) for r in reqs)))
        if kind == "cover" and not entry.get("covers_antecedent_of"):
            if entry.get("purpose") != "functional":
                rep.warn(where, "%s is a cover backing no assert; set "
                                "`purpose: functional` to declare it deliberate scenario "
                                "coverage" % aid)
        for rid in reqs:
            if not REQ_ID_RE.match(str(rid)):
                rep.error(where, "%s cites malformed requirement id '%s'" % (aid, rid))
            elif reqs_readable:
                if rid in retired_reqs:
                    rep.error(where, "%s cites RETIRED requirement %s" % (aid, rid))
                elif rid not in live_reqs:
                    rep.error(where, "%s cites unknown requirement %s" % (aid, rid))

        # group letter agreement
        if m and reqs and reqs_readable:
            first = str(reqs[0])
            want = req_groups.get(first)
            if want and want != m.group("group"):
                rep.warn(where, "%s is in group '%s' but its first requirement %s is in "
                                "group '%s'" % (aid, m.group("group"), first, want))

        prop = (entry.get("property") or "").strip()
        if kind == "assume" and not entry.get("justification"):
            rep.error(where, "%s is an assume with no justification — an unjustified "
                             "assume can make every assertion in the file vacuous" % aid)
        if kind == "assert" and any(tok in prop for tok in IMPLICATION_TOKENS):
            if not entry.get("reachability"):
                rep.error(where, "%s is an implication with no reachability cover "
                                 "(vacuity)" % aid)
        if kind == "assume" and any(tok in prop for tok in IMPLICATION_TOKENS):
            if not entry.get("reachability"):
                rep.error(where, "%s is an implication assume with no reachability "
                                 "cover (vacuity)" % aid)

        # property references only declared signals
        if prop and checker in checkers:
            allowed = set(checkers[checker]["_sig_names"]) | PROPERTY_ALLOWED
            stripped = re.sub(r'"[^"]*"', "", prop)
            for ident in IDENT_RE.findall(stripped):
                if ident in allowed:
                    continue
                if re.match(r"^[0-9]", ident):
                    continue
                rep.error(where, "%s property references '%s', which is not a declared "
                                 "signal of checker '%s'" % (aid, ident, checker))

    # cross-links
    for aid, entry in sorted(assertions.items()):
        r = entry.get("reachability")
        if r:
            if r not in assertions:
                rep.error(where, "%s reachability '%s' is not an assertion in this "
                                 "ledger" % (aid, r))
            elif assertions[r].get("kind") != "cover":
                rep.error(where, "%s reachability '%s' is kind '%s', not cover"
                          % (aid, r, assertions[r].get("kind")))
        c = entry.get("covers_antecedent_of")
        if c and c not in assertions:
            rep.error(where, "%s covers_antecedent_of '%s' is not an assertion in this "
                             "ledger" % (aid, c))

    # --- retired ----------------------------------------------------------
    retired_ids, retired_labels = set(), set()
    for entry in doc.get("retired") or []:
        if not isinstance(entry, dict):
            continue
        if entry.get("id"):
            retired_ids.add(entry["id"])
            if entry["id"] in assertions:
                rep.error(where, "%s appears in both assertions: and retired:" % entry["id"])
        if entry.get("label"):
            retired_labels.add(entry["label"])
        if not entry.get("retired_because"):
            rep.error(where, "retired entry '%s' has no retired_because"
                      % entry.get("id", "?"))
    for label in sorted(retired_labels & set(labels)):
        rep.error(where, "label '%s' is retired but reused by %s" % (label, labels[label]))

    # --- coverage ledger --------------------------------------------------
    ledgered = {}
    for entry in doc.get("coverage") or []:
        if not isinstance(entry, dict):
            rep.error(where, "coverage: entry is not a mapping")
            continue
        rid = entry.get("req")
        if not rid:
            rep.error(where, "coverage: entry with no req")
            continue
        if rid in ledgered:
            rep.error(where, "coverage: %s ledgered twice" % rid)
        ledgered[rid] = entry

        has_a = bool(entry.get("assertions"))
        has_u = bool(entry.get("unassertable"))
        if has_a and has_u:
            rep.error(where, "coverage %s has both assertions: and unassertable:" % rid)
        if not has_a and not has_u:
            rep.error(where, "coverage %s has neither assertions: nor unassertable: — "
                             "silence is the failure this ledger exists to expose" % rid)
        if entry.get("partial") and not has_a:
            rep.error(where, "coverage %s has partial: without assertions:" % rid)

        cited = entry.get("assertions") or []
        for aid in cited:
            if aid not in assertions:
                rep.error(where, "coverage %s cites unknown assertion %s" % (rid, aid))
                continue
            kind = assertions[aid].get("kind")
            if kind != "assert":
                rep.error(where, "coverage %s cites %s, which is kind '%s' — a cover or "
                                 "assume never discharges a requirement. Covers attach to "
                                 "the assert they back via reachability:, not to the "
                                 "coverage row" % (rid, aid, kind))
                continue
            if rid not in (assertions[aid].get("reqs") or []):
                rep.error(where, "coverage %s cites %s, but that assertion does not "
                                 "list %s in its reqs" % (rid, aid, rid))

    if reqs_readable:
        missing = live_reqs - set(ledgered)
        if missing:
            rep.error(where, "%d live requirement(s) absent from coverage: %s"
                      % (len(missing), summarize(missing)))
        stale = set(ledgered) - live_reqs
        for rid in sorted(stale):
            if rid in retired_reqs:
                rep.error(where, "coverage ledgers %s, which is retired in %s"
                          % (rid, os.path.basename(reqs_source or "reqs")))
            else:
                rep.error(where, "coverage ledgers unknown requirement %s" % rid)

    # --- code-facing checks ----------------------------------------------
    if plan_only:
        rep.note("%s: --plan-only, skipped checker-file and anchor checks" % where)
    else:
        check_code(doc, where, checkers, assertions, labels, retired_labels,
                   repo_root, rep)

    # --- summary ----------------------------------------------------------
    n_assert = len([a for a in assertions.values() if a.get("kind") == "assert"])
    n_cover = len([a for a in assertions.values() if a.get("kind") == "cover"])
    n_assume = len([a for a in assertions.values() if a.get("kind") == "assume"])
    n_partial = len([c for c in ledgered.values() if c.get("partial")])
    n_unassertable = len([c for c in ledgered.values() if c.get("unassertable")])
    n_asserted = len(ledgered) - n_unassertable
    rep.note("%s: %d reqs ledgered -> %d asserted (%d partial), %d unassertable; "
             "%d assertions (%d assert / %d cover / %d assume) across %d checker(s)"
             % (where, len(ledgered), n_asserted, n_partial, n_unassertable,
                len(assertions), n_assert, n_cover, n_assume, len(checkers)))


def check_code(doc, where, checkers, assertions, labels, retired_labels,
               repo_root, rep):
    """Checks that need the .scala files: files exist, tags and labels agree, anchors."""
    for name, entry in sorted(checkers.items()):
        rel = entry.get("file")
        if not rel:
            continue
        path = os.path.join(repo_root, rel)
        if not os.path.exists(path):
            rep.error(where, "checker '%s' file %s does not exist" % (name, rel))
            continue
        text = read_text(path)

        objs = OBJECT_RE.findall(text)
        if name not in objs:
            rep.error(rel, "does not define 'object %s'" % name)

        # No implementation tags in a checker: //@req- means "implements".
        # `//@formal-req-` does not contain the literal `//@req-`, so this cannot
        # false-positive on the skill's own tags.
        stray = IMPL_TAG_RE.findall(text)
        if stray:
            rep.error(rel, "contains %d //@req- tag(s) — a checker CHECKS a requirement, "
                           "it does not implement one; use //@formal-req-" % len(stray))

        # Ledger <-> tag agreement, both directions.
        mine = [a for a in assertions.values() if a.get("checker") == name]
        planned_reqs = set()
        for a in mine:
            planned_reqs |= set(str(r) for r in (a.get("reqs") or []))
        found_reqs = set(FORMAL_TAG_RE.findall(text))
        for rid in sorted(planned_reqs - found_reqs):
            rep.error(rel, "ledger says this checker covers %s, but no "
                           "//@formal-req-%s tag is present" % (rid, rid))
        for rid in sorted(found_reqs - planned_reqs):
            rep.error(rel, "tags //@formal-req-%s, which the ledger does not assign to "
                           "this checker" % rid)

        # Every planned label must actually be emitted, and nothing retired.
        found_labels = set(LABEL_RE.findall(text))
        planned_labels = set(a.get("label") for a in mine if a.get("label"))
        for lbl in sorted(planned_labels - found_labels):
            rep.error(rel, "ledger plans label '%s', which is not present in the file"
                      % lbl)
        for lbl in sorted(found_labels - planned_labels):
            if lbl in labels:
                rep.error(rel, "emits label '%s', which the ledger assigns to a "
                               "different checker" % lbl)
            else:
                rep.error(rel, "emits label '%s', which is not in the ledger" % lbl)
        for lbl in sorted(found_labels & retired_labels):
            rep.error(rel, "emits retired label '%s'" % lbl)

        # Unused apply parameters mean a property is missing.
        for sig in entry.get("_sig_names", []):
            body = text
            if len(re.findall(r"\b%s\b" % re.escape(sig), body)) <= 1:
                rep.warn(rel, "apply parameter '%s' is declared but never used — the "
                              "property that needed it may be missing" % sig)

        # The anchor: exactly one, in the DUT's own file.
        dut_rel = entry.get("dut_file")
        if not dut_rel:
            continue
        dut_path = os.path.join(repo_root, dut_rel)
        if not os.path.exists(dut_path):
            rep.error(where, "checker '%s' dut_file %s does not exist" % (name, dut_rel))
            continue
        dut_text = read_text(dut_path)
        anchors = ANCHOR_RE.findall(dut_text)
        mine_anchors = [a for a in anchors if a == name]
        if not mine_anchors:
            rep.error(dut_rel, "no '//@formal-anchor %s' — the checker exists but is "
                               "never bound in, so it compiles clean and checks nothing"
                      % name)
        elif len(mine_anchors) > 1:
            rep.error(dut_rel, "%d anchors for '%s'; exactly one is allowed"
                      % (len(mine_anchors), name))
        if mine_anchors and "layer.block" not in dut_text:
            rep.error(dut_rel, "has a //@formal-anchor but no layer.block(...) — without "
                               "the layer the properties inline into the DUT instead of "
                               "being bound")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def find_repo_root(explicit):
    if explicit:
        return os.path.abspath(explicit)
    for start in (os.getcwd(), os.path.dirname(os.path.abspath(__file__))):
        cur = start
        while True:
            if os.path.isdir(os.path.join(cur, "src", "main", "nlhdl")):
                return cur
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("families", nargs="*", help="families to check (default: all)")
    ap.add_argument("--repo-root", default=None)
    ap.add_argument("--formal-dir", default=None)
    ap.add_argument("--reqs-dir", default=None)
    ap.add_argument("--rtl-dir", default=None)
    ap.add_argument("--plan-only", action="store_true",
                    help="skip checks that need the .scala files")
    args = ap.parse_args()

    repo_root = find_repo_root(args.repo_root)
    if not repo_root:
        sys.stderr.write("error: could not locate the generators/boom root; "
                         "pass --repo-root\n")
        sys.exit(2)

    formal_dir = args.formal_dir or os.path.join(repo_root, "src", "main", "nlhdl", "formal")
    reqs_dir = args.reqs_dir or os.path.join(repo_root, "src", "main", "nlhdl", "reqs")
    rtl_dir = args.rtl_dir or os.path.join(repo_root, "src", "main", "scala")

    if not os.path.isdir(formal_dir):
        sys.stderr.write("error: no formal ledger directory at %s\n"
                         "       run `/req-formal-chisel plan <family>` first, or pass "
                         "--formal-dir\n" % formal_dir)
        sys.exit(2)

    rep = Report()

    known_families = None
    registry = os.path.join(reqs_dir, "families.yaml")
    if os.path.exists(registry):
        doc = load_yaml(registry, rep, "families.yaml")
        if doc is not None:
            known_families = set((doc.get("families") or {}).keys())
    else:
        rep.note("no families.yaml at %s — family-name checks skipped" % registry)

    paths = sorted(globmod.glob(os.path.join(formal_dir, "formal-*.yaml")))
    if args.families:
        wanted = set(args.families)
        paths = [p for p in paths
                 if os.path.basename(p)[len("formal-"):-len(".yaml")] in wanted]
        found = set(os.path.basename(p)[len("formal-"):-len(".yaml")] for p in paths)
        for fam in sorted(wanted - found):
            rep.error("invocation", "no ledger formal-%s.yaml in %s" % (fam, formal_dir))

    if not paths:
        rep.note("no ledgers to check in %s" % formal_dir)
        rep.print_and_exit()

    for path in paths:
        family = os.path.basename(path)[len("formal-"):-len(".yaml")]
        check_ledger(path, family, repo_root, reqs_dir, rtl_dir, known_families,
                     args.plan_only, rep)

    # Stray tags: a //@formal-req- tag outside any ledgered checker file is a
    # property nobody is tracking.
    if not args.plan_only and os.path.isdir(rtl_dir):
        ledgered_files = set()
        for path in paths:
            doc = load_yaml(path, rep, os.path.basename(path))
            for entry in (doc or {}).get("checkers") or []:
                if isinstance(entry, dict) and entry.get("file"):
                    ledgered_files.add(os.path.normpath(
                        os.path.join(repo_root, entry["file"])))
        for scala in scala_files(rtl_dir):
            if os.path.normpath(scala) in ledgered_files:
                continue
            tags = set(FORMAL_TAG_RE.findall(read_text(scala)))
            if tags:
                rep.error(os.path.relpath(scala, repo_root),
                          "carries //@formal-req- tag(s) %s but is not a checker file in "
                          "any ledger" % summarize(tags))

    rep.print_and_exit()


if __name__ == "__main__":
    main()

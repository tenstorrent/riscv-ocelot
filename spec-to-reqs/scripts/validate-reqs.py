#!/usr/bin/env python3
"""
validate-reqs.py — mechanical checks for src/main/nlhdl/reqs/*.yaml

Every check here is deterministic: string-in-file, set difference, uniqueness, regex.
Nothing in this script requires judgement, and nothing in it may be delegated to a model —
a quote-match check that hallucinates success is worse than no check at all.

This script only ever READS. The requirement YAMLs are written as text by the skill, so
that comments and key order survive; round-tripping them through PyYAML would turn every
extraction into an unreviewable whole-file diff.

Usage:
    validate-reqs.py [family ...]
    validate-reqs.py [--repo-root DIR] [--reqs-dir DIR] [--rtl-dir DIR] [family ...]

The repo root is found by walking up from the current directory to a git checkout, then
from this script's own directory as a fallback. Pass --repo-root to override either.

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


ID_RE = re.compile(r"^spec-(?P<family>[a-z0-9_]+)\.(?P<group>[a-z]+)(?P<num>[0-9]+)$")
TAG_RE = re.compile(r"//@req-(spec-[a-z0-9_]+\.[a-z]+[0-9]+)")
ANCHOR_RE = re.compile(r"^\.\.\s+_([A-Za-z0-9_.\-]+):\s*$")
MD_HEADING_RE = re.compile(r"^(#{1,6})\s+(.*?)\s*#*\s*$")
RST_ADORNMENT = set("=-~^\"'`#*+:.<>_")
KINDS = {"function", "interface", "structure", "timing"}
MODALS = ("must", "shall")
LIST_LIMIT = 25  # above this, summarize instead of enumerating

# Files scanned for //@req- tags.
TAG_SUFFIXES = (".sv", ".svh", ".v", ".vh", ".scala", ".md", ".yaml", ".yml")


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
# Spec parsing
# ---------------------------------------------------------------------------

def normalize(text):
    """Collapse all runs of whitespace to single spaces, for quote matching.

    Quotes are copied verbatim from the spec but may be reflowed when embedded in YAML,
    so matching ignores line breaks and indentation and nothing else.
    """
    return " ".join(text.split())


def parse_sections(path, text):
    """Return [(heading, line_no, enclosing_anchor_or_None)] in document order.

    reStructuredText: a non-indented title line followed by a non-indented underline of a
    single repeated adornment character, at least as long as the title. Overlines are not
    used in these specs and are not handled. Simple-table borders are excluded because
    they contain interior spaces.

    Markdown: ATX headings only.
    """
    lines = text.splitlines()
    is_rst = path.endswith(".rst")
    sections = []
    anchors_seen = []  # (line_no, anchor) for every label in the file

    if not is_rst:
        for i, line in enumerate(lines):
            m = MD_HEADING_RE.match(line)
            if m:
                sections.append((m.group(2).strip(), i + 1, None))
        return sections, set()

    for i, line in enumerate(lines):
        m = ANCHOR_RE.match(line)
        if m:
            anchors_seen.append((i + 1, m.group(1)))

    for i in range(len(lines) - 1):
        title, under = lines[i], lines[i + 1]
        t = title.rstrip()
        u = under.rstrip()
        if not t or not u:
            continue
        if title[:1].isspace() or under[:1].isspace():
            continue
        if len(u) < len(t) or len(u) < 3:
            continue
        ch = u[0]
        if ch not in RST_ADORNMENT:
            continue
        if set(u) != set([ch]):
            continue
        if set(t) == set([ch]):  # an overline, or a horizontal rule
            continue
        if t.startswith(".. "):
            continue
        # Nearest preceding anchor is the enclosing one for citation purposes.
        enclosing = None
        for ln, name in anchors_seen:
            if ln <= i + 1:
                enclosing = name
            else:
                break
        sections.append((t.strip(), i + 1, enclosing))

    return sections, set(name for _, name in anchors_seen)


def load_spec(repo_root, rel_path, rep, where):
    abs_path = os.path.join(repo_root, rel_path)
    if not os.path.isfile(abs_path):
        rep.error(where, "spec source not found: %s" % rel_path)
        return None
    try:
        with open(abs_path, "r") as fh:
            text = fh.read()
    except (IOError, UnicodeDecodeError) as exc:
        rep.error(where, "cannot read %s: %s" % (rel_path, exc))
        return None
    sections, anchors = parse_sections(rel_path, text)
    return {
        "text": text,
        "norm": normalize(text),
        "sections": sections,
        "headings": set(h for h, _, _ in sections),
        "anchors": anchors,
    }


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def check_registry(repo_root, doc, registry, specs, rep):
    """Validate families.yaml's corpus declaration and section assignments.

    Returns assign_index: {rel_path: {heading: family_or_None}} for every spec file that
    has an `assignments:` entry. Files absent from the index fall back to the legacy rule
    (every family sourcing a file must ledger all of that file's sections).
    """
    where = "families.yaml"

    # ---- spec_corpus / excluded_sources -----------------------------------
    claimed = set()
    for family, entry in registry.items():
        for rel in (entry.get("sources") or []):
            claimed.add(rel)

    excluded = {}
    for entry in doc.get("excluded_sources") or []:
        if not isinstance(entry, dict):
            rep.error(where, "excluded_sources entry is not a mapping: %r" % (entry,))
            continue
        rel = entry.get("file")
        reason = (entry.get("reason") or "").strip()
        if not rel:
            rep.error(where, "excluded_sources entry needs file:")
            continue
        if not reason:
            rep.error(where, "excluded_sources %s needs a reason:" % rel)
        if rel in claimed:
            rep.error(where, "%s is both excluded and named as a family source" % rel)
        excluded[rel] = reason

    corpus_globs = doc.get("spec_corpus") or []
    if not corpus_globs:
        rep.note("families.yaml declares no spec_corpus: — cannot check for spec files "
                 "that no family claims")
    for pattern in corpus_globs:
        matches = sorted(globmod.glob(os.path.join(repo_root, pattern)))
        if not matches:
            rep.error(where, "spec_corpus pattern %r matches nothing" % pattern)
        for abs_path in matches:
            rel = os.path.relpath(abs_path, repo_root)
            if rel not in claimed and rel not in excluded:
                rep.error(where, "%s is in spec_corpus but no family claims it and it is "
                                 "not in excluded_sources" % rel)

    # ---- assignments -------------------------------------------------------
    assign_index = {}
    for entry in doc.get("assignments") or []:
        if not isinstance(entry, dict):
            rep.error(where, "assignments entry is not a mapping: %r" % (entry,))
            continue
        rel = entry.get("file")
        awhere = "families.yaml [assignments %s]" % rel
        if not rel:
            rep.error(where, "assignments entry needs file:")
            continue
        if rel in assign_index:
            rep.error(awhere, "duplicate assignments block for this file")
            continue
        if rel not in specs:
            specs[rel] = load_spec(repo_root, rel, rep, awhere)
        spec = specs.get(rel)

        table = {}
        owned = entry.get("owned") or {}
        if not isinstance(owned, dict):
            rep.error(awhere, "owned: must be a mapping of family -> [headings]")
            owned = {}
        for family, headings in owned.items():
            if family not in registry:
                rep.error(awhere, "owns sections for undeclared family '%s'" % family)
            elif rel not in (registry[family].get("sources") or []):
                rep.error(awhere, "family '%s' owns sections here but does not declare "
                                  "%s in its sources:" % (family, rel))
            if not isinstance(headings, list):
                rep.error(awhere, "owned['%s'] must be a list of headings" % family)
                continue
            for heading in headings:
                if heading in table:
                    rep.error(awhere, "section %r assigned twice (%s and %s)"
                              % (heading, table[heading] or "unowned", family))
                table[heading] = family

        unowned = entry.get("unowned") or {}
        if not isinstance(unowned, dict):
            rep.error(awhere, "unowned: must be a mapping of heading -> reason")
            unowned = {}
        for heading, reason in unowned.items():
            if heading in table:
                rep.error(awhere, "section %r is both owned and unowned" % heading)
            if not str(reason or "").strip():
                rep.error(awhere, "unowned section %r needs a reason" % heading)
            table[heading] = None

        if spec is not None:
            for heading in sorted(table):
                if heading not in spec["headings"]:
                    rep.error(awhere, "no such section in %s: %r" % (rel, heading))
            for heading, line_no, _ in spec["sections"]:
                if heading not in table:
                    rep.error("%s:%d" % (rel, line_no),
                              "section %r is assigned to no family and not listed as "
                              "unowned in families.yaml" % heading)

        assign_index[rel] = table

    # A family that declares a source but owns nothing in it is a taxonomy error.
    for family, entry in sorted(registry.items()):
        for rel in (entry.get("sources") or []):
            table = assign_index.get(rel)
            if table is None:
                continue
            if family not in set(table.values()):
                rep.error(where, "family '%s' declares source %s but owns no section in it"
                          % (family, rel))

    return assign_index


def check_family_file(repo_root, path, family, registry, specs, rep, assign_index):
    """Validate one spec-<family>.yaml. Returns (live_ids, retired_ids)."""
    where = os.path.relpath(path, repo_root)
    try:
        with open(path, "r") as fh:
            doc = yaml.safe_load(fh)
    except Exception as exc:  # noqa: BLE001 - surfacing any parse failure verbatim
        rep.error(where, "YAML parse failed: %s" % exc)
        return set(), set()

    if not isinstance(doc, dict):
        rep.error(where, "top level must be a mapping")
        return set(), set()

    for key in ("family", "description", "spec_sources", "groups", "reqs", "coverage"):
        if key not in doc:
            rep.error(where, "missing required top-level key '%s'" % key)

    if doc.get("family") != family:
        rep.error(where, "family: '%s' does not match filename family '%s'"
                  % (doc.get("family"), family))

    # spec_sources must agree with the registry, exactly.
    reg_entry = registry.get(family)
    declared = list(doc.get("spec_sources") or [])
    if reg_entry is None:
        rep.error(where, "family '%s' is not declared in families.yaml" % family)
    else:
        reg_sources = list(reg_entry.get("sources") or [])
        if declared != reg_sources:
            rep.error(where, "spec_sources %s disagrees with families.yaml sources %s"
                      % (declared, reg_sources))

    for rel in declared:
        if rel not in specs:
            specs[rel] = load_spec(repo_root, rel, rep, where)

    groups = doc.get("groups") or {}
    if not isinstance(groups, dict):
        rep.error(where, "groups: must be a mapping of letter -> description")
        groups = {}

    live_ids, retired_ids = set(), set()
    req_index = {}

    reqs = doc.get("reqs") or []
    if not isinstance(reqs, list):
        rep.error(where, "reqs: must be a list")
        reqs = []

    for entry in reqs:
        if not isinstance(entry, dict):
            rep.error(where, "reqs: entry is not a mapping: %r" % (entry,))
            continue
        rid = entry.get("id")
        rwhere = "%s [%s]" % (where, rid or "<no id>")

        m = ID_RE.match(rid or "")
        if not m:
            rep.error(rwhere, "malformed id (expected spec-<family>.<group><n>)")
            continue
        if m.group("family") != family:
            rep.error(rwhere, "id family '%s' does not match file family '%s'"
                      % (m.group("family"), family))
        if m.group("group") not in groups:
            rep.error(rwhere, "group '%s' is not declared in groups:" % m.group("group"))
        if rid in live_ids:
            rep.error(rwhere, "duplicate id")
        live_ids.add(rid)
        req_index[rid] = entry

        kind = entry.get("kind")
        if kind not in KINDS:
            rep.error(rwhere, "kind '%s' is not one of %s"
                      % (kind, ", ".join(sorted(KINDS))))

        statement = (entry.get("statement") or "").strip()
        if not statement:
            rep.error(rwhere, "statement is empty")
        else:
            low = statement.lower()
            if not any(mod in low for mod in MODALS):
                rep.error(rwhere, "statement contains neither 'must' nor 'shall'")
            words = len(statement.split())
            if words > 40:
                rep.warn(rwhere, "statement is %d words (limit 40)" % words)
            modal_count = sum(low.count(mod) for mod in MODALS)
            if " and " in low and modal_count > 1:
                rep.warn(rwhere, "statement looks like two conjoined obligations")

        src = entry.get("source")
        if not isinstance(src, dict):
            rep.error(rwhere, "source: missing or not a mapping")
            continue

        rel = src.get("file")
        if not rel:
            rep.error(rwhere, "source.file is missing")
            continue
        if rel not in declared:
            rep.error(rwhere, "source.file '%s' is not in spec_sources" % rel)
        spec = specs.get(rel)
        if spec is None:
            continue

        heading = src.get("heading")
        if not heading:
            rep.error(rwhere, "source.heading is missing")
        elif heading not in spec["headings"]:
            rep.error(rwhere, "source.heading %r not found in %s "
                              "(reworded heading, or wrong file)" % (heading, rel))

        anchor = src.get("anchor")
        if anchor and anchor not in spec["anchors"]:
            rep.error(rwhere, "source.anchor '%s' is not defined in %s" % (anchor, rel))

        quote = (src.get("quote") or "").strip()
        if not quote:
            rep.error(rwhere, "source.quote is missing")
        elif normalize(quote) not in spec["norm"]:
            rep.error(rwhere, "DRIFT: quote no longer occurs in %s" % rel)

    # ---- retired -----------------------------------------------------------
    for entry in doc.get("retired") or []:
        if not isinstance(entry, dict):
            rep.error(where, "retired: entry is not a mapping: %r" % (entry,))
            continue
        rid = entry.get("id")
        rwhere = "%s [retired %s]" % (where, rid or "<no id>")
        if not ID_RE.match(rid or ""):
            rep.error(rwhere, "malformed id")
            continue
        if rid in live_ids:
            rep.error(rwhere, "id is both live and retired")
        if rid in retired_ids:
            rep.error(rwhere, "duplicate retired id")
        retired_ids.add(rid)
        if not (entry.get("retired_because") or "").strip():
            rep.error(rwhere, "retired_because is missing")
        if not (entry.get("statement") or "").strip():
            rep.error(rwhere, "retired entry must keep its statement")

    # ---- coverage ledger ---------------------------------------------------
    ledger = doc.get("coverage") or []
    if not isinstance(ledger, list):
        rep.error(where, "coverage: must be a list")
        ledger = []

    ledger_keys = set()
    covered_reqs = {}
    for entry in ledger:
        if not isinstance(entry, dict):
            rep.error(where, "coverage: entry is not a mapping: %r" % (entry,))
            continue
        rel = entry.get("file")
        heading = entry.get("heading")
        cwhere = "%s [coverage %s / %s]" % (where, rel, heading)
        if not rel or not heading:
            rep.error(cwhere, "coverage entry needs both file: and heading:")
            continue
        if rel not in declared:
            rep.error(cwhere, "coverage file '%s' is not in spec_sources" % rel)
            continue
        key = (rel, heading)
        if key in ledger_keys:
            rep.error(cwhere, "duplicate coverage entry")
        ledger_keys.add(key)

        spec = specs.get(rel)
        if spec is not None and heading not in spec["headings"]:
            rep.error(cwhere, "no such section in %s (heading reworded or removed)" % rel)

        has_reqs = "reqs" in entry and entry["reqs"]
        has_skip = bool((entry.get("skipped") or "").strip())
        if has_reqs and has_skip:
            rep.error(cwhere, "entry has both reqs: and skipped:")
        if not has_reqs and not has_skip:
            rep.error(cwhere, "section is neither mined nor skipped")
        for rid in (entry.get("reqs") or []):
            if rid not in live_ids:
                rep.error(cwhere, "lists unknown or retired req '%s'" % rid)
            covered_reqs.setdefault(rid, []).append(heading)

    # The ledger must match this family's assigned sections exactly. Where families.yaml
    # has no assignments block for a source file, fall back to the legacy rule: the family
    # ledgers every section of that file.
    basename = os.path.basename(path)
    for rel in declared:
        spec = specs.get(rel)
        if spec is None:
            continue
        table = assign_index.get(rel)
        if table is None:
            required = set(h for h, _, _ in spec["sections"])
            permitted = required
            source_desc = "%s" % rel
        else:
            required = set(h for h, fam in table.items() if fam == family)
            permitted = required
            source_desc = "%s (assigned to '%s' in families.yaml)" % (rel, family)

        for heading in sorted(required):
            if (rel, heading) not in ledger_keys:
                line_no = next((ln for h, ln, _ in spec["sections"] if h == heading), 0)
                rep.error("%s:%d" % (rel, line_no),
                          "section %r is absent from %s coverage ledger [%s]"
                          % (heading, basename, source_desc))

        if table is not None:
            for entry_rel, heading in sorted(ledger_keys):
                if entry_rel != rel or heading in permitted:
                    continue
                owner = table.get(heading, "<unassigned>")
                rep.error("%s [coverage %s / %s]" % (where, rel, heading),
                          "family '%s' ledgers a section assigned to '%s' — remove it, or "
                          "change the assignment in families.yaml" % (family, owner))

    # Every live req must be claimed by exactly one ledger entry.
    for rid in sorted(live_ids):
        seen = covered_reqs.get(rid, [])
        if not seen:
            rep.error("%s [%s]" % (where, rid), "req is in no coverage entry")
        elif len(seen) > 1:
            rep.error("%s [%s]" % (where, rid),
                      "req appears in %d coverage entries" % len(seen))

    return live_ids, retired_ids


def scan_tags(rtl_dir):
    """Return {req_id: [(path, line_no), ...]} for every //@req- tag under rtl_dir."""
    tags = {}
    if not os.path.isdir(rtl_dir):
        return tags
    for dirpath, dirnames, filenames in os.walk(rtl_dir):
        dirnames[:] = [d for d in dirnames if not d.startswith(".")]
        for name in filenames:
            if not name.endswith(TAG_SUFFIXES):
                continue
            path = os.path.join(dirpath, name)
            try:
                with open(path, "r") as fh:
                    for line_no, line in enumerate(fh, 1):
                        for rid in TAG_RE.findall(line):
                            tags.setdefault(rid, []).append((path, line_no))
            except (IOError, UnicodeDecodeError):
                continue
    return tags


def find_repo_root(start):
    """Walk up from `start` looking for a git checkout. Returns None if there is none.

    `.git` is tested as a *path*, not a directory: it is a file holding a `gitdir:` pointer
    in a linked worktree or a submodule, and an isdir-only test silently walks past those.
    """
    here = os.path.abspath(start)
    while True:
        if os.path.exists(os.path.join(here, ".git")):
            return here
        parent = os.path.dirname(here)
        if parent == here:          # filesystem root; dirname stops changing
            return None
        here = parent


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("families", nargs="*", help="families to check (default: all)")
    ap.add_argument("--repo-root", default=None)
    ap.add_argument("--reqs-dir", default=None)
    ap.add_argument("--rtl-dir", default=None)
    ap.add_argument("--registry-only", action="store_true",
                    help="check families.yaml (corpus, exclusions, assignments) only")
    args = ap.parse_args()

    repo_root = args.repo_root
    if repo_root is None:
        # Search from the invocation directory first. The skill is normally installed
        # outside the repo it checks (~/.claude/skills/...), so walking up from __file__
        # finds no .git and lands on "/" — which then reports a missing families.yaml at
        # /src/main/nlhdl/reqs/ rather than a detection failure. Keep __file__ as a
        # fallback so an in-repo copy of the script still works from any cwd.
        repo_root = (find_repo_root(os.getcwd())
                     or find_repo_root(os.path.dirname(os.path.abspath(__file__)))
                     or os.getcwd())
    repo_root = os.path.abspath(repo_root)

    reqs_dir = args.reqs_dir or os.path.join(repo_root, "src", "main", "nlhdl", "reqs")
    rtl_dir = args.rtl_dir or os.path.join(repo_root, "src", "main", "nlhdl")

    rep = Report()

    registry_path = os.path.join(reqs_dir, "families.yaml")
    if not os.path.isfile(registry_path):
        sys.stderr.write("error: no families.yaml at %s\n" % registry_path)
        if args.repo_root is None and args.reqs_dir is None:
            sys.stderr.write("  repo root was auto-detected as %s\n" % repo_root)
            sys.stderr.write("  if that is wrong, pass --repo-root DIR (or --reqs-dir DIR)\n")
        sys.exit(2)
    try:
        with open(registry_path, "r") as fh:
            registry_doc = yaml.safe_load(fh) or {}
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write("error: families.yaml parse failed: %s\n" % exc)
        sys.exit(2)

    registry = registry_doc.get("families") or {}
    if not isinstance(registry, dict):
        sys.stderr.write("error: families.yaml 'families:' must be a mapping\n")
        sys.exit(2)

    wanted = set(args.families) if args.families else None
    specs = {}
    all_live, all_retired = set(), set()
    checked = []
    pending = []

    assign_index = check_registry(repo_root, registry_doc, registry, specs, rep)

    if args.registry_only:
        rep.note("registry-only: family files not checked")
        rep.print_and_exit()

    for family in sorted(registry):
        if wanted is not None and family not in wanted:
            continue
        path = os.path.join(reqs_dir, "spec-%s.yaml" % family)
        if not os.path.isfile(path):
            # Not an error: a declared family that has not been extracted yet is the
            # normal state mid-migration, and erroring here would make `validate`
            # useless exactly when the taxonomy is newest.
            pending.append(family)
            continue
        live, retired = check_family_file(repo_root, path, family, registry, specs, rep,
                                          assign_index)
        all_live |= live
        all_retired |= retired
        checked.append(family)

    if wanted:
        for family in sorted(wanted - set(registry)):
            rep.error("argv", "family '%s' is not declared in families.yaml" % family)

    # Stray spec-*.yaml files with no registry entry.
    if os.path.isdir(reqs_dir):
        for name in sorted(os.listdir(reqs_dir)):
            m = re.match(r"^spec-([a-z0-9_]+)\.yaml$", name)
            if m and m.group(1) not in registry:
                rep.error(os.path.join("src/main/nlhdl/reqs", name),
                          "family '%s' is not declared in families.yaml" % m.group(1))

    # ---- RTL tags ----------------------------------------------------------
    tags = scan_tags(rtl_dir)
    for rid in sorted(tags):
        if rid in all_retired:
            for path, line_no in tags[rid]:
                rep.error("%s:%d" % (os.path.relpath(path, repo_root), line_no),
                          "tag references RETIRED req %s" % rid)
        elif rid not in all_live:
            fam = ID_RE.match(rid)
            if wanted is not None and fam and fam.group("family") not in wanted:
                continue  # belongs to a family we were not asked to check
            for path, line_no in tags[rid]:
                rep.error("%s:%d" % (os.path.relpath(path, repo_root), line_no),
                          "tag references UNKNOWN req %s" % rid)

    untagged = sorted(rid for rid in all_live if rid not in tags)
    if untagged:
        # Summarize by family rather than listing every ID: early in a migration this is
        # every requirement in the project, and a thousand-line note buries the errors.
        by_family = {}
        for rid in untagged:
            m = ID_RE.match(rid)
            key = m.group("family") if m else "?"
            by_family[key] = by_family.get(key, 0) + 1
        rep.note("%d live req(s) have no //@req- tag in %s: %s"
                 % (len(untagged), os.path.relpath(rtl_dir, repo_root),
                    ", ".join("%s=%d" % (f, by_family[f]) for f in sorted(by_family))))
        if len(untagged) <= LIST_LIMIT:
            for rid in untagged:
                rep.note("    %s" % rid)
        else:
            rep.note("    (list suppressed above %d; use `trace --unimplemented <family>`)"
                     % LIST_LIMIT)

    if pending:
        rep.note("%d family(ies) declared but not yet extracted: %s"
                 % (len(pending), ", ".join(pending)))
    if checked:
        rep.note("checked families: %s" % ", ".join(checked))
        rep.note("%d live req(s), %d retired, %d tagged in RTL"
                 % (len(all_live), len(all_retired), len(tags)))

    rep.print_and_exit()


if __name__ == "__main__":
    main()

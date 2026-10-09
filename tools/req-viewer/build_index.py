#!/usr/bin/env python3
"""
build_index.py — build the trace index that drives the requirement viewer.

Three artifacts are joined here:

    docs_caracal/_build/html/src/*.html   the rendered spec  — hover targets
    src/main/nlhdl/reqs/spec-*.yaml       the requirements   — each carries a verbatim
                                                               RST quote + anchor + heading
    src/main/{nlhdl,scala,sv}/**          the code           — carries //@req-<id> tags;
                                                               NL_HDL and emitted RTL live in
                                                               separate trees, so all are
                                                               walked (see DEFAULT_TAG_ROOTS)

The hard part is the first join. A requirement quotes *reStructuredText source*
(``|caracal|``, ``:doc:`loadstore```, ``**bold**``), and the viewer must highlight the
*rendered HTML block* that text became. So each quote is stripped of RST inline markup,
folded to plain tokens, and matched against the text of every block inside the section the
requirement cites. Exact containment is the common case; a token-LCS coverage score handles
the rest (rendered ``:doc:`` titles, smart quotes, line-wrapped literals).

Outputs, under --out (default docs_caracal/_build/reqviewer/):

    index.json          docs, sections, blocks, reqs, block<->req mapping, match report
    docs/<name>.html    the article body of each rendered page, with data-block="..."
                        stamped on every hoverable block

This script only ever reads the repo. Nothing under docs_caracal/ or src/ is modified.

Usage:
    build_index.py [--repo-root DIR] [--out DIR] [--quiet] [--report]
"""

from __future__ import print_function

import argparse
import io
import json
import os
import re
import sys

try:
    from html.parser import HTMLParser
except ImportError:  # pragma: no cover - py2
    from HTMLParser import HTMLParser

try:
    import yaml
except ImportError:
    sys.stderr.write("error: PyYAML is required (python3 -c 'import yaml')\n")
    sys.exit(2)


# ---------------------------------------------------------------------------
# Text normalization — the common ground between RST source and rendered HTML
# ---------------------------------------------------------------------------

# Sphinx smartquotes rewrites the source, so quotes must be folded on both sides.
FOLD = {
    u"’": u"'", u"‘": u"'", u"“": u'"', u"”": u'"',
    u"—": u"-", u"–": u"-", u"‑": u"-", u"−": u"-",
    u" ": u" ", u" ": u" ", u" ": u" ", u"​": u"",
    u"…": u"...",
}

TOKEN_RE = re.compile(r"[a-z0-9]+")
SECTION_NUM_RE = re.compile(r"^\s*(\d+(\.\d+)*\.)\s+")


def fold(text):
    for src, dst in FOLD.items():
        if src in text:
            text = text.replace(src, dst)
    return text


def normalize(text):
    """Collapse whitespace and fold typographic variants. Case is preserved."""
    return u" ".join(fold(text).split())


def tokens(text):
    return TOKEN_RE.findall(fold(text).lower())


def strip_rst(quote):
    """Reduce an RST quote to the plain text a reader sees on the rendered page.

    Handled: inline literals, strong/emphasis, roles (:doc:, :ref:, :math:, ...),
    substitutions, external link targets, footnote references, comment markers and
    directive option lines. What survives is text, in reading order.

    Roles are the one lossy case: ``:doc:`loadstore``` renders as the *title* of that
    document ("Loadstore"), not as the target. The role's target is kept as a best guess and
    the fuzzy scorer absorbs the difference.
    """
    q = fold(quote)

    # ``literal`` -> literal   (before single-backtick handling)
    q = re.sub(r"``(.+?)``", r"\1", q, flags=re.S)
    # :role:`text <target>` -> text ;  :role:`target` -> target
    q = re.sub(r":[a-zA-Z:+-]+:`([^`<]*?)\s*<[^`>]*>`", r"\1", q, flags=re.S)
    q = re.sub(r":[a-zA-Z:+-]+:`([^`]*?)`", r"\1", q, flags=re.S)
    # `text <url>`_ -> text ;  `text`_ -> text ;  `text` -> text
    q = re.sub(r"`([^`<]*?)\s*<[^`>]*>`__?", r"\1", q, flags=re.S)
    q = re.sub(r"`([^`]*?)`__?", r"\1", q, flags=re.S)
    q = re.sub(r"`([^`]*?)`", r"\1", q, flags=re.S)
    # **strong** / *emphasis*
    q = re.sub(r"\*\*(.+?)\*\*", r"\1", q, flags=re.S)
    q = re.sub(r"(?<![\w*])\*(?!\s)(.+?)(?<!\s)\*(?![\w*])", r"\1", q, flags=re.S)
    # |substitution| -> substitution   (|caracal| renders as "Caracal")
    q = re.sub(r"\|([A-Za-z0-9_.+-]+)\|", r"\1", q)
    # footnote / citation references
    q = re.sub(r"\[[#*\w-]*\]_", "", q)
    return normalize(q)


def lcs_len(a, b):
    """Length of the longest common subsequence of two token lists."""
    if not a or not b:
        return 0
    prev = [0] * (len(b) + 1)
    for ta in a:
        cur = [0]
        for j, tb in enumerate(b):
            if ta == tb:
                cur.append(prev[j] + 1)
            else:
                left = cur[j]
                up = prev[j + 1]
                cur.append(left if left >= up else up)
        prev = cur
    return prev[-1]


# ---------------------------------------------------------------------------
# A minimal HTML tree — enough to find, tag and re-serialize the article body
# ---------------------------------------------------------------------------

VOID = set("area base br col embed hr img input link meta param source track wbr".split())

# Elements that can carry requirements. A block is the *innermost* such element: docutils
# wraps list-item and cell content in <p>, so the <p> wins over its <li>.
BLOCK_TAGS = set("p pre h1 h2 h3 h4 h5 h6 dt dd li td th figcaption caption".split())

# Never a hover target: permalink icons, admonition titles, table column groups.
SKIP_CLASSES = ("headerlink",)


class Node(object):
    __slots__ = ("tag", "attrs", "children", "parent", "text", "block_id", "section")

    def __init__(self, tag, attrs=None, text=None, parent=None):
        self.tag = tag                  # None for a text node
        self.attrs = attrs or []
        self.children = []
        self.parent = parent
        self.text = text
        self.block_id = None
        self.section = None

    def attr(self, name):
        for k, v in self.attrs:
            if k == name:
                return v or ""
        return ""

    def classes(self):
        return self.attr("class").split()

    def iter_elements(self):
        for child in self.children:
            if child.tag is not None:
                yield child
                for sub in child.iter_elements():
                    yield sub

    def text_content(self):
        out = []
        stack = [self]
        # depth-first, document order
        def walk(node):
            if node.tag is None:
                out.append(node.text)
                return
            if node.tag in ("script", "style"):
                return
            for cls in SKIP_CLASSES:
                if cls in node.classes():
                    return
            for child in node.children:
                walk(child)
        walk(self)
        del stack
        return "".join(out)


class TreeBuilder(HTMLParser):
    """Builds a Node tree. Tolerates the unclosed tags docutils output contains."""

    def __init__(self):
        try:
            HTMLParser.__init__(self, convert_charrefs=True)
        except TypeError:  # pragma: no cover - py2
            HTMLParser.__init__(self)
        self.root = Node("#root")
        self.stack = [self.root]

    # -- construction --------------------------------------------------------
    def handle_starttag(self, tag, attrs):
        node = Node(tag, attrs, parent=self.stack[-1])
        self.stack[-1].children.append(node)
        if tag not in VOID:
            self.stack.append(node)

    def handle_startendtag(self, tag, attrs):
        node = Node(tag, attrs, parent=self.stack[-1])
        self.stack[-1].children.append(node)

    def handle_endtag(self, tag):
        for i in range(len(self.stack) - 1, 0, -1):
            if self.stack[i].tag == tag:
                del self.stack[i:]
                return
        # stray close tag: ignore

    def handle_data(self, data):
        self.stack[-1].children.append(Node(None, text=data, parent=self.stack[-1]))

    def handle_entityref(self, name):  # pragma: no cover - convert_charrefs handles these
        self.stack[-1].children.append(Node(None, text="&%s;" % name))

    def handle_charref(self, name):  # pragma: no cover
        self.stack[-1].children.append(Node(None, text="&#%s;" % name))

    def handle_comment(self, data):
        pass


ATTR_ESCAPE = ((u"&", u"&amp;"), (u'"', u"&quot;"), (u"<", u"&lt;"), (u">", u"&gt;"))
TEXT_ESCAPE = ((u"&", u"&amp;"), (u"<", u"&lt;"), (u">", u"&gt;"))


def esc(text, table):
    for src, dst in table:
        text = text.replace(src, dst)
    return text


def serialize(node, out):
    if node.tag is None:
        out.append(esc(node.text, TEXT_ESCAPE))
        return
    if node.tag == "#root":
        for child in node.children:
            serialize(child, out)
        return
    bits = [node.tag]
    for k, v in node.attrs:
        if v is None:
            bits.append(k)
        else:
            bits.append(u'%s="%s"' % (k, esc(v, ATTR_ESCAPE)))
    if node.block_id is not None:
        bits.append(u'data-block="%s"' % esc(node.block_id, ATTR_ESCAPE))
    out.append(u"<%s>" % u" ".join(bits))
    if node.tag in VOID:
        return
    for child in node.children:
        serialize(child, out)
    out.append(u"</%s>" % node.tag)


def find_article_body(root):
    for el in root.iter_elements():
        if el.attr("itemprop") == "articleBody":
            return el
    for el in root.iter_elements():
        if el.attr("role") == "main":
            return el
    return None


# ---------------------------------------------------------------------------
# Rendered-page model
# ---------------------------------------------------------------------------

def rewrite_urls(body, doc_names):
    """Point asset URLs at the viewer's /sphinx/ mount; mark internal doc links.

    Cross-document links keep their href (so they still mean something if the page is
    opened directly) and gain data-doc/data-frag so the app can route the click to the
    correct pane instead of navigating the iframe.
    """
    for el in body.iter_elements():
        for i, (key, val) in enumerate(el.attrs):
            if key not in ("src", "href") or not val:
                continue
            if val.startswith("../_images/") or val.startswith("../_static/"):
                el.attrs[i] = (key, "/sphinx/" + val[3:])
            elif val.startswith("../_sources/"):
                el.attrs[i] = (key, "/sphinx/" + val[3:])
        href = el.attr("href")
        if not href or el.tag != "a":
            continue
        target, _, frag = href.partition("#")
        if target.endswith(".html"):
            name = os.path.basename(target)[:-5]
            if name in doc_names:
                el.attrs.append(("data-doc", name))
                if frag:
                    el.attrs.append(("data-frag", frag))
        elif not target and frag:
            el.attrs.append(("data-frag", frag))


def collect_sections(body):
    """Return [section], each {id, anchors, heading, number, level, parent, path}."""
    sections = []
    by_node = {}

    def visit(node, parents):
        for child in node.children:
            if child.tag is None:
                continue
            if child.tag == "section":
                sec = build_section(child, parents)
                sections.append(sec)
                by_node[id(child)] = sec
                visit(child, parents + [sec])
            else:
                visit(child, parents)

    def build_section(node, parents):
        anchors = set()
        if node.attr("id"):
            anchors.add(node.attr("id"))
        heading, number, level = "", "", 0
        for child in node.children:
            if child.tag is None:
                continue
            # `.. _label:` before a heading becomes an empty <span id="label">
            if child.tag == "span" and child.attr("id") and not child.text_content().strip():
                anchors.add(child.attr("id"))
            if re.match(r"^h[1-6]$", child.tag or ""):
                level = int(child.tag[1])
                full = normalize(child.text_content())
                m = SECTION_NUM_RE.match(full)
                if m:
                    number = m.group(1)
                    heading = full[m.end():]
                else:
                    heading = full
        return {
            "node": node,
            "id": node.attr("id"),
            "anchors": sorted(anchors),
            "heading": heading,
            "number": number,
            "level": level,
            "path": [p["heading"] for p in parents] + [heading],
            "parent": parents[-1]["id"] if parents else None,
        }

    visit(body, [])
    return sections, by_node


def collect_blocks(body, doc_name, sections, by_node):
    """Stamp data-block on every innermost block element; return block records."""
    blocks = []

    def enclosing_section(node):
        cur = node.parent
        while cur is not None:
            if cur.tag == "section" and id(cur) in by_node:
                return by_node[id(cur)]
            cur = cur.parent
        return None

    def has_block_descendant(node):
        for el in node.iter_elements():
            if el.tag in BLOCK_TAGS:
                return True
        return False

    rows = {}

    def enclosing_row(node):
        cur = node.parent
        while cur is not None:
            if cur.tag == "tr":
                return rows.setdefault(id(cur), "%s/row%d" % (doc_name, len(rows)))
            if cur.tag == "table":
                return None
            cur = cur.parent
        return None

    def visit(node):
        for child in node.children:
            if child.tag is None:
                continue
            if child.tag in BLOCK_TAGS and not has_block_descendant(child):
                text = normalize(child.text_content())
                if text:
                    sec = enclosing_section(child)
                    bid = "%s:%d" % (doc_name, len(blocks))
                    child.block_id = bid
                    blocks.append({
                        "id": bid,
                        "doc": doc_name,
                        "tag": child.tag,
                        "anchor": child.attr("id") or None,
                        "section": sec["id"] if sec else None,
                        "heading": sec["heading"] if sec else "",
                        "row": enclosing_row(child),
                        "text": text,
                        "low": text.lower(),
                        "tokens": tokens(text),
                        "reqs": [],
                    })
                continue  # leaf block: do not descend
            visit(child)

    visit(body)
    return blocks


TOCTREE_ENTRY_RE = re.compile(r"^\s+(?:self|src/([A-Za-z0-9_.-]+))\s*$")


def toctree_order(repo_root, doc_names):
    """Doc names in the order index.rst lists them, unknown pages appended alphabetically.

    Alphabetical order puts case_study first, which is nobody's idea of where to start
    reading. The toctree is the spec's own answer to that question.
    """
    order, seen = [], set()
    path = os.path.join(repo_root, "docs_caracal", "index.rst")
    try:
        with io.open(path, "r", encoding="utf-8") as fh:
            lines = fh.read().splitlines()
    except IOError:
        return sorted(doc_names)
    in_tree = False
    for line in lines:
        if line.startswith(".. toctree::"):
            in_tree = True
            continue
        if in_tree:
            if line.strip() and not line.startswith(" "):
                in_tree = False
                continue
            m = TOCTREE_ENTRY_RE.match(line)
            if m and m.group(1) and m.group(1) in doc_names and m.group(1) not in seen:
                seen.add(m.group(1))
                order.append(m.group(1))
    order.extend(n for n in sorted(doc_names) if n not in seen)
    return order


def load_page(path, doc_name, doc_names):
    with io.open(path, "r", encoding="utf-8", errors="replace") as fh:
        raw = fh.read()
    builder = TreeBuilder()
    builder.feed(raw)
    body = find_article_body(builder.root)
    if body is None:
        return None
    sections, by_node = collect_sections(body)
    title = sections[0]["heading"] if sections else ""
    if not title:
        for el in builder.root.iter_elements():
            if el.tag == "title":
                title = re.split(r"\s+[-\u2014]\s+Caracal", normalize(el.text_content()))[0]
                break
    blocks = collect_blocks(body, doc_name, sections, by_node)
    rewrite_urls(body, doc_names)
    out = []
    for child in body.children:
        serialize(child, out)
    return {
        "name": doc_name,
        "title": title or doc_name,
        "sections": sections,
        "blocks": blocks,
        "html": u"".join(out),
    }


# ---------------------------------------------------------------------------
# Quote -> block matching
# ---------------------------------------------------------------------------

EXACT = "exact"          # the stripped quote occurs verbatim in the block
FUZZY = "fuzzy"          # token coverage >= CONFIDENT, possibly spread over sibling blocks
APPROX = "approx"        # placed, but the caller should treat it as a hint
SECTION_ONLY = "section"  # no block resembled the quote; pinned to the section heading
UNMATCHED = "none"        # the cited page or section does not exist

CONFIDENT = 0.75   # accept a scope's best score and stop widening
PLACEABLE = 0.5    # below this, a block match is not worth showing as the quote's home


def find_section(page, anchor, heading):
    """Locate the cited section: anchor first, heading as fallback."""
    if anchor:
        for sec in page["sections"]:
            if anchor in sec["anchors"]:
                return sec, "anchor"
        # `.. _label:` placed *after* a heading lands on a block, not the section
        for blk in page["blocks"]:
            if blk["anchor"] == anchor:
                for sec in page["sections"]:
                    if sec["id"] == blk["section"]:
                        return sec, "anchor-block"
    if heading:
        want = normalize(heading)
        for sec in page["sections"]:
            if sec["heading"] == want:
                return sec, "heading"
        low = want.lower()
        for sec in page["sections"]:
            if sec["heading"].lower() == low:
                return sec, "heading"
    return None, None


def section_descendants(page, sec):
    """Section ids of sec and everything nested inside it."""
    by_parent = {}
    for other in page["sections"]:
        by_parent.setdefault(other["parent"], []).append(other)
    out, stack = set(), [sec]
    while stack:
        cur = stack.pop()
        out.add(cur["id"])
        stack.extend(by_parent.get(cur["id"], []))
    return out


def score_block(qtokens, blk):
    """Coverage of the quote by one block, in [0, 1]."""
    if not qtokens:
        return 0.0
    return lcs_len(qtokens, blk["tokens"]) / float(len(qtokens))


EXTEND_BEFORE = 2    # units of context a quote may reach backwards (a lead-in sentence)
EXTEND_AFTER = 8     # ... and forwards (the bullet list or table rows it introduces)


def build_units(page):
    """Group blocks into matchable units.

    A list-table row is one unit: docutils renders ``* - VL - 1 - 64`` as one ``<td>`` per
    column, so no single cell resembles the quote while their concatenation does. Every other
    block is a unit on its own. Units carry the blocks the requirement gets attached to.
    """
    units, by_row = [], {}
    for blk in page["blocks"]:
        if blk["row"]:
            unit = by_row.get(blk["row"])
            if unit is None:
                unit = {"idx": len(units), "section": blk["section"], "blocks": [],
                        "tokens": [], "row": blk["row"]}
                by_row[blk["row"]] = unit
                units.append(unit)
            unit["blocks"].append(blk)
            unit["tokens"] = unit["tokens"] + blk["tokens"]
        else:
            units.append({"idx": len(units), "section": blk["section"], "blocks": [blk],
                          "tokens": blk["tokens"], "row": None})
    return units


def best_in_scope(qtokens, scope):
    """Best (blocks, coverage) for the quote among a candidate set of units.

    One unit usually carries the whole quote. When it does not — a lead-in sentence plus the
    bullet list it introduces, or a quote spanning two table rows — the run is extended, but
    only into *neighbouring* units of the same section. Without that restriction a quote made
    of common words collects lookalike paragraphs from all over the page.
    """
    if not scope or not qtokens:
        return [], 0.0
    scored = sorted(((score_block(qtokens, u), u) for u in scope), key=lambda p: -p[0])
    best, seed = scored[0]
    if best <= 0.0:
        return [], 0.0
    if best >= 0.9:
        return list(seed["blocks"]), best

    # Grow a contiguous run outwards from the seed. Contiguity is what separates "this
    # paragraph plus the list it introduces" from "four lookalike paragraphs".
    window = {u["idx"]: u for u in scope
              if u["section"] == seed["section"]
              and seed["idx"] - EXTEND_BEFORE <= u["idx"] <= seed["idx"] + EXTEND_AFTER}
    floor = max(1, int(0.12 * len(qtokens)))
    keep = [seed]
    lo = hi = seed["idx"]
    remaining = drop_matched(list(qtokens), seed["tokens"])
    growing = True
    while remaining and growing:
        growing = False
        for idx in (hi + 1, lo - 1):
            unit = window.get(idx)
            if unit is None or lcs_len(remaining, unit["tokens"]) < floor:
                continue
            keep.append(unit)
            remaining = drop_matched(remaining, unit["tokens"])
            lo, hi = min(lo, idx), max(hi, idx)
            growing = True

    covered = 1.0 - len(remaining) / float(len(qtokens))
    if len(keep) == 1 or covered <= best + 0.02:
        return list(seed["blocks"]), best
    keep.sort(key=lambda u: u["idx"])
    out = []
    for unit in keep:
        out.extend(unit["blocks"])
    return out, covered


def match_quote(page, sec, quote_text, qtokens):
    """Return (blocks, kind, score) for one quote.

    The cited section is searched first, then its subsections, then the whole page — a
    heading or anchor can name the section a requirement *belongs to* while the sentence it
    quotes sits in a neighbour. Widening only stops early on a confident hit, so a mediocre
    in-section score never masks the real home of the quote elsewhere on the page.
    """
    units = page["units"]
    own = [u for u in units if u["section"] == sec["id"]]
    nested_ids = section_descendants(page, sec)
    nested = [u for u in units if u["section"] in nested_ids]
    low = quote_text.lower()

    best_blocks, best_score = [], 0.0
    for scope in (own, nested, units):
        if not scope:
            continue
        hits = [b for u in scope for b in u["blocks"] if low and low in b["low"]]
        if hits:
            return hits[:1], EXACT, 1.0
        blocks, score = best_in_scope(qtokens, scope)
        if score > best_score:
            best_blocks, best_score = blocks, score
        if best_score >= CONFIDENT:
            break

    if best_score >= CONFIDENT:
        return best_blocks, FUZZY, best_score
    if best_score >= PLACEABLE:
        return best_blocks, APPROX, best_score
    return [], SECTION_ONLY, best_score


def drop_matched(remaining, btokens):
    """Remove the tokens of `remaining` that a block accounts for, preserving order."""
    bset = {}
    for t in btokens:
        bset[t] = bset.get(t, 0) + 1
    out = []
    for t in remaining:
        if bset.get(t):
            bset[t] -= 1
        else:
            out.append(t)
    return out


# ---------------------------------------------------------------------------
# Requirements and RTL tags
# ---------------------------------------------------------------------------

ID_RE = re.compile(r"^spec-(?P<family>[a-z0-9_]+)\.(?P<group>[a-z]+)(?P<num>[0-9]+)$")
TAG_RE = re.compile(r"//@req-(spec-[a-z0-9_]+\.[a-z]+[0-9]+)")
TAG_SUFFIXES = (".sv", ".svh", ".v", ".vh", ".scala", ".nlhdl", ".md", ".yaml", ".yml")

# Trees walked for //@req- tags. The NL_HDL spec of a module and the code emitted from it
# live in different trees, so one root cannot feed both right-hand panes: src/main/nlhdl
# supplies the NL_HDL pane, the emitted Chisel under src/main/scala (plus the hand-written
# integration seam alongside it) and the SystemVerilog under src/main/sv supply the RTL pane.
DEFAULT_TAG_ROOTS = (
    os.path.join("src", "main", "nlhdl"),
    os.path.join("src", "main", "scala"),
    os.path.join("src", "main", "sv"),
)

# Directory names never worth walking: VCS metadata, the requirement YAMLs themselves
# (a req citing its own id is not an implementation site), sbt/python build output.
PRUNE_DIRS = (".git", "reqs", "__pycache__", "target", "project")


def load_reqs(reqs_dir):
    families, reqs = {}, []
    fam_path = os.path.join(reqs_dir, "families.yaml")
    registry = {}
    if os.path.isfile(fam_path):
        with io.open(fam_path, "r", encoding="utf-8") as fh:
            registry = yaml.safe_load(fh) or {}
    for key, entry in sorted((registry.get("families") or {}).items()):
        families[key] = {
            "key": key,
            "description": normalize((entry or {}).get("description") or ""),
            "sources": (entry or {}).get("sources") or [],
            "groups": {},
            "count": 0,
        }

    for name in sorted(os.listdir(reqs_dir)):
        if not name.startswith("spec-") or not name.endswith((".yaml", ".yml")):
            continue
        with io.open(os.path.join(reqs_dir, name), "r", encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
        family = doc.get("family") or name[5:].rsplit(".", 1)[0]
        fam = families.setdefault(family, {
            "key": family, "description": "", "sources": [], "groups": {}, "count": 0,
        })
        fam["description"] = fam["description"] or normalize(doc.get("description") or "")
        for gk, gv in (doc.get("groups") or {}).items():
            fam["groups"][gk] = normalize(gv or "")
        for entry in (doc.get("reqs") or []):
            if not isinstance(entry, dict):
                continue
            rid = entry.get("id") or ""
            m = ID_RE.match(rid)
            src = entry.get("source") or {}
            quote = (src.get("quote") or "").strip()
            reqs.append({
                "id": rid,
                "family": family,
                "group": m.group("group") if m else "",
                "num": int(m.group("num")) if m else 0,
                "kind": entry.get("kind") or "",
                "statement": normalize(entry.get("statement") or ""),
                "notes": normalize(entry.get("notes") or ""),
                "file": src.get("file") or "",
                "heading": normalize(src.get("heading") or ""),
                "anchor": src.get("anchor") or "",
                "quote_raw": quote,
                "quote_text": strip_rst(quote),
                "blocks": [],
                "drifted": False,
                "match": UNMATCHED,
                "score": 0.0,
                "impl": {"nlhdl": [], "rtl": []},
            })
            fam["count"] += 1
    reqs.sort(key=lambda r: (r["family"], r["group"], r["num"]))
    return families, reqs


def scan_tags(roots, repo_root):
    """Find //@req-<id> tags under every scanned root.

    Returns {req_id: [{file, line, kind, origin, snippet}]}.

    `kind` picks the pane: a ``*.nlhdl.*`` file is NL_HDL source, anything else is RTL.
    `origin` separates the two kinds of RTL site the viewer now shows side by side — code
    emitted from an NL_HDL spec (a path with a ``generated`` component) and the hand-written
    integration seam that wires it into BOOM. Generated sites sort first: they are the
    module the requirement is about, the seam is where it is plugged in.

    Roots may nest, so hits are deduped by (file, line, id).
    """
    hits, seen = {}, set()
    for root in roots:
        if not os.path.isdir(root):
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in PRUNE_DIRS]
            for name in sorted(filenames):
                if not name.endswith(TAG_SUFFIXES):
                    continue
                path = os.path.join(dirpath, name)
                rel = os.path.relpath(path, repo_root)
                kind = "nlhdl" if ".nlhdl." in name or name.endswith(".nlhdl") else "rtl"
                origin = ("generated" if "generated" in rel.split(os.sep)
                          else "handwritten")
                try:
                    with io.open(path, "r", encoding="utf-8", errors="replace") as fh:
                        lines = fh.read().splitlines()
                except IOError:
                    continue
                for i, line in enumerate(lines):
                    for rid in TAG_RE.findall(line):
                        key = (rel, i + 1, rid)
                        if key in seen:
                            continue
                        seen.add(key)
                        context = lines[i:i + 12]
                        hits.setdefault(rid, []).append({
                            "file": rel, "line": i + 1, "kind": kind, "origin": origin,
                            "snippet": "\n".join(context),
                        })
    for sites in hits.values():
        sites.sort(key=lambda h: (h["origin"] != "generated", h["file"], h["line"]))
    return hits


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build(repo_root, html_dir, reqs_dir, rtl_dirs, out_dir, quiet=False):
    if isinstance(rtl_dirs, str):
        rtl_dirs = [rtl_dirs]
    if not os.path.isdir(html_dir):
        sys.stderr.write(
            "error: rendered docs not found: %s\n"
            "       build them first:  cd docs_caracal && make html\n" % html_dir)
        sys.exit(2)

    doc_names = sorted(n[:-5] for n in os.listdir(html_dir) if n.endswith(".html"))
    pages = {}
    for name in doc_names:
        page = load_page(os.path.join(html_dir, name + ".html"), name, set(doc_names))
        if page is None:
            continue
        for i, blk in enumerate(page["blocks"]):
            blk["id_index"] = i
        page["units"] = build_units(page)
        pages[name] = page

    families, reqs = load_reqs(reqs_dir)
    tags = scan_tags(rtl_dirs, repo_root)

    blocks_by_id = {}
    for page in pages.values():
        for blk in page["blocks"]:
            blocks_by_id[blk["id"]] = blk

    report = {"exact": 0, "fuzzy": 0, "approx": 0, "section": 0, "none": 0,
              "unmatched": [], "no_page": [], "no_section": [], "drifted": []}
    sources = {}

    for req in reqs:
        # A quote that no longer occurs in the .rst cannot be placed on the page either;
        # say so explicitly rather than letting the fuzzy scorer guess at a neighbour.
        rel = req["file"]
        if rel and rel not in sources:
            try:
                with io.open(os.path.join(repo_root, rel), "r", encoding="utf-8") as fh:
                    sources[rel] = normalize(fh.read()).lower()
            except IOError:
                sources[rel] = None
        src_norm = sources.get(rel)
        if src_norm is not None and normalize(req["quote_raw"]).lower() not in src_norm:
            req["drifted"] = True
            report["drifted"].append(req["id"])

        for hit in tags.get(req["id"], []):
            req["impl"][hit["kind"]].append(hit)

        doc_name = os.path.basename(req["file"]).rsplit(".", 1)[0]
        page = pages.get(doc_name)
        if page is None:
            report["no_page"].append(req["id"])
            report["none"] += 1
            continue
        req["doc"] = doc_name
        sec, how = find_section(page, req["anchor"], req["heading"])
        if sec is None:
            report["no_section"].append(req["id"])
            report["none"] += 1
            continue
        req["section"] = sec["id"]
        req["section_how"] = how
        hits, kind, score = match_quote(page, sec, req["quote_text"],
                                        tokens(req["quote_text"]))
        if not hits:
            # Fall back to the section heading block so the req is still reachable.
            head = [b for b in page["blocks"]
                    if b["section"] == sec["id"] and re.match(r"^h[1-6]$", b["tag"])]
            hits = head[:1]
            report["unmatched"].append((req["id"], req["heading"], req["quote_text"][:90]))
        req["blocks"] = [b["id"] for b in hits]
        req["match"] = kind
        req["score"] = round(score, 3)
        for blk in hits:
            blk["reqs"].append(req["id"])
        report[kind] += 1

    # ---- emit --------------------------------------------------------------
    docs_out = os.path.join(out_dir, "docs")
    if not os.path.isdir(docs_out):
        os.makedirs(docs_out)

    doc_index = []
    for name in toctree_order(repo_root, set(pages)):
        page = pages[name]
        with io.open(os.path.join(docs_out, name + ".html"), "w", encoding="utf-8") as fh:
            fh.write(page["html"])
        doc_index.append({
            "name": name,
            "title": page["title"],
            "reqs": sum(len(b["reqs"]) for b in page["blocks"]),
            "sections": [{
                "id": s["id"], "anchors": s["anchors"], "heading": s["heading"],
                "number": s["number"], "level": s["level"], "parent": s["parent"],
            } for s in page["sections"]],
            "blocks": [{
                "id": b["id"], "doc": b["doc"], "tag": b["tag"], "section": b["section"],
                "heading": b["heading"], "reqs": b["reqs"],
                "text": b["text"][:400],
            } for b in page["blocks"]],
        })

    # Land on the first *numbered* chapter carrying requirements: the numbered toctree is
    # the core specification, so that is chapter 1 of what a reader wants, not the glossary.
    def numbered(doc):
        return any(s["number"] for s in doc["sections"])

    landing = (next((d["name"] for d in doc_index if d["reqs"] and numbered(d)), None) or
               next((d["name"] for d in doc_index if d["reqs"]), None) or
               (doc_index[0]["name"] if doc_index else None))
    index = {
        "generated_from": os.path.relpath(html_dir, repo_root),
        "landing": landing,
        "repo_root": repo_root,
        "families": [families[k] for k in sorted(families)],
        "docs": doc_index,
        "reqs": [{
            "id": r["id"], "family": r["family"], "group": r["group"], "kind": r["kind"],
            "statement": r["statement"], "notes": r["notes"],
            "source": {"file": r["file"], "heading": r["heading"], "anchor": r["anchor"],
                       "quote": r["quote_raw"], "quote_text": r["quote_text"]},
            "doc": r.get("doc"), "section": r.get("section"),
            "blocks": r["blocks"], "match": r["match"], "score": r["score"],
            "drifted": r["drifted"],
            "impl": r["impl"],
        } for r in reqs],
        "stats": {
            "reqs": len(reqs),
            "docs": len(doc_index),
            "blocks": len(blocks_by_id),
            "blocks_with_reqs": sum(1 for b in blocks_by_id.values() if b["reqs"]),
            "exact": report["exact"], "fuzzy": report["fuzzy"],
            "approx": report["approx"], "section_only": report["section"],
            "unmatched": report["none"], "drifted": len(report["drifted"]),
            "tagged": sum(1 for r in reqs if r["impl"]["nlhdl"] or r["impl"]["rtl"]),
            "tagged_nlhdl": sum(1 for r in reqs if r["impl"]["nlhdl"]),
            "tagged_rtl": sum(1 for r in reqs if r["impl"]["rtl"]),
        },
    }
    with io.open(os.path.join(out_dir, "index.json"), "w", encoding="utf-8") as fh:
        fh.write(json.dumps(index, indent=1, sort_keys=False, ensure_ascii=False))

    if not quiet:
        s = index["stats"]
        print("req-viewer index -> %s" % out_dir)
        print("  %d docs, %d blocks (%d carry reqs), %d reqs"
              % (s["docs"], s["blocks"], s["blocks_with_reqs"], s["reqs"]))
        print("  quote match: %d exact, %d fuzzy, %d approximate, %d section-only, "
              "%d unplaced" % (s["exact"], s["fuzzy"], s["approx"], s["section_only"],
                               s["unmatched"]))
        if s["drifted"]:
            print("  %d quote(s) no longer occur in the .rst source (drift)"
                  % s["drifted"])
        print("  tags: %d reqs implemented (%d nlhdl, %d rtl) across %d root(s)"
              % (s["tagged"], s["tagged_nlhdl"], s["tagged_rtl"], len(rtl_dirs)))
    return index, report


def resolve_tag_roots(repo_root, given):
    """Absolute tag-scan roots from --rtl-dir values, or the defaults when none were given.

    A value may itself be comma-separated, so `--rtl-dir a,b` and `--rtl-dir a --rtl-dir b`
    mean the same thing. Order is preserved and duplicates dropped, because it decides the
    walk order behind the deduping in scan_tags.
    """
    parts = []
    for value in (given or []):
        parts.extend(p.strip() for p in value.split(",") if p.strip())
    if not parts:
        parts = list(DEFAULT_TAG_ROOTS)
    out = []
    for part in parts:
        path = part if os.path.isabs(part) else os.path.join(repo_root, part)
        path = os.path.abspath(path)
        if path not in out:
            out.append(path)
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_root = os.path.abspath(os.path.join(here, "..", ".."))

    ap = argparse.ArgumentParser(description="Build the requirement-viewer trace index.")
    ap.add_argument("--repo-root", default=default_root)
    ap.add_argument("--html-dir", default=None,
                    help="rendered Sphinx pages (default docs_caracal/_build/html/src)")
    ap.add_argument("--reqs-dir", default=None,
                    help="requirement YAMLs (default src/main/nlhdl/reqs)")
    ap.add_argument("--rtl-dir", action="append", default=None, metavar="DIR",
                    help="tree scanned for //@req- tags; repeatable, or comma-separated "
                         "(default %s)" % ", ".join(DEFAULT_TAG_ROOTS))
    ap.add_argument("--out", default=None,
                    help="output directory (default docs_caracal/_build/reqviewer)")
    ap.add_argument("--report", action="store_true",
                    help="list requirements whose quote could not be placed")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    root = os.path.abspath(args.repo_root)
    html_dir = args.html_dir or os.path.join(root, "docs_caracal", "_build", "html", "src")
    reqs_dir = args.reqs_dir or os.path.join(root, "src", "main", "nlhdl", "reqs")
    rtl_dirs = resolve_tag_roots(root, args.rtl_dir)
    out_dir = args.out or os.path.join(root, "docs_caracal", "_build", "reqviewer")

    _, report = build(root, html_dir, reqs_dir, rtl_dirs, out_dir, quiet=args.quiet)

    if args.report:
        if report["no_page"]:
            print("\nno rendered page (%d):" % len(report["no_page"]))
            for rid in report["no_page"][:40]:
                print("  %s" % rid)
        if report["no_section"]:
            print("\nsection not found (%d):" % len(report["no_section"]))
            for rid in report["no_section"][:40]:
                print("  %s" % rid)
        if report["drifted"]:
            print("\nquote drifted from the .rst (%d):" % len(report["drifted"]))
            for rid in report["drifted"]:
                print("  %s" % rid)
        if report["unmatched"]:
            print("\nquote placed on the heading only (%d):" % len(report["unmatched"]))
            for rid, heading, quote in report["unmatched"][:40]:
                print("  %-18s [%s] %s" % (rid, heading, quote))
    return 0


if __name__ == "__main__":
    sys.exit(main())

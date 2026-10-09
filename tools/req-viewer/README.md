# req-viewer — spec ⇄ requirement ⇄ RTL browser

A four-pane local web app for reading the Caracal spec and the requirements derived from it
side by side. Hover a paragraph in the rendered spec and the requirements that quote it
appear above; click a requirement and its implementation sites appear on the right.

```
┌──────────────────────────────────────────────────────────────────────┐
│ toolbar: spec page · section jump · find · only-cited · Rebuild      │
├──────────────────────────────────────────────────────────────────────┤
│ REQUIREMENTS — the reqs citing the hovered (or pinned) prose         │
├───────────────────────────────────┬──────────────────────────────────┤
│ SPEC                              │ NL_HDL — implementation site     │
│ docs_caracal rendered HTML,       ├──────────────────────────────────┤
│ every block hoverable             │ GENERATED RTL — emitted module   │
└───────────────────────────────────┴──────────────────────────────────┘
```

All three splitters drag; double-click a splitter to reset it. Sizes, the current page and
the *only cited prose* toggle persist in `localStorage`.

## Run it

```sh
cd docs_caracal && make html          # once, and after any spec edit
python3 tools/req-viewer/serve.py     # http://localhost:8765

# To Kill the Server
# by port (also tells you the PID)
ss -ltnp | grep 8765

# by name
pkill -f 'req-viewer/serve.py'

# specific port, if you run several
pkill -f 'req-viewer/serve.py --port 8765'
```

Flags: `--port`, `--host`, `--no-build` (serve the existing index), and `--html-dir` /
`--reqs-dir` / `--rtl-dir` / `--out` to point at non-default trees. `--rtl-dir` is repeatable
(and accepts a comma-separated list) because the tags live in more than one tree; passing it
replaces the default set rather than adding to it. Working over SSH, forward the port:
`ssh -L 8765:localhost:8765 <host>`.

The index is rebuilt at startup and by the **Rebuild** button, so a `make html` plus Rebuild
is enough to pick up spec edits, new requirements or freshly tagged RTL — no restart.

## Interaction

| Action | Result |
|---|---|
| hover a paragraph, list item, table row, code block | the top pane lists every requirement whose quote lands there |
| click that prose | the list is **pinned** — hovering no longer changes it |
| `Esc` or **Clear** | unpin, drop the selection, clear the marks |
| click a requirement card | its exact quote is marked in the spec; the right panes show its NL_HDL and RTL sites |
| hover a card | its blocks light up in the spec without committing |
| `j` / `k` (or ↑ / ↓) | walk the listed requirements |
| `/` | focus **Find** — searches ids, statements, quotes and headings across all families |
| **only cited prose** | fades every block no requirement cites (a coverage read) |
| a cross-reference inside the spec | loads that page in the pane instead of navigating away |
| **whole file** on an implementation card | swaps the snippet for the entire file, scrolled to the tag |
| `:<line>` on an implementation card | opens the file and scrolls to that tag |

Selecting a requirement from another spec page switches the page automatically.

Colour code: blue left rule = some requirement cites this block; blue fill = hovering;
amber fill = pinned; yellow = the selected requirement's quote span.

## How the join works

Three artifacts are joined, all read-only:

| Artifact | Role |
|---|---|
| `docs_caracal/_build/html/src/*.html` | the rendered spec — supplies the hover targets |
| `src/main/nlhdl/reqs/spec-*.yaml` | requirements, each with `source.{file,anchor,heading,quote}` |
| `src/main/nlhdl/**` | scanned for `//@req-<id>` tags — fills the NL_HDL pane |
| `src/main/scala/**`, `src/main/sv/**` | scanned for the same tags — fills the RTL pane |

`build_index.py` does the work:

1. **Blocks.** Each rendered page's article body is parsed, and every innermost block
   element (`p`, `li`, `pre`, `td`, `dt`, `h1`–`h6`, …) is stamped with `data-block`. Those
   are the hover targets. Table cells additionally remember their row.
2. **Section.** A requirement's `anchor` is matched against section ids and the empty
   `<span id="…">` that docutils emits for an `.. _label:`; `heading` is the fallback.
3. **Quote.** The quote is *reStructuredText*, so it is stripped of inline markup
   (` ``literal`` `, `**strong**`, `:doc:`…``, `|caracal|`) and folded (smart quotes, dashes,
   whitespace) to the plain text a reader sees. Exact containment inside a block is the
   common case — 96% of requirements land on exactly one block. Otherwise a token-LCS
   coverage score picks the best block, and a *contiguous* run of neighbouring blocks is
   accepted when the quote genuinely spans several (a lead-in plus its bullet list, or a
   `list-table` row rendered as one cell per column).
4. **Confidence.** Every card carries its match kind — `quote exact`, `fuzzy NN%`, or
   `approx/section` when nothing on the page resembled the quote. A requirement is also
   flagged `quote drifted` when its quote no longer occurs in the `.rst` at all.

Unplaced or drifted counts in the toolbar are the honest signal, and usually mean one of:

- the HTML build is older than the `.rst` — run `make html`, then **Rebuild**;
- the spec was edited and the requirement needs re-extraction —
  `spec-to-reqs/scripts/validate-reqs.py` reports the same drift authoritatively.

In-browser matching mirrors the Python folding rules, so the marked span is the same text
the index matched; when the exact span cannot be located inside a block (a rendered
`:doc:` title, a reflowed literal), the longest locatable run of the quote is marked and the
whole block stays highlighted.

## The right-hand panes

They are driven by `//@req-<id>` tags: files matching `*.nlhdl.*` feed the NL_HDL pane, other
HDL sources feed the RTL pane. A requirement with no tag of that kind gets a placeholder
naming the tag to add and where it is expected.

One card per **file**, not per tag — a requirement is usually tagged at several points in the
same module, and repeated 12-line snippets hide that they are one file. The header carries the
path, the origin, and a `:<line>` button per tag. A card opens at the **whole file**, scrolled
to the first tag; **snippet** collapses it to the tagged lines plus context. Either view is
scrollable, numbered with real file line numbers, and marks the tagged lines — amber for this
requirement, grey for a line tagged for a *different* one, which is how you see a signal
answering to more than one line of the spec. The choice sticks (`localStorage`, `rv.implFull`),
since it is a reading habit rather than a per-file decision.

Full files come from `/api/file?path=<repo-relative>`, whose allowlist is exactly the tag
scan's: a path under a scanned root (symlinks resolved first) with a taggable suffix, under
8 MB. So the route can only serve files whose snippets the index already publishes.

The two panes read from **different trees**, which is why the scan takes a list of roots
(`DEFAULT_TAG_ROOTS` in `build_index.py`):

| Tree | Pane | What it is |
|---|---|---|
| `src/main/nlhdl/**/*.nlhdl.scala` | NL_HDL | the module spec, written by `/nlhdl gen-nlhdl` |
| `src/main/scala/v4/vec/generated/**` | RTL | the Chisel emitted from it by `/nlhdl gen-rtl` |
| `src/main/scala/v4/{common,exu}/**`, `src/main/sv/**` | RTL | the hand-written seam wiring it into BOOM |

Scanning only `src/main/nlhdl` — the original default — left the RTL pane permanently empty,
since generated Chisel is written outside that tree. RTL sites are therefore labelled
`generated` or `hand-written`, generated first: the emitted module is what the requirement is
about, the seam is where it gets plugged in.

## Files

```
tools/req-viewer/
  build_index.py     block extraction, quote matching, tag scan, index.json emitter
  serve.py           localhost server: app, index, tagged sources, instrumented pages, assets
  app/index.html     the four-pane shell
  app/app.css        shell styling
  app/app.js         hover/pin/select wiring, quote marking, splitters, search
  app/page.css       injected into the spec iframe: hover, pin and quote styles
```

`build_index.py` runs standalone too — `--report` lists every requirement whose quote could
not be placed, and `--out DIR` writes the index elsewhere. Requires only PyYAML.

## Smoke test

`app/_probe.html` drives the real UI in a headless browser — hover, pin, hover-while-pinned,
card selection, quote marking across every requirement on the page, both implementation panes
on a requirement tagged in both trees, whole-file expansion and its `/api/file` guard, family
chips, search, cross-page selection, clear, and a splitter drag:

```sh
python3 tools/req-viewer/serve.py --port 8803 &
google-chrome --headless=new --disable-gpu --no-sandbox --window-size=1500,1000 \
  --virtual-time-budget=40000 --dump-dom http://127.0.0.1:8803/app/_probe.html \
  | grep -oE '(PASS|FAIL)  [^<]*'
```

Last run: 39 passed, 0 failed.

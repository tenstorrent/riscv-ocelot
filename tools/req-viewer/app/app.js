/* app.js — spec ⇄ requirement ⇄ RTL browser.
 *
 * Wiring:
 *   hover a block in the spec iframe  -> top pane lists the requirements citing it
 *   click a block                     -> that list is pinned (hover stops changing it)
 *   click a requirement card          -> its exact quote is marked in the spec, and the
 *                                        right-hand panes show its NL_HDL / RTL sites
 *   Esc / Clear                       -> unpin
 *
 * The iframe is same-origin, so its DOM is driven directly rather than over postMessage.
 */

'use strict';

const S = {
  index: null,
  docs: new Map(),        // name -> doc record from index.json
  blocks: new Map(),      // block id -> block record
  reqs: new Map(),        // req id -> req record
  doc: null,              // current doc name
  frame: null,            // iframe document
  hoverBlock: null,
  listedBlock: null,      // block whose requirements the top pane is showing
  pinnedBlock: null,
  selectedReq: null,
  listed: [],             // requirement ids currently in the top pane
  listMode: 'idle',       // idle | hover | pin | search | req
  marks: [],
};

// Debug handle: `RV.index`, `RV.blocks`, `RV.reqs` from the console.
window.RV = S;

const $ = (sel) => document.querySelector(sel);
const el = (tag, cls, text) => {
  const node = document.createElement(tag);
  if (cls) node.className = cls;
  if (text != null) node.textContent = text;
  return node;
};

/* ── text folding: must mirror build_index.py's FOLD/normalize ──────────── */

function fold(s) {
  return s
    .replace(/[’‘]/g, "'")
    .replace(/[“”]/g, '"')
    .replace(/[—–‑−]/g, '-')
    .replace(/[   ]/g, ' ')
    .replace(/​/g, '')
    .replace(/…/g, '...');
}

/* ── boot ───────────────────────────────────────────────────────────────── */

async function boot() {
  restoreLayout();
  wireSplitters();
  wireToolbar();
  wireKeys();
  await loadIndex();
  const saved = localStorage.getItem('rv.doc');
  await openDoc(S.docs.has(saved) ? saved : (S.index.landing || S.index.docs[0].name));
  renderReqList([], 'idle');
  renderImpl(null);
}

async function loadIndex() {
  const res = await fetch('/api/index.json', { cache: 'no-store' });
  if (!res.ok) throw new Error('cannot load /api/index.json');
  S.index = await res.json();

  S.docs.clear(); S.blocks.clear(); S.reqs.clear();
  for (const doc of S.index.docs) {
    S.docs.set(doc.name, doc);
    for (const b of doc.blocks) S.blocks.set(b.id, b);
  }
  for (const r of S.index.reqs) S.reqs.set(r.id, r);

  const sel = $('#doc-select');
  sel.textContent = '';
  for (const doc of S.index.docs) {
    const opt = el('option', null, `${doc.title}  (${doc.reqs})`);
    opt.value = doc.name;
    sel.appendChild(opt);
  }

  const st = S.index.stats;
  const bits = [
    `${st.reqs} reqs`,
    `${st.blocks_with_reqs}/${st.blocks} blocks cited`,
    `${st.exact} exact · ${st.fuzzy} fuzzy`,
    `${st.tagged} implemented`,
  ];
  const stale = (st.section_only || 0) + (st.approx || 0) + (st.unmatched || 0);
  if (st.drifted) bits.push(`${st.drifted} quote${st.drifted === 1 ? '' : 's'} drifted from .rst`);
  if (stale) bits.push(`${stale} unplaced — rebuild docs?`);
  const node = $('#stats');
  node.textContent = bits.join('   ·   ');
  node.style.color = (st.drifted || stale) ? 'var(--warn)' : '';
  node.title = (st.drifted || stale)
    ? 'Requirement quotes that no rendered block matches. Usually the HTML build is older '
      + 'than the .rst: run `make html` in docs_caracal, then Rebuild.'
    : '';
}

/* ── the spec pane ──────────────────────────────────────────────────────── */

const WRAPPER = (body) => `<!DOCTYPE html><html><head><meta charset="utf-8">
<link rel="stylesheet" href="/sphinx/_static/pygments.css">
<link rel="stylesheet" href="/sphinx/_static/css/theme.css">
<link rel="stylesheet" href="/app/page.css">
</head><body class="wy-body-for-nav"><div class="wy-nav-content rv-wrap"><div class="rst-content">
<div class="document" role="main">${body}</div></div></div></body></html>`;

async function openDoc(name, frag, opts) {
  const keepSelection = !opts || opts.restoreSelection !== false;
  const res = await fetch(`/docs/${name}.html`, { cache: 'no-store' });
  if (!res.ok) { toast(`no rendered page for ${name}`); return; }
  const body = await res.text();
  const doc = S.docs.get(name);

  S.doc = name;
  localStorage.setItem('rv.doc', name);
  $('#doc-select').value = name;
  $('#spec-title').textContent = doc.title;
  $('#spec-sub').innerHTML = `<b>${doc.reqs}</b> requirement citations across ` +
                             `<b>${doc.blocks.length}</b> blocks`;
  fillSections(doc);

  const frame = $('#page');
  await new Promise((resolve) => {
    frame.addEventListener('load', resolve, { once: true });
    frame.srcdoc = WRAPPER(body);
  });
  S.frame = frame.contentDocument;
  S.hoverBlock = null;
  S.listedBlock = null;
  S.pinnedBlock = null;
  S.marks = [];
  $('#pin-badge').hidden = true;
  instrument();
  if (frag) scrollToFragment(frag);
  if (keepSelection && S.selectedReq && S.reqs.get(S.selectedReq).doc === name) {
    selectReq(S.selectedReq, false);
  }
}

function instrument() {
  const d = S.frame;
  d.body.classList.toggle('rv-dim', $('#dim-toggle').checked);

  for (const node of d.querySelectorAll('[data-block]')) {
    const rec = S.blocks.get(node.dataset.block);
    if (rec && rec.reqs.length) {
      node.classList.add('rv-cited');
      node.title = rec.reqs.length === 1 ? `1 requirement: ${rec.reqs[0]}`
                                         : `${rec.reqs.length} requirements`;
    }
  }

  d.addEventListener('mouseover', (ev) => {
    const node = ev.target.closest('[data-block]');
    if (!node || node === S.hoverBlock) return;
    setHover(node);
  });

  d.addEventListener('click', (ev) => {
    const link = ev.target.closest('a[data-doc], a[data-frag]');
    if (link) {
      ev.preventDefault();
      if (link.dataset.doc) openDoc(link.dataset.doc, link.dataset.frag);
      else scrollToFragment(link.dataset.frag);
      return;
    }
    if (ev.target.closest('a[href]')) return;   // external link: leave it alone
    const node = ev.target.closest('[data-block]');
    if (!node) return;
    const rec = S.blocks.get(node.dataset.block);
    if (!rec || !rec.reqs.length) { toast('no requirement cites that text'); return; }
    pin(node);
  });
}

function setHover(node) {
  S.hoverBlock = node;
  const rec = S.blocks.get(node.dataset.block);
  if (!rec || !rec.reqs.length) return;         // inert prose: leave the list alone
  if (S.pinnedBlock) return;                    // pinned: hover is display-only
  if (S.listedBlock) S.listedBlock.classList.remove('rv-hover');
  S.listedBlock = node;
  node.classList.add('rv-hover');
  renderReqList(rec.reqs, 'hover', rec);
}

function pin(node) {
  clearPin(false);
  if (S.listedBlock) S.listedBlock.classList.remove('rv-hover');
  S.pinnedBlock = node;
  S.listedBlock = node;
  node.classList.add('rv-pinned');
  node.classList.remove('rv-hover');
  const rec = S.blocks.get(node.dataset.block);
  $('#pin-badge').hidden = false;
  renderReqList(rec.reqs, 'pin', rec);
}

function clearPin(reset = true) {
  if (S.pinnedBlock) S.pinnedBlock.classList.remove('rv-pinned');
  S.pinnedBlock = null;
  $('#pin-badge').hidden = true;
  if (reset) {
    if (S.listedBlock) S.listedBlock.classList.remove('rv-hover');
    S.listedBlock = null;
    clearSelection();
    renderReqList([], 'idle');
    renderImpl(null);
  }
}

function clearSelection() {
  S.selectedReq = null;
  unmark();
  if (S.frame) {
    for (const node of S.frame.querySelectorAll('.rv-selected')) {
      node.classList.remove('rv-selected');
    }
  }
  for (const card of document.querySelectorAll('.card.selected')) {
    card.classList.remove('selected');
  }
}

/* ── the requirements pane ──────────────────────────────────────────────── */

function renderReqList(ids, mode, blockRec) {
  S.listed = ids.slice();
  S.listMode = mode;
  const list = $('#reqs-list');
  list.textContent = '';
  list.classList.toggle('reqs-idle', !ids.length);

  const sub = $('#reqs-source');
  if (mode === 'idle') {
    sub.textContent = 'Hover any paragraph in the spec below; click to pin it.';
  } else if (mode === 'search') {
    sub.innerHTML = `<b>${ids.length}</b> requirement${ids.length === 1 ? '' : 's'} match ` +
                    `“${escapeHtml($('#search').value.trim())}” across the whole corpus`;
  } else {
    const where = blockRec ? (blockRec.heading || S.docs.get(blockRec.doc || S.doc).title) : '';
    sub.innerHTML = `<b>${ids.length}</b> requirement${ids.length === 1 ? '' : 's'} cite ` +
                    `this ${blockRec ? tagName(blockRec.tag) : 'text'}` +
                    (where ? ` in <b>${escapeHtml(where)}</b>` : '') +
                    (mode === 'pin' ? ' — pinned' : '');
  }

  if (!ids.length) {
    if (mode === 'search') {
      list.appendChild(el('div', 'empty',
        'Nothing matches. Try a requirement id (spec-cii.a4), a phrase, or clear the box.'));
    } else {
      renderIdle(list);
    }
    return;
  }
  for (const id of ids) {
    const req = S.reqs.get(id);
    if (req) list.appendChild(reqCard(req));
  }
}

/* Idle: instead of dead space, offer the current page's requirements by family. */
function renderIdle(list) {
  const doc = S.docs.get(S.doc);
  if (!doc) return;
  const onPage = S.index.reqs.filter((r) => r.doc === S.doc);
  const wrap = el('div', 'idle');
  wrap.appendChild(el('p', 'empty',
    'Hover any paragraph, list item, table row or code block in the spec to see the ' +
    'requirements that quote it; click to pin. Or start from a family on this page:'));
  const chips = el('div', 'chips');
  const byFamily = new Map();
  for (const req of onPage) {
    if (!byFamily.has(req.family)) byFamily.set(req.family, []);
    byFamily.get(req.family).push(req.id);
  }
  for (const [family, ids] of [...byFamily].sort((a, b) => b[1].length - a[1].length)) {
    const chip = el('button', 'chip', `${family} ${ids.length}`);
    chip.title = (S.index.families.find((f) => f.key === family) || {}).description || family;
    chip.addEventListener('click', () => {
      renderReqList(ids, 'family');
      $('#reqs-source').innerHTML = `<b>${ids.length}</b> requirements of family ` +
        `<b>${escapeHtml(family)}</b> on <b>${escapeHtml(doc.title)}</b>`;
    });
    chips.appendChild(chip);
  }
  if (onPage.length) {
    const all = el('button', 'chip chip-all', `all ${onPage.length}`);
    all.addEventListener('click', () => {
      renderReqList(onPage.map((r) => r.id), 'family');
      $('#reqs-source').innerHTML = `all <b>${onPage.length}</b> requirements citing ` +
        `<b>${escapeHtml(doc.title)}</b>`;
    });
    chips.appendChild(all);
  }
  wrap.appendChild(chips);
  list.appendChild(wrap);
}

function reqCard(req) {
  const card = el('div', 'card' + (req.drifted ? ' drifted' : ''));
  card.dataset.req = req.id;

  const top = el('div', 'card-top');
  top.appendChild(el('span', 'rid', req.id));
  top.appendChild(el('span', 'badge badge-fam', req.family));
  if (req.kind) top.appendChild(el('span', 'badge badge-kind', req.kind));
  card.appendChild(top);

  card.appendChild(el('p', 'card-statement', req.statement));
  if (req.source.quote_text) card.appendChild(el('p', 'card-quote', req.source.quote_text));

  const foot = el('div', 'card-foot');
  foot.appendChild(el('span', `badge badge-${req.match}`,
    req.match === 'exact' ? 'quote exact' : `${req.match} ${Math.round(req.score * 100)}%`));
  const n = req.impl.nlhdl.length, r = req.impl.rtl.length;
  foot.appendChild(el('span', `badge badge-impl-${n || r ? 'yes' : 'no'}`,
    n || r ? `nlhdl ${n} · rtl ${r}` : 'untagged'));
  if (req.drifted) foot.appendChild(el('span', 'badge badge-drift', 'quote drifted'));
  card.appendChild(foot);

  card.addEventListener('click', () => selectReq(req.id, true));
  card.addEventListener('mouseenter', () => previewReq(req.id));
  return card;
}

function previewReq(id) {
  const req = S.reqs.get(id);
  if (!req || req.doc !== S.doc || !S.frame) return;
  for (const node of S.frame.querySelectorAll('.rv-selected')) node.classList.remove('rv-selected');
  for (const bid of req.blocks) {
    const node = S.frame.querySelector(`[data-block="${cssEscape(bid)}"]`);
    if (node) node.classList.add('rv-selected');
  }
}

async function selectReq(id, scroll) {
  const req = S.reqs.get(id);
  if (!req) return;
  clearSelection();
  S.selectedReq = id;

  for (const card of document.querySelectorAll('.card')) {
    card.classList.toggle('selected', card.dataset.req === id);
  }
  renderImpl(req);

  if (req.doc && req.doc !== S.doc) {
    await openDoc(req.doc, null, { restoreSelection: false });
  }
  if (!S.frame) return;

  let first = null;
  for (const bid of req.blocks) {
    const node = S.frame.querySelector(`[data-block="${cssEscape(bid)}"]`);
    if (!node) continue;
    node.classList.add('rv-selected');
    if (!first) first = node;
    markQuote(node, req.source.quote_text);
  }
  if (first && scroll) {
    first.scrollIntoView({ behavior: 'smooth', block: 'center' });
    flash(first);
  }
}

/* ── quote marking inside a block ───────────────────────────────────────── */

function unmark() {
  for (const mark of S.marks) {
    const parent = mark.parentNode;
    if (!parent) continue;
    while (mark.firstChild) parent.insertBefore(mark.firstChild, mark);
    parent.removeChild(mark);
    parent.normalize();
  }
  S.marks = [];
}

/* Flatten a block's text nodes into one whitespace-collapsed string, keeping a
   character -> (node, offset) map so a match can be turned back into DOM ranges. */
function flatten(root) {
  const chars = [], map = [];
  const walker = S.frame.createTreeWalker(root, NodeFilter.SHOW_TEXT, null);
  let space = true;
  while (walker.nextNode()) {
    const node = walker.currentNode;
    if (node.parentElement && node.parentElement.closest('.headerlink')) continue;
    const text = fold(node.nodeValue);
    for (let i = 0; i < text.length; i++) {
      const ch = text[i];
      if (/\s/.test(ch)) {
        if (space) continue;
        chars.push(' '); map.push([node, i]); space = true;
      } else {
        chars.push(ch); map.push([node, i]); space = false;
      }
    }
  }
  return { text: chars.join(''), map };
}

/* The stored quote is RST stripped to plain text, so it usually occurs verbatim. When it
   does not (a :doc: role rendered as a title, a reflowed literal), fall back to the longest
   leading run of words that does. */
function locate(haystack, needle) {
  const hay = haystack.toLowerCase();
  const want = needle.toLowerCase().trim();
  if (!want) return null;
  let at = hay.indexOf(want);
  if (at >= 0) return [at, want.length];

  const words = want.split(' ');
  const floor = words.length <= 5 ? 2 : 4;
  for (let n = words.length - 1; n >= floor; n--) {
    const probe = words.slice(0, n).join(' ');
    at = hay.indexOf(probe);
    if (at >= 0) return [at, probe.length];
  }
  for (let start = 1; start + floor <= words.length; start++) {
    const probe = words.slice(start, start + Math.min(12, words.length - start)).join(' ');
    at = hay.indexOf(probe);
    if (at >= 0) return [at, probe.length];
  }
  return null;
}

/* A `list-table` row quote — "* - VL - 1 - 64" — renders as one cell per column, so no
   single block contains the row. Fall back to the individual cell texts. */
function needles(quoteText) {
  const whole = quoteText.replace(/^\*\s*-\s*/, '').trim();
  const cells = quoteText.split(/\s+\*?\s*-\s+/)
      .map((part) => part.replace(/^\*\s*/, '').trim())
      .filter((part) => part.length > 1);
  return [whole].concat(cells.length > 1 ? cells : []);
}

function markQuote(block, quoteText) {
  if (!quoteText) return;
  for (const needle of needles(quoteText)) {
    if (markOne(block, needle)) return;
  }
}

function markOne(block, quoteText) {
  const flat = flatten(block);
  const hit = locate(flat.text, quoteText);
  if (!hit) return false;
  const [start, len] = hit;

  // Group the matched characters into one range per text node.
  const spans = [];
  for (let i = start; i < start + len && i < flat.map.length; i++) {
    const [node, off] = flat.map[i];
    const last = spans[spans.length - 1];
    if (last && last.node === node && off === last.end) last.end = off + 1;
    else spans.push({ node, start: off, end: off + 1 });
  }
  // Reverse order: wrapping splits text nodes, which would invalidate later offsets.
  for (const span of spans.reverse()) {
    try {
      const range = S.frame.createRange();
      range.setStart(span.node, span.start);
      range.setEnd(span.node, Math.min(span.end, span.node.nodeValue.length));
      const mark = S.frame.createElement('mark');
      mark.className = 'rv-quote';
      range.surroundContents(mark);
      S.marks.push(mark);
    } catch (err) { /* a split across element edges: skip that fragment */ }
  }
  return S.marks.length > 0;
}

/* ── implementation panes ───────────────────────────────────────────────── */

function renderImpl(req) {
  fillImpl('#nlhdl-body', '#nlhdl-sub', req, 'nlhdl');
  fillImpl('#rtl-body', '#rtl-sub', req, 'rtl');
}

const IMPL_COPY = {
  nlhdl: {
    what: 'NL_HDL source',
    where: 'src/main/nlhdl/**/*.nlhdl.scala',
    how: 'written by <code>/nlhdl gen-nlhdl</code>, module map in <code>src/main/nlhdl/hierarchy.yaml</code>',
  },
  rtl: {
    what: 'generated RTL',
    where: 'src/main/scala/v4/vec/generated/**',
    how: 'produced by <code>/nlhdl gen-rtl</code>; hand-written seam sites under <code>src/main/scala/v4/</code> and <code>src/main/sv/</code> show here too',
  },
};

// The RTL pane mixes code emitted from an NL_HDL spec with the hand-written seam that wires
// it into BOOM. Both are real implementation sites, but they are read differently, so say
// which is which instead of leaving the reader to infer it from the path.
const ORIGIN_LABEL = { generated: 'generated', handwritten: 'hand-written' };

function fillImpl(bodySel, subSel, req, kind) {
  const body = $(bodySel);
  const sub = $(subSel);
  body.textContent = '';

  if (!req) {
    sub.textContent = 'Select a requirement.';
    body.appendChild(el('div', 'empty', 'Pick a requirement card above to see where it is implemented.'));
    return;
  }

  const hits = req.impl[kind];
  const files = groupByFile(hits);
  sub.innerHTML = `<b>${req.id}</b> — ${hits.length} site${hits.length === 1 ? '' : 's'}` +
                  (files.length > 1 ? ` in ${files.length} files` : '');

  const head = el('div', 'statement-quote');
  head.appendChild(el('span', 'rid', req.id));
  head.appendChild(el('span', null, req.statement));
  body.appendChild(head);

  if (!hits.length) {
    const box = el('div', 'impl-empty');
    const copy = IMPL_COPY[kind];
    box.appendChild(el('h3', null, `No ${copy.what} carries this requirement yet`));
    const tag = el('div', 'tag', `//@req-${req.id}`);
    box.appendChild(tag);
    const p = el('p');
    p.innerHTML = `This pane jumps to the code tagged with that line. Expected in ` +
                  `<code>${copy.where}</code> — ${copy.how}.`;
    box.appendChild(p);
    const ul = el('ul');
    for (const line of [
      `family <code>${req.family}</code>, group <code>${req.group}</code>`,
      `spec: <code>${escapeHtml(req.source.file)}</code>` +
        (req.source.anchor ? ` — anchor <code>${escapeHtml(req.source.anchor)}</code>` : ''),
      `section: ${escapeHtml(req.source.heading || '—')}`,
    ]) {
      const li = el('li'); li.innerHTML = line; ul.appendChild(li);
    }
    box.appendChild(ul);
    body.appendChild(box);
    return;
  }

  for (const entry of files) body.appendChild(implCard(req, kind, entry));
}

/* One card per file, not per tag: a requirement is often tagged at several points in the same
 * module, and repeating near-identical snippets hides that they are one file. */
function groupByFile(hits) {
  const byFile = new Map();
  for (const hit of hits) {
    let entry = byFile.get(hit.file);
    if (!entry) {
      entry = { file: hit.file, origin: hit.origin, hits: [] };
      byFile.set(hit.file, entry);
    }
    entry.hits.push(hit);
  }
  return Array.from(byFile.values());
}

/* Whether new cards open at the whole file or at the snippet. Whole file is the default: the
 * question a pane gets asked is "what does this code do", which the twelve lines around a tag
 * rarely answer on their own. Sticky, because it is a reading habit rather than a per-file
 * choice — someone who prefers the tight snippet prefers it every time. */
const FULL_PREF = 'rv.implFull';
const wantsFull = () => localStorage.getItem(FULL_PREF) !== '0';

// path -> file text. A file stays cached for the session: toggling a card, or coming back to a
// requirement, should not refetch what has not changed under a running server.
const fileCache = new Map();

function implCard(req, kind, entry) {
  const card = el('div', 'impl-file');
  const head = el('div', 'impl-file-head');
  head.appendChild(el('span', 'path', entry.file));
  if (kind === 'rtl' && ORIGIN_LABEL[entry.origin]) {
    head.appendChild(el('span', `badge badge-origin-${entry.origin}`, ORIGIN_LABEL[entry.origin]));
  }

  const view = el('div', 'impl-view');
  const jumps = el('span', 'impl-jumps');
  for (const hit of entry.hits) {
    const link = el('button', 'impl-jump', `:${hit.line}`);
    link.title = `scroll to line ${hit.line}`;
    link.addEventListener('click', () => showFull(true).then(() => scrollToLine(view, hit.line)));
    jumps.appendChild(link);
  }
  head.appendChild(jumps);

  const toggle = el('button', 'impl-toggle');
  head.appendChild(toggle);
  card.appendChild(head);
  card.appendChild(view);

  // `full` is the card's own state; the preference only seeds it.
  let full = null;
  async function showFull(next) {
    if (full === next) return;
    if (next) {
      let text = fileCache.get(entry.file);
      if (text === undefined) {
        toggle.textContent = 'loading…';
        try {
          const res = await fetch(`/api/file?path=${encodeURIComponent(entry.file)}`,
                                  { cache: 'no-store' });
          if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
          text = await res.text();
        } catch (err) {
          toggle.textContent = 'retry';
          view.textContent = '';
          view.appendChild(el('div', 'impl-error', `cannot read ${entry.file} — ${err.message}`));
          full = null;
          return;
        }
        fileCache.set(entry.file, text);
      }
      full = true;
      view.classList.add('impl-view-full');
      renderLines(view, text.split('\n'), 1, req.id, entry.hits.map(h => h.line));
      fitView(view);
      toggle.textContent = 'snippet';
      toggle.title = 'collapse to the tagged lines and their context';
      scrollToLine(view, entry.hits[0].line);
    } else {
      full = false;
      view.classList.remove('impl-view-full');
      view.textContent = '';
      for (const hit of entry.hits) {
        const box = el('div', 'impl-snippet');
        renderLines(box, hit.snippet.split('\n'), hit.line, req.id, [hit.line]);
        view.appendChild(box);
      }
      toggle.textContent = 'whole file';
      toggle.title = `open all of ${entry.file} here, scrolled to the tag`;
    }
  }
  toggle.addEventListener('click', () => {
    const next = !full;
    localStorage.setItem(FULL_PREF, next ? '1' : '0');
    showFull(next);
  });
  showFull(wantsFull());
  return card;
}

const TAG_LINE_RE = /\/\/@req-(spec-[a-z0-9_]+\.[a-z]+[0-9]+)/;

/* Render code as one row per line, numbered from `start`. Rows carrying this requirement's tag
 * are marked; rows tagged for a *different* requirement are marked faintly, which is how a
 * reader sees that a signal answers to more than one line of the spec. */
function renderLines(host, lines, start, reqId, hitLines) {
  host.textContent = '';
  const hits = new Set(hitLines);
  const pre = el('pre', 'code-lines');
  for (let i = 0; i < lines.length; i++) {
    const no = start + i;
    const row = el('span', 'code-line', lines[i] || ' ');
    row.dataset.line = String(no);
    const tagged = TAG_LINE_RE.exec(lines[i]);
    if (hits.has(no) || (tagged && tagged[1] === reqId)) row.classList.add('code-hit');
    else if (tagged) row.classList.add('code-tag-other');
    pre.appendChild(row);
  }
  host.appendChild(pre);
}

/* Cap a file view to what its pane can actually show. A taller view is worse than useless: the
 * pane body clips the overflow, so centring the tag inside the view puts it off-screen — which
 * is what a `55vh` cap does in a half-height right-hand pane. */
function fitView(view) {
  const pane = view.closest('.pane-body');
  if (pane) view.style.maxHeight = `${Math.max(160, pane.clientHeight - 72)}px`;
}

function scrollToLine(view, line) {
  const row = view.querySelector(`.code-line[data-line="${line}"]`);
  if (!row) return;
  // Two scrollers stand between the row and the reader: the pane body, then the view. Drive
  // them in that order, and measure with rects throughout — offsetTop counts from the nearest
  // *positioned* ancestor, not from the scroller, so using it here overshoots by however far
  // down the page the view happens to sit. scrollIntoView is no help either: asked to centre
  // inside nested scrollers it settles on a scrollTop that leaves the row out of view.
  const pane = view.closest('.pane-body');
  if (pane) {
    const vr = view.getBoundingClientRect(), pr = pane.getBoundingClientRect();
    const above = vr.top - pr.top - 4;         // negative: the view starts off the pane's top
    const below = vr.bottom - pr.bottom + 4;   // positive: it runs off the pane's bottom
    if (above < 0) pane.scrollTop += above;
    else if (below > 0) pane.scrollTop += Math.min(above, below);
  }
  const offset = row.getBoundingClientRect().top - view.getBoundingClientRect().top;
  view.scrollTop += offset - Math.max(0, (view.clientHeight - row.offsetHeight) / 2);
  row.classList.add('code-flash');
  setTimeout(() => row.classList.remove('code-flash'), 900);
}

/* ── toolbar, search, sections ──────────────────────────────────────────── */

function fillSections(doc) {
  const sel = $('#section-select');
  sel.textContent = '';
  const top = el('option', null, '— jump to section —');
  top.value = '';
  sel.appendChild(top);
  for (const s of doc.sections) {
    if (!s.heading) continue;
    const opt = el('option', null,
      `${'   '.repeat(Math.max(0, s.level - 1))}${s.number} ${s.heading}`);
    opt.value = s.id;
    sel.appendChild(opt);
  }
}

function wireToolbar() {
  $('#doc-select').addEventListener('change', (ev) => {
    clearPin();
    openDoc(ev.target.value);
  });

  $('#section-select').addEventListener('change', (ev) => {
    if (ev.target.value) scrollToFragment(ev.target.value);
  });

  $('#dim-toggle').checked = localStorage.getItem('rv.dim') === '1';
  $('#dim-toggle').addEventListener('change', (ev) => {
    localStorage.setItem('rv.dim', ev.target.checked ? '1' : '0');
    if (S.frame) S.frame.body.classList.toggle('rv-dim', ev.target.checked);
  });

  $('#clear').addEventListener('click', () => clearPin());

  let timer = null;
  $('#search').addEventListener('input', () => {
    clearTimeout(timer);
    timer = setTimeout(runSearch, 140);
  });

  $('#rebuild').addEventListener('click', async () => {
    const btn = $('#rebuild');
    btn.disabled = true; btn.textContent = 'Rebuilding…';
    try {
      const res = await fetch('/api/rebuild', { method: 'POST' });
      const payload = await res.json();
      if (payload.error) { toast(payload.error); }
      else {
        await loadIndex();
        await openDoc(S.docs.has(S.doc) ? S.doc : (S.index.landing || S.index.docs[0].name));
        clearPin();
        toast(`index rebuilt — ${payload.stats.reqs} reqs, ${payload.stats.tagged} implemented`);
      }
    } catch (err) {
      toast(`rebuild failed: ${err}`);
    } finally {
      btn.disabled = false; btn.textContent = 'Rebuild';
    }
  });
}

function runSearch() {
  const raw = $('#search').value.trim();
  if (!raw) {
    if (S.listMode === 'search') renderReqList([], 'idle');
    return;
  }
  const needle = fold(raw).toLowerCase();
  const hits = [];
  for (const req of S.index.reqs) {
    const hay = `${req.id} ${req.family} ${req.kind} ${req.statement} ` +
                `${req.source.quote_text} ${req.source.heading}`;
    if (fold(hay).toLowerCase().includes(needle)) hits.push(req.id);
    if (hits.length >= 300) break;
  }
  renderReqList(hits, 'search');
}

function wireKeys() {
  document.addEventListener('keydown', (ev) => {
    if (ev.key === 'Escape') {
      if (document.activeElement === $('#search') && $('#search').value) {
        $('#search').value = ''; runSearch(); return;
      }
      clearPin(); return;
    }
    if (ev.target.matches('input, select, textarea')) return;
    if (ev.key === '/') { ev.preventDefault(); $('#search').focus(); return; }
    if (ev.key === 'j' || ev.key === 'k' || ev.key === 'ArrowDown' || ev.key === 'ArrowUp') {
      if (!S.listed.length) return;
      ev.preventDefault();
      const step = (ev.key === 'j' || ev.key === 'ArrowDown') ? 1 : -1;
      const at = S.listed.indexOf(S.selectedReq);
      const next = at < 0 ? (step > 0 ? 0 : S.listed.length - 1)
                          : (at + step + S.listed.length) % S.listed.length;
      selectReq(S.listed[next], true);
      const card = document.querySelector(`.card[data-req="${cssEscape(S.listed[next])}"]`);
      if (card) card.scrollIntoView({ block: 'nearest' });
    }
  });
}

/* ── splitters ──────────────────────────────────────────────────────────── */

const LAYOUT = [
  { handle: '#split-top', axis: 'y', varName: '--top-h', unit: 'px',
    measure: () => $('#reqs-pane').getBoundingClientRect().height, min: 70,
    max: () => window.innerHeight - 220 },
  { handle: '#split-mid', axis: 'x', varName: '--left-w', unit: '%',
    measure: () => $('#spec-pane').getBoundingClientRect().width, min: 240,
    max: () => window.innerWidth - 260, total: () => $('#lower').getBoundingClientRect().width },
  { handle: '#split-right', axis: 'y', varName: '--right-top-h', unit: '%',
    measure: () => $('#nlhdl-pane').getBoundingClientRect().height, min: 60,
    max: () => $('#right').getBoundingClientRect().height - 60,
    total: () => $('#right').getBoundingClientRect().height },
];

function wireSplitters() {
  for (const spec of LAYOUT) {
    const handle = $(spec.handle);
    handle.addEventListener('pointerdown', (ev) => {
      ev.preventDefault();
      handle.setPointerCapture(ev.pointerId);
      $('#shell').classList.add('dragging');
      const origin = spec.axis === 'y' ? ev.clientY : ev.clientX;
      const startPx = spec.measure();

      const move = (mv) => {
        const now = spec.axis === 'y' ? mv.clientY : mv.clientX;
        let px = startPx + (now - origin);
        px = Math.max(spec.min, Math.min(px, spec.max()));
        const value = spec.unit === '%'
          ? `${(px / spec.total() * 100).toFixed(2)}%`
          : `${Math.round(px)}px`;
        document.documentElement.style.setProperty(spec.varName, value);
      };
      const up = () => {
        handle.removeEventListener('pointermove', move);
        handle.removeEventListener('pointerup', up);
        $('#shell').classList.remove('dragging');
        saveLayout();
      };
      handle.addEventListener('pointermove', move);
      handle.addEventListener('pointerup', up);
    });
    handle.addEventListener('dblclick', () => {
      document.documentElement.style.removeProperty(spec.varName);
      saveLayout();
    });
  }
}

function saveLayout() {
  const out = {};
  for (const spec of LAYOUT) {
    const value = document.documentElement.style.getPropertyValue(spec.varName);
    if (value) out[spec.varName] = value;
  }
  localStorage.setItem('rv.layout', JSON.stringify(out));
}

function restoreLayout() {
  try {
    const saved = JSON.parse(localStorage.getItem('rv.layout') || '{}');
    for (const [key, value] of Object.entries(saved)) {
      document.documentElement.style.setProperty(key, value);
    }
  } catch (err) { /* ignore a corrupt entry */ }
}

/* ── odds and ends ──────────────────────────────────────────────────────── */

function scrollToFragment(frag) {
  if (!S.frame || !frag) return;
  const target = S.frame.getElementById(frag) ||
                 S.frame.querySelector(`[id="${cssEscape(frag)}"]`);
  if (!target) { toast(`no anchor #${frag} on this page`); return; }
  target.scrollIntoView({ behavior: 'smooth', block: 'start' });
  const block = target.closest('[data-block]') ||
                target.parentElement.querySelector('[data-block]');
  if (block) flash(block);
}

function flash(node) {
  node.classList.remove('rv-flash');
  void node.offsetWidth;
  node.classList.add('rv-flash');
}

function tagName(tag) {
  return { p: 'paragraph', pre: 'code block', li: 'list item', td: 'table cell',
           th: 'table heading', dt: 'term', dd: 'definition', caption: 'caption',
           figcaption: 'caption' }[tag] ||
         (/^h[1-6]$/.test(tag) ? 'heading' : 'block');
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"]/g, (c) =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
}

function cssEscape(s) {
  return window.CSS && CSS.escape ? CSS.escape(s) : String(s).replace(/[^\w-]/g, '\\$&');
}

let toastTimer = null;
function toast(message) {
  const node = $('#toast');
  node.textContent = message;
  node.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { node.hidden = true; }, 2600);
}

boot().catch((err) => {
  document.body.innerHTML =
    `<pre style="padding:20px;color:#b91c1c;white-space:pre-wrap">${escapeHtml(err.stack || err)}</pre>`;
});

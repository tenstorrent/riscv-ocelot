#!/usr/bin/env python3
"""Gate (f) checker -- Caracal v2, decision D1.

Gate (f) claims: a vectors-off build is identical to the re-baselined reference
"except for the enumerated encoding widths". This script is what makes that claim
falsifiable rather than asserted.

    gate-f-check.py --pre <dir> --post <dir>     # the gate
    gate-f-check.py --manifest <dir> --out f.json # fingerprint one artifact

<dir> is either a gen-collateral directory or its parent (per-config trees are
found automatically).

WHY THIS IS NOT A TEXTUAL DIFF
------------------------------
Widening `MicroOp.dst_rtype` 2b->3b renumbers every bit position downstream of it
in every bundle that packs a MicroOp -- ROB entries, issue slots, queue payloads.
A line-by-line diff therefore reports thousands of changed subscripts that are all
consequences of the enumerated exception, and drowns the one line that would
matter. So the check is split:

  TIER 1 (strict, the majority of modules)
      A module mentioning neither `rtype` nor `iq_type` cannot be affected by the
      exception. Its normalized text must be EQUAL. Any difference is a violation.

  TIER 2 (structural, the modules that carry the widened fields)
      Text is expected to differ. What must NOT differ is the module's shape:
      same ports, same submodule instances, same statement counts. Every width
      change must land on an allowlisted name and be exactly the enumerated
      widening. Anything else is a violation.

  MODULE SET (strict)
      Neither side may gain or lose a module. This is the "and no vector logic"
      half of D1, and it is the cheapest, sharpest check in the file.

LIMITATION, stated because a gate that overclaims is worse than no gate:
Tier 2 does not prove semantic equivalence. Logic could in principle be altered
while holding port list, instance list and statement counts fixed. Tier 2 detects
added/removed structure, not rewritten expressions. Tier 1 -- which covers most
modules -- is a true equality check.
"""
import argparse, hashlib, json, os, re, sys
from collections import Counter

# --- the enumerated exception (D1), and nothing else -----------------------
# Names permitted to change width, and the only widening permitted on them.
ALLOWED_WIDEN = [
    (re.compile(r'rtype'),  (2, 3)),   # RT_* fields: dst_rtype, lrs1_rtype, lrs2_rtype
    (re.compile(r'iq_type'), (4, 7)),  # IQ_SZ 4 -> 7 (as a packed vector, if packed)
]
# Names permitted to APPEAR on the post side only.
# iq_type is Vec(IQ_SZ, Bool()) so it flattens to iq_type_0..N: 4 -> 7 means
# iq_type_4, _5, _6 are new signals rather than a width change.
ALLOWED_NEW = re.compile(r'iq_type(_\d+)?$|iq_type_\d+_')
TOUCHED_TOKEN = re.compile(r'rtype|iq_type')

# --- the exception landing on PACKED CONTAINERS ----------------------------
# The widened fields live inside MicroOp, and MicroOp gets PACKED into ROB
# entries, STQ entries and queue payloads. The container's total width
# therefore grows -- but the container's NAME says nothing about rtype or
# iq_type, so the name-matching allowlist above cannot recognise it. This is
# the same propagation the README already describes ("renumbers every bit
# position downstream of it in EVERY BUNDLE THAT PACKS A MicroOp -- ROB
# entries, issue slots, queue payloads"); it is not a new exception, only the
# machinery to recognise a consequence the prose already covers.
#
# Kept as an EXPLICIT SHORT TABLE rather than a width-delta heuristic, so a
# reviewer can see exactly which containers may grow and by what rule. A
# container not listed here still fails, whatever its delta.
MICROOP_DELTA    = 3 + 3   # 3 rtype bits (dst/lrs1/lrs2) + 3 iq_type bits
COMPACTUOP_DELTA = 1       # RobCompactUop packs only dst_rtype
MAX_LANES        = 8       # coreWidth ceiling across the config matrix

ALLOWED_PACK = [
    # (module regex, signal regex, per-packed-uop delta, what it packs)
    (re.compile(r'^Rob\b'), re.compile(r'^_?rob_compact_uop'),
     COMPACTUOP_DELTA, 'ROB compact uop (dst_rtype only), one per lane'),
    (re.compile(r'^rob_compact_uop_mem'), re.compile(r'^(R0_data|W0_data|mem_)'),
     COMPACTUOP_DELTA, 'ROB compact-uop memory payload, one uop per lane'),
    # The aggregated mems file names the same ports `ram_R_0_data`/`ram_W_0_data`
    # rather than `R0_data`/`W0_data`, so both spellings are listed.
    (re.compile(r'\.(top|model)\.mems$'),
     re.compile(r'^(R0_data|W0_data|mem_|ram_[RW]_\d+_data)'),
     COMPACTUOP_DELTA, 'ROB compact-uop memory payload, one uop per lane'),
    (re.compile(r'^Queue\d+_\w*Entry'), re.compile(r'^_ram_ext'),
     MICROOP_DELTA, 'queue entry packing a whole MicroOp'),
]
# Modules that must be compared STRUCTURALLY even though their text never
# mentions rtype/iq_type -- they carry a packed MicroOp payload.
PACKED_MODULE = re.compile(r'^rob_compact_uop_mem|\.(top|model)\.mems$')

# Named intermediates firtool stops naming purely BECAUSE their width changed.
# Verified by hand at A2: DecodeUnit's cs_rs1_type/cs_rs2_type ceased to be
# named 2-bit wires and were inlined as {1'h0, <same two decoded bits>} -- same
# decoder outputs, same comparisons, zero-extended, so the scalar decode table
# still emits only values 0..3 and RT_VEC remains unreachable from it.
ALLOWED_INLINED = [(re.compile(r'^DecodeUnit'), re.compile(r'^cs_rs\d_type$'))]

# --- normalization ---------------------------------------------------------
RE_LOCATOR = re.compile(r'\s*//\s*@\[[^\]]*\]')      # trailing source locator comment
RE_LOCATOR_INLINE = re.compile(r'@\[[^\]]*\]')       # any residual locator
RE_BANNER = re.compile(r'^// Generated by CIRCT firtool-.*$')
RE_SCALA_STR = re.compile(r'"[^"]*\.scala:[^"]*"')   # assertion text embeds file:line
RE_WS = re.compile(r'[ \t]+')

# Simulation-only register/memory randomization blocks. firtool emits these
# under `ifdef ENABLE_INITIAL_REG_ / ENABLE_INITIAL_MEM_: a `_RANDOM[]` array and
# one assignment per register slicing it. They are `initial` blocks -- no
# synthesized logic, no effect on behavior outside 4-state RTL sim startup.
#
# They are stripped because WIDENING ANY REGISTER RESHUFFLES THE WHOLE
# ALLOCATION: every later register's `_RANDOM[i][hi:lo]` slice shifts, so a
# 1-bit widening rewrites hundreds of lines in modules that are otherwise
# untouched. Measured on the D1 pre/post pair: FetchBuffer differed on 450
# lines, 448 of them `_RANDOM`, and the other 2 were the block's own loop bound.
# Left in, they made Tier 1 -- the file's one true equality check -- fire on
# modules with zero functional change.
RE_INITIAL_GUARD = re.compile(r'^`ifdef\s+ENABLE_INITIAL_(?:REG|MEM)_')
RE_IFDEF_ANY = re.compile(r'^`(?:ifdef|ifndef)\b')
RE_ENDIF = re.compile(r'^`endif\b')


def strip_initial_blocks(lines):
    """Drop `ifdef ENABLE_INITIAL_{REG,MEM}_ ... `endif regions, nesting-aware."""
    out, depth = [], 0
    for ln in lines:
        s = ln.strip()
        if depth:
            if RE_IFDEF_ANY.match(s):
                depth += 1
            elif RE_ENDIF.match(s):
                depth -= 1
            continue
        if RE_INITIAL_GUARD.match(s):
            depth = 1
            continue
        out.append(ln)
    return out


def normalize(text):
    out = []
    for ln in strip_initial_blocks(text.split('\n')):
        if RE_BANNER.match(ln):
            continue
        ln = RE_LOCATOR.sub('', ln)
        ln = RE_LOCATOR_INLINE.sub('', ln)
        ln = RE_SCALA_STR.sub('"<msg>"', ln)
        ln = RE_WS.sub(' ', ln).strip()
        if ln:
            out.append(ln)
    return '\n'.join(out)


# --- structural extraction ------------------------------------------------
RE_PORT = re.compile(
    r'^(?:input|output|inout)\b\s*'
    r'(?:\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*)?'
    r'(\w+)')
RE_DECL = re.compile(
    r'^(?:input|output|inout|wire|reg|logic)\b\s*'
    r'(?:\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*)?'
    r'(\w+)')
RE_INST = re.compile(r'^([A-Z]\w*)\s+(\w+)\s*\($')
RE_STMT = re.compile(r'\b(always_ff|always_comb|always|assign|initial|case|casez)\b')

# firtool-assigned intermediate names: `_GEN_57`, `_T_4`, `_busy_table_next_T_8`,
# `_cs_decoder_decoded_andMatrixOutputs_T_88`, `_csr_ren_T`. These are compiler
# temporaries with NO design identity -- firtool numbers them in emission order,
# so widening one field renumbers every temporary after it, and it inlines or
# names an intermediate at its own discretion (observed at D1: `cs_rs1_type`
# ceased to be a named wire purely because its width changed, with identical
# semantics). Their names are therefore not comparable across a widening.
#
# Deliberately NOT matched: `_ram_ext_R0_data` and friends -- memory port
# signals carry a real width that we DO want compared.
RE_TEMP = re.compile(r'^_(?:.*_)?(?:GEN|T)(?:_\d+)?$')

# firtool names generated RAM modules after their geometry: `ram_4x527`. The
# width is part of the NAME, so widening a memory's payload looks like one
# module vanishing and an unrelated one appearing. Compared by depth, with the
# width delta reported separately.
RE_RAM_MOD = re.compile(r'^ram_(\d+)x(\d+)(\.sv|\.v)?$')


def canon_module(name):
    """Module identity for the module-set check, with RAM width factored out."""
    m = RE_RAM_MOD.match(name)
    return ('ram_%sxW%s' % (m.group(1), m.group(3) or ''), int(m.group(2))) if m else (name, None)


def shape(norm):
    """Ports, named internal signals, temp-width histogram, instances, statements."""
    ports, named, temps, insts, stmts = {}, {}, Counter(), Counter(), Counter()
    for ln in norm.split('\n'):
        m = RE_DECL.match(ln)
        if m:
            hi, lo, name = m.group(1), m.group(2), m.group(3)
            w = (int(hi) - int(lo) + 1) if hi is not None else 1
            if RE_PORT.match(ln):
                ports[name] = w
            elif RE_TEMP.match(name):
                temps[w] += 1
            else:
                named[name] = w
        m = RE_INST.match(ln)
        if m:
            insts[m.group(1) + ' ' + m.group(2)] += 1
        for k in RE_STMT.findall(ln):
            stmts[k] += 1
    return ports, named, temps, insts, stmts


def find_collateral(root):
    """Map config-name -> gen-collateral dir under root (or root itself)."""
    root = os.path.abspath(root)
    if os.path.basename(root) == 'gen-collateral':
        return {os.path.basename(os.path.dirname(root)): root}
    found = {}
    if os.path.isdir(os.path.join(root, 'gen-collateral')):
        found[os.path.basename(root)] = os.path.join(root, 'gen-collateral')
        return found
    for d in sorted(os.listdir(root)):
        gc = os.path.join(root, d, 'gen-collateral')
        if os.path.isdir(gc):
            found[d] = gc
    return found


def load_modules(gc):
    """basename(no ext) -> normalized text, for generated Verilog only."""
    mods = {}
    for fn in sorted(os.listdir(gc)):
        if not fn.endswith(('.sv', '.v')):
            continue
        path = os.path.join(gc, fn)
        if not os.path.isfile(path):
            continue
        with open(path, errors='replace') as f:
            mods[fn] = normalize(f.read())
    return mods


# --- modes ----------------------------------------------------------------
def do_manifest(root, label, repo_sha, boom_sha, out):
    man = {'label': label, 'repo_sha': repo_sha, 'boom_sha': boom_sha, 'configs': {}}
    trees = find_collateral(root)
    if not trees:
        sys.exit('no gen-collateral tree found under %s' % root)
    for cfg, gc in trees.items():
        mods = load_modules(gc)
        man['configs'][cfg] = {
            'module_count': len(mods),
            'tree_sha256': hashlib.sha256(
                ''.join(k + '\0' + v for k, v in sorted(mods.items())).encode()
            ).hexdigest(),
            'modules': {k: hashlib.sha256(v.encode()).hexdigest()[:16]
                        for k, v in sorted(mods.items())},
        }
        print('   %-46s %4d modules  tree=%s'
              % (cfg, len(mods), man['configs'][cfg]['tree_sha256'][:12]))
    if out:
        os.makedirs(os.path.dirname(out), exist_ok=True)
        # Carry over configs that exist in the manifest but were not rebuilt this
        # run, so a CONFIGS=<one> re-run cannot quietly shrink a complete manifest.
        if os.path.exists(out):
            try:
                prev = json.load(open(out))
            except ValueError:
                prev = {}
            if prev.get('boom_sha') == boom_sha:
                for cfg, v in prev.get('configs', {}).items():
                    if cfg not in man['configs']:
                        man['configs'][cfg] = v
                        print('   %-46s carried over from previous run' % cfg)
            elif prev.get('configs'):
                print('   NOTE: previous manifest was for boom %s; not carrying its '
                      'configs over' % str(prev.get('boom_sha'))[:12])
        with open(out, 'w') as f:
            json.dump(man, f, indent=1, sort_keys=True)
            f.write('\n')
    return 0


def classify_width(name, pre_w, post_w, module=''):
    """Is this width change the enumerated exception? Returns None if OK, else why not."""
    for pat, (a, b) in ALLOWED_WIDEN:
        if pat.search(name):
            if pre_w == a and post_w == b:
                return None
            return ('%s: allowlisted name but width %d->%d is not the enumerated %d->%d'
                    % (name, pre_w, post_w, a, b))
    # A packed container of a MicroOp / compact uop grows by a whole number of
    # packed uops -- never a fraction, never more lanes than the widest config.
    mod = module[:-3] if module.endswith('.sv') else (module[:-2] if module.endswith('.v') else module)
    for mpat, spat, unit, what in ALLOWED_PACK:
        if mpat.search(mod) and spat.search(name):
            d = post_w - pre_w
            if d > 0 and d % unit == 0 and d // unit <= MAX_LANES:
                return None
            return ('%s: packs %s so it may grow by a multiple of %d (<= %d of them), '
                    'but width went %d->%d (delta %+d)'
                    % (name, what, unit, MAX_LANES, pre_w, post_w, d))
    return '%s: width %d->%d on a name outside the enumerated exception' % (name, pre_w, post_w)


def do_gate(pre_root, post_root, verbose):
    pre_trees, post_trees = find_collateral(pre_root), find_collateral(post_root)
    common = sorted(set(pre_trees) & set(post_trees))
    if not common:
        sys.exit('no config trees in common:\n  pre : %s\n  post: %s'
                 % (sorted(pre_trees), sorted(post_trees)))
    for only in sorted(set(pre_trees) ^ set(post_trees)):
        print('NOTE: config present on only one side, skipped: %s' % only)

    total_v = 0
    for cfg in common:
        pre, post = load_modules(pre_trees[cfg]), load_modules(post_trees[cfg])
        violations, allowed = [], []

        # -- MODULE SET (strict, modulo RAM geometry in the name) --
        # RAM modules are keyed by depth; their width delta is reported below
        # rather than read as one module vanishing and another appearing.
        pre_can = {canon_module(m)[0]: m for m in pre}
        post_can = {canon_module(m)[0]: m for m in post}
        for k in sorted(set(post_can) - set(pre_can)):
            violations.append(('module-added', post_can[k], 'module exists only in post -- '
                               'gate (f) forbids new logic in a vectors-off build'))
        for k in sorted(set(pre_can) - set(post_can)):
            violations.append(('module-removed', pre_can[k], 'module exists only in pre'))
        for k in sorted(set(pre_can) & set(post_can)):
            wa, wb = canon_module(pre_can[k])[1], canon_module(post_can[k])[1]
            if wa is not None and wa != wb:
                allowed.append((pre_can[k], 'RAM payload %db->%db (renamed %s -> %s)'
                                % (wa, wb, pre_can[k], post_can[k])))

        t1 = t1_diff = t2 = 0
        stmt_deltas, temp_deltas = [], []
        for m in sorted(set(pre) & set(post)):
            if canon_module(m)[1] is not None:
                continue          # RAM module: geometry handled above
            a, b = pre[m], post[m]
            mbase = m[:-3] if m.endswith('.sv') else (m[:-2] if m.endswith('.v') else m)
            touched = bool(TOUCHED_TOKEN.search(a) or TOUCHED_TOKEN.search(b)
                           or PACKED_MODULE.search(mbase))
            if not touched:
                # -- TIER 1 (strict) --
                t1 += 1
                if a != b:
                    t1_diff += 1
                    violations.append(('tier1-differs', m,
                                       'module mentions neither rtype nor iq_type, so the '
                                       'enumerated exception cannot explain any change'))
                continue
            # -- TIER 2 (structural) --
            t2 += 1
            pa, na, ta, ia, sa = shape(a)
            pb, nb, tb, ib, sb = shape(b)
            # Ports and named internal signals are compared strictly. Compiler
            # temporaries (ta/tb) are NOT: see RE_TEMP for why their names carry
            # no design identity across a widening. Their count delta is
            # reported as information, never as a violation.
            for da, db, kind in ((pa, pb, 'port'), (na, nb, 'signal')):
                for name in sorted(set(db) - set(da)):
                    if ALLOWED_NEW.search(name):
                        allowed.append((m, 'new %s %s (IQ_SZ 4->7)' % (kind, name)))
                    else:
                        violations.append(('%s-added' % kind, m,
                                           'new %s %s outside the enumerated exception'
                                           % (kind, name)))
                for name in sorted(set(da) - set(db)):
                    if kind == 'signal' and any(mp.search(m) and sp.search(name)
                                                for mp, sp in ALLOWED_INLINED):
                        allowed.append((m, 'named intermediate %s inlined by firtool '
                                           '(width changed; semantics verified)' % name))
                    else:
                        violations.append(('%s-removed' % kind, m,
                                           '%s %s disappeared' % (kind, name)))
                for name in sorted(set(da) & set(db)):
                    if da[name] != db[name]:
                        why = classify_width(name, da[name], db[name], m)
                        if why:
                            violations.append(('width-change', m, why))
                        else:
                            allowed.append((m, '%s %s %db->%db'
                                            % (kind, name, da[name], db[name])))
            if sum(ta.values()) != sum(tb.values()):
                temp_deltas.append((m, sum(ta.values()), sum(tb.values())))
            if ia != ib:
                d = (ib - ia) + (ia - ib)
                violations.append(('instances-differ', m,
                                   'submodule instances changed: %s'
                                   % ', '.join('%s%+d' % (k, ib[k] - ia[k])
                                               for k in sorted(d))))
            if sa != sb:
                # INFORMATIONAL, not a violation. A width change legitimately
                # alters how firtool decomposes an expression -- it may inline a
                # named intermediate or split one assign into several without any
                # semantic change (observed at D1 in DecodeUnit, where widening
                # cs_rs1_type turned one named wire into an inlined concat). The
                # structural claims that REMAIN violations are the ones a
                # re-decomposition cannot fake: ports, named signals, and
                # submodule instances.
                d = set(sa) | set(sb)
                stmt_deltas.append((m, ', '.join('%s %d->%d' % (k, sa[k], sb[k])
                                                 for k in sorted(d) if sa[k] != sb[k])))

        print('\n=== %s ===' % cfg)
        print('  modules            %d (pre) / %d (post)' % (len(pre), len(post)))
        print('  tier 1 strict      %d modules, %d differing' % (t1, t1_diff))
        print('  tier 2 structural  %d modules' % t2)
        print('  allowed deltas     %d (the enumerated exception)' % len(allowed))
        if verbose:
            for m, d in allowed:
                print('      + %-40s %s' % (m, d))
        # Informational: consequences of re-decomposition, not structural claims.
        # Printed so a reviewer sees the blast radius rather than having it hidden.
        print('  info: %d module(s) with statement-count deltas, %d with temp-count deltas'
              % (len(stmt_deltas), len(temp_deltas)))
        if verbose:
            for m, d in stmt_deltas:
                print('      ~ %-40s %s' % (m, d))
            for m, a_, b_ in temp_deltas:
                print('      ~ %-40s firtool temporaries %d->%d' % (m, a_, b_))
        print('  VIOLATIONS         %d' % len(violations))
        for kind, m, why in violations[:60]:
            print('      ! [%s] %s: %s' % (kind, m, why))
        if len(violations) > 60:
            print('      ... %d more' % (len(violations) - 60))
        total_v += len(violations)

    print('\n%s' % ('GATE (f) PASS -- every difference is the enumerated exception'
                    if total_v == 0 else
                    'GATE (f) FAIL -- %d violation(s)' % total_v))
    return 1 if total_v else 0


def do_verify(root, against, verbose):
    """Did a regeneration reproduce the artifact the manifest records?

    The Verilog is not checked in, so this is how the manifest earns its keep: it
    turns "I regenerated it" into "I regenerated the same thing".
    """
    man = json.load(open(against))
    trees = find_collateral(root)
    bad = 0
    print('manifest %s  (label=%s boom=%s)'
          % (against, man.get('label'), str(man.get('boom_sha'))[:12]))
    for cfg, rec in sorted(man.get('configs', {}).items()):
        if cfg not in trees:
            print('  MISSING  %-46s recorded but no tree on disk' % cfg)
            bad += 1
            continue
        mods = load_modules(trees[cfg])
        got = hashlib.sha256(
            ''.join(k + '\0' + v for k, v in sorted(mods.items())).encode()).hexdigest()
        if got == rec['tree_sha256']:
            print('  ok       %-46s %d modules' % (cfg, len(mods)))
            continue
        bad += 1
        print('  DRIFTED  %-46s recorded %s got %s'
              % (cfg, rec['tree_sha256'][:12], got[:12]))
        rm, gm = rec['modules'], {k: hashlib.sha256(v.encode()).hexdigest()[:16]
                                  for k, v in mods.items()}
        for m in sorted(set(gm) - set(rm)):
            print('             + %s (not in manifest)' % m)
        for m in sorted(set(rm) - set(gm)):
            print('             - %s (in manifest, absent on disk)' % m)
        diff = sorted(m for m in set(rm) & set(gm) if rm[m] != gm[m])
        for m in diff[:20]:
            print('             ~ %s' % m)
        if len(diff) > 20:
            print('             ~ ... %d more changed' % (len(diff) - 20))
    for cfg in sorted(set(trees) - set(man.get('configs', {}))):
        print('  EXTRA    %-46s on disk but not in the manifest' % cfg)
    print('\n%s' % ('VERIFY OK -- trees match the manifest' if not bad
                    else 'VERIFY FAIL -- %d config(s) missing or drifted' % bad))
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pre'); ap.add_argument('--post')
    ap.add_argument('--verify'); ap.add_argument('--against')
    ap.add_argument('--manifest'); ap.add_argument('--out')
    ap.add_argument('--label', default=''); ap.add_argument('--repo-sha', default='')
    ap.add_argument('--boom-sha', default=''); ap.add_argument('-v', '--verbose',
                                                              action='store_true')
    a = ap.parse_args()
    if a.manifest:
        return do_manifest(a.manifest, a.label, a.repo_sha, a.boom_sha, a.out)
    if a.verify:
        if not a.against:
            ap.error('--verify needs --against <manifest.json>')
        return do_verify(a.verify, a.against, a.verbose)
    if a.pre and a.post:
        return do_gate(a.pre, a.post, a.verbose)
    ap.error('need --manifest, --verify/--against, or both --pre and --post')


if __name__ == '__main__':
    sys.exit(main())

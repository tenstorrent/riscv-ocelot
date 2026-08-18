#!/usr/bin/env bash
# Regenerate a gate (f) reference artifact.
#
# Gate (f) (plan v2 §6, as relaxed by decision D1) claims a vectors-off build is
# "identical to the re-baselined reference except for the enumerated encoding
# widths". This script produces one side of that comparison.
#
#   ./regen.sh prebaseline     # boom WITHOUT the D1 encoding widening
#   ./regen.sh rebaseline      # boom WITH the D1 encoding widening, still no vector logic
#   ./regen.sh check           # a FRESH vectors-off build, verified against
#                              # manifest/rebaseline.json. This is the mode every
#                              # phase gate after A2 runs. It writes NO manifest --
#                              # re-running `rebaseline` to get a fresh tree would
#                              # overwrite the very reference the gate judges
#                              # against, turning a failure into a silent
#                              # re-baselining. Non-destructive by construction.
#
# The two artifacts must be generated with the SAME chipyard-side config content,
# or the diff between them mixes the encoding delta with a config delta and gate
# (f) stops meaning anything. That is what the neutralization step below enforces:
# the gate-f config set is the PLAIN BOOM v4 configs with Caracal's chipyard-side
# mixins removed. See README.md.
#
# Run from the host, in the repo. Requires a running podman container with the
# chipyard conda env (see CLAUDE.md §1).
set -euo pipefail

WHICH="${1:-}"
case "$WHICH" in
  prebaseline|rebaseline|check) ;;
  *) echo "usage: $0 {prebaseline|rebaseline|check}" >&2; exit 2 ;;
esac

CONTAINER="${CONTAINER:-reverent_turing}"
CONFIGS="${CONFIGS:-SmallBoomV4Config MediumBoomV4Config MegaBoomV4Config}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"          # docs_caracal/v2-rebaseline -> boom -> generators -> repo
BOOMCFG="$REPO/generators/chipyard/src/main/scala/config/BoomConfigs.scala"
GENNAME="generated-src-gatef-$WHICH"
OUTDIR="$REPO/sims/vcs/$GENNAME"

[ -f "$BOOMCFG" ] || { echo "not found: $BOOMCFG" >&2; exit 1; }

# ---------------------------------------------------------------------------
# 1. Record what we are generating from. The manifest is worthless without this.
# ---------------------------------------------------------------------------
BOOM_SHA=$(cd "$REPO/generators/boom" && git rev-parse HEAD)
BOOM_DIRTY=$(cd "$REPO/generators/boom" && \
             git status --porcelain src/main/scala src/main/resources | wc -l)
REPO_SHA=$(cd "$REPO" && git rev-parse HEAD)
echo "== gate-f $WHICH =="
echo "   repo      $REPO_SHA"
echo "   boom      $BOOM_SHA  (scala/resources dirty files: $BOOM_DIRTY)"
echo "   configs   $CONFIGS"
echo "   out       sims/vcs/$GENNAME"
if [ "$BOOM_DIRTY" -ne 0 ]; then
  echo "   WARNING: boom src/main/scala is dirty; this artifact is not reproducible from a SHA." >&2
fi

# ---------------------------------------------------------------------------
# 2. Neutralize Caracal's chipyard-side additions.
#
# Two mechanical rules, and deliberately no more:
#   (a) drop every `WithBoomDebugHarness` mixin line  -- whisper-cosim DPI
#       scaffolding, mixed into the plain V4 configs too, and absent from a
#       pre-vector boom so the file will not even compile with it present;
#   (b) drop every `class *Vector*Config` block       -- references
#       boom.v4.vec.common.VectorParams, which a vectors-off tree lacks.
# Both are restored by the trap below, unconditionally.
# ---------------------------------------------------------------------------
# Serialize: this script mutates a TRACKED source file and restores it on exit. Two
# overlapping runs and the second backs up the first's already-neutralized file, then
# "restores" that -- silently leaving the repo uncompilable. Learned the hard way.
LOCK="$HERE/.regen.lock"
exec 9>"$LOCK"
if ! flock -n 9; then
  echo "another regen.sh holds $LOCK -- refusing to run concurrently" >&2
  echo "(it edits generators/chipyard/.../BoomConfigs.scala in place)" >&2
  exit 1
fi

# Belt and braces: the backup must be the PRISTINE file. If it is already neutralized,
# a previous run died without restoring, and backing it up would make that permanent.
if ! grep -q 'WithVector' "$BOOMCFG"; then
  echo "ERROR: $BOOMCFG appears already neutralized (no WithVector)." >&2
  echo "A previous run left it modified. Restore it before continuing:" >&2
  echo "  git -C '$REPO' checkout generators/chipyard/src/main/scala/config/BoomConfigs.scala" >&2
  exit 1
fi

BACKUP="$(mktemp "${TMPDIR:-/tmp}/BoomConfigs.scala.orig.XXXXXX")"
cp -p "$BOOMCFG" "$BACKUP"

# ---------------------------------------------------------------------------
# RESTORING THE FILE IS NOT ENOUGH -- THE CLASSPATH CACHE MUST BE INVALIDATED TOO.
#
# chipyard elaborates from a cached assembly jar ($base_dir/.classpath_cache/
# chipyard.jar, variables.mk:189-191) whose make rule depends on
# CHIPYARD_SCALA_SOURCES (common.mk:136). While this script has BoomConfigs.scala
# neutralized, any elaboration it triggers REASSEMBLES that jar WITHOUT the vector
# configs -- and `cp -p` then restores the source with its ORIGINAL mtime, which is
# OLDER than the poisoned jar. make therefore sees the cache as up to date, and the
# next vector build dies with:
#     java.lang.ClassNotFoundException: chipyard.MegaBoomV4VectorConfig
# hours later, with nothing in the working tree to explain it (`git status` is
# clean, and the file plainly contains the class).
#
# `cp -p` is deliberate -- a fresh mtime would force a full rebuild of everything
# downstream on every gate-(f) run -- so the fix is to delete the one artifact this
# script actually invalidated rather than to touch the source. Removing the jar is
# also the honest statement: it was assembled from a mutated source and is not
# trustworthy, whatever its timestamp says.
CPCACHE_JAR="$REPO/.classpath_cache/chipyard.jar"
restore() {
  cp -p "$BACKUP" "$BOOMCFG"
  rm -f "$BACKUP"
  echo "   (BoomConfigs.scala restored)"
  if [ -f "$CPCACHE_JAR" ]; then
    rm -f "$CPCACHE_JAR"
    echo "   (dropped .classpath_cache/chipyard.jar -- it was assembled with the"
    echo "    vector configs neutralized; cp -p restore leaves make unable to see that)"
  fi
}
trap restore EXIT

python3 - "$BOOMCFG" <<'PY'
import re, sys
p = sys.argv[1]
lines = open(p).read().split('\n')
out, i, dropped_cls, dropped_dbg = [], 0, [], 0
while i < len(lines):
    ln = lines[i]
    m = re.match(r'^class (\w*Vector\w*Config) extends Config\($', ln)
    if m:
        # (b) swallow the whole Config( ... ) block: to and including the line
        # whose parens balance back to zero.
        depth = 0
        while i < len(lines):
            depth += lines[i].count('(') - lines[i].count(')')
            i += 1
            if depth <= 0:
                break
        dropped_cls.append(m.group(1))
        continue
    if 'WithBoomDebugHarness' in ln and ln.strip().startswith('new '):
        dropped_dbg += 1          # (a)
        i += 1
        continue
    out.append(ln)
    i += 1
open(p, 'w').write('\n'.join(out))
sys.stderr.write("   neutralized: %d WithBoomDebugHarness mixin(s), %d vector config(s): %s\n"
                 % (dropped_dbg, len(dropped_cls), ' '.join(dropped_cls) or '-'))
PY

# Residual check on CODE only -- the file's header comment legitimately names
# WithBoomDebugHarness in prose.
RESIDUAL=$(grep -vn '^\s*//' "$BOOMCFG" \
           | grep 'WithVector\|boom\.v4\.vec\|WithBoomDebugHarness' || true)
if [ -n "$RESIDUAL" ]; then
  echo "ERROR: neutralization incomplete, residual references remain:" >&2
  echo "$RESIDUAL" >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# 3. Elaborate. `verilog` stops after Chisel + firtool -- no simulator build.
#    `generated_src_name` keeps this out of the normal generated-src/ tree.
# ---------------------------------------------------------------------------
# Remove only the trees we are about to rebuild. Removing all of $OUTDIR would mean
# a CONFIGS=<one> re-run silently deletes the other configs and then writes a
# partial manifest over the complete one.
for C in $CONFIGS; do
  rm -rf "$OUTDIR/chipyard.harness.TestHarness.$C"
done
for C in $CONFIGS; do
  echo "-- elaborate $C"
  podman exec "$CONTAINER" bash -lc "set -o pipefail
    source /opt/conda/etc/profile.d/conda.sh && source /root/my-chipyard/env.sh
    cd /root/my-chipyard/sims/vcs
    make CONFIG=$C generated_src_name=$GENNAME -j\$(nproc) verilog 2>&1 | tail -20"
done

# ---------------------------------------------------------------------------
# 4. Fingerprint. This is what gets checked in; the Verilog itself does not
#    (620+ files x 3 configs). gate-f-check.py compares two trees directly when
#    both are present, and falls back to the manifest when only one is.
# ---------------------------------------------------------------------------
if [ "$WHICH" = check ]; then
  # Judge the fresh tree against the checked-in reference. Deliberately writes no
  # manifest: the reference is an input here, never an output.
  python3 "$HERE/gate-f-check.py" --verify "$OUTDIR" \
          --against "$HERE/manifest/rebaseline.json"
  echo "== done: gate (f) check against manifest/rebaseline.json"
else
  python3 "$HERE/gate-f-check.py" --manifest "$OUTDIR" \
          --label "$WHICH" --repo-sha "$REPO_SHA" --boom-sha "$BOOM_SHA" \
          --out "$HERE/manifest/$WHICH.json"
  echo "== done: $HERE/manifest/$WHICH.json"
fi

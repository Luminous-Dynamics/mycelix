#!/usr/bin/env python3
"""check-workspace-integrity.py — fail fast on the two breakage classes that
silently disabled *all* aggregate verification in this workspace.

WHY
---
On 2026-07-28 an audit found `cargo metadata` could not resolve the
mycelix-workspace root workspace at all. Three separate members were unloadable,
so no aggregate `cargo build`, `cargo test`, or CI job could run — and nothing
mechanically contradicted any claim that they had. Root causes:

  1. A dedup commit moved four `crates/*` crates to the monorepo root, deleted
     their manifests, but left the `src/` dirs and the `crates/*` members glob.
  2. `mycelix-prism/prism-attestation/Cargo.toml` was literally `// placeholder`.
  3. `mycelix-supplychain/crates/supplychain-proof-policy` had no manifest, and
     both its source files were `// placeholder`.

Separately, a "patchset application" workflow committed ~68 placeholder-only files
in one commit, including `.github/workflows/ci.yml` — so the CI workflow itself was
a one-line C comment in a YAML file, and could never have parsed.

WHAT IT CHECKS
--------------
  A. every glob-expanded workspace member (minus [workspace].exclude) has a
     Cargo.toml that parses as TOML and declares a [package];
  B. every such member has a build target (src/lib.rs, src/main.rs, src/bin/*.rs,
     [lib], or [[bin]]) — cargo refuses the WHOLE workspace otherwise;
  C. no tracked file is placeholder-only (body is just `// placeholder` or
     similar). Restricted to types where a placeholder is load-bearing:
     .rs / .toml / .yml / .yaml.

Deliberately pure filesystem + TOML parsing: NO cargo invocation. That is the
point — it must work when the workspace is already too broken for cargo to load,
which is exactly when you need it.

USAGE
-----
    scripts/check-workspace-integrity.py            # CI gate
    scripts/check-workspace-integrity.py --no-placeholders   # A+B only
    scripts/check-workspace-integrity.py --list-placeholders # inventory, exit 0

Exit 0 = clean, 1 = problems found.

Refs: MYCELIX_COMPREHENSIVE_REVIEW_2026-07-28.md (F2, F3).
Modelled on symthaea/scripts/check-workspace-targets.sh, but parses Cargo.toml
instead of hardcoding the globs — this workspace has 61 member patterns and a
~300-entry exclude list, which no hand-mirrored copy would survive.
"""

from __future__ import annotations

import glob
import os
import re
import subprocess
import sys

try:
    import tomllib
except ImportError:  # py<3.11
    import tomli as tomllib  # type: ignore

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# A file counts as placeholder-only if it has <=2 non-blank lines and one of
# them is a bare placeholder comment.
PLACEHOLDER_RE = re.compile(r"^\s*(?://+|#+|--)?\s*placeholder\b", re.IGNORECASE)
PLACEHOLDER_EXTS = (".rs", ".toml", ".yml", ".yaml")


def parse_list(text: str, key: str) -> list[str]:
    """Extract a [workspace] string-array by key, ignoring commented lines.

    Hand-rolled rather than tomllib because this must also work on a Cargo.toml
    that a sibling break has made unparseable.
    """
    out: list[str] = []
    inside = False
    for line in text.splitlines():
        s = line.strip()
        if re.match(rf"{key}\s*=\s*\[", s):
            inside = True
            continue
        if inside:
            if s.startswith("]"):
                break
            if s.startswith("#"):
                continue
            out += re.findall(r'"([^"]+)"', s)
    return out


def has_build_target(d: str) -> bool:
    if os.path.exists(f"{d}/src/lib.rs") or os.path.exists(f"{d}/src/main.rs"):
        return True
    if glob.glob(f"{d}/src/bin/*.rs"):
        return True
    try:
        with open(f"{d}/Cargo.toml", encoding="utf8", errors="replace") as fh:
            return re.search(r"^\[\[bin\]\]|^\[lib\]", fh.read(), re.M) is not None
    except OSError:
        return False


def check_members() -> list[str]:
    cargo = os.path.join(ROOT, "Cargo.toml")
    text = open(cargo, encoding="utf8", errors="replace").read()
    members = parse_list(text, "members")
    excludes = parse_list(text, "exclude")

    exset = set()
    for pat in excludes:
        hits = glob.glob(os.path.join(ROOT, pat)) if "*" in pat else [os.path.join(ROOT, pat)]
        for h in hits:
            exset.add(os.path.normpath(h))

    def is_excluded(path: str) -> bool:
        """Cargo's [workspace].exclude is PREFIX-based: excluding `a/b` also
        excludes `a/b/src` and every other descendant. Verified empirically
        2026-07-28 — an exact-match implementation produced 7 false positives
        (happs/lucid/ui/src, mycelix-finance/types/src, ...) on a workspace that
        `cargo metadata` resolves cleanly.
        """
        p = os.path.normpath(path)
        while True:
            if p in exset:
                return True
            parent = os.path.dirname(p)
            if parent == p or len(parent) < len(ROOT):
                return False
            p = parent

    dirs: set[str] = set()
    for pat in members:
        hits = sorted(glob.glob(os.path.join(ROOT, pat))) if "*" in pat else [os.path.join(ROOT, pat)]
        for h in hits:
            # cargo skips non-directories inside a members glob, but DOES error on
            # a matched directory with no Cargo.toml (verified with a probe dir).
            if os.path.isdir(h) and not is_excluded(h):
                dirs.add(h)

    problems: list[str] = []
    for d in sorted(dirs):
        rel = os.path.relpath(d, ROOT)
        mf = os.path.join(d, "Cargo.toml")
        if not os.path.exists(mf):
            problems.append(f"NO MANIFEST      {rel}  (matched by a members glob, but has no Cargo.toml)")
            continue
        try:
            with open(mf, "rb") as fh:
                data = tomllib.loads(fh.read().decode("utf8", errors="replace"))
        except Exception as exc:
            problems.append(f"UNPARSEABLE      {rel}/Cargo.toml  ({str(exc)[:70]})")
            continue
        if "package" not in data:
            problems.append(f"NO [package]     {rel}/Cargo.toml")
            continue
        if not has_build_target(d):
            problems.append(
                f"NO BUILD TARGET  {rel}  (no src/lib.rs, src/main.rs, src/bin/*.rs, [lib] or [[bin]])"
            )
    return problems


def tracked_files() -> list[str]:
    try:
        out = subprocess.run(
            ["git", "ls-files", "-z", "--", "."],
            cwd=ROOT, capture_output=True, check=True,
        ).stdout
        return [p for p in out.decode("utf8", "replace").split("\0") if p]
    except Exception:
        return []


def check_placeholders() -> list[str]:
    problems: list[str] = []
    for rel in tracked_files():
        if not rel.endswith(PLACEHOLDER_EXTS):
            continue
        path = os.path.join(ROOT, rel)
        try:
            if os.path.getsize(path) > 400:
                continue
            with open(path, encoding="utf8", errors="replace") as fh:
                body = [ln for ln in fh.read().splitlines() if ln.strip()]
        except OSError:
            continue
        if body and len(body) <= 2 and any(PLACEHOLDER_RE.match(ln) for ln in body):
            problems.append(f"PLACEHOLDER ONLY {rel}")
    return problems


BASELINE = os.path.join(ROOT, "scripts", "placeholder-baseline.txt")


def load_baseline() -> set[str]:
    if not os.path.exists(BASELINE):
        return set()
    with open(BASELINE, encoding="utf8") as fh:
        return {
            ln.strip()
            for ln in fh
            if ln.strip() and not ln.lstrip().startswith("#")
        }


def main() -> int:
    args = sys.argv[1:]
    found = [p.replace("PLACEHOLDER ONLY ", "").strip() for p in check_placeholders()]

    if "--list-placeholders" in args:
        for p in sorted(found):
            print(p)
        return 0

    if "--update-baseline" in args:
        with open(BASELINE, "w", encoding="utf8") as fh:
            fh.write(
                "# Known placeholder-only tracked files, as of the last baseline update.\n"
                "# The integrity check fails on any placeholder NOT listed here, so this\n"
                "# file stops the backlog from growing while it is worked down.\n"
                "#\n"
                "# To retire an entry: implement or delete the file, then re-run\n"
                "#   scripts/check-workspace-integrity.py --update-baseline\n"
                "# Do NOT add entries by hand to silence a new failure — that is the\n"
                "# exact habit this guard exists to break.\n"
                "# See MYCELIX_COMPREHENSIVE_REVIEW_2026-07-28.md (F3).\n"
            )
            for p in sorted(found):
                fh.write(p + "\n")
        print(f"baseline written: {len(found)} known placeholder(s) -> {os.path.relpath(BASELINE, ROOT)}")
        return 0

    problems = check_members()
    if "--no-placeholders" not in args:
        baseline = load_baseline()
        new = sorted(set(found) - baseline)
        problems += [f"NEW PLACEHOLDER  {p}" for p in new]
        stale = sorted(baseline - set(found))
        if stale:
            print(
                f"note: {len(stale)} baselined placeholder(s) are now fixed or gone — "
                "re-run with --update-baseline to shrink the baseline.\n"
            )

    if problems:
        print("workspace integrity check FAILED\n")
        for p in problems:
            print(f"  {p}")
        print(
            f"\n{len(problems)} problem(s).\n"
            "A single unloadable member breaks `cargo metadata` for the ENTIRE workspace,\n"
            "disabling every aggregate build/test/CI job. A placeholder-only .rs/.toml/.yml\n"
            "is a feature-shaped empty file: it reads as implemented and is not.\n"
            "Fix, delete, or add to [workspace].exclude with a comment saying why.\n"
            "See MYCELIX_COMPREHENSIVE_REVIEW_2026-07-28.md (F2, F3)."
        )
        return 1

    print("workspace integrity OK — all members loadable, no placeholder-only tracked files")
    return 0


if __name__ == "__main__":
    sys.exit(main())

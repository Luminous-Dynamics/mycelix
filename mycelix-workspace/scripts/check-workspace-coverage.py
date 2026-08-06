#!/usr/bin/env python3
"""check-workspace-coverage.py — keep the CI verification gap visible.

WHY
---
The root workspace resolves to ~57 packages, but 28 clusters carry their OWN
`[workspace]`. Anything inside those is invisible to root-level `cargo metadata`,
`cargo check` and `cargo test`. So a green root badge says nothing about most of
the tree — which is exactly how a broken root workspace, a placeholder `ci.yml`
and eight drifting fork clusters all survived unnoticed for months.

This does NOT try to build everything (most clusters are unverified and adding
known-red legs would destroy the signal). It makes the gap explicit and stops it
growing silently:

  * every nested workspace must be declared in `workspace-inventory.txt` with a
    CLASSIFICATION — a NEW undeclared one fails the check;
  * the declared CI-covered set is cross-checked against ci.yml, so a cluster
    cannot quietly claim coverage it does not have;
  * the number of UNVERIFIED CANONICAL workspaces may never increase.

Classifications:

  canonical    Ships, or is intended to. MUST reach CI. Counted against the
               high-water mark below, so this number can only go down.
  experimental Has an owner and limited guarantees. Not required to be green.
  archived     Explicitly non-shipping. Excluded from active verification.
  retire       Tracked for deletion.

WHY A HIGH-WATER MARK: declaring the gap is not the same as closing it. Without a
ratchet, "27 known-unverified" quietly becomes the new normal — the same way 225
allowlisted validators or 68 placeholder files did. The mark makes the backlog
monotonically shrink or the build fails.

USAGE
    scripts/check-workspace-coverage.py              # CI gate
    scripts/check-workspace-coverage.py --update     # re-seed the inventory
"""

from __future__ import annotations

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INVENTORY = os.path.join(ROOT, "scripts", "workspace-inventory.txt")
CI = os.path.join(ROOT, ".github", "workflows", "ci.yml")


def nested_workspaces() -> list[str]:
    found = []
    for name in sorted(os.listdir(ROOT)):
        path = os.path.join(ROOT, name)
        manifest = os.path.join(path, "Cargo.toml")
        if not (os.path.isdir(path) and os.path.exists(manifest)):
            continue
        try:
            with open(manifest, encoding="utf8", errors="replace") as fh:
                if re.search(r"^\[workspace\]", fh.read(), re.M):
                    found.append(name)
        except OSError:
            continue
    return found


def ci_covered() -> set[str]:
    if not os.path.exists(CI):
        return set()
    with open(CI, encoding="utf8") as fh:
        text = fh.read()
    # cluster matrix entries plus any explicit working-directory
    return set(re.findall(r"^\s*-\s+(mycelix-[a-z-]+)\s*$", text, re.M)) | set(
        re.findall(r"working-directory:\s*(mycelix-[a-z-]+)", text)
    )


HIGH_WATER = os.path.join(ROOT, "scripts", "workspace-unverified-high-water.txt")
VALID = {"ci", "canonical", "experimental", "archived", "retire", "unverified"}
# `unverified` is accepted as a legacy synonym for `canonical` (pre-classification
# seeding) so the ratchet works before every cluster has been triaged.
COUNTS_AGAINST_MARK = {"canonical", "unverified"}


def read_high_water() -> int | None:
    try:
        with open(HIGH_WATER, encoding="utf8") as fh:
            for line in fh:
                line = line.split("#", 1)[0].strip()
                if line:
                    return int(line)
    except (OSError, ValueError):
        return None
    return None


def load_inventory() -> dict[str, str]:
    out: dict[str, str] = {}
    if not os.path.exists(INVENTORY):
        return out
    with open(INVENTORY, encoding="utf8") as fh:
        for raw in fh:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 2:
                out[parts[0]] = parts[1]
    return out


def main() -> int:
    found = nested_workspaces()
    covered = ci_covered()

    if "--update" in sys.argv:
        prev = load_inventory()
        with open(INVENTORY, "w", encoding="utf8") as fh:
            fh.write(
                "# Nested workspaces (clusters with their own [workspace]).\n"
                "# Format:  <cluster>  <ci|unverified>\n"
                "#\n"
                "# Root-level cargo commands CANNOT see inside these, so a green root\n"
                "# workspace proves nothing about them. `unverified` is the backlog:\n"
                "# it means nothing in CI builds or tests that cluster at all.\n"
                "#\n"
                "# A NEW nested workspace not listed here fails the check — nested\n"
                "# workspaces fragment verification and should be a deliberate choice.\n"
                "# See MYCELIX_COMPREHENSIVE_REVIEW_2026-07-28.md (F2).\n"
            )
            for c in found:
                fh.write(f"{c}  {'ci' if c in covered else prev.get(c, 'unverified')}\n")
        print(f"inventory written: {len(found)} nested workspace(s)")
        return 0

    inv = load_inventory()
    problems = []

    for c in found:
        if c not in inv:
            problems.append(f"UNDECLARED nested workspace: {c}")
        elif inv[c] == "ci" and c not in covered:
            problems.append(f"CLAIMS CI COVERAGE but absent from ci.yml: {c}")

    for c in inv:
        if c not in found:
            problems.append(f"inventory lists a cluster that no longer has its own workspace: {c}")
        if inv[c] not in VALID:
            problems.append(f"unknown classification '{inv[c]}' for {c} (expected one of {sorted(VALID)})")

    unverified = sorted(c for c in found if inv.get(c) != "ci" and c not in covered)
    unverified_canonical = sorted(
        c for c in unverified if inv.get(c, "canonical") in COUNTS_AGAINST_MARK
    )

    mark = read_high_water()
    if mark is not None and len(unverified_canonical) > mark:
        problems.append(
            f"UNVERIFIED CANONICAL workspaces increased: {len(unverified_canonical)} > "
            f"high-water mark {mark}. Either add the cluster to CI, or reclassify it "
            f"(experimental/archived/retire) with a reason — do NOT raise the mark to pass."
        )

    if problems:
        print("workspace coverage check FAILED\n")
        for p in problems:
            print(f"  {p}")
        print(
            "\nA nested workspace is invisible to root-level cargo commands. Declare it\n"
            "in scripts/workspace-inventory.txt (scripts/check-workspace-coverage.py\n"
            "--update), and prefer NOT adding new ones — they fragment verification.\n"
        )
        return 1

    print(
        f"workspace coverage OK — {len(found)} nested workspace(s), "
        f"{len(found) - len(unverified)} in CI, {len(unverified)} unverified "
        f"({len(unverified_canonical)} canonical, high-water {mark})"
    )
    if unverified_canonical:
        print(
            "\n  CANONICAL but not built or tested by CI at all — this is the backlog:\n    "
            + "\n    ".join(unverified_canonical)
        )
        print(
            "\n  Shrink it by adding a cluster to CI (then lower the high-water mark), or by\n"
            "  reclassifying it experimental/archived/retire with a stated reason.\n"
            "  Raising the mark to make this pass defeats the point."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())

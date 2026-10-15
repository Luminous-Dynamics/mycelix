#!/usr/bin/env python3
"""Guard the D6S qualification scope and dependency-manifest pin boundary."""

from __future__ import annotations

import fnmatch
import re
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_MANIFEST = ROOT / "mycelix-manufacturing/Cargo.toml"
WORKFLOW_PATH = ROOT / ".github/workflows/d6s-canonical-qualification.yml"


def fail(message: str) -> "NoReturn":
    print(f"D6S_SCOPE_CONTRACT_FAILURE: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> int:
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
    workspace = tomllib.loads(WORKSPACE_MANIFEST.read_text(encoding="utf-8"))
    members = workspace.get("workspace", {}).get("members", [])
    if not members:
        fail("workspace member list is missing or empty")

    manifest_paths = [
        "mycelix-manufacturing/Cargo.toml",
        *(f"mycelix-manufacturing/{member}/Cargo.toml" for member in members),
    ]

    pin_paths = {
        path
        for path, _blob in re.findall(
            r'^\s*check_blob\s+"([^"]+)"\s+"([0-9a-f]{40})"\s*$',
            workflow,
            re.MULTILINE,
        )
    }
    missing_pins = sorted(set(manifest_paths) - pin_paths)
    if missing_pins:
        fail("workspace manifests lack immutable Git blob pins: " + ", ".join(missing_pins))

    scope_lines = [
        line.strip()
        for line in workflow.splitlines()
        if "docs/integral/*|scripts/integral/*" in line
    ]
    if len(scope_lines) != 1:
        fail(f"expected one auditable D6S scope pattern, found {len(scope_lines)}")
    scope_pattern = scope_lines[0].removesuffix(")")
    patterns = scope_pattern.split("|")

    # Test every declared workspace manifest, including non-COS members whose
    # dependency declarations still affect the workspace lock graph.
    missed = [
        path for path in manifest_paths
        if not any(fnmatch.fnmatchcase(path, pattern) for pattern in patterns)
    ]
    if missed:
        fail("workspace manifests do not trigger D6S scope: " + ", ".join(missed))

    lock_path = "mycelix-manufacturing/Cargo.lock"
    if not any(fnmatch.fnmatchcase(lock_path, pattern) for pattern in patterns):
        fail("the future committed Cargo.lock is not in the D6S scope")

    if any(fnmatch.fnmatchcase("README.md", pattern) for pattern in patterns):
        fail("unrelated README.md changes incorrectly trigger D6S qualification")

    print(
        "D6S_SCOPE_CONTRACT_OK "
        f"workspace_manifests={len(manifest_paths)} "
        f"pinned_manifests={len(manifest_paths)} "
        "lockfile_scope=true unrelated_path_negative_control=true"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Adversarial contract checks for D6S scope and dependency provenance."""

from __future__ import annotations

import fnmatch
import re
import sys
import tomllib
from pathlib import Path
from typing import NoReturn

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_MANIFEST = ROOT / "mycelix-manufacturing/Cargo.toml"
WORKFLOW_PATH = ROOT / ".github/workflows/d6s-canonical-qualification.yml"


def fail(message: str) -> NoReturn:
    print(f"D6S_SCOPE_CONTRACT_FAILURE: {message}", file=sys.stderr)
    raise SystemExit(1)


def scope_patterns(workflow: str) -> list[str]:
    lines = [
        line.strip()
        for line in workflow.splitlines()
        if "docs/integral/*|scripts/integral/*" in line
    ]
    if len(lines) != 1:
        return []
    return lines[0].removesuffix(")").split("|")


def contract_violations(workflow: str, members: list[str]) -> list[str]:
    violations: list[str] = []
    manifest_paths = [
        "mycelix-manufacturing/Cargo.toml",
        *(f"mycelix-manufacturing/{member}/Cargo.toml" for member in members),
    ]
    pins = {
        path: blob
        for path, blob in re.findall(
            r'^\s*check_blob\s+"([^"]+)"\s+"([0-9a-f]{40})"\s*$',
            workflow,
            re.MULTILINE,
        )
    }
    for path in manifest_paths:
        if path not in pins:
            violations.append(f"workspace manifest lacks an immutable blob pin: {path}")

    patterns = scope_patterns(workflow)
    if not patterns:
        violations.append("expected exactly one auditable D6S scope pattern")
    else:
        for path in manifest_paths:
            if not any(fnmatch.fnmatchcase(path, pattern) for pattern in patterns):
                violations.append(f"workspace manifest does not trigger D6S scope: {path}")
        lock_path = "mycelix-manufacturing/Cargo.lock"
        if not any(fnmatch.fnmatchcase(lock_path, pattern) for pattern in patterns):
            violations.append("Cargo.lock does not trigger D6S scope")
        if any(fnmatch.fnmatchcase("README.md", pattern) for pattern in patterns):
            violations.append("unrelated README.md incorrectly triggers D6S scope")

    if 'test "$(git rev-parse HEAD)" = "$QUALIFIED_SHA"' not in workflow:
        violations.append("workflow does not assert the exact qualification subject")

    lock_start = workflow.find("      - name: Resolve or validate reference-test dependency lock")
    provenance_start = workflow.find("      - name: Capture dependency graph provenance")
    if lock_start < 0 or provenance_start <= lock_start:
        violations.append("dependency-lock resolution/provenance steps are missing or reordered")
        resolution = ""
        provenance = ""
    else:
        resolution = workflow[lock_start:provenance_start]
        provenance_end = workflow.find("\n      - name:", provenance_start + 10)
        provenance = workflow[provenance_start:provenance_end if provenance_end >= 0 else len(workflow)]

    committed_branch = ""
    generated_branch = ""
    conditional = 'if [[ -f "$lockfile" ]]; then'
    if conditional not in resolution:
        violations.append("lockfile branch does not distinguish committed from generated lock")
    else:
        _, rest = resolution.split(conditional, 1)
        branches = rest.split("\n          else\n", 1)
        if len(branches) != 2:
            violations.append("committed/generated lock branches are not a simple if/else")
        else:
            committed_branch, tail = branches
            generated_parts = tail.split("\n          fi", 1)
            if len(generated_parts) != 2:
                violations.append("generated lock branch is not closed by fi")
            else:
                generated_branch = generated_parts[0]

    metadata_command = 'cargo metadata --manifest-path "$manifest" --locked --format-version 1'
    if metadata_command not in committed_branch:
        violations.append("committed lock is not validated by full cargo metadata --locked")
    if "--no-deps" in committed_branch:
        violations.append("committed lock metadata validation incorrectly omits dependency resolution")
    if "D6S_CARGO_LOCK_MODE=committed" not in committed_branch:
        violations.append("committed lock mode is not explicitly recorded")
    if "D6S_CARGO_LOCK_MODE=generated-per-run" not in generated_branch:
        violations.append("generated lock mode is not explicitly labelled per-run")
    if "cargo generate-lockfile --manifest-path \"$manifest\"" not in generated_branch:
        violations.append("missing-lock branch does not explicitly generate a lockfile")
    if "cargo generate-lockfile" in committed_branch:
        violations.append("committed lock branch must not regenerate the dependency graph")

    provenance_requirements = [
        "always()",
        'commit=$QUALIFIED_SHA',
        "GITHUB_RUN_ID",
        "GITHUB_RUN_ATTEMPT",
        "GITHUB_WORKFLOW_REF",
        "git hash-object .github/workflows/d6s-canonical-qualification.yml",
        "rustc --version",
        "workspace_manifest_count=",
        "manifest_blob[$path]=",
        "cargo_lock_sha256=",
        "cargo_lock_git_blob_sha=",
        "tee d6s-dependency-resolution.txt",
    ]
    for required in provenance_requirements:
        if required not in provenance:
            violations.append(f"dependency provenance is missing: {required}")

    artifact_start = workflow.find("      - name: Preserve dependency graph when qualification fails")
    if artifact_start < 0:
        artifact = ""
    else:
        artifact_end = workflow.find("\n      - name:", artifact_start + 10)
        artifact = workflow[artifact_start:artifact_end if artifact_end >= 0 else len(workflow)]
    for required in [
        "always()",
        "actions/upload-artifact@",
        "d6s-dependency-resolution.txt",
        "mycelix-manufacturing/Cargo.lock",
        "if-no-files-found: warn",
    ]:
        if required not in artifact:
            violations.append(f"failure-resilient dependency artifact is missing: {required}")

    if 'echo "cargo_lock_mode=${D6S_CARGO_LOCK_MODE:-unknown}"' not in workflow:
        violations.append("qualification receipt does not disclose lockfile mode")
    if "claim_ceiling=NotQualified" not in workflow:
        violations.append("out-of-scope runs lack a non-qualification claim ceiling")

    return violations


def main() -> int:
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
    workspace = tomllib.loads(WORKSPACE_MANIFEST.read_text(encoding="utf-8"))
    members = workspace.get("workspace", {}).get("members", [])
    if not members:
        fail("workspace member list is missing or empty")

    violations = contract_violations(workflow, members)
    if violations:
        fail("; ".join(violations))

    # Mutation controls prove this checker fails closed for representative
    # scope, resolution, and artifact-retention regressions.
    mutation_cases = [
        (
            "missing member pin",
            re.sub(
                r'^\s*check_blob\s+"'
                + re.escape(f"mycelix-manufacturing/{members[0]}/Cargo.toml")
                + r'"\s+"[0-9a-f]{40}"\s*$',
                "# intentionally removed member pin",
                workflow,
                count=1,
                flags=re.MULTILINE,
            ),
        ),
        (
            "lockfile omitted from scope",
            workflow.replace("|mycelix-manufacturing/Cargo.lock", "", 1),
        ),
        (
            "metadata skips dependency graph",
            workflow.replace(
                'cargo metadata --manifest-path "$manifest" --locked --format-version 1',
                'cargo metadata --manifest-path "$manifest" --locked --no-deps --format-version 1',
                1,
            ),
        ),
        (
            "failure artifact loses always condition",
            workflow.replace(
                "if: ${{ always() && steps.scope.outputs.required == 'true' && hashFiles('mycelix-manufacturing/Cargo.lock') != '' }}\n        uses: actions/upload-artifact@",
                "if: ${{ steps.scope.outputs.required == 'true' && hashFiles('mycelix-manufacturing/Cargo.lock') != '' }}\n        uses: actions/upload-artifact@",
                1,
            ),
        ),
        (
            "generated lock mislabeled",
            workflow.replace("D6S_CARGO_LOCK_MODE=generated-per-run", "D6S_CARGO_LOCK_MODE=generated", 1),
        ),
        (
            "unrelated path becomes in-scope",
            workflow.replace("|mycelix-manufacturing/Cargo.lock", "|README.md|mycelix-manufacturing/Cargo.lock", 1),
        ),
    ]
    for label, mutant in mutation_cases:
        if mutant == workflow:
            fail(f"mutation setup made no change: {label}")
        if not contract_violations(mutant, members):
            fail(f"contract checker failed to reject mutation: {label}")

    print(
        "D6S_WORKFLOW_CONTRACT_OK "
        f"workspace_manifests={len(members) + 1} "
        f"immutable_pins={len([p for p in manifest_paths_for(members) if p in pin_paths_for(workflow)])} "
        f"mutation_controls_rejected={len(mutation_cases)}"
    )
    return 0


def manifest_paths_for(members: list[str]) -> list[str]:
    return [
        "mycelix-manufacturing/Cargo.toml",
        *(f"mycelix-manufacturing/{member}/Cargo.toml" for member in members),
    ]


def pin_paths_for(workflow: str) -> set[str]:
    return {
        path
        for path, _blob in re.findall(
            r'^\s*check_blob\s+"([^"]+)"\s+"([0-9a-f]{40})"\s*$',
            workflow,
            re.MULTILINE,
        )
    }


if __name__ == "__main__":
    raise SystemExit(main())

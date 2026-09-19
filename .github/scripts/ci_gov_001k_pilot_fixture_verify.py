#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, pathlib, re, sys
from typing import Any

SCHEMA = "mycelix.ci-gov.001k.live-pilot-fixture-verifier.v0.1"
EXPECTED_SHA256 = "c1479745b168901f67931ff392b7c3e70ec4959a3841328943380719e9117cfe"
EXPECTED_GROUP = "mycelix-heavy-qualification-v1"
EXPECTED_LABEL = "ci:qualify-pilot"
EXPECTED_RUNNER = "ubuntu-24.04"
EXPECTED_TIMEOUT_MINUTES = 5
EXPECTED_HOLD_SECONDS = 90

class VerificationError(ValueError):
    pass

def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()

def _count(text: str, needle: str) -> int:
    return text.count(needle)

def inspect_fixture(text: str, enforce_digest: bool = True) -> dict[str, Any]:
    data = text.encode()
    if enforce_digest and sha256(data) != EXPECTED_SHA256:
        raise VerificationError("fixture_sha256_mismatch")
    if "\r" in text or not text.endswith("\n"):
        raise VerificationError("noncanonical_line_endings")
    required = (
        "name: CI-GOV-001K live queue:max pilot",
        "pull_request:",
        "types: [labeled]",
        "permissions:\n  contents: read",
        "concurrency:\n  group: mycelix-heavy-qualification-v1\n  queue: max",
        "github.event.label.name == 'ci:qualify-pilot'",
        "github.event.pull_request.draft == true",
        "runs-on: ubuntu-24.04",
        "timeout-minutes: 5",
        "shell: bash",
        "PILOT_PR: ${{ github.event.pull_request.number }}",
        "PILOT_HEAD: ${{ github.event.pull_request.head.sha }}",
        "PILOT_RUN_ID: ${{ github.run_id }}",
        "PILOT_RUN_ATTEMPT: ${{ github.run_attempt }}",
        "set -euo pipefail",
        "sleep 90",
    )
    for marker in required:
        if marker not in text:
            raise VerificationError("missing_required_marker:" + marker)
    forbidden = (
        "cancel-in-progress",
        "workflow_dispatch",
        "repository_dispatch",
        "schedule:",
        "push:",
        "pull_request_target",
        "permissions: write",
        ": write",
        "uses:",
        "actions/checkout",
        "continue-on-error",
        "strategy:",
        "matrix:",
        "services:",
        "container:",
        "secrets.",
        "github.token",
        "GITHUB_TOKEN",
        "curl ",
        "wget ",
        "gh ",
    )
    for marker in forbidden:
        if marker in text:
            raise VerificationError("forbidden_marker:" + marker)
    exact_counts = {
        "pull_request:": 1,
        "types: [labeled]": 1,
        "concurrency:": 1,
        f"group: {EXPECTED_GROUP}": 1,
        "queue: max": 1,
        "jobs:": 1,
        "  pilot:": 1,
        f"runs-on: {EXPECTED_RUNNER}": 1,
        f"timeout-minutes: {EXPECTED_TIMEOUT_MINUTES}": 1,
        "- name: Hold concurrency slot": 1,
        "shell: bash": 1,
        f"sleep {EXPECTED_HOLD_SECONDS}": 1,
        "run: |": 1,
    }
    for marker, expected in exact_counts.items():
        if _count(text, marker) != expected:
            raise VerificationError(f"unexpected_count:{marker}")
    if not re.search(r"(?m)^on:\n  pull_request:\n    types: \[labeled\]\n", text):
        raise VerificationError("trigger_shape_mismatch")
    if not re.search(r"(?m)^permissions:\n  contents: read\n", text):
        raise VerificationError("permissions_shape_mismatch")
    env_block = """        env:
          PILOT_PR: ${{ github.event.pull_request.number }}
          PILOT_HEAD: ${{ github.event.pull_request.head.sha }}
          PILOT_RUN_ID: ${{ github.run_id }}
          PILOT_RUN_ATTEMPT: ${{ github.run_attempt }}
"""
    if env_block not in text or text.count("        env:\n") != 1:
        raise VerificationError("evidence_env_shape_mismatch")
    if text.count("PILOT_PR:") != 1 or text.count("PILOT_HEAD:") != 1 or text.count("PILOT_RUN_ID:") != 1 or text.count("PILOT_RUN_ATTEMPT:") != 1:
        raise VerificationError("evidence_env_shape_mismatch")
    return {
        "schema": SCHEMA,
        "fixture_sha256": sha256(data),
        "group": EXPECTED_GROUP,
        "queue": "max",
        "cancel_in_progress": False,
        "label": EXPECTED_LABEL,
        "runner": EXPECTED_RUNNER,
        "timeout_minutes": EXPECTED_TIMEOUT_MINUTES,
        "hold_seconds": EXPECTED_HOLD_SECONDS,
        "jobs": 1,
        "third_party_actions": 0,
        "repository_mutation": False,
    }

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("fixture", type=pathlib.Path)
    args = p.parse_args()
    try:
        result = inspect_fixture(args.fixture.read_text())
    except (OSError, VerificationError) as exc:
        print(f"VERIFY ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())

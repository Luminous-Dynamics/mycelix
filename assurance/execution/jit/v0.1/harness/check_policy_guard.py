#!/usr/bin/env python3
"""Mutation tests for the JIT-1A public policy guard."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ADMIT = HERE / "admit.py"
POLICY = HERE / "policy.test.toml"
VALID = HERE / "fixtures" / "valid-envelope.json"
NOW = "1800000000000000"


def run(policy: Path):
    return subprocess.run([
        sys.executable,
        str(ADMIT),
        "--envelope", str(VALID),
        "--policy", str(policy),
        "--now-unix-micros", NOW,
    ], text=True, capture_output=True)


def last_failure(stderr: str):
    for line in reversed(stderr.splitlines()):
        try:
            value = json.loads(line)
        except Exception:
            continue
        if isinstance(value, dict) and "failure_code" in value:
            return value
    return None


ok = run(POLICY)
if ok.returncode != 0:
    raise SystemExit(f"canonical policy unexpectedly failed: {ok.stderr}")

with tempfile.TemporaryDirectory(prefix="jit-1a-policy-") as tmp:
    root = Path(tmp)

    extra = root / "extra.toml"
    extra.write_text(POLICY.read_text() + "\nunexpected = \"blocked\"\n")
    result = run(extra)
    if result.returncode != 3 or (last_failure(result.stderr) or {}).get("failure_code") != "H001_UNSAFE_TEST_POLICY":
        raise SystemExit(f"extra policy field was not blocked: {result.stderr}")

    traversal = root / "traversal.toml"
    text = POLICY.read_text().replace(
        'operator_authorization_fixture = "fixtures/operator-authorization.json"',
        'operator_authorization_fixture = "../operator-authorization.json"',
    )
    traversal.write_text(text)
    result = run(traversal)
    if result.returncode != 3 or (last_failure(result.stderr) or {}).get("failure_code") != "H001_UNSAFE_TEST_POLICY":
        raise SystemExit(f"fixture traversal was not blocked: {result.stderr}")

print("JIT-1A public policy guard mutation suite: PASS")

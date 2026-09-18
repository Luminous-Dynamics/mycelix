#!/usr/bin/env python3
"""JIT-1A offline controller mutation tests.

This test suite exercises admission only. It never provisions a VM, registers a
runner, contacts GitHub, or executes qualification commands.
"""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
CONTROLLER = HERE / "controller_dry_run.py"
POLICY = HERE / "policy.test.toml"
VALID = HERE / "fixtures" / "valid-envelope.json"
NOW = "1800000000000000"


def canonical(obj: dict) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def run(envelope: Path, ledger: Path | None = None):
    cmd = [
        sys.executable,
        str(CONTROLLER),
        "--envelope",
        str(envelope),
        "--policy",
        str(POLICY),
        "--now-unix-micros",
        NOW,
    ]
    if ledger is not None:
        cmd += ["--consume-nonce-ledger", str(ledger)]
    return subprocess.run(cmd, text=True, capture_output=True)


def require_pass(proc, label: str):
    if proc.returncode != 0:
        raise SystemExit(f"{label}: expected PASS, got rc={proc.returncode}: {proc.stderr}")
    payload = json.loads(proc.stdout.splitlines()[-1])
    if payload.get("admission_result") != "PASS" or payload.get("execution") != "not_performed" or payload.get("authority") != "none":
        raise SystemExit(f"{label}: unexpected PASS payload: {payload!r}")


def require_fail(proc, code: str, label: str):
    if proc.returncode == 0:
        raise SystemExit(f"{label}: expected {code}, got PASS")
    payload = None
    for line in reversed(proc.stderr.splitlines()):
        try:
            candidate = json.loads(line)
        except Exception:
            continue
        if isinstance(candidate, dict) and "failure_code" in candidate:
            payload = candidate
            break
    if payload is None:
        raise SystemExit(f"{label}: no failure JSON found in stderr: {proc.stderr!r}")
    if payload.get("failure_code") != code:
        raise SystemExit(f"{label}: expected {code}, got {payload!r}")


def write_mutation(root: Path, name: str, mutate):
    obj = json.loads(VALID.read_text())
    mutate(obj)
    path = root / f"{name}.json"
    path.write_bytes(canonical(obj))
    return path


with tempfile.TemporaryDirectory(prefix="jit-1a-") as tmp:
    root = Path(tmp)

    require_pass(run(VALID), "valid-envelope")

    ledger = root / "ledger"
    require_pass(run(VALID, ledger), "first-nonce-consumption")
    require_fail(run(VALID, ledger), "J004_REPLAYED_NONCE", "replayed-nonce")

    noncanonical = root / "noncanonical.json"
    noncanonical.write_text(json.dumps(json.loads(VALID.read_text()), indent=2))
    require_fail(run(noncanonical), "J002_NON_CANONICAL_ENVELOPE", "noncanonical")

    extra = write_mutation(root, "extra-field", lambda obj: obj.__setitem__("unexpected", "x"))
    require_fail(run(extra), "J001_MALFORMED_ENVELOPE", "extra-field")

    expired = write_mutation(root, "expired", lambda obj: obj.__setitem__("expires_at_unix_micros", int(NOW)))
    require_fail(run(expired), "J003_EXPIRED_ENVELOPE", "expired")

    subject = write_mutation(root, "subject", lambda obj: obj.__setitem__("subject_commit", "0" * 40))
    require_fail(run(subject), "J006_SUBJECT_MISMATCH", "subject-mismatch")

    profile = write_mutation(root, "profile", lambda obj: obj.__setitem__("qualification_profile_sha256", "0" * 64))
    require_fail(run(profile), "J023_PROFILE_MISMATCH", "profile-mismatch")

    traversal = write_mutation(root, "workflow-traversal", lambda obj: obj["workflow"].__setitem__("path", ".github/workflows/../evil.yml"))
    require_fail(run(traversal), "J007_WORKFLOW_MISMATCH", "workflow-traversal")

    toolchain = write_mutation(root, "toolchain", lambda obj: obj["toolchain"].__setitem__("cargo", "cargo 0.0.0"))
    require_fail(run(toolchain), "J009_TOOLCHAIN_MISMATCH", "toolchain-mismatch")

    image = write_mutation(root, "runner-image", lambda obj: obj.__setitem__("runner_image_sha256", "0" * 64))
    require_fail(run(image), "J010_RUNNER_IMAGE_MISMATCH", "runner-image-mismatch")

    network = write_mutation(root, "network", lambda obj: obj.__setitem__("network_policy_sha256", "0" * 64))
    require_fail(run(network), "J011_NETWORK_POLICY_MISMATCH", "network-policy-mismatch")

    commands = write_mutation(root, "commands", lambda obj: obj.__setitem__("allowed_commands_sha256", "0" * 64))
    require_fail(run(commands), "J012_COMMAND_PLAN_MISMATCH", "command-plan-mismatch")

    controller = write_mutation(root, "controller", lambda obj: obj.__setitem__("controller_policy_sha256", "0" * 64))
    require_fail(run(controller), "J021_CONTROLLER_POLICY_MISMATCH", "controller-policy-mismatch")

    group = write_mutation(root, "runner-group", lambda obj: obj.__setitem__("runner_group_id", "2"))
    require_fail(run(group), "J022_RUNNER_GROUP_MISMATCH", "runner-group-mismatch")

    operator = write_mutation(root, "operator", lambda obj: obj.__setitem__("operator_authorization_sha256", "0" * 64))
    require_fail(run(operator), "J005_UNAUTHORIZED_OPERATOR", "operator-auth-mismatch")

print("JIT-1A offline admission mutation suite: PASS")

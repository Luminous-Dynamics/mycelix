#!/usr/bin/env python3
"""Public JIT-1A dry-run entry point with closed-world test-policy guarding."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import tomllib

HERE = Path(__file__).resolve().parent
CONTROLLER = HERE / "controller_dry_run.py"

POLICY_FIELDS = {
    "version", "mode", "jit0_root_sha256", "qualification_profile_sha256",
    "runner_group_id", "max_ttl_micros",
    "workflow_path", "workflow_commit", "workflow_file_sha256", "workflow_fixture",
    "network_policy_sha256", "network_policy_fixture",
    "allowed_commands_sha256", "allowed_commands_fixture",
    "runner_image_sha256", "runner_image_fixture",
    "operator_authorization_fixture", "allowed_operator_authorization_sha256",
}
FIXTURE_FIELDS = {
    "workflow_fixture",
    "network_policy_fixture",
    "allowed_commands_fixture",
    "runner_image_fixture",
    "operator_authorization_fixture",
}


def fail(message: str) -> int:
    print(json.dumps({
        "admission_result": "FAIL",
        "failure_code": "H001_UNSAFE_TEST_POLICY",
        "message": message,
    }, sort_keys=True, separators=(",", ":")), file=sys.stderr)
    return 3


def safe_fixture(root: Path, rel: object, label: str) -> bool:
    if not isinstance(rel, str):
        return False
    path = Path(rel)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        return False
    unresolved = root / path
    if unresolved.is_symlink():
        return False
    try:
        target = unresolved.resolve(strict=True)
    except FileNotFoundError:
        return False
    return target.is_file() and root in target.parents


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--envelope", required=True, type=Path)
    parser.add_argument("--policy", required=True, type=Path)
    parser.add_argument("--now-unix-micros", required=True, type=int)
    parser.add_argument("--consume-nonce-ledger", type=Path)
    args = parser.parse_args()

    try:
        policy = tomllib.loads(args.policy.read_text())
    except Exception as exc:
        return fail(f"cannot parse test policy: {exc}")

    if set(policy) != POLICY_FIELDS:
        return fail("test policy field surface mismatch")
    if policy.get("version") != 1 or policy.get("mode") != "jit-1a-test-only":
        return fail("only jit-1a-test-only policy mode is admissible")

    root = args.policy.parent.resolve()
    for field in FIXTURE_FIELDS:
        if not safe_fixture(root, policy.get(field), field):
            return fail(f"unsafe fixture path: {field}")

    argv = [
        sys.executable,
        str(CONTROLLER),
        "--envelope", str(args.envelope),
        "--policy", str(args.policy),
        "--now-unix-micros", str(args.now_unix_micros),
    ]
    if args.consume_nonce_ledger is not None:
        argv += ["--consume-nonce-ledger", str(args.consume_nonce_ledger)]
    os.execv(sys.executable, argv)
    raise AssertionError("unreachable")


if __name__ == "__main__":
    raise SystemExit(main())

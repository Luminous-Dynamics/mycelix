#!/usr/bin/env python3
"""JIT-1A offline admission-controller dry run.

This program is intentionally incapable of registering a GitHub runner or
executing qualification commands. It validates a frozen JIT-0 envelope against
a test-only controller policy and optionally consumes the nonce in a local
atomic ledger.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tomllib

HERE = Path(__file__).resolve().parent
JIT0 = HERE.parent
JIT0_ROOT = "78a8d7d525252b31ff24245514ab9288caec6bea56fba02349e4415a0a55546c"
PROFILE_SHA = "9c9bea898f07a068e8d4c4be7d39adc81bf11a5b4ea9b9fc19d44b90c42a6a9b"
PROFILE_NAME = "assure-002b-l0-v1"
JCS_SAFE_INTEGER_MAX = 9_007_199_254_740_991

HEX40 = re.compile(r"^[0-9a-f]{40}$")
HEX64 = re.compile(r"^[0-9a-f]{64}$")
POSINT = re.compile(r"^[1-9][0-9]{0,19}$")
WORKFLOW = re.compile(r"^\.github/workflows/(?!\.\.?/)(?!.*(?:/\.\.?/|//))[A-Za-z0-9._/-]+\.ya?ml$")

TOP_FIELDS = {
    "schema", "repository", "subject_commit", "subject_tree",
    "expected_parent_commit", "qualified_parent_receipt_sha256",
    "workflow", "qualification_profile", "qualification_profile_sha256",
    "spec_root_sha256", "coverage_root_sha256", "corpus_root_sha256",
    "dependency_lock_sha256", "toolchain", "runner_image_sha256",
    "controller_policy_sha256", "runner_group_id", "network_policy_sha256",
    "allowed_commands_sha256", "receipt_schema",
    "containment_receipt_schema", "expires_at_unix_micros", "nonce_hex",
    "operator_authorization_sha256",
}
WORKFLOW_FIELDS = {"path", "commit", "file_sha256"}
TOOLCHAIN_FIELDS = {"rustc", "cargo", "target"}

PROFILE_FIELDS = {
    "repository": "repository",
    "subject_commit": "subject_commit",
    "subject_tree": "subject_tree",
    "expected_parent_commit": "expected_parent_commit",
    "qualified_parent_receipt_sha256": "qualified_parent_receipt_sha256",
    "spec_root_sha256": "spec_root_sha256",
    "coverage_root_sha256": "coverage_root_sha256",
    "corpus_root_sha256": "corpus_root_sha256",
    "dependency_lock_sha256": "dependency_lock_sha256",
    "receipt_schema": "receipt_schema",
    "containment_receipt_schema": "containment_receipt_schema",
}


class Reject(Exception):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def reject(code: str, message: str) -> None:
    raise Reject(code, message)


def no_float(value: str):
    reject("J001_MALFORMED_ENVELOPE", f"floating point is forbidden: {value}")


def object_no_duplicates(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            reject("J001_MALFORMED_ENVELOPE", f"duplicate key: {key}")
        result[key] = value
    return result


def ascii_walk(value, path="$" ) -> None:
    if isinstance(value, str):
        try:
            value.encode("ascii")
        except UnicodeEncodeError:
            reject("J001_MALFORMED_ENVELOPE", f"non-ASCII string at {path}")
    elif isinstance(value, bool) or value is None:
        reject("J001_MALFORMED_ENVELOPE", f"unsupported JSON scalar at {path}")
    elif isinstance(value, int):
        if abs(value) > JCS_SAFE_INTEGER_MAX:
            reject("J001_MALFORMED_ENVELOPE", f"integer outside JCS-safe range at {path}")
    elif isinstance(value, list):
        reject("J001_MALFORMED_ENVELOPE", f"arrays are not part of envelope v0.1 at {path}")
    elif isinstance(value, dict):
        for key, item in value.items():
            ascii_walk(key, f"{path}.<key>")
            ascii_walk(item, f"{path}.{key}")
    else:
        reject("J001_MALFORMED_ENVELOPE", f"unsupported JSON type at {path}")


def parse_canonical_envelope(path: Path):
    raw = path.read_bytes()
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        reject("J001_MALFORMED_ENVELOPE", f"invalid UTF-8: {exc}")
    try:
        obj = json.loads(
            text,
            object_pairs_hook=object_no_duplicates,
            parse_float=no_float,
            parse_constant=no_float,
        )
    except Reject:
        raise
    except Exception as exc:
        reject("J001_MALFORMED_ENVELOPE", f"invalid JSON: {exc}")
    if not isinstance(obj, dict):
        reject("J001_MALFORMED_ENVELOPE", "top-level envelope must be an object")
    ascii_walk(obj)
    canonical = json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    if raw != canonical:
        reject("J002_NON_CANONICAL_ENVELOPE", "input is not the frozen ASCII/JCS envelope subset")
    return obj, sha256_bytes(raw)


def exact_keys(obj, expected, label):
    if set(obj) != expected:
        reject("J001_MALFORMED_ENVELOPE", f"{label} fields mismatch")


def require_hex(value, regex, code, label):
    if not isinstance(value, str) or regex.fullmatch(value) is None:
        reject(code, f"invalid {label}")


def verify_jit0_contract() -> None:
    check = JIT0 / "check_spec.py"
    proc = subprocess.run(
        [sys.executable, str(check)],
        cwd=JIT0,
        text=True,
        capture_output=True,
    )
    if proc.returncode != 0:
        reject("J008_SPEC_OR_LOCK_MISMATCH", "JIT-0 contract checker failed")
    if f"jit_spec_root_sha256={JIT0_ROOT}" not in proc.stdout:
        reject("J008_SPEC_OR_LOCK_MISMATCH", "unexpected JIT-0 root")
    if f"qualification_profile_sha256={PROFILE_SHA}" not in proc.stdout:
        reject("J023_PROFILE_MISMATCH", "unexpected frozen profile hash")


def load_profile() -> dict:
    profile_path = JIT0 / "profiles" / f"{PROFILE_NAME}.toml"
    if sha256_bytes(profile_path.read_bytes()) != PROFILE_SHA:
        reject("J023_PROFILE_MISMATCH", "profile file hash mismatch")
    return tomllib.loads(profile_path.read_text())


def load_policy(path: Path, envelope: dict) -> dict:
    raw = path.read_bytes()
    digest = sha256_bytes(raw)
    if digest != envelope["controller_policy_sha256"]:
        reject("J021_CONTROLLER_POLICY_MISMATCH", "controller policy hash mismatch")
    policy = tomllib.loads(raw.decode("utf-8"))
    if policy.get("version") != 1 or policy.get("mode") != "jit-1a-test-only":
        reject("J021_CONTROLLER_POLICY_MISMATCH", "JIT-1A accepts test-only policy mode")
    if policy.get("jit0_root_sha256") != JIT0_ROOT:
        reject("J021_CONTROLLER_POLICY_MISMATCH", "policy JIT-0 root mismatch")
    if policy.get("qualification_profile_sha256") != PROFILE_SHA:
        reject("J021_CONTROLLER_POLICY_MISMATCH", "policy profile hash mismatch")
    return policy


def verify_fixture(policy_path: Path, policy: dict, path_key: str, expected: str, code: str) -> None:
    rel = policy.get(path_key)
    if not isinstance(rel, str) or Path(rel).is_absolute() or ".." in Path(rel).parts:
        reject(code, f"invalid policy fixture path: {path_key}")
    target = (policy_path.parent / rel).resolve()
    root = policy_path.parent.resolve()
    if root not in target.parents:
        reject(code, f"fixture escapes harness root: {path_key}")
    if sha256_bytes(target.read_bytes()) != expected:
        reject(code, f"fixture digest mismatch: {path_key}")


def verify_shape(env: dict) -> None:
    exact_keys(env, TOP_FIELDS, "envelope")
    if not isinstance(env.get("workflow"), dict) or not isinstance(env.get("toolchain"), dict):
        reject("J001_MALFORMED_ENVELOPE", "workflow/toolchain must be objects")
    exact_keys(env["workflow"], WORKFLOW_FIELDS, "workflow")
    exact_keys(env["toolchain"], TOOLCHAIN_FIELDS, "toolchain")
    if env["schema"] != "mycelix.qualification-execution-envelope.v0.1":
        reject("J001_MALFORMED_ENVELOPE", "wrong envelope schema")
    for field in ("subject_commit", "subject_tree", "expected_parent_commit"):
        require_hex(env[field], HEX40, "J006_SUBJECT_MISMATCH", field)
    for field in (
        "qualified_parent_receipt_sha256", "qualification_profile_sha256",
        "spec_root_sha256", "coverage_root_sha256", "corpus_root_sha256",
        "dependency_lock_sha256", "runner_image_sha256",
        "controller_policy_sha256", "network_policy_sha256",
        "allowed_commands_sha256", "operator_authorization_sha256",
    ):
        require_hex(env[field], HEX64, "J001_MALFORMED_ENVELOPE", field)
    require_hex(env["nonce_hex"], HEX64, "J001_MALFORMED_ENVELOPE", "nonce_hex")
    require_hex(env["workflow"]["commit"], HEX40, "J007_WORKFLOW_MISMATCH", "workflow.commit")
    require_hex(env["workflow"]["file_sha256"], HEX64, "J007_WORKFLOW_MISMATCH", "workflow.file_sha256")
    if not isinstance(env["workflow"]["path"], str) or WORKFLOW.fullmatch(env["workflow"]["path"]) is None:
        reject("J007_WORKFLOW_MISMATCH", "workflow path is not canonical")
    if not isinstance(env["runner_group_id"], str) or POSINT.fullmatch(env["runner_group_id"]) is None:
        reject("J022_RUNNER_GROUP_MISMATCH", "invalid runner group id")
    expiry = env["expires_at_unix_micros"]
    if not isinstance(expiry, int) or isinstance(expiry, bool) or not (0 <= expiry <= JCS_SAFE_INTEGER_MAX):
        reject("J003_EXPIRED_ENVELOPE", "invalid expiry")


def verify_profile(env: dict, profile: dict) -> None:
    if env["qualification_profile"] != PROFILE_NAME:
        reject("J023_PROFILE_MISMATCH", "profile name mismatch")
    if env["qualification_profile_sha256"] != PROFILE_SHA:
        reject("J023_PROFILE_MISMATCH", "profile hash mismatch")
    for env_key, profile_key in PROFILE_FIELDS.items():
        if env[env_key] != profile[profile_key]:
            code = "J006_SUBJECT_MISMATCH" if env_key in {"repository", "subject_commit", "subject_tree", "expected_parent_commit"} else "J008_SPEC_OR_LOCK_MISMATCH"
            if env_key in {"receipt_schema", "containment_receipt_schema"}:
                code = "J023_PROFILE_MISMATCH"
            reject(code, f"profile-bound field mismatch: {env_key}")
    if env["toolchain"] != {
        "rustc": profile["rustc"],
        "cargo": profile["cargo"],
        "target": profile["target"],
    }:
        reject("J009_TOOLCHAIN_MISMATCH", "toolchain mismatch")


def verify_policy_bindings(env: dict, policy_path: Path, policy: dict, now: int) -> None:
    if env["runner_group_id"] != policy.get("runner_group_id"):
        reject("J022_RUNNER_GROUP_MISMATCH", "runner group mismatch")
    if env["workflow"] != {
        "path": policy.get("workflow_path"),
        "commit": policy.get("workflow_commit"),
        "file_sha256": policy.get("workflow_file_sha256"),
    }:
        reject("J007_WORKFLOW_MISMATCH", "workflow identity mismatch")
    if env["network_policy_sha256"] != policy.get("network_policy_sha256"):
        reject("J011_NETWORK_POLICY_MISMATCH", "network policy mismatch")
    if env["allowed_commands_sha256"] != policy.get("allowed_commands_sha256"):
        reject("J012_COMMAND_PLAN_MISMATCH", "command plan mismatch")
    if env["runner_image_sha256"] != policy.get("runner_image_sha256"):
        reject("J010_RUNNER_IMAGE_MISMATCH", "runner image mismatch")
    if env["operator_authorization_sha256"] not in policy.get("allowed_operator_authorization_sha256", []):
        reject("J005_UNAUTHORIZED_OPERATOR", "operator authorization not allowlisted")
    expiry = env["expires_at_unix_micros"]
    if expiry <= now:
        reject("J003_EXPIRED_ENVELOPE", "envelope expired")
    max_ttl = policy.get("max_ttl_micros")
    if not isinstance(max_ttl, int) or max_ttl <= 0 or expiry - now > max_ttl:
        reject("J003_EXPIRED_ENVELOPE", "expiry exceeds policy TTL")

    verify_fixture(policy_path, policy, "workflow_fixture", env["workflow"]["file_sha256"], "J007_WORKFLOW_MISMATCH")
    verify_fixture(policy_path, policy, "network_policy_fixture", env["network_policy_sha256"], "J011_NETWORK_POLICY_MISMATCH")
    verify_fixture(policy_path, policy, "allowed_commands_fixture", env["allowed_commands_sha256"], "J012_COMMAND_PLAN_MISMATCH")
    verify_fixture(policy_path, policy, "runner_image_fixture", env["runner_image_sha256"], "J010_RUNNER_IMAGE_MISMATCH")

    op_rel = policy.get("operator_authorization_fixture")
    if not isinstance(op_rel, str):
        reject("J005_UNAUTHORIZED_OPERATOR", "operator fixture missing")
    op_path = (policy_path.parent / op_rel).resolve()
    if sha256_bytes(op_path.read_bytes()) != env["operator_authorization_sha256"]:
        reject("J005_UNAUTHORIZED_OPERATOR", "operator authorization fixture mismatch")


def consume_nonce(ledger: Path, nonce: str) -> None:
    ledger.mkdir(mode=0o700, parents=True, exist_ok=True)
    marker = ledger / nonce
    try:
        fd = os.open(marker, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        reject("J004_REPLAYED_NONCE", "nonce already consumed")
    try:
        os.write(fd, b"consumed\n")
        os.fsync(fd)
    finally:
        os.close(fd)
    dfd = os.open(ledger, os.O_RDONLY)
    try:
        os.fsync(dfd)
    finally:
        os.close(dfd)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--envelope", required=True, type=Path)
    parser.add_argument("--policy", required=True, type=Path)
    parser.add_argument("--now-unix-micros", required=True, type=int)
    parser.add_argument("--consume-nonce-ledger", type=Path)
    args = parser.parse_args()

    try:
        verify_jit0_contract()
        env, envelope_sha = parse_canonical_envelope(args.envelope)
        verify_shape(env)
        profile = load_profile()
        verify_profile(env, profile)
        policy = load_policy(args.policy, env)
        verify_policy_bindings(env, args.policy, policy, args.now_unix_micros)
        if args.consume_nonce_ledger is not None:
            consume_nonce(args.consume_nonce_ledger, env["nonce_hex"])
    except Reject as exc:
        print(json.dumps({"admission_result": "FAIL", "failure_code": exc.code, "message": exc.message}, sort_keys=True, separators=(",", ":")), file=sys.stderr)
        return 2

    print(json.dumps({
        "admission_result": "PASS",
        "authority": "none",
        "execution": "not_performed",
        "envelope_sha256": envelope_sha,
        "jit0_root_sha256": JIT0_ROOT,
        "qualification_profile_sha256": PROFILE_SHA,
        "nonce_hex": env["nonce_hex"],
    }, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

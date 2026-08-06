#!/usr/bin/env python3
"""Create and verify root-signed release signer governance policies."""

from __future__ import annotations

import argparse
import base64
import json
import os
import stat
import subprocess
import sys
from pathlib import Path
from typing import Any

from evidence_manifest import EvidenceError, canonical_json_bytes, manifest_hash, require, require_hash32, require_text, sha256_b64

POLICY_SCHEMA = "mycelix-lawful-identity-release-signer-policy-v1"
POLICY_ROLE_RELEASE = "release-evidence"
POLICY_ROLE_DRIVER = "driver-review"
ALLOWED_ROLES = {POLICY_ROLE_RELEASE, POLICY_ROLE_DRIVER}
MAX_JSON_BYTES = 512 * 1024
MAX_SIGNATURE_BYTES = 4096
MAX_KEY_BYTES = 64 * 1024
OPENSSL_TIMEOUT_SECONDS = 30


class PolicyError(RuntimeError):
    pass


def checked_regular_file(value: Path, name: str, maximum_bytes: int) -> Path:
    absolute = value if value.is_absolute() else Path.cwd() / value
    absolute = Path(os.path.abspath(absolute))
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current = current / part
        try:
            mode = current.lstat().st_mode
        except OSError as error:
            raise PolicyError(f"{name} cannot be inspected: {error}") from error
        require(not stat.S_ISLNK(mode), f"{name} may not traverse symlinks")
    resolved = absolute.resolve(strict=True)
    require(resolved.is_file(), f"{name} must be a regular file")
    require(resolved.stat().st_size <= maximum_bytes, f"{name} exceeds size limit")
    return resolved


def run_openssl(*args: str) -> bytes:
    try:
        completed = subprocess.run(
            ["openssl", *args], check=False, capture_output=True, timeout=OPENSSL_TIMEOUT_SECONDS
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise PolicyError(f"openssl invocation failed: {error}") from error
    stderr = completed.stderr.decode("utf-8", errors="replace").strip()
    require(completed.returncode == 0, f"openssl exited {completed.returncode}" + (f": {stderr}" if stderr else ""))
    return completed.stdout


def public_key_der(public_key: Path) -> bytes:
    path = checked_regular_file(public_key, "public key", MAX_KEY_BYTES)
    return run_openssl("pkey", "-pubin", "-in", str(path), "-outform", "DER")


def derived_public_key_der(private_key: Path) -> bytes:
    path = checked_regular_file(private_key, "private key", MAX_KEY_BYTES)
    return run_openssl("pkey", "-in", str(path), "-pubout", "-outform", "DER")


def key_id_from_der(der: bytes) -> str:
    return sha256_b64(der)


def load_json(path: Path, description: str) -> dict[str, Any]:
    checked = checked_regular_file(path, description, MAX_JSON_BYTES)
    try:
        value = json.loads(checked.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise PolicyError(f"invalid {description}: {error}") from error
    require(isinstance(value, dict), f"{description} must be an object")
    return value


def validate_policy_shape(policy: dict[str, Any]) -> None:
    expected = {
        "schema",
        "policy_id",
        "policy_version",
        "created_at_unix_micros",
        "valid_from_unix_micros",
        "valid_until_unix_micros",
        "root_public_key_sha256",
        "keys",
    }
    require(set(policy) == expected, "signer policy has missing or unknown fields")
    require(policy.get("schema") == POLICY_SCHEMA, "unsupported signer policy schema")
    require_hash32(policy.get("policy_id"), "policy_id")
    version = policy.get("policy_version")
    require(isinstance(version, int) and version > 0, "policy_version must be positive")
    created = policy.get("created_at_unix_micros")
    valid_from = policy.get("valid_from_unix_micros")
    valid_until = policy.get("valid_until_unix_micros")
    require(isinstance(created, int) and created > 0, "invalid policy creation time")
    require(isinstance(valid_from, int) and valid_from > 0, "invalid policy validity start")
    require(isinstance(valid_until, int) and valid_until > valid_from, "invalid policy validity end")
    require(created <= valid_from, "policy creation must not follow validity start")
    require_hash32(policy.get("root_public_key_sha256"), "root_public_key_sha256")
    keys = policy.get("keys")
    require(isinstance(keys, list) and 1 <= len(keys) <= 32, "policy requires 1-32 keys")
    seen: set[str] = set()
    for index, key in enumerate(keys):
        require(isinstance(key, dict), f"keys[{index}] must be an object")
        expected_key = {
            "key_id",
            "public_key_der_b64",
            "roles",
            "valid_from_unix_micros",
            "valid_until_unix_micros",
            "revoked_at_unix_micros",
            "revocation_mode",
        }
        require(set(key) == expected_key, f"keys[{index}] has missing or unknown fields")
        key_id = require_hash32(key.get("key_id"), f"keys[{index}].key_id")
        require(key_id not in seen, "policy contains duplicate key ids")
        seen.add(key_id)
        try:
            der = base64.b64decode(key.get("public_key_der_b64"), validate=True)
        except (TypeError, ValueError) as error:
            raise PolicyError(f"keys[{index}].public_key_der_b64 is invalid") from error
        require(0 < len(der) <= MAX_KEY_BYTES, f"keys[{index}] public key is invalid")
        require(key_id_from_der(der) == key_id, f"keys[{index}] key id does not match public key")
        roles = key.get("roles")
        require(isinstance(roles, list) and roles, f"keys[{index}].roles must be non-empty")
        require(all(isinstance(role, str) and role in ALLOWED_ROLES for role in roles), f"keys[{index}] has unsupported roles")
        require(len(roles) == len(set(roles)), f"keys[{index}] has duplicate roles")
        key_from = key.get("valid_from_unix_micros")
        key_until = key.get("valid_until_unix_micros")
        require(isinstance(key_from, int) and valid_from <= key_from < valid_until, f"keys[{index}] validity start is outside policy")
        require(isinstance(key_until, int) and key_from < key_until <= valid_until, f"keys[{index}] validity end is outside policy")
        revoked = key.get("revoked_at_unix_micros")
        require(revoked is None or (isinstance(revoked, int) and revoked > 0), f"keys[{index}] has invalid revocation time")
        mode = key.get("revocation_mode")
        require(mode in {"prospective", "retroactive"}, f"keys[{index}] has invalid revocation mode")
        require(revoked is not None or mode == "prospective", f"keys[{index}] cannot be retroactive without revocation")


def verify_policy(policy_path: Path, signature_path: Path, root_public_key: Path) -> dict[str, Any]:
    policy_file = checked_regular_file(policy_path, "signer policy", MAX_JSON_BYTES)
    signature = checked_regular_file(signature_path, "signer policy signature", MAX_SIGNATURE_BYTES)
    root = checked_regular_file(root_public_key, "policy root public key", MAX_KEY_BYTES)
    policy = load_json(policy_file, "signer policy")
    validate_policy_shape(policy)
    require(policy["root_public_key_sha256"] == key_id_from_der(public_key_der(root)), "signer policy root key mismatch")
    run_openssl(
        "pkeyutl", "-verify", "-rawin", "-pubin", "-inkey", str(root),
        "-in", str(policy_file), "-sigfile", str(signature),
    )
    return policy


def policy_identity(policy_path: Path, policy: dict[str, Any]) -> tuple[str, str]:
    checked = checked_regular_file(policy_path, "signer policy", MAX_JSON_BYTES)
    return sha256_b64(checked.read_bytes()), manifest_hash(policy)


def authorized_key(policy: dict[str, Any], role: str, key_id: str, signed_at_unix_micros: int) -> bytes:
    require(role in ALLOWED_ROLES, "unsupported signer role")
    require_hash32(key_id, "signer key id")
    require(isinstance(signed_at_unix_micros, int) and signed_at_unix_micros > 0, "invalid signing time")
    require(policy["valid_from_unix_micros"] <= signed_at_unix_micros < policy["valid_until_unix_micros"], "signing time is outside policy validity")
    matches = [entry for entry in policy["keys"] if entry["key_id"] == key_id]
    require(len(matches) == 1, "signer key is not present exactly once in policy")
    entry = matches[0]
    require(role in entry["roles"], f"signer key is not authorized for {role}")
    require(entry["valid_from_unix_micros"] <= signed_at_unix_micros < entry["valid_until_unix_micros"], "signing time is outside key validity")
    revoked = entry["revoked_at_unix_micros"]
    if revoked is not None:
        if entry["revocation_mode"] == "retroactive":
            raise PolicyError("signer key is retroactively revoked")
        require(signed_at_unix_micros < revoked, "signing time is at or after prospective revocation")
    return base64.b64decode(entry["public_key_der_b64"], validate=True)


def private_key_id(private_key: Path) -> str:
    return key_id_from_der(derived_public_key_der(private_key))


def verify_signature_with_der(message: Path, signature: Path, der: bytes) -> None:
    message = checked_regular_file(message, "signed message", MAX_JSON_BYTES)
    signature = checked_regular_file(signature, "signature", MAX_SIGNATURE_BYTES)
    import tempfile
    with tempfile.TemporaryDirectory() as temporary:
        key_path = Path(temporary) / "key.der"
        key_path.write_bytes(der)
        run_openssl(
            "pkeyutl", "-verify", "-rawin", "-pubin", "-keyform", "DER",
            "-inkey", str(key_path), "-in", str(message), "-sigfile", str(signature),
        )


def sign_file(message: Path, signature: Path, private_key: Path) -> None:
    message = checked_regular_file(message, "signed message", MAX_JSON_BYTES)
    private_key = checked_regular_file(private_key, "signing key", MAX_KEY_BYTES)
    require(not signature.exists() and not signature.is_symlink(), "signature output already exists")
    run_openssl("pkeyutl", "-sign", "-rawin", "-inkey", str(private_key), "-in", str(message), "-out", str(signature))
    require(signature.is_file() and 0 < signature.stat().st_size <= MAX_SIGNATURE_BYTES, "signature output is invalid")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--signature", type=Path, required=True)
    parser.add_argument("--root-public-key", type=Path, required=True)
    parser.add_argument("--role")
    parser.add_argument("--key-id")
    parser.add_argument("--signed-at-unix-micros", type=int)
    args = parser.parse_args()
    try:
        policy = verify_policy(args.policy, args.signature, args.root_public_key)
        if any(value is not None for value in [args.role, args.key_id, args.signed_at_unix_micros]):
            require(all(value is not None for value in [args.role, args.key_id, args.signed_at_unix_micros]), "role, key id, and signing time must be supplied together")
            authorized_key(policy, args.role, args.key_id, args.signed_at_unix_micros)
        file_hash, canonical_hash = policy_identity(args.policy, policy)
        print("release signer policy: PASS")
        print(f"policy_version={policy['policy_version']}")
        print(f"policy_file_sha256={file_hash}")
        print(f"policy_canonical_hash={canonical_hash}")
        return 0
    except (EvidenceError, PolicyError, OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"release signer policy: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

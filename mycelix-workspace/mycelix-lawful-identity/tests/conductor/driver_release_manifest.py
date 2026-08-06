#!/usr/bin/env python3
"""Create and verify reviewed release manifests for real-conductor drivers."""

from __future__ import annotations

import argparse
import json
import os
import stat
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from evidence_manifest import EvidenceError, canonical_json_bytes, contract_hash, file_sha256_b64, manifest_hash, require, require_hash32, require_text, sha256_b64
from release_signer_policy import (
    MAX_JSON_BYTES,
    MAX_KEY_BYTES,
    MAX_SIGNATURE_BYTES,
    POLICY_ROLE_DRIVER,
    PolicyError,
    authorized_key,
    checked_regular_file,
    policy_identity,
    private_key_id,
    sign_file,
    verify_policy,
    verify_signature_with_der,
)

ROOT = Path(__file__).resolve().parent
CONTRACT_PATH = ROOT / "settlement_scenarios_v2.json"
DRIVER_MANIFEST_SCHEMA = "mycelix-real-conductor-driver-release-v1"
DRIVER_PROTOCOL = "mycelix-real-conductor-driver-v1"
ADAPTER_PROTOCOL = "mycelix-conductor-scenario-adapter-v3"
MAX_DRIVER_STDOUT_BYTES = 256 * 1024
DRIVER_TIMEOUT_SECONDS = 60


class DriverReleaseError(RuntimeError):
    pass


def checked_driver(value: Path) -> Path:
    absolute = value if value.is_absolute() else Path.cwd() / value
    absolute = Path(os.path.abspath(absolute))
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current = current / part
        try:
            mode = current.lstat().st_mode
        except OSError as error:
            raise DriverReleaseError(f"driver cannot be inspected: {error}") from error
        require(not stat.S_ISLNK(mode), "driver may not traverse symlinks")
    resolved = absolute.resolve(strict=True)
    require(resolved.is_file(), "driver must be a regular file")
    require(resolved.stat().st_mode & 0o111 != 0, "driver must be executable")
    require(resolved.stat().st_size <= 64 * 1024 * 1024, "driver exceeds size limit")
    return resolved


def invoke_capabilities(driver: Path) -> dict[str, Any]:
    driver = checked_driver(driver)
    before = file_sha256_b64(driver)
    try:
        completed = subprocess.run([str(driver), "--capabilities"], check=False, capture_output=True, timeout=DRIVER_TIMEOUT_SECONDS)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise DriverReleaseError(f"driver capability invocation failed: {error}") from error
    require(file_sha256_b64(driver) == before, "driver changed during capability invocation")
    require(len(completed.stdout) <= MAX_DRIVER_STDOUT_BYTES, "driver capability output exceeds limit")
    stderr = completed.stderr.decode("utf-8", errors="replace").strip()
    require(completed.returncode == 0, f"driver capability invocation failed with {completed.returncode}" + (f": {stderr}" if stderr else ""))
    try:
        value = json.loads(completed.stdout.decode("utf-8", errors="strict"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise DriverReleaseError(f"driver capabilities are not one UTF-8 JSON value: {error}") from error
    require(isinstance(value, dict), "driver capabilities must be an object")
    require(value.get("driver_protocol") == DRIVER_PROTOCOL, "unsupported driver protocol")
    require_hash32(value.get("driver_release_hash"), "driver_release_hash")
    require_text(value.get("holochain_version"), "holochain_version", 256)
    return value



def reviewed_capabilities_hash(capabilities: dict[str, Any]) -> str:
    require(isinstance(capabilities, dict), "driver capabilities must be an object")
    require("driver_release_hash" in capabilities, "driver capabilities are missing driver_release_hash")
    reviewed = dict(capabilities)
    reviewed.pop("driver_release_hash")
    return manifest_hash(reviewed)

def release_identity(
    driver_executable_sha256: str,
    capabilities_canonical_hash: str,
    source_git_commit: str,
    source_git_tree: str,
    build_recipe_sha256: str,
    lockfile_sha256: str,
    holochain_version: str,
) -> str:
    value = {
        "domain": "MYCELIX:RealConductorDriverRelease:v1",
        "driver_protocol": DRIVER_PROTOCOL,
        "adapter_protocol": ADAPTER_PROTOCOL,
        "scenario_contract_hash": contract_hash(CONTRACT_PATH),
        "driver_executable_sha256": require_hash32(driver_executable_sha256, "driver_executable_sha256"),
        "capabilities_canonical_hash": require_hash32(capabilities_canonical_hash, "capabilities_canonical_hash"),
        "source_git_commit": require_text(source_git_commit, "source_git_commit", 256),
        "source_git_tree": require_text(source_git_tree, "source_git_tree", 256),
        "build_recipe_sha256": require_hash32(build_recipe_sha256, "build_recipe_sha256"),
        "lockfile_sha256": require_hash32(lockfile_sha256, "lockfile_sha256"),
        "holochain_version": require_text(holochain_version, "holochain_version", 256),
    }
    return sha256_b64(canonical_json_bytes(value))


def validate_manifest_shape(value: dict[str, Any]) -> None:
    expected = {
        "schema", "signature_algorithm", "signer_key_id", "signer_policy_version",
        "signer_policy_file_sha256", "signer_policy_canonical_hash", "driver_protocol",
        "adapter_protocol", "scenario_contract_hash", "driver_release_hash",
        "driver_executable_sha256", "capabilities_canonical_hash", "source_git_commit",
        "source_git_tree", "build_recipe_sha256", "lockfile_sha256", "holochain_version",
        "created_at_unix_micros", "valid_from_unix_micros", "valid_until_unix_micros",
    }
    require(set(value) == expected, "driver release manifest has missing or unknown fields")
    require(value.get("schema") == DRIVER_MANIFEST_SCHEMA, "unsupported driver release schema")
    require(value.get("signature_algorithm") == "ed25519", "unsupported driver release signature algorithm")
    require(value.get("driver_protocol") == DRIVER_PROTOCOL, "unsupported driver protocol")
    require(value.get("adapter_protocol") == ADAPTER_PROTOCOL, "unsupported adapter protocol")
    for field in ["signer_key_id", "signer_policy_file_sha256", "signer_policy_canonical_hash", "scenario_contract_hash", "driver_release_hash", "driver_executable_sha256", "capabilities_canonical_hash", "build_recipe_sha256", "lockfile_sha256"]:
        require_hash32(value.get(field), field)
    require(isinstance(value.get("signer_policy_version"), int) and value["signer_policy_version"] > 0, "invalid signer policy version")
    require_text(value.get("source_git_commit"), "source_git_commit", 256)
    require_text(value.get("source_git_tree"), "source_git_tree", 256)
    require_text(value.get("holochain_version"), "holochain_version", 256)
    for field in ["created_at_unix_micros", "valid_from_unix_micros", "valid_until_unix_micros"]:
        require(isinstance(value.get(field), int) and value[field] > 0, f"invalid {field}")
    require(value["created_at_unix_micros"] <= value["valid_from_unix_micros"] < value["valid_until_unix_micros"], "invalid driver release validity interval")


def load_manifest(path: Path) -> dict[str, Any]:
    checked = checked_regular_file(path, "driver release manifest", MAX_JSON_BYTES)
    try:
        value = json.loads(checked.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise DriverReleaseError(f"invalid driver release manifest: {error}") from error
    require(isinstance(value, dict), "driver release manifest must be an object")
    validate_manifest_shape(value)
    return value


def build_manifest(
    driver: Path,
    signing_key: Path,
    signer_policy: Path,
    signer_policy_signature: Path,
    policy_root_public_key: Path,
    source_git_commit: str,
    source_git_tree: str,
    build_recipe_sha256: str,
    lockfile_sha256: str,
    valid_from_unix_micros: int,
    valid_until_unix_micros: int,
    created_at_unix_micros: int,
) -> dict[str, Any]:
    driver = checked_driver(driver)
    capabilities = invoke_capabilities(driver)
    policy = verify_policy(signer_policy, signer_policy_signature, policy_root_public_key)
    signer_key_id = private_key_id(signing_key)
    authorized_key(policy, POLICY_ROLE_DRIVER, signer_key_id, created_at_unix_micros)
    policy_file_hash, policy_canonical_hash = policy_identity(signer_policy, policy)
    executable_hash = file_sha256_b64(driver)
    capabilities_hash = reviewed_capabilities_hash(capabilities)
    expected_release_hash = release_identity(executable_hash, capabilities_hash, source_git_commit, source_git_tree, build_recipe_sha256, lockfile_sha256, capabilities["holochain_version"])
    require(capabilities["driver_release_hash"] == expected_release_hash, "driver capabilities release hash does not match reviewed release identity")
    require(isinstance(valid_from_unix_micros, int) and isinstance(valid_until_unix_micros, int), "driver validity times must be integers")
    require(created_at_unix_micros <= valid_from_unix_micros < valid_until_unix_micros, "invalid driver release validity interval")
    return {
        "schema": DRIVER_MANIFEST_SCHEMA,
        "signature_algorithm": "ed25519",
        "signer_key_id": signer_key_id,
        "signer_policy_version": policy["policy_version"],
        "signer_policy_file_sha256": policy_file_hash,
        "signer_policy_canonical_hash": policy_canonical_hash,
        "driver_protocol": DRIVER_PROTOCOL,
        "adapter_protocol": ADAPTER_PROTOCOL,
        "scenario_contract_hash": contract_hash(CONTRACT_PATH),
        "driver_release_hash": expected_release_hash,
        "driver_executable_sha256": executable_hash,
        "capabilities_canonical_hash": capabilities_hash,
        "source_git_commit": source_git_commit,
        "source_git_tree": source_git_tree,
        "build_recipe_sha256": build_recipe_sha256,
        "lockfile_sha256": lockfile_sha256,
        "holochain_version": capabilities["holochain_version"],
        "created_at_unix_micros": created_at_unix_micros,
        "valid_from_unix_micros": valid_from_unix_micros,
        "valid_until_unix_micros": valid_until_unix_micros,
    }


def verify_driver_release(
    manifest_path: Path,
    signature_path: Path,
    signer_policy: Path,
    signer_policy_signature: Path,
    policy_root_public_key: Path,
    driver: Path,
    observed_capabilities: dict[str, Any] | None = None,
    observed_at_unix_micros: int | None = None,
    minimum_policy_version: int | None = None,
) -> dict[str, Any]:
    manifest_file = checked_regular_file(manifest_path, "driver release manifest", MAX_JSON_BYTES)
    signature = checked_regular_file(signature_path, "driver release signature", MAX_SIGNATURE_BYTES)
    manifest = load_manifest(manifest_file)
    policy = verify_policy(signer_policy, signer_policy_signature, policy_root_public_key)
    policy_file_hash, policy_canonical_hash = policy_identity(signer_policy, policy)
    require(manifest["signer_policy_version"] == policy["policy_version"], "driver release policy version mismatch")
    if minimum_policy_version is not None:
        require(policy["policy_version"] >= minimum_policy_version, "signer policy version is below required minimum")
    require(manifest["signer_policy_file_sha256"] == policy_file_hash, "driver release policy file hash mismatch")
    require(manifest["signer_policy_canonical_hash"] == policy_canonical_hash, "driver release policy canonical hash mismatch")
    signer_der = authorized_key(policy, POLICY_ROLE_DRIVER, manifest["signer_key_id"], manifest["created_at_unix_micros"])
    verify_signature_with_der(manifest_file, signature, signer_der)
    driver = checked_driver(driver)
    require(file_sha256_b64(driver) == manifest["driver_executable_sha256"], "driver executable does not match reviewed release")
    capabilities = observed_capabilities if observed_capabilities is not None else invoke_capabilities(driver)
    require(isinstance(capabilities, dict), "observed driver capabilities must be an object")
    require(reviewed_capabilities_hash(capabilities) == manifest["capabilities_canonical_hash"], "driver capabilities do not match reviewed release")
    require(capabilities.get("driver_release_hash") == manifest["driver_release_hash"], "driver release hash does not match reviewed manifest")
    require(capabilities.get("holochain_version") == manifest["holochain_version"], "driver Holochain version does not match reviewed manifest")
    require(manifest["scenario_contract_hash"] == contract_hash(CONTRACT_PATH), "driver release uses a different scenario contract")
    expected_release_hash = release_identity(manifest["driver_executable_sha256"], manifest["capabilities_canonical_hash"], manifest["source_git_commit"], manifest["source_git_tree"], manifest["build_recipe_sha256"], manifest["lockfile_sha256"], manifest["holochain_version"])
    require(manifest["driver_release_hash"] == expected_release_hash, "driver release identity is internally inconsistent")
    observed_at = time.time_ns() // 1000 if observed_at_unix_micros is None else observed_at_unix_micros
    require(manifest["valid_from_unix_micros"] <= observed_at < manifest["valid_until_unix_micros"], "driver release is outside its validity interval")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--create", action="store_true")
    mode.add_argument("--verify", action="store_true")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--signature", type=Path, required=True)
    parser.add_argument("--driver", type=Path, required=True)
    parser.add_argument("--signing-key", type=Path)
    parser.add_argument("--signer-policy", type=Path, required=True)
    parser.add_argument("--signer-policy-signature", type=Path, required=True)
    parser.add_argument("--policy-root-public-key", type=Path, required=True)
    parser.add_argument("--source-git-commit")
    parser.add_argument("--source-git-tree")
    parser.add_argument("--build-recipe-sha256")
    parser.add_argument("--lockfile-sha256")
    parser.add_argument("--valid-from-unix-micros", type=int)
    parser.add_argument("--valid-until-unix-micros", type=int)
    parser.add_argument("--minimum-policy-version", type=int)
    args = parser.parse_args()
    try:
        if args.create:
            require(args.signing_key is not None, "--signing-key is required with --create")
            for name in ["source_git_commit", "source_git_tree", "build_recipe_sha256", "lockfile_sha256", "valid_from_unix_micros", "valid_until_unix_micros"]:
                require(getattr(args, name) is not None, f"--{name.replace('_', '-')} is required with --create")
            require(not args.manifest.exists() and not args.manifest.is_symlink(), "driver release manifest output already exists")
            require(not args.signature.exists() and not args.signature.is_symlink(), "driver release signature output already exists")
            created_at = time.time_ns() // 1000
            manifest = build_manifest(args.driver, args.signing_key, args.signer_policy, args.signer_policy_signature, args.policy_root_public_key, args.source_git_commit, args.source_git_tree, args.build_recipe_sha256, args.lockfile_sha256, args.valid_from_unix_micros, args.valid_until_unix_micros, created_at)
            args.manifest.write_bytes(canonical_json_bytes(manifest) + b"\n")
            sign_file(args.manifest, args.signature, args.signing_key)
            verify_driver_release(args.manifest, args.signature, args.signer_policy, args.signer_policy_signature, args.policy_root_public_key, args.driver, minimum_policy_version=args.minimum_policy_version)
            print("driver release manifest: CREATED")
        else:
            require(args.signing_key is None, "--signing-key is not valid with --verify")
            manifest = verify_driver_release(args.manifest, args.signature, args.signer_policy, args.signer_policy_signature, args.policy_root_public_key, args.driver, minimum_policy_version=args.minimum_policy_version)
            print("driver release manifest: PASS")
        print(f"driver_release_hash={manifest['driver_release_hash']}")
        print(f"driver_executable_sha256={manifest['driver_executable_sha256']}")
        print(f"signer_key_id={manifest['signer_key_id']}")
        return 0
    except (DriverReleaseError, EvidenceError, PolicyError, OSError, ValueError) as error:
        print(f"driver release manifest: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

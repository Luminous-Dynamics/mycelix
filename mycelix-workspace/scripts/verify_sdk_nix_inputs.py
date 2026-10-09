#!/usr/bin/env python3
"""Fail-closed validation for the Mycelix TypeScript SDK's Nix-imported npm lock."""
from __future__ import annotations

import argparse
import base64
import binascii
import copy
import hashlib
import json
import re
import sys
from pathlib import Path
from urllib.parse import urlsplit


class InputContractError(ValueError):
    """The package manifest and lockfile do not satisfy the Nix import contract."""


SRI_TOKEN = re.compile(r"^sha512-[A-Za-z0-9+/]+={0,2}(?:\?.*)?$")
DEPENDENCY_SECTIONS = ("dependencies", "devDependencies", "optionalDependencies", "peerDependencies", "peerDependenciesMeta")
EXPECTED_REGISTRY = "registry.npmjs.org"
EXPECTED_INSTALL_LIFECYCLE_PACKAGES = {
    "node_modules/esbuild",
    "node_modules/fsevents",
    "node_modules/unrs-resolver",
}


def valid_sha512_token(token: str) -> bool:
    """Accept only canonical base64 SHA-512 SRI tokens (64 decoded bytes)."""
    if not SRI_TOKEN.fullmatch(token):
        return False
    encoded_digest = token.split("-", 1)[1].split("?", 1)[0]
    try:
        decoded = base64.b64decode(encoded_digest, validate=True)
    except (binascii.Error, ValueError):
        return False
    return len(decoded) == 64 and base64.b64encode(decoded).decode("ascii") == encoded_digest


def _object_without_duplicate_keys(pairs: list[tuple[str, object]]) -> dict:
    """Reject duplicate JSON members instead of accepting last-key-wins ambiguity."""
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise InputContractError(f"duplicate JSON object key: {key!r}")
        result[key] = value
    return result


def parse_object(raw: str, label: str) -> dict:
    try:
        value = json.loads(raw, object_pairs_hook=_object_without_duplicate_keys)
    except json.JSONDecodeError as exc:
        raise InputContractError(f"{label} is not valid JSON: {exc.msg} at line {exc.lineno}") from exc
    if not isinstance(value, dict):
        raise InputContractError(f"{label} must be a JSON object")
    return value


def verify_contract(package_raw: str, lock_raw: str) -> dict:
    manifest = parse_object(package_raw, "package.json")
    lock = parse_object(lock_raw, "package-lock.json")

    name = manifest.get("name")
    version = manifest.get("version")
    if not isinstance(name, str) or not name:
        raise InputContractError("package.json must declare a non-empty name")
    if not isinstance(version, str) or not version:
        raise InputContractError("package.json must declare a non-empty version")
    if lock.get("lockfileVersion") != 3:
        raise InputContractError("package-lock.json must use lockfileVersion 3")

    packages = lock.get("packages")
    if not isinstance(packages, dict) or not packages:
        raise InputContractError("package-lock.json packages must be a non-empty object")
    root = packages.get("")
    if not isinstance(root, dict):
        raise InputContractError("package-lock.json is missing the root package entry")
    if root.get("name") != name or root.get("version") != version:
        raise InputContractError("package.json name/version drift from package-lock.json root")

    for section in DEPENDENCY_SECTIONS:
        declared = manifest.get(section, {})
        locked = root.get(section, {})
        if not isinstance(declared, dict) or not isinstance(locked, dict):
            raise InputContractError(f"{section} must be an object in both manifest and lock root")
        if declared != locked:
            raise InputContractError(f"{section} drift between package.json and package-lock.json root")

    resolved_count = 0
    sha512_integrity_count = 0
    lifecycle_packages: list[str] = []
    platform_packages = 0
    for key, entry in sorted(packages.items()):
        if key == "":
            continue
        if not isinstance(key, str):
            raise InputContractError(f"unsupported package-lock entry path: {key!r}")
        path_segments = key.split("/")
        if (
            not key.startswith("node_modules/")
            or "\\" in key
            or any(segment in ("", ".", "..") for segment in path_segments)
        ):
            raise InputContractError(f"unsafe or unsupported package-lock entry path: {key!r}")
        if not isinstance(entry, dict):
            raise InputContractError(f"package entry {key} must be an object")
        if entry.get("link") is True:
            raise InputContractError(f"workspace/link entry is not allowed in the standalone SDK lock: {key}")
        package_version = entry.get("version")
        if not isinstance(package_version, str) or not package_version:
            raise InputContractError(f"package entry lacks a non-empty version: {key}")
        resolved = entry.get("resolved")
        if not isinstance(resolved, str) or not resolved:
            raise InputContractError(f"package entry lacks resolved source URL: {key}")
        has_forbidden_url_character = any(
            character.isspace()
            or ord(character) < 0x20
            or ord(character) == 0x7F
            for character in resolved
        )
        if "\\" in resolved or has_forbidden_url_character:
            raise InputContractError(f"package entry source URL contains whitespace, control, or backslash: {key}")
        try:
            parsed_url = urlsplit(resolved)
        except ValueError as exc:
            raise InputContractError(f"package entry has a malformed source URL: {key}") from exc
        if (
            parsed_url.scheme != "https"
            or parsed_url.netloc != EXPECTED_REGISTRY
            or not parsed_url.path
            or parsed_url.query
            or parsed_url.fragment
        ):
            raise InputContractError(f"package entry has a disallowed source URL: {key}")
        resolved_count += 1

        integrity = entry.get("integrity")
        if not isinstance(integrity, str) or not integrity.strip():
            raise InputContractError(f"package entry lacks an integrity hash: {key}")
        tokens = integrity.split()
        if not tokens or any(not valid_sha512_token(token) for token in tokens):
            raise InputContractError(f"package entry must have canonical SHA-512 integrity metadata: {key}")
        sha512_integrity_count += 1

        if entry.get("hasInstallScript") is True:
            lifecycle_packages.append(key)
        if "os" in entry or "cpu" in entry:
            platform_packages += 1

    if resolved_count == 0:
        raise InputContractError("lockfile contains no resolved package entries")
    if set(lifecycle_packages) != EXPECTED_INSTALL_LIFECYCLE_PACKAGES:
        missing = sorted(EXPECTED_INSTALL_LIFECYCLE_PACKAGES - set(lifecycle_packages))
        unexpected = sorted(set(lifecycle_packages) - EXPECTED_INSTALL_LIFECYCLE_PACKAGES)
        raise InputContractError(
            "install-lifecycle metadata changed; review explicitly "
            f"(missing={missing}, unexpected={unexpected})"
        )
    if platform_packages == 0:
        raise InputContractError("lockfile no longer records platform-constrained/native packages")

    return {
        "schema": "luminous.mycelix.sdk-nix-input-contract.v1",
        "package_name": name,
        "package_version": version,
        "lockfile_version": lock["lockfileVersion"],
        "package_node_count_including_root": len(packages),
        "resolved_package_count": resolved_count,
        "integrity_covered_package_count": resolved_count,
        "sha512_integrity_count": sha512_integrity_count,
        "integrity_coverage_complete": sha512_integrity_count == resolved_count,
        "approved_registry": EXPECTED_REGISTRY,
        "install_lifecycle_script_packages": lifecycle_packages,
        "platform_constrained_package_count": platform_packages,
    }


def run_self_tests(package_raw: str, lock_raw: str) -> list[dict]:
    package = parse_object(package_raw, "package.json")
    lock = parse_object(lock_raw, "package-lock.json")
    tests: list[tuple[str, str, str]] = []

    tests.append(("malformed_package_json", "{", lock_raw))
    tests.append(("malformed_lock_json", package_raw, "{"))
    duplicate_lock_raw = lock_raw.replace(
        '"lockfileVersion": 3,',
        '"lockfileVersion": 2,\\n  "lockfileVersion": 3,',
        1,
    )
    if duplicate_lock_raw == lock_raw:
        raise InputContractError("unable to construct duplicate-key lockfile fixture")
    tests.append(("duplicate_lock_json_key", package_raw, duplicate_lock_raw))

    unsafe_path = copy.deepcopy(lock)
    unsafe_path["packages"]["node_modules/../outside"] = unsafe_path["packages"].pop("node_modules/zod")
    tests.append(("unsafe_package_entry_path", package_raw, json.dumps(unsafe_path)))

    missing_version = copy.deepcopy(lock)
    missing_version["packages"]["node_modules/zod"].pop("version", None)
    tests.append(("missing_package_version", package_raw, json.dumps(missing_version)))

    missing_integrity = copy.deepcopy(lock)
    resolved_key = next(k for k, v in missing_integrity["packages"].items() if k and v.get("resolved"))
    missing_integrity["packages"][resolved_key].pop("integrity", None)
    tests.append(("missing_integrity", package_raw, json.dumps(missing_integrity)))

    weak_integrity = copy.deepcopy(lock)
    weak_integrity["packages"][resolved_key]["integrity"] = "sha1-AbCdEf=="
    tests.append(("weak_integrity_algorithm", package_raw, json.dumps(weak_integrity)))

    malformed_digest = copy.deepcopy(lock)
    malformed_digest["packages"][resolved_key]["integrity"] = "sha512-AAAA=="
    tests.append(("invalid_sha512_digest_length", package_raw, json.dumps(malformed_digest)))

    missing_resolved = copy.deepcopy(lock)
    missing_resolved["packages"][resolved_key].pop("resolved", None)
    tests.append(("missing_resolved_source", package_raw, json.dumps(missing_resolved)))

    bad_scheme = copy.deepcopy(lock)
    bad_scheme["packages"][resolved_key]["resolved"] = (
        bad_scheme["packages"][resolved_key]["resolved"].replace(
            "https://registry.npmjs.org/", "http://registry.npmjs.org/", 1
        )
    )
    tests.append(("non_https_source", package_raw, json.dumps(bad_scheme)))

    credential_url = copy.deepcopy(lock)
    credential_url["packages"][resolved_key]["resolved"] = (
        credential_url["packages"][resolved_key]["resolved"].replace(
            "https://registry.npmjs.org/", "https://attacker@registry.npmjs.org/", 1
        )
    )
    tests.append(("registry_url_credentials", package_raw, json.dumps(credential_url)))

    control_character_url = copy.deepcopy(lock)
    control_character_url["packages"][resolved_key]["resolved"] += chr(10)
    tests.append(("source_url_control_character", package_raw, json.dumps(control_character_url)))

    bad_registry = copy.deepcopy(lock)
    bad_registry["packages"][resolved_key]["resolved"] = (
        bad_registry["packages"][resolved_key]["resolved"].replace(
            "https://registry.npmjs.org/", "https://packages.example.invalid/", 1
        )
    )
    tests.append(("unapproved_registry", package_raw, json.dumps(bad_registry)))

    drifted_manifest = copy.deepcopy(package)
    first_section = next(section for section in DEPENDENCY_SECTIONS if drifted_manifest.get(section))
    first_dep = next(iter(drifted_manifest[first_section]))
    drifted_manifest[first_section][first_dep] = "0.0.0-injected-drift"
    tests.append(("manifest_lock_dependency_drift", json.dumps(drifted_manifest), lock_raw))

    drifted_root = copy.deepcopy(lock)
    drifted_root["packages"][""]["version"] = "0.0.0-injected-drift"
    tests.append(("manifest_lock_identity_drift", package_raw, json.dumps(drifted_root)))

    drifted_peer_dependencies = copy.deepcopy(package)
    drifted_peer_dependencies["peerDependencies"]["svelte"] = "^5.0.0 || ^6.0.0"
    tests.append(("manifest_lock_peer_dependency_drift", json.dumps(drifted_peer_dependencies), lock_raw))

    drifted_peer_metadata = copy.deepcopy(package)
    drifted_peer_metadata["peerDependenciesMeta"]["svelte"]["optional"] = False
    tests.append(("manifest_lock_peer_metadata_drift", json.dumps(drifted_peer_metadata), lock_raw))

    tests.append(("non_v3_lockfile", package_raw, json.dumps({**lock, "lockfileVersion": 2})))

    # A lifecycle metadata mutation must be reviewed, not silently accepted.
    lifecycle_mutation = copy.deepcopy(lock)
    lifecycle_mutation["packages"]["node_modules/esbuild"].pop("hasInstallScript", None)
    tests.append(("install_lifecycle_metadata_drift", package_raw, json.dumps(lifecycle_mutation)))

    unexpected_lifecycle = copy.deepcopy(lock)
    unexpected_lifecycle["packages"]["node_modules/zod"]["hasInstallScript"] = True
    tests.append(("unexpected_install_lifecycle_script", package_raw, json.dumps(unexpected_lifecycle)))

    results: list[dict] = []
    for label, package_input, lock_input in tests:
        try:
            verify_contract(package_input, lock_input)
        except InputContractError as exc:
            results.append({"fixture": label, "status": "PASS_REJECTED", "reason": str(exc)})
        else:
            raise InputContractError(f"negative fixture was incorrectly accepted: {label}")
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Mycelix workspace root (defaults to the parent of this script's scripts directory)",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run fail-closed mutation fixtures before emitting the contract report",
    )
    args = parser.parse_args()

    package_path = args.workspace / "sdk-ts" / "package.json"
    lock_path = args.workspace / "sdk-ts" / "package-lock.json"

    try:
        package_raw = package_path.read_text(encoding="utf-8")
        lock_raw = lock_path.read_text(encoding="utf-8")
        report = verify_contract(package_raw, lock_raw)
        report.update(
            {
                "package_json_sha256": hashlib.sha256(package_raw.encode("utf-8")).hexdigest(),
                "package_lock_sha256": hashlib.sha256(lock_raw.encode("utf-8")).hexdigest(),
            }
        )
        if args.self_test:
            report["negative_fixtures"] = run_self_tests(package_raw, lock_raw)
            report["negative_fixture_count"] = len(report["negative_fixtures"])
            report["negative_fixtures_all_rejected"] = all(
                result["status"] == "PASS_REJECTED" for result in report["negative_fixtures"]
            )
        print(json.dumps(report, sort_keys=True, indent=2))
        return 0
    except (InputContractError, OSError, UnicodeError) as exc:
        print(f"SDK Nix input contract FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

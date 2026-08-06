#!/usr/bin/env python3
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Verify the packed restricted-alpha hApp without Holochain tooling."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import tempfile
from pathlib import Path

from pulse_msgpack import MessagePackError, pack, unpack

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config/alpha-happ-contract.json"
DEFAULT_BUNDLE = ROOT / "holochain/mycelix_pulse_alpha.happ"


class VerificationError(ValueError):
    pass


def bounded_gunzip(payload: bytes, maximum: int, label: str) -> bytes:
    try:
        with gzip.GzipFile(fileobj=__import__("io").BytesIO(payload), mode="rb") as stream:
            output = stream.read(maximum + 1)
    except (OSError, EOFError) as error:
        raise VerificationError(f"{label} is not valid gzip: {error}") from error
    if len(output) > maximum:
        raise VerificationError(f"{label} exceeds uncompressed size limit")
    return output


def object_field(value, field, label):
    if not isinstance(value, dict) or field not in value:
        raise VerificationError(f"{label} is missing {field}")
    return value[field]


def exact_names(items, expected, label):
    if not isinstance(items, list):
        raise VerificationError(f"{label} is not a list")
    names = []
    for item in items:
        if not isinstance(item, dict) or not isinstance(item.get("name"), str):
            raise VerificationError(f"{label} contains an invalid entry")
        names.append(item["name"])
    if len(names) != len(set(names)):
        raise VerificationError(f"{label} contains duplicate names")
    if sorted(names) != sorted(expected):
        raise VerificationError(f"{label} differs: packed={sorted(names)} expected={sorted(expected)}")
    return names


def verify_bundle(path: Path, contract: dict) -> dict:
    limits = contract["limits"]
    compressed = path.read_bytes()
    if len(compressed) > limits["maximum_happ_compressed_bytes"]:
        raise VerificationError("packed hApp exceeds compressed size limit")
    try:
        happ = unpack(bounded_gunzip(compressed, limits["maximum_happ_uncompressed_bytes"], "packed hApp"))
    except MessagePackError as error:
        raise VerificationError(f"packed hApp MessagePack is invalid: {error}") from error

    manifest = object_field(happ, "manifest", "packed hApp")
    resources = object_field(happ, "resources", "packed hApp")
    expected_happ = contract["happ"]
    if manifest.get("manifest_version") != expected_happ["manifest_version"]:
        raise VerificationError("packed hApp manifest version differs")
    if manifest.get("name") != expected_happ["name"]:
        raise VerificationError("packed hApp name differs")
    roles = manifest.get("roles")
    if not isinstance(roles, list) or len(roles) != 1:
        raise VerificationError("packed hApp must contain exactly one role")
    role = roles[0]
    if role.get("name") != expected_happ["role"]:
        raise VerificationError("packed hApp role differs")
    provisioning = role.get("provisioning") or {}
    if provisioning.get("strategy") != expected_happ["provisioning_strategy"]:
        raise VerificationError("packed hApp provisioning strategy differs")
    if provisioning.get("deferred") is not expected_happ["deferred"]:
        raise VerificationError("packed hApp deferred provisioning differs")
    dna_ref = role.get("dna") or {}
    if dna_ref.get("path") != expected_happ["dna_resource"]:
        raise VerificationError("packed hApp DNA resource path differs")
    if dna_ref.get("clone_limit") != expected_happ["clone_limit"]:
        raise VerificationError("packed hApp clone limit differs")
    if not isinstance(resources, dict) or set(resources) != {expected_happ["dna_resource"]}:
        raise VerificationError("packed hApp resources differ from the one-DNA contract")

    dna_compressed = resources[expected_happ["dna_resource"]]
    if not isinstance(dna_compressed, bytes):
        raise VerificationError("packed DNA resource is not binary")
    if len(dna_compressed) > limits["maximum_dna_compressed_bytes"]:
        raise VerificationError("packed DNA exceeds compressed size limit")
    try:
        dna = unpack(bounded_gunzip(dna_compressed, limits["maximum_dna_uncompressed_bytes"], "packed DNA"))
    except MessagePackError as error:
        raise VerificationError(f"packed DNA MessagePack is invalid: {error}") from error

    dna_manifest = object_field(dna, "manifest", "packed DNA")
    dna_resources = object_field(dna, "resources", "packed DNA")
    expected_dna = contract["dna"]
    if dna_manifest.get("manifest_version") != expected_dna["manifest_version"]:
        raise VerificationError("packed DNA manifest version differs")
    if dna_manifest.get("name") != expected_dna["name"]:
        raise VerificationError("packed DNA name differs")
    integrity = dna_manifest.get("integrity") or {}
    coordinator = dna_manifest.get("coordinator") or {}
    if integrity.get("properties") != expected_dna["properties"]:
        raise VerificationError("packed DNA properties differ from the alpha contract")
    integrity_items = integrity.get("zomes")
    coordinator_items = coordinator.get("zomes")
    integrity_names = exact_names(integrity_items, expected_dna["integrity_zomes"], "integrity zomes")
    coordinator_names = exact_names(coordinator_items, expected_dna["coordinator_zomes"], "coordinator zomes")

    resource_paths = []
    for item in [*integrity_items, *coordinator_items]:
        resource = item.get("path")
        if not isinstance(resource, str) or not resource.endswith(".wasm") or "/" in resource or "\\" in resource:
            raise VerificationError(f"zome {item.get('name')!r} has an unsafe WASM resource path")
        resource_paths.append(resource)
    if len(resource_paths) != len(set(resource_paths)):
        raise VerificationError("multiple zomes refer to the same WASM resource")
    if not isinstance(dna_resources, dict) or set(dna_resources) != set(resource_paths):
        raise VerificationError("packed DNA resources do not exactly match declared zome paths")

    wasm = {}
    for resource in sorted(resource_paths):
        payload = dna_resources[resource]
        if not isinstance(payload, bytes) or not payload.startswith(b"\x00asm"):
            raise VerificationError(f"WASM resource is invalid: {resource}")
        if len(payload) > limits["maximum_wasm_bytes"]:
            raise VerificationError(f"WASM resource exceeds size limit: {resource}")
        wasm[resource] = {"bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()}

    return {
        "format": "mycelix-pulse-packed-happ-inspection/v1",
        "bundle_sha256": hashlib.sha256(compressed).hexdigest(),
        "bundle_bytes": len(compressed),
        "happ_name": manifest["name"],
        "role": role["name"],
        "dna_name": dna_manifest["name"],
        "dna_sha256": hashlib.sha256(dna_compressed).hexdigest(),
        "integrity_zomes": sorted(integrity_names),
        "coordinator_zomes": sorted(coordinator_names),
        "wasm": wasm,
    }


def synthetic_bundle(contract: dict, *, omit_coordinator: str | None = None) -> bytes:
    integrity = []
    coordinator = []
    resources = {}
    for name in contract["dna"]["integrity_zomes"]:
        path = f"{name}.wasm"
        integrity.append({"name": name, "path": path, "hash": None, "dependencies": None})
        resources[path] = b"\x00asm" + name.encode()
    for name in contract["dna"]["coordinator_zomes"]:
        if name == omit_coordinator:
            continue
        path = f"{name}.wasm"
        coordinator.append({"name": name, "path": path, "hash": None, "dependencies": []})
        resources[path] = b"\x00asm" + name.encode()
    dna = {
        "manifest": {
            "manifest_version": contract["dna"]["manifest_version"],
            "name": contract["dna"]["name"],
            "integrity": {"network_seed": None, "properties": contract["dna"]["properties"], "zomes": integrity},
            "coordinator": {"zomes": coordinator},
        },
        "resources": resources,
    }
    dna_bytes = gzip.compress(pack(dna), mtime=0)
    happ = {
        "manifest": {
            "manifest_version": contract["happ"]["manifest_version"],
            "name": contract["happ"]["name"],
            "roles": [{
                "name": contract["happ"]["role"],
                "provisioning": {
                    "strategy": contract["happ"]["provisioning_strategy"],
                    "deferred": contract["happ"]["deferred"],
                },
                "dna": {
                    "path": contract["happ"]["dna_resource"],
                    "modifiers": {"network_seed": None, "properties": None},
                    "installed_hash": None,
                    "clone_limit": contract["happ"]["clone_limit"],
                },
            }],
        },
        "resources": {contract["happ"]["dna_resource"]: dna_bytes},
    }
    return gzip.compress(pack(happ), mtime=0)


def self_test(contract: dict) -> None:
    with tempfile.TemporaryDirectory(prefix="pulse-packed-alpha-") as directory:
        good = Path(directory) / "good.happ"
        good.write_bytes(synthetic_bundle(contract))
        report = verify_bundle(good, contract)
        assert report["coordinator_zomes"] == sorted(contract["dna"]["coordinator_zomes"])
        stale = Path(directory) / "stale.happ"
        stale.write_bytes(synthetic_bundle(contract, omit_coordinator="mail_capabilities"))
        try:
            verify_bundle(stale, contract)
        except VerificationError as error:
            assert "coordinator zomes differs" in str(error)
        else:
            raise AssertionError("stale hApp was accepted")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        contract = json.loads(args.contract.read_text())
        if args.self_test:
            self_test(contract)
            print("Packed-alpha verifier self-test passed.")
            return 0
        report = verify_bundle(args.bundle, contract)
        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            print("Packed restricted-alpha hApp verification passed.")
        return 0
    except (OSError, json.JSONDecodeError, VerificationError, MessagePackError) as error:
        print(f"Packed restricted-alpha hApp verification FAILED: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

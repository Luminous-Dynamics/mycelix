#!/usr/bin/env python3
"""Verify that a reviewed driver configuration uses currently authorized interface keys."""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any

from evidence_manifest import canonical_json_bytes, require, require_hash32, sha256_b64
from holochain_real_driver import CONFIG_SCHEMA
from interface_signer_policy import (
    InterfaceKeyPolicyError,
    authorized_certificate,
    verify_interface_key_policy,
)
from release_signer_policy import PolicyError, checked_regular_file, load_json

BINDING_SCHEMA = "mycelix-interface-key-policy-binding-v1"
BINDING_DOMAIN = "MYCELIX:InterfaceKeyPolicyBinding:v1"
MAX_CONFIG_BYTES = 2 * 1024 * 1024
MAX_FLOORS_BYTES = 64 * 1024


def file_sha256(path: Path) -> str:
    return sha256_b64(path.read_bytes())


def load_floors(path: Path, conductor_ids: set[str]) -> dict[str, int]:
    value = load_json(checked_regular_file(path, "interface key epoch floors", MAX_FLOORS_BYTES), "interface key epoch floors")
    require(isinstance(value, dict), "interface key epoch floors must be an object")
    require(set(value) == conductor_ids, "interface key epoch floors must name every configured conductor exactly once")
    output: dict[str, int] = {}
    for conductor_id, epoch in sorted(value.items()):
        require(isinstance(epoch, int) and epoch > 0, f"minimum key epoch for {conductor_id} must be positive")
        output[conductor_id] = epoch
    return output


def verify_binding(
    *,
    driver_config: Path,
    interface_policy: Path,
    interface_policy_signature: Path,
    signer_policy: Path,
    signer_policy_signature: Path,
    policy_root_public_key: Path,
    minimum_signer_policy_version: int,
    minimum_interface_policy_version: int,
    minimum_key_epochs: Path,
    evaluation_time_unix_micros: int,
) -> dict[str, Any]:
    require(isinstance(evaluation_time_unix_micros, int) and evaluation_time_unix_micros > 0, "evaluation time must be positive")
    config_path = checked_regular_file(driver_config, "driver configuration", MAX_CONFIG_BYTES)
    config = load_json(config_path, "driver configuration")
    require(config.get("schema") == CONFIG_SCHEMA, "unsupported driver configuration schema")
    conductors = config.get("conductors")
    require(isinstance(conductors, list) and bool(conductors), "driver configuration has no conductors")
    conductor_ids = {item.get("id") for item in conductors if isinstance(item, dict)}
    require(len(conductor_ids) == len(conductors) and all(isinstance(value, str) and value for value in conductor_ids), "driver conductors are malformed or duplicated")
    floors = load_floors(minimum_key_epochs, conductor_ids)
    policy = verify_interface_key_policy(
        interface_policy,
        interface_policy_signature,
        signer_policy,
        signer_policy_signature,
        policy_root_public_key,
        minimum_signer_policy_version,
        minimum_interface_policy_version,
    )
    require({entry["conductor_id"] for entry in policy["keys"]} == conductor_ids, "interface key policy conductor set differs from driver configuration")
    active = []
    distinct_keys: set[str] = set()
    for conductor in sorted(conductors, key=lambda item: item["id"]):
        conductor_id = conductor["id"]
        certificate = authorized_certificate(
            policy,
            conductor_id,
            conductor.get("agent_pub_key"),
            evaluation_time_unix_micros,
            floors[conductor_id],
        )
        expected = {
            "attestation_signer_pub_key_b64": certificate["attestation_signer_pub_key_b64"],
            "attestation_key_policy_id": policy["policy_id"],
            "attestation_key_policy_version": policy["policy_version"],
            "attestation_key_epoch": certificate["key_epoch"],
            "attestation_key_certificate_hash": certificate["certificate_hash"],
            "attestation_key_valid_from_unix_micros": certificate["valid_from_unix_micros"],
            "attestation_key_valid_until_unix_micros": certificate["valid_until_unix_micros"],
            "attestation_key_revoked_at_unix_micros": certificate["revoked_at_unix_micros"],
            "attestation_key_revocation_mode": certificate["revocation_mode"],
        }
        for field, expected_value in expected.items():
            require(conductor.get(field) == expected_value, f"driver conductor {conductor_id} does not match authorized certificate field {field}")
        key = certificate["attestation_signer_pub_key_b64"]
        require(key not in distinct_keys, "configured conductors share an attestation signing key")
        distinct_keys.add(key)
        active.append({
            "conductor_id": conductor_id,
            "agent_pub_key": conductor["agent_pub_key"],
            "key_epoch": certificate["key_epoch"],
            "certificate_hash": certificate["certificate_hash"],
            "attestation_signer_pub_key_b64": key,
        })
    value = {
        "schema": BINDING_SCHEMA,
        "domain": BINDING_DOMAIN,
        "evaluated_at_unix_micros": evaluation_time_unix_micros,
        "driver_config_file_sha256": file_sha256(config_path),
        "driver_config_schema": config["schema"],
        "interface_key_policy_file_sha256": file_sha256(interface_policy),
        "interface_key_policy_signature_sha256": file_sha256(interface_policy_signature),
        "interface_key_policy_canonical_hash": sha256_b64(canonical_json_bytes(policy)),
        "interface_key_policy_id": policy["policy_id"],
        "interface_key_policy_version": policy["policy_version"],
        "minimum_signer_policy_version": minimum_signer_policy_version,
        "minimum_interface_policy_version": minimum_interface_policy_version,
        "minimum_key_epochs": floors,
        "minimum_key_epochs_file_sha256": file_sha256(minimum_key_epochs),
        "active_certificates": active,
    }
    value["binding_hash"] = sha256_b64(canonical_json_bytes(value))
    return value


def write_output(path: Path, value: dict[str, Any]) -> None:
    require(not path.exists() and not path.is_symlink(), "interface policy binding output already exists")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    require(not temporary.exists() and not temporary.is_symlink(), "temporary interface policy binding path already exists")
    temporary.write_bytes(canonical_json_bytes(value) + b"\n")
    os.chmod(temporary, 0o444)
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--driver-config", type=Path, required=True)
    parser.add_argument("--interface-key-policy", type=Path, required=True)
    parser.add_argument("--interface-key-policy-signature", type=Path, required=True)
    parser.add_argument("--signer-policy", type=Path, required=True)
    parser.add_argument("--signer-policy-signature", type=Path, required=True)
    parser.add_argument("--policy-root-public-key", type=Path, required=True)
    parser.add_argument("--minimum-signer-policy-version", type=int, required=True)
    parser.add_argument("--minimum-interface-policy-version", type=int, required=True)
    parser.add_argument("--minimum-key-epochs", type=Path, required=True)
    parser.add_argument("--evaluation-time-unix-micros", type=int, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        value = verify_binding(
            driver_config=args.driver_config,
            interface_policy=args.interface_key_policy,
            interface_policy_signature=args.interface_key_policy_signature,
            signer_policy=args.signer_policy,
            signer_policy_signature=args.signer_policy_signature,
            policy_root_public_key=args.policy_root_public_key,
            minimum_signer_policy_version=args.minimum_signer_policy_version,
            minimum_interface_policy_version=args.minimum_interface_policy_version,
            minimum_key_epochs=args.minimum_key_epochs,
            evaluation_time_unix_micros=args.evaluation_time_unix_micros,
        )
        if args.output is not None:
            write_output(args.output, value)
        print("interface key policy binding: PASS")
        print(f"policy_id={value['interface_key_policy_id']}")
        print(f"policy_version={value['interface_key_policy_version']}")
        print(f"binding_hash={value['binding_hash']}")
        return 0
    except (InterfaceKeyPolicyError, PolicyError, OSError, ValueError, RuntimeError) as error:
        print(f"interface key policy binding: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

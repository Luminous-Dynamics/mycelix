#!/usr/bin/env python3
"""Root-governed rotation policy for conductor interface-attestation keys."""
from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path
from typing import Any

from evidence_manifest import canonical_json_bytes, manifest_hash, require, require_hash32, require_text, sha256_b64
from release_signer_policy import (
    POLICY_ROLE_DRIVER,
    PolicyError,
    authorized_key,
    checked_regular_file,
    load_json,
    policy_identity,
    private_key_id,
    sign_file,
    verify_policy,
    verify_signature_with_der,
)

INTERFACE_KEY_POLICY_SCHEMA = "mycelix-interface-attestation-key-policy-v2"
INTERFACE_KEY_POLICY_ID_DOMAIN = "MYCELIX:InterfaceAttestationKeyPolicyId:v1"
INTERFACE_KEY_CERTIFICATE_DOMAIN = "MYCELIX:InterfaceAttestationKeyCertificate:v2"
MAX_POLICY_BYTES = 512 * 1024
MAX_SIGNATURE_BYTES = 4096
MAX_KEYS = 32


class InterfaceKeyPolicyError(RuntimeError):
    pass


def require_b64_exact(value: Any, name: str, length: int) -> str:
    require(isinstance(value, str), f"{name} must be base64 text")
    try:
        decoded = base64.b64decode(value, validate=True)
    except Exception as error:
        raise InterfaceKeyPolicyError(f"{name} is not canonical base64: {error}") from error
    require(len(decoded) == length and base64.b64encode(decoded).decode("ascii") == value, f"{name} must decode to exactly {length} bytes")
    return value


def _certificate_payload(policy: dict[str, Any], entry: dict[str, Any]) -> dict[str, Any]:
    # Certificate identities must be computable before the enclosing policy ID
    # so predecessor chains cannot create a policy-id/certificate-hash cycle.
    return {
        "domain": INTERFACE_KEY_CERTIFICATE_DOMAIN,
        "conductor_id": entry["conductor_id"],
        "agent_pub_key": entry["agent_pub_key"],
        "attestation_signer_pub_key_b64": entry["attestation_signer_pub_key_b64"],
        "key_epoch": entry["key_epoch"],
        "valid_from_unix_micros": entry["valid_from_unix_micros"],
        "valid_until_unix_micros": entry["valid_until_unix_micros"],
        "revoked_at_unix_micros": entry["revoked_at_unix_micros"],
        "revocation_mode": entry["revocation_mode"],
        "predecessor_certificate_hash": entry["predecessor_certificate_hash"],
    }


def certificate_hash(policy: dict[str, Any], entry: dict[str, Any]) -> str:
    return sha256_b64(canonical_json_bytes(_certificate_payload(policy, entry)))


def policy_id_payload(policy: dict[str, Any]) -> dict[str, Any]:
    keys = []
    for entry in policy["keys"]:
        keys.append({key: value for key, value in entry.items() if key != "certificate_hash"})
    return {
        "domain": INTERFACE_KEY_POLICY_ID_DOMAIN,
        "policy_version": policy["policy_version"],
        "created_at_unix_micros": policy["created_at_unix_micros"],
        "valid_from_unix_micros": policy["valid_from_unix_micros"],
        "valid_until_unix_micros": policy["valid_until_unix_micros"],
        "signer_policy_file_sha256": policy["signer_policy_file_sha256"],
        "signer_policy_canonical_hash": policy["signer_policy_canonical_hash"],
        "signer_policy_version": policy["signer_policy_version"],
        "driver_review_key_id": policy["driver_review_key_id"],
        "issued_at_unix_micros": policy["issued_at_unix_micros"],
        "keys": keys,
    }


def validate_policy_shape(policy: dict[str, Any]) -> None:
    expected = {
        "schema", "policy_id", "policy_version", "created_at_unix_micros",
        "valid_from_unix_micros", "valid_until_unix_micros",
        "signer_policy_file_sha256", "signer_policy_canonical_hash",
        "signer_policy_version", "driver_review_key_id", "issued_at_unix_micros", "keys",
    }
    require(set(policy) == expected, "interface key policy has missing or unknown fields")
    require(policy.get("schema") == INTERFACE_KEY_POLICY_SCHEMA, "unsupported interface key policy schema")
    version = policy.get("policy_version")
    require(isinstance(version, int) and version > 0, "interface key policy version must be positive")
    created = policy.get("created_at_unix_micros")
    valid_from = policy.get("valid_from_unix_micros")
    valid_until = policy.get("valid_until_unix_micros")
    issued = policy.get("issued_at_unix_micros")
    require(isinstance(created, int) and created > 0, "invalid interface key policy creation time")
    require(isinstance(valid_from, int) and created <= valid_from, "invalid interface key policy validity start")
    require(isinstance(valid_until, int) and valid_until > valid_from, "invalid interface key policy validity end")
    require(isinstance(issued, int) and created <= issued < valid_until, "invalid interface key policy issuance time")
    require_hash32(policy.get("signer_policy_file_sha256"), "signer_policy_file_sha256")
    require_hash32(policy.get("signer_policy_canonical_hash"), "signer_policy_canonical_hash")
    require_hash32(policy.get("driver_review_key_id"), "driver_review_key_id")
    signer_version = policy.get("signer_policy_version")
    require(isinstance(signer_version, int) and signer_version > 0, "signer_policy_version must be positive")
    keys = policy.get("keys")
    require(isinstance(keys, list) and 1 <= len(keys) <= MAX_KEYS, "interface key policy requires 1-32 keys")
    public_keys: set[str] = set()
    certificates: set[str] = set()
    conductor_epochs: set[tuple[str, int]] = set()
    grouped: dict[str, list[dict[str, Any]]] = {}
    for index, entry in enumerate(keys):
        require(isinstance(entry, dict), f"keys[{index}] must be an object")
        expected_entry = {
            "conductor_id", "agent_pub_key", "attestation_signer_pub_key_b64", "key_epoch",
            "valid_from_unix_micros", "valid_until_unix_micros", "revoked_at_unix_micros",
            "revocation_mode", "predecessor_certificate_hash", "certificate_hash",
        }
        require(set(entry) == expected_entry, f"keys[{index}] has missing or unknown fields")
        conductor_id = require_text(entry.get("conductor_id"), f"keys[{index}].conductor_id", 128)
        require_text(entry.get("agent_pub_key"), f"keys[{index}].agent_pub_key")
        public_key = require_b64_exact(entry.get("attestation_signer_pub_key_b64"), f"keys[{index}].attestation_signer_pub_key_b64", 32)
        require(public_key not in public_keys, "interface attestation public keys must be unique")
        public_keys.add(public_key)
        epoch = entry.get("key_epoch")
        require(isinstance(epoch, int) and epoch > 0, f"keys[{index}].key_epoch must be positive")
        require((conductor_id, epoch) not in conductor_epochs, "interface key policy repeats a conductor epoch")
        conductor_epochs.add((conductor_id, epoch))
        key_from = entry.get("valid_from_unix_micros")
        key_until = entry.get("valid_until_unix_micros")
        require(isinstance(key_from, int) and valid_from <= key_from < valid_until, f"keys[{index}] validity start is outside policy")
        require(isinstance(key_until, int) and key_from < key_until <= valid_until, f"keys[{index}] validity end is outside policy")
        revoked = entry.get("revoked_at_unix_micros")
        require(revoked is None or (isinstance(revoked, int) and key_from <= revoked < key_until), f"keys[{index}] has invalid revocation time")
        mode = entry.get("revocation_mode")
        require(mode in {"prospective", "retroactive"}, f"keys[{index}] has invalid revocation mode")
        require(revoked is not None or mode == "prospective", f"keys[{index}] cannot be retroactive without revocation")
        predecessor = entry.get("predecessor_certificate_hash")
        if epoch == 1:
            require(predecessor is None, f"keys[{index}] epoch one may not name a predecessor")
        else:
            require_hash32(predecessor, f"keys[{index}].predecessor_certificate_hash")
        cert = require_hash32(entry.get("certificate_hash"), f"keys[{index}].certificate_hash")
        require(cert == certificate_hash(policy, entry), f"keys[{index}] certificate hash mismatch")
        require(cert not in certificates, "interface key policy repeats a certificate")
        certificates.add(cert)
        grouped.setdefault(conductor_id, []).append(entry)

    for conductor_id, lineage in grouped.items():
        lineage.sort(key=lambda item: item["key_epoch"])
        require(lineage[0]["key_epoch"] == 1, f"{conductor_id} key lineage must begin at epoch one")
        agent = lineage[0]["agent_pub_key"]
        for position, entry in enumerate(lineage):
            require(entry["agent_pub_key"] == agent, f"{conductor_id} key lineage changes agent identity")
            require(entry["key_epoch"] == position + 1, f"{conductor_id} key epochs must be contiguous")
            if position == 0:
                continue
            previous = lineage[position - 1]
            require(entry["predecessor_certificate_hash"] == previous["certificate_hash"], f"{conductor_id} predecessor certificate mismatch")
            previous_effective_end = previous["valid_until_unix_micros"]
            if previous["revoked_at_unix_micros"] is not None and previous["revocation_mode"] == "prospective":
                previous_effective_end = min(previous_effective_end, previous["revoked_at_unix_micros"])
            if previous["revocation_mode"] == "retroactive":
                previous_effective_end = previous["valid_from_unix_micros"]
            require(previous_effective_end <= entry["valid_from_unix_micros"], f"{conductor_id} key epochs overlap")

    expected_policy_id = sha256_b64(canonical_json_bytes(policy_id_payload(policy)))
    require(require_hash32(policy.get("policy_id"), "policy_id") == expected_policy_id, "interface key policy id mismatch")

def verify_interface_key_policy(
    policy_path: Path,
    signature_path: Path,
    signer_policy_path: Path,
    signer_policy_signature: Path,
    policy_root_public_key: Path,
    minimum_signer_policy_version: int,
    minimum_interface_policy_version: int,
) -> dict[str, Any]:
    policy_file = checked_regular_file(policy_path, "interface key policy", MAX_POLICY_BYTES)
    signature_file = checked_regular_file(signature_path, "interface key policy signature", MAX_SIGNATURE_BYTES)
    signer_policy = verify_policy(signer_policy_path, signer_policy_signature, policy_root_public_key)
    require(signer_policy["policy_version"] >= minimum_signer_policy_version, "signer policy version is below protected minimum")
    policy = load_json(policy_file, "interface key policy")
    validate_policy_shape(policy)
    require(policy["policy_version"] >= minimum_interface_policy_version, "interface key policy version is below protected minimum")
    signer_file_hash, signer_canonical_hash = policy_identity(signer_policy_path, signer_policy)
    require(policy["signer_policy_file_sha256"] == signer_file_hash, "interface key policy names another signer-policy file")
    require(policy["signer_policy_canonical_hash"] == signer_canonical_hash, "interface key policy names another signer policy")
    require(policy["signer_policy_version"] == signer_policy["policy_version"], "interface key policy signer-policy version mismatch")
    der = authorized_key(signer_policy, POLICY_ROLE_DRIVER, policy["driver_review_key_id"], policy["issued_at_unix_micros"])
    verify_signature_with_der(policy_file, signature_file, der)
    return policy


def authorized_certificate(policy: dict[str, Any], conductor_id: str, agent_pub_key: str, at_unix_micros: int, minimum_epoch: int) -> dict[str, Any]:
    require(isinstance(at_unix_micros, int) and at_unix_micros > 0, "certificate evaluation time must be positive")
    require(isinstance(minimum_epoch, int) and minimum_epoch > 0, "minimum key epoch must be positive")
    require(policy["valid_from_unix_micros"] <= at_unix_micros < policy["valid_until_unix_micros"], "interface key policy is not valid at evaluation time")
    lineage = [entry for entry in policy["keys"] if entry["conductor_id"] == conductor_id]
    require(bool(lineage), "conductor is absent from interface key policy")
    require(all(entry["agent_pub_key"] == agent_pub_key for entry in lineage), "interface key certificate names another conductor agent")
    eligible = []
    for entry in lineage:
        if entry["key_epoch"] < minimum_epoch:
            continue
        if not (entry["valid_from_unix_micros"] <= at_unix_micros < entry["valid_until_unix_micros"]):
            continue
        revoked = entry["revoked_at_unix_micros"]
        if revoked is not None and (entry["revocation_mode"] == "retroactive" or at_unix_micros >= revoked):
            continue
        eligible.append(entry)
    if len(eligible) != 1:
        raise InterfaceKeyPolicyError("interface key policy does not authorize exactly one certificate at evaluation time")
    return eligible[0]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--signature", type=Path, required=True)
    parser.add_argument("--signer-policy", type=Path, required=True)
    parser.add_argument("--signer-policy-signature", type=Path, required=True)
    parser.add_argument("--policy-root-public-key", type=Path, required=True)
    parser.add_argument("--minimum-signer-policy-version", type=int, required=True)
    parser.add_argument("--minimum-interface-policy-version", type=int, required=True)
    parser.add_argument("--conductor-id")
    parser.add_argument("--agent-pub-key")
    parser.add_argument("--at-unix-micros", type=int)
    parser.add_argument("--minimum-key-epoch", type=int)
    args = parser.parse_args()
    try:
        policy = verify_interface_key_policy(
            args.policy, args.signature, args.signer_policy, args.signer_policy_signature,
            args.policy_root_public_key, args.minimum_signer_policy_version,
            args.minimum_interface_policy_version,
        )
        optional = [args.conductor_id, args.agent_pub_key, args.at_unix_micros, args.minimum_key_epoch]
        if any(value is not None for value in optional):
            require(all(value is not None for value in optional), "conductor id, agent key, evaluation time, and minimum epoch must be supplied together")
            authorized_certificate(policy, args.conductor_id, args.agent_pub_key, args.at_unix_micros, args.minimum_key_epoch)
        print("interface attestation key policy: PASS")
        print(f"policy_version={policy['policy_version']}")
        print(f"policy_id={policy['policy_id']}")
        print(f"key_count={len(policy['keys'])}")
        return 0
    except (InterfaceKeyPolicyError, PolicyError, OSError, ValueError) as error:
        print(f"interface attestation key policy: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

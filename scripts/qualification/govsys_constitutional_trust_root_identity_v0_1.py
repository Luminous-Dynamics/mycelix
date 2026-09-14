#!/usr/bin/env python3
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Independent GOVSYS-003A constitutional trust-root identity oracle."""

from __future__ import annotations

import copy
import hashlib
import json
import re
import struct
from pathlib import Path
from typing import Any

PROTOCOL_VERSION = "mycelix-constitutional-trust-root-v0.1"
IDENTITY_PROFILE = "mycelix-constitutional-trust-root-v1-sha256-framed-semantic"
DOMAIN = b"mycelix/public-institution/constitutional-trust-root/v1"
BOOTSTRAP_MODES = {
    "pinned-constitutional-commitment",
    "genesis-governance-decision",
    "external-institutional-credential",
}
ROTATION_MODES = {"immutable", "predecessor-authorized"}
MAX_ID_BYTES = 512
MAX_PROFILE_BYTES = 256
MAX_NAMESPACE_BYTES = 1024
HEX_32 = re.compile(r"^[0-9a-fA-F]{64}$")
VECTOR_PATH = Path(__file__).with_name("govsys_003a_root_identity_vector_v1.json")


class ContractError(ValueError):
    """Input violates the GOVSYS-003A semantic contract."""


def _u64(value: Any, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0 or value > 0xFFFFFFFFFFFFFFFF:
        raise ContractError(f"{field} must be a u64")
    return value


def _text(value: Any, field: str, max_bytes: int) -> str:
    if not isinstance(value, str):
        raise ContractError(f"{field} must be text")
    raw = value.encode("utf-8")
    if not value.strip() or len(raw) > max_bytes:
        raise ContractError(f"invalid {field}")
    if any(byte < 0x20 or byte == 0x7F for byte in raw):
        raise ContractError(f"control byte in {field}")
    return value


def _digest(value: Any, field: str) -> bytes:
    if not isinstance(value, str) or HEX_32.fullmatch(value) is None:
        raise ContractError(f"{field} must be exactly 32 bytes of hex")
    raw = bytes.fromhex(value)
    if raw == bytes(32):
        raise ContractError(f"{field} must not be zero")
    return raw


def _optional_digest(value: Any, field: str) -> bytes | None:
    if value is None:
        return None
    return _digest(value, field)


def _optional_text(value: Any, field: str, max_bytes: int) -> str | None:
    if value is None:
        return None
    return _text(value, field, max_bytes)


def _rulebook(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != {"id", "version", "digest_hex"}:
        raise ContractError(f"invalid {field} shape")
    _text(value["id"], f"{field}.id", MAX_ID_BYTES)
    _text(value["version"], f"{field}.version", 128)
    _digest(value["digest_hex"], f"{field}.digest_hex")
    return value


def _string_set(value: Any, field: str, max_bytes: int) -> list[str]:
    if not isinstance(value, list):
        raise ContractError(f"{field} must be a list")
    checked = [_text(member, field, max_bytes) for member in value]
    if len(set(checked)) != len(checked):
        raise ContractError(f"duplicate member in {field}")
    return sorted(checked, key=lambda member: member.encode("utf-8"))


def validate_root(root: Any) -> dict[str, Any]:
    required = {
        "protocol_version",
        "institution_id",
        "jurisdiction_id",
        "constitutional_rulebook",
        "generation",
        "predecessor_root_digest_hex",
        "bootstrap_mode",
        "bootstrap_profile",
        "authorized_policy_profiles",
        "authorized_policy_registry_namespaces",
        "provider_authority_institution_id",
        "provider_authority_jurisdiction_id",
        "provider_authority_rulebook",
        "allowed_provider_capabilities",
        "valid_from_ms",
        "expires_at_ms",
        "rotation_mode",
        "rotation_profile",
    }
    if not isinstance(root, dict) or set(root) != required:
        raise ContractError("root shape must contain exactly the GOVSYS-003A fields")

    if root["protocol_version"] != PROTOCOL_VERSION:
        raise ContractError("wrong protocol version")

    _text(root["institution_id"], "institution_id", MAX_ID_BYTES)
    _optional_text(root["jurisdiction_id"], "jurisdiction_id", MAX_ID_BYTES)
    _rulebook(root["constitutional_rulebook"], "constitutional_rulebook")

    generation = _u64(root["generation"], "generation")
    predecessor = _optional_digest(root["predecessor_root_digest_hex"], "predecessor_root_digest_hex")
    if generation == 0 and predecessor is not None:
        raise ContractError("generation zero must not name a predecessor")
    if generation > 0 and predecessor is None:
        raise ContractError("successor generation requires a predecessor digest")

    if root["bootstrap_mode"] not in BOOTSTRAP_MODES:
        raise ContractError("unknown bootstrap mode")
    _text(root["bootstrap_profile"], "bootstrap_profile", MAX_PROFILE_BYTES)

    _string_set(root["authorized_policy_profiles"], "authorized_policy_profiles", MAX_PROFILE_BYTES)
    _string_set(
        root["authorized_policy_registry_namespaces"],
        "authorized_policy_registry_namespaces",
        MAX_NAMESPACE_BYTES,
    )

    _text(root["provider_authority_institution_id"], "provider_authority_institution_id", MAX_ID_BYTES)
    _optional_text(
        root["provider_authority_jurisdiction_id"],
        "provider_authority_jurisdiction_id",
        MAX_ID_BYTES,
    )
    _rulebook(root["provider_authority_rulebook"], "provider_authority_rulebook")
    _string_set(root["allowed_provider_capabilities"], "allowed_provider_capabilities", MAX_ID_BYTES)

    valid_from = _u64(root["valid_from_ms"], "valid_from_ms")
    expires = root["expires_at_ms"]
    if expires is not None:
        expires = _u64(expires, "expires_at_ms")
        if expires <= valid_from:
            raise ContractError("root expiry must be after valid_from_ms")

    rotation_mode = root["rotation_mode"]
    if rotation_mode not in ROTATION_MODES:
        raise ContractError("unknown rotation mode")
    rotation_profile = root["rotation_profile"]
    if rotation_mode == "immutable":
        if rotation_profile is not None:
            raise ContractError("immutable root must not carry a rotation profile")
    else:
        _text(rotation_profile, "rotation_profile", MAX_PROFILE_BYTES)

    return root


def _frame(raw: bytes) -> bytes:
    return struct.pack("<Q", len(raw)) + raw


def _frame_text(value: str) -> bytes:
    return _frame(value.encode("utf-8"))


def _frame_u64(value: int) -> bytes:
    return _frame(struct.pack("<Q", value))


def _frame_optional_text(value: str | None) -> bytes:
    if value is None:
        return _frame(b"\x00")
    return _frame(b"\x01") + _frame_text(value)


def _frame_optional_digest(value: str | None) -> bytes:
    if value is None:
        return _frame(b"\x00")
    return _frame(b"\x01") + _frame(bytes.fromhex(value))


def _frame_optional_u64(value: int | None) -> bytes:
    if value is None:
        return _frame(b"\x00")
    return _frame(b"\x01") + _frame_u64(value)


def _frame_rulebook(value: dict[str, Any]) -> bytes:
    return (
        _frame_text(value["id"])
        + _frame_text(value["version"])
        + _frame(bytes.fromhex(value["digest_hex"]))
    )


def _frame_set(values: list[str]) -> bytes:
    ordered = sorted(values, key=lambda member: member.encode("utf-8"))
    output = bytearray(_frame_u64(len(ordered)))
    for member in ordered:
        output += _frame_text(member)
    return bytes(output)


def canonical_bytes(root: dict[str, Any]) -> bytes:
    validate_root(root)
    output = bytearray(DOMAIN)
    output += _frame_text(IDENTITY_PROFILE)
    output += _frame_text(root["protocol_version"])
    output += _frame_text(root["institution_id"])
    output += _frame_optional_text(root["jurisdiction_id"])
    output += _frame_rulebook(root["constitutional_rulebook"])
    output += _frame_u64(root["generation"])
    output += _frame_optional_digest(root["predecessor_root_digest_hex"])
    output += _frame_text(root["bootstrap_mode"])
    output += _frame_text(root["bootstrap_profile"])
    output += _frame_set(root["authorized_policy_profiles"])
    output += _frame_set(root["authorized_policy_registry_namespaces"])
    output += _frame_text(root["provider_authority_institution_id"])
    output += _frame_optional_text(root["provider_authority_jurisdiction_id"])
    output += _frame_rulebook(root["provider_authority_rulebook"])
    output += _frame_set(root["allowed_provider_capabilities"])
    output += _frame_u64(root["valid_from_ms"])
    output += _frame_optional_u64(root["expires_at_ms"])
    output += _frame_text(root["rotation_mode"])
    output += _frame_optional_text(root["rotation_profile"])
    return bytes(output)


def identity_hex(root: dict[str, Any]) -> str:
    return hashlib.sha256(canonical_bytes(root)).hexdigest()


def _expect_error(root: dict[str, Any]) -> None:
    try:
        identity_hex(root)
    except ContractError:
        return
    raise AssertionError("expected GOVSYS-003A contract rejection")


def self_test() -> None:
    vector = json.loads(VECTOR_PATH.read_text(encoding="utf-8"))
    assert vector["profile"] == IDENTITY_PROFILE
    root = vector["root"]
    expected = vector["expected_digest_hex"]
    actual = identity_hex(root)
    assert actual == expected, (actual, expected)

    reordered = copy.deepcopy(root)
    reordered["authorized_policy_profiles"].reverse()
    reordered["authorized_policy_registry_namespaces"].reverse()
    reordered["allowed_provider_capabilities"].reverse()
    assert identity_hex(reordered) == expected

    duplicate = copy.deepcopy(root)
    duplicate["allowed_provider_capabilities"].append(duplicate["allowed_provider_capabilities"][0])
    _expect_error(duplicate)

    wrong_institution = copy.deepcopy(root)
    wrong_institution["institution_id"] = "institution:other-city"
    assert identity_hex(wrong_institution) != expected

    wrong_rulebook = copy.deepcopy(root)
    wrong_rulebook["constitutional_rulebook"]["digest_hex"] = "33" * 32
    assert identity_hex(wrong_rulebook) != expected

    wrong_provider_scope = copy.deepcopy(root)
    wrong_provider_scope["provider_authority_rulebook"]["digest_hex"] = "44" * 32
    assert identity_hex(wrong_provider_scope) != expected

    wrong_bootstrap = copy.deepcopy(root)
    wrong_bootstrap["bootstrap_mode"] = "genesis-governance-decision"
    assert identity_hex(wrong_bootstrap) != expected

    genesis_with_predecessor = copy.deepcopy(root)
    genesis_with_predecessor["predecessor_root_digest_hex"] = "55" * 32
    _expect_error(genesis_with_predecessor)

    successor_without_predecessor = copy.deepcopy(root)
    successor_without_predecessor["generation"] = 1
    _expect_error(successor_without_predecessor)

    successor = copy.deepcopy(root)
    successor["generation"] = 1
    successor["predecessor_root_digest_hex"] = expected
    assert identity_hex(successor) != expected

    zero_predecessor = copy.deepcopy(successor)
    zero_predecessor["predecessor_root_digest_hex"] = "00" * 32
    _expect_error(zero_predecessor)

    expired = copy.deepcopy(root)
    expired["expires_at_ms"] = expired["valid_from_ms"]
    _expect_error(expired)

    immutable_with_profile = copy.deepcopy(root)
    immutable_with_profile["rotation_mode"] = "immutable"
    _expect_error(immutable_with_profile)

    rotatable_without_profile = copy.deepcopy(root)
    rotatable_without_profile["rotation_profile"] = None
    _expect_error(rotatable_without_profile)

    zero_rulebook = copy.deepcopy(root)
    zero_rulebook["constitutional_rulebook"]["digest_hex"] = "00" * 32
    _expect_error(zero_rulebook)

    control_identifier = copy.deepcopy(root)
    control_identifier["institution_id"] = "institution:city\nsmuggled"
    _expect_error(control_identifier)

    print(f"GOVSYS-003A PASS: {actual}")


if __name__ == "__main__":
    self_test()

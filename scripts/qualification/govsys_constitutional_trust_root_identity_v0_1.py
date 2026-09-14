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

SOURCE_DESCRIPTOR_PROFILE = (
    "mycelix-constitutional-root-source-descriptor-v1-sha256-framed-semantic"
)
SOURCE_DESCRIPTOR_DOMAIN = (
    b"mycelix/public-institution/constitutional-root-source-descriptor/v1"
)
ROTATION_AUTHORITY_PROFILE = (
    "mycelix-constitutional-root-rotation-authority-v1-sha256-framed-semantic"
)
ROTATION_AUTHORITY_DOMAIN = (
    b"mycelix/public-institution/constitutional-root-rotation-authority/v1"
)

BOOTSTRAP_MODES = {
    "pinned-constitutional-commitment",
    "genesis-governance-decision",
    "external-institutional-credential",
}
ROTATION_MODES = {"immutable", "predecessor-authorized"}
MAX_ID_BYTES = 512
MAX_PROFILE_BYTES = 256
MAX_NAMESPACE_BYTES = 1024
MAX_RULEBOOK_VERSION_BYTES = 128
MAX_AUTHORIZED_POLICY_SCOPES = 1024
HEX_32 = re.compile(r"^[0-9a-fA-F]{64}$")
VECTOR_PATH = Path(__file__).with_name("govsys_003a_root_identity_vector_v1.json")


class ContractError(ValueError):
    """Input violates the GOVSYS-003A semantic contract."""


def _u64(value: Any, field: str) -> int:
    if (
        not isinstance(value, int)
        or isinstance(value, bool)
        or value < 0
        or value > 0xFFFFFFFFFFFFFFFF
    ):
        raise ContractError(f"{field} must be a u64")
    return value


def _text(value: Any, field: str, max_bytes: int) -> str:
    if not isinstance(value, str):
        raise ContractError(f"{field} must be text")
    raw = value.encode("utf-8")
    if not raw or len(raw) > max_bytes:
        raise ContractError(f"invalid {field}")
    if raw[0] == 0x20 or raw[-1] == 0x20:
        raise ContractError(f"leading/trailing ASCII space in {field}")
    if any(byte < 0x20 or byte == 0x7F for byte in raw):
        raise ContractError(f"ASCII control byte in {field}")
    return value


def _optional_text(value: Any, field: str, max_bytes: int) -> str | None:
    if value is None:
        return None
    return _text(value, field, max_bytes)


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


def _rulebook(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != {"id", "version", "digest_hex"}:
        raise ContractError(f"invalid {field} shape")
    _text(value["id"], f"{field}.id", MAX_ID_BYTES)
    _text(value["version"], f"{field}.version", MAX_RULEBOOK_VERSION_BYTES)
    _digest(value["digest_hex"], f"{field}.digest_hex")
    return value


def _policy_scope(value: Any, field: str) -> dict[str, Any]:
    required = {
        "policy_identity_profile",
        "policy_registry_namespace",
        "provider_authority_institution_id",
        "provider_authority_jurisdiction_id",
        "provider_authority_rulebook",
        "required_provider_capability",
    }
    if not isinstance(value, dict) or set(value) != required:
        raise ContractError(f"invalid {field} shape")

    profile = _text(
        value["policy_identity_profile"],
        f"{field}.policy_identity_profile",
        MAX_PROFILE_BYTES,
    )
    if profile == IDENTITY_PROFILE:
        raise ContractError(
            "constitutional root profile cannot authorize itself as policy currentness"
        )
    _text(
        value["policy_registry_namespace"],
        f"{field}.policy_registry_namespace",
        MAX_NAMESPACE_BYTES,
    )
    _text(
        value["provider_authority_institution_id"],
        f"{field}.provider_authority_institution_id",
        MAX_ID_BYTES,
    )
    _optional_text(
        value["provider_authority_jurisdiction_id"],
        f"{field}.provider_authority_jurisdiction_id",
        MAX_ID_BYTES,
    )
    _rulebook(value["provider_authority_rulebook"], f"{field}.provider_authority_rulebook")
    _text(
        value["required_provider_capability"],
        f"{field}.required_provider_capability",
        MAX_ID_BYTES,
    )
    return value


def _scope_key(scope: dict[str, Any]) -> tuple[bytes, bytes]:
    return (
        scope["policy_identity_profile"].encode("utf-8"),
        scope["policy_registry_namespace"].encode("utf-8"),
    )


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
        "authoritative_root_source_ref",
        "root_coverage_profile",
        "root_source_verification_profile",
        "root_source_anchor_digest_hex",
        "authorized_policy_scopes",
        "valid_from_ms",
        "expires_at_ms",
        "rotation_mode",
        "rotation_profile",
        "rotation_authority_anchor_digest_hex",
    }
    if not isinstance(root, dict) or set(root) != required:
        raise ContractError("root shape must contain exactly the GOVSYS-003A fields")
    if root["protocol_version"] != PROTOCOL_VERSION:
        raise ContractError("wrong protocol version")

    _text(root["institution_id"], "institution_id", MAX_ID_BYTES)
    _optional_text(root["jurisdiction_id"], "jurisdiction_id", MAX_ID_BYTES)
    _rulebook(root["constitutional_rulebook"], "constitutional_rulebook")

    generation = _u64(root["generation"], "generation")
    predecessor = _optional_digest(
        root["predecessor_root_digest_hex"], "predecessor_root_digest_hex"
    )
    if generation == 0 and predecessor is not None:
        raise ContractError("generation zero must not name a predecessor")
    if generation > 0 and predecessor is None:
        raise ContractError("successor generation requires a predecessor digest")

    if root["bootstrap_mode"] not in BOOTSTRAP_MODES:
        raise ContractError("unknown bootstrap mode")
    _text(root["bootstrap_profile"], "bootstrap_profile", MAX_PROFILE_BYTES)
    _text(
        root["authoritative_root_source_ref"],
        "authoritative_root_source_ref",
        MAX_NAMESPACE_BYTES,
    )
    _text(root["root_coverage_profile"], "root_coverage_profile", MAX_PROFILE_BYTES)
    _text(
        root["root_source_verification_profile"],
        "root_source_verification_profile",
        MAX_PROFILE_BYTES,
    )
    _digest(root["root_source_anchor_digest_hex"], "root_source_anchor_digest_hex")

    scopes = root["authorized_policy_scopes"]
    if not isinstance(scopes, list):
        raise ContractError("authorized_policy_scopes must be a list")
    if len(scopes) > MAX_AUTHORIZED_POLICY_SCOPES:
        raise ContractError("too many authorized policy scopes")

    scope_keys: set[tuple[bytes, bytes]] = set()
    encoded_scopes: list[bytes] = []
    for index, scope in enumerate(scopes):
        _policy_scope(scope, f"authorized_policy_scopes[{index}]")
        key = _scope_key(scope)
        if key in scope_keys:
            raise ContractError(
                "duplicate policy profile/namespace key requires an explicit multi-provider profile"
            )
        scope_keys.add(key)
        encoded_scopes.append(_scope_bytes(scope))
    if len(set(encoded_scopes)) != len(encoded_scopes):
        raise ContractError("duplicate authorized policy scope")

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
    rotation_anchor = root["rotation_authority_anchor_digest_hex"]
    if rotation_mode == "immutable":
        if rotation_profile is not None or rotation_anchor is not None:
            raise ContractError(
                "immutable root must not carry rotation profile or authority anchor"
            )
    else:
        _text(rotation_profile, "rotation_profile", MAX_PROFILE_BYTES)
        _digest(rotation_anchor, "rotation_authority_anchor_digest_hex")

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


def _scope_bytes(scope: dict[str, Any]) -> bytes:
    return (
        _frame_text(scope["policy_identity_profile"])
        + _frame_text(scope["policy_registry_namespace"])
        + _frame_text(scope["provider_authority_institution_id"])
        + _frame_optional_text(scope["provider_authority_jurisdiction_id"])
        + _frame_rulebook(scope["provider_authority_rulebook"])
        + _frame_text(scope["required_provider_capability"])
    )


def _frame_scope_set(scopes: list[dict[str, Any]]) -> bytes:
    encoded = sorted(_scope_bytes(scope) for scope in scopes)
    if len(set(encoded)) != len(encoded):
        raise ContractError("duplicate authorized policy scope")
    return _frame_u64(len(encoded)) + b"".join(encoded)


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
    output += _frame_text(root["authoritative_root_source_ref"])
    output += _frame_text(root["root_coverage_profile"])
    output += _frame_text(root["root_source_verification_profile"])
    output += _frame(bytes.fromhex(root["root_source_anchor_digest_hex"]))
    output += _frame_scope_set(root["authorized_policy_scopes"])
    output += _frame_u64(root["valid_from_ms"])
    output += _frame_optional_u64(root["expires_at_ms"])
    output += _frame_text(root["rotation_mode"])
    output += _frame_optional_text(root["rotation_profile"])
    output += _frame_optional_digest(root["rotation_authority_anchor_digest_hex"])
    return bytes(output)


def identity_hex(root: dict[str, Any]) -> str:
    return hashlib.sha256(canonical_bytes(root)).hexdigest()


def source_descriptor_hex(root: dict[str, Any]) -> str:
    validate_root(root)
    transcript = bytearray(SOURCE_DESCRIPTOR_DOMAIN)
    transcript += _frame_text(SOURCE_DESCRIPTOR_PROFILE)
    transcript += _frame_text(root["authoritative_root_source_ref"])
    transcript += _frame_text(root["root_coverage_profile"])
    transcript += _frame_text(root["root_source_verification_profile"])
    transcript += _frame(bytes.fromhex(root["root_source_anchor_digest_hex"]))
    return hashlib.sha256(transcript).hexdigest()


def rotation_authority_hex(root: dict[str, Any]) -> str | None:
    validate_root(root)
    if root["rotation_mode"] == "immutable":
        return None
    transcript = bytearray(ROTATION_AUTHORITY_DOMAIN)
    transcript += _frame_text(ROTATION_AUTHORITY_PROFILE)
    transcript += _frame_text(root["rotation_profile"])
    transcript += _frame(bytes.fromhex(root["rotation_authority_anchor_digest_hex"]))
    return hashlib.sha256(transcript).hexdigest()


def _expect_error(root: dict[str, Any]) -> None:
    try:
        identity_hex(root)
    except ContractError:
        return
    raise AssertionError("expected GOVSYS-003A contract rejection")


def self_test() -> None:
    vector = json.loads(VECTOR_PATH.read_text(encoding="utf-8"))
    if not isinstance(vector, dict) or set(vector) != {
        "profile",
        "expected_digest_hex",
        "root",
    }:
        raise AssertionError("golden vector wrapper must have exact shape")
    assert vector["profile"] == IDENTITY_PROFILE
    expected = _digest(vector["expected_digest_hex"], "expected_digest_hex")
    root = vector["root"]
    actual = bytes.fromhex(identity_hex(root))
    assert actual == expected, (actual.hex(), expected.hex())

    source_expected = "f97a96e20ce6dd0c86c67e1590252abc678dfd6401128ab44ced4a66ec088126"
    rotation_expected = "cf86718a53410b1e5e38cf5c552b3e4448772da97533fbd08c7b9bbfbee909ae"
    assert source_descriptor_hex(root) == source_expected
    assert rotation_authority_hex(root) == rotation_expected

    reordered = copy.deepcopy(root)
    reordered["authorized_policy_scopes"].reverse()
    assert bytes.fromhex(identity_hex(reordered)) == expected

    duplicate = copy.deepcopy(root)
    duplicate["authorized_policy_scopes"].append(
        copy.deepcopy(duplicate["authorized_policy_scopes"][0])
    )
    _expect_error(duplicate)

    ambiguous_provider = copy.deepcopy(root)
    alternate = copy.deepcopy(ambiguous_provider["authorized_policy_scopes"][0])
    alternate["provider_authority_institution_id"] = "institution:alternate-clerk"
    ambiguous_provider["authorized_policy_scopes"].append(alternate)
    _expect_error(ambiguous_provider)

    self_authorizing = copy.deepcopy(root)
    self_authorizing["authorized_policy_scopes"][0]["policy_identity_profile"] = IDENTITY_PROFILE
    _expect_error(self_authorizing)

    too_many_scopes = copy.deepcopy(root)
    too_many_scopes["authorized_policy_scopes"] = []
    prototype = root["authorized_policy_scopes"][0]
    for index in range(MAX_AUTHORIZED_POLICY_SCOPES + 1):
        scope = copy.deepcopy(prototype)
        scope["policy_registry_namespace"] = f"registry:test:{index}"
        too_many_scopes["authorized_policy_scopes"].append(scope)
    _expect_error(too_many_scopes)

    for mutate in (
        lambda r: r.update(institution_id="institution:other-city"),
        lambda r: r["constitutional_rulebook"].update(digest_hex="55" * 32),
        lambda r: r["authorized_policy_scopes"][0].update(
            policy_identity_profile="mycelix-other-policy-v1"
        ),
        lambda r: r["authorized_policy_scopes"][0].update(
            policy_registry_namespace="registry:other-policy:example-city"
        ),
        lambda r: r["authorized_policy_scopes"][0].update(
            required_provider_capability="administration.other-policy.currentness.attest"
        ),
        lambda r: r.update(bootstrap_mode="genesis-governance-decision"),
        lambda r: r.update(authoritative_root_source_ref="registry:constitutional-root:other"),
        lambda r: r.update(root_coverage_profile="mycelix-other-root-coverage-v1"),
        lambda r: r.update(
            root_source_verification_profile="mycelix-other-root-source-verification-v1"
        ),
        lambda r: r.update(root_source_anchor_digest_hex="66" * 32),
        lambda r: r.update(rotation_profile="constitutional-root-rotation-v2"),
        lambda r: r.update(rotation_authority_anchor_digest_hex="77" * 32),
    ):
        changed = copy.deepcopy(root)
        mutate(changed)
        assert bytes.fromhex(identity_hex(changed)) != expected

    crossed = copy.deepcopy(root)
    a, b = crossed["authorized_policy_scopes"]
    a["required_provider_capability"], b["required_provider_capability"] = (
        b["required_provider_capability"],
        a["required_provider_capability"],
    )
    assert bytes.fromhex(identity_hex(crossed)) != expected

    uppercase_rulebook = copy.deepcopy(root)
    uppercase_rulebook["constitutional_rulebook"]["digest_hex"] = (
        uppercase_rulebook["constitutional_rulebook"]["digest_hex"].upper()
    )
    assert bytes.fromhex(identity_hex(uppercase_rulebook)) == expected

    uppercase_source_anchor = copy.deepcopy(root)
    uppercase_source_anchor["root_source_anchor_digest_hex"] = (
        uppercase_source_anchor["root_source_anchor_digest_hex"].upper()
    )
    assert bytes.fromhex(identity_hex(uppercase_source_anchor)) == expected
    assert source_descriptor_hex(uppercase_source_anchor) == source_expected

    uppercase_rotation_anchor = copy.deepcopy(root)
    uppercase_rotation_anchor["rotation_authority_anchor_digest_hex"] = (
        uppercase_rotation_anchor["rotation_authority_anchor_digest_hex"].upper()
    )
    assert bytes.fromhex(identity_hex(uppercase_rotation_anchor)) == expected
    assert rotation_authority_hex(uppercase_rotation_anchor) == rotation_expected

    changed_rotation_anchor = copy.deepcopy(root)
    changed_rotation_anchor["rotation_authority_anchor_digest_hex"] = "88" * 32
    assert source_descriptor_hex(changed_rotation_anchor) == source_expected
    assert rotation_authority_hex(changed_rotation_anchor) != rotation_expected

    changed_source_anchor = copy.deepcopy(root)
    changed_source_anchor["root_source_anchor_digest_hex"] = "99" * 32
    assert source_descriptor_hex(changed_source_anchor) != source_expected
    assert rotation_authority_hex(changed_source_anchor) == rotation_expected

    changed_rulebook = copy.deepcopy(root)
    changed_rulebook["constitutional_rulebook"]["digest_hex"] = "aa" * 32
    assert source_descriptor_hex(changed_rulebook) == source_expected
    assert rotation_authority_hex(changed_rulebook) == rotation_expected

    genesis_with_predecessor = copy.deepcopy(root)
    genesis_with_predecessor["predecessor_root_digest_hex"] = "77" * 32
    _expect_error(genesis_with_predecessor)

    successor_without_predecessor = copy.deepcopy(root)
    successor_without_predecessor["generation"] = 1
    _expect_error(successor_without_predecessor)

    successor = copy.deepcopy(root)
    successor["generation"] = 1
    successor["predecessor_root_digest_hex"] = expected.hex()
    assert bytes.fromhex(identity_hex(successor)) != expected

    zero_predecessor = copy.deepcopy(successor)
    zero_predecessor["predecessor_root_digest_hex"] = "00" * 32
    _expect_error(zero_predecessor)

    expired = copy.deepcopy(root)
    expired["expires_at_ms"] = expired["valid_from_ms"]
    _expect_error(expired)

    for field in (
        "constitutional_rulebook",
        "root_source_anchor_digest_hex",
        "rotation_authority_anchor_digest_hex",
    ):
        zeroed = copy.deepcopy(root)
        if field == "constitutional_rulebook":
            zeroed[field]["digest_hex"] = "00" * 32
        else:
            zeroed[field] = "00" * 32
        _expect_error(zeroed)

    zero_provider_rulebook = copy.deepcopy(root)
    zero_provider_rulebook["authorized_policy_scopes"][0]["provider_authority_rulebook"][
        "digest_hex"
    ] = "00" * 32
    _expect_error(zero_provider_rulebook)

    immutable_with_profile = copy.deepcopy(root)
    immutable_with_profile["rotation_mode"] = "immutable"
    _expect_error(immutable_with_profile)

    immutable_with_anchor = copy.deepcopy(root)
    immutable_with_anchor["rotation_mode"] = "immutable"
    immutable_with_anchor["rotation_profile"] = None
    _expect_error(immutable_with_anchor)

    immutable = copy.deepcopy(root)
    immutable["rotation_mode"] = "immutable"
    immutable["rotation_profile"] = None
    immutable["rotation_authority_anchor_digest_hex"] = None
    assert rotation_authority_hex(immutable) is None
    assert bytes.fromhex(identity_hex(immutable)) != expected

    rotatable_without_profile = copy.deepcopy(root)
    rotatable_without_profile["rotation_profile"] = None
    _expect_error(rotatable_without_profile)

    rotatable_without_anchor = copy.deepcopy(root)
    rotatable_without_anchor["rotation_authority_anchor_digest_hex"] = None
    _expect_error(rotatable_without_anchor)

    control_identifier = copy.deepcopy(root)
    control_identifier["institution_id"] = "institution:city\nsmuggled"
    _expect_error(control_identifier)

    leading_space = copy.deepcopy(root)
    leading_space["institution_id"] = " institution:city"
    _expect_error(leading_space)

    trailing_space = copy.deepcopy(root)
    trailing_space["authorized_policy_scopes"][0]["policy_registry_namespace"] += " "
    _expect_error(trailing_space)

    print(f"GOVSYS-003A PASS: {actual.hex()}")
    print(f"GOVSYS-003A source descriptor PASS: {source_expected}")
    print(f"GOVSYS-003A rotation authority PASS: {rotation_expected}")


if __name__ == "__main__":
    self_test()

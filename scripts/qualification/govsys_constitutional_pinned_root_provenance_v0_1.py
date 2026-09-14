#!/usr/bin/env python3
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Independent GOVSYS-003B external pinned-root provenance qualifier."""

from __future__ import annotations

import copy
import hashlib
import json
import re
import struct
from pathlib import Path
from typing import Any

from govsys_constitutional_trust_root_identity_v0_1 import (
    ContractError as RootContractError,
    IDENTITY_PROFILE as ROOT_IDENTITY_PROFILE,
    identity_hex as root_identity_hex,
    validate_root,
)

PROTOCOL_VERSION = "mycelix-constitutional-pinned-root-provenance-v0.1"
PIN_PROFILE = "mycelix-provisioned-constitutional-root-pin-v1"
PROVENANCE_PROFILE = (
    "mycelix-constitutional-pinned-root-provenance-v1-sha256-framed-semantic"
)
DOMAIN = b"mycelix/public-institution/constitutional-pinned-root-provenance/v1"
PINNED_BOOTSTRAP_MODE = "pinned-constitutional-commitment"
MAX_PROFILE_BYTES = 256
MAX_REF_BYTES = 1024
HEX_32 = re.compile(r"^[0-9a-fA-F]{64}$")
ROOT_VECTOR_PATH = Path(__file__).with_name("govsys_003a_root_identity_vector_v1.json")
VECTOR_PATH = Path(__file__).with_name("govsys_003b_pinned_root_provenance_vector_v1.json")


class ProvenanceError(ValueError):
    """Input violates the GOVSYS-003B pinned-root provenance contract."""


def _text(value: Any, field: str, max_bytes: int) -> str:
    if not isinstance(value, str):
        raise ProvenanceError(f"{field} must be text")
    raw = value.encode("utf-8")
    if not raw or len(raw) > max_bytes:
        raise ProvenanceError(f"invalid {field}")
    if raw[0] == 0x20 or raw[-1] == 0x20:
        raise ProvenanceError(f"leading/trailing ASCII space in {field}")
    if any(byte < 0x20 or byte == 0x7F for byte in raw):
        raise ProvenanceError(f"ASCII control byte in {field}")
    return value


def _digest(value: Any, field: str) -> bytes:
    if not isinstance(value, str) or HEX_32.fullmatch(value) is None:
        raise ProvenanceError(f"{field} must be exactly 32 bytes of hex")
    raw = bytes.fromhex(value)
    if raw == bytes(32):
        raise ProvenanceError(f"{field} must not be zero")
    return raw


def _frame(raw: bytes) -> bytes:
    return struct.pack("<Q", len(raw)) + raw


def _frame_text(value: str) -> bytes:
    return _frame(value.encode("utf-8"))


def validate_pin(pin: Any) -> dict[str, Any]:
    required = {
        "pin_profile",
        "root_identity_profile",
        "root_digest_hex",
        "bootstrap_profile",
        "provisioning_ref",
    }
    if not isinstance(pin, dict) or set(pin) != required:
        raise ProvenanceError("pin must contain exactly the GOVSYS-003B fields")

    if pin["pin_profile"] != PIN_PROFILE:
        raise ProvenanceError("unsupported provisioned pin profile")
    if pin["root_identity_profile"] != ROOT_IDENTITY_PROFILE:
        raise ProvenanceError("wrong Root-A identity profile")

    _digest(pin["root_digest_hex"], "root_digest_hex")
    _text(pin["bootstrap_profile"], "bootstrap_profile", MAX_PROFILE_BYTES)
    _text(pin["provisioning_ref"], "provisioning_ref", MAX_REF_BYTES)
    return pin


def provenance_bytes(pin: dict[str, Any]) -> bytes:
    validate_pin(pin)
    output = bytearray(DOMAIN)
    output += _frame_text(PROVENANCE_PROFILE)
    output += _frame_text(PROTOCOL_VERSION)
    output += _frame_text(pin["pin_profile"])
    output += _frame_text(pin["root_identity_profile"])
    output += _frame(bytes.fromhex(pin["root_digest_hex"]))
    output += _frame_text(pin["bootstrap_profile"])
    output += _frame_text(pin["provisioning_ref"])
    return bytes(output)


def provenance_identity_hex(pin: dict[str, Any]) -> str:
    return hashlib.sha256(provenance_bytes(pin)).hexdigest()


def qualify_pinned_root(root: Any, pin: Any) -> dict[str, str]:
    try:
        root = validate_root(root)
    except RootContractError as error:
        raise ProvenanceError(f"invalid Root-A candidate: {error}") from error
    pin = validate_pin(pin)

    if root["generation"] != 0:
        raise ProvenanceError("pinned bootstrap is generation-zero only")
    if root["predecessor_root_digest_hex"] is not None:
        raise ProvenanceError("generation-zero pinned root must not name a predecessor")
    if root["bootstrap_mode"] != PINNED_BOOTSTRAP_MODE:
        raise ProvenanceError("Root-A bootstrap mode is not pinned constitutional commitment")
    if root["bootstrap_profile"] != pin["bootstrap_profile"]:
        raise ProvenanceError("bootstrap profile mismatch")

    actual_root_digest = root_identity_hex(root)
    if bytes.fromhex(actual_root_digest) != _digest(pin["root_digest_hex"], "root_digest_hex"):
        raise ProvenanceError("external pin does not match independently recomputed Root-A identity")

    return {
        "root_identity_profile": ROOT_IDENTITY_PROFILE,
        "root_digest_hex": actual_root_digest,
        "provenance_profile": PROVENANCE_PROFILE,
        "provenance_digest_hex": provenance_identity_hex(pin),
    }


def _expect_error(root: Any, pin: Any) -> None:
    try:
        qualify_pinned_root(root, pin)
    except ProvenanceError:
        return
    raise AssertionError("expected GOVSYS-003B qualification rejection")


def self_test() -> None:
    root_vector = json.loads(ROOT_VECTOR_PATH.read_text(encoding="utf-8"))
    vector = json.loads(VECTOR_PATH.read_text(encoding="utf-8"))

    if not isinstance(vector, dict) or set(vector) != {
        "profile",
        "expected_provenance_digest_hex",
        "pin",
    }:
        raise AssertionError("Root-B golden vector wrapper must have exact shape")
    if vector["profile"] != PROVENANCE_PROFILE:
        raise AssertionError("wrong provenance profile")

    root = root_vector["root"]
    pin = vector["pin"]
    qualified = qualify_pinned_root(root, pin)

    assert qualified["root_digest_hex"] == root_vector["expected_digest_hex"]
    assert qualified["provenance_digest_hex"] == vector["expected_provenance_digest_hex"]
    assert qualified["provenance_digest_hex"] == (
        "dc28fbf6ba1396dd34908c24fca6faa51bc98b0b78dcab25091e07be425313db"
    )

    wrong_digest = copy.deepcopy(pin)
    wrong_digest["root_digest_hex"] = "55" * 32
    _expect_error(root, wrong_digest)

    zero_digest = copy.deepcopy(pin)
    zero_digest["root_digest_hex"] = "00" * 32
    _expect_error(root, zero_digest)

    wrong_root_profile = copy.deepcopy(pin)
    wrong_root_profile["root_identity_profile"] = "mycelix-other-root-profile"
    _expect_error(root, wrong_root_profile)

    wrong_pin_profile = copy.deepcopy(pin)
    wrong_pin_profile["pin_profile"] = "mycelix-other-pin-profile"
    _expect_error(root, wrong_pin_profile)

    wrong_bootstrap_profile = copy.deepcopy(pin)
    wrong_bootstrap_profile["bootstrap_profile"] = "deployment-other-pin-v1"
    _expect_error(root, wrong_bootstrap_profile)

    malformed_ref = copy.deepcopy(pin)
    malformed_ref["provisioning_ref"] = " deployment:example"
    _expect_error(root, malformed_ref)

    wrong_mode = copy.deepcopy(root)
    wrong_mode["bootstrap_mode"] = "external-institutional-credential"
    wrong_mode_pin = copy.deepcopy(pin)
    wrong_mode_pin["root_digest_hex"] = root_identity_hex(wrong_mode)
    _expect_error(wrong_mode, wrong_mode_pin)

    generation_one = copy.deepcopy(root)
    generation_one["generation"] = 1
    generation_one["predecessor_root_digest_hex"] = root_vector["expected_digest_hex"]
    generation_one_pin = copy.deepcopy(pin)
    generation_one_pin["root_digest_hex"] = root_identity_hex(generation_one)
    _expect_error(generation_one, generation_one_pin)

    changed_source_anchor = copy.deepcopy(root)
    changed_source_anchor["root_source_anchor_digest_hex"] = "66" * 32
    _expect_error(changed_source_anchor, pin)

    changed_rotation_anchor = copy.deepcopy(root)
    changed_rotation_anchor["rotation_authority_anchor_digest_hex"] = "77" * 32
    _expect_error(changed_rotation_anchor, pin)

    uppercase_pin = copy.deepcopy(pin)
    uppercase_pin["root_digest_hex"] = uppercase_pin["root_digest_hex"].upper()
    upper_qualified = qualify_pinned_root(root, uppercase_pin)
    assert upper_qualified["provenance_digest_hex"] == qualified["provenance_digest_hex"]

    changed_ref = copy.deepcopy(pin)
    changed_ref["provisioning_ref"] = "deployment:example-city:constitutional-root-pin:v2"
    changed_ref_qualified = qualify_pinned_root(root, changed_ref)
    assert changed_ref_qualified["root_digest_hex"] == qualified["root_digest_hex"]
    assert changed_ref_qualified["provenance_digest_hex"] != qualified["provenance_digest_hex"]

    print(
        "GOVSYS-003B PASS: "
        f"root={qualified['root_digest_hex']} "
        f"provenance={qualified['provenance_digest_hex']}"
    )


if __name__ == "__main__":
    self_test()

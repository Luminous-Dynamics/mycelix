#!/usr/bin/env python3
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""GOVSYS-003C-A qualification-first constitutional rooted-lineage composition oracle."""

from __future__ import annotations

import copy
import hashlib
import json
import struct
from pathlib import Path
from typing import Any

import govsys_constitutional_pinned_root_provenance_v0_1 as ROOT_B
import govsys_constitutional_transition_verifier_v0_1 as TRANSITION

ROOT = TRANSITION.ROOT

PROTOCOL_VERSION = "mycelix-constitutional-rooted-lineage-v0.1"
LINEAGE_DOMAIN_PROFILE = (
    "mycelix-constitutional-root-lineage-domain-v1-sha256-framed-semantic"
)
LINEAGE_DOMAIN = b"mycelix/public-institution/constitutional-root-lineage-domain/v1"
CORE_LINEAGE_PROFILE = "mycelix-core-lineage-v1-sha256-framed-semantic"
CORE_LINEAGE_DOMAIN = b"mycelix/core-lineage/rooted-lineage/v1"
CONSTITUTIONAL_LINEAGE_PROFILE = (
    "mycelix-constitutional-root-lineage-v1-sha256-framed-semantic"
)
CONSTITUTIONAL_LINEAGE_DOMAIN = (
    b"mycelix/public-institution/constitutional-root-lineage/v1"
)
QUALIFICATION_PROFILE = (
    "mycelix-constitutional-root-lineage-qualification-v1-sha256-framed-semantic"
)
QUALIFICATION_DOMAIN = (
    b"mycelix/public-institution/constitutional-root-lineage-qualification/v1"
)

VECTOR_PATH = Path(__file__).with_name("govsys_003c_rooted_lineage_vector_v1.json")
TRANSITION_VECTOR_PATH = Path(__file__).with_name(
    "govsys_003c_transition_verifier_vector_v1.json"
)


class RootedLineageError(ValueError):
    pass


def _frame(raw: bytes) -> bytes:
    return struct.pack("<Q", len(raw)) + raw


def _text(value: str) -> bytes:
    return _frame(value.encode("utf-8"))


def _u64(value: int) -> bytes:
    return _frame(struct.pack("<Q", value))


def _optional_text(value: str | None) -> bytes:
    if value is None:
        return _frame(b"\x00")
    return _frame(b"\x01") + _text(value)


def _profiled(profile: str, digest_hex: str) -> bytes:
    return _text(profile) + _frame(bytes.fromhex(digest_hex))


def _sha256(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def lineage_domain_bytes(root: dict[str, Any]) -> bytes:
    ROOT.validate_root(root)
    out = bytearray(LINEAGE_DOMAIN)
    out += _text(LINEAGE_DOMAIN_PROFILE)
    out += _text(ROOT.IDENTITY_PROFILE)
    out += _text(root["protocol_version"])
    out += _text(root["institution_id"])
    out += _optional_text(root["jurisdiction_id"])
    out += _text(root["bootstrap_mode"])
    out += _text(root["bootstrap_profile"])
    out += _text(root["authoritative_root_source_ref"])
    out += _text(root["root_coverage_profile"])
    out += _text(root["root_source_verification_profile"])
    return bytes(out)


def lineage_domain_identity_hex(root: dict[str, Any]) -> str:
    return _sha256(lineage_domain_bytes(root))


def _core_lineage_one_edge_bytes(
    predecessor: dict[str, Any],
    successor: dict[str, Any],
    evidence: dict[str, Any],
) -> bytes:
    domain = lineage_domain_identity_hex(predecessor)
    if lineage_domain_identity_hex(successor) != domain:
        raise RootedLineageError("transition crosses constitutional lineage domain")

    candidate_generation = successor["generation"]
    if candidate_generation != predecessor["generation"] + 1:
        raise RootedLineageError("one-edge projection requires contiguous generation")

    out = bytearray(CORE_LINEAGE_DOMAIN)
    out += _text(CORE_LINEAGE_PROFILE)
    out += _profiled(LINEAGE_DOMAIN_PROFILE, domain)
    out += _u64(predecessor["generation"])
    out += _profiled(ROOT.IDENTITY_PROFILE, evidence["predecessor_root_digest_hex"])
    out += _profiled(
        ROOT.SOURCE_DESCRIPTOR_PROFILE,
        evidence["predecessor_source_descriptor_digest_hex"],
    )
    out += _u64(predecessor["valid_from_ms"])
    out += _u64(1)
    out += _u64(predecessor["generation"])
    out += _profiled(ROOT.IDENTITY_PROFILE, evidence["predecessor_root_digest_hex"])
    out += _profiled(
        ROOT.SOURCE_DESCRIPTOR_PROFILE,
        evidence["predecessor_source_descriptor_digest_hex"],
    )
    out += _u64(successor["generation"])
    out += _profiled(ROOT.IDENTITY_PROFILE, evidence["successor_root_digest_hex"])
    out += _profiled(
        ROOT.SOURCE_DESCRIPTOR_PROFILE,
        evidence["successor_source_descriptor_digest_hex"],
    )
    out += _profiled(
        TRANSITION.TRANSITION_PROFILE,
        evidence["transition_identity_digest_hex"],
    )
    out += _u64(evidence["effective_at_ms"])
    return bytes(out)


def core_lineage_one_edge_identity_hex(
    predecessor: dict[str, Any],
    successor: dict[str, Any],
    evidence: dict[str, Any],
) -> str:
    return _sha256(_core_lineage_one_edge_bytes(predecessor, successor, evidence))


def constitutional_lineage_identity_hex(
    lineage_domain_digest_hex: str,
    core_lineage_digest_hex: str,
) -> str:
    out = bytearray(CONSTITUTIONAL_LINEAGE_DOMAIN)
    out += _text(CONSTITUTIONAL_LINEAGE_PROFILE)
    out += _profiled(LINEAGE_DOMAIN_PROFILE, lineage_domain_digest_hex)
    out += _profiled(CORE_LINEAGE_PROFILE, core_lineage_digest_hex)
    return _sha256(bytes(out))


def qualification_identity_hex(
    root_b_provenance_digest_hex: str,
    constitutional_lineage_digest_hex: str,
) -> str:
    out = bytearray(QUALIFICATION_DOMAIN)
    out += _text(QUALIFICATION_PROFILE)
    out += _profiled(ROOT_B.PROVENANCE_PROFILE, root_b_provenance_digest_hex)
    out += _profiled(
        CONSTITUTIONAL_LINEAGE_PROFILE, constitutional_lineage_digest_hex
    )
    return _sha256(bytes(out))


def qualify_one_edge(
    predecessor: dict[str, Any],
    successor: dict[str, Any],
    pin: dict[str, Any],
    transition_bundle: dict[str, Any],
) -> dict[str, Any]:
    pinned = ROOT_B.qualify_pinned_root(predecessor, pin)
    evidence = TRANSITION.qualify_transition(transition_bundle)

    predecessor_identity = ROOT.identity_hex(predecessor)
    successor_identity = ROOT.identity_hex(successor)
    if pinned["root_digest_hex"] != predecessor_identity:
        raise RootedLineageError("Root-B pin does not bind transition predecessor")
    if evidence["predecessor_root_digest_hex"] != predecessor_identity:
        raise RootedLineageError("transition evidence predecessor mismatch")
    if evidence["successor_root_digest_hex"] != successor_identity:
        raise RootedLineageError("transition evidence successor mismatch")

    domain_digest = lineage_domain_identity_hex(predecessor)
    if lineage_domain_identity_hex(successor) != domain_digest:
        raise RootedLineageError("verified edge changes lineage-domain identity")

    # Replay closure is a set theorem. Even the one-edge qualifier routes through
    # the exact #839 helper so later multi-edge expansion does not invent a second rule.
    TRANSITION.assert_no_replay_conflicts([evidence])

    core_digest = core_lineage_one_edge_identity_hex(predecessor, successor, evidence)
    history_digest = constitutional_lineage_identity_hex(domain_digest, core_digest)
    qualification_digest = qualification_identity_hex(
        pinned["provenance_digest_hex"], history_digest
    )

    return {
        "protocol_version": PROTOCOL_VERSION,
        "lineage_domain_profile": LINEAGE_DOMAIN_PROFILE,
        "lineage_domain_digest_hex": domain_digest,
        "root_b_provenance_profile": pinned["provenance_profile"],
        "root_b_provenance_digest_hex": pinned["provenance_digest_hex"],
        "core_lineage_profile": CORE_LINEAGE_PROFILE,
        "core_lineage_digest_hex": core_digest,
        "constitutional_lineage_profile": CONSTITUTIONAL_LINEAGE_PROFILE,
        "constitutional_lineage_digest_hex": history_digest,
        "qualification_profile": QUALIFICATION_PROFILE,
        "qualification_digest_hex": qualification_digest,
        "endpoint_generation": successor["generation"],
        "endpoint_root_identity_profile": ROOT.IDENTITY_PROFILE,
        "endpoint_root_digest_hex": successor_identity,
        "endpoint_source_descriptor_profile": ROOT.SOURCE_DESCRIPTOR_PROFILE,
        "endpoint_source_descriptor_digest_hex": ROOT.source_descriptor_hex(successor),
        "replay_key_predecessor_root_digest_hex": predecessor_identity,
        "replay_key_nonce_hex": evidence["replay_nonce_hex"],
        "grants_currentness": False,
        "grants_effect_authority": False,
    }


def _transition_bundle(vector: dict[str, Any]) -> dict[str, Any]:
    return {
        key: copy.deepcopy(vector[key])
        for key in (
            "profiles",
            "predecessor_root",
            "successor_root",
            "authorization_material",
            "authorization_proof",
            "candidate",
        )
    }


def _expect_error(fn, *args) -> None:
    try:
        fn(*args)
    except (
        RootedLineageError,
        ROOT_B.ProvenanceError,
        TRANSITION.ContractError,
        ValueError,
    ):
        return
    raise AssertionError("expected rooted-lineage qualification rejection")


def self_test() -> None:
    vector = json.loads(VECTOR_PATH.read_text(encoding="utf-8"))
    transition_vector = json.loads(TRANSITION_VECTOR_PATH.read_text(encoding="utf-8"))

    if not isinstance(vector, dict) or set(vector) != {
        "profile",
        "root_b_pin",
        "alternate_root_b_pin",
        "expected",
    }:
        raise AssertionError("Root-C-A vector wrapper has unexpected shape")
    if vector["profile"] != PROTOCOL_VERSION:
        raise AssertionError("wrong Root-C-A protocol profile")

    predecessor = transition_vector["predecessor_root"]
    successor = transition_vector["successor_root"]
    bundle = _transition_bundle(transition_vector)

    qualified = qualify_one_edge(
        predecessor,
        successor,
        copy.deepcopy(vector["root_b_pin"]),
        copy.deepcopy(bundle),
    )
    expected = vector["expected"]
    primary_expected = expected["primary"]
    for key, value in primary_expected.items():
        if qualified[key] != value:
            raise AssertionError((key, qualified[key], value))

    # The existing #839 fixture rotates source and future rotation authority while
    # preserving the ordinary-rotation domain. Its rulebook also changes.
    assert (
        predecessor["root_source_anchor_digest_hex"]
        != successor["root_source_anchor_digest_hex"]
    )
    assert (
        predecessor["rotation_authority_anchor_digest_hex"]
        != successor["rotation_authority_anchor_digest_hex"]
    )
    assert predecessor["constitutional_rulebook"] != successor["constitutional_rulebook"]
    assert lineage_domain_identity_hex(predecessor) == lineage_domain_identity_hex(successor)

    # Deployment provenance must not contaminate stable constitutional history.
    alternate = qualify_one_edge(
        predecessor,
        successor,
        copy.deepcopy(vector["alternate_root_b_pin"]),
        copy.deepcopy(bundle),
    )
    assert (
        alternate["constitutional_lineage_digest_hex"]
        == qualified["constitutional_lineage_digest_hex"]
    )
    assert alternate["core_lineage_digest_hex"] == qualified["core_lineage_digest_hex"]
    assert (
        alternate["root_b_provenance_digest_hex"]
        != qualified["root_b_provenance_digest_hex"]
    )
    assert alternate["qualification_digest_hex"] != qualified["qualification_digest_hex"]
    assert alternate["root_b_provenance_digest_hex"] == expected["alternate"][
        "root_b_provenance_digest_hex"
    ]
    assert alternate["qualification_digest_hex"] == expected["alternate"][
        "qualification_digest_hex"
    ]

    # Replay theorem: exact duplicate is harmless; same predecessor+nonce bound to
    # a different stable transition identity is an explicit conflict.
    evidence = TRANSITION.qualify_transition(copy.deepcopy(bundle))
    TRANSITION.assert_no_replay_conflicts([evidence, copy.deepcopy(evidence)])
    replay_conflict = copy.deepcopy(evidence)
    replay_conflict["transition_identity_digest_hex"] = "77" * 32
    _expect_error(
        TRANSITION.assert_no_replay_conflicts,
        [evidence, replay_conflict],
    )

    # A domain-bound migration cannot be smuggled through ordinary rotation.
    cross_domain_bundle = copy.deepcopy(bundle)
    cross_domain_bundle["successor_root"]["institution_id"] = "institution:other-city"
    _expect_error(TRANSITION.qualify_transition, cross_domain_bundle)

    # A pin for a different root cannot bootstrap this historical chain.
    wrong_pin = copy.deepcopy(vector["root_b_pin"])
    wrong_pin["root_digest_hex"] = "55" * 32
    _expect_error(
        qualify_one_edge,
        predecessor,
        successor,
        wrong_pin,
        copy.deepcopy(bundle),
    )

    assert not qualified["grants_currentness"]
    assert not qualified["grants_effect_authority"]

    print(
        "GOVSYS-003C-A rooted-lineage composition PASS:",
        qualified["constitutional_lineage_digest_hex"],
    )


if __name__ == "__main__":
    self_test()

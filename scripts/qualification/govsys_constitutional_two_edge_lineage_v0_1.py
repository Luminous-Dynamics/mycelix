#!/usr/bin/env python3
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""GOVSYS-003C-B fresh two-edge constitutional rooted-lineage qualification oracle."""

from __future__ import annotations

import copy
import hashlib
import json
import struct
from pathlib import Path
from typing import Any

import govsys_constitutional_pinned_root_provenance_v0_1 as ROOT_B
import govsys_constitutional_rooted_lineage_v0_1 as ROOT_C_A
import govsys_constitutional_transition_verifier_v0_1 as TRANSITION

ROOT = TRANSITION.ROOT

PROTOCOL_VERSION = "mycelix-constitutional-rooted-lineage-two-edge-v0.1"
VECTOR_PATH = Path(__file__).with_name("govsys_003c_two_edge_lineage_vector_v1.json")


class TwoEdgeLineageError(ValueError):
    pass


def _frame(raw: bytes) -> bytes:
    return struct.pack("<Q", len(raw)) + raw


def _text(value: str) -> bytes:
    return _frame(value.encode("utf-8"))


def _u64(value: int) -> bytes:
    return _frame(struct.pack("<Q", value))


def _profiled(profile: str, digest_hex: str) -> bytes:
    return _text(profile) + _frame(bytes.fromhex(digest_hex))


def _edge_bundle(
    profiles: dict[str, Any],
    predecessor: dict[str, Any],
    successor: dict[str, Any],
    edge: dict[str, Any],
) -> dict[str, Any]:
    return {
        "profiles": copy.deepcopy(profiles),
        "predecessor_root": copy.deepcopy(predecessor),
        "successor_root": copy.deepcopy(successor),
        "authorization_material": copy.deepcopy(edge["authorization_material"]),
        "authorization_proof": copy.deepcopy(edge["authorization_proof"]),
        "candidate": copy.deepcopy(edge["candidate"]),
    }


def _core_two_edge_bytes(
    root: dict[str, Any],
    evidences: list[dict[str, Any]],
) -> bytes:
    if len(evidences) != 2:
        raise TwoEdgeLineageError("two-edge profile requires exactly two verified edges")

    domain = ROOT_C_A.lineage_domain_identity_hex(root)
    remaining = list(evidences)
    ordered: list[dict[str, Any]] = []
    current_root = ROOT.identity_hex(root)
    while remaining:
        candidates = [
            item
            for item in remaining
            if item["predecessor_root_digest_hex"] == current_root
        ]
        if len(candidates) != 1:
            raise TwoEdgeLineageError(
                "verified edge set does not define one unambiguous contiguous chain"
            )
        edge = candidates[0]
        ordered.append(edge)
        remaining.remove(edge)
        current_root = edge["successor_root_digest_hex"]

    if ordered[0]["effective_at_ms"] > ordered[1]["effective_at_ms"]:
        raise TwoEdgeLineageError("verified transition time regresses")

    root_generation = root["generation"]
    if root_generation != 0:
        raise TwoEdgeLineageError("fresh two-edge fixture must anchor generation zero")

    out = bytearray(ROOT_C_A.CORE_LINEAGE_DOMAIN)
    out += _text(ROOT_C_A.CORE_LINEAGE_PROFILE)
    out += _profiled(ROOT_C_A.LINEAGE_DOMAIN_PROFILE, domain)
    out += _u64(root_generation)
    out += _profiled(ROOT.IDENTITY_PROFILE, ROOT.identity_hex(root))
    out += _profiled(ROOT.SOURCE_DESCRIPTOR_PROFILE, ROOT.source_descriptor_hex(root))
    out += _u64(root["valid_from_ms"])
    out += _u64(2)

    for generation, evidence in enumerate(ordered):
        out += _u64(generation)
        out += _profiled(ROOT.IDENTITY_PROFILE, evidence["predecessor_root_digest_hex"])
        out += _profiled(
            ROOT.SOURCE_DESCRIPTOR_PROFILE,
            evidence["predecessor_source_descriptor_digest_hex"],
        )
        out += _u64(generation + 1)
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


def _core_two_edge_identity_hex(
    root: dict[str, Any], evidences: list[dict[str, Any]]
) -> str:
    return hashlib.sha256(_core_two_edge_bytes(root, evidences)).hexdigest()


def qualify_two_edge(vector: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(vector, dict) or set(vector) != {
        "profile",
        "profiles",
        "root_b_pin",
        "roots",
        "edges",
        "expected",
    }:
        raise TwoEdgeLineageError("two-edge vector wrapper has unexpected shape")
    if vector["profile"] != PROTOCOL_VERSION:
        raise TwoEdgeLineageError("wrong two-edge protocol profile")
    if not isinstance(vector["roots"], dict) or set(vector["roots"]) != {"g0", "g1", "g2"}:
        raise TwoEdgeLineageError("two-edge roots must be exactly g0/g1/g2")
    if not isinstance(vector["edges"], list) or len(vector["edges"]) != 2:
        raise TwoEdgeLineageError("two-edge profile requires exactly two edge bundles")

    g0 = vector["roots"]["g0"]
    g1 = vector["roots"]["g1"]
    g2 = vector["roots"]["g2"]
    for root in (g0, g1, g2):
        ROOT.validate_root(root)

    pinned = ROOT_B.qualify_pinned_root(g0, vector["root_b_pin"])
    bundles = [
        _edge_bundle(vector["profiles"], g0, g1, vector["edges"][0]),
        _edge_bundle(vector["profiles"], g1, g2, vector["edges"][1]),
    ]
    evidences = [TRANSITION.qualify_transition(bundle) for bundle in bundles]

    ids = [ROOT.identity_hex(root) for root in (g0, g1, g2)]
    sources = [ROOT.source_descriptor_hex(root) for root in (g0, g1, g2)]
    rotations = [ROOT.rotation_authority_hex(root) for root in (g0, g1, g2)]

    if pinned["root_digest_hex"] != ids[0]:
        raise TwoEdgeLineageError("Root-B pin does not bind G0")
    if evidences[0]["predecessor_root_digest_hex"] != ids[0]:
        raise TwoEdgeLineageError("edge0 predecessor does not bind G0")
    if evidences[0]["successor_root_digest_hex"] != ids[1]:
        raise TwoEdgeLineageError("edge0 successor does not bind G1")
    if evidences[1]["predecessor_root_digest_hex"] != ids[1]:
        raise TwoEdgeLineageError("edge1 predecessor does not bind G1")
    if evidences[1]["successor_root_digest_hex"] != ids[2]:
        raise TwoEdgeLineageError("edge1 successor does not bind G2")

    if [root["generation"] for root in (g0, g1, g2)] != [0, 1, 2]:
        raise TwoEdgeLineageError("fixture generations must be exactly contiguous 0/1/2")
    if g1["predecessor_root_digest_hex"] != ids[0]:
        raise TwoEdgeLineageError("G1 does not point to G0")
    if g2["predecessor_root_digest_hex"] != ids[1]:
        raise TwoEdgeLineageError("G2 does not point to G1")
    if g2["rotation_mode"] != "immutable" or rotations[2] is not None:
        raise TwoEdgeLineageError("G2 must be an immutable terminal root")

    domain = ROOT_C_A.lineage_domain_identity_hex(g0)
    if any(ROOT_C_A.lineage_domain_identity_hex(root) != domain for root in (g1, g2)):
        raise TwoEdgeLineageError("verified chain crosses constitutional lineage domain")

    TRANSITION.assert_no_replay_conflicts(evidences)

    core_digest = _core_two_edge_identity_hex(g0, evidences)
    history_digest = ROOT_C_A.constitutional_lineage_identity_hex(domain, core_digest)
    qualification_digest = ROOT_C_A.qualification_identity_hex(
        pinned["provenance_digest_hex"], history_digest
    )

    return {
        "protocol_version": PROTOCOL_VERSION,
        "root_identity_digest_hex": ids,
        "source_descriptor_digest_hex": sources,
        "rotation_authority_digest_hex": rotations,
        "transition_identity_digest_hex": [e["transition_identity_digest_hex"] for e in evidences],
        "authorization_material_commitment_hex": [e["authorization_material_commitment_hex"] for e in evidences],
        "authorization_proof_digest_hex": [e["authorization_proof_digest_hex"] for e in evidences],
        "lineage_domain_digest_hex": domain,
        "root_b_provenance_digest_hex": pinned["provenance_digest_hex"],
        "core_lineage_digest_hex": core_digest,
        "constitutional_lineage_digest_hex": history_digest,
        "qualification_digest_hex": qualification_digest,
        "endpoint_generation": 2,
        "endpoint_root_digest_hex": ids[2],
        "endpoint_source_descriptor_digest_hex": sources[2],
        "grants_currentness": False,
        "grants_effect_authority": False,
    }


def _expect_error(fn, *args) -> None:
    try:
        fn(*args)
    except (TwoEdgeLineageError, ROOT_B.ProvenanceError, TRANSITION.ContractError, ValueError):
        return
    raise AssertionError("expected two-edge qualification rejection")


def self_test() -> None:
    vector = json.loads(VECTOR_PATH.read_text(encoding="utf-8"))
    qualified = qualify_two_edge(copy.deepcopy(vector))
    expected = vector["expected"]

    for key in (
        "root_identity_digest_hex",
        "source_descriptor_digest_hex",
        "rotation_authority_digest_hex",
        "authorization_material_commitment_hex",
        "authorization_proof_digest_hex",
        "transition_identity_digest_hex",
        "lineage_domain_digest_hex",
        "root_b_provenance_digest_hex",
        "core_lineage_digest_hex",
        "constitutional_lineage_digest_hex",
        "qualification_digest_hex",
        "endpoint_generation",
        "endpoint_root_digest_hex",
        "endpoint_source_descriptor_digest_hex",
    ):
        if qualified[key] != expected[key]:
            raise AssertionError((key, qualified[key], expected[key]))

    for index, edge in enumerate(vector["edges"]):
        if TRANSITION._candidate_bytes(edge["candidate"]).hex() != expected["candidate_canonical_bytes_hex"][index]:
            raise AssertionError(f"edge{index} canonical transcript drifted")

    wrong_signature = copy.deepcopy(vector)
    wrong_signature["edges"][1]["authorization_proof"] = copy.deepcopy(vector["edges"][0]["authorization_proof"])
    _expect_error(qualify_two_edge, wrong_signature)

    wrong_material = copy.deepcopy(vector)
    wrong_material["edges"][1]["authorization_material"] = copy.deepcopy(vector["edges"][0]["authorization_material"])
    _expect_error(qualify_two_edge, wrong_material)

    wrong_middle = copy.deepcopy(vector)
    wrong_middle["roots"]["g1"]["root_source_anchor_digest_hex"] = "7c" * 32
    _expect_error(qualify_two_edge, wrong_middle)

    bundles = [
        _edge_bundle(vector["profiles"], vector["roots"]["g0"], vector["roots"]["g1"], vector["edges"][0]),
        _edge_bundle(vector["profiles"], vector["roots"]["g1"], vector["roots"]["g2"], vector["edges"][1]),
    ]
    evidence0, evidence1 = [TRANSITION.qualify_transition(bundle) for bundle in bundles]
    replay_conflict = copy.deepcopy(evidence0)
    replay_conflict["transition_identity_digest_hex"] = "77" * 32
    _expect_error(TRANSITION.assert_no_replay_conflicts, [evidence0, replay_conflict, evidence1])

    forward = _core_two_edge_identity_hex(vector["roots"]["g0"], [evidence0, evidence1])
    reverse = _core_two_edge_identity_hex(vector["roots"]["g0"], [evidence1, evidence0])
    if forward != reverse:
        raise AssertionError("two-edge structural identity depends on caller order")

    terminal_bundle = _edge_bundle(vector["profiles"], vector["roots"]["g2"], vector["roots"]["g2"], vector["edges"][1])
    _expect_error(TRANSITION.qualify_transition, terminal_bundle)

    assert not qualified["grants_currentness"]
    assert not qualified["grants_effect_authority"]

    print("GOVSYS-003C-B two-edge rooted-lineage PASS:", qualified["constitutional_lineage_digest_hex"])


if __name__ == "__main__":
    self_test()

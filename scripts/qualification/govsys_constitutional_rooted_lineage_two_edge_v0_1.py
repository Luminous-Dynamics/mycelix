#!/usr/bin/env python3
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Two-edge GOVSYS-003C observed constitutional-history conformance theorem."""

from __future__ import annotations

import copy
import hashlib
import json
import struct
import sys
from pathlib import Path
from typing import Any

import govsys_constitutional_pinned_root_provenance_v0_1 as ROOT_B
import govsys_constitutional_transition_verifier_v0_1 as TRANSITION
import govsys_constitutional_rooted_lineage_v0_1 as ONE

ROOT = TRANSITION.ROOT
VECTOR_PATH = Path(__file__).with_name("govsys_003c_two_edge_lineage_vector_v1.json")
TEMPLATE_PATH = Path(__file__).with_name("govsys_003c_transition_verifier_vector_v1.json")
PROFILE = "mycelix-constitutional-rooted-lineage-two-edge-v0.1"


def _frame(raw: bytes) -> bytes:
    return struct.pack("<Q", len(raw)) + raw


def _text(value: str) -> bytes:
    return _frame(value.encode("utf-8"))


def _u64(value: int) -> bytes:
    return _frame(struct.pack("<Q", value))


def _profiled(profile: str, digest_hex: str) -> bytes:
    return _text(profile) + _frame(bytes.fromhex(digest_hex))


def _material_commitment(public_der_hex: str) -> str:
    der = bytes.fromhex(public_der_hex)
    out = bytearray(TRANSITION.MATERIAL_DOMAIN)
    out += _text(TRANSITION.MATERIAL_PROFILE)
    out += _text(TRANSITION.MATERIAL_SUITE)
    out += _frame(der)
    return hashlib.sha256(out).hexdigest()


def _root_identity_set(root: dict[str, Any]) -> tuple[str, str, str | None]:
    return (
        ROOT.identity_hex(root),
        ROOT.source_descriptor_hex(root),
        ROOT.rotation_authority_hex(root),
    )


def _build_fixture(vector: dict[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    template = json.loads(TEMPLATE_PATH.read_text(encoding="utf-8"))
    profiles = copy.deepcopy(template["profiles"])
    base = copy.deepcopy(template["predecessor_root"])
    expected = vector["expected"]
    times = vector["times"]

    anchors = [_material_commitment(value) for value in vector["public_key_spki_der_hex"]]
    if anchors != expected["material_commitments_hex"]:
        raise AssertionError((anchors, expected["material_commitments_hex"]))

    g0 = copy.deepcopy(base)
    g0["constitutional_rulebook"] = copy.deepcopy(vector["rulebooks"][0])
    g0["generation"] = 0
    g0["predecessor_root_digest_hex"] = None
    g0["root_source_anchor_digest_hex"] = vector["root_source_anchor_hex"][0]
    g0["valid_from_ms"] = times["g0_valid_from_ms"]
    g0["expires_at_ms"] = times["g0_expires_at_ms"]
    g0["rotation_mode"] = "predecessor-authorized"
    g0["rotation_profile"] = TRANSITION.NORMAL_ROTATION_PROFILE
    g0["rotation_authority_anchor_digest_hex"] = anchors[0]

    g1 = copy.deepcopy(g0)
    g1["constitutional_rulebook"] = copy.deepcopy(vector["rulebooks"][1])
    g1["generation"] = 1
    g1["predecessor_root_digest_hex"] = ROOT.identity_hex(g0)
    g1["root_source_anchor_digest_hex"] = vector["root_source_anchor_hex"][1]
    g1["valid_from_ms"] = times["edge01_effective_at_ms"]
    g1["expires_at_ms"] = times["g1_expires_at_ms"]
    g1["rotation_authority_anchor_digest_hex"] = anchors[1]

    g2 = copy.deepcopy(g1)
    g2["constitutional_rulebook"] = copy.deepcopy(vector["rulebooks"][2])
    g2["generation"] = 2
    g2["predecessor_root_digest_hex"] = ROOT.identity_hex(g1)
    g2["root_source_anchor_digest_hex"] = vector["root_source_anchor_hex"][2]
    g2["valid_from_ms"] = times["edge12_effective_at_ms"]
    g2["expires_at_ms"] = None
    g2["rotation_mode"] = "immutable"
    g2["rotation_profile"] = None
    g2["rotation_authority_anchor_digest_hex"] = None

    roots = [g0, g1, g2]
    actual_roots = [_root_identity_set(root) for root in roots]
    assert [item[0] for item in actual_roots] == expected["root_digests_hex"]
    assert [item[1] for item in actual_roots] == expected["source_descriptor_digests_hex"]
    assert [item[2] for item in actual_roots] == expected["rotation_authority_digests_hex"]

    def candidate(pred: dict[str, Any], succ: dict[str, Any], authorized: int, effective: int, nonce: str) -> dict[str, Any]:
        pred_rotation = ROOT.rotation_authority_hex(pred)
        if pred_rotation is None:
            raise AssertionError("predecessor must be rotatable")
        succ_rotation = ROOT.rotation_authority_hex(succ)
        return {
            "profile": TRANSITION.TRANSITION_PROFILE,
            "predecessor_root_identity_profile": ROOT.IDENTITY_PROFILE,
            "predecessor_root_digest_hex": ROOT.identity_hex(pred),
            "predecessor_generation": pred["generation"],
            "predecessor_source_descriptor_profile": ROOT.SOURCE_DESCRIPTOR_PROFILE,
            "predecessor_source_descriptor_digest_hex": ROOT.source_descriptor_hex(pred),
            "predecessor_rotation_authority_profile": ROOT.ROTATION_AUTHORITY_PROFILE,
            "predecessor_rotation_authority_digest_hex": pred_rotation,
            "successor_root_identity_profile": ROOT.IDENTITY_PROFILE,
            "successor_root_digest_hex": ROOT.identity_hex(succ),
            "successor_generation": succ["generation"],
            "successor_source_descriptor_profile": ROOT.SOURCE_DESCRIPTOR_PROFILE,
            "successor_source_descriptor_digest_hex": ROOT.source_descriptor_hex(succ),
            "successor_rotation_authority_profile": None if succ_rotation is None else ROOT.ROTATION_AUTHORITY_PROFILE,
            "successor_rotation_authority_digest_hex": succ_rotation,
            "authorized_at_ms": authorized,
            "effective_at_ms": effective,
            "replay_nonce_hex": nonce,
        }

    candidates = [
        candidate(g0, g1, times["edge01_authorized_at_ms"], times["edge01_effective_at_ms"], vector["replay_nonces_hex"][0]),
        candidate(g1, g2, times["edge12_authorized_at_ms"], times["edge12_effective_at_ms"], vector["replay_nonces_hex"][1]),
    ]
    assert [TRANSITION._candidate_digest(item) for item in candidates] == expected["transition_digests_hex"]

    bundles = []
    for index, item in enumerate(candidates):
        bundles.append({
            "profiles": copy.deepcopy(profiles),
            "predecessor_root": copy.deepcopy(roots[index]),
            "successor_root": copy.deepcopy(roots[index + 1]),
            "authorization_material": {
                "profile": TRANSITION.MATERIAL_PROFILE,
                "suite": TRANSITION.MATERIAL_SUITE,
                "public_key_spki_der_hex": vector["public_key_spki_der_hex"][index],
            },
            "authorization_proof": {
                "suite": TRANSITION.PROOF_SUITE,
                "signature_hex": vector["signatures_hex"][index],
            },
            "candidate": item,
        })
    return roots, bundles


def _core_two_edge_identity(roots: list[dict[str, Any]], evidence: list[dict[str, Any]]) -> str:
    domain = ONE.lineage_domain_identity_hex(roots[0])
    out = bytearray(ONE.CORE_LINEAGE_DOMAIN)
    out += _text(ONE.CORE_LINEAGE_PROFILE)
    out += _profiled(ONE.LINEAGE_DOMAIN_PROFILE, domain)
    out += _u64(roots[0]["generation"])
    out += _profiled(ROOT.IDENTITY_PROFILE, ROOT.identity_hex(roots[0]))
    out += _profiled(ROOT.SOURCE_DESCRIPTOR_PROFILE, ROOT.source_descriptor_hex(roots[0]))
    out += _u64(roots[0]["valid_from_ms"])
    out += _u64(len(evidence))
    for index, item in enumerate(evidence):
        pred = roots[index]
        succ = roots[index + 1]
        out += _u64(pred["generation"])
        out += _profiled(ROOT.IDENTITY_PROFILE, item["predecessor_root_digest_hex"])
        out += _profiled(ROOT.SOURCE_DESCRIPTOR_PROFILE, item["predecessor_source_descriptor_digest_hex"])
        out += _u64(succ["generation"])
        out += _profiled(ROOT.IDENTITY_PROFILE, item["successor_root_digest_hex"])
        out += _profiled(ROOT.SOURCE_DESCRIPTOR_PROFILE, item["successor_source_descriptor_digest_hex"])
        out += _profiled(TRANSITION.TRANSITION_PROFILE, item["transition_identity_digest_hex"])
        out += _u64(item["effective_at_ms"])
    return hashlib.sha256(out).hexdigest()


def qualify_two_edge(vector: dict[str, Any], reverse_input: bool = False) -> dict[str, Any]:
    roots, bundles = _build_fixture(vector)
    pin = {
        "pin_profile": ROOT_B.PIN_PROFILE,
        "root_identity_profile": ROOT.IDENTITY_PROFILE,
        "root_digest_hex": ROOT.identity_hex(roots[0]),
        "bootstrap_profile": roots[0]["bootstrap_profile"],
        "provisioning_ref": vector["provisioning_ref"],
    }
    pinned = ROOT_B.qualify_pinned_root(roots[0], pin)

    inputs = list(reversed(bundles)) if reverse_input else bundles
    evidence = [TRANSITION.qualify_transition(copy.deepcopy(bundle)) for bundle in inputs]
    TRANSITION.assert_no_replay_conflicts(evidence)
    evidence.sort(key=lambda item: (item["predecessor_root_digest_hex"], item["effective_at_ms"]))
    # Reorder by actual generation, not arrival order.
    by_predecessor = {item["predecessor_root_digest_hex"]: item for item in evidence}
    ordered = []
    cursor = ROOT.identity_hex(roots[0])
    for expected_successor in roots[1:]:
        item = by_predecessor.get(cursor)
        if item is None:
            raise AssertionError("missing verified contiguous transition")
        if item["successor_root_digest_hex"] != ROOT.identity_hex(expected_successor):
            raise AssertionError("verified transition successor mismatch")
        ordered.append(item)
        cursor = item["successor_root_digest_hex"]
    if len(ordered) != len(evidence):
        raise AssertionError("unreachable or duplicate transition evidence")

    domain = ONE.lineage_domain_identity_hex(roots[0])
    if any(ONE.lineage_domain_identity_hex(root) != domain for root in roots[1:]):
        raise AssertionError("ordinary transition crossed lineage domain")
    core_digest = _core_two_edge_identity(roots, ordered)
    history_digest = ONE.constitutional_lineage_identity_hex(domain, core_digest)
    qualification_digest = ONE.qualification_identity_hex(pinned["provenance_digest_hex"], history_digest)
    return {
        "roots": roots,
        "bundles": bundles,
        "evidence": ordered,
        "lineage_domain_digest_hex": domain,
        "root_b_provenance_digest_hex": pinned["provenance_digest_hex"],
        "core_lineage_digest_hex": core_digest,
        "constitutional_lineage_digest_hex": history_digest,
        "qualification_digest_hex": qualification_digest,
        "endpoint_generation": roots[-1]["generation"],
        "endpoint_root_digest_hex": ROOT.identity_hex(roots[-1]),
        "endpoint_source_descriptor_digest_hex": ROOT.source_descriptor_hex(roots[-1]),
        "grants_currentness": False,
        "grants_effect_authority": False,
    }


def emit_openssl_fixtures(vector: dict[str, Any], output: Path) -> None:
    _, bundles = _build_fixture(vector)
    output.mkdir(parents=True, exist_ok=True)
    for index, bundle in enumerate(bundles):
        (output / f"public-{index}.der").write_bytes(bytes.fromhex(bundle["authorization_material"]["public_key_spki_der_hex"]))
        (output / f"candidate-{index}.bin").write_bytes(TRANSITION._candidate_bytes(bundle["candidate"]))
        (output / f"signature-{index}.bin").write_bytes(bytes.fromhex(bundle["authorization_proof"]["signature_hex"]))


def self_test() -> None:
    vector = json.loads(VECTOR_PATH.read_text(encoding="utf-8"))
    if vector["profile"] != PROFILE:
        raise AssertionError("wrong two-edge profile")
    expected = vector["expected"]
    qualified = qualify_two_edge(vector)
    reversed_qualified = qualify_two_edge(vector, reverse_input=True)

    for field in (
        "lineage_domain_digest_hex",
        "root_b_provenance_digest_hex",
        "core_lineage_digest_hex",
        "constitutional_lineage_digest_hex",
        "qualification_digest_hex",
    ):
        assert qualified[field] == expected[field], (field, qualified[field], expected[field])
        assert reversed_qualified[field] == qualified[field]

    assert qualified["endpoint_generation"] == 2
    assert qualified["endpoint_root_digest_hex"] == expected["root_digests_hex"][2]
    assert qualified["endpoint_source_descriptor_digest_hex"] == expected["source_descriptor_digests_hex"][2]
    assert qualified["roots"][2]["rotation_mode"] == "immutable"
    assert ROOT.rotation_authority_hex(qualified["roots"][2]) is None
    assert not qualified["grants_currentness"]
    assert not qualified["grants_effect_authority"]

    # Each generation owns the next authorization. Reusing G0 material for G1->G2 fails.
    wrong_key = copy.deepcopy(qualified["bundles"][1])
    wrong_key["authorization_material"] = copy.deepcopy(qualified["bundles"][0]["authorization_material"])
    try:
        TRANSITION.qualify_transition(wrong_key)
    except TRANSITION.ContractError:
        pass
    else:
        raise AssertionError("second edge accepted predecessor's predecessor key")

    # Mutating either signed candidate must fail cryptographic verification.
    for index in range(2):
        bad = copy.deepcopy(qualified["bundles"][index])
        bad["candidate"]["replay_nonce_hex"] = ("c3" if index == 0 else "d4") * 32
        try:
            TRANSITION.qualify_transition(bad)
        except TRANSITION.ContractError:
            pass
        else:
            raise AssertionError("mutated signed candidate accepted")

    # Exact duplicate evidence is harmless; nonce reuse across distinct identities is not.
    duplicated = qualified["evidence"] + [copy.deepcopy(qualified["evidence"][0])]
    TRANSITION.assert_no_replay_conflicts(duplicated)
    conflict = copy.deepcopy(qualified["evidence"][0])
    conflict["transition_identity_digest_hex"] = "77" * 32
    try:
        TRANSITION.assert_no_replay_conflicts([qualified["evidence"][0], conflict])
    except TRANSITION.ContractError:
        pass
    else:
        raise AssertionError("replay-nonce conflict accepted")

    print("GOVSYS-003C two-edge rooted lineage PASS:", qualified["constitutional_lineage_digest_hex"])


if __name__ == "__main__":
    vector = json.loads(VECTOR_PATH.read_text(encoding="utf-8"))
    if len(sys.argv) == 3 and sys.argv[1] == "--emit-openssl":
        emit_openssl_fixtures(vector, Path(sys.argv[2]))
    elif len(sys.argv) == 1:
        self_test()
    else:
        raise SystemExit("usage: script [--emit-openssl DIR]")

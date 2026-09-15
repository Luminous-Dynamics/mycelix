#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from validate_ig007cp0_constitution_parameter_profile import load, validate

AUTHORITY = "MeasurementOnly"
SCHEMA = "mycelix-constitution-parameter-counterexamples-v1"
PROFILE_SHA256 = "770552d12489df1d2cdf8b0af676b01ea9a3da21940f70ed8a71910deaa35009"
EXPECTED_CORPUS_SHA256 = "b37be9d2e3fd0cbec3696a19327c26dc4ba7062ec089ad92ad99bc28a11fea8e"


def canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")


def build_corpus(profile: dict) -> dict:
    ref = validate(profile)
    if ref["profile_content_sha256"] != PROFILE_SHA256:
        raise ValueError("CP1 requires exact CP0 profile")

    dispatch = profile["execution_dispatch"]
    coordinator = profile["coordinator_update"]
    gate = profile["set_parameter_gate"]
    integrity = profile["integrity"]
    projection = profile["storage_projection"]

    corpus = {
        "schema": SCHEMA,
        "authority": AUTHORITY,
        "profile": {
            "profile_id": ref["profile_id"],
            "content_sha256": ref["profile_content_sha256"],
            "authority_class": ref["authority_class"],
        },
        "issue": 1002,
        "counterexamples": [
            {
                "id": "CE-CP-01",
                "revision": 1,
                "kind": "ExecutionDownstreamGateMismatch",
                "inputs": {
                    "execution_action": "UpdateParameter",
                    "execution_payload_fields": ["parameter", "value"],
                    "parameter_preexists": True,
                    "proposal_id_received_by_constitution": None,
                },
                "comparison": {
                    "execution_proposal_id_included": dispatch["proposal_id_included"],
                    "existing_parameter_without_proposal_id": gate["existing_parameter_without_proposal_id"],
                    "result": "ObservedExecutionDispatchCannotPassExistingParameterPresenceGate",
                },
                "non_claim": (
                    "Shows the frozen source-contract mismatch for an existing parameter; does not claim a live execution attempt."
                ),
            },
            {
                "id": "CE-CP-02",
                "revision": 1,
                "kind": "BootstrapAuthorizationObservation",
                "inputs": {
                    "parameter_preexists": False,
                    "proposal_id": None,
                    "name_nonempty": True,
                    "value_valid_json": True,
                },
                "comparison": {
                    "new_parameter_without_proposal_id": gate["new_parameter_without_proposal_id"],
                    "changed_by_proposal": None,
                    "result": "NewParameterCreationAllowedWithoutProposalLinkage",
                },
                "non_claim": (
                    "Models the coordinator gate for a previously absent parameter; no live mutation is performed."
                ),
            },
            {
                "id": "CE-CP-03",
                "revision": 1,
                "kind": "ProposalPresenceAuthorityObservation",
                "inputs": {
                    "parameter_preexists": True,
                    "proposal_id": "proposal:fixture",
                    "proposal_existence": "NotReconstructed",
                    "proposal_status": "NotReconstructed",
                    "proposal_type": "NotReconstructed",
                    "exact_parameter_value_authorization": "NotReconstructed",
                    "caller_authority": "NotReconstructed",
                },
                "comparison": {
                    "observed_gate": gate["existing_parameter_with_nonempty_or_unverified_some_proposal_id"],
                    "proposal_id_authority_reconstruction": gate["proposal_id_authority_reconstruction"],
                    "result": "ProposalIdPresencePassesObservedCoordinatorGateWithoutAuthorityReconstruction",
                },
                "non_claim": (
                    "Records that string presence satisfies this gate; it does not claim the fixture proposal exists or is authorized."
                ),
            },
            {
                "id": "CE-CP-04",
                "revision": 1,
                "kind": "IntegrityAuthorityObservation",
                "inputs": {
                    "name": "governance.fixture",
                    "value": "{\"enabled\":true}",
                    "changed_by_proposal": None,
                    "create_action_author": "arbitrary-fixture-author",
                },
                "comparison": {
                    "create_checks": integrity["create_checks"],
                    "create_action_author_binding": integrity["create_action_author_binding"],
                    "changed_by_proposal_authority_verification": integrity["changed_by_proposal_authority_verification"],
                    "result": "IntegrityShapeValidityDoesNotEstablishParameterMutationAuthority",
                },
                "non_claim": (
                    "Pure structural fixture only; no DHT publication or unauthorized mutation is performed."
                ),
            },
            {
                "id": "CE-CP-05",
                "revision": 1,
                "kind": "ProjectionAuthorityObservation",
                "inputs": {
                    "same_parameter_name": "quorum",
                    "publication_a": {"value": "0.60", "link_timestamp": 100},
                    "publication_b": {"value": "0.70", "link_timestamp": 200},
                },
                "comparison": {
                    "link_selection": projection["link_selection"],
                    "selected_fixture": "publication_b",
                    "explicit_authoritative_fork_rule": projection["explicit_authoritative_fork_rule"],
                    "timestamp_selection_is_authority": projection["timestamp_selection_is_authority"],
                    "result": "TimestampSelectedProjectionNotAuthoritativeForkResolution",
                },
                "non_claim": (
                    "Models the frozen read projection; does not claim a particular production fork exists."
                ),
            },
        ],
        "non_claims": [
            "no_live_unauthorized_parameter_mutation",
            "no_deployment_exploit",
            "no_legal_constitutional_invalidity_claim",
            "no_authoritative_currentness_claim",
            "no_governance_safety_claim",
        ],
    }
    corpus["corpus_sha256"] = hashlib.sha256(canonical(corpus)).hexdigest()
    return corpus


def self_test(profile: dict) -> dict:
    first = build_corpus(profile)
    second = build_corpus(profile)
    if canonical(first) != canonical(second):
        raise AssertionError("non-deterministic constitution-parameter corpus")
    if first["corpus_sha256"] != EXPECTED_CORPUS_SHA256:
        raise AssertionError(f"constitution-parameter corpus commitment drift: {first['corpus_sha256']}")

    by_id = {item["id"]: item for item in first["counterexamples"]}
    assert set(by_id) == {"CE-CP-01", "CE-CP-02", "CE-CP-03", "CE-CP-04", "CE-CP-05"}
    assert by_id["CE-CP-01"]["comparison"]["result"] == "ObservedExecutionDispatchCannotPassExistingParameterPresenceGate"
    assert by_id["CE-CP-02"]["comparison"]["result"] == "NewParameterCreationAllowedWithoutProposalLinkage"
    assert by_id["CE-CP-03"]["comparison"]["result"] == "ProposalIdPresencePassesObservedCoordinatorGateWithoutAuthorityReconstruction"
    assert by_id["CE-CP-04"]["comparison"]["result"] == "IntegrityShapeValidityDoesNotEstablishParameterMutationAuthority"
    assert by_id["CE-CP-05"]["comparison"]["result"] == "TimestampSelectedProjectionNotAuthoritativeForkResolution"

    return {
        "authority": AUTHORITY,
        "schema": SCHEMA,
        "profile_sha256": PROFILE_SHA256,
        "corpus_sha256": EXPECTED_CORPUS_SHA256,
        "counterexample_count": 5,
        "issue": 1002,
        "self_test": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--corpus", action="store_true")
    args = parser.parse_args()
    profile = load(args.profile)
    if args.self_test:
        result = self_test(profile)
    elif args.corpus:
        result = build_corpus(profile)
    else:
        parser.error("choose --self-test or --corpus")
    print(json.dumps(result, sort_keys=True, separators=(",", ":"), allow_nan=False))


if __name__ == "__main__":
    main()

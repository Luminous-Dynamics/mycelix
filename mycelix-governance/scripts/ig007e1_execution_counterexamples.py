#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from validate_ig007e0_execution_profile import load, validate

AUTHORITY = "MeasurementOnly"
SCHEMA = "mycelix-observed-execution-counterexamples-v1"
PROFILE_SHA256 = "c977bdcef9e5faac83351050999451432b618d5cc523bece804eba5dd1ae81f6"
EXPECTED_CORPUS_SHA256 = "0c6669e44d6d18396ede43324f5cf3abbb25ddd3a2a9f59abb2c8a3699ba5fd4"


def canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def build_corpus(profile: dict) -> dict:
    ref = validate(profile)
    if ref["profile_content_sha256"] != PROFILE_SHA256:
        raise ValueError("E1 requires exact E0 profile")

    creation = profile["timelock_creation"]
    ready = profile["readiness_transition"]
    execution = profile["execution"]
    dispatch = profile["action_dispatch"]

    corpus = {
        "schema": SCHEMA,
        "authority": AUTHORITY,
        "profile": {
            "profile_id": ref["profile_id"],
            "content_sha256": ref["profile_content_sha256"],
            "authority_class": ref["authority_class"],
        },
        "counterexamples": [
            {
                "id": "CE-TL-01",
                "revision": 1,
                "kind": "AuthorityGapCounterfactual",
                "issue": 904,
                "inputs": {
                    "proposal_id": "MIP-UNBOUND-FIXTURE",
                    "actions": [{"type": "EmitEvent", "event": "fixture", "payload": {}}],
                    "duration_hours": 1,
                },
                "comparison": {
                    "shape_checks": "Satisfied",
                    "proposal_lookup_predicate": creation["proposal_lookup"],
                    "proposal_status_predicate": creation["proposal_status_binding"],
                    "proposal_actions_binding_predicate": creation["proposal_actions_binding"],
                    "policy_duration_binding_predicate": creation["policy_duration_binding"],
                },
                "non_claim": (
                    "Shows absence of source-visible authorization predicates in timelock construction; "
                    "does not execute the fixture."
                ),
            },
            {
                "id": "CE-TL-02",
                "revision": 1,
                "kind": "AuthorityGapCounterfactual",
                "issue": 904,
                "inputs": {"source_status": "Pending", "caller_relation": "TimelockCreator"},
                "comparison": {
                    "creator_check": "Satisfied",
                    "source_status_check": "Satisfied",
                    "threshold_signature_predicate": ready["threshold_signature_verification"],
                    "observed_target_status": ready["target_status"],
                },
                "non_claim": (
                    "Models local predicates of mark_timelock_ready; does not claim a live transition was performed."
                ),
            },
            {
                "id": "CE-TL-03",
                "revision": 1,
                "kind": "ControlFlowDifferential",
                "issue": 904,
                "inputs": {"expired": True},
                "comparison": {
                    "Ready": {
                        "threshold_signature_lookup": "NoneInBranch",
                        "execution_authority_assumption": "ReadyImpliesPreviouslyVerified",
                    },
                    "Pending": {
                        "threshold_signature_lookup": "Attempted",
                        "unavailable_authority_behavior": "WarnAndContinue",
                    },
                },
                "non_claim": "Records source-control-flow differences, not an end-to-end exploit.",
            },
            {
                "id": "CE-TL-04",
                "revision": 1,
                "kind": "ExecutableSurfaceObservation",
                "issue": 904,
                "inputs": {"action_source": execution["action_source"]},
                "comparison": {
                    "TransferCredits": dispatch["TransferCredits"]["target"],
                    "UpdateParameter": dispatch["UpdateParameter"]["target"],
                    "EmitEvent": dispatch["EmitEvent"]["target"],
                },
                "non_claim": (
                    "Records dispatch reachability from timelock action parsing; downstream authorization remains unqualified."
                ),
            },
            {
                "id": "CE-TL-05",
                "revision": 1,
                "kind": "FailOpenAuthorityObservation",
                "issue": 904,
                "inputs": {"source_status": "Pending", "expired": True, "threshold_signing": "Unavailable"},
                "comparison": {
                    "warning": "threshold_signing_unavailable",
                    "signature_verification": "NotEstablished",
                    "source_control_flow": "ExecutionContinuesAfterWarning",
                },
                "non_claim": (
                    "Records the explicit graceful-degradation branch; does not execute governance actions."
                ),
            },
        ],
        "non_claims": [
            "no_live_exploit_claim",
            "no_downstream_authorization_claim",
            "no_financial_mutation",
            "no_constitutional_mutation",
            "no_deployment_currentness",
            "no_governance_safety_claim",
        ],
    }
    corpus["corpus_sha256"] = hashlib.sha256(canonical(corpus)).hexdigest()
    return corpus


def self_test(profile: dict) -> dict:
    first = build_corpus(profile)
    second = build_corpus(profile)
    if canonical(first) != canonical(second):
        raise AssertionError("non-deterministic corpus")
    if first["corpus_sha256"] != EXPECTED_CORPUS_SHA256:
        raise AssertionError("corpus commitment drift")

    by_id = {item["id"]: item for item in first["counterexamples"]}
    assert set(by_id) == {"CE-TL-01", "CE-TL-02", "CE-TL-03", "CE-TL-04", "CE-TL-05"}
    assert by_id["CE-TL-01"]["comparison"]["proposal_lookup_predicate"] == "NoneObserved"
    assert by_id["CE-TL-02"]["comparison"]["threshold_signature_predicate"] == "NoneObserved"
    assert by_id["CE-TL-03"]["comparison"]["Ready"]["threshold_signature_lookup"] == "NoneInBranch"
    assert by_id["CE-TL-04"]["comparison"]["TransferCredits"] == "governance_bridge::transfer_credits"
    assert by_id["CE-TL-04"]["comparison"]["UpdateParameter"] == "constitution::update_parameter"
    assert by_id["CE-TL-05"]["comparison"]["source_control_flow"] == "ExecutionContinuesAfterWarning"

    return {
        "authority": AUTHORITY,
        "schema": SCHEMA,
        "profile_sha256": PROFILE_SHA256,
        "corpus_sha256": EXPECTED_CORPUS_SHA256,
        "counterexample_count": 5,
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
        out = self_test(profile)
    elif args.corpus:
        out = build_corpus(profile)
    else:
        parser.error("choose --self-test or --corpus")
    print(json.dumps(out, sort_keys=True, separators=(",", ":"), allow_nan=False))


if __name__ == "__main__":
    main()

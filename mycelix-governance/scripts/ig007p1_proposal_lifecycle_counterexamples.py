#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from validate_ig007p0_proposal_lifecycle_profile import load, validate

AUTHORITY = "MeasurementOnly"
SCHEMA = "mycelix-proposal-lifecycle-counterexamples-v1"
PROFILE_SHA256 = "7f42e2a8df25df94112d23f261d1f3ffe299d46d37cb3a5a6fe02aca0aa6c108"
EXPECTED_CORPUS_SHA256 = "13eaaa988c73d29d67bccf7381f6f72cabd4eb7be090f36f0b444978cc708324"


def canonical(obj: object) -> bytes:
    return json.dumps(
        obj,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def build_corpus(profile: dict) -> dict:
    ref = validate(profile)
    if ref["profile_content_sha256"] != PROFILE_SHA256:
        raise ValueError("P1 requires exact P0 proposal profile")

    lookup = profile["lookup_projection"]
    update = profile["update_integrity"]

    corpus = {
        "schema": SCHEMA,
        "authority": AUTHORITY,
        "profile": {
            "profile_id": ref["profile_id"],
            "content_sha256": ref["profile_content_sha256"],
            "authority_class": ref["authority_class"],
        },
        "issue": 66,
        "counterexamples": [
            {
                "id": "CE-PROP-01",
                "revision": 1,
                "kind": "ProjectionCurrentnessObservation",
                "inputs": {
                    "creation_status": "Draft",
                    "update_child_status": "Active",
                    "proposal_by_id_target": "CreationAction",
                    "linked_record_present": True,
                },
                "comparison": {
                    "primary_lookup": lookup["primary_lookup"],
                    "linked_record_return": lookup["linked_record_return"],
                    "fallback_reached": False,
                    "observed_read_status": "Draft",
                    "authoritative_currentness": "NotEstablished",
                },
                "non_claim": (
                    "Models the frozen source lookup semantics; does not claim every deployment "
                    "read has this history."
                ),
            },
            {
                "id": "CE-PROP-02",
                "revision": 1,
                "kind": "StructuralUpdateDifferential",
                "inputs": {
                    "original_status": "Draft",
                    "updated_status": "Active",
                    "id_author_unchanged": True,
                    "version_increment": 1,
                    "semantic_content_changed": True,
                },
                "comparison": {
                    "content_freeze_condition": update["content_freeze_condition"],
                    "condition_holds": False,
                    "draft_to_active_content_mutation_structurally_rejected": update[
                        "draft_to_active_content_mutation_structurally_rejected"
                    ],
                    "result": "ContentMutationNotRejectedByObservedUpdateCheck",
                },
                "non_claim": (
                    "Records the pure update-check predicate boundary; does not publish a live "
                    "proposal update."
                ),
            },
            {
                "id": "CE-PROP-03",
                "revision": 1,
                "kind": "UpdateAuthorityPredicateObservation",
                "inputs": {
                    "proposal_author": "did:mycelix:alice",
                    "update_action_author": "did:mycelix:bob",
                    "structural_fields_otherwise_valid": True,
                },
                "comparison": {
                    "update_action_author_binding": update["update_action_author_binding"],
                    "result": "IntegrityDoesNotEstablishUpdateAuthorAuthority",
                },
                "non_claim": (
                    "Records that proposal update integrity does not inspect the update action "
                    "author; no live unauthorized update is claimed."
                ),
            },
            {
                "id": "CE-PROP-04",
                "revision": 1,
                "kind": "TemporalIntegrityObservation",
                "inputs": {
                    "original_voting_starts": 1,
                    "original_voting_ends": 2,
                    "updated_voting_starts": 3,
                    "updated_voting_ends": 2,
                    "created_changed": True,
                    "updated_timestamp_arbitrary": True,
                },
                "comparison": {
                    "voting_starts_immutability": update["voting_starts_immutability"],
                    "voting_ends_immutability": update["voting_ends_immutability"],
                    "created_timestamp_immutability": update["created_timestamp_immutability"],
                    "updated_timestamp_action_binding": update["updated_timestamp_action_binding"],
                    "update_voting_period_order_check": update["update_voting_period_order_check"],
                    "result": "TemporalMutationNotRejectedByObservedUpdateCheck",
                },
                "non_claim": (
                    "Models missing update-level temporal predicates, not a live accepted DHT mutation."
                ),
            },
            {
                "id": "CE-PROP-05",
                "revision": 1,
                "kind": "ForkProjectionObservation",
                "inputs": {
                    "common_parent_status": "Draft",
                    "child_a": {"status": "Active", "version": 2},
                    "child_b": {"status": "Cancelled", "version": 2},
                },
                "comparison": {
                    "both_transition_shapes_allowed": True,
                    "explicit_competing_update_fork_rule": lookup[
                        "explicit_competing_update_fork_rule"
                    ],
                    "proposal_by_id_update_refresh": lookup[
                        "update_refreshes_proposal_by_id_link"
                    ],
                    "result": "NoObservedDeterministicAuthoritativeChildSelection",
                },
                "non_claim": (
                    "Records absence of an explicit authoritative fork projector in the legacy "
                    "profile; it does not claim a particular DHT arrival order."
                ),
            },
        ],
        "non_claims": [
            "no_live_unauthorized_update_claim",
            "no_deployment_exploit_claim",
            "no_authoritative_currentness_claim",
            "no_successor_stack_deployment_claim",
            "no_governance_safety_claim",
        ],
    }
    corpus["corpus_sha256"] = hashlib.sha256(canonical(corpus)).hexdigest()
    return corpus


def self_test(profile: dict) -> dict:
    first = build_corpus(profile)
    second = build_corpus(profile)
    if canonical(first) != canonical(second):
        raise AssertionError("non-deterministic proposal counterexample corpus")
    if first["corpus_sha256"] != EXPECTED_CORPUS_SHA256:
        raise AssertionError(f"proposal corpus commitment drift: {first['corpus_sha256']}")

    by_id = {item["id"]: item for item in first["counterexamples"]}
    assert set(by_id) == {
        "CE-PROP-01",
        "CE-PROP-02",
        "CE-PROP-03",
        "CE-PROP-04",
        "CE-PROP-05",
    }
    assert by_id["CE-PROP-01"]["comparison"]["authoritative_currentness"] == "NotEstablished"
    assert by_id["CE-PROP-02"]["comparison"]["result"] == "ContentMutationNotRejectedByObservedUpdateCheck"
    assert by_id["CE-PROP-03"]["comparison"]["update_action_author_binding"] == "NoneObserved"
    assert by_id["CE-PROP-04"]["comparison"]["update_voting_period_order_check"] == "NoneObserved"
    assert by_id["CE-PROP-05"]["comparison"]["result"] == "NoObservedDeterministicAuthoritativeChildSelection"

    return {
        "authority": AUTHORITY,
        "schema": SCHEMA,
        "profile_sha256": PROFILE_SHA256,
        "corpus_sha256": EXPECTED_CORPUS_SHA256,
        "counterexample_count": 5,
        "issue": 66,
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

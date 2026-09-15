#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from validate_ig007a3_observed_profile_v2 import load, validate

AUTHORITY = "MeasurementOnly"
SCHEMA = "mycelix-observed-voting-counterexamples-v2"
PROFILE_SHA256 = "680af4668889c299b7e0d74531f44894a64a384be71778bca54d3f21ca80ac01"
PREDECESSOR_PROFILE_SHA256 = "cbbbb3553ce465be989b5f096364ea97ccc9b1c5d6aae67d80177df8d8109763"
PREDECESSOR_CORPUS_SHA256 = "bb1cdcfe2205bcf6e6d718b536cbb73ba29a21c9da7dfa664798d02f9f866d90"
EXPECTED_CORPUS_SHA256 = "ee5e7649a773f564b443320689f465080d4641a0f4f09f13c0c49a7087d6dc10"


def canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def tally_basic_records(eligible_voters: int, record_count: int, per_record_for_weight: float) -> dict:
    voter_count = record_count
    votes_for = record_count * per_record_for_weight
    participation_rate = voter_count / eligible_voters
    required_voter_count = max(math.ceil(eligible_voters * 0.15), 3)
    quorum_reached = participation_rate >= 0.15 and voter_count >= required_voter_count
    approval_rate = 1.0 if votes_for > 0.0 else 0.0
    approved = quorum_reached and approval_rate >= 0.50
    return {
        "eligible_voters": eligible_voters,
        "voter_count": voter_count,
        "phi_votes_for": votes_for,
        "phi_votes_against": 0.0,
        "participation_rate": participation_rate,
        "required_voter_count": required_voter_count,
        "quorum_reached": quorum_reached,
        "approval_rate": approval_rate,
        "approved": approved,
    }


def build_corpus(profile: dict) -> dict:
    validated = validate(profile)
    if validated["profile_content_sha256"] != PROFILE_SHA256:
        raise ValueError("A4 requires exact observation-v2 profile")

    delegated = profile["vote_paths"]["delegated_phi_vote"]
    direct = profile["vote_paths"]["phi_weighted_vote"]
    tally = profile["phi_tally"]

    if delegated["duplicate_voter_guard"] != "NoCoordinatorGuardObserved":
        raise ValueError("CE-06 prerequisite changed")
    if tally["defensive_voter_deduplication_observed"] is not False:
        raise ValueError("CE-06 tally prerequisite changed")
    if delegated["outgoing_allocation_conservation"] != "NotEstablishedAcrossApplicableActiveDelegations":
        raise ValueError("CE-07 conservation prerequisite changed")
    if delegated["resolver_cycle_control"] != "VisitedSetPerResolutionTraversal":
        raise ValueError("CE-07 resolver prerequisite changed")
    if direct["proposal_window"] != "verify_voting_period_fail_closed":
        raise ValueError("CE-08 direct admission prerequisite changed")
    if delegated["proposal_window"] != "NoVerifyVotingPeriodCallObserved":
        raise ValueError("CE-08 delegated admission prerequisite changed")
    if delegated["phi_threshold_admission"] != "NoMeetsThresholdCallObserved":
        raise ValueError("CE-09 delegated admission prerequisite changed")

    corpus = {
        "schema": SCHEMA,
        "authority": AUTHORITY,
        "profile": {
            "profile_id": validated["profile_id"],
            "content_sha256": validated["profile_content_sha256"],
            "authority_class": validated["authority_class"],
        },
        "predecessor": {
            "schema": "mycelix-observed-voting-counterexamples-v1",
            "corpus_sha256": PREDECESSOR_CORPUS_SHA256,
            "profile_sha256": PREDECESSOR_PROFILE_SHA256,
        },
        "counterexamples": [
            {
                "id": "CE-06",
                "revision": 1,
                "kind": "AuthorityGapCounterfactual",
                "issue": 876,
                "inputs": {
                    "tier": "Basic",
                    "eligible_voters": 10,
                    "same_voter_identity": "did:example:delegate",
                    "per_record_for_weight": 0.5,
                },
                "comparison": {
                    "one_record": tally_basic_records(10, 1, 0.5),
                    "three_same_voter_records": tally_basic_records(10, 3, 0.5),
                },
                "non_claim": (
                    "Models tally consequence if source-visible delegated duplicate gap permits "
                    "multiple linked records; not a live exploit claim."
                ),
            },
            {
                "id": "CE-07",
                "revision": 1,
                "kind": "InvariantGapCounterfactual",
                "issue": 877,
                "inputs": {
                    "delegator_source_weight": 0.5,
                    "allocations": [
                        {"delegate": "Bob", "percentage": 1.0},
                        {"delegate": "Carol", "percentage": 1.0},
                    ],
                    "scope_relation": "simultaneously_applicable",
                },
                "comparison": {
                    "bob_resolved_delegated_contribution": 0.5,
                    "carol_resolved_delegated_contribution": 0.5,
                    "total_represented_delegator_mass": 1.0,
                    "source_delegator_mass": 0.5,
                    "representation_multiple": 2.0,
                },
                "non_claim": (
                    "Isolates absence of an observed cross-resolution conservation rule; "
                    "does not choose exclusive or fractional successor semantics."
                ),
            },
            {
                "id": "CE-08",
                "revision": 1,
                "kind": "AdmissionGapCounterfactual",
                "issue": 892,
                "inputs": {"proposal_window_state": "Closed"},
                "comparison": {
                    "direct_phi": "RejectClosedWindowByDeclaredSourcePolicy",
                    "delegated_phi": "NoObservedWindowRejectionGate",
                },
                "non_claim": (
                    "NoObservedWindowRejectionGate is not equivalent to accepted execution; "
                    "other runtime failures remain possible."
                ),
            },
            {
                "id": "CE-09",
                "revision": 1,
                "kind": "AdmissionGapCounterfactual",
                "issue": 892,
                "inputs": {
                    "tier": "Major",
                    "phi_provenance": "Attested",
                    "phi_score": 0.2,
                    "required_phi_threshold": 0.4,
                },
                "comparison": {
                    "direct_phi": "RejectBelowTierPhiThresholdByDeclaredSourcePolicy",
                    "delegated_phi": "NoObservedPhiThresholdRejectionGate",
                },
                "non_claim": (
                    "NoObservedPhiThresholdRejectionGate is not equivalent to accepted execution; "
                    "this is a source-policy differential."
                ),
            },
        ],
        "non_claims": [
            "no_live_exploit_claim",
            "no_production_impact_estimate",
            "no_fairness_claim",
            "no_governance_safety_claim",
            "no_policy_migration_authority",
        ],
    }
    corpus["corpus_sha256"] = hashlib.sha256(canonical(corpus)).hexdigest()
    return corpus


def self_test(profile: dict) -> dict:
    corpus = build_corpus(profile)
    if corpus["corpus_sha256"] != EXPECTED_CORPUS_SHA256:
        raise AssertionError("A4 corpus commitment drift")

    by_id = {item["id"]: item for item in corpus["counterexamples"]}
    if set(by_id) != {"CE-06", "CE-07", "CE-08", "CE-09"}:
        raise AssertionError("counterexample set drift")

    ce6 = by_id["CE-06"]["comparison"]
    assert ce6["one_record"]["approved"] is False
    assert ce6["one_record"]["required_voter_count"] == 3
    assert ce6["three_same_voter_records"]["voter_count"] == 3
    assert ce6["three_same_voter_records"]["phi_votes_for"] == 1.5
    assert ce6["three_same_voter_records"]["approved"] is True

    ce7 = by_id["CE-07"]["comparison"]
    assert ce7["source_delegator_mass"] == 0.5
    assert ce7["total_represented_delegator_mass"] == 1.0
    assert ce7["representation_multiple"] == 2.0

    ce8 = by_id["CE-08"]["comparison"]
    assert ce8["direct_phi"] == "RejectClosedWindowByDeclaredSourcePolicy"
    assert ce8["delegated_phi"] == "NoObservedWindowRejectionGate"

    ce9 = by_id["CE-09"]["comparison"]
    assert ce9["direct_phi"] == "RejectBelowTierPhiThresholdByDeclaredSourcePolicy"
    assert ce9["delegated_phi"] == "NoObservedPhiThresholdRejectionGate"

    assert canonical(corpus) == canonical(build_corpus(profile))
    return {
        "authority": AUTHORITY,
        "schema": SCHEMA,
        "profile_sha256": PROFILE_SHA256,
        "predecessor_corpus_sha256": PREDECESSOR_CORPUS_SHA256,
        "corpus_sha256": EXPECTED_CORPUS_SHA256,
        "counterexample_count": 4,
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

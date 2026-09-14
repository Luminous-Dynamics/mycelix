#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from validate_ig007a1_observed_profile import load, validate

AUTHORITY = "MeasurementOnly"
SCHEMA = "mycelix-observed-voting-counterexamples-v1"
PROFILE_SHA256 = "cbbbb3553ce465be989b5f096364ea97ccc9b1c5d6aae67d80177df8d8109763"
EXPECTED_CORPUS_SHA256 = "bb1cdcfe2205bcf6e6d718b536cbb73ba29a21c9da7dfa664798d02f9f866d90"


def canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def multiplicative(phi: float, k: float, stake: float, participation: float, domain: float, provenance: str) -> float:
    consciousness = 1.0 if provenance == "Unavailable" else 0.7 + 0.3 * phi
    value = k * k * consciousness * (1.0 + 0.1 * participation) * (1.0 + 0.05 * stake) * (1.0 + 0.1 * domain)
    return min(1.5, max(0.1, value))


def additive(phi: float, k: float, stake: float, participation: float, domain: float) -> float:
    return 0.30 * phi + 0.25 * k + 0.20 * min(stake, 1.0) + 0.15 * participation + 0.10 * domain


def next_tier(tier: str) -> str:
    return {"Basic": "Major", "Major": "Constitutional", "Constitutional": "Constitutional"}[tier]


def effective_quorum_count(profile: dict, tier: str, eligible_voters: int) -> int:
    policy = profile["tier_policy"][tier]
    return max(math.ceil(eligible_voters * policy["quorum_fraction"]), policy["absolute_quorum_floor"])


def tally(profile: dict, input_tier: str, eligible_voters: int, voter_count: int, votes_for: float, votes_against: float, *, ethics_blocked: bool = False, coherent_effective_floor: bool = False) -> dict:
    effective_tier = next_tier(input_tier) if ethics_blocked else input_tier
    effective_policy = profile["tier_policy"][effective_tier]
    count_source_tier = effective_tier if coherent_effective_floor else input_tier
    participation_rate = voter_count / eligible_voters
    required_count = effective_quorum_count(profile, count_source_tier, eligible_voters)
    quorum_reached = participation_rate >= effective_policy["quorum_fraction"] and voter_count >= required_count
    decisive = votes_for + votes_against
    approval_rate = votes_for / decisive if decisive > 0.0 else 0.0
    approved = quorum_reached and approval_rate >= effective_policy["approval_threshold"]
    return {
        "input_tier": input_tier,
        "effective_tier": effective_tier,
        "required_count_source_tier": count_source_tier,
        "eligible_voters": eligible_voters,
        "voter_count": voter_count,
        "participation_rate": participation_rate,
        "required_quorum_fraction": effective_policy["quorum_fraction"],
        "required_voter_count": required_count,
        "approval_rate": approval_rate,
        "required_approval": effective_policy["approval_threshold"],
        "quorum_reached": quorum_reached,
        "approved": approved,
    }


def build_corpus(profile: dict) -> dict:
    validated = validate(profile)
    if validated["profile_content_sha256"] != PROFILE_SHA256:
        raise ValueError("counterexample corpus requires the exact frozen observed profile")

    corpus = {
        "schema": SCHEMA,
        "authority": AUTHORITY,
        "profile": {
            "profile_id": validated["profile_id"],
            "content_sha256": validated["profile_content_sha256"],
            "authority_class": validated["authority_class"],
        },
        "counterexamples": [
            {
                "id": "CE-01",
                "revision": 1,
                "kind": "ObservedInconsistency",
                "issue": 851,
                "inputs": {"phi": 1.0, "k_trust": 0.25, "stake": 1.0, "participation": 1.0, "domain": 1.0, "phi_provenance": "Attested"},
                "comparison": {
                    "legacy_direct_multiplicative": multiplicative(1.0, 0.25, 1.0, 1.0, 1.0, "Attested"),
                    "phi_weighted_additive": additive(1.0, 0.25, 1.0, 1.0, 1.0),
                },
                "non_claim": "Path-dependent weight divergence; no fairness verdict.",
            },
            {
                "id": "CE-02",
                "revision": 1,
                "kind": "ObservedInconsistency",
                "issue": 851,
                "inputs": {"phi_materialized": 0.0, "k_trust": 1.0, "stake": 0.0, "participation": 0.0, "domain": 1.0, "phi_provenance": "Unavailable"},
                "comparison": {
                    "legacy_direct_multiplicative_unavailable": multiplicative(0.0, 1.0, 0.0, 0.0, 1.0, "Unavailable"),
                    "phi_weighted_additive_materialized_zero": additive(0.0, 1.0, 0.0, 0.0, 1.0),
                },
                "non_claim": "Missing-evidence semantics differ; no normative preference.",
            },
            {
                "id": "CE-03",
                "revision": 1,
                "kind": "AuthorityGap",
                "issue": 855,
                "inputs": {"eligible_voters": 20, "voter_count": 3, "phi_votes_for": 0.55, "phi_votes_against": 0.45, "ethics_blocked": False},
                "comparison": {
                    "caller_basic": tally(profile, "Basic", 20, 3, 0.55, 0.45),
                    "caller_constitutional": tally(profile, "Constitutional", 20, 3, 0.55, 0.45),
                },
                "non_claim": "Shows caller-selected tier is outcome-pivotal; does not assert an authoritative mapping.",
            },
            {
                "id": "CE-04",
                "revision": 1,
                "kind": "AuthorityGap",
                "issue": 856,
                "inputs": {"input_tier": "Basic", "eligible_voters": 12, "voter_count": 3, "phi_votes_for": 0.7, "phi_votes_against": 0.3, "ethics_blocked": True},
                "comparison": {
                    "observed_mixed_escalation": tally(profile, "Basic", 12, 3, 0.7, 0.3, ethics_blocked=True, coherent_effective_floor=False),
                    "coherent_effective_tier_counterfactual": tally(profile, "Basic", 12, 3, 0.7, 0.3, ethics_blocked=True, coherent_effective_floor=True),
                },
                "non_claim": "Counterfactual isolates mixed-tier floor application; it does not authorize a production fix.",
            },
            {
                "id": "CE-05",
                "revision": 1,
                "kind": "PolicyConsequence",
                "issue": None,
                "inputs": {"tier": "Major", "eligible_voters": 20, "voter_count": 5, "raw_for": 1, "raw_against": 0, "raw_abstain": 4, "phi_votes_for": 0.5, "phi_votes_against": 0.0},
                "comparison": {"observed_tally": tally(profile, "Major", 20, 5, 0.5, 0.0)},
                "non_claim": "Abstentions contribute to participation quorum but not approval denominator; not automatically classified as a bug.",
            },
        ],
        "non_claims": ["no_live_exploit_claim", "no_fairness_claim", "no_governance_safety_claim", "no_policy_migration_authority"],
    }
    corpus["corpus_sha256"] = hashlib.sha256(canonical(corpus)).hexdigest()
    return corpus


def self_test(profile: dict) -> dict:
    corpus = build_corpus(profile)
    if corpus["corpus_sha256"] != EXPECTED_CORPUS_SHA256:
        raise AssertionError("counterexample corpus commitment drift")
    by_id = {item["id"]: item for item in corpus["counterexamples"]}
    assert set(by_id) == {"CE-01", "CE-02", "CE-03", "CE-04", "CE-05"}
    assert by_id["CE-01"]["comparison"] == {"legacy_direct_multiplicative": 0.1, "phi_weighted_additive": 0.8125}
    assert by_id["CE-02"]["comparison"] == {"legacy_direct_multiplicative_unavailable": 1.1, "phi_weighted_additive_materialized_zero": 0.35}
    assert by_id["CE-03"]["comparison"]["caller_basic"]["approved"] is True
    assert by_id["CE-03"]["comparison"]["caller_constitutional"]["approved"] is False
    assert by_id["CE-04"]["comparison"]["observed_mixed_escalation"]["required_voter_count"] == 3
    assert by_id["CE-04"]["comparison"]["observed_mixed_escalation"]["approved"] is True
    assert by_id["CE-04"]["comparison"]["coherent_effective_tier_counterfactual"]["required_voter_count"] == 5
    assert by_id["CE-04"]["comparison"]["coherent_effective_tier_counterfactual"]["approved"] is False
    assert by_id["CE-05"]["comparison"]["observed_tally"]["quorum_reached"] is True
    assert by_id["CE-05"]["comparison"]["observed_tally"]["approval_rate"] == 1.0
    assert by_id["CE-05"]["comparison"]["observed_tally"]["approved"] is True
    assert canonical(corpus) == canonical(build_corpus(profile))
    return {"authority": AUTHORITY, "schema": SCHEMA, "profile_sha256": PROFILE_SHA256, "corpus_sha256": EXPECTED_CORPUS_SHA256, "counterexample_count": 5, "self_test": True}


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

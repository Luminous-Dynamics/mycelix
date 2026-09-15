#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

AUTHORITY = "ObservedSourceBound"
SCHEMA = "mycelix-governance-observed-voting-profile-v2"
EXPECTED_SHA256 = "680af4668889c299b7e0d74531f44894a64a384be71778bca54d3f21ca80ac01"
EXPECTED_SUBJECT = "fca2c107a1ea5108823ce617ba4111b6f7f77230"
EXPECTED_GAPS = {
    (851, "split_weight_authority", "Observed"),
    (855, "caller_supplied_tier", "Observed"),
    (856, "partial_ethics_escalation", "Observed"),
    (876, "delegated_duplicate_vote_binding", "Observed"),
    (877, "delegation_power_conservation", "Observed"),
    (892, "delegated_admission_gate_asymmetry", "Observed"),
}
FORBIDDEN_KEYS = {
    "safe", "fair", "meritocratic", "sybil_proof", "governance_score",
    "alignment_score", "flourishing_score", "production_tested", "exploit_confirmed",
}
TOP_KEYS = {
    "schema", "authority_class", "profile_id", "profile_revision",
    "source_binding", "weight_profiles", "vote_paths", "tier_policy",
    "phi_tally", "circuit_breaker", "known_gaps",
    "unsupported_or_unqualified", "non_claims", "profile_content_sha256",
}
SOURCE_KEYS = {
    "repository", "production_subject_sha", "files", "documentation_is_authority",
    "observation_relation_to_v1",
}
SOURCE_FILE_KEYS = {"path", "git_blob_sha1", "role"}
TIER_KEYS = {
    "phi_threshold", "quorum_fraction", "absolute_quorum_floor",
    "approval_threshold", "timelock_hours",
}


def canonical(obj: object) -> bytes:
    return json.dumps(
        obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")


def digest_payload(profile: dict) -> str:
    payload = copy.deepcopy(profile)
    payload.pop("profile_content_sha256", None)
    return hashlib.sha256(canonical(payload)).hexdigest()


def exact_keys(obj: dict, expected: set[str], label: str) -> None:
    actual = set(obj)
    if actual != expected:
        raise ValueError(
            f"{label} keys mismatch: missing={sorted(expected-actual)} "
            f"unknown={sorted(actual-expected)}"
        )


def walk_forbidden(obj: object, path: str = "$") -> None:
    if isinstance(obj, dict):
        bad = FORBIDDEN_KEYS.intersection(obj)
        if bad:
            raise ValueError(f"forbidden verdict keys at {path}: {sorted(bad)}")
        for key, value in obj.items():
            walk_forbidden(value, f"{path}.{key}")
    elif isinstance(obj, list):
        for i, value in enumerate(obj):
            walk_forbidden(value, f"{path}[{i}]")


def validate(profile: dict) -> dict:
    exact_keys(profile, TOP_KEYS, "profile")
    if profile["schema"] != SCHEMA:
        raise ValueError("unexpected schema")
    if profile["authority_class"] != AUTHORITY:
        raise ValueError("authority must remain ObservedSourceBound")
    if profile["profile_id"] != "mycelix-voting-observed-fca2c107-v2":
        raise ValueError("unexpected profile id")
    if profile["profile_revision"] != 2:
        raise ValueError("unexpected profile revision")
    walk_forbidden(profile)

    source = profile["source_binding"]
    exact_keys(source, SOURCE_KEYS, "source_binding")
    if source["repository"] != "Luminous-Dynamics/mycelix":
        raise ValueError("unexpected repository")
    if source["production_subject_sha"] != EXPECTED_SUBJECT:
        raise ValueError("unexpected production subject")
    if source["documentation_is_authority"] is not False:
        raise ValueError("documentation must not be executable authority")
    if source["observation_relation_to_v1"] != "SameSourceSubjectMoreCompleteObservation":
        raise ValueError("v2 must remain an observation refinement, not a production successor")
    if len(source["files"]) != 2:
        raise ValueError("v2 requires exactly two bound production files")
    expected_blobs = {
        "mycelix-governance/zomes/voting/coordinator/src/lib.rs": "969b845e6186cbcad507c742a718060844f82eb2",
        "mycelix-governance/zomes/voting/integrity/src/lib.rs": "658562c8dfaf6a2f1b97a7bfd5cf0fc8a5ab6e66",
    }
    seen = {}
    for item in source["files"]:
        exact_keys(item, SOURCE_FILE_KEYS, "source file")
        seen[item["path"]] = item["git_blob_sha1"]
    if seen != expected_blobs:
        raise ValueError("bound source blob set drift")

    weights = profile["weight_profiles"]
    if set(weights) != {
        "multiplicative-bounded-v1", "additive-composite-v1", "zk-eligibility-proof-v1"
    }:
        raise ValueError("weight-profile set drift")
    if weights["multiplicative-bounded-v1"]["provenance_sensitive"] is not True:
        raise ValueError("multiplicative provenance semantics lost")
    if weights["additive-composite-v1"]["provenance_sensitive"] is not False:
        raise ValueError("additive provenance observation changed")

    paths = profile["vote_paths"]
    direct = paths["phi_weighted_vote"]
    delegated = paths["delegated_phi_vote"]
    if direct["weight_profile"] != "additive-composite-v1":
        raise ValueError("direct Phi weight profile drift")
    if direct["proposal_window"] != "verify_voting_period_fail_closed":
        raise ValueError("direct Phi proposal-window observation drift")
    if direct["phi_threshold_admission"] != "AvailablePhiMustMeetCallerTier;UnavailableSkipsGate":
        raise ValueError("direct Phi threshold observation drift")
    if direct["duplicate_voter_guard"] != "AgentNamespacePlusPhiVoterLink":
        raise ValueError("direct Phi duplicate guard observation drift")

    if delegated["weight_profile"] != "additive-composite-v1":
        raise ValueError("delegated weight profile drift")
    if delegated["tier_selection"] != "CallerSupplied":
        raise ValueError("delegated caller-tier observation must remain explicit")
    if delegated["proposal_window"] != "NoVerifyVotingPeriodCallObserved":
        raise ValueError("do not silently add proposal-window admission to observed v2")
    if delegated["phi_threshold_admission"] != "NoMeetsThresholdCallObserved":
        raise ValueError("do not silently add Phi threshold admission to observed v2")
    if delegated["duplicate_voter_guard"] != "NoCoordinatorGuardObserved":
        raise ValueError("do not silently add duplicate binding to observed v2")
    if delegated["agent_vote_limit_namespace"] is not None:
        raise ValueError("delegated agent vote-limit namespace must remain absent in observed v2")
    if delegated["authorship_binding"] != "VoterMustEqualCommittingAgent":
        raise ValueError("delegated authorship observation drift")
    if delegated["outgoing_allocation_conservation"] != "NotEstablishedAcrossApplicableActiveDelegations":
        raise ValueError("do not silently assert delegation conservation")
    if delegated["resolver_cycle_control"] != "VisitedSetPerResolutionTraversal":
        raise ValueError("resolver cycle-control observation drift")

    expected_tiers = {
        "Basic": (0.3, 0.15, 3, 0.50, 24),
        "Major": (0.4, 0.25, 5, 0.60, 72),
        "Constitutional": (0.6, 0.40, 10, 0.67, 168),
    }
    if set(profile["tier_policy"]) != set(expected_tiers):
        raise ValueError("tier set drift")
    for name, expected in expected_tiers.items():
        tier = profile["tier_policy"][name]
        exact_keys(tier, TIER_KEYS, f"tier.{name}")
        observed = (
            tier["phi_threshold"], tier["quorum_fraction"],
            tier["absolute_quorum_floor"], tier["approval_threshold"],
            tier["timelock_hours"],
        )
        if observed != expected:
            raise ValueError(f"{name} policy drift")

    tally = profile["phi_tally"]
    if tally["tier_selection"] != "CallerSupplied":
        raise ValueError("tally caller-tier gap must remain explicit")
    if tally["defensive_voter_deduplication_observed"] is not False:
        raise ValueError("do not silently assert tally deduplication")
    escalation = tally["ethics_blocked_behavior"]
    if escalation["quorum_fraction_source"] != "EffectiveTier":
        raise ValueError("ethics quorum source drift")
    if escalation["approval_threshold_source"] != "EffectiveTier":
        raise ValueError("ethics approval source drift")
    if escalation["absolute_quorum_floor_source"] != "OriginalInputTier":
        raise ValueError("partial escalation gap must remain explicit")

    gaps = {(g["issue"], g["id"], g["status"]) for g in profile["known_gaps"]}
    if gaps != EXPECTED_GAPS:
        raise ValueError("known-gap set drift")

    actual = digest_payload(profile)
    if profile["profile_content_sha256"] != actual:
        raise ValueError(f"profile commitment mismatch: {actual}")
    if actual != EXPECTED_SHA256:
        raise ValueError("profile differs from frozen v2 commitment")

    return {
        "authority_class": AUTHORITY,
        "schema": SCHEMA,
        "profile_id": profile["profile_id"],
        "profile_revision": 2,
        "profile_content_sha256": actual,
        "known_gap_issues": [851, 855, 856, 876, 877, 892],
        "validated": True,
    }


def load(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        value = json.load(f)
    if not isinstance(value, dict):
        raise ValueError("profile root must be an object")
    return value


def self_test(profile: dict) -> dict:
    summary = validate(profile)
    baseline = digest_payload(profile)
    mutations = []

    def must_change(mutator) -> None:
        candidate = copy.deepcopy(profile)
        candidate.pop("profile_content_sha256", None)
        mutator(candidate)
        after = hashlib.sha256(canonical(candidate)).hexdigest()
        if after == baseline:
            raise AssertionError("semantic mutation did not change profile identity")
        mutations.append(after)

    must_change(lambda p: p["source_binding"]["files"][0].update(git_blob_sha1="0" * 40))
    must_change(lambda p: p["vote_paths"]["delegated_phi_vote"].update(proposal_window="verify_voting_period_fail_closed"))
    must_change(lambda p: p["vote_paths"]["delegated_phi_vote"].update(phi_threshold_admission="AvailablePhiMustMeetCallerTier"))
    must_change(lambda p: p["vote_paths"]["delegated_phi_vote"].update(duplicate_voter_guard="AgentNamespacePlusPhiVoterLink"))
    must_change(lambda p: p["vote_paths"]["delegated_phi_vote"].update(outgoing_allocation_conservation="FractionalConserved"))
    must_change(lambda p: p["vote_paths"]["delegated_phi_vote"].update(resolver_cycle_control="GlobalAcrossProposal"))

    for mutate in [
        lambda p: p["known_gaps"].pop(),
        lambda p: p.update(safe=True),
    ]:
        forged = copy.deepcopy(profile)
        mutate(forged)
        try:
            validate(forged)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid mutation was accepted")

    return {
        **summary,
        "self_test": True,
        "semantic_mutation_commitments": mutations,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    profile = load(args.profile)
    out = self_test(profile) if args.self_test else validate(profile)
    print(json.dumps(out, sort_keys=True, separators=(",", ":"), allow_nan=False))


if __name__ == "__main__":
    main()

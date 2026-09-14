#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

AUTHORITY = "ObservedSourceBound"
SCHEMA = "mycelix-governance-observed-voting-profile-v1"
EXPECTED_SHA256 = "cbbbb3553ce465be989b5f096364ea97ccc9b1c5d6aae67d80177df8d8109763"
FORBIDDEN_KEYS = {
    "safe", "fair", "meritocratic", "sybil_proof", "governance_score",
    "alignment_score", "flourishing_score", "production_tested"
}
TOP_KEYS = {
    "schema", "authority_class", "profile_id", "profile_revision",
    "source_binding", "weight_profiles", "vote_paths", "tier_policy",
    "phi_tally", "circuit_breaker", "known_gaps",
    "unsupported_or_unqualified", "non_claims", "profile_content_sha256",
}
SOURCE_KEYS = {"repository", "production_subject_sha", "files", "documentation_is_authority"}
SOURCE_FILE_KEYS = {"path", "git_blob_sha1", "role"}
TIER_KEYS = {"phi_threshold", "quorum_fraction", "absolute_quorum_floor", "approval_threshold", "timelock_hours"}


def canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")


def digest_payload(profile: dict) -> str:
    payload = copy.deepcopy(profile)
    payload.pop("profile_content_sha256", None)
    return hashlib.sha256(canonical(payload)).hexdigest()


def exact_keys(obj: dict, expected: set[str], label: str) -> None:
    actual = set(obj)
    if actual != expected:
        raise ValueError(f"{label} keys mismatch: missing={sorted(expected-actual)} unknown={sorted(actual-expected)}")


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
    if profile["profile_revision"] != 1:
        raise ValueError("unexpected profile revision")
    if not profile["profile_id"]:
        raise ValueError("profile id must be non-empty")
    walk_forbidden(profile)

    source = profile["source_binding"]
    exact_keys(source, SOURCE_KEYS, "source_binding")
    if source["repository"] != "Luminous-Dynamics/mycelix":
        raise ValueError("unexpected repository")
    if source["production_subject_sha"] != "fca2c107a1ea5108823ce617ba4111b6f7f77230":
        raise ValueError("unexpected production subject")
    if source["documentation_is_authority"] is not False:
        raise ValueError("documentation must not be executable authority")
    if len(source["files"]) != 2:
        raise ValueError("v1 requires exactly two bound production files")
    for item in source["files"]:
        exact_keys(item, SOURCE_FILE_KEYS, "source file")
        if len(item["git_blob_sha1"]) != 40:
            raise ValueError("invalid git blob id")

    weights = profile["weight_profiles"]
    if set(weights) != {"multiplicative-bounded-v1", "additive-composite-v1", "zk-eligibility-proof-v1"}:
        raise ValueError("weight-profile set drift")
    if weights["multiplicative-bounded-v1"]["provenance_sensitive"] is not True:
        raise ValueError("multiplicative missing-Phi semantics lost")
    if weights["additive-composite-v1"]["provenance_sensitive"] is not False:
        raise ValueError("additive provenance observation changed")
    if weights["additive-composite-v1"]["unavailable_phi_materialization"] != 0.0:
        raise ValueError("unavailable Phi materialization changed")

    paths = profile["vote_paths"]
    if paths["legacy_direct_vote"]["weight_profile"] != "multiplicative-bounded-v1":
        raise ValueError("legacy weight authority drift")
    if paths["phi_weighted_vote"]["weight_profile"] != "additive-composite-v1":
        raise ValueError("Phi weight authority drift")
    if paths["phi_weighted_vote"]["tier_selection"] != "CallerSupplied":
        raise ValueError("observed caller-supplied tier gap must remain explicit")
    if paths["phi_weighted_vote"]["authoritative_proposal_type_usage"] != "FetchedThenDiscardedForTierSelection":
        raise ValueError("proposal-type authority observation drift")
    if paths["delegated_phi_vote"]["weight_profile"] != "additive-composite-v1":
        raise ValueError("delegation weight authority drift")

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
    escalation = tally["ethics_blocked_behavior"]
    if escalation["quorum_fraction_source"] != "EffectiveTier":
        raise ValueError("ethics quorum source drift")
    if escalation["approval_threshold_source"] != "EffectiveTier":
        raise ValueError("ethics approval source drift")
    if escalation["absolute_quorum_floor_source"] != "OriginalInputTier":
        raise ValueError("partial escalation gap must remain explicit")

    gaps = {(g["issue"], g["id"], g["status"]) for g in profile["known_gaps"]}
    expected_gaps = {
        (851, "split_weight_authority", "Observed"),
        (855, "caller_supplied_tier", "Observed"),
        (856, "partial_ethics_escalation", "Observed"),
    }
    if gaps != expected_gaps:
        raise ValueError("known-gap set drift")

    actual = digest_payload(profile)
    if profile["profile_content_sha256"] != actual:
        raise ValueError(f"profile commitment mismatch: {actual}")
    if actual != EXPECTED_SHA256:
        raise ValueError("profile differs from frozen v1 commitment")

    return {
        "authority_class": AUTHORITY,
        "schema": SCHEMA,
        "profile_id": profile["profile_id"],
        "profile_content_sha256": actual,
        "known_gap_issues": [851, 855, 856],
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
    mutations = []

    def changed(mutator):
        candidate = copy.deepcopy(profile)
        candidate.pop("profile_content_sha256", None)
        before = digest_payload(profile)
        mutator(candidate)
        after = hashlib.sha256(canonical(candidate)).hexdigest()
        if before == after:
            raise AssertionError("semantic mutation did not change commitment")
        mutations.append(after)

    changed(lambda p: p["source_binding"]["files"][0].update(git_blob_sha1="0" * 40))
    changed(lambda p: p["vote_paths"]["phi_weighted_vote"].update(weight_profile="multiplicative-bounded-v1"))
    changed(lambda p: p["vote_paths"]["phi_weighted_vote"].update(tier_selection="ProposalAuthorityDerived"))
    changed(lambda p: p["phi_tally"]["ethics_blocked_behavior"].update(absolute_quorum_floor_source="EffectiveTier"))

    forged = copy.deepcopy(profile)
    forged["safe"] = True
    try:
        validate(forged)
    except ValueError:
        pass
    else:
        raise AssertionError("forbidden safe verdict was accepted")

    unknown = copy.deepcopy(profile)
    unknown["source_binding"]["mystery"] = "ignored?"
    try:
        validate(unknown)
    except ValueError:
        pass
    else:
        raise AssertionError("unknown source-binding key was accepted")

    return {**summary, "self_test": True, "semantic_mutation_commitments": mutations}


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

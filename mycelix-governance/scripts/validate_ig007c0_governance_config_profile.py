#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

SCHEMA = "mycelix-governance-observed-config-authority-profile-v1"
AUTHORITY = "ObservedSourceBound"
PROFILE_ID = "mycelix-governance-config-observed-fca2c107-v1"
EXPECTED_SHA256 = "de4435a69356557b1812f8beb46d654c66b9c957be9d18c64bd0431f92546d5a"
EXPECTED_SUBJECT = "fca2c107a1ea5108823ce617ba4111b6f7f77230"
EXPECTED_FILES = {
    "mycelix-governance/zomes/bridge/coordinator/src/consciousness_config.rs": "26da234e588bf26d0e25c10dbec34502e00c191a",
    "mycelix-governance/zomes/bridge/integrity/src/lib.rs": "61b20610216e2fd69c701ecaea5322c448eda119",
    "mycelix-governance/zomes/proposals/coordinator/src/lib.rs": "eb8358353ee259ef9c3b46617a61d3439f1c714c",
    "mycelix-governance/zomes/proposals/integrity/src/lib.rs": "986bc0526aec8d37436efbe5ba798bc41705e3cf",
}
TOP_KEYS = {
    "schema", "authority_class", "profile_id", "profile_revision",
    "source_binding", "runtime_read_authority", "declared_design",
    "observed_update_predicates", "integrity_authorization",
    "policy_effect_observation", "known_gaps", "unsupported_or_unqualified",
    "non_claims", "profile_content_sha256",
}
FORBIDDEN_KEYS = {
    "safe", "secure", "fair", "sybil_proof", "governance_score",
    "production_tested", "exploit_confirmed", "deployment_current",
}


def canonical(obj: object) -> bytes:
    return json.dumps(
        obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")


def payload_digest(profile: dict) -> str:
    payload = copy.deepcopy(profile)
    payload.pop("profile_content_sha256", None)
    return hashlib.sha256(canonical(payload)).hexdigest()


def load(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        obj = json.load(f)
    if not isinstance(obj, dict):
        raise ValueError("profile root must be object")
    return obj


def walk_forbidden(obj: object, path: str = "$") -> None:
    if isinstance(obj, dict):
        bad = FORBIDDEN_KEYS.intersection(obj)
        if bad:
            raise ValueError(f"forbidden verdict fields at {path}: {sorted(bad)}")
        for key, value in obj.items():
            walk_forbidden(value, f"{path}.{key}")
    elif isinstance(obj, list):
        for index, value in enumerate(obj):
            walk_forbidden(value, f"{path}[{index}]")


def validate(profile: dict) -> dict:
    if set(profile) != TOP_KEYS:
        raise ValueError("profile top-level keys drift")
    if profile["schema"] != SCHEMA:
        raise ValueError("unexpected schema")
    if profile["authority_class"] != AUTHORITY:
        raise ValueError("authority must remain ObservedSourceBound")
    if profile["profile_id"] != PROFILE_ID or profile["profile_revision"] != 1:
        raise ValueError("unexpected profile identity")
    walk_forbidden(profile)

    source = profile["source_binding"]
    if source.get("repository") != "Luminous-Dynamics/mycelix":
        raise ValueError("repository drift")
    if source.get("production_subject_sha") != EXPECTED_SUBJECT:
        raise ValueError("production subject drift")
    if source.get("documentation_is_authority") is not False:
        raise ValueError("documentation cannot be executable authority")
    files = {
        item.get("path"): item.get("git_blob_sha1")
        for item in source.get("files", [])
        if isinstance(item, dict)
    }
    if files != EXPECTED_FILES:
        raise ValueError("source blob binding drift")

    runtime = profile["runtime_read_authority"]
    if runtime != {
        "config_type": "GovernanceConsciousnessConfig",
        "dynamic_gate_reader": "get_dynamic_consciousness_gate",
        "dynamic_voter_reader": "get_dynamic_min_voter_consciousness",
        "fallback_when_no_config": "HardcodedDefaults",
    }:
        raise ValueError("runtime authority observation drift")

    declared = profile["declared_design"]
    if declared != {
        "evidence_class": "DeclaredDesign",
        "update_entrypoint": "update_consciousness_config",
        "claimed_authorization": [
            "ProposalExists", "ProposalApproved", "ProposalTypeConstitutional"
        ],
    }:
        raise ValueError("declared-design observation drift")

    observed = profile["observed_update_predicates"]
    expected_observed = {
        "proposal_id": "RequiredNonEmpty",
        "proposal_lookup": "proposals::get_proposal",
        "proposal_response": "DecodeOptionRecord",
        "proposal_record_requirement": "SomeRecord",
        "proposal_status_check": "NoneObserved",
        "proposal_type_check": "NoneObserved",
        "exact_action_binding": "NoneObserved",
        "caller_role_binding": "NoneObserved",
        "execution_signature_receipt_binding": "NoneObserved",
        "config_validation": "RangeAndMonotonicity",
        "hardcoded_policy_bounds": "UpperBoundsOnSeveralGateValuesAndMaxWeightCeiling",
    }
    if observed != expected_observed:
        raise ValueError("observed update-predicate drift")

    integrity = profile["integrity_authorization"]
    if integrity != {
        "create_update_validator": "check_consciousness_config",
        "changed_by_proposal_binding": "NoneObserved",
        "proposal_state_binding": "NoneObserved",
        "proposal_type_binding": "NoneObserved",
        "entry_author_authorization": "NoneObserved",
    }:
        raise ValueError("integrity authorization observation drift")

    effect = profile["policy_effect_observation"]
    if effect != {
        "lower_gate_structurally_valid_test": {
            "field": "consciousness_gate_basic",
            "value": 0.1,
            "result": "ValidUnderCheckConsciousnessConfig",
        },
        "interpretation": "AuthorizationWeaknessCanAffectRuntimeParticipationThresholds",
    }:
        raise ValueError("policy-effect observation drift")

    gaps = profile["known_gaps"]
    if gaps != [{"issue": 943, "status": "Observed", "class": "AuthorizationPredicateGap"}]:
        raise ValueError("known-gap set drift")

    unsupported = set(profile["unsupported_or_unqualified"])
    required_unsupported = {
        "LiveUnauthorizedMutation", "DeploymentCurrentnessQualified",
        "GovernanceSafety", "ExploitSuccess", "ProposalApprovalVerified",
        "ConstitutionalProposalTypeVerified",
    }
    if unsupported != required_unsupported:
        raise ValueError("unsupported/unqualified claim set drift")

    actual = payload_digest(profile)
    if profile["profile_content_sha256"] != actual:
        raise ValueError(f"content commitment mismatch: {actual}")
    if actual != EXPECTED_SHA256:
        raise ValueError("profile differs from frozen commitment")

    return {
        "validated": True,
        "authority_class": AUTHORITY,
        "profile_id": PROFILE_ID,
        "profile_revision": 1,
        "profile_content_sha256": actual,
        "known_gap_issue": 943,
    }


def self_test(profile: dict) -> dict:
    out = validate(profile)
    baseline = payload_digest(profile)

    def assert_identity_changes(mutator) -> None:
        candidate = copy.deepcopy(profile)
        candidate.pop("profile_content_sha256", None)
        mutator(candidate)
        if hashlib.sha256(canonical(candidate)).hexdigest() == baseline:
            raise AssertionError("semantic mutation did not change identity")

    assert_identity_changes(
        lambda p: p["source_binding"]["files"][0].update(git_blob_sha1="0" * 40)
    )
    assert_identity_changes(
        lambda p: p["observed_update_predicates"].update(proposal_status_check="Approved")
    )
    assert_identity_changes(
        lambda p: p["observed_update_predicates"].update(proposal_type_check="Constitutional")
    )
    assert_identity_changes(
        lambda p: p["integrity_authorization"].update(changed_by_proposal_binding="Verified")
    )
    assert_identity_changes(
        lambda p: p["policy_effect_observation"]["lower_gate_structurally_valid_test"].update(value=0.2)
    )

    invalid_mutations = [
        lambda p: p.update(safe=True),
        lambda p: p.update(authority_class="ExecutableQualified"),
        lambda p: p["known_gaps"].clear(),
        lambda p: p["declared_design"].update(evidence_class="ObservedExecutable"),
    ]
    for mutator in invalid_mutations:
        candidate = copy.deepcopy(profile)
        mutator(candidate)
        try:
            validate(candidate)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid mutation accepted")

    return {**out, "self_test": True}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    profile = load(args.profile)
    result = self_test(profile) if args.self_test else validate(profile)
    print(json.dumps(result, sort_keys=True, separators=(",", ":"), allow_nan=False))


if __name__ == "__main__":
    main()

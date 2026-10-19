#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

PROFILE_ID = "mycelix-threshold-signing-observed-fca2c107-v1"
PROFILE_SHA256 = "c15dfd860b759747938af2a13129d729fa0af1e75284418c9ea6b9c172f643ac"
SUBJECT = "fca2c107a1ea5108823ce617ba4111b6f7f77230"
CURRENT_MAIN = "31ede2365b81365bb119cd9351b2739119974130"
FILES = {
    "mycelix-governance/zomes/threshold-signing/coordinator/src/lib.rs": "3449df8b03a4dd1774a5f22756d06931c72855b2",
    "mycelix-governance/zomes/threshold-signing/integrity/src/lib.rs": "3fec8344635600c044a494fa72bbbfe408fbe5ec",
    "mycelix-governance/zomes/proposals/coordinator/src/lib.rs": "eb8358353ee259ef9c3b46617a61d3439f1c714c",
    "mycelix-governance/zomes/execution/coordinator/src/lib.rs": "3dbb8a8f69b377e494ccf24164c94bd80f54e0ef",
}
TOP_KEYS = {
    "schema", "authority_class", "profile_id", "profile_revision", "source_binding",
    "producer_api", "threshold_signature_integrity", "proposal_to_signature_link",
    "source_test_observation", "known_gaps", "unsupported_or_unqualified",
    "non_claims", "profile_content_sha256",
}
FORBIDDEN = {
    "signature_secure", "cryptographically_verified", "governance_safe",
    "exploit_confirmed", "deployment_current", "threshold_authorized",
}


def canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode()


def digest(profile: dict) -> str:
    payload = copy.deepcopy(profile)
    payload.pop("profile_content_sha256", None)
    return hashlib.sha256(canonical(payload)).hexdigest()


def load(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        obj = json.load(f)
    if not isinstance(obj, dict):
        raise ValueError("profile root must be object")
    return obj


def forbid(obj: object, path: str = "$") -> None:
    if isinstance(obj, dict):
        bad = FORBIDDEN.intersection(obj)
        if bad:
            raise ValueError(f"forbidden verdict fields at {path}: {sorted(bad)}")
        for key, value in obj.items():
            forbid(value, f"{path}.{key}")
    elif isinstance(obj, list):
        for i, value in enumerate(obj):
            forbid(value, f"{path}[{i}]")


def validate(profile: dict) -> dict:
    if set(profile) != TOP_KEYS:
        raise ValueError("top-level profile drift")
    if profile["schema"] != "mycelix-threshold-signing-observed-profile-v1":
        raise ValueError("schema drift")
    if profile["authority_class"] != "ObservedSourceBound":
        raise ValueError("authority drift")
    if profile["profile_id"] != PROFILE_ID or profile["profile_revision"] != 1:
        raise ValueError("profile identity drift")
    forbid(profile)

    source = profile["source_binding"]
    if source.get("repository") != "Luminous-Dynamics/mycelix":
        raise ValueError("repository drift")
    if source.get("production_subject_sha") != SUBJECT:
        raise ValueError("production subject drift")
    if source.get("tree_equivalent_current_main_sha") != CURRENT_MAIN:
        raise ValueError("tree-equivalent main reference drift")
    actual_files = {
        item.get("path"): item.get("git_blob_sha1")
        for item in source.get("files", []) if isinstance(item, dict)
    }
    if actual_files != FILES:
        raise ValueError("source blob binding drift")

    producer = profile["producer_api"]
    if producer != {
        "coordinator_files": ["lib.rs"],
        "observed_externs": ["create_committee"],
        "consumer_expected_queries": ["get_proposal_signature", "get_committee"],
        "get_proposal_signature": "NoneObserved",
        "get_committee": "NoneObserved",
    }:
        raise ValueError("producer API observation drift")

    integrity = profile["threshold_signature_integrity"]
    if integrity != {
        "create_validator": "check_signature_validity",
        "create_author_binding": "NoneObserved",
        "structural_checks": [
            "SignerCountPositive",
            "SignerCountMatchesSignerListLength",
            "SignedContentHashNonEmpty",
            "AlgorithmSpecificSignaturePresenceAndLength",
        ],
        "cryptographic_signature_verification": "NoneObserved",
        "committee_lookup": "NoneObserved",
        "committee_active_epoch_check": "NoneObserved",
        "committee_threshold_check": "NoneObserved",
        "qualified_signer_membership_check": "NoneObserved",
        "committee_scope_check": "NoneObserved",
        "signed_subject_authorization_reconstruction": "NoneObserved",
        "verified_field_recomputed": "NoneObserved",
        "signature_update_policy": "ImmutableAfterCreate",
    }:
        raise ValueError("signature integrity observation drift")

    if profile["proposal_to_signature_link"] != {
        "create_validation": "UnconditionalValidObserved",
        "exact_subject_authorization_reconstruction": "NoneObserved",
    }:
        raise ValueError("proposal-signature link observation drift")

    if profile["source_test_observation"] != {
        "fixture": "make_test_signature(Ecdsa)",
        "signature_bytes": "64ZeroBytes",
        "signed_content_hash": "32NonzeroFixtureBytes",
        "signer_count": 1,
        "signers": [1],
        "verified": False,
        "pure_validator_result": "AcceptedByCheckSignatureValidity",
        "evidence_class": "StructuralValidatorTest",
    }:
        raise ValueError("source test observation drift")

    if profile["known_gaps"] != [
        {"issue": 959, "class": "ProducerConsumerApiContractGap", "status": "Observed"},
        {"issue": 960, "class": "ThresholdSignatureAuthorityGap", "status": "Observed"},
    ]:
        raise ValueError("known gap drift")

    if set(profile["unsupported_or_unqualified"]) != {
        "CryptographicallyVerifiedThresholdSignature",
        "CommitteeThresholdAuthorized",
        "CommitteeScopeAuthorized",
        "QualifiedSignerSetVerified",
        "ProposalSubjectAuthorizationVerified",
        "LiveForgedSignatureAcceptance",
        "DeploymentCurrentnessQualified",
        "GovernanceSafety",
    }:
        raise ValueError("unsupported claim boundary drift")

    actual = digest(profile)
    if profile["profile_content_sha256"] != actual or actual != PROFILE_SHA256:
        raise ValueError(f"profile commitment drift: {actual}")

    return {
        "validated": True,
        "authority_class": "ObservedSourceBound",
        "profile_id": PROFILE_ID,
        "profile_revision": 1,
        "profile_content_sha256": actual,
        "known_gap_issues": [959, 960],
    }


def self_test(profile: dict) -> dict:
    result = validate(profile)
    baseline = digest(profile)

    def identity_changes(mutator) -> None:
        candidate = copy.deepcopy(profile)
        candidate.pop("profile_content_sha256", None)
        mutator(candidate)
        if hashlib.sha256(canonical(candidate)).hexdigest() == baseline:
            raise AssertionError("semantic mutation did not change identity")

    identity_changes(lambda p: p["producer_api"].update(get_proposal_signature="Observed"))
    identity_changes(lambda p: p["threshold_signature_integrity"].update(cryptographic_signature_verification="Observed"))
    identity_changes(lambda p: p["threshold_signature_integrity"].update(verified_field_recomputed="Observed"))
    identity_changes(lambda p: p["source_test_observation"].update(verified=True))

    invalid = [
        lambda p: p.update(authority_class="ExecutableQualified"),
        lambda p: p["known_gaps"].clear(),
        lambda p: p.update(signature_secure=True),
    ]
    for mutator in invalid:
        candidate = copy.deepcopy(profile)
        mutator(candidate)
        try:
            validate(candidate)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid profile mutation accepted")

    return {**result, "self_test": True}


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

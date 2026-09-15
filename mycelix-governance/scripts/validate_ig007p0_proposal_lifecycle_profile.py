#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

PROFILE_ID = "mycelix-proposal-lifecycle-observed-fca2c107-v1"
PROFILE_SHA256 = "7f42e2a8df25df94112d23f261d1f3ffe299d46d37cb3a5a6fe02aca0aa6c108"
SUBJECT = "fca2c107a1ea5108823ce617ba4111b6f7f77230"
CURRENT_MAIN = "31ede2365b81365bb119cd9351b2739119974130"
FILES = {
    "mycelix-governance/zomes/proposals/coordinator/src/lib.rs": "eb8358353ee259ef9c3b46617a61d3439f1c714c",
    "mycelix-governance/zomes/proposals/integrity/src/lib.rs": "986bc0526aec8d37436efbe5ba798bc41705e3cf",
}
TOP_KEYS = {
    "schema",
    "authority_class",
    "profile_id",
    "profile_revision",
    "source_binding",
    "creation",
    "lookup_projection",
    "update_integrity",
    "coordinator_status_update",
    "known_gaps",
    "unsupported_or_unqualified",
    "non_claims",
    "profile_content_sha256",
}
FORBIDDEN = {
    "authoritative_current",
    "fork_resolved",
    "deployment_current",
    "governance_safe",
    "secure",
}


def canonical(obj: object) -> bytes:
    return json.dumps(
        obj,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


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
        raise ValueError("top-level proposal profile drift")
    if profile["schema"] != "mycelix-proposal-lifecycle-observed-profile-v1":
        raise ValueError("proposal profile schema drift")
    if profile["authority_class"] != "ObservedSourceBound":
        raise ValueError("proposal profile authority drift")
    if profile["profile_id"] != PROFILE_ID or profile["profile_revision"] != 1:
        raise ValueError("proposal profile identity drift")
    forbid(profile)

    source = profile["source_binding"]
    if source.get("repository") != "Luminous-Dynamics/mycelix":
        raise ValueError("proposal source repository drift")
    if source.get("production_subject_sha") != SUBJECT:
        raise ValueError("proposal production subject drift")
    if source.get("tree_equivalent_current_main_sha") != CURRENT_MAIN:
        raise ValueError("proposal tree-equivalent main reference drift")
    actual_files = {
        item.get("path"): item.get("git_blob_sha1")
        for item in source.get("files", [])
        if isinstance(item, dict)
    }
    if actual_files != FILES:
        raise ValueError("proposal source blob binding drift")

    if profile["creation"] != {
        "entrypoint": "create_proposal",
        "proposal_author_source": "CallerSuppliedButIntegrityBoundToCommitter",
        "initial_status": "Draft",
        "initial_version": 1,
        "proposal_by_id_link": "CreatedToProposalCreationAction",
        "voting_period_create_check": "VotingEndAfterVotingStart",
        "actions_json_check": "Required",
    }:
        raise ValueError("proposal creation observation drift")

    if profile["lookup_projection"] != {
        "entrypoint": "get_proposal",
        "primary_lookup": "ProposalByIdLink",
        "primary_link_selection": "MaxLinkTimestamp",
        "update_refreshes_proposal_by_id_link": "NoneObserved",
        "linked_record_return": "ReturnsLinkedRecordWithoutUpdateTraversal",
        "fallback": "LocalChainScanLastMatchingProposalOnlyWhenLinkedLookupDoesNotReturnRecord",
        "explicit_competing_update_fork_rule": "NoneObserved",
    }:
        raise ValueError("proposal lookup/projection observation drift")

    if profile["update_integrity"] != {
        "entrypoint": "validate_update_proposal",
        "update_action_author_binding": "NoneObserved",
        "immutable_fields": ["id", "author"],
        "allowed_status_transitions_shape_checked": True,
        "content_fields": ["title", "description", "actions", "proposal_type"],
        "content_freeze_condition": "OriginalStatusNotDraft",
        "draft_to_active_content_mutation_structurally_rejected": False,
        "voting_starts_immutability": "NoneObserved",
        "voting_ends_immutability": "NoneObserved",
        "created_timestamp_immutability": "NoneObserved",
        "updated_timestamp_action_binding": "NoneObserved",
        "update_voting_period_order_check": "NoneObserved",
        "version_rule": "UpdatedEqualsOriginalPlusOne",
    }:
        raise ValueError("proposal update-integrity observation drift")

    if profile["coordinator_status_update"] != {
        "entrypoint": "update_proposal_status",
        "current_record_source": "get_proposal",
        "update_target": "CurrentRecordActionAddress",
        "proposal_by_id_link_refresh_after_update": "NoneObserved",
    }:
        raise ValueError("proposal coordinator status-update observation drift")

    if profile["known_gaps"] != [
        {"issue": 66, "class": "MutableProposalProjectionAuthorityGap", "status": "Observed"}
    ]:
        raise ValueError("proposal known-gap set drift")

    expected_unsupported = {
        "AuthoritativeCurrentProposalLifecycle",
        "DeterministicProposalForkResolution",
        "ActivationContentFreezeQualified",
        "LifecycleUpdateAuthorAuthorityQualified",
        "ProposalTemporalFieldAuthorityQualified",
        "DeploymentCurrentnessQualified",
        "GovernanceSafety",
    }
    if set(profile["unsupported_or_unqualified"]) != expected_unsupported:
        raise ValueError("proposal unsupported-claim boundary drift")

    actual = digest(profile)
    if profile["profile_content_sha256"] != actual or actual != PROFILE_SHA256:
        raise ValueError(f"proposal profile commitment drift: {actual}")

    return {
        "validated": True,
        "authority_class": "ObservedSourceBound",
        "profile_id": PROFILE_ID,
        "profile_revision": 1,
        "profile_content_sha256": actual,
        "known_gap_issue": 66,
    }


def self_test(profile: dict) -> dict:
    result = validate(profile)
    baseline = digest(profile)

    def identity_changes(mutator) -> None:
        candidate = copy.deepcopy(profile)
        candidate.pop("profile_content_sha256", None)
        mutator(candidate)
        if hashlib.sha256(canonical(candidate)).hexdigest() == baseline:
            raise AssertionError("semantic proposal mutation did not change identity")

    identity_changes(lambda p: p["lookup_projection"].update(update_refreshes_proposal_by_id_link="Observed"))
    identity_changes(lambda p: p["update_integrity"].update(update_action_author_binding="CommitterBound"))
    identity_changes(lambda p: p["update_integrity"].update(draft_to_active_content_mutation_structurally_rejected=True))
    identity_changes(lambda p: p["update_integrity"].update(voting_starts_immutability="Required"))
    identity_changes(lambda p: p["lookup_projection"].update(explicit_competing_update_fork_rule="FailClosed"))

    invalid = [
        lambda p: p.update(authority_class="ExecutableQualified"),
        lambda p: p["known_gaps"].clear(),
        lambda p: p.update(authoritative_current=True),
    ]
    for mutator in invalid:
        candidate = copy.deepcopy(profile)
        mutator(candidate)
        try:
            validate(candidate)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid proposal-profile mutation accepted")

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

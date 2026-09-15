#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

PROFILE_ID = "mycelix-constitution-parameter-observed-fca2c107-v1"
PROFILE_SHA256 = "770552d12489df1d2cdf8b0af676b01ea9a3da21940f70ed8a71910deaa35009"
SUBJECT = "fca2c107a1ea5108823ce617ba4111b6f7f77230"
CURRENT_MAIN = "31ede2365b81365bb119cd9351b2739119974130"
FILES = {
    "mycelix-governance/zomes/execution/coordinator/src/lib.rs": "3dbb8a8f69b377e494ccf24164c94bd80f54e0ef",
    "mycelix-governance/zomes/constitution/coordinator/src/lib.rs": "923a1ce789c8319c79df7f33a9241af50804ec55",
    "mycelix-governance/zomes/constitution/integrity/src/lib.rs": "f83a457a8ff40b0003c07dba9da598a478c5e6f6",
}
TOP_KEYS = {
    "schema", "authority_class", "profile_id", "profile_revision", "source_binding",
    "execution_dispatch", "coordinator_update", "set_parameter_gate", "storage_projection",
    "integrity", "known_gaps", "unsupported_or_unqualified", "non_claims",
    "profile_content_sha256",
}
FORBIDDEN = {
    "authorized_parameter_mutation", "proposal_verified", "caller_authorized",
    "authoritative_current", "fork_resolved", "deployment_current", "governance_safe", "secure",
}


def canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")


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
        raise ValueError("top-level constitution-parameter profile drift")
    if profile["schema"] != "mycelix-constitution-parameter-observed-profile-v1":
        raise ValueError("constitution-parameter schema drift")
    if profile["authority_class"] != "ObservedSourceBound":
        raise ValueError("constitution-parameter authority drift")
    if profile["profile_id"] != PROFILE_ID or profile["profile_revision"] != 1:
        raise ValueError("constitution-parameter profile identity drift")
    forbid(profile)

    source = profile["source_binding"]
    if source.get("repository") != "Luminous-Dynamics/mycelix":
        raise ValueError("repository drift")
    if source.get("production_subject_sha") != SUBJECT:
        raise ValueError("production subject drift")
    if source.get("tree_equivalent_current_main_sha") != CURRENT_MAIN:
        raise ValueError("tree-equivalent main drift")
    actual_files = {
        item.get("path"): item.get("git_blob_sha1")
        for item in source.get("files", []) if isinstance(item, dict)
    }
    if actual_files != FILES:
        raise ValueError("source blob binding drift")

    if profile["execution_dispatch"] != {
        "action_variant": "UpdateParameter",
        "dispatch_target": "constitution::update_parameter",
        "payload_fields": ["parameter", "value"],
        "proposal_id_included": False,
        "qualified_authorization_ref_included": False,
    }:
        raise ValueError("execution dispatch observation drift")

    if profile["coordinator_update"] != {
        "entrypoint": "update_parameter",
        "proposal_id_input": "OptionalCallerSuppliedSerdeDefault",
        "proposal_lookup": "NoneObserved",
        "proposal_status_check": "NoneObserved",
        "proposal_type_check": "NoneObserved",
        "exact_parameter_value_authorization_binding": "NoneObserved",
        "execution_authorization_binding": "NoneObserved",
        "caller_authority_check": "NoneObserved",
        "forwards_proposal_id_to_set_parameter": True,
    }:
        raise ValueError("coordinator update observation drift")

    if profile["set_parameter_gate"] != {
        "entrypoint": "set_parameter",
        "existing_parameter_without_proposal_id": "Rejected",
        "existing_parameter_with_nonempty_or_unverified_some_proposal_id": "PresenceSatisfiesObservedCoordinatorGate",
        "new_parameter_without_proposal_id": "AllowedByObservedCoordinatorGate",
        "proposal_id_authority_reconstruction": "NoneObserved",
        "changed_by_proposal_source": "CopiedFromInputProposalId",
    }:
        raise ValueError("set_parameter gate observation drift")

    if profile["storage_projection"] != {
        "write_primitive": "CreateEntryForEachParameterWrite",
        "index_link": "ParameterIndexCreatedForEachWrite",
        "read_entrypoint": "get_parameter",
        "link_selection": "MaxLinkTimestamp",
        "explicit_authoritative_fork_rule": "NoneObserved",
        "timestamp_selection_is_authority": "NotEstablished",
    }:
        raise ValueError("parameter storage/projection drift")

    if profile["integrity"] != {
        "create_checks": ["NameNonEmpty", "ValueValidJson"],
        "update_checks": ["ValueValidJson"],
        "create_action_author_binding": "NoneObserved",
        "update_action_author_binding": "NoneObserved",
        "changed_by_proposal_authority_verification": "NoneObserved",
        "proposal_status_type_action_verification": "NoneObserved",
        "authorized_current_parameter_lineage": "NoneObserved",
    }:
        raise ValueError("parameter integrity observation drift")

    if profile["known_gaps"] != [
        {"issue": 1002, "class": "ConstitutionParameterMutationAuthorityGap", "status": "Observed"}
    ]:
        raise ValueError("known gap drift")

    if set(profile["unsupported_or_unqualified"]) != {
        "AuthorizedConstitutionParameterMutation",
        "ProposalBoundParameterMutation",
        "AuthorBoundParameterMutation",
        "DeterministicCurrentParameterProjection",
        "ExecutionToParameterAuthorityContinuity",
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
        "known_gap_issue": 1002,
    }


def self_test(profile: dict) -> dict:
    result = validate(profile)
    baseline = digest(profile)

    def identity_changes(mutator) -> None:
        candidate = copy.deepcopy(profile)
        candidate.pop("profile_content_sha256", None)
        mutator(candidate)
        if hashlib.sha256(canonical(candidate)).hexdigest() == baseline:
            raise AssertionError("semantic mutation did not change profile identity")

    identity_changes(lambda p: p["execution_dispatch"].update(proposal_id_included=True))
    identity_changes(lambda p: p["coordinator_update"].update(proposal_status_check="Observed"))
    identity_changes(lambda p: p["set_parameter_gate"].update(new_parameter_without_proposal_id="Rejected"))
    identity_changes(lambda p: p["integrity"].update(create_action_author_binding="CommitterBound"))
    identity_changes(lambda p: p["storage_projection"].update(explicit_authoritative_fork_rule="FailClosed"))

    invalid = [
        lambda p: p.update(authority_class="ExecutableQualified"),
        lambda p: p["known_gaps"].clear(),
        lambda p: p.update(authoritative_current=True),
        lambda p: p.update(governance_safe=True),
    ]
    for mutator in invalid:
        candidate = copy.deepcopy(profile)
        mutator(candidate)
        try:
            validate(candidate)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid constitution-parameter profile mutation accepted")

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

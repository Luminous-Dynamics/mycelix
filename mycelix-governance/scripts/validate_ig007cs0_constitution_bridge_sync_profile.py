#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

PROFILE_ID = "mycelix-constitution-bridge-sync-observed-fca2c107-v1"
PROFILE_SHA256 = "60daae86044098561fa8e41bcdf6f695ab41234760be4b7e235d2780271b681e"
SUBJECT = "fca2c107a1ea5108823ce617ba4111b6f7f77230"
TREE_EQUIV = "31ede2365b81365bb119cd9351b2739119974130"

CONSTITUTION = {
    "path": "mycelix-governance/zomes/constitution/coordinator/src/lib.rs",
    "git_blob_sha1": "923a1ce789c8319c79df7f33a9241af50804ec55",
}
BRIDGE = [
    {"path":"mycelix-governance/zomes/bridge/coordinator/src/attestation.rs","git_blob_sha1":"6d7938084ba699b144ee5951966b1421c034f579"},
    {"path":"mycelix-governance/zomes/bridge/coordinator/src/consciousness.rs","git_blob_sha1":"d0acabf306594ab3ccfae220a9c9bd9aceed0ca6"},
    {"path":"mycelix-governance/zomes/bridge/coordinator/src/consciousness_config.rs","git_blob_sha1":"26da234e588bf26d0e25c10dbec34502e00c191a"},
    {"path":"mycelix-governance/zomes/bridge/coordinator/src/consensus.rs","git_blob_sha1":"3842dfa365953a01ca90bb79059da53f9cb00a6f"},
    {"path":"mycelix-governance/zomes/bridge/coordinator/src/cross_cluster.rs","git_blob_sha1":"3eb0ade8633d6e711fd266bca6eb2ebab616eac2"},
    {"path":"mycelix-governance/zomes/bridge/coordinator/src/lib.rs","git_blob_sha1":"fb278023c269a89f300504c18538ab85b09f1178"},
    {"path":"mycelix-governance/zomes/bridge/coordinator/src/query.rs","git_blob_sha1":"5a02812880a2fdaa8f3ed767a4686afc00c566f9"},
    {"path":"mycelix-governance/zomes/bridge/coordinator/src/validation.rs","git_blob_sha1":"9e13ba58939880738eccb997e54f729da7a11304"},
]
MODULES = [
    "attestation.rs","consciousness.rs","consciousness_config.rs","consensus.rs",
    "cross_cluster.rs","lib.rs","query.rs","validation.rs"
]
PARAMETERS = [
    "phi_basic","phi_proposal_submission","phi_voting","phi_constitutional",
    "min_voter_phi_standard","min_voter_phi_emergency",
    "min_voter_phi_constitutional","max_voting_weight"
]
FORBIDDEN = {
    "runtime_synchronized",
    "deployment_current",
    "governance_safe",
    "safe",
    "secure",
    "authorized_sync",
}

def canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",",":"), ensure_ascii=False, allow_nan=False).encode()

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

def walk_forbidden(obj: object, path: str="$" ) -> None:
    if isinstance(obj, dict):
        bad = FORBIDDEN.intersection(obj)
        if bad:
            raise ValueError(f"forbidden verdict fields at {path}: {sorted(bad)}")
        for key, value in obj.items():
            walk_forbidden(value, f"{path}.{key}")
    elif isinstance(obj, list):
        for i, value in enumerate(obj):
            walk_forbidden(value, f"{path}[{i}]")

def validate(profile: dict) -> dict:
    if profile.get("schema") != "mycelix-constitution-bridge-sync-observed-profile-v1":
        raise ValueError("schema drift")
    if profile.get("authority_class") != "ObservedSourceBound":
        raise ValueError("authority drift")
    if profile.get("profile_id") != PROFILE_ID or profile.get("profile_revision") != 1:
        raise ValueError("profile identity drift")
    walk_forbidden(profile)

    source = profile.get("source_binding", {})
    if source.get("repository") != "Luminous-Dynamics/mycelix":
        raise ValueError("repository drift")
    if source.get("production_subject_sha") != SUBJECT:
        raise ValueError("production subject drift")
    if source.get("tree_equivalent_current_main_sha") != TREE_EQUIV:
        raise ValueError("tree-equivalent history drift")
    if source.get("constitution_coordinator") != CONSTITUTION:
        raise ValueError("constitution blob binding drift")
    if source.get("bridge_coordinator_modules") != BRIDGE:
        raise ValueError("bridge coordinator census/blob drift")

    expected_sync = {
        "caller":"sync_phi_parameter_to_bridge",
        "mapped_parameter_names":PARAMETERS,
        "target_zome":"governance_bridge",
        "target_function":"update_phi_config",
        "call_semantics":"BestEffort",
        "parameter_write_precedes_sync":True,
        "sync_failure_effect":"EmitPhiConfigSyncWarningAndRetainConstitutionParameterWrite",
        "explicit_unsynchronized_state_receipt":"NoneObserved",
        "explicit_retry_reconciliation_contract":"NoneObserved",
    }
    if profile.get("constitution_sync") != expected_sync:
        raise ValueError("constitution sync semantics drift")

    if profile.get("bridge_surface") != {
        "coordinator_module_count":8,
        "coordinator_modules":MODULES,
        "target_symbol":"update_phi_config",
        "target_symbol_occurrences_in_bound_census":0,
        "visible_runtime_config_updater":"update_consciousness_config",
        "visible_updater_module":"consciousness_config.rs",
    }:
        raise ValueError("bridge surface drift")

    if profile.get("authorization_boundary") != {
        "visible_runtime_updater_issue":943,
        "visible_runtime_updater_authority":"ObservedSourceBoundSeparateMechanism",
        "rename_target_to_visible_updater_is_sufficient_repair":False,
    }:
        raise ValueError("authorization dependency drift")

    if profile.get("known_gaps") != [
        {"issue":944,"class":"ConstitutionBridgeSyncContractMismatch","status":"Observed"},
        {"issue":943,"class":"GovernanceConfigAuthorizationGap","status":"SeparateDependency"},
    ]:
        raise ValueError("known-gap set drift")

    expected_unsupported = {
        "ConstitutionRuntimeConfigSynchronization",
        "AtomicConstitutionRuntimePolicyMutation",
        "ReconciliationReceipt",
        "RuntimeConfigCurrentnessQualified",
        "GovernanceConfigAuthorizationQualified",
        "DeploymentCurrentnessQualified",
        "GovernanceSafety",
    }
    if set(profile.get("unsupported_or_unqualified", [])) != expected_unsupported:
        raise ValueError("unsupported-claim boundary drift")

    actual = digest(profile)
    if profile.get("profile_content_sha256") != actual or actual != PROFILE_SHA256:
        raise ValueError(f"profile commitment drift: {actual}")

    return {
        "validated": True,
        "authority_class":"ObservedSourceBound",
        "profile_id":PROFILE_ID,
        "profile_revision":1,
        "profile_content_sha256":actual,
        "known_gap_issues":[943,944],
        "bridge_module_count":8,
        "target_symbol_occurrences":0,
    }

def self_test(profile: dict) -> dict:
    result = validate(profile)
    baseline = digest(profile)

    def changes(mutator) -> None:
        candidate = copy.deepcopy(profile)
        candidate.pop("profile_content_sha256", None)
        mutator(candidate)
        if hashlib.sha256(canonical(candidate)).hexdigest() == baseline:
            raise AssertionError("semantic mutation did not change identity")

    changes(lambda p: p["constitution_sync"].update(call_semantics="FailClosed"))
    changes(lambda p: p["bridge_surface"].update(target_symbol_occurrences_in_bound_census=1))
    changes(lambda p: p["authorization_boundary"].update(rename_target_to_visible_updater_is_sufficient_repair=True))
    changes(lambda p: p["constitution_sync"].update(explicit_unsynchronized_state_receipt="Observed"))

    invalid = [
        lambda p: p.update(authority_class="ExecutableQualified"),
        lambda p: p["known_gaps"].clear(),
        lambda p: p["source_binding"]["bridge_coordinator_modules"].pop(),
        lambda p: p.update(runtime_synchronized=True),
    ]
    for mutate in invalid:
        candidate = copy.deepcopy(profile)
        mutate(candidate)
        try:
            validate(candidate)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid historical sync mutation accepted")
    return {**result, "self_test": True}

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    profile = load(args.profile)
    result = self_test(profile) if args.self_test else validate(profile)
    print(json.dumps(result, sort_keys=True, separators=(",",":"), allow_nan=False))

if __name__ == "__main__":
    main()

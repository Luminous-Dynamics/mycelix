#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

SCHEMA = "mycelix-governance-observed-execution-profile-v1"
AUTHORITY = "ObservedSourceBound"
PROFILE_ID = "mycelix-execution-observed-fca2c107-v1"
EXPECTED_SHA256 = "c977bdcef9e5faac83351050999451432b618d5cc523bece804eba5dd1ae81f6"
EXPECTED_SUBJECT = "fca2c107a1ea5108823ce617ba4111b6f7f77230"
COORDINATOR_BLOB = "3dbb8a8f69b377e494ccf24164c94bd80f54e0ef"
INTEGRITY_BLOB = "657edaee9a314f100a0c4b1609a4596cf243e61d"
FORBIDDEN_KEYS = {
    "safe", "secure", "fair", "sybil_proof", "governance_score",
    "production_tested", "exploit_confirmed", "deployment_current",
}
TOP_KEYS = {
    "schema", "authority_class", "profile_id", "profile_revision",
    "source_binding", "timelock_creation", "readiness_transition",
    "execution", "action_dispatch", "known_gaps",
    "unsupported_or_unqualified", "non_claims", "profile_content_sha256",
}


def canonical(obj: object) -> bytes:
    return json.dumps(
        obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")


def payload_digest(profile: dict) -> str:
    payload = copy.deepcopy(profile)
    payload.pop("profile_content_sha256", None)
    return hashlib.sha256(canonical(payload)).hexdigest()


def walk_forbidden(obj: object, path: str = "$") -> None:
    if isinstance(obj, dict):
        bad = FORBIDDEN_KEYS.intersection(obj)
        if bad:
            raise ValueError(f"forbidden verdict fields at {path}: {sorted(bad)}")
        for key, value in obj.items():
            walk_forbidden(value, f"{path}.{key}")
    elif isinstance(obj, list):
        for i, value in enumerate(obj):
            walk_forbidden(value, f"{path}[{i}]")


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
        raise ValueError("unexpected repository")
    if source.get("production_subject_sha") != EXPECTED_SUBJECT:
        raise ValueError("unexpected production subject")
    if source.get("documentation_is_authority") is not False:
        raise ValueError("documentation cannot be executable authority")
    files = {
        item.get("path"): item.get("git_blob_sha1")
        for item in source.get("files", []) if isinstance(item, dict)
    }
    expected_files = {
        "mycelix-governance/zomes/execution/coordinator/src/lib.rs": COORDINATOR_BLOB,
        "mycelix-governance/zomes/execution/integrity/src/lib.rs": INTEGRITY_BLOB,
    }
    if files != expected_files:
        raise ValueError("source blob binding drift")

    creation = profile["timelock_creation"]
    expected_creation = {
        "entrypoint": "create_timelock",
        "proposal_id_source": "CallerSupplied",
        "actions_source": "CallerSupplied",
        "duration_hours_source": "CallerSupplied",
        "duration_hours_min": 1,
        "duration_hours_max": 8760,
        "proposal_lookup": "NoneObserved",
        "proposal_status_binding": "NoneObserved",
        "proposal_actions_binding": "NoneObserved",
        "policy_duration_binding": "NoneObserved",
        "initial_status": "Pending",
        "integrity_checks": ["ExpiresAfterStarted", "ActionsValidJson", "InitialStatusPending"],
    }
    if creation != expected_creation:
        raise ValueError("timelock creation observation drift")

    ready = profile["readiness_transition"]
    expected_ready = {
        "entrypoint": "mark_timelock_ready",
        "required_source_status": "Pending",
        "caller_rule": "TimelockCreatorOnly",
        "threshold_signature_verification": "NoneObserved",
        "target_status": "Ready",
    }
    if ready != expected_ready:
        raise ValueError("readiness observation drift")

    execution = profile["execution"]
    expected_execution = {
        "entrypoint": "execute_timelock",
        "expiry_required": True,
        "executor_identity_rule": "ExecutorDidMustMatchCaller",
        "ready_signature_policy": "TrustReadyStateNoSignatureLookup",
        "pending_signature_policy": "LookupThresholdSignatureIfAvailableElseWarnAndContinue",
        "unavailable_signing_authority": "WarningOnlyExecutionContinues",
        "action_source": "TimelockStoredActions",
    }
    if execution != expected_execution:
        raise ValueError("execution observation drift")

    dispatch = profile["action_dispatch"]
    if dispatch != {
        "TransferCredits": {
            "target": "governance_bridge::transfer_credits",
            "fail_closed_on_dispatch_error": True,
        },
        "UpdateParameter": {
            "target": "constitution::update_parameter",
            "fail_closed_on_dispatch_error": True,
        },
        "EmitEvent": {
            "target": "emit_signal",
            "fail_closed_on_dispatch_error": False,
        },
    }:
        raise ValueError("action-dispatch observation drift")

    gaps = profile["known_gaps"]
    if len(gaps) != 1 or gaps[0].get("issue") != 904 or gaps[0].get("status") != "Observed":
        raise ValueError("execution gap set drift")

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
        "known_gap_issue": 904,
    }


def load(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        obj = json.load(f)
    if not isinstance(obj, dict):
        raise ValueError("profile root must be object")
    return obj


def self_test(profile: dict) -> dict:
    out = validate(profile)
    baseline = payload_digest(profile)

    def identity_changes(mutator) -> None:
        candidate = copy.deepcopy(profile)
        candidate.pop("profile_content_sha256", None)
        mutator(candidate)
        if hashlib.sha256(canonical(candidate)).hexdigest() == baseline:
            raise AssertionError("semantic mutation did not change identity")

    identity_changes(lambda p: p["source_binding"]["files"][0].update(git_blob_sha1="0" * 40))
    identity_changes(lambda p: p["timelock_creation"].update(proposal_lookup="AuthoritativeProposalLookup"))
    identity_changes(lambda p: p["timelock_creation"].update(actions_source="AuthoritativeProposalActions"))
    identity_changes(lambda p: p["readiness_transition"].update(threshold_signature_verification="Required"))
    identity_changes(lambda p: p["execution"].update(unavailable_signing_authority="FailClosed"))

    for mutator in [
        lambda p: p.update(safe=True),
        lambda p: p["known_gaps"].clear(),
    ]:
        forged = copy.deepcopy(profile)
        mutator(forged)
        try:
            validate(forged)
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
    out = self_test(profile) if args.self_test else validate(profile)
    print(json.dumps(out, sort_keys=True, separators=(",", ":"), allow_nan=False))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Validate the MYC-CONST-003D1B source-bound effect capability profile.

This validator is intentionally independent of JSON Schema tooling. The schema
is a review artifact; this program enforces the narrow source-bound claims and
contains mutation-style self-tests for the capability ceiling.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

EXPECTED_SUBJECT = "15b9c89adf0ac3c6c5a73681614d6bfcd368820a"
EXPECTED_FILES = {
    "execution_coordinator": (
        "mycelix-governance/zomes/execution/coordinator/src/lib.rs",
        "3dbb8a8f69b377e494ccf24164c94bd80f54e0ef",
    ),
    "execution_integrity": (
        "mycelix-governance/zomes/execution/integrity/src/lib.rs",
        "657edaee9a314f100a0c4b1609a4596cf243e61d",
    ),
}
EXPECTED_ACTIONS = {
    "TransferCredits": {
        "effect_boundary": "CrossZomeCall",
        "target": "governance_bridge::transfer_credits",
        "outcome_observability": "Unknown",
    },
    "UpdateParameter": {
        "effect_boundary": "CrossZomeCall",
        "target": "constitution::update_parameter",
        "outcome_observability": "Unknown",
    },
    "EmitEvent": {
        "effect_boundary": "LocalSignal",
        "target": "emit_signal",
        "outcome_observability": "Opaque",
    },
}
EXPECTED_FINDINGS = {
    "effects_dispatched_before_execution_record",
    "execution_identity_created_after_dispatch",
    "sequential_dispatch_can_partially_apply",
    "partial_success_variant_defined_but_not_selected",
    "execution_result_integrity_requires_json",
    "coordinator_result_is_human_text",
    "timelock_failure_collapses_partial_execution",
    "emit_event_delivery_error_ignored",
}
EXPECTED_REQUIRED_CONTRACT = {
    "durable_operation_before_first_effect",
    "durable_ordered_action_intents_before_first_effect",
    "stable_operation_identity_before_first_effect",
    "stable_action_identity_before_attempt",
    "unknown_outcome_blocks_later_actions",
    "non_replay_safe_retry_requires_reconciled_no_effect",
    "successful_prefix_must_not_be_plain_failed",
    "machine_readable_observations",
    "compensation_is_new_forward_effect",
    "physical_batch_atomicity_requires_explicit_provider_evidence",
}
EXPECTED_NON_CLAIMS = {
    "not_runtime_repaired",
    "not_exactly_once_physical_delivery",
    "not_provider_idempotency_established",
    "not_provider_reconciliation_established",
    "not_compensation_established",
    "not_physical_batch_atomicity_established",
    "not_qualified",
}
TOP_KEYS = {
    "schema",
    "profile_id",
    "profile_revision",
    "authority_class",
    "source_binding",
    "batch_observation",
    "actions",
    "runtime_findings",
    "required_runtime_contract",
    "pending_dependencies",
    "non_claims",
}
ACTION_KEYS = {
    "action_type",
    "effect_boundary",
    "target",
    "replay_safety",
    "outcome_observability",
    "compensation",
    "provider_batch_atomicity",
    "safe_retry_without_reconciliation",
    "eligible_for_claimed_atomic_batch",
}


class ValidationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def validate(profile: dict[str, Any]) -> dict[str, Any]:
    require(set(profile) == TOP_KEYS, "top-level key set is not closed")
    require(
        profile["schema"] == "mycelix.constitutional-effect-capability-profile.v1",
        "wrong schema",
    )
    require(
        profile["profile_id"] == "mycelix-execution-effect-capabilities-15b9c89a-v1",
        "wrong profile id",
    )
    require(profile["profile_revision"] == 1, "wrong profile revision")
    require(profile["authority_class"] == "ObservedSourceBound", "authority widened")

    source = profile["source_binding"]
    require(
        set(source) == {"repository", "runtime_subject_sha", "d1a_semantic_head", "files"},
        "source binding key set is not closed",
    )
    require(source["repository"] == "Luminous-Dynamics/mycelix", "wrong repository")
    require(source["runtime_subject_sha"] == EXPECTED_SUBJECT, "runtime subject drift")
    require(source["d1a_semantic_head"] == EXPECTED_SUBJECT, "D1A parent drift")
    files = source["files"]
    require(isinstance(files, list) and len(files) == 2, "expected two bound files")
    by_role = {item.get("role"): item for item in files}
    require(set(by_role) == set(EXPECTED_FILES), "bound file roles changed")
    for role, (path, blob) in EXPECTED_FILES.items():
        item = by_role[role]
        require(
            set(item) == {"path", "git_blob_sha1", "role"},
            f"{role}: file key set is not closed",
        )
        require(item["path"] == path, f"{role}: path drift")
        require(item["git_blob_sha1"] == blob, f"{role}: blob drift")

    batch = profile["batch_observation"]
    expected_batch = {
        "dispatch_order": "Sequential",
        "durable_execution_record_timing": "AfterDispatch",
        "execution_identity_timing": "AfterDispatch",
        "physical_batch_atomicity": "NoneObserved",
        "partial_completion_representation": "CollapsedToFailed",
        "structured_result_compatibility": "MismatchObserved",
    }
    require(batch == expected_batch, "runtime batch observation was widened or rewritten")

    actions = profile["actions"]
    require(isinstance(actions, list) and len(actions) == 3, "expected three action profiles")
    action_map = {item.get("action_type"): item for item in actions}
    require(set(action_map) == set(EXPECTED_ACTIONS), "action census changed")
    for action_type, expected in EXPECTED_ACTIONS.items():
        item = action_map[action_type]
        require(set(item) == ACTION_KEYS, f"{action_type}: action key set is not closed")
        require(item["effect_boundary"] == expected["effect_boundary"], f"{action_type}: boundary drift")
        require(item["target"] == expected["target"], f"{action_type}: target drift")
        require(
            item["outcome_observability"] == expected["outcome_observability"],
            f"{action_type}: observability claim widened",
        )
        require(item["replay_safety"] == "Unknown", f"{action_type}: idempotency not established")
        require(item["compensation"] == "Unknown", f"{action_type}: compensation not established")
        require(
            item["provider_batch_atomicity"] == "NoneObserved",
            f"{action_type}: provider atomicity not established",
        )
        require(
            item["safe_retry_without_reconciliation"] is False,
            f"{action_type}: blind retry must remain denied",
        )
        require(
            item["eligible_for_claimed_atomic_batch"] is False,
            f"{action_type}: physical batch atomicity must remain unclaimed",
        )

    findings = profile["runtime_findings"]
    require(
        isinstance(findings, list) and len(findings) == len(set(findings)),
        "runtime findings must be unique",
    )
    require(set(findings) == EXPECTED_FINDINGS, "runtime finding census changed")

    contract = profile["required_runtime_contract"]
    require(set(contract) == EXPECTED_REQUIRED_CONTRACT, "required contract key set changed")
    require(all(value is True for value in contract.values()), "required contract weakened")

    dependencies = profile["pending_dependencies"]
    require(
        set(dependencies)
        == {
            "MYC-CONST-003B4 qualified concrete ClaimBinding",
            "MYC-CONST-003CR1 qualified runtime refinement crosswalk",
            "provider-specific idempotency/reconciliation/compensation evidence",
        },
        "pending dependency boundary changed",
    )

    non_claims = profile["non_claims"]
    require(set(non_claims) == EXPECTED_NON_CLAIMS, "non-claim boundary changed")

    return {
        "validated": True,
        "profile_id": profile["profile_id"],
        "profile_revision": profile["profile_revision"],
        "authority_class": profile["authority_class"],
        "runtime_subject_sha": source["runtime_subject_sha"],
        "action_count": len(actions),
        "runtime_finding_count": len(findings),
        "profile_content_sha256": canonical_sha256(profile),
    }


def expect_rejected(profile: dict[str, Any], mutate) -> None:
    candidate = copy.deepcopy(profile)
    mutate(candidate)
    try:
        validate(candidate)
    except ValidationError:
        return
    raise ValidationError("self-test mutation unexpectedly validated")


def self_test(profile: dict[str, Any]) -> None:
    expect_rejected(
        profile,
        lambda p: p["actions"][0].__setitem__("replay_safety", "StableKeyIdempotent"),
    )
    expect_rejected(
        profile,
        lambda p: p["actions"][1].__setitem__("outcome_observability", "AuthoritativeQuery"),
    )
    expect_rejected(
        profile,
        lambda p: p["actions"][2].__setitem__("safe_retry_without_reconciliation", True),
    )
    expect_rejected(
        profile,
        lambda p: p["batch_observation"].__setitem__("physical_batch_atomicity", "ProviderAtomic"),
    )
    expect_rejected(
        profile,
        lambda p: p["runtime_findings"].remove("effects_dispatched_before_execution_record"),
    )
    expect_rejected(
        profile,
        lambda p: p["source_binding"]["files"][0].__setitem__("git_blob_sha1", "0" * 40),
    )
    expect_rejected(
        profile,
        lambda p: p["required_runtime_contract"].__setitem__("unknown_outcome_blocks_later_actions", False),
    )
    expect_rejected(
        profile,
        lambda p: p["non_claims"].remove("not_qualified"),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    try:
        profile = json.loads(args.profile.read_text(encoding="utf-8"))
        result = validate(profile)
        if args.self_test:
            self_test(profile)
            result["self_test"] = True
        print(json.dumps(result, sort_keys=True, separators=(",", ":")))
        return 0
    except (OSError, json.JSONDecodeError, ValidationError) as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

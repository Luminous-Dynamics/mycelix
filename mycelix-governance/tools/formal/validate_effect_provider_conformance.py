#!/usr/bin/env python3
"""Validate MYC-CONST-003D1D0 provider-conformance evidence.

This is deliberately a source-bound observation validator, not a provider qualifier.
It rejects capability inflation and verifies the exact runtime snippets underlying
D1D0's negative/compatibility findings.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
DEFAULT_PROFILE = (
    REPO
    / "mycelix-governance"
    / "specs"
    / "constitutional-effect-provider-conformance.v1.json"
)
EXPECTED_RUNTIME_SUBJECT = "15b9c89adf0ac3c6c5a73681614d6bfcd368820a"
EXPECTED_D1C_PARENT = "47d1d764323dbfaf991b5574cfde83abb7a3e4a4"
EXPECTED_ACTIONS = {"TransferCredits", "UpdateParameter", "EmitEvent"}

EXPECTED_BLOBS = {
    "mycelix-governance/zomes/execution/coordinator/src/lib.rs": "3dbb8a8f69b377e494ccf24164c94bd80f54e0ef",
    "mycelix-governance/zomes/bridge/coordinator/src/cross_cluster.rs": "3eb0ade8633d6e711fd266bca6eb2ebab616eac2",
    "mycelix-governance/zomes/bridge/coordinator/src/query.rs": "5a02812880a2fdaa8f3ed767a4686afc00c566f9",
    "mycelix-governance/zomes/bridge/integrity/src/lib.rs": "61b20610216e2fd69c701ecaea5322c448eda119",
    "mycelix-governance/zomes/bridge/coordinator/src/validation.rs": "9e13ba58939880738eccb997e54f729da7a11304",
    "mycelix-finance/zomes/treasury/coordinator/src/lib.rs": "840e66bcb6fdedb27fe2451752511d1317eb50b8",
    "mycelix-governance/zomes/constitution/coordinator/src/lib.rs": "923a1ce789c8319c79df7f33a9241af50804ec55",
    "mycelix-governance/zomes/constitution/integrity/src/lib.rs": "f83a457a8ff40b0003c07dba9da598a478c5e6f6",
}

REQUIRED_NON_CLAIMS = {
    "not_provider_repaired",
    "not_provider_qualified",
    "not_deployment_currentness_qualified",
    "not_finance_idempotency_established",
    "not_parameter_authority_established",
    "not_signal_delivery_receipt_established",
    "not_physical_exactly_once_established",
}

REQUIRED_ADAPTER_TRUE = {
    "request_schema_compatible_with_provider",
    "authorization_must_hold_at_integrity_boundary",
    "stable_action_identity_before_effect",
    "durable_intent_before_effect",
    "known_no_effect_requires_authoritative_provider_evidence",
    "timeout_or_transport_error_is_unknown_outcome",
    "repeat_same_action_identity_cannot_repeat_physical_effect_unless_qualified_replay_safe",
    "machine_readable_receipt_or_query_evidence",
    "derived_projections_cannot_define_constitutional_completion",
}

REQUIRED_OBSERVATIONS = {
    "TransferCredits": {
        "execution_dispatch_uses_legacy_from_to_amount_shape",
        "legacy_transfer_validation_is_structural_only",
        "newer_governance_finance_bridge_calls_execute_governance_transfer",
        "execute_governance_transfer_not_observed_in_exact_treasury_coordinator",
        "treasury_execute_allocation_has_stable_allocation_id_and_query_path",
        "treasury_execute_allocation_debits_before_marking_allocation_executed",
        "treasury_execute_allocation_retry_after_post_debit_record_failure_can_repeat_debit",
        "dkg_allocation_debits_before_creating_executed_allocation_record",
    },
    "UpdateParameter": {
        "execution_dispatch_omits_proposal_id",
        "existing_parameter_change_requires_proposal_id_at_coordinator",
        "missing_parameter_can_enter_create_path_without_proposal_id",
        "parameter_value_must_be_valid_json_at_integrity_boundary",
        "execution_action_value_is_not_canonicalized_as_json",
        "set_parameter_appends_new_parameter_entry_and_index_link",
        "get_parameter_selects_latest_parameter_index_link_by_timestamp",
        "parameter_integrity_does_not_bind_changed_by_proposal_to_authority",
        "parameter_integrity_does_not_bind_parameter_writer_to_governance_authority",
        "parameter_index_link_creation_is_not_authority_bound",
        "phi_config_sync_is_best_effort_derived_projection",
    },
    "EmitEvent": {
        "execution_emit_event_discards_emit_signal_result",
        "signal_delivery_is_not_durable_operation_evidence",
        "governance_bridge_has_durable_broadcast_governance_event_record_path",
        "durable_broadcast_event_id_is_timestamp_generated",
        "governance_bridge_event_creation_only_requires_nonempty_source_happ",
        "governance_bridge_event_updates_are_integrity_valid",
        "recent_events_links_are_not_event_authority_bound",
        "client_signal_should_be_projection_of_durable_event_state",
    },
}


def fail(message: str) -> None:
    raise ValueError(message)


def git_blob(path: str) -> str:
    proc = subprocess.run(
        ["git", "hash-object", path],
        cwd=REPO,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        fail(f"git hash-object failed for {path}: {proc.stderr.strip()}")
    return proc.stdout.strip()


def read(path: str) -> str:
    return (REPO / path).read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        fail(f"required source observation missing: {label}")


def require_absent(text: str, needle: str, label: str) -> None:
    if needle in text:
        fail(f"source observation changed: {label}")


def require_order(text: str, first: str, second: str, label: str) -> None:
    a = text.find(first)
    b = text.find(second)
    if a < 0 or b < 0 or a >= b:
        fail(f"required source ordering missing: {label}")


def lane_map(profile: dict[str, Any]) -> dict[str, dict[str, Any]]:
    lanes = profile.get("provider_lanes")
    if not isinstance(lanes, list) or len(lanes) != 3:
        fail("provider_lanes must contain exactly three lanes")
    result: dict[str, dict[str, Any]] = {}
    for lane in lanes:
        if not isinstance(lane, dict):
            fail("provider lane must be an object")
        action = lane.get("action_type")
        if action in result:
            fail(f"duplicate provider lane {action}")
        result[action] = lane
    if set(result) != EXPECTED_ACTIONS:
        fail(f"provider action census mismatch: {sorted(result)}")
    return result


def validate_profile(profile: dict[str, Any], *, check_source: bool = True) -> dict[str, Any]:
    if profile.get("schema") != "mycelix.constitutional-effect-provider-conformance.v1":
        fail("schema mismatch")
    if profile.get("profile_revision") != 1:
        fail("profile_revision must remain 1")
    if profile.get("authority_class") != "ObservedSourceBound":
        fail("authority_class inflation")

    binding = profile.get("source_binding")
    if not isinstance(binding, dict):
        fail("source_binding missing")
    if binding.get("repository") != "Luminous-Dynamics/mycelix":
        fail("repository binding mismatch")
    if binding.get("runtime_subject_sha") != EXPECTED_RUNTIME_SUBJECT:
        fail("runtime subject binding mismatch")
    if binding.get("d1c_semantic_parent") != EXPECTED_D1C_PARENT:
        fail("D1C semantic parent mismatch")

    observed_files = {
        item.get("path"): item.get("git_blob_sha1")
        for item in binding.get("files", [])
        if isinstance(item, dict)
    }
    if observed_files != EXPECTED_BLOBS:
        fail("source file/blob census mismatch")

    if check_source:
        for path, expected in EXPECTED_BLOBS.items():
            actual = git_blob(path)
            if actual != expected:
                fail(f"source blob drift for {path}: expected {expected}, got {actual}")

    lanes = lane_map(profile)

    finance = lanes["TransferCredits"]
    if finance.get("conformance") != "Incompatible":
        fail("finance lane must remain Incompatible until successor evidence")
    if finance.get("current_dispatch_target") != "governance_bridge::transfer_credits":
        fail("finance dispatch target changed")
    for key in [
        "stable_effect_identity",
        "idempotency",
        "authoritative_outcome_query",
        "receipt_verification",
        "provider_atomicity",
        "authorization_binding",
    ]:
        if finance.get(key) != "NotEstablished":
            fail(f"finance capability inflated: {key}")

    parameter = lanes["UpdateParameter"]
    if parameter.get("conformance") != "Incompatible":
        fail("parameter lane must remain Incompatible until successor evidence")
    if parameter.get("current_dispatch_target") != "constitution::update_parameter":
        fail("parameter dispatch target changed")
    if parameter.get("authorization_binding") != "Incompatible":
        fail("parameter authorization mismatch must remain explicit")
    if parameter.get("authoritative_outcome_query") != "Partial":
        fail("parameter query observation must remain Partial")
    for key in ["stable_effect_identity", "idempotency", "receipt_verification", "provider_atomicity"]:
        if parameter.get(key) != "NotEstablished":
            fail(f"parameter capability inflated: {key}")

    event = lanes["EmitEvent"]
    if event.get("conformance") != "ProjectionOnly":
        fail("EmitEvent must remain ProjectionOnly in this observation profile")
    if event.get("current_dispatch_target") != "emit_signal":
        fail("EmitEvent dispatch target changed")
    for key in ["idempotency", "authoritative_outcome_query", "receipt_verification", "provider_atomicity"]:
        if event.get(key) != "NotApplicableToProjection":
            fail(f"projection capability category changed: {key}")

    for action, required in REQUIRED_OBSERVATIONS.items():
        observations = lanes[action].get("observations")
        if not isinstance(observations, list) or set(observations) != required:
            fail(f"{action} observation census mismatch")
        repairs = lanes[action].get("repair_prerequisites")
        if not isinstance(repairs, list) or not repairs or not all(isinstance(x, str) and x for x in repairs):
            fail(f"{action} repair prerequisites missing")

    adapter = profile.get("required_adapter_contract")
    if not isinstance(adapter, dict) or set(adapter) != REQUIRED_ADAPTER_TRUE:
        fail("required_adapter_contract census mismatch")
    if any(adapter[key] is not True for key in REQUIRED_ADAPTER_TRUE):
        fail("required adapter contract weakened")

    non_claims = profile.get("non_claims")
    if not isinstance(non_claims, list) or set(non_claims) != REQUIRED_NON_CLAIMS:
        fail("non_claim boundary mismatch")

    if check_source:
        validate_source_observations()

    canonical = json.dumps(profile, sort_keys=True, separators=(",", ":")).encode()
    return {
        "validated": True,
        "authority_class": profile["authority_class"],
        "profile_id": profile["profile_id"],
        "profile_revision": profile["profile_revision"],
        "runtime_subject_sha": binding["runtime_subject_sha"],
        "profile_content_sha256": hashlib.sha256(canonical).hexdigest(),
        "provider_conformance": {
            action: lanes[action]["conformance"] for action in sorted(lanes)
        },
    }


def validate_source_observations() -> None:
    execution = read("mycelix-governance/zomes/execution/coordinator/src/lib.rs")
    bridge_cross = read("mycelix-governance/zomes/bridge/coordinator/src/cross_cluster.rs")
    bridge_query = read("mycelix-governance/zomes/bridge/coordinator/src/query.rs")
    bridge_integrity = read("mycelix-governance/zomes/bridge/integrity/src/lib.rs")
    bridge_validation = read("mycelix-governance/zomes/bridge/coordinator/src/validation.rs")
    treasury = read("mycelix-finance/zomes/treasury/coordinator/src/lib.rs")
    constitution = read("mycelix-governance/zomes/constitution/coordinator/src/lib.rs")
    constitution_integrity = read("mycelix-governance/zomes/constitution/integrity/src/lib.rs")

    # Finance dispatch and legacy shape.
    require(execution, '"governance_bridge",\n                    "transfer_credits"', "legacy finance dispatch target")
    require(execution, 'serde_json::json!({"from": from, "to": to, "amount": amount})', "legacy transfer request shape")
    require(bridge_validation, "pub fn check_transfer_credits_input", "legacy transfer structural validator")
    require(bridge_cross, '"treasury", "execute_governance_transfer"', "newer finance bridge target")
    require_absent(treasury, "pub fn execute_governance_transfer", "treasury endpoint remains unobserved")

    # Existing allocation path has stable lookup, but wrong crash ordering.
    require(treasury, "pub fn execute_allocation(allocation_id: String)", "treasury allocation executor")
    require(treasury, "fn get_allocation_record(allocation_id: &str)", "allocation lookup by stable id")
    execute_start = treasury.index("pub fn execute_allocation(allocation_id: String)")
    execute_end = treasury.index("/// Internal helper: fetch an allocation", execute_start)
    execute_body = treasury[execute_start:execute_end]
    require_order(
        execute_body,
        "debit_treasury(&alloc.treasury_id, alloc.amount)?;",
        "EntryTypes::Allocation(executed)",
        "treasury debit occurs before executed record update",
    )

    dkg_start = treasury.index("pub fn execute_dkg_allocation")
    dkg_body = treasury[dkg_start:]
    require_order(
        dkg_body,
        "debit_treasury(&input.treasury_id, input.amount)?;",
        "let allocation = Allocation",
        "DKG debit occurs before executed allocation record creation",
    )

    # Parameter request mismatch and integrity ceiling.
    require(execution, 'serde_json::json!({"parameter": parameter, "value": value})', "parameter dispatch omits proposal id")
    require(constitution, "pub proposal_id: Option<String>", "provider accepts optional proposal id")
    require(constitution, "Parameter already exists. Use a governance proposal to change it.", "existing parameter governance gate")
    require(constitution_integrity, "Parameter value must be valid JSON", "parameter JSON integrity rule")
    require(constitution_integrity, "fn validate_create_parameter(\n    _action: Create", "parameter create integrity does not use author")
    require(constitution_integrity, "fn validate_update_parameter(\n    _action: Update", "parameter update integrity does not use author")
    require(constitution, "call_local_best_effort", "phi config best-effort projection")

    # Event projection versus currently under-bound durable event source.
    require(execution, "let _ = emit_signal", "execution signal result ignored")
    require(bridge_query, "pub fn broadcast_governance_event", "durable governance event endpoint")
    broadcast_start = bridge_query.index("pub fn broadcast_governance_event")
    broadcast_end = bridge_query.index("pub struct BroadcastGovernanceEventInput", broadcast_start)
    broadcast_body = bridge_query[broadcast_start:broadcast_end]
    require_order(
        broadcast_body,
        "create_entry(&EntryTypes::GovernanceBridgeEvent(event))?",
        "create_link(",
        "durable event entry before index link",
    )
    require(bridge_query, 'format!("event:{:?}:{}", input.event_type, now.as_micros())', "timestamp-generated durable event id")
    require(
        bridge_integrity,
        "fn validate_create_event(\n    _action: Create",
        "governance event creation does not use author",
    )
    require(
        bridge_integrity,
        'if event.source_happ.is_empty()',
        "governance event creation only requires source_happ presence",
    )
    require(
        bridge_integrity,
        "EntryTypes::GovernanceBridgeEvent(_) => Ok(ValidateCallbackResult::Valid)",
        "governance event updates remain valid",
    )
    require(
        bridge_integrity,
        "LinkTypes::RecentEvents => Ok(ValidateCallbackResult::Valid)",
        "recent event links lack event-specific authority binding",
    )


def expect_rejected(profile: dict[str, Any], mutator, label: str) -> None:
    candidate = copy.deepcopy(profile)
    mutator(candidate)
    try:
        validate_profile(candidate, check_source=False)
    except ValueError:
        return
    fail(f"self-test mutation survived: {label}")


def self_test(profile: dict[str, Any]) -> list[str]:
    def lane(candidate: dict[str, Any], action: str) -> dict[str, Any]:
        return next(x for x in candidate["provider_lanes"] if x["action_type"] == action)

    cases = [
        (
            "inflate-finance-idempotency",
            lambda p: lane(p, "TransferCredits").__setitem__("idempotency", "Established"),
        ),
        (
            "inflate-finance-conformance",
            lambda p: lane(p, "TransferCredits").__setitem__("conformance", "Compatible"),
        ),
        (
            "hide-parameter-authority-mismatch",
            lambda p: lane(p, "UpdateParameter").__setitem__("authorization_binding", "Established"),
        ),
        (
            "promote-signal-to-effect",
            lambda p: lane(p, "EmitEvent").__setitem__("conformance", "Compatible"),
        ),
        (
            "remove-runtime-finding",
            lambda p: lane(p, "UpdateParameter")["observations"].pop(),
        ),
        (
            "weaken-adapter-contract",
            lambda p: p["required_adapter_contract"].__setitem__("durable_intent_before_effect", False),
        ),
        (
            "drop-nonclaim",
            lambda p: p["non_claims"].pop(),
        ),
        (
            "source-blob-drift",
            lambda p: p["source_binding"]["files"][0].__setitem__("git_blob_sha1", "0" * 40),
        ),
    ]

    passed: list[str] = []
    for label, mutator in cases:
        expect_rejected(profile, mutator, label)
        passed.append(label)
    return passed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    result = validate_profile(profile)
    if args.self_test:
        result["self_test"] = True
        result["mutation_controls"] = self_test(profile)
    else:
        result["self_test"] = False
    print(json.dumps(result, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

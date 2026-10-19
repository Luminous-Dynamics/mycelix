#!/usr/bin/env python3
"""In-memory correction/appeal state machine with privacy-minimized audit chaining.

This synthetic contract harness is not production case-management software.
It does not authenticate actors, store cases durably, send notices, verify
external records, or provide legal advice. Event hashes are tamper-evident
metadata checks, not signatures or proof of trusted timestamps.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import re
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

CONTRACT_PATH = Path(__file__).with_name("APPEAL_CASE_CONTRACT_V1.json")
GENESIS_HASH = "0" * 64
IMMUTABLE_CASE_FIELDS = ("case_ref", "case_owner_ref", "service_code", "issue_code", "jurisdiction_code")
HANDLE_RE = re.compile(r"^[A-Z0-9][A-Z0-9._-]{2,95}$")
CODE_RE = re.compile(r"^[A-Z][A-Z0-9_]{2,63}$")
HASH_RE = re.compile(r"^[0-9a-f]{64}$")
EVENT_FIELDS = {
    "sequence", "timestamp_utc", "actor_ref", "actor_role", "action",
    "from_state", "to_state", "reason_code", "target_actor_ref",
    "previous_hash", "event_hash", "policy_ref", "policy_digest",
}
CASE_DYNAMIC_FIELDS = {
    "current_state", "assigned_steward_ref", "initial_decision_actor_ref",
    "appeal_reviewer_ref", "retention_policy_status", "retention_policy",
    "retention_review_complete", "closed_at_utc",
}
RETENTION_FIELDS = {
    "status", "jurisdiction_code", "record_class", "lawful_basis_ref",
    "responsible_party_ref", "max_retention_days", "deletion_procedure_ref",
    "backup_deletion_policy_ref", "review_evidence_ref",
}
PROHIBITED_EVENT_KEYS = {
    "raw_credential", "credential_bytes", "national_identifier_value", "national_id_number",
    "biometric_template", "biometric_data", "replayable_presentation", "claim_values",
    "person_name", "date_of_birth_value", "residential_address", "free_text_notes",
    "full_credential_or_verifier_response", "case_notes", "message_body", "email_address",
}


def _canonical(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _hash(value: Any) -> str:
    return hashlib.sha256(_canonical(value)).hexdigest()


def _genesis_hash(case: dict[str, Any], contract: dict[str, Any]) -> str:
    # Bind the chain to the case envelope so a valid event list cannot be
    # transplanted to a different case/owner/service.
    return _hash({
        "contract_id": contract["contract_id"],
        **{key: case.get(key) for key in IMMUTABLE_CASE_FIELDS},
    })


def _is_handle(value: Any) -> bool:
    return isinstance(value, str) and bool(HANDLE_RE.fullmatch(value))


def _is_code(value: Any) -> bool:
    return isinstance(value, str) and bool(CODE_RE.fullmatch(value))


def _contains_prohibited_key(value: Any) -> bool:
    if isinstance(value, dict):
        for key, child in value.items():
            if not isinstance(key, str) or key.strip().lower() in PROHIBITED_EVENT_KEYS:
                return True
            if _contains_prohibited_key(child):
                return True
    elif isinstance(value, list):
        return any(_contains_prohibited_key(item) for item in value)
    return False


def _parse_utc(value: Any) -> datetime:
    if not isinstance(value, str) or not re.fullmatch(r"\d{4}-\d\d-\d\dT\d\d:\d\d:\d\dZ", value):
        raise ValueError("timestamp_utc must use YYYY-MM-DDTHH:MM:SSZ")
    try:
        parsed = datetime.strptime(value, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
    except ValueError as exc:
        raise ValueError("timestamp_utc is not a valid UTC timestamp") from exc
    return parsed


def _retention_template(contract: dict[str, Any]) -> dict[str, Any]:
    return copy.deepcopy(contract["retention_policy_template"])


def validate_retention_policy(
    policy: Any,
    contract: dict[str, Any] | None = None,
) -> list[str]:
    """Validate a jurisdiction/record-class-specific policy, never authorize persistence."""
    active_contract = contract or load_contract()
    if not isinstance(policy, dict):
        return ["retention policy must be an object"]
    errors: list[str] = []
    if set(policy) != RETENTION_FIELDS:
        errors.append("retention policy fields must match the closed v1 schema")
    if policy.get("status") != "approved":
        errors.append("retention policy status must be approved")
    if not _is_code(policy.get("jurisdiction_code")) or policy.get("jurisdiction_code") == "JURISDICTION_UNASSIGNED":
        errors.append("retention policy requires an explicitly assigned jurisdiction code")
    classes = active_contract.get("record_class_values", [])
    if not isinstance(policy.get("record_class"), str) or policy.get("record_class") not in classes:
        errors.append("retention policy record_class must use the closed v1 enum")
    for key in (
        "lawful_basis_ref", "responsible_party_ref", "deletion_procedure_ref",
        "backup_deletion_policy_ref", "review_evidence_ref",
    ):
        if not _is_handle(policy.get(key)):
            errors.append(f"retention policy {key} must be a non-empty opaque reference")
    days = policy.get("max_retention_days")
    if type(days) is not int or days < 1 or days > 36500:
        errors.append("max_retention_days must be an integer from 1 to 36500")
    return errors


def new_case(
    case_ref: str,
    case_owner_ref: str,
    service_code: str,
    issue_code: str,
    contract: dict[str, Any] | None = None,
    *,
    jurisdiction_code: str = "JURISDICTION_UNASSIGNED",
) -> dict[str, Any]:
    """Create an in-memory case envelope containing only opaque refs and stable codes."""
    active_contract = contract or load_contract()
    if not _is_handle(case_ref) or not case_ref.startswith("CASE-"):
        raise ValueError("case_ref must be a case-scoped opaque CASE-* handle")
    if not _is_handle(case_owner_ref):
        raise ValueError("case_owner_ref must be an opaque local actor handle")
    if not _is_code(service_code) or not _is_code(issue_code):
        raise ValueError("service_code and issue_code must be stable uppercase codes")
    if issue_code not in active_contract.get("issue_code_values", []):
        raise ValueError("issue_code must use the closed v1 issue enum")
    if not _is_code(jurisdiction_code):
        raise ValueError("jurisdiction_code must be a stable uppercase code")
    retention = _retention_template(active_contract)
    return {
        "case_ref": case_ref,
        "case_owner_ref": case_owner_ref,
        "service_code": service_code,
        "issue_code": issue_code,
        "jurisdiction_code": jurisdiction_code,
        "current_state": "not_opened",
        "assigned_steward_ref": None,
        "initial_decision_actor_ref": None,
        "appeal_reviewer_ref": None,
        "retention_policy_status": retention["status"],
        "retention_policy": retention,
        "retention_review_complete": False,
        "closed_at_utc": None,
        # This demo has no persistence adapter. A reviewed policy is not a
        # runtime command to persist anything.
        "durable_case_storage_enabled": False,
        "public_dht_publication_enabled": False,
    }


def _transitions(contract: dict[str, Any], action: str, state: str) -> list[dict[str, Any]]:
    matches = []
    for transition in contract["transitions"]:
        state_match = (
            transition["from_state"] == state
            or transition["from_state"] == "ANY_AFTER_OPEN" and state not in {"not_opened", "expired"}
        )
        if transition["action"] == action and state_match:
            matches.append(transition)
    return matches


def authorize_action(
    case: dict[str, Any],
    actor_ref: str,
    actor_role: str,
    action: str,
) -> bool:
    """Evaluate contract RBAC against opaque, externally authenticated actor context.

    The caller must authenticate actor_ref and determine the actor's role. This
    function only enforces the synthetic role/scope contract; it is not an
    authentication or authorization server.
    """
    active_contract = load_contract()
    if not _is_handle(actor_ref) or actor_role not in active_contract["role_values"]:
        return False
    declared_rules = [rule for rule in active_contract["access_policy"] if rule.get("action") == action]
    if not declared_rules or actor_role not in {
        role for rule in declared_rules for role in rule.get("roles", [])
    }:
        return False
    if action == "view_case":
        if actor_role == "requester":
            return actor_ref == case["case_owner_ref"]
        if actor_role == "case_steward":
            return actor_ref == case.get("assigned_steward_ref")
        if actor_role == "independent_reviewer":
            return actor_ref == case.get("appeal_reviewer_ref")
        return False
    if action in {"view_audit_metadata"}:
        return actor_role in {"auditor", "privacy_officer"}
    if action in {"review_retention_policy", "expire_case"}:
        return actor_role == "privacy_officer"
    if action == "open_case" or action in {"submit_correction", "request_appeal"}:
        return actor_role == "requester" and actor_ref == case["case_owner_ref"]
    if action == "acknowledge_case":
        return actor_role == "case_steward" and actor_ref != case["case_owner_ref"]
    if action in {"assign_steward", "resume_review", "issue_initial_decision", "close_case"}:
        return actor_role == "case_steward" and actor_ref == case.get("assigned_steward_ref")
    if action == "start_appeal_review":
        return (
            actor_role == "independent_reviewer"
            and actor_ref != case.get("initial_decision_actor_ref")
        )
    if action == "issue_appeal_decision":
        return (
            actor_role == "independent_reviewer"
            and actor_ref == case.get("appeal_reviewer_ref")
            and actor_ref != case.get("initial_decision_actor_ref")
        )
    if action == "record_retention_policy":
        return actor_role == "privacy_officer"
    return False


def _state_snapshot(case: dict[str, Any]) -> dict[str, Any]:
    return {key: copy.deepcopy(case.get(key)) for key in CASE_DYNAMIC_FIELDS}


def _set_state(case: dict[str, Any], state: dict[str, Any]) -> dict[str, Any]:
    updated = copy.deepcopy(case)
    for key, value in state.items():
        updated[key] = copy.deepcopy(value)
    return updated


def _initial_state(contract: dict[str, Any]) -> dict[str, Any]:
    retention = _retention_template(contract)
    return {
        "current_state": "not_opened",
        "assigned_steward_ref": None,
        "initial_decision_actor_ref": None,
        "appeal_reviewer_ref": None,
        "retention_policy_status": retention["status"],
        "retention_policy": retention,
        "retention_review_complete": False,
        "closed_at_utc": None,
    }


def _apply_event_state(
    base_case: dict[str, Any],
    state: dict[str, Any],
    event: dict[str, Any],
    contract: dict[str, Any],
    policy_document: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Validate one candidate event against the state reached by prior events."""
    action = event["action"]
    actor_ref = event["actor_ref"]
    actor_role = event["actor_role"]
    from_state = state["current_state"]
    if event["from_state"] != from_state:
        raise ValueError(f"event from_state mismatch: expected {from_state}")
    matched = [
        t for t in _transitions(contract, action, from_state)
        if t["actor_role"] == actor_role
        and (t["to_state"] == event["to_state"] or t["to_state"] == "SAME" and event["to_state"] == from_state)
        and t["reason_code"] == event["reason_code"]
    ]
    if len(matched) != 1:
        raise ValueError(f"action/role/reason/state transition is not authorized: {action}")
    transition = matched[0]
    if _contains_prohibited_key(event):
        raise ValueError("event contains a prohibited personal/credential-material field")
    if not _is_handle(actor_ref):
        raise ValueError("actor_ref must be an opaque local handle")
    target = event["target_actor_ref"]
    if target is not None and not _is_handle(target):
        raise ValueError("target_actor_ref must be null or an opaque local handle")
    if action not in {"acknowledge_case", "assign_steward"} and target is not None:
        raise ValueError("target_actor_ref is only permitted for steward assignment events")
    if actor_role == "requester" and actor_ref != base_case["case_owner_ref"]:
        raise ValueError("requester must match the case owner")
    if transition.get("requires_owner_match") and actor_ref != base_case["case_owner_ref"]:
        raise ValueError("only the case owner may submit this request")
    if transition.get("requires_current_steward") and actor_ref != state["assigned_steward_ref"]:
        raise ValueError("action requires the currently assigned case steward")
    if transition.get("requires_target_actor") and target is None:
        raise ValueError("this transition requires an explicit opaque target actor reference")
    if action == "acknowledge_case":
        if target == base_case["case_owner_ref"]:
            raise ValueError("case owner cannot be the assigned case steward")
        if state["assigned_steward_ref"] is not None:
            raise ValueError("case already has an assigned steward")
    if action == "assign_steward":
        if target == base_case["case_owner_ref"]:
            raise ValueError("case owner cannot be the assigned case steward")
        if target == state["assigned_steward_ref"]:
            raise ValueError("new steward must differ from current steward")
    if transition.get("must_differ_from_initial_decision_actor") and actor_ref == state["initial_decision_actor_ref"]:
        raise ValueError("independent appeal reviewer must differ from the initial decision-maker")
    if transition.get("requires_appeal_reviewer_match") and actor_ref != state["appeal_reviewer_ref"]:
        raise ValueError("appeal decision must be issued by the assigned independent reviewer")
    if transition.get("requires_approved_retention") and action != "record_retention_policy":
        policy = state["retention_policy"]
        if state["retention_policy_status"] != "approved" or validate_retention_policy(policy):
            raise ValueError("case expiry requires an approved, complete retention policy")
        if not state["closed_at_utc"]:
            raise ValueError("case expiry requires a recorded close timestamp")
        closed_at = _parse_utc(state["closed_at_utc"])
        event_time = _parse_utc(event["timestamp_utc"])
        elapsed = event_time - closed_at
        if elapsed < timedelta(days=policy["max_retention_days"]):
            raise ValueError("retention window has not elapsed")
    if action == "record_retention_policy":
        if state["retention_policy_status"] == "approved":
            raise ValueError("retention policy is immutable after approval in v1")
        if policy_document is None:
            raise ValueError("retention policy review requires the policy document")
        policy_errors = validate_retention_policy(policy_document, contract)
        if policy_errors:
            raise ValueError("invalid approved retention policy: " + "; ".join(policy_errors))
        if policy_document.get("jurisdiction_code") != base_case.get("jurisdiction_code"):
            raise ValueError("retention policy jurisdiction must match the case jurisdiction")
        expected_class = contract.get("issue_record_classes", {}).get(base_case.get("issue_code"))
        if expected_class is None or policy_document.get("record_class") != expected_class:
            raise ValueError("retention policy record class must match the case issue-code mapping")
        if not _is_handle(event["policy_ref"]):
            raise ValueError("policy_ref must be an opaque approved-policy reference")
        if event["policy_digest"] != _hash(policy_document):
            raise ValueError("policy digest does not match supplied policy document")
    elif event["policy_ref"] is not None or event["policy_digest"] is not None:
        raise ValueError("policy references are only permitted on record_retention_policy")

    updated = copy.deepcopy(state)
    updated["current_state"] = from_state if transition["to_state"] == "SAME" else event["to_state"]
    if action == "acknowledge_case":
        updated["assigned_steward_ref"] = target
    elif action == "assign_steward":
        updated["assigned_steward_ref"] = target
    elif action == "issue_initial_decision":
        updated["initial_decision_actor_ref"] = actor_ref
    elif action == "start_appeal_review":
        updated["appeal_reviewer_ref"] = actor_ref
    elif action == "record_retention_policy":
        updated["retention_policy"] = copy.deepcopy(policy_document)
        updated["retention_policy_status"] = "approved"
        updated["retention_review_complete"] = True
    elif action == "close_case":
        updated["closed_at_utc"] = event["timestamp_utc"]
    elif action == "expire_case":
        # Expiry closes the operational case; record deletion/backup deletion
        # still must be performed by a separate, audited storage adapter.
        updated["current_state"] = "expired"
    return updated


def verify_event_chain(
    case: dict[str, Any],
    events: list[dict[str, Any]],
    contract: dict[str, Any] | None = None,
) -> list[str]:
    """Check hash links, timestamp order, policy binding and transition replay."""
    active_contract = contract or load_contract()
    errors: list[str] = []
    state = _initial_state(active_contract)
    previous_hash = _genesis_hash(case, active_contract)
    previous_time: datetime | None = None
    policy_doc = case.get("retention_policy")
    policy_event_digest: str | None = None
    for index, event in enumerate(events):
        if not isinstance(event, dict) or set(event) != EVENT_FIELDS:
            errors.append(f"events[{index}] must match the closed event schema")
            break
        if type(event.get("sequence")) is not int or event["sequence"] != index:
            errors.append(f"events[{index}] sequence is not contiguous")
        if event.get("previous_hash") != previous_hash:
            errors.append(f"events[{index}] previous_hash does not match the chain")
        digest_input = {key: value for key, value in event.items() if key != "event_hash"}
        try:
            computed_hash = _hash(digest_input)
        except (TypeError, ValueError):
            errors.append(f"events[{index}] cannot be canonically hashed")
            break
        if not isinstance(event.get("event_hash"), str) or event["event_hash"] != computed_hash:
            errors.append(f"events[{index}] event hash mismatch")
        try:
            event_time = _parse_utc(event.get("timestamp_utc"))
            if previous_time is not None and event_time < previous_time:
                errors.append(f"events[{index}] timestamp moves backwards")
            previous_time = event_time
        except ValueError as exc:
            errors.append(f"events[{index}] {exc}")
            break
        try:
            if event.get("action") == "record_retention_policy":
                policy_event_digest = event.get("policy_digest")
                state = _apply_event_state(case, state, event, active_contract, policy_doc)
            else:
                state = _apply_event_state(case, state, event, active_contract)
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(f"events[{index}] transition replay failed: {exc}")
            break
        previous_hash = event.get("event_hash", "")
    if policy_event_digest is not None and policy_event_digest != _hash(policy_doc):
        errors.append("current retention policy is not bound to the reviewed policy event")
    if policy_event_digest is None and policy_doc != _retention_template(active_contract):
        errors.append("retention policy differs from the unreviewed initial template")
    actual_state = _state_snapshot(case)
    if actual_state != state:
        errors.append("case current state does not match replayed event-chain state")
    if case.get("durable_case_storage_enabled") is not False:
        errors.append("durable case storage must remain disabled in this harness")
    if case.get("public_dht_publication_enabled") is not False:
        errors.append("public DHT publication must remain disabled")
    return errors


def append_event(
    case: dict[str, Any],
    events: list[dict[str, Any]],
    *,
    actor_ref: str,
    actor_role: str,
    action: str,
    timestamp_utc: str,
    target_actor_ref: str | None = None,
    retention_policy: dict[str, Any] | None = None,
    policy_ref: str | None = None,
    contract: dict[str, Any] | None = None,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Append one authorized event and return new in-memory values, never mutating inputs."""
    active_contract = contract or load_contract()
    current_errors = verify_event_chain(case, events, active_contract)
    if current_errors:
        raise ValueError("existing case/event chain invalid: " + "; ".join(current_errors))
    _parse_utc(timestamp_utc)
    if actor_role not in active_contract["role_values"]:
        raise ValueError("unknown actor role")
    state = _initial_state(active_contract)
    previous_time: datetime | None = None
    policy_doc = case["retention_policy"]
    for existing in events:
        event_time = _parse_utc(existing["timestamp_utc"])
        if previous_time is not None and event_time < previous_time:
            raise ValueError("existing event timestamps are not monotonic")
        previous_time = event_time
        if existing["action"] == "record_retention_policy":
            state = _apply_event_state(case, state, existing, active_contract, policy_doc)
        else:
            state = _apply_event_state(case, state, existing, active_contract)
    if previous_time is not None and _parse_utc(timestamp_utc) < previous_time:
        raise ValueError("new event timestamp cannot precede previous event")
    if not authorize_action(case, actor_ref, actor_role, action):
        raise ValueError("actor is not authorized for requested action")
    transition_options = _transitions(active_contract, action, state["current_state"])
    matched = [t for t in transition_options if t["actor_role"] == actor_role]
    if len(matched) != 1:
        raise ValueError("no unique authorized transition for action and current state")
    transition = matched[0]
    event_policy = retention_policy
    if action == "record_retention_policy":
        if event_policy is None:
            raise ValueError("retention_policy is required for policy review")
        if state["current_state"] == "not_opened" or state["current_state"] == "expired":
            raise ValueError("retention policy must be reviewed on an opened, active case")
        policy_errors = validate_retention_policy(event_policy, active_contract)
        if policy_errors:
            raise ValueError("invalid approved retention policy: " + "; ".join(policy_errors))
        if event_policy.get("jurisdiction_code") != case.get("jurisdiction_code"):
            raise ValueError("retention policy jurisdiction must match the case jurisdiction")
        expected_class = active_contract.get("issue_record_classes", {}).get(case.get("issue_code"))
        if expected_class is None or event_policy.get("record_class") != expected_class:
            raise ValueError("retention policy record class must match the case issue-code mapping")
        new_policy_digest = _hash(event_policy)
    else:
        new_policy_digest = None
        if policy_ref is not None:
            raise ValueError("policy_ref is only permitted with record_retention_policy")
    event = {
        "sequence": len(events),
        "timestamp_utc": timestamp_utc,
        "actor_ref": actor_ref,
        "actor_role": actor_role,
        "action": action,
        "from_state": state["current_state"],
        "to_state": state["current_state"] if transition["to_state"] == "SAME" else transition["to_state"],
        "reason_code": transition["reason_code"],
        "target_actor_ref": target_actor_ref,
        "previous_hash": events[-1]["event_hash"] if events else _genesis_hash(case, active_contract),
        "event_hash": "",
        "policy_ref": policy_ref if action == "record_retention_policy" else None,
        "policy_digest": new_policy_digest,
    }
    next_state = _apply_event_state(case, state, event, active_contract, event_policy)
    event["event_hash"] = _hash({key: value for key, value in event.items() if key != "event_hash"})
    new_case = _set_state(case, next_state)
    if action == "record_retention_policy":
        new_case["retention_policy"] = copy.deepcopy(event_policy)
        new_case["retention_policy_status"] = "approved"
        new_case["retention_review_complete"] = True
    new_events = copy.deepcopy(events) + [event]
    chain_errors = verify_event_chain(new_case, new_events, active_contract)
    if chain_errors:
        raise ValueError("new event failed chain validation: " + "; ".join(chain_errors))
    return new_case, new_events


def authorize_action_for_view(
    case: dict[str, Any],
    actor_ref: str,
    actor_role: str,
    action: str,
) -> bool:
    """Alias kept intentionally explicit for caller-side access-check flows."""
    return authorize_action(case, actor_ref, actor_role, action)


def load_contract(path: Path = CONTRACT_PATH) -> dict[str, Any]:
    """Load the checked-in contract, rejecting duplicate JSON keys."""
    contract = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_reject_duplicate_object_keys)
    errors = validate_contract(contract)
    if errors:
        raise ValueError("invalid appeal-case contract: " + "; ".join(errors))
    return contract


def _reject_duplicate_object_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key: {key}")
        result[key] = value
    return result


def validate_contract(contract: Any) -> list[str]:
    """Check required shape and ensure this artifact remains non-production."""
    if not isinstance(contract, dict):
        return ["contract must be a JSON object"]
    errors: list[str] = []
    required = {
        "schema_id", "schema_version", "contract_id", "title", "maturity",
        "production_enabled", "durable_case_storage_enabled",
        "public_dht_publication_enabled", "external_notifications_enabled",
        "legal_compliance_claimed", "case_fields", "event_fields", "state_values",
        "role_values", "reason_codes", "transitions", "access_policy",
        "retention_policy_template", "issue_code_values", "record_class_values",
        "issue_record_classes", "not_implemented_issue_codes", "prohibited_fields_and_payloads",
        "security_invariants", "nonclaims", "references",
    }
    if set(contract) != required:
        errors.append("top-level fields must match the closed v1 contract")
    if contract.get("schema_id") != "org.luminousdynamics.correction-appeal-case-contract":
        errors.append("unexpected schema_id")
    if type(contract.get("schema_version")) is not int or contract.get("schema_version") != 1:
        errors.append("schema_version must be integer 1")
    if contract.get("contract_id") != "CORRECTION-APPEAL-001":
        errors.append("unexpected contract_id")
    if contract.get("maturity") != "research_only":
        errors.append("maturity must remain research_only")
    for key in ("production_enabled", "durable_case_storage_enabled", "public_dht_publication_enabled", "external_notifications_enabled", "legal_compliance_claimed"):
        if contract.get(key) is not False:
            errors.append(f"{key} must remain false")
    required_case_fields = {
        "case_ref", "case_owner_ref", "service_code", "issue_code", "jurisdiction_code", "current_state",
        "assigned_steward_ref", "initial_decision_actor_ref", "appeal_reviewer_ref",
        "retention_policy_status", "retention_policy", "retention_review_complete", "closed_at_utc",
    }
    if (
        not isinstance(contract.get("case_fields"), list)
        or any(not isinstance(value, str) for value in contract["case_fields"])
        or set(contract["case_fields"]) != required_case_fields
        or len(contract["case_fields"]) != len(required_case_fields)
    ):
        errors.append("case_fields must match the closed v1 case schema")
    if not isinstance(contract.get("event_fields"), list) or set(contract["event_fields"]) != EVENT_FIELDS:
        errors.append("event_fields must match the closed v1 event schema")
    for key in ("state_values", "role_values", "reason_codes", "prohibited_fields_and_payloads", "security_invariants", "nonclaims"):
        value = contract.get(key)
        if not isinstance(value, list) or not value or any(not isinstance(item, str) or not item.strip() for item in value):
            errors.append(f"{key} must be a non-empty string list")
    if isinstance(contract.get("transitions"), list):
        seen = set()
        for index, item in enumerate(contract["transitions"]):
            if not isinstance(item, dict):
                errors.append(f"transitions[{index}] must be an object")
                continue
            marker = (item.get("action"), item.get("from_state"), item.get("to_state"), item.get("actor_role"))
            if any(not isinstance(value, str) for value in marker):
                errors.append(f"transitions[{index}] action/from_state/to_state/actor_role must be strings")
                continue
            if marker in seen:
                errors.append(f"duplicate transition: {marker}")
            seen.add(marker)
            if item.get("actor_role") not in contract.get("role_values", []):
                errors.append(f"transitions[{index}] has an unknown actor role")
            if item.get("reason_code") not in contract.get("reason_codes", []):
                errors.append(f"transitions[{index}] has an unknown reason code")
            if item.get("from_state") not in contract.get("state_values", []) and item.get("from_state") not in {"ANY_AFTER_OPEN"}:
                errors.append(f"transitions[{index}] has an unknown from_state")
            if item.get("to_state") not in contract.get("state_values", []) and item.get("to_state") not in {"SAME"}:
                errors.append(f"transitions[{index}] has an unknown to_state")
    else:
        errors.append("transitions must be a list")
    valid_scopes = {
        "case_owner_only", "owner_or_assigned_case_role", "assigned_case_only",
        "intake_assignment_only", "assigned_appeal_only", "metadata_only",
        "policy_metadata_only", "retention_metadata_only",
    }
    access_policy = contract.get("access_policy")
    if not isinstance(access_policy, list) or not access_policy:
        errors.append("access_policy must be a non-empty array")
    else:
        seen_actions: set[str] = set()
        for index, rule in enumerate(access_policy):
            if not isinstance(rule, dict) or set(rule) != {"action", "roles", "scope"}:
                errors.append(f"access_policy[{index}] must contain exactly action, roles and scope")
                continue
            action = rule.get("action")
            roles = rule.get("roles")
            if not isinstance(action, str) or not action.strip() or action in seen_actions:
                errors.append(f"access_policy[{index}].action must be unique non-empty text")
            elif isinstance(action, str):
                seen_actions.add(action)
            if (
                not isinstance(roles, list)
                or not roles
                or any(not isinstance(role, str) or role not in contract.get("role_values", []) for role in roles)
                or len(roles) != len(set(roles)) if isinstance(roles, list) and all(isinstance(role, str) for role in roles) else True
            ):
                errors.append(f"access_policy[{index}].roles must be unique declared roles")
            if not isinstance(rule.get("scope"), str) or rule["scope"] not in valid_scopes:
                errors.append(f"access_policy[{index}].scope must use the closed v1 enum")
    template = contract.get("retention_policy_template")
    if not isinstance(template, dict) or set(template) != RETENTION_FIELDS or template.get("status") != "not_assessed":
        errors.append("retention_policy_template must remain unassessed and use the closed schema")
    issue_codes = contract.get("issue_code_values")
    record_classes = contract.get("record_class_values")
    issue_mapping = contract.get("issue_record_classes")
    if (
        not isinstance(issue_codes, list)
        or not issue_codes
        or any(not isinstance(value, str) for value in issue_codes)
        or len(issue_codes) != len(set(issue_codes))
    ):
        errors.append("issue_code_values must be a unique string list")
    if (
        not isinstance(record_classes, list)
        or not record_classes
        or any(not isinstance(value, str) for value in record_classes)
        or len(record_classes) != len(set(record_classes))
    ):
        errors.append("record_class_values must be a unique string list")
    if (
        not isinstance(issue_mapping, dict)
        or set(issue_mapping) != set(issue_codes if isinstance(issue_codes, list) else [])
        or any(not isinstance(value, str) or value not in (record_classes if isinstance(record_classes, list) else []) for value in issue_mapping.values())
    ):
        errors.append("issue_record_classes must map every issue code to a declared record class")
    unsupported_codes = contract.get("not_implemented_issue_codes")
    if (
        not isinstance(unsupported_codes, list)
        or any(not isinstance(value, str) or not value.strip() for value in unsupported_codes)
        or len(unsupported_codes) != len(set(unsupported_codes))
        or set(unsupported_codes).intersection(set(issue_codes if isinstance(issue_codes, list) else []))
    ):
        errors.append("not_implemented_issue_codes must be a unique disjoint string list")
    references = contract.get("references")
    if not isinstance(references, list) or not references:
        errors.append("references must be a non-empty array")
    else:
        for index, reference in enumerate(references):
            if not isinstance(reference, dict) or set(reference) != {"id", "url", "use"}:
                errors.append(f"references[{index}] must contain exactly id, url, use")
                continue
            if not isinstance(reference.get("url"), str) or not reference["url"].startswith("https://"):
                errors.append(f"references[{index}].url must be HTTPS")
            if not all(isinstance(reference.get(key), str) and reference[key].strip() for key in ("id", "use")):
                errors.append(f"references[{index}] id and use must be non-empty text")
    return errors


def run_synthetic_demo(source_commit: str, contract: dict[str, Any] | None = None) -> dict[str, Any]:
    """Run a frozen lifecycle through closure and expiry without external effects."""
    active_contract = contract or load_contract()
    if not isinstance(source_commit, str) or not re.fullmatch(r"[0-9a-f]{40}", source_commit):
        raise ValueError("source_commit must be a 40-character lowercase commit SHA")
    case = new_case(
        "CASE-SYN-DEMO-0001",
        "ACTOR-SYN-DEMO-OWNER",
        "SERVICE_APPOINTMENT_INTAKE",
        "ISSUE_RECORD_CORRECTION",
        active_contract,
        jurisdiction_code="JURISDICTION_SYNTHETIC",
    )
    events: list[dict[str, Any]] = []

    def apply(action: str, timestamp: str, actor: str, role: str, **kwargs: Any) -> None:
        nonlocal case, events
        case, events = append_event(
            case, events,
            actor_ref=actor, actor_role=role,
            action=action, timestamp_utc=timestamp,
            contract=active_contract, **kwargs,
        )

    apply("open_case", "2026-01-01T09:00:00Z", "ACTOR-SYN-DEMO-OWNER", "requester")
    policy = {
        "status": "approved",
        "jurisdiction_code": "JURISDICTION_SYNTHETIC",
        "record_class": "service_correction_case",
        "lawful_basis_ref": "POLICY-DEMO-LAWFUL-001",
        "responsible_party_ref": "ACTOR-SYN-DEMO-CONTROLLER",
        "max_retention_days": 1,
        "deletion_procedure_ref": "POLICY-DEMO-DELETE-001",
        "backup_deletion_policy_ref": "POLICY-DEMO-BACKUP-001",
        "review_evidence_ref": "POLICY-DEMO-REVIEW-001",
    }
    apply(
        "record_retention_policy", "2026-01-01T09:00:30Z",
        "ACTOR-SYN-DEMO-PRIVACY", "privacy_officer",
        policy_ref="POLICY-DEMO-RETENTION-001",
        retention_policy=policy,
    )
    apply(
        "acknowledge_case", "2026-01-01T09:01:00Z",
        "ACTOR-SYN-DEMO-INTAKE", "case_steward",
        target_actor_ref="ACTOR-SYN-DEMO-STEWARD-1",
    )
    apply(
        "assign_steward", "2026-01-01T09:02:00Z",
        "ACTOR-SYN-DEMO-STEWARD-1", "case_steward",
        target_actor_ref="ACTOR-SYN-DEMO-STEWARD-2",
    )
    apply("submit_correction", "2026-01-01T09:03:00Z", "ACTOR-SYN-DEMO-OWNER", "requester")
    apply("resume_review", "2026-01-01T09:04:00Z", "ACTOR-SYN-DEMO-STEWARD-2", "case_steward")
    apply("issue_initial_decision", "2026-01-01T09:05:00Z", "ACTOR-SYN-DEMO-STEWARD-2", "case_steward")
    apply("request_appeal", "2026-01-01T09:06:00Z", "ACTOR-SYN-DEMO-OWNER", "requester")
    apply("start_appeal_review", "2026-01-01T09:07:00Z", "ACTOR-SYN-DEMO-REVIEWER", "independent_reviewer")
    apply("issue_appeal_decision", "2026-01-01T09:08:00Z", "ACTOR-SYN-DEMO-REVIEWER", "independent_reviewer")
    apply("close_case", "2026-01-01T09:09:00Z", "ACTOR-SYN-DEMO-STEWARD-2", "case_steward")
    apply("expire_case", "2026-01-02T09:09:00Z", "ACTOR-SYN-DEMO-PRIVACY", "privacy_officer")

    errors = verify_event_chain(case, events, active_contract)
    if errors:
        raise ValueError("synthetic lifecycle failed chain verification: " + "; ".join(errors))
    engine_path = Path(__file__)
    engine_digest = hashlib.sha256(engine_path.read_bytes()).hexdigest()
    contract_digest = _hash(active_contract)
    summaries = [
        {
            key: event[key]
            for key in (
                "sequence", "timestamp_utc", "actor_role", "action", "from_state",
                "to_state", "reason_code", "previous_hash", "event_hash",
            )
        }
        for event in events
    ]
    return {
        "report_schema": "org.luminousdynamics.synthetic-appeal-case-report",
        "report_version": 1,
        "source_commit": source_commit,
        "engine_sha256": engine_digest,
        "contract_sha256": contract_digest,
        "maturity": "synthetic_only",
        "production_enabled": False,
        "durable_case_storage_enabled": False,
        "public_dht_publication_enabled": False,
        "external_notifications_enabled": False,
        "real_identity_data_processed": False,
        "external_endpoints_contacted": False,
        "event_count": len(events),
        "terminal_state": case["current_state"],
        "retention_review_complete": case["retention_review_complete"],
        "independent_appeal_reviewer": case["initial_decision_actor_ref"] != case["appeal_reviewer_ref"],
        "hash_chain_valid": True,
        "chain_head": events[-1]["event_hash"],
        "durable_deletion_performed": False,
        "event_summaries": summaries,
        "nonclaims": [
            "The demonstration uses synthetic opaque actor handles and codes, not real identities.",
            "SHA-256 event chaining is tamper-evident metadata only; it is not a signature or trusted timestamp.",
            "expired is a workflow state, not evidence that records/backups were physically erased.",
            "All persistence adapters, external notifications and public-DHT writes remain disabled.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=CONTRACT_PATH)
    parser.add_argument("--source-commit", help="exact 40-character source commit for synthetic evidence")
    parser.add_argument("--report", type=Path, help="write deterministic synthetic lifecycle evidence JSON")
    args = parser.parse_args()
    try:
        contract = load_contract(args.contract)
        if args.report and not args.source_commit:
            raise ValueError("--report requires --source-commit")
        if args.source_commit:
            report = run_synthetic_demo(args.source_commit, contract)
            if args.report:
                args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            print(
                "SYNTHETIC_APPEAL_CASE=PASS "
                f"events={report['event_count']} state={report['terminal_state']} "
                "hash_chain_valid=true production_enabled=false durable_storage_enabled=false"
            )
        else:
            print(
                "APPEAL_CASE_CONTRACT=PASS "
                f"transitions={len(contract['transitions'])} "
                f"roles={len(contract['role_values'])} "
                "production_enabled=false durable_storage_enabled=false public_dht=false"
            )
    except (OSError, ValueError, TypeError) as exc:
        print(f"APPEAL_CASE_CONTRACT=FAIL {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

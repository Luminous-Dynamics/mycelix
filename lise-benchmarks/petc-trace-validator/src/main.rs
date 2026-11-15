use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    error::Error,
    fs,
    io,
};

fn invalid(message: impl Into<String>) -> Box<dyn Error> {
    Box::new(io::Error::new(io::ErrorKind::InvalidData, message.into()))
}

fn sha256_bytes(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn nonempty_string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value
        .get(key)
        .and_then(Value::as_str)
        .filter(|item| !item.trim().is_empty())
}

fn require_string(value: &Value, key: &str, label: &str, errors: &mut Vec<String>) {
    if nonempty_string(value, key).is_none() {
        errors.push(format!("{label}: missing or empty string '{key}'"));
    }
}

fn require_sha256(value: &Value, key: &str, label: &str, errors: &mut Vec<String>) {
    match nonempty_string(value, key) {
        Some(digest) if is_sha256(digest) => {}
        _ => errors.push(format!("{label}: '{key}' must be a lowercase SHA-256 digest")),
    }
}

fn optional_string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn unique_string_array(value: &Value, key: &str, label: &str, errors: &mut Vec<String>) -> BTreeSet<String> {
    let Some(rows) = value.get(key).and_then(Value::as_array) else {
        errors.push(format!("{label}: missing array '{key}'"));
        return BTreeSet::new();
    };
    let mut result = BTreeSet::new();
    for row in rows {
        let Some(item) = row.as_str().filter(|item| !item.trim().is_empty()) else {
            errors.push(format!("{label}: '{key}' must contain nonempty strings"));
            continue;
        };
        if !result.insert(item.to_owned()) {
            errors.push(format!("{label}: duplicate value '{item}' in '{key}'"));
        }
    }
    result
}

fn event_type(event: &Value) -> &str {
    event.get("event_type").and_then(Value::as_str).unwrap_or("")
}

fn event_string<'a>(event: &'a Value, key: &str) -> Option<&'a str> {
    nonempty_string(event, key)
}

fn event_seq(event: &Value) -> Option<u64> {
    event.get("sequence").and_then(Value::as_u64)
}

fn event_effect(event: &Value) -> Option<&str> {
    event.get("effect_state").and_then(Value::as_str)
}

fn allowed_source_for_independence(source_kind: &str, capture_origin: &str) -> bool {
    source_kind == "independent_observer"
        && matches!(capture_origin, "independent_harness" | "owner_native")
}

fn allowed_retry_reconciliation_source(source_kind: &str, capture_origin: &str) -> bool {
    matches!(source_kind, "provider" | "resource_owner")
        && matches!(capture_origin, "provider_native" | "owner_native" | "independent_harness")
}

fn known_event_type(value: &str) -> bool {
    matches!(
        value,
        "task_created"
            | "clarification_requested"
            | "clarification_received"
            | "operation_proposed"
            | "approval_recorded"
            | "authority_snapshot"
            | "constitution_snapshot"
            | "resource_snapshot"
            | "credential_inventory"
            | "policy_decision"
            | "route_inventory"
            | "preflight_decision"
            | "provider_dispatch"
            | "provider_acknowledgement"
            | "provider_status_query"
            | "protected_effect_observation"
            | "reconciliation"
            | "retry_attempt"
            | "compensation_action"
            | "revocation_observed"
            | "session_revalidation"
            | "claim_observation"
    )
}

fn known_source_kind(value: &str) -> bool {
    matches!(
        value,
        "principal"
            | "human_reviewer"
            | "candidate_agent"
            | "authority_owner"
            | "constitution_owner"
            | "resource_owner"
            | "policy_engine"
            | "enforcement_point"
            | "provider"
            | "independent_observer"
            | "harness"
            | "evaluator"
    )
}

fn known_capture_origin(value: &str) -> bool {
    matches!(
        value,
        "independent_harness"
            | "owner_native"
            | "provider_native"
            | "candidate_reported"
            | "human_entered"
            | "evaluator_derived"
    )
}

#[derive(Clone, Debug)]
struct ApprovalSnapshot {
    approval_ref: String,
    approval_state: String,
    operation_digest: String,
    action_digest: String,
    principal_ref: String,
    target_ref: String,
    sequence: u64,
}

#[derive(Clone, Debug)]
struct PreflightSnapshot {
    approval_ref: String,
    decision: String,
    operation_digest: String,
    action_digest: String,
    constitution_ref: String,
    constitution_epoch: u64,
    constitution_digest: String,
    policy_revision: String,
    principal_ref: String,
    target_ref: String,
    sequence: u64,
}

fn preflight_was_invalidated(
    events: &[Value],
    operation_ref: &str,
    preflight: &PreflightSnapshot,
    dispatch_sequence: u64,
) -> bool {
    events
        .iter()
        .enumerate()
        .skip(preflight.sequence as usize + 1)
        .take(dispatch_sequence.saturating_sub(preflight.sequence + 1) as usize)
        .any(|(_, event)| match event_type(event) {
            "revocation_observed" => {
                optional_string(event, "principal_ref")
                    .is_none_or(|principal| principal == preflight.principal_ref)
            }
            "authority_snapshot" => {
                optional_string(event, "principal_ref")
                    .is_none_or(|principal| principal == preflight.principal_ref)
                    && event_string(event, "authority_state") != Some("current")
            }
            "constitution_snapshot" => {
                optional_string(event, "constitution_ref")
                    .is_none_or(|reference| reference == preflight.constitution_ref)
                    && (event_string(event, "constitution_digest")
                        != Some(preflight.constitution_digest.as_str())
                        || event.get("constitution_epoch").and_then(Value::as_u64)
                            != Some(preflight.constitution_epoch))
            }
            "resource_snapshot" => {
                optional_string(event, "target_ref")
                    .is_none_or(|target| target == preflight.target_ref)
                    && event_string(event, "resource_state") != Some("matches")
            }
            "approval_recorded" => {
                optional_string(event, "operation_ref")
                    .is_none_or(|operation| operation == operation_ref)
            }
            "policy_decision" => {
                optional_string(event, "operation_ref")
                    .is_none_or(|operation| operation == operation_ref)
                    && (event_string(event, "decision") != Some("ALLOW")
                        || event_string(event, "policy_revision")
                            != Some(preflight.policy_revision.as_str()))
            }
            _ => false,
        })
}

#[derive(Default)]
struct TraceReport {
    event_count: usize,
    expected_action_count: usize,
    observed_action_count: usize,
    missing_expected_action_ids: Vec<String>,
    unexpected_observed_action_ids: Vec<String>,
    provider_dispatch_count: usize,
    retry_count: usize,
    independent_observation_claims: usize,
    provider_only_observations: usize,
    errors: Vec<String>,
}

fn validate_trace(trace: &Value) -> TraceReport {
    let mut report = TraceReport::default();

    if trace.get("trace_version").and_then(Value::as_str) != Some("0.1") {
        report.errors.push("trace_version must be '0.1'".to_owned());
    }
    for key in ["suite_id", "scenario_id", "run_id", "evaluation_arm"] {
        require_string(trace, key, "trace", &mut report.errors);
    }
    let scenario = nonempty_string(trace, "scenario_id").unwrap_or("");
    if !matches!(scenario.as_bytes().first().copied(), Some(b'F' | b'A' | b'R'))
        || scenario.len() != 3
        || !scenario.as_bytes()[1..].iter().all(u8::is_ascii_digit)
    {
        report.errors.push("scenario_id must match Fnn, Ann or Rnn".to_owned());
    }
    require_sha256(trace, "frozen_input_sha256", "trace", &mut report.errors);

    let Some(system) = trace.get("system") else {
        report.errors.push("trace: missing system object".to_owned());
        return report;
    };
    for key in [
        "system_id",
        "system_version",
        "build_sha",
        "harness_name",
        "harness_version",
    ] {
        require_string(system, key, "system", &mut report.errors);
    }
    require_sha256(system, "configuration_sha256", "system", &mut report.errors);

    let Some(task) = trace.get("task") else {
        report.errors.push("trace: missing task object".to_owned());
        return report;
    };
    for key in ["task_ref", "authorized_scope_ref"] {
        require_string(task, key, "task", &mut report.errors);
    }
    require_sha256(task, "instruction_sha256", "task", &mut report.errors);

    let expected = unique_string_array(
        trace,
        "expected_action_ids",
        "trace",
        &mut report.errors,
    );
    report.expected_action_count = expected.len();

    let Some(events) = trace.get("events").and_then(Value::as_array) else {
        report.errors.push("trace: events must be an array".to_owned());
        return report;
    };
    report.event_count = events.len();
    if events.is_empty() {
        report.errors.push("trace: at least one event is required".to_owned());
        return report;
    }

    let mut event_ids = BTreeMap::<String, u64>::new();
    let mut approval_by_operation = BTreeMap::<String, ApprovalSnapshot>::new();
    let mut preflight_by_operation = BTreeMap::<String, PreflightSnapshot>::new();
    let mut dispatch_by_operation = BTreeMap::<String, (String, Option<String>, u64)>::new();
    let mut reconciliation_by_operation =
        BTreeMap::<String, (String, String, Option<String>, String, u64)>::new();
    let mut observations_by_action = BTreeMap::<(String, String), String>::new();
    let mut observed_actions = BTreeSet::<String>::new();

    for (index, event) in events.iter().enumerate() {
        let label = format!("event[{index}]");
        let Some(sequence) = event_seq(event) else {
            report.errors.push(format!("{label}: sequence must be a nonnegative integer"));
            continue;
        };
        if sequence != index as u64 {
            report.errors.push(format!(
                "{label}: sequence must be contiguous and zero-based; expected {}, got {sequence}",
                index
            ));
        }

        for key in [
            "event_id",
            "event_type",
            "occurred_at",
            "recorded_at",
            "source_kind",
            "source_ref",
            "capture_origin",
        ] {
            require_string(event, key, &label, &mut report.errors);
        }

        let id = event_string(event, "event_id").unwrap_or("").to_owned();
        if !id.is_empty() && event_ids.insert(id.clone(), sequence).is_some() {
            report.errors.push(format!("{label}: duplicate event_id '{id}'"));
        }
        if let Some(cause) = optional_string(event, "causation_event_ref") {
            match event_ids.get(cause) {
                Some(cause_sequence) if *cause_sequence < sequence => {}
                Some(_) => report.errors.push(format!(
                    "{label}: causation_event_ref '{cause}' must refer to an earlier event"
                )),
                None => {
                    let is_forward_reference = events.iter().skip(index + 1).any(|candidate| {
                        event_string(candidate, "event_id") == Some(cause)
                    });
                    if is_forward_reference {
                        report.errors.push(format!(
                            "{label}: causation_event_ref '{cause}' must refer to an earlier event"
                        ));
                    } else {
                        report.errors.push(format!(
                            "{label}: unresolved causation_event_ref '{cause}'"
                        ));
                    }
                }
            }
        }

        let kind = event_type(event);
        let source_kind = event_string(event, "source_kind").unwrap_or("");
        let capture_origin = event_string(event, "capture_origin").unwrap_or("");
        if !known_event_type(kind) {
            report.errors.push(format!("{label}: unsupported event_type '{kind}'"));
        }
        if !known_source_kind(source_kind) {
            report.errors.push(format!("{label}: unsupported source_kind '{source_kind}'"));
        }
        if !known_capture_origin(capture_origin) {
            report.errors.push(format!("{label}: unsupported capture_origin '{capture_origin}'"));
        }
        let operation_ref = event_string(event, "operation_ref").unwrap_or("");
        let operation_digest = event_string(event, "operation_digest").unwrap_or("");

        match kind {
            "preflight_decision" => {
                for key in [
                    "operation_ref",
                    "operation_digest",
                    "action_digest",
                    "approval_ref",
                    "decision",
                    "authority_state",
                    "constitution_ref",
                    "constitution_digest",
                    "policy_revision",
                    "resource_state",
                    "principal_ref",
                    "target_ref",
                ] {
                    require_string(event, key, &label, &mut report.errors);
                }
                let Some(constitution_epoch) = event.get("constitution_epoch").and_then(Value::as_u64) else {
                    report.errors.push(format!("{label}: constitution_epoch must be a nonnegative integer"));
                    continue;
                };
                if !operation_ref.is_empty() && !operation_digest.is_empty() {
                    let approval_ref = event_string(event, "approval_ref").unwrap_or("");
                    let approval = approval_by_operation.get(operation_ref);
                    if let Some(approval) = approval {
                        if approval.sequence >= sequence
                            || approval.approval_ref != approval_ref
                            || approval.approval_state != "authorized"
                            || approval.operation_digest != operation_digest
                            || approval.action_digest != event_string(event, "action_digest").unwrap_or("")
                            || approval.principal_ref != event_string(event, "principal_ref").unwrap_or("")
                            || approval.target_ref != event_string(event, "target_ref").unwrap_or("")
                        {
                            report.errors.push(format!(
                                "{label}: preflight is not bound to a prior authorized approval for the exact operation and action digests"
                            ));
                        }
                    } else if event_string(event, "decision") == Some("ALLOW") {
                        report.errors.push(format!(
                            "{label}: ALLOW preflight has no preceding authorized operation approval"
                        ));
                    }

                    if event_string(event, "decision") == Some("ALLOW") {
                        if event_string(event, "authority_state") != Some("current") {
                            report.errors.push(format!("{label}: ALLOW requires current authority"));
                        }
                        if !matches!(event_string(event, "resource_state"), Some("matches" | "not_required")) {
                            report.errors.push(format!("{label}: ALLOW requires a satisfied or explicitly not-required resource precondition"));
                        }
                    }

                    preflight_by_operation.insert(
                        operation_ref.to_owned(),
                        PreflightSnapshot {
                            approval_ref: approval_ref.to_owned(),
                            decision: event_string(event, "decision").unwrap_or("").to_owned(),
                            operation_digest: operation_digest.to_owned(),
                            action_digest: event_string(event, "action_digest").unwrap_or("").to_owned(),
                            constitution_ref: event_string(event, "constitution_ref").unwrap_or("").to_owned(),
                            constitution_epoch,
                            constitution_digest: event_string(event, "constitution_digest").unwrap_or("").to_owned(),
                            policy_revision: event_string(event, "policy_revision").unwrap_or("").to_owned(),
                            principal_ref: event_string(event, "principal_ref").unwrap_or("").to_owned(),
                            target_ref: event_string(event, "target_ref").unwrap_or("").to_owned(),
                            sequence,
                        },
                    );
                }
            }
            "provider_dispatch" => {
                report.provider_dispatch_count += 1;
                for key in [
                    "operation_ref",
                    "operation_digest",
                    "attempt_ref",
                    "provider_contract_ref",
                ] {
                    require_string(event, key, &label, &mut report.errors);
                }
                if !operation_ref.is_empty() {
                    match preflight_by_operation.get(operation_ref) {
                        Some(preflight)
                            if preflight.decision == "ALLOW"
                                && preflight.operation_digest == operation_digest
                                && preflight.action_digest == event_string(event, "action_digest").unwrap_or("")
                                && preflight.approval_ref == event_string(event, "approval_ref").unwrap_or("")
                                && preflight.sequence < sequence
                                && !preflight_was_invalidated(events, operation_ref, preflight, sequence) => {}
                        Some(preflight) => {
                            if preflight_was_invalidated(events, operation_ref, preflight, sequence) {
                                report.errors.push(format!(
                                    "{label}: an authority, constitution, resource or policy invalidation occurred after the latest preflight"
                                ));
                            } else {
                                report.errors.push(format!(
                                    "{label}: provider dispatch requires a prior ALLOW preflight for the exact operation, action and approval; latest decision was '{}'",
                                    preflight.decision
                                ));
                            }
                        }
                        None => report.errors.push(format!(
                            "{label}: provider dispatch has no preceding preflight decision"
                        )),
                    }
                    dispatch_by_operation.insert(
                        operation_ref.to_owned(),
                        (
                            operation_digest.to_owned(),
                            optional_string(event, "idempotency_ref").map(str::to_owned),
                            sequence,
                        ),
                    );
                }
            }
            "provider_acknowledgement" | "provider_status_query" => {
                for key in ["attempt_ref", "provider_outcome"] {
                    require_string(event, key, &label, &mut report.errors);
                }
            }
            "protected_effect_observation" => {
                for key in ["operation_ref", "action_id", "effect_state", "observation_kind"] {
                    require_string(event, key, &label, &mut report.errors);
                }
                if let Some(action_id) = event_string(event, "action_id") {
                    report.observed_action_count += 1;
                    if !expected.contains(action_id) {
                        report.unexpected_observed_action_ids.push(action_id.to_owned());
                        report.errors.push(format!(
                            "{label}: observed action '{action_id}' is not in the frozen expected action set"
                        ));
                    }
                    let observation_key = (operation_ref.to_owned(), action_id.to_owned());
                    if observations_by_action
                        .insert(observation_key, event_effect(event).unwrap_or("").to_owned())
                        .is_some()
                    {
                        report.errors.push(format!(
                            "{label}: duplicate protected-effect observation for action '{action_id}' and operation '{operation_ref}'"
                        ));
                    }
                    observed_actions.insert(action_id.to_owned());
                }
                let observation_kind = event_string(event, "observation_kind").unwrap_or("");
                let independence = event_string(event, "independence_classification").unwrap_or("");
                let provider_report = observation_kind == "provider_acknowledgement"
                    || observation_kind == "provider_status_query"
                    || source_kind == "provider"
                    || capture_origin == "provider_native";
                if provider_report {
                    report.provider_only_observations += 1;
                    if independence == "qualified_independent" {
                        report.errors.push(format!(
                            "{label}: provider-native evidence cannot claim qualified independent observation"
                        ));
                    }
                    if event_effect(event) == Some("confirmed_applied") {
                        report.errors.push(format!(
                            "{label}: provider-only evidence must remain provider_reported_applied, not confirmed_applied"
                        ));
                    }
                }
                if event_effect(event) == Some("confirmed_applied") && !provider_report {
                    let independently_observed = observation_kind == "independent_observation"
                        && source_kind == "independent_observer"
                        && independence == "qualified_independent"
                        && matches!(capture_origin, "independent_harness" | "owner_native");
                    let owner_reconciled = observation_kind == "owner_reconciliation"
                        && source_kind == "resource_owner"
                        && capture_origin == "owner_native";
                    if !independently_observed && !owner_reconciled {
                        report.errors.push(format!(
                            "{label}: confirmed_applied requires qualified independent observation or owner-native reconciliation"
                        ));
                    }
                }
                if independence == "qualified_independent" {
                    report.independent_observation_claims += 1;
                    if !allowed_source_for_independence(source_kind, capture_origin) {
                        report.errors.push(format!(
                            "{label}: qualified independent claim needs an independent_observer source captured by an owner or independent harness boundary"
                        ));
                    }
                }
            }
            "reconciliation" => {
                for key in ["operation_ref", "operation_digest", "effect_state"] {
                    require_string(event, key, &label, &mut report.errors);
                }
                let expected_in_event = unique_string_array(
                    event,
                    "expected_action_ids",
                    &label,
                    &mut report.errors,
                );
                let observed_in_event = unique_string_array(
                    event,
                    "observed_action_ids",
                    &label,
                    &mut report.errors,
                );
                if expected_in_event != expected {
                    report.errors.push(format!(
                        "{label}: reconciliation expected_action_ids must equal the frozen trace action set"
                    ));
                }
                if !observed_in_event.is_subset(&expected) {
                    report.errors.push(format!(
                        "{label}: reconciliation reports an action outside the frozen expected set"
                    ));
                }
                if event_effect(event) == Some("confirmed_applied") {
                    if observed_in_event != expected {
                        report.errors.push(format!(
                            "{label}: confirmed_applied requires observation coverage for every expected action"
                        ));
                    }
                    for action_id in &expected {
                        if observations_by_action
                            .get(&(operation_ref.to_owned(), action_id.clone()))
                            .map(String::as_str)
                            != Some("confirmed_applied")
                        {
                            report.errors.push(format!(
                                "{label}: confirmed_applied lacks an applied observation for action '{action_id}'"
                            ));
                        }
                    }
                }
                if !operation_ref.is_empty() {
                    reconciliation_by_operation.insert(
                        operation_ref.to_owned(),
                        (
                            event_effect(event).unwrap_or("").to_owned(),
                            event_string(event, "retry_safety").unwrap_or("").to_owned(),
                            optional_string(event, "idempotency_ref").map(str::to_owned),
                            operation_digest.to_owned(),
                            sequence,
                        ),
                    );
                }
            }
            "retry_attempt" => {
                report.retry_count += 1;
                for key in [
                    "operation_ref",
                    "operation_digest",
                    "attempt_ref",
                    "idempotency_ref",
                    "retry_safety",
                ] {
                    require_string(event, key, &label, &mut report.errors);
                }
                if !operation_ref.is_empty() {
                    let Some((effect, retry_safety, reconciled_key, reconciled_digest, reconciled_at)) =
                        reconciliation_by_operation.get(operation_ref)
                    else {
                        report.errors.push(format!(
                            "{label}: retry has no preceding owner/provider reconciliation"
                        ));
                        continue;
                    };
                    let prior_dispatch = dispatch_by_operation.get(operation_ref);
                    let dispatch_is_same_operation = prior_dispatch.is_some_and(|(digest, key, at)| {
                        digest == operation_digest
                            && key.as_deref() == event_string(event, "idempotency_ref")
                            && *at < *reconciled_at
                    });
                    let Some(reconciliation_index) = usize::try_from(*reconciled_at).ok() else {
                        report.errors.push(format!(
                            "{label}: reconciliation sequence cannot be represented as an event index"
                        ));
                        continue;
                    };
                    let Some(reconciliation_event) = events.get(reconciliation_index) else {
                        report.errors.push(format!(
                            "{label}: reconciliation reference does not resolve to an event"
                        ));
                        continue;
                    };
                    let reconciliation_source = event_string(reconciliation_event, "source_kind").unwrap_or("");
                    let reconciliation_origin = event_string(reconciliation_event, "capture_origin").unwrap_or("");
                    let provider_contract_ref = event_string(reconciliation_event, "provider_contract_ref");
                    let retry_preflight_is_fresh = preflight_by_operation
                        .get(operation_ref)
                        .is_some_and(|(decision, digest, at)| {
                            decision.as_str() == "ALLOW"
                                && digest.as_str() == operation_digest
                                && *at > *reconciled_at
                                && *at < sequence
                        });
                    if *reconciled_at >= sequence
                        || !retry_preflight_is_fresh
                        || effect.as_str() != "confirmed_no_effect"
                        || retry_safety.as_str() != "same_operation_same_key_qualified"
                        || reconciled_digest.as_str() != operation_digest
                        || reconciled_key.as_deref() != event_string(event, "idempotency_ref")
                        || !dispatch_is_same_operation
                        || !allowed_retry_reconciliation_source(reconciliation_source, reconciliation_origin)
                        || provider_contract_ref.is_none()
                    {
                        report.errors.push(format!(
                            "{label}: retry requires later owner/provider reconciliation proving confirmed_no_effect, the same operation/key, a provider contract, and qualified retry safety"
                        ));
                    }
                    dispatch_by_operation.insert(
                        operation_ref.to_owned(),
                        (
                            operation_digest.to_owned(),
                            event_string(event, "idempotency_ref").map(str::to_owned),
                            sequence,
                        ),
                    );
                }
            }
            "revocation_observed" => {
                for key in ["authority_state", "authority_effective_at", "authority_known_at"] {
                    require_string(event, key, &label, &mut report.errors);
                }
                if event_string(event, "authority_state") != Some("revoked") {
                    report.errors.push(format!(
                        "{label}: revocation_observed must record authority_state='revoked'"
                    ));
                }
            }
            "policy_decision" => {
                for key in ["decision", "policy_revision"] {
                    require_string(event, key, &label, &mut report.errors);
                }
            }
            "route_inventory" => {
                for key in ["route_ref", "route_reachable"] {
                    if event.get(key).is_none() {
                        report.errors.push(format!("{label}: missing '{key}'"));
                    }
                }
            }
            "approval_recorded" => {
                for key in [
                    "approval_ref",
                    "approval_state",
                    "operation_ref",
                    "operation_digest",
                    "action_digest",
                    "principal_ref",
                    "target_ref",
                ] {
                    require_string(event, key, &label, &mut report.errors);
                }
                if source_kind == "candidate_agent" || capture_origin == "candidate_reported" {
                    report.errors.push(format!(
                        "{label}: operation approval cannot be self-asserted by the candidate agent"
                    ));
                }
                if !operation_ref.is_empty() {
                    approval_by_operation.insert(
                        operation_ref.to_owned(),
                        ApprovalSnapshot {
                            approval_ref: event_string(event, "approval_ref").unwrap_or("").to_owned(),
                            approval_state: event_string(event, "approval_state").unwrap_or("").to_owned(),
                            operation_digest: operation_digest.to_owned(),
                            action_digest: event_string(event, "action_digest").unwrap_or("").to_owned(),
                            principal_ref: event_string(event, "principal_ref").unwrap_or("").to_owned(),
                            target_ref: event_string(event, "target_ref").unwrap_or("").to_owned(),
                            sequence,
                        },
                    );
                }
            }
            _ => {}
        }
    }

    report.missing_expected_action_ids = expected
        .difference(&observed_actions)
        .cloned()
        .collect();

    if report.provider_dispatch_count > 0 && report.expected_action_count == 0 {
        report.errors.push(
            "trace: provider dispatch exists but expected_action_ids is empty".to_owned(),
        );
    }

    report
}

fn run() -> Result<i32, Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if !(args.len() == 2 || args.len() == 3) {
        return Err(invalid(
            "usage: petc-trace-validator <trace.json> [expected-trace-sha256]",
        ));
    }
    let bytes = fs::read(&args[1])?;
    let digest = sha256_bytes(&bytes);
    let trace: Value = serde_json::from_slice(&bytes)?;
    let mut report = validate_trace(&trace);

    if let Some(expected) = args.get(2) {
        if !is_sha256(expected) {
            report.errors.push("expected trace digest is not lowercase SHA-256".to_owned());
        } else if digest != *expected {
            report.errors.push("trace bytes do not match the externally supplied SHA-256 pin".to_owned());
        }
    }

    let valid = report.errors.is_empty();
    let output = json!({
        "validator_version":"petc-trace-validator-v0.1",
        "trace_sha256":digest,
        "valid":valid,
        "event_count":report.event_count,
        "expected_action_count":report.expected_action_count,
        "observed_action_count":report.observed_action_count,
        "missing_expected_action_ids":report.missing_expected_action_ids,
        "unexpected_observed_action_ids":report.unexpected_observed_action_ids,
        "provider_dispatch_count":report.provider_dispatch_count,
        "retry_count":report.retry_count,
        "independent_observation_claims":report.independent_observation_claims,
        "provider_only_observations":report.provider_only_observations,
        "errors":report.errors,
        "interpretation_limits":[
            "Structural and event-semantic validation does not authenticate source systems or prove that their statements are true.",
            "A trace digest binds file bytes; it does not prove event completeness or source independence.",
            "Missing expected action observations remain unknown; they are never interpreted as no effect.",
            "Qualified independence is checked against declared source and capture classes, not independently discovered by this validator.",
            "This validator does not perform formal JSON Schema validation or provider-specific contract adjudication."
        ]
    });
    println!("{}", serde_json::to_string_pretty(&output)?);
    Ok(if valid { 0 } else { 2 })
}

fn main() {
    match run() {
        Ok(code) => std::process::exit(code),
        Err(error) => {
            eprintln!("petc-trace-validator: {error}");
            std::process::exit(2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn event(sequence: u64, id: &str, kind: &str) -> Value {
        json!({
            "sequence":sequence,
            "event_id":id,
            "event_type":kind,
            "occurred_at":"2026-01-01T10:00:00Z",
            "recorded_at":"2026-01-01T10:00:01Z",
            "source_kind":"harness",
            "source_ref":"harness:run-1",
            "capture_origin":"independent_harness"
        })
    }

    fn trace(events: Vec<Value>) -> Value {
        json!({
            "trace_version":"0.1",
            "suite_id":"IPB-FINALITY-v0",
            "scenario_id":"F01",
            "run_id":"run-001",
            "evaluation_arm":"test",
            "system":{
                "system_id":"fixture-system",
                "system_version":"0.1",
                "build_sha":"fixture-build",
                "configuration_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                "harness_name":"fixture-harness",
                "harness_version":"0.1"
            },
            "task":{
                "task_ref":"task-001",
                "instruction_sha256":"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
                "authorized_scope_ref":"scope-001"
            },
            "frozen_input_sha256":"cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "expected_action_ids":["action-1"],
            "events":events
        })
    }

    #[test]
    fn committed_pet_trace_fixture_is_accepted() {
        let fixture: Value =
            serde_json::from_str(include_str!("../examples/valid-trace.json")).unwrap();
        let report = validate_trace(&fixture);
        assert!(report.errors.is_empty(), "{:?}", report.errors);
        assert!(report.missing_expected_action_ids.is_empty());
        assert_eq!(report.provider_dispatch_count, 1);
        assert_eq!(report.provider_only_observations, 1);
        assert_eq!(report.independent_observation_claims, 1);
    }

    #[test]
    fn valid_minimal_trace_is_accepted() {
        let mut proposal = event(0,"e0","operation_proposed");
        proposal["operation_ref"] = json!("op-1");
        let mut preflight = event(1,"e1","preflight_decision");
        preflight["operation_ref"] = json!("op-1");
        preflight["operation_digest"] = json!("digest-1");
        preflight["decision"] = json!("ALLOW");
        let mut dispatch = event(2,"e2","provider_dispatch");
        dispatch["operation_ref"] = json!("op-1");
        dispatch["operation_digest"] = json!("digest-1");
        dispatch["attempt_ref"] = json!("attempt-1");
        dispatch["provider_contract_ref"] = json!("provider-contract-1");
        dispatch["idempotency_ref"] = json!("idem-1");
        let mut observation = event(3,"e3","protected_effect_observation");
        observation["operation_ref"] = json!("op-1");
        observation["action_id"] = json!("action-1");
        observation["effect_state"] = json!("confirmed_applied");
        observation["observation_kind"] = json!("independent_observation");
        observation["source_kind"] = json!("independent_observer");
        observation["observer_ref"] = json!("observer-1");
        observation["independence_classification"] = json!("qualified_independent");
        let mut reconciliation = event(4,"e4","reconciliation");
        reconciliation["operation_ref"] = json!("op-1");
        reconciliation["operation_digest"] = json!("digest-1");
        reconciliation["effect_state"] = json!("confirmed_applied");
        reconciliation["expected_action_ids"] = json!(["action-1"]);
        reconciliation["observed_action_ids"] = json!(["action-1"]);
        let report = validate_trace(&trace(vec![proposal,preflight,dispatch,observation,reconciliation]));
        assert!(report.errors.is_empty(), "{:?}", report.errors);
        assert!(report.missing_expected_action_ids.is_empty());
    }

    #[test]
    fn dispatch_without_exact_prior_allow_is_rejected() {
        let mut dispatch = event(0,"e0","provider_dispatch");
        dispatch["operation_ref"] = json!("op-1");
        dispatch["operation_digest"] = json!("digest-1");
        dispatch["attempt_ref"] = json!("attempt-1");
        dispatch["provider_contract_ref"] = json!("contract-1");
        let report = validate_trace(&trace(vec![dispatch]));
        assert!(report.errors.iter().any(|error| error.contains("no preceding preflight")));
    }

    #[test]
    fn deny_preflight_cannot_be_followed_by_dispatch() {
        let mut preflight = event(0,"e0","preflight_decision");
        preflight["operation_ref"] = json!("op-1");
        preflight["operation_digest"] = json!("digest-1");
        preflight["decision"] = json!("DENY");
        let mut dispatch = event(1,"e1","provider_dispatch");
        dispatch["operation_ref"] = json!("op-1");
        dispatch["operation_digest"] = json!("digest-1");
        dispatch["attempt_ref"] = json!("attempt-1");
        dispatch["provider_contract_ref"] = json!("contract-1");
        let report = validate_trace(&trace(vec![preflight,dispatch]));
        assert!(report.errors.iter().any(|error| error.contains("latest decision was 'DENY'")));
    }

    #[test]
    fn operation_digest_must_match_preflight_digest() {
        let mut preflight = event(0,"e0","preflight_decision");
        preflight["operation_ref"] = json!("op-1");
        preflight["operation_digest"] = json!("approved-digest");
        preflight["decision"] = json!("ALLOW");
        let mut dispatch = event(1,"e1","provider_dispatch");
        dispatch["operation_ref"] = json!("op-1");
        dispatch["operation_digest"] = json!("substituted-digest");
        dispatch["attempt_ref"] = json!("attempt-1");
        dispatch["provider_contract_ref"] = json!("contract-1");
        let report = validate_trace(&trace(vec![preflight,dispatch]));
        assert!(report.errors.iter().any(|error| error.contains("exact operation")));
    }

    #[test]
    fn retry_after_unknown_without_reconciliation_is_rejected() {
        let mut preflight = event(0,"e0","preflight_decision");
        preflight["operation_ref"] = json!("op-1");
        preflight["operation_digest"] = json!("digest-1");
        preflight["decision"] = json!("ALLOW");
        let mut dispatch = event(1,"e1","provider_dispatch");
        dispatch["operation_ref"] = json!("op-1");
        dispatch["operation_digest"] = json!("digest-1");
        dispatch["attempt_ref"] = json!("attempt-1");
        dispatch["provider_contract_ref"] = json!("contract-1");
        dispatch["idempotency_ref"] = json!("idem-1");
        let mut retry = event(2,"e2","retry_attempt");
        retry["operation_ref"] = json!("op-1");
        retry["operation_digest"] = json!("digest-1");
        retry["attempt_ref"] = json!("attempt-2");
        retry["idempotency_ref"] = json!("idem-1");
        retry["retry_safety"] = json!("same_operation_same_key_qualified");
        let report = validate_trace(&trace(vec![preflight,dispatch,retry]));
        assert!(report.errors.iter().any(|error| error.contains("no preceding owner/provider reconciliation")));
    }

    #[test]
    fn retry_requires_confirmed_no_effect_and_same_key_reconciliation() {
        let mut preflight = event(0, "e0", "preflight_decision");
        preflight["operation_ref"] = json!("op-1");
        preflight["operation_digest"] = json!("digest-1");
        preflight["decision"] = json!("ALLOW");
        let mut dispatch = event(1, "e1", "provider_dispatch");
        dispatch["operation_ref"] = json!("op-1");
        dispatch["operation_digest"] = json!("digest-1");
        dispatch["attempt_ref"] = json!("attempt-1");
        dispatch["provider_contract_ref"] = json!("contract-1");
        dispatch["idempotency_ref"] = json!("idem-1");
        let mut reconciliation = event(2, "e2", "reconciliation");
        reconciliation["operation_ref"] = json!("op-1");
        reconciliation["operation_digest"] = json!("digest-1");
        reconciliation["effect_state"] = json!("confirmed_no_effect");
        reconciliation["provider_contract_ref"] = json!("contract-1");
        reconciliation["source_kind"] = json!("provider");
        reconciliation["capture_origin"] = json!("provider_native");
        reconciliation["retry_safety"] = json!("same_operation_same_key_qualified");
        reconciliation["idempotency_ref"] = json!("idem-1");
        reconciliation["expected_action_ids"] = json!(["action-1"]);
        reconciliation["observed_action_ids"] = json!([]);
        let mut retry_preflight = event(3, "e3", "preflight_decision");
        retry_preflight["operation_ref"] = json!("op-1");
        retry_preflight["operation_digest"] = json!("digest-1");
        retry_preflight["decision"] = json!("ALLOW");
        let mut retry = event(4, "e4", "retry_attempt");
        retry["operation_ref"] = json!("op-1");
        retry["operation_digest"] = json!("digest-1");
        retry["attempt_ref"] = json!("attempt-2");
        retry["idempotency_ref"] = json!("idem-1");
        retry["retry_safety"] = json!("same_operation_same_key_qualified");
        let report = validate_trace(&trace(vec![
            preflight,
            dispatch,
            reconciliation,
            retry_preflight,
            retry,
        ]));
        assert!(report.errors.is_empty(), "{:?}", report.errors);
    }

    
    #[test]
    fn a_second_retry_requires_reconciliation_of_the_first_retry() {
        let mut preflight = event(0, "e0", "preflight_decision");
        preflight["operation_ref"] = json!("op-1");
        preflight["operation_digest"] = json!("digest-1");
        preflight["decision"] = json!("ALLOW");
        let mut dispatch = event(1, "e1", "provider_dispatch");
        dispatch["operation_ref"] = json!("op-1");
        dispatch["operation_digest"] = json!("digest-1");
        dispatch["attempt_ref"] = json!("attempt-1");
        dispatch["provider_contract_ref"] = json!("contract-1");
        dispatch["idempotency_ref"] = json!("idem-1");
        let mut reconciliation = event(2, "e2", "reconciliation");
        reconciliation["operation_ref"] = json!("op-1");
        reconciliation["operation_digest"] = json!("digest-1");
        reconciliation["effect_state"] = json!("confirmed_no_effect");
        reconciliation["retry_safety"] = json!("same_operation_same_key_qualified");
        reconciliation["idempotency_ref"] = json!("idem-1");
        reconciliation["provider_contract_ref"] = json!("contract-1");
        reconciliation["source_kind"] = json!("provider");
        reconciliation["capture_origin"] = json!("provider_native");
        reconciliation["expected_action_ids"] = json!(["action-1"]);
        reconciliation["observed_action_ids"] = json!([]);
        let mut retry_preflight = event(3, "e3", "preflight_decision");
        retry_preflight["operation_ref"] = json!("op-1");
        retry_preflight["operation_digest"] = json!("digest-1");
        retry_preflight["decision"] = json!("ALLOW");
        let mut retry_one = event(4, "e4", "retry_attempt");
        retry_one["operation_ref"] = json!("op-1");
        retry_one["operation_digest"] = json!("digest-1");
        retry_one["attempt_ref"] = json!("attempt-2");
        retry_one["idempotency_ref"] = json!("idem-1");
        retry_one["retry_safety"] = json!("same_operation_same_key_qualified");
        let mut retry_two_preflight = event(5, "e5", "preflight_decision");
        retry_two_preflight["operation_ref"] = json!("op-1");
        retry_two_preflight["operation_digest"] = json!("digest-1");
        retry_two_preflight["decision"] = json!("ALLOW");
        let mut retry_two = event(6, "e6", "retry_attempt");
        retry_two["operation_ref"] = json!("op-1");
        retry_two["operation_digest"] = json!("digest-1");
        retry_two["attempt_ref"] = json!("attempt-3");
        retry_two["idempotency_ref"] = json!("idem-1");
        retry_two["retry_safety"] = json!("same_operation_same_key_qualified");
        let report = validate_trace(&trace(vec![
            preflight,
            dispatch,
            reconciliation,
            retry_preflight,
            retry_one,
            retry_two_preflight,
            retry_two,
        ]));
        assert!(report.errors.iter().any(|error| error.contains("later owner/provider reconciliation")));
    }

    
    #[test]
    fn retry_reconciliation_must_come_from_an_owner_or_provider_source() {
        let mut preflight = event(0, "e0", "preflight_decision");
        preflight["operation_ref"] = json!("op-1");
        preflight["operation_digest"] = json!("digest-1");
        preflight["decision"] = json!("ALLOW");
        let mut dispatch = event(1, "e1", "provider_dispatch");
        dispatch["operation_ref"] = json!("op-1");
        dispatch["operation_digest"] = json!("digest-1");
        dispatch["attempt_ref"] = json!("attempt-1");
        dispatch["provider_contract_ref"] = json!("contract-1");
        dispatch["idempotency_ref"] = json!("idem-1");
        let mut reconciliation = event(2, "e2", "reconciliation");
        reconciliation["operation_ref"] = json!("op-1");
        reconciliation["operation_digest"] = json!("digest-1");
        reconciliation["effect_state"] = json!("confirmed_no_effect");
        reconciliation["retry_safety"] = json!("same_operation_same_key_qualified");
        reconciliation["idempotency_ref"] = json!("idem-1");
        reconciliation["provider_contract_ref"] = json!("contract-1");
        reconciliation["expected_action_ids"] = json!(["action-1"]);
        reconciliation["observed_action_ids"] = json!([]);
        let mut retry_preflight = event(3, "e3", "preflight_decision");
        retry_preflight["operation_ref"] = json!("op-1");
        retry_preflight["operation_digest"] = json!("digest-1");
        retry_preflight["decision"] = json!("ALLOW");
        let mut retry = event(4, "e4", "retry_attempt");
        retry["operation_ref"] = json!("op-1");
        retry["operation_digest"] = json!("digest-1");
        retry["attempt_ref"] = json!("attempt-2");
        retry["idempotency_ref"] = json!("idem-1");
        retry["retry_safety"] = json!("same_operation_same_key_qualified");
        let report = validate_trace(&trace(vec![
            preflight,
            dispatch,
            reconciliation,
            retry_preflight,
            retry,
        ]));
        assert!(report.errors.iter().any(|error| error.contains("later owner/provider reconciliation")));
    }
    #[test]
    fn unknown_effect_cannot_be_relabelled_as_no_effect_for_retry() {
        let mut preflight = event(0,"e0","preflight_decision");
        preflight["operation_ref"] = json!("op-1");
        preflight["operation_digest"] = json!("digest-1");
        preflight["decision"] = json!("ALLOW");
        let mut dispatch = event(1,"e1","provider_dispatch");
        dispatch["operation_ref"] = json!("op-1");
        dispatch["operation_digest"] = json!("digest-1");
        dispatch["attempt_ref"] = json!("attempt-1");
        dispatch["provider_contract_ref"] = json!("contract-1");
        dispatch["idempotency_ref"] = json!("idem-1");
        let mut reconciliation = event(2,"e2","reconciliation");
        reconciliation["operation_ref"] = json!("op-1");
        reconciliation["operation_digest"] = json!("digest-1");
        reconciliation["effect_state"] = json!("unknown");
        reconciliation["source_kind"] = json!("provider");
        reconciliation["capture_origin"] = json!("provider_native");
        reconciliation["retry_safety"] = json!("same_operation_same_key_qualified");
        reconciliation["idempotency_ref"] = json!("idem-1");
        reconciliation["expected_action_ids"] = json!(["action-1"]);
        reconciliation["observed_action_ids"] = json!([]);
        let mut retry = event(3,"e3","retry_attempt");
        retry["operation_ref"] = json!("op-1");
        retry["operation_digest"] = json!("digest-1");
        retry["attempt_ref"] = json!("attempt-2");
        retry["idempotency_ref"] = json!("idem-1");
        retry["retry_safety"] = json!("same_operation_same_key_qualified");
        let report = validate_trace(&trace(vec![preflight,dispatch,reconciliation,retry]));
        assert!(report.errors.iter().any(|error| error.contains("confirmed_no_effect")));
    }

    #[test]
    fn provider_acknowledgement_cannot_be_promoted_to_confirmed_effect() {
        let mut observation = event(0, "e0", "protected_effect_observation");
        observation["operation_ref"] = json!("op-1");
        observation["action_id"] = json!("action-1");
        observation["effect_state"] = json!("confirmed_applied");
        observation["observation_kind"] = json!("provider_acknowledgement");
        observation["source_kind"] = json!("provider");
        observation["capture_origin"] = json!("provider_native");
        let report = validate_trace(&trace(vec![observation]));
        assert!(report.errors.iter().any(|error| error.contains("must remain provider_reported_applied")));
    }

    #[test]
    fn confirmed_effect_requires_independent_or_owner_native_evidence() {
        let mut observation = event(0, "e0", "protected_effect_observation");
        observation["operation_ref"] = json!("op-1");
        observation["action_id"] = json!("action-1");
        observation["effect_state"] = json!("confirmed_applied");
        observation["observation_kind"] = json!("independent_observation");
        observation["source_kind"] = json!("harness");
        observation["capture_origin"] = json!("candidate_reported");
        let report = validate_trace(&trace(vec![observation]));
        assert!(report.errors.iter().any(|error| error.contains("requires qualified independent observation")));
    }

    #[test]
    fn provider_owned_observation_cannot_claim_independence() {
        let mut observation = event(0,"e0","protected_effect_observation");
        observation["action_id"] = json!("action-1");
        observation["effect_state"] = json!("confirmed_applied");
        observation["observation_kind"] = json!("provider_status_query");
        observation["source_kind"] = json!("provider");
        observation["capture_origin"] = json!("provider_native");
        observation["independence_classification"] = json!("qualified_independent");
        let report = validate_trace(&trace(vec![observation]));
        assert!(report.errors.iter().any(|error| error.contains("cannot claim qualified independent")));
    }

    #[test]
    fn missing_action_observation_is_reported_as_missing_not_no_effect() {
        let report = validate_trace(&trace(vec![event(0,"e0","task_created")]));
        assert_eq!(report.missing_expected_action_ids, vec!["action-1".to_owned()]);
        assert!(report.errors.is_empty(), "{:?}", report.errors);
    }

    #[test]
    fn duplicate_event_and_action_ids_are_rejected() {
        let mut first = event(0,"same","operation_proposed");
        first["operation_ref"] = json!("op-1");
        let mut second = event(1,"same","protected_effect_observation");
        second["action_id"] = json!("action-1");
        second["effect_state"] = json!("unknown");
        second["observation_kind"] = json!("owner_reconciliation");
        let mut third = event(2,"third","protected_effect_observation");
        third["action_id"] = json!("action-1");
        third["effect_state"] = json!("unknown");
        third["observation_kind"] = json!("owner_reconciliation");
        let report = validate_trace(&trace(vec![first,second,third]));
        assert!(report.errors.iter().any(|error| error.contains("duplicate event_id")));
        assert!(report.errors.iter().any(|error| error.contains("duplicate protected-effect observation")));
    }

    #[test]
    fn causal_references_must_point_to_prior_events() {
        let mut first = event(0, "e0", "task_created");
        first["causation_event_ref"] = json!("e1");
        let second = event(1, "e1", "operation_proposed");
        let report = validate_trace(&trace(vec![first, second]));
        assert!(report
            .errors
            .iter()
            .any(|error| error.contains("must refer to an earlier event")));
    }

    #[test]
    fn unknown_causal_references_are_rejected() {
        let mut first = event(0, "e0", "task_created");
        first["causation_event_ref"] = json!("missing-event");
        let report = validate_trace(&trace(vec![first]));
        assert!(report
            .errors
            .iter()
            .any(|error| error.contains("unresolved causation_event_ref")));
    }

    #[test]
    fn missing_expected_action_does_not_pass_full_completion_reconciliation() {
        let mut observation = event(0,"e0","protected_effect_observation");
        observation["action_id"] = json!("action-1");
        observation["effect_state"] = json!("confirmed_applied");
        observation["observation_kind"] = json!("independent_observation");
        observation["source_kind"] = json!("independent_observer");
        let mut reconciliation = event(1,"e1","reconciliation");
        reconciliation["operation_ref"] = json!("op-1");
        reconciliation["operation_digest"] = json!("digest-1");
        reconciliation["effect_state"] = json!("confirmed_applied");
        reconciliation["expected_action_ids"] = json!(["action-1"]);
        reconciliation["observed_action_ids"] = json!([]);
        let report = validate_trace(&trace(vec![observation,reconciliation]));
        assert!(report.errors.iter().any(|error| error.contains("coverage for every expected action")));
    }

    #[test]
    fn sequence_numbers_must_be_contiguous_and_zero_based() {
        let report = validate_trace(&trace(vec![event(1,"e0","task_created")]));
        assert!(report.errors.iter().any(|error| error.contains("contiguous and zero-based")));
    }

    #[test]
    fn externally_pinned_trace_bytes_are_sha256_identifiable() {
        let bytes = br#"{"trace_version":"0.1"}"#;
        assert!(is_sha256(&sha256_bytes(bytes)));
        assert!(!is_sha256("sha256:fixture"));
    }
}

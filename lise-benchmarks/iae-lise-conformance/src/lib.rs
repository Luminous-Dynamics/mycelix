#![forbid(unsafe_code)]

//! Cross-owner adapter conformance for the IAE v0.2 -> LISE finality boundary.
//!
//! IAE resolves source claims, time and owner context. LISE evaluates the
//! narrow pre-effect correspondence relation. This crate checks the adapter
//! join; it is not an authorizer, provider client, source authenticator, or
//! production execution path.

use ipb_finality_reference::{
    ApprovalBinding, ApprovalState, AuthorityState, ConstitutionRef, CurrentProtectedState,
    DenialCode, FinalityDecision, FinalityInput, OperationBinding, PriorEffectState, ResourceState,
    RetrySafety, evaluate_finality,
};
use serde_json::{Value, json};
use std::collections::HashSet;

pub const ADAPTER_VERSION: &str = "iae-lise-conformance-v0.1";
pub const SUITE_ID: &str = "IAE-LISE-TEMPORAL-CONFORMANCE-v0.1";

#[derive(Clone, Copy)]
struct Scenario {
    id: &'static str,
    action_at_key: &'static str,
    known_at_key: &'static str,
    expected: &'static str,
}

const SCENARIOS: [Scenario; 4] = [
    Scenario {
        id: "PRE_DISPATCH_POSITIVE_CONTROL",
        action_at_key: "known_at_before_dispatch",
        known_at_key: "known_at_before_dispatch",
        expected: "Permit",
    },
    Scenario {
        id: "PRE_DISPATCH_TIME_AMBIGUOUS",
        action_at_key: "action_at_before_dispatch",
        known_at_key: "known_at_before_dispatch",
        expected: "Hold(PriorOutcomeUnknown)",
    },
    Scenario {
        id: "POST_TIMEOUT",
        action_at_key: "timeout_occurred_at",
        known_at_key: "timeout_observed_at",
        expected: "Hold(PriorOutcomeUnknown)",
    },
    Scenario {
        id: "REATTEMPT_AFTER_REVOCATION",
        action_at_key: "reattempt_action_at",
        known_at_key: "reattempt_known_at",
        expected: "Deny(AuthorityRevoked)",
    },
];

fn member<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a Value, String> {
    value
        .get(key)
        .ok_or_else(|| format!("{context}: missing '{key}'"))
}

fn object<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a Value, String> {
    let v = member(value, key, context)?;
    if v.is_object() {
        Ok(v)
    } else {
        Err(format!("{context}: '{key}' must be an object"))
    }
}

fn array<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a Vec<Value>, String> {
    member(value, key, context)?
        .as_array()
        .ok_or_else(|| format!("{context}: '{key}' must be an array"))
}

fn required_str<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a str, String> {
    member(value, key, context)?
        .as_str()
        .filter(|s| !s.trim().is_empty())
        .ok_or_else(|| format!("{context}: '{key}' must be a nonempty string"))
}

fn optional_string(value: &Value, key: &str, context: &str) -> Result<Option<String>, String> {
    match member(value, key, context)? {
        Value::Null => Ok(None),
        Value::String(s) if !s.trim().is_empty() => Ok(Some(s.clone())),
        _ => Err(format!(
            "{context}: '{key}' must be null or a nonempty string"
        )),
    }
}

fn optional_u64(value: &Value, key: &str, context: &str) -> Result<Option<u64>, String> {
    match member(value, key, context)? {
        Value::Null => Ok(None),
        Value::Number(n) => n
            .as_u64()
            .map(Some)
            .ok_or_else(|| format!("{context}: '{key}' must be a nonnegative integer or null")),
        _ => Err(format!("{context}: '{key}' must be an integer or null")),
    }
}

fn is_fixed_utc_timestamp(s: &str) -> bool {
    let b = s.as_bytes();
    b.len() == 20
        && b[4] == b'-'
        && b[7] == b'-'
        && b[10] == b'T'
        && b[13] == b':'
        && b[16] == b':'
        && b[19] == b'Z'
        && b.iter()
            .enumerate()
            .all(|(i, c)| matches!(i, 4 | 7 | 10 | 13 | 16 | 19) || c.is_ascii_digit())
        && {
            let year = s[0..4].parse::<u32>().unwrap_or(0);
            let month = s[5..7].parse::<u32>().unwrap_or(0);
            let day = s[8..10].parse::<u32>().unwrap_or(0);
            let hour = s[11..13].parse::<u32>().unwrap_or(99);
            let minute = s[14..16].parse::<u32>().unwrap_or(99);
            let second = s[17..19].parse::<u32>().unwrap_or(99);
            let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
            let days = match month {
                1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
                4 | 6 | 9 | 11 => 30,
                2 if leap => 29,
                2 => 28,
                _ => 0,
            };
            year > 0 && day >= 1 && day <= days && hour <= 23 && minute <= 59 && second <= 59
        }
}

fn timestamp(value: &Value, key: &str, context: &str) -> Result<String, String> {
    let s = required_str(value, key, context)?;
    if !is_fixed_utc_timestamp(s) {
        return Err(format!(
            "{context}: '{key}' must be fixed-width UTC YYYY-MM-DDTHH:MM:SSZ"
        ));
    }
    Ok(s.to_owned())
}

fn parse_operation(value: &Value, context: &str) -> Result<OperationBinding, String> {
    Ok(OperationBinding {
        logical_operation_id: required_str(value, "logical_operation_id", context)?.to_owned(),
        canonical_operation_digest: required_str(value, "canonical_operation_digest", context)?
            .to_owned(),
        action_digest: required_str(value, "action_digest", context)?.to_owned(),
        target_ref: required_str(value, "target_ref", context)?.to_owned(),
        destination_or_audience: optional_string(value, "destination_or_audience", context)?,
        amount_minor: optional_u64(value, "amount_minor", context)?,
        currency: optional_string(value, "currency", context)?,
        purpose: optional_string(value, "purpose", context)?,
        resource_revision: optional_string(value, "resource_revision", context)?,
        idempotency_ref: optional_string(value, "idempotency_ref", context)?,
    })
}

fn parse_approval_state(s: &str) -> ApprovalState {
    match s {
        "authorized" => ApprovalState::Authorized,
        "denied" => ApprovalState::Denied,
        "revoked" => ApprovalState::Revoked,
        "historical" => ApprovalState::Historical,
        "pending" => ApprovalState::Pending,
        "unknown" => ApprovalState::Unknown,
        // IAE intent-baseline "accepted" is NOT transaction approval.
        _ => ApprovalState::Unknown,
    }
}

fn parse_authority_status(s: &str) -> AuthorityState {
    match s {
        "active" | "current" => AuthorityState::Current,
        "revoked" => AuthorityState::Revoked,
        "expired" => AuthorityState::Expired,
        "not_yet_valid" => AuthorityState::NotYetValid,
        _ => AuthorityState::Unknown,
    }
}

fn parse_resource_state(s: &str) -> ResourceState {
    match s {
        "matches" => ResourceState::Matches,
        "mismatch" => ResourceState::Mismatch,
        "not_required" => ResourceState::NotRequired,
        _ => ResourceState::Unknown,
    }
}

fn parse_retry_safety(s: &str) -> RetrySafety {
    match s {
        "same_operation_same_idempotency_key_qualified" => {
            RetrySafety::SameOperationSameIdempotencyKeyQualified
        }
        "unsupported" => RetrySafety::Unsupported,
        _ => RetrySafety::Unknown,
    }
}

fn resolve_authority(
    fixture: &Value,
    operation: &Value,
    action_at: &str,
    known_at: &str,
    institution_ref: &str,
) -> Result<AuthorityState, String> {
    let authority = object(fixture, "authority", "fixture")?;
    if required_str(authority, "verification_state", "authority")? != "verified" {
        return Ok(AuthorityState::Unknown);
    }

    let subject = required_str(authority, "subject_ref", "authority")?;
    if required_str(authority, "entity_ref", "authority")? != institution_ref {
        return Ok(AuthorityState::Unknown);
    }
    let actor = required_str(operation, "principal_ref", "approved operation")?;
    let action = required_str(operation, "action", "approved operation")?;
    let purpose = required_str(operation, "purpose", "approved operation")?;
    let target = required_str(operation, "target_ref", "approved operation")?;

    if subject != actor
        || !array(authority, "action_scope", "authority")?
            .iter()
            .any(|v| v.as_str() == Some(action))
        || !array(authority, "purpose_scope", "authority")?
            .iter()
            .any(|v| v.as_str() == Some(purpose))
        || !array(authority, "asset_scope", "authority")?
            .iter()
            .any(|v| v.as_str() == Some(target))
    {
        return Ok(AuthorityState::Unknown);
    }

    let valid_from = timestamp(authority, "valid_from", "authority")?;
    let valid_through = timestamp(authority, "valid_through", "authority")?;
    if action_at < valid_from.as_str() {
        return Ok(AuthorityState::NotYetValid);
    }
    if action_at > valid_through.as_str() {
        return Ok(AuthorityState::Expired);
    }

    let events = array(authority, "status_events", "authority")?;
    let mut visible: Vec<(String, String, String)> = Vec::new();
    for event in events {
        let effective = timestamp(event, "effective_at", "authority status event")?;
        let observed = timestamp(event, "observed_at", "authority status event")?;
        if effective.as_str() <= action_at && observed.as_str() <= known_at {
            visible.push((
                effective,
                observed,
                required_str(event, "status", "authority status event")?.to_owned(),
            ));
        }
    }
    if visible.is_empty() {
        return Ok(AuthorityState::Unknown);
    }
    visible.sort_by(|a, b| (&a.0, &a.1).cmp(&(&b.0, &b.1)));
    let latest = visible.last().expect("visible is nonempty");
    let same_rank_conflict = visible
        .iter()
        .any(|row| row.0 == latest.0 && row.1 == latest.1 && row.2 != latest.2);
    if same_rank_conflict {
        return Ok(AuthorityState::Unknown);
    }
    Ok(parse_authority_status(&latest.2))
}

fn history_receipt_covers_query(
    fixture: &Value,
    operation: &Value,
    action_at: &str,
    known_at: &str,
) -> bool {
    // This receipt closes only the synthetic fixture's explicitly enumerated
    // event universe. Production must provide an owner-authenticated receipt.
    if fixture.get("synthetic").and_then(Value::as_bool) != Some(true) {
        return false;
    }
    let Some(receipt) = fixture.get("operation_history_completeness") else {
        return false;
    };
    let fields_match = [
        ("operation_ref", "logical_operation_id"),
        ("idempotency_ref", "idempotency_ref"),
        ("canonical_operation_digest", "canonical_operation_digest"),
        ("action_digest", "action_digest"),
        ("resource_ref", "target_ref"),
    ]
    .iter()
    .all(|(receipt_key, operation_key)| {
        receipt.get(*receipt_key) == operation.get(*operation_key)
    });
    if !fields_match
        || receipt.get("source_kind").and_then(Value::as_str)
            != Some("synthetic_closed_world_fixture")
        || receipt.get("completeness_state").and_then(Value::as_str)
            != Some("complete_for_scope")
        || receipt.get("pagination_complete").and_then(Value::as_bool) != Some(true)
        || receipt.get("retention_complete").and_then(Value::as_bool) != Some(true)
        || receipt.get("gap_detected").and_then(Value::as_bool) != Some(false)
        || receipt.get("query_scope_exact").and_then(Value::as_bool) != Some(true)
    {
        return false;
    }
    let Ok(coverage_from) = timestamp(receipt, "coverage_from", "history completeness receipt") else {
        return false;
    };
    let Ok(watermark) = timestamp(
        receipt,
        "watermark_occurred_at",
        "history completeness receipt",
    ) else {
        return false;
    };
    let Ok(observed_at) = timestamp(
        receipt,
        "observed_at",
        "history completeness receipt",
    ) else {
        return false;
    };
    coverage_from.as_str() <= action_at
        && watermark.as_str() >= action_at
        && observed_at.as_str() <= known_at
}

fn resolve_prior_effect(
    fixture: &Value,
    operation: &Value,
    action_at: &str,
    known_at: &str,
) -> Result<PriorEffectState, String> {
    let operation_id = required_str(operation, "logical_operation_id", "approved operation")?;
    let idempotency = required_str(operation, "idempotency_ref", "approved operation")?;
    let digest = required_str(
        operation,
        "canonical_operation_digest",
        "approved operation",
    )?;
    let action_digest = required_str(operation, "action_digest", "approved operation")?;
    let history = array(fixture, "operation_history", "fixture")?;

    let mut matching_history = false;
    let mut visible: Vec<(String, String, Value)> = Vec::new();
    for event in history {
        if event.get("operation_ref").and_then(Value::as_str) != Some(operation_id) {
            continue;
        }
        matching_history = true;
        let occurred = timestamp(event, "occurred_at", "operation event")?;
        let observed = timestamp(event, "observed_at", "operation event")?;
        // Events after the action cutoff cannot be prior effects. But an
        // event that may have occurred by action time and was not yet observed
        // at the knowledge cutoff makes prior-effect state unknown.
        if occurred.as_str() > action_at {
            continue;
        }
        if observed.as_str() > known_at {
            return Ok(PriorEffectState::OutcomeUnknownAfterDispatch);
        }
        let event_key = required_str(event, "idempotency_ref", "operation event")?;
        let event_digest = required_str(event, "canonical_operation_digest", "operation event")?;
        let event_action_digest = required_str(event, "action_digest", "operation event")?;
        if event_key != idempotency
            || event_digest != digest
            || event_action_digest != action_digest
        {
            return Ok(PriorEffectState::OutcomeUnknownAfterDispatch);
        }
        visible.push((occurred, observed, event.clone()));
    }

    if visible.is_empty() {
        let complete = history_receipt_covers_query(fixture, operation, action_at, known_at);
        if complete {
            return Ok(PriorEffectState::NeverAttempted);
        }
        return Ok(PriorEffectState::OutcomeUnknownAfterDispatch);
    }
    if !matching_history {
        return Ok(PriorEffectState::OutcomeUnknownAfterDispatch);
    }

    visible.sort_by(|a, b| (&a.0, &a.1).cmp(&(&b.0, &b.1)));
    let latest = visible.last().expect("visible is nonempty");
    let same_rank_conflict = visible.iter().any(|row| {
        row.0 == latest.0 && row.1 == latest.1 && row.2.get("outcome") != latest.2.get("outcome")
    });
    if same_rank_conflict {
        return Ok(PriorEffectState::OutcomeUnknownAfterDispatch);
    }
    let outcome = required_str(&latest.2, "outcome", "operation event")?;
    Ok(match outcome {
        "not_applied" | "confirmed_not_applied" => PriorEffectState::ConfirmedNotApplied,
        "applied" | "confirmed_applied" => PriorEffectState::ConfirmedApplied,
        "partially_applied" => PriorEffectState::PartiallyApplied,
        // A submit acknowledgement and a timeout are not proof of no effect.
        _ => PriorEffectState::OutcomeUnknownAfterDispatch,
    })
}

fn build_input(
    fixture: &Value,
    action_at: &str,
    known_at: &str,
) -> Result<(FinalityInput, AuthorityState, PriorEffectState), String> {
    let finality = object(fixture, "operation_finality", "fixture")?;
    let approved = object(finality, "approved_operation", "operation_finality")?;
    let pending = object(finality, "pending_operation", "operation_finality")?;
    let approval_operation = parse_operation(approved, "approved operation")?;
    let actual_operation = parse_operation(pending, "pending operation")?;

    let aux_equal = required_str(approved, "action", "approved operation")?
        == required_str(pending, "action", "pending operation")?
        && required_str(approved, "principal_ref", "approved operation")?
            == required_str(pending, "principal_ref", "pending operation")?;
    let approval_actor = required_str(approved, "principal_ref", "approved operation")?;
    let action = required_str(approved, "action", "approved operation")?;
    let purpose = required_str(approved, "purpose", "approved operation")?;
    let target = required_str(approved, "target_ref", "approved operation")?;

    let constitution = object(finality, "constitution", "operation_finality")?;
    let constitution_ref = ConstitutionRef {
        institution_ref: required_str(constitution, "institution_ref", "constitution")?.to_owned(),
        epoch: member(constitution, "epoch", "constitution")?
            .as_u64()
            .ok_or_else(|| "constitution: 'epoch' must be an unsigned integer".to_owned())?,
        constitution_digest: required_str(constitution, "constitution_digest", "constitution")?
            .to_owned(),
    };

    let approval_state = parse_approval_state(required_str(
        finality,
        "approval_state",
        "operation_finality",
    )?);
    let approval_policy = required_str(finality, "policy_revision", "operation_finality")?;
    let current_policy = fixture
        .get("current_policy_ref")
        .and_then(Value::as_str)
        .unwrap_or(required_str(fixture, "policy_ref", "fixture")?);
    let current_constitution = fixture
        .get("current_constitution")
        .filter(|v| v.is_object())
        .unwrap_or(constitution);
    let current_constitution_ref = ConstitutionRef {
        institution_ref: required_str(
            current_constitution,
            "institution_ref",
            "current constitution",
        )?
        .to_owned(),
        epoch: member(current_constitution, "epoch", "current constitution")?
            .as_u64()
            .ok_or_else(|| {
                "current constitution: 'epoch' must be an unsigned integer".to_owned()
            })?,
        constitution_digest: required_str(
            current_constitution,
            "constitution_digest",
            "current constitution",
        )?
        .to_owned(),
    };

    let authority_from_history = resolve_authority(
        fixture,
        approved,
        action_at,
        known_at,
        &constitution_ref.institution_ref,
    )?;
    let authority = if aux_equal
        && object(fixture, "authority", "fixture")
            .and_then(|a| required_str(a, "subject_ref", "authority"))
            .map(|s| s == approval_actor)
            .unwrap_or(false)
        && array(
            object(fixture, "authority", "fixture")?,
            "action_scope",
            "authority",
        )?
        .iter()
        .any(|v| v.as_str() == Some(action))
        && array(
            object(fixture, "authority", "fixture")?,
            "purpose_scope",
            "authority",
        )?
        .iter()
        .any(|v| v.as_str() == Some(purpose))
        && array(
            object(fixture, "authority", "fixture")?,
            "asset_scope",
            "authority",
        )?
        .iter()
        .any(|v| v.as_str() == Some(target))
    {
        authority_from_history
    } else {
        AuthorityState::Unknown
    };

    let resource = parse_resource_state(required_str(
        finality,
        "resource_state_before_dispatch",
        "operation_finality",
    )?);
    let prior_effect = resolve_prior_effect(fixture, approved, action_at, known_at)?;
    let retry_safety = parse_retry_safety(required_str(
        finality,
        "retry_safety_after_timeout",
        "operation_finality",
    )?);

    let input = FinalityInput {
        approval: ApprovalBinding {
            operation: approval_operation,
            constitution: constitution_ref.clone(),
            policy_revision: approval_policy.to_owned(),
            state: approval_state,
        },
        actual_operation,
        current: CurrentProtectedState {
            constitution: current_constitution_ref,
            policy_revision: current_policy.to_owned(),
        },
        authority,
        resource,
        prior_effect,
        retry_safety,
    };
    Ok((input, authority, prior_effect))
}

fn decision_label(decision: FinalityDecision) -> String {
    format!("{decision:?}")
}

fn write_trace_violations(fixture: &Value, submission: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    let finality = match object(fixture, "operation_finality", "fixture") {
        Ok(v) => v,
        Err(e) => return vec![e],
    };
    let approved = match object(finality, "approved_operation", "operation_finality") {
        Ok(v) => v,
        Err(e) => return vec![e],
    };
    let execution = match object(fixture, "execution_control", "fixture") {
        Ok(v) => v,
        Err(e) => return vec![e],
    };
    let boundaries = match array(execution, "authorized_write_boundary", "execution_control") {
        Ok(v) => v,
        Err(e) => return vec![e],
    };
    let writes = match array(submission, "write_trace", "submission") {
        Ok(v) => v,
        Err(e) => return vec![e],
    };
    if writes.is_empty() {
        return vec!["write trace is empty despite a recorded submit attempt".to_owned()];
    }

    let mut observed_event_refs = HashSet::new();
    for write in writes {
        let write_fields = match array(write, "fields", "write trace") {
            Ok(v) => v.iter().filter_map(Value::as_str).collect::<Vec<_>>(),
            Err(e) => {
                errors.push(e);
                continue;
            }
        };
        let Some(boundary) = boundaries.iter().find(|b| {
            b.get("operation_ref") == write.get("operation_ref")
                && b.get("canonical_operation_digest") == write.get("canonical_operation_digest")
                && b.get("action_digest") == write.get("action_digest")
                && b.get("idempotency_ref") == write.get("idempotency_ref")
                && b.get("action") == write.get("action")
                && b.get("purpose") == write.get("purpose")
                && b.get("asset_ref") == write.get("asset_ref")
                && b.get("actor_ref") == write.get("actor_ref")
                && b.get("authorization_ref") == write.get("authorization_ref")
                && b.get("possible_external_side_effect")
                    == write.get("possible_external_side_effect")
        }) else {
            errors.push(
                "write trace does not match an owner-supplied exact write boundary".to_owned(),
            );
            continue;
        };
        let allowed = match array(boundary, "fields", "write boundary") {
            Ok(v) => v.iter().filter_map(Value::as_str).collect::<Vec<_>>(),
            Err(e) => {
                errors.push(e);
                continue;
            }
        };
        if write_fields.is_empty() || write_fields.iter().any(|field| !allowed.contains(field)) {
            errors.push("write trace expands beyond the authorized field set".to_owned());
        }
        if required_str(write, "action", "write trace").ok()
            != required_str(approved, "action", "approved operation").ok()
            || required_str(write, "purpose", "write trace").ok()
                != required_str(approved, "purpose", "approved operation").ok()
            || required_str(write, "asset_ref", "write trace").ok()
                != required_str(approved, "target_ref", "approved operation").ok()
            || required_str(write, "actor_ref", "write trace").ok()
                != required_str(approved, "principal_ref", "approved operation").ok()
        {
            errors.push(
                "write trace action/principal/purpose/asset differs from the approved operation"
                    .to_owned(),
            );
        }
        let authority_ref = object(fixture, "authority", "fixture")
            .and_then(|a| required_str(a, "authorization_ref", "authority"))
            .ok();
        if required_str(write, "authorization_ref", "write trace").ok() != authority_ref {
            errors.push(
                "write trace authorization reference differs from authority owner reference"
                    .to_owned(),
            );
        }
        let event_ref = write.get("event_ref").and_then(Value::as_str);
        let event_exists = array(fixture, "operation_history", "fixture")
            .map(|events| {
                events.iter().any(|event| {
                    event.get("event_ref").and_then(Value::as_str) == event_ref
                        && event.get("event_type").and_then(Value::as_str) == Some("submit_attempt")
                })
            })
            .unwrap_or(false);
        if !event_exists {
            errors.push(
                "write trace is not bound to the source operation-history submit event".to_owned(),
            );
        }
        if let Some(event_ref) = event_ref {
            if !observed_event_refs.insert(event_ref.to_owned()) {
                errors.push("duplicate write-trace event reference".to_owned());
            }
        }
    }
    let expected_event_refs: HashSet<String> = array(fixture, "operation_history", "fixture")
        .map(|events| {
            events
                .iter()
                .filter(|event| {
                    event.get("event_type").and_then(Value::as_str) == Some("submit_attempt")
                        && event
                            .get("possible_external_side_effect")
                            .and_then(Value::as_bool)
                            == Some(true)
                })
                .filter_map(|event| {
                    event
                        .get("event_ref")
                        .and_then(Value::as_str)
                        .map(str::to_owned)
                })
                .collect()
        })
        .unwrap_or_default();
    if expected_event_refs != observed_event_refs {
        errors.push(
            "write trace does not cover every possible external submit event exactly once"
                .to_owned(),
        );
    }
    errors
}

fn finality_output_violations(fixture: &Value, submission: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    let Some(result) = submission.get("operation_finality_result") else {
        return vec!["submission is missing operation_finality_result".to_owned()];
    };
    let Some(execution) = submission.get("execution_control") else {
        return vec!["submission is missing execution_control".to_owned()];
    };
    let finality = match object(fixture, "operation_finality", "fixture") {
        Ok(v) => v,
        Err(e) => return vec![e],
    };
    let approved = match object(finality, "approved_operation", "operation_finality") {
        Ok(v) => v,
        Err(e) => return vec![e],
    };

    if result.get("operation_ref").and_then(Value::as_str)
        != required_str(approved, "logical_operation_id", "approved operation").ok()
    {
        errors
            .push("submission operation reference differs from IAE-approved operation".to_owned());
    }
    if result
        .get("canonical_operation_digest")
        .and_then(Value::as_str)
        != required_str(approved, "canonical_operation_digest", "approved operation").ok()
    {
        errors.push(
            "submission canonical operation digest differs from IAE-approved operation".to_owned(),
        );
    }
    if result.get("approval_ref").and_then(Value::as_str)
        != required_str(finality, "approval_ref", "operation_finality").ok()
    {
        errors.push("submission approval reference differs from IAE operation approval".to_owned());
    }
    if result.get("action_digest").and_then(Value::as_str)
        != required_str(approved, "action_digest", "approved operation").ok()
    {
        errors.push("submission action digest differs from IAE-approved operation".to_owned());
    }
    if result.get("idempotency_ref").and_then(Value::as_str)
        != required_str(approved, "idempotency_ref", "approved operation").ok()
    {
        errors.push(
            "submission idempotency reference differs from IAE-approved operation".to_owned(),
        );
    }

    if result.get("disposition").and_then(Value::as_str) != Some("HOLD_FOR_RECONCILIATION")
        || result.get("retry_count").and_then(Value::as_u64) != Some(0)
        || result.get("reattempt_permitted").and_then(Value::as_bool) != Some(false)
        || result
            .get("current_authority_state")
            .and_then(Value::as_str)
            != Some("revoked")
        || result.get("reattempt_disposition").and_then(Value::as_str) != Some("DENY")
    {
        errors.push(
            "submission finality/retry state does not match the vertical-slice contract".to_owned(),
        );
    }
    if result.get("reattempt_reason_code").and_then(Value::as_str)
        != Some("authority.revoked_before_action")
    {
        errors
            .push("submission does not preserve the authority-revocation denial reason".to_owned());
    }

    if execution.get("state").and_then(Value::as_str) != Some("UNKNOWN")
        || execution.get("disposition").and_then(Value::as_str) != Some("HOLD_FOR_RECONCILIATION")
        || execution.get("operation_ref").and_then(Value::as_str)
            != required_str(approved, "logical_operation_id", "approved operation").ok()
        || execution.get("idempotency_ref").and_then(Value::as_str)
            != required_str(approved, "idempotency_ref", "approved operation").ok()
        || execution.get("retry_count").and_then(Value::as_u64) != Some(0)
        || execution
            .get("reattempt_permitted")
            .and_then(Value::as_bool)
            != Some(false)
        || execution
            .get("reconciliation_required")
            .and_then(Value::as_bool)
            != Some(true)
    {
        errors.push(
            "submission execution control does not preserve UNKNOWN, retry=0 and reconciliation"
                .to_owned(),
        );
    }
    if fixture
        .get("execution_control")
        .and_then(|v| v.get("mutation_count"))
        .and_then(Value::as_u64)
        != execution.get("mutation_count").and_then(Value::as_u64)
    {
        errors.push("fixture and submission mutation counts differ".to_owned());
    }
    errors
}

fn synthetic_boundary_blockers(fixture: &Value, submission: &Value) -> Vec<String> {
    let mut blockers = Vec::new();
    for (label, value) in [("fixture", fixture), ("submission", submission)] {
        let Some(environment) = value.get("environment") else {
            blockers.push(format!("{label} environment metadata missing"));
            continue;
        };
        if environment.get("mode").and_then(Value::as_str) != Some("synthetic_fixture_only")
            || environment
                .get("external_io_enabled")
                .and_then(Value::as_bool)
                != Some(false)
            || environment.get("network_enabled").and_then(Value::as_bool) != Some(false)
            || environment
                .get("mutations_enabled")
                .and_then(Value::as_bool)
                != Some(false)
        {
            blockers.push(format!(
                "{label} does not prove fixture-only/no-I/O execution mode"
            ));
        }
    }
    let approved = fixture
        .get("operation_finality")
        .and_then(|v| v.get("approved_operation"));
    for (field, label) in [
        (
            "canonical_digest_verification",
            "canonical operation digest",
        ),
        ("action_digest_verification", "action digest"),
    ] {
        if approved.and_then(|v| v.get(field)).and_then(Value::as_str) != Some("verified") {
            blockers.push(format!(
                "{label} is not cryptographically verified in this synthetic fixture"
            ));
        }
    }
    blockers.push(
        "synthetic fixture: reference Permit cannot authorize a live external mutation".to_owned(),
    );
    blockers
}

fn evaluate_one(
    fixture: &Value,
    scenario: Scenario,
) -> Result<
    (
        FinalityDecision,
        AuthorityState,
        PriorEffectState,
        String,
        String,
    ),
    String,
> {
    let finality = object(fixture, "operation_finality", "fixture")?;
    let action_at;
    let known_at;
    if scenario.id == "POST_TIMEOUT" {
        let timeout = array(fixture, "operation_history", "fixture")?
            .iter()
            .find(|event| event.get("event_type").and_then(Value::as_str) == Some("timeout"))
            .ok_or_else(|| "operation history has no timeout event".to_owned())?;
        action_at = timestamp(timeout, "occurred_at", "timeout event")?;
        known_at = timestamp(timeout, "observed_at", "timeout event")?;
    } else {
        action_at = timestamp(finality, scenario.action_at_key, "operation_finality")?;
        known_at = timestamp(finality, scenario.known_at_key, "operation_finality")?;
    }
    let (input, authority, prior) = build_input(fixture, &action_at, &known_at)?;
    Ok((
        evaluate_finality(&input),
        authority,
        prior,
        action_at,
        known_at,
    ))
}

/// Recompute temporal finality decisions and cross-check the submitted effect trace.
pub fn evaluate_vertical_slice(fixture: &Value, submission: &Value) -> Value {
    let mut cases = Vec::new();
    for scenario in SCENARIOS {
        match evaluate_one(fixture, scenario) {
            Ok((observed, authority, prior, action_at, known_at)) => {
                let observed_label = decision_label(observed);
                cases.push(json!({
                    "scenario_id": scenario.id,
                    "action_at": action_at,
                    "known_at": known_at,
                    "authority_state": format!("{authority:?}"),
                    "prior_effect_state": format!("{prior:?}"),
                    "expected": scenario.expected,
                    "observed": observed_label.clone(),
                    "conformant": observed_label == scenario.expected
                }));
            }
            Err(error) => cases.push(json!({
                "scenario_id": scenario.id,
                "expected": scenario.expected,
                "observed": "INVALID_INPUT",
                "conformant": false,
                "error": error
            })),
        }
    }

    let mut validation_errors = write_trace_violations(fixture, submission);
    validation_errors.extend(finality_output_violations(fixture, submission));
    let blockers = synthetic_boundary_blockers(fixture, submission);
    let case_count = cases.len();
    let conformant_cases = cases
        .iter()
        .filter(|case| case.get("conformant").and_then(Value::as_bool) == Some(true))
        .count();
    let conformant = conformant_cases == case_count && validation_errors.is_empty();

    json!({
        "suite_id": SUITE_ID,
        "adapter_version": ADAPTER_VERSION,
        "case_count": case_count,
        "conformant_case_count": conformant_cases,
        "cases": cases,
        "write_trace_conformant": validation_errors.is_empty(),
        "validation_errors": validation_errors,
        "dispatch_eligible": false,
        "dispatch_blockers": blockers,
        "conformant": conformant,
        "claim_ceiling": "Synthetic adapter conformance only. Reference decisions do not authenticate sources, operation digests, authority evidence, provider state or external effects."
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn inputs() -> (Value, Value) {
        let fixture: Value =
            serde_json::from_str(include_str!("../fixtures/iae-v0.2-fixture.json"))
                .expect("fixture JSON");
        let submission: Value = serde_json::from_str(include_str!(
            "../fixtures/iae-v0.2-reference-submission.json"
        ))
        .expect("submission JSON");
        (fixture, submission)
    }

    fn case<'a>(report: &'a Value, id: &str) -> &'a Value {
        report["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|case| case["scenario_id"] == id)
            .expect("scenario exists")
    }

    #[test]
    fn recomputes_pre_dispatch_timeout_and_revoked_reattempt() {
        let (fixture, submission) = inputs();
        let report = evaluate_vertical_slice(&fixture, &submission);
        assert_eq!(
            case(&report, "PRE_DISPATCH_POSITIVE_CONTROL")["observed"],
            "Permit"
        );
        assert_eq!(
            case(&report, "PRE_DISPATCH_TIME_AMBIGUOUS")["observed"],
            "Hold(PriorOutcomeUnknown)"
        );
        assert_eq!(
            case(&report, "POST_TIMEOUT")["observed"],
            "Hold(PriorOutcomeUnknown)"
        );
        assert_eq!(
            case(&report, "REATTEMPT_AFTER_REVOCATION")["observed"],
            "Deny(AuthorityRevoked)"
        );
        assert_eq!(report["conformant"], true);
        assert_eq!(report["dispatch_eligible"], false);
    }

    #[test]
    fn intent_baseline_acceptance_cannot_authorize_a_transaction() {
        let (mut fixture, submission) = inputs();
        fixture["operation_finality"]["approval_state"] = json!("accepted");
        let report = evaluate_vertical_slice(&fixture, &submission);
        assert_eq!(
            case(&report, "PRE_DISPATCH_POSITIVE_CONTROL")["observed"],
            "Hold(ApprovalUnknown)"
        );
        assert_eq!(report["conformant"], false);
    }

    #[test]
    fn exact_operation_substitution_is_denied() {
        let (mut fixture, submission) = inputs();
        fixture["operation_finality"]["pending_operation"]["idempotency_ref"] =
            json!("IDEM-OTHER-OPERATION");
        let report = evaluate_vertical_slice(&fixture, &submission);
        assert_eq!(
            case(&report, "PRE_DISPATCH_POSITIVE_CONTROL")["observed"],
            "Deny(OperationMismatch)"
        );
        assert_eq!(report["conformant"], false);
    }

    #[test]
    fn action_or_principal_substitution_fails_closed() {
        let (mut fixture, submission) = inputs();
        fixture["operation_finality"]["pending_operation"]["action"] = json!("transfer_title");
        let report = evaluate_vertical_slice(&fixture, &submission);
        assert_eq!(
            case(&report, "PRE_DISPATCH_POSITIVE_CONTROL")["observed"],
            "Hold(AuthorityUnknown)"
        );
        assert_eq!(report["conformant"], false);

        let (mut fixture, submission) = inputs();
        fixture["operation_finality"]["pending_operation"]["principal_ref"] = json!("ACTOR-OTHER");
        let report = evaluate_vertical_slice(&fixture, &submission);
        assert_eq!(
            case(&report, "PRE_DISPATCH_POSITIVE_CONTROL")["observed"],
            "Hold(AuthorityUnknown)"
        );
    }

    #[test]
    fn authority_revocation_is_bitemporal() {
        let (fixture, _) = inputs();
        let finality = &fixture["operation_finality"];
        let approved = &finality["approved_operation"];
        let action_at = "2026-02-06T10:00:00Z";
        let before_observation = resolve_authority(
            &fixture,
            approved,
            action_at,
            "2026-02-01T23:59:59Z",
            "ENTITY-TRUST-01",
        )
        .expect("historical authority resolves");
        assert_eq!(before_observation, AuthorityState::Current);
        let after_observation = resolve_authority(
            &fixture,
            approved,
            action_at,
            "2026-02-06T10:00:00Z",
            "ENTITY-TRUST-01",
        )
        .expect("current authority resolves");
        assert_eq!(after_observation, AuthorityState::Revoked);
    }

    #[test]
    fn unknown_authority_status_holds_instead_of_permitting() {
        let (mut fixture, submission) = inputs();
        fixture["authority"]["verification_state"] = json!("unknown");
        let report = evaluate_vertical_slice(&fixture, &submission);
        assert_eq!(
            case(&report, "PRE_DISPATCH_POSITIVE_CONTROL")["observed"],
            "Hold(AuthorityUnknown)"
        );
    }

    #[test]
    fn policy_and_constitution_drift_are_denied() {
        let (mut fixture, _) = inputs();
        fixture["current_policy_ref"] = json!("POLICY-CHANGED");
        let (input, _, _) =
            build_input(&fixture, "2026-01-25T10:01:00Z", "2026-01-25T10:00:00Z").expect("input");
        assert_eq!(
            evaluate_finality(&input),
            FinalityDecision::Deny(DenialCode::PolicyRevisionMismatch)
        );

        let (mut fixture, _) = inputs();
        fixture["current_constitution"] = json!({
            "institution_ref": "ENTITY-TRUST-01",
            "epoch": 2,
            "constitution_digest": "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
        });
        let (input, _, _) =
            build_input(&fixture, "2026-01-25T10:01:00Z", "2026-01-25T10:00:00Z").expect("input");
        assert_eq!(
            evaluate_finality(&input),
            FinalityDecision::Deny(DenialCode::ConstitutionMismatch)
        );
    }

    #[test]
    fn write_trace_cannot_expand_owner_approved_fields() {
        let (fixture, mut submission) = inputs();
        submission["write_trace"][0]["fields"] =
            json!(["lease_submission_request", "title_transfer"]);
        let errors = write_trace_violations(&fixture, &submission);
        assert!(errors.iter().any(|error| error.contains("expands beyond")));
    }

    #[test]
    fn unknown_outcome_and_revoked_authority_remain_distinct() {
        let (fixture, submission) = inputs();
        let report = evaluate_vertical_slice(&fixture, &submission);
        assert_eq!(
            case(&report, "POST_TIMEOUT")["observed"],
            "Hold(PriorOutcomeUnknown)"
        );
        assert_eq!(
            case(&report, "REATTEMPT_AFTER_REVOCATION")["observed"],
            "Deny(AuthorityRevoked)"
        );
        assert_eq!(report["dispatch_eligible"], false);
    }

    #[test]
    fn authority_from_another_institution_is_not_accepted() {
        let (mut fixture, submission) = inputs();
        fixture["authority"]["entity_ref"] = json!("ENTITY-OTHER");
        let report = evaluate_vertical_slice(&fixture, &submission);
        assert_eq!(
            case(&report, "PRE_DISPATCH_POSITIVE_CONTROL")["observed"],
            "Hold(AuthorityUnknown)"
        );
    }

    #[test]
    fn an_unobserved_effect_before_action_time_remains_unknown() {
        let (mut fixture, _) = inputs();
        fixture["operation_history"][0]["occurred_at"] = json!("2026-01-25T10:00:59Z");
        fixture["operation_history"][0]["observed_at"] = json!("2026-01-25T10:02:00Z");
        let prior = resolve_prior_effect(
            &fixture,
            &fixture["operation_finality"]["approved_operation"],
            "2026-01-25T10:01:00Z",
            "2026-01-25T10:00:00Z",
        )
        .expect("historical effect resolution");
        assert_eq!(prior, PriorEffectState::OutcomeUnknownAfterDispatch);
    }

    #[test]
    fn future_mismatched_event_does_not_rewrite_prior_knowledge() {
        let (mut fixture, _) = inputs();
        fixture["operation_history"].as_array_mut().expect("history").push(json!({
            "event_ref": "OP-EVENT-FUTURE-OTHER",
            "operation_ref": "OP-LEASE-001",
            "idempotency_ref": "IDEM-OTHER",
            "canonical_operation_digest": "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff",
            "action_digest": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "event_type": "submit_attempt",
            "outcome": "submitted",
            "occurred_at": "2026-02-10T10:00:00Z",
            "observed_at": "2026-02-10T10:00:00Z"
        }));
        let prior = resolve_prior_effect(
            &fixture,
            &fixture["operation_finality"]["approved_operation"],
            "2026-01-25T10:00:00Z",
            "2026-01-25T10:00:00Z",
        )
        .expect("past view resolves");
        assert_eq!(prior, PriorEffectState::NeverAttempted);
    }

    #[test]
    fn malformed_calendar_timestamp_is_rejected() {
        assert!(!is_fixed_utc_timestamp("2026-02-30T10:00:00Z"));
        assert!(!is_fixed_utc_timestamp("2026-13-01T10:00:00Z"));
        assert!(is_fixed_utc_timestamp("2024-02-29T23:59:59Z"));
    }

    #[test]
    fn no_visible_event_requires_scoped_complete_history_receipt() {
        let (mut fixture, _) = inputs();
        let operation = &fixture["operation_finality"]["approved_operation"];
        assert_eq!(
            resolve_prior_effect(
                &fixture,
                operation,
                "2026-01-25T10:00:00Z",
                "2026-01-25T10:00:00Z",
            )
            .expect("complete fixture receipt"),
            PriorEffectState::NeverAttempted
        );

        fixture.as_object_mut().expect("fixture object")
            .remove("operation_history_completeness");
        assert_eq!(
            resolve_prior_effect(
                &fixture,
                &fixture["operation_finality"]["approved_operation"],
                "2026-01-25T10:00:00Z",
                "2026-01-25T10:00:00Z",
            )
            .expect("missing receipt remains a held state"),
            PriorEffectState::OutcomeUnknownAfterDispatch
        );
    }

    #[test]
    fn stale_or_incomplete_history_receipt_never_proves_no_attempt() {
        let (mut fixture, _) = inputs();
        fixture["operation_history_completeness"]["watermark_occurred_at"] =
            json!("2026-01-24T10:00:00Z");
        let operation = fixture["operation_finality"]["approved_operation"].clone();
        assert_eq!(
            resolve_prior_effect(
                &fixture,
                &operation,
                "2026-01-25T10:00:00Z",
                "2026-01-25T10:00:00Z",
            )
            .expect("stale watermark is readable"),
            PriorEffectState::OutcomeUnknownAfterDispatch
        );

        fixture["operation_history_completeness"]["watermark_occurred_at"] =
            json!("2026-01-25T10:00:00Z");
        fixture["operation_history_completeness"]["pagination_complete"] = json!(false);
        assert_eq!(
            resolve_prior_effect(
                &fixture,
                &operation,
                "2026-01-25T10:00:00Z",
                "2026-01-25T10:00:00Z",
            )
            .expect("incomplete pagination is readable"),
            PriorEffectState::OutcomeUnknownAfterDispatch
        );
    }

    #[test]
    fn mismatched_or_late_completeness_receipt_is_rejected() {
        let (mut fixture, _) = inputs();
        let operation = fixture["operation_finality"]["approved_operation"].clone();
        fixture["operation_history_completeness"]["idempotency_ref"] =
            json!("IDEM-OTHER");
        assert_eq!(
            resolve_prior_effect(
                &fixture,
                &operation,
                "2026-01-25T10:00:00Z",
                "2026-01-25T10:00:00Z",
            )
            .expect("mismatched scope remains a held state"),
            PriorEffectState::OutcomeUnknownAfterDispatch
        );

        fixture["operation_history_completeness"]["idempotency_ref"] = json!("IDEM-LEASE-001");
        fixture["operation_history_completeness"]["observed_at"] =
            json!("2026-01-25T10:01:00Z");
        assert_eq!(
            resolve_prior_effect(
                &fixture,
                &operation,
                "2026-01-25T10:00:00Z",
                "2026-01-25T10:00:00Z",
            )
            .expect("late receipt remains a held state"),
            PriorEffectState::OutcomeUnknownAfterDispatch
        );
    }

    #[test]
    fn missing_history_does_not_prove_never_attempted() {
        let (mut fixture, submission) = inputs();
        fixture["authority"]["status_events"] = json!([]);
        let report = evaluate_vertical_slice(&fixture, &submission);
        assert_eq!(
            case(&report, "PRE_DISPATCH_POSITIVE_CONTROL")["observed"],
            "Hold(AuthorityUnknown)"
        );

        let (mut fixture, _) = inputs();
        fixture["operation_history"] = json!([]);
        let prior = resolve_prior_effect(
            &fixture,
            &fixture["operation_finality"]["approved_operation"],
            "2026-02-06T10:00:00Z",
            "2026-02-06T10:00:00Z",
        )
        .expect("missing history is a held state");
        assert_eq!(prior, PriorEffectState::OutcomeUnknownAfterDispatch);
    }
}

use jsonschema::{Validator, draft202012};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const CORPUS_VERSION: &str = "battery-passport-refresh-corpus/0.1";
const DYNAMIC_POINT_MIN: u8 = 51;
const DYNAMIC_POINT_MAX: u8 = 71;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RefreshCorpus {
    pub corpus_version: String,
    pub registry_reference: RegistryReference,
    pub cases: Vec<RefreshCase>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RegistryReference {
    pub registry_id: String,
    pub registry_version: String,
    pub guidance_version: String,
    pub registry_sha256: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RefreshCase {
    pub case_id: String,
    pub battery: BatteryIdentity,
    pub applicability: ResolvedApplicability,
    pub policy: RefreshPolicy,
    pub events: Vec<BatteryEvent>,
    pub expected: ExpectedCase,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BatteryIdentity {
    pub battery_ref: String,
    pub category: String,
    pub pack_configuration_sha256: String,
    pub passport_ref: String,
    pub lifecycle_status: String,
    pub connectivity_mode: String,
    pub responsible_operator_ref: String,
    pub first_use_date: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ResolvedApplicability {
    pub registry_sha256: String,
    pub resolved_at: String,
    /// Supplied by the complete category/date applicability compiler.
    pub applicable_dynamic_data_points: Vec<u8>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RefreshPolicy {
    pub policy_ref: String,
    pub policy_sha256: String,
    /// Optional operator policy for usage-data capture. It is not a legal interval.
    pub periodic_usage_capture_policy_ref: Option<String>,
    /// Optional purpose under which proportionate SOH monitoring is performed.
    pub soh_monitoring_purpose_ref: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BatteryEvent {
    pub event_id: String,
    pub sequence: u64,
    pub event_type: String,
    pub battery_ref: String,
    pub pack_configuration_sha256: String,
    pub passport_ref: String,
    pub occurred_at: String,
    pub received_at: String,
    pub actor_ref: String,
    pub authority_state: String,
    pub authorization_ref: Option<String>,
    pub authorization_sha256: Option<String>,
    pub authorization_scope: Vec<String>,
    pub source_class: String,
    pub evidence_ref: String,
    pub evidence_sha256: String,
    pub proposed_status: Option<String>,
    pub resulting_status: Option<String>,
    pub material_change: Option<bool>,
    pub technical_feasible: Option<bool>,
    pub purpose_ref: Option<String>,
    pub capture_policy_ref: Option<String>,
    pub source_window_start: Option<String>,
    pub source_window_end: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ExpectedCase {
    pub event_dispositions: Vec<String>,
    pub pending_data_points: Vec<u8>,
    pub new_passport_link_required: bool,
    pub case_state: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct PendingPoint {
    pub data_point: u8,
    pub disposition: String,
    pub triggering_event_ref: String,
    pub reason: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct EventAudit {
    pub event_id: String,
    pub sequence: u64,
    pub disposition: String,
    pub triggered_data_points: Vec<u8>,
    pub cleared_data_points: Vec<u8>,
    pub reason: String,
    pub delivery_state: String,
    pub new_passport_link_required: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct CaseAudit {
    pub case_id: String,
    pub case_state: String,
    pub event_audits: Vec<EventAudit>,
    pub pending_data_points: Vec<PendingPoint>,
    pub new_passport_link_required: bool,
    pub delivery_state: String,
    pub registry_submission_authorized: bool,
    pub source_grade: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct AuditReport {
    pub corpus_version: String,
    pub input_sha256: String,
    pub canonical_corpus_sha256: String,
    pub report_sha256: String,
    pub case_count: usize,
    pub operator_action_count: usize,
    pub reconciliation_count: usize,
    pub no_trigger_count: usize,
    pub cases: Vec<CaseAudit>,
}

fn sha256(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_utc_timestamp(value: &str) -> bool {
    value.len() >= 20
        && value.as_bytes().get(10) == Some(&b'T')
        && value.ends_with('Z')
        && value.is_ascii()
}

fn compile_schema(bytes: &[u8]) -> Result<Validator, String> {
    let schema: Value =
        serde_json::from_slice(bytes).map_err(|error| format!("schema JSON: {error}"))?;
    if schema.get("$schema").and_then(Value::as_str)
        != Some("https://json-schema.org/draft/2020-12/schema")
    {
        return Err("schema must declare JSON Schema Draft 2020-12".to_owned());
    }
    if !draft202012::meta::is_valid(&schema) {
        return Err("schema fails Draft 2020-12 meta-schema validation".to_owned());
    }
    draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
        .map_err(|error| format!("schema compilation: {error}"))
}

fn validate_schema(validator: &Validator, value: &Value) -> Result<(), String> {
    let errors: Vec<String> = validator
        .iter_errors(value)
        .map(|error| error.to_string())
        .collect();
    if errors.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "refresh planner schema errors: {}",
            errors.join("; ")
        ))
    }
}

fn validate_case_input(corpus: &RefreshCorpus, case: &RefreshCase) -> Vec<String> {
    let mut errors = Vec::new();
    if !nonempty(&case.case_id)
        || !nonempty(&case.battery.battery_ref)
        || !nonempty(&case.battery.passport_ref)
        || !nonempty(&case.battery.responsible_operator_ref)
        || !nonempty(&case.policy.policy_ref)
        || !is_sha256(&case.policy.policy_sha256)
        || !is_sha256(&case.battery.pack_configuration_sha256)
        || !is_sha256(&corpus.registry_reference.registry_sha256)
        || !is_sha256(&case.applicability.registry_sha256)
    {
        errors.push("CASE_IDENTITY_OR_POLICY_DIGEST_INVALID".to_owned());
    }
    if case.applicability.registry_sha256 != corpus.registry_reference.registry_sha256 {
        errors.push("APPLICABILITY_REGISTRY_DIGEST_MISMATCH".to_owned());
    }
    if !matches!(case.battery.category.as_str(), "EV" | "LMT" | "INDUSTRIAL") {
        errors.push("UNSUPPORTED_BATTERY_CATEGORY".to_owned());
    }
    if !matches!(
        case.battery.connectivity_mode.as_str(),
        "CONNECTED" | "OFFLINE_INTERMITTENT"
    ) {
        errors.push("UNSUPPORTED_CONNECTIVITY_MODE".to_owned());
    }
    if !matches!(
        case.battery.lifecycle_status.as_str(),
        "original" | "repurposed" | "re-used" | "remanufactured" | "waste"
    ) {
        errors.push("UNSUPPORTED_LIFECYCLE_STATUS".to_owned());
    }
    if !valid_utc_timestamp(&case.applicability.resolved_at) {
        errors.push("APPLICABILITY_RESOLUTION_TIMESTAMP_INVALID".to_owned());
    }
    if case.applicability.applicable_dynamic_data_points.is_empty() {
        errors.push("NO_RESOLVED_DYNAMIC_DATA_POINTS".to_owned());
    }
    let mut point_ids = BTreeSet::new();
    for point in &case.applicability.applicable_dynamic_data_points {
        if !(DYNAMIC_POINT_MIN..=DYNAMIC_POINT_MAX).contains(point)
            || !point_ids.insert(*point)
        {
            errors.push("DYNAMIC_DATA_POINT_SET_OUT_OF_RANGE_OR_DUPLICATED".to_owned());
            break;
        }
    }
    if case.events.is_empty() {
        errors.push("NO_REFRESH_EVENTS".to_owned());
    }
    errors
}

fn reject_case(case_id: &str) -> CaseAudit {
    CaseAudit {
        case_id: case_id.to_owned(),
        case_state: "REJECTED_INPUT".to_owned(),
        event_audits: Vec::new(),
        pending_data_points: Vec::new(),
        new_passport_link_required: false,
        delivery_state: "NOT_EVALUATED".to_owned(),
        registry_submission_authorized: false,
        source_grade: "NOT_EVALUATED".to_owned(),
    }
}

fn event_is_admitted(battery: &BatteryIdentity, event: &BatteryEvent) -> Result<(), String> {
    if event.battery_ref != battery.battery_ref
        || event.passport_ref != battery.passport_ref
        || event.pack_configuration_sha256 != battery.pack_configuration_sha256
    {
        return Err("event battery/passport/configuration identity conflicts with the case".to_owned());
    }
    if !nonempty(&event.event_id)
        || !nonempty(&event.actor_ref)
        || !nonempty(&event.evidence_ref)
        || !is_sha256(&event.evidence_sha256)
        || !valid_utc_timestamp(&event.occurred_at)
        || !valid_utc_timestamp(&event.received_at)
        || event.received_at.as_str() < event.occurred_at.as_str()
    {
        return Err("event identity, timestamps or evidence digest are invalid".to_owned());
    }
    if !matches!(
        event.source_class.as_str(),
        "SYNTHETIC_KNOWN_TRUTH"
            | "OPERATOR_EVENT_RECORD"
            | "MEASURED_OBSERVATION"
            | "LIFECYCLE_DOCUMENT"
    ) {
        return Err("event source class is not admitted".to_owned());
    }
    match event.authority_state.as_str() {
        "RESPONSIBLE_OPERATOR_DIRECT" => {
            if event.actor_ref != battery.responsible_operator_ref {
                return Err("direct actor is not the responsible economic operator".to_owned());
            }
        }
        "WRITTEN_DELEGATE" => {
            if event.authorization_ref.as_deref().is_none_or(str::is_empty)
                || event
                    .authorization_sha256
                    .as_deref()
                    .is_none_or(|digest| !is_sha256(digest))
                || !event
                    .authorization_scope
                    .iter()
                    .any(|scope| scope == "DYNAMIC_PASSPORT_REFRESH")
            {
                return Err("delegate lacks valid written refresh authority and scope".to_owned());
            }
        }
        _ => return Err("event authority state is unsupported".to_owned()),
    }
    Ok(())
}

fn group_points(
    applicable: &BTreeSet<u8>,
    start: u8,
    end: u8,
) -> Vec<u8> {
    (start..=end)
        .filter(|point| applicable.contains(point))
        .collect()
}

fn apply_pending(
    pending: &mut BTreeMap<u8, PendingPoint>,
    points: &[u8],
    disposition: &str,
    event_id: &str,
    reason: &str,
) {
    for point in points {
        pending.insert(
            *point,
            PendingPoint {
                data_point: *point,
                disposition: disposition.to_owned(),
                triggering_event_ref: event_id.to_owned(),
                reason: reason.to_owned(),
            },
        );
    }
}

fn clear_pending(pending: &mut BTreeMap<u8, PendingPoint>, points: &[u8]) -> Vec<u8> {
    let mut cleared = Vec::new();
    for point in points {
        if pending.remove(point).is_some() {
            cleared.push(*point);
        }
    }
    cleared
}

fn evaluate_case(case: &RefreshCase) -> CaseAudit {
    let input_errors = validate_case_input(
        // The registry reference is checked in audit_corpus before this function.
        // A small synthetic wrapper is unnecessary because the exact check is repeated below.
        &RefreshCorpus {
            corpus_version: CORPUS_VERSION.to_owned(),
            registry_reference: RegistryReference {
                registry_id: "eu-battery-passport-data-point-guidance".to_owned(),
                registry_version: "external".to_owned(),
                guidance_version: "3.2".to_owned(),
                registry_sha256: case.applicability.registry_sha256.clone(),
            },
            cases: Vec::new(),
        },
        case,
    );
    if !input_errors.is_empty() {
        return reject_case(&case.case_id);
    }

    let applicable: BTreeSet<u8> = case
        .applicability
        .applicable_dynamic_data_points
        .iter()
        .copied()
        .collect();
    let performance_and_status = group_points(&applicable, 51, 67);
    let state_of_health = group_points(&applicable, 61, 66);
    let usage_data = group_points(&applicable, 68, 71);

    let mut pending = BTreeMap::new();
    let mut event_audits = Vec::new();
    let mut seen_event_ids = BTreeSet::new();
    let mut last_sequence = 0_u64;
    let mut last_occurred_at: Option<String> = None;
    let mut delivery_state = if case.battery.connectivity_mode == "CONNECTED" {
        "CONNECTED".to_owned()
    } else {
        "OFFLINE".to_owned()
    };
    let mut new_passport_link_required = false;
    let mut source_grade = if case
        .events
        .iter()
        .all(|event| event.source_class == "SYNTHETIC_KNOWN_TRUTH")
    {
        "SYNTHETIC_ONLY".to_owned()
    } else {
        "ADMITTED_EVENT_SOURCES".to_owned()
    };

    for event in &case.events {
        let mut disposition = "REVIEW_REQUIRED".to_owned();
        let mut triggered = Vec::new();
        let mut cleared = Vec::new();
        let mut reason = String::new();
        let mut event_new_passport = false;

        if !seen_event_ids.insert(event.event_id.clone())
            || event.sequence <= last_sequence
            || last_occurred_at
                .as_ref()
                .is_some_and(|last| event.occurred_at.as_str() < last.as_str())
        {
            disposition = "CONFLICTING_EVENT".to_owned();
            reason = "duplicate event ID, non-increasing sequence or regressing event time".to_owned();
        } else if let Err(error) = event_is_admitted(&case.battery, event) {
            disposition = if error.contains("identity conflicts") {
                "CONFLICTING_EVENT".to_owned()
            } else {
                "BLOCKED_UNQUALIFIED_EVENT".to_owned()
            }
            .to_owned();
            reason = error;
        } else {
            last_sequence = event.sequence;
            last_occurred_at = Some(event.occurred_at.clone());
            match event.event_type.as_str() {
                "STATUS_CHANGE_CONSIDERED" => {
                    if event.proposed_status.as_deref().is_none_or(str::is_empty)
                        || event.proposed_status.as_deref() == Some(case.battery.lifecycle_status.as_str())
                    {
                        disposition = "BLOCKED_UNQUALIFIED_EVENT".to_owned();
                        reason = "status consideration event has no proposed lifecycle status".to_owned();
                    } else {
                        triggered = performance_and_status.clone();
                        apply_pending(
                            &mut pending,
                            &triggered,
                            "UPDATE_REQUIRED",
                            &event.event_id,
                            "battery status change is being considered; reassess performance/durability and status points before the decision",
                        );
                        disposition = if triggered.is_empty() {
                            "REVIEW_REQUIRED".to_owned()
                        } else {
                            "UPDATE_REQUIRED".to_owned()
                        };
                        reason = "status-change consideration is a trigger; do not wait for the completed transition".to_owned();
                    }
                }
                "STATUS_CHANGED" => {
                    let Some(status) = event.resulting_status.as_deref() else {
                        disposition = "BLOCKED_UNQUALIFIED_EVENT".to_owned();
                        reason = "completed status-change event has no resulting lifecycle status".to_owned();
                        event_audits.push(EventAudit {
                            event_id: event.event_id.clone(),
                            sequence: event.sequence,
                            disposition,
                            triggered_data_points: triggered,
                            cleared_data_points: cleared,
                            reason,
                            delivery_state: delivery_state.clone(),
                            new_passport_link_required: false,
                        });
                        continue;
                    };
                    if !matches!(
                        status,
                        "original" | "repurposed" | "re-used" | "remanufactured" | "waste"
                    ) {
                        disposition = "BLOCKED_UNQUALIFIED_EVENT".to_owned();
                        reason = "resulting lifecycle status is unsupported".to_owned();
                    } else if status == case.battery.lifecycle_status {
                        disposition = "CONFLICTING_EVENT".to_owned();
                        reason = "STATUS_CHANGED event does not change the declared lifecycle status".to_owned();
                    } else {
                        triggered = performance_and_status.clone();
                        apply_pending(
                            &mut pending,
                            &triggered,
                            "UPDATE_REQUIRED",
                            &event.event_id,
                            "battery status changed; refresh performance/durability and status points against the new lifecycle state",
                        );
                        if matches!(status, "repurposed" | "remanufactured") {
                            new_passport_link_required = true;
                            event_new_passport = true;
                        }
                        disposition = if triggered.is_empty() {
                            "REVIEW_REQUIRED".to_owned()
                        } else {
                            "UPDATE_REQUIRED".to_owned()
                        };
                        reason = "completed status transition requires an evidence refresh; new passport linkage is required when repurposed or remanufactured".to_owned();
                    }
                }
                "FIRST_USE_CONFIRMED" => {
                    let event_date = event.occurred_at.split('T').next().unwrap_or_default();
                    if case.battery.first_use_date.as_deref().is_some_and(|date| date != event_date) {
                        disposition = "CONFLICTING_EVENT".to_owned();
                        reason = "first-use event date conflicts with the existing battery first-use record".to_owned();
                        event_audits.push(EventAudit {
                            event_id: event.event_id.clone(),
                            sequence: event.sequence,
                            disposition,
                            triggered_data_points: triggered,
                            cleared_data_points: cleared,
                            reason,
                            delivery_state: delivery_state.clone(),
                            new_passport_link_required: false,
                        });
                        continue;
                    }
                    triggered = usage_data.clone();
                    apply_pending(
                        &mut pending,
                        &triggered,
                        "REVIEW_REQUIRED",
                        &event.event_id,
                        "first use activates use-related data applicability; capture only observations that now exist",
                    );
                    disposition = "REVIEW_REQUIRED".to_owned();
                    reason = "first use activates use-related points 68–71 without fabricating prior usage history".to_owned();
                }
                "PERIODIC_USAGE_CAPTURE" => {
                    if case
                        .policy
                        .periodic_usage_capture_policy_ref
                        .as_deref()
                        .is_none_or(|policy| Some(policy) != event.capture_policy_ref.as_deref())
                        || event.source_window_start.as_deref().is_none_or(str::is_empty)
                        || event.source_window_end.as_deref().is_none_or(str::is_empty)
                        || event
                            .source_window_start
                            .as_deref()
                            .is_none_or(|value| !valid_utc_timestamp(value))
                        || event
                            .source_window_end
                            .as_deref()
                            .is_none_or(|value| !valid_utc_timestamp(value))
                        || event
                            .source_window_start
                            .as_ref()
                            .zip(event.source_window_end.as_ref())
                            .is_some_and(|(start, end)| start > end)
                    {
                        disposition = "BLOCKED_UNQUALIFIED_EVENT".to_owned();
                        reason = "usage capture lacks the exact declared policy or a valid source window".to_owned();
                    } else {
                        triggered = usage_data.clone();
                        apply_pending(
                            &mut pending,
                            &triggered,
                            "UPDATE_REQUIRED",
                            &event.event_id,
                            "declared periodic-use capture policy emitted a source window for points 68–71",
                        );
                        disposition = "UPDATE_REQUIRED".to_owned();
                        reason = "operator-declared periodic usage capture; this interval is not asserted to be a legal universal cadence".to_owned();
                    }
                }
                "SERVICE_VISIT" => {
                    triggered = usage_data.clone();
                    apply_pending(
                        &mut pending,
                        &triggered,
                        "REVIEW_REQUIRED",
                        &event.event_id,
                        "service visit is an offline collection/reconciliation opportunity for use-related data",
                    );
                    delivery_state = "SERVICE_RECONCILIATION_AVAILABLE".to_owned();
                    disposition = "REVIEW_REQUIRED".to_owned();
                    reason = "service visit creates an evidence-reconciliation opportunity; it does not assert that every field changed".to_owned();
                }
                "SOH_MONITORING_REVIEW" => {
                    if case.policy.soh_monitoring_purpose_ref.as_deref().is_none_or(|purpose| {
                        Some(purpose) != event.purpose_ref.as_deref()
                    }) || event.technical_feasible.is_none()
                    {
                        disposition = "BLOCKED_UNQUALIFIED_EVENT".to_owned();
                        reason = "SOH review lacks a declared purpose or technical-feasibility decision".to_owned();
                    } else if event.technical_feasible == Some(false) {
                        disposition = "DEFERRED_WAITING_FOR_SOURCE".to_owned();
                        reason = "SOH refresh deferred because the declared measurement is not technically feasible; any prior due state remains open".to_owned();
                    } else if event.material_change == Some(true) {
                        triggered = state_of_health.clone();
                        apply_pending(
                            &mut pending,
                            &triggered,
                            "UPDATE_REQUIRED",
                            &event.event_id,
                            "declared SOH-monitoring purpose found a material change using technically feasible evidence",
                        );
                        disposition = "UPDATE_REQUIRED".to_owned();
                        reason = "proportionate SOH monitoring found a material change".to_owned();
                    } else if event.material_change == Some(false) {
                        cleared = clear_pending(&mut pending, &state_of_health);
                        disposition = "NO_CHANGE_RECORDED".to_owned();
                        reason = "technically feasible SOH review found no material change; review receipt retained without fabricating a new value".to_owned();
                    } else {
                        disposition = "DEFERRED_WAITING_FOR_SOURCE".to_owned();
                        reason = "SOH review has no materiality result; no update or no-change conclusion is inferred".to_owned();
                    }
                }
                "NO_MATERIAL_CHANGE_ASSESSED" => {
                    if case.policy.soh_monitoring_purpose_ref.as_deref().is_none_or(|purpose| {
                        Some(purpose) != event.purpose_ref.as_deref()
                    }) || event.technical_feasible != Some(true)
                        || event.material_change != Some(false)
                    {
                        disposition = "BLOCKED_UNQUALIFIED_EVENT".to_owned();
                        reason = "no-material-change receipt lacks the declared purpose, technical feasibility or a negative materiality result".to_owned();
                    } else {
                        cleared = clear_pending(&mut pending, &state_of_health);
                        disposition = "NO_CHANGE_RECORDED".to_owned();
                        reason = "no material change recorded for the declared SOH purpose; no new measured value is fabricated".to_owned();
                    }
                }
                "CONNECTIVITY_UNAVAILABLE" => {
                    delivery_state = "OFFLINE".to_owned();
                    disposition = "CONNECTIVITY_BLOCKED".to_owned();
                    reason = "connectivity loss is recorded separately and does not satisfy or clear an outstanding refresh".to_owned();
                }
                "WASTE_PREPARATION_FOR_REUSE_OR_REPURPOSE" => {
                    if case.battery.lifecycle_status != "waste"
                        || !matches!(
                            event.resulting_status.as_deref(),
                            Some("re-used" | "repurposed")
                        )
                    {
                        disposition = "BLOCKED_UNQUALIFIED_EVENT".to_owned();
                        reason = "waste-preparation event must bind the resulting re-used or repurposed status".to_owned();
                    } else {
                        triggered = performance_and_status.clone();
                        apply_pending(
                            &mut pending,
                            &triggered,
                            "UPDATE_REQUIRED",
                            &event.event_id,
                            "waste battery is being prepared for reuse or repurposing; new passport and predecessor link must be reviewed",
                        );
                        new_passport_link_required = true;
                        event_new_passport = true;
                        disposition = "UPDATE_REQUIRED".to_owned();
                        reason = "preparation for reuse/repurposing after waste status requires a new passport linked to its predecessor".to_owned();
                    }
                }
                _ => {
                    disposition = "BLOCKED_UNQUALIFIED_EVENT".to_owned();
                    reason = "unsupported event type".to_owned();
                }
            }
        }

        event_audits.push(EventAudit {
            event_id: event.event_id.clone(),
            sequence: event.sequence,
            disposition,
            triggered_data_points: triggered,
            cleared_data_points: cleared,
            reason,
            delivery_state: delivery_state.clone(),
            new_passport_link_required: event_new_passport,
        });
    }

    let pending_data_points: Vec<PendingPoint> = pending.into_values().collect();
    let has_blocker = event_audits.iter().any(|event| {
        matches!(
            event.disposition.as_str(),
            "BLOCKED_UNQUALIFIED_EVENT" | "CONFLICTING_EVENT" | "DEFERRED_WAITING_FOR_SOURCE"
        )
    });
    let case_state = if has_blocker {
        "NEEDS_RECONCILIATION"
    } else if !pending_data_points.is_empty() || new_passport_link_required {
        "OPERATOR_ACTION_REQUIRED"
    } else if event_audits
        .iter()
        .any(|event| event.disposition == "NO_CHANGE_RECORDED")
    {
        "REVIEW_RECORDED"
    } else {
        "NO_TRIGGER_RECORDED"
    };
    if case.battery.connectivity_mode == "CONNECTED"
        && delivery_state == "OFFLINE"
    {
        source_grade = "CONNECTIVITY_DEGRADED".to_owned();
    }
    CaseAudit {
        case_id: case.case_id.clone(),
        case_state: case_state.to_owned(),
        event_audits,
        pending_data_points,
        new_passport_link_required,
        delivery_state,
        registry_submission_authorized: false,
        source_grade,
    }
}

fn canonicalize(corpus: &mut RefreshCorpus) {
    corpus.cases.sort_by(|left, right| left.case_id.cmp(&right.case_id));
    for case in &mut corpus.cases {
        case.applicability
            .applicable_dynamic_data_points
            .sort_unstable();
        case.expected.pending_data_points.sort_unstable();
    }
}

pub fn audit_corpus(
    schema_bytes: &[u8],
    corpus_bytes: &[u8],
) -> Result<AuditReport, String> {
    let validator = compile_schema(schema_bytes)?;
    let value: Value =
        serde_json::from_slice(corpus_bytes).map_err(|error| format!("corpus JSON: {error}"))?;
    validate_schema(&validator, &value)?;
    let mut corpus: RefreshCorpus =
        serde_json::from_value(value).map_err(|error| format!("corpus decode: {error}"))?;
    if corpus.corpus_version != CORPUS_VERSION || corpus.cases.is_empty() {
        return Err("unsupported or empty Battery Passport refresh corpus".to_owned());
    }
    if corpus.registry_reference.registry_id != "eu-battery-passport-data-point-guidance"
        || !nonempty(&corpus.registry_reference.registry_version)
        || !nonempty(&corpus.registry_reference.guidance_version)
        || !is_sha256(&corpus.registry_reference.registry_sha256)
    {
        return Err("refresh corpus registry reference is incomplete or malformed".to_owned());
    }
    canonicalize(&mut corpus);
    let canonical_bytes =
        serde_json::to_vec(&corpus).map_err(|error| format!("canonical corpus: {error}"))?;

    let mut case_ids = BTreeSet::new();
    let mut case_audits = Vec::new();
    for case in &corpus.cases {
        if !case_ids.insert(case.case_id.clone()) {
            return Err(format!("duplicate refresh case identity: {}", case.case_id));
        }
        let errors = validate_case_input(&corpus, case);
        if !errors.is_empty() {
            return Err(format!(
                "refresh case {} has invalid input: {}",
                case.case_id,
                errors.join("; ")
            ));
        }
        let audit = evaluate_case(case);
        let event_dispositions: Vec<String> = audit
            .event_audits
            .iter()
            .map(|event| event.disposition.clone())
            .collect();
        let pending: Vec<u8> = audit
            .pending_data_points
            .iter()
            .map(|item| item.data_point)
            .collect();
        if event_dispositions != case.expected.event_dispositions
            || pending != case.expected.pending_data_points
            || audit.new_passport_link_required != case.expected.new_passport_link_required
            || audit.case_state != case.expected.case_state
        {
            return Err(format!(
                "refresh fixture {} expectation mismatch: events={event_dispositions:?}, pending={pending:?}, new_passport={}, state={}; expected events={:?}, pending={:?}, new_passport={}, state={}",
                case.case_id,
                audit.new_passport_link_required,
                audit.case_state,
                case.expected.event_dispositions,
                case.expected.pending_data_points,
                case.expected.new_passport_link_required,
                case.expected.case_state
            ));
        }
        case_audits.push(audit);
    }
    let report = AuditReport {
        corpus_version: corpus.corpus_version,
        input_sha256: sha256(corpus_bytes),
        canonical_corpus_sha256: sha256(&canonical_bytes),
        report_sha256: String::new(),
        case_count: case_audits.len(),
        operator_action_count: case_audits
            .iter()
            .filter(|case| case.case_state == "OPERATOR_ACTION_REQUIRED")
            .count(),
        reconciliation_count: case_audits
            .iter()
            .filter(|case| case.case_state == "NEEDS_RECONCILIATION")
            .count(),
        no_trigger_count: case_audits
            .iter()
            .filter(|case| case.case_state == "NO_TRIGGER_RECORDED")
            .count(),
        cases: case_audits,
    };
    let report_bytes =
        serde_json::to_vec(&report).map_err(|error| format!("report serialization: {error}"))?;
    let mut final_report = report;
    final_report.report_sha256 = sha256(&report_bytes);
    Ok(final_report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const SCHEMA: &[u8] =
        include_bytes!("../../../schemas/mobility/battery_passport_refresh_v0_1.schema.json");
    const CORPUS: &[u8] =
        include_bytes!("../../../fixtures/mobility/battery_passport_refresh_v0_1.json");

    fn report() -> AuditReport {
        audit_corpus(SCHEMA, CORPUS).expect("frozen refresh planner corpus")
    }

    fn case<'a>(report: &'a AuditReport, id: &str) -> &'a CaseAudit {
        report.cases.iter().find(|item| item.case_id == id).expect("case audit")
    }

    #[test]
    fn frozen_event_corpus_replays_expected_refresh_dispositions() {
        let report = report();
        assert_eq!(report.case_count, 12);
        assert_eq!(report.operator_action_count, 7);
        assert_eq!(report.reconciliation_count, 3);
        assert_eq!(report.no_trigger_count, 1);
    }

    #[test]
    fn status_change_consideration_triggers_before_completed_transition() {
        let row = case(&report(), "status_change_considered_before_transition");
        assert_eq!(row.event_audits[0].disposition, "UPDATE_REQUIRED");
        assert!(row.pending_data_points.iter().any(|item| item.data_point == 67));
    }

    #[test]
    fn remanufacture_creates_new_passport_link_requirement() {
        let row = case(&report(), "remanufacture_requires_new_passport_link");
        assert!(row.new_passport_link_required);
        assert!(row.pending_data_points.iter().any(|item| item.data_point == 67));
    }

    #[test]
    fn simple_service_repair_does_not_create_new_passport() {
        let row = case(&report(), "offline_service_visit_is_review_opportunity");
        assert!(!row.new_passport_link_required);
        assert_eq!(row.delivery_state, "SERVICE_RECONCILIATION_AVAILABLE");
    }

    #[test]
    fn first_use_activates_use_related_review_without_inventing_history() {
        let row = case(&report(), "first_use_activates_usage_points");
        assert!(row.pending_data_points.iter().any(|item| item.data_point == 68));
        assert!(row.pending_data_points.iter().any(|item| item.data_point == 71));
    }

    #[test]
    fn offline_connectivity_does_not_clear_due_updates() {
        let row = case(&report(), "connectivity_loss_does_not_satisfy_due_work");
        assert_eq!(row.event_audits[0].disposition, "UPDATE_REQUIRED");
        assert_eq!(row.event_audits[1].disposition, "CONNECTIVITY_BLOCKED");
        assert!(!row.pending_data_points.is_empty());
    }

    #[test]
    fn no_material_change_closes_soh_review_without_fake_measurement() {
        let row = case(&report(), "no_material_soh_change_records_review");
        assert_eq!(row.event_audits[0].disposition, "NO_CHANGE_RECORDED");
        assert!(row.pending_data_points.is_empty());
    }

    #[test]
    fn weekly_refresh_is_not_invented_without_a_declared_trigger() {
        let row = case(&report(), "no_universal_weekly_update");
        assert_eq!(row.case_state, "NO_TRIGGER_RECORDED");
        assert!(row.pending_data_points.is_empty());
    }

    #[test]
    fn periodic_usage_capture_requires_declared_policy_and_window() {
        let row = case(&report(), "declared_periodic_usage_capture");
        assert_eq!(row.event_audits[0].disposition, "UPDATE_REQUIRED");
        assert_eq!(row.pending_data_points.len(), 4);
    }

    #[test]
    fn delegated_actor_requires_written_scope_and_digest() {
        let row = case(&report(), "delegated_actor_missing_scope_is_blocked");
        assert_eq!(row.event_audits[0].disposition, "BLOCKED_UNQUALIFIED_EVENT");
        assert_eq!(row.case_state, "NEEDS_RECONCILIATION");
    }

    #[test]
    fn wrong_configuration_creates_conflicting_event_not_new_truth() {
        let row = case(&report(), "wrong_configuration_event_is_conflict");
        assert_eq!(row.event_audits[0].disposition, "CONFLICTING_EVENT");
        assert!(row.pending_data_points.is_empty());
    }

    #[test]
    fn duplicate_or_out_of_order_source_sequence_is_conflicting() {
        let row = case(&report(), "out_of_order_event_sequence_is_conflict");
        assert_eq!(row.event_audits[1].disposition, "CONFLICTING_EVENT");
    }

    #[test]
    fn waste_preparation_for_reuse_requires_new_passport_lineage() {
        let row = case(&report(), "waste_preparation_for_reuse_requires_link");
        assert!(row.new_passport_link_required);
        assert_eq!(row.event_audits[0].disposition, "UPDATE_REQUIRED");
    }

    #[test]
    fn applicability_id_and_case_order_permutations_are_canonical() {
        let original = report();
        let mut corpus: RefreshCorpus = serde_json::from_slice(CORPUS).expect("corpus");
        corpus.cases.reverse();
        for item in &mut corpus.cases {
            item.applicability.applicable_dynamic_data_points.reverse();
            item.expected.pending_data_points.reverse();
        }
        let bytes = serde_json::to_vec(&corpus).expect("permuted corpus");
        let replay = audit_corpus(SCHEMA, &bytes).expect("permuted audit");
        assert_eq!(original.canonical_corpus_sha256, replay.canonical_corpus_sha256);
        assert_eq!(
            serde_json::to_value(&original.cases).unwrap(),
            serde_json::to_value(&replay.cases).unwrap()
        );
    }
}

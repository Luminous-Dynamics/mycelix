use jsonschema::validator_for;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

const DIALECT: &str = "https://json-schema.org/draft/2020-12/schema";
const VALIDATOR: &str = "jsonschema@0.59.0";
const REPORT_VERSION: &str = "1.0.0";
const CLAIM_CEILING: &str = "Local fixture/schema/corpus consistency only. Does not prove source authenticity, user comprehension, UI behavior, accessibility conformance, production synchronization correctness, agronomic safety, completed service, or field impact.";

const CASES: &str = "research/farmer-ux-001-cases-v1.json";
const CASES_SCHEMA: &str = "research/farmer-ux-001-cases-v1.schema.json";
const ROLES: &str = "research/farmer-ux-001-role-task-matrix-v1.json";
const ROLES_SCHEMA: &str = "research/farmer-ux-001-role-task-matrix-v1.schema.json";
const COPY: &str = "research/farmer-ux-001-copy-v1.json";
const COPY_SCHEMA: &str = "research/farmer-ux-001-copy-v1.schema.json";

const REQUIRED_CASES: &[&str] = &[
    "FUX-OFFLINE-001", "FUX-QUOTE-002", "FUX-QUOTE-003", "FUX-SYNC-004",
    "FUX-ROUTE-005", "FUX-NOPROVIDER-006", "FUX-COMPLETION-007",
    "FUX-REWORK-008", "FUX-EVIDENCE-009", "FUX-CONSENT-010",
    "FUX-HUMAN-011", "FUX-PAYMENT-012", "FUX-AUTH-013", "FUX-SCOPE-014",
    "FUX-ROUTE-015", "FUX-PRIVACY-016", "FUX-DIGEST-017",
    "FUX-LANGUAGE-018", "FUX-CAPACITY-019", "FUX-EXPORT-020",
    "FUX-DEFER-021", "FUX-NOSHOW-022",
];
const REQUIRED_ROLES: &[&str] = &[
    "farmer_smallholder_crop", "farmer_mixed_or_livestock",
    "extension_officer_agronomist", "irrigation_water_service_provider",
    "farm_machinery_operator_or_coordinator", "producer_cooperative_or_food_coordinator",
    "solar_energy_and_pump_technician", "construction_repair_trade",
    "circular_economy_operator",
];
const REQUIRED_COPY: &[&str] = &[
    "local_saved", "sync_pending", "sync_confirmed", "sync_conflict",
    "evidence_stale", "evidence_conflict", "evidence_unknown", "modelled_route",
    "no_route_evidence", "no_provider", "quote_incomplete", "quote_expired",
    "booking_confirmed", "provider_reported_complete", "service_accepted",
    "service_disputed", "permission_revoked", "human_mediation", "outcome_unknown",
];
const REQUIRED_INVARIANTS: &[&str] = &[
    "unknown_is_not_zero_or_verified",
    "modelled_is_not_observed",
    "request_is_not_quote",
    "quote_is_not_booking",
    "booking_is_not_completed_work",
    "provider_completion_is_not_farmer_acceptance",
    "locally_acknowledged_event_must_not_be_lost",
    "duplicate_replay_must_not_duplicate_side_effects",
    "conflicting_commitments_must_not_be_silently_overwritten",
    "recommendation_does_not_authorize_payment_procurement_dispatch_or_actuation",
    "human_mediated_channel_must_preserve_request_reference",
];

const REQUIRED_CASE_STATES: &[(&str, &str, &str)] = &[
    ("FUX-QUOTE-002", "quote_expired_needs_new_offer", "expired_quote"),
    ("FUX-QUOTE-003", "quote_incomplete", "needs_complete_quote"),
    ("FUX-ROUTE-005", "route_access_unknown", "travel_time_unknown"),
    ("FUX-COMPLETION-007", "completed_awaiting_farmer_decision", "provider_reports_completion_review_needed"),
    ("FUX-AUTH-013", "relay_not_authorized", "authorization_needed"),
    ("FUX-SCOPE-014", "quote_scope_mismatch", "quote_does_not_match_request"),
    ("FUX-ROUTE-015", "route_estimate_only", "modelled_time_not_measured"),
    ("FUX-PRIVACY-016", "precision_restricted", "coarsened_or_access_denied"),
    ("FUX-DIGEST-017", "provenance_unverified", "reference_not_authenticated"),
    ("FUX-CAPACITY-019", "provider_capacity_changed", "offer_no_longer_fulfillable"),
    ("FUX-EXPORT-020", "export_includes_sync_state", "export_with_pending_events_disclosed"),
    ("FUX-DEFER-021", "farmer_deferred_with_reminder", "deferred_not_failed"),
    ("FUX-NOSHOW-022", "no_show_unresolved", "service_not_delivered_escalation_available"),
];

const REQUIRED_FORBIDDEN_LABELS: &[(&str, &str)] = &[
    ("FUX-QUOTE-002", "booking_confirmed"),
    ("FUX-QUOTE-003", "booking_confirmed"),
    ("FUX-SYNC-004", "silent_last_write_wins"),
    ("FUX-ROUTE-005", "invented_travel_minutes"),
    ("FUX-NOPROVIDER-006", "request_hidden_from_history"),
    ("FUX-COMPLETION-007", "farmer_accepted"),
    ("FUX-REWORK-008", "accepted_service"),
    ("FUX-CONSENT-010", "future_share_without_authority"),
    ("FUX-HUMAN-011", "forced_app_registration"),
    ("FUX-PAYMENT-012", "payment_completed"),
    ("FUX-AUTH-013", "request_submitted_as_farmer"),
    ("FUX-SCOPE-014", "booking_confirmed"),
    ("FUX-ROUTE-015", "observed_trip_claim"),
    ("FUX-PRIVACY-016", "exact_coordinate_in_public_projection"),
    ("FUX-DIGEST-017", "source_authenticated"),
    ("FUX-LANGUAGE-018", "acceptance_assumed_from_button_click"),
    ("FUX-CAPACITY-019", "booking_confirmed"),
    ("FUX-EXPORT-020", "all_events_claimed_server_confirmed"),
    ("FUX-DEFER-021", "action_claimed_completed"),
    ("FUX-NOSHOW-022", "service_fulfilled"),
    ("FUX-NOSHOW-022", "request_removed_from_metrics"),
];

const REQUIRED_COPY_NONIMPLICATIONS: &[(&str, &str)] = &[
    ("local_saved", "server_received"),
    ("local_saved", "provider_notified"),
    ("local_saved", "booking_confirmed"),
    ("sync_pending", "request_received_by_provider"),
    ("sync_pending", "booking_confirmed"),
    ("sync_confirmed", "provider_available"),
    ("sync_confirmed", "quote_accepted"),
    ("sync_confirmed", "booking_confirmed"),
    ("evidence_stale", "current_field_condition_verified"),
    ("evidence_conflict", "conflicting_values_can_be_averaged_into_truth"),
    ("evidence_unknown", "unknown_means_zero"),
    ("evidence_unknown", "unknown_means_safe"),
    ("evidence_unknown", "unknown_means_false"),
    ("modelled_route", "observed_travel_time"),
    ("modelled_route", "guaranteed_arrival"),
    ("no_route_evidence", "zero_minutes"),
    ("no_route_evidence", "guaranteed_service_access"),
    ("no_provider", "service_delivered"),
    ("no_provider", "request_deleted"),
    ("quote_incomplete", "total_cost_is_known"),
    ("quote_incomplete", "booking_confirmed"),
    ("quote_expired", "old_price_is_still_valid"),
    ("quote_expired", "booking_confirmed"),
    ("booking_confirmed", "work_completed"),
    ("booking_confirmed", "farmer_accepted_completion"),
    ("booking_confirmed", "payment_completed"),
    ("provider_reported_complete", "farmer_accepted"),
    ("provider_reported_complete", "payment_authorized"),
    ("service_accepted", "payment_completed_unless_separately_confirmed"),
    ("service_disputed", "fault_or_guilt_adjudicated"),
    ("service_disputed", "dispute_resolved"),
    ("service_disputed", "farmer_penalized"),
    ("permission_revoked", "external_copies_were_recalled"),
    ("permission_revoked", "all_data_is_deleted_everywhere"),
    ("human_mediation", "app_registration_required"),
    ("human_mediation", "human_relay_authority_is_implicit"),
    ("outcome_unknown", "request_not_applied"),
    ("outcome_unknown", "request_received_by_provider"),
    ("outcome_unknown", "booking_confirmed"),
    ("outcome_unknown", "automatic_retry_is_safe"),
    ("outcome_unknown", "request_was_lost"),
];

#[derive(Debug, Serialize)]
struct ArtifactReport {
    name: String,
    instance_path: String,
    schema_path: String,
    instance_sha256: String,
    schema_sha256: String,
    instance_bytes: usize,
    schema_bytes: usize,
    schema_dialect: String,
    schema_status: String,
    schema_errors: Vec<String>,
}
#[derive(Debug, Serialize)]
struct Counts {
    cases: usize,
    roles: usize,
    copy_states: usize,
    shared_invariants: usize,
}
#[derive(Debug, Serialize)]
struct Report {
    report_version: String,
    tool_version: String,
    validator: String,
    status: String,
    artifacts: Vec<ArtifactReport>,
    counts: Counts,
    schema_error_count: usize,
    semantic_error_count: usize,
    errors: Vec<String>,
    claim_ceiling: String,
}
struct Artifact {
    name: &'static str,
    instance_path: &'static str,
    schema_path: &'static str,
    instance_bytes: Vec<u8>,
    schema_bytes: Vec<u8>,
    instance: Value,
    schema: Value,
    schema_errors: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct MutationCorpus {
    schema_version: String,
    mutations: Vec<MutationSpec>,
}

#[derive(Debug, Deserialize)]
struct MutationSpec {
    id: String,
    target: String,
    collection: String,
    id_field: String,
    entry_id: String,
    operation: String,
    pointer: Option<String>,
    value: Option<Value>,
    expected_error_contains: String,
}

fn sha256(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in Sha256::digest(bytes) {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}
fn read_json(root: &Path, relative: &str) -> Result<(Vec<u8>, Value), String> {
    let path = root.join(relative);
    let bytes = fs::read(&path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let value = serde_json::from_slice(&bytes).map_err(|e| format!("cannot parse {}: {e}", path.display()))?;
    Ok((bytes, value))
}
fn validate_schema(name: &str, instance: &Value, schema: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    match schema.get("$schema").and_then(Value::as_str) {
        Some(DIALECT) => {}
        Some(other) => errors.push(format!("{name}: unsupported schema dialect {other:?}; expected {DIALECT}")),
        None => errors.push(format!("{name}: missing $schema dialect identifier")),
    }
    match jsonschema::meta::validate(schema) {
        Ok(()) => {}
        Err(error) => errors.push(format!("{name}: invalid JSON Schema document: {error}")),
    }

    match validator_for(schema) {
        Ok(validator) => {
            for error in validator.iter_errors(instance) {
                errors.push(format!("{name}: schema violation at {}: {error}", error.instance_path()));
            }
        }
        Err(error) => errors.push(format!("{name}: schema compilation failed: {error}")),
    }
    errors.sort();
    errors
}
fn load(root: &Path, name: &'static str, instance_path: &'static str, schema_path: &'static str) -> Result<Artifact, String> {
    let (instance_bytes, instance) = read_json(root, instance_path)?;
    let (schema_bytes, schema) = read_json(root, schema_path)?;
    let schema_errors = validate_schema(name, &instance, &schema);
    Ok(Artifact { name, instance_path, schema_path, instance_bytes, schema_bytes, instance, schema, schema_errors })
}
fn array<'a>(doc: &'a Value, key: &str) -> &'a [Value] {
    doc.get(key).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])
}
fn ids(doc: &Value, collection: &str, field: &str, label: &str, errors: &mut Vec<String>) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    for (idx, item) in array(doc, collection).iter().enumerate() {
        let Some(id) = item.get(field).and_then(Value::as_str) else {
            errors.push(format!("{label}[{idx}]: missing string {field}"));
            continue;
        };
        if !out.insert(id.to_owned()) {
            errors.push(format!("{label}: duplicate {field} {id:?}"));
        }
    }
    out
}
fn require_ids(found: &BTreeSet<String>, required: &[&str], label: &str, errors: &mut Vec<String>) {
    for id in required {
        if !found.contains(*id) {
            errors.push(format!("{label}: required v1 entry is missing: {id}"));
        }
    }
}
fn string_set(value: Option<&Value>) -> BTreeSet<String> {
    value.and_then(Value::as_array).into_iter().flatten()
        .filter_map(Value::as_str).map(str::to_owned).collect()
}
fn find<'a>(doc: &'a Value, collection: &str, key: &str, wanted: &str) -> Option<&'a Value> {
    array(doc, collection).iter().find(|v| v.get(key).and_then(Value::as_str) == Some(wanted))
}
fn apply_mutation(
    cases: &mut Value,
    roles: &mut Value,
    copy: &mut Value,
    mutation: &MutationSpec,
) -> Result<(), String> {
    let document = match mutation.target.as_str() {
        "cases" => cases,
        "roles" => roles,
        "copy" => copy,
        other => return Err(format!("unknown mutation target {other:?}")),
    };
    let collection = document
        .get_mut(&mutation.collection)
        .and_then(Value::as_array_mut)
        .ok_or_else(|| format!("mutation {}: missing array {}", mutation.id, mutation.collection))?;
    let matches_id = |item: &Value| {
        item.get(&mutation.id_field).and_then(Value::as_str) == Some(mutation.entry_id.as_str())
    };

    match mutation.operation.as_str() {
        "duplicate_entry" => {
            let original = collection.iter().find(|item| matches_id(item)).cloned()
                .ok_or_else(|| format!("mutation {}: source entry not found", mutation.id))?;
            collection.push(original);
        }
        "remove_entry" => {
            let original_len = collection.len();
            collection.retain(|item| !matches_id(item));
            if collection.len() == original_len {
                return Err(format!("mutation {}: source entry not found", mutation.id));
            }
        }
        "set_value" => {
            let entry = collection.iter_mut().find(|item| matches_id(item))
                .ok_or_else(|| format!("mutation {}: source entry not found", mutation.id))?;
            let pointer = mutation.pointer.as_deref()
                .ok_or_else(|| format!("mutation {}: set_value requires pointer", mutation.id))?;
            let target = entry.pointer_mut(pointer)
                .ok_or_else(|| format!("mutation {}: pointer {pointer:?} not found", mutation.id))?;
            *target = mutation.value.clone()
                .ok_or_else(|| format!("mutation {}: set_value requires value", mutation.id))?;
        }
        "append_string" => {
            let entry = collection.iter_mut().find(|item| matches_id(item))
                .ok_or_else(|| format!("mutation {}: source entry not found", mutation.id))?;
            let pointer = mutation.pointer.as_deref()
                .ok_or_else(|| format!("mutation {}: append_string requires pointer", mutation.id))?;
            let target = entry.pointer_mut(pointer).and_then(Value::as_array_mut)
                .ok_or_else(|| format!("mutation {}: pointer {pointer:?} is not an array", mutation.id))?;
            let value = mutation.value.as_ref().and_then(Value::as_str)
                .ok_or_else(|| format!("mutation {}: append_string requires string value", mutation.id))?;
            target.push(Value::String(value.to_owned()));
        }
        "remove_string" => {
            let entry = collection.iter_mut().find(|item| matches_id(item))
                .ok_or_else(|| format!("mutation {}: source entry not found", mutation.id))?;
            let pointer = mutation.pointer.as_deref()
                .ok_or_else(|| format!("mutation {}: remove_string requires pointer", mutation.id))?;
            let target = entry.pointer_mut(pointer).and_then(Value::as_array_mut)
                .ok_or_else(|| format!("mutation {}: pointer {pointer:?} is not an array", mutation.id))?;
            let value = mutation.value.as_ref().and_then(Value::as_str)
                .ok_or_else(|| format!("mutation {}: remove_string requires string value", mutation.id))?;
            target.retain(|item| item.as_str() != Some(value));
        }
        other => return Err(format!("mutation {}: unknown operation {other:?}", mutation.id)),
    }
    Ok(())
}

fn check_count(doc: &Value, case_id: &str, field: &str, wanted: i64, errors: &mut Vec<String>) {
    let path = format!("/expected/side_effect_counts/{field}");
    let found = find(doc, "cases", "id", case_id).and_then(|v| v.pointer(&path)).and_then(Value::as_i64);
    if found != Some(wanted) {
        errors.push(format!("{case_id}: expected side_effect_counts.{field}={wanted}, found {found:?}"));
    }
}
fn semantic_checks(cases: &Value, roles: &Value, copy: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    let case_ids = ids(cases, "cases", "id", "cases", &mut errors);
    let role_ids = ids(roles, "roles", "id", "roles", &mut errors);
    let copy_keys = ids(copy, "states", "key", "copy states", &mut errors);
    require_ids(&case_ids, REQUIRED_CASES, "cases", &mut errors);
    require_ids(&role_ids, REQUIRED_ROLES, "roles", &mut errors);
    require_ids(&copy_keys, REQUIRED_COPY, "copy states", &mut errors);

    let invariants = string_set(cases.get("shared_invariants"));
    for invariant in REQUIRED_INVARIANTS {
        if !invariants.contains(*invariant) {
            errors.push(format!("shared_invariants: missing required invariant {invariant}"));
        }
    }

    for (case_id, request_state, visible_state) in REQUIRED_CASE_STATES {
        match find(cases, "cases", "id", case_id) {
            Some(case) => {
                let actual_request = case.pointer("/expected/request_state").and_then(Value::as_str);
                let actual_visible = case.pointer("/expected/visible_state").and_then(Value::as_str);
                if actual_request != Some(*request_state) {
                    errors.push(format!("{case_id}: expected request_state {request_state:?}, found {actual_request:?}"));
                }
                if actual_visible != Some(*visible_state) {
                    errors.push(format!("{case_id}: expected visible_state {visible_state:?}, found {actual_visible:?}"));
                }
            }
            None => errors.push(format!("{case_id}: pinned v1 state contract cannot be checked because the case is missing")),
        }
    }

    for (case_id, forbidden_label) in REQUIRED_FORBIDDEN_LABELS {
        match find(cases, "cases", "id", case_id) {
            Some(case) => {
                let forbidden = string_set(case.pointer("/expected/must_not_show"));
                if !forbidden.contains(*forbidden_label) {
                    errors.push(format!("{case_id}: must_not_show must retain {forbidden_label:?}"));
                }
            }
            None => errors.push(format!("{case_id}: cannot check forbidden label {forbidden_label:?}; case is missing")),
        }
    }

    for (copy_key, forbidden_implication) in REQUIRED_COPY_NONIMPLICATIONS {
        match find(copy, "states", "key", copy_key) {
            Some(state) => {
                let forbidden = string_set(state.get("must_not_imply"));
                if !forbidden.contains(*forbidden_implication) {
                    errors.push(format!("{copy_key}: must_not_imply must retain {forbidden_implication:?}"));
                }
            }
            None => errors.push(format!("{copy_key}: cannot check non-implication {forbidden_implication:?}; copy state is missing")),
        }
    }

    for (idx, case) in array(cases, "cases").iter().enumerate() {
        let label = case.get("id").and_then(Value::as_str).unwrap_or("<missing-id>");
        let show = string_set(case.pointer("/expected/must_show"));
        let not_show = string_set(case.pointer("/expected/must_not_show"));
        for overlap in show.intersection(&not_show) {
            errors.push(format!("{label}: contradictory visibility contract for {overlap:?}"));
        }
        if show.is_empty() {
            errors.push(format!("cases[{idx}] {label}: must_show must not be empty"));
        }
        if string_set(case.pointer("/expected/invariant_checks")).is_empty() {
            errors.push(format!("cases[{idx}] {label}: invariant_checks must not be empty"));
        }
    }

    let expected: &[(&str, &str, i64)] = &[
        ("FUX-OFFLINE-001", "bookings", 0), ("FUX-OFFLINE-001", "payments", 0),
        ("FUX-QUOTE-002", "bookings", 0), ("FUX-QUOTE-002", "payments", 0),
        ("FUX-QUOTE-003", "bookings", 0), ("FUX-QUOTE-003", "payments", 0),
        ("FUX-SYNC-004", "duplicate_bookings", 0), ("FUX-ROUTE-005", "bookings", 0),
        ("FUX-NOPROVIDER-006", "bookings", 0), ("FUX-NOPROVIDER-006", "payments", 0),
        ("FUX-COMPLETION-007", "accepted_services", 0), ("FUX-COMPLETION-007", "payments", 0),
        ("FUX-REWORK-008", "accepted_services", 0), ("FUX-CONSENT-010", "new_shares_after_revocation", 0),
        ("FUX-PAYMENT-012", "payments", 0), ("FUX-AUTH-013", "service_requests", 0),
        ("FUX-AUTH-013", "bookings", 0), ("FUX-AUTH-013", "payments", 0),
        ("FUX-ROUTE-015", "bookings", 0), ("FUX-PRIVACY-016", "precise_location_disclosures", 0),
        ("FUX-DIGEST-017", "verified_source_claims", 0), ("FUX-CAPACITY-019", "bookings", 0),
        ("FUX-CAPACITY-019", "payments", 0), ("FUX-EXPORT-020", "events_silently_omitted", 0),
        ("FUX-DEFER-021", "unauthorized_actions", 0), ("FUX-NOSHOW-022", "accepted_services", 0),
        ("FUX-NOSHOW-022", "request_denominator_entries", 1),
    ];
    for (case_id, field, wanted) in expected {
        check_count(cases, case_id, field, *wanted, &mut errors);
    }

    for id in ["farmer_smallholder_crop", "extension_officer_agronomist", "irrigation_water_service_provider"] {
        let priority = find(roles, "roles", "id", id).and_then(|v| v.get("priority")).and_then(Value::as_str);
        if priority != Some("P0") {
            errors.push(format!("{id}: expected initial priority P0, found {priority:?}"));
        }
    }
    errors.sort();
    errors.dedup();
    errors
}
fn artifact_report(a: &Artifact) -> ArtifactReport {
    let dialect = a.schema.get("$schema").and_then(Value::as_str).unwrap_or("missing").to_owned();
    ArtifactReport {
        name: a.name.to_owned(),
        instance_path: a.instance_path.to_owned(),
        schema_path: a.schema_path.to_owned(),
        instance_sha256: sha256(&a.instance_bytes),
        schema_sha256: sha256(&a.schema_bytes),
        instance_bytes: a.instance_bytes.len(),
        schema_bytes: a.schema_bytes.len(),
        schema_dialect: dialect,
        schema_status: if a.schema_errors.is_empty() { "PASS" } else { "FAIL" }.to_owned(),
        schema_errors: a.schema_errors.clone(),
    }
}
fn default_root() -> Result<PathBuf, String> {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).parent().and_then(Path::parent).map(Path::to_path_buf)
        .ok_or_else(|| "cannot derive repository root from CARGO_MANIFEST_DIR; use --root".to_owned())
}
fn parse_args() -> Result<(PathBuf, Option<PathBuf>, bool), String> {
    let mut root = None;
    let mut report_path = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--root" => root = Some(PathBuf::from(args.next().ok_or("--root requires a path")?)),
            "--report-json" => report_path = Some(PathBuf::from(args.next().ok_or("--report-json requires a path")?)),
            "--help" | "-h" => return Ok((PathBuf::new(), None, true)),
            _ => return Err(format!("unknown argument {arg:?}; use --help")),
        }
    }
    Ok((root.unwrap_or(default_root()?), report_path, false))
}
fn usage() -> &'static str {
    "Usage: farmer-ux-contract-verifier [--root REPOSITORY_ROOT] [--report-json OUTPUT_PATH]\n\
     Validates local FARMER-UX-001 schemas and pinned v1 corpus invariants.\n\
     It performs no network access and makes no UI or field-impact claim."
}
fn execute(root: &Path) -> Result<Report, String> {
    let artifacts = vec![
        load(root, "cases", CASES, CASES_SCHEMA)?,
        load(root, "roles", ROLES, ROLES_SCHEMA)?,
        load(root, "copy", COPY, COPY_SCHEMA)?,
    ];
    let mut schema_errors = Vec::new();
    for artifact in &artifacts { schema_errors.extend(artifact.schema_errors.clone()); }
    schema_errors.sort();
    let semantic_errors = semantic_checks(&artifacts[0].instance, &artifacts[1].instance, &artifacts[2].instance);
    let mut errors = schema_errors.clone();
    errors.extend(semantic_errors.clone());
    errors.sort();
    errors.dedup();

    let counts = Counts {
        cases: array(&artifacts[0].instance, "cases").len(),
        roles: array(&artifacts[1].instance, "roles").len(),
        copy_states: array(&artifacts[2].instance, "states").len(),
        shared_invariants: string_set(artifacts[0].instance.get("shared_invariants")).len(),
    };
    Ok(Report {
        report_version: REPORT_VERSION.to_owned(),
        tool_version: env!("CARGO_PKG_VERSION").to_owned(),
        validator: VALIDATOR.to_owned(),
        status: if errors.is_empty() { "PASS" } else { "FAIL" }.to_owned(),
        artifacts: artifacts.iter().map(artifact_report).collect(),
        counts,
        schema_error_count: schema_errors.len(),
        semantic_error_count: semantic_errors.len(),
        errors,
        claim_ceiling: CLAIM_CEILING.to_owned(),
    })
}
fn main() -> ExitCode {
    let (root, report_path, help) = match parse_args() {
        Ok(args) => args,
        Err(error) => {
            eprintln!("{error}\n{}", usage());
            return ExitCode::from(2);
        }
    };
    if help {
        println!("{}", usage());
        return ExitCode::SUCCESS;
    }
    let report = match execute(&root) {
        Ok(report) => report,
        Err(error) => {
            eprintln!("FARMER-UX-002: {error}");
            return ExitCode::from(2);
        }
    };
    let output = match serde_json::to_string_pretty(&report) {
        Ok(value) => format!("{value}\n"),
        Err(error) => {
            eprintln!("cannot serialize report: {error}");
            return ExitCode::from(2);
        }
    };
    if let Some(path) = report_path {
        if let Err(error) = fs::write(&path, output.as_bytes()) {
            eprintln!("cannot write report {}: {error}", path.display());
            return ExitCode::from(2);
        }
    }
    print!("{output}");
    if report.status == "PASS" { ExitCode::SUCCESS } else { ExitCode::from(1) }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn parse(s: &str) -> Value { serde_json::from_str(s).expect("fixture should parse") }
    fn cases() -> Value { parse(include_str!("../../farmer-ux-001-cases-v1.json")) }
    fn case_schema() -> Value { parse(include_str!("../../farmer-ux-001-cases-v1.schema.json")) }
    fn roles() -> Value { parse(include_str!("../../farmer-ux-001-role-task-matrix-v1.json")) }
    fn role_schema() -> Value { parse(include_str!("../../farmer-ux-001-role-task-matrix-v1.schema.json")) }
    fn copy() -> Value { parse(include_str!("../../farmer-ux-001-copy-v1.json")) }
    fn copy_schema() -> Value { parse(include_str!("../../farmer-ux-001-copy-v1.schema.json")) }

    #[test]
    fn canonical_corpus_passes_schema_and_semantic_checks() {
        let c = cases(); let r = roles(); let p = copy();
        assert!(validate_schema("cases", &c, &case_schema()).is_empty());
        assert!(validate_schema("roles", &r, &role_schema()).is_empty());
        assert!(validate_schema("copy", &p, &copy_schema()).is_empty());
        assert!(semantic_checks(&c, &r, &p).is_empty());
    }
    #[test]
    fn malformed_schema_document_is_rejected() {
        let mut schema = case_schema();
        schema["type"] = Value::String("not-a-json-schema-type".to_owned());
        let errors = validate_schema("mutated schema", &cases(), &schema);
        assert!(errors.iter().any(|e| e.contains("invalid JSON Schema document")));
    }
    #[test]
    fn duplicate_case_id_is_rejected() {
        let mut c = cases();
        let first = c["cases"][0].clone();
        c["cases"].as_array_mut().expect("array").push(first);
        assert!(semantic_checks(&c, &roles(), &copy()).iter().any(|e| e.contains("duplicate id")));
    }
    #[test]
    fn contradictory_visibility_is_rejected() {
        let mut c = cases();
        let shown = c["cases"][0]["expected"]["must_show"][0].as_str().expect("string").to_owned();
        c["cases"][0]["expected"]["must_not_show"].as_array_mut().expect("array").push(Value::String(shown));
        assert!(semantic_checks(&c, &roles(), &copy()).iter().any(|e| e.contains("contradictory visibility")));
    }
    #[test]
    fn deleting_a_required_risk_case_is_rejected() {
        let mut c = cases();
        c["cases"].as_array_mut().expect("array").retain(|case| case["id"].as_str() != Some("FUX-DIGEST-017"));
        assert!(semantic_checks(&c, &roles(), &copy()).iter().any(|e| e.contains("FUX-DIGEST-017")));
    }
    #[test]
    fn expired_quote_cannot_create_a_booking() {
        let mut c = cases();
        c["cases"].as_array_mut().expect("array").iter_mut()
            .find(|case| case["id"].as_str() == Some("FUX-QUOTE-002")).expect("case")
            ["expected"]["side_effect_counts"]["bookings"] = Value::from(1);
        assert!(semantic_checks(&c, &roles(), &copy()).iter()
            .any(|e| e.contains("FUX-QUOTE-002") && e.contains("side_effect_counts.bookings=0")));
    }
    #[test]
    fn frozen_mutation_corpus_is_rejected_for_expected_reasons() {
        let mutation_document = parse(include_str!("../mutations-v1.json"));
        let mutation_schema = parse(include_str!("../mutations-v1.schema.json"));
        assert!(
            validate_schema("mutations", &mutation_document, &mutation_schema).is_empty(),
            "frozen mutation corpus and its schema must validate"
        );
        let corpus: MutationCorpus = serde_json::from_value(mutation_document)
            .expect("frozen mutation corpus must deserialize");
        assert_eq!(corpus.schema_version, "1.0.0");
        assert_eq!(corpus.mutations.len(), 12);
        let mutation_ids: BTreeSet<&str> = corpus.mutations.iter().map(|mutation| mutation.id.as_str()).collect();
        assert_eq!(mutation_ids.len(), corpus.mutations.len(), "mutation IDs must be unique");

        for mutation in &corpus.mutations {
            let mut c = cases();
            let mut r = roles();
            let mut p = copy();
            apply_mutation(&mut c, &mut r, &mut p, mutation)
                .unwrap_or_else(|error| panic!("{}: {error}", mutation.id));
            let errors = semantic_checks(&c, &r, &p);
            assert!(
                errors.iter().any(|error| error.contains(&mutation.expected_error_contains)),
                "{}: expected error containing {:?}, got: {:?}",
                mutation.id,
                mutation.expected_error_contains,
                errors
            );
        }
    }

    #[test]
    fn sha256_matches_standard_test_vector() {
        assert_eq!(
            sha256(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }
    #[test]
    fn expired_quote_state_drift_is_rejected() {
        let mut c = cases();
        c["cases"].as_array_mut().expect("array").iter_mut()
            .find(|case| case["id"].as_str() == Some("FUX-QUOTE-002")).expect("case")
            ["expected"]["request_state"] = Value::String("booking_confirmed".to_owned());
        assert!(semantic_checks(&c, &roles(), &copy()).iter()
            .any(|e| e.contains("FUX-QUOTE-002") && e.contains("request_state")));
    }
    #[test]
    fn critical_forbidden_ui_label_cannot_be_removed() {
        let mut c = cases();
        c["cases"].as_array_mut().expect("array").iter_mut()
            .find(|case| case["id"].as_str() == Some("FUX-DIGEST-017")).expect("case")
            ["expected"]["must_not_show"].as_array_mut().expect("array")
            .retain(|label| label.as_str() != Some("source_authenticated"));
        assert!(semantic_checks(&c, &roles(), &copy()).iter()
            .any(|e| e.contains("FUX-DIGEST-017") && e.contains("source_authenticated")));
    }
    #[test]
    fn outcome_unknown_copy_must_not_imply_success_failure_or_safe_retry() {
        let mut p = copy();
        p["states"].as_array_mut().expect("array").iter_mut()
            .find(|state| state["key"].as_str() == Some("outcome_unknown")).expect("state")
            ["must_not_imply"].as_array_mut().expect("array")
            .retain(|label| label.as_str() != Some("automatic_retry_is_safe"));
        assert!(semantic_checks(&cases(), &roles(), &p).iter()
            .any(|e| e.contains("outcome_unknown") && e.contains("automatic_retry_is_safe")));
    }

    #[test]
    fn copy_cannot_claim_a_booking_is_completed_or_paid() {
        let mut p = copy();
        p["states"].as_array_mut().expect("array").iter_mut()
            .find(|state| state["key"].as_str() == Some("booking_confirmed")).expect("state")
            ["must_not_imply"].as_array_mut().expect("array")
            .retain(|label| label.as_str() != Some("payment_completed"));
        assert!(semantic_checks(&cases(), &roles(), &p).iter()
            .any(|e| e.contains("booking_confirmed") && e.contains("payment_completed")));
    }

    #[test]
    fn digest_syntax_cannot_become_source_authentication() {
        let mut c = cases();
        c["cases"].as_array_mut().expect("array").iter_mut()
            .find(|case| case["id"].as_str() == Some("FUX-DIGEST-017")).expect("case")
            ["expected"]["side_effect_counts"]["verified_source_claims"] = Value::from(1);
        assert!(semantic_checks(&c, &roles(), &copy()).iter()
            .any(|e| e.contains("FUX-DIGEST-017") && e.contains("side_effect_counts.verified_source_claims=0")));
    }
}

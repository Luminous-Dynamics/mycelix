use jsonschema::validator_for;
use serde::Serialize;
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

const DIALECT: &str = "https://json-schema.org/draft/2020-12/schema";
const CLAIM_CEILING: &str = "Reference model and synthetic-fixture verification only. Does not prove production persistence, actor identity, actual consent, server idempotency, provider delivery, usability, service fulfilment, agronomic safety, or field impact.";
pub const CASES_PATH: &str = "research/farmer-ux-006-channel-cases-v1.json";
pub const CASES_SCHEMA_PATH: &str = "research/farmer-ux-006-channel-cases-v1.schema.json";
pub const ENVELOPE_PATH: &str = "research/farmer-ux-006-channel-envelope-example-v1.json";
pub const ENVELOPE_SCHEMA_PATH: &str = "research/farmer-ux-006-channel-envelope-v1.schema.json";
pub const SCHEDULE_COUNT: u64 = 10_000;

const REQUIRED_CASES: &[&str] = &[
    "FUXC-001", "FUXC-002", "FUXC-003", "FUXC-004", "FUXC-005",
    "FUXC-006", "FUXC-007", "FUXC-008", "FUXC-009", "FUXC-010",
    "FUXC-011", "FUXC-012", "FUXC-013", "FUXC-014", "FUXC-015",
    "FUXC-016", "FUXC-017",
];

#[derive(Debug, Serialize)]
pub struct ArtifactCheck {
    pub path: String,
    pub sha256: String,
    pub schema_path: String,
    pub schema_sha256: String,
    pub schema_dialect: String,
    pub schema_status: String,
    pub errors: Vec<String>,
}

#[derive(Debug, Serialize)]
pub struct ScheduleReport {
    pub schedules_executed: u64,
    pub invariant_violations: Vec<String>,
}

#[derive(Debug, Serialize)]
pub struct DependencyLockEvidence {
    pub path: String,
    pub sha256: String,
}

#[derive(Debug, Serialize)]
pub struct OracleReport {
    pub version: String,
    pub status: String,
    pub case_count: usize,
    pub artifact_checks: Vec<ArtifactCheck>,
    pub semantic_errors: Vec<String>,
    pub adversarial_schedules: ScheduleReport,
    pub dependency_lock: Option<DependencyLockEvidence>,
    pub claim_ceiling: String,
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

fn read_json(root: &Path, path: &str) -> Result<(Vec<u8>, Value), String> {
    let bytes = fs::read(root.join(path)).map_err(|e| format!("cannot read {path}: {e}"))?;
    let value = serde_json::from_slice(&bytes)
        .map_err(|e| format!("cannot parse {path}: {e}"))?;
    Ok((bytes, value))
}

fn validate_schema(instance: &Value, schema: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if schema.get("$schema").and_then(Value::as_str) != Some(DIALECT) {
        errors.push("unsupported or missing JSON Schema dialect".to_owned());
    }
    if let Err(error) = jsonschema::meta::validate(schema) {
        errors.push(format!("invalid JSON Schema document: {error}"));
    }
    match validator_for(schema) {
        Ok(validator) => {
            for error in validator.iter_errors(instance) {
                errors.push(format!("schema violation at {}: {error}", error.instance_path()));
            }
        }
        Err(error) => errors.push(format!("schema compilation failed: {error}")),
    }
    errors.sort();
    errors
}

fn artifact_check(
    path: &str,
    schema_path: &str,
    data: &Value,
    schema: &Value,
    data_bytes: &[u8],
    schema_bytes: &[u8],
) -> ArtifactCheck {
    let errors = validate_schema(data, schema);
    ArtifactCheck {
        path: path.to_owned(),
        sha256: sha256(data_bytes),
        schema_path: schema_path.to_owned(),
        schema_sha256: sha256(schema_bytes),
        schema_dialect: schema
            .get("$schema")
            .and_then(Value::as_str)
            .unwrap_or("missing")
            .to_owned(),
        schema_status: if errors.is_empty() { "PASS" } else { "FAIL" }.to_owned(),
        errors,
    }
}

fn find_case<'a>(document: &'a Value, id: &str) -> Option<&'a Value> {
    document
        .get("cases")?
        .as_array()?
        .iter()
        .find(|case| case.get("case_id").and_then(Value::as_str) == Some(id))
}

fn has_string(case: &Value, field: &str, expected: &str) -> bool {
    case.get(field).and_then(Value::as_str) == Some(expected)
}

fn has_effect(case: &Value, field: &str, expected: &str) -> bool {
    case.get(field)
        .and_then(Value::as_array)
        .is_some_and(|items| items.iter().any(|item| item.as_str() == Some(expected)))
}

fn has_same_two_strings(case: &Value, field: &str) -> bool {
    case.get(field)
        .and_then(Value::as_array)
        .is_some_and(|items| {
            items.len() == 2
                && items[0].as_str().is_some()
                && items[0].as_str() == items[1].as_str()
        })
}

fn has_different_two_strings(case: &Value, field: &str) -> bool {
    case.get(field)
        .and_then(Value::as_array)
        .is_some_and(|items| {
            items.len() == 2
                && items[0].as_str().is_some()
                && items[1].as_str().is_some()
                && items[0].as_str() != items[1].as_str()
        })
}

fn required_case_rule(
    _document: &Value,
    errors: &mut Vec<String>,
    id: &str,
    description: &str,
    holds: bool,
) {
    if !holds {
        errors.push(format!("{id}: missing or weakened invariant: {description}"));
    }
}

pub fn semantic_errors(document: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    let Some(cases) = document.get("cases").and_then(Value::as_array) else {
        return vec!["cases must be an array".to_owned()];
    };

    let mut seen = BTreeSet::new();
    for case in cases {
        let Some(id) = case.get("case_id").and_then(Value::as_str) else {
            errors.push("case without a string case_id".to_owned());
            continue;
        };
        if !seen.insert(id.to_owned()) {
            errors.push(format!("duplicate case_id: {id}"));
        }
    }
    if cases.len() != REQUIRED_CASES.len() {
        errors.push(format!(
            "FARMER-UX-006 v1 requires exactly {} frozen cases; found {}",
            REQUIRED_CASES.len(),
            cases.len()
        ));
    }
    for id in REQUIRED_CASES {
        if !seen.contains(*id) {
            errors.push(format!("required conformance case missing: {id}"));
        }
    }

    for id in REQUIRED_CASES {
        let Some(case) = find_case(document, id) else {
            continue;
        };
        match *id {
            "FUXC-001" => {
                required_case_rule(document, &mut errors, id, "one logical request survives app-to-cooperative handoff",
                    has_string(case, "expected", "same_logical_request")
                        && has_same_two_strings(case, "request_refs")
                        && has_effect(case, "prohibited", "create_second_request"));
            }
            "FUXC-002" => {
                required_case_rule(document, &mut errors, id, "same event ID and same digest are idempotent",
                    has_string(case, "expected", "idempotent_duplicate")
                        && has_same_two_strings(case, "event_ids")
                        && has_same_two_strings(case, "request_refs")
                        && case.get("digests_equal").and_then(Value::as_bool) == Some(true)
                        && has_effect(case, "prohibited", "apply_effect_twice"));
            }
            "FUXC-003" => {
                required_case_rule(document, &mut errors, id, "same event ID with a different digest is quarantined, not overwritten",
                    has_string(case, "expected", "event_id_equivocation_conflict")
                        && has_same_two_strings(case, "event_ids")
                        && case.get("digests_equal").and_then(Value::as_bool) == Some(false)
                        && has_effect(case, "prohibited", "overwrite_original")
                        && has_effect(case, "allowed_effects", "quarantine_and_record_conflict"));
            }
            "FUXC-004" => {
                required_case_rule(document, &mut errors, id, "unverified intermediary authority cannot cause onward payload disclosure",
                    has_string(case, "authority", "unverified_relay")
                        && case.get("actor").and_then(Value::as_str) != case.get("recorder").and_then(Value::as_str)
                        && has_effect(case, "prohibited", "forward_sensitive_payload"));
            }
            "FUXC-005" => {
                required_case_rule(document, &mut errors, id, "relay is not provider receipt, quote, or booking",
                    has_string(case, "expected", "relayed_not_provider_acknowledged")
                        && has_effect(case, "prohibited", "promote_to_provider_receipt")
                        && has_effect(case, "prohibited", "promote_to_booking"));
            }
            "FUXC-006" => {
                required_case_rule(document, &mut errors, id, "late provider receipt attaches to the same request",
                    has_same_two_strings(case, "request_refs")
                        && has_effect(case, "prohibited", "create_new_request")
                        && has_effect(case, "prohibited", "book_without_quote_acceptance"));
            }
            "FUXC-007" => {
                required_case_rule(document, &mut errors, id, "out-of-order events remain uncertain until reconciliation",
                    has_string(case, "expected", "retain_order_uncertainty_and_block_illegal_promotion")
                        && has_effect(case, "prohibited", "silently_reorder_into_valid_booking"));
            }
            "FUXC-008" => {
                required_case_rule(document, &mut errors, id, "revoked consent blocks new disclosure and automatic forwarding retry",
                    has_string(case, "authority", "consent_revoked_before_forward")
                        && has_effect(case, "prohibited", "forward_payload")
                        && has_effect(case, "prohibited", "automatic_retry"));
            }
            "FUXC-009" => {
                required_case_rule(document, &mut errors, id, "a submitted request cannot be silently retargeted",
                    has_string(case, "expected", "preserve_original_scope_and_block_retarget")
                        && has_effect(case, "prohibited", "retarget_existing_request"));
            }
            "FUXC-010" => {
                required_case_rule(document, &mut errors, id, "device timestamp alone cannot promote authoritative order",
                    has_string(case, "time_source", "device_clock")
                        && has_effect(case, "prohibited", "promote_by_client_timestamp_alone"));
            }
            "FUXC-011" => {
                required_case_rule(document, &mut errors, id, "claimed paper time is not server receipt or verified time",
                    has_string(case, "time_source", "paper_claim")
                        && has_effect(case, "prohibited", "backdate_server_receipt")
                        && has_effect(case, "prohibited", "present_claimed_time_as_verified"));
            }
            "FUXC-012" => {
                required_case_rule(document, &mut errors, id, "no-provider outcomes remain in the denominator",
                    has_string(case, "expected", "no_provider_outcome_remains_in_denominator")
                        && has_effect(case, "prohibited", "drop_request_from_report"));
            }
            "FUXC-013" => {
                required_case_rule(document, &mut errors, id, "provider-reported completion is not farmer acceptance",
                    has_string(case, "expected", "provider_reported_not_farmer_accepted")
                        && has_effect(case, "prohibited", "count_as_accepted_service"));
            }
            "FUXC-014" => {
                required_case_rule(document, &mut errors, id, "farmer dispute remains visible and is never force-accepted",
                    has_string(case, "expected", "dispute_remains_visible")
                        && has_effect(case, "prohibited", "erase_provider_claim")
                        && has_effect(case, "prohibited", "force_acceptance"));
            }
            "FUXC-015" => {
                required_case_rule(document, &mut errors, id, "unknown remote outcome stays unknown and is not automatically resent",
                    has_string(case, "expected", "preserve_unknown_without_auto_retry")
                        && has_effect(case, "prohibited", "claim_failure")
                        && has_effect(case, "prohibited", "claim_success")
                        && has_effect(case, "prohibited", "resend_automatically"));
            }
            "FUXC-016" => {
                required_case_rule(document, &mut errors, id, "correction appends and preserves history",
                    has_string(case, "expected", "append_correction_preserve_history")
                        && has_effect(case, "prohibited", "rewrite_original_event")
                        && has_effect(case, "prohibited", "silently_promote_relay_to_authoritative_quote"));
            }
            "FUXC-017" => {
                required_case_rule(document, &mut errors, id, "similar requests with distinct references are linked for review but never auto-merged or dropped",
                    has_string(case, "expected", "suspected_duplicate_link_not_merge")
                        && has_different_two_strings(case, "request_refs")
                        && has_effect(case, "prohibited", "auto_merge_requests")
                        && has_effect(case, "prohibited", "drop_one_request_from_denominator")
                        && has_effect(case, "prohibited", "copy_remote_receipt_across_requests"));
            }
            _ => {}
        }
    }
    errors.sort();
    errors.dedup();
    errors
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvidenceKind {
    FarmerReported,
    Relayed,
    ProviderReceipt,
    QuoteOffered,
    BookingConfirmed,
    ProviderReportedComplete,
    FarmerAcceptance,
    FarmerDispute,
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClaimType {
    FarmerReported,
    Relayed,
    ProviderReceipt,
    QuoteOffered,
    BookingConfirmed,
    ProviderReportedComplete,
    FarmerAccepted,
    DisputedOrRework,
    OutcomeUnknown,
}

/// A channel adapter's claim ceiling, not the service lifecycle evaluator.
/// Each evidence class can support only the matching claim class here.
/// Quote validity, booking acceptance, scheduling and fulfilment remain owned
/// by the canonical Symtropy evaluator.
pub fn evidence_can_support_claim(evidence: EvidenceKind, claim: ClaimType) -> bool {
    matches!(
        (evidence, claim),
        (EvidenceKind::FarmerReported, ClaimType::FarmerReported)
            | (EvidenceKind::Relayed, ClaimType::Relayed)
            | (EvidenceKind::ProviderReceipt, ClaimType::ProviderReceipt)
            | (EvidenceKind::QuoteOffered, ClaimType::QuoteOffered)
            | (EvidenceKind::BookingConfirmed, ClaimType::BookingConfirmed)
            | (
                EvidenceKind::ProviderReportedComplete,
                ClaimType::ProviderReportedComplete
            )
            | (EvidenceKind::FarmerAcceptance, ClaimType::FarmerAccepted)
            | (EvidenceKind::FarmerDispute, ClaimType::DisputedOrRework)
            | (EvidenceKind::OutcomeUnknown, ClaimType::OutcomeUnknown)
    )
}

pub fn forwarding_allowed(authority_verified: bool, consent_current: bool) -> bool {
    authority_verified && consent_current
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Event {
    pub event_id: String,
    pub request_ref: String,
    pub digest: String,
    pub asserting_role: String,
    pub recording_role: String,
    pub evidence: EvidenceKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IngestResult {
    Applied,
    Duplicate,
    Conflict,
}

#[derive(Debug, Default)]
pub struct EventLedger {
    events: BTreeMap<String, Event>,
    applied_event_effects: BTreeSet<String>,
    request_refs: BTreeSet<String>,
    conflicting_events: BTreeMap<String, Vec<Event>>,
    duplicate_deliveries: usize,
}

impl EventLedger {
    pub fn ingest(&mut self, event: Event) -> IngestResult {
        if let Some(previous) = self.events.get(&event.event_id) {
            if previous == &event {
                self.duplicate_deliveries += 1;
                return IngestResult::Duplicate;
            }
            // The event identifier is already bound. A differing envelope must
            // not replace the first event even when payload digests happen to match.
            self.conflicting_events
                .entry(event.event_id.clone())
                .or_default()
                .push(event);
            return IngestResult::Conflict;
        }
        self.request_refs.insert(event.request_ref.clone());
        self.applied_event_effects.insert(event.event_id.clone());
        self.events.insert(event.event_id.clone(), event);
        IngestResult::Applied
    }

    pub fn stored(&self, event_id: &str) -> Option<&Event> {
        self.events.get(event_id)
    }

    pub fn applied_effect_count(&self) -> usize {
        self.applied_event_effects.len()
    }

    pub fn request_count(&self) -> usize {
        self.request_refs.len()
    }

    pub fn conflict_count(&self) -> usize {
        self.conflicting_events.values().map(|events| events.len()).sum()
    }

    pub fn conflicting_events(&self, event_id: &str) -> &[Event] {
        self.conflicting_events
            .get(event_id)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    pub fn duplicate_delivery_count(&self) -> usize {
        self.duplicate_deliveries
    }
}

fn synthetic_event(seed: u64, index: usize) -> Event {
    let evidence = match index % 4 {
        0 => EvidenceKind::Relayed,
        1 => EvidenceKind::OutcomeUnknown,
        2 => EvidenceKind::ProviderReportedComplete,
        _ => EvidenceKind::FarmerReported,
    };
    Event {
        event_id: format!("evt-{seed:05}-{index:02}"),
        request_ref: format!("req-{seed:05}-{index:02}"),
        digest: format!("canonical-digest-{seed}-{index}"),
        asserting_role: if index % 2 == 0 { "farmer" } else { "provider" }.to_owned(),
        recording_role: if index % 3 == 0 { "extension_worker" } else { "participant" }.to_owned(),
        evidence,
    }
}

/// Generates 10,000 deterministic schedules. Each schedule mixes reordering,
/// exact replay and a same-event-ID/different-digest conflict.
pub fn run_deterministic_schedules() -> ScheduleReport {
    let mut violations = Vec::new();
    for seed in 0..SCHEDULE_COUNT {
        let mut events: Vec<Event> = (0..8).map(|index| synthetic_event(seed, index)).collect();
        let duplicate_index = (seed as usize) % 8;
        let conflict_index = (duplicate_index + 3) % 8;
        events.push(events[duplicate_index].clone());
        let mut conflict = events[conflict_index].clone();
        conflict.digest = format!("conflicting-digest-{seed}-{conflict_index}");
        events.push(conflict);

        // Fisher-Yates with a fixed LCG: reproducible without a random dependency.
        let mut random = seed.wrapping_add(1);
        for i in (1..events.len()).rev() {
            random = random.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            let j = (random as usize) % (i + 1);
            events.swap(i, j);
        }

        let mut ledger = EventLedger::default();
        let mut first_digest_by_id = BTreeMap::new();
        let mut applied = 0usize;
        let mut duplicates = 0usize;
        let mut conflicts = 0usize;
        for event in events {
            first_digest_by_id
                .entry(event.event_id.clone())
                .or_insert_with(|| event.digest.clone());
            match ledger.ingest(event) {
                IngestResult::Applied => applied += 1,
                IngestResult::Duplicate => duplicates += 1,
                IngestResult::Conflict => conflicts += 1,
            }
        }

        let mut reasons = Vec::new();
        if applied != 8 || ledger.applied_effect_count() != 8 {
            reasons.push(format!("expected 8 applied event effects, observed {applied}"));
        }
        if duplicates != 1 || ledger.duplicate_delivery_count() != 1 {
            reasons.push(format!("expected 1 exact replay, observed {duplicates}"));
        }
        if conflicts != 1 || ledger.conflict_count() != 1 {
            reasons.push(format!("expected 1 quarantined conflict, observed {conflicts}"));
        }
        if ledger.request_count() != 8 {
            reasons.push(format!("expected 8 distinct requests, observed {}", ledger.request_count()));
        }
        for (event_id, expected_digest) in first_digest_by_id {
            if ledger.stored(&event_id).map(|event| event.digest.as_str()) != Some(expected_digest.as_str()) {
                reasons.push(format!("first-seen event was silently overwritten: {event_id}"));
            }
        }
        if evidence_can_support_claim(EvidenceKind::Relayed, ClaimType::BookingConfirmed) {
            reasons.push("relay evidence was promoted to booking".to_owned());
        }
        if forwarding_allowed(true, false) || forwarding_allowed(false, true) {
            reasons.push("forwarding allowed without both authority and current consent".to_owned());
        }
        if !reasons.is_empty() && violations.len() < 100 {
            violations.push(format!("schedule {seed}: {}", reasons.join("; ")));
        }
    }
    ScheduleReport {
        schedules_executed: SCHEDULE_COUNT,
        invariant_violations: violations,
    }
}

pub fn verify_repository(root: &Path) -> Result<OracleReport, String> {
    let dependency_lock_path = root.join("research/farmer-ux-006-channel-oracle/Cargo.lock");
    let dependency_lock = match fs::read(&dependency_lock_path) {
        Ok(bytes) => Some(DependencyLockEvidence {
            path: "research/farmer-ux-006-channel-oracle/Cargo.lock".to_owned(),
            sha256: sha256(&bytes),
        }),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => None,
        Err(error) => {
            return Err(format!(
                "cannot read dependency lockfile {}: {error}",
                dependency_lock_path.display()
            ));
        }
    };
    let (case_bytes, cases) = read_json(root, CASES_PATH)?;
    let (case_schema_bytes, case_schema) = read_json(root, CASES_SCHEMA_PATH)?;
    let (envelope_bytes, envelope) = read_json(root, ENVELOPE_PATH)?;
    let (envelope_schema_bytes, envelope_schema) = read_json(root, ENVELOPE_SCHEMA_PATH)?;

    let artifact_checks = vec![
        artifact_check(
            CASES_PATH,
            CASES_SCHEMA_PATH,
            &cases,
            &case_schema,
            &case_bytes,
            &case_schema_bytes,
        ),
        artifact_check(
            ENVELOPE_PATH,
            ENVELOPE_SCHEMA_PATH,
            &envelope,
            &envelope_schema,
            &envelope_bytes,
            &envelope_schema_bytes,
        ),
    ];
    let case_count = cases.get("cases").and_then(Value::as_array).map_or(0, Vec::len);
    let mut errors = semantic_errors(&cases);
    let schedules = run_deterministic_schedules();
    errors.sort();
    let schema_failed = artifact_checks
        .iter()
        .any(|artifact| artifact.schema_status != "PASS");
    let status = if schema_failed || !errors.is_empty() || !schedules.invariant_violations.is_empty() {
        "FAIL"
    } else {
        "PASS"
    };
    Ok(OracleReport {
        version: "1.0.0".to_owned(),
        status: status.to_owned(),
        case_count,
        artifact_checks,
        semantic_errors: errors,
        adversarial_schedules: schedules,
        dependency_lock,
        claim_ceiling: CLAIM_CEILING.to_owned(),
    })
}

pub fn find_repository_root() -> Result<PathBuf, String> {
    let mut current = std::env::current_dir().map_err(|e| e.to_string())?;
    loop {
        if current.join(CASES_PATH).is_file() && current.join(CASES_SCHEMA_PATH).is_file() {
            return Ok(current);
        }
        if !current.pop() {
            return Err(format!("could not find {CASES_PATH} walking up from the current directory"));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture_document() -> Value {
        let root = find_repository_root().expect("repository root");
        let (_, value) = read_json(&root, CASES_PATH).expect("fixture document");
        value
    }

    #[test]
    fn schemas_semantics_and_schedule_model_pass() {
        let root = find_repository_root().expect("repository root");
        let report = verify_repository(&root).expect("read conformance fixtures");
        assert_eq!(report.status, "PASS", "report: {}", serde_json::to_string_pretty(&report).unwrap());
        assert_eq!(report.case_count, 17);
        assert_eq!(report.artifact_checks.len(), 2);
        assert!(report.artifact_checks.iter().all(|item| item.schema_status == "PASS"));
        assert_eq!(report.adversarial_schedules.schedules_executed, SCHEDULE_COUNT);
        assert!(report.adversarial_schedules.invariant_violations.is_empty());
        assert!(
            report.dependency_lock.as_ref().is_some_and(|lock| lock.sha256.len() == 64),
            "Cargo should resolve the standalone crate's lock before tests run"
        );
    }

    #[test]
    fn duplicate_case_ids_are_rejected() {
        let mut document = fixture_document();
        let duplicate = document["cases"][1]["case_id"].clone();
        document["cases"][0]["case_id"] = duplicate;
        assert!(semantic_errors(&document).iter().any(|error| error.contains("duplicate case_id")));
    }

    #[test]
    fn conflict_case_cannot_drop_overwrite_prohibition() {
        let mut document = fixture_document();
        let prohibited = document["cases"][2]["prohibited"].as_array_mut().expect("prohibited");
        prohibited.retain(|item| item.as_str() != Some("overwrite_original"));
        assert!(semantic_errors(&document).iter().any(|error| error.contains("FUXC-003")));
    }

    #[test]
    fn relay_case_cannot_be_weakened_into_booking_claim() {
        let mut document = fixture_document();
        let prohibited = document["cases"][4]["prohibited"].as_array_mut().expect("prohibited");
        prohibited.retain(|item| item.as_str() != Some("promote_to_booking"));
        assert!(semantic_errors(&document).iter().any(|error| error.contains("FUXC-005")));
    }

    #[test]
    fn revoked_consent_case_must_prohibit_forwarding() {
        let mut document = fixture_document();
        let prohibited = document["cases"][7]["prohibited"].as_array_mut().expect("prohibited");
        prohibited.retain(|item| item.as_str() != Some("forward_payload"));
        assert!(semantic_errors(&document).iter().any(|error| error.contains("FUXC-008")));
    }

    #[test]
    fn unknown_outcome_case_must_prohibit_automatic_resend() {
        let mut document = fixture_document();
        let prohibited = document["cases"][14]["prohibited"].as_array_mut().expect("prohibited");
        prohibited.retain(|item| item.as_str() != Some("resend_automatically"));
        assert!(semantic_errors(&document).iter().any(|error| error.contains("FUXC-015")));
    }

    #[test]
    fn exact_envelope_replay_is_idempotent_without_provenance_overwrite() {
        let mut ledger = EventLedger::default();
        let original = synthetic_event(42, 0);
        let event_id = original.event_id.clone();
        assert_eq!(ledger.ingest(original.clone()), IngestResult::Applied);
        assert_eq!(ledger.ingest(original.clone()), IngestResult::Duplicate);
        assert_eq!(ledger.applied_effect_count(), 1);
        assert_eq!(
            ledger.stored(&event_id).expect("stored original"),
            &original
        );
        assert_eq!(ledger.duplicate_delivery_count(), 1);
    }

    #[test]
    fn same_payload_digest_with_changed_provenance_is_conflict() {
        let mut ledger = EventLedger::default();
        let original = synthetic_event(42, 0);
        let event_id = original.event_id.clone();
        let mut altered = original.clone();
        altered.recording_role = "different_recorder".to_owned();
        assert_eq!(ledger.ingest(original.clone()), IngestResult::Applied);
        assert_eq!(ledger.ingest(altered.clone()), IngestResult::Conflict);
        assert_eq!(ledger.stored(&event_id).expect("first event remains"), &original);
        assert_eq!(ledger.conflict_count(), 1);
        assert_eq!(ledger.conflicting_events(&event_id), &[altered]);
        assert_eq!(ledger.applied_effect_count(), 1);
    }

    #[test]
    fn same_event_id_different_digest_is_recorded_conflict_not_overwrite() {
        let mut ledger = EventLedger::default();
        let original = synthetic_event(42, 1);
        let event_id = original.event_id.clone();
        let original_digest = original.digest.clone();
        let mut conflict = synthetic_event(42, 1);
        conflict.digest = "different-digest".to_owned();
        assert_eq!(ledger.ingest(original.clone()), IngestResult::Applied);
        assert_eq!(ledger.ingest(conflict.clone()), IngestResult::Conflict);
        assert_eq!(
            ledger.stored(&event_id).expect("stored first event").digest,
            original_digest
        );
        assert_eq!(ledger.applied_effect_count(), 1);
        assert_eq!(ledger.conflict_count(), 1);
        assert_eq!(ledger.conflicting_events(&event_id), &[conflict]);
    }

    #[test]
    fn separate_request_references_are_never_fuzzy_merged() {
        let mut ledger = EventLedger::default();
        let mut first = synthetic_event(11, 1);
        first.request_ref = "req-similar-a".to_owned();
        let mut second = synthetic_event(11, 2);
        second.request_ref = "req-similar-b".to_owned();
        assert_eq!(ledger.ingest(first), IngestResult::Applied);
        assert_eq!(ledger.ingest(second), IngestResult::Applied);
        assert_eq!(ledger.request_count(), 2);
        assert_eq!(ledger.applied_effect_count(), 2);
    }

    #[test]
    fn source_evidence_cannot_claim_a_higher_authority_class() {
        assert!(!evidence_can_support_claim(EvidenceKind::Relayed, ClaimType::ProviderReceipt));
        assert!(!evidence_can_support_claim(EvidenceKind::Relayed, ClaimType::BookingConfirmed));
        assert!(!evidence_can_support_claim(EvidenceKind::ProviderReportedComplete, ClaimType::FarmerAccepted));
        assert!(!evidence_can_support_claim(EvidenceKind::OutcomeUnknown, ClaimType::BookingConfirmed));
        assert!(evidence_can_support_claim(EvidenceKind::ProviderReceipt, ClaimType::ProviderReceipt));
        assert!(evidence_can_support_claim(EvidenceKind::FarmerAcceptance, ClaimType::FarmerAccepted));
    }

    #[test]
    fn consent_and_authority_are_both_required_for_forwarding() {
        assert!(forwarding_allowed(true, true));
        assert!(!forwarding_allowed(true, false));
        assert!(!forwarding_allowed(false, true));
        assert!(!forwarding_allowed(false, false));
    }

    #[test]
    fn deterministic_schedules_are_reproducible_and_clean() {
        let first = run_deterministic_schedules();
        let second = run_deterministic_schedules();
        assert_eq!(first.schedules_executed, 10_000);
        assert_eq!(first.invariant_violations, second.invariant_violations);
        assert!(first.invariant_violations.is_empty());
    }
}

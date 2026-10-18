use jsonschema::validator_for;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

const DIALECT: &str = "https://json-schema.org/draft/2020-12/schema";
const REPORT_VERSION: &str = "1.0.0";
const CLAIM_CEILING: &str = "Fake-store and fixture-contract checks only. Does not prove browser/IndexedDB durability, real remote receipt, server idempotency, production UI behavior, accessibility conformance, agronomic safety, or field impact.";

pub const CASES_PATH: &str = "research/farmer-ux-004-journal-cases-v1.json";
pub const CASES_SCHEMA_PATH: &str = "research/farmer-ux-004-journal-cases-v1.schema.json";
pub const MAPPING_PATH: &str = "research/farmer-ux-004-state-mapping-v1.json";
pub const MAPPING_SCHEMA_PATH: &str = "research/farmer-ux-004-state-mapping-v1.schema.json";

const REQUIRED_INVARIANTS: &[&str] = &[
    "durable_submitted_precedes_network_send",
    "failed_durable_write_blocks_send",
    "submitted_without_authoritative_result_recovers_as_outcome_unknown",
    "outcome_unknown_is_not_dispatch_eligible",
    "client_operation_id_is_not_server_idempotency",
    "request_receipt_is_not_quote_or_booking",
    "booking_is_not_completed_work",
    "provider_completion_is_not_farmer_acceptance",
    "scope_change_never_retargets_existing_operation",
    "unknown_is_not_confirmed_or_rejected",
    "sensitive_payload_is_not_in_generic_journal_or_logs",
    "human_mediated_fallback_preserves_request_reference_where_supported",
    "corrupt_journal_does_not_prove_operation_was_never_sent",
];

const REQUIRED_PROMOTIONS: &[(&str, &str, &[&str])] = &[
    ("RequestReceived", "QuoteOffered", &["complete_scoped_provider_quote", "validity_and_price_terms"]),
    ("QuoteOffered", "BookingConfirmed", &["explicit_farmer_quote_acceptance", "unexpired_quote", "scope_match", "authoritative_booking_confirmation"]),
    ("ProviderReportedComplete", "FarmerAccepted", &["explicit_farmer_acceptance_event"]),
    ("ProviderReportedComplete", "DisputedOrRework", &["explicit_farmer_dispute_event"]),
    ("OutcomeUnknown", "RequestReceived", &["authoritative_reconciliation_receipt"]),
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AttemptStatus {
    Prepared,
    Submitted,
    Confirmed,
    ExplicitlyRejected,
    OutcomeUnknown,
    ReconciledApplied,
    ReconciledNotApplied,
    NotSent,
}
impl AttemptStatus {
    /// A new attempt may be started from Prepared; this does not itself authorize a network send.
    pub fn can_advance_to_submitted(self) -> bool {
        matches!(self, Self::Prepared)
    }
    pub fn needs_reconciliation(self) -> bool {
        matches!(self, Self::Submitted | Self::OutcomeUnknown)
    }
}

/// Network send is allowed only when the exact Submitted record was committed and read back.
/// The boolean is evidence supplied by the journal adapter; it is not inferred from an enum alone.
pub fn network_send_authorized(
    record: Option<&JournalRecord>,
    submitted_commit_verified: bool,
) -> bool {
    submitted_commit_verified
        && record.is_some_and(|r| r.schema_version == 1 && r.status == AttemptStatus::Submitted)
}

pub fn transition_allowed(from: AttemptStatus, to: AttemptStatus) -> bool {
    if from == to {
        return true;
    }
    matches!(
        (from, to),
        (AttemptStatus::Prepared, AttemptStatus::Submitted)
            | (AttemptStatus::Prepared, AttemptStatus::NotSent)
            | (AttemptStatus::Submitted, AttemptStatus::Confirmed)
            | (AttemptStatus::Submitted, AttemptStatus::ExplicitlyRejected)
            | (AttemptStatus::Submitted, AttemptStatus::OutcomeUnknown)
            | (AttemptStatus::OutcomeUnknown, AttemptStatus::ReconciledApplied)
            | (AttemptStatus::OutcomeUnknown, AttemptStatus::ReconciledNotApplied)
    )
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct JournalRecord {
    pub schema_version: u32,
    pub operation_id: String,
    pub operation_kind: String,
    pub target_fingerprint: String,
    pub status: AttemptStatus,
    pub remote_receipt_ref: Option<String>,
}
impl JournalRecord {
    pub fn new(operation_id: &str, target_fingerprint: &str) -> Result<Self, &'static str> {
        if operation_id.trim().is_empty() {
            return Err("operation id must be nonempty");
        }
        if target_fingerprint.trim().is_empty() {
            return Err("target fingerprint must be nonempty");
        }
        Ok(Self {
            schema_version: 1,
            operation_id: operation_id.to_owned(),
            operation_kind: "FarmerProblemReport".to_owned(),
            target_fingerprint: target_fingerprint.to_owned(),
            status: AttemptStatus::Prepared,
            remote_receipt_ref: None,
        })
    }
    pub fn set_status(&mut self, next: AttemptStatus) -> Result<(), &'static str> {
        if !transition_allowed(self.status, next) {
            return Err("illegal status transition");
        }
        self.status = next;
        Ok(())
    }
}

pub trait DurableJournal {
    fn commit(&mut self, record: JournalRecord) -> Result<(), StoreError>;
    fn read(&self, operation_id: &str) -> Option<JournalRecord>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StoreError {
    CommitFailed,
    InvalidTransition,
    IdentityChanged,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RemoteError {
    AmbiguousTransport,
    MalformedResponse,
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RemoteResponse {
    RequestReceived { receipt_ref: String },
    ExplicitlyRejected { reason_code: String },
}
pub trait RemoteService {
    fn send(
        &mut self,
        operation_id: &str,
        target_fingerprint: &str,
    ) -> Result<RemoteResponse, RemoteError>;
}

#[derive(Debug, Clone)]
pub struct FakeJournal {
    records: BTreeMap<String, JournalRecord>,
    commit_count: usize,
    fail_on_commit: Option<usize>,
    pub committed_statuses: Vec<AttemptStatus>,
}
impl Default for FakeJournal {
    fn default() -> Self {
        Self {
            records: BTreeMap::new(),
            commit_count: 0,
            fail_on_commit: None,
            committed_statuses: Vec::new(),
        }
    }
}
impl FakeJournal {
    pub fn fail_on_commit(mut self, ordinal: usize) -> Self {
        self.fail_on_commit = Some(ordinal);
        self
    }
    pub fn commit_count(&self) -> usize {
        self.commit_count
    }
}
impl DurableJournal for FakeJournal {
    fn commit(&mut self, record: JournalRecord) -> Result<(), StoreError> {
        self.commit_count += 1;
        if self.fail_on_commit == Some(self.commit_count) {
            return Err(StoreError::CommitFailed);
        }
        if let Some(previous) = self.records.get(&record.operation_id) {
            if previous.target_fingerprint != record.target_fingerprint
                || previous.operation_kind != record.operation_kind
            {
                return Err(StoreError::IdentityChanged);
            }
            if !transition_allowed(previous.status, record.status) {
                return Err(StoreError::InvalidTransition);
            }
        }
        self.committed_statuses.push(record.status);
        self.records.insert(record.operation_id.clone(), record);
        Ok(())
    }
    fn read(&self, operation_id: &str) -> Option<JournalRecord> {
        self.records.get(operation_id).cloned()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RemoteMode {
    Receipt,
    TypedRejection,
    AmbiguousNoEffect,
    AppliedThenResponseLost,
    MalformedAfterPossibleApply,
}
#[derive(Debug, Clone)]
pub struct FakeRemote {
    pub mode: RemoteMode,
    pub calls: usize,
    pub applied_effects: usize,
}
impl FakeRemote {
    pub fn new(mode: RemoteMode) -> Self {
        Self {
            mode,
            calls: 0,
            applied_effects: 0,
        }
    }
}
impl RemoteService for FakeRemote {
    fn send(
        &mut self,
        _operation_id: &str,
        _target_fingerprint: &str,
    ) -> Result<RemoteResponse, RemoteError> {
        self.calls += 1;
        match self.mode {
            RemoteMode::Receipt => {
                self.applied_effects += 1;
                Ok(RemoteResponse::RequestReceived {
                    receipt_ref: "synthetic-receipt".to_owned(),
                })
            }
            RemoteMode::TypedRejection => Ok(RemoteResponse::ExplicitlyRejected {
                reason_code: "SYNTHETIC_REJECTION".to_owned(),
            }),
            RemoteMode::AmbiguousNoEffect => Err(RemoteError::AmbiguousTransport),
            RemoteMode::AppliedThenResponseLost => {
                self.applied_effects += 1;
                Err(RemoteError::AmbiguousTransport)
            }
            RemoteMode::MalformedAfterPossibleApply => {
                self.applied_effects += 1;
                Err(RemoteError::MalformedResponse)
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ExecutionResult {
    pub status: AttemptStatus,
    pub send_attempted: bool,
    pub durable_status: Option<AttemptStatus>,
    pub note: &'static str,
}

pub fn submit_new<J: DurableJournal, R: RemoteService>(
    journal: &mut J,
    remote: &mut R,
    operation_id: &str,
    target_fingerprint: &str,
) -> ExecutionResult {
    if let Some(existing) = journal.read(operation_id) {
        return ExecutionResult {
            status: existing.status,
            send_attempted: false,
            durable_status: Some(existing.status),
            note: "operation ID already exists; returned existing status without resend",
        };
    }

    let mut record = match JournalRecord::new(operation_id, target_fingerprint) {
        Ok(record) => record,
        Err(_) => {
            return ExecutionResult {
                status: AttemptStatus::NotSent,
                send_attempted: false,
                durable_status: None,
                note: "invalid local operation metadata",
            };
        }
    };

    if journal.commit(record.clone()).is_err() {
        return ExecutionResult {
            status: AttemptStatus::NotSent,
            send_attempted: false,
            durable_status: journal.read(operation_id).map(|r| r.status),
            note: "Prepared commit failed; no send",
        };
    }

    if record.set_status(AttemptStatus::Submitted).is_err()
        || journal.commit(record.clone()).is_err()
    {
        return ExecutionResult {
            status: AttemptStatus::Prepared,
            send_attempted: false,
            durable_status: journal.read(operation_id).map(|r| r.status),
            note: "Submitted commit failed; no send",
        };
    }

    // A journal adapter must return only after Submitted is durably committed.
    // Read back before calling remote so an uncommitted transition cannot dispatch.
    let submitted_record = journal.read(operation_id);
    let marker_verified = submitted_record
        .as_ref()
        .is_some_and(|record| record.status == AttemptStatus::Submitted);
    if !network_send_authorized(submitted_record.as_ref(), marker_verified) {
        return ExecutionResult {
            status: AttemptStatus::OutcomeUnknown,
            send_attempted: false,
            durable_status: submitted_record.map(|r| r.status),
            note: "Submitted commit could not be verified; fail closed",
        };
    }

    let response_status = match remote.send(operation_id, target_fingerprint) {
        Ok(RemoteResponse::RequestReceived { receipt_ref }) => {
            let _ = record.set_status(AttemptStatus::Confirmed);
            record.remote_receipt_ref = Some(receipt_ref);
            AttemptStatus::Confirmed
        }
        Ok(RemoteResponse::ExplicitlyRejected { .. }) => {
            let _ = record.set_status(AttemptStatus::ExplicitlyRejected);
            AttemptStatus::ExplicitlyRejected
        }
        Err(RemoteError::AmbiguousTransport | RemoteError::MalformedResponse) => {
            let _ = record.set_status(AttemptStatus::OutcomeUnknown);
            AttemptStatus::OutcomeUnknown
        }
    };

    if journal.commit(record).is_err() {
        // A response may have arrived, but the final status was not durably saved.
        // Keep caller-visible status unknown; restart recovery handles persisted Submitted.
        return ExecutionResult {
            status: AttemptStatus::OutcomeUnknown,
            send_attempted: true,
            durable_status: journal.read(operation_id).map(|r| r.status),
            note: "result commit failed; outcome unknown",
        };
    }
    ExecutionResult {
        status: response_status,
        send_attempted: true,
        durable_status: journal.read(operation_id).map(|r| r.status),
        note: "typed synthetic remote result recorded",
    }
}

pub fn recover_after_restart<J: DurableJournal>(
    journal: &mut J,
    operation_id: &str,
) -> AttemptStatus {
    let Some(mut record) = journal.read(operation_id) else {
        // Missing/corrupt storage is not proof that no earlier send happened.
        return AttemptStatus::OutcomeUnknown;
    };
    if record.schema_version != 1 {
        return AttemptStatus::OutcomeUnknown;
    }
    if record.status == AttemptStatus::Submitted {
        let _ = record.set_status(AttemptStatus::OutcomeUnknown);
        // Even if this commit fails, the recovered projection remains unknown and
        // no dispatch is permitted until authoritative reconciliation.
        let _ = journal.commit(record);
        return AttemptStatus::OutcomeUnknown;
    }
    record.status
}

pub fn recover_untrusted_journal_bytes(bytes: &[u8]) -> AttemptStatus {
    match serde_json::from_slice::<JournalRecord>(bytes) {
        Ok(record) if record.schema_version == 1 => {
            if record.status == AttemptStatus::Submitted {
                AttemptStatus::OutcomeUnknown
            } else {
                record.status
            }
        }
        // Corrupt/future-version data may have been Submitted before damage.
        _ => AttemptStatus::OutcomeUnknown,
    }
}
pub fn target_matches(record: &JournalRecord, selected_target_fingerprint: &str) -> bool {
    record.target_fingerprint == selected_target_fingerprint
}

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
pub struct FixtureReport {
    pub version: String,
    pub status: String,
    pub case_count: usize,
    pub unknown_case_count: usize,
    pub attempt_status_count: usize,
    pub service_state_count: usize,
    pub allowed_promotion_count: usize,
    pub artifacts: Vec<ArtifactCheck>,
    pub semantic_errors: Vec<String>,
    pub claim_ceiling: String,
}

fn sha256(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(64);
    for byte in Sha256::digest(bytes) {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}
fn read_json(root: &Path, path: &str) -> Result<(Vec<u8>, Value), String> {
    let bytes = fs::read(root.join(path)).map_err(|e| format!("cannot read {path}: {e}"))?;
    let value = serde_json::from_slice(&bytes).map_err(|e| format!("cannot parse {path}: {e}"))?;
    Ok((bytes, value))
}
fn validate_schema(instance: &Value, schema: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if schema.get("$schema").and_then(Value::as_str) != Some(DIALECT) {
        errors.push("unsupported or missing schema dialect".to_owned());
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
    document: &Value,
    schema: &Value,
    data_bytes: &[u8],
    schema_bytes: &[u8],
) -> ArtifactCheck {
    let errors = validate_schema(document, schema);
    ArtifactCheck {
        path: path.to_owned(),
        sha256: sha256(data_bytes),
        schema_path: schema_path.to_owned(),
        schema_sha256: sha256(schema_bytes),
        schema_dialect: schema.get("$schema").and_then(Value::as_str).unwrap_or("missing").to_owned(),
        schema_status: if errors.is_empty() { "PASS" } else { "FAIL" }.to_owned(),
        errors,
    }
}
/// Consult the versioned farmer-service mapping, requiring every evidence item for a guarded promotion.
pub fn promotion_allowed(
    mapping: &Value,
    from_state: &str,
    to_state: &str,
    evidence: &BTreeSet<String>,
) -> bool {
    let promotions = mapping
        .get("allowed_promotions")
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();

    promotions.iter().find(|promotion| {
        promotion.get("from").and_then(Value::as_str) == Some(from_state)
            && promotion.get("to").and_then(Value::as_str) == Some(to_state)
    }).and_then(|promotion| promotion.get("requires").and_then(Value::as_array))
        .is_some_and(|required| {
            !required.is_empty()
                && required.iter().all(|item| {
                    item.as_str().is_some_and(|key| evidence.contains(key))
                })
        })
}

fn semantic_errors(cases: &Value, mapping: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    let case_list = cases.get("cases").and_then(Value::as_array).cloned().unwrap_or_default();
    let ids: Vec<String> = case_list.iter()
        .filter_map(|c| c.get("id").and_then(Value::as_str).map(str::to_owned)).collect();
    if ids.len() != case_list.len() { errors.push("case corpus contains case without string id".to_owned()); }
    if ids.iter().collect::<BTreeSet<_>>().len() != ids.len() { errors.push("case IDs must be unique".to_owned()); }
    if case_list.len() != 16 { errors.push(format!("expected 16 frozen cases, found {}", case_list.len())); }
    let required_invariants: BTreeSet<&str> = REQUIRED_INVARIANTS.iter().copied().collect();
    let corpus_invariants: BTreeSet<&str> = cases.get("shared_invariants").and_then(Value::as_array)
        .into_iter().flatten().filter_map(Value::as_str).collect();
    for invariant in REQUIRED_INVARIANTS {
        if !corpus_invariants.contains(invariant) {
            errors.push(format!("case corpus missing invariant {invariant}"));
        }
    }

    let known_status: BTreeSet<&str> = [
        "Prepared","Submitted","Confirmed","ExplicitlyRejected","OutcomeUnknown",
        "ReconciledApplied","ReconciledNotApplied","NotSent"
    ].into_iter().collect();
    let known_send: BTreeSet<&str> = [
        "blocked","blocked_until_scope_confirmed","possibly_not_called","attempted","no_new_send"
    ].into_iter().collect();
    for case in &case_list {
        let id = case.get("id").and_then(Value::as_str).unwrap_or("<missing-id>");
        let status = case.get("expected_state").and_then(Value::as_str).unwrap_or("");
        let effect = case.get("remote_effect_state").and_then(Value::as_str).unwrap_or("");
        let send = case.get("send").and_then(Value::as_str).unwrap_or("");
        let auto_replay = case.get("auto_replay_allowed").and_then(Value::as_bool);
        let reconcile = case.get("reconciliation_required").and_then(Value::as_bool);
        if !known_status.contains(status) { errors.push(format!("{id}: invalid expected state {status:?}")); }
        if !known_send.contains(send) { errors.push(format!("{id}: invalid send behavior {send:?}")); }
        if auto_replay != Some(false) { errors.push(format!("{id}: automatic replay must be false")); }
        if (status == "OutcomeUnknown") != (reconcile == Some(true)) { errors.push(format!("{id}: unknown/reconciliation mismatch")); }
        if status == "OutcomeUnknown" && effect != "unknown" { errors.push(format!("{id}: OutcomeUnknown must keep remote effect unknown")); }
        if status == "NotSent" && effect != "not_sent" { errors.push(format!("{id}: NotSent requires explicit not-sent evidence")); }
        if id == "FUXJ-015" && (status != "OutcomeUnknown" || effect != "unknown" || case.get("expected_ui").and_then(Value::as_str).unwrap_or("").to_ascii_lowercase().contains("nothing was sent")) {
            errors.push("FUXJ-015: corrupt journal must not claim NotSent".to_owned());
        }
    }

    let statuses = mapping.get("operation_attempt_statuses").and_then(Value::as_array).cloned().unwrap_or_default();
    let mut names = BTreeSet::new();
    for entry in &statuses {
        let name = entry.get("status").and_then(Value::as_str).unwrap_or("");
        if !names.insert(name) { errors.push(format!("duplicate attempt status {name:?}")); }
        if name == "OutcomeUnknown" && (entry.get("can_advance_to_submitted").and_then(Value::as_bool) != Some(false) || entry.get("needs_reconciliation").and_then(Value::as_bool) != Some(true)) {
            errors.push("mapping: OutcomeUnknown cannot start a new attempt and must require reconciliation".to_owned());
        }
        if name == "Prepared" && entry.get("can_advance_to_submitted").and_then(Value::as_bool) != Some(true) {
            errors.push("mapping: Prepared must be able to advance to Submitted but cannot send directly".to_owned());
        }
        if name != "Prepared" && entry.get("can_advance_to_submitted").and_then(Value::as_bool) != Some(false) {
            errors.push(format!("mapping: {name} cannot advance to Submitted"));
        }
    }
    if statuses.len() != 8 || names.len() != 8 { errors.push("mapping must define exactly eight attempt statuses".to_owned()); }
    if mapping.get("network_send_rule").and_then(Value::as_str).is_none_or(str::is_empty) {
        errors.push("mapping missing explicit durable Submitted network-send rule".to_owned());
    }
    let service_count = mapping.get("farmer_service_states").and_then(Value::as_array).map_or(0, Vec::len);
    if service_count != 11 { errors.push(format!("expected 11 service states, found {service_count}")); }
    let promotions = mapping.get("allowed_promotions").and_then(Value::as_array).cloned().unwrap_or_default();
    let promotion_count = promotions.len();
    if promotion_count != 9 { errors.push(format!("expected 9 guarded promotions, found {promotion_count}")); }
    let pairs: Vec<(String, String)> = promotions.iter().filter_map(|p| {
        Some((p.get("from")?.as_str()?.to_owned(), p.get("to")?.as_str()?.to_owned()))
    }).collect();
    if pairs.len() != promotion_count { errors.push("a promotion is missing a string from/to state".to_owned()); }
    if pairs.iter().collect::<BTreeSet<_>>().len() != pairs.len() {
        errors.push("promotion source/target pairs must be unique".to_owned());
    }
    for (from_state, to_state, required_evidence) in REQUIRED_PROMOTIONS {
        let matches: Vec<&Value> = promotions.iter().filter(|p| {
            p.get("from").and_then(Value::as_str) == Some(from_state)
                && p.get("to").and_then(Value::as_str) == Some(to_state)
        }).collect();
        if matches.len() != 1 {
            errors.push(format!("mapping must define exactly one guarded promotion {from_state}->{to_state}"));
            continue;
        }
        let actual: BTreeSet<&str> = matches[0].get("requires").and_then(Value::as_array)
            .into_iter().flatten().filter_map(Value::as_str).collect();
        for required in *required_evidence {
            if !actual.contains(required) {
                errors.push(format!("mapping promotion {from_state}->{to_state} missing evidence requirement {required}"));
            }
        }
    }
    errors.sort();
    errors.dedup();
    errors
}

pub fn default_repository_root() -> Result<PathBuf, String> {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).parent().and_then(Path::parent).map(Path::to_path_buf)
        .ok_or_else(|| "cannot derive repository root; use --root".to_owned())
}
pub fn verify_repository(root: &Path) -> Result<FixtureReport, String> {
    let (cases_bytes, cases_doc) = read_json(root, CASES_PATH)?;
    let (cases_schema_bytes, cases_schema) = read_json(root, CASES_SCHEMA_PATH)?;
    let (mapping_bytes, mapping_doc) = read_json(root, MAPPING_PATH)?;
    let (mapping_schema_bytes, mapping_schema) = read_json(root, MAPPING_SCHEMA_PATH)?;
    let artifacts = vec![
        artifact_check(CASES_PATH, CASES_SCHEMA_PATH, &cases_doc, &cases_schema, &cases_bytes, &cases_schema_bytes),
        artifact_check(MAPPING_PATH, MAPPING_SCHEMA_PATH, &mapping_doc, &mapping_schema, &mapping_bytes, &mapping_schema_bytes),
    ];
    let errors = semantic_errors(&cases_doc, &mapping_doc);
    let schema_failed = artifacts.iter().any(|a| !a.errors.is_empty());
    let cases = cases_doc.get("cases").and_then(Value::as_array).map_or(0, Vec::len);
    let unknown = cases_doc.get("cases").and_then(Value::as_array)
        .map(|list| list.iter().filter(|c| c.get("expected_state").and_then(Value::as_str)==Some("OutcomeUnknown")).count())
        .unwrap_or(0);
    Ok(FixtureReport {
        version: REPORT_VERSION.to_owned(),
        status: if !schema_failed && errors.is_empty() { "PASS" } else { "FAIL" }.to_owned(),
        case_count: cases,
        unknown_case_count: unknown,
        attempt_status_count: mapping_doc.get("operation_attempt_statuses").and_then(Value::as_array).map_or(0, Vec::len),
        service_state_count: mapping_doc.get("farmer_service_states").and_then(Value::as_array).map_or(0, Vec::len),
        allowed_promotion_count: mapping_doc.get("allowed_promotions").and_then(Value::as_array).map_or(0, Vec::len),
        artifacts,
        semantic_errors: errors,
        claim_ceiling: CLAIM_CEILING.to_owned(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record() -> JournalRecord {
        JournalRecord::new("synthetic-op-1", "site-fingerprint-1").expect("valid test operation")
    }

    #[test]
    fn fixture_schemas_and_semantics_pass() {
        let root=default_repository_root().expect("repo root");
        let report=verify_repository(&root).expect("fixtures load");
        assert_eq!(report.status,"PASS","semantic errors: {:?}; artifacts: {:?}",report.semantic_errors,report.artifacts);
        assert_eq!(report.case_count,16);
        assert_eq!(report.unknown_case_count,6);
        assert_eq!(report.attempt_status_count,8);
        assert_eq!(report.service_state_count,11);
    }

    #[test]
    fn prepared_commit_failure_never_sends() {
        let mut journal=FakeJournal::default().fail_on_commit(1);
        let mut remote=FakeRemote::new(RemoteMode::Receipt);
        let result=submit_new(&mut journal,&mut remote,"op-1","site-1");
        assert_eq!(result.status,AttemptStatus::NotSent);
        assert!(!result.send_attempted);
        assert_eq!(remote.calls,0);
    }

    #[test]
    fn submitted_commit_failure_never_sends() {
        let mut journal=FakeJournal::default().fail_on_commit(2);
        let mut remote=FakeRemote::new(RemoteMode::Receipt);
        let result=submit_new(&mut journal,&mut remote,"op-2","site-1");
        assert_eq!(result.status,AttemptStatus::Prepared);
        assert!(!result.send_attempted);
        assert_eq!(remote.calls,0);
        assert_eq!(journal.read("op-2").expect("prepared is durable").status,AttemptStatus::Prepared);
    }

    #[test]
    fn typed_receipt_confirms_only_request_receipt() {
        let mut journal=FakeJournal::default();
        let mut remote=FakeRemote::new(RemoteMode::Receipt);
        let result=submit_new(&mut journal,&mut remote,"op-3","site-1");
        assert_eq!(result.status,AttemptStatus::Confirmed);
        assert_eq!(remote.calls,1);
        assert_eq!(remote.applied_effects,1);
        assert_eq!(journal.read("op-3").expect("record").remote_receipt_ref.as_deref(),Some("synthetic-receipt"));
        assert_eq!(
            journal.committed_statuses,
            vec![AttemptStatus::Prepared, AttemptStatus::Submitted, AttemptStatus::Confirmed]
        );
    }

    #[test]
    fn ambiguous_transport_never_becomes_failure_or_replay() {
        let mut journal=FakeJournal::default();
        let mut remote=FakeRemote::new(RemoteMode::AmbiguousNoEffect);
        let result=submit_new(&mut journal,&mut remote,"op-4","site-1");
        assert_eq!(result.status,AttemptStatus::OutcomeUnknown);
        assert_eq!(remote.calls,1);
        assert!(!AttemptStatus::OutcomeUnknown.can_advance_to_submitted());
        assert!(AttemptStatus::OutcomeUnknown.needs_reconciliation());
    }

    #[test]
    fn remote_applied_but_response_lost_stays_unknown() {
        let mut journal=FakeJournal::default();
        let mut remote=FakeRemote::new(RemoteMode::AppliedThenResponseLost);
        let result=submit_new(&mut journal,&mut remote,"op-5","site-1");
        assert_eq!(result.status,AttemptStatus::OutcomeUnknown);
        assert_eq!(remote.calls,1);
        assert_eq!(remote.applied_effects,1);
        assert_eq!(journal.read("op-5").expect("record").status,AttemptStatus::OutcomeUnknown);
    }

    #[test]
    fn malformed_response_after_possible_apply_stays_unknown() {
        let mut journal = FakeJournal::default();
        let mut remote = FakeRemote::new(RemoteMode::MalformedAfterPossibleApply);
        let result = submit_new(&mut journal, &mut remote, "op-malformed", "site-1");
        assert_eq!(result.status, AttemptStatus::OutcomeUnknown);
        assert_eq!(remote.calls, 1);
        assert_eq!(remote.applied_effects, 1);
        assert_eq!(
            journal.read("op-malformed").expect("record").status,
            AttemptStatus::OutcomeUnknown
        );
    }

    #[test]
    fn typed_rejection_is_not_ambiguous() {
        let mut journal=FakeJournal::default();
        let mut remote=FakeRemote::new(RemoteMode::TypedRejection);
        let result=submit_new(&mut journal,&mut remote,"op-6","site-1");
        assert_eq!(result.status,AttemptStatus::ExplicitlyRejected);
        assert_eq!(remote.calls,1);
        assert_eq!(remote.applied_effects,0);
    }

    #[test]
    fn failure_to_persist_result_recovers_as_unknown() {
        let mut journal=FakeJournal::default().fail_on_commit(3);
        let mut remote=FakeRemote::new(RemoteMode::Receipt);
        let result=submit_new(&mut journal,&mut remote,"op-7","site-1");
        assert_eq!(result.status,AttemptStatus::OutcomeUnknown);
        assert_eq!(remote.applied_effects,1);
        assert_eq!(journal.read("op-7").expect("submitted record remains").status,AttemptStatus::Submitted);
        assert_eq!(recover_after_restart(&mut journal,"op-7"),AttemptStatus::OutcomeUnknown);
    }

    #[test]
    fn restart_recovery_never_dispatches_unknown() {
        let mut journal=FakeJournal::default();
        let mut r=record();
        r.set_status(AttemptStatus::Submitted).expect("transition");
        journal.commit(r).expect("commit submitted");
        assert_eq!(recover_after_restart(&mut journal,"synthetic-op-1"),AttemptStatus::OutcomeUnknown);
        assert!(!recover_after_restart(&mut journal,"synthetic-op-1").can_advance_to_submitted());
    }

    #[test]
    fn corrupt_or_future_version_bytes_are_not_proof_of_not_sent() {
        assert_eq!(recover_untrusted_journal_bytes(b"{not json"),AttemptStatus::OutcomeUnknown);
        let mut r=record();
        r.schema_version=99;
        let bytes=serde_json::to_vec(&r).expect("serialize");
        assert_eq!(recover_untrusted_journal_bytes(&bytes),AttemptStatus::OutcomeUnknown);
    }

    #[test]
    fn selected_site_cannot_retarget_operation() {
        let r=record();
        assert!(!target_matches(&r,"different-site"));
        assert!(target_matches(&r,"site-fingerprint-1"));
    }

    #[test]
    fn journal_rejects_retargeting_same_operation_id() {
        let mut journal = FakeJournal::default();
        let record = JournalRecord::new("op-target", "site-original").expect("valid record");
        journal.commit(record.clone()).expect("prepared commit");
        let mut retargeted = record;
        retargeted.target_fingerprint = "site-changed".to_owned();
        assert_eq!(journal.commit(retargeted), Err(StoreError::IdentityChanged));
        assert_eq!(
            journal.read("op-target").expect("original").target_fingerprint,
            "site-original"
        );
    }

    #[test]
    fn duplicate_operation_id_never_sends_again() {
        let mut journal=FakeJournal::default();
        let mut remote=FakeRemote::new(RemoteMode::Receipt);
        let _first=submit_new(&mut journal,&mut remote,"op-8","site-1");
        let second=submit_new(&mut journal,&mut remote,"op-8","site-1");
        assert_eq!(remote.calls,1);
        assert!(!second.send_attempted);
        assert_eq!(second.status,AttemptStatus::Confirmed);
    }

    #[test]
    fn ambiguous_operation_id_is_never_resent_automatically() {
        let mut journal = FakeJournal::default();
        let mut remote = FakeRemote::new(RemoteMode::AppliedThenResponseLost);
        let first = submit_new(&mut journal, &mut remote, "op-9", "site-1");
        assert_eq!(first.status, AttemptStatus::OutcomeUnknown);
        let second = submit_new(&mut journal, &mut remote, "op-9", "site-1");
        assert_eq!(second.status, AttemptStatus::OutcomeUnknown);
        assert!(!second.send_attempted);
        assert_eq!(remote.calls, 1);
        assert_eq!(remote.applied_effects, 1);
    }

    #[test]
    fn network_send_requires_verified_durable_submitted_record() {
        let mut r = record();
        assert!(!network_send_authorized(Some(&r), false));
        assert!(!network_send_authorized(Some(&r), true));
        r.set_status(AttemptStatus::Submitted).expect("Submitted transition");
        assert!(!network_send_authorized(Some(&r), false));
        assert!(network_send_authorized(Some(&r), true));
        assert!(!network_send_authorized(None, true));
    }

    #[test]
    fn unknown_status_cannot_transition_back_to_submitted() {
        assert!(!transition_allowed(AttemptStatus::OutcomeUnknown, AttemptStatus::Submitted));
        assert!(!transition_allowed(AttemptStatus::Confirmed, AttemptStatus::Submitted));
        assert!(transition_allowed(AttemptStatus::OutcomeUnknown, AttemptStatus::ReconciledApplied));
    }

    fn loaded_mapping() -> Value {
        serde_json::from_str(include_str!("../../farmer-ux-004-state-mapping-v1.json"))
            .expect("mapping fixture")
    }

    #[test]
    fn farmer_acceptance_is_distinct_from_provider_completion() {
        let mapping = loaded_mapping();
        let no_evidence = BTreeSet::new();
        assert!(!promotion_allowed(&mapping, "ProviderReportedComplete", "FarmerAccepted", &no_evidence));
        let evidence = BTreeSet::from(["explicit_farmer_acceptance_event".to_owned()]);
        assert!(promotion_allowed(&mapping, "ProviderReportedComplete", "FarmerAccepted", &evidence));
        assert!(!promotion_allowed(&mapping, "ProviderReportedComplete", "BookingConfirmed", &BTreeSet::from([
            "explicit_farmer_acceptance_event".to_owned()
        ])));
    }

    #[test]
    fn booking_requires_explicit_acceptance_validity_scope_and_confirmation() {
        let mapping = loaded_mapping();
        let incomplete = BTreeSet::from([
            "explicit_farmer_quote_acceptance".to_owned(),
            "unexpired_quote".to_owned(),
        ]);
        assert!(!promotion_allowed(&mapping, "QuoteOffered", "BookingConfirmed", &incomplete));
        let complete = BTreeSet::from([
            "explicit_farmer_quote_acceptance".to_owned(),
            "unexpired_quote".to_owned(),
            "scope_match".to_owned(),
            "authoritative_booking_confirmation".to_owned(),
        ]);
        assert!(promotion_allowed(&mapping, "QuoteOffered", "BookingConfirmed", &complete));
    }

    #[test]
    fn outcome_unknown_needs_authoritative_reconciliation() {
        let mapping = loaded_mapping();
        let none = BTreeSet::new();
        assert!(!promotion_allowed(&mapping, "OutcomeUnknown", "RequestReceived", &none));
        assert!(!promotion_allowed(&mapping, "OutcomeUnknown", "BookingConfirmed", &BTreeSet::from([
            "authoritative_reconciliation_receipt".to_owned()
        ])));
        let receipt = BTreeSet::from(["authoritative_reconciliation_receipt".to_owned()]);
        assert!(promotion_allowed(&mapping, "OutcomeUnknown", "RequestReceived", &receipt));
    }

    #[test]
    fn farmer_service_mapping_mutation_cannot_remove_acceptance_evidence() {
        let mut mapping = loaded_mapping();
        let promotions = mapping["allowed_promotions"].as_array_mut().expect("promotions");
        let promotion = promotions.iter_mut().find(|p| {
            p["from"] == "ProviderReportedComplete" && p["to"] == "FarmerAccepted"
        }).expect("promotion");
        promotion["requires"] = serde_json::json!([]);
        assert!(semantic_errors(&serde_json::from_str(include_str!("../../farmer-ux-004-journal-cases-v1.json")).expect("cases"), &mapping)
            .iter().any(|e| e.contains("missing evidence requirement explicit_farmer_acceptance_event")));
    }

    #[test]
    fn unknown_outcome_semantics_mutation_is_rejected() {
        let mut cases:Value=serde_json::from_str(include_str!("../../farmer-ux-004-journal-cases-v1.json")).expect("cases");
        let case=cases["cases"].as_array_mut().expect("array").iter_mut().find(|c|c["id"]=="FUXJ-015").expect("corrupt case");
        case["expected_state"]=Value::String("NotSent".to_owned());
        case["remote_effect_state"]=Value::String("not_sent".to_owned());
        case["reconciliation_required"]=Value::Bool(false);
        let mapping:Value=serde_json::from_str(include_str!("../../farmer-ux-004-state-mapping-v1.json")).expect("mapping");
        assert!(!semantic_errors(&cases,&mapping).is_empty());
    }
}

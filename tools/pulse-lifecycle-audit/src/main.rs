use serde::{de::Error as DeError, Deserialize, Deserializer, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct TraceBundle {
    schema_version: u32,
    trace_id: String,
    evidence_class: EvidenceClass,
    source_commit: String,
    conductor_build: String,
    dna_hash: String,
    events: Vec<TraceEvent>,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum EvidenceClass {
    Synthetic,
    RuntimeCapture,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum EvidenceSource {
    ConductorHost,
    SourceChain,
    Observer,
    RemotePeer,
    Client,
    Coordinator,
    Synthetic,
}

#[derive(Debug, Clone)]
struct TraceEvent {
    seq: u64,
    event_id: String,
    evidence_ref: String,
    source: EvidenceSource,
    kind: EventKind,
}

// Keep the v1 wire shape flat and strict:
// { seq, event_id, evidence_ref, source, kind: "grant_issued", data: {...} }.
// A nested EventKind field would incorrectly require kind: {kind, data} and
// drift from the checked-in JSON Schema. Decode the tagged enum from the two
// top-level fields while a deny_unknown_fields raw struct enforces closure.
impl<'de> Deserialize<'de> for TraceEvent {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(deny_unknown_fields)]
        struct RawTraceEvent {
            seq: u64,
            event_id: String,
            evidence_ref: String,
            source: EvidenceSource,
            kind: String,
            data: serde_json::Value,
        }

        let raw = RawTraceEvent::deserialize(deserializer)?;
        let tagged = serde_json::json!({
            "kind": raw.kind,
            "data": raw.data,
        });
        let kind = serde_json::from_value(tagged).map_err(D::Error::custom)?;

        Ok(Self {
            seq: raw.seq,
            event_id: raw.event_id,
            evidence_ref: raw.evidence_ref,
            source: raw.source,
            kind,
        })
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", content = "data", rename_all = "snake_case")]
enum EventKind {
    GrantIssued(GrantIssued),
    GrantRevoked(GrantRevoked),
    ClaimAccepted(ClaimAccepted),
    HostCall(HostCall),
    DomainEffect(DomainEffect),
    ProjectionRead(ProjectionRead),
    RemoteSignal(RemoteSignal),
    CallerOutcome(CallerOutcome),
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct GrantIssued {
    grant_ref: String,
    grant_action_hash: String,
    grantor: String,
    grantee: String,
    assigned: bool,
    functions: Vec<String>,
    idempotency_key: String,
    authority_index: u64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct GrantRevoked {
    grant_ref: String,
    target_grant_action_hash: String,
    actor: String,
    revocation_action_hash: String,
    authority_index: u64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ClaimAccepted {
    grant_ref: String,
    grant_action_hash: String,
    recipient: String,
    claim_action_hash: String,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum HostDecision {
    Authorized,
    Unauthorized,
    Unknown,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum HostGrantState {
    Active,
    Revoked,
    Absent,
    Unknown,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum DomainDecision {
    Allow,
    Deny,
    NotRun,
    Unknown,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct HostCall {
    operation_id: String,
    attempt_id: u32,
    grant_ref: String,
    grant_action_hash: String,
    caller: String,
    function: String,
    authority_index: u64,
    host_decision: HostDecision,
    host_state_at_decision: HostGrantState,
    domain_decision: DomainDecision,
    projection_used_for_authorization: bool,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct DomainEffect {
    operation_id: String,
    attempt_id: u32,
    effect_id: String,
    action_hash: String,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ProjectionState {
    Active,
    Revoked,
    Conflicting,
    Unknown,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ProjectionDecision {
    TrustedActive,
    TrustedRevoked,
    Unknown,
    StaleAdvisory,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ProjectionRead {
    grant_ref: String,
    minimum_epoch: u64,
    observed_epoch: u64,
    projection_state: ProjectionState,
    checkpoint_used: bool,
    decision: ProjectionDecision,
    used_for_authorization: bool,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum SignalTrustClass {
    VerifiedRecord,
    AuthenticatedPeerHint,
    UnverifiedHint,
    LocalOnly,
    Unknown,
    Rejected,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct RemoteSignal {
    event_type: String,
    trust_class: SignalTrustClass,
    claimed_sender: Option<String>,
    transport_sender: Option<String>,
    backing_action_hash: Option<String>,
    record_author: Option<String>,
    verified_payload_sha256: Option<String>,
    verifier_build: Option<String>,
    trusted_state_mutation: bool,
    authorizes_operation: bool,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum CallerOutcomeKind {
    Acknowledged,
    Unknown,
    Committed,
    NotCommitted,
    Rejected,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct CallerOutcome {
    operation_id: String,
    attempt_id: u32,
    outcome: CallerOutcomeKind,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "UPPERCASE")]
enum Status {
    Pass,
    Fail,
    Incomplete,
}

#[derive(Debug, Clone, Serialize)]
struct Finding {
    code: String,
    severity: String,
    event_id: Option<String>,
    detail: String,
}

#[derive(Debug, Clone, Default, Serialize)]
struct Counts {
    events: usize,
    grants_issued: usize,
    grants_revoked: usize,
    claims_accepted: usize,
    host_calls: usize,
    authorized_calls: usize,
    unauthorized_calls: usize,
    domain_effects: usize,
    projection_reads: usize,
    unknown_projection_reads: usize,
    remote_signals: usize,
    caller_outcomes_unknown: usize,
}

#[derive(Debug, Clone, Serialize)]
struct AuditReport {
    schema_version: u32,
    status: Status,
    trace_id: String,
    evidence_class: EvidenceClass,
    input_sha256: String,
    counts: Counts,
    findings: Vec<Finding>,
    claim_ceiling: String,
}

#[derive(Debug, Clone)]
struct GrantState {
    grant_action_hash: String,
    grantor: String,
    grantee: String,
    assigned: bool,
    functions: BTreeSet<String>,
    idempotency_key: String,
    issued_index: u64,
    issued_seq: u64,
    revoked_index: Option<u64>,
    revoked_seq: Option<u64>,
    claimed: bool,
}

#[derive(Debug, Clone)]
struct CallState {
    host_authorized: bool,
    domain_allowed: bool,
}

fn finding(
    findings: &mut Vec<Finding>,
    code: &str,
    severity: &str,
    event_id: Option<&str>,
    detail: impl Into<String>,
) {
    findings.push(Finding {
        code: code.to_string(),
        severity: severity.to_string(),
        event_id: event_id.map(ToString::to_string),
        detail: detail.into(),
    });
}

fn valid_source_commit(value: &str) -> bool {
    value.len() == 40 && value.bytes().all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn source_matches_kind(source: EvidenceSource, kind: &EventKind) -> bool {
    match kind {
        EventKind::GrantIssued(_) | EventKind::GrantRevoked(_) |
        EventKind::ClaimAccepted(_) | EventKind::HostCall(_) =>
            matches!(source, EvidenceSource::ConductorHost | EvidenceSource::Synthetic),
        EventKind::DomainEffect(_) =>
            matches!(source, EvidenceSource::SourceChain | EvidenceSource::Synthetic),
        EventKind::ProjectionRead(_) =>
            matches!(source, EvidenceSource::Observer | EvidenceSource::Synthetic),
        EventKind::RemoteSignal(_) =>
            matches!(source, EvidenceSource::RemotePeer | EvidenceSource::Client |
                EvidenceSource::Observer | EvidenceSource::Coordinator | EvidenceSource::Synthetic),
        EventKind::CallerOutcome(_) =>
            matches!(source, EvidenceSource::Client | EvidenceSource::Synthetic),
    }
}

fn audit(bundle: &TraceBundle, input_sha256: String) -> AuditReport {
    let mut findings = Vec::new();
    let mut counts = Counts { events: bundle.events.len(), ..Counts::default() };
    let mut grants: BTreeMap<String, GrantState> = BTreeMap::new();
    let mut idempotency: BTreeMap<String, String> = BTreeMap::new();
    let mut grant_hashes = BTreeSet::new();
    let mut event_ids = BTreeSet::new();
    let mut call_states: BTreeMap<(String, u32), CallState> = BTreeMap::new();
    let mut effects_by_operation: BTreeMap<String, String> = BTreeMap::new();
    let mut effects_by_attempt: BTreeSet<(String, u32)> = BTreeSet::new();
    let mut effect_ids = BTreeSet::new();
    let mut caller_outcomes: BTreeMap<(String, u32), CallerOutcomeKind> = BTreeMap::new();
    let mut last_seq = 0u64;
    let mut last_authority_index = 0u64;
    let mut saw_grant = false;
    let mut saw_host_call = false;

    if bundle.schema_version != 1 {
        finding(&mut findings, "UNSUPPORTED_SCHEMA_VERSION", "incomplete", None,
            format!("expected schema_version 1, found {}", bundle.schema_version));
    }
    if bundle.trace_id.trim().is_empty() || bundle.conductor_build.trim().is_empty() ||
        bundle.dna_hash.trim().is_empty() || !valid_source_commit(&bundle.source_commit) {
        finding(&mut findings, "TRACE_HEADER_INCOMPLETE", "incomplete", None,
            "trace_id, conductor_build, dna_hash and a 40-character lowercase source_commit are required");
    }
    if bundle.events.is_empty() {
        finding(&mut findings, "TRACE_HAS_NO_EVENTS", "incomplete", None,
            "an evidence trace must contain at least one event");
    }

    for event in &bundle.events {
        if event.seq <= last_seq {
            finding(&mut findings, "EVENT_SEQUENCE_NOT_MONOTONIC", "violation",
                Some(&event.event_id), format!("sequence {} is not greater than previous {}", event.seq, last_seq));
        }
        last_seq = event.seq;

        if event.event_id.trim().is_empty() || !event_ids.insert(event.event_id.clone()) {
            finding(&mut findings, "DUPLICATE_OR_EMPTY_EVENT_ID", "violation",
                Some(&event.event_id), "event IDs must be non-empty and unique");
        }
        if event.evidence_ref.trim().is_empty() {
            finding(&mut findings, "EVENT_EVIDENCE_REFERENCE_MISSING", "incomplete",
                Some(&event.event_id), "each event must identify the source receipt or trace span");
        }
        if !source_matches_kind(event.source, &event.kind) {
            finding(&mut findings, "EVIDENCE_PLANE_MISMATCH", "incomplete",
                Some(&event.event_id), "event source is not authoritative for this event kind");
        }
        if bundle.evidence_class == EvidenceClass::RuntimeCapture && event.source == EvidenceSource::Synthetic {
            finding(&mut findings, "SYNTHETIC_EVENT_IN_RUNTIME_CAPTURE", "incomplete",
                Some(&event.event_id), "runtime captures cannot use synthetic evidence sources");
        }

        match &event.kind {
            EventKind::GrantIssued(issue) => {
                saw_grant = true;
                counts.grants_issued += 1;
                if issue.grant_ref.trim().is_empty() || issue.grant_action_hash.trim().is_empty() ||
                    issue.grantor.trim().is_empty() || issue.grantee.trim().is_empty() ||
                    issue.idempotency_key.trim().is_empty() || issue.functions.is_empty() ||
                    issue.functions.iter().any(|f| f.trim().is_empty()) {
                    finding(&mut findings, "GRANT_ISSUANCE_INCOMPLETE", "incomplete",
                        Some(&event.event_id), "grant identity, actors, idempotency key and scope are required");
                }
                if issue.authority_index <= last_authority_index {
                    finding(&mut findings, "HOST_AUTHORITY_ORDER_NOT_MONOTONIC", "violation",
                        Some(&event.event_id), "grant issuance authority index must strictly increase");
                }
                last_authority_index = issue.authority_index;

                if let Some(existing) = idempotency.get(&issue.idempotency_key) {
                    if existing != &issue.grant_ref {
                        finding(&mut findings, "IDEMPOTENCY_KEY_MINTED_MULTIPLE_GRANTS", "violation",
                            Some(&event.event_id), format!("idempotency key {} maps to {} and {}", issue.idempotency_key, existing, issue.grant_ref));
                    }
                } else {
                    idempotency.insert(issue.idempotency_key.clone(), issue.grant_ref.clone());
                }
                if !grant_hashes.insert(issue.grant_action_hash.clone()) {
                    finding(&mut findings, "DUPLICATE_GRANT_ACTION_HASH", "violation",
                        Some(&event.event_id), "grant ActionHash must identify one grant issuance");
                }
                if grants.contains_key(&issue.grant_ref) {
                    finding(&mut findings, "GRANT_REFERENCE_REUSED", "violation",
                        Some(&event.event_id), "grant reference must be unique per issuance");
                } else {
                    grants.insert(issue.grant_ref.clone(), GrantState {
                        grant_action_hash: issue.grant_action_hash.clone(),
                        grantor: issue.grantor.clone(),
                        grantee: issue.grantee.clone(),
                        assigned: issue.assigned,
                        functions: issue.functions.iter().cloned().collect(),
                        idempotency_key: issue.idempotency_key.clone(),
                        issued_index: issue.authority_index,
                        issued_seq: event.seq,
                        revoked_index: None,
                        revoked_seq: None,
                        claimed: false,
                    });
                }
            }
            EventKind::GrantRevoked(revoke) => {
                counts.grants_revoked += 1;
                if revoke.authority_index <= last_authority_index {
                    finding(&mut findings, "HOST_AUTHORITY_ORDER_NOT_MONOTONIC", "violation",
                        Some(&event.event_id), "revocation authority index must strictly increase");
                }
                last_authority_index = revoke.authority_index;
                if revoke.revocation_action_hash.trim().is_empty() {
                    finding(&mut findings, "REVOCATION_EVIDENCE_MISSING", "incomplete",
                        Some(&event.event_id), "revocation ActionHash is required");
                }
                match grants.get_mut(&revoke.grant_ref) {
                    None => finding(&mut findings, "REVOCATION_WITHOUT_ISSUANCE", "incomplete",
                        Some(&event.event_id), "the corresponding host grant issuance is absent from this trace"),
                    Some(grant) => {
                        if revoke.target_grant_action_hash != grant.grant_action_hash {
                            finding(&mut findings, "REVOCATION_TARGET_HASH_MISMATCH", "violation",
                                Some(&event.event_id), "revocation must reference the exact host grant ActionHash");
                        }
                        if revoke.actor != grant.grantor {
                            finding(&mut findings, "REVOCATION_ACTOR_MISMATCH", "violation",
                                Some(&event.event_id), "only the original grantor may revoke the host grant");
                        }
                        if grant.revoked_index.is_some() {
                            finding(&mut findings, "GRANT_REVOKED_MORE_THAN_ONCE", "violation",
                                Some(&event.event_id), "a grant can have only one host revocation transition");
                        }
                        if revoke.authority_index <= grant.issued_index {
                            finding(&mut findings, "REVOCATION_PRECEDES_ISSUANCE", "violation",
                                Some(&event.event_id), "host revocation must follow the matching grant issuance");
                        }
                        if grant.grant_action_hash.trim().is_empty() || grant.idempotency_key.trim().is_empty() {
                            finding(&mut findings, "GRANT_IDENTITY_INCOMPLETE", "incomplete",
                                Some(&event.event_id), "the original grant identity binding is incomplete");
                        }
                        grant.revoked_index = Some(revoke.authority_index);
                        grant.revoked_seq = Some(event.seq);
                    }
                }
            }
            EventKind::ClaimAccepted(claim) => {
                counts.claims_accepted += 1;
                if claim.claim_action_hash.trim().is_empty() {
                    finding(&mut findings, "CLAIM_EVIDENCE_MISSING", "incomplete",
                        Some(&event.event_id), "accepted claim must reference its immutable evidence record");
                }
                match grants.get_mut(&claim.grant_ref) {
                    None => finding(&mut findings, "CLAIM_WITHOUT_GRANT", "incomplete",
                        Some(&event.event_id), "claim references a grant not present in the trace"),
                    Some(grant) => {
                        if claim.grant_action_hash != grant.grant_action_hash {
                            finding(&mut findings, "CLAIM_GRANT_HASH_MISMATCH", "violation",
                                Some(&event.event_id), "claim must reference the exact issued grant ActionHash");
                        }
                        if event.seq <= grant.issued_seq ||
                            grant.revoked_seq.is_some_and(|revoked_seq| event.seq >= revoked_seq) {
                            finding(&mut findings, "CLAIM_OUTSIDE_GRANT_LIFETIME", "violation",
                                Some(&event.event_id), "claim acceptance must occur after issuance and before revocation");
                        }
                        if claim.recipient != grant.grantee {
                            finding(&mut findings, "CLAIM_RECIPIENT_MISMATCH", "violation",
                                Some(&event.event_id), "claim recipient differs from the assigned grantee");
                        }
                        if !grant.assigned {
                            finding(&mut findings, "CLAIM_FOR_NON_ASSIGNED_GRANT", "violation",
                                Some(&event.event_id), "private claim acceptance is only expected for an assigned grant");
                        }
                        if grant.claimed {
                            finding(&mut findings, "DUPLICATE_CLAIM_ACCEPTANCE", "violation",
                                Some(&event.event_id), "the same grant claim was accepted more than once");
                        }
                        grant.claimed = true;
                    }
                }
            }
            EventKind::HostCall(call) => {
                counts.host_calls += 1;
                saw_host_call = true;
                if call.authority_index <= last_authority_index {
                    finding(&mut findings, "HOST_AUTHORITY_ORDER_NOT_MONOTONIC", "violation",
                        Some(&event.event_id), "host call authority index must strictly increase");
                }
                last_authority_index = call.authority_index;
                if call.operation_id.trim().is_empty() || call.attempt_id == 0 {
                    finding(&mut findings, "CALL_IDENTITY_INCOMPLETE", "incomplete",
                        Some(&event.event_id), "logical operation ID and positive attempt ID are required");
                }
                if call.projection_used_for_authorization {
                    finding(&mut findings, "PROJECTION_USED_AS_AUTHORITY", "violation",
                        Some(&event.event_id), "DHT/UI projection cannot authorize a host-protected call");
                }
                let mut host_authorized = false;
                match grants.get(&call.grant_ref) {
                    None => finding(&mut findings, "HOST_CALL_WITHOUT_GRANT", "incomplete",
                        Some(&event.event_id), "matching grant issuance is absent from this trace"),
                    Some(grant) => {
                        if call.grant_action_hash != grant.grant_action_hash {
                            finding(&mut findings, "CALL_GRANT_HASH_MISMATCH", "violation",
                                Some(&event.event_id), "host call must be bound to the exact issued grant ActionHash");
                        }
                        let active = call.authority_index > grant.issued_index &&
                            grant.revoked_index.is_none_or(|revoked| call.authority_index < revoked);
                        let expected_state = if active { HostGrantState::Active }
                            else if grant.revoked_index.is_some_and(|revoked| call.authority_index >= revoked) {
                                HostGrantState::Revoked
                            } else { HostGrantState::Absent };
                        if call.host_state_at_decision == HostGrantState::Unknown {
                            finding(&mut findings, "HOST_STATE_UNKNOWN", "incomplete",
                                Some(&event.event_id), "host grant state at the decision boundary was not observed");
                        } else if call.host_state_at_decision != expected_state {
                            finding(&mut findings, "HOST_STATE_TRACE_CONFLICT", "violation",
                                Some(&event.event_id), format!("recorded host state {:?} conflicts with reconstructed {:?}", call.host_state_at_decision, expected_state));
                        }
                        let caller_scope_ok = call.caller == grant.grantee && grant.functions.contains(&call.function);
                        if call.host_decision == HostDecision::Authorized {
                            host_authorized = true;
                            counts.authorized_calls += 1;
                            if !active {
                                finding(&mut findings, "ALLOW_WITH_INACTIVE_HOST_GRANT", "violation",
                                    Some(&event.event_id), "conductor allowed a call after grant issuance had not occurred or after revocation");
                            }
                            if !caller_scope_ok {
                                finding(&mut findings, "ALLOW_OUTSIDE_ASSIGNED_SCOPE", "violation",
                                    Some(&event.event_id), "authorized caller or function is outside the exact grant scope");
                            }
                            if grant.assigned && !grant.claimed {
                                finding(&mut findings, "ALLOW_BEFORE_PRIVATE_CLAIM", "violation",
                                    Some(&event.event_id), "assigned grant was used before the recipient claim was accepted");
                            }
                        } else if call.host_decision == HostDecision::Unauthorized {
                            counts.unauthorized_calls += 1;
                        } else {
                            finding(&mut findings, "HOST_DECISION_UNKNOWN", "incomplete",
                                Some(&event.event_id), "conductor authorization disposition is unknown");
                        }
                    }
                }
                if call.domain_decision == DomainDecision::Allow && !host_authorized {
                    finding(&mut findings, "DOMAIN_ALLOW_WITHOUT_HOST_ALLOW", "violation",
                        Some(&event.event_id), "domain execution cannot be allowed after host denial or unknown host disposition");
                }
                if call.host_decision == HostDecision::Unauthorized &&
                    call.domain_decision == DomainDecision::Unknown {
                    finding(&mut findings, "DOMAIN_OUTCOME_UNKNOWN_AFTER_HOST_DENIAL", "incomplete",
                        Some(&event.event_id), "record whether the protected zome was not invoked");
                }
                if call.host_decision == HostDecision::Authorized &&
                    call.domain_decision == DomainDecision::Unknown {
                    finding(&mut findings, "DOMAIN_OUTCOME_UNKNOWN_AFTER_HOST_ALLOW", "incomplete",
                        Some(&event.event_id), "host authorization is not proof of domain execution or rejection");
                }
                let key = (call.operation_id.clone(), call.attempt_id);
                if call_states.insert(key, CallState {
                    host_authorized,
                    domain_allowed: call.domain_decision == DomainDecision::Allow,
                }).is_some() {
                    finding(&mut findings, "DUPLICATE_CALL_ATTEMPT", "violation",
                        Some(&event.event_id), "operation ID plus attempt ID must identify one host-call attempt");
                }
            }
            EventKind::DomainEffect(effect) => {
                counts.domain_effects += 1;
                if effect.operation_id.trim().is_empty() || effect.attempt_id == 0 ||
                    effect.effect_id.trim().is_empty() || effect.action_hash.trim().is_empty() {
                    finding(&mut findings, "DOMAIN_EFFECT_EVIDENCE_INCOMPLETE", "incomplete",
                        Some(&event.event_id), "effect ID and source-chain ActionHash must be present");
                }
                if !effect_ids.insert(effect.effect_id.clone()) {
                    finding(&mut findings, "DUPLICATE_EFFECT_ID", "violation",
                        Some(&event.event_id), "effect IDs must be globally unique");
                }
                let attempt_key = (effect.operation_id.clone(), effect.attempt_id);
                if !effects_by_attempt.insert(attempt_key) {
                    finding(&mut findings, "MULTIPLE_EFFECTS_PER_ATTEMPT", "violation",
                        Some(&event.event_id), "one logical operation attempt may produce at most one durable effect");
                }
                if let Some(existing) = effects_by_operation.get(&effect.operation_id) {
                    if existing != &effect.effect_id {
                        finding(&mut findings, "MULTIPLE_EFFECTS_PER_LOGICAL_OPERATION", "violation",
                            Some(&event.event_id), format!("operation {} has effects {} and {}", effect.operation_id, existing, effect.effect_id));
                    } else {
                        finding(&mut findings, "DUPLICATE_EFFECT_OBSERVATION", "violation",
                            Some(&event.event_id), "the same durable effect was recorded twice");
                    }
                } else {
                    effects_by_operation.insert(effect.operation_id.clone(), effect.effect_id.clone());
                }
                match call_states.get(&(effect.operation_id.clone(), effect.attempt_id)) {
                    None => finding(&mut findings, "EFFECT_WITHOUT_HOST_CALL", "incomplete",
                        Some(&event.event_id), "the matching host-call attempt is absent from the trace"),
                    Some(call) => {
                        if !call.host_authorized {
                            finding(&mut findings, "EFFECT_AFTER_HOST_DENIAL", "violation",
                                Some(&event.event_id), "durable domain effect has no matching host authorization");
                        }
                        if !call.domain_allowed {
                            finding(&mut findings, "EFFECT_WITHOUT_DOMAIN_ALLOW", "violation",
                                Some(&event.event_id), "durable domain effect has no matching domain allow disposition");
                        }
                    }
                }
            }
            EventKind::ProjectionRead(read) => {
                counts.projection_reads += 1;
                if read.decision == ProjectionDecision::Unknown {
                    counts.unknown_projection_reads += 1;
                }
                if read.grant_ref.trim().is_empty() {
                    finding(&mut findings, "PROJECTION_GRANT_REFERENCE_MISSING", "incomplete",
                        Some(&event.event_id), "projection read must identify the exact grant it describes");
                }
                if read.used_for_authorization {
                    finding(&mut findings, "PROJECTION_READ_USED_FOR_AUTHORITY", "violation",
                        Some(&event.event_id), "application projection reads are not host authorization");
                }
                let stale_checkpoint = read.observed_epoch < read.minimum_epoch;
                if read.checkpoint_used && stale_checkpoint &&
                    read.decision != ProjectionDecision::Unknown {
                    finding(&mut findings, "STALE_CHECKPOINT_READ_NOT_UNKNOWN", "violation",
                        Some(&event.event_id), "a checkpoint-constrained stale read must return UNKNOWN");
                }

                match grants.get(&read.grant_ref) {
                    None => finding(&mut findings, "PROJECTION_READ_WITHOUT_GRANT", "incomplete",
                        Some(&event.event_id), "the exact grant issuance is absent from this trace"),
                    Some(grant) => {
                        let read_before_issue = event.seq <= grant.issued_seq ||
                            read.observed_epoch < grant.issued_index;
                        let revoked_before_read = grant.revoked_seq
                            .is_some_and(|revoked_seq| event.seq > revoked_seq);
                        let stale_against_known_revocation = revoked_before_read &&
                            grant.revoked_index.is_some_and(|revoked_index| read.observed_epoch < revoked_index);

                        if read_before_issue &&
                            matches!(read.decision, ProjectionDecision::TrustedActive | ProjectionDecision::TrustedRevoked) {
                            finding(&mut findings, "PROJECTION_TRUSTED_BEFORE_GRANT_ISSUANCE", "violation",
                                Some(&event.event_id), "a projection cannot be trusted for a grant before its issuance epoch");
                        }
                        if stale_against_known_revocation &&
                            matches!(read.decision, ProjectionDecision::TrustedActive | ProjectionDecision::TrustedRevoked) {
                            finding(&mut findings, "STALE_PROJECTION_LABELED_TRUSTED", "violation",
                                Some(&event.event_id), "a projection older than a revocation already present in this trace cannot be labeled trusted");
                        }
                        if stale_against_known_revocation && !read.checkpoint_used &&
                            !matches!(read.decision, ProjectionDecision::StaleAdvisory | ProjectionDecision::Unknown) {
                            finding(&mut findings, "STALE_READ_NOT_MARKED_ADVISORY", "violation",
                                Some(&event.event_id), "an unconstrained stale projection must be explicitly advisory or UNKNOWN");
                        }
                        if revoked_before_read &&
                            grant.revoked_index.is_some_and(|revoked_index| read.observed_epoch >= revoked_index) &&
                            read.projection_state == ProjectionState::Active &&
                            read.decision != ProjectionDecision::Unknown {
                            finding(&mut findings, "PROJECTION_CONFLICTS_WITH_REVOKED_AUTHORITY", "violation",
                                Some(&event.event_id), "projection claims active at or after the exact revocation epoch");
                        }
                    }
                }

                if read.decision == ProjectionDecision::TrustedActive &&
                    (read.projection_state != ProjectionState::Active || stale_checkpoint ||
                     read.projection_state == ProjectionState::Conflicting ||
                     read.projection_state == ProjectionState::Unknown) {
                    finding(&mut findings, "FALSE_FRESH_ACTIVE_PROJECTION", "violation",
                        Some(&event.event_id), "trusted-active result conflicts with state or required freshness");
                }
                if read.decision == ProjectionDecision::TrustedRevoked &&
                    (read.projection_state != ProjectionState::Revoked || stale_checkpoint ||
                     read.projection_state == ProjectionState::Conflicting ||
                     read.projection_state == ProjectionState::Unknown) {
                    finding(&mut findings, "FALSE_FRESH_REVOKED_PROJECTION", "violation",
                        Some(&event.event_id), "trusted-revoked result conflicts with state or required freshness");
                }
            }
            EventKind::RemoteSignal(signal) => {
                counts.remote_signals += 1;
                let event_type_valid = !signal.event_type.trim().is_empty()
                    && signal.event_type.len() <= 128
                    && signal.event_type.as_bytes()[0].is_ascii_alphabetic()
                    && signal.event_type.bytes().all(|byte| {
                        byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.')
                    });
                if !event_type_valid {
                    finding(&mut findings, "REMOTE_SIGNAL_EVENT_TYPE_INVALID", "incomplete",
                        Some(&event.event_id), "remote signal event_type must be a non-empty bounded identifier");
                }
                if signal.authorizes_operation {
                    finding(&mut findings, "REMOTE_SIGNAL_USED_AS_AUTHORITY", "violation",
                        Some(&event.event_id), "signals are notifications, never capabilities or authorization decisions");
                }
                if signal.trust_class != SignalTrustClass::VerifiedRecord && signal.trusted_state_mutation {
                    finding(&mut findings, "UNVERIFIED_SIGNAL_MUTATED_TRUSTED_STATE", "violation",
                        Some(&event.event_id), "unverified or advisory signal cannot mutate trusted UI/application state");
                }
                match signal.trust_class {
                    SignalTrustClass::VerifiedRecord => {
                        if event.source != EvidenceSource::Observer {
                            finding(&mut findings, "VERIFIED_SIGNAL_NOT_OBSERVER_ATTESTED", "incomplete",
                                Some(&event.event_id), "VerifiedRecord classification must be emitted by the observer after backing-record validation, not by the remote sender or client");
                        }
                        let digest_valid = signal.verified_payload_sha256.as_deref().is_some_and(|digest| {
                            digest.len() == 64
                                && digest.bytes().all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
                        });
                        match (
                            &signal.claimed_sender,
                            &signal.transport_sender,
                            &signal.backing_action_hash,
                            &signal.record_author,
                            &signal.verifier_build,
                        ) {
                            (Some(claimed), Some(transport), Some(hash), Some(author), Some(verifier))
                                if claimed == transport
                                    && transport == author
                                    && !hash.trim().is_empty()
                                    && !verifier.trim().is_empty()
                                    && digest_valid => {}
                            _ => finding(&mut findings, "VERIFIED_SIGNAL_EVIDENCE_INCOMPLETE", "incomplete",
                                Some(&event.event_id), "VerifiedRecord requires matching claimed/transport/record author, exact backing ActionHash, canonical payload SHA-256, verifier build and observer-plane disposition"),
                        }
                    }
                    SignalTrustClass::AuthenticatedPeerHint => {
                        match (&signal.claimed_sender, &signal.transport_sender) {
                            (Some(claimed), Some(transport)) if claimed == transport => {}
                            _ => finding(&mut findings, "AUTHENTICATED_HINT_SENDER_MISMATCH", "incomplete",
                                Some(&event.event_id), "authenticated peer hint requires a transport-bound sender matching the payload claim"),
                        }
                    }
                    SignalTrustClass::LocalOnly => {
                        if !matches!(event.source, EvidenceSource::Coordinator | EvidenceSource::Client) ||
                            signal.claimed_sender.is_some() || signal.transport_sender.is_some() {
                            finding(&mut findings, "LOCAL_SIGNAL_HAS_REMOTE_PROVENANCE", "incomplete",
                                Some(&event.event_id), "local-only signal must originate from the local coordinator/client plane and omit remote sender claims");
                        }
                    }
                    SignalTrustClass::UnverifiedHint | SignalTrustClass::Unknown | SignalTrustClass::Rejected => {}
                }
            }
            EventKind::CallerOutcome(outcome) => {
                if outcome.operation_id.trim().is_empty() || outcome.attempt_id == 0 {
                    finding(&mut findings, "CALLER_OUTCOME_IDENTITY_INCOMPLETE", "incomplete",
                        Some(&event.event_id), "caller outcome requires logical operation and attempt IDs");
                }
                if outcome.outcome == CallerOutcomeKind::Unknown {
                    counts.caller_outcomes_unknown += 1;
                }
                let key = (outcome.operation_id.clone(), outcome.attempt_id);
                match caller_outcomes.get(&key).copied() {
                    None => {
                        caller_outcomes.insert(key, outcome.outcome);
                    }
                    Some(previous) if previous == outcome.outcome => {}
                    Some(CallerOutcomeKind::Acknowledged | CallerOutcomeKind::Unknown) => {
                        // A later reconciliation may resolve an acknowledgement or
                        // timeout to a definitive outcome for the same attempt.
                        caller_outcomes.insert(key, outcome.outcome);
                    }
                    Some(previous) => {
                        finding(&mut findings, "CALLER_OUTCOME_CONTRADICTION", "violation",
                            Some(&event.event_id),
                            format!("attempt outcome changed from {:?} to {:?}", previous, outcome.outcome));
                    }
                }
            }
        }
    }

    if !saw_grant || !saw_host_call {
        finding(&mut findings, "LIFECYCLE_TRACE_INCOMPLETE", "incomplete", None,
            "trace must contain at least one grant issuance and one host call");
    }
    for ((operation_id, attempt_id), outcome) in caller_outcomes {
        let has_effect = effects_by_attempt.contains(&(operation_id.clone(), attempt_id));
        if matches!(outcome, CallerOutcomeKind::NotCommitted | CallerOutcomeKind::Rejected) && has_effect {
            finding(&mut findings, "NOT_COMMITTED_CONTRADICTS_ATTEMPT_EFFECT", "violation", None,
                format!("operation {operation_id} attempt {attempt_id} was reported {:?} but has a durable effect", outcome));
        }
        if outcome == CallerOutcomeKind::Committed && !has_effect {
            finding(&mut findings, "COMMITTED_OUTCOME_WITHOUT_EFFECT_EVIDENCE", "incomplete", None,
                format!("operation {operation_id} attempt {attempt_id} was reported committed without a matching durable effect receipt"));
        }
    }

    let has_violation = findings.iter().any(|f| f.severity == "violation");
    let has_incomplete = findings.iter().any(|f| f.severity == "incomplete");
    let status = if has_violation { Status::Fail }
        else if has_incomplete { Status::Incomplete }
        else { Status::Pass };

    AuditReport {
        schema_version: 1,
        status,
        trace_id: bundle.trace_id.clone(),
        evidence_class: bundle.evidence_class,
        input_sha256,
        counts,
        findings,
        claim_ceiling: "STRUCTURAL_ONLY: this verifier checks internal consistency of redacted trace fields; it does not cryptographically authenticate receipts, prove conductor honesty, or establish Holochain runtime semantics".to_string(),
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut input: Option<PathBuf> = None;
    let mut output: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--input" => input = Some(PathBuf::from(args.next().ok_or("--input requires a path")?)),
            "--output" => output = Some(PathBuf::from(args.next().ok_or("--output requires a path")?)),
            "--help" | "-h" => {
                println!("pulse-lifecycle-audit --input TRACE.json [--output REPORT.json]");
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}").into()),
        }
    }
    let input = input.ok_or("--input is required")?;
    let raw = fs::read(&input)?;
    let digest = format!("{:x}", Sha256::digest(&raw));
    let bundle: TraceBundle = serde_json::from_slice(&raw)?;
    let report = audit(&bundle, digest);
    let json = serde_json::to_string_pretty(&report)?;
    if let Some(path) = output {
        if let Some(parent) = path.parent() { fs::create_dir_all(parent)?; }
        fs::write(path, &json)?;
    } else {
        println!("{json}");
    }
    match report.status {
        Status::Pass => Ok(()),
        Status::Fail => Err("trace invariant violation".into()),
        Status::Incomplete => Err("trace evidence incomplete".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> TraceBundle {
        serde_json::from_str(include_str!("../examples/safe-trace.json")).unwrap()
    }

    fn run(bundle: &TraceBundle) -> AuditReport {
        audit(bundle, "0".repeat(64))
    }

    #[test]
    fn safe_trace_passes_with_unknown_ack_and_stale_projection() {
        let report = run(&fixture());
        assert_eq!(report.status, Status::Pass, "{:?}", report.findings);
        assert_eq!(report.counts.grants_issued, 1);
        assert_eq!(report.counts.grants_revoked, 1);
        assert_eq!(report.counts.domain_effects, 1);
        assert_eq!(report.counts.unknown_projection_reads, 1);
        assert_eq!(report.counts.caller_outcomes_unknown, 1);
    }

    #[test]
    fn duplicate_grant_retry_for_one_idempotency_key_fails() {
        let mut bundle = fixture();
        let mut duplicate = bundle.events[0].clone();
        duplicate.seq = 10;
        duplicate.event_id = "grant-retry-duplicate".to_string();
        if let EventKind::GrantIssued(ref mut grant) = duplicate.kind {
            grant.grant_ref = "grant-B".to_string();
            grant.grant_action_hash = "action-grant-B".to_string();
            grant.authority_index = 5;
        }
        bundle.events.push(duplicate);
        assert_eq!(run(&bundle).status, Status::Fail);
        assert!(run(&bundle).findings.iter().any(|f| f.code == "IDEMPOTENCY_KEY_MINTED_MULTIPLE_GRANTS"));
    }

    #[test]
    fn allow_after_host_revocation_fails() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::HostCall(_)) &&
            e.seq == 6).unwrap();
        if let EventKind::HostCall(ref mut call) = event.kind {
            call.authority_index = 5;
            call.host_decision = HostDecision::Authorized;
            call.host_state_at_decision = HostGrantState::Active;
            call.domain_decision = DomainDecision::Allow;
            call.projection_used_for_authorization = true;
        }
        assert_eq!(run(&bundle).status, Status::Fail);
        let report = run(&bundle);
        assert!(report.findings.iter().any(|f| f.code == "PROJECTION_USED_AS_AUTHORITY"));
        assert!(report.findings.iter().any(|f| f.code == "HOST_STATE_TRACE_CONFLICT" ||
            f.code == "ALLOW_WITH_INACTIVE_HOST_GRANT"));
    }

    #[test]
    fn unverified_signal_cannot_mutate_trusted_state() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::RemoteSignal(_))).unwrap();
        if let EventKind::RemoteSignal(ref mut signal) = event.kind {
            signal.trusted_state_mutation = true;
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Fail);
        assert!(report.findings.iter().any(|f| f.code == "UNVERIFIED_SIGNAL_MUTATED_TRUSTED_STATE"));
    }

    #[test]
    fn stale_checkpoint_must_be_unknown() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::ProjectionRead(_))).unwrap();
        if let EventKind::ProjectionRead(ref mut read) = event.kind {
            read.decision = ProjectionDecision::TrustedActive;
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Fail);
        assert!(report.findings.iter().any(|f| f.code == "STALE_CHECKPOINT_READ_NOT_UNKNOWN"));
    }

    #[test]
    fn coordinator_assertion_cannot_substitute_for_host_receipt() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::HostCall(_))).unwrap();
        event.source = EvidenceSource::Coordinator;
        let report = run(&bundle);
        assert_eq!(report.status, Status::Incomplete);
        assert!(report.findings.iter().any(|f| f.code == "EVIDENCE_PLANE_MISMATCH"));
    }

    #[test]
    fn stale_unconstrained_projection_cannot_be_labeled_trusted_active() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::ProjectionRead(_))).unwrap();
        if let EventKind::ProjectionRead(ref mut read) = event.kind {
            read.checkpoint_used = false;
            read.minimum_epoch = 0;
            read.decision = ProjectionDecision::TrustedActive;
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Fail);
        assert!(report.findings.iter().any(|f| f.code == "STALE_PROJECTION_LABELED_TRUSTED"));
        assert!(report.findings.iter().any(|f| f.code == "STALE_READ_NOT_MARKED_ADVISORY"));
    }

    #[test]
    fn projection_read_must_reference_an_exact_issued_grant() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::ProjectionRead(_))).unwrap();
        if let EventKind::ProjectionRead(ref mut read) = event.kind {
            read.grant_ref = "grant-not-in-trace".to_string();
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Incomplete);
        assert!(report.findings.iter().any(|f| f.code == "PROJECTION_READ_WITHOUT_GRANT"));
    }

    #[test]
    fn caller_not_committed_is_scoped_to_the_exact_attempt() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::CallerOutcome(_))).unwrap();
        if let EventKind::CallerOutcome(ref mut outcome) = event.kind {
            outcome.attempt_id = 2;
            outcome.outcome = CallerOutcomeKind::NotCommitted;
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Pass, "{:?}", report.findings);
    }

    #[test]
    fn not_committed_outcome_for_effected_attempt_fails() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::CallerOutcome(_))).unwrap();
        if let EventKind::CallerOutcome(ref mut outcome) = event.kind {
            outcome.outcome = CallerOutcomeKind::NotCommitted;
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Fail);
        assert!(report.findings.iter().any(|f| f.code == "NOT_COMMITTED_CONTRADICTS_ATTEMPT_EFFECT"));
    }

    #[test]
    fn unknown_caller_outcome_can_reconcile_to_committed_for_the_same_attempt() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::CallerOutcome(_))).unwrap();
        if let EventKind::CallerOutcome(ref mut outcome) = event.kind {
            outcome.outcome = CallerOutcomeKind::Committed;
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Pass, "{:?}", report.findings);
    }

    #[test]
    fn remote_signal_event_type_must_be_a_bounded_identifier() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::RemoteSignal(_))).unwrap();
        if let EventKind::RemoteSignal(ref mut signal) = event.kind {
            signal.event_type = "not a valid type with spaces".to_string();
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Incomplete);
        assert!(report.findings.iter().any(|f| f.code == "REMOTE_SIGNAL_EVENT_TYPE_INVALID"));
    }

    #[test]
    fn verified_record_signal_requires_observer_attestation() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::RemoteSignal(_))).unwrap();
        if let EventKind::RemoteSignal(ref mut signal) = event.kind {
            signal.trust_class = SignalTrustClass::VerifiedRecord;
            signal.claimed_sender = Some("agent-owner".to_string());
            signal.transport_sender = Some("agent-owner".to_string());
            signal.backing_action_hash = Some("action-attestation-1".to_string());
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Incomplete);
        assert!(report.findings.iter().any(|f| f.code == "VERIFIED_SIGNAL_NOT_OBSERVER_ATTESTED"));
    }

    #[test]
    fn verified_record_signal_requires_record_author_digest_and_verifier_build() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::RemoteSignal(_))).unwrap();
        event.source = EvidenceSource::Observer;
        if let EventKind::RemoteSignal(ref mut signal) = event.kind {
            signal.trust_class = SignalTrustClass::VerifiedRecord;
            signal.claimed_sender = Some("agent-owner".to_string());
            signal.transport_sender = Some("agent-owner".to_string());
            signal.backing_action_hash = Some("action-attestation-1".to_string());
            signal.record_author = Some("agent-attacker".to_string());
            signal.verified_payload_sha256 = Some("a".repeat(64));
            signal.verifier_build = Some("pulse-record-verifier/1".to_string());
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Incomplete);
        assert!(report.findings.iter().any(|f| f.code == "VERIFIED_SIGNAL_EVIDENCE_INCOMPLETE"));
    }

    #[test]
    fn complete_observer_record_receipt_satisfies_structural_verified_record_contract() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::RemoteSignal(_))).unwrap();
        event.source = EvidenceSource::Observer;
        if let EventKind::RemoteSignal(ref mut signal) = event.kind {
            signal.trust_class = SignalTrustClass::VerifiedRecord;
            signal.claimed_sender = Some("agent-owner".to_string());
            signal.transport_sender = Some("agent-owner".to_string());
            signal.backing_action_hash = Some("action-attestation-1".to_string());
            signal.record_author = Some("agent-owner".to_string());
            signal.verified_payload_sha256 = Some("a".repeat(64));
            signal.verifier_build = Some("pulse-record-verifier/1".to_string());
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Pass, "{:?}", report.findings);
    }

    #[test]
    fn authenticated_peer_hint_must_bind_claimed_sender_to_transport_sender() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::RemoteSignal(_))).unwrap();
        if let EventKind::RemoteSignal(ref mut signal) = event.kind {
            signal.trust_class = SignalTrustClass::AuthenticatedPeerHint;
            signal.claimed_sender = Some("agent-owner".to_string());
            signal.transport_sender = Some("agent-attacker".to_string());
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Incomplete);
        assert!(report.findings.iter().any(|f| f.code == "AUTHENTICATED_HINT_SENDER_MISMATCH"));
    }

    #[test]
    fn local_only_signal_cannot_claim_a_remote_sender() {
        let mut bundle = fixture();
        let event = bundle.events.iter_mut().find(|e| matches!(&e.kind, EventKind::RemoteSignal(_))).unwrap();
        event.source = EvidenceSource::Coordinator;
        if let EventKind::RemoteSignal(ref mut signal) = event.kind {
            signal.trust_class = SignalTrustClass::LocalOnly;
            signal.claimed_sender = Some("agent-owner".to_string());
            signal.transport_sender = None;
        }
        let report = run(&bundle);
        assert_eq!(report.status, Status::Incomplete);
        assert!(report.findings.iter().any(|f| f.code == "LOCAL_SIGNAL_HAS_REMOTE_PROVENANCE"));
    }
}

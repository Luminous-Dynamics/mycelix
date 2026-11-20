//! Independently verified per-operation outcome ledger for MCRB.
//!
//! A valid offer ledger fixes the denominator. This module binds one signed
//! outcome proposition to every offer and validates each assertion against a
//! protected source trust policy. It does not itself observe physical truth:
//! it qualifies signed evidence, source roles and declared dependency closure.

use base64::{engine::general_purpose::STANDARD, Engine as _};
use ed25519_dalek::{Signature, VerifyingKey};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

use crate::offer_ledger::VerifiedOfferLedger;

pub const OUTCOME_LEDGER_SCHEMA_VERSION: &str = "mycelix-mcrb-outcome-ledger-v0.1";
pub const OUTCOME_TRUST_POLICY_SCHEMA_VERSION: &str = "mycelix-mcrb-outcome-trust-policy-v0.1";
pub const OUTCOME_ATTESTATION_SCHEMA_VERSION: &str = "mycelix-mcrb-outcome-attestation-v0.1";
pub const OUTCOME_PAYLOAD_TYPE: &str = "application/vnd.in-toto+json";
pub const IN_TOTO_STATEMENT_V1: &str = "https://in-toto.io/Statement/v1";
pub const OUTCOME_PREDICATE_TYPE: &str =
    "https://luminousdynamics.io/attestations/mcrb-operation-outcome/v0.1";
const LEDGER_DOMAIN: &[u8] = b"MYCELIX-MCRB-OUTCOME-LEDGER\0v0.1\0";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum OutcomeClass {
    QualifiedSuccess,
    CorrectDenial,
    UnexpectedDenial,
    Failed,
    Unknown,
    DuplicateEffect,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EffectState {
    Applied,
    ConfirmedNotApplied,
    Partial,
    Unknown,
    NotAttempted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EvidenceKind {
    ApplicationRecord,
    ProviderAcknowledgement,
    ProviderStatusQuery,
    IndependentObservation,
    PolicyDecision,
    FailureTrace,
    Reconciliation,
    CrossPeerRead,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum OutcomeSourceRole {
    EffectExecutor,
    Provider,
    PolicyEngine,
    IndependentObserver,
    ReconciliationAuthority,
    Telemetry,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SourceRelationship {
    QualifiedIndependent,
    KnownSharedDependency,
    ProviderReportedOnly,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutcomeSourceIdentity {
    pub commit: String,
    pub tree_sha: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutcomeMeasurementWindow {
    pub window_id: String,
    pub clock_id: String,
    pub clock_epoch: String,
    pub start_tick: u64,
    pub end_tick: u64,
    pub ticks_per_second: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutcomeLedgerEntry {
    pub sequence: u64,
    pub operation_id: String,
    pub request_digest_sha256: String,
    pub idempotency_key_digest_sha256: Option<String>,
    pub effect_state: EffectState,
    pub attestations: Vec<OutcomeDsseEnvelope>,
    pub prior_outcome_ledger_digest_sha256: Option<String>,
    pub reconciliation_sequence: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutcomeLedgerEnvelope {
    pub schema_version: String,
    pub run_id: String,
    pub scenario_id: String,
    pub scenario_contract_digest_sha256: String,
    pub source: OutcomeSourceIdentity,
    pub runtime_identity_digest_sha256: String,
    pub topology_profile_digest_sha256: String,
    pub offer_ledger_digest_sha256: String,
    pub measurement_window: OutcomeMeasurementWindow,
    pub trust_policy_id: String,
    pub trust_policy_digest_sha256: String,
    pub entry_count: u64,
    pub entries: Vec<OutcomeLedgerEntry>,
    pub ledger_digest_sha256: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutcomeAttestationPredicate {
    pub schema_version: String,
    pub run_id: String,
    pub scenario_id: String,
    pub scenario_contract_digest_sha256: String,
    pub source_commit: String,
    pub source_tree_sha: String,
    pub runtime_identity_digest_sha256: String,
    pub topology_profile_digest_sha256: String,
    pub offer_ledger_digest_sha256: String,
    pub operation_id: String,
    pub operation_sequence: u64,
    pub request_digest_sha256: String,
    pub idempotency_key_digest_sha256: Option<String>,
    pub outcome: OutcomeClass,
    pub effect_state: EffectState,
    pub evidence_kind: EvidenceKind,
    pub evidence_artifact_digest_sha256: String,
    pub source_id: String,
    pub source_role: OutcomeSourceRole,
    pub builder_identity: String,
    pub verifier_identity: String,
    pub verifier_version: String,
    pub verifier_binary_digest_sha256: String,
    pub trust_policy_id: String,
    pub trust_policy_digest_sha256: String,
    pub issued_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutcomeDsseSignature {
    pub keyid: String,
    pub sig: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct OutcomeDsseEnvelope {
    pub payload_type: String,
    pub payload: String,
    pub signatures: Vec<OutcomeDsseSignature>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutcomeInTotoSubject {
    pub name: String,
    pub digest: BTreeMap<String, String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutcomeInTotoStatement {
    #[serde(rename = "_type")]
    pub statement_type: String,
    pub subject: Vec<OutcomeInTotoSubject>,
    #[serde(rename = "predicateType")]
    pub predicate_type: String,
    pub predicate: OutcomeAttestationPredicate,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrustedOutcomeSource {
    pub source_id: String,
    pub key_id: String,
    pub verifier_identity: String,
    pub public_key_hex: String,
    pub source_role: OutcomeSourceRole,
    pub dependency_ids: Vec<String>,
    pub dependency_closure_complete: bool,
    pub allowed_evidence_kinds: Vec<EvidenceKind>,
    pub allowed_effect_states: Vec<EffectState>,
    pub allowed_builder_ids: Vec<String>,
    pub valid_from_unix_ms: u64,
    pub expires_at_unix_ms: u64,
    pub revoked: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutcomeTrustPolicyDocument {
    pub schema_version: String,
    pub policy_id: String,
    pub policy_owner: String,
    pub valid_from_unix_ms: u64,
    pub expires_at_unix_ms: u64,
    pub effect_dependency_ids: Vec<String>,
    pub effect_dependency_closure_complete: bool,
    pub sources: Vec<TrustedOutcomeSource>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedOutcomeTrustPolicy {
    document: OutcomeTrustPolicyDocument,
    digest_sha256: String,
}

impl VerifiedOutcomeTrustPolicy {
    /// The digest pin must come from protected verifier configuration outside
    /// the benchmark runner's mutable input. Hashing a policy does not decide
    /// who is authorized to change the externally pinned trust root.
    pub fn load_pinned(
        document: OutcomeTrustPolicyDocument,
        expected_digest_sha256: &str,
    ) -> Result<Self, OutcomeTrustPolicyError> {
        if document.schema_version != OUTCOME_TRUST_POLICY_SCHEMA_VERSION {
            return Err(OutcomeTrustPolicyError::UnsupportedSchema);
        }
        if document.policy_id.trim().is_empty() || document.policy_owner.trim().is_empty() {
            return Err(OutcomeTrustPolicyError::MissingPolicyIdentity);
        }
        if document.valid_from_unix_ms >= document.expires_at_unix_ms {
            return Err(OutcomeTrustPolicyError::InvalidPolicyWindow);
        }
        if !unique_nonempty(&document.effect_dependency_ids) {
            return Err(OutcomeTrustPolicyError::InvalidEffectDependencySet);
        }

        let mut source_ids = BTreeSet::new();
        let mut key_ids = BTreeSet::new();
        for source in &document.sources {
            if source.source_id.trim().is_empty()
                || source.key_id.trim().is_empty()
                || source.verifier_identity.trim().is_empty()
                || source.allowed_evidence_kinds.is_empty()
                || source.allowed_effect_states.is_empty()
                || source.allowed_builder_ids.is_empty()
                || source.valid_from_unix_ms >= source.expires_at_unix_ms
                || !unique_nonempty(&source.dependency_ids)
                || !unique_nonempty(&source.allowed_builder_ids)
            {
                return Err(OutcomeTrustPolicyError::InvalidSource(source.source_id.clone()));
            }
            if !source_ids.insert(source.source_id.clone()) {
                return Err(OutcomeTrustPolicyError::DuplicateSourceId(source.source_id.clone()));
            }
            if !key_ids.insert(source.key_id.clone()) {
                return Err(OutcomeTrustPolicyError::DuplicateKeyId(source.key_id.clone()));
            }
            decode_hex_32(&source.public_key_hex)
                .map_err(|_| OutcomeTrustPolicyError::InvalidPublicKey(source.key_id.clone()))?;
        }

        let canonical = serde_json::to_vec(&document)
            .map_err(|error| OutcomeTrustPolicyError::Serialization(error.to_string()))?;
        let actual = sha256_hex(&canonical);
        if !valid_sha256_hex(expected_digest_sha256) || actual != expected_digest_sha256 {
            return Err(OutcomeTrustPolicyError::DigestMismatch {
                expected: expected_digest_sha256.to_string(),
                actual,
            });
        }

        Ok(Self {
            document,
            digest_sha256: expected_digest_sha256.to_string(),
        })
    }

    pub fn policy_id(&self) -> &str { &self.document.policy_id }
    pub fn digest_sha256(&self) -> &str { &self.digest_sha256 }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OutcomeTrustPolicyError {
    UnsupportedSchema,
    MissingPolicyIdentity,
    InvalidPolicyWindow,
    InvalidEffectDependencySet,
    InvalidSource(String),
    DuplicateSourceId(String),
    DuplicateKeyId(String),
    InvalidPublicKey(String),
    Serialization(String),
    DigestMismatch { expected: String, actual: String },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OutcomeVerificationContext {
    pub run_id: String,
    pub scenario_id: String,
    pub scenario_contract_digest_sha256: String,
    pub source_commit: String,
    pub source_tree_sha: String,
    pub runtime_identity_digest_sha256: String,
    pub topology_profile_digest_sha256: String,
    pub now_unix_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutcomeLedgerStatus {
    Valid,
    Invalid,
    Incomplete,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OutcomeLedgerIssue {
    UnsupportedSchema,
    EmptyRunId,
    UnknownScenario(String),
    ContextMismatch(&'static str),
    OfferLedgerMismatch,
    MeasurementWindowMismatch,
    TrustPolicyMismatch,
    InvalidDigest(&'static str),
    LedgerDigestMismatch,
    EntryCountMismatch,
    EmptyLedger,
    InvalidSequence { index: usize, expected: u64, actual: u64 },
    OperationBindingMismatch { sequence: u64, field: &'static str },
    DuplicateOperation(String),
    MissingOperation(String),
    MissingAttestation(String),
    InvalidReconciliationLineage(String),
    WrongPayloadType { operation_id: String },
    InvalidPayload(String),
    NonCanonicalPayload(String),
    UnsupportedPredicate(String),
    InvalidSubject(String),
    WrongSignatureCount { operation_id: String, count: usize },
    UnknownSource(String),
    RevokedSource(String),
    SourceNotYetValid(String),
    ExpiredSource(String),
    UnauthorizedEvidenceKind(String),
    UnauthorizedEffectState(String),
    UnauthorizedBuilder(String),
    SourceIdentityMismatch(String),
    PolicyMismatch(String),
    InvalidSignature(String),
    InvalidEvidenceKindForOutcome { operation_id: String, evidence_kind: EvidenceKind },
    InvalidEffectStateForOutcome { operation_id: String, outcome: OutcomeClass, effect_state: EffectState },
    SharedDependencyCannotSupportSuccess(String),
    UnknownDependencyClosureCannotSupportSuccess(String),
    DuplicateSourceAttestation { operation_id: String, source_id: String },
    ConflictingAttestation { operation_id: String },
    DuplicateEffect(String),
    InvalidOutcomeSequence(String),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OutcomeLedgerReport {
    pub status: OutcomeLedgerStatus,
    pub run_id: String,
    pub scenario_id: String,
    pub outcome_ledger_digest_sha256: Option<String>,
    pub offer_ledger_digest_sha256: String,
    pub trust_policy_id: String,
    pub trust_policy_digest_sha256: String,
    pub entry_count: usize,
    pub attestation_count: usize,
    pub qualified_successes: usize,
    pub correct_denials: usize,
    pub unexpected_denials: usize,
    pub failures: usize,
    pub unknown_outcomes: usize,
    pub duplicate_effects: usize,
    pub issues: Vec<OutcomeLedgerIssue>,
}

/// Opaque verifier capability. Only verify_outcome_ledger can construct it.
/// It contains the exact per-offer classification and cannot be made from a
/// caller-supplied Vec<OperationOutcomeSample>.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedOutcomeLedger {
    run_id: String,
    scenario_id: String,
    scenario_contract_digest_sha256: String,
    source_commit: String,
    source_tree_sha: String,
    runtime_identity_digest_sha256: String,
    topology_profile_digest_sha256: String,
    offer_ledger_digest_sha256: String,
    ledger_digest_sha256: String,
    trust_policy_id: String,
    trust_policy_digest_sha256: String,
    entry_count: usize,
    attestation_count: usize,
    qualified_successes: usize,
    correct_denials: usize,
    unexpected_denials: usize,
    failures: usize,
    unknown_outcomes: usize,
    duplicate_effects: usize,
    operation_outcomes: Vec<(String, OutcomeClass)>,
}

impl VerifiedOutcomeLedger {
    pub fn run_id(&self) -> &str { &self.run_id }
    pub fn scenario_id(&self) -> &str { &self.scenario_id }
    pub fn scenario_contract_digest_sha256(&self) -> &str { &self.scenario_contract_digest_sha256 }
    pub fn source_commit(&self) -> &str { &self.source_commit }
    pub fn source_tree_sha(&self) -> &str { &self.source_tree_sha }
    pub fn runtime_identity_digest_sha256(&self) -> &str { &self.runtime_identity_digest_sha256 }
    pub fn topology_profile_digest_sha256(&self) -> &str { &self.topology_profile_digest_sha256 }
    pub fn offer_ledger_digest_sha256(&self) -> &str { &self.offer_ledger_digest_sha256 }
    pub fn ledger_digest_sha256(&self) -> &str { &self.ledger_digest_sha256 }
    pub fn trust_policy_id(&self) -> &str { &self.trust_policy_id }
    pub fn trust_policy_digest_sha256(&self) -> &str { &self.trust_policy_digest_sha256 }
    pub fn entry_count(&self) -> usize { self.entry_count }
    pub fn attestation_count(&self) -> usize { self.attestation_count }
    pub fn qualified_successes(&self) -> usize { self.qualified_successes }
    pub fn correct_denials(&self) -> usize { self.correct_denials }
    pub fn unexpected_denials(&self) -> usize { self.unexpected_denials }
    pub fn failures(&self) -> usize { self.failures }
    pub fn unknown_outcomes(&self) -> usize { self.unknown_outcomes }
    pub fn duplicate_effects(&self) -> usize { self.duplicate_effects }
    pub fn operation_outcomes(&self) -> &[(String, OutcomeClass)] { &self.operation_outcomes }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OutcomeLedgerVerification {
    Verified(VerifiedOutcomeLedger),
    Rejected(OutcomeLedgerReport),
}

impl OutcomeLedgerVerification {
    pub fn verified(&self) -> Option<&VerifiedOutcomeLedger> {
        match self { Self::Verified(value) => Some(value), Self::Rejected(_) => None }
    }
    pub fn report(&self) -> Option<&OutcomeLedgerReport> {
        match self { Self::Verified(_) => None, Self::Rejected(report) => Some(report) }
    }
}

#[derive(Serialize)]
struct OutcomeLedgerDigestMaterial<'a> {
    schema_version: &'a str,
    run_id: &'a str,
    scenario_id: &'a str,
    scenario_contract_digest_sha256: &'a str,
    source: &'a OutcomeSourceIdentity,
    runtime_identity_digest_sha256: &'a str,
    topology_profile_digest_sha256: &'a str,
    offer_ledger_digest_sha256: &'a str,
    measurement_window: &'a OutcomeMeasurementWindow,
    trust_policy_id: &'a str,
    trust_policy_digest_sha256: &'a str,
    entry_count: u64,
    entries: &'a [OutcomeLedgerEntry],
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut hex = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut hex, "{byte:02x}");
    }
    hex
}

fn valid_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_git_sha(value: &str) -> bool {
    value.len() == 40 && value.bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn unique_nonempty(values: &[String]) -> bool {
    let mut seen = BTreeSet::new();
    values.iter().all(|value| !value.trim().is_empty() && seen.insert(value))
}

fn decode_hex_32(value: &str) -> Result<[u8; 32], ()> {
    if value.len() != 64 { return Err(()); }
    let mut bytes = [0_u8; 32];
    for (index, pair) in value.as_bytes().chunks_exact(2).enumerate() {
        let high = hex_nibble(pair[0]).ok_or(())?;
        let low = hex_nibble(pair[1]).ok_or(())?;
        bytes[index] = (high << 4) | low;
    }
    Ok(bytes)
}

fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        _ => None,
    }
}

fn dsse_pae(payload_type: &str, payload: &[u8]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(payload_type.len() + payload.len() + 64);
    bytes.extend_from_slice(b"DSSEv1 ");
    bytes.extend_from_slice(payload_type.len().to_string().as_bytes());
    bytes.push(b' ');
    bytes.extend_from_slice(payload_type.as_bytes());
    bytes.push(b' ');
    bytes.extend_from_slice(payload.len().to_string().as_bytes());
    bytes.push(b' ');
    bytes.extend_from_slice(payload);
    bytes
}

/// Compute a domain-separated digest over the typed, ordered ledger envelope.
/// The struct has no map-valued fields, so serde's declared field order is stable.
pub fn calculate_outcome_ledger_digest(envelope: &OutcomeLedgerEnvelope) -> Result<String, String> {
    let material = OutcomeLedgerDigestMaterial {
        schema_version: &envelope.schema_version,
        run_id: &envelope.run_id,
        scenario_id: &envelope.scenario_id,
        scenario_contract_digest_sha256: &envelope.scenario_contract_digest_sha256,
        source: &envelope.source,
        runtime_identity_digest_sha256: &envelope.runtime_identity_digest_sha256,
        topology_profile_digest_sha256: &envelope.topology_profile_digest_sha256,
        offer_ledger_digest_sha256: &envelope.offer_ledger_digest_sha256,
        measurement_window: &envelope.measurement_window,
        trust_policy_id: &envelope.trust_policy_id,
        trust_policy_digest_sha256: &envelope.trust_policy_digest_sha256,
        entry_count: envelope.entry_count,
        entries: &envelope.entries,
    };
    let payload = serde_json::to_vec(&material).map_err(|error| error.to_string())?;
    let mut bytes = Vec::with_capacity(LEDGER_DOMAIN.len() + payload.len() + 8);
    bytes.extend_from_slice(LEDGER_DOMAIN);
    bytes.extend_from_slice(&(payload.len() as u64).to_be_bytes());
    bytes.extend_from_slice(&payload);
    Ok(sha256_hex(&bytes))
}

fn classify_source_relationship(
    policy: &OutcomeTrustPolicyDocument,
    source: &TrustedOutcomeSource,
) -> SourceRelationship {
    if !policy.effect_dependency_closure_complete || !source.dependency_closure_complete {
        return SourceRelationship::Unknown;
    }
    let effect: BTreeSet<&str> = policy.effect_dependency_ids.iter().map(String::as_str).collect();
    if source.dependency_ids.iter().any(|dependency| effect.contains(dependency.as_str())) {
        return SourceRelationship::KnownSharedDependency;
    }
    if source.source_role == OutcomeSourceRole::IndependentObserver {
        SourceRelationship::QualifiedIndependent
    } else {
        SourceRelationship::ProviderReportedOnly
    }
}

fn verify_attestation(
    envelope: &OutcomeDsseEnvelope,
    entry: &OutcomeLedgerEntry,
    binding: &crate::offer_ledger::VerifiedOfferBinding,
    ledger: &OutcomeLedgerEnvelope,
    policy: &VerifiedOutcomeTrustPolicy,
    context: &OutcomeVerificationContext,
) -> Result<(OutcomeAttestationPredicate, SourceRelationship, String), OutcomeLedgerIssue> {
    let operation_id = entry.operation_id.clone();
    if envelope.payload_type != OUTCOME_PAYLOAD_TYPE {
        return Err(OutcomeLedgerIssue::WrongPayloadType { operation_id });
    }
    if envelope.signatures.len() != 1 {
        return Err(OutcomeLedgerIssue::WrongSignatureCount {
            operation_id,
            count: envelope.signatures.len(),
        });
    }
    let payload = STANDARD.decode(&envelope.payload)
        .map_err(|_| OutcomeLedgerIssue::InvalidPayload(entry.operation_id.clone()))?;
    let statement: OutcomeInTotoStatement = serde_json::from_slice(&payload)
        .map_err(|_| OutcomeLedgerIssue::InvalidPayload(entry.operation_id.clone()))?;
    if serde_json::to_vec(&statement)
        .map_err(|_| OutcomeLedgerIssue::InvalidPayload(entry.operation_id.clone()))? != payload
    {
        return Err(OutcomeLedgerIssue::NonCanonicalPayload(entry.operation_id.clone()));
    }
    if statement.statement_type != IN_TOTO_STATEMENT_V1
        || statement.predicate_type != OUTCOME_PREDICATE_TYPE
    {
        return Err(OutcomeLedgerIssue::UnsupportedPredicate(entry.operation_id.clone()));
    }
    let predicate = statement.predicate;
    let expected_subject = OutcomeInTotoSubject {
        name: format!("mcrb-evidence/{}/{}", entry.operation_id, predicate.evidence_kind.as_str()),
        digest: BTreeMap::from([("sha256".to_string(), predicate.evidence_artifact_digest_sha256.clone())]),
    };
    if statement.subject.as_slice() != [expected_subject] {
        return Err(OutcomeLedgerIssue::InvalidSubject(entry.operation_id.clone()));
    }

    if predicate.schema_version != OUTCOME_ATTESTATION_SCHEMA_VERSION {
        return Err(OutcomeLedgerIssue::UnsupportedPredicate(entry.operation_id.clone()));
    }
    let fields = [
        ("run_id", predicate.run_id.as_str(), ledger.run_id.as_str()),
        ("scenario_id", predicate.scenario_id.as_str(), ledger.scenario_id.as_str()),
        ("scenario_contract_digest_sha256", predicate.scenario_contract_digest_sha256.as_str(), ledger.scenario_contract_digest_sha256.as_str()),
        ("source_commit", predicate.source_commit.as_str(), context.source_commit.as_str()),
        ("source_tree_sha", predicate.source_tree_sha.as_str(), context.source_tree_sha.as_str()),
        ("runtime_identity_digest_sha256", predicate.runtime_identity_digest_sha256.as_str(), context.runtime_identity_digest_sha256.as_str()),
        ("topology_profile_digest_sha256", predicate.topology_profile_digest_sha256.as_str(), context.topology_profile_digest_sha256.as_str()),
        ("offer_ledger_digest_sha256", predicate.offer_ledger_digest_sha256.as_str(), ledger.offer_ledger_digest_sha256.as_str()),
        ("operation_id", predicate.operation_id.as_str(), entry.operation_id.as_str()),
        ("request_digest_sha256", predicate.request_digest_sha256.as_str(), binding.request_digest_sha256()),
        ("trust_policy_id", predicate.trust_policy_id.as_str(), policy.policy_id()),
        ("trust_policy_digest_sha256", predicate.trust_policy_digest_sha256.as_str(), policy.digest_sha256()),
    ];
    for (field, actual, expected) in fields {
        if actual != expected {
            return Err(OutcomeLedgerIssue::ContextMismatch(field));
        }
    }
    if predicate.operation_sequence != binding.sequence() {
        return Err(OutcomeLedgerIssue::ContextMismatch("operation_sequence"));
    }
    if predicate.idempotency_key_digest_sha256.as_deref() != binding.idempotency_key_digest_sha256() {
        return Err(OutcomeLedgerIssue::ContextMismatch("idempotency_key_digest_sha256"));
    }
    if predicate.effect_state != entry.effect_state {
        return Err(OutcomeLedgerIssue::ContextMismatch("effect_state"));
    }
    if !valid_sha256_hex(&predicate.evidence_artifact_digest_sha256)
        || !valid_sha256_hex(&predicate.verifier_binary_digest_sha256)
    {
        return Err(OutcomeLedgerIssue::InvalidDigest("attestation"));
    }
    if !valid_git_sha(&predicate.source_commit) || !valid_git_sha(&predicate.source_tree_sha) {
        return Err(OutcomeLedgerIssue::InvalidDigest("source"));
    }
    if predicate.builder_identity.trim().is_empty()
        || predicate.verifier_identity.trim().is_empty()
        || predicate.verifier_version.trim().is_empty()
        || predicate.source_id.trim().is_empty()
    {
        return Err(OutcomeLedgerIssue::InvalidPayload(entry.operation_id.clone()));
    }
    if predicate.issued_at_unix_ms > context.now_unix_ms {
        return Err(OutcomeLedgerIssue::SourceNotYetValid(predicate.source_id.clone()));
    }

    let dsse_signature = &envelope.signatures[0];
    let source = policy.document.sources.iter()
        .find(|source| source.key_id == dsse_signature.keyid && source.source_id == predicate.source_id)
        .ok_or_else(|| OutcomeLedgerIssue::UnknownSource(predicate.source_id.clone()))?;
    if source.revoked {
        return Err(OutcomeLedgerIssue::RevokedSource(source.source_id.clone()));
    }
    if context.now_unix_ms < policy.document.valid_from_unix_ms
        || predicate.issued_at_unix_ms < policy.document.valid_from_unix_ms
        || context.now_unix_ms < source.valid_from_unix_ms
        || predicate.issued_at_unix_ms < source.valid_from_unix_ms
    {
        return Err(OutcomeLedgerIssue::SourceNotYetValid(source.source_id.clone()));
    }
    if context.now_unix_ms > policy.document.expires_at_unix_ms
        || predicate.issued_at_unix_ms > policy.document.expires_at_unix_ms
        || context.now_unix_ms > source.expires_at_unix_ms
        || predicate.issued_at_unix_ms > source.expires_at_unix_ms
    {
        return Err(OutcomeLedgerIssue::ExpiredSource(source.source_id.clone()));
    }
    if source.verifier_identity != predicate.verifier_identity || source.source_role != predicate.source_role {
        return Err(OutcomeLedgerIssue::SourceIdentityMismatch(source.source_id.clone()));
    }
    if !source.allowed_evidence_kinds.contains(&predicate.evidence_kind) {
        return Err(OutcomeLedgerIssue::UnauthorizedEvidenceKind(source.source_id.clone()));
    }
    if !source.allowed_effect_states.contains(&predicate.effect_state) {
        return Err(OutcomeLedgerIssue::UnauthorizedEffectState(source.source_id.clone()));
    }
    if !source.allowed_builder_ids.contains(&predicate.builder_identity) {
        return Err(OutcomeLedgerIssue::UnauthorizedBuilder(source.source_id.clone()));
    }
    if source.dependency_ids.iter().any(|dependency| dependency.trim().is_empty()) {
        return Err(OutcomeLedgerIssue::InvalidPayload(entry.operation_id.clone()));
    }

    let public_key_bytes = decode_hex_32(&source.public_key_hex)
        .map_err(|_| OutcomeLedgerIssue::UnknownSource(source.source_id.clone()))?;
    let public_key = VerifyingKey::from_bytes(&public_key_bytes)
        .map_err(|_| OutcomeLedgerIssue::UnknownSource(source.source_id.clone()))?;
    let signature_bytes = STANDARD.decode(&dsse_signature.sig)
        .map_err(|_| OutcomeLedgerIssue::InvalidSignature(entry.operation_id.clone()))?;
    let signature = Signature::from_slice(&signature_bytes)
        .map_err(|_| OutcomeLedgerIssue::InvalidSignature(entry.operation_id.clone()))?;
    public_key.verify_strict(&dsse_pae(OUTCOME_PAYLOAD_TYPE, &payload), &signature)
        .map_err(|_| OutcomeLedgerIssue::InvalidSignature(entry.operation_id.clone()))?;

    let relationship = classify_source_relationship(&policy.document, source);
    let envelope_digest = sha256_hex(&serde_json::to_vec(envelope)
        .map_err(|_| OutcomeLedgerIssue::InvalidPayload(entry.operation_id.clone()))?);
    Ok((predicate, relationship, envelope_digest))
}

fn validate_outcome_semantics(
    entry: &OutcomeLedgerEntry,
    predicates: &[(OutcomeAttestationPredicate, SourceRelationship, String)],
    requires_cross_host: bool,
) -> Vec<OutcomeLedgerIssue> {
    let mut issues = Vec::new();
    let Some((first, _, _)) = predicates.first() else {
        issues.push(OutcomeLedgerIssue::MissingAttestation(entry.operation_id.clone()));
        return issues;
    };
    if predicates.iter().any(|(predicate, _, _)| {
        predicate.outcome != first.outcome || predicate.effect_state != first.effect_state
    }) {
        issues.push(OutcomeLedgerIssue::ConflictingAttestation {
            operation_id: entry.operation_id.clone(),
        });
        return issues;
    }

    let effect_valid = match first.outcome {
        OutcomeClass::QualifiedSuccess | OutcomeClass::DuplicateEffect => first.effect_state == EffectState::Applied,
        OutcomeClass::CorrectDenial | OutcomeClass::UnexpectedDenial => {
            matches!(first.effect_state, EffectState::ConfirmedNotApplied | EffectState::NotAttempted)
        }
        OutcomeClass::Failed => matches!(first.effect_state, EffectState::ConfirmedNotApplied | EffectState::Partial),
        OutcomeClass::Unknown => matches!(first.effect_state, EffectState::Unknown | EffectState::NotAttempted | EffectState::Partial),
    };
    if !effect_valid {
        issues.push(OutcomeLedgerIssue::InvalidEffectStateForOutcome {
            operation_id: entry.operation_id.clone(),
            outcome: first.outcome,
            effect_state: first.effect_state,
        });
    }

    let evidence_valid = match first.outcome {
        OutcomeClass::QualifiedSuccess => predicates.iter().any(|(predicate, relationship, _)| {
            let evidence_matches = if requires_cross_host {
                predicate.evidence_kind == EvidenceKind::CrossPeerRead
            } else {
                matches!(
                    predicate.evidence_kind,
                    EvidenceKind::IndependentObservation | EvidenceKind::ApplicationRecord | EvidenceKind::CrossPeerRead
                )
            };
            evidence_matches
                && predicate.source_role == OutcomeSourceRole::IndependentObserver
                && *relationship == SourceRelationship::QualifiedIndependent
        }),
        OutcomeClass::CorrectDenial | OutcomeClass::UnexpectedDenial => predicates.iter().any(|(predicate, _, _)| {
            predicate.evidence_kind == EvidenceKind::PolicyDecision
                && predicate.source_role == OutcomeSourceRole::PolicyEngine
        }),
        OutcomeClass::Failed => predicates.iter().any(|(predicate, _, _)| {
            matches!(predicate.evidence_kind, EvidenceKind::FailureTrace | EvidenceKind::Reconciliation)
        }),
        OutcomeClass::Unknown => true,
        OutcomeClass::DuplicateEffect => predicates.iter().any(|(predicate, relationship, _)| {
            matches!(predicate.evidence_kind, EvidenceKind::IndependentObservation | EvidenceKind::Reconciliation)
                && predicate.source_role == OutcomeSourceRole::IndependentObserver
                && *relationship == SourceRelationship::QualifiedIndependent
        }),
    };
    if !evidence_valid {
        issues.push(OutcomeLedgerIssue::InvalidEvidenceKindForOutcome {
            operation_id: entry.operation_id.clone(),
            evidence_kind: first.evidence_kind,
        });
    }

    if first.outcome == OutcomeClass::QualifiedSuccess
        && !predicates.iter().any(|(_, relationship, _)| *relationship == SourceRelationship::QualifiedIndependent)
    {
        if predicates.iter().any(|(_, relationship, _)| *relationship == SourceRelationship::KnownSharedDependency) {
            issues.push(OutcomeLedgerIssue::SharedDependencyCannotSupportSuccess(entry.operation_id.clone()));
        } else {
            issues.push(OutcomeLedgerIssue::UnknownDependencyClosureCannotSupportSuccess(entry.operation_id.clone()));
        }
    }
    if first.outcome == OutcomeClass::DuplicateEffect {
        issues.push(OutcomeLedgerIssue::DuplicateEffect(entry.operation_id.clone()));
    }
    issues
}

pub fn verify_outcome_ledger(
    envelope: &OutcomeLedgerEnvelope,
    offer: &VerifiedOfferLedger,
    policy: &VerifiedOutcomeTrustPolicy,
    context: &OutcomeVerificationContext,
) -> OutcomeLedgerVerification {
    let mut report = OutcomeLedgerReport {
        status: OutcomeLedgerStatus::Invalid,
        run_id: envelope.run_id.clone(),
        scenario_id: envelope.scenario_id.clone(),
        outcome_ledger_digest_sha256: None,
        offer_ledger_digest_sha256: offer.ledger_digest_sha256().to_string(),
        trust_policy_id: policy.policy_id().to_string(),
        trust_policy_digest_sha256: policy.digest_sha256().to_string(),
        entry_count: envelope.entries.len(),
        attestation_count: 0,
        qualified_successes: 0,
        correct_denials: 0,
        unexpected_denials: 0,
        failures: 0,
        unknown_outcomes: 0,
        duplicate_effects: 0,
        issues: Vec::new(),
    };
    let push = |report: &mut OutcomeLedgerReport, issue| {
        if !report.issues.contains(&issue) {
            report.issues.push(issue);
        }
    };

    if envelope.schema_version != OUTCOME_LEDGER_SCHEMA_VERSION {
        push(&mut report, OutcomeLedgerIssue::UnsupportedSchema);
    }
    if envelope.run_id.trim().is_empty() {
        push(&mut report, OutcomeLedgerIssue::EmptyRunId);
    }
    for (field, actual, expected) in [
        ("run_id", envelope.run_id.as_str(), offer.run_id()),
        ("scenario_id", envelope.scenario_id.as_str(), offer.scenario_id()),
        ("scenario_contract_digest_sha256", envelope.scenario_contract_digest_sha256.as_str(), offer.scenario_contract_digest_sha256()),
        ("offer_ledger_digest_sha256", envelope.offer_ledger_digest_sha256.as_str(), offer.ledger_digest_sha256()),
    ] {
        if actual != expected {
            push(&mut report, OutcomeLedgerIssue::ContextMismatch(field));
        }
    }
    for (field, actual, expected) in [
        ("run_id", envelope.run_id.as_str(), context.run_id.as_str()),
        ("scenario_id", envelope.scenario_id.as_str(), context.scenario_id.as_str()),
        ("scenario_contract_digest_sha256", envelope.scenario_contract_digest_sha256.as_str(), context.scenario_contract_digest_sha256.as_str()),
    ] {
        if actual != expected {
            push(&mut report, OutcomeLedgerIssue::ContextMismatch(field));
        }
    }
    if envelope.source.commit != context.source_commit || envelope.source.tree_sha != context.source_tree_sha {
        push(&mut report, OutcomeLedgerIssue::ContextMismatch("source_identity"));
    }
    if envelope.runtime_identity_digest_sha256 != context.runtime_identity_digest_sha256 {
        push(&mut report, OutcomeLedgerIssue::ContextMismatch("runtime_identity_digest_sha256"));
    }
    if envelope.topology_profile_digest_sha256 != context.topology_profile_digest_sha256 {
        push(&mut report, OutcomeLedgerIssue::ContextMismatch("topology_profile_digest_sha256"));
    }
    if envelope.trust_policy_id != policy.policy_id() || envelope.trust_policy_digest_sha256 != policy.digest_sha256() {
        push(&mut report, OutcomeLedgerIssue::TrustPolicyMismatch);
    }
    if envelope.measurement_window.window_id != offer.measurement_window_id()
        || envelope.measurement_window.clock_id != offer.monotonic_clock_id()
        || envelope.measurement_window.clock_epoch != offer.monotonic_clock_epoch()
        || envelope.measurement_window.start_tick != offer.measurement_start_ticks()
        || envelope.measurement_window.end_tick != offer.measurement_end_ticks()
        || envelope.measurement_window.ticks_per_second != offer.ticks_per_second()
    {
        push(&mut report, OutcomeLedgerIssue::MeasurementWindowMismatch);
    }
    if !valid_sha256_hex(&envelope.scenario_contract_digest_sha256)
        || !valid_sha256_hex(&envelope.runtime_identity_digest_sha256)
        || !valid_sha256_hex(&envelope.topology_profile_digest_sha256)
        || !valid_sha256_hex(&envelope.offer_ledger_digest_sha256)
        || !valid_sha256_hex(&envelope.trust_policy_digest_sha256)
    {
        push(&mut report, OutcomeLedgerIssue::InvalidDigest("ledger_context"));
    }
    if !valid_git_sha(&envelope.source.commit) || !valid_git_sha(&envelope.source.tree_sha) {
        push(&mut report, OutcomeLedgerIssue::InvalidDigest("source_identity"));
    }
    if envelope.entries.is_empty() {
        push(&mut report, OutcomeLedgerIssue::EmptyLedger);
    }
    if envelope.entry_count != envelope.entries.len() as u64 {
        push(&mut report, OutcomeLedgerIssue::EntryCountMismatch);
    }
    if envelope.entries.len() != offer.entry_count() {
        push(&mut report, OutcomeLedgerIssue::EntryCountMismatch);
    }

    let mut operations = BTreeSet::new();
    let mut operation_outcomes = Vec::with_capacity(envelope.entries.len());
    let mut attestation_digests = BTreeSet::new();
    for (index, entry) in envelope.entries.iter().enumerate() {
        let expected_sequence = index as u64 + 1;
        if entry.sequence != expected_sequence {
            push(&mut report, OutcomeLedgerIssue::InvalidSequence {
                index,
                expected: expected_sequence,
                actual: entry.sequence,
            });
        }
        if !operations.insert(entry.operation_id.clone()) {
            push(&mut report, OutcomeLedgerIssue::DuplicateOperation(entry.operation_id.clone()));
        }
        let Some(binding) = offer.operation_bindings().get(index) else {
            push(&mut report, OutcomeLedgerIssue::EntryCountMismatch);
            continue;
        };
        if entry.sequence != binding.sequence() {
            push(&mut report, OutcomeLedgerIssue::OperationBindingMismatch { sequence: entry.sequence, field: "sequence" });
        }
        if entry.operation_id != binding.operation_id() {
            push(&mut report, OutcomeLedgerIssue::OperationBindingMismatch { sequence: entry.sequence, field: "operation_id" });
        }
        if entry.request_digest_sha256 != binding.request_digest_sha256() {
            push(&mut report, OutcomeLedgerIssue::OperationBindingMismatch { sequence: entry.sequence, field: "request_digest_sha256" });
        }
        if entry.idempotency_key_digest_sha256.as_deref() != binding.idempotency_key_digest_sha256() {
            push(&mut report, OutcomeLedgerIssue::OperationBindingMismatch { sequence: entry.sequence, field: "idempotency_key_digest_sha256" });
        }
        if !valid_sha256_hex(&entry.request_digest_sha256)
            || entry.idempotency_key_digest_sha256.as_deref().is_some_and(|digest| !valid_sha256_hex(digest))
        {
            push(&mut report, OutcomeLedgerIssue::InvalidDigest("offer_binding"));
        }
        match (&entry.prior_outcome_ledger_digest_sha256, entry.reconciliation_sequence) {
            (None, None) => {}
            (Some(digest), Some(sequence)) if valid_sha256_hex(digest) && sequence > 1 => {}
            _ => push(&mut report, OutcomeLedgerIssue::InvalidReconciliationLineage(entry.operation_id.clone())),
        }

        if entry.attestations.is_empty() {
            push(&mut report, OutcomeLedgerIssue::MissingAttestation(entry.operation_id.clone()));
            continue;
        }
        let mut predicates = Vec::new();
        let mut seen_sources = BTreeSet::new();
        for attestation in &entry.attestations {
            match verify_attestation(attestation, entry, binding, envelope, policy, context) {
                Ok((predicate, relationship, digest)) => {
                    if !seen_sources.insert(predicate.source_id.clone()) {
                        push(&mut report, OutcomeLedgerIssue::DuplicateSourceAttestation {
                            operation_id: entry.operation_id.clone(),
                            source_id: predicate.source_id.clone(),
                        });
                    }
                    attestation_digests.insert(digest);
                    predicates.push((predicate, relationship, digest));
                    report.attestation_count += 1;
                }
                Err(issue) => push(&mut report, issue),
            }
        }
        for issue in validate_outcome_semantics(entry, &predicates, offer.requires_cross_host_visibility()) {
            push(&mut report, issue);
        }
        if let Some((predicate, _, _)) = predicates.first() {
            operation_outcomes.push((entry.operation_id.clone(), predicate.outcome));
            match predicate.outcome {
                OutcomeClass::QualifiedSuccess => report.qualified_successes += 1,
                OutcomeClass::CorrectDenial => report.correct_denials += 1,
                OutcomeClass::UnexpectedDenial => report.unexpected_denials += 1,
                OutcomeClass::Failed => report.failures += 1,
                OutcomeClass::Unknown => report.unknown_outcomes += 1,
                OutcomeClass::DuplicateEffect => report.duplicate_effects += 1,
            }
        }
    }
    for binding in offer.operation_bindings() {
        if !operations.contains(binding.operation_id()) {
            push(&mut report, OutcomeLedgerIssue::MissingOperation(binding.operation_id().to_string()));
        }
    }

    let calculated_digest = match calculate_outcome_ledger_digest(envelope) {
        Ok(digest) => digest,
        Err(_) => {
            push(&mut report, OutcomeLedgerIssue::InvalidDigest("ledger_root"));
            String::new()
        }
    };
    if !valid_sha256_hex(&envelope.ledger_digest_sha256) || envelope.ledger_digest_sha256 != calculated_digest {
        push(&mut report, OutcomeLedgerIssue::LedgerDigestMismatch);
    } else {
        report.outcome_ledger_digest_sha256 = Some(calculated_digest);
    }

    let incomplete_issues = report.issues.iter().any(|issue| {
        matches!(
            issue,
            OutcomeLedgerIssue::MissingAttestation(_)
                | OutcomeLedgerIssue::MissingOperation(_)
                | OutcomeLedgerIssue::UnknownDependencyClosureCannotSupportSuccess(_)
                | OutcomeLedgerIssue::SharedDependencyCannotSupportSuccess(_)
        )
    });
    let invalid_issues = report.issues.iter().any(|issue| {
        matches!(
            issue,
            OutcomeLedgerIssue::DuplicateEffect(_)
                | OutcomeLedgerIssue::DuplicateOperation(_)
                | OutcomeLedgerIssue::InvalidSequence { .. }
                | OutcomeLedgerIssue::OperationBindingMismatch { .. }
                | OutcomeLedgerIssue::ConflictingAttestation { .. }
                | OutcomeLedgerIssue::InvalidSignature(_)
                | OutcomeLedgerIssue::LedgerDigestMismatch
                | OutcomeLedgerIssue::UnknownSource(_)
                | OutcomeLedgerIssue::RevokedSource(_)
                | OutcomeLedgerIssue::UnauthorizedEvidenceKind(_)
                | OutcomeLedgerIssue::UnauthorizedEffectState(_)
                | OutcomeLedgerIssue::InvalidEvidenceKindForOutcome { .. }
                | OutcomeLedgerIssue::InvalidEffectStateForOutcome { .. }
                | OutcomeLedgerIssue::ContextMismatch(_)
                | OutcomeLedgerIssue::TrustPolicyMismatch
                | OutcomeLedgerIssue::MeasurementWindowMismatch
                | OutcomeLedgerIssue::InvalidDigest(_)
        )
    });
    report.status = if invalid_issues {
        OutcomeLedgerStatus::Invalid
    } else if incomplete_issues || !report.issues.is_empty() {
        OutcomeLedgerStatus::Incomplete
    } else {
        OutcomeLedgerStatus::Valid
    };
    if report.status != OutcomeLedgerStatus::Valid {
        return OutcomeLedgerVerification::Rejected(report);
    }

    OutcomeLedgerVerification::Verified(VerifiedOutcomeLedger {
        run_id: envelope.run_id.clone(),
        scenario_id: envelope.scenario_id.clone(),
        scenario_contract_digest_sha256: envelope.scenario_contract_digest_sha256.clone(),
        source_commit: envelope.source.commit.clone(),
        source_tree_sha: envelope.source.tree_sha.clone(),
        runtime_identity_digest_sha256: envelope.runtime_identity_digest_sha256.clone(),
        topology_profile_digest_sha256: envelope.topology_profile_digest_sha256.clone(),
        offer_ledger_digest_sha256: envelope.offer_ledger_digest_sha256.clone(),
        ledger_digest_sha256: envelope.ledger_digest_sha256.clone(),
        trust_policy_id: policy.policy_id().to_string(),
        trust_policy_digest_sha256: policy.digest_sha256().to_string(),
        entry_count: envelope.entries.len(),
        attestation_count: report.attestation_count,
        qualified_successes: report.qualified_successes,
        correct_denials: report.correct_denials,
        unexpected_denials: report.unexpected_denials,
        failures: report.failures,
        unknown_outcomes: report.unknown_outcomes,
        duplicate_effects: report.duplicate_effects,
        operation_outcomes,
    })
}

impl EvidenceKind {
    fn as_str(self) -> &'static str {
        match self {
            Self::ApplicationRecord => "application-record",
            Self::ProviderAcknowledgement => "provider-acknowledgement",
            Self::ProviderStatusQuery => "provider-status-query",
            Self::IndependentObservation => "independent-observation",
            Self::PolicyDecision => "policy-decision",
            Self::FailureTrace => "failure-trace",
            Self::Reconciliation => "reconciliation",
            Self::CrossPeerRead => "cross-peer-read",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::offer_ledger::{
        calculate_offer_ledger_digest, embedded_scenario_manifest_digest, verify_offer_ledger,
        OfferLedgerEntry, OfferLedgerEnvelope, OFFER_LEDGER_SCHEMA_VERSION,
    };
    use ed25519_dalek::{SigningKey, Signer};

    fn digest(value: &str) -> String { sha256_hex(value.as_bytes()) }

    fn offer_envelope(scenario: &str) -> OfferLedgerEnvelope {
        let cross_host = matches!(scenario, "MCRB-02" | "MCRB-03" | "MCRB-05");
        let run_id = "run-outcome-test".to_string();
        let generator = digest("generator");
        let manifest = digest("workload-manifest");
        let parameters = digest("parameters");
        let mut envelope = OfferLedgerEnvelope {
            schema_version: OFFER_LEDGER_SCHEMA_VERSION.to_string(),
            run_id: run_id.clone(),
            scenario_id: scenario.to_string(),
            measurement_window_id: "window-outcome".to_string(),
            scenario_contract_digest_sha256: embedded_scenario_manifest_digest(),
            requires_cross_host_visibility: cross_host,
            workload_manifest_digest_sha256: manifest.clone(),
            generator_digest_sha256: generator.clone(),
            parameter_set_digest_sha256: parameters.clone(),
            clock_source: "CLOCK_MONOTONIC_RAW".to_string(),
            monotonic_clock_id: "clock-peer-a".to_string(),
            monotonic_clock_epoch: "boot-epoch-1".to_string(),
            measurement_start_ticks: 100,
            measurement_end_ticks: 1_000_000_100,
            ticks_per_second: 1_000_000_000,
            entry_count: 2,
            entries: (0..2).map(|index| OfferLedgerEntry {
                sequence: index + 1,
                run_id: run_id.clone(),
                scenario_id: scenario.to_string(),
                operation_id: format!("op-{}", index + 1),
                request_digest_sha256: digest(&format!("request-{index}")),
                idempotency_key: Some(format!("idem-{index}")),
                scheduled_monotonic_tick: 200 + index as u64,
                monotonic_clock_id: "clock-peer-a".to_string(),
                monotonic_clock_epoch: "boot-epoch-1".to_string(),
                operation_class: "read-workflow".to_string(),
                requires_idempotency_key: false,
                generator_digest_sha256: generator.clone(),
                workload_manifest_digest_sha256: manifest.clone(),
                parameter_set_digest_sha256: parameters.clone(),
            }).collect(),
            ledger_digest_sha256: String::new(),
        };
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        envelope
    }

    fn verified_offer(scenario: &str) -> VerifiedOfferLedger {
        verify_offer_ledger(&offer_envelope(scenario)).expect("test offer ledger must verify")
    }

    fn context(offer: &VerifiedOfferLedger) -> OutcomeVerificationContext {
        OutcomeVerificationContext {
            run_id: offer.run_id().to_string(),
            scenario_id: offer.scenario_id().to_string(),
            scenario_contract_digest_sha256: offer.scenario_contract_digest_sha256().to_string(),
            source_commit: "a".repeat(40),
            source_tree_sha: "b".repeat(40),
            runtime_identity_digest_sha256: digest("runtime"),
            topology_profile_digest_sha256: digest("topology"),
            now_unix_ms: 1_800_000_000_000,
        }
    }

    fn signing_key() -> SigningKey { SigningKey::from_bytes(&[31_u8; 32]) }

    fn trust_policy(key: &SigningKey, role: OutcomeSourceRole, deps: Vec<String>, complete: bool) -> VerifiedOutcomeTrustPolicy {
        let public_key_hex = key.verifying_key().to_bytes().iter().map(|byte| format!("{byte:02x}")).collect::<String>();
        let document = OutcomeTrustPolicyDocument {
            schema_version: OUTCOME_TRUST_POLICY_SCHEMA_VERSION.to_string(),
            policy_id: "outcome-test-policy-v0.1".to_string(),
            policy_owner: "independent-test-policy-owner".to_string(),
            valid_from_unix_ms: 1_700_000_000_000,
            expires_at_unix_ms: 1_900_000_000_000,
            effect_dependency_ids: vec!["effect-db".to_string(), "identity-root".to_string()],
            effect_dependency_closure_complete: true,
            sources: vec![TrustedOutcomeSource {
                source_id: "observer-a".to_string(),
                key_id: "observer-key".to_string(),
                verifier_identity: "outcome-test-observer".to_string(),
                public_key_hex,
                source_role: role,
                dependency_ids: deps,
                dependency_closure_complete: complete,
                allowed_evidence_kinds: vec![
                    EvidenceKind::ApplicationRecord,
                    EvidenceKind::ProviderAcknowledgement,
                    EvidenceKind::ProviderStatusQuery,
                    EvidenceKind::IndependentObservation,
                    EvidenceKind::PolicyDecision,
                    EvidenceKind::FailureTrace,
                    EvidenceKind::Reconciliation,
                    EvidenceKind::CrossPeerRead,
                ],
                allowed_effect_states: vec![
                    EffectState::Applied,
                    EffectState::ConfirmedNotApplied,
                    EffectState::Partial,
                    EffectState::Unknown,
                    EffectState::NotAttempted,
                ],
                allowed_builder_ids: vec!["trusted-benchmark-builder".to_string()],
                valid_from_unix_ms: 1_700_000_000_000,
                expires_at_unix_ms: 1_900_000_000_000,
                revoked: false,
            }],
        };
        let pin = sha256_hex(&serde_json::to_vec(&document).unwrap());
        VerifiedOutcomeTrustPolicy::load_pinned(document, &pin).unwrap()
    }

    fn make_attestation(
        offer: &VerifiedOfferLedger,
        context: &OutcomeVerificationContext,
        policy: &VerifiedOutcomeTrustPolicy,
        key: &SigningKey,
        binding_index: usize,
        outcome: OutcomeClass,
        effect_state: EffectState,
        evidence_kind: EvidenceKind,
    ) -> OutcomeDsseEnvelope {
        use ed25519_dalek::Signer;
        let binding = &offer.operation_bindings()[binding_index];
        let evidence_digest = digest(&format!("evidence-{}", binding.operation_id()));
        let predicate = OutcomeAttestationPredicate {
            schema_version: OUTCOME_ATTESTATION_SCHEMA_VERSION.to_string(),
            run_id: context.run_id.clone(),
            scenario_id: context.scenario_id.clone(),
            scenario_contract_digest_sha256: context.scenario_contract_digest_sha256.clone(),
            source_commit: context.source_commit.clone(),
            source_tree_sha: context.source_tree_sha.clone(),
            runtime_identity_digest_sha256: context.runtime_identity_digest_sha256.clone(),
            topology_profile_digest_sha256: context.topology_profile_digest_sha256.clone(),
            offer_ledger_digest_sha256: offer.ledger_digest_sha256().to_string(),
            operation_id: binding.operation_id().to_string(),
            operation_sequence: binding.sequence(),
            request_digest_sha256: binding.request_digest_sha256().to_string(),
            idempotency_key_digest_sha256: binding.idempotency_key_digest_sha256().map(str::to_string),
            outcome,
            effect_state,
            evidence_kind,
            evidence_artifact_digest_sha256: evidence_digest.clone(),
            source_id: "observer-a".to_string(),
            source_role: policy.document.sources[0].source_role,
            builder_identity: "trusted-benchmark-builder".to_string(),
            verifier_identity: "outcome-test-observer".to_string(),
            verifier_version: "0.1.0".to_string(),
            verifier_binary_digest_sha256: digest("observer-binary"),
            trust_policy_id: policy.policy_id().to_string(),
            trust_policy_digest_sha256: policy.digest_sha256().to_string(),
            issued_at_unix_ms: context.now_unix_ms - 1,
        };
        let statement = OutcomeInTotoStatement {
            statement_type: IN_TOTO_STATEMENT_V1.to_string(),
            subject: vec![OutcomeInTotoSubject {
                name: format!("mcrb-evidence/{}/{}", binding.operation_id(), evidence_kind.as_str()),
                digest: BTreeMap::from([("sha256".to_string(), evidence_digest)]),
            }],
            predicate_type: OUTCOME_PREDICATE_TYPE.to_string(),
            predicate,
        };
        let payload = serde_json::to_vec(&statement).unwrap();
        let signature = key.sign(&dsse_pae(OUTCOME_PAYLOAD_TYPE, &payload));
        OutcomeDsseEnvelope {
            payload_type: OUTCOME_PAYLOAD_TYPE.to_string(),
            payload: STANDARD.encode(payload),
            signatures: vec![OutcomeDsseSignature {
                keyid: "observer-key".to_string(),
                sig: STANDARD.encode(signature.to_bytes()),
            }],
        }
    }

    fn valid_ledger(offer: &VerifiedOfferLedger, policy: &VerifiedOutcomeTrustPolicy, key: &SigningKey, context: &OutcomeVerificationContext) -> OutcomeLedgerEnvelope {
        let entries = offer.operation_bindings().iter().enumerate().map(|(index, binding)| {
            let envelope = make_attestation(
                offer, context, policy, key, index,
                OutcomeClass::QualifiedSuccess,
                EffectState::Applied,
                if offer.requires_cross_host_visibility() {
                    EvidenceKind::CrossPeerRead
                } else {
                    EvidenceKind::IndependentObservation
                },
            );
            OutcomeLedgerEntry {
                sequence: binding.sequence(),
                operation_id: binding.operation_id().to_string(),
                request_digest_sha256: binding.request_digest_sha256().to_string(),
                idempotency_key_digest_sha256: binding.idempotency_key_digest_sha256().map(str::to_string),
                effect_state: EffectState::Applied,
                attestations: vec![envelope],
                prior_outcome_ledger_digest_sha256: None,
                reconciliation_sequence: None,
            }
        }).collect::<Vec<_>>();
        let mut ledger = OutcomeLedgerEnvelope {
            schema_version: OUTCOME_LEDGER_SCHEMA_VERSION.to_string(),
            run_id: offer.run_id().to_string(),
            scenario_id: offer.scenario_id().to_string(),
            scenario_contract_digest_sha256: offer.scenario_contract_digest_sha256().to_string(),
            source: OutcomeSourceIdentity {
                commit: context.source_commit.clone(),
                tree_sha: context.source_tree_sha.clone(),
            },
            runtime_identity_digest_sha256: context.runtime_identity_digest_sha256.clone(),
            topology_profile_digest_sha256: context.topology_profile_digest_sha256.clone(),
            offer_ledger_digest_sha256: offer.ledger_digest_sha256().to_string(),
            measurement_window: OutcomeMeasurementWindow {
                window_id: offer.measurement_window_id().to_string(),
                clock_id: offer.monotonic_clock_id().to_string(),
                clock_epoch: offer.monotonic_clock_epoch().to_string(),
                start_tick: offer.measurement_start_ticks(),
                end_tick: offer.measurement_end_ticks(),
                ticks_per_second: offer.ticks_per_second(),
            },
            trust_policy_id: policy.policy_id().to_string(),
            trust_policy_digest_sha256: policy.digest_sha256().to_string(),
            entry_count: entries.len() as u64,
            entries,
            ledger_digest_sha256: String::new(),
        };
        ledger.ledger_digest_sha256 = calculate_outcome_ledger_digest(&ledger).unwrap();
        ledger
    }

    #[test]
    fn independently_observed_local_successes_verify_against_exact_offers() {
        let offer = verified_offer("MCRB-01");
        let context = context(&offer);
        let key = signing_key();
        let policy = trust_policy(
            &key,
            OutcomeSourceRole::IndependentObserver,
            vec!["observer-host".to_string()],
            true,
        );
        let ledger = valid_ledger(&offer, &policy, &key, &context);
        let result = verify_outcome_ledger(&ledger, &offer, &policy, &context);
        let verified = result.verified().expect("verified evidence ledger");
        assert_eq!(verified.entry_count(), 2);
        assert_eq!(verified.qualified_successes(), 2);
        assert_eq!(verified.unknown_outcomes(), 0);
    }

    #[test]
    fn remote_visibility_requires_cross_peer_read_not_provider_acknowledgement() {
        let offer = verified_offer("MCRB-02");
        let context = context(&offer);
        let key = signing_key();
        let policy = trust_policy(&key, OutcomeSourceRole::IndependentObserver, vec!["observer-host".into()], true);
        let mut ledger = valid_ledger(&offer, &policy, &key, &context);
        let replacement = make_attestation(
            &offer, &context, &policy, &key, 0,
            OutcomeClass::QualifiedSuccess, EffectState::Applied, EvidenceKind::ProviderAcknowledgement,
        );
        ledger.entries[0].attestations = vec![replacement];
        ledger.ledger_digest_sha256 = calculate_outcome_ledger_digest(&ledger).unwrap();
        let result = verify_outcome_ledger(&ledger, &offer, &policy, &context);
        assert!(result.verified().is_none());
        assert!(result.report().unwrap().issues.iter().any(|issue| matches!(issue, OutcomeLedgerIssue::InvalidEvidenceKindForOutcome { .. })));
    }

    #[test]
    fn shared_dependency_observer_cannot_qualify_success() {
        let offer = verified_offer("MCRB-01");
        let context = context(&offer);
        let key = signing_key();
        let policy = trust_policy(&key, OutcomeSourceRole::IndependentObserver, vec!["effect-db".into()], true);
        let ledger = valid_ledger(&offer, &policy, &key, &context);
        let result = verify_outcome_ledger(&ledger, &offer, &policy, &context);
        assert!(result.verified().is_none());
        assert!(result.report().unwrap().issues.iter().any(|issue| matches!(issue, OutcomeLedgerIssue::SharedDependencyCannotSupportSuccess(_))));
    }

    #[test]
    fn unknown_dependency_closure_cannot_be_promoted_to_independent_success() {
        let offer = verified_offer("MCRB-01");
        let context = context(&offer);
        let key = signing_key();
        let policy = trust_policy(&key, OutcomeSourceRole::IndependentObserver, vec!["observer-host".into()], false);
        let ledger = valid_ledger(&offer, &policy, &key, &context);
        let result = verify_outcome_ledger(&ledger, &offer, &policy, &context);
        assert!(result.verified().is_none());
        assert!(result.report().unwrap().issues.iter().any(|issue| matches!(issue, OutcomeLedgerIssue::UnknownDependencyClosureCannotSupportSuccess(_))));
    }

    #[test]
    fn tampered_request_binding_and_signature_are_rejected() {
        let offer = verified_offer("MCRB-01");
        let context = context(&offer);
        let key = signing_key();
        let policy = trust_policy(&key, OutcomeSourceRole::IndependentObserver, vec!["observer-host".into()], true);
        let mut ledger = valid_ledger(&offer, &policy, &key, &context);
        ledger.entries[0].request_digest_sha256 = digest("other-request");
        ledger.ledger_digest_sha256 = calculate_outcome_ledger_digest(&ledger).unwrap();
        assert!(verify_outcome_ledger(&ledger, &offer, &policy, &context).verified().is_none());

        let mut ledger = valid_ledger(&offer, &policy, &key, &context);
        ledger.entries[0].attestations[0].signatures[0].sig = STANDARD.encode([0_u8; 64]);
        ledger.ledger_digest_sha256 = calculate_outcome_ledger_digest(&ledger).unwrap();
        assert!(verify_outcome_ledger(&ledger, &offer, &policy, &context).verified().is_none());
    }

    #[test]
    fn explicit_unknown_is_preserved_as_verified_but_not_a_qualified_success() {
        let offer = verified_offer("MCRB-01");
        let context = context(&offer);
        let key = signing_key();
        let policy = trust_policy(&key, OutcomeSourceRole::IndependentObserver, vec!["observer-host".into()], true);
        let mut ledger = valid_ledger(&offer, &policy, &key, &context);
        let unknown = make_attestation(
            &offer, &context, &policy, &key, 0,
            OutcomeClass::Unknown, EffectState::Unknown, EvidenceKind::ProviderStatusQuery,
        );
        ledger.entries[0].effect_state = EffectState::Unknown;
        ledger.entries[0].attestations = vec![unknown];
        ledger.ledger_digest_sha256 = calculate_outcome_ledger_digest(&ledger).unwrap();
        let verified = verify_outcome_ledger(&ledger, &offer, &policy, &context).verified().unwrap();
        assert_eq!(verified.qualified_successes(), 1);
        assert_eq!(verified.unknown_outcomes(), 1);
    }

    #[test]
    fn duplicate_effect_is_a_hard_invalidity() {
        let offer = verified_offer("MCRB-01");
        let context = context(&offer);
        let key = signing_key();
        let policy = trust_policy(&key, OutcomeSourceRole::IndependentObserver, vec!["observer-host".into()], true);
        let mut ledger = valid_ledger(&offer, &policy, &key, &context);
        let duplicate = make_attestation(
            &offer, &context, &policy, &key, 0,
            OutcomeClass::DuplicateEffect, EffectState::Applied, EvidenceKind::IndependentObservation,
        );
        ledger.entries[0].attestations = vec![duplicate];
        ledger.ledger_digest_sha256 = calculate_outcome_ledger_digest(&ledger).unwrap();
        let result = verify_outcome_ledger(&ledger, &offer, &policy, &context);
        assert!(result.verified().is_none());
        assert!(result.report().unwrap().issues.iter().any(|issue| matches!(issue, OutcomeLedgerIssue::DuplicateEffect(_))));
    }
}

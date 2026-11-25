// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Craft Work Evidence V1 reference contract.
//!
//! This crate verifies that an attestation is correctly signed, issued by a
//! trusted and unrevoked authority, scoped to the requested work, conflict-
//! disclosed, time-valid, and not replayed according to caller-supplied state.
//! It does NOT prove that the underlying work or observation is factually true.
//! A host must persist returned replay keys atomically before triggering any
//! downstream effect.

use ed25519_dalek::{Signature, VerifyingKey};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const WORK_EVIDENCE_SCHEMA_VERSION: u16 = 1;
const CLAIM_DOMAIN: &[u8] = b"mycelix:craft:work-claim:v1\0";
const AUTHORITY_DOMAIN: &[u8] = b"mycelix:craft:assessor-authority:v1\0";
const ATTESTATION_DOMAIN: &[u8] = b"mycelix:craft:work-attestation:v1\0";
const MAX_EVIDENCE_ARTIFACTS: usize = 256;
const MAX_ATTESTATIONS: usize = 32;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceProvenanceV1 {
    pub actor_did: String,
    pub method: EvidenceCaptureMethod,
    pub source_ref: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceCaptureMethod {
    DirectObservation,
    InstrumentMeasurement,
    Inspection,
    RepositoryRecord,
    EmployerRecord,
    LearnerProvided,
    Other,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceArtifactRefV1 {
    pub artifact_id: String,
    pub uri: String,
    pub media_type: String,
    /// SHA-256 of the exact artifact bytes, lowercase hexadecimal.
    pub sha256_hex: String,
    pub captured_at_unix: u64,
    pub provenance: EvidenceProvenanceV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorkEvidenceClaimV1 {
    pub schema_version: u16,
    pub claim_id: String,
    pub revision: u32,
    pub subject_did: String,
    pub organization: String,
    pub professional_domain: String,
    pub participant_role: String,
    pub contribution_scope: String,
    pub outcome_summary: String,
    pub agreement_ref: Option<String>,
    pub work_order_ref: Option<String>,
    pub observed_from_unix: u64,
    pub observed_until_unix: u64,
    pub created_at_unix: u64,
    pub evidence: Vec<EvidenceArtifactRefV1>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssessorAuthorityV1 {
    pub schema_version: u16,
    pub authority_id: String,
    pub revision: u32,
    pub issuer_did: String,
    pub issuer_key_id: String,
    pub assessor_did: String,
    pub professional_domain: String,
    pub scopes: Vec<String>,
    pub jurisdiction: String,
    pub valid_from_unix: u64,
    pub valid_until_unix: u64,
    /// Ed25519 signature over the versioned canonical authority payload.
    pub signature_hex: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConflictKind {
    SelfInterest,
    Family,
    Financial,
    Supervisory,
    BusinessPartner,
    Other,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AttestationOutcome {
    Corroborated,
    UnableToCorroborate,
    Disputed,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorkAttestationV1 {
    pub schema_version: u16,
    pub attestation_id: String,
    pub claim_digest_hex: String,
    pub verifier_did: String,
    pub assessor_key_id: String,
    pub authority_id: String,
    pub professional_domain: String,
    pub scope: String,
    pub jurisdiction: String,
    pub outcome: AttestationOutcome,
    pub observed_at_unix: u64,
    pub nonce: String,
    /// This is a signed declaration, not proof that undisclosed conflicts do
    /// not exist. Independent audit and sanctions remain policy concerns.
    pub conflicts_declared_complete: bool,
    pub conflicts: Vec<ConflictKind>,
    pub limitations: Vec<String>,
    /// Ed25519 signature over the versioned canonical attestation payload.
    pub signature_hex: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrustedKeyV1 {
    pub controller_did: String,
    pub public_key_hex: String,
    pub valid_from_unix: u64,
    pub valid_until_unix: u64,
    pub revoked: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrustStoreV1 {
    pub trusted_authority_ids: BTreeSet<String>,
    pub revoked_authority_ids: BTreeSet<String>,
    pub keys: BTreeMap<String, TrustedKeyV1>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerificationPolicyV1 {
    pub professional_domain: String,
    pub required_scope: String,
    pub jurisdiction: String,
    pub minimum_distinct_assessors: u8,
    pub max_attestation_age_seconds: Option<u64>,
    pub prohibited_conflicts: BTreeSet<ConflictKind>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedAttestationV1 {
    pub attestation_id: String,
    pub verifier_did: String,
    pub authority_id: String,
    /// Both values must be atomically reserved by the host: one-time assessor
    /// nonce and globally unique attestation identifier.
    pub replay_keys: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedWorkEvidenceV1 {
    pub claim_id: String,
    pub claim_revision: u32,
    pub claim_digest_hex: String,
    pub attestations: Vec<VerifiedAttestationV1>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VerificationError {
    UnsupportedSchema,
    InvalidClaim(&'static str),
    InvalidAuthority(&'static str),
    InvalidAttestation(&'static str),
    InvalidPolicy(&'static str),
    ClaimDigestMismatch,
    UnknownAuthority,
    RevokedAuthority,
    UnknownKey,
    RevokedKey,
    KeyControllerMismatch,
    KeyOutsideValidity,
    InvalidSignature,
    AuthorityOutsideValidity,
    AuthorityOutOfScope,
    AssessorMismatch,
    SelfAttestation,
    FutureObservation,
    PrematureAttestation,
    AttestationTooOld,
    ConflictDisclosureIncomplete,
    ProhibitedConflict,
    NonCorroboratingOutcome,
    Replay,
    DuplicateAssessor,
    InsufficientDistinctAssessors,
    TooManyAttestations,
    ArtifactDigestMismatch,
}

type VerifyResult<T> = Result<T, VerificationError>;

#[derive(Default)]
struct CanonicalWriter {
    bytes: Vec<u8>,
}

impl CanonicalWriter {
    fn new(domain: &[u8]) -> Self {
        Self { bytes: domain.to_vec() }
    }

    fn u8(&mut self, value: u8) {
        self.bytes.push(value);
    }

    fn u16(&mut self, value: u16) {
        self.bytes.extend_from_slice(&value.to_be_bytes());
    }

    fn u32(&mut self, value: u32) {
        self.bytes.extend_from_slice(&value.to_be_bytes());
    }

    fn u64(&mut self, value: u64) {
        self.bytes.extend_from_slice(&value.to_be_bytes());
    }

    fn boolean(&mut self, value: bool) {
        self.u8(u8::from(value));
    }

    fn string(&mut self, value: &str) {
        self.u32(value.len() as u32);
        self.bytes.extend_from_slice(value.as_bytes());
    }

    fn optional_string(&mut self, value: &Option<String>) {
        match value {
            Some(value) => {
                self.u8(1);
                self.string(value);
            }
            None => self.u8(0),
        }
    }

    fn finish(self) -> Vec<u8> {
        self.bytes
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    hex_encode(&Sha256::digest(bytes))
}

fn hex_encode(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        let value = *byte;
        output.push(HEX[(value >> 4) as usize] as char);
        output.push(HEX[(value & 0x0f) as usize] as char);
    }
    output
}

fn hex_decode(value: &str) -> Option<Vec<u8>> {
    if value.len() % 2 != 0 {
        return None;
    }
    let mut output = Vec::with_capacity(value.len() / 2);
    for pair in value.as_bytes().chunks_exact(2) {
        let high = (pair[0] as char).to_digit(16)?;
        let low = (pair[1] as char).to_digit(16)?;
        output.push(((high << 4) | low) as u8);
    }
    Some(output)
}

fn valid_hex_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

fn push_sorted_strings(writer: &mut CanonicalWriter, values: &[String]) {
    let mut sorted = values.to_vec();
    sorted.sort();
    writer.u32(sorted.len() as u32);
    for value in sorted {
        writer.string(&value);
    }
}

fn push_sorted_conflicts(writer: &mut CanonicalWriter, values: &[ConflictKind]) {
    let mut sorted = values.to_vec();
    sorted.sort();
    writer.u32(sorted.len() as u32);
    for value in sorted {
        writer.u8(match value {
            ConflictKind::SelfInterest => 0,
            ConflictKind::Family => 1,
            ConflictKind::Financial => 2,
            ConflictKind::Supervisory => 3,
            ConflictKind::BusinessPartner => 4,
            ConflictKind::Other => 5,
        });
    }
}

fn claim_payload(claim: &WorkEvidenceClaimV1) -> Vec<u8> {
    let mut w = CanonicalWriter::new(CLAIM_DOMAIN);
    w.u16(claim.schema_version);
    w.string(&claim.claim_id);
    w.u32(claim.revision);
    w.string(&claim.subject_did);
    w.string(&claim.organization);
    w.string(&claim.professional_domain);
    w.string(&claim.participant_role);
    w.string(&claim.contribution_scope);
    w.string(&claim.outcome_summary);
    w.optional_string(&claim.agreement_ref);
    w.optional_string(&claim.work_order_ref);
    w.u64(claim.observed_from_unix);
    w.u64(claim.observed_until_unix);
    w.u64(claim.created_at_unix);

    let mut evidence = claim.evidence.clone();
    evidence.sort_by(|a, b| a.artifact_id.cmp(&b.artifact_id));
    w.u32(evidence.len() as u32);
    for item in evidence {
        w.string(&item.artifact_id);
        w.string(&item.uri);
        w.string(&item.media_type);
        w.string(&item.sha256_hex);
        w.u64(item.captured_at_unix);
        w.string(&item.provenance.actor_did);
        w.u8(match item.provenance.method {
            EvidenceCaptureMethod::DirectObservation => 0,
            EvidenceCaptureMethod::InstrumentMeasurement => 1,
            EvidenceCaptureMethod::Inspection => 2,
            EvidenceCaptureMethod::RepositoryRecord => 3,
            EvidenceCaptureMethod::EmployerRecord => 4,
            EvidenceCaptureMethod::LearnerProvided => 5,
            EvidenceCaptureMethod::Other => 6,
        });
        w.optional_string(&item.provenance.source_ref);
    }
    w.finish()
}

fn authority_payload(authority: &AssessorAuthorityV1) -> Vec<u8> {
    let mut w = CanonicalWriter::new(AUTHORITY_DOMAIN);
    w.u16(authority.schema_version);
    w.string(&authority.authority_id);
    w.u32(authority.revision);
    w.string(&authority.issuer_did);
    w.string(&authority.issuer_key_id);
    w.string(&authority.assessor_did);
    w.string(&authority.professional_domain);
    push_sorted_strings(&mut w, &authority.scopes);
    w.string(&authority.jurisdiction);
    w.u64(authority.valid_from_unix);
    w.u64(authority.valid_until_unix);
    w.finish()
}

fn attestation_payload(attestation: &WorkAttestationV1) -> Vec<u8> {
    let mut w = CanonicalWriter::new(ATTESTATION_DOMAIN);
    w.u16(attestation.schema_version);
    w.string(&attestation.attestation_id);
    w.string(&attestation.claim_digest_hex);
    w.string(&attestation.verifier_did);
    w.string(&attestation.assessor_key_id);
    w.string(&attestation.authority_id);
    w.string(&attestation.professional_domain);
    w.string(&attestation.scope);
    w.string(&attestation.jurisdiction);
    w.u8(match attestation.outcome {
        AttestationOutcome::Corroborated => 0,
        AttestationOutcome::UnableToCorroborate => 1,
        AttestationOutcome::Disputed => 2,
    });
    w.u64(attestation.observed_at_unix);
    w.string(&attestation.nonce);
    w.boolean(attestation.conflicts_declared_complete);
    push_sorted_conflicts(&mut w, &attestation.conflicts);
    push_sorted_strings(&mut w, &attestation.limitations);
    w.finish()
}

pub fn claim_digest_hex(claim: &WorkEvidenceClaimV1) -> VerifyResult<String> {
    validate_claim(claim)?;
    Ok(sha256_hex(&claim_payload(claim)))
}

pub fn verify_artifact_bytes(
    artifact: &EvidenceArtifactRefV1,
    bytes: &[u8],
) -> VerifyResult<()> {
    if !valid_hex_digest(&artifact.sha256_hex) || sha256_hex(bytes) != artifact.sha256_hex {
        return Err(VerificationError::ArtifactDigestMismatch);
    }
    Ok(())
}

pub fn replay_keys(attestation: &WorkAttestationV1) -> [String; 2] {
    [
        format!(
            "nonce:{}:{}:{}",
            attestation.verifier_did, attestation.authority_id, attestation.nonce
        ),
        format!("attestation:{}:{}", attestation.authority_id, attestation.attestation_id),
    ]
}

fn validate_claim(claim: &WorkEvidenceClaimV1) -> VerifyResult<()> {
    if claim.schema_version != WORK_EVIDENCE_SCHEMA_VERSION {
        return Err(VerificationError::UnsupportedSchema);
    }
    for value in [
        claim.claim_id.as_str(),
        claim.subject_did.as_str(),
        claim.organization.as_str(),
        claim.professional_domain.as_str(),
        claim.participant_role.as_str(),
        claim.contribution_scope.as_str(),
        claim.outcome_summary.as_str(),
    ] {
        if value.trim().is_empty() {
            return Err(VerificationError::InvalidClaim("required field is empty"));
        }
        if value.len() > 4096 {
            return Err(VerificationError::InvalidClaim("field exceeds 4096 bytes"));
        }
    }
    if claim.agreement_ref.as_ref().is_some_and(|value| value.len() > 4096)
        || claim.work_order_ref.as_ref().is_some_and(|value| value.len() > 4096)
    {
        return Err(VerificationError::InvalidClaim("agreement or work-order reference exceeds 4096 bytes"));
    }
    if claim.revision == 0 {
        return Err(VerificationError::InvalidClaim("revision must be positive"));
    }
    if claim.observed_until_unix < claim.observed_from_unix {
        return Err(VerificationError::InvalidClaim("observation interval is inverted"));
    }
    if claim.evidence.is_empty() {
        return Err(VerificationError::InvalidClaim("at least one evidence artifact is required"));
    }
    if claim.evidence.len() > MAX_EVIDENCE_ARTIFACTS {
        return Err(VerificationError::InvalidClaim("too many evidence artifacts"));
    }

    let mut ids = BTreeSet::new();
    for item in &claim.evidence {
        if item.artifact_id.trim().is_empty()
            || item.uri.trim().is_empty()
            || item.media_type.trim().is_empty()
            || item.provenance.actor_did.trim().is_empty()
        {
            return Err(VerificationError::InvalidClaim("artifact metadata is incomplete"));
        }
        if item.artifact_id.len() > 512
            || item.uri.len() > 4096
            || item.media_type.len() > 256
            || item.provenance.actor_did.len() > 512
            || item.provenance.source_ref.as_ref().is_some_and(|value| value.len() > 4096)
        {
            return Err(VerificationError::InvalidClaim("artifact field exceeds its size bound"));
        }
        if !ids.insert(item.artifact_id.as_str()) {
            return Err(VerificationError::InvalidClaim("duplicate artifact id"));
        }
        if !valid_hex_digest(&item.sha256_hex) {
            return Err(VerificationError::InvalidClaim("artifact digest is not lowercase SHA-256 hex"));
        }
    }
    Ok(())
}

fn validate_claim_timestamps(claim: &WorkEvidenceClaimV1, now_unix: u64) -> VerifyResult<()> {
    if claim.observed_until_unix > claim.created_at_unix
        || claim.created_at_unix > now_unix
    {
        return Err(VerificationError::InvalidClaim(
            "observation must end before claim creation, and claim creation cannot be in the future",
        ));
    }
    if claim
        .evidence
        .iter()
        .any(|artifact| artifact.captured_at_unix > claim.created_at_unix
            || artifact.captured_at_unix > now_unix)
    {
        return Err(VerificationError::InvalidClaim(
            "evidence capture cannot occur after claim creation or in the future",
        ));
    }
    Ok(())
}

fn verify_key_signature(
    key: &TrustedKeyV1,
    payload: &[u8],
    signature_hex: &str,
    at_unix: u64,
    now_unix: u64,
) -> VerifyResult<()> {
    if key.public_key_hex.len() != 64 {
        return Err(VerificationError::UnknownKey);
    }
    if signature_hex.len() != 128 {
        return Err(VerificationError::InvalidSignature);
    }
    if key.revoked {
        return Err(VerificationError::RevokedKey);
    }
    if key.valid_from_unix > at_unix || key.valid_until_unix < at_unix
        || key.valid_from_unix > now_unix || key.valid_until_unix < now_unix
    {
        return Err(VerificationError::KeyOutsideValidity);
    }
    let key_bytes = hex_decode(&key.public_key_hex).ok_or(VerificationError::UnknownKey)?;
    let key_array: [u8; 32] = key_bytes
        .try_into()
        .map_err(|_| VerificationError::UnknownKey)?;
    let verifying_key =
        VerifyingKey::from_bytes(&key_array).map_err(|_| VerificationError::UnknownKey)?;
    let signature_bytes =
        hex_decode(signature_hex).ok_or(VerificationError::InvalidSignature)?;
    let signature =
        Signature::from_slice(&signature_bytes).map_err(|_| VerificationError::InvalidSignature)?;
    verifying_key
        .verify_strict(payload, &signature)
        .map_err(|_| VerificationError::InvalidSignature)
}

fn validate_authority(authority: &AssessorAuthorityV1) -> VerifyResult<()> {
    if authority.schema_version != WORK_EVIDENCE_SCHEMA_VERSION {
        return Err(VerificationError::UnsupportedSchema);
    }
    if authority.revision == 0
        || authority.authority_id.trim().is_empty()
        || authority.issuer_did.trim().is_empty()
        || authority.issuer_key_id.trim().is_empty()
        || authority.assessor_did.trim().is_empty()
        || authority.professional_domain.trim().is_empty()
        || authority.jurisdiction.trim().is_empty()
        || authority.valid_until_unix < authority.valid_from_unix
        || authority.scopes.is_empty()
        || authority.scopes.len() > 32
    {
        return Err(VerificationError::InvalidAuthority("authority fields or validity interval are invalid"));
    }
    if [
        authority.authority_id.as_str(),
        authority.issuer_did.as_str(),
        authority.issuer_key_id.as_str(),
        authority.assessor_did.as_str(),
        authority.professional_domain.as_str(),
        authority.jurisdiction.as_str(),
    ]
    .iter()
    .any(|value| value.len() > 512)
    {
        return Err(VerificationError::InvalidAuthority("authority field exceeds 512 bytes"));
    }
    let mut scopes = BTreeSet::new();
    for scope in &authority.scopes {
        if scope.trim().is_empty() || scope.len() > 512 || !scopes.insert(scope.as_str()) {
            return Err(VerificationError::InvalidAuthority("authority scopes are empty, oversized, or duplicated"));
        }
    }
    Ok(())
}

fn validate_attestation(attestation: &WorkAttestationV1) -> VerifyResult<()> {
    if attestation.schema_version != WORK_EVIDENCE_SCHEMA_VERSION {
        return Err(VerificationError::UnsupportedSchema);
    }
    for value in [
        attestation.attestation_id.as_str(),
        attestation.verifier_did.as_str(),
        attestation.assessor_key_id.as_str(),
        attestation.authority_id.as_str(),
        attestation.professional_domain.as_str(),
        attestation.scope.as_str(),
        attestation.jurisdiction.as_str(),
        attestation.nonce.as_str(),
    ] {
        if value.trim().is_empty() || value.len() > 512 {
            return Err(VerificationError::InvalidAttestation("required field is empty or oversized"));
        }
    }
    if attestation.nonce.len() > 256 || !valid_hex_digest(&attestation.claim_digest_hex) {
        return Err(VerificationError::InvalidAttestation("nonce or claim digest is invalid"));
    }
    if attestation.conflicts.len() > 16 || attestation.limitations.len() > 32 {
        return Err(VerificationError::InvalidAttestation("too many conflicts or limitations"));
    }
    let mut conflicts = BTreeSet::new();
    if attestation.conflicts.iter().any(|value| !conflicts.insert(*value)) {
        return Err(VerificationError::InvalidAttestation("duplicate conflict declaration"));
    }
    if attestation.limitations.iter().any(|value| value.len() > 2048) {
        return Err(VerificationError::InvalidAttestation("limitation exceeds 2048 bytes"));
    }
    Ok(())
}

fn validate_policy(policy: &VerificationPolicyV1) -> VerifyResult<()> {
    if policy.professional_domain.trim().is_empty()
        || policy.required_scope.trim().is_empty()
        || policy.jurisdiction.trim().is_empty()
        || policy.minimum_distinct_assessors == 0
        || usize::from(policy.minimum_distinct_assessors) > MAX_ATTESTATIONS
    {
        return Err(VerificationError::InvalidPolicy("verification policy is incomplete or outside supported bounds"));
    }
    Ok(())
}

fn verify_one_attestation(
    claim_digest: &str,
    claim: &WorkEvidenceClaimV1,
    authority: &AssessorAuthorityV1,
    attestation: &WorkAttestationV1,
    policy: &VerificationPolicyV1,
    trust: &TrustStoreV1,
    now_unix: u64,
    consumed_replay_keys: &BTreeSet<String>,
) -> VerifyResult<VerifiedAttestationV1> {
    validate_authority(authority)?;
    validate_attestation(attestation)?;
    if attestation.claim_digest_hex != claim_digest {
        return Err(VerificationError::ClaimDigestMismatch);
    }
    if attestation.verifier_did == claim.subject_did {
        return Err(VerificationError::SelfAttestation);
    }
    if attestation.verifier_did != authority.assessor_did {
        return Err(VerificationError::AssessorMismatch);
    }
    if attestation.authority_id != authority.authority_id {
        return Err(VerificationError::AssessorMismatch);
    }
    if !trust.trusted_authority_ids.contains(&authority.authority_id) {
        return Err(VerificationError::UnknownAuthority);
    }
    if trust.revoked_authority_ids.contains(&authority.authority_id) {
        return Err(VerificationError::RevokedAuthority);
    }
    if authority.valid_from_unix > attestation.observed_at_unix
        || authority.valid_until_unix < attestation.observed_at_unix
        || authority.valid_from_unix > now_unix
        || authority.valid_until_unix < now_unix
    {
        return Err(VerificationError::AuthorityOutsideValidity);
    }
    if authority.professional_domain != policy.professional_domain
        || attestation.professional_domain != policy.professional_domain
        || !authority.scopes.iter().any(|scope| scope == &policy.required_scope)
        || attestation.scope != policy.required_scope
        || authority.jurisdiction != policy.jurisdiction
        || attestation.jurisdiction != policy.jurisdiction
        || claim.professional_domain != policy.professional_domain
    {
        return Err(VerificationError::AuthorityOutOfScope);
    }
    if attestation.observed_at_unix > now_unix {
        return Err(VerificationError::FutureObservation);
    }
    if attestation.observed_at_unix < claim.created_at_unix {
        return Err(VerificationError::PrematureAttestation);
    }
    if let Some(max_age) = policy.max_attestation_age_seconds {
        if now_unix.saturating_sub(attestation.observed_at_unix) > max_age {
            return Err(VerificationError::AttestationTooOld);
        }
    }
    if !attestation.conflicts_declared_complete {
        return Err(VerificationError::ConflictDisclosureIncomplete);
    }
    if attestation
        .conflicts
        .iter()
        .any(|conflict| policy.prohibited_conflicts.contains(conflict))
    {
        return Err(VerificationError::ProhibitedConflict);
    }
    if attestation.outcome != AttestationOutcome::Corroborated {
        return Err(VerificationError::NonCorroboratingOutcome);
    }

    let key = trust
        .keys
        .get(&authority.issuer_key_id)
        .ok_or(VerificationError::UnknownKey)?;
    if key.controller_did != authority.issuer_did {
        return Err(VerificationError::KeyControllerMismatch);
    }
    verify_key_signature(
        key,
        &authority_payload(authority),
        &authority.signature_hex,
        attestation.observed_at_unix,
        now_unix,
    )?;

    let assessor_key = trust
        .keys
        .get(&attestation.assessor_key_id)
        .ok_or(VerificationError::UnknownKey)?;
    if assessor_key.controller_did != attestation.verifier_did {
        return Err(VerificationError::KeyControllerMismatch);
    }
    verify_key_signature(
        assessor_key,
        &attestation_payload(attestation),
        &attestation.signature_hex,
        attestation.observed_at_unix,
        now_unix,
    )?;

    let keys = replay_keys(attestation);
    if keys.iter().any(|key| consumed_replay_keys.contains(key)) {
        return Err(VerificationError::Replay);
    }
    Ok(VerifiedAttestationV1 {
        attestation_id: attestation.attestation_id.clone(),
        verifier_did: attestation.verifier_did.clone(),
        authority_id: attestation.authority_id.clone(),
        replay_keys: keys.to_vec(),
    })
}

/// Verify a complete work-evidence bundle. A passing result means that the
/// required number of distinct, in-scope assessors signed non-conflicted
/// corroboration statements under trusted, time-valid authority. It does not
/// establish the truth of the underlying claim; that remains an evidentiary
/// and adjudicative question outside cryptographic verification.
pub fn verify_work_evidence_bundle(
    claim: &WorkEvidenceClaimV1,
    authorities: &BTreeMap<String, AssessorAuthorityV1>,
    attestations: &[WorkAttestationV1],
    policy: &VerificationPolicyV1,
    trust: &TrustStoreV1,
    now_unix: u64,
    consumed_replay_keys: &BTreeSet<String>,
) -> VerifyResult<VerifiedWorkEvidenceV1> {
    validate_claim(claim)?;
    validate_claim_timestamps(claim, now_unix)?;
    validate_policy(policy)?;
    if attestations.is_empty() || attestations.len() > MAX_ATTESTATIONS {
        return Err(VerificationError::TooManyAttestations);
    }

    let digest = sha256_hex(&claim_payload(claim));
    let mut seen_assessors = BTreeSet::new();
    let mut seen_replay_keys = BTreeSet::new();
    let mut verified = Vec::with_capacity(attestations.len());

    for attestation in attestations {
        if !seen_assessors.insert(attestation.verifier_did.as_str()) {
            return Err(VerificationError::DuplicateAssessor);
        }
        let authority = authorities
            .get(&attestation.authority_id)
            .ok_or(VerificationError::UnknownAuthority)?;
        let item = verify_one_attestation(
            &digest,
            claim,
            authority,
            attestation,
            policy,
            trust,
            now_unix,
            consumed_replay_keys,
        )?;
        for replay_key in &item.replay_keys {
            if !seen_replay_keys.insert(replay_key.clone()) {
                return Err(VerificationError::Replay);
            }
        }
        verified.push(item);
    }

    if verified.len() < usize::from(policy.minimum_distinct_assessors) {
        return Err(VerificationError::InsufficientDistinctAssessors);
    }

    Ok(VerifiedWorkEvidenceV1 {
        claim_id: claim.claim_id.clone(),
        claim_revision: claim.revision,
        claim_digest_hex: digest,
        attestations: verified,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};

    const NOW: u64 = 1_800_000_000;

    #[derive(Deserialize)]
    struct Corpus {
        cases: Vec<CorpusCase>,
    }

    #[derive(Deserialize)]
    struct CorpusCase {
        case_id: String,
        mutation: String,
        expected_accept: bool,
    }

    fn issuer_key() -> SigningKey {
        SigningKey::from_bytes(&[7; 32])
    }

    fn assessor_key() -> SigningKey {
        SigningKey::from_bytes(&[8; 32])
    }

    fn hex(bytes: &[u8]) -> String {
        hex_encode(bytes)
    }

    fn sign_authority(authority: &mut AssessorAuthorityV1, key: &SigningKey) {
        authority.signature_hex = hex(&key.sign(&authority_payload(authority)).to_bytes());
    }

    fn sign_attestation(attestation: &mut WorkAttestationV1, key: &SigningKey) {
        attestation.signature_hex = hex(&key.sign(&attestation_payload(attestation)).to_bytes());
    }

    fn sample_claim() -> WorkEvidenceClaimV1 {
        WorkEvidenceClaimV1 {
            schema_version: 1,
            claim_id: "repair-pump-0001".into(),
            revision: 1,
            subject_did: "did:mycelix:worker-1".into(),
            organization: "Community Water Cooperative".into(),
            professional_domain: "water-systems".into(),
            participant_role: "repair-technician".into(),
            contribution_scope: "diagnose and repair circulation pump".into(),
            outcome_summary: "Pump restored to the declared flow range".into(),
            agreement_ref: Some("agreement:repair-001".into()),
            work_order_ref: Some("work-order:repair-001".into()),
            observed_from_unix: NOW - 3600,
            observed_until_unix: NOW - 120,
            created_at_unix: NOW - 60,
            evidence: vec![EvidenceArtifactRefV1 {
                artifact_id: "flow-test-after".into(),
                uri: "urn:fixture:flow-test-after".into(),
                media_type: "application/json".into(),
                sha256_hex: sha256_hex(b"{\"flow_lpm\": 18.2}"),
                captured_at_unix: NOW - 120,
                provenance: EvidenceProvenanceV1 {
                    actor_did: "did:mycelix:inspector-2".into(),
                    method: EvidenceCaptureMethod::InstrumentMeasurement,
                    source_ref: Some("sensor:flow-meter-01".into()),
                },
            }],
        }
    }

    fn sample_authority() -> AssessorAuthorityV1 {
        let mut authority = AssessorAuthorityV1 {
            schema_version: 1,
            authority_id: "authority:water-guild-za".into(),
            revision: 1,
            issuer_did: "did:mycelix:water-guild".into(),
            issuer_key_id: "key:water-guild-1".into(),
            assessor_did: "did:mycelix:inspector-2".into(),
            professional_domain: "water-systems".into(),
            scopes: vec!["pump-repair".into(), "flow-inspection".into()],
            jurisdiction: "ZA-GP".into(),
            valid_from_unix: NOW - 86_400,
            valid_until_unix: NOW + 86_400,
            signature_hex: String::new(),
        };
        sign_authority(&mut authority, &issuer_key());
        authority
    }

    fn sample_attestation(claim: &WorkEvidenceClaimV1) -> WorkAttestationV1 {
        let mut attestation = WorkAttestationV1 {
            schema_version: 1,
            attestation_id: "attestation:0001".into(),
            claim_digest_hex: claim_digest_hex(claim).unwrap(),
            verifier_did: "did:mycelix:inspector-2".into(),
            assessor_key_id: "key:inspector-2".into(),
            authority_id: "authority:water-guild-za".into(),
            professional_domain: "water-systems".into(),
            scope: "pump-repair".into(),
            jurisdiction: "ZA-GP".into(),
            outcome: AttestationOutcome::Corroborated,
            observed_at_unix: NOW - 30,
            nonce: "nonce-0001".into(),
            conflicts_declared_complete: true,
            conflicts: vec![],
            limitations: vec!["Single-cycle test; recurrence window not complete".into()],
            signature_hex: String::new(),
        };
        sign_attestation(&mut attestation, &assessor_key());
        attestation
    }

    fn sample_trust() -> TrustStoreV1 {
        let issuer_public = issuer_key().verifying_key().to_bytes();
        let assessor_public = assessor_key().verifying_key().to_bytes();
        let mut keys = BTreeMap::new();
        keys.insert(
            "key:water-guild-1".into(),
            TrustedKeyV1 {
                controller_did: "did:mycelix:water-guild".into(),
                public_key_hex: hex(&issuer_public),
                valid_from_unix: NOW - 86_400,
                valid_until_unix: NOW + 86_400,
                revoked: false,
            },
        );
        keys.insert(
            "key:inspector-2".into(),
            TrustedKeyV1 {
                controller_did: "did:mycelix:inspector-2".into(),
                public_key_hex: hex(&assessor_public),
                valid_from_unix: NOW - 86_400,
                valid_until_unix: NOW + 86_400,
                revoked: false,
            },
        );
        TrustStoreV1 {
            trusted_authority_ids: BTreeSet::from(["authority:water-guild-za".into()]),
            revoked_authority_ids: BTreeSet::new(),
            keys,
        }
    }

    fn sample_policy() -> VerificationPolicyV1 {
        VerificationPolicyV1 {
            professional_domain: "water-systems".into(),
            required_scope: "pump-repair".into(),
            jurisdiction: "ZA-GP".into(),
            minimum_distinct_assessors: 1,
            max_attestation_age_seconds: Some(86_400),
            prohibited_conflicts: BTreeSet::from([
                ConflictKind::SelfInterest,
                ConflictKind::Family,
                ConflictKind::Financial,
                ConflictKind::Supervisory,
                ConflictKind::BusinessPartner,
            ]),
        }
    }

    fn verify_fixture(
        mutation: &str,
    ) -> VerifyResult<VerifiedWorkEvidenceV1> {
        let mut claim = sample_claim();
        let mut authority = sample_authority();
        let mut attestation = sample_attestation(&claim);
        let mut trust = sample_trust();
        let policy = sample_policy();
        let mut consumed = BTreeSet::new();

        match mutation {
            "valid" => {}
            "claim_digest_mismatch" => {
                attestation.claim_digest_hex = "00".repeat(32);
                sign_attestation(&mut attestation, &assessor_key());
            }
            "self_attestation" => {
                claim.subject_did = attestation.verifier_did.clone();
                attestation.claim_digest_hex = claim_digest_hex(&claim).unwrap();
                sign_attestation(&mut attestation, &assessor_key());
            }
            "expired_authority" => {
                authority.valid_until_unix = NOW - 1;
                sign_authority(&mut authority, &issuer_key());
            }
            "untrusted_authority" => {
                trust.trusted_authority_ids.clear();
            }
            "revoked_authority" => {
                trust.revoked_authority_ids.insert(authority.authority_id.clone());
            }
            "authority_scope_mismatch" => {
                authority.scopes = vec!["electrical-maintenance".into()];
                sign_authority(&mut authority, &issuer_key());
            }
            "attestation_scope_mismatch" => {
                attestation.scope = "electrical-maintenance".into();
                sign_attestation(&mut attestation, &assessor_key());
            }
            "incomplete_conflict_disclosure" => {
                attestation.conflicts_declared_complete = false;
                sign_attestation(&mut attestation, &assessor_key());
            }
            "prohibited_conflict" => {
                attestation.conflicts.push(ConflictKind::Financial);
                sign_attestation(&mut attestation, &assessor_key());
            }
            "tampered_attestation_signature" => {
                attestation.signature_hex = "00".repeat(64);
            }
            "replayed_nonce" => {
                consumed.insert(replay_keys(&attestation)[0].clone());
            }
            "replayed_attestation_id" => {
                consumed.insert(replay_keys(&attestation)[1].clone());
            }
            "wrong_assessor" => {
                authority.assessor_did = "did:mycelix:unlicensed-party".into();
                sign_authority(&mut authority, &issuer_key());
            }
            "tampered_authority" => {
                authority.signature_hex = "00".repeat(64);
            }
            "future_observation" => {
                attestation.observed_at_unix = NOW + 1;
                sign_attestation(&mut attestation, &assessor_key());
            }
            "future_claim_creation" => {
                claim.created_at_unix = NOW + 1;
                attestation.claim_digest_hex = claim_digest_hex(&claim).unwrap();
                sign_attestation(&mut attestation, &assessor_key());
            }
            "future_artifact_capture" => {
                claim.evidence[0].captured_at_unix = NOW + 1;
                attestation.claim_digest_hex = claim_digest_hex(&claim).unwrap();
                sign_attestation(&mut attestation, &assessor_key());
            }
            "attestation_precedes_claim" => {
                attestation.observed_at_unix = claim.created_at_unix - 1;
                sign_attestation(&mut attestation, &assessor_key());
            }
            "unsupported_schema" => {
                attestation.schema_version = 2;
                sign_attestation(&mut attestation, &assessor_key());
            }
            "disputed_outcome" => {
                attestation.outcome = AttestationOutcome::Disputed;
                sign_attestation(&mut attestation, &assessor_key());
            }
            "revoked_assessor_key" => {
                trust.keys.get_mut("key:inspector-2").unwrap().revoked = true;
            }
            "expired_assessor_key" => {
                trust.keys.get_mut("key:inspector-2").unwrap().valid_until_unix = NOW - 1;
            }
            other => panic!("unknown corpus mutation: {other}"),
        }

        let authorities = BTreeMap::from([(authority.authority_id.clone(), authority)]);
        verify_work_evidence_bundle(
            &claim,
            &authorities,
            &[attestation],
            &policy,
            &trust,
            NOW,
            &consumed,
        )
    }

    #[test]
    fn adversarial_corpus_matches_expected_dispositions() {
        let corpus: Corpus =
            serde_json::from_str(include_str!("../fixtures/adversarial-v1.json")).unwrap();
        assert_eq!(corpus.cases.len(), 23);
        for case in corpus.cases {
            let result = verify_fixture(&case.mutation);
            assert_eq!(
                result.is_ok(),
                case.expected_accept,
                "{} ({}) unexpectedly produced: {:?}",
                case.case_id,
                case.mutation,
                result.err()
            );
        }
    }

    #[test]
    fn canonical_claim_digest_is_order_independent_for_artifacts() {
        let mut a = sample_claim();
        let mut b = a.clone();
        b.evidence.reverse();
        assert_eq!(claim_digest_hex(&a).unwrap(), claim_digest_hex(&b).unwrap());
        a.revision += 1;
        assert_ne!(claim_digest_hex(&a).unwrap(), claim_digest_hex(&b).unwrap());
    }

    #[test]
    fn artifact_bytes_must_match_the_declared_digest() {
        let claim = sample_claim();
        let artifact = &claim.evidence[0];
        assert!(verify_artifact_bytes(artifact, b"{\"flow_lpm\": 18.2}").is_ok());
        assert_eq!(
            verify_artifact_bytes(artifact, b"{\"flow_lpm\": 0.1}"),
            Err(VerificationError::ArtifactDigestMismatch)
        );
    }

    #[test]
    fn distinct_assessors_can_satisfy_policy_quorum() {
        let claim = sample_claim();
        let first_authority = sample_authority();
        let second_signer = SigningKey::from_bytes(&[9; 32]);
        let second_authority_id = "authority:independent-inspectors-za";

        let mut second_authority = AssessorAuthorityV1 {
            schema_version: 1,
            authority_id: second_authority_id.into(),
            revision: 1,
            issuer_did: "did:mycelix:water-guild".into(),
            issuer_key_id: "key:water-guild-1".into(),
            assessor_did: "did:mycelix:inspector-3".into(),
            professional_domain: "water-systems".into(),
            scopes: vec!["pump-repair".into()],
            jurisdiction: "ZA-GP".into(),
            valid_from_unix: NOW - 86_400,
            valid_until_unix: NOW + 86_400,
            signature_hex: String::new(),
        };
        sign_authority(&mut second_authority, &issuer_key());

        let first = sample_attestation(&claim);
        let mut second = sample_attestation(&claim);
        second.attestation_id = "attestation:0002".into();
        second.verifier_did = "did:mycelix:inspector-3".into();
        second.assessor_key_id = "key:inspector-3".into();
        second.authority_id = second_authority_id.into();
        second.nonce = "nonce-0002".into();
        sign_attestation(&mut second, &second_signer);

        let mut trust = sample_trust();
        trust.trusted_authority_ids.insert(second_authority_id.into());
        trust.keys.insert(
            "key:inspector-3".into(),
            TrustedKeyV1 {
                controller_did: "did:mycelix:inspector-3".into(),
                public_key_hex: hex(&second_signer.verifying_key().to_bytes()),
                valid_from_unix: NOW - 86_400,
                valid_until_unix: NOW + 86_400,
                revoked: false,
            },
        );

        let authorities = BTreeMap::from([
            (first_authority.authority_id.clone(), first_authority),
            (second_authority.authority_id.clone(), second_authority),
        ]);
        let mut policy = sample_policy();
        policy.minimum_distinct_assessors = 2;

        let result = verify_work_evidence_bundle(
            &claim,
            &authorities,
            &[first, second],
            &policy,
            &trust,
            NOW,
            &BTreeSet::new(),
        )
        .unwrap();

        assert_eq!(result.attestations.len(), 2);
        assert_ne!(
            result.attestations[0].verifier_did,
            result.attestations[1].verifier_did
        );
    }

    #[test]
    fn duplicate_assessor_cannot_satisfy_independent_quorum() {
        let claim = sample_claim();
        let authority = sample_authority();
        let first = sample_attestation(&claim);
        let mut second = first.clone();
        second.attestation_id = "attestation:0002".into();
        second.nonce = "nonce-0002".into();
        sign_attestation(&mut second, &assessor_key());
        let authorities = BTreeMap::from([(authority.authority_id.clone(), authority)]);
        let mut policy = sample_policy();
        policy.minimum_distinct_assessors = 2;
        let result = verify_work_evidence_bundle(
            &claim,
            &authorities,
            &[first, second],
            &policy,
            &sample_trust(),
            NOW,
            &BTreeSet::new(),
        );
        assert_eq!(result, Err(VerificationError::DuplicateAssessor));
    }
}

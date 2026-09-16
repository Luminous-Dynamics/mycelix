use std::collections::BTreeSet;

use mycelix_business_core::{Digest32, ExecutionAttemptRef, ReferenceId};
use sha2::{Digest, Sha256};

use crate::{
    FinalityEvidence, FinalityProfile, SettlementEvaluationContext,
    SettlementObservation, SettlementSubject,
};

pub const SETTLEMENT_COMMITMENT_PROFILE_REVISION: u16 = 1;

const PROFILE_DOMAIN: &[u8] = b"MYCELIX_FINANCE_SETTLEMENT_FINALITY_PROFILE_V1\0";
const EVIDENCE_DOMAIN: &[u8] = b"MYCELIX_FINANCE_SETTLEMENT_EVIDENCE_V1\0";
const OBSERVATION_DOMAIN: &[u8] = b"MYCELIX_FINANCE_SETTLEMENT_OBSERVATION_V1\0";
const EVALUATION_CONTEXT_DOMAIN: &[u8] =
    b"MYCELIX_FINANCE_SETTLEMENT_EVALUATION_CONTEXT_V1\0";
const FRONTIER_DOMAIN: &[u8] = b"MYCELIX_FINANCE_SETTLEMENT_FRONTIER_V1\0";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalEncodingError {
    LengthOverflow,
}

pub fn canonical_finality_profile_bytes(
    profile: &FinalityProfile,
) -> Result<Vec<u8>, CanonicalEncodingError> {
    let mut out = Vec::with_capacity(256);
    out.extend_from_slice(PROFILE_DOMAIN);
    push_u16(&mut out, profile.commitment_profile_revision);
    push_reference(&mut out, &profile.profile_ref.id)?;
    push_u64(&mut out, profile.profile_ref.revision);
    push_reference(&mut out, &profile.rail)?;
    push_reference(&mut out, &profile.network)?;
    push_reference_set(&mut out, &profile.required_evidence_kinds)?;
    push_u16(&mut out, profile.min_distinct_sources);
    push_u64(&mut out, profile.max_observation_age_ms);
    push_u8(&mut out, profile.reversal_model.canonical_tag());
    Ok(out)
}

pub fn finality_profile_commitment(
    profile: &FinalityProfile,
) -> Result<Digest32, CanonicalEncodingError> {
    Ok(sha256(&canonical_finality_profile_bytes(profile)?))
}

pub fn canonical_evaluation_context_bytes(
    context: &SettlementEvaluationContext,
) -> Result<Vec<u8>, CanonicalEncodingError> {
    let mut out = Vec::with_capacity(192);
    out.extend_from_slice(EVALUATION_CONTEXT_DOMAIN);
    push_u16(&mut out, context.commitment_profile_revision);
    push_u8(&mut out, context.class.canonical_tag());
    push_u64(&mut out, context.evaluation_time_unix_ms);
    push_reference(&mut out, &context.temporal_profile_id)?;
    push_u64(&mut out, context.temporal_profile_revision);
    push_digest(&mut out, context.temporal_context_digest);
    Ok(out)
}

pub fn evaluation_context_commitment(
    context: &SettlementEvaluationContext,
) -> Result<Digest32, CanonicalEncodingError> {
    Ok(sha256(&canonical_evaluation_context_bytes(context)?))
}

pub fn canonical_evidence_bytes(
    evidence: &FinalityEvidence,
) -> Result<Vec<u8>, CanonicalEncodingError> {
    let mut out = Vec::with_capacity(256);
    out.extend_from_slice(EVIDENCE_DOMAIN);
    push_u16(&mut out, SETTLEMENT_COMMITMENT_PROFILE_REVISION);
    push_reference(&mut out, &evidence.evidence_id)?;
    push_reference(&mut out, &evidence.subject)?;
    push_reference(&mut out, &evidence.operation_id)?;
    push_u64(&mut out, evidence.operation_revision);
    push_reference(&mut out, &evidence.observation_id)?;
    push_reference(&mut out, &evidence.kind)?;
    push_reference(&mut out, &evidence.source)?;
    push_digest(&mut out, evidence.digest);
    Ok(out)
}

pub fn evidence_commitment(
    evidence: &FinalityEvidence,
) -> Result<Digest32, CanonicalEncodingError> {
    Ok(sha256(&canonical_evidence_bytes(evidence)?))
}

pub fn canonical_observation_bytes(
    observation: &SettlementObservation,
) -> Result<Vec<u8>, CanonicalEncodingError> {
    let mut evidence_commitments = BTreeSet::new();
    for evidence in &observation.evidence {
        evidence_commitments.insert(evidence_commitment(evidence)?);
    }

    let mut out = Vec::with_capacity(512);
    out.extend_from_slice(OBSERVATION_DOMAIN);
    push_u16(&mut out, SETTLEMENT_COMMITMENT_PROFILE_REVISION);
    push_reference(&mut out, &observation.observation_id)?;
    push_reference(&mut out, &observation.subject)?;
    push_digest(&mut out, observation.financial_effect_commitment);
    push_attempt(&mut out, &observation.attempt)?;
    push_reference(&mut out, &observation.rail)?;
    push_reference(&mut out, &observation.network)?;
    push_reference(&mut out, &observation.operation_id)?;
    push_u64(&mut out, observation.revision);
    push_string(&mut out, observation.amount.asset().as_str())?;
    push_u64(&mut out, observation.amount.atomic_units());
    push_u8(&mut out, observation.state.canonical_tag());
    push_u64(&mut out, observation.observed_at_unix_ms);
    push_digest_set(&mut out, &evidence_commitments)?;
    Ok(out)
}

pub fn observation_commitment(
    observation: &SettlementObservation,
) -> Result<Digest32, CanonicalEncodingError> {
    Ok(sha256(&canonical_observation_bytes(observation)?))
}

pub fn canonical_selected_evidence_frontier_bytes(
    subject: &SettlementSubject,
    profile: &FinalityProfile,
    evaluation_context_commitment: Digest32,
    selected_observation_commitments: &BTreeSet<Digest32>,
) -> Result<Vec<u8>, CanonicalEncodingError> {
    let mut out = Vec::with_capacity(512);
    out.extend_from_slice(FRONTIER_DOMAIN);
    push_u16(&mut out, SETTLEMENT_COMMITMENT_PROFILE_REVISION);
    push_reference(&mut out, &subject.id)?;
    push_digest(&mut out, subject.financial_effect_commitment);
    push_attempt(&mut out, &subject.attempt)?;
    push_reference(&mut out, &subject.rail)?;
    push_reference(&mut out, &subject.network)?;
    push_digest(&mut out, profile.profile_ref.digest);
    push_string(&mut out, subject.amount.asset().as_str())?;
    push_u64(&mut out, subject.amount.atomic_units());
    push_digest(&mut out, evaluation_context_commitment);
    push_digest_set(&mut out, selected_observation_commitments)?;
    Ok(out)
}

pub fn selected_evidence_frontier_commitment(
    subject: &SettlementSubject,
    profile: &FinalityProfile,
    evaluation_context_commitment: Digest32,
    selected_observation_commitments: &BTreeSet<Digest32>,
) -> Result<Digest32, CanonicalEncodingError> {
    Ok(sha256(&canonical_selected_evidence_frontier_bytes(
        subject,
        profile,
        evaluation_context_commitment,
        selected_observation_commitments,
    )?))
}

fn sha256(bytes: &[u8]) -> Digest32 {
    let digest = Sha256::digest(bytes);
    let mut out = [0_u8; 32];
    out.copy_from_slice(&digest);
    Digest32(out)
}

fn push_u8(out: &mut Vec<u8>, value: u8) {
    out.push(value);
}

fn push_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_digest(out: &mut Vec<u8>, digest: Digest32) {
    out.extend_from_slice(&digest.0);
}

fn push_string(out: &mut Vec<u8>, value: &str) -> Result<(), CanonicalEncodingError> {
    let length = u32::try_from(value.len()).map_err(|_| CanonicalEncodingError::LengthOverflow)?;
    push_u32(out, length);
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_reference(
    out: &mut Vec<u8>,
    reference: &ReferenceId,
) -> Result<(), CanonicalEncodingError> {
    push_string(out, reference.as_str())
}

fn push_attempt(
    out: &mut Vec<u8>,
    attempt: &ExecutionAttemptRef,
) -> Result<(), CanonicalEncodingError> {
    push_reference(out, attempt.as_ref_id())
}

fn push_reference_set(
    out: &mut Vec<u8>,
    values: &BTreeSet<ReferenceId>,
) -> Result<(), CanonicalEncodingError> {
    let count = u32::try_from(values.len()).map_err(|_| CanonicalEncodingError::LengthOverflow)?;
    push_u32(out, count);
    for value in values {
        push_reference(out, value)?;
    }
    Ok(())
}

fn push_digest_set(
    out: &mut Vec<u8>,
    values: &BTreeSet<Digest32>,
) -> Result<(), CanonicalEncodingError> {
    let count = u32::try_from(values.len()).map_err(|_| CanonicalEncodingError::LengthOverflow)?;
    push_u32(out, count);
    for value in values {
        push_digest(out, *value);
    }
    Ok(())
}

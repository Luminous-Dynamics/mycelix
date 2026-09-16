use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{Digest32, ReferenceId};
use sha2::{Digest, Sha256};

use crate::canonical::{
    CanonicalEncodingError, SETTLEMENT_COMMITMENT_PROFILE_REVISION, observation_commitment,
    selected_evidence_frontier_commitment,
};
use crate::model::{
    EvaluationContextClass, FinalityProfile, QualifiedSettlement, SettlementEvaluationContext,
    SettlementObservation, SettlementQualificationInvalidation, SettlementSubject,
};
use crate::qualify::{
    SettlementInvalidationError, SettlementQualificationError, derive_invalidation,
    qualify_settlement,
};

const INVALIDATION_RECEIPT_DOMAIN: &[u8] = b"MYCELIX_FINANCE_SETTLEMENT_INVALIDATION_RECEIPT_V1\0";

/// Failure while constructing an audit-complete settlement qualification receipt.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SettlementReceiptError {
    Qualification(SettlementQualificationError),
    Commitment(CanonicalEncodingError),
    FrontierMismatch,
}

impl From<SettlementQualificationError> for SettlementReceiptError {
    fn from(value: SettlementQualificationError) -> Self {
        Self::Qualification(value)
    }
}

impl From<CanonicalEncodingError> for SettlementReceiptError {
    fn from(value: CanonicalEncodingError) -> Self {
        Self::Commitment(value)
    }
}

/// Audit-complete wrapper around a sealed settlement proof.
///
/// `QualifiedSettlement` carries the derived frontier. This receipt additionally
/// preserves the exact set of current observation commitments selected by the
/// qualifier, including current rejected/reversed operations that do not
/// contribute settled value. Construction re-derives the frontier and fails if
/// those two representations disagree.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualifiedSettlementReceipt {
    settlement: QualifiedSettlement,
    selected_observation_commitments: BTreeSet<Digest32>,
}

impl QualifiedSettlementReceipt {
    pub fn settlement(&self) -> &QualifiedSettlement {
        &self.settlement
    }

    pub fn selected_observation_commitments(&self) -> &BTreeSet<Digest32> {
        &self.selected_observation_commitments
    }

    pub fn into_settlement(self) -> QualifiedSettlement {
        self.settlement
    }
}

/// Qualify a settlement and retain the complete current-observation selection
/// needed to independently reconstruct the derived evidence frontier.
pub fn qualify_settlement_with_receipt(
    subject: &SettlementSubject,
    profile: &FinalityProfile,
    observations: &[SettlementObservation],
    context: &SettlementEvaluationContext,
) -> Result<QualifiedSettlementReceipt, SettlementReceiptError> {
    let settlement = qualify_settlement(subject, profile, observations, context)?;
    let selected_observation_commitments = selected_current_observation_commitments(observations)?;
    let reconstructed = selected_evidence_frontier_commitment(
        subject,
        profile,
        context.commitment(),
        &selected_observation_commitments,
    )?;
    if reconstructed != settlement.evidence_frontier() {
        return Err(SettlementReceiptError::FrontierMismatch);
    }

    Ok(QualifiedSettlementReceipt {
        settlement,
        selected_observation_commitments,
    })
}

fn selected_current_observation_commitments(
    observations: &[SettlementObservation],
) -> Result<BTreeSet<Digest32>, CanonicalEncodingError> {
    let mut max_revision_by_operation: BTreeMap<ReferenceId, u64> = BTreeMap::new();
    for observation in observations {
        max_revision_by_operation
            .entry(observation.operation_id.clone())
            .and_modify(|revision| *revision = (*revision).max(observation.revision))
            .or_insert(observation.revision);
    }

    let mut selected = BTreeSet::new();
    for observation in observations {
        if max_revision_by_operation.get(&observation.operation_id) == Some(&observation.revision) {
            selected.insert(observation_commitment(observation)?);
        }
    }
    Ok(selected)
}

/// Audit-complete invalidation wrapper preserving the temporal context identity
/// under which the invalidation theorem was evaluated.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettlementInvalidationReceipt {
    invalidation: SettlementQualificationInvalidation,
    evaluation_context_commitment: Digest32,
    evaluation_context_class: EvaluationContextClass,
    receipt_commitment: Digest32,
}

impl SettlementInvalidationReceipt {
    pub fn invalidation(&self) -> &SettlementQualificationInvalidation {
        &self.invalidation
    }

    pub fn evaluation_context_commitment(&self) -> Digest32 {
        self.evaluation_context_commitment
    }

    pub fn evaluation_context_class(&self) -> EvaluationContextClass {
        self.evaluation_context_class
    }

    pub fn receipt_commitment(&self) -> Digest32 {
        self.receipt_commitment
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SettlementInvalidationReceiptError {
    Invalidation(SettlementInvalidationError),
    Commitment(CanonicalEncodingError),
}

impl From<SettlementInvalidationError> for SettlementInvalidationReceiptError {
    fn from(value: SettlementInvalidationError) -> Self {
        Self::Invalidation(value)
    }
}

impl From<CanonicalEncodingError> for SettlementInvalidationReceiptError {
    fn from(value: CanonicalEncodingError) -> Self {
        Self::Commitment(value)
    }
}

/// Derive an invalidation and preserve the full evaluation-context identity.
pub fn derive_invalidation_with_receipt(
    prior: &QualifiedSettlement,
    profile: &FinalityProfile,
    observation: &SettlementObservation,
    context: &SettlementEvaluationContext,
) -> Result<SettlementInvalidationReceipt, SettlementInvalidationReceiptError> {
    let invalidation = derive_invalidation(prior, profile, observation, context)?;
    let evaluation_context_commitment = context.commitment();
    let evaluation_context_class = context.class();
    let receipt_commitment = settlement_invalidation_receipt_commitment(
        &invalidation,
        evaluation_context_commitment,
        evaluation_context_class,
    )?;

    Ok(SettlementInvalidationReceipt {
        invalidation,
        evaluation_context_commitment,
        evaluation_context_class,
        receipt_commitment,
    })
}

/// Canonical language-neutral bytes for the invalidation receipt.
pub fn canonical_settlement_invalidation_receipt_bytes(
    invalidation: &SettlementQualificationInvalidation,
    evaluation_context_commitment: Digest32,
    evaluation_context_class: EvaluationContextClass,
) -> Result<Vec<u8>, CanonicalEncodingError> {
    let mut out = Vec::with_capacity(384);
    out.extend_from_slice(INVALIDATION_RECEIPT_DOMAIN);
    push_u16(&mut out, SETTLEMENT_COMMITMENT_PROFILE_REVISION);
    push_reference(&mut out, invalidation.subject())?;
    push_digest(&mut out, invalidation.financial_effect_commitment());
    push_reference(&mut out, &invalidation.prior_profile().id)?;
    push_u64(&mut out, invalidation.prior_profile().revision);
    push_digest(&mut out, invalidation.prior_profile().digest);
    push_reference(&mut out, invalidation.operation_id())?;
    push_u64(&mut out, invalidation.prior_revision());
    push_reference(&mut out, invalidation.invalidating_observation())?;
    push_digest(&mut out, invalidation.invalidating_observation_commitment());
    push_digest(&mut out, evaluation_context_commitment);
    push_u8(&mut out, evaluation_context_class.canonical_tag());
    push_u64(&mut out, invalidation.invalidated_at_unix_ms());
    push_reference_set(&mut out, invalidation.evidence_ids())?;
    Ok(out)
}

pub fn settlement_invalidation_receipt_commitment(
    invalidation: &SettlementQualificationInvalidation,
    evaluation_context_commitment: Digest32,
    evaluation_context_class: EvaluationContextClass,
) -> Result<Digest32, CanonicalEncodingError> {
    Ok(sha256(&canonical_settlement_invalidation_receipt_bytes(
        invalidation,
        evaluation_context_commitment,
        evaluation_context_class,
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

fn push_reference(
    out: &mut Vec<u8>,
    reference: &ReferenceId,
) -> Result<(), CanonicalEncodingError> {
    let bytes = reference.as_str().as_bytes();
    let length = u32::try_from(bytes.len()).map_err(|_| CanonicalEncodingError::LengthOverflow)?;
    push_u32(out, length);
    out.extend_from_slice(bytes);
    Ok(())
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

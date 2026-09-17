use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_finance_exact::{AssetAmount, ExactArithmeticError};

use crate::{
    CanonicalEncodingError, EvaluationContextError, FinalityEvidence, FinalityProfile,
    FinalityProfileError, ObservedSettlementState, QualifiedOperation, QualifiedSettlement,
    SettlementEvaluationContext, SettlementInvalidationParts, SettlementObservation,
    SettlementQualificationInvalidation, SettlementSubject, evidence_commitment,
    observation_commitment, selected_evidence_frontier_commitment,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SettlementQualificationError {
    InvalidProfile(FinalityProfileError),
    InvalidEvaluationContext(EvaluationContextError),
    ZeroSubjectAmount,
    ProfileMismatch,
    RailMismatch,
    NetworkMismatch,
    NoObservations,
    SubjectMismatch,
    EffectCommitmentMismatch,
    AttemptMismatch,
    ObservationRailMismatch,
    ObservationNetworkMismatch,
    ObservationIdConflict,
    ZeroObservationRevision,
    ZeroAppliedAmount,
    AssetMismatch,
    FutureObservation,
    StaleObservation,
    ConflictingCurrentRevision,
    MissingEvidence,
    EvidenceSubjectMismatch,
    EvidenceOperationMismatch,
    EvidenceRevisionMismatch,
    EvidenceObservationMismatch,
    EvidenceIdConflict,
    MissingEvidenceKind,
    InsufficientDistinctSources,
    OutstandingUncertainty,
    PartialSettlement,
    OverSettlement,
    Arithmetic(ExactArithmeticError),
    Commitment(CanonicalEncodingError),
}

impl From<ExactArithmeticError> for SettlementQualificationError {
    fn from(value: ExactArithmeticError) -> Self {
        Self::Arithmetic(value)
    }
}

impl From<CanonicalEncodingError> for SettlementQualificationError {
    fn from(value: CanonicalEncodingError) -> Self {
        Self::Commitment(value)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SettlementInvalidationError {
    InvalidProfile(FinalityProfileError),
    InvalidEvaluationContext(EvaluationContextError),
    ProfileMismatch,
    EvaluationPredatesPrior,
    SubjectMismatch,
    EffectCommitmentMismatch,
    AttemptMismatch,
    RailMismatch,
    NetworkMismatch,
    AssetMismatch,
    OperationNotPreviouslyQualified,
    RevisionDidNotAdvance,
    FutureObservation,
    StaleObservation,
    MissingEvidence,
    EvidenceSubjectMismatch,
    EvidenceOperationMismatch,
    EvidenceRevisionMismatch,
    EvidenceObservationMismatch,
    EvidenceIdConflict,
    MissingEvidenceKind,
    InsufficientDistinctSources,
    ObservationDoesNotInvalidate,
    Commitment(CanonicalEncodingError),
}

impl From<CanonicalEncodingError> for SettlementInvalidationError {
    fn from(value: CanonicalEncodingError) -> Self {
        Self::Commitment(value)
    }
}

fn validate_evidence_binding(
    observation: &SettlementObservation,
    evidence: &FinalityEvidence,
) -> Result<(), SettlementQualificationError> {
    if evidence.subject != observation.subject {
        return Err(SettlementQualificationError::EvidenceSubjectMismatch);
    }
    if evidence.operation_id != observation.operation_id {
        return Err(SettlementQualificationError::EvidenceOperationMismatch);
    }
    if evidence.operation_revision != observation.revision {
        return Err(SettlementQualificationError::EvidenceRevisionMismatch);
    }
    if evidence.observation_id != observation.observation_id {
        return Err(SettlementQualificationError::EvidenceObservationMismatch);
    }
    Ok(())
}

fn validate_invalidation_evidence_binding(
    observation: &SettlementObservation,
    evidence: &FinalityEvidence,
) -> Result<(), SettlementInvalidationError> {
    if evidence.subject != observation.subject {
        return Err(SettlementInvalidationError::EvidenceSubjectMismatch);
    }
    if evidence.operation_id != observation.operation_id {
        return Err(SettlementInvalidationError::EvidenceOperationMismatch);
    }
    if evidence.operation_revision != observation.revision {
        return Err(SettlementInvalidationError::EvidenceRevisionMismatch);
    }
    if evidence.observation_id != observation.observation_id {
        return Err(SettlementInvalidationError::EvidenceObservationMismatch);
    }
    Ok(())
}

pub fn qualify_settlement(
    subject: &SettlementSubject,
    profile: &FinalityProfile,
    observations: &[SettlementObservation],
    context: &SettlementEvaluationContext,
) -> Result<QualifiedSettlement, SettlementQualificationError> {
    profile
        .validate()
        .map_err(SettlementQualificationError::InvalidProfile)?;
    context
        .validate()
        .map_err(SettlementQualificationError::InvalidEvaluationContext)?;

    if subject.amount.atomic_units() == 0 {
        return Err(SettlementQualificationError::ZeroSubjectAmount);
    }
    if &subject.required_profile != profile.profile_ref() {
        return Err(SettlementQualificationError::ProfileMismatch);
    }
    if &subject.rail != profile.rail() {
        return Err(SettlementQualificationError::RailMismatch);
    }
    if &subject.network != profile.network() {
        return Err(SettlementQualificationError::NetworkMismatch);
    }
    if observations.is_empty() {
        return Err(SettlementQualificationError::NoObservations);
    }

    let mut by_operation: BTreeMap<ReferenceId, Vec<&SettlementObservation>> = BTreeMap::new();
    let mut observation_ids: BTreeMap<ReferenceId, Digest32> = BTreeMap::new();
    let mut evidence_ids: BTreeMap<ReferenceId, Digest32> = BTreeMap::new();

    for observation in observations {
        if observation.subject != subject.id {
            return Err(SettlementQualificationError::SubjectMismatch);
        }
        if observation.financial_effect_commitment != subject.financial_effect_commitment {
            return Err(SettlementQualificationError::EffectCommitmentMismatch);
        }
        if observation.attempt != subject.attempt {
            return Err(SettlementQualificationError::AttemptMismatch);
        }
        if observation.rail != subject.rail {
            return Err(SettlementQualificationError::ObservationRailMismatch);
        }
        if observation.network != subject.network {
            return Err(SettlementQualificationError::ObservationNetworkMismatch);
        }
        if observation.revision == 0 {
            return Err(SettlementQualificationError::ZeroObservationRevision);
        }
        if observation.amount.asset() != subject.amount.asset() {
            return Err(SettlementQualificationError::AssetMismatch);
        }
        if observation.observed_at_unix_ms > context.evaluation_time_unix_ms() {
            return Err(SettlementQualificationError::FutureObservation);
        }
        if observation.state == ObservedSettlementState::Applied
            && observation.amount.atomic_units() == 0
        {
            return Err(SettlementQualificationError::ZeroAppliedAmount);
        }

        for evidence in &observation.evidence {
            validate_evidence_binding(observation, evidence)?;
            let commitment = evidence_commitment(evidence)?;
            match evidence_ids.get(&evidence.evidence_id) {
                Some(existing) if *existing != commitment => {
                    return Err(SettlementQualificationError::EvidenceIdConflict);
                }
                Some(_) => {}
                None => {
                    evidence_ids.insert(evidence.evidence_id.clone(), commitment);
                }
            }
        }

        let commitment = observation_commitment(observation)?;
        match observation_ids.get(&observation.observation_id) {
            Some(existing) if *existing != commitment => {
                return Err(SettlementQualificationError::ObservationIdConflict);
            }
            Some(_) => {}
            None => {
                observation_ids.insert(observation.observation_id.clone(), commitment);
            }
        }

        by_operation
            .entry(observation.operation_id.clone())
            .or_default()
            .push(observation);
    }

    let mut settled = AssetAmount::new(0, subject.amount.asset().clone());
    let mut qualified_operations = BTreeMap::new();
    let mut selected_observation_commitments = BTreeSet::new();

    for (operation_id, operation_observations) in by_operation {
        let max_revision = operation_observations
            .iter()
            .map(|observation| observation.revision)
            .max()
            .expect("operation group is non-empty");
        let current: Vec<&SettlementObservation> = operation_observations
            .into_iter()
            .filter(|observation| observation.revision == max_revision)
            .collect();
        let first = current[0];

        for observation in &current {
            if observation.state != first.state || observation.amount != first.amount {
                return Err(SettlementQualificationError::ConflictingCurrentRevision);
            }
            if context
                .evaluation_time_unix_ms()
                .saturating_sub(observation.observed_at_unix_ms)
                > profile.max_observation_age_ms()
            {
                return Err(SettlementQualificationError::StaleObservation);
            }
            if observation.evidence.is_empty() {
                return Err(SettlementQualificationError::MissingEvidence);
            }
        }

        let mut merged_evidence = Vec::new();
        let mut operation_observation_commitments = BTreeSet::new();
        for observation in current {
            let commitment = observation_commitment(observation)?;
            selected_observation_commitments.insert(commitment);
            operation_observation_commitments.insert(commitment);
            merged_evidence.extend(observation.evidence.iter());
        }

        let kinds: BTreeSet<ReferenceId> = merged_evidence
            .iter()
            .map(|evidence| evidence.kind.clone())
            .collect();
        if !profile.required_evidence_kinds().is_subset(&kinds) {
            return Err(SettlementQualificationError::MissingEvidenceKind);
        }
        let sources: BTreeSet<ReferenceId> = merged_evidence
            .iter()
            .map(|evidence| evidence.source.clone())
            .collect();
        if sources.len() < usize::from(profile.min_distinct_sources()) {
            return Err(SettlementQualificationError::InsufficientDistinctSources);
        }

        match first.state {
            ObservedSettlementState::Pending | ObservedSettlementState::Unknown => {
                return Err(SettlementQualificationError::OutstandingUncertainty);
            }
            ObservedSettlementState::Rejected | ObservedSettlementState::Reversed => continue,
            ObservedSettlementState::Applied => {}
        }

        settled = settled.checked_add(&first.amount)?;
        let operation_evidence_ids = merged_evidence
            .iter()
            .map(|evidence| evidence.evidence_id.clone())
            .collect();
        qualified_operations.insert(
            operation_id,
            QualifiedOperation::new(
                max_revision,
                first.amount.clone(),
                operation_evidence_ids,
                operation_observation_commitments,
            ),
        );
    }

    if settled.atomic_units() < subject.amount.atomic_units() {
        return Err(SettlementQualificationError::PartialSettlement);
    }
    if settled.atomic_units() > subject.amount.atomic_units() {
        return Err(SettlementQualificationError::OverSettlement);
    }

    let frontier = selected_evidence_frontier_commitment(
        subject,
        profile,
        context.commitment(),
        &selected_observation_commitments,
    )?;

    Ok(QualifiedSettlement::new(
        subject.id.clone(),
        subject.financial_effect_commitment,
        subject.attempt.clone(),
        subject.rail.clone(),
        subject.network.clone(),
        subject.amount.clone(),
        profile.profile_ref().clone(),
        frontier,
        context.commitment(),
        context.class(),
        context.evaluation_time_unix_ms(),
        qualified_operations,
        profile.reversal_model(),
    ))
}

pub fn derive_invalidation(
    prior: &QualifiedSettlement,
    profile: &FinalityProfile,
    observation: &SettlementObservation,
    context: &SettlementEvaluationContext,
) -> Result<SettlementQualificationInvalidation, SettlementInvalidationError> {
    profile
        .validate()
        .map_err(SettlementInvalidationError::InvalidProfile)?;
    context
        .validate()
        .map_err(SettlementInvalidationError::InvalidEvaluationContext)?;

    if profile.profile_ref() != prior.profile() {
        return Err(SettlementInvalidationError::ProfileMismatch);
    }
    if context.evaluation_time_unix_ms() < prior.evaluated_at_unix_ms() {
        return Err(SettlementInvalidationError::EvaluationPredatesPrior);
    }
    if &observation.subject != prior.subject() {
        return Err(SettlementInvalidationError::SubjectMismatch);
    }
    if observation.financial_effect_commitment != prior.financial_effect_commitment() {
        return Err(SettlementInvalidationError::EffectCommitmentMismatch);
    }
    if &observation.attempt != prior.attempt() {
        return Err(SettlementInvalidationError::AttemptMismatch);
    }
    if &observation.rail != prior.rail() {
        return Err(SettlementInvalidationError::RailMismatch);
    }
    if &observation.network != prior.network() {
        return Err(SettlementInvalidationError::NetworkMismatch);
    }
    if observation.amount.asset() != prior.amount().asset() {
        return Err(SettlementInvalidationError::AssetMismatch);
    }
    let prior_operation = prior
        .operations()
        .get(&observation.operation_id)
        .ok_or(SettlementInvalidationError::OperationNotPreviouslyQualified)?;
    if observation.revision <= prior_operation.revision() {
        return Err(SettlementInvalidationError::RevisionDidNotAdvance);
    }
    if observation.observed_at_unix_ms > context.evaluation_time_unix_ms() {
        return Err(SettlementInvalidationError::FutureObservation);
    }
    if context
        .evaluation_time_unix_ms()
        .saturating_sub(observation.observed_at_unix_ms)
        > profile.max_observation_age_ms()
    {
        return Err(SettlementInvalidationError::StaleObservation);
    }
    if observation.evidence.is_empty() {
        return Err(SettlementInvalidationError::MissingEvidence);
    }

    let mut seen_evidence_ids = BTreeMap::new();
    for evidence in &observation.evidence {
        validate_invalidation_evidence_binding(observation, evidence)?;
        let commitment = evidence_commitment(evidence)?;
        match seen_evidence_ids.get(&evidence.evidence_id) {
            Some(existing) if *existing != commitment => {
                return Err(SettlementInvalidationError::EvidenceIdConflict);
            }
            Some(_) => {}
            None => {
                seen_evidence_ids.insert(evidence.evidence_id.clone(), commitment);
            }
        }
    }

    let kinds: BTreeSet<ReferenceId> = observation
        .evidence
        .iter()
        .map(|evidence| evidence.kind.clone())
        .collect();
    if !profile.required_evidence_kinds().is_subset(&kinds) {
        return Err(SettlementInvalidationError::MissingEvidenceKind);
    }
    let sources: BTreeSet<ReferenceId> = observation
        .evidence
        .iter()
        .map(|evidence| evidence.source.clone())
        .collect();
    if sources.len() < usize::from(profile.min_distinct_sources()) {
        return Err(SettlementInvalidationError::InsufficientDistinctSources);
    }

    let invalidates = observation.state != ObservedSettlementState::Applied
        || &observation.amount != prior_operation.amount();
    if !invalidates {
        return Err(SettlementInvalidationError::ObservationDoesNotInvalidate);
    }

    let observation_commitment = observation_commitment(observation)?;
    let evidence_ids = observation
        .evidence
        .iter()
        .map(|evidence| evidence.evidence_id.clone())
        .collect();

    Ok(SettlementQualificationInvalidation::from_parts(
        SettlementInvalidationParts {
            subject: prior.subject().clone(),
            financial_effect_commitment: prior.financial_effect_commitment(),
            prior_profile: prior.profile().clone(),
            operation_id: observation.operation_id.clone(),
            prior_revision: prior_operation.revision(),
            invalidating_observation: observation.observation_id.clone(),
            invalidating_observation_commitment: observation_commitment,
            invalidated_at_unix_ms: context.evaluation_time_unix_ms(),
            evidence_ids,
        },
    ))
}

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{Digest32, ExecutionAttemptRef, ReferenceId};
use mycelix_finance_exact::{AssetAmount, ExactArithmeticError};

use crate::{FinalityEvidence, FinalityProfile, FinalityProfileError, ObservedSettlementState, QualifiedOperation, QualifiedSettlement, SettlementObservation, SettlementSubject};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SettlementQualificationError {
    InvalidProfile(FinalityProfileError),
    ZeroSubjectAmount,
    ProfileMismatch,
    RailMismatch,
    NetworkMismatch,
    NoObservations,
    SubjectMismatch,
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
    EvidenceIdConflict,
    MissingEvidenceKind,
    InsufficientDistinctSources,
    OutstandingUncertainty,
    PartialSettlement,
    OverSettlement,
    Arithmetic(ExactArithmeticError),
}

impl From<ExactArithmeticError> for SettlementQualificationError {
    fn from(value: ExactArithmeticError) -> Self { Self::Arithmetic(value) }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ObservationFingerprint {
    subject: ReferenceId,
    attempt: ExecutionAttemptRef,
    rail: ReferenceId,
    network: ReferenceId,
    operation_id: ReferenceId,
    revision: u64,
    amount: AssetAmount,
    state: ObservedSettlementState,
    observed_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct EvidenceFingerprint {
    subject: ReferenceId,
    operation_id: ReferenceId,
    kind: ReferenceId,
    source: ReferenceId,
    digest: Digest32,
}

#[derive(Debug, Clone)]
struct CurrentOperation {
    revision: u64,
    state: ObservedSettlementState,
    amount: AssetAmount,
    observed_at_unix_ms: u64,
    evidence: Vec<FinalityEvidence>,
}

pub fn qualify_settlement(
    subject: &SettlementSubject,
    profile: &FinalityProfile,
    observations: &[SettlementObservation],
    qualified_at_unix_ms: u64,
    evidence_frontier: Digest32,
) -> Result<QualifiedSettlement, SettlementQualificationError> {
    profile.validate().map_err(SettlementQualificationError::InvalidProfile)?;
    if subject.amount.atomic_units() == 0 { return Err(SettlementQualificationError::ZeroSubjectAmount); }
    if subject.required_profile != profile.profile_ref { return Err(SettlementQualificationError::ProfileMismatch); }
    if subject.rail != profile.rail { return Err(SettlementQualificationError::RailMismatch); }
    if subject.network != profile.network { return Err(SettlementQualificationError::NetworkMismatch); }
    if observations.is_empty() { return Err(SettlementQualificationError::NoObservations); }

    let mut by_operation: BTreeMap<ReferenceId, Vec<&SettlementObservation>> = BTreeMap::new();
    let mut observation_ids: BTreeMap<ReferenceId, ObservationFingerprint> = BTreeMap::new();

    for observation in observations {
        if observation.subject != subject.id { return Err(SettlementQualificationError::SubjectMismatch); }
        if observation.attempt != subject.attempt { return Err(SettlementQualificationError::AttemptMismatch); }
        if observation.rail != subject.rail { return Err(SettlementQualificationError::ObservationRailMismatch); }
        if observation.network != subject.network { return Err(SettlementQualificationError::ObservationNetworkMismatch); }
        if observation.revision == 0 { return Err(SettlementQualificationError::ZeroObservationRevision); }
        if observation.amount.asset() != subject.amount.asset() { return Err(SettlementQualificationError::AssetMismatch); }
        if observation.observed_at_unix_ms > qualified_at_unix_ms { return Err(SettlementQualificationError::FutureObservation); }
        if observation.state == ObservedSettlementState::Applied && observation.amount.atomic_units() == 0 {
            return Err(SettlementQualificationError::ZeroAppliedAmount);
        }

        let fingerprint = ObservationFingerprint {
            subject: observation.subject.clone(),
            attempt: observation.attempt.clone(),
            rail: observation.rail.clone(),
            network: observation.network.clone(),
            operation_id: observation.operation_id.clone(),
            revision: observation.revision,
            amount: observation.amount.clone(),
            state: observation.state,
            observed_at_unix_ms: observation.observed_at_unix_ms,
        };
        match observation_ids.get(&observation.observation_id) {
            Some(existing) if existing != &fingerprint => return Err(SettlementQualificationError::ObservationIdConflict),
            Some(_) => {}
            None => { observation_ids.insert(observation.observation_id.clone(), fingerprint); }
        }
        by_operation.entry(observation.operation_id.clone()).or_default().push(observation);
    }

    let mut current_operations: BTreeMap<ReferenceId, CurrentOperation> = BTreeMap::new();
    let mut global_evidence_ids: BTreeMap<ReferenceId, EvidenceFingerprint> = BTreeMap::new();

    for (operation_id, operation_observations) in by_operation {
        let max_revision = operation_observations.iter().map(|observation| observation.revision).max().expect("operation group is non-empty");
        let current: Vec<&SettlementObservation> = operation_observations.into_iter().filter(|observation| observation.revision == max_revision).collect();
        let first = current[0];
        let mut merged_evidence = Vec::new();
        let mut observed_at_unix_ms = first.observed_at_unix_ms;

        for observation in current {
            if observation.state != first.state || observation.amount != first.amount {
                return Err(SettlementQualificationError::ConflictingCurrentRevision);
            }
            observed_at_unix_ms = observed_at_unix_ms.max(observation.observed_at_unix_ms);
            merged_evidence.extend(observation.evidence.iter().cloned());
        }
        if qualified_at_unix_ms.saturating_sub(observed_at_unix_ms) > profile.max_observation_age_ms {
            return Err(SettlementQualificationError::StaleObservation);
        }
        if merged_evidence.is_empty() { return Err(SettlementQualificationError::MissingEvidence); }

        for evidence in &merged_evidence {
            if evidence.subject != subject.id { return Err(SettlementQualificationError::EvidenceSubjectMismatch); }
            if evidence.operation_id != operation_id { return Err(SettlementQualificationError::EvidenceOperationMismatch); }
            let fingerprint = EvidenceFingerprint {
                subject: evidence.subject.clone(),
                operation_id: evidence.operation_id.clone(),
                kind: evidence.kind.clone(),
                source: evidence.source.clone(),
                digest: evidence.digest,
            };
            match global_evidence_ids.get(&evidence.evidence_id) {
                Some(existing) if existing != &fingerprint => return Err(SettlementQualificationError::EvidenceIdConflict),
                Some(_) => {}
                None => { global_evidence_ids.insert(evidence.evidence_id.clone(), fingerprint); }
            }
        }

        current_operations.insert(operation_id, CurrentOperation {
            revision: max_revision,
            state: first.state,
            amount: first.amount.clone(),
            observed_at_unix_ms,
            evidence: merged_evidence,
        });
    }

    let mut settled = AssetAmount::new(0, subject.amount.asset().clone());
    let mut qualified_operations = BTreeMap::new();
    for (operation_id, operation) in current_operations {
        let _current_observation_time = operation.observed_at_unix_ms;
        let kinds: BTreeSet<ReferenceId> = operation.evidence.iter().map(|evidence| evidence.kind.clone()).collect();
        if !profile.required_evidence_kinds.is_subset(&kinds) { return Err(SettlementQualificationError::MissingEvidenceKind); }
        let sources: BTreeSet<ReferenceId> = operation.evidence.iter().map(|evidence| evidence.source.clone()).collect();
        if sources.len() < usize::from(profile.min_distinct_sources) { return Err(SettlementQualificationError::InsufficientDistinctSources); }

        match operation.state {
            ObservedSettlementState::Pending | ObservedSettlementState::Unknown => return Err(SettlementQualificationError::OutstandingUncertainty),
            ObservedSettlementState::Rejected | ObservedSettlementState::Reversed => continue,
            ObservedSettlementState::Applied => {}
        }

        settled = settled.checked_add(&operation.amount)?;
        let evidence_ids = operation.evidence.iter().map(|evidence| evidence.evidence_id.clone()).collect();
        qualified_operations.insert(operation_id, QualifiedOperation::new(operation.revision, operation.amount, evidence_ids));
    }

    if settled.atomic_units() < subject.amount.atomic_units() { return Err(SettlementQualificationError::PartialSettlement); }
    if settled.atomic_units() > subject.amount.atomic_units() { return Err(SettlementQualificationError::OverSettlement); }

    Ok(QualifiedSettlement::new(
        subject.id.clone(), subject.attempt.clone(), subject.rail.clone(), subject.network.clone(),
        subject.amount.clone(), profile.profile_ref.clone(), evidence_frontier, qualified_operations,
        qualified_at_unix_ms, profile.reversal_model,
    ))
}

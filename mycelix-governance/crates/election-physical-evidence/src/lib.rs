//! Physical election evidence contracts for Mycelix public elections.
//!
//! This crate defines paper-ballot accounting conservation, exact custody lineage,
//! scanner/CVR reconciliation, and audit evidence boundaries. It does not implement
//! a risk-limiting-audit algorithm or replace jurisdiction-specific election law.

use election_integrity_types::{Digest32, PUBLIC_ELECTION_PROFILE_ID};
use serde::{Deserialize, Serialize};

pub const PHYSICAL_EVIDENCE_PROFILE_ID: &str = "mycelix-public-election-physical-evidence-v1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BallotAccountingScopeV1 {
    pub election_constitution_digest: Digest32,
    pub jurisdiction_scope_digest: Digest32,
    pub location_or_batch_scope_digest: Digest32,
    pub ballot_style_digest: Digest32,
    pub physical_unit_definition_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccountingScopeViolation {
    ZeroElectionConstitutionDigest,
    ZeroJurisdictionScopeDigest,
    ZeroLocationOrBatchScopeDigest,
    ZeroBallotStyleDigest,
    ZeroPhysicalUnitDefinitionDigest,
}

pub fn validate_accounting_scope(
    scope: &BallotAccountingScopeV1,
) -> Result<(), AccountingScopeViolation> {
    let zero = [0_u8; 32];
    if scope.election_constitution_digest == zero {
        return Err(AccountingScopeViolation::ZeroElectionConstitutionDigest);
    }
    if scope.jurisdiction_scope_digest == zero {
        return Err(AccountingScopeViolation::ZeroJurisdictionScopeDigest);
    }
    if scope.location_or_batch_scope_digest == zero {
        return Err(AccountingScopeViolation::ZeroLocationOrBatchScopeDigest);
    }
    if scope.ballot_style_digest == zero {
        return Err(AccountingScopeViolation::ZeroBallotStyleDigest);
    }
    if scope.physical_unit_definition_digest == zero {
        return Err(AccountingScopeViolation::ZeroPhysicalUnitDefinitionDigest);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BallotStockAccountingV1 {
    pub public_election_profile_id: String,
    pub physical_evidence_profile_id: String,
    pub scope: BallotAccountingScopeV1,
    pub opening_stock: u64,
    pub supplemental_stock_received: u64,
    pub cast_regular: u64,
    pub cast_provisional_sealed: u64,
    pub spoiled: u64,
    pub issued_not_cast: u64,
    pub unused: u64,
    pub quarantined: u64,
    pub exception_evidence_digest: Option<Digest32>,
    pub accounting_evidence_digest: Digest32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BallotAccountingViolation {
    WrongPublicElectionProfile,
    WrongPhysicalEvidenceProfile,
    Scope(AccountingScopeViolation),
    CountOverflow,
    ConservationFailure { available: u64, dispositioned: u64 },
    MissingExceptionEvidence,
    ZeroExceptionEvidenceDigest,
    ZeroAccountingEvidenceDigest,
}

fn checked_sum(values: &[u64]) -> Option<u64> {
    values
        .iter()
        .try_fold(0_u64, |sum, value| sum.checked_add(*value))
}

pub fn validate_ballot_stock_accounting(
    accounting: &BallotStockAccountingV1,
) -> Result<(), BallotAccountingViolation> {
    if accounting.public_election_profile_id != PUBLIC_ELECTION_PROFILE_ID {
        return Err(BallotAccountingViolation::WrongPublicElectionProfile);
    }
    if accounting.physical_evidence_profile_id != PHYSICAL_EVIDENCE_PROFILE_ID {
        return Err(BallotAccountingViolation::WrongPhysicalEvidenceProfile);
    }
    validate_accounting_scope(&accounting.scope).map_err(BallotAccountingViolation::Scope)?;

    let available = checked_sum(&[
        accounting.opening_stock,
        accounting.supplemental_stock_received,
    ])
    .ok_or(BallotAccountingViolation::CountOverflow)?;
    let dispositioned = checked_sum(&[
        accounting.cast_regular,
        accounting.cast_provisional_sealed,
        accounting.spoiled,
        accounting.issued_not_cast,
        accounting.unused,
        accounting.quarantined,
    ])
    .ok_or(BallotAccountingViolation::CountOverflow)?;

    if available != dispositioned {
        return Err(BallotAccountingViolation::ConservationFailure {
            available,
            dispositioned,
        });
    }

    let exceptional_count = checked_sum(&[
        accounting.cast_provisional_sealed,
        accounting.issued_not_cast,
        accounting.quarantined,
    ])
    .ok_or(BallotAccountingViolation::CountOverflow)?;
    if exceptional_count > 0 && accounting.exception_evidence_digest.is_none() {
        return Err(BallotAccountingViolation::MissingExceptionEvidence);
    }
    if accounting.exception_evidence_digest == Some([0_u8; 32]) {
        return Err(BallotAccountingViolation::ZeroExceptionEvidenceDigest);
    }
    if accounting.accounting_evidence_digest == [0_u8; 32] {
        return Err(BallotAccountingViolation::ZeroAccountingEvidenceDigest);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PhysicalBallotBatchV1 {
    pub election_constitution_digest: Digest32,
    pub batch_digest: Digest32,
    pub accounting_scope_digest: Digest32,
    pub ballot_style_digest: Digest32,
    pub physical_unit_definition_digest: Digest32,
    pub physical_unit_count: u64,
    pub initial_container_state_digest: Digest32,
    pub batch_manifest_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BallotBatchViolation {
    ZeroElectionConstitutionDigest,
    ZeroBatchDigest,
    ZeroAccountingScopeDigest,
    ZeroBallotStyleDigest,
    ZeroPhysicalUnitDefinitionDigest,
    EmptyBatch,
    ZeroInitialContainerStateDigest,
    ZeroBatchManifestDigest,
}

pub fn validate_ballot_batch(batch: &PhysicalBallotBatchV1) -> Result<(), BallotBatchViolation> {
    let zero = [0_u8; 32];
    if batch.election_constitution_digest == zero {
        return Err(BallotBatchViolation::ZeroElectionConstitutionDigest);
    }
    if batch.batch_digest == zero {
        return Err(BallotBatchViolation::ZeroBatchDigest);
    }
    if batch.accounting_scope_digest == zero {
        return Err(BallotBatchViolation::ZeroAccountingScopeDigest);
    }
    if batch.ballot_style_digest == zero {
        return Err(BallotBatchViolation::ZeroBallotStyleDigest);
    }
    if batch.physical_unit_definition_digest == zero {
        return Err(BallotBatchViolation::ZeroPhysicalUnitDefinitionDigest);
    }
    if batch.physical_unit_count == 0 {
        return Err(BallotBatchViolation::EmptyBatch);
    }
    if batch.initial_container_state_digest == zero {
        return Err(BallotBatchViolation::ZeroInitialContainerStateDigest);
    }
    if batch.batch_manifest_digest == zero {
        return Err(BallotBatchViolation::ZeroBatchManifestDigest);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SealStateV1 {
    Sealed {
        seal_identifier_digest: Digest32,
        seal_application_evidence_digest: Digest32,
    },
    OpenForAuthorizedProcess {
        authorization_digest: Digest32,
        opening_evidence_digest: Digest32,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ContainerStateV1 {
    pub container_identifier_digest: Digest32,
    pub seal_state: SealStateV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CustodyEventKind {
    StockTransfer,
    PollsCloseTransfer,
    SecureStorageTransfer,
    AuthorizedOpening,
    Reseal,
    AuditRetrieval,
    AuditReturn,
    RecountTransfer,
    StorageInspection,
    OtherGovernedProcess,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CustodyEventV1 {
    pub election_constitution_digest: Digest32,
    pub ballot_batch_digest: Digest32,
    pub custody_sequence: u64,
    pub previous_event_digest: Option<Digest32>,
    pub event_kind: CustodyEventKind,
    pub from_control_domain_digest: Digest32,
    pub to_control_domain_digest: Digest32,
    pub before_state: ContainerStateV1,
    pub after_state: ContainerStateV1,
    pub governing_authorization_digest: Digest32,
    pub witness_evidence_digest: Digest32,
    pub temporal_evidence_digest: Digest32,
    pub event_evidence_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CustodyEventViolation {
    ZeroElectionConstitutionDigest,
    ZeroBallotBatchDigest,
    GenesisHasPredecessor,
    SuccessorMissingPredecessor,
    ZeroPredecessorDigest,
    ZeroFromControlDomainDigest,
    ZeroToControlDomainDigest,
    ZeroContainerDigest,
    ZeroSealOrAuthorizationDigest,
    ZeroSealEvidenceDigest,
    ZeroGoverningAuthorizationDigest,
    ZeroWitnessEvidenceDigest,
    ZeroTemporalEvidenceDigest,
    ZeroEventEvidenceDigest,
}

fn validate_container_state(state: &ContainerStateV1) -> Result<(), CustodyEventViolation> {
    let zero = [0_u8; 32];
    if state.container_identifier_digest == zero {
        return Err(CustodyEventViolation::ZeroContainerDigest);
    }
    match &state.seal_state {
        SealStateV1::Sealed {
            seal_identifier_digest,
            seal_application_evidence_digest,
        } => {
            if *seal_identifier_digest == zero {
                return Err(CustodyEventViolation::ZeroSealOrAuthorizationDigest);
            }
            if *seal_application_evidence_digest == zero {
                return Err(CustodyEventViolation::ZeroSealEvidenceDigest);
            }
        }
        SealStateV1::OpenForAuthorizedProcess {
            authorization_digest,
            opening_evidence_digest,
        } => {
            if *authorization_digest == zero {
                return Err(CustodyEventViolation::ZeroSealOrAuthorizationDigest);
            }
            if *opening_evidence_digest == zero {
                return Err(CustodyEventViolation::ZeroSealEvidenceDigest);
            }
        }
    }
    Ok(())
}

pub fn validate_custody_event_structure(
    event: &CustodyEventV1,
) -> Result<(), CustodyEventViolation> {
    let zero = [0_u8; 32];
    if event.election_constitution_digest == zero {
        return Err(CustodyEventViolation::ZeroElectionConstitutionDigest);
    }
    if event.ballot_batch_digest == zero {
        return Err(CustodyEventViolation::ZeroBallotBatchDigest);
    }
    if event.custody_sequence == 0 {
        if event.previous_event_digest.is_some() {
            return Err(CustodyEventViolation::GenesisHasPredecessor);
        }
    } else {
        let previous = event
            .previous_event_digest
            .ok_or(CustodyEventViolation::SuccessorMissingPredecessor)?;
        if previous == zero {
            return Err(CustodyEventViolation::ZeroPredecessorDigest);
        }
    }
    if event.from_control_domain_digest == zero {
        return Err(CustodyEventViolation::ZeroFromControlDomainDigest);
    }
    if event.to_control_domain_digest == zero {
        return Err(CustodyEventViolation::ZeroToControlDomainDigest);
    }
    validate_container_state(&event.before_state)?;
    validate_container_state(&event.after_state)?;
    if event.governing_authorization_digest == zero {
        return Err(CustodyEventViolation::ZeroGoverningAuthorizationDigest);
    }
    if event.witness_evidence_digest == zero {
        return Err(CustodyEventViolation::ZeroWitnessEvidenceDigest);
    }
    if event.temporal_evidence_digest == zero {
        return Err(CustodyEventViolation::ZeroTemporalEvidenceDigest);
    }
    if event.event_evidence_digest == zero {
        return Err(CustodyEventViolation::ZeroEventEvidenceDigest);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CustodySuccessorViolation {
    Previous(CustodyEventViolation),
    Next(CustodyEventViolation),
    WrongElectionConstitution,
    WrongBallotBatch,
    SequenceNotContiguous,
    PredecessorDigestMismatch,
    ContainerStateDiscontinuity,
}

pub fn validate_custody_successor(
    previous: &CustodyEventV1,
    previous_event_digest: Digest32,
    next: &CustodyEventV1,
) -> Result<(), CustodySuccessorViolation> {
    validate_custody_event_structure(previous).map_err(CustodySuccessorViolation::Previous)?;
    validate_custody_event_structure(next).map_err(CustodySuccessorViolation::Next)?;
    if previous.election_constitution_digest != next.election_constitution_digest {
        return Err(CustodySuccessorViolation::WrongElectionConstitution);
    }
    if previous.ballot_batch_digest != next.ballot_batch_digest {
        return Err(CustodySuccessorViolation::WrongBallotBatch);
    }
    if next.custody_sequence != previous.custody_sequence + 1 {
        return Err(CustodySuccessorViolation::SequenceNotContiguous);
    }
    if next.previous_event_digest != Some(previous_event_digest) {
        return Err(CustodySuccessorViolation::PredecessorDigestMismatch);
    }
    if next.before_state != previous.after_state {
        return Err(CustodySuccessorViolation::ContainerStateDiscontinuity);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CustodyEventRefV1 {
    pub custody_sequence: u64,
    pub event_digest: Digest32,
    pub previous_event_digest: Option<Digest32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CustodyPairClassification {
    SameEvent,
    DifferentSequence,
    SiblingFork,
    DivergentHistory,
}

pub fn classify_custody_pair(
    first: &CustodyEventRefV1,
    second: &CustodyEventRefV1,
) -> CustodyPairClassification {
    if first == second {
        return CustodyPairClassification::SameEvent;
    }
    if first.custody_sequence != second.custody_sequence {
        return CustodyPairClassification::DifferentSequence;
    }
    if first.previous_event_digest == second.previous_event_digest {
        return CustodyPairClassification::SiblingFork;
    }
    CustodyPairClassification::DivergentHistory
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum CustodyConflictPolicyV1 {
    #[default]
    FreezePendingEvidenceResolution,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CustodyChainSummaryV1 {
    pub ballot_batch_digest: Digest32,
    pub genesis_event_digest: Digest32,
    pub final_event_digest: Digest32,
    pub event_count: u64,
    pub conflicting_event_count: u64,
    pub final_container_state_digest: Digest32,
    pub complete_checkpoint_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CustodyChainSummaryViolation {
    ZeroBallotBatchDigest,
    ZeroGenesisEventDigest,
    ZeroFinalEventDigest,
    EmptyChain,
    ConflictingEventsPresent,
    ZeroFinalContainerStateDigest,
    ZeroCompleteCheckpointDigest,
}

pub fn validate_custody_chain_summary(
    summary: &CustodyChainSummaryV1,
) -> Result<(), CustodyChainSummaryViolation> {
    let zero = [0_u8; 32];
    if summary.ballot_batch_digest == zero {
        return Err(CustodyChainSummaryViolation::ZeroBallotBatchDigest);
    }
    if summary.genesis_event_digest == zero {
        return Err(CustodyChainSummaryViolation::ZeroGenesisEventDigest);
    }
    if summary.final_event_digest == zero {
        return Err(CustodyChainSummaryViolation::ZeroFinalEventDigest);
    }
    if summary.event_count == 0 {
        return Err(CustodyChainSummaryViolation::EmptyChain);
    }
    if summary.conflicting_event_count != 0 {
        return Err(CustodyChainSummaryViolation::ConflictingEventsPresent);
    }
    if summary.final_container_state_digest == zero {
        return Err(CustodyChainSummaryViolation::ZeroFinalContainerStateDigest);
    }
    if summary.complete_checkpoint_digest == zero {
        return Err(CustodyChainSummaryViolation::ZeroCompleteCheckpointDigest);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CaptureReconciliationV1 {
    pub election_constitution_digest: Digest32,
    pub accounting_scope_digest: Digest32,
    pub cardinality_profile_digest: Digest32,
    pub physical_cast_unit_count: u64,
    pub expected_unique_cvr_count: u64,
    pub observed_unique_cvr_count: u64,
    pub duplicate_cvr_count: u64,
    pub unresolved_mapping_count: u64,
    pub scanner_manifest_digest: Digest32,
    pub cvr_export_digest: Digest32,
    pub cvr_interoperability_profile_digest: Digest32,
    pub reconciliation_evidence_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CaptureReconciliationDisposition {
    Clean,
    Discrepant,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CaptureReconciliationViolation {
    ZeroElectionConstitutionDigest,
    ZeroAccountingScopeDigest,
    ZeroCardinalityProfileDigest,
    EmptyPhysicalCastPopulation,
    EmptyExpectedCvrPopulation,
    ZeroScannerManifestDigest,
    ZeroCvrExportDigest,
    ZeroCvrInteroperabilityProfileDigest,
    ZeroReconciliationEvidenceDigest,
}

pub fn classify_capture_reconciliation(
    reconciliation: &CaptureReconciliationV1,
) -> Result<CaptureReconciliationDisposition, CaptureReconciliationViolation> {
    let zero = [0_u8; 32];
    if reconciliation.election_constitution_digest == zero {
        return Err(CaptureReconciliationViolation::ZeroElectionConstitutionDigest);
    }
    if reconciliation.accounting_scope_digest == zero {
        return Err(CaptureReconciliationViolation::ZeroAccountingScopeDigest);
    }
    if reconciliation.cardinality_profile_digest == zero {
        return Err(CaptureReconciliationViolation::ZeroCardinalityProfileDigest);
    }
    if reconciliation.physical_cast_unit_count == 0 {
        return Err(CaptureReconciliationViolation::EmptyPhysicalCastPopulation);
    }
    if reconciliation.expected_unique_cvr_count == 0 {
        return Err(CaptureReconciliationViolation::EmptyExpectedCvrPopulation);
    }
    if reconciliation.scanner_manifest_digest == zero {
        return Err(CaptureReconciliationViolation::ZeroScannerManifestDigest);
    }
    if reconciliation.cvr_export_digest == zero {
        return Err(CaptureReconciliationViolation::ZeroCvrExportDigest);
    }
    if reconciliation.cvr_interoperability_profile_digest == zero {
        return Err(CaptureReconciliationViolation::ZeroCvrInteroperabilityProfileDigest);
    }
    if reconciliation.reconciliation_evidence_digest == zero {
        return Err(CaptureReconciliationViolation::ZeroReconciliationEvidenceDigest);
    }

    if reconciliation.expected_unique_cvr_count == reconciliation.observed_unique_cvr_count
        && reconciliation.duplicate_cvr_count == 0
        && reconciliation.unresolved_mapping_count == 0
    {
        Ok(CaptureReconciliationDisposition::Clean)
    } else {
        Ok(CaptureReconciliationDisposition::Discrepant)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuditMethodClass {
    RiskLimiting,
    TraditionalStatutory,
    FullHandCount,
    OtherGovernedMethod,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuditMethodPolicyV1 {
    pub audit_method_class: AuditMethodClass,
    pub audit_policy_digest: Digest32,
    pub method_profile_digest: Digest32,
    pub method_parameters_digest: Digest32,
    pub risk_limit_parts_per_million: Option<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AuditMethodPolicyViolation {
    ZeroAuditPolicyDigest,
    ZeroMethodProfileDigest,
    ZeroMethodParametersDigest,
    RiskLimitingAuditMissingRiskLimit,
    InvalidRiskLimit,
    NonRiskLimitingAuditHasRiskLimit,
}

pub fn validate_audit_method_policy(
    policy: &AuditMethodPolicyV1,
) -> Result<(), AuditMethodPolicyViolation> {
    let zero = [0_u8; 32];
    if policy.audit_policy_digest == zero {
        return Err(AuditMethodPolicyViolation::ZeroAuditPolicyDigest);
    }
    if policy.method_profile_digest == zero {
        return Err(AuditMethodPolicyViolation::ZeroMethodProfileDigest);
    }
    if policy.method_parameters_digest == zero {
        return Err(AuditMethodPolicyViolation::ZeroMethodParametersDigest);
    }
    match (
        policy.audit_method_class,
        policy.risk_limit_parts_per_million,
    ) {
        (AuditMethodClass::RiskLimiting, None) => {
            Err(AuditMethodPolicyViolation::RiskLimitingAuditMissingRiskLimit)
        }
        (AuditMethodClass::RiskLimiting, Some(risk_limit))
            if risk_limit == 0 || risk_limit >= 1_000_000 =>
        {
            Err(AuditMethodPolicyViolation::InvalidRiskLimit)
        }
        (AuditMethodClass::RiskLimiting, Some(_)) => Ok(()),
        (_, Some(_)) => Err(AuditMethodPolicyViolation::NonRiskLimitingAuditHasRiskLimit),
        (_, None) => Ok(()),
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuditSampleV1 {
    pub population_checkpoint_digest: Digest32,
    pub ballot_manifest_digest: Digest32,
    pub audit_policy_digest: Digest32,
    pub sample_selection_evidence_digest: Digest32,
    pub sampled_positions_digest: Digest32,
    pub population_size: u64,
    pub sample_size: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AuditSampleViolation {
    ZeroPopulationCheckpointDigest,
    ZeroBallotManifestDigest,
    ZeroAuditPolicyDigest,
    ZeroSampleSelectionEvidenceDigest,
    ZeroSampledPositionsDigest,
    EmptyPopulation,
    EmptySample,
    SampleExceedsPopulation,
}

pub fn validate_audit_sample(sample: &AuditSampleV1) -> Result<(), AuditSampleViolation> {
    let zero = [0_u8; 32];
    if sample.population_checkpoint_digest == zero {
        return Err(AuditSampleViolation::ZeroPopulationCheckpointDigest);
    }
    if sample.ballot_manifest_digest == zero {
        return Err(AuditSampleViolation::ZeroBallotManifestDigest);
    }
    if sample.audit_policy_digest == zero {
        return Err(AuditSampleViolation::ZeroAuditPolicyDigest);
    }
    if sample.sample_selection_evidence_digest == zero {
        return Err(AuditSampleViolation::ZeroSampleSelectionEvidenceDigest);
    }
    if sample.sampled_positions_digest == zero {
        return Err(AuditSampleViolation::ZeroSampledPositionsDigest);
    }
    if sample.population_size == 0 {
        return Err(AuditSampleViolation::EmptyPopulation);
    }
    if sample.sample_size == 0 {
        return Err(AuditSampleViolation::EmptySample);
    }
    if sample.sample_size > sample.population_size {
        return Err(AuditSampleViolation::SampleExceedsPopulation);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuditObservationClass {
    Match,
    VoteOverstatement,
    VoteUnderstatement,
    MissingPhysicalRecord,
    MissingCvr,
    UninterpretablePhysicalRecord,
    GovernedAdjudicationRequired,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuditObservationRefV1 {
    pub sample_ordinal: u64,
    pub anonymized_physical_unit_digest: Digest32,
    pub cvr_digest: Option<Digest32>,
    pub observation_class: AuditObservationClass,
    pub observation_evidence_digest: Digest32,
    pub adjudication_evidence_digest: Option<Digest32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AuditObservationViolation {
    ZeroPhysicalUnitDigest,
    ZeroCvrDigest,
    ZeroObservationEvidenceDigest,
    MissingAdjudicationEvidence,
    ZeroAdjudicationEvidenceDigest,
}

pub fn validate_audit_observation(
    observation: &AuditObservationRefV1,
) -> Result<(), AuditObservationViolation> {
    let zero = [0_u8; 32];
    if observation.anonymized_physical_unit_digest == zero {
        return Err(AuditObservationViolation::ZeroPhysicalUnitDigest);
    }
    if observation.cvr_digest == Some(zero) {
        return Err(AuditObservationViolation::ZeroCvrDigest);
    }
    if observation.observation_evidence_digest == zero {
        return Err(AuditObservationViolation::ZeroObservationEvidenceDigest);
    }
    if observation.observation_class == AuditObservationClass::GovernedAdjudicationRequired
        && observation.adjudication_evidence_digest.is_none()
    {
        return Err(AuditObservationViolation::MissingAdjudicationEvidence);
    }
    if observation.adjudication_evidence_digest == Some(zero) {
        return Err(AuditObservationViolation::ZeroAdjudicationEvidenceDigest);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuditDisposition {
    OutcomeConfirmed,
    EscalateSample,
    FullHandCountRequired,
    Indeterminate,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuditResultRefV1 {
    pub audit_policy_digest: Digest32,
    pub method_profile_digest: Digest32,
    pub population_checkpoint_digest: Digest32,
    pub sample_digest: Digest32,
    pub observations_root_digest: Digest32,
    pub audit_computation_evidence_digest: Digest32,
    pub disposition: AuditDisposition,
    pub result_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AuditResultViolation {
    ZeroAuditPolicyDigest,
    ZeroMethodProfileDigest,
    ZeroPopulationCheckpointDigest,
    ZeroSampleDigest,
    ZeroObservationsRootDigest,
    ZeroAuditComputationEvidenceDigest,
    ZeroResultDigest,
}

pub fn validate_audit_result(result: &AuditResultRefV1) -> Result<(), AuditResultViolation> {
    let zero = [0_u8; 32];
    if result.audit_policy_digest == zero {
        return Err(AuditResultViolation::ZeroAuditPolicyDigest);
    }
    if result.method_profile_digest == zero {
        return Err(AuditResultViolation::ZeroMethodProfileDigest);
    }
    if result.population_checkpoint_digest == zero {
        return Err(AuditResultViolation::ZeroPopulationCheckpointDigest);
    }
    if result.sample_digest == zero {
        return Err(AuditResultViolation::ZeroSampleDigest);
    }
    if result.observations_root_digest == zero {
        return Err(AuditResultViolation::ZeroObservationsRootDigest);
    }
    if result.audit_computation_evidence_digest == zero {
        return Err(AuditResultViolation::ZeroAuditComputationEvidenceDigest);
    }
    if result.result_digest == zero {
        return Err(AuditResultViolation::ZeroResultDigest);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PhysicalEvidenceBundleRefV1 {
    pub election_constitution_digest: Digest32,
    pub accounting_receipt_digest: Digest32,
    pub custody_summary_digest: Digest32,
    pub capture_reconciliation_digest: Digest32,
    pub audit_result_digest: Digest32,
    pub physical_evidence_root_digest: Digest32,
    pub transparency_checkpoint_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhysicalEvidenceBundleViolation {
    ZeroElectionConstitutionDigest,
    ZeroAccountingReceiptDigest,
    ZeroCustodySummaryDigest,
    ZeroCaptureReconciliationDigest,
    ZeroAuditResultDigest,
    ZeroPhysicalEvidenceRootDigest,
    ZeroTransparencyCheckpointDigest,
}

pub fn validate_physical_evidence_bundle_ref(
    bundle: &PhysicalEvidenceBundleRefV1,
) -> Result<(), PhysicalEvidenceBundleViolation> {
    let zero = [0_u8; 32];
    if bundle.election_constitution_digest == zero {
        return Err(PhysicalEvidenceBundleViolation::ZeroElectionConstitutionDigest);
    }
    if bundle.accounting_receipt_digest == zero {
        return Err(PhysicalEvidenceBundleViolation::ZeroAccountingReceiptDigest);
    }
    if bundle.custody_summary_digest == zero {
        return Err(PhysicalEvidenceBundleViolation::ZeroCustodySummaryDigest);
    }
    if bundle.capture_reconciliation_digest == zero {
        return Err(PhysicalEvidenceBundleViolation::ZeroCaptureReconciliationDigest);
    }
    if bundle.audit_result_digest == zero {
        return Err(PhysicalEvidenceBundleViolation::ZeroAuditResultDigest);
    }
    if bundle.physical_evidence_root_digest == zero {
        return Err(PhysicalEvidenceBundleViolation::ZeroPhysicalEvidenceRootDigest);
    }
    if bundle.transparency_checkpoint_digest == zero {
        return Err(PhysicalEvidenceBundleViolation::ZeroTransparencyCheckpointDigest);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn accounting_scope() -> BallotAccountingScopeV1 {
        BallotAccountingScopeV1 {
            election_constitution_digest: digest(1),
            jurisdiction_scope_digest: digest(2),
            location_or_batch_scope_digest: digest(3),
            ballot_style_digest: digest(4),
            physical_unit_definition_digest: digest(5),
        }
    }

    #[test]
    fn ballot_accounting_obeys_exact_conservation() {
        let accounting = BallotStockAccountingV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            physical_evidence_profile_id: PHYSICAL_EVIDENCE_PROFILE_ID.to_owned(),
            scope: accounting_scope(),
            opening_stock: 1_000,
            supplemental_stock_received: 50,
            cast_regular: 800,
            cast_provisional_sealed: 20,
            spoiled: 30,
            issued_not_cast: 10,
            unused: 180,
            quarantined: 10,
            exception_evidence_digest: Some(digest(6)),
            accounting_evidence_digest: digest(7),
        };
        assert_eq!(validate_ballot_stock_accounting(&accounting), Ok(()));
    }

    #[test]
    fn silent_ballot_loss_fails_conservation() {
        let accounting = BallotStockAccountingV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            physical_evidence_profile_id: PHYSICAL_EVIDENCE_PROFILE_ID.to_owned(),
            scope: accounting_scope(),
            opening_stock: 100,
            supplemental_stock_received: 0,
            cast_regular: 80,
            cast_provisional_sealed: 0,
            spoiled: 5,
            issued_not_cast: 0,
            unused: 14,
            quarantined: 0,
            exception_evidence_digest: None,
            accounting_evidence_digest: digest(7),
        };
        assert_eq!(
            validate_ballot_stock_accounting(&accounting),
            Err(BallotAccountingViolation::ConservationFailure {
                available: 100,
                dispositioned: 99,
            })
        );
    }

    fn sealed(container: u8, seal: u8) -> ContainerStateV1 {
        ContainerStateV1 {
            container_identifier_digest: digest(container),
            seal_state: SealStateV1::Sealed {
                seal_identifier_digest: digest(seal),
                seal_application_evidence_digest: digest(seal + 20),
            },
        }
    }

    fn custody_event(sequence: u64, previous: Option<Digest32>) -> CustodyEventV1 {
        CustodyEventV1 {
            election_constitution_digest: digest(1),
            ballot_batch_digest: digest(10),
            custody_sequence: sequence,
            previous_event_digest: previous,
            event_kind: CustodyEventKind::SecureStorageTransfer,
            from_control_domain_digest: digest(11),
            to_control_domain_digest: digest(12),
            before_state: sealed(13, 14),
            after_state: sealed(13, 15),
            governing_authorization_digest: digest(16),
            witness_evidence_digest: digest(17),
            temporal_evidence_digest: digest(18),
            event_evidence_digest: digest(19),
        }
    }

    #[test]
    fn custody_successor_requires_exact_container_state_continuity() {
        let previous = custody_event(0, None);
        let mut next = custody_event(1, Some(digest(30)));
        next.before_state = previous.after_state.clone();
        assert_eq!(
            validate_custody_successor(&previous, digest(30), &next),
            Ok(())
        );
    }

    #[test]
    fn custody_gap_fails_closed() {
        let previous = custody_event(0, None);
        let mut next = custody_event(2, Some(digest(30)));
        next.before_state = previous.after_state.clone();
        assert_eq!(
            validate_custody_successor(&previous, digest(30), &next),
            Err(CustodySuccessorViolation::SequenceNotContiguous)
        );
    }

    #[test]
    fn sibling_custody_events_are_conflicts_not_first_arrival_winners() {
        let first = CustodyEventRefV1 {
            custody_sequence: 5,
            event_digest: digest(40),
            previous_event_digest: Some(digest(39)),
        };
        let second = CustodyEventRefV1 {
            custody_sequence: 5,
            event_digest: digest(41),
            previous_event_digest: Some(digest(39)),
        };
        assert_eq!(
            classify_custody_pair(&first, &second),
            CustodyPairClassification::SiblingFork
        );
        assert_eq!(
            CustodyConflictPolicyV1::default(),
            CustodyConflictPolicyV1::FreezePendingEvidenceResolution
        );
    }

    #[test]
    fn cvr_reconciliation_detects_missing_or_duplicate_records() {
        let clean = CaptureReconciliationV1 {
            election_constitution_digest: digest(1),
            accounting_scope_digest: digest(2),
            cardinality_profile_digest: digest(3),
            physical_cast_unit_count: 100,
            expected_unique_cvr_count: 100,
            observed_unique_cvr_count: 100,
            duplicate_cvr_count: 0,
            unresolved_mapping_count: 0,
            scanner_manifest_digest: digest(4),
            cvr_export_digest: digest(5),
            cvr_interoperability_profile_digest: digest(6),
            reconciliation_evidence_digest: digest(7),
        };
        assert_eq!(
            classify_capture_reconciliation(&clean),
            Ok(CaptureReconciliationDisposition::Clean)
        );

        let mut broken = clean;
        broken.observed_unique_cvr_count = 99;
        broken.duplicate_cvr_count = 1;
        assert_eq!(
            classify_capture_reconciliation(&broken),
            Ok(CaptureReconciliationDisposition::Discrepant)
        );
    }

    #[test]
    fn risk_limiting_audit_requires_explicit_nontrivial_risk_limit() {
        let policy = AuditMethodPolicyV1 {
            audit_method_class: AuditMethodClass::RiskLimiting,
            audit_policy_digest: digest(1),
            method_profile_digest: digest(2),
            method_parameters_digest: digest(3),
            risk_limit_parts_per_million: Some(50_000),
        };
        assert_eq!(validate_audit_method_policy(&policy), Ok(()));

        let mut broken = policy;
        broken.risk_limit_parts_per_million = Some(0);
        assert_eq!(
            validate_audit_method_policy(&broken),
            Err(AuditMethodPolicyViolation::InvalidRiskLimit)
        );
    }

    #[test]
    fn audit_sample_cannot_exceed_frozen_population() {
        let sample = AuditSampleV1 {
            population_checkpoint_digest: digest(1),
            ballot_manifest_digest: digest(2),
            audit_policy_digest: digest(3),
            sample_selection_evidence_digest: digest(4),
            sampled_positions_digest: digest(5),
            population_size: 100,
            sample_size: 101,
        };
        assert_eq!(
            validate_audit_sample(&sample),
            Err(AuditSampleViolation::SampleExceedsPopulation)
        );
    }

    #[test]
    fn governed_adjudication_requires_evidence() {
        let observation = AuditObservationRefV1 {
            sample_ordinal: 0,
            anonymized_physical_unit_digest: digest(1),
            cvr_digest: Some(digest(2)),
            observation_class: AuditObservationClass::GovernedAdjudicationRequired,
            observation_evidence_digest: digest(3),
            adjudication_evidence_digest: None,
        };
        assert_eq!(
            validate_audit_observation(&observation),
            Err(AuditObservationViolation::MissingAdjudicationEvidence)
        );
    }
}

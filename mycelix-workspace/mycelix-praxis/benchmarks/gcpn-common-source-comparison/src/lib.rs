//! Common-source, paired forecast-comparison contract for GCPN.
//!
//! This crate validates comparability and accounts for availability. It does
//! not fetch or redistribute data, calculate forecast scores, or issue a legal
//! opinion about dataset licensing. Unknown redistribution terms are never
//! interpreted as permission.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum RedistributionPolicy {
    AllowedByDeclaredTerms,
    RestrictedByDeclaredTerms,
    Unknown,
}

impl RedistributionPolicy {
    pub const fn label(self) -> &'static str {
        match self {
            Self::AllowedByDeclaredTerms => "allowed_by_declared_terms",
            Self::RestrictedByDeclaredTerms => "restricted_by_declared_terms",
            Self::Unknown => "unknown",
        }
    }

    /// Reflects only the declared manifest value; it is not a legal conclusion.
    pub const fn permits_redistribution_by_declaration(self) -> bool {
        matches!(self, Self::AllowedByDeclaredTerms)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DatasetReleaseManifest {
    pub dataset_id: String,
    pub release_id: String,
    pub source_locator: String,
    pub citation: String,
    pub retrieved_at: u64,
    pub artifact_sha256: String,
    pub license_id: String,
    pub terms_url: String,
    pub redistribution_policy: RedistributionPolicy,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SpatialSupportContract {
    pub support_id: String,
    pub grid_sha256: String,
    pub mask_sha256: String,
    pub coordinate_convention: String,
    pub area_weight_scheme: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerificationContract {
    pub source_id: String,
    pub source_version: String,
    pub release_id: String,
    pub artifact_sha256: String,
    pub observation_lineage_id: String,
    pub processing_epoch: String,
    pub as_of_cutoff: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComparisonContract {
    pub comparison_id: String,
    pub predictand_id: String,
    pub unit: String,
    pub temporal_support_id: String,
    pub spatial_support_id: String,
    pub climatology_id: String,
    pub evaluation_start: u64,
    pub evaluation_end: u64,
    pub as_of_cutoff: u64,
    pub intrinsic_skill_claim: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProductKind {
    Deterministic,
    Ensemble,
}

impl ProductKind {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Deterministic => "deterministic",
            Self::Ensemble => "ensemble",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PostProcessingProvenance {
    pub operation_id: String,
    pub operation_version: String,
    pub fit_data_sha256: String,
    pub trained_through: u64,
    pub uses_evaluation_outcomes: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ModelOutputManifest {
    pub model_id: String,
    pub model_version: String,
    pub archive_release: String,
    pub artifact_sha256: String,
    pub predictand_id: String,
    pub unit: String,
    pub temporal_support_id: String,
    pub spatial_support_id: String,
    pub grid_sha256: String,
    pub mask_sha256: String,
    pub climatology_id: String,
    pub product_kind: ProductKind,
    pub ensemble_members: Option<usize>,
    pub training_cutoff: Option<u64>,
    pub post_processing: Option<PostProcessingProvenance>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ForecastInputRequirement {
    DeterministicOnly,
    EnsembleOnly,
    AnyKind,
    EqualEnsembleSize,
    ExactEnsembleSize(usize),
    MinimumEnsembleSize(usize),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MetricManifest {
    pub metric_id: String,
    pub definition_version: String,
    pub scoring_engine: String,
    pub scoring_engine_version: String,
    pub configuration_sha256: String,
    pub input_requirement: ForecastInputRequirement,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum MissingReason {
    NotProduced,
    NotArchived,
    RetrievalFailure,
    QualityRejected,
    PendingAtCutoff,
    Unknown,
}

impl MissingReason {
    pub const fn label(self) -> &'static str {
        match self {
            Self::NotProduced => "not_produced",
            Self::NotArchived => "not_archived",
            Self::RetrievalFailure => "retrieval_failure",
            Self::QualityRejected => "quality_rejected",
            Self::PendingAtCutoff => "pending_at_cutoff",
            Self::Unknown => "unknown",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Availability {
    Available {
        artifact_sha256: String,
        recorded_at: u64,
    },
    Missing {
        reason: MissingReason,
    },
}

impl Availability {
    pub const fn is_available(&self) -> bool {
        matches!(self, Self::Available { .. })
    }

    pub const fn label(&self) -> &'static str {
        match self {
            Self::Available { .. } => "available",
            Self::Missing { reason } => reason.label(),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ForecastAvailability {
    Available {
        artifact_sha256: String,
        initialized_at: u64,
        recorded_at: u64,
    },
    Missing {
        reason: MissingReason,
    },
}

impl ForecastAvailability {
    pub const fn is_available(&self) -> bool {
        matches!(self, Self::Available { .. })
    }

    pub const fn label(&self) -> &'static str {
        match self {
            Self::Available { .. } => "available",
            Self::Missing { reason } => reason.label(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExclusionReason {
    OutsideEvaluationPeriod,
    ExcludedByFrozenProtocol,
    InvalidCase,
    DisputedCase,
}

impl ExclusionReason {
    pub const fn label(self) -> &'static str {
        match self {
            Self::OutsideEvaluationPeriod => "outside_evaluation_period",
            Self::ExcludedByFrozenProtocol => "excluded_by_frozen_protocol",
            Self::InvalidCase => "invalid_case",
            Self::DisputedCase => "disputed_case",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CaseEligibility {
    Eligible,
    Excluded(ExclusionReason),
}

impl CaseEligibility {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Eligible => "eligible",
            Self::Excluded(reason) => reason.label(),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComparisonCase {
    pub case_id: String,
    pub initialization_time: u64,
    pub valid_time: u64,
    pub lead_seconds: u64,
    pub eligibility: CaseEligibility,
    pub candidate: ForecastAvailability,
    pub reference: ForecastAvailability,
    pub verification: Availability,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComparisonManifest {
    pub case_frame_id: String,
    pub dataset_release: DatasetReleaseManifest,
    pub spatial_support: SpatialSupportContract,
    pub verification: VerificationContract,
    pub contract: ComparisonContract,
    pub candidate: ModelOutputManifest,
    pub reference: ModelOutputManifest,
    pub metric: MetricManifest,
    pub cases: Vec<ComparisonCase>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContractViolation {
    InvalidIdentifier,
    MissingText,
    InvalidSha256,
    InvalidEvaluationPeriod,
    VerificationCutoffMismatch,
    DatasetVerificationMismatch,
    DatasetRetrievedAfterCutoff,
    SpatialSupportMismatch,
    GridMismatch,
    MaskMismatch,
    PredictandMismatch,
    UnitMismatch,
    TemporalSupportMismatch,
    ClimatologyMismatch,
    InvalidEnsembleMetadata,
    MetricInputMismatch,
    MissingTrainingCutoff,
    TrainingOverlapsEvaluation,
    PostProcessingOutcomeLeakage,
    EmptyCaseFrame,
    DuplicateCaseKey,
    InvalidCaseTime,
    EligibleCaseOutsidePeriod,
    AvailabilityAfterCutoff,
    ForecastInitializationMismatch,
    InvalidCaseArtifactDigest,
}

impl ContractViolation {
    pub const fn label(self) -> &'static str {
        match self {
            Self::InvalidIdentifier => "invalid_identifier",
            Self::MissingText => "missing_text",
            Self::InvalidSha256 => "invalid_sha256",
            Self::InvalidEvaluationPeriod => "invalid_evaluation_period",
            Self::VerificationCutoffMismatch => "verification_cutoff_mismatch",
            Self::DatasetVerificationMismatch => "dataset_verification_mismatch",
            Self::DatasetRetrievedAfterCutoff => "dataset_retrieved_after_cutoff",
            Self::SpatialSupportMismatch => "spatial_support_mismatch",
            Self::GridMismatch => "grid_mismatch",
            Self::MaskMismatch => "mask_mismatch",
            Self::PredictandMismatch => "predictand_mismatch",
            Self::UnitMismatch => "unit_mismatch",
            Self::TemporalSupportMismatch => "temporal_support_mismatch",
            Self::ClimatologyMismatch => "climatology_mismatch",
            Self::InvalidEnsembleMetadata => "invalid_ensemble_metadata",
            Self::MetricInputMismatch => "metric_input_mismatch",
            Self::MissingTrainingCutoff => "missing_training_cutoff",
            Self::TrainingOverlapsEvaluation => "training_overlaps_evaluation",
            Self::PostProcessingOutcomeLeakage => "postprocessing_outcome_leakage",
            Self::EmptyCaseFrame => "empty_case_frame",
            Self::DuplicateCaseKey => "duplicate_case_key",
            Self::InvalidCaseTime => "invalid_case_time",
            Self::EligibleCaseOutsidePeriod => "eligible_case_outside_period",
            Self::AvailabilityAfterCutoff => "availability_after_cutoff",
            Self::ForecastInitializationMismatch => "forecast_initialization_mismatch",
            Self::InvalidCaseArtifactDigest => "invalid_case_artifact_digest",
        }
    }
}

impl fmt::Display for ContractViolation {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{}", self.label())
    }
}

impl std::error::Error for ContractViolation {}

fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 256
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"-_.:".contains(&byte))
}

fn valid_text(value: &str) -> bool {
    !value.trim().is_empty() && value.len() <= 2048
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn validate_model(
    model: &ModelOutputManifest,
    contract: &ComparisonContract,
    spatial: &SpatialSupportContract,
) -> Result<(), ContractViolation> {
    for value in [
        model.model_id.as_str(),
        model.model_version.as_str(),
        model.archive_release.as_str(),
        model.predictand_id.as_str(),
        model.temporal_support_id.as_str(),
        model.spatial_support_id.as_str(),
        model.climatology_id.as_str(),
    ] {
        if !valid_identifier(value) {
            return Err(ContractViolation::InvalidIdentifier);
        }
    }

    if !valid_text(&model.unit) {
        return Err(ContractViolation::MissingText);
    }
    if !valid_sha256(&model.artifact_sha256) {
        return Err(ContractViolation::InvalidSha256);
    }
    if !valid_sha256(&model.grid_sha256) {
        return Err(ContractViolation::InvalidSha256);
    }
    if !valid_sha256(&model.mask_sha256) {
        return Err(ContractViolation::InvalidSha256);
    }

    if model.predictand_id != contract.predictand_id {
        return Err(ContractViolation::PredictandMismatch);
    }
    if model.unit != contract.unit {
        return Err(ContractViolation::UnitMismatch);
    }
    if model.temporal_support_id != contract.temporal_support_id {
        return Err(ContractViolation::TemporalSupportMismatch);
    }
    if model.spatial_support_id != spatial.support_id
        || model.spatial_support_id != contract.spatial_support_id
    {
        return Err(ContractViolation::SpatialSupportMismatch);
    }
    if model.grid_sha256 != spatial.grid_sha256 {
        return Err(ContractViolation::GridMismatch);
    }
    if model.mask_sha256 != spatial.mask_sha256 {
        return Err(ContractViolation::MaskMismatch);
    }
    if model.climatology_id != contract.climatology_id {
        return Err(ContractViolation::ClimatologyMismatch);
    }

    match (model.product_kind, model.ensemble_members) {
        (ProductKind::Deterministic, None) => {}
        (ProductKind::Ensemble, Some(count)) if count > 0 => {}
        _ => return Err(ContractViolation::InvalidEnsembleMetadata),
    }

    if contract.intrinsic_skill_claim {
        let Some(training_cutoff) = model.training_cutoff else {
            return Err(ContractViolation::MissingTrainingCutoff);
        };
        if training_cutoff >= contract.evaluation_start {
            return Err(ContractViolation::TrainingOverlapsEvaluation);
        }

        if let Some(post_processing) = &model.post_processing {
            if !valid_identifier(&post_processing.operation_id)
                || !valid_identifier(&post_processing.operation_version)
            {
                return Err(ContractViolation::InvalidIdentifier);
            }
            if !valid_sha256(&post_processing.fit_data_sha256) {
                return Err(ContractViolation::InvalidSha256);
            }
            if post_processing.uses_evaluation_outcomes {
                return Err(ContractViolation::PostProcessingOutcomeLeakage);
            }
            if post_processing.trained_through >= contract.evaluation_start {
                return Err(ContractViolation::TrainingOverlapsEvaluation);
            }
        }
    } else if let Some(post_processing) = &model.post_processing {
        if !valid_identifier(&post_processing.operation_id)
            || !valid_identifier(&post_processing.operation_version)
        {
            return Err(ContractViolation::InvalidIdentifier);
        }
        if !valid_sha256(&post_processing.fit_data_sha256) {
            return Err(ContractViolation::InvalidSha256);
        }
    }

    Ok(())
}

fn validate_metric(
    metric: &MetricManifest,
    candidate: &ModelOutputManifest,
    reference: &ModelOutputManifest,
) -> Result<(), ContractViolation> {
    for value in [
        metric.metric_id.as_str(),
        metric.definition_version.as_str(),
        metric.scoring_engine.as_str(),
        metric.scoring_engine_version.as_str(),
    ] {
        if !valid_identifier(value) {
            return Err(ContractViolation::InvalidIdentifier);
        }
    }
    if !valid_sha256(&metric.configuration_sha256) {
        return Err(ContractViolation::InvalidSha256);
    }

    let candidate_members = candidate.ensemble_members.unwrap_or(0);
    let reference_members = reference.ensemble_members.unwrap_or(0);
    let both_deterministic = candidate.product_kind == ProductKind::Deterministic
        && reference.product_kind == ProductKind::Deterministic;
    let both_ensemble = candidate.product_kind == ProductKind::Ensemble
        && reference.product_kind == ProductKind::Ensemble;

    let valid = match metric.input_requirement {
        ForecastInputRequirement::DeterministicOnly => both_deterministic,
        ForecastInputRequirement::EnsembleOnly => both_ensemble,
        ForecastInputRequirement::AnyKind => true,
        ForecastInputRequirement::EqualEnsembleSize => {
            both_ensemble && candidate_members == reference_members
        }
        ForecastInputRequirement::ExactEnsembleSize(expected) => {
            expected > 0
                && both_ensemble
                && candidate_members == expected
                && reference_members == expected
        }
        ForecastInputRequirement::MinimumEnsembleSize(minimum) => {
            minimum > 0
                && both_ensemble
                && candidate_members >= minimum
                && reference_members >= minimum
        }
    };

    if valid {
        Ok(())
    } else {
        Err(ContractViolation::MetricInputMismatch)
    }
}

impl ComparisonManifest {
    pub fn validate(&self) -> Result<(), ContractViolation> {
        if !valid_identifier(&self.case_frame_id)
            || !valid_identifier(&self.dataset_release.dataset_id)
            || !valid_identifier(&self.dataset_release.release_id)
            || !valid_identifier(&self.dataset_release.license_id)
            || !valid_identifier(&self.spatial_support.support_id)
            || !valid_identifier(&self.verification.source_id)
            || !valid_identifier(&self.verification.source_version)
            || !valid_identifier(&self.verification.release_id)
            || !valid_identifier(&self.verification.observation_lineage_id)
            || !valid_identifier(&self.verification.processing_epoch)
            || !valid_identifier(&self.contract.comparison_id)
            || !valid_identifier(&self.contract.predictand_id)
            || !valid_identifier(&self.contract.temporal_support_id)
            || !valid_identifier(&self.contract.spatial_support_id)
            || !valid_identifier(&self.contract.climatology_id)
        {
            return Err(ContractViolation::InvalidIdentifier);
        }

        for value in [
            self.dataset_release.source_locator.as_str(),
            self.dataset_release.citation.as_str(),
            self.dataset_release.terms_url.as_str(),
            self.spatial_support.coordinate_convention.as_str(),
            self.spatial_support.area_weight_scheme.as_str(),
            self.contract.unit.as_str(),
        ] {
            if !valid_text(value) {
                return Err(ContractViolation::MissingText);
            }
        }

        if !valid_sha256(&self.dataset_release.artifact_sha256)
            || !valid_sha256(&self.spatial_support.grid_sha256)
            || !valid_sha256(&self.spatial_support.mask_sha256)
            || !valid_sha256(&self.verification.artifact_sha256)
        {
            return Err(ContractViolation::InvalidSha256);
        }

        if self.contract.evaluation_start > self.contract.evaluation_end
            || self.contract.as_of_cutoff < self.contract.evaluation_end
        {
            return Err(ContractViolation::InvalidEvaluationPeriod);
        }
        if self.verification.as_of_cutoff != self.contract.as_of_cutoff {
            return Err(ContractViolation::VerificationCutoffMismatch);
        }
        if self.dataset_release.dataset_id != self.verification.source_id
            || self.dataset_release.release_id != self.verification.release_id
            || self.dataset_release.artifact_sha256 != self.verification.artifact_sha256
        {
            return Err(ContractViolation::DatasetVerificationMismatch);
        }
        if self.dataset_release.retrieved_at > self.contract.as_of_cutoff {
            return Err(ContractViolation::DatasetRetrievedAfterCutoff);
        }
        if self.spatial_support.support_id != self.contract.spatial_support_id {
            return Err(ContractViolation::SpatialSupportMismatch);
        }

        validate_model(&self.candidate, &self.contract, &self.spatial_support)?;
        validate_model(&self.reference, &self.contract, &self.spatial_support)?;
        validate_metric(&self.metric, &self.candidate, &self.reference)?;

        if self.cases.is_empty() {
            return Err(ContractViolation::EmptyCaseFrame);
        }

        let mut keys = BTreeSet::new();
        for case in &self.cases {
            if !valid_identifier(&case.case_id) {
                return Err(ContractViolation::InvalidIdentifier);
            }
            if case.valid_time < case.initialization_time
                || case.valid_time - case.initialization_time != case.lead_seconds
            {
                return Err(ContractViolation::InvalidCaseTime);
            }
            let key = (
                case.case_id.as_str(),
                case.initialization_time,
                case.valid_time,
                case.lead_seconds,
            );
            if !keys.insert(key) {
                return Err(ContractViolation::DuplicateCaseKey);
            }
            if case.eligibility == CaseEligibility::Eligible
                && (case.valid_time < self.contract.evaluation_start
                    || case.valid_time > self.contract.evaluation_end)
            {
                return Err(ContractViolation::EligibleCaseOutsidePeriod);
            }

            for forecast in [&case.candidate, &case.reference] {
                if let ForecastAvailability::Available {
                    artifact_sha256,
                    initialized_at,
                    recorded_at,
                } = forecast
                {
                    if !valid_sha256(artifact_sha256) {
                        return Err(ContractViolation::InvalidCaseArtifactDigest);
                    }
                    if *initialized_at != case.initialization_time {
                        return Err(ContractViolation::ForecastInitializationMismatch);
                    }
                    if *recorded_at > self.contract.as_of_cutoff {
                        return Err(ContractViolation::AvailabilityAfterCutoff);
                    }
                }
            }

            if let Availability::Available {
                artifact_sha256,
                recorded_at,
            } = &case.verification
            {
                if !valid_sha256(artifact_sha256) {
                    return Err(ContractViolation::InvalidCaseArtifactDigest);
                }
                if *recorded_at > self.contract.as_of_cutoff {
                    return Err(ContractViolation::AvailabilityAfterCutoff);
                }
            }
        }

        Ok(())
    }

    pub fn report(&self) -> Result<ComparisonReport, ContractViolation> {
        self.validate()?;
        Ok(ComparisonReport::from_valid_manifest(self))
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CaseReport {
    pub case_id: String,
    pub initialization_time: u64,
    pub valid_time: u64,
    pub lead_seconds: u64,
    pub eligibility: CaseEligibility,
    pub candidate: ForecastAvailability,
    pub reference: ForecastAvailability,
    pub verification: Availability,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ComparisonCounts {
    pub eligible_cases: usize,
    pub excluded_cases: usize,
    pub candidate_available: usize,
    pub reference_available: usize,
    pub candidate_only_forecasts: usize,
    pub reference_only_forecasts: usize,
    pub both_forecasts_missing: usize,
    pub paired_forecasts: usize,
    pub verification_available: usize,
    pub verification_missing: usize,
    pub candidate_verifiable: usize,
    pub reference_verifiable: usize,
    pub paired_verifiable: usize,
    pub pairs_without_verification: usize,
    pub candidate_missing_by_reason: BTreeMap<MissingReason, usize>,
    pub reference_missing_by_reason: BTreeMap<MissingReason, usize>,
    pub verification_missing_by_reason: BTreeMap<MissingReason, usize>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComparisonReport {
    pub comparison_id: String,
    pub case_frame_id: String,
    pub verification_source: String,
    pub verification_version: String,
    pub as_of_cutoff: u64,
    pub counts: ComparisonCounts,
    pub cases: Vec<CaseReport>,
}

fn add_missing_reason(counts: &mut BTreeMap<MissingReason, usize>, availability: &Availability) {
    if let Availability::Missing { reason } = availability {
        *counts.entry(*reason).or_insert(0) += 1;
    }
}

impl ComparisonReport {
    fn from_valid_manifest(manifest: &ComparisonManifest) -> Self {
        let mut counts = ComparisonCounts::default();
        let mut cases = Vec::with_capacity(manifest.cases.len());

        for case in &manifest.cases {
            let eligible = case.eligibility == CaseEligibility::Eligible;
            cases.push(CaseReport {
                case_id: case.case_id.clone(),
                initialization_time: case.initialization_time,
                valid_time: case.valid_time,
                lead_seconds: case.lead_seconds,
                eligibility: case.eligibility,
                candidate: case.candidate.clone(),
                reference: case.reference.clone(),
                verification: case.verification.clone(),
            });

            if !eligible {
                counts.excluded_cases += 1;
                continue;
            }

            counts.eligible_cases += 1;
            let candidate_available = case.candidate.is_available();
            let reference_available = case.reference.is_available();
            let verification_available = case.verification.is_available();

            if candidate_available {
                counts.candidate_available += 1;
            } else {
                if let ForecastAvailability::Missing { reason } = &case.candidate {
                    *counts
                        .candidate_missing_by_reason
                        .entry(*reason)
                        .or_insert(0) += 1;
                }
            }
            if reference_available {
                counts.reference_available += 1;
            } else {
                if let ForecastAvailability::Missing { reason } = &case.reference {
                    *counts
                        .reference_missing_by_reason
                        .entry(*reason)
                        .or_insert(0) += 1;
                }
            }
            if verification_available {
                counts.verification_available += 1;
            } else {
                counts.verification_missing += 1;
                add_missing_reason(
                    &mut counts.verification_missing_by_reason,
                    &case.verification,
                );
            }

            match (candidate_available, reference_available) {
                (true, true) => counts.paired_forecasts += 1,
                (true, false) => counts.candidate_only_forecasts += 1,
                (false, true) => counts.reference_only_forecasts += 1,
                (false, false) => counts.both_forecasts_missing += 1,
            }

            if candidate_available && verification_available {
                counts.candidate_verifiable += 1;
            }
            if reference_available && verification_available {
                counts.reference_verifiable += 1;
            }
            if candidate_available && reference_available && verification_available {
                counts.paired_verifiable += 1;
            }
            if candidate_available && reference_available && !verification_available {
                counts.pairs_without_verification += 1;
            }
        }

        cases.sort_by(|left, right| {
            (
                left.case_id.as_str(),
                left.initialization_time,
                left.valid_time,
                left.lead_seconds,
            )
                .cmp(&(
                    right.case_id.as_str(),
                    right.initialization_time,
                    right.valid_time,
                    right.lead_seconds,
                ))
        });

        Self {
            comparison_id: manifest.contract.comparison_id.clone(),
            case_frame_id: manifest.case_frame_id.clone(),
            verification_source: manifest.verification.source_id.clone(),
            verification_version: manifest.verification.source_version.clone(),
            as_of_cutoff: manifest.contract.as_of_cutoff,
            counts,
            cases,
        }
    }

    pub fn summary_line(&self) -> String {
        format!(
            "eligible={} excluded={} candidate_available={} reference_available={} candidate_only={} reference_only={} both_missing={} paired_forecasts={} verification_available={} verification_missing={} candidate_verifiable={} reference_verifiable={} paired_verifiable={} pairs_without_verification={}",
            self.counts.eligible_cases,
            self.counts.excluded_cases,
            self.counts.candidate_available,
            self.counts.reference_available,
            self.counts.candidate_only_forecasts,
            self.counts.reference_only_forecasts,
            self.counts.both_forecasts_missing,
            self.counts.paired_forecasts,
            self.counts.verification_available,
            self.counts.verification_missing,
            self.counts.candidate_verifiable,
            self.counts.reference_verifiable,
            self.counts.paired_verifiable,
            self.counts.pairs_without_verification,
        )
    }

    pub fn to_csv(&self) -> String {
        let mut output = String::from(
            "case_id,initialization_time,valid_time,lead_seconds,eligibility,candidate,reference,verification\n",
        );

        for case in &self.cases {
            output.push_str(&format!(
                "{},{},{},{},{},{},{},{}\n",
                case.case_id,
                case.initialization_time,
                case.valid_time,
                case.lead_seconds,
                case.eligibility.label(),
                case.candidate.label(),
                case.reference.label(),
                case.verification.label(),
            ));
        }
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DIGEST_A: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const DIGEST_B: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const DIGEST_C: &str = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";

    fn available(digest: &str) -> Availability {
        Availability::Available {
            artifact_sha256: digest.into(),
            recorded_at: 250,
        }
    }

    fn forecast_available(digest: &str, initialized_at: u64) -> ForecastAvailability {
        ForecastAvailability::Available {
            artifact_sha256: digest.into(),
            initialized_at,
            recorded_at: 250,
        }
    }

    fn missing(reason: MissingReason) -> Availability {
        Availability::Missing { reason }
    }

    fn forecast_missing(reason: MissingReason) -> ForecastAvailability {
        ForecastAvailability::Missing { reason }
    }

    fn model(
        model_id: &str,
        digest: &str,
        kind: ProductKind,
        members: Option<usize>,
    ) -> ModelOutputManifest {
        ModelOutputManifest {
            model_id: model_id.into(),
            model_version: "v1".into(),
            archive_release: "archive-v1".into(),
            artifact_sha256: digest.into(),
            predictand_id: "t2m".into(),
            unit: "K".into(),
            temporal_support_id: "hourly".into(),
            spatial_support_id: "global-grid-v1".into(),
            grid_sha256: DIGEST_A.into(),
            mask_sha256: DIGEST_B.into(),
            climatology_id: "era5-clim-1991-2020".into(),
            product_kind: kind,
            ensemble_members: members,
            training_cutoff: Some(50),
            post_processing: None,
        }
    }

    fn fixture() -> ComparisonManifest {
        ComparisonManifest {
            case_frame_id: "frame-v1".into(),
            dataset_release: DatasetReleaseManifest {
                dataset_id: "era5".into(),
                release_id: "era5-release-v1".into(),
                source_locator: "https://example.invalid/era5".into(),
                citation: "ERA5 dataset release v1".into(),
                retrieved_at: 240,
                artifact_sha256: DIGEST_C.into(),
                license_id: "declared-license-v1".into(),
                terms_url: "https://example.invalid/terms".into(),
                redistribution_policy: RedistributionPolicy::Unknown,
            },
            spatial_support: SpatialSupportContract {
                support_id: "global-grid-v1".into(),
                grid_sha256: DIGEST_A.into(),
                mask_sha256: DIGEST_B.into(),
                coordinate_convention: "latitude_longitude".into(),
                area_weight_scheme: "cell_area".into(),
            },
            verification: VerificationContract {
                source_id: "era5".into(),
                source_version: "v1".into(),
                release_id: "era5-release-v1".into(),
                artifact_sha256: DIGEST_C.into(),
                observation_lineage_id: "era5-observation-lineage-v1".into(),
                processing_epoch: "processor-v1".into(),
                as_of_cutoff: 300,
            },
            contract: ComparisonContract {
                comparison_id: "common-source-v1".into(),
                predictand_id: "t2m".into(),
                unit: "K".into(),
                temporal_support_id: "hourly".into(),
                spatial_support_id: "global-grid-v1".into(),
                climatology_id: "era5-clim-1991-2020".into(),
                evaluation_start: 100,
                evaluation_end: 200,
                as_of_cutoff: 300,
                intrinsic_skill_claim: true,
            },
            candidate: model(
                "candidate-model",
                "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
                ProductKind::Deterministic,
                None,
            ),
            reference: model(
                "reference-model",
                "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",
                ProductKind::Deterministic,
                None,
            ),
            metric: MetricManifest {
                metric_id: "rmse".into(),
                definition_version: "v1".into(),
                scoring_engine: "gcpn-score-engine".into(),
                scoring_engine_version: "v1".into(),
                configuration_sha256:
                    "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff".into(),
                input_requirement: ForecastInputRequirement::DeterministicOnly,
            },
            cases: vec![
                ComparisonCase {
                    case_id: "case-001".into(),
                    initialization_time: 100,
                    valid_time: 106,
                    lead_seconds: 6,
                    eligibility: CaseEligibility::Eligible,
                    candidate: forecast_available(DIGEST_A, 100),
                    reference: forecast_available(DIGEST_B, 100),
                    verification: available(DIGEST_C),
                },
                ComparisonCase {
                    case_id: "case-002".into(),
                    initialization_time: 110,
                    valid_time: 116,
                    lead_seconds: 6,
                    eligibility: CaseEligibility::Eligible,
                    candidate: forecast_missing(MissingReason::NotProduced),
                    reference: forecast_available(DIGEST_A, 110),
                    verification: available(DIGEST_B),
                },
                ComparisonCase {
                    case_id: "case-003".into(),
                    initialization_time: 120,
                    valid_time: 126,
                    lead_seconds: 6,
                    eligibility: CaseEligibility::Eligible,
                    candidate: forecast_available(DIGEST_C, 120),
                    reference: forecast_missing(MissingReason::NotArchived),
                    verification: missing(MissingReason::PendingAtCutoff),
                },
                ComparisonCase {
                    case_id: "case-004".into(),
                    initialization_time: 130,
                    valid_time: 136,
                    lead_seconds: 6,
                    eligibility: CaseEligibility::Excluded(
                        ExclusionReason::ExcludedByFrozenProtocol,
                    ),
                    candidate: forecast_missing(MissingReason::NotProduced),
                    reference: forecast_missing(MissingReason::NotProduced),
                    verification: missing(MissingReason::Unknown),
                },
            ],
        }
    }

    #[test]
    fn known_good_common_source_contract_validates() {
        assert_eq!(fixture().validate(), Ok(()));
    }

    #[test]
    fn report_counts_exact_paired_and_marginal_availability() {
        let report = fixture().report().expect("fixture should report");
        assert_eq!(report.counts.eligible_cases, 3);
        assert_eq!(report.counts.excluded_cases, 1);
        assert_eq!(report.counts.candidate_available, 2);
        assert_eq!(report.counts.reference_available, 2);
        assert_eq!(report.counts.candidate_only_forecasts, 1);
        assert_eq!(report.counts.reference_only_forecasts, 1);
        assert_eq!(report.counts.both_forecasts_missing, 0);
        assert_eq!(report.counts.paired_forecasts, 1);
        assert_eq!(report.counts.verification_available, 2);
        assert_eq!(report.counts.verification_missing, 1);
        assert_eq!(report.counts.candidate_verifiable, 1);
        assert_eq!(report.counts.reference_verifiable, 2);
        assert_eq!(report.counts.paired_verifiable, 1);
        assert_eq!(report.counts.pairs_without_verification, 0);
    }

    #[test]
    fn missing_reasons_are_preserved_by_stream() {
        let report = fixture().report().expect("fixture should report");
        assert_eq!(
            report.counts.candidate_missing_by_reason[&MissingReason::NotProduced],
            1
        );
        assert_eq!(
            report.counts.reference_missing_by_reason[&MissingReason::NotArchived],
            1
        );
        assert_eq!(
            report.counts.verification_missing_by_reason[&MissingReason::PendingAtCutoff],
            1
        );
    }

    #[test]
    fn unknown_redistribution_terms_never_become_permission() {
        let manifest = fixture();
        assert!(
            !manifest
                .dataset_release
                .redistribution_policy
                .permits_redistribution_by_declaration()
        );
    }

    #[test]
    fn declared_terms_flag_is_not_a_legal_opinion() {
        assert!(
            RedistributionPolicy::AllowedByDeclaredTerms.permits_redistribution_by_declaration()
        );
        assert!(
            !RedistributionPolicy::RestrictedByDeclaredTerms
                .permits_redistribution_by_declaration()
        );
    }

    #[test]
    fn verification_cutoff_must_match_comparison_cutoff() {
        let mut manifest = fixture();
        manifest.verification.as_of_cutoff = 299;
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::VerificationCutoffMismatch)
        );
    }

    #[test]
    fn dataset_release_must_match_verification_identity_and_digest() {
        let mut manifest = fixture();
        manifest.dataset_release.release_id = "different-release".into();
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::DatasetVerificationMismatch)
        );
    }

    #[test]
    fn spatial_grid_mismatch_is_rejected() {
        let mut manifest = fixture();
        manifest.candidate.grid_sha256 = DIGEST_C.into();
        assert_eq!(manifest.validate(), Err(ContractViolation::GridMismatch));
    }

    #[test]
    fn spatial_mask_mismatch_is_rejected() {
        let mut manifest = fixture();
        manifest.reference.mask_sha256 = DIGEST_C.into();
        assert_eq!(manifest.validate(), Err(ContractViolation::MaskMismatch));
    }

    #[test]
    fn predictand_mismatch_is_rejected() {
        let mut manifest = fixture();
        manifest.reference.predictand_id = "precipitation".into();
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::PredictandMismatch)
        );
    }

    #[test]
    fn unit_mismatch_is_rejected() {
        let mut manifest = fixture();
        manifest.candidate.unit = "celsius".into();
        assert_eq!(manifest.validate(), Err(ContractViolation::UnitMismatch));
    }

    #[test]
    fn temporal_support_mismatch_is_rejected() {
        let mut manifest = fixture();
        manifest.reference.temporal_support_id = "daily".into();
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::TemporalSupportMismatch)
        );
    }

    #[test]
    fn climatology_mismatch_is_rejected() {
        let mut manifest = fixture();
        manifest.candidate.climatology_id = "different-climatology".into();
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::ClimatologyMismatch)
        );
    }

    #[test]
    fn model_digest_must_be_sha256_hex() {
        let mut manifest = fixture();
        manifest.candidate.artifact_sha256 = "not-a-digest".into();
        assert_eq!(manifest.validate(), Err(ContractViolation::InvalidSha256));
    }

    #[test]
    fn duplicate_case_key_is_rejected_even_when_availability_differs() {
        let mut manifest = fixture();
        let mut duplicate = manifest.cases[0].clone();
        duplicate.candidate = forecast_missing(MissingReason::NotProduced);
        manifest.cases.push(duplicate);
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::DuplicateCaseKey)
        );
    }

    #[test]
    fn lead_time_must_match_valid_and_initialization_times() {
        let mut manifest = fixture();
        manifest.cases[0].lead_seconds = 7;
        assert_eq!(manifest.validate(), Err(ContractViolation::InvalidCaseTime));
    }

    #[test]
    fn eligible_case_must_be_inside_frozen_evaluation_period() {
        let mut manifest = fixture();
        manifest.cases[0].valid_time = 99;
        manifest.cases[0].initialization_time = 93;
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::EligibleCaseOutsidePeriod)
        );
    }

    #[test]
    fn excluded_case_remains_visible_but_is_not_in_score_denominator() {
        let report = fixture().report().expect("fixture should report");
        assert_eq!(report.cases.len(), 4);
        assert_eq!(
            report
                .cases
                .iter()
                .filter(|case| case.eligibility == CaseEligibility::Eligible)
                .count(),
            3
        );
        assert_eq!(
            report.counts.eligible_cases + report.counts.excluded_cases,
            4
        );
    }

    #[test]
    fn forecast_initialization_must_match_the_case_initialization() {
        let mut manifest = fixture();
        manifest.cases[0].candidate = ForecastAvailability::Available {
            artifact_sha256: DIGEST_A.into(),
            initialized_at: 99,
            recorded_at: 250,
        };
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::ForecastInitializationMismatch)
        );
    }

    #[test]
    fn available_artifact_after_cutoff_is_rejected() {
        let mut manifest = fixture();
        manifest.cases[0].verification = Availability::Available {
            artifact_sha256: DIGEST_A.into(),
            recorded_at: 301,
        };
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::AvailabilityAfterCutoff)
        );
    }

    #[test]
    fn equal_ensemble_metric_requires_matching_member_counts() {
        let mut manifest = fixture();
        manifest.candidate.product_kind = ProductKind::Ensemble;
        manifest.candidate.ensemble_members = Some(20);
        manifest.reference.product_kind = ProductKind::Ensemble;
        manifest.reference.ensemble_members = Some(10);
        manifest.metric.input_requirement = ForecastInputRequirement::EqualEnsembleSize;
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::MetricInputMismatch)
        );
    }

    #[test]
    fn exact_ensemble_metric_checks_both_member_counts() {
        let mut manifest = fixture();
        manifest.candidate.product_kind = ProductKind::Ensemble;
        manifest.candidate.ensemble_members = Some(20);
        manifest.reference.product_kind = ProductKind::Ensemble;
        manifest.reference.ensemble_members = Some(20);
        manifest.metric.input_requirement = ForecastInputRequirement::ExactEnsembleSize(20);
        assert_eq!(manifest.validate(), Ok(()));
    }

    #[test]
    fn evaluation_outcome_leakage_is_rejected_for_intrinsic_skill_claim() {
        let mut manifest = fixture();
        manifest.candidate.post_processing = Some(PostProcessingProvenance {
            operation_id: "calibration-v1".into(),
            operation_version: "1".into(),
            fit_data_sha256: DIGEST_A.into(),
            trained_through: 50,
            uses_evaluation_outcomes: true,
        });
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::PostProcessingOutcomeLeakage)
        );
    }

    #[test]
    fn training_cutoff_must_precede_evaluation_for_intrinsic_skill_claim() {
        let mut manifest = fixture();
        manifest.reference.training_cutoff = Some(100);
        assert_eq!(
            manifest.validate(),
            Err(ContractViolation::TrainingOverlapsEvaluation)
        );
    }

    #[test]
    fn report_rows_are_canonical_independent_of_input_order() {
        let mut manifest = fixture();
        let report_a = manifest.report().expect("fixture should report");
        manifest.cases.reverse();
        let report_b = manifest.report().expect("reversed fixture should report");
        assert_eq!(report_a.to_csv(), report_b.to_csv());
        assert_eq!(report_a.summary_line(), report_b.summary_line());
    }

    #[test]
    fn report_never_calculates_or_invents_forecast_scores() {
        let report = fixture().report().expect("fixture should report");
        assert!(!report.to_csv().contains("rmse_value"));
        assert_eq!(report.counts.paired_forecasts, 1);
    }
}

//! Exact paired pFSS scoring over a validated common-source case frame.
//!
//! This adapter accepts only the intersection of eligible candidate,
//! reference and verification availability recorded in the common-source
//! manifest. It refuses to reconstruct a denominator from supplied arrays.
//! Candidate/reference pairs are scored with shared observations, grid masks,
//! area weights, lead-time keys and the paired event-block bootstrap.

mod sha256;

use sha256::sha256_hex;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use gcpn_common_source_comparison::{
    Availability, CaseEligibility, ComparisonCounts, ComparisonManifest, ComparisonReport,
    ContractViolation, ForecastAvailability,
};
use gcpn_probabilistic_fss::{
    BlockBootstrapConfig, FssCase, FssConfig, FssError, PairedBlockBootstrapReport,
    PairedFssBlockCase, paired_block_bootstrap,
};

#[derive(Clone, Debug, PartialEq)]
pub struct ForecastGridArtifact {
    pub case_id: String,
    pub artifact_sha256: String,
    pub initialized_at: u64,
    pub valid_time: u64,
    pub lead_seconds: u64,
    pub recorded_at: u64,
    pub grid_sha256: String,
    pub mask_sha256: String,
    pub spatial_support_id: String,
    pub event_block_id: String,
    pub field: FssCase,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerificationGridArtifact {
    pub artifact_sha256: String,
    pub recorded_at: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PairedGridArtifact {
    pub case_id: String,
    pub candidate: ForecastGridArtifact,
    pub reference: ForecastGridArtifact,
    pub verification: VerificationGridArtifact,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EventBlockAssignment {
    pub case_id: String,
    pub event_block_id: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EventBlockManifest {
    pub scheme_id: String,
    pub scheme_version: String,
    pub mapping_sha256: String,
    pub assignments: Vec<EventBlockAssignment>,
}

/// Stable length-prefixed canonical metric configuration digest.
pub fn fss_config_sha256(config: &FssConfig) -> String {
    let mut canonical = b"GCPN-PFSS-CONFIG-V1\0".to_vec();
    canonical.extend_from_slice(&config.threshold.to_bits().to_be_bytes());
    push_string(&mut canonical, config.threshold_operator.label());
    push_string(&mut canonical, &config.unit);
    canonical.extend_from_slice(&(config.window_width_cells as u64).to_be_bytes());
    push_string(&mut canonical, config.boundary_policy.label());
    push_string(&mut canonical, config.missing_policy.label());
    sha256_hex(&canonical)
}

/// Stable digest of a sorted event-block map and its named assignment scheme.
pub fn event_block_manifest_sha256(manifest: &EventBlockManifest) -> Result<String, AdapterError> {
    if !valid_identifier(&manifest.scheme_id) || !valid_identifier(&manifest.scheme_version) {
        return Err(AdapterError::EventBlockMappingMismatch);
    }
    let mut assignments = manifest.assignments.clone();
    assignments.sort_by(|left, right| left.case_id.cmp(&right.case_id));
    let mut seen = BTreeSet::new();
    let mut canonical = b"GCPN-EVENT-BLOCK-MAP-V1\0".to_vec();
    push_string(&mut canonical, &manifest.scheme_id);
    push_string(&mut canonical, &manifest.scheme_version);
    for assignment in &assignments {
        if !valid_identifier(&assignment.case_id)
            || !valid_identifier(&assignment.event_block_id)
            || !seen.insert(assignment.case_id.as_str())
        {
            return Err(AdapterError::EventBlockMappingMismatch);
        }
        push_string(&mut canonical, &assignment.case_id);
        push_string(&mut canonical, &assignment.event_block_id);
    }
    Ok(sha256_hex(&canonical))
}

fn push_string(output: &mut Vec<u8>, value: &str) {
    output.extend_from_slice(&(value.len() as u64).to_be_bytes());
    output.extend_from_slice(value.as_bytes());
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdapterError {
    CommonSourceViolation(ContractViolation),
    ScoringFailure(FssError),
    ScoreUnitMismatch,
    MissingPairedGridArtifact,
    UnmanifestedGridArtifact,
    DuplicateGridArtifact,
    ForecastArtifactMismatch,
    VerificationArtifactMismatch,
    ForecastTimeMismatch,
    SpatialSupportMismatch,
    PairSupportMismatch,
    EventBlockMismatch,
    EventBlockMappingMismatch,
    MetricContractMismatch,
    EnsembleMemberCountMismatch,
    PairedDenominatorMismatch,
}

impl fmt::Display for AdapterError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CommonSourceViolation(error) => {
                write!(formatter, "common-source contract violation: {error}")
            }
            Self::ScoringFailure(error) => write!(formatter, "pFSS scoring failure: {error}"),
            Self::ScoreUnitMismatch => {
                formatter.write_str("FSS unit differs from the frozen comparison unit")
            }
            Self::MissingPairedGridArtifact => {
                formatter.write_str("paired-verifiable case has no grid artifact pair")
            }
            Self::UnmanifestedGridArtifact => {
                formatter.write_str("grid artifact pair is absent from the common-source frame")
            }
            Self::DuplicateGridArtifact => {
                formatter.write_str("duplicate grid artifact pair for a common-source case")
            }
            Self::ForecastArtifactMismatch => formatter
                .write_str("forecast artifact digest/time does not match common-source record"),
            Self::VerificationArtifactMismatch => formatter
                .write_str("verification artifact digest/time does not match common-source record"),
            Self::ForecastTimeMismatch => formatter.write_str(
                "forecast initialization, valid time or lead differs from common-source case",
            ),
            Self::SpatialSupportMismatch => formatter
                .write_str("grid, mask or spatial support differs from common-source contract"),
            Self::PairSupportMismatch => formatter.write_str(
                "candidate and reference do not share identical observation/grid/mask/weights",
            ),
            Self::EventBlockMismatch => {
                formatter.write_str("candidate/reference event-block identity differs")
            }
            Self::EventBlockMappingMismatch => {
                formatter.write_str("event-block mapping is not exactly bound to paired case IDs")
            }
            Self::MetricContractMismatch => formatter.write_str(
                "scoring engine, metric version or FSS config digest differs from manifest",
            ),
            Self::EnsembleMemberCountMismatch => formatter
                .write_str("scored ensemble member count differs from model output manifest"),
            Self::PairedDenominatorMismatch => formatter
                .write_str("scored case IDs differ from common-source paired-verifiable case IDs"),
        }
    }
}

impl std::error::Error for AdapterError {}

#[derive(Clone, Debug, PartialEq)]
pub struct PairedScoreReport {
    pub comparison_id: String,
    pub case_frame_id: String,
    pub verification_source: String,
    pub verification_version: String,
    pub as_of_cutoff: u64,
    pub availability: ComparisonCounts,
    pub scored_case_ids: Vec<String>,
    pub fss_config: FssConfig,
    pub metric_configuration_sha256: String,
    pub candidate_ensemble_members: usize,
    pub reference_ensemble_members: usize,
    pub event_block_scheme_id: String,
    pub event_block_scheme_version: String,
    pub event_block_mapping_sha256: String,
    pub bootstrap: PairedBlockBootstrapReport,
}

impl PairedScoreReport {
    pub fn summary_line(&self) -> String {
        format!(
            "comparison={} frame={} as_of={} eligible={} excluded={} candidate_only={} reference_only={} both_missing={} paired_forecasts={} verification_missing={} pairs_without_verification={} paired_verifiable={} scored={} candidate_members={} reference_members={} event_block_scheme={}:{} candidate_pfss={:.8} reference_pfss={:.8} paired_delta={:.8} ci95_lower={:.8} ci95_upper={:.8} bootstrap_blocks={} bootstrap_replicates={} bootstrap_degenerate={}",
            self.comparison_id,
            self.case_frame_id,
            self.as_of_cutoff,
            self.availability.eligible_cases,
            self.availability.excluded_cases,
            self.availability.candidate_only_forecasts,
            self.availability.reference_only_forecasts,
            self.availability.both_forecasts_missing,
            self.availability.paired_forecasts,
            self.availability.verification_missing,
            self.availability.pairs_without_verification,
            self.availability.paired_verifiable,
            self.scored_case_ids.len(),
            self.candidate_ensemble_members,
            self.reference_ensemble_members,
            self.event_block_scheme_id,
            self.event_block_scheme_version,
            self.bootstrap.candidate_p_fss,
            self.bootstrap.reference_p_fss,
            self.bootstrap.mean_paired_delta,
            self.bootstrap.lower_95_percent,
            self.bootstrap.upper_95_percent,
            self.bootstrap.independent_event_blocks,
            self.bootstrap.requested_replicates,
            self.bootstrap.degenerate_replicates,
        )
    }

    pub fn to_csv(&self) -> String {
        let mut output = String::from(
            "case_id,comparison_id,case_frame_id,verification_source,verification_version,as_of_cutoff,threshold,threshold_operator,unit,window_width_cells,candidate_members,reference_members,event_block_scheme,event_block_version,event_block_mapping_sha256,candidate_pfss,reference_pfss,paired_delta,ci95_lower,ci95_upper\n",
        );
        for case_id in &self.scored_case_ids {
            output.push_str(&format!(
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{:.8},{:.8},{:.8},{:.8},{:.8}\n",
                case_id,
                self.comparison_id,
                self.case_frame_id,
                self.verification_source,
                self.verification_version,
                self.as_of_cutoff,
                self.fss_config.threshold,
                self.fss_config.threshold_operator.label(),
                self.fss_config.unit,
                self.fss_config.window_width_cells,
                self.candidate_ensemble_members,
                self.reference_ensemble_members,
                self.event_block_scheme_id,
                self.event_block_scheme_version,
                self.event_block_mapping_sha256,
                self.bootstrap.candidate_p_fss,
                self.bootstrap.reference_p_fss,
                self.bootstrap.mean_paired_delta,
                self.bootstrap.lower_95_percent,
                self.bootstrap.upper_95_percent,
            ));
        }
        output
    }
}

fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"-_.:".contains(&byte))
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn is_paired_verifiable(case: &gcpn_common_source_comparison::CaseReport) -> bool {
    case.eligibility == CaseEligibility::Eligible
        && case.candidate.is_available()
        && case.reference.is_available()
        && case.verification.is_available()
}

fn validate_forecast_artifact(
    source: &ForecastAvailability,
    artifact: &ForecastGridArtifact,
    case_id: &str,
    initialization_time: u64,
    valid_time: u64,
    lead_seconds: u64,
    as_of_cutoff: u64,
) -> Result<(), AdapterError> {
    let ForecastAvailability::Available {
        artifact_sha256,
        initialized_at,
        recorded_at,
    } = source
    else {
        return Err(AdapterError::MissingPairedGridArtifact);
    };

    if artifact.case_id != case_id || artifact.field.case_id != case_id {
        return Err(AdapterError::ForecastArtifactMismatch);
    }
    if artifact.artifact_sha256 != *artifact_sha256
        || artifact.initialized_at != *initialized_at
        || artifact.recorded_at != *recorded_at
        || artifact.recorded_at > as_of_cutoff
    {
        return Err(AdapterError::ForecastArtifactMismatch);
    }
    if artifact.initialized_at != initialization_time
        || artifact.valid_time != valid_time
        || artifact.lead_seconds != lead_seconds
    {
        return Err(AdapterError::ForecastTimeMismatch);
    }
    Ok(())
}

fn validate_verification_artifact(
    source: &Availability,
    artifact: &VerificationGridArtifact,
    as_of_cutoff: u64,
) -> Result<(), AdapterError> {
    let Availability::Available {
        artifact_sha256,
        recorded_at,
    } = source
    else {
        return Err(AdapterError::MissingPairedGridArtifact);
    };

    if artifact.artifact_sha256 != *artifact_sha256
        || artifact.recorded_at != *recorded_at
        || artifact.recorded_at > as_of_cutoff
    {
        return Err(AdapterError::VerificationArtifactMismatch);
    }
    Ok(())
}

pub fn score_paired_manifest(
    manifest: &ComparisonManifest,
    artifacts: &[PairedGridArtifact],
    event_blocks: &EventBlockManifest,
    fss_config: &FssConfig,
    bootstrap_config: &BlockBootstrapConfig,
) -> Result<PairedScoreReport, AdapterError> {
    let comparison_report: ComparisonReport = manifest
        .report()
        .map_err(AdapterError::CommonSourceViolation)?;

    if fss_config.unit != manifest.contract.unit {
        return Err(AdapterError::ScoreUnitMismatch);
    }
    if manifest.metric.metric_id != "pfss"
        || manifest.metric.definition_version != "v0.1"
        || manifest.metric.scoring_engine != "gcpn-probabilistic-fss"
        || manifest.metric.scoring_engine_version != "0.1.0"
        || manifest.metric.configuration_sha256 != fss_config_sha256(fss_config)
    {
        return Err(AdapterError::MetricContractMismatch);
    }
    if !valid_sha256(&event_blocks.mapping_sha256)
        || event_block_manifest_sha256(event_blocks).as_deref()
            != Ok(event_blocks.mapping_sha256.as_str())
    {
        return Err(AdapterError::EventBlockMappingMismatch);
    }

    let expected: BTreeMap<&str, &gcpn_common_source_comparison::CaseReport> = comparison_report
        .cases
        .iter()
        .filter(|case| is_paired_verifiable(case))
        .map(|case| (case.case_id.as_str(), case))
        .collect();

    if expected.len() != comparison_report.counts.paired_verifiable {
        return Err(AdapterError::PairedDenominatorMismatch);
    }

    let mut assignments = BTreeMap::new();
    for assignment in &event_blocks.assignments {
        if assignments
            .insert(assignment.case_id.as_str(), assignment.event_block_id.as_str())
            .is_some()
        {
            return Err(AdapterError::EventBlockMappingMismatch);
        }
    }
    if assignments.len() != expected.len()
        || expected.keys().any(|case_id| !assignments.contains_key(case_id))
    {
        return Err(AdapterError::EventBlockMappingMismatch);
    }

    let mut supplied = BTreeMap::new();
    for artifact in artifacts {
        if !valid_identifier(&artifact.case_id)
            || !valid_identifier(&artifact.candidate.event_block_id)
            || !valid_identifier(&artifact.reference.event_block_id)
        {
            return Err(AdapterError::EventBlockMismatch);
        }
        if artifact.candidate.event_block_id != artifact.reference.event_block_id {
            return Err(AdapterError::EventBlockMismatch);
        }
        if !expected.contains_key(artifact.case_id.as_str()) {
            return Err(AdapterError::UnmanifestedGridArtifact);
        }
        if assignments.get(artifact.case_id.as_str()).copied()
            != Some(artifact.candidate.event_block_id.as_str())
        {
            return Err(AdapterError::EventBlockMappingMismatch);
        }
        if supplied
            .insert(artifact.case_id.as_str(), artifact)
            .is_some()
        {
            return Err(AdapterError::DuplicateGridArtifact);
        }
    }

    if supplied.len() != expected.len()
        || expected
            .keys()
            .any(|case_id| !supplied.contains_key(case_id))
    {
        return Err(AdapterError::MissingPairedGridArtifact);
    }

    let mut paired_cases = Vec::with_capacity(expected.len());
    for (case_id, source_case) in &expected {
        let artifact = supplied
            .get(case_id)
            .ok_or(AdapterError::MissingPairedGridArtifact)?;

        validate_forecast_artifact(
            &source_case.candidate,
            &artifact.candidate,
            case_id,
            source_case.initialization_time,
            source_case.valid_time,
            source_case.lead_seconds,
            comparison_report.as_of_cutoff,
        )?;
        validate_forecast_artifact(
            &source_case.reference,
            &artifact.reference,
            case_id,
            source_case.initialization_time,
            source_case.valid_time,
            source_case.lead_seconds,
            comparison_report.as_of_cutoff,
        )?;
        validate_verification_artifact(
            &source_case.verification,
            &artifact.verification,
            comparison_report.as_of_cutoff,
        )?;

        for (forecast, model) in [
            (&artifact.candidate, &manifest.candidate),
            (&artifact.reference, &manifest.reference),
        ] {
            if !valid_sha256(&forecast.artifact_sha256)
                || forecast.grid_sha256 != manifest.spatial_support.grid_sha256
                || forecast.mask_sha256 != manifest.spatial_support.mask_sha256
                || forecast.spatial_support_id != manifest.spatial_support.support_id
            {
                return Err(AdapterError::SpatialSupportMismatch);
            }
            let declared_members = match (model.product_kind, model.ensemble_members) {
                (gcpn_common_source_comparison::ProductKind::Deterministic, None) => 1,
                (gcpn_common_source_comparison::ProductKind::Ensemble, Some(count)) if count > 0 => count,
                _ => return Err(AdapterError::EnsembleMemberCountMismatch),
            };
            if forecast.field.forecast_members.len() != declared_members {
                return Err(AdapterError::EnsembleMemberCountMismatch);
            }
            forecast
                .field
                .validate(fss_config)
                .map_err(AdapterError::ScoringFailure)?;
        }

        let candidate = &artifact.candidate.field;
        let reference = &artifact.reference.field;
        if candidate.rows != reference.rows
            || candidate.columns != reference.columns
            || candidate.observation != reference.observation
            || candidate.cell_weights != reference.cell_weights
            || candidate.valid_mask != reference.valid_mask
        {
            return Err(AdapterError::PairSupportMismatch);
        }

        paired_cases.push(PairedFssBlockCase {
            event_block_id: artifact.candidate.event_block_id.clone(),
            case_id: case_id.to_string(),
            candidate: candidate.clone(),
            reference: reference.clone(),
        });
    }

    paired_cases.sort_by(|left, right| left.case_id.cmp(&right.case_id));
    let scored_case_ids: Vec<String> = paired_cases
        .iter()
        .map(|case| case.case_id.clone())
        .collect();

    let bootstrap = paired_block_bootstrap(&paired_cases, fss_config, bootstrap_config)
        .map_err(AdapterError::ScoringFailure)?;

    if scored_case_ids.len() != comparison_report.counts.paired_verifiable
        || scored_case_ids
            .windows(2)
            .any(|window| window[0] >= window[1])
    {
        return Err(AdapterError::PairedDenominatorMismatch);
    }

    Ok(PairedScoreReport {
        comparison_id: comparison_report.comparison_id,
        case_frame_id: comparison_report.case_frame_id,
        verification_source: comparison_report.verification_source,
        verification_version: comparison_report.verification_version,
        as_of_cutoff: comparison_report.as_of_cutoff,
        availability: comparison_report.counts,
        scored_case_ids,
        fss_config: fss_config.clone(),
        metric_configuration_sha256: manifest.metric.configuration_sha256.clone(),
        candidate_ensemble_members: manifest.candidate.ensemble_members.unwrap_or(1),
        reference_ensemble_members: manifest.reference.ensemble_members.unwrap_or(1),
        event_block_scheme_id: event_blocks.scheme_id.clone(),
        event_block_scheme_version: event_blocks.scheme_version.clone(),
        event_block_mapping_sha256: event_blocks.mapping_sha256.clone(),
        bootstrap,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use gcpn_common_source_comparison::{
        CaseEligibility, ComparisonCase, ComparisonContract, ComparisonManifest,
        DatasetReleaseManifest, ExclusionReason, ForecastAvailability, ForecastInputRequirement,
        MetricManifest, MissingReason, ModelOutputManifest, ProductKind, RedistributionPolicy,
        SpatialSupportContract, VerificationContract,
    };
    use gcpn_probabilistic_fss::{BoundaryPolicy, FssCase, MissingPolicy, ThresholdOperator};

    const DIGEST_A: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const DIGEST_B: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const DIGEST_C: &str = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
    const DIGEST_D: &str = "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd";
    const DIGEST_E: &str = "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee";
    const DIGEST_F: &str = "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";

    fn model(model_id: &str, digest: &str) -> ModelOutputManifest {
        ModelOutputManifest {
            model_id: model_id.into(),
            model_version: "v1".into(),
            archive_release: "archive-v1".into(),
            artifact_sha256: digest.into(),
            predictand_id: "t2m".into(),
            unit: "K".into(),
            temporal_support_id: "hourly".into(),
            spatial_support_id: "grid-v1".into(),
            grid_sha256: DIGEST_A.into(),
            mask_sha256: DIGEST_B.into(),
            climatology_id: "climatology-v1".into(),
            product_kind: ProductKind::Ensemble,
            ensemble_members: Some(3),
            training_cutoff: Some(50),
            post_processing: None,
        }
    }

    fn available_forecast(digest: &str, initialized_at: u64) -> ForecastAvailability {
        ForecastAvailability::Available {
            artifact_sha256: digest.into(),
            initialized_at,
            recorded_at: 300,
        }
    }

    fn available_verification(digest: &str) -> Availability {
        Availability::Available {
            artifact_sha256: digest.into(),
            recorded_at: 310,
        }
    }

    fn comparison_case(
        case_id: &str,
        initialization_time: u64,
        valid_time: u64,
        candidate: ForecastAvailability,
        reference: ForecastAvailability,
        verification: Availability,
        eligibility: CaseEligibility,
    ) -> ComparisonCase {
        ComparisonCase {
            case_id: case_id.into(),
            initialization_time,
            valid_time,
            lead_seconds: valid_time - initialization_time,
            eligibility,
            candidate,
            reference,
            verification,
        }
    }

    fn manifest() -> ComparisonManifest {
        ComparisonManifest {
            case_frame_id: "frame-v1".into(),
            dataset_release: DatasetReleaseManifest {
                dataset_id: "verification-source".into(),
                release_id: "release-v1".into(),
                source_locator: "https://example.invalid/verification".into(),
                citation: "Synthetic verification fixture".into(),
                retrieved_at: 200,
                artifact_sha256: DIGEST_C.into(),
                license_id: "license-v1".into(),
                terms_url: "https://example.invalid/terms".into(),
                redistribution_policy: RedistributionPolicy::Unknown,
            },
            spatial_support: SpatialSupportContract {
                support_id: "grid-v1".into(),
                grid_sha256: DIGEST_A.into(),
                mask_sha256: DIGEST_B.into(),
                coordinate_convention: "local_cartesian".into(),
                area_weight_scheme: "uniform".into(),
            },
            verification: VerificationContract {
                source_id: "verification-source".into(),
                source_version: "version-v1".into(),
                release_id: "release-v1".into(),
                artifact_sha256: DIGEST_C.into(),
                observation_lineage_id: "verification-lineage-v1".into(),
                processing_epoch: "processing-v1".into(),
                as_of_cutoff: 400,
            },
            contract: ComparisonContract {
                comparison_id: "comparison-v1".into(),
                predictand_id: "t2m".into(),
                unit: "K".into(),
                temporal_support_id: "hourly".into(),
                spatial_support_id: "grid-v1".into(),
                climatology_id: "climatology-v1".into(),
                evaluation_start: 100,
                evaluation_end: 300,
                as_of_cutoff: 400,
                intrinsic_skill_claim: true,
            },
            candidate: model("candidate-model", DIGEST_D),
            reference: model("reference-model", DIGEST_E),
            metric: MetricManifest {
                metric_id: "pfss".into(),
                definition_version: "v0.1".into(),
                scoring_engine: "gcpn-probabilistic-fss".into(),
                scoring_engine_version: "0.1.0".into(),
                configuration_sha256: fss_config_sha256(&config()),
                input_requirement: ForecastInputRequirement::EnsembleOnly,
            },
            cases: vec![
                comparison_case(
                    "case-a",
                    100,
                    110,
                    available_forecast(DIGEST_D, 100),
                    available_forecast(DIGEST_E, 100),
                    available_verification(DIGEST_C),
                    CaseEligibility::Eligible,
                ),
                comparison_case(
                    "case-b",
                    200,
                    210,
                    available_forecast(DIGEST_F, 200),
                    available_forecast(DIGEST_A, 200),
                    available_verification(DIGEST_C),
                    CaseEligibility::Eligible,
                ),
            ],
        }
    }

    fn config() -> FssConfig {
        FssConfig {
            threshold: 0.5,
            threshold_operator: ThresholdOperator::GreaterThanOrEqual,
            unit: "K".into(),
            window_width_cells: 1,
            boundary_policy: BoundaryPolicy::TruncateAtDomain,
            missing_policy: MissingPolicy::RejectCase,
        }
    }

    fn field(case_id: &str, forecast_events: [bool; 4], observation_events: [bool; 4]) -> FssCase {
        FssCase {
            case_id: case_id.into(),
            rows: 2,
            columns: 2,
            forecast_members: vec![
                forecast_events
                    .into_iter()
                    .map(|event| Some(if event { 1.0 } else { 0.0 }))
                    .collect(),
                forecast_events
                    .into_iter()
                    .map(|event| Some(if event { 1.0 } else { 0.0 }))
                    .collect(),
                forecast_events
                    .into_iter()
                    .map(|event| Some(if event { 1.0 } else { 0.0 }))
                    .collect(),
            ],
            observation: observation_events
                .into_iter()
                .map(|event| Some(if event { 1.0 } else { 0.0 }))
                .collect(),
            cell_weights: vec![1.0; 4],
            valid_mask: vec![true; 4],
        }
    }

    fn artifact(
        case_id: &str,
        event_block_id: &str,
        candidate_digest: &str,
        reference_digest: &str,
        verification_digest: &str,
        initialized_at: u64,
        valid_time: u64,
        candidate_events: [bool; 4],
        reference_events: [bool; 4],
        observation_events: [bool; 4],
    ) -> PairedGridArtifact {
        PairedGridArtifact {
            case_id: case_id.into(),
            candidate: ForecastGridArtifact {
                case_id: case_id.into(),
                artifact_sha256: candidate_digest.into(),
                initialized_at,
                valid_time,
                lead_seconds: valid_time - initialized_at,
                recorded_at: 300,
                grid_sha256: DIGEST_A.into(),
                mask_sha256: DIGEST_B.into(),
                spatial_support_id: "grid-v1".into(),
                event_block_id: event_block_id.into(),
                field: field(case_id, candidate_events, observation_events),
            },
            reference: ForecastGridArtifact {
                case_id: case_id.into(),
                artifact_sha256: reference_digest.into(),
                initialized_at,
                valid_time,
                lead_seconds: valid_time - initialized_at,
                recorded_at: 300,
                grid_sha256: DIGEST_A.into(),
                mask_sha256: DIGEST_B.into(),
                spatial_support_id: "grid-v1".into(),
                event_block_id: event_block_id.into(),
                field: field(case_id, reference_events, observation_events),
            },
            verification: VerificationGridArtifact {
                artifact_sha256: verification_digest.into(),
                recorded_at: 310,
            },
        }
    }

    fn artifacts() -> Vec<PairedGridArtifact> {
        let observed = [true, false, false, false];
        let miss = [false, false, false, false];
        vec![
            artifact(
                "case-a", "storm-a", DIGEST_D, DIGEST_E, DIGEST_C, 100, 110, observed, miss,
                observed,
            ),
            artifact(
                "case-b", "storm-b", DIGEST_F, DIGEST_A, DIGEST_C, 200, 210, observed, miss,
                observed,
            ),
        ]
    }

    fn event_blocks() -> EventBlockManifest {
        let mut blocks = EventBlockManifest {
            scheme_id: "storm-event-v1".into(),
            scheme_version: "1".into(),
            mapping_sha256: String::new(),
            assignments: vec![
                EventBlockAssignment {
                    case_id: "case-a".into(),
                    event_block_id: "storm-a".into(),
                },
                EventBlockAssignment {
                    case_id: "case-b".into(),
                    event_block_id: "storm-b".into(),
                },
            ],
        };
        blocks.mapping_sha256 =
            event_block_manifest_sha256(&blocks).expect("event-block mapping should hash");
        blocks
    }

    #[test]
    fn paired_score_uses_exact_common_source_intersection() {
        let comparison = manifest();
        let report = score_paired_manifest(
            &comparison,
            &artifacts(),
            &event_blocks(),
            &config(),
            &BlockBootstrapConfig {
                replicates: 128,
                seed: 7,
            },
        )
        .expect("paired common-source score should run");

        assert_eq!(report.availability.eligible_cases, 2);
        assert_eq!(report.availability.paired_verifiable, 2);
        assert_eq!(report.scored_case_ids, vec!["case-a", "case-b"]);
        assert_eq!(report.bootstrap.candidate_p_fss, 1.0);
        assert_eq!(report.bootstrap.reference_p_fss, 0.0);
        assert_eq!(report.bootstrap.mean_paired_delta, 1.0);
        assert_eq!(report.bootstrap.independent_event_blocks, 2);
    }

    #[test]
    fn candidate_only_and_excluded_cases_remain_outside_score_denominator() {
        let mut comparison = manifest();
        comparison.cases.push(comparison_case(
            "candidate-only",
            250,
            260,
            available_forecast(DIGEST_D, 250),
            ForecastAvailability::Missing {
                reason: MissingReason::NotProduced,
            },
            available_verification(DIGEST_C),
            CaseEligibility::Eligible,
        ));
        comparison.cases.push(comparison_case(
            "excluded",
            270,
            280,
            ForecastAvailability::Missing {
                reason: MissingReason::NotProduced,
            },
            ForecastAvailability::Missing {
                reason: MissingReason::NotProduced,
            },
            Availability::Missing {
                reason: MissingReason::Unknown,
            },
            CaseEligibility::Excluded(ExclusionReason::ExcludedByFrozenProtocol),
        ));

        let report = score_paired_manifest(
            &comparison,
            &artifacts(),
            &event_blocks(),
            &config(),
            &BlockBootstrapConfig {
                replicates: 128,
                seed: 8,
            },
        )
        .expect("unpaired/excluded rows should remain visible");

        assert_eq!(report.availability.candidate_only_forecasts, 1);
        assert_eq!(report.availability.excluded_cases, 1);
        assert_eq!(report.scored_case_ids, vec!["case-a", "case-b"]);
    }

    #[test]
    fn missing_grid_for_paired_verifiable_case_is_fatal() {
        let mut supplied = artifacts();
        supplied.pop();
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &supplied,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 9,
                },
            ),
            Err(AdapterError::MissingPairedGridArtifact)
        );
    }

    #[test]
    fn unmanifested_or_duplicate_grid_artifacts_are_rejected() {
        let mut supplied = artifacts();
        let mut extra = supplied[0].clone();
        extra.case_id = "not-in-frame".into();
        extra.candidate.case_id = "not-in-frame".into();
        extra.reference.case_id = "not-in-frame".into();
        extra.candidate.field.case_id = "not-in-frame".into();
        extra.reference.field.case_id = "not-in-frame".into();
        supplied.push(extra);
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &supplied,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 10,
                },
            ),
            Err(AdapterError::UnmanifestedGridArtifact)
        );

        let mut duplicate = artifacts();
        duplicate.push(duplicate[0].clone());
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &duplicate,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 10,
                },
            ),
            Err(AdapterError::DuplicateGridArtifact)
        );
    }

    #[test]
    fn candidate_reference_observation_mask_or_weight_drift_is_rejected() {
        let mut supplied = artifacts();
        supplied[0].reference.field.observation[0] = Some(0.0);
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &supplied,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 11,
                },
            ),
            Err(AdapterError::PairSupportMismatch)
        );

        let mut supplied = artifacts();
        supplied[0].reference.field.valid_mask[0] = false;
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &supplied,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 12,
                },
            ),
            Err(AdapterError::PairSupportMismatch)
        );

        let mut supplied = artifacts();
        supplied[0].reference.field.cell_weights[0] = 2.0;
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &supplied,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 13,
                },
            ),
            Err(AdapterError::PairSupportMismatch)
        );
    }

    #[test]
    fn forecast_or_verification_digest_mismatch_is_rejected() {
        let mut supplied = artifacts();
        supplied[0].candidate.artifact_sha256 = DIGEST_B.into();
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &supplied,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 14,
                },
            ),
            Err(AdapterError::ForecastArtifactMismatch)
        );

        let mut supplied = artifacts();
        supplied[0].verification.artifact_sha256 = DIGEST_D.into();
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &supplied,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 15,
                },
            ),
            Err(AdapterError::VerificationArtifactMismatch)
        );
    }

    #[test]
    fn initialization_lead_and_spatial_support_mismatch_are_rejected() {
        let mut supplied = artifacts();
        supplied[0].candidate.initialized_at = 99;
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &supplied,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 16,
                },
            ),
            Err(AdapterError::ForecastArtifactMismatch)
        );

        let mut supplied = artifacts();
        supplied[0].reference.lead_seconds = 9;
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &supplied,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 17,
                },
            ),
            Err(AdapterError::ForecastTimeMismatch)
        );

        let mut supplied = artifacts();
        supplied[0].reference.grid_sha256 = DIGEST_B.into();
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &supplied,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 18,
                },
            ),
            Err(AdapterError::SpatialSupportMismatch)
        );
    }

    #[test]
    fn score_unit_must_match_common_source_manifest() {
        let mut score_config = config();
        score_config.unit = "C".into();
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &artifacts(),
                &event_blocks(),
                &score_config,
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 19,
                },
            ),
            Err(AdapterError::ScoreUnitMismatch)
        );
    }

    #[test]
    fn metric_identity_and_configuration_digest_are_enforced() {
        let mut comparison = manifest();
        comparison.metric.metric_id = "rmse".into();
        assert_eq!(
            score_paired_manifest(
                &comparison,
                &artifacts(),
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 21,
                },
            ),
            Err(AdapterError::MetricContractMismatch)
        );

        let mut comparison = manifest();
        comparison.metric.configuration_sha256 = DIGEST_F.into();
        assert_eq!(
            score_paired_manifest(
                &comparison,
                &artifacts(),
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 22,
                },
            ),
            Err(AdapterError::MetricContractMismatch)
        );
    }

    #[test]
    fn event_block_manifest_digest_and_exact_case_coverage_are_enforced() {
        let mut blocks = event_blocks();
        blocks.assignments[0].event_block_id = "different-storm".into();
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &artifacts(),
                &blocks,
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 23,
                },
            ),
            Err(AdapterError::EventBlockMappingMismatch)
        );

        let mut blocks = event_blocks();
        blocks.assignments.pop();
        blocks.mapping_sha256 = event_block_manifest_sha256(&blocks)
            .expect("modified mapping should still hash");
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &artifacts(),
                &blocks,
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 24,
                },
            ),
            Err(AdapterError::EventBlockMappingMismatch)
        );
    }

    #[test]
    fn runtime_ensemble_member_counts_match_model_output_manifests() {
        let mut supplied = artifacts();
        supplied[0].candidate.field.forecast_members.pop();
        assert_eq!(
            score_paired_manifest(
                &manifest(),
                &supplied,
                &event_blocks(),
                &config(),
                &BlockBootstrapConfig {
                    replicates: 128,
                    seed: 25,
                },
            ),
            Err(AdapterError::EnsembleMemberCountMismatch)
        );
    }

    #[test]
    fn config_and_event_block_hashes_are_stable() {
        assert_eq!(fss_config_sha256(&config()), fss_config_sha256(&config()));
        assert_eq!(
            event_block_manifest_sha256(&event_blocks()).expect("map digest"),
            event_blocks().mapping_sha256
        );
    }

    #[test]
    fn deterministic_output_preserves_source_and_case_provenance() {
        let comparison = manifest();
        let bootstrap = BlockBootstrapConfig {
            replicates: 128,
            seed: 20,
        };
        let first = score_paired_manifest(&comparison, &artifacts(), &event_blocks(), &config(), &bootstrap)
            .expect("first score");
        let second = score_paired_manifest(&comparison, &artifacts(), &event_blocks(), &config(), &bootstrap)
            .expect("second score");

        assert_eq!(first, second);
        assert!(first.to_csv().starts_with(
            "case_id,comparison_id,case_frame_id,verification_source,verification_version,"
        ));
        assert!(first.to_csv().contains("case-a"));
        assert!(first.to_csv().contains("case-b"));
    }
}

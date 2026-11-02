//! Synthetic reference-observation error DGP and probabilistic FSS robustness.
//!
//! Forecast fields are frozen. Only the verifying observation is perturbed.
//! Scores against the latent field are oracle diagnostics; operational claims
//! must use the observed reference and state its uncertainty. Unknown severity
//! parameters produce sensitivity envelopes, never a silently corrected score.

use std::collections::{BTreeMap, BTreeSet};

use gcpn_probabilistic_fss::{
    random_reference, evaluate, BoundaryPolicy, FssCase, FssConfig, FssScore,
    MissingPolicy, RandomBaselineConfig, ThresholdOperator,
};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ErrorFamily {
    ZeroError,
    IndependentNoise,
    SpatiallyCorrelated,
    AnisotropicAdvected,
    SharedCommonMode,
    ThresholdHeteroscedastic,
    EventDependentMissingness,
    UpperCensoring,
    PointToCellRepresentativeness,
    ConstantBias,
    TemporalCorrelation,
    ThresholdBoundaryShift,
}

impl ErrorFamily {
    pub const ALL: [Self; 12] = [
        Self::ZeroError,
        Self::IndependentNoise,
        Self::SpatiallyCorrelated,
        Self::AnisotropicAdvected,
        Self::SharedCommonMode,
        Self::ThresholdHeteroscedastic,
        Self::EventDependentMissingness,
        Self::UpperCensoring,
        Self::PointToCellRepresentativeness,
        Self::ConstantBias,
        Self::TemporalCorrelation,
        Self::ThresholdBoundaryShift,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::ZeroError => "zero_error",
            Self::IndependentNoise => "independent_noise",
            Self::SpatiallyCorrelated => "spatially_correlated",
            Self::AnisotropicAdvected => "anisotropic_advected",
            Self::SharedCommonMode => "shared_common_mode",
            Self::ThresholdHeteroscedastic => "threshold_heteroscedastic",
            Self::EventDependentMissingness => "event_dependent_missingness",
            Self::UpperCensoring => "upper_censoring",
            Self::PointToCellRepresentativeness => "point_to_cell_representativeness",
            Self::ConstantBias => "constant_bias",
            Self::TemporalCorrelation => "temporal_correlation",
            Self::ThresholdBoundaryShift => "threshold_boundary_shift",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum ParameterKnowledge {
    Known,
    UnknownRange { lower: f64, upper: f64 },
}

#[derive(Clone, Debug, PartialEq)]
pub struct ObservationErrorDgp {
    pub family: ErrorFamily,
    /// Family-specific severity: standard deviation, censoring amount,
    /// missing probability, constant bias, or threshold shift.
    pub severity: f64,
    /// Kernel radius for spatial and temporal correlation, in grid cells.
    pub correlation_radius: usize,
    pub parameter_knowledge: ParameterKnowledge,
}

impl ObservationErrorDgp {
    pub fn validate(&self, rows: usize, columns: usize) -> Result<(), DgpError> {
        if !self.severity.is_finite() || self.severity < 0.0 {
            return Err(DgpError::InvalidSeverity);
        }
        if matches!(
            self.family,
            ErrorFamily::SpatiallyCorrelated
                | ErrorFamily::AnisotropicAdvected
                | ErrorFamily::TemporalCorrelation
        ) && (self.correlation_radius == 0
            || self.correlation_radius > rows.max(columns).min(32))
        {
            return Err(DgpError::InvalidCorrelationRadius);
        }
        if self.family == ErrorFamily::EventDependentMissingness && self.severity > 0.95 {
            return Err(DgpError::InvalidMissingProbability);
        }
        if let ParameterKnowledge::UnknownRange { lower, upper } = &self.parameter_knowledge {
            if !lower.is_finite() || !upper.is_finite() || *lower < 0.0 || *lower > *upper {
                return Err(DgpError::InvalidUnknownRange);
            }
        }
        Ok(())
    }

    fn severity_values(&self) -> Vec<f64> {
        match &self.parameter_knowledge {
            ParameterKnowledge::Known => vec![self.severity],
            ParameterKnowledge::UnknownRange { lower, upper } => {
                let mut values = vec![*lower, (*lower + *upper) / 2.0, *upper];
                values.sort_by(f64::total_cmp);
                values.dedup_by(|left, right| left.to_bits() == right.to_bits());
                values
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SpatialFixture {
    pub case_id: String,
    pub rows: usize,
    pub columns: usize,
    pub threshold: f64,
    pub unit: String,
    pub latent_truth: Vec<f64>,
    pub candidate_forecast: Vec<f64>,
    pub reference_forecast: Vec<f64>,
    pub cell_weights: Vec<f64>,
    pub valid_mask: Vec<bool>,
}

impl SpatialFixture {
    pub fn validate(&self) -> Result<(), DgpError> {
        if !valid_identifier(&self.case_id) {
            return Err(DgpError::InvalidCaseId);
        }
        if self.rows < 2 || self.columns < 2 {
            return Err(DgpError::InvalidDimensions);
        }
        let cells = self
            .rows
            .checked_mul(self.columns)
            .ok_or(DgpError::DimensionOverflow)?;
        if self.latent_truth.len() != cells
            || self.candidate_forecast.len() != cells
            || self.reference_forecast.len() != cells
            || self.cell_weights.len() != cells
            || self.valid_mask.len() != cells
        {
            return Err(DgpError::VectorLengthMismatch);
        }
        if !self.threshold.is_finite() || self.unit.trim().is_empty() {
            return Err(DgpError::InvalidFixtureMetadata);
        }
        if self
            .latent_truth
            .iter()
            .chain(&self.candidate_forecast)
            .chain(&self.reference_forecast)
            .any(|value| !value.is_finite())
        {
            return Err(DgpError::NonFiniteField);
        }
        if self.cell_weights.iter().enumerate().any(|(index, weight)| {
            !weight.is_finite()
                || *weight < 0.0
                || (self.valid_mask[index] && *weight <= 0.0)
        }) {
            return Err(DgpError::InvalidWeights);
        }
        if self.valid_mask.iter().filter(|value| **value).count() < 4 {
            return Err(DgpError::InsufficientValidCells);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DgpTrial {
    pub seed: u64,
    /// Trials sharing an event-block ID are temporally dependent. Confidence
    /// intervals are formed over these blocks, never over grid cells.
    pub event_block_id: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AppliedObservation {
    pub values: Vec<Option<f64>>,
    pub missing_cells: usize,
    pub error_lineage_id: String,
    pub mean_error: f64,
    pub error_rms: f64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DgpError {
    InvalidSeverity,
    InvalidCorrelationRadius,
    InvalidMissingProbability,
    InvalidUnknownRange,
    InvalidCaseId,
    InvalidDimensions,
    DimensionOverflow,
    VectorLengthMismatch,
    InvalidFixtureMetadata,
    NonFiniteField,
    InvalidWeights,
    InsufficientValidCells,
    InvalidTrialSet,
    InsufficientIndependentEventBlocks,
    InvalidWindowWidth,
    ScoringFailure(String),
}

impl std::fmt::Display for DgpError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidSeverity => write!(formatter, "severity must be finite and non-negative"),
            Self::InvalidCorrelationRadius => write!(formatter, "correlation radius is outside the allowed range"),
            Self::InvalidMissingProbability => write!(formatter, "missing probability must be <= 0.95"),
            Self::InvalidUnknownRange => write!(formatter, "unknown parameter interval is invalid"),
            Self::InvalidCaseId => write!(formatter, "case ID is not canonical"),
            Self::InvalidDimensions => write!(formatter, "grid dimensions must be at least 2 by 2"),
            Self::DimensionOverflow => write!(formatter, "grid dimensions overflow"),
            Self::VectorLengthMismatch => write!(formatter, "field or support vector length mismatch"),
            Self::InvalidFixtureMetadata => write!(formatter, "fixture threshold or unit is invalid"),
            Self::NonFiniteField => write!(formatter, "field contains a non-finite value"),
            Self::InvalidWeights => write!(formatter, "cell weights are invalid"),
            Self::InsufficientValidCells => write!(formatter, "too few valid cells"),
            Self::InvalidTrialSet => write!(formatter, "trial IDs or seeds are invalid or duplicated"),
            Self::InsufficientIndependentEventBlocks => write!(formatter, "at least two independent event blocks are required"),
            Self::InvalidWindowWidth => write!(formatter, "window widths must be positive odd values"),
            Self::ScoringFailure(message) => write!(formatter, "pFSS scoring failed: {message}"),
        }
    }
}

impl std::error::Error for DgpError {}

#[derive(Clone, Debug, PartialEq)]
pub struct ScaleTrial {
    pub seed: u64,
    pub event_block_id: String,
    pub error_lineage_id: String,
    pub mean_observation_error: f64,
    pub observation_error_rms: f64,
    pub window_width: usize,
    pub latent_candidate_p_fss: f64,
    pub latent_reference_p_fss: f64,
    pub observed_candidate_p_fss: f64,
    pub observed_reference_p_fss: f64,
    pub latent_random_null_p95: f64,
    pub observed_random_null_p95: f64,
    pub observed_event_frequency: f64,
    pub missing_cells: usize,
    pub rank_reversed: bool,
    pub candidate_exceeds_observed_null: bool,
    pub reference_exceeds_observed_null: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ScaleAggregate {
    pub window_width: usize,
    pub trials: usize,
    pub independent_event_blocks: usize,
    pub rank_reversal_blocks: usize,
    pub rank_reversal_probability: f64,
    pub rank_reversal_ci_lower: f64,
    pub rank_reversal_ci_upper: f64,
    pub mean_latent_candidate_minus_reference: f64,
    pub mean_observed_candidate_minus_reference: f64,
    pub mean_observation_event_frequency: f64,
    pub mean_missing_cells: f64,
    pub candidate_observed_null_exceedance_rate: f64,
    pub reference_observed_null_exceedance_rate: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SeverityReport {
    pub family: ErrorFamily,
    pub severity: f64,
    pub parameter_knowledge: ParameterKnowledge,
    pub sensitivity_only: bool,
    pub scales: Vec<ScaleAggregate>,
    pub trials: Vec<ScaleTrial>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DgpReport {
    pub family: ErrorFamily,
    pub parameter_knowledge: ParameterKnowledge,
    pub independent_event_blocks: usize,
    pub severity_reports: Vec<SeverityReport>,
}

impl DgpReport {
    pub fn summary_line(&self) -> String {
        format!(
            "family={} parameter_knowledge={} independent_event_blocks={} severity_scenarios={}",
            self.family.label(),
            match &self.parameter_knowledge {
                ParameterKnowledge::Known => "known",
                ParameterKnowledge::UnknownRange { .. } => "unknown_range",
            },
            self.independent_event_blocks,
            self.severity_reports.len(),
        )
    }

    pub fn to_trial_csv(&self) -> String {
        let mut output = String::from(
            "family,severity,parameter_knowledge,seed,event_block_id,error_lineage_id,window_width,latent_candidate_p_fss,latent_reference_p_fss,observed_candidate_p_fss,observed_reference_p_fss,latent_random_null_p95,observed_random_null_p95,observed_event_frequency,missing_cells,mean_observation_error,observation_error_rms,rank_reversed,candidate_exceeds_observed_null,reference_exceeds_observed_null\n",
        );
        for severity in &self.severity_reports {
            let knowledge = match &severity.parameter_knowledge {
                ParameterKnowledge::Known => "known",
                ParameterKnowledge::UnknownRange { .. } => "unknown_range",
            };
            for trial in &severity.trials {
                output.push_str(&format!(
                    "{},{},{},{},{},{},{},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{},{:.8},{:.8},{},{},{}\n",
                    severity.family.label(),
                    severity.severity,
                    knowledge,
                    trial.seed,
                    trial.event_block_id,
                    trial.error_lineage_id,
                    trial.window_width,
                    trial.latent_candidate_p_fss,
                    trial.latent_reference_p_fss,
                    trial.observed_candidate_p_fss,
                    trial.observed_reference_p_fss,
                    trial.latent_random_null_p95,
                    trial.observed_random_null_p95,
                    trial.observed_event_frequency,
                    trial.missing_cells,
                    trial.mean_observation_error,
                    trial.observation_error_rms,
                    trial.rank_reversed,
                    trial.candidate_exceeds_observed_null,
                    trial.reference_exceeds_observed_null,
                ));
            }
        }
        output
    }

    pub fn to_csv(&self) -> String {
        let mut output = String::from(
            "family,severity,parameter_knowledge,window_width,trials,event_blocks,rank_reversal_blocks,rank_reversal_probability,rank_reversal_ci_lower,rank_reversal_ci_upper,latent_delta,observed_delta,observed_event_frequency,missing_cells,candidate_observed_null_exceedance_rate,reference_observed_null_exceedance_rate,sensitivity_only\n",
        );
        for severity in &self.severity_reports {
            let knowledge = match &severity.parameter_knowledge {
                ParameterKnowledge::Known => "known",
                ParameterKnowledge::UnknownRange { .. } => "unknown_range",
            };
            for scale in &severity.scales {
                output.push_str(&format!(
                    "{},{},{},{},{},{},{},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.4},{:.8},{:.8},{}\n",
                    severity.family.label(),
                    severity.severity,
                    knowledge,
                    scale.window_width,
                    scale.trials,
                    scale.independent_event_blocks,
                    scale.rank_reversal_blocks,
                    scale.rank_reversal_probability,
                    scale.rank_reversal_ci_lower,
                    scale.rank_reversal_ci_upper,
                    scale.mean_latent_candidate_minus_reference,
                    scale.mean_observed_candidate_minus_reference,
                    scale.mean_observation_event_frequency,
                    scale.mean_missing_cells,
                    scale.candidate_observed_null_exceedance_rate,
                    scale.reference_observed_null_exceedance_rate,
                    severity.sensitivity_only,
                ));
            }
        }
        output
    }
}

pub fn apply_observation_error(
    fixture: &SpatialFixture,
    dgp: &ObservationErrorDgp,
    trial: &DgpTrial,
) -> Result<AppliedObservation, DgpError> {
    fixture.validate()?;
    dgp.validate(fixture.rows, fixture.columns)?;
    if !valid_identifier(&trial.event_block_id) {
        return Err(DgpError::InvalidTrialSet);
    }

    let cells = fixture.rows * fixture.columns;
    let block_seed = stable_id_hash(&trial.event_block_id);
    let mut local_rng = SplitMix64::new(trial.seed ^ 0x9e37_79b9_7f4a_7c15);
    let independent = standardized_field(
        (0..cells).map(|_| local_rng.signed()).collect(),
    );
    let spatially_correlated = || {
        correlated_field(
            fixture.rows,
            fixture.columns,
            trial.seed,
            dgp.correlation_radius.max(1),
            dgp.correlation_radius.max(1),
        )
    };
    let temporally_correlated = || {
        correlated_field(
            fixture.rows,
            fixture.columns,
            block_seed,
            dgp.correlation_radius.max(1),
            dgp.correlation_radius.max(1),
        )
    };

    let error_lineage_id = match dgp.family {
        ErrorFamily::SharedCommonMode | ErrorFamily::TemporalCorrelation => {
            format!("shared-error-{:016x}", block_seed)
        }
        _ => format!("{}-seed-{:016x}", dgp.family.label(), trial.seed),
    };

    let mut error = vec![0.0; cells];
    let mut missing = vec![false; cells];
    let mut rng = SplitMix64::new(trial.seed ^ block_seed.rotate_left(17));
    let mut block_rng = SplitMix64::new(block_seed ^ 0x51ed_270b_d7a3_9c41);

    match dgp.family {
        ErrorFamily::ZeroError => {}
        ErrorFamily::IndependentNoise => {
            for index in 0..cells {
                error[index] = dgp.severity * independent[index];
            }
        }
        ErrorFamily::SpatiallyCorrelated => {
            let field = spatially_correlated();
            for index in 0..cells {
                error[index] = dgp.severity * field[index];
            }
        }
        ErrorFamily::AnisotropicAdvected => {
            let field = correlated_field(
                fixture.rows,
                fixture.columns,
                trial.seed,
                (dgp.correlation_radius / 2).max(1),
                dgp.correlation_radius.saturating_mul(2).max(1),
            );
            for index in 0..cells {
                error[index] = dgp.severity * field[index];
            }
        }
        ErrorFamily::SharedCommonMode => {
            let common = dgp.severity * block_rng.signed();
            for index in 0..cells {
                error[index] = common + dgp.severity * 0.2 * independent[index];
            }
        }
        ErrorFamily::ThresholdHeteroscedastic => {
            for index in 0..cells {
                let distance = (fixture.latent_truth[index] - fixture.threshold).abs();
                let multiplier = 1.0 + 2.0 * (-distance / 0.15).exp();
                error[index] = dgp.severity * multiplier * independent[index];
            }
        }
        ErrorFamily::EventDependentMissingness => {
            for index in 0..cells {
                let probability = if fixture.latent_truth[index] >= fixture.threshold {
                    (dgp.severity * 2.0).min(0.95)
                } else {
                    (dgp.severity * 0.2).min(0.95)
                };
                missing[index] = rng.next_f64() < probability;
            }
        }
        ErrorFamily::UpperCensoring => {
            for index in 0..cells {
                if fixture.latent_truth[index] >= fixture.threshold {
                    let cap = fixture.threshold - dgp.severity;
                    error[index] = cap - fixture.latent_truth[index];
                }
            }
        }
        ErrorFamily::PointToCellRepresentativeness => {
            let distance = dgp.severity.round() as isize;
            let dx = if trial.seed.is_multiple_of(2) { distance } else { -distance };
            let dy = if (trial.seed >> 4).is_multiple_of(2) { distance } else { -distance };
            for row in 0..fixture.rows {
                for column in 0..fixture.columns {
                    let source_row = wrap_index(row as isize + dy, fixture.rows);
                    let source_column = wrap_index(column as isize + dx, fixture.columns);
                    let index = row * fixture.columns + column;
                    let source = source_row * fixture.columns + source_column;
                    error[index] = fixture.latent_truth[source] - fixture.latent_truth[index];
                }
            }
        }
        ErrorFamily::ConstantBias => {
            error.fill(dgp.severity);
        }
        ErrorFamily::TemporalCorrelation => {
            let shared = temporally_correlated();
            for index in 0..cells {
                error[index] = dgp.severity
                    * (0.8 * shared[index] + 0.6 * independent[index]);
            }
        }
        ErrorFamily::ThresholdBoundaryShift => {}
    }

    let mut values = Vec::with_capacity(cells);
    let mut error_sum = 0.0;
    let mut squared_error_sum = 0.0;
    let mut observed_count = 0usize;
    let missing_cells = missing.iter().filter(|value| **value).count();

    for index in 0..cells {
        if missing[index] || !fixture.valid_mask[index] {
            values.push(None);
            continue;
        }
        let observed = fixture.latent_truth[index] + error[index];
        error_sum += error[index];
        squared_error_sum += error[index] * error[index];
        observed_count += 1;
        values.push(Some(observed));
    }

    let mean_error = if observed_count == 0 {
        0.0
    } else {
        error_sum / observed_count as f64
    };
    let error_rms = if observed_count == 0 {
        0.0
    } else {
        (squared_error_sum / observed_count as f64).sqrt()
    };

    Ok(AppliedObservation {
        values,
        missing_cells,
        error_lineage_id,
        mean_error,
        error_rms,
        error_lineage_id,
    })
}

pub fn run_benchmark(
    fixture: &SpatialFixture,
    dgp: &ObservationErrorDgp,
    trials: &[DgpTrial],
    window_widths: &[usize],
    random_draws: usize,
) -> Result<DgpReport, DgpError> {
    fixture.validate()?;
    dgp.validate(fixture.rows, fixture.columns)?;
    validate_trials(trials)?;
    if window_widths.is_empty()
        || window_widths
            .iter()
            .any(|width| *width == 0 || width.is_multiple_of(2))
    {
        return Err(DgpError::InvalidWindowWidth);
    }
    if random_draws < 32 {
        return Err(DgpError::InvalidTrialSet);
    }
    let block_count = trials
        .iter()
        .map(|trial| trial.event_block_id.as_str())
        .collect::<BTreeSet<_>>()
        .len();
    if block_count < 2 {
        return Err(DgpError::InsufficientIndependentEventBlocks);
    }

    let mut severity_reports = Vec::new();
    for severity in dgp.severity_values() {
        let mut active_dgp = dgp.clone();
        active_dgp.severity = severity;
        active_dgp.parameter_knowledge = ParameterKnowledge::Known;
        let mut all_trials = Vec::new();

        for trial in trials {
            let observation = apply_observation_error(fixture, &active_dgp, trial)?;
            for window_width in window_widths {
                let threshold = if active_dgp.family == ErrorFamily::ThresholdBoundaryShift {
                    let sign = if trial.seed.is_multiple_of(2) { 1.0 } else { -1.0 };
                    fixture.threshold + sign * severity
                } else {
                    fixture.threshold
                };
                let config = FssConfig {
                    threshold,
                    threshold_operator: ThresholdOperator::GreaterThanOrEqual,
                    unit: fixture.unit.clone(),
                    window_width_cells: *window_width,
                    boundary_policy: BoundaryPolicy::TruncateAtDomain,
                    missing_policy: MissingPolicy::PairwiseComplete,
                };

                let latent_candidate = qualified_p_fss(score_case(
                    fixture,
                    &fixture.candidate_forecast,
                    fixture.latent_truth.iter().copied().map(Some).collect(),
                    *window_width,
                    threshold,
                )?)?;
                let latent_reference = qualified_p_fss(score_case(
                    fixture,
                    &fixture.reference_forecast,
                    fixture.latent_truth.iter().copied().map(Some).collect(),
                    *window_width,
                    threshold,
                )?)?;
                let observed_candidate = qualified_p_fss(score_case(
                    fixture,
                    &fixture.candidate_forecast,
                    observation.values.clone(),
                    *window_width,
                    threshold,
                )?)?;
                let observed_reference = qualified_p_fss(score_case(
                    fixture,
                    &fixture.reference_forecast,
                    observation.values.clone(),
                    *window_width,
                    threshold,
                )?)?;

                let random_config = RandomBaselineConfig {
                    draws: random_draws,
                    seed: trial.seed ^ (*window_width as u64).rotate_left(11),
                };
                let observed_case = make_case(
                    fixture,
                    "observed-random-null",
                    &fixture.candidate_forecast,
                    observation.values.clone(),
                );
                let latent_case = make_case(
                    fixture,
                    "latent-random-null",
                    &fixture.candidate_forecast,
                    fixture.latent_truth.iter().copied().map(Some).collect(),
                );
                let observed_null = random_reference(&observed_case, &config, &random_config)
                    .map_err(|error| DgpError::ScoringFailure(error.to_string()))?;
                let latent_null = random_reference(
                    &latent_case,
                    &config,
                    &RandomBaselineConfig {
                        draws: random_draws,
                        seed: random_config.seed ^ 0x51ed_270b,
                    },
                )
                .map_err(|error| DgpError::ScoringFailure(error.to_string()))?;

                let latent_delta = latent_candidate.p_fss - latent_reference.p_fss;
                let observed_delta = observed_candidate.p_fss - observed_reference.p_fss;
                let rank_reversed = latent_delta.signum() != 0.0
                    && observed_delta.signum() != 0.0
                    && latent_delta.signum() != observed_delta.signum();
                all_trials.push(ScaleTrial {
                    seed: trial.seed,
                    event_block_id: trial.event_block_id.clone(),
                    error_lineage_id: observation.error_lineage_id.clone(),
                    mean_observation_error: observation.mean_error,
                    observation_error_rms: observation.error_rms,
                    window_width: *window_width,
                    latent_candidate_p_fss: latent_candidate.p_fss,
                    latent_reference_p_fss: latent_reference.p_fss,
                    observed_candidate_p_fss: observed_candidate.p_fss,
                    observed_reference_p_fss: observed_reference.p_fss,
                    latent_random_null_p95: latent_null.null_p95,
                    observed_random_null_p95: observed_null.null_p95,
                    observed_event_frequency: observed_candidate.observation_event_frequency,
                    missing_cells: observation.missing_cells,
                    rank_reversed,
                    candidate_exceeds_observed_null: observed_candidate.p_fss
                        > observed_null.null_p95,
                    reference_exceeds_observed_null: observed_reference.p_fss
                        > observed_null.null_p95,
                });
            }
        }

        let scales = window_widths
            .iter()
            .map(|window| aggregate_scale(&all_trials, *window))
            .collect::<Result<Vec<_>, _>>()?;
        severity_reports.push(SeverityReport {
            family: dgp.family,
            severity,
            parameter_knowledge: dgp.parameter_knowledge.clone(),
            sensitivity_only: matches!(
                &dgp.parameter_knowledge,
                ParameterKnowledge::UnknownRange { .. }
            ),
            scales,
            trials: all_trials,
        });
    }

    Ok(DgpReport {
        family: dgp.family,
        parameter_knowledge: dgp.parameter_knowledge.clone(),
        independent_event_blocks: block_count,
        severity_reports,
    })
}

fn qualified_p_fss(score: FssScore) -> Result<f64, DgpError> {
    score
        .p_fss
        .ok_or_else(|| DgpError::ScoringFailure("degenerate no-event pFSS case".into()))
}

fn score_case(
    fixture: &SpatialFixture,
    forecast: &[f64],
    observation: Vec<Option<f64>>,
    window_width: usize,
    threshold: f64,
) -> Result<FssScore, DgpError> {
    let config = FssConfig {
        threshold,
        threshold_operator: ThresholdOperator::GreaterThanOrEqual,
        unit: fixture.unit.clone(),
        window_width_cells: window_width,
        boundary_policy: BoundaryPolicy::TruncateAtDomain,
        missing_policy: MissingPolicy::PairwiseComplete,
    };
    let case = make_case(fixture, "score-case", forecast, observation);
    evaluate(&case, &config).map_err(|error| DgpError::ScoringFailure(error.to_string()))
}

fn make_case(
    fixture: &SpatialFixture,
    case_id: &str,
    forecast: &[f64],
    observation: Vec<Option<f64>>,
) -> FssCase {
    FssCase {
        case_id: case_id.into(),
        rows: fixture.rows,
        columns: fixture.columns,
        forecast_members: vec![forecast.iter().copied().map(Some).collect()],
        observation,
        cell_weights: fixture.cell_weights.clone(),
        valid_mask: fixture.valid_mask.clone(),
    }
}

fn aggregate_scale(trials: &[ScaleTrial], window_width: usize) -> Result<ScaleAggregate, DgpError> {
    let matching: Vec<_> = trials
        .iter()
        .filter(|trial| trial.window_width == window_width)
        .collect();
    if matching.is_empty() {
        return Err(DgpError::InvalidTrialSet);
    }

    let mut block_reversals: BTreeMap<&str, (usize, usize)> = BTreeMap::new();
    for trial in &matching {
        let entry = block_reversals
            .entry(trial.event_block_id.as_str())
            .or_insert((0, 0));
        entry.0 += usize::from(trial.rank_reversed);
        entry.1 += 1;
    }
    if block_reversals.len() < 2 {
        return Err(DgpError::InsufficientIndependentEventBlocks);
    }
    let block_rates: Vec<f64> = block_reversals
        .values()
        .map(|(reversed, total)| *reversed as f64 / *total as f64)
        .collect();
    let probability = block_rates.iter().sum::<f64>() / block_rates.len() as f64;
    let sample_variance = block_rates
        .iter()
        .map(|rate| (rate - probability).powi(2))
        .sum::<f64>()
        / (block_rates.len() - 1) as f64;
    let standard_error = (sample_variance / block_rates.len() as f64).sqrt();
    let (ci_lower, ci_upper) = (
        (probability - 1.96 * standard_error).max(0.0),
        (probability + 1.96 * standard_error).min(1.0),
    );

    let mean = |select: fn(&ScaleTrial) -> f64| {
        matching.iter().map(|trial| select(trial)).sum::<f64>() / matching.len() as f64
    };
    let rank_reversal_blocks = block_rates.iter().filter(|rate| **rate > 0.0).count();

    Ok(ScaleAggregate {
        window_width,
        trials: matching.len(),
        independent_event_blocks: block_rates.len(),
        rank_reversal_blocks,
        rank_reversal_probability: probability,
        rank_reversal_ci_lower: ci_lower,
        rank_reversal_ci_upper: ci_upper,
        mean_latent_candidate_minus_reference: mean(|trial| {
            trial.latent_candidate_p_fss - trial.latent_reference_p_fss
        }),
        mean_observed_candidate_minus_reference: mean(|trial| {
            trial.observed_candidate_p_fss - trial.observed_reference_p_fss
        }),
        mean_observation_event_frequency: mean(|trial| trial.observed_event_frequency),
        mean_missing_cells: mean(|trial| trial.missing_cells as f64),
        candidate_observed_null_exceedance_rate: mean(|trial| {
            if trial.candidate_exceeds_observed_null { 1.0 } else { 0.0 }
        }),
        reference_observed_null_exceedance_rate: mean(|trial| {
            if trial.reference_exceeds_observed_null { 1.0 } else { 0.0 }
        }),
    })
}

fn validate_trials(trials: &[DgpTrial]) -> Result<(), DgpError> {
    if trials.len() < 2 {
        return Err(DgpError::InvalidTrialSet);
    }
    let mut seeds = BTreeSet::new();
    let mut identifiers = BTreeSet::new();
    for trial in trials {
        if !valid_identifier(&trial.event_block_id)
            || !seeds.insert(trial.seed)
            || !identifiers.insert((trial.seed, trial.event_block_id.as_str()))
        {
            return Err(DgpError::InvalidTrialSet);
        }
    }
    Ok(())
}

fn correlated_field(
    rows: usize,
    columns: usize,
    seed: u64,
    radius_y: usize,
    radius_x: usize,
) -> Vec<f64> {
    let cells = rows * columns;
    let mut rng = SplitMix64::new(seed);
    let innovations: Vec<f64> = (0..cells).map(|_| rng.signed()).collect();
    let mut field = vec![0.0; cells];
    let mut weights = Vec::new();
    for dy in -(radius_y as isize)..=radius_y as isize {
        for dx in -(radius_x as isize)..=radius_x as isize {
            let ry = radius_y.max(1) as f64;
            let rx = radius_x.max(1) as f64;
            let distance = (dy as f64 / ry).powi(2) + (dx as f64 / rx).powi(2);
            weights.push((dy, dx, (-0.5 * distance).exp()));
        }
    }

    for row in 0..rows {
        for column in 0..columns {
            let mut sum = 0.0;
            let mut weight_sum = 0.0;
            for (dy, dx, weight) in &weights {
                let source_row = wrap_index(row as isize + dy, rows);
                let source_column = wrap_index(column as isize + dx, columns);
                sum += innovations[source_row * columns + source_column] * weight;
                weight_sum += weight;
            }
            field[row * columns + column] = sum / weight_sum;
        }
    }
    standardized_field(field)
}

fn standardized_field(mut values: Vec<f64>) -> Vec<f64> {
    if values.is_empty() {
        return values;
    }
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    for value in &mut values {
        *value -= mean;
    }
    let rms = (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt();
    if rms > 0.0 {
        for value in &mut values {
            *value /= rms;
        }
    }
    values
}

fn wrap_index(index: isize, size: usize) -> usize {
    index.rem_euclid(size as isize) as usize
}

fn stable_id_hash(value: &str) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for byte in value.bytes() {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"-_.:".contains(&byte))
}

struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut value = self.state;
        value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        value ^ (value >> 31)
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / ((1_u64 << 53) as f64))
    }

    fn signed(&mut self) -> f64 {
        (2.0 * self.next_f64()) - 1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> SpatialFixture {
        let rows = 12;
        let columns = 12;
        let mut latent = vec![0.0; rows * columns];
        for row in 0..rows {
            for column in 0..columns {
                let first = (3..=6).contains(&row) && (2..=5).contains(&column);
                let second = (7..=9).contains(&row) && (7..=9).contains(&column);
                latent[row * columns + column] = if first || second { 2.0 } else { 0.0 };
            }
        }
        let candidate = latent.clone();
        let mut reference = vec![0.0; rows * columns];
        for row in 0..rows {
            for column in 0..columns {
                let source_column = wrap_index(column as isize - 2, columns);
                reference[row * columns + column] = latent[row * columns + source_column];
            }
        }
        SpatialFixture {
            case_id: "synthetic-event-1".into(),
            rows,
            columns,
            threshold: 1.0,
            unit: "mm/h".into(),
            latent_truth: latent,
            candidate_forecast: candidate,
            reference_forecast: reference,
            cell_weights: vec![1.0; rows * columns],
            valid_mask: vec![true; rows * columns],
        }
    }

    fn dgp(family: ErrorFamily, severity: f64) -> ObservationErrorDgp {
        ObservationErrorDgp {
            family,
            severity,
            correlation_radius: 2,
            parameter_knowledge: ParameterKnowledge::Known,
        }
    }

    fn trials() -> Vec<DgpTrial> {
        (0..8)
            .map(|index| DgpTrial {
                seed: index,
                event_block_id: format!("event-block-{}", index / 2),
            })
            .collect()
    }

    #[test]
    fn zero_error_observation_recovers_latent_field_exactly() {
        let fixture = fixture();
        let result = apply_observation_error(
            &fixture,
            &dgp(ErrorFamily::ZeroError, 0.0),
            &trials()[0],
        )
        .unwrap();
        let observed: Vec<f64> = result.values.into_iter().map(Option::unwrap).collect();
        assert_eq!(observed, fixture.latent_truth);
        assert_eq!(result.missing_cells, 0);
    }

    #[test]
    fn zero_error_scores_match_latent_oracle() {
        let report = run_benchmark(
            &fixture(),
            &dgp(ErrorFamily::ZeroError, 0.0),
            &trials(),
            &[1, 3],
            64,
        )
        .unwrap();
        for row in &report.severity_reports[0].trials {
            assert!((row.latent_candidate_p_fss - row.observed_candidate_p_fss).abs() < 1e-12);
            assert!((row.latent_reference_p_fss - row.observed_reference_p_fss).abs() < 1e-12);
            assert!(!row.rank_reversed);
        }
    }

    #[test]
    fn same_seed_reproduces_spatially_correlated_observations() {
        let fixture = fixture();
        let dgp = dgp(ErrorFamily::SpatiallyCorrelated, 0.5);
        let trial = &trials()[0];
        assert_eq!(
            apply_observation_error(&fixture, &dgp, trial).unwrap(),
            apply_observation_error(&fixture, &dgp, trial).unwrap()
        );
    }

    #[test]
    fn spatial_filter_has_positive_neighbour_correlation() {
        let field = correlated_field(32, 32, 42, 2, 2);
        let mut products = Vec::new();
        for row in 0..32 {
            for column in 0..31 {
                products.push(field[row * 32 + column] * field[row * 32 + column + 1]);
            }
        }
        let mean_product = products.iter().sum::<f64>() / products.len() as f64;
        assert!(mean_product > 0.0);
    }

    #[test]
    fn event_dependent_missingness_preserves_a_typed_missing_count() {
        let result = apply_observation_error(
            &fixture(),
            &dgp(ErrorFamily::EventDependentMissingness, 0.35),
            &trials()[0],
        )
        .unwrap();
        assert!(result.missing_cells > 0);
        assert_eq!(
            result.values.iter().filter(|value| value.is_none()).count(),
            result.missing_cells
        );
    }

    #[test]
    fn spatially_correlated_error_is_trial_specific_within_an_event_block() {
        let fixture = fixture();
        let dgp = dgp(ErrorFamily::SpatiallyCorrelated, 0.5);
        let first = DgpTrial {
            seed: 1,
            event_block_id: "same-event".into(),
        };
        let second = DgpTrial {
            seed: 2,
            event_block_id: "same-event".into(),
        };
        let a = apply_observation_error(&fixture, &dgp, &first).unwrap();
        let b = apply_observation_error(&fixture, &dgp, &second).unwrap();
        assert_ne!(a.error_lineage_id, b.error_lineage_id);
        assert_ne!(a.values, b.values);
    }

    #[test]
    fn temporal_error_lineage_is_shared_within_an_event_block() {
        let fixture = fixture();
        let dgp = dgp(ErrorFamily::TemporalCorrelation, 0.5);
        let first = DgpTrial {
            seed: 1,
            event_block_id: "cyclone-event-a".into(),
        };
        let second = DgpTrial {
            seed: 2,
            event_block_id: "cyclone-event-a".into(),
        };
        let third = DgpTrial {
            seed: 3,
            event_block_id: "cyclone-event-b".into(),
        };
        let a = apply_observation_error(&fixture, &dgp, &first).unwrap();
        let b = apply_observation_error(&fixture, &dgp, &second).unwrap();
        let c = apply_observation_error(&fixture, &dgp, &third).unwrap();
        assert_eq!(a.error_lineage_id, b.error_lineage_id);
        assert_ne!(a.error_lineage_id, c.error_lineage_id);
    }

    #[test]
    fn unknown_severity_is_reported_as_a_sensitivity_envelope() {
        let mut config = dgp(ErrorFamily::SpatiallyCorrelated, 0.3);
        config.parameter_knowledge = ParameterKnowledge::UnknownRange {
            lower: 0.1,
            upper: 0.8,
        };
        let report = run_benchmark(&fixture(), &config, &trials(), &[3], 64).unwrap();
        assert_eq!(report.severity_reports.len(), 3);
        assert!(report.severity_reports.iter().all(|result| result.sensitivity_only));
        assert_eq!(report.severity_reports[0].severity, 0.1);
        assert_eq!(report.severity_reports[2].severity, 0.8);
    }

    #[test]
    fn rank_reversal_uncertainty_uses_event_blocks_not_grid_cells() {
        let report = run_benchmark(
            &fixture(),
            &dgp(ErrorFamily::IndependentNoise, 0.8),
            &trials(),
            &[1],
            64,
        )
        .unwrap();
        let aggregate = &report.severity_reports[0].scales[0];
        assert_eq!(aggregate.trials, 8);
        assert_eq!(aggregate.independent_event_blocks, 4);
        assert!(aggregate.rank_reversal_probability.is_finite());
        assert!(aggregate.rank_reversal_ci_lower <= aggregate.rank_reversal_ci_upper);
    }

    #[test]
    fn observed_and_latent_random_references_are_distinct_estimands() {
        let report = run_benchmark(
            &fixture(),
            &dgp(ErrorFamily::ConstantBias, 0.6),
            &trials(),
            &[3],
            64,
        )
        .unwrap();
        let rows = &report.severity_reports[0].trials;
        assert!(rows.iter().all(|row| row.observed_random_null_p95.is_finite()));
        assert!(rows.iter().all(|row| row.latent_random_null_p95.is_finite()));
    }

    #[test]
    fn temporal_support_is_deterministically_aggregated_by_scale() {
        let report = run_benchmark(
            &fixture(),
            &dgp(ErrorFamily::ThresholdHeteroscedastic, 0.25),
            &trials(),
            &[1, 3, 5],
            64,
        )
        .unwrap();
        assert_eq!(report.severity_reports[0].scales.len(), 3);
        assert_eq!(report.severity_reports[0].scales[0].window_width, 1);
    }

    #[test]
    fn report_serialization_is_deterministic() {
        let report = run_benchmark(
            &fixture(),
            &dgp(ErrorFamily::SharedCommonMode, 0.4),
            &trials(),
            &[1, 3],
            64,
        )
        .unwrap();
        assert_eq!(report.to_csv(), report.to_csv());
        assert!(report.to_csv().starts_with("family,severity,parameter_knowledge,window_width,"));
    }

    #[test]
    fn invalid_unknown_parameter_range_is_rejected() {
        let mut config = dgp(ErrorFamily::IndependentNoise, 0.3);
        config.parameter_knowledge = ParameterKnowledge::UnknownRange {
            lower: 1.0,
            upper: 0.5,
        };
        assert_eq!(
            config.validate(12, 12),
            Err(DgpError::InvalidUnknownRange)
        );
    }
}

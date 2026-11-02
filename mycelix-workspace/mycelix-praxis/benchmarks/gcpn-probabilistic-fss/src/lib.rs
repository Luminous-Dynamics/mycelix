//! Deterministic FSS and probabilistic ensemble FSS (pFSS).
//!
//! The pFSS implementation follows the ensemble/neighbourhood probability
//! formulation in Necker et al. (2024). It is intentionally distinct from
//! member-averaged FSS, ensemble-mean FSS, and ensemble-aggregated FSS.
//!
//! The empirical random-reference procedure samples spatially independent
//! Bernoulli event fields using the observed area-weighted base rate and the
//! exact ensemble size. It is a reproducible Monte Carlo null, not a
//! climatology, persistence forecast, cryptographic RNG, or operational claim.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

const MAX_WINDOW_WIDTH: usize = 2047;
const MIN_RANDOM_DRAWS: usize = 99;
const MAX_RANDOM_DRAWS: usize = 1_000_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ThresholdOperator {
    GreaterThan,
    GreaterThanOrEqual,
}

impl ThresholdOperator {
    pub const fn label(self) -> &'static str {
        match self {
            Self::GreaterThan => "gt",
            Self::GreaterThanOrEqual => "gte",
        }
    }

    pub fn exceeds(self, value: f64, threshold: f64) -> bool {
        match self {
            Self::GreaterThan => value > threshold,
            Self::GreaterThanOrEqual => value >= threshold,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoundaryPolicy {
    TruncateAtDomain,
    RequireFullWindow,
}

impl BoundaryPolicy {
    pub const fn label(self) -> &'static str {
        match self {
            Self::TruncateAtDomain => "truncate_at_domain",
            Self::RequireFullWindow => "require_full_window",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MissingPolicy {
    RejectCase,
    PairwiseComplete,
}

impl MissingPolicy {
    pub const fn label(self) -> &'static str {
        match self {
            Self::RejectCase => "reject_case",
            Self::PairwiseComplete => "pairwise_complete",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct FssConfig {
    pub threshold: f64,
    pub threshold_operator: ThresholdOperator,
    pub unit: String,
    pub window_width_cells: usize,
    pub boundary_policy: BoundaryPolicy,
    pub missing_policy: MissingPolicy,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FssCase {
    pub case_id: String,
    pub rows: usize,
    pub columns: usize,
    /// One vector per ensemble member, each aligned to row-major grid cells.
    pub forecast_members: Vec<Vec<Option<f64>>>,
    pub observation: Vec<Option<f64>>,
    /// Positive area weights for supported cells. Masked cells may have zero.
    pub cell_weights: Vec<f64>,
    pub valid_mask: Vec<bool>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FssStatus {
    Qualified,
    DegenerateNoEvents,
}

impl FssStatus {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Qualified => "qualified",
            Self::DegenerateNoEvents => "degenerate_no_events",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct FssScore {
    pub case_id: String,
    pub threshold: f64,
    pub threshold_operator: ThresholdOperator,
    pub unit: String,
    pub window_width_cells: usize,
    pub boundary_policy: BoundaryPolicy,
    pub missing_policy: MissingPolicy,
    pub ensemble_members: usize,
    pub usable_grid_cells: usize,
    pub valid_mask_cells: usize,
    pub centre_count: usize,
    pub centre_weight_sum: f64,
    pub observation_event_frequency: f64,
    pub mean_forecast_fraction: f64,
    pub mean_observation_fraction: f64,
    pub fractions_brier_score: f64,
    pub worst_fractions_brier_score: f64,
    pub p_fss: Option<f64>,
    pub status: FssStatus,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FssError {
    InvalidCaseId,
    InvalidDimensions,
    DimensionOverflow,
    VectorLengthMismatch,
    NoEnsembleMembers,
    InvalidWindowWidth,
    InvalidThreshold,
    MissingUnit,
    InvalidWeight,
    NonFiniteValue,
    MissingData,
    NoUsableCells,
    NoValidCentres,
    NumericalInvariant,
    InvalidRandomDrawCount,
    RandomBaselineRequiresQualifiedScore,
    InvalidEventBlockId,
    DuplicateCaseId,
    InsufficientIndependentEventBlocks,
    NoQualifiedBootstrapReplicates,
}

impl FssError {
    pub const fn label(self) -> &'static str {
        match self {
            Self::InvalidCaseId => "invalid_case_id",
            Self::InvalidDimensions => "invalid_dimensions",
            Self::DimensionOverflow => "dimension_overflow",
            Self::VectorLengthMismatch => "vector_length_mismatch",
            Self::NoEnsembleMembers => "no_ensemble_members",
            Self::InvalidWindowWidth => "invalid_window_width",
            Self::InvalidThreshold => "invalid_threshold",
            Self::MissingUnit => "missing_unit",
            Self::InvalidWeight => "invalid_weight",
            Self::NonFiniteValue => "non_finite_value",
            Self::MissingData => "missing_data",
            Self::NoUsableCells => "no_usable_cells",
            Self::NoValidCentres => "no_valid_centres",
            Self::NumericalInvariant => "numerical_invariant",
            Self::InvalidRandomDrawCount => "invalid_random_draw_count",
            Self::RandomBaselineRequiresQualifiedScore => {
                "random_baseline_requires_qualified_score"
            }
            Self::InvalidEventBlockId => "invalid_event_block_id",
            Self::DuplicateCaseId => "duplicate_case_id",
            Self::InsufficientIndependentEventBlocks => "insufficient_independent_event_blocks",
            Self::NoQualifiedBootstrapReplicates => "no_qualified_bootstrap_replicates",
        }
    }
}

impl fmt::Display for FssError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{}", self.label())
    }
}

impl std::error::Error for FssError {}

#[derive(Clone, Debug, PartialEq)]
pub struct RandomBaselineConfig {
    pub draws: usize,
    pub seed: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RandomBaselineReport {
    pub case_id: String,
    pub threshold: f64,
    pub threshold_operator: ThresholdOperator,
    pub unit: String,
    pub window_width_cells: usize,
    pub ensemble_members: usize,
    pub draws: usize,
    pub seed: u64,
    pub observed_event_frequency: f64,
    pub null_mean: f64,
    pub null_p95: f64,
    /// One-sided Monte Carlo probability that the random null is >= observed.
    pub upper_tail_p_value: f64,
    pub exceedances: usize,
}

#[derive(Clone, Debug)]
struct PreparedCase {
    usable: Vec<bool>,
    observation_events: Vec<bool>,
    member_events: Vec<Vec<bool>>,
    usable_grid_cells: usize,
    valid_mask_cells: usize,
    observation_event_frequency: f64,
}

impl FssCase {
    pub fn validate(&self, config: &FssConfig) -> Result<(), FssError> {
        if !valid_identifier(&self.case_id) {
            return Err(FssError::InvalidCaseId);
        }
        if self.rows == 0 || self.columns == 0 {
            return Err(FssError::InvalidDimensions);
        }
        let cells = self
            .rows
            .checked_mul(self.columns)
            .ok_or(FssError::DimensionOverflow)?;
        if self.forecast_members.is_empty() {
            return Err(FssError::NoEnsembleMembers);
        }
        if self.observation.len() != cells
            || self.cell_weights.len() != cells
            || self.valid_mask.len() != cells
            || self
                .forecast_members
                .iter()
                .any(|member| member.len() != cells)
        {
            return Err(FssError::VectorLengthMismatch);
        }
        if !config.threshold.is_finite() {
            return Err(FssError::InvalidThreshold);
        }
        if config.unit.trim().is_empty() || config.unit.len() > 64 {
            return Err(FssError::MissingUnit);
        }
        if config.window_width_cells == 0
            || config.window_width_cells.is_multiple_of(2)
            || config.window_width_cells > MAX_WINDOW_WIDTH
        {
            return Err(FssError::InvalidWindowWidth);
        }

        for (index, weight) in self.cell_weights.iter().enumerate() {
            if !weight.is_finite() || *weight < 0.0 {
                return Err(FssError::InvalidWeight);
            }
            if self.valid_mask[index] && *weight <= 0.0 {
                return Err(FssError::InvalidWeight);
            }
        }

        for value in self
            .observation
            .iter()
            .chain(self.forecast_members.iter().flat_map(|member| member.iter()))
            .flatten()
        {
            if !value.is_finite() {
                return Err(FssError::NonFiniteValue);
            }
        }

        if config.missing_policy == MissingPolicy::RejectCase {
            for index in 0..cells {
                if self.valid_mask[index]
                    && (self.observation[index].is_none()
                        || self
                            .forecast_members
                            .iter()
                            .any(|member| member[index].is_none()))
                {
                    return Err(FssError::MissingData);
                }
            }
        }

        Ok(())
    }
}

pub fn evaluate(case: &FssCase, config: &FssConfig) -> Result<FssScore, FssError> {
    case.validate(config)?;
    let prepared = prepare_case(case, config)?;
    evaluate_events(case, config, &prepared, &prepared.member_events)
}

pub fn random_reference(
    case: &FssCase,
    config: &FssConfig,
    random_config: &RandomBaselineConfig,
) -> Result<RandomBaselineReport, FssError> {
    case.validate(config)?;
    if !(MIN_RANDOM_DRAWS..=MAX_RANDOM_DRAWS).contains(&random_config.draws) {
        return Err(FssError::InvalidRandomDrawCount);
    }

    let prepared = prepare_case(case, config)?;
    let observed_score =
        evaluate_events(case, config, &prepared, &prepared.member_events)?;
    let Some(observed_p_fss) = observed_score.p_fss else {
        return Err(FssError::RandomBaselineRequiresQualifiedScore);
    };

    let member_count = case.forecast_members.len();
    let cells = case.rows * case.columns;
    let mut rng = SplitMix64::new(random_config.seed);
    let mut null_scores = Vec::with_capacity(random_config.draws);

    for _ in 0..random_config.draws {
        let mut random_members = vec![vec![false; cells]; member_count];
        for member in &mut random_members {
            for index in 0..cells {
                if prepared.usable[index] {
                    member[index] = rng.next_f64() < prepared.observation_event_frequency;
                }
            }
        }

        let score = evaluate_events(case, config, &prepared, &random_members)?;
        let Some(score_value) = score.p_fss else {
            return Err(FssError::NumericalInvariant);
        };
        null_scores.push(score_value);
    }

    null_scores.sort_by(f64::total_cmp);
    let null_mean = null_scores.iter().sum::<f64>() / null_scores.len() as f64;
    let p95_index = ((0.95 * null_scores.len() as f64).ceil() as usize)
        .saturating_sub(1)
        .min(null_scores.len() - 1);
    let null_p95 = null_scores[p95_index];
    let exceedances = null_scores
        .iter()
        .filter(|score| **score >= observed_p_fss)
        .count();
    let upper_tail_p_value =
        (exceedances as f64 + 1.0) / (random_config.draws as f64 + 1.0);

    Ok(RandomBaselineReport {
        case_id: case.case_id.clone(),
        threshold: config.threshold,
        threshold_operator: config.threshold_operator,
        unit: config.unit.clone(),
        window_width_cells: config.window_width_cells,
        ensemble_members: member_count,
        draws: random_config.draws,
        seed: random_config.seed,
        observed_event_frequency: prepared.observation_event_frequency,
        null_mean,
        null_p95,
        upper_tail_p_value,
        exceedances,
    })
}

fn prepare_case(case: &FssCase, config: &FssConfig) -> Result<PreparedCase, FssError> {
    let cells = case.rows * case.columns;
    let mut usable = vec![false; cells];
    let mut observation_events = vec![false; cells];
    let mut member_events = vec![vec![false; cells]; case.forecast_members.len()];
    let mut usable_grid_cells = 0usize;
    let mut valid_mask_cells = 0usize;
    let mut total_weight = 0.0;
    let mut observed_event_weight = 0.0;

    for index in 0..cells {
        if !case.valid_mask[index] {
            continue;
        }
        valid_mask_cells += 1;

        let Some(observation) = case.observation[index] else {
            continue;
        };
        let mut values = Vec::with_capacity(case.forecast_members.len());
        let mut complete = true;
        for member in &case.forecast_members {
            match member[index] {
                Some(value) => values.push(value),
                None => {
                    complete = false;
                    break;
                }
            }
        }
        if !complete {
            continue;
        }

        let weight = case.cell_weights[index];
        let observed_event = config
            .threshold_operator
            .exceeds(observation, config.threshold);
        usable[index] = true;
        observation_events[index] = observed_event;
        usable_grid_cells += 1;
        total_weight += weight;
        if observed_event {
            observed_event_weight += weight;
        }

        for (member_index, value) in values.into_iter().enumerate() {
            member_events[member_index][index] = config
                .threshold_operator
                .exceeds(value, config.threshold);
        }
    }

    if usable_grid_cells == 0 || total_weight <= 0.0 {
        return Err(FssError::NoUsableCells);
    }

    let observation_event_frequency = observed_event_weight / total_weight;
    Ok(PreparedCase {
        usable,
        observation_events,
        member_events,
        usable_grid_cells,
        valid_mask_cells,
        observation_event_frequency,
    })
}

fn evaluate_events(
    case: &FssCase,
    config: &FssConfig,
    prepared: &PreparedCase,
    member_events: &[Vec<bool>],
) -> Result<FssScore, FssError> {
    if member_events.len() != case.forecast_members.len()
        || member_events
            .iter()
            .any(|member| member.len() != case.rows * case.columns)
    {
        return Err(FssError::VectorLengthMismatch);
    }

    let radius = config.window_width_cells / 2;
    let mut centre_count = 0usize;
    let mut centre_weight_sum = 0.0;
    let mut forecast_fraction_sum = 0.0;
    let mut observation_fraction_sum = 0.0;
    let mut fbs_sum = 0.0;
    let mut wfbs_sum = 0.0;

    for centre in 0..case.rows * case.columns {
        if !prepared.usable[centre] {
            continue;
        }
        let row = centre / case.columns;
        let column = centre % case.columns;

        if config.boundary_policy == BoundaryPolicy::RequireFullWindow
            && (row < radius
                || column < radius
                || row + radius >= case.rows
                || column + radius >= case.columns)
        {
            continue;
        }

        let row_start = row.saturating_sub(radius);
        let row_end = (row + radius).min(case.rows - 1);
        let col_start = column.saturating_sub(radius);
        let col_end = (column + radius).min(case.columns - 1);

        let mut neighbourhood_weight = 0.0;
        let mut observation_weighted_sum = 0.0;
        let mut forecast_weighted_sum = 0.0;

        for neighbour_row in row_start..=row_end {
            for neighbour_column in col_start..=col_end {
                let index = neighbour_row * case.columns + neighbour_column;
                if !prepared.usable[index] {
                    continue;
                }

                let weight = case.cell_weights[index];
                neighbourhood_weight += weight;
                if prepared.observation_events[index] {
                    observation_weighted_sum += weight;
                }

                let member_event_count = member_events
                    .iter()
                    .filter(|member| member[index])
                    .count();
                let ensemble_probability =
                    member_event_count as f64 / member_events.len() as f64;
                forecast_weighted_sum += weight * ensemble_probability;
            }
        }

        if neighbourhood_weight <= 0.0 {
            continue;
        }

        let forecast_fraction = forecast_weighted_sum / neighbourhood_weight;
        let observation_fraction = observation_weighted_sum / neighbourhood_weight;
        let centre_weight = case.cell_weights[centre];
        let difference = forecast_fraction - observation_fraction;

        centre_count += 1;
        centre_weight_sum += centre_weight;
        forecast_fraction_sum += centre_weight * forecast_fraction;
        observation_fraction_sum += centre_weight * observation_fraction;
        fbs_sum += centre_weight * difference * difference;
        wfbs_sum += centre_weight
            * (forecast_fraction * forecast_fraction
                + observation_fraction * observation_fraction);
    }

    if centre_count == 0 || centre_weight_sum <= 0.0 {
        return Err(FssError::NoValidCentres);
    }

    let fractions_brier_score = fbs_sum / centre_weight_sum;
    let worst_fractions_brier_score = wfbs_sum / centre_weight_sum;
    let (p_fss, status) = if worst_fractions_brier_score == 0.0 {
        (None, FssStatus::DegenerateNoEvents)
    } else {
        let score = 1.0 - fractions_brier_score / worst_fractions_brier_score;
        if !score.is_finite() || !(-1e-12..=1.0 + 1e-12).contains(&score) {
            return Err(FssError::NumericalInvariant);
        }
        (Some(score), FssStatus::Qualified)
    };

    Ok(FssScore {
        case_id: case.case_id.clone(),
        threshold: config.threshold,
        threshold_operator: config.threshold_operator,
        unit: config.unit.clone(),
        window_width_cells: config.window_width_cells,
        boundary_policy: config.boundary_policy,
        missing_policy: config.missing_policy,
        ensemble_members: member_events.len(),
        usable_grid_cells: prepared.usable_grid_cells,
        valid_mask_cells: prepared.valid_mask_cells,
        centre_count,
        centre_weight_sum,
        observation_event_frequency: prepared.observation_event_frequency,
        mean_forecast_fraction: forecast_fraction_sum / centre_weight_sum,
        mean_observation_fraction: observation_fraction_sum / centre_weight_sum,
        fractions_brier_score,
        worst_fractions_brier_score,
        p_fss,
        status,
    })
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
        let mantissa = self.next_u64() >> 11;
        mantissa as f64 * (1.0 / ((1_u64 << 53) as f64))
    }

    fn next_index(&mut self, length: usize) -> usize {
        ((u128::from(self.next_u64()) * length as u128) >> 64) as usize
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct FssBlockCase {
    /// Cases in the same event block are sampled together to preserve
    /// within-event temporal and spatial dependence.
    pub event_block_id: String,
    pub case: FssCase,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BootstrapPoolingPolicy {
    AggregateFbsAndWfbsComponents,
}

impl BootstrapPoolingPolicy {
    pub const fn label(self) -> &'static str {
        match self {
            Self::AggregateFbsAndWfbsComponents => "aggregate_fbs_wfbs_components",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BlockBootstrapConfig {
    pub replicates: usize,
    pub seed: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BlockBootstrapReport {
    pub case_count: usize,
    pub independent_event_blocks: usize,
    pub requested_replicates: usize,
    pub qualified_replicates: usize,
    pub degenerate_replicates: usize,
    pub seed: u64,
    pub pooling_policy: BootstrapPoolingPolicy,
    pub mean_p_fss: f64,
    pub lower_95_percent: f64,
    pub median: f64,
    pub upper_95_percent: f64,
}

/// Percentile bootstrap over independent event blocks, never grid cells.
/// All cases sharing an event_block_id are kept together in each resample.
/// The score pools FBS and WFBS components before taking their ratio.
pub fn block_bootstrap(
    samples: &[FssBlockCase],
    config: &FssConfig,
    bootstrap: &BlockBootstrapConfig,
) -> Result<BlockBootstrapReport, FssError> {
    if !(MIN_RANDOM_DRAWS..=MAX_RANDOM_DRAWS).contains(&bootstrap.replicates) {
        return Err(FssError::InvalidRandomDrawCount);
    }
    if samples.is_empty() {
        return Err(FssError::NoUsableCells);
    }

    let mut blocks: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
    let mut case_ids = BTreeSet::new();
    for (index, sample) in samples.iter().enumerate() {
        if !valid_identifier(&sample.event_block_id) {
            return Err(FssError::InvalidEventBlockId);
        }
        if !case_ids.insert(sample.case.case_id.as_str()) {
            return Err(FssError::DuplicateCaseId);
        }
        sample.case.validate(config)?;
        blocks
            .entry(sample.event_block_id.as_str())
            .or_default()
            .push(index);
    }
    if blocks.len() < 2 {
        return Err(FssError::InsufficientIndependentEventBlocks);
    }

    let block_indices: Vec<&Vec<usize>> = blocks.values().collect();
    let mut rng = SplitMix64::new(bootstrap.seed);
    let mut replicate_scores = Vec::with_capacity(bootstrap.replicates);
    let mut degenerate_replicates = 0usize;

    for _ in 0..bootstrap.replicates {
        let mut fbs_component_sum = 0.0;
        let mut wfbs_component_sum = 0.0;

        // Draw as many independent event blocks as the original sample has.
        for _ in 0..block_indices.len() {
            let selected = block_indices[rng.next_index(block_indices.len())];
            for sample_index in selected {
                let score = evaluate(&samples[*sample_index].case, config)?;
                if score.status == FssStatus::Qualified {
                    fbs_component_sum +=
                        score.fractions_brier_score * score.centre_weight_sum;
                    wfbs_component_sum +=
                        score.worst_fractions_brier_score * score.centre_weight_sum;
                }
            }
        }

        if wfbs_component_sum == 0.0 {
            degenerate_replicates += 1;
            continue;
        }
        let p_fss = 1.0 - fbs_component_sum / wfbs_component_sum;
        if !p_fss.is_finite() || !(-1e-12..=1.0 + 1e-12).contains(&p_fss) {
            return Err(FssError::NumericalInvariant);
        }
        replicate_scores.push(p_fss);
    }

    if replicate_scores.len() < 2 {
        return Err(FssError::NoQualifiedBootstrapReplicates);
    }
    replicate_scores.sort_by(f64::total_cmp);
    let mean_p_fss = replicate_scores.iter().sum::<f64>() / replicate_scores.len() as f64;

    Ok(BlockBootstrapReport {
        case_count: samples.len(),
        independent_event_blocks: blocks.len(),
        requested_replicates: bootstrap.replicates,
        qualified_replicates: replicate_scores.len(),
        degenerate_replicates,
        seed: bootstrap.seed,
        pooling_policy: BootstrapPoolingPolicy::AggregateFbsAndWfbsComponents,
        mean_p_fss,
        lower_95_percent: quantile(&replicate_scores, 0.025),
        median: quantile(&replicate_scores, 0.5),
        upper_95_percent: quantile(&replicate_scores, 0.975),
    })
}

fn quantile(sorted_values: &[f64], probability: f64) -> f64 {
    let position = probability * (sorted_values.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    let fraction = position - lower as f64;
    sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config(window_width_cells: usize) -> FssConfig {
        FssConfig {
            threshold: 0.5,
            threshold_operator: ThresholdOperator::GreaterThanOrEqual,
            unit: "mm".into(),
            window_width_cells,
            boundary_policy: BoundaryPolicy::TruncateAtDomain,
            missing_policy: MissingPolicy::RejectCase,
        }
    }

    fn case(
        case_id: &str,
        rows: usize,
        columns: usize,
        forecast_members: Vec<Vec<Option<f64>>>,
        observation: Vec<Option<f64>>,
    ) -> FssCase {
        let cells = rows * columns;
        FssCase {
            case_id: case_id.into(),
            rows,
            columns,
            forecast_members,
            observation,
            cell_weights: vec![1.0; cells],
            valid_mask: vec![true; cells],
        }
    }

    fn values(events: &[bool]) -> Vec<Option<f64>> {
        events
            .iter()
            .map(|event| Some(if *event { 1.0 } else { 0.0 }))
            .collect()
    }

    #[test]
    fn perfect_single_member_forecast_scores_one() {
        let events = [
            false, false, false, false, true, true, false, false, true,
        ];
        let input = case(
            "perfect",
            3,
            3,
            vec![values(&events)],
            values(&events),
        );
        let score = evaluate(&input, &config(1)).expect("perfect forecast must score");
        assert_eq!(score.status, FssStatus::Qualified);
        assert_eq!(score.p_fss, Some(1.0));
    }

    #[test]
    fn one_member_p_fss_equals_deterministic_fss_definition() {
        let observed = [false, true, false, false, false, true, false, false, false];
        let forecast = [false, false, true, false, false, true, false, false, false];
        let input = case(
            "one-member",
            3,
            3,
            vec![values(&forecast)],
            values(&observed),
        );

        let score = evaluate(&input, &config(3)).expect("one-member FSS must score");
        assert_eq!(score.status, FssStatus::Qualified);
        assert!(score.p_fss.is_some_and(|value| (0.0..=1.0).contains(&value)));
    }

    #[test]
    fn no_event_on_either_field_is_typed_degenerate_not_perfect() {
        let zeros = [false; 9];
        let input = case("no-events", 3, 3, vec![values(&zeros)], values(&zeros));
        let score = evaluate(&input, &config(1)).expect("degenerate case is reportable");
        assert_eq!(score.status, FssStatus::DegenerateNoEvents);
        assert_eq!(score.p_fss, None);
        assert_eq!(score.worst_fractions_brier_score, 0.0);
    }

    #[test]
    fn complete_miss_has_zero_fss_when_observed_events_exist() {
        let observed = [false, false, false, false, true, false, false, false, false];
        let forecast = [false; 9];
        let input = case(
            "complete-miss",
            3,
            3,
            vec![values(&forecast)],
            values(&observed),
        );
        let score = evaluate(&input, &config(1)).expect("complete miss must score");
        assert_eq!(score.p_fss, Some(0.0));
    }

    #[test]
    fn spatial_neighbourhood_can_credit_a_small_event_displacement() {
        let mut observed = [false; 25];
        let mut forecast = [false; 25];
        observed[12] = true;
        forecast[13] = true;
        let input = case(
            "one-cell-displacement",
            5,
            5,
            vec![values(&forecast)],
            values(&observed),
        );

        let pointwise = evaluate(&input, &config(1)).expect("pointwise score");
        let neighbourhood = evaluate(&input, &config(3)).expect("neighbourhood score");
        assert!(pointwise.p_fss.unwrap() < neighbourhood.p_fss.unwrap());
    }

    #[test]
    fn same_pointwise_event_count_can_have_different_spatial_scores() {
        let observed = [
            false, false, false, false, false,
            false, true, true, false, false,
            false, false, false, false, false,
            false, false, false, false, false,
            false, false, false, false, false,
        ];
        let aligned = observed;
        let displaced = [
            false, false, false, false, false,
            false, false, false, false, false,
            false, false, false, false, false,
            false, false, true, true, false,
            false, false, false, false, false,
        ];

        let aligned_case = case(
            "aligned",
            5,
            5,
            vec![values(&aligned)],
            values(&observed),
        );
        let displaced_case = case(
            "displaced",
            5,
            5,
            vec![values(&displaced)],
            values(&observed),
        );
        let aligned_score =
            evaluate(&aligned_case, &config(3)).expect("aligned score");
        let displaced_score =
            evaluate(&displaced_case, &config(3)).expect("displaced score");

        assert_eq!(aligned_score.observation_event_frequency, displaced_score.observation_event_frequency);
        assert!(aligned_score.p_fss.unwrap() > displaced_score.p_fss.unwrap());
    }

    #[test]
    fn threshold_equality_convention_is_explicit() {
        let input = case(
            "threshold-equality",
            1,
            2,
            vec![vec![Some(0.5), Some(0.0)]],
            vec![Some(0.5), Some(0.0)],
        );
        let inclusive = evaluate(&input, &config(1)).expect("inclusive threshold");
        let mut exclusive_config = config(1);
        exclusive_config.threshold_operator = ThresholdOperator::GreaterThan;
        let exclusive = evaluate(&input, &exclusive_config).expect("exclusive threshold");

        assert_eq!(inclusive.observation_event_frequency, 0.5);
        assert_eq!(exclusive.observation_event_frequency, 0.0);
        assert_eq!(exclusive.status, FssStatus::DegenerateNoEvents);
    }

    #[test]
    fn area_weights_change_neighbourhood_fractions_and_are_preserved() {
        let mut input = case(
            "weighted",
            1,
            3,
            vec![values(&[true, false, false])],
            values(&[true, false, false]),
        );
        input.cell_weights = vec![4.0, 1.0, 1.0];
        let score = evaluate(&input, &config(3)).expect("weighted score");
        assert_eq!(score.observation_event_frequency, 4.0 / 6.0);
        assert_eq!(score.centre_weight_sum, 6.0);
    }

    #[test]
    fn strict_missing_policy_rejects_missing_member_or_observation() {
        let mut input = case(
            "strict-missing",
            1,
            3,
            vec![vec![Some(1.0), None, Some(0.0)]],
            vec![Some(1.0), Some(0.0), Some(0.0)],
        );
        assert_eq!(evaluate(&input, &config(1)), Err(FssError::MissingData));

        input.forecast_members[0][1] = Some(0.0);
        input.observation[1] = None;
        assert_eq!(evaluate(&input, &config(1)), Err(FssError::MissingData));
    }

    #[test]
    fn pairwise_complete_policy_drops_missing_cells_without_turning_them_into_no_events() {
        let input = case(
            "pairwise-complete",
            1,
            3,
            vec![vec![Some(1.0), None, Some(0.0)]],
            vec![Some(1.0), Some(1.0), Some(0.0)],
        );
        let mut cfg = config(1);
        cfg.missing_policy = MissingPolicy::PairwiseComplete;
        let score = evaluate(&input, &cfg).expect("pairwise complete score");
        assert_eq!(score.valid_mask_cells, 3);
        assert_eq!(score.usable_grid_cells, 2);
        assert_eq!(score.centre_count, 2);
        assert_eq!(score.observation_event_frequency, 0.5);
    }

    #[test]
    fn irregular_mask_is_not_treated_as_missing() {
        let mut input = case(
            "irregular-mask",
            2,
            2,
            vec![values(&[true, false, false, true])],
            values(&[true, false, false, true]),
        );
        input.valid_mask[1] = false;
        input.valid_mask[2] = false;
        input.cell_weights[1] = 0.0;
        input.cell_weights[2] = 0.0;
        let score = evaluate(&input, &config(3)).expect("masked score");
        assert_eq!(score.valid_mask_cells, 2);
        assert_eq!(score.usable_grid_cells, 2);
    }

    #[test]
    fn full_window_boundary_policy_excludes_edge_centres() {
        let events = [false; 25];
        let input = case(
            "boundary",
            5,
            5,
            vec![values(&events)],
            values(&events),
        );
        let truncated = evaluate(&input, &config(3)).expect("truncated boundary");
        let mut full_config = config(3);
        full_config.boundary_policy = BoundaryPolicy::RequireFullWindow;
        let full = evaluate(&input, &full_config).expect("full-window boundary");
        assert_eq!(truncated.centre_count, 25);
        assert_eq!(full.centre_count, 9);
    }

    #[test]
    fn even_zero_and_excessive_window_widths_are_rejected() {
        let events = [false; 9];
        let input = case("invalid-window", 3, 3, vec![values(&events)], values(&events));
        for width in [0, 2, 2049] {
            assert_eq!(evaluate(&input, &config(width)), Err(FssError::InvalidWindowWidth));
        }
    }

    #[test]
    fn invalid_weights_are_rejected() {
        let events = [false; 4];
        let mut input = case("invalid-weight", 2, 2, vec![values(&events)], values(&events));
        input.cell_weights[0] = f64::NAN;
        assert_eq!(evaluate(&input, &config(1)), Err(FssError::InvalidWeight));

        input.cell_weights[0] = 0.0;
        assert_eq!(evaluate(&input, &config(1)), Err(FssError::InvalidWeight));

        input.cell_weights[0] = -1.0;
        assert_eq!(evaluate(&input, &config(1)), Err(FssError::InvalidWeight));
    }

    #[test]
    fn non_finite_threshold_or_grid_values_are_rejected() {
        let events = [false; 4];
        let input = case("non-finite", 2, 2, vec![values(&events)], values(&events));
        let mut cfg = config(1);
        cfg.threshold = f64::NAN;
        assert_eq!(evaluate(&input, &cfg), Err(FssError::InvalidThreshold));

        let mut invalid = input.clone();
        invalid.observation[0] = Some(f64::INFINITY);
        assert_eq!(evaluate(&invalid, &config(1)), Err(FssError::NonFiniteValue));
    }

    #[test]
    fn shape_and_identifier_mismatches_are_rejected() {
        let events = [false; 4];
        let mut input = case("shape", 2, 2, vec![values(&events)], values(&events));
        input.observation.pop();
        assert_eq!(evaluate(&input, &config(1)), Err(FssError::VectorLengthMismatch));

        input.case_id = "contains space".into();
        assert_eq!(evaluate(&input, &config(1)), Err(FssError::InvalidCaseId));
    }

    #[test]
    fn one_member_and_repeated_members_have_identical_p_fss() {
        let observed = [false, true, false, false, true, false, false, false, false];
        let forecast = [false, false, true, false, true, false, false, false, false];
        let one_member = case(
            "one-member",
            3,
            3,
            vec![values(&forecast)],
            values(&observed),
        );
        let repeated_members = case(
            "repeated-members",
            3,
            3,
            vec![values(&forecast), values(&forecast), values(&forecast)],
            values(&observed),
        );
        let score_one = evaluate(&one_member, &config(3)).expect("one member");
        let score_repeated =
            evaluate(&repeated_members, &config(3)).expect("repeated members");
        assert_eq!(score_one.p_fss, score_repeated.p_fss);
    }

    #[test]
    fn random_reference_is_deterministic_for_fixed_seed() {
        let observed = [
            false, true, false, false, true, false, false, false, true,
            false, false, false, true, false, false, false,
        ];
        let forecast_a = [
            false, true, false, false, false, false, false, false, true,
            false, false, false, true, false, false, false,
        ];
        let forecast_b = [
            true, false, false, false, true, false, false, false, false,
            false, false, false, true, false, false, false,
        ];
        let input = case(
            "random-null",
            4,
            4,
            vec![values(&forecast_a), values(&forecast_b), values(&forecast_a)],
            values(&observed),
        );
        let cfg = config(3);
        let null_cfg = RandomBaselineConfig { draws: 256, seed: 42 };
        let first = random_reference(&input, &cfg, &null_cfg).expect("first null");
        let second = random_reference(&input, &cfg, &null_cfg).expect("second null");
        assert_eq!(first, second);
        assert_eq!(first.draws, 256);
        assert_eq!(first.seed, 42);
        assert!((0.0..=1.0).contains(&first.upper_tail_p_value));
    }

    #[test]
    fn random_reference_matches_observed_base_rate_and_member_count() {
        let observed = [
            false, true, false, false, true, false, false, false, true,
            false, false, false, true, false, false, false,
        ];
        let forecast = [false; 16];
        let input = case(
            "random-null-contract",
            4,
            4,
            vec![values(&forecast); 5],
            values(&observed),
        );
        let null = random_reference(
            &input,
            &config(1),
            &RandomBaselineConfig { draws: 256, seed: 9 },
        )
        .expect("random reference");
        assert_eq!(null.ensemble_members, 5);
        assert_eq!(null.observed_event_frequency, 0.25);
    }

    #[test]
    fn random_reference_requires_qualified_observed_case() {
        let zeros = [false; 9];
        let input = case("empty-null", 3, 3, vec![values(&zeros)], values(&zeros));
        assert_eq!(
            random_reference(
                &input,
                &config(1),
                &RandomBaselineConfig { draws: 256, seed: 1 }
            ),
            Err(FssError::RandomBaselineRequiresQualifiedScore)
        );
    }

    #[test]
    fn random_reference_rejects_too_few_draws() {
        let observed = [false, true, false, false];
        let input = case(
            "too-few-draws",
            2,
            2,
            vec![values(&observed)],
            values(&observed),
        );
        assert_eq!(
            random_reference(
                &input,
                &config(1),
                &RandomBaselineConfig { draws: 10, seed: 1 }
            ),
            Err(FssError::InvalidRandomDrawCount)
        );
    }

    fn block_sample(
        block: &str,
        case_id: &str,
        forecast: &[bool],
        observed: &[bool],
    ) -> FssBlockCase {
        FssBlockCase {
            event_block_id: block.into(),
            case: case(case_id, 2, 2, vec![values(forecast)], values(observed)),
        }
    }

    #[test]
    fn event_block_bootstrap_is_deterministic_and_reports_its_sampling_unit() {
        let observed = [true, false, false, true];
        let samples = vec![
            block_sample("event-a", "case-a1", &[true, false, false, true], &observed),
            block_sample("event-a", "case-a2", &[true, false, false, false], &observed),
            block_sample("event-b", "case-b1", &[false, false, true, true], &observed),
            block_sample("event-b", "case-b2", &[false, false, false, true], &observed),
            block_sample("event-c", "case-c1", &[true, false, true, false], &observed),
            block_sample("event-c", "case-c2", &[false, false, true, false], &observed),
        ];
        let bootstrap = BlockBootstrapConfig {
            replicates: 256,
            seed: 77,
        };
        let first = block_bootstrap(&samples, &config(1), &bootstrap)
            .expect("event-block bootstrap must qualify");
        let second = block_bootstrap(&samples, &config(1), &bootstrap)
            .expect("repeat bootstrap must qualify");

        assert_eq!(first, second);
        assert_eq!(first.case_count, 6);
        assert_eq!(first.independent_event_blocks, 3);
        assert_eq!(first.requested_replicates, 256);
        assert_eq!(
            first.qualified_replicates + first.degenerate_replicates,
            256
        );
        assert_eq!(
            first.pooling_policy,
            BootstrapPoolingPolicy::AggregateFbsAndWfbsComponents
        );
        assert!(first.lower_95_percent <= first.median);
        assert!(first.median <= first.upper_95_percent);
    }

    #[test]
    fn event_block_bootstrap_keeps_degenerate_resamples_visible() {
        let no_events = [false, false, false, false];
        let event = [true, false, false, false];
        let samples = vec![
            block_sample("quiet-event", "quiet-1", &no_events, &no_events),
            block_sample("quiet-event", "quiet-2", &no_events, &no_events),
            block_sample("active-event", "active-1", &event, &event),
            block_sample("active-event", "active-2", &event, &event),
        ];
        let report = block_bootstrap(
            &samples,
            &config(1),
            &BlockBootstrapConfig {
                replicates: 256,
                seed: 17,
            },
        )
        .expect("non-degenerate bootstrap draws should remain");
        assert!(report.degenerate_replicates > 0);
        assert!(report.qualified_replicates > 0);
        assert_eq!(
            report.qualified_replicates + report.degenerate_replicates,
            256
        );
    }

    #[test]
    fn event_block_bootstrap_rejects_case_level_iid_shortcuts() {
        let observed = [true, false, false, false];
        let samples = vec![block_sample(
            "single-weather-event",
            "case-one",
            &observed,
            &observed,
        )];
        assert_eq!(
            block_bootstrap(
                &samples,
                &config(1),
                &BlockBootstrapConfig {
                    replicates: 256,
                    seed: 1
                }
            ),
            Err(FssError::InsufficientIndependentEventBlocks)
        );
    }

    #[test]
    fn event_block_bootstrap_rejects_duplicate_case_keys() {
        let observed = [true, false, false, false];
        let samples = vec![
            block_sample("event-a", "duplicate-case", &observed, &observed),
            block_sample("event-b", "duplicate-case", &observed, &observed),
        ];
        assert_eq!(
            block_bootstrap(
                &samples,
                &config(1),
                &BlockBootstrapConfig {
                    replicates: 256,
                    seed: 1
                }
            ),
            Err(FssError::DuplicateCaseId)
        );
    }
}

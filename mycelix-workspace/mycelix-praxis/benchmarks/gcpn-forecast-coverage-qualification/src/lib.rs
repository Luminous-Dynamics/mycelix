//! Known-DGP qualification core for paired probabilistic forecast intervals.
//!
//! This crate is a synthetic statistical-method benchmark. It does not produce
//! forecasts, issue warnings, select operational models, or authorize actions.
//!
//! The top-level resampling unit is an independently generated synthetic
//! system. Event IDs are nested within systems and deliberately share a
//! system-level latent state, so event- and case-level resampling are available
//! as negative controls rather than assumed-valid alternatives.

use std::collections::BTreeSet;

const WILSON_Z_95: f64 = 1.959_963_984_540_054;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootstrapUnit {
    Case,
    Event,
    IndependentSystem,
}

impl BootstrapUnit {
    pub const ALL: [Self; 3] = [Self::Case, Self::Event, Self::IndependentSystem];

    pub fn label(self) -> &'static str {
        match self {
            Self::Case => "iid_case_negative_control",
            Self::Event => "event_id_negative_control",
            Self::IndependentSystem => "independent_system_primary",
        }
    }

    fn seed_salt(self) -> u64 {
        match self {
            Self::Case => 0x243f_6a88_85a3_08d3,
            Self::Event => 0x1319_8a2e_0370_7344,
            Self::IndependentSystem => 0xa409_3822_299f_31d0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DgpConfig {
    pub independent_systems: usize,
    pub high_risk_probability: f64,
    pub events_per_system: usize,
    pub low_cases_per_event: usize,
    pub high_cases_per_event: usize,
    pub low_event_probability: f64,
    pub high_event_probability: f64,
    pub candidate_probability: f64,
    pub reference_probability: f64,
}

impl Default for DgpConfig {
    fn default() -> Self {
        Self {
            independent_systems: 40,
            high_risk_probability: 0.20,
            events_per_system: 2,
            low_cases_per_event: 1,
            high_cases_per_event: 4,
            low_event_probability: 0.10,
            high_event_probability: 0.60,
            candidate_probability: 0.20,
            reference_probability: 0.35,
        }
    }
}

impl DgpConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.independent_systems < 2 {
            return Err("at least two independent systems are required".into());
        }
        if self.events_per_system == 0
            || self.low_cases_per_event == 0
            || self.high_cases_per_event == 0
        {
            return Err("event and case counts must be positive".into());
        }
        for (name, value) in [
            ("high_risk_probability", self.high_risk_probability),
            ("low_event_probability", self.low_event_probability),
            ("high_event_probability", self.high_event_probability),
            ("candidate_probability", self.candidate_probability),
            ("reference_probability", self.reference_probability),
        ] {
            if !value.is_finite() || !(0.0..=1.0).contains(&value) {
                return Err(format!("{name} must be finite and in [0, 1]"));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ForecastCase {
    pub case_id: String,
    pub system_id: String,
    pub event_id: String,
    pub outcome: bool,
    pub candidate_probability: f64,
    pub reference_probability: f64,
}

impl ForecastCase {
    /// Paired Brier-loss difference: candidate minus reference.
    /// Negative values favour the candidate.
    pub fn paired_brier_difference(&self) -> f64 {
        let y = if self.outcome { 1.0 } else { 0.0 };
        (self.candidate_probability - y).powi(2)
            - (self.reference_probability - y).powi(2)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct EventCluster {
    pub event_id: String,
    pub cases: Vec<ForecastCase>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct IndependentSystem {
    pub system_id: String,
    pub high_risk_state: bool,
    pub events: Vec<EventCluster>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Dataset {
    pub systems: Vec<IndependentSystem>,
}

impl Dataset {
    pub fn validate(&self) -> Result<(), String> {
        let mut system_ids = BTreeSet::new();
        let mut event_ids = BTreeSet::new();
        let mut case_ids = BTreeSet::new();

        for system in &self.systems {
            if !system_ids.insert(system.system_id.as_str()) {
                return Err(format!("duplicate system id: {}", system.system_id));
            }
            if system.events.is_empty() {
                return Err(format!("system {} has no events", system.system_id));
            }
            for event in &system.events {
                if !event_ids.insert(event.event_id.as_str()) {
                    return Err(format!("duplicate event id: {}", event.event_id));
                }
                if event.cases.is_empty() {
                    return Err(format!("event {} has no cases", event.event_id));
                }
                for case in &event.cases {
                    if !case_ids.insert(case.case_id.as_str()) {
                        return Err(format!("duplicate case id: {}", case.case_id));
                    }
                    if case.system_id != system.system_id || case.event_id != event.event_id {
                        return Err(format!(
                            "case {} has a system/event parent mismatch",
                            case.case_id
                        ));
                    }
                    for (name, probability) in [
                        ("candidate", case.candidate_probability),
                        ("reference", case.reference_probability),
                    ] {
                        if !probability.is_finite() || !(0.0..=1.0).contains(&probability) {
                            return Err(format!(
                                "case {} has invalid {name} probability",
                                case.case_id
                            ));
                        }
                    }
                }
            }
        }

        if self.systems.len() < 2 {
            return Err("at least two independent systems are required".into());
        }
        Ok(())
    }

    pub fn case_count(&self) -> usize {
        self.systems
            .iter()
            .flat_map(|system| &system.events)
            .map(|event| event.cases.len())
            .sum()
    }

    pub fn event_count(&self) -> usize {
        self.systems.iter().map(|system| system.events.len()).sum()
    }

    /// Case-weighted sample estimand: sum of all paired score differences
    /// divided by the number of matched forecast cases.
    pub fn estimate_case_weighted(&self) -> Result<f64, String> {
        self.validate()?;
        let mut difference_sum = 0.0;
        let mut count = 0usize;
        for case in self
            .systems
            .iter()
            .flat_map(|system| &system.events)
            .flat_map(|event| &event.cases)
        {
            difference_sum += case.paired_brier_difference();
            count += 1;
        }
        if count == 0 {
            return Err("no forecast cases available".into());
        }
        Ok(difference_sum / count as f64)
    }

    /// System-weighted sample estimand: average each independent system's
    /// mean paired score difference, giving each system equal weight.
    pub fn estimate_system_weighted(&self) -> Result<f64, String> {
        self.validate()?;
        let mut system_means = Vec::with_capacity(self.systems.len());
        for system in &self.systems {
            let mut sum = 0.0;
            let mut count = 0usize;
            for case in system.events.iter().flat_map(|event| &event.cases) {
                sum += case.paired_brier_difference();
                count += 1;
            }
            if count == 0 {
                return Err(format!("system {} has no cases", system.system_id));
            }
            system_means.push(sum / count as f64);
        }
        Ok(system_means.iter().sum::<f64>() / system_means.len() as f64)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PopulationTruth {
    /// E[total paired loss difference per system] / E[cases per system].
    pub case_weighted: f64,
    /// E[mean paired loss difference within a system].
    pub system_weighted: f64,
    pub expected_cases_per_system: f64,
    pub low_state_expected_difference: f64,
    pub high_state_expected_difference: f64,
}

fn expected_brier_difference(
    outcome_probability: f64,
    candidate_probability: f64,
    reference_probability: f64,
) -> f64 {
    (candidate_probability - outcome_probability).powi(2)
        - (reference_probability - outcome_probability).powi(2)
}

/// Analytical population oracle for the two-state DGP.
///
/// The oracle integrates over the declared system-state and cluster-size
/// probabilities. It is not computed from a simulated dataset.
pub fn analytic_truth(config: &DgpConfig) -> Result<PopulationTruth, String> {
    config.validate()?;
    let p_high = config.high_risk_probability;
    let low_cases = (config.events_per_system * config.low_cases_per_event) as f64;
    let high_cases = (config.events_per_system * config.high_cases_per_event) as f64;
    let delta_low = expected_brier_difference(
        config.low_event_probability,
        config.candidate_probability,
        config.reference_probability,
    );
    let delta_high = expected_brier_difference(
        config.high_event_probability,
        config.candidate_probability,
        config.reference_probability,
    );
    let expected_cases = (1.0 - p_high) * low_cases + p_high * high_cases;
    let expected_difference = (1.0 - p_high) * low_cases * delta_low
        + p_high * high_cases * delta_high;
    let system_weighted =
        (1.0 - p_high) * delta_low + p_high * delta_high;

    Ok(PopulationTruth {
        case_weighted: expected_difference / expected_cases,
        system_weighted,
        expected_cases_per_system: expected_cases,
        low_state_expected_difference: delta_low,
        high_state_expected_difference: delta_high,
    })
}

#[derive(Debug, Clone, Copy)]
struct XorShift64Star {
    state: u64,
}

impl XorShift64Star {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 {
                0x9e37_79b9_7f4a_7c15
            } else {
                seed
            },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }

    fn next_unit_f64(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64) * (1.0 / ((1u64 << 53) as f64))
    }

    fn index(&mut self, upper: usize) -> usize {
        let bound = upper as u64;
        let threshold = bound.wrapping_neg() % bound;
        loop {
            let value = self.next_u64();
            if value >= threshold {
                return (value % bound) as usize;
            }
        }
    }
}

fn derive_seed(seed: u64, stream: u64) -> u64 {
    let mut value = seed ^ stream.wrapping_mul(0x9e37_79b9_7f4a_7c15);
    value ^= value >> 30;
    value = value.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value ^= value >> 27;
    value = value.wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

pub fn generate_dataset(config: &DgpConfig, seed: u64) -> Result<Dataset, String> {
    config.validate()?;
    let mut rng = XorShift64Star::new(seed);
    let mut systems = Vec::with_capacity(config.independent_systems);

    for system_index in 0..config.independent_systems {
        let high_risk_state = rng.next_unit_f64() < config.high_risk_probability;
        let outcome_probability = if high_risk_state {
            config.high_event_probability
        } else {
            config.low_event_probability
        };
        let cases_per_event = if high_risk_state {
            config.high_cases_per_event
        } else {
            config.low_cases_per_event
        };
        let system_id = format!("system-{system_index:05}");
        let mut events = Vec::with_capacity(config.events_per_system);

        for event_index in 0..config.events_per_system {
            let event_id = format!("{system_id}-event-{event_index:03}");
            let mut cases = Vec::with_capacity(cases_per_event);
            for case_index in 0..cases_per_event {
                let outcome = rng.next_unit_f64() < outcome_probability;
                cases.push(ForecastCase {
                    case_id: format!("{event_id}-case-{case_index:04}"),
                    system_id: system_id.clone(),
                    event_id: event_id.clone(),
                    outcome,
                    candidate_probability: config.candidate_probability,
                    reference_probability: config.reference_probability,
                });
            }
            events.push(EventCluster { event_id, cases });
        }
        systems.push(IndependentSystem {
            system_id,
            high_risk_state,
            events,
        });
    }

    let dataset = Dataset { systems };
    dataset.validate()?;
    Ok(dataset)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BootstrapInterval {
    pub method: BootstrapUnit,
    pub estimate: f64,
    pub lower: f64,
    pub upper: f64,
    pub confidence_level: f64,
    pub replicates: usize,
    pub resampling_units: usize,
}

fn resampling_units(dataset: &Dataset, method: BootstrapUnit) -> Vec<Vec<&ForecastCase>> {
    match method {
        BootstrapUnit::Case => dataset
            .systems
            .iter()
            .flat_map(|system| &system.events)
            .flat_map(|event| &event.cases)
            .map(|case| vec![case])
            .collect(),
        BootstrapUnit::Event => dataset
            .systems
            .iter()
            .flat_map(|system| &system.events)
            .map(|event| event.cases.iter().collect())
            .collect(),
        BootstrapUnit::IndependentSystem => dataset
            .systems
            .iter()
            .map(|system| {
                system
                    .events
                    .iter()
                    .flat_map(|event| &event.cases)
                    .collect()
            })
            .collect(),
    }
}

fn percentile(sorted: &[f64], probability: f64) -> f64 {
    let position = probability * (sorted.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    let fraction = position - lower as f64;
    sorted[lower] + (sorted[upper] - sorted[lower]) * fraction
}

/// Paired percentile bootstrap for a declared resampling unit.
///
/// All candidate/reference losses and the shared outcome stay in the same
/// ForecastCase. The estimator is recomputed as a ratio of total paired loss
/// differences to total cases in each replicate (case-weighted estimand).
pub fn bootstrap_interval(
    dataset: &Dataset,
    method: BootstrapUnit,
    confidence_level: f64,
    replicates: usize,
    seed: u64,
) -> Result<BootstrapInterval, String> {
    dataset.validate()?;
    if !confidence_level.is_finite() || !(0.0..1.0).contains(&confidence_level) {
        return Err("confidence level must be finite and strictly between 0 and 1".into());
    }
    if replicates < 100 {
        return Err("at least 100 bootstrap replicates are required".into());
    }

    let units = resampling_units(dataset, method);
    if units.len() < 2 {
        return Err("at least two resampling units are required".into());
    }

    let estimate = dataset.estimate_case_weighted()?;
    let unit_summaries: Vec<(f64, usize)> = units
        .iter()
        .map(|unit| {
            (
                unit.iter()
                    .map(|case| case.paired_brier_difference())
                    .sum::<f64>(),
                unit.len(),
            )
        })
        .collect();

    let mut rng = XorShift64Star::new(seed);
    let mut samples = Vec::with_capacity(replicates);
    for _ in 0..replicates {
        let mut difference_sum = 0.0;
        let mut case_count = 0usize;
        for _ in 0..units.len() {
            let (unit_difference, unit_cases) = unit_summaries[rng.index(units.len())];
            difference_sum += unit_difference;
            case_count += unit_cases;
        }
        if case_count == 0 {
            return Err("bootstrap replicate contained no forecast cases".into());
        }
        samples.push(difference_sum / case_count as f64);
    }

    samples.sort_by(f64::total_cmp);
    let alpha = (1.0 - confidence_level) / 2.0;
    Ok(BootstrapInterval {
        method,
        estimate,
        lower: percentile(&samples, alpha),
        upper: percentile(&samples, 1.0 - alpha),
        confidence_level,
        replicates,
        resampling_units: units.len(),
    })
}

/// Reflected/basic percentile interval for the same paired bootstrap draws.
pub fn basic_bootstrap_interval(
    dataset: &Dataset,
    method: BootstrapUnit,
    confidence_level: f64,
    replicates: usize,
    seed: u64,
) -> Result<BootstrapInterval, String> {
    let percentile = bootstrap_interval(dataset, method, confidence_level, replicates, seed)?;
    Ok(BootstrapInterval {
        method: percentile.method,
        estimate: percentile.estimate,
        lower: 2.0 * percentile.estimate - percentile.upper,
        upper: 2.0 * percentile.estimate - percentile.lower,
        confidence_level: percentile.confidence_level,
        replicates: percentile.replicates,
        resampling_units: percentile.resampling_units,
    })
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StudentizedSystemInterval {
    pub estimate: f64,
    pub lower: f64,
    pub upper: f64,
    pub confidence_level: f64,
    pub requested_replicates: usize,
    pub valid_replicates: usize,
    pub abstentions: usize,
    pub independent_systems: usize,
    pub original_standard_error: f64,
}

fn ratio_cluster_standard_error(units: &[(f64, usize)], estimate: f64) -> Option<f64> {
    if units.len() < 2 {
        return None;
    }
    let count = units.len() as f64;
    let total_n: usize = units.iter().map(|(_, n)| *n).sum();
    if total_n == 0 {
        return None;
    }
    let mean_n = total_n as f64 / count;
    let residuals: Vec<f64> = units
        .iter()
        .map(|(difference, n)| *difference - estimate * (*n as f64))
        .collect();
    let residual_mean = residuals.iter().sum::<f64>() / count;
    let sample_variance = residuals
        .iter()
        .map(|value| (value - residual_mean).powi(2))
        .sum::<f64>()
        / (count - 1.0);
    let standard_error = (sample_variance / (count * mean_n * mean_n)).sqrt();
    (standard_error.is_finite() && standard_error > 0.0).then_some(standard_error)
}

/// Paired cluster-bootstrap-t interval for the case-weighted ratio estimand.
///
/// Resampling unit is the independent system. Each bootstrap replicate
/// recomputes the ratio estimator and a cluster-level delta-method standard
/// error. Degenerate replicates are counted as abstentions, not discarded
/// silently. This is a research candidate, not an assumed small-cluster fix.
pub fn studentized_system_bootstrap_interval(
    dataset: &Dataset,
    confidence_level: f64,
    replicates: usize,
    seed: u64,
) -> Result<StudentizedSystemInterval, String> {
    dataset.validate()?;
    if !confidence_level.is_finite() || !(0.0..1.0).contains(&confidence_level) {
        return Err("confidence level must be finite and strictly between 0 and 1".into());
    }
    if replicates < 100 {
        return Err("at least 100 studentized bootstrap replicates are required".into());
    }

    let units: Vec<(f64, usize)> = dataset
        .systems
        .iter()
        .map(|system| {
            let cases: Vec<&ForecastCase> = system
                .events
                .iter()
                .flat_map(|event| &event.cases)
                .collect();
            (
                cases
                    .iter()
                    .map(|case| case.paired_brier_difference())
                    .sum::<f64>(),
                cases.len(),
            )
        })
        .collect();
    let estimate = dataset.estimate_case_weighted()?;
    let original_se = ratio_cluster_standard_error(&units, estimate)
        .ok_or_else(|| "original independent-system standard error is degenerate".to_string())?;

    let mut rng = XorShift64Star::new(seed);
    let mut studentized = Vec::with_capacity(replicates);
    for _ in 0..replicates {
        let sampled: Vec<(f64, usize)> = (0..units.len())
            .map(|_| units[rng.index(units.len())])
            .collect();
        let total_difference = sampled.iter().map(|(d, _)| *d).sum::<f64>();
        let total_cases = sampled.iter().map(|(_, n)| *n).sum::<usize>();
        if total_cases == 0 {
            continue;
        }
        let replicate_estimate = total_difference / total_cases as f64;
        if let Some(replicate_se) = ratio_cluster_standard_error(&sampled, replicate_estimate) {
            let statistic = (replicate_estimate - estimate) / replicate_se;
            if statistic.is_finite() {
                studentized.push(statistic);
            }
        }
    }

    let abstentions = replicates - studentized.len();
    let minimum_valid = ((replicates as f64) * 0.80).ceil() as usize;
    if studentized.len() < minimum_valid {
        return Err(format!(
            "studentized bootstrap has too many degenerate replicates: valid={}, requested={}, minimum_valid={minimum_valid}",
            studentized.len(), replicates
        ));
    }
    studentized.sort_by(f64::total_cmp);
    let alpha = (1.0 - confidence_level) / 2.0;
    let lower_t = percentile(&studentized, alpha);
    let upper_t = percentile(&studentized, 1.0 - alpha);
    Ok(StudentizedSystemInterval {
        estimate,
        lower: estimate - upper_t * original_se,
        upper: estimate - lower_t * original_se,
        confidence_level,
        requested_replicates: replicates,
        valid_replicates: studentized.len(),
        abstentions,
        independent_systems: units.len(),
        original_standard_error: original_se,
    })
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CoverageSummary {
    pub method: BootstrapUnit,
    pub outer_datasets: usize,
    pub valid_intervals: usize,
    pub covered_intervals: usize,
    /// Coverage conditional on an interval being emitted. Abstentions are
    /// reported separately and must never be hidden.
    pub empirical_coverage: Option<f64>,
    pub coverage_mcse: Option<f64>,
    pub coverage_wilson_95_low: Option<f64>,
    pub coverage_wilson_95_high: Option<f64>,
    pub abstentions: usize,
    pub mean_interval_width: Option<f64>,
    pub estimator_bias: Option<f64>,
    pub estimator_rmse: Option<f64>,
}

fn wilson_interval(successes: usize, trials: usize) -> (f64, f64) {
    let n = trials as f64;
    let p = successes as f64 / n;
    let z2 = WILSON_Z_95 * WILSON_Z_95;
    let denominator = 1.0 + z2 / n;
    let center = (p + z2 / (2.0 * n)) / denominator;
    let half_width = WILSON_Z_95
        * ((p * (1.0 - p) / n) + z2 / (4.0 * n * n)).sqrt()
        / denominator;
    ((center - half_width).max(0.0), (center + half_width).min(1.0))
}

/// Pilot/confirmatory coverage driver. Callers must use separate seeds for
/// pilot and confirmatory runs and must freeze the DGP/method matrix before
/// interpreting confirmatory results.
#[derive(Debug, Clone, PartialEq)]
pub struct AlternativeCoverageSummary {
    pub interval_method: &'static str,
    pub outer_datasets: usize,
    pub valid_intervals: usize,
    pub covered_intervals: usize,
    pub empirical_coverage: Option<f64>,
    pub coverage_mcse: Option<f64>,
    pub coverage_wilson_95_low: Option<f64>,
    pub coverage_wilson_95_high: Option<f64>,
    pub abstentions: usize,
    pub mean_interval_width: Option<f64>,
}

/// Exploratory coverage comparison for the basic and studentized independent-
/// system intervals. Pilot and confirmatory runs must use separately frozen
/// seeds, counts and DGP matrices.
pub fn alternative_coverage_study(
    config: &DgpConfig,
    outer_datasets: usize,
    bootstrap_replicates: usize,
    confidence_level: f64,
    seed: u64,
) -> Result<Vec<AlternativeCoverageSummary>, String> {
    config.validate()?;
    if outer_datasets == 0 {
        return Err("outer Monte Carlo dataset count must be positive".into());
    }
    if bootstrap_replicates < 100 {
        return Err("at least 100 bootstrap replicates are required".into());
    }
    if !confidence_level.is_finite() || !(0.0..1.0).contains(&confidence_level) {
        return Err("confidence level must be finite and strictly between 0 and 1".into());
    }
    let truth = analytic_truth(config)?.case_weighted;
    let methods = [
        ("independent_system_basic", false),
        ("independent_system_studentized_t", true),
    ];
    let mut summaries = Vec::with_capacity(methods.len());

    for (label, studentized_method) in methods {
        let mut valid = 0usize;
        let mut covered = 0usize;
        let mut abstentions = 0usize;
        let mut width_sum = 0.0;

        for trial in 0..outer_datasets {
            let dataset = generate_dataset(config, derive_seed(seed, trial as u64))?;
            let method_seed = derive_seed(
                seed ^ (if studentized_method {
                    0x5be0_cd19_137e_2179
                } else {
                    0x1f83_d9ab_fb41_bd6b
                }),
                trial as u64 + 1,
            );
            let interval = if studentized_method {
                studentized_system_bootstrap_interval(
                    &dataset,
                    confidence_level,
                    bootstrap_replicates,
                    method_seed,
                )
                .map(|interval| (interval.lower, interval.upper))
            } else {
                basic_bootstrap_interval(
                    &dataset,
                    BootstrapUnit::IndependentSystem,
                    confidence_level,
                    bootstrap_replicates,
                    method_seed,
                )
                .map(|interval| (interval.lower, interval.upper))
            };

            match interval {
                Ok((lower, upper)) if lower.is_finite() && upper.is_finite() && lower <= upper => {
                    valid += 1;
                    if lower <= truth && truth <= upper {
                        covered += 1;
                    }
                    width_sum += upper - lower;
                }
                _ => abstentions += 1,
            }
        }

        if valid == 0 {
            summaries.push(AlternativeCoverageSummary {
                interval_method: label,
                outer_datasets,
                valid_intervals: 0,
                covered_intervals: 0,
                empirical_coverage: None,
                coverage_mcse: None,
                coverage_wilson_95_low: None,
                coverage_wilson_95_high: None,
                abstentions,
                mean_interval_width: None,
            });
            continue;
        }

        let coverage = covered as f64 / valid as f64;
        let mcse = (coverage * (1.0 - coverage) / valid as f64).sqrt();
        let (wilson_low, wilson_high) = wilson_interval(covered, valid);
        summaries.push(AlternativeCoverageSummary {
            interval_method: label,
            outer_datasets,
            valid_intervals: valid,
            covered_intervals: covered,
            empirical_coverage: Some(coverage),
            coverage_mcse: Some(mcse),
            coverage_wilson_95_low: Some(wilson_low),
            coverage_wilson_95_high: Some(wilson_high),
            abstentions,
            mean_interval_width: Some(width_sum / valid as f64),
        });
    }
    Ok(summaries)
}

pub fn coverage_study(
    config: &DgpConfig,
    outer_datasets: usize,
    bootstrap_replicates: usize,
    confidence_level: f64,
    seed: u64,
) -> Result<Vec<CoverageSummary>, String> {
    config.validate()?;
    if outer_datasets == 0 {
        return Err("outer Monte Carlo dataset count must be positive".into());
    }
    if bootstrap_replicates < 100 {
        return Err("at least 100 bootstrap replicates are required".into());
    }
    if !confidence_level.is_finite() || !(0.0..1.0).contains(&confidence_level) {
        return Err("confidence level must be finite and strictly between 0 and 1".into());
    }
    let truth = analytic_truth(config)?.case_weighted;
    let mut summaries = Vec::with_capacity(BootstrapUnit::ALL.len());

    for method in BootstrapUnit::ALL {
        let mut valid = 0usize;
        let mut covered = 0usize;
        let mut abstentions = 0usize;
        let mut width_sum = 0.0;
        let mut bias_sum = 0.0;
        let mut squared_error_sum = 0.0;

        for trial in 0..outer_datasets {
            let trial_seed = derive_seed(seed, trial as u64);
            let dataset = generate_dataset(config, trial_seed)?;
            let bootstrap_seed =
                derive_seed(seed ^ method.seed_salt(), trial as u64 + 1);
            match bootstrap_interval(
                &dataset,
                method,
                confidence_level,
                bootstrap_replicates,
                bootstrap_seed,
            ) {
                Ok(interval) => {
                    valid += 1;
                    if interval.lower <= truth && truth <= interval.upper {
                        covered += 1;
                    }
                    width_sum += interval.upper - interval.lower;
                    let error = interval.estimate - truth;
                    bias_sum += error;
                    squared_error_sum += error * error;
                }
                Err(_) => abstentions += 1,
            }
        }

        if valid == 0 {
            summaries.push(CoverageSummary {
                method,
                outer_datasets,
                valid_intervals: 0,
                covered_intervals: 0,
                empirical_coverage: None,
                coverage_mcse: None,
                coverage_wilson_95_low: None,
                coverage_wilson_95_high: None,
                abstentions,
                mean_interval_width: None,
                estimator_bias: None,
                estimator_rmse: None,
            });
            continue;
        }

        let coverage = covered as f64 / valid as f64;
        let mcse = (coverage * (1.0 - coverage) / valid as f64).sqrt();
        let (wilson_low, wilson_high) = wilson_interval(covered, valid);
        summaries.push(CoverageSummary {
            method,
            outer_datasets,
            valid_intervals: valid,
            covered_intervals: covered,
            empirical_coverage: Some(coverage),
            coverage_mcse: Some(mcse),
            coverage_wilson_95_low: Some(wilson_low),
            coverage_wilson_95_high: Some(wilson_high),
            abstentions,
            mean_interval_width: Some(width_sum / valid as f64),
            estimator_bias: Some(bias_sum / valid as f64),
            estimator_rmse: Some((squared_error_sum / valid as f64).sqrt()),
        });
    }

    Ok(summaries)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(left: f64, right: f64, tolerance: f64) {
        assert!(
            (left - right).abs() <= tolerance,
            "left={left:.12}, right={right:.12}, tolerance={tolerance}"
        );
    }

    #[test]
    fn analytic_oracle_distinguishes_case_and_system_estimands() {
        let truth = analytic_truth(&DgpConfig::default()).unwrap();
        close(truth.case_weighted, 0.0225, 1e-12);
        close(truth.system_weighted, -0.0225, 1e-12);
        assert!(truth.case_weighted * truth.system_weighted < 0.0);
    }

    #[test]
    fn generated_dataset_is_deterministic_and_well_formed() {
        let config = DgpConfig::default();
        let a = generate_dataset(&config, 0x6a09_e667_f3bc_c909).unwrap();
        let b = generate_dataset(&config, 0x6a09_e667_f3bc_c909).unwrap();
        assert_eq!(a, b);
        assert_eq!(a.systems.len(), config.independent_systems);
        assert_eq!(a.event_count(), config.independent_systems * config.events_per_system);
        a.validate().unwrap();
    }

    #[test]
    fn events_are_nested_under_a_shared_independent_system() {
        let dataset = generate_dataset(&DgpConfig::default(), 17).unwrap();
        for system in &dataset.systems {
            assert_eq!(system.events.len(), 2);
            for event in &system.events {
                for case in &event.cases {
                    assert_eq!(case.system_id, system.system_id);
                    assert_eq!(case.event_id, event.event_id);
                }
            }
        }
    }

    #[test]
    fn paired_score_difference_uses_the_same_outcome_for_both_models() {
        let case = ForecastCase {
            case_id: "case-1".into(),
            system_id: "system-1".into(),
            event_id: "event-1".into(),
            outcome: true,
            candidate_probability: 0.2,
            reference_probability: 0.35,
        };
        close(
            case.paired_brier_difference(),
            (0.2_f64 - 1.0).powi(2) - (0.35_f64 - 1.0).powi(2),
            1e-12,
        );
    }

    #[test]
    fn bootstrap_is_reproducible_and_exposes_resampling_unit() {
        let dataset = generate_dataset(&DgpConfig::default(), 42).unwrap();
        for method in BootstrapUnit::ALL {
            let a = bootstrap_interval(&dataset, method, 0.95, 500, 1234).unwrap();
            let b = bootstrap_interval(&dataset, method, 0.95, 500, 1234).unwrap();
            assert_eq!(a, b);
            assert_eq!(
                a.resampling_units,
                match method {
                    BootstrapUnit::Case => dataset.case_count(),
                    BootstrapUnit::Event => dataset.event_count(),
                    BootstrapUnit::IndependentSystem => dataset.systems.len(),
                }
            );
            assert!(a.lower <= a.upper);
        }
    }

    #[test]
    fn basic_interval_reflects_percentile_endpoints_about_the_paired_estimate() {
        let dataset = generate_dataset(&DgpConfig::default(), 731).unwrap();
        let percentile = bootstrap_interval(&dataset, BootstrapUnit::IndependentSystem, 0.95, 500, 88).unwrap();
        let basic = basic_bootstrap_interval(&dataset, BootstrapUnit::IndependentSystem, 0.95, 500, 88).unwrap();
        close(basic.lower, 2.0 * percentile.estimate - percentile.upper, 1e-12);
        close(basic.upper, 2.0 * percentile.estimate - percentile.lower, 1e-12);
        assert!(basic.lower <= basic.upper);
    }

    #[test]
    fn studentized_system_interval_is_deterministic_and_reports_degenerate_draws() {
        let dataset = generate_dataset(&DgpConfig::default(), 812).unwrap();
        let first = studentized_system_bootstrap_interval(&dataset, 0.95, 500, 992).unwrap();
        let second = studentized_system_bootstrap_interval(&dataset, 0.95, 500, 992).unwrap();
        assert_eq!(first, second);
        assert!(first.lower <= first.upper);
        assert_eq!(first.valid_replicates + first.abstentions, first.requested_replicates);
        assert!(first.valid_replicates >= 400);
        assert_eq!(first.independent_systems, 40);
    }

    #[test]
    fn coverage_study_reports_mcse_wilson_interval_and_abstentions() {
        let results = coverage_study(&DgpConfig::default(), 25, 100, 0.95, 991).unwrap();
        assert_eq!(results.len(), 3);
        for result in results {
            assert_eq!(result.valid_intervals + result.abstentions, 25);
            if let (Some(low), Some(high)) = (
                result.coverage_wilson_95_low,
                result.coverage_wilson_95_high,
            ) {
                assert!(low <= high);
            }
        }
    }

    #[test]
    fn alternative_coverage_study_reports_basic_and_studentized_methods_and_abstentions() {
        let report = alternative_coverage_study(&DgpConfig::default(), 20, 100, 0.95, 8821).unwrap();
        assert_eq!(report.len(), 2);
        for method in report {
            assert_eq!(method.valid_intervals + method.abstentions, 20);
            if let (Some(low), Some(high)) = (method.coverage_wilson_95_low, method.coverage_wilson_95_high) {
                assert!(low <= high);
            }
        }
    }

    #[test]
    fn coverage_study_rejects_invalid_confidence_instead_of_reporting_abstentions() {
        assert!(coverage_study(&DgpConfig::default(), 10, 100, 1.0, 7).is_err());
        assert!(coverage_study(&DgpConfig::default(), 10, 100, f64::NAN, 7).is_err());
    }

    #[test]
    fn invalid_dgp_parameters_fail_closed() {
        let mut config = DgpConfig::default();
        config.high_risk_probability = f64::NAN;
        assert!(config.validate().is_err());
        config.high_risk_probability = 0.2;
        config.independent_systems = 1;
        assert!(config.validate().is_err());
    }

    #[test]
    fn invalid_dataset_parent_link_is_rejected() {
        let mut dataset = generate_dataset(&DgpConfig::default(), 101).unwrap();
        dataset.systems[0].events[0].cases[0].system_id = "wrong-parent".into();
        assert!(dataset.validate().unwrap_err().contains("parent mismatch"));
    }

    #[test]
    fn too_few_bootstrap_replicates_are_rejected() {
        let dataset = generate_dataset(&DgpConfig::default(), 19).unwrap();
        assert!(bootstrap_interval(
            &dataset,
            BootstrapUnit::IndependentSystem,
            0.95,
            99,
            1
        )
        .is_err());
    }
}

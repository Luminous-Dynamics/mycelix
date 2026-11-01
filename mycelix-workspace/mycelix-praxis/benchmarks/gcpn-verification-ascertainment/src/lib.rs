//! Outcome-ascertainment qualification for GCPN forecast verification.
//!
//! This crate evaluates the observation process behind a verification sample.
//! It does not produce forecasts, impute individual outcomes, issue warnings,
//! infer causal effects, or establish that a propensity model is correct.
//!
//! Missing outcomes remain typed missing states. The engine reports observed-
//! case estimates, sharp finite-frame bounds, assumption-dependent IPW
//! estimates, caller-supplied sensitivity scenarios, and stratum coverage.
//! Sampling uncertainty for correlated systems belongs to the separate
//! independent-system bootstrap programme.

use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScoreBounds {
    pub lower: f64,
    pub upper: f64,
}

impl ScoreBounds {
    pub fn validate(&self) -> Result<(), String> {
        if !self.lower.is_finite() || !self.upper.is_finite() {
            return Err("score bounds must be finite".into());
        }
        if self.lower > self.upper {
            return Err("score lower bound must not exceed upper bound".into());
        }
        Ok(())
    }

    fn contains(&self, value: f64) -> bool {
        value.is_finite() && value >= self.lower && value <= self.upper
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Metric {
    Candidate,
    Reference,
    CandidateMinusReference,
}

impl Metric {
    pub fn label(self) -> &'static str {
        match self {
            Self::Candidate => "candidate_score",
            Self::Reference => "reference_score",
            Self::CandidateMinusReference => "candidate_minus_reference",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ComparisonMode {
    Single {
        model_id: String,
    },
    Paired {
        candidate_model_id: String,
        reference_model_id: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum MissingReason {
    PendingAtCutoff,
    NotCollected,
    SensorUnavailable,
    CommunicationUnavailable,
    QualityRejected,
    Disputed,
    AccessConstrained,
    Unknown,
}

impl MissingReason {
    pub fn label(&self) -> &'static str {
        match self {
            Self::PendingAtCutoff => "pending_at_cutoff",
            Self::NotCollected => "not_collected",
            Self::SensorUnavailable => "sensor_unavailable",
            Self::CommunicationUnavailable => "communication_unavailable",
            Self::QualityRejected => "quality_rejected",
            Self::Disputed => "disputed",
            Self::AccessConstrained => "access_constrained",
            Self::Unknown => "unknown_reason",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ObservedScores {
    /// Both model outputs must be keyed to the same canonical eligible case.
    pub candidate_case_id: String,
    pub candidate_score: f64,
    pub reference_case_id: Option<String>,
    pub reference_score: Option<f64>,
    /// Verification source and version for the common outcome used by scores.
    pub verification_source: String,
    pub verification_version: String,
    /// Transaction/knowledge time at which the verified score became available.
    pub recorded_at: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum OutcomeStatus {
    Observed(ObservedScores),
    Missing { reason: MissingReason },
}

#[derive(Debug, Clone, PartialEq)]
pub struct EligibleCase {
    pub case_id: String,
    pub independent_system_id: String,
    /// Predeclared, coarse, non-identifying stratum label.
    pub stratum_id: String,
    /// Declared probability that this eligible outcome would be verified.
    /// It is not inferred or validated by this crate.
    pub inclusion_probability: Option<f64>,
    pub outcome: OutcomeStatus,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EvaluationFrame {
    pub target_frame_id: String,
    pub score_name: String,
    pub verification_source: String,
    pub verification_version: String,
    pub comparison: ComparisonMode,
    pub score_bounds: ScoreBounds,
    pub as_of_cutoff: u64,
    pub propensity_model_id: Option<String>,
    pub cases: Vec<EligibleCase>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PopulationBounds {
    pub lower: f64,
    pub upper: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MetricSummary {
    pub metric: Metric,
    pub score_bounds: ScoreBounds,
    pub eligible_cases: usize,
    pub observed_cases: usize,
    pub missing_cases: usize,
    pub observed_score_sum: f64,
    pub observed_mean: Option<f64>,
    /// Sharp finite-frame bounds requiring only the declared score range.
    pub population_bounds: PopulationBounds,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StratumReport {
    pub stratum_id: String,
    pub eligible_cases: usize,
    pub observed_cases: usize,
    pub missing_cases: usize,
    pub coverage_rate: f64,
    pub missing_by_reason: BTreeMap<MissingReason, usize>,
    pub candidate: MetricSummary,
    pub reference: Option<MetricSummary>,
    pub paired_difference: Option<MetricSummary>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FrameReport {
    pub target_frame_id: String,
    pub score_name: String,
    pub verification_source: String,
    pub verification_version: String,
    pub as_of_cutoff: u64,
    pub eligible_cases: usize,
    pub observed_cases: usize,
    pub missing_cases: usize,
    pub coverage_rate: f64,
    pub independent_systems: usize,
    pub missing_by_reason: BTreeMap<MissingReason, usize>,
    pub candidate: MetricSummary,
    pub reference: Option<MetricSummary>,
    pub paired_difference: Option<MetricSummary>,
    pub strata: Vec<StratumReport>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct IpwEstimate {
    pub metric: Metric,
    pub propensity_model_id: String,
    pub minimum_inclusion_probability: f64,
    pub minimum_observed_inclusion_probability: f64,
    pub maximum_observed_inverse_weight: f64,
    pub horvitz_thompson_mean: f64,
    pub hajek_mean: f64,
    pub observed_weight_sum: f64,
    pub effective_sample_size: f64,
    pub eligible_cases: usize,
    pub observed_cases: usize,
    pub identifying_assumption: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensitivityPoint {
    pub delta: f64,
    pub assumed_missing_mean: f64,
    pub population_mean: f64,
}

impl EvaluationFrame {
    pub fn validate(&self) -> Result<(), String> {
        for (name, value) in [
            ("target_frame_id", self.target_frame_id.as_str()),
            ("score_name", self.score_name.as_str()),
            ("verification_source", self.verification_source.as_str()),
            ("verification_version", self.verification_version.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(format!("{name} must not be empty"));
            }
        }

        self.score_bounds.validate()?;

        match &self.comparison {
            ComparisonMode::Single { model_id } => {
                if model_id.trim().is_empty() {
                    return Err("single-model identifier must not be empty".into());
                }
            }
            ComparisonMode::Paired {
                candidate_model_id,
                reference_model_id,
            } => {
                if candidate_model_id.trim().is_empty()
                    || reference_model_id.trim().is_empty()
                {
                    return Err("paired candidate and reference model identifiers are required".into());
                }
                if candidate_model_id == reference_model_id {
                    return Err("candidate and reference model identifiers must differ".into());
                }
            }
        }

        if self.cases.is_empty() {
            return Err("target frame must contain at least one eligible case".into());
        }

        if let Some(model_id) = &self.propensity_model_id {
            if model_id.trim().is_empty() {
                return Err("propensity model identifier must not be empty when supplied".into());
            }
        }

        let mut case_ids = BTreeSet::new();
        for case in &self.cases {
            if case.case_id.trim().is_empty()
                || case.independent_system_id.trim().is_empty()
                || case.stratum_id.trim().is_empty()
            {
                return Err("case, independent-system and stratum identifiers are required".into());
            }
            if !case_ids.insert(case.case_id.as_str()) {
                return Err(format!("duplicate eligible case id: {}", case.case_id));
            }

            if let Some(probability) = case.inclusion_probability {
                if !probability.is_finite() || !(0.0..=1.0).contains(&probability) {
                    return Err(format!(
                        "case {} has an inclusion probability outside [0, 1]",
                        case.case_id
                    ));
                }
            }

            if let OutcomeStatus::Observed(scores) = &case.outcome {
                if scores.candidate_case_id != case.case_id {
                    return Err(format!(
                        "candidate score case id does not match eligible case {}",
                        case.case_id
                    ));
                }
                if !self.score_bounds.contains(scores.candidate_score) {
                    return Err(format!(
                        "candidate score for case {} is outside declared score bounds",
                        case.case_id
                    ));
                }
                if scores.verification_source != self.verification_source
                    || scores.verification_version != self.verification_version
                {
                    return Err(format!(
                        "verification source/version mismatch for case {}",
                        case.case_id
                    ));
                }
                if scores.recorded_at > self.as_of_cutoff {
                    return Err(format!(
                        "case {} was verified after the selected as-of cutoff",
                        case.case_id
                    ));
                }

                match &self.comparison {
                    ComparisonMode::Single { .. } => {
                        if scores.reference_case_id.is_some() || scores.reference_score.is_some() {
                            return Err(format!(
                                "single-model frame cannot contain reference scores for case {}",
                                case.case_id
                            ));
                        }
                    }
                    ComparisonMode::Paired { .. } => {
                        if scores.reference_case_id.as_deref() != Some(case.case_id.as_str()) {
                            return Err(format!(
                                "reference score case id does not match eligible case {}",
                                case.case_id
                            ));
                        }
                        let reference_score = scores.reference_score.ok_or_else(|| {
                            format!("paired reference score is missing for case {}", case.case_id)
                        })?;
                        if !self.score_bounds.contains(reference_score) {
                            return Err(format!(
                                "reference score for case {} is outside declared score bounds",
                                case.case_id
                            ));
                        }
                    }
                }
            }
        }

        Ok(())
    }

    fn metric_bounds(&self, metric: Metric) -> Result<ScoreBounds, String> {
        match (&self.comparison, metric) {
            (ComparisonMode::Single { .. }, Metric::Candidate) => Ok(self.score_bounds),
            (ComparisonMode::Single { .. }, _) => {
                Err("reference and paired-difference metrics require a paired frame".into())
            }
            (ComparisonMode::Paired { .. }, Metric::Candidate | Metric::Reference) => {
                Ok(self.score_bounds)
            }
            (ComparisonMode::Paired { .. }, Metric::CandidateMinusReference) => {
                Ok(ScoreBounds {
                    lower: self.score_bounds.lower - self.score_bounds.upper,
                    upper: self.score_bounds.upper - self.score_bounds.lower,
                })
            }
        }
    }

    fn observed_score(&self, case: &EligibleCase, metric: Metric) -> Option<f64> {
        let OutcomeStatus::Observed(scores) = &case.outcome else {
            return None;
        };

        match metric {
            Metric::Candidate => Some(scores.candidate_score),
            Metric::Reference => scores.reference_score,
            Metric::CandidateMinusReference => {
                scores.reference_score.map(|reference| scores.candidate_score - reference)
            }
        }
    }

    fn summarize_cases(
        &self,
        cases: &[&EligibleCase],
        metric: Metric,
    ) -> Result<MetricSummary, String> {
        if cases.is_empty() {
            return Err("cannot summarize an empty target frame or stratum".into());
        }

        let bounds = self.metric_bounds(metric)?;
        let mut score_sum = 0.0;
        let mut observed_cases = 0usize;
        for case in cases {
            if let Some(score) = self.observed_score(case, metric) {
                score_sum += score;
                observed_cases += 1;
            }
        }

        let eligible_cases = cases.len();
        let missing_cases = eligible_cases - observed_cases;
        let missing_lower = missing_cases as f64 * bounds.lower;
        let missing_upper = missing_cases as f64 * bounds.upper;
        let population_bounds = PopulationBounds {
            lower: (score_sum + missing_lower) / eligible_cases as f64,
            upper: (score_sum + missing_upper) / eligible_cases as f64,
        };

        Ok(MetricSummary {
            metric,
            score_bounds: bounds,
            eligible_cases,
            observed_cases,
            missing_cases,
            observed_score_sum: score_sum,
            observed_mean: if observed_cases == 0 {
                None
            } else {
                Some(score_sum / observed_cases as f64)
            },
            population_bounds,
        })
    }

    fn missing_reason_counts(cases: &[&EligibleCase]) -> BTreeMap<MissingReason, usize> {
        let mut counts = BTreeMap::new();
        for case in cases {
            if let OutcomeStatus::Missing { reason } = &case.outcome {
                *counts.entry(reason.clone()).or_insert(0) += 1;
            }
        }
        counts
    }

    fn stratum_report(
        &self,
        stratum_id: String,
        cases: &[&EligibleCase],
    ) -> Result<StratumReport, String> {
        let candidate = self.summarize_cases(cases, Metric::Candidate)?;
        let (reference, paired_difference) = match &self.comparison {
            ComparisonMode::Single { .. } => (None, None),
            ComparisonMode::Paired { .. } => (
                Some(self.summarize_cases(cases, Metric::Reference)?),
                Some(self.summarize_cases(cases, Metric::CandidateMinusReference)?),
            ),
        };
        let observed_cases = candidate.observed_cases;
        let eligible_cases = cases.len();
        Ok(StratumReport {
            stratum_id,
            eligible_cases,
            observed_cases,
            missing_cases: eligible_cases - observed_cases,
            coverage_rate: observed_cases as f64 / eligible_cases as f64,
            missing_by_reason: Self::missing_reason_counts(cases),
            candidate,
            reference,
            paired_difference,
        })
    }

    /// Build whole-frame and predeclared-stratum summaries.
    pub fn report(&self) -> Result<FrameReport, String> {
        self.validate()?;
        let all_cases: Vec<&EligibleCase> = self.cases.iter().collect();
        let candidate = self.summarize_cases(&all_cases, Metric::Candidate)?;
        let (reference, paired_difference) = match &self.comparison {
            ComparisonMode::Single { .. } => (None, None),
            ComparisonMode::Paired { .. } => (
                Some(self.summarize_cases(&all_cases, Metric::Reference)?),
                Some(self.summarize_cases(&all_cases, Metric::CandidateMinusReference)?),
            ),
        };

        let observed_cases = candidate.observed_cases;
        let eligible_cases = self.cases.len();
        let independent_systems = self
            .cases
            .iter()
            .map(|case| case.independent_system_id.as_str())
            .collect::<BTreeSet<_>>()
            .len();

        let mut groups: BTreeMap<String, Vec<&EligibleCase>> = BTreeMap::new();
        for case in &self.cases {
            groups.entry(case.stratum_id.clone()).or_default().push(case);
        }

        let mut strata = Vec::with_capacity(groups.len());
        for (stratum_id, cases) in groups {
            strata.push(self.stratum_report(stratum_id, &cases)?);
        }

        Ok(FrameReport {
            target_frame_id: self.target_frame_id.clone(),
            score_name: self.score_name.clone(),
            verification_source: self.verification_source.clone(),
            verification_version: self.verification_version.clone(),
            as_of_cutoff: self.as_of_cutoff,
            eligible_cases,
            observed_cases,
            missing_cases: eligible_cases - observed_cases,
            coverage_rate: observed_cases as f64 / eligible_cases as f64,
            independent_systems,
            missing_by_reason: Self::missing_reason_counts(&all_cases),
            candidate,
            reference,
            paired_difference,
            strata,
        })
    }

    /// Assumption-dependent inverse-probability estimates for one score.
    ///
    /// The caller must supply a predeclared positivity floor. No weights are
    /// clipped. A missing propensity or a probability below the floor disables
    /// the estimate; the caller should retain the complete-case result and
    /// assumption-light bounds instead.
    pub fn ipw(
        &self,
        metric: Metric,
        minimum_inclusion_probability: f64,
    ) -> Result<IpwEstimate, String> {
        self.validate()?;
        self.metric_bounds(metric)?;

        if !minimum_inclusion_probability.is_finite()
            || minimum_inclusion_probability <= 0.0
            || minimum_inclusion_probability > 1.0
        {
            return Err("declared positivity floor must be finite and in (0, 1]".into());
        }

        let propensity_model_id = self
            .propensity_model_id
            .as_ref()
            .filter(|value| !value.trim().is_empty())
            .ok_or_else(|| "IPW requires a declared propensity model identifier".to_string())?;

        let mut weighted_score_sum = 0.0;
        let mut weight_sum = 0.0;
        let mut squared_weight_sum = 0.0;
        let mut minimum_observed_probability = 1.0_f64;
        let mut maximum_inverse_weight = 0.0_f64;
        let mut observed_cases = 0usize;

        for case in &self.cases {
            let probability = case.inclusion_probability.ok_or_else(|| {
                format!("case {} has no declared inclusion probability", case.case_id)
            })?;
            if probability < minimum_inclusion_probability {
                return Err(format!(
                    "case {} violates the declared positivity floor",
                    case.case_id
                ));
            }

            if let Some(score) = self.observed_score(case, metric) {
                let weight = 1.0 / probability;
                weighted_score_sum += weight * score;
                weight_sum += weight;
                squared_weight_sum += weight * weight;
                minimum_observed_probability =
                    minimum_observed_probability.min(probability);
                maximum_inverse_weight = maximum_inverse_weight.max(weight);
                observed_cases += 1;
            }
        }

        if observed_cases == 0 || weight_sum <= 0.0 {
            return Err("IPW requires at least one verified outcome".into());
        }
        if !weighted_score_sum.is_finite()
            || !weight_sum.is_finite()
            || !squared_weight_sum.is_finite()
            || squared_weight_sum <= 0.0
        {
            return Err("IPW arithmetic overflowed or became non-finite".into());
        }

        Ok(IpwEstimate {
            metric,
            propensity_model_id: propensity_model_id.clone(),
            minimum_inclusion_probability,
            minimum_observed_inclusion_probability: minimum_observed_probability,
            maximum_observed_inverse_weight: maximum_inverse_weight,
            horvitz_thompson_mean: weighted_score_sum / self.cases.len() as f64,
            hajek_mean: weighted_score_sum / weight_sum,
            observed_weight_sum: weight_sum,
            effective_sample_size: weight_sum * weight_sum / squared_weight_sum,
            eligible_cases: self.cases.len(),
            observed_cases,
            identifying_assumption:
                "verification is conditionally ignorable given the declared propensity covariates and the eligible frame is complete",
        })
    }

    /// Evaluate only the caller-supplied pattern-mixture deltas.
    ///
    /// Positive delta means the assumed missing-case score is worse when the
    /// metric is a loss. This method never selects or optimizes a delta.
    pub fn sensitivity_grid(
        &self,
        metric: Metric,
        deltas: &[f64],
    ) -> Result<Vec<SensitivityPoint>, String> {
        self.validate()?;
        let bounds = self.metric_bounds(metric)?;
        if deltas.is_empty() {
            return Err("sensitivity delta grid must not be empty".into());
        }

        let all_cases: Vec<&EligibleCase> = self.cases.iter().collect();
        let summary = self.summarize_cases(&all_cases, metric)?;
        let observed_mean = summary
            .observed_mean
            .ok_or_else(|| "pattern-mixture sensitivity requires at least one observed score".to_string())?;

        let mut output = Vec::with_capacity(deltas.len());
        for delta in deltas {
            if !delta.is_finite() {
                return Err("sensitivity deltas must be finite".into());
            }
            let assumed_missing_mean = (observed_mean + delta).clamp(bounds.lower, bounds.upper);
            let population_mean = (summary.observed_score_sum
                + summary.missing_cases as f64 * assumed_missing_mean)
                / summary.eligible_cases as f64;
            output.push(SensitivityPoint {
                delta: *delta,
                assumed_missing_mean,
                population_mean,
            });
        }
        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observed(
        case_id: &str,
        candidate_score: f64,
        reference_score: Option<f64>,
        recorded_at: u64,
    ) -> OutcomeStatus {
        OutcomeStatus::Observed(ObservedScores {
            candidate_case_id: case_id.to_string(),
            candidate_score,
            reference_case_id: reference_score.map(|_| case_id.to_string()),
            reference_score,
            verification_source: "fixture-observations".into(),
            verification_version: "v1".into(),
            recorded_at,
        })
    }

    fn missing(reason: MissingReason) -> OutcomeStatus {
        OutcomeStatus::Missing { reason }
    }

    fn case(
        id: &str,
        system: &str,
        stratum: &str,
        probability: Option<f64>,
        outcome: OutcomeStatus,
    ) -> EligibleCase {
        EligibleCase {
            case_id: id.into(),
            independent_system_id: system.into(),
            stratum_id: stratum.into(),
            inclusion_probability: probability,
            outcome,
        }
    }

    fn paired_frame(cases: Vec<EligibleCase>) -> EvaluationFrame {
        EvaluationFrame {
            target_frame_id: "frame-2026-01".into(),
            score_name: "brier_loss".into(),
            verification_source: "fixture-observations".into(),
            verification_version: "v1".into(),
            comparison: ComparisonMode::Paired {
                candidate_model_id: "candidate".into(),
                reference_model_id: "reference".into(),
            },
            score_bounds: ScoreBounds {
                lower: 0.0,
                upper: 1.0,
            },
            as_of_cutoff: 100,
            propensity_model_id: Some("fixture-propensity-v1".into()),
            cases,
        }
    }

    fn single_frame(cases: Vec<EligibleCase>) -> EvaluationFrame {
        let mut frame = paired_frame(cases);
        frame.comparison = ComparisonMode::Single {
            model_id: "candidate".into(),
        };
        frame
    }

    #[test]
    fn paired_report_keeps_candidate_reference_and_difference_separate() {
        let frame = paired_frame(vec![
            case("a", "sys-1", "urban", Some(1.0), observed("a", 0.2, Some(0.1), 90)),
            case("b", "sys-1", "urban", Some(1.0), observed("b", 0.4, Some(0.6), 91)),
            case("c", "sys-2", "remote", Some(0.5), missing(MissingReason::SensorUnavailable)),
        ]);

        let report = frame.report().unwrap();
        assert_eq!(report.eligible_cases, 3);
        assert_eq!(report.observed_cases, 2);
        assert_eq!(report.missing_cases, 1);
        assert!((report.coverage_rate - 2.0 / 3.0).abs() < 1e-12);
        assert!((report.candidate.observed_mean.unwrap() - 0.3).abs() < 1e-12);
        assert!((report.candidate.population_bounds.lower - 0.2).abs() < 1e-12);
        assert!((report.candidate.population_bounds.upper - (0.6 + 1.0) / 3.0).abs() < 1e-12);

        let reference = report.reference.unwrap();
        assert!((reference.observed_mean.unwrap() - 0.35).abs() < 1e-12);
        let difference = report.paired_difference.unwrap();
        assert!((difference.observed_mean.unwrap() - (-0.05)).abs() < 1e-12);
        assert!(difference.population_bounds.lower < 0.0);
        assert!(difference.population_bounds.upper > 0.0);
    }

    #[test]
    fn complete_verification_collapses_bounds_to_observed_mean() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 0.2, None, 90)),
            case("b", "sys-2", "zone-b", Some(1.0), observed("b", 0.6, None, 91)),
        ]);
        let summary = frame.report().unwrap().candidate;
        assert!((summary.observed_mean.unwrap() - 0.4).abs() < 1e-12);
        assert!((summary.population_bounds.lower - 0.4).abs() < 1e-12);
        assert!((summary.population_bounds.upper - 0.4).abs() < 1e-12);
    }

    #[test]
    fn all_missing_frame_has_bounds_but_no_point_estimate() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(0.2), missing(MissingReason::NotCollected)),
            case("b", "sys-2", "zone-b", Some(0.1), missing(MissingReason::Unknown)),
        ]);
        let summary = frame.report().unwrap().candidate;
        assert_eq!(summary.observed_mean, None);
        assert_eq!(summary.population_bounds, PopulationBounds { lower: 0.0, upper: 1.0 });
    }

    #[test]
    fn missing_reasons_remain_distinct() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(0.5), missing(MissingReason::PendingAtCutoff)),
            case("b", "sys-1", "zone-a", Some(0.5), missing(MissingReason::CommunicationUnavailable)),
            case("c", "sys-2", "zone-b", Some(0.5), missing(MissingReason::Disputed)),
        ]);
        let counts = frame.report().unwrap().missing_by_reason;
        assert_eq!(counts.get(&MissingReason::PendingAtCutoff), Some(&1));
        assert_eq!(counts.get(&MissingReason::CommunicationUnavailable), Some(&1));
        assert_eq!(counts.get(&MissingReason::Disputed), Some(&1));
    }

    #[test]
    fn zero_observation_stratum_has_no_mean_and_full_bounds() {
        let frame = single_frame(vec![
            case("a", "sys-1", "observed", Some(1.0), observed("a", 0.25, None, 90)),
            case("b", "sys-2", "unobserved", Some(0.1), missing(MissingReason::SensorUnavailable)),
            case("c", "sys-2", "unobserved", Some(0.1), missing(MissingReason::SensorUnavailable)),
        ]);
        let report = frame.report().unwrap();
        let stratum = report.strata.iter().find(|entry| entry.stratum_id == "unobserved").unwrap();
        assert_eq!(stratum.candidate.observed_mean, None);
        assert_eq!(stratum.candidate.population_bounds, PopulationBounds { lower: 0.0, upper: 1.0 });
        assert_eq!(stratum.coverage_rate, 0.0);
    }

    #[test]
    fn observations_recorded_after_cutoff_are_rejected() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 0.2, None, 101)),
        ]);
        assert!(frame.validate().unwrap_err().contains("after the selected as-of cutoff"));
    }

    #[test]
    fn candidate_score_outside_bounds_is_rejected() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 1.1, None, 90)),
        ]);
        assert!(frame.validate().unwrap_err().contains("outside declared score bounds"));
    }

    #[test]
    fn duplicate_eligible_case_ids_are_rejected() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 0.2, None, 90)),
            case("a", "sys-2", "zone-b", Some(1.0), missing(MissingReason::NotCollected)),
        ]);
        assert!(frame.validate().unwrap_err().contains("duplicate eligible case id"));
    }

    #[test]
    fn candidate_case_key_must_match_eligible_frame_key() {
        let mut scores = match observed("a", 0.2, Some(0.3), 90) {
            OutcomeStatus::Observed(value) => value,
            _ => unreachable!(),
        };
        scores.candidate_case_id = "different".into();
        let frame = paired_frame(vec![case("a", "sys-1", "zone-a", Some(1.0), OutcomeStatus::Observed(scores))]);
        assert!(frame.validate().unwrap_err().contains("candidate score case id"));
    }

    #[test]
    fn reference_case_key_must_match_candidate_case_key() {
        let mut scores = match observed("a", 0.2, Some(0.3), 90) {
            OutcomeStatus::Observed(value) => value,
            _ => unreachable!(),
        };
        scores.reference_case_id = Some("different".into());
        let frame = paired_frame(vec![case("a", "sys-1", "zone-a", Some(1.0), OutcomeStatus::Observed(scores))]);
        assert!(frame.validate().unwrap_err().contains("reference score case id"));
    }

    #[test]
    fn verification_source_and_version_must_match_the_frozen_frame() {
        let mut scores = match observed("a", 0.2, Some(0.3), 90) {
            OutcomeStatus::Observed(value) => value,
            _ => unreachable!(),
        };
        scores.verification_version = "v2".into();
        let frame = paired_frame(vec![case("a", "sys-1", "zone-a", Some(1.0), OutcomeStatus::Observed(scores))]);
        assert!(frame.validate().unwrap_err().contains("verification source/version mismatch"));
    }

    #[test]
    fn paired_mode_requires_a_reference_score_for_every_observed_case() {
        let mut scores = match observed("a", 0.2, None, 90) {
            OutcomeStatus::Observed(value) => value,
            _ => unreachable!(),
        };
        scores.reference_case_id = Some("a".into());
        let frame = paired_frame(vec![case(
            "a",
            "sys-1",
            "zone-a",
            Some(1.0),
            OutcomeStatus::Observed(scores),
        )]);
        assert!(frame.validate().unwrap_err().contains("reference score is missing"));
    }

    #[test]
    fn single_mode_rejects_reference_scores() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 0.2, Some(0.3), 90)),
        ]);
        assert!(frame.validate().unwrap_err().contains("single-model frame"));
    }

    #[test]
    fn paired_difference_is_candidate_minus_reference() {
        let frame = paired_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 0.2, Some(0.7), 90)),
        ]);
        let summary = frame.report().unwrap().paired_difference.unwrap();
        assert!((summary.observed_mean.unwrap() + 0.5).abs() < 1e-12);
        assert!(summary.population_bounds.lower <= -0.5);
        assert!(summary.population_bounds.upper >= -0.5);
    }

    #[test]
    fn ht_and_hajek_match_constant_propensity_oracle() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(0.5), observed("a", 0.2, None, 90)),
            case("b", "sys-1", "zone-a", Some(0.5), observed("b", 0.8, None, 91)),
            case("c", "sys-2", "zone-b", Some(0.5), missing(MissingReason::NotCollected)),
            case("d", "sys-2", "zone-b", Some(0.5), missing(MissingReason::SensorUnavailable)),
        ]);
        let estimate = frame.ipw(Metric::Candidate, 0.1).unwrap();
        assert!((estimate.horvitz_thompson_mean - 0.5).abs() < 1e-12);
        assert!((estimate.hajek_mean - 0.5).abs() < 1e-12);
        assert!((estimate.effective_sample_size - 2.0).abs() < 1e-12);
        assert_eq!(estimate.observed_cases, 2);
        assert_eq!(estimate.eligible_cases, 4);
    }

    #[test]
    fn ipw_refuses_missing_propensity_values() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(0.5), observed("a", 0.2, None, 90)),
            case("b", "sys-2", "zone-b", None, missing(MissingReason::NotCollected)),
        ]);
        assert!(frame.ipw(Metric::Candidate, 0.1).unwrap_err().contains("no declared inclusion probability"));
    }

    #[test]
    fn ipw_refuses_zero_or_below_floor_propensity() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(0.5), observed("a", 0.2, None, 90)),
            case("b", "sys-2", "zone-b", Some(0.0), missing(MissingReason::NotCollected)),
        ]);
        assert!(frame.ipw(Metric::Candidate, 0.01).unwrap_err().contains("positivity floor"));
    }

    #[test]
    fn ipw_requires_a_named_propensity_model() {
        let mut frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 0.2, None, 90)),
        ]);
        frame.propensity_model_id = None;
        assert!(frame.ipw(Metric::Candidate, 0.01).unwrap_err().contains("propensity model identifier"));
    }

    #[test]
    fn ipw_reports_effective_sample_size_for_unequal_weights() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(0.25), observed("a", 0.2, None, 90)),
            case("b", "sys-2", "zone-b", Some(0.75), observed("b", 0.8, None, 91)),
            case("c", "sys-3", "zone-c", Some(0.5), missing(MissingReason::NotCollected)),
        ]);
        let estimate = frame.ipw(Metric::Candidate, 0.1).unwrap();
        assert!((estimate.effective_sample_size - 1.6).abs() < 1e-12);
        assert!((estimate.maximum_observed_inverse_weight - 4.0).abs() < 1e-12);
    }

    #[test]
    fn ipw_is_not_clipped_to_the_score_range() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(0.1), observed("a", 0.8, None, 90)),
            case("b", "sys-2", "zone-b", Some(0.9), missing(MissingReason::NotCollected)),
        ]);
        let estimate = frame.ipw(Metric::Candidate, 0.05).unwrap();
        assert!(estimate.horvitz_thompson_mean > 1.0);
    }

    #[test]
    fn sensitivity_grid_evaluates_only_the_supplied_deltas() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 0.2, None, 90)),
            case("b", "sys-2", "zone-b", Some(0.5), observed("b", 0.4, None, 91)),
            case("c", "sys-3", "zone-c", Some(0.2), missing(MissingReason::NotCollected)),
            case("d", "sys-4", "zone-d", Some(0.2), missing(MissingReason::Unknown)),
        ]);
        let points = frame.sensitivity_grid(Metric::Candidate, &[-0.2, 0.0, 0.2]).unwrap();
        assert_eq!(points.len(), 3);
        assert!((points[0].population_mean - 0.2).abs() < 1e-12);
        assert!((points[1].population_mean - 0.3).abs() < 1e-12);
        assert!((points[2].population_mean - 0.4).abs() < 1e-12);
        assert_eq!(points.iter().map(|point| point.delta).collect::<Vec<_>>(), vec![-0.2, 0.0, 0.2]);
    }

    #[test]
    fn sensitivity_assumptions_are_clipped_to_declared_score_bounds() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 0.8, None, 90)),
            case("b", "sys-2", "zone-b", Some(0.1), missing(MissingReason::Unknown)),
        ]);
        let point = frame.sensitivity_grid(Metric::Candidate, &[0.5]).unwrap()[0];
        assert_eq!(point.assumed_missing_mean, 1.0);
        assert!((point.population_mean - 0.9).abs() < 1e-12);
    }

    #[test]
    fn sensitivity_requires_at_least_one_observed_score() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(0.1), missing(MissingReason::NotCollected)),
        ]);
        assert!(frame.sensitivity_grid(Metric::Candidate, &[0.0]).unwrap_err().contains("at least one observed score"));
    }

    #[test]
    fn empty_sensitivity_grid_is_rejected() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 0.2, None, 90)),
        ]);
        assert!(frame.sensitivity_grid(Metric::Candidate, &[]).unwrap_err().contains("must not be empty"));
    }

    #[test]
    fn case_order_does_not_change_the_report() {
        let first = vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 0.2, Some(0.4), 90)),
            case("b", "sys-2", "zone-b", Some(0.5), missing(MissingReason::NotCollected)),
            case("c", "sys-3", "zone-a", Some(1.0), observed("c", 0.6, Some(0.2), 91)),
        ];
        let mut second = first.clone();
        second.reverse();
        assert_eq!(paired_frame(first).report().unwrap(), paired_frame(second).report().unwrap());
    }

    #[test]
    fn missing_probability_out_of_range_is_rejected() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.1), observed("a", 0.2, None, 90)),
        ]);
        assert!(frame.validate().unwrap_err().contains("inclusion probability outside [0, 1]"));
    }

    #[test]
    fn empty_target_frame_is_rejected() {
        let frame = single_frame(Vec::new());
        assert!(frame.validate().unwrap_err().contains("at least one eligible case"));
    }

    #[test]
    fn reference_and_paired_metrics_are_unavailable_for_single_model_frames() {
        let frame = single_frame(vec![
            case("a", "sys-1", "zone-a", Some(1.0), observed("a", 0.2, None, 90)),
        ]);
        assert!(frame.ipw(Metric::Reference, 0.1).unwrap_err().contains("paired frame"));
        assert!(frame.sensitivity_grid(Metric::CandidateMinusReference, &[0.0]).unwrap_err().contains("paired frame"));
    }

    #[test]
    fn declared_system_and_stratum_denominators_are_reported() {
        let frame = single_frame(vec![
            case("a", "sys-1", "urban", Some(1.0), observed("a", 0.2, None, 90)),
            case("b", "sys-1", "urban", Some(0.5), missing(MissingReason::NotCollected)),
            case("c", "sys-2", "remote", Some(0.2), missing(MissingReason::AccessConstrained)),
        ]);
        let report = frame.report().unwrap();
        assert_eq!(report.independent_systems, 2);
        let urban = report.strata.iter().find(|entry| entry.stratum_id == "urban").unwrap();
        assert_eq!(urban.eligible_cases, 2);
        assert_eq!(urban.observed_cases, 1);
        assert_eq!(urban.missing_cases, 1);
        assert_eq!(urban.coverage_rate, 0.5);
    }
}

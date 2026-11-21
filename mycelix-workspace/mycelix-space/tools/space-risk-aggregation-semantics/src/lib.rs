#![forbid(unsafe_code)]

//! Typed collision-risk aggregation semantics.
//!
//! The crate distinguishes event-level Pc, mission/fleet path risk,
//! long-term collision frequency, probability of detection (P_d), and
//! observation significance (p_obs). These are not interchangeable.
//!
//! If only event marginals are available on a compatible common probability
//! space, Frechet bounds are returned unless mutual independence is explicitly
//! supported by evidence. The product-survival formula is not silently
//! assumed. Prefer complete joint path scenarios; one joint scenario is one
//! independent statistical unit, irrespective of the number of events within
//! that path.

use std::collections::BTreeSet;
use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub struct EventLevelPc {
    pub event_id: String,
    pub value: f64,
    pub probability_space_id: String,
    pub conditioning_context_id: String,
    pub event_definition_id: String,
    pub event_horizon_id: String,
    pub source_model_id: String,
    pub source_model_revision: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FrechetUnionBounds {
    pub method: &'static str,
    pub event_count: usize,
    pub lower_bound: f64,
    pub upper_bound: f64,
    pub sum_of_marginals: f64,
    pub probability_space_id: String,
    pub conditioning_context_id: String,
    pub event_definition_id: String,
    pub event_horizon_id: String,
    pub source_model_id: String,
    pub source_model_revision: String,
    pub point_estimate: Option<f64>,
    pub dependence_status: &'static str,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MutualIndependenceEvidence {
    pub event_family_id: String,
    pub independence_model_id: String,
    pub evidence_ref: String,
    pub mutually_independent_events: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct IndependentProductRisk {
    pub method: &'static str,
    pub event_count: usize,
    pub mission_union_probability: f64,
    pub aggregate_survival_probability: f64,
    pub log_survival_probability: f64,
    pub event_family_id: String,
    pub independence_model_id: String,
    pub independence_evidence_ref: String,
    pub probability_space_id: String,
    pub conditioning_context_id: String,
    pub event_horizon_id: String,
    pub saturation_warning: bool,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointScenarioEvidence {
    pub scenario_set_id: String,
    pub scenario_sampling_design_id: String,
    pub scenario_sampling_evidence_ref: String,
    pub scenario_order_sha256: String,
    pub probability_space_id: String,
    pub conditioning_context_id: String,
    pub event_definition_id: String,
    pub event_horizon_id: String,
    pub joint_model_id: String,
    pub joint_model_revision: String,
    pub scenarios_are_iid_joint_paths: bool,
    pub all_shared_dependencies_are_included: bool,
    pub event_set_is_frozen_before_evaluation: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct JointScenario {
    pub scenario_id: String,
    /// Outcomes aligned with the frozen event_ids supplied to the estimator.
    pub event_outcomes: Vec<bool>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct JointPathRiskReport {
    pub method: &'static str,
    pub risk_quantity_type: &'static str,
    pub scenario_set_id: String,
    pub scenario_sampling_design_id: String,
    pub scenario_sampling_evidence_ref: String,
    pub scenario_order_sha256: String,
    pub probability_space_id: String,
    pub conditioning_context_id: String,
    pub event_definition_id: String,
    pub event_horizon_id: String,
    pub joint_model_id: String,
    pub joint_model_revision: String,
    pub event_count: usize,
    pub independent_joint_scenario_count: usize,
    pub total_event_outcomes: usize,
    pub scenarios_with_any_collision: usize,
    pub estimate_any_collision: f64,
    pub confidence_delta: f64,
    pub one_sided_confidence_level: f64,
    pub hoeffding_radius: f64,
    pub upper_confidence_bound: f64,
    pub statistical_unit: &'static str,
    pub within_scenario_event_outcomes_are_dependent: bool,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LongTermCollisionFrequency {
    pub collisions_observed: u64,
    pub exposure_object_years: f64,
    pub rate_per_object_year: f64,
    pub population_scope_id: String,
    pub source_model_id: String,
    pub source_model_revision: String,
    pub frequency_method_id: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ProbabilityOfDetection {
    pub value: f64,
    pub declared_pc_action_threshold: f64,
    pub policy_revision: String,
    pub calibration_dataset_id: String,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ObservationSignificance {
    pub p_value: f64,
    pub null_hypothesis_id: String,
    pub test_statistic_id: String,
    pub measurement_model_revision: String,
    pub significance_level: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub enum AggregationError {
    EmptyEventSet,
    InvalidIdentifier(&'static str),
    InvalidProbability {
        event_id: String,
        value: f64,
    },
    IncompatibleProbabilitySpaces,
    IncompatibleConditioningContexts,
    IncompatibleEventDefinitions,
    IncompatibleEventHorizons,
    IncompatibleSourceModelRevisions,
    DuplicateEventId(String),
    IndependenceEvidenceMissing,
    IndependenceNotAttested,
    EmptyJointScenarioSet,
    InsufficientJointScenarios(usize),
    DuplicateScenarioId(String),
    InvalidScenarioOrderDigest,
    JointScenarioEvidenceMissing,
    JointScenarioSamplingNotIid,
    JointScenarioDependenciesOmitted,
    JointScenarioEventSetNotFrozen,
    JointScenarioOutcomeCountMismatch {
        scenario_id: String,
        expected: usize,
        actual: usize,
    },
    InvalidDelta(f64),
    InvalidFrequencyExposure,
    InvalidProbabilityOfDetection,
    InvalidObservationSignificance,
    ArithmeticOverflow,
}

impl fmt::Display for AggregationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyEventSet => write!(f, "at least one event marginal is required"),
            Self::InvalidIdentifier(field) => write!(
                f,
                "{field} must be non-empty and use only ASCII letters, digits, dot, underscore, colon, slash and hyphen"
            ),
            Self::InvalidProbability { event_id, value } => write!(
                f,
                "event {event_id} has invalid probability {value}; expected a finite value in [0,1]"
            ),
            Self::IncompatibleProbabilitySpaces => write!(
                f,
                "event marginals do not share one declared probability space"
            ),
            Self::IncompatibleConditioningContexts => {
                write!(f, "event marginals have incompatible conditioning contexts")
            }
            Self::IncompatibleEventDefinitions => {
                write!(f, "event marginals use incompatible event definitions")
            }
            Self::IncompatibleEventHorizons => {
                write!(f, "event marginals use incompatible risk horizons")
            }
            Self::IncompatibleSourceModelRevisions => {
                write!(f, "event marginals use incompatible source model revisions")
            }
            Self::DuplicateEventId(value) => write!(f, "duplicate event ID {value}"),
            Self::IndependenceEvidenceMissing => write!(
                f,
                "mutual-independence evidence reference and event-family ID are required"
            ),
            Self::IndependenceNotAttested => write!(
                f,
                "product aggregation requires explicit mutual-independence evidence"
            ),
            Self::EmptyJointScenarioSet => {
                write!(f, "at least one complete joint scenario is required")
            }
            Self::InsufficientJointScenarios(count) => write!(
                f,
                "at least two independent joint scenarios are required; got {count}"
            ),
            Self::DuplicateScenarioId(value) => write!(f, "duplicate joint scenario ID {value}"),
            Self::InvalidScenarioOrderDigest => {
                write!(f, "scenario order digest must be lowercase SHA-256")
            }
            Self::JointScenarioEvidenceMissing => write!(
                f,
                "joint scenario set, probability space, conditioning, event, horizon and model evidence are required"
            ),
            Self::JointScenarioSamplingNotIid => write!(
                f,
                "complete joint scenarios must be IID under the declared joint model"
            ),
            Self::JointScenarioDependenciesOmitted => write!(
                f,
                "joint scenario model must include all declared shared dependencies"
            ),
            Self::JointScenarioEventSetNotFrozen => {
                write!(f, "the event set must be frozen before scenario evaluation")
            }
            Self::JointScenarioOutcomeCountMismatch {
                scenario_id,
                expected,
                actual,
            } => write!(
                f,
                "scenario {scenario_id} has {actual} event outcomes; expected {expected}"
            ),
            Self::InvalidDelta(value) => {
                write!(f, "delta must be finite and in (0,1); got {value}")
            }
            Self::InvalidFrequencyExposure => write!(
                f,
                "collision frequency requires a positive finite exposure and a non-negative finite rate"
            ),
            Self::InvalidProbabilityOfDetection => write!(
                f,
                "P_d and its threshold must be finite probabilities in [0,1], with model and policy provenance"
            ),
            Self::InvalidObservationSignificance => write!(
                f,
                "p_obs and significance level must be finite probabilities in [0,1], with a declared null and test"
            ),
            Self::ArithmeticOverflow => write!(f, "event/scenario count overflowed usize"),
        }
    }
}

impl std::error::Error for AggregationError {}

/// Validate that marginals describe events in one compatible probability space.
pub fn validate_event_marginals(events: &[EventLevelPc]) -> Result<(), AggregationError> {
    if events.is_empty() {
        return Err(AggregationError::EmptyEventSet);
    }
    let first = &events[0];
    let mut seen = BTreeSet::new();
    for event in events {
        for (field, value) in [
            ("event_id", &event.event_id),
            ("probability_space_id", &event.probability_space_id),
            ("conditioning_context_id", &event.conditioning_context_id),
            ("event_definition_id", &event.event_definition_id),
            ("event_horizon_id", &event.event_horizon_id),
            ("source_model_id", &event.source_model_id),
            ("source_model_revision", &event.source_model_revision),
        ] {
            if !valid_identifier(value) {
                return Err(AggregationError::InvalidIdentifier(field));
            }
        }
        if !event.value.is_finite() || !(0.0..=1.0).contains(&event.value) {
            return Err(AggregationError::InvalidProbability {
                event_id: event.event_id.clone(),
                value: event.value,
            });
        }
        if !seen.insert(event.event_id.as_str()) {
            return Err(AggregationError::DuplicateEventId(event.event_id.clone()));
        }
        if event.probability_space_id != first.probability_space_id {
            return Err(AggregationError::IncompatibleProbabilitySpaces);
        }
        if event.conditioning_context_id != first.conditioning_context_id {
            return Err(AggregationError::IncompatibleConditioningContexts);
        }
        if event.event_definition_id != first.event_definition_id {
            return Err(AggregationError::IncompatibleEventDefinitions);
        }
        if event.event_horizon_id != first.event_horizon_id {
            return Err(AggregationError::IncompatibleEventHorizons);
        }
        if event.source_model_id != first.source_model_id
            || event.source_model_revision != first.source_model_revision
        {
            return Err(AggregationError::IncompatibleSourceModelRevisions);
        }
    }
    Ok(())
}

/// Return sharp Frechet union bounds from compatible marginals without an
/// independence assumption. No point estimate is returned.
pub fn frechet_union_bounds(
    events: &[EventLevelPc],
) -> Result<FrechetUnionBounds, AggregationError> {
    validate_event_marginals(events)?;
    let lower = events.iter().map(|event| event.value).fold(0.0, f64::max);
    let sum = events.iter().map(|event| event.value).sum::<f64>();
    let upper = sum.min(1.0);
    let first = &events[0];
    Ok(FrechetUnionBounds {
        method: "frechet_bounds_unknown_event_dependence",
        event_count: events.len(),
        lower_bound: lower,
        upper_bound: upper,
        sum_of_marginals: sum,
        probability_space_id: first.probability_space_id.clone(),
        conditioning_context_id: first.conditioning_context_id.clone(),
        event_definition_id: first.event_definition_id.clone(),
        event_horizon_id: first.event_horizon_id.clone(),
        source_model_id: first.source_model_id.clone(),
        source_model_revision: first.source_model_revision.clone(),
        point_estimate: None,
        dependence_status: "UNKNOWN_DEPENDENCE_FRECHET_BOUNDS_ONLY",
        claim_ceiling: "sharp_union_probability_bounds_given_compatible_event_marginals_not_a_point_estimate",
    })
}

/// Compute product survival only after explicit mutual-independence evidence.
///
/// The formula is numerically evaluated as -expm1(sum(log1p(-p_i))) to avoid
/// catastrophic cancellation for small event probabilities.
pub fn independent_product_union_probability(
    events: &[EventLevelPc],
    evidence: &MutualIndependenceEvidence,
) -> Result<IndependentProductRisk, AggregationError> {
    validate_event_marginals(events)?;
    if !valid_identifier(&evidence.event_family_id)
        || !valid_identifier(&evidence.independence_model_id)
        || evidence.evidence_ref.trim().is_empty()
    {
        return Err(AggregationError::IndependenceEvidenceMissing);
    }
    if !evidence.mutually_independent_events {
        return Err(AggregationError::IndependenceNotAttested);
    }

    let mut log_survival = 0.0;
    for event in events {
        if event.value == 1.0 {
            log_survival = f64::NEG_INFINITY;
            break;
        }
        log_survival += (-event.value).ln_1p();
    }
    let survival = log_survival.exp();
    let union = if log_survival == f64::NEG_INFINITY {
        1.0
    } else {
        -log_survival.exp_m1()
    };
    let saturation_warning = union >= 1.0 - 1.0e-8 || survival <= 1.0e-8;
    let first = &events[0];

    Ok(IndependentProductRisk {
        method: "independent_event_product_survival",
        event_count: events.len(),
        mission_union_probability: union.clamp(0.0, 1.0),
        aggregate_survival_probability: survival.clamp(0.0, 1.0),
        log_survival_probability: log_survival,
        event_family_id: evidence.event_family_id.clone(),
        independence_model_id: evidence.independence_model_id.clone(),
        independence_evidence_ref: evidence.evidence_ref.clone(),
        probability_space_id: first.probability_space_id.clone(),
        conditioning_context_id: first.conditioning_context_id.clone(),
        event_horizon_id: first.event_horizon_id.clone(),
        saturation_warning,
        claim_ceiling: "product_survival_model_conditional_on_explicit_mutual_independence_evidence_not_long_term_frequency",
    })
}

/// Estimate P(any collision) from complete independent joint mission/fleet
/// scenarios. The Hoeffding bound uses the count of independent scenarios;
/// event outcomes within one scenario may be arbitrarily dependent.
pub fn joint_path_risk(
    event_ids: &[String],
    scenarios: &[JointScenario],
    delta: f64,
    evidence: &JointScenarioEvidence,
) -> Result<JointPathRiskReport, AggregationError> {
    validate_delta(delta)?;
    if event_ids.is_empty() {
        return Err(AggregationError::EmptyEventSet);
    }
    if scenarios.is_empty() {
        return Err(AggregationError::EmptyJointScenarioSet);
    }
    if scenarios.len() < 2 {
        return Err(AggregationError::InsufficientJointScenarios(
            scenarios.len(),
        ));
    }
    for (field, value) in [
        ("scenario_set_id", &evidence.scenario_set_id),
        (
            "scenario_sampling_design_id",
            &evidence.scenario_sampling_design_id,
        ),
        ("probability_space_id", &evidence.probability_space_id),
        ("conditioning_context_id", &evidence.conditioning_context_id),
        ("event_definition_id", &evidence.event_definition_id),
        ("event_horizon_id", &evidence.event_horizon_id),
        ("joint_model_id", &evidence.joint_model_id),
        ("joint_model_revision", &evidence.joint_model_revision),
    ] {
        if !valid_identifier(value) {
            return Err(AggregationError::InvalidIdentifier(field));
        }
    }
    if evidence.scenario_sampling_evidence_ref.trim().is_empty() {
        return Err(AggregationError::JointScenarioEvidenceMissing);
    }
    if !valid_sha256(&evidence.scenario_order_sha256) {
        return Err(AggregationError::InvalidScenarioOrderDigest);
    }
    if !evidence.scenarios_are_iid_joint_paths {
        return Err(AggregationError::JointScenarioSamplingNotIid);
    }
    if !evidence.all_shared_dependencies_are_included {
        return Err(AggregationError::JointScenarioDependenciesOmitted);
    }
    if !evidence.event_set_is_frozen_before_evaluation {
        return Err(AggregationError::JointScenarioEventSetNotFrozen);
    }

    let mut event_id_set = BTreeSet::new();
    for event_id in event_ids {
        if !valid_identifier(event_id) {
            return Err(AggregationError::InvalidIdentifier("event_id"));
        }
        if !event_id_set.insert(event_id.as_str()) {
            return Err(AggregationError::DuplicateEventId(event_id.clone()));
        }
    }

    let mut scenario_ids = BTreeSet::new();
    let mut any_collision_count = 0usize;
    for scenario in scenarios {
        if !valid_identifier(&scenario.scenario_id) {
            return Err(AggregationError::InvalidIdentifier("scenario_id"));
        }
        if !scenario_ids.insert(scenario.scenario_id.as_str()) {
            return Err(AggregationError::DuplicateScenarioId(
                scenario.scenario_id.clone(),
            ));
        }
        if scenario.event_outcomes.len() != event_ids.len() {
            return Err(AggregationError::JointScenarioOutcomeCountMismatch {
                scenario_id: scenario.scenario_id.clone(),
                expected: event_ids.len(),
                actual: scenario.event_outcomes.len(),
            });
        }
        if scenario.event_outcomes.iter().any(|outcome| *outcome) {
            any_collision_count += 1;
        }
    }

    let k = scenarios.len();
    let estimate = any_collision_count as f64 / k as f64;
    let radius = ((1.0 / delta).ln() / (2.0 * k as f64)).sqrt();
    let upper = (estimate + radius).min(1.0);
    Ok(JointPathRiskReport {
        method: "joint_scenario_any_collision_path_estimator_hoeffding",
        risk_quantity_type: "mission_or_fleet_joint_path_probability_any_collision",
        scenario_set_id: evidence.scenario_set_id.clone(),
        scenario_sampling_design_id: evidence.scenario_sampling_design_id.clone(),
        scenario_sampling_evidence_ref: evidence.scenario_sampling_evidence_ref.clone(),
        scenario_order_sha256: evidence.scenario_order_sha256.clone(),
        probability_space_id: evidence.probability_space_id.clone(),
        conditioning_context_id: evidence.conditioning_context_id.clone(),
        event_definition_id: evidence.event_definition_id.clone(),
        event_horizon_id: evidence.event_horizon_id.clone(),
        joint_model_id: evidence.joint_model_id.clone(),
        joint_model_revision: evidence.joint_model_revision.clone(),
        event_count: event_ids.len(),
        independent_joint_scenario_count: k,
        total_event_outcomes: k
            .checked_mul(event_ids.len())
            .ok_or(AggregationError::ArithmeticOverflow)?,
        scenarios_with_any_collision: any_collision_count,
        estimate_any_collision: estimate,
        confidence_delta: delta,
        one_sided_confidence_level: 1.0 - delta,
        hoeffding_radius: radius,
        upper_confidence_bound: upper,
        statistical_unit: "independent_complete_joint_mission_or_fleet_scenario",
        within_scenario_event_outcomes_are_dependent: true,
        claim_ceiling: "statistical_bound_conditional_on_iid_complete_joint_scenarios_and_fixed_event_set_not_long_term_collision_frequency",
    })
}

/// Long-term frequency is a separate rate quantity, never inferred from Pc.
pub fn long_term_collision_frequency(
    collisions_observed: u64,
    exposure_object_years: f64,
    population_scope_id: String,
    source_model_id: String,
    source_model_revision: String,
    frequency_method_id: String,
) -> Result<LongTermCollisionFrequency, AggregationError> {
    if !exposure_object_years.is_finite()
        || exposure_object_years <= 0.0
        || !valid_identifier(&population_scope_id)
        || !valid_identifier(&source_model_id)
        || !valid_identifier(&source_model_revision)
        || !valid_identifier(&frequency_method_id)
    {
        return Err(AggregationError::InvalidFrequencyExposure);
    }
    Ok(LongTermCollisionFrequency {
        collisions_observed,
        exposure_object_years,
        rate_per_object_year: collisions_observed as f64 / exposure_object_years,
        population_scope_id,
        source_model_id,
        source_model_revision,
        frequency_method_id,
    })
}

pub fn probability_of_detection(
    value: f64,
    declared_pc_action_threshold: f64,
    policy_revision: String,
    calibration_dataset_id: String,
) -> Result<ProbabilityOfDetection, AggregationError> {
    if !valid_probability(value)
        || !valid_probability(declared_pc_action_threshold)
        || !valid_identifier(&policy_revision)
        || !valid_identifier(&calibration_dataset_id)
    {
        return Err(AggregationError::InvalidProbabilityOfDetection);
    }
    Ok(ProbabilityOfDetection {
        value,
        declared_pc_action_threshold,
        policy_revision,
        calibration_dataset_id,
        claim_ceiling: "detection_probability_under_declared_policy_and_calibration_population_not_collision_probability",
    })
}

pub fn observation_significance(
    p_value: f64,
    significance_level: f64,
    null_hypothesis_id: String,
    test_statistic_id: String,
    measurement_model_revision: String,
) -> Result<ObservationSignificance, AggregationError> {
    if !valid_probability(p_value)
        || !valid_probability(significance_level)
        || !valid_identifier(&null_hypothesis_id)
        || !valid_identifier(&test_statistic_id)
        || !valid_identifier(&measurement_model_revision)
    {
        return Err(AggregationError::InvalidObservationSignificance);
    }
    Ok(ObservationSignificance {
        p_value,
        null_hypothesis_id,
        test_statistic_id,
        measurement_model_revision,
        significance_level,
    })
}

fn valid_probability(value: f64) -> bool {
    value.is_finite() && (0.0..=1.0).contains(&value)
}

fn valid_identifier(value: &str) -> bool {
    !value.trim().is_empty()
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"._:/-".contains(&byte))
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn validate_delta(delta: f64) -> Result<(), AggregationError> {
    if !delta.is_finite() || delta <= 0.0 || delta >= 1.0 {
        return Err(AggregationError::InvalidDelta(delta));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(id: &str, p: f64) -> EventLevelPc {
        EventLevelPc {
            event_id: id.into(),
            value: p,
            probability_space_id: "probability-space-v1".into(),
            conditioning_context_id: "conditioned-on-current-od-v1".into(),
            event_definition_id: "hard-body-contact-v1".into(),
            event_horizon_id: "mission-horizon-v1".into(),
            source_model_id: "joint-state-model-v1".into(),
            source_model_revision: "sha256:model-v1".into(),
        }
    }

    fn independence() -> MutualIndependenceEvidence {
        MutualIndependenceEvidence {
            event_family_id: "event-family-v1".into(),
            independence_model_id: "mutually-independent-events-v1".into(),
            evidence_ref: "fixture://independence/evidence-v1".into(),
            mutually_independent_events: true,
        }
    }

    fn scenario_evidence() -> JointScenarioEvidence {
        JointScenarioEvidence {
            scenario_set_id: "joint-scenario-set-v1".into(),
            scenario_sampling_design_id: "iid-complete-joint-scenarios-v1".into(),
            scenario_sampling_evidence_ref: "fixture://joint-scenarios/iid-design-v1".into(),
            scenario_order_sha256:
                "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            probability_space_id: "probability-space-v1".into(),
            conditioning_context_id: "conditioned-on-current-od-v1".into(),
            event_definition_id: "hard-body-contact-v1".into(),
            event_horizon_id: "mission-horizon-v1".into(),
            joint_model_id: "joint-mission-model-v1".into(),
            joint_model_revision: "sha256:joint-model-v1".into(),
            scenarios_are_iid_joint_paths: true,
            all_shared_dependencies_are_included: true,
            event_set_is_frozen_before_evaluation: true,
        }
    }

    #[test]
    fn frechet_bounds_hold_without_independence() {
        let events = vec![
            event("event-a", 0.2),
            event("event-b", 0.3),
            event("event-c", 0.4),
        ];
        let report = frechet_union_bounds(&events).unwrap();
        assert!((report.lower_bound - 0.4).abs() < 1e-14);
        assert!((report.upper_bound - 0.9).abs() < 1e-14);
        assert_eq!(report.point_estimate, None);
    }

    #[test]
    fn frechet_bounds_cover_perfect_positive_dependence_and_mutual_exclusion() {
        let same_event_probability = 0.25;
        let events = vec![
            event("a", same_event_probability),
            event("b", same_event_probability),
        ];
        let bounds = frechet_union_bounds(&events).unwrap();
        // Perfect positive dependence: union = 0.25.
        assert!(bounds.lower_bound <= 0.25 && 0.25 <= bounds.upper_bound);
        // Mutually exclusive events: union = 0.50.
        assert!(bounds.lower_bound <= 0.50 && 0.50 <= bounds.upper_bound);
    }

    #[test]
    fn product_formula_requires_explicit_independence_and_is_stable_for_small_p() {
        let events = vec![event("a", 1.0e-12), event("b", 2.0e-12)];
        let no_evidence = MutualIndependenceEvidence {
            mutually_independent_events: false,
            ..independence()
        };
        assert_eq!(
            independent_product_union_probability(&events, &no_evidence),
            Err(AggregationError::IndependenceNotAttested)
        );
        let report = independent_product_union_probability(&events, &independence()).unwrap();
        assert!((report.mission_union_probability - 3.0e-12).abs() < 1.0e-24);
        assert!(!report.saturation_warning);
    }

    #[test]
    fn large_event_family_exposes_saturation() {
        let events = (0..100_000)
            .map(|i| event(&format!("event-{i}"), 1.0e-3))
            .collect::<Vec<_>>();
        let report = independent_product_union_probability(&events, &independence()).unwrap();
        assert!(report.mission_union_probability > 1.0 - 1.0e-8);
        assert!(report.saturation_warning);
    }

    #[test]
    fn incompatible_contexts_cannot_be_aggregated() {
        let mut events = vec![event("a", 0.1), event("b", 0.2)];
        events[1].conditioning_context_id = "different-od-context".into();
        assert_eq!(
            frechet_union_bounds(&events),
            Err(AggregationError::IncompatibleConditioningContexts)
        );
    }

    #[test]
    fn joint_scenario_any_collision_uses_joint_scenario_count() {
        let event_ids = vec!["event-a".into(), "event-b".into(), "event-c".into()];
        let scenarios = vec![
            JointScenario {
                scenario_id: "scenario-1".into(),
                event_outcomes: vec![false, false, false],
            },
            JointScenario {
                scenario_id: "scenario-2".into(),
                event_outcomes: vec![true, true, false],
            },
            JointScenario {
                scenario_id: "scenario-3".into(),
                event_outcomes: vec![false, true, true],
            },
            JointScenario {
                scenario_id: "scenario-4".into(),
                event_outcomes: vec![false, false, false],
            },
        ];
        let report = joint_path_risk(&event_ids, &scenarios, 0.05, &scenario_evidence()).unwrap();
        assert_eq!(report.total_event_outcomes, 12);
        assert_eq!(report.independent_joint_scenario_count, 4);
        assert_eq!(report.scenarios_with_any_collision, 2);
        assert_eq!(report.estimate_any_collision, 0.5);
        assert!(report.within_scenario_event_outcomes_are_dependent);
    }

    #[test]
    fn joint_scenario_model_rejects_omitted_shared_dependencies_and_bad_shape() {
        let event_ids = vec!["event-a".into(), "event-b".into()];
        let scenarios = vec![
            JointScenario {
                scenario_id: "scenario-1".into(),
                event_outcomes: vec![false, false],
            },
            JointScenario {
                scenario_id: "scenario-2".into(),
                event_outcomes: vec![true, false],
            },
        ];
        let mut evidence = scenario_evidence();
        evidence.all_shared_dependencies_are_included = false;
        assert_eq!(
            joint_path_risk(&event_ids, &scenarios, 0.05, &evidence),
            Err(AggregationError::JointScenarioDependenciesOmitted)
        );
        assert!(matches!(
            joint_path_risk(
                &event_ids,
                &[JointScenario {
                    scenario_id: "scenario-1".into(),
                    event_outcomes: vec![false]
                }],
                0.05,
                &scenario_evidence()
            ),
            Err(AggregationError::InsufficientJointScenarios(1))
        ));
    }

    #[test]
    fn frequency_pd_and_significance_remain_separate_typed_quantities() {
        let frequency = long_term_collision_frequency(
            3,
            1000.0,
            "tracked-leo-population-v1".into(),
            "flux-model-v1".into(),
            "sha256:flux-v1".into(),
            "observed-count-over-exposure-v1".into(),
        )
        .unwrap();
        assert!((frequency.rate_per_object_year - 0.003).abs() < 1e-14);

        let pd = probability_of_detection(
            0.95,
            1.0e-4,
            "maneuver-policy-v1".into(),
            "policy-calibration-v1".into(),
        )
        .unwrap();
        assert_eq!(pd.value, 0.95);

        let significance = observation_significance(
            0.01,
            0.05,
            "collision-null-v1".into(),
            "miss-distance-test-v1".into(),
            "measurement-model-v1".into(),
        )
        .unwrap();
        assert_eq!(significance.p_value, 0.01);
    }

    #[test]
    fn duplicate_event_and_scenario_ids_are_rejected() {
        let duplicated = vec![event("a", 0.1), event("a", 0.2)];
        assert_eq!(
            frechet_union_bounds(&duplicated),
            Err(AggregationError::DuplicateEventId("a".into()))
        );

        let event_ids = vec!["a".into()];
        let scenarios = vec![
            JointScenario {
                scenario_id: "same".into(),
                event_outcomes: vec![false],
            },
            JointScenario {
                scenario_id: "same".into(),
                event_outcomes: vec![true],
            },
        ];
        assert_eq!(
            joint_path_risk(&event_ids, &scenarios, 0.05, &scenario_evidence()),
            Err(AggregationError::DuplicateScenarioId("same".into()))
        );
    }
}

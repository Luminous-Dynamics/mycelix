//! Detection-aware tracer sensor and threshold co-design.
//!
//! This module compares candidate censoring configurations (LOD/UQL/bin
//! boundaries) under a common conditional predictive model set. It converts
//! each sensor's bins into a finite discrete design and scores/selects them
//! under Shannon, fixed-alpha Sibson and per-design KL-radius references.
//!
//! It is synthetic measurement-design software. It does not calibrate real
//! detector thresholds, model tracer-mass effects or certify physical safety.

use crate::censored_tracer::build_censored_discrete_design;
use crate::information_gain::ConditionalGaussianModel;
use crate::robust_information::{
    select_discrete_design, select_discrete_design_at_kl_radius,
    DesignInformationObjective, DiscreteDesignCandidate, DiscreteDesignModel,
    DiscreteDesignSelection, RadiusRobustInformation,
};
use std::collections::BTreeSet;

/// One configurable tracer detector / reporting setup.
#[derive(Clone, Debug, PartialEq)]
pub struct TracerSensorOption {
    pub sensor_id: String,
    pub cost_units: u64,
    pub eligible: bool,
    /// Finite ascending censoring boundaries in the modelled observation scale.
    /// They may represent LOD, LLOQ/UQL or additional sampling/reporting bins.
    pub bin_boundaries: Vec<f64>,
}

/// Per-option information scores and censoring probabilities.
#[derive(Clone, Debug, PartialEq)]
pub struct TracerSensorScore {
    pub sensor_id: String,
    pub cost_units: u64,
    pub eligible: bool,
    pub feasible: bool,
    pub outcome_ids: Vec<String>,
    pub shannon_information_nats: f64,
    pub sibson_information_nats: f64,
    pub per_design_alpha: f64,
    pub per_design_kl_radius_nats: f64,
    pub per_design_robust_information_nats: f64,
    pub alpha_search_clamped: bool,
}

/// Complete one-step sensor co-design result.
#[derive(Clone, Debug, PartialEq)]
pub struct TracerSensorDesignResult {
    pub scores: Vec<TracerSensorScore>,
    pub nominal_selection: Option<DiscreteDesignSelection>,
    pub fixed_alpha_selection: Option<DiscreteDesignSelection>,
    pub fixed_kl_radius_selection: Option<(DiscreteDesignSelection, RadiusRobustInformation)>,
}

fn validate_sensor_options(options: &[TracerSensorOption]) -> Result<(), String> {
    if options.is_empty() {
        return Err("at least one tracer sensor option is required".into());
    }
    let mut identifiers = BTreeSet::new();
    for option in options {
        if option.sensor_id.trim().is_empty() || !identifiers.insert(option.sensor_id.as_str()) {
            return Err("tracer sensor IDs must be non-empty and unique".into());
        }
        if option.bin_boundaries.is_empty()
            || option.bin_boundaries.iter().any(|value| !value.is_finite())
            || option.bin_boundaries.windows(2).any(|pair| pair[1] <= pair[0])
        {
            return Err(format!(
                "sensor {:?} boundaries must be finite and strictly increasing",
                option.sensor_id
            ));
        }
    }
    Ok(())
}

fn as_candidate(
    option: &TracerSensorOption,
    design: &DiscreteDesignModel,
) -> DiscreteDesignCandidate {
    DiscreteDesignCandidate {
        action_id: option.sensor_id.clone(),
        cost_units: option.cost_units,
        eligible: option.eligible,
        outcome_ids: design.outcome_ids.clone(),
        likelihoods: design.likelihoods.clone(),
    }
}

/// Compare sensor configurations with different detection/quantification bins.
///
/// The expected information and robust metrics are model-discrimination
/// quantities for the supplied conditional predictive mixture. They are not
/// decision EVSI, nor estimates of physical tracer recovery.
pub fn compare_tracer_sensor_options(
    components: &[ConditionalGaussianModel],
    options: &[TracerSensorOption],
    budget_units: u64,
    sibson_alpha: f64,
    kl_radius_nats: f64,
    minimum_alpha: f64,
    alpha_bisection_iterations: usize,
) -> Result<TracerSensorDesignResult, String> {
    if !sibson_alpha.is_finite() || !(0.0..1.0).contains(&sibson_alpha) {
        return Err("fixed Sibson alpha must lie strictly between zero and one".into());
    }
    if !kl_radius_nats.is_finite() || kl_radius_nats < 0.0 {
        return Err("KL radius must be finite and non-negative".into());
    }
    if !minimum_alpha.is_finite()
        || !(0.0..1.0).contains(&minimum_alpha)
        || minimum_alpha >= 1.0 - 1e-8
    {
        return Err("minimum alpha must lie strictly between zero and 1 - 1e-8".into());
    }
    if alpha_bisection_iterations == 0 {
        return Err("at least one alpha bisection iteration is required".into());
    }
    validate_sensor_options(options)?;

    let mut designs = Vec::with_capacity(options.len());
    let mut candidates = Vec::with_capacity(options.len());
    let mut scores = Vec::with_capacity(options.len());

    for option in options {
        let design = build_censored_discrete_design(components, &option.bin_boundaries)?;
        let shannon = design.shannon_mutual_information()?;
        let sibson = design.sibson_alpha_mutual_information(sibson_alpha)?;
        let robust = crate::robust_information::robust_information_at_kl_radius(
            &design,
            kl_radius_nats,
            minimum_alpha,
            alpha_bisection_iterations,
        )?;
        let feasible = option.eligible && option.cost_units <= budget_units;

        scores.push(TracerSensorScore {
            sensor_id: option.sensor_id.clone(),
            cost_units: option.cost_units,
            eligible: option.eligible,
            feasible,
            outcome_ids: design.outcome_ids.clone(),
            shannon_information_nats: shannon,
            sibson_information_nats: sibson,
            per_design_alpha: robust.alpha,
            per_design_kl_radius_nats: robust.effective_kl_radius_nats,
            per_design_robust_information_nats: robust.sibson_information_nats,
            alpha_search_clamped: robust.alpha_search_clamped,
        });
        candidates.push(as_candidate(option, &design));
        designs.push(design);
    }

    let model_ids = components.iter()
        .map(|component| component.model_id.clone())
        .collect::<Vec<_>>();
    let prior_weights = components.iter()
        .map(|component| component.posterior_weight)
        .collect::<Vec<_>>();

    let nominal_selection = select_discrete_design(
        &model_ids,
        &prior_weights,
        &candidates,
        budget_units,
        DesignInformationObjective::Shannon,
    )?;
    let fixed_alpha_selection = select_discrete_design(
        &model_ids,
        &prior_weights,
        &candidates,
        budget_units,
        DesignInformationObjective::SibsonAlpha(sibson_alpha),
    )?;
    let fixed_kl_radius_selection = select_discrete_design_at_kl_radius(
        &model_ids,
        &prior_weights,
        &candidates,
        budget_units,
        kl_radius_nats,
        minimum_alpha,
        alpha_bisection_iterations,
    )?;

    Ok(TracerSensorDesignResult {
        scores,
        nominal_selection,
        fixed_alpha_selection,
        fixed_kl_radius_selection,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multivariate_gaussian::MultivariateGaussian;

    fn component(id: &str, weight: f64, mean: f64, standard_deviation: f64) -> ConditionalGaussianModel {
        ConditionalGaussianModel {
            model_id: id.into(),
            posterior_weight: weight,
            predictive_distribution: MultivariateGaussian {
                mean: vec![mean],
                covariance: vec![vec![standard_deviation * standard_deviation]],
            },
        }
    }

    fn model_pair() -> Vec<ConditionalGaussianModel> {
        vec![
            component("fast_path", 0.5, 0.2, 0.4),
            component("slow_path", 0.5, 1.0, 0.4),
        ]
    }

    fn options() -> Vec<TracerSensorOption> {
        vec![
            TracerSensorOption {
                sensor_id: "coarse_detector".into(),
                cost_units: 1,
                eligible: true,
                bin_boundaries: vec![0.5],
            },
            TracerSensorOption {
                sensor_id: "sensitive_detector".into(),
                cost_units: 2,
                eligible: true,
                bin_boundaries: vec![0.1, 0.5, 1.0],
            },
            TracerSensorOption {
                sensor_id: "perfect_but_ineligible".into(),
                cost_units: 1,
                eligible: false,
                bin_boundaries: vec![0.5, 0.8],
            },
            TracerSensorOption {
                sensor_id: "perfect_but_over_budget".into(),
                cost_units: 5,
                eligible: true,
                bin_boundaries: vec![0.5, 0.8],
            },
        ]
    }

    #[test]
    fn more_informative_censoring_bins_retain_more_model_information() {
        let result = compare_tracer_sensor_options(
            &model_pair(), &options()[..2], 2, 0.5, 0.05, 0.05, 64,
        ).unwrap();
        assert!(
            result.scores[1].shannon_information_nats
                >= result.scores[0].shannon_information_nats - 1e-8
        );
    }

    #[test]
    fn sensor_selector_enforces_budget_and_eligibility_before_ranking() {
        let result = compare_tracer_sensor_options(
            &model_pair(), &options(), 2, 0.5, 0.05, 0.05, 64,
        ).unwrap();
        assert!(result.nominal_selection.is_some());
        assert!(result.fixed_alpha_selection.is_some());
        assert!(result.fixed_kl_radius_selection.is_some());
        assert_eq!(result.scores[2].feasible, false);
        assert_eq!(result.scores[3].feasible, false);
        assert!(result.nominal_selection.unwrap().cost_units <= 2);
        assert!(result.fixed_alpha_selection.unwrap().cost_units <= 2);
    }

    #[test]
    fn sensor_scores_preserve_outcome_probability_and_report_robust_search() {
        let result = compare_tracer_sensor_options(
            &model_pair(), &options(), 2, 0.5, 0.05, 0.05, 64,
        ).unwrap();
        assert!(result.scores.iter().all(|score| {
            score.shannon_information_nats >= 0.0
                && score.sibson_information_nats >= 0.0
                && score.per_design_kl_radius_nats >= 0.0
                && score.per_design_robust_information_nats >= 0.0
        }));
        assert!(result.scores[0].outcome_ids.len() == 2);
        assert!(result.scores[1].outcome_ids.len() == 4);
    }

    #[test]
    fn rejects_duplicate_ids_invalid_bins_and_invalid_robust_controls() {
        let mut duplicate = options();
        duplicate[1].sensor_id = duplicate[0].sensor_id.clone();
        assert!(compare_tracer_sensor_options(
            &model_pair(), &duplicate, 2, 0.5, 0.05, 0.05, 64,
        ).is_err());
        let mut invalid_bins = options();
        invalid_bins[0].bin_boundaries = vec![0.5, 0.4];
        assert!(compare_tracer_sensor_options(
            &model_pair(), &invalid_bins, 2, 0.5, 0.05, 0.05, 64,
        ).is_err());
        assert!(compare_tracer_sensor_options(
            &model_pair(), &options(), 2, 1.0, 0.05, 0.05, 64,
        ).is_err());
    }
}

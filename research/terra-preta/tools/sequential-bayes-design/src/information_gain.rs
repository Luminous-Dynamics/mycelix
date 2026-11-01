//! Conditional mutual information for finite Gaussian model mixtures.
//!
//! The estimator computes I(M; Y_next | observed history) from posterior
//! model weights and each model's conditional Gaussian predictive law. It
//! estimates model-discrimination information, not decision value. Monte
//! Carlo standard error covers sampling error under the supplied mixture;
//! it does not account for covariance, model-family or simulator uncertainty.

use crate::multivariate_gaussian::{
    cholesky_factor, ConditionalGaussianModel, MultivariateGaussianModel,
};
use crate::observation_history::{
    conditional_prediction_for_action, ObservationAction, ObservationHistory, ObservationLayout,
};
use std::collections::BTreeSet;

/// Monte Carlo estimate of conditional model-identity information gain.
#[derive(Clone, Debug, PartialEq)]
pub struct InformationGainEstimate {
    /// Estimated I(M; Y_next | history), in nats. A small negative estimate
    /// within Monte Carlo error is retained rather than silently clipped.
    pub expected_information_nats: f64,
    /// Standard error from independent samples per non-zero posterior model.
    pub monte_carlo_standard_error_nats: f64,
    /// Number of draws for each model with strictly positive posterior weight.
    pub samples_per_model: usize,
    /// Total generated observation vectors.
    pub total_samples: usize,
    /// Seed used by the deterministic sampler.
    pub seed: u64,
}

#[derive(Clone, Debug)]
struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut value = self.state;
        value = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        value = (value ^ (value >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        value ^ (value >> 31)
    }

    fn uniform_open01(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / 9_007_199_254_740_992.0
    }

    fn standard_normal_pair(&mut self) -> (f64, f64) {
        let radius = (-2.0 * self.uniform_open01().ln()).sqrt();
        let angle = std::f64::consts::TAU * self.uniform_open01();
        (radius * angle.cos(), radius * angle.sin())
    }

    fn standard_normals(&mut self, dimension: usize) -> Vec<f64> {
        let mut values = Vec::with_capacity(dimension);
        while values.len() < dimension {
            let (first, second) = self.standard_normal_pair();
            values.push(first);
            if values.len() < dimension {
                values.push(second);
            }
        }
        values
    }
}

fn sample_gaussian(
    component: &ConditionalGaussianModel,
    rng: &mut SplitMix64,
) -> Result<Vec<f64>, String> {
    component.predictive_distribution.validate()?;
    let (lower, _) = cholesky_factor(&component.predictive_distribution.covariance)?;
    let standard_normals = rng.standard_normals(component.predictive_distribution.mean.len());

    let sample: Vec<f64> = component.predictive_distribution.mean.iter()
        .enumerate()
        .map(|(row, mean)| {
            let offset: f64 = lower[row].iter()
                .take(row + 1)
                .zip(standard_normals.iter())
                .map(|(factor, normal)| factor * normal)
                .sum();
            mean + offset
        })
        .collect();

    if sample.iter().all(|value| value.is_finite()) {
        Ok(sample)
    } else {
        Err("Gaussian sampling produced a non-finite observation".into())
    }
}

fn mixture_log_density(
    components: &[ConditionalGaussianModel],
    normalized_weights: &[f64],
    observation: &[f64],
) -> Result<f64, String> {
    let log_terms = components.iter()
        .zip(normalized_weights)
        .map(|(component, weight)| {
            if *weight == 0.0 {
                Ok(f64::NEG_INFINITY)
            } else {
                Ok(weight.ln() + component.predictive_distribution.log_density(observation)?)
            }
        })
        .collect::<Result<Vec<_>, String>>()?;

    let maximum = log_terms.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if !maximum.is_finite() {
        return Err("all predictive mixture components assign zero numerical density".into());
    }
    let scaled_sum: f64 = log_terms.iter()
        .map(|value| (value - maximum).exp())
        .sum();
    if !scaled_sum.is_finite() || scaled_sum <= 0.0 {
        return Err("predictive mixture density normalization failed".into());
    }
    Ok(maximum + scaled_sum.ln())
}

fn validate_components(
    components: &[ConditionalGaussianModel],
) -> Result<Vec<f64>, String> {
    if components.is_empty() {
        return Err("at least one predictive model component is required".into());
    }
    let dimension = components[0].predictive_distribution.mean.len();
    if dimension == 0 {
        return Err("predictive observation dimension must be non-zero".into());
    }

    let mut identifiers = BTreeSet::new();
    let mut weight_total = 0.0;
    for component in components {
        if component.model_id.trim().is_empty()
            || !identifiers.insert(component.model_id.as_str())
        {
            return Err("predictive model identifiers must be non-empty and unique".into());
        }
        if !component.posterior_weight.is_finite() || component.posterior_weight < 0.0 {
            return Err(format!("invalid posterior weight for {:?}", component.model_id));
        }
        if component.predictive_distribution.mean.len() != dimension {
            return Err("predictive component dimensions must agree".into());
        }
        component.predictive_distribution.validate()?;
        weight_total += component.posterior_weight;
    }
    if !weight_total.is_finite() || weight_total <= 0.0 {
        return Err("predictive model weights must have finite positive sum".into());
    }

    Ok(components.iter()
        .map(|component| component.posterior_weight / weight_total)
        .collect())
}

/// Estimate model-identity information in a proposed next measurement:
///
/// `I(M;Y) = Σ_m w_m E_{Y~p_m}[log p_m(Y) - log Σ_j w_j p_j(Y)]`.
///
/// Each component is sampled independently with a deterministic SplitMix64 /
/// Box-Muller stream. The returned standard error is the weighted combination
/// of per-component Monte Carlo standard errors. It does not include
/// finite-sample bias, model misspecification, covariance-estimation error or
/// scenario-family uncertainty.
pub fn estimate_model_information_gain(
    components: &[ConditionalGaussianModel],
    samples_per_model: usize,
    seed: u64,
) -> Result<InformationGainEstimate, String> {
    if samples_per_model < 2 {
        return Err("at least two samples per positive-weight model are required".into());
    }
    let weights = validate_components(components)?;

    let mut expected_information = 0.0;
    let mut variance_of_mean = 0.0;
    let mut total_samples = 0usize;

    for (model_index, (component, weight)) in components.iter().zip(&weights).enumerate() {
        if *weight == 0.0 {
            continue;
        }

        let model_seed = seed
            ^ (model_index as u64).wrapping_mul(0xD6E8_FEB8_6659_FD93);
        let mut rng = SplitMix64::new(model_seed);
        let mut mean = 0.0;
        let mut sum_squared_deviations = 0.0;

        for sample_index in 0..samples_per_model {
            let observation = sample_gaussian(component, &mut rng)?;
            let component_log_density =
                component.predictive_distribution.log_density(&observation)?;
            let mixture_density =
                mixture_log_density(components, &weights, &observation)?;
            let contribution = component_log_density - mixture_density;
            if !contribution.is_finite() {
                return Err("information-gain sample contribution is not finite".into());
            }

            let count = (sample_index + 1) as f64;
            let delta = contribution - mean;
            mean += delta / count;
            let delta_after = contribution - mean;
            sum_squared_deviations += delta * delta_after;
        }

        let sample_variance =
            (sum_squared_deviations / (samples_per_model - 1) as f64).max(0.0);
        expected_information += weight * mean;
        variance_of_mean += weight * weight * sample_variance / samples_per_model as f64;
        total_samples += samples_per_model;
    }

    let standard_error = variance_of_mean.sqrt();
    if !expected_information.is_finite() || !standard_error.is_finite() {
        return Err("information-gain estimate or uncertainty is not finite".into());
    }

    Ok(InformationGainEstimate {
        expected_information_nats: expected_information,
        monte_carlo_standard_error_nats: standard_error,
        samples_per_model,
        total_samples,
        seed,
    })
}


/// Estimate the information value of a candidate action using stable
/// observation identities and the full correlated measurement history.
///
/// This composes the history-to-conditional-mixture bridge with the Monte
/// Carlo model-information estimator. It estimates
/// `I(M; Y_action | Y_history)`; it is not decision EVSI and it does not
/// infer covariance from support metadata.
pub fn estimate_action_information_gain(
    models: &[MultivariateGaussianModel],
    layout: &ObservationLayout,
    history: &ObservationHistory,
    action: &ObservationAction,
    samples_per_model: usize,
    seed: u64,
) -> Result<InformationGainEstimate, String> {
    let components =
        conditional_prediction_for_action(models, layout, history, action)?;
    estimate_model_information_gain(&components, samples_per_model, seed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multivariate_gaussian::MultivariateGaussian;

    fn component(id: &str, weight: f64, correlation: f64) -> ConditionalGaussianModel {
        ConditionalGaussianModel {
            model_id: id.into(),
            posterior_weight: weight,
            predictive_distribution: MultivariateGaussian {
                mean: vec![0.0, 0.0],
                covariance: vec![
                    vec![1.0, correlation],
                    vec![correlation, 1.0],
                ],
            },
        }
    }

    fn cross_modal_models() -> Vec<MultivariateGaussianModel> {
        use crate::multivariate_gaussian::MultivariateGaussian;
        vec![
            MultivariateGaussianModel {
                model_id: "positive-cross-modal-correlation".into(),
                prior_weight: 0.5,
                observation_model: MultivariateGaussian {
                    mean: vec![0.0, 0.0],
                    covariance: vec![vec![1.0, 0.8], vec![0.8, 1.0]],
                },
            },
            MultivariateGaussianModel {
                model_id: "negative-cross-modal-correlation".into(),
                prior_weight: 0.5,
                observation_model: MultivariateGaussian {
                    mean: vec![0.0, 0.0],
                    covariance: vec![vec![1.0, -0.8], vec![-0.8, 1.0]],
                },
            },
        ]
    }

    fn cross_modal_layout() -> ObservationLayout {
        use crate::observation_history::ObservationCoordinate;
        ObservationLayout::new(vec![
            ObservationCoordinate {
                coordinate_id: "ct:pore-connectivity:core-A".into(),
                modality: "ct".into(),
                support_id: "core-A:imaged-volume".into(),
                unit: "1".into(),
            },
            ObservationCoordinate {
                coordinate_id: "tracer:arrival:core-A".into(),
                modality: "tracer".into(),
                support_id: "core-A:depth-10-20cm".into(),
                unit: "hour".into(),
            },
        ]).unwrap()
    }

    #[test]
    fn separate_ct_or_tracer_marginals_have_zero_information_but_the_pair_is_complementary() {
        use crate::observation_history::ObservationAction;

        let models = cross_modal_models();
        let layout = cross_modal_layout();
        let prior_history = ObservationHistory::empty();
        let ct_action = ObservationAction {
            action_id: "ct-only".into(),
            coordinate_ids: vec!["ct:pore-connectivity:core-A".into()],
        };
        let tracer_action = ObservationAction {
            action_id: "tracer-only".into(),
            coordinate_ids: vec!["tracer:arrival:core-A".into()],
        };
        let pair_action = ObservationAction {
            action_id: "ct-plus-tracer".into(),
            coordinate_ids: vec![
                "ct:pore-connectivity:core-A".into(),
                "tracer:arrival:core-A".into(),
            ],
        };

        let ct_gain = estimate_action_information_gain(
            &models, &layout, &prior_history, &ct_action, 2048, 20261101,
        ).unwrap();
        let tracer_gain = estimate_action_information_gain(
            &models, &layout, &prior_history, &tracer_action, 2048, 20261102,
        ).unwrap();
        let pair_gain = estimate_action_information_gain(
            &models, &layout, &prior_history, &pair_action, 2048, 20261103,
        ).unwrap();

        assert!(ct_gain.expected_information_nats.abs() < 1e-12);
        assert!(tracer_gain.expected_information_nats.abs() < 1e-12);
        assert!(pair_gain.expected_information_nats > 0.2);
        assert!(pair_gain.monte_carlo_standard_error_nats > 0.0);
        assert_eq!(pair_gain.total_samples, 4096);
    }

    #[test]
    fn tracer_information_becomes_positive_after_ct_history_is_observed() {
        use crate::observation_history::ObservationAction;

        let models = cross_modal_models();
        let layout = cross_modal_layout();
        let ct_action = ObservationAction {
            action_id: "ct-scan".into(),
            coordinate_ids: vec!["ct:pore-connectivity:core-A".into()],
        };
        let tracer_action = ObservationAction {
            action_id: "event-tracer".into(),
            coordinate_ids: vec!["tracer:arrival:core-A".into()],
        };
        let history = ObservationHistory::empty()
            .with_appended(&layout, &ct_action, &[1.0]).unwrap();

        let conditional_gain = estimate_action_information_gain(
            &models, &layout, &history, &tracer_action, 2048, 20261104,
        ).unwrap();

        assert!(conditional_gain.expected_information_nats > 0.2);
        assert!(conditional_gain.monte_carlo_standard_error_nats > 0.0);
        assert_eq!(conditional_gain.total_samples, 4096);
    }

    #[test]
    fn one_model_has_exactly_zero_model_identity_information() {
        let estimate = estimate_model_information_gain(
            &[component("only", 1.0, 0.7)],
            64,
            42,
        ).unwrap();
        assert!(estimate.expected_information_nats.abs() < 1e-12);
        assert!(estimate.monte_carlo_standard_error_nats < 1e-12);
        assert_eq!(estimate.total_samples, 64);
    }

    #[test]
    fn identical_predictive_distributions_have_zero_information_gain() {
        let estimate = estimate_model_information_gain(
            &[component("a", 0.3, 0.4), component("b", 0.7, 0.4)],
            128,
            43,
        ).unwrap();
        assert!(estimate.expected_information_nats.abs() < 1e-12);
        assert!(estimate.monte_carlo_standard_error_nats < 1e-12);
    }

    #[test]
    fn cross_covariance_only_difference_has_positive_information_gain() {
        let components = vec![
            component("positive", 0.5, 0.8),
            component("negative", 0.5, -0.8),
        ];
        let first = estimate_model_information_gain(&components, 1024, 20261101).unwrap();
        let replay = estimate_model_information_gain(&components, 1024, 20261101).unwrap();

        assert!(first.expected_information_nats > 0.1);
        assert!(first.monte_carlo_standard_error_nats > 0.0);
        assert_eq!(first, replay);
        assert_eq!(first.total_samples, 2048);
    }

    #[test]
    fn zero_weight_component_is_not_sampled_or_given_information_credit() {
        let estimate = estimate_model_information_gain(
            &[component("active", 1.0, 0.0), component("excluded", 0.0, 0.9)],
            64,
            44,
        ).unwrap();
        assert!(estimate.expected_information_nats.abs() < 1e-12);
        assert_eq!(estimate.total_samples, 64);
    }

    #[test]
    fn rejects_invalid_sample_count_weights_ids_and_dimensions() {
        assert!(estimate_model_information_gain(&[component("one", 1.0, 0.0)], 1, 1).is_err());
        assert!(estimate_model_information_gain(&[], 2, 1).is_err());
        assert!(estimate_model_information_gain(
            &[component("bad", -0.1, 0.0)], 2, 1
        ).is_err());
        assert!(estimate_model_information_gain(
            &[component("same", 0.5, 0.0), component("same", 0.5, 0.1)],
            2,
            1,
        ).is_err());
        let bad_dimension = ConditionalGaussianModel {
            model_id: "dimension".into(),
            posterior_weight: 0.5,
            predictive_distribution: MultivariateGaussian {
                mean: vec![0.0],
                covariance: vec![vec![1.0]],
            },
        };
        assert!(estimate_model_information_gain(
            &[component("a", 0.5, 0.0), bad_dimension],
            2,
            1,
        ).is_err());
    }
}

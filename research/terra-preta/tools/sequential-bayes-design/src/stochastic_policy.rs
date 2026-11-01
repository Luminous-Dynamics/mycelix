//! Finite stochastic-policy robust design and PAC-Bayes Gibbs policy algebra.
//!
//! This module implements exact finite-action reference calculations for
//! (a) the shared-average-KL maximin dual with one common beta across a
//! stochastic policy, and (b) the closed-form Gibbs optimizer of the
//! PAC-Bayes variational objective for supplied action reward estimates.
//!
//! It does not implement the paper's nested Monte Carlo estimator or certify
//! the regularity/boundedness constants required by its PAC-Bayes theorem.

use crate::robust_information::DiscreteDesignModel;
use std::collections::BTreeSet;

/// One finite candidate design in a stochastic design policy.
#[derive(Clone, Debug, PartialEq)]
pub struct StochasticPolicyCandidate {
    pub action_id: String,
    pub cost_units: u64,
    pub eligible: bool,
    pub design: DiscreteDesignModel,
}

/// A single action's probability and robust information reward.
#[derive(Clone, Debug, PartialEq)]
pub struct GibbsActionWeight {
    pub action_id: String,
    pub probability: f64,
    pub prior_probability: f64,
    pub sibson_information_nats: f64,
    pub cost_units: u64,
}

/// Gibbs policy that exactly maximizes a finite reward-minus-KL objective.
#[derive(Clone, Debug, PartialEq)]
pub struct GibbsPolicy {
    pub precision_lambda: f64,
    pub action_weights: Vec<GibbsActionWeight>,
    pub expected_information_nats: f64,
    pub kl_to_prior_nats: f64,
    pub variational_objective_nats: f64,
    pub expected_cost_units: f64,
}

/// Maximin robust value of a fixed stochastic policy under one shared
/// average-KL ambiguity radius.
#[derive(Clone, Debug, PartialEq)]
pub struct SharedKlPolicyValue {
    pub requested_average_kl_radius_nats: f64,
    /// One common dual variable shared by all actions in the policy.
    /// None denotes the zero-radius nominal limit beta -> infinity.
    pub common_dual_beta: Option<f64>,
    /// alpha = beta / (1 + beta); alpha=1 denotes the nominal limit.
    pub alpha: f64,
    pub expected_sibson_information_nats: f64,
    pub adversarial_kl_penalty_nats: f64,
    pub robust_policy_value_nats: f64,
    pub expected_cost_units: f64,
    pub policy_probabilities: Vec<f64>,
}

/// Inputs to the finite-sample PAC-Bayes lower-bound formula.
///
/// Constants must be externally justified under the paper's assumptions.
/// This calculator cannot validate those assumptions from data.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PacBayesConcentrationConfig {
    pub outer_samples: usize,
    pub inner_samples: usize,
    pub confidence_delta: f64,
    pub precision_lambda: f64,
    pub lipschitz_f: f64,
    pub lipschitz_h: f64,
    pub concentration_c_h: f64,
    pub sigma_w: f64,
}

/// Algebraic result of the paper's PAC-Bayes lower-bound expression.
#[derive(Clone, Debug, PartialEq)]
pub struct PacBayesLowerBound {
    pub lower_bound_nats: f64,
    pub empirical_policy_reward_nats: f64,
    pub policy_kl_to_prior_nats: f64,
    pub estimator_penalty_nats: f64,
    pub policy_complexity_penalty_nats: f64,
    pub required_inner_samples: f64,
    pub actual_outer_samples: usize,
    pub actual_inner_samples: usize,
    pub confidence_delta: f64,
    pub validity_scope: &'static str,
}

fn validate_action_pool(
    candidates: &[StochasticPolicyCandidate],
) -> Result<(), String> {
    if candidates.is_empty() {
        return Err("at least one stochastic-policy candidate is required".into());
    }
    let mut action_ids = BTreeSet::new();
    for candidate in candidates {
        if candidate.action_id.trim().is_empty()
            || !action_ids.insert(candidate.action_id.as_str())
        {
            return Err("stochastic-policy action IDs must be non-empty and unique".into());
        }
        candidate.design.validate()?;
    }
    Ok(())
}

fn validate_distribution(probabilities: &[f64], dimension: usize, label: &str) -> Result<(), String> {
    if probabilities.len() != dimension {
        return Err(format!("{label} probabilities must align with the candidate actions"));
    }
    if probabilities.iter().any(|value| !value.is_finite() || *value < 0.0) {
        return Err(format!("{label} probabilities must be finite and non-negative"));
    }
    let total: f64 = probabilities.iter().sum();
    if !total.is_finite() || (total - 1.0).abs() > 1e-10 {
        return Err(format!("{label} probabilities must sum to one"));
    }
    Ok(())
}

fn validate_policy_support(
    candidates: &[StochasticPolicyCandidate],
    probabilities: &[f64],
    hard_action_cost_cap_units: u64,
) -> Result<(), String> {
    validate_distribution(probabilities, candidates.len(), "policy")?;
    for (candidate, probability) in candidates.iter().zip(probabilities) {
        if *probability > 0.0
            && (!candidate.eligible || candidate.cost_units > hard_action_cost_cap_units)
        {
            return Err(format!(
                "policy assigns positive mass to ineligible or over-cap action {:?}",
                candidate.action_id
            ));
        }
    }
    Ok(())
}

fn expected_cost(
    candidates: &[StochasticPolicyCandidate],
    probabilities: &[f64],
) -> f64 {
    candidates.iter().zip(probabilities)
        .map(|(candidate, probability)| candidate.cost_units as f64 * probability)
        .sum()
}

fn expected_sibson_at_beta(
    candidates: &[StochasticPolicyCandidate],
    probabilities: &[f64],
    beta: f64,
) -> Result<(f64, f64), String> {
    if !beta.is_finite() || beta < 0.0 {
        return Err("common dual beta must be finite and non-negative".into());
    }
    if beta == 0.0 {
        return Ok((0.0, 0.0));
    }
    let alpha = beta / (1.0 + beta);
    let mut expected_information = 0.0;
    for (candidate, probability) in candidates.iter().zip(probabilities) {
        if *probability > 0.0 {
            let action_information = if alpha <= 1e-10 {
                0.0
            } else if alpha >= 1.0 - 1e-12 {
                candidate.design.shannon_mutual_information()?
            } else {
                candidate.design.sibson_alpha_mutual_information(alpha)?
            };
            expected_information += probability * action_information;
        }
    }
    let reported_alpha = if alpha >= 1.0 - 1e-12 { 1.0 } else { alpha };
    Ok((reported_alpha, expected_information))
}

/// Evaluate a *fixed* stochastic policy with a single common dual beta.
///
/// Implements the finite form of
/// `sup_{beta>0} { E_pi[I^S_{beta/(1+beta)}(design)] - beta * rho }`.
/// The ambiguity budget is averaged over the policy; beta is not inverted
/// independently for each action.
pub fn evaluate_shared_kl_policy_at_beta(
    candidates: &[StochasticPolicyCandidate],
    policy_probabilities: &[f64],
    hard_action_cost_cap_units: u64,
    average_kl_radius_nats: f64,
    common_dual_beta: f64,
) -> Result<SharedKlPolicyValue, String> {
    validate_action_pool(candidates)?;
    validate_policy_support(candidates, policy_probabilities, hard_action_cost_cap_units)?;
    if !average_kl_radius_nats.is_finite() || average_kl_radius_nats < 0.0 {
        return Err("average KL radius must be finite and non-negative".into());
    }
    if !common_dual_beta.is_finite() || common_dual_beta < 0.0 {
        return Err("common dual beta must be finite and non-negative".into());
    }
    if average_kl_radius_nats == 0.0 {
        if common_dual_beta != 0.0 {
            return Err("zero ambiguity radius uses the nominal beta-to-infinity limit".into());
        }
        let mut nominal_information = 0.0;
        for (candidate, probability) in candidates.iter().zip(policy_probabilities) {
            if *probability > 0.0 {
                nominal_information += probability
                    * candidate.design.shannon_mutual_information()?;
            }
        }
        return Ok(SharedKlPolicyValue {
            requested_average_kl_radius_nats: 0.0,
            common_dual_beta: None,
            alpha: 1.0,
            expected_sibson_information_nats: nominal_information,
            adversarial_kl_penalty_nats: 0.0,
            robust_policy_value_nats: nominal_information,
            expected_cost_units: expected_cost(candidates, policy_probabilities),
            policy_probabilities: policy_probabilities.to_vec(),
        });
    }

    let (alpha, expected_information) =
        expected_sibson_at_beta(candidates, policy_probabilities, common_dual_beta)?;
    let penalty = common_dual_beta * average_kl_radius_nats;
    Ok(SharedKlPolicyValue {
        requested_average_kl_radius_nats: average_kl_radius_nats,
        common_dual_beta: Some(common_dual_beta),
        alpha,
        expected_sibson_information_nats: expected_information,
        adversarial_kl_penalty_nats: penalty,
        robust_policy_value_nats: expected_information - penalty,
        expected_cost_units: expected_cost(candidates, policy_probabilities),
        policy_probabilities: policy_probabilities.to_vec(),
    })
}

/// Maximize the shared-average-KL dual for one fixed finite stochastic policy.
///
/// For positive radius, beta is bounded above by max_i I(M;X_i)/rho because
/// Sibson information cannot exceed nominal Shannon information. The dual is
/// concave for the fixed policy; golden-section maximization searches this
/// finite bracket. At rho=0 the exact nominal limit is returned.
pub fn optimize_shared_kl_policy(
    candidates: &[StochasticPolicyCandidate],
    policy_probabilities: &[f64],
    hard_action_cost_cap_units: u64,
    average_kl_radius_nats: f64,
    iterations: usize,
) -> Result<SharedKlPolicyValue, String> {
    validate_action_pool(candidates)?;
    validate_policy_support(candidates, policy_probabilities, hard_action_cost_cap_units)?;
    if !average_kl_radius_nats.is_finite() || average_kl_radius_nats < 0.0 {
        return Err("average KL radius must be finite and non-negative".into());
    }
    if iterations == 0 {
        return Err("at least one dual-search iteration is required".into());
    }
    if average_kl_radius_nats == 0.0 {
        return evaluate_shared_kl_policy_at_beta(
            candidates, policy_probabilities, hard_action_cost_cap_units, 0.0, 0.0,
        );
    }

    let mut maximum_information = 0.0_f64;
    for (candidate, probability) in candidates.iter().zip(policy_probabilities) {
        if *probability > 0.0 {
            maximum_information = maximum_information
                .max(candidate.design.shannon_mutual_information()?);
        }
    }
    if maximum_information <= 1e-14 {
        return Ok(SharedKlPolicyValue {
            requested_average_kl_radius_nats: average_kl_radius_nats,
            common_dual_beta: Some(0.0),
            alpha: 0.0,
            expected_sibson_information_nats: 0.0,
            adversarial_kl_penalty_nats: 0.0,
            robust_policy_value_nats: 0.0,
            expected_cost_units: expected_cost(candidates, policy_probabilities),
            policy_probabilities: policy_probabilities.to_vec(),
        });
    }

    let upper = maximum_information / average_kl_radius_nats;
    if !upper.is_finite() || upper <= 0.0 {
        return Err("shared-KL dual search interval is invalid".into());
    }
    let objective = |beta: f64| -> Result<f64, String> {
        let (_, expected_information) =
            expected_sibson_at_beta(candidates, policy_probabilities, beta)?;
        Ok(expected_information - beta * average_kl_radius_nats)
    };

    let golden = (5.0_f64.sqrt() - 1.0) / 2.0;
    let mut left = 0.0;
    let mut right = upper;
    let mut x1 = right - golden * (right - left);
    let mut x2 = left + golden * (right - left);
    let mut y1 = objective(x1)?;
    let mut y2 = objective(x2)?;

    for _ in 0..iterations {
        if y1 < y2 {
            left = x1;
            x1 = x2;
            y1 = y2;
            x2 = left + golden * (right - left);
            y2 = objective(x2)?;
        } else {
            right = x2;
            x2 = x1;
            y2 = y1;
            x1 = right - golden * (right - left);
            y1 = objective(x1)?;
        }
    }

    let candidates_beta = [0.0, upper, x1, x2, 0.5 * (left + right)];
    let mut best_beta = 0.0;
    let mut best_value = 0.0;
    for beta in candidates_beta {
        let value = objective(beta)?;
        if value > best_value {
            best_value = value;
            best_beta = beta;
        }
    }
    let result = evaluate_shared_kl_policy_at_beta(
        candidates,
        policy_probabilities,
        hard_action_cost_cap_units,
        average_kl_radius_nats,
        best_beta,
    )?;
    Ok(result)
}

/// Compute the finite Gibbs policy from exact or supplied per-action robust
/// reward estimates:
///
/// `pi*(action) ∝ pi0(action) * exp(lambda * reward(action))`.
///
/// This is the closed-form optimizer of E_pi[reward] - KL(pi || pi0)/lambda.
/// It is not itself a confidence bound; the nested-Monte-Carlo PAC-Bayes
/// lower-bound calculation is provided separately below.
pub fn gibbs_policy_from_rewards(
    action_ids: &[String],
    reward_estimates_nats: &[f64],
    prior_policy: &[f64],
    eligible: &[bool],
    action_costs: &[u64],
    hard_action_cost_cap_units: u64,
    precision_lambda: f64,
) -> Result<GibbsPolicy, String> {
    let count = action_ids.len();
    if count == 0
        || reward_estimates_nats.len() != count
        || eligible.len() != count
        || action_costs.len() != count
    {
        return Err("Gibbs policy inputs must be non-empty and dimensionally aligned".into());
    }
    validate_distribution(prior_policy, count, "prior policy")?;
    if !precision_lambda.is_finite() || precision_lambda <= 0.0 {
        return Err("Gibbs precision lambda must be finite and positive".into());
    }
    if action_ids.iter().any(|id| id.trim().is_empty()) {
        return Err("Gibbs action IDs must be non-empty".into());
    }
    let unique_ids: BTreeSet<&str> = action_ids.iter().map(String::as_str).collect();
    if unique_ids.len() != count {
        return Err("Gibbs action IDs must be unique".into());
    }
    if reward_estimates_nats.iter().any(|value| !value.is_finite()) {
        return Err("Gibbs reward estimates must be finite".into());
    }
    for (index, prior_probability) in prior_policy.iter().enumerate() {
        if *prior_probability > 0.0
            && (!eligible[index] || action_costs[index] > hard_action_cost_cap_units)
        {
            return Err(format!(
                "prior policy assigns mass to ineligible or over-cap action {:?}",
                action_ids[index]
            ));
        }
    }

    let log_weights: Vec<f64> = prior_policy.iter()
        .zip(reward_estimates_nats)
        .map(|(prior, reward)| {
            if *prior == 0.0 { f64::NEG_INFINITY }
            else { prior.ln() + precision_lambda * reward }
        })
        .collect();
    let maximum = log_weights.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if !maximum.is_finite() {
        return Err("Gibbs policy has no feasible positive prior mass".into());
    }
    let unnormalized: Vec<f64> = log_weights.iter()
        .map(|value| (value - maximum).exp())
        .collect();
    let total: f64 = unnormalized.iter().sum();
    if !total.is_finite() || total <= 0.0 {
        return Err("Gibbs policy normalization failed".into());
    }
    let probabilities: Vec<f64> = unnormalized.iter()
        .map(|value| value / total)
        .collect();

    let mut expected_reward = 0.0;
    let mut kl_to_prior = 0.0;
    let mut mean_cost = 0.0;
    for (((probability, prior), reward), cost) in probabilities.iter()
        .zip(prior_policy)
        .zip(reward_estimates_nats)
        .zip(action_costs)
    {
        expected_reward += probability * reward;
        mean_cost += probability * *cost as f64;
        if *probability > 0.0 {
            if *prior <= 0.0 {
                return Err("Gibbs policy left the support of its prior".into());
            }
            kl_to_prior += probability * (probability / prior).ln();
        }
    }
    let variational_objective = expected_reward - kl_to_prior / precision_lambda;
    let action_weights = action_ids.iter()
        .zip(&probabilities)
        .zip(prior_policy)
        .zip(reward_estimates_nats)
        .zip(action_costs)
        .map(|((((action_id, probability), prior_probability), reward), cost_units)| {
            GibbsActionWeight {
                action_id: action_id.clone(),
                probability: *probability,
                prior_probability: *prior_probability,
                sibson_information_nats: *reward,
                cost_units: *cost_units,
            }
        })
        .collect();

    Ok(GibbsPolicy {
        precision_lambda,
        action_weights,
        expected_information_nats: expected_reward,
        kl_to_prior_nats: kl_to_prior,
        variational_objective_nats: variational_objective,
        expected_cost_units: mean_cost,
    })
}

/// Compute the PAC-Bayes lower-bound formula from Proposition 6.
///
/// The bound is conditional on the paper's regularity and boundedness
/// assumptions and on externally justified values for all concentration
/// constants. This function checks numerical prerequisites and the stated
/// inner-sample-size condition, but cannot certify those assumptions.
pub fn calculate_pac_bayes_lower_bound(
    policy_probabilities: &[f64],
    prior_policy: &[f64],
    reward_estimates_nats: &[f64],
    config: PacBayesConcentrationConfig,
) -> Result<PacBayesLowerBound, String> {
    let count = policy_probabilities.len();
    if count == 0
        || prior_policy.len() != count
        || reward_estimates_nats.len() != count
    {
        return Err("PAC-Bayes policy, prior and reward vectors must be non-empty and aligned".into());
    }
    validate_distribution(policy_probabilities, count, "posterior policy")?;
    validate_distribution(prior_policy, count, "prior policy")?;
    if reward_estimates_nats.iter().any(|value| !value.is_finite()) {
        return Err("PAC-Bayes reward estimates must be finite".into());
    }
    if config.outer_samples == 0 || config.inner_samples == 0 {
        return Err("PAC-Bayes outer and inner sample counts must be positive".into());
    }
    if !config.confidence_delta.is_finite()
        || !(0.0..1.0).contains(&config.confidence_delta)
    {
        return Err("PAC-Bayes confidence delta must lie strictly between zero and one".into());
    }
    if !config.precision_lambda.is_finite() || config.precision_lambda <= 0.0 {
        return Err("PAC-Bayes precision lambda must be finite and positive".into());
    }
    for (name, value) in [
        ("L_f", config.lipschitz_f),
        ("L_h", config.lipschitz_h),
        ("C_h", config.concentration_c_h),
    ] {
        if !value.is_finite() || value <= 0.0 {
            return Err(format!("PAC-Bayes constant {name} must be finite and positive"));
        }
    }
    if !config.sigma_w.is_finite() || config.sigma_w < 0.0 {
        return Err("PAC-Bayes sigma_w must be finite and non-negative".into());
    }

    let log_term = (2.0 / config.confidence_delta).ln();
    let required_inner_samples = 2.0
        * config.outer_samples as f64
        * config.lipschitz_h.powi(2)
        * config.sigma_w.powi(2)
        / (config.concentration_c_h.powi(2) * log_term);
    if !required_inner_samples.is_finite() {
        return Err("PAC-Bayes required inner sample count is non-finite".into());
    }
    if (config.inner_samples as f64) < required_inner_samples {
        return Err(format!(
            "inner sample count {} is below the required concentration threshold {required_inner_samples}",
            config.inner_samples
        ));
    }

    let mut empirical_policy_reward = 0.0;
    let mut policy_kl = 0.0;
    for ((probability, prior), reward) in policy_probabilities.iter()
        .zip(prior_policy)
        .zip(reward_estimates_nats)
    {
        empirical_policy_reward += probability * reward;
        if *probability > 0.0 {
            if *prior <= 0.0 {
                return Err("posterior policy is not absolutely continuous with respect to its prior".into());
            }
            policy_kl += probability * (probability / prior).ln();
        }
    }

    let estimator_penalty = config.precision_lambda
        * config.lipschitz_f.powi(2)
        * config.concentration_c_h.powi(2)
        / (2.0 * config.outer_samples as f64);
    let complexity_penalty = (
        policy_kl + (1.0 / config.confidence_delta).ln()
    ) / config.precision_lambda;
    let lower_bound = empirical_policy_reward - estimator_penalty - complexity_penalty;

    Ok(PacBayesLowerBound {
        lower_bound_nats: lower_bound,
        empirical_policy_reward_nats: empirical_policy_reward,
        policy_kl_to_prior_nats: policy_kl,
        estimator_penalty_nats: estimator_penalty,
        policy_complexity_penalty_nats: complexity_penalty,
        required_inner_samples,
        actual_outer_samples: config.outer_samples,
        actual_inner_samples: config.inner_samples,
        confidence_delta: config.confidence_delta,
        validity_scope: "conditional_on_external_verification_of_paper_regularity_and_boundedness_assumptions",
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn candidate(id: &str, likelihoods: Vec<Vec<f64>>, cost: u64) -> StochasticPolicyCandidate {
        StochasticPolicyCandidate {
            action_id: id.into(),
            cost_units: cost,
            eligible: true,
            design: DiscreteDesignModel {
                model_ids: vec!["m0".into(), "m1".into()],
                prior_weights: vec![0.5, 0.5],
                outcome_ids: (0..likelihoods[0].len())
                    .map(|index| format!("outcome-{index}"))
                    .collect(),
                likelihoods,
            },
        }
    }

    fn ranking_candidates() -> Vec<StochasticPolicyCandidate> {
        vec![
            candidate(
                "nominal-sharp",
                vec![vec![0.025, 0.938, 0.037], vec![0.958, 0.018, 0.024]],
                1,
            ),
            candidate(
                "robust-balanced",
                vec![vec![0.486, 0.514, 0.0], vec![0.0, 0.208, 0.792]],
                2,
            ),
        ]
    }

    #[test]
    fn shared_kl_policy_uses_one_common_beta_for_all_actions() {
        let candidates = ranking_candidates();
        let result = optimize_shared_kl_policy(&candidates, &[0.5, 0.5], 2, 0.05, 96).unwrap();
        let beta = result.common_dual_beta.unwrap();
        assert!(beta.is_finite() && beta > 0.0);
        assert!((result.alpha - beta / (1.0 + beta)).abs() < 1e-12);

        let action_values: Vec<f64> = candidates.iter()
            .map(|candidate| candidate.design.sibson_alpha_mutual_information(result.alpha).unwrap())
            .collect();
        let expected: f64 = action_values.iter().sum::<f64>() / 2.0;
        assert!((result.expected_sibson_information_nats - expected).abs() < 1e-12);
        assert!((result.robust_policy_value_nats
            - (expected - beta * 0.05)).abs() < 1e-12);
    }

    #[test]
    fn zero_shared_kl_radius_recovers_policy_average_shannon_information() {
        let candidates = ranking_candidates();
        let result = optimize_shared_kl_policy(&candidates, &[0.25, 0.75], 2, 0.0, 64).unwrap();
        let expected = 0.25 * candidates[0].design.shannon_mutual_information().unwrap()
            + 0.75 * candidates[1].design.shannon_mutual_information().unwrap();
        assert_eq!(result.alpha, 1.0);
        assert_eq!(result.common_dual_beta, None);
        assert!((result.robust_policy_value_nats - expected).abs() < 1e-12);
    }

    #[test]
    fn shared_kl_dual_handles_extreme_radii_without_alpha_rounding_failure() {
        let candidates = ranking_candidates();
        let tiny_radius = optimize_shared_kl_policy(
            &candidates, &[0.5, 0.5], 2, 1e-20, 64,
        ).unwrap();
        assert_eq!(tiny_radius.alpha, 1.0);
        assert!(tiny_radius.robust_policy_value_nats.is_finite());

        let large_radius = optimize_shared_kl_policy(
            &candidates, &[0.5, 0.5], 2, 1e6, 64,
        ).unwrap();
        assert!(large_radius.alpha.is_finite());
        assert!(large_radius.robust_policy_value_nats.is_finite());
    }

    #[test]
    fn shared_kl_policy_rejects_ineligible_or_over_cap_positive_mass() {
        let mut candidates = ranking_candidates();
        candidates[1].eligible = false;
        assert!(optimize_shared_kl_policy(&candidates, &[0.5, 0.5], 2, 0.05, 64).is_err());
        candidates[1].eligible = true;
        assert!(optimize_shared_kl_policy(&candidates, &[0.5, 0.5], 1, 0.05, 64).is_err());
    }

    #[test]
    fn gibbs_policy_matches_closed_form_and_variational_objective() {
        let ids = vec!["a".into(), "b".into(), "c".into()];
        let rewards = vec![0.2, 0.7, 0.4];
        let prior = vec![0.2, 0.5, 0.3];
        let policy = gibbs_policy_from_rewards(
            &ids, &rewards, &prior, &[true, true, true], &[1, 1, 1], 2, 3.0,
        ).unwrap();
        let unnormalized: Vec<f64> = prior.iter().zip(&rewards)
            .map(|(p, r)| p * (3.0_f64 * r).exp())
            .collect();
        let normalizer: f64 = unnormalized.iter().sum();
        for (actual, unnormalized) in policy.action_weights.iter().zip(unnormalized) {
            assert!((actual.probability - unnormalized / normalizer).abs() < 1e-12);
        }
        assert!((policy.variational_objective_nats
            - (policy.expected_information_nats - policy.kl_to_prior_nats / 3.0)).abs() < 1e-12);
        assert!((policy.action_weights.iter().map(|item| item.probability).sum::<f64>() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn gibbs_policy_keeps_zero_prior_mass_at_zero_and_enforces_hard_cap() {
        let ids = vec!["feasible".into(), "excluded".into()];
        let policy = gibbs_policy_from_rewards(
            &ids, &[0.2, 1.0], &[1.0, 0.0], &[true, false], &[1, 99], 2, 2.0,
        ).unwrap();
        assert_eq!(policy.action_weights[1].probability, 0.0);
        assert_eq!(policy.action_weights[0].probability, 1.0);
        assert!(gibbs_policy_from_rewards(
            &ids, &[0.2, 1.0], &[0.5, 0.5], &[true, false], &[1, 1], 2, 2.0,
        ).is_err());
    }

    fn pac_config() -> PacBayesConcentrationConfig {
        PacBayesConcentrationConfig {
            outer_samples: 1_000,
            inner_samples: 100_000,
            confidence_delta: 0.05,
            precision_lambda: 2.0,
            lipschitz_f: 1.0,
            lipschitz_h: 1.0,
            concentration_c_h: 1.0,
            sigma_w: 1.0,
        }
    }

    #[test]
    fn pac_bayes_lower_bound_matches_published_formula() {
        let policy = [0.25, 0.75];
        let prior = [0.5, 0.5];
        let rewards = [0.2, 0.6];
        let config = pac_config();
        let result = calculate_pac_bayes_lower_bound(&policy, &prior, &rewards, config).unwrap();
        let expected_reward = 0.25 * 0.2 + 0.75 * 0.6;
        let kl = 0.25 * (0.25_f64 / 0.5).ln() + 0.75 * (0.75_f64 / 0.5).ln();
        let penalty = config.precision_lambda / (2.0 * config.outer_samples as f64);
        let complexity = (kl + (1.0 / config.confidence_delta).ln()) / config.precision_lambda;
        assert!((result.lower_bound_nats - (expected_reward - penalty - complexity)).abs() < 1e-12);
        assert!(result.required_inner_samples < config.inner_samples as f64);
        assert!(result.validity_scope.contains("external_verification"));
    }

    #[test]
    fn pac_bayes_lower_bound_rejects_insufficient_inner_samples() {
        let mut config = pac_config();
        config.inner_samples = 1;
        assert!(calculate_pac_bayes_lower_bound(&[0.5, 0.5], &[0.5, 0.5], &[0.2, 0.4], config).is_err());
    }

    #[test]
    fn pac_bayes_lower_bound_rejects_policy_mass_outside_prior_support() {
        let config = pac_config();
        assert!(calculate_pac_bayes_lower_bound(&[0.0, 1.0], &[1.0, 0.0], &[0.2, 0.4], config).is_err());
    }

    #[test]
    fn shared_kl_optimizer_rejects_invalid_distribution_and_search_budget() {
        let candidates = ranking_candidates();
        assert!(optimize_shared_kl_policy(&candidates, &[0.2, 0.2], 2, 0.05, 64).is_err());
        assert!(optimize_shared_kl_policy(&candidates, &[0.5, 0.5], 2, 0.05, 0).is_err());
    }
}

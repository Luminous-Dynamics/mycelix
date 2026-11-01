//! Exact finite-discrete Sibson alpha-mutual information reference.
//!
//! This is a mathematical reference for the robust-information objective in
//! maximin robust Bayesian experimental design. It is not the paper's
//! PAC-Bayes stochastic-policy optimizer, KL-ambiguity solver for continuous
//! observation models, or a decision-EVSI implementation.

use std::collections::BTreeSet;

/// Finite model prior and model-conditional outcome likelihoods for one design.
#[derive(Clone, Debug, PartialEq)]
pub struct DiscreteDesignModel {
    pub model_ids: Vec<String>,
    /// Non-negative prior masses, normalized internally.
    pub prior_weights: Vec<f64>,
    pub outcome_ids: Vec<String>,
    /// Row-major p(outcome | model), each row summing to one.
    pub likelihoods: Vec<Vec<f64>>,
}

/// Exact finite-discrete information metrics for one candidate design.
#[derive(Clone, Debug, PartialEq)]
pub struct RobustInformationMetrics {
    pub shannon_mutual_information_nats: f64,
    pub sibson_alpha: f64,
    pub sibson_mutual_information_nats: f64,
}

/// Worst-case joint distribution associated with the Sibson alpha objective.
///
/// `alpha_outcome_marginal` is the tilted outcome marginal p_alpha(x).
/// `joint_weights` is q*(model, outcome) from the published geometric-mixture
/// expression. `kl_to_nominal_nats` is the KL cost D_KL(q* || p(model,outcome)).
#[derive(Clone, Debug, PartialEq)]
pub struct WorstCaseJointDistribution {
    pub alpha: f64,
    pub alpha_outcome_marginal: Vec<f64>,
    pub model_marginal: Vec<f64>,
    pub outcome_marginal: Vec<f64>,
    pub joint_weights: Vec<Vec<f64>>,
    pub kl_to_nominal_nats: f64,
}

/// Validate identifiers, prior and likelihood matrix, returning normalized prior.
impl DiscreteDesignModel {
    pub fn validate(&self) -> Result<Vec<f64>, String> {
        if self.model_ids.is_empty() || self.outcome_ids.is_empty() {
            return Err("at least one model and one outcome are required".into());
        }
        if self.model_ids.len() != self.prior_weights.len()
            || self.model_ids.len() != self.likelihoods.len()
        {
            return Err("model IDs, prior weights and likelihood rows must align".into());
        }
        if self.model_ids.iter().any(|id| id.trim().is_empty())
            || self.outcome_ids.iter().any(|id| id.trim().is_empty())
        {
            return Err("model and outcome identifiers must be non-empty".into());
        }
        let unique_models: BTreeSet<&str> = self.model_ids.iter().map(String::as_str).collect();
        let unique_outcomes: BTreeSet<&str> = self.outcome_ids.iter().map(String::as_str).collect();
        if unique_models.len() != self.model_ids.len()
            || unique_outcomes.len() != self.outcome_ids.len()
        {
            return Err("model and outcome identifiers must be unique".into());
        }

        let prior_total: f64 = self.prior_weights.iter().sum();
        if !prior_total.is_finite() || prior_total <= 0.0
            || self.prior_weights.iter().any(|weight| !weight.is_finite() || *weight < 0.0)
        {
            return Err("prior weights must be finite, non-negative and have positive sum".into());
        }

        for (model_index, row) in self.likelihoods.iter().enumerate() {
            if row.len() != self.outcome_ids.len() {
                return Err(format!(
                    "likelihood row for {:?} has wrong outcome dimension",
                    self.model_ids[model_index]
                ));
            }
            if row.iter().any(|probability| {
                !probability.is_finite() || *probability < 0.0 || *probability > 1.0
            }) {
                return Err(format!(
                    "likelihood row for {:?} contains an invalid probability",
                    self.model_ids[model_index]
                ));
            }
            let row_total: f64 = row.iter().sum();
            if (row_total - 1.0).abs() > 1e-10 {
                return Err(format!(
                    "likelihood row for {:?} sums to {row_total}, not one",
                    self.model_ids[model_index]
                ));
            }
        }

        Ok(self.prior_weights.iter().map(|weight| weight / prior_total).collect())
    }

    /// Nominal Shannon mutual information I(M; X) for this candidate design.
    pub fn shannon_mutual_information(&self) -> Result<f64, String> {
        let prior = self.validate()?;
        let mut marginal_outcomes = vec![0.0; self.outcome_ids.len()];
        for (row, weight) in self.likelihoods.iter().zip(&prior) {
            for (marginal, likelihood) in marginal_outcomes.iter_mut().zip(row) {
                *marginal += weight * likelihood;
            }
        }

        let mut information = 0.0;
        for (row, weight) in self.likelihoods.iter().zip(&prior) {
            for (likelihood, marginal) in row.iter().zip(&marginal_outcomes) {
                if *marginal > 0.0 {
                    let joint = weight * likelihood;
                    if joint > 0.0 {
                        information += joint * (likelihood / marginal).ln();
                    }
                }
            }
        }
        if information.is_finite() {
            Ok(information.max(0.0))
        } else {
            Err("Shannon mutual information is not finite".into())
        }
    }

    /// Exact Sibson alpha-mutual information in nats for 0 < alpha < 1.
    ///
    /// I^S_alpha(M;X) = alpha/(alpha-1) * log Σ_x
    /// (Σ_m p(m) p(x|m)^alpha)^(1/alpha).
    pub fn sibson_alpha_mutual_information(&self, alpha: f64) -> Result<f64, String> {
        if !alpha.is_finite() || !(0.0..1.0).contains(&alpha) {
            return Err("Sibson alpha must be finite and strictly between zero and one".into());
        }
        let prior = self.validate()?;
        let mut inner_sums = vec![0.0; self.outcome_ids.len()];
        for (row, weight) in self.likelihoods.iter().zip(&prior) {
            for (inner_sum, likelihood) in inner_sums.iter_mut().zip(row) {
                *inner_sum += weight * likelihood.powf(alpha);
            }
        }
        let outer_sum: f64 = inner_sums.iter()
            .map(|inner_sum| inner_sum.powf(1.0 / alpha))
            .sum();
        if !outer_sum.is_finite() || outer_sum <= 0.0 {
            return Err("Sibson outer sum must be finite and positive".into());
        }

        let information = alpha / (alpha - 1.0) * outer_sum.ln();
        if !information.is_finite() {
            return Err("Sibson mutual information is not finite".into());
        }
        Ok(information.max(0.0))
    }

    /// Rényi alpha-divergence D_alpha[p(model | outcome) || p(model)]
    /// for one outcome under the nominal Bayesian posterior.
    ///
    /// This is the conditional robust-information quantity in the
    /// maximin-design paper. It uses the ordinary posterior in the divergence;
    /// the alpha-tilted posterior is a distinct worst-case belief update.
    pub fn conditional_renyi_information(
        &self,
        outcome_index: usize,
        alpha: f64,
    ) -> Result<f64, String> {
        if !alpha.is_finite() || !(0.0..1.0).contains(&alpha) {
            return Err("Rényi alpha must be finite and strictly between zero and one".into());
        }
        if outcome_index >= self.outcome_ids.len() {
            return Err("outcome index is out of range".into());
        }
        let prior = self.validate()?;
        let marginal: f64 = self.likelihoods.iter()
            .zip(&prior)
            .map(|(row, weight)| weight * row[outcome_index])
            .sum();
        if marginal <= 0.0 {
            return Err("conditional Rényi information is undefined for a zero-probability outcome".into());
        }

        let posterior: Vec<f64> = self.likelihoods.iter()
            .zip(&prior)
            .map(|(row, weight)| weight * row[outcome_index] / marginal)
            .collect();
        let affinity: f64 = posterior.iter()
            .zip(&prior)
            .map(|(post, prior)| {
                if *post == 0.0 || *prior == 0.0 {
                    0.0
                } else {
                    post.powf(alpha) * prior.powf(1.0 - alpha)
                }
            })
            .sum();
        if !affinity.is_finite() || affinity <= 0.0 || affinity > 1.0 + 1e-10 {
            return Err("Rényi affinity is invalid".into());
        }
        let divergence = affinity.min(1.0).ln() / (alpha - 1.0);
        if divergence.is_finite() && divergence >= -1e-10 {
            Ok(divergence.max(0.0))
        } else {
            Err("conditional Rényi divergence is invalid".into())
        }
    }

    /// Reconstruct Sibson alpha-MI as the risk-sensitive exponential average
    /// of conditional Rényi divergences under the nominal outcome marginal:
    ///
    /// I^S_alpha = alpha/(alpha-1) * log E_p(x)[
    ///     exp((alpha-1)/alpha * D_alpha[p(model|x) || p(model)])
    /// ].
    ///
    /// This is an independent identity-based qualification path for the
    /// direct Sibson formula.
    pub fn sibson_from_conditional_renyi(&self, alpha: f64) -> Result<f64, String> {
        if !alpha.is_finite() || !(0.0..1.0).contains(&alpha) {
            return Err("Rényi alpha must be finite and strictly between zero and one".into());
        }
        let prior = self.validate()?;
        let mut risk_sensitive_expectation = 0.0;

        for outcome in 0..self.outcome_ids.len() {
            let marginal: f64 = self.likelihoods.iter()
                .zip(&prior)
                .map(|(row, weight)| weight * row[outcome])
                .sum();
            if marginal == 0.0 {
                continue;
            }
            let conditional_gain = self.conditional_renyi_information(outcome, alpha)?;
            let exponent = (alpha - 1.0) / alpha * conditional_gain;
            risk_sensitive_expectation += marginal * exponent.exp();
        }
        if !risk_sensitive_expectation.is_finite() || risk_sensitive_expectation <= 0.0 {
            return Err("risk-sensitive expectation is invalid".into());
        }
        let value = alpha / (alpha - 1.0) * risk_sensitive_expectation.ln();
        if value.is_finite() && value >= -1e-10 {
            Ok(value.max(0.0))
        } else {
            Err("risk-sensitive Sibson information is invalid".into())
        }
    }

    /// Construct the finite worst-case joint distribution q* associated
    /// with Sibson alpha-MI, using the published geometric-mixture form:
    ///
    /// q*(m,x) ∝ [p(m) p_alpha(x)]^(1-alpha) [p(m,x)]^alpha.
    ///
    /// Its conditional q*(m|x) is the alpha-tilted posterior.
    pub fn worst_case_joint_distribution(
        &self,
        alpha: f64,
    ) -> Result<WorstCaseJointDistribution, String> {
        if !alpha.is_finite() || !(0.0..1.0).contains(&alpha) {
            return Err("worst-case alpha must be finite and strictly between zero and one".into());
        }
        let prior = self.validate()?;
        let outcome_count = self.outcome_ids.len();

        let mut alpha_inner_sums = vec![0.0; outcome_count];
        for (row, weight) in self.likelihoods.iter().zip(&prior) {
            for (inner_sum, likelihood) in alpha_inner_sums.iter_mut().zip(row) {
                *inner_sum += weight * likelihood.powf(alpha);
            }
        }
        let mut alpha_outcome_marginal: Vec<f64> = alpha_inner_sums.iter()
            .map(|inner_sum| inner_sum.powf(1.0 / alpha))
            .collect();
        let alpha_total: f64 = alpha_outcome_marginal.iter().sum();
        if !alpha_total.is_finite() || alpha_total <= 0.0 {
            return Err("alpha outcome marginal has no finite positive mass".into());
        }
        for probability in &mut alpha_outcome_marginal {
            *probability /= alpha_total;
        }

        let mut joint_weights = vec![vec![0.0; outcome_count]; self.model_ids.len()];
        let mut joint_total = 0.0;
        for ((row, weight), joint_row) in self.likelihoods.iter()
            .zip(&prior)
            .zip(joint_weights.iter_mut())
        {
            for ((likelihood, alpha_outcome), joint_value) in row.iter()
                .zip(&alpha_outcome_marginal)
                .zip(joint_row.iter_mut())
            {
                let nominal_joint = weight * likelihood;
                let alpha_reference = weight * alpha_outcome;
                let value = if nominal_joint == 0.0 || alpha_reference == 0.0 {
                    0.0
                } else {
                    alpha_reference.powf(1.0 - alpha)
                        * nominal_joint.powf(alpha)
                };
                if !value.is_finite() || value < 0.0 {
                    return Err("worst-case joint weight is invalid".into());
                }
                *joint_value = value;
                joint_total += value;
            }
        }
        if !joint_total.is_finite() || joint_total <= 0.0 {
            return Err("worst-case joint distribution has no finite positive mass".into());
        }
        for row in &mut joint_weights {
            for value in row {
                *value /= joint_total;
            }
        }

        let mut model_marginal = vec![0.0; self.model_ids.len()];
        let mut outcome_marginal = vec![0.0; outcome_count];
        let mut kl_to_nominal_nats = 0.0;
        for (((joint_row, nominal_row), model_prior), model_total) in joint_weights.iter()
            .zip(&self.likelihoods)
            .zip(&prior)
            .zip(model_marginal.iter_mut())
        {
            for ((probability, likelihood), outcome_total) in joint_row.iter()
                .zip(nominal_row)
                .zip(outcome_marginal.iter_mut())
            {
                *model_total += probability;
                *outcome_total += probability;
                if *probability > 0.0 {
                    let nominal = model_prior * likelihood;
                    if nominal <= 0.0 {
                        return Err("worst-case distribution escaped nominal support".into());
                    }
                    kl_to_nominal_nats += probability * (probability / nominal).ln();
                }
            }
        }
        if !kl_to_nominal_nats.is_finite() || kl_to_nominal_nats < -1e-10 {
            return Err("worst-case KL ambiguity cost is invalid".into());
        }

        Ok(WorstCaseJointDistribution {
            alpha,
            alpha_outcome_marginal,
            model_marginal,
            outcome_marginal,
            joint_weights,
            kl_to_nominal_nats: kl_to_nominal_nats.max(0.0),
        })
    }

    /// Numerically invert the single-design KL ambiguity cost to obtain alpha.
    ///
    /// The caller supplies the smallest alpha to consider because the alpha
    /// -> 0 limit may be singular or support-sensitive. A zero radius returns
    /// the nominal limit alpha=1. This is a finite one-design reference, not
    /// the paper's stochastic-policy average-radius optimizer.
    pub fn alpha_for_kl_radius(
        &self,
        radius_nats: f64,
        minimum_alpha: f64,
        iterations: usize,
    ) -> Result<f64, String> {
        if !radius_nats.is_finite() || radius_nats < 0.0 {
            return Err("KL radius must be finite and non-negative".into());
        }
        if !minimum_alpha.is_finite() || !(0.0..1.0).contains(&minimum_alpha) {
            return Err("minimum alpha must lie strictly between zero and one".into());
        }
        if minimum_alpha >= 1.0 - 1e-8 {
            return Err("minimum alpha must leave a non-empty bisection interval".into());
        }
        if iterations == 0 {
            return Err("at least one bisection iteration is required".into());
        }
        self.validate()?;
        if radius_nats == 0.0 {
            return Ok(1.0);
        }

        let maximum_radius = self
            .worst_case_joint_distribution(minimum_alpha)?
            .kl_to_nominal_nats;
        if radius_nats > maximum_radius {
            return Err(format!(
                "requested radius {radius_nats} exceeds the radius {maximum_radius} at minimum alpha {minimum_alpha}"
            ));
        }
        if maximum_radius == 0.0 {
            return Err("candidate design has no nonzero KL ambiguity radius".into());
        }

        // The dual relationship is monotone: lower alpha permits a larger
        // adversarial deviation. Verify it on a fixed grid before bisection.
        let mut previous_radius = maximum_radius;
        for index in 1..=32 {
            let alpha = minimum_alpha
                + (1.0 - 1e-8 - minimum_alpha) * index as f64 / 32.0;
            let current = self
                .worst_case_joint_distribution(alpha)?
                .kl_to_nominal_nats;
            if current > previous_radius + 1e-9 {
                return Err("KL ambiguity radius is not monotone over the checked alpha grid".into());
            }
            previous_radius = current;
        }

        let mut lower = minimum_alpha;
        let mut upper = 1.0 - 1e-8;
        for _ in 0..iterations {
            let midpoint = 0.5 * (lower + upper);
            let current = self
                .worst_case_joint_distribution(midpoint)?
                .kl_to_nominal_nats;
            if current > radius_nats {
                lower = midpoint;
            } else {
                upper = midpoint;
            }
        }
        Ok(0.5 * (lower + upper))
    }

    /// Return both nominal Shannon MI and Sibson alpha-MI.
    pub fn information_metrics(&self, alpha: f64) -> Result<RobustInformationMetrics, String> {
        Ok(RobustInformationMetrics {
            shannon_mutual_information_nats: self.shannon_mutual_information()?,
            sibson_alpha: alpha,
            sibson_mutual_information_nats: self.sibson_alpha_mutual_information(alpha)?,
        })
    }

    /// Alpha-tilted posterior p_alpha(model | outcome) proportional to
    /// p(model) p(outcome | model)^alpha.
    pub fn alpha_tilted_posterior(
        &self,
        outcome_index: usize,
        alpha: f64,
    ) -> Result<Vec<f64>, String> {
        if !alpha.is_finite() || !(0.0..=1.0).contains(&alpha) || alpha == 0.0 {
            return Err("posterior tilt alpha must lie in (0, 1]".into());
        }
        if outcome_index >= self.outcome_ids.len() {
            return Err("outcome index is out of range".into());
        }
        let prior = self.validate()?;
        let unnormalized: Vec<f64> = self.likelihoods.iter()
            .zip(&prior)
            .map(|(row, weight)| weight * row[outcome_index].powf(alpha))
            .collect();
        let total: f64 = unnormalized.iter().sum();
        if !total.is_finite() || total <= 0.0 {
            return Err("alpha-tilted posterior has no finite positive mass".into());
        }
        Ok(unnormalized.iter().map(|weight| weight / total).collect())
    }
}


/// One candidate discrete measurement action for a one-step design comparison.
#[derive(Clone, Debug, PartialEq)]
pub struct DiscreteDesignCandidate {
    pub action_id: String,
    pub cost_units: u64,
    pub eligible: bool,
    pub outcome_ids: Vec<String>,
    /// Row-major p(outcome | model), in the common model order.
    pub likelihoods: Vec<Vec<f64>>,
}

/// Objective used to rank feasible one-step candidate measurement actions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DesignInformationObjective {
    Shannon,
    SibsonAlpha(f64),
}

/// Deterministic selection result with hard-feasibility accounting.
#[derive(Clone, Debug, PartialEq)]
pub struct DiscreteDesignSelection {
    pub action_id: String,
    pub cost_units: u64,
    pub objective: DesignInformationObjective,
    pub objective_value_nats: f64,
    pub candidate_count: usize,
    pub feasible_candidate_count: usize,
}

/// Robust per-design information score for a declared KL ambiguity radius.
#[derive(Clone, Debug, PartialEq)]
pub struct RadiusRobustInformation {
    pub requested_kl_radius_nats: f64,
    pub effective_kl_radius_nats: f64,
    pub alpha: f64,
    pub sibson_information_nats: f64,
    /// True when the requested radius exceeds the supported alpha search range.
    pub alpha_search_clamped: bool,
}

fn validate_kl_ambiguity_inputs(
    radius_nats: f64,
    minimum_alpha: f64,
    iterations: usize,
) -> Result<(), String> {
    if !radius_nats.is_finite() || radius_nats < 0.0 {
        return Err("KL ambiguity radius must be finite and non-negative".into());
    }
    if !minimum_alpha.is_finite()
        || !(0.0..1.0).contains(&minimum_alpha)
        || minimum_alpha >= 1.0 - 1e-8
    {
        return Err("minimum alpha must lie strictly between zero and 1 - 1e-8".into());
    }
    if iterations == 0 {
        return Err("at least one bisection iteration is required".into());
    }
    Ok(())
}

/// Deterministic one-design maximin information reference for a KL radius.
///
/// For each fixed candidate design, this inverts the q* KL cost to choose its
/// alpha, then evaluates Sibson alpha-MI. This is the fixed-design ambiguity
/// problem, not the paper's stochastic-policy problem with one shared average
/// KL budget across a policy distribution.
pub fn robust_information_at_kl_radius(
    design: &DiscreteDesignModel,
    radius_nats: f64,
    minimum_alpha: f64,
    iterations: usize,
) -> Result<RadiusRobustInformation, String> {
    validate_kl_ambiguity_inputs(radius_nats, minimum_alpha, iterations)?;
    design.validate()?;
    if radius_nats == 0.0 {
        return Ok(RadiusRobustInformation {
            requested_kl_radius_nats: 0.0,
            effective_kl_radius_nats: 0.0,
            alpha: 1.0,
            sibson_information_nats: design.shannon_mutual_information()?,
            alpha_search_clamped: false,
        });
    }

    let maximum_radius = design
        .worst_case_joint_distribution(minimum_alpha)?
        .kl_to_nominal_nats;
    if maximum_radius <= 1e-14 {
        return Ok(RadiusRobustInformation {
            requested_kl_radius_nats: radius_nats,
            effective_kl_radius_nats: 0.0,
            alpha: 1.0,
            sibson_information_nats: 0.0,
            alpha_search_clamped: false,
        });
    }

    let alpha_search_clamped = radius_nats >= maximum_radius;
    let alpha = if alpha_search_clamped {
        minimum_alpha
    } else {
        design.alpha_for_kl_radius(radius_nats, minimum_alpha, iterations)?
    };
    let worst_case = design.worst_case_joint_distribution(alpha)?;
    let information = design.sibson_alpha_mutual_information(alpha)?;

    Ok(RadiusRobustInformation {
        requested_kl_radius_nats: radius_nats,
        effective_kl_radius_nats: worst_case.kl_to_nominal_nats,
        alpha,
        sibson_information_nats: information,
        alpha_search_clamped,
    })
}

/// Select the best feasible action when every deterministic candidate is
/// evaluated under the same per-design KL ambiguity radius.
///
/// A candidate whose supported q* KL path saturates below the requested
/// radius is evaluated at `minimum_alpha` and marked as clamped in its
/// detailed score. This API is not a shared-average-KL stochastic-policy
/// optimizer.
pub fn select_discrete_design_at_kl_radius(
    model_ids: &[String],
    prior_weights: &[f64],
    candidates: &[DiscreteDesignCandidate],
    budget_units: u64,
    radius_nats: f64,
    minimum_alpha: f64,
    iterations: usize,
) -> Result<Option<(DiscreteDesignSelection, RadiusRobustInformation)>, String> {
    validate_kl_ambiguity_inputs(radius_nats, minimum_alpha, iterations)?;
    let mut action_ids = BTreeSet::new();
    let mut best: Option<(DiscreteDesignSelection, RadiusRobustInformation)> = None;
    let mut feasible_candidate_count = 0;

    for candidate in candidates {
        if candidate.action_id.trim().is_empty()
            || !action_ids.insert(candidate.action_id.as_str())
        {
            return Err("candidate action IDs must be non-empty and unique".into());
        }
        let design = DiscreteDesignModel {
            model_ids: model_ids.to_vec(),
            prior_weights: prior_weights.to_vec(),
            outcome_ids: candidate.outcome_ids.clone(),
            likelihoods: candidate.likelihoods.clone(),
        };
        design.validate()?;
        if !candidate.eligible || candidate.cost_units > budget_units {
            continue;
        }
        feasible_candidate_count += 1;
        let score = robust_information_at_kl_radius(
            &design,
            radius_nats,
            minimum_alpha,
            iterations,
        )?;
        let should_replace = match &best {
            None => true,
            Some((current, _)) => {
                score.sibson_information_nats > current.objective_value_nats + 1e-12
                    || ((score.sibson_information_nats - current.objective_value_nats).abs() <= 1e-12
                        && candidate.action_id < current.action_id)
            }
        };
        if should_replace {
            best = Some((
                DiscreteDesignSelection {
                    action_id: candidate.action_id.clone(),
                    cost_units: candidate.cost_units,
                    objective: DesignInformationObjective::SibsonAlpha(score.alpha),
                    objective_value_nats: score.sibson_information_nats,
                    candidate_count: candidates.len(),
                    feasible_candidate_count: 0,
                },
                score,
            ));
        }
    }

    if let Some((selection, _)) = &mut best {
        selection.feasible_candidate_count = feasible_candidate_count;
    }
    Ok(best)
}

/// Select the best single action by nominal Shannon MI or finite Sibson alpha-MI.
///
/// This is a deterministic one-step reference with hard eligibility and budget
/// filters. It is not a multi-step stochastic policy, a PAC-Bayes policy
/// optimizer or a sequential EVSI controller.
pub fn select_discrete_design(
    model_ids: &[String],
    prior_weights: &[f64],
    candidates: &[DiscreteDesignCandidate],
    budget_units: u64,
    objective: DesignInformationObjective,
) -> Result<Option<DiscreteDesignSelection>, String> {
    if let DesignInformationObjective::SibsonAlpha(alpha) = objective {
        if !alpha.is_finite() || !(0.0..1.0).contains(&alpha) {
            return Err("Sibson alpha must be finite and strictly between zero and one".into());
        }
    }
    let mut action_ids = BTreeSet::new();
    let mut best: Option<DiscreteDesignSelection> = None;
    let mut feasible_candidate_count = 0;

    for candidate in candidates {
        if candidate.action_id.trim().is_empty()
            || !action_ids.insert(candidate.action_id.as_str())
        {
            return Err("candidate action IDs must be non-empty and unique".into());
        }
        let design = DiscreteDesignModel {
            model_ids: model_ids.to_vec(),
            prior_weights: prior_weights.to_vec(),
            outcome_ids: candidate.outcome_ids.clone(),
            likelihoods: candidate.likelihoods.clone(),
        };
        // Validate even ineligible candidates so malformed model inputs cannot
        // be hidden behind a feasibility flag.
        design.validate()?;

        if !candidate.eligible || candidate.cost_units > budget_units {
            continue;
        }
        feasible_candidate_count += 1;
        let value = match objective {
            DesignInformationObjective::Shannon => design.shannon_mutual_information()?,
            DesignInformationObjective::SibsonAlpha(alpha) => {
                design.sibson_alpha_mutual_information(alpha)?
            }
        };
        if !value.is_finite() {
            return Err(format!("objective for action {:?} is not finite", candidate.action_id));
        }

        let should_replace = match &best {
            None => true,
            Some(current) => {
                value > current.objective_value_nats + 1e-12
                    || ((value - current.objective_value_nats).abs() <= 1e-12
                        && candidate.action_id < current.action_id)
            }
        };
        if should_replace {
            best = Some(DiscreteDesignSelection {
                action_id: candidate.action_id.clone(),
                cost_units: candidate.cost_units,
                objective,
                objective_value_nats: value,
                candidate_count: candidates.len(),
                feasible_candidate_count: 0,
            });
        }
    }

    if let Some(selection) = &mut best {
        selection.feasible_candidate_count = feasible_candidate_count;
    }
    Ok(best)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn independent_design() -> DiscreteDesignModel {
        DiscreteDesignModel {
            model_ids: vec!["matrix".into(), "preferential".into()],
            prior_weights: vec![0.4, 0.6],
            outcome_ids: vec!["negative".into(), "positive".into()],
            likelihoods: vec![vec![0.25, 0.75], vec![0.25, 0.75]],
        }
    }

    fn complementary_ct_tracer_pair() -> DiscreteDesignModel {
        DiscreteDesignModel {
            model_ids: vec!["positive_cross_covariance".into(), "negative_cross_covariance".into()],
            prior_weights: vec![0.5, 0.5],
            outcome_ids: vec!["ct_plus_tracer_plus".into(), "ct_plus_tracer_minus".into(),
                "ct_minus_tracer_plus".into(), "ct_minus_tracer_minus".into()],
            likelihoods: vec![
                vec![0.45, 0.05, 0.05, 0.45],
                vec![0.05, 0.45, 0.45, 0.05],
            ],
        }
    }

    fn robust_ranking_fixture() -> Vec<DiscreteDesignCandidate> {
        vec![
            DiscreteDesignCandidate {
                action_id: "nominal_sharp".into(),
                cost_units: 2,
                eligible: true,
                outcome_ids: vec!["x0".into(), "x1".into(), "x2".into()],
                likelihoods: vec![
                    vec![0.025, 0.938, 0.037],
                    vec![0.958, 0.018, 0.024],
                ],
            },
            DiscreteDesignCandidate {
                action_id: "robust_balanced".into(),
                cost_units: 2,
                eligible: true,
                outcome_ids: vec!["y0".into(), "y1".into(), "y2".into()],
                likelihoods: vec![
                    vec![0.486, 0.514, 0.0],
                    vec![0.0, 0.208, 0.792],
                ],
            },
        ]
    }

    #[test]
    fn rejects_invalid_robust_objective_even_when_no_action_is_feasible() {
        let mut candidates = robust_ranking_fixture();
        for candidate in &mut candidates {
            candidate.eligible = false;
        }
        assert!(select_discrete_design(
            &["m0".into(), "m1".into()],
            &[0.5, 0.5],
            &candidates,
            2,
            DesignInformationObjective::SibsonAlpha(1.0),
        ).is_err());
    }

    #[test]
    fn rejects_invalid_kl_search_controls_even_at_zero_radius() {
        let design = complementary_ct_tracer_pair();
        assert!(robust_information_at_kl_radius(&design, 0.0, 1.0, 64).is_err());
        assert!(robust_information_at_kl_radius(&design, 0.0, 0.05, 0).is_err());
    }

    #[test]
    fn radius_selector_validates_controls_before_empty_feasible_pool() {
        let mut candidates = robust_ranking_fixture();
        for candidate in &mut candidates {
            candidate.eligible = false;
        }
        assert!(select_discrete_design_at_kl_radius(
            &["m0".into(), "m1".into()],
            &[0.5, 0.5],
            &candidates,
            2,
            0.1,
            1.0,
            64,
        ).is_err());
    }

    #[test]
    fn fixed_kl_radius_resolves_alpha_and_recovers_requested_radius() {
        let design = complementary_ct_tracer_pair();
        let target_radius = design.worst_case_joint_distribution(0.5).unwrap()
            .kl_to_nominal_nats;
        let score = robust_information_at_kl_radius(
            &design,
            target_radius,
            0.05,
            64,
        ).unwrap();
        assert!((score.alpha - 0.5).abs() < 1e-8);
        assert!((score.effective_kl_radius_nats - target_radius).abs() < 1e-8);
        assert!((score.sibson_information_nats
            - design.sibson_alpha_mutual_information(0.5).unwrap()).abs() < 1e-10);
        assert!(!score.alpha_search_clamped);
    }

    #[test]
    fn zero_kl_radius_recovers_nominal_shannon_design_value() {
        let design = complementary_ct_tracer_pair();
        let score = robust_information_at_kl_radius(&design, 0.0, 0.05, 64).unwrap();
        assert_eq!(score.alpha, 1.0);
        assert_eq!(score.effective_kl_radius_nats, 0.0);
        assert!((score.sibson_information_nats
            - design.shannon_mutual_information().unwrap()).abs() < 1e-12);
    }

    #[test]
    fn uninformative_design_handles_positive_kl_radius_without_false_information() {
        let design = independent_design();
        let score = robust_information_at_kl_radius(&design, 0.1, 0.05, 64).unwrap();
        assert_eq!(score.alpha, 1.0);
        assert_eq!(score.sibson_information_nats, 0.0);
        assert_eq!(score.effective_kl_radius_nats, 0.0);
    }

    #[test]
    fn per_design_radius_selector_enforces_budget_and_eligibility() {
        let candidates = robust_ranking_fixture();
        let selected = select_discrete_design_at_kl_radius(
            &["m0".into(), "m1".into()],
            &[0.5, 0.5],
            &candidates,
            2,
            0.02,
            0.05,
            48,
        ).unwrap().unwrap();
        assert_eq!(selected.0.feasible_candidate_count, 2);
        assert!(selected.1.effective_kl_radius_nats <= 0.02 + 1e-8);
        assert!(selected.1.alpha >= 0.05);
    }

    #[test]
    fn per_design_radius_selector_marks_alpha_search_clamp() {
        let design = complementary_ct_tracer_pair();
        let maximum = design.worst_case_joint_distribution(0.05).unwrap()
            .kl_to_nominal_nats;
        let score = robust_information_at_kl_radius(
            &design,
            maximum * 2.0,
            0.05,
            64,
        ).unwrap();
        assert!(score.alpha_search_clamped);
        assert_eq!(score.alpha, 0.05);
        assert!((score.effective_kl_radius_nats - maximum).abs() < 1e-12);
    }

    #[test]
    fn robust_sibson_selection_can_differ_from_nominal_shannon_selection() {
        let candidates = robust_ranking_fixture();
        let model_ids = vec!["m0".into(), "m1".into()];
        let prior = vec![0.5, 0.5];

        let nominal = select_discrete_design(
            &model_ids,
            &prior,
            &candidates,
            2,
            DesignInformationObjective::Shannon,
        ).unwrap().unwrap();
        let robust = select_discrete_design(
            &model_ids,
            &prior,
            &candidates,
            2,
            DesignInformationObjective::SibsonAlpha(0.3),
        ).unwrap().unwrap();

        assert_eq!(nominal.action_id, "nominal_sharp");
        assert_eq!(robust.action_id, "robust_balanced");

        let nominal_model = DiscreteDesignModel {
            model_ids: model_ids.clone(),
            prior_weights: prior.clone(),
            outcome_ids: candidates[0].outcome_ids.clone(),
            likelihoods: candidates[0].likelihoods.clone(),
        };
        let robust_model = DiscreteDesignModel {
            model_ids,
            prior_weights: prior,
            outcome_ids: candidates[1].outcome_ids.clone(),
            likelihoods: candidates[1].likelihoods.clone(),
        };
        assert!(
            nominal_model.shannon_mutual_information().unwrap()
                > robust_model.shannon_mutual_information().unwrap()
        );
        assert!(
            nominal_model.sibson_alpha_mutual_information(0.3).unwrap()
                < robust_model.sibson_alpha_mutual_information(0.3).unwrap()
        );
        assert_eq!(robust.feasible_candidate_count, 2);
    }

    #[test]
    fn robust_selector_enforces_eligibility_and_budget_before_ranking() {
        let mut candidates = robust_ranking_fixture();
        candidates[0].cost_units = 5;
        candidates[1].eligible = false;
        candidates.push(DiscreteDesignCandidate {
            action_id: "feasible".into(),
            cost_units: 1,
            eligible: true,
            outcome_ids: vec!["negative".into(), "positive".into()],
            likelihoods: vec![vec![0.8, 0.2], vec![0.2, 0.8]],
        });

        let selected = select_discrete_design(
            &["m0".into(), "m1".into()],
            &[0.5, 0.5],
            &candidates,
            2,
            DesignInformationObjective::SibsonAlpha(0.5),
        ).unwrap().unwrap();
        assert_eq!(selected.action_id, "feasible");
        assert_eq!(selected.cost_units, 1);
        assert_eq!(selected.feasible_candidate_count, 1);
    }

    #[test]
    fn robust_selector_returns_none_when_no_candidate_is_feasible() {
        let mut candidates = robust_ranking_fixture();
        candidates[0].eligible = false;
        candidates[1].cost_units = 10;
        assert!(select_discrete_design(
            &["m0".into(), "m1".into()],
            &[0.5, 0.5],
            &candidates,
            2,
            DesignInformationObjective::SibsonAlpha(0.5),
        ).unwrap().is_none());
    }

    #[test]
    fn robust_selector_uses_deterministic_action_id_tie_break() {
        let make_candidate = |id: &str| DiscreteDesignCandidate {
            action_id: id.into(),
            cost_units: 1,
            eligible: true,
            outcome_ids: vec!["a".into(), "b".into()],
            likelihoods: vec![vec![0.8, 0.2], vec![0.2, 0.8]],
        };
        let candidates = vec![make_candidate("z_action"), make_candidate("a_action")];
        let selected = select_discrete_design(
            &["m0".into(), "m1".into()],
            &[0.5, 0.5],
            &candidates,
            1,
            DesignInformationObjective::SibsonAlpha(0.5),
        ).unwrap().unwrap();
        assert_eq!(selected.action_id, "a_action");
    }

    #[test]
    fn robust_selector_rejects_duplicate_action_ids_and_invalid_objectives() {
        let candidate = robust_ranking_fixture().remove(0);
        assert!(select_discrete_design(
            &["m0".into(), "m1".into()],
            &[0.5, 0.5],
            &[candidate.clone(), candidate],
            2,
            DesignInformationObjective::SibsonAlpha(0.5),
        ).is_err());
        assert!(select_discrete_design(
            &["m0".into(), "m1".into()],
            &[0.5, 0.5],
            &[robust_ranking_fixture().remove(0)],
            2,
            DesignInformationObjective::SibsonAlpha(1.0),
        ).is_err());
    }

    #[test]
    fn independent_observation_has_zero_shannon_and_sibson_information() {
        let design = independent_design();
        assert!(design.shannon_mutual_information().unwrap().abs() < 1e-12);
        for alpha in [0.2, 0.5, 0.8, 0.99] {
            assert!(design.sibson_alpha_mutual_information(alpha).unwrap().abs() < 1e-12);
        }
    }

    #[test]
    fn sibson_information_converges_to_shannon_as_alpha_approaches_one() {
        let design = complementary_ct_tracer_pair();
        let shannon = design.shannon_mutual_information().unwrap();
        let sibson = design.sibson_alpha_mutual_information(0.999).unwrap();
        assert!((sibson - shannon).abs() < 2e-3);
    }

    #[test]
    fn sibson_information_is_monotone_over_preregistered_alpha_grid() {
        let design = complementary_ct_tracer_pair();
        let values: Vec<f64> = [0.2, 0.4, 0.6, 0.8, 0.95]
            .iter()
            .map(|alpha| design.sibson_alpha_mutual_information(*alpha).unwrap())
            .collect();
        assert!(values.windows(2).all(|window| window[0] <= window[1] + 1e-12));
    }

    #[test]
    fn correlated_ct_tracer_pair_has_information_when_each_marginal_does_not() {
        let pair = complementary_ct_tracer_pair();
        let joint_shannon = pair.shannon_mutual_information().unwrap();
        let joint_sibson = pair.sibson_alpha_mutual_information(0.7).unwrap();

        let ct_only = DiscreteDesignModel {
            model_ids: pair.model_ids.clone(),
            prior_weights: pair.prior_weights.clone(),
            outcome_ids: vec!["ct_minus".into(), "ct_plus".into()],
            likelihoods: vec![vec![0.5, 0.5], vec![0.5, 0.5]],
        };
        let tracer_only = ct_only.clone();
        assert!(ct_only.shannon_mutual_information().unwrap().abs() < 1e-12);
        assert!(tracer_only.sibson_alpha_mutual_information(0.7).unwrap().abs() < 1e-12);
        assert!(joint_shannon > 0.3);
        assert!(joint_sibson > 0.2);
        assert!(joint_sibson < joint_shannon);
    }

    #[test]
    fn alpha_tilted_posterior_tempers_evidence() {
        let design = complementary_ct_tracer_pair();
        let nominal = design.alpha_tilted_posterior(0, 1.0).unwrap();
        let robust = design.alpha_tilted_posterior(0, 0.5).unwrap();
        assert!((nominal[0] - 0.9).abs() < 1e-12);
        assert!((robust[0] - 0.75).abs() < 1e-12);
        assert!(robust[0] < nominal[0]);
    }

    #[test]
    fn risk_sensitive_renyi_identity_matches_direct_sibson_information() {
        let design = complementary_ct_tracer_pair();
        for alpha in [0.1, 0.2, 0.4, 0.6, 0.7, 0.8, 0.95, 0.999] {
            let direct = design.sibson_alpha_mutual_information(alpha).unwrap();
            let conditional = design.sibson_from_conditional_renyi(alpha).unwrap();
            assert!(
                (direct - conditional).abs() < 1e-10,
                "alpha={alpha}: direct={direct}, conditional={conditional}"
            );
        }
    }

    #[test]
    fn uninformative_outcomes_have_zero_conditional_renyi_gain() {
        let design = independent_design();
        for outcome in 0..design.outcome_ids.len() {
            assert!(design.conditional_renyi_information(outcome, 0.5).unwrap().abs() < 1e-12);
        }
        assert!(design.sibson_from_conditional_renyi(0.5).unwrap().abs() < 1e-12);
    }

    #[test]
    fn conditional_renyi_gain_handles_zero_likelihood_support() {
        let design = complementary_ct_tracer_pair();
        let gain = design.conditional_renyi_information(0, 0.5).unwrap();
        assert!(gain.is_finite() && gain > 0.0);
        assert!(design.conditional_renyi_information(999, 0.5).is_err());
        assert!(design.conditional_renyi_information(0, 1.0).is_err());
    }

    #[test]
    fn worst_case_joint_has_alpha_tilted_conditional_posterior() {
        let design = complementary_ct_tracer_pair();
        let alpha = 0.5;
        let worst_case = design.worst_case_joint_distribution(alpha).unwrap();
        let tilted = design.alpha_tilted_posterior(0, alpha).unwrap();
        let conditional: Vec<f64> = worst_case.joint_weights.iter()
            .map(|row| row[0] / worst_case.outcome_marginal[0])
            .collect();
        assert!((conditional[0] - tilted[0]).abs() < 1e-12);
        assert!((conditional[1] - tilted[1]).abs() < 1e-12);
        assert!((worst_case.joint_weights.iter().flatten().sum::<f64>() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn worst_case_kl_cost_tends_to_zero_at_nominal_alpha_limit() {
        let design = complementary_ct_tracer_pair();
        let at_half = design.worst_case_joint_distribution(0.5).unwrap();
        let near_one = design.worst_case_joint_distribution(0.999).unwrap();
        assert!(at_half.kl_to_nominal_nats > 0.05);
        assert!(near_one.kl_to_nominal_nats < at_half.kl_to_nominal_nats);
        assert!(near_one.kl_to_nominal_nats < 1e-5);
    }

    #[test]
    fn alpha_inverts_the_finite_single_design_kl_ambiguity_radius() {
        let design = complementary_ct_tracer_pair();
        let radius = design.worst_case_joint_distribution(0.5).unwrap().kl_to_nominal_nats;
        let recovered = design.alpha_for_kl_radius(radius, 0.05, 64).unwrap();
        assert!((recovered - 0.5).abs() < 1e-8);
        assert_eq!(design.alpha_for_kl_radius(0.0, 0.05, 64).unwrap(), 1.0);
        assert!(design.alpha_for_kl_radius(radius + 1.0, 0.05, 64).is_err());
    }

    #[test]
    fn independent_design_has_zero_adversarial_kl_cost() {
        let design = independent_design();
        let worst_case = design.worst_case_joint_distribution(0.5).unwrap();
        assert!(worst_case.kl_to_nominal_nats.abs() < 1e-12);
        assert!(design.alpha_for_kl_radius(0.01, 0.05, 32).is_err());
    }

    #[test]
    fn rejects_invalid_alpha_probabilities_and_outcome_indices() {
        let design = complementary_ct_tracer_pair();
        for alpha in [0.0, 1.0, -0.2, f64::NAN, f64::INFINITY] {
            assert!(design.sibson_alpha_mutual_information(alpha).is_err());
        }
        assert!(design.alpha_tilted_posterior(0, 0.0).is_err());
        assert!(design.alpha_tilted_posterior(0, 1.1).is_err());
        assert!(design.alpha_tilted_posterior(99, 0.5).is_err());

        let invalid = DiscreteDesignModel {
            model_ids: vec!["a".into()],
            prior_weights: vec![1.0],
            outcome_ids: vec!["x".into(), "y".into()],
            likelihoods: vec![vec![0.2, 0.2]],
        };
        assert!(invalid.shannon_mutual_information().is_err());
    }
}

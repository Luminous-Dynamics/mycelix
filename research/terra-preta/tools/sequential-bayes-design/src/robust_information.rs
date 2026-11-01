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
        let outcome_count = self.outcome_ids.len();
        let mut information = 0.0;

        for outcome in 0..outcome_count {
            let marginal: f64 = self.likelihoods.iter()
                .zip(&prior)
                .map(|(row, weight)| weight * row[outcome])
                .sum();
            if marginal == 0.0 {
                continue;
            }
            for (model, row) in self.likelihoods.iter().enumerate() {
                let joint = prior[model] * row[outcome];
                if joint > 0.0 {
                    information += joint * (row[outcome] / marginal).ln();
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
        let mut outer_sum = 0.0;

        for outcome in 0..self.outcome_ids.len() {
            let inner_sum: f64 = self.likelihoods.iter()
                .zip(&prior)
                .map(|(row, weight)| weight * row[outcome].powf(alpha))
                .sum();
            outer_sum += inner_sum.powf(1.0 / alpha);
        }
        if !outer_sum.is_finite() || outer_sum <= 0.0 {
            return Err("Sibson outer sum must be finite and positive".into());
        }

        let information = alpha / (alpha - 1.0) * outer_sum.ln();
        if !information.is_finite() {
            return Err("Sibson mutual information is not finite".into());
        }
        Ok(information.max(0.0))
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

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Proof of Gradient Quality (PoGQ)
//!
//! Byzantine-resistant federated learning with up to 45% adversary tolerance.
//! This module provides the core PoGQ algorithm for validating gradient updates
//! in federated learning scenarios.

use crate::{Error, Result};
use serde::{Deserialize, Serialize};

/// Configuration for PoGQ validator
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoGQConfig {
    /// Byzantine fault tolerance threshold (default: 0.45 = 45%)
    pub bft_threshold: f64,

    /// Minimum gradient quality score (0.0 - 1.0)
    pub min_quality_score: f64,

    /// Number of validation rounds
    pub validation_rounds: usize,

    /// Enable adaptive threshold adjustment
    pub adaptive_threshold: bool,
}

impl Default for PoGQConfig {
    fn default() -> Self {
        Self {
            bft_threshold: crate::DEFAULT_BFT_THRESHOLD,
            min_quality_score: 0.7,
            validation_rounds: 3,
            adaptive_threshold: true,
        }
    }
}

/// Gradient update from a federated learning participant
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GradientUpdate {
    /// Participant identifier
    pub participant_id: String,

    /// Gradient data (serialized weights)
    pub gradients: Vec<f64>,

    /// Timestamp of the update
    pub timestamp: chrono::DateTime<chrono::Utc>,

    /// Proof of computation (zk-STARK or similar)
    pub proof: Option<Vec<u8>>,
}

/// Quality score for a gradient update
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityScore {
    /// Overall quality score (0.0 - 1.0)
    pub score: f64,

    /// Consistency with other gradients
    pub consistency: f64,

    /// Magnitude check (detects extreme outliers)
    pub magnitude_check: f64,

    /// Direction alignment with consensus
    pub direction_alignment: f64,

    /// Is this gradient considered valid?
    pub is_valid: bool,
}

fn l2_norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

fn cosine_similarity(a: &[f64], b: &[f64]) -> f64 {
    let dot: f64 = a.iter().zip(b).map(|(x, y)| x * y).sum();
    let norm_a = l2_norm(a);
    let norm_b = l2_norm(b);
    if norm_a == 0.0 || norm_b == 0.0 {
        return 0.0;
    }
    (dot / (norm_a * norm_b)).clamp(-1.0, 1.0)
}

fn coordinate_wise_median(gradients: &[&[f64]]) -> Vec<f64> {
    let dim = gradients[0].len();
    (0..dim)
        .map(|i| {
            let mut vals: Vec<f64> = gradients.iter().map(|g| g[i]).collect();
            vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let mid = vals.len() / 2;
            if vals.len() % 2 == 0 {
                (vals[mid - 1] + vals[mid]) / 2.0
            } else {
                vals[mid]
            }
        })
        .collect()
}

fn coordinate_wise_mean(gradients: &[&[f64]]) -> Vec<f64> {
    let dim = gradients[0].len();
    let n = gradients.len() as f64;
    (0..dim)
        .map(|i| gradients.iter().map(|g| g[i]).sum::<f64>() / n)
        .collect()
}

/// PoGQ Validator
pub struct PoGQValidator {
    config: PoGQConfig,
}

impl PoGQValidator {
    /// Create a new PoGQ validator
    pub fn new(config: PoGQConfig) -> Self {
        Self { config }
    }

    /// Validate a batch of gradient updates
    ///
    /// Returns quality scores for each gradient and identifies Byzantine actors.
    ///
    /// Scores each gradient against the batch's coordinate-wise median (a
    /// standard Byzantine-robust reference point, per the module's BFT
    /// design goal) and mean, plus a magnitude-outlier check against the
    /// batch's norm distribution. Does NOT verify `proof` cryptographically
    /// -- zk-STARK verification is a separate concern from statistical
    /// gradient-quality scoring and isn't implemented here; a gradient with
    /// no proof is scored purely on its statistical properties, same as one
    /// with an (unverified) proof.
    pub fn validate_gradients(&self, gradients: &[GradientUpdate]) -> Result<Vec<QualityScore>> {
        if gradients.is_empty() {
            return Err(Error::PoGQ("No gradients to validate".to_string()));
        }

        let dim = gradients[0].gradients.len();
        if dim == 0 || gradients.iter().any(|g| g.gradients.len() != dim) {
            return Err(Error::PoGQ(
                "Gradients must be non-empty and share the same dimensionality".to_string(),
            ));
        }

        let vectors: Vec<&[f64]> = gradients.iter().map(|g| g.gradients.as_slice()).collect();
        let median = coordinate_wise_median(&vectors);
        let mean = coordinate_wise_mean(&vectors);

        let norms: Vec<f64> = vectors.iter().map(|v| l2_norm(v)).collect();
        let mean_norm = norms.iter().sum::<f64>() / norms.len() as f64;
        let norm_variance =
            norms.iter().map(|n| (n - mean_norm).powi(2)).sum::<f64>() / norms.len() as f64;
        let norm_std = norm_variance.sqrt().max(1e-9);

        let scores = gradients
            .iter()
            .zip(norms.iter())
            .map(|(grad, &norm)| {
                // Consistency: agreement with the robust (median) consensus.
                let consistency = cosine_similarity(&grad.gradients, &median).max(0.0);

                // Direction alignment: agreement with the naive average --
                // deliberately a different reference point than consistency
                // so the two can diverge for a gradient that pulls the mean
                // toward it but disagrees with the robust median (the
                // signature of a coordinated Byzantine minority).
                let direction_alignment = cosine_similarity(&grad.gradients, &mean).max(0.0);

                // Magnitude check: how many std devs is this gradient's norm
                // from the batch mean norm (clamped at 3 std devs to 0).
                let z_score = ((norm - mean_norm) / norm_std).abs();
                let magnitude_check = (1.0 - (z_score / 3.0)).clamp(0.0, 1.0);

                let score = 0.5 * consistency + 0.3 * magnitude_check + 0.2 * direction_alignment;
                let is_valid = score >= self.config.min_quality_score
                    && grad.gradients.iter().all(|v| v.is_finite());

                QualityScore {
                    score,
                    consistency,
                    magnitude_check,
                    direction_alignment,
                    is_valid,
                }
            })
            .collect();

        Ok(scores)
    }

    /// Aggregate valid gradients into a consensus update
    pub fn aggregate_gradients(
        &self,
        gradients: &[GradientUpdate],
        scores: &[QualityScore],
    ) -> Result<Vec<f64>> {
        if gradients.len() != scores.len() {
            return Err(Error::PoGQ("Mismatched gradients and scores".to_string()));
        }

        // Filter valid gradients
        let valid_gradients: Vec<&GradientUpdate> = gradients
            .iter()
            .zip(scores.iter())
            .filter(|(_, score)| score.is_valid)
            .map(|(grad, _)| grad)
            .collect();

        if valid_gradients.is_empty() {
            return Err(Error::PoGQ("No valid gradients to aggregate".to_string()));
        }

        // TODO: Implement weighted aggregation based on quality scores
        // For now, simple averaging
        let gradient_size = valid_gradients[0].gradients.len();
        let mut aggregated = vec![0.0; gradient_size];

        for grad in valid_gradients.iter() {
            for (i, &val) in grad.gradients.iter().enumerate() {
                aggregated[i] += val;
            }
        }

        let count = valid_gradients.len() as f64;
        aggregated.iter_mut().for_each(|v| *v /= count);

        Ok(aggregated)
    }

    /// Detect Byzantine actors based on quality scores
    pub fn detect_byzantine(
        &self,
        gradients: &[GradientUpdate],
        scores: &[QualityScore],
    ) -> Vec<String> {
        gradients
            .iter()
            .zip(scores.iter())
            .filter(|(_, score)| !score.is_valid || score.score < self.config.min_quality_score)
            .map(|(grad, _)| grad.participant_id.clone())
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pogq_validator_creation() {
        let config = PoGQConfig::default();
        let validator = PoGQValidator::new(config);
        assert_eq!(validator.config.bft_threshold, 0.45);
    }

    #[test]
    fn test_validate_empty_gradients() {
        let validator = PoGQValidator::new(PoGQConfig::default());
        let result = validator.validate_gradients(&[]);
        assert!(result.is_err());
    }

    fn make_gradient(id: &str, values: Vec<f64>) -> GradientUpdate {
        GradientUpdate {
            participant_id: id.to_string(),
            gradients: values,
            timestamp: chrono::Utc::now(),
            proof: None,
        }
    }

    #[test]
    fn test_validate_rejects_mismatched_dimensions() {
        let validator = PoGQValidator::new(PoGQConfig::default());
        let gradients = vec![
            make_gradient("a", vec![1.0, 2.0, 3.0]),
            make_gradient("b", vec![1.0, 2.0]),
        ];
        assert!(validator.validate_gradients(&gradients).is_err());
    }

    #[test]
    fn test_validate_scores_outlier_lower_than_consensus() {
        let validator = PoGQValidator::new(PoGQConfig::default());
        // Five participants agree closely; one submits a wildly different,
        // much larger-magnitude gradient (a Byzantine/faulty outlier).
        let gradients = vec![
            make_gradient("consensus_1", vec![1.0, 1.0, 1.0]),
            make_gradient("consensus_2", vec![1.05, 0.95, 1.0]),
            make_gradient("consensus_3", vec![0.95, 1.05, 1.0]),
            make_gradient("consensus_4", vec![1.0, 1.0, 1.05]),
            make_gradient("consensus_5", vec![0.98, 1.02, 0.99]),
            make_gradient("outlier", vec![-50.0, 80.0, -30.0]),
        ];
        let scores = validator.validate_gradients(&gradients).unwrap();
        let consensus_min = scores[0..5]
            .iter()
            .map(|s| s.score)
            .fold(f64::MAX, f64::min);
        let outlier_score = scores[5].score;
        assert!(
            outlier_score < consensus_min,
            "outlier score {outlier_score} should be lower than the lowest consensus score {consensus_min}"
        );
        assert!(scores[0..5].iter().all(|s| s.is_valid));
        assert!(!scores[5].is_valid);
    }

    #[test]
    fn test_detect_byzantine_flags_the_outlier() {
        let validator = PoGQValidator::new(PoGQConfig::default());
        let gradients = vec![
            make_gradient("consensus_1", vec![1.0, 1.0, 1.0]),
            make_gradient("consensus_2", vec![1.05, 0.95, 1.0]),
            make_gradient("consensus_3", vec![0.95, 1.05, 1.0]),
            make_gradient("outlier", vec![-50.0, 80.0, -30.0]),
        ];
        let scores = validator.validate_gradients(&gradients).unwrap();
        let byzantine = validator.detect_byzantine(&gradients, &scores);
        assert_eq!(byzantine, vec!["outlier".to_string()]);
    }
}

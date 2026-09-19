// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Canonical aggregation admission boundary.
//!
//! This module validates structural input invariants before an aggregation
//! algorithm/profile is selected. It intentionally does not define Byzantine
//! robustness, privacy, secure aggregation, model-promotion authority, or
//! scientific validity.

use std::collections::HashSet;

use thiserror::Error;

use crate::types::GradientUpdate;

/// Structural failures that prevent a set of updates from entering the
/// canonical aggregation path.
#[derive(Debug, Error)]
pub enum AggregationAdmissionError {
    #[error("no gradient updates provided")]
    NoUpdates,

    #[error("participant identifier is empty at update index {index}")]
    EmptyParticipantId { index: usize },

    #[error("duplicate participant contribution: {participant_id}")]
    DuplicateParticipant { participant_id: String },

    #[error("gradient array is empty for participant {participant_id}")]
    EmptyGradients { participant_id: String },

    #[error(
        "gradient size mismatch for participant {participant_id}: expected {expected}, got {actual}"
    )]
    GradientSizeMismatch {
        participant_id: String,
        expected: usize,
        actual: usize,
    },

    #[error(
        "model version mismatch for participant {participant_id}: expected {expected}, got {actual}"
    )]
    ModelVersionMismatch {
        participant_id: String,
        expected: u64,
        actual: u64,
    },

    #[error("non-finite gradient for participant {participant_id} at coordinate {coordinate}")]
    NonFiniteGradient {
        participant_id: String,
        coordinate: usize,
    },

    #[error("invalid zero batch size for participant {participant_id}")]
    InvalidBatchSize { participant_id: String },

    #[error("non-finite loss for participant {participant_id}")]
    NonFiniteLoss { participant_id: String },

    #[error("non-finite accuracy for participant {participant_id}")]
    NonFiniteAccuracy { participant_id: String },
}

/// A batch that has passed the structural invariants required by the canonical
/// aggregation path.
///
/// Construction is the admission gate: downstream canonical algorithm profiles
/// should accept this type rather than an arbitrary `&[GradientUpdate]`.
#[derive(Debug, Clone, Copy)]
pub struct ValidatedAggregationBatchV1<'a> {
    updates: &'a [GradientUpdate],
    model_version: u64,
    gradient_dimension: usize,
}

impl<'a> ValidatedAggregationBatchV1<'a> {
    /// Validate a batch without mutating or reordering caller-owned updates.
    pub fn new(updates: &'a [GradientUpdate]) -> Result<Self, AggregationAdmissionError> {
        let first = updates.first().ok_or(AggregationAdmissionError::NoUpdates)?;
        let model_version = first.model_version;
        let gradient_dimension = first.gradients.len();

        if gradient_dimension == 0 {
            return Err(AggregationAdmissionError::EmptyGradients {
                participant_id: first.participant_id.clone(),
            });
        }

        let mut participants = HashSet::with_capacity(updates.len());

        for (index, update) in updates.iter().enumerate() {
            if update.participant_id.is_empty() {
                return Err(AggregationAdmissionError::EmptyParticipantId { index });
            }

            if !participants.insert(update.participant_id.as_str()) {
                return Err(AggregationAdmissionError::DuplicateParticipant {
                    participant_id: update.participant_id.clone(),
                });
            }

            if update.model_version != model_version {
                return Err(AggregationAdmissionError::ModelVersionMismatch {
                    participant_id: update.participant_id.clone(),
                    expected: model_version,
                    actual: update.model_version,
                });
            }

            if update.gradients.is_empty() {
                return Err(AggregationAdmissionError::EmptyGradients {
                    participant_id: update.participant_id.clone(),
                });
            }

            if update.gradients.len() != gradient_dimension {
                return Err(AggregationAdmissionError::GradientSizeMismatch {
                    participant_id: update.participant_id.clone(),
                    expected: gradient_dimension,
                    actual: update.gradients.len(),
                });
            }

            if let Some((coordinate, _)) = update
                .gradients
                .iter()
                .enumerate()
                .find(|(_, value)| !value.is_finite())
            {
                return Err(AggregationAdmissionError::NonFiniteGradient {
                    participant_id: update.participant_id.clone(),
                    coordinate,
                });
            }

            if update.metadata.batch_size == 0 {
                return Err(AggregationAdmissionError::InvalidBatchSize {
                    participant_id: update.participant_id.clone(),
                });
            }

            if !update.metadata.loss.is_finite() {
                return Err(AggregationAdmissionError::NonFiniteLoss {
                    participant_id: update.participant_id.clone(),
                });
            }

            if update
                .metadata
                .accuracy
                .is_some_and(|accuracy| !accuracy.is_finite())
            {
                return Err(AggregationAdmissionError::NonFiniteAccuracy {
                    participant_id: update.participant_id.clone(),
                });
            }
        }

        Ok(Self {
            updates,
            model_version,
            gradient_dimension,
        })
    }

    pub fn updates(&self) -> &'a [GradientUpdate] {
        self.updates
    }

    pub fn model_version(&self) -> u64 {
        self.model_version
    }

    pub fn gradient_dimension(&self) -> usize {
        self.gradient_dimension
    }

    pub fn participant_count(&self) -> usize {
        self.updates.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_updates() -> Vec<GradientUpdate> {
        vec![
            GradientUpdate::new("p1".into(), 7, vec![0.1, 0.2], 10, 0.5),
            GradientUpdate::new("p2".into(), 7, vec![0.3, 0.4], 20, 0.4),
        ]
    }

    #[test]
    fn admits_structurally_valid_batch() {
        let updates = valid_updates();
        let batch = ValidatedAggregationBatchV1::new(&updates).unwrap();

        assert_eq!(batch.model_version(), 7);
        assert_eq!(batch.gradient_dimension(), 2);
        assert_eq!(batch.participant_count(), 2);
        assert_eq!(batch.updates(), updates.as_slice());
    }

    #[test]
    fn rejects_non_finite_gradient() {
        for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut updates = valid_updates();
            updates[1].gradients[0] = value;
            assert!(matches!(
                ValidatedAggregationBatchV1::new(&updates),
                Err(AggregationAdmissionError::NonFiniteGradient { .. })
            ));
        }
    }

    #[test]
    fn rejects_mixed_model_versions() {
        let mut updates = valid_updates();
        updates[1].model_version = 8;
        assert!(matches!(
            ValidatedAggregationBatchV1::new(&updates),
            Err(AggregationAdmissionError::ModelVersionMismatch {
                expected: 7,
                actual: 8,
                ..
            })
        ));
    }

    #[test]
    fn rejects_duplicate_participant() {
        let mut updates = valid_updates();
        updates[1].participant_id = updates[0].participant_id.clone();
        assert!(matches!(
            ValidatedAggregationBatchV1::new(&updates),
            Err(AggregationAdmissionError::DuplicateParticipant { .. })
        ));
    }

    #[test]
    fn rejects_invalid_metadata() {
        let mut zero_batch = valid_updates();
        zero_batch[0].metadata.batch_size = 0;
        assert!(matches!(
            ValidatedAggregationBatchV1::new(&zero_batch),
            Err(AggregationAdmissionError::InvalidBatchSize { .. })
        ));

        let mut non_finite_loss = valid_updates();
        non_finite_loss[0].metadata.loss = f32::NAN;
        assert!(matches!(
            ValidatedAggregationBatchV1::new(&non_finite_loss),
            Err(AggregationAdmissionError::NonFiniteLoss { .. })
        ));

        let mut non_finite_accuracy = valid_updates();
        non_finite_accuracy[0].metadata.accuracy = Some(f32::INFINITY);
        assert!(matches!(
            ValidatedAggregationBatchV1::new(&non_finite_accuracy),
            Err(AggregationAdmissionError::NonFiniteAccuracy { .. })
        ));
    }

    #[test]
    fn rejects_empty_or_mismatched_shapes() {
        let mut empty = valid_updates();
        empty[0].gradients.clear();
        assert!(matches!(
            ValidatedAggregationBatchV1::new(&empty),
            Err(AggregationAdmissionError::EmptyGradients { .. })
        ));

        let mut mismatched = valid_updates();
        mismatched[1].gradients.push(0.5);
        assert!(matches!(
            ValidatedAggregationBatchV1::new(&mismatched),
            Err(AggregationAdmissionError::GradientSizeMismatch { .. })
        ));
    }
}

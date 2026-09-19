// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Theorem-bound Krum v1 selection profile.
//!
//! This module is intentionally separate from the historical `aggregation::krum`
//! compatibility function. It operates only on a structurally admitted batch and
//! binds the Byzantine tolerance parameter `f`, population feasibility relation,
//! squared-distance metric, neighbor count, and deterministic tie-breaking.

use thiserror::Error;

use crate::admission::ValidatedAggregationBatchV1;

/// Exact algorithm parameters for canonical single-selection Krum v1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KrumProfileV1 {
    max_byzantine: usize,
}

impl KrumProfileV1 {
    pub const fn new(max_byzantine: usize) -> Self {
        Self { max_byzantine }
    }

    pub const fn max_byzantine(&self) -> usize {
        self.max_byzantine
    }
}

#[derive(Debug, Error)]
pub enum KrumV1Error {
    #[error(
        "Krum v1 population is infeasible: n={participants}, f={max_byzantine}; require 2f + 2 < n"
    )]
    InfeasiblePopulation {
        participants: usize,
        max_byzantine: usize,
    },

    #[error("Krum v1 population bound overflow for f={max_byzantine}")]
    PopulationBoundOverflow { max_byzantine: usize },

    #[error(
        "non-finite squared distance between participants {left_participant} and {right_participant}"
    )]
    NonFiniteDistance {
        left_participant: String,
        right_participant: String,
    },

    #[error("non-finite Krum score for participant {participant_id}")]
    NonFiniteScore { participant_id: String },
}

/// Evidence-bearing output of the Krum selector before any model update is
/// applied. This is a selection result, not deployment or promotion authority.
#[derive(Debug, Clone, PartialEq)]
pub struct KrumSelectionV1 {
    participant_id: String,
    update_index: usize,
    score: f64,
    neighbor_count: usize,
    max_byzantine: usize,
}

impl KrumSelectionV1 {
    pub fn participant_id(&self) -> &str {
        &self.participant_id
    }

    pub const fn update_index(&self) -> usize {
        self.update_index
    }

    pub const fn score(&self) -> f64 {
        self.score
    }

    pub const fn neighbor_count(&self) -> usize {
        self.neighbor_count
    }

    pub const fn max_byzantine(&self) -> usize {
        self.max_byzantine
    }
}

/// Select exactly one update according to Krum v1.
///
/// Profile semantics:
///
/// - require `2f + 2 < n`;
/// - for each candidate, compute squared Euclidean distance to every other
///   admitted update using f64 accumulation over f32 coordinates;
/// - score the candidate by summing the `n - f - 2` smallest distances;
/// - select the lowest score;
/// - break exact score ties by participant identifier, then stable input index.
pub fn krum_select_v1(
    batch: &ValidatedAggregationBatchV1<'_>,
    profile: KrumProfileV1,
) -> Result<KrumSelectionV1, KrumV1Error> {
    let n = batch.participant_count();
    let f = profile.max_byzantine();
    let minimum_population = f
        .checked_mul(2)
        .and_then(|value| value.checked_add(3))
        .ok_or(KrumV1Error::PopulationBoundOverflow { max_byzantine: f })?;

    if n < minimum_population {
        return Err(KrumV1Error::InfeasiblePopulation {
            participants: n,
            max_byzantine: f,
        });
    }

    let neighbor_count = n - f - 2;
    let updates = batch.updates();
    let mut best: Option<(f64, &str, usize)> = None;

    for (candidate_index, candidate) in updates.iter().enumerate() {
        let mut distances = Vec::with_capacity(n - 1);

        for (other_index, other) in updates.iter().enumerate() {
            if candidate_index == other_index {
                continue;
            }

            let distance = squared_euclidean_distance_f64(
                &candidate.gradients,
                &other.gradients,
            )
            .ok_or_else(|| KrumV1Error::NonFiniteDistance {
                left_participant: candidate.participant_id.clone(),
                right_participant: other.participant_id.clone(),
            })?;

            distances.push(distance);
        }

        distances.sort_unstable_by(f64::total_cmp);
        let score = distances.iter().take(neighbor_count).copied().sum::<f64>();
        if !score.is_finite() {
            return Err(KrumV1Error::NonFiniteScore {
                participant_id: candidate.participant_id.clone(),
            });
        }

        let candidate_key = (score, candidate.participant_id.as_str(), candidate_index);
        let replace = match best {
            None => true,
            Some((best_score, best_id, best_index)) => {
                score.total_cmp(&best_score).is_lt()
                    || (score.total_cmp(&best_score).is_eq()
                        && (candidate.participant_id.as_str() < best_id
                            || (candidate.participant_id.as_str() == best_id
                                && candidate_index < best_index)))
            }
        };

        if replace {
            best = Some(candidate_key);
        }
    }

    let (score, participant_id, update_index) =
        best.expect("validated feasible Krum population must contain candidates");

    Ok(KrumSelectionV1 {
        participant_id: participant_id.to_owned(),
        update_index,
        score,
        neighbor_count,
        max_byzantine: f,
    })
}

fn squared_euclidean_distance_f64(left: &[f32], right: &[f32]) -> Option<f64> {
    debug_assert_eq!(left.len(), right.len());

    let mut sum = 0.0_f64;
    for (&left_value, &right_value) in left.iter().zip(right.iter()) {
        let delta = f64::from(left_value) - f64::from(right_value);
        let term = delta * delta;
        if !term.is_finite() {
            return None;
        }
        sum += term;
        if !sum.is_finite() {
            return None;
        }
    }

    Some(sum)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::GradientUpdate;

    fn admitted(updates: &[GradientUpdate]) -> ValidatedAggregationBatchV1<'_> {
        ValidatedAggregationBatchV1::new(updates).unwrap()
    }

    #[test]
    fn rejects_infeasible_population() {
        let updates = vec![
            GradientUpdate::new("p1".into(), 1, vec![0.0], 1, 0.1),
            GradientUpdate::new("p2".into(), 1, vec![0.1], 1, 0.1),
            GradientUpdate::new("p3".into(), 1, vec![0.2], 1, 0.1),
            GradientUpdate::new("p4".into(), 1, vec![0.3], 1, 0.1),
        ];
        let batch = admitted(&updates);

        assert!(matches!(
            krum_select_v1(&batch, KrumProfileV1::new(1)),
            Err(KrumV1Error::InfeasiblePopulation {
                participants: 4,
                max_byzantine: 1,
            })
        ));
    }

    #[test]
    fn rejects_clear_byzantine_outlier() {
        let updates = vec![
            GradientUpdate::new("p1".into(), 1, vec![0.0, 0.0], 1, 0.1),
            GradientUpdate::new("p2".into(), 1, vec![0.1, 0.0], 1, 0.1),
            GradientUpdate::new("p3".into(), 1, vec![0.0, 0.1], 1, 0.1),
            GradientUpdate::new("p4".into(), 1, vec![0.1, 0.1], 1, 0.1),
            GradientUpdate::new("byz".into(), 1, vec![100.0, 100.0], 1, 0.1),
        ];
        let batch = admitted(&updates);
        let selection = krum_select_v1(&batch, KrumProfileV1::new(1)).unwrap();

        assert_ne!(selection.participant_id(), "byz");
        assert_eq!(selection.neighbor_count(), 2);
        assert_eq!(selection.max_byzantine(), 1);
    }

    #[test]
    fn exact_score_tie_break_is_input_order_independent() {
        let first_order = vec![
            GradientUpdate::new("p-c".into(), 1, vec![2.0], 1, 0.1),
            GradientUpdate::new("p-a".into(), 1, vec![0.0], 1, 0.1),
            GradientUpdate::new("p-b".into(), 1, vec![1.0], 1, 0.1),
        ];
        let second_order = vec![
            GradientUpdate::new("p-b".into(), 1, vec![1.0], 1, 0.1),
            GradientUpdate::new("p-c".into(), 1, vec![2.0], 1, 0.1),
            GradientUpdate::new("p-a".into(), 1, vec![0.0], 1, 0.1),
        ];

        let first = krum_select_v1(&admitted(&first_order), KrumProfileV1::new(0)).unwrap();
        let second = krum_select_v1(&admitted(&second_order), KrumProfileV1::new(0)).unwrap();

        assert_eq!(first.participant_id(), "p-a");
        assert_eq!(second.participant_id(), "p-a");
        assert!((first.score() - 1.0).abs() < f64::EPSILON);
        assert!((second.score() - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn uses_n_minus_f_minus_two_neighbors() {
        let updates = vec![
            GradientUpdate::new("p1".into(), 1, vec![0.0], 1, 0.1),
            GradientUpdate::new("p2".into(), 1, vec![0.1], 1, 0.1),
            GradientUpdate::new("p3".into(), 1, vec![0.2], 1, 0.1),
            GradientUpdate::new("p4".into(), 1, vec![0.3], 1, 0.1),
            GradientUpdate::new("p5".into(), 1, vec![9.0], 1, 0.1),
        ];
        let batch = admitted(&updates);
        let selection = krum_select_v1(&batch, KrumProfileV1::new(1)).unwrap();

        assert_eq!(selection.neighbor_count(), 2);
    }
}

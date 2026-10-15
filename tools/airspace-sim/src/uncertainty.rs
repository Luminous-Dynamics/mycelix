//! Test-only prototype for compositional scalar affine uncertainty.
//!
//! This intentionally is not a flight-dynamics or reachable-set implementation.
//! Source IDs identify the same latent additive error only when their provenance
//! contract says so. Different IDs are conservatively ranged independently;
//! this enlarges the admitted set when cross-source dependencies are unknown.

use std::collections::BTreeMap;

const MAX_SOURCE_TERMS: usize = 64;
const MAX_COMPOSITION_INPUTS: usize = MAX_SOURCE_TERMS * 2;
const MAX_SOURCE_ID_BYTES: usize = 128;
const MAX_ABS_SOURCE_BOUND_MM: i128 = 1_000_000_000_000;
const MAX_ABS_COEFFICIENT: i128 = 1_000_000;
const MAX_ABS_COMPOSED_BOUND_MM: i128 = 1_000_000_000_000_000;

#[derive(Debug, Clone, PartialEq, Eq)]
struct SourceContribution {
    /// Stable provenance identity for one latent additive error.
    source_id: String,
    /// Inclusive bounds of the latent error, in synthetic millimetres.
    lower_mm: i64,
    upper_mm: i64,
    /// Exact integer sensitivity of this output to the named latent error.
    coefficient: i64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct AffineUncertainty {
    /// Canonically sorted and unique by source_id after validation.
    terms: Vec<SourceContribution>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct IntervalBounds {
    lower_mm: i64,
    upper_mm: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum UncertaintyError {
    EmptyModel,
    TooManyTerms,
    EmptySourceId,
    SourceIdTooLong,
    InvalidSourceBounds,
    SourceBoundsMismatch,
    CoefficientOutOfRange,
    ArithmeticOverflow,
    OutputBoundsTooWide,
}

impl AffineUncertainty {
    fn new(terms: Vec<SourceContribution>) -> Result<Self, UncertaintyError> {
        Self::normalize(terms, MAX_SOURCE_TERMS)
    }

    /// Adds two affine error expressions. Contributions from the same stable
    /// source are combined algebraically, preserving a declared shared cause.
    /// Different source IDs are never inferred to be correlated or identical.
    fn compose(&self, other: &Self) -> Result<Self, UncertaintyError> {
        let mut terms = Vec::with_capacity(self.terms.len() + other.terms.len());
        terms.extend(self.terms.iter().cloned());
        terms.extend(other.terms.iter().cloned());
        Self::normalize(terms, MAX_COMPOSITION_INPUTS)
    }

    /// Computes an outward-exact interval for this bounded integer-affine form.
    /// This is a set bound under the declared source ranges, not a probability
    /// or physical aircraft-state guarantee.
    fn bounds(&self) -> Result<IntervalBounds, UncertaintyError> {
        if self.terms.is_empty() {
            return Err(UncertaintyError::EmptyModel);
        }

        let mut lower_total: i128 = 0;
        let mut upper_total: i128 = 0;
        for term in &self.terms {
            let coefficient = i128::from(term.coefficient);
            let source_lower = i128::from(term.lower_mm);
            let source_upper = i128::from(term.upper_mm);

            let (term_lower, term_upper) = if coefficient >= 0 {
                (
                    coefficient
                        .checked_mul(source_lower)
                        .ok_or(UncertaintyError::ArithmeticOverflow)?,
                    coefficient
                        .checked_mul(source_upper)
                        .ok_or(UncertaintyError::ArithmeticOverflow)?,
                )
            } else {
                (
                    coefficient
                        .checked_mul(source_upper)
                        .ok_or(UncertaintyError::ArithmeticOverflow)?,
                    coefficient
                        .checked_mul(source_lower)
                        .ok_or(UncertaintyError::ArithmeticOverflow)?,
                )
            };
            lower_total = lower_total
                .checked_add(term_lower)
                .ok_or(UncertaintyError::ArithmeticOverflow)?;
            upper_total = upper_total
                .checked_add(term_upper)
                .ok_or(UncertaintyError::ArithmeticOverflow)?;
        }

        if lower_total > upper_total
            || lower_total.abs() > MAX_ABS_COMPOSED_BOUND_MM
            || upper_total.abs() > MAX_ABS_COMPOSED_BOUND_MM
        {
            return Err(UncertaintyError::OutputBoundsTooWide);
        }

        Ok(IntervalBounds {
            lower_mm: i64::try_from(lower_total)
                .map_err(|_| UncertaintyError::ArithmeticOverflow)?,
            upper_mm: i64::try_from(upper_total)
                .map_err(|_| UncertaintyError::ArithmeticOverflow)?,
        })
    }

    fn normalize(
        terms: Vec<SourceContribution>,
        max_input_terms: usize,
    ) -> Result<Self, UncertaintyError> {
        if terms.is_empty() {
            return Err(UncertaintyError::EmptyModel);
        }
        if terms.len() > max_input_terms {
            return Err(UncertaintyError::TooManyTerms);
        }

        let mut by_source: BTreeMap<String, SourceContribution> = BTreeMap::new();
        for term in terms {
            if term.source_id.trim().is_empty() {
                return Err(UncertaintyError::EmptySourceId);
            }
            if term.source_id.len() > MAX_SOURCE_ID_BYTES {
                return Err(UncertaintyError::SourceIdTooLong);
            }
            if term.lower_mm > term.upper_mm
                || i128::from(term.lower_mm).abs() > MAX_ABS_SOURCE_BOUND_MM
                || i128::from(term.upper_mm).abs() > MAX_ABS_SOURCE_BOUND_MM
            {
                return Err(UncertaintyError::InvalidSourceBounds);
            }
            if i128::from(term.coefficient).abs() > MAX_ABS_COEFFICIENT {
                return Err(UncertaintyError::CoefficientOutOfRange);
            }

            if let Some(existing) = by_source.get_mut(&term.source_id) {
                if existing.lower_mm != term.lower_mm || existing.upper_mm != term.upper_mm {
                    return Err(UncertaintyError::SourceBoundsMismatch);
                }
                let combined = existing
                    .coefficient
                    .checked_add(term.coefficient)
                    .ok_or(UncertaintyError::ArithmeticOverflow)?;
                if i128::from(combined).abs() > MAX_ABS_COEFFICIENT {
                    return Err(UncertaintyError::CoefficientOutOfRange);
                }
                existing.coefficient = combined;
            } else {
                by_source.insert(term.source_id.clone(), term);
                if by_source.len() > MAX_SOURCE_TERMS {
                    return Err(UncertaintyError::TooManyTerms);
                }
            }
        }

        Ok(Self {
            terms: by_source.into_values().collect(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contribution(
        source_id: &str,
        lower_mm: i64,
        upper_mm: i64,
        coefficient: i64,
    ) -> SourceContribution {
        SourceContribution {
            source_id: source_id.to_owned(),
            lower_mm,
            upper_mm,
            coefficient,
        }
    }

    #[test]
    fn shared_common_mode_source_can_cancel_only_via_explicit_identity_and_coefficients() {
        let forward =
            AffineUncertainty::new(vec![contribution("datum-bias", -10, 10, 1)]).unwrap();
        let inverse =
            AffineUncertainty::new(vec![contribution("datum-bias", -10, 10, -1)]).unwrap();

        let relative = forward.compose(&inverse).unwrap();
        assert_eq!(
            relative.bounds().unwrap(),
            IntervalBounds {
                lower_mm: 0,
                upper_mm: 0
            }
        );
        assert_eq!(relative.terms.len(), 1);
        assert_eq!(relative.terms[0].coefficient, 0);
    }

    #[test]
    fn different_source_ids_never_cancel_by_assumption() {
        let first =
            AffineUncertainty::new(vec![contribution("datum-source-a", -10, 10, 1)]).unwrap();
        let second =
            AffineUncertainty::new(vec![contribution("datum-source-b", -10, 10, -1)]).unwrap();

        assert_eq!(
            first.compose(&second).unwrap().bounds().unwrap(),
            IntervalBounds {
                lower_mm: -20,
                upper_mm: 20
            }
        );
    }

    #[test]
    fn distinct_source_ranges_compose_to_a_conservative_sum() {
        let observed = AffineUncertainty::new(vec![
            contribution("observation", -2, 3, 1),
            contribution("transform", -4, 1, 2),
        ])
        .unwrap();

        assert_eq!(
            observed.bounds().unwrap(),
            IntervalBounds {
                lower_mm: -10,
                upper_mm: 5
            }
        );
    }

    #[test]
    fn negative_coefficients_reverse_interval_endpoints() {
        let model =
            AffineUncertainty::new(vec![contribution("clock-error", -2, 3, -2)]).unwrap();

        assert_eq!(
            model.bounds().unwrap(),
            IntervalBounds {
                lower_mm: -6,
                upper_mm: 4
            }
        );
    }

    #[test]
    fn source_identity_with_conflicting_bounds_fails_closed() {
        let error = AffineUncertainty::new(vec![
            contribution("shared-source", -10, 10, 1),
            contribution("shared-source", -11, 10, -1),
        ])
        .unwrap_err();

        assert_eq!(error, UncertaintyError::SourceBoundsMismatch);
    }

    #[test]
    fn canonical_form_and_bounds_are_invariant_to_input_order() {
        let terms = vec![
            contribution("z-source", -5, 5, 2),
            contribution("a-source", -3, 7, -1),
            contribution("z-source", -5, 5, -1),
        ];
        let forward = AffineUncertainty::new(terms.clone()).unwrap();
        let reversed = AffineUncertainty::new(terms.into_iter().rev().collect()).unwrap();

        assert_eq!(forward, reversed);
        assert_eq!(
            forward.bounds().unwrap(),
            IntervalBounds {
                lower_mm: -12,
                upper_mm: 8
            }
        );
    }

    #[test]
    fn absent_uncertainty_model_is_not_silently_promoted_to_zero() {
        assert_eq!(
            AffineUncertainty::new(vec![]).unwrap_err(),
            UncertaintyError::EmptyModel
        );
    }

    #[test]
    fn work_and_source_domain_are_bounded() {
        let too_many = (0..=MAX_SOURCE_TERMS)
            .map(|index| contribution(&format!("source-{index}"), -1, 1, 1))
            .collect();
        assert_eq!(
            AffineUncertainty::new(too_many).unwrap_err(),
            UncertaintyError::TooManyTerms
        );

        assert_eq!(
            AffineUncertainty::new(vec![contribution(
                "oversized-source",
                -MAX_ABS_SOURCE_BOUND_MM as i64 - 1,
                1,
                1
            )])
            .unwrap_err(),
            UncertaintyError::InvalidSourceBounds
        );
    }

    #[test]
    fn coefficient_overflow_and_excessive_composed_bounds_fail_closed() {
        let positive =
            AffineUncertainty::new(vec![contribution("same", -2, 2, MAX_ABS_COEFFICIENT as i64)])
                .unwrap();
        assert_eq!(
            positive.compose(&positive).unwrap_err(),
            UncertaintyError::CoefficientOutOfRange
        );

        let wide = AffineUncertainty::new(vec![contribution(
            "wide",
            -MAX_ABS_SOURCE_BOUND_MM as i64,
            MAX_ABS_SOURCE_BOUND_MM as i64,
            MAX_ABS_COEFFICIENT as i64,
        )])
        .unwrap();
        assert_eq!(
            wide.bounds().unwrap_err(),
            UncertaintyError::OutputBoundsTooWide
        );
    }
}

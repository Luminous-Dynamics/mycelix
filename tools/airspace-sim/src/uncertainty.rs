//! Test-only prototype for compositional scalar affine uncertainty.
//!
//! This intentionally is not a flight-dynamics or reachable-set implementation.
//! Source IDs preserve observation origin; dependency IDs name a declared shared
//! latent additive error. Different origins may share a dependency and one origin
//! may contribute independent error modes. These IDs are caller assertions, not
//! authenticated provenance or correlation proofs.

use std::collections::BTreeMap;

const MAX_SOURCE_TERMS: usize = 64;
const MAX_COMPOSITION_INPUTS: usize = MAX_SOURCE_TERMS * 2;
const MAX_SOURCE_ID_BYTES: usize = 128;
const MAX_ABS_SOURCE_BOUND_MM: i128 = 1_000_000_000_000;
const MAX_ABS_COEFFICIENT: i128 = 1_000_000;
const MAX_ABS_COMPOSED_BOUND_MM: i128 = 1_000_000_000_000_000;

#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize)]
struct SourceContribution {
    /// Stable source/origin identity retained for audit; it does not imply dependence.
    source_id: String,
    /// Declared identity of one shared latent error; not authenticated by this prototype.
    dependency_id: String,
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
    NonCanonicalSourceId,
    SourceIdTooLong,
    EmptyDependencyId,
    NonCanonicalDependencyId,
    DependencyIdTooLong,
    InvalidSourceBounds,
    DependencyBoundsMismatch,
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
    /// Computes an exact interval over declared latent dependencies.
    /// Contributions sharing dependency_id vary together even when their source
    /// origins differ; that declaration is not authenticated by this prototype.
    fn bounds(&self) -> Result<IntervalBounds, UncertaintyError> {
        if self.terms.is_empty() {
            return Err(UncertaintyError::EmptyModel);
        }
        let mut groups: BTreeMap<&str, (i64, i64, i128)> = BTreeMap::new();
        for term in &self.terms {
            let group = groups.entry(term.dependency_id.as_str())
                .or_insert((term.lower_mm, term.upper_mm, 0));
            if group.0 != term.lower_mm || group.1 != term.upper_mm {
                return Err(UncertaintyError::DependencyBoundsMismatch);
            }
            group.2 = group.2.checked_add(i128::from(term.coefficient))
                .ok_or(UncertaintyError::ArithmeticOverflow)?;
        }

        let mut lower_total: i128 = 0;
        let mut upper_total: i128 = 0;
        for (source_lower, source_upper, coefficient) in groups.values().copied() {
            let (term_lower, term_upper) = if coefficient >= 0 {
                (
                    coefficient.checked_mul(i128::from(source_lower))
                        .ok_or(UncertaintyError::ArithmeticOverflow)?,
                    coefficient.checked_mul(i128::from(source_upper))
                        .ok_or(UncertaintyError::ArithmeticOverflow)?,
                )
            } else {
                (
                    coefficient.checked_mul(i128::from(source_upper))
                        .ok_or(UncertaintyError::ArithmeticOverflow)?,
                    coefficient.checked_mul(i128::from(source_lower))
                        .ok_or(UncertaintyError::ArithmeticOverflow)?,
                )
            };
            lower_total = lower_total.checked_add(term_lower)
                .ok_or(UncertaintyError::ArithmeticOverflow)?;
            upper_total = upper_total.checked_add(term_upper)
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
        if terms.is_empty() { return Err(UncertaintyError::EmptyModel); }
        if terms.len() > max_input_terms { return Err(UncertaintyError::TooManyTerms); }

        // Preserve origin/dependency pairs for audit; bounds group by dependency.
        let mut by_dependency_and_source: BTreeMap<(String, String), SourceContribution> = BTreeMap::new();
        let mut dependency_bounds: BTreeMap<String, (i64, i64)> = BTreeMap::new();
        for term in terms {
            if term.source_id.trim().is_empty() { return Err(UncertaintyError::EmptySourceId); }
            if term.source_id.as_str() != term.source_id.trim() { return Err(UncertaintyError::NonCanonicalSourceId); }
            if term.source_id.len() > MAX_SOURCE_ID_BYTES { return Err(UncertaintyError::SourceIdTooLong); }
            if term.dependency_id.trim().is_empty() { return Err(UncertaintyError::EmptyDependencyId); }
            if term.dependency_id.as_str() != term.dependency_id.trim() { return Err(UncertaintyError::NonCanonicalDependencyId); }
            if term.dependency_id.len() > MAX_SOURCE_ID_BYTES { return Err(UncertaintyError::DependencyIdTooLong); }
            if term.lower_mm > term.upper_mm
                || i128::from(term.lower_mm).abs() > MAX_ABS_SOURCE_BOUND_MM
                || i128::from(term.upper_mm).abs() > MAX_ABS_SOURCE_BOUND_MM
            {
                return Err(UncertaintyError::InvalidSourceBounds);
            }
            if i128::from(term.coefficient).abs() > MAX_ABS_COEFFICIENT {
                return Err(UncertaintyError::CoefficientOutOfRange);
            }
            match dependency_bounds.get(&term.dependency_id) {
                Some(bounds) if *bounds != (term.lower_mm, term.upper_mm) => {
                    return Err(UncertaintyError::DependencyBoundsMismatch);
                }
                Some(_) => {}
                None => { dependency_bounds.insert(term.dependency_id.clone(), (term.lower_mm, term.upper_mm)); }
            }

            let key = (term.dependency_id.clone(), term.source_id.clone());
            if let Some(existing) = by_dependency_and_source.get_mut(&key) {
                existing.coefficient = existing.coefficient.checked_add(term.coefficient)
                    .ok_or(UncertaintyError::ArithmeticOverflow)?;
            } else {
                by_dependency_and_source.insert(key, term);
                if by_dependency_and_source.len() > MAX_SOURCE_TERMS {
                    return Err(UncertaintyError::TooManyTerms);
                }
            }
        }
        // Validate the aggregate sensitivity across every origin for each latent error.
        let mut dependency_coefficients: BTreeMap<String, i128> = BTreeMap::new();
        for term in by_dependency_and_source.values() {
            let coefficient = dependency_coefficients.entry(term.dependency_id.clone()).or_insert(0);
            *coefficient = coefficient.checked_add(i128::from(term.coefficient))
                .ok_or(UncertaintyError::ArithmeticOverflow)?;
        }
        if dependency_coefficients.values().any(|c| c.abs() > MAX_ABS_COEFFICIENT) {
            return Err(UncertaintyError::CoefficientOutOfRange);
        }
        Ok(Self { terms: by_dependency_and_source.into_values().collect() })
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
        contribution_with_dependency(source_id, source_id, lower_mm, upper_mm, coefficient)
    }

    fn contribution_with_dependency(
        source_id: &str,
        dependency_id: &str,
        lower_mm: i64,
        upper_mm: i64,
        coefficient: i64,
    ) -> SourceContribution {
        SourceContribution {
            source_id: source_id.to_owned(),
            dependency_id: dependency_id.to_owned(),
            lower_mm,
            upper_mm,
            coefficient,
        }
    }

    #[derive(serde::Deserialize)]
    struct FixtureSet {
        schema: String,
        status: String,
        cases: Vec<FixtureCase>,
    }

    #[derive(serde::Deserialize)]
    struct FixtureCase {
        id: String,
        terms: Vec<SourceContribution>,
        expected_bounds_mm: Option<[i64; 2]>,
        expected_error: Option<String>,
    }

    fn error_code(error: UncertaintyError) -> &'static str {
        match error {
            UncertaintyError::EmptyModel => "empty_model",
            UncertaintyError::TooManyTerms => "too_many_terms",
            UncertaintyError::EmptySourceId => "empty_source_id",
            UncertaintyError::NonCanonicalSourceId => "non_canonical_source_id",
            UncertaintyError::SourceIdTooLong => "source_id_too_long",
            UncertaintyError::EmptyDependencyId => "empty_dependency_id",
            UncertaintyError::NonCanonicalDependencyId => "non_canonical_dependency_id",
            UncertaintyError::DependencyIdTooLong => "dependency_id_too_long",
            UncertaintyError::InvalidSourceBounds => "invalid_source_bounds",
            UncertaintyError::DependencyBoundsMismatch => "dependency_bounds_mismatch",
            UncertaintyError::CoefficientOutOfRange => "coefficient_out_of_range",
            UncertaintyError::ArithmeticOverflow => "arithmetic_overflow",
            UncertaintyError::OutputBoundsTooWide => "output_bounds_too_wide",
        }
    }

    fn enumerate_exact_bounds(
        terms: &[SourceContribution],
        index: usize,
        current: i128,
        extrema: &mut Option<(i128, i128)>,
    ) {
        if index == terms.len() {
            match extrema {
                Some((lower, upper)) => {
                    *lower = (*lower).min(current);
                    *upper = (*upper).max(current);
                }
                None => *extrema = Some((current, current)),
            }
            return;
        }

        let term = &terms[index];
        for value in term.lower_mm..=term.upper_mm {
            enumerate_exact_bounds(
                terms,
                index + 1,
                current + i128::from(term.coefficient) * i128::from(value),
                extrema,
            );
        }
    }

    fn next_deterministic(seed: &mut u64) -> u64 {
        *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        *seed
    }

    #[test]
    fn exhaustive_integer_oracle_checks_one_thousand_generated_affine_domains() {
        let mut seed = 0xA17E_2026_D15C_A11Eu64;
        for case_index in 0..1_000 {
            let count = (next_deterministic(&mut seed) % 4 + 1) as usize;
            let mut terms = Vec::with_capacity(count);
            for source_index in 0..count {
                let lower_mm = -((next_deterministic(&mut seed) % 3) as i64);
                let upper_mm = (next_deterministic(&mut seed) % 3) as i64;
                let coefficient = (next_deterministic(&mut seed) % 11) as i64 - 5;
                terms.push(contribution(
                    &format!("oracle-{case_index}-source-{source_index}"),
                    lower_mm,
                    upper_mm,
                    coefficient,
                ));
            }

            let model = AffineUncertainty::new(terms.clone())
                .unwrap_or_else(|error| panic!("generated case {case_index} rejected: {error:?}"));
            let mut extrema = None;
            enumerate_exact_bounds(&terms, 0, 0, &mut extrema);
            let (oracle_lower, oracle_upper) = extrema.expect("at least one concrete assignment");
            let actual = model.bounds().unwrap_or_else(|error| {
                panic!("generated case {case_index} failed to bound: {error:?}")
            });
            assert_eq!(
                (i128::from(actual.lower_mm), i128::from(actual.upper_mm)),
                (oracle_lower, oracle_upper),
                "interval bound disagreed with exhaustive point oracle for case {case_index}: {terms:?}"
            );

            let mut reversed = terms;
            reversed.reverse();
            let permuted = AffineUncertainty::new(reversed)
                .unwrap_or_else(|error| panic!("permuted case {case_index} rejected: {error:?}"));
            assert_eq!(
                model,
                permuted,
                "canonical uncertainty form changed under reversal for case {case_index}"
            );
        }
    }
    fn enumerate_grouped_source_bounds(
        groups: &[(i64, i64, i128)],
        index: usize,
        current: i128,
        extrema: &mut Option<(i128, i128)>,
    ) {
        if index == groups.len() {
            match extrema {
                Some((lower, upper)) => {
                    *lower = (*lower).min(current);
                    *upper = (*upper).max(current);
                }
                None => *extrema = Some((current, current)),
            }
            return;
        }

        let (lower, upper, coefficient) = groups[index];
        for value in lower..=upper {
            enumerate_grouped_source_bounds(
                groups,
                index + 1,
                current + coefficient * i128::from(value),
                extrema,
            );
        }
    }

    fn grouped_source_oracle(terms: &[SourceContribution]) -> (i128, i128) {
        let mut sources: BTreeMap<String, (i64, i64, i128)> = BTreeMap::new();
        for term in terms {
            let entry = sources.entry(term.dependency_id.clone()).or_insert((
                term.lower_mm, term.upper_mm, 0
            ));
            assert_eq!((entry.0, entry.1), (term.lower_mm, term.upper_mm));
            entry.2 += i128::from(term.coefficient);
        }
        let groups: Vec<_> = sources.into_values().collect();
        let mut extrema = None;
        enumerate_grouped_source_bounds(&groups, 0, 0, &mut extrema);
        extrema.expect("at least one distinct source assignment")
    }

    #[test]
    fn exhaustive_shared_source_oracle_checks_one_thousand_correlated_domains() {
        let mut seed = 0xC011_A710_5A0C_E123u64;
        for case_index in 0..1_000 {
            let count = (next_deterministic(&mut seed) % 8 + 1) as usize;
            let mut terms = Vec::with_capacity(count);
            for _ in 0..count {
                let source_index = next_deterministic(&mut seed) % 3;
                let coefficient = (next_deterministic(&mut seed) % 11) as i64 - 5;
                terms.push(contribution(
                    &format!("shared-{source_index}"),
                    -2, 2, coefficient,
                ));
            }

            let model = AffineUncertainty::new(terms.clone())
                .unwrap_or_else(|error| panic!("shared case {case_index} rejected: {error:?}"));
            let expected = grouped_source_oracle(&terms);
            let actual = model.bounds().unwrap_or_else(|error| {
                panic!("shared case {case_index} failed to bound: {error:?}")
            });
            assert_eq!(
                (i128::from(actual.lower_mm), i128::from(actual.upper_mm)),
                expected,
                "shared-source point oracle disagreed in case {case_index}: {terms:?}"
            );

            let mut reversed = terms;
            reversed.reverse();
            assert_eq!(
                model,
                AffineUncertainty::new(reversed).expect("permutation should normalize identically"),
                "shared-source canonical form changed under permutation in case {case_index}"
            );
        }
    }

    #[test]
    fn independent_point_oracle_rejects_deliberately_under_approximated_bounds() {
        let terms = vec![
            contribution("shared-x", -2, 2, 1),
            contribution("shared-x", -2, 2, -1),
            contribution("independent-y", -1, 3, 2),
        ];
        let model = AffineUncertainty::new(terms.clone()).unwrap();
        let (exact_lower, exact_upper) = grouped_source_oracle(&terms);
        let sound = model.bounds().unwrap();
        assert!(i128::from(sound.lower_mm) <= exact_lower);
        assert!(i128::from(sound.upper_mm) >= exact_upper);

        // Simulate a producer that reports a narrowed interval after a bug.
        let mutated = IntervalBounds {
            lower_mm: sound.lower_mm.saturating_add(1),
            upper_mm: sound.upper_mm.saturating_sub(1),
        };
        let rejected_by_oracle =
            i128::from(mutated.lower_mm) > exact_lower || i128::from(mutated.upper_mm) < exact_upper;
        assert!(rejected_by_oracle, "the independent oracle missed an under-approximation mutant");
    }
    #[test]
    fn hand_derived_json_vectors_match_affine_uncertainty_contract() {
        let fixture: FixtureSet = serde_json::from_str(include_str!("../fixtures/uncertainty_vectors.json"))
            .expect("uncertainty vector fixture must parse");
        assert_eq!(fixture.schema, "airspace-affine-uncertainty-v2");
        assert_eq!(fixture.status, "synthetic_research_vectors_with_declared_dependencies_not_authenticated_provenance");
        assert_eq!(fixture.cases.len(), 11);

        for case in fixture.cases {
            match AffineUncertainty::new(case.terms) {
                Ok(model) => {
                    match model.bounds() {
                        Ok(actual) => {
                            assert!(
                                case.expected_error.is_none(),
                                "{} expected error {}, but bounds were produced",
                                case.id,
                                case.expected_error.as_deref().unwrap_or("")
                            );
                            let expected = case.expected_bounds_mm
                                .unwrap_or_else(|| panic!("{} missing expected bounds", case.id));
                            assert_eq!(
                                [actual.lower_mm, actual.upper_mm],
                                expected,
                                "unexpected conservative bounds for {}",
                                case.id
                            );
                        }
                        Err(error) => {
                            assert_eq!(
                                case.expected_error.as_deref(),
                                Some(error_code(error)),
                                "unexpected bounds error for {}",
                                case.id
                            );
                            assert!(
                                case.expected_bounds_mm.is_none(),
                                "{} has both expected error and expected bounds",
                                case.id
                            );
                        }
                    }
                }
                Err(error) => {
                    assert_eq!(
                        case.expected_error.as_deref(),
                        Some(error_code(error)),
                        "unexpected rejection for {}",
                        case.id
                    );
                    assert!(
                        case.expected_bounds_mm.is_none(),
                        "{} has both expected error and expected bounds",
                        case.id
                    );
                }
            }
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
    fn noncanonical_source_identifiers_are_rejected() {
        assert_eq!(
            AffineUncertainty::new(vec![contribution(" datum-bias ", -1, 1, 1)]).unwrap_err(),
            UncertaintyError::NonCanonicalSourceId
        );
    }

    #[test]
    fn coefficient_fold_is_input_order_invariant_near_the_declared_limit() {
        let max = MAX_ABS_COEFFICIENT as i64;
        let terms = vec![
            contribution("same", -1, 1, max),
            contribution("same", -1, 1, max),
            contribution("same", -1, 1, -max),
        ];
        let forward = AffineUncertainty::new(terms.clone()).unwrap();
        let reversed = AffineUncertainty::new(terms.into_iter().rev().collect()).unwrap();

        assert_eq!(forward, reversed);
        assert_eq!(
            forward.bounds().unwrap(),
            IntervalBounds {
                lower_mm: -max,
                upper_mm: max
            }
        );
    }

    #[test]
    fn dependency_identity_with_conflicting_bounds_fails_closed() {
        let error = AffineUncertainty::new(vec![
            contribution("shared-source", -10, 10, 1),
            contribution("shared-source", -11, 10, -1),
        ])
        .unwrap_err();

        assert_eq!(error, UncertaintyError::DependencyBoundsMismatch);
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

    #[test]
    fn different_origins_can_share_one_explicit_latent_dependency() {
        let terms = vec![
            contribution_with_dependency("sensor-a", "shared-datum-bias", -10, 10, 1),
            contribution_with_dependency("sensor-b", "shared-datum-bias", -10, 10, -1),
        ];
        let model = AffineUncertainty::new(terms.clone()).unwrap();
        assert_eq!(model.bounds().unwrap(), IntervalBounds { lower_mm: 0, upper_mm: 0 });
        assert_eq!(model.terms.len(), 2, "retain both source origins for audit");
        assert_eq!(grouped_source_oracle(&terms), (0, 0));
    }

    #[test]
    fn one_origin_with_distinct_dependency_modes_must_not_cancel() {
        let model = AffineUncertainty::new(vec![
            contribution_with_dependency("navigation-unit", "bias-mode", -10, 10, 1),
            contribution_with_dependency("navigation-unit", "noise-mode", -10, 10, -1),
        ]).unwrap();
        assert_eq!(model.bounds().unwrap(), IntervalBounds { lower_mm: -20, upper_mm: 20 });
        assert_eq!(model.terms.len(), 2);
    }

    #[test]
    fn dependency_ids_cannot_claim_incompatible_ranges() {
        let error = AffineUncertainty::new(vec![
            contribution_with_dependency("sensor-a", "shared-bias", -10, 10, 1),
            contribution_with_dependency("sensor-b", "shared-bias", -11, 10, -1),
        ]).unwrap_err();
        assert_eq!(error, UncertaintyError::DependencyBoundsMismatch);
    }

    #[test]
    fn noncanonical_dependency_identifiers_fail_closed() {
        assert_eq!(
            AffineUncertainty::new(vec![contribution_with_dependency(
                "sensor", " shared-bias ", -1, 1, 1
            )]).unwrap_err(),
            UncertaintyError::NonCanonicalDependencyId
        );
    }

}

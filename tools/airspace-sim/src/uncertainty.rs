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
const MAX_DEPENDENCY_ID_BYTES: usize = 128;
const MAX_CONTRACT_LABEL_BYTES: usize = 128;
const MAX_ABS_SOURCE_BOUND_MM: i128 = 1_000_000_000_000;
const MAX_ABS_COEFFICIENT: i128 = 1_000_000;
const MAX_ABS_COMPOSED_BOUND_MM: i128 = 1_000_000_000_000_000;

#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize)]
struct DependencyContract {
    quantity_kind: String,
    unit: String,
    reference_frame: String,
    time_basis: String,
    valid_from_s: i64,
    valid_until_s: i64,
    provenance_status: String,
}

impl DependencyContract {
    fn validate(&self) -> Result<(), UncertaintyError> {
        for label in [
            self.quantity_kind.as_str(),
            self.unit.as_str(),
            self.reference_frame.as_str(),
            self.time_basis.as_str(),
            self.provenance_status.as_str(),
        ] {
            if label.trim().is_empty()
                || label.trim() != label
                || label.len() > MAX_CONTRACT_LABEL_BYTES
            {
                return Err(UncertaintyError::InvalidDependencyContractLabel);
            }
        }
        if self.valid_from_s >= self.valid_until_s {
            return Err(UncertaintyError::InvalidDependencyValidity);
        }
        if self.quantity_kind != "position_error"
            || self.unit != "synthetic_mm"
            || self.reference_frame != "synthetic_local_cartesian_v1"
            || self.time_basis != "synthetic_epoch_seconds_v1"
        {
            return Err(UncertaintyError::UnsupportedDependencyContract);
        }
        if self.provenance_status != "declared_unverified" {
            return Err(UncertaintyError::UnsupportedProvenanceStatus);
        }
        Ok(())
    }

    fn check_time(&self, evaluation_time_s: i64) -> Result<(), UncertaintyError> {
        if evaluation_time_s < self.valid_from_s {
            return Err(UncertaintyError::DependencyNotYetValid);
        }
        if evaluation_time_s >= self.valid_until_s {
            return Err(UncertaintyError::DependencyExpired);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize)]
struct SourceContribution {
    /// Stable source/origin identity retained for audit; it does not imply dependence.
    source_id: String,
    /// Declared identity of one shared latent error; not authenticated by this prototype.
    dependency_id: String,
    /// Typed quantity/frame/time/validity/provenance contract for this dependency.
    dependency_contract: DependencyContract,
    /// Inclusive bounds of the latent error, in synthetic millimetres.
    lower_mm: i64,
    upper_mm: i64,
    /// Exact integer sensitivity of this output to the named latent error.
    coefficient: i64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct AffineUncertainty {
    /// Canonically sorted by (dependency_id, source_id); duplicate pairs are folded.
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
    DependencyContractMismatch,
    InvalidDependencyContractLabel,
    InvalidDependencyValidity,
    UnsupportedDependencyContract,
    UnsupportedProvenanceStatus,
    DependencyNotYetValid,
    DependencyExpired,
    CoefficientOutOfRange,
    ArithmeticOverflow,
    OutputBoundsTooWide,
}

impl AffineUncertainty {
    fn new(terms: Vec<SourceContribution>) -> Result<Self, UncertaintyError> {
        Self::normalize(terms, MAX_SOURCE_TERMS)
    }

    /// Adds two affine expressions. Contributions with the same origin/dependency
    /// pair are folded for audit; bounds group all origins by dependency_id. The
    /// dependency declaration is not authenticated and different IDs do not prove
    /// physical independence.
    fn compose(&self, other: &Self) -> Result<Self, UncertaintyError> {
        let mut terms = Vec::with_capacity(self.terms.len() + other.terms.len());
        terms.extend(self.terms.iter().cloned());
        terms.extend(other.terms.iter().cloned());
        Self::normalize(terms, MAX_COMPOSITION_INPUTS)
    }

    /// Computes an exact interval over the declared latent dependencies.
    /// Contributions sharing a dependency_id vary together even when their source
    /// origins differ. This is a set bound under asserted integer ranges, not a
    /// probability or physical aircraft-state guarantee.
    fn bounds_at(&self, evaluation_time_s: i64) -> Result<IntervalBounds, UncertaintyError> {
        if self.terms.is_empty() {
            return Err(UncertaintyError::EmptyModel);
        }
        let mut groups: BTreeMap<&str, (i64, i64, i128)> = BTreeMap::new();
        for term in &self.terms {
            term.dependency_contract.check_time(evaluation_time_s)?;
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
        let mut dependency_contracts: BTreeMap<String, DependencyContract> = BTreeMap::new();
        for term in terms {
            if term.source_id.trim().is_empty() { return Err(UncertaintyError::EmptySourceId); }
            if term.source_id.as_str() != term.source_id.trim() { return Err(UncertaintyError::NonCanonicalSourceId); }
            if term.source_id.len() > MAX_SOURCE_ID_BYTES { return Err(UncertaintyError::SourceIdTooLong); }
            if term.dependency_id.trim().is_empty() { return Err(UncertaintyError::EmptyDependencyId); }
            if term.dependency_id.as_str() != term.dependency_id.trim() { return Err(UncertaintyError::NonCanonicalDependencyId); }
            if term.dependency_id.len() > MAX_DEPENDENCY_ID_BYTES { return Err(UncertaintyError::DependencyIdTooLong); }
            term.dependency_contract.validate()?;
            match dependency_contracts.get(&term.dependency_id) {
                Some(contract) if contract != &term.dependency_contract => {
                    return Err(UncertaintyError::DependencyContractMismatch);
                }
                Some(_) => {}
                None => {
                    dependency_contracts.insert(term.dependency_id.clone(), term.dependency_contract.clone());
                }
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
    use sha2::{Digest, Sha256};

    fn contribution(
        source_id: &str,
        lower_mm: i64,
        upper_mm: i64,
        coefficient: i64,
    ) -> SourceContribution {
        contribution_with_dependency(source_id, source_id, lower_mm, upper_mm, coefficient)
    }

    fn default_dependency_contract() -> DependencyContract {
        DependencyContract {
            quantity_kind: "position_error".to_owned(),
            unit: "synthetic_mm".to_owned(),
            reference_frame: "synthetic_local_cartesian_v1".to_owned(),
            time_basis: "synthetic_epoch_seconds_v1".to_owned(),
            valid_from_s: 0,
            valid_until_s: 1_000,
            provenance_status: "declared_unverified".to_owned(),
        }
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
            dependency_contract: default_dependency_contract(),
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
        evaluation_time_s: Option<i64>,
    }

    #[derive(serde::Deserialize)]
    struct EvidenceReceiptContract {
        schema: String,
        status: String,
        hash_algorithm: String,
        input_fixture_path: String,
        input_fixture_sha256: String,
        supported_dependency_contract_sha256: String,
        expected_outcomes_sha256: String,
        evaluator_id: String,
        checker_id: String,
        evaluation_time_policy: String,
        case_count: usize,
        source_revision_binding: String,
        signature_status: String,
        authority_ceiling: String,
    }

    #[derive(Debug, Clone, serde::Serialize)]
    struct ObservedCaseOutcome {
        id: String,
        evaluation_time_s: i64,
        outcome: String,
        lower_mm: Option<i64>,
        upper_mm: Option<i64>,
        error_code: Option<String>,
    }

    fn sha256_hex(bytes: &[u8]) -> String {
        use std::fmt::Write as _;
        let digest = Sha256::digest(bytes);
        let mut output = String::with_capacity(64);
        for byte in digest {
            write!(&mut output, "{byte:02x}")
                .expect("writing to a String cannot fail");
        }
        output
    }

    fn is_sha256_hex(value: &str) -> bool {
        value.len() == 64
            && value.bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    }

    fn observed_fixture_outcomes(fixture: &FixtureSet) -> Vec<ObservedCaseOutcome> {
        fixture.cases.iter().map(|case| {
            let evaluation_time_s = case.evaluation_time_s.unwrap_or(0);
            let result = match AffineUncertainty::new(case.terms.clone()) {
                Err(error) => Err(error),
                Ok(model) => model.bounds_at(evaluation_time_s),
            };
            match result {
                Ok(bounds) => ObservedCaseOutcome {
                    id: case.id.clone(),
                    evaluation_time_s,
                    outcome: "bounds".to_owned(),
                    lower_mm: Some(bounds.lower_mm),
                    upper_mm: Some(bounds.upper_mm),
                    error_code: None,
                },
                Err(error) => ObservedCaseOutcome {
                    id: case.id.clone(),
                    evaluation_time_s,
                    outcome: "error".to_owned(),
                    lower_mm: None,
                    upper_mm: None,
                    error_code: Some(error_code(error).to_owned()),
                },
            }
        }).collect()
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
            UncertaintyError::DependencyContractMismatch => "dependency_contract_mismatch",
            UncertaintyError::InvalidDependencyContractLabel => "invalid_dependency_contract_label",
            UncertaintyError::InvalidDependencyValidity => "invalid_dependency_validity",
            UncertaintyError::UnsupportedDependencyContract => "unsupported_dependency_contract",
            UncertaintyError::UnsupportedProvenanceStatus => "unsupported_provenance_status",
            UncertaintyError::DependencyNotYetValid => "dependency_not_yet_valid",
            UncertaintyError::DependencyExpired => "dependency_expired",
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
            let actual = model.bounds_at(0).unwrap_or_else(|error| {
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
    /// Exhaustively assigns one value per declared latent dependency, then evaluates
    /// the original contributions one-by-one. Unlike the production evaluator, this
    /// test oracle does not coalesce sensitivities or use a dependency-keyed map.
    /// Callers intentionally use tiny finite domains so enumeration is tractable.
    fn enumerate_dependency_assignments(
        terms: &[SourceContribution],
        dependencies: &[(String, i64, i64)],
        dependency_index: usize,
        assigned_values: &mut Vec<i64>,
        extrema: &mut Option<(i128, i128)>,
    ) {
        if dependency_index == dependencies.len() {
            let mut total = 0_i128;
            for term in terms {
                let value_index = dependencies
                    .iter()
                    .position(|(dependency_id, _, _)| dependency_id.as_str() == term.dependency_id.as_str())
                    .expect("every contribution must name one enumerated dependency");
                let contribution = i128::from(term.coefficient)
                    .checked_mul(i128::from(assigned_values[value_index]))
                    .expect("small finite oracle product must fit in i128");
                total = total
                    .checked_add(contribution)
                    .expect("small finite oracle sum must fit in i128");
            }
            match extrema {
                Some((lower, upper)) => {
                    *lower = (*lower).min(total);
                    *upper = (*upper).max(total);
                }
                None => *extrema = Some((total, total)),
            }
            return;
        }

        let (_, lower, upper) = &dependencies[dependency_index];
        for value in *lower..=*upper {
            assigned_values.push(value);
            enumerate_dependency_assignments(
                terms,
                dependencies,
                dependency_index + 1,
                assigned_values,
                extrema,
            );
            assigned_values.pop().expect("the recursive assignment stack must be balanced");
        }
    }

    fn exhaustive_dependency_assignment_oracle(
        terms: &[SourceContribution],
    ) -> (i128, i128) {
        let mut dependencies: Vec<(String, i64, i64)> = Vec::new();
        for term in terms {
            if let Some((_, lower, upper)) = dependencies
                .iter()
                .find(|(dependency_id, _, _)| dependency_id.as_str() == term.dependency_id.as_str())
            {
                assert_eq!(
                    (*lower, *upper),
                    (term.lower_mm, term.upper_mm),
                    "one dependency cannot have conflicting finite domains"
                );
            } else {
                dependencies.push((
                    term.dependency_id.clone(),
                    term.lower_mm,
                    term.upper_mm,
                ));
            }
        }

        assert!(!dependencies.is_empty(), "oracle input must not be empty");
        let mut extrema = None;
        enumerate_dependency_assignments(
            terms,
            &dependencies,
            0,
            &mut Vec::with_capacity(dependencies.len()),
            &mut extrema,
        );
        extrema.expect("each finite dependency has at least one assigned value")
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
            let expected = exhaustive_dependency_assignment_oracle(&terms);
            let actual = model.bounds_at(0).unwrap_or_else(|error| {
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
    fn independent_list_oracle_detects_shared_dependency_under_approximation() {
        let terms = vec![
            contribution_with_dependency("sensor-east", "shared-datum", -2, 2, 1),
            contribution_with_dependency("sensor-west", "shared-datum", -2, 2, -1),
            contribution_with_dependency("navigation", "independent-noise", -1, 3, 2),
        ];
        let model = AffineUncertainty::new(terms.clone()).unwrap();
        let exact = exhaustive_dependency_assignment_oracle(&terms);
        let actual = model.bounds_at(0).unwrap();
        assert_eq!(
            (i128::from(actual.lower_mm), i128::from(actual.upper_mm)),
            exact,
            "the evaluator must agree with a structurally different point-assignment oracle"
        );
        assert_eq!(exact, (-2, 6));

        // A narrowed producer report is a fail-closed disagreement, not a weaker pass.
        let mutated = IntervalBounds {
            lower_mm: actual.lower_mm + 1,
            upper_mm: actual.upper_mm - 1,
        };
        let oracle_detects_mutant =
            i128::from(mutated.lower_mm) > exact.0 || i128::from(mutated.upper_mm) < exact.1;
        assert!(
            oracle_detects_mutant,
            "the independent oracle accepted an under-approximated output"
        );
    }

    #[test]
    fn independent_point_oracle_rejects_deliberately_under_approximated_bounds() {
        let terms = vec![
            contribution("shared-x", -2, 2, 1),
            contribution("shared-x", -2, 2, -1),
            contribution("independent-y", -1, 3, 2),
        ];
        let model = AffineUncertainty::new(terms.clone()).unwrap();
        let (exact_lower, exact_upper) = exhaustive_dependency_assignment_oracle(&terms);
        let sound = model.bounds_at(0).unwrap();
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
        assert_eq!(fixture.schema, "airspace-affine-uncertainty-v3");
        assert_eq!(fixture.status, "synthetic_research_vectors_with_declared_dependencies_not_authenticated_provenance");
        assert_eq!(fixture.cases.len(), 22);

        for case in fixture.cases {
            match AffineUncertainty::new(case.terms) {
                Ok(model) => {
                    match model.bounds_at(case.evaluation_time_s.unwrap_or(0)) {
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
    fn shared_common_mode_dependency_can_cancel_only_via_explicit_identity_and_coefficients() {
        let forward =
            AffineUncertainty::new(vec![contribution("datum-bias", -10, 10, 1)]).unwrap();
        let inverse =
            AffineUncertainty::new(vec![contribution("datum-bias", -10, 10, -1)]).unwrap();

        let relative = forward.compose(&inverse).unwrap();
        assert_eq!(
            relative.bounds_at(0).unwrap(),
            IntervalBounds {
                lower_mm: 0,
                upper_mm: 0
            }
        );
        assert_eq!(relative.terms.len(), 1);
        assert_eq!(relative.terms[0].coefficient, 0);
    }

    #[test]
    fn different_dependency_ids_never_cancel_by_assumption() {
        let first =
            AffineUncertainty::new(vec![contribution("datum-source-a", -10, 10, 1)]).unwrap();
        let second =
            AffineUncertainty::new(vec![contribution("datum-source-b", -10, 10, -1)]).unwrap();

        assert_eq!(
            first.compose(&second).unwrap().bounds_at(0).unwrap(),
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
            observed.bounds_at(0).unwrap(),
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
            model.bounds_at(0).unwrap(),
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
            forward.bounds_at(0).unwrap(),
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
            forward.bounds_at(0).unwrap(),
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
            wide.bounds_at(0).unwrap_err(),
            UncertaintyError::OutputBoundsTooWide
        );
    }

    #[test]
    fn different_origins_can_share_one_explicit_latent_dependency() {
        let first_term = contribution_with_dependency("sensor-a", "shared-datum-bias", -10, 10, 1);
        let second_term = contribution_with_dependency("sensor-b", "shared-datum-bias", -10, 10, -1);
        let first = AffineUncertainty::new(vec![first_term.clone()]).unwrap();
        let second = AffineUncertainty::new(vec![second_term.clone()]).unwrap();
        let composed = first.compose(&second).unwrap();
        let terms = vec![first_term, second_term];

        assert_eq!(composed.bounds_at(0).unwrap(), IntervalBounds { lower_mm: 0, upper_mm: 0 });
        assert_eq!(composed.terms.len(), 2, "retain both source origins for audit");
        assert_eq!(exhaustive_dependency_assignment_oracle(&terms), (0, 0));
    }

    #[test]
    fn one_origin_with_distinct_dependency_modes_must_not_cancel() {
        let model = AffineUncertainty::new(vec![
            contribution_with_dependency("navigation-unit", "bias-mode", -10, 10, 1),
            contribution_with_dependency("navigation-unit", "noise-mode", -2, 2, -1),
        ]).unwrap();
        assert_eq!(model.bounds_at(0).unwrap(), IntervalBounds { lower_mm: -12, upper_mm: 12 });
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
    fn dependency_identifier_domain_is_bounded_and_canonical() {
        assert_eq!(
            AffineUncertainty::new(vec![contribution_with_dependency(
                "sensor", " shared-bias ", -1, 1, 1
            )]).unwrap_err(),
            UncertaintyError::NonCanonicalDependencyId
        );
        assert_eq!(
            AffineUncertainty::new(vec![contribution_with_dependency(
                "sensor", "", -1, 1, 1
            )]).unwrap_err(),
            UncertaintyError::EmptyDependencyId
        );
        let oversized = "d".repeat(MAX_DEPENDENCY_ID_BYTES + 1);
        assert_eq!(
            AffineUncertainty::new(vec![contribution_with_dependency(
                "sensor", &oversized, -1, 1, 1
            )]).unwrap_err(),
            UncertaintyError::DependencyIdTooLong
        );
    }


    #[test]
    fn replay_evidence_receipt_binds_exact_inputs_supported_contract_and_observed_outcomes() {
        let receipt: EvidenceReceiptContract = serde_json::from_str(include_str!(
            "../fixtures/uncertainty_evidence_receipt_contract.json"
        )).expect("evidence receipt contract must parse");
        let fixture: FixtureSet = serde_json::from_str(include_str!(
            "../fixtures/uncertainty_vectors.json"
        )).expect("vector fixture must parse");

        assert_eq!(receipt.schema, "airspace-uncertainty-evidence-receipt-v1");
        assert_eq!(receipt.status, "expected_digest_contract_not_runtime_attestation");
        assert_eq!(receipt.hash_algorithm, "sha-256");
        assert_eq!(
            receipt.input_fixture_path,
            "tools/airspace-sim/fixtures/uncertainty_vectors.json"
        );
        assert!(is_sha256_hex(&receipt.input_fixture_sha256));
        assert!(is_sha256_hex(&receipt.supported_dependency_contract_sha256));
        assert!(is_sha256_hex(&receipt.expected_outcomes_sha256));
        assert_eq!(
            receipt.input_fixture_sha256,
            sha256_hex(include_bytes!("../fixtures/uncertainty_vectors.json"))
        );

        let supported_contract = serde_json::to_vec(&default_dependency_contract())
            .expect("supported dependency contract must serialize deterministically");
        assert_eq!(
            receipt.supported_dependency_contract_sha256,
            sha256_hex(&supported_contract)
        );

        let outcome_bytes = serde_json::to_vec(&observed_fixture_outcomes(&fixture))
            .expect("observed outcomes must serialize deterministically");
        assert_eq!(
            receipt.expected_outcomes_sha256,
            sha256_hex(&outcome_bytes),
            "replayed outcomes did not match the receipt's pinned result digest"
        );
        assert_eq!(receipt.case_count, fixture.cases.len());
        assert_eq!(receipt.evaluator_id, "scalar_affine_interval_v3");
        assert_eq!(
            receipt.checker_id,
            "golden_vectors_plus_finite_domain_point_oracle_v1"
        );
        assert_eq!(receipt.evaluation_time_policy, "case_override_else_zero");
        assert_eq!(
            receipt.source_revision_binding,
            "external_exact_head_ci_context_required"
        );
        assert_eq!(receipt.signature_status, "unsigned");
        assert_eq!(
            receipt.authority_ceiling,
            "offline_synthetic_research_only_no_operational_aviation_authority"
        );
    }

    #[test]
    fn replay_evidence_receipt_detects_input_contract_and_output_mutations() {
        let fixture_bytes = include_bytes!("../fixtures/uncertainty_vectors.json");
        let original_input_digest = sha256_hex(fixture_bytes);
        let mut changed_input = fixture_bytes.to_vec();
        changed_input.push(b' ');
        assert_ne!(sha256_hex(&changed_input), original_input_digest);

        let supported_contract = default_dependency_contract();
        let original_contract_digest = sha256_hex(
            &serde_json::to_vec(&supported_contract).unwrap()
        );
        let mut changed_contract = supported_contract;
        changed_contract.unit = "feet".to_owned();
        assert_ne!(
            sha256_hex(&serde_json::to_vec(&changed_contract).unwrap()),
            original_contract_digest
        );

        let fixture: FixtureSet = serde_json::from_str(include_str!(
            "../fixtures/uncertainty_vectors.json"
        )).unwrap();
        let outcomes = observed_fixture_outcomes(&fixture);
        let original_outcome_digest = sha256_hex(&serde_json::to_vec(&outcomes).unwrap());
        let mut changed_outcomes = outcomes;
        let first = changed_outcomes.first_mut().expect("fixture is non-empty");
        first.lower_mm = Some(first.lower_mm.expect("first outcome has a lower bound") + 1);
        assert_ne!(
            sha256_hex(&serde_json::to_vec(&changed_outcomes).unwrap()),
            original_outcome_digest
        );
    }

    #[test]
    fn dependency_contract_rejects_unknown_units_frames_time_bases_and_provenance() {
        let mut term = contribution("sensor", -1, 1, 1);
        term.dependency_contract.unit = "feet".to_owned();
        assert_eq!(AffineUncertainty::new(vec![term]).unwrap_err(), UncertaintyError::UnsupportedDependencyContract);

        let mut term = contribution("sensor", -1, 1, 1);
        term.dependency_contract.reference_frame = "WGS84".to_owned();
        assert_eq!(AffineUncertainty::new(vec![term]).unwrap_err(), UncertaintyError::UnsupportedDependencyContract);

        let mut term = contribution("sensor", -1, 1, 1);
        term.dependency_contract.time_basis = "UTC".to_owned();
        assert_eq!(AffineUncertainty::new(vec![term]).unwrap_err(), UncertaintyError::UnsupportedDependencyContract);

        let mut term = contribution("sensor", -1, 1, 1);
        term.dependency_contract.provenance_status = "authenticated".to_owned();
        assert_eq!(AffineUncertainty::new(vec![term]).unwrap_err(), UncertaintyError::UnsupportedProvenanceStatus);
    }

    #[test]
    fn dependency_contract_must_match_for_every_contribution_to_one_dependency() {
        let first = contribution_with_dependency("sensor-a", "shared", -1, 1, 1);
        let mut second = contribution_with_dependency("sensor-b", "shared", -1, 1, -1);
        second.dependency_contract.valid_until_s = 900;
        assert_eq!(
            AffineUncertainty::new(vec![first, second]).unwrap_err(),
            UncertaintyError::DependencyContractMismatch
        );
    }

    #[test]
    fn dependency_contract_validity_uses_a_half_open_interval() {
        let mut term = contribution("sensor", -1, 1, 1);
        term.dependency_contract.valid_from_s = 10;
        term.dependency_contract.valid_until_s = 20;
        let model = AffineUncertainty::new(vec![term]).unwrap();
        assert_eq!(model.bounds_at(9).unwrap_err(), UncertaintyError::DependencyNotYetValid);
        assert_eq!(model.bounds_at(10).unwrap(), IntervalBounds { lower_mm: -1, upper_mm: 1 });
        assert_eq!(model.bounds_at(19).unwrap(), IntervalBounds { lower_mm: -1, upper_mm: 1 });
        assert_eq!(model.bounds_at(20).unwrap_err(), UncertaintyError::DependencyExpired);
    }

    #[test]
    fn dependency_contract_rejects_empty_labels_and_invalid_validity_windows() {
        let mut term = contribution("sensor", -1, 1, 1);
        term.dependency_contract.unit.clear();
        assert_eq!(AffineUncertainty::new(vec![term]).unwrap_err(), UncertaintyError::InvalidDependencyContractLabel);
        let mut term = contribution("sensor", -1, 1, 1);
        term.dependency_contract.valid_from_s = 20;
        term.dependency_contract.valid_until_s = 20;
        assert_eq!(AffineUncertainty::new(vec![term]).unwrap_err(), UncertaintyError::InvalidDependencyValidity);
    }

}

//! Test-only dimensional guard for composing heterogeneous uncertainty sources.
//!
//! Kept separate from uncertainty.rs: that prototype accepts only synthetic
//! position-error values. Here, clock offsets and transform residuals must declare
//! input dimensions and explicit sensitivity to a common synthetic millimetre output.
//! This is not a qualified coordinate/time transform engine or aviation uncertainty model.

use std::collections::BTreeMap;
use serde::Deserialize;

const MAX_TERMS: usize = 64;
const MAX_LABEL_BYTES: usize = 128;
const MAX_ABS_INPUT_VALUE: i128 = 1_000_000_000;
const MAX_ABS_SENSITIVITY_X1000: i128 = 1_000_000_000;
const SENSITIVITY_SCALE: i128 = 1_000;
const MAX_ABS_OUTPUT_MM: i128 = 1_000_000_000_000;
const VALIDITY_TIME_BASIS: &str = "synthetic_epoch_seconds_v1";
const OUTPUT_FRAME: &str = "synthetic_local_cartesian_v1";

#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
struct TypedErrorTerm {
    source_id: String,
    dependency_id: String,
    quantity_kind: String,
    unit: String,
    reference_frame: String,
    validity_time_basis: String,
    valid_from_s: i64,
    valid_until_s: i64,
    lower_value: i64,
    upper_value: i64,
    /// Sensitivity in millimetres per input unit, multiplied by 1,000.
    /// For clock offsets in milliseconds, velocity in mm/s is represented directly:
    /// (velocity / 1,000) mm/ms * 1,000 == velocity.
    sensitivity_scaled: i64,
    sensitivity_basis: String,
    transform_id: Option<String>,
    transform_source_frame: Option<String>,
    transform_target_frame: Option<String>,
    sensitivity_model_id: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct DependencyContract {
    quantity_kind: String,
    unit: String,
    reference_frame: String,
    validity_time_basis: String,
    valid_from_s: i64,
    valid_until_s: i64,
    transform_id: Option<String>,
    transform_source_frame: Option<String>,
    transform_target_frame: Option<String>,
}

impl From<&TypedErrorTerm> for DependencyContract {
    fn from(term: &TypedErrorTerm) -> Self {
        Self {
            quantity_kind: term.quantity_kind.clone(),
            unit: term.unit.clone(),
            reference_frame: term.reference_frame.clone(),
            validity_time_basis: term.validity_time_basis.clone(),
            valid_from_s: term.valid_from_s,
            valid_until_s: term.valid_until_s,
            transform_id: term.transform_id.clone(),
            transform_source_frame: term.transform_source_frame.clone(),
            transform_target_frame: term.transform_target_frame.clone(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LineageError {
    EmptyModel,
    TooManyTerms,
    InvalidContractLabel,
    InvalidValidity,
    InvalidBounds,
    SensitivityOutOfRange,
    UnsupportedQuantityUnit,
    UnsupportedReferenceFrame,
    InvalidSensitivityBasis,
    MissingTransformIdentity,
    TransformTargetFrameMismatch,
    UnexpectedTransformMetadata,
    MissingSensitivityModel,
    UnexpectedSensitivityModel,
    DependencyContractMismatch,
    DependencyBoundsMismatch,
    DependencyNotYetValid,
    DependencyExpired,
    ArithmeticOverflow,
    OutputBoundsTooWide,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct BoundsMm {
    lower_mm: i64,
    upper_mm: i64,
}

#[derive(Debug, Clone)]
struct TypedLineageModel {
    terms: Vec<TypedErrorTerm>,
}

impl TypedLineageModel {
    fn new(terms: Vec<TypedErrorTerm>) -> Result<Self, LineageError> {
        if terms.is_empty() {
            return Err(LineageError::EmptyModel);
        }
        if terms.len() > MAX_TERMS {
            return Err(LineageError::TooManyTerms);
        }

        let mut contracts: BTreeMap<String, DependencyContract> = BTreeMap::new();
        let mut bounds: BTreeMap<String, (i64, i64)> = BTreeMap::new();
        for term in &terms {
            validate_term(term)?;
            let contract = DependencyContract::from(term);
            match contracts.get(&term.dependency_id) {
                Some(existing) if existing != &contract => {
                    return Err(LineageError::DependencyContractMismatch);
                }
                Some(_) => {}
                None => {
                    contracts.insert(term.dependency_id.clone(), contract);
                }
            }
            match bounds.get(&term.dependency_id) {
                Some(existing) if *existing != (term.lower_value, term.upper_value) => {
                    return Err(LineageError::DependencyBoundsMismatch);
                }
                Some(_) => {}
                None => {
                    bounds.insert(term.dependency_id.clone(), (term.lower_value, term.upper_value));
                }
            }
        }
        Ok(Self { terms })
    }

    fn bounds_at(&self, evaluation_time_s: i64) -> Result<BoundsMm, LineageError> {
        let mut groups: BTreeMap<&str, (i64, i64, i128)> = BTreeMap::new();
        for term in &self.terms {
            if evaluation_time_s < term.valid_from_s {
                return Err(LineageError::DependencyNotYetValid);
            }
            if evaluation_time_s >= term.valid_until_s {
                return Err(LineageError::DependencyExpired);
            }
            let group = groups.entry(term.dependency_id.as_str())
                .or_insert((term.lower_value, term.upper_value, 0));
            if (group.0, group.1) != (term.lower_value, term.upper_value) {
                return Err(LineageError::DependencyBoundsMismatch);
            }
            group.2 = group.2.checked_add(i128::from(term.sensitivity_scaled))
                .ok_or(LineageError::ArithmeticOverflow)?;
        }

        let mut lower_scaled: i128 = 0;
        let mut upper_scaled: i128 = 0;
        for (input_lower, input_upper, sensitivity) in groups.values().copied() {
            let (term_lower, term_upper) = if sensitivity >= 0 {
                (
                    sensitivity.checked_mul(i128::from(input_lower))
                        .ok_or(LineageError::ArithmeticOverflow)?,
                    sensitivity.checked_mul(i128::from(input_upper))
                        .ok_or(LineageError::ArithmeticOverflow)?,
                )
            } else {
                (
                    sensitivity.checked_mul(i128::from(input_upper))
                        .ok_or(LineageError::ArithmeticOverflow)?,
                    sensitivity.checked_mul(i128::from(input_lower))
                        .ok_or(LineageError::ArithmeticOverflow)?,
                )
            };
            lower_scaled = lower_scaled.checked_add(term_lower)
                .ok_or(LineageError::ArithmeticOverflow)?;
            upper_scaled = upper_scaled.checked_add(term_upper)
                .ok_or(LineageError::ArithmeticOverflow)?;
        }

        if lower_scaled > upper_scaled
            || lower_scaled.abs() > MAX_ABS_OUTPUT_MM * SENSITIVITY_SCALE
            || upper_scaled.abs() > MAX_ABS_OUTPUT_MM * SENSITIVITY_SCALE
        {
            return Err(LineageError::OutputBoundsTooWide);
        }

        // Outward rounding never narrows the common-denominator interval.
        let lower_mm = lower_scaled.div_euclid(SENSITIVITY_SCALE);
        let upper_floor = upper_scaled.div_euclid(SENSITIVITY_SCALE);
        let upper_mm = if upper_scaled.rem_euclid(SENSITIVITY_SCALE) == 0 {
            upper_floor
        } else {
            upper_floor.checked_add(1).ok_or(LineageError::ArithmeticOverflow)?
        };
        Ok(BoundsMm {
            lower_mm: i64::try_from(lower_mm).map_err(|_| LineageError::ArithmeticOverflow)?,
            upper_mm: i64::try_from(upper_mm).map_err(|_| LineageError::ArithmeticOverflow)?,
        })
    }
}

fn valid_label(value: &str) -> bool {
    !value.trim().is_empty() && value.trim() == value && value.len() <= MAX_LABEL_BYTES
}

fn validate_term(term: &TypedErrorTerm) -> Result<(), LineageError> {
    for label in [
        term.source_id.as_str(),
        term.dependency_id.as_str(),
        term.quantity_kind.as_str(),
        term.unit.as_str(),
        term.reference_frame.as_str(),
        term.validity_time_basis.as_str(),
        term.sensitivity_basis.as_str(),
    ] {
        if !valid_label(label) {
            return Err(LineageError::InvalidContractLabel);
        }
    }
    if term.valid_from_s >= term.valid_until_s {
        return Err(LineageError::InvalidValidity);
    }
    if term.validity_time_basis != VALIDITY_TIME_BASIS {
        return Err(LineageError::UnsupportedQuantityUnit);
    }
    if term.lower_value > term.upper_value
        || i128::from(term.lower_value).abs() > MAX_ABS_INPUT_VALUE
        || i128::from(term.upper_value).abs() > MAX_ABS_INPUT_VALUE
    {
        return Err(LineageError::InvalidBounds);
    }
    if i128::from(term.sensitivity_scaled).abs() > MAX_ABS_SENSITIVITY_X1000 {
        return Err(LineageError::SensitivityOutOfRange);
    }

    match term.quantity_kind.as_str() {
        "position_error" => {
            if term.unit != "synthetic_mm" {
                return Err(LineageError::UnsupportedQuantityUnit);
            }
            if term.reference_frame != OUTPUT_FRAME {
                return Err(LineageError::UnsupportedReferenceFrame);
            }
            if term.sensitivity_basis != "mm_per_mm_x1000" {
                return Err(LineageError::InvalidSensitivityBasis);
            }
            if term.transform_id.is_some() || term.transform_source_frame.is_some() || term.transform_target_frame.is_some() {
                return Err(LineageError::UnexpectedTransformMetadata);
            }
            if term.sensitivity_model_id.is_some() {
                return Err(LineageError::UnexpectedSensitivityModel);
            }
        }
        "transform_error" => {
            if term.unit != "synthetic_mm" {
                return Err(LineageError::UnsupportedQuantityUnit);
            }
            if term.reference_frame != OUTPUT_FRAME {
                return Err(LineageError::UnsupportedReferenceFrame);
            }
            if term.sensitivity_basis != "mm_per_mm_x1000" {
                return Err(LineageError::InvalidSensitivityBasis);
            }
            if !term.transform_id.as_deref().is_some_and(valid_label)
                || !term.transform_source_frame.as_deref().is_some_and(valid_label)
            {
                return Err(LineageError::MissingTransformIdentity);
            }
            let target_frame = term.transform_target_frame.as_deref()
                .ok_or(LineageError::MissingTransformIdentity)?;
            if target_frame != term.reference_frame {
                return Err(LineageError::TransformTargetFrameMismatch);
            }
            if term.sensitivity_model_id.is_some() {
                return Err(LineageError::UnexpectedSensitivityModel);
            }
        }
        "clock_offset_error" => {
            if term.unit != "synthetic_ms" {
                return Err(LineageError::UnsupportedQuantityUnit);
            }
            if term.reference_frame != "not_spatial" {
                return Err(LineageError::UnsupportedReferenceFrame);
            }
            if term.sensitivity_basis != "mm_per_ms_x1000" {
                return Err(LineageError::InvalidSensitivityBasis);
            }
            if term.transform_id.is_some() || term.transform_source_frame.is_some() || term.transform_target_frame.is_some() {
                return Err(LineageError::UnexpectedTransformMetadata);
            }
            if !term.sensitivity_model_id.as_deref().is_some_and(valid_label) {
                return Err(LineageError::MissingSensitivityModel);
            }
        }
        _ => return Err(LineageError::UnsupportedQuantityUnit),
    }
    Ok(())
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FixtureSet {
    schema: String,
    status: String,
    authority_ceiling: String,
    output: OutputContract,
    cases: Vec<FixtureCase>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct OutputContract {
    quantity_kind: String,
    unit: String,
    reference_frame: String,
    sensitivity_scale_denominator: i64,
    rounding: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FixtureCase {
    id: String,
    evaluation_time_s: i64,
    terms: Vec<TypedErrorTerm>,
    expected_bounds_mm: Option<[i64; 2]>,
    expected_error: Option<String>,
}

fn error_code(error: LineageError) -> &'static str {
    match error {
        LineageError::EmptyModel => "empty_model",
        LineageError::TooManyTerms => "too_many_terms",
        LineageError::InvalidContractLabel => "invalid_contract_label",
        LineageError::InvalidValidity => "invalid_validity",
        LineageError::InvalidBounds => "invalid_bounds",
        LineageError::SensitivityOutOfRange => "sensitivity_out_of_range",
        LineageError::UnsupportedQuantityUnit => "unsupported_quantity_unit",
        LineageError::UnsupportedReferenceFrame => "unsupported_reference_frame",
        LineageError::InvalidSensitivityBasis => "invalid_sensitivity_basis",
        LineageError::MissingTransformIdentity => "missing_transform_identity",
        LineageError::TransformTargetFrameMismatch => "transform_target_frame_mismatch",
        LineageError::UnexpectedTransformMetadata => "unexpected_transform_metadata",
        LineageError::MissingSensitivityModel => "missing_sensitivity_model",
        LineageError::UnexpectedSensitivityModel => "unexpected_sensitivity_model",
        LineageError::DependencyContractMismatch => "dependency_contract_mismatch",
        LineageError::DependencyBoundsMismatch => "dependency_bounds_mismatch",
        LineageError::DependencyNotYetValid => "dependency_not_yet_valid",
        LineageError::DependencyExpired => "dependency_expired",
        LineageError::ArithmeticOverflow => "arithmetic_overflow",
        LineageError::OutputBoundsTooWide => "output_bounds_too_wide",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn typed_lineage_fixture_matches_units_conversions_and_negative_controls() {
        let fixture: FixtureSet = serde_json::from_str(include_str!(
            "../fixtures/typed_uncertainty_lineage.json"
        )).expect("typed uncertainty lineage fixture must parse");
        assert_eq!(fixture.schema, "airspace-typed-uncertainty-lineage-v1");
        assert_eq!(fixture.status, "synthetic_test_contract_not_production_transform_engine");
        assert_eq!(
            fixture.authority_ceiling,
            "offline_synthetic_research_only_no_operational_aviation_authority"
        );
        assert_eq!(fixture.cases.len(), 7);
        assert_eq!(fixture.output.quantity_kind, "position_error");
        assert_eq!(fixture.output.unit, "synthetic_mm");
        assert_eq!(fixture.output.reference_frame, OUTPUT_FRAME);
        assert_eq!(fixture.output.sensitivity_scale_denominator, 1_000);
        assert_eq!(fixture.output.rounding, "outward_floor_lower_and_ceil_upper");

        for case in fixture.cases {
            let result = TypedLineageModel::new(case.terms)
                .and_then(|model| model.bounds_at(case.evaluation_time_s));
            match result {
                Ok(bounds) => {
                    assert!(case.expected_error.is_none(), "{} unexpectedly produced bounds", case.id);
                    assert_eq!(
                        [bounds.lower_mm, bounds.upper_mm],
                        case.expected_bounds_mm.expect("successful case needs expected bounds"),
                        "typed lineage mismatch for {}",
                        case.id
                    );
                }
                Err(error) => {
                    assert_eq!(
                        case.expected_error.as_deref(),
                        Some(error_code(error)),
                        "typed lineage error mismatch for {}",
                        case.id
                    );
                    assert!(case.expected_bounds_mm.is_none(), "{} has both bounds and error", case.id);
                }
            }
        }
    }

    #[test]
    fn clock_conversion_rounds_outward_and_never_understates_fractional_millimetres() {
        let clock = TypedErrorTerm {
            source_id: "clock-test".to_owned(),
            dependency_id: "clock-test".to_owned(),
            quantity_kind: "clock_offset_error".to_owned(),
            unit: "synthetic_ms".to_owned(),
            reference_frame: "not_spatial".to_owned(),
            validity_time_basis: VALIDITY_TIME_BASIS.to_owned(),
            valid_from_s: 0,
            valid_until_s: 10,
            lower_value: -1,
            upper_value: 1,
            sensitivity_scaled: 15,
            sensitivity_basis: "mm_per_ms_x1000".to_owned(),
            transform_id: None,
            transform_source_frame: None,
            transform_target_frame: None,
            sensitivity_model_id: Some("synthetic_constant_velocity_v1".to_owned()),
        };
        let bounds = TypedLineageModel::new(vec![clock]).unwrap().bounds_at(0).unwrap();
        assert_eq!(bounds, BoundsMm { lower_mm: -1, upper_mm: 1 });
    }

    #[test]
    fn one_dependency_cannot_silently_change_dimension_or_validity() {
        let base = TypedErrorTerm {
            source_id: "clock-a".to_owned(),
            dependency_id: "shared-clock".to_owned(),
            quantity_kind: "clock_offset_error".to_owned(),
            unit: "synthetic_ms".to_owned(),
            reference_frame: "not_spatial".to_owned(),
            validity_time_basis: VALIDITY_TIME_BASIS.to_owned(),
            valid_from_s: 0,
            valid_until_s: 10,
            lower_value: -2,
            upper_value: 2,
            sensitivity_scaled: 15,
            sensitivity_basis: "mm_per_ms_x1000".to_owned(),
            transform_id: None,
            transform_source_frame: None,
            transform_target_frame: None,
            sensitivity_model_id: Some("synthetic_constant_velocity_v1".to_owned()),
        };
        let mut changed = base.clone();
        changed.source_id = "same-id-as-position".to_owned();
        changed.quantity_kind = "position_error".to_owned();
        changed.unit = "synthetic_mm".to_owned();
        changed.reference_frame = OUTPUT_FRAME.to_owned();
        changed.sensitivity_basis = "mm_per_mm_x1000".to_owned();
        changed.sensitivity_model_id = None;
        changed.sensitivity_scaled = 1_000;
        assert_eq!(
            TypedLineageModel::new(vec![base.clone(), changed]).unwrap_err(),
            LineageError::DependencyContractMismatch
        );

        let mut expired_contract = base.clone();
        expired_contract.source_id = "clock-c".to_owned();
        expired_contract.valid_until_s = 9;
        assert_eq!(
            TypedLineageModel::new(vec![base, expired_contract]).unwrap_err(),
            LineageError::DependencyContractMismatch
        );
    }
    #[test]
    fn unknown_contract_fields_fail_closed_instead_of_being_ignored() {
        let mut value: serde_json::Value = serde_json::from_str(include_str!(
            "../fixtures/typed_uncertainty_lineage.json"
        )).expect("typed lineage fixture must parse before mutation");
        value["cases"][0]["terms"][0]
            .as_object_mut()
            .expect("first term must be an object")
            .insert("sensitivity_sclaed".to_owned(), serde_json::json!(1_000));

        let error = serde_json::from_value::<FixtureSet>(value)
            .expect_err("an unknown misspelled contract field must be rejected");
        assert!(
            error.to_string().contains("unknown field"),
            "expected an unknown-field error, got: {error}"
        );
    }

    fn next_oracle_seed(seed: &mut u64) -> u64 {
        *seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        *seed
    }

    fn enumerate_lineage_assignments(
        terms: &[TypedErrorTerm],
        dependencies: &[(String, i64, i64)],
        index: usize,
        assigned: &mut Vec<i64>,
        extrema: &mut Option<(i128, i128)>,
    ) {
        if index == dependencies.len() {
            // Deliberately evaluate the original terms one by one. This oracle
            // doesn't use the producer's dependency-coefficient coalescing loop.
            let mut total_scaled = 0_i128;
            for term in terms {
                let value_index = dependencies
                    .iter()
                    .position(|(id, _, _)| id.as_str() == term.dependency_id.as_str())
                    .expect("each term must map to a finite dependency assignment");
                let contribution = i128::from(term.sensitivity_scaled)
                    .checked_mul(i128::from(assigned[value_index]))
                    .expect("small oracle product must fit in i128");
                total_scaled = total_scaled
                    .checked_add(contribution)
                    .expect("small oracle sum must fit in i128");
            }
            match extrema {
                Some((lower, upper)) => {
                    *lower = (*lower).min(total_scaled);
                    *upper = (*upper).max(total_scaled);
                }
                None => *extrema = Some((total_scaled, total_scaled)),
            }
            return;
        }

        let (_, low, high) = &dependencies[index];
        for value in *low..=*high {
            assigned.push(value);
            enumerate_lineage_assignments(terms, dependencies, index + 1, assigned, extrema);
            assigned.pop().expect("oracle assignment stack must be balanced");
        }
    }

    fn lineage_termwise_oracle(terms: &[TypedErrorTerm]) -> BoundsMm {
        let mut dependencies: Vec<(String, i64, i64)> = Vec::new();
        for term in terms {
            if let Some((_, low, high)) = dependencies
                .iter()
                .find(|(id, _, _)| id.as_str() == term.dependency_id.as_str())
            {
                assert_eq!((*low, *high), (term.lower_value, term.upper_value));
            } else {
                dependencies.push((
                    term.dependency_id.clone(),
                    term.lower_value,
                    term.upper_value,
                ));
            }
        }
        let mut extrema = None;
        enumerate_lineage_assignments(
            terms,
            &dependencies,
            0,
            &mut Vec::with_capacity(dependencies.len()),
            &mut extrema,
        );
        let (lower_scaled, upper_scaled) = extrema.expect("oracle has at least one dependency");
        // Independent outward-rounding implementation using quotient/remainder
        // and Rust's truncation-towards-zero semantics.
        let lower_mm = if lower_scaled >= 0 {
            lower_scaled / SENSITIVITY_SCALE
        } else {
            -((-lower_scaled + SENSITIVITY_SCALE - 1) / SENSITIVITY_SCALE)
        };
        let upper_mm = if upper_scaled >= 0 {
            (upper_scaled + SENSITIVITY_SCALE - 1) / SENSITIVITY_SCALE
        } else {
            -((-upper_scaled) / SENSITIVITY_SCALE)
        };
        BoundsMm {
            lower_mm: i64::try_from(lower_mm).expect("small oracle bound fits i64"),
            upper_mm: i64::try_from(upper_mm).expect("small oracle bound fits i64"),
        }
    }

    fn generated_lineage_term(
        source_id: String,
        dependency_id: String,
        quantity_kind: &str,
        low: i64,
        high: i64,
        sensitivity_scaled: i64,
    ) -> TypedErrorTerm {
        let is_clock = quantity_kind == "clock_offset_error";
        TypedErrorTerm {
            source_id,
            dependency_id,
            quantity_kind: quantity_kind.to_owned(),
            unit: if is_clock { "synthetic_ms" } else { "synthetic_mm" }.to_owned(),
            reference_frame: if is_clock {
                "not_spatial"
            } else {
                OUTPUT_FRAME
            }.to_owned(),
            validity_time_basis: VALIDITY_TIME_BASIS.to_owned(),
            valid_from_s: 0,
            valid_until_s: 100,
            lower_value: low,
            upper_value: high,
            sensitivity_scaled,
            sensitivity_basis: if is_clock {
                "mm_per_ms_x1000"
            } else {
                "mm_per_mm_x1000"
            }.to_owned(),
            transform_id: None,
            transform_source_frame: None,
            transform_target_frame: None,
            sensitivity_model_id: if is_clock {
                Some("synthetic_generated_clock_sensitivity_v1".to_owned())
            } else {
                None
            },
        }
    }

    #[test]
    fn generated_typed_lineage_domains_match_termwise_assignment_oracle() {
        let mut seed = 0x71ED_2026_0230_5A11_u64;
        for case_index in 0..500 {
            let dependency_count = (next_oracle_seed(&mut seed) % 3 + 1) as usize;
            let mut terms = Vec::new();
            for dependency_index in 0..dependency_count {
                let kind = if next_oracle_seed(&mut seed) % 2 == 0 {
                    "position_error"
                } else {
                    "clock_offset_error"
                };
                let lower = -((next_oracle_seed(&mut seed) % 3) as i64);
                let upper = (next_oracle_seed(&mut seed) % 3) as i64;
                let contribution_count = (next_oracle_seed(&mut seed) % 3 + 1) as usize;
                for contribution_index in 0..contribution_count {
                    let sensitivity = (next_oracle_seed(&mut seed) % 5_001) as i64 - 2_500;
                    terms.push(generated_lineage_term(
                        format!("generated-{case_index}-source-{dependency_index}-{contribution_index}"),
                        format!("generated-{case_index}-dependency-{dependency_index}"),
                        kind,
                        lower,
                        upper,
                        sensitivity,
                    ));
                }
            }

            let model = TypedLineageModel::new(terms.clone())
                .unwrap_or_else(|error| panic!("generated lineage case {case_index} rejected: {error:?}"));
            let actual = model.bounds_at(10)
                .unwrap_or_else(|error| panic!("generated lineage case {case_index} failed: {error:?}"));
            let expected = lineage_termwise_oracle(&terms);
            assert_eq!(
                actual,
                expected,
                "typed evaluator disagreed with exhaustive termwise oracle for case {case_index}: {terms:?}"
            );

            let mut reversed = terms.clone();
            reversed.reverse();
            let permuted = TypedLineageModel::new(reversed)
                .unwrap_or_else(|error| panic!("reversed case {case_index} rejected: {error:?}"))
                .bounds_at(10)
                .unwrap_or_else(|error| panic!("reversed case {case_index} failed: {error:?}"));
            assert_eq!(
                actual,
                permuted,
                "typed output changed under term permutation for case {case_index}"
            );
        }
    }


    fn bounds_enclose(outer: BoundsMm, inner: BoundsMm) -> bool {
        outer.lower_mm <= inner.lower_mm && outer.upper_mm >= inner.upper_mm
    }

    #[test]
    fn generated_lineage_metamorphic_laws_preserve_or_expand_bounds() {
        // Independent seed from the differential-oracle test: 500 fixed, replayable domains.
        let mut seed = 0xA1B2_C3D4_2026_0250_u64;
        for case_index in 0..500 {
            let dependency_count = (next_oracle_seed(&mut seed) % 3 + 1) as usize;
            let mut terms = Vec::new();
            for dependency_index in 0..dependency_count {
                let kind = if next_oracle_seed(&mut seed) % 2 == 0 {
                    "position_error"
                } else {
                    "clock_offset_error"
                };
                let lower = -((next_oracle_seed(&mut seed) % 3) as i64);
                let upper = (next_oracle_seed(&mut seed) % 3) as i64;
                let contribution_count = (next_oracle_seed(&mut seed) % 3 + 1) as usize;
                for contribution_index in 0..contribution_count {
                    let sensitivity = (next_oracle_seed(&mut seed) % 5_001) as i64 - 2_500;
                    terms.push(generated_lineage_term(
                        format!("meta-{case_index}-source-{dependency_index}-{contribution_index}"),
                        format!("meta-{case_index}-dependency-{dependency_index}"),
                        kind,
                        lower,
                        upper,
                        sensitivity,
                    ));
                }
            }

            let baseline = TypedLineageModel::new(terms.clone())
                .unwrap_or_else(|error| panic!("metamorphic case {case_index} rejected: {error:?}"))
                .bounds_at(10)
                .unwrap_or_else(|error| panic!("metamorphic case {case_index} failed: {error:?}"));

            // Relation 1: source labels are provenance labels, not numeric coefficients.
            let relabelled: Vec<TypedErrorTerm> = terms.iter().enumerate().map(|(index, term)| {
                let mut changed = term.clone();
                changed.source_id = format!("renamed-{case_index}-{index}");
                changed
            }).collect();
            let relabelled_bounds = TypedLineageModel::new(relabelled)
                .unwrap_or_else(|error| panic!("relabelled case {case_index} rejected: {error:?}"))
                .bounds_at(10)
                .unwrap_or_else(|error| panic!("relabelled case {case_index} failed: {error:?}"));
            assert_eq!(
                baseline, relabelled_bounds,
                "numeric bounds changed after source-label relabelling in case {case_index}"
            );

            // Relation 2: splitting one linear contribution into two contributions whose
            // sensitivities sum exactly must preserve the same dependency-level function.
            let mut split = terms.clone();
            let original = split[0].clone();
            let left_sensitivity = original.sensitivity_scaled / 2;
            split[0].sensitivity_scaled = left_sensitivity;
            let mut second_half = original.clone();
            second_half.source_id = format!("split-source-{case_index}");
            second_half.sensitivity_scaled =
                original.sensitivity_scaled - left_sensitivity;
            split.push(second_half);
            let split_bounds = TypedLineageModel::new(split)
                .unwrap_or_else(|error| panic!("split case {case_index} rejected: {error:?}"))
                .bounds_at(10)
                .unwrap_or_else(|error| panic!("split case {case_index} failed: {error:?}"));
            assert_eq!(
                baseline, split_bounds,
                "equivalent coefficient split changed bounds in case {case_index}"
            );

            // Relation 3: widening one shared dependency's input interval, consistently
            // across every term that refers to it, cannot shrink the output interval.
            let widened_dependency = terms[0].dependency_id.clone();
            let mut widened = terms.clone();
            for term in &mut widened {
                if term.dependency_id == widened_dependency {
                    term.lower_value -= 1;
                    term.upper_value += 1;
                }
            }
            let widened_bounds = TypedLineageModel::new(widened)
                .unwrap_or_else(|error| panic!("widened case {case_index} rejected: {error:?}"))
                .bounds_at(10)
                .unwrap_or_else(|error| panic!("widened case {case_index} failed: {error:?}"));
            assert!(
                bounds_enclose(widened_bounds, baseline),
                "widening dependency {widened_dependency} shrank bounds in case {case_index}: baseline={baseline:?}, widened={widened_bounds:?}"
            );
        }

        // Negative control: a deliberately narrowed interval must violate containment.
        let broad = generated_lineage_term(
            "metamorphic-control-source".to_owned(),
            "metamorphic-control-dependency".to_owned(),
            "position_error",
            -2,
            2,
            1_000,
        );
        let baseline = TypedLineageModel::new(vec![broad.clone()])
            .unwrap().bounds_at(10).unwrap();
        let mut broader = broad;
        broader.lower_value = -3;
        broader.upper_value = 3;
        let widened = TypedLineageModel::new(vec![broader])
            .unwrap().bounds_at(10).unwrap();
        assert!(bounds_enclose(widened, baseline));
        let narrowed_mutant = BoundsMm {
            lower_mm: widened.lower_mm + 1,
            upper_mm: widened.upper_mm - 1,
        };
        assert!(
            !bounds_enclose(narrowed_mutant, widened),
            "the metamorphic containment control failed to detect a narrowed-output mutant"
        );
    }

}

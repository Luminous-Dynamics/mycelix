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
struct FixtureSet {
    schema: String,
    status: String,
    authority_ceiling: String,
    cases: Vec<FixtureCase>,
}

#[derive(Debug, Deserialize)]
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
}

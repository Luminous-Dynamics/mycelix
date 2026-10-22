//! Reusable, fail-closed comparability decisions for KPI definitions.
//!
//! The comparator consumes the actual `measurement` and `target` objects.
//! Display labels, local metric IDs, source-vintage strings, and caller-supplied
//! fingerprints are deliberately outside the semantic comparison.
//!
//! This module establishes only declared-contract comparability. It does not
//! validate source truth, prove two organizations implemented the same process,
//! or authorize policy changes and service dispatch.

use serde_json::Value;
use std::collections::BTreeSet;

/// Fields that define whether two observed KPI values are directly comparable.
/// Keep this list versioned by review: adding/removing a field changes semantics.
pub const MEASUREMENT_FIELDS: [&str; 11] = [
    "canonical_concept_id",
    "unit",
    "numerator_definition",
    "denominator_definition",
    "eligible_population",
    "clock_start",
    "deadline_rule",
    "category_rule",
    "geographic_scope",
    "aggregation",
    "inclusion_rule",
];

/// Fields governing whether target gaps are comparable across policy vintages.
pub const TARGET_POLICY_FIELDS: [&str; 4] = [
    "value",
    "unit",
    "target_population",
    "policy_vintage",
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasurementStatus {
    Comparable,
    NotComparable,
    Indeterminate,
}

impl MeasurementStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Comparable => "comparable",
            Self::NotComparable => "not_comparable",
            Self::Indeterminate => "indeterminate",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetPolicyStatus {
    SameTargetPolicy,
    ChangedTargetPolicy,
    Indeterminate,
}

impl TargetPolicyStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::SameTargetPolicy => "same_target_policy",
            Self::ChangedTargetPolicy => "changed_target_policy",
            Self::Indeterminate => "indeterminate",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeasurementDecision {
    pub status: MeasurementStatus,
    /// Known, non-null field values that differ, in canonical field order.
    pub changed_fields: Vec<&'static str>,
    /// Missing, null, empty, or incorrectly typed fields on either side.
    pub unknown_fields: Vec<&'static str>,
    /// Extra axis keys not recognized by this contract version; any such key forces abstention.
    pub unsupported_fields: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TargetPolicyDecision {
    pub status: TargetPolicyStatus,
    /// Known, non-null policy fields that differ, in canonical field order.
    pub changed_fields: Vec<&'static str>,
    /// Missing, null, empty, or incorrectly typed policy fields.
    pub unknown_fields: Vec<&'static str>,
    /// Extra axis keys not recognized by this contract version; any such key forces abstention.
    pub unsupported_fields: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KpiComparability {
    pub measurement: MeasurementDecision,
    pub target_policy: TargetPolicyDecision,
}

/// Returns a canonical exact-decimal tuple: sign, significant digits, and
/// base-10 exponent. It avoids f64 rounding when comparing large integer targets.
fn canonical_number(value: &Value) -> Option<(bool, String, i64)> {
    let raw = value.as_number()?.to_string();
    let (mantissa, exponent) = if let Some((mantissa, exponent)) = raw.split_once('e') {
        (mantissa, exponent.parse::<i64>().ok()?)
    } else if let Some((mantissa, exponent)) = raw.split_once('E') {
        (mantissa, exponent.parse::<i64>().ok()?)
    } else {
        (raw.as_str(), 0)
    };

    let negative = mantissa.starts_with('-');
    let unsigned = mantissa.strip_prefix('-').unwrap_or(mantissa);
    let mut parts = unsigned.split('.');
    let whole = parts.next()?;
    let fractional = parts.next().unwrap_or("");
    if parts.next().is_some() {
        return None;
    }

    let mut digits = format!("{whole}{fractional}");
    let Some(first_nonzero) = digits.find(|character| character != '0') else {
        return Some((false, "0".to_owned(), 0));
    };
    digits = digits[first_nonzero..].to_owned();
    let mut exponent = exponent.checked_sub(i64::try_from(fractional.len()).ok()?)?;
    while digits.ends_with('0') {
        digits.pop();
        exponent = exponent.checked_add(1)?;
    }
    Some((negative, digits, exponent))
}

fn values_equal(left: &Value, right: &Value) -> bool {
    if left == right {
        return true;
    }
    if left.is_number() && right.is_number() {
        return match (canonical_number(left), canonical_number(right)) {
            (Some(left), Some(right)) => left == right,
            _ => false,
        };
    }
    false
}

fn known_field<'a>(axis: Option<&'a Value>, field: &str, target_axis: bool) -> Option<&'a Value> {
    let value = axis?.get(field)?;
    if value.is_null() {
        return None;
    }

    if target_axis && field == "value" {
        if !value.as_number().and_then(|number| number.as_f64()).is_some_and(|n| n.is_finite()) {
            return None;
        }
    } else if value.as_str().is_none_or(|text| text.trim().is_empty()) {
        return None;
    }
    Some(value)
}

fn compare_fields(
    left_axis: Option<&Value>,
    right_axis: Option<&Value>,
    fields: &[&'static str],
    target_axis: bool,
) -> (Vec<&'static str>, Vec<&'static str>) {
    let mut changed = Vec::new();
    let mut unknown = Vec::new();

    for field in fields {
        match (
            known_field(left_axis, field, target_axis),
            known_field(right_axis, field, target_axis),
        ) {
            (Some(left), Some(right)) => {
                if !values_equal(left, right) {
                    changed.push(*field);
                }
            }
            _ => unknown.push(*field),
        }
    }

    (changed, unknown)
}

fn unsupported_axis_fields(
    left_axis: Option<&Value>,
    right_axis: Option<&Value>,
    allowed_fields: &[&str],
) -> Vec<String> {
    let mut unsupported = BTreeSet::new();
    for axis in [left_axis, right_axis].into_iter().flatten() {
        if let Some(object) = axis.as_object() {
            for key in object.keys() {
                if !allowed_fields.iter().any(|field| *field == key.as_str()) {
                    unsupported.insert(key.clone());
                }
            }
        }
    }
    unsupported.into_iter().collect()
}

/// Compare two KPI contracts on separate measurement and target-policy axes.
///
/// Critical string fields must be nonempty strings; target `value` must be a
/// finite JSON number. A missing, null, empty, or malformed field is unknown,
/// not a guessed value. Known mismatches take precedence over unknowns: a known
/// semantic conflict makes the pair not directly comparable even when another
/// field is missing. Unknowns with no known mismatch produce an indeterminate
/// result. The function never infers values or trusts caller-supplied fingerprints.
pub fn compare_kpi_contracts(left: &Value, right: &Value) -> KpiComparability {
    let (measurement_changed, measurement_unknown) = compare_fields(
        left.get("measurement"),
        right.get("measurement"),
        &MEASUREMENT_FIELDS,
        false,
    );
    let measurement_unsupported = unsupported_axis_fields(
        left.get("measurement"),
        right.get("measurement"),
        &MEASUREMENT_FIELDS,
    );
    let measurement_status = if !measurement_changed.is_empty() {
        MeasurementStatus::NotComparable
    } else if !measurement_unknown.is_empty() || !measurement_unsupported.is_empty() {
        MeasurementStatus::Indeterminate
    } else {
        MeasurementStatus::Comparable
    };

    let (target_changed, target_unknown) = compare_fields(
        left.get("target"),
        right.get("target"),
        &TARGET_POLICY_FIELDS,
        true,
    );
    let target_unsupported = unsupported_axis_fields(
        left.get("target"),
        right.get("target"),
        &TARGET_POLICY_FIELDS,
    );
    let target_status = if !target_changed.is_empty() {
        TargetPolicyStatus::ChangedTargetPolicy
    } else if !target_unknown.is_empty() || !target_unsupported.is_empty() {
        TargetPolicyStatus::Indeterminate
    } else {
        TargetPolicyStatus::SameTargetPolicy
    };

    KpiComparability {
        measurement: MeasurementDecision {
            status: measurement_status,
            changed_fields: measurement_changed,
            unknown_fields: measurement_unknown,
            unsupported_fields: measurement_unsupported,
        },
        target_policy: TargetPolicyDecision {
            status: target_status,
            changed_fields: target_changed,
            unknown_fields: target_unknown,
            unsupported_fields: target_unsupported,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn integer_and_decimal_encodings_of_same_target_are_equal() {
        let left = json!({"measurement": {}, "target": {"value": 90}});
        let right = json!({"measurement": {}, "target": {"value": 90.0}});
        let result = compare_kpi_contracts(&left, &right);
        assert!(!result.target_policy.changed_fields.contains(&"value"));
    }

    #[test]
    fn distinct_large_integer_targets_do_not_collapse_through_f64() {
        let left = json!({"measurement": {}, "target": {"value": 9_007_199_254_740_992_u64}});
        let right = json!({"measurement": {}, "target": {"value": 9_007_199_254_740_993_u64}});
        let result = compare_kpi_contracts(&left, &right);
        assert_eq!(result.target_policy.status, TargetPolicyStatus::ChangedTargetPolicy);
        assert!(result.target_policy.changed_fields.contains(&"value"));
    }

    #[test]
    fn malformed_types_abstain_instead_of_becoming_comparable() {
        let left = json!({"measurement": {"unit": false}, "target": {"value": "90"}});
        let right = json!({"measurement": {"unit": false}, "target": {"value": "90"}});
        let result = compare_kpi_contracts(&left, &right);
        assert_eq!(result.measurement.status, MeasurementStatus::Indeterminate);
        assert!(result.measurement.unknown_fields.contains(&"unit"));
        assert_eq!(result.target_policy.status, TargetPolicyStatus::Indeterminate);
        assert!(result.target_policy.unknown_fields.contains(&"value"));
    }

    #[test]
    fn unknown_axis_keys_abstain_until_the_contract_is_versioned() {
        let left = json!({
            "measurement": {"unit": "percent"},
            "target": {"value": 90, "comparison_operator": "at_least"}
        });
        let right = json!({
            "measurement": {"unit": "percent"},
            "target": {"value": 90, "comparison_operator": "at_most"}
        });
        let result = compare_kpi_contracts(&left, &right);
        assert_eq!(result.measurement.status, MeasurementStatus::Indeterminate);
        assert_eq!(result.target_policy.status, TargetPolicyStatus::Indeterminate);
        assert_eq!(result.target_policy.unsupported_fields, vec!["comparison_operator".to_owned()]);
    }

    #[test]
    fn missing_axes_abstain_without_panicking() {
        let result = compare_kpi_contracts(&Value::Null, &json!({}));
        assert_eq!(result.measurement.status, MeasurementStatus::Indeterminate);
        assert_eq!(result.target_policy.status, TargetPolicyStatus::Indeterminate);
        assert_eq!(result.measurement.unknown_fields.len(), MEASUREMENT_FIELDS.len());
        assert_eq!(result.target_policy.unknown_fields.len(), TARGET_POLICY_FIELDS.len());
    }
}

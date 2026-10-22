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
    /// Missing or null fields on either side, in canonical field order.
    pub unknown_fields: Vec<&'static str>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TargetPolicyDecision {
    pub status: TargetPolicyStatus,
    /// Known, non-null policy fields that differ, in canonical field order.
    pub changed_fields: Vec<&'static str>,
    /// Missing or null policy fields on either side, in canonical field order.
    pub unknown_fields: Vec<&'static str>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KpiComparability {
    pub measurement: MeasurementDecision,
    pub target_policy: TargetPolicyDecision,
}

fn values_equal(left: &Value, right: &Value) -> bool {
    if left == right {
        return true;
    }

    // JSON producers often encode mathematically identical numbers as 90 and
    // 90.0. Treat finite numeric encodings of the same value as equal, without
    // applying a tolerance that could hide a policy threshold change.
    match (left.as_f64(), right.as_f64()) {
        (Some(a), Some(b)) => a.is_finite() && b.is_finite() && a == b,
        _ => false,
    }
}

fn compare_fields(
    left_axis: Option<&Value>,
    right_axis: Option<&Value>,
    fields: &[&'static str],
) -> (Vec<&'static str>, Vec<&'static str>) {
    let mut changed = Vec::new();
    let mut unknown = Vec::new();

    for field in fields {
        match (
            left_axis.and_then(|axis| axis.get(*field)),
            right_axis.and_then(|axis| axis.get(*field)),
        ) {
            (Some(left), Some(right)) if !left.is_null() && !right.is_null() => {
                if !values_equal(left, right) {
                    changed.push(*field);
                }
            }
            _ => unknown.push(*field),
        }
    }

    (changed, unknown)
}

/// Compare two KPI contracts on separate measurement and target-policy axes.
///
/// Known mismatches take precedence over unknowns: a known semantic conflict
/// makes the pair not directly comparable even if some other critical field is
/// missing. Unknowns with no known mismatch produce an indeterminate result.
/// This function never infers values or trusts caller-provided fingerprints.
pub fn compare_kpi_contracts(left: &Value, right: &Value) -> KpiComparability {
    let (measurement_changed, measurement_unknown) = compare_fields(
        left.get("measurement"),
        right.get("measurement"),
        &MEASUREMENT_FIELDS,
    );
    let measurement_status = if !measurement_changed.is_empty() {
        MeasurementStatus::NotComparable
    } else if !measurement_unknown.is_empty() {
        MeasurementStatus::Indeterminate
    } else {
        MeasurementStatus::Comparable
    };

    let (target_changed, target_unknown) = compare_fields(
        left.get("target"),
        right.get("target"),
        &TARGET_POLICY_FIELDS,
    );
    let target_status = if !target_changed.is_empty() {
        TargetPolicyStatus::ChangedTargetPolicy
    } else if !target_unknown.is_empty() {
        TargetPolicyStatus::Indeterminate
    } else {
        TargetPolicyStatus::SameTargetPolicy
    };

    KpiComparability {
        measurement: MeasurementDecision {
            status: measurement_status,
            changed_fields: measurement_changed,
            unknown_fields: measurement_unknown,
        },
        target_policy: TargetPolicyDecision {
            status: target_status,
            changed_fields: target_changed,
            unknown_fields: target_unknown,
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
    fn missing_axes_abstain_without_panicking() {
        let result = compare_kpi_contracts(&Value::Null, &json!({}));
        assert_eq!(result.measurement.status, MeasurementStatus::Indeterminate);
        assert_eq!(result.target_policy.status, TargetPolicyStatus::Indeterminate);
        assert_eq!(result.measurement.unknown_fields.len(), MEASUREMENT_FIELDS.len());
        assert_eq!(result.target_policy.unknown_fields.len(), TARGET_POLICY_FIELDS.len());
    }
}

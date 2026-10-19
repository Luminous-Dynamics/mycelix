//! Explicit intersection algebra for numeric requirements that are known to
//! apply conjunctively to the same building outcome.
//!
//! This module does not decide whether two legal clauses are conjunctive,
//! whether one jurisdiction outranks another, or how text should be normalized.
//! A trusted, reviewed rule adapter must resolve source applicability,
//! translate units to the declared canonical quantity kind, and only pass
//! assertions whose relationship is genuinely conjunctive. Unresolved legal
//! alternatives return conflicts; source applicability other than Applicable
//! blocks the composition call.
//!
//! Numeric values use signed integer millionths of the named canonical SI unit
//! (for example, LengthMetres + 3_500_000 means 3.5 m). Integer arithmetic
//! avoids floating-point boundary disagreements. Unit conversion and its
//! precision policy belong to the upstream, versioned rule adapter.

use crate::regulatory::SourceApplicability;
use std::collections::{BTreeMap, HashSet};

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum CanonicalQuantityKind {
    LengthMetres,
    AreaSquareMetres,
    VolumeCubicMetres,
    DurationSeconds,
    AirflowCubicMetresPerSecond,
    PressurePascals,
    TemperatureKelvin,
    PowerWatts,
    EnergyJoules,
    Ratio,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NumericConstraintOperator {
    AtLeast,
    GreaterThan,
    AtMost,
    LessThan,
    Exactly,
}

/// Whether the upstream legal/engineering interpreter established that the
/// assertions apply cumulatively. Unresolved or disjunctive pathways must not
/// be intersected as if every bound were simultaneously mandatory.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NumericConstraintRelationship {
    Conjunctive,
    Unresolved,
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct NumericConstraintSource {
    pub module_id: String,
    pub statement_id: String,
    pub clause_ref: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NumericConstraintAssertion {
    pub requirement_id: String,
    pub quantity_kind: CanonicalQuantityKind,
    pub operator: NumericConstraintOperator,
    pub value_micro_units: i64,
    pub relationship: NumericConstraintRelationship,
    /// Must already have been evaluated for the project's jurisdiction and
    /// assessment date by the time-aware regulatory module.
    pub source_applicability: SourceApplicability,
    pub source: NumericConstraintSource,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NumericRangeBound {
    pub value_micro_units: i64,
    pub inclusive: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectiveNumericConstraint {
    pub requirement_id: String,
    pub quantity_kind: CanonicalQuantityKind,
    pub lower_bound: Option<NumericRangeBound>,
    pub upper_bound: Option<NumericRangeBound>,
    pub exact_value_micro_units: Option<i64>,
    /// The source assertions from which this intersection was constructed.
    pub contributing_sources: Vec<NumericConstraintSource>,
    /// Sources that determine the final lower/upper boundaries. Multiple
    /// agreeing sources are retained rather than collapsed.
    pub lower_bound_sources: Vec<NumericConstraintSource>,
    pub upper_bound_sources: Vec<NumericConstraintSource>,
    pub exact_value_sources: Vec<NumericConstraintSource>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NumericConflictReason {
    MixedQuantityKinds,
    RelationshipNotEstablishedConjunctive,
    MultipleExactValues,
    EmptyIntersection,
    ExactValueOutsideBounds,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NumericConstraintConflict {
    pub requirement_id: String,
    pub reason: NumericConflictReason,
    /// Original assertions are retained intact for an expert to resolve.
    pub assertions: Vec<NumericConstraintAssertion>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NumericConstraintIssue {
    EmptyRequirementId,
    EmptySourceField {
        statement_id: String,
        field: &'static str,
    },
    DuplicateStatementId(String),
    SourceNotApplicable {
        statement_id: String,
        source_applicability: SourceApplicability,
    },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NumericConstraintCompositionReport {
    pub effective_constraints: Vec<EffectiveNumericConstraint>,
    pub conflicts: Vec<NumericConstraintConflict>,
}

/// Intersect explicitly conjunctive numeric requirements by requirement ID.
///
/// A stronger lower bound is the greatest lower endpoint; at equal values an
/// exclusive endpoint is stronger than an inclusive endpoint. The upper bound
/// is dual. Exact values must agree and satisfy all bounds. Different quantity
/// kinds, unresolved relationships, contradictory exact values, and empty
/// intersections remain explicit conflicts; they are never repaired by
/// inventing legal precedence or choosing a convenient value.
pub fn compose_conjunctive_numeric_constraints(
    assertions: &[NumericConstraintAssertion],
) -> Result<NumericConstraintCompositionReport, Vec<NumericConstraintIssue>> {
    let mut issues = Vec::new();
    let mut statement_ids = HashSet::new();
    let mut grouped: BTreeMap<String, Vec<NumericConstraintAssertion>> = BTreeMap::new();

    for assertion in assertions {
        if assertion.requirement_id.trim().is_empty() {
            issues.push(NumericConstraintIssue::EmptyRequirementId);
        }
        if assertion.source_applicability != SourceApplicability::Applicable {
            issues.push(NumericConstraintIssue::SourceNotApplicable {
                statement_id: assertion.source.statement_id.clone(),
                source_applicability: assertion.source_applicability.clone(),
            });
        }
        for (field, value) in [
            ("module_id", assertion.source.module_id.as_str()),
            ("statement_id", assertion.source.statement_id.as_str()),
            ("clause_ref", assertion.source.clause_ref.as_str()),
        ] {
            if value.trim().is_empty() {
                issues.push(NumericConstraintIssue::EmptySourceField {
                    statement_id: assertion.source.statement_id.clone(),
                    field,
                });
            }
        }
        if !assertion.source.statement_id.trim().is_empty()
            && !statement_ids.insert(assertion.source.statement_id.as_str())
        {
            issues.push(NumericConstraintIssue::DuplicateStatementId(
                assertion.source.statement_id.clone(),
            ));
        }
        if !assertion.requirement_id.trim().is_empty() {
            grouped
                .entry(assertion.requirement_id.clone())
                .or_default()
                .push(assertion.clone());
        }
    }
    if !issues.is_empty() {
        return Err(issues);
    }

    let mut effective_constraints = Vec::new();
    let mut conflicts = Vec::new();
    for (requirement_id, mut group) in grouped {
        group.sort_by(|a, b| {
            (&a.source.module_id, &a.source.statement_id)
                .cmp(&(&b.source.module_id, &b.source.statement_id))
        });

        let first_kind = group[0].quantity_kind;
        if group.iter().any(|item| item.quantity_kind != first_kind) {
            conflicts.push(NumericConstraintConflict {
                requirement_id,
                reason: NumericConflictReason::MixedQuantityKinds,
                assertions: group,
            });
            continue;
        }
        if group
            .iter()
            .any(|item| item.relationship != NumericConstraintRelationship::Conjunctive)
        {
            conflicts.push(NumericConstraintConflict {
                requirement_id,
                reason: NumericConflictReason::RelationshipNotEstablishedConjunctive,
                assertions: group,
            });
            continue;
        }

        let mut lower: Option<NumericRangeBound> = None;
        let mut upper: Option<NumericRangeBound> = None;
        let mut exacts: BTreeMap<i64, Vec<NumericConstraintSource>> = BTreeMap::new();
        let mut lower_sources: Vec<NumericConstraintSource> = Vec::new();
        let mut upper_sources: Vec<NumericConstraintSource> = Vec::new();

        for item in &group {
            match item.operator {
                NumericConstraintOperator::AtLeast | NumericConstraintOperator::GreaterThan => {
                    let candidate = NumericRangeBound {
                        value_micro_units: item.value_micro_units,
                        inclusive: item.operator == NumericConstraintOperator::AtLeast,
                    };
                    match lower {
                        None => {
                            lower = Some(candidate);
                            lower_sources = vec![item.source.clone()];
                        }
                        Some(current) if is_stronger_lower(candidate, current) => {
                            lower = Some(candidate);
                            lower_sources = vec![item.source.clone()];
                        }
                        Some(current)
                            if candidate.value_micro_units == current.value_micro_units
                                && candidate.inclusive == current.inclusive =>
                        {
                            lower_sources.push(item.source.clone());
                        }
                        _ => {}
                    }
                }
                NumericConstraintOperator::AtMost | NumericConstraintOperator::LessThan => {
                    let candidate = NumericRangeBound {
                        value_micro_units: item.value_micro_units,
                        inclusive: item.operator == NumericConstraintOperator::AtMost,
                    };
                    match upper {
                        None => {
                            upper = Some(candidate);
                            upper_sources = vec![item.source.clone()];
                        }
                        Some(current) if is_stronger_upper(candidate, current) => {
                            upper = Some(candidate);
                            upper_sources = vec![item.source.clone()];
                        }
                        Some(current)
                            if candidate.value_micro_units == current.value_micro_units
                                && candidate.inclusive == current.inclusive =>
                        {
                            upper_sources.push(item.source.clone());
                        }
                        _ => {}
                    }
                }
                NumericConstraintOperator::Exactly => {
                    exacts
                        .entry(item.value_micro_units)
                        .or_default()
                        .push(item.source.clone());
                }
            }
        }

        let mut all_sources: Vec<_> = group.iter().map(|item| item.source.clone()).collect();
        all_sources.sort();
        lower_sources.sort();
        upper_sources.sort();

        if exacts.len() > 1 {
            conflicts.push(NumericConstraintConflict {
                requirement_id,
                reason: NumericConflictReason::MultipleExactValues,
                assertions: group,
            });
            continue;
        }

        let exact = exacts.keys().next().copied();
        let exact_sources = exacts.into_values().next().unwrap_or_default();
        if range_is_empty(lower, upper) {
            conflicts.push(NumericConstraintConflict {
                requirement_id,
                reason: NumericConflictReason::EmptyIntersection,
                assertions: group,
            });
            continue;
        }
        if let Some(value) = exact {
            if !value_satisfies(value, lower, upper) {
                conflicts.push(NumericConstraintConflict {
                    requirement_id,
                    reason: NumericConflictReason::ExactValueOutsideBounds,
                    assertions: group,
                });
                continue;
            }
        }

        effective_constraints.push(EffectiveNumericConstraint {
            requirement_id,
            quantity_kind: first_kind,
            lower_bound: lower,
            upper_bound: upper,
            exact_value_micro_units: exact,
            contributing_sources: all_sources,
            lower_bound_sources: lower_sources,
            upper_bound_sources: upper_sources,
            exact_value_sources: exact_sources,
        });
    }

    Ok(NumericConstraintCompositionReport {
        effective_constraints,
        conflicts,
    })
}

fn is_stronger_lower(candidate: NumericRangeBound, current: NumericRangeBound) -> bool {
    candidate.value_micro_units > current.value_micro_units
        || (candidate.value_micro_units == current.value_micro_units
            && !candidate.inclusive
            && current.inclusive)
}

fn is_stronger_upper(candidate: NumericRangeBound, current: NumericRangeBound) -> bool {
    candidate.value_micro_units < current.value_micro_units
        || (candidate.value_micro_units == current.value_micro_units
            && !candidate.inclusive
            && current.inclusive)
}

fn range_is_empty(
    lower: Option<NumericRangeBound>,
    upper: Option<NumericRangeBound>,
) -> bool {
    match (lower, upper) {
        (Some(lower), Some(upper)) => {
            lower.value_micro_units > upper.value_micro_units
                || (lower.value_micro_units == upper.value_micro_units
                    && (!lower.inclusive || !upper.inclusive))
        }
        _ => false,
    }
}

fn value_satisfies(
    value: i64,
    lower: Option<NumericRangeBound>,
    upper: Option<NumericRangeBound>,
) -> bool {
    let lower_ok = lower.is_none_or(|bound| {
        value > bound.value_micro_units
            || (value == bound.value_micro_units && bound.inclusive)
    });
    let upper_ok = upper.is_none_or(|bound| {
        value < bound.value_micro_units
            || (value == bound.value_micro_units && bound.inclusive)
    });
    lower_ok && upper_ok
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regulatory::NonBindingReason;

    fn assertion(
        module_id: &str,
        statement_id: &str,
        requirement_id: &str,
        quantity_kind: CanonicalQuantityKind,
        operator: NumericConstraintOperator,
        value_micro_units: i64,
    ) -> NumericConstraintAssertion {
        NumericConstraintAssertion {
            requirement_id: requirement_id.into(),
            quantity_kind,
            operator,
            value_micro_units,
            relationship: NumericConstraintRelationship::Conjunctive,
            source_applicability: SourceApplicability::Applicable,
            source: NumericConstraintSource {
                module_id: module_id.into(),
                statement_id: statement_id.into(),
                clause_ref: format!("clause:{statement_id}"),
            },
        }
    }

    #[test]
    fn compatible_minimums_compose_to_the_strongest_bound() {
        let assertions = vec![
            assertion("national", "n:length", "door.clear_width", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::AtLeast, 800_000),
            assertion("municipal", "m:length", "door.clear_width", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::AtLeast, 900_000),
        ];
        let result = compose_conjunctive_numeric_constraints(&assertions).unwrap();
        assert!(result.conflicts.is_empty());
        let effective = &result.effective_constraints[0];
        assert_eq!(effective.lower_bound.unwrap().value_micro_units, 900_000);
        assert_eq!(effective.lower_bound_sources[0].statement_id, "m:length");
        assert_eq!(effective.contributing_sources.len(), 2);
    }

    #[test]
    fn compatible_upper_bounds_compose_to_the_strongest_upper_limit() {
        let assertions = vec![
            assertion("a", "a:max", "travel.distance", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::AtMost, 30_000_000),
            assertion("b", "b:max", "travel.distance", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::AtMost, 25_000_000),
        ];
        let result = compose_conjunctive_numeric_constraints(&assertions).unwrap();
        assert_eq!(result.effective_constraints[0].upper_bound.unwrap().value_micro_units, 25_000_000);
    }

    #[test]
    fn strict_endpoint_wins_when_values_are_equal() {
        let assertions = vec![
            assertion("a", "a:min", "clearance", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::AtLeast, 1_000_000),
            assertion("b", "b:min", "clearance", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::GreaterThan, 1_000_000),
        ];
        let result = compose_conjunctive_numeric_constraints(&assertions).unwrap();
        let lower = result.effective_constraints[0].lower_bound.unwrap();
        assert_eq!(lower.value_micro_units, 1_000_000);
        assert!(!lower.inclusive);
    }

    #[test]
    fn disjoint_ranges_remain_a_conflict() {
        let assertions = vec![
            assertion("a", "a:min", "opening.width", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::AtLeast, 1_000_000),
            assertion("b", "b:max", "opening.width", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::AtMost, 900_000),
        ];
        let result = compose_conjunctive_numeric_constraints(&assertions).unwrap();
        assert!(result.effective_constraints.is_empty());
        assert_eq!(result.conflicts[0].reason, NumericConflictReason::EmptyIntersection);
        assert_eq!(result.conflicts[0].assertions.len(), 2);
    }

    #[test]
    fn exclusive_equal_endpoints_make_an_empty_range() {
        let assertions = vec![
            assertion("a", "a:min", "clearance", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::GreaterThan, 1_000_000),
            assertion("b", "b:max", "clearance", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::AtMost, 1_000_000),
        ];
        let result = compose_conjunctive_numeric_constraints(&assertions).unwrap();
        assert_eq!(result.conflicts[0].reason, NumericConflictReason::EmptyIntersection);
    }

    #[test]
    fn exact_value_must_satisfy_every_bound() {
        let assertions = vec![
            assertion("a", "a:exact", "opening.width", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::Exactly, 900_000),
            assertion("b", "b:min", "opening.width", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::AtLeast, 1_000_000),
        ];
        let result = compose_conjunctive_numeric_constraints(&assertions).unwrap();
        assert_eq!(result.conflicts[0].reason, NumericConflictReason::ExactValueOutsideBounds);
    }

    #[test]
    fn distinct_exact_values_conflict_and_preserve_sources() {
        let assertions = vec![
            assertion("a", "a:exact", "room.height", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::Exactly, 2_400_000),
            assertion("b", "b:exact", "room.height", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::Exactly, 2_600_000),
        ];
        let result = compose_conjunctive_numeric_constraints(&assertions).unwrap();
        assert_eq!(result.conflicts[0].reason, NumericConflictReason::MultipleExactValues);
        assert_eq!(result.conflicts[0].assertions.len(), 2);
    }

    #[test]
    fn different_quantity_dimensions_never_compare_as_numbers() {
        let assertions = vec![
            assertion("a", "a:value", "same-id", CanonicalQuantityKind::LengthMetres, NumericConstraintOperator::AtLeast, 2_000_000),
            assertion("b", "b:value", "same-id", CanonicalQuantityKind::AreaSquareMetres, NumericConstraintOperator::AtLeast, 2_000_000),
        ];
        let result = compose_conjunctive_numeric_constraints(&assertions).unwrap();
        assert_eq!(result.conflicts[0].reason, NumericConflictReason::MixedQuantityKinds);
    }

    #[test]
    fn unresolved_legal_or_alternative_relationship_is_not_intersected() {
        let mut item = assertion("a", "a:rule", "ventilation", CanonicalQuantityKind::AirflowCubicMetresPerSecond, NumericConstraintOperator::AtLeast, 500_000);
        item.relationship = NumericConstraintRelationship::Unresolved;
        let result = compose_conjunctive_numeric_constraints(&[item]).unwrap();
        assert_eq!(
            result.conflicts[0].reason,
            NumericConflictReason::RelationshipNotEstablishedConjunctive
        );
    }

    #[test]
    fn non_applicable_sources_block_numeric_composition() {
        let mut item = assertion("a", "a:proposal", "ventilation", CanonicalQuantityKind::AirflowCubicMetresPerSecond, NumericConstraintOperator::AtLeast, 500_000);
        item.source_applicability =
            SourceApplicability::NonBinding(NonBindingReason::Proposal);
        let issues = compose_conjunctive_numeric_constraints(&[item]).unwrap_err();
        assert!(matches!(
            issues.as_slice(),
            [NumericConstraintIssue::SourceNotApplicable { .. }]
        ));
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-010 explicit institutional robustness envelopes.
//!
//! AC-010 preserves analytical uncertainty as an explicit set of defensible
//! assumptions and scenarios. It exposes exact bounds across the supplied
//! specification set without collapsing uncertainty into a scalar robustness
//! score or pretending an exploratory set is exhaustive.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::capture_observation::{
    CaptureContract, CaptureContractViolation, CaptureMetric, CaptureObservation, CaptureSubject,
    MetricValue, ProvenanceRef,
};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum RobustnessDimension {
    IdentityReconciliation,
    ProcurementWeighting,
    RegistryCompleteness,
    EvidenceCorroboration,
    TimeWindow,
    MethodChoice,
    Custom(String),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct RobustnessAssumption {
    pub id: String,
    pub dimension: RobustnessDimension,
    pub statement: String,
    pub admissibility_ref: String,
    pub provenance: Vec<ProvenanceRef>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum RobustnessScenarioKind {
    Baseline,
    SingleDimension,
    Joint,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct RobustnessScenario {
    pub id: String,
    pub kind: RobustnessScenarioKind,
    pub assumption_refs: Vec<String>,
    pub observation: CaptureObservation,
}

/// AC-010 never infers exhaustive coverage from scenario count.
///
/// `EnumeratedWithinDeclaredScope` is an evidence-bearing *claim about a declared
/// scope*. The caller remains responsible for proving that the scope definition
/// and its enumeration are authoritative enough for the intended use.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum RobustnessCoverage {
    Exploratory {
        limitation: String,
    },
    EnumeratedWithinDeclaredScope {
        scope_ref: String,
        provenance: Vec<ProvenanceRef>,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct RobustnessBounds {
    pub minimum: MetricValue,
    pub minimum_scenario_refs: Vec<String>,
    pub maximum: MetricValue,
    pub maximum_scenario_refs: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct InstitutionalRobustnessEnvelope {
    pub id: String,
    pub subject: CaptureSubject,
    pub metric: CaptureMetric,
    pub unit: String,
    pub baseline_scenario_ref: String,
    pub assumptions: Vec<RobustnessAssumption>,
    pub scenarios: Vec<RobustnessScenario>,
    pub coverage: RobustnessCoverage,
    pub bounds: RobustnessBounds,
    pub invariant_across_supplied_scenarios: bool,
    pub method_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum RobustnessEnvelopeError {
    MissingEnvelopeId,
    TooFewScenarios,
    InvalidAssumption,
    DuplicateAssumptionId,
    InvalidScenario,
    DuplicateScenarioId,
    InvalidObservation {
        index: usize,
        violations: Vec<CaptureContractViolation>,
    },
    MissingBaseline,
    MultipleBaselines,
    BaselineHasAssumptions,
    NonBaselineMissingAssumptions,
    NonCanonicalAssumptionRefs,
    UnknownAssumptionRef,
    InvalidScenarioDimensionShape,
    DuplicateSpecification,
    UnusedAssumption,
    HiddenMethodChange,
    SubjectMismatch,
    MetricMismatch,
    UnitMismatch,
    InvalidCoverage,
    ArithmeticOverflow,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct InstitutionalRobustnessContract;

impl InstitutionalRobustnessContract {
    pub fn build(
        envelope_id: &str,
        assumptions: &[RobustnessAssumption],
        scenarios: &[RobustnessScenario],
        coverage: RobustnessCoverage,
    ) -> Result<InstitutionalRobustnessEnvelope, Vec<RobustnessEnvelopeError>> {
        let mut errors = Vec::new();

        if envelope_id.trim().is_empty() {
            errors.push(RobustnessEnvelopeError::MissingEnvelopeId);
        }
        if scenarios.len() < 2 {
            errors.push(RobustnessEnvelopeError::TooFewScenarios);
        }
        if !coverage_valid(&coverage) {
            errors.push(RobustnessEnvelopeError::InvalidCoverage);
        }

        let mut assumption_map = BTreeMap::new();
        for assumption in assumptions {
            if !assumption_valid(assumption) {
                errors.push(RobustnessEnvelopeError::InvalidAssumption);
            }
            if assumption_map
                .insert(assumption.id.clone(), assumption.clone())
                .is_some()
            {
                errors.push(RobustnessEnvelopeError::DuplicateAssumptionId);
            }
        }

        let mut scenario_ids = BTreeSet::new();
        let mut baselines = Vec::new();
        let mut used_assumptions = BTreeSet::new();
        let mut specifications = BTreeSet::new();

        for (index, scenario) in scenarios.iter().enumerate() {
            if scenario.id.trim().is_empty() {
                errors.push(RobustnessEnvelopeError::InvalidScenario);
            }
            if !scenario_ids.insert(scenario.id.clone()) {
                errors.push(RobustnessEnvelopeError::DuplicateScenarioId);
            }
            if let Err(violations) = CaptureContract::validate_observation(&scenario.observation) {
                errors.push(RobustnessEnvelopeError::InvalidObservation { index, violations });
            }
            if !sorted_unique(&scenario.assumption_refs) {
                errors.push(RobustnessEnvelopeError::NonCanonicalAssumptionRefs);
            }

            match scenario.kind {
                RobustnessScenarioKind::Baseline => {
                    baselines.push(scenario.id.clone());
                    if !scenario.assumption_refs.is_empty() {
                        errors.push(RobustnessEnvelopeError::BaselineHasAssumptions);
                    }
                }
                _ if scenario.assumption_refs.is_empty() => {
                    errors.push(RobustnessEnvelopeError::NonBaselineMissingAssumptions);
                }
                _ => {}
            }

            let mut dimensions = BTreeSet::new();
            for reference in &scenario.assumption_refs {
                match assumption_map.get(reference) {
                    Some(assumption) => {
                        used_assumptions.insert(reference.clone());
                        dimensions.insert(assumption.dimension.clone());
                    }
                    None => errors.push(RobustnessEnvelopeError::UnknownAssumptionRef),
                }
            }

            if matches!(scenario.kind, RobustnessScenarioKind::SingleDimension)
                && dimensions.len() != 1
            {
                errors.push(RobustnessEnvelopeError::InvalidScenarioDimensionShape);
            }
            if matches!(scenario.kind, RobustnessScenarioKind::Joint) && dimensions.len() < 2 {
                errors.push(RobustnessEnvelopeError::InvalidScenarioDimensionShape);
            }

            if !specifications.insert(scenario.assumption_refs.clone()) {
                errors.push(RobustnessEnvelopeError::DuplicateSpecification);
            }
        }

        match baselines.len() {
            0 => errors.push(RobustnessEnvelopeError::MissingBaseline),
            1 => {}
            _ => errors.push(RobustnessEnvelopeError::MultipleBaselines),
        }

        if assumption_map
            .keys()
            .any(|id| !used_assumptions.contains(id))
        {
            errors.push(RobustnessEnvelopeError::UnusedAssumption);
        }

        if !errors.is_empty() {
            return Err(errors);
        }

        let baseline_ref = baselines[0].clone();
        let baseline = scenarios
            .iter()
            .find(|scenario| scenario.id == baseline_ref)
            .expect("validated baseline");
        let subject = baseline.observation.subject.clone();
        let metric = baseline.observation.measurement.metric.clone();
        let unit = baseline.observation.measurement.value.unit.clone();
        let baseline_method = baseline.observation.measurement.method_ref.as_str();

        for scenario in scenarios {
            if scenario.observation.subject != subject {
                errors.push(RobustnessEnvelopeError::SubjectMismatch);
            }
            if scenario.observation.measurement.metric != metric {
                errors.push(RobustnessEnvelopeError::MetricMismatch);
            }
            if scenario.observation.measurement.value.unit != unit {
                errors.push(RobustnessEnvelopeError::UnitMismatch);
            }

            if scenario.observation.measurement.method_ref != baseline_method
                && !scenario_has_dimension(
                    scenario,
                    &assumption_map,
                    &RobustnessDimension::MethodChoice,
                )
            {
                errors.push(RobustnessEnvelopeError::HiddenMethodChange);
            }
        }

        if !errors.is_empty() {
            return Err(errors);
        }

        let bounds = exact_bounds(scenarios)?;
        let mut invariant = true;
        for scenario in scenarios {
            if compare_metric_values(
                &baseline.observation.measurement.value,
                &scenario.observation.measurement.value,
            )? != Ordering::Equal
            {
                invariant = false;
                break;
            }
        }

        let mut assumptions_out: Vec<_> = assumption_map.into_values().collect();
        assumptions_out.sort_by(|left, right| left.id.cmp(&right.id));
        let mut scenarios_out = scenarios.to_vec();
        scenarios_out.sort_by(|left, right| left.id.cmp(&right.id));

        Ok(InstitutionalRobustnessEnvelope {
            id: envelope_id.into(),
            subject,
            metric,
            unit,
            baseline_scenario_ref: baseline_ref,
            assumptions: assumptions_out,
            scenarios: scenarios_out,
            coverage,
            bounds,
            invariant_across_supplied_scenarios: invariant,
            method_ref: "mycelix:ac-010:institutional-robustness-envelope:v1".into(),
        })
    }
}

fn assumption_valid(assumption: &RobustnessAssumption) -> bool {
    !assumption.id.trim().is_empty()
        && !assumption.statement.trim().is_empty()
        && !assumption.admissibility_ref.trim().is_empty()
        && !matches!(
            &assumption.dimension,
            RobustnessDimension::Custom(value) if value.trim().is_empty()
        )
        && !assumption.provenance.is_empty()
        && assumption.provenance.iter().all(strict_provenance)
}

fn coverage_valid(coverage: &RobustnessCoverage) -> bool {
    match coverage {
        RobustnessCoverage::Exploratory { limitation } => !limitation.trim().is_empty(),
        RobustnessCoverage::EnumeratedWithinDeclaredScope {
            scope_ref,
            provenance,
        } => {
            !scope_ref.trim().is_empty()
                && !provenance.is_empty()
                && provenance.iter().all(strict_provenance)
        }
    }
}

fn strict_provenance(provenance: &ProvenanceRef) -> bool {
    !provenance.source_ref.trim().is_empty()
        && provenance
            .content_hash
            .as_deref()
            .is_some_and(|hash| !hash.trim().is_empty())
}

fn sorted_unique(values: &[String]) -> bool {
    values.iter().all(|value| !value.trim().is_empty())
        && values
            .windows(2)
            .all(|pair| pair[0].as_str() < pair[1].as_str())
}

fn scenario_has_dimension(
    scenario: &RobustnessScenario,
    assumptions: &BTreeMap<String, RobustnessAssumption>,
    dimension: &RobustnessDimension,
) -> bool {
    scenario.assumption_refs.iter().any(|reference| {
        assumptions
            .get(reference)
            .is_some_and(|assumption| &assumption.dimension == dimension)
    })
}

fn compare_metric_values(
    left: &MetricValue,
    right: &MetricValue,
) -> Result<Ordering, Vec<RobustnessEnvelopeError>> {
    let left_cross = i128::from(left.numerator)
        .checked_mul(i128::from(right.denominator))
        .ok_or_else(|| vec![RobustnessEnvelopeError::ArithmeticOverflow])?;
    let right_cross = i128::from(right.numerator)
        .checked_mul(i128::from(left.denominator))
        .ok_or_else(|| vec![RobustnessEnvelopeError::ArithmeticOverflow])?;
    Ok(left_cross.cmp(&right_cross))
}

fn exact_bounds(
    scenarios: &[RobustnessScenario],
) -> Result<RobustnessBounds, Vec<RobustnessEnvelopeError>> {
    let mut minimum = scenarios[0].observation.measurement.value.clone();
    let mut maximum = minimum.clone();
    let mut minimum_refs = vec![scenarios[0].id.clone()];
    let mut maximum_refs = minimum_refs.clone();

    for scenario in scenarios.iter().skip(1) {
        let value = &scenario.observation.measurement.value;
        match compare_metric_values(value, &minimum)? {
            Ordering::Less => {
                minimum = value.clone();
                minimum_refs = vec![scenario.id.clone()];
            }
            Ordering::Equal => minimum_refs.push(scenario.id.clone()),
            Ordering::Greater => {}
        }
        match compare_metric_values(value, &maximum)? {
            Ordering::Greater => {
                maximum = value.clone();
                maximum_refs = vec![scenario.id.clone()];
            }
            Ordering::Equal => maximum_refs.push(scenario.id.clone()),
            Ordering::Less => {}
        }
    }

    minimum_refs.sort();
    maximum_refs.sort();
    Ok(RobustnessBounds {
        minimum,
        minimum_scenario_refs: minimum_refs,
        maximum,
        maximum_scenario_refs: maximum_refs,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture_observation::{
        ConfidenceAssessment, ConfidenceLevel, Measurement, Uncertainty,
    };

    fn provenance(id: &str) -> ProvenanceRef {
        ProvenanceRef {
            source_ref: format!("source:{id}"),
            content_hash: Some(format!("sha256:{id}")),
        }
    }

    fn assumption(id: &str, dimension: RobustnessDimension) -> RobustnessAssumption {
        RobustnessAssumption {
            id: id.into(),
            dimension,
            statement: format!("vary {id}"),
            admissibility_ref: format!("rule:{id}"),
            provenance: vec![provenance(id)],
        }
    }

    fn observation(id: &str, numerator: i64, denominator: u64, method: &str) -> CaptureObservation {
        CaptureObservation {
            id: format!("obs:{id}"),
            subject: CaptureSubject::ContractingProcedure("procurement".into()),
            measurement: Measurement {
                metric: CaptureMetric::ProcurementSupplierConcentration,
                value: MetricValue {
                    numerator,
                    denominator,
                    unit: "hhi_ratio".into(),
                },
                method_ref: method.into(),
            },
            provenance: vec![provenance(id)],
            uncertainty: Uncertainty {
                confidence: ConfidenceAssessment::Qualitative {
                    level: ConfidenceLevel::Moderate,
                    basis: "fixture".into(),
                },
                limitations: vec!["fixture".into()],
            },
            observed_at: 1,
        }
    }

    fn baseline(numerator: i64, denominator: u64) -> RobustnessScenario {
        RobustnessScenario {
            id: "baseline".into(),
            kind: RobustnessScenarioKind::Baseline,
            assumption_refs: vec![],
            observation: observation("base", numerator, denominator, "method:v1"),
        }
    }

    #[test]
    fn equivalent_fractions_are_invariant() {
        let assumptions = vec![assumption(
            "identity",
            RobustnessDimension::IdentityReconciliation,
        )];
        let scenarios = vec![
            baseline(1, 2),
            RobustnessScenario {
                id: "alt".into(),
                kind: RobustnessScenarioKind::SingleDimension,
                assumption_refs: vec!["identity".into()],
                observation: observation("alt", 2, 4, "method:v1"),
            },
        ];
        let envelope = InstitutionalRobustnessContract::build(
            "envelope",
            &assumptions,
            &scenarios,
            RobustnessCoverage::Exploratory {
                limitation: "one dimension".into(),
            },
        )
        .unwrap();
        assert!(envelope.invariant_across_supplied_scenarios);
    }

    #[test]
    fn exact_bounds_are_preserved() {
        let assumptions = vec![
            assumption("identity", RobustnessDimension::IdentityReconciliation),
            assumption("window", RobustnessDimension::TimeWindow),
        ];
        let scenarios = vec![
            baseline(1, 2),
            RobustnessScenario {
                id: "high".into(),
                kind: RobustnessScenarioKind::SingleDimension,
                assumption_refs: vec!["identity".into()],
                observation: observation("high", 2, 3, "method:v1"),
            },
            RobustnessScenario {
                id: "low".into(),
                kind: RobustnessScenarioKind::SingleDimension,
                assumption_refs: vec!["window".into()],
                observation: observation("low", 1, 3, "method:v1"),
            },
        ];
        let envelope = InstitutionalRobustnessContract::build(
            "envelope",
            &assumptions,
            &scenarios,
            RobustnessCoverage::Exploratory {
                limitation: "two dimensions".into(),
            },
        )
        .unwrap();
        assert_eq!(envelope.bounds.minimum_scenario_refs, vec!["low"]);
        assert_eq!(envelope.bounds.maximum_scenario_refs, vec!["high"]);
    }

    #[test]
    fn duplicate_specification_fails_closed() {
        let assumptions = vec![assumption(
            "identity",
            RobustnessDimension::IdentityReconciliation,
        )];
        let scenarios = vec![
            baseline(1, 2),
            RobustnessScenario {
                id: "x".into(),
                kind: RobustnessScenarioKind::SingleDimension,
                assumption_refs: vec!["identity".into()],
                observation: observation("x", 2, 3, "method:v1"),
            },
            RobustnessScenario {
                id: "y".into(),
                kind: RobustnessScenarioKind::SingleDimension,
                assumption_refs: vec!["identity".into()],
                observation: observation("y", 3, 4, "method:v1"),
            },
        ];
        assert!(InstitutionalRobustnessContract::build(
            "envelope",
            &assumptions,
            &scenarios,
            RobustnessCoverage::Exploratory {
                limitation: "fixture".into(),
            },
        )
        .unwrap_err()
        .contains(&RobustnessEnvelopeError::DuplicateSpecification));
    }

    #[test]
    fn hidden_method_change_fails_closed() {
        let assumptions = vec![assumption(
            "identity",
            RobustnessDimension::IdentityReconciliation,
        )];
        let scenarios = vec![
            baseline(1, 2),
            RobustnessScenario {
                id: "alt".into(),
                kind: RobustnessScenarioKind::SingleDimension,
                assumption_refs: vec!["identity".into()],
                observation: observation("alt", 2, 3, "method:v2"),
            },
        ];
        assert!(InstitutionalRobustnessContract::build(
            "envelope",
            &assumptions,
            &scenarios,
            RobustnessCoverage::Exploratory {
                limitation: "fixture".into(),
            },
        )
        .unwrap_err()
        .contains(&RobustnessEnvelopeError::HiddenMethodChange));
    }

    #[test]
    fn declared_method_choice_allows_method_change() {
        let assumptions = vec![assumption("method", RobustnessDimension::MethodChoice)];
        let scenarios = vec![
            baseline(1, 2),
            RobustnessScenario {
                id: "alt".into(),
                kind: RobustnessScenarioKind::SingleDimension,
                assumption_refs: vec!["method".into()],
                observation: observation("alt", 2, 3, "method:v2"),
            },
        ];
        assert!(InstitutionalRobustnessContract::build(
            "envelope",
            &assumptions,
            &scenarios,
            RobustnessCoverage::Exploratory {
                limitation: "fixture".into(),
            },
        )
        .is_ok());
    }
}

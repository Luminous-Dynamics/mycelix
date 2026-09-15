// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-013 procurement weighting robustness bridge.
//!
//! AC-013 compares AC-004 equal-award-count supplier concentration with AC-012
//! value-weighted supplier concentration over the exact same award-edge population.
//! The value specification explicitly declares both a ProcurementWeighting and a
//! MethodChoice assumption before AC-010 is allowed to compare the observations.

use serde::{Deserialize, Serialize};

use crate::capture_metrics::{
    CaptureMetricEngine, MetricComputationError, ObservationContext,
};
use crate::institutional_graph::InstitutionalEdge;
use crate::institutional_robustness::{
    InstitutionalRobustnessContract, InstitutionalRobustnessEnvelope, RobustnessAssumption,
    RobustnessCoverage, RobustnessDimension, RobustnessEnvelopeError, RobustnessScenario,
    RobustnessScenarioKind,
};
use crate::procurement_robustness::ProcurementAssumptionDescriptor;
use crate::procurement_value::{
    ProcurementAwardValueSet, ProcurementValueContract, ProcurementValueError,
};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProcurementWeightingRobustnessResult {
    pub envelope: InstitutionalRobustnessEnvelope,
    pub count_input_edge_refs: Vec<String>,
    pub value_input_edge_refs: Vec<String>,
    pub value_snapshot_ref: String,
    pub value_record_refs: Vec<String>,
    pub bridge_method_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ProcurementWeightingError {
    MissingEnvelopeId,
    DuplicateAssumptionId,
    CountMetric(MetricComputationError),
    ValueMetric(Vec<ProcurementValueError>),
    InputEdgeLineageMismatch,
    RobustnessEnvelope(Vec<RobustnessEnvelopeError>),
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ProcurementWeightingRobustnessContract;

impl ProcurementWeightingRobustnessContract {
    pub fn compare_count_and_value(
        envelope_id: &str,
        edges: &[InstitutionalEdge],
        values: &ProcurementAwardValueSet,
        context: ObservationContext,
        weighting_assumption: ProcurementAssumptionDescriptor,
        method_assumption: ProcurementAssumptionDescriptor,
        coverage: RobustnessCoverage,
    ) -> Result<ProcurementWeightingRobustnessResult, Vec<ProcurementWeightingError>> {
        if envelope_id.trim().is_empty() {
            return Err(vec![ProcurementWeightingError::MissingEnvelopeId]);
        }
        if weighting_assumption.id == method_assumption.id {
            return Err(vec![ProcurementWeightingError::DuplicateAssumptionId]);
        }

        let mut count_context = context.clone();
        count_context.observation_id = stable_key(
            "ac-013-count-observation",
            &[envelope_id, context.observation_id.as_str()],
        );
        count_context.limitations.push(
            "baseline supplier concentration weights each award relationship equally".into(),
        );
        let count = CaptureMetricEngine::procurement_supplier_concentration(
            edges,
            count_context,
        )
        .map_err(|error| vec![ProcurementWeightingError::CountMetric(error)])?;

        let mut value_context = context;
        value_context.observation_id = stable_key(
            "ac-013-value-observation",
            &[envelope_id, value_context.observation_id.as_str()],
        );
        value_context.limitations.push(
            "alternative supplier concentration uses AC-012 qualified supplier-attributed award-value weighting"
                .into(),
        );
        let value = ProcurementValueContract::value_weighted_supplier_concentration(
            edges,
            values,
            value_context,
        )
        .map_err(|errors| vec![ProcurementWeightingError::ValueMetric(errors)])?;

        if count.input_edge_refs != value.derived.input_edge_refs {
            return Err(vec![ProcurementWeightingError::InputEdgeLineageMismatch]);
        }

        let assumptions = vec![
            to_assumption(
                weighting_assumption,
                RobustnessDimension::ProcurementWeighting,
            ),
            to_assumption(method_assumption, RobustnessDimension::MethodChoice),
        ];

        let baseline_ref = stable_key("ac-013-count-scenario", &[envelope_id]);
        let value_ref = stable_key("ac-013-value-scenario", &[envelope_id]);
        let mut alternative_assumption_refs: Vec<_> = assumptions
            .iter()
            .map(|assumption| assumption.id.clone())
            .collect();
        alternative_assumption_refs.sort();

        let scenarios = vec![
            RobustnessScenario {
                id: baseline_ref,
                kind: RobustnessScenarioKind::Baseline,
                assumption_refs: vec![],
                observation: count.observation,
            },
            RobustnessScenario {
                id: value_ref,
                kind: RobustnessScenarioKind::Joint,
                assumption_refs: alternative_assumption_refs,
                observation: value.derived.observation,
            },
        ];

        let envelope = InstitutionalRobustnessContract::build(
            envelope_id,
            &assumptions,
            &scenarios,
            coverage,
        )
        .map_err(|errors| vec![ProcurementWeightingError::RobustnessEnvelope(errors)])?;

        Ok(ProcurementWeightingRobustnessResult {
            envelope,
            count_input_edge_refs: count.input_edge_refs,
            value_input_edge_refs: value.derived.input_edge_refs,
            value_snapshot_ref: value.value_snapshot_ref,
            value_record_refs: value.value_record_refs,
            bridge_method_ref: "mycelix:ac-013:procurement-weighting-robustness-bridge:v1".into(),
        })
    }
}

fn to_assumption(
    descriptor: ProcurementAssumptionDescriptor,
    dimension: RobustnessDimension,
) -> RobustnessAssumption {
    RobustnessAssumption {
        id: descriptor.id,
        dimension,
        statement: descriptor.statement,
        admissibility_ref: descriptor.admissibility_ref,
        provenance: descriptor.provenance,
    }
}

fn stable_key(namespace: &str, parts: &[&str]) -> String {
    let mut output = format!("{namespace}|");
    for part in parts {
        output.push_str(&format!("{}:{}|", part.len(), part));
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture_observation::{
        CaptureSubject, ConfidenceAssessment, ConfidenceLevel, ProvenanceRef,
    };
    use crate::institutional_graph::{
        AssertionStatus, DisclosureClass, InstitutionalNodeKind, InstitutionalNodeRef,
        InstitutionalRelationKind, ProcurementRole,
    };
    use crate::procurement_value::{ExactCurrencyAmount, SupplierAttributedAwardValue};

    fn provenance(id: &str) -> ProvenanceRef {
        ProvenanceRef {
            source_ref: format!("source:{id}"),
            content_hash: Some(format!("sha256:{id}")),
        }
    }

    fn node(id: &str, kind: InstitutionalNodeKind) -> InstitutionalNodeRef {
        InstitutionalNodeRef {
            id: id.into(),
            kind,
        }
    }

    fn award(id: &str, supplier: &str) -> InstitutionalEdge {
        InstitutionalEdge {
            id: id.into(),
            from: node(supplier, InstitutionalNodeKind::Organization),
            to: node("procedure", InstitutionalNodeKind::ProcurementProcedure),
            relation: InstitutionalRelationKind::ProcurementParticipation {
                role: ProcurementRole::Awardee,
            },
            disclosure: DisclosureClass::PublicMetadata,
            assertion_status: AssertionStatus::Declared,
            provenance: vec![provenance(id)],
            challenge_refs: vec![],
            recorded_at: 10,
            valid_from: None,
            valid_until: None,
        }
    }

    fn value_record(edge: &str, amount: u64) -> SupplierAttributedAwardValue {
        SupplierAttributedAwardValue {
            value_ref: format!("value:{edge}"),
            award_edge_ref: edge.into(),
            award_ref: format!("award:{edge}"),
            amount: ExactCurrencyAmount {
                coefficient: amount,
                scale: 0,
                currency: "USD".into(),
            },
            provenance: vec![provenance(&format!("value:{edge}"))],
        }
    }

    fn values() -> ProcurementAwardValueSet {
        ProcurementAwardValueSet {
            snapshot_ref: "snapshot:values".into(),
            value_semantics_ref: "ocds:1.1.5:award.value".into(),
            currency_registry_ref: "iso4217:2015".into(),
            currency_registry_provenance: vec![provenance("iso4217")],
            records: vec![value_record("a", 100), value_record("b", 300)],
        }
    }

    fn descriptor(id: &str) -> ProcurementAssumptionDescriptor {
        ProcurementAssumptionDescriptor {
            id: id.into(),
            statement: format!("assumption {id}"),
            admissibility_ref: format!("rule:{id}"),
            provenance: vec![provenance(id)],
        }
    }

    fn context() -> ObservationContext {
        ObservationContext {
            observation_id: "comparison".into(),
            subject: CaptureSubject::ContractingProcedure("procedure".into()),
            observed_at: 100,
            confidence: ConfidenceAssessment::Qualitative {
                level: ConfidenceLevel::Moderate,
                basis: "fixture".into(),
            },
            limitations: vec!["fixture".into()],
        }
    }

    #[test]
    fn count_and_value_specs_share_exact_edge_lineage() {
        let edges = vec![award("a", "supplier-a"), award("b", "supplier-b")];
        let result = ProcurementWeightingRobustnessContract::compare_count_and_value(
            "weighting-envelope",
            &edges,
            &values(),
            context(),
            descriptor("weighting"),
            descriptor("method"),
            RobustnessCoverage::Exploratory {
                limitation: "two explicit weighting specifications".into(),
            },
        )
        .unwrap();
        assert_eq!(result.count_input_edge_refs, result.value_input_edge_refs);
        assert_eq!(result.envelope.scenarios.len(), 2);
    }

    #[test]
    fn value_spec_explicitly_declares_weighting_and_method_choice() {
        let edges = vec![award("a", "supplier-a"), award("b", "supplier-b")];
        let result = ProcurementWeightingRobustnessContract::compare_count_and_value(
            "weighting-envelope",
            &edges,
            &values(),
            context(),
            descriptor("weighting"),
            descriptor("method"),
            RobustnessCoverage::Exploratory {
                limitation: "fixture".into(),
            },
        )
        .unwrap();
        let alternative = result
            .envelope
            .scenarios
            .iter()
            .find(|scenario| !scenario.assumption_refs.is_empty())
            .unwrap();
        assert_eq!(alternative.assumption_refs, vec!["method", "weighting"]);
        assert!(matches!(alternative.kind, RobustnessScenarioKind::Joint));
    }

    #[test]
    fn weighting_changes_exact_hhi_without_changing_metric_family() {
        let edges = vec![award("a", "supplier-a"), award("b", "supplier-b")];
        let result = ProcurementWeightingRobustnessContract::compare_count_and_value(
            "weighting-envelope",
            &edges,
            &values(),
            context(),
            descriptor("weighting"),
            descriptor("method"),
            RobustnessCoverage::Exploratory {
                limitation: "fixture".into(),
            },
        )
        .unwrap();
        let baseline = result
            .envelope
            .scenarios
            .iter()
            .find(|scenario| scenario.assumption_refs.is_empty())
            .unwrap();
        let alternative = result
            .envelope
            .scenarios
            .iter()
            .find(|scenario| !scenario.assumption_refs.is_empty())
            .unwrap();
        assert_eq!(baseline.observation.measurement.value.numerator, 2);
        assert_eq!(baseline.observation.measurement.value.denominator, 4);
        assert_eq!(alternative.observation.measurement.value.numerator, 100_000);
        assert_eq!(alternative.observation.measurement.value.denominator, 160_000);
        assert_eq!(baseline.observation.measurement.metric, alternative.observation.measurement.metric);
    }

    #[test]
    fn assumption_ids_must_be_distinct() {
        let edges = vec![award("a", "supplier-a"), award("b", "supplier-b")];
        assert_eq!(
            ProcurementWeightingRobustnessContract::compare_count_and_value(
                "weighting-envelope",
                &edges,
                &values(),
                context(),
                descriptor("same"),
                descriptor("same"),
                RobustnessCoverage::Exploratory {
                    limitation: "fixture".into(),
                },
            )
            .unwrap_err(),
            vec![ProcurementWeightingError::DuplicateAssumptionId]
        );
    }
}

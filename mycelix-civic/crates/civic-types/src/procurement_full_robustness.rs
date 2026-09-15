// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-014 full procurement robustness composition.
//!
//! AC-014 composes the qualified identity, evidence, time-window, and weighting
//! axes into one explicit specification matrix. It never mutates AC-003 source
//! edges and never hides a method change behind a weighting label.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use crate::capture_metrics::{CaptureMetricEngine, MetricComputationError, ObservationContext};
use crate::equivalence_view::{EquivalenceComponent, QualifiedEquivalenceView};
use crate::identity_qualification::{
    IdentityQualificationContract, IdentityQualificationError, IdentityQualificationVerifier,
    QualifiedIdentityLinkInput,
};
use crate::institutional_graph::{
    AssertionStatus, InstitutionalEdge, InstitutionalNodeKind, InstitutionalRelationKind,
    ProcurementRole,
};
use crate::institutional_robustness::{
    InstitutionalRobustnessContract, InstitutionalRobustnessEnvelope, RobustnessAssumption,
    RobustnessCoverage, RobustnessDimension, RobustnessEnvelopeError, RobustnessScenario,
    RobustnessScenarioKind,
};
use crate::procurement_robustness::{
    ProcurementAssumptionDescriptor, ProcurementRobustnessPlan, ProcurementTimeWindow,
    MAX_PROCUREMENT_TIME_WINDOWS,
};
use crate::procurement_value::{
    ProcurementAwardValueSet, ProcurementValueContract, ProcurementValueError,
};

pub const MAX_FULL_PROCUREMENT_SCENARIOS: usize = 128;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProcurementWeightingAssumptions {
    pub weighting: ProcurementAssumptionDescriptor,
    pub method_choice: ProcurementAssumptionDescriptor,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FullProcurementRobustnessPlan {
    pub base: ProcurementRobustnessPlan,
    pub value_weighting: Option<ProcurementWeightingAssumptions>,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcurementWeightingMode {
    AwardCount,
    QualifiedAwardValue,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FullProcurementScenarioLineage {
    pub scenario_ref: String,
    pub weighting_mode: ProcurementWeightingMode,
    pub input_edge_refs: Vec<String>,
    pub applied_equivalence_component_refs: Vec<String>,
    pub applied_identity_link_refs: Vec<String>,
    pub value_snapshot_ref: Option<String>,
    pub value_record_refs: Vec<String>,
    pub metric_method_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FullProcurementRobustnessMatrix {
    pub envelope: InstitutionalRobustnessEnvelope,
    pub scenario_lineage: Vec<FullProcurementScenarioLineage>,
    pub qualification_receipt_refs: Vec<String>,
    pub matrix_method_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum FullProcurementRobustnessError {
    InvalidPlan,
    NoPerturbationAxis,
    TooManyTimeWindows,
    TooManyScenarios,
    InvalidTimeWindow { window_index: usize },
    DuplicateAssumptionId { assumption_id: String },
    IdentityConfigurationMismatch,
    MissingIdentityPolicyReference,
    WeightingConfigurationMismatch,
    IdentityQualification(Vec<IdentityQualificationError>),
    ValuePreflight(Vec<ProcurementValueError>),
    ScenarioCountMetric {
        scenario_ref: String,
        error: MetricComputationError,
    },
    ScenarioValueMetric {
        scenario_ref: String,
        errors: Vec<ProcurementValueError>,
    },
    InputEdgeLineageMismatch { scenario_ref: String },
    RobustnessEnvelope(Vec<RobustnessEnvelopeError>),
}

#[derive(Debug, Default, Clone, Copy)]
pub struct FullProcurementRobustnessContract;

impl FullProcurementRobustnessContract {
    pub fn build<V: IdentityQualificationVerifier>(
        edges: &[InstitutionalEdge],
        identity_inputs: &[QualifiedIdentityLinkInput],
        required_identity_policy_ref: Option<&str>,
        values: Option<&ProcurementAwardValueSet>,
        plan: FullProcurementRobustnessPlan,
        coverage: RobustnessCoverage,
        verifier: &V,
    ) -> Result<FullProcurementRobustnessMatrix, Vec<FullProcurementRobustnessError>> {
        validate_plan(
            &plan,
            identity_inputs,
            required_identity_policy_ref,
            values,
        )?;

        let qualified_view = if identity_inputs.is_empty() {
            None
        } else {
            Some(
                IdentityQualificationContract::build_equivalence_view(
                    identity_inputs,
                    required_identity_policy_ref.expect("validated identity policy"),
                    plan.base.evaluated_at,
                    verifier,
                )
                .map_err(|errors| {
                    vec![FullProcurementRobustnessError::IdentityQualification(errors)]
                })?,
            )
        };

        if let Some(values) = values {
            let context = ObservationContext {
                observation_id: format!("{}:value-preflight", plan.base.matrix_id),
                subject: plan.base.subject.clone(),
                observed_at: plan.base.evaluated_at,
                confidence: plan.base.confidence.clone(),
                limitations: plan.base.limitations.clone(),
            };
            ProcurementValueContract::value_weighted_supplier_concentration(edges, values, context)
                .map_err(|errors| vec![FullProcurementRobustnessError::ValuePreflight(errors)])?;
        }

        let assumptions = build_assumptions(&plan);
        let identity_options = if qualified_view.is_some() {
            vec![false, true]
        } else {
            vec![false]
        };
        let evidence_options = if plan.base.corroborated_only.is_some() {
            vec![false, true]
        } else {
            vec![false]
        };
        let mut window_options: Vec<Option<&ProcurementTimeWindow>> = vec![None];
        window_options.extend(plan.base.time_windows.iter().map(Some));
        let weighting_options = if plan.value_weighting.is_some() {
            vec![
                ProcurementWeightingMode::AwardCount,
                ProcurementWeightingMode::QualifiedAwardValue,
            ]
        } else {
            vec![ProcurementWeightingMode::AwardCount]
        };

        let mut scenarios = Vec::new();
        let mut scenario_lineage = Vec::new();

        for resolved_identity in identity_options {
            for corroborated_only in &evidence_options {
                for window in &window_options {
                    for weighting_mode in &weighting_options {
                        let mut assumption_refs = structural_assumption_refs(
                            &plan,
                            resolved_identity,
                            *corroborated_only,
                            *window,
                        );
                        if *weighting_mode == ProcurementWeightingMode::QualifiedAwardValue {
                            let weighting = plan
                                .value_weighting
                                .as_ref()
                                .expect("validated weighting assumptions");
                            assumption_refs.push(weighting.weighting.id.clone());
                            assumption_refs.push(weighting.method_choice.id.clone());
                        }
                        assumption_refs.sort();

                        let scenario_ref = scenario_key(&plan.base.matrix_id, &assumption_refs);
                        let selected = select_edges(edges, *corroborated_only, *window);
                        let mut limitations = plan.base.limitations.clone();
                        if *corroborated_only {
                            limitations.push(
                                "scenario admits only AC-003 corroborated assertions".into(),
                            );
                        }
                        if let Some(window) = window {
                            limitations.push(format!(
                                "scenario admits recorded_at in [{}, {})",
                                window.start_recorded_at, window.end_recorded_at
                            ));
                        }

                        let (metric_edges, applied_components, applied_links) =
                            if resolved_identity {
                                limitations.push(
                                    "supplier identity regrouping uses one AC-008-qualified equivalence snapshot shared by the entire matrix"
                                        .into(),
                                );
                                project_edges_through_view(
                                    &selected,
                                    qualified_view
                                        .as_ref()
                                        .expect("validated qualified identity view"),
                                )
                            } else {
                                (selected, vec![], vec![])
                            };

                        let context = ObservationContext {
                            observation_id: format!("{}:observation", scenario_ref),
                            subject: plan.base.subject.clone(),
                            observed_at: plan.base.evaluated_at,
                            confidence: plan.base.confidence.clone(),
                            limitations,
                        };

                        let (observation, input_edge_refs, value_snapshot_ref, value_record_refs) =
                            match weighting_mode {
                                ProcurementWeightingMode::AwardCount => {
                                    let derived = CaptureMetricEngine::procurement_supplier_concentration(
                                        &metric_edges,
                                        context,
                                    )
                                    .map_err(|error| {
                                        vec![FullProcurementRobustnessError::ScenarioCountMetric {
                                            scenario_ref: scenario_ref.clone(),
                                            error,
                                        }]
                                    })?;
                                    (
                                        derived.observation,
                                        derived.input_edge_refs,
                                        None,
                                        vec![],
                                    )
                                }
                                ProcurementWeightingMode::QualifiedAwardValue => {
                                    let count_check = CaptureMetricEngine::procurement_supplier_concentration(
                                        &metric_edges,
                                        context.clone(),
                                    )
                                    .map_err(|error| {
                                        vec![FullProcurementRobustnessError::ScenarioCountMetric {
                                            scenario_ref: scenario_ref.clone(),
                                            error,
                                        }]
                                    })?;
                                    let subset = subset_values(
                                        values.expect("validated value set"),
                                        &award_edge_ids(&metric_edges),
                                    );
                                    let weighted = ProcurementValueContract::value_weighted_supplier_concentration(
                                        &metric_edges,
                                        &subset,
                                        context,
                                    )
                                    .map_err(|errors| {
                                        vec![FullProcurementRobustnessError::ScenarioValueMetric {
                                            scenario_ref: scenario_ref.clone(),
                                            errors,
                                        }]
                                    })?;
                                    if count_check.input_edge_refs != weighted.derived.input_edge_refs {
                                        return Err(vec![
                                            FullProcurementRobustnessError::InputEdgeLineageMismatch {
                                                scenario_ref,
                                            },
                                        ]);
                                    }
                                    (
                                        weighted.derived.observation,
                                        weighted.derived.input_edge_refs,
                                        Some(weighted.value_snapshot_ref),
                                        weighted.value_record_refs,
                                    )
                                }
                            };

                        let method_ref = observation.measurement.method_ref.clone();
                        let kind = scenario_kind(
                            resolved_identity,
                            *corroborated_only,
                            window.is_some(),
                            *weighting_mode,
                        );
                        scenarios.push(RobustnessScenario {
                            id: scenario_ref.clone(),
                            kind,
                            assumption_refs,
                            observation,
                        });
                        scenario_lineage.push(FullProcurementScenarioLineage {
                            scenario_ref,
                            weighting_mode: *weighting_mode,
                            input_edge_refs,
                            applied_equivalence_component_refs: applied_components,
                            applied_identity_link_refs: applied_links,
                            value_snapshot_ref,
                            value_record_refs,
                            metric_method_ref: method_ref,
                        });
                    }
                }
            }
        }

        scenarios.sort_by(|left, right| left.id.cmp(&right.id));
        scenario_lineage.sort_by(|left, right| left.scenario_ref.cmp(&right.scenario_ref));
        let envelope = InstitutionalRobustnessContract::build(
            &plan.base.matrix_id,
            &assumptions,
            &scenarios,
            coverage,
        )
        .map_err(|errors| {
            vec![FullProcurementRobustnessError::RobustnessEnvelope(errors)]
        })?;

        let mut qualification_receipt_refs: Vec<_> = identity_inputs
            .iter()
            .map(|input| input.receipt.receipt_ref.clone())
            .collect();
        qualification_receipt_refs.sort();
        qualification_receipt_refs.dedup();

        Ok(FullProcurementRobustnessMatrix {
            envelope,
            scenario_lineage,
            qualification_receipt_refs,
            matrix_method_ref: "mycelix:ac-014:full-procurement-robustness-matrix:v1".into(),
        })
    }
}

fn validate_plan(
    plan: &FullProcurementRobustnessPlan,
    identity_inputs: &[QualifiedIdentityLinkInput],
    required_identity_policy_ref: Option<&str>,
    values: Option<&ProcurementAwardValueSet>,
) -> Result<(), Vec<FullProcurementRobustnessError>> {
    let mut errors = Vec::new();
    let base = &plan.base;
    if base.matrix_id.trim().is_empty()
        || base.limitations.is_empty()
        || base.limitations.iter().any(|item| item.trim().is_empty())
        || !matches!(
            &base.subject,
            crate::capture_observation::CaptureSubject::ContractingProcedure(_)
        )
    {
        errors.push(FullProcurementRobustnessError::InvalidPlan);
    }
    if base.time_windows.len() > MAX_PROCUREMENT_TIME_WINDOWS {
        errors.push(FullProcurementRobustnessError::TooManyTimeWindows);
    }

    let identity_enabled = base.identity_resolution.is_some();
    if identity_enabled != !identity_inputs.is_empty() {
        errors.push(FullProcurementRobustnessError::IdentityConfigurationMismatch);
    }
    if identity_enabled
        && !required_identity_policy_ref.is_some_and(|reference| !reference.trim().is_empty())
    {
        errors.push(FullProcurementRobustnessError::MissingIdentityPolicyReference);
    }

    let weighting_enabled = plan.value_weighting.is_some();
    if weighting_enabled != values.is_some() {
        errors.push(FullProcurementRobustnessError::WeightingConfigurationMismatch);
    }
    if !identity_enabled
        && base.corroborated_only.is_none()
        && base.time_windows.is_empty()
        && !weighting_enabled
    {
        errors.push(FullProcurementRobustnessError::NoPerturbationAxis);
    }

    let mut ids = BTreeSet::new();
    for descriptor in base
        .identity_resolution
        .iter()
        .chain(base.corroborated_only.iter())
    {
        validate_descriptor(descriptor, &mut ids, &mut errors);
    }
    for (window_index, window) in base.time_windows.iter().enumerate() {
        validate_descriptor(&window.descriptor, &mut ids, &mut errors);
        if window.start_recorded_at >= window.end_recorded_at {
            errors.push(FullProcurementRobustnessError::InvalidTimeWindow { window_index });
        }
    }
    if let Some(weighting) = &plan.value_weighting {
        validate_descriptor(&weighting.weighting, &mut ids, &mut errors);
        validate_descriptor(&weighting.method_choice, &mut ids, &mut errors);
    }

    let scenario_count = (if identity_enabled { 2 } else { 1 })
        * (if base.corroborated_only.is_some() { 2 } else { 1 })
        * (1 + base.time_windows.len())
        * (if weighting_enabled { 2 } else { 1 });
    if scenario_count > MAX_FULL_PROCUREMENT_SCENARIOS {
        errors.push(FullProcurementRobustnessError::TooManyScenarios);
    }

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

fn validate_descriptor(
    descriptor: &ProcurementAssumptionDescriptor,
    seen: &mut BTreeSet<String>,
    errors: &mut Vec<FullProcurementRobustnessError>,
) {
    if descriptor.id.trim().is_empty()
        || descriptor.statement.trim().is_empty()
        || descriptor.admissibility_ref.trim().is_empty()
        || descriptor.provenance.is_empty()
        || descriptor.provenance.iter().any(|provenance| {
            provenance.source_ref.trim().is_empty()
                || provenance
                    .content_hash
                    .as_deref()
                    .is_none_or(|hash| hash.trim().is_empty())
        })
    {
        errors.push(FullProcurementRobustnessError::InvalidPlan);
    }
    if !seen.insert(descriptor.id.clone()) {
        errors.push(FullProcurementRobustnessError::DuplicateAssumptionId {
            assumption_id: descriptor.id.clone(),
        });
    }
}

fn build_assumptions(plan: &FullProcurementRobustnessPlan) -> Vec<RobustnessAssumption> {
    let mut assumptions = Vec::new();
    if let Some(descriptor) = &plan.base.identity_resolution {
        assumptions.push(to_assumption(
            descriptor,
            RobustnessDimension::IdentityReconciliation,
        ));
    }
    if let Some(descriptor) = &plan.base.corroborated_only {
        assumptions.push(to_assumption(
            descriptor,
            RobustnessDimension::EvidenceCorroboration,
        ));
    }
    for window in &plan.base.time_windows {
        assumptions.push(to_assumption(
            &window.descriptor,
            RobustnessDimension::TimeWindow,
        ));
    }
    if let Some(weighting) = &plan.value_weighting {
        assumptions.push(to_assumption(
            &weighting.weighting,
            RobustnessDimension::ProcurementWeighting,
        ));
        assumptions.push(to_assumption(
            &weighting.method_choice,
            RobustnessDimension::MethodChoice,
        ));
    }
    assumptions.sort_by(|left, right| left.id.cmp(&right.id));
    assumptions
}

fn to_assumption(
    descriptor: &ProcurementAssumptionDescriptor,
    dimension: RobustnessDimension,
) -> RobustnessAssumption {
    RobustnessAssumption {
        id: descriptor.id.clone(),
        dimension,
        statement: descriptor.statement.clone(),
        admissibility_ref: descriptor.admissibility_ref.clone(),
        provenance: descriptor.provenance.clone(),
    }
}

fn structural_assumption_refs(
    plan: &FullProcurementRobustnessPlan,
    resolved_identity: bool,
    corroborated_only: bool,
    window: Option<&ProcurementTimeWindow>,
) -> Vec<String> {
    let mut refs = Vec::new();
    if resolved_identity {
        refs.push(
            plan.base
                .identity_resolution
                .as_ref()
                .expect("validated identity descriptor")
                .id
                .clone(),
        );
    }
    if corroborated_only {
        refs.push(
            plan.base
                .corroborated_only
                .as_ref()
                .expect("validated evidence descriptor")
                .id
                .clone(),
        );
    }
    if let Some(window) = window {
        refs.push(window.descriptor.id.clone());
    }
    refs
}

fn select_edges(
    edges: &[InstitutionalEdge],
    corroborated_only: bool,
    window: Option<&ProcurementTimeWindow>,
) -> Vec<InstitutionalEdge> {
    edges
        .iter()
        .filter(|edge| !corroborated_only || edge.assertion_status == AssertionStatus::Corroborated)
        .filter(|edge| {
            window.is_none_or(|window| {
                edge.recorded_at >= window.start_recorded_at
                    && edge.recorded_at < window.end_recorded_at
            })
        })
        .cloned()
        .collect()
}

fn project_edges_through_view(
    edges: &[InstitutionalEdge],
    view: &QualifiedEquivalenceView,
) -> (Vec<InstitutionalEdge>, Vec<String>, Vec<String>) {
    let award_actors: BTreeSet<_> = edges
        .iter()
        .filter(|edge| {
            matches!(
                &edge.relation,
                InstitutionalRelationKind::ProcurementParticipation {
                    role: ProcurementRole::Awardee
                }
            )
        })
        .map(|edge| edge.from.id.clone())
        .collect();
    let active_components: Vec<&EquivalenceComponent> = view
        .components
        .iter()
        .filter(|component| {
            component
                .members
                .iter()
                .filter(|member| award_actors.contains(*member))
                .count()
                >= 2
        })
        .collect();
    let active_ids: BTreeSet<_> = active_components
        .iter()
        .map(|component| component.id.clone())
        .collect();

    let mut projected = edges.to_vec();
    for edge in &mut projected {
        if !matches!(
            &edge.relation,
            InstitutionalRelationKind::ProcurementParticipation {
                role: ProcurementRole::Awardee
            }
        ) {
            continue;
        }
        let Some(component_id) = view.node_to_component.get(&edge.from.id) else {
            continue;
        };
        if active_ids.contains(component_id) {
            edge.from.id = component_id.clone();
            edge.from.kind = InstitutionalNodeKind::Aggregate;
        }
    }

    let mut component_refs: Vec<_> = active_components
        .iter()
        .map(|component| component.id.clone())
        .collect();
    component_refs.sort();
    let link_refs = active_components
        .iter()
        .flat_map(|component| component.identity_link_refs.iter().cloned())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    (projected, component_refs, link_refs)
}

fn award_edge_ids(edges: &[InstitutionalEdge]) -> BTreeSet<String> {
    edges
        .iter()
        .filter(|edge| {
            matches!(
                &edge.relation,
                InstitutionalRelationKind::ProcurementParticipation {
                    role: ProcurementRole::Awardee
                }
            )
        })
        .map(|edge| edge.id.clone())
        .collect()
}

fn subset_values(
    values: &ProcurementAwardValueSet,
    selected_award_edges: &BTreeSet<String>,
) -> ProcurementAwardValueSet {
    ProcurementAwardValueSet {
        snapshot_ref: values.snapshot_ref.clone(),
        value_semantics_ref: values.value_semantics_ref.clone(),
        currency_registry_ref: values.currency_registry_ref.clone(),
        currency_registry_provenance: values.currency_registry_provenance.clone(),
        records: values
            .records
            .iter()
            .filter(|record| selected_award_edges.contains(&record.award_edge_ref))
            .cloned()
            .collect(),
    }
}

fn scenario_kind(
    identity: bool,
    evidence: bool,
    window: bool,
    weighting: ProcurementWeightingMode,
) -> RobustnessScenarioKind {
    let dimensions = usize::from(identity)
        + usize::from(evidence)
        + usize::from(window)
        + if weighting == ProcurementWeightingMode::QualifiedAwardValue {
            2
        } else {
            0
        };
    match dimensions {
        0 => RobustnessScenarioKind::Baseline,
        1 => RobustnessScenarioKind::SingleDimension,
        _ => RobustnessScenarioKind::Joint,
    }
}

fn scenario_key(matrix_id: &str, assumption_refs: &[String]) -> String {
    if assumption_refs.is_empty() {
        return format!("ac-014:{matrix_id}:baseline");
    }
    let encoded = assumption_refs
        .iter()
        .map(|reference| format!("{}:{reference}", reference.len()))
        .collect::<Vec<_>>()
        .join("|");
    format!("ac-014:{matrix_id}:{encoded}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture_observation::{
        CaptureSubject, ConfidenceAssessment, ConfidenceLevel, ProvenanceRef,
    };
    use crate::identity_qualification::{
        IdentityQualificationReceipt, QualificationVerifierFailure,
    };
    use crate::identity_resolution::{
        EntityBindingEvidence, EntityIdentityLink, IdentityLinkStatus, IdentityVerification,
        IdentityVerificationKind,
    };
    use crate::institutional_graph::{
        AssertionStatus, DisclosureClass, InstitutionalNodeKind, InstitutionalNodeRef,
    };
    use crate::procurement_value::{ExactCurrencyAmount, SupplierAttributedAwardValue};
    use crate::standards_ingestion::ExternalStandard;
    use std::cell::Cell;

    fn prov(id: &str) -> ProvenanceRef {
        ProvenanceRef {
            source_ref: format!("source:{id}"),
            content_hash: Some(format!("sha256:{id}")),
        }
    }

    fn node(id: &str) -> InstitutionalNodeRef {
        InstitutionalNodeRef {
            id: id.into(),
            kind: InstitutionalNodeKind::Organization,
        }
    }

    fn award(
        id: &str,
        supplier: &str,
        recorded_at: u64,
        status: AssertionStatus,
    ) -> InstitutionalEdge {
        let provenance = if status == AssertionStatus::Corroborated {
            vec![prov(&format!("{id}:a")), prov(&format!("{id}:b"))]
        } else {
            vec![prov(id)]
        };
        InstitutionalEdge {
            id: id.into(),
            from: node(supplier),
            to: InstitutionalNodeRef {
                id: "procedure".into(),
                kind: InstitutionalNodeKind::ProcurementProcedure,
            },
            relation: InstitutionalRelationKind::ProcurementParticipation {
                role: ProcurementRole::Awardee,
            },
            disclosure: DisclosureClass::PublicMetadata,
            assertion_status: status,
            provenance,
            challenge_refs: vec![],
            recorded_at,
            valid_from: None,
            valid_until: None,
        }
    }

    fn descriptor(id: &str) -> ProcurementAssumptionDescriptor {
        ProcurementAssumptionDescriptor {
            id: id.into(),
            statement: format!("vary {id}"),
            admissibility_ref: format!("rule:{id}"),
            provenance: vec![prov(id)],
        }
    }

    fn base_plan() -> ProcurementRobustnessPlan {
        ProcurementRobustnessPlan {
            matrix_id: "matrix:test".into(),
            subject: CaptureSubject::ContractingProcedure("procedure".into()),
            evaluated_at: 100,
            confidence: ConfidenceAssessment::Qualitative {
                level: ConfidenceLevel::Moderate,
                basis: "fixture".into(),
            },
            limitations: vec!["fixture".into()],
            identity_resolution: None,
            corroborated_only: Some(descriptor("evidence")),
            time_windows: vec![ProcurementTimeWindow {
                descriptor: descriptor("window"),
                start_recorded_at: 0,
                end_recorded_at: 20,
            }],
        }
    }

    fn values() -> ProcurementAwardValueSet {
        ProcurementAwardValueSet {
            snapshot_ref: "values:snapshot".into(),
            value_semantics_ref: "ocds:award.value".into(),
            currency_registry_ref: "iso4217".into(),
            currency_registry_provenance: vec![prov("iso4217")],
            records: vec![
                SupplierAttributedAwardValue {
                    value_ref: "value:a".into(),
                    award_edge_ref: "a".into(),
                    award_ref: "award:a".into(),
                    amount: ExactCurrencyAmount {
                        coefficient: 100,
                        scale: 0,
                        currency: "USD".into(),
                    },
                    provenance: vec![prov("value:a")],
                },
                SupplierAttributedAwardValue {
                    value_ref: "value:b".into(),
                    award_edge_ref: "b".into(),
                    award_ref: "award:b".into(),
                    amount: ExactCurrencyAmount {
                        coefficient: 300,
                        scale: 0,
                        currency: "USD".into(),
                    },
                    provenance: vec![prov("value:b")],
                },
            ],
        }
    }

    struct NeverVerifier;
    impl IdentityQualificationVerifier for NeverVerifier {
        fn verify(
            &self,
            _link: &EntityIdentityLink,
            _receipt: &IdentityQualificationReceipt,
        ) -> Result<(), QualificationVerifierFailure> {
            panic!("no identity verification expected")
        }
    }

    #[test]
    fn evidence_time_and_weighting_form_full_cartesian_matrix() {
        let edges = vec![
            award("a", "supplier-a", 10, AssertionStatus::Corroborated),
            award("b", "supplier-b", 30, AssertionStatus::Corroborated),
        ];
        let plan = FullProcurementRobustnessPlan {
            base: base_plan(),
            value_weighting: Some(ProcurementWeightingAssumptions {
                weighting: descriptor("weighting"),
                method_choice: descriptor("method"),
            }),
        };
        let matrix = FullProcurementRobustnessContract::build(
            &edges,
            &[],
            None,
            Some(&values()),
            plan,
            RobustnessCoverage::Exploratory {
                limitation: "fixture".into(),
            },
            &NeverVerifier,
        )
        .unwrap();
        assert_eq!(matrix.envelope.scenarios.len(), 8);
        assert_eq!(matrix.scenario_lineage.len(), 8);
        assert_eq!(
            matrix
                .scenario_lineage
                .iter()
                .filter(|lineage| lineage.weighting_mode == ProcurementWeightingMode::QualifiedAwardValue)
                .count(),
            4
        );
    }

    #[test]
    fn filtered_value_scenario_preserves_exact_monetary_subset() {
        let edges = vec![
            award("a", "supplier-a", 10, AssertionStatus::Corroborated),
            award("b", "supplier-b", 30, AssertionStatus::Corroborated),
        ];
        let plan = FullProcurementRobustnessPlan {
            base: base_plan(),
            value_weighting: Some(ProcurementWeightingAssumptions {
                weighting: descriptor("weighting"),
                method_choice: descriptor("method"),
            }),
        };
        let matrix = FullProcurementRobustnessContract::build(
            &edges,
            &[],
            None,
            Some(&values()),
            plan,
            RobustnessCoverage::Exploratory {
                limitation: "fixture".into(),
            },
            &NeverVerifier,
        )
        .unwrap();
        let lineage = matrix
            .scenario_lineage
            .iter()
            .find(|lineage| {
                lineage.weighting_mode == ProcurementWeightingMode::QualifiedAwardValue
                    && lineage.input_edge_refs == vec!["a".to_string()]
            })
            .expect("windowed value scenario");
        assert_eq!(lineage.value_record_refs, vec!["value:a"]);
    }

    struct CountingVerifier {
        calls: Cell<usize>,
    }
    impl IdentityQualificationVerifier for CountingVerifier {
        fn verify(
            &self,
            link: &EntityIdentityLink,
            receipt: &IdentityQualificationReceipt,
        ) -> Result<(), QualificationVerifierFailure> {
            self.calls.set(self.calls.get() + 1);
            if receipt.subject_commitment == format!("commit:{}", link.id) {
                Ok(())
            } else {
                Err(QualificationVerifierFailure {
                    code: "bad-commitment".into(),
                })
            }
        }
    }

    fn identity_input() -> QualifiedIdentityLinkInput {
        let left = node("supplier-a");
        let right = node("supplier-b");
        let link = EntityIdentityLink {
            id: "link:ab".into(),
            left: left.clone(),
            right: right.clone(),
            status: IdentityLinkStatus::Corroborated,
            binding_evidence: vec![
                EntityBindingEvidence {
                    node: left,
                    scheme: "GB-COH".into(),
                    identifier: "09506232".into(),
                    standard: ExternalStandard::Ocds11SchemaRevision115,
                    source_ref: "source:left".into(),
                    content_hash: "sha256:left".into(),
                    validation_receipt_ref: "validation:left".into(),
                    policy_ref: "ingestion:v1".into(),
                    observed_at: 10,
                },
                EntityBindingEvidence {
                    node: right,
                    scheme: "GB-COH".into(),
                    identifier: "09506232".into(),
                    standard: ExternalStandard::Bods04,
                    source_ref: "source:right".into(),
                    content_hash: "sha256:right".into(),
                    validation_receipt_ref: "validation:right".into(),
                    policy_ref: "ingestion:v1".into(),
                    observed_at: 10,
                },
            ],
            verification_evidence: vec![IdentityVerification {
                verification_ref: "verification:registry".into(),
                verifier_ref: "verifier:registry".into(),
                kind: IdentityVerificationKind::AuthoritativeRegistryLookup {
                    scheme: "GB-COH".into(),
                    identifier: "09506232".into(),
                    registry_ref: "registry:companies-house".into(),
                },
                provenance: vec![prov("registry")],
                verified_at: 20,
            }],
            challenge_refs: vec![],
            review_ref: Some("review:ab".into()),
            review_rationale: Some("exact identifier reviewed".into()),
            superseded_by: None,
            reversible: true,
            recorded_at: 30,
        };
        QualifiedIdentityLinkInput {
            receipt: IdentityQualificationReceipt {
                receipt_ref: "receipt:ab".into(),
                link_ref: link.id.clone(),
                subject_commitment: format!("commit:{}", link.id),
                authority_ref: "authority:test".into(),
                qualification_policy_ref: "identity-policy:v1".into(),
                verification_method_ref: "test-verifier:v1".into(),
                evidence: vec![prov("receipt")],
                issued_at: 1,
                expires_at: 200,
            },
            link,
        }
    }

    #[test]
    fn identity_is_verified_once_for_entire_matrix() {
        let edges = vec![
            award("a", "supplier-a", 10, AssertionStatus::Declared),
            award("b", "supplier-b", 30, AssertionStatus::Declared),
        ];
        let mut base = base_plan();
        base.corroborated_only = None;
        base.time_windows.clear();
        base.identity_resolution = Some(descriptor("identity"));
        let plan = FullProcurementRobustnessPlan {
            base,
            value_weighting: Some(ProcurementWeightingAssumptions {
                weighting: descriptor("weighting"),
                method_choice: descriptor("method"),
            }),
        };
        let verifier = CountingVerifier { calls: Cell::new(0) };
        let matrix = FullProcurementRobustnessContract::build(
            &edges,
            &[identity_input()],
            Some("identity-policy:v1"),
            Some(&values()),
            plan,
            RobustnessCoverage::Exploratory {
                limitation: "fixture".into(),
            },
            &verifier,
        )
        .unwrap();
        assert_eq!(verifier.calls.get(), 1);
        assert_eq!(matrix.envelope.scenarios.len(), 4);
    }

    #[test]
    fn weighting_requires_a_value_set() {
        let plan = FullProcurementRobustnessPlan {
            base: base_plan(),
            value_weighting: Some(ProcurementWeightingAssumptions {
                weighting: descriptor("weighting"),
                method_choice: descriptor("method"),
            }),
        };
        let error = FullProcurementRobustnessContract::build(
            &[award("a", "supplier-a", 10, AssertionStatus::Corroborated)],
            &[],
            None,
            None,
            plan,
            RobustnessCoverage::Exploratory {
                limitation: "fixture".into(),
            },
            &NeverVerifier,
        )
        .unwrap_err();
        assert!(error.contains(&FullProcurementRobustnessError::WeightingConfigurationMismatch));
    }
}

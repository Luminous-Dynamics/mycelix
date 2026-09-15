// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-011 procurement-specific robustness matrix builder.
//!
//! Identity is qualified once at one evaluation time, freezing one trust snapshot
//! that is reused while selected procurement specifications are enumerated.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use crate::capture_metrics::{CaptureMetricEngine, MetricComputationError, ObservationContext};
use crate::capture_observation::{CaptureSubject, ConfidenceAssessment, ProvenanceRef};
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

pub const MAX_PROCUREMENT_TIME_WINDOWS: usize = 12;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProcurementAssumptionDescriptor {
    pub id: String,
    pub statement: String,
    pub admissibility_ref: String,
    pub provenance: Vec<ProvenanceRef>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProcurementTimeWindow {
    pub descriptor: ProcurementAssumptionDescriptor,
    pub start_recorded_at: u64,
    pub end_recorded_at: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProcurementRobustnessPlan {
    pub matrix_id: String,
    pub subject: CaptureSubject,
    pub evaluated_at: u64,
    pub confidence: ConfidenceAssessment,
    pub limitations: Vec<String>,
    pub identity_resolution: Option<ProcurementAssumptionDescriptor>,
    pub corroborated_only: Option<ProcurementAssumptionDescriptor>,
    pub time_windows: Vec<ProcurementTimeWindow>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProcurementScenarioLineage {
    pub scenario_ref: String,
    pub input_edge_refs: Vec<String>,
    pub applied_equivalence_component_refs: Vec<String>,
    pub applied_identity_link_refs: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProcurementRobustnessMatrix {
    pub envelope: InstitutionalRobustnessEnvelope,
    pub scenario_lineage: Vec<ProcurementScenarioLineage>,
    pub qualification_receipt_refs: Vec<String>,
    pub matrix_method_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ProcurementRobustnessError {
    InvalidPlan,
    NoPerturbationAxis,
    TooManyTimeWindows,
    InvalidTimeWindow { window_index: usize },
    DuplicateAssumptionId { assumption_id: String },
    IdentityConfigurationMismatch,
    MissingIdentityPolicyReference,
    IdentityQualification(Vec<IdentityQualificationError>),
    ScenarioMetric {
        scenario_ref: String,
        error: MetricComputationError,
    },
    RobustnessEnvelope(Vec<RobustnessEnvelopeError>),
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ProcurementRobustnessContract;

impl ProcurementRobustnessContract {
    pub fn build<V: IdentityQualificationVerifier>(
        edges: &[InstitutionalEdge],
        identity_inputs: &[QualifiedIdentityLinkInput],
        required_identity_policy_ref: Option<&str>,
        plan: ProcurementRobustnessPlan,
        coverage: RobustnessCoverage,
        verifier: &V,
    ) -> Result<ProcurementRobustnessMatrix, Vec<ProcurementRobustnessError>> {
        validate_plan(&plan, identity_inputs, required_identity_policy_ref)?;

        let qualified_view = if identity_inputs.is_empty() {
            None
        } else {
            Some(
                IdentityQualificationContract::build_equivalence_view(
                    identity_inputs,
                    required_identity_policy_ref.expect("validated identity policy"),
                    plan.evaluated_at,
                    verifier,
                )
                .map_err(|errors| vec![ProcurementRobustnessError::IdentityQualification(errors)])?,
            )
        };

        let mut assumptions = Vec::new();
        if let Some(descriptor) = &plan.identity_resolution {
            assumptions.push(to_assumption(
                descriptor,
                RobustnessDimension::IdentityReconciliation,
            ));
        }
        if let Some(descriptor) = &plan.corroborated_only {
            assumptions.push(to_assumption(
                descriptor,
                RobustnessDimension::EvidenceCorroboration,
            ));
        }
        for window in &plan.time_windows {
            assumptions.push(to_assumption(
                &window.descriptor,
                RobustnessDimension::TimeWindow,
            ));
        }
        assumptions.sort_by(|left, right| left.id.cmp(&right.id));

        let identity_options = if qualified_view.is_some() {
            vec![false, true]
        } else {
            vec![false]
        };
        let evidence_options = if plan.corroborated_only.is_some() {
            vec![false, true]
        } else {
            vec![false]
        };
        let mut window_options: Vec<Option<&ProcurementTimeWindow>> = vec![None];
        window_options.extend(plan.time_windows.iter().map(Some));

        let mut scenarios = Vec::new();
        let mut scenario_lineage = Vec::new();
        for resolved_identity in identity_options {
            for corroborated_only in &evidence_options {
                for window in &window_options {
                    let mut assumption_refs = Vec::new();
                    if resolved_identity {
                        assumption_refs.push(
                            plan.identity_resolution
                                .as_ref()
                                .expect("validated identity descriptor")
                                .id
                                .clone(),
                        );
                    }
                    if *corroborated_only {
                        assumption_refs.push(
                            plan.corroborated_only
                                .as_ref()
                                .expect("validated evidence descriptor")
                                .id
                                .clone(),
                        );
                    }
                    if let Some(window) = window {
                        assumption_refs.push(window.descriptor.id.clone());
                    }
                    assumption_refs.sort();

                    let baseline = assumption_refs.is_empty();
                    let scenario_ref = scenario_key(&plan.matrix_id, &assumption_refs);
                    let selected = select_edges(edges, *corroborated_only, *window);

                    let mut limitations = plan.limitations.clone();
                    if *corroborated_only {
                        limitations.push(
                            "scenario includes only AC-003 corroborated assertions".into(),
                        );
                    }
                    if let Some(window) = window {
                        limitations.push(format!(
                            "scenario includes only records with recorded_at in [{}, {})",
                            window.start_recorded_at, window.end_recorded_at
                        ));
                    }

                    let (metric_edges, applied_components, applied_links) = if resolved_identity {
                        limitations.push(
                            "supplier identity regrouping uses one AC-008-qualified equivalence snapshot shared across the entire matrix"
                                .into(),
                        );
                        project_edges_through_view(
                            &selected,
                            qualified_view.as_ref().expect("validated qualified view"),
                        )
                    } else {
                        (selected, vec![], vec![])
                    };

                    let context = ObservationContext {
                        observation_id: stable_key(
                            "ac-011-observation",
                            &[plan.matrix_id.as_str(), scenario_ref.as_str()],
                        ),
                        subject: plan.subject.clone(),
                        observed_at: plan.evaluated_at,
                        confidence: plan.confidence.clone(),
                        limitations,
                    };
                    let derived = CaptureMetricEngine::procurement_supplier_concentration(
                        &metric_edges,
                        context,
                    )
                    .map_err(|error| {
                        vec![ProcurementRobustnessError::ScenarioMetric {
                            scenario_ref: scenario_ref.clone(),
                            error,
                        }]
                    })?;

                    let kind = if baseline {
                        RobustnessScenarioKind::Baseline
                    } else if dimension_count(
                        resolved_identity,
                        *corroborated_only,
                        window.is_some(),
                    ) == 1
                    {
                        RobustnessScenarioKind::SingleDimension
                    } else {
                        RobustnessScenarioKind::Joint
                    };

                    scenarios.push(RobustnessScenario {
                        id: scenario_ref.clone(),
                        kind,
                        assumption_refs,
                        observation: derived.observation,
                    });
                    scenario_lineage.push(ProcurementScenarioLineage {
                        scenario_ref,
                        input_edge_refs: derived.input_edge_refs,
                        applied_equivalence_component_refs: applied_components,
                        applied_identity_link_refs: applied_links,
                    });
                }
            }
        }

        scenarios.sort_by(|left, right| left.id.cmp(&right.id));
        scenario_lineage.sort_by(|left, right| left.scenario_ref.cmp(&right.scenario_ref));
        let envelope = InstitutionalRobustnessContract::build(
            &plan.matrix_id,
            &assumptions,
            &scenarios,
            coverage,
        )
        .map_err(|errors| vec![ProcurementRobustnessError::RobustnessEnvelope(errors)])?;

        let mut qualification_receipt_refs: Vec<_> = identity_inputs
            .iter()
            .map(|input| input.receipt.receipt_ref.clone())
            .collect();
        qualification_receipt_refs.sort();
        qualification_receipt_refs.dedup();

        Ok(ProcurementRobustnessMatrix {
            envelope,
            scenario_lineage,
            qualification_receipt_refs,
            matrix_method_ref: "mycelix:ac-011:procurement-robustness-matrix:v1".into(),
        })
    }
}

fn validate_plan(
    plan: &ProcurementRobustnessPlan,
    identity_inputs: &[QualifiedIdentityLinkInput],
    required_identity_policy_ref: Option<&str>,
) -> Result<(), Vec<ProcurementRobustnessError>> {
    let mut errors = Vec::new();
    if plan.matrix_id.trim().is_empty()
        || plan.limitations.is_empty()
        || plan.limitations.iter().any(|item| item.trim().is_empty())
        || !matches!(&plan.subject, CaptureSubject::ContractingProcedure(_))
    {
        errors.push(ProcurementRobustnessError::InvalidPlan);
    }
    if plan.identity_resolution.is_none()
        && plan.corroborated_only.is_none()
        && plan.time_windows.is_empty()
    {
        errors.push(ProcurementRobustnessError::NoPerturbationAxis);
    }
    if plan.time_windows.len() > MAX_PROCUREMENT_TIME_WINDOWS {
        errors.push(ProcurementRobustnessError::TooManyTimeWindows);
    }

    let identity_enabled = plan.identity_resolution.is_some();
    if identity_enabled != !identity_inputs.is_empty() {
        errors.push(ProcurementRobustnessError::IdentityConfigurationMismatch);
    }
    if identity_enabled
        && !required_identity_policy_ref.is_some_and(|reference| !reference.trim().is_empty())
    {
        errors.push(ProcurementRobustnessError::MissingIdentityPolicyReference);
    }

    let mut assumption_ids = BTreeSet::new();
    for descriptor in plan
        .identity_resolution
        .iter()
        .chain(plan.corroborated_only.iter())
    {
        validate_descriptor(descriptor, &mut assumption_ids, &mut errors);
    }
    for (window_index, window) in plan.time_windows.iter().enumerate() {
        validate_descriptor(&window.descriptor, &mut assumption_ids, &mut errors);
        if window.start_recorded_at >= window.end_recorded_at {
            errors.push(ProcurementRobustnessError::InvalidTimeWindow { window_index });
        }
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
    errors: &mut Vec<ProcurementRobustnessError>,
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
        errors.push(ProcurementRobustnessError::InvalidPlan);
    }
    if !seen.insert(descriptor.id.clone()) {
        errors.push(ProcurementRobustnessError::DuplicateAssumptionId {
            assumption_id: descriptor.id.clone(),
        });
    }
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

fn dimension_count(identity: bool, evidence: bool, window: bool) -> usize {
    usize::from(identity) + usize::from(evidence) + usize::from(window)
}

fn scenario_key(matrix_id: &str, assumption_refs: &[String]) -> String {
    if assumption_refs.is_empty() {
        return stable_key("ac-011-procurement-baseline", &[matrix_id]);
    }
    let parts: Vec<_> = std::iter::once(matrix_id)
        .chain(assumption_refs.iter().map(String::as_str))
        .collect();
    stable_key("ac-011-procurement-scenario", &parts)
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
    use crate::capture_observation::ConfidenceLevel;
    use crate::identity_qualification::{
        IdentityQualificationReceipt, QualificationVerifierFailure,
    };
    use crate::identity_resolution::{
        EntityBindingEvidence, EntityIdentityLink, IdentityLinkStatus, IdentityVerification,
        IdentityVerificationKind,
    };
    use crate::institutional_graph::{DisclosureClass, InstitutionalNodeRef};
    use crate::standards_ingestion::ExternalStandard;

    struct TestVerifier;
    impl IdentityQualificationVerifier for TestVerifier {
        fn verify(
            &self,
            link: &EntityIdentityLink,
            receipt: &IdentityQualificationReceipt,
        ) -> Result<(), QualificationVerifierFailure> {
            if receipt.subject_commitment == format!("commit:{}", link.id) {
                Ok(())
            } else {
                Err(QualificationVerifierFailure {
                    code: "bad-commitment".into(),
                })
            }
        }
    }

    fn provenance(id: &str) -> ProvenanceRef {
        ProvenanceRef {
            source_ref: format!("source:{id}"),
            content_hash: Some(format!("sha256:{id}")),
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
        let mut edge_provenance = vec![provenance(id)];
        if status == AssertionStatus::Corroborated {
            edge_provenance.push(provenance(&format!("{id}:independent")));
        }
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
            provenance: edge_provenance,
            challenge_refs: vec![],
            recorded_at,
            valid_from: None,
            valid_until: None,
        }
    }

    fn qualified_link_input() -> QualifiedIdentityLinkInput {
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
                verification_ref: "verify:ab".into(),
                verifier_ref: "registry-verifier".into(),
                kind: IdentityVerificationKind::AuthoritativeRegistryLookup {
                    scheme: "GB-COH".into(),
                    identifier: "09506232".into(),
                    registry_ref: "registry:companies-house".into(),
                },
                provenance: vec![provenance("registry")],
                verified_at: 20,
            }],
            challenge_refs: vec![],
            review_ref: Some("review:ab".into()),
            review_rationale: Some("exact registry identifier".into()),
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
                evidence: vec![provenance("qualification")],
                issued_at: 1,
                expires_at: 1_000,
            },
            link,
        }
    }

    fn plan() -> ProcurementRobustnessPlan {
        ProcurementRobustnessPlan {
            matrix_id: "matrix:test".into(),
            subject: CaptureSubject::ContractingProcedure("procedure".into()),
            evaluated_at: 100,
            confidence: ConfidenceAssessment::Qualitative {
                level: ConfidenceLevel::Moderate,
                basis: "fixture".into(),
            },
            limitations: vec!["fixture matrix".into()],
            identity_resolution: Some(descriptor("identity")),
            corroborated_only: Some(descriptor("corroborated")),
            time_windows: vec![ProcurementTimeWindow {
                descriptor: descriptor("recent"),
                start_recorded_at: 50,
                end_recorded_at: 100,
            }],
        }
    }

    #[test]
    fn enumerates_full_selected_axis_product() {
        let edges = vec![
            award("a1", "supplier-a", 60, AssertionStatus::Corroborated),
            award("b1", "supplier-b", 70, AssertionStatus::Corroborated),
            award("c1", "supplier-c", 40, AssertionStatus::Declared),
        ];
        let result = ProcurementRobustnessContract::build(
            &edges,
            &[qualified_link_input()],
            Some("identity-policy:v1"),
            plan(),
            RobustnessCoverage::Exploratory {
                limitation: "fixture scope".into(),
            },
            &TestVerifier,
        )
        .unwrap();
        assert_eq!(result.envelope.scenarios.len(), 8);
        assert_eq!(result.scenario_lineage.len(), 8);
        assert_eq!(result.qualification_receipt_refs, vec!["receipt:ab"]);
    }

    #[test]
    fn invalid_window_fails_before_qualification() {
        let mut invalid = plan();
        invalid.time_windows[0].end_recorded_at = 50;
        assert!(ProcurementRobustnessContract::build(
            &[award("a1", "supplier-a", 60, AssertionStatus::Corroborated)],
            &[qualified_link_input()],
            Some("identity-policy:v1"),
            invalid,
            RobustnessCoverage::Exploratory {
                limitation: "fixture".into(),
            },
            &TestVerifier,
        )
        .unwrap_err()
        .contains(&ProcurementRobustnessError::InvalidTimeWindow { window_index: 0 }));
    }

    #[test]
    fn baseline_only_plan_is_rejected() {
        let mut baseline_only = plan();
        baseline_only.identity_resolution = None;
        baseline_only.corroborated_only = None;
        baseline_only.time_windows.clear();
        assert!(validate_plan(&baseline_only, &[], None)
            .unwrap_err()
            .contains(&ProcurementRobustnessError::NoPerturbationAxis));
    }
}

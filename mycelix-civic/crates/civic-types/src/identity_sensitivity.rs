// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-009 identity-resolution sensitivity diagnostics.
//!
//! AC-009 explains how much a reversible identity projection changed procurement
//! supplier concentration and attributes that exact HHI delta to the equivalence
//! components responsible for regrouping. It is robustness analysis, not an
//! accusation or finding of wrongdoing.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::equivalence_view::{EquivalenceComponent, IdentityProjectedObservation};
use crate::institutional_graph::{
    InstitutionalEdge, InstitutionalGraphContract, InstitutionalGraphViolation,
    InstitutionalRelationKind, ProcurementRole,
};

pub const AC007_PROJECTION_METHOD: &str =
    "mycelix:ac-007:qualified-entity-equivalence-view:v1";
pub const AC004_SUPPLIER_HHI_METHOD: &str =
    "mycelix:ac-004:procurement-supplier-hhi:v1";

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ExactDiagnosticRatio {
    pub numerator: u64,
    pub denominator: u64,
    pub unit: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct SupplierMemberCount {
    pub node_ref: String,
    pub award_count: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ComponentSensitivity {
    pub component_id: String,
    pub identity_link_refs: Vec<String>,
    pub supplier_member_counts: Vec<SupplierMemberCount>,
    pub hhi_delta: ExactDiagnosticRatio,
    pub share_of_identity_delta: Option<ExactDiagnosticRatio>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct IdentityProjectionSensitivityReport {
    pub projection_id: String,
    pub award_count: u64,
    pub distinct_supplier_records_before: u64,
    pub effective_suppliers_after: u64,
    pub identity_hhi_delta: ExactDiagnosticRatio,
    pub components: Vec<ComponentSensitivity>,
    pub dominant_component_id: Option<String>,
    pub dominant_component_share: Option<ExactDiagnosticRatio>,
    pub diagnostic_method_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum IdentitySensitivityError {
    MissingProjectionId,
    UnsupportedProjectionMethod,
    UnsupportedMetricMethod,
    InvalidAwardEdge {
        index: usize,
        violations: Vec<InstitutionalGraphViolation>,
    },
    EmptyAwardPopulation,
    MissingAwardEdgeId,
    DuplicateAwardEdgeId { edge_id: String },
    InputEdgeLineageMismatch,
    NonCanonicalAppliedLinkLineage,
    DuplicateComponentId { component_id: String },
    InvalidComponent,
    OverlappingComponentMember { node_ref: String },
    AppliedComponentDoesNotRegroupSuppliers { component_id: String },
    ArithmeticOverflow,
    BaselineMetricMismatch,
    ProjectedMetricMismatch,
    ComponentDeltaMismatch,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct IdentitySensitivityContract;

impl IdentitySensitivityContract {
    pub fn analyze_procurement_supplier_projection(
        edges: &[InstitutionalEdge],
        projection: &IdentityProjectedObservation,
    ) -> Result<IdentityProjectionSensitivityReport, Vec<IdentitySensitivityError>> {
        let mut errors = Vec::new();

        if projection.projection_id.trim().is_empty() {
            errors.push(IdentitySensitivityError::MissingProjectionId);
        }
        if projection.projection_method_ref != AC007_PROJECTION_METHOD {
            errors.push(IdentitySensitivityError::UnsupportedProjectionMethod);
        }
        if projection.baseline.observation.measurement.method_ref != AC004_SUPPLIER_HHI_METHOD
            || projection.projected.observation.measurement.method_ref != AC004_SUPPLIER_HHI_METHOD
        {
            errors.push(IdentitySensitivityError::UnsupportedMetricMethod);
        }

        let mut award_edges = Vec::new();
        let mut edge_ids = BTreeSet::new();
        for (index, edge) in edges.iter().enumerate() {
            if !matches!(
                &edge.relation,
                InstitutionalRelationKind::ProcurementParticipation {
                    role: ProcurementRole::Awardee
                }
            ) {
                continue;
            }
            if edge.id.trim().is_empty() {
                errors.push(IdentitySensitivityError::MissingAwardEdgeId);
                continue;
            }
            if !edge_ids.insert(edge.id.clone()) {
                errors.push(IdentitySensitivityError::DuplicateAwardEdgeId {
                    edge_id: edge.id.clone(),
                });
            }
            if let Err(violations) = InstitutionalGraphContract::validate_edge(edge) {
                errors.push(IdentitySensitivityError::InvalidAwardEdge { index, violations });
            }
            award_edges.push(edge);
        }
        if award_edges.is_empty() {
            errors.push(IdentitySensitivityError::EmptyAwardPopulation);
        }

        let expected_edge_refs: Vec<_> = edge_ids.iter().cloned().collect();
        if projection.baseline.input_edge_refs != expected_edge_refs
            || projection.projected.input_edge_refs != expected_edge_refs
        {
            errors.push(IdentitySensitivityError::InputEdgeLineageMismatch);
        }

        let component_link_refs: Vec<_> = projection
            .applied_components
            .iter()
            .flat_map(|component| component.identity_link_refs.iter().cloned())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        if component_link_refs != projection.applied_identity_link_refs {
            errors.push(IdentitySensitivityError::NonCanonicalAppliedLinkLineage);
        }

        if !errors.is_empty() {
            return Err(errors);
        }

        let award_count = u64::try_from(award_edges.len())
            .map_err(|_| vec![IdentitySensitivityError::ArithmeticOverflow])?;
        let expected_denominator = award_count
            .checked_mul(award_count)
            .ok_or_else(|| vec![IdentitySensitivityError::ArithmeticOverflow])?;

        let mut raw_counts = BTreeMap::<String, u64>::new();
        for edge in &award_edges {
            let count = raw_counts.entry(edge.from.id.clone()).or_insert(0);
            *count = count
                .checked_add(1)
                .ok_or_else(|| vec![IdentitySensitivityError::ArithmeticOverflow])?;
        }
        let raw_numerator = sum_squares(raw_counts.values().copied())
            .map_err(|error| vec![error])?;

        let mut member_to_component = BTreeMap::<String, String>::new();
        let mut component_ids = BTreeSet::new();
        for component in &projection.applied_components {
            if component.id.trim().is_empty()
                || component.members.len() < 2
                || component.identity_link_refs.is_empty()
                || !strictly_sorted_unique(&component.members)
                || !strictly_sorted_unique(&component.identity_link_refs)
            {
                errors.push(IdentitySensitivityError::InvalidComponent);
                continue;
            }
            if !component_ids.insert(component.id.clone()) {
                errors.push(IdentitySensitivityError::DuplicateComponentId {
                    component_id: component.id.clone(),
                });
            }
            for member in &component.members {
                if let Some(existing) = member_to_component.insert(member.clone(), component.id.clone()) {
                    if existing != component.id {
                        errors.push(IdentitySensitivityError::OverlappingComponentMember {
                            node_ref: member.clone(),
                        });
                    }
                }
            }
        }
        if !errors.is_empty() {
            return Err(errors);
        }

        let mut projected_counts = BTreeMap::<String, u64>::new();
        for (supplier, count) in &raw_counts {
            let grouping_key = member_to_component
                .get(supplier)
                .cloned()
                .unwrap_or_else(|| supplier.clone());
            let grouped = projected_counts.entry(grouping_key).or_insert(0);
            *grouped = grouped
                .checked_add(*count)
                .ok_or_else(|| vec![IdentitySensitivityError::ArithmeticOverflow])?;
        }
        let projected_numerator = sum_squares(projected_counts.values().copied())
            .map_err(|error| vec![error])?;

        if !metric_equals(
            &projection.baseline.observation.measurement.value,
            raw_numerator,
            expected_denominator,
        ) {
            errors.push(IdentitySensitivityError::BaselineMetricMismatch);
        }
        if !metric_equals(
            &projection.projected.observation.measurement.value,
            projected_numerator,
            expected_denominator,
        ) {
            errors.push(IdentitySensitivityError::ProjectedMetricMismatch);
        }
        if !errors.is_empty() {
            return Err(errors);
        }

        let total_delta = projected_numerator
            .checked_sub(raw_numerator)
            .ok_or_else(|| vec![IdentitySensitivityError::ProjectedMetricMismatch])?;

        let mut component_reports = Vec::new();
        let mut summed_component_delta = 0_u64;
        let mut dominant: Option<(String, u64)> = None;

        for component in &projection.applied_components {
            let mut member_counts = Vec::new();
            let mut component_total = 0_u64;
            let mut component_raw_squares = 0_u64;
            for member in &component.members {
                let Some(count) = raw_counts.get(member).copied() else {
                    continue;
                };
                member_counts.push(SupplierMemberCount {
                    node_ref: member.clone(),
                    award_count: count,
                });
                component_total = component_total
                    .checked_add(count)
                    .ok_or_else(|| vec![IdentitySensitivityError::ArithmeticOverflow])?;
                component_raw_squares = component_raw_squares
                    .checked_add(
                        count
                            .checked_mul(count)
                            .ok_or_else(|| vec![IdentitySensitivityError::ArithmeticOverflow])?,
                    )
                    .ok_or_else(|| vec![IdentitySensitivityError::ArithmeticOverflow])?;
            }
            if member_counts.len() < 2 {
                errors.push(IdentitySensitivityError::AppliedComponentDoesNotRegroupSuppliers {
                    component_id: component.id.clone(),
                });
                continue;
            }
            let merged_square = component_total
                .checked_mul(component_total)
                .ok_or_else(|| vec![IdentitySensitivityError::ArithmeticOverflow])?;
            let contribution = merged_square
                .checked_sub(component_raw_squares)
                .ok_or_else(|| vec![IdentitySensitivityError::ArithmeticOverflow])?;
            summed_component_delta = summed_component_delta
                .checked_add(contribution)
                .ok_or_else(|| vec![IdentitySensitivityError::ArithmeticOverflow])?;

            let share = if total_delta == 0 {
                None
            } else {
                Some(reduced_ratio(
                    contribution,
                    total_delta,
                    "share_of_identity_hhi_delta",
                ))
            };
            match &dominant {
                Some((_, current)) if *current >= contribution => {}
                _ => dominant = Some((component.id.clone(), contribution)),
            }
            component_reports.push(ComponentSensitivity {
                component_id: component.id.clone(),
                identity_link_refs: component.identity_link_refs.clone(),
                supplier_member_counts: member_counts,
                hhi_delta: reduced_ratio(contribution, expected_denominator, "hhi_delta"),
                share_of_identity_delta: share,
            });
        }

        if !errors.is_empty() {
            return Err(errors);
        }
        if summed_component_delta != total_delta {
            return Err(vec![IdentitySensitivityError::ComponentDeltaMismatch]);
        }

        component_reports.sort_by(|a, b| a.component_id.cmp(&b.component_id));
        let (dominant_component_id, dominant_component_share) = match dominant {
            Some((id, contribution)) if total_delta > 0 => (
                Some(id),
                Some(reduced_ratio(
                    contribution,
                    total_delta,
                    "share_of_identity_hhi_delta",
                )),
            ),
            _ => (None, None),
        };

        Ok(IdentityProjectionSensitivityReport {
            projection_id: projection.projection_id.clone(),
            award_count,
            distinct_supplier_records_before: u64::try_from(raw_counts.len())
                .map_err(|_| vec![IdentitySensitivityError::ArithmeticOverflow])?,
            effective_suppliers_after: u64::try_from(projected_counts.len())
                .map_err(|_| vec![IdentitySensitivityError::ArithmeticOverflow])?,
            identity_hhi_delta: reduced_ratio(total_delta, expected_denominator, "hhi_delta"),
            components: component_reports,
            dominant_component_id,
            dominant_component_share,
            diagnostic_method_ref: "mycelix:ac-009:identity-projection-sensitivity:v1".into(),
        })
    }
}

fn metric_equals(value: &crate::capture_observation::MetricValue, numerator: u64, denominator: u64) -> bool {
    value.unit == "hhi_ratio"
        && value.denominator == denominator
        && i64::try_from(numerator).ok() == Some(value.numerator)
}

fn sum_squares(values: impl IntoIterator<Item = u64>) -> Result<u64, IdentitySensitivityError> {
    values.into_iter().try_fold(0_u64, |acc, value| {
        let square = value
            .checked_mul(value)
            .ok_or(IdentitySensitivityError::ArithmeticOverflow)?;
        acc.checked_add(square)
            .ok_or(IdentitySensitivityError::ArithmeticOverflow)
    })
}

fn reduced_ratio(numerator: u64, denominator: u64, unit: &str) -> ExactDiagnosticRatio {
    let divisor = gcd(numerator, denominator);
    ExactDiagnosticRatio {
        numerator: numerator / divisor,
        denominator: denominator / divisor,
        unit: unit.into(),
    }
}

fn gcd(mut left: u64, mut right: u64) -> u64 {
    if left == 0 {
        return right.max(1);
    }
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left.max(1)
}

fn strictly_sorted_unique(values: &[String]) -> bool {
    values
        .windows(2)
        .all(|pair| pair[0].as_str() < pair[1].as_str())
        && values.iter().all(|value| !value.trim().is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture_metrics::ObservationContext;
    use crate::capture_observation::{
        CaptureSubject, ConfidenceAssessment, ConfidenceLevel, ProvenanceRef,
    };
    use crate::equivalence_view::EquivalenceViewContract;
    use crate::identity_resolution::{
        EntityBindingEvidence, EntityIdentityLink, IdentityLinkStatus, IdentityVerification,
        IdentityVerificationKind,
    };
    use crate::institutional_graph::{
        AssertionStatus, DisclosureClass, InstitutionalNodeKind, InstitutionalNodeRef,
    };
    use crate::standards_ingestion::ExternalStandard;

    fn node(id: &str) -> InstitutionalNodeRef {
        InstitutionalNodeRef {
            id: id.into(),
            kind: InstitutionalNodeKind::Organization,
        }
    }

    fn link(id: &str, left: &str, right: &str, identifier: &str) -> EntityIdentityLink {
        let left_node = node(left);
        let right_node = node(right);
        EntityIdentityLink {
            id: id.into(),
            left: left_node.clone(),
            right: right_node.clone(),
            status: IdentityLinkStatus::Corroborated,
            binding_evidence: vec![
                EntityBindingEvidence {
                    node: left_node,
                    scheme: "GB-COH".into(),
                    identifier: identifier.into(),
                    standard: ExternalStandard::Ocds11SchemaRevision115,
                    source_ref: format!("source:{id}:left"),
                    content_hash: format!("sha256:{id}:left"),
                    validation_receipt_ref: format!("validation:{id}:left"),
                    policy_ref: "policy:v1".into(),
                    observed_at: 100,
                },
                EntityBindingEvidence {
                    node: right_node,
                    scheme: "GB-COH".into(),
                    identifier: identifier.into(),
                    standard: ExternalStandard::Bods04,
                    source_ref: format!("source:{id}:right"),
                    content_hash: format!("sha256:{id}:right"),
                    validation_receipt_ref: format!("validation:{id}:right"),
                    policy_ref: "policy:v1".into(),
                    observed_at: 100,
                },
            ],
            verification_evidence: vec![IdentityVerification {
                verification_ref: format!("verification:{id}"),
                verifier_ref: "verifier:test".into(),
                kind: IdentityVerificationKind::AuthoritativeRegistryLookup {
                    scheme: "GB-COH".into(),
                    identifier: identifier.into(),
                    registry_ref: "registry:test".into(),
                },
                provenance: vec![ProvenanceRef {
                    source_ref: format!("registry:{id}"),
                    content_hash: Some(format!("sha256:registry:{id}")),
                }],
                verified_at: 150,
            }],
            challenge_refs: vec![],
            review_ref: Some(format!("review:{id}")),
            review_rationale: Some("reviewed exact identifier".into()),
            superseded_by: None,
            reversible: true,
            recorded_at: 160,
        }
    }

    fn award(id: &str, supplier: &str) -> InstitutionalEdge {
        InstitutionalEdge {
            id: id.into(),
            from: node(supplier),
            to: InstitutionalNodeRef {
                id: format!("procedure:{id}"),
                kind: InstitutionalNodeKind::ProcurementProcedure,
            },
            relation: InstitutionalRelationKind::ProcurementParticipation {
                role: ProcurementRole::Awardee,
            },
            disclosure: DisclosureClass::PublicMetadata,
            assertion_status: AssertionStatus::Declared,
            provenance: vec![ProvenanceRef {
                source_ref: format!("source:{id}"),
                content_hash: Some(format!("sha256:{id}")),
            }],
            challenge_refs: vec![],
            recorded_at: 100,
            valid_from: None,
            valid_until: None,
        }
    }

    fn context(id: &str) -> ObservationContext {
        ObservationContext {
            observation_id: id.into(),
            subject: CaptureSubject::ContractingProcedure("municipal-procurement".into()),
            observed_at: 300,
            confidence: ConfidenceAssessment::Qualitative {
                level: ConfidenceLevel::Moderate,
                basis: "sensitivity fixture".into(),
            },
            limitations: vec!["fixture".into()],
        }
    }

    #[test]
    fn one_component_exactly_explains_identity_hhi_delta() {
        let edges = vec![
            award("a1", "supplier:a"),
            award("b1", "supplier:b"),
            award("c1", "supplier:c"),
            award("c2", "supplier:c"),
        ];
        let projection = EquivalenceViewContract::procurement_supplier_concentration(
            &edges,
            &[link("link:ab", "supplier:a", "supplier:b", "09506232")],
            context("projection:sensitivity"),
        )
        .unwrap();
        let report = IdentitySensitivityContract::analyze_procurement_supplier_projection(
            &edges,
            &projection,
        )
        .expect("valid projection has exact sensitivity decomposition");

        assert_eq!(report.identity_hhi_delta.numerator, 1);
        assert_eq!(report.identity_hhi_delta.denominator, 8);
        assert_eq!(report.components.len(), 1);
        assert_eq!(report.components[0].hhi_delta, report.identity_hhi_delta);
        assert_eq!(
            report.components[0].share_of_identity_delta,
            Some(ExactDiagnosticRatio {
                numerator: 1,
                denominator: 1,
                unit: "share_of_identity_hhi_delta".into(),
            })
        );
        assert_eq!(report.distinct_supplier_records_before, 3);
        assert_eq!(report.effective_suppliers_after, 2);
    }

    #[test]
    fn two_components_decompose_without_cross_component_interference() {
        let edges = vec![
            award("a1", "supplier:a"),
            award("b1", "supplier:b"),
            award("c1", "supplier:c"),
            award("d1", "supplier:d"),
        ];
        let projection = EquivalenceViewContract::procurement_supplier_concentration(
            &edges,
            &[
                link("link:ab", "supplier:a", "supplier:b", "11111111"),
                link("link:cd", "supplier:c", "supplier:d", "22222222"),
            ],
            context("projection:two-components"),
        )
        .unwrap();
        let report = IdentitySensitivityContract::analyze_procurement_supplier_projection(
            &edges,
            &projection,
        )
        .unwrap();
        assert_eq!(report.components.len(), 2);
        assert_eq!(report.identity_hhi_delta.numerator, 1);
        assert_eq!(report.identity_hhi_delta.denominator, 4);
        assert!(report.components.iter().all(|component| {
            component.share_of_identity_delta.as_ref().is_some_and(|share| {
                share.numerator == 1 && share.denominator == 2
            })
        }));
    }

    #[test]
    fn tampered_projected_metric_is_rejected() {
        let edges = vec![award("a1", "supplier:a"), award("b1", "supplier:b")];
        let mut projection = EquivalenceViewContract::procurement_supplier_concentration(
            &edges,
            &[link("link:ab", "supplier:a", "supplier:b", "09506232")],
            context("projection:tamper"),
        )
        .unwrap();
        projection.projected.observation.measurement.value.numerator = 1;
        let errors = IdentitySensitivityContract::analyze_procurement_supplier_projection(
            &edges,
            &projection,
        )
        .expect_err("diagnostics must verify the projection receipt before explaining it");
        assert!(errors.contains(&IdentitySensitivityError::ProjectedMetricMismatch));
    }

    #[test]
    fn tampered_link_lineage_is_rejected() {
        let edges = vec![award("a1", "supplier:a"), award("b1", "supplier:b")];
        let mut projection = EquivalenceViewContract::procurement_supplier_concentration(
            &edges,
            &[link("link:ab", "supplier:a", "supplier:b", "09506232")],
            context("projection:lineage-tamper"),
        )
        .unwrap();
        projection.applied_identity_link_refs = vec!["link:other".into()];
        let errors = IdentitySensitivityContract::analyze_procurement_supplier_projection(
            &edges,
            &projection,
        )
        .expect_err("component and receipt lineage must agree exactly");
        assert!(errors.contains(&IdentitySensitivityError::NonCanonicalAppliedLinkLineage));
    }
}

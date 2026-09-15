// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-007 qualified equivalence views and reversible metric projection.
//!
//! AC-007 never mutates AC-003 source nodes or edges. It constructs an ephemeral,
//! deterministic equivalence view from AC-006 aggregation-eligible links, checks
//! cross-link contradictions before the view is used, and records the exact link
//! set responsible for any analytical regrouping.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::capture_metrics::{
    CaptureMetricEngine, DerivedCaptureObservation, MetricComputationError, ObservationContext,
};
use crate::identity_resolution::{
    EntityIdentityLink, IdentityLinkStatus, IdentityResolutionContract, IdentityResolutionViolation,
    IdentityVerificationKind,
};
use crate::institutional_graph::{
    InstitutionalEdge, InstitutionalNodeKind, InstitutionalRelationKind, ProcurementRole,
};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct EquivalenceComponent {
    pub id: String,
    pub members: Vec<String>,
    pub identity_link_refs: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct QualifiedEquivalenceView {
    pub components: Vec<EquivalenceComponent>,
    pub node_to_component: BTreeMap<String, String>,
    pub identity_link_refs: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct IdentityProjectedObservation {
    pub projection_id: String,
    pub baseline: DerivedCaptureObservation,
    pub projected: DerivedCaptureObservation,
    pub applied_components: Vec<EquivalenceComponent>,
    pub applied_identity_link_refs: Vec<String>,
    pub projection_method_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum EquivalenceViewError {
    DuplicateLinkId { link_id: String },
    InvalidIdentityLink {
        index: usize,
        violations: Vec<IdentityResolutionViolation>,
    },
    IdentityLinkNotAggregationEligible { index: usize },
    NodeKindConflict { node_id: String },
    ConflictingIdentifiers {
        component_id: String,
        scheme: String,
        identifiers: Vec<String>,
    },
    BaselineMetric(MetricComputationError),
    ProjectedMetric(MetricComputationError),
}

#[derive(Debug, Default, Clone, Copy)]
pub struct EquivalenceViewContract;

impl EquivalenceViewContract {
    /// Build a deterministic equivalence view only from fully qualified AC-006 links.
    ///
    /// Components are provisional until all cross-link contradiction checks pass.
    /// Nothing in this operation rewrites source graph state.
    pub fn build(
        links: &[EntityIdentityLink],
    ) -> Result<QualifiedEquivalenceView, Vec<EquivalenceViewError>> {
        let mut errors = Vec::new();
        let mut seen_link_ids = BTreeSet::new();
        let mut node_kinds = BTreeMap::<String, InstitutionalNodeKind>::new();
        let mut adjacency = BTreeMap::<String, BTreeSet<String>>::new();

        for (index, link) in links.iter().enumerate() {
            if !seen_link_ids.insert(link.id.clone()) {
                errors.push(EquivalenceViewError::DuplicateLinkId {
                    link_id: link.id.clone(),
                });
            }
            if let Err(violations) = IdentityResolutionContract::validate_link(link) {
                errors.push(EquivalenceViewError::InvalidIdentityLink { index, violations });
                continue;
            }
            if link.status != IdentityLinkStatus::Corroborated
                || !IdentityResolutionContract::aggregation_eligible(link)
            {
                errors.push(EquivalenceViewError::IdentityLinkNotAggregationEligible { index });
                continue;
            }

            for node in [&link.left, &link.right] {
                match node_kinds.get(&node.id) {
                    Some(existing) if existing != &node.kind => {
                        errors.push(EquivalenceViewError::NodeKindConflict {
                            node_id: node.id.clone(),
                        });
                    }
                    _ => {
                        node_kinds
                            .entry(node.id.clone())
                            .or_insert_with(|| node.kind.clone());
                    }
                }
            }
            adjacency
                .entry(link.left.id.clone())
                .or_default()
                .insert(link.right.id.clone());
            adjacency
                .entry(link.right.id.clone())
                .or_default()
                .insert(link.left.id.clone());
        }

        if !errors.is_empty() {
            return Err(errors);
        }

        let mut visited = BTreeSet::<String>::new();
        let mut components = Vec::new();
        let mut node_to_component = BTreeMap::new();

        for start in adjacency.keys() {
            if visited.contains(start) {
                continue;
            }
            let mut frontier = vec![start.clone()];
            let mut member_set = BTreeSet::new();
            while let Some(node) = frontier.pop() {
                if !visited.insert(node.clone()) {
                    continue;
                }
                member_set.insert(node.clone());
                if let Some(neighbors) = adjacency.get(&node) {
                    for neighbor in neighbors.iter().rev() {
                        if !visited.contains(neighbor) {
                            frontier.push(neighbor.clone());
                        }
                    }
                }
            }

            let members: Vec<_> = member_set.iter().cloned().collect();
            let component_id = component_id(&members);
            let mut component_links: Vec<_> = links
                .iter()
                .filter(|link| {
                    member_set.contains(&link.left.id) && member_set.contains(&link.right.id)
                })
                .map(|link| link.id.clone())
                .collect();
            component_links.sort();
            component_links.dedup();

            if let Some(error) = identifier_conflict(
                &component_id,
                &member_set,
                links,
            ) {
                errors.push(error);
                continue;
            }

            for member in &members {
                node_to_component.insert(member.clone(), component_id.clone());
            }
            components.push(EquivalenceComponent {
                id: component_id,
                members,
                identity_link_refs: component_links,
            });
        }

        if !errors.is_empty() {
            return Err(errors);
        }

        components.sort_by(|a, b| a.id.cmp(&b.id));
        let identity_link_refs = seen_link_ids.into_iter().collect();
        Ok(QualifiedEquivalenceView {
            components,
            node_to_component,
            identity_link_refs,
        })
    }

    /// Recompute supplier concentration under a reversible AC-006 equivalence view.
    ///
    /// Only components that contain at least two suppliers present in the current
    /// award population are applied. The source edges are cloned for the projected
    /// calculation and are never mutated.
    pub fn procurement_supplier_concentration(
        edges: &[InstitutionalEdge],
        links: &[EntityIdentityLink],
        context: ObservationContext,
    ) -> Result<IdentityProjectedObservation, Vec<EquivalenceViewError>> {
        let view = Self::build(links)?;
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

        let applied_components: Vec<_> = view
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
            .cloned()
            .collect();
        let active_component_ids: BTreeSet<_> = applied_components
            .iter()
            .map(|component| component.id.clone())
            .collect();
        let applied_identity_link_refs: Vec<_> = applied_components
            .iter()
            .flat_map(|component| component.identity_link_refs.iter().cloned())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();

        let projection_id = context.observation_id.clone();
        let mut baseline_context = context.clone();
        baseline_context.observation_id = stable_key(
            "ac-007-baseline",
            &[projection_id.as_str()],
        );
        let baseline = CaptureMetricEngine::procurement_supplier_concentration(
            edges,
            baseline_context,
        )
        .map_err(|error| vec![EquivalenceViewError::BaselineMetric(error)])?;

        let mut projected_edges = edges.to_vec();
        for edge in &mut projected_edges {
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
            if !active_component_ids.contains(component_id) {
                continue;
            }
            edge.from.id = component_id.clone();
            edge.from.kind = InstitutionalNodeKind::Aggregate;
        }

        let mut projected_context = context;
        projected_context.observation_id = stable_key(
            "ac-007-identity-projected",
            &[projection_id.as_str()],
        );
        projected_context.limitations.push(
            "supplier identities are regrouped only through AC-006 aggregation-eligible links; source graph nodes remain unchanged"
                .into(),
        );
        let projected = CaptureMetricEngine::procurement_supplier_concentration(
            &projected_edges,
            projected_context,
        )
        .map_err(|error| vec![EquivalenceViewError::ProjectedMetric(error)])?;

        Ok(IdentityProjectedObservation {
            projection_id,
            baseline,
            projected,
            applied_components,
            applied_identity_link_refs,
            projection_method_ref: "mycelix:ac-007:qualified-entity-equivalence-view:v1".into(),
        })
    }
}

fn identifier_conflict(
    component_id: &str,
    members: &BTreeSet<String>,
    links: &[EntityIdentityLink],
) -> Option<EquivalenceViewError> {
    let component_links: Vec<_> = links
        .iter()
        .filter(|link| members.contains(&link.left.id) && members.contains(&link.right.id))
        .collect();
    let mut identifiers_by_scheme = BTreeMap::<String, BTreeSet<String>>::new();
    for link in &component_links {
        for evidence in &link.binding_evidence {
            if members.contains(&evidence.node.id) {
                identifiers_by_scheme
                    .entry(evidence.scheme.clone())
                    .or_default()
                    .insert(evidence.identifier.clone());
            }
        }
    }

    for (scheme, identifiers) in identifiers_by_scheme {
        if identifiers.len() <= 1 {
            continue;
        }
        let values: Vec<_> = identifiers.into_iter().collect();
        for left_index in 0..values.len() {
            for right_index in (left_index + 1)..values.len() {
                if !has_authoritative_same_scheme_crosswalk(
                    &component_links,
                    &scheme,
                    &values[left_index],
                    &values[right_index],
                ) {
                    return Some(EquivalenceViewError::ConflictingIdentifiers {
                        component_id: component_id.to_string(),
                        scheme,
                        identifiers: values,
                    });
                }
            }
        }
    }
    None
}

fn has_authoritative_same_scheme_crosswalk(
    links: &[&EntityIdentityLink],
    scheme: &str,
    left_identifier: &str,
    right_identifier: &str,
) -> bool {
    links.iter().any(|link| {
        link.verification_evidence.iter().any(|verification| {
            matches!(
                &verification.kind,
                IdentityVerificationKind::AuthoritativeCrosswalk {
                    left_scheme,
                    left_identifier: verified_left,
                    right_scheme,
                    right_identifier: verified_right,
                    ..
                } if left_scheme == scheme
                    && right_scheme == scheme
                    && ((verified_left == left_identifier && verified_right == right_identifier)
                        || (verified_left == right_identifier && verified_right == left_identifier))
            )
        })
    })
}

fn component_id(members: &[String]) -> String {
    let parts: Vec<_> = members.iter().map(String::as_str).collect();
    stable_key("entity-equivalence-component", &parts)
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
    use crate::capture_observation::{CaptureSubject, ConfidenceAssessment, ConfidenceLevel, ProvenanceRef};
    use crate::identity_resolution::{
        EntityBindingEvidence, IdentityVerification, IdentityVerificationKind,
    };
    use crate::institutional_graph::{
        AssertionStatus, DisclosureClass, InstitutionalNodeRef,
    };
    use crate::standards_ingestion::ExternalStandard;

    fn node(id: &str, kind: InstitutionalNodeKind) -> InstitutionalNodeRef {
        InstitutionalNodeRef {
            id: id.into(),
            kind,
        }
    }

    fn qualified_link(
        id: &str,
        left: InstitutionalNodeRef,
        right: InstitutionalNodeRef,
        scheme: &str,
        identifier: &str,
    ) -> EntityIdentityLink {
        let left_evidence = EntityBindingEvidence {
            node: left.clone(),
            scheme: scheme.into(),
            identifier: identifier.into(),
            standard: ExternalStandard::Ocds11SchemaRevision115,
            source_ref: format!("source:{id}:left"),
            content_hash: format!("sha256:{id}:left"),
            validation_receipt_ref: format!("receipt:{id}:left"),
            policy_ref: "policy:v1".into(),
            observed_at: 100,
        };
        let right_evidence = EntityBindingEvidence {
            node: right.clone(),
            scheme: scheme.into(),
            identifier: identifier.into(),
            standard: ExternalStandard::Bods04,
            source_ref: format!("source:{id}:right"),
            content_hash: format!("sha256:{id}:right"),
            validation_receipt_ref: format!("receipt:{id}:right"),
            policy_ref: "policy:v1".into(),
            observed_at: 100,
        };
        EntityIdentityLink {
            id: id.into(),
            left,
            right,
            status: IdentityLinkStatus::Corroborated,
            binding_evidence: vec![left_evidence, right_evidence],
            verification_evidence: vec![IdentityVerification {
                verification_ref: format!("verification:{id}"),
                verifier_ref: "verifier:independent".into(),
                kind: IdentityVerificationKind::AuthoritativeRegistryLookup {
                    scheme: scheme.into(),
                    identifier: identifier.into(),
                    registry_ref: "registry:authoritative".into(),
                },
                provenance: vec![ProvenanceRef {
                    source_ref: format!("registry:{id}"),
                    content_hash: Some(format!("sha256:registry:{id}")),
                }],
                verified_at: 200,
            }],
            challenge_refs: vec![],
            review_ref: Some(format!("review:{id}")),
            review_rationale: Some("authoritative public-entity identifier verified".into()),
            superseded_by: None,
            reversible: true,
            recorded_at: 200,
        }
    }

    fn award(id: &str, supplier: &str) -> InstitutionalEdge {
        InstitutionalEdge {
            id: id.into(),
            from: node(supplier, InstitutionalNodeKind::Organization),
            to: node(
                &format!("procedure:{id}"),
                InstitutionalNodeKind::ProcurementProcedure,
            ),
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
                basis: "complete declared award-edge population for the period".into(),
            },
            limitations: vec!["source publication completeness remains external".into()],
        }
    }

    #[test]
    fn empty_identity_view_preserves_raw_metric() {
        let edges = vec![award("a1", "supplier:a"), award("b1", "supplier:b")];
        let result = EquivalenceViewContract::procurement_supplier_concentration(
            &edges,
            &[],
            context("projection:none"),
        )
        .expect("empty identity view is a valid no-op");
        assert_eq!(
            result.baseline.observation.measurement.value,
            result.projected.observation.measurement.value
        );
        assert!(result.applied_identity_link_refs.is_empty());
    }

    #[test]
    fn qualified_identity_view_changes_grouping_without_mutating_source_edges() {
        let edges = vec![
            award("a1", "supplier:a"),
            award("b1", "supplier:b"),
            award("c1", "supplier:c"),
            award("c2", "supplier:c"),
        ];
        let original_actor_ids: Vec<_> = edges.iter().map(|edge| edge.from.id.clone()).collect();
        let link = qualified_link(
            "link:ab",
            node("supplier:a", InstitutionalNodeKind::Organization),
            node("supplier:b", InstitutionalNodeKind::Organization),
            "GB-COH",
            "09506232",
        );
        let result = EquivalenceViewContract::procurement_supplier_concentration(
            &edges,
            &[link],
            context("projection:ab"),
        )
        .expect("qualified link can project a reversible grouping");

        assert_eq!(result.baseline.observation.measurement.value.numerator, 6);
        assert_eq!(result.baseline.observation.measurement.value.denominator, 16);
        assert_eq!(result.projected.observation.measurement.value.numerator, 8);
        assert_eq!(result.projected.observation.measurement.value.denominator, 16);
        assert_eq!(result.applied_identity_link_refs, vec!["link:ab"]);
        assert_eq!(
            edges.iter().map(|edge| edge.from.id.clone()).collect::<Vec<_>>(),
            original_actor_ids,
            "source graph edges must remain untouched"
        );
    }

    #[test]
    fn unresolved_challenge_cannot_enter_equivalence_view() {
        let mut link = qualified_link(
            "link:ab",
            node("supplier:a", InstitutionalNodeKind::Organization),
            node("supplier:b", InstitutionalNodeKind::Organization),
            "GB-COH",
            "09506232",
        );
        link.challenge_refs = vec!["challenge:1".into()];
        let errors = EquivalenceViewContract::build(&[link])
            .expect_err("challenged corroboration must fail before closure");
        assert!(errors.iter().any(|error| matches!(
            error,
            EquivalenceViewError::InvalidIdentityLink { .. }
        )));
    }

    #[test]
    fn conflicting_same_scheme_identifiers_block_transitive_component() {
        let a = node("entity:a", InstitutionalNodeKind::Organization);
        let b = node("entity:b", InstitutionalNodeKind::Organization);
        let c = node("entity:c", InstitutionalNodeKind::LegalEntity);
        let ab = qualified_link("link:ab", a, b.clone(), "GB-COH", "11111111");
        let bc = qualified_link("link:bc", b, c, "GB-COH", "22222222");
        let errors = EquivalenceViewContract::build(&[ab, bc])
            .expect_err("same registry scheme with incompatible IDs must not transitively collapse");
        assert!(errors.iter().any(|error| matches!(
            error,
            EquivalenceViewError::ConflictingIdentifiers { scheme, .. } if scheme == "GB-COH"
        )));
    }

    #[test]
    fn component_lineage_contains_exact_links_used_for_metric_regrouping() {
        let edges = vec![award("a1", "supplier:a"), award("b1", "supplier:b")];
        let used = qualified_link(
            "link:used",
            node("supplier:a", InstitutionalNodeKind::Organization),
            node("supplier:b", InstitutionalNodeKind::Organization),
            "GB-COH",
            "09506232",
        );
        let unused = qualified_link(
            "link:unused",
            node("supplier:x", InstitutionalNodeKind::Organization),
            node("supplier:y", InstitutionalNodeKind::LegalEntity),
            "US-DE",
            "9999999",
        );
        let result = EquivalenceViewContract::procurement_supplier_concentration(
            &edges,
            &[used, unused],
            context("projection:lineage"),
        )
        .expect("valid independent components");
        assert_eq!(result.applied_identity_link_refs, vec!["link:used"]);
        assert_eq!(result.applied_components.len(), 1);
    }
}

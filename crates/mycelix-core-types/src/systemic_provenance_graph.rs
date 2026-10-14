// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Ancestry-aware provenance graph semantics.
//!
//! This layer strengthens pairwise provenance-family independence by accounting
//! for known derivation/common-upstream ancestry and lineage completeness.
//!
//! Core invariants:
//!
//! ```text
//! explicit independence claim != independence when known ancestry contradicts it
//! shared known ancestor => not independent
//! incomplete lineage != proven independence
//! unknown lineage != independent lineage
//! provenance cycle != valid evidence ancestry
//! ```

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use std::collections::{HashMap, HashSet};

use crate::systemic_provenance::{
    ImportReceipt, IndependenceAssessment, IndependenceStatus, ProvenanceFamilyId,
};

/// Whether the modeled upstream lineage for a provenance family is sufficiently
/// complete for pairwise independence to be established.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum LineageCompleteness {
    /// All upstream families relevant to the current independence analysis are
    /// claimed to be represented.
    Complete,
    /// Some upstream lineage is known to be omitted.
    Incomplete,
    /// Completeness has not been established.
    #[default]
    Unknown,
}

/// One provenance family plus its known direct upstream families.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FamilyLineage {
    pub family_id: ProvenanceFamilyId,
    pub parent_families: Vec<ProvenanceFamilyId>,
    pub completeness: LineageCompleteness,
}

impl FamilyLineage {
    pub fn new(
        family_id: ProvenanceFamilyId,
        parent_families: Vec<ProvenanceFamilyId>,
        completeness: LineageCompleteness,
    ) -> Self {
        Self {
            family_id,
            parent_families,
            completeness,
        }
    }

    /// Construct lineage from an import receipt while requiring the caller to
    /// state whether the upstream family list is complete for independence use.
    pub fn from_receipt(receipt: &ImportReceipt, completeness: LineageCompleteness) -> Self {
        Self {
            family_id: receipt.provenance_family.clone(),
            parent_families: receipt.parent_families.clone(),
            completeness,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProvenanceGraphError {
    SelfDependency(ProvenanceFamilyId),
    CycleDetected,
}

/// Directed family ancestry graph: child -> known direct upstream parents.
#[derive(Debug, Clone, Default)]
pub struct ProvenanceGraph {
    parents: HashMap<ProvenanceFamilyId, HashSet<ProvenanceFamilyId>>,
    completeness: HashMap<ProvenanceFamilyId, LineageCompleteness>,
}

impl ProvenanceGraph {
    pub fn from_lineages(lineages: &[FamilyLineage]) -> Result<Self, ProvenanceGraphError> {
        let mut graph = Self::default();

        for lineage in lineages {
            graph.parents.entry(lineage.family_id.clone()).or_default();
            graph
                .completeness
                .insert(lineage.family_id.clone(), lineage.completeness);

            for parent in &lineage.parent_families {
                if parent == &lineage.family_id {
                    return Err(ProvenanceGraphError::SelfDependency(
                        lineage.family_id.clone(),
                    ));
                }
                graph.parents.entry(parent.clone()).or_default();
                graph
                    .parents
                    .get_mut(&lineage.family_id)
                    .expect("family inserted above")
                    .insert(parent.clone());
            }
        }

        if graph.has_cycle() {
            return Err(ProvenanceGraphError::CycleDetected);
        }

        Ok(graph)
    }

    pub fn lineage_completeness(&self, family: &ProvenanceFamilyId) -> LineageCompleteness {
        self.completeness
            .get(family)
            .copied()
            .unwrap_or(LineageCompleteness::Unknown)
    }

    /// Known ancestors including the family itself.
    pub fn known_ancestors(&self, family: &ProvenanceFamilyId) -> HashSet<ProvenanceFamilyId> {
        let mut ancestors = HashSet::new();
        let mut stack = vec![family.clone()];

        while let Some(current) = stack.pop() {
            if !ancestors.insert(current.clone()) {
                continue;
            }
            if let Some(parents) = self.parents.get(&current) {
                stack.extend(parents.iter().cloned());
            }
        }

        ancestors
    }

    /// Whether two families have a known ancestral dependency, including the
    /// case where one is an ancestor of the other.
    pub fn shares_known_ancestry(
        &self,
        left: &ProvenanceFamilyId,
        right: &ProvenanceFamilyId,
    ) -> bool {
        let left_ancestors = self.known_ancestors(left);
        let right_ancestors = self.known_ancestors(right);
        left_ancestors
            .iter()
            .any(|ancestor| right_ancestors.contains(ancestor))
    }

    fn has_cycle(&self) -> bool {
        let mut visiting = HashSet::new();
        let mut visited = HashSet::new();

        self.parents.keys().any(|node| {
            Self::visit_for_cycle(node, &self.parents, &mut visiting, &mut visited)
        })
    }

    fn visit_for_cycle(
        node: &ProvenanceFamilyId,
        parents: &HashMap<ProvenanceFamilyId, HashSet<ProvenanceFamilyId>>,
        visiting: &mut HashSet<ProvenanceFamilyId>,
        visited: &mut HashSet<ProvenanceFamilyId>,
    ) -> bool {
        if visited.contains(node) {
            return false;
        }
        if !visiting.insert(node.clone()) {
            return true;
        }

        if let Some(upstream) = parents.get(node) {
            for parent in upstream {
                if Self::visit_for_cycle(parent, parents, visiting, visited) {
                    return true;
                }
            }
        }

        visiting.remove(node);
        visited.insert(node.clone());
        false
    }
}

/// Resolve independence with ancestry and lineage-completeness safeguards.
///
/// Known shared ancestry always dominates an explicit `Independent` assessment.
/// An `Independent` assessment is admitted only when both lineages are marked
/// complete and the assessment carries at least one basis reference.
pub fn resolve_independence_with_graph(
    left: &ProvenanceFamilyId,
    right: &ProvenanceFamilyId,
    assessments: &[IndependenceAssessment],
    graph: &ProvenanceGraph,
) -> IndependenceStatus {
    if left == right || graph.shares_known_ancestry(left, right) {
        return IndependenceStatus::NotIndependent;
    }

    let latest = assessments
        .iter()
        .rev()
        .find(|assessment| assessment.applies_to(left, right));

    if matches!(latest.map(|assessment| assessment.status), Some(IndependenceStatus::NotIndependent)) {
        return IndependenceStatus::NotIndependent;
    }

    if graph.lineage_completeness(left) != LineageCompleteness::Complete
        || graph.lineage_completeness(right) != LineageCompleteness::Complete
    {
        return IndependenceStatus::Unknown;
    }

    match latest {
        Some(assessment)
            if assessment.status == IndependenceStatus::Independent
                && !assessment.basis.is_empty() =>
        {
            IndependenceStatus::Independent
        }
        _ => IndependenceStatus::Unknown,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::systemic_provenance::IndependenceAssessment;

    fn family(id: &str) -> ProvenanceFamilyId {
        ProvenanceFamilyId::new(id).unwrap()
    }

    fn independent(left: &ProvenanceFamilyId, right: &ProvenanceFamilyId) -> IndependenceAssessment {
        IndependenceAssessment {
            left_family: left.clone(),
            right_family: right.clone(),
            status: IndependenceStatus::Independent,
            assessed_at: 100,
            basis: vec!["independent primary acquisition evidence".into()],
        }
    }

    #[test]
    fn rejects_direct_self_dependency() {
        let a = family("family:a");
        let result = ProvenanceGraph::from_lineages(&[FamilyLineage::new(
            a.clone(),
            vec![a],
            LineageCompleteness::Complete,
        )]);
        assert!(matches!(result, Err(ProvenanceGraphError::SelfDependency(_))));
    }

    #[test]
    fn rejects_multi_node_cycle() {
        let a = family("family:a");
        let b = family("family:b");
        let graph = ProvenanceGraph::from_lineages(&[
            FamilyLineage::new(a.clone(), vec![b.clone()], LineageCompleteness::Complete),
            FamilyLineage::new(b, vec![a], LineageCompleteness::Complete),
        ]);
        assert_eq!(graph.unwrap_err(), ProvenanceGraphError::CycleDetected);
    }

    #[test]
    fn shared_known_ancestor_blocks_explicit_independence() {
        let upstream = family("family:upstream");
        let a = family("family:a");
        let b = family("family:b");
        let graph = ProvenanceGraph::from_lineages(&[
            FamilyLineage::new(
                upstream.clone(),
                vec![],
                LineageCompleteness::Complete,
            ),
            FamilyLineage::new(
                a.clone(),
                vec![upstream.clone()],
                LineageCompleteness::Complete,
            ),
            FamilyLineage::new(
                b.clone(),
                vec![upstream],
                LineageCompleteness::Complete,
            ),
        ])
        .unwrap();

        assert_eq!(
            resolve_independence_with_graph(&a, &b, &[independent(&a, &b)], &graph),
            IndependenceStatus::NotIndependent
        );
    }

    #[test]
    fn parent_child_families_are_not_independent() {
        let parent = family("family:parent");
        let child = family("family:child");
        let graph = ProvenanceGraph::from_lineages(&[
            FamilyLineage::new(parent.clone(), vec![], LineageCompleteness::Complete),
            FamilyLineage::new(
                child.clone(),
                vec![parent.clone()],
                LineageCompleteness::Complete,
            ),
        ])
        .unwrap();

        assert_eq!(
            resolve_independence_with_graph(
                &parent,
                &child,
                &[independent(&parent, &child)],
                &graph,
            ),
            IndependenceStatus::NotIndependent
        );
    }

    #[test]
    fn incomplete_lineage_blocks_independence_promotion() {
        let a = family("family:a");
        let b = family("family:b");
        let graph = ProvenanceGraph::from_lineages(&[
            FamilyLineage::new(a.clone(), vec![], LineageCompleteness::Incomplete),
            FamilyLineage::new(b.clone(), vec![], LineageCompleteness::Complete),
        ])
        .unwrap();

        assert_eq!(
            resolve_independence_with_graph(&a, &b, &[independent(&a, &b)], &graph),
            IndependenceStatus::Unknown
        );
    }

    #[test]
    fn empty_independence_basis_does_not_promote() {
        let a = family("family:a");
        let b = family("family:b");
        let graph = ProvenanceGraph::from_lineages(&[
            FamilyLineage::new(a.clone(), vec![], LineageCompleteness::Complete),
            FamilyLineage::new(b.clone(), vec![], LineageCompleteness::Complete),
        ])
        .unwrap();
        let assessment = IndependenceAssessment {
            left_family: a.clone(),
            right_family: b.clone(),
            status: IndependenceStatus::Independent,
            assessed_at: 100,
            basis: vec![],
        };

        assert_eq!(
            resolve_independence_with_graph(&a, &b, &[assessment], &graph),
            IndependenceStatus::Unknown
        );
    }

    #[test]
    fn complete_unrelated_lineages_can_admit_explicit_independence() {
        let a = family("family:a");
        let b = family("family:b");
        let graph = ProvenanceGraph::from_lineages(&[
            FamilyLineage::new(a.clone(), vec![], LineageCompleteness::Complete),
            FamilyLineage::new(b.clone(), vec![], LineageCompleteness::Complete),
        ])
        .unwrap();

        assert_eq!(
            resolve_independence_with_graph(&a, &b, &[independent(&a, &b)], &graph),
            IndependenceStatus::Independent
        );
    }
}

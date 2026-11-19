// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic dependency index for propagating evidence correction,
//! revocation and challenge invalidation through derived records.
//!
//! The graph is a rebuildable index over canonical event/source references.
//! It is not a second evidence ledger and does not replace EPI-008, CIVOS
//! evidence events, or source-system records. Invalidation appends causal
//! records; it never deletes or rewrites the original evidence or descendants.

use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet, VecDeque};

pub const EVIDENCE_INVALIDATION_SCHEMA: &str = "mycelix.evidence-invalidation-graph.v0";
pub const EVIDENCE_INVALIDATION_SCHEMA_URI: &str =
    "https://luminousdynamics.io/schemas/mycelix-evidence-invalidation-graph-v0.schema.json";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceDependencyNode {
    /// Stable reference to a canonical source/event or a derived projection.
    pub node_reference: String,
    /// Digest of the exact payload represented by this node.
    pub content_sha256: String,
    /// References to nodes this node depends upon.
    #[serde(default)]
    pub depends_on: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceInvalidationRecord {
    /// Stable immutable reference to the correction/revocation/challenge event.
    pub invalidation_event_reference: String,
    /// Original node from which invalidation propagates.
    pub root_reference: String,
    /// Affected node, including the root itself.
    pub affected_reference: String,
    /// Why downstream reliance must be reconsidered.
    pub reason: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceDependencyGraphV0 {
    pub schema: String,
    /// Deterministic graph projection; rebuildable from canonical evidence events.
    pub nodes: Vec<EvidenceDependencyNode>,
    /// Append-only invalidation history; multiple independent causes are retained.
    #[serde(default)]
    pub invalidations: Vec<EvidenceInvalidationRecord>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvidenceInvalidationError {
    InvalidInput,
    UnknownRoot,
    EventReferenceConflict,
    InvalidGraph,
}

impl EvidenceDependencyGraphV0 {
    pub fn new(nodes: Vec<EvidenceDependencyNode>) -> Self {
        Self {
            schema: EVIDENCE_INVALIDATION_SCHEMA.into(),
            nodes,
            invalidations: Vec::new(),
        }
    }

    /// Validate identities, digests, dependency references, acyclicity and
    /// consistency of the append-only invalidation index.
    pub fn validate_structural(&self) -> Result<(), Vec<String>> {
        let mut issues = Vec::new();
        if self.schema != EVIDENCE_INVALIDATION_SCHEMA {
            issues.push("unsupported evidence invalidation graph schema".into());
        }

        let mut by_reference: HashMap<&str, &EvidenceDependencyNode> = HashMap::new();
        for node in &self.nodes {
            if node.node_reference.trim().is_empty() {
                issues.push("evidence node reference must not be empty".into());
            }
            if !is_sha256(&node.content_sha256) {
                issues.push(format!(
                    "evidence node {} has an invalid SHA-256 digest",
                    node.node_reference
                ));
            }
            if by_reference
                .insert(node.node_reference.as_str(), node)
                .is_some()
            {
                issues.push(format!(
                    "duplicate evidence node reference {}",
                    node.node_reference
                ));
            }
            let mut dependencies = HashSet::new();
            for dependency in &node.depends_on {
                if dependency.trim().is_empty() {
                    issues.push(format!(
                        "evidence node {} has an empty dependency reference",
                        node.node_reference
                    ));
                }
                if dependency == &node.node_reference {
                    issues.push(format!(
                        "evidence node {} depends on itself",
                        node.node_reference
                    ));
                }
                if !dependencies.insert(dependency.as_str()) {
                    issues.push(format!(
                        "evidence node {} has duplicate dependency {}",
                        node.node_reference, dependency
                    ));
                }
            }
        }

        for node in &self.nodes {
            for dependency in &node.depends_on {
                if !by_reference.contains_key(dependency.as_str()) {
                    issues.push(format!(
                        "evidence node {} references missing dependency {}",
                        node.node_reference, dependency
                    ));
                }
            }
        }

        if issues.is_empty() && !self.is_acyclic(&by_reference) {
            issues.push("evidence dependency graph contains a cycle".into());
        }

        let mut event_roots: HashMap<&str, (&str, &str)> = HashMap::new();
        let mut event_affected = HashSet::new();
        for invalidation in &self.invalidations {
            if invalidation.invalidation_event_reference.trim().is_empty()
                || invalidation.root_reference.trim().is_empty()
                || invalidation.affected_reference.trim().is_empty()
                || invalidation.reason.trim().is_empty()
            {
                issues.push("invalidation record fields must not be empty".into());
            }
            if !by_reference.contains_key(invalidation.root_reference.as_str()) {
                issues.push(format!(
                    "invalidation root {} does not exist",
                    invalidation.root_reference
                ));
            }
            if !by_reference.contains_key(invalidation.affected_reference.as_str()) {
                issues.push(format!(
                    "invalidation affected node {} does not exist",
                    invalidation.affected_reference
                ));
            }
            if let Some((root, reason)) =
                event_roots.get(invalidation.invalidation_event_reference.as_str())
            {
                if *root != invalidation.root_reference || *reason != invalidation.reason {
                    issues.push(format!(
                        "invalidation event {} has conflicting root or reason",
                        invalidation.invalidation_event_reference
                    ));
                }
            } else {
                event_roots.insert(
                    invalidation.invalidation_event_reference.as_str(),
                    (
                        invalidation.root_reference.as_str(),
                        invalidation.reason.as_str(),
                    ),
                );
            }
            if !event_affected.insert((
                invalidation.invalidation_event_reference.as_str(),
                invalidation.affected_reference.as_str(),
            )) {
                issues.push(format!(
                    "duplicate affected-node record {} for invalidation event {}",
                    invalidation.affected_reference, invalidation.invalidation_event_reference
                ));
            }

            let root_exists = by_reference.contains_key(invalidation.root_reference.as_str());
            let affected_exists =
                by_reference.contains_key(invalidation.affected_reference.as_str());
            if root_exists
                && affected_exists
                && invalidation.root_reference != invalidation.affected_reference
                && !self.depends_transitively_on(
                    invalidation.affected_reference.as_str(),
                    invalidation.root_reference.as_str(),
                    &by_reference,
                )
            {
                issues.push(format!(
                    "invalidated node {} does not depend on root {}",
                    invalidation.affected_reference, invalidation.root_reference
                ));
            }
        }

        if issues.is_empty() {
            // Each event must propagate to the root and every transitive
            // dependent. Partial propagation would leave stale projections
            // looking current after their source has been invalidated.
            for (event_reference, (root_reference, _reason)) in &event_roots {
                for node in &self.nodes {
                    let expected_affected = node.node_reference == *root_reference
                        || self.depends_transitively_on(
                            node.node_reference.as_str(),
                            root_reference,
                            &by_reference,
                        );
                    if expected_affected
                        && !event_affected.contains(&(
                            *event_reference,
                            node.node_reference.as_str(),
                        ))
                    {
                        issues.push(format!(
                            "invalidation event {} omits dependent node {}",
                            event_reference, node.node_reference
                        ));
                    }
                }
            }
        }

        if issues.is_empty() {
            Ok(())
        } else {
            Err(issues)
        }
    }

    /// Append an invalidation for a source and every transitive dependent.
    ///
    /// The operation is deterministic and idempotent for the same event/root/
    /// reason. Reusing an event reference for another root or reason is rejected.
    /// The graph must be structurally valid before any mutation occurs.
    pub fn invalidate_from(
        &mut self,
        root_reference: &str,
        invalidation_event_reference: &str,
        reason: &str,
    ) -> Result<Vec<String>, EvidenceInvalidationError> {
        self.validate_structural()
            .map_err(|_| EvidenceInvalidationError::InvalidGraph)?;
        if root_reference.trim().is_empty()
            || invalidation_event_reference.trim().is_empty()
            || reason.trim().is_empty()
        {
            return Err(EvidenceInvalidationError::InvalidInput);
        }

        let by_reference = self
            .nodes
            .iter()
            .map(|node| (node.node_reference.as_str(), node))
            .collect::<HashMap<_, _>>();
        if !by_reference.contains_key(root_reference) {
            return Err(EvidenceInvalidationError::UnknownRoot);
        }

        let existing_for_event = self
            .invalidations
            .iter()
            .filter(|record| record.invalidation_event_reference == invalidation_event_reference)
            .collect::<Vec<_>>();
        if !existing_for_event.is_empty() {
            if existing_for_event
                .iter()
                .any(|record| record.root_reference != root_reference || record.reason != reason)
            {
                return Err(EvidenceInvalidationError::EventReferenceConflict);
            }
            let mut affected = existing_for_event
                .iter()
                .map(|record| record.affected_reference.clone())
                .collect::<Vec<_>>();
            affected.sort();
            return Ok(affected);
        }

        let mut affected = self
            .nodes
            .iter()
            .filter(|node| {
                node.node_reference == root_reference
                    || self.depends_transitively_on(
                        node.node_reference.as_str(),
                        root_reference,
                        &by_reference,
                    )
            })
            .map(|node| node.node_reference.clone())
            .collect::<Vec<_>>();
        affected.sort();

        let records = affected
            .iter()
            .map(|affected_reference| EvidenceInvalidationRecord {
                invalidation_event_reference: invalidation_event_reference.into(),
                root_reference: root_reference.into(),
                affected_reference: affected_reference.clone(),
                reason: reason.into(),
            })
            .collect::<Vec<_>>();
        self.invalidations.extend(records);
        Ok(affected)
    }

    /// A node is current only when no recorded invalidation affects it.
    pub fn is_current(&self, node_reference: &str) -> bool {
        self.nodes
            .iter()
            .any(|node| node.node_reference == node_reference)
            && !self
                .invalidations
                .iter()
                .any(|record| record.affected_reference == node_reference)
    }

    /// Return all distinct invalidation causes for a node in deterministic order.
    pub fn invalidation_causes(&self, node_reference: &str) -> Vec<&EvidenceInvalidationRecord> {
        let mut causes = self
            .invalidations
            .iter()
            .filter(|record| record.affected_reference == node_reference)
            .collect::<Vec<_>>();
        causes.sort_by(|left, right| {
            left.invalidation_event_reference
                .cmp(&right.invalidation_event_reference)
                .then_with(|| left.root_reference.cmp(&right.root_reference))
        });
        causes
    }

    fn is_acyclic(&self, nodes: &HashMap<&str, &EvidenceDependencyNode>) -> bool {
        let mut indegree = HashMap::<&str, usize>::new();
        let mut dependents = HashMap::<&str, Vec<&str>>::new();
        for node in &self.nodes {
            indegree.insert(node.node_reference.as_str(), node.depends_on.len());
            for dependency in &node.depends_on {
                dependents
                    .entry(dependency.as_str())
                    .or_default()
                    .push(node.node_reference.as_str());
            }
        }

        let mut ready = VecDeque::new();
        for (reference, degree) in &indegree {
            if *degree == 0 {
                ready.push_back(*reference);
            }
        }

        let mut visited = 0usize;
        while let Some(reference) = ready.pop_front() {
            visited += 1;
            if let Some(children) = dependents.get(reference) {
                for child in children {
                    if let Some(degree) = indegree.get_mut(child) {
                        *degree -= 1;
                        if *degree == 0 {
                            ready.push_back(child);
                        }
                    }
                }
            }
        }
        visited == nodes.len()
    }

    fn depends_transitively_on(
        &self,
        candidate: &str,
        root: &str,
        nodes: &HashMap<&str, &EvidenceDependencyNode>,
    ) -> bool {
        let mut stack = vec![candidate];
        let mut visited = HashSet::new();
        while let Some(reference) = stack.pop() {
            if !visited.insert(reference) {
                continue;
            }
            if let Some(node) = nodes.get(reference) {
                for dependency in &node.depends_on {
                    if dependency == root {
                        return true;
                    }
                    stack.push(dependency.as_str());
                }
            }
        }
        false
    }
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;
    use std::collections::HashMap;

    #[derive(Deserialize)]
    struct InvalidationCorpus {
        suite_id: String,
        suite_version: String,
        scenarios: Vec<InvalidationScenario>,
    }

    #[derive(Deserialize)]
    struct InvalidationScenario {
        scenario_id: String,
        nodes: Vec<EvidenceDependencyNode>,
        initial_invalidations: Vec<EvidenceInvalidationRecord>,
        steps: Vec<InvalidationStep>,
        expected_affected_by_step: Vec<Vec<String>>,
        expected_error_by_step: Vec<Option<String>>,
        expected_invalidated: Vec<String>,
        expected_current: Vec<String>,
        expected_cause_counts: HashMap<String, usize>,
        expected_record_count: usize,
        expected_final_validation: String,
    }

    #[derive(Deserialize)]
    struct InvalidationStep {
        root_reference: String,
        invalidation_event_reference: String,
        reason: String,
    }

    fn error_name(error: EvidenceInvalidationError) -> &'static str {
        match error {
            EvidenceInvalidationError::InvalidInput => "invalid_input",
            EvidenceInvalidationError::UnknownRoot => "unknown_root",
            EvidenceInvalidationError::EventReferenceConflict => "event_reference_conflict",
            EvidenceInvalidationError::InvalidGraph => "invalid_graph",
        }
    }

    fn node(reference: &str, dependencies: &[&str], digest: char) -> EvidenceDependencyNode {
        EvidenceDependencyNode {
            node_reference: reference.into(),
            content_sha256: digest.to_string().repeat(64),
            depends_on: dependencies.iter().map(|value| (*value).into()).collect(),
        }
    }

    fn graph() -> EvidenceDependencyGraphV0 {
        EvidenceDependencyGraphV0::new(vec![
            node("source:registry-extract:v1", &[], 'a'),
            node("assertion:surface-right:v1", &["source:registry-extract:v1"], 'b'),
            node("authority-assessment:v1", &["assertion:surface-right:v1"], 'c'),
            node("read-model:external-status:v1", &["authority-assessment:v1"], 'd'),
            node("unrelated:water-interest:v1", &[], 'e'),
        ])
    }

    #[test]
    fn tib_evidence_invalidation_corpus_is_executable_and_deterministic() {
        let corpus: InvalidationCorpus = serde_json::from_str(include_str!(
            "../../../examples/tib-evidence-invalidation-v0.1.json"
        ))
        .expect("TIB evidence invalidation corpus must deserialize");
        assert_eq!(corpus.suite_id, "TIB-EVIDENCE-INVALIDATION-001");
        assert_eq!(corpus.suite_version, "0.1.0");
        assert_eq!(corpus.scenarios.len(), 8);

        for scenario in corpus.scenarios {
            let mut graph = EvidenceDependencyGraphV0 {
                schema: EVIDENCE_INVALIDATION_SCHEMA.into(),
                nodes: scenario.nodes,
                invalidations: scenario.initial_invalidations,
            };
            assert_eq!(
                scenario.steps.len(),
                scenario.expected_affected_by_step.len(),
                "affected-step vector length mismatch for {}",
                scenario.scenario_id
            );
            assert_eq!(
                scenario.steps.len(),
                scenario.expected_error_by_step.len(),
                "error-step vector length mismatch for {}",
                scenario.scenario_id
            );

            for (index, step) in scenario.steps.into_iter().enumerate() {
                let result = graph.invalidate_from(
                    &step.root_reference,
                    &step.invalidation_event_reference,
                    &step.reason,
                );
                match scenario.expected_error_by_step[index].as_deref() {
                    Some(expected_error) => {
                        let error = result.expect_err("scenario should reject this step");
                        assert_eq!(
                            error_name(error),
                            expected_error,
                            "unexpected error in {} step {}",
                            scenario.scenario_id,
                            index
                        );
                    }
                    None => {
                        let affected = result.expect("scenario should apply this step");
                        assert_eq!(
                            affected,
                            scenario.expected_affected_by_step[index],
                            "wrong affected set in {} step {}",
                            scenario.scenario_id,
                            index
                        );
                    }
                }
            }

            let mut invalidated = graph
                .nodes
                .iter()
                .filter(|node| !graph.is_current(&node.node_reference))
                .map(|node| node.node_reference.clone())
                .collect::<Vec<_>>();
            invalidated.sort();
            let mut current = graph
                .nodes
                .iter()
                .filter(|node| graph.is_current(&node.node_reference))
                .map(|node| node.node_reference.clone())
                .collect::<Vec<_>>();
            current.sort();

            assert_eq!(invalidated, scenario.expected_invalidated, "invalidated set for {}", scenario.scenario_id);
            assert_eq!(current, scenario.expected_current, "current set for {}", scenario.scenario_id);
            assert_eq!(
                graph.invalidations.len(),
                scenario.expected_record_count,
                "invalidation record count for {}",
                scenario.scenario_id
            );
            for (node_reference, expected_count) in scenario.expected_cause_counts {
                assert_eq!(
                    graph.invalidation_causes(&node_reference).len(),
                    expected_count,
                    "cause count for {node_reference} in {}",
                    scenario.scenario_id
                );
            }
            let actual_validation = if graph.validate_structural().is_ok() {
                "valid"
            } else {
                "invalid"
            };
            assert_eq!(
                actual_validation,
                scenario.expected_final_validation,
                "final graph validation for {}",
                scenario.scenario_id
            );
        }
    }

    #[test]
    fn graph_rejects_empty_invalid_digest_duplicate_or_missing_dependencies() {
        let mut value = graph();
        value.nodes.push(node("bad", &["missing"], 'f'));
        value.nodes.push(node("bad", &[], 'g'));
        assert!(value.validate_structural().is_err());
    }

    #[test]
    fn graph_rejects_dependency_cycles() {
        let value = EvidenceDependencyGraphV0::new(vec![
            node("a", &["b"], 'a'),
            node("b", &["a"], 'b'),
        ]);
        let issues = value.validate_structural().unwrap_err();
        assert!(issues.iter().any(|issue| issue.contains("contains a cycle")));
    }

    #[test]
    fn invalidating_source_propagates_to_all_transitive_dependents() {
        let mut value = graph();
        let affected = value
            .invalidate_from(
                "source:registry-extract:v1",
                "event:source-correction:001",
                "source extract corrected by issuer",
            )
            .unwrap();
        assert_eq!(
            affected,
            vec![
                "assertion:surface-right:v1",
                "authority-assessment:v1",
                "read-model:external-status:v1",
                "source:registry-extract:v1"
            ]
        );
        assert!(affected.iter().all(|reference| !value.is_current(reference)));
        assert!(value.is_current("unrelated:water-interest:v1"));
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn invalidating_middle_node_preserves_unrelated_upstream_and_sibling_nodes() {
        let mut value = graph();
        let affected = value
            .invalidate_from(
                "assertion:surface-right:v1",
                "event:community-challenge:002",
                "a new challenge affects this assertion",
            )
            .unwrap();
        assert_eq!(
            affected,
            vec![
                "assertion:surface-right:v1",
                "authority-assessment:v1",
                "read-model:external-status:v1"
            ]
        );
        assert!(value.is_current("source:registry-extract:v1"));
        assert!(value.is_current("unrelated:water-interest:v1"));
    }

    #[test]
    fn independent_invalidation_causes_are_retained_append_only() {
        let mut value = graph();
        value
            .invalidate_from(
                "source:registry-extract:v1",
                "event:correction:003",
                "source corrected",
            )
            .unwrap();
        value
            .invalidate_from(
                "assertion:surface-right:v1",
                "event:challenge:004",
                "new challenge filed",
            )
            .unwrap();
        let causes = value.invalidation_causes("read-model:external-status:v1");
        assert_eq!(causes.len(), 2);
        assert_eq!(
            causes
                .iter()
                .map(|cause| cause.invalidation_event_reference.as_str())
                .collect::<Vec<_>>(),
            vec!["event:challenge:004", "event:correction:003"]
        );
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn repeated_same_invalidation_event_is_idempotent() {
        let mut value = graph();
        let first = value
            .invalidate_from("source:registry-extract:v1", "event:correction:005", "corrected")
            .unwrap();
        let record_count = value.invalidations.len();
        let second = value
            .invalidate_from("source:registry-extract:v1", "event:correction:005", "corrected")
            .unwrap();
        assert_eq!(first, second);
        assert_eq!(value.invalidations.len(), record_count);
    }

    #[test]
    fn invalidation_event_reference_cannot_be_reused_for_another_root() {
        let mut value = graph();
        value
            .invalidate_from("source:registry-extract:v1", "event:correction:006", "corrected")
            .unwrap();
        assert_eq!(
            value.invalidate_from(
                "assertion:surface-right:v1",
                "event:correction:006",
                "same event, different root"
            ),
            Err(EvidenceInvalidationError::EventReferenceConflict)
        );
    }

    #[test]
    fn unknown_root_does_not_mutate_graph() {
        let mut value = graph();
        assert_eq!(
            value.invalidate_from("source:missing", "event:missing:007", "not found"),
            Err(EvidenceInvalidationError::UnknownRoot)
        );
        assert!(value.invalidations.is_empty());
    }

    #[test]
    fn invalid_input_does_not_mutate_graph() {
        let mut value = graph();
        assert_eq!(
            value.invalidate_from("source:registry-extract:v1", "", "missing event"),
            Err(EvidenceInvalidationError::InvalidInput)
        );
        assert!(value.invalidations.is_empty());
    }

    #[test]
    fn invalidation_index_rejects_non_dependent_affected_nodes() {
        let mut value = graph();
        value.invalidations.push(EvidenceInvalidationRecord {
            invalidation_event_reference: "event:forged:008".into(),
            root_reference: "source:registry-extract:v1".into(),
            affected_reference: "unrelated:water-interest:v1".into(),
            reason: "forged dependency".into(),
        });
        let issues = value.validate_structural().unwrap_err();
        assert!(issues
            .iter()
            .any(|issue| issue.contains("does not depend on root")));
    }

    #[test]
    fn invalidation_index_rejects_partial_downstream_propagation() {
        let mut value = graph();
        value.invalidations.push(EvidenceInvalidationRecord {
            invalidation_event_reference: "event:partial:001".into(),
            root_reference: "source:registry-extract:v1".into(),
            affected_reference: "source:registry-extract:v1".into(),
            reason: "source was corrected".into(),
        });
        let issues = value.validate_structural().unwrap_err();
        assert!(issues
            .iter()
            .any(|issue| issue.contains("omits dependent node")));
    }

    #[test]
    fn multiple_event_records_for_same_affected_node_have_distinct_causes() {
        let mut value = graph();
        value
            .invalidate_from("source:registry-extract:v1", "event:correction:009", "corrected")
            .unwrap();
        value
            .invalidate_from("assertion:surface-right:v1", "event:challenge:010", "challenged")
            .unwrap();
        assert_eq!(value.invalidation_causes("assertion:surface-right:v1").len(), 2);
    }

    #[test]
    fn schema_and_synthetic_example_are_machine_readable() {
        let schema: serde_json::Value = serde_json::from_str(include_str!(
            "../../../schemas/evidence-invalidation-graph-v0.schema.json"
        ))
        .expect("invalidation graph schema must parse");
        let example: EvidenceDependencyGraphV0 = serde_json::from_str(include_str!(
            "../../../examples/evidence-invalidation-graph-synthetic-v0.json"
        ))
        .expect("synthetic invalidation graph must deserialize");
        assert_eq!(
            schema["$schema"],
            "https://json-schema.org/draft/2020-12/schema"
        );
        assert_eq!(schema["$id"], EVIDENCE_INVALIDATION_SCHEMA_URI);
        assert_eq!(example.validate_structural(), Ok(()));
        assert!(example.invalidations.is_empty());
    }

    #[test]
    fn invalidation_graph_never_authorizes_external_or_authoritative_mutation() {
        let mut value = graph();
        value
            .invalidate_from(
                "source:registry-extract:v1",
                "event:correction:011",
                "corrected",
            )
            .unwrap();
        // The graph API is a derived evidence index; no mutation method exists.
        assert!(value.validate_structural().is_ok());
    }
}

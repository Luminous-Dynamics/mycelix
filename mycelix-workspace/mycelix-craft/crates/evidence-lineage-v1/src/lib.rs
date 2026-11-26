#![forbid(unsafe_code)]

//! Deterministic impact propagation for versioned evidence and decision artifacts.
//!
//! This crate evaluates an explicitly supplied artifact dependency graph. It does
//! not authenticate actors, verify signatures, establish source truth, or grant
//! authority. Those checks belong to the host that admits change events.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub type DigestV1 = [u8; 32];

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct ArtifactRefV1 {
    pub artifact_id: String,
    pub revision: u64,
    pub digest: DigestV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArtifactKindV1 {
    Source,
    Assumption,
    Inference,
    NeedProposal,
    Claim,
    ClaimSupportResult,
    AuthorityDecision,
    FederationAgreement,
    Project,
    Requirement,
    WorkPackage,
    Measurement,
    Verification,
    Policy,
    Credential,
    Other,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DependencyRelationV1 {
    SourcedFrom,
    DerivedFrom,
    SemanticallySupportedBy,
    Ratifies,
    Authorizes,
    Implements,
    Verifies,
    Measures,
    GovernedBy,
    Other,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImpactPolicyV1 {
    /// The dependent artifact must be re-evaluated before it is relied on again.
    RevalidateOnChange,
    /// The dependent artifact is unusable until replaced by a new artifact.
    InvalidateOnChange,
    /// The relation is informational and must not confer or remove validity.
    Informational,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ArtifactDependencyV1 {
    pub reference: ArtifactRefV1,
    pub relation: DependencyRelationV1,
    pub impact_policy: ImpactPolicyV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ArtifactNodeV1 {
    pub reference: ArtifactRefV1,
    pub kind: ArtifactKindV1,
    pub dependencies: Vec<ArtifactDependencyV1>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChangeReasonV1 {
    Corrected,
    Superseded,
    Revoked,
    Expired,
    Withdrawn,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChangeEventV1 {
    pub event_id: String,
    /// Host-assigned, unique sequence number within the event stream.
    pub sequence: u64,
    pub target: ArtifactRefV1,
    pub reason: ChangeReasonV1,
    /// Required for Corrected and Superseded; must be an existing higher revision
    /// of the same logical artifact.
    pub replacement: Option<ArtifactRefV1>,
    pub occurred_at: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArtifactStateV1 {
    Current,
    NeedsRevalidation,
    Invalidated,
    Superseded,
    Revoked,
    Expired,
    Withdrawn,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ArtifactStateRecordV1 {
    pub reference: ArtifactRefV1,
    pub state: ArtifactStateV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ImpactRecordV1 {
    pub affected_artifact: ArtifactRefV1,
    pub dependency: ArtifactRefV1,
    pub upstream_state: ArtifactStateV1,
    pub policy: ImpactPolicyV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct LineageEvaluationV1 {
    /// Sorted by artifact identifier and revision.
    pub artifacts: Vec<ArtifactStateRecordV1>,
    /// Sorted deterministically by affected artifact and dependency.
    pub impacts: Vec<ImpactRecordV1>,
    /// Unique events, sorted by their host-assigned sequence.
    pub applied_event_ids: Vec<String>,
}

impl LineageEvaluationV1 {
    pub fn state_for(&self, reference: &ArtifactRefV1) -> Option<ArtifactStateV1> {
        self.artifacts
            .iter()
            .find(|record| record.reference == *reference)
            .map(|record| record.state)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineageErrorV1 {
    EmptyGraph,
    InvalidArtifactReference,
    DuplicateArtifactRevision,
    DuplicateDependency,
    MissingDependency,
    DependencyDigestMismatch,
    DependencyCycle,
    InvalidChangeEvent,
    UnknownChangeTarget,
    ChangeTargetDigestMismatch,
    DuplicateChangeSequence,
    ConflictingEventIdentifier,
    MultipleChangesForArtifact,
    InvalidReplacement,
}

impl fmt::Display for LineageErrorV1 {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for LineageErrorV1 {}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct ArtifactKey {
    artifact_id: String,
    revision: u64,
}

fn key(reference: &ArtifactRefV1) -> ArtifactKey {
    ArtifactKey {
        artifact_id: reference.artifact_id.clone(),
        revision: reference.revision,
    }
}

fn valid_reference(reference: &ArtifactRefV1) -> bool {
    !reference.artifact_id.trim().is_empty()
        && reference.artifact_id.trim() == reference.artifact_id
        && reference.revision > 0
        && reference.digest != [0; 32]
}

fn state_for_change(reason: ChangeReasonV1) -> ArtifactStateV1 {
    match reason {
        ChangeReasonV1::Corrected | ChangeReasonV1::Superseded => ArtifactStateV1::Superseded,
        ChangeReasonV1::Revoked => ArtifactStateV1::Revoked,
        ChangeReasonV1::Expired => ArtifactStateV1::Expired,
        ChangeReasonV1::Withdrawn => ArtifactStateV1::Withdrawn,
    }
}

/// Evaluate exact-version dependencies and propagate change impact.
///
/// This function is pure: it does not mutate artifacts, events, approvals, or
/// external authority. A stale artifact becomes usable again only when a host
/// creates a new artifact revision whose dependencies pin the newly validated
/// inputs.
pub fn evaluate_lineage(
    nodes: &[ArtifactNodeV1],
    events: &[ChangeEventV1],
) -> Result<LineageEvaluationV1, LineageErrorV1> {
    if nodes.is_empty() {
        return Err(LineageErrorV1::EmptyGraph);
    }

    let mut node_map: BTreeMap<ArtifactKey, &ArtifactNodeV1> = BTreeMap::new();
    for node in nodes {
        if !valid_reference(&node.reference) {
            return Err(LineageErrorV1::InvalidArtifactReference);
        }
        let node_key = key(&node.reference);
        if node_map.insert(node_key, node).is_some() {
            return Err(LineageErrorV1::DuplicateArtifactRevision);
        }
    }

    let mut indegree: BTreeMap<ArtifactKey, usize> = node_map
        .keys()
        .cloned()
        .map(|node_key| (node_key, 0))
        .collect();
    let mut dependents: BTreeMap<ArtifactKey, Vec<ArtifactKey>> = BTreeMap::new();

    for (node_key, node) in &node_map {
        let mut seen_dependencies = BTreeSet::new();
        for dependency in &node.dependencies {
            if !valid_reference(&dependency.reference) {
                return Err(LineageErrorV1::InvalidArtifactReference);
            }
            let dependency_key = key(&dependency.reference);
            if !seen_dependencies.insert(dependency_key.clone()) {
                return Err(LineageErrorV1::DuplicateDependency);
            }
            let Some(parent) = node_map.get(&dependency_key) else {
                return Err(LineageErrorV1::MissingDependency);
            };
            if parent.reference.digest != dependency.reference.digest {
                return Err(LineageErrorV1::DependencyDigestMismatch);
            }
            if let Some(count) = indegree.get_mut(node_key) {
                *count += 1;
            } else {
                return Err(LineageErrorV1::DuplicateArtifactRevision);
            }
            dependents
                .entry(dependency_key)
                .or_default()
                .push(node_key.clone());
        }
    }

    // Kahn's algorithm yields a deterministic parent-before-dependent traversal.
    let mut ready: BTreeSet<ArtifactKey> = indegree
        .iter()
        .filter(|(_, count)| **count == 0)
        .map(|(node_key, _)| (*node_key).clone())
        .collect();
    let mut topological_order = Vec::with_capacity(node_map.len());

    while let Some(node_key) = ready.iter().next().cloned() {
        ready.remove(&node_key);
        topological_order.push(node_key.clone());
        if let Some(children) = dependents.get(&node_key) {
            for child in children {
                let Some(count) = indegree.get_mut(child) else {
                    return Err(LineageErrorV1::DuplicateArtifactRevision);
                };
                *count -= 1;
                if *count == 0 {
                    ready.insert(child.clone());
                }
            }
        }
    }

    if topological_order.len() != node_map.len() {
        return Err(LineageErrorV1::DependencyCycle);
    }

    let mut unique_events: BTreeMap<String, ChangeEventV1> = BTreeMap::new();
    let mut sequence_owners: BTreeMap<u64, String> = BTreeMap::new();
    let mut target_events: BTreeMap<ArtifactKey, ChangeEventV1> = BTreeMap::new();

    for event in events {
        if event.event_id.trim().is_empty()
            || event.event_id.trim() != event.event_id
            || event.sequence == 0
            || event.occurred_at == 0
            || !valid_reference(&event.target)
        {
            return Err(LineageErrorV1::InvalidChangeEvent);
        }

        if let Some(previous) = unique_events.get(&event.event_id) {
            if previous == event {
                // Exact replay of an already admitted event is idempotent.
                continue;
            }
            return Err(LineageErrorV1::ConflictingEventIdentifier);
        }

        if sequence_owners.contains_key(&event.sequence) {
            return Err(LineageErrorV1::DuplicateChangeSequence);
        }

        let target_key = key(&event.target);
        let Some(target_node) = node_map.get(&target_key) else {
            return Err(LineageErrorV1::UnknownChangeTarget);
        };
        if target_node.reference.digest != event.target.digest {
            return Err(LineageErrorV1::ChangeTargetDigestMismatch);
        }

        let requires_replacement = matches!(
            event.reason,
            ChangeReasonV1::Corrected | ChangeReasonV1::Superseded
        );
        match (&event.replacement, requires_replacement) {
            (Some(replacement), true) => {
                if !valid_reference(replacement)
                    || replacement.artifact_id != event.target.artifact_id
                    || replacement.revision <= event.target.revision
                {
                    return Err(LineageErrorV1::InvalidReplacement);
                }
                let Some(replacement_node) = node_map.get(&key(replacement)) else {
                    return Err(LineageErrorV1::InvalidReplacement);
                };
                if replacement_node.reference.digest != replacement.digest {
                    return Err(LineageErrorV1::InvalidReplacement);
                }
            }
            (None, true) => return Err(LineageErrorV1::InvalidReplacement),
            (Some(_), false) => return Err(LineageErrorV1::InvalidReplacement),
            (None, false) => {}
        }

        if target_events.insert(target_key, event.clone()).is_some() {
            return Err(LineageErrorV1::MultipleChangesForArtifact);
        }
        sequence_owners.insert(event.sequence, event.event_id.clone());
        unique_events.insert(event.event_id.clone(), event.clone());
    }

    let mut states: BTreeMap<ArtifactKey, ArtifactStateV1> = BTreeMap::new();
    let mut impacts = Vec::new();

    for node_key in topological_order {
        let Some(node) = node_map.get(&node_key) else {
            return Err(LineageErrorV1::DuplicateArtifactRevision);
        };

        let state = if let Some(event) = target_events.get(&node_key) {
            state_for_change(event.reason)
        } else {
            let mut invalidated = false;
            let mut needs_revalidation = false;

            for dependency in &node.dependencies {
                if dependency.impact_policy == ImpactPolicyV1::Informational {
                    continue;
                }
                let dependency_key = key(&dependency.reference);
                let Some(upstream_state) = states.get(&dependency_key).copied() else {
                    return Err(LineageErrorV1::MissingDependency);
                };
                if upstream_state == ArtifactStateV1::Current {
                    continue;
                }

                impacts.push(ImpactRecordV1 {
                    affected_artifact: node.reference.clone(),
                    dependency: dependency.reference.clone(),
                    upstream_state,
                    policy: dependency.impact_policy,
                });

                match dependency.impact_policy {
                    ImpactPolicyV1::RevalidateOnChange => needs_revalidation = true,
                    ImpactPolicyV1::InvalidateOnChange => invalidated = true,
                    ImpactPolicyV1::Informational => {}
                }
            }

            if invalidated {
                ArtifactStateV1::Invalidated
            } else if needs_revalidation {
                ArtifactStateV1::NeedsRevalidation
            } else {
                ArtifactStateV1::Current
            }
        };

        states.insert(node_key, state);
    }

    impacts.sort_by(|left, right| {
        left.affected_artifact
            .artifact_id
            .cmp(&right.affected_artifact.artifact_id)
            .then_with(|| left.affected_artifact.revision.cmp(&right.affected_artifact.revision))
            .then_with(|| left.dependency.artifact_id.cmp(&right.dependency.artifact_id))
            .then_with(|| left.dependency.revision.cmp(&right.dependency.revision))
    });

    let artifacts = states
        .into_iter()
        .filter_map(|(artifact_key, state)| {
            node_map.get(&artifact_key).map(|node| ArtifactStateRecordV1 {
                reference: node.reference.clone(),
                state,
            })
        })
        .collect();

    let mut ordered_events = unique_events.into_values().collect::<Vec<_>>();
    ordered_events.sort_by_key(|event| event.sequence);
    let applied_event_ids = ordered_events
        .into_iter()
        .map(|event| event.event_id)
        .collect();

    Ok(LineageEvaluationV1 {
        artifacts,
        impacts,
        applied_event_ids,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reference(id: &str, revision: u64, digest_byte: u8) -> ArtifactRefV1 {
        ArtifactRefV1 {
            artifact_id: id.to_owned(),
            revision,
            digest: [digest_byte; 32],
        }
    }

    fn dependency(
        reference: ArtifactRefV1,
        relation: DependencyRelationV1,
        impact_policy: ImpactPolicyV1,
    ) -> ArtifactDependencyV1 {
        ArtifactDependencyV1 {
            reference,
            relation,
            impact_policy,
        }
    }

    fn node(
        reference: ArtifactRefV1,
        dependencies: Vec<ArtifactDependencyV1>,
    ) -> ArtifactNodeV1 {
        ArtifactNodeV1 {
            reference,
            kind: ArtifactKindV1::Other,
            dependencies,
        }
    }

    fn event(
        event_id: &str,
        sequence: u64,
        target: ArtifactRefV1,
        reason: ChangeReasonV1,
        replacement: Option<ArtifactRefV1>,
    ) -> ChangeEventV1 {
        ChangeEventV1 {
            event_id: event_id.to_owned(),
            sequence,
            target,
            reason,
            replacement,
            occurred_at: 1_800_000_000 + sequence,
        }
    }

    fn state(
        evaluation: &LineageEvaluationV1,
        reference: &ArtifactRefV1,
    ) -> ArtifactStateV1 {
        evaluation.state_for(reference).expect("artifact state exists")
    }

    #[test]
    fn revocation_propagates_through_claim_decision_and_project() {
        let source = reference("source", 1, 1);
        let claim = reference("claim", 1, 2);
        let decision = reference("decision", 1, 3);
        let project = reference("project", 1, 4);
        let nodes = vec![
            node(source.clone(), vec![]),
            node(
                claim.clone(),
                vec![dependency(
                    source.clone(),
                    DependencyRelationV1::SourcedFrom,
                    ImpactPolicyV1::InvalidateOnChange,
                )],
            ),
            node(
                decision.clone(),
                vec![dependency(
                    claim.clone(),
                    DependencyRelationV1::SemanticallySupportedBy,
                    ImpactPolicyV1::InvalidateOnChange,
                )],
            ),
            node(
                project.clone(),
                vec![dependency(
                    decision.clone(),
                    DependencyRelationV1::Implements,
                    ImpactPolicyV1::RevalidateOnChange,
                )],
            ),
        ];

        let evaluation = evaluate_lineage(
            &nodes,
            &[event(
                "revoke-source",
                1,
                source.clone(),
                ChangeReasonV1::Revoked,
                None,
            )],
        )
        .expect("valid lineage");

        assert_eq!(state(&evaluation, &source), ArtifactStateV1::Revoked);
        assert_eq!(state(&evaluation, &claim), ArtifactStateV1::Invalidated);
        assert_eq!(state(&evaluation, &decision), ArtifactStateV1::Invalidated);
        assert_eq!(
            state(&evaluation, &project),
            ArtifactStateV1::NeedsRevalidation
        );
        assert_eq!(evaluation.impacts.len(), 3);
    }

    #[test]
    fn corrected_source_requires_new_claim_revision() {
        let source_v1 = reference("source", 1, 1);
        let source_v2 = reference("source", 2, 5);
        let claim_v1 = reference("claim", 1, 2);
        let claim_v2 = reference("claim", 2, 6);
        let nodes = vec![
            node(source_v1.clone(), vec![]),
            node(source_v2.clone(), vec![]),
            node(
                claim_v1.clone(),
                vec![dependency(
                    source_v1.clone(),
                    DependencyRelationV1::SourcedFrom,
                    ImpactPolicyV1::RevalidateOnChange,
                )],
            ),
            node(
                claim_v2.clone(),
                vec![dependency(
                    source_v2.clone(),
                    DependencyRelationV1::SourcedFrom,
                    ImpactPolicyV1::RevalidateOnChange,
                )],
            ),
        ];
        let evaluation = evaluate_lineage(
            &nodes,
            &[event(
                "correct-source",
                1,
                source_v1.clone(),
                ChangeReasonV1::Corrected,
                Some(source_v2.clone()),
            )],
        )
        .expect("valid correction");

        assert_eq!(state(&evaluation, &source_v1), ArtifactStateV1::Superseded);
        assert_eq!(state(&evaluation, &source_v2), ArtifactStateV1::Current);
        assert_eq!(
            state(&evaluation, &claim_v1),
            ArtifactStateV1::NeedsRevalidation
        );
        assert_eq!(state(&evaluation, &claim_v2), ArtifactStateV1::Current);
    }

    #[test]
    fn informational_edges_and_independent_observations_are_not_invalidated() {
        let source = reference("source", 1, 1);
        let annotation = reference("annotation", 1, 2);
        let observation = reference("independent-observation", 1, 3);
        let nodes = vec![
            node(source.clone(), vec![]),
            node(
                annotation.clone(),
                vec![dependency(
                    source.clone(),
                    DependencyRelationV1::Other,
                    ImpactPolicyV1::Informational,
                )],
            ),
            node(observation.clone(), vec![]),
        ];

        let evaluation = evaluate_lineage(
            &nodes,
            &[event(
                "expire-source",
                1,
                source.clone(),
                ChangeReasonV1::Expired,
                None,
            )],
        )
        .expect("valid lineage");

        assert_eq!(state(&evaluation, &annotation), ArtifactStateV1::Current);
        assert_eq!(state(&evaluation, &observation), ArtifactStateV1::Current);
        assert!(evaluation.impacts.is_empty());
    }

    #[test]
    fn expired_evidence_marks_dependents_for_revalidation() {
        let evidence = reference("evidence", 1, 1);
        let assessment = reference("assessment", 1, 2);
        let nodes = vec![
            node(evidence.clone(), vec![]),
            node(
                assessment.clone(),
                vec![dependency(
                    evidence.clone(),
                    DependencyRelationV1::SemanticallySupportedBy,
                    ImpactPolicyV1::RevalidateOnChange,
                )],
            ),
        ];
        let evaluation = evaluate_lineage(
            &nodes,
            &[event(
                "expire-evidence",
                1,
                evidence,
                ChangeReasonV1::Expired,
                None,
            )],
        )
        .expect("valid lineage");
        assert_eq!(
            state(&evaluation, &assessment),
            ArtifactStateV1::NeedsRevalidation
        );
    }

    #[test]
    fn all_nine_two_edge_impact_policy_combinations_match_the_oracle() {
        let policies = [
            ImpactPolicyV1::RevalidateOnChange,
            ImpactPolicyV1::InvalidateOnChange,
            ImpactPolicyV1::Informational,
        ];
        let source = reference("source", 1, 1);
        let intermediate = reference("intermediate", 1, 2);
        let final_artifact = reference("final", 1, 3);

        for first_policy in policies {
            for second_policy in policies {
                let nodes = vec![
                    node(source.clone(), vec![]),
                    node(
                        intermediate.clone(),
                        vec![dependency(
                            source.clone(),
                            DependencyRelationV1::DerivedFrom,
                            first_policy,
                        )],
                    ),
                    node(
                        final_artifact.clone(),
                        vec![dependency(
                            intermediate.clone(),
                            DependencyRelationV1::DerivedFrom,
                            second_policy,
                        )],
                    ),
                ];
                let evaluation = evaluate_lineage(
                    &nodes,
                    &[event(
                        "expire-source",
                        1,
                        source.clone(),
                        ChangeReasonV1::Expired,
                        None,
                    )],
                )
                .expect("well-formed two-edge lineage");

                let expected_intermediate = match first_policy {
                    ImpactPolicyV1::RevalidateOnChange => {
                        ArtifactStateV1::NeedsRevalidation
                    }
                    ImpactPolicyV1::InvalidateOnChange => ArtifactStateV1::Invalidated,
                    ImpactPolicyV1::Informational => ArtifactStateV1::Current,
                };
                let expected_final = if second_policy == ImpactPolicyV1::Informational
                    || expected_intermediate == ArtifactStateV1::Current
                {
                    ArtifactStateV1::Current
                } else {
                    match second_policy {
                        ImpactPolicyV1::RevalidateOnChange => {
                            ArtifactStateV1::NeedsRevalidation
                        }
                        ImpactPolicyV1::InvalidateOnChange => ArtifactStateV1::Invalidated,
                        ImpactPolicyV1::Informational => ArtifactStateV1::Current,
                    }
                };

                assert_eq!(
                    state(&evaluation, &intermediate),
                    expected_intermediate,
                    "first policy {first_policy:?}"
                );
                assert_eq!(
                    state(&evaluation, &final_artifact),
                    expected_final,
                    "first policy {first_policy:?}, second policy {second_policy:?}"
                );
            }
        }
    }

    #[test]
    fn digest_mismatched_dependency_is_rejected() {
        let source = reference("source", 1, 1);
        let wrong_source = reference("source", 1, 9);
        let claim = reference("claim", 1, 2);
        let nodes = vec![
            node(source, vec![]),
            node(
                claim,
                vec![dependency(
                    wrong_source,
                    DependencyRelationV1::SourcedFrom,
                    ImpactPolicyV1::RevalidateOnChange,
                )],
            ),
        ];
        assert_eq!(
            evaluate_lineage(&nodes, &[]),
            Err(LineageErrorV1::DependencyDigestMismatch)
        );
    }

    #[test]
    fn missing_dependency_is_rejected() {
        let claim = reference("claim", 1, 2);
        let missing = reference("missing-source", 1, 1);
        let nodes = vec![node(
            claim,
            vec![dependency(
                missing,
                DependencyRelationV1::SourcedFrom,
                ImpactPolicyV1::RevalidateOnChange,
            )],
        )];
        assert_eq!(
            evaluate_lineage(&nodes, &[]),
            Err(LineageErrorV1::MissingDependency)
        );
    }

    #[test]
    fn duplicate_dependency_is_rejected() {
        let source = reference("source", 1, 1);
        let claim = reference("claim", 1, 2);
        let dependency_ref = dependency(
            source.clone(),
            DependencyRelationV1::SourcedFrom,
            ImpactPolicyV1::RevalidateOnChange,
        );
        let nodes = vec![
            node(source, vec![]),
            node(claim, vec![dependency_ref.clone(), dependency_ref]),
        ];
        assert_eq!(
            evaluate_lineage(&nodes, &[]),
            Err(LineageErrorV1::DuplicateDependency)
        );
    }

    #[test]
    fn exact_event_replay_is_idempotent() {
        let source = reference("source", 1, 1);
        let nodes = vec![node(source.clone(), vec![])];
        let change = event(
            "expire-source",
            1,
            source,
            ChangeReasonV1::Expired,
            None,
        );
        let evaluation = evaluate_lineage(&nodes, &[change.clone(), change])
            .expect("exact event replay is idempotent");
        assert_eq!(evaluation.applied_event_ids, vec!["expire-source"]);
    }

    #[test]
    fn event_identifier_reuse_with_different_payload_is_rejected() {
        let source = reference("source", 1, 1);
        let nodes = vec![node(source.clone(), vec![])];
        let first = event(
            "same-id",
            1,
            source.clone(),
            ChangeReasonV1::Expired,
            None,
        );
        let second = event(
            "same-id",
            1,
            source,
            ChangeReasonV1::Revoked,
            None,
        );
        assert_eq!(
            evaluate_lineage(&nodes, &[first, second]),
            Err(LineageErrorV1::ConflictingEventIdentifier)
        );
    }

    #[test]
    fn duplicate_event_sequence_is_rejected() {
        let source_a = reference("source-a", 1, 1);
        let source_b = reference("source-b", 1, 2);
        let nodes = vec![node(source_a.clone(), vec![]), node(source_b.clone(), vec![])];
        let first = event(
            "expire-a",
            1,
            source_a,
            ChangeReasonV1::Expired,
            None,
        );
        let second = event(
            "expire-b",
            1,
            source_b,
            ChangeReasonV1::Expired,
            None,
        );
        assert_eq!(
            evaluate_lineage(&nodes, &[first, second]),
            Err(LineageErrorV1::DuplicateChangeSequence)
        );
    }

    #[test]
    fn correction_requires_existing_higher_revision_of_same_artifact() {
        let source_v1 = reference("source", 1, 1);
        let wrong_replacement = reference("other-source", 2, 2);
        let nodes = vec![node(source_v1.clone(), vec![]), node(wrong_replacement.clone(), vec![])];
        let change = event(
            "correct-source",
            1,
            source_v1,
            ChangeReasonV1::Corrected,
            Some(wrong_replacement),
        );
        assert_eq!(
            evaluate_lineage(&nodes, &[change]),
            Err(LineageErrorV1::InvalidReplacement)
        );
    }

    #[test]
    fn change_target_digest_must_match_exact_artifact_version() {
        let source = reference("source", 1, 1);
        let mismatched_target = reference("source", 1, 8);
        let nodes = vec![node(source, vec![])];
        let change = event(
            "expire-source",
            1,
            mismatched_target,
            ChangeReasonV1::Expired,
            None,
        );
        assert_eq!(
            evaluate_lineage(&nodes, &[change]),
            Err(LineageErrorV1::ChangeTargetDigestMismatch)
        );
    }

    #[test]
    fn dependency_cycle_is_rejected() {
        let first = reference("first", 1, 1);
        let second = reference("second", 1, 2);
        let nodes = vec![
            node(
                first.clone(),
                vec![dependency(
                    second.clone(),
                    DependencyRelationV1::DerivedFrom,
                    ImpactPolicyV1::RevalidateOnChange,
                )],
            ),
            node(
                second.clone(),
                vec![dependency(
                    first.clone(),
                    DependencyRelationV1::DerivedFrom,
                    ImpactPolicyV1::RevalidateOnChange,
                )],
            ),
        ];
        assert_eq!(
            evaluate_lineage(&nodes, &[]),
            Err(LineageErrorV1::DependencyCycle)
        );
    }

    fn oracle_has_cycle(node_count: usize, edges: &[(usize, usize)]) -> bool {
        fn visit(node: usize, adjacency: &[Vec<usize>], marks: &mut [u8]) -> bool {
            if marks[node] == 1 {
                return true;
            }
            if marks[node] == 2 {
                return false;
            }
            marks[node] = 1;
            for next in &adjacency[node] {
                if visit(*next, adjacency, marks) {
                    return true;
                }
            }
            marks[node] = 2;
            false
        }

        let mut adjacency = vec![Vec::new(); node_count];
        for (from, to) in edges {
            adjacency[*from].push(*to);
        }
        let mut marks = vec![0; node_count];
        (0..node_count).any(|node| visit(node, &adjacency, &mut marks))
    }

    #[test]
    fn exhaustive_four_node_graphs_match_independent_cycle_oracle() {
        let references = [
            reference("a", 1, 1),
            reference("b", 1, 2),
            reference("c", 1, 3),
            reference("d", 1, 4),
        ];
        let possible_edges = (0..4)
            .flat_map(|from| (0..4).filter(move |to| *to != from).map(move |to| (from, to)))
            .collect::<Vec<_>>();

        for mask in 0_u16..(1_u16 << possible_edges.len()) {
            let selected = possible_edges
                .iter()
                .enumerate()
                .filter(|(index, _)| (mask & (1 << *index)) != 0)
                .map(|(_, edge)| *edge)
                .collect::<Vec<_>>();
            let mut nodes = references
                .iter()
                .cloned()
                .map(|reference| node(reference, vec![]))
                .collect::<Vec<_>>();
            for (from, to) in &selected {
                nodes[*to].dependencies.push(dependency(
                    references[*from].clone(),
                    DependencyRelationV1::DerivedFrom,
                    ImpactPolicyV1::RevalidateOnChange,
                ));
            }

            let expected_cycle = oracle_has_cycle(4, &selected);
            let actual = evaluate_lineage(&nodes, &[]);
            assert_eq!(
                actual.is_err(),
                expected_cycle,
                "graph mask {mask} disagreed with independent oracle"
            );
            if expected_cycle {
                assert_eq!(actual, Err(LineageErrorV1::DependencyCycle));
            }
        }
    }
}

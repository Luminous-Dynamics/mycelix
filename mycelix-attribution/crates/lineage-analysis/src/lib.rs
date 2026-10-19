// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Non-authoritative structural analysis for Mycelix contribution lineage.
//!
//! This crate consumes lineage/response payloads that callers should already have
//! obtained through qualified Attribution query paths. It re-runs pure field validation
//! defensively, but it does not establish DHT validity, identity authenticity, fraud,
//! collusion, reputation, causal truth, or entitlement.
//!
//! Cycles, reciprocal support, repeated claims, mixed responder positions, and actor
//! concentration are observations. None is automatically evidence of misconduct.

#![deny(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use lineage_integrity::{
    validate_attestation_fields, validate_response_fields, LineageAttestation, LineageResponse,
    ResponseDisposition,
};
use serde::{Deserialize, Serialize};

pub const MAX_ANALYSIS_RECORDS: usize = 4_096;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SubjectAuthorBinding {
    /// Exact Holochain action-hash string for the evidence subject.
    pub subject_action: String,
    pub author_did: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActorVolume {
    pub actor_did: String,
    pub count: usize,
    /// Descriptive share of the analyzed input, in basis points. Not reputation/risk.
    pub share_bps: u16,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CycleComponent {
    pub members: Vec<String>,
    pub internal_edge_count: usize,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReciprocalSubjectPair {
    pub left_subject: String,
    pub right_subject: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RepeatedLineageEdge {
    pub source_ref: String,
    pub target_ref: String,
    pub claim_count: usize,
    pub unique_attestors: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LineageStructureReport {
    pub claim_count: usize,
    pub unique_subject_count: usize,
    /// Strongly connected components of size >1. These show cycle-bearing regions;
    /// they do not classify the claims inside them as invalid or collusive.
    pub cycle_components: Vec<CycleComponent>,
    pub reciprocal_subject_pairs: Vec<ReciprocalSubjectPair>,
    pub repeated_edges: Vec<RepeatedLineageEdge>,
    pub attestor_volume: Vec<ActorVolume>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReciprocalActorPair {
    pub left_actor: String,
    pub right_actor: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResponderSubjectPattern {
    pub subject_action: String,
    pub responder_did: String,
    pub corroborate_count: usize,
    pub contest_count: usize,
    pub retract_count: usize,
    pub supersede_count: usize,
    /// True when one responder both corroborates and contests the same exact subject.
    pub mixed_independent_position: bool,
    /// True when the responder supplied more than one response of the same disposition.
    pub repeated_same_position: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResponseStructureReport {
    pub response_count: usize,
    pub unique_subject_count: usize,
    /// Subject actions for which no unambiguous author binding was provided.
    pub unmatched_subject_actions: Vec<String>,
    /// Defensive observation. Qualified CL-03 semantics should prevent this for
    /// corroboration, but the analyzer does not silently assume its input is perfect.
    pub self_corroboration_subject_actions: Vec<String>,
    /// Only groups with repeated or mixed response behavior are surfaced here.
    pub responder_subject_patterns: Vec<ResponderSubjectPattern>,
    /// Reciprocal corroboration between actors: A corroborates a subject authored by B
    /// and B corroborates a subject authored by A somewhere in the analyzed input.
    pub reciprocal_corroboration_pairs: Vec<ReciprocalActorPair>,
    /// SCCs of the actor corroboration graph. These are structural rings, not guilt.
    pub corroboration_cycle_components: Vec<CycleComponent>,
    pub responder_volume: Vec<ActorVolume>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AnalysisError {
    TooManyAttestations { count: usize, max: usize },
    TooManyResponses { count: usize, max: usize },
    TooManySubjectBindings { count: usize, max: usize },
    InvalidAttestation { index: usize, reason: String },
    InvalidResponse { index: usize, reason: String },
    InvalidSubjectBinding { index: usize, field: &'static str },
    ConflictingSubjectAuthors {
        subject_action: String,
        first_author: String,
        second_author: String,
    },
}

impl std::fmt::Display for AnalysisError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TooManyAttestations { count, max } => {
                write!(f, "too many lineage attestations: {count}; maximum is {max}")
            }
            Self::TooManyResponses { count, max } => {
                write!(f, "too many lineage responses: {count}; maximum is {max}")
            }
            Self::TooManySubjectBindings { count, max } => {
                write!(f, "too many subject-author bindings: {count}; maximum is {max}")
            }
            Self::InvalidAttestation { index, reason } => {
                write!(f, "invalid lineage attestation at index {index}: {reason}")
            }
            Self::InvalidResponse { index, reason } => {
                write!(f, "invalid lineage response at index {index}: {reason}")
            }
            Self::InvalidSubjectBinding { index, field } => {
                write!(f, "invalid subject-author binding at index {index}: {field}")
            }
            Self::ConflictingSubjectAuthors {
                subject_action,
                first_author,
                second_author,
            } => write!(
                f,
                "subject action {subject_action} has conflicting authors: {first_author} vs {second_author}"
            ),
        }
    }
}

impl std::error::Error for AnalysisError {}

#[derive(Default)]
struct EdgeAccumulator {
    claim_count: usize,
    attestors: BTreeSet<String>,
}

pub fn analyze_lineage(
    attestations: &[LineageAttestation],
) -> Result<LineageStructureReport, AnalysisError> {
    if attestations.len() > MAX_ANALYSIS_RECORDS {
        return Err(AnalysisError::TooManyAttestations {
            count: attestations.len(),
            max: MAX_ANALYSIS_RECORDS,
        });
    }

    let mut subjects = BTreeSet::new();
    let mut edges = BTreeSet::<(String, String)>::new();
    let mut edge_stats = BTreeMap::<(String, String), EdgeAccumulator>::new();
    let mut actor_counts = BTreeMap::<String, usize>::new();

    for (index, claim) in attestations.iter().enumerate() {
        validate_attestation_fields(claim).map_err(|reason| AnalysisError::InvalidAttestation {
            index,
            reason,
        })?;

        subjects.insert(claim.source_ref.clone());
        subjects.insert(claim.target_ref.clone());
        let edge = (claim.source_ref.clone(), claim.target_ref.clone());
        edges.insert(edge.clone());
        let stats = edge_stats.entry(edge).or_default();
        stats.claim_count += 1;
        stats.attestors.insert(claim.attestor_did.clone());
        *actor_counts.entry(claim.attestor_did.clone()).or_default() += 1;
    }

    let cycle_components = strongly_connected_components(&subjects, &edges);

    let mut reciprocal_subject_pairs = BTreeSet::new();
    for (source, target) in &edges {
        if edges.contains(&(target.clone(), source.clone())) {
            reciprocal_subject_pairs.insert(ordered_pair(source, target));
        }
    }

    let repeated_edges = edge_stats
        .into_iter()
        .filter_map(|((source_ref, target_ref), stats)| {
            (stats.claim_count > 1).then(|| RepeatedLineageEdge {
                source_ref,
                target_ref,
                claim_count: stats.claim_count,
                unique_attestors: stats.attestors.into_iter().collect(),
            })
        })
        .collect();

    Ok(LineageStructureReport {
        claim_count: attestations.len(),
        unique_subject_count: subjects.len(),
        cycle_components,
        reciprocal_subject_pairs: reciprocal_subject_pairs
            .into_iter()
            .map(|(left_subject, right_subject)| ReciprocalSubjectPair {
                left_subject,
                right_subject,
            })
            .collect(),
        repeated_edges,
        attestor_volume: actor_volume(actor_counts, attestations.len()),
    })
}

pub fn analyze_responses(
    responses: &[LineageResponse],
    subject_authors: &[SubjectAuthorBinding],
) -> Result<ResponseStructureReport, AnalysisError> {
    if responses.len() > MAX_ANALYSIS_RECORDS {
        return Err(AnalysisError::TooManyResponses {
            count: responses.len(),
            max: MAX_ANALYSIS_RECORDS,
        });
    }
    if subject_authors.len() > MAX_ANALYSIS_RECORDS {
        return Err(AnalysisError::TooManySubjectBindings {
            count: subject_authors.len(),
            max: MAX_ANALYSIS_RECORDS,
        });
    }

    let author_by_subject = validate_subject_bindings(subject_authors)?;
    let mut responder_counts = BTreeMap::<String, usize>::new();
    let mut unique_subjects = BTreeSet::new();
    let mut unmatched = BTreeSet::new();
    let mut self_corroboration = BTreeSet::new();
    let mut grouped = BTreeMap::<(String, String), [usize; 4]>::new();
    let mut corroboration_edges = BTreeSet::<(String, String)>::new();
    let mut corroboration_actors = BTreeSet::new();

    for (index, response) in responses.iter().enumerate() {
        validate_response_fields(response).map_err(|reason| AnalysisError::InvalidResponse {
            index,
            reason,
        })?;

        let subject = response.subject_action.to_string();
        unique_subjects.insert(subject.clone());
        *responder_counts
            .entry(response.responder_did.clone())
            .or_default() += 1;

        let counts = grouped
            .entry((subject.clone(), response.responder_did.clone()))
            .or_insert([0; 4]);
        counts[disposition_index(&response.disposition)] += 1;

        if let Some(author) = author_by_subject.get(&subject) {
            if matches!(&response.disposition, ResponseDisposition::Corroborate) {
                if author == &response.responder_did {
                    self_corroboration.insert(subject.clone());
                }
                corroboration_actors.insert(response.responder_did.clone());
                corroboration_actors.insert(author.clone());
                corroboration_edges.insert((response.responder_did.clone(), author.clone()));
            }
        } else {
            unmatched.insert(subject);
        }
    }

    let responder_subject_patterns = grouped
        .into_iter()
        .filter_map(|((subject_action, responder_did), counts)| {
            let total = counts.iter().sum::<usize>();
            if total <= 1 {
                return None;
            }
            Some(ResponderSubjectPattern {
                subject_action,
                responder_did,
                corroborate_count: counts[0],
                contest_count: counts[1],
                retract_count: counts[2],
                supersede_count: counts[3],
                mixed_independent_position: counts[0] > 0 && counts[1] > 0,
                repeated_same_position: counts.iter().any(|count| *count > 1),
            })
        })
        .collect();

    let mut reciprocal_pairs = BTreeSet::new();
    for (from, to) in &corroboration_edges {
        if from != to && corroboration_edges.contains(&(to.clone(), from.clone())) {
            reciprocal_pairs.insert(ordered_pair(from, to));
        }
    }

    Ok(ResponseStructureReport {
        response_count: responses.len(),
        unique_subject_count: unique_subjects.len(),
        unmatched_subject_actions: unmatched.into_iter().collect(),
        self_corroboration_subject_actions: self_corroboration.into_iter().collect(),
        responder_subject_patterns,
        reciprocal_corroboration_pairs: reciprocal_pairs
            .into_iter()
            .map(|(left_actor, right_actor)| ReciprocalActorPair {
                left_actor,
                right_actor,
            })
            .collect(),
        corroboration_cycle_components: strongly_connected_components(
            &corroboration_actors,
            &corroboration_edges,
        ),
        responder_volume: actor_volume(responder_counts, responses.len()),
    })
}

fn validate_subject_bindings(
    bindings: &[SubjectAuthorBinding],
) -> Result<HashMap<String, String>, AnalysisError> {
    let mut result = HashMap::new();
    for (index, binding) in bindings.iter().enumerate() {
        if binding.subject_action.trim().is_empty() {
            return Err(AnalysisError::InvalidSubjectBinding {
                index,
                field: "subject_action",
            });
        }
        if binding.author_did.trim().is_empty() {
            return Err(AnalysisError::InvalidSubjectBinding {
                index,
                field: "author_did",
            });
        }
        if let Some(existing) = result.get(&binding.subject_action) {
            if existing != &binding.author_did {
                return Err(AnalysisError::ConflictingSubjectAuthors {
                    subject_action: binding.subject_action.clone(),
                    first_author: existing.clone(),
                    second_author: binding.author_did.clone(),
                });
            }
        } else {
            result.insert(binding.subject_action.clone(), binding.author_did.clone());
        }
    }
    Ok(result)
}

fn disposition_index(disposition: &ResponseDisposition) -> usize {
    match disposition {
        ResponseDisposition::Corroborate => 0,
        ResponseDisposition::Contest => 1,
        ResponseDisposition::Retract => 2,
        ResponseDisposition::Supersede => 3,
    }
}

fn ordered_pair(left: &str, right: &str) -> (String, String) {
    if left <= right {
        (left.to_string(), right.to_string())
    } else {
        (right.to_string(), left.to_string())
    }
}

fn actor_volume(counts: BTreeMap<String, usize>, total: usize) -> Vec<ActorVolume> {
    let mut result = counts
        .into_iter()
        .map(|(actor_did, count)| ActorVolume {
            actor_did,
            count,
            share_bps: if total == 0 {
                0
            } else {
                ((count * 10_000) / total).min(10_000) as u16
            },
        })
        .collect::<Vec<_>>();
    result.sort_by(|a, b| b.count.cmp(&a.count).then_with(|| a.actor_did.cmp(&b.actor_did)));
    result
}

/// Deterministic iterative Kosaraju SCC pass. Only components with >1 member are
/// returned; single-node self-correlations are reported separately where relevant.
fn strongly_connected_components(
    nodes: &BTreeSet<String>,
    edges: &BTreeSet<(String, String)>,
) -> Vec<CycleComponent> {
    let mut forward = BTreeMap::<String, Vec<String>>::new();
    let mut reverse = BTreeMap::<String, Vec<String>>::new();
    for node in nodes {
        forward.entry(node.clone()).or_default();
        reverse.entry(node.clone()).or_default();
    }
    for (from, to) in edges {
        forward.entry(from.clone()).or_default().push(to.clone());
        reverse.entry(to.clone()).or_default().push(from.clone());
    }
    for neighbors in forward.values_mut() {
        neighbors.sort();
        neighbors.dedup();
    }
    for neighbors in reverse.values_mut() {
        neighbors.sort();
        neighbors.dedup();
    }

    let mut visited = HashSet::new();
    let mut finish_order = Vec::with_capacity(nodes.len());
    for start in nodes {
        if visited.contains(start) {
            continue;
        }
        let mut stack = vec![(start.clone(), false)];
        while let Some((node, expanded)) = stack.pop() {
            if expanded {
                finish_order.push(node);
                continue;
            }
            if !visited.insert(node.clone()) {
                continue;
            }
            stack.push((node.clone(), true));
            if let Some(neighbors) = forward.get(&node) {
                for neighbor in neighbors.iter().rev() {
                    if !visited.contains(neighbor) {
                        stack.push((neighbor.clone(), false));
                    }
                }
            }
        }
    }

    let mut assigned = HashSet::new();
    let mut components = Vec::new();
    for start in finish_order.into_iter().rev() {
        if !assigned.insert(start.clone()) {
            continue;
        }
        let mut members = Vec::new();
        let mut stack = vec![start];
        while let Some(node) = stack.pop() {
            members.push(node.clone());
            if let Some(neighbors) = reverse.get(&node) {
                for neighbor in neighbors.iter().rev() {
                    if assigned.insert(neighbor.clone()) {
                        stack.push(neighbor.clone());
                    }
                }
            }
        }
        if members.len() <= 1 {
            continue;
        }
        members.sort();
        let member_set = members.iter().cloned().collect::<HashSet<_>>();
        let internal_edge_count = edges
            .iter()
            .filter(|(from, to)| member_set.contains(from) && member_set.contains(to))
            .count();
        components.push(CycleComponent {
            members,
            internal_edge_count,
        });
    }
    components.sort_by(|a, b| a.members.cmp(&b.members));
    components
}

#[cfg(test)]
mod tests {
    use hdi::prelude::ActionHash;
    use lineage_integrity::{EvidenceRef, LineageRelation, LINEAGE_SCHEMA_VERSION};

    use super::*;

    fn claim(attestor: &str, id: &str, source: &str, target: &str) -> LineageAttestation {
        LineageAttestation {
            schema_version: LINEAGE_SCHEMA_VERSION,
            id: id.into(),
            attestor_did: attestor.into(),
            source_ref: source.into(),
            target_ref: target.into(),
            relation: LineageRelation::EnabledBy,
            confidence_bps: 7_000,
            evidence_refs: vec![],
            rationale: "structural test claim".into(),
        }
    }

    fn action(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    fn response(
        responder: &str,
        id: &str,
        subject: ActionHash,
        disposition: ResponseDisposition,
    ) -> LineageResponse {
        let independent = matches!(
            &disposition,
            ResponseDisposition::Corroborate | ResponseDisposition::Contest
        );
        LineageResponse {
            schema_version: LINEAGE_SCHEMA_VERSION,
            id: id.into(),
            responder_did: responder.into(),
            subject_action: subject,
            disposition,
            confidence_bps: independent.then_some(7_000),
            evidence_refs: Vec::<EvidenceRef>::new(),
            rationale: "structural test response".into(),
            replacement_action: None,
        }
    }

    #[test]
    fn dag_has_no_cycle_component() {
        let report = analyze_lineage(&[
            claim("did:a", "1", "A", "B"),
            claim("did:b", "2", "B", "C"),
        ])
        .unwrap();
        assert!(report.cycle_components.is_empty());
        assert!(report.reciprocal_subject_pairs.is_empty());
    }

    #[test]
    fn reciprocal_edge_is_visible_as_pair_and_scc() {
        let report = analyze_lineage(&[
            claim("did:a", "1", "A", "B"),
            claim("did:b", "2", "B", "A"),
        ])
        .unwrap();
        assert_eq!(report.reciprocal_subject_pairs.len(), 1);
        assert_eq!(report.cycle_components.len(), 1);
        assert_eq!(report.cycle_components[0].members, vec!["A", "B"]);
    }

    #[test]
    fn repeated_claims_preserve_total_and_unique_attestors() {
        let report = analyze_lineage(&[
            claim("did:a", "1", "A", "B"),
            claim("did:a", "2", "A", "B"),
            claim("did:b", "3", "A", "B"),
        ])
        .unwrap();
        assert_eq!(report.repeated_edges.len(), 1);
        assert_eq!(report.repeated_edges[0].claim_count, 3);
        assert_eq!(
            report.repeated_edges[0].unique_attestors,
            vec!["did:a", "did:b"]
        );
    }

    #[test]
    fn reciprocal_actor_corroboration_is_structural_only() {
        let subject_a = action(1);
        let subject_b = action(2);
        let bindings = vec![
            SubjectAuthorBinding {
                subject_action: subject_a.to_string(),
                author_did: "did:a".into(),
            },
            SubjectAuthorBinding {
                subject_action: subject_b.to_string(),
                author_did: "did:b".into(),
            },
        ];
        let report = analyze_responses(
            &[
                response("did:a", "ra", subject_b, ResponseDisposition::Corroborate),
                response("did:b", "rb", subject_a, ResponseDisposition::Corroborate),
            ],
            &bindings,
        )
        .unwrap();
        assert_eq!(report.reciprocal_corroboration_pairs.len(), 1);
        assert_eq!(report.corroboration_cycle_components.len(), 1);
    }

    #[test]
    fn three_actor_corroboration_ring_is_surfaced() {
        let a = action(1);
        let b = action(2);
        let c = action(3);
        let bindings = vec![
            SubjectAuthorBinding {
                subject_action: a.to_string(),
                author_did: "did:a".into(),
            },
            SubjectAuthorBinding {
                subject_action: b.to_string(),
                author_did: "did:b".into(),
            },
            SubjectAuthorBinding {
                subject_action: c.to_string(),
                author_did: "did:c".into(),
            },
        ];
        let report = analyze_responses(
            &[
                response("did:a", "1", b, ResponseDisposition::Corroborate),
                response("did:b", "2", c, ResponseDisposition::Corroborate),
                response("did:c", "3", a, ResponseDisposition::Corroborate),
            ],
            &bindings,
        )
        .unwrap();
        assert_eq!(report.corroboration_cycle_components.len(), 1);
        assert_eq!(
            report.corroboration_cycle_components[0].members,
            vec!["did:a", "did:b", "did:c"]
        );
    }

    #[test]
    fn mixed_responder_position_is_surfaced_without_resolution() {
        let subject = action(4);
        let responses = vec![
            response("did:x", "1", subject.clone(), ResponseDisposition::Corroborate),
            response("did:x", "2", subject.clone(), ResponseDisposition::Contest),
        ];
        let bindings = vec![SubjectAuthorBinding {
            subject_action: subject.to_string(),
            author_did: "did:author".into(),
        }];
        let report = analyze_responses(&responses, &bindings).unwrap();
        assert_eq!(report.responder_subject_patterns.len(), 1);
        assert!(report.responder_subject_patterns[0].mixed_independent_position);
    }

    #[test]
    fn unmatched_subject_is_counted_not_guessed() {
        let subject = action(5);
        let report = analyze_responses(
            &[response(
                "did:x",
                "1",
                subject.clone(),
                ResponseDisposition::Corroborate,
            )],
            &[],
        )
        .unwrap();
        assert_eq!(report.unmatched_subject_actions, vec![subject.to_string()]);
        assert!(report.corroboration_cycle_components.is_empty());
    }

    #[test]
    fn defensive_self_corroboration_is_observable_not_scored() {
        let subject = action(7);
        let report = analyze_responses(
            &[response(
                "did:a",
                "1",
                subject.clone(),
                ResponseDisposition::Corroborate,
            )],
            &[SubjectAuthorBinding {
                subject_action: subject.to_string(),
                author_did: "did:a".into(),
            }],
        )
        .unwrap();
        assert_eq!(
            report.self_corroboration_subject_actions,
            vec![subject.to_string()]
        );
    }

    #[test]
    fn conflicting_subject_author_bindings_fail_closed() {
        let subject = action(6).to_string();
        let error = analyze_responses(
            &[],
            &[
                SubjectAuthorBinding {
                    subject_action: subject.clone(),
                    author_did: "did:a".into(),
                },
                SubjectAuthorBinding {
                    subject_action: subject.clone(),
                    author_did: "did:b".into(),
                },
            ],
        )
        .unwrap_err();
        assert!(matches!(error, AnalysisError::ConflictingSubjectAuthors { .. }));
    }

    #[test]
    fn actor_volume_is_deterministic_and_descriptive() {
        let report = analyze_lineage(&[
            claim("did:b", "1", "A", "B"),
            claim("did:a", "2", "B", "C"),
            claim("did:a", "3", "C", "D"),
        ])
        .unwrap();
        assert_eq!(report.attestor_volume[0].actor_did, "did:a");
        assert_eq!(report.attestor_volume[0].count, 2);
        assert_eq!(report.attestor_volume[0].share_bps, 6_666);
    }
}

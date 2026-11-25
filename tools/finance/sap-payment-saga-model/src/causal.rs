//! Partial-order causal projection for owner-scoped SAP journal events.
//!
//! This module deliberately has no wall-clock field and never derives a global
//! order from event arrival, timestamps, or owner-name sorting. It combines
//! only (a) explicit owner-local predecessor links and (b) explicit causal
//! references. Events in the same topological layer are incomparable in the
//! supplied graph; this does not prove that they occurred simultaneously.
//!
//! The verifier requires a complete owner prefix beginning at sequence zero.
//! A future snapshot profile can add independently authenticated base
//! frontiers; this v1 module does not silently accept truncated histories.

use std::collections::{BTreeMap, BTreeSet};

use sha2::{Digest, Sha256};

pub const CAUSAL_GRAPH_SCHEMA_V1: u16 = 1;
const CAUSAL_GRAPH_DOMAIN_V1: &[u8] = b"mycelix:sap-causal-graph:v1\0";
const MAX_EVENTS_V1: usize = 100_000;
const MAX_REFERENCES_PER_EVENT_V1: usize = 64;
const MAX_COMPONENT_BYTES_V1: usize = 1_024;

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct EventRefV1 {
    pub owner_id: String,
    pub event_id: String,
}

impl EventRefV1 {
    pub fn new(owner_id: impl Into<String>, event_id: impl Into<String>) -> Self {
        Self {
            owner_id: owner_id.into(),
            event_id: event_id.into(),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CausalEventV1 {
    /// Stable owner namespace. This is not inferred from an event ID.
    pub owner_id: String,
    /// Owner-native event identifier, interpreted only within owner_id.
    pub event_id: String,
    /// Zero-based, owner-local sequence. No cross-owner sequence is implied.
    pub owner_sequence: u64,
    /// Exact previous event on this owner's chain, absent only at sequence 0.
    pub owner_predecessor: Option<EventRefV1>,
    /// Explicit cross-event happens-before references. These do not replace
    /// owner_predecessor and do not imply a global total order.
    pub caused_by: Vec<EventRefV1>,
}

impl CausalEventV1 {
    pub fn reference(&self) -> EventRefV1 {
        EventRefV1::new(self.owner_id.clone(), self.event_id.clone())
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerFrontierV1 {
    pub owner_id: String,
    pub sequence: u64,
    pub event_id: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CausalProjectionV1 {
    /// Domain-separated SHA-256 over the canonical, input-order-independent
    /// graph subject. This identifies the supplied graph; it is not a
    /// signature, authority grant, or authenticity proof.
    pub graph_digest: String,
    pub event_count: usize,
    /// Count of unique edges after owner-predecessor and caused_by edges merge.
    pub edge_count: usize,
    pub owner_frontiers: Vec<OwnerFrontierV1>,
    /// Deterministic Kahn layers. Nodes in one layer are mutually incomparable
    /// under this supplied graph, not necessarily concurrent in real time.
    pub topological_layers: Vec<Vec<EventRefV1>>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CausalErrorV1 {
    EmptyGraph,
    TooManyEvents,
    TooManyCausalReferences,
    InvalidComponent,
    DuplicateEventReference,
    DuplicateOwnerSequence,
    OwnerSequenceGap,
    UnexpectedGenesisPredecessor,
    MissingOwnerPredecessor,
    WrongOwnerPredecessor,
    DuplicateCausalReference,
    SelfCausality,
    MissingCausalReference,
    CausalCycle,
    CanonicalLengthOverflow,
}

fn validate_component_v1(value: &str) -> Result<(), CausalErrorV1> {
    if value.is_empty() || value.len() > MAX_COMPONENT_BYTES_V1 {
        return Err(CausalErrorV1::InvalidComponent);
    }
    Ok(())
}

fn append_u32_v1(output: &mut Vec<u8>, value: usize) -> Result<(), CausalErrorV1> {
    let value = u32::try_from(value).map_err(|_| CausalErrorV1::CanonicalLengthOverflow)?;
    output.extend_from_slice(&value.to_be_bytes());
    Ok(())
}

fn append_text_v1(output: &mut Vec<u8>, value: &str) -> Result<(), CausalErrorV1> {
    validate_component_v1(value)?;
    append_u32_v1(output, value.len())?;
    output.extend_from_slice(value.as_bytes());
    Ok(())
}

fn append_reference_v1(
    output: &mut Vec<u8>,
    reference: &EventRefV1,
) -> Result<(), CausalErrorV1> {
    append_text_v1(output, &reference.owner_id)?;
    append_text_v1(output, &reference.event_id)
}

fn canonical_graph_bytes_v1(
    events: &[CausalEventV1],
) -> Result<Vec<u8>, CausalErrorV1> {
    let mut ordered: Vec<&CausalEventV1> = events.iter().collect();
    ordered.sort_by(|left, right| {
        (&left.owner_id, left.owner_sequence, &left.event_id).cmp(&(
            &right.owner_id,
            right.owner_sequence,
            &right.event_id,
        ))
    });

    let mut output = Vec::new();
    output.extend_from_slice(CAUSAL_GRAPH_DOMAIN_V1);
    output.extend_from_slice(&CAUSAL_GRAPH_SCHEMA_V1.to_be_bytes());
    append_u32_v1(&mut output, ordered.len())?;

    for event in ordered {
        append_text_v1(&mut output, &event.owner_id)?;
        append_text_v1(&mut output, &event.event_id)?;
        output.extend_from_slice(&event.owner_sequence.to_be_bytes());
        match &event.owner_predecessor {
            None => output.push(0),
            Some(reference) => {
                output.push(1);
                append_reference_v1(&mut output, reference)?;
            }
        }

        let mut causes = event.caused_by.clone();
        causes.sort();
        append_u32_v1(&mut output, causes.len())?;
        for reference in &causes {
            append_reference_v1(&mut output, reference)?;
        }
    }
    Ok(output)
}

/// Validate and project a complete set of owner-local event prefixes.
///
/// Only explicit predecessor/cause links create ordering constraints. Missing
/// references, gaps and cycles are errors; no timestamp or lexical order is
/// used to repair an incomplete graph.
pub fn project_causal_graph_v1(
    events: &[CausalEventV1],
) -> Result<CausalProjectionV1, CausalErrorV1> {
    if events.is_empty() {
        return Err(CausalErrorV1::EmptyGraph);
    }
    if events.len() > MAX_EVENTS_V1 {
        return Err(CausalErrorV1::TooManyEvents);
    }

    let mut by_reference = BTreeMap::<EventRefV1, usize>::new();
    let mut by_owner = BTreeMap::<String, Vec<(u64, usize)>>::new();

    for (index, event) in events.iter().enumerate() {
        validate_component_v1(&event.owner_id)?;
        validate_component_v1(&event.event_id)?;
        if by_reference.insert(event.reference(), index).is_some() {
            return Err(CausalErrorV1::DuplicateEventReference);
        }
        by_owner
            .entry(event.owner_id.clone())
            .or_default()
            .push((event.owner_sequence, index));
        if event.caused_by.len() > MAX_REFERENCES_PER_EVENT_V1 {
            return Err(CausalErrorV1::TooManyCausalReferences);
        }
    }

    let mut edges = BTreeSet::<(usize, usize)>::new();
    let mut owner_frontiers = Vec::with_capacity(by_owner.len());

    for (owner_id, owner_events) in &mut by_owner {
        owner_events.sort_by_key(|(sequence, _)| *sequence);
        let mut previous: Option<(u64, EventRefV1, usize)> = None;

        for (expected_sequence, (sequence, index)) in owner_events.iter().enumerate() {
            if *sequence != expected_sequence as u64 {
                if previous
                    .as_ref()
                    .is_some_and(|(prior, _, _)| *prior == *sequence)
                {
                    return Err(CausalErrorV1::DuplicateOwnerSequence);
                }
                return Err(CausalErrorV1::OwnerSequenceGap);
            }

            let event = &events[*index];
            let current_reference = event.reference();
            match (sequence, &event.owner_predecessor, &previous) {
                (0, None, None) => {}
                (0, Some(_), None) => {
                    return Err(CausalErrorV1::UnexpectedGenesisPredecessor);
                }
                (0, _, Some(_)) => return Err(CausalErrorV1::DuplicateOwnerSequence),
                (_, None, _) => return Err(CausalErrorV1::MissingOwnerPredecessor),
                (_, Some(_), None) => return Err(CausalErrorV1::OwnerSequenceGap),
                (_, Some(reference), Some((prior_sequence, prior_reference, prior_index))) => {
                    if *prior_sequence + 1 != *sequence || reference != prior_reference {
                        return Err(CausalErrorV1::WrongOwnerPredecessor);
                    }
                    edges.insert((*prior_index, *index));
                }
            }
            previous = Some((*sequence, current_reference, *index));
        }

        if let Some((sequence, reference, _)) = previous {
            owner_frontiers.push(OwnerFrontierV1 {
                owner_id: owner_id.clone(),
                sequence,
                event_id: reference.event_id,
            });
        }
    }

    for (index, event) in events.iter().enumerate() {
        let current = event.reference();
        let mut seen = BTreeSet::new();
        for parent in &event.caused_by {
            if !seen.insert(parent) {
                return Err(CausalErrorV1::DuplicateCausalReference);
            }
            if *parent == current {
                return Err(CausalErrorV1::SelfCausality);
            }
            let Some(parent_index) = by_reference.get(parent).copied() else {
                return Err(CausalErrorV1::MissingCausalReference);
            };
            edges.insert((parent_index, index));
        }
    }

    let mut adjacency = vec![Vec::<usize>::new(); events.len()];
    let mut indegree = vec![0usize; events.len()];
    for (source, target) in &edges {
        adjacency[*source].push(*target);
        indegree[*target] += 1;
    }

    let references: Vec<EventRefV1> = events.iter().map(CausalEventV1::reference).collect();
    let mut ready: Vec<usize> = indegree
        .iter()
        .enumerate()
        .filter_map(|(index, degree)| (*degree == 0).then_some(index))
        .collect();
    let mut topological_layers = Vec::new();
    let mut processed = 0usize;

    while !ready.is_empty() {
        ready.sort_by(|left, right| references[*left].cmp(&references[*right]));
        let layer = std::mem::take(&mut ready);
        let mut projected_layer = Vec::with_capacity(layer.len());

        for source in layer {
            processed += 1;
            projected_layer.push(references[source].clone());
            for target in &adjacency[source] {
                indegree[*target] -= 1;
                if indegree[*target] == 0 {
                    ready.push(*target);
                }
            }
        }
        topological_layers.push(projected_layer);
    }

    if processed != events.len() {
        return Err(CausalErrorV1::CausalCycle);
    }

    let canonical = canonical_graph_bytes_v1(events)?;
    let digest = Sha256::digest(canonical);
    let mut graph_digest = String::with_capacity(71);
    graph_digest.push_str("sha256:");
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut graph_digest, "{byte:02x}");
    }

    Ok(CausalProjectionV1 {
        graph_digest,
        event_count: events.len(),
        edge_count: edges.len(),
        owner_frontiers,
        topological_layers,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reference(owner: &str, event: &str) -> EventRefV1 {
        EventRefV1::new(owner, event)
    }

    fn event(
        owner: &str,
        event_id: &str,
        sequence: u64,
        predecessor: Option<EventRefV1>,
        caused_by: Vec<EventRefV1>,
    ) -> CausalEventV1 {
        CausalEventV1 {
            owner_id: owner.into(),
            event_id: event_id.into(),
            owner_sequence: sequence,
            owner_predecessor: predecessor,
            caused_by,
        }
    }

    fn sample() -> Vec<CausalEventV1> {
        let p0 = reference("payments", "p0");
        let t0 = reference("treasury", "t0");
        vec![
            event("payments", "p0", 0, None, vec![]),
            event("treasury", "t0", 0, None, vec![]),
            event("payments", "p1", 1, Some(p0.clone()), vec![]),
            event("hearth", "h0", 0, None, vec![p0, t0]),
        ]
    }

    #[test]
    fn independent_owner_effects_remain_incomparable() {
        let projection = project_causal_graph_v1(&sample()).unwrap();
        assert_eq!(projection.event_count, 4);
        assert_eq!(projection.edge_count, 3);
        assert_eq!(projection.topological_layers.len(), 2);
        assert_eq!(
            projection.topological_layers[0],
            vec![reference("payments", "p0"), reference("treasury", "t0")]
        );
        assert_eq!(
            projection.topological_layers[1],
            vec![reference("hearth", "h0"), reference("payments", "p1")]
        );
    }

    #[test]
    fn owner_frontiers_are_independent_and_deterministic() {
        let projection = project_causal_graph_v1(&sample()).unwrap();
        assert_eq!(
            projection.owner_frontiers,
            vec![
                OwnerFrontierV1 {
                    owner_id: "hearth".into(),
                    sequence: 0,
                    event_id: "h0".into(),
                },
                OwnerFrontierV1 {
                    owner_id: "payments".into(),
                    sequence: 1,
                    event_id: "p1".into(),
                },
                OwnerFrontierV1 {
                    owner_id: "treasury".into(),
                    sequence: 0,
                    event_id: "t0".into(),
                },
            ]
        );
    }

    #[test]
    fn canonical_digest_and_layers_ignore_input_order() {
        let events = sample();
        let first = project_causal_graph_v1(&events).unwrap();
        let mut reversed = events;
        reversed.reverse();
        let second = project_causal_graph_v1(&reversed).unwrap();
        assert_eq!(first.graph_digest, second.graph_digest);
        assert_eq!(first.topological_layers, second.topological_layers);
        assert_eq!(first.owner_frontiers, second.owner_frontiers);
    }

    #[test]
    fn canonical_digest_matches_shared_known_answer() {
        let expected = include_str!("../fixtures/sap-payment-causal-graph-v1.kat").trim();
        assert_eq!(
            project_causal_graph_v1(&sample()).unwrap().graph_digest,
            expected
        );
    }

    #[test]
    fn missing_owner_sequence_is_rejected() {
        let events = vec![
            event("payments", "p0", 0, None, vec![]),
            event("payments", "p2", 2, Some(reference("payments", "p1")), vec![]),
        ];
        assert_eq!(
            project_causal_graph_v1(&events),
            Err(CausalErrorV1::OwnerSequenceGap)
        );
    }

    #[test]
    fn owner_predecessor_must_be_exact_and_owner_local() {
        let events = vec![
            event("payments", "p0", 0, None, vec![]),
            event("payments", "p1", 1, Some(reference("treasury", "t0")), vec![]),
        ];
        assert_eq!(
            project_causal_graph_v1(&events),
            Err(CausalErrorV1::WrongOwnerPredecessor)
        );
    }

    #[test]
    fn missing_causal_reference_is_not_repaired_by_arrival_order() {
        let events = vec![event(
            "treasury",
            "t0",
            0,
            None,
            vec![reference("payments", "missing")],
        )];
        assert_eq!(
            project_causal_graph_v1(&events),
            Err(CausalErrorV1::MissingCausalReference)
        );
    }

    #[test]
    fn cross_owner_causal_cycle_is_rejected() {
        let p0 = reference("payments", "p0");
        let t0 = reference("treasury", "t0");
        let events = vec![
            event("payments", "p0", 0, None, vec![t0.clone()]),
            event("treasury", "t0", 0, None, vec![p0]),
        ];
        assert_eq!(
            project_causal_graph_v1(&events),
            Err(CausalErrorV1::CausalCycle)
        );
    }

    #[test]
    fn duplicate_event_reference_is_rejected() {
        let events = vec![
            event("payments", "p0", 0, None, vec![]),
            event("payments", "p0", 1, Some(reference("payments", "p0")), vec![]),
        ];
        assert_eq!(
            project_causal_graph_v1(&events),
            Err(CausalErrorV1::DuplicateEventReference)
        );
    }

    #[test]
    fn duplicate_causal_reference_is_rejected() {
        let p0 = reference("payments", "p0");
        let events = vec![
            event("payments", "p0", 0, None, vec![]),
            event("treasury", "t0", 0, None, vec![p0.clone(), p0]),
        ];
        assert_eq!(
            project_causal_graph_v1(&events),
            Err(CausalErrorV1::DuplicateCausalReference)
        );
    }

    #[test]
    fn empty_graph_is_rejected() {
        assert_eq!(
            project_causal_graph_v1(&[]),
            Err(CausalErrorV1::EmptyGraph)
        );
    }
}

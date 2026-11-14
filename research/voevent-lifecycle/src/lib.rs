#![forbid(unsafe_code)]

//! A read-only, receipt-time-bounded replay of VOEvent lifecycle citations.
//!
//! This crate preserves source-native time strings and publisher attribution.
//! It does not authenticate publishers, validate VOEvent XML, or infer truth.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SourceTime {
    /// Exact source-provided time lexical value. No parsing or normalization is performed.
    pub value: String,
    /// Source-provided time scale, such as UTC or TT, when available.
    pub scale: Option<String>,
    /// Source-provided reference frame, when available.
    pub reference_frame: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CitationRelation {
    Followup,
    Supersedes,
    Retraction,
}

impl CitationRelation {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Followup => "followup",
            Self::Supersedes => "supersedes",
            Self::Retraction => "retraction",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Citation {
    /// Identifier of the previously published packet being cited.
    pub target_id: String,
    pub relation: CitationRelation,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Packet {
    /// Stable packet identifier, kept as supplied by the source.
    pub id: String,
    /// SHA-256 supplied by the upstream byte-preserving parser, when available.
    /// This crate carries the value; it does not recompute or authenticate it.
    pub packet_sha256: Option<String>,
    /// Publisher identifier asserted by the packet source; this is not authentication.
    pub publisher: Option<String>,
    /// Fabric receipt time in Unix milliseconds.
    pub received_at_unix_ms: u64,
    /// Exact publication-time lexical value, if supplied. Not parsed or compared.
    pub published_at: Option<String>,
    /// Exact event-time fields, including source time scale/frame. Not normalized.
    pub event_time: Option<SourceTime>,
    pub citations: Vec<Citation>,
}

// JSON projection DTOs intentionally mirror the bounded fields emitted by
// research/voevent-header's parse_packet CLI. Unknown parser fields are ignored
// for forward compatibility; required evidence-bearing fields remain required.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
pub struct ParsedHeaderProjection {
    pub ivoid: String,
    pub packet_sha256: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
pub struct ParsedWhoProjection {
    pub author_ivoid: Option<String>,
    pub source_created_at: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
pub struct ParsedCitationProjection {
    pub target_ivoid: String,
    pub relation: CitationRelation,
}

#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
pub struct ParsedVoEventProjection {
    pub header: ParsedHeaderProjection,
    pub who: ParsedWhoProjection,
    pub citations_present: bool,
    pub citations: Vec<ParsedCitationProjection>,
}

#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
pub struct ReceivedParsedPacket {
    /// Supplied by the ingest receipt layer, not copied from Who/Date.
    pub received_at_unix_ms: u64,
    pub packet: ParsedVoEventProjection,
}

#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
pub struct ReplayManifest {
    pub cutoff_unix_ms: u64,
    pub packets: Vec<ReceivedParsedPacket>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ManifestError {
    InvalidProjection { packet_index: usize, reason: String },
    Replay(ReplayError),
}

impl fmt::Display for ManifestError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidProjection { packet_index, reason } => {
                write!(f, "packet projection {packet_index} rejected: {reason}")
            }
            Self::Replay(error) => write!(f, "lifecycle replay rejected: {error}"),
        }
    }
}

impl Error for ManifestError {}

fn is_ivoid_lexeme(value: &str) -> bool {
    value.starts_with("ivo://")
        && value.len() > "ivo://".len()
        && !value
            .chars()
            .any(|character| character.is_control() || character.is_whitespace())
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn packet_from_projection(
    received: ReceivedParsedPacket,
    packet_index: usize,
) -> Result<Packet, ManifestError> {
    let projection = received.packet;
    let id = projection.header.ivoid;
    if !is_ivoid_lexeme(&id) {
        return Err(ManifestError::InvalidProjection {
            packet_index,
            reason: "root IVOID is empty, malformed, or contains whitespace".to_owned(),
        });
    }

    let packet_sha256 = projection.header.packet_sha256;
    if !is_sha256_hex(&packet_sha256) {
        return Err(ManifestError::InvalidProjection {
            packet_index,
            reason: "packet SHA-256 must be exactly 64 hexadecimal characters".to_owned(),
        });
    }

    let publisher = projection.who.author_ivoid;
    if let Some(author_ivoid) = &publisher {
        if !is_ivoid_lexeme(author_ivoid) {
            return Err(ManifestError::InvalidProjection {
                packet_index,
                reason: "Who/AuthorIVORN is malformed or contains whitespace".to_owned(),
            });
        }
    }

    let published_at = projection.who.source_created_at;
    if published_at.as_ref().is_some_and(|value| value.is_empty()) {
        return Err(ManifestError::InvalidProjection {
            packet_index,
            reason: "Who/Date must not be an empty string".to_owned(),
        });
    }

    if !projection.citations_present && !projection.citations.is_empty() {
        return Err(ManifestError::InvalidProjection {
            packet_index,
            reason: "citations_present=false conflicts with a non-empty citation set".to_owned(),
        });
    }
    if projection.citations_present && projection.citations.is_empty() {
        return Err(ManifestError::InvalidProjection {
            packet_index,
            reason: "citations_present=true requires at least one parsed citation".to_owned(),
        });
    }

    let mut citations = Vec::with_capacity(projection.citations.len());
    for citation in projection.citations {
        if !is_ivoid_lexeme(&citation.target_ivoid) {
            return Err(ManifestError::InvalidProjection {
                packet_index,
                reason: format!(
                    "citation target {:?} is not a valid IVOID lexeme",
                    citation.target_ivoid
                ),
            });
        }
        citations.push(Citation {
            target_id: citation.target_ivoid,
            relation: citation.relation,
        });
    }

    Ok(Packet {
        id,
        packet_sha256: Some(packet_sha256),
        publisher,
        received_at_unix_ms: received.received_at_unix_ms,
        published_at,
        event_time: None,
        citations,
    })
}

/// Convert the parser's JSON projections plus separately supplied ingress
/// receipts into a receipt-time-bounded lifecycle replay.
pub fn replay_manifest(manifest: ReplayManifest) -> Result<Replay, ManifestError> {
    let mut packets = Vec::with_capacity(manifest.packets.len());
    for (index, received) in manifest.packets.into_iter().enumerate() {
        packets.push(packet_from_projection(received, index)?);
    }
    replay_as_of(&packets, manifest.cutoff_unix_ms).map_err(ManifestError::Replay)
}


#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VisiblePacket {
    pub id: String,
    pub packet_sha256: Option<String>,
    pub publisher: Option<String>,
    pub received_at_unix_ms: u64,
    pub published_at: Option<String>,
    pub event_time: Option<SourceTime>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LifecycleEdge {
    /// The newly received packet that makes the citation assertion.
    pub asserting_packet_id: String,
    pub asserting_packet_sha256: Option<String>,
    /// The prior packet named by the citation.
    pub cited_packet_id: String,
    pub cited_packet_sha256: Option<String>,
    pub relation: CitationRelation,
    /// Publisher attribution copied from the asserting packet, not verified identity.
    pub asserting_publisher: Option<String>,
    /// Receipt time of the asserting packet; not event time or publication time.
    pub asserted_packet_received_at_unix_ms: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct UnresolvedCitation {
    pub asserting_packet_id: String,
    pub asserting_packet_sha256: Option<String>,
    pub cited_packet_id: String,
    pub relation: CitationRelation,
    pub asserting_publisher: Option<String>,
    pub asserted_packet_received_at_unix_ms: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct LifecycleEffect {
    pub packet_id: String,
    /// Packets that assert this packet is superseded. History is never deleted.
    pub superseded_by: Vec<String>,
    /// Packets that assert this packet is retracted. History is never deleted.
    pub retracted_by: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReplayDiagnostic {
    SelfReference {
        packet_id: String,
        relation: CitationRelation,
    },
    SupersessionCycle {
        asserting_packet_id: String,
        cited_packet_id: String,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Replay {
    pub cutoff_unix_ms: u64,
    pub visible_packets: Vec<VisiblePacket>,
    pub edges: Vec<LifecycleEdge>,
    pub unresolved_citations: Vec<UnresolvedCitation>,
    pub effects: Vec<LifecycleEffect>,
    /// Structural contradictions are reported, never used to erase or rewrite edges.
    pub diagnostics: Vec<ReplayDiagnostic>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReplayError {
    EmptyPacketId { received_at_unix_ms: u64 },
    EmptyPublisher { packet_id: String },
    DuplicatePacketId { packet_id: String },
    EmptyCitationTarget { packet_id: String },
}

impl fmt::Display for ReplayError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyPacketId { received_at_unix_ms } => {
                write!(f, "packet at receipt time {received_at_unix_ms} has an empty identifier")
            }
            Self::EmptyPublisher { packet_id } => {
                write!(f, "packet {packet_id:?} has an empty publisher identifier")
            }
            Self::DuplicatePacketId { packet_id } => {
                write!(f, "duplicate packet identifier in the as-of view: {packet_id:?}")
            }
            Self::EmptyCitationTarget { packet_id } => {
                write!(f, "packet {packet_id:?} contains an empty citation target")
            }
        }
    }
}

impl Error for ReplayError {}

/// Reconstruct the lifecycle graph using only packets received at or before the cutoff.
///
/// Only receipt time controls visibility. Publication time and event time are copied verbatim
/// and are never used to make a packet visible earlier. Validation is cutoff-scoped, so
/// malformed future packets cannot poison a past view.
pub fn replay_as_of(
    packets: &[Packet],
    cutoff_unix_ms: u64,
) -> Result<Replay, ReplayError> {
    let mut visible: Vec<&Packet> = packets
        .iter()
        .filter(|packet| packet.received_at_unix_ms <= cutoff_unix_ms)
        .collect();

    let mut seen_ids = BTreeSet::new();
    for packet in &visible {
        if packet.id.trim().is_empty() {
            return Err(ReplayError::EmptyPacketId {
                received_at_unix_ms: packet.received_at_unix_ms,
            });
        }
        if let Some(publisher) = &packet.publisher {
            if publisher.trim().is_empty() {
                return Err(ReplayError::EmptyPublisher {
                    packet_id: packet.id.clone(),
                });
            }
        }
        if !seen_ids.insert(packet.id.clone()) {
            return Err(ReplayError::DuplicatePacketId {
                packet_id: packet.id.clone(),
            });
        }
        for citation in &packet.citations {
            if citation.target_id.trim().is_empty() {
                return Err(ReplayError::EmptyCitationTarget {
                    packet_id: packet.id.clone(),
                });
            }
        }
    }

    // Explicit ordering makes replay output independent of caller input order.
    visible.sort_by_key(|packet| (packet.received_at_unix_ms, packet.id.clone()));

    let visible_ids: BTreeSet<String> = visible.iter().map(|packet| packet.id.clone()).collect();
    let visible_digests: BTreeMap<String, Option<String>> = visible
        .iter()
        .map(|packet| (packet.id.clone(), packet.packet_sha256.clone()))
        .collect();
    let visible_packets = visible
        .iter()
        .map(|packet| VisiblePacket {
            id: packet.id.clone(),
            packet_sha256: packet.packet_sha256.clone(),
            publisher: packet.publisher.clone(),
            received_at_unix_ms: packet.received_at_unix_ms,
            published_at: packet.published_at.clone(),
            event_time: packet.event_time.clone(),
        })
        .collect();

    let mut edges = Vec::new();
    let mut unresolved_citations = Vec::new();
    let mut effects_by_packet: BTreeMap<String, LifecycleEffect> = BTreeMap::new();
    let mut diagnostics = Vec::new();
    let mut supersession_graph: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();

    for packet in &visible {
        for citation in &packet.citations {
            if citation.target_id == packet.id {
                diagnostics.push(ReplayDiagnostic::SelfReference {
                    packet_id: packet.id.clone(),
                    relation: citation.relation,
                });
            }

            if citation.relation == CitationRelation::Supersedes {
                if has_path(&supersession_graph, &citation.target_id, &packet.id) {
                    diagnostics.push(ReplayDiagnostic::SupersessionCycle {
                        asserting_packet_id: packet.id.clone(),
                        cited_packet_id: citation.target_id.clone(),
                    });
                }
                supersession_graph
                    .entry(packet.id.clone())
                    .or_default()
                    .insert(citation.target_id.clone());
            }

            let edge = LifecycleEdge {
                asserting_packet_id: packet.id.clone(),
                asserting_packet_sha256: packet.packet_sha256.clone(),
                cited_packet_id: citation.target_id.clone(),
                cited_packet_sha256: visible_digests
                    .get(&citation.target_id)
                    .cloned()
                    .flatten(),
                relation: citation.relation,
                asserting_publisher: packet.publisher.clone(),
                asserted_packet_received_at_unix_ms: packet.received_at_unix_ms,
            };

            if visible_ids.contains(&citation.target_id) {
                let effect = effects_by_packet
                    .entry(citation.target_id.clone())
                    .or_insert_with(|| LifecycleEffect {
                        packet_id: citation.target_id.clone(),
                        ..LifecycleEffect::default()
                    });
                match citation.relation {
                    CitationRelation::Followup => {}
                    CitationRelation::Supersedes => {
                        effect.superseded_by.push(packet.id.clone());
                    }
                    CitationRelation::Retraction => {
                        effect.retracted_by.push(packet.id.clone());
                    }
                }
            } else {
                unresolved_citations.push(UnresolvedCitation {
                    asserting_packet_id: packet.id.clone(),
                    asserting_packet_sha256: packet.packet_sha256.clone(),
                    cited_packet_id: citation.target_id.clone(),
                    relation: citation.relation,
                    asserting_publisher: packet.publisher.clone(),
                    asserted_packet_received_at_unix_ms: packet.received_at_unix_ms,
                });
            }
            // Keep every source assertion in the edge stream, including unresolved edges.
            edges.push(edge);
        }
    }

    Ok(Replay {
        cutoff_unix_ms,
        visible_packets,
        edges,
        unresolved_citations,
        effects: effects_by_packet.into_values().collect(),
        diagnostics,
    })
}

fn has_path(
    graph: &BTreeMap<String, BTreeSet<String>>,
    start: &str,
    goal: &str,
) -> bool {
    let mut pending = vec![start.to_owned()];
    let mut visited = BTreeSet::new();

    while let Some(node) = pending.pop() {
        if node == goal {
            return true;
        }
        if visited.insert(node.clone()) {
            if let Some(next_nodes) = graph.get(&node) {
                pending.extend(next_nodes.iter().cloned());
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(id: &str, publisher: &str, received_at_unix_ms: u64) -> Packet {
        Packet {
            id: id.to_owned(),
            packet_sha256: None,
            publisher: Some(publisher.to_owned()),
            received_at_unix_ms,
            published_at: None,
            event_time: None,
            citations: Vec::new(),
        }
    }

    fn citation(target_id: &str, relation: CitationRelation) -> Citation {
        Citation {
            target_id: target_id.to_owned(),
            relation,
        }
    }

    fn effect<'a>(replay: &'a Replay, packet_id: &str) -> &'a LifecycleEffect {
        replay
            .effects
            .iter()
            .find(|item| item.packet_id == packet_id)
            .expect("expected lifecycle effect")
    }

    #[test]
    fn parser_projection_manifest_preserves_creation_receipt_and_digest_separately() {
        let manifest_json = r#"{
            "cutoff_unix_ms": 200,
            "packets": [
                {
                    "received_at_unix_ms": 100,
                    "packet": {
                        "header": {
                            "ivoid": "ivo://survey.example/event?one",
                            "version": "2.1",
                            "packet_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
                        },
                        "who": {
                            "present": true,
                            "author_ivoid": "ivo://publisher.example/stream",
                            "source_created_at": "2026-11-14T08:00:00Z"
                        },
                        "citations_present": false,
                        "citations": [],
                        "citations_description": null
                    }
                },
                {
                    "received_at_unix_ms": 300,
                    "packet": {
                        "header": {
                            "ivoid": "ivo://survey.example/event?two",
                            "version": "2.1",
                            "packet_sha256": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
                        },
                        "who": {
                            "present": true,
                            "author_ivoid": "ivo://review.example/stream",
                            "source_created_at": "2026-11-14T07:55:00Z"
                        },
                        "citations_present": true,
                        "citations": [{
                            "target_ivoid": "ivo://survey.example/event?one",
                            "relation": "supersedes",
                            "attributes": [{"name": "cite", "value": "supersedes"}]
                        }],
                        "citations_description": "Reprocessed."
                    }
                }
            ]
        }"#;
        let manifest: ReplayManifest =
            serde_json::from_str(manifest_json).expect("parser projection manifest");
        let before = replay_manifest(manifest.clone()).expect("historical replay");
        assert_eq!(before.visible_packets.len(), 1);
        assert!(before.edges.is_empty());

        let mut later = manifest;
        later.cutoff_unix_ms = 300;
        let after = replay_manifest(later).expect("later replay");
        assert_eq!(after.visible_packets.len(), 2);
        assert_eq!(after.edges.len(), 1);
        assert_eq!(
            after.visible_packets[1].publisher.as_deref(),
            Some("ivo://review.example/stream")
        );
        assert_eq!(
            after.visible_packets[1].published_at.as_deref(),
            Some("2026-11-14T07:55:00Z")
        );
        assert_eq!(
            after.visible_packets[1].packet_sha256.as_deref(),
            Some("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb")
        );
        assert_eq!(
            after.edges[0].asserting_packet_received_at_unix_ms,
            300
        );
        assert_eq!(
            after.edges[0].asserting_packet_sha256.as_deref(),
            Some("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb")
        );
    }

    #[test]
    fn parser_projection_manifest_rejects_invalid_digest_and_citation_presence_mismatch() {
        let invalid_digest = ReplayManifest {
            cutoff_unix_ms: 10,
            packets: vec![ReceivedParsedPacket {
                received_at_unix_ms: 1,
                packet: ParsedVoEventProjection {
                    header: ParsedHeaderProjection {
                        ivoid: "ivo://survey.example/event?one".to_owned(),
                        packet_sha256: "not-a-digest".to_owned(),
                    },
                    who: ParsedWhoProjection {
                        author_ivoid: None,
                        source_created_at: None,
                    },
                    citations_present: false,
                    citations: Vec::new(),
                },
            }],
        };
        assert!(matches!(
            replay_manifest(invalid_digest),
            Err(ManifestError::InvalidProjection { .. })
        ));

        let inconsistent = ReplayManifest {
            cutoff_unix_ms: 10,
            packets: vec![ReceivedParsedPacket {
                received_at_unix_ms: 1,
                packet: ParsedVoEventProjection {
                    header: ParsedHeaderProjection {
                        ivoid: "ivo://survey.example/event?one".to_owned(),
                        packet_sha256: "c".repeat(64),
                    },
                    who: ParsedWhoProjection {
                        author_ivoid: None,
                        source_created_at: None,
                    },
                    citations_present: true,
                    citations: Vec::new(),
                },
            }],
        };
        assert!(matches!(
            replay_manifest(inconsistent),
            Err(ManifestError::InvalidProjection { .. })
        ));
    }

    #[test]
    fn late_arrival_never_leaks_into_an_earlier_cutoff() {
        let original = packet("ivo://survey/a#event-1", "ivo://survey/a", 10);
        let mut correction = packet("ivo://survey/b#event-2", "ivo://survey/b", 30);
        correction.published_at = Some("2026-11-14T09:00:00Z".to_owned());
        correction.citations.push(citation(
            &original.id,
            CitationRelation::Supersedes,
        ));

        let history = vec![original, correction];
        let earlier = replay_as_of(&history, 20).expect("earlier replay");
        assert_eq!(earlier.visible_packets.len(), 1);
        assert!(earlier.edges.is_empty());
        assert!(earlier.effects.is_empty());

        let later = replay_as_of(&history, 30).expect("later replay");
        assert_eq!(later.visible_packets.len(), 2);
        assert_eq!(
            effect(&later, "ivo://survey/a#event-1").superseded_by,
            vec!["ivo://survey/b#event-2"]
        );
    }

    #[test]
    fn retraction_by_another_publisher_is_attributed_and_history_is_retained() {
        let original = packet("ivo://survey/a#candidate", "ivo://survey/a", 10);
        let mut retraction = packet("ivo://review/b#notice", "ivo://review/b", 20);
        retraction
            .citations
            .push(citation(&original.id, CitationRelation::Retraction));

        let replay = replay_as_of(&[retraction, original], 20).expect("replay");
        assert_eq!(replay.visible_packets.len(), 2);
        assert_eq!(
            effect(&replay, "ivo://survey/a#candidate").retracted_by,
            vec!["ivo://review/b#notice"]
        );
        assert_eq!(
            replay.edges[0].asserting_publisher.as_deref(),
            Some("ivo://review/b")
        );
    }

    #[test]
    fn followup_does_not_mark_the_cited_packet_superseded_or_retracted() {
        let original = packet("ivo://survey/a#event", "ivo://survey/a", 10);
        let mut followup = packet("ivo://survey/b#followup", "ivo://survey/b", 20);
        followup
            .citations
            .push(citation(&original.id, CitationRelation::Followup));

        let replay = replay_as_of(&[original, followup], 20).expect("replay");
        assert!(replay.effects.is_empty());
        assert_eq!(replay.edges.len(), 1);
        assert_eq!(replay.edges[0].relation, CitationRelation::Followup);
    }

    #[test]
    fn unresolved_reference_resolves_only_when_target_is_received() {
        let mut citing = packet("ivo://survey/b#followup", "ivo://survey/b", 20);
        citing
            .citations
            .push(citation("ivo://survey/a#earlier", CitationRelation::Followup));
        let mut target = packet("ivo://survey/a#earlier", "ivo://survey/a", 50);
        target.published_at = Some("2026-11-14T08:00:00Z".to_owned());

        let before = replay_as_of(&[citing.clone(), target.clone()], 30).expect("before");
        assert_eq!(before.unresolved_citations.len(), 1);
        assert_eq!(before.edges.len(), 1);

        let after = replay_as_of(&[citing, target], 50).expect("after");
        assert!(after.unresolved_citations.is_empty());
        assert_eq!(after.edges.len(), 1);
        assert_eq!(after.visible_packets.len(), 2);
    }

    #[test]
    fn future_duplicate_does_not_poison_past_replay() {
        let valid = packet("ivo://survey/a#event", "ivo://survey/a", 10);
        let duplicate = packet("ivo://survey/a#event", "ivo://survey/b", 100);

        let history = vec![valid, duplicate];
        assert!(replay_as_of(&history, 50).is_ok());
        assert!(matches!(
            replay_as_of(&history, 100),
            Err(ReplayError::DuplicatePacketId { .. })
        ));
    }

    #[test]
    fn future_malformed_packet_does_not_poison_past_replay() {
        let valid = packet("ivo://survey/a#event", "ivo://survey/a", 10);
        let mut malformed = packet("placeholder", "ivo://survey/c", 200);
        malformed.id.clear();

        let history = vec![valid, malformed];
        assert!(replay_as_of(&history, 50).is_ok());
        assert!(matches!(
            replay_as_of(&history, 200),
            Err(ReplayError::EmptyPacketId { .. })
        ));
    }

    #[test]
    fn supersession_cycle_is_reported_without_rewriting_the_citation_graph() {
        let mut a = packet("ivo://survey/a#1", "ivo://survey/a", 10);
        let mut b = packet("ivo://survey/b#1", "ivo://survey/b", 20);
        a.citations
            .push(citation(&b.id, CitationRelation::Supersedes));
        b.citations
            .push(citation(&a.id, CitationRelation::Supersedes));

        let replay = replay_as_of(&[a, b], 20).expect("replay");
        assert_eq!(replay.edges.len(), 2);
        assert!(replay.diagnostics.iter().any(|diagnostic| matches!(
            diagnostic,
            ReplayDiagnostic::SupersessionCycle { .. }
        )));
    }

    #[test]
    fn source_time_lexemes_and_time_scale_are_preserved_without_conversion() {
        let mut source = packet("ivo://survey/a#event", "ivo://survey/a", 10);
        source.event_time = Some(SourceTime {
            value: "2026-11-14T12:00:00.000".to_owned(),
            scale: Some("TT".to_owned()),
            reference_frame: Some("source-native-frame".to_owned()),
        });
        source.published_at = Some("2026-11-14T11:58:00".to_owned());

        let replay = replay_as_of(&[source], 10).expect("replay");
        let visible = &replay.visible_packets[0];
        assert_eq!(
            visible.event_time.as_ref().expect("event time").scale.as_deref(),
            Some("TT")
        );
        assert_eq!(
            visible.event_time.as_ref().expect("event time").value,
            "2026-11-14T12:00:00.000"
        );
        assert_eq!(
            visible.published_at.as_deref(),
            Some("2026-11-14T11:58:00")
        );
    }

    #[test]
    fn output_order_is_deterministic_for_different_input_packet_orders() {
        let a = packet("ivo://survey/a#1", "ivo://survey/a", 10);
        let b = packet("ivo://survey/b#1", "ivo://survey/b", 10);
        let mut c = packet("ivo://survey/c#1", "ivo://survey/c", 20);
        c.citations.push(citation(&a.id, CitationRelation::Followup));
        c.citations.push(citation(&b.id, CitationRelation::Retraction));

        let left = replay_as_of(&[a.clone(), b.clone(), c.clone()], 20).expect("left");
        let right = replay_as_of(&[c, b, a], 20).expect("right");
        assert_eq!(left, right);
        assert_eq!(
            left.visible_packets
                .iter()
                .map(|packet| packet.id.as_str())
                .collect::<Vec<_>>(),
            vec!["ivo://survey/a#1", "ivo://survey/b#1", "ivo://survey/c#1"]
        );
    }

    #[test]
    fn simultaneous_supersession_and_retraction_are_both_preserved() {
        let original = packet("ivo://survey/a#event", "ivo://survey/a", 10);
        let mut superseder = packet("ivo://survey/b#revision", "ivo://survey/b", 20);
        superseder
            .citations
            .push(citation(&original.id, CitationRelation::Supersedes));
        let mut retractor = packet("ivo://review/c#notice", "ivo://review/c", 30);
        retractor
            .citations
            .push(citation(&original.id, CitationRelation::Retraction));

        let replay = replay_as_of(&[original, superseder, retractor], 30).expect("replay");
        let state = effect(&replay, "ivo://survey/a#event");
        assert_eq!(state.superseded_by, vec!["ivo://survey/b#revision"]);
        assert_eq!(state.retracted_by, vec!["ivo://review/c#notice"]);
    }


    #[test]
    fn lifecycle_edges_carry_exact_packet_digest_bindings_when_available() {
        let mut original = packet("ivo://survey/a#event", "ivo://survey/a", 10);
        original.packet_sha256 = Some("b".repeat(64));
        let mut followup = packet("ivo://survey/b#followup", "ivo://survey/b", 20);
        followup.packet_sha256 = Some("a".repeat(64));
        followup
            .citations
            .push(citation(&original.id, CitationRelation::Followup));

        let replay = replay_as_of(&[original, followup], 20).expect("replay");
        assert_eq!(
            replay.edges[0].asserting_packet_sha256,
            Some("a".repeat(64))
        );
        assert_eq!(
            replay.edges[0].cited_packet_sha256,
            Some("b".repeat(64))
        );
    }

    #[test]
    fn self_reference_is_reported_and_retained() {
        let mut source = packet("ivo://survey/a#event", "ivo://survey/a", 10);
        source
            .citations
            .push(citation(&source.id, CitationRelation::Followup));

        let replay = replay_as_of(&[source], 10).expect("replay");
        assert_eq!(replay.edges.len(), 1);
        assert!(replay.diagnostics.iter().any(|diagnostic| matches!(
            diagnostic,
            ReplayDiagnostic::SelfReference { .. }
        )));
    }
}

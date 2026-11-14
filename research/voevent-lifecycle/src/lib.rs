#![forbid(unsafe_code)]

//! A read-only, receipt-time-bounded replay of VOEvent lifecycle citations.
//!
//! This crate parses exact source XML using the shared VOEvent parser and
//! preserves receipt-time-bounded lifecycle history. It does not authenticate
//! the ingress receipt or publisher, qualify rights, or infer scientific truth.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

use mycelix_voevent_header::{
    parse_packet as parse_voevent_packet, CitationRelation as SourceCitationRelation,
    PacketRole as SourcePacketRole,
};
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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PacketRole {
    Observation,
    Prediction,
    Utility,
    Test,
}

impl PacketRole {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Observation => "observation",
            Self::Prediction => "prediction",
            Self::Utility => "utility",
            Self::Test => "test",
        }
    }
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
    /// Effective VOEvent role; test/utility/prediction must not be collapsed into observation.
    pub role: PacketRole,
    /// True only when observation was supplied through the VOEvent 2.1 implicit default.
    pub role_was_implicit: bool,
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

/// A raw source packet paired with an independently supplied ingress receipt.
///
/// Receipt identity and time are assertions of the ingest boundary. This crate
/// does not authenticate the receipt or replace a signed append-only receipt log.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
pub struct ReceivedPacket {
    pub received_at_unix_ms: u64,
    pub raw_packet_xml: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
pub struct ReplayManifest {
    pub cutoff_unix_ms: u64,
    pub packets: Vec<ReceivedPacket>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ManifestError {
    EmptyOrOversizedPacket { packet_index: usize },
    PacketParse { packet_index: usize, message: String },
    Replay(ReplayError),
}

impl fmt::Display for ManifestError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyOrOversizedPacket { packet_index } => {
                write!(f, "source packet {packet_index} is empty or exceeds 2 MiB")
            }
            Self::PacketParse { packet_index, message } => {
                write!(f, "source packet {packet_index} rejected by VOEvent parser: {message}")
            }
            Self::Replay(error) => write!(f, "lifecycle replay rejected: {error}"),
        }
    }
}

impl Error for ManifestError {}

const MAX_VOEVENT_PACKET_BYTES: usize = 2 * 1024 * 1024;

/// Parse the exact source bytes and derive every packet field from that parse.
/// There is no caller-supplied metadata projection to become inconsistent with
/// the digest. The only independent field is the ingress receipt timestamp.
fn packet_from_source(
    received: ReceivedPacket,
    packet_index: usize,
) -> Result<Packet, ManifestError> {
    let raw_packet_bytes = received.raw_packet_xml.as_bytes();
    if raw_packet_bytes.is_empty() || raw_packet_bytes.len() > MAX_VOEVENT_PACKET_BYTES {
        return Err(ManifestError::EmptyOrOversizedPacket { packet_index });
    }

    let parsed = parse_voevent_packet(raw_packet_bytes).map_err(|error| {
        ManifestError::PacketParse {
            packet_index,
            message: error.to_string(),
        }
    })?;

    let role = match parsed.header.effective_role {
        SourcePacketRole::Observation => PacketRole::Observation,
        SourcePacketRole::Prediction => PacketRole::Prediction,
        SourcePacketRole::Utility => PacketRole::Utility,
        SourcePacketRole::Test => PacketRole::Test,
    };
    let citations = parsed
        .citations
        .into_iter()
        .map(|citation| Citation {
            target_id: citation.target_ivoid,
            relation: match citation.relation {
                SourceCitationRelation::Followup => CitationRelation::Followup,
                SourceCitationRelation::Supersedes => CitationRelation::Supersedes,
                SourceCitationRelation::Retraction => CitationRelation::Retraction,
            },
        })
        .collect();

    Ok(Packet {
        id: parsed.header.ivoid,
        role,
        role_was_implicit: parsed.header.role_was_implicit,
        packet_sha256: Some(parsed.header.packet_sha256),
        publisher: parsed.who.author_ivoid,
        received_at_unix_ms: received.received_at_unix_ms,
        published_at: parsed.who.source_created_at,
        event_time: None,
        citations,
    })
}

/// Reconstruct the event graph from exact source XML and separate ingress receipts.
///
/// Receipt-time cutoff is applied before parsing: future packets cannot poison a
/// historical replay with malformed XML or metadata. Each included packet is
/// parsed directly by the same versioned parser crate used by the CLI.
pub fn replay_manifest(manifest: ReplayManifest) -> Result<Replay, ManifestError> {
    let mut packets = Vec::with_capacity(manifest.packets.len());
    for (index, received) in manifest.packets.into_iter().enumerate() {
        if received.received_at_unix_ms > manifest.cutoff_unix_ms {
            continue;
        }
        packets.push(packet_from_source(received, index)?);
    }
    replay_as_of(&packets, manifest.cutoff_unix_ms).map_err(ManifestError::Replay)
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VisiblePacket {
    pub id: String,
    pub role: PacketRole,
    pub role_was_implicit: bool,
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
    pub asserting_packet_role: PacketRole,
    pub asserting_role_was_implicit: bool,
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
    pub asserting_packet_role: PacketRole,
    pub asserting_role_was_implicit: bool,
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
            role: packet.role,
            role_was_implicit: packet.role_was_implicit,
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
                asserting_packet_role: packet.role,
                asserting_role_was_implicit: packet.role_was_implicit,
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
                    asserting_packet_role: packet.role,
                    asserting_role_was_implicit: packet.role_was_implicit,
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
            role: PacketRole::Observation,
            role_was_implicit: true,
            packet_sha256: None,
            publisher: Some(publisher.to_owned()),
            received_at_unix_ms,
            published_at: None,
            event_time: None,
            citations: Vec::new(),
        }
    }

    fn source_packet(
        ivoid: &str,
        role: Option<&str>,
        who: Option<(&str, &str)>,
        citations: &[(&str, &str)],
    ) -> String {
        let role_attr = role.map(|value| format!(" role=\\"{value}\\"")).unwrap_or_default();
        let who_xml = who
            .map(|(author, date)| format!(
                "<Who><AuthorIVORN>{author}</AuthorIVORN><Date>{date}</Date></Who>"
            ))
            .unwrap_or_default();
        let citation_xml = if citations.is_empty() {
            String::new()
        } else {
            let edges = citations.iter().map(|(relation, target)| format!(
                "<EventIVORN cite=\\"{relation}\\">{target}</EventIVORN>"
            )).collect::<String>();
            format!("<Citations>{edges}</Citations>")
        };
        format!(
            "<VOEvent ivorn=\\"{ivoid}\\" version=\\"2.1\\"{role_attr}>{who_xml}{citation_xml}<What/></VOEvent>"
        )
    }

    fn received(received_at_unix_ms: u64, raw_packet_xml: String) -> ReceivedPacket {
        ReceivedPacket {
            received_at_unix_ms,
            raw_packet_xml,
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
    fn source_native_manifest_parses_exact_xml_and_preserves_receipt_time() {
        let original = source_packet(
            "ivo://survey.example/event?one",
            None,
            Some(("ivo://publisher.example/stream", "2026-11-14T08:00:00Z")),
            &[],
        );
        let correction = source_packet(
            "ivo://survey.example/event?two",
            Some("observation"),
            Some(("ivo://review.example/stream", "2026-11-14T07:55:00Z")),
            &[("supersedes", "ivo://survey.example/event?one")],
        );

        let manifest = ReplayManifest {
            cutoff_unix_ms: 200,
            packets: vec![
                received(100, original),
                received(300, correction),
            ],
        };
        let before = replay_manifest(manifest.clone()).expect("historical replay");
        assert_eq!(before.visible_packets.len(), 1);
        assert!(before.edges.is_empty());

        let mut later = manifest;
        later.cutoff_unix_ms = 300;
        let after = replay_manifest(later).expect("later replay");
        assert_eq!(after.visible_packets.len(), 2);
        assert_eq!(after.edges.len(), 1);
        assert_eq!(after.edges[0].relation, CitationRelation::Supersedes);
        assert_eq!(
            after.visible_packets[1].publisher.as_deref(),
            Some("ivo://review.example/stream")
        );
        assert_eq!(
            after.visible_packets[1].published_at.as_deref(),
            Some("2026-11-14T07:55:00Z")
        );
        assert_eq!(after.visible_packets[1].received_at_unix_ms, 300);
        assert_eq!(
            after.edges[0].asserting_packet_sha256,
            after.visible_packets[1].packet_sha256
        );
    }

    #[test]
    fn checked_in_source_packet_fixture_replays_without_a_metadata_projection() {
        let manifest: ReplayManifest = serde_json::from_str(include_str!(
            "../tests/fixtures/source-packet-thread.json"
        ))
        .expect("raw source packet fixture");
        let historical = replay_manifest(manifest.clone()).expect("historical fixture replay");
        assert_eq!(historical.visible_packets.len(), 1);
        assert!(historical.edges.is_empty());

        let mut later = manifest;
        later.cutoff_unix_ms = 300;
        let complete = replay_manifest(later).expect("complete fixture replay");
        assert_eq!(complete.visible_packets.len(), 2);
        assert_eq!(complete.edges.len(), 1);
        assert_eq!(complete.edges[0].relation, CitationRelation::Supersedes);
        assert!(complete.visible_packets.iter().all(|packet| {
            packet.packet_sha256.as_ref().is_some_and(|digest| digest.len() == 64)
        }));
    }

    #[test]
    fn source_parser_preserves_explicit_and_implicit_roles() {
        let implicit = received(
            1,
            source_packet("ivo://survey.example/implicit", None, None, &[]),
        );
        let explicit = received(
            2,
            source_packet("ivo://survey.example/test", Some("test"), None, &[]),
        );
        let replay = replay_manifest(ReplayManifest {
            cutoff_unix_ms: 10,
            packets: vec![implicit, explicit],
        })
        .expect("role parsing");

        assert_eq!(replay.visible_packets[0].role, PacketRole::Observation);
        assert!(replay.visible_packets[0].role_was_implicit);
        assert_eq!(replay.visible_packets[1].role, PacketRole::Test);
        assert!(!replay.visible_packets[1].role_was_implicit);
    }

    #[test]
    fn source_native_parser_rejects_malformed_xml_and_invalid_citations() {
        let malformed = received(1, "<VOEvent".to_owned());
        assert!(matches!(
            replay_manifest(ReplayManifest {
                cutoff_unix_ms: 10,
                packets: vec![malformed],
            }),
            Err(ManifestError::PacketParse { .. })
        ));

        let invalid_relation = received(
            1,
            "<VOEvent ivorn=\\"ivo://survey.example/a\\" version=\\"2.1\\"><Citations><EventIVORN cite=\\"associated\\">ivo://survey.example/b</EventIVORN></Citations></VOEvent>".to_owned(),
        );
        assert!(matches!(
            replay_manifest(ReplayManifest {
                cutoff_unix_ms: 10,
                packets: vec![invalid_relation],
            }),
            Err(ManifestError::PacketParse { .. })
        ));
    }

    #[test]
    fn malformed_future_source_xml_does_not_poison_historical_cutoff() {
        let valid = received(
            1,
            source_packet("ivo://survey.example/valid", None, None, &[]),
        );
        let malformed_future = received(30, "<not-xml".to_owned());

        let historical = replay_manifest(ReplayManifest {
            cutoff_unix_ms: 10,
            packets: vec![valid.clone(), malformed_future.clone()],
        })
        .expect("future malformed packet is outside historical view");
        assert_eq!(historical.visible_packets.len(), 1);

        assert!(matches!(
            replay_manifest(ReplayManifest {
                cutoff_unix_ms: 30,
                packets: vec![valid, malformed_future],
            }),
            Err(ManifestError::PacketParse { .. })
        ));
    }

    #[test]
    fn raw_packet_size_is_bounded_before_parser_invocation() {
        let too_large = received(1, " ".repeat(MAX_VOEVENT_PACKET_BYTES + 1));
        assert_eq!(
            replay_manifest(ReplayManifest {
                cutoff_unix_ms: 10,
                packets: vec![too_large],
            }),
            Err(ManifestError::EmptyOrOversizedPacket { packet_index: 0 })
        );
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

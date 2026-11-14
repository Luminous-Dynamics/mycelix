// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bounded VOEvent 2.1 root-header extraction.
//!
//! This crate deliberately does not claim full VOEvent XSD validation. It
//! validates the root packet identity/version/role, preserves all root
//! attributes and hashes the exact packet bytes. Domain fields, STC semantics,
//! citations, publisher authentication, rights and scientific validity remain
//! separate qualification steps.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::io::Cursor;
use thiserror::Error;

pub const MAX_PACKET_BYTES: usize = 2 * 1024 * 1024;
pub const VOEVENT_VERSION: &str = "2.1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PacketRole {
    Observation,
    Prediction,
    Utility,
    Test,
}

impl PacketRole {
    fn parse(value: &str) -> Result<Self, ParseError> {
        match value {
            "observation" => Ok(Self::Observation),
            "prediction" => Ok(Self::Prediction),
            "utility" => Ok(Self::Utility),
            "test" => Ok(Self::Test),
            other => Err(ParseError::InvalidRole(other.to_owned())),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct RootAttribute {
    pub name: String,
    pub value: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct VoEventHeader {
    /// Exact source-native value of the VOEvent XML ivorn attribute.
    pub ivoid: String,
    pub version: String,
    /// None means the packet omitted role; the effective role is still
    /// observation according to the VOEvent 2.1 default.
    pub raw_role: Option<String>,
    pub effective_role: PacketRole,
    pub role_was_implicit: bool,
    /// All root attributes in source order, including uninterpreted values.
    pub root_attributes: Vec<RootAttribute>,
    pub packet_byte_length: usize,
    pub packet_sha256: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CitationRelation {
    Followup,
    Supersedes,
    Retraction,
}

impl CitationRelation {
    fn parse(value: &str) -> Result<Self, ParseError> {
        match value {
            "followup" => Ok(Self::Followup),
            "supersedes" => Ok(Self::Supersedes),
            "retraction" => Ok(Self::Retraction),
            other => Err(ParseError::InvalidCitationRelation(other.to_owned())),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct VoEventCitation {
    /// Decoded IVOID text from the EventIVORN element, after XML entity resolution.
    pub target_ivoid: String,
    pub relation: CitationRelation,
    /// All EventIVORN attributes in source order, including uninterpreted values.
    pub attributes: Vec<RootAttribute>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct VoEventPacket {
    pub header: VoEventHeader,
    /// Absence is meaningful: a packet without Citations announces a new event.
    pub citations_present: bool,
    pub citations: Vec<VoEventCitation>,
    /// Optional source text from Citations/Description.
    pub citations_description: Option<String>,
}


#[derive(Debug, Error, PartialEq, Eq)]
pub enum ParseError {
    #[error("packet is empty")]
    EmptyPacket,
    #[error("packet exceeds the 2 MiB parser limit")]
    PacketTooLarge,
    #[error("only UTF-8 XML packets are supported by this parser")]
    UnsupportedEncoding,
    #[error("malformed XML: {0}")]
    MalformedXml(String),
    #[error("DOCTYPE declarations are not accepted")]
    DoctypeNotAllowed,
    #[error("the document must have exactly one unprefixed VOEvent root element")]
    InvalidRoot,
    #[error("the document contains more than one root element")]
    MultipleRootElements,
    #[error("the XML document ended before the root element was closed")]
    IncompleteDocument,
    #[error("mismatched XML element: expected closing {expected}, found {found}")]
    MismatchedElement { expected: String, found: String },
    #[error("non-whitespace text occurs outside the root element")]
    TextOutsideRoot,
    #[error("required root attribute is missing: {0}")]
    MissingAttribute(&'static str),
    #[error("unsupported VOEvent version: {0}")]
    UnsupportedVersion(String),
    #[error("invalid IVOID in root ivorn attribute: {0}")]
    InvalidIvoid(String),
    #[error("unsupported VOEvent role: {0}")]
    InvalidRole(String),
    #[error("EventIVORN is missing its required cite attribute")]
    MissingCitationAttribute,
    #[error("unsupported EventIVORN cite relation: {0}")]
    InvalidCitationRelation(String),
    #[error("invalid IVOID in EventIVORN text: {0}")]
    InvalidCitationTarget(String),
    #[error("a VOEvent packet contains more than one Citations element")]
    DuplicateCitations,
    #[error("Citations must contain at least one EventIVORN element")]
    EmptyCitations,
    #[error("Citations contains more than one Description element")]
    DuplicateCitationsDescription,
    #[error("EventIVORN and Citations/Description must contain simple text only; found nested element {0}")]
    NestedCitationElement(String),
    #[error("unsupported or invalid XML character reference in citation text: {0}")]
    InvalidCitationCharacterReference(String),
}

fn validate_ivoid(value: &str) -> Result<(), String> {
    // Conservative lexical check only; registry resolution is a separate qualification step.
    if !value.starts_with("ivo://")
        || value.len() <= "ivo://".len()
        || value
            .chars()
            .any(|character| character.is_control() || character.is_whitespace())
    {
        return Err(value.to_owned());
    }
    Ok(())
}

fn parse_root_attributes(element: &BytesStart<'_>) -> Result<VoEventHeader, ParseError> {
    if element.name().as_ref() != b"VOEvent" {
        return Err(ParseError::InvalidRoot);
    }

    let mut root_attributes = Vec::new();
    let mut ivoid: Option<String> = None;
    let mut version: Option<String> = None;
    let mut raw_role: Option<String> = None;

    for attribute in element.attributes().with_checks(true) {
        let attribute = attribute.map_err(|error| ParseError::MalformedXml(error.to_string()))?;
        let name = std::str::from_utf8(attribute.key.as_ref())
            .map_err(|_| ParseError::MalformedXml("non-UTF-8 attribute name".to_owned()))?
            .to_owned();
        let value = attribute
            .unescape_value()
            .map_err(|error| ParseError::MalformedXml(error.to_string()))?
            .into_owned();

        match name.as_str() {
            "ivorn" => ivoid = Some(value.clone()),
            "version" => version = Some(value.clone()),
            "role" => raw_role = Some(value.clone()),
            _ => {}
        }

        root_attributes.push(RootAttribute { name, value });
    }

    let ivoid = ivoid.ok_or(ParseError::MissingAttribute("ivorn"))?;
    let version = version.ok_or(ParseError::MissingAttribute("version"))?;

    validate_ivoid(&ivoid).map_err(ParseError::InvalidIvoid)?;

    if version != VOEVENT_VERSION {
        return Err(ParseError::UnsupportedVersion(version));
    }

    let effective_role = match raw_role.as_deref() {
        Some(role) => PacketRole::parse(role)?,
        None => PacketRole::Observation,
    };
    let role_was_implicit = raw_role.is_none();

    Ok(VoEventHeader {
        ivoid,
        version,
        raw_role,
        effective_role,
        role_was_implicit,
        root_attributes,
        packet_byte_length: 0,
        packet_sha256: String::new(),
    })
}

/// Parse the VOEvent 2.1 root header and bind it to the exact packet bytes.
///
/// This is not a full XSD validator. It establishes root identity, version,
/// role semantics, XML well-formedness and byte integrity. A valid header is
/// not source authentication, calibration, astrophysical truth or authority.
pub fn parse_header(packet: &[u8]) -> Result<VoEventHeader, ParseError> {
    if packet.is_empty() {
        return Err(ParseError::EmptyPacket);
    }
    if packet.len() > MAX_PACKET_BYTES {
        return Err(ParseError::PacketTooLarge);
    }
    std::str::from_utf8(packet).map_err(|_| ParseError::UnsupportedEncoding)?;

    let mut reader = Reader::from_reader(Cursor::new(packet));
    reader.config_mut().trim_text(true);
    let mut buffer = Vec::new();
    let mut element_stack: Vec<Vec<u8>> = Vec::new();
    let mut root_seen = false;
    let mut root_closed = false;
    let mut header: Option<VoEventHeader> = None;

    loop {
        let event = reader
            .read_event_into(&mut buffer)
            .map_err(|error| ParseError::MalformedXml(error.to_string()))?;

        match event {
            Event::Start(element) => {
                if element_stack.is_empty() {
                    if root_seen {
                        return Err(ParseError::MultipleRootElements);
                    }
                    header = Some(parse_root_attributes(&element)?);
                    root_seen = true;
                }
                element_stack.push(element.name().as_ref().to_vec());
            }
            Event::Empty(element) => {
                if element_stack.is_empty() {
                    if root_seen {
                        return Err(ParseError::MultipleRootElements);
                    }
                    header = Some(parse_root_attributes(&element)?);
                    root_seen = true;
                    root_closed = true;
                }
            }
            Event::End(element) => {
                let Some(opening_name) = element_stack.pop() else {
                    return Err(ParseError::MalformedXml(
                        "closing element without an open element".to_owned(),
                    ));
                };
                let closing_name = element.name();
                if opening_name.as_slice() != closing_name.as_ref() {
                    return Err(ParseError::MismatchedElement {
                        expected: String::from_utf8_lossy(&opening_name).into_owned(),
                        found: String::from_utf8_lossy(closing_name.as_ref()).into_owned(),
                    });
                }
                if element_stack.is_empty() {
                    root_closed = true;
                }
            }
            Event::Text(text) if element_stack.is_empty() => {
                let decoded = text
                    .decode()
                    .map_err(|error| ParseError::MalformedXml(error.to_string()))?;
                if !decoded.trim().is_empty() {
                    return Err(ParseError::TextOutsideRoot);
                }
            }
            Event::CData(text) if element_stack.is_empty() => {
                let decoded = text
                    .decode()
                    .map_err(|error| ParseError::MalformedXml(error.to_string()))?;
                if !decoded.trim().is_empty() {
                    return Err(ParseError::TextOutsideRoot);
                }
            }
            Event::DocType(_) => return Err(ParseError::DoctypeNotAllowed),
            Event::GeneralRef(_) if element_stack.is_empty() => {
                return Err(ParseError::MalformedXml(
                    "entity reference outside root".to_owned(),
                ));
            }
            Event::Eof => break,
            _ => {}
        }

        buffer.clear();
    }

    if !root_seen || !root_closed || !element_stack.is_empty() {
        return Err(ParseError::IncompleteDocument);
    }

    let mut header = header.ok_or(ParseError::InvalidRoot)?;
    header.packet_byte_length = packet.len();
    header.packet_sha256 = hex::encode(Sha256::digest(packet));
    Ok(header)
}


#[derive(Debug)]
struct CitationTextBuilder {
    depth: usize,
    text: String,
}

#[derive(Debug)]
struct CitationBuilder {
    depth: usize,
    relation: CitationRelation,
    attributes: Vec<RootAttribute>,
    text: String,
}

fn parse_citation_attributes(
    element: &BytesStart<'_>,
) -> Result<(CitationRelation, Vec<RootAttribute>), ParseError> {
    let mut attributes = Vec::new();
    let mut cite: Option<String> = None;

    for attribute in element.attributes().with_checks(true) {
        let attribute =
            attribute.map_err(|error| ParseError::MalformedXml(error.to_string()))?;
        let name = std::str::from_utf8(attribute.key.as_ref())
            .map_err(|_| ParseError::MalformedXml("non-UTF-8 citation attribute name".to_owned()))?
            .to_owned();
        let value = attribute
            .unescape_value()
            .map_err(|error| ParseError::MalformedXml(error.to_string()))?
            .into_owned();

        if name == "cite" {
            cite = Some(value.clone());
        }
        attributes.push(RootAttribute { name, value });
    }

    let raw_cite = cite.ok_or(ParseError::MissingCitationAttribute)?;
    Ok((CitationRelation::parse(&raw_cite)?, attributes))
}

fn is_legal_xml_character(value: u32) -> bool {
    matches!(value, 0x9 | 0xA | 0xD)
        || (0x20..=0xD7FF).contains(&value)
        || (0xE000..=0xFFFD).contains(&value)
        || (0x10000..=0x10FFFF).contains(&value)
}

fn decode_citation_reference(reference: &[u8]) -> Result<char, ParseError> {
    let raw = std::str::from_utf8(reference)
        .map_err(|_| ParseError::InvalidCitationCharacterReference("non-UTF-8 reference".to_owned()))?;

    let character = match raw {
        "amp" => '&',
        "lt" => '<',
        "gt" => '>',
        "apos" => '\'',
        "quot" => '"',
        _ if raw.starts_with("#x") || raw.starts_with("#X") => {
            let value = u32::from_str_radix(&raw[2..], 16).map_err(|_| {
                ParseError::InvalidCitationCharacterReference(raw.to_owned())
            })?;
            char::from_u32(value).ok_or_else(|| {
                ParseError::InvalidCitationCharacterReference(raw.to_owned())
            })?
        }
        _ if raw.starts_with('#') => {
            let value = raw[1..].parse::<u32>().map_err(|_| {
                ParseError::InvalidCitationCharacterReference(raw.to_owned())
            })?;
            char::from_u32(value).ok_or_else(|| {
                ParseError::InvalidCitationCharacterReference(raw.to_owned())
            })?
        }
        _ => {
            return Err(ParseError::InvalidCitationCharacterReference(
                raw.to_owned(),
            ));
        }
    };

    if !is_legal_xml_character(character as u32) {
        return Err(ParseError::InvalidCitationCharacterReference(
            raw.to_owned(),
        ));
    }
    Ok(character)
}

/// Extract the typed Citations/EventIVORN relationship set from an already
/// well-formed, bounded packet. Only direct children of the root Citations
/// element are interpreted; the exact original bytes remain hash-bound by the
/// accompanying header.
fn parse_citations(
    packet: &[u8],
) -> Result<(bool, Vec<VoEventCitation>, Option<String>), ParseError> {
    let mut reader = Reader::from_reader(Cursor::new(packet));
    // Preserve text chunks here; citation targets are trimmed only after all
    // plain-text, CDATA and predefined/numeric character-reference segments
    // have been assembled.
    let mut buffer = Vec::new();
    let mut element_stack: Vec<Vec<u8>> = Vec::new();
    let mut citations_seen = false;
    let mut citations_depth: Option<usize> = None;
    let mut citations = Vec::new();
    let mut current_citation: Option<CitationBuilder> = None;
    let mut current_description: Option<CitationTextBuilder> = None;
    let mut description_seen = false;
    let mut description: Option<String> = None;

    loop {
        let event = reader
            .read_event_into(&mut buffer)
            .map_err(|error| ParseError::MalformedXml(error.to_string()))?;

        match event {
            Event::Start(element) => {
                let name = element.name().as_ref().to_vec();
                let depth = element_stack.len() + 1;

                if element_stack.len() == 1 && name.as_slice() == b"Citations" {
                    if citations_seen {
                        return Err(ParseError::DuplicateCitations);
                    }
                    citations_seen = true;
                    citations_depth = Some(depth);
                } else if citations_depth == Some(element_stack.len())
                    && name.as_slice() == b"EventIVORN"
                {
                    let (relation, attributes) = parse_citation_attributes(&element)?;
                    current_citation = Some(CitationBuilder {
                        depth,
                        relation,
                        attributes,
                        text: String::new(),
                    });
                } else if citations_depth == Some(element_stack.len())
                    && name.as_slice() == b"Description"
                {
                    if description_seen {
                        return Err(ParseError::DuplicateCitationsDescription);
                    }
                    description_seen = true;
                    current_description = Some(CitationTextBuilder {
                        depth,
                        text: String::new(),
                    });
                } else if current_citation
                    .as_ref()
                    .is_some_and(|builder| element_stack.len() >= builder.depth)
                {
                    return Err(ParseError::NestedCitationElement(
                        String::from_utf8_lossy(&name).into_owned(),
                    ));
                } else if current_description
                    .as_ref()
                    .is_some_and(|builder| element_stack.len() >= builder.depth)
                {
                    return Err(ParseError::NestedCitationElement(
                        String::from_utf8_lossy(&name).into_owned(),
                    ));
                }

                element_stack.push(name);
            }
            Event::Empty(element) => {
                let name = element.name().as_ref().to_vec();

                // Empty nested elements are markup too. Reject them inside
                // EventIVORN/Description just as we reject non-empty children.
                if current_citation
                    .as_ref()
                    .is_some_and(|builder| element_stack.len() >= builder.depth)
                    || current_description
                        .as_ref()
                        .is_some_and(|builder| element_stack.len() >= builder.depth)
                {
                    return Err(ParseError::NestedCitationElement(
                        String::from_utf8_lossy(&name).into_owned(),
                    ));
                }

                if element_stack.len() == 1 && name.as_slice() == b"Citations" {
                    if citations_seen {
                        return Err(ParseError::DuplicateCitations);
                    }
                    return Err(ParseError::EmptyCitations);
                }

                if citations_depth == Some(element_stack.len())
                    && name.as_slice() == b"EventIVORN"
                {
                    let (relation, attributes) = parse_citation_attributes(&element)?;
                    return Err(ParseError::InvalidCitationTarget(format!(
                        "empty EventIVORN with relation {relation:?} and {} attributes",
                        attributes.len()
                    )));
                }

                if citations_depth == Some(element_stack.len())
                    && name.as_slice() == b"Description"
                {
                    if description_seen {
                        return Err(ParseError::DuplicateCitationsDescription);
                    }
                    description_seen = true;
                    description = Some(String::new());
                }
            }
            Event::Text(text) => {
                let decoded = text
                    .decode()
                    .map_err(|error| ParseError::MalformedXml(error.to_string()))?;
                if let Some(builder) = current_citation.as_mut() {
                    if element_stack.len() == builder.depth {
                        builder.text.push_str(&decoded);
                    }
                } else if let Some(builder) = current_description.as_mut() {
                    if element_stack.len() == builder.depth {
                        builder.text.push_str(&decoded);
                    }
                }
            }
            Event::CData(text) => {
                let decoded = text
                    .decode()
                    .map_err(|error| ParseError::MalformedXml(error.to_string()))?;
                if let Some(builder) = current_citation.as_mut() {
                    if element_stack.len() == builder.depth {
                        builder.text.push_str(&decoded);
                    }
                } else if let Some(builder) = current_description.as_mut() {
                    if element_stack.len() == builder.depth {
                        builder.text.push_str(&decoded);
                    }
                }
            }
            Event::GeneralRef(reference) => {
                let target = if let Some(builder) = current_citation.as_mut() {
                    if element_stack.len() == builder.depth {
                        Some(&mut builder.text)
                    } else {
                        None
                    }
                } else if let Some(builder) = current_description.as_mut() {
                    if element_stack.len() == builder.depth {
                        Some(&mut builder.text)
                    } else {
                        None
                    }
                } else {
                    None
                };

                if let Some(target) = target {
                    target.push(decode_citation_reference(reference.as_ref())?);
                }
            }
            Event::End(element) => {
                let depth = element_stack.len();
                let name = element.name().as_ref();

                if let Some(builder) = current_citation.as_ref() {
                    if builder.depth == depth && name == b"EventIVORN" {
                        let builder = current_citation.take().expect("citation builder exists");
                        let target_ivoid = builder.text.trim().to_owned();
                        validate_ivoid(&target_ivoid)
                            .map_err(ParseError::InvalidCitationTarget)?;
                        citations.push(VoEventCitation {
                            target_ivoid,
                            relation: builder.relation,
                            attributes: builder.attributes,
                        });
                    }
                }

                if let Some(builder) = current_description.as_ref() {
                    if builder.depth == depth && name == b"Description" {
                        let builder = current_description
                            .take()
                            .expect("description builder exists");
                        description = Some(builder.text.trim().to_owned());
                    }
                }

                if citations_depth == Some(depth) && name == b"Citations" {
                    if citations.is_empty() {
                        return Err(ParseError::EmptyCitations);
                    }
                    citations_depth = None;
                }

                element_stack.pop();
            }
            Event::DocType(_) => return Err(ParseError::DoctypeNotAllowed),
            Event::Eof => break,
            _ => {}
        }

        buffer.clear();
    }

    Ok((citations_seen, citations, description))
}

/// Parse the root header and the typed lifecycle citations from one exact packet.
///
/// This remains a bounded extraction profile, not full VOEvent XSD validation.
/// The byte length and digest bind all extracted values to the same packet.
pub fn parse_packet(packet: &[u8]) -> Result<VoEventPacket, ParseError> {
    let header = parse_header(packet)?;
    let (citations_present, citations, citations_description) = parse_citations(packet)?;
    Ok(VoEventPacket {
        header,
        citations_present,
        citations,
        citations_description,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(root_attributes: &str) -> Vec<u8> {
        format!(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<VOEvent {root_attributes}><Who><AuthorIVORN>ivo://example.org/author</AuthorIVORN></Who><What/></VOEvent>"
        )
        .into_bytes()
    }

    fn packet_with_body(body: &str) -> Vec<u8> {
        format!(
            "<VOEvent ivorn=\"ivo://example.org/stream?packet-1\" version=\"2.1\">{body}</VOEvent>"
        )
        .into_bytes()
    }

    #[test]
    fn preserves_explicit_observation_role_and_source_identity() {
        let bytes = packet(
            "ivorn=\"ivo://example.org/transient?event-001\" version=\"2.1\" role=\"observation\"",
        );
        let header = parse_header(&bytes).unwrap();

        assert_eq!(header.ivoid, "ivo://example.org/transient?event-001");
        assert_eq!(header.version, "2.1");
        assert_eq!(header.raw_role.as_deref(), Some("observation"));
        assert_eq!(header.effective_role, PacketRole::Observation);
        assert!(!header.role_was_implicit);
        assert_eq!(header.packet_byte_length, bytes.len());
        assert_eq!(header.packet_sha256.len(), 64);
    }

    #[test]
    fn missing_role_uses_standard_default_without_erasing_omission() {
        let bytes = packet("ivorn=\"ivo://example.org/stream?event-002\" version=\"2.1\"");
        let header = parse_header(&bytes).unwrap();

        assert_eq!(header.raw_role, None);
        assert_eq!(header.effective_role, PacketRole::Observation);
        assert!(header.role_was_implicit);
    }

    #[test]
    fn preserves_non_observation_packet_roles() {
        for (role, expected) in [
            ("prediction", PacketRole::Prediction),
            ("utility", PacketRole::Utility),
            ("test", PacketRole::Test),
        ] {
            let bytes = packet(&format!(
                "ivorn=\"ivo://example.org/stream?{role}\" version=\"2.1\" role=\"{role}\""
            ));
            let header = parse_header(&bytes).unwrap();
            assert_eq!(header.raw_role.as_deref(), Some(role));
            assert_eq!(header.effective_role, expected);
        }
    }

    #[test]
    fn rejects_unknown_roles() {
        let bytes =
            packet("ivorn=\"ivo://example.org/stream?bad\" version=\"2.1\" role=\"candidate\"");
        assert!(matches!(
            parse_header(&bytes),
            Err(ParseError::InvalidRole(_))
        ));
    }

    #[test]
    fn rejects_wrong_or_missing_version() {
        let wrong = packet("ivorn=\"ivo://example.org/stream?1\" version=\"2.0\"");
        assert!(matches!(
            parse_header(&wrong),
            Err(ParseError::UnsupportedVersion(_))
        ));

        let missing = packet("ivorn=\"ivo://example.org/stream?1\"");
        assert_eq!(
            parse_header(&missing),
            Err(ParseError::MissingAttribute("version"))
        );
    }

    #[test]
    fn rejects_missing_or_non_ivoid_identity() {
        let missing = packet("version=\"2.1\"");
        assert_eq!(
            parse_header(&missing),
            Err(ParseError::MissingAttribute("ivorn"))
        );

        let invalid = packet("ivorn=\"https://example.org/event/1\" version=\"2.1\"");
        assert!(matches!(
            parse_header(&invalid),
            Err(ParseError::InvalidIvoid(_))
        ));
    }

    #[test]
    fn rejects_doctype_and_duplicate_root_attributes() {
        let doctype = br#"<!DOCTYPE VOEvent [<!ENTITY x "y">]><VOEvent ivorn="ivo://example.org/a" version="2.1"/>"#;
        assert_eq!(parse_header(doctype), Err(ParseError::DoctypeNotAllowed));

        let duplicate =
            br#"<VOEvent ivorn="ivo://example.org/a" ivorn="ivo://example.org/b" version="2.1"/>"#;
        assert!(matches!(
            parse_header(duplicate),
            Err(ParseError::MalformedXml(_))
        ));
    }

    #[test]
    fn rejects_multiple_roots_and_incomplete_xml() {
        let multiple = br#"<VOEvent ivorn="ivo://example.org/a" version="2.1"/><VOEvent ivorn="ivo://example.org/b" version="2.1"/>"#;
        assert_eq!(
            parse_header(multiple),
            Err(ParseError::MultipleRootElements)
        );

        let incomplete = br#"<VOEvent ivorn="ivo://example.org/a" version="2.1"><Who>"#;
        assert!(matches!(
            parse_header(incomplete),
            Err(ParseError::IncompleteDocument | ParseError::MalformedXml(_))
        ));
    }

    #[test]
    fn rejects_mismatched_nested_element_names() {
        let mismatched =
            br#"<VOEvent ivorn="ivo://example.org/a" version="2.1"><Who></What></VOEvent>"#;
        assert!(matches!(
            parse_header(mismatched),
            Err(ParseError::MismatchedElement { .. } | ParseError::MalformedXml(_))
        ));
    }

    #[test]
    fn rejects_non_whitespace_outside_document_root() {
        let bytes = br#"stray<VOEvent ivorn="ivo://example.org/a" version="2.1"/>"#;
        assert_eq!(parse_header(bytes), Err(ParseError::TextOutsideRoot));
    }

    #[test]
    fn rejects_oversized_and_non_utf8_packets() {
        let oversized = vec![b' '; MAX_PACKET_BYTES + 1];
        assert_eq!(parse_header(&oversized), Err(ParseError::PacketTooLarge));

        assert_eq!(
            parse_header(&[0xff, 0xfe, 0x00, 0x00]),
            Err(ParseError::UnsupportedEncoding)
        );
    }

    #[test]
    fn packet_without_citations_preserves_new_event_semantics() {
        let bytes = packet_with_body("<Who/><What/>");
        let parsed = parse_packet(&bytes).expect("packet");
        assert!(!parsed.citations_present);
        assert!(parsed.citations.is_empty());
        assert_eq!(parsed.citations_description, None);
        assert_eq!(parsed.header.packet_byte_length, bytes.len());
    }

    #[test]
    fn extracts_multiple_typed_citations_and_preserves_attributes_and_description() {
        let bytes = packet_with_body(
            r#"<Citations>
                <EventIVORN cite="followup">ivo://survey.example/event?first</EventIVORN>
                <EventIVORN cite="supersedes" publisher-note="reprocessed">ivo://survey.example/event?second&amp;revised</EventIVORN>
                <Description> Reprocessed &amp; corrected after independent review. </Description>
            </Citations>"#,
        );
        let parsed = parse_packet(&bytes).expect("packet with citations");

        assert!(parsed.citations_present);
        assert_eq!(parsed.citations.len(), 2);
        assert_eq!(parsed.citations[0].relation, CitationRelation::Followup);
        assert_eq!(
            parsed.citations[0].target_ivoid,
            "ivo://survey.example/event?first"
        );
        assert_eq!(parsed.citations[1].relation, CitationRelation::Supersedes);
        assert_eq!(
            parsed.citations[1].target_ivoid,
            "ivo://survey.example/event?second&revised"
        );
        assert!(parsed.citations[1].attributes.iter().any(|attribute| {
            attribute.name == "publisher-note" && attribute.value == "reprocessed"
        }));
        assert_eq!(
            parsed.citations_description.as_deref(),
            Some("Reprocessed & corrected after independent review.")
        );
        assert_eq!(parsed.header.packet_sha256.len(), 64);
    }

    #[test]
    fn decodes_numeric_references_in_event_ivoid_text() {
        let bytes = packet_with_body(
            r#"<Citations><EventIVORN cite="followup">ivo://survey.example/event&#x2D;42</EventIVORN></Citations>"#,
        );
        let parsed = parse_packet(&bytes).expect("numeric character reference");
        assert_eq!(
            parsed.citations[0].target_ivoid,
            "ivo://survey.example/event-42"
        );
    }

    #[test]
    fn rejects_event_ivoid_without_required_cite_attribute() {
        let bytes = packet_with_body(
            r#"<Citations><EventIVORN>ivo://survey.example/event-1</EventIVORN></Citations>"#,
        );
        assert_eq!(
            parse_packet(&bytes),
            Err(ParseError::MissingCitationAttribute)
        );
    }

    #[test]
    fn rejects_unknown_citation_relations() {
        let bytes = packet_with_body(
            r#"<Citations><EventIVORN cite="associated">ivo://survey.example/event-1</EventIVORN></Citations>"#,
        );
        assert!(matches!(
            parse_packet(&bytes),
            Err(ParseError::InvalidCitationRelation(_))
        ));
    }

    #[test]
    fn rejects_non_ivoid_citation_targets_and_empty_event_ivoorn() {
        let invalid = packet_with_body(
            r#"<Citations><EventIVORN cite="followup">https://example.org/event-1</EventIVORN></Citations>"#,
        );
        assert!(matches!(
            parse_packet(&invalid),
            Err(ParseError::InvalidCitationTarget(_))
        ));

        let empty = packet_with_body(
            r#"<Citations><EventIVORN cite="retraction"/></Citations>"#,
        );
        assert!(matches!(
            parse_packet(&empty),
            Err(ParseError::InvalidCitationTarget(_))
        ));
    }

    #[test]
    fn rejects_empty_duplicate_and_nested_citation_structures() {
        let empty = packet_with_body("<Citations/>");
        assert_eq!(parse_packet(&empty), Err(ParseError::EmptyCitations));

        let duplicate = packet_with_body(
            r#"<Citations><EventIVORN cite="followup">ivo://survey.example/a</EventIVORN></Citations><Citations><EventIVORN cite="followup">ivo://survey.example/b</EventIVORN></Citations>"#,
        );
        assert_eq!(parse_packet(&duplicate), Err(ParseError::DuplicateCitations));

        let nested = packet_with_body(
            r#"<Citations><EventIVORN cite="followup"><Name>ivo://survey.example/a</Name></EventIVORN></Citations>"#,
        );
        assert!(matches!(
            parse_packet(&nested),
            Err(ParseError::NestedCitationElement(_))
        ));

        let nested_empty = packet_with_body(
            r#"<Citations><EventIVORN cite="followup"><Name/></EventIVORN></Citations>"#,
        );
        assert!(matches!(
            parse_packet(&nested_empty),
            Err(ParseError::NestedCitationElement(_))
        ));
    }

    #[test]
    fn rejects_duplicate_citation_descriptions_and_external_entities() {
        let duplicate = packet_with_body(
            r#"<Citations><EventIVORN cite="followup">ivo://survey.example/a</EventIVORN><Description>one</Description><Description>two</Description></Citations>"#,
        );
        assert_eq!(
            parse_packet(&duplicate),
            Err(ParseError::DuplicateCitationsDescription)
        );

        let unresolved = packet_with_body(
            r#"<Citations><EventIVORN cite="followup">ivo://survey.example/a&custom;</EventIVORN></Citations>"#,
        );
        assert!(matches!(
            parse_packet(&unresolved),
            Err(ParseError::MalformedXml(_) | ParseError::InvalidCitationCharacterReference(_))
        ));
    }

    #[test]
    fn retains_uninterpreted_root_attributes_for_loss_accounting() {
        let bytes =
            packet("ivorn=\"ivo://example.org/stream?3\" version=\"2.1\" custom=\"preserve-me\"");
        let header = parse_header(&bytes).unwrap();
        assert!(
            header
                .root_attributes
                .iter()
                .any(|attribute| attribute.name == "custom" && attribute.value == "preserve-me")
        );
    }
}

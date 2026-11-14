// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bounded VOEvent 2.1 root-header extraction.
//!
//! This crate deliberately does not claim full VOEvent XSD validation. It
//! validates the root packet identity/version/role, preserves all root
//! attributes and hashes the exact packet bytes. Domain fields, STC semantics,
//! citations, publisher authentication, rights and scientific validity remain
//! separate qualification steps.

use quick_xml::events::{BytesStart, Event};
use quick_xml::Reader;
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
        let attribute =
            attribute.map_err(|error| ParseError::MalformedXml(error.to_string()))?;
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

    // Conservative lexical check only; this does not resolve the IVOID in a registry.
    if !ivoid.starts_with("ivo://")
        || ivoid.len() <= "ivo://".len()
        || ivoid
            .chars()
            .any(|character| character.is_control() || character.is_whitespace())
    {
        return Err(ParseError::InvalidIvoid(ivoid));
    }

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

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(root_attributes: &str) -> Vec<u8> {
        format!(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<VOEvent {root_attributes}><Who><AuthorIVORN>ivo://example.org/author</AuthorIVORN></Who><What/></VOEvent>"
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
        let bytes = packet(
            "ivorn=\"ivo://example.org/stream?bad\" version=\"2.1\" role=\"candidate\"",
        );
        assert!(matches!(parse_header(&bytes), Err(ParseError::InvalidRole(_))));
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

        let duplicate = br#"<VOEvent ivorn="ivo://example.org/a" ivorn="ivo://example.org/b" version="2.1"/>"#;
        assert!(matches!(parse_header(duplicate), Err(ParseError::MalformedXml(_))));
    }

    #[test]
    fn rejects_multiple_roots_and_incomplete_xml() {
        let multiple = br#"<VOEvent ivorn="ivo://example.org/a" version="2.1"/><VOEvent ivorn="ivo://example.org/b" version="2.1"/>"#;
        assert_eq!(parse_header(multiple), Err(ParseError::MultipleRootElements));

        let incomplete = br#"<VOEvent ivorn="ivo://example.org/a" version="2.1"><Who>"#;
        assert!(matches!(
            parse_header(incomplete),
            Err(ParseError::IncompleteDocument | ParseError::MalformedXml(_))
        ));
    }

    #[test]
    fn rejects_mismatched_nested_element_names() {
        let mismatched = br#"<VOEvent ivorn="ivo://example.org/a" version="2.1"><Who></What></VOEvent>"#;
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
    fn retains_uninterpreted_root_attributes_for_loss_accounting() {
        let bytes = packet(
            "ivorn=\"ivo://example.org/stream?3\" version=\"2.1\" custom=\"preserve-me\"",
        );
        let header = parse_header(&bytes).unwrap();
        assert!(header
            .root_attributes
            .iter()
            .any(|attribute| attribute.name == "custom" && attribute.value == "preserve-me"));
    }
}

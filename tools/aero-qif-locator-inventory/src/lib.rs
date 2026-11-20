//! Independent, profile-bounded QIF trace-locator inventory from raw XML bytes.
//!
//! This crate does not run XSD/XSLT validation, dereference URIs, resolve external
//! entities, parse AP242, or consume adapter mapping rows. It scans immutable QIF
//! source bytes with a namespace-aware streaming parser and emits the expected
//! locator set for a separately versioned Test 7/8 trace profile.

use aero_qif_ingest_qualification::{
    QifDocumentRole, SourceFieldLocatorV1,
};
use quick_xml::{
    escape::{resolve_xml_entity, unescape},
    events::Event,
    name::ResolveResult,
    reader::NsReader,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    io::Cursor,
};
use thiserror::Error;

pub const INVENTORY_SCHEMA: &str = "AeroQifTraceLocatorInventoryV1";
pub const PROFILE_SCHEMA: &str = "AeroQifTraceLocatorProfileV1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LocatorCaptureMode {
    Element,
    ElementId,
    Text,
    QifReference,
    DocumentQpid,
    ExternalPlanQpid,
    ExternalPlanUri,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceLocatorRuleV1 {
    pub rule_id: String,
    pub document_role: QifDocumentRole,
    /// Prefix-independent Clark names, including the QIFDocument root.
    pub expanded_path: Vec<String>,
    pub capture: LocatorCaptureMode,
    /// Expanded attribute names captured as inert source metadata, e.g. "{}QPId" and "{}URI".
    pub capture_attributes: Vec<String>,
    /// This rule captures the Results document's ExternalQIFDocumentReference/@id.
    pub declares_external_document_reference: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceLocatorLimitsV1 {
    pub max_source_bytes: u64,
    pub max_elements: u64,
    pub max_depth: u32,
    pub max_attributes_per_element: u32,
    pub max_text_bytes_per_element: u64,
    pub max_total_text_bytes: u64,
    pub max_locators: u64,
    pub max_namespace_bindings: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceLocatorProfileV1 {
    pub schema: String,
    pub profile_id: String,
    pub profile_version: String,
    pub qif_standard_version: String,
    pub rules: Vec<TraceLocatorRuleV1>,
    pub limits: TraceLocatorLimitsV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceLocatorRecordV1 {
    pub rule_id: String,
    pub capture: LocatorCaptureMode,
    pub locator: SourceFieldLocatorV1,
    /// SHA-256 of simple-content bytes after XML entity decoding; raw values are not copied.
    pub field_value_sha256: Option<String>,
    /// Decoded source value for explicitly selected identity/URI fields only.
    pub captured_value: Option<String>,
    /// Selected expanded-name attributes retained as source metadata; never dereferenced.
    pub captured_attributes: BTreeMap<String, String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceLocatorSourceDocumentV1 {
    pub qpid: String,
    pub role: QifDocumentRole,
    pub sha256: String,
    pub byte_length: u64,
    pub root_expanded_name: String,
    pub element_count: u64,
    pub max_observed_depth: u32,
    pub text_bytes: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceLocatorInventoryBodyV1 {
    pub inventory_id: String,
    pub profile_id: String,
    pub profile_version: String,
    pub profile_sha256: String,
    pub qif_standard_version: String,
    pub documents: Vec<TraceLocatorSourceDocumentV1>,
    pub locators: Vec<TraceLocatorRecordV1>,
    pub declared_external_document_reference_ids: Vec<u32>,
    pub external_document_references: Vec<ExternalQifDocumentReferenceV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExternalQifDocumentReferenceV1 {
    pub results_qpid: String,
    pub local_reference_id: u32,
    pub plan_qpid: String,
    pub source_uri: Option<String>,
    pub source_locator: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceLocatorInventoryV1 {
    pub schema: String,
    pub body: TraceLocatorInventoryBodyV1,
    pub inventory_sha256: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InventoryDisposition {
    Complete,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TraceLocatorInventoryReport {
    pub disposition: InventoryDisposition,
    pub profile_sha256: String,
    pub inventory_sha256: String,
    pub source_document_count: usize,
    pub locator_count: usize,
    pub declared_external_reference_count: usize,
    pub elements_scanned: u64,
    pub text_bytes_scanned: u64,
    pub max_observed_depth: u32,
}

pub struct RawQifDocument<'a> {
    pub qpid: &'a str,
    pub role: QifDocumentRole,
    pub bytes: &'a [u8],
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum InventoryError {
    #[error("unsupported inventory/profile schema")]
    WrongSchema,
    #[error("empty profile or inventory identity at {0}")]
    InvalidIdentity(String),
    #[error("invalid QIF document QPId {0}")]
    InvalidQpid(String),
    #[error("profile has no rules")]
    EmptyProfile,
    #[error("duplicate trace rule {0}")]
    DuplicateRule(String),
    #[error("invalid trace rule {0}")]
    InvalidRule(String),
    #[error("duplicate selector for {0}")]
    DuplicateSelector(String),
    #[error("source document exceeds maximum bytes")]
    SourceTooLarge,
    #[error("source document is not UTF-8 under the declared profile")]
    UnsupportedEncoding,
    #[error("XML declaration specifies an unsupported encoding")]
    UnsupportedDeclaredEncoding,
    #[error("multiple XML root elements")]
    MultipleRoots,
    #[error("document root is not QIFDocument")]
    WrongRoot,
    #[error("unbound XML namespace prefix {0}")]
    UnboundNamespace(String),
    #[error("XML parse failed at byte {position}: {reason}")]
    XmlParse { position: u64, reason: String },
    #[error("DTD/DOCTYPE declarations are forbidden in the offline locator profile")]
    DoctypeForbidden,
    #[error("custom XML entity {0} is not admitted in the offline locator profile")]
    CustomEntityForbidden(String),
    #[error("invalid XML character reference {0}")]
    InvalidCharacterReference(String),
    #[error("duplicate or malformed XML attribute")]
    InvalidAttribute,
    #[error("element count exceeds profile limit")]
    ElementLimitExceeded,
    #[error("XML nesting depth exceeds profile limit")]
    DepthLimitExceeded,
    #[error("attributes per element exceed profile limit")]
    AttributeLimitExceeded,
    #[error("simple content exceeds per-element text limit")]
    ElementTextLimitExceeded,
    #[error("document text exceeds profile limit")]
    TotalTextLimitExceeded,
    #[error("trace locator count exceeds profile limit")]
    LocatorLimitExceeded,
    #[error("malformed or mismatched XML element stack")]
    ElementStackMismatch,
    #[error("document has no root element or did not close completely")]
    IncompleteDocument,
    #[error("trace-reference field has missing or invalid numeric content at {0}")]
    InvalidQifReference(String),
    #[error("QIF element identity is missing or not a positive integer at {0}")]
    InvalidElementId(String),
    #[error("QIF reference at {0} points to an undeclared Results external-document ID {1}")]
    UndeclaredExternalReference(String, u32),
    #[error("external Plan QPId is missing or invalid at {0}")]
    InvalidExternalPlanQpid(String),
    #[error("source document QPId element does not match the supplied document identity: {0}")]
    DocumentQpidMismatch(String),
    #[error("external Plan QPId at {0} does not match any supplied Plan document")]
    UnknownExternalPlanQpid(String),
    #[error("duplicate Results external-document reference ID {0}")]
    DuplicateExternalReferenceId(u32),
    #[error("duplicate Results external-document reference id {0}")]
    DuplicateExternalReferenceId(u32),
    #[error("inventory canonicalization failed")]
    CanonicalizationFailed,
    #[error("inventory digest is malformed or does not match its body")]
    InventoryDigestMismatch,
    #[error("profile digest is malformed or does not match the profile")]
    ProfileDigestMismatch,
}

#[derive(Debug)]
struct ElementFrame {
    expanded_name: String,
    expanded_path: Vec<String>,
    locator_path: String,
    child_occurrences: BTreeMap<String, u64>,
    matching_rules: Vec<usize>,
    element_id: Option<u32>,
    external_xid: Option<u32>,
    external_document_reference_id: Option<u32>,
    is_external_document_reference: bool,
    captured_attributes: BTreeMap<String, String>,
    direct_text: String,
    has_child_element: bool,
}

#[derive(Debug, Clone)]
struct ScannedDocument {
    source: TraceLocatorSourceDocumentV1,
    locators: Vec<TraceLocatorRecordV1>,
    declared_external_ids: BTreeSet<u32>,
}

fn valid_qpid(value: &str) -> bool {
    if value.len() != 36 {
        return false;
    }
    value.bytes().enumerate().all(|(index, byte)| {
        if matches!(index, 8 | 13 | 18 | 23) {
            byte == b'-'
        } else {
            byte.is_ascii_hexdigit()
        }
    })
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value.bytes().all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn locator_sort_key(locator: &SourceFieldLocatorV1) -> (
    String,
    String,
    Option<u32>,
    Option<u32>,
    Option<u32>,
) {
    (
        locator.document_qpid.to_ascii_lowercase(),
        locator.source_path.clone(),
        locator.source_element_id,
        locator.external_document_reference_id,
        locator.external_xid,
    )
}

fn canonical_profile(profile: &TraceLocatorProfileV1) -> TraceLocatorProfileV1 {
    let mut canonical = profile.clone();
    canonical.rules.sort_by(|left, right| left.rule_id.cmp(&right.rule_id));
    canonical
}

fn canonical_inventory_body(body: &TraceLocatorInventoryBodyV1) -> TraceLocatorInventoryBodyV1 {
    let mut canonical = body.clone();
    canonical.documents.sort_by(|left, right| {
        left.qpid.to_ascii_lowercase().cmp(&right.qpid.to_ascii_lowercase())
    });
    canonical.locators.sort_by(|left, right| {
        locator_sort_key(&left.locator).cmp(&locator_sort_key(&right.locator))
            .then(left.rule_id.cmp(&right.rule_id))
            .then(left.capture.cmp(&right.capture))
    });
    canonical.declared_external_document_reference_ids.sort_unstable();
    canonical
}

/// Versioned Rust-struct JSON digest; this is not RFC 8785/JCS.
pub fn compute_profile_digest(profile: &TraceLocatorProfileV1) -> Result<String, InventoryError> {
    let bytes = serde_json::to_vec(&canonical_profile(profile))
        .map_err(|_| InventoryError::CanonicalizationFailed)?;
    Ok(sha256_hex(&bytes))
}

pub fn compute_inventory_digest(
    inventory: &TraceLocatorInventoryV1,
) -> Result<String, InventoryError> {
    let bytes = serde_json::to_vec(&canonical_inventory_body(&inventory.body))
        .map_err(|_| InventoryError::CanonicalizationFailed)?;
    Ok(sha256_hex(&bytes))
}

pub fn validate_profile(profile: &TraceLocatorProfileV1) -> Result<(), InventoryError> {
    if profile.schema != PROFILE_SCHEMA {
        return Err(InventoryError::WrongSchema);
    }
    for (field, value) in [
        ("profile_id", profile.profile_id.as_str()),
        ("profile_version", profile.profile_version.as_str()),
        ("qif_standard_version", profile.qif_standard_version.as_str()),
    ] {
        if value.trim().is_empty() {
            return Err(InventoryError::InvalidIdentity(field.into()));
        }
    }
    if profile.rules.is_empty() {
        return Err(InventoryError::EmptyProfile);
    }
    let limits = &profile.limits;
    if limits.max_source_bytes == 0
        || limits.max_source_bytes > 100 * 1024 * 1024
        || limits.max_elements == 0
        || limits.max_elements > 2_000_000
        || limits.max_depth == 0
        || limits.max_depth > 128
        || limits.max_attributes_per_element == 0
        || limits.max_attributes_per_element > 256
        || limits.max_text_bytes_per_element == 0
        || limits.max_text_bytes_per_element > 10 * 1024 * 1024
        || limits.max_total_text_bytes == 0
        || limits.max_total_text_bytes > 100 * 1024 * 1024
        || limits.max_locators == 0
        || limits.max_locators > 2_000_000
        || limits.max_namespace_bindings == 0
        || limits.max_namespace_bindings > 4096
    {
        return Err(InventoryError::InvalidIdentity("limits".into()));
    }

    let mut rule_ids = BTreeSet::new();
    let mut selectors = BTreeSet::new();
    for rule in &profile.rules {
        if rule.rule_id.trim().is_empty()
            || rule.expanded_path.len() < 2
            || rule.expanded_path.first().is_none_or(|root| !root.ends_with("}QIFDocument"))
            || rule.expanded_path.iter().enumerate().any(|(index, name)| {
                !name.starts_with('{')
                    || !name.contains('}')
                    || (name.ends_with("}*") && name.len() <= 3)
                    || (name.ends_with("}*") && index == 0)
            })
        {
            return Err(InventoryError::InvalidRule(rule.rule_id.clone()));
        }
        if !rule_ids.insert(rule.rule_id.as_str()) {
            return Err(InventoryError::DuplicateRule(rule.rule_id.clone()));
        }
        let selector = (
            format!("{:?}", rule.document_role),
            rule.expanded_path.join("/"),
            rule.capture,
        );
        if !selectors.insert(selector) {
            return Err(InventoryError::DuplicateSelector(rule.rule_id.clone()));
        }
        if rule.capture_attributes.iter().any(|name| !name.starts_with("{}") || name.len() <= 2) {
            return Err(InventoryError::InvalidRule(rule.rule_id.clone()));
        }
        if rule.declares_external_document_reference
            && (rule.document_role != QifDocumentRole::Results
                || rule.capture != LocatorCaptureMode::ElementId)
        {
            return Err(InventoryError::InvalidRule(rule.rule_id.clone()));
        }
    }
    Ok(())
}

fn expanded_path_matches(pattern: &[String], actual: &[String]) -> bool {
    pattern.len() == actual.len()
        && pattern.iter().zip(actual).all(|(expected, observed)| {
            if expected.ends_with("}*") {
                observed.starts_with(expected.trim_end_matches('*'))
                    && observed.len() > expected.len() - 1
            } else {
                expected == observed
            }
        })
}

fn parse_utf8_profile(bytes: &[u8]) -> Result<&str, InventoryError> {
    let text = std::str::from_utf8(bytes).map_err(|_| InventoryError::UnsupportedEncoding)?;
    let prefix = text.trim_start_matches('\u{feff}').get(..text.trim_start_matches('\u{feff}').len().min(256)).unwrap_or("");
    if let Some(declaration_end) = prefix.find("?>") {
        let declaration = &prefix[..declaration_end].to_ascii_lowercase();
        if let Some(encoding_pos) = declaration.find("encoding") {
            let after = &declaration[encoding_pos + "encoding".len()..];
            let Some(eq_pos) = after.find('=') else {
                return Err(InventoryError::UnsupportedDeclaredEncoding);
            };
            let value = after[eq_pos + 1..].trim_start();
            let Some(quote) = value.chars().next() else {
                return Err(InventoryError::UnsupportedDeclaredEncoding);
            };
            if quote != '\'' && quote != '"' {
                return Err(InventoryError::UnsupportedDeclaredEncoding);
            }
            let Some(end) = value[1..].find(quote) else {
                return Err(InventoryError::UnsupportedDeclaredEncoding);
            };
            let encoding = &value[1..1 + end];
            if !matches!(encoding, "utf-8" | "utf8") {
                return Err(InventoryError::UnsupportedDeclaredEncoding);
            }
        }
    }
    Ok(text)
}

fn expanded_name(resolve: ResolveResult<'_>, local_name: &str) -> Result<String, InventoryError> {
    let namespace = match resolve {
        ResolveResult::Bound(namespace) => namespace.as_ref(),
        ResolveResult::Unbound => "",
        ResolveResult::Unknown(prefix) => return Err(InventoryError::UnboundNamespace(prefix)),
    };
    Ok(format!("{{{namespace}}}{local_name}"))
}

fn parsed_numeric_attribute(value: &str, path: &str) -> Result<u32, InventoryError> {
    let value = value.trim();
    let parsed = value.parse::<u32>().map_err(|_| InventoryError::InvalidElementId(path.into()))?;
    if parsed == 0 {
        return Err(InventoryError::InvalidElementId(path.into()));
    }
    Ok(parsed)
}

fn attribute_values(
    start: &quick_xml::events::BytesStart<'_>,
    resolver: &quick_xml::name::NamespaceResolver,
    max_attributes: u32,
    path: &str,
) -> Result<(Option<u32>, Option<u32>, BTreeMap<String, String>, u32), InventoryError> {
    let mut element_id = None;
    let mut external_xid = None;
    let mut attributes = BTreeMap::new();
    let mut count = 0u32;
    for attribute in start.attributes().with_checks(true) {
        let attribute = attribute.map_err(|_| InventoryError::InvalidAttribute)?;
        count += 1;
        if count > max_attributes {
            return Err(InventoryError::AttributeLimitExceeded);
        }
        let (namespace, local_name) = resolver.resolve_attribute(attribute.key);
        let expanded = expanded_name(namespace, local_name.as_ref())?;
        // Namespace declarations are parser control metadata, not QIF fields.
        if expanded.starts_with("{http://www.w3.org/2000/xmlns/}") {
            continue;
        }
        let value = unescape(attribute.value.as_ref())
            .map_err(|_| InventoryError::InvalidAttribute)?
            .into_owned();
        if expanded == "{}id" {
            element_id = Some(parsed_numeric_attribute(&value, path)?);
        } else if expanded == "{}xId" {
            external_xid = Some(parsed_numeric_attribute(&value, path)?);
        }
        attributes.insert(expanded, value);
    }
    Ok((element_id, external_xid, attributes, count))
}

fn append_locator(
    locators: &mut Vec<TraceLocatorRecordV1>,
    rule: &TraceLocatorRuleV1,
    qpid: &str,
    path: &str,
    element_id: Option<u32>,
    external_reference_id: Option<u32>,
    external_xid: Option<u32>,
    field_value: Option<&str>,
    captured_value: Option<&str>,
    captured_attributes: &BTreeMap<String, String>,
    max_locators: u64,
) -> Result<(), InventoryError> {
    if locators.len() as u64 >= max_locators {
        return Err(InventoryError::LocatorLimitExceeded);
    }
    let locator = SourceFieldLocatorV1 {
        document_qpid: qpid.to_ascii_lowercase(),
        source_path: path.to_owned(),
        source_element_id: element_id,
        external_document_reference_id: external_reference_id,
        external_xid,
    };
    let field_value_sha256 = field_value.map(|value| sha256_hex(value.as_bytes()));
    locators.push(TraceLocatorRecordV1 {
        rule_id: rule.rule_id.clone(),
        capture: rule.capture,
        locator,
        field_value_sha256,
        captured_value: captured_value.map(str::to_owned),
        captured_attributes: captured_attributes.clone(),
    });
    Ok(())
}

fn finalize_frame(
    frame: ElementFrame,
    profile: &TraceLocatorProfileV1,
    qpid: &str,
    locators: &mut Vec<TraceLocatorRecordV1>,
    declared_external_ids: &mut BTreeSet<u32>,
) -> Result<(), InventoryError> {
    for rule_index in frame.matching_rules {
        let rule = &profile.rules[rule_index];
        match rule.capture {
            LocatorCaptureMode::Element => {
                append_locator(
                    locators,
                    rule,
                    qpid,
                    &frame.locator_path,
                    frame.element_id,
                    None,
                    None,
                    None,
                    None,
                    &frame.captured_attributes,
                    profile.limits.max_locators,
                )?;
            }
            LocatorCaptureMode::ElementId => {
                let element_id = frame.element_id
                    .ok_or_else(|| InventoryError::InvalidElementId(frame.locator_path.clone()))?;
                append_locator(
                    locators,
                    rule,
                    qpid,
                    &format!("{}@id", frame.locator_path),
                    Some(element_id),
                    None,
                    None,
                    Some(&element_id.to_string()),
                    None,
                    &frame.captured_attributes,
                    profile.limits.max_locators,
                )?;
                if rule.declares_external_document_reference {
                    declared_external_ids.insert(element_id);
                }
            }
            LocatorCaptureMode::Text => {
                append_locator(
                    locators,
                    rule,
                    qpid,
                    &format!("{}#text", frame.locator_path),
                    frame.element_id,
                    None,
                    None,
                    Some(&frame.direct_text),
                    None,
                    &frame.captured_attributes,
                    profile.limits.max_locators,
                )?;
            }
            LocatorCaptureMode::DocumentQpid => {
                if frame.has_child_element {
                    return Err(InventoryError::InvalidQpid(frame.locator_path.clone()));
                }
                let value = frame.direct_text.trim();
                if !valid_qpid(value) || !value.eq_ignore_ascii_case(qpid) {
                    return Err(InventoryError::DocumentQpidMismatch(frame.locator_path.clone()));
                }
                append_locator(
                    locators,
                    rule,
                    qpid,
                    &format!("{}#qpid", frame.locator_path),
                    None,
                    None,
                    None,
                    Some(value),
                    Some(value),
                    &frame.captured_attributes,
                    profile.limits.max_locators,
                )?;
            }
            LocatorCaptureMode::ExternalPlanQpid => {
                if frame.has_child_element {
                    return Err(InventoryError::InvalidExternalPlanQpid(frame.locator_path.clone()));
                }
                let value = frame.direct_text.trim();
                if !valid_qpid(value) {
                    return Err(InventoryError::InvalidExternalPlanQpid(frame.locator_path.clone()));
                }
                let external_reference_id = frame.external_document_reference_id
                    .ok_or_else(|| InventoryError::InvalidExternalPlanQpid(frame.locator_path.clone()))?;
                append_locator(
                    locators,
                    rule,
                    qpid,
                    &format!("{}#qpid", frame.locator_path),
                    None,
                    Some(external_reference_id),
                    None,
                    Some(value),
                    Some(value),
                    &frame.captured_attributes,
                    profile.limits.max_locators,
                )?;
            }
            LocatorCaptureMode::ExternalPlanUri => {
                if frame.has_child_element {
                    return Err(InventoryError::InvalidIdentity(frame.locator_path.clone()));
                }
                if frame.direct_text.chars().any(|character| character.is_control()) {
                    return Err(InventoryError::InvalidIdentity(format!("external_uri:{}", frame.locator_path)));
                }
                let external_reference_id = frame.external_document_reference_id
                    .ok_or_else(|| InventoryError::InvalidIdentity(frame.locator_path.clone()))?;
                append_locator(
                    locators,
                    rule,
                    qpid,
                    &format!("{}#uri", frame.locator_path),
                    None,
                    Some(external_reference_id),
                    None,
                    Some(&frame.direct_text),
                    Some(&frame.direct_text),
                    &frame.captured_attributes,
                    profile.limits.max_locators,
                )?;
            }
            LocatorCaptureMode::QifReference => {
                if frame.has_child_element {
                    return Err(InventoryError::InvalidQifReference(frame.locator_path));
                }
                let reference_id = parsed_numeric_attribute(
                    &frame.direct_text,
                    &frame.locator_path,
                ).map_err(|_| InventoryError::InvalidQifReference(frame.locator_path.clone()))?;
                append_locator(
                    locators,
                    rule,
                    qpid,
                    &format!("{}#qif-reference", frame.locator_path),
                    frame.element_id,
                    Some(reference_id),
                    frame.external_xid,
                    Some(&frame.direct_text),
                    None,
                    &frame.captured_attributes,
                    profile.limits.max_locators,
                )?;
            }
            LocatorCaptureMode::DocumentQpid => {
                if frame.has_child_element {
                    return Err(InventoryError::InvalidQpid(frame.locator_path.clone()));
                }
                let value = frame.direct_text.trim();
                if !valid_qpid(value) || !value.eq_ignore_ascii_case(qpid) {
                    return Err(InventoryError::DocumentQpidMismatch(frame.locator_path.clone()));
                }
                append_locator(
                    locators,
                    rule,
                    qpid,
                    &format!("{}#qpid", frame.locator_path),
                    None,
                    None,
                    None,
                    Some(value),
                    Some(value),
                    &frame.captured_attributes,
                    profile.limits.max_locators,
                )?;
            }
            LocatorCaptureMode::ExternalPlanQpid => {
                if frame.has_child_element {
                    return Err(InventoryError::InvalidExternalPlanQpid(frame.locator_path.clone()));
                }
                let value = frame.direct_text.trim();
                if !valid_qpid(value) {
                    return Err(InventoryError::InvalidExternalPlanQpid(frame.locator_path.clone()));
                }
                let external_reference_id = frame.external_document_reference_id
                    .ok_or_else(|| InventoryError::InvalidExternalPlanQpid(frame.locator_path.clone()))?;
                append_locator(
                    locators,
                    rule,
                    qpid,
                    &format!("{}#qpid", frame.locator_path),
                    None,
                    Some(external_reference_id),
                    None,
                    Some(value),
                    Some(value),
                    &frame.captured_attributes,
                    profile.limits.max_locators,
                )?;
            }
            LocatorCaptureMode::ExternalPlanUri => {
                if frame.has_child_element {
                    return Err(InventoryError::InvalidIdentity(frame.locator_path.clone()));
                }
                if frame.direct_text.chars().any(|character| character.is_control()) {
                    return Err(InventoryError::InvalidIdentity(format!("external_uri:{}", frame.locator_path)));
                }
                let external_reference_id = frame.external_document_reference_id
                    .ok_or_else(|| InventoryError::InvalidIdentity(frame.locator_path.clone()))?;
                append_locator(
                    locators,
                    rule,
                    qpid,
                    &format!("{}#uri", frame.locator_path),
                    None,
                    Some(external_reference_id),
                    None,
                    Some(&frame.direct_text),
                    Some(&frame.direct_text),
                    &frame.captured_attributes,
                    profile.limits.max_locators,
                )?;
            }
        }
    }
    Ok(())
}

fn scan_document(
    profile: &TraceLocatorProfileV1,
    qpid: &str,
    role: QifDocumentRole,
    bytes: &[u8],
) -> Result<ScannedDocument, InventoryError> {
    if bytes.len() as u64 > profile.limits.max_source_bytes {
        return Err(InventoryError::SourceTooLarge);
    }
    let _text = parse_utf8_profile(bytes)?;
    let mut reader = NsReader::from_reader(Cursor::new(bytes));
    reader.config_mut().check_end_names = true;
    reader.resolver_mut().set_max_namespace_bindings(profile.limits.max_namespace_bindings as usize);
    let mut buffer = Vec::new();
    let mut stack: Vec<ElementFrame> = Vec::new();
    let mut locator_rows = Vec::new();
    let mut declared_external_ids = BTreeSet::new();
    let mut element_count = 0u64;
    let mut max_depth = 0u32;
    let mut total_text_bytes = 0u64;
    let mut root_expanded_name = None;

    loop {
        let (resolution, event) = reader.read_resolved_event_into(&mut buffer)
            .map_err(|error| InventoryError::XmlParse {
                position: reader.error_position(),
                reason: error.to_string(),
            })?;
        let is_empty_element = matches!(&event, Event::Empty(_));
        match event {
            Event::Start(start) | Event::Empty(start) => {
                element_count += 1;
                if element_count > profile.limits.max_elements {
                    return Err(InventoryError::ElementLimitExceeded);
                }
                let local = start.local_name();
                let local_name = local.as_ref();
                let expanded = expanded_name(resolution, local_name)?;
                let (element_id, external_xid, raw_attributes, _) = attribute_values(
                    &start,
                    reader.resolver(),
                    profile.limits.max_attributes_per_element,
                    &expanded,
                )?;

                let occurrence = if let Some(parent) = stack.last_mut() {
                    parent.has_child_element = true;
                    let counter = parent.child_occurrences.entry(expanded.clone()).or_insert(0);
                    *counter += 1;
                    *counter
                } else {
                    if root_expanded_name.is_some() {
                        return Err(InventoryError::MultipleRoots);
                    }
                    if local_name != "QIFDocument" {
                        return Err(InventoryError::WrongRoot);
                    }
                    root_expanded_name = Some(expanded.clone());
                    1
                };

                let mut expanded_path = stack
                    .last()
                    .map(|parent| parent.expanded_path.clone())
                    .unwrap_or_default();
                expanded_path.push(expanded.clone());
                let locator_path = if let Some(parent) = stack.last() {
                    format!("{}/{}[{}]", parent.locator_path, expanded, occurrence)
                } else {
                    "/QIFDocument".to_owned()
                };
                let matching_rules = profile.rules.iter().enumerate()
                    .filter_map(|(index, rule)| {
                        (rule.document_role == role && expanded_path_matches(&rule.expanded_path, &expanded_path))
                            .then_some(index)
                    })
                    .collect::<Vec<_>>();
                let mut captured_attributes = BTreeMap::new();
                for rule_index in &matching_rules {
                    for attribute_name in &profile.rules[*rule_index].capture_attributes {
                        if let Some(value) = raw_attributes.get(attribute_name) {
                            captured_attributes.insert(attribute_name.clone(), value.clone());
                        }
                    }
                }

                let is_external_document_reference = matching_rules.iter()
                    .any(|rule_index| profile.rules[*rule_index].declares_external_document_reference);
                let parent_external_reference_id = stack.iter().rev()
                    .find(|parent| parent.is_external_document_reference)
                    .and_then(|parent| parent.element_id);
                let frame = ElementFrame {
                    expanded_name: expanded,
                    expanded_path,
                    locator_path,
                    child_occurrences: BTreeMap::new(),
                    matching_rules,
                    element_id,
                    external_xid,
                    external_document_reference_id: if is_external_document_reference {
                        element_id
                    } else {
                        parent_external_reference_id
                    },
                    is_external_document_reference,
                    captured_attributes,
                    direct_text: String::new(),
                    has_child_element: false,
                };
                max_depth = max_depth.max((stack.len() + 1) as u32);
                if max_depth > profile.limits.max_depth {
                    return Err(InventoryError::DepthLimitExceeded);
                }
                if is_empty_element {
                    finalize_frame(frame, profile, qpid, &mut locator_rows, &mut declared_external_ids)?;
                } else {
                    stack.push(frame);
                }
            }
            Event::End(end) => {
                let frame = stack.pop().ok_or(InventoryError::ElementStackMismatch)?;
                let local_name = end.name().local_name();
                let actual = expanded_name(resolution, local_name.as_ref())?;
                if frame.expanded_name != actual {
                    return Err(InventoryError::ElementStackMismatch);
                }
                finalize_frame(frame, profile, qpid, &mut locator_rows, &mut declared_external_ids)?;
            }
            Event::Text(text) => {
                if let Some(frame) = stack.last_mut() {
                    let value = text.as_ref();
                    let new_len = frame.direct_text.len() as u64 + value.len() as u64;
                    if new_len > profile.limits.max_text_bytes_per_element {
                        return Err(InventoryError::ElementTextLimitExceeded);
                    }
                    total_text_bytes = total_text_bytes.saturating_add(value.len() as u64);
                    if total_text_bytes > profile.limits.max_total_text_bytes {
                        return Err(InventoryError::TotalTextLimitExceeded);
                    }
                    frame.direct_text.push_str(value);
                }
            }
            Event::CData(text) => {
                if let Some(frame) = stack.last_mut() {
                    let value = text.as_ref();
                    let new_len = frame.direct_text.len() as u64 + value.len() as u64;
                    if new_len > profile.limits.max_text_bytes_per_element {
                        return Err(InventoryError::ElementTextLimitExceeded);
                    }
                    total_text_bytes = total_text_bytes.saturating_add(value.len() as u64);
                    if total_text_bytes > profile.limits.max_total_text_bytes {
                        return Err(InventoryError::TotalTextLimitExceeded);
                    }
                    frame.direct_text.push_str(value);
                }
            }
            Event::GeneralRef(reference) => {
                let name = reference.as_ref();
                let replacement = if let Some(value) = resolve_xml_entity(name) {
                    value.to_owned()
                } else if name.starts_with('#') {
                    unescape(&format!("&{name};"))
                        .map_err(|_| InventoryError::InvalidCharacterReference(name.into()))?
                        .into_owned()
                } else {
                    return Err(InventoryError::CustomEntityForbidden(name.into()));
                };
                if let Some(frame) = stack.last_mut() {
                    let new_len = frame.direct_text.len() as u64 + replacement.len() as u64;
                    if new_len > profile.limits.max_text_bytes_per_element {
                        return Err(InventoryError::ElementTextLimitExceeded);
                    }
                    total_text_bytes = total_text_bytes.saturating_add(replacement.len() as u64);
                    if total_text_bytes > profile.limits.max_total_text_bytes {
                        return Err(InventoryError::TotalTextLimitExceeded);
                    }
                    frame.direct_text.push_str(&replacement);
                }
            }
            Event::DocType(_) => return Err(InventoryError::DoctypeForbidden),
            Event::Eof => break,
            Event::Decl(_) | Event::Comment(_) | Event::PI(_) => {}
        }
        buffer.clear();
    }

    if !stack.is_empty() || root_expanded_name.is_none() {
        return Err(InventoryError::IncompleteDocument);
    }

    let mut locators = BTreeSet::new();
    for record in &locator_rows {
        if !locators.insert(locator_sort_key(&record.locator)) {
            return Err(InventoryError::DuplicateSelector(record.locator.source_path.clone()));
        }
        if let Some(reference_id) = record.locator.external_document_reference_id {
            if role != QifDocumentRole::Results
                || !declared_external_ids.contains(&reference_id)
            {
                return Err(InventoryError::UndeclaredExternalReference(
                    record.locator.source_path.clone(),
                    reference_id,
                ));
            }
        }
    }

    Ok(ScannedDocument {
        source: TraceLocatorSourceDocumentV1 {
            qpid: qpid.to_ascii_lowercase(),
            role,
            sha256: sha256_hex(bytes),
            byte_length: bytes.len() as u64,
            root_expanded_name: root_expanded_name.unwrap_or_default(),
            element_count,
            max_observed_depth: max_depth,
            text_bytes: total_text_bytes,
        },
        locators: locator_rows,
        declared_external_ids,
    })
}

pub fn derive_trace_locator_inventory(
    inventory_id: &str,
    profile: &TraceLocatorProfileV1,
    documents: &[RawQifDocument<'_>],
) -> Result<(TraceLocatorInventoryV1, TraceLocatorInventoryReport), InventoryError> {
    validate_profile(profile)?;
    if inventory_id.trim().is_empty() || documents.is_empty() {
        return Err(InventoryError::InvalidIdentity("inventory_id/documents".into()));
    }
    let profile_sha256 = compute_profile_digest(profile)?;
    if !valid_sha256(&profile_sha256) {
        return Err(InventoryError::ProfileDigestMismatch);
    }

    let mut qpid_set = BTreeSet::new();
    let mut scanned = Vec::new();
    for document in documents {
        if !valid_qpid(document.qpid) {
            return Err(InventoryError::InvalidQpid(document.qpid.into()));
        }
        if !qpid_set.insert(document.qpid.to_ascii_lowercase()) {
            return Err(InventoryError::InvalidIdentity(format!("duplicate_qpid:{}", document.qpid)));
        }
        scanned.push(scan_document(profile, document.qpid, document.role, document.bytes)?);
    }

    let results: Vec<_> = scanned.iter()
        .filter(|document| document.source.role == QifDocumentRole::Results)
        .collect();
    if results.len() != 1 {
        return Err(InventoryError::InvalidIdentity("results_document_cardinality".into()));
    }
    let declared_external_document_reference_ids = results[0]
        .declared_external_ids.iter().copied().collect::<Vec<_>>();
    let plan_qpids: BTreeSet<_> = scanned.iter()
        .filter(|document| document.source.role == QifDocumentRole::Plan)
        .map(|document| document.source.qpid.to_ascii_lowercase())
        .collect();
    let mut external_document_references = Vec::new();
    let mut external_reference_ids = BTreeSet::new();
    for record in &results[0].locators {
        if record.rule_id != "results.external_document_reference" {
            continue;
        }
        let local_reference_id = record.locator.source_element_id
            .ok_or_else(|| InventoryError::InvalidElementId(record.locator.source_path.clone()))?;
        if !external_reference_ids.insert(local_reference_id) {
            return Err(InventoryError::DuplicateExternalReferenceId(local_reference_id));
        }
        let plan_qpid_records: Vec<_> = results[0].locators.iter()
            .filter(|candidate| {
                candidate.rule_id == "results.external_document_plan_qpid"
                    && candidate.locator.external_document_reference_id == Some(local_reference_id)
            })
            .collect();
        if plan_qpid_records.len() != 1 {
            return Err(InventoryError::InvalidExternalPlanQpid(record.locator.source_path.clone()));
        }
        let plan_qpid = plan_qpid_records[0].captured_value.as_ref()
            .ok_or_else(|| InventoryError::InvalidExternalPlanQpid(record.locator.source_path.clone()))?
            .to_ascii_lowercase();
        if !plan_qpids.contains(&plan_qpid) {
            return Err(InventoryError::UnknownExternalPlanQpid(record.locator.source_path.clone()));
        }
        let uri_records: Vec<_> = results[0].locators.iter()
            .filter(|candidate| {
                candidate.rule_id == "results.external_document_uri"
                    && candidate.locator.external_document_reference_id == Some(local_reference_id)
            })
            .collect();
        if uri_records.len() > 1 {
            return Err(InventoryError::DuplicateExternalReferenceId(local_reference_id));
        }
        let source_uri = uri_records.first().and_then(|candidate| candidate.captured_value.clone());
        external_document_references.push(ExternalQifDocumentReferenceV1 {
            results_qpid: results[0].source.qpid.to_ascii_lowercase(),
            local_reference_id,
            plan_qpid,
            source_uri,
            source_locator: record.locator.source_path.clone(),
        });
    }
    external_document_references.sort_by(|left, right| left.local_reference_id.cmp(&right.local_reference_id));

    let mut source_documents = scanned.iter().map(|document| document.source.clone()).collect::<Vec<_>>();
    source_documents.sort_by(|left, right| left.qpid.cmp(&right.qpid));
    let mut locators = scanned.iter().flat_map(|document| document.locators.clone()).collect::<Vec<_>>();
    locators.sort_by(|left, right| {
        locator_sort_key(&left.locator).cmp(&locator_sort_key(&right.locator))
            .then(left.rule_id.cmp(&right.rule_id))
    });

    let body = TraceLocatorInventoryBodyV1 {
        inventory_id: inventory_id.into(),
        profile_id: profile.profile_id.clone(),
        profile_version: profile.profile_version.clone(),
        profile_sha256,
        qif_standard_version: profile.qif_standard_version.clone(),
        documents: source_documents,
        locators,
        declared_external_document_reference_ids,
        external_document_references,
    };
    let mut inventory = TraceLocatorInventoryV1 {
        schema: INVENTORY_SCHEMA.into(),
        body,
        inventory_sha256: String::new(),
    };
    inventory.inventory_sha256 = compute_inventory_digest(&inventory)?;

    let report = TraceLocatorInventoryReport {
        disposition: InventoryDisposition::Complete,
        profile_sha256: inventory.body.profile_sha256.clone(),
        inventory_sha256: inventory.inventory_sha256.clone(),
        source_document_count: inventory.body.documents.len(),
        locator_count: inventory.body.locators.len(),
        declared_external_reference_count: inventory.body.declared_external_document_reference_ids.len(),
        elements_scanned: inventory.body.documents.iter().map(|document| document.element_count).sum(),
        text_bytes_scanned: inventory.body.documents.iter().map(|document| document.text_bytes).sum(),
        max_observed_depth: inventory.body.documents.iter().map(|document| document.max_observed_depth).max().unwrap_or(0),
    };
    Ok((inventory, report))
}

pub fn inventory_required_locators(
    inventory: &TraceLocatorInventoryV1,
) -> Result<Vec<SourceFieldLocatorV1>, InventoryError> {
    if inventory.schema != INVENTORY_SCHEMA
        || !valid_sha256(&inventory.inventory_sha256)
        || compute_inventory_digest(inventory)? != inventory.inventory_sha256
        || !valid_sha256(&inventory.body.profile_sha256)
    {
        return Err(InventoryError::InventoryDigestMismatch);
    }
    let mut locators = inventory.body.locators.iter()
        .map(|record| record.locator.clone())
        .collect::<Vec<_>>();
    locators.sort_by_key(locator_sort_key);
    Ok(locators)
}

#[cfg(test)]
mod tests {
    use super::*;

    const NS: &str = "http://qifstandards.org/xsd/qif3";

    fn expanded(local: &str) -> String {
        format!("{{{NS}}}{local}")
    }

    fn profile() -> TraceLocatorProfileV1 {
        TraceLocatorProfileV1 {
            schema: PROFILE_SCHEMA.into(),
            profile_id: "AERO_QIF_TEST7_8_TRACE".into(),
            profile_version: "1.0.0".into(),
            qif_standard_version: "3.0.0".into(),
            rules: vec![
                TraceLocatorRuleV1 {
                    rule_id: "results.document_qpid".into(),
                    document_role: QifDocumentRole::Results,
                    expanded_path: vec![expanded("QIFDocument"), expanded("QPId")],
                    capture: LocatorCaptureMode::DocumentQpid,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
                TraceLocatorRuleV1 {
                    rule_id: "plan.document_qpid".into(),
                    document_role: QifDocumentRole::Plan,
                    expanded_path: vec![expanded("QIFDocument"), expanded("QPId")],
                    capture: LocatorCaptureMode::DocumentQpid,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
                TraceLocatorRuleV1 {
                    rule_id: "results.external_document_reference".into(),
                    document_role: QifDocumentRole::Results,
                    expanded_path: vec![
                        expanded("QIFDocument"),
                        expanded("ExternalQIFReferences"),
                        expanded("ExternalQIFDocument"),
                    ],
                    capture: LocatorCaptureMode::ElementId,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: true,
                },
                TraceLocatorRuleV1 {
                    rule_id: "results.external_document_plan_qpid".into(),
                    document_role: QifDocumentRole::Results,
                    expanded_path: vec![
                        expanded("QIFDocument"),
                        expanded("ExternalQIFReferences"),
                        expanded("ExternalQIFDocument"),
                        expanded("QPId"),
                    ],
                    capture: LocatorCaptureMode::ExternalPlanQpid,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
                TraceLocatorRuleV1 {
                    rule_id: "results.external_document_uri".into(),
                    document_role: QifDocumentRole::Results,
                    expanded_path: vec![
                        expanded("QIFDocument"),
                        expanded("ExternalQIFReferences"),
                        expanded("ExternalQIFDocument"),
                        expanded("URI"),
                    ],
                    capture: LocatorCaptureMode::ExternalPlanUri,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
                TraceLocatorRuleV1 {
                    rule_id: "results.measurement_results_set".into(),
                    document_role: QifDocumentRole::Results,
                    expanded_path: vec![
                        expanded("QIFDocument"),
                        expanded("Results"),
                        expanded("MeasurementResultsSet"),
                    ],
                    capture: LocatorCaptureMode::Element,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
                TraceLocatorRuleV1 {
                    rule_id: "results.measurement_results".into(),
                    document_role: QifDocumentRole::Results,
                    expanded_path: vec![
                        expanded("QIFDocument"),
                        expanded("Results"),
                        expanded("MeasurementResultsSet"),
                        expanded("MeasurementResults"),
                    ],
                    capture: LocatorCaptureMode::ElementId,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
                TraceLocatorRuleV1 {
                    rule_id: "results.characteristic_measurement".into(),
                    document_role: QifDocumentRole::Results,
                    expanded_path: vec![
                        expanded("QIFDocument"),
                        expanded("Results"),
                        expanded("MeasurementResultsSet"),
                        expanded("MeasurementResults"),
                        expanded("MeasuredCharacteristics"),
                        expanded("CharacteristicMeasurements"),
                        expanded("*"),
                    ],
                    capture: LocatorCaptureMode::ElementId,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
                TraceLocatorRuleV1 {
                    rule_id: "results.characteristic_item_external_reference".into(),
                    document_role: QifDocumentRole::Results,
                    expanded_path: vec![
                        expanded("QIFDocument"),
                        expanded("Results"),
                        expanded("MeasurementResultsSet"),
                        expanded("MeasurementResults"),
                        expanded("MeasuredCharacteristics"),
                        expanded("CharacteristicMeasurements"),
                        expanded("*"),
                        expanded("CharacteristicItemId"),
                    ],
                    capture: LocatorCaptureMode::QifReference,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
                TraceLocatorRuleV1 {
                    rule_id: "plan.characteristic_item".into(),
                    document_role: QifDocumentRole::Plan,
                    expanded_path: vec![
                        expanded("QIFDocument"),
                        expanded("Characteristics"),
                        expanded("CharacteristicItems"),
                        expanded("*"),
                    ],
                    capture: LocatorCaptureMode::ElementId,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
                TraceLocatorRuleV1 {
                    rule_id: "plan.characteristic_item_nominal_reference".into(),
                    document_role: QifDocumentRole::Plan,
                    expanded_path: vec![
                        expanded("QIFDocument"),
                        expanded("Characteristics"),
                        expanded("CharacteristicItems"),
                        expanded("*"),
                        expanded("CharacteristicNominalId"),
                    ],
                    capture: LocatorCaptureMode::QifReference,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
                TraceLocatorRuleV1 {
                    rule_id: "plan.characteristic_nominal".into(),
                    document_role: QifDocumentRole::Plan,
                    expanded_path: vec![
                        expanded("QIFDocument"),
                        expanded("Characteristics"),
                        expanded("CharacteristicNominals"),
                        expanded("*"),
                    ],
                    capture: LocatorCaptureMode::ElementId,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
                TraceLocatorRuleV1 {
                    rule_id: "plan.entity_external_id".into(),
                    document_role: QifDocumentRole::Plan,
                    expanded_path: vec![
                        expanded("QIFDocument"),
                        expanded("Characteristics"),
                        expanded("CharacteristicNominals"),
                        expanded("*"),
                        expanded("EntityExternalIds"),
                        expanded("Id"),
                    ],
                    capture: LocatorCaptureMode::Text,
                    capture_attributes: Vec::new(),
                    declares_external_document_reference: false,
                },
            ],
            limits: TraceLocatorLimitsV1 {
                max_source_bytes: 1024 * 1024,
                max_elements: 10_000,
                max_depth: 32,
                max_attributes_per_element: 32,
                max_text_bytes_per_element: 4096,
                max_total_text_bytes: 1024 * 1024,
                max_locators: 10_000,
                max_namespace_bindings: 64,
            },
        }
    }

    fn docs<'a>(results: &'a [u8], plan: &'a [u8]) -> Vec<RawQifDocument<'a>> {
        vec![
            RawQifDocument {
                qpid: "10000000-0000-4000-8000-000000000001",
                role: QifDocumentRole::Results,
                bytes: results,
            },
            RawQifDocument {
                qpid: "20000000-0000-4000-8000-000000000002",
                role: QifDocumentRole::Plan,
                bytes: plan,
            },
        ]
    }

    const RESULTS: &str = r#"
<QIFDocument xmlns="urn:qif-test-v3">
  <Results>
    <ExternalQIFDocumentReference id="7" QPId="20000000-0000-4000-8000-000000000002" URI="Plan.qif"/>
    <MeasurementResultsSet id="10">
      <CharacteristicMeasurement id="901">
        <CharacteristicItemId xId="301">7</CharacteristicItemId>
      </CharacteristicMeasurement>
      <CharacteristicMeasurement id="902">
        <CharacteristicItemId xId="302">7</CharacteristicItemId>
      </CharacteristicMeasurement>
    </MeasurementResultsSet>
    <MeasurementResultsSet id="11">
      <CharacteristicMeasurement id="903">
        <CharacteristicItemId xId="301">7</CharacteristicItemId>
      </CharacteristicMeasurement>
    </MeasurementResultsSet>
  </Results>
</QIFDocument>
"#;

    const PLAN: &str = r#"
<q:QIFDocument xmlns:q="urn:qif-test-v3">
  <q:Plan>
    <q:CharacteristicItem id="301"/>
    <q:CharacteristicItem id="302"/>
    <q:CharacteristicNominal id="501">
      <q:EntityExternalIds><q:Id>9001</q:Id></q:EntityExternalIds>
    </q:CharacteristicNominal>
  </q:Plan>
</q:QIFDocument>
"#;

    #[test]
    fn derives_namespace_aware_inventory_from_raw_results_and_plan() {
        let documents = docs(RESULTS.as_bytes(), PLAN.as_bytes());
        let (inventory, report) = derive_trace_locator_inventory("test-inventory-1", &profile(), &documents).unwrap();
        assert_eq!(report.disposition, InventoryDisposition::Complete);
        assert_eq!(report.source_document_count, 2);
        assert_eq!(report.declared_external_reference_count, 1);
        assert_eq!(report.locator_count, 13);
        assert_eq!(
            inventory_required_locators(&inventory).unwrap().len(),
            13
        );
    }

    #[test]
    fn prefixes_do_not_change_expanded_name_identity_or_inventory() {
        let results_prefixed = RESULTS
            .replace("<QIFDocument xmlns=\"urn:qif-test-v3\">", "<a:QIFDocument xmlns:a=\"urn:qif-test-v3\">")
            .replace("</QIFDocument>", "</a:QIFDocument>")
            .replace("<Results>", "<a:Results>").replace("</Results>", "</a:Results>")
            .replace("<ExternalQIFDocumentReference", "<a:ExternalQIFDocumentReference")
            .replace("/>", "/>")
            .replace("<MeasurementResultsSet", "<a:MeasurementResultsSet")
            .replace("</MeasurementResultsSet>", "</a:MeasurementResultsSet>")
            .replace("<CharacteristicMeasurement", "<a:CharacteristicMeasurement")
            .replace("</CharacteristicMeasurement>", "</a:CharacteristicMeasurement>")
            .replace("<CharacteristicItemId", "<a:CharacteristicItemId")
            .replace("</CharacteristicItemId>", "</a:CharacteristicItemId>");
        let docs1 = docs(RESULTS.as_bytes(), PLAN.as_bytes());
        let docs2 = docs(results_prefixed.as_bytes(), PLAN.as_bytes());
        let (first, _) = derive_trace_locator_inventory("same", &profile(), &docs1).unwrap();
        let (second, _) = derive_trace_locator_inventory("same", &profile(), &docs2).unwrap();
        let a = inventory_required_locators(&first).unwrap();
        let b = inventory_required_locators(&second).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn same_numeric_xid_in_other_plan_document_does_not_change_source_inventory() {
        let plan2 = PLAN.replace("id=\"301\"", "id=\"301\"");
        let docs = docs(RESULTS.as_bytes(), plan2.as_bytes());
        let (inventory, _) = derive_trace_locator_inventory("id-scope", &profile(), &docs).unwrap();
        let xid_refs: Vec<_> = inventory.body.locators.iter()
            .filter_map(|record| record.locator.external_xid)
            .collect();
        assert_eq!(xid_refs, vec![301, 302, 301]);
    }

    #[test]
    fn missing_external_xid_remains_a_locator_because_qif_xid_is_optional() {
        let results = RESULTS.replace(" xId=\"301\"", "");
        let docs = docs(results.as_bytes(), PLAN.as_bytes());
        let (inventory, _) = derive_trace_locator_inventory("optional-xid", &profile(), &docs).unwrap();
        let refs: Vec<_> = inventory.body.locators.iter()
            .filter(|record| record.rule_id == "results.characteristic_item_external_reference")
            .collect();
        assert_eq!(refs[0].locator.external_document_reference_id, Some(7));
        assert_eq!(refs[0].locator.external_xid, None);
    }

    #[test]
    fn dtd_and_doctype_are_rejected_without_external_resolution() {
        let xml = r#"<!DOCTYPE QIFDocument SYSTEM "file:///etc/passwd"><QIFDocument xmlns="urn:qif-test-v3"><Results/></QIFDocument>"#;
        assert_eq!(
            derive_trace_locator_inventory("doctype", &profile(), &docs(xml.as_bytes(), PLAN.as_bytes())),
            Err(InventoryError::DoctypeForbidden)
        );
    }

    #[test]
    fn custom_entity_is_rejected_but_predefined_and_numeric_refs_are_preserved() {
        let custom = r#"<!DOCTYPE QIFDocument [<!ENTITY p "7">]><QIFDocument xmlns="urn:qif-test-v3"><Results><ExternalQIFDocumentReference id="&p;"/></Results></QIFDocument>"#;
        assert_eq!(
            derive_trace_locator_inventory("custom-entity", &profile(), &docs(custom.as_bytes(), PLAN.as_bytes())),
            Err(InventoryError::DoctypeForbidden)
        );
        let results = RESULTS.replace(">7</CharacteristicItemId>", "&#55;</CharacteristicItemId>");
        let (inventory, _) = derive_trace_locator_inventory("numeric-reference", &profile(), &docs(results.as_bytes(), PLAN.as_bytes())).unwrap();
        assert_eq!(
            inventory.body.locators.iter().find(|row| row.rule_id == "results.characteristic_item_external_reference").unwrap().locator.external_document_reference_id,
            Some(7)
        );
    }

    #[test]
    fn malformed_end_tag_is_rejected() {
        let broken = RESULTS.replace("</CharacteristicMeasurement>", "</MeasurementResultsSet>");
        assert!(matches!(
            derive_trace_locator_inventory("malformed", &profile(), &docs(broken.as_bytes(), PLAN.as_bytes())),
            Err(InventoryError::XmlParse { .. }) | Err(InventoryError::ElementStackMismatch)
        ));
    }

    #[test]
    fn unbound_prefix_is_rejected() {
        let xml = r#"<q:QIFDocument><q:Results/></q:QIFDocument>"#;
        assert!(matches!(
            derive_trace_locator_inventory("unbound", &profile(), &docs(xml.as_bytes(), PLAN.as_bytes())),
            Err(InventoryError::UnboundNamespace(_))
        ));
    }

    #[test]
    fn byte_and_depth_limits_fail_closed() {
        let mut small = profile();
        small.limits.max_source_bytes = 32;
        assert_eq!(
            derive_trace_locator_inventory("byte-limit", &small, &docs(RESULTS.as_bytes(), PLAN.as_bytes())),
            Err(InventoryError::SourceTooLarge)
        );
        let deep = r#"<QIFDocument xmlns="urn:qif-test-v3"><Results><MeasurementResultsSet><CharacteristicMeasurement><CharacteristicItemId>7</CharacteristicItemId></CharacteristicMeasurement></MeasurementResultsSet></Results></QIFDocument>"#;
        let mut shallow = profile();
        shallow.limits.max_depth = 2;
        assert_eq!(
            derive_trace_locator_inventory("depth-limit", &shallow, &docs(deep.as_bytes(), PLAN.as_bytes())),
            Err(InventoryError::DepthLimitExceeded)
        );
    }

    #[test]
    fn inventory_digest_changes_when_source_bytes_or_profile_changes() {
        let documents = docs(RESULTS.as_bytes(), PLAN.as_bytes());
        let (first, _) = derive_trace_locator_inventory("digest", &profile(), &documents).unwrap();
        let changed_profile = {
            let mut value = profile();
            value.profile_version = "1.0.1".into();
            value
        };
        let (second, _) = derive_trace_locator_inventory("digest", &changed_profile, &documents).unwrap();
        assert_ne!(first.body.profile_sha256, second.body.profile_sha256);
        assert_ne!(first.inventory_sha256, second.inventory_sha256);
    }
}

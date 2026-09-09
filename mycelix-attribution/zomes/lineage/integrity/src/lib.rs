// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Contribution-lineage evidence for Mycelix Attribution.
//!
//! V1 is deliberately evidence-only. A contribution record or lineage attestation does
//! not create ownership, payment, reputation, governance weight, execution authority,
//! or control over downstream artifacts.
//!
//! Index links are discovery aids only. CL-02 binds every index to the exact target
//! record semantics at DHT validation time, and coordinator queries re-read target
//! records and verify payload semantics again before returning them.

use hdi::prelude::*;
use serde::{Deserialize, Serialize};

pub const LINEAGE_SCHEMA_VERSION: u16 = 1;
pub const MAX_ID_LEN: usize = 256;
pub const MAX_SUBJECT_REF_LEN: usize = 1024;
pub const MAX_TITLE_LEN: usize = 200;
pub const MAX_DESCRIPTION_LEN: usize = 4_000;
pub const MAX_RATIONALE_LEN: usize = 4_000;
pub const MAX_EVIDENCE_REFS: usize = 32;
pub const MAX_EVIDENCE_KIND_LEN: usize = 64;
pub const MAX_EVIDENCE_REF_LEN: usize = 2_048;
pub const MAX_EVIDENCE_NOTE_LEN: usize = 2_000;
pub const MAX_CONFIDENCE_BPS: u16 = 10_000;
pub const MAX_ANCHOR_LEN: usize = 2_048;

pub const ALL_CONTRIBUTIONS_ANCHOR: &str = "lineage:v1:contributions:all";
pub const ALL_ATTESTATIONS_ANCHOR: &str = "lineage:v1:attestations:all";

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ContributionKind {
    Code,
    Design,
    Research,
    Dataset,
    Validation,
    Documentation,
    Infrastructure,
    Funding,
    Compute,
    Care,
    Stewardship,
    Other,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum LineageRelation {
    BuiltUpon,
    DerivedFrom,
    EnabledBy,
    ValidatedBy,
    Reused,
    InspiredBy,
    FundedBy,
    HostedBy,
    MaintainedBy,
    Other,
}

/// Bounded reference to evidence supporting a contribution or lineage claim.
/// Shape is validated here; truth/availability qualification belongs to later layers.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct EvidenceRef {
    pub kind: String,
    pub reference: String,
    pub note: Option<String>,
}

/// Self-authored description of a contribution, artifact, or enabling capability.
/// The Holochain action timestamp, rather than a caller-controlled field, is time provenance.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ContributionRecord {
    pub schema_version: u16,
    pub id: String,
    pub contributor_did: String,
    pub subject_ref: String,
    pub kind: ContributionKind,
    pub title: String,
    pub description: String,
    pub evidence_refs: Vec<EvidenceRef>,
}

/// Self-authored, contestable source→target relationship claim.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct LineageAttestation {
    pub schema_version: u16,
    pub id: String,
    pub attestor_did: String,
    pub source_ref: String,
    pub target_ref: String,
    pub relation: LineageRelation,
    pub confidence_bps: u16,
    pub evidence_refs: Vec<EvidenceRef>,
    pub rationale: String,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    ContributionRecord(ContributionRecord),
    LineageAttestation(LineageAttestation),
}

/// Discovery indexes only. A link never independently establishes the semantic property
/// named by its variant; integrity and consumers both validate the target payload.
#[hdk_link_types]
pub enum LinkTypes {
    AllContributions,
    ContributionById,
    SubjectToContribution,
    ContributorToContribution,
    AllAttestations,
    AttestationById,
    SourceToAttestation,
    TargetToAttestation,
    AttestorToAttestation,
}

fn invalid(message: impl Into<String>) -> ValidateCallbackResult {
    ValidateCallbackResult::Invalid(message.into())
}

fn validate_required_bounded(field: &str, value: &str, max_len: usize) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{field} must not be empty"));
    }
    if value.len() > max_len {
        return Err(format!("{field} must be at most {max_len} bytes"));
    }
    Ok(())
}

fn validate_optional_bounded(field: &str, value: Option<&str>, max_len: usize) -> Result<(), String> {
    if let Some(value) = value {
        if value.len() > max_len {
            return Err(format!("{field} must be at most {max_len} bytes"));
        }
    }
    Ok(())
}

fn validate_schema(schema_version: u16) -> Result<(), String> {
    if schema_version != LINEAGE_SCHEMA_VERSION {
        return Err(format!(
            "unsupported lineage schema version {schema_version}; expected {LINEAGE_SCHEMA_VERSION}"
        ));
    }
    Ok(())
}

fn validate_evidence_refs(evidence_refs: &[EvidenceRef]) -> Result<(), String> {
    if evidence_refs.len() > MAX_EVIDENCE_REFS {
        return Err(format!(
            "evidence_refs must contain at most {MAX_EVIDENCE_REFS} items"
        ));
    }
    for evidence in evidence_refs {
        validate_required_bounded("evidence.kind", &evidence.kind, MAX_EVIDENCE_KIND_LEN)?;
        validate_required_bounded("evidence.reference", &evidence.reference, MAX_EVIDENCE_REF_LEN)?;
        validate_optional_bounded(
            "evidence.note",
            evidence.note.as_deref(),
            MAX_EVIDENCE_NOTE_LEN,
        )?;
    }
    Ok(())
}

fn require_claimant_is_author(claimant_did: &str, author: &AgentPubKey) -> Result<(), String> {
    let expected = format!("did:mycelix:{author}");
    if claimant_did != expected {
        return Err(format!(
            "claimant DID must be the committing agent (lineage identity forgery): expected '{expected}', got '{claimant_did}'"
        ));
    }
    Ok(())
}

pub fn validate_contribution_fields(record: &ContributionRecord) -> Result<(), String> {
    validate_schema(record.schema_version)?;
    validate_required_bounded("id", &record.id, MAX_ID_LEN)?;
    validate_required_bounded("contributor_did", &record.contributor_did, MAX_ID_LEN)?;
    validate_required_bounded("subject_ref", &record.subject_ref, MAX_SUBJECT_REF_LEN)?;
    validate_required_bounded("title", &record.title, MAX_TITLE_LEN)?;
    validate_required_bounded("description", &record.description, MAX_DESCRIPTION_LEN)?;
    validate_evidence_refs(&record.evidence_refs)?;
    Ok(())
}

pub fn validate_attestation_fields(attestation: &LineageAttestation) -> Result<(), String> {
    validate_schema(attestation.schema_version)?;
    validate_required_bounded("id", &attestation.id, MAX_ID_LEN)?;
    validate_required_bounded("attestor_did", &attestation.attestor_did, MAX_ID_LEN)?;
    validate_required_bounded("source_ref", &attestation.source_ref, MAX_SUBJECT_REF_LEN)?;
    validate_required_bounded("target_ref", &attestation.target_ref, MAX_SUBJECT_REF_LEN)?;
    if attestation.source_ref.trim() == attestation.target_ref.trim() {
        return Err("source_ref and target_ref must differ".into());
    }
    if attestation.confidence_bps > MAX_CONFIDENCE_BPS {
        return Err(format!(
            "confidence_bps must be within 0..={MAX_CONFIDENCE_BPS}"
        ));
    }
    validate_evidence_refs(&attestation.evidence_refs)?;
    validate_required_bounded("rationale", &attestation.rationale, MAX_RATIONALE_LEN)?;
    Ok(())
}

pub fn validate_create_contribution(
    action: &Create,
    record: &ContributionRecord,
) -> ValidateCallbackResult {
    if let Err(error) = validate_contribution_fields(record) {
        return invalid(error);
    }
    if let Err(error) = require_claimant_is_author(&record.contributor_did, &action.author) {
        return invalid(error);
    }
    ValidateCallbackResult::Valid
}

pub fn validate_create_attestation(
    action: &Create,
    attestation: &LineageAttestation,
) -> ValidateCallbackResult {
    if let Err(error) = validate_attestation_fields(attestation) {
        return invalid(error);
    }
    if let Err(error) = require_claimant_is_author(&attestation.attestor_did, &action.author) {
        return invalid(error);
    }
    ValidateCallbackResult::Valid
}

fn validate_anchor(anchor: &Anchor) -> ValidateCallbackResult {
    match validate_required_bounded("anchor", &anchor.0, MAX_ANCHOR_LEN) {
        Ok(()) => ValidateCallbackResult::Valid,
        Err(error) => invalid(error),
    }
}

/// Length-prefix components so caller-controlled delimiters cannot make two semantic
/// component tuples hash to the same anchor string.
fn component(value: &str) -> String {
    format!("{}:{value}", value.len())
}

pub fn contribution_id_anchor(contributor_did: &str, id: &str) -> String {
    format!(
        "lineage:v1:contribution:id:{}:{}",
        component(contributor_did),
        component(id)
    )
}

pub fn contribution_subject_anchor(subject_ref: &str) -> String {
    format!(
        "lineage:v1:contribution:subject:{}",
        component(subject_ref)
    )
}

pub fn contribution_contributor_anchor(contributor_did: &str) -> String {
    format!(
        "lineage:v1:contribution:contributor:{}",
        component(contributor_did)
    )
}

pub fn attestation_id_anchor(attestor_did: &str, id: &str) -> String {
    format!(
        "lineage:v1:attestation:id:{}:{}",
        component(attestor_did),
        component(id)
    )
}

pub fn attestation_source_anchor(source_ref: &str) -> String {
    format!(
        "lineage:v1:attestation:source:{}",
        component(source_ref)
    )
}

pub fn attestation_target_anchor(target_ref: &str) -> String {
    format!(
        "lineage:v1:attestation:target:{}",
        component(target_ref)
    )
}

pub fn attestation_attestor_anchor(attestor_did: &str) -> String {
    format!(
        "lineage:v1:attestation:attestor:{}",
        component(attestor_did)
    )
}

fn validate_index_shape(
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    tag: LinkTag,
) -> Result<(EntryHash, ActionHash), String> {
    let base = EntryHash::try_from(base_address)
        .map_err(|_| "lineage index base must be an EntryHash anchor".to_string())?;
    let target = ActionHash::try_from(target_address)
        .map_err(|_| "lineage index target must be an ActionHash".to_string())?;
    if !tag.0.is_empty() {
        return Err("lineage index links must use an empty tag".into());
    }
    Ok((base, target))
}

fn contribution_anchor_for(link_type: &LinkTypes, record: &ContributionRecord) -> Option<String> {
    match link_type {
        LinkTypes::AllContributions => Some(ALL_CONTRIBUTIONS_ANCHOR.into()),
        LinkTypes::ContributionById => {
            Some(contribution_id_anchor(&record.contributor_did, &record.id))
        }
        LinkTypes::SubjectToContribution => Some(contribution_subject_anchor(&record.subject_ref)),
        LinkTypes::ContributorToContribution => {
            Some(contribution_contributor_anchor(&record.contributor_did))
        }
        _ => None,
    }
}

fn attestation_anchor_for(link_type: &LinkTypes, claim: &LineageAttestation) -> Option<String> {
    match link_type {
        LinkTypes::AllAttestations => Some(ALL_ATTESTATIONS_ANCHOR.into()),
        LinkTypes::AttestationById => Some(attestation_id_anchor(&claim.attestor_did, &claim.id)),
        LinkTypes::SourceToAttestation => Some(attestation_source_anchor(&claim.source_ref)),
        LinkTypes::TargetToAttestation => Some(attestation_target_anchor(&claim.target_ref)),
        LinkTypes::AttestorToAttestation => Some(attestation_attestor_anchor(&claim.attestor_did)),
        _ => None,
    }
}

fn wrong_target_type(link_type: &LinkTypes) -> ValidateCallbackResult {
    invalid(format!(
        "lineage index {:?} points to the wrong entry type",
        link_type
    ))
}

fn validate_expected_anchor(base: EntryHash, expected: String) -> ExternResult<ValidateCallbackResult> {
    if expected.len() > MAX_ANCHOR_LEN {
        return Ok(invalid("derived lineage anchor exceeds MAX_ANCHOR_LEN"));
    }
    let expected_hash = hash_entry(Anchor(expected))?;
    if base != expected_hash {
        return Ok(invalid(
            "lineage index base does not match the canonical anchor derived from the target payload",
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Bind index shape, author, target type, and anchor semantics at DHT validation time.
///
/// `must_get_valid_record` deliberately makes the index depend on a target record that
/// the visible network considers valid. If the dependency is unavailable, Holochain
/// returns an unresolved dependency rather than accepting an unverifiable index.
fn validate_create_index_link(
    link_type: LinkTypes,
    action: CreateLink,
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    tag: LinkTag,
) -> ExternResult<ValidateCallbackResult> {
    let (base, target) = match validate_index_shape(base_address, target_address, tag) {
        Ok(parts) => parts,
        Err(error) => return Ok(invalid(error)),
    };

    let target_record = must_get_valid_record(target)?;
    if target_record.action().author() != &action.author {
        return Ok(invalid(
            "lineage index author must match the author of the indexed target record",
        ));
    }

    if let Some(expected) = match target_record
        .entry()
        .to_app_option::<ContributionRecord>()
    {
        Ok(Some(record)) => contribution_anchor_for(&link_type, &record),
        Ok(None) | Err(_) => None,
    } {
        return validate_expected_anchor(base, expected);
    }

    if let Some(expected) = match target_record
        .entry()
        .to_app_option::<LineageAttestation>()
    {
        Ok(Some(claim)) => attestation_anchor_for(&link_type, &claim),
        Ok(None) | Err(_) => None,
    } {
        return validate_expected_anchor(base, expected);
    }

    Ok(wrong_target_type(&link_type))
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => Ok(match app_entry {
                EntryTypes::Anchor(anchor) => validate_anchor(&anchor),
                EntryTypes::ContributionRecord(record) => {
                    validate_create_contribution(&action, &record)
                }
                EntryTypes::LineageAttestation(attestation) => {
                    validate_create_attestation(&action, &attestation)
                }
            }),
            OpEntry::UpdateEntry { .. } => Ok(invalid(
                "lineage evidence is immutable in v1; publish a correction/supersession entry instead",
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            action,
            base_address,
            target_address,
            tag,
        } => validate_create_index_link(link_type, action, base_address, target_address, tag),
        FlatOp::RegisterDeleteLink { .. } => Ok(invalid(
            "lineage index links are append-only in v1; delete-link operations are not permitted",
        )),
        FlatOp::RegisterUpdate(_) => Ok(invalid(
            "lineage evidence is immutable in v1; updates are not permitted",
        )),
        FlatOp::RegisterDelete(_) => Ok(invalid(
            "lineage evidence is append-only in v1; deletes are not permitted",
        )),
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_action() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![7u8; 36]),
            timestamp: Timestamp::from_micros(1_000),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![1u8; 36]),
            weight: Default::default(),
        }
    }

    fn author_did() -> String {
        format!("did:mycelix:{}", test_action().author)
    }

    fn evidence() -> EvidenceRef {
        EvidenceRef {
            kind: "measurement".into(),
            reference: "blake3:abc123".into(),
            note: Some("supports the claim; does not establish causality alone".into()),
        }
    }

    fn contribution() -> ContributionRecord {
        ContributionRecord {
            schema_version: LINEAGE_SCHEMA_VERSION,
            id: "contribution:alpha".into(),
            contributor_did: author_did(),
            subject_ref: "artifact:alpha".into(),
            kind: ContributionKind::Code,
            title: "Reusable parser".into(),
            description: "A reusable parser used by a downstream tool.".into(),
            evidence_refs: vec![evidence()],
        }
    }

    fn attestation() -> LineageAttestation {
        LineageAttestation {
            schema_version: LINEAGE_SCHEMA_VERSION,
            id: "lineage:alpha-beta".into(),
            attestor_did: author_did(),
            source_ref: "artifact:alpha".into(),
            target_ref: "artifact:beta".into(),
            relation: LineageRelation::BuiltUpon,
            confidence_bps: 8_000,
            evidence_refs: vec![evidence()],
            rationale: "Beta imports and extends Alpha's parser API.".into(),
        }
    }

    #[test]
    fn accepts_self_authored_contribution() {
        assert_eq!(
            validate_create_contribution(&test_action(), &contribution()),
            ValidateCallbackResult::Valid
        );
    }

    #[test]
    fn rejects_contributor_identity_forgery() {
        let mut record = contribution();
        record.contributor_did = "did:mycelix:someone-else".into();
        assert!(matches!(
            validate_create_contribution(&test_action(), &record),
            ValidateCallbackResult::Invalid(message) if message.contains("identity forgery")
        ));
    }

    #[test]
    fn accepts_self_authored_lineage_claim() {
        assert_eq!(
            validate_create_attestation(&test_action(), &attestation()),
            ValidateCallbackResult::Valid
        );
    }

    #[test]
    fn rejects_attestor_identity_forgery() {
        let mut claim = attestation();
        claim.attestor_did = "did:mycelix:someone-else".into();
        assert!(matches!(
            validate_create_attestation(&test_action(), &claim),
            ValidateCallbackResult::Invalid(message) if message.contains("identity forgery")
        ));
    }

    #[test]
    fn rejects_self_referential_lineage() {
        let mut claim = attestation();
        claim.target_ref = claim.source_ref.clone();
        assert_eq!(
            validate_attestation_fields(&claim),
            Err("source_ref and target_ref must differ".into())
        );
    }

    #[test]
    fn rejects_confidence_over_one_hundred_percent() {
        let mut claim = attestation();
        claim.confidence_bps = 10_001;
        assert!(validate_attestation_fields(&claim)
            .unwrap_err()
            .contains("confidence_bps"));
    }

    #[test]
    fn rejects_unknown_schema() {
        let mut record = contribution();
        record.schema_version = LINEAGE_SCHEMA_VERSION + 1;
        assert!(validate_contribution_fields(&record)
            .unwrap_err()
            .contains("unsupported lineage schema"));
    }

    #[test]
    fn rejects_empty_subject_reference() {
        let mut record = contribution();
        record.subject_ref = "   ".into();
        assert!(validate_contribution_fields(&record)
            .unwrap_err()
            .contains("subject_ref"));
    }

    #[test]
    fn rejects_too_many_evidence_references() {
        let mut claim = attestation();
        claim.evidence_refs = vec![evidence(); MAX_EVIDENCE_REFS + 1];
        assert!(validate_attestation_fields(&claim)
            .unwrap_err()
            .contains("evidence_refs"));
    }

    #[test]
    fn rejects_unbounded_evidence_reference() {
        let mut claim = attestation();
        claim.evidence_refs[0].reference = "x".repeat(MAX_EVIDENCE_REF_LEN + 1);
        assert!(validate_attestation_fields(&claim)
            .unwrap_err()
            .contains("evidence.reference"));
    }

    #[test]
    fn zero_confidence_is_valid_and_remains_explicit() {
        let mut claim = attestation();
        claim.confidence_bps = 0;
        assert_eq!(validate_attestation_fields(&claim), Ok(()));
    }

    #[test]
    fn index_shape_accepts_action_target_and_empty_tag() {
        let result = validate_index_shape(
            AnyLinkableHash::from(EntryHash::from_raw_36(vec![1u8; 36])),
            AnyLinkableHash::from(ActionHash::from_raw_36(vec![2u8; 36])),
            LinkTag::new(Vec::<u8>::new()),
        );
        assert!(result.is_ok());
    }

    #[test]
    fn index_shape_rejects_entry_target() {
        let result = validate_index_shape(
            AnyLinkableHash::from(EntryHash::from_raw_36(vec![1u8; 36])),
            AnyLinkableHash::from(EntryHash::from_raw_36(vec![2u8; 36])),
            LinkTag::new(Vec::<u8>::new()),
        );
        assert!(result.is_err());
    }

    #[test]
    fn index_shape_rejects_nonempty_tag() {
        let result = validate_index_shape(
            AnyLinkableHash::from(EntryHash::from_raw_36(vec![1u8; 36])),
            AnyLinkableHash::from(ActionHash::from_raw_36(vec![2u8; 36])),
            LinkTag::new(vec![1]),
        );
        assert!(result.is_err());
    }

    #[test]
    fn scoped_anchor_encoding_is_unambiguous() {
        assert_ne!(
            contribution_id_anchor("did:example:a:b", "c"),
            contribution_id_anchor("did:example:a", "b:c")
        );
    }

    #[test]
    fn contribution_link_types_derive_only_contribution_anchors() {
        let record = contribution();
        assert_eq!(
            contribution_anchor_for(&LinkTypes::AllContributions, &record),
            Some(ALL_CONTRIBUTIONS_ANCHOR.into())
        );
        assert!(contribution_anchor_for(&LinkTypes::AllAttestations, &record).is_none());
    }

    #[test]
    fn attestation_link_types_derive_only_attestation_anchors() {
        let claim = attestation();
        assert_eq!(
            attestation_anchor_for(&LinkTypes::AllAttestations, &claim),
            Some(ALL_ATTESTATIONS_ANCHOR.into())
        );
        assert!(attestation_anchor_for(&LinkTypes::AllContributions, &claim).is_none());
    }
}

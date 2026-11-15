// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Contribution-lineage evidence for Mycelix Attribution.
//!
//! V1 is deliberately evidence-only. A contribution record, lineage attestation, or
//! response does not create ownership, payment, reputation, governance weight,
//! execution authority, or control over downstream artifacts.
//!
//! Index links are discovery aids only. Every index is bound to the exact target record
//! semantics at DHT validation time, and coordinator queries re-read target records and
//! verify payload semantics again before returning them.
//!
//! CL-03 adds immutable responses to evidence: independent corroboration/contest and
//! author-only retraction/supersession. Historical evidence is never edited or deleted.

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
pub const ALL_RESPONSES_ANCHOR: &str = "lineage:v1:responses:all";

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

/// How a responder relates to an existing contribution or lineage-attestation record.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ResponseDisposition {
    Corroborate,
    Contest,
    Retract,
    Supersede,
}

/// Bounded reference to evidence supporting a contribution, claim, or response.
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

/// Immutable response to one exact contribution or lineage attestation action.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct LineageResponse {
    pub schema_version: u16,
    pub id: String,
    pub responder_did: String,
    pub subject_action: ActionHash,
    pub disposition: ResponseDisposition,
    /// Required for independent corroboration/contest; absent for retract/supersede.
    pub confidence_bps: Option<u16>,
    pub evidence_refs: Vec<EvidenceRef>,
    pub rationale: String,
    /// Required only for `Supersede`.
    pub replacement_action: Option<ActionHash>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    ContributionRecord(ContributionRecord),
    LineageAttestation(LineageAttestation),
    LineageResponse(LineageResponse),
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
    AllResponses,
    ResponseById,
    SubjectToResponse,
    ResponderToResponse,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EvidenceRecordKind {
    Contribution,
    Attestation,
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

pub fn validate_response_fields(response: &LineageResponse) -> Result<(), String> {
    validate_schema(response.schema_version)?;
    validate_required_bounded("id", &response.id, MAX_ID_LEN)?;
    validate_required_bounded("responder_did", &response.responder_did, MAX_ID_LEN)?;
    validate_required_bounded("rationale", &response.rationale, MAX_RATIONALE_LEN)?;
    validate_evidence_refs(&response.evidence_refs)?;

    if response.replacement_action.as_ref() == Some(&response.subject_action) {
        return Err("replacement_action must differ from subject_action".into());
    }

    match response.disposition {
        ResponseDisposition::Corroborate | ResponseDisposition::Contest => {
            let Some(confidence) = response.confidence_bps else {
                return Err("corroborate/contest responses require confidence_bps".into());
            };
            if confidence > MAX_CONFIDENCE_BPS {
                return Err(format!(
                    "confidence_bps must be within 0..={MAX_CONFIDENCE_BPS}"
                ));
            }
            if response.replacement_action.is_some() {
                return Err(
                    "corroborate/contest responses must not specify replacement_action".into(),
                );
            }
        }
        ResponseDisposition::Retract => {
            if response.confidence_bps.is_some() {
                return Err("retract responses must not specify confidence_bps".into());
            }
            if response.replacement_action.is_some() {
                return Err("retract responses must not specify replacement_action".into());
            }
        }
        ResponseDisposition::Supersede => {
            if response.confidence_bps.is_some() {
                return Err("supersede responses must not specify confidence_bps".into());
            }
            if response.replacement_action.is_none() {
                return Err("supersede responses require replacement_action".into());
            }
        }
    }

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

fn evidence_record_kind(record: &Record) -> Option<EvidenceRecordKind> {
    match record.entry().to_app_option::<ContributionRecord>() {
        Ok(Some(payload)) if validate_contribution_fields(&payload).is_ok() => {
            return Some(EvidenceRecordKind::Contribution);
        }
        _ => {}
    }
    match record.entry().to_app_option::<LineageAttestation>() {
        Ok(Some(payload)) if validate_attestation_fields(&payload).is_ok() => {
            Some(EvidenceRecordKind::Attestation)
        }
        _ => None,
    }
}

fn record_author_did(record: &Record) -> String {
    format!("did:mycelix:{}", record.action().author())
}

fn validate_create_response(
    action: &Create,
    response: &LineageResponse,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = validate_response_fields(response) {
        return Ok(invalid(error));
    }
    if let Err(error) = require_claimant_is_author(&response.responder_did, &action.author) {
        return Ok(invalid(error));
    }

    let subject = must_get_valid_record(response.subject_action.clone())?;
    let Some(subject_kind) = evidence_record_kind(&subject) else {
        return Ok(invalid(
            "lineage response subject must be a valid contribution or lineage attestation",
        ));
    };
    let subject_author = record_author_did(&subject);

    match response.disposition {
        ResponseDisposition::Corroborate | ResponseDisposition::Contest => {
            if response.responder_did == subject_author {
                return Ok(invalid(
                    "corroborate/contest responses must be independent of the subject author; use retract/supersede for self-correction",
                ));
            }
        }
        ResponseDisposition::Retract => {
            if response.responder_did != subject_author {
                return Ok(invalid(
                    "only the original subject author may retract lineage evidence",
                ));
            }
        }
        ResponseDisposition::Supersede => {
            if response.responder_did != subject_author {
                return Ok(invalid(
                    "only the original subject author may supersede lineage evidence",
                ));
            }
            let Some(replacement_hash) = response.replacement_action.clone() else {
                return Ok(invalid("supersede response is missing replacement_action"));
            };
            let replacement = must_get_valid_record(replacement_hash)?;
            let Some(replacement_kind) = evidence_record_kind(&replacement) else {
                return Ok(invalid(
                    "superseding replacement must be a valid contribution or lineage attestation",
                ));
            };
            if replacement_kind != subject_kind {
                return Ok(invalid(
                    "superseding replacement must use the same evidence entry family as the subject",
                ));
            }
            if record_author_did(&replacement) != subject_author {
                return Ok(invalid(
                    "superseding replacement must be authored by the original subject author",
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
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

pub fn response_id_anchor(responder_did: &str, id: &str) -> String {
    format!(
        "lineage:v1:response:id:{}:{}",
        component(responder_did),
        component(id)
    )
}

pub fn response_responder_anchor(responder_did: &str) -> String {
    format!(
        "lineage:v1:response:responder:{}",
        component(responder_did)
    )
}

fn validate_index_target_and_tag(
    target_address: AnyLinkableHash,
    tag: LinkTag,
) -> Result<ActionHash, String> {
    let target = ActionHash::try_from(target_address)
        .map_err(|_| "lineage index target must be an ActionHash".to_string())?;
    if !tag.0.is_empty() {
        return Err("lineage index links must use an empty tag".into());
    }
    Ok(target)
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

fn response_anchor_for(link_type: &LinkTypes, response: &LineageResponse) -> Option<String> {
    match link_type {
        LinkTypes::AllResponses => Some(ALL_RESPONSES_ANCHOR.into()),
        LinkTypes::ResponseById => Some(response_id_anchor(&response.responder_did, &response.id)),
        LinkTypes::ResponderToResponse => Some(response_responder_anchor(&response.responder_did)),
        _ => None,
    }
}

fn validate_expected_entry_anchor(
    base_address: AnyLinkableHash,
    expected: String,
) -> ExternResult<ValidateCallbackResult> {
    let base = match EntryHash::try_from(base_address) {
        Ok(base) => base,
        Err(_) => return Ok(invalid("lineage anchor index base must be an EntryHash")),
    };
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

fn validate_response_subject_base(
    base_address: AnyLinkableHash,
    response: &LineageResponse,
) -> ValidateCallbackResult {
    let Ok(base) = ActionHash::try_from(base_address) else {
        return invalid("SubjectToResponse base must be an ActionHash");
    };
    if base != response.subject_action {
        return invalid("SubjectToResponse base must equal response.subject_action");
    }
    ValidateCallbackResult::Valid
}

/// Bind index shape, author, target type, and base semantics at DHT validation time.
fn validate_create_index_link(
    link_type: LinkTypes,
    action: CreateLink,
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    tag: LinkTag,
) -> ExternResult<ValidateCallbackResult> {
    let target = match validate_index_target_and_tag(target_address, tag) {
        Ok(target) => target,
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
        return validate_expected_entry_anchor(base_address, expected);
    }

    if let Some(expected) = match target_record
        .entry()
        .to_app_option::<LineageAttestation>()
    {
        Ok(Some(claim)) => attestation_anchor_for(&link_type, &claim),
        Ok(None) | Err(_) => None,
    } {
        return validate_expected_entry_anchor(base_address, expected);
    }

    if let Ok(Some(response)) = target_record.entry().to_app_option::<LineageResponse>() {
        if validate_response_fields(&response).is_err() {
            return Ok(invalid("lineage response index target has invalid fields"));
        }
        if link_type == LinkTypes::SubjectToResponse {
            return Ok(validate_response_subject_base(base_address, &response));
        }
        if let Some(expected) = response_anchor_for(&link_type, &response) {
            return validate_expected_entry_anchor(base_address, expected);
        }
    }

    Ok(invalid(
        "lineage index points to the wrong entry type for this index family",
    ))
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(anchor) => Ok(validate_anchor(&anchor)),
                EntryTypes::ContributionRecord(record) => {
                    Ok(validate_create_contribution(&action, &record))
                }
                EntryTypes::LineageAttestation(attestation) => {
                    Ok(validate_create_attestation(&action, &attestation))
                }
                EntryTypes::LineageResponse(response) => {
                    validate_create_response(&action, &response)
                }
            },
            OpEntry::UpdateEntry { .. } => Ok(invalid(
                "lineage evidence is immutable in v1; publish an append-only response instead",
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

    fn response(disposition: ResponseDisposition) -> LineageResponse {
        LineageResponse {
            schema_version: LINEAGE_SCHEMA_VERSION,
            id: "response:1".into(),
            responder_did: author_did(),
            subject_action: ActionHash::from_raw_36(vec![3u8; 36]),
            disposition: disposition.clone(),
            confidence_bps: match disposition {
                ResponseDisposition::Corroborate | ResponseDisposition::Contest => Some(7_500),
                ResponseDisposition::Retract | ResponseDisposition::Supersede => None,
            },
            evidence_refs: vec![evidence()],
            rationale: "Evidence-bearing response".into(),
            replacement_action: match disposition {
                ResponseDisposition::Supersede => {
                    Some(ActionHash::from_raw_36(vec![4u8; 36]))
                }
                _ => None,
            },
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
    fn rejects_self_referential_lineage() {
        let mut claim = attestation();
        claim.target_ref = claim.source_ref.clone();
        assert_eq!(
            validate_attestation_fields(&claim),
            Err("source_ref and target_ref must differ".into())
        );
    }

    #[test]
    fn contest_requires_explicit_confidence() {
        let mut item = response(ResponseDisposition::Contest);
        item.confidence_bps = None;
        assert!(validate_response_fields(&item)
            .unwrap_err()
            .contains("require confidence_bps"));
    }

    #[test]
    fn retract_rejects_confidence_and_replacement() {
        let mut item = response(ResponseDisposition::Retract);
        item.confidence_bps = Some(5_000);
        assert!(validate_response_fields(&item)
            .unwrap_err()
            .contains("must not specify confidence_bps"));

        let mut item = response(ResponseDisposition::Retract);
        item.replacement_action = Some(ActionHash::from_raw_36(vec![9u8; 36]));
        assert!(validate_response_fields(&item)
            .unwrap_err()
            .contains("must not specify replacement_action"));
    }

    #[test]
    fn supersede_requires_distinct_replacement() {
        let mut item = response(ResponseDisposition::Supersede);
        item.replacement_action = None;
        assert!(validate_response_fields(&item)
            .unwrap_err()
            .contains("require replacement_action"));

        let mut item = response(ResponseDisposition::Supersede);
        item.replacement_action = Some(item.subject_action.clone());
        assert!(validate_response_fields(&item)
            .unwrap_err()
            .contains("must differ"));
    }

    #[test]
    fn index_target_and_tag_accept_action_target_and_empty_tag() {
        let result = validate_index_target_and_tag(
            AnyLinkableHash::from(ActionHash::from_raw_36(vec![2u8; 36])),
            LinkTag::new(Vec::<u8>::new()),
        );
        assert!(result.is_ok());
    }

    #[test]
    fn index_target_and_tag_reject_entry_target() {
        let result = validate_index_target_and_tag(
            AnyLinkableHash::from(EntryHash::from_raw_36(vec![2u8; 36])),
            LinkTag::new(Vec::<u8>::new()),
        );
        assert!(result.is_err());
    }

    #[test]
    fn scoped_anchor_encoding_is_unambiguous() {
        assert_ne!(
            contribution_id_anchor("did:example:a:b", "c"),
            contribution_id_anchor("did:example:a", "b:c")
        );
        assert_ne!(
            response_id_anchor("did:example:a:b", "c"),
            response_id_anchor("did:example:a", "b:c")
        );
    }

    #[test]
    fn response_anchor_family_excludes_subject_action_index() {
        let item = response(ResponseDisposition::Contest);
        assert_eq!(
            response_anchor_for(&LinkTypes::AllResponses, &item),
            Some(ALL_RESPONSES_ANCHOR.into())
        );
        assert!(response_anchor_for(&LinkTypes::SubjectToResponse, &item).is_none());
    }

    #[test]
    fn subject_response_index_requires_exact_action_base() {
        let item = response(ResponseDisposition::Contest);
        assert_eq!(
            validate_response_subject_base(
                AnyLinkableHash::from(item.subject_action.clone()),
                &item,
            ),
            ValidateCallbackResult::Valid
        );
        assert!(matches!(
            validate_response_subject_base(
                AnyLinkableHash::from(ActionHash::from_raw_36(vec![8u8; 36])),
                &item,
            ),
            ValidateCallbackResult::Invalid(_)
        ));
    }
}

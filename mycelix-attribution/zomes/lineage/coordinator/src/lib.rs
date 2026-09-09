// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Coordinator for contribution-lineage evidence.
//!
//! This zome indexes and queries immutable evidence. It does not assign ownership,
//! reputation, payments, governance weight, execution authority, or causal truth.
//! Index links are treated as hints: every query re-reads and re-validates the target
//! record payload before returning it.

use hdk::prelude::*;
use lineage_integrity::*;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

const MAX_PAGE_SIZE: u32 = 200;

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "type", content = "payload")]
pub enum LineageSignal {
    ContributionPublished {
        id: String,
        contributor_did: String,
        subject_ref: String,
    },
    AttestationPublished {
        id: String,
        attestor_did: String,
        source_ref: String,
        target_ref: String,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct PaginationInput {
    pub offset: u32,
    pub limit: u32,
}

impl Default for PaginationInput {
    fn default() -> Self {
        Self {
            offset: 0,
            limit: 50,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PaginatedRecords {
    pub items: Vec<Record>,
    pub total: u32,
    pub offset: u32,
    pub limit: u32,
    pub has_more: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ScopedIdInput {
    pub actor_did: String,
    pub id: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct SubjectQueryInput {
    pub subject_ref: String,
    pub pagination: PaginationInput,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ActorQueryInput {
    pub actor_did: String,
    pub pagination: PaginationInput,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct EdgeQueryInput {
    pub subject_ref: String,
    pub pagination: PaginationInput,
}

fn guest(message: impl Into<String>) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(message.into()))
}

fn caller_did() -> ExternResult<String> {
    Ok(format!(
        "did:mycelix:{}",
        agent_info()?.agent_initial_pubkey
    ))
}

fn require_caller(claimant_did: &str) -> ExternResult<()> {
    let expected = caller_did()?;
    if claimant_did != expected {
        return Err(guest(format!(
            "lineage claimant DID must match the calling agent; expected '{expected}', got '{claimant_did}'"
        )));
    }
    Ok(())
}

fn validate_query_value(field: &str, value: &str, max_len: usize) -> ExternResult<()> {
    if value.trim().is_empty() {
        return Err(guest(format!("{field} must not be empty")));
    }
    if value.len() > max_len {
        return Err(guest(format!("{field} must be at most {max_len} bytes")));
    }
    Ok(())
}

fn validate_pagination(pagination: &PaginationInput) -> ExternResult<()> {
    if pagination.limit == 0 || pagination.limit > MAX_PAGE_SIZE {
        return Err(guest(format!(
            "pagination limit must be within 1..={MAX_PAGE_SIZE}"
        )));
    }
    Ok(())
}

fn paginate(records: Vec<Record>, pagination: PaginationInput) -> PaginatedRecords {
    let total = records.len().min(u32::MAX as usize) as u32;
    let start = (pagination.offset as usize).min(records.len());
    let end = start
        .saturating_add(pagination.limit as usize)
        .min(records.len());
    let items = records.into_iter().skip(start).take(end - start).collect();
    PaginatedRecords {
        items,
        total,
        offset: pagination.offset,
        limit: pagination.limit,
        has_more: end < total as usize,
    }
}

fn anchor_hash(anchor: impl Into<String>) -> ExternResult<EntryHash> {
    hash_entry(&Anchor(anchor.into()))
}

fn create_index(
    anchor: impl Into<String>,
    target: ActionHash,
    link_type: LinkTypes,
) -> ExternResult<()> {
    create_link(anchor_hash(anchor)?, target, link_type, ())?;
    Ok(())
}

/// Read action-hash index targets, silently dropping malformed targets, missing records,
/// and duplicate links. Payload semantics are checked by the typed filters below.
fn resolve_action_index(anchor: String, link_type: LinkTypes) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash(anchor)?, link_type)?,
        GetStrategy::default(),
    )?;

    let mut seen = HashSet::<ActionHash>::new();
    let mut records = Vec::new();
    for link in links {
        let Ok(action_hash) = ActionHash::try_from(link.target) else {
            continue;
        };
        if !seen.insert(action_hash.clone()) {
            continue;
        }
        if let Some(record) = get(action_hash, GetOptions::default())? {
            records.push(record);
        }
    }
    Ok(records)
}

fn record_author_did(record: &Record) -> String {
    format!("did:mycelix:{}", record.action().author())
}

fn contribution_payload(record: &Record) -> Option<ContributionRecord> {
    match record.entry().to_app_option::<ContributionRecord>() {
        Ok(Some(payload))
            if validate_contribution_fields(&payload).is_ok()
                && payload.contributor_did == record_author_did(record) =>
        {
            Some(payload)
        }
        _ => None,
    }
}

fn attestation_payload(record: &Record) -> Option<LineageAttestation> {
    match record.entry().to_app_option::<LineageAttestation>() {
        Ok(Some(payload))
            if validate_attestation_fields(&payload).is_ok()
                && payload.attestor_did == record_author_did(record) =>
        {
            Some(payload)
        }
        _ => None,
    }
}

fn filter_contributions<F>(records: Vec<Record>, predicate: F) -> Vec<Record>
where
    F: Fn(&ContributionRecord) -> bool,
{
    records
        .into_iter()
        .filter(|record| {
            contribution_payload(record)
                .as_ref()
                .is_some_and(&predicate)
        })
        .collect()
}

fn filter_attestations<F>(records: Vec<Record>, predicate: F) -> Vec<Record>
where
    F: Fn(&LineageAttestation) -> bool,
{
    records
        .into_iter()
        .filter(|record| {
            attestation_payload(record)
                .as_ref()
                .is_some_and(&predicate)
        })
        .collect()
}

fn contribution_matches_id(record: &ContributionRecord, actor_did: &str, id: &str) -> bool {
    record.contributor_did == actor_did && record.id == id
}

fn attestation_matches_id(claim: &LineageAttestation, actor_did: &str, id: &str) -> bool {
    claim.attestor_did == actor_did && claim.id == id
}

fn find_contribution_by_id(actor_did: &str, id: &str) -> ExternResult<Option<Record>> {
    let records = resolve_action_index(
        contribution_id_anchor(actor_did, id),
        LinkTypes::ContributionById,
    )?;
    Ok(filter_contributions(records, |record| {
        contribution_matches_id(record, actor_did, id)
    })
    .into_iter()
    .next())
}

fn find_attestation_by_id(actor_did: &str, id: &str) -> ExternResult<Option<Record>> {
    let records = resolve_action_index(
        attestation_id_anchor(actor_did, id),
        LinkTypes::AttestationById,
    )?;
    Ok(filter_attestations(records, |claim| {
        attestation_matches_id(claim, actor_did, id)
    })
    .into_iter()
    .next())
}

#[hdk_extern]
pub fn publish_contribution(record: ContributionRecord) -> ExternResult<Record> {
    validate_contribution_fields(&record).map_err(guest)?;
    require_caller(&record.contributor_did)?;

    // Coordinator-level duplicate suppression only. Concurrent publishes may race;
    // DHT validity does not depend on global uniqueness of caller-provided IDs.
    if find_contribution_by_id(&record.contributor_did, &record.id)?.is_some() {
        return Err(guest(format!(
            "contribution id '{}' already exists for this contributor",
            record.id
        )));
    }

    let action_hash = create_entry(&EntryTypes::ContributionRecord(record.clone()))?;

    create_index(
        ALL_CONTRIBUTIONS_ANCHOR,
        action_hash.clone(),
        LinkTypes::AllContributions,
    )?;
    create_index(
        contribution_id_anchor(&record.contributor_did, &record.id),
        action_hash.clone(),
        LinkTypes::ContributionById,
    )?;
    create_index(
        contribution_subject_anchor(&record.subject_ref),
        action_hash.clone(),
        LinkTypes::SubjectToContribution,
    )?;
    create_index(
        contribution_contributor_anchor(&record.contributor_did),
        action_hash.clone(),
        LinkTypes::ContributorToContribution,
    )?;

    let _ = emit_signal(&LineageSignal::ContributionPublished {
        id: record.id.clone(),
        contributor_did: record.contributor_did.clone(),
        subject_ref: record.subject_ref.clone(),
    });

    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        guest("could not fetch newly created contribution record")
    })
}

#[hdk_extern]
pub fn publish_attestation(claim: LineageAttestation) -> ExternResult<Record> {
    validate_attestation_fields(&claim).map_err(guest)?;
    require_caller(&claim.attestor_did)?;

    if find_attestation_by_id(&claim.attestor_did, &claim.id)?.is_some() {
        return Err(guest(format!(
            "attestation id '{}' already exists for this attestor",
            claim.id
        )));
    }

    let action_hash = create_entry(&EntryTypes::LineageAttestation(claim.clone()))?;

    create_index(
        ALL_ATTESTATIONS_ANCHOR,
        action_hash.clone(),
        LinkTypes::AllAttestations,
    )?;
    create_index(
        attestation_id_anchor(&claim.attestor_did, &claim.id),
        action_hash.clone(),
        LinkTypes::AttestationById,
    )?;
    create_index(
        attestation_source_anchor(&claim.source_ref),
        action_hash.clone(),
        LinkTypes::SourceToAttestation,
    )?;
    create_index(
        attestation_target_anchor(&claim.target_ref),
        action_hash.clone(),
        LinkTypes::TargetToAttestation,
    )?;
    create_index(
        attestation_attestor_anchor(&claim.attestor_did),
        action_hash.clone(),
        LinkTypes::AttestorToAttestation,
    )?;

    let _ = emit_signal(&LineageSignal::AttestationPublished {
        id: claim.id.clone(),
        attestor_did: claim.attestor_did.clone(),
        source_ref: claim.source_ref.clone(),
        target_ref: claim.target_ref.clone(),
    });

    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        guest("could not fetch newly created lineage attestation")
    })
}

#[hdk_extern]
pub fn get_contribution_by_id(input: ScopedIdInput) -> ExternResult<Option<Record>> {
    validate_query_value("actor_did", &input.actor_did, MAX_ID_LEN)?;
    validate_query_value("id", &input.id, MAX_ID_LEN)?;
    find_contribution_by_id(&input.actor_did, &input.id)
}

#[hdk_extern]
pub fn get_attestation_by_id(input: ScopedIdInput) -> ExternResult<Option<Record>> {
    validate_query_value("actor_did", &input.actor_did, MAX_ID_LEN)?;
    validate_query_value("id", &input.id, MAX_ID_LEN)?;
    find_attestation_by_id(&input.actor_did, &input.id)
}

#[hdk_extern]
pub fn get_all_contributions(pagination: PaginationInput) -> ExternResult<PaginatedRecords> {
    validate_pagination(&pagination)?;
    let records = resolve_action_index(
        ALL_CONTRIBUTIONS_ANCHOR.into(),
        LinkTypes::AllContributions,
    )?;
    Ok(paginate(
        filter_contributions(records, |_| true),
        pagination,
    ))
}

#[hdk_extern]
pub fn get_contributions_for_subject(input: SubjectQueryInput) -> ExternResult<PaginatedRecords> {
    validate_query_value("subject_ref", &input.subject_ref, MAX_SUBJECT_REF_LEN)?;
    validate_pagination(&input.pagination)?;
    let records = resolve_action_index(
        contribution_subject_anchor(&input.subject_ref),
        LinkTypes::SubjectToContribution,
    )?;
    Ok(paginate(
        filter_contributions(records, |record| record.subject_ref == input.subject_ref),
        input.pagination,
    ))
}

#[hdk_extern]
pub fn get_contributions_by_contributor(input: ActorQueryInput) -> ExternResult<PaginatedRecords> {
    validate_query_value("actor_did", &input.actor_did, MAX_ID_LEN)?;
    validate_pagination(&input.pagination)?;
    let records = resolve_action_index(
        contribution_contributor_anchor(&input.actor_did),
        LinkTypes::ContributorToContribution,
    )?;
    Ok(paginate(
        filter_contributions(records, |record| {
            record.contributor_did == input.actor_did
        }),
        input.pagination,
    ))
}

#[hdk_extern]
pub fn get_all_attestations(pagination: PaginationInput) -> ExternResult<PaginatedRecords> {
    validate_pagination(&pagination)?;
    let records = resolve_action_index(
        ALL_ATTESTATIONS_ANCHOR.into(),
        LinkTypes::AllAttestations,
    )?;
    Ok(paginate(
        filter_attestations(records, |_| true),
        pagination,
    ))
}

#[hdk_extern]
pub fn get_attestations_from_source(input: EdgeQueryInput) -> ExternResult<PaginatedRecords> {
    validate_query_value("subject_ref", &input.subject_ref, MAX_SUBJECT_REF_LEN)?;
    validate_pagination(&input.pagination)?;
    let records = resolve_action_index(
        attestation_source_anchor(&input.subject_ref),
        LinkTypes::SourceToAttestation,
    )?;
    Ok(paginate(
        filter_attestations(records, |claim| claim.source_ref == input.subject_ref),
        input.pagination,
    ))
}

#[hdk_extern]
pub fn get_attestations_to_target(input: EdgeQueryInput) -> ExternResult<PaginatedRecords> {
    validate_query_value("subject_ref", &input.subject_ref, MAX_SUBJECT_REF_LEN)?;
    validate_pagination(&input.pagination)?;
    let records = resolve_action_index(
        attestation_target_anchor(&input.subject_ref),
        LinkTypes::TargetToAttestation,
    )?;
    Ok(paginate(
        filter_attestations(records, |claim| claim.target_ref == input.subject_ref),
        input.pagination,
    ))
}

#[hdk_extern]
pub fn get_attestations_by_attestor(input: ActorQueryInput) -> ExternResult<PaginatedRecords> {
    validate_query_value("actor_did", &input.actor_did, MAX_ID_LEN)?;
    validate_pagination(&input.pagination)?;
    let records = resolve_action_index(
        attestation_attestor_anchor(&input.actor_did),
        LinkTypes::AttestorToAttestation,
    )?;
    Ok(paginate(
        filter_attestations(records, |claim| claim.attestor_did == input.actor_did),
        input.pagination,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contribution(actor: &str, id: &str, subject: &str) -> ContributionRecord {
        ContributionRecord {
            schema_version: LINEAGE_SCHEMA_VERSION,
            id: id.into(),
            contributor_did: actor.into(),
            subject_ref: subject.into(),
            kind: ContributionKind::Code,
            title: "Example".into(),
            description: "Example contribution".into(),
            evidence_refs: vec![],
        }
    }

    fn attestation(actor: &str, id: &str, source: &str, target: &str) -> LineageAttestation {
        LineageAttestation {
            schema_version: LINEAGE_SCHEMA_VERSION,
            id: id.into(),
            attestor_did: actor.into(),
            source_ref: source.into(),
            target_ref: target.into(),
            relation: LineageRelation::EnabledBy,
            confidence_bps: 5_000,
            evidence_refs: vec![],
            rationale: "Test claim".into(),
        }
    }

    #[test]
    fn ids_are_scoped_to_actor() {
        let record = contribution("did:mycelix:alice", "same-id", "artifact:a");
        assert!(contribution_matches_id(
            &record,
            "did:mycelix:alice",
            "same-id"
        ));
        assert!(!contribution_matches_id(
            &record,
            "did:mycelix:bob",
            "same-id"
        ));
    }

    #[test]
    fn attestation_ids_are_scoped_to_attestor() {
        let claim = attestation(
            "did:mycelix:alice",
            "same-id",
            "artifact:a",
            "artifact:b",
        );
        assert!(attestation_matches_id(
            &claim,
            "did:mycelix:alice",
            "same-id"
        ));
        assert!(!attestation_matches_id(
            &claim,
            "did:mycelix:bob",
            "same-id"
        ));
    }

    #[test]
    fn pagination_rejects_zero_and_oversize_limits() {
        assert!(validate_pagination(&PaginationInput { offset: 0, limit: 0 }).is_err());
        assert!(validate_pagination(&PaginationInput {
            offset: 0,
            limit: MAX_PAGE_SIZE + 1,
        })
        .is_err());
    }

    #[test]
    fn source_and_target_queries_remain_distinct() {
        let claim = attestation(
            "did:mycelix:alice",
            "edge",
            "artifact:source",
            "artifact:target",
        );
        assert_eq!(claim.source_ref, "artifact:source");
        assert_eq!(claim.target_ref, "artifact:target");
        assert_ne!(
            attestation_source_anchor(&claim.source_ref),
            attestation_target_anchor(&claim.target_ref)
        );
    }
}

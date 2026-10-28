#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Coordinator APIs for immutable federated clearing evidence.
//!
//! These calls author signed Holochain evidence; they do not write PostgreSQL.
//! The outbox dispatcher should call them with the stable event IDs and digests
//! committed by PostgreSQL. A successful call is an authored action, not proof
//! that the DHT validation-receipt threshold or external settlement finality is met.

use hdk::prelude::*;
use mycelix_finance_shared::anchor_hash;
use federated_clearing_integrity::*;
pub use federated_clearing_integrity::{
    ClearingDirection, FederatedObligationClaim, FederatedReservationManifest,
    FederatedSettlementEvidence,
};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct FederatedPublicationReceipt {
    pub replayed: bool,
    pub action_hash: ActionHash,
    pub entry_hash: EntryHash,
    pub event_id: String,
}

fn fail(reason: impl Into<String>) -> ExternResult<FederatedPublicationReceipt> {
    Err(wasm_error!(WasmErrorInner::Guest(reason.into())))
}

fn manifest_anchor(id: &str) -> ExternResult<EntryHash> {
    anchor_hash(&format!("ncf:federated-clearing:manifest:{id}"))
}
fn event_anchor(id: &str) -> ExternResult<EntryHash> {
    anchor_hash(&format!("ncf:federated-clearing:event:{id}"))
}
fn work_anchor(digest: &str) -> ExternResult<EntryHash> {
    anchor_hash(&format!("ncf:federated-clearing:work:{digest}"))
}
fn settlement_anchor(id: &str) -> ExternResult<EntryHash> {
    anchor_hash(&format!("ncf:federated-clearing:settlement:{id}"))
}

fn current_agent() -> ExternResult<AgentPubKey> {
    Ok(agent_info()?.agent_initial_pubkey)
}

fn linked_records<T>(
    anchor: EntryHash,
    link_type: LinkTypes,
) -> ExternResult<Vec<(ActionHash, T)>>
where
    T: TryFrom<SerializedBytes, Error = SerializedBytesError> + Clone,
{
    let links = get_links(LinkQuery::try_new(anchor, link_type)?, GetStrategy::default())?;
    let mut found = Vec::new();
    for link in links {
        let Some(action_hash) = link.target.clone().into_action_hash() else { continue; };
        let Some(record) = get(action_hash.clone(), GetOptions::default())? else {
            // A published index whose target is not available is not permission to
            // author a second claim. Return what is available and let reconciliation retry.
            continue;
        };
        if let Some(entry) = record.entry().to_app_option::<T>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("indexed entry decode failed: {e}"))))? {
            found.push((action_hash, entry));
        }
    }
    Ok(found)
}

fn receipt_for<T>(action_hash: ActionHash, entry: &T, event_id: String, replayed: bool) -> ExternResult<FederatedPublicationReceipt>
where
    T: TryInto<SerializedBytes> + Clone,
    <T as TryInto<SerializedBytes>>::Error: std::fmt::Display,
{
    let entry_hash = hash_entry(entry)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("entry hash failed: {e}"))))?;
    Ok(FederatedPublicationReceipt { replayed, action_hash, entry_hash, event_id })
}

#[hdk_extern]
pub fn publish_reservation_manifest(manifest: FederatedReservationManifest) -> ExternResult<FederatedPublicationReceipt> {
    if current_agent()? != manifest.home_agent {
        return fail("only the designated home agent may publish this reservation manifest");
    }
    let existing = linked_records::<FederatedReservationManifest>(
        manifest_anchor(&manifest.manifest_id)?, LinkTypes::ReservationManifestById
    )?;
    if !existing.is_empty() {
        for (action_hash, previous) in existing {
            if previous == manifest {
                return receipt_for(action_hash, &manifest, manifest.manifest_id.clone(), true);
            }
            return fail("manifest ID already exists with conflicting immutable contents");
        }
    }

    let action_hash = create_entry(&EntryTypes::FederatedReservationManifest(manifest.clone()))?;
    create_link(
        manifest_anchor(&manifest.manifest_id)?,
        action_hash.clone(),
        LinkTypes::ReservationManifestById,
        (),
    )?;
    receipt_for(action_hash, &manifest, manifest.manifest_id.clone(), false)
}

#[hdk_extern]
pub fn publish_obligation_claim(claim: FederatedObligationClaim) -> ExternResult<FederatedPublicationReceipt> {
    if current_agent()? != claim.home_agent {
        return fail("only the designated home agent may publish this obligation claim");
    }
    let manifest_record = get(claim.reservation_manifest_action_hash.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("reservation manifest is not locally available; retry after dependency arrives".into())))?;
    let manifest = manifest_record.entry().to_app_option::<FederatedReservationManifest>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("reservation manifest decode failed: {e}"))))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("referenced action is not a federated reservation manifest".into())))?;
    if claim.home_agent != manifest.home_agent
        || claim.community_did != manifest.community_did
        || claim.instrument_id != manifest.instrument_id
        || claim.profile_revision != manifest.profile_revision
        || claim.agreement_id != manifest.agreement_id
        || claim.direction != manifest.direction
        || claim.shard_id != manifest.shard_id
        || claim.epoch != manifest.epoch {
        return fail("obligation scope differs from its reservation manifest");
    }

    let existing_event = linked_records::<FederatedObligationClaim>(
        event_anchor(&claim.event_id)?, LinkTypes::ObligationClaimByEventId
    )?;
    for (action_hash, previous) in existing_event {
        if previous == claim {
            return receipt_for(action_hash, &claim, claim.event_id.clone(), true);
        }
        return fail("outbox event ID already exists with conflicting immutable contents");
    }

    // Canonical-work reuse is never a new obligation. SQL is the authority for
    // global uniqueness in its coordinator domain; this DHT index is a receipt/
    // reconciliation aid, not a cross-agent global unique constraint.
    let existing_work = linked_records::<FederatedObligationClaim>(
        work_anchor(&claim.canonical_work_digest_hex)?, LinkTypes::ObligationClaimByCanonicalWork
    )?;
    for (action_hash, previous) in existing_work {
        if previous.canonical_work_digest_hex != claim.canonical_work_digest_hex
            || previous.principal_did != claim.principal_did
            || previous.community_did != claim.community_did
            || previous.instrument_id != claim.instrument_id
            || previous.profile_revision != claim.profile_revision
            || previous.amount_units != claim.amount_units
            || previous.agreement_id != claim.agreement_id
            || previous.direction != claim.direction {
            return fail("canonical work already has a conflicting obligation claim");
        }
        return receipt_for(action_hash, &previous, claim.event_id.clone(), true);
    }

    let action_hash = create_entry(&EntryTypes::FederatedObligationClaim(claim.clone()))?;
    create_link(event_anchor(&claim.event_id)?, action_hash.clone(), LinkTypes::ObligationClaimByEventId, ())?;
    create_link(work_anchor(&claim.canonical_work_digest_hex)?, action_hash.clone(), LinkTypes::ObligationClaimByCanonicalWork, ())?;
    receipt_for(action_hash, &claim, claim.event_id.clone(), false)
}

#[hdk_extern]
pub fn publish_settlement_evidence(settlement: FederatedSettlementEvidence) -> ExternResult<FederatedPublicationReceipt> {
    if current_agent()? != settlement.home_agent {
        return fail("only the designated home agent may publish settlement evidence in profile v1");
    }
    let existing = linked_records::<FederatedSettlementEvidence>(
        settlement_anchor(&settlement.settlement_id)?, LinkTypes::SettlementEvidenceById
    )?;
    for (action_hash, previous) in existing {
        if previous == settlement {
            return receipt_for(action_hash, &settlement, settlement.event_id.clone(), true);
        }
        return fail("settlement ID already exists with conflicting immutable contents");
    }

    let claim_record = get(settlement.obligation_claim_action_hash.clone(), GetOptions::default())?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("obligation claim is not locally available; retry after dependency arrives".into())))?;
    let claim = claim_record.entry().to_app_option::<FederatedObligationClaim>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("obligation claim decode failed: {e}"))))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("referenced action is not an obligation claim".into())))?;
    if claim.community_did != settlement.community_did
        || claim.instrument_id != settlement.instrument_id
        || claim.direction != settlement.direction
        || claim.amount_units != settlement.amount_units
        || claim.home_agent != settlement.home_agent {
        return fail("settlement evidence differs from the canonical obligation claim");
    }

    let action_hash = create_entry(&EntryTypes::FederatedSettlementEvidence(settlement.clone()))?;
    create_link(settlement_anchor(&settlement.settlement_id)?, action_hash.clone(), LinkTypes::SettlementEvidenceById, ())?;
    receipt_for(action_hash, &settlement, settlement.event_id.clone(), false)
}

#[hdk_extern]
pub fn get_federated_claim_by_event_id(event_id: String) -> ExternResult<Option<Record>> {
    let links = get_links(LinkQuery::try_new(event_anchor(&event_id)?, LinkTypes::ObligationClaimByEventId)?, GetStrategy::default())?;
    for link in links {
        let Some(action_hash) = link.target.into_action_hash() else { continue; };
        if let Some(record) = get(action_hash, GetOptions::default())? {
            if let Some(claim) = record.entry().to_app_option::<FederatedObligationClaim>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("claim decode failed: {e}"))))? {
                if claim.event_id == event_id { return Ok(Some(record)); }
            }
        }
    }
    Ok(None)
}

#[hdk_extern]
pub fn get_reservation_manifest(manifest_id: String) -> ExternResult<Option<Record>> {
    let links = get_links(LinkQuery::try_new(manifest_anchor(&manifest_id)?, LinkTypes::ReservationManifestById)?, GetStrategy::default())?;
    for link in links {
        let Some(action_hash) = link.target.into_action_hash() else { continue; };
        if let Some(record) = get(action_hash, GetOptions::default())? {
            if let Some(m) = record.entry().to_app_option::<FederatedReservationManifest>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("manifest decode failed: {e}"))))? {
                if m.manifest_id == manifest_id { return Ok(Some(record)); }
            }
        }
    }
    Ok(None)
}

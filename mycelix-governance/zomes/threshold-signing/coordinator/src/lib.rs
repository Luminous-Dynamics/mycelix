// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Threshold Signing Coordinator Zome
//!
//! Business logic for DKG-based threshold signatures on governance decisions.
//!
//! Workflow:
//! 1. Create committee with threshold and member count
//! 2. Members register with their K-Vector trust scores
//! 3. Members run DKG ceremony off-chain using feldman-dkg
//! 4. Public commitments are submitted to advance ceremony
//! 5. Once complete, members can collectively sign governance decisions
//! 6. Threshold signatures are verified and stored

use hdk::prelude::*;
use mycelix_zome_helpers as _;
use threshold_signing_integrity::*;


/// Retrieve the latest committee record addressed by deterministic committee ID.
///
/// The returned record is read-only DHT data; no state is mutated.
#[hdk_extern]
pub fn get_committee(committee_id: String) -> ExternResult<Option<Record>> {
    if committee_id.is_empty() || committee_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Committee ID must be 1-256 characters".into()
        )));
    }

    // Anchor lookup is itself an entry hash, so the address can be reconstructed
    // from public input. We search current DHT links from the deterministic anchor.
    let anchor = format!("committee:{}", committee_id);
    let anchor_entry_hash = hash_entry(Anchor(anchor.clone()))
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;

    let links = get_links(
        LinkQuery::try_new(anchor_entry_hash, LinkTypes::CommitteeById)?,
        GetStrategy::default(),
    )?;

    let link = links.into_iter().max_by_key(|link| link.timestamp);
    let Some(link) = link else {
        return Ok(None);
    };

    let action_hash = ActionHash::try_from(link.target)
        .map_err(|_| wasm_error!(WasmErrorInner::Guest(
            "Invalid committee link target".into()
        )))?;

    get(action_hash, GetOptions::default())
}

/// Retrieve the latest threshold signature linked to a proposal.
///
/// This is intentionally read-only. Signature creation and cryptographic
/// verification remain separate boundaries.
#[hdk_extern]
pub fn get_proposal_signature(proposal_id: String) -> ExternResult<Option<Record>> {
    if proposal_id.is_empty() || proposal_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal ID must be 1-256 characters".into()
        )));
    }

    let proposal_anchor_hash = hash_entry(Anchor(format!(
        "proposal-signature:{}",
        proposal_id
    )))
    .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;

    let links = get_links(
        LinkQuery::try_new(proposal_anchor_hash, LinkTypes::ProposalToSignature)?,
        GetStrategy::default(),
    )?;

    let link = links.into_iter().max_by_key(|link| link.timestamp);
    let Some(link) = link else {
        return Ok(None);
    };

    let action_hash = ActionHash::try_from(link.target)
        .map_err(|_| wasm_error!(WasmErrorInner::Guest(
            "Invalid proposal signature link target".into()
        )))?;

    get(action_hash, GetOptions::default())
}

// ============================================================================
// REAL-TIME SIGNALS
// ============================================================================

#[derive(Serialize, Deserialize, Debug)]
#[serde(tag = "type")]
pub enum ThresholdSignal {
    CommitteeCreated {
        committee_id: String,
    },
    MemberRegistered {
        committee_id: String,
        member_did: String,
    },
    CeremonyAdvanced {
        committee_id: String,
        stage: u8,
    },
    DecisionSigned {
        decision_hash: ActionHash,
    },
}

fn deterministic_anchor_hash(
    anchor: &str,
) -> ExternResult<EntryHash> {
    let action_hash = create_entry(&EntryTypes::Anchor(Anchor(anchor.to_owned())))?;
    let record = get(action_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Could not read deterministic anchor after creation".into())
    ))?;
    record
        .entry()
        .as_option()
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Deterministic anchor record has no entry".into()
        )))?
        .as_entry_hash()
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Deterministic anchor record is not an entry".into()
        )))
}

// ── Committee Management ─────────────────────────────────────────────────────

/// Input for creating a new signing committee.
#[derive(Serialize, Deserialize, Debug)]
pub struct CreateCommitteeInput {
    pub committee_id: String,
    pub name: String,
    pub threshold: u32,
    pub member_count: u32,
    pub scope: CommitteeScope,
    pub min_phi: Option<f64>,
    pub signature_algorithm: ThresholdSignatureAlgorithm,
    pub pq_required: bool,
}

#[hdk_extern]
pub fn create_committee(input: CreateCommitteeInput) -> ExternResult<Record> {
    let committee = SigningCommittee {
        id: input.committee_id.clone(),
        name: input.name,
        threshold: input.threshold,
        member_count: input.member_count,
        phase: DkgPhase::Registration,
        public_key: None,
        commitments: Vec::new(),
        scope: input.scope,
        created_at: sys_time()?,
        active: true,
        epoch: 1,
        min_phi: input.min_phi,
        signature_algorithm: input.signature_algorithm,
        pq_required: input.pq_required,
    };

    let action_hash = create_entry(&EntryTypes::SigningCommittee(committee))?;

    let anchor_action_hash = create_entry(&EntryTypes::Anchor(Anchor(
        format!("committee:{}", input.committee_id)
    )))?;
    let anchor_record = get(anchor_action_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Committee anchor could not be read after creation".into()
        )))?;
    let committee_anchor = anchor_record
        .entry()
        .to_app_option::<Anchor>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Committee anchor entry could not be decoded".into()
        )))?;
    let _ = committee_anchor;

    // Resolve the entry hash of the newly-created anchor record without relying
    // on a caller-supplied lookup value.
    let committee_anchor_hash = {
        let mut found: Option<EntryHash> = None;
        if let Some(record) = get(anchor_action_hash, GetOptions::default())? {
            if let Some(entry) = record.entry().as_option() {
                found = entry.as_entry_hash();
            }
        }
        found.ok_or(wasm_error!(WasmErrorInner::Guest(
            "Committee anchor is not an entry hash".into()
        )))?
    };

    create_link(
        committee_anchor_hash,
        action_hash.clone(),
        LinkTypes::CommitteeById,
        (),
    )?;

    let _ = emit_signal(ThresholdSignal::CommitteeCreated {
        committee_id: input.committee_id,
    });

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Committee record not found".into()
    )))
}

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
use k256::ecdsa::{signature::hazmat::PrehashVerifier, Signature, VerifyingKey};

/// Deterministic hash for a threshold-signing integrity anchor.
fn anchor_hash(anchor: &str) -> ExternResult<EntryHash> {
    hash_entry(EntryTypes::Anchor(Anchor(anchor.to_owned())))
}


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

    let anchor_entry_hash = anchor_hash(&format!("committee:{}", committee_id))?;

    let links = get_links(
        LinkQuery::try_new(anchor_entry_hash, LinkTypes::CommitteeById)?,
        GetStrategy::default(),
    )?;

    if links.len() > 1 {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Ambiguous committee ID '{}': {} committee records are linked to the deterministic ID anchor.",
            committee_id,
            links.len()
        ))));
    }

    let Some(link) = links.into_iter().next() else {
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

    let proposal_anchor_hash = anchor_hash(&format!("proposal-signature:{}", proposal_id))?;

    let links = get_links(
        LinkQuery::try_new(proposal_anchor_hash, LinkTypes::ProposalToSignature)?,
        GetStrategy::default(),
    )?;

    if links.len() > 1 {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Ambiguous proposal signature '{}': {} signature records are linked to the deterministic proposal anchor.",
            proposal_id,
            links.len()
        ))));
    }

    let Some(link) = links.into_iter().next() else {
        return Ok(None);
    };

    let action_hash = ActionHash::try_from(link.target)
        .map_err(|_| wasm_error!(WasmErrorInner::Guest(
            "Invalid proposal signature link target".into()
        )))?;

    get(action_hash, GetOptions::default())
}

fn authorization_digest(proposal_id: &str, action_key_digest: &str) -> [u8; 32] {
    let mut hasher = blake3::Hasher::new();
    hasher.update(b"MYCELIX-GOVERNANCE-EXECUTION-AUTHORIZATION\0V2\0");
    for value in [proposal_id, action_key_digest] {
        hasher.update(&(value.len() as u64).to_be_bytes());
        hasher.update(value.as_bytes());
    }
    *hasher.finalize().as_bytes()
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifyProposalSignatureInput {
    pub proposal_id: String,
    pub action_key_digest: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ProposalSignatureVerification {
    pub proposal_id: String,
    pub action_key_digest: String,
    pub signature_id: String,
    pub committee_id: String,
    pub signer_count: u32,
    pub threshold: u32,
    pub algorithm: ThresholdSignatureAlgorithm,
    pub verified: bool,
}

/// Cryptographically verify the threshold authorization for one exact proposal material action.
///
/// This function is read-only. It never mutates committee/signature state and never trusts
/// the ThresholdSignature.verified bit. The returned receipt means the checked signature,
/// committee configuration, signer metadata, and exact authorization digest all passed.
#[hdk_extern]
pub fn verify_proposal_signature(
    input: VerifyProposalSignatureInput,
) -> ExternResult<ProposalSignatureVerification> {
    if input.proposal_id.is_empty() || input.proposal_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal ID must be 1-256 characters".into()
        )));
    }
    if input.action_key_digest.trim().is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Action key digest is required".into()
        )));
    }

    let signature_record = get_proposal_signature(input.proposal_id.clone())?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "No threshold signature is indexed for this proposal".into()
        )))?;

    let signature: ThresholdSignature = signature_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Indexed threshold signature has no entry".into()
        )))?;

    let (kind, signed_id) = signature
        .signed_content_description
        .split_once(':')
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Threshold signature content description is not structurally proposal-bound".into()
        )))?;

    if signed_id != input.proposal_id {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Threshold signature targets a different proposal ID".into()
        )));
    }
    if !matches!(kind, "proposal" | "constitutional" | "treasury" | "protocol") {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Threshold signature has an unsupported proposal kind".into()
        )));
    }

    let expected_hash = authorization_digest(
        &input.proposal_id,
        &input.action_key_digest,
    );
    if signature.signed_content_hash.as_slice() != expected_hash {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Threshold signature is not bound to the exact approved proposal action".into()
        )));
    }

    let committee_record = get_committee(signature.committee_id.clone())?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Threshold signature references an unavailable committee".into()
        )))?;

    let committee: SigningCommittee = committee_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Committee record has no SigningCommittee entry".into()
        )))?;

    if !committee.active || committee.phase != DkgPhase::Complete {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Signing committee is not active and complete".into()
        )));
    }

    if committee.threshold == 0
        || committee.member_count == 0
        || committee.threshold > committee.member_count
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Signing committee threshold/member configuration is invalid".into()
        )));
    }

    if committee.signature_algorithm != signature.signature_algorithm {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Threshold signature algorithm does not match committee configuration".into()
        )));
    }

    if committee.pq_required
        && !matches!(
            signature.signature_algorithm,
            ThresholdSignatureAlgorithm::MlDsa65
                | ThresholdSignatureAlgorithm::HybridEcdsaMlDsa65
        )
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Committee requires post-quantum authorization but signature is not PQ/hybrid".into()
        )));
    }

    if signature.signer_count < committee.threshold
        || signature.signer_count as usize != signature.signers.len()
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Threshold signature signer metadata does not satisfy committee threshold".into()
        )));
    }

    let mut unique_signers = std::collections::BTreeSet::new();
    for signer in &signature.signers {
        if *signer == 0 || *signer > committee.member_count {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Signer {} is outside committee membership range",
                signer
            ))));
        }
        if !unique_signers.insert(*signer) {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Duplicate signer {} in threshold signature",
                signer
            ))));
        }
    }

    match signature.signature_algorithm {
        ThresholdSignatureAlgorithm::Ecdsa => {
            let public_key = committee.public_key.as_deref().ok_or(wasm_error!(
                WasmErrorInner::Guest(
                    "ECDSA verification requires committee public key".into()
                )
            ))?;

            if public_key.len() != 33 {
                return Err(wasm_error!(WasmErrorInner::Guest(format!(
                    "Committee ECDSA public key must be 33-byte compressed SEC1, got {}",
                    public_key.len()
                ))));
            }

            let verifying_key = VerifyingKey::from_sec1_bytes(public_key)
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!(
                    "Invalid committee ECDSA public key: {e}"
                ))))?;

            if signature.signature.len() != 64 {
                return Err(wasm_error!(WasmErrorInner::Guest(format!(
                    "ECDSA signature must be exactly 64 raw r||s bytes, got {}",
                    signature.signature.len()
                ))));
            }

            let sig = Signature::from_slice(&signature.signature)
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!(
                    "Invalid ECDSA signature encoding: {e}"
                ))))?;

            verifying_key
                .verify_prehash(&signature.signed_content_hash, &sig)
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!(
                    "ECDSA threshold authorization verification failed: {e}"
                ))))?;
        }
        ThresholdSignatureAlgorithm::MlDsa65
        | ThresholdSignatureAlgorithm::HybridEcdsaMlDsa65 => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "PQ/hybrid threshold authorization verifier is not wired".into()
            )));
        }
    }

    Ok(ProposalSignatureVerification {
        proposal_id: input.proposal_id,
        action_key_digest: input.action_key_digest,
        signature_id: signature.id,
        committee_id: signature.committee_id,
        signer_count: signature.signer_count,
        threshold: committee.threshold,
        algorithm: signature.signature_algorithm,
        verified: true,
    })
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

    let committee_anchor = Anchor(format!("committee:{}", input.committee_id));
    let committee_anchor_hash = anchor_hash(committee_anchor.0.as_str())?;
    let _anchor_action_hash = create_entry(&EntryTypes::Anchor(committee_anchor))?;

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

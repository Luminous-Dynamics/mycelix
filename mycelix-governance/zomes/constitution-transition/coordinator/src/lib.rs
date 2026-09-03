// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed constitutional transition orchestration.
//! Stored entries are candidates, never authority by existence.

use constitution_transition_integrity::{
    parent_anchor_name, Anchor, ConstitutionTransitionCandidate, EntryTypes, LinkTypes,
};
use hdk::prelude::*;
use mycelix_governance_constitution::{
    project_verified_lineage, ConstitutionGenesisManifest, ConstitutionStatement,
    ConstitutionTransitionAuthorization, Digest32, TransitionVerificationEvidence,
    VerifiedConstitutionTransition, GENESIS_MANIFEST_PROFILE, STATEMENT_PROFILE,
};
use serde::de::DeserializeOwned;
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};

const RIGHTS_VERIFIER_ZOME: &str = "governance_rights_verifier";
const THRESHOLD_VERIFIER_ZOME: &str = "governance_threshold_authority_verifier";
const MAX_REF_BYTES: usize = 2048;

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedConstitutionGenesisMirror {
    dna_hash: String,
    manifest_digest: Digest32,
    manifest_digest_profile: String,
    statement_digest: Digest32,
    statement_digest_profile: String,
    statement: ConstitutionStatement,
    amendments_enabled: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyConstitutionalTallyRequest {
    proposal_id: String,
    tally_action: ActionHash,
    expected_digest: Digest32,
    expected_profile: String,
    transition_authorization_digest: Digest32,
    parent_statement_digest: Digest32,
    child_statement_digest: Digest32,
    transition_nonce: Digest32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyConstitutionalTallyResponse {
    verified: bool,
    proposal_id: String,
    tally_action: ActionHash,
    digest: Digest32,
    profile: String,
    transition_authorization_digest: Digest32,
    receipt_ref: String,
    verified_at_ms: u64,
    reason: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyThresholdAuthorizationRequest {
    proposal_id: String,
    expected_digest: Digest32,
    expected_profile: String,
    transition_authorization_digest: Digest32,
    parent_statement_digest: Digest32,
    child_statement_digest: Digest32,
    transition_nonce: Digest32,
    authorized_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyThresholdAuthorizationResponse {
    verified: bool,
    proposal_id: String,
    digest: Digest32,
    profile: String,
    transition_authorization_digest: Digest32,
    transition_nonce: Digest32,
    receipt_ref: String,
    verified_at_ms: u64,
    reason: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SubmitTransitionCandidateInput {
    pub child: ConstitutionStatement,
    pub authorization: ConstitutionTransitionAuthorization,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifiedCurrentConstitution {
    pub dna_hash: String,
    pub statement: ConstitutionStatement,
    pub statement_digest: Digest32,
    /// Canonical verified authorizations consumed along the projected lineage.
    pub verified_transition_count: u64,
    /// Unique candidate records inspected only on parent indexes reached by the lineage.
    pub candidate_count: u64,
    pub legacy_constitution_authoritative: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ConstitutionTransitionStatus {
    pub protocol: String,
    pub transition_contract_installed: bool,
    pub amendments_operational: bool,
    pub required_rights_verifier_zome: String,
    pub required_threshold_verifier_zome: String,
    pub legacy_constitution_authoritative: bool,
    pub note: String,
}

fn call_local<I, O>(zome: &str, function: &str, input: I) -> ExternResult<O>
where
    I: Serialize,
    O: DeserializeOwned,
{
    let response = call(
        CallTargetCell::Local,
        ZomeName::from(zome),
        FunctionName::from(function),
        None,
        ExternIO::encode(input)
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
    )?;
    let io = match response {
        ZomeCallResponse::Ok(io) => io,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "{zome}::{function} unavailable; constitutional governance fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot decode {zome}::{function} response: {e}"
        )))
    })
}

fn require_ref(value: &str, field: &str) -> ExternResult<()> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} must be 1-{MAX_REF_BYTES} bytes"
        ))));
    }
    Ok(())
}

fn parent_anchor_hash(parent: Digest32) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(parent_anchor_name(parent))))
}

fn manifest_from_genesis(
    statement: &ConstitutionStatement,
) -> ExternResult<ConstitutionGenesisManifest> {
    if statement.version != 1 || statement.parent_statement_digest.is_some() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Constitution authority did not return a genesis statement".into(),
        )));
    }
    let manifest = ConstitutionGenesisManifest {
        protocol_version: statement.protocol_version.clone(),
        network_id: statement.network_id.clone(),
        institution_id: statement.institution_id.clone(),
        constitution_id: statement.constitution_id.clone(),
        rulebook_id: statement.rulebook_id.clone(),
        rulebook_version: statement.rulebook_version.clone(),
        rulebook: statement.rulebook.clone(),
        charter: statement.charter.clone(),
        parameters: statement.parameters.clone(),
        amendment_policy: statement.amendment_policy.clone(),
        binding_vote_profile: statement.binding_vote_profile.clone(),
        threshold_authority_profile: statement.threshold_authority_profile.clone(),
        effective_from_ms: statement.effective_from_ms,
    };
    manifest.verify_genesis_statement(statement).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Verified genesis cannot be reconstructed as a manifest: {e}"
        )))
    })?;
    Ok(manifest)
}

fn load_genesis() -> ExternResult<VerifiedConstitutionGenesisMirror> {
    let genesis: VerifiedConstitutionGenesisMirror = call_local(
        "constitution_authority",
        "get_verified_constitution_genesis",
        (),
    )?;
    if genesis.amendments_enabled {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Genesis authority unexpectedly claims amendments are already enabled".into(),
        )));
    }
    if genesis.dna_hash.trim().is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Constitution authority returned an empty DNA hash".into(),
        )));
    }
    if genesis.manifest_digest_profile != GENESIS_MANIFEST_PROFILE
        || genesis.statement_digest_profile != STATEMENT_PROFILE
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Constitution authority returned an unexpected canonical digest profile".into(),
        )));
    }
    let statement_digest = genesis.statement.digest().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot digest verified genesis statement: {e}"
        )))
    })?;
    if statement_digest != genesis.statement_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Constitution authority returned an inconsistent genesis statement digest".into(),
        )));
    }
    let manifest = manifest_from_genesis(&genesis.statement)?;
    let manifest_digest = manifest.digest().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot digest reconstructed genesis manifest: {e}"
        )))
    })?;
    if manifest_digest != genesis.manifest_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Constitution authority returned an inconsistent genesis manifest digest".into(),
        )));
    }
    Ok(genesis)
}

fn decode_candidate(record: &Record) -> Option<ConstitutionTransitionCandidate> {
    record
        .entry()
        .to_app_option::<ConstitutionTransitionCandidate>()
        .ok()
        .flatten()
}

/// Resolve only candidates indexed under one exact parent statement. Repeated
/// links to the same action are deduplicated before any record/verifier work.
fn list_candidate_records_for_parent(parent: Digest32) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(
            parent_anchor_hash(parent)?,
            LinkTypes::ParentToTransitionCandidate,
        )?,
        GetStrategy::default(),
    )?;
    let mut targets = BTreeMap::<String, ActionHash>::new();
    for link in links {
        if let Ok(action_hash) = ActionHash::try_from(link.target) {
            targets
                .entry(action_hash.to_string())
                .or_insert(action_hash);
        }
    }
    let mut records = Vec::with_capacity(targets.len());
    for (_, action_hash) in targets {
        if let Some(record) = get(action_hash, GetOptions::default())? {
            records.push(record);
        }
    }
    Ok(records)
}

fn verify_external_evidence(
    candidate: &ConstitutionTransitionCandidate,
) -> ExternResult<Option<VerifiedConstitutionTransition>> {
    candidate.validate_structure().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Stored transition candidate is structurally invalid: {e}"
        )))
    })?;

    let authorization_digest = candidate.authorization.digest().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot digest constitutional authorization: {e}"
        )))
    })?;
    let proposal_id = candidate.authorization.proposal_id.as_str().to_string();

    // Re-resolve the exact proposal authority binding used for cheap admission.
    let verified_authority: Option<Record> = call_local(
        "proposal_authority",
        "get_verified_proposal_authority_context",
        proposal_id.clone(),
    )?;
    let Some(verified_authority) = verified_authority else {
        return Ok(None);
    };
    if *verified_authority.action_address() != candidate.proposal_authority_binding {
        return Ok(None);
    }

    let tally_record: Option<Record> = call_local(
        "binding_voting",
        "get_verified_binding_tally",
        proposal_id.clone(),
    )?;
    let Some(tally_record) = tally_record else {
        return Ok(None);
    };

    let tally: VerifyConstitutionalTallyResponse = call_local(
        RIGHTS_VERIFIER_ZOME,
        "verify_constitutional_binding_tally",
        VerifyConstitutionalTallyRequest {
            proposal_id: proposal_id.clone(),
            tally_action: tally_record.action_address().clone(),
            expected_digest: candidate.authorization.binding_tally.digest,
            expected_profile: candidate.authorization.binding_tally.profile.clone(),
            transition_authorization_digest: authorization_digest,
            parent_statement_digest: candidate.authorization.from_statement_digest,
            child_statement_digest: candidate.authorization.to_statement_digest,
            transition_nonce: candidate.authorization.transition_nonce,
        },
    )?;
    if !tally.verified {
        let _ = tally.reason.as_deref();
        return Ok(None);
    }
    require_ref(&tally.receipt_ref, "binding tally verification receipt")?;
    if tally.proposal_id != proposal_id
        || tally.tally_action != *tally_record.action_address()
        || tally.digest != candidate.authorization.binding_tally.digest
        || tally.profile != candidate.authorization.binding_tally.profile
        || tally.transition_authorization_digest != authorization_digest
        || tally.verified_at_ms < candidate.authorization.authorized_at_ms
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Binding tally verifier response does not exactly bind the constitutional transition"
                .into(),
        )));
    }

    let threshold: VerifyThresholdAuthorizationResponse = call_local(
        THRESHOLD_VERIFIER_ZOME,
        "verify_constitutional_threshold_authorization",
        VerifyThresholdAuthorizationRequest {
            proposal_id: proposal_id.clone(),
            expected_digest: candidate.authorization.threshold_authorization.digest,
            expected_profile: candidate.authorization.threshold_authorization.profile.clone(),
            transition_authorization_digest: authorization_digest,
            parent_statement_digest: candidate.authorization.from_statement_digest,
            child_statement_digest: candidate.authorization.to_statement_digest,
            transition_nonce: candidate.authorization.transition_nonce,
            authorized_at_ms: candidate.authorization.authorized_at_ms,
        },
    )?;
    if !threshold.verified {
        let _ = threshold.reason.as_deref();
        return Ok(None);
    }
    require_ref(
        &threshold.receipt_ref,
        "threshold authorization verification receipt",
    )?;
    if threshold.proposal_id != proposal_id
        || threshold.digest != candidate.authorization.threshold_authorization.digest
        || threshold.profile != candidate.authorization.threshold_authorization.profile
        || threshold.transition_authorization_digest != authorization_digest
        || threshold.transition_nonce != candidate.authorization.transition_nonce
        || threshold.verified_at_ms < candidate.authorization.authorized_at_ms
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Threshold verifier response does not exactly bind the constitutional transition"
                .into(),
        )));
    }

    let verification = TransitionVerificationEvidence {
        authorization_digest,
        binding_tally_verification_ref: tally.receipt_ref,
        threshold_verification_ref: threshold.receipt_ref,
        verified_at_ms: tally.verified_at_ms.max(threshold.verified_at_ms),
    };
    verification
        .validate_against(&candidate.authorization)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;

    Ok(Some(VerifiedConstitutionTransition {
        child: candidate.child.clone(),
        authorization: candidate.authorization.clone(),
        verification,
    }))
}

#[hdk_extern]
pub fn submit_constitution_transition_candidate(
    input: SubmitTransitionCandidateInput,
) -> ExternResult<Record> {
    input.child.validate().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid child constitution statement: {e}"
        )))
    })?;
    input.authorization.validate().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid transition authorization: {e}"
        )))
    })?;
    let child_digest = input.child.digest().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot digest child constitution: {e}"
        )))
    })?;
    if child_digest != input.authorization.to_statement_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Child statement does not match transition authorization target".into(),
        )));
    }

    let proposal_id = input.authorization.proposal_id.as_str().to_string();
    let authority_record: Option<Record> = call_local(
        "proposal_authority",
        "get_verified_proposal_authority_context",
        proposal_id,
    )?;
    let authority_record = authority_record.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Transition candidate requires a verified proposal authority context".into(),
        ))
    })?;

    let now = sys_time()?;
    let caller = agent_info()?.agent_initial_pubkey;
    let submitted_by = format!("did:mycelix:{caller}");
    let authorization_digest = input.authorization.digest().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot digest transition authorization: {e}"
        )))
    })?;
    let parent_digest = input.authorization.from_statement_digest;
    let candidate = ConstitutionTransitionCandidate {
        id: format!(
            "constitution-transition:{}:{}:{}",
            input.child.constitution_id.as_str(),
            input.child.version,
            authorization_digest.to_hex()
        ),
        child: input.child,
        authorization: input.authorization,
        proposal_authority_binding: authority_record.action_address().clone(),
        submitted_by,
        submitted_at: now,
    };
    candidate
        .validate_structure()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;

    let action_hash = create_entry(&EntryTypes::ConstitutionTransitionCandidate(candidate))?;
    let anchor = Anchor(parent_anchor_name(parent_digest));
    create_entry(&EntryTypes::Anchor(anchor.clone()))?;
    create_link(
        hash_entry(&EntryTypes::Anchor(anchor))?,
        action_hash.clone(),
        LinkTypes::ParentToTransitionCandidate,
        authorization_digest.0.to_vec(),
    )?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve constitutional transition candidate".into(),
        ))
    })
}

#[hdk_extern]
pub fn get_verified_current_constitution(_: ()) -> ExternResult<VerifiedCurrentConstitution> {
    let genesis = load_genesis()?;
    let manifest = manifest_from_genesis(&genesis.statement)?;
    let mut current = manifest.genesis_statement().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot derive constitutional genesis statement: {e}"
        )))
    })?;

    let mut verified_history: Vec<VerifiedConstitutionTransition> = Vec::new();
    let mut nonce_bindings: BTreeMap<String, Digest32> = BTreeMap::new();
    let mut candidate_count = 0u64;

    loop {
        let parent_digest = current.digest().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Cannot digest current constitution: {e}"
            )))
        })?;
        let records = list_candidate_records_for_parent(parent_digest)?;
        candidate_count = candidate_count.saturating_add(records.len() as u64);
        if records.is_empty() {
            break;
        }

        let mut by_authorization: BTreeMap<String, VerifiedConstitutionTransition> =
            BTreeMap::new();
        for record in records {
            let Some(candidate) = decode_candidate(&record) else {
                continue;
            };
            // Defense in depth against malformed or historical indexes.
            if candidate.authorization.from_statement_digest != parent_digest {
                continue;
            }
            let Some(transition) = verify_external_evidence(&candidate)? else {
                continue;
            };
            let authorization_digest = transition.authorization.digest().map_err(|e| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Cannot digest verified transition authorization: {e}"
                )))
            })?;
            let authorization_key = authorization_digest.to_hex();
            let nonce_key = transition.authorization.transition_nonce.to_hex();

            if let Some(existing_digest) = nonce_bindings.get(&nonce_key) {
                if *existing_digest != authorization_digest {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "Replay nonce is bound to multiple verified constitutional authorizations; fail closed"
                            .into(),
                    )));
                }
            } else {
                nonce_bindings.insert(nonce_key, authorization_digest);
            }

            if let Some(existing) = by_authorization.get(&authorization_key) {
                if existing.child != transition.child
                    || existing.authorization != transition.authorization
                {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "One authorization digest resolves to conflicting constitutional semantics; fail closed"
                            .into(),
                    )));
                }
                continue;
            }
            by_authorization.insert(authorization_key, transition);
        }

        if by_authorization.is_empty() {
            break;
        }
        verified_history.extend(by_authorization.into_values());
        let next = project_verified_lineage(&manifest, &verified_history).map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Cannot project verified constitutional lineage: {e}"
            )))
        })?;
        let next_digest = next.digest().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Cannot digest projected constitution: {e}"
            )))
        })?;
        if next_digest == parent_digest {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Verified transition set did not advance constitutional lineage; fail closed".into(),
            )));
        }
        current = next;
    }

    let statement_digest = current.digest().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot digest projected constitution: {e}"
        )))
    })?;
    Ok(VerifiedCurrentConstitution {
        dna_hash: genesis.dna_hash,
        statement: current,
        statement_digest,
        verified_transition_count: verified_history.len() as u64,
        candidate_count,
        legacy_constitution_authoritative: false,
    })
}

#[hdk_extern]
pub fn get_constitution_transition_status(_: ()) -> ExternResult<ConstitutionTransitionStatus> {
    Ok(ConstitutionTransitionStatus {
        protocol: "mycelix-governance-constitution-transition-v0.1".into(),
        transition_contract_installed: true,
        amendments_operational: false,
        required_rights_verifier_zome: RIGHTS_VERIFIER_ZOME.into(),
        required_threshold_verifier_zome: THRESHOLD_VERIFIER_ZOME.into(),
        legacy_constitution_authoritative: false,
        note: "Transition contract is installed, but amendment authority remains disabled until both verifier services and adversarial release gates are implemented.".into(),
    })
}

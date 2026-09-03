// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Binding governance voting runtime.
//!
//! Authority chain:
//!
//! verified proposal authority context
//! -> verified electorate + tally + finality policy
//! -> author-bound, score-independent ballot
//! -> frozen ballot-set finality checkpoint
//! -> deterministic raw-count tally
//!
//! No Phi, reputation, stake, quadratic weight, model score, or advisory signal
//! is accepted anywhere in the binding ballot/tally API.

use binding_voting_integrity::*;
use hdk::prelude::*;
use mycelix_governance_authority::ProposalAuthorityContext;
use mycelix_governance_electorate::{
    BallotQualification, ElectorateMembershipEvidence, ElectorateSnapshot,
    ElectorateSnapshotId, RevocationSemantics, SnapshotBoundTally, tally_snapshot_ballots,
    PROTOCOL_VERSION as ELECTORATE_PROTOCOL_VERSION,
};
use mycelix_governance_rights::{
    BallotChoice, BallotId, BindingBallot, BindingTallyPolicy, ProposalRef, RightPermit,
    PROTOCOL_VERSION as RIGHTS_PROTOCOL_VERSION,
};
use mycelix_institutional_core::{AuthorityGrantId, Digest32, PrincipalId};
use serde::de::DeserializeOwned;
use serde::Serialize;
use std::collections::BTreeMap;

const RIGHTS_VERIFIER_ZOME: &str = "governance_rights_verifier";
const MAX_SOURCE_REFS: usize = 32;
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

// ---------------------------------------------------------------------------
// Mirrors for other governance zomes. Keep these synchronized with their
// integrity definitions; we avoid cross-linking additional integrity crates
// into this coordinator WASM.
// ---------------------------------------------------------------------------

#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct ProposalAuthorityBindingMirror {
    id: String,
    proposal_action_hash: ActionHash,
    proposal_author: String,
    context: ProposalAuthorityContext,
    actions_digest_profile: String,
    signing_policy_digest_profile: String,
    created_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct ProposalMirror {
    id: String,
    title: String,
    description: String,
    proposal_type: ProposalTypeMirror,
    author: String,
    status: ProposalStatusMirror,
    actions: String,
    discussion_url: Option<String>,
    voting_starts: Timestamp,
    voting_ends: Timestamp,
    created: Timestamp,
    updated: Timestamp,
    version: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
enum ProposalTypeMirror {
    Standard,
    Emergency,
    Constitutional,
    Parameter,
    Funding,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
enum ProposalStatusMirror {
    Draft,
    Active,
    Ended,
    Approved,
    Signed,
    Rejected,
    Executed,
    Cancelled,
    Failed,
}

// ---------------------------------------------------------------------------
// Public inputs
// ---------------------------------------------------------------------------

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateElectionConfigurationInput {
    pub proposal_id: String,
    pub eligible_voters: u64,
    pub eligibility_criteria_digest: Digest32,
    pub eligibility_criteria_profile: String,
    pub membership_commitment: Digest32,
    pub membership_commitment_profile: String,
    pub membership_verifier_policy_digest: Digest32,
    pub membership_verifier_policy_profile: String,
    pub membership_verifier_policy_ref: String,
    pub membership_verification_profile: String,
    pub snapshot_authorized_by: AuthorityGrantId,
    pub snapshot_proof_ref: String,
    pub tally_policy: BindingTallyPolicy,
    pub tally_policy_profile: String,
    pub ballot_finality_policy_digest: Digest32,
    pub ballot_finality_policy_profile: String,
    pub ballot_finality_policy_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CastBindingBallotInput {
    pub proposal_id: String,
    pub choice: BallotChoice,
    /// References to credential/governance evidence understood by the verifier.
    /// They are not trusted merely because they are supplied by the voter.
    pub eligibility_source_refs: Vec<String>,
    /// Witness/proof for membership in the frozen electorate commitment.
    pub membership_proof_ref: String,
}

// ---------------------------------------------------------------------------
// Local verifier contract. The verifier is intentionally a separate zome
// boundary. If it is missing or returns a non-exact response, binding governance
// fails closed.
// ---------------------------------------------------------------------------

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyElectionConfigurationRequest {
    proposal_authority_binding: ActionHash,
    proposal_authority_context: ProposalAuthorityContext,
    proposal_author: String,
    snapshot: ElectorateSnapshot,
    tally_policy: BindingTallyPolicy,
    tally_policy_profile: String,
    ballot_finality_policy_digest: Digest32,
    ballot_finality_policy_profile: String,
    ballot_finality_policy_ref: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyElectionConfigurationResponse {
    verified: bool,
    proposal_authority_binding: ActionHash,
    snapshot: ElectorateSnapshot,
    tally_policy: BindingTallyPolicy,
    tally_policy_profile: String,
    ballot_finality_policy_digest: Digest32,
    ballot_finality_policy_profile: String,
    ballot_finality_policy_ref: String,
    receipt_ref: String,
    reason: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyBallotQualificationRequest {
    proposal_authority_binding: ActionHash,
    election_configuration: ActionHash,
    snapshot: ElectorateSnapshot,
    voter: PrincipalId,
    eligibility_source_refs: Vec<String>,
    membership_proof_ref: String,
    cast_at_ms: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyBallotQualificationResponse {
    verified: bool,
    proposal_authority_binding: ActionHash,
    election_configuration: ActionHash,
    voter: PrincipalId,
    permit: RightPermit,
    membership: ElectorateMembershipEvidence,
    ballot_proof_ref: String,
    receipt_ref: String,
    reason: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyStoredBallotReceiptRequest {
    proposal_authority_binding: ActionHash,
    election_configuration: ActionHash,
    ballot_action: ActionHash,
    snapshot: ElectorateSnapshot,
    ballot: BindingBallotRecord,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyStoredBallotReceiptResponse {
    verified_at_cast: bool,
    election_configuration: ActionHash,
    ballot_action: ActionHash,
    voter: PrincipalId,
    receipt_ref: String,
    reason: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct AttestBallotSetFinalityRequest {
    proposal_authority_binding: ActionHash,
    election_configuration: ActionHash,
    ballot_closes_at_ms: u64,
    finality_policy_digest: Digest32,
    finality_policy_profile: String,
    finality_policy_ref: String,
    selected_ballots: Vec<ActionHash>,
    superseded_ballots: Vec<ActionHash>,
    observed_valid_ballots: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct AttestBallotSetFinalityResponse {
    finalized: bool,
    election_configuration: ActionHash,
    finality_policy_digest: Digest32,
    finality_policy_profile: String,
    finality_policy_ref: String,
    selected_ballots: Vec<ActionHash>,
    superseded_ballots: Vec<ActionHash>,
    observed_valid_ballots: u64,
    observer_count: u32,
    required_observer_count: u32,
    receipt_ref: String,
    reason: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyCheckpointRequest {
    checkpoint_action: ActionHash,
    checkpoint: BallotSetCheckpoint,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifyCheckpointResponse {
    verified: bool,
    checkpoint_action: ActionHash,
    election_configuration: ActionHash,
    selected_ballots: Vec<ActionHash>,
    superseded_ballots: Vec<ActionHash>,
    receipt_ref: String,
    reason: Option<String>,
}

// ---------------------------------------------------------------------------
// Basic helpers
// ---------------------------------------------------------------------------

fn anchor_hash(name: &str) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(name.to_string())))
}

fn proposal_election_anchor(proposal_id: &str) -> String {
    format!("binding-election:{proposal_id}")
}

fn voter_anchor(voter: &PrincipalId) -> String {
    format!("binding-voter:{}", voter.as_str())
}

fn timestamp_ms(timestamp: Timestamp, field: &str) -> ExternResult<u64> {
    let micros = timestamp.as_micros();
    if micros <= 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} timestamp must be positive"
        ))));
    }
    Ok(micros as u64 / 1_000)
}

fn validate_ref(value: &str, field: &str) -> ExternResult<()> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} must be 1-{MAX_REF_BYTES} bytes"
        ))));
    }
    Ok(())
}

fn validate_profile(value: &str, field: &str) -> ExternResult<()> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{field} is not a canonical profile token"
        ))));
    }
    Ok(())
}

fn caller_principal() -> ExternResult<PrincipalId> {
    let caller = agent_info()?.agent_initial_pubkey;
    PrincipalId::new(format!("did:mycelix:{caller}"))
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))
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
                "{zome}::{function} unavailable; binding governance fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot decode {zome}::{function} response: {e}"
        )))
    })
}

fn fetch_proposal(proposal_id: &str) -> ExternResult<(Record, ProposalMirror)> {
    let maybe: Option<Record> = call_local("proposals", "get_proposal", proposal_id.to_string())?;
    let record = maybe.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Proposal '{proposal_id}' not found"
        )))
    })?;
    let proposal: ProposalMirror = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Proposal entry missing".into())))?;
    if proposal.id != proposal_id {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal lookup returned a different proposal id".into()
        )));
    }
    Ok((record, proposal))
}

fn fetch_verified_authority(
    proposal_id: &str,
) -> ExternResult<(Record, ProposalAuthorityBindingMirror)> {
    let maybe: Option<Record> = call_local(
        "proposal_authority",
        "get_verified_proposal_authority_context",
        proposal_id.to_string(),
    )?;
    let record = maybe.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Binding governance requires a verified proposal authority context".into(),
        ))
    })?;
    let binding: ProposalAuthorityBindingMirror = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Proposal authority record has no binding entry".into(),
            ))
        })?;
    if binding.context.proposal_id.as_str() != proposal_id {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal authority context targets a different proposal".into()
        )));
    }
    Ok((record, binding))
}

fn decode_election(record: &Record) -> Option<ElectionConfiguration> {
    record.entry().to_app_option::<ElectionConfiguration>().ok().flatten()
}

fn decode_ballot(record: &Record) -> Option<BindingBallotRecord> {
    record.entry().to_app_option::<BindingBallotRecord>().ok().flatten()
}

fn decode_checkpoint(record: &Record) -> Option<BallotSetCheckpoint> {
    record.entry().to_app_option::<BallotSetCheckpoint>().ok().flatten()
}

fn decode_tally(record: &Record) -> Option<BindingTallyRecord> {
    record.entry().to_app_option::<BindingTallyRecord>().ok().flatten()
}

fn same_election_contract(a: &ElectionConfiguration, b: &ElectionConfiguration) -> bool {
    a.proposal_id == b.proposal_id
        && a.proposal_authority_binding == b.proposal_authority_binding
        && a.snapshot == b.snapshot
        && a.tally_policy == b.tally_policy
        && a.tally_policy_profile == b.tally_policy_profile
        && a.ballot_finality_policy_digest == b.ballot_finality_policy_digest
        && a.ballot_finality_policy_profile == b.ballot_finality_policy_profile
        && a.ballot_finality_policy_ref == b.ballot_finality_policy_ref
}

// ---------------------------------------------------------------------------
// Election configuration verification
// ---------------------------------------------------------------------------

fn verify_election_with_boundary(
    authority_action: &ActionHash,
    authority: &ProposalAuthorityBindingMirror,
    election: &ElectionConfiguration,
) -> ExternResult<bool> {
    let response: VerifyElectionConfigurationResponse = call_local(
        RIGHTS_VERIFIER_ZOME,
        "verify_election_configuration",
        VerifyElectionConfigurationRequest {
            proposal_authority_binding: authority_action.clone(),
            proposal_authority_context: authority.context.clone(),
            proposal_author: authority.proposal_author.clone(),
            snapshot: election.snapshot.clone(),
            tally_policy: election.tally_policy.clone(),
            tally_policy_profile: election.tally_policy_profile.clone(),
            ballot_finality_policy_digest: election.ballot_finality_policy_digest,
            ballot_finality_policy_profile: election.ballot_finality_policy_profile.clone(),
            ballot_finality_policy_ref: election.ballot_finality_policy_ref.clone(),
        },
    )?;

    if !response.verified {
        return Ok(false);
    }
    validate_ref(&response.receipt_ref, "election verifier receipt")?;
    Ok(response.proposal_authority_binding == *authority_action
        && response.snapshot == election.snapshot
        && response.tally_policy == election.tally_policy
        && response.tally_policy_profile == election.tally_policy_profile
        && response.ballot_finality_policy_digest == election.ballot_finality_policy_digest
        && response.ballot_finality_policy_profile == election.ballot_finality_policy_profile
        && response.ballot_finality_policy_ref == election.ballot_finality_policy_ref
        && response.receipt_ref == election.verifier_receipt_ref)
}

fn list_election_records(proposal_id: &str) -> ExternResult<Vec<Record>> {
    let anchor = proposal_election_anchor(proposal_id);
    let links = get_links(
        LinkQuery::try_new(anchor_hash(&anchor)?, LinkTypes::ProposalToElectionConfiguration)?,
        GetStrategy::default(),
    )?;
    let mut records = Vec::new();
    for link in links {
        if let Ok(action_hash) = ActionHash::try_from(link.target) {
            if let Some(record) = get(action_hash, GetOptions::default())? {
                records.push(record);
            }
        }
    }
    Ok(records)
}

#[hdk_extern]
pub fn get_verified_election_configuration(
    proposal_id: String,
) -> ExternResult<Option<Record>> {
    if proposal_id.is_empty() || proposal_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal id must be 1-256 bytes".into()
        )));
    }
    let (authority_record, authority) = fetch_verified_authority(&proposal_id)?;
    let (_, proposal) = fetch_proposal(&proposal_id)?;
    let authority_action = authority_record.action_address().clone();
    let open_ms = timestamp_ms(proposal.voting_starts, "proposal voting_starts")?;
    let close_ms = timestamp_ms(proposal.voting_ends, "proposal voting_ends")?;

    let mut verified: Vec<(Record, ElectionConfiguration)> = Vec::new();
    for record in list_election_records(&proposal_id)? {
        let Some(election) = decode_election(&record) else {
            continue;
        };
        if election.validate_structure().is_err()
            || election.proposal_authority_binding != authority_action
            || election.created_by != proposal.author
            || election.snapshot.institution != authority.context.institution
            || election.snapshot.jurisdiction != authority.context.jurisdiction
            || election.snapshot.rulebook != authority.context.rulebook
            || election.snapshot.ballot_opens_at_ms != open_ms
            || election.snapshot.ballot_closes_at_ms != close_ms
        {
            continue;
        }
        if verify_election_with_boundary(&authority_action, &authority, &election)? {
            verified.push((record, election));
        }
    }

    if verified.is_empty() {
        return Ok(None);
    }
    let reference = verified[0].1.clone();
    if verified
        .iter()
        .skip(1)
        .any(|(_, candidate)| !same_election_contract(&reference, candidate))
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Conflicting verified binding-election configurations exist; fail closed".into(),
        )));
    }
    let record = verified
        .into_iter()
        .map(|(record, _)| record)
        .max_by_key(|record| record.action().action_seq())
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Election disappeared".into())))?;
    Ok(Some(record))
}

#[hdk_extern]
pub fn create_election_configuration(
    input: CreateElectionConfigurationInput,
) -> ExternResult<Record> {
    validate_profile(&input.eligibility_criteria_profile, "eligibility criteria profile")?;
    validate_profile(&input.membership_commitment_profile, "membership commitment profile")?;
    validate_profile(
        &input.membership_verifier_policy_profile,
        "membership verifier policy profile",
    )?;
    validate_profile(
        &input.membership_verification_profile,
        "membership verification profile",
    )?;
    validate_profile(&input.tally_policy_profile, "tally policy profile")?;
    validate_profile(
        &input.ballot_finality_policy_profile,
        "ballot finality policy profile",
    )?;
    validate_ref(
        &input.membership_verifier_policy_ref,
        "membership verifier policy reference",
    )?;
    validate_ref(&input.snapshot_proof_ref, "snapshot proof reference")?;
    validate_ref(
        &input.ballot_finality_policy_ref,
        "ballot finality policy reference",
    )?;
    input
        .tally_policy
        .validate()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;

    let (authority_record, authority) = fetch_verified_authority(&input.proposal_id)?;
    let (_, proposal) = fetch_proposal(&input.proposal_id)?;
    if proposal.status != ProposalStatusMirror::Active {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Binding election configuration requires an Active proposal".into()
        )));
    }
    let caller = caller_principal()?;
    if caller.as_str() != proposal.author {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the proposal author may submit the binding election configuration".into(),
        )));
    }

    let now = sys_time()?;
    let now_ms = timestamp_ms(now, "current")?;
    let open_ms = timestamp_ms(proposal.voting_starts, "proposal voting_starts")?;
    let close_ms = timestamp_ms(proposal.voting_ends, "proposal voting_ends")?;
    if now_ms > open_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Binding electorate must be frozen no later than ballot opening".into(),
        )));
    }
    if authority.context.expires_at_ms <= close_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal authority context expires before the binding ballot closes".into(),
        )));
    }

    let authority_action = authority_record.action_address().clone();
    let snapshot = ElectorateSnapshot {
        protocol_version: ELECTORATE_PROTOCOL_VERSION.into(),
        id: ElectorateSnapshotId::new(format!(
            "electorate:{}:{}",
            input.proposal_id,
            now.as_micros()
        ))
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
        proposal: ProposalRef::new(input.proposal_id.clone())
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
        institution: authority.context.institution.clone(),
        jurisdiction: authority.context.jurisdiction.clone(),
        rulebook: authority.context.rulebook.clone(),
        eligible_voters: input.eligible_voters,
        eligibility_criteria_digest: input.eligibility_criteria_digest,
        eligibility_criteria_profile: input.eligibility_criteria_profile,
        membership_commitment: input.membership_commitment,
        membership_commitment_profile: input.membership_commitment_profile,
        membership_verifier_policy_digest: input.membership_verifier_policy_digest,
        membership_verifier_policy_profile: input.membership_verifier_policy_profile,
        membership_verifier_policy_ref: input.membership_verifier_policy_ref,
        membership_verification_profile: input.membership_verification_profile,
        captured_at_ms: now_ms,
        ballot_opens_at_ms: open_ms,
        ballot_closes_at_ms: close_ms,
        // v0.1 admission validates revocation/expiry at cast time. Once a ballot
        // has an exact frozen-snapshot qualification receipt, tally verification
        // checks that receipt rather than silently consulting mutable live scores.
        revocation_semantics: RevocationSemantics::RejectRevokedBallotKeepDenominator,
        authorized_by: input.snapshot_authorized_by,
        snapshot_proof_ref: input.snapshot_proof_ref,
    };
    snapshot
        .validate()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;

    let candidate = ElectionConfiguration {
        id: format!("binding-election:{}:{}", input.proposal_id, now.as_micros()),
        proposal_id: input.proposal_id.clone(),
        proposal_authority_binding: authority_action.clone(),
        snapshot,
        tally_policy: input.tally_policy,
        tally_policy_profile: input.tally_policy_profile,
        ballot_finality_policy_digest: input.ballot_finality_policy_digest,
        ballot_finality_policy_profile: input.ballot_finality_policy_profile,
        ballot_finality_policy_ref: input.ballot_finality_policy_ref,
        verifier_receipt_ref: "pending-verifier-receipt".into(),
        created_by: caller.as_str().to_string(),
        created_at: now,
    };

    let response: VerifyElectionConfigurationResponse = call_local(
        RIGHTS_VERIFIER_ZOME,
        "verify_election_configuration",
        VerifyElectionConfigurationRequest {
            proposal_authority_binding: authority_action.clone(),
            proposal_authority_context: authority.context.clone(),
            proposal_author: authority.proposal_author.clone(),
            snapshot: candidate.snapshot.clone(),
            tally_policy: candidate.tally_policy.clone(),
            tally_policy_profile: candidate.tally_policy_profile.clone(),
            ballot_finality_policy_digest: candidate.ballot_finality_policy_digest,
            ballot_finality_policy_profile: candidate.ballot_finality_policy_profile.clone(),
            ballot_finality_policy_ref: candidate.ballot_finality_policy_ref.clone(),
        },
    )?;
    if !response.verified {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Binding election configuration denied by verifier: {}",
            response.reason.unwrap_or_else(|| "unspecified reason".into())
        ))));
    }
    validate_ref(&response.receipt_ref, "election verifier receipt")?;
    if response.proposal_authority_binding != authority_action
        || response.snapshot != candidate.snapshot
        || response.tally_policy != candidate.tally_policy
        || response.tally_policy_profile != candidate.tally_policy_profile
        || response.ballot_finality_policy_digest != candidate.ballot_finality_policy_digest
        || response.ballot_finality_policy_profile != candidate.ballot_finality_policy_profile
        || response.ballot_finality_policy_ref != candidate.ballot_finality_policy_ref
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Election verifier response did not exactly bind the submitted configuration".into(),
        )));
    }

    let mut election = candidate;
    election.verifier_receipt_ref = response.receipt_ref;
    election
        .validate_structure()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;

    if let Some(existing_record) = get_verified_election_configuration(input.proposal_id.clone())? {
        let existing = decode_election(&existing_record).ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Verified election record cannot be decoded".into(),
            ))
        })?;
        if same_election_contract(&existing, &election) {
            return Ok(existing_record);
        }
        return Err(wasm_error!(WasmErrorInner::Guest(
            "A different verified binding-election configuration already exists".into(),
        )));
    }

    let action_hash = create_entry(&EntryTypes::ElectionConfiguration(election))?;
    let anchor = proposal_election_anchor(&input.proposal_id);
    create_entry(&EntryTypes::Anchor(Anchor(anchor.clone())))?;
    create_link(
        anchor_hash(&anchor)?,
        action_hash.clone(),
        LinkTypes::ProposalToElectionConfiguration,
        (),
    )?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve created binding-election configuration".into(),
        ))
    })
}

// ---------------------------------------------------------------------------
// Binding ballot admission
// ---------------------------------------------------------------------------

#[hdk_extern]
pub fn cast_binding_ballot(input: CastBindingBallotInput) -> ExternResult<Record> {
    if input.eligibility_source_refs.is_empty()
        || input.eligibility_source_refs.len() > MAX_SOURCE_REFS
    {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Eligibility source refs must contain 1-{MAX_SOURCE_REFS} items"
        ))));
    }
    for source in &input.eligibility_source_refs {
        validate_ref(source, "eligibility source reference")?;
    }
    validate_ref(&input.membership_proof_ref, "electorate membership proof")?;

    let election_record = get_verified_election_configuration(input.proposal_id.clone())?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "No verified binding-election configuration exists".into(),
            ))
        })?;
    let election = decode_election(&election_record).ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Verified election configuration cannot be decoded".into(),
        ))
    })?;

    let now = sys_time()?;
    let now_ms = timestamp_ms(now, "current")?;
    if !election.snapshot.ballot_open_at(now_ms) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Binding ballot is outside the configured voting window".into(),
        )));
    }

    let voter = caller_principal()?;
    let response: VerifyBallotQualificationResponse = call_local(
        RIGHTS_VERIFIER_ZOME,
        "verify_ballot_qualification",
        VerifyBallotQualificationRequest {
            proposal_authority_binding: election.proposal_authority_binding.clone(),
            election_configuration: election_record.action_address().clone(),
            snapshot: election.snapshot.clone(),
            voter: voter.clone(),
            eligibility_source_refs: input.eligibility_source_refs,
            membership_proof_ref: input.membership_proof_ref,
            cast_at_ms: now_ms,
        },
    )?;
    if !response.verified {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Binding ballot eligibility denied: {}",
            response.reason.unwrap_or_else(|| "unspecified reason".into())
        ))));
    }
    validate_ref(&response.ballot_proof_ref, "binding ballot proof receipt")?;
    validate_ref(&response.receipt_ref, "binding ballot verifier receipt")?;
    if response.proposal_authority_binding != election.proposal_authority_binding
        || response.election_configuration != *election_record.action_address()
        || response.voter != voter
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Ballot verifier response is bound to different authority/election/voter data".into(),
        )));
    }

    // A snapshot represents the electorate for the whole ballot. Qualification
    // evidence must therefore remain valid through ballot close; this prevents
    // ordinary credential expiry during the vote from silently changing who can
    // participate. Fraud/revocation discovered later goes through an explicit
    // challenge/remedy path rather than mutating the tally invisibly.
    if response.permit.valid_until_ms < election.snapshot.ballot_closes_at_ms
        || response.membership.expires_at_ms < election.snapshot.ballot_closes_at_ms
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Frozen-snapshot ballot qualification must remain valid through ballot close".into(),
        )));
    }

    let ballot = BindingBallot {
        protocol_version: RIGHTS_PROTOCOL_VERSION.into(),
        id: BallotId::new(format!(
            "binding-ballot:{}:{}:{}",
            input.proposal_id,
            voter.as_str(),
            now.as_micros()
        ))
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
        proposal: ProposalRef::new(input.proposal_id.clone())
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
        voter: voter.clone(),
        eligibility_grant_id: response.permit.eligibility_grant_id.clone(),
        choice: input.choice,
        cast_at_ms: now_ms,
        ballot_proof_ref: response.ballot_proof_ref,
    };

    let qualification = BallotQualification {
        ballot: ballot.clone(),
        permit: response.permit.clone(),
        membership: response.membership.clone(),
    };
    qualification
        .validate_against_snapshot(&election.snapshot)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;

    let entry = BindingBallotRecord {
        id: format!("bbr:{}:{}", input.proposal_id, now.as_micros()),
        proposal_id: input.proposal_id,
        proposal_authority_binding: election.proposal_authority_binding.clone(),
        election_configuration: election_record.action_address().clone(),
        ballot,
        permit: response.permit,
        membership: response.membership,
        verifier_receipt_ref: response.receipt_ref,
        committed_at: now,
    };
    entry
        .validate_structure()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;

    let action_hash = create_entry(&EntryTypes::BindingBallot(entry.clone()))?;
    create_link(
        election_record.action_address().clone(),
        action_hash.clone(),
        LinkTypes::ElectionConfigurationToBallot,
        (),
    )?;
    let voter_anchor_name = voter_anchor(&voter);
    create_entry(&EntryTypes::Anchor(Anchor(voter_anchor_name.clone())))?;
    create_link(
        anchor_hash(&voter_anchor_name)?,
        action_hash.clone(),
        LinkTypes::VoterToBindingBallot,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve committed binding ballot".into(),
        ))
    })
}

fn verify_stored_ballot(
    election_record: &Record,
    election: &ElectionConfiguration,
    record: &Record,
    ballot: &BindingBallotRecord,
) -> ExternResult<bool> {
    if ballot.validate_structure().is_err()
        || ballot.proposal_id != election.proposal_id
        || ballot.proposal_authority_binding != election.proposal_authority_binding
        || ballot.election_configuration != *election_record.action_address()
        || ballot
            .qualification()
            .validate_against_snapshot(&election.snapshot)
            .is_err()
    {
        return Ok(false);
    }
    let response: VerifyStoredBallotReceiptResponse = call_local(
        RIGHTS_VERIFIER_ZOME,
        "verify_stored_ballot_receipt",
        VerifyStoredBallotReceiptRequest {
            proposal_authority_binding: election.proposal_authority_binding.clone(),
            election_configuration: election_record.action_address().clone(),
            ballot_action: record.action_address().clone(),
            snapshot: election.snapshot.clone(),
            ballot: ballot.clone(),
        },
    )?;
    if !response.verified_at_cast {
        return Ok(false);
    }
    Ok(response.election_configuration == *election_record.action_address()
        && response.ballot_action == *record.action_address()
        && response.voter == ballot.ballot.voter
        && response.receipt_ref == ballot.verifier_receipt_ref)
}

fn list_valid_ballots(
    election_record: &Record,
    election: &ElectionConfiguration,
) -> ExternResult<Vec<(Record, BindingBallotRecord)>> {
    let links = get_links(
        LinkQuery::try_new(
            election_record.action_address().clone(),
            LinkTypes::ElectionConfigurationToBallot,
        )?,
        GetStrategy::default(),
    )?;
    let mut valid = Vec::new();
    for link in links {
        let Ok(action_hash) = ActionHash::try_from(link.target) else {
            continue;
        };
        let Some(record) = get(action_hash, GetOptions::default())? else {
            continue;
        };
        let Some(ballot) = decode_ballot(&record) else {
            continue;
        };
        if verify_stored_ballot(election_record, election, &record, &ballot)? {
            valid.push((record, ballot));
        }
    }
    Ok(valid)
}

/// Select the latest valid source-chain action for each voter. Holochain action
/// sequence is author-chain order, so this projection is independent of DHT
/// arrival order and naturally supports append-only re-voting.
fn project_latest_per_voter(
    ballots: Vec<(Record, BindingBallotRecord)>,
) -> (Vec<(Record, BindingBallotRecord)>, Vec<ActionHash>) {
    let mut latest: BTreeMap<String, (Record, BindingBallotRecord)> = BTreeMap::new();
    let mut superseded = Vec::new();

    for (record, ballot) in ballots {
        let voter = ballot.ballot.voter.as_str().to_string();
        match latest.get(&voter) {
            None => {
                latest.insert(voter, (record, ballot));
            }
            Some((current_record, _)) => {
                let current_key = (
                    current_record.action().action_seq(),
                    current_record.action_address().to_string(),
                );
                let candidate_key = (
                    record.action().action_seq(),
                    record.action_address().to_string(),
                );
                if candidate_key > current_key {
                    superseded.push(current_record.action_address().clone());
                    latest.insert(voter, (record, ballot));
                } else {
                    superseded.push(record.action_address().clone());
                }
            }
        }
    }

    let selected = latest.into_values().collect::<Vec<_>>();
    superseded.sort_by_key(ToString::to_string);
    (selected, superseded)
}

// ---------------------------------------------------------------------------
// Ballot-set finality
// ---------------------------------------------------------------------------

fn list_checkpoint_records(election_action: &ActionHash) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(
            election_action.clone(),
            LinkTypes::ElectionConfigurationToCheckpoint,
        )?,
        GetStrategy::default(),
    )?;
    let mut records = Vec::new();
    for link in links {
        if let Ok(action_hash) = ActionHash::try_from(link.target) {
            if let Some(record) = get(action_hash, GetOptions::default())? {
                records.push(record);
            }
        }
    }
    Ok(records)
}

fn verify_checkpoint_with_boundary(
    record: &Record,
    checkpoint: &BallotSetCheckpoint,
) -> ExternResult<bool> {
    let response: VerifyCheckpointResponse = call_local(
        RIGHTS_VERIFIER_ZOME,
        "verify_ballot_set_checkpoint",
        VerifyCheckpointRequest {
            checkpoint_action: record.action_address().clone(),
            checkpoint: checkpoint.clone(),
        },
    )?;
    Ok(response.verified
        && response.checkpoint_action == *record.action_address()
        && response.election_configuration == checkpoint.election_configuration
        && response.selected_ballots == checkpoint.selected_ballots
        && response.superseded_ballots == checkpoint.superseded_ballots
        && response.receipt_ref == checkpoint.verifier_receipt_ref)
}

fn same_checkpoint_contract(a: &BallotSetCheckpoint, b: &BallotSetCheckpoint) -> bool {
    a.proposal_id == b.proposal_id
        && a.proposal_authority_binding == b.proposal_authority_binding
        && a.election_configuration == b.election_configuration
        && a.selected_ballots == b.selected_ballots
        && a.superseded_ballots == b.superseded_ballots
        && a.finality_policy_digest == b.finality_policy_digest
        && a.finality_policy_profile == b.finality_policy_profile
        && a.finality_policy_ref == b.finality_policy_ref
}

#[hdk_extern]
pub fn get_verified_ballot_set_checkpoint(
    proposal_id: String,
) -> ExternResult<Option<Record>> {
    let election_record = get_verified_election_configuration(proposal_id.clone())?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "No verified binding-election configuration exists".into(),
            ))
        })?;
    let election = decode_election(&election_record).ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Verified election configuration cannot be decoded".into(),
        ))
    })?;

    let mut verified: Vec<(Record, BallotSetCheckpoint)> = Vec::new();
    for record in list_checkpoint_records(election_record.action_address())? {
        let Some(checkpoint) = decode_checkpoint(&record) else {
            continue;
        };
        if checkpoint.validate_structure().is_err()
            || checkpoint.proposal_id != proposal_id
            || checkpoint.proposal_authority_binding != election.proposal_authority_binding
            || checkpoint.election_configuration != *election_record.action_address()
            || checkpoint.finality_policy_digest != election.ballot_finality_policy_digest
            || checkpoint.finality_policy_profile != election.ballot_finality_policy_profile
            || checkpoint.finality_policy_ref != election.ballot_finality_policy_ref
        {
            continue;
        }
        if verify_checkpoint_with_boundary(&record, &checkpoint)? {
            verified.push((record, checkpoint));
        }
    }
    if verified.is_empty() {
        return Ok(None);
    }
    let reference = verified[0].1.clone();
    if verified
        .iter()
        .skip(1)
        .any(|(_, candidate)| !same_checkpoint_contract(&reference, candidate))
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Conflicting verified ballot-set checkpoints exist; fail closed".into(),
        )));
    }
    Ok(verified
        .into_iter()
        .map(|(record, _)| record)
        .max_by_key(|record| record.action().action_seq()))
}

#[hdk_extern]
pub fn finalize_ballot_set(proposal_id: String) -> ExternResult<Record> {
    if let Some(existing) = get_verified_ballot_set_checkpoint(proposal_id.clone())? {
        return Ok(existing);
    }
    let election_record = get_verified_election_configuration(proposal_id.clone())?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "No verified binding-election configuration exists".into(),
            ))
        })?;
    let election = decode_election(&election_record).ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Verified election configuration cannot be decoded".into(),
        ))
    })?;
    let now = sys_time()?;
    let now_ms = timestamp_ms(now, "current")?;
    if now_ms < election.snapshot.ballot_closes_at_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Ballot set cannot be finalized before voting closes".into(),
        )));
    }

    let valid = list_valid_ballots(&election_record, &election)?;
    let observed_valid_ballots = valid.len() as u64;
    let (selected, superseded_ballots) = project_latest_per_voter(valid);
    let selected_ballots = selected
        .iter()
        .map(|(record, _)| record.action_address().clone())
        .collect::<Vec<_>>();

    let response: AttestBallotSetFinalityResponse = call_local(
        RIGHTS_VERIFIER_ZOME,
        "attest_ballot_set_finality",
        AttestBallotSetFinalityRequest {
            proposal_authority_binding: election.proposal_authority_binding.clone(),
            election_configuration: election_record.action_address().clone(),
            ballot_closes_at_ms: election.snapshot.ballot_closes_at_ms,
            finality_policy_digest: election.ballot_finality_policy_digest,
            finality_policy_profile: election.ballot_finality_policy_profile.clone(),
            finality_policy_ref: election.ballot_finality_policy_ref.clone(),
            selected_ballots: selected_ballots.clone(),
            superseded_ballots: superseded_ballots.clone(),
            observed_valid_ballots,
        },
    )?;
    if !response.finalized {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Ballot-set finality not established: {}",
            response.reason.unwrap_or_else(|| "finality policy not yet satisfied".into())
        ))));
    }
    validate_ref(&response.receipt_ref, "ballot-set finality receipt")?;
    if response.election_configuration != *election_record.action_address()
        || response.finality_policy_digest != election.ballot_finality_policy_digest
        || response.finality_policy_profile != election.ballot_finality_policy_profile
        || response.finality_policy_ref != election.ballot_finality_policy_ref
        || response.selected_ballots != selected_ballots
        || response.superseded_ballots != superseded_ballots
        || response.observed_valid_ballots != observed_valid_ballots
        || response.required_observer_count == 0
        || response.observer_count < response.required_observer_count
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Finality verifier response does not exactly bind the observed ballot set/policy"
                .into(),
        )));
    }

    let finalized_by = caller_principal()?;
    let checkpoint = BallotSetCheckpoint {
        id: format!("ballot-checkpoint:{}:{}", proposal_id, now.as_micros()),
        proposal_id,
        proposal_authority_binding: election.proposal_authority_binding,
        election_configuration: election_record.action_address().clone(),
        selected_ballots,
        superseded_ballots,
        observed_valid_ballots,
        finality_policy_digest: response.finality_policy_digest,
        finality_policy_profile: response.finality_policy_profile,
        finality_policy_ref: response.finality_policy_ref,
        observer_count: response.observer_count,
        required_observer_count: response.required_observer_count,
        verifier_receipt_ref: response.receipt_ref,
        finalized_by: finalized_by.as_str().to_string(),
        finalized_at: now,
    };
    checkpoint
        .validate_structure()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;
    let action_hash = create_entry(&EntryTypes::BallotSetCheckpoint(checkpoint))?;
    create_link(
        election_record.action_address().clone(),
        action_hash.clone(),
        LinkTypes::ElectionConfigurationToCheckpoint,
        (),
    )?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve finalized ballot-set checkpoint".into(),
        ))
    })
}

// ---------------------------------------------------------------------------
// Deterministic tally over a finalized ballot set
// ---------------------------------------------------------------------------

fn recompute_tally(
    proposal_id: &str,
) -> ExternResult<(Record, ElectionConfiguration, Record, BallotSetCheckpoint, SnapshotBoundTally)> {
    let election_record = get_verified_election_configuration(proposal_id.to_string())?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "No verified binding-election configuration exists".into(),
            ))
        })?;
    let election = decode_election(&election_record).ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Verified election configuration cannot be decoded".into(),
        ))
    })?;
    let checkpoint_record = get_verified_ballot_set_checkpoint(proposal_id.to_string())?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "No verified ballot-set finality checkpoint exists".into(),
            ))
        })?;
    let checkpoint = decode_checkpoint(&checkpoint_record).ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Verified ballot-set checkpoint cannot be decoded".into(),
        ))
    })?;

    let mut qualifications = Vec::with_capacity(checkpoint.selected_ballots.len());
    for action_hash in &checkpoint.selected_ballots {
        let record = get(action_hash.clone(), GetOptions::default())?.ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Finalized ballot action '{}' is unavailable",
                action_hash
            )))
        })?;
        let ballot = decode_ballot(&record).ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Finalized ballot action does not decode as a binding ballot".into(),
            ))
        })?;
        if !verify_stored_ballot(&election_record, &election, &record, &ballot)? {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Finalized ballot no longer verifies against its frozen qualification receipt"
                    .into(),
            )));
        }
        qualifications.push(ballot.qualification());
    }

    let tally = tally_snapshot_ballots(
        &election.snapshot,
        &election.tally_policy,
        &qualifications,
    )
    .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    Ok((election_record, election, checkpoint_record, checkpoint, tally))
}

fn list_tally_records(checkpoint_action: &ActionHash) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(checkpoint_action.clone(), LinkTypes::CheckpointToTally)?,
        GetStrategy::default(),
    )?;
    let mut records = Vec::new();
    for link in links {
        if let Ok(action_hash) = ActionHash::try_from(link.target) {
            if let Some(record) = get(action_hash, GetOptions::default())? {
                records.push(record);
            }
        }
    }
    Ok(records)
}

#[hdk_extern]
pub fn get_verified_binding_tally(proposal_id: String) -> ExternResult<Option<Record>> {
    let (election_record, election, checkpoint_record, _, expected) =
        recompute_tally(&proposal_id)?;
    let mut matching = Vec::new();
    for record in list_tally_records(checkpoint_record.action_address())? {
        let Some(tally) = decode_tally(&record) else {
            continue;
        };
        if tally.validate_structure().is_ok()
            && tally.proposal_id == proposal_id
            && tally.proposal_authority_binding == election.proposal_authority_binding
            && tally.election_configuration == *election_record.action_address()
            && tally.ballot_set_checkpoint == *checkpoint_record.action_address()
            && tally.tally == expected
        {
            matching.push(record);
        }
    }
    Ok(matching
        .into_iter()
        .max_by_key(|record| record.action().action_seq()))
}

#[hdk_extern]
pub fn compute_binding_tally(proposal_id: String) -> ExternResult<Record> {
    if let Some(existing) = get_verified_binding_tally(proposal_id.clone())? {
        return Ok(existing);
    }
    let (election_record, election, checkpoint_record, _, tally) =
        recompute_tally(&proposal_id)?;
    let now = sys_time()?;
    let computed_by = caller_principal()?;
    let entry = BindingTallyRecord {
        id: format!("binding-tally:{}:{}", proposal_id, now.as_micros()),
        proposal_id,
        proposal_authority_binding: election.proposal_authority_binding,
        election_configuration: election_record.action_address().clone(),
        ballot_set_checkpoint: checkpoint_record.action_address().clone(),
        tally,
        computed_by: computed_by.as_str().to_string(),
        computed_at: now,
    };
    entry
        .validate_structure()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;
    let action_hash = create_entry(&EntryTypes::BindingTally(entry))?;
    create_link(
        checkpoint_record.action_address().clone(),
        action_hash.clone(),
        LinkTypes::CheckpointToTally,
        (),
    )?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "Could not retrieve computed binding tally".into(),
        ))
    })
}

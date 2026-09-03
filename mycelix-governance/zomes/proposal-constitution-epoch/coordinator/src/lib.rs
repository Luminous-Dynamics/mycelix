// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Verified current constitutional epoch for binding governance proposals.
//!
//! A persisted epoch record is only a candidate. Authoritative reads re-resolve
//! the exact proposal authority context, verified binding election, and current
//! verified constitutional lineage. Once the constitution advances, an older
//! epoch stops verifying. Because a new epoch may only be published no later
//! than ballot opening, constitutional carry-forward cannot be retrofitted after
//! a vote has already happened.

use hdk::prelude::*;
use mycelix_governance_authority::ProposalAuthorityContext;
use mycelix_governance_constitution::{ConstitutionStatement, Digest32};
use mycelix_governance_electorate::ElectorateSnapshot;
use mycelix_governance_rights::BindingTallyPolicy;
use mycelix_institutional_core::Digest32 as InstitutionalDigest32;
use proposal_constitution_epoch_integrity::*;
use serde::de::DeserializeOwned;
use serde::Serialize;

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
struct ElectionConfigurationMirror {
    id: String,
    proposal_id: String,
    proposal_authority_binding: ActionHash,
    snapshot: ElectorateSnapshot,
    tally_policy: BindingTallyPolicy,
    tally_policy_profile: String,
    ballot_finality_policy_digest: InstitutionalDigest32,
    ballot_finality_policy_profile: String,
    ballot_finality_policy_ref: String,
    verifier_receipt_ref: String,
    created_by: String,
    created_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct VerifiedCurrentConstitutionMirror {
    dna_hash: String,
    statement: ConstitutionStatement,
    statement_digest: Digest32,
    verified_transition_count: u64,
    candidate_count: u64,
    legacy_constitution_authoritative: bool,
}

struct CurrentFacts {
    authority_record: Record,
    authority: ProposalAuthorityBindingMirror,
    election_record: Record,
    election: ElectionConfigurationMirror,
    constitution: VerifiedCurrentConstitutionMirror,
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
                "{zome}::{function} unavailable; constitutional epoch fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot decode {zome}::{function} response: {e}"
        )))
    })
}

fn anchor_hash(name: &str) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(name.to_string())))
}

fn epoch_anchor(proposal_id: &str) -> String {
    format!("proposal-constitution-epoch:{proposal_id}")
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

fn decode_authority(record: &Record) -> ExternResult<ProposalAuthorityBindingMirror> {
    record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Verified proposal authority record has no application entry".into(),
            ))
        })
}

fn decode_election(record: &Record) -> ExternResult<ElectionConfigurationMirror> {
    record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Verified election record has no application entry".into(),
            ))
        })
}

fn resolve_current_facts(proposal_id: &str) -> ExternResult<CurrentFacts> {
    if proposal_id.is_empty() || proposal_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal id must be 1-256 bytes".into(),
        )));
    }

    let authority_record: Record = call_local::<_, Option<Record>>(
        "proposal_authority",
        "get_verified_proposal_authority_context",
        proposal_id.to_string(),
    )?
    .ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "No verified proposal authority context exists".into(),
        ))
    })?;
    let authority = decode_authority(&authority_record)?;
    if authority.context.proposal_id.as_str() != proposal_id {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal authority context targets a different proposal".into(),
        )));
    }

    let election_record: Record = call_local::<_, Option<Record>>(
        "binding_voting",
        "get_verified_election_configuration",
        proposal_id.to_string(),
    )?
    .ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "No verified binding-election configuration exists".into(),
        ))
    })?;
    let election = decode_election(&election_record)?;
    if election.proposal_id != proposal_id
        || election.proposal_authority_binding != *authority_record.action_address()
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Verified election is bound to different proposal authority".into(),
        )));
    }
    if election.snapshot.institution != authority.context.institution
        || election.snapshot.jurisdiction != authority.context.jurisdiction
        || election.snapshot.rulebook != authority.context.rulebook
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Verified election snapshot differs from proposal institutional authority".into(),
        )));
    }

    let constitution: VerifiedCurrentConstitutionMirror = call_local(
        "constitution_transition",
        "get_verified_current_constitution",
        (),
    )?;
    if constitution.legacy_constitution_authoritative {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Binding governance refuses a legacy constitutional authority plane".into(),
        )));
    }
    if constitution.dna_hash.trim().is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Verified constitution returned an empty DNA hash".into(),
        )));
    }
    let recomputed = constitution.statement.digest().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot digest current verified constitution: {e}"
        )))
    })?;
    if recomputed != constitution.statement_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Verified constitution statement digest does not recompute exactly".into(),
        )));
    }
    if authority.context.institution.as_str() != constitution.statement.institution_id.as_str()
        || authority.context.rulebook.id.as_str() != constitution.statement.rulebook_id.as_str()
        || authority.context.rulebook.version != constitution.statement.rulebook_version
        || authority.context.rulebook.digest.0 != constitution.statement.rulebook.digest.0
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal authority does not match the current verified constitutional institution/rulebook"
                .into(),
        )));
    }

    Ok(CurrentFacts {
        authority_record,
        authority,
        election_record,
        election,
        constitution,
    })
}

fn list_epoch_records(proposal_id: &str) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(
            anchor_hash(&epoch_anchor(proposal_id))?,
            LinkTypes::ProposalToConstitutionEpoch,
        )?,
        GetStrategy::default(),
    )?;
    let mut records = Vec::new();
    for link in links {
        let Ok(action_hash) = ActionHash::try_from(link.target) else {
            continue;
        };
        if let Some(record) = get(action_hash, GetOptions::default())? {
            records.push(record);
        }
    }
    Ok(records)
}

fn decode_epoch(record: &Record) -> Option<ProposalConstitutionEpochBinding> {
    record
        .entry()
        .to_app_option::<ProposalConstitutionEpochBinding>()
        .ok()
        .flatten()
}

fn candidate_matches_current(
    record: &Record,
    epoch: &ProposalConstitutionEpochBinding,
    facts: &CurrentFacts,
) -> ExternResult<bool> {
    if epoch.validate_structure().is_err()
        || epoch.proposal_id != facts.election.proposal_id
        || epoch.proposal_authority_binding != *facts.authority_record.action_address()
        || epoch.election_configuration != *facts.election_record.action_address()
        || epoch.constitution_statement_digest != facts.constitution.statement_digest
        || epoch.constitution_version != facts.constitution.statement.version
        || epoch.ballot_opens_at_ms != facts.election.snapshot.ballot_opens_at_ms
    {
        return Ok(false);
    }

    let action_ms = timestamp_ms(record.action().timestamp(), "epoch create action")?;
    let bound_ms = timestamp_ms(epoch.bound_at, "epoch bound_at")?;
    if action_ms != bound_ms || bound_ms > facts.election.snapshot.ballot_opens_at_ms {
        return Ok(false);
    }
    if facts.constitution.statement.effective_from_ms > bound_ms {
        return Ok(false);
    }
    Ok(true)
}

/// Resolve the exact currently valid constitutional epoch for a binding
/// proposal. If the constitution advanced after the ballot was configured, old
/// epoch records stop matching and this returns None rather than carrying them
/// forward automatically.
#[hdk_extern]
pub fn get_verified_proposal_constitution_epoch(
    proposal_id: String,
) -> ExternResult<Option<Record>> {
    let facts = resolve_current_facts(&proposal_id)?;
    let mut matching = Vec::new();
    for record in list_epoch_records(&proposal_id)? {
        let Some(epoch) = decode_epoch(&record) else {
            continue;
        };
        if candidate_matches_current(&record, &epoch, &facts)? {
            matching.push((record, epoch));
        }
    }
    if matching.is_empty() {
        return Ok(None);
    }

    let reference = &matching[0].1;
    if matching.iter().skip(1).any(|(_, candidate)| {
        candidate.proposal_authority_binding != reference.proposal_authority_binding
            || candidate.election_configuration != reference.election_configuration
            || candidate.constitution_statement_digest != reference.constitution_statement_digest
            || candidate.constitution_version != reference.constitution_version
            || candidate.ballot_opens_at_ms != reference.ballot_opens_at_ms
    }) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Conflicting verified constitutional epochs exist for one proposal".into(),
        )));
    }

    Ok(matching
        .into_iter()
        .map(|(record, _)| record)
        .max_by_key(|record| record.action().action_seq()))
}

/// Bind the proposal/election to the exact current constitutional statement.
/// Anyone may publish the same verified fact; publisher identity is provenance,
/// not authority. Publication is impossible after ballot opening.
#[hdk_extern]
pub fn bind_proposal_constitution_epoch(proposal_id: String) -> ExternResult<Record> {
    if let Some(existing) = get_verified_proposal_constitution_epoch(proposal_id.clone())? {
        return Ok(existing);
    }

    let facts = resolve_current_facts(&proposal_id)?;
    let now = sys_time()?;
    let now_ms = timestamp_ms(now, "current")?;
    if now_ms > facts.election.snapshot.ballot_opens_at_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal constitutional epoch must be bound no later than ballot opening; reauthorization requires a new proposal"
                .into(),
        )));
    }
    if facts.constitution.statement.effective_from_ms > now_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Current verified constitution is not yet effective".into(),
        )));
    }

    let caller = agent_info()?.agent_initial_pubkey;
    let epoch = ProposalConstitutionEpochBinding {
        id: format!("proposal-epoch:{}:{}", proposal_id, now.as_micros()),
        proposal_id: proposal_id.clone(),
        proposal_authority_binding: facts.authority_record.action_address().clone(),
        election_configuration: facts.election_record.action_address().clone(),
        constitution_statement_digest: facts.constitution.statement_digest,
        constitution_version: facts.constitution.statement.version,
        ballot_opens_at_ms: facts.election.snapshot.ballot_opens_at_ms,
        bound_by: format!("did:mycelix:{caller}"),
        bound_at: now,
    };
    epoch
        .validate_structure()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;

    let action_hash = create_entry(&EntryTypes::ProposalConstitutionEpoch(epoch))?;
    let anchor = epoch_anchor(&proposal_id);
    create_entry(&EntryTypes::Anchor(Anchor(anchor.clone())))?;
    create_link(
        anchor_hash(&anchor)?,
        action_hash.clone(),
        LinkTypes::ProposalToConstitutionEpoch,
        (),
    )?;

    let verified = get_verified_proposal_constitution_epoch(proposal_id)?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "New constitutional epoch did not survive authoritative re-verification".into(),
        ))
    })?;
    if *verified.action_address() != action_hash {
        // Equivalent duplicate publication is harmless; verify the returned
        // record still names exactly the current epoch.
        let returned = decode_epoch(&verified).ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Verified constitutional epoch cannot be decoded".into(),
            ))
        })?;
        if returned.constitution_statement_digest != facts.constitution.statement_digest {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Constitutional epoch changed during publication".into(),
            )));
        }
    }
    Ok(verified)
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ProposalConstitutionEpochStatus {
    pub protocol: String,
    pub binding_required_before_ballot_open: bool,
    pub current_constitution_reverified_on_read: bool,
    pub automatic_carry_forward_across_constitutions: bool,
}

#[hdk_extern]
pub fn get_proposal_constitution_epoch_status(
    _: (),
) -> ExternResult<ProposalConstitutionEpochStatus> {
    Ok(ProposalConstitutionEpochStatus {
        protocol: "mycelix-governance-proposal-constitution-epoch-v0.1".into(),
        binding_required_before_ballot_open: true,
        current_constitution_reverified_on_read: true,
        automatic_carry_forward_across_constitutions: false,
    })
}

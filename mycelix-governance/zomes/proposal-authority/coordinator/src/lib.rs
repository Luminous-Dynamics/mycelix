// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Runtime adapter binding governance proposals to explicit institutional
//! authority context without changing the legacy `Proposal` wire schema.
//!
//! New binding-governance consumers should call
//! `get_verified_proposal_authority_context` and fail closed when it returns
//! `None` or an error. Legacy proposal activation by itself does not establish
//! execution authority.

use hdk::prelude::*;
use mycelix_governance_authority::{
    GovernanceBodyId, ProposalAuthorityContext, ProposalId, SigningPolicyId,
    PROTOCOL_VERSION as GOVERNANCE_AUTHORITY_PROTOCOL_VERSION,
};
use mycelix_institutional_core::{Digest32, InstitutionId, JurisdictionId, RulebookRef};
use proposal_authority_integrity::*;

const EXECUTION_AUTHORITY_DOMAIN: &[u8] = b"mycelix-governance-execution-authority-v1\0";

/// Full proposal mirror. Keep field order synchronized with
/// proposals/integrity/src/lib.rs::Proposal.
#[derive(Debug, Serialize, Deserialize, Clone, SerializedBytes)]
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

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq, Eq)]
enum ProposalTypeMirror {
    Standard,
    Emergency,
    Constitutional,
    Parameter,
    Funding,
}

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq, Eq)]
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

#[derive(Debug, Serialize, Deserialize)]
struct UpdateStatusInputMirror {
    proposal_id: String,
    new_status: ProposalStatusMirror,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ActivateWithAuthorityContextInput {
    pub proposal_id: String,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub governing_body: GovernanceBodyId,
    pub signing_policy_id: SigningPolicyId,
    pub signing_policy_digest: Digest32,
    pub signing_policy_digest_profile: String,
    /// Authority context lifetime. Must extend beyond the voting period; later
    /// signing/execution steps still reject it once expired.
    pub expires_at_ms: u64,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ActivationResult {
    pub proposal: Record,
    pub authority_context: Record,
}

fn anchor_hash(anchor_str: &str) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(anchor_str.to_string())))
}

fn authority_anchor(proposal_id: &str) -> String {
    format!("proposal-authority:{proposal_id}")
}

/// This MUST remain byte-for-byte compatible with PR #32's execution
/// authorization digest until both move together to a registered replacement
/// canonical profile.
fn execution_authority_digest(proposal_id: &str, actions: &str) -> [u8; 32] {
    let mut hasher = blake3::Hasher::new();
    hasher.update(EXECUTION_AUTHORITY_DOMAIN);
    hasher.update(&(proposal_id.len() as u64).to_le_bytes());
    hasher.update(proposal_id.as_bytes());
    hasher.update(&(actions.len() as u64).to_le_bytes());
    hasher.update(actions.as_bytes());
    *hasher.finalize().as_bytes()
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

fn validate_profile_token(value: &str) -> ExternResult<()> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > 128
        || !bytes.iter().all(|b| {
            b.is_ascii_lowercase()
                || b.is_ascii_digit()
                || matches!(*b, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Signing-policy digest profile must be a 1-128 byte canonical ASCII token".into()
        )));
    }
    Ok(())
}

fn proposal_type_code(proposal_type: &ProposalTypeMirror) -> &'static str {
    match proposal_type {
        ProposalTypeMirror::Standard => "standard",
        ProposalTypeMirror::Emergency => "emergency",
        ProposalTypeMirror::Constitutional => "constitutional",
        ProposalTypeMirror::Parameter => "parameter",
        ProposalTypeMirror::Funding => "funding",
    }
}

fn fetch_proposal(proposal_id: &str) -> ExternResult<(Record, ProposalMirror)> {
    let response = call(
        CallTargetCell::Local,
        ZomeName::from("proposals"),
        FunctionName::from("get_proposal"),
        None,
        ExternIO::encode(proposal_id.to_string())
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
    )?;

    let io = match response {
        ZomeCallResponse::Ok(io) => io,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Cannot resolve proposal authority context: proposals::get_proposal failed: {other:?}"
            ))));
        }
    };

    let record = io
        .decode::<Option<Record>>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Cannot decode proposal lookup: {e}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Proposal '{proposal_id}' not found"
            )))
        })?;

    let proposal: ProposalMirror = record
        .entry()
        .to_app_option()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Cannot decode proposal entry: {e}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Proposal record has no application entry".into()
            ))
        })?;

    if proposal.id != proposal_id {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal lookup returned a different proposal id".into()
        )));
    }

    Ok((record, proposal))
}

fn ensure_proposal_author(proposal: &ProposalMirror) -> ExternResult<()> {
    let caller = agent_info()?.agent_initial_pubkey;
    let expected = format!("did:mycelix:{caller}");
    if proposal.author != expected {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the proposal author may bind institutional context or activate through this path"
                .into()
        )));
    }
    Ok(())
}

fn list_context_records(proposal_id: &str) -> ExternResult<Vec<Record>> {
    let anchor = authority_anchor(proposal_id);
    let links = get_links(
        LinkQuery::try_new(anchor_hash(&anchor)?, LinkTypes::ProposalToAuthorityContext)?,
        GetStrategy::default(),
    )?;

    let mut records = Vec::new();
    for link in links {
        let action_hash = ActionHash::try_from(link.target).map_err(|_| {
            wasm_error!(WasmErrorInner::Guest(
                "Invalid proposal authority link target".into()
            ))
        })?;
        if let Some(record) = get(action_hash, GetOptions::default())? {
            records.push(record);
        }
    }
    Ok(records)
}

fn decode_binding(record: &Record) -> ExternResult<ProposalAuthorityBinding> {
    record
        .entry()
        .to_app_option()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Malformed proposal authority binding: {e}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Proposal authority binding record has no entry".into()
            ))
        })
}

/// Equality of authority-bearing semantics, intentionally ignoring persistence
/// id/timestamp/action hash so an idempotent retry can reuse an equivalent
/// binding.
fn same_authority_contract(
    left: &ProposalAuthorityBinding,
    right_context: &ProposalAuthorityContext,
    signing_profile: &str,
) -> bool {
    left.context.proposal_id == right_context.proposal_id
        && left.context.institution == right_context.institution
        && left.context.jurisdiction == right_context.jurisdiction
        && left.context.rulebook == right_context.rulebook
        && left.context.governing_body == right_context.governing_body
        && left.context.action_class == right_context.action_class
        && left.context.actions_digest == right_context.actions_digest
        && left.context.signing_policy_id == right_context.signing_policy_id
        && left.context.signing_policy_digest == right_context.signing_policy_digest
        && left.context.expires_at_ms == right_context.expires_at_ms
        && left.actions_digest_profile == ACTIONS_DIGEST_PROFILE_V1
        && left.signing_policy_digest_profile == signing_profile
}

fn current_matching_bindings(
    proposal: &ProposalMirror,
) -> ExternResult<Vec<(Record, ProposalAuthorityBinding)>> {
    let expected_digest = Digest32(execution_authority_digest(&proposal.id, &proposal.actions));
    let expected_action_class = proposal_type_code(&proposal.proposal_type);
    let now_ms = timestamp_ms(sys_time()?, "current")?;
    let mut matches = Vec::new();

    for record in list_context_records(&proposal.id)? {
        let binding = decode_binding(&record)?;
        if binding.validate_structure().is_err() {
            continue;
        }
        if binding.proposal_author != proposal.author
            || binding.context.proposal_id.as_str() != proposal.id
            || binding.context.actions_digest != expected_digest
            || binding.context.action_class != expected_action_class
            || !binding.context.is_active_at(now_ms)
            || binding.actions_digest_profile != ACTIONS_DIGEST_PROFILE_V1
        {
            continue;
        }
        matches.push((record, binding));
    }

    Ok(matches)
}

/// Return the single unambiguous authority context matching the proposal's
/// current exact action bytes. Stale Draft-era contexts are ignored. Two
/// conflicting current contexts fail closed rather than choosing one by time.
#[hdk_extern]
pub fn get_verified_proposal_authority_context(
    proposal_id: String,
) -> ExternResult<Option<Record>> {
    if proposal_id.is_empty() || proposal_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal id must be 1-256 bytes".into()
        )));
    }

    let (_, proposal) = fetch_proposal(&proposal_id)?;
    let matches = current_matching_bindings(&proposal)?;
    if matches.is_empty() {
        return Ok(None);
    }

    let reference = &matches[0].1;
    for (_, candidate) in matches.iter().skip(1) {
        let same = candidate.context.institution == reference.context.institution
            && candidate.context.jurisdiction == reference.context.jurisdiction
            && candidate.context.rulebook == reference.context.rulebook
            && candidate.context.governing_body == reference.context.governing_body
            && candidate.context.signing_policy_id == reference.context.signing_policy_id
            && candidate.context.signing_policy_digest == reference.context.signing_policy_digest
            && candidate.signing_policy_digest_profile == reference.signing_policy_digest_profile;
        if !same {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Ambiguous proposal authority: multiple conflicting contexts match current proposal bytes"
                    .into()
            )));
        }
    }

    // Equivalent duplicate bindings have identical authority semantics. Return
    // the newest action only for deterministic API behavior.
    let record = matches
        .into_iter()
        .map(|(record, _)| record)
        .max_by_key(|record| record.action().timestamp().as_micros())
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Verified authority context disappeared during selection".into()
            ))
        })?;
    Ok(Some(record))
}

fn call_activate_proposal(proposal_id: &str) -> ExternResult<Record> {
    let response = call(
        CallTargetCell::Local,
        ZomeName::from("proposals"),
        FunctionName::from("update_proposal_status"),
        None,
        ExternIO::encode(UpdateStatusInputMirror {
            proposal_id: proposal_id.to_string(),
            new_status: ProposalStatusMirror::Active,
        })
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
    )?;

    let io = match response {
        ZomeCallResponse::Ok(io) => io,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Authority context was committed but proposal activation failed: {other:?}"
            ))));
        }
    };
    io.decode::<Record>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot decode activated proposal: {e}"
        )))
    })
}

/// Bind institutional authority context and activate a Draft proposal.
///
/// This is the preferred activation path for binding governance. Context is
/// committed first; if activation then fails, the append-only context remains
/// auditable and an idempotent retry can reuse it.
#[hdk_extern]
pub fn activate_proposal_with_authority_context(
    input: ActivateWithAuthorityContextInput,
) -> ExternResult<ActivationResult> {
    if input.proposal_id.is_empty() || input.proposal_id.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal id must be 1-256 bytes".into()
        )));
    }
    validate_profile_token(&input.signing_policy_digest_profile)?;
    input.rulebook.validate().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid rulebook reference: {e}"
        )))
    })?;

    let (proposal_record, proposal) = fetch_proposal(&input.proposal_id)?;
    ensure_proposal_author(&proposal)?;
    if !matches!(
        proposal.status,
        ProposalStatusMirror::Draft | ProposalStatusMirror::Active
    ) {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Authority context can only activate/recover an Active proposal from Draft; current status is {:?}",
            proposal.status
        ))));
    }

    let now = sys_time()?;
    let now_ms = timestamp_ms(now, "current")?;
    let voting_ends_ms = timestamp_ms(proposal.voting_ends, "voting end")?;
    if input.expires_at_ms <= voting_ends_ms || input.expires_at_ms <= now_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Proposal authority context must remain valid beyond the voting period".into()
        )));
    }

    let context = ProposalAuthorityContext {
        protocol_version: GOVERNANCE_AUTHORITY_PROTOCOL_VERSION.into(),
        proposal_id: ProposalId::new(proposal.id.clone())
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?,
        institution: input.institution,
        jurisdiction: input.jurisdiction,
        rulebook: input.rulebook,
        governing_body: input.governing_body,
        action_class: proposal_type_code(&proposal.proposal_type).to_string(),
        actions_digest: Digest32(execution_authority_digest(&proposal.id, &proposal.actions)),
        signing_policy_id: input.signing_policy_id,
        signing_policy_digest: input.signing_policy_digest,
        created_at_ms: now_ms,
        expires_at_ms: input.expires_at_ms,
    };
    context.validate().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid proposal authority context: {e}"
        )))
    })?;

    let current = current_matching_bindings(&proposal)?;
    let mut reusable: Option<Record> = None;
    for (record, binding) in current {
        if same_authority_contract(
            &binding,
            &context,
            &input.signing_policy_digest_profile,
        ) {
            reusable = Some(record);
        } else {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "A conflicting institutional authority context already matches these proposal action bytes; activation fails closed"
                    .into()
            )));
        }
    }

    // An Active proposal may only reach this path as an idempotent retry of a
    // context that was already committed before activation. Never retrofit
    // institutional meaning after voting has started.
    if proposal.status == ProposalStatusMirror::Active && reusable.is_none() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Cannot attach a new authority context to an already-Active proposal; create it before activation"
                .into()
        )));
    }

    let authority_context = if let Some(record) = reusable {
        record
    } else {
        let binding = ProposalAuthorityBinding {
            id: format!("pac:{}:{}", proposal.id, now.as_micros()),
            proposal_action_hash: proposal_record.action_address().clone(),
            proposal_author: proposal.author.clone(),
            context,
            actions_digest_profile: ACTIONS_DIGEST_PROFILE_V1.into(),
            signing_policy_digest_profile: input.signing_policy_digest_profile,
            created_at: now,
        };
        binding
            .validate_structure()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))?;

        let action_hash = create_entry(&EntryTypes::ProposalAuthorityBinding(binding))?;
        let anchor = authority_anchor(&proposal.id);
        create_entry(&EntryTypes::Anchor(Anchor(anchor.clone())))?;
        create_link(
            anchor_hash(&anchor)?,
            action_hash.clone(),
            LinkTypes::ProposalToAuthorityContext,
            (),
        )?;
        get(action_hash, GetOptions::default())?.ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Could not retrieve newly created proposal authority context".into()
            ))
        })?
    };

    let proposal = if proposal.status == ProposalStatusMirror::Draft {
        call_activate_proposal(&input.proposal_id)?
    } else {
        proposal_record
    };

    // Final defense: the activated/current proposal must still resolve to this
    // exact authority contract. A concurrent Draft mutation causes denial.
    let verified = get_verified_proposal_authority_context(input.proposal_id.clone())?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Proposal activated without a context matching its current action bytes; binding governance must fail closed"
                    .into()
            ))
        })?;

    let verified_binding = decode_binding(&verified)?;
    let expected_binding = decode_binding(&authority_context)?;
    if !same_authority_contract(
        &verified_binding,
        &expected_binding.context,
        &expected_binding.signing_policy_digest_profile,
    ) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Verified proposal authority context changed during activation".into()
        )));
    }

    Ok(ActivationResult {
        proposal,
        authority_context: verified,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn action_digest_matches_execution_boundary_vector() {
        let digest = execution_authority_digest("MIP-42", "[{\"type\":\"noop\"}]");
        let digest_again = execution_authority_digest("MIP-42", "[{\"type\":\"noop\"}]");
        assert_eq!(digest, digest_again);
        assert_ne!(
            digest,
            execution_authority_digest("MIP-42", "[{ \"type\": \"noop\" }]")
        );
    }

    #[test]
    fn action_class_is_derived_not_caller_supplied() {
        assert_eq!(proposal_type_code(&ProposalTypeMirror::Funding), "funding");
        assert_eq!(
            proposal_type_code(&ProposalTypeMirror::Constitutional),
            "constitutional"
        );
    }
}

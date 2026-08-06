// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Causal Contribution Coordinator Zome
//!
//! Business logic for ZK-verified causal contributions on Holochain 0.6.
//! This zome records verified contributions and updates reputation scores
//! through cross-zome calls to the reputation zome.

use causal_contribution_integrity::{
    AggregatedCausalScore, CausalContributionRecord, CivitasConfig, CommitteeEntry, EntryTypes,
    LinkTypes, ReceiptEntry,
};
use hdk::prelude::*;

// === Helpers ===

/// Helper to ensure a path exists and return its entry hash
fn ensure_path(path: Path, link_type: LinkTypes) -> ExternResult<EntryHash> {
    let typed = path.typed(link_type)?;
    typed.ensure()?;
    typed.path_entry_hash()
}

/// Link type filter for config anchor
fn config_link_filter() -> LinkTypeFilter {
    LinkTypeFilter::single_type(0.into(), (LinkTypes::CivitasConfigAnchor as u8).into())
}

/// Link type filter for committee anchor
fn committee_link_filter() -> LinkTypeFilter {
    LinkTypeFilter::single_type(0.into(), (LinkTypes::CommitteeAnchor as u8).into())
}

/// Link type filter for agent causal score
fn causal_score_link_filter() -> LinkTypeFilter {
    LinkTypeFilter::single_type(0.into(), (LinkTypes::AgentToCausalScore as u8).into())
}

/// Link type filter for contribution records
fn contribution_link_filter() -> LinkTypeFilter {
    LinkTypeFilter::single_type(
        0.into(),
        (LinkTypes::AgentToContributionRecords as u8).into(),
    )
}

/// Link type filter for round anchor
fn round_link_filter() -> LinkTypeFilter {
    LinkTypeFilter::single_type(0.into(), (LinkTypes::RoundAnchor as u8).into())
}

// === Config Management ===

const CONFIG_ANCHOR: &str = "civitas_config";
const COMMITTEE_ANCHOR: &str = "committees";
const ROUND_ANCHOR: &str = "rounds";

/// Initializes the Civitas config. Previously callable by ANY agent with zero
/// authorization -- any agent could bootstrap themselves as the system's sole admin, or
/// call this repeatedly to flood competing configs (get_civitas_config always resolves to
/// "latest by timestamp"). This coordinator-side fix rejects re-initialization once a
/// config already exists, closing the ongoing griefing vector.
///
/// NOTE (disclosed limitation, not fully resolved): this does NOT solve the deeper
/// first-admin bootstrap problem -- who legitimately gets to call this the very first
/// time still has no authorization check at all. That needs an out-of-band root of trust
/// (a hardcoded genesis agent, a membrane proof, or a governance vote) this DNA doesn't
/// currently have, and this is a coordinator-side check only -- a fully malicious modified
/// coordinator could still bypass it. See memory/mycelix_attribution_author_binding_jul8.md.
#[hdk_extern]
pub fn init_civitas_config(config: CivitasConfig) -> ExternResult<ActionHash> {
    if get_civitas_config(())?.is_some() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Civitas config already initialized".to_string()
        )));
    }

    let config_path = Path::from(CONFIG_ANCHOR);
    let config_entry_hash = ensure_path(config_path, LinkTypes::CivitasConfigAnchor)?;

    let action_hash = create_entry(&EntryTypes::CivitasConfig(config))?;

    create_link(
        config_entry_hash,
        action_hash.clone(),
        LinkTypes::CivitasConfigAnchor,
        (),
    )?;

    Ok(action_hash)
}

/// Gets the current Civitas config
#[hdk_extern]
pub fn get_civitas_config(_: ()) -> ExternResult<Option<CivitasConfig>> {
    let config_path = Path::from(CONFIG_ANCHOR);
    let config_entry_hash = ensure_path(config_path, LinkTypes::CivitasConfigAnchor)?;

    let links = get_links(
        LinkQuery::new(config_entry_hash, config_link_filter()),
        GetStrategy::default(),
    )?;

    if links.is_empty() {
        return Ok(None);
    }

    let latest_link = links
        .into_iter()
        .max_by_key(|l| l.timestamp)
        .ok_or(wasm_error!(WasmErrorInner::Guest("No config found".into())))?;

    let target_hash = ActionHash::try_from(latest_link.target)
        .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid action hash".into())))?;

    let record = get(target_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Config not found".into())
    ))?;

    let config: CivitasConfig = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid config entry".into()
        )))?;

    Ok(Some(config))
}

// === Committee Management ===

/// Input for registering a committee
#[derive(Serialize, Deserialize, Debug)]
pub struct RegisterCommitteeInput {
    pub round_id: u64,
    pub members: Vec<AgentPubKey>,
}

/// Registers a verification committee for a round. Previously callable by ANY agent with
/// zero authorization -- any agent could appoint an arbitrary committee for any round. Now
/// requires the caller to be a listed admin in the current Civitas config (coordinator-side
/// check -- see init_civitas_config's doc comment for why a full DHT-level admin check
/// isn't straightforward here).
#[hdk_extern]
pub fn register_committee(input: RegisterCommitteeInput) -> ExternResult<ActionHash> {
    let config = get_civitas_config(())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Civitas not configured".to_string()
    )))?;
    let caller = agent_info()?.agent_initial_pubkey;
    if !config.admins.contains(&caller) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only a Civitas admin may register a committee".to_string()
        )));
    }

    let committee = CommitteeEntry {
        round_id: input.round_id,
        members: input.members,
    };

    let committee_path = Path::from(format!("{}/{}", COMMITTEE_ANCHOR, input.round_id));
    let committee_entry_hash = ensure_path(committee_path, LinkTypes::CommitteeAnchor)?;

    let action_hash = create_entry(&EntryTypes::CommitteeEntry(committee))?;

    create_link(
        committee_entry_hash,
        action_hash.clone(),
        LinkTypes::CommitteeAnchor,
        (),
    )?;

    Ok(action_hash)
}

/// Gets the committee for a specific round
#[hdk_extern]
pub fn get_committee(round_id: u64) -> ExternResult<Option<CommitteeEntry>> {
    let committee_path = Path::from(format!("{}/{}", COMMITTEE_ANCHOR, round_id));
    let committee_entry_hash = ensure_path(committee_path, LinkTypes::CommitteeAnchor)?;

    let links = get_links(
        LinkQuery::new(committee_entry_hash, committee_link_filter()),
        GetStrategy::default(),
    )?;

    if links.is_empty() {
        return Ok(None);
    }

    let latest_link = links
        .into_iter()
        .max_by_key(|l| l.timestamp)
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "No committee found".into()
        )))?;

    let target_hash = ActionHash::try_from(latest_link.target)
        .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid action hash".into())))?;

    let record = get(target_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Committee not found".into())
    ))?;

    let committee: CommitteeEntry = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid committee entry".into()
        )))?;

    Ok(Some(committee))
}

// === Receipt & Contribution Management ===

/// Input for submitting a receipt with attestations
#[derive(Serialize, Deserialize, Debug)]
pub struct SubmitReceiptInput {
    pub data: Vec<u8>,
    pub attesters: Vec<AgentPubKey>,
    pub signatures: Vec<Signature>,
    pub meta: Option<Vec<u8>>,
}

/// Submits a ZK receipt with committee attestations
#[hdk_extern]
pub fn submit_receipt(input: SubmitReceiptInput) -> ExternResult<EntryHash> {
    // Verify minimum attestations
    let config = get_civitas_config(())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Civitas not configured".into()
    )))?;

    if input.attesters.len() < config.min_attestations as usize {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Insufficient attestations: {} < {}",
            input.attesters.len(),
            config.min_attestations
        ))));
    }

    let receipt = ReceiptEntry {
        data: input.data,
        attesters: input.attesters,
        signatures: input.signatures,
        meta: input.meta,
    };

    let action_hash = create_entry(&EntryTypes::ReceiptEntry(receipt))?;

    // Get the entry hash from the action
    let record = get(action_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Failed to get created entry".into())
    ))?;

    let entry_hash = record
        .action()
        .entry_hash()
        .ok_or(wasm_error!(WasmErrorInner::Guest("No entry hash".into())))?
        .clone();

    Ok(entry_hash)
}

/// Link type filter for proof_hash -> contribution (replay guard)
fn proof_contribution_filter() -> LinkTypeFilter {
    LinkTypeFilter::single_type(0.into(), (LinkTypes::ProofHashToContribution as u8).into())
}

/// Input for recording a causal contribution
#[derive(Serialize, Deserialize, Debug)]
pub struct RecordContributionInput {
    pub agent: AgentPubKey,
    pub round_id: u64,
    pub contribution_score: f64,
    pub proof_hash: EntryHash,
}

/// Records a verified causal contribution.
///
/// # Security redesign (2026-07-27, MASTER_ROADMAP.md P0-#1)
///
/// Previously accepted an arbitrary client-supplied `agent`+
/// `contribution_score` with zero authentication of the caller, and never
/// even dereferenced `proof_hash` -- any agent could record a fabricated
/// contribution for any other agent, pointing at any (possibly nonexistent)
/// proof_hash, which then flowed straight into reputation via a cross-zome
/// call. Now requires the caller to be one of the referenced receipt's real
/// attesters (the only entities who cryptographically signed off on it --
/// see `causal_contribution_integrity::validate_receipt_entry`), and
/// requires the receipt to actually exist. The raw contribution *score*
/// itself is still trusted from the attester's honest reporting (the
/// receipt's `data` is opaque ZK proof bytes this zome cannot parse to
/// derive a score from) -- this closes impersonation, not miscalibration by
/// a genuine attester. Also adds a best-effort replay guard so the same
/// receipt can't be used to record more than one contribution.
#[hdk_extern]
pub fn record_contribution(input: RecordContributionInput) -> ExternResult<ActionHash> {
    let caller = agent_info()?.agent_initial_pubkey;

    let receipt_record =
        get(input.proof_hash.clone(), GetOptions::default())?.ok_or(wasm_error!(
            WasmErrorInner::Guest("proof_hash does not resolve to any entry".into())
        ))?;
    let receipt: ReceiptEntry = receipt_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "proof_hash does not resolve to a ReceiptEntry".into()
        )))?;

    if !receipt.attesters.contains(&caller) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only an attester listed on the referenced receipt may record a contribution from it"
                .into()
        )));
    }

    // Best-effort replay guard (disclosed limitation, see doc comment
    // above): refuse to record a second contribution from the same receipt.
    let already_recorded = get_links(
        LinkQuery::new(input.proof_hash.clone(), proof_contribution_filter()),
        GetStrategy::default(),
    )?;
    if !already_recorded.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "A contribution has already been recorded from this receipt".into()
        )));
    }

    let now = sys_time()?;

    let contribution = CausalContributionRecord {
        agent: input.agent.clone(),
        round_id: input.round_id,
        contribution_score: input.contribution_score,
        proof_hash: input.proof_hash.clone(),
        timestamp: now,
    };

    // Create the contribution record
    let action_hash = create_entry(&EntryTypes::CausalContributionRecord(contribution))?;

    // Mark the receipt as consumed (best-effort replay guard).
    create_link(
        input.proof_hash,
        action_hash.clone(),
        LinkTypes::ProofHashToContribution,
        (),
    )?;

    // Link from agent path to contribution
    let agent_path = Path::from(format!("agents/{}/contributions", input.agent));
    let agent_entry_hash = ensure_path(agent_path, LinkTypes::AgentToContributionRecords)?;

    create_link(
        agent_entry_hash,
        action_hash.clone(),
        LinkTypes::AgentToContributionRecords,
        (),
    )?;

    // Link from round to contribution
    let round_path = Path::from(format!("{}/{}", ROUND_ANCHOR, input.round_id));
    let round_entry_hash = ensure_path(round_path, LinkTypes::RoundAnchor)?;

    create_link(
        round_entry_hash,
        action_hash.clone(),
        LinkTypes::RoundAnchor,
        (),
    )?;

    // Update the aggregated score
    update_aggregated_score(input.agent.clone(), input.contribution_score)?;

    // Cross-zome call to update reputation: pass this record's own
    // ActionHash, not raw agent/score -- civitas_reputation derives both
    // from the record itself via must_get_valid_record.
    let update_input = serde_json::json!({
        "contribution_record_hash": action_hash
    });

    // Call the reputation zome (ignore errors as it may not be installed)
    let _result = call(
        CallTargetCell::Local,
        ZomeName::from("civitas_reputation"),
        FunctionName::from("update_causal_reputation"),
        None,
        update_input,
    );

    Ok(action_hash)
}

/// Updates the aggregated causal score for an agent
fn update_aggregated_score(agent: AgentPubKey, contribution: f64) -> ExternResult<ActionHash> {
    let now = sys_time()?;

    let agent_path = Path::from(format!("agents/{}/score", agent));
    let agent_entry_hash = ensure_path(agent_path, LinkTypes::AgentToCausalScore)?;

    let links = get_links(
        LinkQuery::new(agent_entry_hash.clone(), causal_score_link_filter()),
        GetStrategy::default(),
    )?;

    let (mut score, maybe_prev) = if links.is_empty() {
        (
            AggregatedCausalScore {
                agent: agent.clone(),
                total_score: 0.0,
                rounds_participated: 0,
                last_updated: now,
            },
            None,
        )
    } else {
        let latest_link = links
            .into_iter()
            .max_by_key(|l| l.timestamp)
            .ok_or(wasm_error!(WasmErrorInner::Guest("No score found".into())))?;

        let target_hash = ActionHash::try_from(latest_link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid action hash".into())))?;

        let record = get(target_hash.clone(), GetOptions::default())?
            .ok_or(wasm_error!(WasmErrorInner::Guest("Score not found".into())))?;

        let prev_score: AggregatedCausalScore = record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Invalid score entry".into()
            )))?;

        (prev_score, Some(target_hash))
    };

    score.total_score += contribution;
    score.rounds_participated += 1;
    score.last_updated = now;

    let action_hash = if let Some(prev_hash) = maybe_prev {
        update_entry(prev_hash, &score)?
    } else {
        let hash = create_entry(&EntryTypes::AggregatedCausalScore(score))?;
        create_link(
            agent_entry_hash,
            hash.clone(),
            LinkTypes::AgentToCausalScore,
            (),
        )?;
        hash
    };

    Ok(action_hash)
}

/// Gets the aggregated causal score for an agent
#[hdk_extern]
pub fn get_aggregated_score(agent: AgentPubKey) -> ExternResult<Option<AggregatedCausalScore>> {
    let agent_path = Path::from(format!("agents/{}/score", agent));
    let agent_entry_hash = ensure_path(agent_path, LinkTypes::AgentToCausalScore)?;

    let links = get_links(
        LinkQuery::new(agent_entry_hash, causal_score_link_filter()),
        GetStrategy::default(),
    )?;

    if links.is_empty() {
        return Ok(None);
    }

    let latest_link = links
        .into_iter()
        .max_by_key(|l| l.timestamp)
        .ok_or(wasm_error!(WasmErrorInner::Guest("No score found".into())))?;

    let target_hash = ActionHash::try_from(latest_link.target)
        .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid action hash".into())))?;

    let record = get(target_hash, GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Score not found".into())))?;

    let score: AggregatedCausalScore = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid score entry".into()
        )))?;

    Ok(Some(score))
}

/// Gets contribution records for a specific round
#[hdk_extern]
pub fn get_round_contributions(round_id: u64) -> ExternResult<Vec<CausalContributionRecord>> {
    let round_path = Path::from(format!("{}/{}", ROUND_ANCHOR, round_id));
    let round_entry_hash = ensure_path(round_path, LinkTypes::RoundAnchor)?;

    let links = get_links(
        LinkQuery::new(round_entry_hash, round_link_filter()),
        GetStrategy::default(),
    )?;

    let mut contributions = Vec::with_capacity(links.len());

    for link in links {
        let target_hash = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid action hash".into())))?;

        if let Some(record) = get(target_hash, GetOptions::default())? {
            if let Some(contribution) = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                contributions.push(contribution);
            }
        }
    }

    Ok(contributions)
}

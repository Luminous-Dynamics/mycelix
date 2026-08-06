// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Civitas Reputation Coordinator Zome
//!
//! Business logic for updating and querying agent reputation scores on
//! Holochain 0.6.
//!
//! # Security redesign (2026-07-27, MASTER_ROADMAP.md P0-#1)
//!
//! `update_causal_reputation` no longer accepts a raw client-supplied
//! `agent`+`score` -- it takes a reference to an already-validated
//! `CausalContributionRecord` and derives the agent/score from that record's
//! own content via `must_get_valid_record`, which transitively enforces
//! `causal_contribution`'s own author-binding fix. See
//! `civitas_reputation_integrity`'s module doc for the full rationale and
//! disclosed residual limitations (replay-guard is best-effort, not
//! airtight).

use civitas_reputation_integrity::{
    CausalContributionRecordMirror, CivitasReputationScore, EntryTypes, INITIAL_REPUTATION,
    LinkTypes, apply_contribution,
};
use hdk::prelude::*;

/// Helper function to get the link type filter
fn reputation_link_filter() -> LinkTypeFilter {
    LinkTypeFilter::single_type(0.into(), (LinkTypes::AgentToReputationScore as u8).into())
}

fn contribution_applied_filter() -> LinkTypeFilter {
    LinkTypeFilter::single_type(
        0.into(),
        (LinkTypes::ContributionRecordApplied as u8).into(),
    )
}

/// Helper function to ensure a path exists and return its entry hash
fn ensure_path(path: Path, link_type: LinkTypes) -> ExternResult<EntryHash> {
    let typed = path.typed(link_type)?;
    typed.ensure()?;
    typed.path_entry_hash()
}

/// Input for the update_causal_reputation function. Previously
/// `{ agent: AgentPubKey, causal_contribution_score: f64 }` -- both
/// raw, unauthenticated client input. Now a single reference to an
/// already-validated record; agent and score are derived from it.
#[derive(Serialize, Deserialize, Debug)]
pub struct UpdateCausalReputationInput {
    pub contribution_record_hash: ActionHash,
}

/// Updates an agent's causal reputation score, derived from a real,
/// already-validated `CausalContributionRecord` -- never from raw caller
/// input. Called by `causal_contribution::record_contribution` via
/// cross-zome call, passing the record it just created.
#[hdk_extern]
pub fn update_causal_reputation(input: UpdateCausalReputationInput) -> ExternResult<ActionHash> {
    // must_get_valid_record only succeeds once causal_contribution's own
    // validation has passed for this record -- this is the actual security
    // boundary the whole redesign rests on, not a convenience fetch.
    let contribution_record = must_get_valid_record(input.contribution_record_hash.clone())?;
    let contribution: CausalContributionRecordMirror = contribution_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "contribution_record_hash does not resolve to a CausalContributionRecord".into()
        )))?;

    // Best-effort replay guard (see integrity zome's module doc for why this
    // isn't a fully airtight, DHT-wide guarantee): refuse to apply the same
    // contribution record to reputation twice.
    let already_applied = get_links(
        LinkQuery::new(
            input.contribution_record_hash.clone(),
            contribution_applied_filter(),
        ),
        GetStrategy::default(),
    )?;
    if !already_applied.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "This contribution record has already been applied to reputation".into()
        )));
    }

    let now = sys_time()?;
    let agent = contribution.agent.clone();

    let agent_path = Path::from(format!("agents/{}", agent));
    let agent_entry_hash = ensure_path(agent_path, LinkTypes::AgentToReputationScore)?;

    let links = get_links(
        LinkQuery::new(agent_entry_hash.clone(), reputation_link_filter()),
        GetStrategy::default(),
    )?;

    let (reputation, rounds_participated, maybe_prev_hash) = if links.is_empty() {
        let (reputation, rounds) =
            apply_contribution(INITIAL_REPUTATION, 0, contribution.contribution_score);
        (reputation, rounds, None)
    } else {
        let latest_link = links
            .into_iter()
            .max_by_key(|l| l.timestamp)
            .ok_or(wasm_error!(WasmErrorInner::Guest("No links found".into())))?;

        let target_hash = ActionHash::try_from(latest_link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid action hash".into())))?;

        let record = get(target_hash.clone(), GetOptions::default())?.ok_or(wasm_error!(
            WasmErrorInner::Guest("Record not found".into())
        ))?;

        let prev: CivitasReputationScore = record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(e))?
            .ok_or(wasm_error!(WasmErrorInner::Guest("Invalid entry".into())))?;

        let (reputation, rounds) = apply_contribution(
            prev.reputation,
            prev.rounds_participated,
            contribution.contribution_score,
        );
        (reputation, rounds, Some(target_hash))
    };

    let score = CivitasReputationScore {
        agent: agent.clone(),
        reputation,
        rounds_participated,
        last_updated: now,
        justifying_record: input.contribution_record_hash.clone(),
    };

    let action_hash = if let Some(prev_hash) = maybe_prev_hash {
        update_entry(prev_hash, &score)?
    } else {
        let hash = create_entry(&EntryTypes::CivitasReputationScore(score))?;
        create_link(
            agent_entry_hash,
            hash.clone(),
            LinkTypes::AgentToReputationScore,
            (),
        )?;
        hash
    };

    // Mark the contribution record as applied (best-effort replay guard).
    create_link(
        input.contribution_record_hash,
        action_hash.clone(),
        LinkTypes::ContributionRecordApplied,
        (),
    )?;

    Ok(action_hash)
}

/// Gets an agent's causal reputation score.
#[hdk_extern]
pub fn get_causal_reputation(agent: AgentPubKey) -> ExternResult<Option<CivitasReputationScore>> {
    let agent_path = Path::from(format!("agents/{}", agent));
    let agent_entry_hash = ensure_path(agent_path, LinkTypes::AgentToReputationScore)?;

    let links = get_links(
        LinkQuery::new(agent_entry_hash, reputation_link_filter()),
        GetStrategy::default(),
    )?;

    if links.is_empty() {
        return Ok(None);
    }

    let latest_link = links
        .into_iter()
        .max_by_key(|l| l.timestamp)
        .ok_or(wasm_error!(WasmErrorInner::Guest("No links found".into())))?;

    let target_hash = ActionHash::try_from(latest_link.target)
        .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid action hash".into())))?;

    let record = get(target_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Record not found".into())
    ))?;

    let score: CivitasReputationScore = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Invalid entry".into())))?;

    Ok(Some(score))
}

/// Gets reputation scores for multiple agents (batch query)
#[hdk_extern]
pub fn get_reputations_batch(
    agents: Vec<AgentPubKey>,
) -> ExternResult<Vec<(AgentPubKey, Option<CivitasReputationScore>)>> {
    let mut results = Vec::with_capacity(agents.len());

    for agent in agents {
        let score = get_causal_reputation(agent.clone())?;
        results.push((agent, score));
    }

    Ok(results)
}

/// Calculates the trust threshold for an agent based on their reputation
#[hdk_extern]
pub fn get_trust_threshold(agent: AgentPubKey) -> ExternResult<f64> {
    match get_causal_reputation(agent)? {
        Some(score) => {
            // High reputation = high trust threshold
            // Low reputation = low trust threshold (more scrutiny)
            let base_threshold = 0.5;
            let reputation_factor = score.reputation * 0.4; // Max 0.4 bonus
            let rounds_factor = (score.rounds_participated as f64 / 100.0).min(0.1); // Max 0.1 bonus

            Ok(base_threshold + reputation_factor + rounds_factor)
        }
        None => Ok(0.5), // Default threshold for new agents
    }
}

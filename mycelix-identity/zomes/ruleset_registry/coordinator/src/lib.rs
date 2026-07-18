// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Ruleset Registry Coordinator Zome
//!
//! Publish/query a federated marketplace of `RulesetRecord`s — see
//! `ruleset_registry_integrity` module docs and
//! `WARDED_NODE_DESIGN_2026-07-11.md` §"5b policy" for why there is
//! deliberately no "canonical ruleset" concept anywhere in this zome.
//! A warded node's guardian, not this registry, decides which publisher(s)
//! to trust (via `web-of-trust`) before ever calling
//! `symthaea_memetics::MemeticImmuneSystem::vaccinate_ruleset` on what it
//! fetches from here.

use hdk::prelude::*;
use mycelix_bridge_common::civic_requirement_basic;
use ruleset_registry_integrity::*;

use mycelix_zome_helpers as _;

/// Input for [`publish_ruleset`] — `publisher` is deliberately absent; the
/// coordinator sets it from `agent_info()` so the DHT-level check in the
/// integrity zome (`action.author() == publisher`) can never be bypassed by
/// a caller supplying someone else's identity.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PublishRulesetInput {
    pub name: String,
    pub version: String,
    pub source: String,
    pub entries: Vec<RulesetEntryRecord>,
}

fn anchor_hash(anchor_str: &str) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(anchor_str.to_string())))
}

fn ensure_anchor(anchor_str: &str) -> ExternResult<EntryHash> {
    create_entry(&EntryTypes::Anchor(Anchor(anchor_str.to_string())))?;
    anchor_hash(anchor_str)
}

/// Publish a new `RulesetRecord`. Any agent meeting the (low) civic bar may
/// publish — this is deliberately a low bar (spam/Sybil resistance only,
/// not a quality or "trustworthiness" gate): quality/trust is the
/// *guardian's* decision at import time, not this registry's to pre-judge.
#[hdk_extern]
pub fn publish_ruleset(input: PublishRulesetInput) -> ExternResult<Record> {
    let _eligibility = mycelix_zome_helpers::require_civic(
        "identity_bridge",
        &civic_requirement_basic(),
        "publish_ruleset",
    )?;

    let publisher = agent_info()?.agent_initial_pubkey;
    let record = RulesetRecord {
        publisher: publisher.clone(),
        name: input.name,
        version: input.version,
        source: input.source,
        entries: input.entries,
    };

    let action_hash = create_entry(&EntryTypes::RulesetRecord(record))?;

    create_link(
        publisher,
        action_hash.clone(),
        LinkTypes::PublisherToRulesets,
        (),
    )?;

    let all_anchor = ensure_anchor("all_rulesets")?;
    create_link(all_anchor, action_hash.clone(), LinkTypes::AllRulesets, ())?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Record not found immediately after create".into()
    )))
}

/// Fetch one ruleset by its action hash.
#[hdk_extern]
pub fn get_ruleset(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}

/// All rulesets a given publisher has ever published, newest-link-first as
/// returned by `get_links` (callers wanting a specific version should filter
/// on `RulesetRecord.version` themselves — this zome doesn't interpret
/// version strings, e.g. as semver, on purpose: publishers may use dates,
/// hashes, or anything else).
#[hdk_extern]
pub fn get_rulesets_by_publisher(publisher: AgentPubKey) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(publisher, LinkTypes::PublisherToRulesets)?,
        GetStrategy::default(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(target) = link.target.into_action_hash() {
            if let Some(record) = get(target, GetOptions::default())? {
                records.push(record);
            }
        }
    }
    Ok(records)
}

/// Every published ruleset, up to `limit` — the "browse the marketplace"
/// entry point. A guardian's tooling is expected to use this to discover
/// publishers, then decide which to actually trust via `web-of-trust`
/// before ever importing their content; this call surfaces everything
/// published, trusted or not, by design (no pre-filtering here — see
/// module docs on why this registry doesn't pick winners).
#[hdk_extern]
pub fn get_all_rulesets(limit: u32) -> ExternResult<Vec<Record>> {
    let all_anchor = anchor_hash("all_rulesets")?;
    let links = get_links(
        LinkQuery::try_new(all_anchor, LinkTypes::AllRulesets)?,
        GetStrategy::default(),
    )?;

    let mut records = Vec::new();
    for link in links.into_iter().take(limit as usize) {
        if let Some(target) = link.target.into_action_hash() {
            if let Some(record) = get(target, GetOptions::default())? {
                records.push(record);
            }
        }
    }
    Ok(records)
}

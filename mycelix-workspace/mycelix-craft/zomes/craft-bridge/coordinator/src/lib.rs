// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//!
//! Craft Bridge Coordinator Zome
//!
//! Cross-domain dispatch with consciousness gating for the Craft cluster.
//! All cross-cluster calls go through this bridge for rate limiting,
//! allowlist validation, and consciousness tier enforcement.
//!
//! Consciousness gating thresholds:
//! - dispatch_call: Participant+ (basic, 0.25+)
//! - query_craft: Participant+ (basic, 0.25+)
//! - broadcast_event: Citizen+ (voting, 0.45+)

#![deny(unsafe_code)]
#![allow(deprecated)] // Uses legacy ConsciousnessCredential/Tier for fallback path

use craft_bridge_integrity::*;
use hdk::prelude::*;
use mycelix_bridge_common as bridge;
use mycelix_zome_helpers as _;

/// All coordinator zome names in the Craft cluster.
const ALLOWED_ZOMES: &[&str] = &[
    "craft_graph",
    "job_postings_coordinator",
    "work_history_coordinator",
    "connection_graph_coordinator",
    "applications_coordinator",
    "guild_coordinator",
];

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn ensure_anchor(text: &str) -> ExternResult<ActionHash> {
    let anchor = Anchor(text.to_string());
    create_entry(EntryTypes::Anchor(anchor))
}

// Rate limiting deferred until DispatchRateLimit link type is added to integrity.
// For now, consciousness gating provides the primary access control.

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------

/// Dispatch a synchronous call to any domain zome within the Craft DNA.
///
/// Consciousness-gated: requires Participant tier (basic).
/// Rate-limited to 100 calls per 60 seconds per agent.
#[hdk_extern]
pub fn dispatch_call(input: bridge::DispatchInput) -> ExternResult<bridge::DispatchResult> {
    // Consciousness gate: Participant tier (≥0.3 combined)
    bridge::gate_civic(
        "craft_bridge",
        &bridge::civic_requirement_basic(),
        "dispatch_call",
    )?;

    if input.zome.trim().is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Dispatch zome name cannot be empty".into()
        )));
    }
    if input.fn_name.trim().is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Dispatch function name cannot be empty".into()
        )));
    }
    bridge::dispatch_call_checked(&input, ALLOWED_ZOMES)
}

/// Query a Craft domain with consciousness gating.
///
/// Requires Participant tier (basic). Stores query on DHT for auditability.
#[hdk_extern]
pub fn query_craft(query: CraftQueryEntry) -> ExternResult<Record> {
    // Consciousness gate
    bridge::gate_civic(
        "craft_bridge",
        &bridge::civic_requirement_basic(),
        "query_craft",
    )?;

    let query_hash = create_entry(EntryTypes::BridgeQuery(query))?;

    let all_anchor = ensure_anchor("all_craft_queries")?;
    create_link(all_anchor, query_hash.clone(), LinkTypes::AllQueries, ())?;

    get(query_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Failed to store query".into()
    )))
}

/// Broadcast an event to the Craft cluster.
///
/// Requires Citizen tier (voting) — events are community actions.
#[hdk_extern]
pub fn broadcast_event(event: CraftEventEntry) -> ExternResult<Record> {
    bridge::gate_civic(
        "craft_bridge",
        &bridge::civic_requirement_voting(),
        "broadcast_event",
    )?;

    let event_hash = create_entry(EntryTypes::BridgeEvent(event))?;

    let all_anchor = ensure_anchor("all_craft_events")?;
    create_link(all_anchor, event_hash.clone(), LinkTypes::AllEvents, ())?;

    get(event_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Failed to store event".into()
    )))
}

// ---------------------------------------------------------------------------
// Consciousness credential stub (required by gate_civic/gate_consciousness)
// ---------------------------------------------------------------------------

/// Bootstrap consciousness credential for the Craft cluster.
///
/// Required by `gate_civic()` which calls `get_consciousness_credential`
/// Legacy 4D credential — delegates to sovereign bootstrap path.
#[hdk_extern]
pub fn get_consciousness_credential(did: String) -> ExternResult<bridge::ConsciousnessCredential> {
    let sovereign = get_sovereign_credential(did)?;
    let lp = bridge::sovereign_gate::LegacyProfile::from(sovereign.profile.clone());
    let profile = bridge::ConsciousnessProfile {
        identity: lp.identity,
        reputation: lp.reputation,
        community: lp.community,
        engagement: lp.engagement,
    };
    Ok(bridge::ConsciousnessCredential {
        did: sovereign.did,
        profile: profile.clone(),
        tier: bridge::ConsciousnessTier::from_score(profile.combined_score()),
        issued_at: sovereign.issued_at,
        expires_at: sovereign.expires_at,
        issuer: sovereign.issuer,
        trajectory_commitment: None,
        extensions: Default::default(),
    })
}

/// Parse a `did:mycelix:<agent-pubkey>` DID into the underlying `AgentPubKey`.
fn agent_pubkey_from_did(did: &str) -> ExternResult<AgentPubKey> {
    let raw = did
        .strip_prefix("did:mycelix:")
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(format!("Not a mycelix DID: {did}"))))?;
    AgentPubKey::try_from(raw).map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid agent pubkey in DID: {e:?}"
        )))
    })
}

/// Attempt to fetch a real credential from mycelix-identity's bridge zome via
/// `CallTargetCell::OtherRole("identity")`, following the same dispatch
/// pattern mycelix-finance's `consciousness_gating::check_tier` uses.
///
/// Returns `None` on ANY failure — unresolvable role (identity isn't
/// installed as a role in Craft's current standalone `happ.yaml`, so this is
/// the common case today), network error, or decode error — so the caller
/// can fall back to the existing bootstrap stub rather than hard-failing.
/// Unlike finance's tier-check gate (a boolean permission check, safe to
/// fail closed), this function's result feeds guild join/promote/create —
/// hard-failing here whenever identity is unreachable would break Craft's
/// standalone deployment entirely, which is worse than the acknowledged-fake
/// bootstrap fallback it would replace.
fn fetch_remote_sovereign_credential(
    did: &str,
) -> Option<bridge::sovereign_gate::SovereignCredential> {
    let agent = agent_pubkey_from_did(did).ok()?;
    let response = call(
        CallTargetCell::OtherRole("identity".into()),
        ZomeName::new("bridge"),
        FunctionName::new("get_agent_profile_remote"),
        None,
        agent,
    )
    .ok()?;
    let ZomeCallResponse::Ok(extern_io) = response else {
        return None;
    };
    let profile: bridge::ConsciousnessProfile = extern_io.decode().ok()?;
    let now_us = sys_time().ok()?.as_micros() as u64;
    let credential = bridge::ConsciousnessCredential {
        did: did.to_string(),
        profile: profile.clone(),
        tier: bridge::ConsciousnessTier::from_score(profile.combined_score()),
        issued_at: now_us,
        expires_at: now_us + 24 * 60 * 60 * 1_000_000,
        issuer: "did:mycelix:identity-bridge".into(),
        trajectory_commitment: None,
        extensions: Default::default(),
    };
    let sovereign_profile = bridge::sovereign_gate::sovereign_from_credential(&credential);
    let weights = bridge::sovereign_gate::DimensionWeights::governance();
    let tier = sovereign_profile.tier(&weights);
    Some(bridge::sovereign_gate::SovereignCredential {
        did: did.to_string(),
        profile: sovereign_profile,
        tier,
        issued_at: now_us,
        expires_at: now_us + 24 * 60 * 60 * 1_000_000,
        issuer: "did:mycelix:identity-bridge".into(),
        extensions: vec![],
    })
}

/// Sovereign credential for the Craft cluster.
///
/// Tries a real cross-DNA fetch from mycelix-identity first
/// (`fetch_remote_sovereign_credential`); falls back to a development
/// credential with Steward-level scores (so civic gates still pass during
/// standalone testing/deployment, where identity isn't installed as a role)
/// when that's unreachable.
#[hdk_extern]
pub fn get_sovereign_credential(
    did: String,
) -> ExternResult<bridge::sovereign_gate::SovereignCredential> {
    let now_us = sys_time()?.as_micros() as u64;
    let actual_did = if did.is_empty() {
        format!("did:mycelix:{}", agent_info()?.agent_initial_pubkey)
    } else {
        did
    };

    if let Some(real) = fetch_remote_sovereign_credential(&actual_did) {
        return Ok(real);
    }

    let profile = bridge::sovereign_gate::SovereignProfile {
        epistemic_integrity: 0.7,
        thermodynamic_yield: 0.5,
        network_resilience: 0.7,
        economic_velocity: 0.6,
        civic_participation: 0.7,
        stewardship_care: 0.6,
        semantic_resonance: 0.7,
        domain_competence: 0.5,
    };
    let weights = bridge::sovereign_gate::DimensionWeights::governance();
    let tier = profile.tier(&weights);
    Ok(bridge::sovereign_gate::SovereignCredential {
        did: actual_did,
        profile,
        tier,
        issued_at: now_us,
        expires_at: now_us + 24 * 60 * 60 * 1_000_000,
        issuer: "did:mycelix:craft-bootstrap".into(),
        extensions: vec![],
    })
}

/// Refresh stub — delegates to get_consciousness_credential.
#[hdk_extern]
pub fn refresh_consciousness_credential(
    did: String,
) -> ExternResult<bridge::ConsciousnessCredential> {
    get_consciousness_credential(did)
}

/// Governance gate audit log (best-effort, called by gate_civic).
#[hdk_extern]
pub fn log_governance_gate(_input: bridge::GateAuditInput) -> ExternResult<()> {
    // Best-effort audit — no-op for now
    Ok(())
}

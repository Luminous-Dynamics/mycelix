#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// issuer-trust-tier coordinator — classify credential issuers into tiers.
//
// The tier is a hint for verifiers: "this issuer is sovereign" means
// a credential from this issuer carries more weight than a peer
// classification. The tier NEVER influences Mycelix governance weight
// or consciousness-tier computation. It lives here as metadata for
// the lawful-identity credential import + verification flow only.
//
// AUTHORIZATION MODEL (fixed 2026-07-10; see integrity crate for the
// DHT-level enforcement of part (a)). A classification is a PERSONAL,
// per-agent view of an issuer -- matching `IssuerTier`'s own doc
// comments ("user-configurable list; never canonical, never
// hardcoded"). There is no admin/allowlist/genesis-agent concept
// anywhere in this hApp (checked `legal_did` and `cross_did_zkp`: both
// use a plain per-agent-anchor pattern, not a permission list), so any
// live agent MAY classify any issuer -- but:
//   (a) every classification is cryptographically bound to its real
//       author via `classified_by` (rejected at validation time
//       otherwise -- see integrity `validate_classification`), so no
//       agent can forge a classification "as" a different classifier.
//   (b) `lookup_tier` no longer collapses every classifier's opinion
//       across the whole DHT into one global "latest write wins"
//       value. Previously, one forged/wrong classification with a
//       fresher timestamp silently overrode what EVERY verifier
//       network-wide saw for that issuer. It now resolves only from
//       the CALLING agent's own classification history (`caller()`),
//       so "latest wins" only ever applies within an agent's own
//       history, which only that agent's author-bound writes can
//       populate.
//   (c) `list_classifications_for_issuer` is new: it exposes every
//       classifier's claim for an issuer, attributed by agent, so a
//       caller who wants the full picture (not just their own view)
//       sees WHO claimed WHAT instead of one collapsed "winner".

use hdk::prelude::*;
use issuer_trust_tier_integrity::{EntryTypes, IssuerClassification, IssuerTier, LinkTypes};

// ============================================================================
// Helpers
// ============================================================================

fn now_iso_8601() -> ExternResult<String> {
    let ts = sys_time()?;
    let (secs, nanos) = ts.as_seconds_and_nanos();
    Ok(format!("{}.{:09}Z", secs, nanos))
}

/// Path anchoring classifications for a specific tier.
fn tier_path(tier: IssuerTier) -> Path {
    Path::from(format!("tier/{}", tier.as_str()))
}

/// Global (unscoped-by-classifier) path anchoring every classification
/// ever made for a specific issuer. Deliberately not a source of truth
/// -- see `list_classifications_for_issuer` -- because each linked
/// entry carries its own author-bound `classified_by`.
fn issuer_path(issuer_did: &str) -> Path {
    Path::from(format!("issuer/{}", issuer_did))
}

/// Get the calling agent's pubkey. Mirrors the `legal_did` zome's
/// `caller()` helper -- used both to author-bind new classifications
/// and as the base for the per-agent `AgentIssuerAnchor` link that
/// scopes `lookup_tier` to the caller's own classification history.
fn caller() -> ExternResult<AgentPubKey> {
    agent_info().map(|info| info.agent_initial_pubkey)
}

// ============================================================================
// Classify
// ============================================================================

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ClassifyIssuerInput {
    pub issuer_did: String,
    pub tier: IssuerTier,
    pub rationale: Option<String>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ClassifyIssuerOutput {
    pub action_hash: ActionHash,
}

#[hdk_extern]
pub fn classify_issuer(input: ClassifyIssuerInput) -> ExternResult<ClassifyIssuerOutput> {
    if input.issuer_did.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "issuer_did must not be empty".to_string()
        )));
    }

    // Author-bind this classification to the real caller (never taken
    // from `input` -- there is no user-suppliable "classifier" field,
    // so this cannot be forged as a different agent).
    let classifying_agent = caller()?;

    let entry = IssuerClassification {
        issuer_did: input.issuer_did.clone(),
        tier: input.tier,
        classified_at: now_iso_8601()?,
        rationale: input.rationale,
        classified_by: classifying_agent.clone(),
    };

    let ah = create_entry(&EntryTypes::IssuerClassification(entry))?;

    // Link from the tier anchor for tier queries (transparency listing;
    // safe unscoped since each entry is separately attributed).
    let tier_anchor = tier_path(input.tier).path_entry_hash()?;
    create_link(
        tier_anchor,
        ah.clone(),
        LinkTypes::TierAnchor,
        LinkTag::new(input.issuer_did.as_bytes().to_vec()),
    )?;

    // Link from the global issuer anchor (transparency listing across
    // every classifier -- see `list_classifications_for_issuer`).
    let issuer_anchor = issuer_path(&input.issuer_did).path_entry_hash()?;
    create_link(
        issuer_anchor,
        ah.clone(),
        LinkTypes::IssuerAnchor,
        LinkTag::new(input.tier.as_str().as_bytes().to_vec()),
    )?;

    // Link from the classifying agent's own anchor -- this is what lets
    // `lookup_tier` resolve "my own" classification without ever being
    // overridden by another agent's write.
    create_link(
        classifying_agent,
        ah.clone(),
        LinkTypes::AgentIssuerAnchor,
        LinkTag::new(input.issuer_did.as_bytes().to_vec()),
    )?;

    Ok(ClassifyIssuerOutput { action_hash: ah })
}

// ============================================================================
// Lookup the CALLING agent's own latest tier classification for an issuer
// ============================================================================

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct LookupTierInput {
    pub issuer_did: String,
}

/// Resolves the calling agent's OWN most recent classification of
/// `issuer_did` -- never another agent's. This is the scoping fix: prior
/// to 2026-07-10 this read from a single global anchor and picked
/// whichever classification (from ANY agent) had the latest timestamp,
/// so one forged/wrong write could silently override what every other
/// caller in the network saw. Use `list_classifications_for_issuer` to
/// see every classifier's opinion, attributed.
#[hdk_extern]
pub fn lookup_tier(input: LookupTierInput) -> ExternResult<Option<IssuerClassification>> {
    let caller_pk = caller()?;
    let links = get_links(
        LinkQuery::try_new(caller_pk.clone(), LinkTypes::AgentIssuerAnchor)?,
        GetStrategy::Local,
    )?;

    // Pick the most recent classification by timestamp -- safe here
    // because every candidate is first verified to actually have been
    // authored by `caller_pk`, so "latest wins" only ever resolves
    // within one agent's own history.
    let mut best: Option<IssuerClassification> = None;
    for link in links {
        let ah: ActionHash = link
            .target
            .try_into()
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("bad link target".to_string())))?;
        if let Some(record) = get(ah, GetOptions::default())? {
            if let Ok(Some(entry)) = record.entry().to_app_option::<IssuerClassification>() {
                // Defense in depth: a link's *base* hash can be filed by
                // any agent regardless of whose pubkey it is (Holochain
                // does not restrict who may create a link under a given
                // base), so don't trust the base alone. Re-check the
                // entry's own author-bound `classified_by` (enforced at
                // validation time) actually matches the agent we queried
                // under, and that it's the issuer we asked about.
                if entry.issuer_did != input.issuer_did || entry.classified_by != caller_pk {
                    continue;
                }
                match &best {
                    None => best = Some(entry),
                    Some(prev) if entry.classified_at > prev.classified_at => best = Some(entry),
                    _ => {}
                }
            }
        }
    }
    Ok(best)
}

// ============================================================================
// List every classifier's opinion for an issuer (transparency, attributed)
// ============================================================================

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ListClassificationsForIssuerInput {
    pub issuer_did: String,
}

/// Every classification any agent has ever made for `issuer_did`, each
/// still carrying its own author-bound `classified_by`. This is the
/// transparency counterpart to `lookup_tier`: instead of collapsing
/// conflicting opinions into one silently-overwritten "winner", it hands
/// the caller the full picture so they can decide for themselves which
/// classifier(s) to trust.
#[hdk_extern]
pub fn list_classifications_for_issuer(
    input: ListClassificationsForIssuerInput,
) -> ExternResult<Vec<IssuerClassification>> {
    let anchor = issuer_path(&input.issuer_did).path_entry_hash()?;
    let links = get_links(
        LinkQuery::try_new(anchor, LinkTypes::IssuerAnchor)?,
        GetStrategy::Local,
    )?;

    let mut out = Vec::with_capacity(links.len());
    for link in links {
        let ah: ActionHash = link
            .target
            .try_into()
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("bad link target".to_string())))?;
        if let Some(record) = get(ah, GetOptions::default())? {
            if let Ok(Some(entry)) = record.entry().to_app_option::<IssuerClassification>() {
                out.push(entry);
            }
        }
    }
    Ok(out)
}

// ============================================================================
// List issuers at a given tier
// ============================================================================

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ListByTierInput {
    pub tier: IssuerTier,
}

#[hdk_extern]
pub fn list_by_tier(input: ListByTierInput) -> ExternResult<Vec<IssuerClassification>> {
    let anchor = tier_path(input.tier).path_entry_hash()?;
    let links = get_links(
        LinkQuery::try_new(anchor, LinkTypes::TierAnchor)?,
        GetStrategy::Local,
    )?;

    let mut out = Vec::with_capacity(links.len());
    for link in links {
        let ah: ActionHash = link
            .target
            .try_into()
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("bad link target".to_string())))?;
        if let Some(record) = get(ah, GetOptions::default())? {
            if let Ok(Some(entry)) = record.entry().to_app_option::<IssuerClassification>() {
                out.push(entry);
            }
        }
    }
    Ok(out)
}

// ============================================================================
// Ping
// ============================================================================

#[hdk_extern]
pub fn ping(_: ()) -> ExternResult<String> {
    Ok("issuer_trust_tier:pong".to_string())
}

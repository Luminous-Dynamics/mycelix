#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// issuer-trust-tier integrity — three-tier classification for credential issuers.
// No tier ever influences Mycelix governance weight.

use hdi::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum IssuerTier {
    /// State-backed identity issuers: `did:web:state.gov`, `did:web:gov.uk`, etc.
    /// User-configurable list; never canonical, never hardcoded.
    Sovereign,
    /// Regulated KYC/AML providers: `did:web:jumio.com`, `did:web:onfido.com`.
    /// User-configurable; requires explicit opt-in.
    RegulatedIntermediary,
    /// Default tier. All issuers start here.
    Peer,
}

impl IssuerTier {
    pub fn as_str(self) -> &'static str {
        match self {
            IssuerTier::Sovereign => "sovereign",
            IssuerTier::RegulatedIntermediary => "regulated_intermediary",
            IssuerTier::Peer => "peer",
        }
    }
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct IssuerClassification {
    /// Issuer DID string.
    pub issuer_did: String,
    /// Assigned tier.
    pub tier: IssuerTier,
    /// ISO 8601 classification timestamp.
    pub classified_at: String,
    /// Optional rationale (freeform, for audit trail).
    pub rationale: Option<String>,
    /// The agent who authored this classification. **Authorization fix,
    /// 2026-07-10** (see `validate_classification` below): bound at the
    /// DHT validation layer to the real action author, so no agent can
    /// forge a classification attributed to a different classifier. A
    /// classification is a PERSONAL, per-agent view of an issuer -- see
    /// `IssuerTier`'s own doc comments ("user-configurable... never
    /// canonical, never hardcoded") -- never a single network-wide
    /// truth, so any live agent may classify (there is no admin/
    /// allowlist concept anywhere in this hApp; see the `legal_did` and
    /// `cross_did_zkp` zomes for the same per-agent-anchor convention).
    /// What was missing was scoping+attribution, not gatekeeping; see
    /// the coordinator's `lookup_tier`/`list_classifications_for_issuer`
    /// for how this field is used to fix the "latest write wins,
    /// globally" bug.
    pub classified_by: AgentPubKey,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    IssuerClassification(IssuerClassification),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Per-tier anchor → classifications at that tier (any classifier).
    /// The anchor is a Path keyed by tier string (e.g., "tier/sovereign").
    /// Safe despite being unscoped-by-classifier: each linked entry
    /// carries its own author-bound `classified_by`, so callers can see
    /// (and filter on) who made each claim rather than trusting the
    /// anchor's mere presence.
    TierAnchor,
    /// Global per-issuer anchor → every classification ever made for
    /// that issuer DID, by any classifier. The anchor is a Path keyed by
    /// issuer DID (e.g., "issuer/did:web:state.gov"). Intentionally
    /// unscoped -- this is the *transparency* index
    /// (`list_classifications_for_issuer`), not a source of truth: it
    /// hands back every classifier's attributed opinion rather than
    /// collapsing them into one.
    IssuerAnchor,
    /// Per-agent anchor (base = the classifying agent's own
    /// `AgentPubKey`) → that agent's own classification history. This is
    /// the fix for the "one forged/wrong write with a fresher timestamp
    /// silently overrides what the whole network sees" bug:
    /// `lookup_tier` resolves only from the CALLING agent's own links
    /// here, so "latest wins" only ever applies within one agent's own
    /// history, which only that agent can write to at the entry level
    /// (see `validate_classification`'s author check). Note the *link*
    /// itself can still be filed under any base by any agent (Holochain
    /// does not restrict who may create a link under a given base
    /// hash) -- the coordinator additionally re-checks
    /// `entry.classified_by == base` after fetching, so a forged link
    /// under someone else's base is simply skipped rather than trusted.
    AgentIssuerAnchor,
}

// ============================================================================
// Validation
// ============================================================================

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// **P0 author-binding pass, 2026-07-09**: no identity field exists on
/// `IssuerClassification` (case a) and no coordinator function calls
/// `update_entry` (confirmed via grep -- create-only). Closes the
/// wide-open RegisterUpdate/RegisterDelete bug that previously routed
/// both through the unconditional `_ => Valid` catch-all.
///
/// **Fixed, 2026-07-10** (previously disclosed-not-fixed here): added a
/// `classified_by: AgentPubKey` field to `IssuerClassification`, bound
/// to the real action author below. Combined with the coordinator's new
/// per-agent `AgentIssuerAnchor` link and rescoped `lookup_tier`, a
/// forged/wrong classification from one agent can no longer silently
/// override what every OTHER caller network-wide sees for an issuer --
/// each agent's own classification history is the only thing
/// `lookup_tier` resolves from, and `list_classifications_for_issuer`
/// exposes every classifier's claim with attribution instead of
/// collapsing to one "winner". See the coordinator module doc for the
/// full authorization model.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, action }) => match app_entry {
            EntryTypes::IssuerClassification(entry) => {
                validate_classification(&entry, EntryCreationAction::Create(action))
            }
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry { .. }) => Ok(ValidateCallbackResult::Invalid(
            "Issuer classifications are immutable".into(),
        )),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Issuer classifications are immutable".into(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_classification(
    entry: &IssuerClassification,
    action: EntryCreationAction,
) -> ExternResult<ValidateCallbackResult> {
    if entry.issuer_did.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "issuer_did empty".to_string(),
        ));
    }
    if entry.classified_at.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "classified_at empty".to_string(),
        ));
    }
    // Author binding: `classified_by` must match the real DHT action
    // author. Without this check, any agent could set `classified_by`
    // to an arbitrary `AgentPubKey` and forge a classification falsely
    // attributed to a different (perhaps more-trusted) classifier.
    if entry.classified_by != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "classified_by must match the action author".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

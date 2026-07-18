// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Ruleset Registry Integrity Zome
//!
//! Mycelix-side substrate for `WARDED_NODE_DESIGN_2026-07-11.md` Phase 5b:
//! a federated marketplace of shareable memetic-pathogen rulesets (mirrors
//! `symthaea-memetics::{Ruleset, RulesetEntry}`), each independently
//! published under its own agent's DID.
//!
//! **Policy this zome enforces structurally, not just by convention**
//! (decided 2026-07-11 — see the design doc's "5b policy" section):
//! there is deliberately **no "canonical" concept anywhere in this data
//! model**. Any agent meeting the (low) civic bar in the coordinator may
//! publish a `RulesetRecord`; a warded node's guardian decides which
//! publisher(s) to trust via the existing `web-of-trust` graph — this zome
//! never picks a winner. Do not add a "canonical"/"official" flag here; that
//! would silently re-introduce the single-curator model the owner rejected.
//!
//! **Immutable, append-only.** A `RulesetRecord` cannot be updated or
//! deleted once published — a new version is a new entry, linked to the
//! same publisher+name. This sidesteps an entire bug class found repeatedly
//! elsewhere in this monorepo's author-binding audit (permissively-`Valid`
//! `RegisterUpdate`/`RegisterDelete` letting a modified coordinator rewrite
//! or erase published content): there is no mutation path to leave open.

use hdi::prelude::*;

/// One pathogen signature within a [`RulesetRecord`] (mirrors
/// `symthaea_memetics::RulesetEntry`). The signature is opaque raw HDC
/// payload bytes to this zome — interpretation belongs entirely to the
/// consuming node. Not size-enforced to a specific HDC dimension here, so
/// this crate isn't coupled to a particular Symthaea release.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct RulesetEntryRecord {
    /// Raw HDC signature bytes.
    pub signature: Vec<u8>,
    /// Human-readable description of what this entry targets.
    pub description: String,
}

/// A named, versioned, publisher-attested collection of pathogen signatures.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RulesetRecord {
    /// The publishing agent. **Redundant with `action.author()` by design**
    /// — stored for convenient querying (building links/filters without an
    /// extra chain-activity lookup), but never trusted on its own. See
    /// `validate_create_ruleset` below for the actual DHT-level enforcement:
    /// a mismatch here is rejected outright, closing exactly the kind of
    /// self-reported-identity forgery gap this monorepo's P0 author-binding
    /// audit found repeatedly across other clusters.
    pub publisher: AgentPubKey,
    /// Short human-readable name, e.g. `"family-safety-baseline"`.
    pub name: String,
    /// Free-form version string (semver, a date, whatever the publisher uses).
    pub version: String,
    /// Free-form description of this ruleset's origin/purpose, for a
    /// guardian's own record — not verified by this zome; provenance trust
    /// is mediated by `publisher`'s DID via `web-of-trust`, not this field.
    pub source: String,
    pub entries: Vec<RulesetEntryRecord>,
}

/// Anchor entry for deterministic link bases (all-rulesets listing).
#[hdk_entry_helper]
#[derive(Clone)]
pub struct Anchor(pub String);

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(name = "RulesetRecord", visibility = "public")]
    RulesetRecord(RulesetRecord),
    #[entry_type(name = "Anchor", visibility = "public")]
    Anchor(Anchor),
}

#[hdk_link_types]
pub enum LinkTypes {
    PublisherToRulesets,
    AllRulesets,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::RulesetRecord(rs) => {
                    validate_create_ruleset(EntryCreationAction::Create(action), rs)
                }
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::RulesetRecord(_) => Ok(ValidateCallbackResult::Invalid(
                    "RulesetRecord is immutable — publish a new version as a new entry".into(),
                )),
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "RulesetRecord is immutable — no update path exists".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "RulesetRecord cannot be deleted (append-only registry)".into(),
        )),
        FlatOp::RegisterCreateLink { .. }
        | FlatOp::RegisterDeleteLink { .. }
        | FlatOp::StoreRecord(_)
        | FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
    }
}

/// Bind a `RulesetRecord` to its actual committing agent, and apply basic
/// size/shape sanity limits.
fn validate_create_ruleset(
    action: EntryCreationAction,
    rs: RulesetRecord,
) -> ExternResult<ValidateCallbackResult> {
    // The real enforcement: the coordinator sets `publisher` from
    // `agent_info()` (see `publish_ruleset`), but that trusts the
    // coordinator — this is the DHT-level check a modified/malicious
    // coordinator cannot bypass. Direct AgentPubKey comparison (no
    // `did:mycelix:{}` string formatting) so there's no room for a
    // formatting mismatch to itself become a bug.
    if action.author() != &rs.publisher {
        return Ok(ValidateCallbackResult::Invalid(
            "RulesetRecord.publisher must be the committing agent (forgery)".to_string(),
        ));
    }

    if rs.name.is_empty() || rs.name.len() > 128 {
        return Ok(ValidateCallbackResult::Invalid(
            "name must be 1..=128 chars".to_string(),
        ));
    }
    if rs.version.is_empty() || rs.version.len() > 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "version must be 1..=64 chars".to_string(),
        ));
    }
    if rs.source.len() > 512 {
        return Ok(ValidateCallbackResult::Invalid(
            "source must be <=512 chars".to_string(),
        ));
    }
    // Sanity bound, not a protocol limit: a single-entry DHT record keeps
    // this simple (see module docs); a chunked/paginated design for very
    // large rulesets is future work, not attempted here.
    if rs.entries.len() > 2_000 {
        return Ok(ValidateCallbackResult::Invalid(
            "a single RulesetRecord supports at most 2,000 entries — split into \
             multiple versions/rulesets for larger sets"
                .to_string(),
        ));
    }
    for entry in &rs.entries {
        if entry.description.len() > 512 {
            return Ok(ValidateCallbackResult::Invalid(
                "entry description must be <=512 chars".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ruleset_entry_record_serializes() {
        // Sanity check the type compiles and round-trips through serde,
        // independent of any Holochain runtime.
        let e = RulesetEntryRecord {
            signature: vec![0u8; 2048],
            description: "test".to_string(),
        };
        let json = serde_json::to_string(&e).unwrap();
        let back: RulesetEntryRecord = serde_json::from_str(&json).unwrap();
        assert_eq!(back.description, "test");
        assert_eq!(back.signature.len(), 2048);
    }
}

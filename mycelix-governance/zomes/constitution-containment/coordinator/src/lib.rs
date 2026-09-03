// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Temporary fail-closed constitutional coordinator.
//!
//! This coordinator intentionally preserves the deployed zome name `constitution`
//! while removing mutation authority from the runtime until constitutional
//! genesis and proposal-bound authorization are independently verified.
//!
//! Read endpoints remain available for migration/audit. Every mutation endpoint
//! denies with the same explicit containment reason. The legacy coordinator is
//! retained in source history but is not installed by the governance DNA while
//! this containment zome is active.

use constitution_integrity::*;
use hdk::prelude::*;

const CONTAINMENT_REASON: &str = "constitutional mutation is temporarily disabled: governance requires a DNA-bound genesis root and verified proposal/action authorization";

fn deny<T>() -> ExternResult<T> {
    Err(wasm_error!(WasmErrorInner::Guest(CONTAINMENT_REASON.into())))
}

fn anchor_hash(anchor_str: &str) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(anchor_str.to_string())))
}

#[hdk_extern]
pub fn init(_: ()) -> ExternResult<InitCallbackResult> {
    Ok(InitCallbackResult::Pass)
}

// -----------------------------------------------------------------------------
// Read-only compatibility surface
// -----------------------------------------------------------------------------

#[hdk_extern]
pub fn get_current_charter(_: ()) -> ExternResult<Option<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash("current_charter")?, LinkTypes::CurrentCharter)?,
        GetStrategy::default(),
    )?;

    let Some(link) = links.into_iter().max_by_key(|link| link.timestamp) else {
        return Ok(None);
    };
    let action_hash = ActionHash::try_from(link.target)
        .map_err(|_| wasm_error!(WasmErrorInner::Guest("invalid current-charter link target".into())))?;
    get(action_hash, GetOptions::default())
}

#[hdk_extern]
pub fn get_parameter(name: String) -> ExternResult<Option<Record>> {
    if name.is_empty() || name.len() > 256 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "parameter name must be 1-256 characters".into(),
        )));
    }

    let links = get_links(
        LinkQuery::try_new(
            anchor_hash(&format!("param:{name}"))?,
            LinkTypes::ParameterIndex,
        )?,
        GetStrategy::default(),
    )?;

    let Some(link) = links.into_iter().max_by_key(|link| link.timestamp) else {
        return Ok(None);
    };
    let action_hash = ActionHash::try_from(link.target)
        .map_err(|_| wasm_error!(WasmErrorInner::Guest("invalid parameter link target".into())))?;
    get(action_hash, GetOptions::default())
}

/// Audit/migration helper. Unlike the legacy source-chain-only list endpoint,
/// callers should prefer indexed `get_parameter` lookups for authoritative use.
#[hdk_extern]
pub fn list_parameters(_: ()) -> ExternResult<Vec<Record>> {
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::GovernanceParameter,
        )?))
        .include_entries(true);
    query(filter)
}

// -----------------------------------------------------------------------------
// Mutation compatibility surface: deliberately unavailable
// -----------------------------------------------------------------------------

#[derive(Serialize, Deserialize, Debug)]
pub struct ProposeAmendmentInput {
    pub amendment_type: AmendmentType,
    pub article: Option<String>,
    pub original_text: Option<String>,
    pub new_text: String,
    pub rationale: String,
    pub proposer_did: String,
    pub proposal_id: String,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SetParameterInput {
    pub name: String,
    pub value: String,
    pub value_type: ParameterType,
    pub description: String,
    pub min_value: Option<String>,
    pub max_value: Option<String>,
    pub proposal_id: Option<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct UpdateParameterInput {
    pub parameter: String,
    pub value: String,
    #[serde(default)]
    pub proposal_id: Option<String>,
}

#[hdk_extern]
pub fn create_charter(_: Charter) -> ExternResult<Record> {
    deny()
}

#[hdk_extern]
pub fn propose_amendment(_: ProposeAmendmentInput) -> ExternResult<Record> {
    deny()
}

#[hdk_extern]
pub fn ratify_amendment(_: String) -> ExternResult<Record> {
    deny()
}

#[hdk_extern]
pub fn set_parameter(_: SetParameterInput) -> ExternResult<Record> {
    deny()
}

#[hdk_extern]
pub fn update_parameter(_: UpdateParameterInput) -> ExternResult<Record> {
    deny()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn containment_reason_names_missing_authority_root() {
        assert!(CONTAINMENT_REASON.contains("DNA-bound genesis root"));
        assert!(CONTAINMENT_REASON.contains("proposal/action authorization"));
    }

    #[test]
    fn containment_is_not_score_or_phi_dependent() {
        let lower = CONTAINMENT_REASON.to_ascii_lowercase();
        assert!(!lower.contains("phi"));
        assert!(!lower.contains("reputation"));
        assert!(!lower.contains("score"));
    }
}

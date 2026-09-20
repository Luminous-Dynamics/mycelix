#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Data Sharing Preferences Coordinator — CRUD for per-user flow controls.
//!
//! The bridge dispatch layer queries these preferences before routing
//! cross-cluster calls. If a user has blocked a flow, the bridge returns
//! BRG-011 (UserBlocked) instead of executing the call.

use std::collections::BTreeMap;

use data_preferences_integrity::*;
use hdk::prelude::*;

getrandom::register_custom_getrandom!(my_custom_getrandom);

pub fn my_custom_getrandom(buf: &mut [u8]) -> Result<(), getrandom::Error> {
    let bytes = random_bytes(buf.len() as u32).map_err(|_| getrandom::Error::UNSUPPORTED)?;
    buf.copy_from_slice(bytes.as_ref());
    Ok(())
}

use personal_leptos_types::{
    ConditionalMutationResultView, ConditionalPreferenceMutationInputView,
    DataSharingPreferenceEvidenceView, DataSharingPreferenceView, MutationReceiptView,
    PreferenceChangeLogView,
};

/// Set a data sharing preference for a cluster pair.
///
/// Creates or updates the preference. Logs the change for audit.
#[hdk_extern]
pub fn set_preference(pref: DataSharingPreference) -> ExternResult<ActionHash> {
    let agent = agent_info()?.agent_initial_pubkey;

    // Check if a preference already exists for this pair
    let existing = get_preference_for_pair(&pref.source_cluster, &pref.target_cluster)?;

    let action_hash = create_entry(EntryTypes::DataSharingPreference(pref.clone()))?;

    // Link from agent
    create_link(
        agent.clone(),
        action_hash.clone(),
        LinkTypes::AgentToPreferences,
        LinkTag::new(format!("{}→{}", pref.source_cluster, pref.target_cluster)),
    )?;

    // Log the change
    let was_allowed = existing.map(|e| e.allowed).unwrap_or(true);
    if was_allowed != pref.allowed {
        let log = PreferenceChangeLog {
            source_cluster: pref.source_cluster,
            target_cluster: pref.target_cluster,
            was_allowed,
            now_allowed: pref.allowed,
            changed_at: pref.updated_at,
        };
        let log_hash = create_entry(EntryTypes::PreferenceChangeLog(log))?;
        create_link(agent, log_hash, LinkTypes::AgentToChangeLog, ())?;
    }

    Ok(action_hash)
}

#[hdk_extern]
pub fn set_preference_view(pref: DataSharingPreferenceView) -> ExternResult<MutationReceiptView> {
    let action_hash = set_preference(DataSharingPreference {
        source_cluster: pref.source_cluster,
        target_cluster: pref.target_cluster,
        allowed: pref.allowed,
        blocked_zomes: pref.blocked_zomes,
        reason: pref.reason,
        updated_at: sys_time()?,
    })?;
    Ok(MutationReceiptView {
        action_hash: action_hash.to_string(),
    })
}

fn expected_action_matches(expected: Option<&str>, current: Option<&str>) -> bool {
    expected == current
}

/// Conditionally replace one cluster-pair preference only when the exact
/// source-chain action observed by the caller is still current for that pair.
///
/// `expected_action_hash=None` means the caller observed authoritative absence
/// for the pair. A mismatch returns a typed conflict and creates no preference
/// or change-log action.
#[hdk_extern]
pub fn set_preference_view_if_current(
    input: ConditionalPreferenceMutationInputView,
) -> ExternResult<ConditionalMutationResultView> {
    let source = input.preference.source_cluster.clone();
    let target = input.preference.target_cluster.clone();
    let current = get_preference_record_for_pair(&source, &target)?;
    let current_action_hash = current
        .as_ref()
        .map(|(record, _)| record.action_address().to_string());

    if !expected_action_matches(
        input.expected_action_hash.as_deref(),
        current_action_hash.as_deref(),
    ) {
        return Ok(ConditionalMutationResultView::Conflict {
            current_action_hash,
        });
    }

    let receipt = set_preference_view(input.preference)?;
    Ok(ConditionalMutationResultView::Committed { receipt })
}

fn should_replace_action_seq(current: Option<u32>, candidate: u32) -> bool {
    current.map(|seq| candidate > seq).unwrap_or(true)
}

fn preference_from_record(record: &Record) -> ExternResult<Option<DataSharingPreference>> {
    record
        .entry()
        .to_app_option::<DataSharingPreference>()
        .map_err(|e| wasm_error!("Deserialize: {}", e))
}

/// Load exactly one current preference Record per cluster pair.
///
/// `get_links` is treated as an unordered candidate set. Only links authored by
/// this agent are admitted into self-state, and the target Record with the
/// highest source-chain action sequence wins for each pair.
fn get_my_preference_records() -> ExternResult<Vec<(Record, DataSharingPreference)>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::new(
            agent.clone(),
            LinkTypes::AgentToPreferences.try_into_filter()?,
        )
        .author(agent),
        GetStrategy::Network,
    )?;

    let mut latest: BTreeMap<(String, String), (u32, Record, DataSharingPreference)> =
        BTreeMap::new();

    for link in links {
        let hash: ActionHash = link
            .target
            .into_action_hash()
            .ok_or(wasm_error!("Invalid link target"))?;
        let Some(record) = get(hash, GetOptions::default())? else {
            continue;
        };
        let Some(pref) = preference_from_record(&record)? else {
            continue;
        };

        let key = (pref.source_cluster.clone(), pref.target_cluster.clone());
        let candidate_seq = record.action().action_seq();
        let current_seq = latest.get(&key).map(|(seq, _, _)| *seq);
        if should_replace_action_seq(current_seq, candidate_seq) {
            latest.insert(key, (candidate_seq, record, pref));
        }
    }

    Ok(latest
        .into_values()
        .map(|(_, record, pref)| (record, pref))
        .collect())
}

/// Get the current data sharing preference for each cluster pair.
#[hdk_extern]
pub fn get_my_preferences(_: ()) -> ExternResult<Vec<DataSharingPreference>> {
    Ok(get_my_preference_records()?
        .into_iter()
        .map(|(_, pref)| pref)
        .collect())
}

#[hdk_extern]
pub fn get_my_preferences_view(_: ()) -> ExternResult<Vec<DataSharingPreferenceView>> {
    Ok(get_my_preferences(())?
        .into_iter()
        .map(|pref| DataSharingPreferenceView {
            source_cluster: pref.source_cluster,
            target_cluster: pref.target_cluster,
            allowed: pref.allowed,
            blocked_zomes: pref.blocked_zomes,
            reason: pref.reason,
            updated_at: pref.updated_at.as_micros(),
        })
        .collect())
}

/// Get current data-sharing preference values together with the exact source-chain
/// action from which each returned row was decoded.
///
/// Exactly one current Record per cluster pair is returned. The action identity
/// therefore describes the same deterministic current row that the legacy view
/// endpoint exposes.
#[hdk_extern]
pub fn get_my_preferences_evidence_view(
    _: (),
) -> ExternResult<Vec<DataSharingPreferenceEvidenceView>> {
    Ok(get_my_preference_records()?
        .into_iter()
        .map(|(record, pref)| DataSharingPreferenceEvidenceView {
            action_hash: record.action_address().to_string(),
            preference: DataSharingPreferenceView {
                source_cluster: pref.source_cluster,
                target_cluster: pref.target_cluster,
                allowed: pref.allowed,
                blocked_zomes: pref.blocked_zomes,
                reason: pref.reason,
                updated_at: pref.updated_at.as_micros(),
            },
        })
        .collect())
}

/// Check if a specific flow is allowed for the current agent.
///
/// Returns true if no preference exists (default: allow all).
/// This is the function the bridge dispatch layer calls.
#[hdk_extern]
pub fn is_flow_allowed(input: FlowCheckInput) -> ExternResult<bool> {
    let pref = get_preference_for_pair(&input.source_cluster, &input.target_cluster)?;
    match pref {
        Some(p) => {
            if !p.allowed {
                return Ok(false);
            }
            // Check per-zome blocks
            if !p.blocked_zomes.is_empty() && p.blocked_zomes.contains(&input.zome_name) {
                return Ok(false);
            }
            Ok(true)
        }
        None => Ok(true), // No preference = allow
    }
}

/// Get the preference change log for the current agent.
#[hdk_extern]
pub fn get_change_log(_: ()) -> ExternResult<Vec<PreferenceChangeLog>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::new(agent.clone(), LinkTypes::AgentToChangeLog.try_into_filter()?).author(agent),
        GetStrategy::Network,
    )?;

    let mut logs = Vec::new();
    for link in links {
        let hash: ActionHash = link
            .target
            .into_action_hash()
            .ok_or(wasm_error!("Invalid link target"))?;
        if let Some(record) = get(hash, GetOptions::default())? {
            if let Some(log) = record
                .entry()
                .to_app_option::<PreferenceChangeLog>()
                .map_err(|e| wasm_error!("Deserialize: {}", e))?
            {
                logs.push(log);
            }
        }
    }

    Ok(logs)
}

#[hdk_extern]
pub fn get_change_log_view(_: ()) -> ExternResult<Vec<PreferenceChangeLogView>> {
    Ok(get_change_log(())?
        .into_iter()
        .map(|log| PreferenceChangeLogView {
            source_cluster: log.source_cluster,
            target_cluster: log.target_cluster,
            was_allowed: log.was_allowed,
            now_allowed: log.now_allowed,
            changed_at: log.changed_at.as_micros(),
        })
        .collect())
}

#[derive(Serialize, Deserialize, Debug)]
pub struct FlowCheckInput {
    pub source_cluster: String,
    pub target_cluster: String,
    pub zome_name: String,
}

/// Internal: find the current self-authored preference Record and decoded value
/// for one cluster pair.
fn get_preference_record_for_pair(
    source: &str,
    target: &str,
) -> ExternResult<Option<(Record, DataSharingPreference)>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let tag = LinkTag::new(format!("{source}→{target}"));
    let links = get_links(
        LinkQuery::new(
            agent.clone(),
            LinkTypes::AgentToPreferences.try_into_filter()?,
        )
        .tag_prefix(tag)
        .author(agent),
        GetStrategy::Network,
    )?;

    let mut latest: Option<(u32, Record, DataSharingPreference)> = None;
    for link in links {
        let hash: ActionHash = link
            .target
            .into_action_hash()
            .ok_or(wasm_error!("Invalid link target"))?;
        let Some(record) = get(hash, GetOptions::default())? else {
            continue;
        };
        let Some(pref) = preference_from_record(&record)? else {
            continue;
        };

        let candidate_seq = record.action().action_seq();
        let current_seq = latest.as_ref().map(|(seq, _, _)| *seq);
        if should_replace_action_seq(current_seq, candidate_seq) {
            latest = Some((candidate_seq, record, pref));
        }
    }

    Ok(latest.map(|(_, record, pref)| (record, pref)))
}

/// Internal: find the current self-authored preference value for a cluster pair.
fn get_preference_for_pair(
    source: &str,
    target: &str,
) -> ExternResult<Option<DataSharingPreference>> {
    Ok(get_preference_record_for_pair(source, target)?.map(|(_, pref)| pref))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_chain_sequence_is_the_current_preference_order() {
        assert!(should_replace_action_seq(None, 4));
        assert!(should_replace_action_seq(Some(4), 5));
        assert!(!should_replace_action_seq(Some(5), 5));
        assert!(!should_replace_action_seq(Some(6), 5));
    }

    #[test]
    fn conditional_preference_precondition_requires_exact_action_identity() {
        assert!(expected_action_matches(None, None));
        assert!(expected_action_matches(Some("A"), Some("A")));
        assert!(!expected_action_matches(None, Some("A")));
        assert!(!expected_action_matches(Some("A"), None));
        assert!(!expected_action_matches(Some("A"), Some("B")));
    }
}

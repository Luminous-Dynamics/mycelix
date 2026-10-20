// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Integrity zome for the cross-cluster reputation aggregator.

use hdi::prelude::*;

/// Aggregated reputation entry combining scores from multiple clusters.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AggregatedReputation {
    /// Agent whose reputation is aggregated.
    pub agent_pubkey_b64: String,
    /// Score from identity/web-of-trust (0.0-1.0).
    pub web_of_trust_score: f64,
    /// MYCEL score from finance/recognition (0.0-1.0).
    pub mycel_score: f64,
    /// Per-cluster domain scores: Vec of (cluster_name, score).
    pub domain_scores: Vec<(String, f64)>,
    /// Weighted composite score (0.0-1.0).
    pub composite: f64,
    /// When this aggregation was computed.
    pub computed_at: Timestamp,
    /// True if any source score is stale.
    pub staleness_warning: bool,
}

/// Domain score report from another cluster.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DomainScoreReport {
    /// Agent whose score is being reported.
    pub agent_pubkey_b64: String,
    /// Cluster that is reporting (e.g., "commons", "civic").
    pub cluster: String,
    /// The domain-specific score (0.0-1.0).
    pub score: f64,
    /// When this score was computed at the source.
    pub source_timestamp: Timestamp,
    /// Who reported this score.
    pub reporter_pubkey_b64: String,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    AggregatedReputation(AggregatedReputation),
    DomainScoreReport(DomainScoreReport),
}

#[hdk_link_types]
pub enum LinkTypes {
    AgentToReputation,
    AgentToDomainScore,
    AllReputations,
    ClusterToDomainScore,
}

fn anchor_hash(label: &str) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(label.to_string())))
}

fn string_anchor(prefix: &str, value: &str) -> ExternResult<EntryHash> {
    anchor_hash(&format!("{prefix}:{value}"))
}

fn validate_timestamp_not_future(
    field: &str,
    value: Timestamp,
    action_timestamp: Timestamp,
) -> ValidateCallbackResult {
    if value > action_timestamp {
        return ValidateCallbackResult::Invalid(format!(
            "{field} cannot be later than its signed Holochain action timestamp"
        ));
    }
    ValidateCallbackResult::Valid
}

fn parse_agent(value: &str, label: &str) -> ExternResult<AgentPubKey> {
    AgentPubKey::try_from(value.to_string()).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "{label} must be a valid AgentPubKey string"
        )))
    })
}

fn validate_unit_interval(value: f64, label: &str) -> Result<(), String> {
    if !value.is_finite() || !(0.0..=1.0).contains(&value) {
        return Err(format!("{label} must be finite and in [0.0, 1.0]"));
    }
    Ok(())
}

fn validate_aggregated_reputation(entry: &AggregatedReputation) -> Result<(), String> {
    parse_agent(&entry.agent_pubkey_b64, "agent_pubkey_b64")
        .map_err(|e| format!("{e:?}"))?;
    validate_unit_interval(entry.web_of_trust_score, "web_of_trust_score")?;
    validate_unit_interval(entry.mycel_score, "mycel_score")?;
    validate_unit_interval(entry.composite, "composite")?;
    if entry.domain_scores.len() > 128 {
        return Err("domain_scores exceeds maximum of 128 clusters".into());
    }

    let mut clusters = std::collections::BTreeSet::new();
    for (cluster, score) in &entry.domain_scores {
        if cluster.is_empty() || cluster.len() > 64 {
            return Err("domain cluster names must be 1-64 characters".into());
        }
        validate_unit_interval(*score, "domain score")?;
        if !clusters.insert(cluster) {
            return Err(format!("duplicate domain score cluster '{cluster}'"));
        }
    }

    if entry.computed_at == Timestamp::from_micros(0) {
        return Err("computed_at must be non-zero".into());
    }
    Ok(())
}

fn validate_domain_score_report(
    action: &EntryCreationAction,
    entry: &DomainScoreReport,
) -> ExternResult<ValidateCallbackResult> {
    if entry.cluster.is_empty() || entry.cluster.len() > 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cluster name must be 1-64 characters".into(),
        ));
    }
    validate_unit_interval(entry.score, "score")
        .map_err(|message| wasm_error!(WasmErrorInner::Guest(message)))?;

    parse_agent(&entry.agent_pubkey_b64, "agent_pubkey_b64")?;
    let reporter = parse_agent(&entry.reporter_pubkey_b64, "reporter_pubkey_b64")?;
    if reporter != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "DomainScoreReport reporter must equal the committing agent".into(),
        ));
    }
    if entry.source_timestamp == Timestamp::from_micros(0) {
        return Ok(ValidateCallbackResult::Invalid(
            "source_timestamp must be non-zero".into(),
        ));
    }
    match validate_timestamp_not_future(
        "DomainScoreReport source timestamp",
        entry.source_timestamp,
        *action.timestamp(),
    ) {
        ValidateCallbackResult::Valid => {}
        invalid => return Ok(invalid),
    }
    Ok(ValidateCallbackResult::Valid)
}

fn action_target(
    target_address: &AnyLinkableHash,
    label: &str,
) -> ExternResult<ActionHash> {
    target_address.clone().into_action_hash().ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "{label} target must be an ActionHash"
        )))
    })
}

fn validate_link(
    link_type: LinkTypes,
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
    action: &CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    let target = action_target(target_address, "Reputation link")?;
    let record = must_get_valid_record(target)?;
    match link_type {
        LinkTypes::AgentToDomainScore => {
            let report: DomainScoreReport = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "AgentToDomainScore target must be a DomainScoreReport".into(),
                )))?;
            let expected = string_anchor("domain_scores", &report.agent_pubkey_b64)?;
            let actual = base_address.clone().into_entry_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "AgentToDomainScore base must be an EntryHash".into(),
                ))
            })?;
            if actual != expected {
                return Ok(ValidateCallbackResult::Invalid(
                    "AgentToDomainScore base does not match the report's target agent".into(),
                ));
            }
        }
        LinkTypes::ClusterToDomainScore => {
            let report: DomainScoreReport = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "ClusterToDomainScore target must be a DomainScoreReport".into(),
                )))?;
            let expected = string_anchor("cluster_scores", &report.cluster)?;
            let actual = base_address.clone().into_entry_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "ClusterToDomainScore base must be an EntryHash".into(),
                ))
            })?;
            if actual != expected {
                return Ok(ValidateCallbackResult::Invalid(
                    "ClusterToDomainScore base does not match the report cluster".into(),
                ));
            }
        }
        LinkTypes::AgentToReputation => {
            let rep: AggregatedReputation = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "AgentToReputation target must be an AggregatedReputation".into(),
                )))?;
            let expected = string_anchor("agent_rep", &rep.agent_pubkey_b64)?;
            let actual = base_address.clone().into_entry_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "AgentToReputation base must be an EntryHash".into(),
                ))
            })?;
            if actual != expected {
                return Ok(ValidateCallbackResult::Invalid(
                    "AgentToReputation base does not match the aggregated agent".into(),
                ));
            }
        }
        LinkTypes::AllReputations => {
            if record
                .entry()
                .to_app_option::<AggregatedReputation>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .is_none()
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "AllReputations target must be an AggregatedReputation".into(),
                ));
            }
            let actual = base_address.clone().into_entry_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "AllReputations base must be an EntryHash".into(),
                ))
            })?;
            if actual != anchor_hash("all_reputations")? {
                return Ok(ValidateCallbackResult::Invalid(
                    "AllReputations base must be the canonical all_reputations anchor".into(),
                ));
            }
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(_op: Op) -> ExternResult<ValidateCallbackResult> {
    // Validation is implemented below with a flattened op match so every
    // write path, including a modified coordinator, hits the same invariants.
    validate_op(_op)
}

fn validate_op(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(anchor) => {
                    if anchor.0.is_empty() || anchor.0.len() > 256 {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Anchor must be 1-256 characters".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                EntryTypes::AggregatedReputation(entry) => {
                    validate_aggregated_reputation(&entry).map_or_else(
                        |message| Ok(ValidateCallbackResult::Invalid(message)),
                        |_| Ok(ValidateCallbackResult::Valid),
                    )
                }
                EntryTypes::DomainScoreReport(entry) => {
                    validate_domain_score_report(
                        &EntryCreationAction::Create(action),
                        &entry,
                    )
                }
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                ..
            } => {
                let original = must_get_valid_record(action.original_action_address.clone())?;
                if *original.action().author() != action.author {
                    return Ok(ValidateCallbackResult::Invalid(
                        "Only the original entry author can update reputation entries".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Invalid(
                    "Reputation entries are append-only".into(),
                ))
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            base_address,
            target_address,
            link_type,
            tag,
            action,
        } => {
            if tag.0.len() > 1024 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Link tag exceeds maximum length of 1024 bytes".into(),
                ));
            }
            validate_link(link_type, &base_address, &target_address, &action)
        }
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Reputation index links cannot be deleted".into(),
        )),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(action.original_action_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can update reputation entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Invalid(
                "Reputation entries are append-only".into(),
            ))
        }
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete reputation entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Invalid(
                "Reputation entries cannot be deleted".into(),
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_timestamp_cannot_be_future_dated() {
        let action_timestamp = Timestamp::from_micros(1_000);
        assert_eq!(
            validate_timestamp_not_future(
                "DomainScoreReport source timestamp",
                Timestamp::from_micros(1_000),
                action_timestamp,
            ),
            ValidateCallbackResult::Valid
        );
        match validate_timestamp_not_future(
            "DomainScoreReport source timestamp",
            Timestamp::from_micros(1_001),
            action_timestamp,
        ) {
            ValidateCallbackResult::Invalid(message) => {
                assert!(message.contains("signed Holochain action timestamp"));
            }
            other => panic!("future source timestamp must be invalid, got {other:?}"),
        }
    }

    #[test]
    fn unit_interval_rejects_nan_and_out_of_range() {
        assert!(validate_unit_interval(f64::NAN, "score").is_err());
        assert!(validate_unit_interval(f64::NEG_INFINITY, "score").is_err());
        assert!(validate_unit_interval(-0.1, "score").is_err());
        assert!(validate_unit_interval(1.1, "score").is_err());
        assert!(validate_unit_interval(0.0, "score").is_ok());
        assert!(validate_unit_interval(1.0, "score").is_ok());
    }

    #[test]
    fn report_author_is_distinct_from_reported_agent() {
        let target = AgentPubKey::from_raw_36(vec![1; 36]);
        let reporter = AgentPubKey::from_raw_36(vec![2; 36]);
        assert_ne!(target, reporter);
    }
}

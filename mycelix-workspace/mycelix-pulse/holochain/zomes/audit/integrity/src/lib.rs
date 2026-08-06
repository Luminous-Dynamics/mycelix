// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Audit Integrity Zome
//!
//! Tamper-evident audit log entries.

use hdi::prelude::*;

/// Audit log entry
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AuditEntry {
    pub id: String,
    pub timestamp: u64,
    pub category: AuditCategory,
    pub action: String,
    pub severity: AuditSeverity,
    pub actor: AuditActor,
    pub target: Option<AuditTarget>,
    pub details: AuditDetails,
    pub metadata: AuditMetadata,
    pub signature: Option<Vec<u8>>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum AuditCategory {
    Authentication,
    Email,
    Contact,
    Trust,
    Sync,
    Settings,
    Security,
    Admin,
    System,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum AuditSeverity {
    Debug,
    Info,
    Warning,
    Error,
    Critical,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct AuditActor {
    pub agent_pub_key: Option<AgentPubKey>,
    pub email: Option<String>,
    pub ip_address: Option<String>,
    pub user_agent: Option<String>,
    pub session_id: Option<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct AuditTarget {
    pub target_type: String,
    pub id: Option<String>,
    pub hash: Option<ActionHash>,
    pub description: Option<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct AuditDetails {
    pub message: String,
    pub before: Option<String>, // JSON string
    pub after: Option<String>,  // JSON string
    pub error: Option<String>,
    pub stack_trace: Option<String>,
    pub custom: Option<String>, // JSON string
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct AuditMetadata {
    pub source: AuditSource,
    pub version: String,
    pub correlation_id: Option<String>,
    pub parent_id: Option<String>,
    pub tags: Option<Vec<String>>,
    pub retention: Option<u32>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum AuditSource {
    Client,
    Server,
    System,
    External,
}

/// Audit summary for reporting
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AuditSummary {
    pub period_start: u64,
    pub period_end: u64,
    pub total_entries: u32,
    pub by_category: Vec<(String, u32)>,
    pub by_severity: Vec<(String, u32)>,
    pub unique_actors: u32,
    pub generated_at: u64,
    pub hash: String,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    AuditEntry(AuditEntry),
    AuditSummary(AuditSummary),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllAuditEntries,
    EntriesByCategory,
    EntriesBySeverity,
    EntriesByActor,
    EntriesByCorrelation,
    AuditSummaries,
}

/// Validate audit entry - entries are append-only
fn validate_create_audit_entry(
    action: Create,
    entry: AuditEntry,
) -> ExternResult<ValidateCallbackResult> {
    // Validate required fields
    if entry.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Audit entry ID cannot be empty".to_string(),
        ));
    }

    if entry.details.message.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Audit message cannot be empty".to_string(),
        ));
    }

    if entry.action.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Audit action cannot be empty".to_string(),
        ));
    }

    // create_audit_entry/create_entries take the whole AuditEntry as raw caller
    // input -- any agent could otherwise forge an entry claiming another agent
    // performed some action. actor.agent_pub_key is optional (audit entries about
    // unauthenticated events -- e.g. failed logins -- legitimately have no known
    // agent, tracked only by ip_address/user_agent instead), but when present it
    // must be the entry's own committer: no cross-zome caller or admin/moderator
    // role exists anywhere in this hApp that would need to log an entry on another
    // agent's behalf.
    if let Some(agent) = &entry.actor.agent_pub_key {
        if agent != &action.author {
            return Ok(ValidateCallbackResult::Invalid(
                "AuditEntry actor.agent_pub_key must match action author".to_string(),
            ));
        }
    }

    // Note: Time-based validation moved to coordinator zome since sys_time
    // is not available in integrity zomes

    Ok(ValidateCallbackResult::Valid)
}

/// Main validation dispatcher
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::AuditEntry(entry) => validate_create_audit_entry(action, entry),
                EntryTypes::AuditSummary(_) => Ok(ValidateCallbackResult::Valid),
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::AuditEntry(_) => Ok(ValidateCallbackResult::Invalid(
                    "Audit entries cannot be updated".to_string(),
                )),
                EntryTypes::AuditSummary(_) => Ok(ValidateCallbackResult::Valid),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

/// Proves `validate_create_audit_entry`'s P0 author-binding fix: an AuditEntry claiming a
/// different agent as `actor.agent_pub_key` than the entry's real committer is rejected
/// (previously any agent could forge an entry blaming another agent). Host-independent --
/// no HDI mocking needed, this check only compares against `action.author`.
#[cfg(test)]
mod tests {
    use super::*;

    fn test_entry(actor_agent: Option<AgentPubKey>) -> AuditEntry {
        AuditEntry {
            id: "evt-1".to_string(),
            timestamp: 0,
            category: AuditCategory::Security,
            action: "login_failed".to_string(),
            severity: AuditSeverity::Warning,
            actor: AuditActor {
                agent_pub_key: actor_agent,
                email: None,
                ip_address: Some("127.0.0.1".to_string()),
                user_agent: None,
                session_id: None,
            },
            target: None,
            details: AuditDetails {
                message: "failed login attempt".to_string(),
                before: None,
                after: None,
                error: None,
                stack_trace: None,
                custom: None,
            },
            metadata: AuditMetadata {
                source: AuditSource::Server,
                version: "1".to_string(),
                correlation_id: None,
                parent_id: None,
                tags: None,
                retention: None,
            },
            signature: None,
        }
    }

    fn test_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![1; 36]),
            weight: Default::default(),
        }
    }

    #[test]
    fn forged_actor_agent_is_rejected() {
        let committer = AgentPubKey::from_raw_36(vec![1; 36]);
        let framed = AgentPubKey::from_raw_36(vec![2; 36]);
        let entry = test_entry(Some(framed));
        let result = validate_create_audit_entry(test_action(committer), entry).unwrap();
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(_)),
            "an AuditEntry naming a different agent as actor.agent_pub_key must be rejected \
             -- previously any agent could forge an entry blaming another agent"
        );
    }

    #[test]
    fn entry_with_no_named_agent_is_accepted() {
        let committer = AgentPubKey::from_raw_36(vec![1; 36]);
        let entry = test_entry(None);
        let result = validate_create_audit_entry(test_action(committer), entry).unwrap();
        assert!(
            matches!(result, ValidateCallbackResult::Valid),
            "unauthenticated events (e.g. failed logins with no known agent) must remain valid"
        );
    }

    #[test]
    fn entry_naming_its_real_committer_is_accepted() {
        let committer = AgentPubKey::from_raw_36(vec![1; 36]);
        let entry = test_entry(Some(committer.clone()));
        let result = validate_create_audit_entry(test_action(committer), entry).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }
}

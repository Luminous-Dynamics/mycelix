// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Security Integrity Zome - Entry types for security logging and auditing
use hdi::prelude::*;

/// Security event types
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum SecurityEventType {
    /// XSS attempt detected
    XssAttempt,
    /// SQL injection attempt
    InjectionAttempt,
    /// Invalid IPFS CID submitted
    InvalidCid,
    /// Rate limit exceeded
    RateLimitExceeded,
    /// Profanity detected
    ProfanityDetected,
    /// Suspicious activity pattern
    SuspiciousActivity,
    /// Authorization failure
    AuthorizationFailure,
    /// Input validation failure
    ValidationFailure,
}

/// Security severity levels
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum SecuritySeverity {
    Low = 1,
    Medium = 2,
    High = 3,
    Critical = 4,
}

/// Security log entry for auditing and threat detection
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SecurityLog {
    /// Event type
    pub event_type: SecurityEventType,

    /// Severity level
    pub severity: SecuritySeverity,

    /// Agent who triggered the event
    pub agent: AgentPubKey,

    /// Zome/function where event occurred
    pub zome_name: String,
    pub function_name: String,

    /// Event description
    pub description: String,

    /// Input that triggered the event (sanitized)
    pub trigger_input: Option<String>,

    /// Timestamp
    pub timestamp: Timestamp,

    /// Additional metadata
    pub metadata: String,
}

/// Link types for security logs
#[hdk_link_types]
pub enum LinkTypes {
    /// Links from agent to their security events
    AgentToSecurityLogs,

    /// Links from event type to logs
    EventTypeToLogs,

    /// All security logs anchor
    AllSecurityLogs,
}

/// Entry types for this integrity zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    SecurityLog(SecurityLog),
}

/// Validation function for SecurityLog entries
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::SecurityLog(log) => validate_create_security_log(&log, &action),
            },
            // SecurityLog is confirmed create-only (no coordinator function ever calls
            // update_entry on it) -- reject outright rather than leave an unbound dead-code
            // path (P0 wide-open RegisterUpdate gap, confirmed dozens of times elsewhere in
            // this pass).
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "SecurityLog entries cannot be updated".to_string(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "SecurityLog entries cannot be updated".to_string(),
        )),
        FlatOp::RegisterCreateLink { link_type, .. } => {
            // All link types are valid for security logging
            match link_type {
                LinkTypes::AgentToSecurityLogs => Ok(ValidateCallbackResult::Valid),
                LinkTypes::EventTypeToLogs => Ok(ValidateCallbackResult::Valid),
                LinkTypes::AllSecurityLogs => Ok(ValidateCallbackResult::Valid),
            }
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Bind the log to its committer -- log_security_event already derives agent from
/// agent_info() coordinator-side with zero user input (P0 author-binding gap).
fn validate_create_security_log(
    log: &SecurityLog,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if let ValidateCallbackResult::Invalid(reason) = validate_security_log(log)? {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }
    if log.agent != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "SecurityLog must be recorded by the committing agent (agent forgery)".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate security log data
fn validate_security_log(log: &SecurityLog) -> ExternResult<ValidateCallbackResult> {
    // Description required
    if log.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Security log description cannot be empty".into(),
        ));
    }

    // Description length check
    if log.description.len() > 1000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Security log description too long (max 1000 characters)".into(),
        ));
    }

    // Zome and function names required
    if log.zome_name.is_empty() || log.function_name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Zome and function names required".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

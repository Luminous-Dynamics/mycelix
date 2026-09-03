// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Integrity rules for governance execution-lifecycle candidates.
//!
//! Candidate existence is not authority. The coordinator's verified projection
//! must independently re-resolve the execution domain and verify every candidate
//! event before feeding it to the pure lifecycle projector.

use hdi::prelude::*;
use mycelix_governance_execution_lifecycle::{ExecutionDomain, LifecycleEvent, LifecycleEventKind};

const MAX_ID_BYTES: usize = 512;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct LifecycleEventCandidate {
    pub id: String,
    pub proposal_id: String,
    pub domain: ExecutionDomain,
    pub event: LifecycleEvent,
    pub published_by: String,
    pub published_at: Timestamp,
}

impl LifecycleEventCandidate {
    pub fn validate_structure(&self) -> Result<(), String> {
        validate_text(&self.id, "lifecycle candidate id")?;
        validate_text(&self.proposal_id, "proposal id")?;
        if !self.proposal_id.starts_with("MIP-") {
            return Err("Execution lifecycle proposal id must start with MIP-".into());
        }
        self.domain
            .validate()
            .map_err(|e| format!("Invalid execution domain: {e}"))?;
        if self.domain.proposal_id != self.proposal_id {
            return Err("Lifecycle candidate domain targets a different proposal".into());
        }
        self.event
            .validate()
            .map_err(|e| format!("Invalid lifecycle event: {e}"))?;
        let domain_digest = self
            .domain
            .digest()
            .map_err(|e| format!("Cannot digest execution domain: {e}"))?;
        if self.event.execution_domain_digest != domain_digest {
            return Err("Lifecycle event targets a different execution domain".into());
        }
        require_mycelix_did(&self.published_by, "lifecycle publisher")?;
        if self.event.actor_id != self.published_by {
            return Err("Lifecycle event actor must equal its committing publisher".into());
        }
        if requires_executor_actor(&self.event.kind)
            && self.event.actor_id != self.domain.executor_principal
        {
            return Err("Execution attempt event actor differs from bound executor principal".into());
        }
        let published_ms = timestamp_ms(self.published_at, "lifecycle published_at")?;
        if self.event.occurred_at_ms != published_ms {
            return Err("Lifecycle event time must equal candidate publication time".into());
        }
        Ok(())
    }
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    LifecycleEventCandidate(LifecycleEventCandidate),
}

#[hdk_link_types]
pub enum LinkTypes {
    ProposalToLifecycleEvent,
    DomainToLifecycleEvent,
}

fn validate_create_candidate(
    action: Create,
    entry: LifecycleEventCandidate,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = entry.validate_structure() {
        return Ok(ValidateCallbackResult::Invalid(error));
    }
    let expected_author = format!("did:mycelix:{}", action.author);
    if entry.published_by != expected_author {
        return Ok(ValidateCallbackResult::Invalid(
            "Lifecycle candidate published_by must equal the committing agent".into(),
        ));
    }
    if timestamp_ms(action.timestamp, "lifecycle action timestamp")?
        != timestamp_ms(entry.published_at, "lifecycle published_at")?
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Lifecycle publication time must equal the create-action timestamp".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::LifecycleEventCandidate(entry) => {
                    validate_create_candidate(action, entry)
                }
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::LifecycleEventCandidate(_) => Ok(ValidateCallbackResult::Invalid(
                    "Execution lifecycle candidates are immutable".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Execution lifecycle links are append-only".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Execution lifecycle candidates are append-only".into(),
        )),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn requires_executor_actor(kind: &LifecycleEventKind) -> bool {
    matches!(
        kind,
        LifecycleEventKind::Claimed { .. }
            | LifecycleEventKind::Completed { .. }
            | LifecycleEventKind::Failed { .. }
            | LifecycleEventKind::Uncertain { .. }
    )
}

fn validate_text(value: &str, field: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{field} must not be empty"));
    }
    if value.len() > MAX_ID_BYTES {
        return Err(format!("{field} exceeds {MAX_ID_BYTES} bytes"));
    }
    Ok(())
}

fn require_mycelix_did(value: &str, field: &str) -> Result<(), String> {
    validate_text(value, field)?;
    if !value.starts_with("did:mycelix:") {
        return Err(format!("{field} must be a did:mycelix identifier"));
    }
    Ok(())
}

fn timestamp_ms(timestamp: Timestamp, field: &str) -> Result<u64, String> {
    let micros = timestamp.as_micros();
    if micros <= 0 {
        return Err(format!("{field} must be positive"));
    }
    Ok(micros as u64 / 1_000)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_governance_execution_lifecycle::{
        ProfiledDigest, PROTOCOL_VERSION as LIFECYCLE_PROTOCOL_VERSION,
    };
    use mycelix_institutional_core::Digest32;

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn ts(ms: i64) -> Timestamp {
        Timestamp::from_micros(ms * 1_000)
    }

    fn domain() -> ExecutionDomain {
        ExecutionDomain {
            protocol_version: LIFECYCLE_PROTOCOL_VERSION.into(),
            proposal_id: "MIP-42".into(),
            timelock_ref: "timelock:MIP-42".into(),
            proposal_authority_ref: "authority:1".into(),
            constitutional_epoch: ProfiledDigest {
                digest: digest(1),
                profile: "constitution-v1".into(),
            },
            actions: ProfiledDigest {
                digest: digest(2),
                profile: "actions-v1".into(),
            },
            binding_tally_ref: "tally:1".into(),
            threshold_authorization_ref: "threshold:1".into(),
            executor_principal: "did:mycelix:executor".into(),
            executor_authority_ref: "executor-grant:1".into(),
            effect_safety_policy: ProfiledDigest {
                digest: digest(3),
                profile: "effect-safety-v1".into(),
            },
        }
    }

    #[test]
    fn executor_claim_actor_must_match_bound_principal() {
        let domain = domain();
        let event = LifecycleEvent {
            protocol_version: LIFECYCLE_PROTOCOL_VERSION.into(),
            execution_domain_digest: domain.digest().unwrap(),
            parent_event_id: Some(digest(9)),
            actor_id: "did:mycelix:not-executor".into(),
            occurred_at_ms: 10,
            kind: LifecycleEventKind::Claimed {
                claim_nonce: digest(4),
            },
        };
        let candidate = LifecycleEventCandidate {
            id: "candidate:1".into(),
            proposal_id: "MIP-42".into(),
            domain,
            event,
            published_by: "did:mycelix:not-executor".into(),
            published_at: ts(10),
        };
        assert!(candidate.validate_structure().is_err());
    }

    #[test]
    fn event_time_must_equal_publication_time() {
        let domain = domain();
        let event = LifecycleEvent {
            protocol_version: LIFECYCLE_PROTOCOL_VERSION.into(),
            execution_domain_digest: domain.digest().unwrap(),
            parent_event_id: None,
            actor_id: "did:mycelix:authority".into(),
            occurred_at_ms: 9,
            kind: LifecycleEventKind::Registered,
        };
        let candidate = LifecycleEventCandidate {
            id: "candidate:1".into(),
            proposal_id: "MIP-42".into(),
            domain,
            event,
            published_by: "did:mycelix:authority".into(),
            published_at: ts(10),
        };
        assert!(candidate.validate_structure().is_err());
    }
}

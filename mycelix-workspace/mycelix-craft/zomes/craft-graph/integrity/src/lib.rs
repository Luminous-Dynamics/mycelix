// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Craft Graph Integrity Zome
//!
//! Defines entry types and validation rules for the Craft/Workforce graph.

use hdi::prelude::*;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CraftProfile {
    pub agent_did: String,
    pub display_name: String,
    pub headline: String,
    pub bio: String,
    pub location: String,
    pub website: String,
    pub avatar_url: String,
    pub primary_skill: String,
    pub mastery_level: u16,
    pub endorsements_count: u32,
    pub updated_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PublishedCredential {
    pub credential_hash: ActionHash,
    pub issuer_did: String,
    pub issuer: String,
    pub visibility: String,
    pub title: String,
    pub summary: Option<String>,
    pub mastery_level_at_issue: Option<u16>,
    pub last_retention_check: Option<String>,
    pub issued_on: String,
    pub expires_on: Option<String>,
    pub source_dna: String,
    pub entry_hash: String,
    pub action_hash: String,
    pub vitality_permille: Option<u16>,
    pub guild_id: Option<String>,
    pub guild_name: Option<String>,
    pub epistemic_code: Option<String>,
    pub fl_model_version: Option<String>,
    pub mastery_permille: Option<u16>,
    pub verified: Option<bool>,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SkillEndorsement {
    pub subject_did: String,
    pub endorsed_agent: AgentPubKey,
    pub skill: String,
    pub weight: u16,
    pub rationale: String,
    pub evidence: String,
    pub timestamp: i64,
    pub created_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RetentionCheck {
    pub agent: AgentPubKey,
    pub skill: String,
    pub credential_id: String,
    pub retention_score_permille: u16,
    pub questions_attempted: u16,
    pub questions_correct: u16,
    pub timestamp: i64,
    pub checked_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CompositeProfile {
    pub identity_hash: ActionHash,
    pub workforce_hash: ActionHash,
    pub agent: AgentPubKey,
    pub archetype_name: String,
    pub credential_titles: Vec<String>,
    pub coverage_permille: u16,
    pub career_profile_match: Option<String>,
    pub detected_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RetentionProof {
    pub check_hash: ActionHash,
    pub proof_data: Vec<u8>,
    pub credential_id: String,
    pub threshold_permille: u16,
    pub proof_bytes: Vec<u8>,
    pub score_commitment: Vec<u8>,
    pub domain_tag: String,
    pub proven_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(EntryTypesUnit)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    CraftProfile(CraftProfile),
    #[entry_type(visibility = "public")]
    PublishedCredential(PublishedCredential),
    #[entry_type(visibility = "public")]
    SkillEndorsement(SkillEndorsement),
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    #[entry_type(visibility = "public")]
    RetentionCheck(RetentionCheck),
    #[entry_type(visibility = "public")]
    CompositeProfile(CompositeProfile),
    #[entry_type(visibility = "public")]
    RetentionProof(RetentionProof),
}

#[hdk_link_types]
pub enum LinkTypes {
    AgentToProfile,
    AgentToCredential,
    SkillToCredential,
    SkillEndorsement,
    ProfileToCredential,
    GuildToCredential,
    AgentToEndorsement,
    EndorsedAgentToEndorsement,
    CredentialToRetentionCheck,
    AgentToCompositeProfile,
    CredentialToRetentionProof,
}

// ============== Validation Functions ==============

/// Enforce that a profile's `agent_did` is the committing agent's own DID.
/// Pure so it can be unit-tested without a full Create action. The
/// coordinator's `create_profile` already derives `agent_did` entirely from
/// `agent_info()` (no user input at all, no on-behalf path), so binding
/// unconditionally never rejects a legitimate profile -- this just makes
/// that coordinator-side guarantee real at the DHT level too (a modified
/// coordinator could otherwise set `agent_did` to anything).
fn require_profile_agent_is_author(agent_did: &str, author_did: &str) -> ValidateCallbackResult {
    if agent_did != author_did {
        return ValidateCallbackResult::Invalid(format!(
            "CraftProfile agent_did must be the committing agent (profile \
             identity forgery). Expected '{author_did}', got '{agent_did}'"
        ));
    }
    ValidateCallbackResult::Valid
}

pub fn validate_create_profile(
    action: &Create,
    profile: &CraftProfile,
) -> ExternResult<ValidateCallbackResult> {
    if profile.mastery_level > 1000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Mastery level must be in 0..1000".into(),
        ));
    }
    let author_did = action.author.to_string();
    if let ValidateCallbackResult::Invalid(msg) =
        require_profile_agent_is_author(&profile.agent_did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    Ok(ValidateCallbackResult::Valid)
}

pub fn validate_create_skill_endorsement(
    endorsement: &SkillEndorsement,
) -> ExternResult<ValidateCallbackResult> {
    if endorsement.weight > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Weight must be in 0..100".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(entry) => match entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::CraftProfile(profile) => validate_create_profile(&action, &profile),
                EntryTypes::SkillEndorsement(endorsement) => {
                    validate_create_skill_endorsement(&endorsement)
                }
                _ => Ok(ValidateCallbackResult::Valid),
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

#[cfg(test)]
mod tests {
    use super::*;

    fn test_action() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0u8; 36]),
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    /// The DID string that matches `test_action()`'s author -- craft-graph's
    /// `agent_did` is the raw `AgentPubKey::to_string()` (NOT prefixed with
    /// "did:mycelix:", unlike mycelix-attribution's zomes -- matches the
    /// coordinator's own `agent.to_string()` in `create_profile`).
    fn test_author_agent_did() -> String {
        test_action().author.to_string()
    }

    fn valid_profile() -> CraftProfile {
        CraftProfile {
            agent_did: test_author_agent_did(),
            display_name: "Ada".into(),
            headline: "Rust + Holochain".into(),
            bio: "Building sovereign infrastructure.".into(),
            location: "Remote".into(),
            website: "https://example.com".into(),
            avatar_url: "https://example.com/avatar.png".into(),
            primary_skill: "Rust".into(),
            mastery_level: 800,
            endorsements_count: 0,
            updated_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn test_valid_profile() {
        let result = validate_create_profile(&test_action(), &valid_profile()).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_mastery_level_over_1000_rejected() {
        let mut p = valid_profile();
        p.mastery_level = 1001;
        let result = validate_create_profile(&test_action(), &p).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_profile_identity_forgery_rejected() {
        // The P0 case: an agent commits a CraftProfile claiming a DIFFERENT
        // agent_did than their own -- must be rejected (profile identity
        // forgery / impersonation).
        let mut p = valid_profile();
        p.agent_did = "someone-else-entirely".into();
        let result = validate_create_profile(&test_action(), &p).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_require_profile_agent_is_author_helper() {
        let me = "uhCAkSELF";
        let victim = "uhCAkVICTIM";
        assert!(matches!(
            require_profile_agent_is_author(me, me),
            ValidateCallbackResult::Valid
        ));
        assert!(matches!(
            require_profile_agent_is_author(victim, me),
            ValidateCallbackResult::Invalid(_)
        ));
    }
}

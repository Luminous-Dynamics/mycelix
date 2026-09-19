// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Pure consensus helper for Hearth membership authority proofs.
//!
//! This crate intentionally performs no Holochain host calls. Integrity zomes
//! gather deterministic source-chain evidence, decode canonical membership
//! revisions, then feed the ordered revisions into this kernel.
//!
//! The governing rule is simple but security-sensitive:
//!
//! > A historical Active membership is not authority if a newer matching
//! > membership revision exists before the action being validated.

use hdi::prelude::{ActionHash, AgentPubKey};
use hearth_types::{MemberRole, MembershipStatus};
use std::fmt;

/// Canonical membership evidence reduced to the fields needed for authority.
///
/// Integrity adapters SHOULD construct this only from records that have already
/// passed their own zome validation and whose AppEntryDef matches the claimed
/// HearthMembership entry definition.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MembershipRevision {
    pub action_hash: ActionHash,
    pub hearth_hash: ActionHash,
    pub agent: AgentPubKey,
    pub role: MemberRole,
    pub status: MembershipStatus,
}

impl MembershipRevision {
    pub fn new(
        action_hash: ActionHash,
        hearth_hash: ActionHash,
        agent: AgentPubKey,
        role: MemberRole,
        status: MembershipStatus,
    ) -> Self {
        Self {
            action_hash,
            hearth_hash,
            agent,
            role,
            status,
        }
    }
}

/// Failure to prove fresh Hearth membership authority.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MembershipAuthorityError {
    /// The claimed membership belongs to another agent.
    ClaimedAgentMismatch,
    /// The claimed membership belongs to another Hearth.
    ClaimedHearthMismatch,
    /// The claimed membership is not Active even before freshness is checked.
    ClaimedMembershipInactive(MembershipStatus),
    /// The actor chain contains a newer matching membership revision.
    Superseded {
        claimed: ActionHash,
        latest: ActionHash,
        latest_status: MembershipStatus,
    },
    /// The supplied newest-first chain evidence never contained the claimed
    /// matching membership revision.
    ClaimedMembershipNotInEvidence,
}

impl fmt::Display for MembershipAuthorityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ClaimedAgentMismatch => {
                write!(f, "claimed Hearth membership belongs to another agent")
            }
            Self::ClaimedHearthMismatch => {
                write!(f, "claimed Hearth membership belongs to another Hearth")
            }
            Self::ClaimedMembershipInactive(status) => {
                write!(f, "claimed Hearth membership is not Active: {status:?}")
            }
            Self::Superseded {
                claimed,
                latest,
                latest_status,
            } => write!(
                f,
                "claimed Hearth membership {claimed} is superseded by {latest} with status {latest_status:?}"
            ),
            Self::ClaimedMembershipNotInEvidence => write!(
                f,
                "claimed Hearth membership was not found in the supplied source-chain evidence"
            ),
        }
    }
}

impl std::error::Error for MembershipAuthorityError {}

/// Return the actor's fresh current Hearth role when the claimed membership is
/// exactly the newest matching revision in source-chain evidence.
///
/// `revisions_newest_first` MUST be ordered from the transition's previous
/// action backward toward `claimed.action_hash`. It may contain memberships for
/// other Hearths; those are ignored. An integrity adapter is expected to obtain
/// this ordering from deterministic `must_get_agent_activity` evidence.
///
/// Security properties:
///
/// - a newer Departed/Ancestral revision invalidates an old Active claim;
/// - a newer Active revision also invalidates an older Active claim, forcing
///   callers to present the latest authority-bearing revision;
/// - membership activity in other Hearths cannot revoke or strengthen this
///   Hearth's authority;
/// - another agent's membership cannot satisfy the proof.
pub fn require_fresh_active_membership<'a, I>(
    actor: &AgentPubKey,
    hearth_hash: &ActionHash,
    claimed: &MembershipRevision,
    revisions_newest_first: I,
) -> Result<MemberRole, MembershipAuthorityError>
where
    I: IntoIterator<Item = &'a MembershipRevision>,
{
    if &claimed.agent != actor {
        return Err(MembershipAuthorityError::ClaimedAgentMismatch);
    }
    if &claimed.hearth_hash != hearth_hash {
        return Err(MembershipAuthorityError::ClaimedHearthMismatch);
    }
    if claimed.status != MembershipStatus::Active {
        return Err(MembershipAuthorityError::ClaimedMembershipInactive(
            claimed.status.clone(),
        ));
    }

    for revision in revisions_newest_first {
        if &revision.agent != actor || &revision.hearth_hash != hearth_hash {
            continue;
        }

        if revision.action_hash != claimed.action_hash {
            return Err(MembershipAuthorityError::Superseded {
                claimed: claimed.action_hash.clone(),
                latest: revision.action_hash.clone(),
                latest_status: revision.status.clone(),
            });
        }

        // The first matching revision is the claimed revision itself. Because
        // the claim was already required Active, freshness + activity are now
        // both established.
        return Ok(claimed.role.clone());
    }

    Err(MembershipAuthorityError::ClaimedMembershipNotInEvidence)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
    }

    fn hash(byte: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![byte; 36])
    }

    fn membership(
        action_byte: u8,
        hearth_byte: u8,
        agent_byte: u8,
        role: MemberRole,
        status: MembershipStatus,
    ) -> MembershipRevision {
        MembershipRevision::new(
            hash(action_byte),
            hash(hearth_byte),
            agent(agent_byte),
            role,
            status,
        )
    }

    #[test]
    fn latest_active_claim_passes_and_returns_role() {
        let actor = agent(0xaa);
        let hearth = hash(0x11);
        let claim = membership(
            0x21,
            0x11,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Active,
        );

        let role = require_fresh_active_membership(&actor, &hearth, &claim, [&claim]).unwrap();
        assert_eq!(role, MemberRole::Adult);
    }

    #[test]
    fn newer_departed_revision_revokes_old_active_claim() {
        let actor = agent(0xaa);
        let hearth = hash(0x11);
        let claim = membership(
            0x21,
            0x11,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Active,
        );
        let departed = membership(
            0x22,
            0x11,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Departed,
        );

        let error = require_fresh_active_membership(
            &actor,
            &hearth,
            &claim,
            [&departed, &claim],
        )
        .unwrap_err();

        assert_eq!(
            error,
            MembershipAuthorityError::Superseded {
                claimed: claim.action_hash.clone(),
                latest: departed.action_hash.clone(),
                latest_status: MembershipStatus::Departed,
            }
        );
    }

    #[test]
    fn newer_active_revision_also_makes_old_claim_stale() {
        let actor = agent(0xaa);
        let hearth = hash(0x11);
        let old = membership(
            0x21,
            0x11,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Active,
        );
        let newer = membership(
            0x22,
            0x11,
            0xaa,
            MemberRole::Elder,
            MembershipStatus::Active,
        );

        assert!(matches!(
            require_fresh_active_membership(&actor, &hearth, &old, [&newer, &old]),
            Err(MembershipAuthorityError::Superseded { latest_status: MembershipStatus::Active, .. })
        ));

        let role = require_fresh_active_membership(&actor, &hearth, &newer, [&newer]).unwrap();
        assert_eq!(role, MemberRole::Elder);
    }

    #[test]
    fn other_hearth_revisions_do_not_supersede_claim() {
        let actor = agent(0xaa);
        let target_hearth = hash(0x11);
        let claim = membership(
            0x21,
            0x11,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Active,
        );
        let other_hearth_departure = membership(
            0x30,
            0x12,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Departed,
        );

        let role = require_fresh_active_membership(
            &actor,
            &target_hearth,
            &claim,
            [&other_hearth_departure, &claim],
        )
        .unwrap();
        assert_eq!(role, MemberRole::Adult);
    }

    #[test]
    fn other_agent_revisions_do_not_supersede_claim() {
        let actor = agent(0xaa);
        let hearth = hash(0x11);
        let claim = membership(
            0x21,
            0x11,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Active,
        );
        let other_agent_departure = membership(
            0x30,
            0x11,
            0xbb,
            MemberRole::Adult,
            MembershipStatus::Departed,
        );

        let role = require_fresh_active_membership(
            &actor,
            &hearth,
            &claim,
            [&other_agent_departure, &claim],
        )
        .unwrap();
        assert_eq!(role, MemberRole::Adult);
    }

    #[test]
    fn claimed_membership_must_belong_to_actor() {
        let actor = agent(0xaa);
        let hearth = hash(0x11);
        let claim = membership(
            0x21,
            0x11,
            0xbb,
            MemberRole::Adult,
            MembershipStatus::Active,
        );

        assert_eq!(
            require_fresh_active_membership(&actor, &hearth, &claim, [&claim]),
            Err(MembershipAuthorityError::ClaimedAgentMismatch)
        );
    }

    #[test]
    fn claimed_membership_must_belong_to_requested_hearth() {
        let actor = agent(0xaa);
        let hearth = hash(0x11);
        let claim = membership(
            0x21,
            0x12,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Active,
        );

        assert_eq!(
            require_fresh_active_membership(&actor, &hearth, &claim, [&claim]),
            Err(MembershipAuthorityError::ClaimedHearthMismatch)
        );
    }

    #[test]
    fn claimed_membership_must_itself_be_active() {
        let actor = agent(0xaa);
        let hearth = hash(0x11);
        let claim = membership(
            0x21,
            0x11,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Departed,
        );

        assert_eq!(
            require_fresh_active_membership(&actor, &hearth, &claim, [&claim]),
            Err(MembershipAuthorityError::ClaimedMembershipInactive(
                MembershipStatus::Departed
            ))
        );
    }

    #[test]
    fn claimed_membership_must_appear_in_chain_evidence() {
        let actor = agent(0xaa);
        let hearth = hash(0x11);
        let claim = membership(
            0x21,
            0x11,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Active,
        );
        let unrelated = membership(
            0x30,
            0x12,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Active,
        );

        assert_eq!(
            require_fresh_active_membership(&actor, &hearth, &claim, [&unrelated]),
            Err(MembershipAuthorityError::ClaimedMembershipNotInEvidence)
        );
    }

    #[test]
    fn newest_matching_revision_is_authoritative_even_with_interleaved_hearths() {
        let actor = agent(0xaa);
        let hearth = hash(0x11);
        let old = membership(
            0x21,
            0x11,
            0xaa,
            MemberRole::Adult,
            MembershipStatus::Active,
        );
        let other_hearth = membership(
            0x22,
            0x12,
            0xaa,
            MemberRole::Founder,
            MembershipStatus::Active,
        );
        let current = membership(
            0x23,
            0x11,
            0xaa,
            MemberRole::Elder,
            MembershipStatus::Active,
        );

        let role = require_fresh_active_membership(
            &actor,
            &hearth,
            &current,
            [&other_hearth, &current, &old],
        )
        .unwrap();
        assert_eq!(role, MemberRole::Elder);
    }
}

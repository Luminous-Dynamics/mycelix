// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Debris Bounties Integrity Zome
//!
//! Implements the "Kessler Cleanup Market" - a decentralized bounty system
//! for debris removal. Organizations can post bounties on specific debris
//! objects, and removal services can claim them upon verified removal.
//!
//! # How It Works
//!
//! 1. **Bounty Creation**: Operator posts bounty on debris threatening their assets
//! 2. **Bounty Aggregation**: Multiple parties can contribute to same bounty
//! 3. **Removal Claim**: Service provider claims intent to remove
//! 4. **Verification**: Network verifies debris is no longer tracked
//! 5. **Payout**: Bounty released to remover (via external settlement)

use hdi::prelude::*;
use mycelix_space_shared::SpaceTimestamp;

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    /// Bounty on a debris object
    DebrisBounty(DebrisBounty),

    /// Contribution to a bounty
    BountyContribution(BountyContribution),

    /// Claim to remove debris
    RemovalClaim(RemovalClaim),

    /// Verification of removal
    RemovalVerification(RemovalVerification),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Bounties for an object
    ObjectBounties,
    /// Contributions to a bounty
    BountyContributions,
    /// Claims on a bounty
    BountyClaims,
    /// All active bounties
    ActiveBounties,
    /// Bounties by contributor
    ContributorBounties,
    /// Verifications for a claim
    ClaimVerifications,
}

/// A bounty for debris removal
#[hdk_entry_helper]
#[derive(Clone)]
pub struct DebrisBounty {
    /// Unique bounty ID
    pub bounty_id: String,

    /// Target debris NORAD ID
    pub debris_norad_id: u32,

    /// Why this debris is a problem
    pub justification: String,

    /// Bounty amount (in smallest currency unit)
    pub amount: u64,

    /// Currency/token identifier
    pub currency: String,

    /// Expiration time (bounty void after this)
    pub expires_at: Option<SpaceTimestamp>,

    /// Bounty status
    pub status: BountyStatus,

    /// Creator
    pub creator: AgentPubKey,

    /// Created at
    pub created_at: SpaceTimestamp,

    /// Requirements for claiming
    pub requirements: RemovalRequirements,
}

/// Bounty status
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum BountyStatus {
    /// Open for claims
    Open,
    /// Claimed by a remover
    Claimed,
    /// Removal in progress
    InProgress,
    /// Pending verification
    PendingVerification,
    /// Successfully completed
    Completed,
    /// Expired without completion
    Expired,
    /// Cancelled by creator
    Cancelled,
}

/// Requirements for claiming the bounty
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RemovalRequirements {
    /// Minimum trust level to claim
    pub min_trust_level: u8,

    /// Required removal method
    pub allowed_methods: Vec<RemovalMethod>,

    /// Deadline for completion after claiming
    pub completion_deadline_days: u32,

    /// Number of independent verifications needed
    pub verification_threshold: u32,
}

/// Methods for debris removal
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum RemovalMethod {
    /// Controlled deorbit
    Deorbit,
    /// Capture and removal
    Capture,
    /// Deflection to graveyard orbit
    GraveyardOrbit,
    /// Any method
    Any,
}

/// Contribution to a bounty (allows multiple funders)
#[hdk_entry_helper]
#[derive(Clone)]
pub struct BountyContribution {
    /// Bounty being contributed to
    pub bounty_id: String,

    /// Amount contributed
    pub amount: u64,

    /// Currency
    pub currency: String,

    /// Contributor
    pub contributor: AgentPubKey,

    /// Message/reason
    pub message: Option<String>,

    /// Contributed at
    pub contributed_at: SpaceTimestamp,
}

/// Claim to remove debris
#[hdk_entry_helper]
#[derive(Clone)]
pub struct RemovalClaim {
    /// Bounty being claimed
    pub bounty_id: String,

    /// Organization claiming
    pub claimer: AgentPubKey,

    /// Organization name
    pub organization: String,

    /// Proposed removal method
    pub method: RemovalMethod,

    /// Estimated completion date
    pub estimated_completion: SpaceTimestamp,

    /// Mission plan summary
    pub mission_plan: String,

    /// Claim status
    pub status: ClaimStatus,

    /// Claimed at
    pub claimed_at: SpaceTimestamp,
}

/// Claim status
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum ClaimStatus {
    /// Pending approval
    Pending,
    /// Approved, work can begin
    Approved,
    /// Work in progress
    InProgress,
    /// Submitted for verification
    Submitted,
    /// Completed and verified
    Completed,
    /// Failed or abandoned
    Failed,
    /// Rejected
    Rejected,
}

/// Verification of successful removal
#[hdk_entry_helper]
#[derive(Clone)]
pub struct RemovalVerification {
    /// Claim being verified
    pub claim_id: ActionHash,

    /// Verifier
    pub verifier: AgentPubKey,

    /// Verification result
    pub verified: bool,

    /// Evidence/reasoning
    pub evidence: VerificationEvidence,

    /// Verified at
    pub verified_at: SpaceTimestamp,
}

/// Evidence for verification
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerificationEvidence {
    /// Last observed time (should be before deorbit)
    pub last_observed: Option<SpaceTimestamp>,

    /// Predicted reentry time
    pub predicted_reentry: Option<SpaceTimestamp>,

    /// Number of sensors that lost track
    pub sensors_lost_track: u32,

    /// Hash of supporting data
    pub data_hash: Option<[u8; 32]>,

    /// Textual notes
    pub notes: String,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, action }) => match app_entry {
            EntryTypes::DebrisBounty(bounty) => {
                validate_create_bounty(EntryCreationAction::Create(action), bounty)
            }
            EntryTypes::BountyContribution(contrib) => {
                validate_create_contribution(EntryCreationAction::Create(action), contrib)
            }
            EntryTypes::RemovalClaim(claim) => {
                validate_create_claim(EntryCreationAction::Create(action), claim)
            }
            EntryTypes::RemovalVerification(verif) => {
                validate_create_verification(EntryCreationAction::Create(action), verif)
            }
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry {
            app_entry, action, ..
        }) => match app_entry {
            EntryTypes::DebrisBounty(bounty) => validate_update_bounty(action, bounty),
            // Contributions/claims/verifications have no update path in the
            // coordinator at all -- reject outright rather than leave an
            // unbound dead-code path (P0 wide-open RegisterUpdate gap).
            EntryTypes::BountyContribution(_) => Ok(ValidateCallbackResult::Invalid(
                "BountyContribution entries cannot be updated".to_string(),
            )),
            EntryTypes::RemovalClaim(_) => Ok(ValidateCallbackResult::Invalid(
                "RemovalClaim entries cannot be updated".to_string(),
            )),
            EntryTypes::RemovalVerification(_) => Ok(ValidateCallbackResult::Invalid(
                "RemovalVerification entries cannot be updated".to_string(),
            )),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_bounty(
    action: EntryCreationAction,
    bounty: DebrisBounty,
) -> ExternResult<ValidateCallbackResult> {
    // NORAD ID must be valid
    if bounty.debris_norad_id == 0 || bounty.debris_norad_id > 999999 {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid debris NORAD ID".to_string(),
        ));
    }

    // Amount must be positive
    if bounty.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Bounty amount must be positive".to_string(),
        ));
    }

    // Justification must not be empty
    if bounty.justification.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Justification cannot be empty".to_string(),
        ));
    }

    // Bind the bounty to its committer -- create_bounty already derives
    // creator from agent_info() coordinator-side with zero user input, so
    // this never rejects a legitimate bounty; it's the real DHT-level
    // enforcement a modified coordinator could otherwise bypass (P0
    // author-binding gap).
    if bounty.creator != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "DebrisBounty must be created by the committing agent (creator forgery)".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Re-derives `update_bounty_status`'s own coordinator-side rule -- "only the
/// bounty creator can cancel a bounty" -- at the DHT level, since a modified
/// coordinator could otherwise skip that check entirely (P0 author-binding
/// gap). Other status transitions stay role-gated (via `gate_space_operation`
/// coordinator-side), not ownership-gated, matching the coordinator's own
/// deliberate design -- this only checks the Cancelled transition
/// specifically, not every field on every update.
fn validate_update_bounty(
    action: Update,
    bounty: DebrisBounty,
) -> ExternResult<ValidateCallbackResult> {
    if bounty.debris_norad_id == 0 || bounty.debris_norad_id > 999999 {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid debris NORAD ID".to_string(),
        ));
    }
    if bounty.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Bounty amount must be positive".to_string(),
        ));
    }
    if bounty.justification.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Justification cannot be empty".to_string(),
        ));
    }

    if bounty.status == BountyStatus::Cancelled {
        let original_record = must_get_valid_record(action.original_action_address.clone())?;
        let original_bounty: DebrisBounty = original_record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(e))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Invalid original DebrisBounty entry".to_string()
            )))?;
        if action.author != original_bounty.creator {
            return Ok(ValidateCallbackResult::Invalid(
                "Only the bounty creator can cancel a bounty".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_contribution(
    action: EntryCreationAction,
    contrib: BountyContribution,
) -> ExternResult<ValidateCallbackResult> {
    if contrib.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Contribution amount must be positive".to_string(),
        ));
    }

    // Bind the contribution to its committer -- contribute_to_bounty already
    // derives contributor from agent_info() coordinator-side with zero user
    // input (P0 author-binding gap).
    if contrib.contributor != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "BountyContribution must be contributed by the committing agent (contributor forgery)"
                .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_claim(
    action: EntryCreationAction,
    claim: RemovalClaim,
) -> ExternResult<ValidateCallbackResult> {
    if claim.organization.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Organization name cannot be empty".to_string(),
        ));
    }

    if claim.mission_plan.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Mission plan cannot be empty".to_string(),
        ));
    }

    // Bind the claim to its committer -- claim_bounty already derives
    // claimer from agent_info() coordinator-side with zero user input (P0
    // author-binding gap).
    if claim.claimer != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "RemovalClaim must be claimed by the committing agent (claimer forgery)".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_verification(
    action: EntryCreationAction,
    verif: RemovalVerification,
) -> ExternResult<ValidateCallbackResult> {
    // Evidence notes must not be empty
    if verif.evidence.notes.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Verification evidence notes cannot be empty".to_string(),
        ));
    }

    // At least one form of evidence must be present
    let has_observation = verif.evidence.last_observed.is_some();
    let has_reentry = verif.evidence.predicted_reentry.is_some();
    let has_sensor_loss = verif.evidence.sensors_lost_track > 0;
    let has_data = verif.evidence.data_hash.is_some();

    if !has_observation && !has_reentry && !has_sensor_loss && !has_data {
        return Ok(ValidateCallbackResult::Invalid(
            "Verification must include at least one form of evidence (observation, reentry prediction, sensor loss, or data hash)".to_string()
        ));
    }

    // If reentry is predicted, it should be after last observation
    if let (Some(last_obs), Some(reentry)) = (
        &verif.evidence.last_observed,
        &verif.evidence.predicted_reentry,
    ) {
        if reentry.micros < last_obs.micros {
            return Ok(ValidateCallbackResult::Invalid(
                "Predicted reentry cannot be before last observation".to_string(),
            ));
        }
    }

    // Bind the verification to its committer -- submit_verification already
    // derives verifier from agent_info() coordinator-side with zero user
    // input (P0 author-binding gap).
    if verif.verifier != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "RemovalVerification must be verified by the committing agent (verifier forgery)"
                .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

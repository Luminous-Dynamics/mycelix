#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Staking Integrity Zome
//!
//! Defines entry types for SAP-based collateral staking with MYCEL weighting:
//! - Collateral stakes (SAP + MYCEL score)
//! - Slashing conditions and evidence
//! - Crypto escrow with release conditions
//! - Reward distributions

use hdi::prelude::*;
use mycelix_bridge_entry_types::{did_for_author, require_did_is_author};

// =============================================================================
// STRING LENGTH LIMITS — Prevent DHT bloat attacks
// =============================================================================

const MAX_DID_LEN: usize = 256;
const MAX_ID_LEN: usize = 256;
const MAX_PURPOSE_LEN: usize = 1024;
/// Maximum size for serialized slashing evidence (64KB)
const MAX_EVIDENCE_BYTES: usize = 65536;

/// Collateral Stake Position
///
/// Represents a validator's stake using SAP collateral with MYCEL weighting.
/// Stake weight is determined by 1.0 + mycel_score, giving a range of 1.0-2.0.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CollateralStake {
    /// Unique stake identifier
    pub id: String,
    /// Staker's DID
    pub staker_did: String,
    /// SAP collateral amount
    pub sap_amount: u64,
    /// MYCEL score at time of staking (affects weight)
    pub mycel_score: f32,
    /// Stake weight = 1.0 + mycel_score (range 1.0-2.0)
    pub stake_weight: f32,
    /// Stake creation timestamp
    pub staked_at: Timestamp,
    /// Unbonding period end (None if not unbonding)
    pub unbonding_until: Option<Timestamp>,
    /// Current stake status
    pub status: StakeStatus,
    /// Accumulated pending rewards
    pub pending_rewards: u64,
    /// Last reward claim timestamp
    pub last_reward_claim: Timestamp,
}

/// Stake status
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum StakeStatus {
    /// Active and earning rewards
    Active,
    /// In unbonding period (21 days)
    Unbonding,
    /// Withdrawn after unbonding
    Withdrawn,
    /// Partially slashed
    Slashed,
    /// Fully slashed (jailed)
    Jailed,
}

impl StakeStatus {
    /// Valid status transitions. Prevents reversal of terminal states.
    pub fn can_transition_to(&self, new: &StakeStatus) -> bool {
        matches!(
            (self, new),
            (StakeStatus::Active, StakeStatus::Unbonding)
                | (StakeStatus::Active, StakeStatus::Slashed)
                | (StakeStatus::Active, StakeStatus::Jailed)
                | (StakeStatus::Unbonding, StakeStatus::Withdrawn)
                | (StakeStatus::Slashed, StakeStatus::Unbonding)
                | (StakeStatus::Slashed, StakeStatus::Jailed)
        )
    }
}

/// Slashing Event
///
/// Records a slashing event with cryptographic evidence
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SlashingEvent {
    /// Unique event identifier
    pub id: String,
    /// Stake ID being slashed
    pub stake_id: String,
    /// Staker DID
    pub staker_did: String,
    /// Slashing reason
    pub reason: SlashingReason,
    /// Percentage of stake slashed (0-100)
    pub slash_percentage: u8,
    /// SAP amount slashed
    pub sap_slashed: u64,
    /// Cryptographic evidence hash
    pub evidence_hash: Vec<u8>,
    /// Evidence data (serialized SlashingEvidence)
    pub evidence: Vec<u8>,
    /// Timestamp of slashing
    pub slashed_at: Timestamp,
    /// Whether staker is jailed (cannot restake)
    pub jailed: bool,
    /// Jail release timestamp (if jailed)
    pub jail_release: Option<Timestamp>,
}

/// Reasons for slashing
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum SlashingReason {
    /// Double signing blocks
    DoubleSigning,
    /// Prolonged downtime
    Downtime,
    /// Byzantine behavior in consensus
    ByzantineConsensus,
    /// Invalid gradient submission (FL)
    InvalidGradient,
    /// Cartel/collusion detected
    CartelActivity,
    /// Governance manipulation
    GovernanceManipulation,
}

impl SlashingReason {
    /// Get default slash percentage for this reason
    pub fn default_slash_percentage(&self) -> u8 {
        match self {
            SlashingReason::DoubleSigning => 100,     // Full slash, jail
            SlashingReason::Downtime => 5,            // Minor
            SlashingReason::ByzantineConsensus => 50, // Severe
            SlashingReason::InvalidGradient => 10,    // Moderate
            SlashingReason::CartelActivity => 75,     // Very severe
            SlashingReason::GovernanceManipulation => 50,
        }
    }

    /// Check if this reason results in jailing
    pub fn results_in_jail(&self) -> bool {
        matches!(
            self,
            SlashingReason::DoubleSigning | SlashingReason::CartelActivity
        )
    }
}

/// Cryptographic slashing evidence
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SlashingEvidence {
    /// Type of evidence
    pub evidence_type: EvidenceType,
    /// Block/round number where violation occurred
    pub violation_height: u64,
    /// Conflicting data (e.g., two signed blocks)
    pub conflicting_data: Vec<Vec<u8>>,
    /// Signatures proving the violation
    pub signatures: Vec<Vec<u8>>,
    /// Timestamp of violation
    pub violation_time: u64,
    /// Merkle proof of inclusion (if applicable)
    pub merkle_proof: Option<Vec<u8>>,
    /// Witnesses who reported the violation
    pub reporters: Vec<String>,
}

/// Types of slashing evidence
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EvidenceType {
    /// Two different blocks signed at same height
    DoubleSignEvidence,
    /// Missed blocks attestation
    DowntimeEvidence,
    /// Invalid state transition proof
    InvalidStateProof,
    /// Gradient analysis showing Byzantine behavior
    GradientAnalysis,
    /// Graph clustering showing cartel
    CartelProof,
}

/// Escrow with cryptographic release conditions
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CryptoEscrow {
    /// Unique escrow identifier
    pub id: String,
    /// Depositor DID
    pub depositor_did: String,
    /// Beneficiary DID
    pub beneficiary_did: String,
    /// SAP amount escrowed
    pub sap_amount: u64,
    /// Escrow purpose
    pub purpose: String,
    /// Release conditions
    pub conditions: Vec<ReleaseCondition>,
    /// Number of conditions that must be met
    pub required_conditions: u8,
    /// Conditions currently met
    pub met_conditions: Vec<u8>,
    /// Hash commitment of release secret (for hash-lock)
    pub hash_lock: Option<Vec<u8>>,
    /// Timelock expiration
    pub timelock: Option<Timestamp>,
    /// Multi-sig threshold (e.g., 2-of-3)
    pub multisig_threshold: Option<u8>,
    /// Multi-sig signers
    pub multisig_signers: Vec<String>,
    /// Collected signatures
    pub collected_signatures: Vec<EscrowSignature>,
    /// Escrow status
    pub status: EscrowStatus,
    /// Creation timestamp
    pub created_at: Timestamp,
    /// Release timestamp
    pub released_at: Option<Timestamp>,
}

/// Release conditions for escrow
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ReleaseCondition {
    /// Time-based release
    Timelock { release_time: i64 },
    /// Hash preimage reveal
    HashLock {
        hash: Vec<u8>,
        hash_type: EscrowHashType,
    },
    /// Multi-signature approval
    MultiSig { threshold: u8, signers: Vec<String> },
    /// Oracle price attestation
    OraclePrice {
        oracle_id: String,
        asset: String,
        min_price: f64,
    },
    /// Governance proposal passed
    GovernanceApproval { proposal_id: String },
    /// MYCEL threshold met
    MycelThreshold { min_mycel_score: f32 },
}

/// Hash types for hash-lock
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EscrowHashType {
    Sha256,
    Sha3_256,
    Blake2b,
    Keccak256,
}

/// Signature on escrow release (legacy, embedded in CryptoEscrow)
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EscrowSignature {
    pub signer_did: String,
    pub signature: Vec<u8>,
    pub signed_at: i64,
}

/// Immutable escrow signature entry (RC-18 fix: avoids race condition on mutable array)
///
/// Each signature is stored as its own DHT entry and linked to the escrow via
/// `EscrowToSignatures`. This eliminates the concurrent-update race on
/// `CryptoEscrow.collected_signatures`.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EscrowSignatureEntry {
    /// Escrow ID this signature belongs to
    pub escrow_id: String,
    /// Signer's DID
    pub signer_did: String,
    /// Cryptographic signature bytes
    pub signature: Vec<u8>,
    /// When the signature was created
    pub timestamp: Timestamp,
}

/// Escrow status
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EscrowStatus {
    /// Awaiting condition fulfillment
    Pending,
    /// Conditions met, awaiting release
    Releasable,
    /// Released to beneficiary
    Released,
    /// Refunded to depositor
    Refunded,
    /// Disputed
    Disputed,
    /// Expired without resolution
    Expired,
}

impl EscrowStatus {
    /// Valid status transitions. Terminal states (Released, Refunded, Expired) cannot change.
    pub fn can_transition_to(&self, new: &EscrowStatus) -> bool {
        matches!(
            (self, new),
            (EscrowStatus::Pending, EscrowStatus::Releasable)
                | (EscrowStatus::Pending, EscrowStatus::Refunded)
                | (EscrowStatus::Pending, EscrowStatus::Disputed)
                | (EscrowStatus::Pending, EscrowStatus::Expired)
                | (EscrowStatus::Releasable, EscrowStatus::Released)
                | (EscrowStatus::Releasable, EscrowStatus::Disputed)
                | (EscrowStatus::Disputed, EscrowStatus::Released)
                | (EscrowStatus::Disputed, EscrowStatus::Refunded)
        )
    }
}

/// Staking rewards distribution
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RewardDistribution {
    /// Distribution ID
    pub id: String,
    /// Epoch number
    pub epoch: u64,
    /// Total rewards distributed
    pub total_rewards: u64,
    /// Per-staker allocations
    pub allocations: Vec<RewardAllocation>,
    /// Distribution timestamp
    pub distributed_at: Timestamp,
    /// Merkle root of allocations (for efficient claiming)
    pub merkle_root: Vec<u8>,
}

/// Individual reward allocation
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct RewardAllocation {
    pub stake_id: String,
    pub staker_did: String,
    pub amount: u64,
    pub merkle_proof: Vec<u8>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    CollateralStake(CollateralStake),
    SlashingEvent(SlashingEvent),
    CryptoEscrow(CryptoEscrow),
    RewardDistribution(RewardDistribution),
    EscrowSignatureEntry(EscrowSignatureEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Staker to their stakes
    StakerToStake,
    /// Stake to slashing events
    StakeToSlashing,
    /// Escrow to depositor
    DepositorToEscrow,
    /// Escrow to beneficiary
    BeneficiaryToEscrow,
    /// Epoch to reward distribution
    EpochToRewards,
    /// Active stakes anchor
    ActiveStakes,
    /// Stake ID to stake entry (O(1) lookup by ID)
    StakeIdToStake,
    /// Escrow ID to escrow entry (O(1) lookup by ID)
    EscrowIdToEscrow,
    /// Link from governance_agents anchor to authorized agent pubkeys
    GovernanceAgents,
    /// Link from escrow signature anchor to individual EscrowSignatureEntry entries
    EscrowToSignatures,
}

/// Validation callback
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::CollateralStake(stake) => validate_create_stake(action, stake),
                EntryTypes::SlashingEvent(event) => validate_slashing_event(action, event),
                EntryTypes::CryptoEscrow(escrow) => validate_escrow(action, escrow),
                EntryTypes::RewardDistribution(dist) => validate_reward_distribution(action, dist),
                EntryTypes::EscrowSignatureEntry(sig) => {
                    validate_escrow_signature_entry(action, sig)
                }
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::CollateralStake(stake) => validate_update_stake(action, stake),
                EntryTypes::CryptoEscrow(escrow) => validate_update_escrow(action, escrow),
                EntryTypes::SlashingEvent(_) => Ok(ValidateCallbackResult::Invalid(
                    "Slashing events are immutable".into(),
                )),
                EntryTypes::RewardDistribution(_) => Ok(ValidateCallbackResult::Invalid(
                    "Reward distributions are immutable".into(),
                )),
                EntryTypes::EscrowSignatureEntry(_) => Ok(ValidateCallbackResult::Invalid(
                    "Escrow signatures are immutable".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::StakerToStake => Ok(ValidateCallbackResult::Valid),
            LinkTypes::StakeToSlashing => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DepositorToEscrow => Ok(ValidateCallbackResult::Valid),
            LinkTypes::BeneficiaryToEscrow => Ok(ValidateCallbackResult::Valid),
            LinkTypes::EpochToRewards => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ActiveStakes => Ok(ValidateCallbackResult::Valid),
            LinkTypes::StakeIdToStake => Ok(ValidateCallbackResult::Valid),
            LinkTypes::EscrowIdToEscrow => Ok(ValidateCallbackResult::Valid),
            LinkTypes::GovernanceAgents => Ok(ValidateCallbackResult::Valid),
            LinkTypes::EscrowToSignatures => Ok(ValidateCallbackResult::Valid),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_stake(
    action: Create,
    stake: CollateralStake,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the stake to its committer — the coordinator's create_stake already
    // enforces verify_caller_is_did(staker_did); enforce it in integrity too so
    // a modified coordinator can't forge stakes for another DID or spin up Sybil
    // stakes on victims' identities. (You always stake your OWN collateral, so
    // this never rejects a legitimate stake.)
    let author_did = did_for_author(&action.author);
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("Stake", "staker_did", &stake.staker_did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    // String length checks — prevent DHT bloat
    if stake.staker_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if stake.id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Stake ID exceeds maximum length".into(),
        ));
    }

    // Validate DID format
    if !stake.staker_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Staker must be a valid DID".into(),
        ));
    }

    // Validate SAP amount is non-zero
    if stake.sap_amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "SAP collateral amount must be greater than zero".into(),
        ));
    }

    // Validate MYCEL score range (0.0 to 1.0) — is_finite rejects NaN/Infinity
    if !stake.mycel_score.is_finite() || stake.mycel_score < 0.0 || stake.mycel_score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "MYCEL score must be a finite number in [0.0, 1.0]".into(),
        ));
    }

    // Validate stake weight = 1.0 + mycel_score (range 1.0-2.0)
    if !stake.stake_weight.is_finite() || stake.stake_weight < 1.0 || stake.stake_weight > 2.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Stake weight must be a finite number in [1.0, 2.0]".into(),
        ));
    }

    // Verify stake weight is consistent with mycel_score
    let expected_weight = 1.0 + stake.mycel_score;
    if (stake.stake_weight - expected_weight).abs() > 0.001 {
        return Ok(ValidateCallbackResult::Invalid(
            "Stake weight must equal 1.0 + mycel_score".into(),
        ));
    }

    // New stake must be Active
    if stake.status != StakeStatus::Active {
        return Ok(ValidateCallbackResult::Invalid(
            "New stake must have Active status".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate stake update — enforces status transition rules

fn validate_stake_update_terms(
    original_author: &AgentPubKey,
    update_author: &AgentPubKey,
    original: &CollateralStake,
    updated: &CollateralStake,
) -> ValidateCallbackResult {
    // Ordinary owner-controlled lifecycle/refresh updates must remain on the
    // stake owner's source chain. Governance slashing is intentionally left as
    // a separate authority theorem because its legitimate author is not the
    // stake owner.
    let owner_controlled = matches!(
        (&original.status, &updated.status),
        (StakeStatus::Active, StakeStatus::Active)
            | (StakeStatus::Active, StakeStatus::Unbonding)
            | (StakeStatus::Unbonding, StakeStatus::Withdrawn)
    );
    if owner_controlled && original_author != update_author {
        return ValidateCallbackResult::Invalid(
            "Owner-controlled stake updates must be authored by the original staker"
                .into(),
        );
    }

    // Identity and historical provenance are immutable across every update.
    if original.id != updated.id {
        return ValidateCallbackResult::Invalid(
            "Stake ID is immutable across updates".into(),
        );
    }
    if original.staker_did != updated.staker_did {
        return ValidateCallbackResult::Invalid(
            "Stake staker_did is immutable across updates".into(),
        );
    }
    if original.staked_at != updated.staked_at {
        return ValidateCallbackResult::Invalid(
            "Stake staked_at is immutable across updates".into(),
        );
    }
    if original.pending_rewards != updated.pending_rewards {
        return ValidateCallbackResult::Invalid(
            "Stake pending_rewards must be preserved by this update protocol".into(),
        );
    }
    if original.last_reward_claim != updated.last_reward_claim {
        return ValidateCallbackResult::Invalid(
            "Stake last_reward_claim must be preserved by this update protocol".into(),
        );
    }

    // Status changes are evaluated against the exact validated predecessor.
    if original.status != updated.status && !original.status.can_transition_to(&updated.status) {
        return ValidateCallbackResult::Invalid(format!(
            "Invalid status transition: {:?} -> {:?}",
            original.status, updated.status
        ));
    }

    // There is currently no non-governance coordinator path after a stake has
    // already entered Slashed. Do not leave those transitions open merely
    // because the generic status enum admits them; AC-145 will add an explicit
    // addressable governance authorization proof for any such mutation.
    if original.status == StakeStatus::Slashed && original.status != updated.status {
        return ValidateCallbackResult::Invalid(
            "Slashed stakes require explicit governance authorization for further status transitions"
                .into(),
        );
    }

    // SAP collateral may only be consumed by the documented terminalizing
    // transitions. Every ordinary refresh/lifecycle update preserves it.
    let collateral_terminalization = matches!(
        (&original.status, &updated.status),
        (StakeStatus::Active, StakeStatus::Slashed | StakeStatus::Jailed)
            | (StakeStatus::Unbonding, StakeStatus::Withdrawn)
    );
    if collateral_terminalization {
        if updated.sap_amount != 0 {
            return ValidateCallbackResult::Invalid(
                "Terminal stake update must set sap_amount to zero".into(),
            );
        }
    } else if original.sap_amount != updated.sap_amount {
        return ValidateCallbackResult::Invalid(
            "Stake sap_amount is immutable except for documented terminalization".into(),
        );
    }

    // Only Active -> Unbonding may introduce the unbonding deadline.
    let unbonding_initialization = matches!(
        (&original.status, &updated.status),
        (StakeStatus::Active, StakeStatus::Unbonding)
    );
    if unbonding_initialization {
        if original.unbonding_until.is_some() || updated.unbonding_until.is_none() {
            return ValidateCallbackResult::Invalid(
                "Active -> Unbonding must initialize exactly one unbonding deadline".into(),
            );
        }
    } else if original.unbonding_until != updated.unbonding_until {
        return ValidateCallbackResult::Invalid(
            "Stake unbonding_until may only change on Active -> Unbonding".into(),
        );
    }

    // MYCEL/weight changes are limited to an owner-controlled Active refresh.
    // Integrity validates their internal relation; recognition authority is a
    // separate coordinator/cross-zome theorem.
    let mycel_refresh = matches!(
        (&original.status, &updated.status),
        (StakeStatus::Active, StakeStatus::Active)
    );
    if !mycel_refresh
        && (original.mycel_score != updated.mycel_score
            || original.stake_weight != updated.stake_weight)
    {
        return ValidateCallbackResult::Invalid(
            "MYCEL and stake weight are immutable outside Active refresh".into(),
        );
    }

    // Every update must preserve the score/weight domain and relation.
    if !updated.mycel_score.is_finite()
        || updated.mycel_score < 0.0
        || updated.mycel_score > 1.0
    {
        return ValidateCallbackResult::Invalid(
            "MYCEL score must be a finite number in [0.0, 1.0]".into(),
        );
    }
    if !updated.stake_weight.is_finite()
        || updated.stake_weight < 1.0
        || updated.stake_weight > 2.0
    {
        return ValidateCallbackResult::Invalid(
            "Stake weight must be in [1.0, 2.0]".into(),
        );
    }
    let expected_weight = 1.0 + updated.mycel_score;
    if (updated.stake_weight - expected_weight).abs() > 0.001 {
        return ValidateCallbackResult::Invalid(
            "Stake weight must equal 1.0 + mycel_score".into(),
        );
    }

    ValidateCallbackResult::Valid
}


fn validate_update_stake(
    action: Update,
    stake: CollateralStake,
) -> ExternResult<ValidateCallbackResult> {
    // A staking update is valid only with an exact, valid predecessor.
    // Never convert a dependency/decoding failure into Valid.
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original = original_record
        .entry()
        .to_app_option::<CollateralStake>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode original CollateralStake predecessor: {e:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Original staking update predecessor is not a CollateralStake entry".into()
            ))
        })?;

    Ok(validate_stake_update_terms(
        original_record.action().author(),
        &action.author,
        &original,
        &stake,
    ))
}

/// Validate slashing event
fn validate_slashing_event(
    _action: Create,
    event: SlashingEvent,
) -> ExternResult<ValidateCallbackResult> {
    // String length checks — prevent DHT bloat
    if event.staker_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if event.id.len() > MAX_ID_LEN || event.stake_id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "ID exceeds maximum length".into(),
        ));
    }
    if event.evidence.len() > MAX_EVIDENCE_BYTES {
        return Ok(ValidateCallbackResult::Invalid(
            "Slashing evidence exceeds maximum size of 64KB".into(),
        ));
    }

    // Slash percentage must be valid
    if event.slash_percentage > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Slash percentage cannot exceed 100".into(),
        ));
    }

    // Evidence hash must be 32 bytes
    if event.evidence_hash.len() != 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "Evidence hash must be 32 bytes".into(),
        ));
    }

    // Evidence must not be empty
    if event.evidence.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Slashing evidence is required".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate escrow
fn validate_escrow(_action: Create, escrow: CryptoEscrow) -> ExternResult<ValidateCallbackResult> {
    // String length checks — prevent DHT bloat
    if escrow.depositor_did.len() > MAX_DID_LEN || escrow.beneficiary_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if escrow.id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Escrow ID exceeds maximum length".into(),
        ));
    }
    if escrow.purpose.len() > MAX_PURPOSE_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Purpose exceeds maximum length".into(),
        ));
    }
    for signer in &escrow.multisig_signers {
        if signer.len() > MAX_DID_LEN {
            return Ok(ValidateCallbackResult::Invalid(
                "Signer DID exceeds maximum length".into(),
            ));
        }
    }

    // Depositor must be a valid DID
    if !escrow.depositor_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Depositor must be a valid DID".into(),
        ));
    }

    // Beneficiary must be a valid DID
    if !escrow.beneficiary_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Beneficiary must be a valid DID".into(),
        ));
    }

    // SAP amount must be non-zero
    if escrow.sap_amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "SAP escrow amount must be greater than zero".into(),
        ));
    }

    // Validate float fields in release conditions (NaN bypasses comparison operators)
    for condition in &escrow.conditions {
        match condition {
            ReleaseCondition::OraclePrice { min_price, .. } => {
                if !min_price.is_finite() || *min_price <= 0.0 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "OraclePrice min_price must be a finite positive number".into(),
                    ));
                }
            }
            ReleaseCondition::MycelThreshold { min_mycel_score } => {
                if !min_mycel_score.is_finite() || *min_mycel_score < 0.0 || *min_mycel_score > 1.0
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "MycelThreshold min_mycel_score must be a finite number in [0.0, 1.0]"
                            .into(),
                    ));
                }
            }
            _ => {}
        }
    }

    // Must have at least one condition
    if escrow.conditions.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "At least one release condition is required".into(),
        ));
    }

    // Required conditions cannot exceed total conditions
    if escrow.required_conditions as usize > escrow.conditions.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "Required conditions cannot exceed total conditions".into(),
        ));
    }

    // New escrow must be Pending
    if escrow.status != EscrowStatus::Pending {
        return Ok(ValidateCallbackResult::Invalid(
            "New escrow must have Pending status".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate escrow update.
fn validate_update_escrow(
    action: Update,
    escrow: CryptoEscrow,
) -> ExternResult<ValidateCallbackResult> {
    if !escrow.depositor_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Depositor must be a valid DID".into(),
        ));
    }

    // An escrow update is valid only relative to its exact, valid predecessor.
    // Never turn a missing/unresolved/malformed predecessor into Valid.
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original = original_record
        .entry()
        .to_app_option::<CryptoEscrow>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode original CryptoEscrow predecessor: {e:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Original escrow update predecessor is not a CryptoEscrow entry".into(),
            ))
        })?;

    let semantic = validate_escrow_update_terms(&original, &escrow);
    if matches!(semantic, ValidateCallbackResult::Invalid(_)) {
        return Ok(semantic);
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate escrow signature entry
fn validate_escrow_signature_entry(
    _action: Create,
    sig: EscrowSignatureEntry,
) -> ExternResult<ValidateCallbackResult> {
    // String length checks
    if sig.escrow_id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Escrow ID exceeds maximum length".into(),
        ));
    }
    if sig.signer_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Signer DID exceeds maximum length".into(),
        ));
    }

    // Signer must be a valid DID
    if !sig.signer_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Signer must be a valid DID".into(),
        ));
    }

    // Signature must not be empty
    if sig.signature.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Signature must not be empty".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate reward distribution
fn validate_reward_distribution(
    _action: Create,
    dist: RewardDistribution,
) -> ExternResult<ValidateCallbackResult> {
    // String length checks — prevent DHT bloat
    if dist.id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Distribution ID exceeds maximum length".into(),
        ));
    }
    for alloc in &dist.allocations {
        if alloc.stake_id.len() > MAX_ID_LEN || alloc.staker_did.len() > MAX_DID_LEN {
            return Ok(ValidateCallbackResult::Invalid(
                "Allocation contains oversized ID or DID".into(),
            ));
        }
    }

    // Merkle root must be 32 bytes
    if dist.merkle_root.len() != 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "Merkle root must be 32 bytes".into(),
        ));
    }

    // Must have at least one allocation
    if dist.allocations.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "At least one allocation is required".into(),
        ));
    }

    // Sum of allocations must equal total rewards
    let sum: u64 = dist.allocations.iter().map(|a| a.amount).sum();
    if sum != dist.total_rewards {
        return Ok(ValidateCallbackResult::Invalid(
            "Sum of allocations must equal total rewards".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

// =============================================================================
// UNIT TESTS
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn staker_must_be_committing_agent() {
        let me = "did:mycelix:uhCAkSELF";
        assert!(matches!(
            require_did_is_author("Stake", "staker_did", me, me),
            ValidateCallbackResult::Valid
        ));
        // Forging a stake as another DID (or a Sybil identity) is rejected.
        match require_did_is_author("Stake", "staker_did", "did:mycelix:uhCAkVICTIM", me) {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(
                    msg.contains("forgery") || msg.contains("Sybil"),
                    "reason: {msg}"
                );
            }
            other => panic!("forged staker must be Invalid, got {other:?}"),
        }
    }

    fn ts(micros: i64) -> Timestamp {
        Timestamp::from_micros(micros)
    }

    fn valid_stake() -> CollateralStake {
        CollateralStake {
            id: "stake:test:001".into(),
            // Must equal make_create()'s author — validate_create_stake binds
            // staker_did to the committing agent. The bind landed without this
            // fixture being updated, so this test was permanently red (invisible
            // because CI ran `cargo check`, never `cargo test`). Fixed 2026-07-29.
            staker_did: test_author_did(),
            sap_amount: 1000,
            mycel_score: 0.5,
            stake_weight: 1.5,
            staked_at: ts(1_000_000),
            unbonding_until: None,
            status: StakeStatus::Active,
            pending_rewards: 0,
            last_reward_claim: ts(1_000_000),
        }
    }

    /// DID of the agent `make_create()` attributes actions to.
    fn test_author_did() -> String {
        format!("did:mycelix:{}", AgentPubKey::from_raw_36(vec![0; 36]))
    }

    fn make_create() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0; 36]),
            timestamp: ts(1_000_000),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    fn make_update() -> Update {
        Update {
            author: AgentPubKey::from_raw_36(vec![0; 36]),
            timestamp: ts(2_000_000),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            original_action_address: ActionHash::from_raw_36(vec![0; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    // ---- Stake update hardening ----

    #[test]
    fn stake_active_refresh_requires_original_staker_author() {
        let original = valid_stake();
        let mut updated = original.clone();
        updated.mycel_score = 0.8;
        updated.stake_weight = 1.8;

        let other_author = AgentPubKey::from_raw_36(vec![1; 36]);
        let result = validate_stake_update_terms(
            &AgentPubKey::from_raw_36(vec![0; 36]),
            &other_author,
            &original,
            &updated,
        );

        assert!(matches!(
            result,
            ValidateCallbackResult::Invalid(msg) if msg.contains("original staker")
        ));
    }

    #[test]
    fn stake_active_refresh_allows_owner_and_rebinds_weight() {
        let original = valid_stake();
        let mut updated = original.clone();
        updated.mycel_score = 0.8;
        updated.stake_weight = 1.8;

        let result = validate_stake_update_terms(
            &AgentPubKey::from_raw_36(vec![0; 36]),
            &AgentPubKey::from_raw_36(vec![0; 36]),
            &original,
            &updated,
        );

        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn stake_update_rejects_identity_or_collateral_mutation() {
        let original = valid_stake();

        let mut cases = Vec::new();

        let mut changed_id = original.clone();
        changed_id.id.push_str("-mutated");
        cases.push(changed_id);

        let mut changed_did = original.clone();
        changed_did.staker_did = "did:mycelix:other".into();
        cases.push(changed_did);

        let mut changed_amount = original.clone();
        changed_amount.sap_amount += 1;
        cases.push(changed_amount);

        let mut changed_time = original.clone();
        changed_time.staked_at = ts(2_000_000);
        cases.push(changed_time);

        for updated in cases {
            let result = validate_stake_update_terms(
                &AgentPubKey::from_raw_36(vec![0; 36]),
                &AgentPubKey::from_raw_36(vec![0; 36]),
                &original,
                &updated,
            );
            assert!(
                matches!(result, ValidateCallbackResult::Invalid(_)),
                "semantic predecessor binding must reject mutated stake terms"
            );
        }
    }

    #[test]
    fn active_to_unbonding_requires_one_new_deadline_and_preserves_collateral() {
        let original = valid_stake();
        let mut updated = original.clone();
        updated.status = StakeStatus::Unbonding;
        updated.unbonding_until = Some(ts(3_000_000));

        let result = validate_stake_update_terms(
            &AgentPubKey::from_raw_36(vec![0; 36]),
            &AgentPubKey::from_raw_36(vec![0; 36]),
            &original,
            &updated,
        );

        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn slashed_stake_cannot_change_status_without_governance_proof() {
        let mut original = valid_stake();
        original.status = StakeStatus::Slashed;

        for target in [StakeStatus::Jailed, StakeStatus::Unbonding] {
            let mut updated = original.clone();
            updated.status = target;
            updated.unbonding_until = None;

            let result = validate_stake_update_terms(
                &AgentPubKey::from_raw_36(vec![0; 36]),
                &AgentPubKey::from_raw_36(vec![0; 36]),
                &original,
                &updated,
            );

            assert!(
                matches!(result, ValidateCallbackResult::Invalid(msg)
                    if msg.contains("Slashed stakes require explicit governance authorization")),
                "post-slash status escalation must fail closed"
            );
        }
    }

    #[test]
    fn withdrawal_terminalization_requires_zero_remaining_stake() {
        let mut original = valid_stake();
        original.status = StakeStatus::Unbonding;
        original.unbonding_until = Some(ts(3_000_000));

        let mut updated = original.clone();
        updated.status = StakeStatus::Withdrawn;

        let result = validate_stake_update_terms(
            &AgentPubKey::from_raw_36(vec![0; 36]),
            &AgentPubKey::from_raw_36(vec![0; 36]),
            &original,
            &updated,
        );

        assert!(!matches!(result, ValidateCallbackResult::Invalid(_)));

        updated.sap_amount = 1;
        let result = validate_stake_update_terms(
            &AgentPubKey::from_raw_36(vec![0; 36]),
            &AgentPubKey::from_raw_36(vec![0; 36]),
            &original,
            &updated,
        );

        assert!(matches!(
            result,
            ValidateCallbackResult::Invalid(msg) if msg.contains("sap_amount")
        ));
    }

    // ---- Stake creation ----

    #[test]
    fn test_stake_create_valid() {
        let result = validate_create_stake(make_create(), valid_stake()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_stake_rejects_nan_mycel_score() {
        let mut stake = valid_stake();
        stake.mycel_score = f32::NAN;
        let result = validate_create_stake(make_create(), stake).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_stake_rejects_infinity_mycel_score() {
        let mut stake = valid_stake();
        stake.mycel_score = f32::INFINITY;
        let result = validate_create_stake(make_create(), stake).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_stake_rejects_nan_weight() {
        let mut stake = valid_stake();
        stake.stake_weight = f32::NAN;
        let result = validate_create_stake(make_create(), stake).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_stake_rejects_zero_sap() {
        let mut stake = valid_stake();
        stake.sap_amount = 0;
        let result = validate_create_stake(make_create(), stake).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_stake_rejects_inconsistent_weight() {
        let mut stake = valid_stake();
        stake.stake_weight = 1.9; // Should be 1.5 for mycel_score=0.5
        let result = validate_create_stake(make_create(), stake).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_stake_rejects_non_active_status() {
        let mut stake = valid_stake();
        stake.status = StakeStatus::Withdrawn;
        let result = validate_create_stake(make_create(), stake).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_stake_rejects_invalid_did() {
        let mut stake = valid_stake();
        stake.staker_did = "not-a-did".into();
        let result = validate_create_stake(make_create(), stake).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- Stake update ----

    #[test]
    fn test_stake_update_rejects_nan() {
        let mut stake = valid_stake();
        stake.mycel_score = f32::NAN;
        let result = validate_update_stake(make_update(), stake).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_stake_update_rejects_infinity_weight() {
        let mut stake = valid_stake();
        stake.stake_weight = f32::INFINITY;
        let result = validate_update_stake(make_update(), stake).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }



fn validate_escrow_update_terms(
    original: &CryptoEscrow,
    escrow: &CryptoEscrow,
) -> ValidateCallbackResult {
    if original.id != escrow.id {
        return ValidateCallbackResult::Invalid("Escrow ID is immutable across updates".into());
    }
    if original.depositor_did != escrow.depositor_did {
        return ValidateCallbackResult::Invalid(
            "Escrow depositor_did is immutable across updates".into(),
        );
    }
    if original.beneficiary_did != escrow.beneficiary_did {
        return ValidateCallbackResult::Invalid(
            "Escrow beneficiary_did is immutable across updates".into(),
        );
    }
    if original.sap_amount != escrow.sap_amount {
        return ValidateCallbackResult::Invalid("Escrow sap_amount is immutable across updates".into());
    }
    if original.purpose != escrow.purpose {
        return ValidateCallbackResult::Invalid("Escrow purpose is immutable across updates".into());
    }
    if original.conditions != escrow.conditions {
        return ValidateCallbackResult::Invalid("Escrow conditions are immutable across updates".into());
    }
    if original.required_conditions != escrow.required_conditions {
        return ValidateCallbackResult::Invalid(
            "Escrow required_conditions are immutable across updates".into(),
        );
    }
    if original.hash_lock != escrow.hash_lock {
        return ValidateCallbackResult::Invalid("Escrow hash_lock is immutable across updates".into());
    }
    if original.timelock != escrow.timelock {
        return ValidateCallbackResult::Invalid("Escrow timelock is immutable across updates".into());
    }
    if original.multisig_threshold != escrow.multisig_threshold {
        return ValidateCallbackResult::Invalid(
            "Escrow multisig_threshold is immutable across updates".into(),
        );
    }
    if original.multisig_signers != escrow.multisig_signers {
        return ValidateCallbackResult::Invalid(
            "Escrow multisig_signers are immutable across updates".into(),
        );
    }
    if original.created_at != escrow.created_at {
        return ValidateCallbackResult::Invalid("Escrow created_at is immutable across updates".into());
    }
    if original.status != escrow.status && !original.status.can_transition_to(&escrow.status) {
        return ValidateCallbackResult::Invalid(format!(
            "Invalid escrow status transition: {:?} → {:?}",
            original.status, escrow.status
        ));
    }
    if !matches!(escrow.status, EscrowStatus::Released) && escrow.released_at.is_some() {
        return ValidateCallbackResult::Invalid(
            "released_at may only be set when escrow is Released".into(),
        );
    }
    if original.status == EscrowStatus::Released && escrow.status != EscrowStatus::Released {
        return ValidateCallbackResult::Invalid("Released escrow is terminal and cannot transition further".into());
    }
    if original.status == EscrowStatus::Refunded && escrow.status != EscrowStatus::Refunded {
        return ValidateCallbackResult::Invalid("Refunded escrow is terminal and cannot transition further".into());
    }
    if original.status == EscrowStatus::Expired && escrow.status != EscrowStatus::Expired {
        return ValidateCallbackResult::Invalid("Expired escrow is terminal and cannot transition further".into());
    }
    if original.collected_signatures != escrow.collected_signatures {
        return ValidateCallbackResult::Invalid(
            "Escrow collected_signatures must be preserved; use EscrowSignatureEntry".into(),
        );
    }
    ValidateCallbackResult::Valid
}

    // ---- Escrow: release condition float validation ----

    fn valid_escrow() -> CryptoEscrow {
        CryptoEscrow {
            id: "escrow:test:001".into(),
            depositor_did: "did:mycelix:alice".into(),
            beneficiary_did: "did:mycelix:bob".into(),
            sap_amount: 500,
            purpose: "Test escrow".into(),
            conditions: vec![ReleaseCondition::Timelock {
                release_time: 2_000_000,
            }],
            required_conditions: 1,
            met_conditions: vec![],
            hash_lock: None,
            timelock: None,
            multisig_threshold: None,
            multisig_signers: vec![],
            collected_signatures: vec![],
            status: EscrowStatus::Pending,
            created_at: ts(1_000_000),
            released_at: None,
        }
    }



    #[test]
    fn escrow_update_preserves_identity_and_terms() {
        let original = valid_escrow();
        let mut updated = original.clone();
        updated.status = EscrowStatus::Releasable;
        updated.met_conditions = vec![0];

        let result = validate_escrow_update_terms(&original, &updated);
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn escrow_update_rejects_identity_and_term_mutation() {
        let original = valid_escrow();
        let mut cases = Vec::new();

        let mut changed_id = original.clone();
        changed_id.id.push_str("-mutated");
        cases.push(changed_id);

        let mut changed_beneficiary = original.clone();
        changed_beneficiary.beneficiary_did = "did:mycelix:mallory".into();
        cases.push(changed_beneficiary);

        let mut changed_amount = original.clone();
        changed_amount.sap_amount += 1;
        cases.push(changed_amount);

        let mut changed_purpose = original.clone();
        changed_purpose.purpose.push_str(" mutated");
        cases.push(changed_purpose);

        for updated in cases {
            let result = validate_escrow_update_terms(&original, &updated);
            assert!(
                matches!(result, ValidateCallbackResult::Invalid(_)),
                "escrow identity/terms must be immutable"
            );
        }
    }

    #[test]
    fn test_escrow_create_valid() {
        let result = validate_escrow(make_create(), valid_escrow()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_escrow_rejects_nan_oracle_price() {
        let mut escrow = valid_escrow();
        escrow.conditions = vec![ReleaseCondition::OraclePrice {
            oracle_id: "oracle:test".into(),
            asset: "ETH".into(),
            min_price: f64::NAN,
        }];
        let result = validate_escrow(make_create(), escrow).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_escrow_rejects_infinity_oracle_price() {
        let mut escrow = valid_escrow();
        escrow.conditions = vec![ReleaseCondition::OraclePrice {
            oracle_id: "oracle:test".into(),
            asset: "ETH".into(),
            min_price: f64::INFINITY,
        }];
        let result = validate_escrow(make_create(), escrow).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_escrow_rejects_nan_mycel_threshold() {
        let mut escrow = valid_escrow();
        escrow.conditions = vec![ReleaseCondition::MycelThreshold {
            min_mycel_score: f32::NAN,
        }];
        let result = validate_escrow(make_create(), escrow).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_escrow_rejects_mycel_threshold_out_of_range() {
        let mut escrow = valid_escrow();
        escrow.conditions = vec![ReleaseCondition::MycelThreshold {
            min_mycel_score: 1.5, // > 1.0
        }];
        let result = validate_escrow(make_create(), escrow).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_escrow_rejects_zero_sap() {
        let mut escrow = valid_escrow();
        escrow.sap_amount = 0;
        let result = validate_escrow(make_create(), escrow).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_escrow_rejects_no_conditions() {
        let mut escrow = valid_escrow();
        escrow.conditions = vec![];
        let result = validate_escrow(make_create(), escrow).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- Slashing event ----

    #[test]
    fn test_slashing_rejects_over_100_percent() {
        let event = SlashingEvent {
            id: "slash:test".into(),
            stake_id: "stake:test".into(),
            staker_did: "did:mycelix:alice".into(),
            reason: SlashingReason::Downtime,
            slash_percentage: 101,
            sap_slashed: 100,
            evidence_hash: vec![0; 32],
            evidence: vec![1, 2, 3],
            slashed_at: ts(1_000_000),
            jailed: false,
            jail_release: None,
        };
        let result = validate_slashing_event(make_create(), event).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_slashing_rejects_empty_evidence() {
        let event = SlashingEvent {
            id: "slash:test".into(),
            stake_id: "stake:test".into(),
            staker_did: "did:mycelix:alice".into(),
            reason: SlashingReason::DoubleSigning,
            slash_percentage: 100,
            sap_slashed: 1000,
            evidence_hash: vec![0; 32],
            evidence: vec![],
            slashed_at: ts(1_000_000),
            jailed: true,
            jail_release: None,
        };
        let result = validate_slashing_event(make_create(), event).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // =========================================================================
    // Status transition state machines
    // =========================================================================

    #[test]
    fn test_stake_status_valid_transitions() {
        use StakeStatus::*;
        assert!(Active.can_transition_to(&Unbonding));
        assert!(Active.can_transition_to(&Slashed));
        assert!(Active.can_transition_to(&Jailed));
        assert!(Unbonding.can_transition_to(&Withdrawn));
        assert!(Slashed.can_transition_to(&Unbonding));
        assert!(Slashed.can_transition_to(&Jailed));
    }

    #[test]
    fn test_stake_status_invalid_transitions() {
        use StakeStatus::*;
        // Terminal states
        assert!(!Withdrawn.can_transition_to(&Active));
        assert!(!Withdrawn.can_transition_to(&Unbonding));
        assert!(!Jailed.can_transition_to(&Active));
        assert!(!Jailed.can_transition_to(&Slashed));
        // Cannot go backwards
        assert!(!Unbonding.can_transition_to(&Active));
        assert!(!Slashed.can_transition_to(&Active));
    }

    #[test]
    fn test_escrow_status_valid_transitions() {
        use EscrowStatus::*;
        assert!(Pending.can_transition_to(&Releasable));
        assert!(Pending.can_transition_to(&Refunded));
        assert!(Pending.can_transition_to(&Disputed));
        assert!(Pending.can_transition_to(&Expired));
        assert!(Releasable.can_transition_to(&Released));
        assert!(Releasable.can_transition_to(&Disputed));
        assert!(Disputed.can_transition_to(&Released));
        assert!(Disputed.can_transition_to(&Refunded));
    }

    #[test]
    fn test_escrow_status_invalid_transitions() {
        use EscrowStatus::*;
        // Terminal states
        assert!(!Released.can_transition_to(&Pending));
        assert!(!Released.can_transition_to(&Refunded));
        assert!(!Refunded.can_transition_to(&Pending));
        assert!(!Refunded.can_transition_to(&Released));
        assert!(!Expired.can_transition_to(&Pending));
        assert!(!Expired.can_transition_to(&Released));
    }
}

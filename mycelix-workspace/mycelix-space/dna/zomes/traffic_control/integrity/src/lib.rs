// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Traffic Control Integrity Zome
//!
//! Implements automated space traffic coordination through
//! AI-mediated negotiation between operators. When a conjunction
//! is detected, the involved operators' AI agents negotiate
//! who should maneuver and how.
//!
//! # The Negotiation Protocol
//!
//! 1. **Conjunction Detected**: System identifies close approach
//! 2. **Negotiation Initiated**: Both operators receive notification
//! 3. **Position Exchange**: Each side shares their constraints/preferences
//! 4. **Proposal Generation**: AI generates maneuver options
//! 5. **Agreement**: Both sides sign off on solution
//! 6. **Execution**: Chosen operator executes maneuver
//! 7. **Confirmation**: Network verifies new orbits are safe

use hdi::prelude::*;
use mycelix_space_shared::SpaceTimestamp;

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    /// Negotiation session for a conjunction
    NegotiationSession(NegotiationSession),

    /// Operator's position in negotiation
    NegotiationPosition(NegotiationPosition),

    /// Proposed solution
    ManeuverProposal(ManeuverProposal),

    /// Agreement on a solution
    NegotiationAgreement(NegotiationAgreement),

    /// Lambert-computed avoidance maneuver option
    AvoidanceOption(AvoidanceOption),

    // --- Multi-party negotiation (N-operator) ---
    /// Multi-party conjunction proposal — extends bilateral to N operators
    ConjunctionProposal(ConjunctionProposal),

    /// Operator vote on a conjunction proposal
    OperatorVote(OperatorVote),

    /// Multi-party agreement cosigned by quorum of operators
    MultiPartyAgreement(MultiPartyAgreement),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Sessions for a conjunction
    ConjunctionSessions,
    /// Positions in a session
    SessionPositions,
    /// Proposals in a session
    SessionProposals,
    /// Active sessions for an operator
    OperatorSessions,
    /// Avoidance options for a session
    SessionAvoidanceOptions,
    /// Bilateral agreements for a session (uniqueness constraint)
    SessionAgreements,

    // --- Multi-party negotiation links ---
    /// Conjunction proposal → operator votes
    ProposalToVotes,
    /// Conjunction ID → conjunction proposals
    ConjunctionProposals,
    /// Conjunction proposal → multi-party agreement
    ProposalAgreements,
    /// Operator → conjunction proposals they are part of
    OperatorProposals,
}

/// A negotiation session between operators
#[hdk_entry_helper]
#[derive(Clone)]
pub struct NegotiationSession {
    /// Unique session ID
    pub session_id: String,

    /// Related conjunction event ID
    pub conjunction_id: String,

    /// Primary object operator
    pub primary_operator: AgentPubKey,

    /// Secondary object operator
    pub secondary_operator: AgentPubKey,

    /// Primary object NORAD ID
    pub primary_norad_id: u32,

    /// Secondary object NORAD ID
    pub secondary_norad_id: u32,

    /// TCA for this conjunction
    pub tca: SpaceTimestamp,

    /// Session status
    pub status: SessionStatus,

    /// Deadline for reaching agreement
    pub deadline: SpaceTimestamp,

    /// Created at
    pub created_at: SpaceTimestamp,
}

/// Session status
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum SessionStatus {
    /// Waiting for both parties to join
    Pending,
    /// Both parties engaged, exchanging positions
    Active,
    /// Proposals being evaluated
    Proposing,
    /// Agreement reached
    Agreed,
    /// Maneuver executed
    Executed,
    /// Failed to reach agreement
    Failed,
    /// Timed out
    Expired,
}

/// Operator's position/constraints for negotiation
#[hdk_entry_helper]
#[derive(Clone)]
pub struct NegotiationPosition {
    /// Session this position is for
    pub session_id: String,

    /// Operator submitting position
    pub operator: AgentPubKey,

    /// Object they're representing
    pub norad_id: u32,

    /// Maneuver capability
    pub maneuver_capability: ManeuverCapability,

    /// Preferences
    pub preferences: OperatorPreferences,

    /// Submitted at
    pub submitted_at: SpaceTimestamp,
}

/// Operator's maneuver capabilities
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ManeuverCapability {
    /// Can this object maneuver?
    pub can_maneuver: bool,

    /// Maximum delta-V available (m/s)
    pub max_delta_v_ms: Option<f64>,

    /// Minimum lead time for maneuver (hours)
    pub min_lead_time_hours: Option<f64>,

    /// Fuel status (0-100%)
    pub fuel_percentage: Option<f64>,

    /// Other constraints
    pub constraints: Vec<String>,
}

/// Operator preferences for resolution
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperatorPreferences {
    /// Willingness to maneuver (0-100)
    pub willingness_to_maneuver: u8,

    /// Acceptable risk levels
    pub acceptable_risk: AcceptableRisk,

    /// Preferred maneuver timing
    pub preferred_timing: Option<SpaceTimestamp>,

    /// Maximum acceptable collision probability
    pub max_acceptable_pc: f64,
}

/// Acceptable risk threshold
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum AcceptableRisk {
    /// Conservative (Pc < 1e-6)
    Conservative,
    /// Standard (Pc < 1e-5)
    Standard,
    /// Relaxed (Pc < 1e-4)
    Relaxed,
}

/// A proposed maneuver solution
#[hdk_entry_helper]
#[derive(Clone)]
pub struct ManeuverProposal {
    /// Session this proposal is for
    pub session_id: String,

    /// Who generated this proposal
    pub proposer: AgentPubKey,

    /// Object that would maneuver
    pub maneuvering_object: u32,

    /// Proposed burn time
    pub burn_time: SpaceTimestamp,

    /// Proposed delta-V (m/s)
    pub delta_v_ms: f64,

    /// Direction (unit vector)
    pub direction: [f64; 3],

    /// Resulting miss distance (km)
    pub resulting_miss_km: f64,

    /// Resulting collision probability
    pub resulting_pc: f64,

    /// Cost estimate
    pub cost_estimate: Option<CostEstimate>,

    /// Proposal status
    pub status: ProposalStatus,

    /// Created at
    pub created_at: SpaceTimestamp,
}

/// Cost estimate for a maneuver
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CostEstimate {
    /// Delta-V cost
    pub delta_v_ms: f64,

    /// Mission impact (days of lifetime lost, etc.)
    pub mission_impact: String,

    /// Monetary cost estimate (optional)
    pub monetary_cost: Option<u64>,

    /// Currency
    pub currency: Option<String>,
}

/// Proposal status
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum ProposalStatus {
    /// Pending review
    Pending,
    /// Accepted by one party
    PartiallyAccepted,
    /// Accepted by all parties
    Accepted,
    /// Rejected
    Rejected,
    /// Superseded by better proposal
    Superseded,
}

/// A Lambert-computed avoidance maneuver option for a conjunction.
///
/// Generated by `generate_avoidance_options()` — each option represents a
/// different direction (prograde, retrograde, radial, normal) with the ΔV
/// required and the resulting post-maneuver miss distance and Pc.
#[hdk_entry_helper]
#[derive(Clone)]
pub struct AvoidanceOption {
    /// Session this option was generated for
    pub session_id: String,

    /// When the maneuver burn would occur
    pub maneuver_time: SpaceTimestamp,

    /// Delta-V vector [dvx, dvy, dvz] in km/s (TEME)
    pub delta_v: Vec<f64>,

    /// Total ΔV magnitude in m/s
    pub delta_v_magnitude_ms: f64,

    /// Resulting miss distance after maneuver (km)
    pub resulting_miss_distance_km: f64,

    /// Resulting collision probability after maneuver
    pub resulting_pc: f64,

    /// Transfer type: "prograde", "retrograde", "radial", or "normal"
    pub transfer_type: String,
}

/// Agreement between operators
#[hdk_entry_helper]
#[derive(Clone)]
pub struct NegotiationAgreement {
    /// Session this agreement is for
    pub session_id: String,

    /// Accepted proposal
    pub accepted_proposal: ActionHash,

    /// Primary operator signature
    pub primary_signature: Option<AgentPubKey>,

    /// Secondary operator signature
    pub secondary_signature: Option<AgentPubKey>,

    /// Agreement time
    pub agreed_at: SpaceTimestamp,

    /// Execution deadline
    pub execution_deadline: SpaceTimestamp,
}

// =============================================================================
// Multi-party negotiation entry types (N-operator coordination)
// =============================================================================

/// Multi-party conjunction proposal — extends bilateral to N operators.
///
/// When a conjunction involves objects from more than two operators (e.g.,
/// mega-constellation scenarios), this entry tracks the multi-party
/// negotiation with pre-computed maneuver options and weighted voting.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ConjunctionProposal {
    /// Unique proposal identifier
    pub proposal_id: String,
    /// Related conjunction event ID
    pub conjunction_id: String,
    /// All operators affected by this conjunction
    pub affected_operators: Vec<AgentPubKey>,
    /// Time of closest approach (microseconds since epoch)
    pub tca: i64,
    /// Primary object NORAD ID
    pub primary_norad_id: u32,
    /// Secondary objects involved in the conjunction
    pub secondary_norad_ids: Vec<u32>,
    /// Pre-computed avoidance maneuver options (from Lambert solver)
    pub maneuver_options: Vec<ManeuverOption>,
    /// Deadline for voting (microseconds since epoch)
    pub voting_deadline: i64,
    /// Current proposal status
    pub status: MultiPartyProposalStatus,
    /// Fraction of operators needed for quorum (0.5-1.0)
    pub quorum_threshold: f64,
    /// Created at (microseconds since epoch)
    pub created_at: i64,
    /// Agent who created this proposal
    pub created_by: AgentPubKey,
}

/// Status of a multi-party conjunction proposal.
///
/// Named `MultiPartyProposalStatus` to avoid collision with the existing
/// bilateral `ProposalStatus` enum.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum MultiPartyProposalStatus {
    /// Accepting votes from affected operators
    Voting,
    /// Quorum reached, winning option approved
    Approved,
    /// Quorum not reached or majority rejected
    Rejected,
    /// Voting deadline passed without quorum
    Expired,
    /// Approved maneuver is being executed
    Executing,
    /// Maneuver executed and verified
    Completed,
}

impl MultiPartyProposalStatus {
    /// Valid status transitions for the proposal state machine.
    pub fn is_valid_transition(&self, next: &Self) -> bool {
        matches!(
            (self, next),
            (Self::Voting, Self::Approved)
                | (Self::Voting, Self::Rejected)
                | (Self::Voting, Self::Expired)
                | (Self::Approved, Self::Executing)
                | (Self::Approved, Self::Expired)
                | (Self::Executing, Self::Completed)
        )
    }
}

/// A pre-computed avoidance maneuver option for multi-party proposals.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ManeuverOption {
    /// Index of this option (for voting reference)
    pub option_index: u32,
    /// Human-readable description
    pub description: String,
    /// NORAD ID of the object that would maneuver
    pub maneuvering_norad_id: u32,
    /// Required delta-V in m/s
    pub delta_v_ms: f64,
    /// Maneuver direction (unit vector)
    pub direction: [f64; 3],
    /// Post-maneuver miss distance in km
    pub post_maneuver_miss_km: f64,
    /// Post-maneuver collision probability
    pub post_maneuver_pc: f64,
}

/// Operator vote on a conjunction proposal.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct OperatorVote {
    /// Proposal this vote is for
    pub proposal_id: String,
    /// Voting operator
    pub voter: AgentPubKey,
    /// Index of preferred maneuver option
    pub preferred_option_index: u32,
    /// Operator's trust/consciousness weight (0.0-1.0)
    pub vote_weight: f64,
    /// Optional justification for the vote
    pub justification: Option<String>,
    /// Voted at (microseconds since epoch)
    pub voted_at: i64,
}

/// Multi-party agreement cosigned by quorum of operators.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MultiPartyAgreement {
    /// Proposal this agreement resolves
    pub proposal_id: String,
    /// Related conjunction event ID
    pub conjunction_id: String,
    /// Index of the approved maneuver option
    pub approved_option_index: u32,
    /// Operators who voted for the winning option
    pub approving_operators: Vec<AgentPubKey>,
    /// Sum of approving operators' vote weights
    pub total_vote_weight: f64,
    /// Whether quorum was met
    pub quorum_met: bool,
    /// Deadline for executing the maneuver (microseconds since epoch)
    pub execution_deadline: i64,
    /// Created at (microseconds since epoch)
    pub created_at: i64,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, action }) => match app_entry {
            EntryTypes::NegotiationSession(session) => {
                validate_create_session(EntryCreationAction::Create(action), session)
            }
            EntryTypes::NegotiationPosition(pos) => {
                validate_create_position(EntryCreationAction::Create(action), pos)
            }
            EntryTypes::ManeuverProposal(prop) => {
                validate_create_proposal(EntryCreationAction::Create(action), prop)
            }
            EntryTypes::NegotiationAgreement(agr) => {
                validate_create_agreement(EntryCreationAction::Create(action), agr)
            }
            EntryTypes::AvoidanceOption(opt) => validate_avoidance_option(&opt),
            EntryTypes::ConjunctionProposal(cp) => {
                validate_create_conjunction_proposal(EntryCreationAction::Create(action), cp)
            }
            EntryTypes::OperatorVote(vote) => {
                validate_create_operator_vote(EntryCreationAction::Create(action), vote)
            }
            EntryTypes::MultiPartyAgreement(mpa) => validate_multi_party_agreement(&mpa),
        },
        // Most entry types here are create-only in practice (no coordinator function ever
        // calls update_entry on them) -- reject outright rather than leave an unbound
        // dead-code path (P0 wide-open RegisterUpdate gap, confirmed dozens of times
        // elsewhere in this pass). NegotiationAgreement (cosign_agreement) and
        // ConjunctionProposal (tally_proposal) have real update paths and get their own
        // validators below -- see those functions' doc comments for why neither uses the
        // simple author-binding pattern.
        FlatOp::StoreEntry(OpEntry::UpdateEntry {
            app_entry, action, ..
        }) => match app_entry {
            EntryTypes::NegotiationSession(_) => Ok(ValidateCallbackResult::Invalid(
                "NegotiationSession entries cannot be updated".to_string(),
            )),
            EntryTypes::NegotiationPosition(_) => Ok(ValidateCallbackResult::Invalid(
                "NegotiationPosition entries cannot be updated".to_string(),
            )),
            EntryTypes::ManeuverProposal(_) => Ok(ValidateCallbackResult::Invalid(
                "ManeuverProposal entries cannot be updated".to_string(),
            )),
            EntryTypes::NegotiationAgreement(agr) => validate_update_agreement(action, agr),
            EntryTypes::AvoidanceOption(_) => Ok(ValidateCallbackResult::Invalid(
                "AvoidanceOption entries cannot be updated".to_string(),
            )),
            EntryTypes::ConjunctionProposal(cp) => validate_update_conjunction_proposal(action, cp),
            EntryTypes::OperatorVote(_) => Ok(ValidateCallbackResult::Invalid(
                "OperatorVote entries cannot be updated".to_string(),
            )),
            EntryTypes::MultiPartyAgreement(_) => Ok(ValidateCallbackResult::Invalid(
                "MultiPartyAgreement entries cannot be updated".to_string(),
            )),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_session(
    action: EntryCreationAction,
    session: NegotiationSession,
) -> ExternResult<ValidateCallbackResult> {
    // Session ID must not be empty
    if session.session_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Session ID cannot be empty".to_string(),
        ));
    }

    // Conjunction ID must not be empty
    if session.conjunction_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Conjunction ID cannot be empty".to_string(),
        ));
    }

    // Both NORAD IDs must be valid
    if session.primary_norad_id == 0 || session.primary_norad_id > 999999 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid primary NORAD ID: {}",
            session.primary_norad_id
        )));
    }
    if session.secondary_norad_id == 0 || session.secondary_norad_id > 999999 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid secondary NORAD ID: {}",
            session.secondary_norad_id
        )));
    }

    // Objects must be different
    if session.primary_norad_id == session.secondary_norad_id {
        return Ok(ValidateCallbackResult::Invalid(
            "Primary and secondary NORAD IDs must be different".to_string(),
        ));
    }

    // Operators must be different
    if session.primary_operator == session.secondary_operator {
        return Ok(ValidateCallbackResult::Invalid(
            "Primary and secondary operators must be different".to_string(),
        ));
    }

    // Deadline must be after TCA (negotiation window)
    if session.deadline.micros < session.tca.micros {
        return Ok(ValidateCallbackResult::Invalid(
            "Deadline cannot be before TCA".to_string(),
        ));
    }

    // Bind the session to its committer -- initiate_negotiation takes BOTH
    // primary_operator and secondary_operator as raw caller input with no
    // agent_info() derivation, so unlike most other zomes in this pass this
    // isn't a single self-reported field to bind: require the committing
    // agent be one of the two named parties, not an arbitrary third agent
    // naming two victims (P0 author-binding gap).
    if *action.author() != session.primary_operator
        && *action.author() != session.secondary_operator
    {
        return Ok(ValidateCallbackResult::Invalid(
            "NegotiationSession must be initiated by one of its named operators (impersonation)"
                .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_position(
    action: EntryCreationAction,
    pos: NegotiationPosition,
) -> ExternResult<ValidateCallbackResult> {
    // Session ID must not be empty
    if pos.session_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Session ID cannot be empty".to_string(),
        ));
    }

    // NORAD ID must be valid
    if pos.norad_id == 0 || pos.norad_id > 999999 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid NORAD ID: {}",
            pos.norad_id
        )));
    }

    // Validate maneuver capability fields
    if let Some(max_dv) = pos.maneuver_capability.max_delta_v_ms {
        if !max_dv.is_finite() || max_dv < 0.0 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "max_delta_v_ms must be non-negative and finite, got {}",
                max_dv
            )));
        }
    }

    if let Some(lead_time) = pos.maneuver_capability.min_lead_time_hours {
        if !lead_time.is_finite() || lead_time < 0.0 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "min_lead_time_hours must be non-negative and finite, got {}",
                lead_time
            )));
        }
    }

    if let Some(fuel) = pos.maneuver_capability.fuel_percentage {
        if !fuel.is_finite() || !(0.0..=100.0).contains(&fuel) {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "fuel_percentage must be between 0 and 100, got {}",
                fuel
            )));
        }
    }

    // Validate preferences
    if pos.preferences.willingness_to_maneuver > 100 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "willingness_to_maneuver must be 0-100, got {}",
            pos.preferences.willingness_to_maneuver
        )));
    }

    if !pos.preferences.max_acceptable_pc.is_finite()
        || pos.preferences.max_acceptable_pc < 0.0
        || pos.preferences.max_acceptable_pc > 1.0
    {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "max_acceptable_pc must be between 0 and 1, got {}",
            pos.preferences.max_acceptable_pc
        )));
    }

    // Bind the position to its committer -- submit_position already derives
    // operator from agent_info() coordinator-side with zero user input (P0
    // author-binding gap).
    if pos.operator != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "NegotiationPosition must be submitted by the committing agent (operator forgery)"
                .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_proposal(
    action: EntryCreationAction,
    prop: ManeuverProposal,
) -> ExternResult<ValidateCallbackResult> {
    // Session ID must not be empty
    if prop.session_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Session ID cannot be empty".to_string(),
        ));
    }

    // NORAD ID must be valid
    if prop.maneuvering_object == 0 || prop.maneuvering_object > 999999 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid maneuvering object NORAD ID: {}",
            prop.maneuvering_object
        )));
    }

    // Delta-V must be positive and finite
    if !prop.delta_v_ms.is_finite() || prop.delta_v_ms <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Delta-V must be a positive finite number, got {}",
            prop.delta_v_ms
        )));
    }

    // Direction must be finite and a unit vector
    for &component in &prop.direction {
        if !component.is_finite() {
            return Ok(ValidateCallbackResult::Invalid(
                "Direction components must be finite numbers".to_string(),
            ));
        }
    }
    let mag_sq: f64 = prop.direction.iter().map(|x| x * x).sum();
    if (mag_sq - 1.0).abs() > 0.01 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Direction must be a unit vector (magnitude squared: {})",
            mag_sq
        )));
    }

    // Resulting miss distance must be non-negative and finite
    if !prop.resulting_miss_km.is_finite() || prop.resulting_miss_km < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Resulting miss distance must be non-negative and finite, got {}",
            prop.resulting_miss_km
        )));
    }

    // Resulting Pc must be in [0, 1]
    if !prop.resulting_pc.is_finite() || prop.resulting_pc < 0.0 || prop.resulting_pc > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Resulting Pc must be between 0 and 1, got {}",
            prop.resulting_pc
        )));
    }

    // Validate cost estimate if present
    if let Some(ref cost) = prop.cost_estimate {
        if !cost.delta_v_ms.is_finite() || cost.delta_v_ms < 0.0 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Cost estimate delta_v_ms must be non-negative and finite, got {}",
                cost.delta_v_ms
            )));
        }
    }

    // Bind the proposal to its committer -- submit_proposal already derives
    // proposer from agent_info() coordinator-side with zero user input (P0
    // author-binding gap).
    if prop.proposer != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "ManeuverProposal must be proposed by the committing agent (proposer forgery)"
                .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Structural checks shared by create and update -- no author-binding here, each caller
/// applies its own binding rule (create: primary_signature == committer; update: see
/// `validate_update_agreement`'s doc comment for why a simple bind doesn't fit there).
fn validate_agreement_content(agr: &NegotiationAgreement) -> Option<ValidateCallbackResult> {
    if agr.session_id.trim().is_empty() {
        return Some(ValidateCallbackResult::Invalid(
            "Session ID cannot be empty".to_string(),
        ));
    }

    // At least the primary signature must be present (creator signs first)
    if agr.primary_signature.is_none() {
        return Some(ValidateCallbackResult::Invalid(
            "Agreement must have at least a primary signature".to_string(),
        ));
    }

    // If both signatures are present, they must be different
    if let (Some(primary), Some(secondary)) = (&agr.primary_signature, &agr.secondary_signature) {
        if primary == secondary {
            return Some(ValidateCallbackResult::Invalid(
                "Primary and secondary signatures must be from different agents".to_string(),
            ));
        }
    }

    // Execution deadline must be after agreement time
    if agr.execution_deadline.micros < agr.agreed_at.micros {
        return Some(ValidateCallbackResult::Invalid(
            "Execution deadline cannot be before agreement time".to_string(),
        ));
    }

    None
}

fn validate_create_agreement(
    action: EntryCreationAction,
    agr: NegotiationAgreement,
) -> ExternResult<ValidateCallbackResult> {
    if let Some(invalid) = validate_agreement_content(&agr) {
        return Ok(invalid);
    }

    // Bind the agreement to its committer -- accept_proposal already derives
    // primary_signature: Some(agent_info()) coordinator-side with zero user
    // input, so this never rejects a legitimate accept; it's the real
    // DHT-level enforcement a modified coordinator could otherwise bypass
    // (P0 author-binding gap).
    if agr.primary_signature != Some(action.author().clone()) {
        return Ok(ValidateCallbackResult::Invalid(
            "NegotiationAgreement's primary_signature must be the committing agent \
             (signature forgery)"
                .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Re-derives `cosign_agreement`'s own exact intended invariant at the DHT level (mirrors the
/// `debris_bounties` precedent from the prior mycelix-space pass: re-derive a specific
/// coordinator-disclosed rule, not a blanket one). `cosign_agreement` is the ONLY coordinator
/// function that updates this entry, and it only ever does one thing: set
/// `secondary_signature` from `agent_info()`, after checking (coordinator-side only, until
/// now) that it isn't already set and isn't the same as `primary_signature`. This closes both
/// the wide-open-update bug (every field was previously unvalidated on update) and the
/// identity-forgery gap (the committing agent must actually BE the new secondary_signature) in
/// one check -- every other field must be byte-identical to the pre-update entry.
fn validate_update_agreement(
    action: Update,
    agr: NegotiationAgreement,
) -> ExternResult<ValidateCallbackResult> {
    if let Some(invalid) = validate_agreement_content(&agr) {
        return Ok(invalid);
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: NegotiationAgreement = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid original NegotiationAgreement entry".to_string()
        )))?;

    if agr.session_id != original.session_id
        || agr.accepted_proposal != original.accepted_proposal
        || agr.primary_signature != original.primary_signature
        || agr.agreed_at.micros != original.agreed_at.micros
        || agr.execution_deadline.micros != original.execution_deadline.micros
    {
        return Ok(ValidateCallbackResult::Invalid(
            "NegotiationAgreement updates may only set secondary_signature -- all other \
             fields must be unchanged"
                .to_string(),
        ));
    }

    if original.secondary_signature.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "NegotiationAgreement already has both signatures".to_string(),
        ));
    }

    if agr.secondary_signature != Some(action.author.clone()) {
        return Ok(ValidateCallbackResult::Invalid(
            "NegotiationAgreement's secondary_signature must be set to the committing agent \
             (cosignature forgery)"
                .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_avoidance_option(opt: &AvoidanceOption) -> ExternResult<ValidateCallbackResult> {
    if opt.session_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Session ID cannot be empty".to_string(),
        ));
    }

    // Delta-V vector must have exactly 3 elements
    if opt.delta_v.len() != 3 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "delta_v must have 3 elements, got {}",
            opt.delta_v.len()
        )));
    }

    // All delta-V elements must be finite
    for (i, &v) in opt.delta_v.iter().enumerate() {
        if !v.is_finite() {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "delta_v[{}] is not finite: {}",
                i, v
            )));
        }
    }

    // Magnitude must be positive and finite
    if !opt.delta_v_magnitude_ms.is_finite() || opt.delta_v_magnitude_ms <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "delta_v_magnitude_ms must be positive and finite, got {}",
            opt.delta_v_magnitude_ms
        )));
    }

    // Miss distance must be non-negative and finite
    if !opt.resulting_miss_distance_km.is_finite() || opt.resulting_miss_distance_km < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "resulting_miss_distance_km must be non-negative and finite, got {}",
            opt.resulting_miss_distance_km
        )));
    }

    // Pc must be in [0, 1]
    if !opt.resulting_pc.is_finite() || opt.resulting_pc < 0.0 || opt.resulting_pc > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "resulting_pc must be between 0 and 1, got {}",
            opt.resulting_pc
        )));
    }

    // Transfer type must be one of the valid types
    let valid_types = ["prograde", "retrograde", "radial", "normal"];
    if !valid_types.contains(&opt.transfer_type.as_str()) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "transfer_type must be one of {:?}, got '{}'",
            valid_types, opt.transfer_type
        )));
    }

    Ok(ValidateCallbackResult::Valid)
}

// =============================================================================
// Multi-party negotiation validation
// =============================================================================

/// Structural checks shared by create and update, unchanged from before this pass -- see
/// `validate_create_conjunction_proposal`/`validate_update_conjunction_proposal` for the
/// author-binding (create) and transition-validation (update) rules layered on top.
fn validate_conjunction_proposal_content(
    cp: &ConjunctionProposal,
) -> ExternResult<ValidateCallbackResult> {
    // proposal_id must not be empty and <= 256 chars
    if cp.proposal_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Proposal ID cannot be empty".to_string(),
        ));
    }
    if cp.proposal_id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Proposal ID too long: {} chars (max 256)",
            cp.proposal_id.len()
        )));
    }

    // conjunction_id must not be empty
    if cp.conjunction_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Conjunction ID cannot be empty".to_string(),
        ));
    }
    if cp.conjunction_id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Conjunction ID too long: {} chars (max 256)",
            cp.conjunction_id.len()
        )));
    }

    // At least 2 affected operators
    if cp.affected_operators.len() < 2 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "At least 2 affected operators required, got {}",
            cp.affected_operators.len()
        )));
    }

    // Cap at 64 operators to bound DHT load
    if cp.affected_operators.len() > 64 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Too many affected operators: {} (max 64)",
            cp.affected_operators.len()
        )));
    }

    // Primary NORAD ID must be valid
    if cp.primary_norad_id == 0 || cp.primary_norad_id > 999999 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid primary NORAD ID: {}",
            cp.primary_norad_id
        )));
    }

    // At least 1 secondary NORAD ID
    if cp.secondary_norad_ids.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "At least one secondary NORAD ID required".to_string(),
        ));
    }

    // Validate all secondary NORAD IDs
    for &norad in &cp.secondary_norad_ids {
        if norad == 0 || norad > 999999 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Invalid secondary NORAD ID: {}",
                norad
            )));
        }
    }

    // Primary must not appear in secondary list
    if cp.secondary_norad_ids.contains(&cp.primary_norad_id) {
        return Ok(ValidateCallbackResult::Invalid(
            "Primary NORAD ID must not appear in secondary list".to_string(),
        ));
    }

    // At least 2 maneuver options
    if cp.maneuver_options.len() < 2 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "At least 2 maneuver options required, got {}",
            cp.maneuver_options.len()
        )));
    }

    // Cap maneuver options at 32
    if cp.maneuver_options.len() > 32 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Too many maneuver options: {} (max 32)",
            cp.maneuver_options.len()
        )));
    }

    // Validate each maneuver option
    for opt in &cp.maneuver_options {
        if opt.maneuvering_norad_id == 0 || opt.maneuvering_norad_id > 999999 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Invalid maneuvering NORAD ID in option {}: {}",
                opt.option_index, opt.maneuvering_norad_id
            )));
        }
        if !opt.delta_v_ms.is_finite() || opt.delta_v_ms < 0.0 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "delta_v_ms must be non-negative and finite in option {}, got {}",
                opt.option_index, opt.delta_v_ms
            )));
        }
        for &d in &opt.direction {
            if !d.is_finite() {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "Direction components must be finite in option {}",
                    opt.option_index
                )));
            }
        }
        if !opt.post_maneuver_miss_km.is_finite() || opt.post_maneuver_miss_km < 0.0 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "post_maneuver_miss_km must be non-negative and finite in option {}, got {}",
                opt.option_index, opt.post_maneuver_miss_km
            )));
        }
        if !opt.post_maneuver_pc.is_finite()
            || opt.post_maneuver_pc < 0.0
            || opt.post_maneuver_pc > 1.0
        {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "post_maneuver_pc must be between 0 and 1 in option {}, got {}",
                opt.option_index, opt.post_maneuver_pc
            )));
        }
        if opt.description.len() > 1024 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Description too long in option {}: {} chars (max 1024)",
                opt.option_index,
                opt.description.len()
            )));
        }
    }

    // quorum_threshold must be in [0.5, 1.0]
    if !cp.quorum_threshold.is_finite() || cp.quorum_threshold < 0.5 || cp.quorum_threshold > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "quorum_threshold must be between 0.5 and 1.0, got {}",
            cp.quorum_threshold
        )));
    }

    // voting_deadline must be after tca (can't vote after closest approach has passed)
    // Note: deadline > tca is fine — operators may vote after TCA if maneuver lead-time allows
    // We just require the deadline is plausible (non-zero)
    if cp.voting_deadline <= 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "voting_deadline must be a positive timestamp".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_conjunction_proposal(
    action: EntryCreationAction,
    cp: ConjunctionProposal,
) -> ExternResult<ValidateCallbackResult> {
    if let ok @ Ok(ValidateCallbackResult::Invalid(_)) = validate_conjunction_proposal_content(&cp)
    {
        return ok;
    }

    // Bind the proposal to its committer -- create_conjunction_proposal
    // already derives created_by from agent_info() coordinator-side with
    // zero user input (P0 author-binding gap). affected_operators is
    // deliberately left unbound: unlike created_by, it's ambiguous whether
    // the creator must be a member of that list -- an automated
    // conjunction-screening service plausibly needs to name operators it
    // isn't one of. Disclosed, not fixed; would need a real design
    // decision, not a mechanical bind.
    if cp.created_by != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "ConjunctionProposal must be created by the committing agent (creator forgery)"
                .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// `tally_proposal` is intentionally callable by ANY observer (it has no `gate_space_operation`
/// call at all) and only ever updates `status`, guarded by
/// `MultiPartyProposalStatus::is_valid_transition`'s state machine -- previously enforced only
/// coordinator-side. This is not an author-binding rule (there's no "correct committer" for a
/// tally anyone may compute) -- it re-applies the existing structural checks to the updated
/// entry and re-derives the transition-validity check at the DHT level, closing "a modified
/// coordinator could write anything on update" without restricting who may call it. Does NOT
/// verify the tally itself was honest (i.e. that `status` genuinely reflects the real vote
/// weights) -- that would need re-deriving the whole weighted quorum via `must_get` over every
/// `OperatorVote`, a substantially larger piece of work; disclosed, not fixed here.
fn validate_update_conjunction_proposal(
    action: Update,
    cp: ConjunctionProposal,
) -> ExternResult<ValidateCallbackResult> {
    if let ok @ Ok(ValidateCallbackResult::Invalid(_)) = validate_conjunction_proposal_content(&cp)
    {
        return ok;
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: ConjunctionProposal = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid original ConjunctionProposal entry".to_string()
        )))?;

    if !original.status.is_valid_transition(&cp.status) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid ConjunctionProposal status transition: {:?} -> {:?}",
            original.status, cp.status
        )));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_operator_vote(
    action: EntryCreationAction,
    vote: OperatorVote,
) -> ExternResult<ValidateCallbackResult> {
    // proposal_id must not be empty
    if vote.proposal_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Proposal ID cannot be empty".to_string(),
        ));
    }
    if vote.proposal_id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Proposal ID too long: {} chars (max 256)",
            vote.proposal_id.len()
        )));
    }

    // vote_weight must be in [0.0, 1.0]
    if !vote.vote_weight.is_finite() || vote.vote_weight < 0.0 || vote.vote_weight > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "vote_weight must be between 0.0 and 1.0, got {}",
            vote.vote_weight
        )));
    }

    // justification length cap
    if let Some(ref j) = vote.justification {
        if j.len() > 2048 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Justification too long: {} chars (max 2048)",
                j.len()
            )));
        }
    }

    // voted_at must be a positive timestamp
    if vote.voted_at <= 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "voted_at must be a positive timestamp".to_string(),
        ));
    }

    // Bind the vote to its committer -- vote_on_proposal already derives
    // voter from agent_info() coordinator-side with zero user input (P0
    // author-binding gap).
    if vote.voter != *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "OperatorVote must be cast by the committing agent (voter forgery)".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Deliberately no author-binding here, and left as-is (structural checks only) --
/// `approving_operators` is constructed server-side in `tally_proposal` by filtering already-
/// created (and, after this pass, genuinely author-bound) `OperatorVote` entries for the
/// winning option, so it's trustworthy *by construction* as long as `tally_proposal` is what
/// actually created it. But nothing here stops a modified coordinator from skipping
/// `tally_proposal` and calling `create_entry` directly with a fabricated
/// `approving_operators` list -- closing that would need this validator to independently
/// re-derive the entire weighted quorum tally via `must_get` chain-walking over every real
/// vote for the referenced proposal, a substantially larger, genuinely new piece of work, not
/// a variant of this pass's established author-binding pattern. Disclosed, not fixed.
fn validate_multi_party_agreement(
    mpa: &MultiPartyAgreement,
) -> ExternResult<ValidateCallbackResult> {
    // proposal_id must not be empty
    if mpa.proposal_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Proposal ID cannot be empty".to_string(),
        ));
    }

    // conjunction_id must not be empty
    if mpa.conjunction_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Conjunction ID cannot be empty".to_string(),
        ));
    }

    // Must have at least 2 approving operators
    if mpa.approving_operators.len() < 2 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "At least 2 approving operators required, got {}",
            mpa.approving_operators.len()
        )));
    }

    // total_vote_weight must be positive and finite
    if !mpa.total_vote_weight.is_finite() || mpa.total_vote_weight <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "total_vote_weight must be positive and finite, got {}",
            mpa.total_vote_weight
        )));
    }

    // execution_deadline must be positive
    if mpa.execution_deadline <= 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "execution_deadline must be a positive timestamp".to_string(),
        ));
    }

    // created_at must be positive
    if mpa.created_at <= 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "created_at must be a positive timestamp".to_string(),
        ));
    }

    // execution_deadline must be after created_at
    if mpa.execution_deadline < mpa.created_at {
        return Ok(ValidateCallbackResult::Invalid(
            "execution_deadline cannot be before created_at".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

// =============================================================================
// Unit tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_proposal_status_valid_transitions() {
        use MultiPartyProposalStatus::*;

        // Valid transitions
        assert!(Voting.is_valid_transition(&Approved));
        assert!(Voting.is_valid_transition(&Rejected));
        assert!(Voting.is_valid_transition(&Expired));
        assert!(Approved.is_valid_transition(&Executing));
        assert!(Approved.is_valid_transition(&Expired));
        assert!(Executing.is_valid_transition(&Completed));
    }

    #[test]
    fn test_proposal_status_invalid_transitions() {
        use MultiPartyProposalStatus::*;

        // Invalid transitions
        assert!(!Voting.is_valid_transition(&Completed));
        assert!(!Voting.is_valid_transition(&Executing));
        assert!(!Approved.is_valid_transition(&Rejected));
        assert!(!Approved.is_valid_transition(&Voting));
        assert!(!Rejected.is_valid_transition(&Approved));
        assert!(!Rejected.is_valid_transition(&Voting));
        assert!(!Expired.is_valid_transition(&Voting));
        assert!(!Expired.is_valid_transition(&Approved));
        assert!(!Completed.is_valid_transition(&Voting));
        assert!(!Completed.is_valid_transition(&Executing));
        assert!(!Executing.is_valid_transition(&Voting));
        assert!(!Executing.is_valid_transition(&Rejected));
    }

    #[test]
    fn test_proposal_status_terminal_states() {
        use MultiPartyProposalStatus::*;

        // Terminal states should not transition to anything
        let terminals = [Rejected, Expired, Completed];
        let all = [Voting, Approved, Rejected, Expired, Executing, Completed];
        for terminal in &terminals {
            for next in &all {
                assert!(
                    !terminal.is_valid_transition(next),
                    "{:?} -> {:?} should be invalid",
                    terminal,
                    next
                );
            }
        }
    }

    #[test]
    fn test_maneuver_option_direction_finite() {
        let opt = ManeuverOption {
            option_index: 0,
            description: "Prograde burn".to_string(),
            maneuvering_norad_id: 25544,
            delta_v_ms: 0.5,
            direction: [1.0, 0.0, 0.0],
            post_maneuver_miss_km: 10.0,
            post_maneuver_pc: 1e-6,
        };
        assert!(opt.direction.iter().all(|d| d.is_finite()));
    }

    #[test]
    fn test_maneuver_option_pc_bounds() {
        let opt = ManeuverOption {
            option_index: 0,
            description: "Test".to_string(),
            maneuvering_norad_id: 25544,
            delta_v_ms: 1.0,
            direction: [0.0, 1.0, 0.0],
            post_maneuver_miss_km: 5.0,
            post_maneuver_pc: 0.5,
        };
        assert!(opt.post_maneuver_pc >= 0.0 && opt.post_maneuver_pc <= 1.0);
    }
}

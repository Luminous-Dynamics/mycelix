// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Participatory Budgeting Integrity Zome
//!
//! Defines entry types and DHT-level shape/attribution invariants for community
//! budget cycles, project proposals, allocation votes, and fund-disbursement
//! records.
//!
//! This module deliberately does **not** claim to solve governed lifecycle
//! authorization or Finance settlement. Static integrity and author binding are
//! only the first authority boundary:
//! coordinator acceptance != DHT validity
//! author attribution != role authorization
//! governance disbursement record != Finance settlement proof

use hdi::prelude::*;
use mycelix_bridge_entry_types::{did_for_author, require_did_is_author};
use serde::{Deserialize, Serialize};

const MAX_ID_LEN: usize = 256;
const MAX_NAME_LEN: usize = 200;
const MAX_DESCRIPTION_LEN: usize = 8_192;
const MAX_CURRENCY_LEN: usize = 64;
const MAX_DID_LEN: usize = 256;
const MAX_DOMAIN_LEN: usize = 256;
const MAX_MILESTONES: usize = 100;

// ============================================================================
// Entry Types
// ============================================================================

/// A budget cycle defines a time-bounded funding round.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct BudgetCycle {
    pub cycle_id: String,
    pub name: String,
    pub total_budget: u64,
    pub currency: String,
    pub oversight_council_id: Option<String>,
    pub phase: BudgetPhase,
    pub proposal_deadline: u64,
    pub deliberation_deadline: u64,
    pub voting_deadline: u64,
    pub min_proposal_tier: u8,
    pub min_voting_tier: u8,
    pub voice_credits_per_voter: u64,
    pub created_at: Timestamp,
    pub creator_did: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum BudgetPhase {
    Proposal,
    Deliberation,
    Voting,
    Execution,
    Complete,
    Cancelled,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct BudgetProject {
    pub project_id: String,
    pub cycle_id: String,
    pub title: String,
    pub description: String,
    pub requested_amount: u64,
    pub minimum_amount: u64,
    pub proposer_did: String,
    pub beneficiary_domain: String,
    pub milestones: Vec<ProjectMilestone>,
    pub status: ProjectStatus,
    pub votes_received: u64,
    pub effective_weight: f64,
    pub allocated_amount: Option<u64>,
    pub created_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ProjectMilestone {
    pub description: String,
    pub percentage: u8,
    pub verified: bool,
    pub verifier_did: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProjectStatus {
    Proposed,
    UnderReview,
    Approved,
    PartiallyFunded,
    FullyFunded,
    InExecution,
    Completed,
    Rejected,
    Withdrawn,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct BudgetVote {
    pub cycle_id: String,
    pub project_id: String,
    pub voter_did: String,
    pub credits_spent: u64,
    pub direction: VoteDirection,
    pub voter_tier: u8,
    pub created_at: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoteDirection {
    Support,
    Oppose,
}

/// Governance-side record that a milestone disbursement was authorized/recorded.
///
/// This entry does not, by itself, prove that Mycelix Finance accepted an
/// allocation or that a monetary payment settled.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Disbursement {
    pub project_id: String,
    pub milestone_index: u32,
    pub amount: u64,
    pub recipient_did: String,
    pub authorizer_did: String,
    pub disbursed_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

// ============================================================================
// Entry & Link Types
// ============================================================================

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    BudgetCycle(BudgetCycle),
    BudgetProject(BudgetProject),
    BudgetVote(BudgetVote),
    Disbursement(Disbursement),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllCycles,
    CycleToProject,
    ProjectToVote,
    ProjectToDisbursement,
    AgentToVote,
    AgentToProject,
    CyclePhaseIndex,
}

// ============================================================================
// Validation
// ============================================================================

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(anchor) => Ok(validate_anchor(&anchor)),
                EntryTypes::BudgetCycle(cycle) => validate_create_budget_cycle(action, cycle),
                EntryTypes::BudgetProject(project) => {
                    validate_create_budget_project(action, project)
                }
                EntryTypes::BudgetVote(vote) => validate_create_budget_vote(action, vote),
                EntryTypes::Disbursement(disbursement) => {
                    validate_create_disbursement(action, disbursement)
                }
            },
            OpEntry::UpdateEntry { app_entry, .. } => Ok(match app_entry {
                // Predecessor-state and authorization checks are intentionally
                // deferred to GOV-BUDGET-AUTH-001B. This tranche still ensures
                // that any replacement entry satisfies the static schema.
                EntryTypes::BudgetCycle(cycle) => validate_budget_cycle(&cycle),
                EntryTypes::BudgetProject(project) => validate_budget_project(&project),
                // Votes and disbursement audit records are immutable. A new
                // correction/supersession primitive can be introduced later;
                // direct replacement would destroy audit meaning.
                EntryTypes::BudgetVote(_) => invalid("budget votes are immutable"),
                EntryTypes::Disbursement(_) => invalid("disbursement records are immutable"),
                EntryTypes::Anchor(_) => invalid("budget anchors are immutable"),
            }),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            base_address,
            target_address,
            ..
        } => {
            if base_address.as_ref().len() != 39 || target_address.as_ref().len() != 39 {
                return Ok(invalid("budget links must connect valid Holochain hashes"));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_budget_cycle(
    action: Create,
    cycle: BudgetCycle,
) -> ExternResult<ValidateCallbackResult> {
    let shape = validate_budget_cycle(&cycle);
    if !matches!(shape, ValidateCallbackResult::Valid) {
        return Ok(shape);
    }
    Ok(bind_author(
        "BudgetCycle",
        "creator_did",
        &cycle.creator_did,
        &action,
    ))
}

fn validate_create_budget_project(
    action: Create,
    project: BudgetProject,
) -> ExternResult<ValidateCallbackResult> {
    let shape = validate_budget_project(&project);
    if !matches!(shape, ValidateCallbackResult::Valid) {
        return Ok(shape);
    }
    Ok(bind_author(
        "BudgetProject",
        "proposer_did",
        &project.proposer_did,
        &action,
    ))
}

fn validate_create_budget_vote(
    action: Create,
    vote: BudgetVote,
) -> ExternResult<ValidateCallbackResult> {
    let shape = validate_budget_vote(&vote);
    if !matches!(shape, ValidateCallbackResult::Valid) {
        return Ok(shape);
    }
    Ok(bind_author(
        "BudgetVote",
        "voter_did",
        &vote.voter_did,
        &action,
    ))
}

fn validate_create_disbursement(
    action: Create,
    disbursement: Disbursement,
) -> ExternResult<ValidateCallbackResult> {
    let shape = validate_disbursement(&disbursement);
    if !matches!(shape, ValidateCallbackResult::Valid) {
        return Ok(shape);
    }
    Ok(bind_author(
        "Disbursement",
        "authorizer_did",
        &disbursement.authorizer_did,
        &action,
    ))
}

fn bind_author(entity: &str, field: &str, claimed_did: &str, action: &Create) -> ValidateCallbackResult {
    let author_did = did_for_author(&action.author);
    require_did_is_author(entity, field, claimed_did, &author_did)
}

fn validate_anchor(anchor: &Anchor) -> ValidateCallbackResult {
    validate_text(&anchor.0, "budget anchor", MAX_ID_LEN)
}

fn validate_budget_cycle(cycle: &BudgetCycle) -> ValidateCallbackResult {
    if let Some(invalid) = first_invalid([
        validate_text_result(&cycle.cycle_id, "budget cycle id", MAX_ID_LEN),
        validate_text_result(&cycle.name, "budget cycle name", MAX_NAME_LEN),
        validate_text_result(&cycle.currency, "budget currency", MAX_CURRENCY_LEN),
        validate_text_result(&cycle.creator_did, "budget cycle creator DID", MAX_DID_LEN),
    ]) {
        return invalid;
    }
    if let Some(council) = &cycle.oversight_council_id {
        if let Err(message) = validate_text_result(council, "oversight council id", MAX_ID_LEN) {
            return invalid(message);
        }
    }
    if cycle.total_budget == 0 {
        return invalid("budget cycle total_budget must be greater than zero");
    }
    if cycle.voice_credits_per_voter == 0 {
        return invalid("voice_credits_per_voter must be greater than zero");
    }
    if cycle.proposal_deadline >= cycle.deliberation_deadline
        || cycle.deliberation_deadline >= cycle.voting_deadline
    {
        return invalid("budget deadlines must satisfy proposal < deliberation < voting");
    }
    ValidateCallbackResult::Valid
}

fn validate_budget_project(project: &BudgetProject) -> ValidateCallbackResult {
    if let Some(invalid) = first_invalid([
        validate_text_result(&project.project_id, "budget project id", MAX_ID_LEN),
        validate_text_result(&project.cycle_id, "budget project cycle id", MAX_ID_LEN),
        validate_text_result(&project.title, "budget project title", MAX_NAME_LEN),
        validate_text_result(
            &project.description,
            "budget project description",
            MAX_DESCRIPTION_LEN,
        ),
        validate_text_result(&project.proposer_did, "budget project proposer DID", MAX_DID_LEN),
        validate_text_result(
            &project.beneficiary_domain,
            "budget beneficiary domain",
            MAX_DOMAIN_LEN,
        ),
    ]) {
        return invalid;
    }
    if project.requested_amount == 0 {
        return invalid("budget project requested_amount must be greater than zero");
    }
    if project.minimum_amount > project.requested_amount {
        return invalid("budget project minimum_amount cannot exceed requested_amount");
    }
    if !project.effective_weight.is_finite() || project.effective_weight < 0.0 {
        return invalid("budget project effective_weight must be finite and non-negative");
    }
    if let Some(allocated) = project.allocated_amount {
        if allocated == 0 {
            return invalid("allocated_amount, when present, must be greater than zero");
        }
        if allocated > project.requested_amount {
            return invalid("allocated_amount cannot exceed requested_amount");
        }
    }
    validate_milestones(&project.milestones)
}

fn validate_milestones(milestones: &[ProjectMilestone]) -> ValidateCallbackResult {
    if milestones.len() > MAX_MILESTONES {
        return invalid("budget project has too many milestones");
    }
    if milestones.is_empty() {
        return ValidateCallbackResult::Valid;
    }

    let mut total = 0u16;
    for milestone in milestones {
        if let Err(message) =
            validate_text_result(&milestone.description, "milestone description", MAX_DESCRIPTION_LEN)
        {
            return invalid(message);
        }
        if milestone.percentage == 0 || milestone.percentage > 100 {
            return invalid("each milestone percentage must be in 1..=100");
        }
        total += u16::from(milestone.percentage);
        match (milestone.verified, &milestone.verifier_did) {
            (true, Some(verifier)) => {
                if let Err(message) =
                    validate_text_result(verifier, "milestone verifier DID", MAX_DID_LEN)
                {
                    return invalid(message);
                }
            }
            (true, None) => return invalid("verified milestone requires verifier_did"),
            (false, Some(_)) => return invalid("unverified milestone cannot carry verifier_did"),
            (false, None) => {}
        }
    }
    if total != 100 {
        return invalid("milestone percentages must sum exactly to 100");
    }
    ValidateCallbackResult::Valid
}

fn validate_budget_vote(vote: &BudgetVote) -> ValidateCallbackResult {
    if let Some(invalid) = first_invalid([
        validate_text_result(&vote.cycle_id, "budget vote cycle id", MAX_ID_LEN),
        validate_text_result(&vote.project_id, "budget vote project id", MAX_ID_LEN),
        validate_text_result(&vote.voter_did, "budget voter DID", MAX_DID_LEN),
    ]) {
        return invalid;
    }
    if vote.credits_spent == 0 {
        return invalid("budget vote credits_spent must be greater than zero");
    }
    ValidateCallbackResult::Valid
}

fn validate_disbursement(disbursement: &Disbursement) -> ValidateCallbackResult {
    if let Some(invalid) = first_invalid([
        validate_text_result(&disbursement.project_id, "disbursement project id", MAX_ID_LEN),
        validate_text_result(
            &disbursement.recipient_did,
            "disbursement recipient DID",
            MAX_DID_LEN,
        ),
        validate_text_result(
            &disbursement.authorizer_did,
            "disbursement authorizer DID",
            MAX_DID_LEN,
        ),
    ]) {
        return invalid;
    }
    if disbursement.amount == 0 {
        return invalid("disbursement amount must be greater than zero");
    }
    ValidateCallbackResult::Valid
}

fn validate_text(value: &str, label: &str, max_len: usize) -> ValidateCallbackResult {
    match validate_text_result(value, label, max_len) {
        Ok(()) => ValidateCallbackResult::Valid,
        Err(message) => invalid(message),
    }
}

fn validate_text_result(value: &str, label: &str, max_len: usize) -> Result<(), String> {
    if value.is_empty() || value.trim() != value {
        return Err(format!("{label} must be a non-empty canonical string"));
    }
    if value.len() > max_len {
        return Err(format!("{label} cannot exceed {max_len} bytes"));
    }
    if value.chars().any(char::is_control) {
        return Err(format!("{label} cannot contain control characters"));
    }
    Ok(())
}

fn first_invalid<const N: usize>(checks: [Result<(), String>; N]) -> Option<ValidateCallbackResult> {
    checks
        .into_iter()
        .find_map(|result| result.err().map(invalid))
}

fn invalid(message: impl Into<String>) -> ValidateCallbackResult {
    ValidateCallbackResult::Invalid(message.into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deadlines_must_be_strictly_ordered() {
        let cycle = BudgetCycle {
            cycle_id: "Q2-2026".into(),
            name: "Quarterly commons budget".into(),
            total_budget: 1_000,
            currency: "SAP".into(),
            oversight_council_id: None,
            phase: BudgetPhase::Proposal,
            proposal_deadline: 10,
            deliberation_deadline: 20,
            voting_deadline: 30,
            min_proposal_tier: 0,
            min_voting_tier: 0,
            voice_credits_per_voter: 100,
            created_at: Timestamp::from_micros(0),
            creator_did: "did:mycelix:creator".into(),
        };
        assert!(matches!(validate_budget_cycle(&cycle), ValidateCallbackResult::Valid));

        let mut invalid_cycle = cycle;
        invalid_cycle.deliberation_deadline = 10;
        assert!(matches!(
            validate_budget_cycle(&invalid_cycle),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn milestone_percentages_must_sum_to_one_hundred() {
        let valid = vec![
            ProjectMilestone {
                description: "First deliverable".into(),
                percentage: 40,
                verified: false,
                verifier_did: None,
            },
            ProjectMilestone {
                description: "Second deliverable".into(),
                percentage: 60,
                verified: false,
                verifier_did: None,
            },
        ];
        assert!(matches!(validate_milestones(&valid), ValidateCallbackResult::Valid));

        let mut invalid_set = valid;
        invalid_set[1].percentage = 59;
        assert!(matches!(
            validate_milestones(&invalid_set),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn verifier_state_must_be_consistent() {
        let invalid_milestone = vec![ProjectMilestone {
            description: "Evidence deposited".into(),
            percentage: 100,
            verified: true,
            verifier_did: None,
        }];
        assert!(matches!(
            validate_milestones(&invalid_milestone),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn project_financial_shape_is_bounded() {
        let project = BudgetProject {
            project_id: "project:1".into(),
            cycle_id: "cycle:1".into(),
            title: "Replication project".into(),
            description: "Reproduce a registered experiment".into(),
            requested_amount: 1_000,
            minimum_amount: 500,
            proposer_did: "did:mycelix:researcher".into(),
            beneficiary_domain: "desci:replication".into(),
            milestones: vec![],
            status: ProjectStatus::Proposed,
            votes_received: 0,
            effective_weight: 0.0,
            allocated_amount: None,
            created_at: Timestamp::from_micros(0),
        };
        assert!(matches!(
            validate_budget_project(&project),
            ValidateCallbackResult::Valid
        ));

        let mut invalid_project = project;
        invalid_project.minimum_amount = 1_001;
        assert!(matches!(
            validate_budget_project(&invalid_project),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn vote_and_disbursement_amounts_must_be_nonzero() {
        let vote = BudgetVote {
            cycle_id: "cycle:1".into(),
            project_id: "project:1".into(),
            voter_did: "did:mycelix:voter".into(),
            credits_spent: 0,
            direction: VoteDirection::Support,
            voter_tier: 0,
            created_at: Timestamp::from_micros(0),
        };
        assert!(matches!(
            validate_budget_vote(&vote),
            ValidateCallbackResult::Invalid(_)
        ));

        let disbursement = Disbursement {
            project_id: "project:1".into(),
            milestone_index: 0,
            amount: 0,
            recipient_did: "did:mycelix:recipient".into(),
            authorizer_did: "did:mycelix:authorizer".into(),
            disbursed_at: Timestamp::from_micros(0),
        };
        assert!(matches!(
            validate_disbursement(&disbursement),
            ValidateCallbackResult::Invalid(_)
        ));
    }
}

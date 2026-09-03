// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Score-independent civic rights and binding-ballot primitives.
//!
//! The central separation is:
//!
//! > Credentials/capabilities establish eligibility. Scores may inform
//! > deliberation, but never create, remove, or multiply a core civic right.
//!
//! This crate intentionally has no Phi, consciousness, reputation, stake,
//! wealth, model-score, or advisory-score field in either rights authorization
//! or the binding ballot/tally path.

use mycelix_institutional_core::{
    InstitutionId, JurisdictionId, PrincipalId, RulebookRef,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-rights-v0.1";
const MAX_ID_BYTES: usize = 512;

macro_rules! id_type {
    ($name:ident, $field:literal) => {
        #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
        pub struct $name(pub String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, RightsError> {
                let value = value.into();
                validate_text(&value, $field, MAX_ID_BYTES)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

id_type!(EligibilityGrantId, "eligibility_grant_id");
id_type!(BallotId, "ballot_id");
id_type!(ProposalRef, "proposal_ref");
id_type!(AdvisoryRef, "advisory_ref");

/// Rights that an institution can explicitly grant/recognize.
///
/// The first group are procedural rights: an institution may scope who is a
/// participant, but once a principal is inside the applicable rights class a
/// model score must not silently remove these rights.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum CivicRight {
    Vote,
    ChallengeDecision,
    Appeal,
    ContestRecord,
    InspectEvidence,
    ReceiveNotice,
    OverrideVote,
    DelegateVote,
    ObserveProceedings,
    ExitInstitution,
}

/// Explicit, non-score sources from which civic eligibility may be derived.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EligibilitySourceKind {
    MembershipCredential,
    PersonhoodCredential,
    CitizenshipCredential,
    RoleCredential,
    GovernanceDecision,
    Delegation,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EligibilitySourceRef {
    pub kind: EligibilitySourceKind,
    pub reference: String,
    /// Host-verified proof/credential reference.
    pub proof_ref: String,
}

impl EligibilitySourceRef {
    pub fn validate(&self) -> Result<(), RightsError> {
        validate_text(&self.reference, "eligibility.source.reference", MAX_ID_BYTES)?;
        validate_text(&self.proof_ref, "eligibility.source.proof_ref", MAX_ID_BYTES)
    }
}

/// Explicit institutional recognition of a principal's civic rights.
///
/// There is deliberately no score or weight. The host verifies the credential
/// or governance source and its revocation state.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CivicEligibilityGrant {
    pub protocol_version: String,
    pub id: EligibilityGrantId,
    pub principal: PrincipalId,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub rights: Vec<CivicRight>,
    pub sources: Vec<EligibilitySourceRef>,
    pub issued_at_ms: u64,
    pub expires_at_ms: u64,
    pub grant_proof_ref: String,
}

impl CivicEligibilityGrant {
    pub fn validate(&self) -> Result<(), RightsError> {
        require_protocol(&self.protocol_version)?;
        self.rulebook.validate().map_err(|_| RightsError::InvalidRulebook)?;
        if self.rights.is_empty() {
            return Err(RightsError::NoRights);
        }
        let unique: BTreeSet<&CivicRight> = self.rights.iter().collect();
        if unique.len() != self.rights.len() {
            return Err(RightsError::DuplicateRight);
        }
        if self.sources.is_empty() {
            return Err(RightsError::NoEligibilitySources);
        }
        for source in &self.sources {
            source.validate()?;
        }
        if self.issued_at_ms == 0 || self.expires_at_ms <= self.issued_at_ms {
            return Err(RightsError::InvalidTimeRange("eligibility grant"));
        }
        validate_text(
            &self.grant_proof_ref,
            "eligibility.grant_proof_ref",
            MAX_ID_BYTES,
        )
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.issued_at_ms <= now_ms && now_ms < self.expires_at_ms
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RightRequirement {
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub required_right: CivicRight,
}

impl RightRequirement {
    pub fn validate(&self) -> Result<(), RightsError> {
        self.rulebook.validate().map_err(|_| RightsError::InvalidRulebook)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RightPermit {
    pub eligibility_grant_id: EligibilityGrantId,
    pub principal: PrincipalId,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub right: CivicRight,
    pub valid_until_ms: u64,
}

/// Authorize one civic right from explicit institutional eligibility.
///
/// The function signature is intentionally incapable of accepting a Phi score,
/// reputation score, stake balance, model assessment, or advisory signal.
pub fn authorize_right(
    grant: &CivicEligibilityGrant,
    requirement: &RightRequirement,
    now_ms: u64,
) -> Result<RightPermit, RightsError> {
    grant.validate()?;
    requirement.validate()?;
    if !grant.is_active_at(now_ms) {
        return Err(RightsError::EligibilityInactive);
    }
    if grant.institution != requirement.institution {
        return Err(RightsError::InstitutionMismatch);
    }
    if grant.jurisdiction != requirement.jurisdiction {
        return Err(RightsError::JurisdictionMismatch);
    }
    if grant.rulebook != requirement.rulebook {
        return Err(RightsError::RulebookMismatch);
    }
    if !grant.rights.contains(&requirement.required_right) {
        return Err(RightsError::RightNotGranted);
    }
    Ok(RightPermit {
        eligibility_grant_id: grant.id.clone(),
        principal: grant.principal.clone(),
        institution: grant.institution.clone(),
        jurisdiction: grant.jurisdiction.clone(),
        rulebook: grant.rulebook.clone(),
        right: requirement.required_right.clone(),
        valid_until_ms: grant.expires_at_ms,
    })
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BallotChoice {
    For,
    Against,
    Abstain,
}

/// Binding civic ballot.
///
/// There is intentionally no `weight: f64`. Every unique eligible principal
/// contributes exactly one binding ballot to this tally domain.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BindingBallot {
    pub protocol_version: String,
    pub id: BallotId,
    pub proposal: ProposalRef,
    pub voter: PrincipalId,
    pub eligibility_grant_id: EligibilityGrantId,
    pub choice: BallotChoice,
    pub cast_at_ms: u64,
    pub ballot_proof_ref: String,
}

impl BindingBallot {
    pub fn validate(&self) -> Result<(), RightsError> {
        require_protocol(&self.protocol_version)?;
        if self.cast_at_ms == 0 {
            return Err(RightsError::ZeroTimestamp);
        }
        validate_text(
            &self.ballot_proof_ref,
            "ballot.ballot_proof_ref",
            MAX_ID_BYTES,
        )
    }

    pub fn validate_against_permit(&self, permit: &RightPermit) -> Result<(), RightsError> {
        self.validate()?;
        if permit.right != CivicRight::Vote {
            return Err(RightsError::PermitIsNotVoteRight);
        }
        if self.voter != permit.principal {
            return Err(RightsError::BallotVoterMismatch);
        }
        if self.eligibility_grant_id != permit.eligibility_grant_id {
            return Err(RightsError::EligibilityGrantMismatch);
        }
        if self.cast_at_ms >= permit.valid_until_ms {
            return Err(RightsError::EligibilityInactive);
        }
        Ok(())
    }
}

/// Binding tally policy in integer basis points.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BindingTallyPolicy {
    /// 0..=10_000 fraction of eligible voters required to participate.
    pub quorum_bp: u16,
    /// 0..=10_000 fraction of decisive ballots required for approval.
    pub approval_bp: u16,
    /// Absolute minimum number of unique voters, independent of population size.
    pub absolute_quorum_floor: u64,
}

impl BindingTallyPolicy {
    pub fn validate(&self) -> Result<(), RightsError> {
        if self.quorum_bp > 10_000 || self.approval_bp > 10_000 {
            return Err(RightsError::BasisPointsOutOfRange);
        }
        if self.approval_bp < 5_000 {
            return Err(RightsError::ApprovalBelowSimpleMajority);
        }
        if self.absolute_quorum_floor == 0 {
            return Err(RightsError::ZeroAbsoluteQuorum);
        }
        Ok(())
    }

    pub fn required_voters(&self, eligible_voters: u64) -> Result<u64, RightsError> {
        self.validate()?;
        if eligible_voters == 0 {
            return Err(RightsError::ZeroEligibleVoters);
        }
        let proportional = eligible_voters
            .saturating_mul(self.quorum_bp as u64)
            .saturating_add(9_999)
            / 10_000;
        Ok(proportional.max(self.absolute_quorum_floor))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BindingTally {
    pub proposal: ProposalRef,
    pub eligible_voters: u64,
    pub unique_voters: u64,
    pub votes_for: u64,
    pub votes_against: u64,
    pub abstentions: u64,
    pub required_voters: u64,
    pub quorum_reached: bool,
    pub approval_bp_observed: u16,
    pub approved: bool,
}

/// Tally binding ballots as one eligible principal = one ballot.
///
/// Inputs from any advisory/score system are intentionally absent.
pub fn tally_binding_ballots(
    proposal: &ProposalRef,
    ballots: &[BindingBallot],
    eligible_voters: u64,
    policy: &BindingTallyPolicy,
) -> Result<BindingTally, RightsError> {
    policy.validate()?;
    if eligible_voters == 0 {
        return Err(RightsError::ZeroEligibleVoters);
    }

    let mut ballot_ids = BTreeSet::new();
    let mut voters = BTreeSet::new();
    let mut votes_for = 0u64;
    let mut votes_against = 0u64;
    let mut abstentions = 0u64;

    for ballot in ballots {
        ballot.validate()?;
        if &ballot.proposal != proposal {
            return Err(RightsError::ProposalMismatch);
        }
        if !ballot_ids.insert(ballot.id.clone()) {
            return Err(RightsError::DuplicateBallotId);
        }
        if !voters.insert(ballot.voter.clone()) {
            return Err(RightsError::DuplicateVoter);
        }
        match ballot.choice {
            BallotChoice::For => votes_for += 1,
            BallotChoice::Against => votes_against += 1,
            BallotChoice::Abstain => abstentions += 1,
        }
    }

    let unique_voters = voters.len() as u64;
    if unique_voters > eligible_voters {
        return Err(RightsError::MoreVotersThanEligiblePopulation);
    }
    let required_voters = policy.required_voters(eligible_voters)?;
    let quorum_reached = unique_voters >= required_voters;

    let decisive = votes_for + votes_against;
    let approval_bp_observed = if decisive == 0 {
        0
    } else {
        ((votes_for.saturating_mul(10_000) / decisive).min(10_000)) as u16
    };
    let approved = quorum_reached
        && decisive > 0
        && approval_bp_observed >= policy.approval_bp;

    Ok(BindingTally {
        proposal: proposal.clone(),
        eligible_voters,
        unique_voters,
        votes_for,
        votes_against,
        abstentions,
        required_voters,
        quorum_reached,
        approval_bp_observed,
        approved,
    })
}

/// Explicitly non-binding assessment reference.
///
/// A host may attach Phi/reputation/model analysis here for transparency or
/// deliberation. No authorization or tally function in this crate accepts it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdvisoryAssessment {
    pub protocol_version: String,
    pub id: AdvisoryRef,
    pub proposal: ProposalRef,
    pub assessment_type: String,
    pub result_ref: String,
    pub generated_at_ms: u64,
}

impl AdvisoryAssessment {
    pub fn validate(&self) -> Result<(), RightsError> {
        require_protocol(&self.protocol_version)?;
        validate_text(&self.assessment_type, "advisory.type", 256)?;
        validate_text(&self.result_ref, "advisory.result_ref", MAX_ID_BYTES)?;
        if self.generated_at_ms == 0 {
            return Err(RightsError::ZeroTimestamp);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RightsError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    InvalidRulebook,
    NoRights,
    DuplicateRight,
    NoEligibilitySources,
    InvalidTimeRange(&'static str),
    EligibilityInactive,
    InstitutionMismatch,
    JurisdictionMismatch,
    RulebookMismatch,
    RightNotGranted,
    ZeroTimestamp,
    PermitIsNotVoteRight,
    BallotVoterMismatch,
    EligibilityGrantMismatch,
    BasisPointsOutOfRange,
    ApprovalBelowSimpleMajority,
    ZeroAbsoluteQuorum,
    ZeroEligibleVoters,
    ProposalMismatch,
    DuplicateBallotId,
    DuplicateVoter,
    MoreVotersThanEligiblePopulation,
}

impl fmt::Display for RightsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::InvalidRulebook => write!(f, "invalid rulebook reference"),
            Self::NoRights => write!(f, "eligibility grant contains no rights"),
            Self::DuplicateRight => write!(f, "eligibility grant contains duplicate rights"),
            Self::NoEligibilitySources => write!(f, "eligibility grant has no sources"),
            Self::InvalidTimeRange(field) => write!(f, "invalid time range for {field}"),
            Self::EligibilityInactive => write!(f, "eligibility is inactive"),
            Self::InstitutionMismatch => write!(f, "institution mismatch"),
            Self::JurisdictionMismatch => write!(f, "jurisdiction mismatch"),
            Self::RulebookMismatch => write!(f, "rulebook mismatch"),
            Self::RightNotGranted => write!(f, "required civic right is not granted"),
            Self::ZeroTimestamp => write!(f, "timestamp must be non-zero"),
            Self::PermitIsNotVoteRight => write!(f, "permit does not authorize voting"),
            Self::BallotVoterMismatch => write!(f, "ballot voter does not match permit principal"),
            Self::EligibilityGrantMismatch => write!(f, "ballot references a different eligibility grant"),
            Self::BasisPointsOutOfRange => write!(f, "basis-point threshold exceeds 10,000"),
            Self::ApprovalBelowSimpleMajority => write!(f, "binding approval threshold cannot be below simple majority"),
            Self::ZeroAbsoluteQuorum => write!(f, "absolute quorum floor must be positive"),
            Self::ZeroEligibleVoters => write!(f, "eligible-voter population must be positive"),
            Self::ProposalMismatch => write!(f, "ballot belongs to a different proposal"),
            Self::DuplicateBallotId => write!(f, "duplicate ballot id"),
            Self::DuplicateVoter => write!(f, "one principal cannot contribute multiple binding ballots"),
            Self::MoreVotersThanEligiblePopulation => write!(f, "binding voters exceed eligible population"),
        }
    }
}

impl std::error::Error for RightsError {}

fn require_protocol(version: &str) -> Result<(), RightsError> {
    if version == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(RightsError::WrongProtocolVersion)
    }
}

fn validate_text(value: &str, field: &'static str, max: usize) -> Result<(), RightsError> {
    if value.trim().is_empty() {
        return Err(RightsError::Empty(field));
    }
    if value.len() > max {
        return Err(RightsError::TooLong(field));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_institutional_core::{RulebookId, Digest32};

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:community:v1").unwrap(),
            version: "1.0.0".into(),
            digest: Digest32([7; 32]),
        }
    }

    fn principal(name: &str) -> PrincipalId {
        PrincipalId::new(format!("did:example:{name}")).unwrap()
    }

    fn institution() -> InstitutionId {
        InstitutionId::new("institution:community").unwrap()
    }

    fn grant(name: &str) -> CivicEligibilityGrant {
        CivicEligibilityGrant {
            protocol_version: PROTOCOL_VERSION.into(),
            id: EligibilityGrantId::new(format!("eligibility:{name}")).unwrap(),
            principal: principal(name),
            institution: institution(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:local").unwrap()),
            rulebook: rulebook(),
            rights: vec![
                CivicRight::Vote,
                CivicRight::ChallengeDecision,
                CivicRight::Appeal,
                CivicRight::InspectEvidence,
            ],
            sources: vec![EligibilitySourceRef {
                kind: EligibilitySourceKind::MembershipCredential,
                reference: format!("credential:member:{name}"),
                proof_ref: format!("proof:member:{name}"),
            }],
            issued_at_ms: 1_000,
            expires_at_ms: 20_000,
            grant_proof_ref: format!("proof:eligibility:{name}"),
        }
    }

    fn vote_requirement() -> RightRequirement {
        RightRequirement {
            institution: institution(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:local").unwrap()),
            rulebook: rulebook(),
            required_right: CivicRight::Vote,
        }
    }

    fn ballot(name: &str, choice: BallotChoice) -> BindingBallot {
        BindingBallot {
            protocol_version: PROTOCOL_VERSION.into(),
            id: BallotId::new(format!("ballot:{name}")).unwrap(),
            proposal: ProposalRef::new("MIP-101").unwrap(),
            voter: principal(name),
            eligibility_grant_id: EligibilityGrantId::new(format!("eligibility:{name}")).unwrap(),
            choice,
            cast_at_ms: 5_000,
            ballot_proof_ref: format!("proof:ballot:{name}"),
        }
    }

    #[test]
    fn explicit_eligibility_authorizes_vote_right_without_scores() {
        let permit = authorize_right(&grant("alice"), &vote_requirement(), 5_000).unwrap();
        assert_eq!(permit.right, CivicRight::Vote);
        assert_eq!(permit.principal, principal("alice"));
    }

    #[test]
    fn wrong_rulebook_cannot_authorize_right() {
        let mut requirement = vote_requirement();
        requirement.rulebook.digest = Digest32([8; 32]);
        assert_eq!(
            authorize_right(&grant("alice"), &requirement, 5_000).unwrap_err(),
            RightsError::RulebookMismatch
        );
    }

    #[test]
    fn expired_eligibility_cannot_authorize_right() {
        assert_eq!(
            authorize_right(&grant("alice"), &vote_requirement(), 20_000).unwrap_err(),
            RightsError::EligibilityInactive
        );
    }

    #[test]
    fn ballot_binds_voter_to_eligibility_permit() {
        let permit = authorize_right(&grant("alice"), &vote_requirement(), 5_000).unwrap();
        assert!(ballot("alice", BallotChoice::For)
            .validate_against_permit(&permit)
            .is_ok());

        let forged = ballot("mallory", BallotChoice::For);
        assert_eq!(
            forged.validate_against_permit(&permit).unwrap_err(),
            RightsError::BallotVoterMismatch
        );
    }

    #[test]
    fn one_principal_gets_one_binding_ballot() {
        let proposal = ProposalRef::new("MIP-101").unwrap();
        let ballots = vec![
            ballot("alice", BallotChoice::For),
            BindingBallot {
                id: BallotId::new("ballot:alice:second").unwrap(),
                ..ballot("alice", BallotChoice::Against)
            },
        ];
        let policy = BindingTallyPolicy {
            quorum_bp: 2_500,
            approval_bp: 5_001,
            absolute_quorum_floor: 2,
        };
        assert_eq!(
            tally_binding_ballots(&proposal, &ballots, 10, &policy).unwrap_err(),
            RightsError::DuplicateVoter
        );
    }

    #[test]
    fn quorum_is_based_on_eligible_people_not_vote_weight() {
        let proposal = ProposalRef::new("MIP-101").unwrap();
        let ballots = vec![
            ballot("alice", BallotChoice::For),
            ballot("bob", BallotChoice::For),
            ballot("carol", BallotChoice::Against),
        ];
        let policy = BindingTallyPolicy {
            quorum_bp: 2_500,
            approval_bp: 6_000,
            absolute_quorum_floor: 2,
        };
        // 25% of 10 = 3 after ceiling. Exactly three distinct voters reach quorum.
        let tally = tally_binding_ballots(&proposal, &ballots, 10, &policy).unwrap();
        assert_eq!(tally.required_voters, 3);
        assert!(tally.quorum_reached);
        assert_eq!(tally.votes_for, 2);
        assert_eq!(tally.votes_against, 1);
        assert!(tally.approved); // 66.66% >= 60%
    }

    #[test]
    fn absolute_quorum_floor_prevents_tiny_population_capture() {
        let proposal = ProposalRef::new("MIP-101").unwrap();
        let ballots = vec![
            ballot("alice", BallotChoice::For),
            ballot("bob", BallotChoice::For),
        ];
        let policy = BindingTallyPolicy {
            quorum_bp: 1_500,
            approval_bp: 5_001,
            absolute_quorum_floor: 3,
        };
        let tally = tally_binding_ballots(&proposal, &ballots, 100, &policy).unwrap();
        assert_eq!(tally.required_voters, 15);
        assert!(!tally.quorum_reached);
        assert!(!tally.approved);
    }

    #[test]
    fn advisory_assessments_have_no_binding_tally_input() {
        let assessment = AdvisoryAssessment {
            protocol_version: PROTOCOL_VERSION.into(),
            id: AdvisoryRef::new("advisory:phi:alice").unwrap(),
            proposal: ProposalRef::new("MIP-101").unwrap(),
            assessment_type: "phi-distribution".into(),
            result_ref: "artifact:analysis:1".into(),
            generated_at_ms: 4_000,
        };
        assert!(assessment.validate().is_ok());

        // The binding tally API accepts only ballots, population and policy.
        // There is deliberately no path to pass `assessment` into the tally.
        let policy = BindingTallyPolicy {
            quorum_bp: 2_500,
            approval_bp: 5_001,
            absolute_quorum_floor: 2,
        };
        let proposal = ProposalRef::new("MIP-101").unwrap();
        let tally = tally_binding_ballots(
            &proposal,
            &[
                ballot("alice", BallotChoice::For),
                ballot("bob", BallotChoice::Against),
                ballot("carol", BallotChoice::For),
            ],
            10,
            &policy,
        )
        .unwrap();
        assert_eq!(tally.unique_voters, 3);
    }

    #[test]
    fn wire_round_trip_preserves_no_weight_ballot_shape() {
        let ballot = ballot("alice", BallotChoice::For);
        let json = serde_json::to_string(&ballot).unwrap();
        assert!(!json.contains("weight"));
        let decoded: BindingBallot = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, ballot);
    }
}

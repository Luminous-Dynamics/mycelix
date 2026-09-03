// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Immutable electorate snapshots for binding Mycelix governance.
//!
//! A binding tally needs two independent facts:
//!
//! 1. this principal has the right to vote under the proposal's institution /
//!    jurisdiction / rulebook; and
//! 2. this proposal's electorate denominator was fixed by an authenticated,
//!    immutable snapshot rather than supplied by the tally caller.
//!
//! This crate models the second fact and binds it to the first. It contains no
//! Holochain, VC, ZK, signature, Xenia, Symthaea, or network dependency. Hosts
//! verify the referenced authority/proofs and persist snapshots immutably.

use mycelix_governance_rights::{
    BindingBallot, BindingTally, BindingTallyPolicy, CivicRight, EligibilityGrantId, ProposalRef,
    RightPermit, tally_binding_ballots,
};
use mycelix_institutional_core::{
    AuthorityGrantId, Digest32, InstitutionId, JurisdictionId, PrincipalId, RulebookRef,
};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-electorate-v0.1";
const MAX_ID_BYTES: usize = 512;
const MAX_PROFILE_BYTES: usize = 128;

macro_rules! id_type {
    ($name:ident, $field:literal) => {
        #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
        pub struct $name(pub String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, ElectorateError> {
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

id_type!(ElectorateSnapshotId, "electorate_snapshot_id");

/// What happens to eligibility after the electorate has been frozen.
///
/// The denominator never changes silently during a ballot. If a deployment
/// needs a materially different electorate it must run a distinct ballot with
/// a new snapshot, preserving the original result/history.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RevocationSemantics {
    /// Eligibility is evaluated at the snapshot boundary for this ballot.
    /// Subsequent credential changes affect future ballots, not this one.
    FrozenAtSnapshot,
    /// A later revocation may prevent the affected principal from casting, but
    /// the original electorate denominator remains fixed for auditability.
    RejectRevokedBallotKeepDenominator,
}

/// Immutable electorate population statement for one proposal.
///
/// `membership_commitment` commits the exact member set without requiring the
/// public protocol to publish that set. `eligible_voters` is the population
/// count committed by that same member-set profile and is never accepted as a
/// standalone caller parameter by this crate's tally API.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ElectorateSnapshot {
    pub protocol_version: String,
    pub id: ElectorateSnapshotId,
    pub proposal: ProposalRef,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,

    /// Number of principals in the committed electorate set.
    pub eligible_voters: u64,

    /// Digest of the exact rule/credential predicate used to select members.
    pub eligibility_criteria_digest: Digest32,
    pub eligibility_criteria_profile: String,

    /// Privacy-preserving commitment to the exact electorate member set.
    pub membership_commitment: Digest32,
    pub membership_commitment_profile: String,

    /// Snapshot must be captured no later than ballot opening.
    pub captured_at_ms: u64,
    pub ballot_opens_at_ms: u64,
    pub ballot_closes_at_ms: u64,
    pub revocation_semantics: RevocationSemantics,

    /// Institutional authority that authorized/froze this snapshot.
    pub authorized_by: AuthorityGrantId,
    /// Host-verified proof/receipt covering this exact snapshot.
    pub snapshot_proof_ref: String,
}

impl ElectorateSnapshot {
    pub fn validate(&self) -> Result<(), ElectorateError> {
        require_protocol(&self.protocol_version)?;
        validate_text(self.id.as_str(), "snapshot.id", MAX_ID_BYTES)?;
        validate_text(self.proposal.as_str(), "snapshot.proposal", MAX_ID_BYTES)?;
        validate_text(
            self.institution.as_str(),
            "snapshot.institution",
            MAX_ID_BYTES,
        )?;
        if let Some(jurisdiction) = &self.jurisdiction {
            validate_text(
                jurisdiction.as_str(),
                "snapshot.jurisdiction",
                MAX_ID_BYTES,
            )?;
        }
        self.rulebook
            .validate()
            .map_err(|_| ElectorateError::InvalidRulebook)?;
        if self.eligible_voters == 0 {
            return Err(ElectorateError::EmptyElectorate);
        }
        require_digest(
            self.eligibility_criteria_digest,
            "snapshot.eligibility_criteria_digest",
        )?;
        require_digest(
            self.membership_commitment,
            "snapshot.membership_commitment",
        )?;
        validate_profile(
            &self.eligibility_criteria_profile,
            "snapshot.eligibility_criteria_profile",
        )?;
        validate_profile(
            &self.membership_commitment_profile,
            "snapshot.membership_commitment_profile",
        )?;
        if self.captured_at_ms == 0
            || self.ballot_opens_at_ms == 0
            || self.ballot_closes_at_ms <= self.ballot_opens_at_ms
            || self.captured_at_ms > self.ballot_opens_at_ms
        {
            return Err(ElectorateError::InvalidSnapshotWindow);
        }
        validate_text(
            self.authorized_by.as_str(),
            "snapshot.authorized_by",
            MAX_ID_BYTES,
        )?;
        validate_text(
            &self.snapshot_proof_ref,
            "snapshot.snapshot_proof_ref",
            MAX_ID_BYTES,
        )
    }

    pub fn ballot_open_at(&self, timestamp_ms: u64) -> bool {
        self.ballot_opens_at_ms <= timestamp_ms && timestamp_ms < self.ballot_closes_at_ms
    }
}

/// Host-produced evidence that one principal is a member of the exact committed
/// electorate set.
///
/// The host MUST verify the proof named by `membership_proof_ref` against
/// `membership_commitment` under `verification_profile`. A populated struct is
/// not, by itself, cryptographic proof.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ElectorateMembershipEvidence {
    pub protocol_version: String,
    pub snapshot_id: ElectorateSnapshotId,
    pub principal: PrincipalId,
    pub eligibility_grant_id: EligibilityGrantId,
    pub membership_commitment: Digest32,
    pub verification_profile: String,
    pub verifier_id: String,
    pub trust_domain_id: String,
    pub verified_at_ms: u64,
    pub expires_at_ms: u64,
    pub membership_proof_ref: String,
}

impl ElectorateMembershipEvidence {
    pub fn validate(&self) -> Result<(), ElectorateError> {
        require_protocol(&self.protocol_version)?;
        validate_text(
            self.snapshot_id.as_str(),
            "membership.snapshot_id",
            MAX_ID_BYTES,
        )?;
        validate_text(
            self.principal.as_str(),
            "membership.principal",
            MAX_ID_BYTES,
        )?;
        validate_text(
            self.eligibility_grant_id.as_str(),
            "membership.eligibility_grant_id",
            MAX_ID_BYTES,
        )?;
        require_digest(
            self.membership_commitment,
            "membership.membership_commitment",
        )?;
        validate_profile(&self.verification_profile, "membership.verification_profile")?;
        validate_text(&self.verifier_id, "membership.verifier_id", MAX_ID_BYTES)?;
        validate_text(
            &self.trust_domain_id,
            "membership.trust_domain_id",
            MAX_ID_BYTES,
        )?;
        if self.verified_at_ms == 0 || self.expires_at_ms <= self.verified_at_ms {
            return Err(ElectorateError::InvalidMembershipLifetime);
        }
        validate_text(
            &self.membership_proof_ref,
            "membership.membership_proof_ref",
            MAX_ID_BYTES,
        )
    }
}

/// Full qualification package for one binding ballot.
///
/// The package intentionally contains the ballot, the explicit civic-right
/// permit and the electorate membership evidence so tallying can re-check all
/// structural bindings instead of trusting a pre-computed `eligible: true` bit.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BallotQualification {
    pub ballot: BindingBallot,
    pub permit: RightPermit,
    pub membership: ElectorateMembershipEvidence,
}

impl BallotQualification {
    pub fn validate_against_snapshot(
        &self,
        snapshot: &ElectorateSnapshot,
    ) -> Result<(), ElectorateError> {
        snapshot.validate()?;
        self.membership.validate()?;
        self.ballot
            .validate_against_permit(&self.permit)
            .map_err(|_| ElectorateError::InvalidRightPermit)?;

        if self.permit.right != CivicRight::Vote {
            return Err(ElectorateError::PermitIsNotVoteRight);
        }
        if self.ballot.proposal != snapshot.proposal {
            return Err(ElectorateError::ProposalMismatch);
        }
        if self.permit.institution != snapshot.institution {
            return Err(ElectorateError::InstitutionMismatch);
        }
        if self.permit.jurisdiction != snapshot.jurisdiction {
            return Err(ElectorateError::JurisdictionMismatch);
        }
        if self.permit.rulebook != snapshot.rulebook {
            return Err(ElectorateError::RulebookMismatch);
        }
        if !snapshot.ballot_open_at(self.ballot.cast_at_ms) {
            return Err(ElectorateError::BallotOutsideSnapshotWindow);
        }
        if self.membership.snapshot_id != snapshot.id {
            return Err(ElectorateError::SnapshotMismatch);
        }
        if self.membership.principal != self.ballot.voter
            || self.membership.principal != self.permit.principal
        {
            return Err(ElectorateError::PrincipalMismatch);
        }
        if self.membership.eligibility_grant_id != self.ballot.eligibility_grant_id
            || self.membership.eligibility_grant_id != self.permit.eligibility_grant_id
        {
            return Err(ElectorateError::EligibilityGrantMismatch);
        }
        if self.membership.membership_commitment != snapshot.membership_commitment {
            return Err(ElectorateError::MembershipCommitmentMismatch);
        }
        if self.membership.verification_profile != snapshot.membership_commitment_profile {
            return Err(ElectorateError::MembershipProfileMismatch);
        }
        if self.membership.verified_at_ms > self.ballot.cast_at_ms
            || self.membership.expires_at_ms <= self.ballot.cast_at_ms
        {
            return Err(ElectorateError::MembershipEvidenceInactive);
        }

        Ok(())
    }
}

/// Binding result that always names the electorate snapshot used as the quorum
/// denominator. Consumers do not receive a naked tally detached from its
/// population provenance.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SnapshotBoundTally {
    pub snapshot_id: ElectorateSnapshotId,
    pub membership_commitment: Digest32,
    pub snapshot_proof_ref: String,
    pub tally: BindingTally,
}

/// Tally qualified binding ballots against an authenticated electorate snapshot.
///
/// There is deliberately no `eligible_voters` argument. The denominator comes
/// only from the snapshot. Every ballot is re-checked against its civic-right
/// permit and membership evidence before raw-count tallying.
pub fn tally_snapshot_ballots(
    snapshot: &ElectorateSnapshot,
    policy: &BindingTallyPolicy,
    qualifications: &[BallotQualification],
) -> Result<SnapshotBoundTally, ElectorateError> {
    snapshot.validate()?;

    let mut ballots = Vec::with_capacity(qualifications.len());
    for qualification in qualifications {
        qualification.validate_against_snapshot(snapshot)?;
        ballots.push(qualification.ballot.clone());
    }

    let tally = tally_binding_ballots(
        &snapshot.proposal,
        &ballots,
        snapshot.eligible_voters,
        policy,
    )
    .map_err(|_| ElectorateError::InvalidBindingTally)?;

    Ok(SnapshotBoundTally {
        snapshot_id: snapshot.id.clone(),
        membership_commitment: snapshot.membership_commitment,
        snapshot_proof_ref: snapshot.snapshot_proof_ref.clone(),
        tally,
    })
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ElectorateError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    InvalidProfile(&'static str),
    ZeroDigest(&'static str),
    InvalidRulebook,
    EmptyElectorate,
    InvalidSnapshotWindow,
    InvalidMembershipLifetime,
    InvalidRightPermit,
    PermitIsNotVoteRight,
    ProposalMismatch,
    InstitutionMismatch,
    JurisdictionMismatch,
    RulebookMismatch,
    BallotOutsideSnapshotWindow,
    SnapshotMismatch,
    PrincipalMismatch,
    EligibilityGrantMismatch,
    MembershipCommitmentMismatch,
    MembershipProfileMismatch,
    MembershipEvidenceInactive,
    InvalidBindingTally,
}

impl fmt::Display for ElectorateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong electorate protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::InvalidProfile(field) => write!(f, "{field} is not a canonical profile token"),
            Self::ZeroDigest(field) => write!(f, "{field} must not be the zero digest"),
            Self::InvalidRulebook => write!(f, "invalid electorate rulebook"),
            Self::EmptyElectorate => write!(f, "electorate must contain at least one principal"),
            Self::InvalidSnapshotWindow => write!(f, "invalid electorate snapshot/voting window"),
            Self::InvalidMembershipLifetime => write!(f, "invalid membership evidence lifetime"),
            Self::InvalidRightPermit => write!(f, "invalid civic-right permit for ballot"),
            Self::PermitIsNotVoteRight => write!(f, "permit does not grant voting right"),
            Self::ProposalMismatch => write!(f, "ballot and electorate snapshot target different proposals"),
            Self::InstitutionMismatch => write!(f, "permit and electorate institution differ"),
            Self::JurisdictionMismatch => write!(f, "permit and electorate jurisdiction differ"),
            Self::RulebookMismatch => write!(f, "permit and electorate rulebook differ"),
            Self::BallotOutsideSnapshotWindow => write!(f, "ballot is outside electorate voting window"),
            Self::SnapshotMismatch => write!(f, "membership evidence targets a different snapshot"),
            Self::PrincipalMismatch => write!(f, "ballot, permit and membership principal differ"),
            Self::EligibilityGrantMismatch => write!(f, "ballot, permit and membership grant differ"),
            Self::MembershipCommitmentMismatch => write!(f, "membership evidence targets another electorate commitment"),
            Self::MembershipProfileMismatch => write!(f, "membership proof profile differs from electorate commitment profile"),
            Self::MembershipEvidenceInactive => write!(f, "membership evidence is not active when ballot is cast"),
            Self::InvalidBindingTally => write!(f, "binding tally rejected qualified ballot set"),
        }
    }
}

impl std::error::Error for ElectorateError {}

fn require_protocol(version: &str) -> Result<(), ElectorateError> {
    if version == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ElectorateError::WrongProtocolVersion)
    }
}

fn validate_text(
    value: &str,
    field: &'static str,
    max_bytes: usize,
) -> Result<(), ElectorateError> {
    if value.trim().is_empty() {
        return Err(ElectorateError::Empty(field));
    }
    if value.len() > max_bytes {
        return Err(ElectorateError::TooLong(field));
    }
    Ok(())
}

fn validate_profile(value: &str, field: &'static str) -> Result<(), ElectorateError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        return Err(ElectorateError::InvalidProfile(field));
    }
    Ok(())
}

fn require_digest(digest: Digest32, field: &'static str) -> Result<(), ElectorateError> {
    if digest.is_zero() {
        Err(ElectorateError::ZeroDigest(field))
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_governance_rights::{
        BallotChoice, BallotId, BindingBallot, CivicRight, EligibilityGrantId, RightPermit,
        PROTOCOL_VERSION as RIGHTS_PROTOCOL_VERSION,
    };
    use mycelix_institutional_core::{
        InstitutionId, JurisdictionId, PrincipalId, RulebookId, RulebookRef,
    };

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:community:v1").unwrap(),
            version: "1".into(),
            digest: digest(7),
        }
    }

    fn snapshot() -> ElectorateSnapshot {
        ElectorateSnapshot {
            protocol_version: PROTOCOL_VERSION.into(),
            id: ElectorateSnapshotId::new("electorate:MIP-42:v1").unwrap(),
            proposal: ProposalRef::new("MIP-42").unwrap(),
            institution: InstitutionId::new("institution:community").unwrap(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:za").unwrap()),
            rulebook: rulebook(),
            eligible_voters: 100,
            eligibility_criteria_digest: digest(8),
            eligibility_criteria_profile: "mycelix-eligibility-criteria-v1".into(),
            membership_commitment: digest(9),
            membership_commitment_profile: "mycelix-electorate-membership-v1".into(),
            captured_at_ms: 900,
            ballot_opens_at_ms: 1_000,
            ballot_closes_at_ms: 10_000,
            revocation_semantics: RevocationSemantics::RejectRevokedBallotKeepDenominator,
            authorized_by: AuthorityGrantId::new("grant:electorate:v1").unwrap(),
            snapshot_proof_ref: "proof:electorate:v1".into(),
        }
    }

    fn qualification(name: &str, choice: BallotChoice, cast_at_ms: u64) -> BallotQualification {
        let principal = PrincipalId::new(format!("did:mycelix:{name}")).unwrap();
        let grant_id = EligibilityGrantId::new(format!("eligibility:{name}")).unwrap();
        let permit = RightPermit {
            eligibility_grant_id: grant_id.clone(),
            principal: principal.clone(),
            institution: InstitutionId::new("institution:community").unwrap(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:za").unwrap()),
            rulebook: rulebook(),
            right: CivicRight::Vote,
            valid_until_ms: 20_000,
        };
        BallotQualification {
            ballot: BindingBallot {
                protocol_version: RIGHTS_PROTOCOL_VERSION.into(),
                id: BallotId::new(format!("ballot:{name}")).unwrap(),
                proposal: ProposalRef::new("MIP-42").unwrap(),
                voter: principal.clone(),
                eligibility_grant_id: grant_id.clone(),
                choice,
                cast_at_ms,
                ballot_proof_ref: format!("proof:ballot:{name}"),
            },
            permit,
            membership: ElectorateMembershipEvidence {
                protocol_version: PROTOCOL_VERSION.into(),
                snapshot_id: ElectorateSnapshotId::new("electorate:MIP-42:v1").unwrap(),
                principal,
                eligibility_grant_id: grant_id,
                membership_commitment: digest(9),
                verification_profile: "mycelix-electorate-membership-v1".into(),
                verifier_id: "verifier:membership:v1".into(),
                trust_domain_id: "trust-domain:community:v1".into(),
                verified_at_ms: 1_100,
                expires_at_ms: 9_000,
                membership_proof_ref: format!("proof:membership:{name}"),
            },
        }
    }

    #[test]
    fn tally_uses_snapshot_denominator_not_caller_number() {
        let snapshot = snapshot();
        let policy = BindingTallyPolicy {
            quorum_bp: 2_500,
            approval_bp: 5_000,
            absolute_quorum_floor: 5,
        };
        let qualifications = (0..25)
            .map(|index| qualification(&format!("v{index}"), BallotChoice::For, 2_000))
            .collect::<Vec<_>>();
        let result = tally_snapshot_ballots(&snapshot, &policy, &qualifications).unwrap();
        assert_eq!(result.tally.eligible_voters, 100);
        assert_eq!(result.tally.required_voters, 25);
        assert!(result.tally.quorum_reached);
        assert!(result.tally.approved);
    }

    #[test]
    fn membership_from_another_snapshot_is_rejected() {
        let snapshot = snapshot();
        let mut qualification = qualification("alice", BallotChoice::For, 2_000);
        qualification.membership.snapshot_id =
            ElectorateSnapshotId::new("electorate:other:v1").unwrap();
        assert_eq!(
            qualification.validate_against_snapshot(&snapshot).unwrap_err(),
            ElectorateError::SnapshotMismatch
        );
    }

    #[test]
    fn wrong_rulebook_cannot_qualify_ballot() {
        let snapshot = snapshot();
        let mut qualification = qualification("alice", BallotChoice::For, 2_000);
        qualification.permit.rulebook = RulebookRef {
            id: RulebookId::new("rulebook:other").unwrap(),
            version: "1".into(),
            digest: digest(3),
        };
        assert_eq!(
            qualification.validate_against_snapshot(&snapshot).unwrap_err(),
            ElectorateError::RulebookMismatch
        );
    }

    #[test]
    fn expired_membership_evidence_is_rejected() {
        let snapshot = snapshot();
        let mut qualification = qualification("alice", BallotChoice::For, 2_000);
        qualification.membership.expires_at_ms = 2_000;
        assert_eq!(
            qualification.validate_against_snapshot(&snapshot).unwrap_err(),
            ElectorateError::MembershipEvidenceInactive
        );
    }

    #[test]
    fn ballot_outside_snapshot_window_is_rejected() {
        let snapshot = snapshot();
        let qualification = qualification("alice", BallotChoice::For, 10_000);
        assert_eq!(
            qualification.validate_against_snapshot(&snapshot).unwrap_err(),
            ElectorateError::BallotOutsideSnapshotWindow
        );
    }

    #[test]
    fn snapshot_requires_explicit_membership_profile() {
        let mut snapshot = snapshot();
        snapshot.membership_commitment_profile = "".into();
        assert!(matches!(
            snapshot.validate(),
            Err(ElectorateError::InvalidProfile(
                "snapshot.membership_commitment_profile"
            ))
        ));
    }

    #[test]
    fn wire_round_trip_preserves_snapshot_commitments() {
        let snapshot = snapshot();
        let json = serde_json::to_string(&snapshot).unwrap();
        let decoded: ElectorateSnapshot = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, snapshot);
    }
}

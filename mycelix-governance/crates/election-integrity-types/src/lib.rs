//! Public-election integrity foundation for Mycelix governance.
//!
//! This crate deliberately does not implement ballot cryptography. It defines the
//! fail-closed authority, lifecycle, evidence, threat, and certification contracts
//! that a future public-election protocol must satisfy.

use serde::{Deserialize, Serialize};

pub const PUBLIC_ELECTION_PROFILE_ID: &str = "mycelix-public-election-v1";
pub type Digest32 = [u8; 32];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum InfluenceRule {
    EqualWeight,
    ReputationWeighted,
    ConsciousnessWeighted,
    Quadratic,
    Delegated,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PublicElectionAuthorityPolicy {
    pub influence_rule: InfluenceRule,
    pub identity_bearing_ballots: bool,
    pub ballot_reason_text: bool,
    pub administrator_can_finalize_unilaterally: bool,
    pub remote_marked_ballot_transmission: bool,
    pub voter_verifiable_paper_required: bool,
    pub end_to_end_verifiability_required: bool,
}

impl Default for PublicElectionAuthorityPolicy {
    fn default() -> Self {
        Self {
            influence_rule: InfluenceRule::EqualWeight,
            identity_bearing_ballots: false,
            ballot_reason_text: false,
            administrator_can_finalize_unilaterally: false,
            remote_marked_ballot_transmission: false,
            voter_verifiable_paper_required: true,
            end_to_end_verifiability_required: true,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AuthorityPolicyViolation {
    InfluenceMustBeEqualWeight,
    IdentityBearingBallotsForbidden,
    BallotReasonTextForbidden,
    UnilateralAdministratorFinalizationForbidden,
    RemoteMarkedBallotTransmissionForbidden,
    VoterVerifiablePaperRequired,
    EndToEndVerifiabilityRequired,
}

pub fn validate_public_election_authority_policy(
    policy: &PublicElectionAuthorityPolicy,
) -> Result<(), AuthorityPolicyViolation> {
    if policy.influence_rule != InfluenceRule::EqualWeight {
        return Err(AuthorityPolicyViolation::InfluenceMustBeEqualWeight);
    }
    if policy.identity_bearing_ballots {
        return Err(AuthorityPolicyViolation::IdentityBearingBallotsForbidden);
    }
    if policy.ballot_reason_text {
        return Err(AuthorityPolicyViolation::BallotReasonTextForbidden);
    }
    if policy.administrator_can_finalize_unilaterally {
        return Err(AuthorityPolicyViolation::UnilateralAdministratorFinalizationForbidden);
    }
    if policy.remote_marked_ballot_transmission {
        return Err(AuthorityPolicyViolation::RemoteMarkedBallotTransmissionForbidden);
    }
    if !policy.voter_verifiable_paper_required {
        return Err(AuthorityPolicyViolation::VoterVerifiablePaperRequired);
    }
    if !policy.end_to_end_verifiability_required {
        return Err(AuthorityPolicyViolation::EndToEndVerifiabilityRequired);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ElectionConstitutionV1 {
    pub profile_id: String,
    pub election_id: String,
    pub election_definition_digest: Digest32,
    pub jurisdiction_snapshot_digest: Digest32,
    pub eligibility_rules_digest: Digest32,
    pub ballot_definition_digest: Digest32,
    pub trustee_policy_digest: Digest32,
    pub audit_policy_digest: Digest32,
    pub dispute_policy_digest: Digest32,
    pub certification_policy_digest: Digest32,
    pub authority_policy: PublicElectionAuthorityPolicy,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConstitutionViolation {
    WrongProfile,
    EmptyElectionId,
    ZeroElectionDefinitionDigest,
    ZeroJurisdictionSnapshotDigest,
    ZeroEligibilityRulesDigest,
    ZeroBallotDefinitionDigest,
    ZeroTrusteePolicyDigest,
    ZeroAuditPolicyDigest,
    ZeroDisputePolicyDigest,
    ZeroCertificationPolicyDigest,
    AuthorityPolicy(AuthorityPolicyViolation),
}

pub fn validate_election_constitution(
    constitution: &ElectionConstitutionV1,
) -> Result<(), ConstitutionViolation> {
    if constitution.profile_id != PUBLIC_ELECTION_PROFILE_ID {
        return Err(ConstitutionViolation::WrongProfile);
    }
    if constitution.election_id.trim().is_empty() {
        return Err(ConstitutionViolation::EmptyElectionId);
    }

    let zero = [0_u8; 32];
    if constitution.election_definition_digest == zero {
        return Err(ConstitutionViolation::ZeroElectionDefinitionDigest);
    }
    if constitution.jurisdiction_snapshot_digest == zero {
        return Err(ConstitutionViolation::ZeroJurisdictionSnapshotDigest);
    }
    if constitution.eligibility_rules_digest == zero {
        return Err(ConstitutionViolation::ZeroEligibilityRulesDigest);
    }
    if constitution.ballot_definition_digest == zero {
        return Err(ConstitutionViolation::ZeroBallotDefinitionDigest);
    }
    if constitution.trustee_policy_digest == zero {
        return Err(ConstitutionViolation::ZeroTrusteePolicyDigest);
    }
    if constitution.audit_policy_digest == zero {
        return Err(ConstitutionViolation::ZeroAuditPolicyDigest);
    }
    if constitution.dispute_policy_digest == zero {
        return Err(ConstitutionViolation::ZeroDisputePolicyDigest);
    }
    if constitution.certification_policy_digest == zero {
        return Err(ConstitutionViolation::ZeroCertificationPolicyDigest);
    }

    validate_public_election_authority_policy(&constitution.authority_policy)
        .map_err(ConstitutionViolation::AuthorityPolicy)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ElectionPhase {
    Draft,
    Reviewed,
    Frozen,
    TrusteeCeremony,
    CredentialCeremony,
    Voting,
    PollsClosed,
    CryptographicTally,
    PhysicalAudit,
    ChallengeWindow,
    Certified,
    Archived,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransitionEvidence {
    pub threat_model_bound: bool,
    pub constitution_validated: bool,
    pub transparency_checkpoint_present: bool,
    pub trustee_ceremony_complete: bool,
    pub credential_ceremony_complete: bool,
    pub final_ballot_manifest_present: bool,
    pub tally_proof_verified: bool,
    pub physical_audit_passed: bool,
    pub unresolved_qualifying_challenges: bool,
    pub independent_verifier_quorum_passed: bool,
    pub certification_evidence_complete: bool,
    pub archive_package_complete: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransitionViolation {
    InvalidTransition,
    ThreatModelNotBound,
    ConstitutionNotValidated,
    TransparencyCheckpointMissing,
    TrusteeCeremonyIncomplete,
    CredentialCeremonyIncomplete,
    FinalBallotManifestMissing,
    TallyProofNotVerified,
    PhysicalAuditNotPassed,
    UnresolvedQualifyingChallenges,
    IndependentVerifierQuorumNotPassed,
    CertificationEvidenceIncomplete,
    ArchivePackageIncomplete,
}

pub fn validate_transition(
    from: ElectionPhase,
    to: ElectionPhase,
    evidence: &TransitionEvidence,
) -> Result<(), TransitionViolation> {
    match (from, to) {
        (ElectionPhase::Draft, ElectionPhase::Reviewed) => {
            if !evidence.threat_model_bound {
                return Err(TransitionViolation::ThreatModelNotBound);
            }
        }
        (ElectionPhase::Reviewed, ElectionPhase::Frozen) => {
            if !evidence.constitution_validated {
                return Err(TransitionViolation::ConstitutionNotValidated);
            }
        }
        (ElectionPhase::Frozen, ElectionPhase::TrusteeCeremony) => {
            if !evidence.transparency_checkpoint_present {
                return Err(TransitionViolation::TransparencyCheckpointMissing);
            }
        }
        (ElectionPhase::TrusteeCeremony, ElectionPhase::CredentialCeremony) => {
            if !evidence.trustee_ceremony_complete {
                return Err(TransitionViolation::TrusteeCeremonyIncomplete);
            }
        }
        (ElectionPhase::CredentialCeremony, ElectionPhase::Voting) => {
            if !evidence.credential_ceremony_complete {
                return Err(TransitionViolation::CredentialCeremonyIncomplete);
            }
            if !evidence.transparency_checkpoint_present {
                return Err(TransitionViolation::TransparencyCheckpointMissing);
            }
        }
        (ElectionPhase::Voting, ElectionPhase::PollsClosed) => {
            if !evidence.final_ballot_manifest_present {
                return Err(TransitionViolation::FinalBallotManifestMissing);
            }
            if !evidence.transparency_checkpoint_present {
                return Err(TransitionViolation::TransparencyCheckpointMissing);
            }
        }
        (ElectionPhase::PollsClosed, ElectionPhase::CryptographicTally) => {
            if !evidence.final_ballot_manifest_present {
                return Err(TransitionViolation::FinalBallotManifestMissing);
            }
        }
        (ElectionPhase::CryptographicTally, ElectionPhase::PhysicalAudit) => {
            if !evidence.tally_proof_verified {
                return Err(TransitionViolation::TallyProofNotVerified);
            }
        }
        (ElectionPhase::PhysicalAudit, ElectionPhase::ChallengeWindow) => {
            if !evidence.physical_audit_passed {
                return Err(TransitionViolation::PhysicalAuditNotPassed);
            }
        }
        (ElectionPhase::ChallengeWindow, ElectionPhase::Certified) => {
            if evidence.unresolved_qualifying_challenges {
                return Err(TransitionViolation::UnresolvedQualifyingChallenges);
            }
            if !evidence.independent_verifier_quorum_passed {
                return Err(TransitionViolation::IndependentVerifierQuorumNotPassed);
            }
            if !evidence.certification_evidence_complete {
                return Err(TransitionViolation::CertificationEvidenceIncomplete);
            }
        }
        (ElectionPhase::Certified, ElectionPhase::Archived) => {
            if !evidence.archive_package_complete {
                return Err(TransitionViolation::ArchivePackageIncomplete);
            }
        }
        _ => return Err(TransitionViolation::InvalidTransition),
    }

    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AdversaryClass {
    NationState,
    MaliciousElectionOfficial,
    ColludingTrusteeSubset,
    MaliciousVoter,
    CompromisedVotingDevice,
    CompromisedScanner,
    SupplyChainCompromise,
    CompromisedMycelixNodes,
    MaliciousVerifier,
    NetworkPartition,
    Coercer,
    StolenCredential,
}

pub const REQUIRED_ADVERSARY_CLASSES: [AdversaryClass; 12] = [
    AdversaryClass::NationState,
    AdversaryClass::MaliciousElectionOfficial,
    AdversaryClass::ColludingTrusteeSubset,
    AdversaryClass::MaliciousVoter,
    AdversaryClass::CompromisedVotingDevice,
    AdversaryClass::CompromisedScanner,
    AdversaryClass::SupplyChainCompromise,
    AdversaryClass::CompromisedMycelixNodes,
    AdversaryClass::MaliciousVerifier,
    AdversaryClass::NetworkPartition,
    AdversaryClass::Coercer,
    AdversaryClass::StolenCredential,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ElectionEvidenceKind {
    ElectionDefinition,
    JurisdictionSnapshot,
    EligibilityRules,
    BallotDefinition,
    TrusteePolicy,
    TrusteeCeremony,
    CredentialCeremony,
    TransparencyCheckpoint,
    EncryptedBallotRecord,
    PhysicalBallotManifest,
    CustodyTransfer,
    TallyProof,
    AuditSample,
    AuditResult,
    Challenge,
    ChallengeResolution,
    CertificationEvidence,
    ArchivePackage,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ElectionEvidenceRef {
    pub kind: ElectionEvidenceKind,
    pub digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ElectionTheoremId {
    Eligibility,
    Uniqueness,
    BallotSecrecy,
    ReceiptFreeness,
    CastAsIntended,
    RecordedAsCast,
    TalliedAsRecorded,
    SoftwareIndependence,
    AdministrativeNonAuthority,
    EvidenceContinuity,
    RecoverableVerification,
    EvidenceBeforeCertification,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ElectionTheorem {
    pub id: ElectionTheoremId,
    pub statement: &'static str,
    pub required_for_certification: bool,
}

pub const PUBLIC_ELECTION_THEOREMS: [ElectionTheorem; 12] = [
    ElectionTheorem {
        id: ElectionTheoremId::Eligibility,
        statement: "Every counted ballot corresponds to election authority held by an eligible elector.",
        required_for_certification: true,
    },
    ElectionTheorem {
        id: ElectionTheoremId::Uniqueness,
        statement: "No eligible elector contributes more voting authority than the frozen election rules permit.",
        required_for_certification: true,
    },
    ElectionTheorem {
        id: ElectionTheoremId::BallotSecrecy,
        statement: "Published election evidence does not reveal the elector-to-choice relation.",
        required_for_certification: true,
    },
    ElectionTheorem {
        id: ElectionTheoremId::ReceiptFreeness,
        statement: "The protocol does not intentionally create transferable proof of an elector's selections.",
        required_for_certification: true,
    },
    ElectionTheorem {
        id: ElectionTheoremId::CastAsIntended,
        statement: "The protocol supplies evidence that a voter's intended selections were encoded as claimed.",
        required_for_certification: true,
    },
    ElectionTheorem {
        id: ElectionTheoremId::RecordedAsCast,
        statement: "Accepted ballots can be checked for inclusion in the election record without revealing voter choice.",
        required_for_certification: true,
    },
    ElectionTheorem {
        id: ElectionTheoremId::TalliedAsRecorded,
        statement: "The certified tally is independently derivable from the accepted election record.",
        required_for_certification: true,
    },
    ElectionTheorem {
        id: ElectionTheoremId::SoftwareIndependence,
        statement: "A software fault cannot silently produce an undetectably incorrect certified outcome.",
        required_for_certification: true,
    },
    ElectionTheorem {
        id: ElectionTheoremId::AdministrativeNonAuthority,
        statement: "No single election administrator can unilaterally determine or certify the election outcome.",
        required_for_certification: true,
    },
    ElectionTheorem {
        id: ElectionTheoremId::EvidenceContinuity,
        statement: "Required election evidence cannot be silently deleted, substituted, or history-rewritten without detection.",
        required_for_certification: true,
    },
    ElectionTheorem {
        id: ElectionTheoremId::RecoverableVerification,
        statement: "Verification remains possible from preserved evidence even when ordinary Mycelix infrastructure is unavailable.",
        required_for_certification: true,
    },
    ElectionTheorem {
        id: ElectionTheoremId::EvidenceBeforeCertification,
        statement: "Certification authority exists only after the frozen certification evidence requirements are satisfied.",
        required_for_certification: true,
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn valid_constitution() -> ElectionConstitutionV1 {
        ElectionConstitutionV1 {
            profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            election_id: "election-2030-general".to_owned(),
            election_definition_digest: digest(1),
            jurisdiction_snapshot_digest: digest(2),
            eligibility_rules_digest: digest(3),
            ballot_definition_digest: digest(4),
            trustee_policy_digest: digest(5),
            audit_policy_digest: digest(6),
            dispute_policy_digest: digest(7),
            certification_policy_digest: digest(8),
            authority_policy: PublicElectionAuthorityPolicy::default(),
        }
    }

    #[test]
    fn public_profile_rejects_weighted_governance_semantics() {
        let mut policy = PublicElectionAuthorityPolicy::default();
        policy.influence_rule = InfluenceRule::ReputationWeighted;

        assert_eq!(
            validate_public_election_authority_policy(&policy),
            Err(AuthorityPolicyViolation::InfluenceMustBeEqualWeight)
        );
    }

    #[test]
    fn public_profile_rejects_identity_bearing_ballots() {
        let mut policy = PublicElectionAuthorityPolicy::default();
        policy.identity_bearing_ballots = true;

        assert_eq!(
            validate_public_election_authority_policy(&policy),
            Err(AuthorityPolicyViolation::IdentityBearingBallotsForbidden)
        );
    }

    #[test]
    fn constitution_binds_all_required_policy_digests() {
        let constitution = valid_constitution();
        assert_eq!(validate_election_constitution(&constitution), Ok(()));

        let mut broken = constitution;
        broken.audit_policy_digest = [0; 32];
        assert_eq!(
            validate_election_constitution(&broken),
            Err(ConstitutionViolation::ZeroAuditPolicyDigest)
        );
    }

    #[test]
    fn lifecycle_forbids_shortcut_to_certification() {
        let evidence = TransitionEvidence {
            independent_verifier_quorum_passed: true,
            certification_evidence_complete: true,
            ..TransitionEvidence::default()
        };

        assert_eq!(
            validate_transition(ElectionPhase::Voting, ElectionPhase::Certified, &evidence),
            Err(TransitionViolation::InvalidTransition)
        );
    }

    #[test]
    fn certification_fails_closed_on_unresolved_challenge() {
        let evidence = TransitionEvidence {
            unresolved_qualifying_challenges: true,
            independent_verifier_quorum_passed: true,
            certification_evidence_complete: true,
            ..TransitionEvidence::default()
        };

        assert_eq!(
            validate_transition(
                ElectionPhase::ChallengeWindow,
                ElectionPhase::Certified,
                &evidence
            ),
            Err(TransitionViolation::UnresolvedQualifyingChallenges)
        );
    }

    #[test]
    fn certification_requires_independent_verifier_quorum() {
        let evidence = TransitionEvidence {
            certification_evidence_complete: true,
            ..TransitionEvidence::default()
        };

        assert_eq!(
            validate_transition(
                ElectionPhase::ChallengeWindow,
                ElectionPhase::Certified,
                &evidence
            ),
            Err(TransitionViolation::IndependentVerifierQuorumNotPassed)
        );
    }

    #[test]
    fn theorem_registry_is_complete_for_v1() {
        assert_eq!(PUBLIC_ELECTION_THEOREMS.len(), 12);
        assert!(
            PUBLIC_ELECTION_THEOREMS
                .iter()
                .all(|theorem| theorem.required_for_certification)
        );
    }

    #[test]
    fn threat_model_contains_compromised_endpoint_and_coercion_classes() {
        assert!(REQUIRED_ADVERSARY_CLASSES.contains(&AdversaryClass::CompromisedVotingDevice));
        assert!(REQUIRED_ADVERSARY_CLASSES.contains(&AdversaryClass::Coercer));
        assert!(REQUIRED_ADVERSARY_CLASSES.contains(&AdversaryClass::SupplyChainCompromise));
    }
}

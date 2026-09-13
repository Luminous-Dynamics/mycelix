//! Anonymous public-election authority contracts for Mycelix.
//!
//! This crate does not implement a credential scheme or ZK backend. It defines
//! the public statement, privacy requirements, scope-local nullifier contract,
//! and fail-closed duplicate-authority classification that future proof systems
//! must satisfy.

use election_integrity_types::{Digest32, PUBLIC_ELECTION_PROFILE_ID};
use serde::{Deserialize, Serialize};

pub const ANONYMOUS_AUTHORITY_PROFILE_ID: &str = "mycelix-public-election-anonymous-authority-v1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorizationScopeV1 {
    pub election_constitution_digest: Digest32,
    pub election_definition_digest: Digest32,
    pub contest_scope_digest: Digest32,
    pub eligibility_rules_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AuthorizationScopeViolation {
    ZeroElectionConstitutionDigest,
    ZeroElectionDefinitionDigest,
    ZeroContestScopeDigest,
    ZeroEligibilityRulesDigest,
}

pub fn validate_authorization_scope(
    scope: &AuthorizationScopeV1,
) -> Result<(), AuthorizationScopeViolation> {
    let zero = [0_u8; 32];
    if scope.election_constitution_digest == zero {
        return Err(AuthorizationScopeViolation::ZeroElectionConstitutionDigest);
    }
    if scope.election_definition_digest == zero {
        return Err(AuthorizationScopeViolation::ZeroElectionDefinitionDigest);
    }
    if scope.contest_scope_digest == zero {
        return Err(AuthorizationScopeViolation::ZeroContestScopeDigest);
    }
    if scope.eligibility_rules_digest == zero {
        return Err(AuthorizationScopeViolation::ZeroEligibilityRulesDigest);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AnonymousEligibilityPrivacyRequirementsV1 {
    pub hides_civil_identity: bool,
    pub hides_credential_serial: bool,
    pub hides_account_keys: bool,
    pub hides_governance_scores: bool,
    pub unlinkable_across_authorization_scopes: bool,
    pub verifier_to_verifier_unlinkability_required: bool,
    pub issuer_to_verifier_linkage_resistance_required: bool,
    pub revocation_check_must_preserve_ballot_unlinkability: bool,
    pub proof_transcript_must_not_embed_stable_holder_pseudonym: bool,
}

impl Default for AnonymousEligibilityPrivacyRequirementsV1 {
    fn default() -> Self {
        Self {
            hides_civil_identity: true,
            hides_credential_serial: true,
            hides_account_keys: true,
            hides_governance_scores: true,
            unlinkable_across_authorization_scopes: true,
            verifier_to_verifier_unlinkability_required: true,
            issuer_to_verifier_linkage_resistance_required: true,
            revocation_check_must_preserve_ballot_unlinkability: true,
            proof_transcript_must_not_embed_stable_holder_pseudonym: true,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrivacyRequirementViolation {
    CivilIdentityMustBeHidden,
    CredentialSerialMustBeHidden,
    AccountKeysMustBeHidden,
    GovernanceScoresMustBeHidden,
    CrossScopeUnlinkabilityRequired,
    VerifierToVerifierUnlinkabilityRequired,
    IssuerVerifierLinkageResistanceRequired,
    RevocationMustPreserveUnlinkability,
    StableHolderPseudonymForbidden,
}

pub fn validate_privacy_requirements(
    requirements: &AnonymousEligibilityPrivacyRequirementsV1,
) -> Result<(), PrivacyRequirementViolation> {
    if !requirements.hides_civil_identity {
        return Err(PrivacyRequirementViolation::CivilIdentityMustBeHidden);
    }
    if !requirements.hides_credential_serial {
        return Err(PrivacyRequirementViolation::CredentialSerialMustBeHidden);
    }
    if !requirements.hides_account_keys {
        return Err(PrivacyRequirementViolation::AccountKeysMustBeHidden);
    }
    if !requirements.hides_governance_scores {
        return Err(PrivacyRequirementViolation::GovernanceScoresMustBeHidden);
    }
    if !requirements.unlinkable_across_authorization_scopes {
        return Err(PrivacyRequirementViolation::CrossScopeUnlinkabilityRequired);
    }
    if !requirements.verifier_to_verifier_unlinkability_required {
        return Err(PrivacyRequirementViolation::VerifierToVerifierUnlinkabilityRequired);
    }
    if !requirements.issuer_to_verifier_linkage_resistance_required {
        return Err(PrivacyRequirementViolation::IssuerVerifierLinkageResistanceRequired);
    }
    if !requirements.revocation_check_must_preserve_ballot_unlinkability {
        return Err(PrivacyRequirementViolation::RevocationMustPreserveUnlinkability);
    }
    if !requirements.proof_transcript_must_not_embed_stable_holder_pseudonym {
        return Err(PrivacyRequirementViolation::StableHolderPseudonymForbidden);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AnonymousEligibilityPublicStatementV1 {
    pub public_election_profile_id: String,
    pub anonymous_authority_profile_id: String,
    pub scope: AuthorizationScopeV1,
    pub credential_issuer_policy_digest: Digest32,
    pub credential_schema_digest: Digest32,
    pub revocation_snapshot_digest: Digest32,
    pub proof_system_profile_digest: Digest32,
    pub privacy_policy_digest: Digest32,
    pub nullifier_derivation_profile_digest: Digest32,
    pub scope_local_nullifier: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnonymousEligibilityStatementViolation {
    WrongPublicElectionProfile,
    WrongAnonymousAuthorityProfile,
    Scope(AuthorizationScopeViolation),
    ZeroCredentialIssuerPolicyDigest,
    ZeroCredentialSchemaDigest,
    ZeroRevocationSnapshotDigest,
    ZeroProofSystemProfileDigest,
    ZeroPrivacyPolicyDigest,
    ZeroNullifierDerivationProfileDigest,
    ZeroNullifier,
}

pub fn validate_anonymous_eligibility_statement(
    statement: &AnonymousEligibilityPublicStatementV1,
) -> Result<(), AnonymousEligibilityStatementViolation> {
    if statement.public_election_profile_id != PUBLIC_ELECTION_PROFILE_ID {
        return Err(AnonymousEligibilityStatementViolation::WrongPublicElectionProfile);
    }
    if statement.anonymous_authority_profile_id != ANONYMOUS_AUTHORITY_PROFILE_ID {
        return Err(AnonymousEligibilityStatementViolation::WrongAnonymousAuthorityProfile);
    }
    validate_authorization_scope(&statement.scope)
        .map_err(AnonymousEligibilityStatementViolation::Scope)?;

    let zero = [0_u8; 32];
    if statement.credential_issuer_policy_digest == zero {
        return Err(AnonymousEligibilityStatementViolation::ZeroCredentialIssuerPolicyDigest);
    }
    if statement.credential_schema_digest == zero {
        return Err(AnonymousEligibilityStatementViolation::ZeroCredentialSchemaDigest);
    }
    if statement.revocation_snapshot_digest == zero {
        return Err(AnonymousEligibilityStatementViolation::ZeroRevocationSnapshotDigest);
    }
    if statement.proof_system_profile_digest == zero {
        return Err(AnonymousEligibilityStatementViolation::ZeroProofSystemProfileDigest);
    }
    if statement.privacy_policy_digest == zero {
        return Err(AnonymousEligibilityStatementViolation::ZeroPrivacyPolicyDigest);
    }
    if statement.nullifier_derivation_profile_digest == zero {
        return Err(AnonymousEligibilityStatementViolation::ZeroNullifierDerivationProfileDigest);
    }
    if statement.scope_local_nullifier == zero {
        return Err(AnonymousEligibilityStatementViolation::ZeroNullifier);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NullifierSecurityPropertyId {
    DeterministicWithinScope,
    DomainSeparatedByScope,
    UnlinkableAcrossScopes,
    OneWayFromPublicTranscript,
    BoundToEligibleCredentialWitness,
    StableIdentityNotEncoded,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NullifierSecurityProperty {
    pub id: NullifierSecurityPropertyId,
    pub statement: &'static str,
}

pub const REQUIRED_NULLIFIER_SECURITY_PROPERTIES: [NullifierSecurityProperty; 6] = [
    NullifierSecurityProperty {
        id: NullifierSecurityPropertyId::DeterministicWithinScope,
        statement: "The same eligible secret authority presented for the same frozen authorization scope derives the same nullifier.",
    },
    NullifierSecurityProperty {
        id: NullifierSecurityPropertyId::DomainSeparatedByScope,
        statement: "Nullifier derivation is cryptographically domain-separated by the exact frozen authorization scope.",
    },
    NullifierSecurityProperty {
        id: NullifierSecurityPropertyId::UnlinkableAcrossScopes,
        statement: "Nullifiers derived from the same private authority for different authorization scopes are not publicly linkable.",
    },
    NullifierSecurityProperty {
        id: NullifierSecurityPropertyId::OneWayFromPublicTranscript,
        statement: "The public nullifier and proof transcript do not reveal the private authority witness used to derive them.",
    },
    NullifierSecurityProperty {
        id: NullifierSecurityPropertyId::BoundToEligibleCredentialWitness,
        statement: "A valid nullifier proof is bound to the same hidden credential witness that satisfies eligibility.",
    },
    NullifierSecurityProperty {
        id: NullifierSecurityPropertyId::StableIdentityNotEncoded,
        statement: "The nullifier does not encode a stable civil, account, device, or cross-election pseudonymous identity.",
    },
];

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ScopedAuthorityClaimV1 {
    pub scope: AuthorizationScopeV1,
    pub scope_local_nullifier: Digest32,
    pub ballot_commitment_digest: Digest32,
    pub anonymous_eligibility_proof_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScopedAuthorityClaimViolation {
    Scope(AuthorizationScopeViolation),
    ZeroNullifier,
    ZeroBallotCommitmentDigest,
    ZeroEligibilityProofDigest,
}

pub fn validate_scoped_authority_claim(
    claim: &ScopedAuthorityClaimV1,
) -> Result<(), ScopedAuthorityClaimViolation> {
    validate_authorization_scope(&claim.scope).map_err(ScopedAuthorityClaimViolation::Scope)?;
    let zero = [0_u8; 32];
    if claim.scope_local_nullifier == zero {
        return Err(ScopedAuthorityClaimViolation::ZeroNullifier);
    }
    if claim.ballot_commitment_digest == zero {
        return Err(ScopedAuthorityClaimViolation::ZeroBallotCommitmentDigest);
    }
    if claim.anonymous_eligibility_proof_digest == zero {
        return Err(ScopedAuthorityClaimViolation::ZeroEligibilityProofDigest);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PairwiseAuthorityClassification {
    DistinctScope,
    DistinctAuthority,
    IdempotentReplay,
    ConflictingUseOfSameAuthority,
}

pub fn classify_pairwise_authority(
    accepted: &ScopedAuthorityClaimV1,
    incoming: &ScopedAuthorityClaimV1,
) -> PairwiseAuthorityClassification {
    if accepted.scope != incoming.scope {
        return PairwiseAuthorityClassification::DistinctScope;
    }
    if accepted.scope_local_nullifier != incoming.scope_local_nullifier {
        return PairwiseAuthorityClassification::DistinctAuthority;
    }
    if accepted.ballot_commitment_digest == incoming.ballot_commitment_digest
        && accepted.anonymous_eligibility_proof_digest
            == incoming.anonymous_eligibility_proof_digest
    {
        return PairwiseAuthorityClassification::IdempotentReplay;
    }
    PairwiseAuthorityClassification::ConflictingUseOfSameAuthority
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum DuplicateAuthorityPolicyV1 {
    #[default]
    FreezePendingEvidenceResolution,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NullifierCensusV1 {
    pub scope: AuthorizationScopeV1,
    pub complete_checkpoint_digest: Digest32,
    pub observed_claim_count: u64,
    pub unique_nullifier_count: u64,
    pub conflicting_nullifier_count: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NullifierCensusViolation {
    Scope(AuthorizationScopeViolation),
    ZeroCheckpointDigest,
    ConflictingAuthorityObserved,
    CountMismatch,
}

pub fn validate_nullifier_census_for_tally(
    census: &NullifierCensusV1,
) -> Result<(), NullifierCensusViolation> {
    validate_authorization_scope(&census.scope).map_err(NullifierCensusViolation::Scope)?;
    if census.complete_checkpoint_digest == [0_u8; 32] {
        return Err(NullifierCensusViolation::ZeroCheckpointDigest);
    }
    if census.conflicting_nullifier_count != 0 {
        return Err(NullifierCensusViolation::ConflictingAuthorityObserved);
    }
    if census.observed_claim_count != census.unique_nullifier_count {
        return Err(NullifierCensusViolation::CountMismatch);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn scope(contest: u8) -> AuthorizationScopeV1 {
        AuthorizationScopeV1 {
            election_constitution_digest: digest(1),
            election_definition_digest: digest(2),
            contest_scope_digest: digest(contest),
            eligibility_rules_digest: digest(4),
        }
    }

    fn statement() -> AnonymousEligibilityPublicStatementV1 {
        AnonymousEligibilityPublicStatementV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            anonymous_authority_profile_id: ANONYMOUS_AUTHORITY_PROFILE_ID.to_owned(),
            scope: scope(3),
            credential_issuer_policy_digest: digest(5),
            credential_schema_digest: digest(6),
            revocation_snapshot_digest: digest(7),
            proof_system_profile_digest: digest(8),
            privacy_policy_digest: digest(9),
            nullifier_derivation_profile_digest: digest(10),
            scope_local_nullifier: digest(11),
        }
    }

    fn claim(scope_value: AuthorizationScopeV1, nullifier: u8, ballot: u8, proof: u8) -> ScopedAuthorityClaimV1 {
        ScopedAuthorityClaimV1 {
            scope: scope_value,
            scope_local_nullifier: digest(nullifier),
            ballot_commitment_digest: digest(ballot),
            anonymous_eligibility_proof_digest: digest(proof),
        }
    }

    #[test]
    fn anonymous_statement_has_no_public_identity_slot() {
        let candidate = statement();
        assert_eq!(validate_anonymous_eligibility_statement(&candidate), Ok(()));
    }

    #[test]
    fn public_statement_binds_frozen_revocation_and_proof_profiles() {
        let mut candidate = statement();
        candidate.revocation_snapshot_digest = [0; 32];
        assert_eq!(
            validate_anonymous_eligibility_statement(&candidate),
            Err(AnonymousEligibilityStatementViolation::ZeroRevocationSnapshotDigest)
        );
    }

    #[test]
    fn privacy_requirements_fail_closed() {
        let mut requirements = AnonymousEligibilityPrivacyRequirementsV1::default();
        requirements.unlinkable_across_authorization_scopes = false;
        assert_eq!(
            validate_privacy_requirements(&requirements),
            Err(PrivacyRequirementViolation::CrossScopeUnlinkabilityRequired)
        );
    }

    #[test]
    fn exact_replay_is_not_a_second_vote() {
        let first = claim(scope(3), 11, 12, 13);
        let replay = first.clone();
        assert_eq!(
            classify_pairwise_authority(&first, &replay),
            PairwiseAuthorityClassification::IdempotentReplay
        );
    }

    #[test]
    fn same_nullifier_with_changed_ballot_is_a_conflict_not_first_arrival_authority() {
        let first = claim(scope(3), 11, 12, 13);
        let conflicting = claim(scope(3), 11, 99, 14);
        assert_eq!(
            classify_pairwise_authority(&first, &conflicting),
            PairwiseAuthorityClassification::ConflictingUseOfSameAuthority
        );
        assert_eq!(
            DuplicateAuthorityPolicyV1::default(),
            DuplicateAuthorityPolicyV1::FreezePendingEvidenceResolution
        );
    }

    #[test]
    fn same_private_authority_must_not_be_linked_by_cross_scope_equality() {
        let first_scope = claim(scope(3), 11, 12, 13);
        let second_scope = claim(scope(8), 44, 15, 16);
        assert_eq!(
            classify_pairwise_authority(&first_scope, &second_scope),
            PairwiseAuthorityClassification::DistinctScope
        );
    }

    #[test]
    fn tally_census_rejects_any_unresolved_authority_conflict() {
        let census = NullifierCensusV1 {
            scope: scope(3),
            complete_checkpoint_digest: digest(20),
            observed_claim_count: 101,
            unique_nullifier_count: 100,
            conflicting_nullifier_count: 1,
        };
        assert_eq!(
            validate_nullifier_census_for_tally(&census),
            Err(NullifierCensusViolation::ConflictingAuthorityObserved)
        );
    }

    #[test]
    fn tally_census_requires_exact_uniqueness_count() {
        let census = NullifierCensusV1 {
            scope: scope(3),
            complete_checkpoint_digest: digest(20),
            observed_claim_count: 100,
            unique_nullifier_count: 99,
            conflicting_nullifier_count: 0,
        };
        assert_eq!(
            validate_nullifier_census_for_tally(&census),
            Err(NullifierCensusViolation::CountMismatch)
        );
    }

    #[test]
    fn nullifier_property_registry_is_complete_for_v1() {
        assert_eq!(REQUIRED_NULLIFIER_SECURITY_PROPERTIES.len(), 6);
    }
}

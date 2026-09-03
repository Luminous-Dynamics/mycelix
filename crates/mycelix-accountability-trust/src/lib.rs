// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Trust-aware verification for reciprocal-accountability evidence.
//!
//! The core receipt records *what* evidence roles are attached. The provider-neutral
//! verifier proves that each referenced artifact is cryptographically valid. This
//! crate adds a third question: are enough of those valid proofs controlled by
//! sufficiently independent verifier roots and trust domains?
//!
//! Trust identity is returned by the verifier *after* proof verification. It is not
//! accepted from caller-controlled `scheme` or `verifier_profile` strings.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Display;

use mycelix_accountability_core::{
    AccessReceipt, AccountabilityPolicy, AttestationRef, AttestationRole, Commitment32,
    NotificationDisposition, evaluate_notification, policy_commitment, validate_receipt,
};
use mycelix_accountability_verifier::{
    AttestationVerificationContext, VerificationError, verification_context,
};
use thiserror::Error;

const TRUST_POLICY_DOMAIN: &[u8] = b"mycelix:accountability-attestation-trust-policy:v1";
const TRUST_POLICY_CODEC: &[u8] = b"mycelix-accountability-trust-policy-v1";

/// Cryptographically resolved authority identity for one verified proof.
///
/// `verifier_id` should identify the concrete trusted verifier/key root. A provider
/// may derive it from a public-key fingerprint, hardware-attestation root, proof
/// verifier key, or another stable cryptographic authority identifier.
///
/// `trust_domain_id` identifies the independently administered trust domain to
/// which that verifier belongs. Production adapters MUST derive this from trusted
/// configuration/credentials/governance state, not an untrusted proof label.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct VerifiedAttestationAuthority {
    pub verifier_id: Commitment32,
    pub trust_domain_id: Commitment32,
}

/// Evidence provider that both verifies a proof and resolves its trusted authority.
pub trait TrustAwareAttestationVerifier {
    type Error: Display;

    fn verify_with_authority(
        &self,
        attestation: &AttestationRef,
        expected: &AttestationVerificationContext,
    ) -> Result<VerifiedAttestationAuthority, Self::Error>;
}

/// Minimum independent evidence required for one semantic proof role.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct RoleTrustRequirement {
    pub role: AttestationRole,
    pub min_verified_attestations: u8,
    pub min_distinct_verifiers: u8,
    pub min_distinct_trust_domains: u8,
}

/// Cross-role separation requirement.
///
/// For example, a high-assurance profile can require `ExecutionBinding` and
/// `ExternalWitness` to have disjoint verifier roots and disjoint trust domains so
/// the same operator organization cannot witness its own pre-disclosure claim.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct CrossRoleIndependence {
    pub left: AttestationRole,
    pub right: AttestationRole,
    pub require_distinct_verifiers: bool,
    pub require_distinct_trust_domains: bool,
}

/// Runtime trust composition required before protected output may be released.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AttestationTrustPolicy {
    pub role_requirements: Vec<RoleTrustRequirement>,
    pub cross_role_independence: Vec<CrossRoleIndependence>,
}

/// One cryptographically verified evidence artifact and its resolved authority.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifiedEvidenceIdentity {
    pub role: AttestationRole,
    pub proof_digest: Commitment32,
    pub verifier_id: Commitment32,
    pub trust_domain_id: Commitment32,
}

/// Receipt snapshot that passed structural, cryptographic, and trust-composition
/// verification under one exact accountability policy and one exact trust policy.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TrustVerifiedReceipt {
    receipt: AccessReceipt,
    verification_context: AttestationVerificationContext,
    evidence: Vec<VerifiedEvidenceIdentity>,
    accountability_policy_digest: Commitment32,
    trust_policy_digest: Commitment32,
}

impl TrustVerifiedReceipt {
    pub fn receipt(&self) -> &AccessReceipt {
        &self.receipt
    }

    pub fn verification_context(&self) -> &AttestationVerificationContext {
        &self.verification_context
    }

    pub fn evidence(&self) -> &[VerifiedEvidenceIdentity] {
        &self.evidence
    }

    pub const fn accountability_policy_digest(&self) -> Commitment32 {
        self.accountability_policy_digest
    }

    pub const fn trust_policy_digest(&self) -> Commitment32 {
        self.trust_policy_digest
    }

    /// Evaluate notice semantics only under the exact accountability policy that
    /// was present at trust verification time.
    pub fn evaluate_notification(
        &self,
        now_ms: u64,
        policy: &AccountabilityPolicy,
    ) -> Result<NotificationDisposition, TrustVerificationError> {
        let digest = policy_commitment(policy)
            .map_err(|_| TrustVerificationError::CommitmentEncoding)?;
        if digest != self.accountability_policy_digest {
            return Err(TrustVerificationError::AccountabilityPolicyChanged);
        }
        evaluate_notification(&self.receipt, now_ms, policy)
            .map_err(|error| TrustVerificationError::BaseVerification(
                VerificationError::Structural(error),
            ))
    }

    /// Fail closed when a caller claims this snapshot was qualified under a
    /// different evidence-trust policy.
    pub fn require_trust_policy(
        &self,
        policy: &AttestationTrustPolicy,
    ) -> Result<(), TrustVerificationError> {
        let digest = attestation_trust_policy_commitment(policy)?;
        if digest != self.trust_policy_digest {
            return Err(TrustVerificationError::TrustPolicyChanged);
        }
        Ok(())
    }
}

/// Fail-closed errors from trust-aware proof composition.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum TrustVerificationError {
    #[error("base accountability verification failed: {0}")]
    BaseVerification(VerificationError),
    #[error("failed to compute accountability/trust policy commitment")]
    CommitmentEncoding,
    #[error("attestation trust policy has no role requirements")]
    EmptyTrustPolicy,
    #[error("trust requirement for {role:?} has a zero threshold")]
    ZeroTrustThreshold { role: AttestationRole },
    #[error("trust requirement for {role:?} has impossible distinctness thresholds")]
    ImpossibleTrustThreshold { role: AttestationRole },
    #[error("trust policy repeats a requirement for {role:?}")]
    DuplicateRoleRequirement { role: AttestationRole },
    #[error("cross-role independence cannot compare a role with itself: {role:?}")]
    SelfIndependenceRequirement { role: AttestationRole },
    #[error("cross-role independence references a role without a trust requirement: {role:?}")]
    IndependenceRoleNotRequired { role: AttestationRole },
    #[error("cross-role independence requirement is duplicated")]
    DuplicateIndependenceRequirement,
    #[error("{role:?} attestation is bound to a different receipt statement")]
    StatementMismatch { role: AttestationRole },
    #[error("{role:?} attestation failed cryptographic verification: {reason}")]
    CryptographicVerificationFailed {
        role: AttestationRole,
        reason: String,
    },
    #[error("{role:?} verifier returned an all-zero verifier identity")]
    ZeroVerifierId { role: AttestationRole },
    #[error("{role:?} verifier returned an all-zero trust-domain identity")]
    ZeroTrustDomainId { role: AttestationRole },
    #[error("the same proof digest was attached more than once")]
    DuplicateProofDigest,
    #[error("{role:?} has {found} verified attestations but requires {required}")]
    InsufficientVerifiedAttestations {
        role: AttestationRole,
        found: usize,
        required: usize,
    },
    #[error("{role:?} has {found} distinct verifier roots but requires {required}")]
    InsufficientDistinctVerifiers {
        role: AttestationRole,
        found: usize,
        required: usize,
    },
    #[error("{role:?} has {found} distinct trust domains but requires {required}")]
    InsufficientDistinctTrustDomains {
        role: AttestationRole,
        found: usize,
        required: usize,
    },
    #[error("{left:?} and {right:?} share a verifier root despite independence policy")]
    CrossRoleVerifierOverlap {
        left: AttestationRole,
        right: AttestationRole,
    },
    #[error("{left:?} and {right:?} share a trust domain despite independence policy")]
    CrossRoleTrustDomainOverlap {
        left: AttestationRole,
        right: AttestationRole,
    },
    #[error("accountability policy changed after trust verification")]
    AccountabilityPolicyChanged,
    #[error("attestation trust policy changed after trust verification")]
    TrustPolicyChanged,
}

/// Cryptographically verify every evidence artifact, resolve its trusted authority,
/// and enforce independent verifier/trust-domain composition before returning a
/// disclosure-safe snapshot.
pub fn verify_finalized_receipt_with_trust<V: TrustAwareAttestationVerifier>(
    receipt: &AccessReceipt,
    accountability_policy: &AccountabilityPolicy,
    trust_policy: &AttestationTrustPolicy,
    verifier: &V,
) -> Result<TrustVerifiedReceipt, TrustVerificationError> {
    validate_trust_policy(trust_policy)?;
    validate_receipt(receipt, accountability_policy).map_err(|error| {
        TrustVerificationError::BaseVerification(VerificationError::Structural(error))
    })?;

    let expected = verification_context(receipt, accountability_policy)
        .map_err(TrustVerificationError::BaseVerification)?;
    let mut evidence = Vec::with_capacity(receipt.attestations.len());
    let mut proof_digests = BTreeSet::new();

    for attestation in &receipt.attestations {
        if attestation.statement_digest != expected.statement_digest {
            return Err(TrustVerificationError::StatementMismatch {
                role: attestation.role,
            });
        }
        if !proof_digests.insert(attestation.proof_digest) {
            return Err(TrustVerificationError::DuplicateProofDigest);
        }

        let authority = verifier
            .verify_with_authority(attestation, &expected)
            .map_err(|error| TrustVerificationError::CryptographicVerificationFailed {
                role: attestation.role,
                reason: error.to_string(),
            })?;

        if authority.verifier_id.is_zero() {
            return Err(TrustVerificationError::ZeroVerifierId {
                role: attestation.role,
            });
        }
        if authority.trust_domain_id.is_zero() {
            return Err(TrustVerificationError::ZeroTrustDomainId {
                role: attestation.role,
            });
        }

        evidence.push(VerifiedEvidenceIdentity {
            role: attestation.role,
            proof_digest: attestation.proof_digest,
            verifier_id: authority.verifier_id,
            trust_domain_id: authority.trust_domain_id,
        });
    }

    enforce_trust_policy(&evidence, trust_policy)?;

    let accountability_policy_digest = policy_commitment(accountability_policy)
        .map_err(|_| TrustVerificationError::CommitmentEncoding)?;
    let trust_policy_digest = attestation_trust_policy_commitment(trust_policy)?;

    Ok(TrustVerifiedReceipt {
        receipt: receipt.clone(),
        verification_context: expected,
        evidence,
        accountability_policy_digest,
        trust_policy_digest,
    })
}

/// Stable commitment to the evidence-trust policy used at verification time.
///
/// Requirements and role-pairs are sorted before encoding, so insertion order does
/// not change the commitment. This commitment identifies runtime trust semantics;
/// it is intentionally separate from the semantic receipt commitment.
pub fn attestation_trust_policy_commitment(
    policy: &AttestationTrustPolicy,
) -> Result<Commitment32, TrustVerificationError> {
    validate_trust_policy(policy)?;

    let mut requirements = policy.role_requirements.clone();
    requirements.sort_unstable();

    let mut independence: Vec<_> = policy
        .cross_role_independence
        .iter()
        .copied()
        .map(normalize_independence)
        .collect();
    independence.sort_unstable();

    let mut hasher = blake3::Hasher::new();
    hasher.update(TRUST_POLICY_DOMAIN);
    hasher.update(&[0]);
    hasher.update(TRUST_POLICY_CODEC);
    hasher.update(&[0]);
    hasher.update(&(requirements.len() as u64).to_be_bytes());
    for requirement in requirements {
        hasher.update(&[role_tag(requirement.role)]);
        hasher.update(&[requirement.min_verified_attestations]);
        hasher.update(&[requirement.min_distinct_verifiers]);
        hasher.update(&[requirement.min_distinct_trust_domains]);
    }
    hasher.update(&(independence.len() as u64).to_be_bytes());
    for item in independence {
        hasher.update(&[role_tag(item.left)]);
        hasher.update(&[role_tag(item.right)]);
        hasher.update(&[u8::from(item.require_distinct_verifiers)]);
        hasher.update(&[u8::from(item.require_distinct_trust_domains)]);
    }
    Ok(Commitment32(*hasher.finalize().as_bytes()))
}

fn validate_trust_policy(policy: &AttestationTrustPolicy) -> Result<(), TrustVerificationError> {
    if policy.role_requirements.is_empty() {
        return Err(TrustVerificationError::EmptyTrustPolicy);
    }

    let mut required_roles = BTreeSet::new();
    for requirement in &policy.role_requirements {
        if requirement.min_verified_attestations == 0
            || requirement.min_distinct_verifiers == 0
            || requirement.min_distinct_trust_domains == 0
        {
            return Err(TrustVerificationError::ZeroTrustThreshold {
                role: requirement.role,
            });
        }
        if requirement.min_distinct_verifiers > requirement.min_verified_attestations
            || requirement.min_distinct_trust_domains > requirement.min_verified_attestations
        {
            return Err(TrustVerificationError::ImpossibleTrustThreshold {
                role: requirement.role,
            });
        }
        if !required_roles.insert(requirement.role) {
            return Err(TrustVerificationError::DuplicateRoleRequirement {
                role: requirement.role,
            });
        }
    }

    let mut independence_pairs = BTreeSet::new();
    for item in &policy.cross_role_independence {
        if item.left == item.right {
            return Err(TrustVerificationError::SelfIndependenceRequirement {
                role: item.left,
            });
        }
        if !required_roles.contains(&item.left) {
            return Err(TrustVerificationError::IndependenceRoleNotRequired {
                role: item.left,
            });
        }
        if !required_roles.contains(&item.right) {
            return Err(TrustVerificationError::IndependenceRoleNotRequired {
                role: item.right,
            });
        }
        if !item.require_distinct_verifiers && !item.require_distinct_trust_domains {
            return Err(TrustVerificationError::DuplicateIndependenceRequirement);
        }
        if !independence_pairs.insert(normalize_independence(*item)) {
            return Err(TrustVerificationError::DuplicateIndependenceRequirement);
        }
    }

    Ok(())
}

fn enforce_trust_policy(
    evidence: &[VerifiedEvidenceIdentity],
    policy: &AttestationTrustPolicy,
) -> Result<(), TrustVerificationError> {
    let mut by_role: BTreeMap<AttestationRole, Vec<&VerifiedEvidenceIdentity>> = BTreeMap::new();
    for item in evidence {
        by_role.entry(item.role).or_default().push(item);
    }

    for requirement in &policy.role_requirements {
        let items = by_role.get(&requirement.role).map(Vec::as_slice).unwrap_or(&[]);
        if items.len() < usize::from(requirement.min_verified_attestations) {
            return Err(TrustVerificationError::InsufficientVerifiedAttestations {
                role: requirement.role,
                found: items.len(),
                required: usize::from(requirement.min_verified_attestations),
            });
        }
        let verifiers: BTreeSet<_> = items.iter().map(|item| item.verifier_id).collect();
        if verifiers.len() < usize::from(requirement.min_distinct_verifiers) {
            return Err(TrustVerificationError::InsufficientDistinctVerifiers {
                role: requirement.role,
                found: verifiers.len(),
                required: usize::from(requirement.min_distinct_verifiers),
            });
        }
        let domains: BTreeSet<_> = items.iter().map(|item| item.trust_domain_id).collect();
        if domains.len() < usize::from(requirement.min_distinct_trust_domains) {
            return Err(TrustVerificationError::InsufficientDistinctTrustDomains {
                role: requirement.role,
                found: domains.len(),
                required: usize::from(requirement.min_distinct_trust_domains),
            });
        }
    }

    for requirement in &policy.cross_role_independence {
        let left_items = by_role.get(&requirement.left).map(Vec::as_slice).unwrap_or(&[]);
        let right_items = by_role.get(&requirement.right).map(Vec::as_slice).unwrap_or(&[]);

        if requirement.require_distinct_verifiers {
            let left: BTreeSet<_> = left_items.iter().map(|item| item.verifier_id).collect();
            let right: BTreeSet<_> = right_items.iter().map(|item| item.verifier_id).collect();
            if !left.is_disjoint(&right) {
                return Err(TrustVerificationError::CrossRoleVerifierOverlap {
                    left: requirement.left,
                    right: requirement.right,
                });
            }
        }

        if requirement.require_distinct_trust_domains {
            let left: BTreeSet<_> = left_items.iter().map(|item| item.trust_domain_id).collect();
            let right: BTreeSet<_> = right_items.iter().map(|item| item.trust_domain_id).collect();
            if !left.is_disjoint(&right) {
                return Err(TrustVerificationError::CrossRoleTrustDomainOverlap {
                    left: requirement.left,
                    right: requirement.right,
                });
            }
        }
    }

    Ok(())
}

fn normalize_independence(mut item: CrossRoleIndependence) -> CrossRoleIndependence {
    if role_tag(item.left) > role_tag(item.right) {
        std::mem::swap(&mut item.left, &mut item.right);
    }
    item
}

const fn role_tag(role: AttestationRole) -> u8 {
    match role {
        AttestationRole::ExecutionBinding => 0,
        AttestationRole::ComputationProof => 1,
        AttestationRole::PolicyProof => 2,
        AttestationRole::ExternalWitness => 3,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_accountability_core::{
        AuthorityType, DisclosureKind, DisclosureSummary, LegalAuthority, LookupOutcome,
        NotificationDirective, PairwiseSubjectId, PurposeBinding, QueryBudgetCharge,
        RECIPROCAL_ACCOUNTABILITY_PROTOCOL_VERSION, RequestingPrincipal, SubjectRight,
    };

    const DAY_MS: u64 = 24 * 60 * 60 * 1000;

    fn c(byte: u8) -> Commitment32 {
        Commitment32([byte; 32])
    }

    fn rights() -> Vec<SubjectRight> {
        vec![
            SubjectRight::Know,
            SubjectRight::Inspect,
            SubjectRight::Contest,
            SubjectRight::HumanReview,
            SubjectRight::Appeal,
            SubjectRight::ProofOfPolicy,
        ]
    }

    fn accountability_policy() -> AccountabilityPolicy {
        AccountabilityPolicy {
            max_delay_ms: 0,
            min_delay_approvals: 0,
            require_approval_outside_requester_org: false,
            permitted_delay_reasons: Vec::new(),
            required_subject_rights: rights(),
            require_query_budget_charge: true,
            required_attestation_roles: vec![
                AttestationRole::ExecutionBinding,
                AttestationRole::ExternalWitness,
            ],
        }
    }

    fn receipt() -> AccessReceipt {
        let occurred_at_ms = 1_800_000_000_000;
        AccessReceipt {
            protocol_version: RECIPROCAL_ACCOUNTABILITY_PROTOCOL_VERSION,
            receipt_id: "trust-receipt-1".into(),
            subject: PairwiseSubjectId("pairwise:trust-subject".into()),
            requester: RequestingPrincipal {
                organization_id: "agency-a".into(),
                actor_id: "operator-a".into(),
                role: "investigator".into(),
                authenticated_source_id: c(1),
            },
            purpose: PurposeBinding {
                purpose_code: "bounded-test".into(),
                plain_language_purpose: "Test independent evidence composition".into(),
                matter_id: None,
                scope_digest: c(2),
                expires_at_ms: occurred_at_ms + DAY_MS,
            },
            legal_authority: LegalAuthority {
                authority_type: AuthorityType::Consent,
                authority_id: "consent-1".into(),
                jurisdiction: "test".into(),
            },
            query_digest: c(3),
            policy_version: "policy-v1".into(),
            occurred_at_ms,
            outcome: LookupOutcome::Allowed,
            disclosure: DisclosureSummary {
                kind: DisclosureKind::PredicateOnly,
                data_classes: vec!["predicate".into()],
                item_count: 1,
                result_digest: Some(c(4)),
            },
            notification: NotificationDirective::Immediate,
            rights: rights(),
            query_budget_charge: Some(QueryBudgetCharge {
                budget_id: "budget-1".into(),
                units: 1,
                remaining_units: 9,
                window_ends_at_ms: occurred_at_ms + DAY_MS,
            }),
            inference: None,
            attestations: Vec::new(),
        }
    }

    fn attestation(
        role: AttestationRole,
        statement: Commitment32,
        proof_byte: u8,
    ) -> AttestationRef {
        AttestationRef {
            role,
            scheme: "test-proof-v1".into(),
            statement_digest: statement,
            proof_digest: c(proof_byte),
            verifier_profile: "caller-visible-profile-is-not-trust-identity".into(),
        }
    }

    fn finalized_receipt() -> AccessReceipt {
        let mut receipt = receipt();
        let statement = verification_context(&receipt, &accountability_policy())
            .unwrap()
            .statement_digest;
        receipt.attestations = vec![
            attestation(AttestationRole::ExecutionBinding, statement, 50),
            attestation(AttestationRole::ExternalWitness, statement, 60),
        ];
        receipt
    }

    fn high_assurance_policy() -> AttestationTrustPolicy {
        AttestationTrustPolicy {
            role_requirements: vec![
                RoleTrustRequirement {
                    role: AttestationRole::ExecutionBinding,
                    min_verified_attestations: 1,
                    min_distinct_verifiers: 1,
                    min_distinct_trust_domains: 1,
                },
                RoleTrustRequirement {
                    role: AttestationRole::ExternalWitness,
                    min_verified_attestations: 1,
                    min_distinct_verifiers: 1,
                    min_distinct_trust_domains: 1,
                },
            ],
            cross_role_independence: vec![CrossRoleIndependence {
                left: AttestationRole::ExecutionBinding,
                right: AttestationRole::ExternalWitness,
                require_distinct_verifiers: true,
                require_distinct_trust_domains: true,
            }],
        }
    }

    struct TestVerifier {
        witness_domain: Commitment32,
        execution_verifier: Commitment32,
    }

    impl TrustAwareAttestationVerifier for TestVerifier {
        type Error = &'static str;

        fn verify_with_authority(
            &self,
            attestation: &AttestationRef,
            expected: &AttestationVerificationContext,
        ) -> Result<VerifiedAttestationAuthority, Self::Error> {
            if attestation.scheme != "test-proof-v1" {
                return Err("unsupported scheme");
            }
            if attestation.statement_digest != expected.statement_digest {
                return Err("wrong statement");
            }
            match attestation.role {
                AttestationRole::ExecutionBinding => Ok(VerifiedAttestationAuthority {
                    verifier_id: self.execution_verifier,
                    trust_domain_id: c(20),
                }),
                AttestationRole::ExternalWitness => Ok(VerifiedAttestationAuthority {
                    verifier_id: c(30),
                    trust_domain_id: self.witness_domain,
                }),
                AttestationRole::ComputationProof => Ok(VerifiedAttestationAuthority {
                    verifier_id: c(attestation.proof_digest.0[0]),
                    trust_domain_id: c(40),
                }),
                AttestationRole::PolicyProof => Ok(VerifiedAttestationAuthority {
                    verifier_id: c(50),
                    trust_domain_id: c(50),
                }),
            }
        }
    }

    fn verifier() -> TestVerifier {
        TestVerifier {
            witness_domain: c(30),
            execution_verifier: c(10),
        }
    }

    #[test]
    fn independent_execution_and_witness_pass() {
        let receipt = finalized_receipt();
        let verified = verify_finalized_receipt_with_trust(
            &receipt,
            &accountability_policy(),
            &high_assurance_policy(),
            &verifier(),
        )
        .unwrap();
        assert_eq!(verified.evidence().len(), 2);
        assert!(!verified.trust_policy_digest().is_zero());
        assert_eq!(
            verified.evaluate_notification(receipt.occurred_at_ms, &accountability_policy()),
            Ok(NotificationDisposition::DeliverNow)
        );
    }

    #[test]
    fn witness_controlled_by_requester_trust_domain_fails() {
        let receipt = finalized_receipt();
        let verifier = TestVerifier {
            witness_domain: c(20),
            execution_verifier: c(10),
        };
        assert_eq!(
            verify_finalized_receipt_with_trust(
                &receipt,
                &accountability_policy(),
                &high_assurance_policy(),
                &verifier,
            ),
            Err(TrustVerificationError::CrossRoleTrustDomainOverlap {
                left: AttestationRole::ExecutionBinding,
                right: AttestationRole::ExternalWitness,
            })
        );
    }

    #[test]
    fn same_verifier_cannot_self_witness() {
        let receipt = finalized_receipt();
        let verifier = TestVerifier {
            witness_domain: c(30),
            execution_verifier: c(30),
        };
        assert_eq!(
            verify_finalized_receipt_with_trust(
                &receipt,
                &accountability_policy(),
                &high_assurance_policy(),
                &verifier,
            ),
            Err(TrustVerificationError::CrossRoleVerifierOverlap {
                left: AttestationRole::ExecutionBinding,
                right: AttestationRole::ExternalWitness,
            })
        );
    }

    #[test]
    fn duplicate_proof_digest_cannot_inflate_or_cross_roles() {
        let mut receipt = finalized_receipt();
        receipt.attestations[1].proof_digest = receipt.attestations[0].proof_digest;
        assert_eq!(
            verify_finalized_receipt_with_trust(
                &receipt,
                &accountability_policy(),
                &high_assurance_policy(),
                &verifier(),
            ),
            Err(TrustVerificationError::DuplicateProofDigest)
        );
    }

    #[test]
    fn role_threshold_requires_distinct_verifier_roots() {
        let mut receipt = finalized_receipt();
        let statement = receipt.attestations[0].statement_digest;
        receipt.attestations.insert(
            1,
            attestation(AttestationRole::ExecutionBinding, statement, 51),
        );

        let policy = AttestationTrustPolicy {
            role_requirements: vec![
                RoleTrustRequirement {
                    role: AttestationRole::ExecutionBinding,
                    min_verified_attestations: 2,
                    min_distinct_verifiers: 2,
                    min_distinct_trust_domains: 1,
                },
                RoleTrustRequirement {
                    role: AttestationRole::ExternalWitness,
                    min_verified_attestations: 1,
                    min_distinct_verifiers: 1,
                    min_distinct_trust_domains: 1,
                },
            ],
            cross_role_independence: Vec::new(),
        };

        assert_eq!(
            verify_finalized_receipt_with_trust(
                &receipt,
                &accountability_policy(),
                &policy,
                &verifier(),
            ),
            Err(TrustVerificationError::InsufficientDistinctVerifiers {
                role: AttestationRole::ExecutionBinding,
                found: 1,
                required: 2,
            })
        );
    }

    #[test]
    fn trust_policy_commitment_is_order_invariant() {
        let first = high_assurance_policy();
        let expected = attestation_trust_policy_commitment(&first).unwrap();
        let mut reordered = first;
        reordered.role_requirements.reverse();
        reordered.cross_role_independence[0] = CrossRoleIndependence {
            left: AttestationRole::ExternalWitness,
            right: AttestationRole::ExecutionBinding,
            require_distinct_verifiers: true,
            require_distinct_trust_domains: true,
        };
        assert_eq!(expected, attestation_trust_policy_commitment(&reordered).unwrap());
    }
}

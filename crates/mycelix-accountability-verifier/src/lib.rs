// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provider-neutral cryptographic verification for reciprocal-accountability receipts.
//!
//! `mycelix-accountability-core` owns semantic/privacy invariants and opaque
//! attestation references. This crate owns the missing fail-closed boundary between
//! "a proof reference is present" and "the referenced proof was actually verified".
//!
//! Providers (Xenia, Symthaea, independent witnesses, future proof systems) resolve
//! an [`AttestationRef`] from their own evidence store and authenticate its scheme,
//! verifier profile/key material, proof bytes, and public statement. Mycelix itself
//! does not need to depend on those implementations.

use std::fmt::Display;

use mycelix_accountability_core::{
    pre_attestation_receipt_commitment, policy_commitment, purpose_commitment,
    validate_pre_attestation_receipt, validate_receipt, AccessReceipt, AccountabilityError,
    AccountabilityPolicy, AttestationRef, AttestationRole, Commitment32, NotificationDisposition,
};
use thiserror::Error;

/// Commitment-only context that every evidence provider receives during
/// verification.
///
/// This deliberately contains no subject identifier, actor name, case/matter ID,
/// purpose text, authority ID, or record data. It is nevertheless rich enough for
/// Xenia/Symthaea adapters to prove that their internally bound query, purpose,
/// policy, requester and result are the same commitments carried by the exact
/// Mycelix receipt statement.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AttestationVerificationContext {
    /// Canonical pre-attestation receipt commitment shared by every proof role.
    pub statement_digest: Commitment32,
    /// Authenticated requester/source fingerprint expected by execution evidence.
    pub requester_source_id: Commitment32,
    /// Exact query commitment from the receipt.
    pub query_digest: Commitment32,
    /// Canonical purpose/scope commitment derived from the receipt.
    pub purpose_digest: Commitment32,
    /// Canonical accountability-policy commitment used for this verification.
    pub policy_digest: Commitment32,
    /// Minimum-necessary result commitment, when the receipt declares one.
    pub result_digest: Option<Commitment32>,
}

/// Cryptographic verifier implemented by an evidence provider or adapter.
///
/// A verifier MUST resolve/authenticate `proof_digest`, MUST treat `scheme` and
/// `verifier_profile` as claims to be checked rather than trusted metadata, and MUST
/// verify all public bindings relevant to its proof role against `expected`.
pub trait AttestationVerifier {
    type Error: Display;

    fn verify(
        &self,
        attestation: &AttestationRef,
        expected: &AttestationVerificationContext,
    ) -> Result<(), Self::Error>;
}

/// Fail-closed verification failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum VerificationError {
    #[error("receipt failed structural validation: {0:?}")]
    Structural(AccountabilityError),
    #[error("failed to compute a canonical accountability commitment")]
    CommitmentEncoding,
    #[error("pre-attestation receipt already contains evidence references")]
    EvidenceAttachedBeforeCommitment,
    #[error("{role:?} attestation is bound to a different receipt statement")]
    StatementMismatch { role: AttestationRole },
    #[error("{role:?} attestation failed cryptographic verification: {reason}")]
    CryptographicVerificationFailed {
        role: AttestationRole,
        reason: String,
    },
    #[error("accountability policy changed after receipt verification")]
    PolicyChangedAfterVerification,
}

/// Immutable, cryptographically verified receipt snapshot.
///
/// This is intentionally not an authorization token. It records that all proof
/// references attached to this exact snapshot were verified under one exact policy.
/// Downstream code should pass this type, rather than a raw [`AccessReceipt`], across
/// the final evidence/disclosure boundary.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedReceipt {
    receipt: AccessReceipt,
    verification_context: AttestationVerificationContext,
    verified_attestation_count: usize,
}

impl VerifiedReceipt {
    /// Read the immutable receipt snapshot that was actually verified.
    pub fn receipt(&self) -> &AccessReceipt {
        &self.receipt
    }

    /// Commitment-only context against which every attached proof was checked.
    pub fn verification_context(&self) -> &AttestationVerificationContext {
        &self.verification_context
    }

    /// Public statement shared by every proof in this verified bundle.
    pub const fn statement_digest(&self) -> Commitment32 {
        self.verification_context.statement_digest
    }

    /// Policy commitment used during verification.
    pub const fn policy_digest(&self) -> Commitment32 {
        self.verification_context.policy_digest
    }

    /// Number of attached attestations that were cryptographically checked.
    pub const fn verified_attestation_count(&self) -> usize {
        self.verified_attestation_count
    }

    /// Evaluate notification using the same policy that was verified.
    ///
    /// Replacing the policy after proof verification fails closed rather than
    /// silently changing notification/delay semantics beneath a verified receipt.
    pub fn evaluate_notification(
        &self,
        now_ms: u64,
        policy: &AccountabilityPolicy,
    ) -> Result<NotificationDisposition, VerificationError> {
        let current_policy = policy_commitment(policy)
            .map_err(|_| VerificationError::CommitmentEncoding)?;
        if current_policy != self.verification_context.policy_digest {
            return Err(VerificationError::PolicyChangedAfterVerification);
        }
        mycelix_accountability_core::evaluate_notification(&self.receipt, now_ms, policy)
            .map_err(VerificationError::Structural)
    }
}

/// Validate the semantic receipt at the freeze point and return the exact statement
/// that Xenia/Symthaea/witnesses must prove.
///
/// Unlike the lower-level core structural validator, this guard deliberately rejects
/// *all* already-attached evidence. That makes "commit first, prove second" an
/// executable invariant and prevents callers from accidentally blessing a mutable or
/// circular pre-attestation state.
pub fn freeze_pre_attestation_statement(
    receipt: &AccessReceipt,
    policy: &AccountabilityPolicy,
) -> Result<Commitment32, VerificationError> {
    if !receipt.attestations.is_empty() {
        return Err(VerificationError::EvidenceAttachedBeforeCommitment);
    }
    validate_pre_attestation_receipt(receipt, policy)
        .map_err(VerificationError::Structural)?;
    pre_attestation_receipt_commitment(receipt)
        .map_err(|_| VerificationError::CommitmentEncoding)
}

/// Derive the complete privacy-preserving public context used by provider
/// verifiers. The receipt is structurally validated first, so callers cannot
/// obtain a verification context for malformed accountability state.
pub fn verification_context(
    receipt: &AccessReceipt,
    policy: &AccountabilityPolicy,
) -> Result<AttestationVerificationContext, VerificationError> {
    validate_pre_attestation_receipt(receipt, policy)
        .map_err(VerificationError::Structural)?;
    let statement_digest = pre_attestation_receipt_commitment(receipt)
        .map_err(|_| VerificationError::CommitmentEncoding)?;
    let purpose_digest = purpose_commitment(&receipt.purpose)
        .map_err(|_| VerificationError::CommitmentEncoding)?;
    let policy_digest = policy_commitment(policy)
        .map_err(|_| VerificationError::CommitmentEncoding)?;

    Ok(AttestationVerificationContext {
        statement_digest,
        requester_source_id: receipt.requester.authenticated_source_id,
        query_digest: receipt.query_digest,
        purpose_digest,
        policy_digest,
        result_digest: receipt.disclosure.result_digest,
    })
}

/// Structurally validate and cryptographically verify a finalized receipt.
///
/// Every attached attestation is verified, not merely the roles required by policy.
/// This prevents an invalid optional/decorative proof from hitchhiking beside valid
/// required evidence. The core validator still decides which roles are mandatory;
/// the provider adapter decides whether each referenced proof is authentic and valid.
pub fn verify_finalized_receipt<V: AttestationVerifier>(
    receipt: &AccessReceipt,
    policy: &AccountabilityPolicy,
    verifier: &V,
) -> Result<VerifiedReceipt, VerificationError> {
    validate_receipt(receipt, policy).map_err(VerificationError::Structural)?;

    let expected = verification_context(receipt, policy)?;

    for attestation in &receipt.attestations {
        if attestation.statement_digest != expected.statement_digest {
            return Err(VerificationError::StatementMismatch {
                role: attestation.role,
            });
        }

        verifier
            .verify(attestation, &expected)
            .map_err(|error| VerificationError::CryptographicVerificationFailed {
                role: attestation.role,
                reason: error.to_string(),
            })?;
    }

    Ok(VerifiedReceipt {
        receipt: receipt.clone(),
        verification_context: expected,
        verified_attestation_count: receipt.attestations.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_accountability_core::{
        AuthorityType, DisclosureKind, DisclosureSummary, LegalAuthority, LookupOutcome,
        NotificationDirective, PairwiseSubjectId, PurposeBinding, QueryBudgetCharge,
        RequestingPrincipal, SubjectRight, ACCOUNTABILITY_COMMITMENT_ALGORITHM,
        RECIPROCAL_ACCOUNTABILITY_PROTOCOL_VERSION,
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

    fn policy() -> AccountabilityPolicy {
        AccountabilityPolicy {
            max_delay_ms: 30 * DAY_MS,
            min_delay_approvals: 2,
            require_approval_outside_requester_org: true,
            permitted_delay_reasons: Vec::new(),
            required_subject_rights: rights(),
            require_query_budget_charge: true,
            required_attestation_roles: vec![AttestationRole::ExecutionBinding],
        }
    }

    fn receipt() -> AccessReceipt {
        let occurred_at_ms = 1_800_000_000_000;
        AccessReceipt {
            protocol_version: RECIPROCAL_ACCOUNTABILITY_PROTOCOL_VERSION,
            receipt_id: "receipt-verify-001".into(),
            subject: PairwiseSubjectId("pairwise:subject:verify".into()),
            requester: RequestingPrincipal {
                organization_id: "agency-a".into(),
                actor_id: "operator-7".into(),
                role: "investigator".into(),
                authenticated_source_id: c(1),
            },
            purpose: PurposeBinding {
                purpose_code: "minimum-necessary-test".into(),
                plain_language_purpose: "Verify a bounded predicate".into(),
                matter_id: None,
                scope_digest: c(2),
                expires_at_ms: occurred_at_ms + DAY_MS,
            },
            legal_authority: LegalAuthority {
                authority_type: AuthorityType::Consent,
                authority_id: "consent:test".into(),
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
        statement_digest: Commitment32,
        scheme: &str,
    ) -> AttestationRef {
        AttestationRef {
            role,
            scheme: scheme.into(),
            statement_digest,
            proof_digest: c(match role {
                AttestationRole::ExecutionBinding => 50,
                AttestationRole::ComputationProof => 51,
                AttestationRole::PolicyProof => 52,
                AttestationRole::ExternalWitness => 53,
            }),
            verifier_profile: "test-verifier-v1".into(),
        }
    }

    struct TestVerifier {
        reject_role: Option<AttestationRole>,
    }

    impl AttestationVerifier for TestVerifier {
        type Error = &'static str;

        fn verify(
            &self,
            attestation: &AttestationRef,
            expected: &AttestationVerificationContext,
        ) -> Result<(), Self::Error> {
            if attestation.scheme != "test-proof-v1" {
                return Err("unsupported proof scheme");
            }
            if attestation.verifier_profile != "test-verifier-v1" {
                return Err("untrusted verifier profile");
            }
            if attestation.statement_digest != expected.statement_digest {
                return Err("statement mismatch");
            }
            if expected.requester_source_id != c(1)
                || expected.query_digest != c(3)
                || expected.result_digest != Some(c(4))
                || expected.purpose_digest.is_zero()
                || expected.policy_digest.is_zero()
            {
                return Err("commitment context mismatch");
            }
            if self.reject_role == Some(attestation.role) {
                return Err("cryptographic proof rejected");
            }
            Ok(())
        }
    }

    fn finalized_receipt() -> AccessReceipt {
        let mut receipt = receipt();
        let statement = freeze_pre_attestation_statement(&receipt, &policy()).unwrap();
        receipt.attestations.push(attestation(
            AttestationRole::ExecutionBinding,
            statement,
            "test-proof-v1",
        ));
        receipt
    }

    #[test]
    fn freeze_point_rejects_any_already_attached_evidence() {
        let mut receipt = receipt();
        receipt.attestations.push(attestation(
            AttestationRole::ExecutionBinding,
            c(90),
            "test-proof-v1",
        ));
        assert_eq!(
            freeze_pre_attestation_statement(&receipt, &policy()),
            Err(VerificationError::EvidenceAttachedBeforeCommitment)
        );
    }

    #[test]
    fn proof_reference_presence_is_not_cryptographic_verification() {
        let receipt = finalized_receipt();
        assert_eq!(validate_receipt(&receipt, &policy()), Ok(()));

        let verifier = TestVerifier {
            reject_role: Some(AttestationRole::ExecutionBinding),
        };
        assert!(matches!(
            verify_finalized_receipt(&receipt, &policy(), &verifier),
            Err(VerificationError::CryptographicVerificationFailed {
                role: AttestationRole::ExecutionBinding,
                ..
            })
        ));
    }

    #[test]
    fn proof_must_bind_exact_pre_attestation_statement() {
        let mut receipt = finalized_receipt();
        receipt.attestations[0].statement_digest = c(99);
        let verifier = TestVerifier { reject_role: None };
        assert_eq!(
            verify_finalized_receipt(&receipt, &policy(), &verifier),
            Err(VerificationError::StatementMismatch {
                role: AttestationRole::ExecutionBinding
            })
        );
    }

    #[test]
    fn optional_attestations_are_verified_too() {
        let mut receipt = finalized_receipt();
        let statement = pre_attestation_receipt_commitment(&receipt).unwrap();
        receipt.attestations.push(attestation(
            AttestationRole::ComputationProof,
            statement,
            "test-proof-v1",
        ));
        let verifier = TestVerifier {
            reject_role: Some(AttestationRole::ComputationProof),
        };
        assert!(matches!(
            verify_finalized_receipt(&receipt, &policy(), &verifier),
            Err(VerificationError::CryptographicVerificationFailed {
                role: AttestationRole::ComputationProof,
                ..
            })
        ));
    }

    #[test]
    fn unsupported_scheme_fails_closed() {
        let mut receipt = finalized_receipt();
        receipt.attestations[0].scheme = "made-up-scheme".into();
        let verifier = TestVerifier { reject_role: None };
        assert!(matches!(
            verify_finalized_receipt(&receipt, &policy(), &verifier),
            Err(VerificationError::CryptographicVerificationFailed {
                role: AttestationRole::ExecutionBinding,
                ..
            })
        ));
    }

    #[test]
    fn verification_context_contains_only_expected_public_commitments() {
        let receipt = receipt();
        let context = verification_context(&receipt, &policy()).unwrap();
        assert_eq!(context.requester_source_id, c(1));
        assert_eq!(context.query_digest, c(3));
        assert_eq!(context.result_digest, Some(c(4)));
        assert!(!context.purpose_digest.is_zero());
        assert!(!context.policy_digest.is_zero());
        assert_eq!(
            context.statement_digest,
            pre_attestation_receipt_commitment(&receipt).unwrap()
        );
    }

    #[test]
    fn verified_snapshot_is_bound_to_the_policy_used_for_verification() {
        let receipt = finalized_receipt();
        let verifier = TestVerifier { reject_role: None };
        let verified = verify_finalized_receipt(&receipt, &policy(), &verifier).unwrap();
        assert_eq!(verified.verified_attestation_count(), 1);
        assert_eq!(verified.verification_context().query_digest, c(3));
        assert_eq!(
            verified.evaluate_notification(receipt.occurred_at_ms, &policy()),
            Ok(NotificationDisposition::DeliverNow)
        );

        let mut changed_policy = policy();
        changed_policy.require_query_budget_charge = false;
        assert_eq!(
            verified.evaluate_notification(receipt.occurred_at_ms, &changed_policy),
            Err(VerificationError::PolicyChangedAfterVerification)
        );
    }

    #[test]
    fn verifier_contract_uses_the_same_commitment_profile_as_core() {
        assert_eq!(ACCOUNTABILITY_COMMITMENT_ALGORITHM, "blake3-256");
        let receipt = receipt();
        let frozen = freeze_pre_attestation_statement(&receipt, &policy()).unwrap();
        assert_eq!(frozen, pre_attestation_receipt_commitment(&receipt).unwrap());
    }
}

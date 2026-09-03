// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Acyclic pre-disclosure evidence-bundle verification for reciprocal accountability.
//!
//! The receipt statement is frozen first. Xenia/Symthaea/policy proofs bind that
//! statement and are independently qualified by `mycelix-accountability-trust`.
//! This crate then commits the exact verified non-witness evidence set and only
//! afterwards permits `ExternalWitness` proofs to attest that new bundle statement.
//! This avoids a receipt <-> proof <-> witness hash cycle while making it possible
//! to distinguish "the receipt existed" from "the required evidence bundle existed".

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Display;

use mycelix_accountability_core::{
    AccessReceipt, AccountabilityError, AccountabilityPolicy, AttestationRef, AttestationRole,
    Commitment32, policy_commitment, pre_attestation_receipt_commitment,
    validate_pre_attestation_receipt,
};
use mycelix_accountability_trust::{
    TrustVerifiedReceipt, VerifiedAttestationAuthority, VerifiedEvidenceIdentity,
};
use thiserror::Error;

const PRE_WITNESS_BUNDLE_DOMAIN: &[u8] =
    b"mycelix:accountability-pre-witness-evidence-bundle:v1";
const PRE_WITNESS_BUNDLE_CODEC: &[u8] = b"mycelix-accountability-pre-witness-bundle-v1";
const WITNESS_POLICY_DOMAIN: &[u8] = b"mycelix:accountability-external-witness-policy:v1";
const WITNESS_POLICY_CODEC: &[u8] = b"mycelix-accountability-external-witness-policy-v1";
const FINAL_BUNDLE_DOMAIN: &[u8] = b"mycelix:accountability-final-evidence-bundle:v1";
const FINAL_BUNDLE_CODEC: &[u8] = b"mycelix-accountability-final-evidence-bundle-v1";

/// Trust requirements for the external witnesses that attest the complete
/// pre-witness evidence bundle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExternalWitnessTrustPolicy {
    /// Minimum number of cryptographically verified witness artifacts.
    pub min_verified_witnesses: u8,
    /// Minimum number of distinct witness verifier/key roots.
    pub min_distinct_verifiers: u8,
    /// Minimum number of independently administered witness trust domains.
    pub min_distinct_trust_domains: u8,
    /// Require every witness verifier root to be absent from the non-witness
    /// evidence set.
    pub require_verifier_disjoint_from_non_witness: bool,
    /// Require every witness administrative trust domain to be absent from the
    /// non-witness evidence set.
    pub require_trust_domain_disjoint_from_non_witness: bool,
}

/// Public commitment-only context supplied to an external-witness verifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExternalWitnessVerificationContext {
    /// Statement the witness MUST attest: the complete verified non-witness
    /// evidence bundle, not merely the semantic receipt statement.
    pub pre_witness_bundle_digest: Commitment32,
    /// Underlying semantic receipt statement for audit/debug correlation.
    pub receipt_statement_digest: Commitment32,
    /// Exact accountability-policy commitment qualified by the base snapshot.
    pub accountability_policy_digest: Commitment32,
    /// Exact non-witness trust-policy commitment qualified by the base snapshot.
    pub non_witness_trust_policy_digest: Commitment32,
}

/// Provider that verifies an external witness and resolves the cryptographic
/// authority that actually produced it.
pub trait ExternalWitnessVerifier {
    /// Provider-specific verification error.
    type Error: Display;

    /// Verify the witness proof and return its trusted verifier/trust-domain
    /// identities. `scheme` and `verifier_profile` are claims to check, not trust
    /// anchors.
    fn verify_external_witness(
        &self,
        attestation: &AttestationRef,
        expected: &ExternalWitnessVerificationContext,
    ) -> Result<VerifiedAttestationAuthority, Self::Error>;
}

/// A receipt whose non-witness proofs were independently trust-qualified and whose
/// required external witnesses attest the exact resulting evidence bundle.
///
/// Possession of this type is the high-assurance software boundary for the claim
/// "the required proof bundle was witnessed before release". It still does not,
/// by itself, create trustworthy wall-clock time; the witness provider must bind a
/// trustworthy order/timestamp source if a deployment requires that property.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PreDisclosureVerifiedBundle {
    base: TrustVerifiedReceipt,
    finalized_receipt: AccessReceipt,
    witnesses: Vec<VerifiedEvidenceIdentity>,
    pre_witness_bundle_digest: Commitment32,
    witness_policy_digest: Commitment32,
    finalized_evidence_bundle_digest: Commitment32,
}

impl PreDisclosureVerifiedBundle {
    /// Final receipt containing both provider proofs and external witness refs.
    pub fn receipt(&self) -> &AccessReceipt {
        &self.finalized_receipt
    }

    /// Trust-qualified non-witness snapshot from which the witness statement was
    /// derived.
    pub fn base(&self) -> &TrustVerifiedReceipt {
        &self.base
    }

    /// Resolved witness evidence identities.
    pub fn witnesses(&self) -> &[VerifiedEvidenceIdentity] {
        &self.witnesses
    }

    /// Canonical statement externally witnessed before disclosure.
    pub const fn pre_witness_bundle_digest(&self) -> Commitment32 {
        self.pre_witness_bundle_digest
    }

    /// Exact witness trust policy used to qualify this bundle.
    pub const fn witness_policy_digest(&self) -> Commitment32 {
        self.witness_policy_digest
    }

    /// Archival commitment over the pre-witness statement plus the complete
    /// resolved witness set.
    pub const fn finalized_evidence_bundle_digest(&self) -> Commitment32 {
        self.finalized_evidence_bundle_digest
    }
}

/// Fail-closed evidence-bundle verification errors.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum BundleVerificationError {
    /// Final receipt failed semantic/privacy validation.
    #[error("final receipt failed pre-attestation validation: {0:?}")]
    InvalidFinalReceipt(AccountabilityError),
    /// Failed to compute a canonical commitment.
    #[error("failed to compute canonical accountability commitment")]
    CommitmentEncoding,
    /// Caller supplied a different accountability policy than the one used to
    /// qualify the non-witness evidence.
    #[error("accountability policy changed after non-witness trust verification")]
    AccountabilityPolicyChanged,
    /// Final receipt semantics do not match the receipt statement qualified by the
    /// base proof set.
    #[error("final receipt semantics differ from the trust-qualified receipt statement")]
    SemanticReceiptMismatch,
    /// A trust-qualified base snapshot already contains external witness evidence.
    #[error("pre-witness base must not contain ExternalWitness evidence")]
    BaseContainsExternalWitness,
    /// Base receipt proof refs and resolved cryptographic identities disagree.
    #[error("trust-qualified base proof refs do not match resolved evidence identities")]
    BaseEvidenceIdentityMismatch,
    /// The final receipt changed, added, or removed a non-witness proof after the
    /// base snapshot was trust-qualified.
    #[error("final receipt changed the trust-qualified non-witness evidence set")]
    FinalNonWitnessEvidenceChanged,
    /// The same proof artifact digest appeared more than once in the final bundle.
    #[error("the same proof digest appears more than once in the evidence bundle")]
    DuplicateProofDigest,
    /// External witness policy has a zero requirement.
    #[error("external witness policy thresholds must be non-zero")]
    ZeroWitnessThreshold,
    /// Distinctness threshold exceeds the number of required witness artifacts.
    #[error("external witness distinctness threshold exceeds witness count threshold")]
    ImpossibleWitnessThreshold,
    /// Witness is still bound to the receipt statement or another stale bundle.
    #[error("external witness is bound to a different pre-witness evidence bundle")]
    WitnessStatementMismatch,
    /// Witness provider rejected the cryptographic proof.
    #[error("external witness failed cryptographic verification: {reason}")]
    WitnessCryptographicVerificationFailed { reason: String },
    /// Provider returned an invalid verifier identity.
    #[error("external witness verifier returned an all-zero verifier identity")]
    ZeroWitnessVerifierId,
    /// Provider returned an invalid administrative trust-domain identity.
    #[error("external witness verifier returned an all-zero trust-domain identity")]
    ZeroWitnessTrustDomainId,
    /// Not enough witness artifacts were verified.
    #[error("verified {found} external witnesses but policy requires {required}")]
    InsufficientWitnesses { found: usize, required: usize },
    /// Not enough independent verifier roots were present.
    #[error("verified {found} distinct witness verifier roots but policy requires {required}")]
    InsufficientDistinctWitnessVerifiers { found: usize, required: usize },
    /// Not enough independently administered witness domains were present.
    #[error("verified {found} distinct witness trust domains but policy requires {required}")]
    InsufficientDistinctWitnessTrustDomains { found: usize, required: usize },
    /// A witness reused a verifier/key root already present in provider evidence.
    #[error("external witness shares a verifier root with non-witness evidence")]
    WitnessVerifierOverlapsNonWitness,
    /// A witness is administered by a trust domain already present in provider
    /// evidence.
    #[error("external witness shares a trust domain with non-witness evidence")]
    WitnessTrustDomainOverlapsNonWitness,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct ProofRefKey {
    role: AttestationRole,
    statement_digest: Commitment32,
    proof_digest: Commitment32,
    scheme: String,
    verifier_profile: String,
}

impl From<&AttestationRef> for ProofRefKey {
    fn from(value: &AttestationRef) -> Self {
        Self {
            role: value.role,
            statement_digest: value.statement_digest,
            proof_digest: value.proof_digest,
            scheme: value.scheme.clone(),
            verifier_profile: value.verifier_profile.clone(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct VerifiedProofRecord {
    reference: ProofRefKey,
    verifier_id: Commitment32,
    trust_domain_id: Commitment32,
}

/// Compute the canonical statement that independent witnesses must attest.
///
/// Only a `TrustVerifiedReceipt` is accepted, so caller-written proof refs cannot
/// enter this commitment without first passing cryptographic verification and the
/// non-witness trust-composition policy.
pub fn pre_witness_evidence_bundle_commitment(
    base: &TrustVerifiedReceipt,
) -> Result<Commitment32, BundleVerificationError> {
    let records = verified_non_witness_records(base)?;

    let mut hasher = blake3::Hasher::new();
    hasher.update(PRE_WITNESS_BUNDLE_DOMAIN);
    hasher.update(&[0]);
    hasher.update(PRE_WITNESS_BUNDLE_CODEC);
    hasher.update(&[0]);
    hasher.update(base.verification_context().statement_digest.as_bytes());
    hasher.update(base.accountability_policy_digest().as_bytes());
    hasher.update(base.trust_policy_digest().as_bytes());
    hasher.update(&(records.len() as u64).to_be_bytes());
    for record in records {
        encode_verified_record(&mut hasher, &record);
    }
    Ok(Commitment32(*hasher.finalize().as_bytes()))
}

/// Stable commitment to the external-witness trust semantics used for this bundle.
pub fn external_witness_trust_policy_commitment(
    policy: &ExternalWitnessTrustPolicy,
) -> Result<Commitment32, BundleVerificationError> {
    validate_witness_policy(policy)?;
    let mut hasher = blake3::Hasher::new();
    hasher.update(WITNESS_POLICY_DOMAIN);
    hasher.update(&[0]);
    hasher.update(WITNESS_POLICY_CODEC);
    hasher.update(&[0]);
    hasher.update(&[policy.min_verified_witnesses]);
    hasher.update(&[policy.min_distinct_verifiers]);
    hasher.update(&[policy.min_distinct_trust_domains]);
    hasher.update(&[u8::from(
        policy.require_verifier_disjoint_from_non_witness,
    )]);
    hasher.update(&[u8::from(
        policy.require_trust_domain_disjoint_from_non_witness,
    )]);
    Ok(Commitment32(*hasher.finalize().as_bytes()))
}

/// Verify the complete acyclic evidence DAG and return the disclosure-safe bundle
/// type only after required external witnesses prove the pre-witness bundle.
pub fn verify_pre_disclosure_evidence_bundle<V: ExternalWitnessVerifier>(
    base: &TrustVerifiedReceipt,
    finalized_receipt: &AccessReceipt,
    accountability_policy: &AccountabilityPolicy,
    witness_policy: &ExternalWitnessTrustPolicy,
    verifier: &V,
) -> Result<PreDisclosureVerifiedBundle, BundleVerificationError> {
    validate_witness_policy(witness_policy)?;
    validate_pre_attestation_receipt(finalized_receipt, accountability_policy)
        .map_err(BundleVerificationError::InvalidFinalReceipt)?;

    let accountability_policy_digest = policy_commitment(accountability_policy)
        .map_err(|_| BundleVerificationError::CommitmentEncoding)?;
    if accountability_policy_digest != base.accountability_policy_digest() {
        return Err(BundleVerificationError::AccountabilityPolicyChanged);
    }

    let final_statement = pre_attestation_receipt_commitment(finalized_receipt)
        .map_err(|_| BundleVerificationError::CommitmentEncoding)?;
    if final_statement != base.verification_context().statement_digest {
        return Err(BundleVerificationError::SemanticReceiptMismatch);
    }

    // This also proves that the base contains no witness evidence and that every
    // base proof ref has exactly one resolved authority identity.
    let _base_records = verified_non_witness_records(base)?;
    require_same_non_witness_refs(base.receipt(), finalized_receipt)?;

    let mut all_proof_digests = BTreeSet::new();
    for attestation in &finalized_receipt.attestations {
        if !all_proof_digests.insert(attestation.proof_digest) {
            return Err(BundleVerificationError::DuplicateProofDigest);
        }
    }

    let pre_witness_bundle_digest = pre_witness_evidence_bundle_commitment(base)?;
    let witness_policy_digest = external_witness_trust_policy_commitment(witness_policy)?;
    let witness_context = ExternalWitnessVerificationContext {
        pre_witness_bundle_digest,
        receipt_statement_digest: base.verification_context().statement_digest,
        accountability_policy_digest: base.accountability_policy_digest(),
        non_witness_trust_policy_digest: base.trust_policy_digest(),
    };

    let base_verifiers: BTreeSet<_> = base.evidence().iter().map(|item| item.verifier_id).collect();
    let base_domains: BTreeSet<_> = base
        .evidence()
        .iter()
        .map(|item| item.trust_domain_id)
        .collect();

    let mut witness_evidence = Vec::new();
    let mut witness_records = Vec::new();
    for attestation in finalized_receipt
        .attestations
        .iter()
        .filter(|item| item.role == AttestationRole::ExternalWitness)
    {
        if attestation.statement_digest != pre_witness_bundle_digest {
            return Err(BundleVerificationError::WitnessStatementMismatch);
        }

        let authority = verifier
            .verify_external_witness(attestation, &witness_context)
            .map_err(|error| BundleVerificationError::WitnessCryptographicVerificationFailed {
                reason: error.to_string(),
            })?;
        if authority.verifier_id.is_zero() {
            return Err(BundleVerificationError::ZeroWitnessVerifierId);
        }
        if authority.trust_domain_id.is_zero() {
            return Err(BundleVerificationError::ZeroWitnessTrustDomainId);
        }
        if witness_policy.require_verifier_disjoint_from_non_witness
            && base_verifiers.contains(&authority.verifier_id)
        {
            return Err(BundleVerificationError::WitnessVerifierOverlapsNonWitness);
        }
        if witness_policy.require_trust_domain_disjoint_from_non_witness
            && base_domains.contains(&authority.trust_domain_id)
        {
            return Err(BundleVerificationError::WitnessTrustDomainOverlapsNonWitness);
        }

        witness_evidence.push(VerifiedEvidenceIdentity {
            role: AttestationRole::ExternalWitness,
            proof_digest: attestation.proof_digest,
            verifier_id: authority.verifier_id,
            trust_domain_id: authority.trust_domain_id,
        });
        witness_records.push(VerifiedProofRecord {
            reference: ProofRefKey::from(attestation),
            verifier_id: authority.verifier_id,
            trust_domain_id: authority.trust_domain_id,
        });
    }

    enforce_witness_policy(&witness_evidence, witness_policy)?;
    witness_records.sort_unstable();
    let finalized_evidence_bundle_digest = finalized_bundle_commitment(
        pre_witness_bundle_digest,
        witness_policy_digest,
        &witness_records,
    );

    Ok(PreDisclosureVerifiedBundle {
        base: base.clone(),
        finalized_receipt: finalized_receipt.clone(),
        witnesses: witness_evidence,
        pre_witness_bundle_digest,
        witness_policy_digest,
        finalized_evidence_bundle_digest,
    })
}

fn verified_non_witness_records(
    base: &TrustVerifiedReceipt,
) -> Result<Vec<VerifiedProofRecord>, BundleVerificationError> {
    if base
        .receipt()
        .attestations
        .iter()
        .any(|item| item.role == AttestationRole::ExternalWitness)
        || base
            .evidence()
            .iter()
            .any(|item| item.role == AttestationRole::ExternalWitness)
    {
        return Err(BundleVerificationError::BaseContainsExternalWitness);
    }

    let mut by_digest = BTreeMap::new();
    for identity in base.evidence() {
        if by_digest.insert(identity.proof_digest, identity).is_some() {
            return Err(BundleVerificationError::BaseEvidenceIdentityMismatch);
        }
    }

    let mut records = Vec::with_capacity(base.receipt().attestations.len());
    for attestation in &base.receipt().attestations {
        if attestation.statement_digest != base.verification_context().statement_digest {
            return Err(BundleVerificationError::BaseEvidenceIdentityMismatch);
        }
        let identity = by_digest
            .get(&attestation.proof_digest)
            .ok_or(BundleVerificationError::BaseEvidenceIdentityMismatch)?;
        if identity.role != attestation.role {
            return Err(BundleVerificationError::BaseEvidenceIdentityMismatch);
        }
        records.push(VerifiedProofRecord {
            reference: ProofRefKey::from(attestation),
            verifier_id: identity.verifier_id,
            trust_domain_id: identity.trust_domain_id,
        });
    }
    if records.len() != by_digest.len() {
        return Err(BundleVerificationError::BaseEvidenceIdentityMismatch);
    }
    records.sort_unstable();
    Ok(records)
}

fn require_same_non_witness_refs(
    base_receipt: &AccessReceipt,
    finalized_receipt: &AccessReceipt,
) -> Result<(), BundleVerificationError> {
    let mut base: Vec<_> = base_receipt
        .attestations
        .iter()
        .filter(|item| item.role != AttestationRole::ExternalWitness)
        .map(ProofRefKey::from)
        .collect();
    let mut finalized: Vec<_> = finalized_receipt
        .attestations
        .iter()
        .filter(|item| item.role != AttestationRole::ExternalWitness)
        .map(ProofRefKey::from)
        .collect();
    base.sort_unstable();
    finalized.sort_unstable();
    if base != finalized {
        return Err(BundleVerificationError::FinalNonWitnessEvidenceChanged);
    }
    Ok(())
}

fn validate_witness_policy(
    policy: &ExternalWitnessTrustPolicy,
) -> Result<(), BundleVerificationError> {
    if policy.min_verified_witnesses == 0
        || policy.min_distinct_verifiers == 0
        || policy.min_distinct_trust_domains == 0
    {
        return Err(BundleVerificationError::ZeroWitnessThreshold);
    }
    if policy.min_distinct_verifiers > policy.min_verified_witnesses
        || policy.min_distinct_trust_domains > policy.min_verified_witnesses
    {
        return Err(BundleVerificationError::ImpossibleWitnessThreshold);
    }
    Ok(())
}

fn enforce_witness_policy(
    witnesses: &[VerifiedEvidenceIdentity],
    policy: &ExternalWitnessTrustPolicy,
) -> Result<(), BundleVerificationError> {
    let required = usize::from(policy.min_verified_witnesses);
    if witnesses.len() < required {
        return Err(BundleVerificationError::InsufficientWitnesses {
            found: witnesses.len(),
            required,
        });
    }
    let verifiers: BTreeSet<_> = witnesses.iter().map(|item| item.verifier_id).collect();
    let required_verifiers = usize::from(policy.min_distinct_verifiers);
    if verifiers.len() < required_verifiers {
        return Err(BundleVerificationError::InsufficientDistinctWitnessVerifiers {
            found: verifiers.len(),
            required: required_verifiers,
        });
    }
    let domains: BTreeSet<_> = witnesses.iter().map(|item| item.trust_domain_id).collect();
    let required_domains = usize::from(policy.min_distinct_trust_domains);
    if domains.len() < required_domains {
        return Err(
            BundleVerificationError::InsufficientDistinctWitnessTrustDomains {
                found: domains.len(),
                required: required_domains,
            },
        );
    }
    Ok(())
}

fn encode_verified_record(hasher: &mut blake3::Hasher, record: &VerifiedProofRecord) {
    hasher.update(&[role_tag(record.reference.role)]);
    hasher.update(record.reference.statement_digest.as_bytes());
    hasher.update(record.reference.proof_digest.as_bytes());
    update_len_prefixed(hasher, record.reference.scheme.as_bytes());
    update_len_prefixed(hasher, record.reference.verifier_profile.as_bytes());
    hasher.update(record.verifier_id.as_bytes());
    hasher.update(record.trust_domain_id.as_bytes());
}

fn finalized_bundle_commitment(
    pre_witness_bundle_digest: Commitment32,
    witness_policy_digest: Commitment32,
    witnesses: &[VerifiedProofRecord],
) -> Commitment32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(FINAL_BUNDLE_DOMAIN);
    hasher.update(&[0]);
    hasher.update(FINAL_BUNDLE_CODEC);
    hasher.update(&[0]);
    hasher.update(pre_witness_bundle_digest.as_bytes());
    hasher.update(witness_policy_digest.as_bytes());
    hasher.update(&(witnesses.len() as u64).to_be_bytes());
    for witness in witnesses {
        encode_verified_record(&mut hasher, witness);
    }
    Commitment32(*hasher.finalize().as_bytes())
}

fn update_len_prefixed(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
}

fn role_tag(role: AttestationRole) -> u8 {
    match role {
        AttestationRole::ExecutionBinding => 1,
        AttestationRole::ComputationProof => 2,
        AttestationRole::PolicyProof => 3,
        AttestationRole::ExternalWitness => 4,
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
    use mycelix_accountability_trust::{
        AttestationTrustPolicy, RoleTrustRequirement, TrustAwareAttestationVerifier,
        verify_finalized_receipt_with_trust,
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

    fn accountability_policy(required_roles: Vec<AttestationRole>) -> AccountabilityPolicy {
        AccountabilityPolicy {
            max_delay_ms: 30 * DAY_MS,
            min_delay_approvals: 2,
            require_approval_outside_requester_org: true,
            permitted_delay_reasons: Vec::new(),
            required_subject_rights: rights(),
            require_query_budget_charge: true,
            required_attestation_roles: required_roles,
        }
    }

    fn bare_receipt() -> AccessReceipt {
        let occurred_at_ms = 1_800_000_000_000;
        AccessReceipt {
            protocol_version: RECIPROCAL_ACCOUNTABILITY_PROTOCOL_VERSION,
            receipt_id: "receipt-bundle-001".into(),
            subject: PairwiseSubjectId("pairwise:subject:bundle".into()),
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
        proof_digest: Commitment32,
    ) -> AttestationRef {
        let (scheme, profile) = match role {
            AttestationRole::ExecutionBinding => ("xenia-test-v1", "xenia-verifier-v1"),
            AttestationRole::ComputationProof => ("symthaea-test-v1", "risc0-verifier-v1"),
            AttestationRole::PolicyProof => ("policy-test-v1", "policy-verifier-v1"),
            AttestationRole::ExternalWitness => ("witness-test-v1", "witness-verifier-v1"),
        };
        AttestationRef {
            role,
            scheme: scheme.into(),
            statement_digest,
            proof_digest,
            verifier_profile: profile.into(),
        }
    }

    struct BaseVerifier;

    impl TrustAwareAttestationVerifier for BaseVerifier {
        type Error = &'static str;

        fn verify_with_authority(
            &self,
            attestation: &AttestationRef,
            expected: &mycelix_accountability_verifier::AttestationVerificationContext,
        ) -> Result<VerifiedAttestationAuthority, Self::Error> {
            if attestation.statement_digest != expected.statement_digest {
                return Err("statement mismatch");
            }
            match attestation.role {
                AttestationRole::ExecutionBinding if attestation.scheme == "xenia-test-v1" => {
                    Ok(VerifiedAttestationAuthority {
                        verifier_id: c(60),
                        trust_domain_id: c(70),
                    })
                }
                AttestationRole::ComputationProof
                    if attestation.scheme == "symthaea-test-v1" =>
                {
                    Ok(VerifiedAttestationAuthority {
                        verifier_id: c(61),
                        trust_domain_id: c(71),
                    })
                }
                _ => Err("unsupported base proof"),
            }
        }
    }

    struct WitnessVerifier {
        verifier_id: Commitment32,
        trust_domain_id: Commitment32,
        reject: bool,
    }

    impl ExternalWitnessVerifier for WitnessVerifier {
        type Error = &'static str;

        fn verify_external_witness(
            &self,
            attestation: &AttestationRef,
            expected: &ExternalWitnessVerificationContext,
        ) -> Result<VerifiedAttestationAuthority, Self::Error> {
            if self.reject {
                return Err("witness rejected");
            }
            if attestation.role != AttestationRole::ExternalWitness
                || attestation.scheme != "witness-test-v1"
                || attestation.verifier_profile != "witness-verifier-v1"
                || attestation.statement_digest != expected.pre_witness_bundle_digest
            {
                return Err("witness context mismatch");
            }
            Ok(VerifiedAttestationAuthority {
                verifier_id: self.verifier_id,
                trust_domain_id: self.trust_domain_id,
            })
        }
    }

    fn trust_policy(required_roles: &[AttestationRole]) -> AttestationTrustPolicy {
        AttestationTrustPolicy {
            role_requirements: required_roles
                .iter()
                .copied()
                .map(|role| RoleTrustRequirement {
                    role,
                    min_verified_attestations: 1,
                    min_distinct_verifiers: 1,
                    min_distinct_trust_domains: 1,
                })
                .collect(),
            cross_role_independence: Vec::new(),
        }
    }

    fn verified_base_with_roles(required_roles: &[AttestationRole]) -> TrustVerifiedReceipt {
        let mut receipt = bare_receipt();
        let statement = pre_attestation_receipt_commitment(&receipt).unwrap();
        if required_roles.contains(&AttestationRole::ExecutionBinding) {
            receipt.attestations.push(attestation(
                AttestationRole::ExecutionBinding,
                statement,
                c(50),
            ));
        }
        if required_roles.contains(&AttestationRole::ComputationProof) {
            receipt.attestations.push(attestation(
                AttestationRole::ComputationProof,
                statement,
                c(51),
            ));
        }
        let policy = accountability_policy(required_roles.to_vec());
        verify_finalized_receipt_with_trust(
            &receipt,
            &policy,
            &trust_policy(required_roles),
            &BaseVerifier,
        )
        .unwrap()
    }

    fn full_base() -> TrustVerifiedReceipt {
        verified_base_with_roles(&[
            AttestationRole::ExecutionBinding,
            AttestationRole::ComputationProof,
        ])
    }

    fn full_policy() -> AccountabilityPolicy {
        accountability_policy(vec![
            AttestationRole::ExecutionBinding,
            AttestationRole::ComputationProof,
        ])
    }

    fn witness_policy() -> ExternalWitnessTrustPolicy {
        ExternalWitnessTrustPolicy {
            min_verified_witnesses: 1,
            min_distinct_verifiers: 1,
            min_distinct_trust_domains: 1,
            require_verifier_disjoint_from_non_witness: true,
            require_trust_domain_disjoint_from_non_witness: true,
        }
    }

    fn accepting_witness_verifier() -> WitnessVerifier {
        WitnessVerifier {
            verifier_id: c(80),
            trust_domain_id: c(81),
            reject: false,
        }
    }

    fn finalized_with_witness(base: &TrustVerifiedReceipt, proof_digest: Commitment32) -> AccessReceipt {
        let mut receipt = base.receipt().clone();
        let bundle = pre_witness_evidence_bundle_commitment(base).unwrap();
        receipt.attestations.push(attestation(
            AttestationRole::ExternalWitness,
            bundle,
            proof_digest,
        ));
        receipt
    }

    #[test]
    fn witness_over_receipt_statement_is_rejected() {
        let base = full_base();
        let mut receipt = base.receipt().clone();
        receipt.attestations.push(attestation(
            AttestationRole::ExternalWitness,
            base.verification_context().statement_digest,
            c(90),
        ));
        assert_eq!(
            verify_pre_disclosure_evidence_bundle(
                &base,
                &receipt,
                &full_policy(),
                &witness_policy(),
                &accepting_witness_verifier(),
            ),
            Err(BundleVerificationError::WitnessStatementMismatch)
        );
    }

    #[test]
    fn witness_from_older_bundle_missing_computation_proof_is_rejected() {
        let full = full_base();
        let older = verified_base_with_roles(&[AttestationRole::ExecutionBinding]);
        let stale_bundle = pre_witness_evidence_bundle_commitment(&older).unwrap();
        let mut receipt = full.receipt().clone();
        receipt.attestations.push(attestation(
            AttestationRole::ExternalWitness,
            stale_bundle,
            c(90),
        ));
        assert_eq!(
            verify_pre_disclosure_evidence_bundle(
                &full,
                &receipt,
                &full_policy(),
                &witness_policy(),
                &accepting_witness_verifier(),
            ),
            Err(BundleVerificationError::WitnessStatementMismatch)
        );
    }

    #[test]
    fn changing_non_witness_proof_changes_pre_witness_bundle() {
        let first = full_base();
        let mut receipt = first.receipt().clone();
        receipt.attestations[1].proof_digest = c(52);
        let changed = verify_finalized_receipt_with_trust(
            &receipt,
            &full_policy(),
            &trust_policy(&[
                AttestationRole::ExecutionBinding,
                AttestationRole::ComputationProof,
            ]),
            &BaseVerifier,
        )
        .unwrap();
        assert_ne!(
            pre_witness_evidence_bundle_commitment(&first).unwrap(),
            pre_witness_evidence_bundle_commitment(&changed).unwrap()
        );
    }

    #[test]
    fn reordering_equivalent_non_witness_refs_does_not_change_bundle() {
        let first = full_base();
        let mut receipt = first.receipt().clone();
        receipt.attestations.swap(0, 1);
        let reordered = verify_finalized_receipt_with_trust(
            &receipt,
            &full_policy(),
            &trust_policy(&[
                AttestationRole::ExecutionBinding,
                AttestationRole::ComputationProof,
            ]),
            &BaseVerifier,
        )
        .unwrap();
        assert_eq!(
            pre_witness_evidence_bundle_commitment(&first).unwrap(),
            pre_witness_evidence_bundle_commitment(&reordered).unwrap()
        );
    }

    #[test]
    fn adding_witness_ref_does_not_change_pre_witness_bundle() {
        let base = full_base();
        let expected = pre_witness_evidence_bundle_commitment(&base).unwrap();
        let receipt = finalized_with_witness(&base, c(90));
        let verified = verify_pre_disclosure_evidence_bundle(
            &base,
            &receipt,
            &full_policy(),
            &witness_policy(),
            &accepting_witness_verifier(),
        )
        .unwrap();
        assert_eq!(verified.pre_witness_bundle_digest(), expected);
    }

    #[test]
    fn duplicate_witness_proof_digest_cannot_inflate_threshold() {
        let base = full_base();
        let mut receipt = finalized_with_witness(&base, c(90));
        let bundle = pre_witness_evidence_bundle_commitment(&base).unwrap();
        receipt.attestations.push(attestation(
            AttestationRole::ExternalWitness,
            bundle,
            c(90),
        ));
        assert_eq!(
            verify_pre_disclosure_evidence_bundle(
                &base,
                &receipt,
                &full_policy(),
                &witness_policy(),
                &accepting_witness_verifier(),
            ),
            Err(BundleVerificationError::DuplicateProofDigest)
        );
    }

    #[test]
    fn final_archival_digest_changes_when_witness_artifact_changes() {
        let base = full_base();
        let first = verify_pre_disclosure_evidence_bundle(
            &base,
            &finalized_with_witness(&base, c(90)),
            &full_policy(),
            &witness_policy(),
            &accepting_witness_verifier(),
        )
        .unwrap();
        let second = verify_pre_disclosure_evidence_bundle(
            &base,
            &finalized_with_witness(&base, c(91)),
            &full_policy(),
            &witness_policy(),
            &accepting_witness_verifier(),
        )
        .unwrap();
        assert_ne!(
            first.finalized_evidence_bundle_digest(),
            second.finalized_evidence_bundle_digest()
        );
    }

    #[test]
    fn high_assurance_type_is_unavailable_until_witness_verifies() {
        let base = full_base();
        let no_witness = base.receipt().clone();
        assert_eq!(
            verify_pre_disclosure_evidence_bundle(
                &base,
                &no_witness,
                &full_policy(),
                &witness_policy(),
                &accepting_witness_verifier(),
            ),
            Err(BundleVerificationError::InsufficientWitnesses {
                found: 0,
                required: 1,
            })
        );

        let receipt = finalized_with_witness(&base, c(90));
        let rejecting = WitnessVerifier {
            verifier_id: c(80),
            trust_domain_id: c(81),
            reject: true,
        };
        assert!(matches!(
            verify_pre_disclosure_evidence_bundle(
                &base,
                &receipt,
                &full_policy(),
                &witness_policy(),
                &rejecting,
            ),
            Err(BundleVerificationError::WitnessCryptographicVerificationFailed { .. })
        ));
    }

    #[test]
    fn witness_cannot_share_non_witness_trust_domain() {
        let base = full_base();
        let receipt = finalized_with_witness(&base, c(90));
        let same_domain = WitnessVerifier {
            verifier_id: c(80),
            trust_domain_id: c(70),
            reject: false,
        };
        assert_eq!(
            verify_pre_disclosure_evidence_bundle(
                &base,
                &receipt,
                &full_policy(),
                &witness_policy(),
                &same_domain,
            ),
            Err(BundleVerificationError::WitnessTrustDomainOverlapsNonWitness)
        );
    }
}

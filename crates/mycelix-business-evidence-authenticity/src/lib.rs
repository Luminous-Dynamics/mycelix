// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Reference-only authenticity contracts for Business evidence.
//!
//! Business does not verify signatures or credentials here. It consumes a receipt issued by an
//! external verifier such as Xenia/Identity and binds that receipt to one exact evidence digest,
//! issuer, credential epoch, revocation frontier, verifier epoch, and validity window.

use mycelix_business_core::{Digest32, ReferenceId};
use sha2::{Digest, Sha256};

pub const AUTHENTICITY_IS_EXTERNALLY_VERIFIED: bool = true;
pub const BUSINESS_DOES_NOT_VERIFY_CRYPTOGRAPHY: bool = true;
pub const AUTHENTICITY_GRANTS_NO_BUSINESS_AUTHORITY: bool = true;

fn zero_digest(value: &Digest32) -> bool {
    value == &Digest32([0; 32])
}

fn hash_str(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> Digest32 {
    Digest32(hasher.finalize().into())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedEvidenceAuthenticityRef {
    pub subject_digest: Digest32,
    pub claimed_issuer: ReferenceId,
    pub verifier_domain: ReferenceId,
    pub verifier_receipt_digest: Digest32,
    pub verification_method: ReferenceId,
    pub verification_policy_digest: Digest32,
    pub verifier_epoch: u64,
    pub receipt_sequence: u64,
    pub credential: ReferenceId,
    pub credential_epoch: u64,
    pub revocation_frontier_digest: Digest32,
    pub verified_at_unix_ms: u64,
    pub valid_until_unix_ms: u64,
    pub binding_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AuthenticityError {
    ZeroSubjectDigest,
    ZeroReceiptDigest,
    ZeroPolicyDigest,
    ZeroRevocationFrontier,
    ZeroVerifierEpoch,
    ZeroReceiptSequence,
    ZeroCredentialEpoch,
    InvalidValidityWindow,
    BindingDigestMismatch,
    SubjectMismatch,
    IssuerMismatch,
    NotYetValid,
    Expired,
    VerifierEpochChanged { bound: u64, current: u64 },
    CredentialEpochChanged { bound: u64, current: u64 },
    RevocationFrontierChanged,
}

impl VerifiedEvidenceAuthenticityRef {
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        subject_digest: Digest32,
        claimed_issuer: ReferenceId,
        verifier_domain: ReferenceId,
        verifier_receipt_digest: Digest32,
        verification_method: ReferenceId,
        verification_policy_digest: Digest32,
        verifier_epoch: u64,
        receipt_sequence: u64,
        credential: ReferenceId,
        credential_epoch: u64,
        revocation_frontier_digest: Digest32,
        verified_at_unix_ms: u64,
        valid_until_unix_ms: u64,
    ) -> Result<Self, AuthenticityError> {
        let mut value = Self {
            subject_digest,
            claimed_issuer,
            verifier_domain,
            verifier_receipt_digest,
            verification_method,
            verification_policy_digest,
            verifier_epoch,
            receipt_sequence,
            credential,
            credential_epoch,
            revocation_frontier_digest,
            verified_at_unix_ms,
            valid_until_unix_ms,
            binding_digest: Digest32([0; 32]),
        };
        value.binding_digest = authenticity_binding_digest(&value);
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), AuthenticityError> {
        if zero_digest(&self.subject_digest) {
            return Err(AuthenticityError::ZeroSubjectDigest);
        }
        if zero_digest(&self.verifier_receipt_digest) {
            return Err(AuthenticityError::ZeroReceiptDigest);
        }
        if zero_digest(&self.verification_policy_digest) {
            return Err(AuthenticityError::ZeroPolicyDigest);
        }
        if zero_digest(&self.revocation_frontier_digest) {
            return Err(AuthenticityError::ZeroRevocationFrontier);
        }
        if self.verifier_epoch == 0 {
            return Err(AuthenticityError::ZeroVerifierEpoch);
        }
        if self.receipt_sequence == 0 {
            return Err(AuthenticityError::ZeroReceiptSequence);
        }
        if self.credential_epoch == 0 {
            return Err(AuthenticityError::ZeroCredentialEpoch);
        }
        if self.verified_at_unix_ms == 0 || self.verified_at_unix_ms >= self.valid_until_unix_ms {
            return Err(AuthenticityError::InvalidValidityWindow);
        }
        if zero_digest(&self.binding_digest)
            || self.binding_digest != authenticity_binding_digest(self)
        {
            return Err(AuthenticityError::BindingDigestMismatch);
        }
        Ok(())
    }

    pub fn binds_subject_and_issuer(
        &self,
        subject_digest: Digest32,
        claimed_issuer: &ReferenceId,
    ) -> Result<(), AuthenticityError> {
        self.validate()?;
        if self.subject_digest != subject_digest {
            return Err(AuthenticityError::SubjectMismatch);
        }
        if &self.claimed_issuer != claimed_issuer {
            return Err(AuthenticityError::IssuerMismatch);
        }
        Ok(())
    }

    pub fn validate_at(
        &self,
        now_unix_ms: u64,
        current_verifier_epoch: u64,
        current_credential_epoch: u64,
        current_revocation_frontier_digest: Digest32,
    ) -> Result<(), AuthenticityError> {
        self.validate()?;
        if current_verifier_epoch != self.verifier_epoch {
            return Err(AuthenticityError::VerifierEpochChanged {
                bound: self.verifier_epoch,
                current: current_verifier_epoch,
            });
        }
        if current_credential_epoch != self.credential_epoch {
            return Err(AuthenticityError::CredentialEpochChanged {
                bound: self.credential_epoch,
                current: current_credential_epoch,
            });
        }
        if current_revocation_frontier_digest != self.revocation_frontier_digest {
            return Err(AuthenticityError::RevocationFrontierChanged);
        }
        if now_unix_ms < self.verified_at_unix_ms {
            return Err(AuthenticityError::NotYetValid);
        }
        if now_unix_ms >= self.valid_until_unix_ms {
            return Err(AuthenticityError::Expired);
        }
        Ok(())
    }
}

fn authenticity_binding_digest(value: &VerifiedEvidenceAuthenticityRef) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:verified-evidence-authenticity-ref:v1");
    hasher.update(value.subject_digest.0);
    hash_str(&mut hasher, value.claimed_issuer.as_str());
    hash_str(&mut hasher, value.verifier_domain.as_str());
    hasher.update(value.verifier_receipt_digest.0);
    hash_str(&mut hasher, value.verification_method.as_str());
    hasher.update(value.verification_policy_digest.0);
    hasher.update(value.verifier_epoch.to_be_bytes());
    hasher.update(value.receipt_sequence.to_be_bytes());
    hash_str(&mut hasher, value.credential.as_str());
    hasher.update(value.credential_epoch.to_be_bytes());
    hasher.update(value.revocation_frontier_digest.0);
    hasher.update(value.verified_at_unix_ms.to_be_bytes());
    hasher.update(value.valid_until_unix_ms.to_be_bytes());
    finish_digest(hasher)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn authenticity() -> VerifiedEvidenceAuthenticityRef {
        VerifiedEvidenceAuthenticityRef::build(
            Digest32::repeat(1),
            id("issuer:provider"),
            id("verifier:xenia"),
            Digest32::repeat(2),
            id("method:signature-verification:v1"),
            Digest32::repeat(3),
            7,
            11,
            id("credential:provider-signing-key"),
            4,
            Digest32::repeat(5),
            1_000,
            2_000,
        )
        .unwrap()
    }

    #[test]
    fn exact_subject_and_issuer_bind() {
        let value = authenticity();
        assert!(value
            .binds_subject_and_issuer(Digest32::repeat(1), &id("issuer:provider"))
            .is_ok());
        assert!(matches!(
            value.binds_subject_and_issuer(Digest32::repeat(9), &id("issuer:provider")),
            Err(AuthenticityError::SubjectMismatch)
        ));
    }

    #[test]
    fn epoch_or_revocation_drift_requires_reverification() {
        let value = authenticity();
        assert!(value
            .validate_at(1_500, 7, 4, Digest32::repeat(5))
            .is_ok());
        assert!(matches!(
            value.validate_at(1_500, 8, 4, Digest32::repeat(5)),
            Err(AuthenticityError::VerifierEpochChanged { .. })
        ));
        assert!(matches!(
            value.validate_at(1_500, 7, 4, Digest32::repeat(6)),
            Err(AuthenticityError::RevocationFrontierChanged)
        ));
    }

    #[test]
    fn expired_receipt_fails_closed() {
        let value = authenticity();
        assert!(matches!(
            value.validate_at(2_000, 7, 4, Digest32::repeat(5)),
            Err(AuthenticityError::Expired)
        ));
    }

    #[test]
    fn metadata_tampering_breaks_binding_digest() {
        let mut value = authenticity();
        value.credential_epoch += 1;
        assert!(matches!(
            value.validate(),
            Err(AuthenticityError::BindingDigestMismatch)
        ));
    }
}

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002B2A — provider-neutral registry subject and structural evidence core.
//!
//! This crate defines exact registry snapshot subjects, verifier trust policy,
//! and structural observation consistency only. It intentionally does **not**
//! verify provider cryptography or establish currentness.
//!
//! Governing boundary:
//!
//! ```text
//! exact subject commitment
//! + trusted provider/verifier pair in supplied policy
//! + structurally matching provider observation
//!     != provider cryptographically verified
//!     != registry authenticated
//!     != registry current
//!     != contact-discovery composition qualified
//! ```

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub const REGISTRY_SUBJECT_DOMAIN: &str = "mycelix-psi-registry-snapshot-v1";
pub const REGISTRY_TRUST_POLICY_DOMAIN: &str = "mycelix-psi-registry-trust-policy-v1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RegistrySnapshotSubjectV1 {
    pub service_domain: String,
    pub snapshot_sha256: String,
    pub registry_epoch: String,
    pub sequence: u64,
    pub previous_snapshot_commitment_sha256: Option<String>,
    pub voprf_key_epoch: String,
    pub voprf_backend_profile: String,
    pub canonicalization_profile: String,
    pub equality_domain: String,
    pub construction_profile: String,
}

impl RegistrySnapshotSubjectV1 {
    pub fn validate(&self) -> Result<(), RegistryEvidenceError> {
        if !nonempty(&self.service_domain) {
            return Err(RegistryEvidenceError::EmptyServiceDomain);
        }
        if !is_sha256_hex(&self.snapshot_sha256) {
            return Err(RegistryEvidenceError::InvalidSnapshotDigest);
        }
        if !nonempty(&self.registry_epoch) {
            return Err(RegistryEvidenceError::EmptyRegistryEpoch);
        }
        if self.sequence == 0 && self.previous_snapshot_commitment_sha256.is_some() {
            return Err(RegistryEvidenceError::GenesisHasPredecessor);
        }
        if self.sequence > 0 && self.previous_snapshot_commitment_sha256.is_none() {
            return Err(RegistryEvidenceError::MissingPredecessor);
        }
        if let Some(previous) = &self.previous_snapshot_commitment_sha256 {
            if !is_sha256_hex(previous) {
                return Err(RegistryEvidenceError::InvalidPredecessorDigest);
            }
        }
        for value in [
            &self.voprf_key_epoch,
            &self.voprf_backend_profile,
            &self.canonicalization_profile,
            &self.equality_domain,
            &self.construction_profile,
        ] {
            if !nonempty(value) {
                return Err(RegistryEvidenceError::EmptySemanticIdentity);
            }
        }
        Ok(())
    }

    pub fn commitment_sha256(&self) -> Result<String, RegistryEvidenceError> {
        self.validate()?;
        let mut bytes = Vec::new();
        append_field(&mut bytes, REGISTRY_SUBJECT_DOMAIN.as_bytes());
        append_field(&mut bytes, self.service_domain.as_bytes());
        append_field(&mut bytes, self.snapshot_sha256.as_bytes());
        append_field(&mut bytes, self.registry_epoch.as_bytes());
        append_field(&mut bytes, &self.sequence.to_be_bytes());
        match &self.previous_snapshot_commitment_sha256 {
            Some(previous) => {
                append_field(&mut bytes, b"previous-present");
                append_field(&mut bytes, previous.as_bytes());
            }
            None => append_field(&mut bytes, b"previous-none"),
        }
        append_field(&mut bytes, self.voprf_key_epoch.as_bytes());
        append_field(&mut bytes, self.voprf_backend_profile.as_bytes());
        append_field(&mut bytes, self.canonicalization_profile.as_bytes());
        append_field(&mut bytes, self.equality_domain.as_bytes());
        append_field(&mut bytes, self.construction_profile.as_bytes());
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct TrustedProviderVerifierV1 {
    pub provider_namespace: String,
    pub verifier_identity_sha256: String,
}

impl TrustedProviderVerifierV1 {
    fn validate(&self) -> Result<(), RegistryEvidenceError> {
        if !nonempty(&self.provider_namespace) {
            return Err(RegistryEvidenceError::EmptyProviderNamespace);
        }
        if !is_sha256_hex(&self.verifier_identity_sha256) {
            return Err(RegistryEvidenceError::InvalidVerifierIdentity);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RegistryTrustPolicyV1 {
    pub service_domain: String,
    pub sequence: u64,
    pub previous_policy_commitment_sha256: Option<String>,
    pub trusted_verifiers: Vec<TrustedProviderVerifierV1>,
}

impl RegistryTrustPolicyV1 {
    pub fn new(
        service_domain: impl Into<String>,
        sequence: u64,
        previous_policy_commitment_sha256: Option<String>,
        mut trusted_verifiers: Vec<TrustedProviderVerifierV1>,
    ) -> Result<Self, RegistryEvidenceError> {
        trusted_verifiers.sort();
        let value = Self {
            service_domain: service_domain.into(),
            sequence,
            previous_policy_commitment_sha256,
            trusted_verifiers,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), RegistryEvidenceError> {
        if !nonempty(&self.service_domain) {
            return Err(RegistryEvidenceError::EmptyServiceDomain);
        }
        if self.sequence == 0 && self.previous_policy_commitment_sha256.is_some() {
            return Err(RegistryEvidenceError::GenesisPolicyHasPredecessor);
        }
        if self.sequence > 0 && self.previous_policy_commitment_sha256.is_none() {
            return Err(RegistryEvidenceError::MissingPolicyPredecessor);
        }
        if let Some(previous) = &self.previous_policy_commitment_sha256 {
            if !is_sha256_hex(previous) {
                return Err(RegistryEvidenceError::InvalidPolicyPredecessorDigest);
            }
        }
        if self.trusted_verifiers.is_empty() {
            return Err(RegistryEvidenceError::EmptyTrustedVerifierSet);
        }
        for verifier in &self.trusted_verifiers {
            verifier.validate()?;
        }
        if !self.trusted_verifiers.windows(2).all(|w| w[0] < w[1]) {
            return Err(RegistryEvidenceError::NonCanonicalOrDuplicateVerifierSet);
        }
        Ok(())
    }

    pub fn commitment_sha256(&self) -> Result<String, RegistryEvidenceError> {
        self.validate()?;
        let mut bytes = Vec::new();
        append_field(&mut bytes, REGISTRY_TRUST_POLICY_DOMAIN.as_bytes());
        append_field(&mut bytes, self.service_domain.as_bytes());
        append_field(&mut bytes, &self.sequence.to_be_bytes());
        match &self.previous_policy_commitment_sha256 {
            Some(previous) => {
                append_field(&mut bytes, b"previous-present");
                append_field(&mut bytes, previous.as_bytes());
            }
            None => append_field(&mut bytes, b"previous-none"),
        }
        for verifier in &self.trusted_verifiers {
            append_field(&mut bytes, verifier.provider_namespace.as_bytes());
            append_field(&mut bytes, verifier.verifier_identity_sha256.as_bytes());
        }
        Ok(sha256_hex(&bytes))
    }

    fn trusts(&self, provider_namespace: &str, verifier_identity_sha256: &str) -> bool {
        self.trusted_verifiers.iter().any(|entry| {
            entry.provider_namespace == provider_namespace
                && entry.verifier_identity_sha256 == verifier_identity_sha256
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProviderClaimDisposition {
    Unspecified,
    ClaimedPositive,
    ClaimedNegative,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RegistryProviderObservationV1 {
    pub subject_commitment_sha256: String,
    pub provider_namespace: String,
    pub verifier_identity_sha256: String,
    pub verification_profile: String,
    pub provider_receipt_commitment_sha256: String,
    pub observed_sequence: u64,
    pub authenticity: ProviderClaimDisposition,
    pub currentness: ProviderClaimDisposition,
}

impl RegistryProviderObservationV1 {
    fn validate(&self) -> Result<(), RegistryEvidenceError> {
        if !is_sha256_hex(&self.subject_commitment_sha256) {
            return Err(RegistryEvidenceError::InvalidSubjectCommitment);
        }
        if !nonempty(&self.provider_namespace) {
            return Err(RegistryEvidenceError::EmptyProviderNamespace);
        }
        if !is_sha256_hex(&self.verifier_identity_sha256) {
            return Err(RegistryEvidenceError::InvalidVerifierIdentity);
        }
        if !nonempty(&self.verification_profile) {
            return Err(RegistryEvidenceError::EmptyVerificationProfile);
        }
        if !is_sha256_hex(&self.provider_receipt_commitment_sha256) {
            return Err(RegistryEvidenceError::InvalidProviderReceiptCommitment);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct StructurallyConsistentRegistryObservationV1 {
    subject_commitment_sha256: String,
    trust_policy_commitment_sha256: String,
    provider_namespace: String,
    verifier_identity_sha256: String,
    provider_receipt_commitment_sha256: String,
    authenticity_claim: ProviderClaimDisposition,
    currentness_claim: ProviderClaimDisposition,
}

impl StructurallyConsistentRegistryObservationV1 {
    pub fn subject_commitment_sha256(&self) -> &str {
        &self.subject_commitment_sha256
    }

    pub fn trust_policy_commitment_sha256(&self) -> &str {
        &self.trust_policy_commitment_sha256
    }

    pub const fn provider_cryptographically_verified(&self) -> bool {
        false
    }

    pub const fn registry_authenticated(&self) -> bool {
        false
    }

    pub const fn registry_current(&self) -> bool {
        false
    }

    pub const fn contact_discovery_composition_qualified(&self) -> bool {
        false
    }
}

pub fn bind_structurally_consistent_observation_v1(
    subject: &RegistrySnapshotSubjectV1,
    policy: &RegistryTrustPolicyV1,
    observation: &RegistryProviderObservationV1,
) -> Result<StructurallyConsistentRegistryObservationV1, RegistryEvidenceError> {
    subject.validate()?;
    policy.validate()?;
    observation.validate()?;

    if policy.service_domain != subject.service_domain {
        return Err(RegistryEvidenceError::PolicyServiceDomainMismatch);
    }
    let subject_commitment = subject.commitment_sha256()?;
    if observation.subject_commitment_sha256 != subject_commitment {
        return Err(RegistryEvidenceError::ObservationSubjectMismatch);
    }
    if observation.observed_sequence != subject.sequence {
        return Err(RegistryEvidenceError::ObservationSequenceMismatch);
    }
    if !policy.trusts(
        &observation.provider_namespace,
        &observation.verifier_identity_sha256,
    ) {
        return Err(RegistryEvidenceError::VerifierNotTrustedForProvider);
    }

    Ok(StructurallyConsistentRegistryObservationV1 {
        subject_commitment_sha256: subject_commitment,
        trust_policy_commitment_sha256: policy.commitment_sha256()?,
        provider_namespace: observation.provider_namespace.clone(),
        verifier_identity_sha256: observation.verifier_identity_sha256.clone(),
        provider_receipt_commitment_sha256: observation.provider_receipt_commitment_sha256.clone(),
        authenticity_claim: observation.authenticity,
        currentness_claim: observation.currentness,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum RegistryEvidenceError {
    EmptyServiceDomain,
    InvalidSnapshotDigest,
    EmptyRegistryEpoch,
    GenesisHasPredecessor,
    MissingPredecessor,
    InvalidPredecessorDigest,
    EmptySemanticIdentity,
    EmptyProviderNamespace,
    InvalidVerifierIdentity,
    GenesisPolicyHasPredecessor,
    MissingPolicyPredecessor,
    InvalidPolicyPredecessorDigest,
    EmptyTrustedVerifierSet,
    NonCanonicalOrDuplicateVerifierSet,
    InvalidSubjectCommitment,
    EmptyVerificationProfile,
    InvalidProviderReceiptCommitment,
    PolicyServiceDomainMismatch,
    ObservationSubjectMismatch,
    ObservationSequenceMismatch,
    VerifierNotTrustedForProvider,
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn append_field(out: &mut Vec<u8>, field: &[u8]) {
    let len = u32::try_from(field.len()).expect("registry semantic fields are bounded below u32::MAX");
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(field);
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for byte in digest {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    fn subject(sequence: u64) -> RegistrySnapshotSubjectV1 {
        RegistrySnapshotSubjectV1 {
            service_domain: "contacts.mycelix.test".into(),
            snapshot_sha256: "11".repeat(32),
            registry_epoch: "registry-19".into(),
            sequence,
            previous_snapshot_commitment_sha256: (sequence > 0).then(|| "22".repeat(32)),
            voprf_key_epoch: "key-7".into(),
            voprf_backend_profile: "rfc9497-ristretto255-sha512-voprf-tagged-set-v1".into(),
            canonicalization_profile: "ascii-trim-lower-synthetic-contact-v1".into(),
            equality_domain: "synthetic-contact-id-ascii-lower-v1".into(),
            construction_profile: "voprf-tagged-set-v1".into(),
        }
    }

    fn verifier(provider: &str, byte: &str) -> TrustedProviderVerifierV1 {
        TrustedProviderVerifierV1 {
            provider_namespace: provider.into(),
            verifier_identity_sha256: byte.repeat(32),
        }
    }

    fn policy() -> RegistryTrustPolicyV1 {
        RegistryTrustPolicyV1::new(
            "contacts.mycelix.test",
            0,
            None,
            vec![verifier("xenia", "33"), verifier("holochain", "44")],
        )
        .unwrap()
    }

    fn observation(subject: &RegistrySnapshotSubjectV1) -> RegistryProviderObservationV1 {
        RegistryProviderObservationV1 {
            subject_commitment_sha256: subject.commitment_sha256().unwrap(),
            provider_namespace: "xenia".into(),
            verifier_identity_sha256: "33".repeat(32),
            verification_profile: "xenia-registry-provider-v1".into(),
            provider_receipt_commitment_sha256: "55".repeat(32),
            observed_sequence: subject.sequence,
            authenticity: ProviderClaimDisposition::ClaimedPositive,
            currentness: ProviderClaimDisposition::ClaimedPositive,
        }
    }

    #[test]
    fn subject_commitment_changes_with_key_epoch() {
        let a = subject(0);
        let mut b = a.clone();
        b.voprf_key_epoch = "key-8".into();
        assert_ne!(a.commitment_sha256().unwrap(), b.commitment_sha256().unwrap());
    }

    #[test]
    fn non_genesis_subject_requires_predecessor() {
        let mut value = subject(1);
        value.previous_snapshot_commitment_sha256 = None;
        assert_eq!(value.validate(), Err(RegistryEvidenceError::MissingPredecessor));
    }

    #[test]
    fn verifier_order_is_canonicalized() {
        let policy = RegistryTrustPolicyV1::new(
            "contacts.mycelix.test",
            0,
            None,
            vec![verifier("xenia", "33"), verifier("holochain", "44")],
        )
        .unwrap();
        assert!(policy.trusted_verifiers.windows(2).all(|w| w[0] < w[1]));
    }

    #[test]
    fn duplicate_verifier_pair_is_rejected() {
        let duplicate = verifier("xenia", "33");
        assert_eq!(
            RegistryTrustPolicyV1::new(
                "contacts.mycelix.test",
                0,
                None,
                vec![duplicate.clone(), duplicate],
            ),
            Err(RegistryEvidenceError::NonCanonicalOrDuplicateVerifierSet)
        );
    }

    #[test]
    fn verifier_identity_is_scoped_to_provider_namespace() {
        let subject = subject(0);
        let mut observation = observation(&subject);
        observation.provider_namespace = "other-provider".into();
        assert_eq!(
            bind_structurally_consistent_observation_v1(&subject, &policy(), &observation),
            Err(RegistryEvidenceError::VerifierNotTrustedForProvider)
        );
    }

    #[test]
    fn observation_cannot_borrow_provider_evidence_from_other_subject() {
        let subject_a = subject(0);
        let mut subject_b = subject(0);
        subject_b.snapshot_sha256 = "66".repeat(32);
        let observation = observation(&subject_a);
        assert_eq!(
            bind_structurally_consistent_observation_v1(&subject_b, &policy(), &observation),
            Err(RegistryEvidenceError::ObservationSubjectMismatch)
        );
    }

    #[test]
    fn structural_positive_never_promotes_provider_claims() {
        let subject = subject(0);
        let bound =
            bind_structurally_consistent_observation_v1(&subject, &policy(), &observation(&subject))
                .unwrap();
        assert!(!bound.provider_cryptographically_verified());
        assert!(!bound.registry_authenticated());
        assert!(!bound.registry_current());
        assert!(!bound.contact_discovery_composition_qualified());
    }

    #[test]
    fn policy_rotation_changes_policy_commitment() {
        let old = policy();
        let old_commitment = old.commitment_sha256().unwrap();
        let new = RegistryTrustPolicyV1::new(
            "contacts.mycelix.test",
            1,
            Some(old_commitment.clone()),
            vec![verifier("xenia", "77")],
        )
        .unwrap();
        assert_ne!(old_commitment, new.commitment_sha256().unwrap());
    }

    #[test]
    fn claimed_currentness_is_not_currentness_evidence() {
        let subject = subject(0);
        let bound =
            bind_structurally_consistent_observation_v1(&subject, &policy(), &observation(&subject))
                .unwrap();
        assert!(!bound.registry_current());
    }

    #[test]
    fn observation_sequence_must_match_exact_subject() {
        let subject = subject(0);
        let mut observation = observation(&subject);
        observation.observed_sequence = 1;
        assert_eq!(
            bind_structurally_consistent_observation_v1(&subject, &policy(), &observation),
            Err(RegistryEvidenceError::ObservationSequenceMismatch)
        );
    }
}

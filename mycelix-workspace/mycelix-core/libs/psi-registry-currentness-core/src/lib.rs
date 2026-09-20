// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002B2C r2 — provider-neutral structural currentness/completeness semantics.
//!
//! This crate evaluates caller-supplied head observations against one exact
//! currentness requirement. It deliberately cannot establish currentness.
//!
//! ```text
//! structurally consistent latest-head claims
//!     != provider evidence verified
//!     != trusted clock
//!     != completeness
//!     != registry current
//! ```

#![forbid(unsafe_code)]

use psi_registry_evidence_core::RegistrySnapshotSubjectV1;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

pub const CURRENTNESS_REQUIREMENT_DOMAIN_V1: &str = "mycelix-psi-registry-currentness-requirement-v1";
pub const HEAD_OBSERVATION_DOMAIN_V1: &str = "mycelix-psi-registry-head-observation-v1";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum HeadEvidenceClass {
    AppendOnlyHeadWitness,
    SignedHeadCheckpoint,
    TransparencyConsistencyProof,
    ProviderObservedHead,
    QuorumHeadAgreement,
}

impl HeadEvidenceClass {
    pub const fn wire_id(self) -> &'static str {
        match self {
            Self::AppendOnlyHeadWitness => "append-only-head-witness-v1",
            Self::SignedHeadCheckpoint => "signed-head-checkpoint-v1",
            Self::TransparencyConsistencyProof => "transparency-consistency-proof-v1",
            Self::ProviderObservedHead => "provider-observed-head-v1",
            Self::QuorumHeadAgreement => "quorum-head-agreement-v1",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CurrentnessRequirementV1 {
    pub service_domain: String,
    pub registry_epoch: String,
    pub evidence_profile: String,
    pub required_evidence_class: HeadEvidenceClass,
    /// Independence is counted by provider namespace, never verifier-key count.
    pub minimum_distinct_provider_namespaces: u16,
    pub maximum_observation_age_secs: Option<u64>,
    pub trusted_clock_profile: Option<String>,
}

impl CurrentnessRequirementV1 {
    pub fn validate(&self) -> Result<(), CurrentnessStructuralFailure> {
        if self.service_domain.trim().is_empty()
            || self.registry_epoch.trim().is_empty()
            || self.evidence_profile.trim().is_empty()
            || self.minimum_distinct_provider_namespaces == 0
        {
            return Err(CurrentnessStructuralFailure::InvalidRequirement);
        }
        if self.maximum_observation_age_secs.is_some()
            && self
                .trusted_clock_profile
                .as_deref()
                .is_none_or(|value| value.trim().is_empty())
        {
            return Err(CurrentnessStructuralFailure::ClockProfileRequired);
        }
        Ok(())
    }

    pub fn commitment_sha256(&self) -> Result<String, CurrentnessStructuralFailure> {
        self.validate()?;
        let mut bytes = Vec::new();
        append_field(&mut bytes, CURRENTNESS_REQUIREMENT_DOMAIN_V1.as_bytes());
        append_field(&mut bytes, self.service_domain.as_bytes());
        append_field(&mut bytes, self.registry_epoch.as_bytes());
        append_field(&mut bytes, self.evidence_profile.as_bytes());
        append_field(&mut bytes, self.required_evidence_class.wire_id().as_bytes());
        append_field(
            &mut bytes,
            &self.minimum_distinct_provider_namespaces.to_be_bytes(),
        );
        match self.maximum_observation_age_secs {
            Some(value) => {
                append_field(&mut bytes, b"max-age-present");
                append_field(&mut bytes, &value.to_be_bytes());
            }
            None => append_field(&mut bytes, b"max-age-none"),
        }
        match &self.trusted_clock_profile {
            Some(value) => {
                append_field(&mut bytes, b"clock-present");
                append_field(&mut bytes, value.as_bytes());
            }
            None => append_field(&mut bytes, b"clock-none"),
        }
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RegistryHeadObservationV1 {
    pub service_domain: String,
    pub registry_epoch: String,
    pub evidence_profile: String,
    pub evidence_class: HeadEvidenceClass,
    pub provider_namespace: String,
    pub verifier_identity_sha256: String,
    pub claimed_latest_sequence: u64,
    pub claimed_latest_subject_commitment_sha256: String,
    pub evidence_receipt_commitment_sha256: String,
    pub observed_at_unix_secs: Option<u64>,
    pub clock_profile: Option<String>,
}

impl RegistryHeadObservationV1 {
    pub fn validate(&self) -> Result<(), CurrentnessStructuralFailure> {
        if self.service_domain.trim().is_empty()
            || self.registry_epoch.trim().is_empty()
            || self.evidence_profile.trim().is_empty()
            || self.provider_namespace.trim().is_empty()
        {
            return Err(CurrentnessStructuralFailure::MalformedObservation);
        }
        if !is_sha256_hex(&self.verifier_identity_sha256)
            || !is_sha256_hex(&self.claimed_latest_subject_commitment_sha256)
            || !is_sha256_hex(&self.evidence_receipt_commitment_sha256)
        {
            return Err(CurrentnessStructuralFailure::MalformedObservation);
        }
        if self.observed_at_unix_secs.is_some()
            != self
                .clock_profile
                .as_deref()
                .is_some_and(|value| !value.trim().is_empty())
        {
            return Err(CurrentnessStructuralFailure::MalformedObservation);
        }
        Ok(())
    }

    pub fn commitment_sha256(&self) -> Result<String, CurrentnessStructuralFailure> {
        self.validate()?;
        let mut bytes = Vec::new();
        append_field(&mut bytes, HEAD_OBSERVATION_DOMAIN_V1.as_bytes());
        append_field(&mut bytes, self.service_domain.as_bytes());
        append_field(&mut bytes, self.registry_epoch.as_bytes());
        append_field(&mut bytes, self.evidence_profile.as_bytes());
        append_field(&mut bytes, self.evidence_class.wire_id().as_bytes());
        append_field(&mut bytes, self.provider_namespace.as_bytes());
        append_field(&mut bytes, self.verifier_identity_sha256.as_bytes());
        append_field(&mut bytes, &self.claimed_latest_sequence.to_be_bytes());
        append_field(
            &mut bytes,
            self.claimed_latest_subject_commitment_sha256.as_bytes(),
        );
        append_field(&mut bytes, self.evidence_receipt_commitment_sha256.as_bytes());
        match (self.observed_at_unix_secs, &self.clock_profile) {
            (Some(time), Some(profile)) => {
                append_field(&mut bytes, b"observation-time-present");
                append_field(&mut bytes, &time.to_be_bytes());
                append_field(&mut bytes, profile.as_bytes());
            }
            _ => append_field(&mut bytes, b"observation-time-none"),
        }
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClockReferenceV1 {
    pub unix_secs: u64,
    pub clock_profile: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CurrentnessStructuralDisposition {
    StructurallyConsistentLatestClaim,
    ClaimedStale,
    Conflict,
    IncompleteEvidence,
    UnsupportedProfile,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CurrentnessStructuralFailure {
    InvalidRequirement,
    ClockProfileRequired,
    EmptyEvidence,
    MalformedObservation,
    ServiceDomainMismatch,
    RegistryEpochMismatch,
    EvidenceProfileMismatch,
    EvidenceClassMismatch,
    DuplicateProviderNamespaceObservation,
    ConflictingHeadClaims,
    CandidateBehindClaim,
    CandidateAheadOfClaims,
    InsufficientDistinctProviderNamespaces,
    ClockReferenceRequired,
    ClockProfileMismatch,
    ObservationFromFuture,
    ObservationTooOld,
    SubjectCommitmentInvalid,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CurrentnessStructuralEvaluationV1 {
    pub disposition: CurrentnessStructuralDisposition,
    pub failure: Option<CurrentnessStructuralFailure>,
    pub requirement_commitment_sha256: String,
    pub candidate_subject_commitment_sha256: String,
    pub agreed_claimed_latest_sequence: Option<u64>,
    pub agreed_claimed_latest_subject_commitment_sha256: Option<String>,
    pub distinct_provider_namespace_count: u16,
}

impl CurrentnessStructuralEvaluationV1 {
    pub const fn registry_current(&self) -> bool { false }
    pub const fn completeness_established(&self) -> bool { false }
    pub const fn provider_evidence_verified(&self) -> bool { false }
    pub const fn contact_discovery_composition_qualified(&self) -> bool { false }
}

pub fn evaluate_currentness_claims_v1(
    candidate: &RegistrySnapshotSubjectV1,
    requirement: &CurrentnessRequirementV1,
    observations: &[RegistryHeadObservationV1],
    clock_reference: Option<&ClockReferenceV1>,
) -> Result<CurrentnessStructuralEvaluationV1, CurrentnessStructuralFailure> {
    requirement.validate()?;
    candidate
        .validate()
        .map_err(|_| CurrentnessStructuralFailure::SubjectCommitmentInvalid)?;
    let candidate_commitment = candidate
        .commitment_sha256()
        .map_err(|_| CurrentnessStructuralFailure::SubjectCommitmentInvalid)?;
    let requirement_commitment = requirement.commitment_sha256()?;

    if candidate.service_domain != requirement.service_domain {
        return Err(CurrentnessStructuralFailure::ServiceDomainMismatch);
    }
    if candidate.registry_epoch != requirement.registry_epoch {
        return Err(CurrentnessStructuralFailure::RegistryEpochMismatch);
    }
    if observations.is_empty() {
        return Ok(evaluation(
            CurrentnessStructuralDisposition::IncompleteEvidence,
            Some(CurrentnessStructuralFailure::EmptyEvidence),
            requirement_commitment,
            candidate_commitment,
            None,
            None,
            0,
        ));
    }

    let mut provider_namespaces = BTreeSet::new();
    let mut agreed_head: Option<(u64, String)> = None;

    for observation in observations {
        observation.validate()?;
        if observation.service_domain != requirement.service_domain {
            return Err(CurrentnessStructuralFailure::ServiceDomainMismatch);
        }
        if observation.registry_epoch != requirement.registry_epoch {
            return Err(CurrentnessStructuralFailure::RegistryEpochMismatch);
        }
        if observation.evidence_profile != requirement.evidence_profile {
            return Ok(evaluation(
                CurrentnessStructuralDisposition::UnsupportedProfile,
                Some(CurrentnessStructuralFailure::EvidenceProfileMismatch),
                requirement_commitment,
                candidate_commitment,
                None,
                None,
                provider_count(&provider_namespaces),
            ));
        }
        if observation.evidence_class != requirement.required_evidence_class {
            return Ok(evaluation(
                CurrentnessStructuralDisposition::UnsupportedProfile,
                Some(CurrentnessStructuralFailure::EvidenceClassMismatch),
                requirement_commitment,
                candidate_commitment,
                None,
                None,
                provider_count(&provider_namespaces),
            ));
        }

        if !provider_namespaces.insert(observation.provider_namespace.clone()) {
            return Ok(evaluation(
                CurrentnessStructuralDisposition::IncompleteEvidence,
                Some(CurrentnessStructuralFailure::DuplicateProviderNamespaceObservation),
                requirement_commitment,
                candidate_commitment,
                None,
                None,
                provider_count(&provider_namespaces),
            ));
        }

        if let Some(max_age) = requirement.maximum_observation_age_secs {
            let reference = clock_reference.ok_or(CurrentnessStructuralFailure::ClockReferenceRequired)?;
            let required_clock = requirement
                .trusted_clock_profile
                .as_ref()
                .ok_or(CurrentnessStructuralFailure::ClockProfileRequired)?;
            if reference.clock_profile != *required_clock
                || observation.clock_profile.as_ref() != Some(required_clock)
            {
                return Ok(evaluation(
                    CurrentnessStructuralDisposition::UnsupportedProfile,
                    Some(CurrentnessStructuralFailure::ClockProfileMismatch),
                    requirement_commitment,
                    candidate_commitment,
                    None,
                    None,
                    provider_count(&provider_namespaces),
                ));
            }
            let observed = observation
                .observed_at_unix_secs
                .ok_or(CurrentnessStructuralFailure::ClockReferenceRequired)?;
            if observed > reference.unix_secs {
                return Ok(evaluation(
                    CurrentnessStructuralDisposition::IncompleteEvidence,
                    Some(CurrentnessStructuralFailure::ObservationFromFuture),
                    requirement_commitment,
                    candidate_commitment,
                    None,
                    None,
                    provider_count(&provider_namespaces),
                ));
            }
            if reference.unix_secs - observed > max_age {
                return Ok(evaluation(
                    CurrentnessStructuralDisposition::IncompleteEvidence,
                    Some(CurrentnessStructuralFailure::ObservationTooOld),
                    requirement_commitment,
                    candidate_commitment,
                    None,
                    None,
                    provider_count(&provider_namespaces),
                ));
            }
        }

        let head = (
            observation.claimed_latest_sequence,
            observation.claimed_latest_subject_commitment_sha256.clone(),
        );
        if let Some(existing) = &agreed_head {
            if existing != &head {
                return Ok(evaluation(
                    CurrentnessStructuralDisposition::Conflict,
                    Some(CurrentnessStructuralFailure::ConflictingHeadClaims),
                    requirement_commitment,
                    candidate_commitment,
                    None,
                    None,
                    provider_count(&provider_namespaces),
                ));
            }
        } else {
            agreed_head = Some(head);
        }
    }

    let distinct = provider_count(&provider_namespaces);
    if distinct < requirement.minimum_distinct_provider_namespaces {
        return Ok(evaluation(
            CurrentnessStructuralDisposition::IncompleteEvidence,
            Some(CurrentnessStructuralFailure::InsufficientDistinctProviderNamespaces),
            requirement_commitment,
            candidate_commitment,
            agreed_head.as_ref().map(|value| value.0),
            agreed_head.as_ref().map(|value| value.1.clone()),
            distinct,
        ));
    }

    let (latest_sequence, latest_commitment) = agreed_head.expect("non-empty observations set head");
    if candidate.sequence < latest_sequence {
        return Ok(evaluation(
            CurrentnessStructuralDisposition::ClaimedStale,
            Some(CurrentnessStructuralFailure::CandidateBehindClaim),
            requirement_commitment,
            candidate_commitment,
            Some(latest_sequence),
            Some(latest_commitment),
            distinct,
        ));
    }
    if candidate.sequence > latest_sequence {
        return Ok(evaluation(
            CurrentnessStructuralDisposition::IncompleteEvidence,
            Some(CurrentnessStructuralFailure::CandidateAheadOfClaims),
            requirement_commitment,
            candidate_commitment,
            Some(latest_sequence),
            Some(latest_commitment),
            distinct,
        ));
    }
    if candidate_commitment != latest_commitment {
        return Ok(evaluation(
            CurrentnessStructuralDisposition::Conflict,
            Some(CurrentnessStructuralFailure::ConflictingHeadClaims),
            requirement_commitment,
            candidate_commitment,
            Some(latest_sequence),
            Some(latest_commitment),
            distinct,
        ));
    }

    Ok(evaluation(
        CurrentnessStructuralDisposition::StructurallyConsistentLatestClaim,
        None,
        requirement_commitment,
        candidate_commitment.clone(),
        Some(candidate.sequence),
        Some(candidate_commitment),
        distinct,
    ))
}

fn provider_count(providers: &BTreeSet<String>) -> u16 {
    u16::try_from(providers.len()).unwrap_or(u16::MAX)
}

fn evaluation(
    disposition: CurrentnessStructuralDisposition,
    failure: Option<CurrentnessStructuralFailure>,
    requirement_commitment_sha256: String,
    candidate_subject_commitment_sha256: String,
    agreed_claimed_latest_sequence: Option<u64>,
    agreed_claimed_latest_subject_commitment_sha256: Option<String>,
    distinct_provider_namespace_count: u16,
) -> CurrentnessStructuralEvaluationV1 {
    CurrentnessStructuralEvaluationV1 {
        disposition,
        failure,
        requirement_commitment_sha256,
        candidate_subject_commitment_sha256,
        agreed_claimed_latest_sequence,
        agreed_claimed_latest_subject_commitment_sha256,
        distinct_provider_namespace_count,
    }
}

fn append_field(out: &mut Vec<u8>, field: &[u8]) {
    let len = u32::try_from(field.len()).expect("currentness semantic fields are bounded below u32::MAX");
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(field);
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(64);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for byte in digest {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn subject(sequence: u64, previous: Option<String>) -> RegistrySnapshotSubjectV1 {
        RegistrySnapshotSubjectV1 {
            service_domain: "contacts.mycelix.test".into(),
            snapshot_sha256: "11".repeat(32),
            registry_epoch: "registry-19".into(),
            sequence,
            previous_snapshot_commitment_sha256: previous,
            voprf_key_epoch: "key-7".into(),
            voprf_backend_profile: "rfc9497-ristretto255-sha512-voprf-tagged-set-v1".into(),
            canonicalization_profile: "ascii-trim-lower-synthetic-contact-v1".into(),
            equality_domain: "synthetic-contact-id-ascii-lower-v1".into(),
            construction_profile: "voprf-tagged-set-v1".into(),
        }
    }

    fn requirement(minimum: u16) -> CurrentnessRequirementV1 {
        CurrentnessRequirementV1 {
            service_domain: "contacts.mycelix.test".into(),
            registry_epoch: "registry-19".into(),
            evidence_profile: "synthetic-head-profile-v1".into(),
            required_evidence_class: HeadEvidenceClass::ProviderObservedHead,
            minimum_distinct_provider_namespaces: minimum,
            maximum_observation_age_secs: None,
            trusted_clock_profile: None,
        }
    }

    fn observation(
        provider: &str,
        verifier_byte: &str,
        candidate: &RegistrySnapshotSubjectV1,
    ) -> RegistryHeadObservationV1 {
        RegistryHeadObservationV1 {
            service_domain: candidate.service_domain.clone(),
            registry_epoch: candidate.registry_epoch.clone(),
            evidence_profile: "synthetic-head-profile-v1".into(),
            evidence_class: HeadEvidenceClass::ProviderObservedHead,
            provider_namespace: provider.into(),
            verifier_identity_sha256: verifier_byte.repeat(32),
            claimed_latest_sequence: candidate.sequence,
            claimed_latest_subject_commitment_sha256: candidate.commitment_sha256().unwrap(),
            evidence_receipt_commitment_sha256: "55".repeat(32),
            observed_at_unix_secs: None,
            clock_profile: None,
        }
    }

    #[test]
    fn evidence_class_wire_ids_are_explicit_and_stable() {
        assert_eq!(HeadEvidenceClass::AppendOnlyHeadWitness.wire_id(), "append-only-head-witness-v1");
        assert_eq!(HeadEvidenceClass::SignedHeadCheckpoint.wire_id(), "signed-head-checkpoint-v1");
        assert_eq!(HeadEvidenceClass::TransparencyConsistencyProof.wire_id(), "transparency-consistency-proof-v1");
        assert_eq!(HeadEvidenceClass::ProviderObservedHead.wire_id(), "provider-observed-head-v1");
        assert_eq!(HeadEvidenceClass::QuorumHeadAgreement.wire_id(), "quorum-head-agreement-v1");
    }

    #[test]
    fn agreeing_claims_are_structural_only_not_currentness() {
        let candidate = subject(0, None);
        let observations = vec![
            observation("provider-a", "22", &candidate),
            observation("provider-b", "33", &candidate),
        ];
        let result = evaluate_currentness_claims_v1(&candidate, &requirement(2), &observations, None).unwrap();
        assert_eq!(result.disposition, CurrentnessStructuralDisposition::StructurallyConsistentLatestClaim);
        assert_eq!(result.distinct_provider_namespace_count, 2);
        assert!(!result.registry_current());
        assert!(!result.completeness_established());
        assert!(!result.provider_evidence_verified());
        assert!(!result.contact_discovery_composition_qualified());
    }

    #[test]
    fn same_provider_namespace_with_different_keys_cannot_amplify_quorum() {
        let candidate = subject(0, None);
        let result = evaluate_currentness_claims_v1(
            &candidate,
            &requirement(2),
            &[
                observation("provider-a", "22", &candidate),
                observation("provider-a", "33", &candidate),
            ],
            None,
        ).unwrap();
        assert_eq!(result.failure, Some(CurrentnessStructuralFailure::DuplicateProviderNamespaceObservation));
    }

    #[test]
    fn evidence_class_must_match_exact_requirement() {
        let candidate = subject(0, None);
        let mut obs = observation("provider-a", "22", &candidate);
        obs.evidence_class = HeadEvidenceClass::SignedHeadCheckpoint;
        let result = evaluate_currentness_claims_v1(&candidate, &requirement(1), &[obs], None).unwrap();
        assert_eq!(result.disposition, CurrentnessStructuralDisposition::UnsupportedProfile);
        assert_eq!(result.failure, Some(CurrentnessStructuralFailure::EvidenceClassMismatch));
    }

    #[test]
    fn empty_evidence_is_incomplete() {
        let candidate = subject(0, None);
        let result = evaluate_currentness_claims_v1(&candidate, &requirement(1), &[], None).unwrap();
        assert_eq!(result.failure, Some(CurrentnessStructuralFailure::EmptyEvidence));
    }

    #[test]
    fn same_sequence_different_subject_is_conflict() {
        let candidate = subject(0, None);
        let mut conflicting = observation("provider-b", "33", &candidate);
        conflicting.claimed_latest_subject_commitment_sha256 = "77".repeat(32);
        let result = evaluate_currentness_claims_v1(
            &candidate,
            &requirement(2),
            &[observation("provider-a", "22", &candidate), conflicting],
            None,
        ).unwrap();
        assert_eq!(result.disposition, CurrentnessStructuralDisposition::Conflict);
    }

    #[test]
    fn candidate_behind_unanimous_claim_is_only_claimed_stale() {
        let candidate = subject(0, None);
        let later = subject(1, Some(candidate.commitment_sha256().unwrap()));
        let result = evaluate_currentness_claims_v1(
            &candidate,
            &requirement(1),
            &[observation("provider-a", "22", &later)],
            None,
        ).unwrap();
        assert_eq!(result.disposition, CurrentnessStructuralDisposition::ClaimedStale);
        assert_eq!(result.failure, Some(CurrentnessStructuralFailure::CandidateBehindClaim));
    }

    #[test]
    fn candidate_ahead_of_observations_is_incomplete_not_current() {
        let genesis = subject(0, None);
        let candidate = subject(1, Some(genesis.commitment_sha256().unwrap()));
        let result = evaluate_currentness_claims_v1(
            &candidate,
            &requirement(1),
            &[observation("provider-a", "22", &genesis)],
            None,
        ).unwrap();
        assert_eq!(result.failure, Some(CurrentnessStructuralFailure::CandidateAheadOfClaims));
    }

    #[test]
    fn insufficient_distinct_provider_namespaces_is_incomplete() {
        let candidate = subject(0, None);
        let result = evaluate_currentness_claims_v1(
            &candidate,
            &requirement(2),
            &[observation("provider-a", "22", &candidate)],
            None,
        ).unwrap();
        assert_eq!(result.failure, Some(CurrentnessStructuralFailure::InsufficientDistinctProviderNamespaces));
    }

    #[test]
    fn max_age_requires_exact_clock_profile_and_rejects_stale_observation() {
        let candidate = subject(0, None);
        let mut req = requirement(1);
        req.maximum_observation_age_secs = Some(60);
        req.trusted_clock_profile = Some("clock-a".into());
        let mut obs = observation("provider-a", "22", &candidate);
        obs.observed_at_unix_secs = Some(100);
        obs.clock_profile = Some("clock-a".into());
        let clock = ClockReferenceV1 { unix_secs: 200, clock_profile: "clock-a".into() };
        let result = evaluate_currentness_claims_v1(&candidate, &req, &[obs], Some(&clock)).unwrap();
        assert_eq!(result.failure, Some(CurrentnessStructuralFailure::ObservationTooOld));
    }

    #[test]
    fn future_observation_is_incomplete() {
        let candidate = subject(0, None);
        let mut req = requirement(1);
        req.maximum_observation_age_secs = Some(60);
        req.trusted_clock_profile = Some("clock-a".into());
        let mut obs = observation("provider-a", "22", &candidate);
        obs.observed_at_unix_secs = Some(201);
        obs.clock_profile = Some("clock-a".into());
        let clock = ClockReferenceV1 { unix_secs: 200, clock_profile: "clock-a".into() };
        let result = evaluate_currentness_claims_v1(&candidate, &req, &[obs], Some(&clock)).unwrap();
        assert_eq!(result.failure, Some(CurrentnessStructuralFailure::ObservationFromFuture));
    }
}

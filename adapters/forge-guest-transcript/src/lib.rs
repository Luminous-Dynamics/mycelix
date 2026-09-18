// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D3B2C1: positive guest success transcript for Forge M0.
//!
//! This crate is intentionally pure. A transcript cannot qualify itself: the
//! exact concrete phase commitments are supplied separately to
//! [`qualify_guest_transcript`], while structural phase subjects are derived
//! from the exact guest plan.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_guest_plan::{
    GuestPhase, GuestPlanError, GuestVerificationPlanV1, GUEST_REPLAY_REPOSITORY,
    GUEST_TRANSCRIPT_PATH,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const TRANSCRIPT_DOMAIN_V1: &[u8] = b"mycelix-forge/guest-transcript/v1\0";
const QUALIFIED_DOMAIN_V1: &[u8] = b"mycelix-forge/guest-transcript-qualified/v1\0";
const REPLAY_PHASE_DOMAIN_V1: &[u8] = b"mycelix-forge/guest-phase/replay/v1\0";
const AMBIENT_PHASE_DOMAIN_V1: &[u8] = b"mycelix-forge/guest-phase/no-ambient-git/v1\0";
const GIT_VALIDATION_PHASE_DOMAIN_V1: &[u8] = b"mycelix-forge/guest-phase/git-validation/v1\0";
const OUTPUT_PHASE_DOMAIN_V1: &[u8] = b"mycelix-forge/guest-phase/transcript-output/v1\0";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GuestPhaseEvidence {
    phase: GuestPhase,
    evidence: Digest,
}

impl GuestPhaseEvidence {
    pub fn new(phase: GuestPhase, evidence: Digest) -> Self {
        Self { phase, evidence }
    }

    pub const fn phase(&self) -> GuestPhase {
        self.phase
    }

    pub fn evidence(&self) -> &Digest {
        &self.evidence
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct GuestTranscriptV1 {
    plan_digest: Digest,
    execution_subject: Digest,
    request_digest: Digest,
    run_challenge: Digest,
    phases: Vec<GuestPhaseEvidence>,
}

impl GuestTranscriptV1 {
    pub fn new(
        plan_digest: Digest,
        execution_subject: Digest,
        request_digest: Digest,
        run_challenge: Digest,
        phases: Vec<GuestPhaseEvidence>,
    ) -> Result<Self, GuestTranscriptError> {
        validate_phase_sequence(&phases)?;
        Ok(Self {
            plan_digest,
            execution_subject,
            request_digest,
            run_challenge,
            phases,
        })
    }

    pub fn plan_digest(&self) -> &Digest {
        &self.plan_digest
    }

    pub fn execution_subject(&self) -> &Digest {
        &self.execution_subject
    }

    pub fn request_digest(&self) -> &Digest {
        &self.request_digest
    }

    pub fn run_challenge(&self) -> &Digest {
        &self.run_challenge
    }

    pub fn phases(&self) -> &[GuestPhaseEvidence] {
        &self.phases
    }

    pub fn phase_evidence(&self, phase: GuestPhase) -> &Digest {
        self.phases
            .iter()
            .find(|entry| entry.phase == phase)
            .expect("exact guest phase sequence validated")
            .evidence()
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, GuestTranscriptError> {
        validate_phase_sequence(&self.phases)?;
        let mut out = Vec::new();
        out.extend_from_slice(TRANSCRIPT_DOMAIN_V1);
        push_digest(&mut out, &self.plan_digest)?;
        push_digest(&mut out, &self.execution_subject)?;
        push_digest(&mut out, &self.request_digest)?;
        push_digest(&mut out, &self.run_challenge)?;
        push_count(&mut out, self.phases.len(), "guest phases")?;
        for phase in &self.phases {
            out.push(phase_code(phase.phase));
            push_digest(&mut out, &phase.evidence)?;
        }
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, GuestTranscriptError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for GuestTranscriptV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            plan_digest: Digest,
            execution_subject: Digest,
            request_digest: Digest,
            run_challenge: Digest,
            phases: Vec<GuestPhaseEvidence>,
        }

        let wire = Wire::deserialize(deserializer)?;
        Self::new(
            wire.plan_digest,
            wire.execution_subject,
            wire.request_digest,
            wire.run_challenge,
            wire.phases,
        )
        .map_err(D::Error::custom)
    }
}

/// Concrete evidence commitments that a transcript is not permitted to choose
/// for itself. Structural phase subjects are derived independently from the
/// guest plan by the qualifier.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GuestTranscriptBindings {
    isolation_probe: Digest,
    policy_trust_inventory: Digest,
    local_trust_qualification: Digest,
    gittuf_verification: Digest,
}

impl GuestTranscriptBindings {
    pub fn new(
        isolation_probe: Digest,
        policy_trust_inventory: Digest,
        local_trust_qualification: Digest,
        gittuf_verification: Digest,
    ) -> Self {
        Self {
            isolation_probe,
            policy_trust_inventory,
            local_trust_qualification,
            gittuf_verification,
        }
    }

    pub fn isolation_probe(&self) -> &Digest {
        &self.isolation_probe
    }

    pub fn policy_trust_inventory(&self) -> &Digest {
        &self.policy_trust_inventory
    }

    pub fn local_trust_qualification(&self) -> &Digest {
        &self.local_trust_qualification
    }

    pub fn gittuf_verification(&self) -> &Digest {
        &self.gittuf_verification
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedGuestTranscript {
    plan_digest: Digest,
    transcript_digest: Digest,
    execution_subject: Digest,
    request_digest: Digest,
    run_challenge: Digest,
    isolation_probe: Digest,
    replay_bundle: Digest,
    ambient_object_sources: Digest,
    git_object_validation: Digest,
    policy_trust_inventory: Digest,
    local_trust_qualification: Digest,
    gittuf_verification: Digest,
    transcript_output: Digest,
    evidence_digest: Digest,
}

impl QualifiedGuestTranscript {
    pub fn plan_digest(&self) -> &Digest {
        &self.plan_digest
    }

    pub fn transcript_digest(&self) -> &Digest {
        &self.transcript_digest
    }

    pub fn execution_subject(&self) -> &Digest {
        &self.execution_subject
    }

    pub fn request_digest(&self) -> &Digest {
        &self.request_digest
    }

    pub fn run_challenge(&self) -> &Digest {
        &self.run_challenge
    }

    pub fn isolation_probe(&self) -> &Digest {
        &self.isolation_probe
    }

    pub fn replay_bundle(&self) -> &Digest {
        &self.replay_bundle
    }

    pub fn ambient_object_sources(&self) -> &Digest {
        &self.ambient_object_sources
    }

    pub fn git_object_validation(&self) -> &Digest {
        &self.git_object_validation
    }

    pub fn policy_trust_inventory(&self) -> &Digest {
        &self.policy_trust_inventory
    }

    pub fn local_trust_qualification(&self) -> &Digest {
        &self.local_trust_qualification
    }

    pub fn gittuf_verification(&self) -> &Digest {
        &self.gittuf_verification
    }

    pub fn transcript_output(&self) -> &Digest {
        &self.transcript_output
    }

    pub fn evidence_digest(&self) -> &Digest {
        &self.evidence_digest
    }
}

pub fn qualify_guest_transcript(
    plan: &GuestVerificationPlanV1,
    transcript: &GuestTranscriptV1,
    bindings: &GuestTranscriptBindings,
) -> Result<QualifiedGuestTranscript, GuestTranscriptError> {
    let expected_plan = plan.digest(DigestAlgorithm::Sha256)?;
    if transcript.plan_digest != expected_plan {
        return Err(GuestTranscriptError::PlanDigestMismatch);
    }
    if transcript.execution_subject != *plan.execution_subject() {
        return Err(GuestTranscriptError::ExecutionSubjectMismatch);
    }
    let request_digest = plan.request_digest()?;
    if transcript.request_digest != request_digest {
        return Err(GuestTranscriptError::RequestDigestMismatch);
    }
    if transcript.run_challenge != *plan.run_challenge() {
        return Err(GuestTranscriptError::RunChallengeMismatch);
    }
    if bindings.gittuf_verification != *plan.expected_replay_receipt() {
        return Err(GuestTranscriptError::ExpectedReplayReceiptMismatch);
    }

    let replay_bundle = replay_phase_subject(plan)?;
    let ambient_object_sources = ambient_sources_phase_subject(plan)?;
    let git_object_validation = git_validation_phase_subject(plan)?;
    let transcript_output = transcript_output_policy()?;

    let expected = [
        (GuestPhase::IsolationProbe, &bindings.isolation_probe),
        (GuestPhase::ReplayBundle, &replay_bundle),
        (
            GuestPhase::RejectAmbientObjectSources,
            &ambient_object_sources,
        ),
        (GuestPhase::GitObjectValidation, &git_object_validation),
        (
            GuestPhase::PolicyTrustInventory,
            &bindings.policy_trust_inventory,
        ),
        (
            GuestPhase::LocalTrustQualification,
            &bindings.local_trust_qualification,
        ),
        (GuestPhase::GittufVerification, &bindings.gittuf_verification),
        (GuestPhase::EmitTranscript, &transcript_output),
    ];
    for (phase, evidence) in expected {
        if transcript.phase_evidence(phase) != evidence {
            return Err(GuestTranscriptError::PhaseEvidenceMismatch(phase));
        }
    }

    let transcript_digest = transcript.digest(DigestAlgorithm::Sha256)?;
    let mut out = Vec::new();
    out.extend_from_slice(QUALIFIED_DOMAIN_V1);
    push_digest(&mut out, &expected_plan)?;
    push_digest(&mut out, &transcript_digest)?;
    for phase in GuestPhase::required_sequence() {
        push_digest(&mut out, transcript.phase_evidence(phase))?;
    }
    let evidence_digest = Digest::of_bytes(DigestAlgorithm::Sha256, &out);

    Ok(QualifiedGuestTranscript {
        plan_digest: expected_plan,
        transcript_digest,
        execution_subject: transcript.execution_subject.clone(),
        request_digest,
        run_challenge: transcript.run_challenge.clone(),
        isolation_probe: bindings.isolation_probe.clone(),
        replay_bundle,
        ambient_object_sources,
        git_object_validation,
        policy_trust_inventory: bindings.policy_trust_inventory.clone(),
        local_trust_qualification: bindings.local_trust_qualification.clone(),
        gittuf_verification: bindings.gittuf_verification.clone(),
        transcript_output,
        evidence_digest,
    })
}

pub fn replay_phase_subject(plan: &GuestVerificationPlanV1) -> Result<Digest, GuestTranscriptError> {
    derive_phase_subject(
        REPLAY_PHASE_DOMAIN_V1,
        &[
            plan.bundle_manifest(),
            &plan.request_digest()?,
            plan.run_challenge(),
        ],
    )
}

pub fn ambient_sources_phase_subject(
    plan: &GuestVerificationPlanV1,
) -> Result<Digest, GuestTranscriptError> {
    let mut out = Vec::new();
    out.extend_from_slice(AMBIENT_PHASE_DOMAIN_V1);
    push_digest(&mut out, plan.git_object_validation_policy())?;
    push_string(&mut out, GUEST_REPLAY_REPOSITORY, "replay repository")?;
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &out))
}

pub fn git_validation_phase_subject(
    plan: &GuestVerificationPlanV1,
) -> Result<Digest, GuestTranscriptError> {
    derive_phase_subject(
        GIT_VALIDATION_PHASE_DOMAIN_V1,
        &[
            plan.git_object_validation_policy(),
            plan.bundle_manifest(),
            &plan.request_digest()?,
        ],
    )
}

pub fn transcript_output_policy() -> Result<Digest, GuestTranscriptError> {
    let mut out = Vec::new();
    out.extend_from_slice(OUTPUT_PHASE_DOMAIN_V1);
    push_string(&mut out, GUEST_TRANSCRIPT_PATH, "transcript path")?;
    push_string(&mut out, "application/json", "transcript media type")?;
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &out))
}

fn derive_phase_subject(
    domain: &[u8],
    digests: &[&Digest],
) -> Result<Digest, GuestTranscriptError> {
    let mut out = Vec::new();
    out.extend_from_slice(domain);
    for digest in digests {
        push_digest(&mut out, digest)?;
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &out))
}

fn validate_phase_sequence(phases: &[GuestPhaseEvidence]) -> Result<(), GuestTranscriptError> {
    let expected = GuestPhase::required_sequence();
    if phases.len() != expected.len()
        || phases
            .iter()
            .zip(expected)
            .any(|(observed, required)| observed.phase != required)
    {
        return Err(GuestTranscriptError::InvalidPhaseSequence);
    }
    Ok(())
}

fn phase_code(phase: GuestPhase) -> u8 {
    match phase {
        GuestPhase::IsolationProbe => 1,
        GuestPhase::ReplayBundle => 2,
        GuestPhase::RejectAmbientObjectSources => 3,
        GuestPhase::GitObjectValidation => 4,
        GuestPhase::PolicyTrustInventory => 5,
        GuestPhase::LocalTrustQualification => 6,
        GuestPhase::GittufVerification => 7,
        GuestPhase::EmitTranscript => 8,
    }
}

fn push_count(
    out: &mut Vec<u8>,
    count: usize,
    field: &'static str,
) -> Result<(), GuestTranscriptError> {
    let count = u16::try_from(count)
        .map_err(|_| GuestTranscriptError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), GuestTranscriptError> {
    let len = u16::try_from(value.len())
        .map_err(|_| GuestTranscriptError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), GuestTranscriptError> {
    push_string(out, digest.algorithm().id(), "digest algorithm")?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| GuestTranscriptError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum GuestTranscriptError {
    #[error(transparent)]
    GuestPlan(#[from] GuestPlanError),
    #[error("guest transcript phases are missing, duplicated, extra, or out of order")]
    InvalidPhaseSequence,
    #[error("guest transcript names a different guest plan")]
    PlanDigestMismatch,
    #[error("guest transcript names a different execution subject")]
    ExecutionSubjectMismatch,
    #[error("guest transcript names a different repository request")]
    RequestDigestMismatch,
    #[error("guest transcript names a different run challenge")]
    RunChallengeMismatch,
    #[error("gittuf binding differs from the guest plan's expected replay receipt")]
    ExpectedReplayReceiptMismatch,
    #[error("guest phase evidence mismatch: {0:?}")]
    PhaseEvidenceMismatch(GuestPhase),
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{
        ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN,
    };
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryAdoption, RepositoryPolicyState,
        RepositoryRef, RepositoryTip, RepositoryVerificationRequest,
    };

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn git_sha1(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn request() -> RepositoryVerificationRequest {
        let project = ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x22)),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let adoption = RepositoryAdoption::new(
            project.clone(),
            RepositoryTip::new(
                RepositoryRef::new("refs/heads/main").unwrap(),
                git_sha1(0x33),
            ),
            digest(0x44),
            digest(0x55),
            digest(0x66),
            1_000,
        );
        let policy = RepositoryPolicyState::new(project, 0, None, digest(0x66)).unwrap();
        RepositoryVerificationRequest::new(
            &adoption,
            git_sha1(0x33),
            git_sha1(0x77),
            digest(0x44),
            digest(0x55),
            &policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn plan() -> GuestVerificationPlanV1 {
        GuestVerificationPlanV1::new(
            digest(1),
            request(),
            digest(3),
            digest(4),
            digest(5),
            digest(6),
            digest(7),
            digest(8),
            digest(9),
            digest(10),
        )
        .unwrap()
    }

    fn bindings(plan: &GuestVerificationPlanV1) -> GuestTranscriptBindings {
        GuestTranscriptBindings::new(
            digest(20),
            digest(21),
            digest(22),
            plan.expected_replay_receipt().clone(),
        )
    }

    fn transcript(
        plan: &GuestVerificationPlanV1,
        bindings: &GuestTranscriptBindings,
    ) -> GuestTranscriptV1 {
        let phases = vec![
            GuestPhaseEvidence::new(GuestPhase::IsolationProbe, bindings.isolation_probe.clone()),
            GuestPhaseEvidence::new(GuestPhase::ReplayBundle, replay_phase_subject(plan).unwrap()),
            GuestPhaseEvidence::new(
                GuestPhase::RejectAmbientObjectSources,
                ambient_sources_phase_subject(plan).unwrap(),
            ),
            GuestPhaseEvidence::new(
                GuestPhase::GitObjectValidation,
                git_validation_phase_subject(plan).unwrap(),
            ),
            GuestPhaseEvidence::new(
                GuestPhase::PolicyTrustInventory,
                bindings.policy_trust_inventory.clone(),
            ),
            GuestPhaseEvidence::new(
                GuestPhase::LocalTrustQualification,
                bindings.local_trust_qualification.clone(),
            ),
            GuestPhaseEvidence::new(
                GuestPhase::GittufVerification,
                bindings.gittuf_verification.clone(),
            ),
            GuestPhaseEvidence::new(
                GuestPhase::EmitTranscript,
                transcript_output_policy().unwrap(),
            ),
        ];
        GuestTranscriptV1::new(
            plan.digest(DigestAlgorithm::Sha256).unwrap(),
            plan.execution_subject().clone(),
            plan.request_digest().unwrap(),
            plan.run_challenge().clone(),
            phases,
        )
        .unwrap()
    }

    #[test]
    fn exact_guest_transcript_qualifies() {
        let plan = plan();
        let bindings = bindings(&plan);
        let transcript = transcript(&plan, &bindings);
        let qualified = qualify_guest_transcript(&plan, &transcript, &bindings).unwrap();
        assert_eq!(qualified.gittuf_verification(), plan.expected_replay_receipt());
    }

    #[test]
    fn phase_reordering_fails_closed() {
        let plan = plan();
        let bindings = bindings(&plan);
        let mut transcript = transcript(&plan, &bindings);
        transcript.phases.swap(0, 1);
        assert_eq!(
            transcript.canonical_bytes().unwrap_err(),
            GuestTranscriptError::InvalidPhaseSequence
        );
    }

    #[test]
    fn self_asserted_gittuf_receipt_does_not_qualify() {
        let plan = plan();
        let mut bindings = bindings(&plan);
        bindings.gittuf_verification = digest(0xee);
        let transcript = transcript(&plan, &bindings);
        assert_eq!(
            qualify_guest_transcript(&plan, &transcript, &bindings).unwrap_err(),
            GuestTranscriptError::ExpectedReplayReceiptMismatch
        );
    }

    #[test]
    fn deserialization_revalidates_phase_sequence() {
        let plan = plan();
        let bindings = bindings(&plan);
        let transcript = transcript(&plan, &bindings);
        let mut value = serde_json::to_value(transcript).unwrap();
        value["phases"].as_array_mut().unwrap().reverse();
        assert!(serde_json::from_value::<GuestTranscriptV1>(value).is_err());
    }
}

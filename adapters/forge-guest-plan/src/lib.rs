// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D3B2B: canonical in-sandbox verification plan for Forge M0.
//!
//! The plan is intentionally not an arbitrary argv graph. The v1 phase order,
//! Git object-validation policy, sandbox paths, and required Git environment
//! are protocol constants. The eventual guest runner executes this contract;
//! it does not get to reinterpret it.
//!
//! The plan binds the execution *subject*, not the enclosing ExecutionSpec or
//! HermeticRunPlan digests. The plan itself is mounted as an ExecutionSpec
//! input, so binding those enclosing digests here would create a recursive hash
//! dependency.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_repository::{
    RepositoryVerificationError, RepositoryVerificationRequest,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const GIT_POLICY_DOMAIN_V1: &[u8] = b"mycelix-forge/git-object-validation-policy/v1\0";
const GUEST_PLAN_DOMAIN_V1: &[u8] = b"mycelix-forge/guest-verification-plan/v1\0";

pub const GUEST_REPLAY_REPOSITORY: &str = "/work/repository.git";
pub const GUEST_BUNDLE_PATH: &str = "/inputs/repository.bundle";
pub const GUEST_MANIFEST_PATH: &str = "/inputs/repository-bundle-manifest.json";
pub const GUEST_RUN_CHALLENGE_PATH: &str = "/inputs/run-challenge";
pub const GUEST_PLAN_PATH: &str = "/inputs/guest-verification-plan.json";
pub const GUEST_ISOLATION_POLICY_PATH: &str = "/inputs/linux-isolation-policy.json";
pub const GUEST_NIX_CLOSURE_PATH: &str = "/inputs/nix-closure-manifest.json";
pub const GUEST_SANDBOX_INVOCATION_PATH: &str = "/inputs/sandbox-invocation.json";
pub const GUEST_TRANSCRIPT_PATH: &str = "/work/guest-transcript.json";

pub const GIT_GLOBAL_ARGS: &[&str] = &["--no-replace-objects"];
pub const GIT_FSCK_ARGS: &[&str] = &[
    "fsck",
    "--full",
    "--strict",
    "--no-reflogs",
    "--no-progress",
];

pub const REQUIRED_GIT_ENVIRONMENT: &[(&str, &str)] = &[
    ("GIT_CONFIG_GLOBAL", "/dev/null"),
    ("GIT_CONFIG_NOSYSTEM", "1"),
    ("GIT_NO_LAZY_FETCH", "1"),
    ("GIT_TERMINAL_PROMPT", "0"),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GitObjectValidationPolicyV1;

impl GitObjectValidationPolicyV1 {
    pub const fn strict() -> Self {
        Self
    }

    pub const fn rejects_connectivity_only(self) -> bool {
        true
    }

    pub const fn rejects_alternates_file(self) -> bool {
        true
    }

    pub const fn rejects_object_directory_env(self) -> bool {
        true
    }

    pub const fn rejects_alternate_object_directories_env(self) -> bool {
        true
    }

    pub const fn disables_replace_objects(self) -> bool {
        true
    }

    pub const fn rejects_shallow_state(self) -> bool {
        true
    }

    pub const fn rejects_promisor_state(self) -> bool {
        true
    }

    pub const fn rejects_remotes(self) -> bool {
        true
    }

    pub const fn global_args(self) -> &'static [&'static str] {
        GIT_GLOBAL_ARGS
    }

    pub const fn fsck_args(self) -> &'static [&'static str] {
        GIT_FSCK_ARGS
    }

    pub const fn required_environment(self) -> &'static [(&'static str, &'static str)] {
        REQUIRED_GIT_ENVIRONMENT
    }

    pub fn canonical_bytes(self) -> Result<Vec<u8>, GuestPlanError> {
        let mut out = Vec::new();
        out.extend_from_slice(GIT_POLICY_DOMAIN_V1);
        push_string(&mut out, GUEST_REPLAY_REPOSITORY, "replay repository")?;

        push_count(&mut out, GIT_GLOBAL_ARGS.len(), "Git global args")?;
        for arg in GIT_GLOBAL_ARGS {
            push_string(&mut out, arg, "Git global arg")?;
        }

        push_count(&mut out, GIT_FSCK_ARGS.len(), "fsck args")?;
        for arg in GIT_FSCK_ARGS {
            push_string(&mut out, arg, "fsck arg")?;
        }

        push_count(
            &mut out,
            REQUIRED_GIT_ENVIRONMENT.len(),
            "required Git environment",
        )?;
        for (key, value) in REQUIRED_GIT_ENVIRONMENT {
            push_string(&mut out, key, "environment key")?;
            push_string(&mut out, value, "environment value")?;
        }

        for flag in [
            self.rejects_connectivity_only(),
            self.rejects_alternates_file(),
            self.rejects_object_directory_env(),
            self.rejects_alternate_object_directories_env(),
            self.disables_replace_objects(),
            self.rejects_shallow_state(),
            self.rejects_promisor_state(),
            self.rejects_remotes(),
        ] {
            out.push(u8::from(flag));
        }
        Ok(out)
    }

    pub fn digest(self, algorithm: DigestAlgorithm) -> Result<Digest, GuestPlanError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum GuestPhase {
    IsolationProbe,
    ReplayBundle,
    RejectAmbientObjectSources,
    GitObjectValidation,
    PolicyTrustInventory,
    LocalTrustQualification,
    GittufVerification,
    EmitTranscript,
}

impl GuestPhase {
    const fn code(self) -> u8 {
        match self {
            Self::IsolationProbe => 1,
            Self::ReplayBundle => 2,
            Self::RejectAmbientObjectSources => 3,
            Self::GitObjectValidation => 4,
            Self::PolicyTrustInventory => 5,
            Self::LocalTrustQualification => 6,
            Self::GittufVerification => 7,
            Self::EmitTranscript => 8,
        }
    }

    pub const fn required_sequence() -> [Self; 8] {
        [
            Self::IsolationProbe,
            Self::ReplayBundle,
            Self::RejectAmbientObjectSources,
            Self::GitObjectValidation,
            Self::PolicyTrustInventory,
            Self::LocalTrustQualification,
            Self::GittufVerification,
            Self::EmitTranscript,
        ]
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct GuestVerificationPlanV1 {
    execution_subject: Digest,
    request: RepositoryVerificationRequest,
    bundle_manifest: Digest,
    policy_state: Digest,
    isolation_policy: Digest,
    nix_closure: Digest,
    sandbox_invocation: Digest,
    trust_profile: Digest,
    run_challenge: Digest,
    expected_replay_receipt: Digest,
    git_object_validation_policy: Digest,
}

impl GuestVerificationPlanV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        execution_subject: Digest,
        request: RepositoryVerificationRequest,
        bundle_manifest: Digest,
        policy_state: Digest,
        isolation_policy: Digest,
        nix_closure: Digest,
        sandbox_invocation: Digest,
        trust_profile: Digest,
        run_challenge: Digest,
        expected_replay_receipt: Digest,
    ) -> Result<Self, GuestPlanError> {
        let expected_request = request.digest(DigestAlgorithm::Sha256)?;
        if expected_request.as_bytes().is_empty() {
            return Err(GuestPlanError::InvalidRequestCommitment);
        }
        let git_object_validation_policy =
            GitObjectValidationPolicyV1::strict().digest(DigestAlgorithm::Sha256)?;
        Ok(Self {
            execution_subject,
            request,
            bundle_manifest,
            policy_state,
            isolation_policy,
            nix_closure,
            sandbox_invocation,
            trust_profile,
            run_challenge,
            expected_replay_receipt,
            git_object_validation_policy,
        })
    }

    pub fn execution_subject(&self) -> &Digest {
        &self.execution_subject
    }

    pub fn request(&self) -> &RepositoryVerificationRequest {
        &self.request
    }

    pub fn request_digest(&self) -> Result<Digest, GuestPlanError> {
        Ok(self.request.digest(DigestAlgorithm::Sha256)?)
    }

    pub fn bundle_manifest(&self) -> &Digest {
        &self.bundle_manifest
    }

    pub fn policy_state(&self) -> &Digest {
        &self.policy_state
    }

    pub fn isolation_policy(&self) -> &Digest {
        &self.isolation_policy
    }

    pub fn nix_closure(&self) -> &Digest {
        &self.nix_closure
    }

    pub fn sandbox_invocation(&self) -> &Digest {
        &self.sandbox_invocation
    }

    pub fn trust_profile(&self) -> &Digest {
        &self.trust_profile
    }

    pub fn run_challenge(&self) -> &Digest {
        &self.run_challenge
    }

    pub fn expected_replay_receipt(&self) -> &Digest {
        &self.expected_replay_receipt
    }

    pub fn git_object_validation_policy(&self) -> &Digest {
        &self.git_object_validation_policy
    }

    pub const fn phase_sequence(&self) -> [GuestPhase; 8] {
        GuestPhase::required_sequence()
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, GuestPlanError> {
        let strict_policy = GitObjectValidationPolicyV1::strict().digest(DigestAlgorithm::Sha256)?;
        if self.git_object_validation_policy != strict_policy {
            return Err(GuestPlanError::UnexpectedGitObjectValidationPolicy);
        }

        let mut out = Vec::new();
        out.extend_from_slice(GUEST_PLAN_DOMAIN_V1);
        push_digest(&mut out, &self.execution_subject)?;
        push_digest(&mut out, &self.request_digest()?)?;
        for digest in [
            &self.bundle_manifest,
            &self.policy_state,
            &self.isolation_policy,
            &self.nix_closure,
            &self.sandbox_invocation,
            &self.trust_profile,
            &self.run_challenge,
            &self.expected_replay_receipt,
            &self.git_object_validation_policy,
        ] {
            push_digest(&mut out, digest)?;
        }

        for path in [
            GUEST_REPLAY_REPOSITORY,
            GUEST_BUNDLE_PATH,
            GUEST_MANIFEST_PATH,
            GUEST_RUN_CHALLENGE_PATH,
            GUEST_PLAN_PATH,
            GUEST_ISOLATION_POLICY_PATH,
            GUEST_NIX_CLOSURE_PATH,
            GUEST_SANDBOX_INVOCATION_PATH,
            GUEST_TRANSCRIPT_PATH,
        ] {
            push_string(&mut out, path, "guest path")?;
        }

        let phases = GuestPhase::required_sequence();
        push_count(&mut out, phases.len(), "guest phases")?;
        for phase in phases {
            out.push(phase.code());
        }
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, GuestPlanError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for GuestVerificationPlanV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            execution_subject: Digest,
            request: RepositoryVerificationRequest,
            bundle_manifest: Digest,
            policy_state: Digest,
            isolation_policy: Digest,
            nix_closure: Digest,
            sandbox_invocation: Digest,
            trust_profile: Digest,
            run_challenge: Digest,
            expected_replay_receipt: Digest,
            git_object_validation_policy: Digest,
        }

        let wire = Wire::deserialize(deserializer)?;
        let plan = Self::new(
            wire.execution_subject,
            wire.request,
            wire.bundle_manifest,
            wire.policy_state,
            wire.isolation_policy,
            wire.nix_closure,
            wire.sandbox_invocation,
            wire.trust_profile,
            wire.run_challenge,
            wire.expected_replay_receipt,
        )
        .map_err(D::Error::custom)?;
        if plan.git_object_validation_policy != wire.git_object_validation_policy {
            return Err(D::Error::custom(
                GuestPlanError::UnexpectedGitObjectValidationPolicy,
            ));
        }
        Ok(plan)
    }
}

fn push_count(
    out: &mut Vec<u8>,
    count: usize,
    field: &'static str,
) -> Result<(), GuestPlanError> {
    let count =
        u16::try_from(count).map_err(|_| GuestPlanError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), GuestPlanError> {
    let len =
        u16::try_from(value.len()).map_err(|_| GuestPlanError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), GuestPlanError> {
    push_string(out, digest.algorithm().id(), "digest algorithm")?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| GuestPlanError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum GuestPlanError {
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    #[error("invalid request commitment")]
    InvalidRequestCommitment,
    #[error("guest plan does not use the exact strict Git object-validation policy")]
    UnexpectedGitObjectValidationPolicy,
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
        RepositoryRef, RepositoryTip,
    };

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn git_sha1(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x22)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn request() -> RepositoryVerificationRequest {
        let project = project();
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

    #[test]
    fn strict_git_policy_contains_no_connectivity_only_shortcut() {
        let policy = GitObjectValidationPolicyV1::strict();
        assert!(policy.global_args().contains(&"--no-replace-objects"));
        assert!(policy.fsck_args().contains(&"--full"));
        assert!(policy.fsck_args().contains(&"--strict"));
        assert!(!policy.fsck_args().contains(&"--connectivity-only"));
        assert!(policy.rejects_connectivity_only());
        assert!(policy.disables_replace_objects());
    }

    #[test]
    fn guest_phase_order_is_frozen() {
        assert_eq!(
            GuestPhase::required_sequence(),
            [
                GuestPhase::IsolationProbe,
                GuestPhase::ReplayBundle,
                GuestPhase::RejectAmbientObjectSources,
                GuestPhase::GitObjectValidation,
                GuestPhase::PolicyTrustInventory,
                GuestPhase::LocalTrustQualification,
                GuestPhase::GittufVerification,
                GuestPhase::EmitTranscript,
            ]
        );
    }

    #[test]
    fn request_bytes_are_carried_but_commitment_is_semantic() {
        let plan = plan();
        assert_eq!(
            plan.request_digest().unwrap(),
            plan.request().digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn deserialization_rejects_weakened_git_policy_digest() {
        let mut value = serde_json::to_value(plan()).unwrap();
        value["git_object_validation_policy"] = serde_json::to_value(digest(0xff)).unwrap();
        assert!(serde_json::from_value::<GuestVerificationPlanV1>(value).is_err());
    }

    #[test]
    fn fixed_paths_are_inside_committed_sandbox_roots() {
        assert!(GUEST_REPLAY_REPOSITORY.starts_with("/work/"));
        for path in [
            GUEST_BUNDLE_PATH,
            GUEST_MANIFEST_PATH,
            GUEST_RUN_CHALLENGE_PATH,
            GUEST_PLAN_PATH,
            GUEST_ISOLATION_POLICY_PATH,
            GUEST_NIX_CLOSURE_PATH,
            GUEST_SANDBOX_INVOCATION_PATH,
        ] {
            assert!(path.starts_with("/inputs/"));
        }
        assert!(GUEST_TRANSCRIPT_PATH.starts_with("/work/"));
    }
}

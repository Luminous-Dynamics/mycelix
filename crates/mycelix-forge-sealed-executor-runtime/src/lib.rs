// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-009K: concrete sealed Git transaction execution.
//!
//! The runtime accepts only a positive FORGE-009J invocation. Provider-fixed
//! executable/repository configuration is held by the runtime instance, while
//! higher-level authority and durable execution journaling are injected through
//! explicit traits.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, DigestAlgorithm, ProtocolVersion};
use mycelix_forge_sealed_executor_interface::{
    SealedExecutorInterfaceError, SealedExecutorInvocationV1,
};
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};
use thiserror::Error;

const EXECUTION_OBSERVATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/sealed-executor-runtime-observation/v1\0";
const RUNTIME_CONFIG_DOMAIN_V1: &[u8] =
    b"mycelix-forge/sealed-executor-runtime-config/v1\0";
const MAX_CONFIG_PATH_LEN: usize = 4096;

/// Fixed runtime configuration supplied by the provider deployment.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SealedExecutorRuntimeConfigV1 {
    executor_identity: Digest,
    repository_identity: Digest,
    git_executable: PathBuf,
    repository_path: PathBuf,
    timeout: Duration,
}

impl SealedExecutorRuntimeConfigV1 {
    /// Construct provider-fixed runtime configuration.
    pub fn new(
        executor_identity: Digest,
        repository_identity: Digest,
        git_executable: PathBuf,
        repository_path: PathBuf,
        timeout: Duration,
    ) -> Result<Self, SealedExecutorRuntimeError> {
        if !git_executable.is_absolute() {
            return Err(SealedExecutorRuntimeError::PathMustBeAbsolute("git executable"));
        }
        if !repository_path.is_absolute() {
            return Err(SealedExecutorRuntimeError::PathMustBeAbsolute("repository"));
        }
        if timeout.is_zero() {
            return Err(SealedExecutorRuntimeError::InvalidTimeout);
        }
        Ok(Self {
            executor_identity,
            repository_identity,
            git_executable,
            repository_path,
            timeout,
        })
    }

    /// Exact provider-fixed executor identity.
    pub fn executor_identity(&self) -> &Digest {
        &self.executor_identity
    }

    /// Exact provider-fixed repository identity.
    pub fn repository_identity(&self) -> &Digest {
        &self.repository_identity
    }

    /// Provider-fixed Git executable path.
    pub fn git_executable(&self) -> &std::path::Path {
        &self.git_executable
    }

    /// Provider-fixed repository path.
    pub fn repository_path(&self) -> &std::path::Path {
        &self.repository_path
    }

    /// Provider-fixed process timeout.
    pub fn timeout(&self) -> Duration {
        self.timeout
    }

    /// Canonical commitment to the exact provider-fixed runtime configuration.
    ///
    /// The commitment binds the executor identity, repository identity,
    /// executable path, repository path, and timeout. A merge-execution
    /// authorization must carry this exact commitment before execution can begin.
    pub fn commitment(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, SealedExecutorRuntimeError> {
        let mut out = Vec::new();
        out.extend_from_slice(RUNTIME_CONFIG_DOMAIN_V1);
        out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
        push_digest(&mut out, &self.executor_identity)?;
        push_digest(&mut out, &self.repository_identity)?;
        push_path(&mut out, &self.git_executable, "git executable")?;
        push_path(&mut out, &self.repository_path, "repository")?;
        out.extend_from_slice(&self.timeout.as_secs().to_be_bytes());
        out.extend_from_slice(&self.timeout.subsec_nanos().to_be_bytes());
        Ok(Digest::of_bytes(algorithm, &out))
    }
}

/// One-shot positive merge-execution authorization.
///
/// This type is intentionally not deserializable, cloneable, or constructible
/// through the public API. A future authority qualifier in this crate must
/// construct it only after the complete merge-execution theorem succeeds.
#[must_use = "merge-execution authorization must be consumed exactly once"]
pub struct MergeExecutionAuthorizationV1 {
    invocation_commitment: Digest,
    runtime_config_commitment: Digest,
    authority_evidence: Digest,
    evidence_commitment: Digest,
}

impl MergeExecutionAuthorizationV1 {
    /// Exact invocation commitment carried by the authorization.
    pub fn invocation_commitment(&self) -> &Digest {
        &self.invocation_commitment
    }

    /// Exact provider-fixed runtime configuration commitment.
    pub fn runtime_config_commitment(&self) -> &Digest {
        &self.runtime_config_commitment
    }

    /// Higher-level authority evidence commitment.
    pub fn authority_evidence(&self) -> &Digest {
        &self.authority_evidence
    }

    /// Aggregate merge-execution authorization commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Opaque proof that durable preparation has already been committed.
///
/// This capability is intentionally not deserializable, cloneable, or publicly
/// constructible. A trusted coordinator inside this crate may issue it only
/// after durable preparation has committed the exact invocation, runtime
/// configuration, and merge authority.
#[must_use = "prepared execution must be consumed exactly once"]
pub struct PreparedExecutionPermitV1 {
    invocation_commitment: Digest,
    runtime_config_commitment: Digest,
    authority_evidence: Digest,
    preparation_evidence: Digest,
}

impl PreparedExecutionPermitV1 {
    /// Issue an opaque preparation permit after durable preparation succeeds.
    fn issue(
        invocation_commitment: Digest,
        runtime_config_commitment: Digest,
        authority_evidence: Digest,
        preparation_evidence: Digest,
    ) -> Self {
        Self {
            invocation_commitment,
            runtime_config_commitment,
            authority_evidence,
            preparation_evidence,
        }
    }

    /// Exact invocation commitment covered by durable preparation.
    pub fn invocation_commitment(&self) -> &Digest {
        &self.invocation_commitment
    }

    /// Exact provider-fixed runtime configuration commitment covered by durable preparation.
    pub fn runtime_config_commitment(&self) -> &Digest {
        &self.runtime_config_commitment
    }

    /// Exact merge-authority evidence covered by durable preparation.
    pub fn authority_evidence(&self) -> &Digest {
        &self.authority_evidence
    }

    /// Exact durable preparation evidence commitment.
    pub fn preparation_evidence(&self) -> &Digest {
        &self.preparation_evidence
    }
}

/// Durable execution journal.
///
/// `prepare` is the only path by which the runtime can mint the opaque
/// [`PreparedExecutionPermitV1`]. The journal implementation is responsible
/// for durably committing the prepared record and returning its exact evidence
/// commitment. `complete` appends terminal evidence after the external process
/// attempt.
pub trait SealedExecutorExecutionJournalV1 {
    /// Durably commit preparation and return its exact evidence commitment.
    fn prepare(
        &self,
        invocation: &SealedExecutorInvocationV1,
        authority_evidence: &Digest,
    ) -> Result<Digest, SealedExecutorJournalErrorV1>;

    /// Durably append terminal process outcome bound to the exact prepared execution.
    fn complete(
        &self,
        invocation: &SealedExecutorInvocationV1,
        authority_evidence: &Digest,
        preparation_evidence: &Digest,
        outcome: &SealedExecutorOutcomeV1,
    ) -> Result<(), SealedExecutorJournalErrorV1>;
}

/// Process outcome recorded after an admitted execution attempt.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SealedExecutorOutcomeV1 {
    /// Process completed normally.
    Completed {
        /// Exit code, when supplied by the operating system.
        exit_code: Option<i32>,
    },
    /// The process could not be spawned, so no child process was created.
    SpawnFailed,
    /// The runtime observed an uncertain process state and must not auto-retry.
    InDoubt {
        /// Reason the outcome cannot be treated as terminal success/failure.
        reason: SealedExecutorInDoubtReasonV1,
    },
}

impl SealedExecutorOutcomeV1 {
    /// Whether the process reported exit code zero.
    pub const fn succeeded(&self) -> bool {
        matches!(self, Self::Completed { exit_code: Some(0) })
    }

    /// Whether reconciliation is required before another execution attempt.
    pub const fn is_in_doubt(&self) -> bool {
        matches!(self, Self::InDoubt { .. })
    }
}

/// Reason an admitted execution attempt must enter reconciliation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SealedExecutorInDoubtReasonV1 {
    /// Exact stdin could not be fully delivered after process creation.
    InputWriteFailed,
    /// The fixed timeout elapsed before a terminal process result was observed.
    TimedOut,
    /// The operating system did not provide a reliable wait result.
    WaitFailed,
}

impl SealedExecutorInDoubtReasonV1 {
    const fn code(&self) -> u8 {
        match self {
            Self::InputWriteFailed => 0,
            Self::TimedOut => 1,
            Self::WaitFailed => 2,
        }
    }
}

impl SealedExecutorOutcomeV1 {
    /// Canonical outcome commitment.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Digest {
        let mut out = Vec::new();
        out.extend_from_slice(EXECUTION_OBSERVATION_DOMAIN_V1);
        match self {
            Self::Completed { exit_code } => {
                out.push(0);
                match exit_code {
                    Some(code) => {
                        out.push(1);
                        out.extend_from_slice(&code.to_be_bytes());
                    }
                    None => out.push(0),
                }
            }
            Self::SpawnFailed => out.push(1),
            Self::InDoubt { reason } => {
                out.push(2);
                out.push(reason.code());
            }
        }
        Digest::of_bytes(algorithm, &out)
    }
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), SealedExecutorRuntimeError> {
    let algorithm = digest.algorithm().id().as_bytes();
    let algorithm_len = u16::try_from(algorithm.len())
        .map_err(|_| SealedExecutorRuntimeError::CanonicalFieldTooLarge("digest algorithm"))?;
    let bytes = digest.as_bytes();
    let digest_len = u32::try_from(bytes.len())
        .map_err(|_| SealedExecutorRuntimeError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&algorithm_len.to_be_bytes());
    out.extend_from_slice(algorithm);
    out.extend_from_slice(&digest_len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

fn push_path(
    out: &mut Vec<u8>,
    path: &std::path::Path,
    label: &'static str,
) -> Result<(), SealedExecutorRuntimeError> {
    let value = path
        .to_str()
        .ok_or(SealedExecutorRuntimeError::NonUtf8Path(label))?;
    let bytes = value.as_bytes();
    if bytes.len() > MAX_CONFIG_PATH_LEN {
        return Err(SealedExecutorRuntimeError::ConfigPathTooLong(label));
    }
    let len = u32::try_from(bytes.len())
        .map_err(|_| SealedExecutorRuntimeError::CanonicalFieldTooLarge("path"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Concrete provider-fixed sealed Git executor.
#[derive(Debug)]
pub struct SealedGitTransactionExecutorV1 {
    config: SealedExecutorRuntimeConfigV1,
}

impl SealedGitTransactionExecutorV1 {
    /// Construct a concrete executor from provider-fixed deployment settings.
    pub fn new(config: SealedExecutorRuntimeConfigV1) -> Self {
        Self { config }
    }

    /// Provider-fixed runtime configuration.
    pub fn config(&self) -> &SealedExecutorRuntimeConfigV1 {
        &self.config
    }

    /// Durably prepare exactly one sealed invocation after authority succeeds.
    ///
    /// No process is spawned by this method. The returned opaque permit can
    /// only be created after the journal reports durable preparation success.
    pub fn prepare<J>(
        &self,
        invocation: &SealedExecutorInvocationV1,
        authorization: &MergeExecutionAuthorizationV1,
        journal: &J,
    ) -> Result<PreparedExecutionPermitV1, SealedExecutorRuntimeError>
    where
        J: SealedExecutorExecutionJournalV1,
    {
        let (invocation_commitment, runtime_config_commitment) =
            self.validate_authorization(invocation, authorization)?;
        let preparation_evidence = journal
            .prepare(invocation, &authorization.authority_evidence)
            .map_err(SealedExecutorRuntimeError::PreparePersistenceFailed)?;
        Ok(PreparedExecutionPermitV1::issue(
            invocation_commitment,
            runtime_config_commitment,
            authorization.authority_evidence.clone(),
            preparation_evidence,
        ))
    }

    /// Execute exactly one sealed invocation after authority and durable
    /// preparation have succeeded.
    ///
    /// No process is spawned before both gates succeed.
    pub fn execute<J>(
        &self,
        invocation: &SealedExecutorInvocationV1,
        authorization: MergeExecutionAuthorizationV1,
        prepared: PreparedExecutionPermitV1,
        journal: &J,
    ) -> Result<SealedExecutorOutcomeV1, SealedExecutorRuntimeError>
    where
        J: SealedExecutorExecutionJournalV1,
    {
        let (expected_invocation, expected_config) =
            self.validate_authorization(invocation, &authorization)?;
        if prepared.invocation_commitment != expected_invocation {
            return Err(SealedExecutorRuntimeError::PreparedExecutionInvocationMismatch);
        }
        if prepared.runtime_config_commitment != expected_config {
            return Err(SealedExecutorRuntimeError::PreparedExecutionRuntimeConfigMismatch);
        }
        if prepared.authority_evidence != authorization.authority_evidence {
            return Err(SealedExecutorRuntimeError::PreparedExecutionAuthorityMismatch);
        }

        let authority_evidence = authorization.authority_evidence.clone();
        let preparation_evidence = prepared.preparation_evidence.clone();
        let outcome = self.spawn_and_wait(invocation.command_args(), invocation.stdin_bytes());
        match journal.complete(
            invocation,
            &authority_evidence,
            &preparation_evidence,
            &outcome,
        ) {
            Ok(()) => Ok(outcome),
            Err(error) => Err(SealedExecutorRuntimeError::CompletionPersistenceFailed {
                outcome,
                source: error,
            }),
        }
    }

    fn validate_authorization(
        &self,
        invocation: &SealedExecutorInvocationV1,
        authorization: &MergeExecutionAuthorizationV1,
    ) -> Result<(Digest, Digest), SealedExecutorRuntimeError> {
        self.validate_invocation(invocation)?;
        let algorithm = authorization.invocation_commitment.algorithm();
        let expected_invocation = invocation
            .digest(algorithm)
            .map_err(SealedExecutorRuntimeError::InvocationDigestFailed)?;
        if authorization.invocation_commitment != expected_invocation {
            return Err(SealedExecutorRuntimeError::AuthorizationInvocationMismatch);
        }

        let config_algorithm = authorization.runtime_config_commitment.algorithm();
        let expected_config = self.config.commitment(config_algorithm)?;
        if authorization.runtime_config_commitment != expected_config {
            return Err(SealedExecutorRuntimeError::AuthorizationRuntimeConfigMismatch);
        }
        Ok((expected_invocation, expected_config))
    }

    fn validate_invocation(
        &self,
        invocation: &SealedExecutorInvocationV1,
    ) -> Result<(), SealedExecutorRuntimeError> {
        if invocation.executor_identity() != self.config.executor_identity() {
            return Err(SealedExecutorRuntimeError::ExecutorIdentityMismatch);
        }
        if invocation.repository_identity() != self.config.repository_identity() {
            return Err(SealedExecutorRuntimeError::RepositoryIdentityMismatch);
        }
        Ok(())
    }

    fn spawn_and_wait(
        &self,
        command_args: &[String],
        stdin_bytes: &[u8],
    ) -> SealedExecutorOutcomeV1 {
        let mut command = Command::new(&self.config.git_executable);
        command
            .current_dir(&self.config.repository_path)
            .args(command_args)
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .env_clear()
            .env("GIT_CONFIG_NOSYSTEM", "1")
            .env("GIT_TERMINAL_PROMPT", "0");

        let mut child = match command.spawn() {
            Ok(child) => child,
            Err(_) => return SealedExecutorOutcomeV1::SpawnFailed,
        };

        let Some(mut stdin) = child.stdin.take() else {
            let _ = child.kill();
            let _ = child.wait();
            return SealedExecutorOutcomeV1::InDoubt {
                reason: SealedExecutorInDoubtReasonV1::InputWriteFailed,
            };
        };
        if stdin.write_all(stdin_bytes).is_err() {
            if child.kill().is_err() {
                return SealedExecutorOutcomeV1::InDoubt {
                    reason: SealedExecutorInDoubtReasonV1::InputWriteFailed,
                };
            }
            let _ = child.wait();
            return SealedExecutorOutcomeV1::InDoubt {
                reason: SealedExecutorInDoubtReasonV1::InputWriteFailed,
            };
        }

        let deadline = Instant::now() + self.config.timeout;
        loop {
            match child.try_wait() {
                Ok(Some(status)) => {
                    return SealedExecutorOutcomeV1::Completed {
                        exit_code: status.code(),
                    };
                }
                Ok(None) if Instant::now() >= deadline => {
                    if child.kill().is_err() {
                        return SealedExecutorOutcomeV1::InDoubt {
                            reason: SealedExecutorInDoubtReasonV1::WaitFailed,
                        };
                    }
                    return match child.wait() {
                        Ok(_) => SealedExecutorOutcomeV1::InDoubt {
                            reason: SealedExecutorInDoubtReasonV1::TimedOut,
                        },
                        Err(_) => SealedExecutorOutcomeV1::InDoubt {
                            reason: SealedExecutorInDoubtReasonV1::WaitFailed,
                        },
                    };
                }
                Ok(None) => thread::sleep(Duration::from_millis(10)),
                Err(_) => {
                    let _ = child.kill();
                    return SealedExecutorOutcomeV1::InDoubt {
                        reason: SealedExecutorInDoubtReasonV1::WaitFailed,
                    };
                }
            }
        }
    }
}

/// Durable preparation/completion failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum SealedExecutorJournalErrorV1 {
    /// Preparation could not be durably committed.
    #[error("sealed executor preparation persistence failed")]
    PrepareFailed,
    /// Terminal evidence could not be durably committed.
    #[error("sealed executor completion persistence failed")]
    CompleteFailed,
}

/// FORGE-009K runtime failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum SealedExecutorRuntimeError {
    /// Invocation executor identity differs from provider configuration.
    #[error("sealed executor invocation executor identity mismatch")]
    ExecutorIdentityMismatch,
    /// Invocation repository identity differs from provider configuration.
    #[error("sealed executor invocation repository identity mismatch")]
    RepositoryIdentityMismatch,
    /// Authorization is bound to a different invocation.
    #[error("sealed executor authorization does not bind the exact invocation")]
    AuthorizationInvocationMismatch,
    /// Invocation digest could not be canonicalized.
    #[error("sealed executor invocation digest failed: {0}")]
    InvocationDigestFailed(SealedExecutorInterfaceError),
    /// Authorization is bound to a different provider-fixed runtime configuration.
    #[error("sealed executor authorization does not bind the exact runtime configuration")]
    AuthorizationRuntimeConfigMismatch,
    /// Durable preparation is bound to a different invocation.
    #[error("sealed executor preparation permit does not bind the exact invocation")]
    PreparedExecutionInvocationMismatch,
    /// Durable preparation is bound to a different provider-fixed runtime configuration.
    #[error("sealed executor preparation permit does not bind the exact runtime configuration")]
    PreparedExecutionRuntimeConfigMismatch,
    /// Durable preparation is bound to different merge authority.
    #[error("sealed executor preparation permit does not bind the exact merge authority")]
    PreparedExecutionAuthorityMismatch,
    /// Process completed or failed, but terminal evidence could not be persisted.
    ///
    /// This is an in-doubt outcome and must not be automatically retried.
    #[error("sealed executor terminal evidence is in doubt: {source}")]
    CompletionPersistenceFailed {
        /// Observed process outcome.
        outcome: SealedExecutorOutcomeV1,
        /// Persistence error.
        source: SealedExecutorJournalErrorV1,
    },
    /// A runtime path must be absolute.
    #[error("{0} path must be absolute")]
    PathMustBeAbsolute(&'static str),
    /// Zero timeout would disable the timeout boundary.
    #[error("sealed executor timeout must be non-zero")]
    InvalidTimeout,
    /// Canonical commitment field exceeded its bounded encoding size.
    #[error("sealed executor canonical field too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
    /// A runtime configuration path is not valid UTF-8 for the canonical v1 commitment.
    #[error("sealed executor {0} path must be valid UTF-8")]
    NonUtf8Path(&'static str),
    /// A runtime configuration path exceeds the v1 canonical commitment bound.
    #[error("sealed executor {0} path exceeds the v1 canonical bound")]
    ConfigPathTooLong(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    struct RecordingJournal;

    impl SealedExecutorExecutionJournalV1 for RecordingJournal {
        fn prepare(
            &self,
            _invocation: &SealedExecutorInvocationV1,
            _authority_evidence: &Digest,
        ) -> Result<Digest, SealedExecutorJournalErrorV1> {
            Ok(digest(4))
        }

        fn complete(
            &self,
            _invocation: &SealedExecutorInvocationV1,
            _authority_evidence: &Digest,
            _preparation_evidence: &Digest,
            _outcome: &SealedExecutorOutcomeV1,
        ) -> Result<(), SealedExecutorJournalErrorV1> {
            Ok(())
        }
    }

    #[test]
    fn runtime_requires_absolute_provider_paths() {
        assert_eq!(
            SealedExecutorRuntimeConfigV1::new(
                digest(1),
                digest(2),
                PathBuf::from("git"),
                PathBuf::from("/repo"),
                Duration::from_secs(1),
            )
            .unwrap_err(),
            SealedExecutorRuntimeError::PathMustBeAbsolute("git executable")
        );
        assert_eq!(
            SealedExecutorRuntimeConfigV1::new(
                digest(1),
                digest(2),
                PathBuf::from("/usr/bin/git"),
                PathBuf::from("repo"),
                Duration::from_secs(1),
            )
            .unwrap_err(),
            SealedExecutorRuntimeError::PathMustBeAbsolute("repository")
        );
    }

    #[test]
    fn journal_is_an_explicit_execution_gate() {
        fn assert_journal<T: SealedExecutorExecutionJournalV1>() {}
        assert_journal::<RecordingJournal>();
    }

    #[test]
    fn prepare_mints_permit_only_after_journal_commit() {
        let config = SealedExecutorRuntimeConfigV1::new(
            digest(1),
            digest(2),
            PathBuf::from("/usr/bin/true"),
            std::env::temp_dir(),
            Duration::from_secs(1),
        )
        .unwrap();
        let executor = SealedGitTransactionExecutorV1::new(config);
        let invocation = SealedExecutorInvocationV1 {
            executor_identity: digest(1),
            repository_identity: digest(2),
            plan_evidence: digest(3),
            interface_commitment: digest(4),
            command_args: vec!["--fixed".to_owned()],
            stdin_bytes: vec![0],
        };
        let invocation_commitment = invocation.digest(DigestAlgorithm::Sha256).unwrap();
        let runtime_config_commitment = executor
            .config()
            .commitment(DigestAlgorithm::Sha256)
            .unwrap();
        let authorization = MergeExecutionAuthorizationV1 {
            invocation_commitment,
            runtime_config_commitment,
            authority_evidence: digest(5),
            evidence_commitment: digest(6),
        };

        let permit = executor
            .prepare(&invocation, &authorization, &RecordingJournal)
            .unwrap();

        assert_eq!(
            permit.invocation_commitment(),
            authorization.invocation_commitment()
        );
        assert_eq!(
            permit.runtime_config_commitment(),
            authorization.runtime_config_commitment()
        );
        assert_eq!(
            permit.authority_evidence(),
            authorization.authority_evidence()
        );
        assert_eq!(permit.preparation_evidence(), &digest(4));
    }

    #[test]
    fn prepared_execution_permit_binds_exact_authority_and_configuration() {
        let invocation = digest(1);
        let runtime_config = digest(2);
        let authority = digest(3);
        let preparation = digest(4);
        let permit = PreparedExecutionPermitV1::issue(
            invocation.clone(),
            runtime_config.clone(),
            authority.clone(),
            preparation.clone(),
        );
        assert_eq!(permit.invocation_commitment(), &invocation);
        assert_eq!(permit.runtime_config_commitment(), &runtime_config);
        assert_eq!(permit.authority_evidence(), &authority);
        assert_eq!(permit.preparation_evidence(), &preparation);
    }

    #[test]
    fn runtime_config_commitment_binds_all_provider_fixed_execution_inputs() {
        let base = SealedExecutorRuntimeConfigV1::new(
            digest(1),
            digest(2),
            PathBuf::from("/usr/bin/git"),
            PathBuf::from("/repo"),
            Duration::from_secs(10),
        )
        .unwrap();
        let changed_executable = SealedExecutorRuntimeConfigV1::new(
            digest(1),
            digest(2),
            PathBuf::from("/usr/bin/git-alt"),
            PathBuf::from("/repo"),
            Duration::from_secs(10),
        )
        .unwrap();
        let changed_repository = SealedExecutorRuntimeConfigV1::new(
            digest(1),
            digest(2),
            PathBuf::from("/usr/bin/git"),
            PathBuf::from("/other-repo"),
            Duration::from_secs(10),
        )
        .unwrap();
        let changed_timeout = SealedExecutorRuntimeConfigV1::new(
            digest(1),
            digest(2),
            PathBuf::from("/usr/bin/git"),
            PathBuf::from("/repo"),
            Duration::from_secs(11),
        )
        .unwrap();

        let commitment = base.commitment(DigestAlgorithm::Sha256).unwrap();
        assert_ne!(
            commitment,
            changed_executable.commitment(DigestAlgorithm::Sha256).unwrap()
        );
        assert_ne!(
            commitment,
            changed_repository.commitment(DigestAlgorithm::Sha256).unwrap()
        );
        assert_ne!(
            commitment,
            changed_timeout.commitment(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn direct_process_boundary_executes_without_shell() {
        let config = SealedExecutorRuntimeConfigV1::new(
            digest(1),
            digest(2),
            PathBuf::from("/usr/bin/true"),
            std::env::temp_dir(),
            Duration::from_secs(1),
        )
        .unwrap();
        let executor = SealedGitTransactionExecutorV1::new(config);
        let outcome = executor.spawn_and_wait(
            &["--forge-fixed-argv".to_owned()],
            b"start\0",
        );
        assert_eq!(
            outcome,
            SealedExecutorOutcomeV1::Completed { exit_code: Some(0) }
        );
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn fixed_timeout_produces_in_doubt_outcome() {
        let config = SealedExecutorRuntimeConfigV1::new(
            digest(1),
            digest(2),
            PathBuf::from("/usr/bin/sleep"),
            std::env::temp_dir(),
            Duration::from_millis(20),
        )
        .unwrap();
        let executor = SealedGitTransactionExecutorV1::new(config);
        let outcome = executor.spawn_and_wait(&["1".to_owned()], b"");
        assert_eq!(
            outcome,
            SealedExecutorOutcomeV1::InDoubt {
                reason: SealedExecutorInDoubtReasonV1::TimedOut,
            }
        );
    }

    #[test]
    fn outcome_digest_is_domain_separated() {
        let a = SealedExecutorOutcomeV1::Completed { exit_code: Some(0) }
            .digest(DigestAlgorithm::Sha256);
        let b = SealedExecutorOutcomeV1::InDoubt {
            reason: SealedExecutorInDoubtReasonV1::TimedOut,
        }
        .digest(DigestAlgorithm::Sha256);
        assert_ne!(a, b);
    }

    #[test]
    fn successful_outcome_requires_zero_exit() {
        assert!(SealedExecutorOutcomeV1::Completed { exit_code: Some(0) }.succeeded());
        assert!(!SealedExecutorOutcomeV1::Completed { exit_code: Some(1) }.succeeded());
        assert!(!SealedExecutorOutcomeV1::InDoubt {
            reason: SealedExecutorInDoubtReasonV1::TimedOut,
        }
        .succeeded());
        assert!(
            SealedExecutorOutcomeV1::InDoubt {
                reason: SealedExecutorInDoubtReasonV1::TimedOut,
            }
            .is_in_doubt()
        );
    }
}

#![deny(unsafe_code)]

//! Pure provider execution-instance identity for EVIDENCE-CI-004C.
//!
//! This crate deliberately owns **identity only**. It does not bind arbitrary observations
//! or receipts to an attempt; provider-specific composition must prove where an attempt came
//! from before attaching evidence to this identity.

use std::fmt;

pub const MAX_PROVIDER_PROFILE_BYTES_V1: usize = 256;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ProviderProfileId(String);

impl ProviderProfileId {
    pub fn new(value: impl Into<String>) -> Result<Self, ExecutionInstanceError> {
        let value = value.into();
        if value.is_empty()
            || value.len() > MAX_PROVIDER_PROFILE_BYTES_V1
            || value.chars().any(char::is_control)
        {
            return Err(ExecutionInstanceError::InvalidProviderProfile);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProviderAttemptObservationV1 {
    Known(u32),
    Unknown,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExecutionInstanceProposalV1 {
    pub provider_profile: ProviderProfileId,
    pub repository_id: u64,
    pub provider_run_id: u64,
    pub provider_attempt: ProviderAttemptObservationV1,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ExecutionInstanceV1 {
    provider_profile: ProviderProfileId,
    repository_id: u64,
    provider_run_id: u64,
    provider_attempt: u32,
}

impl ExecutionInstanceV1 {
    pub fn provider_profile(&self) -> &ProviderProfileId {
        &self.provider_profile
    }

    pub const fn repository_id(&self) -> u64 {
        self.repository_id
    }

    pub const fn provider_run_id(&self) -> u64 {
        self.provider_run_id
    }

    pub const fn provider_attempt(&self) -> u32 {
        self.provider_attempt
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExecutionInstanceError {
    InvalidProviderProfile,
    UnknownAttempt,
    InvalidAttempt,
    InvalidRepositoryId,
    InvalidProviderRunId,
    ExecutionInstanceMismatch,
}

impl fmt::Display for ExecutionInstanceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let text = match self {
            Self::InvalidProviderProfile => "provider profile is empty, too long, or contains control characters",
            Self::UnknownAttempt => "provider execution attempt is unknown",
            Self::InvalidAttempt => "provider execution attempt must be non-zero",
            Self::InvalidRepositoryId => "repository id must be non-zero",
            Self::InvalidProviderRunId => "provider run id must be non-zero",
            Self::ExecutionInstanceMismatch => "execution instances are not identical",
        };
        f.write_str(text)
    }
}

impl std::error::Error for ExecutionInstanceError {}

pub fn qualify_execution_instance_v1(
    proposal: ExecutionInstanceProposalV1,
) -> Result<ExecutionInstanceV1, ExecutionInstanceError> {
    if proposal.repository_id == 0 {
        return Err(ExecutionInstanceError::InvalidRepositoryId);
    }
    if proposal.provider_run_id == 0 {
        return Err(ExecutionInstanceError::InvalidProviderRunId);
    }
    let provider_attempt = match proposal.provider_attempt {
        ProviderAttemptObservationV1::Known(0) => {
            return Err(ExecutionInstanceError::InvalidAttempt)
        }
        ProviderAttemptObservationV1::Known(value) => value,
        ProviderAttemptObservationV1::Unknown => {
            return Err(ExecutionInstanceError::UnknownAttempt)
        }
    };

    Ok(ExecutionInstanceV1 {
        provider_profile: proposal.provider_profile,
        repository_id: proposal.repository_id,
        provider_run_id: proposal.provider_run_id,
        provider_attempt,
    })
}

pub fn require_same_execution_instance_v1(
    left: &ExecutionInstanceV1,
    right: &ExecutionInstanceV1,
) -> Result<(), ExecutionInstanceError> {
    if left == right {
        Ok(())
    } else {
        Err(ExecutionInstanceError::ExecutionInstanceMismatch)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn provider(value: &str) -> ProviderProfileId {
        ProviderProfileId::new(value).expect("static provider profile")
    }

    fn instance(run: u64, attempt: u32) -> ExecutionInstanceV1 {
        qualify_execution_instance_v1(ExecutionInstanceProposalV1 {
            provider_profile: provider("github-actions-v1"),
            repository_id: 1176351975,
            provider_run_id: run,
            provider_attempt: ProviderAttemptObservationV1::Known(attempt),
        })
        .expect("valid execution instance")
    }

    #[test]
    fn same_identity_is_equal() {
        assert_eq!(instance(42, 1), instance(42, 1));
        assert!(require_same_execution_instance_v1(&instance(42, 1), &instance(42, 1)).is_ok());
    }

    #[test]
    fn same_run_different_attempt_is_different_execution() {
        assert_ne!(instance(42, 1), instance(42, 2));
        assert_eq!(
            require_same_execution_instance_v1(&instance(42, 1), &instance(42, 2)),
            Err(ExecutionInstanceError::ExecutionInstanceMismatch)
        );
    }

    #[test]
    fn different_run_is_different_execution() {
        assert_ne!(instance(42, 1), instance(43, 1));
    }

    #[test]
    fn different_repository_is_different_execution() {
        let first = instance(42, 1);
        let second = qualify_execution_instance_v1(ExecutionInstanceProposalV1 {
            provider_profile: provider("github-actions-v1"),
            repository_id: 1176351976,
            provider_run_id: 42,
            provider_attempt: ProviderAttemptObservationV1::Known(1),
        })
        .unwrap();
        assert_ne!(first, second);
    }

    #[test]
    fn provider_profile_is_part_of_identity() {
        let first = instance(42, 1);
        let second = qualify_execution_instance_v1(ExecutionInstanceProposalV1 {
            provider_profile: provider("github-enterprise-actions-v1"),
            repository_id: 1176351975,
            provider_run_id: 42,
            provider_attempt: ProviderAttemptObservationV1::Known(1),
        })
        .unwrap();
        assert_ne!(first, second);
    }

    #[test]
    fn unknown_and_zero_attempts_fail_closed() {
        for attempt in [
            ProviderAttemptObservationV1::Unknown,
            ProviderAttemptObservationV1::Known(0),
        ] {
            assert!(qualify_execution_instance_v1(ExecutionInstanceProposalV1 {
                provider_profile: provider("github-actions-v1"),
                repository_id: 1176351975,
                provider_run_id: 42,
                provider_attempt: attempt,
            })
            .is_err());
        }
    }

    #[test]
    fn zero_repository_and_run_ids_fail_closed() {
        for (repository_id, run_id, expected) in [
            (0, 42, ExecutionInstanceError::InvalidRepositoryId),
            (1176351975, 0, ExecutionInstanceError::InvalidProviderRunId),
        ] {
            assert_eq!(
                qualify_execution_instance_v1(ExecutionInstanceProposalV1 {
                    provider_profile: provider("github-actions-v1"),
                    repository_id,
                    provider_run_id: run_id,
                    provider_attempt: ProviderAttemptObservationV1::Known(1),
                }),
                Err(expected)
            );
        }
    }

    #[test]
    fn invalid_provider_profile_fails_closed() {
        assert_eq!(
            ProviderProfileId::new(""),
            Err(ExecutionInstanceError::InvalidProviderProfile)
        );
    }
}

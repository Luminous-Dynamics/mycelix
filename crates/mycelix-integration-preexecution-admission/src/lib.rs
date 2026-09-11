//! Same-call preexecution admission for the Mycelix integration plane.
//!
//! This layer composes exact current institutional/capability coverage with a
//! freshly re-run INT-04 claim/provider-profile binding. It deliberately stops
//! before provider payload materialization, `DispatchStarted`, coordinator-update
//! exclusion, or any external effect.

use mycelix_institutional_core::Digest32;
use mycelix_integration_core::{
    CanonicalEncodeV1, ContentCommitment, DigestAlgorithm, IntegrationCommand,
};
use mycelix_integration_current_capability_coverage::MatchedCurrentIntegrationCapabilityCoverage;
use mycelix_integration_execution_binding::{
    qualify_execution_binding, ProviderProfileTrustRoot, QualificationError,
    QualifiedExecutionBinding, QualifiedProviderExecutionProfile,
};
use mycelix_integration_runtime::ExecutionClaim;
use thiserror::Error;

pub const ADMISSION_PROFILE: &str =
    "mycelix-integration-preexecution-admission-v1-blake3-framed";
const DOMAIN_ADMISSION: &[u8] = b"mycelix/integration/preexecution-admission/v1";

/// Non-deserializable composition of current authority/capability coverage and a
/// same-invocation exact attempt/provider-policy binding.
///
/// The provider trust root is checked during construction, but a future
/// materialization/effect boundary must recheck the then-current trust root. This
/// object therefore cannot be treated as a reusable execution capability.
#[derive(Clone, Debug)]
pub struct QualifiedIntegrationPreexecutionAdmission {
    coverage: MatchedCurrentIntegrationCapabilityCoverage,
    execution_binding: QualifiedExecutionBinding,
    admission_digest: Digest32,
    qualified_at_ms: i64,
    authority_valid_until_ms: u64,
    claim_lease_until_ms: i64,
    provider_profile_valid_through_ms: i64,
}

impl QualifiedIntegrationPreexecutionAdmission {
    pub fn coverage(&self) -> &MatchedCurrentIntegrationCapabilityCoverage {
        &self.coverage
    }

    pub fn execution_binding(&self) -> &QualifiedExecutionBinding {
        &self.execution_binding
    }

    pub fn admission_digest(&self) -> Digest32 {
        self.admission_digest
    }

    pub fn admission_profile(&self) -> &'static str {
        ADMISSION_PROFILE
    }

    pub fn qualified_at_ms(&self) -> i64 {
        self.qualified_at_ms
    }

    /// Exclusive institutional/currentness horizon.
    pub fn authority_valid_until_ms(&self) -> u64 {
        self.authority_valid_until_ms
    }

    /// Exclusive INT-03 attempt lease horizon.
    pub fn claim_lease_until_ms(&self) -> i64 {
        self.claim_lease_until_ms
    }

    /// Inclusive provider-profile semantic horizon inherited from INT-04.
    pub fn provider_profile_valid_through_ms(&self) -> i64 {
        self.provider_profile_valid_through_ms
    }

    pub const fn current_authority_capability_bound_here(&self) -> bool {
        true
    }

    pub const fn provider_profile_requalified_here(&self) -> bool {
        true
    }

    pub const fn exact_attempt_bound_here(&self) -> bool {
        true
    }

    pub const fn provider_trust_root_origin_verified_here(&self) -> bool {
        false
    }

    /// Provider-profile trust-root currentness is intentionally same-call only.
    /// A later materializer/effect-start boundary must recheck the current root.
    pub const fn reusable_without_provider_root_recheck_here(&self) -> bool {
        false
    }

    pub const fn provider_payload_materialized_here(&self) -> bool {
        false
    }

    pub const fn dispatch_started_here(&self) -> bool {
        false
    }

    pub const fn coordinator_update_excluded_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_preexecution_admission<C>(
    coverage: &MatchedCurrentIntegrationCapabilityCoverage,
    claim: &ExecutionClaim,
    command: &IntegrationCommand<C>,
    provider: &QualifiedProviderExecutionProfile,
    current_trust_root: &ProviderProfileTrustRoot,
    now_ms: i64,
) -> Result<QualifiedIntegrationPreexecutionAdmission, PreexecutionAdmissionError>
where
    C: CanonicalEncodeV1,
{
    let now_u64 = u64::try_from(now_ms).map_err(|_| PreexecutionAdmissionError::InvalidTime)?;
    if now_ms < 0 {
        return Err(PreexecutionAdmissionError::InvalidTime);
    }
    if coverage.verified_at_ms() > now_u64 || coverage.valid_until_ms() <= now_u64 {
        return Err(PreexecutionAdmissionError::CoverageNotLive);
    }

    // Re-run the exact attempt + command + provider-profile + CURRENT trust-root
    // theorem now. A cached QualifiedExecutionBinding is not accepted as input.
    let execution_binding = qualify_execution_binding(
        claim,
        command,
        provider,
        current_trust_root,
        now_ms,
    )?;

    validate_join(
        coverage.current_action().command_commitment(),
        execution_binding.command_commitment(),
        coverage.verified_at_ms(),
        coverage.valid_until_ms(),
        execution_binding.qualified_at_ms(),
        execution_binding.lease_until_ms(),
        provider.profile().not_after_ms,
        now_ms,
    )?;

    let admission_digest = admission_digest(
        coverage.qualification_digest(),
        execution_binding.binding_commitment(),
        execution_binding.provider_profile_commitment(),
        execution_binding.provider_trust_root_commitment(),
        execution_binding.materializer_release(),
    );

    Ok(QualifiedIntegrationPreexecutionAdmission {
        coverage: coverage.clone(),
        authority_valid_until_ms: coverage.valid_until_ms(),
        claim_lease_until_ms: execution_binding.lease_until_ms(),
        provider_profile_valid_through_ms: provider.profile().not_after_ms,
        qualified_at_ms: now_ms,
        execution_binding,
        admission_digest,
    })
}

#[allow(clippy::too_many_arguments)]
fn validate_join(
    authority_command: &ContentCommitment,
    execution_command: &ContentCommitment,
    authority_verified_at_ms: u64,
    authority_valid_until_ms: u64,
    binding_qualified_at_ms: i64,
    claim_lease_until_ms: i64,
    provider_profile_valid_through_ms: i64,
    now_ms: i64,
) -> Result<(), PreexecutionAdmissionError> {
    if now_ms < 0 {
        return Err(PreexecutionAdmissionError::InvalidTime);
    }
    let now_u64 = u64::try_from(now_ms).map_err(|_| PreexecutionAdmissionError::InvalidTime)?;
    if authority_command != execution_command {
        return Err(PreexecutionAdmissionError::CommandCommitmentMismatch);
    }
    if authority_verified_at_ms > now_u64 || authority_valid_until_ms <= now_u64 {
        return Err(PreexecutionAdmissionError::CoverageNotLive);
    }
    if binding_qualified_at_ms > now_ms || claim_lease_until_ms <= now_ms {
        return Err(PreexecutionAdmissionError::ExecutionBindingNotLive);
    }
    if provider_profile_valid_through_ms < now_ms {
        return Err(PreexecutionAdmissionError::ProviderProfileNotLive);
    }
    Ok(())
}

fn admission_digest(
    coverage: Digest32,
    binding: &ContentCommitment,
    provider_profile: &ContentCommitment,
    provider_root: &ContentCommitment,
    materializer_release: &ContentCommitment,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_ADMISSION);
    frame(&mut h, ADMISSION_PROFILE.as_bytes());
    frame(&mut h, &coverage.0);
    frame_commitment(&mut h, binding);
    frame_commitment(&mut h, provider_profile);
    frame_commitment(&mut h, provider_root);
    frame_commitment(&mut h, materializer_release);
    Digest32(*h.finalize().as_bytes())
}

fn frame_commitment(h: &mut blake3::Hasher, commitment: &ContentCommitment) {
    frame(
        h,
        &[match commitment.algorithm {
            DigestAlgorithm::Sha256 => 1,
        }],
    );
    frame(h, &commitment.digest);
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum PreexecutionAdmissionError {
    #[error("preexecution qualification time is invalid")]
    InvalidTime,
    #[error("current institutional/capability coverage is not live")]
    CoverageNotLive,
    #[error("authority coverage and provider binding refer to different canonical commands")]
    CommandCommitmentMismatch,
    #[error("same-call INT-04 execution binding is not live")]
    ExecutionBindingNotLive,
    #[error("provider execution profile is no longer semantically current")]
    ProviderProfileNotLive,
    #[error(transparent)]
    ExecutionBinding(#[from] QualificationError),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(byte: u8) -> ContentCommitment {
        ContentCommitment::sha256(&[byte])
    }

    #[test]
    fn exact_live_join_is_accepted() {
        assert!(validate_join(&c(1), &c(1), 10, 30, 11, 25, 24, 20).is_ok());
    }

    #[test]
    fn command_substitution_fails_closed() {
        assert!(matches!(
            validate_join(&c(1), &c(2), 10, 30, 11, 25, 24, 20),
            Err(PreexecutionAdmissionError::CommandCommitmentMismatch)
        ));
    }

    #[test]
    fn expired_authority_coverage_fails_closed() {
        assert!(matches!(
            validate_join(&c(1), &c(1), 10, 20, 11, 25, 24, 20),
            Err(PreexecutionAdmissionError::CoverageNotLive)
        ));
    }

    #[test]
    fn expired_attempt_lease_fails_closed() {
        assert!(matches!(
            validate_join(&c(1), &c(1), 10, 30, 11, 20, 24, 20),
            Err(PreexecutionAdmissionError::ExecutionBindingNotLive)
        ));
    }

    #[test]
    fn expired_provider_profile_fails_closed() {
        assert!(matches!(
            validate_join(&c(1), &c(1), 10, 30, 11, 25, 19, 20),
            Err(PreexecutionAdmissionError::ProviderProfileNotLive)
        ));
    }
}

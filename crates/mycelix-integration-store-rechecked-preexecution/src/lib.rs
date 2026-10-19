//! Same-invocation store-rechecked preexecution admission.
//!
//! This layer deliberately does not accept a cached preexecution admission or a
//! cached current-attempt proof. It re-runs #646's provider/root qualification,
//! then re-reads INT-03's file-backed durable attempt in the same invocation at
//! the same timestamp.
//!
//! The result is still not atomic with `DispatchStarted`, provider payload
//! materialization, coordinator-update exclusion, or any external effect.

use mycelix_institutional_core::Digest32;
use mycelix_integration_core::{
    CanonicalEncodeV1, ConnectorInstanceId, ContentCommitment, DigestAlgorithm,
    ExecutionAttemptId, IntegrationCommand, IntegrationCommandId,
};
use mycelix_integration_current_capability_coverage::MatchedCurrentIntegrationCapabilityCoverage;
use mycelix_integration_execution_binding::{
    ProviderProfileTrustRoot, QualifiedExecutionBinding, QualifiedProviderExecutionProfile,
};
use mycelix_integration_preexecution_admission::{
    qualify_preexecution_admission, PreexecutionAdmissionError,
    QualifiedIntegrationPreexecutionAdmission,
};
use mycelix_integration_runtime::{
    CurrentAttemptQualificationError, ExecutionClaim, QualifiedCurrentExecutionAttempt,
    SqliteIntegrationStore,
};
use thiserror::Error;

pub const QUALIFICATION_PROFILE: &str =
    "mycelix-integration-store-rechecked-preexecution-v1-blake3-framed";
const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/integration/store-rechecked-preexecution/v1";

/// Non-deserializable evidence that the provider/root admission and the exact
/// file-backed `AttemptPrepared` row were requalified in one invocation at one
/// timestamp.
#[derive(Clone, Debug)]
pub struct QualifiedStoreRecheckedPreexecutionAdmission {
    admission: QualifiedIntegrationPreexecutionAdmission,
    current_attempt: QualifiedCurrentExecutionAttempt,
    qualification_digest: Digest32,
    qualified_at_ms: i64,
}

impl QualifiedStoreRecheckedPreexecutionAdmission {
    pub fn admission(&self) -> &QualifiedIntegrationPreexecutionAdmission {
        &self.admission
    }

    pub fn current_attempt(&self) -> &QualifiedCurrentExecutionAttempt {
        &self.current_attempt
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &'static str {
        QUALIFICATION_PROFILE
    }

    pub fn qualified_at_ms(&self) -> i64 {
        self.qualified_at_ms
    }

    pub const fn provider_profile_and_root_requalified_here(&self) -> bool {
        true
    }

    pub const fn durable_attempt_requalified_here(&self) -> bool {
        true
    }

    pub const fn same_invocation_requalification_here(&self) -> bool {
        true
    }

    /// The supplied trust root was checked as the current root for the call, but
    /// this theorem does not authenticate the root's origin.
    pub const fn provider_trust_root_origin_verified_here(&self) -> bool {
        false
    }

    /// The file-backed read transaction ends before this object is returned.
    pub const fn atomic_with_dispatch_started_here(&self) -> bool {
        false
    }

    pub const fn coordinator_update_excluded_here(&self) -> bool {
        false
    }

    pub const fn provider_payload_materialized_here(&self) -> bool {
        false
    }

    pub const fn reusable_without_final_recheck_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[allow(clippy::too_many_arguments)]
pub fn qualify_store_rechecked_preexecution<C>(
    store: &SqliteIntegrationStore,
    coverage: &MatchedCurrentIntegrationCapabilityCoverage,
    claim: &ExecutionClaim,
    worker_id: &str,
    command: &IntegrationCommand<C>,
    provider: &QualifiedProviderExecutionProfile,
    current_trust_root: &ProviderProfileTrustRoot,
    now_ms: i64,
) -> Result<QualifiedStoreRecheckedPreexecutionAdmission, StoreRecheckedPreexecutionError>
where
    C: CanonicalEncodeV1,
{
    if now_ms < 0 {
        return Err(StoreRecheckedPreexecutionError::InvalidTime);
    }

    // Re-run provider profile + current trust-root compatibility now. A cached
    // QualifiedIntegrationPreexecutionAdmission is intentionally not accepted.
    let admission = qualify_preexecution_admission(
        coverage,
        claim,
        command,
        provider,
        current_trust_root,
        now_ms,
    )?;

    // Re-read the durable AttemptPrepared row after the provider/root theorem in
    // this same invocation. This is the last durable attempt observation made by
    // this function, but it still does not survive return as an atomic lock.
    let current_attempt = store.qualify_current_execution_attempt(claim, worker_id, now_ms)?;

    validate_exact_join(
        admission.execution_binding(),
        current_attempt.claim(),
        admission.qualified_at_ms(),
        current_attempt.qualified_at_ms(),
        admission.authority_valid_until_ms(),
        admission.provider_profile_valid_through_ms(),
        now_ms,
    )?;

    let qualification_digest = qualification_digest(&admission, &current_attempt);
    Ok(QualifiedStoreRecheckedPreexecutionAdmission {
        admission,
        current_attempt,
        qualification_digest,
        qualified_at_ms: now_ms,
    })
}

#[allow(clippy::too_many_arguments)]
fn validate_exact_join(
    binding: &QualifiedExecutionBinding,
    current_claim: &ExecutionClaim,
    admission_qualified_at_ms: i64,
    attempt_qualified_at_ms: i64,
    authority_valid_until_ms: u64,
    provider_profile_valid_through_ms: i64,
    now_ms: i64,
) -> Result<(), StoreRecheckedPreexecutionError> {
    validate_join_facts(
        binding.entry_id(),
        current_claim.entry_id,
        binding.attempt_id(),
        &current_claim.attempt_id,
        binding.command_id(),
        &current_claim.command_id,
        binding.connector_instance(),
        &current_claim.connector_instance,
        binding.command_commitment(),
        &current_claim.command_commitment,
        binding.lease_until_ms(),
        current_claim.lease_until_ms,
        admission_qualified_at_ms,
        attempt_qualified_at_ms,
        authority_valid_until_ms,
        provider_profile_valid_through_ms,
        now_ms,
    )
}

#[allow(clippy::too_many_arguments)]
fn validate_join_facts(
    binding_entry_id: i64,
    attempt_entry_id: i64,
    binding_attempt_id: &ExecutionAttemptId,
    attempt_id: &ExecutionAttemptId,
    binding_command_id: &IntegrationCommandId,
    attempt_command_id: &IntegrationCommandId,
    binding_connector: &ConnectorInstanceId,
    attempt_connector: &ConnectorInstanceId,
    binding_command: &ContentCommitment,
    attempt_command: &ContentCommitment,
    binding_lease_until_ms: i64,
    attempt_lease_until_ms: i64,
    admission_qualified_at_ms: i64,
    attempt_qualified_at_ms: i64,
    authority_valid_until_ms: u64,
    provider_profile_valid_through_ms: i64,
    now_ms: i64,
) -> Result<(), StoreRecheckedPreexecutionError> {
    if now_ms < 0 {
        return Err(StoreRecheckedPreexecutionError::InvalidTime);
    }
    if binding_entry_id != attempt_entry_id {
        return Err(StoreRecheckedPreexecutionError::EntryMismatch);
    }
    if binding_attempt_id != attempt_id {
        return Err(StoreRecheckedPreexecutionError::AttemptMismatch);
    }
    if binding_command_id != attempt_command_id {
        return Err(StoreRecheckedPreexecutionError::CommandIdMismatch);
    }
    if binding_connector != attempt_connector {
        return Err(StoreRecheckedPreexecutionError::ConnectorMismatch);
    }
    if binding_command != attempt_command {
        return Err(StoreRecheckedPreexecutionError::CommandCommitmentMismatch);
    }
    if binding_lease_until_ms != attempt_lease_until_ms {
        return Err(StoreRecheckedPreexecutionError::LeaseMismatch);
    }
    if admission_qualified_at_ms != now_ms || attempt_qualified_at_ms != now_ms {
        return Err(StoreRecheckedPreexecutionError::ObservationTimeMismatch);
    }
    let now_u64 = u64::try_from(now_ms).map_err(|_| StoreRecheckedPreexecutionError::InvalidTime)?;
    if authority_valid_until_ms <= now_u64 {
        return Err(StoreRecheckedPreexecutionError::AuthorityNotLive);
    }
    if provider_profile_valid_through_ms < now_ms {
        return Err(StoreRecheckedPreexecutionError::ProviderProfileNotLive);
    }
    if attempt_lease_until_ms <= now_ms {
        return Err(StoreRecheckedPreexecutionError::AttemptNotLive);
    }
    Ok(())
}

fn qualification_digest(
    admission: &QualifiedIntegrationPreexecutionAdmission,
    current_attempt: &QualifiedCurrentExecutionAttempt,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_QUALIFICATION);
    frame(&mut h, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut h, &admission.admission_digest().0);
    frame_commitment(&mut h, admission.execution_binding().binding_commitment());
    frame(&mut h, current_attempt.qualification_profile().as_bytes());
    frame(&mut h, &current_attempt.store_state_digest());
    frame(&mut h, &current_attempt.store_device().to_le_bytes());
    frame(&mut h, &current_attempt.store_inode().to_le_bytes());
    frame(
        &mut h,
        &current_attempt.durable_updated_at_ms().to_le_bytes(),
    );
    frame(&mut h, &current_attempt.qualified_at_ms().to_le_bytes());
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
pub enum StoreRecheckedPreexecutionError {
    #[error("store-rechecked preexecution time is invalid")]
    InvalidTime,
    #[error("provider/attempt binding and durable attempt refer to different outbox entries")]
    EntryMismatch,
    #[error("provider/attempt binding and durable attempt use different attempt fences")]
    AttemptMismatch,
    #[error("provider/attempt binding and durable attempt use different command ids")]
    CommandIdMismatch,
    #[error("provider/attempt binding and durable attempt use different connectors")]
    ConnectorMismatch,
    #[error("provider/attempt binding and durable attempt use different command commitments")]
    CommandCommitmentMismatch,
    #[error("provider/attempt binding and durable attempt use different lease horizons")]
    LeaseMismatch,
    #[error("provider/root admission and durable attempt were not requalified at the same requested instant")]
    ObservationTimeMismatch,
    #[error("institutional authority/capability coverage expired before the store recheck")]
    AuthorityNotLive,
    #[error("provider execution profile expired before the store recheck")]
    ProviderProfileNotLive,
    #[error("durable attempt lease expired before the store recheck")]
    AttemptNotLive,
    #[error(transparent)]
    Preexecution(#[from] PreexecutionAdmissionError),
    #[error(transparent)]
    CurrentAttempt(#[from] CurrentAttemptQualificationError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_integration_core::ValidationError;

    fn iid<T>(value: &str, constructor: fn(String) -> Result<T, ValidationError>) -> T {
        constructor(value.to_owned()).unwrap()
    }

    fn c(byte: u8) -> ContentCommitment {
        ContentCommitment::sha256(&[byte])
    }

    #[test]
    fn exact_same_instant_facts_join() {
        let attempt = iid("1:1", ExecutionAttemptId::new);
        let command = iid("cmd-1", IntegrationCommandId::new);
        let connector = iid("provider-prod-1", ConnectorInstanceId::new);
        assert!(validate_join_facts(
            1,
            1,
            &attempt,
            &attempt,
            &command,
            &command,
            &connector,
            &connector,
            &c(1),
            &c(1),
            2_000,
            2_000,
            1_000,
            1_000,
            3_000,
            2_500,
            1_000,
        )
        .is_ok());
    }

    #[test]
    fn attempt_substitution_fails_closed() {
        let binding_attempt = iid("1:1", ExecutionAttemptId::new);
        let other_attempt = iid("1:2", ExecutionAttemptId::new);
        let command = iid("cmd-1", IntegrationCommandId::new);
        let connector = iid("provider-prod-1", ConnectorInstanceId::new);
        assert!(matches!(
            validate_join_facts(
                1,
                1,
                &binding_attempt,
                &other_attempt,
                &command,
                &command,
                &connector,
                &connector,
                &c(1),
                &c(1),
                2_000,
                2_000,
                1_000,
                1_000,
                3_000,
                2_500,
                1_000,
            ),
            Err(StoreRecheckedPreexecutionError::AttemptMismatch)
        ));
    }

    #[test]
    fn command_substitution_fails_closed() {
        let attempt = iid("1:1", ExecutionAttemptId::new);
        let command = iid("cmd-1", IntegrationCommandId::new);
        let other_command = iid("cmd-2", IntegrationCommandId::new);
        let connector = iid("provider-prod-1", ConnectorInstanceId::new);
        assert!(matches!(
            validate_join_facts(
                1,
                1,
                &attempt,
                &attempt,
                &command,
                &other_command,
                &connector,
                &connector,
                &c(1),
                &c(1),
                2_000,
                2_000,
                1_000,
                1_000,
                3_000,
                2_500,
                1_000,
            ),
            Err(StoreRecheckedPreexecutionError::CommandIdMismatch)
        ));
    }

    #[test]
    fn stale_observation_time_fails_closed() {
        let attempt = iid("1:1", ExecutionAttemptId::new);
        let command = iid("cmd-1", IntegrationCommandId::new);
        let connector = iid("provider-prod-1", ConnectorInstanceId::new);
        assert!(matches!(
            validate_join_facts(
                1,
                1,
                &attempt,
                &attempt,
                &command,
                &command,
                &connector,
                &connector,
                &c(1),
                &c(1),
                2_000,
                2_000,
                999,
                1_000,
                3_000,
                2_500,
                1_000,
            ),
            Err(StoreRecheckedPreexecutionError::ObservationTimeMismatch)
        ));
    }

    #[test]
    fn expired_attempt_fails_closed() {
        let attempt = iid("1:1", ExecutionAttemptId::new);
        let command = iid("cmd-1", IntegrationCommandId::new);
        let connector = iid("provider-prod-1", ConnectorInstanceId::new);
        assert!(matches!(
            validate_join_facts(
                1,
                1,
                &attempt,
                &attempt,
                &command,
                &command,
                &connector,
                &connector,
                &c(1),
                &c(1),
                1_000,
                1_000,
                1_000,
                1_000,
                3_000,
                2_500,
                1_000,
            ),
            Err(StoreRecheckedPreexecutionError::AttemptNotLive)
        ));
    }
}

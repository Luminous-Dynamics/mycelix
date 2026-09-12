//! Final pre-provider transport-admission composition for Mycelix integrations.
//!
//! This crate performs no provider I/O. It owns the live shared coordinator-
//! mutation exclusion guard while composing current provider-root policy,
//! live authority/capability coverage, exact store state, deterministic payload
//! materialization and atomic durable `DispatchStarted` into one non-forgeable
//! short-lived token.

use mycelix_authority_coordinator_admin_isolation::BrokerProcessBinding;
use mycelix_authority_coordinator_admin_mutation_exclusion::AdminMutationExclusionBinding;
use mycelix_authority_coordinator_conductor_process::TrustedConductorProcessStore;
use mycelix_authority_freshness::VerifiedAuthorityFreshness;
use mycelix_institutional_core::Digest32;
use mycelix_integration_admin_mutation_exclusion::{
    begin_integration_admin_mutation_exclusion, IntegrationAdminMutationExclusionGuard,
    IntegrationExclusionError,
};
use mycelix_integration_atomic_dispatch_start::{
    bind_materialization_and_start_dispatch, AtomicDispatchStartError,
    QualifiedAtomicDispatchStart,
};
use mycelix_integration_core::{CanonicalEncodeV1, IntegrationCommand};
use mycelix_integration_current_capability_coverage::MatchedCurrentIntegrationCapabilityCoverage;
use mycelix_integration_execution_binding::QualifiedProviderExecutionProfile;
use mycelix_integration_materializer_determinism::QualifiedDeterministicWasmMaterializer;
use mycelix_integration_materializer_runtime::{
    materialize_provider_request, MaterializedProviderRequest, MaterializerRuntimeError,
};
use mycelix_integration_provider_trust_currentness::{
    qualify_current_provider_trust_policy, ProviderTrustCurrentnessError,
    QualifiedCurrentProviderTrustPolicy,
};
use mycelix_integration_provider_trust_policy::QualifiedAdoptedProviderTrustPolicy;
use mycelix_integration_runtime::{ExecutionClaim, SqliteIntegrationStore};
use mycelix_integration_store_rechecked_preexecution::{
    qualify_store_rechecked_preexecution, QualifiedStoreRecheckedPreexecutionAdmission,
    StoreRecheckedPreexecutionError,
};
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};
use thiserror::Error;

pub const TRANSPORT_ADMISSION_PROFILE: &str =
    "mycelix-integration-transport-admission-v1-blake3-framed";
const DOMAIN_TRANSPORT_ADMISSION: &[u8] = b"mycelix/integration/transport-admission/v1";

/// Live transport admission. Possession of this process-local object proves the
/// exact shared coordinator-mutation exclusion guard is still owned while the
/// exact payload and exact durable dispatch-start proof remain bound together.
///
/// Future provider transports should accept this type (or an even narrower
/// connector-specific wrapper around it), never a raw ExecutionClaim,
/// preexecution object, materialized bytes, or caller-constructed trust root.
pub struct QualifiedIntegrationTransportAdmission<'a> {
    guard: IntegrationAdminMutationExclusionGuard<'a>,
    current_provider_trust: QualifiedCurrentProviderTrustPolicy,
    final_preexecution: QualifiedStoreRecheckedPreexecutionAdmission,
    materialized: MaterializedProviderRequest,
    atomic_dispatch: QualifiedAtomicDispatchStart,
    transport_digest: Digest32,
    admitted_at_ms: i64,
    transport_deadline_ms: i64,
}

impl QualifiedIntegrationTransportAdmission<'_> {
    pub fn request_bytes(&self) -> &[u8] {
        self.materialized.bytes()
    }

    pub fn materialized(&self) -> &MaterializedProviderRequest {
        &self.materialized
    }

    pub fn atomic_dispatch(&self) -> &QualifiedAtomicDispatchStart {
        &self.atomic_dispatch
    }

    pub fn final_preexecution(&self) -> &QualifiedStoreRecheckedPreexecutionAdmission {
        &self.final_preexecution
    }

    pub fn current_provider_trust(&self) -> &QualifiedCurrentProviderTrustPolicy {
        &self.current_provider_trust
    }

    pub fn transport_digest(&self) -> Digest32 {
        self.transport_digest
    }

    pub fn admitted_at_ms(&self) -> i64 {
        self.admitted_at_ms
    }

    /// Exclusive deadline. Future transport code must refuse issuance when
    /// `now_ms >= transport_deadline_ms`.
    pub fn transport_deadline_ms(&self) -> i64 {
        self.transport_deadline_ms
    }

    pub fn is_live_at(&self, now_ms: i64) -> bool {
        now_ms >= self.admitted_at_ms && now_ms < self.transport_deadline_ms
    }

    pub fn entry_id(&self) -> i64 {
        self.atomic_dispatch.dispatch().entry_id
    }

    pub const fn authority_capability_coverage_live_here(&self) -> bool {
        true
    }

    pub const fn current_adopted_provider_root_requalified_inside_guard_here(&self) -> bool {
        true
    }

    pub const fn durable_attempt_requalified_inside_guard_here(&self) -> bool {
        true
    }

    pub fn shared_coordinator_mutation_exclusion_held_here(&self) -> bool {
        self.guard.shared_admin_mutation_lock_held_here()
            && self.guard.admin_isolation_held_here()
    }

    pub const fn exact_payload_materialized_here(&self) -> bool {
        true
    }

    pub const fn dispatch_started_durably_here(&self) -> bool {
        true
    }

    /// This object is the composed capability a future provider transport may
    /// require. It does not originate institutional authority; it materializes a
    /// short-lived admission from independently qualified authority evidence.
    pub const fn provider_transport_admitted_here(&self) -> bool {
        true
    }

    pub const fn originates_execution_authority_here(&self) -> bool {
        false
    }

    pub const fn provider_call_started_here(&self) -> bool {
        false
    }

    pub const fn provider_commit_confirmed_here(&self) -> bool {
        false
    }
}

#[allow(clippy::too_many_arguments)]
pub fn compose_transport_admission<'a, C>(
    exclusion_binding: &AdminMutationExclusionBinding,
    conductor_store: &'a TrustedConductorProcessStore,
    broker_binding: &BrokerProcessBinding,
    store: &SqliteIntegrationStore,
    store_path: impl AsRef<Path>,
    coverage: &MatchedCurrentIntegrationCapabilityCoverage,
    claim: &ExecutionClaim,
    worker_id: &str,
    command: &IntegrationCommand<C>,
    provider: &QualifiedProviderExecutionProfile,
    adopted_provider_trust: &QualifiedAdoptedProviderTrustPolicy,
    provider_trust_freshness: &VerifiedAuthorityFreshness,
    materializer: &QualifiedDeterministicWasmMaterializer,
    fuel_limit: u64,
) -> Result<QualifiedIntegrationTransportAdmission<'a>, TransportAdmissionError>
where
    C: CanonicalEncodeV1,
{
    validate_provider_authority_domain(coverage, command, provider, adopted_provider_trust)?;

    // Pre-lock qualification exists only to bind #655's live exclusion subject.
    // Nothing effectful can occur from this object alone.
    let precheck_at_ms = system_now_ms()?;
    let precheck_at_u64 = u64::try_from(precheck_at_ms)
        .map_err(|_| TransportAdmissionError::ClockOverflow)?;
    let pre_root = qualify_current_provider_trust_policy(
        adopted_provider_trust,
        provider_trust_freshness,
        precheck_at_u64,
    )?;
    let pre_admission = qualify_store_rechecked_preexecution(
        store,
        coverage,
        claim,
        worker_id,
        command,
        provider,
        pre_root.trust_root(),
        precheck_at_ms,
    )?;

    let guard = begin_integration_admin_mutation_exclusion(
        exclusion_binding,
        conductor_store,
        broker_binding,
        &pre_admission,
    )?;

    // Re-establish provider-root currentness and exact store/provider admission
    // after the shared exclusion interval is actually live.
    let inside_at_ms = system_now_ms()?;
    let inside_at_u64 = u64::try_from(inside_at_ms)
        .map_err(|_| TransportAdmissionError::ClockOverflow)?;
    if inside_at_u64 < guard.started_at_ms() {
        return Err(TransportAdmissionError::ClockRegression);
    }
    let inside_root = qualify_current_provider_trust_policy(
        adopted_provider_trust,
        provider_trust_freshness,
        inside_at_u64,
    )?;
    let inside_admission = qualify_store_rechecked_preexecution(
        store,
        coverage,
        claim,
        worker_id,
        command,
        provider,
        inside_root.trust_root(),
        inside_at_ms,
    )?;
    validate_guard_subject(&guard, &inside_admission)?;

    // The caller cannot substitute arbitrary canonical bytes. Materialization
    // input is generated here from the exact typed command already covered by
    // authority/provider/store qualification.
    let canonical_command = command.canonical_preimage_v1();
    let materialized = materialize_provider_request(
        materializer,
        inside_admission.admission(),
        &canonical_command,
        fuel_limit,
    )?;

    // Materialization may consume enough time for a lease or root policy to
    // expire. Re-run current root + store/provider qualification once more after
    // materialization and immediately before the durable dispatch transition.
    let final_at_ms = system_now_ms()?;
    if final_at_ms < inside_at_ms {
        return Err(TransportAdmissionError::ClockRegression);
    }
    let final_at_u64 = u64::try_from(final_at_ms)
        .map_err(|_| TransportAdmissionError::ClockOverflow)?;
    let final_root = qualify_current_provider_trust_policy(
        adopted_provider_trust,
        provider_trust_freshness,
        final_at_u64,
    )?;
    let final_admission = qualify_store_rechecked_preexecution(
        store,
        coverage,
        claim,
        worker_id,
        command,
        provider,
        final_root.trust_root(),
        final_at_ms,
    )?;
    validate_guard_subject(&guard, &final_admission)?;
    validate_materialization_against_final_admission(&materialized, &final_admission)?;

    let transport_deadline_ms = transport_deadline(
        &final_admission,
        &final_root,
        provider,
        final_at_ms,
    )?;

    // From this call onward a successful return means DispatchStarted is already
    // durable. `bind_materialization_and_start_dispatch` guarantees its own commit
    // is the last fallible operation internally. This function performs only
    // pure/infallible construction after the call returns Ok.
    let atomic_dispatch = bind_materialization_and_start_dispatch(
        final_admission.current_attempt(),
        &materialized,
        store_path,
        final_at_ms,
    )?;

    let transport_digest = transport_digest(
        guard.subject_digest(),
        final_root.qualification_digest(),
        final_admission.qualification_digest(),
        materialized.materialization_digest(),
        atomic_dispatch.binding_digest(),
        final_at_ms,
        transport_deadline_ms,
    );

    Ok(QualifiedIntegrationTransportAdmission {
        guard,
        current_provider_trust: final_root,
        final_preexecution: final_admission,
        materialized,
        atomic_dispatch,
        transport_digest,
        admitted_at_ms: final_at_ms,
        transport_deadline_ms,
    })
}

fn validate_provider_authority_domain<C>(
    coverage: &MatchedCurrentIntegrationCapabilityCoverage,
    command: &IntegrationCommand<C>,
    provider: &QualifiedProviderExecutionProfile,
    adopted_provider_trust: &QualifiedAdoptedProviderTrustPolicy,
) -> Result<(), TransportAdmissionError> {
    let capability_policy = coverage
        .current_mapping()
        .current_policy()
        .adopted_policy()
        .policy();
    let trust_policy = adopted_provider_trust.policy();
    let provider_profile = provider.profile();

    if trust_policy.institution != capability_policy.institution {
        return Err(TransportAdmissionError::ProviderTrustInstitutionMismatch);
    }
    if trust_policy.jurisdiction != capability_policy.jurisdiction {
        return Err(TransportAdmissionError::ProviderTrustJurisdictionMismatch);
    }
    if trust_policy.rulebook != capability_policy.rulebook {
        return Err(TransportAdmissionError::ProviderTrustRulebookMismatch);
    }
    if trust_policy.connector_instance != command.connector_instance
        || trust_policy.connector_instance != provider_profile.connector_instance
    {
        return Err(TransportAdmissionError::ProviderTrustConnectorMismatch);
    }
    if trust_policy.system != command.system || trust_policy.system != provider_profile.system {
        return Err(TransportAdmissionError::ProviderTrustSystemMismatch);
    }
    Ok(())
}

fn validate_guard_subject(
    guard: &IntegrationAdminMutationExclusionGuard<'_>,
    admission: &QualifiedStoreRecheckedPreexecutionAdmission,
) -> Result<(), TransportAdmissionError> {
    let execution = admission.admission().execution_binding();
    if guard.entry_id() != execution.entry_id()
        || guard.attempt_id() != execution.attempt_id()
        || guard.command_commitment() != execution.command_commitment()
    {
        return Err(TransportAdmissionError::ExclusionSubjectMismatch);
    }
    Ok(())
}

fn validate_materialization_against_final_admission(
    materialized: &MaterializedProviderRequest,
    admission: &QualifiedStoreRecheckedPreexecutionAdmission,
) -> Result<(), TransportAdmissionError> {
    let binding = admission.admission().execution_binding();
    if materialized.entry_id() != binding.entry_id()
        || materialized.attempt_id() != binding.attempt_id()
        || materialized.command_id() != binding.command_id()
        || materialized.connector_instance() != binding.connector_instance()
        || materialized.input_commitment() != binding.command_commitment()
        || materialized.provider_profile_commitment() != binding.provider_profile_commitment()
        || materialized.provider_trust_root_commitment()
            != binding.provider_trust_root_commitment()
        || materialized.materializer_release() != binding.materializer_release()
    {
        return Err(TransportAdmissionError::MaterializationFinalAdmissionMismatch);
    }
    Ok(())
}

fn transport_deadline(
    admission: &QualifiedStoreRecheckedPreexecutionAdmission,
    root: &QualifiedCurrentProviderTrustPolicy,
    provider: &QualifiedProviderExecutionProfile,
    now_ms: i64,
) -> Result<i64, TransportAdmissionError> {
    let authority_until = i64::try_from(admission.admission().authority_valid_until_ms())
        .map_err(|_| TransportAdmissionError::DeadlineOverflow)?;
    let root_until = i64::try_from(root.valid_until_ms())
        .map_err(|_| TransportAdmissionError::DeadlineOverflow)?;
    let attempt_until = admission.current_attempt().claim().lease_until_ms;
    let profile_exclusive = provider
        .profile()
        .not_after_ms
        .checked_add(1)
        .unwrap_or(i64::MAX);
    let deadline = authority_until
        .min(root_until)
        .min(attempt_until)
        .min(profile_exclusive);
    if deadline <= now_ms {
        return Err(TransportAdmissionError::NoLiveTransportWindow);
    }
    Ok(deadline)
}

fn transport_digest(
    exclusion_subject: Digest32,
    current_root: Digest32,
    final_preexecution: Digest32,
    materialization: Digest32,
    atomic_dispatch: Digest32,
    admitted_at_ms: i64,
    deadline_ms: i64,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_TRANSPORT_ADMISSION);
    frame(&mut h, TRANSPORT_ADMISSION_PROFILE.as_bytes());
    frame(&mut h, &exclusion_subject.0);
    frame(&mut h, &current_root.0);
    frame(&mut h, &final_preexecution.0);
    frame(&mut h, &materialization.0);
    frame(&mut h, &atomic_dispatch.0);
    frame(&mut h, &admitted_at_ms.to_le_bytes());
    frame(&mut h, &deadline_ms.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn system_now_ms() -> Result<i64, TransportAdmissionError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| TransportAdmissionError::ClockBeforeUnixEpoch)?;
    i64::try_from(duration.as_millis()).map_err(|_| TransportAdmissionError::ClockOverflow)
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum TransportAdmissionError {
    #[error("system clock is before Unix epoch")]
    ClockBeforeUnixEpoch,
    #[error("system clock does not fit signed milliseconds")]
    ClockOverflow,
    #[error("system clock moved backwards during transport admission")]
    ClockRegression,
    #[error("provider trust policy institution differs from capability authority")]
    ProviderTrustInstitutionMismatch,
    #[error("provider trust policy jurisdiction differs from capability authority")]
    ProviderTrustJurisdictionMismatch,
    #[error("provider trust policy rulebook differs from capability authority")]
    ProviderTrustRulebookMismatch,
    #[error("provider trust policy connector differs from command/provider profile")]
    ProviderTrustConnectorMismatch,
    #[error("provider trust policy system differs from command/provider profile")]
    ProviderTrustSystemMismatch,
    #[error("live exclusion subject differs from requalified execution subject")]
    ExclusionSubjectMismatch,
    #[error("materialized request differs from final provider/store admission")]
    MaterializationFinalAdmissionMismatch,
    #[error("transport admission deadline overflows signed milliseconds")]
    DeadlineOverflow,
    #[error("no live transport window remains")]
    NoLiveTransportWindow,
    #[error(transparent)]
    ProviderTrust(#[from] ProviderTrustCurrentnessError),
    #[error(transparent)]
    Preexecution(#[from] StoreRecheckedPreexecutionError),
    #[error(transparent)]
    Exclusion(#[from] IntegrationExclusionError),
    #[error(transparent)]
    Materializer(#[from] MaterializerRuntimeError),
    #[error(transparent)]
    AtomicDispatch(#[from] AtomicDispatchStartError),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transport_digest_changes_with_dispatch_binding() {
        let a = transport_digest(
            Digest32([1; 32]),
            Digest32([2; 32]),
            Digest32([3; 32]),
            Digest32([4; 32]),
            Digest32([5; 32]),
            10,
            20,
        );
        let b = transport_digest(
            Digest32([1; 32]),
            Digest32([2; 32]),
            Digest32([3; 32]),
            Digest32([4; 32]),
            Digest32([6; 32]),
            10,
            20,
        );
        assert_ne!(a, b);
    }
}

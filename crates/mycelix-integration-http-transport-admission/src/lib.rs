//! Final HTTP-specific pre-I/O admission.
//!
//! This theorem composes current institutional authority, current provider-root
//! trust, current HTTP effect-safety policy, a canonical path/query/body plan,
//! the shared coordinator-mutation exclusion interval, and context-bound atomic
//! `DispatchStarted` into one opaque live token.
//!
//! No network I/O occurs here. The canonical materializer output and qualified
//! HTTP plan remain private fields so a future one-shot native HTTPS module can
//! live in this same crate without adding a public `send(bytes)` escape hatch.

use mycelix_authority_coordinator_admin_isolation::BrokerProcessBinding;
use mycelix_authority_coordinator_admin_mutation_exclusion::AdminMutationExclusionBinding;
use mycelix_authority_coordinator_conductor_process::TrustedConductorProcessStore;
use mycelix_authority_freshness::VerifiedAuthorityFreshness;
use mycelix_institutional_core::Digest32;
use mycelix_integration_admin_mutation_exclusion::{
    begin_integration_admin_mutation_exclusion, IntegrationAdminMutationExclusionGuard,
    IntegrationExclusionError,
};
use mycelix_integration_atomic_dispatch_context::{
    bind_context_and_start_dispatch, ContextDispatchError, DispatchContextCommitment,
    QualifiedContextBoundDispatchStart,
};
use mycelix_integration_core::{CanonicalEncodeV1, ContentCommitment, IntegrationCommand};
use mycelix_integration_current_capability_coverage::MatchedCurrentIntegrationCapabilityCoverage;
use mycelix_integration_execution_binding::QualifiedProviderExecutionProfile;
use mycelix_integration_http_request_plan::{
    qualify_http_request_plan, HttpRequestPlanError, QualifiedHttpRequestPlan,
};
use mycelix_integration_http_transport_policy::{
    bind_http_transport_to_provider, qualify_current_http_transport_policy,
    HttpTransportPolicyError, QualifiedAdoptedHttpTransportPolicy,
    QualifiedHttpTransportProviderBinding,
};
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

pub const HTTP_DISPATCH_CONTEXT_PROFILE: &str =
    "mycelix-integration-http-dispatch-context-v1-blake3-framed";
pub const HTTP_TRANSPORT_ADMISSION_PROFILE: &str =
    "mycelix-integration-http-transport-admission-v1-blake3-framed";
const DOMAIN_HTTP_CONTEXT: &[u8] = b"mycelix/integration/http-dispatch-context/v1";
const DOMAIN_HTTP_ADMISSION: &[u8] = b"mycelix/integration/http-transport-admission/v1";

/// Opaque live capability for one exact HTTP transport attempt.
///
/// The raw materializer output and qualified plan are private. There is no public
/// getter returning request bytes, request body, endpoint, credential slot, or
/// a reusable transport object.
pub struct QualifiedHttpTransportAdmission<'a> {
    guard: IntegrationAdminMutationExclusionGuard<'a>,
    current_provider_trust: QualifiedCurrentProviderTrustPolicy,
    final_preexecution: QualifiedStoreRecheckedPreexecutionAdmission,
    http_binding: QualifiedHttpTransportProviderBinding,
    request_plan: QualifiedHttpRequestPlan,
    materialized: MaterializedProviderRequest,
    atomic_dispatch: QualifiedContextBoundDispatchStart,
    admission_digest: Digest32,
    admitted_at_ms: i64,
    transport_deadline_ms: i64,
}

impl QualifiedHttpTransportAdmission<'_> {
    pub fn admission_digest(&self) -> Digest32 {
        self.admission_digest
    }

    pub fn admitted_at_ms(&self) -> i64 {
        self.admitted_at_ms
    }

    pub fn transport_deadline_ms(&self) -> i64 {
        self.transport_deadline_ms
    }

    pub fn is_live_at(&self, now_ms: i64) -> bool {
        now_ms >= self.admitted_at_ms && now_ms < self.transport_deadline_ms
    }

    pub fn entry_id(&self) -> i64 {
        self.atomic_dispatch.dispatch().entry_id
    }

    pub fn output_commitment(&self) -> &ContentCommitment {
        self.materialized.output_commitment()
    }

    pub fn request_plan_commitment(&self) -> Digest32 {
        self.request_plan.qualification_digest()
    }

    pub fn http_transport_policy_binding_digest(&self) -> Digest32 {
        self.http_binding.binding_digest()
    }

    pub fn context_bound_dispatch_digest(&self) -> Digest32 {
        self.atomic_dispatch.binding_digest()
    }

    pub fn current_provider_root_digest(&self) -> Digest32 {
        self.current_provider_trust.qualification_digest()
    }

    pub fn final_store_preexecution_digest(&self) -> Digest32 {
        self.final_preexecution.qualification_digest()
    }

    pub fn shared_coordinator_mutation_exclusion_held_here(&self) -> bool {
        self.guard.shared_admin_mutation_lock_held_here()
            && self.guard.admin_isolation_held_here()
    }

    pub const fn current_http_effect_safety_bound_here(&self) -> bool {
        true
    }

    pub const fn canonical_request_plan_bound_here(&self) -> bool {
        true
    }

    pub const fn transport_context_durable_with_dispatch_here(&self) -> bool {
        true
    }

    pub const fn raw_payload_extractable_here(&self) -> bool {
        false
    }

    pub const fn raw_request_plan_extractable_here(&self) -> bool {
        false
    }

    pub const fn native_http_request_started_here(&self) -> bool {
        false
    }

    pub const fn provider_business_outcome_verified_here(&self) -> bool {
        false
    }

    pub const fn originates_execution_authority_here(&self) -> bool {
        false
    }
}

#[allow(clippy::too_many_arguments)]
pub fn compose_http_transport_admission<'a, C>(
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
    adopted_http_policy: &QualifiedAdoptedHttpTransportPolicy,
    http_policy_freshness: &VerifiedAuthorityFreshness,
    materializer: &QualifiedDeterministicWasmMaterializer,
    fuel_limit: u64,
) -> Result<QualifiedHttpTransportAdmission<'a>, HttpTransportAdmissionError>
where
    C: CanonicalEncodeV1,
{
    validate_authority_domains(
        coverage,
        command,
        provider,
        adopted_provider_trust,
        adopted_http_policy,
    )?;

    // Pre-lock qualification only establishes the exact #655 exclusion subject
    // and rejects stale deployment policy early. It is not reusable for I/O.
    let pre_at_ms = system_now_ms()?;
    let pre_at_u64 = to_u64_time(pre_at_ms)?;
    let pre_root = qualify_current_provider_trust_policy(
        adopted_provider_trust,
        provider_trust_freshness,
        pre_at_u64,
    )?;
    let pre_http = qualify_current_http_transport_policy(
        adopted_http_policy,
        http_policy_freshness,
        pre_at_u64,
    )?;
    let _pre_http_binding = bind_http_transport_to_provider(&pre_http, provider, pre_at_u64)?;
    let pre_admission = qualify_store_rechecked_preexecution(
        store,
        coverage,
        claim,
        worker_id,
        command,
        provider,
        pre_root.trust_root(),
        pre_at_ms,
    )?;

    let guard = begin_integration_admin_mutation_exclusion(
        exclusion_binding,
        conductor_store,
        broker_binding,
        &pre_admission,
    )?;

    // Re-establish every deployment/currentness boundary after the exclusion
    // interval is live.
    let inside_at_ms = system_now_ms()?;
    let inside_at_u64 = to_u64_time(inside_at_ms)?;
    if inside_at_u64 < guard.started_at_ms() {
        return Err(HttpTransportAdmissionError::ClockRegression);
    }
    let inside_root = qualify_current_provider_trust_policy(
        adopted_provider_trust,
        provider_trust_freshness,
        inside_at_u64,
    )?;
    let inside_http = qualify_current_http_transport_policy(
        adopted_http_policy,
        http_policy_freshness,
        inside_at_u64,
    )?;
    let inside_http_binding =
        bind_http_transport_to_provider(&inside_http, provider, inside_at_u64)?;
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

    // The materializer sees only the exact typed-command canonical preimage. Its
    // output must parse as #684's canonical path/query/body request plan.
    let canonical_command = command.canonical_preimage_v1();
    let materialized = materialize_provider_request(
        materializer,
        inside_admission.admission(),
        &canonical_command,
        fuel_limit,
    )?;
    let _inside_plan = qualify_http_request_plan(
        &materialized,
        &inside_http_binding,
        inside_at_u64,
    )?;

    // Materialization may take enough time to cross an authority/root/policy/
    // attempt horizon. Requalify all of them again immediately before dispatch.
    let final_at_ms = system_now_ms()?;
    if final_at_ms < inside_at_ms {
        return Err(HttpTransportAdmissionError::ClockRegression);
    }
    let final_at_u64 = to_u64_time(final_at_ms)?;
    let final_root = qualify_current_provider_trust_policy(
        adopted_provider_trust,
        provider_trust_freshness,
        final_at_u64,
    )?;
    let final_http = qualify_current_http_transport_policy(
        adopted_http_policy,
        http_policy_freshness,
        final_at_u64,
    )?;
    if final_http.verified_at_ms() > final_at_u64 {
        return Err(HttpTransportAdmissionError::HttpPolicyObservationFromFuture);
    }
    let final_http_binding =
        bind_http_transport_to_provider(&final_http, provider, final_at_u64)?;
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
    let final_plan = qualify_http_request_plan(
        &materialized,
        &final_http_binding,
        final_at_u64,
    )?;

    let context = http_dispatch_context(&final_http_binding, &final_plan)?;
    let transport_deadline_ms = transport_deadline(
        &final_admission,
        &final_root,
        &final_http_binding,
        provider,
        final_at_ms,
    )?;

    // From this successful call onward DispatchStarted is durable with the exact
    // HTTP transport-policy + request-plan context. Everything after it is pure
    // and infallible construction.
    let atomic_dispatch = bind_context_and_start_dispatch(
        final_admission.current_attempt(),
        &materialized,
        &context,
        store_path,
        final_at_ms,
    )?;

    let admission_digest = http_transport_admission_digest(
        guard.subject_digest(),
        final_root.qualification_digest(),
        final_admission.qualification_digest(),
        final_http_binding.binding_digest(),
        final_plan.qualification_digest(),
        materialized.materialization_digest(),
        atomic_dispatch.binding_digest(),
        final_at_ms,
        transport_deadline_ms,
    );

    Ok(QualifiedHttpTransportAdmission {
        guard,
        current_provider_trust: final_root,
        final_preexecution: final_admission,
        http_binding: final_http_binding,
        request_plan: final_plan,
        materialized,
        atomic_dispatch,
        admission_digest,
        admitted_at_ms: final_at_ms,
        transport_deadline_ms,
    })
}

fn validate_authority_domains<C>(
    coverage: &MatchedCurrentIntegrationCapabilityCoverage,
    command: &IntegrationCommand<C>,
    provider: &QualifiedProviderExecutionProfile,
    adopted_provider_trust: &QualifiedAdoptedProviderTrustPolicy,
    adopted_http_policy: &QualifiedAdoptedHttpTransportPolicy,
) -> Result<(), HttpTransportAdmissionError> {
    let capability = coverage
        .current_mapping()
        .current_policy()
        .adopted_policy()
        .policy();
    let trust = adopted_provider_trust.policy();
    let http = adopted_http_policy.policy();
    let profile = provider.profile();

    if trust.institution != capability.institution || http.institution != capability.institution {
        return Err(HttpTransportAdmissionError::InstitutionMismatch);
    }
    if trust.jurisdiction != capability.jurisdiction || http.jurisdiction != capability.jurisdiction
    {
        return Err(HttpTransportAdmissionError::JurisdictionMismatch);
    }
    if trust.rulebook != capability.rulebook || http.rulebook != capability.rulebook {
        return Err(HttpTransportAdmissionError::RulebookMismatch);
    }
    if trust.connector_instance != command.connector_instance
        || http.connector_instance != command.connector_instance
        || profile.connector_instance != command.connector_instance
    {
        return Err(HttpTransportAdmissionError::ConnectorMismatch);
    }
    if trust.system != command.system || http.system != command.system || profile.system != command.system
    {
        return Err(HttpTransportAdmissionError::SystemMismatch);
    }
    if http.operation_kind != command.operation_kind || profile.operation_kind != command.operation_kind
    {
        return Err(HttpTransportAdmissionError::OperationMismatch);
    }
    Ok(())
}

fn validate_guard_subject(
    guard: &IntegrationAdminMutationExclusionGuard<'_>,
    admission: &QualifiedStoreRecheckedPreexecutionAdmission,
) -> Result<(), HttpTransportAdmissionError> {
    let execution = admission.admission().execution_binding();
    if guard.entry_id() != execution.entry_id()
        || guard.attempt_id() != execution.attempt_id()
        || guard.command_commitment() != execution.command_commitment()
    {
        return Err(HttpTransportAdmissionError::ExclusionSubjectMismatch);
    }
    Ok(())
}

fn validate_materialization_against_final_admission(
    materialized: &MaterializedProviderRequest,
    admission: &QualifiedStoreRecheckedPreexecutionAdmission,
) -> Result<(), HttpTransportAdmissionError> {
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
        return Err(HttpTransportAdmissionError::MaterializationFinalAdmissionMismatch);
    }
    Ok(())
}

fn http_dispatch_context(
    binding: &QualifiedHttpTransportProviderBinding,
    plan: &QualifiedHttpRequestPlan,
) -> Result<DispatchContextCommitment, HttpTransportAdmissionError> {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_HTTP_CONTEXT);
    frame(&mut h, HTTP_DISPATCH_CONTEXT_PROFILE.as_bytes());
    frame(&mut h, &binding.binding_digest().0);
    frame(&mut h, &plan.qualification_digest().0);
    let digest = Digest32(*h.finalize().as_bytes());
    DispatchContextCommitment::new(HTTP_DISPATCH_CONTEXT_PROFILE, digest)
        .map_err(HttpTransportAdmissionError::ContextDispatch)
}

fn transport_deadline(
    admission: &QualifiedStoreRecheckedPreexecutionAdmission,
    root: &QualifiedCurrentProviderTrustPolicy,
    http: &QualifiedHttpTransportProviderBinding,
    provider: &QualifiedProviderExecutionProfile,
    now_ms: i64,
) -> Result<i64, HttpTransportAdmissionError> {
    let authority_until = i64::try_from(admission.admission().authority_valid_until_ms())
        .map_err(|_| HttpTransportAdmissionError::DeadlineOverflow)?;
    let root_until = i64::try_from(root.valid_until_ms())
        .map_err(|_| HttpTransportAdmissionError::DeadlineOverflow)?;
    let http_until = i64::try_from(http.valid_until_ms())
        .map_err(|_| HttpTransportAdmissionError::DeadlineOverflow)?;
    let attempt_until = admission.current_attempt().claim().lease_until_ms;
    let profile_exclusive = provider
        .profile()
        .not_after_ms
        .checked_add(1)
        .unwrap_or(i64::MAX);
    let deadline = authority_until
        .min(root_until)
        .min(http_until)
        .min(attempt_until)
        .min(profile_exclusive);
    if deadline <= now_ms {
        return Err(HttpTransportAdmissionError::NoLiveTransportWindow);
    }
    Ok(deadline)
}

#[allow(clippy::too_many_arguments)]
fn http_transport_admission_digest(
    exclusion_subject: Digest32,
    current_root: Digest32,
    final_preexecution: Digest32,
    http_binding: Digest32,
    request_plan: Digest32,
    materialization: Digest32,
    context_dispatch: Digest32,
    admitted_at_ms: i64,
    deadline_ms: i64,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_HTTP_ADMISSION);
    frame(&mut h, HTTP_TRANSPORT_ADMISSION_PROFILE.as_bytes());
    frame(&mut h, &exclusion_subject.0);
    frame(&mut h, &current_root.0);
    frame(&mut h, &final_preexecution.0);
    frame(&mut h, &http_binding.0);
    frame(&mut h, &request_plan.0);
    frame(&mut h, &materialization.0);
    frame(&mut h, &context_dispatch.0);
    frame(&mut h, &admitted_at_ms.to_le_bytes());
    frame(&mut h, &deadline_ms.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn to_u64_time(value: i64) -> Result<u64, HttpTransportAdmissionError> {
    u64::try_from(value).map_err(|_| HttpTransportAdmissionError::ClockOverflow)
}

fn system_now_ms() -> Result<i64, HttpTransportAdmissionError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| HttpTransportAdmissionError::ClockBeforeUnixEpoch)?;
    i64::try_from(duration.as_millis()).map_err(|_| HttpTransportAdmissionError::ClockOverflow)
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum HttpTransportAdmissionError {
    #[error("system clock is before Unix epoch")]
    ClockBeforeUnixEpoch,
    #[error("system clock does not fit transport-admission time representation")]
    ClockOverflow,
    #[error("system clock moved backwards during transport admission")]
    ClockRegression,
    #[error("HTTP policy observation is from the future")]
    HttpPolicyObservationFromFuture,
    #[error("provider trust / HTTP policy institution differs from execution authority")]
    InstitutionMismatch,
    #[error("provider trust / HTTP policy jurisdiction differs from execution authority")]
    JurisdictionMismatch,
    #[error("provider trust / HTTP policy rulebook differs from execution authority")]
    RulebookMismatch,
    #[error("connector differs across command/provider/trust/HTTP policy")]
    ConnectorMismatch,
    #[error("external system differs across command/provider/trust/HTTP policy")]
    SystemMismatch,
    #[error("operation differs across command/provider/HTTP policy")]
    OperationMismatch,
    #[error("live exclusion subject differs from final execution subject")]
    ExclusionSubjectMismatch,
    #[error("materialized request differs from final store/provider admission")]
    MaterializationFinalAdmissionMismatch,
    #[error("no live HTTP transport window remains")]
    NoLiveTransportWindow,
    #[error("transport deadline cannot be represented")]
    DeadlineOverflow,
    #[error(transparent)]
    ProviderTrust(#[from] ProviderTrustCurrentnessError),
    #[error(transparent)]
    HttpPolicy(#[from] HttpTransportPolicyError),
    #[error(transparent)]
    Preexecution(#[from] StoreRecheckedPreexecutionError),
    #[error(transparent)]
    Exclusion(#[from] IntegrationExclusionError),
    #[error(transparent)]
    Materializer(#[from] MaterializerRuntimeError),
    #[error(transparent)]
    RequestPlan(#[from] HttpRequestPlanError),
    #[error(transparent)]
    ContextDispatch(#[from] ContextDispatchError),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn http_context_changes_when_plan_or_policy_changes() {
        fn digest(binding: u8, plan: u8) -> Digest32 {
            let mut h = blake3::Hasher::new();
            h.update(DOMAIN_HTTP_CONTEXT);
            frame(&mut h, HTTP_DISPATCH_CONTEXT_PROFILE.as_bytes());
            frame(&mut h, &Digest32([binding; 32]).0);
            frame(&mut h, &Digest32([plan; 32]).0);
            Digest32(*h.finalize().as_bytes())
        }
        assert_ne!(digest(1, 2), digest(1, 3));
        assert_ne!(digest(1, 2), digest(4, 2));
    }
}

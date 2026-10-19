//! Route + credential-version bound HTTP issuance admission.
//!
//! This theorem strengthens the earlier HTTP transport admission by requiring
//! the exact qualified public route, exact credential-slot version metadata, and
//! secret-free issuance descriptor to exist before the durable
//! `AttemptPrepared -> DispatchStarted` transition commits.
//!
//! It still performs no DNS, secret-manager, TLS, socket, or provider I/O. DNS
//! and credential metadata origin remain explicitly unverified here; the future
//! native boundary must create/authenticate those observations itself.

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
use mycelix_integration_http_credential_slot::{
    qualify_http_credential_slot, CredentialSlotError, CredentialSlotMetadataObservation,
    QualifiedHttpCredentialSlot,
};
use mycelix_integration_http_egress_route::{
    qualify_public_https_route, DnsResolutionObservation, HttpEgressRouteError,
    QualifiedPublicHttpsRoute,
};
use mycelix_integration_http_issuance_descriptor::{
    qualify_http_issuance_descriptor, IssuanceDescriptorError,
    QualifiedHttpIssuanceDescriptor,
};
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

pub const HTTP_ISSUANCE_CONTEXT_PROFILE: &str =
    "mycelix-integration-http-issuance-context-v2-blake3-framed";
pub const HTTP_ISSUANCE_ADMISSION_PROFILE: &str =
    "mycelix-integration-http-issuance-admission-v2-blake3-framed";
const DOMAIN_ISSUANCE_CONTEXT: &[u8] = b"mycelix/integration/http-issuance-context/v2";
const DOMAIN_ISSUANCE_ADMISSION: &[u8] = b"mycelix/integration/http-issuance-admission/v2";

/// Opaque live pre-I/O capability whose durable DispatchStarted context binds
/// the exact HTTP policy, canonical plan, route, credential version metadata,
/// and secret-free issuance descriptor.
pub struct QualifiedHttpIssuanceAdmission<'a> {
    guard: IntegrationAdminMutationExclusionGuard<'a>,
    current_provider_trust: QualifiedCurrentProviderTrustPolicy,
    final_preexecution: QualifiedStoreRecheckedPreexecutionAdmission,
    http_binding: QualifiedHttpTransportProviderBinding,
    request_plan: QualifiedHttpRequestPlan,
    route: QualifiedPublicHttpsRoute,
    credential: QualifiedHttpCredentialSlot,
    descriptor: QualifiedHttpIssuanceDescriptor,
    materialized: MaterializedProviderRequest,
    atomic_dispatch: QualifiedContextBoundDispatchStart,
    admission_digest: Digest32,
    admitted_at_ms: i64,
    issuance_deadline_ms: i64,
}

impl QualifiedHttpIssuanceAdmission<'_> {
    pub fn admission_digest(&self) -> Digest32 {
        self.admission_digest
    }

    pub fn admitted_at_ms(&self) -> i64 {
        self.admitted_at_ms
    }

    pub fn issuance_deadline_ms(&self) -> i64 {
        self.issuance_deadline_ms
    }

    pub fn is_live_at(&self, now_ms: i64) -> bool {
        now_ms >= self.admitted_at_ms && now_ms < self.issuance_deadline_ms
    }

    pub fn entry_id(&self) -> i64 {
        self.atomic_dispatch.dispatch().entry_id
    }

    pub fn output_commitment(&self) -> &ContentCommitment {
        self.materialized.output_commitment()
    }

    pub fn http_transport_policy_binding_digest(&self) -> Digest32 {
        self.http_binding.binding_digest()
    }

    pub fn request_plan_digest(&self) -> Digest32 {
        self.request_plan.qualification_digest()
    }

    pub fn route_digest(&self) -> Digest32 {
        self.route.route_digest()
    }

    pub fn credential_version_digest(&self) -> Digest32 {
        self.credential.qualification_digest()
    }

    pub fn issuance_descriptor_digest(&self) -> Digest32 {
        self.descriptor.issuance_digest()
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

    pub const fn route_bound_durably_with_dispatch_here(&self) -> bool {
        true
    }

    pub const fn credential_version_bound_durably_with_dispatch_here(&self) -> bool {
        true
    }

    pub const fn secret_free_issuance_identity_bound_durably_here(&self) -> bool {
        true
    }

    pub const fn resolver_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn credential_metadata_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn credential_material_present_here(&self) -> bool {
        false
    }

    pub const fn raw_payload_extractable_here(&self) -> bool {
        false
    }

    pub const fn raw_request_plan_extractable_here(&self) -> bool {
        false
    }

    /// This theorem deliberately does not authorize native I/O because the DNS
    /// and credential metadata observations are still caller-shaped inputs.
    pub const fn native_io_authorized_here(&self) -> bool {
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
pub fn compose_http_issuance_admission<'a, C>(
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
    dns_observation: DnsResolutionObservation,
    credential_observation: CredentialSlotMetadataObservation,
) -> Result<QualifiedHttpIssuanceAdmission<'a>, HttpIssuanceAdmissionError>
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

    // Pre-lock evidence is used only to establish the exact shared coordinator
    // mutation exclusion subject and reject obviously stale deployment state.
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

    // Re-establish mutable/current state after the exclusion interval is live.
    let inside_at_ms = system_now_ms()?;
    let inside_at_u64 = to_u64_time(inside_at_ms)?;
    if inside_at_u64 < guard.started_at_ms() {
        return Err(HttpIssuanceAdmissionError::ClockRegression);
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

    let canonical_command = command.canonical_preimage_v1();
    let materialized = materialize_provider_request(
        materializer,
        inside_admission.admission(),
        &canonical_command,
        fuel_limit,
    )?;
    let _inside_plan =
        qualify_http_request_plan(&materialized, &inside_http_binding, inside_at_u64)?;

    // Materialization can consume enough time for authority/root/policy/attempt
    // state to expire. Re-run every mutable gate before deriving the issuance
    // descriptor and before the durable dispatch transition.
    let final_at_ms = system_now_ms()?;
    if final_at_ms < inside_at_ms {
        return Err(HttpIssuanceAdmissionError::ClockRegression);
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
        return Err(HttpIssuanceAdmissionError::HttpPolicyObservationFromFuture);
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

    let final_plan =
        qualify_http_request_plan(&materialized, &final_http_binding, final_at_u64)?;
    let final_route =
        qualify_public_https_route(&final_http_binding, dns_observation, final_at_u64)?;
    let final_credential = qualify_http_credential_slot(
        &final_http_binding,
        credential_observation,
        final_at_u64,
    )?;
    let final_descriptor = qualify_http_issuance_descriptor(
        command,
        &final_http_binding,
        &final_plan,
        &final_route,
        &final_credential,
        final_at_u64,
    )?;
    validate_descriptor_against_final_admission(&final_descriptor, &final_admission)?;

    let context = issuance_dispatch_context(
        &final_http_binding,
        &final_plan,
        &final_route,
        &final_credential,
        &final_descriptor,
        &materialized,
    )?;
    let issuance_deadline_ms = issuance_deadline(
        &final_admission,
        &final_root,
        &final_http_binding,
        &final_route,
        &final_credential,
        &final_descriptor,
        provider,
        final_at_ms,
    )?;

    // FINAL FALLIBLE BOUNDARY. On success, DispatchStarted and the exact HTTP
    // issuance context are durable together. Everything after this call is pure
    // infallible construction.
    let atomic_dispatch = bind_context_and_start_dispatch(
        final_admission.current_attempt(),
        &materialized,
        &context,
        store_path,
        final_at_ms,
    )?;

    let admission_digest = issuance_admission_digest(
        guard.subject_digest(),
        final_root.qualification_digest(),
        final_admission.qualification_digest(),
        final_http_binding.binding_digest(),
        final_plan.qualification_digest(),
        final_route.route_digest(),
        final_credential.qualification_digest(),
        final_descriptor.issuance_digest(),
        materialized.materialization_digest(),
        atomic_dispatch.binding_digest(),
        final_at_ms,
        issuance_deadline_ms,
    );

    Ok(QualifiedHttpIssuanceAdmission {
        guard,
        current_provider_trust: final_root,
        final_preexecution: final_admission,
        http_binding: final_http_binding,
        request_plan: final_plan,
        route: final_route,
        credential: final_credential,
        descriptor: final_descriptor,
        materialized,
        atomic_dispatch,
        admission_digest,
        admitted_at_ms: final_at_ms,
        issuance_deadline_ms,
    })
}

fn validate_authority_domains<C>(
    coverage: &MatchedCurrentIntegrationCapabilityCoverage,
    command: &IntegrationCommand<C>,
    provider: &QualifiedProviderExecutionProfile,
    adopted_provider_trust: &QualifiedAdoptedProviderTrustPolicy,
    adopted_http_policy: &QualifiedAdoptedHttpTransportPolicy,
) -> Result<(), HttpIssuanceAdmissionError> {
    let capability = coverage
        .current_mapping()
        .current_policy()
        .adopted_policy()
        .policy();
    let trust = adopted_provider_trust.policy();
    let http = adopted_http_policy.policy();
    let profile = provider.profile();

    if trust.institution != capability.institution || http.institution != capability.institution {
        return Err(HttpIssuanceAdmissionError::InstitutionMismatch);
    }
    if trust.jurisdiction != capability.jurisdiction || http.jurisdiction != capability.jurisdiction
    {
        return Err(HttpIssuanceAdmissionError::JurisdictionMismatch);
    }
    if trust.rulebook != capability.rulebook || http.rulebook != capability.rulebook {
        return Err(HttpIssuanceAdmissionError::RulebookMismatch);
    }
    if trust.connector_instance != command.connector_instance
        || http.connector_instance != command.connector_instance
        || profile.connector_instance != command.connector_instance
    {
        return Err(HttpIssuanceAdmissionError::ConnectorMismatch);
    }
    if trust.system != command.system || http.system != command.system || profile.system != command.system
    {
        return Err(HttpIssuanceAdmissionError::SystemMismatch);
    }
    if http.operation_kind != command.operation_kind || profile.operation_kind != command.operation_kind
    {
        return Err(HttpIssuanceAdmissionError::OperationMismatch);
    }
    Ok(())
}

fn validate_guard_subject(
    guard: &IntegrationAdminMutationExclusionGuard<'_>,
    admission: &QualifiedStoreRecheckedPreexecutionAdmission,
) -> Result<(), HttpIssuanceAdmissionError> {
    let execution = admission.admission().execution_binding();
    if guard.entry_id() != execution.entry_id()
        || guard.attempt_id() != execution.attempt_id()
        || guard.command_commitment() != execution.command_commitment()
    {
        return Err(HttpIssuanceAdmissionError::ExclusionSubjectMismatch);
    }
    Ok(())
}

fn validate_materialization_against_final_admission(
    materialized: &MaterializedProviderRequest,
    admission: &QualifiedStoreRecheckedPreexecutionAdmission,
) -> Result<(), HttpIssuanceAdmissionError> {
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
        return Err(HttpIssuanceAdmissionError::MaterializationFinalAdmissionMismatch);
    }
    Ok(())
}

fn validate_descriptor_against_final_admission(
    descriptor: &QualifiedHttpIssuanceDescriptor,
    admission: &QualifiedStoreRecheckedPreexecutionAdmission,
) -> Result<(), HttpIssuanceAdmissionError> {
    if descriptor.command_commitment()
        != admission.admission().execution_binding().command_commitment()
    {
        return Err(HttpIssuanceAdmissionError::DescriptorFinalAdmissionMismatch);
    }
    Ok(())
}

fn issuance_dispatch_context(
    binding: &QualifiedHttpTransportProviderBinding,
    plan: &QualifiedHttpRequestPlan,
    route: &QualifiedPublicHttpsRoute,
    credential: &QualifiedHttpCredentialSlot,
    descriptor: &QualifiedHttpIssuanceDescriptor,
    materialized: &MaterializedProviderRequest,
) -> Result<DispatchContextCommitment, HttpIssuanceAdmissionError> {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_ISSUANCE_CONTEXT);
    frame(&mut h, HTTP_ISSUANCE_CONTEXT_PROFILE.as_bytes());
    frame(&mut h, &binding.binding_digest().0);
    frame(&mut h, &plan.qualification_digest().0);
    frame(&mut h, &route.route_digest().0);
    frame(&mut h, &credential.qualification_digest().0);
    frame(&mut h, &descriptor.issuance_digest().0);
    frame(&mut h, &materialized.materialization_digest().0);
    let digest = Digest32(*h.finalize().as_bytes());
    DispatchContextCommitment::new(HTTP_ISSUANCE_CONTEXT_PROFILE, digest)
        .map_err(HttpIssuanceAdmissionError::ContextDispatch)
}

#[allow(clippy::too_many_arguments)]
fn issuance_deadline(
    admission: &QualifiedStoreRecheckedPreexecutionAdmission,
    root: &QualifiedCurrentProviderTrustPolicy,
    http: &QualifiedHttpTransportProviderBinding,
    route: &QualifiedPublicHttpsRoute,
    credential: &QualifiedHttpCredentialSlot,
    descriptor: &QualifiedHttpIssuanceDescriptor,
    provider: &QualifiedProviderExecutionProfile,
    now_ms: i64,
) -> Result<i64, HttpIssuanceAdmissionError> {
    let authority_until = i64::try_from(admission.admission().authority_valid_until_ms())
        .map_err(|_| HttpIssuanceAdmissionError::DeadlineOverflow)?;
    let root_until = i64::try_from(root.valid_until_ms())
        .map_err(|_| HttpIssuanceAdmissionError::DeadlineOverflow)?;
    let http_until = i64::try_from(http.valid_until_ms())
        .map_err(|_| HttpIssuanceAdmissionError::DeadlineOverflow)?;
    let route_until = i64::try_from(route.valid_until_ms())
        .map_err(|_| HttpIssuanceAdmissionError::DeadlineOverflow)?;
    let credential_until = i64::try_from(credential.valid_until_ms())
        .map_err(|_| HttpIssuanceAdmissionError::DeadlineOverflow)?;
    let descriptor_until = i64::try_from(descriptor.valid_until_ms())
        .map_err(|_| HttpIssuanceAdmissionError::DeadlineOverflow)?;
    let attempt_until = admission.current_attempt().claim().lease_until_ms;
    let profile_exclusive = provider
        .profile()
        .not_after_ms
        .checked_add(1)
        .unwrap_or(i64::MAX);

    let deadline = authority_until
        .min(root_until)
        .min(http_until)
        .min(route_until)
        .min(credential_until)
        .min(descriptor_until)
        .min(attempt_until)
        .min(profile_exclusive);
    if deadline <= now_ms {
        return Err(HttpIssuanceAdmissionError::NoLiveIssuanceWindow);
    }
    Ok(deadline)
}

#[allow(clippy::too_many_arguments)]
fn issuance_admission_digest(
    exclusion_subject: Digest32,
    current_root: Digest32,
    final_preexecution: Digest32,
    http_binding: Digest32,
    request_plan: Digest32,
    route: Digest32,
    credential: Digest32,
    descriptor: Digest32,
    materialization: Digest32,
    context_dispatch: Digest32,
    admitted_at_ms: i64,
    deadline_ms: i64,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_ISSUANCE_ADMISSION);
    frame(&mut h, HTTP_ISSUANCE_ADMISSION_PROFILE.as_bytes());
    frame(&mut h, &exclusion_subject.0);
    frame(&mut h, &current_root.0);
    frame(&mut h, &final_preexecution.0);
    frame(&mut h, &http_binding.0);
    frame(&mut h, &request_plan.0);
    frame(&mut h, &route.0);
    frame(&mut h, &credential.0);
    frame(&mut h, &descriptor.0);
    frame(&mut h, &materialization.0);
    frame(&mut h, &context_dispatch.0);
    frame(&mut h, &admitted_at_ms.to_le_bytes());
    frame(&mut h, &deadline_ms.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn to_u64_time(value: i64) -> Result<u64, HttpIssuanceAdmissionError> {
    u64::try_from(value).map_err(|_| HttpIssuanceAdmissionError::ClockOverflow)
}

fn system_now_ms() -> Result<i64, HttpIssuanceAdmissionError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| HttpIssuanceAdmissionError::ClockBeforeUnixEpoch)?;
    i64::try_from(duration.as_millis()).map_err(|_| HttpIssuanceAdmissionError::ClockOverflow)
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum HttpIssuanceAdmissionError {
    #[error("system clock is before Unix epoch")]
    ClockBeforeUnixEpoch,
    #[error("system clock does not fit issuance-admission time representation")]
    ClockOverflow,
    #[error("system clock moved backwards during issuance admission")]
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
    #[error("issuance descriptor command differs from final store/provider admission")]
    DescriptorFinalAdmissionMismatch,
    #[error("no live HTTP issuance window remains")]
    NoLiveIssuanceWindow,
    #[error("issuance deadline cannot be represented")]
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
    Route(#[from] HttpEgressRouteError),
    #[error(transparent)]
    Credential(#[from] CredentialSlotError),
    #[error(transparent)]
    Descriptor(#[from] IssuanceDescriptorError),
    #[error(transparent)]
    ContextDispatch(#[from] ContextDispatchError),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn issuance_context_changes_for_each_http_identity_dimension() {
        fn digest(binding: u8, plan: u8, route: u8, credential: u8, descriptor: u8) -> Digest32 {
            let mut h = blake3::Hasher::new();
            h.update(DOMAIN_ISSUANCE_CONTEXT);
            frame(&mut h, HTTP_ISSUANCE_CONTEXT_PROFILE.as_bytes());
            frame(&mut h, &Digest32([binding; 32]).0);
            frame(&mut h, &Digest32([plan; 32]).0);
            frame(&mut h, &Digest32([route; 32]).0);
            frame(&mut h, &Digest32([credential; 32]).0);
            frame(&mut h, &Digest32([descriptor; 32]).0);
            frame(&mut h, &Digest32([9; 32]).0);
            Digest32(*h.finalize().as_bytes())
        }

        let base = digest(1, 2, 3, 4, 5);
        assert_ne!(base, digest(9, 2, 3, 4, 5));
        assert_ne!(base, digest(1, 9, 3, 4, 5));
        assert_ne!(base, digest(1, 2, 9, 4, 5));
        assert_ne!(base, digest(1, 2, 3, 9, 5));
        assert_ne!(base, digest(1, 2, 3, 4, 9));
    }
}
